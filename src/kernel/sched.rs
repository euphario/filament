//! Scheduler - Internal Kernel Scheduling Mechanisms
//!
//! This module provides the kernel-internal scheduling functions. These are
//! **not** syscall entry points - they are called by syscall handlers and
//! interrupt handlers to manage task execution.
//!
//! # Three-Phase Scheduling (SMP-Safe)
//!
//! The key design constraint is that the scheduler lock MUST NOT be held
//! across context_switch(). This module uses three-phase scheduling:
//!
//! 1. **Phase 1**: Acquire lock, make decision, extract SwitchDecision, RELEASE LOCK
//! 2. **Phase 2**: Context switch with NO LOCK (only IRQs disabled)
//! 3. **Phase 3**: Reacquire lock for finalization
//!
//! # Public API
//!
//! | Function | Purpose |
//! |----------|---------|
//! | [`reschedule()`] | Pick next task and context switch |
//! | [`wake(pid)`] | Wake a sleeping/waiting task by PID |
//! | [`sleep_current(reason)`] | Sleep current task (event loop) |
//! | [`wait_current(reason, deadline)`] | Wait with timeout |
//! | [`yield_current()`] | Voluntarily give up CPU |
//!
//! # Task States
//!
//! - **Ready** → Running (via `reschedule`)
//! - **Running** → Ready (via `yield_current` or preemption)
//! - **Running** → Sleeping (via `sleep_current`) - no deadline
//! - **Running** → Waiting (via `wait_current`) - has deadline
//! - **Sleeping/Waiting** → Ready (via `wake` or timeout)

use crate::{kerror, kdebug, kwarn};
use super::task::{self, TaskState, current_slot, set_current_slot, TrapFrame};
use super::percpu;

/// Slot reserved for idle task on CPU 0 (backward compat)
pub const IDLE_SLOT: usize = 0;

/// Get the idle slot for a given CPU. Slot N = CPU N's idle task.
#[inline]
pub fn idle_slot_for_cpu(cpu: u32) -> usize {
    cpu as usize
}

/// Check if a slot is an idle slot (any CPU's idle).
#[inline]
pub fn is_idle_slot(slot: usize) -> bool {
    slot < percpu::MAX_CPUS
}

// ============================================================================
// SMP-Safe Context Switch Data
// ============================================================================

/// Decision data extracted under lock, used after lock release.
///
/// All pointers are valid because:
/// - from_slot is current task (won't be reaped while we're running)
/// - to_slot is marked Running before lock release (won't be reaped)
struct SwitchDecision {
    /// Slot we're switching from
    from_slot: usize,
    /// Slot we're switching to
    to_slot: usize,
    /// Context pointer for from_task (where to save)
    from_ctx: *mut task::CpuContext,
    /// Context pointer for to_task (what to restore)
    to_ctx: *const task::CpuContext,
    /// Trap frame pointer for target task
    to_trap_frame: *mut TrapFrame,
    /// TTBR0 value for target task's address space (0 if kernel task)
    to_ttbr0: u64,
}

// SAFETY: Pointers are valid for duration of context switch because both tasks
// are pinned (from=Running/Blocked, to=Running). Tasks can't be reaped while
// Running, and from_task is us so it can't be reaped.
unsafe impl Send for SwitchDecision {}

// ============================================================================
// Core Scheduling Operations
// ============================================================================

/// Block reason for atomic sleep+reschedule operations.
enum BlockReason {
    /// No blocking - normal preemptive reschedule
    None,
    /// Sleep (no deadline)
    Sleep(task::SleepReason),
    /// Wait with deadline
    Wait(task::WaitReason, u64),
}

/// Reschedule: pick the next task to run and switch to it.
///
/// Uses three-phase scheduling for SMP safety:
/// 1. Phase 1: Acquire lock, make decision, RELEASE LOCK
/// 2. Phase 2: Context switch with NO LOCK
/// 3. Phase 3: Reacquire lock for finalization
///
/// # Returns
/// true if we switched to a different task, false if staying on current.
pub fn reschedule() -> bool {
    reschedule_inner(BlockReason::None)
}

/// Atomically sleep the current task and reschedule.
///
/// Returns false if called from idle task. Returns true otherwise
/// (task was blocked and has now been woken and rescheduled back).
pub fn sleep_and_reschedule(reason: task::SleepReason) -> bool {
    let slot = current_slot();
    if is_idle_slot(slot) {
        return false;
    }
    reschedule_inner(BlockReason::Sleep(reason));
    true
}

/// Atomically wait the current task and reschedule.
///
/// Same as sleep_and_reschedule but with a deadline.
pub fn wait_and_reschedule(reason: task::WaitReason, deadline: u64) -> bool {
    let slot = current_slot();
    if is_idle_slot(slot) {
        return false;
    }
    reschedule_inner(BlockReason::Wait(reason, deadline));
    true
}

fn reschedule_inner(block: BlockReason) -> bool {
    // ========================================================================
    // PHASE 1: Decision under lock
    // ========================================================================
    let decision: Option<SwitchDecision> = {
        let mut sched = task::scheduler();
        let caller_slot = current_slot();

        // If caller wants to block, do the state transition NOW under lock.
        // This is atomic with the scheduling decision — no other CPU can see
        // this task as Ready between the block and the context switch setup.
        let force_full_switch = match block {
            BlockReason::None => false,
            BlockReason::Sleep(reason) => {
                if is_idle_slot(caller_slot) {
                    kerror!("sched", "block_idle_attempt");
                    return false;
                }
                if let Some(task) = sched.task_mut(caller_slot) {
                    if task.set_sleeping(reason).is_err() {
                        kerror!("sched", "invalid_sleep_transition"; from = task.state().name());
                        return false;
                    }
                }
                sched.notify_blocked(caller_slot);
                true // Blocked task always needs full context switch
            }
            BlockReason::Wait(reason, deadline) => {
                if is_idle_slot(caller_slot) {
                    kerror!("sched", "block_idle_attempt");
                    return false;
                }
                if let Some(task) = sched.task_mut(caller_slot) {
                    if task.set_waiting(reason, deadline).is_err() {
                        kerror!("sched", "invalid_wait_transition"; from = task.state().name());
                        return false;
                    }
                }
                sched.notify_blocked(caller_slot);
                sched.note_deadline(deadline);
                true // Blocked task always needs full context switch
            }
        };

        // Track whether the task was Running at entry (before any block transition).
        // If force_full_switch is set, the task is now blocked and MUST use full switch.
        let was_running = if force_full_switch {
            false // Treat as non-running to force full context switch
        } else {
            sched.task(caller_slot)
                .map(|t| t.state().is_running())
                .unwrap_or(false)
        };

        let cpu = percpu::cpu_id();

        // SMP FIX: Do NOT mark current task Ready here. A task must not
        // become Ready (schedulable) while its context hasn't been saved yet.
        // For simple preemption: transition happens below (safe - trap frame
        // already saved by IRQ handler, no kernel stack involvement).
        // For full context switch: transition happens in Phase 3 via
        // pending_stack_release after context_switch saves the context.

        // Find next task to run (from-task NOT on ready bitset)
        let caller_state_name = sched.task(caller_slot).map(|t| t.state().name()).unwrap_or("none");
        let next_slot = match sched.schedule() {
            Some(slot) if slot != caller_slot => {
                kdebug!("resched", "pick"; from = caller_slot as u64, to = slot as u64);
                slot
            }
            Some(_) => {
                // Same task selected — keep running (task is still Running, no undo needed)
                return false;
            }
            None => {
                // No ready tasks — check if we need idle
                let blocked = sched.task(caller_slot)
                    .map(|t| t.is_blocked()).unwrap_or(false);
                if !blocked {
                    let terminated = sched.task(caller_slot)
                        .map(|t| t.is_terminated()).unwrap_or(false);
                    if terminated {
                        // BUG: Exiting task with nothing ready — must go to idle, not keep running!
                        kerror!("resched", "terminated_no_ready"; slot = caller_slot as u64, state = caller_state_name);
                        idle_slot_for_cpu(percpu::cpu_id())
                    } else {
                        // Not blocked — keep running. May be Ready if woken between
                        // sleep_current/wait_current and this reschedule call. Fix up.
                        if let Some(t) = sched.task_mut(caller_slot) {
                            if *t.state() == TaskState::Ready {
                                t.clear_kernel_stack_owner();
                                crate::transition_or_evict!(t, set_running, cpu);
                            }
                        }
                        return false;
                    }
                } else {
                    // Blocked and nothing ready — go to this CPU's idle
                    idle_slot_for_cpu(percpu::cpu_id())
                }
            }
        };

        // Check if we need full context switch.
        // Simple preemption (trap-frame-only swap) is ONLY safe when:
        // 1. The from-task was Running at entry (entered from userspace via IRQ).
        //    If it was already non-Running, reschedule() was called from deep in a
        //    syscall handler and the kernel stack must be preserved.
        // 2. The from-task is not blocked (it can resume from user trap frame)
        // 3. The to-task doesn't need kernel context restore
        let from_blocked = sched.task(caller_slot)
            .map(|t| t.is_blocked()).unwrap_or(false);
        let to_needs_context = sched.task(next_slot)
            .map(|t| t.needs_context_restore()).unwrap_or(false);

        if was_running && !from_blocked && !to_needs_context {
            // Simple preemption: trap-frame-only swap, no context_switch.
            // Safe to mark Ready now — trap frame already saved by IRQ handler,
            // no kernel stack involvement, no context_switch race window.
            if let Some(current) = sched.task_mut(caller_slot) {
                if current.state().is_running() {
                    crate::transition_or_evict!(current, set_ready);
                    sched.notify_ready(caller_slot);
                }
            }

            set_current_slot(next_slot);

            // Extract and update per-CPU data while we have the lock
            if let Some(next) = sched.task_mut(next_slot) {
                crate::transition_or_evict!(next, set_running, cpu);

                // Update trap frame pointer
                let trap_ptr = &mut next.trap_frame as *mut TrapFrame;
                percpu::set_trap_frame(trap_ptr);

                // Update TTBR0
                if let Some(ref addr_space) = next.address_space {
                    percpu::set_ttbr0(addr_space.get_ttbr0());
                }
            }
            // Only set flag for user tasks (not idle) - this tells IRQ handler
            // to use user return path instead of kernel return path
            if !is_idle_slot(next_slot) {
                percpu::set_syscall_switched(1);
            }
            return true;
        }

        // Need full context switch - extract everything we need
        let from_ctx = match sched.task_mut(caller_slot) {
            Some(t) => {
                // Only mark for restore if task is alive
                // Terminated tasks (Evicting, Exiting, Dying, Dead) should never be restored
                if !t.is_terminated() {
                    t.mark_context_saved();
                }
                // SMP EXCLUSIVITY for blocked tasks:
                // Blocked tasks are visible to wake paths (which add them to the
                // ready bitset). kernel_stack_owner prevents a woken task from being
                // scheduled while context_switch is still saving its registers.
                // Cleared by pending_stack_release handler after context_switch.
                //
                // Preempted tasks (Running) don't need this — they stay Running
                // (not on ready bitset) until the pending handler transitions them
                // to Ready after context_switch saves their context.
                if t.is_blocked() {
                    t.set_kernel_stack_owner(cpu);
                }
                &mut t.context as *mut task::CpuContext
            }
            None => return false,
        };

        let (to_ctx, to_trap_frame, to_ttbr0) = match sched.task_mut(next_slot) {
            Some(t) => {
                // Prepare target state while holding lock
                crate::transition_or_evict!(t, set_running, cpu);
                t.mark_context_restored();
                // Clear any stale kernel_stack_owner from previous switch.
                // We're about to restore this task's context and run on its stack.
                t.clear_kernel_stack_owner();

                let ctx = &t.context as *const task::CpuContext;
                let trap = &mut t.trap_frame as *mut TrapFrame;
                let ttbr0 = t.address_space.as_ref()
                    .map(|a| a.get_ttbr0()).unwrap_or(0);
                (ctx, trap, ttbr0)
            }
            None => return false,
        };

        // Update per-CPU slot before releasing lock
        set_current_slot(next_slot);

        // Debug: Log context values before context switch
        let from_x30 = unsafe { (*from_ctx).x30 };
        let to_x30 = unsafe { (*to_ctx).x30 };
        kdebug!("sched", "context_switch_prep";
            from_slot = caller_slot as u64,
            to_slot = next_slot as u64,
            from_x30 = from_x30,
            to_x30 = to_x30
        );

        Some(SwitchDecision {
            from_slot: caller_slot,
            to_slot: next_slot,
            from_ctx,
            to_ctx,
            to_trap_frame,
            to_ttbr0,
        })
    }; // LOCK RELEASED HERE

    let decision = match decision {
        Some(d) => d,
        None => return false,
    };

    // ========================================================================
    // PHASE 2: Context switch with NO LOCK
    // ========================================================================
    // Only IRQs are disabled (IrqGuard from SpinLock was dropped, take new one)
    let _irq_guard = crate::arch::aarch64::sync::IrqGuard::new();

    // Activate target address space
    if decision.to_ttbr0 != 0 {
        unsafe {
            core::arch::asm!(
                "msr ttbr0_el1, {0}",
                "isb",
                "tlbi vmalle1is",
                "dsb ish",
                "isb",
                in(reg) decision.to_ttbr0,
            );
        }
        percpu::set_ttbr0(decision.to_ttbr0);
    }

    // Update per-CPU data
    percpu::set_trap_frame(decision.to_trap_frame);
    // Only set flag for user tasks (not idle) - this tells IRQ handler
    // to use user return path instead of kernel return path
    if !is_idle_slot(decision.to_slot) {
        percpu::set_syscall_switched(1);
    }

    // SAFETY CHECK: Verify target context has valid x30 (return address)
    // A context with x30=0 would cause an instruction abort at address 0
    let to_x30 = unsafe { (*decision.to_ctx).x30 };
    if to_x30 == 0 {
        kerror!("sched", "switch_to_null_x30";
            from_slot = decision.from_slot as u64,
            to_slot = decision.to_slot as u64,
            to_sp = unsafe { (*decision.to_ctx).sp }
        );
        // Don't switch to a task with null return address - this would crash
        // Instead, mark it for eviction and return without switching
        {
            let mut sched = task::scheduler();
            if let Some(task) = sched.task_mut(decision.to_slot) {
                crate::kerror!("sched", "evicting_corrupt_task";
                    slot = decision.to_slot as u64,
                    pid = task.id as u64
                );
                let _ = task.evict(task::EvictionReason::StateCorruption);
            }
        }
        return false;
    }

    // Record which task needs finalization after context_switch.
    // Set AFTER all safety checks (x30 verify) so abort paths don't leave stale pending.
    percpu::set_pending_release(decision.from_slot as u32);

    // Do context switch - returns when switched BACK to us
    unsafe {
        task::context_switch(decision.from_ctx, decision.to_ctx);
    }

    // ========================================================================
    // PHASE 3: Finalization (reacquire lock)
    // ========================================================================
    // We've been switched back to from_slot.
    //
    // CRITICAL: Read pending_stack_release BEFORE re-enabling IRQs.
    // This was set by whoever context_switch'd TO us on this CPU.
    // Reading it while IRQs are disabled prevents a nested reschedule
    // from overwriting it.
    let pending_slot = percpu::take_pending_release();

    drop(_irq_guard);

    // CRITICAL: Clear syscall_switched because we're returning to our
    // original context (from_slot), not a new task. The flag was set when we
    // originally switched away, but now we're coming BACK. If we don't clear it,
    // irq_from_kernel would incorrectly use irq_kernel_to_user path which would
    // ERET to from_slot's trap_frame (which could be idle with ELR=0).
    percpu::set_syscall_switched(0);

    {
        let mut sched = task::scheduler();

        // --- Process pending_stack_release from the context_switch that brought us here ---
        // The CPU that context_switch'd TO us stored its from-task slot in per-CPU
        // pending. We finalize that task now (under lock):
        //   - Preempted (Running): transition to Ready + add to ready bitset
        //   - Blocked (Sleeping/Waiting): clear kernel_stack_owner so wake makes it schedulable
        //   - Terminated: nothing to do
        if pending_slot != 0xFFFFFFFF {
            let ps = pending_slot as usize;
            if let Some(t) = sched.task_mut(ps) {
                if t.state().is_running() {
                    // Preempted task: context saved, now truly Ready
                    crate::transition_or_evict!(t, set_ready);
                    sched.notify_ready(ps);
                } else if !t.is_terminated() {
                    // Blocked or woken-before-reschedule: context saved,
                    // clear kernel_stack_owner so task is schedulable
                    t.clear_kernel_stack_owner();
                }
            }
        }

        // --- Restore our own state (from_slot) ---
        set_current_slot(decision.from_slot);
        let cpu = percpu::cpu_id();

        if let Some(t) = sched.task_mut(decision.from_slot) {
            match t.state() {
                TaskState::Ready => {
                    crate::transition_or_evict!(t, set_running, cpu);
                }
                TaskState::Running { .. } => { /* Already running */ }
                TaskState::Dying { .. } | TaskState::Dead |
                TaskState::Exiting { .. } | TaskState::Evicting { .. } => {
                    // Died while switched out - cleanup handles it
                }
                _ => {
                    kwarn!("sched", "resume_unexpected"; slot = decision.from_slot as u64);
                }
            }
            t.mark_context_restored();

            // Update per-CPU trap frame pointer
            let trap_ptr = &mut t.trap_frame as *mut TrapFrame;
            percpu::set_trap_frame(trap_ptr);

            // Restore address space
            if let Some(ref addr_space) = t.address_space {
                unsafe { addr_space.activate(); }
                percpu::set_ttbr0(addr_space.get_ttbr0());
            }
        }
    } // Lock released

    false
}

// ============================================================================
// Blocking Operations
// ============================================================================

/// Wake a blocked task by PID.
///
/// Uses try_scheduler() to avoid deadlock when called from contexts that
/// already hold the scheduler lock (e.g., cleanup paths, reap_terminated).
/// If the lock is held, defers the wake via request_wake().
///
/// After waking, sends a reschedule IPI to other CPUs so they can pick
/// up the newly ready task.
///
/// # Returns
/// true if task was woken (or wake was deferred), false if task not found.
pub fn wake(pid: u32) -> bool {
    if let Some(mut sched) = task::try_scheduler() {
        let woken = sched.wake_by_pid(pid);
        if woken {
            // Send reschedule IPI to other CPUs
            send_reschedule_ipi();
        }
        woken
    } else {
        // Lock held - defer the wake
        crate::arch::aarch64::sync::cpu_flags().request_wake(pid);
        true // Wake will be processed by process_pending_wakes
    }
}

/// Send reschedule IPI (SGI 0) to ONE idle CPU.
///
/// Targeted: only wakes a single idle CPU instead of broadcasting to all.
/// If no CPU is idle, the timer tick will eventually catch up.
fn send_reschedule_ipi() {
    let my_cpu = percpu::cpu_id();
    for i in 0..percpu::MAX_CPUS {
        if i as u32 != my_cpu {
            if let Some(cpu_data) = percpu::try_get_cpu_data(i) {
                if cpu_data.get_state() == percpu::CpuState::Online && cpu_data.is_idle() {
                    crate::platform::current::gic::send_sgi(i as u32, 0);
                    return; // Only need to wake ONE idle CPU
                }
            }
        }
    }
}

/// Put current task to sleep (event loop waiting for any event).
///
/// Does NOT reschedule - caller must do that after setting up wait condition.
///
/// # Returns
/// true if state transition succeeded
pub fn sleep_current(reason: task::SleepReason) -> bool {
    let mut sched = task::scheduler();
    let slot = current_slot();

    if is_idle_slot(slot) {
        kerror!("sched", "block_idle_attempt");
        return false;
    }

    let ok = if let Some(task) = sched.task_mut(slot) {
        if task.set_sleeping(reason).is_err() {
            kerror!("sched", "invalid_sleep_transition"; from = task.state().name());
            return false;
        }
        // CRITICAL: Mark task as needing full context restore and set
        // kernel_stack_owner. Between sleep_current() returning and the caller's
        // reschedule(), a wake could make this task Ready + on bitset. Without
        // kernel_stack_owner, another CPU could select it while our kernel stack
        // is still live. The pending_stack_release handler clears this after
        // context_switch.
        task.mark_context_saved();
        task.set_kernel_stack_owner(percpu::cpu_id());
        true
    } else {
        false
    };
    if ok {
        sched.notify_blocked(slot);
    }
    ok
}

/// Put current task into waiting state with deadline.
///
/// Does NOT reschedule - caller must do that after setting up wait condition.
///
/// # Returns
/// true if state transition succeeded
pub fn wait_current(reason: task::WaitReason, deadline: u64) -> bool {
    let mut sched = task::scheduler();
    let slot = current_slot();

    if is_idle_slot(slot) {
        kerror!("sched", "block_idle_attempt");
        return false;
    }

    let (ok, pid) = if let Some(task) = sched.task_mut(slot) {
        if task.set_waiting(reason, deadline).is_err() {
            kerror!("sched", "invalid_wait_transition"; from = task.state().name());
            return false;
        }
        // CRITICAL: Same as sleep_current — prevent race with wake paths
        task.mark_context_saved();
        task.set_kernel_stack_owner(percpu::cpu_id());
        (true, task.id)
    } else {
        (false, 0)
    };
    if ok {
        sched.notify_blocked(slot);
        kdebug!("sched", "blocked"; pid = pid as u64, deadline = deadline);
        sched.note_deadline(deadline);
    }
    ok
}

/// Voluntarily yield the CPU to another task.
pub fn yield_current() {
    reschedule();
}

// ============================================================================
// Timer Tick Handling
// ============================================================================

/// Handle timer tick - check timeouts and set reschedule flag if needed.
///
/// Called from timer interrupt handler. Uses try_scheduler() to avoid
/// deadlock if the scheduler lock is held by interrupted code.
pub fn timer_tick(current_time: u64) {
    // Use try_scheduler to avoid deadlock if syscall holds the lock
    let Some(mut sched) = task::try_scheduler() else {
        // Lock held - skip this tick, timeouts will be checked next tick
        // Still set need_resched so we retry after the syscall completes
        crate::arch::aarch64::sync::cpu_flags().set_need_resched();
        return;
    };

    let woken = sched.check_timeouts(current_time);

    if woken > 0 {
        crate::arch::aarch64::sync::cpu_flags().set_need_resched();
    }

    // Drop scheduler lock before liveness check (it takes its own lock)
    drop(sched);

    // Periodic liveness check (every LIVENESS_CHECK_INTERVAL ticks)
    // Uses atomic counter — multiple CPUs call timer_tick() concurrently.
    use core::sync::atomic::{AtomicU64, Ordering};
    static LIVENESS_TICK_COUNTER: AtomicU64 = AtomicU64::new(0);
    let prev = LIVENESS_TICK_COUNTER.fetch_add(1, Ordering::Relaxed);
    if prev + 1 >= super::liveness::LIVENESS_CHECK_INTERVAL {
        // CAS to zero — only one CPU runs the liveness check
        if LIVENESS_TICK_COUNTER.compare_exchange(
            prev + 1,
            0,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ).is_ok() {
            let _ = super::liveness::check_liveness(current_time);
        }
    }
}

// ============================================================================
// Pending Stack Release Processing (Trampoline Support)
// ============================================================================

/// Process any pending_stack_release left by a context_switch that returned
/// to a trampoline (user_task_trampoline, idle_entry) instead of back to
/// reschedule_inner's Phase 3.
///
/// Must be called from code paths entered via context_switch that bypass
/// Phase 3. This finalizes the from-task:
/// - Preempted (Running): transition to Ready + add to ready bitset
/// - Blocked: clear kernel_stack_owner so wake makes it schedulable
/// - Terminated: nothing to do
pub fn process_pending_from_switch() {
    let pending_slot = percpu::take_pending_release();
    if pending_slot == 0xFFFFFFFF {
        return;
    }

    let mut sched = task::scheduler();
    let ps = pending_slot as usize;
    if let Some(t) = sched.task_mut(ps) {
        if t.state().is_running() {
            crate::transition_or_evict!(t, set_ready);
            sched.notify_ready(ps);
        } else if !t.is_terminated() {
            t.clear_kernel_stack_owner();
        }
    }
}
