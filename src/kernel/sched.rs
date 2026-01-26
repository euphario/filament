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
use super::task::{self, TaskState, current_slot, set_current_slot};
use super::task::{CURRENT_TRAP_FRAME, CURRENT_TTBR0, SYSCALL_SWITCHED_TASK, TrapFrame};
use core::sync::atomic::Ordering;

/// Slot reserved for idle task
pub const IDLE_SLOT: usize = 0;

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
    // ========================================================================
    // PHASE 1: Decision under lock
    // ========================================================================
    let decision: Option<SwitchDecision> = {
        let mut sched = task::scheduler();
        let caller_slot = current_slot();

        // Mark current task as Ready if it was Running
        if let Some(current) = sched.task_mut(caller_slot) {
            if *current.state() == TaskState::Running {
                crate::transition_or_evict!(current, set_ready);
            }
        }

        // Find next task to run
        let next_slot = match sched.schedule() {
            Some(slot) if slot != caller_slot => slot,
            Some(_) => {
                // Same task selected - set back to Running, return
                if let Some(t) = sched.task_mut(caller_slot) {
                    if *t.state() == TaskState::Ready {
                        crate::transition_or_evict!(t, set_running);
                    }
                }
                return false;
            }
            None => {
                // No ready tasks - check if we need idle
                let blocked = sched.task(caller_slot)
                    .map(|t| t.is_blocked()).unwrap_or(false);
                if !blocked {
                    // Not blocked, just keep running
                    if let Some(t) = sched.task_mut(caller_slot) {
                        if *t.state() == TaskState::Ready {
                            crate::transition_or_evict!(t, set_running);
                        }
                    }
                    return false;
                }
                // Blocked and nothing ready - go to idle
                IDLE_SLOT
            }
        };

        // Check if we need full context switch
        let from_blocked = sched.task(caller_slot)
            .map(|t| t.is_blocked()).unwrap_or(false);
        let to_needs_context = sched.task(next_slot)
            .map(|t| t.needs_context_restore).unwrap_or(false);

        if !from_blocked && !to_needs_context {
            // Simple preemption case - no context_switch needed
            // Just update variables, actual switch at eret
            set_current_slot(next_slot);

            // Extract and update globals while we have the lock
            if let Some(next) = sched.task_mut(next_slot) {
                crate::transition_or_evict!(next, set_running);

                // Update trap frame pointer
                let trap_ptr = &mut next.trap_frame as *mut TrapFrame;
                CURRENT_TRAP_FRAME.store(trap_ptr, Ordering::Release);

                // Update TTBR0
                if let Some(ref addr_space) = next.address_space {
                    CURRENT_TTBR0.store(addr_space.get_ttbr0(), Ordering::Release);
                }
            }
            // Only set flag for user tasks (not idle) - this tells IRQ handler
            // to use user return path instead of kernel return path
            if next_slot != IDLE_SLOT {
                SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);
            }
            return true;
        }

        // Need full context switch - extract everything we need
        let from_ctx = match sched.task_mut(caller_slot) {
            Some(t) => {
                // Only mark for restore if task is alive
                // Terminated tasks (Evicting, Exiting, Dying, Dead) should never be restored
                if !t.is_terminated() {
                    t.needs_context_restore = true;
                }
                &mut t.context as *mut task::CpuContext
            }
            None => return false,
        };

        let (to_ctx, to_trap_frame, to_ttbr0) = match sched.task_mut(next_slot) {
            Some(t) => {
                // Prepare target state while holding lock
                crate::transition_or_evict!(t, set_running);
                t.needs_context_restore = false;

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
                "tlbi vmalle1",
                "dsb sy",
                "isb",
                in(reg) decision.to_ttbr0,
            );
        }
        CURRENT_TTBR0.store(decision.to_ttbr0, Ordering::Release);
    }

    // Update globals
    CURRENT_TRAP_FRAME.store(decision.to_trap_frame, Ordering::Release);
    // Only set flag for user tasks (not idle) - this tells IRQ handler
    // to use user return path instead of kernel return path
    if decision.to_slot != IDLE_SLOT {
        SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);
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

    // Do context switch - returns when switched BACK to us
    unsafe {
        task::context_switch(decision.from_ctx, decision.to_ctx);
    }

    // ========================================================================
    // PHASE 3: Finalization (reacquire lock)
    // ========================================================================
    // We've been switched back to from_slot
    drop(_irq_guard);

    // CRITICAL: Clear SYSCALL_SWITCHED_TASK because we're returning to our
    // original context (from_slot), not a new task. The flag was set when we
    // originally switched away, but now we're coming BACK. If we don't clear it,
    // irq_from_kernel would incorrectly use irq_kernel_to_user path which would
    // ERET to from_slot's trap_frame (which could be idle with ELR=0).
    SYSCALL_SWITCHED_TASK.store(0, Ordering::Release);

    {
        let mut sched = task::scheduler();
        set_current_slot(decision.from_slot);

        if let Some(t) = sched.task_mut(decision.from_slot) {
            match t.state() {
                TaskState::Ready => {
                    crate::transition_or_evict!(t, set_running);
                }
                TaskState::Running => { /* Already running */ }
                TaskState::Dying { .. } | TaskState::Dead |
                TaskState::Exiting { .. } | TaskState::Evicting { .. } => {
                    // Died while switched out - cleanup handles it
                }
                _ => {
                    kwarn!("sched", "resume_unexpected"; slot = decision.from_slot as u64);
                }
            }
            t.needs_context_restore = false;

            // Update trap frame pointer
            let trap_ptr = &mut t.trap_frame as *mut TrapFrame;
            CURRENT_TRAP_FRAME.store(trap_ptr, Ordering::Release);

            // Restore address space
            if let Some(ref addr_space) = t.address_space {
                unsafe { addr_space.activate(); }
                CURRENT_TTBR0.store(addr_space.get_ttbr0(), Ordering::Release);
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
/// # Returns
/// true if task was woken (or wake was deferred), false if task not found.
pub fn wake(pid: u32) -> bool {
    if let Some(mut sched) = task::try_scheduler() {
        sched.wake_by_pid(pid)
    } else {
        // Lock held - defer the wake
        crate::arch::aarch64::sync::cpu_flags().request_wake(pid);
        true // Wake will be processed by process_pending_wakes
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

    if slot == IDLE_SLOT {
        kerror!("sched", "block_idle_attempt");
        return false;
    }

    if let Some(task) = sched.task_mut(slot) {
        if task.set_sleeping(reason).is_err() {
            kerror!("sched", "invalid_sleep_transition"; from = task.state().name());
            return false;
        }
        return true;
    }
    false
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

    if slot == IDLE_SLOT {
        kerror!("sched", "block_idle_attempt");
        return false;
    }

    if let Some(task) = sched.task_mut(slot) {
        if task.set_waiting(reason, deadline).is_err() {
            kerror!("sched", "invalid_wait_transition"; from = task.state().name());
            return false;
        }
        kdebug!("sched", "blocked"; pid = task.id as u64, deadline = deadline);
        sched.note_deadline(deadline);
        return true;
    }
    false
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
    static mut LIVENESS_TICK_COUNTER: u64 = 0;
    unsafe {
        LIVENESS_TICK_COUNTER += 1;
        if LIVENESS_TICK_COUNTER >= super::liveness::LIVENESS_CHECK_INTERVAL {
            LIVENESS_TICK_COUNTER = 0;
            let _ = super::liveness::check_liveness(current_time);
        }
    }
}
