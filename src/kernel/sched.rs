//! Scheduler - Internal Kernel Scheduling Mechanisms
//!
//! This module provides the kernel-internal scheduling functions. These are
//! **not** syscall entry points - they are called by syscall handlers and
//! interrupt handlers to manage task execution.
//!
//! # Architecture
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────────────┐
//! │  Timer Interrupt / Syscall / Exception                           │
//! └──────────────────────────────┬───────────────────────────────────┘
//!                                │
//!                                ▼
//!          ┌─────────────────────────────────────────┐
//!          │           sched::reschedule()           │
//!          │  1. Mark current task Ready             │
//!          │  2. Pick highest-priority Ready task    │
//!          │  3. Context switch to new task          │
//!          └─────────────────────────────────────────┘
//!                                │
//!            ┌───────────────────┼───────────────────┐
//!            ▼                   ▼                   ▼
//!       ┌─────────┐        ┌─────────┐        ┌─────────┐
//!       │ Task A  │        │ Task B  │        │  Idle   │
//!       │ Ready   │        │ Running │        │ slot 0  │
//!       └─────────┘        └─────────┘        └─────────┘
//! ```
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
//! This module enforces the task state machine:
//!
//! - **Ready** → Running (via `reschedule`)
//! - **Running** → Ready (via `yield_current` or preemption)
//! - **Running** → Sleeping (via `sleep_current`) - no deadline
//! - **Running** → Waiting (via `wait_current`) - has deadline
//! - **Sleeping/Waiting** → Ready (via `wake` or timeout)
//!
//! # Sleeping vs Waiting
//!
//! Two distinct blocked states for different use cases:
//!
//! | State | Use Case | Deadline | Example |
//! |-------|----------|----------|---------|
//! | Sleeping | Event loop idle | None | devd waiting for events |
//! | Waiting | Specific request | Yes | IPC receive with timeout |
//!
//! This distinction enables:
//! - **Tickless**: Only scan Waiting tasks for next wake time
//! - **Liveness**: Only probe Sleeping tasks; Waiting has deadline enforcement
//!
//! # Design Principles
//!
//! 1. **Idle task**: Slot 0 is always the idle task (never blocks, lowest priority)
//! 2. **State machine**: All transitions validated by [`task::state`] module
//! 3. **Separation**: syscall.rs → sched.rs → task.rs (layered responsibility)
//! 4. **IRQ-safe**: All operations use spinlocks, safe from interrupt context

use crate::{kerror, kdebug};
use super::task::{self, TaskState, Priority, current_slot, set_current_slot};
use super::task::{update_current_task_globals, SYSCALL_SWITCHED_TASK};
use core::sync::atomic::Ordering;

/// Idle task priority (lower than Low)
pub const PRIORITY_IDLE: Priority = Priority::Low;  // We'll use slot 0 check instead

/// Slot reserved for idle task
pub const IDLE_SLOT: usize = 0;

// ============================================================================
// Idle State Machine
// ============================================================================
//
// When no task is Ready, the scheduler enters idle mode:
//
//     ┌─────────────────────────────────────────────────────────┐
//     │                  reschedule()                           │
//     └──────────────────────┬──────────────────────────────────┘
//                            │
//              ┌─────────────┴─────────────┐
//              │ schedule() returns None   │
//              │ (no Ready tasks)          │
//              └─────────────┬─────────────┘
//                            │
//                            ▼
//     ┌──────────────────────────────────────────────────────────┐
//     │              compute_idle_deadline()                     │
//     │  Scan all Waiting tasks for earliest deadline            │
//     └──────────────────────┬───────────────────────────────────┘
//                            │
//          ┌─────────────────┴─────────────────┐
//          │                                   │
//          ▼                                   ▼
//   ┌──────────────┐                   ┌──────────────┐
//   │ Has deadline │                   │ No deadline  │
//   │ (Waiting)    │                   │ (Sleeping)   │
//   └──────┬───────┘                   └──────┬───────┘
//          │                                   │
//          ▼                                   ▼
//   ┌──────────────┐                   ┌──────────────┐
//   │ Set timer    │                   │ Disable      │
//   │ for deadline │                   │ timer        │
//   └──────┬───────┘                   └──────┬───────┘
//          │                                   │
//          └─────────────┬─────────────────────┘
//                        │
//                        ▼
//               ┌────────────────┐
//               │      WFI       │◄────────┐
//               │ (wait for IRQ) │         │
//               └───────┬────────┘         │
//                       │                  │
//                       ▼                  │
//              ┌────────────────┐          │
//              │ IRQ fires      │          │
//              │ (timer/UART/..)│          │
//              └───────┬────────┘          │
//                      │                   │
//                      ▼                   │
//             ┌─────────────────┐          │
//             │ Check timeouts  │          │
//             │ Wake expired    │          │
//             └───────┬─────────┘          │
//                     │                    │
//        ┌────────────┴────────────┐       │
//        │                         │       │
//        ▼                         ▼       │
// ┌─────────────┐          ┌─────────────┐ │
// │ Has Ready   │          │ No Ready    │─┘
// │ task        │          │ yet         │
// └──────┬──────┘          └─────────────┘
//        │
//        ▼
// ┌─────────────────┐
// │ Switch to task  │
// │ Exit idle       │
// └─────────────────┘

/// Reason for entering idle state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdleReason {
    /// All tasks are blocked (normal operation)
    AllBlocked,
    /// Current task blocked, staying in idle until woken
    CurrentBlocked,
}

/// Result of computing idle deadline
#[derive(Debug, Clone, Copy)]
pub enum IdleDeadline {
    /// No deadline - wait indefinitely for external interrupt
    Infinite,
    /// Wake at specific tick (earliest Waiting task deadline)
    At(u64),
}

// ============================================================================
// Core Scheduling Operations
// ============================================================================

/// Reschedule: pick the next task to run and switch to it.
///
/// This is the main scheduling entry point. It:
/// 1. Finds the highest-priority Ready task
/// 2. Switches to that task (or enters idle if none ready)
/// 3. Updates all scheduler globals correctly
///
/// # State Machine
///
/// ```text
/// reschedule() called
///       │
///       ▼
/// ┌─────────────────┐     ┌─────────────────┐
/// │ Has Ready task? │─Yes─▶ Switch to task  │
/// └────────┬────────┘     └─────────────────┘
///          │No
///          ▼
/// ┌─────────────────┐     ┌─────────────────┐
/// │ Current blocked?│─No──▶ Return (yield)  │
/// └────────┬────────┘     └─────────────────┘
///          │Yes
///          ▼
/// ┌─────────────────┐
/// │  enter_idle()   │
/// │  (state machine)│
/// └─────────────────┘
/// ```
///
/// # Safety
/// Must be called with IRQs disabled (or via IrqGuard).
///
/// # Returns
/// true if we switched to a different task, false if we're staying on current.
pub fn reschedule() -> bool {
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    unsafe {
        let sched = task::scheduler();
        let caller_slot = current_slot();

        // Mark current task as Ready if it was Running
        // (Unless it's Blocked or Terminated, which should stay as-is)
        if let Some(current) = sched.task_mut(caller_slot) {
            if *current.state() == TaskState::Running {
                let _ = current.set_ready();
            }
        }

        // Find next task to run
        if let Some(next_slot) = sched.schedule() {
            return switch_to_task(sched, caller_slot, next_slot);
        }

        // No ready tasks - check if we can safely return
        let current_blocked = sched.task(caller_slot)
            .map(|t| t.is_blocked())
            .unwrap_or(false);

        if !current_blocked {
            // Current task is not blocked (e.g., yield from Ready task)
            // Safe to return - task will continue
            return false;
        }

        // Current task is blocked and nothing is ready
        // Enter the idle state machine
        enter_idle(sched, IdleReason::CurrentBlocked)
    }
}

/// Switch to a specific task slot.
///
/// This handles two cases:
/// 1. **Current task runnable (Running/Ready)**: Just set variables and return.
///    The actual switch happens at eret (exception return).
/// 2. **Current task blocked (Sleeping/Waiting)**: Do a proper kernel context
///    switch so we can resume the blocked task's kernel execution later.
///
/// # Safety
/// Caller must hold scheduler lock (IRQs disabled).
unsafe fn switch_to_task(
    sched: &mut task::Scheduler,
    from_slot: usize,
    to_slot: usize,
) -> bool {
    if to_slot == from_slot {
        // Same task continues
        if let Some(current) = sched.task_mut(from_slot) {
            let _ = current.set_running();
        }
        return false;
    }

    // Check if current task is blocked (Sleeping/Waiting).
    // If so, we need a proper kernel context switch to suspend its execution.
    let from_blocked = sched.task(from_slot)
        .map(|t| t.is_blocked())
        .unwrap_or(false);

    // Check if target task has saved kernel context that needs restoring.
    // This is true for:
    // - New tasks (initial context with trampoline)
    // - Tasks that were blocked and had their context saved
    // - Kernel tasks (like idle)
    let to_needs_context = sched.task(to_slot)
        .map(|t| t.context_saved)
        .unwrap_or(false);


    // Need context_switch if:
    // - FROM task is blocked (need to save its context), OR
    // - TO task has saved context (need to restore via context_switch)
    if from_blocked || to_needs_context {
        // Need full kernel context switch.
        // This saves from's kernel stack state and restores to's.
        //
        // Get context pointers BEFORE modifying any state
        let from_ctx = match sched.task_mut(from_slot) {
            Some(t) => {
                // Mark that from_task's context is being saved.
                // CRITICAL: We ALWAYS set this when entering the context_switch branch,
                // not just when blocked. context_switch() saves the context regardless
                // of WHY we're switching. If we only set it when blocked, a yielding
                // (non-blocked) task that context_switches out won't have context_saved
                // set, and when switching back, we might skip context_switch entirely
                // because to_needs_context would be false.
                t.context_saved = true;
                &mut t.context as *mut task::CpuContext
            }
            None => return false,
        };
        let to_ctx = match sched.task(to_slot) {
            Some(t) => &t.context as *const task::CpuContext,
            None => return false,
        };

        // Switch address space for target task
        if let Some(ref task) = sched.task(to_slot) {
            if let Some(ref addr_space) = task.address_space {
                addr_space.activate();
            }
        }

        // Update scheduler state (per-CPU slot)
        set_current_slot(to_slot);

        // Mark target as Running and clear its context_saved flag
        // (it's about to be restored, so it won't need context_switch next time
        // unless it blocks again)
        if let Some(next) = sched.task_mut(to_slot) {
            let _ = next.set_running();
            next.context_saved = false;
        }

        // Update globals for target task
        update_current_task_globals();
        SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);

        // Do the actual context switch - saves from_ctx, loads to_ctx
        // When context_switch returns, we've been switched BACK to from_slot
        task::context_switch(from_ctx, to_ctx);

        // === We resume here when switched back ===
        // Restore our state (slot, address space, globals)
        set_current_slot(from_slot);

        // Mark ourselves as Running again and clear our context_saved flag
        if let Some(t) = sched.task_mut(from_slot) {
            let _ = t.set_running();
            t.context_saved = false;  // We've been restored
        }

        // Restore our address space
        if let Some(ref task) = sched.task(from_slot) {
            if let Some(ref addr_space) = task.address_space {
                addr_space.activate();
            }
        }

        // Update globals to point to us
        update_current_task_globals();

        // Return false - we didn't "switch" in the sense of eret, we resumed
        return false;
    }

    // Current task is runnable (Running/Ready) and target doesn't need context restore.
    // This is the preemption case - simple variable-based switch.
    // The actual switch happens at eret using trap frames.

    set_current_slot(to_slot);

    if let Some(next) = sched.task_mut(to_slot) {
        let _ = next.set_running();
    }

    update_current_task_globals();
    SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);

    true
}

/// Enter the idle state - context switch to idle task.
///
/// This is called when no task is ready to run and the current task is blocked.
/// Instead of doing inline WFI, we context switch to the real idle task in slot 0.
/// The idle task runs WFI in a loop, and when woken by an interrupt, the scheduler
/// will context switch to whatever task became ready.
///
/// # Safety
/// Caller must hold scheduler lock (IRQs disabled).
///
/// # Returns
/// true when we switch to idle (or another task if one became ready)
unsafe fn enter_idle(sched: &mut task::Scheduler, reason: IdleReason) -> bool {
    kdebug!("sched", "enter_idle"; reason = reason as u64);

    // First, check if any deadlines have passed (timeout wakeups)
    let current_time = crate::platform::current::timer::counter();
    let woken = sched.check_timeouts(current_time);

    if woken > 0 {
        // A task was woken - try to schedule to it
        if let Some(next_slot) = sched.schedule() {
            kdebug!("sched", "idle_woke_task"; slot = next_slot as u64);
            return switch_to_task(sched, current_slot(), next_slot);
        }
    }

    // No ready tasks found - switch to the idle task (slot 0)
    // The idle task will loop doing WFI until an interrupt wakes a task,
    // at which point the timer IRQ handler will set NEED_RESCHED and
    // do_resched_if_needed will switch away from idle.
    if current_slot() == IDLE_SLOT {
        // Already the idle task - should not happen normally
        // Just return, we'll loop in idle_entry
        return false;
    }

    kdebug!("sched", "switch_to_idle");
    switch_to_task(sched, current_slot(), IDLE_SLOT)
}

/// Compute the earliest deadline from all Waiting tasks and ObjectService timers.
///
/// This scans all tasks and finds the minimum deadline among:
/// 1. Tasks in Waiting state (have deadline in task state)
/// 2. Scheduler's next_deadline (from ObjectService timers via note_deadline())
///
/// # Returns
/// - `IdleDeadline::At(tick)` - earliest deadline tick
/// - `IdleDeadline::Infinite` - no deadlines pending
fn compute_idle_deadline(sched: &task::Scheduler) -> IdleDeadline {
    let mut earliest: u64 = u64::MAX;

    // Check task Waiting state deadlines
    for (_slot, task_opt) in sched.iter_tasks() {
        if let Some(task) = task_opt {
            if let Some(deadline) = task.state().deadline() {
                if deadline < earliest {
                    earliest = deadline;
                }
            }
        }
    }

    // Also check scheduler's next_deadline (ObjectService timers)
    let sched_deadline = sched.get_next_deadline();
    if sched_deadline < earliest {
        earliest = sched_deadline;
    }

    if earliest == u64::MAX {
        IdleDeadline::Infinite
    } else {
        IdleDeadline::At(earliest)
    }
}

/// Wake a blocked task by PID.
///
/// This is the ONLY way a Blocked task becomes Ready (other than timeout).
/// Calling yield from a blocked task does NOT make it Ready.
///
/// # Returns
/// true if task was woken, false if not found or not blocked.
pub fn wake(pid: u32) -> bool {
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    unsafe {
        let sched = task::scheduler();
        sched.wake_by_pid(pid)
    }
}

/// Block the current task with a reason.
///
/// The task will stay Blocked until explicitly woken via `wake()` or timeout.
/// Put current task to sleep (event loop waiting for any event).
///
/// Use this for processes that are idle in their event loop,
/// waiting for any incoming event. No timeout - they sleep until woken.
///
/// # Important
/// Does NOT reschedule - caller must do that after setting up wait condition.
///
/// # Returns
/// true if state transition succeeded, false if invalid (e.g., blocking idle task)
pub fn sleep_current(reason: task::SleepReason) -> bool {
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    unsafe {
        let sched = task::scheduler();
        let slot = current_slot();

        if let Some(task) = sched.task_mut(slot) {
            // Don't block the idle task!
            if slot == IDLE_SLOT {
                kerror!("sched", "block_idle_attempt");
                return false;
            }

            // Use state machine for validated transition
            if task.set_sleeping(reason).is_err() {
                kerror!("sched", "invalid_sleep_transition"; from = task.state().name());
                return false;
            }
            return true;
        }
    }
    false
}

/// Put current task into waiting state (specific request with deadline).
///
/// Use this for request-response patterns where we expect a reply
/// within a specific time. If deadline passes, liveness checks can act.
///
/// # Important
/// Does NOT reschedule - caller must do that after setting up wait condition.
///
/// # Returns
/// true if state transition succeeded, false if invalid (e.g., blocking idle task)
pub fn wait_current(reason: task::WaitReason, deadline: u64) -> bool {
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    unsafe {
        let sched = task::scheduler();
        let slot = current_slot();

        if let Some(task) = sched.task_mut(slot) {
            if slot == IDLE_SLOT {
                kerror!("sched", "block_idle_attempt");
                return false;
            }

            // Use state machine for validated transition
            if task.set_waiting(reason, deadline).is_err() {
                kerror!("sched", "invalid_wait_transition"; from = task.state().name());
                return false;
            }
            // Notify scheduler of deadline for tickless optimization
            sched.note_deadline(deadline);
            return true;
        }
    }
    false
}

/// Yield the current task's CPU voluntarily.
///
/// This marks the current task as Ready (not Running) and reschedules.
///
/// # Important
/// This does NOT change a Blocked task to Ready! If the task is Blocked,
/// it stays Blocked. This is critical for correct event waiting.
///
/// # Returns
/// true if we switched to a different task.
pub fn yield_current() -> bool {
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    unsafe {
        let sched = task::scheduler();
        let slot = current_slot();

        if let Some(task) = sched.task_mut(slot) {
            // Only change Running → Ready, don't touch Blocked!
            if *task.state() == TaskState::Running {
                let _ = task.set_ready();
            }
            // If task is Blocked, it stays Blocked - this is intentional!
        }
    }

    // Now reschedule
    reschedule()
}

/// Check if current task is blocked.
pub fn is_current_blocked() -> bool {
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    unsafe {
        let sched = task::scheduler();
        let slot = current_slot();

        if let Some(task) = sched.task(slot) {
            return task.is_blocked();
        }
    }
    false
}

// ============================================================================
// Idle Task
// ============================================================================

// NOTE: Idle task creation is now handled internally by task::init_scheduler().
// The idle task is a proper kernel task in slot 0 with its own stack and context.
// See kernel/idle.rs for the idle task implementation.

// ============================================================================
// Timer Integration
// ============================================================================

/// Called from timer interrupt handler.
///
/// This:
/// 1. Calls check_timeouts to wake tasks whose timers expired
/// 2. Periodically checks liveness of waiting tasks
/// 3. Sets the need_resched flag if any task was woken
/// 4. Actual reschedule happens at exception return (do_resched_if_needed)
/// Called from timer interrupt handler with the current hardware counter value.
///
/// The `current_time` parameter is the canonical time source (hardware counter)
/// used for all scheduling decisions. This ensures consistent units across:
/// - Task deadlines (Waiting state)
/// - Timer objects
/// - Tickless wake calculations
///
/// # Arguments
/// * `current_time` - Hardware counter value from `timer::counter()`
pub fn timer_tick(current_time: u64) {
    // IRQs are already disabled in interrupt context

    unsafe {
        let sched = task::scheduler();
        let woken = sched.check_timeouts(current_time);

        // Periodically check liveness of waiting tasks
        // Convert interval to counter units: LIVENESS_CHECK_INTERVAL is in "ticks" (~100/sec)
        // For now, check every ~100 timer interrupts by using a simple counter
        static mut LIVENESS_COUNTER: u64 = 0;
        LIVENESS_COUNTER += 1;
        if LIVENESS_COUNTER % super::liveness::LIVENESS_CHECK_INTERVAL == 0 {
            let actions = super::liveness::check_liveness(current_time);
            if actions > 0 {
                // Liveness check took action - need to reschedule
                crate::arch::aarch64::sync::cpu_flags().set_need_resched();
            }
        }

        if woken > 0 {
            // A task was woken - signal that we need to reschedule
            crate::arch::aarch64::sync::cpu_flags().set_need_resched();
        }
    }
}

// ============================================================================
// Debugging
// ============================================================================

/// Print scheduler state for debugging.
pub fn dump_state() {
    unsafe {
        let sched = task::scheduler();

        crate::kdebug!("sched", "dump_start");
        for (i, task_opt) in sched.iter_tasks() {
            if let Some(task) = task_opt {
                crate::kdebug!("sched", "task"; slot = i as u64, pid = task.id as u64, name = task.name_str());
            }
        }
    }
}
