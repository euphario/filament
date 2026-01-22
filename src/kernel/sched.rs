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

use crate::{kwarn, kerror, kinfo};
use super::task::{self, TaskState, Priority, current_slot, set_current_slot};
use super::task::{update_current_task_globals, SYSCALL_SWITCHED_TASK};
use core::sync::atomic::Ordering;

/// Idle task priority (lower than Low)
pub const PRIORITY_IDLE: Priority = Priority::Low;  // We'll use slot 0 check instead

/// Slot reserved for idle task
pub const IDLE_SLOT: usize = 0;

// ============================================================================
// Core Scheduling Operations
// ============================================================================

/// Reschedule: pick the next task to run and switch to it.
///
/// This is the main scheduling entry point. It:
/// 1. Finds the highest-priority Ready task
/// 2. Switches to that task (or idle if none ready)
/// 3. Updates all scheduler globals correctly
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
            if next_slot != caller_slot {
                // Switch to different task
                set_current_slot(next_slot);

                if let Some(next) = sched.task_mut(next_slot) {
                    let _ = next.set_running();
                }

                update_current_task_globals();
                SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);

                return true;
            } else {
                // Same task continues
                if let Some(current) = sched.task_mut(caller_slot) {
                    let _ = current.set_running();
                }
                return false;
            }
        }

        // No ready tasks at all - this shouldn't happen if idle task exists
        // But handle gracefully by staying on current
        kwarn!("sched", "no_ready_tasks");
        false
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

/// Run the idle loop.
///
/// This is called when no other tasks are runnable. It:
/// 1. Enables IRQs (so timer can fire)
/// 2. Waits for interrupt (WFI)
/// 3. Checks if any task is now ready
/// 4. Loops until a task is ready
///
/// # Safety
/// Only the idle task (slot 0) should call this.
pub fn run_idle() -> ! {
    loop {
        // Check if there's a ready task before WFI
        let has_ready = unsafe {
            let sched = task::scheduler();
            sched.has_ready_task_besides(IDLE_SLOT)
        };

        if has_ready {
            // Found a ready task - reschedule
            if reschedule() {
                // Switched to another task - when we come back, loop again
                continue;
            }
        }

        // No ready tasks - wait for interrupt
        // CRITICAL: Enable IRQs before WFI so timer can fire!
        unsafe {
            core::arch::asm!("msr daifclr, #2");  // Enable IRQs
            core::arch::asm!("wfi");
            core::arch::asm!("msr daifset, #2");  // Disable IRQs
        }

        // Timer interrupt fired - check_timeouts may have woken a task
        // Loop back and check again
    }
}

/// Initialize the idle task in slot 0.
///
/// The idle task is a minimal kernel task that:
/// - Never blocks
/// - Has lowest priority
/// - Runs WFI when nothing else to do
///
/// # Returns
/// true if idle task was created successfully.
pub fn init_idle_task() -> bool {
    unsafe {
        let sched = task::scheduler();

        // Create idle task in slot 0
        // We don't use new_kernel because idle doesn't need a real stack
        // for context switching (it never blocks or switches normally)

        if sched.slot_occupied(IDLE_SLOT) {
            return true;
        }

        // For now, we'll create a minimal placeholder
        // The real idle loop runs in the boot context
        kinfo!("sched", "idle_init"; slot = IDLE_SLOT as u64);
        true
    }
}

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
pub fn timer_tick(current_tick: u64) {
    // IRQs are already disabled in interrupt context

    // Debug: confirm timer_tick is called
    if current_tick % 1000 == 0 {
        crate::kdebug!("sched", "timer_tick"; t = current_tick);
    }

    unsafe {
        let sched = task::scheduler();
        let woken = sched.check_timeouts(current_tick);

        // Periodically check liveness of waiting tasks
        // Only run every LIVENESS_CHECK_INTERVAL ticks to reduce overhead
        if current_tick % super::liveness::LIVENESS_CHECK_INTERVAL == 0 {
            let actions = super::liveness::check_liveness(current_tick);
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
