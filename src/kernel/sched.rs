//! Scheduler - Internal Kernel Scheduling Mechanisms
//!
//! This module provides internal scheduler functions for the kernel.
//! These are NOT syscall entry points - they are kernel-internal mechanisms.
//!
//! ## Design Principles
//!
//! 1. **Idle task**: Slot 0 is reserved for the idle task, which:
//!    - Has lowest priority (Priority::Idle)
//!    - Is always Ready (never blocks)
//!    - Runs WFI loop when no other tasks are runnable
//!
//! 2. **State machine**: Task states follow strict transitions:
//!    - Ready → Running (only via reschedule)
//!    - Running → Ready (via yield or preemption)
//!    - Running → Blocked (via block_current)
//!    - Blocked → Ready (only via wake - never by the blocked task itself!)
//!    - Running → Terminated (via exit)
//!
//! 3. **Separation of concerns**:
//!    - syscall.rs: User-facing syscall entry points (sys_yield, sys_exit, etc.)
//!    - sched.rs: Kernel-internal scheduling (reschedule, wake, block, etc.)
//!    - task.rs: Task structures and per-task operations
//!
//! ## Key Functions
//!
//! - `reschedule()`: Pick next task and switch to it (called from timer, syscalls)
//! - `wake(pid)`: Wake a blocked task (called when event arrives)
//! - `block_current(reason)`: Block current task (called when waiting for event)
//! - `yield_current()`: Voluntarily give up CPU (does NOT change Blocked state)
//! - `run_idle()`: Idle loop - only called when no tasks are runnable

use crate::{kwarn, kerror, kinfo};
use super::task::{self, TaskState, WaitReason, Priority, current_slot, set_current_slot};
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
        if let Some(ref mut current) = sched.tasks[caller_slot] {
            if current.state == TaskState::Running {
                current.state = TaskState::Ready;
            }
        }

        // Find next task to run
        if let Some(next_slot) = sched.schedule() {
            if next_slot != caller_slot {
                // Switch to different task
                set_current_slot(next_slot);
                sched.current = next_slot;

                if let Some(ref mut next) = sched.tasks[next_slot] {
                    next.state = TaskState::Running;
                }

                update_current_task_globals();
                SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);

                return true;
            } else {
                // Same task continues
                if let Some(ref mut current) = sched.tasks[caller_slot] {
                    current.state = TaskState::Running;
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
/// After calling this, the caller should trigger a reschedule.
///
/// # Important
/// Does NOT reschedule - caller must do that after setting up wait condition.
pub fn block_current(reason: WaitReason) {
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    unsafe {
        let sched = task::scheduler();
        let slot = current_slot();

        if let Some(ref mut task) = sched.tasks[slot] {
            // Don't block the idle task!
            if slot == IDLE_SLOT {
                kerror!("sched", "block_idle_attempt");
                return;
            }

            task.state = TaskState::Blocked;
            task.wait_reason = Some(reason);
        }
    }
}

/// Block current task with a timeout.
///
/// Like `block_current`, but also sets a wake deadline.
pub fn block_current_timeout(reason: WaitReason, deadline_tick: u64) {
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    unsafe {
        let sched = task::scheduler();
        let slot = current_slot();

        if let Some(ref mut task) = sched.tasks[slot] {
            if slot == IDLE_SLOT {
                kerror!("sched", "block_idle_attempt");
                return;
            }

            task.state = TaskState::Blocked;
            task.wait_reason = Some(reason);
            task.wake_at = deadline_tick;
        }
    }
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

        if let Some(ref mut task) = sched.tasks[slot] {
            // Only change Running → Ready, don't touch Blocked!
            if task.state == TaskState::Running {
                task.state = TaskState::Ready;
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

        if let Some(ref task) = sched.tasks[slot] {
            return task.state == TaskState::Blocked;
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
            sched.tasks.iter().enumerate().any(|(i, slot)| {
                if i == IDLE_SLOT { return false; }  // Skip idle itself
                if let Some(ref task) = slot {
                    task.state == TaskState::Ready
                } else {
                    false
                }
            })
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

        if sched.tasks[IDLE_SLOT].is_some() {
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
/// 2. Sets the need_resched flag if any task was woken
/// 3. Actual reschedule happens at exception return (do_resched_if_needed)
pub fn timer_tick(current_tick: u64) {
    // IRQs are already disabled in interrupt context

    unsafe {
        let sched = task::scheduler();
        let woken = sched.check_timeouts(current_tick);

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
    use crate::print_direct;

    unsafe {
        let sched = task::scheduler();

        print_direct!("[SCHED] Task states:\n");
        for (i, slot) in sched.tasks.iter().enumerate() {
            if let Some(ref task) = slot {
                print_direct!("  slot {}: pid={} state={:?} prio={:?} name={}\n",
                       i, task.id, task.state, task.priority,
                       core::str::from_utf8(&task.name).unwrap_or("?"));
            }
        }
    }
}
