//! Task Lifecycle Operations
//!
//! This module provides clean, centralized functions for task lifecycle events.
//! These are the ONLY places where task state transitions and their side effects happen.
//!
//! # Design Principles
//!
//! 1. **Single responsibility**: Each function handles one lifecycle event completely
//! 2. **State machine enforcement**: All transitions go through TaskState methods
//! 3. **Side effects are explicit**: Notifications, waking, cleanup are clearly documented
//! 4. **Syscalls are thin wrappers**: sys_exit just calls lifecycle::exit()
//!
//! # Functions
//!
//! - `exit(task_id, code)` - Voluntary exit, notifies parent, starts cleanup
//! - `kill(task_id, killer_id)` - Forced termination (SIGKILL equivalent)
//! - `wait_child(parent_id, pid, flags)` - Wait for child exit (sys_wait)
//! - `wake(task_id)` - Wake a blocked task
//! - `reap(task_id)` - Final cleanup, free slot

use crate::kinfo;
use super::state::SleepReason;
use super::{TaskId, Scheduler, MAX_TASKS};
use crate::kernel::ipc::{waker, traits::WakeReason};

/// Result of wait_child operation
#[derive(Debug, Clone, Copy)]
pub enum WaitResult {
    /// Child has exited with given code
    Exited { pid: TaskId, code: i32 },
    /// No child ready, would block (caller should sleep)
    WouldBlock,
    /// No children exist
    NoChildren,
    /// Specified PID is not a child
    NotChild,
}

/// Error from lifecycle operations
#[derive(Debug, Clone, Copy)]
pub enum LifecycleError {
    /// Task not found
    NotFound,
    /// Invalid state for this operation
    InvalidState,
    /// Permission denied
    PermissionDenied,
    /// No children
    NoChildren,
}

// ============================================================================
// Core Lifecycle Functions
// ============================================================================

/// Exit a task voluntarily
///
/// Called when a task calls exit() syscall.
///
/// # Effects
/// 1. Task state → Exiting
/// 2. Clear task's timers
/// 3. Notify parent via ChildExitObject handles
/// 4. Wake parent if blocked (for sys_wait compatibility)
/// 5. Task will be reaped later by scheduler
///
/// # Safety
/// Must be called with scheduler lock held (or from task context)
pub fn exit(sched: &mut Scheduler, task_id: TaskId, code: i32) -> Result<(), LifecycleError> {
    let slot = sched.slot_by_pid(task_id).ok_or(LifecycleError::NotFound)?;

    let parent_id = {
        let task = sched.tasks[slot].as_mut().ok_or(LifecycleError::NotFound)?;

        // Transition state via state machine
        if let Err(_e) = task.set_exiting(code) {
            // Already terminated? That's ok, just return
            if task.is_terminated() {
                return Ok(());
            }
            return Err(LifecycleError::InvalidState);
        }


        // Clear is_init flag so new devd can be init
        task.is_init = false;

        kinfo!("lifecycle", "exit"; pid = task_id, code = code as i64);

        task.parent_id
    };

    // Notify parent (outside of task borrow)
    if parent_id != 0 {
        notify_parent_of_exit(sched, parent_id, task_id, code);
    }

    Ok(())
}

/// Kill a task forcefully (SIGKILL)
///
/// Called when another task sends kill signal.
///
/// # Effects
/// Same as exit(), but:
/// - Checks kill permission
/// - Uses exit code -9 (SIGKILL)
/// - Can kill blocked tasks
pub fn kill(
    sched: &mut Scheduler,
    task_id: TaskId,
    killer_id: TaskId,
) -> Result<(), LifecycleError> {
    let slot = sched.slot_by_pid(task_id).ok_or(LifecycleError::NotFound)?;

    // Check permission
    let (parent_id, allowed, was_init) = {
        let task = sched.tasks[slot].as_ref().ok_or(LifecycleError::NotFound)?;

        // Check allowlist
        let is_parent = task.parent_id == killer_id;
        let is_self = task_id == killer_id;
        let in_allowlist = task.signal_allowlist_count == 0
            || task.signal_allowlist[..task.signal_allowlist_count]
                .iter()
                .any(|&pid| pid == killer_id);

        // Check capabilities (killer needs CAP_KILL for non-self, non-child)
        let has_cap_kill = if let Some(killer_slot) = sched.slot_by_pid(killer_id) {
            sched.tasks[killer_slot]
                .as_ref()
                .map(|t| t.has_capability(crate::kernel::caps::Capabilities::KILL))
                .unwrap_or(false)
        } else {
            false
        };

        let allowed = is_self || is_parent || in_allowlist || has_cap_kill;
        (task.parent_id, allowed, task.is_init)
    };

    if !allowed {
        return Err(LifecycleError::PermissionDenied);
    }

    // Now do the actual kill
    {
        let task = sched.tasks[slot].as_mut().ok_or(LifecycleError::NotFound)?;

        if let Err(_e) = task.set_exiting(-9) {
            if task.is_terminated() {
                return Ok(());
            }
            return Err(LifecycleError::InvalidState);
        }


        task.is_init = false;

        kinfo!("lifecycle", "kill"; pid = task_id, killer = killer_id);
    }

    // Notify parent
    if parent_id != 0 && parent_id != killer_id {
        notify_parent_of_exit(sched, parent_id, task_id, -9);
    }

    // If this was the init process (devd), trigger recovery
    if was_init {
        kinfo!("lifecycle", "init_killed"; pid = task_id, triggering = "recovery");
        crate::DEVD_LIVENESS_KILLED.store(true, core::sync::atomic::Ordering::SeqCst);
    }

    Ok(())
}

/// Wait for a child to exit
///
/// Called from sys_wait syscall.
///
/// # Arguments
/// - `parent_id`: The waiting parent task
/// - `target_pid`: Specific child (-1 = any child, >0 = specific PID)
/// - `no_hang`: If true, return WouldBlock instead of blocking
///
/// # Returns
/// - `Exited { pid, code }`: Child has exited, slot has been reaped
/// - `WouldBlock`: No child ready, caller should block
/// - `NoChildren`: Parent has no children
/// - `NotChild`: Specified PID is not a child of parent
pub fn wait_child(
    sched: &mut Scheduler,
    parent_id: TaskId,
    target_pid: i32,
    no_hang: bool,
) -> WaitResult {
    let parent_slot = match sched.slot_by_pid(parent_id) {
        Some(s) => s,
        None => return WaitResult::NoChildren,
    };

    // Check if parent has children
    let has_children = sched.tasks[parent_slot]
        .as_ref()
        .map(|t| t.has_children())
        .unwrap_or(false);

    if !has_children {
        return WaitResult::NoChildren;
    }

    // Look for exited child
    let mut found: Option<(usize, TaskId, i32)> = None;

    for slot in 0..MAX_TASKS {
        if let Some(ref task) = sched.tasks[slot] {
            // Must be child of parent
            if task.parent_id != parent_id {
                continue;
            }

            // If specific PID requested, must match
            if target_pid > 0 && task.id != target_pid as u32 {
                continue;
            }

            // Check if terminated
            if let Some(code) = task.state().exit_code() {
                found = Some((slot, task.id, code));
                break;
            }
        }
    }

    if let Some((child_slot, child_pid, exit_code)) = found {
        // Reap the child
        reap_child(sched, parent_slot, child_slot, child_pid);

        return WaitResult::Exited {
            pid: child_pid,
            code: exit_code,
        };
    }

    // No exited child found
    if no_hang {
        WaitResult::WouldBlock
    } else {
        WaitResult::WouldBlock // Caller will block
    }
}

/// Wake a blocked task
///
/// Called when an event arrives that a task is waiting for.
///
/// # Effects
/// - Task state: Sleeping/Waiting → Ready
/// - Task is added back to run queue
/// - Liveness state reset to Normal
///
/// # Returns
/// - true if task was actually woken (was blocked)
/// - false if task was not blocked
pub fn wake(sched: &mut Scheduler, task_id: TaskId) -> bool {
    let slot = match sched.slot_by_pid(task_id) {
        Some(s) => s,
        None => return false,
    };

    let task = match sched.tasks[slot].as_mut() {
        Some(t) => t,
        None => return false,
    };

    // Only wake if blocked
    if !task.is_blocked() {
        return false;
    }

    // Transition to Ready via state machine
    if task.wake().is_err() {
        return false;
    }

    // Reset liveness state
    task.liveness_state = crate::kernel::liveness::LivenessState::Normal;

    true
}

/// Sleep the current task (no deadline)
///
/// Called when a task blocks waiting for an event with no timeout.
///
/// # Effects
/// - Task state: Running → Sleeping
/// - Task is removed from CPU (scheduler will pick next)
/// - Liveness will monitor this task
pub fn sleep_current(sched: &mut Scheduler, slot: usize, reason: SleepReason) -> Result<(), LifecycleError> {
    let task = sched.tasks[slot].as_mut().ok_or(LifecycleError::NotFound)?;

    task.set_sleeping(reason).map_err(|_| LifecycleError::InvalidState)
}

/// Wait until deadline (with timeout)
///
/// Called when a task blocks with a deadline (timer, timed wait).
///
/// # Effects
/// - Task state: Running → Waiting { deadline }
/// - Task is removed from CPU (scheduler will pick next)
/// - Scheduler will wake task when deadline passes
///
/// # Arguments
/// - `slot`: Task slot in scheduler
/// - `reason`: Why the task is waiting
/// - `deadline`: Wake deadline in timer ticks
pub fn wait_until(
    sched: &mut Scheduler,
    slot: usize,
    reason: super::state::WaitReason,
    deadline: u64,
) -> Result<(), LifecycleError> {
    let task = sched.tasks[slot].as_mut().ok_or(LifecycleError::NotFound)?;

    task.set_waiting(reason, deadline).map_err(|_| LifecycleError::InvalidState)
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Notify parent that a child has exited
///
/// This handles both:
/// 1. ProcessObject handle notifications (for unified syscall waiting)
/// 2. Direct wake of blocked parent (for sys_wait compatibility)
fn notify_parent_of_exit(sched: &mut Scheduler, parent_id: TaskId, child_pid: TaskId, code: i32) {
    let parent_slot = match sched.slot_by_pid(parent_id) {
        Some(s) => s,
        None => return,
    };

    // Check if parent is blocked
    let should_wake_parent = sched.task(parent_slot)
        .map(|p| p.is_blocked())
        .unwrap_or(false);

    // Notify via ObjectService (acquires ObjectService.tables lock, not scheduler)
    let wake_list = crate::kernel::object_service::object_service().notify_child_exit(
        parent_id, child_pid, code,
    );

    // Wake handle subscribers
    waker::wake(&wake_list, WakeReason::ChildExit);

    // Direct wake of parent if blocked (outside borrow)
    if should_wake_parent {
        if let Some(ref mut parent) = sched.tasks[parent_slot] {
            crate::transition_or_log!(parent, wake);
            parent.liveness_state = crate::kernel::liveness::LivenessState::Normal;
        }
    }
}

/// Reap a terminated child - remove from parent's list, free slot
fn reap_child(sched: &mut Scheduler, parent_slot: usize, child_slot: usize, child_pid: TaskId) {
    // Remove from parent's child list
    if let Some(ref mut parent) = sched.tasks[parent_slot] {
        parent.remove_child(child_pid);
    }

    // Bump generation and free the slot
    sched.bump_generation(child_slot);
    sched.tasks[child_slot] = None;
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    // Tests would go here but require scheduler infrastructure
}
