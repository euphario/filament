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

use crate::{kinfo, kdebug};
use super::{TaskId, Scheduler, MAX_TASKS};
use crate::kernel::ipc::{waker, traits::WakeReason};

/// Flag set when probed exits — checked by exit handler to spawn devd
#[no_mangle]
pub static PROBED_EXITED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

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
/// Info needed for deferred parent notification (outside scheduler lock)
pub struct ExitInfo {
    pub parent_id: TaskId,
    pub child_pid: TaskId,
    pub code: i32,
}

pub fn exit(sched: &mut Scheduler, task_id: TaskId, code: i32) -> Result<Option<ExitInfo>, LifecycleError> {
    let slot = sched.slot_by_pid(task_id).ok_or(LifecycleError::NotFound)?;

    let (parent_id, was_probed) = {
        let task = sched.tasks[slot].as_mut().ok_or(LifecycleError::NotFound)?;

        // Transition state via state machine
        if let Err(_e) = task.set_exiting(code) {
            // Already terminated? That's ok, just return
            if task.is_terminated() {
                return Ok(None);
            }
            return Err(LifecycleError::InvalidState);
        }

        // Clear is_init flag so new devd can be init
        task.is_init = false;

        // Detect probed exit — trigger bus creation lock and devd spawn
        let probed = task.is_probed;
        if probed {
            task.is_probed = false;
        }

        kinfo!("lifecycle", "exit"; pid = task_id, code = code as i64);

        (task.parent_id, probed)
    };

    // NOTE: Don't wake parent here. Parent will be woken by:
    // 1. NotifyParentExit microtask → ProcessObject WaitQueue (immediate, outside lock)
    // 2. KillChildren microtask → wake_parent_if_sleeping
    // Waking here causes a lost-wake race: devd wakes, polls Mux, finds nothing,
    // goes back to sleep. Then consoled's set_port_state arrives during the
    // Running→Sleeping transition and the channel wake is lost.

    // If probed just exited, lock bus creation and spawn devd
    if was_probed {
        handle_probed_exit(sched);
    }

    // Return info for deferred ObjectService notification (must happen outside scheduler lock)
    if parent_id != 0 {
        Ok(Some(ExitInfo { parent_id, child_pid: task_id, code }))
    } else {
        Ok(None)
    }
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
///
/// Returns ExitInfo for deferred ObjectService notification (must happen outside scheduler lock).
pub fn kill(
    sched: &mut Scheduler,
    task_id: TaskId,
    killer_id: TaskId,
) -> Result<Option<ExitInfo>, LifecycleError> {
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
                return Ok(None);
            }
            return Err(LifecycleError::InvalidState);
        }

        task.is_init = false;

        kinfo!("lifecycle", "kill"; pid = task_id, killer = killer_id);
    }

    // NOTE: Don't wake parent here — same lost-wake race as exit().
    // Parent will be woken by microtask pipeline:
    // 1. NotifyParentExit → ProcessObject WaitQueue (immediate, outside lock)
    // 2. KillChildren → wake_parent_if_sleeping

    // If this was the init process (devd), trigger recovery
    if was_init {
        kinfo!("lifecycle", "init_killed"; pid = task_id, triggering = "recovery");
        crate::DEVD_LIVENESS_KILLED.store(true, core::sync::atomic::Ordering::SeqCst);
    }

    // Return info for deferred ObjectService notification (must happen outside scheduler lock)
    if parent_id != 0 && parent_id != killer_id {
        Ok(Some(ExitInfo { parent_id, child_pid: task_id, code: -9 }))
    } else {
        Ok(None)
    }
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
    _no_hang: bool,
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

    // No exited child found — caller decides whether to block or return
    WaitResult::WouldBlock
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Complete deferred parent notification after scheduler lock is released.
///
/// This performs the ObjectService notification that cannot happen under the
/// scheduler lock (would cause ABBA deadlock with devd's ObjectService→scheduler
/// lock order in read_mux_via_service → sleep_and_reschedule).
///
/// Must be called OUTSIDE the scheduler lock.
pub fn complete_exit_notification(info: ExitInfo) {
    // Notify via ObjectService (acquires per-slot ObjectService lock)
    let wake_list = crate::kernel::object_service::object_service().notify_child_exit(
        info.parent_id, info.child_pid, info.code,
    );

    // Diagnostic: log whether ProcessObject subscriber was found
    kdebug!("lifecycle", "exit_notify"; parent = info.parent_id, child = info.child_pid, subscribers = wake_list.len() as u64);

    // Wake handle subscribers (e.g. ProcessObject waiters)
    waker::wake(&wake_list, WakeReason::ChildExit);

    // Belt-and-suspenders: if no subscriber was registered on the ProcessObject
    // (e.g. parent's Mux didn't include the watcher handle, or parent was between
    // poll cycles), wake the parent directly. The parent will re-poll its Mux and
    // discover the ProcessObject is now readable via exit_code.
    if wake_list.is_empty() {
        waker::wake_pid(info.parent_id);
    }
}

/// Reap a terminated child — collect exit code, defer cleanup to microtask pipeline.
///
/// Phase 1 (IPC teardown) runs inline if the microtask pipeline hasn't started it yet.
/// Phase 2 (shmem finalize, slot free) is NEVER run here — it goes through the
/// microtask grace period so dependent tasks (e.g. shell using consoled's shmem)
/// have time to be killed and unmap before their pages are pulled out.
///
/// The child is detached (parent_id = 0) so wait_child won't find it again.
/// The timer scanner picks up the GracePeriod and enqueues Phase 2 later.
fn reap_child(sched: &mut Scheduler, parent_slot: usize, child_slot: usize, child_pid: TaskId) {
    use crate::kernel::ipc::{waker, traits::WakeReason};
    use crate::kernel::task::tcb::CleanupPhase;

    // Remove from parent's child list
    if let Some(ref mut parent) = sched.tasks[parent_slot] {
        parent.remove_child(child_pid);
    }

    // Check cleanup progress to determine what work remains
    let cleanup_phase = sched.tasks[child_slot]
        .as_ref()
        .map(|t| t.cleanup_phase)
        .unwrap_or(CleanupPhase::None);

    let needs_phase1 = matches!(cleanup_phase, CleanupPhase::None);

    // Phase 1: IPC cleanup (only if not yet started by microtask pipeline)
    if needs_phase1 {
        let ipc_peers = super::Scheduler::do_ipc_cleanup(child_pid);
        for peer in ipc_peers.iter() {
            let wake_list = crate::kernel::object_service::object_service()
                .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::CLOSED);
            waker::wake(&wake_list, WakeReason::Closed);
        }

        crate::kernel::shmem::begin_cleanup(child_pid);
    }

    // Detach from parent so wait_child won't find this task again.
    // Phase 2 (FinalCleanup + SlotReap) goes through the microtask grace period.
    if let Some(task) = sched.task_mut(child_slot) {
        task.parent_id = 0;
        if matches!(task.cleanup_phase, CleanupPhase::None | CleanupPhase::Phase1Enqueued) {
            task.cleanup_phase = CleanupPhase::GracePeriod {
                until: crate::platform::current::timer::deadline_ns(100_000_000),
            };
        }
    }
}

/// Handle probed exit: lock bus creation, set flag for deferred devd spawn
///
/// Called from lifecycle::exit() when the probed process exits.
/// At this point the scheduler lock is held by the caller, so we can't
/// spawn devd here (spawn_from_path needs the scheduler lock).
/// Instead we set PROBED_EXITED flag and handle devd spawn after lock release.
fn handle_probed_exit(_sched: &mut Scheduler) {
    use crate::kernel::bus;

    kdebug!("lifecycle", "probed_exit"; action = "lock_bus_creation");

    // 1. Lock bus creation permanently
    bus::lock_bus_creation();

    // 2. Run hardware reset sequences for buses that need it (PCIe, USB)
    while bus::continue_init() {}

    // 3. Set flag — devd will be spawned after scheduler lock is released
    PROBED_EXITED.store(true, core::sync::atomic::Ordering::SeqCst);
}

/// Complete probed exit: spawn devd
/// Must be called OUTSIDE the scheduler lock.
pub fn complete_probed_exit() {
    use crate::kernel::elf;
    use crate::kernel::caps::Capabilities;

    if !PROBED_EXITED.swap(false, core::sync::atomic::Ordering::SeqCst) {
        return;
    }

    kdebug!("lifecycle", "probed_exit"; action = "spawning_devd");
    match elf::spawn_from_path("bin/devd") {
        Ok((_task_id, slot)) => {
            super::with_scheduler(|sched| {
                if let Some(task) = sched.task_mut(slot) {
                    task.set_capabilities(Capabilities::ALL);
                    task.set_priority(super::Priority::High);
                    task.is_init = true;
                }
            });
            kinfo!("lifecycle", "devd_spawned"; slot = slot as u64);
        }
        Err(_e) => {
            // Fatal: can't spawn devd
            crate::print_str_uart("FATAL: probed_exit: cannot spawn devd\r\n");
            loop {
                unsafe { core::arch::asm!("wfe"); }
            }
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    // Tests would go here but require scheduler infrastructure
}
