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

use crate::{kinfo, knotice, kdebug};
use super::{TaskId, Scheduler, MAX_TASKS};
use crate::kernel::ipc::{waker, traits::WakeReason};
use crate::kernel::process;

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

    let ptable = process::process_table();

    // Get metadata from ProcessTable
    let (parent_id, was_probed) = {
        let task = sched.tasks[slot].as_mut().ok_or(LifecycleError::NotFound)?;

        // Transition state via state machine
        if let Err(_e) = task.set_exiting(code) {
            if task.is_terminated() {
                return Ok(None);
            }
            return Err(LifecycleError::InvalidState);
        }

        // Read and clear flags in ProcessTable
        let parent_id = ptable.with(slot, |p| p.parent_id).unwrap_or(0);
        let was_probed = ptable.with_mut(slot, |p| {
            let probed = p.is_probed;
            if probed {
                p.is_probed = false;
            }
            p.is_init = false;
            probed
        }).unwrap_or(false);

        knotice!("lifecycle", "exit"; pid = task_id, code = code as i64);

        (parent_id, was_probed)
    };

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

    let ptable = process::process_table();

    // Check permission via ProcessTable
    let (parent_id, allowed, was_init) = {
        let task = sched.tasks[slot].as_ref().ok_or(LifecycleError::NotFound)?;
        let _ = task; // existence check only

        let (parent_id, is_init, signal_allowed) = ptable.with(slot, |p| {
            let is_parent = p.parent_id == killer_id;
            let is_self = task_id == killer_id;
            let in_allowlist = p.can_receive_signal_from(killer_id);
            (p.parent_id, p.is_init, is_self || is_parent || in_allowlist)
        }).unwrap_or((0, false, false));

        // Check capabilities (killer needs CAP_KILL for non-self, non-child)
        let has_cap_kill = if let Some(killer_slot) = sched.slot_by_pid(killer_id) {
            ptable.with(killer_slot, |p| {
                p.has_capability(crate::kernel::caps::Capabilities::KILL)
            }).unwrap_or(false)
        } else {
            false
        };

        (parent_id, signal_allowed || has_cap_kill, is_init)
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

        ptable.with_mut(slot, |p| { p.is_init = false; });

        kinfo!("lifecycle", "kill"; pid = task_id, killer = killer_id);
    }

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

    // Check if parent has children (via ProcessTable)
    let has_children = process::process_table()
        .with(parent_slot, |p| p.has_children())
        .unwrap_or(false);

    if !has_children {
        return WaitResult::NoChildren;
    }

    // Look for exited child — need to check parent_id via ProcessTable
    let mut found: Option<(usize, TaskId, i32)> = None;

    for slot in 0..MAX_TASKS {
        if let Some(ref task) = sched.tasks[slot] {
            // Check parent_id via ProcessTable
            let is_child = process::process_table()
                .with(slot, |p| p.parent_id == parent_id)
                .unwrap_or(false);

            if !is_child {
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
fn reap_child(_sched: &mut Scheduler, parent_slot: usize, child_slot: usize, child_pid: TaskId) {
    use crate::kernel::ipc::{waker, traits::WakeReason};
    use crate::kernel::task::tcb::CleanupPhase;

    let ptable = process::process_table();

    // Remove from parent's child list (via ProcessTable)
    ptable.with_mut(parent_slot, |p| {
        p.remove_child(child_pid);
    });

    // Check cleanup progress (via ProcessTable)
    let cleanup_phase = ptable.with(child_slot, |p| p.cleanup_phase)
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

    // Detach from parent and set grace period (via ProcessTable)
    ptable.with_mut(child_slot, |p| {
        p.parent_id = 0;
        if matches!(p.cleanup_phase, CleanupPhase::None | CleanupPhase::Phase1Enqueued) {
            p.cleanup_phase = CleanupPhase::GracePeriod {
                until: crate::platform::current::timer::deadline_ns(100_000_000),
            };
        }
    });
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

/// Complete probed exit: spawn busd then devd
/// Must be called OUTSIDE the scheduler lock.
pub fn complete_probed_exit() {
    use crate::kernel::elf;
    use crate::kernel::caps::Capabilities;

    if !PROBED_EXITED.swap(false, core::sync::atomic::Ordering::SeqCst) {
        return;
    }

    // Spawn busd first — it registers the bus: port that devd connects to
    spawn_busd();

    match elf::spawn_from_path("bin/devd2", 0, Capabilities::from_bits(0)) {
        Ok((task_id, slot)) => {
            // Set capabilities via ProcessTable
            process::process_table().with_mut(slot, |p| {
                p.set_capabilities(Capabilities::ALL);
                p.is_init = true;
            });
            // Set priority via scheduler
            super::with_scheduler(|sched| {
                if let Some(task) = sched.task_mut(slot) {
                    task.set_priority(super::Priority::Critical);
                }
            });
            kinfo!("lifecycle", "spawned"; binary = "devd2", pid = task_id as u64);
        }
        Err(_e) => {
            crate::print_str_uart("FATAL: probed_exit: cannot spawn devd\r\n");
            loop {
                unsafe { core::arch::asm!("wfe"); }
            }
        }
    }
}

/// Spawn the bus daemon (busd) — message switch for all IPC routing.
/// busd registers the `bus:` port. Must be spawned before devd.
pub fn spawn_busd() {
    use crate::kernel::elf;
    use crate::kernel::caps::Capabilities;

    match elf::spawn_from_path("bin/busd", 0, Capabilities::from_bits(0)) {
        Ok((task_id, slot)) => {
            // Set capabilities via ProcessTable
            process::process_table().with_mut(slot, |p| {
                p.set_capabilities(Capabilities::ALL);
            });
            // Set priority via scheduler
            super::with_scheduler(|sched| {
                if let Some(task) = sched.task_mut(slot) {
                    task.set_priority(super::Priority::Critical);
                }
            });
            kinfo!("lifecycle", "spawned"; binary = "busd", pid = task_id as u64);
        }
        Err(_e) => {
            crate::print_str_uart("WARN: failed to spawn busd\r\n");
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
