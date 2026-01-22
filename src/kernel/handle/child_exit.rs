//! Child Exit Handle Object
//!
//! Handle for waiting on child process exits.
//! Used by devd to efficiently wait for driver process crashes/exits.
//!
//! # Design
//!
//! Uses the ipc subscriber pattern:
//! 1. Parent creates ChildExitObject handle via sys_handle_child_exit_create
//! 2. Parent waits on handle (registers subscriber)
//! 3. Child exits → kernel calls notify_exit() on parent's handles
//! 4. notify_exit() stores exit info, returns subscriber to wake
//! 5. Parent wakes, polls handle, gets exit info
//!
//! No event queue involved - clean state machine.

use super::traits::{Waitable, Closable, WaitResult, WaitFilter, CloseAction};
use crate::kernel::task::TaskId;
use crate::kernel::ipc::traits::Subscriber;
use crate::kernel::ipc::waker::WakeList;

/// Maximum pending exits per handle
/// (in case child exits before parent starts waiting)
const MAX_PENDING: usize = 4;

/// ChildExit object for handle system
///
/// Stores pending exits and subscriber for wake management.
/// Does NOT use the event queue - self-contained state machine.
#[derive(Clone, Debug)]
pub struct ChildExitObject {
    /// Child PID to watch (0 = any child)
    pub child_pid: u32,

    /// Pending exits: (pid, exit_code) pairs
    pending: [(u32, i32); MAX_PENDING],
    pending_count: u8,

    /// Subscriber waiting on this handle (typically just one)
    subscriber: Option<Subscriber>,
}

impl ChildExitObject {
    /// Create a new child exit handle for a specific child
    pub const fn new(child_pid: u32) -> Self {
        Self {
            child_pid,
            pending: [(0, 0); MAX_PENDING],
            pending_count: 0,
            subscriber: None,
        }
    }

    /// Create a handle to wait for any child exit
    pub const fn any() -> Self {
        Self::new(0)
    }

    /// Check if this handle matches the given child PID
    pub fn matches(&self, pid: u32) -> bool {
        self.child_pid == 0 || self.child_pid == pid
    }

    /// Notify this handle that a child exited
    ///
    /// Called by kernel when a child process exits.
    /// Stores the exit info and returns any subscriber to wake.
    ///
    /// # Returns
    /// WakeList containing the subscriber to wake (if any)
    pub fn notify_exit(&mut self, child_pid: u32, exit_code: i32) -> WakeList {
        // Check if we care about this child
        if !self.matches(child_pid) {
            return WakeList::new();
        }

        // Store the exit info
        if (self.pending_count as usize) < MAX_PENDING {
            self.pending[self.pending_count as usize] = (child_pid, exit_code);
            self.pending_count += 1;
        }
        // If queue full, drop oldest (shouldn't happen in practice)

        // Return subscriber to wake
        let mut wake_list = WakeList::new();
        if let Some(sub) = self.subscriber {
            wake_list.push(sub);
        }
        wake_list
    }

    /// Consume (pop) a pending exit after poll returns ready
    ///
    /// Called by handle_wait after poll() returns Some.
    /// This separates the check (poll) from the consume step.
    pub fn consume(&mut self) {
        if self.pending_count > 0 {
            // Shift remaining entries
            for i in 0..(self.pending_count as usize - 1) {
                self.pending[i] = self.pending[i + 1];
            }
            self.pending_count -= 1;
        }
    }
}

impl Waitable for ChildExitObject {
    fn poll(&self, filter: WaitFilter) -> Option<WaitResult> {
        match filter {
            WaitFilter::ChildExit => {
                // Check for pending exit
                if self.pending_count > 0 {
                    let (pid, code) = self.pending[0];
                    crate::kinfo!("handle", "child_exit_poll"; pid = pid, code = code);
                    return Some(WaitResult::new(
                        super::Handle::INVALID,
                        WaitFilter::ChildExit,
                        // Pack pid and code into data
                        ((pid as u64) << 32) | (code as u32 as u64),
                    ));
                }
                None
            }
            _ => None,
        }
    }

    fn register_waker(&mut self, task_id: TaskId) {
        self.subscriber = Some(Subscriber::simple(task_id));
    }

    fn unregister_waker(&mut self, _task_id: TaskId) {
        self.subscriber = None;
    }
}

impl Closable for ChildExitObject {
    fn close(&self, _owner_pid: u32) -> CloseAction {
        // ChildExit handles don't need special cleanup
        CloseAction::None
    }
}

impl Default for ChildExitObject {
    fn default() -> Self {
        Self::any()
    }
}

// ============================================================================
// Kernel API for notifying child exits
// ============================================================================

/// Notify all ChildExitObject handles in a task about a child exit
///
/// Called by liveness.rs or task exit code when a child process exits.
/// Returns WakeList of subscribers to wake.
///
/// # Safety
/// Must be called with scheduler access (typically under IrqGuard)
pub fn notify_child_exit_to_task(parent_task: &mut crate::kernel::task::Task, child_pid: u32, exit_code: i32) -> WakeList {
    let mut wake_list = WakeList::new();

    // Iterate parent's handle table looking for ChildExitObject handles
    for entry in parent_task.handle_table.entries_mut() {
        if let Some(ref mut handle_entry) = entry {
            if let super::KernelObject::ChildExit(ref mut child_exit) = handle_entry.object {
                let subs = child_exit.notify_exit(child_pid, exit_code);
                wake_list.merge(&subs);
            }
        }
    }

    wake_list
}
