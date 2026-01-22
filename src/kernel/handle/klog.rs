//! Klog Handle Object
//!
//! Handle for waiting on kernel log availability.
//! Used by logd to efficiently wait for logs instead of polling.

use super::traits::{Waitable, Closable, WaitResult, WaitFilter, CloseAction};
use crate::kernel::task::TaskId;

/// Klog object for handle system
///
/// Allows waiting on kernel log buffer availability.
/// When logs are available, poll() returns Readable.
#[derive(Clone, Copy, Debug)]
pub struct KlogObject {
    /// Task to wake when logs become available (0 = none)
    waker_task: TaskId,
}

impl KlogObject {
    /// Create a new klog handle object
    pub const fn new() -> Self {
        Self {
            waker_task: 0,
        }
    }
}

impl Waitable for KlogObject {
    fn poll(&self, filter: WaitFilter) -> Option<WaitResult> {
        match filter {
            WaitFilter::Readable => {
                // Check if kernel log buffer has data
                if crate::klog::has_pending() {
                    Some(WaitResult::new(
                        super::Handle::INVALID,
                        WaitFilter::Readable,
                        1, // At least one record available
                    ))
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn register_waker(&mut self, task_id: TaskId) {
        self.waker_task = task_id;
        // Klog waking is handled by broadcast_event in klog.rs
        // When logs are written, Event::klog_ready() wakes subscribers
    }

    fn unregister_waker(&mut self, _task_id: TaskId) {
        self.waker_task = 0;
    }
}

impl Closable for KlogObject {
    fn close(&self, _owner_pid: u32) -> CloseAction {
        // Klog handles don't need special cleanup
        CloseAction::None
    }
}

impl Default for KlogObject {
    fn default() -> Self {
        Self::new()
    }
}
