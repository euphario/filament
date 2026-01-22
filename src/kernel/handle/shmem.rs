//! Shared Memory Handle Object
//!
//! Handle for monitoring shared memory regions.
//! Used by consoled to detect when the shell (shmem owner) dies.

use super::traits::{Waitable, Closable, WaitResult, WaitFilter, CloseAction};
use crate::kernel::task::TaskId;

/// Shmem object for handle system
///
/// Allows monitoring a shared memory region for invalidation.
/// When the shmem owner dies, poll() returns Closed.
#[derive(Clone, Copy, Debug)]
pub struct ShmemObject {
    /// Shared memory ID to monitor
    pub shmem_id: u32,

    /// Task to wake when shmem state changes (0 = none)
    waker_task: TaskId,
}

impl ShmemObject {
    /// Create a new shmem handle for a specific region
    pub const fn new(shmem_id: u32) -> Self {
        Self {
            shmem_id,
            waker_task: 0,
        }
    }
}

impl Waitable for ShmemObject {
    fn poll(&self, filter: WaitFilter) -> Option<WaitResult> {
        match filter {
            WaitFilter::Closed | WaitFilter::Error => {
                // Check if there's a ShmemInvalid event for this shmem_id
                // This happens when the shmem owner dies
                unsafe {
                    let sched = crate::kernel::task::scheduler();
                    let slot = crate::kernel::task::current_slot();

                    if let Some(ref task) = sched.tasks[slot] {
                        // Check if there's a ShmemInvalid event in the queue
                        if let Some((found_id, _owner_pid)) = task.event_queue.peek_shmem_invalid(self.shmem_id) {
                            return Some(WaitResult::new(
                                super::Handle::INVALID,
                                WaitFilter::Closed,
                                found_id as u64,
                            ));
                        }
                    }
                }
                None
            }
            WaitFilter::Readable => {
                // Check if shmem region is in Dying state (owner exiting)
                if let Some(region) = crate::kernel::shmem::get_region(self.shmem_id) {
                    if region.state == crate::kernel::shmem::RegionState::Dying {
                        return Some(WaitResult::new(
                            super::Handle::INVALID,
                            WaitFilter::Closed, // Use Closed to signal invalidation
                            self.shmem_id as u64,
                        ));
                    }
                }
                None
            }
            _ => None,
        }
    }

    fn register_waker(&mut self, task_id: TaskId) {
        self.waker_task = task_id;
        // Shmem invalidation events are delivered via the event queue
        // when the owner dies (Event::shmem_invalid())
    }

    fn unregister_waker(&mut self, _task_id: TaskId) {
        self.waker_task = 0;
    }
}

impl Closable for ShmemObject {
    fn close(&self, _owner_pid: u32) -> CloseAction {
        // Shmem handles don't need special cleanup
        // The actual shmem region is managed separately
        CloseAction::None
    }
}

impl Default for ShmemObject {
    fn default() -> Self {
        Self::new(0)
    }
}
