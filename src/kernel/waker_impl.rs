//! Kernel Waker Implementation
//!
//! Implements the `Waker` trait by delegating to `sched::wake()`.
//! This provides a clean trait boundary for waking tasks.

use crate::kernel::traits::waker::{Waker, Subscriber, WakeReason, WakeList};

/// Kernel waker implementation
///
/// A zero-sized type that implements `Waker` by delegating to the scheduler.
pub struct KernelWaker;

impl KernelWaker {
    /// Create a new kernel waker instance
    pub const fn new() -> Self {
        Self
    }
}

impl Waker for KernelWaker {
    fn wake(&self, sub: &Subscriber, _reason: WakeReason) {
        // Delegate to sched::wake() which handles try_scheduler, microtask
        // fallback, AND send_reschedule_ipi to wake idle CPUs.
        crate::kernel::sched::wake(sub.task_id);
    }

    fn wake_all(&self, list: &WakeList, reason: WakeReason) {
        for sub in list.iter() {
            self.wake(&sub, reason);
        }
    }

    fn wake_pid(&self, task_id: u32, reason: WakeReason) {
        self.wake(&Subscriber::simple(task_id), reason);
    }
}

/// Global kernel waker instance
pub static KERNEL_WAKER: KernelWaker = KernelWaker::new();

/// Get a reference to the global kernel waker
pub fn kernel_waker() -> &'static dyn Waker {
    &KERNEL_WAKER
}

// ============================================================================
// Legacy Compatibility
// ============================================================================

/// Wake subscribers from a WakeList (convenience function)
///
/// This is a wrapper around the trait method for easier migration.
pub fn wake(list: &WakeList, reason: WakeReason) {
    KERNEL_WAKER.wake_all(list, reason);
}

/// Wake a single task by PID (convenience function)
pub fn wake_pid(task_id: u32) {
    KERNEL_WAKER.wake_pid(task_id, WakeReason::Readable);
}
