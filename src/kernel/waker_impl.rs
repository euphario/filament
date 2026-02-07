//! Kernel Waker Implementation
//!
//! Implements the `Waker` trait by delegating to the scheduler.
//! This provides a clean trait boundary for waking tasks.
//!
//! # Thread Safety
//!
//! Uses try_scheduler() to avoid deadlocks when called from contexts that
//! already hold the scheduler lock (e.g., cleanup paths, reap_terminated).
//! If the lock is held, defers the wake via request_wake().

use crate::kernel::traits::waker::{Waker, Subscriber, WakeReason, WakeList};
use crate::kernel::task;

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
        // Use try_scheduler to avoid deadlock when called from cleanup paths
        if let Some(mut sched) = task::try_scheduler() {
            // Generation check is handled by wake_by_pid -> slot_by_pid:
            // slot_by_pid compares task.id == pid which includes the generation
            // bits, so stale PIDs will fail the lookup and won't wake anything.
            sched.wake_by_pid(sub.task_id);
        } else {
            // Lock held - defer the wake
            crate::kernel::arch::sync::cpu_flags().request_wake(sub.task_id);
        }
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
