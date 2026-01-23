//! Blocking Operations Support
//!
//! This module provides the infrastructure for blocking syscalls.
//! The key pattern is:
//!
//! ```text
//! loop {
//!     match try_operation() {
//!         Success(n) => return Ok(n),
//!         Closed => return Err(Closed),
//!         NeedBlock => {
//!             subscribe(task, handle);
//!             sleep_current(SleepReason::Ipc);
//!             reschedule();
//!             // Loop back and retry
//!         }
//!     }
//! }
//! ```
//!
//! # Key Concepts
//!
//! - **subscribe()** - Register task to be woken when object has data
//! - **sleep_current()** - Put current task to sleep (Sleeping state)
//! - **reschedule()** - Switch to another task
//! - **wake()** - Wake a sleeping task (from IPC/timer/etc)

use crate::kernel::task::TaskId;
use crate::kernel::task::state::SleepReason;
use crate::kernel::sched;

// ============================================================================
// Blocking Context
// ============================================================================

/// Context for a potentially blocking operation
///
/// Tracks whether the current operation has subscribed for wakeup
/// and handles the sleep/wake cycle.
pub struct BlockingContext {
    /// Task that may block
    task_id: TaskId,
    /// Whether we've subscribed for wakeup
    subscribed: bool,
}

impl BlockingContext {
    /// Create a new blocking context for the current task
    pub fn new(task_id: TaskId) -> Self {
        Self {
            task_id,
            subscribed: false,
        }
    }

    /// Get the task ID
    pub fn task_id(&self) -> TaskId {
        self.task_id
    }

    /// Mark that we've subscribed for wakeup
    pub fn mark_subscribed(&mut self) {
        self.subscribed = true;
    }

    /// Check if subscribed
    pub fn is_subscribed(&self) -> bool {
        self.subscribed
    }

    /// Block the current task until woken
    ///
    /// This puts the task to sleep and reschedules. When woken,
    /// returns to allow retrying the operation.
    pub fn block(&self, reason: SleepReason) {
        // Put current task to Sleeping state
        if !sched::sleep_current(reason) {
            // Failed to sleep (task not running?) - shouldn't happen
            return;
        }

        // Switch to another task
        sched::reschedule();

        // When we get here, we've been woken up
        // The caller will loop back and retry the operation
    }
}

// ============================================================================
// Sleep Reasons for Object Operations
// ============================================================================

/// Standard sleep reasons for object operations
pub mod reasons {
    use crate::kernel::task::state::SleepReason;

    /// Waiting for data on a channel/port/stdin
    pub const IPC: SleepReason = SleepReason::Ipc;

    /// Waiting for mux events / event loop
    pub const EVENT_LOOP: SleepReason = SleepReason::EventLoop;
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Create a blocking context for the current task
pub fn blocking_context() -> Option<BlockingContext> {
    // Get current task ID from scheduler
    crate::kernel::task::with_scheduler(|sched| {
        sched.current_task().map(|t| BlockingContext::new(t.id))
    })
}

/// Wake a task that was blocked on an object
///
/// This is called by ObjectService when data becomes available
/// on an object that a task is waiting for.
pub fn wake_task(task_id: TaskId) {
    sched::wake(task_id);
}
