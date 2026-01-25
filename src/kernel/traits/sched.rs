//! Scheduler Trait Definitions
//!
//! These traits define the contract for scheduler operations.
//! This enables mock implementations for testing without hardware.
//!
//! # Design Philosophy
//!
//! The scheduler trait provides the minimum operations needed by other
//! kernel subsystems (IPC, events, etc.) without exposing internal
//! implementation details like task slots or context switching.
//!
//! # Thread Safety
//!
//! All operations are thread-safe. Implementations use internal locking.

/// Task ID (matches kernel's TaskId type)
pub type TaskId = u32;

// ============================================================================
// Task State Information
// ============================================================================

/// Simplified task state for trait boundary
///
/// This is a coarse-grained view of task state, suitable for decisions
/// about waking, blocking, and scheduling.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TaskStateInfo {
    /// Task is ready to run
    Ready,
    /// Task is currently running (on some CPU)
    Running,
    /// Task is sleeping (no deadline, waiting for explicit wake)
    Sleeping,
    /// Task is waiting with a deadline
    Waiting { deadline: u64 },
    /// Task is terminated (exiting, dying, or dead)
    Terminated,
}

impl TaskStateInfo {
    /// Check if task can be woken
    pub fn can_wake(&self) -> bool {
        matches!(self, TaskStateInfo::Sleeping | TaskStateInfo::Waiting { .. })
    }

    /// Check if task is runnable
    pub fn is_runnable(&self) -> bool {
        matches!(self, TaskStateInfo::Ready | TaskStateInfo::Running)
    }

    /// Check if task is blocked
    pub fn is_blocked(&self) -> bool {
        matches!(self, TaskStateInfo::Sleeping | TaskStateInfo::Waiting { .. })
    }
}

/// Sleep reason for trait boundary
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SleepReason {
    /// Waiting in event loop
    EventLoop,
    /// Waiting for hardware IRQ
    Irq,
    /// Waiting for child process
    ChildWait,
    /// Other/unspecified reason
    Other,
}

/// Wait reason for trait boundary (has deadline)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaitReason {
    /// IPC call with timeout
    IpcCall,
    /// Timer expired
    Timer,
    /// Shmem notification
    ShmemNotify,
    /// Other/unspecified reason
    Other,
}

// ============================================================================
// Scheduler Backend Trait
// ============================================================================

/// Trait for scheduler operations
///
/// This is the main entry point for scheduler interactions.
/// The kernel holds a single instance implementing this trait.
///
/// # Contract
///
/// 1. All operations are thread-safe (internal locking)
/// 2. `wake()` only succeeds if task is blocked
/// 3. `sleep_current()` / `wait_current()` transitions current task
/// 4. Operations return meaningful results (don't silently fail)
///
/// # Why Not Individual Task Traits?
///
/// Unlike IpcBackend which operates on specific channels, scheduler
/// operations are global and affect system-wide state. Individual
/// task objects would need complex lifetime management.
pub trait SchedulerBackend: Send + Sync {
    // ========================================================================
    // Current Task Operations
    // ========================================================================

    /// Get the current task's ID
    ///
    /// Returns None if no task is running (kernel init context).
    fn current_task_id(&self) -> Option<TaskId>;

    /// Get current task's state
    fn current_task_state(&self) -> Option<TaskStateInfo>;

    // ========================================================================
    // Task Lookup
    // ========================================================================

    /// Get task state by ID
    ///
    /// Returns None if task doesn't exist or ID is stale.
    fn task_state(&self, task_id: TaskId) -> Option<TaskStateInfo>;

    /// Check if a task exists (and ID is not stale)
    fn task_exists(&self, task_id: TaskId) -> bool {
        self.task_state(task_id).is_some()
    }

    // ========================================================================
    // Wake Operations
    // ========================================================================

    /// Wake a blocked task
    ///
    /// # Returns
    /// - `true` if task was woken (was blocked)
    /// - `false` if task not found, stale ID, or not blocked
    fn wake(&self, task_id: TaskId) -> bool;

    /// Wake multiple tasks
    ///
    /// Returns count of tasks actually woken.
    fn wake_all(&self, task_ids: &[TaskId]) -> usize {
        task_ids.iter().filter(|&&id| self.wake(id)).count()
    }

    // ========================================================================
    // Block Operations
    // ========================================================================

    /// Put current task to sleep (no deadline)
    ///
    /// Task will stay sleeping until explicitly woken via `wake()`.
    ///
    /// # Returns
    /// - `true` if state transition succeeded
    /// - `false` if invalid (e.g., trying to sleep the idle task)
    fn sleep_current(&self, reason: SleepReason) -> bool;

    /// Put current task into waiting state with deadline
    ///
    /// Task will wake when either:
    /// - Explicitly woken via `wake()`
    /// - Deadline passes (checked by timeout handler)
    ///
    /// # Returns
    /// - `true` if state transition succeeded
    /// - `false` if invalid
    fn wait_current(&self, reason: WaitReason, deadline: u64) -> bool;

    // ========================================================================
    // Yield Operations
    // ========================================================================

    /// Voluntarily yield the CPU
    ///
    /// Current task goes to Ready state, scheduler picks next task.
    /// May return immediately if this is the highest priority task.
    fn yield_current(&self);

    /// Request a reschedule
    ///
    /// Sets a flag that will trigger rescheduling at the next safe point
    /// (syscall exit, exception return). Does not immediately switch.
    fn request_resched(&self);

    // ========================================================================
    // Timeout Support
    // ========================================================================

    /// Get current tick count
    ///
    /// Used for deadline calculations.
    fn current_tick(&self) -> u64;

    /// Register a deadline for tickless optimization
    ///
    /// Informs the scheduler of an upcoming deadline so it can
    /// optimize timer interrupt frequency.
    fn note_deadline(&self, deadline: u64);
}

// ============================================================================
// Error Types
// ============================================================================

/// Scheduler errors for trait boundary
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedError {
    /// Task not found or ID is stale
    NotFound,
    /// Invalid state transition
    InvalidTransition,
    /// Operation not permitted (e.g., blocking idle task)
    NotPermitted,
}

impl SchedError {
    pub fn to_errno(self) -> i64 {
        match self {
            SchedError::NotFound => -3,        // ESRCH
            SchedError::InvalidTransition => -22, // EINVAL
            SchedError::NotPermitted => -1,    // EPERM
        }
    }
}
