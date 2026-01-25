//! Process Operations Trait
//!
//! This trait provides syscall-level process lifecycle operations.
//! These are the minimal operations needed for a microkernel:
//!
//! - `exit` - Terminate current process
//! - `kill` - Terminate another process (with capability check)
//! - `spawn` - Create new process from ELF
//!
//! # Design Philosophy
//!
//! In a pure microkernel, there is no `wait()` syscall. Instead:
//! - Child exit generates an IPC message to parent
//! - Parent blocks on IPC channel, not a special wait syscall
//!
//! Similarly, `yield()` and `sleep_for_ipc()` are unnecessary because:
//! - Tasks block on IPC when waiting (implicit in read/accept)
//! - Scheduler picks another task when current blocks
//! - No explicit "sleep" or "yield" needed
//!
//! NOTE: ps_info is temporarily kept for compatibility but will
//! eventually move to an init service.

use super::task::{TaskId, Capabilities};

/// Error type for process operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcessError {
    /// Process not found
    NotFound,
    /// No free process slots
    NoSlots,
    /// Invalid ELF binary
    InvalidElf,
    /// Out of memory
    OutOfMemory,
    /// Permission denied
    PermissionDenied,
    /// Invalid operation for current state
    InvalidState,
    /// No current task
    NoTask,
}

impl ProcessError {
    pub fn to_errno(self) -> i64 {
        match self {
            ProcessError::NotFound => -3,        // ESRCH
            ProcessError::NoSlots => -12,        // ENOMEM
            ProcessError::InvalidElf => -8,      // ENOEXEC
            ProcessError::OutOfMemory => -12,    // ENOMEM
            ProcessError::PermissionDenied => -1, // EPERM
            ProcessError::InvalidState => -22,   // EINVAL
            ProcessError::NoTask => -3,          // ESRCH
        }
    }
}

/// Trait for process lifecycle operations
///
/// These are the minimal process operations for a microkernel.
/// Only three operations: exit, kill, spawn.
///
/// # Contract
///
/// 1. exit() never returns
/// 2. kill() requires KILL capability for non-children
/// 3. spawn() creates process with subset of caller's capabilities
pub trait ProcessOps: Send + Sync {
    /// Exit current process
    ///
    /// Terminates the current task with the given exit code.
    /// Notifies parent via IPC.
    /// Never returns.
    ///
    /// # Arguments
    /// * `task_id` - Current task (must match actual current task)
    /// * `code` - Exit code
    fn exit(&self, task_id: TaskId, code: i32) -> !;

    /// Kill another process
    ///
    /// Terminates target process and notifies its parent.
    ///
    /// # Security
    /// - Caller can always kill self
    /// - Caller can kill children
    /// - Caller needs KILL capability for others
    /// - Cannot kill init (PID 1)
    ///
    /// # Arguments
    /// * `killer` - Calling task
    /// * `target` - Task to kill
    fn kill(&self, killer: TaskId, target: TaskId) -> Result<(), ProcessError>;

    /// Spawn a new process from ELF
    ///
    /// Creates a new process from the given ELF binary.
    /// Child inherits subset of parent's capabilities.
    ///
    /// # Arguments
    /// * `parent` - Parent task ID
    /// * `elf_path` - Path to ELF in ramfs
    /// * `caps` - Capabilities to grant (intersected with parent's)
    ///
    /// # Returns
    /// New process ID on success
    fn spawn(
        &self,
        parent: TaskId,
        elf_path: &str,
        caps: Capabilities,
    ) -> Result<TaskId, ProcessError>;
}

// ============================================================================
// Mock Implementation for Testing
// ============================================================================

/// Mock ProcessOps implementation for unit testing
///
/// Note: exit() cannot be easily mocked because it returns `!`.
/// Tests should avoid calling exit() on the mock, or expect a panic.
#[cfg(any(test, feature = "mock"))]
pub struct MockProcessOps {
    kill_result: Result<(), ProcessError>,
    spawn_result: Result<TaskId, ProcessError>,
}

#[cfg(any(test, feature = "mock"))]
impl MockProcessOps {
    /// Create a new mock with default (success) results
    pub const fn new() -> Self {
        Self {
            kill_result: Ok(()),
            spawn_result: Ok(1), // Default spawned PID
        }
    }

    /// Configure kill() result
    pub const fn with_kill_result(mut self, result: Result<(), ProcessError>) -> Self {
        self.kill_result = result;
        self
    }

    /// Configure spawn() result
    pub const fn with_spawn_result(mut self, result: Result<TaskId, ProcessError>) -> Self {
        self.spawn_result = result;
        self
    }
}

#[cfg(any(test, feature = "mock"))]
impl ProcessOps for MockProcessOps {
    fn exit(&self, _task_id: TaskId, _code: i32) -> ! {
        // Cannot return from exit - panic in mock
        panic!("MockProcessOps::exit() called - tests should not call exit()")
    }

    fn kill(&self, _killer: TaskId, _target: TaskId) -> Result<(), ProcessError> {
        self.kill_result
    }

    fn spawn(&self, _parent: TaskId, _elf_path: &str, _caps: Capabilities) -> Result<TaskId, ProcessError> {
        self.spawn_result
    }
}
