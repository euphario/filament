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
