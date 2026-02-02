//! Process Operations Trait
//!
//! This trait provides syscall-level process lifecycle operations.
//! These are the minimal operations needed for a microkernel:
//!
//! - `exit` - Terminate current process
//! - `kill` - Terminate another process (with capability check)
//! - `spawn` - Create new process from ELF (multiple source types)
//! - `daemonize` - Detach from parent (become a daemon)

use super::task::{TaskId, Capabilities};

/// How to locate the ELF binary for spawning
pub enum SpawnSource<'a> {
    /// Spawn by built-in ELF ID (legacy sys_spawn)
    ElfId(u32, &'a str),
    /// Spawn by ramfs path (sys_exec, default caps)
    Path(&'a str),
    /// Spawn by ramfs path with explicit caps (sys_exec_with_caps)
    PathWithCaps(&'a str, Capabilities),
    /// Spawn from ELF data in memory (sys_exec_mem)
    Memory(&'a [u8], &'a str),
}

/// Result of waiting for a child process
#[derive(Debug, Clone, Copy)]
pub enum WaitChildResult {
    /// Child exited with the given PID and exit code
    Exited { pid: TaskId, code: i32 },
    /// No child has exited yet (only returned when `no_hang` is true)
    WouldBlock,
    /// Caller has no children
    NoChildren,
    /// Target PID is not a child of the caller
    NotChild,
}

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
    fn kill(&self, killer: TaskId, target: TaskId) -> Result<(), ProcessError>;

    /// Spawn a new process from an ELF source
    ///
    /// Creates a new process from the given source (built-in ID, ramfs path,
    /// path with explicit caps, or raw ELF data in memory).
    /// Child inherits subset of parent's capabilities.
    fn spawn(&self, parent: TaskId, source: SpawnSource) -> Result<TaskId, ProcessError>;

    /// Detach from parent process (become a daemon)
    ///
    /// Removes the parent-child relationship. The parent is woken if blocked.
    fn daemonize(&self, task_id: TaskId) -> Result<(), ProcessError>;

    /// Get capabilities of a process
    ///
    /// Returns the capability bitmask for the given task, or error if not found.
    fn get_capabilities(&self, task_id: TaskId) -> Result<u64, ProcessError>;

    /// Wait for a child process to exit
    ///
    /// Handles the blocking retry loop internally. When `no_hang` is false,
    /// this blocks until a child exits. When `no_hang` is true, returns
    /// `WouldBlock` immediately if no child has exited.
    fn wait_child(&self, caller: TaskId, target_pid: i32, no_hang: bool) -> WaitChildResult;

    /// List running processes into a caller-provided buffer
    ///
    /// Returns the number of entries written. Skips idle tasks.
    fn list_processes(&self, buf: &mut [abi::ProcessInfo], max: usize) -> usize;
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
    wait_child_result: WaitChildResult,
    list_processes_count: usize,
}

#[cfg(any(test, feature = "mock"))]
impl MockProcessOps {
    /// Create a new mock with default (success) results
    pub const fn new() -> Self {
        Self {
            kill_result: Ok(()),
            spawn_result: Ok(1), // Default spawned PID
            wait_child_result: WaitChildResult::NoChildren,
            list_processes_count: 0,
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

    /// Configure wait_child() result
    pub const fn with_wait_child_result(mut self, result: WaitChildResult) -> Self {
        self.wait_child_result = result;
        self
    }

    /// Configure list_processes() count
    pub const fn with_list_processes_count(mut self, count: usize) -> Self {
        self.list_processes_count = count;
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

    fn spawn(&self, _parent: TaskId, _source: SpawnSource) -> Result<TaskId, ProcessError> {
        self.spawn_result
    }

    fn daemonize(&self, _task_id: TaskId) -> Result<(), ProcessError> {
        Ok(())
    }

    fn get_capabilities(&self, _task_id: TaskId) -> Result<u64, ProcessError> {
        Ok(0)
    }

    fn wait_child(&self, _caller: TaskId, _target_pid: i32, _no_hang: bool) -> WaitChildResult {
        self.wait_child_result
    }

    fn list_processes(&self, _buf: &mut [abi::ProcessInfo], _max: usize) -> usize {
        self.list_processes_count
    }
}
