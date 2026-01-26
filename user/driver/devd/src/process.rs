//! Process Management
//!
//! Handles process spawning and termination.
//! Uses trait-based design for testability.

use userlib::ipc::Process;
use userlib::error::SysError;
use userlib::syscall;

// =============================================================================
// ProcessManager Trait
// =============================================================================

/// Process lifecycle operations trait
pub trait ProcessManager {
    /// Spawn a new process from binary name
    /// Returns (pid, watcher) on success
    fn spawn(&mut self, binary: &str) -> Result<(u32, Process), SysError>;

    /// Kill a process by PID
    fn kill(&mut self, pid: u32) -> Result<(), SysError>;
}

// =============================================================================
// SyscallProcessManager Implementation
// =============================================================================

/// Concrete implementation using syscalls
pub struct SyscallProcessManager;

impl SyscallProcessManager {
    pub const fn new() -> Self {
        Self
    }
}

impl ProcessManager for SyscallProcessManager {
    fn spawn(&mut self, binary: &str) -> Result<(u32, Process), SysError> {
        let pid = syscall::exec(binary);
        if pid < 0 {
            return Err(SysError::from_errno(-pid as i32));
        }
        let pid = pid as u32;

        let watcher = Process::watch(pid)?;
        Ok((pid, watcher))
    }

    fn kill(&mut self, pid: u32) -> Result<(), SysError> {
        let ret = syscall::kill(pid);
        if ret < 0 {
            Err(SysError::from_errno(-ret as i32))
        } else {
            Ok(())
        }
    }
}

// =============================================================================
// Mock Implementation for Testing
// =============================================================================

#[cfg(test)]
pub struct MockProcessManager {
    next_pid: u32,
    spawned: Vec<(u32, String)>,
    killed: Vec<u32>,
}

#[cfg(test)]
impl MockProcessManager {
    pub fn new() -> Self {
        Self {
            next_pid: 100,
            spawned: Vec::new(),
            killed: Vec::new(),
        }
    }

    pub fn spawned_binaries(&self) -> &[(u32, String)] {
        &self.spawned
    }

    pub fn killed_pids(&self) -> &[u32] {
        &self.killed
    }
}

#[cfg(test)]
impl ProcessManager for MockProcessManager {
    fn spawn(&mut self, binary: &str) -> Result<(u32, Process), SysError> {
        let pid = self.next_pid;
        self.next_pid += 1;
        self.spawned.push((pid, binary.to_string()));
        // Can't create real Process in mock, so this would need adjustment
        // For now, return error to indicate mock limitation
        Err(SysError::Unsupported)
    }

    fn kill(&mut self, pid: u32) -> Result<(), SysError> {
        self.killed.push(pid);
        Ok(())
    }
}
