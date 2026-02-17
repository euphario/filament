//! Process Management
//!
//! Handles process spawning and termination.
//! Uses trait-based design for testability.
//!
//! Spawns use exec_with_channel (syscall 75) to create a supervision channel
//! at spawn time. The parent end is returned for protocol messages and
//! presence monitoring (channel CLOSED = child died).

use userlib::ipc::Channel;
use userlib::error::SysError;
use userlib::syscall;

// =============================================================================
// ProcessManager Trait
// =============================================================================

/// Process lifecycle operations trait
pub trait ProcessManager {
    /// Spawn a new process from binary name
    /// Returns (pid, supervision_channel) on success
    fn spawn(&mut self, binary: &str) -> Result<(u32, Channel), SysError>;

    /// Spawn a new process with explicit capabilities
    ///
    /// If `caps` is 0, inherits parent's capabilities unchanged (same as spawn).
    /// If `caps` is non-zero, uses exec_with_channel to restrict the child.
    fn spawn_with_caps(&mut self, binary: &str, caps: u64) -> Result<(u32, Channel), SysError>;

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
    fn spawn(&mut self, binary: &str) -> Result<(u32, Channel), SysError> {
        self.spawn_with_caps(binary, 0)
    }

    fn spawn_with_caps(&mut self, binary: &str, caps: u64) -> Result<(u32, Channel), SysError> {
        let (parent_ch, child_ch) = Channel::pair()?;
        let child_handle = child_ch.into_raw_handle();

        let pid = syscall::exec_with_channel(binary, caps, child_handle, abi::priority::INHERIT);
        if pid < 0 {
            // Child handle was consumed by into_raw_handle but exec failed —
            // the kernel did not transfer it, so it's leaked. We can't recover
            // it, but parent_ch will be dropped and cleaned up.
            return Err(SysError::from_errno(-pid as i32));
        }

        Ok((pid as u32, parent_ch))
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
    fn spawn(&mut self, binary: &str) -> Result<(u32, Channel), SysError> {
        let pid = self.next_pid;
        self.next_pid += 1;
        self.spawned.push((pid, binary.to_string()));
        // Can't create real Channel in mock, so this would need adjustment
        // For now, return error to indicate mock limitation
        Err(SysError::Unsupported)
    }

    fn spawn_with_caps(&mut self, binary: &str, _caps: u64) -> Result<(u32, Channel), SysError> {
        self.spawn(binary)
    }

    fn kill(&mut self, pid: u32) -> Result<(), SysError> {
        self.killed.push(pid);
        Ok(())
    }
}
