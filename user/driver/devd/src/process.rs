//! Process Management
//!
//! Handles process spawning and termination.
//! Uses trait-based design for testability.
//!
//! Spawns use exec_with_mailbox (syscall 76) which creates:
//! - A shared 4KB shmem page with birth context (MailboxHeader + spawn metadata)
//! - A kernel SupervisionQueue for bidirectional parent↔child messaging

use userlib::error::SysError;
use userlib::syscall;

// =============================================================================
// ProcessManager Trait
// =============================================================================

/// Process lifecycle operations trait
pub trait ProcessManager {
    /// Spawn a new process with explicit capabilities and a mailbox page.
    ///
    /// `mailbox_data` is copied into a shared 4KB shmem page. The child
    /// receives it at Handle::MAILBOX (slot 5). The parent gets back
    /// (child_pid, parent_shmem_handle, parent_superq_handle).
    fn spawn_with_caps(&mut self, binary: &str, caps: u64, mailbox_data: &[u8]) -> Result<(u32, abi::Handle, abi::Handle), SysError>;

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
    fn spawn_with_caps(&mut self, binary: &str, caps: u64, mailbox_data: &[u8]) -> Result<(u32, abi::Handle, abi::Handle), SysError> {
        match syscall::exec_with_mailbox(binary, caps, mailbox_data) {
            Ok((pid, shmem_handle, superq_handle)) => Ok((pid, shmem_handle, superq_handle)),
            Err(errno) => Err(SysError::from_errno(-errno as i32)),
        }
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
    fn spawn_with_caps(&mut self, binary: &str, _caps: u64, _mailbox_data: &[u8]) -> Result<(u32, abi::Handle, abi::Handle), SysError> {
        let pid = self.next_pid;
        self.next_pid += 1;
        self.spawned.push((pid, binary.to_string()));
        // Can't create real Handle in mock
        Err(SysError::Unsupported)
    }

    fn kill(&mut self, pid: u32) -> Result<(), SysError> {
        self.killed.push(pid);
        Ok(())
    }
}
