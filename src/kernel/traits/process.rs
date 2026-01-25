//! Process Management Trait Definitions
//!
//! These traits define the contract for process operations.
//! This enables mock implementations for testing without hardware.
//!
//! # Design Philosophy
//!
//! The process trait provides operations for process lifecycle management
//! without exposing internal details like address space manipulation or
//! kernel stack allocation.

/// Process ID
pub type Pid = u32;

// ============================================================================
// Process State Information
// ============================================================================

/// Simplified process state for trait boundary
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProcessStateInfo {
    /// Process is being created
    Creating,
    /// Process is ready to run
    Ready,
    /// Process is currently running
    Running,
    /// Process is blocked waiting for something
    Blocked,
    /// Process has exited but not yet reaped
    Zombie { exit_code: i32 },
    /// Process slot is free (no process)
    Free,
}

impl ProcessStateInfo {
    /// Check if process is runnable
    pub fn is_runnable(&self) -> bool {
        matches!(self, ProcessStateInfo::Ready | ProcessStateInfo::Running)
    }

    /// Check if process is blocked
    pub fn is_blocked(&self) -> bool {
        matches!(self, ProcessStateInfo::Blocked)
    }

    /// Check if process is a zombie
    pub fn is_zombie(&self) -> bool {
        matches!(self, ProcessStateInfo::Zombie { .. })
    }

    /// Check if process is terminated (zombie or free)
    pub fn is_terminated(&self) -> bool {
        matches!(self, ProcessStateInfo::Zombie { .. } | ProcessStateInfo::Free)
    }
}

// ============================================================================
// Process Info Structure
// ============================================================================

/// Basic process information for queries
#[derive(Clone)]
pub struct ProcessInfo {
    /// Process ID
    pub pid: Pid,
    /// Parent process ID
    pub parent_pid: Pid,
    /// Current state
    pub state: ProcessStateInfo,
    /// Process name (truncated to 32 bytes)
    pub name: [u8; 32],
}

impl ProcessInfo {
    /// Get name as string slice
    pub fn name_str(&self) -> &str {
        let len = self.name.iter().position(|&c| c == 0).unwrap_or(32);
        core::str::from_utf8(&self.name[..len]).unwrap_or("???")
    }
}

// ============================================================================
// Process Backend Trait
// ============================================================================

/// Trait for process management operations
///
/// This is the main entry point for process operations.
/// The kernel holds a single instance implementing this trait.
///
/// # Contract
///
/// 1. All operations are thread-safe (internal locking)
/// 2. PIDs are unique and never reused while process exists
/// 3. Zombie processes persist until reaped
/// 4. Operations return meaningful results
pub trait ProcessBackend: Send + Sync {
    // ========================================================================
    // Current Process
    // ========================================================================

    /// Get the current process's PID
    ///
    /// Returns None if no process is running (kernel context).
    fn current_pid(&self) -> Option<Pid>;

    /// Get the current process's info
    fn current_info(&self) -> Option<ProcessInfo>;

    // ========================================================================
    // Process Lookup
    // ========================================================================

    /// Get process info by PID
    fn get_info(&self, pid: Pid) -> Option<ProcessInfo>;

    /// Get process state by PID
    fn get_state(&self, pid: Pid) -> Option<ProcessStateInfo>;

    /// Check if a process exists
    fn exists(&self, pid: Pid) -> bool {
        self.get_state(pid).is_some()
    }

    // ========================================================================
    // Process Lifecycle
    // ========================================================================

    /// Create a new process
    ///
    /// # Arguments
    /// * `parent` - Parent process PID
    /// * `name` - Process name (will be truncated to 32 bytes)
    ///
    /// # Returns
    /// New process PID on success, None if no slots available.
    fn create(&self, parent: Pid, name: &str) -> Option<Pid>;

    /// Terminate a process
    ///
    /// Transitions process to Zombie state with given exit code.
    /// Process remains in table until reaped.
    fn terminate(&self, pid: Pid, exit_code: i32);

    /// Reap a zombie process
    ///
    /// Removes zombie process from table and returns its exit code.
    /// Only works on zombie processes.
    ///
    /// # Returns
    /// Exit code if process was zombie, None if not found or not zombie.
    fn reap(&self, pid: Pid) -> Option<i32>;

    // ========================================================================
    // Parent-Child Relationships
    // ========================================================================

    /// Get parent PID for a process
    fn parent_of(&self, pid: Pid) -> Option<Pid>;

    /// Count children of a process
    fn child_count(&self, pid: Pid) -> usize;

    /// Check if process has any zombie children
    fn has_zombie_children(&self, pid: Pid) -> bool;

    // ========================================================================
    // Wake Operations
    // ========================================================================

    /// Wake a blocked process
    ///
    /// Transitions from Blocked to Ready state.
    ///
    /// # Returns
    /// true if process was woken, false if not blocked or not found.
    fn wake(&self, pid: Pid) -> bool;
}

// Note: ProcessError is defined in traits/process_ops.rs for syscall layer.
// This module defines ProcessBackend for process lifecycle operations.
