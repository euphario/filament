//! Syscall Context Trait
//!
//! This trait is the entry point for all syscall operations.
//! Syscalls call trait methods, never access kernel internals directly.
//!
//! # Design Philosophy
//!
//! Syscalls are thin dispatchers. They:
//! 1. Validate arguments
//! 2. Call trait method
//! 3. Return result
//!
//! All kernel logic lives behind traits. Syscalls never access
//! scheduler, task structs, or internal state directly.

use super::task::TaskId;
use crate::kernel::error::KernelError;

/// Error type for syscall context operations — unified KernelError
pub type SyscallError = KernelError;

/// Syscall context - provides access to all kernel subsystems for syscalls
///
/// A single implementation of this trait is created at syscall entry
/// and passed to all syscall handlers. The handlers call trait methods
/// instead of directly accessing kernel internals.
///
/// # Contract
///
/// 1. Created once at syscall entry
/// 2. Provides current task context
/// 3. Delegates to subsystem traits for operations
/// 4. All operations are thread-safe
pub trait SyscallContext: Send + Sync {
    /// Get the current task's ID
    ///
    /// Returns None if no task is running (kernel-only context).
    fn current_task_id(&self) -> Option<TaskId>;

    /// Check if current task has a specific capability
    ///
    /// # Arguments
    /// * `cap` - Capability bits to check
    fn has_capability(&self, cap: u64) -> bool;

    /// Require a capability, returning error if not present
    fn require_capability(&self, cap: u64) -> Result<(), SyscallError> {
        if self.has_capability(cap) {
            Ok(())
        } else {
            Err(SyscallError::PermDenied)
        }
    }

    // ========================================================================
    // Subsystem Access
    // ========================================================================

    /// Access object operations (the unified 5-syscall interface)
    fn objects(&self) -> &dyn ObjectOps;

    /// Access memory operations (mmap, munmap, etc.)
    fn memory(&self) -> &dyn MemoryOps;

    /// Access process operations (exit, kill, spawn)
    fn process(&self) -> &dyn ProcessOps;

    /// Access user memory operations (copy to/from user)
    fn uaccess(&self) -> &dyn UserAccess;
}

// Forward declare the subsystem traits (defined in other files)
use super::object_ops::ObjectOps;
use super::memory_ops::MemoryOps;
use super::process_ops::ProcessOps;
use super::user_access::UserAccess;
