//! Syscall Context Backend Implementation
//!
//! Implements the `SyscallContext` trait which is the entry point for all
//! syscalls. It provides access to the current task and all subsystem backends.
//!
//! # Design
//!
//! The syscall context is created at syscall entry and provides:
//! - Current task identification
//! - Capability checking
//! - Access to all subsystem backends (ObjectOps, MemoryOps, ProcessOps, UserAccess)
//!
//! This allows syscalls to be thin dispatchers that validate, dispatch, and return.

use crate::kernel::traits::syscall_ctx::SyscallContext;
use crate::kernel::error::KernelError;
use crate::kernel::traits::task::TaskId;
use crate::kernel::traits::object_ops::ObjectOps;
use crate::kernel::traits::raw_object_ops::RawObjectOps;
use crate::kernel::traits::memory_ops::MemoryOps;
use crate::kernel::traits::process_ops::ProcessOps;
use crate::kernel::traits::user_access::UserAccess;
use crate::kernel::traits::misc_ops::MiscOps;
use crate::kernel::task;
use crate::kernel::caps::Capabilities;

// ============================================================================
// Kernel Syscall Context Implementation
// ============================================================================

/// Kernel syscall context implementation
///
/// Created at syscall entry, provides access to current task and all backends.
pub struct KernelSyscallContext {
    /// Cached current task ID (None if no task is running)
    task_id: Option<TaskId>,
}

impl KernelSyscallContext {
    /// Create a new syscall context
    ///
    /// This should be called at syscall entry. It captures the current task ID.
    pub fn new() -> Self {
        let task_id = task::with_scheduler(|sched| {
            sched.current_task().map(|t| t.id)
        });
        Self { task_id }
    }

    /// Create a syscall context with a specific task ID (for testing)
    #[cfg(test)]
    pub fn with_task_id(task_id: TaskId) -> Self {
        Self { task_id: Some(task_id) }
    }
}

impl SyscallContext for KernelSyscallContext {
    fn current_task_id(&self) -> Option<TaskId> {
        self.task_id
    }

    fn has_capability(&self, cap: u64) -> bool {
        let Some(task_id) = self.task_id else {
            return false;
        };

        task::with_scheduler(|sched| {
            sched.slot_by_pid(task_id)
                .and_then(|slot| sched.task(slot))
                .map(|t| t.capabilities.has(Capabilities::from_bits(cap)))
                .unwrap_or(false)
        })
    }

    fn require_capability(&self, cap: u64) -> Result<(), KernelError> {
        if self.has_capability(cap) {
            Ok(())
        } else {
            Err(KernelError::PermDenied)
        }
    }

    fn objects(&self) -> &dyn ObjectOps {
        // Return the global object ops backend
        crate::kernel::object_ops_impl::object_ops_backend()
    }

    fn raw_objects(&self) -> &dyn RawObjectOps {
        crate::kernel::raw_object_ops_impl::raw_object_ops_backend()
    }

    fn memory(&self) -> &dyn MemoryOps {
        // Return the global memory ops backend
        crate::kernel::memory_ops_impl::memory_ops_backend()
    }

    fn process(&self) -> &dyn ProcessOps {
        // Return the global process ops backend
        crate::kernel::process_ops_impl::process_ops_backend()
    }

    fn uaccess(&self) -> &dyn UserAccess {
        // Return the global user access backend
        crate::kernel::user_access_impl::user_access_backend()
    }

    fn misc(&self) -> &dyn MiscOps {
        crate::kernel::misc_ops_impl::misc_ops_backend()
    }
}

// ============================================================================
// Global Instance Factory
// ============================================================================

/// Create a new syscall context for the current syscall
///
/// This should be called at the beginning of syscall dispatch.
pub fn create_syscall_context() -> KernelSyscallContext {
    KernelSyscallContext::new()
}
