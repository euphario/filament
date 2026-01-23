//! Memory Operations Backend Implementation
//!
//! Implements the `MemoryOps` trait for anonymous memory allocation.
//! DMA and device MMIO mapping go through ObjectOps instead.

use crate::kernel::traits::memory_ops::{MemoryOps, MemoryError};
use crate::kernel::traits::task::TaskId;
use crate::kernel::task;

// ============================================================================
// Kernel Memory Operations Implementation
// ============================================================================

/// Kernel memory operations backend implementation
///
/// A zero-sized type that implements `MemoryOps` by delegating to the
/// task's address space operations.
pub struct KernelMemoryOps;

impl KernelMemoryOps {
    /// Create a new kernel memory operations instance
    pub const fn new() -> Self {
        Self
    }
}

impl MemoryOps for KernelMemoryOps {
    fn mmap(
        &self,
        task_id: TaskId,
        size: usize,
        writable: bool,
        executable: bool,
    ) -> Result<u64, MemoryError> {
        if size == 0 {
            return Err(MemoryError::InvalidSize);
        }

        task::with_scheduler(|sched| {
            let slot = sched.slot_by_pid(task_id)
                .ok_or(MemoryError::NoTask)?;

            let task = sched.task_mut(slot)
                .ok_or(MemoryError::NoTask)?;

            // Use task's mmap method for anonymous memory
            // The task method handles page allocation and mapping
            task.mmap(size, writable, executable)
                .ok_or(MemoryError::OutOfMemory)
        })
    }

    fn munmap(
        &self,
        task_id: TaskId,
        addr: u64,
        size: usize,
    ) -> Result<(), MemoryError> {
        if size == 0 {
            return Err(MemoryError::InvalidSize);
        }

        task::with_scheduler(|sched| {
            let slot = sched.slot_by_pid(task_id)
                .ok_or(MemoryError::NoTask)?;

            let task = sched.task_mut(slot)
                .ok_or(MemoryError::NoTask)?;

            // Use task's munmap method
            if task.munmap(addr, size) {
                Ok(())
            } else {
                Err(MemoryError::NotMapped)
            }
        })
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global kernel memory operations backend
pub static MEMORY_OPS_BACKEND: KernelMemoryOps = KernelMemoryOps::new();

/// Get a reference to the global memory operations backend
pub fn memory_ops_backend() -> &'static dyn MemoryOps {
    &MEMORY_OPS_BACKEND
}
