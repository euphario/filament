//! Memory Operations Trait
//!
//! This trait provides syscall-level memory operations for tasks.
//! It operates on a task's address space for anonymous memory only.
//!
//! # Operations
//!
//! - `mmap` - Allocate anonymous memory
//! - `munmap` - Free memory
//!
//! # What's NOT Here
//!
//! DMA and device MMIO mapping go through the unified object interface:
//! ```ignore
//! open(ObjectType::DmaPool, size) → handle
//! map(handle) → (vaddr, paddr, size)
//!
//! open(ObjectType::Mmio, phys_addr) → handle
//! map(handle) → vaddr
//! ```

use super::task::TaskId;

/// Error type for memory operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryError {
    /// Invalid size (zero, overflow, or not aligned)
    InvalidSize,
    /// Invalid address
    InvalidAddress,
    /// Out of memory
    OutOfMemory,
    /// Permission denied
    PermissionDenied,
    /// Region not found (for unmap)
    NotFound,
    /// Address not mapped
    NotMapped,
    /// No current task
    NoTask,
}

impl MemoryError {
    pub fn to_errno(self) -> i64 {
        match self {
            MemoryError::InvalidSize => -22,      // EINVAL
            MemoryError::InvalidAddress => -14,   // EFAULT
            MemoryError::OutOfMemory => -12,      // ENOMEM
            MemoryError::PermissionDenied => -1,  // EPERM
            MemoryError::NotFound => -2,          // ENOENT
            MemoryError::NotMapped => -22,        // EINVAL
            MemoryError::NoTask => -3,            // ESRCH
        }
    }
}

/// Trait for task memory operations
///
/// These operations modify a task's address space for anonymous memory.
/// DMA and device mapping use ObjectOps instead.
///
/// # Contract
///
/// 1. All operations are task-local
/// 2. Addresses are page-aligned
/// 3. Operations are thread-safe
/// 4. Resources are tracked per-task for cleanup on exit
pub trait MemoryOps: Send + Sync {
    /// Allocate anonymous memory in task's address space
    ///
    /// # Arguments
    /// * `task_id` - Target task
    /// * `size` - Size in bytes (will be page-aligned)
    /// * `writable` - Allow writes
    /// * `executable` - Allow execution
    ///
    /// # Returns
    /// Virtual address on success
    fn mmap(
        &self,
        task_id: TaskId,
        size: usize,
        writable: bool,
        executable: bool,
    ) -> Result<u64, MemoryError>;

    /// Free memory in task's address space
    ///
    /// # Arguments
    /// * `task_id` - Target task
    /// * `addr` - Address to unmap (must be start of allocation)
    /// * `size` - Size to unmap
    fn munmap(&self, task_id: TaskId, addr: u64, size: usize) -> Result<(), MemoryError>;
}
