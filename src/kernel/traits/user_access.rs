//! User Access Trait
//!
//! This trait provides safe access to user-space memory from syscalls.
//! All user pointers must be validated and copied safely.
//!
//! # Security
//!
//! Direct access to user pointers is unsafe because:
//! 1. User might provide kernel address
//! 2. User might provide unmapped address
//! 3. Page might be swapped out (not applicable in our simple kernel)
//!
//! This trait encapsulates the validation and copy logic.

use super::task::TaskId;

/// Error type for user access operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UAccessError {
    /// Pointer is in kernel space (invalid)
    KernelAddress,
    /// Pointer is not mapped
    NotMapped,
    /// No current task
    NoTask,
    /// Buffer too small
    BufferTooSmall,
    /// Invalid alignment
    BadAlignment,
}

impl UAccessError {
    pub fn to_errno(self) -> i64 {
        match self {
            UAccessError::KernelAddress => -14,  // EFAULT
            UAccessError::NotMapped => -14,      // EFAULT
            UAccessError::NoTask => -3,          // ESRCH
            UAccessError::BufferTooSmall => -22, // EINVAL
            UAccessError::BadAlignment => -22,   // EINVAL
        }
    }
}

/// Trait for safe user memory access
///
/// # Contract
///
/// 1. All pointers are validated before access
/// 2. Kernel addresses are rejected
/// 3. Unmapped addresses cause error (not panic)
/// 4. Copy is atomic from user perspective
pub trait UserAccess: Send + Sync {
    /// Copy data from user space to kernel buffer
    ///
    /// # Arguments
    /// * `task_id` - Task whose address space to access
    /// * `dst` - Kernel buffer to copy into
    /// * `src` - User address to copy from
    ///
    /// # Returns
    /// Number of bytes copied on success
    fn copy_from_user(
        &self,
        task_id: TaskId,
        dst: &mut [u8],
        src: u64,
    ) -> Result<usize, UAccessError>;

    /// Copy data from kernel buffer to user space
    ///
    /// # Arguments
    /// * `task_id` - Task whose address space to access
    /// * `dst` - User address to copy to
    /// * `src` - Kernel buffer to copy from
    ///
    /// # Returns
    /// Number of bytes copied on success
    fn copy_to_user(
        &self,
        task_id: TaskId,
        dst: u64,
        src: &[u8],
    ) -> Result<usize, UAccessError>;

    /// Validate that a user pointer range is readable
    ///
    /// # Arguments
    /// * `task_id` - Task whose address space to check
    /// * `ptr` - User address
    /// * `len` - Length of range
    fn validate_read(
        &self,
        task_id: TaskId,
        ptr: u64,
        len: usize,
    ) -> Result<(), UAccessError>;

    /// Validate that a user pointer range is writable
    ///
    /// # Arguments
    /// * `task_id` - Task whose address space to check
    /// * `ptr` - User address
    /// * `len` - Length of range
    fn validate_write(
        &self,
        task_id: TaskId,
        ptr: u64,
        len: usize,
    ) -> Result<(), UAccessError>;

}

// ============================================================================
// Extension trait for generic get_user/put_user (not dyn-compatible)
// ============================================================================

/// Extension methods for UserAccess that use generics
///
/// These are provided separately because generic methods prevent
/// a trait from being dyn-compatible.
pub trait UserAccessExt: UserAccess {
    /// Read a single value from user space
    fn get_user<T: Copy + Sized>(
        &self,
        task_id: TaskId,
        ptr: u64,
    ) -> Result<T, UAccessError> {
        let mut val = core::mem::MaybeUninit::<T>::uninit();
        let dst = unsafe {
            core::slice::from_raw_parts_mut(
                val.as_mut_ptr() as *mut u8,
                core::mem::size_of::<T>(),
            )
        };
        self.copy_from_user(task_id, dst, ptr)?;
        Ok(unsafe { val.assume_init() })
    }

    /// Write a single value to user space
    fn put_user<T: Copy + Sized>(
        &self,
        task_id: TaskId,
        ptr: u64,
        val: T,
    ) -> Result<(), UAccessError> {
        let src = unsafe {
            core::slice::from_raw_parts(
                &val as *const T as *const u8,
                core::mem::size_of::<T>(),
            )
        };
        self.copy_to_user(task_id, ptr, src)?;
        Ok(())
    }
}

// Blanket implementation for all UserAccess implementors
impl<T: UserAccess + ?Sized> UserAccessExt for T {}
