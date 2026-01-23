//! User Access Backend Implementation
//!
//! Implements the `UserAccess` trait by wrapping the existing uaccess module.
//! This provides a clean trait boundary for testing without changing the
//! internal implementation.

use crate::kernel::traits::user_access::{UserAccess, UAccessError};
use crate::kernel::traits::task::TaskId;
use crate::kernel::uaccess;

// ============================================================================
// Kernel User Access Implementation
// ============================================================================

/// Kernel user access backend implementation
///
/// A zero-sized type that implements `UserAccess` by delegating to the
/// existing uaccess module.
pub struct KernelUserAccess;

impl KernelUserAccess {
    /// Create a new kernel user access instance
    pub const fn new() -> Self {
        Self
    }
}

/// Convert uaccess error to trait error
fn convert_error(e: uaccess::UAccessError) -> UAccessError {
    match e {
        uaccess::UAccessError::NotMapped => UAccessError::NotMapped,
        uaccess::UAccessError::KernelAddress => UAccessError::KernelAddress,
        uaccess::UAccessError::Unaligned => UAccessError::BadAlignment,
        uaccess::UAccessError::NullPointer => UAccessError::NotMapped,
        uaccess::UAccessError::Overflow => UAccessError::NotMapped,
        uaccess::UAccessError::NotReadable => UAccessError::NotMapped,
        uaccess::UAccessError::NotWritable => UAccessError::NotMapped,
        uaccess::UAccessError::InvalidLength => UAccessError::BufferTooSmall,
    }
}

impl UserAccess for KernelUserAccess {
    fn copy_from_user(
        &self,
        _task_id: TaskId,
        dst: &mut [u8],
        src: u64,
    ) -> Result<usize, UAccessError> {
        // Note: Current uaccess uses current task's page tables
        // task_id is ignored but kept for future task-specific access
        match uaccess::copy_from_user(dst, src) {
            Ok(n) => Ok(n),
            Err(e) => Err(convert_error(e)),
        }
    }

    fn copy_to_user(
        &self,
        _task_id: TaskId,
        dst: u64,
        src: &[u8],
    ) -> Result<usize, UAccessError> {
        // Note: Current uaccess uses current task's page tables
        match uaccess::copy_to_user(dst, src) {
            Ok(n) => Ok(n),
            Err(e) => Err(convert_error(e)),
        }
    }

    fn validate_read(
        &self,
        _task_id: TaskId,
        ptr: u64,
        len: usize,
    ) -> Result<(), UAccessError> {
        match uaccess::validate_user_read(ptr, len) {
            Ok(()) => Ok(()),
            Err(e) => Err(convert_error(e)),
        }
    }

    fn validate_write(
        &self,
        _task_id: TaskId,
        ptr: u64,
        len: usize,
    ) -> Result<(), UAccessError> {
        match uaccess::validate_user_write(ptr, len) {
            Ok(()) => Ok(()),
            Err(e) => Err(convert_error(e)),
        }
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global kernel user access backend
pub static USER_ACCESS_BACKEND: KernelUserAccess = KernelUserAccess::new();

/// Get a reference to the global user access backend
pub fn user_access_backend() -> &'static dyn UserAccess {
    &USER_ACCESS_BACKEND
}
