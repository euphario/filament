//! Raw Object Operations Backend Implementation
//!
//! Implements the `RawObjectOps` trait by delegating to `object::syscall`.
//! This is the production implementation used by the syscall dispatcher.

use crate::kernel::traits::raw_object_ops::RawObjectOps;
use crate::kernel::object;

/// Kernel raw object operations backend implementation
///
/// Delegates directly to the object::syscall functions.
pub struct KernelRawObjectOps;

impl KernelRawObjectOps {
    pub const fn new() -> Self {
        Self
    }
}

impl RawObjectOps for KernelRawObjectOps {
    fn open(&self, type_id: u32, arg: u64, flags: usize) -> i64 {
        object::syscall::open(type_id, arg, flags)
    }

    fn read(&self, handle: u32, buf_ptr: u64, buf_len: usize) -> i64 {
        object::syscall::read(handle, buf_ptr, buf_len)
    }

    fn write(&self, handle: u32, buf_ptr: u64, buf_len: usize) -> i64 {
        object::syscall::write(handle, buf_ptr, buf_len)
    }

    fn map(&self, handle: u32, flags: u32) -> i64 {
        object::syscall::map(handle, flags)
    }

    fn close(&self, handle: u32) -> i64 {
        object::syscall::close(handle)
    }
}

/// Global kernel raw object operations backend
pub static RAW_OBJECT_OPS_BACKEND: KernelRawObjectOps = KernelRawObjectOps::new();

/// Get a reference to the global raw object operations backend
pub fn raw_object_ops_backend() -> &'static dyn RawObjectOps {
    &RAW_OBJECT_OPS_BACKEND
}
