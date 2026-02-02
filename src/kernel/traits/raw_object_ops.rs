//! Raw Object Operations Trait
//!
//! This trait matches the actual syscall signatures for the unified 5-syscall
//! interface. Unlike `ObjectOps` which takes kernel-space `&[u8]` buffers,
//! `RawObjectOps` passes raw user pointers and lengths — exactly what the
//! syscall dispatch receives from userspace.
//!
//! # The 5 Syscalls
//!
//! | Syscall | Method | Signature |
//! |---------|--------|-----------|
//! | open(100) | open | (type_id, arg, flags) -> i64 |
//! | read(101) | read | (handle, buf_ptr, buf_len) -> i64 |
//! | write(102) | write | (handle, buf_ptr, buf_len) -> i64 |
//! | map(103) | map | (handle, flags) -> i64 |
//! | close(104) | close | (handle) -> i64 |

/// Trait for raw syscall-level object operations.
///
/// Each method takes the exact arguments from the syscall registers
/// and returns a raw i64 result (positive = success, negative = errno).
///
/// The implementation handles user pointer validation internally.
pub trait RawObjectOps: Send + Sync {
    /// Open/create an object (syscall 100)
    fn open(&self, type_id: u32, arg: u64, flags: usize) -> i64;

    /// Read from an object (syscall 101)
    fn read(&self, handle: u32, buf_ptr: u64, buf_len: usize) -> i64;

    /// Write to an object (syscall 102)
    fn write(&self, handle: u32, buf_ptr: u64, buf_len: usize) -> i64;

    /// Map an object to memory (syscall 103)
    fn map(&self, handle: u32, flags: u32) -> i64;

    /// Close an object handle (syscall 104)
    fn close(&self, handle: u32) -> i64;
}

// ============================================================================
// Mock Implementation (for testing)
// ============================================================================

#[cfg(any(test, feature = "mock"))]
pub struct MockRawObjectOps {
    pub open_result: i64,
    pub read_result: i64,
    pub write_result: i64,
    pub map_result: i64,
    pub close_result: i64,
}

#[cfg(any(test, feature = "mock"))]
impl MockRawObjectOps {
    pub const fn new() -> Self {
        Self {
            open_result: 0,
            read_result: 0,
            write_result: 0,
            map_result: 0,
            close_result: 0,
        }
    }

    pub const fn with_open_result(mut self, result: i64) -> Self {
        self.open_result = result;
        self
    }

    pub const fn with_read_result(mut self, result: i64) -> Self {
        self.read_result = result;
        self
    }

    pub const fn with_write_result(mut self, result: i64) -> Self {
        self.write_result = result;
        self
    }

    pub const fn with_map_result(mut self, result: i64) -> Self {
        self.map_result = result;
        self
    }

    pub const fn with_close_result(mut self, result: i64) -> Self {
        self.close_result = result;
        self
    }
}

#[cfg(any(test, feature = "mock"))]
impl RawObjectOps for MockRawObjectOps {
    fn open(&self, _type_id: u32, _arg: u64, _flags: usize) -> i64 { self.open_result }
    fn read(&self, _handle: u32, _buf_ptr: u64, _buf_len: usize) -> i64 { self.read_result }
    fn write(&self, _handle: u32, _buf_ptr: u64, _buf_len: usize) -> i64 { self.write_result }
    fn map(&self, _handle: u32, _flags: u32) -> i64 { self.map_result }
    fn close(&self, _handle: u32) -> i64 { self.close_result }
}
