//! Miscellaneous Operations Trait
//!
//! This trait provides syscall-level operations for logging and ramfs.
//! These are operations that don't fit into ProcessOps, MemoryOps, or ObjectOps.

/// Trait for miscellaneous kernel operations
///
/// Covers logging (klog) and ramfs listing.
pub trait MiscOps: Send + Sync {
    /// Write a user log message into the kernel log ring
    ///
    /// The caller provides the parsed level and message bytes.
    /// Returns bytes written on success, negative errno on failure.
    fn write_user_log(&self, caller_pid: u32, level: u8, msg: &[u8]) -> i64;

    /// Read one formatted log record from the kernel ring
    ///
    /// Reads into `buf` and returns the number of bytes written,
    /// or 0 if no records are available.
    fn read_log_record(&self, buf: &mut [u8]) -> i64;

    /// Write a pre-built binary log record into the kernel ring
    ///
    /// The record must be in standard binary format (RecordHeader + fields).
    /// Returns bytes written on success, negative errno on failure.
    fn write_raw_record(&self, record: &[u8]) -> i64;

    /// List ramfs entries into a caller-provided buffer
    ///
    /// Returns the number of entries written.
    fn list_ramfs(&self, buf: &mut [abi::RamfsListEntry], max: usize) -> usize;

    /// Sleep until the given deadline (absolute nanoseconds since boot).
    ///
    /// Atomically blocks the current task with a timer deadline and reschedules.
    /// Returns Ok(()) if the task was woken, Err if the transition failed.
    fn sleep_until(&self, deadline: u64) -> Result<(), crate::kernel::error::KernelError>;
}

// ============================================================================
// Mock Implementation for Testing
// ============================================================================

#[cfg(any(test, feature = "mock"))]
pub struct MockMiscOps {
    write_user_log_result: i64,
    read_log_record_result: i64,
    write_raw_record_result: i64,
    list_ramfs_count: usize,
}

#[cfg(any(test, feature = "mock"))]
impl MockMiscOps {
    pub const fn new() -> Self {
        Self {
            write_user_log_result: 0,
            read_log_record_result: 0,
            write_raw_record_result: 0,
            list_ramfs_count: 0,
        }
    }

    pub const fn with_write_user_log_result(mut self, result: i64) -> Self {
        self.write_user_log_result = result;
        self
    }

    pub const fn with_list_ramfs_count(mut self, count: usize) -> Self {
        self.list_ramfs_count = count;
        self
    }
}

#[cfg(any(test, feature = "mock"))]
impl MiscOps for MockMiscOps {
    fn write_user_log(&self, _caller_pid: u32, _level: u8, _msg: &[u8]) -> i64 {
        self.write_user_log_result
    }

    fn read_log_record(&self, _buf: &mut [u8]) -> i64 {
        self.read_log_record_result
    }

    fn write_raw_record(&self, _record: &[u8]) -> i64 {
        self.write_raw_record_result
    }

    fn list_ramfs(&self, _buf: &mut [abi::RamfsListEntry], _max: usize) -> usize {
        self.list_ramfs_count
    }

    fn sleep_until(&self, _deadline: u64) -> Result<(), crate::kernel::error::KernelError> {
        Ok(())
    }
}
