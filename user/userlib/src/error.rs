//! System Error Types
//!
//! Canonical error type for syscall results. All syscall wrappers return
//! `Result<T, SysError>`. Domain-specific error types (IpcError, etc.)
//! can convert from SysError via `From` trait.

use core::fmt;
use crate::serialize::{Serialize, ValueType};

/// Result type for syscall operations
pub type SysResult<T> = Result<T, SysError>;

/// System error codes (POSIX errno values)
///
/// This is the canonical error type for syscall returns. Domain-specific
/// errors should implement `From<SysError>` for convenient conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SysError {
    /// EPERM (1) - Operation not permitted
    PermissionDenied,
    /// ENOENT (2) - No such file or directory
    NotFound,
    /// ESRCH (3) - No such process
    NoProcess,
    /// EINTR (4) - Interrupted system call
    Interrupted,
    /// EIO (5) - I/O error
    IoError,
    /// ENXIO (6) - No such device or address
    NoDevice,
    /// ENOEXEC (8) - Exec format error
    ExecFormat,
    /// EBADF (9) - Bad file descriptor
    BadFd,
    /// ECHILD (10) - No child processes
    NoChild,
    /// EAGAIN (11) - Resource temporarily unavailable
    WouldBlock,
    /// ENOMEM (12) - Out of memory
    OutOfMemory,
    /// EACCES (13) - Permission denied (access)
    AccessDenied,
    /// EFAULT (14) - Bad address
    BadAddress,
    /// EBUSY (16) - Device or resource busy
    Busy,
    /// EEXIST (17) - File/resource already exists
    AlreadyExists,
    /// ENODEV (19) - No such device
    NotSupported,
    /// EINVAL (22) - Invalid argument
    InvalidArgument,
    /// ENFILE (23) - Too many open files in system
    TooManyFiles,
    /// ENOSPC (28) - No space left on device
    NoSpace,
    /// ENOSYS (38) - Function not implemented
    NotImplemented,
    /// EMSGSIZE (90) - Message too long
    MessageTooLarge,
    /// EOPNOTSUPP (95) - Operation not supported
    OperationNotSupported,
    /// ECONNRESET (104) - Connection reset by peer
    ConnectionReset,
    /// ETIMEDOUT (110) - Connection timed out
    Timeout,
    /// ECONNREFUSED (111) - Connection refused
    ConnectionRefused,
    /// Unknown error with raw errno
    Unknown(i32),
}

impl SysError {
    /// Create from raw errno value (negative)
    pub fn from_errno(errno: i32) -> Self {
        match errno {
            -1 => SysError::PermissionDenied,
            -2 => SysError::NotFound,
            -3 => SysError::NoProcess,
            -4 => SysError::Interrupted,
            -5 => SysError::IoError,
            -6 => SysError::NoDevice,
            -8 => SysError::ExecFormat,
            -9 => SysError::BadFd,
            -10 => SysError::NoChild,
            -11 => SysError::WouldBlock,
            -12 => SysError::OutOfMemory,
            -13 => SysError::AccessDenied,
            -14 => SysError::BadAddress,
            -16 => SysError::Busy,
            -17 => SysError::AlreadyExists,
            -19 => SysError::NotSupported,
            -22 => SysError::InvalidArgument,
            -23 => SysError::TooManyFiles,
            -28 => SysError::NoSpace,
            -38 => SysError::NotImplemented,
            -90 => SysError::MessageTooLarge,
            -95 => SysError::OperationNotSupported,
            -104 => SysError::ConnectionReset,
            -110 => SysError::Timeout,
            -111 => SysError::ConnectionRefused,
            e => SysError::Unknown(e),
        }
    }

    /// Convert to raw errno value (negative)
    pub fn to_errno(self) -> i32 {
        match self {
            SysError::PermissionDenied => -1,
            SysError::NotFound => -2,
            SysError::NoProcess => -3,
            SysError::Interrupted => -4,
            SysError::IoError => -5,
            SysError::NoDevice => -6,
            SysError::ExecFormat => -8,
            SysError::BadFd => -9,
            SysError::NoChild => -10,
            SysError::WouldBlock => -11,
            SysError::OutOfMemory => -12,
            SysError::AccessDenied => -13,
            SysError::BadAddress => -14,
            SysError::Busy => -16,
            SysError::AlreadyExists => -17,
            SysError::NotSupported => -19,
            SysError::InvalidArgument => -22,
            SysError::TooManyFiles => -23,
            SysError::NoSpace => -28,
            SysError::NotImplemented => -38,
            SysError::MessageTooLarge => -90,
            SysError::OperationNotSupported => -95,
            SysError::ConnectionReset => -104,
            SysError::Timeout => -110,
            SysError::ConnectionRefused => -111,
            SysError::Unknown(e) => e,
        }
    }

    /// Check if error is transient (worth retrying)
    pub fn is_transient(&self) -> bool {
        matches!(
            self,
            SysError::WouldBlock | SysError::Interrupted | SysError::Busy | SysError::Timeout
        )
    }

    /// Short description of the error
    pub fn as_str(&self) -> &'static str {
        match self {
            SysError::PermissionDenied => "permission denied",
            SysError::NotFound => "not found",
            SysError::NoProcess => "no such process",
            SysError::Interrupted => "interrupted",
            SysError::IoError => "I/O error",
            SysError::NoDevice => "no such device",
            SysError::ExecFormat => "exec format error",
            SysError::BadFd => "bad file descriptor",
            SysError::NoChild => "no child processes",
            SysError::WouldBlock => "would block",
            SysError::OutOfMemory => "out of memory",
            SysError::AccessDenied => "access denied",
            SysError::BadAddress => "bad address",
            SysError::Busy => "resource busy",
            SysError::AlreadyExists => "already exists",
            SysError::NotSupported => "not supported",
            SysError::InvalidArgument => "invalid argument",
            SysError::TooManyFiles => "too many files",
            SysError::NoSpace => "no space",
            SysError::NotImplemented => "not implemented",
            SysError::MessageTooLarge => "message too large",
            SysError::OperationNotSupported => "operation not supported",
            SysError::ConnectionReset => "connection reset",
            SysError::Timeout => "timeout",
            SysError::ConnectionRefused => "connection refused",
            SysError::Unknown(_) => "unknown error",
        }
    }
}

impl fmt::Display for SysError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SysError::Unknown(e) => write!(f, "error {}", e),
            _ => write!(f, "{}", self.as_str()),
        }
    }
}

/// Helper to convert raw syscall result to SysResult
///
/// Syscalls return i64 where negative = error, non-negative = success.
#[inline]
pub fn check_errno(result: i64) -> SysResult<u64> {
    if result < 0 {
        Err(SysError::from_errno(result as i32))
    } else {
        Ok(result as u64)
    }
}

/// Helper to convert raw syscall result to SysResult with i32 errno
#[inline]
pub fn check_errno_i32(result: i32) -> SysResult<u32> {
    if result < 0 {
        Err(SysError::from_errno(result))
    } else {
        Ok(result as u32)
    }
}

// Implement Serialize for structured logging (ulog)
impl Serialize for SysError {
    fn type_marker(&self) -> u8 {
        ValueType::Str as u8
    }

    fn serialized_size(&self) -> usize {
        // 2 bytes length prefix + string bytes
        2 + self.as_str().len()
    }

    fn serialize(&self, buf: &mut [u8]) -> usize {
        let s = self.as_str();
        let len = s.len();
        if buf.len() < 2 + len {
            return 0;
        }
        buf[0] = (len & 0xFF) as u8;
        buf[1] = ((len >> 8) & 0xFF) as u8;
        buf[2..2 + len].copy_from_slice(s.as_bytes());
        2 + len
    }
}
