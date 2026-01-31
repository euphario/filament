//! System Error Types
//!
//! Canonical error type for syscall results. All syscall wrappers return
//! `Result<T, SysError>`. Domain-specific error types (IpcError, etc.)
//! can convert from SysError via `From` trait.

use core::fmt;
use abi::errno;

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
    /// EMFILE (24) - Too many open handles per process
    OutOfHandles,
    /// ENOSPC (28) - No space left on device
    NoSpace,
    /// ENOSYS (38) - Function not implemented
    NotImplemented,
    /// EMSGSIZE (90) - Message too long
    MessageTooLarge,
    /// EOPNOTSUPP (95) - Operation not supported
    OperationNotSupported,
    /// EPIPE (32) - Broken pipe / peer closed
    PeerClosed,
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
    pub fn from_errno(e: i32) -> Self {
        match e {
            errno::EPERM => SysError::PermissionDenied,
            errno::ENOENT => SysError::NotFound,
            errno::ESRCH => SysError::NoProcess,
            errno::EINTR => SysError::Interrupted,
            errno::EIO => SysError::IoError,
            errno::ENXIO => SysError::NoDevice,
            errno::ENOEXEC => SysError::ExecFormat,
            errno::EBADF => SysError::BadFd,
            errno::ECHILD => SysError::NoChild,
            errno::EAGAIN => SysError::WouldBlock,
            errno::ENOMEM => SysError::OutOfMemory,
            errno::EACCES => SysError::AccessDenied,
            errno::EFAULT => SysError::BadAddress,
            errno::EBUSY => SysError::Busy,
            errno::EEXIST => SysError::AlreadyExists,
            errno::ENODEV => SysError::NotSupported,
            errno::EINVAL => SysError::InvalidArgument,
            errno::ENFILE => SysError::TooManyFiles,
            errno::EMFILE => SysError::OutOfHandles,
            errno::ENOSPC => SysError::NoSpace,
            errno::ENOSYS => SysError::NotImplemented,
            errno::EMSGSIZE => SysError::MessageTooLarge,
            errno::EOPNOTSUPP => SysError::OperationNotSupported,
            errno::EPIPE => SysError::PeerClosed,
            errno::ECONNRESET => SysError::ConnectionReset,
            errno::ETIMEDOUT => SysError::Timeout,
            errno::ECONNREFUSED => SysError::ConnectionRefused,
            _ => SysError::Unknown(e),
        }
    }

    /// Convert to raw errno value (negative)
    pub fn to_errno(self) -> i32 {
        match self {
            SysError::PermissionDenied => errno::EPERM,
            SysError::NotFound => errno::ENOENT,
            SysError::NoProcess => errno::ESRCH,
            SysError::Interrupted => errno::EINTR,
            SysError::IoError => errno::EIO,
            SysError::NoDevice => errno::ENXIO,
            SysError::ExecFormat => errno::ENOEXEC,
            SysError::BadFd => errno::EBADF,
            SysError::NoChild => errno::ECHILD,
            SysError::WouldBlock => errno::EAGAIN,
            SysError::OutOfMemory => errno::ENOMEM,
            SysError::AccessDenied => errno::EACCES,
            SysError::BadAddress => errno::EFAULT,
            SysError::Busy => errno::EBUSY,
            SysError::AlreadyExists => errno::EEXIST,
            SysError::NotSupported => errno::ENODEV,
            SysError::InvalidArgument => errno::EINVAL,
            SysError::TooManyFiles => errno::ENFILE,
            SysError::OutOfHandles => errno::EMFILE,
            SysError::NoSpace => errno::ENOSPC,
            SysError::NotImplemented => errno::ENOSYS,
            SysError::MessageTooLarge => errno::EMSGSIZE,
            SysError::OperationNotSupported => errno::EOPNOTSUPP,
            SysError::PeerClosed => errno::EPIPE,
            SysError::ConnectionReset => errno::ECONNRESET,
            SysError::Timeout => errno::ETIMEDOUT,
            SysError::ConnectionRefused => errno::ECONNREFUSED,
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
            SysError::OutOfHandles => "too many handles",
            SysError::NoSpace => "no space",
            SysError::NotImplemented => "not implemented",
            SysError::MessageTooLarge => "message too large",
            SysError::OperationNotSupported => "operation not supported",
            SysError::PeerClosed => "peer closed",
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

