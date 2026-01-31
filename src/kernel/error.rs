//! Unified Kernel Error Type
//!
//! Single error type for all kernel operations that cross the syscall boundary.
//! All values map directly to POSIX errno (from `abi::errno`).
//!
//! # Design
//!
//! - ONE error type replaces Error, ObjError, ObjectError, SyscallError
//! - IpcError stays as internal diagnostic type (carries context)
//! - `#[repr(i32)]` so `to_errno()` is a trivial cast
//!
//! # Migration
//!
//! Phase 1 (this file): Define KernelError, coexist with old types.
//! Phase 4: Replace all old error types with KernelError.

/// Error codes - use abi::errno as single source of truth
use abi::errno;

/// Unified kernel error type.
///
/// Every variant's discriminant IS the errno value, so `to_errno()` is
/// a zero-cost cast. This is the only error type that crosses the
/// kernel/userspace boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum KernelError {
    // ── Handle/resource errors ──
    /// Invalid handle (EBADF)
    BadHandle    = errno::EBADF,       // -9
    /// Not found (ENOENT)
    NotFound     = errno::ENOENT,      // -2
    /// Already exists (EEXIST)
    Exists       = errno::EEXIST,      // -17
    /// No space / resource exhausted (ENOSPC)
    NoSpace      = errno::ENOSPC,      // -28
    /// Out of memory (ENOMEM)
    OutOfMemory  = errno::ENOMEM,      // -12
    /// Out of handles / too many open files (EMFILE)
    OutOfHandles = errno::EMFILE,      // -24
    /// No such process (ESRCH)
    NoProcess    = errno::ESRCH,       // -3
    /// No child processes (ECHILD)
    NoChild      = errno::ECHILD,      // -10

    // ── Argument/operation errors ──
    /// Invalid argument (EINVAL)
    InvalidArg   = errno::EINVAL,      // -22
    /// Would block / try again (EAGAIN)
    WouldBlock   = errno::EAGAIN,      // -11
    /// Timeout expired (ETIMEDOUT)
    Timeout      = errno::ETIMEDOUT,   // -110
    /// Not implemented / not supported (ENOSYS)
    NotSupported = errno::ENOSYS,      // -38

    // ── Permission errors ──
    /// Operation not permitted (EPERM)
    PermDenied   = errno::EPERM,       // -1
    /// Access denied (EACCES)
    AccessDenied = errno::EACCES,      // -13

    // ── Connection/peer errors ──
    /// Peer closed / broken pipe (EPIPE)
    PeerClosed   = errno::EPIPE,       // -32
    /// Connection reset (ECONNRESET)
    ConnReset    = errno::ECONNRESET,  // -104
    /// Connection refused (ECONNREFUSED)
    ConnRefused  = errno::ECONNREFUSED,// -111

    // ── I/O errors ──
    /// Bad user address (EFAULT)
    BadAddress   = errno::EFAULT,      // -14
    /// I/O error (EIO)
    Io           = errno::EIO,         // -5
    /// Message too large (EMSGSIZE)
    MsgTooLarge  = errno::EMSGSIZE,    // -90
    /// Resource busy (EBUSY)
    Busy         = errno::EBUSY,       // -16
}

impl KernelError {
    /// Convert to errno value for returning to userspace.
    /// Zero-cost: the enum discriminant IS the errno.
    pub fn to_errno(self) -> i64 {
        self as i64
    }

    /// Convert from an errno value to a KernelError.
    /// Used when interfacing with subsystems that return raw errno.
    pub fn from_errno(e: i64) -> Self {
        match e {
            e if e == errno::EBADF as i64 => KernelError::BadHandle,
            e if e == errno::ENOENT as i64 => KernelError::NotFound,
            e if e == errno::EEXIST as i64 => KernelError::Exists,
            e if e == errno::ENOSPC as i64 => KernelError::NoSpace,
            e if e == errno::ENOMEM as i64 => KernelError::OutOfMemory,
            e if e == errno::EMFILE as i64 => KernelError::OutOfHandles,
            e if e == errno::ESRCH as i64 => KernelError::NoProcess,
            e if e == errno::ECHILD as i64 => KernelError::NoChild,
            e if e == errno::EINVAL as i64 => KernelError::InvalidArg,
            e if e == errno::EAGAIN as i64 => KernelError::WouldBlock,
            e if e == errno::ETIMEDOUT as i64 => KernelError::Timeout,
            e if e == errno::ENOSYS as i64 => KernelError::NotSupported,
            e if e == errno::EPERM as i64 => KernelError::PermDenied,
            e if e == errno::EACCES as i64 => KernelError::AccessDenied,
            e if e == errno::EPIPE as i64 => KernelError::PeerClosed,
            e if e == errno::ECONNRESET as i64 => KernelError::ConnReset,
            e if e == errno::ECONNREFUSED as i64 => KernelError::ConnRefused,
            e if e == errno::EFAULT as i64 => KernelError::BadAddress,
            e if e == errno::EIO as i64 => KernelError::Io,
            e if e == errno::EMSGSIZE as i64 => KernelError::MsgTooLarge,
            e if e == errno::EBUSY as i64 => KernelError::Busy,
            _ => KernelError::InvalidArg, // Default fallback
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_all_errors_negative() {
        let errors = [
            KernelError::BadHandle,
            KernelError::NotFound,
            KernelError::Exists,
            KernelError::NoSpace,
            KernelError::OutOfMemory,
            KernelError::OutOfHandles,
            KernelError::NoProcess,
            KernelError::NoChild,
            KernelError::InvalidArg,
            KernelError::WouldBlock,
            KernelError::Timeout,
            KernelError::NotSupported,
            KernelError::PermDenied,
            KernelError::AccessDenied,
            KernelError::PeerClosed,
            KernelError::ConnReset,
            KernelError::ConnRefused,
            KernelError::BadAddress,
            KernelError::Io,
            KernelError::MsgTooLarge,
            KernelError::Busy,
        ];
        for e in &errors {
            assert!(e.to_errno() < 0, "{:?} should be negative", e);
        }
    }

    #[test]
    fn test_errno_values_match_abi() {
        assert_eq!(KernelError::BadHandle.to_errno(), errno::EBADF as i64);
        assert_eq!(KernelError::NotFound.to_errno(), errno::ENOENT as i64);
        assert_eq!(KernelError::Exists.to_errno(), errno::EEXIST as i64);
        assert_eq!(KernelError::NoSpace.to_errno(), errno::ENOSPC as i64);
        assert_eq!(KernelError::OutOfMemory.to_errno(), errno::ENOMEM as i64);
        assert_eq!(KernelError::OutOfHandles.to_errno(), errno::EMFILE as i64);
        assert_eq!(KernelError::NoProcess.to_errno(), errno::ESRCH as i64);
        assert_eq!(KernelError::NoChild.to_errno(), errno::ECHILD as i64);
        assert_eq!(KernelError::InvalidArg.to_errno(), errno::EINVAL as i64);
        assert_eq!(KernelError::WouldBlock.to_errno(), errno::EAGAIN as i64);
        assert_eq!(KernelError::Timeout.to_errno(), errno::ETIMEDOUT as i64);
        assert_eq!(KernelError::NotSupported.to_errno(), errno::ENOSYS as i64);
        assert_eq!(KernelError::PermDenied.to_errno(), errno::EPERM as i64);
        assert_eq!(KernelError::AccessDenied.to_errno(), errno::EACCES as i64);
        assert_eq!(KernelError::PeerClosed.to_errno(), errno::EPIPE as i64);
        assert_eq!(KernelError::ConnReset.to_errno(), errno::ECONNRESET as i64);
        assert_eq!(KernelError::ConnRefused.to_errno(), errno::ECONNREFUSED as i64);
        assert_eq!(KernelError::BadAddress.to_errno(), errno::EFAULT as i64);
        assert_eq!(KernelError::Io.to_errno(), errno::EIO as i64);
        assert_eq!(KernelError::MsgTooLarge.to_errno(), errno::EMSGSIZE as i64);
        assert_eq!(KernelError::Busy.to_errno(), errno::EBUSY as i64);
    }

    #[test]
    fn test_timeout_is_new() {
        // Timeout (-110) is new in KernelError, didn't exist in old Error enum
        assert_eq!(KernelError::Timeout.to_errno(), -110);
    }

    #[test]
    fn test_size() {
        // Should be 4 bytes (i32 repr)
        assert_eq!(core::mem::size_of::<KernelError>(), 4);
    }
}
