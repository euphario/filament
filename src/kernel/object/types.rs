//! Type definitions and constants for the object system

/// Mux filter flags - imported from ABI (single source of truth)
pub use abi::mux_filter as filter;

/// Error codes - use abi::errno as single source of truth
pub use abi::errno;

#[repr(i32)]
#[derive(Clone, Copy, Debug)]
pub enum Error {
    /// Invalid handle
    BadHandle = errno::EBADF,
    /// Invalid argument
    InvalidArg = errno::EINVAL,
    /// Operation not supported for this type
    NotSupported = errno::ENOSYS,
    /// Would block (non-blocking mode)
    WouldBlock = errno::EAGAIN,
    /// No space / resource exhausted
    NoSpace = errno::ENOSPC,
    /// Peer closed
    PeerClosed = errno::EPIPE,
    /// Permission denied
    PermDenied = errno::EPERM,
    /// Not found
    NotFound = errno::ENOENT,
    /// Already exists
    Exists = errno::EEXIST,
    /// Bad address
    BadAddress = errno::EFAULT,
}

impl Error {
    pub fn to_errno(self) -> i64 {
        self as i64
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_bad_handle() {
        let e = Error::BadHandle;
        assert_eq!(e.to_errno(), errno::EBADF as i64);
    }

    #[test]
    fn test_error_invalid_arg() {
        let e = Error::InvalidArg;
        assert_eq!(e.to_errno(), errno::EINVAL as i64);
    }

    #[test]
    fn test_error_not_supported() {
        let e = Error::NotSupported;
        assert_eq!(e.to_errno(), errno::ENOSYS as i64);
    }

    #[test]
    fn test_error_would_block() {
        let e = Error::WouldBlock;
        assert_eq!(e.to_errno(), errno::EAGAIN as i64);
    }

    #[test]
    fn test_error_no_space() {
        let e = Error::NoSpace;
        assert_eq!(e.to_errno(), errno::ENOSPC as i64);
    }

    #[test]
    fn test_error_peer_closed() {
        let e = Error::PeerClosed;
        assert_eq!(e.to_errno(), errno::EPIPE as i64);
    }

    #[test]
    fn test_error_perm_denied() {
        let e = Error::PermDenied;
        assert_eq!(e.to_errno(), errno::EPERM as i64);
    }

    #[test]
    fn test_error_not_found() {
        let e = Error::NotFound;
        assert_eq!(e.to_errno(), errno::ENOENT as i64);
    }

    #[test]
    fn test_error_exists() {
        let e = Error::Exists;
        assert_eq!(e.to_errno(), errno::EEXIST as i64);
    }

    #[test]
    fn test_error_bad_address() {
        let e = Error::BadAddress;
        assert_eq!(e.to_errno(), errno::EFAULT as i64);
    }

    #[test]
    fn test_all_errors_negative() {
        // All error codes should be negative for errno convention
        assert!(Error::BadHandle.to_errno() < 0);
        assert!(Error::InvalidArg.to_errno() < 0);
        assert!(Error::NotSupported.to_errno() < 0);
        assert!(Error::WouldBlock.to_errno() < 0);
        assert!(Error::NoSpace.to_errno() < 0);
        assert!(Error::PeerClosed.to_errno() < 0);
        assert!(Error::PermDenied.to_errno() < 0);
        assert!(Error::NotFound.to_errno() < 0);
        assert!(Error::Exists.to_errno() < 0);
        assert!(Error::BadAddress.to_errno() < 0);
    }
}
