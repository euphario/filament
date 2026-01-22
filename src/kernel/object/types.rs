//! Type definitions and constants for the object system

/// Mux filter flags - imported from ABI (single source of truth)
pub use abi::mux_filter as filter;

/// Mux types - imported from ABI (single source of truth)
pub use abi::{MuxCommand, MuxAddWatch};

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
