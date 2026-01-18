//! IPC Error Types
//!
//! Comprehensive error handling for IPC operations.

use core::fmt;
use crate::error::SysError;

/// Result type for IPC operations
pub type IpcResult<T> = Result<T, IpcError>;

/// IPC error codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IpcError {
    // Connection errors
    /// Port not found (service not running)
    PortNotFound,
    /// Connection refused by server
    ConnectionRefused,
    /// Already connected
    AlreadyConnected,
    /// Not connected
    NotConnected,

    // Channel errors
    /// Channel was closed by peer
    ChannelClosed,
    /// Channel ID is invalid
    InvalidChannel,
    /// Send queue is full
    SendQueueFull,
    /// Receive queue is empty (non-blocking)
    WouldBlock,

    // Message errors
    /// Message too large
    MessageTooLarge,
    /// Message malformed (failed to deserialize)
    MalformedMessage,
    /// Unexpected message type
    UnexpectedMessage,
    /// Message truncated
    Truncated,

    // Timeout errors
    /// Operation timed out
    Timeout,
    /// Deadline exceeded
    DeadlineExceeded,

    // Resource errors
    /// Out of memory
    OutOfMemory,
    /// Too many open channels
    TooManyChannels,
    /// Resource busy
    ResourceBusy,

    // Protocol errors
    /// Handshake failed
    HandshakeFailed,
    /// Protocol version mismatch
    VersionMismatch,
    /// Invalid request
    InvalidRequest,
    /// Server returned error
    ServerError(i32),

    // System errors
    /// Permission denied
    PermissionDenied,
    /// Internal error
    Internal,
    /// Unknown error with raw errno
    Unknown(i32),
}

impl IpcError {
    /// Create from raw syscall error code
    pub fn from_errno(errno: i64) -> Self {
        match errno {
            -2 => IpcError::PortNotFound,      // ENOENT
            -9 => IpcError::InvalidChannel,    // EBADF
            -11 => IpcError::WouldBlock,       // EAGAIN/EWOULDBLOCK
            -12 => IpcError::OutOfMemory,      // ENOMEM
            -13 => IpcError::PermissionDenied, // EACCES
            -16 => IpcError::ResourceBusy,     // EBUSY
            -22 => IpcError::InvalidRequest,   // EINVAL
            -23 => IpcError::TooManyChannels,  // ENFILE
            -90 => IpcError::MessageTooLarge,  // EMSGSIZE
            -104 => IpcError::ChannelClosed,   // ECONNRESET
            -110 => IpcError::Timeout,         // ETIMEDOUT
            -111 => IpcError::ConnectionRefused, // ECONNREFUSED
            e => IpcError::Unknown(e as i32),
        }
    }

    /// Convert to raw errno (for compatibility)
    pub fn to_errno(&self) -> i32 {
        match self {
            IpcError::PortNotFound => -2,
            IpcError::InvalidChannel => -9,
            IpcError::WouldBlock => -11,
            IpcError::OutOfMemory => -12,
            IpcError::PermissionDenied => -13,
            IpcError::ResourceBusy => -16,
            IpcError::InvalidRequest => -22,
            IpcError::TooManyChannels => -23,
            IpcError::MessageTooLarge => -90,
            IpcError::ChannelClosed => -104,
            IpcError::Timeout => -110,
            IpcError::ConnectionRefused => -111,
            IpcError::Unknown(e) => *e,
            _ => -1, // Generic error
        }
    }

    /// Check if error is recoverable (worth retrying)
    pub fn is_recoverable(&self) -> bool {
        matches!(
            self,
            IpcError::WouldBlock
                | IpcError::Timeout
                | IpcError::ResourceBusy
                | IpcError::SendQueueFull
        )
    }

    /// Check if connection is still usable after this error
    pub fn is_fatal(&self) -> bool {
        matches!(
            self,
            IpcError::ChannelClosed
                | IpcError::InvalidChannel
                | IpcError::ConnectionRefused
                | IpcError::PortNotFound
        )
    }
}

impl fmt::Display for IpcError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IpcError::PortNotFound => write!(f, "port not found"),
            IpcError::ConnectionRefused => write!(f, "connection refused"),
            IpcError::AlreadyConnected => write!(f, "already connected"),
            IpcError::NotConnected => write!(f, "not connected"),
            IpcError::ChannelClosed => write!(f, "channel closed"),
            IpcError::InvalidChannel => write!(f, "invalid channel"),
            IpcError::SendQueueFull => write!(f, "send queue full"),
            IpcError::WouldBlock => write!(f, "would block"),
            IpcError::MessageTooLarge => write!(f, "message too large"),
            IpcError::MalformedMessage => write!(f, "malformed message"),
            IpcError::UnexpectedMessage => write!(f, "unexpected message"),
            IpcError::Truncated => write!(f, "message truncated"),
            IpcError::Timeout => write!(f, "timeout"),
            IpcError::DeadlineExceeded => write!(f, "deadline exceeded"),
            IpcError::OutOfMemory => write!(f, "out of memory"),
            IpcError::TooManyChannels => write!(f, "too many channels"),
            IpcError::ResourceBusy => write!(f, "resource busy"),
            IpcError::HandshakeFailed => write!(f, "handshake failed"),
            IpcError::VersionMismatch => write!(f, "version mismatch"),
            IpcError::InvalidRequest => write!(f, "invalid request"),
            IpcError::ServerError(code) => write!(f, "server error: {}", code),
            IpcError::PermissionDenied => write!(f, "permission denied"),
            IpcError::Internal => write!(f, "internal error"),
            IpcError::Unknown(e) => write!(f, "unknown error: {}", e),
        }
    }
}

impl From<SysError> for IpcError {
    fn from(e: SysError) -> Self {
        match e {
            SysError::NotFound => IpcError::PortNotFound,
            SysError::BadFd => IpcError::InvalidChannel,
            SysError::WouldBlock => IpcError::WouldBlock,
            SysError::OutOfMemory => IpcError::OutOfMemory,
            SysError::PermissionDenied | SysError::AccessDenied => IpcError::PermissionDenied,
            SysError::Busy => IpcError::ResourceBusy,
            SysError::InvalidArgument => IpcError::InvalidRequest,
            SysError::TooManyFiles => IpcError::TooManyChannels,
            SysError::MessageTooLarge => IpcError::MessageTooLarge,
            SysError::ConnectionReset => IpcError::ChannelClosed,
            SysError::Timeout => IpcError::Timeout,
            SysError::ConnectionRefused => IpcError::ConnectionRefused,
            _ => IpcError::Unknown(e.to_errno()),
        }
    }
}
