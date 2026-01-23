//! IPC Errors
//!
//! Typed errors with full context for debugging.
//! Each error variant includes relevant information about what failed.

use super::types::{ChannelId, TaskId, MAX_PORT_NAME};

/// IPC error type with full context
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum IpcError {
    // ========================================================================
    // Channel Errors
    // ========================================================================

    /// Channel ID not found in table
    InvalidChannel {
        id: ChannelId,
    },

    /// Channel is closed (fully terminated)
    Closed,

    /// Peer has closed their end (half-closed state)
    PeerClosed,

    /// Not the owner of this channel
    NotOwner {
        channel: ChannelId,
        owner: TaskId,
        caller: TaskId,
    },

    // ========================================================================
    // Queue Errors
    // ========================================================================

    /// Message queue is full (backpressure)
    QueueFull,

    /// No message available (non-blocking receive)
    WouldBlock,

    /// Message payload exceeds maximum size
    MessageTooLarge {
        size: usize,
        max: usize,
    },

    // ========================================================================
    // Port Errors
    // ========================================================================

    /// Port name already registered
    PortExists {
        name: [u8; MAX_PORT_NAME],
        name_len: u8,
    },

    /// Port not found by name
    PortNotFound,

    /// Port ID not found
    InvalidPort {
        id: u32,
    },

    /// Not the owner of this port
    PortNotOwner {
        port_id: u32,
        owner: TaskId,
        caller: TaskId,
    },

    /// No pending connections to accept
    NoPending,

    /// Too many pending connections
    PendingFull,

    // ========================================================================
    // Resource Errors
    // ========================================================================

    /// No space for new channels
    NoChannelSpace,

    /// No space for new ports
    NoPortSpace,

    /// Subscriber set is full (cannot add more subscribers)
    SubscribersFull,

    // ========================================================================
    // Handle Errors
    // ========================================================================

    /// Stale handle (generation mismatch)
    StaleHandle {
        expected: u32,
        got: u32,
    },

    /// Invalid handle
    InvalidHandle,

    // ========================================================================
    // Operation Errors
    // ========================================================================

    /// Operation not supported on this object type
    NotSupported,

    /// Timeout expired
    Timeout,

    /// Operation was interrupted
    Interrupted,
}

impl IpcError {
    /// Convert to errno for syscall return
    ///
    /// Errno values follow POSIX conventions where applicable.
    pub fn to_errno(&self) -> i64 {
        match self {
            // EBADF (9) - Bad file descriptor
            IpcError::InvalidChannel { .. } => -9,
            IpcError::InvalidPort { .. } => -9,
            IpcError::StaleHandle { .. } => -9,
            IpcError::InvalidHandle => -9,

            // ECONNRESET (104) - Connection reset by peer
            IpcError::Closed => -104,
            IpcError::PeerClosed => -104,

            // EAGAIN (11) - Resource temporarily unavailable
            IpcError::QueueFull => -11,
            IpcError::WouldBlock => -11,
            IpcError::NoPending => -11,

            // EACCES (13) - Permission denied
            IpcError::NotOwner { .. } => -13,
            IpcError::PortNotOwner { .. } => -13,

            // EMSGSIZE (90) - Message too long
            IpcError::MessageTooLarge { .. } => -90,

            // EEXIST (17) - File exists
            IpcError::PortExists { .. } => -17,

            // ENOENT (2) - No such file or directory
            IpcError::PortNotFound => -2,

            // ENOSPC (28) - No space left on device
            IpcError::NoChannelSpace => -28,
            IpcError::NoPortSpace => -28,
            IpcError::PendingFull => -28,
            IpcError::SubscribersFull => -28,

            // ENOTSUP (95) - Operation not supported
            IpcError::NotSupported => -95,

            // ETIMEDOUT (110) - Connection timed out
            IpcError::Timeout => -110,

            // EINTR (4) - Interrupted system call
            IpcError::Interrupted => -4,
        }
    }

    /// Get a short string description
    pub fn as_str(&self) -> &'static str {
        match self {
            IpcError::InvalidChannel { .. } => "invalid channel",
            IpcError::Closed => "channel closed",
            IpcError::PeerClosed => "peer closed",
            IpcError::NotOwner { .. } => "not owner",
            IpcError::QueueFull => "queue full",
            IpcError::WouldBlock => "would block",
            IpcError::MessageTooLarge { .. } => "message too large",
            IpcError::PortExists { .. } => "port exists",
            IpcError::PortNotFound => "port not found",
            IpcError::InvalidPort { .. } => "invalid port",
            IpcError::PortNotOwner { .. } => "port not owner",
            IpcError::NoPending => "no pending",
            IpcError::PendingFull => "pending full",
            IpcError::NoChannelSpace => "no channel space",
            IpcError::NoPortSpace => "no port space",
            IpcError::SubscribersFull => "subscribers full",
            IpcError::StaleHandle { .. } => "stale handle",
            IpcError::InvalidHandle => "invalid handle",
            IpcError::NotSupported => "not supported",
            IpcError::Timeout => "timeout",
            IpcError::Interrupted => "interrupted",
        }
    }

    /// Create a NotOwner error
    pub fn not_owner(channel: ChannelId, owner: TaskId, caller: TaskId) -> Self {
        Self::NotOwner { channel, owner, caller }
    }

    /// Create a PortNotOwner error
    pub fn port_not_owner(port_id: u32, owner: TaskId, caller: TaskId) -> Self {
        Self::PortNotOwner { port_id, owner, caller }
    }

    /// Create a PortExists error from a name slice
    pub fn port_exists(name: &[u8]) -> Self {
        let mut name_arr = [0u8; MAX_PORT_NAME];
        let len = core::cmp::min(name.len(), MAX_PORT_NAME);
        name_arr[..len].copy_from_slice(&name[..len]);
        Self::PortExists { name: name_arr, name_len: len as u8 }
    }
}

impl core::fmt::Display for IpcError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            IpcError::InvalidChannel { id } => {
                write!(f, "invalid channel: {}", id)
            }
            IpcError::Closed => write!(f, "channel closed"),
            IpcError::PeerClosed => write!(f, "peer closed"),
            IpcError::NotOwner { channel, owner, caller } => {
                write!(f, "channel {} owned by {}, caller {}", channel, owner, caller)
            }
            IpcError::QueueFull => write!(f, "message queue full"),
            IpcError::WouldBlock => write!(f, "operation would block"),
            IpcError::MessageTooLarge { size, max } => {
                write!(f, "message {} bytes exceeds max {}", size, max)
            }
            IpcError::PortExists { name, name_len } => {
                let name_str = core::str::from_utf8(&name[..*name_len as usize]).unwrap_or("<invalid>");
                write!(f, "port '{}' already exists", name_str)
            }
            IpcError::PortNotFound => write!(f, "port not found"),
            IpcError::InvalidPort { id } => write!(f, "invalid port: {}", id),
            IpcError::PortNotOwner { port_id, owner, caller } => {
                write!(f, "port {} owned by {}, caller {}", port_id, owner, caller)
            }
            IpcError::NoPending => write!(f, "no pending connections"),
            IpcError::PendingFull => write!(f, "too many pending connections"),
            IpcError::NoChannelSpace => write!(f, "no space for new channels"),
            IpcError::NoPortSpace => write!(f, "no space for new ports"),
            IpcError::SubscribersFull => write!(f, "subscriber set is full"),
            IpcError::StaleHandle { expected, got } => {
                write!(f, "stale handle: expected gen {}, got {}", expected, got)
            }
            IpcError::InvalidHandle => write!(f, "invalid handle"),
            IpcError::NotSupported => write!(f, "operation not supported"),
            IpcError::Timeout => write!(f, "operation timed out"),
            IpcError::Interrupted => write!(f, "operation interrupted"),
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
    fn test_errno_values() {
        assert_eq!(IpcError::InvalidChannel { id: 0 }.to_errno(), -9);
        assert_eq!(IpcError::Closed.to_errno(), -104);
        assert_eq!(IpcError::QueueFull.to_errno(), -11);
        assert_eq!(IpcError::WouldBlock.to_errno(), -11);
        assert_eq!(IpcError::PortNotFound.to_errno(), -2);
        assert_eq!(IpcError::port_exists(b"test").to_errno(), -17);
    }

    #[test]
    fn test_error_messages() {
        assert_eq!(IpcError::Closed.as_str(), "channel closed");
        assert_eq!(IpcError::PeerClosed.as_str(), "peer closed");
        assert_eq!(IpcError::QueueFull.as_str(), "queue full");
    }

    #[test]
    fn test_port_exists_from_name() {
        let err = IpcError::port_exists(b"test_port:");
        match err {
            IpcError::PortExists { name, name_len } => {
                assert_eq!(name_len, 10);
                assert_eq!(&name[..10], b"test_port:");
            }
            _ => panic!("wrong error type"),
        }
    }

    #[test]
    fn test_not_owner() {
        let err = IpcError::not_owner(42, 1, 2);
        match err {
            IpcError::NotOwner { channel, owner, caller } => {
                assert_eq!(channel, 42);
                assert_eq!(owner, 1);
                assert_eq!(caller, 2);
            }
            _ => panic!("wrong error type"),
        }
    }
}
