//! Core IPC Types
//!
//! This module defines the fundamental types for IPC:
//! - Message: Complete message with header and payload
//! - MessageHeader: Metadata (type, sender, ID, length)
//! - MessageType: What kind of message this is
//! - Constants: Size limits and capacity values

use super::super::process::Pid;

// ============================================================================
// Constants
// ============================================================================

/// Maximum inline message payload size
/// 576 bytes = 512 (sector) + 64 (headers/overhead)
pub const MAX_INLINE_PAYLOAD: usize = 576;

/// Maximum messages in a queue
pub const MAX_QUEUE_SIZE: usize = 8;

/// Maximum number of channels
/// Scales with MAX_BUSES: each bus needs ~4 channels (control + scheme opens)
pub const MAX_CHANNELS: usize = crate::kernel::bus::MAX_BUSES * 4 + 32;

/// Maximum number of ports
pub const MAX_PORTS: usize = 32;

/// Maximum port name length
pub const MAX_PORT_NAME: usize = 32;

/// Maximum pending connections per port
pub const MAX_PENDING_PER_PORT: usize = 4;

/// Channel ID type (0 = invalid)
pub type ChannelId = u32;

/// Port ID type (0 = invalid)
pub type PortId = u32;

/// Task ID type (matches Pid for compatibility)
pub type TaskId = u32;

// ============================================================================
// Message Type
// ============================================================================

/// Message types
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    /// Regular data message
    Data = 0,
    /// Request (expects reply)
    Request = 1,
    /// Reply to a request
    Reply = 2,
    /// Error response
    Error = 3,
    /// Channel closed notification
    Close = 4,
    /// Connect request (for ports)
    Connect = 5,
    /// Accept connection
    Accept = 6,
}

impl MessageType {
    /// Create from raw u32 value
    pub const fn from_raw(v: u32) -> Option<Self> {
        match v {
            0 => Some(Self::Data),
            1 => Some(Self::Request),
            2 => Some(Self::Reply),
            3 => Some(Self::Error),
            4 => Some(Self::Close),
            5 => Some(Self::Connect),
            6 => Some(Self::Accept),
            _ => None,
        }
    }
}

// ============================================================================
// Message Header
// ============================================================================

/// Message header containing metadata
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MessageHeader {
    /// Message type
    pub msg_type: MessageType,
    /// Sender PID
    pub sender: Pid,
    /// Message ID (for request/reply matching)
    pub msg_id: u32,
    /// Payload length (actual bytes used in payload)
    pub payload_len: u32,
    /// Flags (reserved for future use)
    pub flags: u32,
}

impl MessageHeader {
    /// Create a new header with minimal fields
    pub const fn new(msg_type: MessageType, sender: Pid) -> Self {
        Self {
            msg_type,
            sender,
            msg_id: 0,
            payload_len: 0,
            flags: 0,
        }
    }

    /// Create header for a data message
    pub const fn data(sender: Pid) -> Self {
        Self::new(MessageType::Data, sender)
    }

    /// Create header for a request message
    pub const fn request(sender: Pid, msg_id: u32) -> Self {
        Self {
            msg_type: MessageType::Request,
            sender,
            msg_id,
            payload_len: 0,
            flags: 0,
        }
    }

    /// Create header for a reply message
    pub const fn reply(sender: Pid, msg_id: u32) -> Self {
        Self {
            msg_type: MessageType::Reply,
            sender,
            msg_id,
            payload_len: 0,
            flags: 0,
        }
    }

    /// Create header for an error message
    pub const fn error(sender: Pid, msg_id: u32) -> Self {
        Self {
            msg_type: MessageType::Error,
            sender,
            msg_id,
            payload_len: 0,
            flags: 0,
        }
    }
}

impl Default for MessageHeader {
    fn default() -> Self {
        Self::new(MessageType::Data, 0)
    }
}

// ============================================================================
// Message
// ============================================================================

/// A complete message with header and inline payload
///
/// The payload is inline (not heap-allocated) for simplicity and
/// because most IPC messages are small. For large data transfers,
/// use shared memory.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Message {
    /// Message metadata
    pub header: MessageHeader,
    /// Inline payload buffer
    pub payload: [u8; MAX_INLINE_PAYLOAD],
}

impl Message {
    /// Create an empty message
    pub const fn new() -> Self {
        Self {
            header: MessageHeader::new(MessageType::Data, 0),
            payload: [0; MAX_INLINE_PAYLOAD],
        }
    }

    /// Create a data message with payload
    pub fn data(sender: Pid, data: &[u8]) -> Self {
        let mut msg = Self::new();
        msg.header.msg_type = MessageType::Data;
        msg.header.sender = sender;
        let len = core::cmp::min(data.len(), MAX_INLINE_PAYLOAD);
        msg.header.payload_len = len as u32;
        msg.payload[..len].copy_from_slice(&data[..len]);
        msg
    }

    /// Create a request message
    pub fn request(sender: Pid, msg_id: u32, data: &[u8]) -> Self {
        let mut msg = Self::data(sender, data);
        msg.header.msg_type = MessageType::Request;
        msg.header.msg_id = msg_id;
        msg
    }

    /// Create a reply message
    pub fn reply(sender: Pid, msg_id: u32, data: &[u8]) -> Self {
        let mut msg = Self::data(sender, data);
        msg.header.msg_type = MessageType::Reply;
        msg.header.msg_id = msg_id;
        msg
    }

    /// Create an error message
    pub fn error(sender: Pid, msg_id: u32, error_code: i32) -> Self {
        let mut msg = Self::new();
        msg.header.msg_type = MessageType::Error;
        msg.header.sender = sender;
        msg.header.msg_id = msg_id;
        msg.header.payload_len = 4;
        msg.payload[0..4].copy_from_slice(&error_code.to_le_bytes());
        msg
    }

    /// Create a close notification message
    pub fn close(sender: Pid) -> Self {
        let mut msg = Self::new();
        msg.header.msg_type = MessageType::Close;
        msg.header.sender = sender;
        msg
    }

    /// Get payload as slice (clamped to actual length)
    ///
    /// Returns the valid portion of the payload buffer. The length is clamped
    /// to MAX_INLINE_PAYLOAD to prevent out-of-bounds access if payload_len
    /// is corrupted.
    pub fn payload_slice(&self) -> &[u8] {
        let len = core::cmp::min(self.header.payload_len as usize, MAX_INLINE_PAYLOAD);
        &self.payload[..len]
    }

    /// Get mutable payload slice
    pub fn payload_slice_mut(&mut self) -> &mut [u8] {
        let len = core::cmp::min(self.header.payload_len as usize, MAX_INLINE_PAYLOAD);
        &mut self.payload[..len]
    }

    /// Set payload from slice
    pub fn set_payload(&mut self, data: &[u8]) {
        let len = core::cmp::min(data.len(), MAX_INLINE_PAYLOAD);
        self.payload[..len].copy_from_slice(&data[..len]);
        self.header.payload_len = len as u32;
    }

    /// Check if this is a close notification
    pub fn is_close(&self) -> bool {
        self.header.msg_type == MessageType::Close
    }

    /// Get error code from error message
    pub fn error_code(&self) -> Option<i32> {
        if self.header.msg_type == MessageType::Error && self.header.payload_len >= 4 {
            Some(i32::from_le_bytes([
                self.payload[0],
                self.payload[1],
                self.payload[2],
                self.payload[3],
            ]))
        } else {
            None
        }
    }
}

impl Default for Message {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Debug for Message {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Message")
            .field("header", &self.header)
            .field("payload_len", &self.header.payload_len)
            .finish_non_exhaustive()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_type_from_raw() {
        assert_eq!(MessageType::from_raw(0), Some(MessageType::Data));
        assert_eq!(MessageType::from_raw(1), Some(MessageType::Request));
        assert_eq!(MessageType::from_raw(6), Some(MessageType::Accept));
        assert_eq!(MessageType::from_raw(7), None);
        assert_eq!(MessageType::from_raw(255), None);
    }

    #[test]
    fn test_message_data() {
        let msg = Message::data(42, b"hello");
        assert_eq!(msg.header.msg_type, MessageType::Data);
        assert_eq!(msg.header.sender, 42);
        assert_eq!(msg.header.payload_len, 5);
        assert_eq!(msg.payload_slice(), b"hello");
    }

    #[test]
    fn test_message_request_reply() {
        let req = Message::request(1, 100, b"request");
        assert_eq!(req.header.msg_type, MessageType::Request);
        assert_eq!(req.header.msg_id, 100);

        let rep = Message::reply(2, 100, b"reply");
        assert_eq!(rep.header.msg_type, MessageType::Reply);
        assert_eq!(rep.header.msg_id, 100);
    }

    #[test]
    fn test_message_error() {
        let err = Message::error(1, 42, -99);
        assert_eq!(err.header.msg_type, MessageType::Error);
        assert_eq!(err.error_code(), Some(-99));
    }

    #[test]
    fn test_message_payload_clamping() {
        let mut msg = Message::new();
        msg.header.payload_len = 9999; // Corrupted value
        assert_eq!(msg.payload_slice().len(), MAX_INLINE_PAYLOAD);
    }

    #[test]
    fn test_message_set_payload() {
        let mut msg = Message::new();
        msg.set_payload(b"test data");
        assert_eq!(msg.header.payload_len, 9);
        assert_eq!(msg.payload_slice(), b"test data");
    }
}
