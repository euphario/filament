//! Protocol Trait and Serialization
//!
//! Defines the Protocol trait for type-safe IPC and simple
//! serialization primitives for no_std environments.

use super::error::{IpcError, IpcResult};
use super::MAX_MESSAGE_SIZE;

/// Protocol definition trait
///
/// Implement this trait to define a type-safe IPC protocol between
/// a client and server.
///
/// # Example
///
/// ```rust
/// pub struct MyProtocol;
///
/// impl Protocol for MyProtocol {
///     type Request = MyRequest;
///     type Response = MyResponse;
///     const PORT_NAME: &'static [u8] = b"myservice";
/// }
/// ```
pub trait Protocol {
    /// Request message type
    type Request: Message;

    /// Response message type
    type Response: Message;

    /// Port name for service registration/connection
    const PORT_NAME: &'static [u8];

    /// Protocol version (default: 1)
    const VERSION: u8 = 1;

    /// Maximum request size (default: MAX_MESSAGE_SIZE)
    const MAX_REQUEST_SIZE: usize = MAX_MESSAGE_SIZE;

    /// Maximum response size (default: MAX_MESSAGE_SIZE)
    const MAX_RESPONSE_SIZE: usize = MAX_MESSAGE_SIZE;
}

/// Message trait for serializable types
///
/// All protocol request and response types must implement this.
pub trait Message: Sized {
    /// Serialize to byte buffer
    ///
    /// Returns number of bytes written, or error if buffer too small.
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize>;

    /// Deserialize from byte buffer
    ///
    /// Returns the message and number of bytes consumed.
    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)>;

    /// Get the serialized size of this message
    fn serialized_size(&self) -> usize;
}

/// Simple serialization trait (write to buffer)
pub trait Serialize {
    /// Write to buffer, return bytes written
    fn write_to(&self, buf: &mut [u8]) -> IpcResult<usize>;
}

/// Simple deserialization trait (read from buffer)
pub trait Deserialize: Sized {
    /// Read from buffer, return value and bytes consumed
    fn read_from(buf: &[u8]) -> IpcResult<(Self, usize)>;
}

// Implement Serialize/Deserialize for primitive types

impl Serialize for u8 {
    fn write_to(&self, buf: &mut [u8]) -> IpcResult<usize> {
        if buf.is_empty() {
            return Err(IpcError::MessageTooLarge);
        }
        buf[0] = *self;
        Ok(1)
    }
}

impl Deserialize for u8 {
    fn read_from(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }
        Ok((buf[0], 1))
    }
}

impl Serialize for u16 {
    fn write_to(&self, buf: &mut [u8]) -> IpcResult<usize> {
        if buf.len() < 2 {
            return Err(IpcError::MessageTooLarge);
        }
        buf[..2].copy_from_slice(&self.to_le_bytes());
        Ok(2)
    }
}

impl Deserialize for u16 {
    fn read_from(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.len() < 2 {
            return Err(IpcError::Truncated);
        }
        Ok((u16::from_le_bytes([buf[0], buf[1]]), 2))
    }
}

impl Serialize for u32 {
    fn write_to(&self, buf: &mut [u8]) -> IpcResult<usize> {
        if buf.len() < 4 {
            return Err(IpcError::MessageTooLarge);
        }
        buf[..4].copy_from_slice(&self.to_le_bytes());
        Ok(4)
    }
}

impl Deserialize for u32 {
    fn read_from(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.len() < 4 {
            return Err(IpcError::Truncated);
        }
        Ok((
            u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            4,
        ))
    }
}

impl Serialize for u64 {
    fn write_to(&self, buf: &mut [u8]) -> IpcResult<usize> {
        if buf.len() < 8 {
            return Err(IpcError::MessageTooLarge);
        }
        buf[..8].copy_from_slice(&self.to_le_bytes());
        Ok(8)
    }
}

impl Deserialize for u64 {
    fn read_from(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.len() < 8 {
            return Err(IpcError::Truncated);
        }
        Ok((
            u64::from_le_bytes([
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
            ]),
            8,
        ))
    }
}

impl Serialize for i32 {
    fn write_to(&self, buf: &mut [u8]) -> IpcResult<usize> {
        (*self as u32).write_to(buf)
    }
}

impl Deserialize for i32 {
    fn read_from(buf: &[u8]) -> IpcResult<(Self, usize)> {
        let (val, n) = u32::read_from(buf)?;
        Ok((val as i32, n))
    }
}

impl Serialize for i64 {
    fn write_to(&self, buf: &mut [u8]) -> IpcResult<usize> {
        (*self as u64).write_to(buf)
    }
}

impl Deserialize for i64 {
    fn read_from(buf: &[u8]) -> IpcResult<(Self, usize)> {
        let (val, n) = u64::read_from(buf)?;
        Ok((val as i64, n))
    }
}

impl Serialize for bool {
    fn write_to(&self, buf: &mut [u8]) -> IpcResult<usize> {
        (*self as u8).write_to(buf)
    }
}

impl Deserialize for bool {
    fn read_from(buf: &[u8]) -> IpcResult<(Self, usize)> {
        let (val, n) = u8::read_from(buf)?;
        Ok((val != 0, n))
    }
}

/// Helper for implementing Message on #[repr(C)] structs
///
/// # Safety
/// Only use this for types that are:
/// - #[repr(C)]
/// - Have no padding
/// - Have no pointers
/// - Are the same size on all platforms
pub unsafe trait ReprCMessage: Sized + Copy {
    fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<Self>())
        }
    }

    fn from_bytes(data: &[u8]) -> Option<&Self> {
        if data.len() >= core::mem::size_of::<Self>() {
            Some(unsafe { &*(data.as_ptr() as *const Self) })
        } else {
            None
        }
    }
}

/// Implement Message for ReprCMessage types
impl<T: ReprCMessage> Message for T {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        let size = core::mem::size_of::<Self>();
        if buf.len() < size {
            return Err(IpcError::MessageTooLarge);
        }
        buf[..size].copy_from_slice(self.as_bytes());
        Ok(size)
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        let size = core::mem::size_of::<Self>();
        if buf.len() < size {
            return Err(IpcError::Truncated);
        }
        match Self::from_bytes(buf) {
            Some(msg) => Ok((*msg, size)),
            None => Err(IpcError::MalformedMessage),
        }
    }

    fn serialized_size(&self) -> usize {
        core::mem::size_of::<Self>()
    }
}

/// Empty message (for protocols with no request/response data)
#[derive(Debug, Clone, Copy, Default)]
pub struct Empty;

impl Message for Empty {
    fn serialize(&self, _buf: &mut [u8]) -> IpcResult<usize> {
        Ok(0)
    }

    fn deserialize(_buf: &[u8]) -> IpcResult<(Self, usize)> {
        Ok((Empty, 0))
    }

    fn serialized_size(&self) -> usize {
        0
    }
}
