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

// =============================================================================
// Cursor-based serialization helpers
// =============================================================================

/// Writer for serializing messages with automatic position tracking
///
/// # Example
/// ```rust
/// let mut buf = [0u8; 64];
/// let mut w = Writer::new(&mut buf);
/// w.write_u8(CMD_FIND_DEVICE)?;
/// w.write_u16(vendor_id)?;
/// w.write_u16(device_id)?;
/// let len = w.finish();
/// ```
pub struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    /// Create a new writer
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Get current position (bytes written)
    #[inline]
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Finish writing and return total bytes written
    #[inline]
    pub fn finish(self) -> usize {
        self.pos
    }

    /// Check if we have enough space
    #[inline]
    fn check_space(&self, n: usize) -> IpcResult<()> {
        if self.pos + n > self.buf.len() {
            Err(IpcError::MessageTooLarge)
        } else {
            Ok(())
        }
    }

    /// Write a u8
    #[inline]
    pub fn write_u8(&mut self, v: u8) -> IpcResult<()> {
        self.check_space(1)?;
        self.buf[self.pos] = v;
        self.pos += 1;
        Ok(())
    }

    /// Write a u16 (little-endian)
    #[inline]
    pub fn write_u16(&mut self, v: u16) -> IpcResult<()> {
        self.check_space(2)?;
        self.buf[self.pos..self.pos + 2].copy_from_slice(&v.to_le_bytes());
        self.pos += 2;
        Ok(())
    }

    /// Write a u32 (little-endian)
    #[inline]
    pub fn write_u32(&mut self, v: u32) -> IpcResult<()> {
        self.check_space(4)?;
        self.buf[self.pos..self.pos + 4].copy_from_slice(&v.to_le_bytes());
        self.pos += 4;
        Ok(())
    }

    /// Write a u64 (little-endian)
    #[inline]
    pub fn write_u64(&mut self, v: u64) -> IpcResult<()> {
        self.check_space(8)?;
        self.buf[self.pos..self.pos + 8].copy_from_slice(&v.to_le_bytes());
        self.pos += 8;
        Ok(())
    }

    /// Write an i32 (little-endian)
    #[inline]
    pub fn write_i32(&mut self, v: i32) -> IpcResult<()> {
        self.write_u32(v as u32)
    }

    /// Write an i64 (little-endian)
    #[inline]
    pub fn write_i64(&mut self, v: i64) -> IpcResult<()> {
        self.write_u64(v as u64)
    }

    /// Write a bool (1 byte)
    #[inline]
    pub fn write_bool(&mut self, v: bool) -> IpcResult<()> {
        self.write_u8(if v { 1 } else { 0 })
    }

    /// Write raw bytes
    #[inline]
    pub fn write_bytes(&mut self, data: &[u8]) -> IpcResult<()> {
        self.check_space(data.len())?;
        self.buf[self.pos..self.pos + data.len()].copy_from_slice(data);
        self.pos += data.len();
        Ok(())
    }

    /// Write a fixed-size byte array (no length prefix)
    #[inline]
    pub fn write_array<const N: usize>(&mut self, data: &[u8; N]) -> IpcResult<()> {
        self.write_bytes(data)
    }

    /// Write any type that implements Serialize
    #[inline]
    pub fn write<T: Serialize>(&mut self, v: &T) -> IpcResult<()> {
        let n = v.write_to(&mut self.buf[self.pos..])?;
        self.pos += n;
        Ok(())
    }
}

/// Reader for deserializing messages with automatic position tracking
///
/// # Example
/// ```rust
/// let mut r = Reader::new(&buf[..len]);
/// let cmd = r.read_u8()?;
/// let vendor_id = r.read_u16()?;
/// let device_id = r.read_u16()?;
/// let consumed = r.position();
/// ```
pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    /// Create a new reader
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Get current position (bytes consumed)
    #[inline]
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Get remaining bytes
    #[inline]
    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    /// Check if we have enough data
    #[inline]
    fn check_data(&self, n: usize) -> IpcResult<()> {
        if self.pos + n > self.buf.len() {
            Err(IpcError::Truncated)
        } else {
            Ok(())
        }
    }

    /// Read a u8
    #[inline]
    pub fn read_u8(&mut self) -> IpcResult<u8> {
        self.check_data(1)?;
        let v = self.buf[self.pos];
        self.pos += 1;
        Ok(v)
    }

    /// Read a u16 (little-endian)
    #[inline]
    pub fn read_u16(&mut self) -> IpcResult<u16> {
        self.check_data(2)?;
        let v = u16::from_le_bytes([self.buf[self.pos], self.buf[self.pos + 1]]);
        self.pos += 2;
        Ok(v)
    }

    /// Read a u32 (little-endian)
    #[inline]
    pub fn read_u32(&mut self) -> IpcResult<u32> {
        self.check_data(4)?;
        let v = u32::from_le_bytes([
            self.buf[self.pos],
            self.buf[self.pos + 1],
            self.buf[self.pos + 2],
            self.buf[self.pos + 3],
        ]);
        self.pos += 4;
        Ok(v)
    }

    /// Read a u64 (little-endian)
    #[inline]
    pub fn read_u64(&mut self) -> IpcResult<u64> {
        self.check_data(8)?;
        let v = u64::from_le_bytes([
            self.buf[self.pos],
            self.buf[self.pos + 1],
            self.buf[self.pos + 2],
            self.buf[self.pos + 3],
            self.buf[self.pos + 4],
            self.buf[self.pos + 5],
            self.buf[self.pos + 6],
            self.buf[self.pos + 7],
        ]);
        self.pos += 8;
        Ok(v)
    }

    /// Read an i32 (little-endian)
    #[inline]
    pub fn read_i32(&mut self) -> IpcResult<i32> {
        Ok(self.read_u32()? as i32)
    }

    /// Read an i64 (little-endian)
    #[inline]
    pub fn read_i64(&mut self) -> IpcResult<i64> {
        Ok(self.read_u64()? as i64)
    }

    /// Read a bool (1 byte)
    #[inline]
    pub fn read_bool(&mut self) -> IpcResult<bool> {
        Ok(self.read_u8()? != 0)
    }

    /// Read raw bytes into slice
    #[inline]
    pub fn read_bytes(&mut self, out: &mut [u8]) -> IpcResult<()> {
        self.check_data(out.len())?;
        out.copy_from_slice(&self.buf[self.pos..self.pos + out.len()]);
        self.pos += out.len();
        Ok(())
    }

    /// Read a fixed-size byte array
    #[inline]
    pub fn read_array<const N: usize>(&mut self) -> IpcResult<[u8; N]> {
        self.check_data(N)?;
        let mut arr = [0u8; N];
        arr.copy_from_slice(&self.buf[self.pos..self.pos + N]);
        self.pos += N;
        Ok(arr)
    }

    /// Read any type that implements Deserialize
    #[inline]
    pub fn read<T: Deserialize>(&mut self) -> IpcResult<T> {
        let (v, n) = T::read_from(&self.buf[self.pos..])?;
        self.pos += n;
        Ok(v)
    }

    /// Peek at the next byte without consuming it
    #[inline]
    pub fn peek_u8(&self) -> IpcResult<u8> {
        if self.pos >= self.buf.len() {
            Err(IpcError::Truncated)
        } else {
            Ok(self.buf[self.pos])
        }
    }
}

// =============================================================================
// Empty message type
// =============================================================================

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
