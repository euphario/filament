//! Block Device Protocol
//!
//! Protocol for block device access (USB MSC, NVMe, etc.).
//!
//! ## Architecture
//!
//! Block devices use a two-phase protocol:
//! 1. **Handshake** via IPC channels to establish shared memory
//! 2. **Data transfer** via shared ring buffer (zero-copy DMA)
//!
//! ```text
//! ┌─────────────┐     IPC Handshake      ┌─────────────┐
//! │  Client     │ ───────────────────────│  Server     │
//! │  (fatfs)    │                        │  (usbd)     │
//! └─────────────┘                        └─────────────┘
//!       │                                      │
//!       │  1. Send PID                         │
//!       │  2. Receive shmem_id                 │
//!       │  3. Map shared ring buffer           │
//!       ▼                                      ▼
//! ┌─────────────────────────────────────────────────────┐
//! │              Shared Ring Buffer                     │
//! │  ┌─────────┬────────────────┬────────────────────┐  │
//! │  │ Header  │ Submission Q   │ Completion Q       │  │
//! │  │ 64B     │ (requests)     │ (responses)        │  │
//! │  ├─────────┴────────────────┴────────────────────┤  │
//! │  │              Data Buffer (DMA target)         │  │
//! │  └───────────────────────────────────────────────┘  │
//! └─────────────────────────────────────────────────────┘
//! ```
//!
//! ## Usage
//!
//! See `userlib::ring::BlockRing` for the ring buffer implementation
//! and `usb::BlockClient` for the client API.

use super::super::error::{IpcError, IpcResult};
use super::super::protocol::{Protocol, Message};

/// Block device handshake protocol
///
/// This is only used for the initial handshake. After that,
/// all communication happens via the shared ring buffer.
pub struct BlockProtocol;

impl Protocol for BlockProtocol {
    type Request = BlockHandshake;
    type Response = BlockHandshakeResponse;
    const PORT_NAME: &'static [u8] = b"usb";  // or "nvme"
}

/// Handshake request (client sends PID)
#[derive(Debug, Clone, Copy)]
pub struct BlockHandshake {
    pub client_pid: u32,
}

impl Message for BlockHandshake {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        if buf.len() < 4 {
            return Err(IpcError::MessageTooLarge);
        }
        buf[..4].copy_from_slice(&self.client_pid.to_le_bytes());
        Ok(4)
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.len() < 4 {
            return Err(IpcError::Truncated);
        }
        let client_pid = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        Ok((BlockHandshake { client_pid }, 4))
    }

    fn serialized_size(&self) -> usize {
        4
    }
}

/// Handshake response (server sends shmem_id)
#[derive(Debug, Clone, Copy)]
pub struct BlockHandshakeResponse {
    pub shmem_id: u32,
}

impl Message for BlockHandshakeResponse {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        if buf.len() < 4 {
            return Err(IpcError::MessageTooLarge);
        }
        buf[..4].copy_from_slice(&self.shmem_id.to_le_bytes());
        Ok(4)
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.len() < 4 {
            return Err(IpcError::Truncated);
        }
        let shmem_id = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        Ok((BlockHandshakeResponse { shmem_id }, 4))
    }

    fn serialized_size(&self) -> usize {
        4
    }
}

/// Block operation types (for ring buffer requests)
///
/// These are used in `BlockRequest` via the ring buffer, not IPC messages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BlockOp {
    /// Get device info (block size, count)
    Info = 0,
    /// Read blocks
    Read = 1,
    /// Write blocks
    Write = 2,
    /// Flush/sync
    Flush = 3,
}

/// Block request status codes
pub mod status {
    /// Success
    pub const OK: u8 = 0;
    /// I/O error
    pub const IO_ERROR: u8 = 1;
    /// Invalid parameter
    pub const INVALID: u8 = 2;
    /// Device not ready
    pub const NOT_READY: u8 = 3;
    /// Device disconnected
    pub const DISCONNECTED: u8 = 4;
}

/// Port names for block devices
pub mod port {
    pub const USB: &[u8] = b"usb";
    pub const NVME: &[u8] = b"nvme";
}

// Re-export the ring buffer types for convenience
pub use crate::ring::{BlockRing, BlockRequest, BlockResponse};
