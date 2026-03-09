//! Block Device Protocol Messages
//!
//! Protocol for communication between block device drivers (qemu-usbd, usbd)
//! and filesystems (fatfs). Matches the BlockDevice trait semantics.
//!
//! ## Message Flow
//!
//! ```text
//! fatfs ←→ qemu-usbd
//!   BLK_INFO  →
//!             ← BLK_INFO_RESP (block_size, block_count)
//!   BLK_READ  →
//!             ← BLK_DATA (block data)
//!   BLK_WRITE →
//!             ← BLK_RESULT (ok/error)
//! ```

// =============================================================================
// Message Types
// =============================================================================

pub mod msg {
    /// Request block device info (block_size, block_count)
    pub const BLK_INFO: u16 = 0x0500;
    /// Read blocks at LBA
    pub const BLK_READ: u16 = 0x0501;
    /// Write blocks at LBA
    pub const BLK_WRITE: u16 = 0x0502;

    /// Block device info response
    pub const BLK_INFO_RESP: u16 = 0x0580;
    /// Read response with data
    pub const BLK_DATA: u16 = 0x0581;
    /// Generic result (ok/error)
    pub const BLK_RESULT: u16 = 0x0582;
}

pub mod error {
    pub const OK: i32 = 0;
    pub const IO: i32 = -1;
    pub const OUT_OF_RANGE: i32 = -2;
    pub const NOT_READY: i32 = -3;
    pub const INVALID_MESSAGE: i32 = -4;
}

/// Block operation error enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlkError {
    Ok,
    Io,
    OutOfRange,
    NotReady,
    InvalidMessage,
}

impl BlkError {
    pub fn from_i32(code: i32) -> Self {
        match code {
            error::OK => BlkError::Ok,
            error::IO => BlkError::Io,
            error::OUT_OF_RANGE => BlkError::OutOfRange,
            error::NOT_READY => BlkError::NotReady,
            _ => BlkError::InvalidMessage,
        }
    }

    pub fn to_i32(self) -> i32 {
        match self {
            BlkError::Ok => error::OK,
            BlkError::Io => error::IO,
            BlkError::OutOfRange => error::OUT_OF_RANGE,
            BlkError::NotReady => error::NOT_READY,
            BlkError::InvalidMessage => error::INVALID_MESSAGE,
        }
    }

    pub fn is_ok(self) -> bool {
        matches!(self, BlkError::Ok)
    }
}

// =============================================================================
// Message Header
// =============================================================================

/// Current block protocol version
pub const BLK_PROTOCOL_VERSION: u8 = 1;

/// Common header for all block messages (8 bytes)
///
/// Layout: [msg_type: u16, version: u8, flags: u8, seq_id: u32]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlkHeader {
    pub msg_type: u16,
    pub version: u8,
    pub flags: u8,
    pub seq_id: u32,
}

impl BlkHeader {
    pub const SIZE: usize = 8;

    pub fn new(msg_type: u16, seq_id: u32) -> Self {
        Self {
            msg_type,
            version: BLK_PROTOCOL_VERSION,
            flags: 0,
            seq_id,
        }
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            msg_type: u16::from_le_bytes([buf[0], buf[1]]),
            version: buf[2],
            flags: buf[3],
            seq_id: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        })
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..2].copy_from_slice(&self.msg_type.to_le_bytes());
        buf[2] = self.version;
        buf[3] = self.flags;
        buf[4..8].copy_from_slice(&self.seq_id.to_le_bytes());
        buf
    }
}

// =============================================================================
// Request Messages
// =============================================================================

/// BLK_INFO request - get block device geometry (8 bytes, header only)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlkInfo {
    pub header: BlkHeader,
}

impl BlkInfo {
    pub const SIZE: usize = BlkHeader::SIZE;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: BlkHeader::new(msg::BLK_INFO, seq_id),
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        self.header.to_bytes()
    }
}

/// BLK_READ request - read blocks at LBA (24 bytes)
///
/// Layout: [header: 8, lba: u64, count: u32, _pad: u32]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlkRead {
    pub header: BlkHeader,
    /// Starting LBA
    pub lba: u64,
    /// Number of blocks to read
    pub count: u32,
    pub _pad: u32,
}

impl BlkRead {
    pub const SIZE: usize = 24;

    pub fn new(seq_id: u32, lba: u64, count: u32) -> Self {
        Self {
            header: BlkHeader::new(msg::BLK_READ, seq_id),
            lba,
            count,
            _pad: 0,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..16].copy_from_slice(&self.lba.to_le_bytes());
        buf[16..20].copy_from_slice(&self.count.to_le_bytes());
        buf[20..24].copy_from_slice(&self._pad.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: BlkHeader::from_bytes(buf)?,
            lba: u64::from_le_bytes([buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]]),
            count: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            _pad: 0,
        })
    }
}

/// BLK_WRITE request - write blocks at LBA
///
/// Layout: [header: 8, lba: u64, count: u32, _pad: u32, data...]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlkWrite {
    pub header: BlkHeader,
    /// Starting LBA
    pub lba: u64,
    /// Number of blocks to write
    pub count: u32,
    pub _pad: u32,
    // Followed by: data bytes
}

impl BlkWrite {
    pub const HEADER_SIZE: usize = 24;

    pub fn new(seq_id: u32, lba: u64, count: u32) -> Self {
        Self {
            header: BlkHeader::new(msg::BLK_WRITE, seq_id),
            lba,
            count,
            _pad: 0,
        }
    }

    /// Write request header + data to buffer
    pub fn write_to(&self, buf: &mut [u8], data: &[u8]) -> Option<usize> {
        let total = Self::HEADER_SIZE + data.len();
        if buf.len() < total {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..16].copy_from_slice(&self.lba.to_le_bytes());
        buf[16..20].copy_from_slice(&self.count.to_le_bytes());
        buf[20..24].copy_from_slice(&self._pad.to_le_bytes());
        buf[24..24 + data.len()].copy_from_slice(data);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let req = Self {
            header: BlkHeader::from_bytes(buf)?,
            lba: u64::from_le_bytes([buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]]),
            count: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            _pad: 0,
        };
        Some((req, &buf[Self::HEADER_SIZE..]))
    }
}

// =============================================================================
// Response Messages
// =============================================================================

/// BLK_INFO_RESP - block device geometry (24 bytes)
///
/// Layout: [header: 8, block_size: u32, _pad: u32, block_count: u64]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlkInfoResp {
    pub header: BlkHeader,
    /// Block size in bytes (typically 512)
    pub block_size: u32,
    pub _pad: u32,
    /// Total number of blocks
    pub block_count: u64,
}

impl BlkInfoResp {
    pub const SIZE: usize = 24;

    pub fn new(seq_id: u32, block_size: u32, block_count: u64) -> Self {
        Self {
            header: BlkHeader::new(msg::BLK_INFO_RESP, seq_id),
            block_size,
            _pad: 0,
            block_count,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.block_size.to_le_bytes());
        buf[12..16].copy_from_slice(&self._pad.to_le_bytes());
        buf[16..24].copy_from_slice(&self.block_count.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: BlkHeader::from_bytes(buf)?,
            block_size: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            _pad: 0,
            block_count: u64::from_le_bytes([buf[16], buf[17], buf[18], buf[19], buf[20], buf[21], buf[22], buf[23]]),
        })
    }
}

/// BLK_DATA - read response with block data
///
/// Layout: [header: 8, bytes_read: u32, error: i32, data...]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlkData {
    pub header: BlkHeader,
    /// Bytes of data following header (block_size * count)
    pub bytes_read: u32,
    /// Error code (0 = success)
    pub error: i32,
    // Followed by: data bytes
}

impl BlkData {
    pub const HEADER_SIZE: usize = 16;

    pub fn new(seq_id: u32, bytes_read: u32, error: i32) -> Self {
        Self {
            header: BlkHeader::new(msg::BLK_DATA, seq_id),
            bytes_read,
            error,
        }
    }

    /// Write response header + data to buffer
    pub fn write_to(&self, buf: &mut [u8], data: &[u8]) -> Option<usize> {
        let total = Self::HEADER_SIZE + data.len();
        if buf.len() < total {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.bytes_read.to_le_bytes());
        buf[12..16].copy_from_slice(&self.error.to_le_bytes());
        buf[16..16 + data.len()].copy_from_slice(data);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let resp = Self {
            header: BlkHeader::from_bytes(buf)?,
            bytes_read: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            error: i32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
        };
        let data_len = resp.bytes_read as usize;
        if buf.len() < Self::HEADER_SIZE + data_len {
            return None;
        }
        Some((resp, &buf[Self::HEADER_SIZE..Self::HEADER_SIZE + data_len]))
    }
}

/// BLK_RESULT - generic result response (16 bytes)
///
/// Layout: [header: 8, error: i32, _pad: u32]
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlkResult {
    pub header: BlkHeader,
    /// Error code (0 = success)
    pub error: i32,
    pub _pad: u32,
}

impl BlkResult {
    pub const SIZE: usize = 16;

    pub fn new(seq_id: u32, error: i32) -> Self {
        Self {
            header: BlkHeader::new(msg::BLK_RESULT, seq_id),
            error,
            _pad: 0,
        }
    }

    pub fn ok(seq_id: u32) -> Self {
        Self::new(seq_id, error::OK)
    }

    pub fn err(seq_id: u32, err: BlkError) -> Self {
        Self::new(seq_id, err.to_i32())
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.error.to_le_bytes());
        buf[12..16].copy_from_slice(&self._pad.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: BlkHeader::from_bytes(buf)?,
            error: i32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            _pad: 0,
        })
    }
}
