//! Filesystem Protocol
//!
//! Generic filesystem IPC protocol for file operations.
//! Used by fatfs and potentially other filesystem drivers.
//!
//! ## Operations
//!
//! - `Open`: Open a file by path
//! - `Read`: Read data from an open file
//! - `Write`: Write data to an open file
//! - `Close`: Close an open file
//! - `Stat`: Get file metadata
//! - `ReadDir`: List directory contents
//! - `ReadToShmem`: Read file directly into shared memory (for DMA)
//!
//! ## Example
//!
//! ```rust
//! use userlib::ipc::protocols::{FsClient, FsRequest};
//!
//! let mut fs = FsClient::connect()?;
//!
//! // Read a file into shared memory
//! let (data, size) = fs.read_file(b"/firmware/mt7996.bin", max_size)?;
//! ```

use super::super::error::{IpcError, IpcResult};
use super::super::protocol::{Protocol, Message};

/// Maximum path length
pub const MAX_PATH: usize = 256;

/// Maximum inline data in a single message
pub const MAX_INLINE_DATA: usize = 256;

/// Maximum directory entries per response
pub const MAX_DIR_ENTRIES: usize = 16;

/// Filesystem service protocol
pub struct FsProtocol;

impl Protocol for FsProtocol {
    type Request = FsRequest;
    type Response = FsResponse;
    const PORT_NAME: &'static [u8] = b"fatfs";
}

/// Command codes
const FS_CMD_OPEN: u8 = 1;
const FS_CMD_READ: u8 = 2;
const FS_CMD_WRITE: u8 = 3;
const FS_CMD_CLOSE: u8 = 4;
const FS_CMD_STAT: u8 = 5;
const FS_CMD_READ_DIR: u8 = 6;
const FS_CMD_READ_SHMEM: u8 = 7;  // Read directly into shared memory

/// Open flags
pub mod flags {
    pub const READ: u32 = 1;
    pub const WRITE: u32 = 2;
    pub const CREATE: u32 = 4;
    pub const TRUNCATE: u32 = 8;
    pub const APPEND: u32 = 16;
}

/// File types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FileType {
    Regular = 0,
    Directory = 1,
    Unknown = 255,
}

impl From<u8> for FileType {
    fn from(v: u8) -> Self {
        match v {
            0 => FileType::Regular,
            1 => FileType::Directory,
            _ => FileType::Unknown,
        }
    }
}

/// File metadata
#[derive(Debug, Clone, Copy, Default)]
pub struct FileStat {
    pub file_type: u8,
    pub size: u64,
    pub created: u64,
    pub modified: u64,
}

/// Directory entry
#[derive(Debug, Clone)]
pub struct DirEntry {
    pub name: [u8; 64],
    pub name_len: u8,
    pub file_type: FileType,
    pub size: u64,
}

impl DirEntry {
    pub fn name_str(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }
}

impl Default for DirEntry {
    fn default() -> Self {
        Self {
            name: [0u8; 64],
            name_len: 0,
            file_type: FileType::Unknown,
            size: 0,
        }
    }
}

/// Filesystem request messages
#[derive(Debug, Clone)]
pub enum FsRequest {
    /// Open a file
    Open {
        path: [u8; MAX_PATH],
        path_len: u16,
        flags: u32,
    },
    /// Read from an open file
    Read {
        fd: u32,
        offset: u64,
        size: u32,
    },
    /// Write to an open file
    Write {
        fd: u32,
        offset: u64,
        data: [u8; MAX_INLINE_DATA],
        data_len: u16,
    },
    /// Close an open file
    Close { fd: u32 },
    /// Get file metadata
    Stat {
        path: [u8; MAX_PATH],
        path_len: u16,
    },
    /// Read directory contents
    ReadDir {
        path: [u8; MAX_PATH],
        path_len: u16,
        offset: u32,  // Entry offset for pagination
    },
    /// Read file directly into shared memory
    /// This is for large transfers (firmware, etc.)
    ReadToShmem {
        path: [u8; MAX_PATH],
        path_len: u16,
        shmem_id: u32,
        max_size: u32,
        requester_pid: u32,
    },
}

impl FsRequest {
    /// Create an Open request
    pub fn open(path: &[u8], flags: u32) -> Self {
        let mut p = [0u8; MAX_PATH];
        let len = core::cmp::min(path.len(), MAX_PATH);
        p[..len].copy_from_slice(&path[..len]);
        FsRequest::Open {
            path: p,
            path_len: len as u16,
            flags,
        }
    }

    /// Create a Stat request
    pub fn stat(path: &[u8]) -> Self {
        let mut p = [0u8; MAX_PATH];
        let len = core::cmp::min(path.len(), MAX_PATH);
        p[..len].copy_from_slice(&path[..len]);
        FsRequest::Stat {
            path: p,
            path_len: len as u16,
        }
    }

    /// Create a ReadDir request
    pub fn read_dir(path: &[u8], offset: u32) -> Self {
        let mut p = [0u8; MAX_PATH];
        let len = core::cmp::min(path.len(), MAX_PATH);
        p[..len].copy_from_slice(&path[..len]);
        FsRequest::ReadDir {
            path: p,
            path_len: len as u16,
            offset,
        }
    }

    /// Create a ReadToShmem request
    pub fn read_to_shmem(path: &[u8], shmem_id: u32, max_size: u32, requester_pid: u32) -> Self {
        let mut p = [0u8; MAX_PATH];
        let len = core::cmp::min(path.len(), MAX_PATH);
        p[..len].copy_from_slice(&path[..len]);
        FsRequest::ReadToShmem {
            path: p,
            path_len: len as u16,
            shmem_id,
            max_size,
            requester_pid,
        }
    }
}

impl Message for FsRequest {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        match self {
            FsRequest::Open { path, path_len, flags } => {
                let needed = 1 + 2 + 4 + *path_len as usize;
                if buf.len() < needed {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = FS_CMD_OPEN;
                buf[1..3].copy_from_slice(&path_len.to_le_bytes());
                buf[3..7].copy_from_slice(&flags.to_le_bytes());
                buf[7..7 + *path_len as usize].copy_from_slice(&path[..*path_len as usize]);
                Ok(7 + *path_len as usize)
            }
            FsRequest::Read { fd, offset, size } => {
                if buf.len() < 17 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = FS_CMD_READ;
                buf[1..5].copy_from_slice(&fd.to_le_bytes());
                buf[5..13].copy_from_slice(&offset.to_le_bytes());
                buf[13..17].copy_from_slice(&size.to_le_bytes());
                Ok(17)
            }
            FsRequest::Write { fd, offset, data, data_len } => {
                let needed = 1 + 4 + 8 + 2 + *data_len as usize;
                if buf.len() < needed {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = FS_CMD_WRITE;
                buf[1..5].copy_from_slice(&fd.to_le_bytes());
                buf[5..13].copy_from_slice(&offset.to_le_bytes());
                buf[13..15].copy_from_slice(&data_len.to_le_bytes());
                buf[15..15 + *data_len as usize].copy_from_slice(&data[..*data_len as usize]);
                Ok(15 + *data_len as usize)
            }
            FsRequest::Close { fd } => {
                if buf.len() < 5 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = FS_CMD_CLOSE;
                buf[1..5].copy_from_slice(&fd.to_le_bytes());
                Ok(5)
            }
            FsRequest::Stat { path, path_len } => {
                let needed = 1 + 2 + *path_len as usize;
                if buf.len() < needed {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = FS_CMD_STAT;
                buf[1..3].copy_from_slice(&path_len.to_le_bytes());
                buf[3..3 + *path_len as usize].copy_from_slice(&path[..*path_len as usize]);
                Ok(3 + *path_len as usize)
            }
            FsRequest::ReadDir { path, path_len, offset } => {
                let needed = 1 + 2 + 4 + *path_len as usize;
                if buf.len() < needed {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = FS_CMD_READ_DIR;
                buf[1..3].copy_from_slice(&path_len.to_le_bytes());
                buf[3..7].copy_from_slice(&offset.to_le_bytes());
                buf[7..7 + *path_len as usize].copy_from_slice(&path[..*path_len as usize]);
                Ok(7 + *path_len as usize)
            }
            FsRequest::ReadToShmem { path, path_len, shmem_id, max_size, requester_pid } => {
                let needed = 1 + 2 + 4 + 4 + 4 + *path_len as usize;
                if buf.len() < needed {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = FS_CMD_READ_SHMEM;
                buf[1..3].copy_from_slice(&path_len.to_le_bytes());
                buf[3..7].copy_from_slice(&shmem_id.to_le_bytes());
                buf[7..11].copy_from_slice(&max_size.to_le_bytes());
                buf[11..15].copy_from_slice(&requester_pid.to_le_bytes());
                buf[15..15 + *path_len as usize].copy_from_slice(&path[..*path_len as usize]);
                Ok(15 + *path_len as usize)
            }
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }

        match buf[0] {
            FS_CMD_OPEN => {
                if buf.len() < 7 {
                    return Err(IpcError::Truncated);
                }
                let path_len = u16::from_le_bytes([buf[1], buf[2]]);
                let flags = u32::from_le_bytes([buf[3], buf[4], buf[5], buf[6]]);
                let total = 7 + path_len as usize;
                if buf.len() < total {
                    return Err(IpcError::Truncated);
                }
                let mut path = [0u8; MAX_PATH];
                path[..path_len as usize].copy_from_slice(&buf[7..total]);
                Ok((FsRequest::Open { path, path_len, flags }, total))
            }
            FS_CMD_READ => {
                if buf.len() < 17 {
                    return Err(IpcError::Truncated);
                }
                let fd = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
                let offset = u64::from_le_bytes([
                    buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12],
                ]);
                let size = u32::from_le_bytes([buf[13], buf[14], buf[15], buf[16]]);
                Ok((FsRequest::Read { fd, offset, size }, 17))
            }
            FS_CMD_CLOSE => {
                if buf.len() < 5 {
                    return Err(IpcError::Truncated);
                }
                let fd = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
                Ok((FsRequest::Close { fd }, 5))
            }
            FS_CMD_STAT => {
                if buf.len() < 3 {
                    return Err(IpcError::Truncated);
                }
                let path_len = u16::from_le_bytes([buf[1], buf[2]]);
                let total = 3 + path_len as usize;
                if buf.len() < total {
                    return Err(IpcError::Truncated);
                }
                let mut path = [0u8; MAX_PATH];
                path[..path_len as usize].copy_from_slice(&buf[3..total]);
                Ok((FsRequest::Stat { path, path_len }, total))
            }
            FS_CMD_READ_SHMEM => {
                if buf.len() < 15 {
                    return Err(IpcError::Truncated);
                }
                let path_len = u16::from_le_bytes([buf[1], buf[2]]);
                let shmem_id = u32::from_le_bytes([buf[3], buf[4], buf[5], buf[6]]);
                let max_size = u32::from_le_bytes([buf[7], buf[8], buf[9], buf[10]]);
                let requester_pid = u32::from_le_bytes([buf[11], buf[12], buf[13], buf[14]]);
                let total = 15 + path_len as usize;
                if buf.len() < total {
                    return Err(IpcError::Truncated);
                }
                let mut path = [0u8; MAX_PATH];
                path[..path_len as usize].copy_from_slice(&buf[15..total]);
                Ok((
                    FsRequest::ReadToShmem {
                        path,
                        path_len,
                        shmem_id,
                        max_size,
                        requester_pid,
                    },
                    total,
                ))
            }
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    fn serialized_size(&self) -> usize {
        match self {
            FsRequest::Open { path_len, .. } => 7 + *path_len as usize,
            FsRequest::Read { .. } => 17,
            FsRequest::Write { data_len, .. } => 15 + *data_len as usize,
            FsRequest::Close { .. } => 5,
            FsRequest::Stat { path_len, .. } => 3 + *path_len as usize,
            FsRequest::ReadDir { path_len, .. } => 7 + *path_len as usize,
            FsRequest::ReadToShmem { path_len, .. } => 15 + *path_len as usize,
        }
    }
}

/// Filesystem response messages
#[derive(Debug, Clone)]
pub enum FsResponse {
    /// File opened successfully
    Opened { fd: u32 },
    /// Read data (inline, for small reads)
    Data {
        data: [u8; MAX_INLINE_DATA],
        data_len: u16,
        eof: bool,
    },
    /// Write completed
    Written { bytes: u32 },
    /// File closed
    Closed,
    /// File metadata
    Stat(FileStat),
    /// Directory entries
    DirEntries {
        entries: [DirEntry; MAX_DIR_ENTRIES],
        count: u8,
        more: bool,  // More entries available
    },
    /// Read to shmem completed
    ShmemRead { bytes: u64 },
    /// Error
    Error(i32),
}

/// Error codes
pub mod error {
    pub const NOT_FOUND: i32 = -2;
    pub const PERMISSION: i32 = -13;
    pub const INVALID: i32 = -22;
    pub const IO: i32 = -5;
    pub const NO_SPACE: i32 = -28;
    pub const IS_DIR: i32 = -21;
    pub const NOT_DIR: i32 = -20;
}

impl Message for FsResponse {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        match self {
            FsResponse::Opened { fd } => {
                if buf.len() < 5 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = 0; // Success type
                buf[1..5].copy_from_slice(&fd.to_le_bytes());
                Ok(5)
            }
            FsResponse::ShmemRead { bytes } => {
                if buf.len() < 9 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = 7; // ShmemRead type
                buf[1..9].copy_from_slice(&bytes.to_le_bytes());
                Ok(9)
            }
            FsResponse::Error(code) => {
                if buf.len() < 5 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = 255; // Error type
                buf[1..5].copy_from_slice(&code.to_le_bytes());
                Ok(5)
            }
            FsResponse::Closed => {
                if buf.is_empty() {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = 4; // Closed type
                Ok(1)
            }
            // TODO: Implement other response serialization
            _ => Err(IpcError::Internal),
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }

        match buf[0] {
            0 => {
                // Opened
                if buf.len() < 5 {
                    return Err(IpcError::Truncated);
                }
                let fd = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
                Ok((FsResponse::Opened { fd }, 5))
            }
            4 => Ok((FsResponse::Closed, 1)),
            7 => {
                // ShmemRead
                if buf.len() < 9 {
                    return Err(IpcError::Truncated);
                }
                let bytes = u64::from_le_bytes([
                    buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8],
                ]);
                Ok((FsResponse::ShmemRead { bytes }, 9))
            }
            255 => {
                // Error
                if buf.len() < 5 {
                    return Err(IpcError::Truncated);
                }
                let code = i32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
                Ok((FsResponse::Error(code), 5))
            }
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    fn serialized_size(&self) -> usize {
        match self {
            FsResponse::Opened { .. } => 5,
            FsResponse::Data { data_len, .. } => 4 + *data_len as usize,
            FsResponse::Written { .. } => 5,
            FsResponse::Closed => 1,
            FsResponse::Stat(_) => 25,
            FsResponse::DirEntries { count, .. } => 2 + (*count as usize * 74),
            FsResponse::ShmemRead { .. } => 9,
            FsResponse::Error(_) => 5,
        }
    }
}

/// Convenience client for filesystem operations
pub struct FsClient {
    inner: super::super::Client<FsProtocol>,
    server_pid: Option<u32>,
}

impl FsClient {
    /// Connect to filesystem service
    pub fn connect() -> IpcResult<Self> {
        let mut inner = super::super::Client::<FsProtocol>::connect()?;

        // Perform PID handshake (needed for shmem_allow)
        let server_pid = inner.handshake_pid()?;

        Ok(Self {
            inner,
            server_pid: Some(server_pid),
        })
    }

    /// Get server PID (for shmem_allow)
    pub fn server_pid(&self) -> Option<u32> {
        self.server_pid
    }

    /// Read a file into shared memory
    ///
    /// Returns (vaddr, paddr, size, shmem_id) on success.
    /// Caller is responsible for calling shmem_destroy when done.
    pub fn read_file(&mut self, path: &[u8], max_size: usize) -> IpcResult<(u64, u64, usize, u32)> {
        use crate::syscall;

        // Create shared memory
        let mut vaddr: u64 = 0;
        let mut paddr: u64 = 0;
        let shmem_id = syscall::shmem_create(max_size, &mut vaddr, &mut paddr);
        if shmem_id < 0 {
            return Err(IpcError::OutOfMemory);
        }
        let shmem_id = shmem_id as u32;

        // Allow server to access
        if let Some(pid) = self.server_pid {
            syscall::shmem_allow(shmem_id, pid);
        }

        // Send request
        let request = FsRequest::read_to_shmem(
            path,
            shmem_id,
            max_size as u32,
            syscall::getpid() as u32,
        );

        let response = self.inner.request(&request)?;

        match response {
            FsResponse::ShmemRead { bytes } => Ok((vaddr, paddr, bytes as usize, shmem_id)),
            FsResponse::Error(code) => {
                syscall::shmem_destroy(shmem_id);
                Err(IpcError::ServerError(code))
            }
            _ => {
                syscall::shmem_destroy(shmem_id);
                Err(IpcError::UnexpectedMessage)
            }
        }
    }
}
