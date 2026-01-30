//! VFS Protocol
//!
//! Wire protocol for filesystem operations over DataPort rings.
//! Used between shell↔vfsd and vfsd↔fatfsd.

use crate::ring::io_op::FS_BASE;

// =============================================================================
// Operation codes (carried in IoSqe.opcode)
// =============================================================================

pub mod fs_op {
    use super::FS_BASE;

    pub const OPEN: u8    = FS_BASE;       // 0x80
    pub const READ: u8    = FS_BASE + 1;   // 0x81
    pub const WRITE: u8   = FS_BASE + 2;   // 0x82
    pub const CLOSE: u8   = FS_BASE + 3;   // 0x83
    pub const READDIR: u8 = FS_BASE + 4;   // 0x84
    pub const STAT: u8    = FS_BASE + 5;   // 0x85
    pub const MKDIR: u8   = FS_BASE + 6;   // 0x86
    pub const UNLINK: u8  = FS_BASE + 7;   // 0x87
    pub const RENAME: u8  = FS_BASE + 8;   // 0x88
}

// =============================================================================
// Open flags (carried in IoSqe.param for OPEN)
// =============================================================================

pub mod open_flags {
    pub const RDONLY: u32 = 0;
    pub const WRONLY: u32 = 1;
    pub const RDWR: u32   = 2;
    pub const CREATE: u32 = 0x100;
    pub const TRUNC: u32  = 0x200;
    pub const DIR: u32    = 0x400;
}

// =============================================================================
// File type constants
// =============================================================================

pub mod file_type {
    pub const FILE: u8 = 1;
    pub const DIR: u8 = 2;
}

// =============================================================================
// VFS error codes (carried in IoCqe.result on error)
// =============================================================================

pub mod vfs_error {
    pub const OK: u32           = 0;
    pub const NOT_FOUND: u32    = 1;
    pub const PERMISSION: u32   = 2;
    pub const NO_SPACE: u32     = 3;
    pub const NOT_DIR: u32      = 4;
    pub const IS_DIR: u32       = 5;
    pub const EXISTS: u32       = 6;
    pub const IO_ERROR: u32     = 7;
    pub const NO_MOUNT: u32     = 8;
    pub const TOO_MANY: u32     = 9;  // too many open files
    pub const READ_ONLY: u32    = 10;
}

// =============================================================================
// Directory entry (returned in pool for READDIR)
// =============================================================================

/// VFS directory entry — 64 bytes, packed into pool buffer for READDIR.
///
/// IoCqe.result = entry count, IoCqe.transferred = total bytes.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct VfsDirEntry {
    pub name_len: u8,
    pub file_type: u8,
    pub _pad: [u8; 2],
    pub size: u32,
    pub name: [u8; 56],
}

impl VfsDirEntry {
    pub const SIZE: usize = 64;

    pub const fn empty() -> Self {
        Self {
            name_len: 0,
            file_type: 0,
            _pad: [0; 2],
            size: 0,
            name: [0; 56],
        }
    }

    pub fn name_bytes(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }

    pub fn set_name(&mut self, name: &[u8]) {
        let len = name.len().min(56);
        self.name[..len].copy_from_slice(&name[..len]);
        self.name_len = len as u8;
    }

    /// Write this entry into a byte buffer at the given offset.
    pub fn write_to(&self, buf: &mut [u8], offset: usize) -> bool {
        if offset + Self::SIZE > buf.len() {
            return false;
        }
        let dst = &mut buf[offset..offset + Self::SIZE];
        dst[0] = self.name_len;
        dst[1] = self.file_type;
        dst[2] = 0;
        dst[3] = 0;
        dst[4..8].copy_from_slice(&self.size.to_le_bytes());
        dst[8..64].copy_from_slice(&self.name);
        true
    }

    /// Read an entry from a byte buffer at the given offset.
    pub fn from_bytes(buf: &[u8], offset: usize) -> Option<Self> {
        if offset + Self::SIZE > buf.len() {
            return None;
        }
        let src = &buf[offset..offset + Self::SIZE];
        let mut entry = Self::empty();
        entry.name_len = src[0];
        entry.file_type = src[1];
        entry.size = u32::from_le_bytes([src[4], src[5], src[6], src[7]]);
        entry.name.copy_from_slice(&src[8..64]);
        Some(entry)
    }
}

// =============================================================================
// Stat result (returned in pool for STAT)
// =============================================================================

/// VFS stat result — 16 bytes.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct VfsStat {
    pub size: u64,
    pub file_type: u8,
    pub _pad: [u8; 7],
}

impl VfsStat {
    pub const SIZE: usize = 16;

    pub const fn empty() -> Self {
        Self {
            size: 0,
            file_type: 0,
            _pad: [0; 7],
        }
    }

    pub fn write_to(&self, buf: &mut [u8], offset: usize) -> bool {
        if offset + Self::SIZE > buf.len() {
            return false;
        }
        let dst = &mut buf[offset..offset + Self::SIZE];
        dst[0..8].copy_from_slice(&self.size.to_le_bytes());
        dst[8] = self.file_type;
        dst[9..16].copy_from_slice(&[0u8; 7]);
        true
    }

    pub fn from_bytes(buf: &[u8], offset: usize) -> Option<Self> {
        if offset + Self::SIZE > buf.len() {
            return None;
        }
        let src = &buf[offset..offset + Self::SIZE];
        Some(Self {
            size: u64::from_le_bytes([
                src[0], src[1], src[2], src[3],
                src[4], src[5], src[6], src[7],
            ]),
            file_type: src[8],
            _pad: [0; 7],
        })
    }
}

// =============================================================================
// Shmem Mount Table (vfsd publishes, shell/apps read directly)
// =============================================================================

/// A single mount entry in the shared memory mount table.
///
/// 32 bytes. Written by vfsd, read by shell/apps.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct MountEntry {
    /// 1 = slot in use, 0 = empty
    pub in_use: u8,
    /// Length of prefix (max 24)
    pub prefix_len: u8,
    pub _pad: [u8; 2],
    /// Shmem ID of the FS driver's DataPort — connect here for I/O
    pub fs_shmem_id: u32,
    /// Mount prefix, e.g. "/fat0/"
    pub prefix: [u8; 24],
}

const _: () = assert!(core::mem::size_of::<MountEntry>() == 32);

impl MountEntry {
    pub const SIZE: usize = 32;

    pub const fn empty() -> Self {
        Self {
            in_use: 0,
            prefix_len: 0,
            _pad: [0; 2],
            fs_shmem_id: 0,
            prefix: [0; 24],
        }
    }

    /// Read a MountEntry from raw shmem bytes at offset.
    pub fn from_raw(data: &[u8], offset: usize) -> Option<Self> {
        if offset + Self::SIZE > data.len() {
            return None;
        }
        let s = &data[offset..offset + Self::SIZE];
        Some(Self {
            in_use: s[0],
            prefix_len: s[1],
            _pad: [0; 2],
            fs_shmem_id: u32::from_le_bytes([s[4], s[5], s[6], s[7]]),
            prefix: {
                let mut p = [0u8; 24];
                p.copy_from_slice(&s[8..32]);
                p
            },
        })
    }

    /// Write this entry to raw shmem bytes at offset.
    pub fn write_to_raw(&self, data: &mut [u8], offset: usize) -> bool {
        if offset + Self::SIZE > data.len() {
            return false;
        }
        let d = &mut data[offset..offset + Self::SIZE];
        d[0] = self.in_use;
        d[1] = self.prefix_len;
        d[2] = 0;
        d[3] = 0;
        d[4..8].copy_from_slice(&self.fs_shmem_id.to_le_bytes());
        d[8..32].copy_from_slice(&self.prefix);
        true
    }

    pub fn prefix_bytes(&self) -> &[u8] {
        &self.prefix[..self.prefix_len as usize]
    }
}

/// Shared memory mount table header.
///
/// 32 bytes header + 8 × 32-byte entries = 288 bytes total (fits in one page).
/// vfsd creates and owns this shmem region. Shell/apps map it read-only.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct MountTable {
    /// Bumped on every change (readers detect updates)
    pub version: u32,
    /// Number of active mounts
    pub count: u32,
    pub _pad: [u8; 24],
}

const _: () = assert!(core::mem::size_of::<MountTable>() == 32);

impl MountTable {
    pub const HEADER_SIZE: usize = 32;
    pub const MAX_MOUNTS: usize = 8;
    /// Total size including entries
    pub const TOTAL_SIZE: usize = Self::HEADER_SIZE + Self::MAX_MOUNTS * MountEntry::SIZE;

    /// Read header from raw shmem bytes.
    pub fn from_raw(data: &[u8]) -> Option<Self> {
        if data.len() < Self::HEADER_SIZE {
            return None;
        }
        Some(Self {
            version: u32::from_le_bytes([data[0], data[1], data[2], data[3]]),
            count: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
            _pad: [0; 24],
        })
    }

    /// Write header to raw shmem bytes.
    pub fn write_to_raw(&self, data: &mut [u8]) -> bool {
        if data.len() < Self::HEADER_SIZE {
            return false;
        }
        data[0..4].copy_from_slice(&self.version.to_le_bytes());
        data[4..8].copy_from_slice(&self.count.to_le_bytes());
        for b in &mut data[8..Self::HEADER_SIZE] { *b = 0; }
        true
    }

    /// Read entry at index from raw shmem bytes.
    pub fn read_entry(data: &[u8], index: usize) -> Option<MountEntry> {
        if index >= Self::MAX_MOUNTS {
            return None;
        }
        MountEntry::from_raw(data, Self::HEADER_SIZE + index * MountEntry::SIZE)
    }

    /// Write entry at index to raw shmem bytes.
    pub fn write_entry(data: &mut [u8], index: usize, entry: &MountEntry) -> bool {
        if index >= Self::MAX_MOUNTS {
            return false;
        }
        entry.write_to_raw(data, Self::HEADER_SIZE + index * MountEntry::SIZE)
    }
}

// =============================================================================
// IoSqe field mapping reference
// =============================================================================
//
// | Operation | opcode | param       | lba         | data_offset | data_len |
// |-----------|--------|-------------|-------------|-------------|----------|
// | OPEN      | 0x80   | open_flags  | 0           | path in pool| path len |
// | READ      | 0x81   | handle      | file_offset | buf in pool | buf len  |
// | WRITE     | 0x82   | handle      | file_offset | data in pool| data len |
// | CLOSE     | 0x83   | handle      | 0           | 0           | 0        |
// | READDIR   | 0x84   | handle      | 0           | buf in pool | buf len  |
// | STAT      | 0x85   | handle      | 0           | buf in pool | 16       |
// | MKDIR     | 0x86   | 0           | 0           | path in pool| path len |
// | UNLINK    | 0x87   | 0           | 0           | path in pool| path len |
// | RENAME    | 0x88   | dst_offset  | dst_len     | src in pool | src len  |
//
// IoCqe: result = handle (OPEN) or entry count (READDIR). transferred = bytes.
