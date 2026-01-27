//! VFS Protocol Messages
//!
//! Protocol for communication between:
//! - shell ↔ vfsd (filesystem commands)
//! - vfsd ↔ fatfs (filesystem operations)

// =============================================================================
// Message Types
// =============================================================================

pub mod msg {
    // Shell → vfsd requests
    pub const LIST_DIR: u16 = 0x0100;
    pub const READ_FILE: u16 = 0x0101;
    pub const WRITE_FILE: u16 = 0x0102;
    pub const COPY_FILE: u16 = 0x0103;
    pub const MAKE_DIR: u16 = 0x0104;
    pub const REMOVE: u16 = 0x0105;
    pub const STAT: u16 = 0x0106;
    pub const MOUNT: u16 = 0x0107;
    pub const UNMOUNT: u16 = 0x0108;

    // vfsd → shell responses
    pub const DIR_ENTRIES: u16 = 0x0200;
    pub const FILE_DATA: u16 = 0x0201;
    pub const FILE_INFO: u16 = 0x0202;
    pub const RESULT: u16 = 0x0203;

    // vfsd ↔ fatfs (filesystem driver protocol)
    pub const FS_OPEN: u16 = 0x0300;
    pub const FS_READ: u16 = 0x0301;
    pub const FS_WRITE: u16 = 0x0302;
    pub const FS_CLOSE: u16 = 0x0303;
    pub const FS_LIST: u16 = 0x0304;
    pub const FS_STAT: u16 = 0x0305;
    pub const FS_MKDIR: u16 = 0x0306;
    pub const FS_REMOVE: u16 = 0x0307;

    // fatfs → vfsd
    pub const FS_REGISTER: u16 = 0x0400;
    pub const FS_DATA: u16 = 0x0401;
    pub const FS_RESULT: u16 = 0x0402;
    pub const FS_ENTRIES: u16 = 0x0403;
}

pub mod error {
    pub const OK: i32 = 0;
    pub const NOT_FOUND: i32 = -1;
    pub const PERMISSION_DENIED: i32 = -2;
    pub const IO_ERROR: i32 = -3;
    pub const INVALID_PATH: i32 = -4;
    pub const NOT_DIRECTORY: i32 = -5;
    pub const NOT_FILE: i32 = -6;
    pub const ALREADY_EXISTS: i32 = -7;
    pub const NOT_EMPTY: i32 = -8;
    pub const NO_SPACE: i32 = -9;
    pub const NOT_MOUNTED: i32 = -10;
}

pub mod file_type {
    pub const FILE: u8 = 0;
    pub const DIRECTORY: u8 = 1;
    pub const SYMLINK: u8 = 2;
}

pub mod open_mode {
    pub const READ: u8 = 0x01;
    pub const WRITE: u8 = 0x02;
    pub const CREATE: u8 = 0x04;
    pub const TRUNCATE: u8 = 0x08;
    pub const APPEND: u8 = 0x10;
}

// =============================================================================
// Message Header
// =============================================================================

/// Common header for all VFS messages (8 bytes)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VfsHeader {
    pub msg_type: u16,
    pub flags: u16,
    pub seq_id: u32,
}

impl VfsHeader {
    pub const SIZE: usize = 8;

    pub fn new(msg_type: u16, seq_id: u32) -> Self {
        Self { msg_type, flags: 0, seq_id }
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            msg_type: u16::from_le_bytes([buf[0], buf[1]]),
            flags: u16::from_le_bytes([buf[2], buf[3]]),
            seq_id: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        })
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..2].copy_from_slice(&self.msg_type.to_le_bytes());
        buf[2..4].copy_from_slice(&self.flags.to_le_bytes());
        buf[4..8].copy_from_slice(&self.seq_id.to_le_bytes());
        buf
    }
}

// =============================================================================
// Shell → vfsd Messages
// =============================================================================

/// List directory request
#[repr(C)]
pub struct ListDir {
    pub header: VfsHeader,
    pub path_len: u16,
    pub _pad: u16,
    // Followed by: path bytes
}

impl ListDir {
    pub const HEADER_SIZE: usize = VfsHeader::SIZE + 4;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: VfsHeader::new(msg::LIST_DIR, seq_id),
            path_len: 0,
            _pad: 0,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], path: &[u8]) -> Option<usize> {
        let total = Self::HEADER_SIZE + path.len();
        if buf.len() < total {
            return None;
        }
        buf[0..VfsHeader::SIZE].copy_from_slice(&self.header.to_bytes());
        buf[VfsHeader::SIZE..VfsHeader::SIZE + 2].copy_from_slice(&(path.len() as u16).to_le_bytes());
        buf[VfsHeader::SIZE + 2..VfsHeader::SIZE + 4].copy_from_slice(&[0, 0]);
        buf[Self::HEADER_SIZE..total].copy_from_slice(path);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let header = VfsHeader::from_bytes(buf)?;
        let path_len = u16::from_le_bytes([buf[VfsHeader::SIZE], buf[VfsHeader::SIZE + 1]]) as usize;
        if buf.len() < Self::HEADER_SIZE + path_len {
            return None;
        }
        let path = &buf[Self::HEADER_SIZE..Self::HEADER_SIZE + path_len];
        Some((Self { header, path_len: path_len as u16, _pad: 0 }, path))
    }
}

/// Read file request
#[repr(C)]
pub struct ReadFile {
    pub header: VfsHeader,
    pub path_len: u16,
    pub _pad: u16,
    pub offset: u64,
    pub len: u32,
    pub _pad2: u32,
    // Followed by: path bytes
}

impl ReadFile {
    pub const HEADER_SIZE: usize = VfsHeader::SIZE + 20;

    pub fn new(seq_id: u32, offset: u64, len: u32) -> Self {
        Self {
            header: VfsHeader::new(msg::READ_FILE, seq_id),
            path_len: 0,
            _pad: 0,
            offset,
            len,
            _pad2: 0,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], path: &[u8]) -> Option<usize> {
        let total = Self::HEADER_SIZE + path.len();
        if buf.len() < total {
            return None;
        }
        buf[0..VfsHeader::SIZE].copy_from_slice(&self.header.to_bytes());
        buf[VfsHeader::SIZE..VfsHeader::SIZE + 2].copy_from_slice(&(path.len() as u16).to_le_bytes());
        buf[VfsHeader::SIZE + 2..VfsHeader::SIZE + 4].copy_from_slice(&[0, 0]);
        buf[VfsHeader::SIZE + 4..VfsHeader::SIZE + 12].copy_from_slice(&self.offset.to_le_bytes());
        buf[VfsHeader::SIZE + 12..VfsHeader::SIZE + 16].copy_from_slice(&self.len.to_le_bytes());
        buf[VfsHeader::SIZE + 16..VfsHeader::SIZE + 20].copy_from_slice(&[0, 0, 0, 0]);
        buf[Self::HEADER_SIZE..total].copy_from_slice(path);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let header = VfsHeader::from_bytes(buf)?;
        let path_len = u16::from_le_bytes([buf[VfsHeader::SIZE], buf[VfsHeader::SIZE + 1]]) as usize;
        let offset = u64::from_le_bytes([
            buf[VfsHeader::SIZE + 4], buf[VfsHeader::SIZE + 5],
            buf[VfsHeader::SIZE + 6], buf[VfsHeader::SIZE + 7],
            buf[VfsHeader::SIZE + 8], buf[VfsHeader::SIZE + 9],
            buf[VfsHeader::SIZE + 10], buf[VfsHeader::SIZE + 11],
        ]);
        let len = u32::from_le_bytes([
            buf[VfsHeader::SIZE + 12], buf[VfsHeader::SIZE + 13],
            buf[VfsHeader::SIZE + 14], buf[VfsHeader::SIZE + 15],
        ]);
        if buf.len() < Self::HEADER_SIZE + path_len {
            return None;
        }
        let path = &buf[Self::HEADER_SIZE..Self::HEADER_SIZE + path_len];
        Some((Self { header, path_len: path_len as u16, _pad: 0, offset, len, _pad2: 0 }, path))
    }
}

/// Copy file request
#[repr(C)]
pub struct CopyFile {
    pub header: VfsHeader,
    pub src_len: u16,
    pub dst_len: u16,
    // Followed by: src path, dst path
}

impl CopyFile {
    pub const HEADER_SIZE: usize = VfsHeader::SIZE + 4;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: VfsHeader::new(msg::COPY_FILE, seq_id),
            src_len: 0,
            dst_len: 0,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], src: &[u8], dst: &[u8]) -> Option<usize> {
        let total = Self::HEADER_SIZE + src.len() + dst.len();
        if buf.len() < total {
            return None;
        }
        buf[0..VfsHeader::SIZE].copy_from_slice(&self.header.to_bytes());
        buf[VfsHeader::SIZE..VfsHeader::SIZE + 2].copy_from_slice(&(src.len() as u16).to_le_bytes());
        buf[VfsHeader::SIZE + 2..VfsHeader::SIZE + 4].copy_from_slice(&(dst.len() as u16).to_le_bytes());
        buf[Self::HEADER_SIZE..Self::HEADER_SIZE + src.len()].copy_from_slice(src);
        buf[Self::HEADER_SIZE + src.len()..total].copy_from_slice(dst);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8], &[u8])> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let header = VfsHeader::from_bytes(buf)?;
        let src_len = u16::from_le_bytes([buf[VfsHeader::SIZE], buf[VfsHeader::SIZE + 1]]) as usize;
        let dst_len = u16::from_le_bytes([buf[VfsHeader::SIZE + 2], buf[VfsHeader::SIZE + 3]]) as usize;
        if buf.len() < Self::HEADER_SIZE + src_len + dst_len {
            return None;
        }
        let src = &buf[Self::HEADER_SIZE..Self::HEADER_SIZE + src_len];
        let dst = &buf[Self::HEADER_SIZE + src_len..Self::HEADER_SIZE + src_len + dst_len];
        Some((Self { header, src_len: src_len as u16, dst_len: dst_len as u16 }, src, dst))
    }
}

/// Stat request (get file info)
#[repr(C)]
pub struct Stat {
    pub header: VfsHeader,
    pub path_len: u16,
    pub _pad: u16,
    // Followed by: path bytes
}

impl Stat {
    pub const HEADER_SIZE: usize = VfsHeader::SIZE + 4;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: VfsHeader::new(msg::STAT, seq_id),
            path_len: 0,
            _pad: 0,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], path: &[u8]) -> Option<usize> {
        let total = Self::HEADER_SIZE + path.len();
        if buf.len() < total {
            return None;
        }
        buf[0..VfsHeader::SIZE].copy_from_slice(&self.header.to_bytes());
        buf[VfsHeader::SIZE..VfsHeader::SIZE + 2].copy_from_slice(&(path.len() as u16).to_le_bytes());
        buf[VfsHeader::SIZE + 2..VfsHeader::SIZE + 4].copy_from_slice(&[0, 0]);
        buf[Self::HEADER_SIZE..total].copy_from_slice(path);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let header = VfsHeader::from_bytes(buf)?;
        let path_len = u16::from_le_bytes([buf[VfsHeader::SIZE], buf[VfsHeader::SIZE + 1]]) as usize;
        if buf.len() < Self::HEADER_SIZE + path_len {
            return None;
        }
        let path = &buf[Self::HEADER_SIZE..Self::HEADER_SIZE + path_len];
        Some((Self { header, path_len: path_len as u16, _pad: 0 }, path))
    }
}

// =============================================================================
// vfsd → shell Responses
// =============================================================================

/// Directory entry
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DirEntry {
    pub file_type: u8,
    pub name_len: u8,
    pub _pad: [u8; 2],
    pub size: u64,
    // Followed by: name bytes (name_len)
}

impl DirEntry {
    pub const HEADER_SIZE: usize = 12;

    pub fn new(file_type: u8, size: u64) -> Self {
        Self { file_type, name_len: 0, _pad: [0; 2], size }
    }

    pub fn write_to(&self, buf: &mut [u8], name: &[u8]) -> Option<usize> {
        let total = Self::HEADER_SIZE + name.len();
        if buf.len() < total {
            return None;
        }
        buf[0] = self.file_type;
        buf[1] = name.len() as u8;
        buf[2..4].copy_from_slice(&[0, 0]);
        buf[4..12].copy_from_slice(&self.size.to_le_bytes());
        buf[Self::HEADER_SIZE..total].copy_from_slice(name);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let file_type = buf[0];
        let name_len = buf[1] as usize;
        let size = u64::from_le_bytes([
            buf[4], buf[5], buf[6], buf[7],
            buf[8], buf[9], buf[10], buf[11],
        ]);
        if buf.len() < Self::HEADER_SIZE + name_len {
            return None;
        }
        let name = &buf[Self::HEADER_SIZE..Self::HEADER_SIZE + name_len];
        Some((Self { file_type, name_len: name_len as u8, _pad: [0; 2], size }, name))
    }

    pub fn total_size(&self) -> usize {
        Self::HEADER_SIZE + self.name_len as usize
    }
}

/// Directory listing response
#[repr(C)]
pub struct DirEntries {
    pub header: VfsHeader,
    pub count: u16,
    pub more: u8,  // 1 if more entries available
    pub _pad: u8,
    // Followed by: DirEntry[] (each with variable-length name)
}

impl DirEntries {
    pub const HEADER_SIZE: usize = VfsHeader::SIZE + 4;

    pub fn new(seq_id: u32, count: u16, more: bool) -> Self {
        Self {
            header: VfsHeader::new(msg::DIR_ENTRIES, seq_id),
            count,
            more: if more { 1 } else { 0 },
            _pad: 0,
        }
    }

    pub fn write_header(&self, buf: &mut [u8]) -> Option<usize> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        buf[0..VfsHeader::SIZE].copy_from_slice(&self.header.to_bytes());
        buf[VfsHeader::SIZE..VfsHeader::SIZE + 2].copy_from_slice(&self.count.to_le_bytes());
        buf[VfsHeader::SIZE + 2] = self.more;
        buf[VfsHeader::SIZE + 3] = 0;
        Some(Self::HEADER_SIZE)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let header = VfsHeader::from_bytes(buf)?;
        let count = u16::from_le_bytes([buf[VfsHeader::SIZE], buf[VfsHeader::SIZE + 1]]);
        let more = buf[VfsHeader::SIZE + 2];
        Some(Self { header, count, more, _pad: 0 })
    }
}

/// File data response (chunk)
#[repr(C)]
pub struct FileData {
    pub header: VfsHeader,
    pub len: u32,
    pub eof: u8,  // 1 if this is the last chunk
    pub _pad: [u8; 3],
    // Followed by: data bytes
}

impl FileData {
    pub const HEADER_SIZE: usize = VfsHeader::SIZE + 8;

    pub fn new(seq_id: u32, len: u32, eof: bool) -> Self {
        Self {
            header: VfsHeader::new(msg::FILE_DATA, seq_id),
            len,
            eof: if eof { 1 } else { 0 },
            _pad: [0; 3],
        }
    }

    pub fn write_to(&self, buf: &mut [u8], data: &[u8]) -> Option<usize> {
        let total = Self::HEADER_SIZE + data.len();
        if buf.len() < total {
            return None;
        }
        buf[0..VfsHeader::SIZE].copy_from_slice(&self.header.to_bytes());
        buf[VfsHeader::SIZE..VfsHeader::SIZE + 4].copy_from_slice(&(data.len() as u32).to_le_bytes());
        buf[VfsHeader::SIZE + 4] = self.eof;
        buf[VfsHeader::SIZE + 5..VfsHeader::SIZE + 8].copy_from_slice(&[0, 0, 0]);
        buf[Self::HEADER_SIZE..total].copy_from_slice(data);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let header = VfsHeader::from_bytes(buf)?;
        let len = u32::from_le_bytes([
            buf[VfsHeader::SIZE], buf[VfsHeader::SIZE + 1],
            buf[VfsHeader::SIZE + 2], buf[VfsHeader::SIZE + 3],
        ]) as usize;
        let eof = buf[VfsHeader::SIZE + 4];
        if buf.len() < Self::HEADER_SIZE + len {
            return None;
        }
        let data = &buf[Self::HEADER_SIZE..Self::HEADER_SIZE + len];
        Some((Self { header, len: len as u32, eof, _pad: [0; 3] }, data))
    }
}

/// File info response
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct FileInfo {
    pub header: VfsHeader,
    pub file_type: u8,
    pub _pad: [u8; 3],
    pub size: u64,
    pub created: u64,
    pub modified: u64,
}

impl FileInfo {
    pub const SIZE: usize = VfsHeader::SIZE + 28;

    pub fn new(seq_id: u32, file_type: u8, size: u64) -> Self {
        Self {
            header: VfsHeader::new(msg::FILE_INFO, seq_id),
            file_type,
            _pad: [0; 3],
            size,
            created: 0,
            modified: 0,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..VfsHeader::SIZE].copy_from_slice(&self.header.to_bytes());
        buf[VfsHeader::SIZE] = self.file_type;
        buf[VfsHeader::SIZE + 4..VfsHeader::SIZE + 12].copy_from_slice(&self.size.to_le_bytes());
        buf[VfsHeader::SIZE + 12..VfsHeader::SIZE + 20].copy_from_slice(&self.created.to_le_bytes());
        buf[VfsHeader::SIZE + 20..VfsHeader::SIZE + 28].copy_from_slice(&self.modified.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let header = VfsHeader::from_bytes(buf)?;
        let file_type = buf[VfsHeader::SIZE];
        let size = u64::from_le_bytes([
            buf[VfsHeader::SIZE + 4], buf[VfsHeader::SIZE + 5],
            buf[VfsHeader::SIZE + 6], buf[VfsHeader::SIZE + 7],
            buf[VfsHeader::SIZE + 8], buf[VfsHeader::SIZE + 9],
            buf[VfsHeader::SIZE + 10], buf[VfsHeader::SIZE + 11],
        ]);
        let created = u64::from_le_bytes([
            buf[VfsHeader::SIZE + 12], buf[VfsHeader::SIZE + 13],
            buf[VfsHeader::SIZE + 14], buf[VfsHeader::SIZE + 15],
            buf[VfsHeader::SIZE + 16], buf[VfsHeader::SIZE + 17],
            buf[VfsHeader::SIZE + 18], buf[VfsHeader::SIZE + 19],
        ]);
        let modified = u64::from_le_bytes([
            buf[VfsHeader::SIZE + 20], buf[VfsHeader::SIZE + 21],
            buf[VfsHeader::SIZE + 22], buf[VfsHeader::SIZE + 23],
            buf[VfsHeader::SIZE + 24], buf[VfsHeader::SIZE + 25],
            buf[VfsHeader::SIZE + 26], buf[VfsHeader::SIZE + 27],
        ]);
        Some(Self { header, file_type, _pad: [0; 3], size, created, modified })
    }
}

/// Generic result response
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Result {
    pub header: VfsHeader,
    pub code: i32,
    pub _pad: u32,
}

impl Result {
    pub const SIZE: usize = VfsHeader::SIZE + 8;

    pub fn new(seq_id: u32, code: i32) -> Self {
        Self {
            header: VfsHeader::new(msg::RESULT, seq_id),
            code,
            _pad: 0,
        }
    }

    pub fn ok(seq_id: u32) -> Self {
        Self::new(seq_id, error::OK)
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..VfsHeader::SIZE].copy_from_slice(&self.header.to_bytes());
        buf[VfsHeader::SIZE..VfsHeader::SIZE + 4].copy_from_slice(&self.code.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let header = VfsHeader::from_bytes(buf)?;
        let code = i32::from_le_bytes([
            buf[VfsHeader::SIZE], buf[VfsHeader::SIZE + 1],
            buf[VfsHeader::SIZE + 2], buf[VfsHeader::SIZE + 3],
        ]);
        Some(Self { header, code, _pad: 0 })
    }
}

// =============================================================================
// fatfs → vfsd Registration
// =============================================================================

/// Filesystem registration (fatfs → vfsd)
#[repr(C)]
pub struct FsRegister {
    pub header: VfsHeader,
    pub fs_type: u8,      // e.g., FAT12, FAT16, FAT32
    pub _pad: [u8; 3],
    pub mount_len: u16,   // Mount point path length
    pub port_len: u16,    // Port name length
    // Followed by: mount point path, port name
}

impl FsRegister {
    pub const HEADER_SIZE: usize = VfsHeader::SIZE + 8;

    pub fn new(seq_id: u32, fs_type: u8) -> Self {
        Self {
            header: VfsHeader::new(msg::FS_REGISTER, seq_id),
            fs_type,
            _pad: [0; 3],
            mount_len: 0,
            port_len: 0,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], mount_point: &[u8], port_name: &[u8]) -> Option<usize> {
        let total = Self::HEADER_SIZE + mount_point.len() + port_name.len();
        if buf.len() < total {
            return None;
        }
        buf[0..VfsHeader::SIZE].copy_from_slice(&self.header.to_bytes());
        buf[VfsHeader::SIZE] = self.fs_type;
        buf[VfsHeader::SIZE + 1..VfsHeader::SIZE + 4].copy_from_slice(&[0, 0, 0]);
        buf[VfsHeader::SIZE + 4..VfsHeader::SIZE + 6].copy_from_slice(&(mount_point.len() as u16).to_le_bytes());
        buf[VfsHeader::SIZE + 6..VfsHeader::SIZE + 8].copy_from_slice(&(port_name.len() as u16).to_le_bytes());
        buf[Self::HEADER_SIZE..Self::HEADER_SIZE + mount_point.len()].copy_from_slice(mount_point);
        buf[Self::HEADER_SIZE + mount_point.len()..total].copy_from_slice(port_name);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8], &[u8])> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let header = VfsHeader::from_bytes(buf)?;
        let fs_type = buf[VfsHeader::SIZE];
        let mount_len = u16::from_le_bytes([buf[VfsHeader::SIZE + 4], buf[VfsHeader::SIZE + 5]]) as usize;
        let port_len = u16::from_le_bytes([buf[VfsHeader::SIZE + 6], buf[VfsHeader::SIZE + 7]]) as usize;
        if buf.len() < Self::HEADER_SIZE + mount_len + port_len {
            return None;
        }
        let mount = &buf[Self::HEADER_SIZE..Self::HEADER_SIZE + mount_len];
        let port = &buf[Self::HEADER_SIZE + mount_len..Self::HEADER_SIZE + mount_len + port_len];
        Some((Self { header, fs_type, _pad: [0; 3], mount_len: mount_len as u16, port_len: port_len as u16 }, mount, port))
    }
}

// =============================================================================
// Filesystem types
// =============================================================================

pub mod fs_type {
    pub const UNKNOWN: u8 = 0;
    pub const FAT12: u8 = 1;
    pub const FAT16: u8 = 2;
    pub const FAT32: u8 = 3;
    pub const RAMFS: u8 = 4;
}
