//! VFS Client Library
//!
//! Reads vfsd's shmem mount table to discover filesystems, then connects
//! directly to FS drivers' DataPorts for I/O. No routing through vfsd.
//!
//! ## Architecture
//!
//! ```text
//! VfsClient
//!   ├── maps vfsd shmem (mount table in DataPort pool)
//!   ├── reads MountEntry[] → gets fs_shmem_id per mount
//!   └── connects to fatfsd DataPort → issues VFS ops directly
//! ```

use crate::data_port::DataPort;
use crate::ipc::{Channel, Shmem, Timer, Mux, MuxFilter};
use crate::ring::{IoSqe, IoCqe, io_status};
use crate::vfs_proto::{fs_op, file_type, MountTable, MountEntry, VfsDirEntry, VfsStat};
use crate::query::{QueryHeader, QueryPort, PortInfoResponse, msg, error};
use crate::syscall;
use crate::syscall::RamfsListEntry;

// =============================================================================
// Constants
// =============================================================================

const MAX_MOUNTS: usize = MountTable::MAX_MOUNTS;

/// Virtual handle for root directory "/"
const HANDLE_ROOT: u32 = 0xFF00_0000;
/// Virtual handle for /bin (ramfs)
const HANDLE_BIN: u32 = 0xFE00_0000;
/// Virtual handle for /mnt (synthetic)
const HANDLE_MNT: u32 = 0xFD00_0000;

// =============================================================================
// Error type
// =============================================================================

#[derive(Clone, Copy, Debug)]
pub enum VfsError {
    /// vfsd not found or not available
    NotAvailable,
    /// Connection to vfsd failed
    ConnectFailed,
    /// Pool allocation failed (out of buffer space)
    PoolFull,
    /// Ring submission failed
    RingFull,
    /// I/O error from filesystem
    IoError,
    /// File or directory not found
    NotFound,
    /// Not a directory
    NotDir,
    /// Is a directory (tried to read dir as file)
    IsDir,
    /// Too many open files
    TooMany,
    /// Timeout waiting for completion
    Timeout,
    /// Read-only filesystem
    ReadOnly,
    /// No mount point matched
    NoMount,
    /// Permission denied
    Permission,
}

impl VfsError {
    fn from_cqe(cqe: &IoCqe) -> Self {
        use crate::vfs_proto::vfs_error;
        if cqe.status != io_status::OK {
            return VfsError::IoError;
        }
        match cqe.result {
            vfs_error::NOT_FOUND => VfsError::NotFound,
            vfs_error::PERMISSION => VfsError::Permission,
            vfs_error::NO_SPACE => VfsError::PoolFull,
            vfs_error::NOT_DIR => VfsError::NotDir,
            vfs_error::IS_DIR => VfsError::IsDir,
            vfs_error::IO_ERROR => VfsError::IoError,
            vfs_error::NO_MOUNT => VfsError::NoMount,
            vfs_error::TOO_MANY => VfsError::TooMany,
            vfs_error::READ_ONLY => VfsError::ReadOnly,
            _ => VfsError::IoError,
        }
    }
}

// =============================================================================
// Cached mount connection
// =============================================================================

struct MountConnection {
    port: DataPort,
    _prefix: [u8; 24],
    _prefix_len: u8,
}

// =============================================================================
// VfsClient
// =============================================================================

/// Client for VFS operations.
///
/// Maps vfsd's shmem mount table, reads mount entries, and connects
/// directly to FS drivers' DataPorts for I/O.
pub struct VfsClient {
    /// Mapped vfsd shmem (contains mount table in DataPort pool)
    vfsd_shmem: Shmem,
    /// Pool offset within the shmem (from LayeredRingHeader)
    pool_offset: u32,
    /// Cached per-mount DataPort connections
    connections: [Option<MountConnection>; MAX_MOUNTS],
}

impl VfsClient {
    /// Discover vfsd via devd-query and map its mount table shmem.
    pub fn discover() -> Result<Self, VfsError> {
        // Connect to devd-query
        let mut channel = Channel::connect(b"devd-query:")
            .map_err(|_| VfsError::NotAvailable)?;

        // Build QUERY_PORT request for "vfs:"
        let req = QueryPort::new(1);
        let mut buf = [0u8; 64];
        let len = req.write_to(&mut buf, b"vfs:")
            .ok_or(VfsError::NotAvailable)?;

        channel.send(&buf[..len]).map_err(|_| VfsError::NotAvailable)?;

        // Wait for response with timeout
        let response = recv_with_timeout(&mut channel, 2000)
            .ok_or(VfsError::NotAvailable)?;

        // Parse PORT_INFO response
        let header = QueryHeader::from_bytes(&response)
            .ok_or(VfsError::NotAvailable)?;

        if header.msg_type == msg::ERROR || header.msg_type != msg::PORT_INFO {
            return Err(VfsError::NotAvailable);
        }

        let info = PortInfoResponse::from_bytes(&response)
            .ok_or(VfsError::NotAvailable)?;

        if info.result != error::OK || info.shmem_id == 0 {
            return Err(VfsError::NotAvailable);
        }

        Self::connect(info.shmem_id)
    }

    /// Connect directly by vfsd's shmem_id.
    pub fn connect(shmem_id: u32) -> Result<Self, VfsError> {
        let shmem = Shmem::open_existing(shmem_id)
            .map_err(|_| VfsError::ConnectFailed)?;

        // Read pool_offset from LayeredRingHeader at byte offset 36
        let ptr = shmem.as_ptr();
        let pool_offset = unsafe {
            let bytes = core::slice::from_raw_parts(ptr.add(36), 4);
            u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
        };

        Ok(Self {
            vfsd_shmem: shmem,
            pool_offset,
            connections: [
                None, None, None, None,
                None, None, None, None,
            ],
        })
    }

    /// Read the mount table from the mapped shmem.
    fn read_mount_table(&self) -> ([MountEntry; MAX_MOUNTS], u32) {
        let ptr = self.vfsd_shmem.as_ptr();
        let mut raw = [0u8; MountTable::TOTAL_SIZE];
        unsafe {
            let src = ptr.add(self.pool_offset as usize);
            core::ptr::copy_nonoverlapping(src, raw.as_mut_ptr(), MountTable::TOTAL_SIZE);
        }

        let header = MountTable::from_raw(&raw).unwrap_or(MountTable {
            version: 0,
            count: 0,
            _pad: [0; 24],
        });

        let mut entries = [MountEntry::empty(); MAX_MOUNTS];
        for i in 0..MAX_MOUNTS {
            if let Some(entry) = MountTable::read_entry(&raw, i) {
                entries[i] = entry;
            }
        }

        (entries, header.count)
    }

    /// Find the mount that matches a path (longest prefix match).
    /// Returns (mount_index, stripped path for FS driver).
    fn find_mount<'a>(&self, path: &'a [u8]) -> Option<(usize, MountEntry, &'a [u8])> {
        let (entries, _count) = self.read_mount_table();

        let mut best_idx = None;
        let mut best_len = 0;
        let mut best_entry = MountEntry::empty();

        for i in 0..MAX_MOUNTS {
            if entries[i].in_use == 0 {
                continue;
            }
            let prefix = entries[i].prefix_bytes();
            if path.len() >= prefix.len() && &path[..prefix.len()] == prefix {
                if prefix.len() > best_len {
                    best_len = prefix.len();
                    best_idx = Some(i);
                    best_entry = entries[i];
                }
            }
        }

        // Also match without trailing slash (e.g., "/fat0" matches "/fat0/")
        if best_idx.is_none() {
            for i in 0..MAX_MOUNTS {
                if entries[i].in_use == 0 {
                    continue;
                }
                let prefix = entries[i].prefix_bytes();
                if prefix.len() > 1 && prefix[prefix.len() - 1] == b'/' {
                    let prefix_no_slash = &prefix[..prefix.len() - 1];
                    if path == prefix_no_slash {
                        return Some((i, entries[i], b""));
                    }
                }
            }
        }

        best_idx.map(|i| {
            let prefix_len = entries[i].prefix_len as usize;
            (i, best_entry, &path[prefix_len..])
        })
    }

    /// Get or create a DataPort connection for a mount.
    fn get_connection(&mut self, mount_idx: usize, entry: &MountEntry) -> Result<&mut DataPort, VfsError> {
        if self.connections[mount_idx].is_none() {
            let port = DataPort::connect(entry.fs_shmem_id)
                .map_err(|_| VfsError::ConnectFailed)?;
            self.connections[mount_idx] = Some(MountConnection {
                port,
                _prefix: entry.prefix,
                _prefix_len: entry.prefix_len,
            });
        }
        Ok(&mut self.connections[mount_idx].as_mut().unwrap().port)
    }

    /// Open a file or directory.
    ///
    /// Returns a handle encoded as `(mount_idx << 24) | fs_handle`.
    pub fn open(&mut self, path: &[u8], flags: u32) -> Result<u32, VfsError> {
        // Handle virtual root
        if path == b"/" {
            return Ok(HANDLE_ROOT);
        }

        // Handle /bin (ramfs)
        if path == b"/bin" || path == b"/bin/" {
            return Ok(HANDLE_BIN);
        }

        // Handle /mnt (synthetic mount directory)
        if path == b"/mnt" || path == b"/mnt/" {
            return Ok(HANDLE_MNT);
        }

        let (mount_idx, entry, remainder) = self.find_mount(path)
            .ok_or(VfsError::NoMount)?;

        let port = self.get_connection(mount_idx, &entry)?;

        // Build path for FS: prepend "/" to remainder
        let mut fs_path = [0u8; 256];
        fs_path[0] = b'/';
        let remainder_len = remainder.len().min(255);
        fs_path[1..1 + remainder_len].copy_from_slice(&remainder[..remainder_len]);
        let fs_path_len = 1 + remainder_len;

        let path_len = fs_path_len as u32;
        let offset = port.alloc(path_len).ok_or(VfsError::PoolFull)?;
        port.pool_write(offset, &fs_path[..fs_path_len]);

        let tag = port.next_tag();
        let sqe = IoSqe {
            opcode: fs_op::OPEN,
            flags: 0,
            priority: 0,
            tag,
            lba: 0,
            data_offset: offset,
            data_len: path_len,
            param: flags as u64,
        };

        if !port.submit(&sqe) {
            return Err(VfsError::RingFull);
        }
        port.notify();

        let cqe = poll_completion(port, tag)?;
        port.reset_pool();

        if cqe.status != io_status::OK {
            return Err(VfsError::from_cqe(&cqe));
        }

        // Encode handle: (mount_idx << 24) | fs_handle
        let encoded = ((mount_idx as u32) << 24) | (cqe.result & 0x00FF_FFFF);
        Ok(encoded)
    }

    /// Read from an open file.
    ///
    /// Returns number of bytes read.
    pub fn read(
        &mut self,
        handle: u32,
        file_offset: u64,
        buf: &mut [u8],
    ) -> Result<usize, VfsError> {
        let (mount_idx, fs_handle) = decode_handle(handle);

        if mount_idx >= MAX_MOUNTS {
            return Err(VfsError::NotFound);
        }

        // Need to get connection — read mount table to get entry
        let (entries, _) = self.read_mount_table();
        if entries[mount_idx].in_use == 0 {
            return Err(VfsError::NotFound);
        }
        let entry = entries[mount_idx];
        let port = self.get_connection(mount_idx, &entry)?;

        let buf_len = buf.len() as u32;
        let offset = port.alloc(buf_len).ok_or(VfsError::PoolFull)?;

        let tag = port.next_tag();
        let sqe = IoSqe {
            opcode: fs_op::READ,
            flags: 0,
            priority: 0,
            tag,
            lba: file_offset,
            data_offset: offset,
            data_len: buf_len,
            param: fs_handle as u64,
        };

        if !port.submit(&sqe) {
            return Err(VfsError::RingFull);
        }
        port.notify();

        let cqe = poll_completion(port, tag)?;

        if cqe.status != io_status::OK {
            port.reset_pool();
            return Err(VfsError::from_cqe(&cqe));
        }

        let transferred = cqe.transferred as usize;
        if transferred > 0 {
            port.pool_read(offset, &mut buf[..transferred]);
        }

        port.reset_pool();
        Ok(transferred)
    }

    /// Close an open file handle.
    pub fn close(&mut self, handle: u32) -> Result<(), VfsError> {
        if handle == HANDLE_ROOT || handle == HANDLE_BIN || handle == HANDLE_MNT {
            return Ok(()); // Virtual handles
        }

        let (mount_idx, fs_handle) = decode_handle(handle);

        if mount_idx >= MAX_MOUNTS {
            return Err(VfsError::NotFound);
        }

        let (entries, _) = self.read_mount_table();
        if entries[mount_idx].in_use == 0 {
            return Err(VfsError::NotFound);
        }
        let entry = entries[mount_idx];
        let port = self.get_connection(mount_idx, &entry)?;

        let tag = port.next_tag();
        let sqe = IoSqe {
            opcode: fs_op::CLOSE,
            flags: 0,
            priority: 0,
            tag,
            lba: 0,
            data_offset: 0,
            data_len: 0,
            param: fs_handle as u64,
        };

        if !port.submit(&sqe) {
            return Err(VfsError::RingFull);
        }
        port.notify();

        let cqe = poll_completion(port, tag)?;
        port.reset_pool();

        if cqe.status != io_status::OK {
            return Err(VfsError::from_cqe(&cqe));
        }

        Ok(())
    }

    /// Read directory entries.
    ///
    /// `handle` must be an open directory handle.
    /// Returns the number of entries read into `entries`.
    pub fn readdir(
        &mut self,
        handle: u32,
        entries: &mut [VfsDirEntry],
    ) -> Result<usize, VfsError> {
        // Virtual root
        if handle == HANDLE_ROOT {
            return self.readdir_virtual_root(entries);
        }
        // /bin (ramfs)
        if handle == HANDLE_BIN {
            return Self::readdir_bin(entries);
        }
        // /mnt (synthetic)
        if handle == HANDLE_MNT {
            return self.readdir_mnt(entries);
        }

        let (mount_idx, fs_handle) = decode_handle(handle);

        if mount_idx >= MAX_MOUNTS {
            return Err(VfsError::NotFound);
        }

        let (mount_entries, _) = self.read_mount_table();
        if mount_entries[mount_idx].in_use == 0 {
            return Err(VfsError::NotFound);
        }
        let entry = mount_entries[mount_idx];
        let port = self.get_connection(mount_idx, &entry)?;

        let buf_len = (entries.len() * VfsDirEntry::SIZE) as u32;
        let offset = port.alloc(buf_len).ok_or(VfsError::PoolFull)?;

        let tag = port.next_tag();
        let sqe = IoSqe {
            opcode: fs_op::READDIR,
            flags: 0,
            priority: 0,
            tag,
            lba: 0,
            data_offset: offset,
            data_len: buf_len,
            param: fs_handle as u64,
        };

        if !port.submit(&sqe) {
            return Err(VfsError::RingFull);
        }
        port.notify();

        let cqe = poll_completion(port, tag)?;

        if cqe.status != io_status::OK {
            port.reset_pool();
            return Err(VfsError::from_cqe(&cqe));
        }

        let count = (cqe.result as usize).min(entries.len());

        if count > 0 {
            let total_bytes = (count * VfsDirEntry::SIZE) as u32;
            let mut raw = [0u8; 64 * 32]; // max 32 entries at 64 bytes each
            let read_len = total_bytes.min(raw.len() as u32);
            port.pool_read(offset, &mut raw[..read_len as usize]);

            for i in 0..count {
                if let Some(de) = VfsDirEntry::from_bytes(&raw, i * VfsDirEntry::SIZE) {
                    entries[i] = de;
                }
            }
        }

        port.reset_pool();
        Ok(count)
    }

    /// Get file/directory status.
    pub fn stat(&mut self, handle: u32) -> Result<VfsStat, VfsError> {
        if handle == HANDLE_ROOT || handle == HANDLE_BIN || handle == HANDLE_MNT {
            return Ok(VfsStat {
                size: 0,
                file_type: file_type::DIR,
                _pad: [0; 7],
            });
        }

        let (mount_idx, fs_handle) = decode_handle(handle);

        if mount_idx >= MAX_MOUNTS {
            return Err(VfsError::NotFound);
        }

        let (mount_entries, _) = self.read_mount_table();
        if mount_entries[mount_idx].in_use == 0 {
            return Err(VfsError::NotFound);
        }
        let entry = mount_entries[mount_idx];
        let port = self.get_connection(mount_idx, &entry)?;

        let offset = port.alloc(VfsStat::SIZE as u32).ok_or(VfsError::PoolFull)?;

        let tag = port.next_tag();
        let sqe = IoSqe {
            opcode: fs_op::STAT,
            flags: 0,
            priority: 0,
            tag,
            lba: 0,
            data_offset: offset,
            data_len: VfsStat::SIZE as u32,
            param: fs_handle as u64,
        };

        if !port.submit(&sqe) {
            return Err(VfsError::RingFull);
        }
        port.notify();

        let cqe = poll_completion(port, tag)?;

        if cqe.status != io_status::OK {
            port.reset_pool();
            return Err(VfsError::from_cqe(&cqe));
        }

        let mut raw = [0u8; 16];
        port.pool_read(offset, &mut raw);

        port.reset_pool();

        VfsStat::from_bytes(&raw, 0).ok_or(VfsError::IoError)
    }

    // =========================================================================
    // Virtual root directory
    // =========================================================================

    /// List virtual root directory: /bin (ramfs) + /mnt (synthetic mount parent).
    fn readdir_virtual_root(&self, entries: &mut [VfsDirEntry]) -> Result<usize, VfsError> {
        let mut count = 0;

        // Add "bin" directory (initrd)
        if count < entries.len() {
            let mut e = VfsDirEntry::empty();
            e.set_name(b"bin");
            e.file_type = file_type::DIR;
            entries[count] = e;
            count += 1;
        }

        // Add "mnt" directory (synthetic parent for mounts)
        if count < entries.len() {
            let mut e = VfsDirEntry::empty();
            e.set_name(b"mnt");
            e.file_type = file_type::DIR;
            entries[count] = e;
            count += 1;
        }

        Ok(count)
    }

    /// List /bin directory from ramfs (initrd binaries).
    ///
    /// Ramfs entries have TAR paths like `./bin/devd` or `bin/devd`.
    /// We filter to entries under `bin/` and strip that prefix.
    fn readdir_bin(entries: &mut [VfsDirEntry]) -> Result<usize, VfsError> {
        let mut ramfs_buf = [RamfsListEntry::empty(); 16];
        let ret = syscall::ramfs_list(&mut ramfs_buf);
        if ret < 0 {
            return Err(VfsError::IoError);
        }

        let ramfs_count = ret as usize;
        let mut count = 0;

        for i in 0..ramfs_count {
            if count >= entries.len() {
                break;
            }
            let full_name = ramfs_buf[i].name_str();

            // Strip leading "./" if present
            let name = if full_name.starts_with(b"./") {
                &full_name[2..]
            } else {
                full_name
            };

            // Filter to direct children of bin/
            let child_name = if name.starts_with(b"bin/") {
                let rest = &name[4..];
                // Strip trailing slash
                let rest = if !rest.is_empty() && rest[rest.len() - 1] == b'/' {
                    &rest[..rest.len() - 1]
                } else {
                    rest
                };
                // Skip if empty (the "bin/" directory entry itself)
                if rest.is_empty() {
                    continue;
                }
                // Skip nested entries (e.g., bin/sub/file)
                if rest.iter().any(|&c| c == b'/') {
                    continue;
                }
                rest
            } else {
                continue;
            };

            let mut e = VfsDirEntry::empty();
            e.set_name(child_name);
            e.file_type = if ramfs_buf[i].file_type == 1 {
                file_type::DIR
            } else {
                file_type::FILE
            };
            e.size = ramfs_buf[i].size as u32;
            entries[count] = e;
            count += 1;
        }

        Ok(count)
    }

    /// List /mnt directory by extracting top-level names from mount prefixes.
    fn readdir_mnt(&self, entries: &mut [VfsDirEntry]) -> Result<usize, VfsError> {
        let (mount_entries, _) = self.read_mount_table();
        let mut count = 0;

        for i in 0..MAX_MOUNTS {
            if mount_entries[i].in_use == 0 || count >= entries.len() {
                continue;
            }
            let prefix = mount_entries[i].prefix_bytes();
            // Match "/mnt/X/" → extract "X"
            if prefix.starts_with(b"/mnt/") {
                let rest = &prefix[5..];
                let name_end = rest.iter().position(|&c| c == b'/').unwrap_or(rest.len());
                if name_end > 0 {
                    let mut e = VfsDirEntry::empty();
                    e.set_name(&rest[..name_end]);
                    e.file_type = file_type::DIR;
                    entries[count] = e;
                    count += 1;
                }
            }
        }

        Ok(count)
    }
}

// =============================================================================
// Helpers
// =============================================================================

fn decode_handle(handle: u32) -> (usize, u32) {
    let mount_idx = (handle >> 24) as usize;
    let fs_handle = handle & 0x00FF_FFFF;
    (mount_idx, fs_handle)
}

/// Poll for a specific completion by tag, with timeout.
fn poll_completion(port: &DataPort, tag: u32) -> Result<IoCqe, VfsError> {
    for _ in 0..2000 {
        if let Some(cqe) = port.poll_cq() {
            if cqe.tag == tag {
                return Ok(cqe);
            }
        }
        syscall::sleep_us(1000);
    }
    Err(VfsError::Timeout)
}

/// Receive IPC response with timeout (in ms).
fn recv_with_timeout(channel: &mut Channel, timeout_ms: u64) -> Option<[u8; 512]> {
    let mux = Mux::new().ok()?;
    let mut timer = Timer::new().ok()?;

    // Timer::set() takes a relative duration in nanoseconds
    timer.set(timeout_ms * 1_000_000).ok()?;

    mux.add(channel.handle(), MuxFilter::Readable).ok()?;
    mux.add(timer.handle(), MuxFilter::Readable).ok()?;

    let event = mux.wait().ok()?;
    if event.handle == timer.handle() {
        return None;
    }

    let mut response = [0u8; 512];
    match channel.recv(&mut response) {
        Ok(n) if n > 0 => Some(response),
        _ => None,
    }
}
