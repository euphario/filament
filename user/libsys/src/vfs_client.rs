//! VFS Client Library
//!
//! Resolves paths via devd's mount table, then connects directly to FS
//! drivers' DataPorts for I/O. No vfsd process needed.
//!
//! ## Architecture
//!
//! ```text
//! VfsClient
//!   ├── asks devd-query to resolve path → (shmem_id, remaining_path)
//!   └── connects to fatfsd DataPort → issues VFS ops directly
//! ```

use crate::data_port::DataPort;
use crate::ipc::{Channel, Timer, Mux, MuxFilter};
use crate::ring::{IoSqe, IoCqe, io_status};
use crate::vfs_proto::{fs_op, file_type, VfsDirEntry, VfsStat};
use crate::query::{
    QueryHeader, ResolvePath, ResolvePathResponse, MountsListResponse, MountListEntry,
    mount_transport, msg, error,
};
use crate::syscall;
use crate::syscall::RamfsListEntry;

// =============================================================================
// Constants
// =============================================================================

const MAX_CONNECTIONS: usize = 8;

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
    /// devd not found or not available
    NotAvailable,
    /// Connection to devd failed
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
        // Check result code first — error status CQEs carry the specific
        // error in `result` (e.g., NOT_FOUND, PERMISSION).
        match cqe.result {
            vfs_error::NOT_FOUND => VfsError::NotFound,
            vfs_error::PERMISSION => VfsError::Permission,
            vfs_error::NO_SPACE => VfsError::PoolFull,
            vfs_error::NOT_DIR => VfsError::NotDir,
            vfs_error::IS_DIR => VfsError::IsDir,
            vfs_error::NO_MOUNT => VfsError::NoMount,
            vfs_error::TOO_MANY => VfsError::TooMany,
            vfs_error::READ_ONLY => VfsError::ReadOnly,
            _ if cqe.status != io_status::OK => VfsError::IoError,
            _ => VfsError::IoError,
        }
    }
}

// =============================================================================
// Cached mount connection
// =============================================================================

struct MountConnection {
    port: DataPort,
    shmem_id: u32,
}

// =============================================================================
// VfsClient
// =============================================================================

/// Client for VFS operations.
///
/// Resolves paths via devd-query, connects directly to FS drivers' DataPorts.
pub struct VfsClient {
    /// Channel to devd-query for path resolution
    devd_channel: Channel,
    /// Cached per-mount DataPort connections (keyed by shmem_id)
    connections: [Option<MountConnection>; MAX_CONNECTIONS],
}

impl VfsClient {
    /// Discover devd-query and create a VfsClient.
    pub fn discover() -> Result<Self, VfsError> {
        let channel = Channel::connect(b"devd-query:")
            .map_err(|_| VfsError::NotAvailable)?;

        Ok(Self {
            devd_channel: channel,
            connections: [
                None, None, None, None,
                None, None, None, None,
            ],
        })
    }

    /// Resolve a path via devd, returning (shmem_id, remaining_path).
    fn resolve_path(&mut self, path: &[u8]) -> Result<(u32, [u8; 64], usize), VfsError> {
        let mut buf = [0u8; ResolvePath::MAX_SIZE];
        let len = ResolvePath::write_to(&mut buf, 1, path)
            .ok_or(VfsError::NotAvailable)?;

        self.devd_channel.send(&buf[..len])
            .map_err(|_| VfsError::NotAvailable)?;

        let response = recv_with_timeout(&mut self.devd_channel, 2000)
            .ok_or(VfsError::Timeout)?;

        let (result, transport_type, _port_name, remaining, shmem_id) =
            ResolvePathResponse::from_bytes(&response)
                .ok_or(VfsError::NotAvailable)?;

        if result != 0 {
            return Err(VfsError::NoMount);
        }

        if transport_type != mount_transport::DATAPORT || shmem_id == 0 {
            return Err(VfsError::NotAvailable);
        }

        let mut rem = [0u8; 64];
        let rem_len = remaining.len().min(64);
        rem[..rem_len].copy_from_slice(&remaining[..rem_len]);

        Ok((shmem_id, rem, rem_len))
    }

    /// Get or create a DataPort connection for a shmem_id.
    fn get_connection(&mut self, shmem_id: u32) -> Result<usize, VfsError> {
        // Check if already connected
        for i in 0..MAX_CONNECTIONS {
            if let Some(conn) = &self.connections[i] {
                if conn.shmem_id == shmem_id {
                    return Ok(i);
                }
            }
        }

        // Find free slot
        let slot = self.connections.iter().position(|c| c.is_none())
            .ok_or(VfsError::TooMany)?;

        let port = DataPort::connect(shmem_id)
            .map_err(|_| VfsError::ConnectFailed)?;

        self.connections[slot] = Some(MountConnection {
            port,
            shmem_id,
        });

        Ok(slot)
    }

    /// Open a file or directory.
    ///
    /// Returns a handle encoded as `(connection_idx << 24) | fs_handle`.
    pub fn open(&mut self, path: &[u8], flags: u32) -> Result<u32, VfsError> {
        // Handle virtual root
        if path == b"/" {
            return Ok(HANDLE_ROOT);
        }
        if path == b"/bin" || path == b"/bin/" {
            return Ok(HANDLE_BIN);
        }
        if path == b"/mnt" || path == b"/mnt/" {
            return Ok(HANDLE_MNT);
        }

        let (shmem_id, remaining, rem_len) = self.resolve_path(path)?;
        let conn_idx = self.get_connection(shmem_id)?;
        let port = &mut self.connections[conn_idx].as_mut().unwrap().port;

        // Build path for FS: prepend "/" to remainder
        let mut fs_path = [0u8; 256];
        fs_path[0] = b'/';
        let copy_len = rem_len.min(255);
        fs_path[1..1 + copy_len].copy_from_slice(&remaining[..copy_len]);
        let fs_path_len = 1 + copy_len;

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

        let encoded = ((conn_idx as u32) << 24) | (cqe.result & 0x00FF_FFFF);
        Ok(encoded)
    }

    /// Read from an open file.
    pub fn read(
        &mut self,
        handle: u32,
        file_offset: u64,
        buf: &mut [u8],
    ) -> Result<usize, VfsError> {
        let (conn_idx, fs_handle) = decode_handle(handle);

        if conn_idx >= MAX_CONNECTIONS || self.connections[conn_idx].is_none() {
            return Err(VfsError::NotFound);
        }

        // Read in pool-slot-sized chunks (pool allocator rejects > slot_size)
        let slot_size = crate::ring::POOL_SLOT_SIZE as usize;
        let mut total = 0usize;

        while total < buf.len() {
            let chunk = (buf.len() - total).min(slot_size);
            let port = &mut self.connections[conn_idx].as_mut().unwrap().port;

            let chunk_len = chunk as u32;
            let offset = port.alloc(chunk_len).ok_or(VfsError::PoolFull)?;

            let tag = port.next_tag();
            let sqe = IoSqe {
                opcode: fs_op::READ,
                flags: 0,
                priority: 0,
                tag,
                lba: file_offset + total as u64,
                data_offset: offset,
                data_len: chunk_len,
                param: fs_handle as u64,
            };

            if !port.submit(&sqe) {
                port.reset_pool();
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
                port.pool_read(offset, &mut buf[total..total + transferred]);
            }

            port.reset_pool();
            total += transferred;

            // Short read = EOF
            if transferred < chunk {
                break;
            }
        }

        Ok(total)
    }

    /// Close an open file handle.
    pub fn close(&mut self, handle: u32) -> Result<(), VfsError> {
        if handle == HANDLE_ROOT || handle == HANDLE_BIN || handle == HANDLE_MNT {
            return Ok(());
        }

        let (conn_idx, fs_handle) = decode_handle(handle);

        if conn_idx >= MAX_CONNECTIONS || self.connections[conn_idx].is_none() {
            return Err(VfsError::NotFound);
        }

        let port = &mut self.connections[conn_idx].as_mut().unwrap().port;

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
    pub fn readdir(
        &mut self,
        handle: u32,
        entries: &mut [VfsDirEntry],
    ) -> Result<usize, VfsError> {
        if handle == HANDLE_ROOT {
            return self.readdir_virtual_root(entries);
        }
        if handle == HANDLE_BIN {
            return Self::readdir_bin(entries);
        }
        if handle == HANDLE_MNT {
            return self.readdir_mnt(entries);
        }

        let (conn_idx, fs_handle) = decode_handle(handle);

        if conn_idx >= MAX_CONNECTIONS || self.connections[conn_idx].is_none() {
            return Err(VfsError::NotFound);
        }

        let port = &mut self.connections[conn_idx].as_mut().unwrap().port;

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

        let (conn_idx, fs_handle) = decode_handle(handle);

        if conn_idx >= MAX_CONNECTIONS || self.connections[conn_idx].is_none() {
            return Err(VfsError::NotFound);
        }

        let port = &mut self.connections[conn_idx].as_mut().unwrap().port;

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

    fn readdir_virtual_root(&self, entries: &mut [VfsDirEntry]) -> Result<usize, VfsError> {
        let mut count = 0;

        if count < entries.len() {
            let mut e = VfsDirEntry::empty();
            e.set_name(b"bin");
            e.file_type = file_type::DIR;
            entries[count] = e;
            count += 1;
        }

        if count < entries.len() {
            let mut e = VfsDirEntry::empty();
            e.set_name(b"mnt");
            e.file_type = file_type::DIR;
            entries[count] = e;
            count += 1;
        }

        Ok(count)
    }

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

            let name = if full_name.starts_with(b"./") {
                &full_name[2..]
            } else {
                full_name
            };

            let child_name = if name.starts_with(b"bin/") {
                let rest = &name[4..];
                let rest = if !rest.is_empty() && rest[rest.len() - 1] == b'/' {
                    &rest[..rest.len() - 1]
                } else {
                    rest
                };
                if rest.is_empty() {
                    continue;
                }
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

    /// List /mnt directory by querying devd for mount prefixes.
    fn readdir_mnt(&mut self, entries: &mut [VfsDirEntry]) -> Result<usize, VfsError> {
        // Send LIST_MOUNTS to devd
        let header = QueryHeader::new(msg::LIST_MOUNTS, 2);
        let buf = header.to_bytes();
        self.devd_channel.send(&buf)
            .map_err(|_| VfsError::NotAvailable)?;

        let response = recv_with_timeout(&mut self.devd_channel, 2000)
            .ok_or(VfsError::Timeout)?;

        let resp_header = QueryHeader::from_bytes(&response)
            .ok_or(VfsError::NotAvailable)?;

        if resp_header.msg_type != msg::MOUNTS_LIST {
            return Err(VfsError::NotAvailable);
        }

        let (entry_count, _total) = MountsListResponse::parse_header(&response)
            .ok_or(VfsError::NotAvailable)?;

        let mut count = 0;
        let data = &response[MountsListResponse::HEADER_SIZE..];
        for i in 0..entry_count as usize {
            if count >= entries.len() {
                break;
            }
            let offset = i * MountListEntry::SIZE;
            if offset + MountListEntry::SIZE > data.len() {
                break;
            }
            if let Some((_transport, prefix, _port_name, _shmem_id)) =
                MountListEntry::from_bytes(&data[offset..])
            {
                // Extract mount name from prefix (e.g., "/mnt/usb0/" → "usb0")
                let name = if prefix.starts_with(b"/mnt/") {
                    let rest = &prefix[5..];
                    let name_end = rest.iter().position(|&c| c == b'/').unwrap_or(rest.len());
                    &rest[..name_end]
                } else if prefix.starts_with(b"/") {
                    // Non-mnt prefix, show the path segment after first /
                    let rest = &prefix[1..];
                    let name_end = rest.iter().position(|&c| c == b'/').unwrap_or(rest.len());
                    &rest[..name_end]
                } else {
                    prefix
                };

                if !name.is_empty() {
                    let mut e = VfsDirEntry::empty();
                    e.set_name(name);
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
    let conn_idx = (handle >> 24) as usize;
    let fs_handle = handle & 0x00FF_FFFF;
    (conn_idx, fs_handle)
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
