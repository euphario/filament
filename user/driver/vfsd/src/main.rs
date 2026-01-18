//! VFS Daemon - Virtual Filesystem Service
//!
//! Provides VfsProtocol IPC interface to the initrd filesystem.
//! Mount point: /bin (read-only)

#![no_std]
#![no_main]

use userlib::{
    println,
    uinfo, uerror,
    syscall::{
        self, port_register, port_accept, receive, send,
        open, read, close,
        ramfs_list, RamfsEntry,
    },
    ipc::{
        Message,  // Import Message trait for serialize/deserialize
        protocols::{
            FsRequest, FsResponse, FsInfo, FileStat, DirEntry, FileType,
            fs_caps, fs_error, filesystem::MAX_DIR_ENTRIES,
        },
    },
};

const PORT_NAME: &[u8] = b"vfs";
const MOUNT_POINT: &str = "/bin";
const FS_TYPE: &str = "initrd";

/// Maximum ramfs entries we support
const MAX_RAMFS_ENTRIES: usize = 64;

/// Cached ramfs entries for directory listing
/// Using raw pointers to avoid mutable reference warnings on static
static mut RAMFS_CACHE: [RamfsEntry; MAX_RAMFS_ENTRIES] = [RamfsEntry::empty(); MAX_RAMFS_ENTRIES];
static mut RAMFS_COUNT: usize = 0;

/// Get mutable reference to ramfs cache (single-threaded, safe in our context)
#[inline]
fn ramfs_cache() -> &'static mut [RamfsEntry; MAX_RAMFS_ENTRIES] {
    unsafe { &mut *core::ptr::addr_of_mut!(RAMFS_CACHE) }
}

/// Get ramfs count
#[inline]
fn ramfs_count() -> usize {
    unsafe { *core::ptr::addr_of!(RAMFS_COUNT) }
}

/// Set ramfs count
#[inline]
fn set_ramfs_count(count: usize) {
    unsafe { *core::ptr::addr_of_mut!(RAMFS_COUNT) = count; }
}

/// Initialize ramfs cache
fn init_ramfs_cache() -> i64 {
    let count = ramfs_list(ramfs_cache());
    if count >= 0 {
        set_ramfs_count(count as usize);
        uinfo!("vfsd", "init_ok"; entries = count as u64);
    } else {
        uerror!("vfsd", "init_failed"; err = count);
    }
    count
}

/// Get filesystem info
fn handle_get_info() -> FsResponse {
    let info = FsInfo::new(
        fs_caps::READ | fs_caps::EXECUTE,  // Read-only, executable
        MOUNT_POINT,
        FS_TYPE,
    );
    FsResponse::Info(info)
}

/// Stat a file
fn handle_stat(path: &[u8]) -> FsResponse {
    let path_str = core::str::from_utf8(path).unwrap_or("");

    // Strip leading "/" for comparison with ramfs entries
    let lookup = path_str.trim_start_matches('/');

    // Search ramfs cache
    {
        let cache = ramfs_cache();
        for i in 0..ramfs_count() {
            let entry = &cache[i];
            let entry_name = entry.name_str();

            // Strip leading "./" from tar entries
            let entry_clean = entry_name.trim_start_matches("./");

            if entry_clean == lookup {
                return FsResponse::Stat(FileStat {
                    file_type: if entry.is_dir() { 1 } else { 0 },
                    size: entry.size,
                    created: 0,
                    modified: 0,
                });
            }
        }
    }

    FsResponse::Error(fs_error::NOT_FOUND)
}

/// Read directory entries
fn handle_read_dir(path: &[u8], offset: u32) -> FsResponse {
    let path_str = core::str::from_utf8(path).unwrap_or("");

    // We only support "/" or "/bin" or empty path for now
    let lookup_prefix = path_str.trim_start_matches('/').trim_end_matches('/');

    let mut entries: [DirEntry; MAX_DIR_ENTRIES] = core::array::from_fn(|_| DirEntry::default());
    let mut count = 0u8;
    let mut skipped = 0u32;

    {
        let cache = ramfs_cache();
        for i in 0..ramfs_count() {
            if count as usize >= MAX_DIR_ENTRIES {
                break;
            }

            let entry = &cache[i];
            let entry_name = entry.name_str();

            // Strip leading "./" from tar entries
            let entry_clean = entry_name.trim_start_matches("./");

            // Check if entry is in the requested directory
            let in_dir = if lookup_prefix.is_empty() || lookup_prefix == "bin" {
                // Root directory - include all bin/* entries
                entry_clean.starts_with("bin/") || entry_clean == "bin"
            } else {
                // Subdirectory - match prefix
                entry_clean.starts_with(lookup_prefix) && entry_clean.len() > lookup_prefix.len()
            };

            if !in_dir {
                continue;
            }

            // Handle pagination
            if skipped < offset {
                skipped += 1;
                continue;
            }

            // Extract just the filename (last component)
            let filename = entry_clean.rsplit('/').next().unwrap_or(entry_clean);
            if filename.is_empty() {
                continue;
            }

            // Create directory entry
            let mut dir_entry = DirEntry::default();
            let name_len = filename.len().min(64);
            dir_entry.name[..name_len].copy_from_slice(filename.as_bytes());
            dir_entry.name_len = name_len as u8;
            dir_entry.file_type = if entry.is_dir() { FileType::Directory } else { FileType::Regular };
            dir_entry.size = entry.size;

            entries[count as usize] = dir_entry;
            count += 1;
        }
    }

    let more = false; // TODO: proper pagination
    FsResponse::DirEntries { entries, count, more }
}

/// Read file to shared memory
fn handle_read_to_shmem(path: &[u8], shmem_id: u32, max_size: u32, _requester_pid: u32) -> FsResponse {
    // Try to open the file via kernel ramfs
    let fd = open(path, 0); // O_RDONLY
    if fd < 0 {
        return FsResponse::Error(fs_error::NOT_FOUND);
    }
    let fd = fd as u32;

    // Map the shared memory
    let mut vaddr: u64 = 0;
    let mut _paddr: u64 = 0;
    let result = syscall::shmem_map(shmem_id, &mut vaddr, &mut _paddr);
    if result < 0 {
        close(fd);
        return FsResponse::Error(fs_error::IO);
    }

    // Read file into shared memory
    let buf = unsafe { core::slice::from_raw_parts_mut(vaddr as *mut u8, max_size as usize) };
    let mut total_read = 0usize;

    loop {
        let chunk = &mut buf[total_read..];
        if chunk.is_empty() {
            break;
        }

        let n = read(fd, chunk);
        if n < 0 {
            close(fd);
            syscall::shmem_unmap(shmem_id);
            return FsResponse::Error(fs_error::IO);
        }
        if n == 0 {
            break; // EOF
        }
        total_read += n as usize;
    }

    close(fd);
    syscall::shmem_unmap(shmem_id);

    FsResponse::ShmemRead { bytes: total_read as u64 }
}

/// Handle a single IPC request
fn handle_request(request: &FsRequest) -> FsResponse {
    match request {
        FsRequest::GetInfo => handle_get_info(),

        FsRequest::Stat { path, path_len } => {
            handle_stat(&path[..*path_len as usize])
        }

        FsRequest::ReadDir { path, path_len, offset } => {
            handle_read_dir(&path[..*path_len as usize], *offset)
        }

        FsRequest::ReadToShmem { path, path_len, shmem_id, max_size, requester_pid } => {
            handle_read_to_shmem(
                &path[..*path_len as usize],
                *shmem_id,
                *max_size,
                *requester_pid,
            )
        }

        // Read-only filesystem doesn't support these
        FsRequest::Open { .. } => FsResponse::Error(fs_error::PERMISSION),
        FsRequest::Read { .. } => FsResponse::Error(fs_error::PERMISSION),
        FsRequest::Write { .. } => FsResponse::Error(fs_error::PERMISSION),
        FsRequest::Close { .. } => FsResponse::Error(fs_error::PERMISSION),
    }
}

/// IPC request/response buffers
static mut REQ_BUF: [u8; 512] = [0u8; 512];
static mut RSP_BUF: [u8; 2048] = [0u8; 2048];

/// Get mutable reference to request buffer (single-threaded, safe in our context)
#[inline]
fn req_buf() -> &'static mut [u8; 512] {
    unsafe { &mut *core::ptr::addr_of_mut!(REQ_BUF) }
}

/// Get mutable reference to response buffer (single-threaded, safe in our context)
#[inline]
fn rsp_buf() -> &'static mut [u8; 2048] {
    unsafe { &mut *core::ptr::addr_of_mut!(RSP_BUF) }
}

#[unsafe(no_mangle)]
fn main() {
    println!("vfsd: Starting VFS daemon...");

    // Initialize ramfs cache
    if init_ramfs_cache() < 0 {
        uerror!("vfsd", "startup_failed"; reason = "ramfs_list failed");
        return;
    }

    // Register our port
    let result = port_register(PORT_NAME);
    if result < 0 {
        uerror!("vfsd", "port_register_failed"; err = result);
        return;
    }
    let port_id = result as u32;

    uinfo!("vfsd", "ready"; port = "vfs", entries = ramfs_count() as u64);

    // Main loop: accept connections and handle requests
    loop {
        // Accept a connection
        let result = port_accept(port_id);
        if result < 0 {
            syscall::yield_now();
            continue;
        }
        let channel_id = result as u32;

        // Handle requests on this channel
        loop {
            // Receive request
            let n = receive(channel_id, req_buf());
            if n <= 0 {
                // Channel closed or error
                syscall::channel_close(channel_id);
                break;
            }

            // Deserialize request
            let request = match FsRequest::deserialize(&req_buf()[..n as usize]) {
                Ok((req, _)) => req,
                Err(_) => {
                    // Send error response
                    let response = FsResponse::Error(fs_error::INVALID);
                    if let Ok(len) = response.serialize(rsp_buf()) {
                        let _ = send(channel_id, &rsp_buf()[..len]);
                    }
                    continue;
                }
            };

            // Handle request
            let response = handle_request(&request);

            // Serialize and send response
            match response.serialize(rsp_buf()) {
                Ok(len) => {
                    let _ = send(channel_id, &rsp_buf()[..len]);
                }
                Err(_) => {
                    // Send error response
                    let err_response = FsResponse::Error(fs_error::IO);
                    if let Ok(len) = err_response.serialize(rsp_buf()) {
                        let _ = send(channel_id, &rsp_buf()[..len]);
                    }
                }
            }
        }
    }
}
