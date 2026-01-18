//! FAT Filesystem Driver
//!
//! Userspace FAT12/16/32 filesystem driver that reads from block storage
//! (NVMe or USB mass storage) via ring buffers.
//!
//! Uses type-safe IPC via `Server<FsProtocol>`.

#![no_std]
#![no_main]
#![allow(dead_code)]  // FAT parsing utilities reserved for future use

use userlib::syscall;
use userlib::{uinfo, uwarn, uerror};
use userlib::ipc::{Server, Connection, IpcError};
use userlib::ipc::protocols::{FsProtocol, FsRequest, FsResponse, fs_error};

mod fat;

use fat::{FatFilesystem, FatType, LfnCollector};
// Use USB library's BlockClient for ring buffer access
use usb::BlockClient;

// Entry point
#[unsafe(no_mangle)]
fn main() {
    uinfo!("fatfs", "init_start");

    // Try NVMe first (faster), then fall back to USB
    let mut block = if let Some(b) = BlockClient::connect(b"nvme") {
        uinfo!("fatfs", "block_connected"; device = "nvme");
        b
    } else if let Some(b) = BlockClient::connect(b"usb") {
        uinfo!("fatfs", "block_connected"; device = "usb");
        b
    } else {
        uerror!("fatfs", "no_block_device");
        syscall::exit(1);
    };

    // Get block device info
    // NOTE: port_connect() blocks until usbd has MSC ready, so this should succeed immediately
    let (block_size, block_count) = match block.get_info() {
        Some(info) => info,
        None => {
            uerror!("fatfs", "read_error"; sector = 0u64);
            syscall::exit(1);
        }
    };

    // Read sector 0 (could be MBR or FAT boot sector)
    let bs = block_size as usize;
    if bs > 4096 {
        uerror!("fatfs", "read_error"; sector = 0u64);
        syscall::exit(1);
    }

    let mut sector0 = [0u8; 4096];
    if block.read_sector(0).is_none() {
        uerror!("fatfs", "read_error"; sector = 0u64);
        syscall::exit(1);
    }
    sector0[..bs].copy_from_slice(&block.data()[..bs]);

    // Check boot signature
    let (sig0, sig1) = if bs >= 512 {
        let end_sig = (sector0[bs - 2], sector0[bs - 1]);
        let std_sig = (sector0[510], sector0[511]);
        if end_sig == (0x55, 0xAA) {
            end_sig
        } else if std_sig == (0x55, 0xAA) {
            std_sig
        } else {
            std_sig
        }
    } else {
        (sector0[bs - 2], sector0[bs - 1])
    };

    // Check if this is an MBR (partition table) or direct FAT boot sector
    // FAT boot sector starts with jump: EB xx 90 or E9 xx xx
    // MBR starts with executable code (often 33 C0 = xor ax,ax)
    let partition_start = if sector0[0] == 0xEB || sector0[0] == 0xE9 {
        // Direct FAT boot sector (no partition table)
        0u64
    } else {
        // Check MBR signature
        if sig0 != 0x55 || sig1 != 0xAA {
            uerror!("fatfs", "mount_failed");
            syscall::exit(1);
        }

        // Parse partition table (starts at offset 0x1BE)
        let mut found_lba = 0u64;

        for i in 0..4 {
            let entry_offset = 0x1BE + i * 16;
            let part_type = sector0[entry_offset + 4];
            let lba_start = u32::from_le_bytes([
                sector0[entry_offset + 8],
                sector0[entry_offset + 9],
                sector0[entry_offset + 10],
                sector0[entry_offset + 11],
            ]);
            let _size_sectors = u32::from_le_bytes([
                sector0[entry_offset + 12],
                sector0[entry_offset + 13],
                sector0[entry_offset + 14],
                sector0[entry_offset + 15],
            ]);

            // Use first FAT partition found
            if part_type != 0 && found_lba == 0 && (part_type == 0x01 || part_type == 0x04 ||
                part_type == 0x06 || part_type == 0x0B || part_type == 0x0C || part_type == 0x0E) {
                found_lba = lba_start as u64;
            }
        }

        if found_lba == 0 {
            uerror!("fatfs", "mount_failed");
            syscall::exit(1);
        }

        found_lba
    };

    // Read actual FAT boot sector
    let mut boot_sector = [0u8; 512];
    if partition_start == 0 {
        boot_sector.copy_from_slice(&sector0);
    } else {
        if block.read_sector(partition_start).is_none() {
            uerror!("fatfs", "read_error"; sector = partition_start);
            syscall::exit(1);
        }
        boot_sector.copy_from_slice(&block.data()[..512]);
    }

    // Parse FAT filesystem
    let fs = match FatFilesystem::mount(&boot_sector, block_size, block_count) {
        Some(mut fs) => {
            // Adjust sector offsets for partition start
            fs.partition_start = partition_start as u32;
            fs
        }
        None => {
            uerror!("fatfs", "mount_failed");
            syscall::exit(1);
        }
    };

    uinfo!("fatfs", "mounted"; fat_type = "FAT32", clusters = fs.total_clusters as u64);

    // Register as filesystem service
    let server = match Server::<FsProtocol>::register() {
        Ok(s) => {
            uinfo!("fatfs", "port_registered");
            s
        }
        Err(IpcError::ResourceBusy) => {
            uwarn!("fatfs", "already_running");
            syscall::exit(1);
        }
        Err(_) => {
            uerror!("fatfs", "port_register_failed");
            syscall::exit(1);
        }
    };

    // Service loop (devd supervises us, no need to daemonize)
    filesystem_service_loop(&fs, &mut block, &server);
}

/// Handle filesystem requests from clients
fn filesystem_service_loop(fs: &FatFilesystem, block: &mut BlockClient, server: &Server<FsProtocol>) {
    let my_pid = syscall::getpid();
    uinfo!("fatfs", "service_running"; pid = my_pid as u64);

    loop {
        // Accept a connection
        let mut conn: Connection<FsProtocol> = match server.accept() {
            Ok(c) => c,
            Err(_) => {
                syscall::yield_now();
                continue;
            }
        };

        // Perform PID handshake (receive client PID, send our PID)
        let client_pid = match conn.handshake_pid() {
            Ok(pid) => {
                uinfo!("fatfs", "client_connected"; pid = pid as u64);
                pid
            }
            Err(_) => {
                continue;
            }
        };

        // Handle requests on this connection until disconnect
        loop {
            match conn.receive() {
                Ok(request) => {
                    let response = handle_fs_request(fs, block, &request, client_pid);
                    if conn.send(&response).is_err() {
                        break; // Client disconnected
                    }
                }
                Err(IpcError::ChannelClosed) => break,
                Err(IpcError::WouldBlock) => {
                    syscall::yield_now();
                    continue;
                }
                Err(_) => break,
            }
        }
        // Connection dropped here, channel closed automatically
    }
}

/// Handle a single filesystem request
fn handle_fs_request(
    fs: &FatFilesystem,
    block: &mut BlockClient,
    request: &FsRequest,
    _client_pid: u32,
) -> FsResponse {
    match request {
        FsRequest::ReadToShmem { path, path_len, shmem_id, max_size, requester_pid: _ } => {
            let filename = &path[..*path_len as usize];
            let filename_str = core::str::from_utf8(filename).unwrap_or("???");

            // Map the shared memory
            let mut vaddr: u64 = 0;
            let mut paddr: u64 = 0;
            let map_result = syscall::shmem_map(*shmem_id, &mut vaddr, &mut paddr);

            if map_result < 0 {
                uerror!("fatfs", "read_error"; sector = 0u64);
                return FsResponse::Error(fs_error::IO);
            }

            // Validate addresses before use
            if vaddr == 0 || vaddr > 0x7FFFFFFFFFFF {
                uerror!("fatfs", "read_error"; sector = 0u64);
                syscall::shmem_unmap(*shmem_id);
                return FsResponse::Error(fs_error::IO);
            }

            // Find the file
            let file = match find_file(fs, block, filename) {
                Some(f) => f,
                None => {
                    uerror!("fatfs", "file_not_found"; name = filename_str);
                    syscall::shmem_unmap(*shmem_id);
                    return FsResponse::Error(fs_error::NOT_FOUND);
                }
            };

            // Check size
            if file.size > *max_size {
                syscall::shmem_unmap(*shmem_id);
                return FsResponse::Error(fs_error::NO_SPACE);
            }

            // Read file into shared memory
            let buffer = unsafe {
                core::slice::from_raw_parts_mut(vaddr as *mut u8, *max_size as usize)
            };

            let bytes_read = match read_file(fs, block, &file, buffer) {
                Some(n) => n,
                None => {
                    uerror!("fatfs", "read_error"; sector = 0u64);
                    syscall::shmem_unmap(*shmem_id);
                    return FsResponse::Error(fs_error::IO);
                }
            };

            uinfo!("fatfs", "file_loaded"; bytes = bytes_read as u64);

            // Unmap shared memory - we're done with it
            syscall::shmem_unmap(*shmem_id);

            FsResponse::ShmemRead { bytes: bytes_read as u64 }
        }

        FsRequest::Stat { path, path_len } => {
            let filename = &path[..*path_len as usize];

            match find_file(fs, block, filename) {
                Some(file) => {
                    use userlib::ipc::protocols::FileStat;
                    FsResponse::Stat(FileStat {
                        file_type: 0, // Regular file
                        size: file.size as u64,
                        created: 0,
                        modified: 0,
                    })
                }
                None => FsResponse::Error(fs_error::NOT_FOUND),
            }
        }

        // Other operations not yet implemented
        _ => {
            FsResponse::Error(fs_error::INVALID)
        }
    }
}

/// File info returned by find_file
#[derive(Debug, Clone, Copy)]
pub struct FileInfo {
    pub start_cluster: u32,
    pub size: u32,
}

/// Case-insensitive byte slice comparison
fn name_matches(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    for i in 0..a.len() {
        if a[i].to_ascii_uppercase() != b[i].to_ascii_uppercase() {
            return false;
        }
    }
    true
}

/// Find a file in the root directory by name (supports both 8.3 and LFN)
fn find_file(fs: &FatFilesystem, block: &mut BlockClient, filename: &[u8]) -> Option<FileInfo> {
    let mut sector_buf = [0u8; 512];
    let mut lfn = LfnCollector::new();

    let root_start_sector = if fs.fat_type == FatType::Fat32 {
        fs.cluster_to_sector(fs.root_cluster)
    } else {
        fs.root_dir_absolute_sector()
    };

    // Search multiple sectors (up to 32 sectors = 512 entries)
    for sector_offset in 0..32u32 {
        if block.read_sector((root_start_sector + sector_offset) as u64).is_none() {
            break;
        }
        sector_buf.copy_from_slice(&block.data()[..512]);

        for i in 0..16 {
            let entry = &sector_buf[i * 32..(i + 1) * 32];

            // End of directory
            if entry[0] == 0 {
                return None;
            }

            // Deleted entry - reset LFN
            if entry[0] == 0xE5 {
                lfn.reset();
                continue;
            }

            // LFN entry
            if entry[11] == 0x0F {
                lfn.add_lfn_entry(entry);
                continue;
            }

            // Regular 8.3 entry - check if we have a matching LFN or 8.3 name
            let has_lfn = lfn.is_complete();

            // Try LFN match first
            if has_lfn {
                let lfn_name = lfn.name_slice();
                if name_matches(lfn_name, filename) {
                    let start_cluster = u32::from_le_bytes([entry[26], entry[27], entry[20], entry[21]]);
                    let size = u32::from_le_bytes([entry[28], entry[29], entry[30], entry[31]]);
                    return Some(FileInfo { start_cluster, size });
                }
            }

            // Try 8.3 name match
            let mut short_name = [0u8; 12];
            let mut short_len = 0;
            for j in 0..8 {
                if entry[j] != 0x20 {
                    short_name[short_len] = entry[j];
                    short_len += 1;
                }
            }
            if entry[8] != 0x20 {
                short_name[short_len] = b'.';
                short_len += 1;
                for j in 8..11 {
                    if entry[j] != 0x20 {
                        short_name[short_len] = entry[j];
                        short_len += 1;
                    }
                }
            }

            if name_matches(&short_name[..short_len], filename) {
                let start_cluster = u32::from_le_bytes([entry[26], entry[27], entry[20], entry[21]]);
                let size = u32::from_le_bytes([entry[28], entry[29], entry[30], entry[31]]);
                return Some(FileInfo { start_cluster, size });
            }

            // Reset LFN for next entry
            lfn.reset();
        }
    }
    None
}

/// Read file contents into buffer, following cluster chain
/// Returns number of bytes read, or None on error
fn read_file(
    fs: &FatFilesystem,
    block: &mut BlockClient,
    file: &FileInfo,
    buffer: &mut [u8],
) -> Option<usize> {
    if file.size == 0 {
        return Some(0);
    }

    let _bytes_per_cluster = fs.bytes_per_cluster() as usize;
    let sectors_per_cluster = fs.sectors_per_cluster_u32();
    let mut bytes_read = 0usize;
    let mut current_cluster = file.start_cluster;
    let max_bytes = core::cmp::min(buffer.len(), file.size as usize);

    let mut fat_sector_buf = [0u8; 512];
    let mut cached_fat_sector: Option<u32> = None;

    while bytes_read < max_bytes && current_cluster >= 2 {
        // Read this cluster's data
        let cluster_start = fs.cluster_to_sector(current_cluster);

        for sector_offset in 0..sectors_per_cluster {
            if bytes_read >= max_bytes {
                break;
            }

            let sector = cluster_start + sector_offset;

            // Retry logic for transient USB errors
            let mut read_ok = false;
            for retry in 0..5 {
                if block.read_sector(sector as u64).is_some() {
                    read_ok = true;
                    break;
                }
                if retry < 4 {
                    uwarn!("fatfs", "read_retry"; sector = sector as u64, attempt = (retry + 1) as u64);
                    // Brief delay before retry
                    for _ in 0..100 {
                        syscall::yield_now();
                    }
                }
            }

            if !read_ok {
                uerror!("fatfs", "read_error"; sector = sector as u64);
                return None;
            }

            let data = block.data();
            let bytes_to_copy = core::cmp::min(512, max_bytes - bytes_read);
            buffer[bytes_read..bytes_read + bytes_to_copy].copy_from_slice(&data[..bytes_to_copy]);
            bytes_read += bytes_to_copy;
        }

        // Get next cluster from FAT
        let (fat_sector, offset) = fs.fat_sector_for_cluster(current_cluster);

        // Cache FAT sector reads (with retry logic)
        if cached_fat_sector != Some(fat_sector) {
            let mut fat_read_ok = false;
            for retry in 0..5 {
                if block.read_sector(fat_sector as u64).is_some() {
                    fat_read_ok = true;
                    break;
                }
                if retry < 4 {
                    uwarn!("fatfs", "read_retry"; sector = fat_sector as u64, attempt = (retry + 1) as u64);
                    for _ in 0..100 {
                        syscall::yield_now();
                    }
                }
            }
            if !fat_read_ok {
                uerror!("fatfs", "read_error"; sector = fat_sector as u64);
                return None;
            }
            fat_sector_buf.copy_from_slice(&block.data()[..512]);
            cached_fat_sector = Some(fat_sector);
        }

        match fs.next_cluster_from_sector(&fat_sector_buf, current_cluster, offset) {
            Some(next) => current_cluster = next,
            None => break, // End of chain
        }
    }

    Some(bytes_read)
}

