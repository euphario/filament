//! FAT Filesystem Driver
//!
//! Userspace FAT12/16/32 filesystem driver that reads from USB mass storage
//! via ring buffers to usbd.

#![no_std]
#![no_main]

use userlib::{println, print, syscall};
use userlib::firmware::{FirmwareRequest, FirmwareReply, FATFS_PORT, error as fw_error};

mod fat;

use fat::{FatFilesystem, FatType, LfnCollector};
// Use USB library's BlockClient for ring buffer access
use usb::BlockClient;

// Entry point
#[unsafe(no_mangle)]
fn main() {
    println!("[fatfs] FAT filesystem driver starting...");

    // Connect to usbd via "usb" port using ring buffer protocol
    println!("[fatfs] Connecting to usbd...");
    let mut block = match BlockClient::connect(b"usb") {
        Some(b) => b,
        None => {
            println!("[fatfs] ERROR: Failed to connect to usbd");
            syscall::exit(1);
        }
    };
    println!("[fatfs] Connected via ring buffer (data buffer: {} bytes)", block.buffer_size());

    // Debug: Test buffer mapping by touching pages
    {
        let data = block.data();
        let ptr = data.as_ptr();
        println!("[fatfs] Data buffer at 0x{:x}, testing mapping...", ptr as u64);

        // Touch every 4KB page to verify mapping
        let page_size = 4096usize;
        let num_pages = (block.buffer_size() + page_size - 1) / page_size;
        for i in 0..num_pages {
            let offset = i * page_size;
            if offset < data.len() {
                // Read one byte from each page
                let _byte = unsafe { core::ptr::read_volatile(ptr.add(offset)) };
                if i < 5 || i == num_pages - 1 {
                    println!("[fatfs]   Page {} (offset 0x{:x}): OK", i, offset);
                } else if i == 5 {
                    println!("[fatfs]   ... testing remaining pages ...");
                }
            }
        }
        println!("[fatfs] All {} pages accessible!", num_pages);
    }

    // Get block device info
    // NOTE: port_connect() blocks until usbd has MSC ready, so this should succeed immediately
    let (block_size, block_count) = match block.get_info() {
        Some(info) => info,
        None => {
            println!("[fatfs] ERROR: Failed to get block device info");
            syscall::exit(1);
        }
    };
    println!("[fatfs] Block device: {} blocks x {} bytes", block_count, block_size);

    // Read sector 0 (could be MBR or FAT boot sector)
    let bs = block_size as usize;
    if bs > 4096 {
        println!("[fatfs] ERROR: Block size {} too large", bs);
        syscall::exit(1);
    }

    let mut sector0 = [0u8; 4096];
    if block.read_sector(0).is_none() {
        println!("[fatfs] ERROR: Failed to read sector 0");
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
    println!("[fatfs] Boot signature: {:02x} {:02x}", sig0, sig1);

    // Check if this is an MBR (partition table) or direct FAT boot sector
    // FAT boot sector starts with jump: EB xx 90 or E9 xx xx
    // MBR starts with executable code (often 33 C0 = xor ax,ax)
    let partition_start = if sector0[0] == 0xEB || sector0[0] == 0xE9 {
        // Direct FAT boot sector (no partition table)
        println!("[fatfs] Direct FAT boot sector (no MBR)");
        0u64
    } else {
        // Check MBR signature
        if sig0 != 0x55 || sig1 != 0xAA {
            println!("[fatfs] ERROR: Invalid MBR signature");
            syscall::exit(1);
        }

        // Parse partition table (starts at offset 0x1BE)
        println!("[fatfs] MBR detected, scanning partitions...");
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
            let size_sectors = u32::from_le_bytes([
                sector0[entry_offset + 12],
                sector0[entry_offset + 13],
                sector0[entry_offset + 14],
                sector0[entry_offset + 15],
            ]);

            if part_type != 0 {
                let type_name = match part_type {
                    0x01 => "FAT12",
                    0x04 | 0x06 | 0x0E => "FAT16",
                    0x0B | 0x0C => "FAT32",
                    0x07 => "NTFS/exFAT",
                    0x83 => "Linux",
                    0xEE => "GPT",
                    _ => "Unknown",
                };
                println!("[fatfs]   Partition {}: type=0x{:02x} ({}) LBA={} size={}",
                    i, part_type, type_name, lba_start, size_sectors);

                // Use first FAT partition found
                if found_lba == 0 && (part_type == 0x01 || part_type == 0x04 ||
                    part_type == 0x06 || part_type == 0x0B || part_type == 0x0C || part_type == 0x0E) {
                    found_lba = lba_start as u64;
                }
            }
        }

        if found_lba == 0 {
            println!("[fatfs] ERROR: No FAT partition found");
            syscall::exit(1);
        }

        println!("[fatfs] Using partition at LBA {}", found_lba);
        found_lba
    };

    // Read actual FAT boot sector
    let mut boot_sector = [0u8; 512];
    if partition_start == 0 {
        boot_sector.copy_from_slice(&sector0);
    } else {
        if block.read_sector(partition_start).is_none() {
            println!("[fatfs] ERROR: Failed to read FAT boot sector at LBA {}", partition_start);
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
            println!("[fatfs] ERROR: Failed to mount FAT filesystem");
            println!("[fatfs] Boot sector dump:");
            for i in 0..64 {
                if i % 16 == 0 { print!("  {:04x}: ", i); }
                print!("{:02x} ", boot_sector[i]);
                if i % 16 == 15 { println!(); }
            }
            syscall::exit(1);
        }
    };

    println!("[fatfs] Mounted {:?} filesystem", fs.fat_type);
    println!("[fatfs]   Bytes per sector: {}", fs.bytes_per_sector);
    println!("[fatfs]   Sectors per cluster: {}", fs.sectors_per_cluster);
    println!("[fatfs]   Total clusters: {}", fs.total_clusters);

    // List root directory
    println!("[fatfs] Root directory:");
    list_root_directory(&fs, &mut block);

    // Test file reading - try to find and read a small test file
    println!();
    println!("[fatfs] Testing file read...");

    // Try to find README.TXT or any .TXT file
    if let Some(file) = find_file(&fs, &mut block, b"README.TXT") {
        println!("[fatfs] Found README.TXT: cluster={}, size={}", file.start_cluster, file.size);

        // Read first 256 bytes
        let mut buf = [0u8; 256];
        let read_size = core::cmp::min(256, file.size as usize);
        if let Some(bytes_read) = read_file(&fs, &mut block, &file, &mut buf[..read_size]) {
            println!("[fatfs] Read {} bytes:", bytes_read);
            // Print as text if it looks like ASCII
            if let Ok(text) = core::str::from_utf8(&buf[..bytes_read]) {
                for line in text.lines().take(5) {
                    println!("  {}", line);
                }
            }
        }
    } else {
        println!("[fatfs] README.TXT not found (normal if not present)");
    }

    // Try to find MT7996 firmware files (now with LFN support!)
    let firmware_files = [
        b"mt7996_rom_patch.bin" as &[u8],
        b"mt7996_wm.bin",
        b"mt7996_wa.bin",
        b"mt7996_dsp.bin",
        b"mt7996_eeprom.bin",
    ];
    println!();
    println!("[fatfs] Searching for MT7996 firmware files:");
    for fw_name in firmware_files {
        let name = core::str::from_utf8(fw_name).unwrap_or("?");
        if let Some(file) = find_file(&fs, &mut block, fw_name) {
            println!("[fatfs]   Found {}: {} bytes", name, file.size);
        } else {
            println!("[fatfs]   {} not found", name);
        }
    }

    // Skip performance test for faster startup, go straight to service mode
    // read_performance_test(&mut block, block_count);

    // Register as firmware service
    println!();
    println!("[fatfs] Registering as firmware service...");
    let listen_channel = syscall::port_register(FATFS_PORT);
    if listen_channel < 0 {
        println!("[fatfs] ERROR: Failed to register port: {}", listen_channel);
        syscall::exit(1);
    }
    let listen_channel = listen_channel as u32;
    println!("[fatfs] Listening on port 'fatfs' (channel {})", listen_channel);

    // Daemonize - detach from parent so shell can continue
    let result = syscall::daemonize();
    if result != 0 {
        println!("[fatfs] WARNING: daemonize failed: {}", result);
    }

    // Service loop
    firmware_service_loop(&fs, &mut block, listen_channel);
}

/// Handle firmware requests from clients
fn firmware_service_loop(fs: &FatFilesystem, block: &mut BlockClient, listen_channel: u32) {
    let mut request_buf = [0u8; core::mem::size_of::<FirmwareRequest>()];
    let my_pid = syscall::getpid();

    loop {
        // Accept a connection
        let client_channel = syscall::port_accept(listen_channel);
        if client_channel < 0 {
            // No pending connections, yield and try again
            syscall::yield_now();
            continue;
        }
        let client_channel = client_channel as u32;

        // First message is handshake - receive ping, send our PID
        let mut ping_buf = [0u8; 4];
        let n = syscall::receive(client_channel, &mut ping_buf);
        if n < 0 {
            syscall::channel_close(client_channel);
            continue;
        }

        // Send our PID so client can shmem_allow us
        let pid_bytes = my_pid.to_le_bytes();
        if syscall::send(client_channel, &pid_bytes) < 0 {
            syscall::channel_close(client_channel);
            continue;
        }

        // Now receive actual firmware request
        let n = syscall::receive(client_channel, &mut request_buf);
        if n < 0 {
            println!("[fatfs] Receive error: {}", n);
            syscall::channel_close(client_channel);
            continue;
        }

        let request = match FirmwareRequest::from_bytes(&request_buf[..n as usize]) {
            Some(r) => r,
            None => {
                println!("[fatfs] Invalid request (size={})", n);
                let reply = FirmwareReply::error(fw_error::READ_ERROR);
                syscall::send(client_channel, reply.as_bytes());
                syscall::channel_close(client_channel);
                continue;
            }
        };

        let filename = request.filename_str();
        let filename_str = core::str::from_utf8(filename).unwrap_or("???");
        println!("[fatfs] Request: {} (shmem={}, max={})",
            filename_str, request.shmem_id, request.max_size);

        // Allow the requester to access the shmem (they created it, we need to map it)
        // Actually, the requester owns it, we need them to allow US
        // The protocol should have requester call shmem_allow(shmem_id, fatfs_pid)
        // For now, try to map it directly

        // Map the shared memory
        let mut vaddr: u64 = 0;
        let mut paddr: u64 = 0;
        let map_result = syscall::shmem_map(request.shmem_id, &mut vaddr, &mut paddr);
        if map_result < 0 {
            println!("[fatfs] Failed to map shmem {}: {}", request.shmem_id, map_result);
            let reply = FirmwareReply::error(fw_error::NO_MEMORY);
            syscall::send(client_channel, reply.as_bytes());
            syscall::channel_close(client_channel);
            continue;
        }

        // Find the file
        let file = match find_file(fs, block, filename) {
            Some(f) => f,
            None => {
                println!("[fatfs] File not found: {}", filename_str);
                let reply = FirmwareReply::error(fw_error::NOT_FOUND);
                syscall::send(client_channel, reply.as_bytes());
                syscall::channel_close(client_channel);
                continue;
            }
        };

        // Check size
        if file.size > request.max_size {
            println!("[fatfs] File too large: {} > {}", file.size, request.max_size);
            let reply = FirmwareReply::error(fw_error::TOO_LARGE);
            syscall::send(client_channel, reply.as_bytes());
            syscall::channel_close(client_channel);
            continue;
        }

        // Read file into shared memory
        let buffer = unsafe {
            core::slice::from_raw_parts_mut(vaddr as *mut u8, request.max_size as usize)
        };

        let bytes_read = match read_file(fs, block, &file, buffer) {
            Some(n) => n,
            None => {
                println!("[fatfs] Read error");
                let reply = FirmwareReply::error(fw_error::READ_ERROR);
                syscall::send(client_channel, reply.as_bytes());
                syscall::channel_close(client_channel);
                continue;
            }
        };

        println!("[fatfs] Loaded {} bytes", bytes_read);

        // Send success reply
        let reply = FirmwareReply::success(bytes_read);
        syscall::send(client_channel, reply.as_bytes());
        syscall::channel_close(client_channel);
    }
}

/// Read performance test using ring buffer / DMA
fn read_performance_test(block: &mut BlockClient, _block_count: u64) {
    // Get timer frequency
    let freq: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
    }

    let block_size = block.block_size();

    // Test 1: Synchronous reads (baseline)
    println!("[fatfs] === Synchronous Read Performance ===");
    let test_lba = 1000u64;
    let test_sizes = [1u32, 4, 8, 16, 32, 64];
    let iterations = 100u32;

    for &sectors in &test_sizes {
        let start: u64;
        unsafe {
            core::arch::asm!("mrs {}, cntpct_el0", out(reg) start);
        }

        let mut success = 0u32;
        for _ in 0..iterations {
            if block.read(test_lba, sectors).is_some() {
                success += 1;
            }
        }

        let end: u64;
        unsafe {
            core::arch::asm!("mrs {}, cntpct_el0", out(reg) end);
        }

        let elapsed_ticks = end - start;
        let elapsed_us = (elapsed_ticks * 1_000_000) / freq;
        let bytes_transferred = (success as u64) * (sectors as u64) * 512;

        if elapsed_us > 0 && success > 0 {
            let kb_per_sec = (bytes_transferred * 1_000_000) / (elapsed_us * 1024);
            println!("[fatfs]   Sync {}x{}: {}/{} OK, {} KB/s",
                sectors, iterations, success, iterations, kb_per_sec);
        }
    }

    // Test 2: Pipelined reads (submit batch, then collect)
    println!("[fatfs] === Pipelined Read Performance ===");
    let pipeline_depths = [4u32, 8, 16, 32];
    let sectors_per_read = 16u32;  // 8KB per read
    let bytes_per_read = (sectors_per_read * block_size) as usize;

    for &depth in &pipeline_depths {
        // Calculate how many reads fit in buffer
        let max_reads = block.buffer_size() / bytes_per_read;
        let actual_depth = depth.min(max_reads as u32);
        let total_reads = 100u32;

        let start: u64;
        unsafe {
            core::arch::asm!("mrs {}, cntpct_el0", out(reg) start);
        }

        let mut success = 0u32;
        let mut submitted = 0u32;
        let mut completed = 0u32;

        // Initial batch submission
        while submitted < actual_depth && submitted < total_reads {
            let buf_offset = (submitted as usize % actual_depth as usize) * bytes_per_read;
            let lba = test_lba + (submitted as u64 * sectors_per_read as u64);
            if block.submit_read(lba, sectors_per_read, buf_offset as u32).is_some() {
                submitted += 1;
            } else {
                break;
            }
        }
        block.notify();

        // Pipeline: as completions come in, submit more
        while completed < total_reads {
            // Wait for at least one completion
            block.wait_completions(1, 5000);

            // Collect all available completions
            while let Some((tag, bytes)) = block.collect_completion() {
                if bytes > 0 {
                    success += 1;
                }
                completed += 1;

                // Submit another if we have more to do
                if submitted < total_reads {
                    let buf_offset = (submitted as usize % actual_depth as usize) * bytes_per_read;
                    let lba = test_lba + (submitted as u64 * sectors_per_read as u64);
                    if block.submit_read(lba, sectors_per_read, buf_offset as u32).is_some() {
                        submitted += 1;
                        block.notify();
                    }
                }
            }
        }

        let end: u64;
        unsafe {
            core::arch::asm!("mrs {}, cntpct_el0", out(reg) end);
        }

        let elapsed_ticks = end - start;
        let elapsed_us = (elapsed_ticks * 1_000_000) / freq;
        let bytes_transferred = (success as u64) * bytes_per_read as u64;

        if elapsed_us > 0 && success > 0 {
            let kb_per_sec = (bytes_transferred * 1_000_000) / (elapsed_us * 1024);
            println!("[fatfs]   Pipeline depth={} ({}x{}): {}/{} OK, {} KB/s",
                actual_depth, sectors_per_read, total_reads, success, total_reads, kb_per_sec);
        }
    }

    // Verify we can read actual data
    println!("[fatfs] Verifying read data...");
    if let Some(bytes) = block.read(0, 1) {
        // Check for MBR signature or FAT boot sector
        let data = block.data();
        let sig0 = data[510];
        let sig1 = data[511];
        if sig0 == 0x55 && sig1 == 0xAA {
            println!("[fatfs]   Read {} bytes, MBR/boot signature OK!", bytes);
        } else {
            println!("[fatfs]   Read {} bytes (sig: {:02x} {:02x})", bytes, sig0, sig1);
        }
    } else {
        println!("[fatfs]   Read failed!");
    }
}

fn list_root_directory(fs: &FatFilesystem, block: &mut BlockClient) {
    let mut sector_buf = [0u8; 512];
    let mut lfn = LfnCollector::new();

    let root_start_sector = if fs.fat_type == FatType::Fat32 {
        fs.cluster_to_sector(fs.root_cluster)
    } else {
        fs.root_dir_absolute_sector()
    };

    println!("[fatfs] Reading root directory at sector {}", root_start_sector);

    // Read multiple sectors to show more entries
    for sector_offset in 0..4u32 {
        if block.read_sector((root_start_sector + sector_offset) as u64).is_none() {
            if sector_offset == 0 {
                println!("  (failed to read root directory)");
            }
            return;
        }
        sector_buf.copy_from_slice(&block.data()[..512]);

        for i in 0..16 {
            let entry = &sector_buf[i * 32..(i + 1) * 32];

            if entry[0] == 0 {
                return; // End of directory
            }
            if entry[0] == 0xE5 {
                lfn.reset();
                continue;
            }

            // LFN entry - collect it
            if entry[11] == 0x0F {
                lfn.add_lfn_entry(entry);
                continue;
            }

            // Regular entry - use LFN if available, else 8.3
            let display_name: &[u8];
            let mut short_name = [0u8; 12];
            let mut short_len = 0;

            // Build 8.3 name
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

            // Use LFN if complete, otherwise use short name
            if lfn.is_complete() {
                display_name = lfn.name_slice();
            } else {
                display_name = &short_name[..short_len];
            }

            let attr = entry[11];
            let is_dir = (attr & 0x10) != 0;
            let size = u32::from_le_bytes([entry[28], entry[29], entry[30], entry[31]]);

            let name_str = core::str::from_utf8(display_name).unwrap_or("???");
            if is_dir {
                println!("  <DIR> {}", name_str);
            } else {
                println!("  {:>8} {}", size, name_str);
            }

            lfn.reset();
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

    let bytes_per_cluster = fs.bytes_per_cluster() as usize;
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
            if block.read_sector(sector as u64).is_none() {
                println!("[fatfs] Read error at sector {}", sector);
                return None;
            }

            let data = block.data();
            let bytes_to_copy = core::cmp::min(512, max_bytes - bytes_read);
            buffer[bytes_read..bytes_read + bytes_to_copy].copy_from_slice(&data[..bytes_to_copy]);
            bytes_read += bytes_to_copy;
        }

        // Get next cluster from FAT
        let (fat_sector, offset) = fs.fat_sector_for_cluster(current_cluster);

        // Cache FAT sector reads
        if cached_fat_sector != Some(fat_sector) {
            if block.read_sector(fat_sector as u64).is_none() {
                println!("[fatfs] FAT read error at sector {}", fat_sector);
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

