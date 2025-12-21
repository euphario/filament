//! FAT Filesystem Driver
//!
//! Userspace FAT12/16/32 filesystem driver that reads from USB mass storage
//! via ring buffers to usbd.

#![no_std]
#![no_main]

use userlib::{println, print, syscall};

mod fat;

use fat::{FatFilesystem, FatType};
// Use USB library's BlockClient for ring buffer access
use usb::{BlockClient, delay_ms};

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

    // Get block device info (retry if device not ready yet)
    println!("[fatfs] Waiting for block device...");
    let mut retries = 0;
    let (block_size, block_count) = loop {
        if let Some(info) = block.get_info() {
            break info;
        }
        retries += 1;
        if retries >= 30 {
            println!("[fatfs] ERROR: Timeout waiting for block device (30 retries)");
            syscall::exit(1);
        }
        // Wait 500ms between retries (30 retries = 15 seconds total timeout)
        delay_ms(500);
        syscall::yield_now();
    };
    println!("[fatfs] Device ready after {} retries", retries);
    println!("[fatfs] Block device: {} blocks x {} bytes", block_count, block_size);

    // Read sector 0 (could be MBR or FAT boot sector)
    // Support up to 4096-byte logical sectors
    let bs = block_size as usize;
    if bs > 4096 {
        println!("[fatfs] ERROR: Block size {} too large (max 4096)", bs);
        syscall::exit(1);
    }
    let mut sector0 = [0u8; 4096];
    if block.read_sector(0).is_none() {
        println!("[fatfs] ERROR: Failed to read sector 0");
        syscall::exit(1);
    }
    sector0[..bs].copy_from_slice(&block.data()[..bs]);

    // Debug: dump first 16 bytes of sector 0
    print!("[fatfs] Sector 0 first 16 bytes: ");
    for i in 0..16 {
        print!("{:02x} ", sector0[i]);
    }
    println!();

    // Signature location depends on logical sector size
    // For MBR, signature is at end of first 512 bytes (offsets 510/511)
    // For 4K native, signature could be at bs-2, but MBR is still in first 512 bytes
    // Try end of logical sector first, then fall back to 510/511
    let (sig0, sig1) = if bs >= 512 {
        let end_sig = (sector0[bs - 2], sector0[bs - 1]);
        let std_sig = (sector0[510], sector0[511]);
        if end_sig == (0x55, 0xAA) {
            println!("[fatfs] Signature at sector end (offset {})", bs - 2);
            end_sig
        } else if std_sig == (0x55, 0xAA) {
            println!("[fatfs] Signature at standard offset 510");
            std_sig
        } else {
            println!("[fatfs] Checking both sig locations: end={:02x}{:02x}, 510={:02x}{:02x}",
                     end_sig.0, end_sig.1, std_sig.0, std_sig.1);
            // Default to standard location
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

    // Read performance test (now uses ring buffer / DMA)
    println!("[fatfs] Running read performance test...");
    read_performance_test(&mut block, block_count);

    println!("[fatfs] Done.");
    syscall::exit(0);
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

    // For FAT12/16, root directory is at a fixed location
    // For FAT32, it starts at root_cluster
    // All methods now include partition_start offset
    let root_start_sector = if fs.fat_type == FatType::Fat32 {
        fs.cluster_to_sector(fs.root_cluster)
    } else {
        fs.root_dir_absolute_sector()
    };

    println!("[fatfs] Reading root directory at sector {}", root_start_sector);

    // Read first sector of root directory
    if block.read_sector(root_start_sector as u64).is_none() {
        println!("  (failed to read root directory)");
        return;
    }
    sector_buf.copy_from_slice(&block.data()[..512]);

    // Parse directory entries (32 bytes each)
    for i in 0..16 {
        let entry = &sector_buf[i * 32..(i + 1) * 32];
        if entry[0] == 0 {
            break; // End of directory
        }
        if entry[0] == 0xE5 {
            continue; // Deleted entry
        }
        if entry[11] == 0x0F {
            continue; // Long filename entry (skip for now)
        }

        // Parse 8.3 filename
        let mut name = [0u8; 12];
        let mut name_len = 0;

        // Copy name (8 chars)
        for j in 0..8 {
            if entry[j] != 0x20 {
                name[name_len] = entry[j];
                name_len += 1;
            }
        }

        // Add dot and extension if present
        if entry[8] != 0x20 {
            name[name_len] = b'.';
            name_len += 1;
            for j in 8..11 {
                if entry[j] != 0x20 {
                    name[name_len] = entry[j];
                    name_len += 1;
                }
            }
        }

        let attr = entry[11];
        let is_dir = (attr & 0x10) != 0;
        let size = u32::from_le_bytes([entry[28], entry[29], entry[30], entry[31]]);

        let name_str = core::str::from_utf8(&name[..name_len]).unwrap_or("???");
        if is_dir {
            println!("  <DIR> {}", name_str);
        } else {
            println!("  {:>8} {}", size, name_str);
        }
    }
}

