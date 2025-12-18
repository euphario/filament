//! FAT Filesystem Driver
//!
//! Userspace FAT12/16/32 filesystem driver that reads from USB mass storage
//! via IPC to usbd.

#![no_std]
#![no_main]

use userlib::{println, print, syscall};

mod fat;
mod block;

use fat::{FatFilesystem, FatType};
use block::BlockDevice;

// Entry point
#[unsafe(no_mangle)]
fn main() {
    println!("[fatfs] FAT filesystem driver starting...");

    // Connect to usbd via "usb" port
    let usb_channel = syscall::port_connect(b"usb");
    if usb_channel < 0 {
        println!("[fatfs] ERROR: Failed to connect to usb port: {}", usb_channel);
        syscall::exit(1);
    }
    let usb_channel = usb_channel as u32;
    println!("[fatfs] Connected to usbd on channel {}", usb_channel);

    // Create block device client
    let mut block = BlockDevice::new(usb_channel);

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
        // Small delay - yield CPU and wait
        for _ in 0..100000 {
            core::hint::spin_loop();
        }
        syscall::yield_now();
    };
    println!("[fatfs] Device ready after {} retries", retries);
    println!("[fatfs] Block device: {} blocks x {} bytes", block_count, block_size);

    // Read sector 0 (could be MBR or FAT boot sector)
    let mut sector0 = [0u8; 512];
    if !block.read_sector(0, &mut sector0) {
        println!("[fatfs] ERROR: Failed to read sector 0");
        syscall::exit(1);
    }

    // Check if this is an MBR (partition table) or direct FAT boot sector
    // FAT boot sector starts with jump: EB xx 90 or E9 xx xx
    // MBR starts with executable code (often 33 C0 = xor ax,ax)
    let partition_start = if sector0[0] == 0xEB || sector0[0] == 0xE9 {
        // Direct FAT boot sector (no partition table)
        println!("[fatfs] Direct FAT boot sector (no MBR)");
        0u64
    } else {
        // Check MBR signature
        if sector0[510] != 0x55 || sector0[511] != 0xAA {
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
        if !block.read_sector(partition_start, &mut boot_sector) {
            println!("[fatfs] ERROR: Failed to read FAT boot sector at LBA {}", partition_start);
            syscall::exit(1);
        }
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

    // Write performance test
    println!("[fatfs] Running write performance test...");
    write_performance_test(&mut block, block_count);

    println!("[fatfs] Done.");
    syscall::exit(0);
}

/// Simple write performance test
/// Writes sectors to the end of the device where there's likely free space
fn write_performance_test(block: &mut BlockDevice, block_count: u64) {
    // Use sectors near the end of the device (last 2000 sectors)
    // This avoids corrupting filesystem data in most cases
    let test_lba = block_count.saturating_sub(2000);
    if test_lba == 0 {
        println!("[fatfs] Device too small for write test");
        return;
    }

    // Create test pattern
    let mut test_sector = [0u8; 512];
    for i in 0..512 {
        test_sector[i] = (i & 0xFF) as u8;
    }

    // Get timer frequency for timing
    let freq: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
    }

    // Write test: 1000 sectors (512KB) sequentially
    let write_count = 1000u32;
    println!("[fatfs] Writing {} sectors ({} KB) starting at LBA {}...",
        write_count, write_count / 2, test_lba);

    let start: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) start);
    }

    let mut success_count = 0u32;
    for i in 0..write_count {
        // Modify test pattern slightly for each sector
        test_sector[0] = (i & 0xFF) as u8;
        test_sector[1] = ((i >> 8) & 0xFF) as u8;
        test_sector[510] = (i & 0xFF) as u8;
        test_sector[511] = ((i >> 8) & 0xFF) as u8;

        if block.write_sector(test_lba + i as u64, &test_sector) {
            success_count += 1;
        } else {
            println!("[fatfs]   Write failed at sector {}", i);
            break;
        }
    }

    let end: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) end);
    }

    let elapsed_ticks = end - start;
    let elapsed_us = (elapsed_ticks * 1_000_000) / freq;
    let bytes_written = (success_count as u64) * 512;

    println!("[fatfs] Write test complete:");
    println!("[fatfs]   Sectors written: {}/{}", success_count, write_count);
    println!("[fatfs]   Time: {} ms", elapsed_us / 1000);
    if elapsed_us > 0 {
        let bytes_per_sec = (bytes_written * 1_000_000) / elapsed_us;
        println!("[fatfs]   Throughput: {} KB/s ({} MB/s)",
            bytes_per_sec / 1024, bytes_per_sec / (1024 * 1024));
    }

    // Verify a sample of the written data (every 100th sector)
    println!("[fatfs] Verifying sample of written data...");
    let mut verify_sector = [0u8; 512];
    let mut verify_ok = true;
    let mut verified_count = 0u32;

    for i in (0..success_count).step_by(100) {
        if !block.read_sector(test_lba + i as u64, &mut verify_sector) {
            println!("[fatfs]   Read failed at sector {}", i);
            verify_ok = false;
            break;
        }

        // Check expected pattern
        let expected_lo = (i & 0xFF) as u8;
        let expected_hi = ((i >> 8) & 0xFF) as u8;
        if verify_sector[0] != expected_lo || verify_sector[1] != expected_hi ||
           verify_sector[510] != expected_lo || verify_sector[511] != expected_hi {
            println!("[fatfs]   Verify mismatch at sector {}", i);
            verify_ok = false;
        }
        verified_count += 1;
    }

    if verify_ok && verified_count > 0 {
        println!("[fatfs] Verified {} samples - all passed!", verified_count);
    }
}

fn list_root_directory(fs: &FatFilesystem, block: &mut BlockDevice) {
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
    if !block.read_sector(root_start_sector as u64, &mut sector_buf) {
        println!("  (failed to read root directory)");
        return;
    }

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
