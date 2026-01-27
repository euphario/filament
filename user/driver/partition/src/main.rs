//! Partition Driver
//!
//! Demonstrates the composable driver stack. Spawned by devd when a Block
//! device appears. Reads the partition table and registers partition ports.
//!
//! ## Flow
//!
//! 1. devd spawns this driver when a Block port is registered
//! 2. Driver discovers block devices via devd-query:
//! 3. For each block device, reads MBR from sector 0
//! 4. Registers partition ports (part0:, part1:, etc.) with parent=disk
//! 5. fatfs driver gets spawned for each partition
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  fatfs (spawned per partition)          │
//! │  connects to: part0:, part1:, ...       │
//! ├─────────────────────────────────────────┤
//! │  partition (this driver)                │
//! │  provides: part0:, part1:, ...          │
//! │  consumes: disk0: (or msc0:, nvme0:)    │
//! ├─────────────────────────────────────────┤
//! │  usbd / nvmed (block device)            │
//! │  provides: disk0: (Block type)          │
//! └─────────────────────────────────────────┘
//! ```

#![no_std]
#![no_main]

use userlib::syscall::{self, LogLevel};
use userlib::ipc::Channel;
use userlib::query::{
    QueryHeader, PortRegister, PortRegisterResponse,
    ListDevices, DeviceListResponse,
    msg, port_type, class, error,
};
use userlib::error::SysError;

// =============================================================================
// Logging
// =============================================================================

macro_rules! plog {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        const PREFIX: &[u8] = b"[partition] ";
        let mut buf = [0u8; 128];
        buf[..PREFIX.len()].copy_from_slice(PREFIX);
        let mut pos = PREFIX.len();
        struct W<'a> { b: &'a mut [u8], p: &'a mut usize }
        impl core::fmt::Write for W<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                for &b in s.as_bytes() {
                    if *self.p < self.b.len() { self.b[*self.p] = b; *self.p += 1; }
                }
                Ok(())
            }
        }
        let _ = write!(W { b: &mut buf, p: &mut pos }, $($arg)*);
        syscall::klog(LogLevel::Info, &buf[..pos]);
    }};
}

macro_rules! perror {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        const PREFIX: &[u8] = b"[partition] ";
        let mut buf = [0u8; 128];
        buf[..PREFIX.len()].copy_from_slice(PREFIX);
        let mut pos = PREFIX.len();
        struct W<'a> { b: &'a mut [u8], p: &'a mut usize }
        impl core::fmt::Write for W<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                for &b in s.as_bytes() {
                    if *self.p < self.b.len() { self.b[*self.p] = b; *self.p += 1; }
                }
                Ok(())
            }
        }
        let _ = write!(W { b: &mut buf, p: &mut pos }, $($arg)*);
        syscall::klog(LogLevel::Error, &buf[..pos]);
    }};
}

// =============================================================================
// MBR Structures (used when DataPort is available to read from disk)
// =============================================================================

/// MBR partition entry (16 bytes)
#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
struct MbrEntry {
    bootable: u8,
    partition_type: u8,
    start_lba: u32,
    sector_count: u32,
}

#[allow(dead_code)]
impl MbrEntry {
    fn from_bytes(data: &[u8]) -> Self {
        Self {
            bootable: data[0],
            partition_type: data[4],
            start_lba: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
            sector_count: u32::from_le_bytes([data[12], data[13], data[14], data[15]]),
        }
    }

    fn is_valid(&self) -> bool {
        self.partition_type != 0 && self.sector_count > 0
    }

    fn is_fat(&self) -> bool {
        matches!(self.partition_type,
            0x01 | 0x04 | 0x06 | 0x0B | 0x0C | 0x0E |
            0x14 | 0x16 | 0x1B | 0x1C | 0x1E)
    }
}

#[allow(dead_code)]
fn partition_type_name(ptype: u8) -> &'static str {
    match ptype {
        0x00 => "Empty",
        0x01 => "FAT12",
        0x04 | 0x06 | 0x0E => "FAT16",
        0x05 | 0x0F => "Extended",
        0x07 => "NTFS/exFAT",
        0x0B | 0x0C => "FAT32",
        0x82 => "Linux swap",
        0x83 => "Linux",
        0xEE => "GPT protective",
        0xEF => "EFI System",
        _ => "Unknown",
    }
}

/// Parsed MBR
#[allow(dead_code)]
struct Mbr {
    entries: [MbrEntry; 4],
    valid: bool,
}

#[allow(dead_code)]
impl Mbr {
    fn parse(sector: &[u8]) -> Self {
        let valid = sector.len() >= 512 && sector[510] == 0x55 && sector[511] == 0xAA;

        let mut entries = [MbrEntry {
            bootable: 0,
            partition_type: 0,
            start_lba: 0,
            sector_count: 0,
        }; 4];

        if valid {
            for i in 0..4 {
                let offset = 446 + i * 16;
                entries[i] = MbrEntry::from_bytes(&sector[offset..offset + 16]);
            }
        }

        Self { entries, valid }
    }
}

// =============================================================================
// Partition Driver
// =============================================================================

struct PartitionDriver {
    /// Connection to devd-query:
    devd_channel: Option<Channel>,
    /// Sequence ID for messages
    next_seq: u32,
    /// Discovered partitions
    partitions: [Option<PartitionInfo>; 16],
    partition_count: usize,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
struct PartitionInfo {
    /// Parent disk port name
    disk_name: [u8; 32],
    disk_name_len: u8,
    /// Partition index on disk
    index: u8,
    /// Partition type
    partition_type: u8,
    /// Start LBA
    start_lba: u32,
    /// Sector count
    sector_count: u32,
}

impl PartitionDriver {
    const fn new() -> Self {
        Self {
            devd_channel: None,
            next_seq: 1,
            partitions: [const { None }; 16],
            partition_count: 0,
        }
    }

    fn init(&mut self) -> Result<(), SysError> {
        plog!("connecting to devd-query:");

        // Connect to devd
        let channel = Channel::connect(b"devd-query:")?;
        self.devd_channel = Some(channel);

        plog!("connected to devd");
        Ok(())
    }

    fn next_seq(&mut self) -> u32 {
        let seq = self.next_seq;
        self.next_seq += 1;
        seq
    }

    /// Discover block devices and scan their partitions
    fn discover_and_scan(&mut self) -> Result<(), SysError> {
        plog!("discovering block devices");

        // Query devd for mass storage devices
        let devices = self.list_block_devices()?;

        plog!("found {} block device(s)", devices);

        // For each device, try to read MBR
        // For now, we'll use a hardcoded disk name since we don't have
        // a way to get the port name from devd yet
        // TODO: Add device-to-port mapping in devd

        // Try common disk names
        for disk_name in &[b"disk0:" as &[u8], b"msc0:", b"nvme0:"] {
            plog!("trying disk: {}", core::str::from_utf8(disk_name).unwrap_or("?"));

            // For now, we can't actually read the disk without DataPort
            // Just register example partitions to demonstrate the flow
            if devices > 0 {
                self.register_example_partitions(disk_name)?;
                break;
            }
        }

        Ok(())
    }

    /// List block devices via devd-query
    fn list_block_devices(&mut self) -> Result<usize, SysError> {
        // Get seq first, then channel (avoid double mutable borrow)
        let seq = self.next_seq();

        let channel = self.devd_channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        // Send LIST_DEVICES request
        let req = ListDevices::new(seq, Some(class::MASS_STORAGE));
        let _ = channel.send(&req.to_bytes())?;

        // Read response
        let mut buf = [0u8; 512];
        let len = channel.recv(&mut buf)?;

        if len < DeviceListResponse::HEADER_SIZE {
            return Ok(0);
        }

        let header = QueryHeader::from_bytes(&buf).ok_or(SysError::IoError)?;
        if header.msg_type == msg::ERROR {
            return Ok(0);
        }

        let resp = DeviceListResponse::from_bytes(&buf).ok_or(SysError::IoError)?;
        Ok(resp.count as usize)
    }

    /// Register example partitions (for demonstration)
    ///
    /// In a real implementation, this would read the MBR from the disk.
    /// For now, we just register placeholder partitions.
    fn register_example_partitions(&mut self, disk_name: &[u8]) -> Result<(), SysError> {
        // Create partition names
        let part_names: [&[u8]; 2] = [b"part0:", b"part1:"];

        for (i, part_name) in part_names.iter().enumerate() {
            plog!("registering {} (parent={})",
                core::str::from_utf8(part_name).unwrap_or("?"),
                core::str::from_utf8(disk_name).unwrap_or("?"));

            // Register with devd
            let result = self.register_port(
                part_name,
                Some(disk_name),
                port_type::PARTITION,
            );

            match result {
                Ok(()) => {
                    plog!("registered {}", core::str::from_utf8(part_name).unwrap_or("?"));

                    // Track partition
                    if self.partition_count < 16 {
                        let mut info = PartitionInfo {
                            disk_name: [0; 32],
                            disk_name_len: disk_name.len().min(32) as u8,
                            index: i as u8,
                            partition_type: 0x0B, // FAT32
                            start_lba: (i as u32 + 1) * 2048,
                            sector_count: 1024 * 1024, // 512MB
                        };
                        info.disk_name[..info.disk_name_len as usize]
                            .copy_from_slice(&disk_name[..info.disk_name_len as usize]);
                        self.partitions[self.partition_count] = Some(info);
                        self.partition_count += 1;
                    }
                }
                Err(e) => {
                    perror!("failed to register {}: {:?}",
                        core::str::from_utf8(part_name).unwrap_or("?"), e);
                }
            }
        }

        Ok(())
    }

    /// Register a port with devd
    fn register_port(
        &mut self,
        name: &[u8],
        parent: Option<&[u8]>,
        ptype: u8,
    ) -> Result<(), SysError> {
        // Get seq first (avoid double mutable borrow)
        let seq = self.next_seq();

        let channel = self.devd_channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        // Build REGISTER_PORT message
        let reg = PortRegister::new(seq, ptype);

        let mut buf = [0u8; 128];
        let len = reg.write_to(&mut buf, name, parent, 0)
            .ok_or(SysError::InvalidArgument)?;

        // Send request
        channel.send(&buf[..len])?;

        // Read response
        let mut resp_buf = [0u8; 64];
        let resp_len = channel.recv(&mut resp_buf)?;

        if resp_len < PortRegisterResponse::SIZE {
            return Err(SysError::IoError);
        }

        // Check result
        let header = QueryHeader::from_bytes(&resp_buf).ok_or(SysError::IoError)?;
        if header.msg_type == msg::ERROR {
            return Err(SysError::PermissionDenied);
        }

        // Parse result code
        let result = i32::from_le_bytes([
            resp_buf[8], resp_buf[9], resp_buf[10], resp_buf[11]
        ]);

        if result != error::OK {
            return Err(SysError::InvalidArgument);
        }

        Ok(())
    }

    /// Main event loop
    fn run(&mut self) -> ! {
        plog!("partition driver running");

        // Discover and scan partitions
        if let Err(e) = self.discover_and_scan() {
            perror!("discovery failed: {:?}", e);
        }

        plog!("registered {} partition(s)", self.partition_count);

        // For now, just sleep forever
        // In a real implementation, we'd handle I/O forwarding here
        loop {
            syscall::sleep_ms(10000);
        }
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: PartitionDriver = PartitionDriver::new();

#[unsafe(no_mangle)]
fn main() {
    syscall::klog(LogLevel::Info, b"[partition] starting");

    // SAFETY: Single-threaded userspace driver, no concurrent access
    let driver = unsafe { &mut *(&raw mut DRIVER) };

    if let Err(e) = driver.init() {
        perror!("init failed: {:?}", e);
        syscall::exit(1);
    }

    driver.run()
}
