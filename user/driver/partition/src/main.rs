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
use userlib::devd::{DevdClient, PortType, DriverState, SpawnHandler, SpawnFilter, run_driver_loop};
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
    /// Connection to devd
    devd_client: Option<DevdClient>,
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
            devd_client: None,
            partitions: [const { None }; 16],
            partition_count: 0,
        }
    }

    fn init(&mut self) -> Result<(), SysError> {
        plog!("connecting to devd-query:");

        // Connect to devd
        let client = DevdClient::connect()?;
        self.devd_client = Some(client);

        plog!("connected to devd");
        Ok(())
    }

    /// Discover block devices and scan their partitions
    fn discover_and_scan(&mut self) -> Result<(), SysError> {
        plog!("discovering block devices");

        // Query devd for mass storage devices
        // Retry a few times in case driver is still registering
        let mut devices = 0;
        for retry in 0..10 {
            devices = self.list_block_devices()?;
            if devices > 0 {
                break;
            }
            if retry < 9 {
                syscall::sleep_ms(50);
            }
        }

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

    /// List block devices via devd
    fn list_block_devices(&mut self) -> Result<usize, SysError> {
        let client = self.devd_client.as_mut().ok_or(SysError::ConnectionRefused)?;
        client.list_devices(Some(userlib::devd::DeviceClass::MassStorage))
    }

    /// Register example partitions (for demonstration)
    ///
    /// In a real implementation, this would read the MBR from the disk.
    /// For now, we just register placeholder partitions.
    fn register_example_partitions(&mut self, disk_name: &[u8]) -> Result<(), SysError> {
        let client = self.devd_client.as_mut().ok_or(SysError::ConnectionRefused)?;

        // Create partition names
        let part_names: [&[u8]; 2] = [b"part0:", b"part1:"];

        for (i, part_name) in part_names.iter().enumerate() {
            plog!("registering {} (parent={})",
                core::str::from_utf8(part_name).unwrap_or("?"),
                core::str::from_utf8(disk_name).unwrap_or("?"));

            // Register partition port with devd
            match client.register_port(part_name, PortType::Partition, Some(disk_name)) {
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

    /// Main event loop
    fn run(&mut self) -> ! {
        plog!("partition driver running");

        // Discover and scan partitions
        if let Err(e) = self.discover_and_scan() {
            perror!("discovery failed: {:?}", e);
        }

        plog!("registered {} partition(s)", self.partition_count);

        // Take the devd client for the service loop
        if let Some(mut client) = self.devd_client.take() {
            // Report ready state
            let _ = client.report_state(DriverState::Ready);
            plog!("reported ready state, entering service loop");

            // Use the generic driver service loop (event-driven, no polling)
            run_driver_loop(client, PartitionSpawnHandler);
        } else {
            perror!("no devd client, exiting");
            syscall::exit(1);
        }
    }
}

/// Spawn handler for partition driver
///
/// Spawns fatfs (or other filesystem drivers) for partitions.
struct PartitionSpawnHandler;

impl SpawnHandler for PartitionSpawnHandler {
    fn handle_spawn(&mut self, binary: &str, filter: &SpawnFilter) -> (u8, [u32; 16]) {
        plog!("received SPAWN_CHILD: binary={}", binary);

        // Log filter info
        match filter.mode {
            0 => plog!("  filter: EXACT pattern={}",
                core::str::from_utf8(filter.pattern_bytes()).unwrap_or("?")),
            1 => plog!("  filter: BY_TYPE port_type={}", filter.port_type),
            2 => plog!("  filter: CHILDREN_OF parent={}",
                core::str::from_utf8(filter.pattern_bytes()).unwrap_or("?")),
            3 => plog!("  filter: ALL"),
            _ => plog!("  filter: unknown"),
        }

        // Spawn the child
        let mut pids = [0u32; 16];
        let pid = syscall::exec(binary);
        if pid > 0 {
            plog!("  spawned child PID={}", pid);
            pids[0] = pid as u32;
            (1, pids)
        } else {
            perror!("  failed to spawn: error={}", -pid);
            (0, pids)
        }
    }

    fn handle_stop(&mut self, pid: u32) {
        plog!("received STOP_CHILD: PID={}", pid);
        let result = syscall::kill(pid);
        if result == 0 {
            plog!("  child killed successfully");
        } else {
            perror!("  kill failed: error={}", -result);
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
