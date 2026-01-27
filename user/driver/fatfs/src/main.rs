//! FAT Filesystem Driver
//!
//! Userspace FAT12/16/32 filesystem driver that reads from block storage.
//! Spawned by devd when a Partition port is registered.
//!
//! ## Flow
//!
//! 1. partition driver registers partX: ports
//! 2. devd spawns fatfs for each partition
//! 3. fatfs probes partition for FAT signature
//! 4. If FAT found, registers fsX: port for VFS access
//!
//! ## Current Status
//!
//! This is a stub that integrates with the new devd bidirectional protocol.
//! Full FAT implementation pending.

#![no_std]
#![no_main]

use userlib::syscall::{self, LogLevel};
use userlib::devd::{DevdClient, PortType, DriverState, SpawnHandler, SpawnFilter, run_driver_loop};
use userlib::error::SysError;

// =============================================================================
// Logging
// =============================================================================

macro_rules! flog {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        const PREFIX: &[u8] = b"[fatfs] ";
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

macro_rules! ferror {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        const PREFIX: &[u8] = b"[fatfs] ";
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
// FAT Driver
// =============================================================================

struct FatfsDriver {
    /// Connection to devd
    devd_client: Option<DevdClient>,
}

impl FatfsDriver {
    const fn new() -> Self {
        Self {
            devd_client: None,
        }
    }

    fn init(&mut self) -> Result<(), SysError> {
        flog!("connecting to devd");

        let client = DevdClient::connect()?;
        self.devd_client = Some(client);

        flog!("connected to devd");
        Ok(())
    }

    fn probe_partition(&mut self) -> Result<(), SysError> {
        // TODO: Connect to partition port and read boot sector
        // For now, just log that we're probing
        flog!("probing partition for FAT signature");

        // TODO: Read sector 0, check for FAT signature
        // If FAT found:
        //   - Parse BPB (BIOS Parameter Block)
        //   - Determine FAT type (12/16/32)
        //   - Register fsX: port with devd

        flog!("FAT probe: not yet implemented");
        Ok(())
    }

    fn run(&mut self) -> ! {
        flog!("fatfs driver starting");

        // Probe the partition
        if let Err(e) = self.probe_partition() {
            ferror!("probe failed: {:?}", e);
        }

        // Take the devd client for the service loop
        if let Some(mut client) = self.devd_client.take() {
            // Report ready state
            let _ = client.report_state(DriverState::Ready);
            flog!("reported ready, entering service loop");

            // Use the generic driver service loop
            run_driver_loop(client, FatfsSpawnHandler, 100);
        } else {
            ferror!("no devd client");
            syscall::exit(1);
        }
    }
}

/// Spawn handler for fatfs
///
/// fatfs is a leaf driver - it doesn't spawn children.
struct FatfsSpawnHandler;

impl SpawnHandler for FatfsSpawnHandler {
    fn handle_spawn(&mut self, binary: &str, _filter: &SpawnFilter) -> (u8, [u32; 16]) {
        // fatfs doesn't spawn children
        flog!("unexpected SPAWN_CHILD for {}", binary);
        (0, [0u32; 16])
    }

    fn handle_stop(&mut self, pid: u32) {
        flog!("unexpected STOP_CHILD for pid={}", pid);
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: FatfsDriver = FatfsDriver::new();

#[unsafe(no_mangle)]
fn main() {
    syscall::klog(LogLevel::Info, b"[fatfs] starting");

    // SAFETY: Single-threaded userspace driver
    let driver = unsafe { &mut *(&raw mut DRIVER) };

    if let Err(e) = driver.init() {
        ferror!("init failed: {:?}", e);
        syscall::exit(1);
    }

    driver.run()
}
