//! MT7996 Firmware Loading
//!
//! Handles loading firmware files from ramfs and transferring them to the device.
//!
//! ## Firmware Files
//!
//! The MT7996 requires several firmware images:
//! - `mt7996_rom_patch.bin` - ROM patch (loaded first, ~37KB)
//! - `mt7996_wm.bin` - WiFi Manager firmware (~2.6MB)
//! - `mt7996_wa.bin` - WiFi Agent firmware (~509KB)
//! - `mt7996_dsp.bin` - DSP firmware (~192KB)
//! - `mt7996_eeprom.bin` - EEPROM calibration data (~8KB)
//!
//! ## Loading Sequence
//!
//! 1. Load ROM patch
//! 2. Load WM firmware (main processor)
//! 3. Load WA firmware (acceleration processor)
//! 4. Load DSP firmware (optional)

use userlib::syscall;
use userlib::firmware::FirmwareClient;

/// Firmware loading error
#[derive(Debug, Clone, Copy)]
pub enum FirmwareError {
    /// File not found in ramfs
    FileNotFound,
    /// Failed to open firmware file
    OpenFailed,
    /// Failed to read firmware file
    ReadFailed,
    /// Failed to allocate DMA memory
    DmaAllocFailed,
    /// Firmware too large
    TooLarge,
    /// Firmware header invalid
    InvalidHeader,
    /// Device communication error
    DeviceError,
    /// Timeout waiting for response
    Timeout,
    /// USB/fatfs service not available
    UsbNotAvailable,
}

/// Maximum firmware size (4MB should be enough for the largest file)
pub const MAX_FIRMWARE_SIZE: usize = 4 * 1024 * 1024;

/// Firmware file paths (ramfs)
pub mod paths {
    pub const ROM_PATCH: &str = "lib/firmware/mediatek/mt7996/mt7996_rom_patch.bin";
    pub const WM: &str = "lib/firmware/mediatek/mt7996/mt7996_wm.bin";
    pub const WA: &str = "lib/firmware/mediatek/mt7996/mt7996_wa.bin";
    pub const DSP: &str = "lib/firmware/mediatek/mt7996/mt7996_dsp.bin";
    pub const EEPROM: &str = "lib/firmware/mediatek/mt7996/mt7996_eeprom.bin";
}

/// Firmware filenames for USB loading (just the filename, no path)
pub mod usb_names {
    pub const ROM_PATCH: &[u8] = b"mt7996_rom_patch.bin";
    pub const WM: &[u8] = b"mt7996_wm.bin";
    pub const WA: &[u8] = b"mt7996_wa.bin";
    pub const DSP: &[u8] = b"mt7996_dsp.bin";
    pub const EEPROM: &[u8] = b"mt7996_eeprom.bin";
}

/// Firmware data buffer with DMA physical address
pub struct Firmware {
    /// Virtual address of firmware data
    pub vaddr: u64,
    /// Physical address for DMA
    pub paddr: u64,
    /// Size of firmware data
    pub size: usize,
}

impl Firmware {
    /// Load firmware from ramfs file path
    ///
    /// Allocates DMA memory and reads the entire file into it.
    pub fn load(path: &str) -> Result<Self, FirmwareError> {
        // Open the firmware file
        let fd = syscall::open_str(path, syscall::O_RDONLY);
        if fd < 0 {
            return Err(FirmwareError::FileNotFound);
        }
        let fd = fd as u32;

        // Get file size by seeking to end
        let size = syscall::lseek(fd, 0, syscall::SEEK_END);
        if size < 0 {
            syscall::close(fd);
            return Err(FirmwareError::ReadFailed);
        }
        let size = size as usize;

        if size > MAX_FIRMWARE_SIZE {
            syscall::close(fd);
            return Err(FirmwareError::TooLarge);
        }

        // Seek back to start
        if syscall::lseek(fd, 0, syscall::SEEK_SET) < 0 {
            syscall::close(fd);
            return Err(FirmwareError::ReadFailed);
        }

        // Allocate DMA memory
        let mut paddr: u64 = 0;
        let vaddr = syscall::mmap_dma(size, &mut paddr);
        if vaddr < 0 {
            syscall::close(fd);
            return Err(FirmwareError::DmaAllocFailed);
        }
        let vaddr = vaddr as u64;

        // Read firmware into DMA buffer
        let buf = unsafe { core::slice::from_raw_parts_mut(vaddr as *mut u8, size) };
        let mut total_read = 0usize;

        while total_read < size {
            let remaining = size - total_read;
            let chunk_size = core::cmp::min(remaining, 4096); // Read 4KB at a time

            let n = syscall::read(fd, &mut buf[total_read..total_read + chunk_size]);
            if n < 0 {
                syscall::close(fd);
                syscall::munmap(vaddr, size);
                return Err(FirmwareError::ReadFailed);
            }
            if n == 0 {
                break; // EOF
            }
            total_read += n as usize;
        }

        syscall::close(fd);

        if total_read != size {
            syscall::munmap(vaddr, size);
            return Err(FirmwareError::ReadFailed);
        }

        Ok(Self { vaddr, paddr, size })
    }

    /// Get firmware data as a slice
    pub fn data(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.vaddr as *const u8, self.size) }
    }

    /// Get firmware data as a mutable slice
    pub fn data_mut(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.vaddr as *mut u8, self.size) }
    }
}

impl Drop for Firmware {
    fn drop(&mut self) {
        syscall::munmap(self.vaddr, self.size);
    }
}

/// Check if firmware files are present in ramfs
pub fn check_firmware_files() -> bool {
    // Try to open each required firmware file
    for path in [paths::ROM_PATCH, paths::WM, paths::WA] {
        let fd = syscall::open_str(path, syscall::O_RDONLY);
        if fd < 0 {
            return false;
        }
        syscall::close(fd as u32);
    }
    true
}

/// Check if fatfs service is available (for USB firmware loading)
pub fn check_usb_available() -> bool {
    FirmwareClient::connect().is_some()
}

/// Firmware loaded from USB with shared memory handle
pub struct UsbFirmware {
    /// Virtual address of firmware data
    pub vaddr: u64,
    /// Physical address for DMA
    pub paddr: u64,
    /// Size of firmware data
    pub size: usize,
    /// Shared memory ID (for cleanup)
    shmem_id: u32,
}

impl UsbFirmware {
    /// Load firmware from USB via fatfs service
    pub fn load(client: &FirmwareClient, filename: &[u8]) -> Result<Self, FirmwareError> {
        match client.load_alloc(filename, MAX_FIRMWARE_SIZE) {
            Ok((vaddr, paddr, size, shmem_id)) => {
                Ok(Self { vaddr, paddr, size, shmem_id })
            }
            Err(code) => {
                if code == userlib::firmware::error::NOT_FOUND {
                    Err(FirmwareError::FileNotFound)
                } else if code == userlib::firmware::error::TOO_LARGE {
                    Err(FirmwareError::TooLarge)
                } else if code == userlib::firmware::error::NO_MEMORY {
                    Err(FirmwareError::DmaAllocFailed)
                } else {
                    Err(FirmwareError::ReadFailed)
                }
            }
        }
    }

    /// Get firmware data as a slice
    pub fn data(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.vaddr as *const u8, self.size) }
    }
}

impl Drop for UsbFirmware {
    fn drop(&mut self) {
        syscall::shmem_destroy(self.shmem_id);
    }
}
