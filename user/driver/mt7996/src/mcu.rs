//! MT7996 MCU (Microcontroller Unit) Interface
//!
//! The MT7996 has an embedded MCU that handles radio control.
//! Communication happens via DMA rings and mailbox registers.
//!
//! ## MCU Command Flow
//!
//! 1. Write command to TX DMA ring
//! 2. Ring doorbell to notify MCU
//! 3. Wait for response in RX DMA ring
//! 4. Parse response
//!
//! ## Firmware Loading Flow
//!
//! 1. Reset MCU
//! 2. Initialize WFDMA
//! 3. Load ROM patch via DMA
//! 4. Wait for patch ready
//! 5. Load WM firmware via DMA chunks
//! 6. Wait for WM ready
//! 7. Load WA firmware
//! 8. Wait for WA ready

use crate::device::Mt7996Device;
use crate::dma::{Wfdma, mcu_cmd};
use crate::firmware::{Firmware, FirmwareError};
use userlib::syscall;

/// MCU state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McuState {
    /// MCU not initialized
    Uninit,
    /// ROM patch loaded
    Patched,
    /// WM firmware loaded
    WmLoaded,
    /// WA firmware loaded
    Ready,
    /// Error state
    Error,
}

/// Firmware chunk size for DMA transfer (4KB)
const FW_CHUNK_SIZE: usize = 4096;

/// MCU interface for MT7996
pub struct Mcu<'a> {
    dev: &'a Mt7996Device,
    wfdma: Wfdma<'a>,
    state: McuState,
}

impl<'a> Mcu<'a> {
    /// Create new MCU interface
    pub fn new(dev: &'a Mt7996Device) -> Self {
        Self {
            wfdma: Wfdma::new(dev),
            dev,
            state: McuState::Uninit,
        }
    }

    /// Get current MCU state
    pub fn state(&self) -> McuState {
        self.state
    }

    /// Reset the MCU and initialize WFDMA
    pub fn reset(&mut self) -> bool {
        // Check current MCU state
        let mcu_state = self.wfdma.mcu_state();
        userlib::println!("  MCU state before reset: 0x{:08x}", mcu_state);

        // If already in normal state, we might need to restart
        if (mcu_state & mcu_cmd::NORMAL_STATE) != 0 {
            userlib::println!("  MCU already running, issuing stop");
            self.wfdma.set_mcu_cmd(mcu_cmd::STOP_DMA);

            // Wait for DMA to stop
            for _ in 0..100 {
                syscall::yield_now();
            }
        }

        // Initialize WFDMA
        if !self.wfdma.init() {
            userlib::println!("  WFDMA init failed");
            return false;
        }

        self.state = McuState::Uninit;
        true
    }

    /// Load ROM patch firmware via DMA
    pub fn load_rom_patch(&mut self, fw: &Firmware) -> Result<(), FirmwareError> {
        userlib::println!("  Loading ROM patch ({} bytes)...", fw.size);

        let data = fw.data();
        if data.len() < 32 {
            return Err(FirmwareError::InvalidHeader);
        }

        // Show firmware header info
        let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        userlib::println!("  ROM patch magic: 0x{:08x}", magic);

        // Transfer firmware via DMA
        self.transfer_firmware(fw)?;

        self.state = McuState::Patched;
        Ok(())
    }

    /// Load WM (WiFi Manager) firmware via DMA
    pub fn load_wm(&mut self, fw: &Firmware) -> Result<(), FirmwareError> {
        if self.state != McuState::Patched {
            userlib::println!("  Warning: Loading WM without ROM patch");
        }

        userlib::println!("  Loading WM firmware ({} bytes)...", fw.size);

        let data = fw.data();
        if data.len() < 32 {
            return Err(FirmwareError::InvalidHeader);
        }

        let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        userlib::println!("  WM magic: 0x{:08x}", magic);

        // Transfer firmware via DMA
        self.transfer_firmware(fw)?;

        self.state = McuState::WmLoaded;
        Ok(())
    }

    /// Load WA (WiFi Agent) firmware via DMA
    pub fn load_wa(&mut self, fw: &Firmware) -> Result<(), FirmwareError> {
        if self.state != McuState::WmLoaded {
            userlib::println!("  Warning: Loading WA without WM");
        }

        userlib::println!("  Loading WA firmware ({} bytes)...", fw.size);

        let data = fw.data();
        if data.len() < 32 {
            return Err(FirmwareError::InvalidHeader);
        }

        let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        userlib::println!("  WA magic: 0x{:08x}", magic);

        // Transfer firmware via DMA
        self.transfer_firmware(fw)?;

        self.state = McuState::Ready;
        Ok(())
    }

    /// Transfer firmware data via DMA in chunks
    fn transfer_firmware(&mut self, fw: &Firmware) -> Result<(), FirmwareError> {
        let total_size = fw.size;
        let mut offset = 0usize;
        let mut chunks = 0u32;

        while offset < total_size {
            let remaining = total_size - offset;
            let chunk_size = core::cmp::min(remaining, FW_CHUNK_SIZE);
            let is_last = offset + chunk_size >= total_size;

            // Calculate physical address for this chunk
            let chunk_paddr = fw.paddr + offset as u64;

            // Send chunk via DMA
            if !self.wfdma.send_firmware_chunk(chunk_paddr, chunk_size, is_last) {
                userlib::println!("  DMA send failed at offset {}", offset);
                return Err(FirmwareError::DeviceError);
            }

            // Wait for DMA completion every few chunks or on last chunk
            if chunks % 8 == 7 || is_last {
                if !self.wfdma.wait_dma_done(100) {
                    userlib::println!("  DMA timeout at offset {}", offset);
                    return Err(FirmwareError::Timeout);
                }
            }

            offset += chunk_size;
            chunks += 1;
        }

        userlib::println!("  Transferred {} chunks ({} bytes)", chunks, total_size);
        Ok(())
    }

    /// Full firmware loading sequence
    pub fn load_firmware(&mut self) -> Result<(), FirmwareError> {
        use crate::firmware::paths;

        userlib::println!();
        userlib::println!("=== Loading MT7996 Firmware ===");

        // Reset MCU and init WFDMA
        userlib::print!("Resetting MCU... ");
        if !self.reset() {
            userlib::println!("FAILED");
            return Err(FirmwareError::DeviceError);
        }
        userlib::println!("OK");

        // Load ROM patch
        userlib::print!("Loading ROM patch... ");
        let rom_patch = Firmware::load(paths::ROM_PATCH)?;
        self.load_rom_patch(&rom_patch)?;
        userlib::println!("OK");

        // Load WM firmware
        userlib::print!("Loading WM firmware... ");
        let wm = Firmware::load(paths::WM)?;
        self.load_wm(&wm)?;
        userlib::println!("OK");

        // Load WA firmware
        userlib::print!("Loading WA firmware... ");
        let wa = Firmware::load(paths::WA)?;
        self.load_wa(&wa)?;
        userlib::println!("OK");

        userlib::println!();
        userlib::println!("=== Firmware loaded successfully ===");

        Ok(())
    }
}
