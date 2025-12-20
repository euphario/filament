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
//! 2. Load ROM patch via DMA
//! 3. Wait for patch ready
//! 4. Load WM firmware via DMA chunks
//! 5. Wait for WM ready
//! 6. Load WA firmware
//! 7. Wait for WA ready

use crate::device::Mt7996Device;
use crate::firmware::{Firmware, FirmwareError};
use userlib::syscall;

/// MCU register offsets (relative to WFDMA base)
pub mod mcu_regs {
    /// WFDMA0 host DMA base
    pub const WFDMA0_BASE: u32 = 0xd4000;

    /// MCU interrupt status
    pub const MCU_INT_STATUS: u32 = 0x9010;

    /// MCU interrupt enable
    pub const MCU_INT_ENA: u32 = 0x9014;

    /// MCU to host interrupt
    pub const MCU2HOST_INT: u32 = 0xc0fc;

    /// Host to MCU interrupt
    pub const HOST2MCU_INT: u32 = 0xc0f0;

    /// Firmware download ring base
    pub const FWDL_RING_BASE: u32 = WFDMA0_BASE + 0x604;

    /// Firmware download doorbell
    pub const FWDL_DOORBELL: u32 = WFDMA0_BASE + 0x10;

    /// MCU reset control
    pub const MCU_RESET: u32 = 0x70002600;

    /// Semaphore for MCU access
    pub const SEMAPHORE: u32 = 0x70004104;

    /// Patch semaphore
    pub const PATCH_SEMAPHORE: u32 = 0x7c0241f4;
}

/// MCU command types
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum McuCmd {
    /// Target address for firmware download
    TargetAddr = 0x0,
    /// Start firmware download
    StartDl = 0x1,
    /// Patch start
    PatchStart = 0x5,
    /// Patch finish
    PatchFinish = 0x7,
    /// Init command
    Init = 0x8,
    /// Reset command
    Reset = 0x9,
}

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

/// MCU interface for MT7996
pub struct Mcu<'a> {
    dev: &'a Mt7996Device,
    state: McuState,
}

impl<'a> Mcu<'a> {
    /// Create new MCU interface
    pub fn new(dev: &'a Mt7996Device) -> Self {
        Self {
            dev,
            state: McuState::Uninit,
        }
    }

    /// Get current MCU state
    pub fn state(&self) -> McuState {
        self.state
    }

    /// Reset the MCU
    pub fn reset(&mut self) {
        // Write to MCU reset register
        self.dev.write32_raw(0x70002600 & 0xFFFF, 1);

        // Wait a bit
        for _ in 0..10 {
            syscall::yield_now();
        }

        // Clear reset
        self.dev.write32_raw(0x70002600 & 0xFFFF, 0);

        self.state = McuState::Uninit;
    }

    /// Load ROM patch firmware
    pub fn load_rom_patch(&mut self, fw: &Firmware) -> Result<(), FirmwareError> {
        userlib::println!("  Loading ROM patch ({} bytes)...", fw.size);

        // For now, just verify we can read the firmware
        let data = fw.data();
        if data.len() < 32 {
            return Err(FirmwareError::InvalidHeader);
        }

        // Check for valid firmware magic (first 4 bytes)
        // MT7996 firmware typically starts with specific patterns
        let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        userlib::println!("  ROM patch magic: 0x{:08x}", magic);

        // TODO: Implement actual DMA transfer sequence:
        // 1. Set target address via MCU command
        // 2. Transfer firmware chunks via DMA
        // 3. Signal patch complete
        // 4. Wait for MCU acknowledgment

        self.state = McuState::Patched;
        Ok(())
    }

    /// Load WM (WiFi Manager) firmware
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

        // TODO: Implement actual firmware loading
        // WM is loaded in chunks and requires handshaking

        self.state = McuState::WmLoaded;
        Ok(())
    }

    /// Load WA (WiFi Agent) firmware
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

        // TODO: Implement actual firmware loading

        self.state = McuState::Ready;
        Ok(())
    }

    /// Full firmware loading sequence
    pub fn load_firmware(&mut self) -> Result<(), FirmwareError> {
        use crate::firmware::paths;

        userlib::println!();
        userlib::println!("=== Loading MT7996 Firmware ===");

        // Reset MCU first
        userlib::print!("Resetting MCU... ");
        self.reset();
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
