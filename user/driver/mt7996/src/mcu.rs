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
//! 1. Acquire driver ownership
//! 2. Check firmware is in download state
//! 3. Initialize WFDMA
//! 4. Load ROM patch via DMA
//! 5. Wait for patch ready
//! 6. Load WM firmware via DMA chunks
//! 7. Wait for WM ready
//! 8. Load WA firmware
//! 9. Wait for WA ready

use crate::device::Mt7996Device;
use crate::dma::{Wfdma, mcu_cmd, mcu, PatchHeader, PatchSection};
use crate::firmware::{Firmware, FirmwareError};
use userlib::{syscall, trace, trace_err, poll_until};

/// TOP register block (for driver ownership and firmware state)
mod top {
    /// TOP register base
    pub const BASE: u32 = 0xe0000;

    /// Low-Power Control Register - Host Band
    pub const LPCR_HOST_BAND: u32 = 0x10;
    /// FW owns the device
    pub const LPCR_HOST_FW_OWN: u32 = 1 << 0;
    /// Driver owns the device
    pub const LPCR_HOST_DRV_OWN: u32 = 1 << 1;
    /// FW ownership status
    pub const LPCR_HOST_FW_OWN_STAT: u32 = 1 << 2;

    /// IRQ status register
    pub const LPCR_HOST_BAND_IRQ_STAT: u32 = 0x14;
    pub const LPCR_HOST_BAND_STAT: u32 = 1 << 0;

    /// Misc register (contains FW state)
    pub const MISC: u32 = 0xf0;
    /// FW state mask (bits 2:0)
    pub const MISC_FW_STATE_MASK: u32 = 0x7;
}

/// Firmware state values
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FwState {
    /// Initial state
    Initial = 0,
    /// Firmware download mode
    FwDownload = 1,
    /// Normal operation
    NormalOperation = 2,
    /// Normal TRX
    NormalTrx = 3,
    /// Ready (WA loaded)
    Ready = 7,
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

    /// Read TOP register
    fn read_top(&self, offset: u32) -> u32 {
        self.dev.read32_raw(top::BASE + offset)
    }

    /// Write TOP register
    fn write_top(&self, offset: u32, value: u32) {
        self.dev.write32_raw(top::BASE + offset, value);
    }

    /// Get current firmware state
    pub fn fw_state(&self) -> u32 {
        self.read_top(top::MISC) & top::MISC_FW_STATE_MASK
    }

    /// Acquire driver ownership
    fn driver_own(&self) -> bool {
        trace!(MCU, "acquiring driver ownership...");

        // Write driver own bit
        self.write_top(top::LPCR_HOST_BAND, top::LPCR_HOST_DRV_OWN);

        // Poll until FW ownership status clears (500ms timeout)
        let acquired = poll_until("driver_own", 500, || {
            let val = self.read_top(top::LPCR_HOST_BAND);
            (val & top::LPCR_HOST_FW_OWN_STAT) == 0
        });

        if acquired {
            trace!(MCU, "driver own acquired");
            // Clear IRQ status
            self.write_top(top::LPCR_HOST_BAND_IRQ_STAT, top::LPCR_HOST_BAND_STAT);
        } else {
            trace_err!("timeout waiting for driver own");
        }
        acquired
    }

    /// Wait for firmware to reach a specific state
    fn wait_fw_state(&self, target: FwState, timeout_ms: u32) -> bool {
        let target_val = target as u32;
        for _ in 0..timeout_ms {
            let current = self.fw_state();
            if current == target_val {
                return true;
            }
            // ~1ms delay
            for _ in 0..10 {
                syscall::yield_now();
            }
        }
        false
    }

    /// Reset the MCU and initialize WFDMA
    pub fn reset(&mut self) -> bool {
        // Step 1: Acquire driver ownership
        if !self.driver_own() {
            trace_err!("failed to acquire driver ownership");
            return false;
        }

        // Step 2: Check firmware state
        let fw_state = self.fw_state();
        trace!(MCU, "firmware state: {}", fw_state);

        if fw_state != FwState::FwDownload as u32 {
            trace!(MCU, "firmware not in download state ({}), MCU restart may be needed", fw_state);
            // Try to proceed anyway - device might need reset
        }

        // Check MCU command state
        let mcu_state = self.wfdma.mcu_state();
        trace!(MCU, "MCU cmd state: 0x{:08x}", mcu_state);

        // If already in normal state, we might need to restart
        if (mcu_state & mcu_cmd::NORMAL_STATE) != 0 {
            trace!(MCU, "MCU already running, issuing stop");
            self.wfdma.set_mcu_cmd(mcu_cmd::STOP_DMA);

            // Wait for DMA to stop
            for _ in 0..100 {
                syscall::yield_now();
            }
        }

        // Step 3: Initialize WFDMA
        if !self.wfdma.init() {
            trace_err!("WFDMA init failed");
            return false;
        }

        self.state = McuState::Uninit;
        true
    }

    /// Load ROM patch firmware via DMA
    pub fn load_rom_patch(&mut self, fw: &Firmware) -> Result<(), FirmwareError> {
        trace!(FW, "loading ROM patch ({} bytes)...", fw.size);

        let data = fw.data();
        let hdr_size = core::mem::size_of::<PatchHeader>();
        let sec_size = core::mem::size_of::<PatchSection>();

        if data.len() < hdr_size {
            trace_err!("patch too small for header");
            return Err(FirmwareError::InvalidHeader);
        }

        // Parse patch header
        let hdr = unsafe { &*(data.as_ptr() as *const PatchHeader) };

        // Display patch info
        trace!(FW, "patch build_date={:?} platform={:?}",
            core::str::from_utf8(&hdr.build_date).unwrap_or("?"),
            core::str::from_utf8(&hdr.platform).unwrap_or("?"));
        trace!(FW, "patch hw_sw_ver=0x{:08x} patch_ver=0x{:08x} regions={}",
            u32::from_be(hdr.hw_sw_ver), u32::from_be(hdr.patch_ver), hdr.n_region());

        let n_region = hdr.n_region() as usize;
        if n_region == 0 {
            trace_err!("no patch regions!");
            return Err(FirmwareError::InvalidHeader);
        }

        // Sections follow the header
        let sec_offset = hdr_size;
        if data.len() < sec_offset + n_region * sec_size {
            trace_err!("patch too small for sections");
            return Err(FirmwareError::InvalidHeader);
        }

        // Process each region
        for i in 0..n_region {
            let sec_ptr = unsafe { data.as_ptr().add(sec_offset + i * sec_size) };
            let sec = unsafe { &*(sec_ptr as *const PatchSection) };

            let addr = sec.target_addr();
            let len = sec.data_len() as usize;
            let offset = sec.offset() as usize;
            let sec_sz = sec.sec_size() as usize;

            trace!(FW, "region {}: addr=0x{:08x} len={} offset={} size={}",
                i, addr, len, offset, sec_sz);

            // Validate data range
            if offset + sec_sz > data.len() {
                trace_err!("region {} extends past end of file", i);
                return Err(FirmwareError::InvalidHeader);
            }

            // Send TARGET_ADDRESS_LEN_REQ to tell MCU where to load this region
            if !self.wfdma.send_init_download(addr, len as u32, mcu::PATCH_MODE_NORMAL) {
                trace_err!("init_download failed for region {}", i);
                return Err(FirmwareError::DeviceError);
            }

            // Wait for command to complete
            if !self.wfdma.wait_dma_done(100) {
                trace_err!("init_download timeout for region {}", i);
                return Err(FirmwareError::Timeout);
            }

            // Send firmware data via FW_SCATTER
            let region_data = &data[offset..offset + sec_sz];
            self.transfer_region(region_data)?;
        }

        // Send patch finish
        if !self.wfdma.send_patch_finish() {
            trace_err!("patch_finish failed");
            return Err(FirmwareError::DeviceError);
        }

        if !self.wfdma.wait_dma_done(100) {
            trace_err!("patch_finish timeout");
            return Err(FirmwareError::Timeout);
        }

        self.state = McuState::Patched;
        Ok(())
    }

    /// Transfer a region of firmware via FW_SCATTER commands
    fn transfer_region(&mut self, data: &[u8]) -> Result<(), FirmwareError> {
        let total_size = data.len();
        let mut offset = 0usize;
        let mut chunks = 0u32;

        while offset < total_size {
            let remaining = total_size - offset;
            let chunk_size = core::cmp::min(remaining, FW_CHUNK_SIZE);

            let chunk_data = &data[offset..offset + chunk_size];

            if !self.wfdma.send_fw_scatter(chunk_data) {
                trace_err!("FW_SCATTER failed at offset {}", offset);
                return Err(FirmwareError::DeviceError);
            }

            if !self.wfdma.wait_dma_done(100) {
                trace_err!("FW_SCATTER timeout at offset {}", offset);
                return Err(FirmwareError::Timeout);
            }

            offset += chunk_size;
            chunks += 1;
        }

        trace!(FW, "sent {} chunks ({} bytes)", chunks, total_size);
        Ok(())
    }

    /// Load WM (WiFi Manager) firmware via DMA
    pub fn load_wm(&mut self, fw: &Firmware) -> Result<(), FirmwareError> {
        if self.state != McuState::Patched {
            trace!(FW, "warning: loading WM without ROM patch");
        }

        trace!(FW, "loading WM firmware ({} bytes)...", fw.size);

        let data = fw.data();
        if data.len() < 32 {
            return Err(FirmwareError::InvalidHeader);
        }

        let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        trace!(FW, "WM magic: 0x{:08x}", magic);

        // Transfer firmware via DMA
        self.transfer_firmware(fw)?;

        self.state = McuState::WmLoaded;
        Ok(())
    }

    /// Load WA (WiFi Agent) firmware via DMA
    pub fn load_wa(&mut self, fw: &Firmware) -> Result<(), FirmwareError> {
        if self.state != McuState::WmLoaded {
            trace!(FW, "warning: loading WA without WM");
        }

        trace!(FW, "loading WA firmware ({} bytes)...", fw.size);

        let data = fw.data();
        if data.len() < 32 {
            return Err(FirmwareError::InvalidHeader);
        }

        let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        trace!(FW, "WA magic: 0x{:08x}", magic);

        // Transfer firmware via DMA
        self.transfer_firmware(fw)?;

        self.state = McuState::Ready;
        Ok(())
    }

    /// Transfer firmware data via DMA using MCU FW_SCATTER commands
    fn transfer_firmware(&mut self, fw: &Firmware) -> Result<(), FirmwareError> {
        let data = fw.data();
        let total_size = data.len();
        let mut offset = 0usize;
        let mut chunks = 0u32;

        while offset < total_size {
            let remaining = total_size - offset;
            let chunk_size = core::cmp::min(remaining, FW_CHUNK_SIZE);

            // Get chunk data
            let chunk_data = &data[offset..offset + chunk_size];

            // Send FW_SCATTER command with this chunk
            if !self.wfdma.send_fw_scatter(chunk_data) {
                trace_err!("DMA send failed at offset {}", offset);
                return Err(FirmwareError::DeviceError);
            }

            // Wait for DMA completion after each chunk (since we reuse the command buffer)
            if !self.wfdma.wait_dma_done(100) {
                trace_err!("DMA timeout at offset {}", offset);
                return Err(FirmwareError::Timeout);
            }

            offset += chunk_size;
            chunks += 1;
        }

        trace!(FW, "transferred {} chunks ({} bytes)", chunks, total_size);
        Ok(())
    }

    /// Full firmware loading sequence
    pub fn load_firmware(&mut self) -> Result<(), FirmwareError> {
        use crate::firmware::paths;

        trace!(INIT, "=== Loading MT7996 Firmware ===");

        // Reset MCU and init WFDMA
        trace!(INIT, "resetting MCU...");
        if !self.reset() {
            trace_err!("MCU reset failed");
            return Err(FirmwareError::DeviceError);
        }
        trace!(INIT, "MCU reset OK");

        // Load ROM patch
        trace!(INIT, "loading ROM patch...");
        let rom_patch = Firmware::load(paths::ROM_PATCH)?;
        self.load_rom_patch(&rom_patch)?;
        trace!(INIT, "ROM patch OK");

        // Load WM firmware
        trace!(INIT, "loading WM firmware...");
        let wm = Firmware::load(paths::WM)?;
        self.load_wm(&wm)?;
        trace!(INIT, "WM firmware OK");

        // Load WA firmware
        trace!(INIT, "loading WA firmware...");
        let wa = Firmware::load(paths::WA)?;
        self.load_wa(&wa)?;
        trace!(INIT, "WA firmware OK");

        trace!(INIT, "=== Firmware loaded successfully ===");

        Ok(())
    }
}
