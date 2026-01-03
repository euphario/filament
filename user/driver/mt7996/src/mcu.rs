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

    /// Low-Power Control Register - Host Band 0
    /// Linux: MT_TOP_LPCR_HOST_BAND(band) = MT_TOP(0x10 + (band * 0x10))
    pub const LPCR_HOST_BAND: u32 = 0x10;
    /// Low-Power Control Register - Host Band 1 (HIF2)
    pub const LPCR_HOST_BAND1: u32 = 0x20;
    /// FW owns the device
    pub const LPCR_HOST_FW_OWN: u32 = 1 << 0;
    /// Driver owns the device
    pub const LPCR_HOST_DRV_OWN: u32 = 1 << 1;
    /// FW ownership status
    pub const LPCR_HOST_FW_OWN_STAT: u32 = 1 << 2;

    /// IRQ status register - Band 0
    pub const LPCR_HOST_BAND_IRQ_STAT: u32 = 0x14;
    /// IRQ status register - Band 1 (HIF2)
    pub const LPCR_HOST_BAND1_IRQ_STAT: u32 = 0x24;
    pub const LPCR_HOST_BAND_STAT: u32 = 1 << 0;

    /// Misc register (contains FW state)
    pub const MISC: u32 = 0xf0;
    /// FW state mask (bits 2:0)
    pub const MISC_FW_STATE_MASK: u32 = 0x7;
}

/// SWDEF register block (software-defined mode control)
/// Located in WF_MCU_SYSRAM region: physical 0x00400000 → BAR 0x80000
mod swdef {
    /// WF_MCU_SYSRAM base in BAR space
    const MCU_SYSRAM_BASE: u32 = 0x80000;
    /// SWDEF offset from MCU_SYSRAM base (0x00401400 - 0x00400000 = 0x1400)
    const SWDEF_OFFSET: u32 = 0x1400;

    /// SWDEF_MODE register - controls firmware operation mode
    /// Physical: 0x0040143c, BAR offset: 0x8143c
    /// MUST be set to NORMAL_MODE before driver_own()!
    pub const MODE: u32 = MCU_SYSRAM_BASE + SWDEF_OFFSET + 0x3c;

    /// Normal mode value - required for firmware download
    pub const NORMAL_MODE: u32 = 0;
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

    /// Acquire driver ownership for a specific band
    /// Linux: mt7996_driver_own(dev, band)
    /// Band 0 = primary interface, Band 1 = HIF2 (secondary interface)
    fn driver_own_band(&self, band: u8) -> bool {
        use userlib::println;

        // Select registers based on band
        // Linux: MT_TOP_LPCR_HOST_BAND(band) = MT_TOP(0x10 + (band * 0x10))
        let (lpcr_reg, irq_stat_reg) = if band == 0 {
            (top::LPCR_HOST_BAND, top::LPCR_HOST_BAND_IRQ_STAT)
        } else {
            (top::LPCR_HOST_BAND1, top::LPCR_HOST_BAND1_IRQ_STAT)
        };

        trace!(MCU, "acquiring driver ownership for band {}...", band);

        // Read initial state
        let initial = self.read_top(lpcr_reg);
        println!("[MCU] LPCR band{} initial: 0x{:08x} (FW_OWN={}, DRV_OWN={}, FW_OWN_STAT={})",
            band,
            initial,
            (initial & top::LPCR_HOST_FW_OWN) != 0,
            (initial & top::LPCR_HOST_DRV_OWN) != 0,
            (initial & top::LPCR_HOST_FW_OWN_STAT) != 0);

        // Write driver own bit
        self.write_top(lpcr_reg, top::LPCR_HOST_DRV_OWN);

        // Poll until FW ownership status clears (500ms timeout)
        let mut last_val = 0u32;
        let acquired = poll_until("driver_own", 500, || {
            let val = self.read_top(lpcr_reg);
            last_val = val;
            (val & top::LPCR_HOST_FW_OWN_STAT) == 0
        });

        if acquired {
            trace!(MCU, "driver own band{} acquired", band);
            // Clear IRQ status
            self.write_top(irq_stat_reg, top::LPCR_HOST_BAND_STAT);
            println!("[MCU] Driver ownership band{} acquired", band);
        } else {
            println!("[MCU] driver_own band{} timeout: LPCR=0x{:08x}", band, last_val);
            trace_err!("timeout waiting for driver own band{}", band);
        }
        acquired
    }

    /// Acquire driver ownership (band 0 - legacy wrapper)
    fn driver_own(&self) -> bool {
        self.driver_own_band(0)
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
    ///
    /// Linux order (from mt7996 driver analysis):
    /// 1. wfsys_reset() - unconditional in PCI probe (pci.c:139)
    /// 2. DMA init - in init_hardware (init.c:1189-1191) - BEFORE driver ownership!
    /// 3. SWDEF_MODE = normal - in mcu_init_firmware (mcu.c:3302)
    /// 4. Driver ownership - in mcu_init_firmware (mcu.c:3304-3312)
    /// 5. Load firmware
    pub fn reset(&mut self) -> bool {
        use userlib::println;

        // Step 1: wfsys_reset() - unconditional per Linux
        println!("[MCU] Performing wfsys_reset...");
        self.wfdma.wfsys_reset();
        println!("[MCU] wfsys_reset done");

        // Step 2: Initialize WFDMA - BEFORE driver ownership per Linux!
        // Linux: mt7996_init_hardware() → mt7996_dma_init() happens before mcu_init()
        println!("[MCU] Initializing WFDMA (before driver ownership, per Linux order)...");
        if !self.wfdma.init() {
            println!("[MCU] ERROR: WFDMA init failed");
            return false;
        }
        println!("[MCU] WFDMA init OK");

        // Step 3: Force firmware operation mode to normal
        println!("[MCU] Setting SWDEF_MODE to normal (0x{:x} <- 0)", swdef::MODE);
        self.dev.write32_raw(swdef::MODE, swdef::NORMAL_MODE);

        // Verify the write
        let swdef_readback = self.dev.read32_raw(swdef::MODE);
        println!("[MCU] SWDEF_MODE readback: 0x{:08x}", swdef_readback);

        // Step 4: Acquire driver ownership
        // Try up to 2 times with wfsys_reset between attempts
        for attempt in 0..2 {
            if attempt > 0 {
                println!("[MCU] Retry {} - performing wfsys_reset + DMA reinit...", attempt);
                self.wfdma.wfsys_reset();
                if !self.wfdma.init() {
                    println!("[MCU] ERROR: WFDMA reinit failed");
                    return false;
                }
                self.dev.write32_raw(swdef::MODE, swdef::NORMAL_MODE);
            }

            // Try to force release FW ownership first (write FW_OWN bit)
            let lpcr = self.read_top(top::LPCR_HOST_BAND);
            if (lpcr & top::LPCR_HOST_FW_OWN_STAT) != 0 {
                println!("[MCU] FW_OWN_STAT set, forcing release...");
                self.write_top(top::LPCR_HOST_BAND, top::LPCR_HOST_FW_OWN);
                // Wait a bit for FW to release
                for _ in 0..50 {
                    syscall::yield_now();
                }
            }

            // Acquire driver ownership for band 0
            println!("[MCU] Acquiring driver ownership (band 0)...");
            if self.driver_own_band(0) {
                // Linux: also acquire driver ownership for band 1 (HIF2) BEFORE firmware loading
                // This is CRITICAL - both interfaces must be owned before fw download
                if self.dev.has_hif2() {
                    println!("[MCU] Acquiring driver ownership (band 1 / HIF2)...");
                    if !self.driver_own_band(1) {
                        println!("[MCU] WARNING: Failed to acquire HIF2 driver ownership");
                        // Continue anyway - primary interface acquired
                    }
                }
                break;
            }

            if attempt == 0 {
                println!("[MCU] Driver ownership failed, will retry with reset...");
            } else {
                println!("[MCU] ERROR: Failed to acquire driver ownership after retry");
                return false;
            }
        }

        // Step 5: Check firmware state
        let fw_state = self.fw_state();
        println!("[MCU] Firmware state: {} (expected 1=FwDownload)", fw_state);

        if fw_state != FwState::FwDownload as u32 {
            println!("[MCU] WARNING: Firmware not in download state, MCU restart may be needed");
        }

        // Check MCU command state
        let mcu_state = self.wfdma.mcu_state();
        println!("[MCU] MCU cmd state: 0x{:08x}", mcu_state);

        // If already in normal state, we might need to restart
        if (mcu_state & mcu_cmd::NORMAL_STATE) != 0 {
            println!("[MCU] MCU already running, issuing stop");
            self.wfdma.set_mcu_cmd(mcu_cmd::STOP_DMA);

            // Wait for DMA to stop
            for _ in 0..100 {
                syscall::yield_now();
            }
        }

        // Step 6: Enable WFDMA AFTER driver ownership (per Linux load_firmware order)
        // Linux: mt7996_dma_enable() is called in mt7996_load_firmware() after driver_own()
        println!("[MCU] Enabling WFDMA (after driver ownership, per Linux order)...");
        if !self.wfdma.enable() {
            println!("[MCU] ERROR: WFDMA enable failed");
            return false;
        }
        println!("[MCU] WFDMA enable OK");

        self.state = McuState::Uninit;
        true
    }

    /// Load ROM patch firmware via DMA
    pub fn load_rom_patch(&mut self, fw: &Firmware) -> Result<(), FirmwareError> {
        use userlib::{println, print};
        trace!(FW, "loading ROM patch ({} bytes)...", fw.size);

        // Step 1: Acquire patch semaphore (MANDATORY per deepwiki)
        // This prevents concurrent patch loading and must be done before loading.
        // Semaphore commands go via WM ring, so use wait_dma_done_wm.
        println!("[PATCH] Acquiring patch semaphore...");
        if !self.wfdma.send_patch_sem_ctrl(true) {
            trace_err!("failed to send patch semaphore GET");
            return Err(FirmwareError::DeviceError);
        }

        // Wait for semaphore response (WM ring)
        if !self.wfdma.wait_dma_done_wm(100) {
            trace_err!("patch semaphore GET timeout");
            return Err(FirmwareError::Timeout);
        }
        println!("[PATCH] Patch semaphore acquired");

        let data = fw.data();
        let hdr_size = core::mem::size_of::<PatchHeader>();
        let sec_size = core::mem::size_of::<PatchSection>();

        println!("[PATCH] data ptr=0x{:x} len={} hdr_size={}",
            data.as_ptr() as usize, data.len(), hdr_size);

        if data.len() < hdr_size {
            trace_err!("patch too small for header");
            return Err(FirmwareError::InvalidHeader);
        }

        // Dump first 64 bytes of header for debugging
        println!("[PATCH] First 64 bytes:");
        for row in 0..4 {
            let offset = row * 16;
            print!("[PATCH]   {:03x}: ", offset);
            for i in 0..16 {
                if offset + i < data.len() {
                    print!("{:02x} ", data[offset + i]);
                }
            }
            println!();
        }

        // Parse patch header
        let hdr = unsafe { &*(data.as_ptr() as *const PatchHeader) };

        // Display patch info (copy values to avoid packed struct alignment issues)
        let hw_sw_ver_raw = { hdr.hw_sw_ver };
        let desc_n_region_raw = { hdr.desc_n_region };
        println!("[PATCH] build_date: {:?}", core::str::from_utf8(&hdr.build_date).unwrap_or("?"));
        println!("[PATCH] platform: {:?}", core::str::from_utf8(&hdr.platform).unwrap_or("?"));
        println!("[PATCH] hw_sw_ver=0x{:08x} (raw: 0x{:08x})",
            u32::from_be(hw_sw_ver_raw), hw_sw_ver_raw);
        println!("[PATCH] desc_n_region raw=0x{:08x} be={}",
            desc_n_region_raw, hdr.n_region());

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

        // Release patch semaphore (WM ring)
        println!("[PATCH] Releasing patch semaphore...");
        if !self.wfdma.send_patch_sem_ctrl(false) {
            trace_err!("failed to send patch semaphore RELEASE");
            // Continue anyway - patch is loaded
        } else if !self.wfdma.wait_dma_done_wm(100) {
            trace_err!("patch semaphore RELEASE timeout");
            // Continue anyway
        }
        println!("[PATCH] Patch semaphore released");

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
    /// Tries ramfs first, falls back to USB via fatfs
    pub fn load_firmware(&mut self) -> Result<(), FirmwareError> {
        use crate::firmware::check_firmware_files;
        use userlib::println;

        // Check if firmware is available in ramfs (embedded in initrd)
        if check_firmware_files() {
            println!("[MT7996] === Loading Firmware from ramfs ===");
            return self.load_firmware_ramfs();
        }

        // Fall back to USB loading
        println!("[MT7996] Firmware not in ramfs, trying USB...");
        self.load_firmware_usb()
    }

    /// Load firmware from ramfs (embedded in initrd)
    fn load_firmware_ramfs(&mut self) -> Result<(), FirmwareError> {
        use crate::firmware::{paths, Firmware};
        use userlib::println;

        // Reset MCU and init WFDMA
        println!("[MT7996] Resetting MCU...");
        if !self.reset() {
            println!("[MT7996] ERROR: MCU reset failed");
            return Err(FirmwareError::DeviceError);
        }
        println!("[MT7996] MCU reset OK, fw_state={}", self.fw_state());

        // Load ROM patch from ramfs
        println!("[MT7996] Loading ROM patch from ramfs...");
        let rom_patch = Firmware::load(paths::ROM_PATCH)?;
        println!("[MT7996] ROM patch loaded: {} bytes @ virt=0x{:x} phys=0x{:x}",
            rom_patch.size, rom_patch.vaddr, rom_patch.paddr);
        self.load_rom_patch(&rom_patch)?;
        println!("[MT7996] ROM patch sent OK");

        // DEBUG: Stop here for faster iteration (skip WM/WA)
        println!("[MT7996] === DEBUG: Stopping after ROM patch ===");
        return Ok(());

        #[allow(unreachable_code)]
        {
            // Load WM firmware from ramfs
            println!("[MT7996] Loading WM firmware from ramfs...");
            let wm = Firmware::load(paths::WM)?;
            println!("[MT7996] WM loaded: {} bytes @ virt=0x{:x} phys=0x{:x}",
                wm.size, wm.vaddr, wm.paddr);
            self.load_wm(&wm)?;
            println!("[MT7996] WM firmware sent OK");

            // Load WA firmware from ramfs
            println!("[MT7996] Loading WA firmware from ramfs...");
            let wa = Firmware::load(paths::WA)?;
            println!("[MT7996] WA loaded: {} bytes @ virt=0x{:x} phys=0x{:x}",
                wa.size, wa.vaddr, wa.paddr);
            self.load_wa(&wa)?;
            println!("[MT7996] WA firmware sent OK");

            println!("[MT7996] === Firmware loading complete ===");
            self.state = McuState::Ready;
            Ok(())
        }
    }

    /// Load firmware from USB via fatfs
    fn load_firmware_usb(&mut self) -> Result<(), FirmwareError> {
        use crate::firmware::{usb_names, UsbFirmware};
        use userlib::firmware::FirmwareClient;
        use userlib::println;

        println!("[MT7996] === Loading Firmware from USB ===");

        // Connect to fatfs service
        println!("[MT7996] Connecting to fatfs...");
        let client = match FirmwareClient::connect() {
            Some(c) => {
                println!("[MT7996] fatfs connected");
                c
            }
            None => {
                println!("[MT7996] ERROR: Failed to connect to fatfs");
                return Err(FirmwareError::UsbNotAvailable);
            }
        };

        // Reset MCU and init WFDMA
        println!("[MT7996] Resetting MCU...");
        if !self.reset() {
            println!("[MT7996] ERROR: MCU reset failed");
            return Err(FirmwareError::DeviceError);
        }
        println!("[MT7996] MCU reset OK, fw_state={}", self.fw_state());

        // Load ROM patch from USB
        let patch_name = core::str::from_utf8(usb_names::ROM_PATCH).unwrap_or("?");
        println!("[MT7996] Loading ROM patch '{}'...", patch_name);
        let rom_patch = match UsbFirmware::load(&client, usb_names::ROM_PATCH) {
            Ok(fw) => {
                println!("[MT7996] ROM patch loaded: {} bytes @ virt=0x{:x} phys=0x{:x}",
                    fw.size, fw.vaddr, fw.paddr);
                fw
            }
            Err(e) => {
                println!("[MT7996] ERROR: Failed to load ROM patch: {:?}", e);
                return Err(e);
            }
        };
        match self.load_rom_patch_usb(&rom_patch) {
            Ok(()) => println!("[MT7996] ROM patch sent OK"),
            Err(e) => {
                println!("[MT7996] ERROR: ROM patch transfer failed: {:?}", e);
                return Err(e);
            }
        }

        // Load WM firmware from USB
        let wm_name = core::str::from_utf8(usb_names::WM).unwrap_or("?");
        println!("[MT7996] Loading WM firmware '{}'...", wm_name);
        let wm = match UsbFirmware::load(&client, usb_names::WM) {
            Ok(fw) => {
                println!("[MT7996] WM loaded: {} bytes @ virt=0x{:x} phys=0x{:x}",
                    fw.size, fw.vaddr, fw.paddr);
                fw
            }
            Err(e) => {
                println!("[MT7996] ERROR: Failed to load WM: {:?}", e);
                return Err(e);
            }
        };
        match self.load_wm_usb(&wm) {
            Ok(()) => println!("[MT7996] WM firmware sent OK"),
            Err(e) => {
                println!("[MT7996] ERROR: WM transfer failed: {:?}", e);
                return Err(e);
            }
        }

        // Load WA firmware from USB
        let wa_name = core::str::from_utf8(usb_names::WA).unwrap_or("?");
        println!("[MT7996] Loading WA firmware '{}'...", wa_name);
        let wa = match UsbFirmware::load(&client, usb_names::WA) {
            Ok(fw) => {
                println!("[MT7996] WA loaded: {} bytes @ virt=0x{:x} phys=0x{:x}",
                    fw.size, fw.vaddr, fw.paddr);
                fw
            }
            Err(e) => {
                println!("[MT7996] ERROR: Failed to load WA: {:?}", e);
                return Err(e);
            }
        };
        match self.load_wa_usb(&wa) {
            Ok(()) => println!("[MT7996] WA firmware sent OK"),
            Err(e) => {
                println!("[MT7996] ERROR: WA transfer failed: {:?}", e);
                return Err(e);
            }
        }

        println!("[MT7996] === Firmware loaded successfully ===");

        Ok(())
    }

    /// Load ROM patch from USB firmware buffer
    fn load_rom_patch_usb(&mut self, fw: &crate::firmware::UsbFirmware) -> Result<(), FirmwareError> {
        use userlib::println;
        use crate::dma::PatchHeader;

        // Step 1: Acquire patch semaphore (MANDATORY per deepwiki)
        // Semaphore commands go via WM ring, so use wait_dma_done_wm.
        println!("[MT7996] Acquiring patch semaphore...");
        if !self.wfdma.send_patch_sem_ctrl(true) {
            println!("[MT7996] ERROR: failed to send patch semaphore GET");
            return Err(FirmwareError::DeviceError);
        }

        if !self.wfdma.wait_dma_done_wm(100) {
            println!("[MT7996] ERROR: patch semaphore GET timeout");
            return Err(FirmwareError::Timeout);
        }
        println!("[MT7996] Patch semaphore acquired");

        println!("[MT7996] Parsing ROM patch ({} bytes)...", fw.size);

        let data = fw.data();
        let hdr_size = core::mem::size_of::<PatchHeader>();

        if data.len() < hdr_size {
            println!("[MT7996] ERROR: Patch too small for header");
            return Err(FirmwareError::InvalidHeader);
        }

        // Parse patch header
        let hdr = unsafe { &*(data.as_ptr() as *const PatchHeader) };

        // Display patch info
        let build_date = core::str::from_utf8(&hdr.build_date).unwrap_or("?");
        let platform = core::str::from_utf8(&hdr.platform).unwrap_or("?");
        println!("[MT7996] Patch: build={} platform={}", build_date.trim_end_matches('\0'), platform);
        println!("[MT7996] Patch: hw_sw_ver=0x{:08x} patch_ver=0x{:08x} regions={}",
            u32::from_be(hdr.hw_sw_ver), u32::from_be(hdr.patch_ver), hdr.n_region());

        let n_region = hdr.n_region() as usize;
        if n_region == 0 {
            println!("[MT7996] ERROR: No patch regions!");
            return Err(FirmwareError::InvalidHeader);
        }

        // Sections follow the header
        let sec_size = core::mem::size_of::<crate::dma::PatchSection>();
        let sec_offset = hdr_size;
        if data.len() < sec_offset + n_region * sec_size {
            println!("[MT7996] ERROR: Patch too small for sections");
            return Err(FirmwareError::InvalidHeader);
        }

        // Process each region
        for i in 0..n_region {
            let sec_ptr = unsafe { data.as_ptr().add(sec_offset + i * sec_size) };
            let sec = unsafe { &*(sec_ptr as *const crate::dma::PatchSection) };

            let addr = sec.target_addr();
            let len = sec.data_len() as usize;
            let offset = sec.offset() as usize;
            let sec_sz = sec.sec_size() as usize;

            println!("[MT7996] Region {}: addr=0x{:08x} len={} offset={} size={}",
                i, addr, len, offset, sec_sz);

            // Validate data range
            if offset + sec_sz > data.len() {
                println!("[MT7996] ERROR: Region {} extends past end of file", i);
                return Err(FirmwareError::InvalidHeader);
            }

            // Send TARGET_ADDRESS_LEN_REQ
            if !self.wfdma.send_init_download(addr, len as u32, crate::dma::mcu::PATCH_MODE_NORMAL) {
                println!("[MT7996] ERROR: init_download failed for region {}", i);
                return Err(FirmwareError::DeviceError);
            }

            if !self.wfdma.wait_dma_done(100) {
                println!("[MT7996] ERROR: init_download timeout for region {}", i);
                return Err(FirmwareError::Timeout);
            }

            // Send firmware data via FW_SCATTER
            let region_data = &data[offset..offset + sec_sz];
            self.transfer_region(region_data)?;
        }

        // Send patch finish
        if !self.wfdma.send_patch_finish() {
            println!("[MT7996] ERROR: patch_finish failed");
            return Err(FirmwareError::DeviceError);
        }

        if !self.wfdma.wait_dma_done(100) {
            println!("[MT7996] ERROR: patch_finish timeout");
            return Err(FirmwareError::Timeout);
        }

        // Release patch semaphore (WM ring)
        println!("[MT7996] Releasing patch semaphore...");
        if !self.wfdma.send_patch_sem_ctrl(false) {
            println!("[MT7996] WARNING: failed to send patch semaphore RELEASE");
            // Continue anyway - patch is loaded
        } else if !self.wfdma.wait_dma_done_wm(100) {
            println!("[MT7996] WARNING: patch semaphore RELEASE timeout");
            // Continue anyway
        }
        println!("[MT7996] Patch semaphore released");

        self.state = McuState::Patched;
        Ok(())
    }

    /// Load WM firmware from USB buffer
    fn load_wm_usb(&mut self, fw: &crate::firmware::UsbFirmware) -> Result<(), FirmwareError> {
        use userlib::println;

        if self.state != McuState::Patched {
            println!("[MT7996] WARNING: Loading WM without ROM patch");
        }

        println!("[MT7996] Transferring WM firmware ({} bytes)...", fw.size);

        let data = fw.data();
        if data.len() < 32 {
            return Err(FirmwareError::InvalidHeader);
        }

        let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        println!("[MT7996] WM magic: 0x{:08x}", magic);

        // Transfer firmware via DMA
        self.transfer_region(data)?;

        self.state = McuState::WmLoaded;
        Ok(())
    }

    /// Load WA firmware from USB buffer
    fn load_wa_usb(&mut self, fw: &crate::firmware::UsbFirmware) -> Result<(), FirmwareError> {
        use userlib::println;

        if self.state != McuState::WmLoaded {
            println!("[MT7996] WARNING: Loading WA without WM");
        }

        println!("[MT7996] Transferring WA firmware ({} bytes)...", fw.size);

        let data = fw.data();
        if data.len() < 32 {
            return Err(FirmwareError::InvalidHeader);
        }

        let magic = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        println!("[MT7996] WA magic: 0x{:08x}", magic);

        // Transfer firmware via DMA
        self.transfer_region(data)?;

        self.state = McuState::Ready;
        Ok(())
    }
}
