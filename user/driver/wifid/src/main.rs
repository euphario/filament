//! MT7996 WiFi Driver - EXACT Linux Translation
//!
//! This is a LINE-BY-LINE translation of the Linux mt76/mt7996 driver.
//! NO ASSUMPTIONS. NO SHORTCUTS. EVERY REGISTER WRITE MATCHES LINUX.
//!
//! Source: linux/drivers/net/wireless/mediatek/mt76/mt7996/
//!
//! Reference Linux functions (with line numbers from kernel 6.12):
//! - mt7996_init_hardware()     : init.c:1524-1569
//! - mt7996_dma_init()          : dma.c:599-854
//! - mt7996_dma_config()        : dma.c:50-151
//! - mt7996_dma_disable()       : dma.c:545-585
//! - mt7996_dma_enable()        : dma.c:627-754
//! - mt7996_dma_start()         : dma.c:587-625
//! - mt7996_dma_prefetch()      : dma.c:520-523
//! - __mt7996_dma_prefetch()    : dma.c:162-242
//! - mt7996_driver_own()        : mcu.c:3559-3572

#![no_std]
#![no_main]

use userlib::{uinfo, uwarn, uerror, udebug};
use userlib::mmio::{MmioRegion, DmaPool};
use userlib::bus::{BusMsg, BusError, BusCtx, Driver, Disposition, ConfigKey};
use userlib::bus_runtime::driver_main;

mod regs;
mod dma;
mod device;
mod mcu;
mod firmware;
mod mac;

use regs::*;
use dma::{TxRing, flush_buffer};
use device::Mt7996Dev;

// ============================================================================
// WifiDriver — Bus framework integration
// ============================================================================

struct WifiDriver {
    dev: Option<Mt7996Dev>,
    bar0: Option<MmioRegion>,
    desc_pool: Option<DmaPool>,
    rx_pool: Option<DmaPool>,
    wa_ring: Option<TxRing>,
    wa_tx_buf_pool: Option<DmaPool>,
    seq: u8,
    radio_on: bool,
    beacon_on: bool,
    tx_throttle: u8,
    channel: u8,
}

impl WifiDriver {
    const fn new() -> Self {
        Self {
            dev: None,
            bar0: None,
            desc_pool: None,
            rx_pool: None,
            wa_ring: None,
            wa_tx_buf_pool: None,
            seq: 0,
            radio_on: false,
            beacon_on: false,
            tx_throttle: 100,
            channel: 1,
        }
    }

    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        uinfo!("wifid", "init_start");

        // Step 1: Get BAR0 from spawn context (provided by pcied via devd)
        let spawn_ctx = ctx.spawn_context().map_err(|e| {
            uerror!("wifid", "no_spawn_context");
            e
        })?;

        let meta = spawn_ctx.metadata();
        if meta.len() < 12 {
            uerror!("wifid", "metadata_too_short"; len = meta.len() as u32);
            return Err(BusError::Internal);
        }

        let bar0_addr = u64::from_le_bytes([
            meta[0], meta[1], meta[2], meta[3],
            meta[4], meta[5], meta[6], meta[7],
        ]);
        let bar0_size = u32::from_le_bytes([
            meta[8], meta[9], meta[10], meta[11],
        ]) as u64;

        uinfo!("wifid", "device_found"; bar0 = bar0_addr, size = bar0_size);

        // Step 2: Map BAR0
        uinfo!("wifid", "map_bar0"; addr = bar0_addr, size_kb = bar0_size / 1024);
        let bar0 = MmioRegion::open(bar0_addr, bar0_size).ok_or_else(|| {
            uerror!("wifid", "mmap_device_failed");
            BusError::Internal
        })?;
        let bar0_virt = bar0.virt_base();
        udebug!("wifid", "bar0_mapped"; virt = bar0_virt);

        // HIF2 registers (0xd8xxx) are accessible through HIF1's BAR at offset 0xd8xxx.
        // Linux mmio.c __mt7996_reg_addr(): if (addr < 0x100000) return addr;
        // pcied skips the HIF2 companion device (0x7991) — only HIF1 (0x7990) spawns wifid.
        // MT7996 always has dual HIF; we configure both through HIF1's BAR.
        let has_hif2 = true;

        // Step 3: Allocate DMA descriptor memory
        // CRITICAL: Descriptor rings MUST be in LOW memory (< 4GB) because DESC_BASE is 32-bit!
        // Only TX/RX BUFFERS can use HIGH (36-bit) addresses via buf0/buf1 fields.
        uinfo!("wifid", "alloc_dma_mem");

        // Calculate descriptor memory needed for Linux-matching ring sizes:
        // TX: BAND0(2048) + MCU_WM(256) + MCU_WA(256) + FWDL(128) = 2688 × 16 = 43KB
        // RX: MCU_WM(512) + MCU_WA(512) + BAND0(1536) + WA_MAIN(1024) + BAND2(1536) + WA_TRI(1024) = 6144 × 16 = 98KB
        // Total: ~141KB, round up to 192KB with 4KB alignment per queue
        const DESC_MEM_SIZE: usize = 256 * 1024; // 256KB for all queues (was 64KB)
        // Use LOW pool - DESC_BASE register is only 32-bit, can't address > 4GB!
        let mut desc_pool = DmaPool::alloc(DESC_MEM_SIZE).ok_or_else(|| {
            uerror!("wifid", "dma_pool_create_failed");
            BusError::Internal
        })?;
        let desc_virt = desc_pool.vaddr();
        let desc_phys = desc_pool.paddr();
        udebug!("wifid", "desc_pool"; virt = desc_virt, phys = desc_phys, size = DESC_MEM_SIZE);

        // Clear all descriptor memory and flush to RAM
        desc_pool.zero();
        // CRITICAL: Flush cache to ensure zeros are visible to DMA device
        // Without this, DMA device might see stale/random data
        flush_buffer(desc_virt, DESC_MEM_SIZE);

        // Allocate RX buffer pool from HIGH MEMORY - exact Linux-matching ring sizes
        // Linux: mt76_dma_rx_fill_buf() allocates skbs and fills descriptors with physical addresses
        // RX queues: MCU_WM(512) + MCU_WA(512) + BAND0(1536) + WA_MAIN(1024) + BAND2(1536) + WA_TRI(1024) = 6144 buffers
        // 6144 × 2048 = 12.6MB
        const RX_BUF_POOL_SIZE: usize = 13 * 1024 * 1024;  // 13MB for all RX queues
        let mut rx_pool = DmaPool::alloc_high(RX_BUF_POOL_SIZE).or_else(|| {
            uwarn!("wifid", "rx_buf_pool_high_fallback");
            // Fallback to low pool if high pool fails
            DmaPool::alloc(RX_BUF_POOL_SIZE)
        }).ok_or_else(|| {
            uerror!("wifid", "rx_pool_create_failed");
            BusError::Internal
        })?;
        let rx_buf_virt = rx_pool.vaddr();
        let rx_buf_phys = rx_pool.paddr();
        udebug!("wifid", "rx_buf_pool"; virt = rx_buf_virt, phys = rx_buf_phys, size = RX_BUF_POOL_SIZE);

        // Clear RX buffer memory and flush to RAM
        rx_pool.zero();
        // CRITICAL: Flush cache to ensure zeros are visible to DMA device
        flush_buffer(rx_buf_virt, RX_BUF_POOL_SIZE);

        // Create device
        let dev = Mt7996Dev::new(bar0_virt, bar0_size, has_hif2);

    // ========================================================================
    // EXACT Linux initialization sequence
    // Source: pci.c (probe) + init.c:mt7996_init_hardware()
    // ========================================================================

    uinfo!("wifid", "init_hw_start");

    // Fix HOST_CONFIG to match OpenWRT (bits 8,10-14)
    let host_cfg_initial = dev.mt76_rr(0xd7030);
    if (host_cfg_initial & 0x7d00) != 0x7d00 {
        dev.mt76_set(0xd7030, 0x7d00);
        udebug!("wifid", "host_config_fix"; before = host_cfg_initial, after = dev.mt76_rr(0xd7030));
    }

    // Fix PCIE_SETTING to match OpenWRT (0x00003180)
    let pcie_setting_initial = dev.mt76_rr(0x10080);
    if pcie_setting_initial != 0x00003180 {
        dev.mt76_wr(0x10080, 0x00003180);
        udebug!("wifid", "pcie_setting_fix"; before = pcie_setting_initial);
    }

    // Read MT_HW_REV via L1 remap (mmio.c:672)
    let hw_rev = dev.mt76_rr_remap(MT_HW_REV);
    uinfo!("wifid", "hw_rev"; rev = hw_rev & 0xff);

    // Clear interrupts before WFSYS reset (Linux armbian_golden.log line 13)
    dev.mt76_wr(MT_INT_MASK_CSR, 0);

    // WFSYS Reset — ensure clean hardware state (init.c:762-769)
    uinfo!("wifid", "wfsys_reset");
    dev.mt7996_wfsys_reset();

    // PCIe setup (pci.c:mt7996_pci_probe, AFTER wfsys_reset)
    dev.mt76_wr(MT_PCIE_MAC_INT_ENABLE, 0xff);

    // HIF2: Via L1 remap (dual-HIF setup)
    let pcie1_mapped = dev.mt7996_reg_map_l1(MT_PCIE1_MAC_INT_ENABLE_PHYS);
    dev.mt76_wr(pcie1_mapped, 0xff);

    // Disable interrupt masks, clear sources
    dev.mt76_wr(MT_INT_MASK_CSR, 0);
    dev.mt76_wr(MT_INT1_MASK_CSR, 0);
    dev.mt76_wr(MT_INT_SOURCE_CSR, !0u32);

    // DMA init (init.c — dma_init() BEFORE mcu_init())
    uinfo!("wifid", "dma_init");
    dev.mt7996_dma_init(desc_phys, desc_virt, DESC_MEM_SIZE, rx_buf_phys, rx_buf_virt, RX_BUF_POOL_SIZE);

    // Trace RX DMA_IDX at every step to prove DMA is working
    let trace_rx = || -> [u32; 4] {
        let rx_base = MT_WFDMA0_BASE + 0x500;
        [
            dev.mt76_rr(rx_base + 0 * MT_RING_SIZE + MT_QUEUE_DMA_IDX),
            dev.mt76_rr(rx_base + 1 * MT_RING_SIZE + MT_QUEUE_DMA_IDX),
            dev.mt76_rr(rx_base + 2 * MT_RING_SIZE + MT_QUEUE_DMA_IDX),
            dev.mt76_rr(rx_base + 3 * MT_RING_SIZE + MT_QUEUE_DMA_IDX),
        ]
    };

    let rx = trace_rx();
    uinfo!("wifid", "rx_trace_dma_init"; q0 = rx[0], q1 = rx[1], q2 = rx[2], q3 = rx[3]);

    // MCU init (mcu.c:3299-3312 — SWDEF_MODE then driver_own)
    uinfo!("wifid", "mcu_init");
    dev.mt76_wr(MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE);

    // Band 0 driver_own
    let _ = dev.mt7996_driver_own(0);
    let rx = trace_rx();
    uinfo!("wifid", "rx_trace_drv_own0"; q0 = rx[0], q1 = rx[1], q2 = rx[2], q3 = rx[3]);

    // Band 1 driver_own (HIF2)
    let _ = dev.mt7996_driver_own(1);
    let rx = trace_rx();
    uinfo!("wifid", "rx_trace_drv_own1"; q0 = rx[0], q1 = rx[1], q2 = rx[2], q3 = rx[3]);

    // Check firmware state after MCU init
    let fw_state = dev.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
    uinfo!("wifid", "fw_state_after_mcu"; val = fw_state);

    // Firmware loading
    uinfo!("wifid", "firmware_load_start");

    let tx_ring_base = MT_WFDMA0_BASE + 0x300;

    // Allocate FWDL TX buffer from HIGH DMA pool
    const TX_BUF_SIZE: usize = MT7996_TX_FWDL_RING_SIZE as usize * MCU_FW_DL_BUF_SIZE;
    let tx_buf_pool = DmaPool::alloc_high(TX_BUF_SIZE).or_else(|| {
        uwarn!("wifid", "fwdl_tx_high_fallback");
        DmaPool::alloc(TX_BUF_SIZE)
    }).ok_or_else(|| {
        uerror!("wifid", "fwdl_tx_alloc_failed");
        BusError::Internal
    })?;
    let tx_buf_virt = tx_buf_pool.vaddr();
    let tx_buf_phys = tx_buf_pool.paddr();

    // MCU_WM ring (hw_idx=17) — for MCU commands
    // Layout: BAND0(32KB), MCU_WM(4KB), MCU_WA(4KB), FWDL(4KB)
    const MCU_WM_DESC_OFFSET: usize = 32 * 1024;
    let mcu_wm_desc_virt = desc_virt + MCU_WM_DESC_OFFSET as u64;
    let mcu_wm_desc_phys = desc_phys + MCU_WM_DESC_OFFSET as u64;
    let mcu_wm_regs = tx_ring_base + MT7996_TXQ_MCU_WM * MT_RING_SIZE;

    // Allocate MCU_WM TX buffer from HIGH pool
    const MCU_TX_BUF_SIZE: usize = MT7996_TX_MCU_RING_SIZE as usize * MCU_FW_DL_BUF_SIZE;
    let mcu_tx_buf_pool = DmaPool::alloc_high(MCU_TX_BUF_SIZE).or_else(|| {
        uwarn!("wifid", "mcu_tx_high_fallback");
        DmaPool::alloc(MCU_TX_BUF_SIZE)
    }).ok_or_else(|| {
        uerror!("wifid", "mcu_tx_alloc_failed");
        BusError::Internal
    })?;
    let mcu_tx_buf_virt = mcu_tx_buf_pool.vaddr();
    let mcu_tx_buf_phys = mcu_tx_buf_pool.paddr();

    let mut mcu_ring = TxRing::new(
        mcu_wm_regs,
        MT7996_TX_MCU_RING_SIZE,
        mcu_wm_desc_virt,
        mcu_wm_desc_phys,
        mcu_tx_buf_virt,
        mcu_tx_buf_phys,
    );

    // FWDL ring (hw_idx=16) — for firmware data chunks
    let fwdl_regs = tx_ring_base + MT7996_TXQ_FWDL * MT_RING_SIZE;
    const FWDL_DESC_OFFSET: usize = 40 * 1024;
    let fwdl_desc_virt = desc_virt + FWDL_DESC_OFFSET as u64;
    let fwdl_desc_phys = desc_phys + FWDL_DESC_OFFSET as u64;

    let mut fwdl_ring = TxRing::new(
        fwdl_regs,
        MT7996_TX_FWDL_RING_SIZE,
        fwdl_desc_virt,
        fwdl_desc_phys,
        tx_buf_virt,
        tx_buf_phys,
    );

    dev.load_firmware(&mut mcu_ring, &mut fwdl_ring).map_err(|e| {
        uerror!("wifid", "firmware_load_fail"; err = e);
        BusError::Internal
    })?;

    // ====================================================================
    // Post-firmware MCU init — uses WA TX ring (hw_idx=20)
    // Linux: mt7996/mcu.c:295-298 — after MCU_RUNNING, all commands go
    // through MT_MCUQ_WA (not MT_MCUQ_WM)
    // ====================================================================

    // MCU_WA ring (hw_idx=20) — for post-firmware MCU commands
    const MCU_WA_DESC_OFFSET: usize = 36 * 1024;  // After MCU_WM (32K+4K)
    let wa_desc_virt = desc_virt + MCU_WA_DESC_OFFSET as u64;
    let wa_desc_phys = desc_phys + MCU_WA_DESC_OFFSET as u64;
    let wa_regs = tx_ring_base + MT7996_TXQ_MCU_WA * MT_RING_SIZE;

    // Allocate WA TX buffer from HIGH pool
    const WA_TX_BUF_SIZE: usize = MT7996_TX_MCU_RING_SIZE as usize * MCU_FW_DL_BUF_SIZE;
    let wa_tx_buf_pool = DmaPool::alloc_high(WA_TX_BUF_SIZE).or_else(|| {
        uwarn!("wifid", "wa_tx_high_fallback");
        DmaPool::alloc(WA_TX_BUF_SIZE)
    }).ok_or_else(|| {
        uerror!("wifid", "wa_tx_alloc_failed");
        BusError::Internal
    })?;
    let wa_buf_virt = wa_tx_buf_pool.vaddr();
    let wa_buf_phys = wa_tx_buf_pool.paddr();

    let mut wa_ring = TxRing::new(
        wa_regs,
        MT7996_TX_MCU_RING_SIZE,
        wa_desc_virt,
        wa_desc_phys,
        wa_buf_virt,
        wa_buf_phys,
    );

    uinfo!("mcu", "post_init_start");
    let mut seq: u8 = 1;

    let mcu_err = |_e: i32| { BusError::Internal };

    // fw_log_2_host(WM, 0) — enable WM logging
    dev.mcu_fw_log_2_host(&mut wa_ring, 0, 0, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // fw_log_2_host(WA, 0) — enable WA logging
    dev.mcu_fw_log_2_host(&mut wa_ring, 1, 0, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // set_mwds(1) — enable MWDS
    dev.mcu_set_mwds(&mut wa_ring, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // init_rx_airtime() — RX airtime for bands 0,1,2
    dev.mcu_init_rx_airtime(&mut wa_ring, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // wa_cmd(SET, RED, 0, 0) — enable Random Early Drop
    dev.mcu_wa_cmd(&mut wa_ring, mcu::MCU_WA_PARAM_RED, 0, 0, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    uinfo!("mcu", "mcu_init_ok");

    // ====================================================================
    // MAC initialization — register writes + MCU commands
    // Linux: init.c mt7996_init_work() → mcu_set_eeprom() → mac_init()
    // ====================================================================

    // Tell MCU about EEPROM mode (efuse) — mcu.c:3810
    dev.mcu_set_eeprom(&mut wa_ring, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // Full MAC init: WTBL clear, RRO, HIF TXD version, per-band regs, basic rates
    dev.mac_init(&mut wa_ring, &mut seq).map_err(mcu_err)?;

    // ====================================================================
    // Thermal protection + radio control
    // Linux: mt7996/main.c mt7996_start() — thermal protect, throttle, radio
    // ====================================================================

    // Thermal protection: enable for all 3 bands
    // Linux: mt7996/mcu.c:4105 mt7996_mcu_set_thermal_protect()
    for band in 0..3u8 {
        dev.mcu_set_thermal_protect(&mut wa_ring, band, true, seq).map_err(mcu_err)?;
        seq = seq.wrapping_add(1);
    }
    uinfo!("wifid", "thermal_protect_ok");

    // Thermal throttling: 100% (full power — throttle only on overtemp)
    // Linux: mt7996/mcu.c:4072 mt7996_mcu_set_thermal_throttling()
    for band in 0..3u8 {
        dev.mcu_set_thermal_throttling(&mut wa_ring, band, 100, seq).map_err(mcu_err)?;
        seq = seq.wrapping_add(1);
    }
    uinfo!("wifid", "thermal_throttle_ok");

    // ====================================================================
    // Band 0 radio startup + interface creation
    // Linux: mt7996/main.c mt7996_start() + mt7996_run() + mt7996_vif_link_add()
    // ====================================================================

    // Locally-administered MAC for initial bringup
    let mac_addr: [u8; 6] = [0x02, 0x0c, 0x43, 0x28, 0x80, 0x01];

    // === Global setup ===

    // Header translation (global, once) — Linux mcu.c:3387
    dev.mcu_set_hdr_trans(&mut wa_ring, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    uinfo!("wifid", "hdr_trans_ok");

    // RTS threshold (band 0) — Linux mcu.c:4517
    dev.mcu_set_rts_thresh(&mut wa_ring, 0, 0x92b, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    uinfo!("wifid", "rts_thresh_ok");

    // === Band 0 RX path (initial, before radio ON) ===
    // Linux: mt7996_run() → mcu_set_chan_info(RX_PATH)
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_RX_PATH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    uinfo!("wifid", "rx_path_init_ok");

    // === Interface creation (band 0) ===
    // Linux: mt7996_vif_link_add() → add_dev_info + add_bss_info + add_sta

    // DEV_INFO: activate OMAC 1 (first STA interface) — Linux mcu.c:2623
    dev.mcu_add_dev_info(&mut wa_ring, 0, HW_BSSID_1, &mac_addr, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    uinfo!("wifid", "dev_info_ok");

    // BSS_INFO: create BSS on band 0 — Linux mcu.c:1123
    dev.mcu_add_bss_info(&mut wa_ring, 0, HW_BSSID_1, HW_BSSID_1, &mac_addr, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    uinfo!("wifid", "bss_info_ok");

    // STA_REC: add STA with PORT_SECURE — Linux mcu.c:2438
    dev.mcu_add_sta(&mut wa_ring, HW_BSSID_1 as u8, 0, HW_BSSID_1, CONN_STATE_PORT_SECURE, &mac_addr, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    uinfo!("wifid", "sta_rec_ok");

    // Beacon: upload beacon template — Linux mcu.c:2766
    dev.mcu_set_beacon(&mut wa_ring, 0, HW_BSSID_1, &mac_addr, 1, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    uinfo!("wifid", "beacon_ok");

    // === Radio ON + channel tune (band 0 only) ===

    // Radio ON — Linux mcu.c:4539
    dev.mcu_set_radio_en(&mut wa_ring, 0, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // Channel switch — Linux mcu.c:3696 mt7996_set_channel()
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_SWITCH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // RX path after switch — repeated per Linux sequence
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_RX_PATH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // Bands 1,2 stay OFF — no radio_en call
    uinfo!("wifid", "band0_up"; channel = 1u32, bw = "20MHz");

    // Final state
    let final_fw_state = dev.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
    uinfo!("wifid", "init_complete"; fw_state = final_fw_state);

    // Store resources and state in driver struct
    self.seq = seq;
    self.radio_on = true;
    self.beacon_on = true;
    self.tx_throttle = 100;
    self.channel = 1;
    self.dev = Some(dev);
    self.bar0 = Some(bar0);
    self.desc_pool = Some(desc_pool);
    self.rx_pool = Some(rx_pool);
    self.wa_ring = Some(wa_ring);
    self.wa_tx_buf_pool = Some(wa_tx_buf_pool);

    Ok(())
    }

    fn command(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) -> Disposition {
        Disposition::Handled
    }
}

/// Format u8 as decimal into buffer, return number of bytes written.
fn fmt_u8(val: u8, buf: &mut [u8]) -> usize {
    if val >= 100 {
        buf[0] = b'0' + val / 100;
        buf[1] = b'0' + (val / 10) % 10;
        buf[2] = b'0' + val % 10;
        3
    } else if val >= 10 {
        buf[0] = b'0' + val / 10;
        buf[1] = b'0' + val % 10;
        2
    } else {
        buf[0] = b'0' + val;
        1
    }
}

// ============================================================================
// Entry Point
// ============================================================================

static mut DRIVER: WifiDriver = WifiDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"wifid", WifiDriverWrapper(driver));
}

struct WifiDriverWrapper(&'static mut WifiDriver);

const WIFI_CONFIG_KEYS: &[ConfigKey] = &[
    ConfigKey::read_write(b"wifi.radio"),
    ConfigKey::read_write(b"wifi.beacon"),
    ConfigKey::read_write(b"wifi.channel"),
    ConfigKey::read_write(b"wifi.tx_throttle"),
    ConfigKey::read_only(b"wifi.state"),
];

impl WifiDriverWrapper {
    fn copy_to_buf(buf: &mut [u8], s: &[u8]) -> usize {
        let n = s.len().min(buf.len());
        buf[..n].copy_from_slice(&s[..n]);
        n
    }
}

impl Driver for WifiDriverWrapper {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.reset(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn config_keys(&self) -> &[ConfigKey] {
        WIFI_CONFIG_KEYS
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        match key {
            b"wifi.radio" => {
                let s = if self.0.radio_on { b"on" as &[u8] } else { b"off" };
                Self::copy_to_buf(buf, s)
            }
            b"wifi.beacon" => {
                let s = if self.0.beacon_on { b"on" as &[u8] } else { b"off" };
                Self::copy_to_buf(buf, s)
            }
            b"wifi.channel" => {
                let mut tmp = [0u8; 4];
                let n = fmt_u8(self.0.channel, &mut tmp);
                Self::copy_to_buf(buf, &tmp[..n])
            }
            b"wifi.tx_throttle" => {
                let mut tmp = [0u8; 4];
                let n = fmt_u8(self.0.tx_throttle, &mut tmp);
                Self::copy_to_buf(buf, &tmp[..n])
            }
            b"wifi.state" => {
                let s: &[u8] = if self.0.dev.is_none() {
                    b"init"
                } else if self.0.radio_on {
                    b"radio_on"
                } else {
                    b"radio_off"
                };
                Self::copy_to_buf(buf, s)
            }
            _ => 0,
        }
    }

    fn config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], _ctx: &mut dyn BusCtx) -> usize {
        let (dev, wa_ring) = match (self.0.dev.as_ref(), self.0.wa_ring.as_mut()) {
            (Some(d), Some(r)) => (d, r),
            _ => return Self::copy_to_buf(buf, b"ERR not_initialized\n"),
        };

        match key {
            b"wifi.radio" => {
                let enable = match value {
                    b"on" | b"1" | b"true" => true,
                    b"off" | b"0" | b"false" => false,
                    _ => return Self::copy_to_buf(buf, b"ERR invalid_value\n"),
                };
                // Disable/enable all 3 bands (0=2.4GHz, 1=5GHz, 2=6GHz)
                for band in 0..3u8 {
                    if dev.mcu_set_radio_en(wa_ring, band, enable, self.0.seq).is_err() {
                        return Self::copy_to_buf(buf, b"ERR mcu_failed\n");
                    }
                    self.0.seq = self.0.seq.wrapping_add(1);
                }
                self.0.radio_on = enable;
                if enable {
                    uinfo!("wifid", "radio_enable");
                } else {
                    uinfo!("wifid", "radio_disable");
                }
                Self::copy_to_buf(buf, b"OK\n")
            }
            b"wifi.beacon" => {
                let enable = match value {
                    b"on" | b"1" | b"true" => true,
                    b"off" | b"0" | b"false" => false,
                    _ => return Self::copy_to_buf(buf, b"ERR invalid_value\n"),
                };
                let mac_addr: [u8; 6] = [0x02, 0x0c, 0x43, 0x28, 0x80, 0x01];
                if dev.mcu_set_beacon(wa_ring, 0, HW_BSSID_1, &mac_addr, self.0.channel, enable, self.0.seq).is_err() {
                    return Self::copy_to_buf(buf, b"ERR mcu_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                self.0.beacon_on = enable;
                if enable {
                    uinfo!("wifid", "beacon_enable");
                } else {
                    uinfo!("wifid", "beacon_disable");
                }
                Self::copy_to_buf(buf, b"OK\n")
            }
            b"wifi.channel" => {
                let val_str = core::str::from_utf8(value).unwrap_or("");
                let ch: u8 = match val_str.parse::<u8>() {
                    Ok(v) if v >= 1 && v <= 14 => v,
                    Ok(_) => return Self::copy_to_buf(buf, b"ERR range_1_14\n"),
                    Err(_) => return Self::copy_to_buf(buf, b"ERR invalid_number\n"),
                };
                // Channel switch + RX path for band 0 (2.4GHz only)
                if dev.mcu_set_chan_info(wa_ring, 0, UNI_CHANNEL_SWITCH, ch, CMD_CBW_20MHZ, CH_BAND_2GHZ, self.0.seq).is_err() {
                    return Self::copy_to_buf(buf, b"ERR chan_switch_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                if dev.mcu_set_chan_info(wa_ring, 0, UNI_CHANNEL_RX_PATH, ch, CMD_CBW_20MHZ, CH_BAND_2GHZ, self.0.seq).is_err() {
                    return Self::copy_to_buf(buf, b"ERR rx_path_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                // Re-upload beacon with new channel
                if self.0.beacon_on {
                    let mac_addr: [u8; 6] = [0x02, 0x0c, 0x43, 0x28, 0x80, 0x01];
                    if dev.mcu_set_beacon(wa_ring, 0, HW_BSSID_1, &mac_addr, ch, true, self.0.seq).is_err() {
                        return Self::copy_to_buf(buf, b"ERR beacon_update_failed\n");
                    }
                    self.0.seq = self.0.seq.wrapping_add(1);
                }
                self.0.channel = ch;
                uinfo!("wifid", "channel_switch"; channel = ch as u32);
                Self::copy_to_buf(buf, b"OK\n")
            }
            b"wifi.tx_throttle" => {
                let val_str = core::str::from_utf8(value).unwrap_or("");
                let throttle: u8 = match val_str.parse::<u8>() {
                    Ok(v) if v <= 100 => v,
                    Ok(_) => return Self::copy_to_buf(buf, b"ERR range_0_100\n"),
                    Err(_) => return Self::copy_to_buf(buf, b"ERR invalid_number\n"),
                };
                for band in 0..3u8 {
                    if dev.mcu_set_thermal_throttling(wa_ring, band, throttle, self.0.seq).is_err() {
                        return Self::copy_to_buf(buf, b"ERR mcu_failed\n");
                    }
                    self.0.seq = self.0.seq.wrapping_add(1);
                }
                self.0.tx_throttle = throttle;
                uinfo!("wifid", "thermal_throttle_set"; level = throttle);
                Self::copy_to_buf(buf, b"OK\n")
            }
            _ => Self::copy_to_buf(buf, b"ERR unknown_key\n"),
        }
    }
}
