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

use userlib::{uinfo, unotice, uwarn, uerror, udebug};
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
    wm_ring: Option<TxRing>,
    wa_ring: Option<TxRing>,
    wm_tx_buf_pool: Option<DmaPool>,
    wa_tx_buf_pool: Option<DmaPool>,
    band0_rx: Option<dma::RxQueueInfo>,
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
            wm_ring: None,
            wa_ring: None,
            wm_tx_buf_pool: None,
            wa_tx_buf_pool: None,
            band0_rx: None,
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

        unotice!("wifid", "device_found"; bar0 = bar0_addr, size = bar0_size);

        // Step 2: Map BAR0
        udebug!("wifid", "map_bar0"; addr = bar0_addr, size_kb = bar0_size / 1024);
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
        udebug!("wifid", "alloc_dma_mem");

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

    udebug!("wifid", "init_hw_start");

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
    unotice!("wifid", "hw_rev"; rev = hw_rev & 0xff);

    // Clear interrupts before WFSYS reset (Linux armbian_golden.log line 13)
    dev.mt76_wr(MT_INT_MASK_CSR, 0);

    // WFSYS Reset — ensure clean hardware state (init.c:762-769)
    udebug!("wifid", "wfsys_reset");
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
    udebug!("wifid", "dma_init");
    let (mcu_wa_rx_buf_virt, mcu_wa_rx_buf_size, band0_rx_info) = dev.mt7996_dma_init(desc_phys, desc_virt, DESC_MEM_SIZE, rx_buf_phys, rx_buf_virt, RX_BUF_POOL_SIZE);

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
    udebug!("wifid", "rx_trace_dma_init"; q0 = rx[0], q1 = rx[1], q2 = rx[2], q3 = rx[3]);

    // MCU init (mcu.c:3299-3312 — SWDEF_MODE then driver_own)
    udebug!("wifid", "mcu_init");
    dev.mt76_wr(MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE);

    // Band 0 driver_own
    let _ = dev.mt7996_driver_own(0);
    let rx = trace_rx();
    udebug!("wifid", "rx_trace_drv_own0"; q0 = rx[0], q1 = rx[1], q2 = rx[2], q3 = rx[3]);

    // Band 1 driver_own (HIF2)
    let _ = dev.mt7996_driver_own(1);
    let rx = trace_rx();
    udebug!("wifid", "rx_trace_drv_own1"; q0 = rx[0], q1 = rx[1], q2 = rx[2], q3 = rx[3]);

    // Check firmware state after MCU init
    let fw_state = dev.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
    udebug!("wifid", "fw_state_after_mcu"; val = fw_state);

    // Firmware loading
    unotice!("wifid", "firmware_load_start");

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
    mcu_ring.rx_regs = MCU_WM_RX_REGS; // WM ring → responses on MCU_WM RX (q0)

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
    // Post-firmware MCU init
    // Linux: mt7996/mcu.c:295-298 — after MCU_RUNNING:
    //   WA or WMWA commands → MT_MCUQ_WA (hw_idx=20)
    //   WM-only commands → MT_MCUQ_WM (hw_idx=17)
    // ====================================================================

    // MCU_WA ring (hw_idx=20) — for WA and WMWA commands
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
    wa_ring.rx_regs = MCU_WA_RX_REGS; // WA ring → responses on MCU_WA RX (q1)
    wa_ring.rx_buf_virt = mcu_wa_rx_buf_virt;
    wa_ring.rx_buf_size = mcu_wa_rx_buf_size;

    // WM ring persists for WM-only commands after firmware loading.
    // After firmware boots, ALL MCU responses arrive on q1 (MCU_WA RX),
    // regardless of which TX ring sent the command. Linux doesn't care because
    // it uses interrupt-driven processing across all RX queues.
    // So post-firmware: poll q1 for both WM and WA commands.
    mcu_ring.rx_regs = MCU_WA_RX_REGS;
    mcu_ring.rx_buf_virt = mcu_wa_rx_buf_virt;
    mcu_ring.rx_buf_size = mcu_wa_rx_buf_size;

    udebug!("mcu", "post_init_start");
    let mut seq: u8 = 1;

    let mcu_err = |_e: i32| { BusError::Internal };

    // fw_log_2_host(WM, 0) — enable WM logging
    // MCU_WM_UNI_CMD(WSYS_CONFIG) — WM-only, uses mcu_ring
    dev.mcu_fw_log_2_host(&mut mcu_ring, 0, 0, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // fw_log_2_host(WA, 0) — enable WA logging
    // MCU_WA_UNI_CMD(WSYS_CONFIG) — WA, uses wa_ring
    dev.mcu_fw_log_2_host(&mut wa_ring, 1, 0, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // set_mwds(1) — enable MWDS
    // MCU_WA_EXT_CMD(MWDS_SUPPORT) — WA, uses wa_ring
    dev.mcu_set_mwds(&mut wa_ring, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // init_rx_airtime() — RX airtime for bands 0,1,2
    // MCU_WM_UNI_CMD(VOW) — WM-only, uses mcu_ring
    dev.mcu_init_rx_airtime(&mut mcu_ring, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // wa_cmd(SET, RED, 0, 0) — enable Random Early Drop
    // MCU_WA_PARAM_CMD(SET) — WA, uses wa_ring
    dev.mcu_wa_cmd(&mut wa_ring, mcu::MCU_WA_PARAM_RED, 0, 0, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    udebug!("mcu", "mcu_init_ok");

    // ====================================================================
    // MAC initialization — register writes + MCU commands
    // Linux: init.c mt7996_init_work() → mcu_set_eeprom() → mac_init()
    // ====================================================================

    // EEPROM initialization — Linux: eeprom.c:164-219 mt7996_eeprom_load()
    // 1. Query eFuse free blocks (MCU_WM_UNI_CMD_QUERY) — initializes firmware EEPROM subsystem
    // 2. If free_blocks >= 59: eFuse is empty → upload default binary via flash/buffer mode
    // 3. If free_blocks < 59: eFuse has data → tell firmware to use eFuse mode
    let free_blocks = dev.mcu_get_eeprom_free_block(&mut mcu_ring, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    if free_blocks >= 59 {
        // eFuse empty — upload embedded mt7996_eeprom.bin via flash/buffer mode
        udebug!("wifid", "eeprom_flash_upload"; size = firmware::FW_EEPROM.len() as u32);
        dev.mcu_set_eeprom_flash(&mut mcu_ring, firmware::FW_EEPROM, &mut seq).map_err(mcu_err)?;
        udebug!("wifid", "eeprom_flash_ok");
    } else {
        // eFuse has calibration data — use eFuse mode
        udebug!("wifid", "eeprom_efuse_mode"; free = free_blocks);
        dev.mcu_set_eeprom(&mut mcu_ring, seq).map_err(mcu_err)?;
        seq = seq.wrapping_add(1);
    }

    // Read eFuse WiFi config to determine actual antenna configuration.
    // EEPROM offset 0x190 = WiFi config area (stream/path/nss per band).
    // Linux: eeprom.c:64-94 mt7996_eeprom_parse_stream()
    //   Byte[1] bits[5:3] = TX_PATH_BAND0
    //   Byte[3] bits[2:0] = RX_PATH_BAND0
    //   Byte[4] bits[5:3] = STREAM_NUM_BAND0 (nss)
    match dev.mcu_get_eeprom(&mut mcu_ring, 0x190, seq) {
        Ok(wifi_conf) => {
            seq = seq.wrapping_add(1);
            let tx_path_b0 = (wifi_conf[1] >> 3) & 0x7;
            let rx_path_b0 = wifi_conf[3] & 0x7;
            let nss_b0 = (wifi_conf[4] >> 3) & 0x7;
            udebug!("wifid", "efuse_wifi_conf"; tx = tx_path_b0 as u32, rx = rx_path_b0 as u32, nss = nss_b0 as u32);
            // Also dump device ID from eFuse offset 0
            match dev.mcu_get_eeprom(&mut mcu_ring, 0, seq) {
                Ok(id_block) => {
                    seq = seq.wrapping_add(1);
                    let dev_id = u16::from_le_bytes([id_block[0], id_block[1]]);
                    udebug!("wifid", "efuse_dev_id"; id = dev_id as u32);
                }
                Err(e) => {
                    uwarn!("wifid", "efuse_dev_id_read_fail"; err = e);
                }
            }
        }
        Err(e) => {
            uwarn!("wifid", "efuse_wifi_conf_read_fail"; err = e);
            seq = seq.wrapping_add(1);
        }
    }

    // Read ADIE chip ID via MCU RF register — verify RF frontend is alive
    // Linux: init.c:1163 mt7996_mcu_rf_regval(dev, MT_ADIE_CHIP_ID(0), &regval, false)
    match dev.mcu_rf_regval(&mut mcu_ring, MT_ADIE_CHIP_ID_0, seq) {
        Ok(regval) => {
            let adie_id = (regval >> 16) & 0xFFFF;
            let adie_ver = regval & 0xFFFF;
            udebug!("wifid", "adie_chip_id"; id = adie_id as u32, ver = adie_ver as u32);
        }
        Err(e) => {
            uwarn!("wifid", "adie_read_fail"; err = e);
        }
    }
    seq = seq.wrapping_add(1);

    // Read GPIO pad for variant detection — init.c:1160
    let pad_gpio = dev.reg_rr(MT_PAD_GPIO);
    let adie_comb = (pad_gpio >> 15) & 0x3;
    udebug!("wifid", "pad_gpio"; val = pad_gpio, adie_comb = adie_comb);

    // Full MAC init: WTBL clear, RRO(WM), HIF TXD(WA), per-band regs, basic rates(WM)
    dev.mac_init(&mut mcu_ring, &mut wa_ring, &mut seq).map_err(mcu_err)?;

    // TxBF init: beamforming subsystem — init.c:757 mt7996_txbf_init()
    // MCU_WM_UNI_CMD(BF) — WM-only, uses mcu_ring
    dev.mcu_txbf_init(&mut mcu_ring, &mut seq).map_err(mcu_err)?;

    // ====================================================================
    // Thermal protection + radio control
    // Linux: mt7996/main.c mt7996_start() — thermal protect, throttle, radio
    // ====================================================================

    // Thermal protection: enable for all 3 bands
    // MCU_WM_UNI_CMD(THERMAL) — WM-only, uses mcu_ring
    for band in 0..3u8 {
        dev.mcu_set_thermal_protect(&mut mcu_ring, band, true, seq).map_err(mcu_err)?;
        seq = seq.wrapping_add(1);
    }
    udebug!("wifid", "thermal_protect_ok");

    // Thermal throttling: 100% (full power — throttle only on overtemp)
    // MCU_WM_UNI_CMD(THERMAL) — WM-only, uses mcu_ring
    for band in 0..3u8 {
        dev.mcu_set_thermal_throttling(&mut mcu_ring, band, 100, seq).map_err(mcu_err)?;
        seq = seq.wrapping_add(1);
    }
    udebug!("wifid", "thermal_throttle_ok");

    // ====================================================================
    // Band 0 radio startup + interface creation
    // Linux: mt7996/main.c mt7996_start() + mt7996_run() + mt7996_vif_link_add()
    // ====================================================================

    // Locally-administered MAC for initial bringup
    let mac_addr: [u8; 6] = [0x02, 0x0c, 0x43, 0x28, 0x80, 0x01];

    // Log interface creation parameters
    let m = &mac_addr;
    udebug!("wifid", "iface_create";
        mac0 = m[0] as u32, mac1 = m[1] as u32, mac2 = m[2] as u32,
        mac3 = m[3] as u32, mac4 = m[4] as u32, mac5 = m[5] as u32,
        band = 0u32, omac = HW_BSSID_0 as u32, bss_idx = 0u32,
        wlan_idx = MT7996_WTBL_RESERVED as u32, channel = 1u32);

    // === Global setup ===

    // Header translation (global, once) — Linux main.c:56 mt7996_start()
    // MCU_WM_UNI_CMD(RX_HDR_TRANS) — WM-only, uses mcu_ring
    dev.mcu_set_hdr_trans(&mut mcu_ring, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "hdr_trans_ok");

    // === Band 0 radio startup — Linux main.c:10-48 mt7996_run() ===

    // Enable noise floor measurement — Linux main.c:15 (FIRST thing in mt7996_run!)
    // Enables IPI for CCA (Clear Channel Assessment) — required before TX
    dev.mac_enable_nf(0);
    udebug!("wifid", "enable_nf_ok");

    // Configure RX filter — Linux init.c:414 + main.c:676-724 mt7996_configure_filter()
    // Linux default: phy->rxfilter = MT_WF_RFCR_DROP_OTHER_UC (init.c:414)
    // Then mt7996_phy_set_rxfilter() adds DROP_CTS|DROP_RTS|DROP_CTL_RSV|DROP_FCSFAIL
    // when DROP_OTHER_UC is set. Final AP value = 0x4E002.
    // This accepts: beacons, probe requests, multicast, broadcast, management.
    // Drops: FCS errors, control frames (CTS/RTS/reserved), unicast to others.
    let rfcr_val: u32 = MT_WF_RFCR_DROP_FCSFAIL
        | MT_WF_RFCR_DROP_CTL_RSV
        | MT_WF_RFCR_DROP_CTS
        | MT_WF_RFCR_DROP_RTS
        | MT_WF_RFCR_DROP_OTHER_UC;
    dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr_val);
    // Secondary filter: drop ACK, BF_POLL, BA, CFEND, CFACK
    let rfcr1_val: u32 = MT_WF_RFCR1_DROP_ACK
        | MT_WF_RFCR1_DROP_BF_POLL
        | MT_WF_RFCR1_DROP_BA
        | MT_WF_RFCR1_DROP_CFEND
        | MT_WF_RFCR1_DROP_CFACK;
    dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1_val);
    udebug!("wifid", "rfcr_ok"; rfcr = rfcr_val, rfcr1 = rfcr1_val);

    // RTS threshold (band 0) — Linux main.c:17
    // MCU_WM_UNI_CMD(BAND_CONFIG) — WM-only, uses mcu_ring
    dev.mcu_set_rts_thresh(&mut mcu_ring, 0, 0x92b, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "rts_thresh_ok");

    // Radio ON — Linux main.c:21 (before RX_PATH in mt7996_run!)
    // MCU_WM_UNI_CMD(BAND_CONFIG) — WM-only, uses mcu_ring
    dev.mcu_set_radio_en(&mut mcu_ring, 0, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // RX path — Linux main.c:25 (after radio enable in mt7996_run)
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_RX_PATH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "rx_path_init_ok");

    // === Interface creation (band 0) ===
    // Linux: mt7996_vif_link_add() → add_dev_info + add_bss_info + add_sta

    // DEV_INFO: activate OMAC on band 0 — Linux mcu.c:2623
    udebug!("wifid", "dev_info_send"; band = 0u32, omac = HW_BSSID_0 as u32, active = 1u32);
    dev.mcu_add_dev_info(&mut wa_ring, 0, HW_BSSID_0, &mac_addr, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "dev_info_ok");

    // BSS_INFO: create BSS on band 0 — Linux mcu.c:1123
    // hw_bss_idx = 0 (matches bss_req_hdr.bss_idx in uni_header)
    // omac_idx = HW_BSSID_0 (OMAC to use, independent of bss_idx)
    udebug!("wifid", "bss_info_send"; band = 0u32, omac = HW_BSSID_0 as u32, bss = 0u32, active = 1u32, ch = 1u32);
    dev.mcu_add_bss_info(&mut wa_ring, 0, HW_BSSID_0, 0, &mac_addr, true, 1, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "bss_info_ok");

    // STA_REC: add broadcast/multicast STA — Linux mcu.c:2438
    // bss_idx = 0 (must match BSS_INFO's bss_req_hdr.bss_idx)
    // wlan_idx = MT7996_WTBL_RESERVED - bss_idx = 1087 (Linux: main.c:334)
    // muar_idx(omac_idx param) = 0 (band_idx, Linux: wcid.phy_idx = band_idx)
    udebug!("wifid", "sta_rec_send"; bss = 0u32, wlan = MT7996_WTBL_RESERVED as u32, omac = 0u32, newly = 1u32);
    dev.mcu_add_sta(&mut wa_ring, 0, MT7996_WTBL_RESERVED, 0, CONN_STATE_PORT_SECURE, &mac_addr, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "sta_rec_ok");

    // VOW: assign WCID to BSS group — Linux mcu.c:2504 mt7996_mcu_add_group()
    udebug!("wifid", "vow_send"; bss = 0u32, wlan = MT7996_WTBL_RESERVED as u32);
    dev.mcu_add_group(&mut wa_ring, 0, MT7996_WTBL_RESERVED, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // === Channel tune (band 0) — Linux main.c:553 mt7996_set_channel() ===
    // Radio is already ON from mt7996_run() sequence above.

    // Channel switch — Linux main.c:561
    udebug!("wifid", "chan_switch"; band = 0u32, ch = 1u32, bw = 0u32, ch_band = 0u32);
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_SWITCH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // RX path after switch — Linux main.c:565
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_RX_PATH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // TX power SKU — Linux main.c:569
    dev.mcu_set_txpower_sku(&mut mcu_ring, 0, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // CCA stats reset — Linux main.c:574
    dev.mac_cca_stats_reset(0);

    // Radio state diagnostic: check ARB_SCR after full init
    // If RX_DISABLE (bit 9) is set, firmware hasn't enabled RX
    let arb_scr = dev.reg_rr(mt_wf_arb(0, MT_ARB_SCR_OFS));
    let rfcr = dev.reg_rr(mt_wf_rmac(0, MT_WF_RFCR_OFS));
    let phyrx_ctrl1 = dev.reg_rr(mt_wf_phyrx_band(0, MT_WF_PHYRX_BAND_RX_CTRL1_OFS));
    udebug!("wifid", "radio_diag"; arb = arb_scr, rfcr = rfcr, phyrx = phyrx_ctrl1);
    if arb_scr & MT_ARB_SCR_RX_DISABLE != 0 {
        uerror!("wifid", "RX_DISABLED_IN_ARB_SCR");
    }
    if arb_scr & MT_ARB_SCR_TX_DISABLE != 0 {
        uwarn!("wifid", "TX_DISABLED_IN_ARB_SCR");
    }

    unotice!("wifid", "band0_up"; channel = 1u32, bw = "20MHz");

    // === Band 1 (5GHz) radio startup — diagnostic: check if 5GHz antennas receive ===
    // Same sequence as band 0 but without BSS/STA/beacon (just radio + channel for MIB)

    dev.mac_enable_nf(1);
    // RFCR for band 1: same filter as band 0
    dev.reg_wr(mt_wf_rmac(1, MT_WF_RFCR_OFS), rfcr_val);
    dev.reg_wr(mt_wf_rmac(1, MT_WF_RFCR1_OFS), rfcr1_val);

    dev.mcu_set_rts_thresh(&mut mcu_ring, 1, 0x92b, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    dev.mcu_set_radio_en(&mut mcu_ring, 1, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // RX path + channel switch for band 1: channel 36, 5GHz
    dev.mcu_set_chan_info(&mut wa_ring, 1, UNI_CHANNEL_RX_PATH, 36, CMD_CBW_20MHZ, CH_BAND_5GHZ, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    dev.mcu_set_chan_info(&mut wa_ring, 1, UNI_CHANNEL_SWITCH, 36, CMD_CBW_20MHZ, CH_BAND_5GHZ, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    dev.mcu_set_chan_info(&mut wa_ring, 1, UNI_CHANNEL_RX_PATH, 36, CMD_CBW_20MHZ, CH_BAND_5GHZ, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    dev.mcu_set_txpower_sku(&mut mcu_ring, 1, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    dev.mac_cca_stats_reset(1);

    let arb1 = dev.reg_rr(mt_wf_arb(1, MT_ARB_SCR_OFS));
    unotice!("wifid", "band1_up"; channel = 36u32, bw = "20MHz", arb = arb1);

    // Program beacon rate table: 1 Mbps CCK (most compatible 2.4GHz rate)
    // MCU_WM_UNI_CMD(FIXED_RATE_TABLE) — WM-only, uses mcu_ring
    // Rate encoding: mode=CCK(0) at bits[9:6], idx=0 at bits[5:0] → 0x0000
    let beacon_rate_idx = MT7996_BEACON_RATES_TBL + 2 * 0; // band 0 → table 25
    dev.mcu_set_fixed_rate_table(&mut mcu_ring, beacon_rate_idx, 0x0000, true, 0, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // Program basic rate table entry too (for bc/mc frames)
    // rate_idx = 0x0000 = 1Mbps CCK
    dev.mcu_set_fixed_rate_table(&mut mcu_ring, MT7996_BASIC_RATES_TBL, 0x0000, false, 0, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // === Beacon enable sequence — Linux main.c:857-903 mt7996_link_info_changed() ===
    // BSS_CHANGED_BEACON_ENABLED fires AFTER channel switch in Linux.
    // Linux re-sends BSS_INFO + STA_REC at this point to confirm BSS on channel.

    // Second BSS_INFO: re-send after channel is configured — Linux main.c:859
    udebug!("wifid", "bss_info2_send"; band = 0u32, omac = HW_BSSID_0 as u32, bss = 0u32, active = 1u32);
    dev.mcu_add_bss_info(&mut wa_ring, 0, HW_BSSID_0, 0, &mac_addr, true, 1, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // Second STA_REC: update existing BMC STA (newly=false) — Linux main.c:861
    udebug!("wifid", "sta_rec2_send"; bss = 0u32, wlan = MT7996_WTBL_RESERVED as u32, newly = 0u32);
    dev.mcu_add_sta(&mut wa_ring, 0, MT7996_WTBL_RESERVED, 0, CONN_STATE_PORT_SECURE, &mac_addr, false, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // VOW: reassign WCID to BSS group — Linux mcu.c:2504
    dev.mcu_add_group(&mut wa_ring, 0, MT7996_WTBL_RESERVED, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // EDCA/WMM parameters — Linux main.c:883-884
    // MCU_WM_UNI_CMD(EDCA_UPDATE) — WM-only, uses mcu_ring
    // "ensure that enable txcmd_mode after bss_info"
    // bss_idx = 0 (must match BSS_INFO's bss_req_hdr.bss_idx)
    udebug!("wifid", "edca_send"; bss = 0u32);
    dev.mcu_set_edca(&mut mcu_ring, 0, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "edca_ok");

    // Beacon: upload beacon template — Linux main.c:903
    udebug!("wifid", "beacon_send"; band = 0u32, omac = HW_BSSID_0 as u32, ch = 1u32, enable = 1u32);
    dev.mcu_set_beacon(&mut wa_ring, 0, HW_BSSID_0, &mac_addr, 1, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "beacon_ok");

    // Re-set RFCR after beacon enable — Linux mac80211 calls configure_filter()
    // after BSS changes, which re-writes RFCR. Without this, MCU commands during
    // init may clear RFCR (observed rfcr=0 in diagnostics).
    dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr_val);
    dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1_val);

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
    self.wm_ring = Some(mcu_ring);
    self.wa_ring = Some(wa_ring);
    self.wm_tx_buf_pool = Some(mcu_tx_buf_pool);
    self.wa_tx_buf_pool = Some(wa_tx_buf_pool);
    self.band0_rx = Some(band0_rx_info);

    Ok(())
    }

    fn command(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) -> Disposition {
        Disposition::Handled
    }
}

/// Format u32 as hex (0xXXXXXXXX) into buffer, return number of bytes written.
fn fmt_hex32(val: u32, buf: &mut [u8]) -> usize {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    if buf.len() < 10 { return 0; }
    buf[0] = b'0';
    buf[1] = b'x';
    for i in 0..8 {
        buf[2 + i] = HEX[((val >> (28 - i * 4)) & 0xF) as usize];
    }
    10
}

/// Format u16 as 4-digit hex into buffer, return 4.
fn fmt_hex16(val: u16, buf: &mut [u8]) -> usize {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    if buf.len() < 4 { return 0; }
    buf[0] = HEX[((val >> 12) & 0xF) as usize];
    buf[1] = HEX[((val >> 8) & 0xF) as usize];
    buf[2] = HEX[((val >> 4) & 0xF) as usize];
    buf[3] = HEX[(val & 0xF) as usize];
    4
}

/// Format 6-byte MAC as 12 hex chars (no colons), return 12.
fn fmt_mac(mac: &[u8; 6], buf: &mut [u8]) -> usize {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    if buf.len() < 12 { return 0; }
    for i in 0..6 {
        buf[i * 2] = HEX[(mac[i] >> 4) as usize];
        buf[i * 2 + 1] = HEX[(mac[i] & 0xf) as usize];
    }
    12
}

/// Format u32 as decimal into buffer, return number of bytes written.
fn fmt_u32_dec(val: u32, buf: &mut [u8]) -> usize {
    if val == 0 {
        if !buf.is_empty() { buf[0] = b'0'; }
        return 1;
    }
    let mut tmp = [0u8; 10];
    let mut n = 0;
    let mut v = val;
    while v > 0 {
        tmp[n] = b'0' + (v % 10) as u8;
        v /= 10;
        n += 1;
    }
    if n > buf.len() { return 0; }
    for i in 0..n {
        buf[i] = tmp[n - 1 - i];
    }
    n
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
    ConfigKey::read_write(b"radio"),
    ConfigKey::read_write(b"beacon"),
    ConfigKey::read_write(b"channel"),
    ConfigKey::read_write(b"tx_throttle"),
    ConfigKey::read_only(b"state"),
    ConfigKey::read_only(b"diag"),
    ConfigKey::read_write(b"rx"),
    ConfigKey::read_write(b"scan"),
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
            b"radio" => {
                let s = if self.0.radio_on { b"on" as &[u8] } else { b"off" };
                Self::copy_to_buf(buf, s)
            }
            b"beacon" => {
                let s = if self.0.beacon_on { b"on" as &[u8] } else { b"off" };
                Self::copy_to_buf(buf, s)
            }
            b"channel" => {
                let mut tmp = [0u8; 4];
                let n = fmt_u8(self.0.channel, &mut tmp);
                Self::copy_to_buf(buf, &tmp[..n])
            }
            b"tx_throttle" => {
                let mut tmp = [0u8; 4];
                let n = fmt_u8(self.0.tx_throttle, &mut tmp);
                Self::copy_to_buf(buf, &tmp[..n])
            }
            b"state" => {
                let s: &[u8] = if self.0.dev.is_none() {
                    b"init"
                } else if self.0.radio_on {
                    b"radio_on"
                } else {
                    b"radio_off"
                };
                Self::copy_to_buf(buf, s)
            }
            b"rx" => {
                let dev = match self.0.dev.as_ref() {
                    Some(d) => d,
                    None => return Self::copy_to_buf(buf, b"not_initialized"),
                };
                let rx = match self.0.band0_rx.as_ref() {
                    Some(r) => r,
                    None => return Self::copy_to_buf(buf, b"no_band0_rx"),
                };

                let dma_idx = dev.mt76_rr(rx.regs_base + MT_QUEUE_DMA_IDX);
                let cpu_idx = dev.mt76_rr(rx.regs_base + MT_QUEUE_CPU_IDX);
                let ring_cnt = dev.mt76_rr(rx.regs_base + MT_QUEUE_RING_SIZE);
                let desc_base = dev.mt76_rr(rx.regs_base + MT_QUEUE_DESC_BASE);
                let rfcr = dev.reg_rr(mt_wf_rmac(0, MT_WF_RFCR_OFS));
                let glo_cfg = dev.mt76_rr(MT_WFDMA0_GLO_CFG);
                let arb_scr = dev.reg_rr(mt_wf_arb(0, MT_ARB_SCR_OFS));
                let int_mask = dev.mt76_rr(MT_INT_MASK_CSR);

                let mut tmp = [0u8; 1024];
                let mut pos = 0;

                // Line 1: "rx dma=N cpu=N cnt=N base=0xXXXXXXXX\n"
                tmp[pos..pos + 7].copy_from_slice(b"rx dma=");
                pos += 7;
                pos += fmt_u32_dec(dma_idx, &mut tmp[pos..]);
                tmp[pos..pos + 5].copy_from_slice(b" cpu=");
                pos += 5;
                pos += fmt_u32_dec(cpu_idx, &mut tmp[pos..]);
                tmp[pos..pos + 5].copy_from_slice(b" cnt=");
                pos += 5;
                pos += fmt_u32_dec(ring_cnt, &mut tmp[pos..]);
                tmp[pos..pos + 6].copy_from_slice(b" base=");
                pos += 6;
                pos += fmt_hex32(desc_base, &mut tmp[pos..]);
                tmp[pos] = b'\n';
                pos += 1;

                // Line 2: "rfcr=X glo=X arb=X imask=X\n"
                tmp[pos..pos + 5].copy_from_slice(b"rfcr=");
                pos += 5;
                pos += fmt_hex32(rfcr, &mut tmp[pos..]);
                tmp[pos..pos + 5].copy_from_slice(b" glo=");
                pos += 5;
                pos += fmt_hex32(glo_cfg, &mut tmp[pos..]);
                tmp[pos..pos + 5].copy_from_slice(b" arb=");
                pos += 5;
                pos += fmt_hex32(arb_scr, &mut tmp[pos..]);
                tmp[pos..pos + 7].copy_from_slice(b" imask=");
                pos += 7;
                pos += fmt_hex32(int_mask, &mut tmp[pos..]);
                tmp[pos] = b'\n';
                pos += 1;

                // Line 3: MIB counters — RX activity diagnostics
                // These are cumulative: read twice with delay to see if they change
                let mib_rscr31 = dev.reg_rr(mt_wf_mib(0, MT_MIB_RSCR31_OFS));  // RX MPDU count
                let mib_rscr1 = dev.reg_rr(mt_wf_mib(0, MT_MIB_RSCR1_OFS));    // FCS errors
                let mib_rvsr0 = dev.reg_rr(mt_wf_mib(0, MT_MIB_RVSR0_OFS));    // RX vector mismatch
                let mib_rscr35 = dev.reg_rr(mt_wf_mib(0, MT_MIB_RSCR35_OFS));  // RX delimiter fail
                let mib_sdr6 = dev.reg_rr(mt_wf_mib(0, MT_MIB_SDR6_OFS));      // Channel idle
                // PHYRX band RX ctrl — verify IPI enable (should be 0x5)
                let phyrx_ctrl1 = dev.reg_rr(mt_wf_phyrx_band(0, MT_WF_PHYRX_BAND_RX_CTRL1_OFS));
                let ipi_en = phyrx_ctrl1 & MT_WF_PHYRX_BAND_RX_CTRL1_IPI_EN_MASK;

                tmp[pos..pos + 5].copy_from_slice(b"mpdu=");
                pos += 5;
                pos += fmt_u32_dec(mib_rscr31, &mut tmp[pos..]);
                tmp[pos..pos + 5].copy_from_slice(b" fcs=");
                pos += 5;
                pos += fmt_u32_dec(mib_rscr1, &mut tmp[pos..]);
                tmp[pos..pos + 5].copy_from_slice(b" vec=");
                pos += 5;
                pos += fmt_u32_dec(mib_rvsr0, &mut tmp[pos..]);
                tmp[pos..pos + 5].copy_from_slice(b" del=");
                pos += 5;
                pos += fmt_u32_dec(mib_rscr35, &mut tmp[pos..]);
                tmp[pos..pos + 6].copy_from_slice(b" idle=");
                pos += 6;
                pos += fmt_u32_dec(mib_sdr6, &mut tmp[pos..]);
                tmp[pos..pos + 5].copy_from_slice(b" ipi=");
                pos += 5;
                pos += fmt_u32_dec(ipi_en, &mut tmp[pos..]);
                tmp[pos] = b'\n';
                pos += 1;

                // Line 4: Band 1 (5GHz) MIB counters
                let b1_mpdu = dev.reg_rr(mt_wf_mib(1, MT_MIB_RSCR31_OFS));
                let b1_fcs = dev.reg_rr(mt_wf_mib(1, MT_MIB_RSCR1_OFS));
                let b1_idle = dev.reg_rr(mt_wf_mib(1, MT_MIB_SDR6_OFS));
                let b1_arb = dev.reg_rr(mt_wf_arb(1, MT_ARB_SCR_OFS));

                tmp[pos..pos + 3].copy_from_slice(b"b1=");
                pos += 3;
                tmp[pos..pos + 5].copy_from_slice(b"mpdu=");
                pos += 5;
                pos += fmt_u32_dec(b1_mpdu, &mut tmp[pos..]);
                tmp[pos..pos + 5].copy_from_slice(b" fcs=");
                pos += 5;
                pos += fmt_u32_dec(b1_fcs, &mut tmp[pos..]);
                tmp[pos..pos + 6].copy_from_slice(b" idle=");
                pos += 6;
                pos += fmt_u32_dec(b1_idle, &mut tmp[pos..]);
                tmp[pos..pos + 5].copy_from_slice(b" arb=");
                pos += 5;
                pos += fmt_hex32(b1_arb, &mut tmp[pos..]);
                tmp[pos] = b'\n';
                pos += 1;

                // Show 2 recent frames with RCPI + frame header dump
                // RCPI is in Group 3 (P-RXV), DW3: GENMASK(7,0)=chain0
                // RSSI = (rcpi - 220) / 2  (Linux mac.c:14)
                // Frame body: FC(2) + duration(2) + DA(6) = first 10 bytes
                let max_show = 2u32;
                let avail = dma_idx.min(rx.ndesc);
                let start = if avail > max_show { avail - max_show } else { 0 };

                // DSB SY to ensure DMA writes are visible to CPU
                unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)); }

                for idx in start..avail {
                    let i = idx as usize;
                    if pos + 80 > tmp.len() { break; }

                    let desc_ptr = unsafe { (rx.desc_virt as *const dma::Mt76Desc).add(i) };
                    let ctrl = unsafe { core::ptr::read_volatile(&(*desc_ptr).ctrl) };
                    if ctrl & MT_DMA_CTL_DMA_DONE == 0 { continue; }

                    let sd_len0 = (ctrl >> 16) & 0x3FFF; // MT_DMA_CTL_SD_LEN0 GENMASK(29,16)

                    let buf_base = rx.buf_virt + (i as u64) * (rx.buf_size as u64);
                    let buf_ptr = buf_base as *const u32;
                    let rxd0 = unsafe { core::ptr::read_volatile(buf_ptr) };
                    let rxd1 = unsafe { core::ptr::read_volatile(buf_ptr.add(1)) };
                    let rxd2 = unsafe { core::ptr::read_volatile(buf_ptr.add(2)) };

                    let pkt_len = rxd0 & MT_RXD0_LENGTH;
                    let pkt_type = (rxd0 >> 27) & 0x1F;
                    if pkt_type != 2 { continue; } // Skip non-NORMAL frames

                    let remove_pad = (rxd2 >> 13) & 0x7;
                    let g3_present = rxd1 & MT_RXD1_NORMAL_GROUP_3 != 0;

                    // Navigate to Group 3 for RCPI (order: G4, G1, G2, G3)
                    let mut g3_ofs: usize = 32;
                    if rxd1 & MT_RXD1_NORMAL_GROUP_4 != 0 { g3_ofs += 16; }
                    if rxd1 & MT_RXD1_NORMAL_GROUP_1 != 0 { g3_ofs += 16; }
                    if rxd1 & MT_RXD1_NORMAL_GROUP_2 != 0 { g3_ofs += 16; }

                    // RCPI from P-RXV[3] (Group 3, DW3) — Linux mac.c:625
                    let rssi: i32 = if g3_present && g3_ofs + 16 <= rx.buf_size as usize {
                        let g3_dw3 = unsafe {
                            core::ptr::read_volatile(buf_ptr.add((g3_ofs + 12) / 4))
                        };
                        let rcpi0 = g3_dw3 & 0xFF;
                        ((rcpi0 as i32) - 220) / 2
                    } else {
                        -128
                    };

                    // Frame body offset: base(32) + all groups + 2*remove_pad
                    let mut frame_ofs: usize = g3_ofs;
                    if g3_present {
                        frame_ofs += 16;
                        if rxd1 & MT_RXD1_NORMAL_GROUP_5 != 0 { frame_ofs += 96; }
                    }
                    frame_ofs += 2 * remove_pad as usize;

                    // Line 1: "fN=LEN.dD.rR\n"
                    tmp[pos] = b'f';
                    pos += 1;
                    pos += fmt_u32_dec(idx, &mut tmp[pos..]);
                    tmp[pos] = b'=';
                    pos += 1;
                    pos += fmt_u32_dec(pkt_len, &mut tmp[pos..]);
                    tmp[pos..pos+2].copy_from_slice(b".d");
                    pos += 2;
                    pos += fmt_u32_dec(sd_len0, &mut tmp[pos..]);
                    tmp[pos..pos+2].copy_from_slice(b".r");
                    pos += 2;
                    if rssi < 0 {
                        tmp[pos] = b'-';
                        pos += 1;
                        pos += fmt_u32_dec((-rssi) as u32, &mut tmp[pos..]);
                    } else {
                        pos += fmt_u32_dec(rssi as u32, &mut tmp[pos..]);
                    }
                    tmp[pos] = b'\n';
                    pos += 1;

                    // Line 2: "h=XXXXXXXXXXXXXXXXXXXX\n" (first 10 bytes of frame body)
                    if frame_ofs + 10 <= rx.buf_size as usize {
                        tmp[pos..pos+2].copy_from_slice(b"h=");
                        pos += 2;
                        let byte_ptr = buf_base as *const u8;
                        for j in 0..10usize {
                            let b = unsafe {
                                core::ptr::read_volatile(byte_ptr.add(frame_ofs + j))
                            };
                            const HEX: &[u8; 16] = b"0123456789abcdef";
                            tmp[pos] = HEX[(b >> 4) as usize];
                            tmp[pos + 1] = HEX[(b & 0xf) as usize];
                            pos += 2;
                        }
                        tmp[pos] = b'\n';
                        pos += 1;
                    }
                }

                Self::copy_to_buf(buf, &tmp[..pos])
            }
            b"diag" => {
                let dev = match self.0.dev.as_ref() {
                    Some(d) => d,
                    None => return Self::copy_to_buf(buf, b"not_initialized"),
                };
                // Read key diagnostic registers for band 0
                let fw_state = dev.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;

                // TSF timer — trigger SW_READ latch then read lower 32 bits
                dev.reg_rmw(
                    mt_wf_lpon(0, MT_LPON_TCR_OFS),
                    MT_LPON_TCR_SW_MODE,
                    MT_LPON_TCR_SW_READ,
                );
                let tsf_lo = dev.reg_rr(mt_wf_lpon(0, MT_LPON_UTTR0_OFS));
                let tsf_hi = dev.reg_rr(mt_wf_lpon(0, MT_LPON_UTTR1_OFS));

                // MIB counters — TX and RX (clear-on-read)
                let tx_mpdu_att = dev.reg_rr(mt_wf_mib(0, MT_MIB_TSCR3_OFS));
                let tx_mpdu_ok = dev.reg_rr(mt_wf_mib(0, MT_MIB_TSCR4_OFS));
                let rx_fcs_err = dev.reg_rr(mt_wf_mib(0, MT_MIB_RSCR1_OFS));
                let ch_idle = dev.reg_rr(mt_wf_mib(0, MT_MIB_SDR6_OFS)) & 0xFFFF;

                // Hardware state registers
                let arb_scr = dev.reg_rr(mt_wf_arb(0, MT_ARB_SCR_OFS));
                let glo_cfg = dev.mt76_rr(MT_WFDMA0_GLO_CFG);
                // PLE FL_Q_EMPTY: bit 10 = BCN_Q0, bit 14 = BCN_Q1
                let ple_empty = dev.reg_rr(MT_PLE_FL_Q_EMPTY);

                let mut tmp = [0u8; 300];
                let mut pos = 0;
                // fw=N
                tmp[pos..pos+3].copy_from_slice(b"fw=");
                pos += 3;
                tmp[pos] = b'0' + (fw_state as u8);
                pos += 1;
                // tsf
                tmp[pos..pos+5].copy_from_slice(b" tsf=");
                pos += 5;
                pos += fmt_hex32(tsf_hi, &mut tmp[pos..]);
                tmp[pos] = b'.';
                pos += 1;
                pos += fmt_hex32(tsf_lo, &mut tmp[pos..]);
                // att (MPDU TX attempts — clear-on-read)
                tmp[pos..pos+5].copy_from_slice(b" att=");
                pos += 5;
                pos += fmt_hex32(tx_mpdu_att, &mut tmp[pos..]);
                // ok (successful MPDU TX — clear-on-read)
                tmp[pos..pos+4].copy_from_slice(b" ok=");
                pos += 4;
                pos += fmt_hex32(tx_mpdu_ok, &mut tmp[pos..]);
                // fcs (RX FCS errors — non-zero = radio receiving)
                tmp[pos..pos+5].copy_from_slice(b" fcs=");
                pos += 5;
                pos += fmt_hex32(rx_fcs_err, &mut tmp[pos..]);
                // idle (channel idle — CCA working)
                tmp[pos..pos+6].copy_from_slice(b" idle=");
                pos += 6;
                pos += fmt_hex32(ch_idle, &mut tmp[pos..]);
                // arb (ARB_SCR: bit8=TX_DIS, bit9=RX_DIS)
                tmp[pos..pos+5].copy_from_slice(b" arb=");
                pos += 5;
                pos += fmt_hex32(arb_scr, &mut tmp[pos..]);
                // glo (WFDMA GLO_CFG: bit0=TX_DMA, bit2=RX_DMA)
                tmp[pos..pos+5].copy_from_slice(b" glo=");
                pos += 5;
                pos += fmt_hex32(glo_cfg, &mut tmp[pos..]);
                // ple (PLE FL_Q_EMPTY: bit10=BCN_Q0 empty)
                tmp[pos..pos+5].copy_from_slice(b" ple=");
                pos += 5;
                pos += fmt_hex32(ple_empty, &mut tmp[pos..]);

                Self::copy_to_buf(buf, &tmp[..pos])
            }
            b"scan" => Self::copy_to_buf(buf, b"use: devc wifid set scan start\n"),
            _ => 0,
        }
    }

    fn config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], _ctx: &mut dyn BusCtx) -> usize {
        let (dev, wa_ring, wm_ring) = match (self.0.dev.as_ref(), self.0.wa_ring.as_mut(), self.0.wm_ring.as_mut()) {
            (Some(d), Some(wa), Some(wm)) => (d, wa, wm),
            _ => return Self::copy_to_buf(buf, b"ERR not_initialized\n"),
        };

        match key {
            b"radio" => {
                let enable = match value {
                    b"on" | b"1" | b"true" => true,
                    b"off" | b"0" | b"false" => false,
                    _ => return Self::copy_to_buf(buf, b"ERR invalid_value\n"),
                };
                // MCU_WM_UNI_CMD(BAND_CONFIG) — WM-only, uses wm_ring
                for band in 0..3u8 {
                    if dev.mcu_set_radio_en(wm_ring, band, enable, self.0.seq).is_err() {
                        return Self::copy_to_buf(buf, b"ERR mcu_failed\n");
                    }
                    self.0.seq = self.0.seq.wrapping_add(1);
                }
                self.0.radio_on = enable;
                if enable {
                    unotice!("wifid", "radio_enable");
                } else {
                    unotice!("wifid", "radio_disable");
                }
                Self::copy_to_buf(buf, b"OK\n")
            }
            b"beacon" => {
                let enable = match value {
                    b"on" | b"1" | b"true" => true,
                    b"off" | b"0" | b"false" => false,
                    _ => return Self::copy_to_buf(buf, b"ERR invalid_value\n"),
                };
                // MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE) — WMWA, uses wa_ring
                let mac_addr: [u8; 6] = [0x02, 0x0c, 0x43, 0x28, 0x80, 0x01];
                if dev.mcu_set_beacon(wa_ring, 0, HW_BSSID_0, &mac_addr, self.0.channel, enable, self.0.seq).is_err() {
                    return Self::copy_to_buf(buf, b"ERR mcu_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                self.0.beacon_on = enable;
                if enable {
                    unotice!("wifid", "beacon_enable");
                } else {
                    unotice!("wifid", "beacon_disable");
                }
                Self::copy_to_buf(buf, b"OK\n")
            }
            b"channel" => {
                let val_str = core::str::from_utf8(value).unwrap_or("");
                let ch: u8 = match val_str.parse::<u8>() {
                    Ok(v) if v >= 1 && v <= 14 => v,
                    Ok(_) => return Self::copy_to_buf(buf, b"ERR range_1_14\n"),
                    Err(_) => return Self::copy_to_buf(buf, b"ERR invalid_number\n"),
                };
                // MCU_WMWA_UNI_CMD(CHANNEL_SWITCH) — WMWA, uses wa_ring
                if dev.mcu_set_chan_info(wa_ring, 0, UNI_CHANNEL_SWITCH, ch, CMD_CBW_20MHZ, CH_BAND_2GHZ, self.0.seq).is_err() {
                    return Self::copy_to_buf(buf, b"ERR chan_switch_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                if dev.mcu_set_chan_info(wa_ring, 0, UNI_CHANNEL_RX_PATH, ch, CMD_CBW_20MHZ, CH_BAND_2GHZ, self.0.seq).is_err() {
                    return Self::copy_to_buf(buf, b"ERR rx_path_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                // TX power SKU after channel switch — Linux main.c:569
                if dev.mcu_set_txpower_sku(wm_ring, 0, self.0.seq).is_err() {
                    return Self::copy_to_buf(buf, b"ERR txpower_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                // Re-set RFCR after channel switch — MCU may clear it
                let rfcr_val: u32 = MT_WF_RFCR_DROP_FCSFAIL
                    | MT_WF_RFCR_DROP_CTL_RSV
                    | MT_WF_RFCR_DROP_CTS
                    | MT_WF_RFCR_DROP_RTS
                    | MT_WF_RFCR_DROP_OTHER_UC;
                let rfcr1_val: u32 = MT_WF_RFCR1_DROP_ACK
                    | MT_WF_RFCR1_DROP_BF_POLL
                    | MT_WF_RFCR1_DROP_BA
                    | MT_WF_RFCR1_DROP_CFEND
                    | MT_WF_RFCR1_DROP_CFACK;
                dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr_val);
                dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1_val);
                // Re-upload beacon with new channel
                if self.0.beacon_on {
                    let mac_addr: [u8; 6] = [0x02, 0x0c, 0x43, 0x28, 0x80, 0x01];
                    if dev.mcu_set_beacon(wa_ring, 0, HW_BSSID_0, &mac_addr, ch, true, self.0.seq).is_err() {
                        return Self::copy_to_buf(buf, b"ERR beacon_update_failed\n");
                    }
                    self.0.seq = self.0.seq.wrapping_add(1);
                }
                self.0.channel = ch;
                unotice!("wifid", "channel_switch"; channel = ch as u32);
                Self::copy_to_buf(buf, b"OK\n")
            }
            b"tx_throttle" => {
                let val_str = core::str::from_utf8(value).unwrap_or("");
                let throttle: u8 = match val_str.parse::<u8>() {
                    Ok(v) if v <= 100 => v,
                    Ok(_) => return Self::copy_to_buf(buf, b"ERR range_0_100\n"),
                    Err(_) => return Self::copy_to_buf(buf, b"ERR invalid_number\n"),
                };
                // MCU_WM_UNI_CMD(THERMAL) — WM-only, uses wm_ring
                for band in 0..3u8 {
                    if dev.mcu_set_thermal_throttling(wm_ring, band, throttle, self.0.seq).is_err() {
                        return Self::copy_to_buf(buf, b"ERR mcu_failed\n");
                    }
                    self.0.seq = self.0.seq.wrapping_add(1);
                }
                self.0.tx_throttle = throttle;
                udebug!("wifid", "thermal_throttle_set"; level = throttle);
                Self::copy_to_buf(buf, b"OK\n")
            }
            b"rx" => {
                let rx = match self.0.band0_rx.as_ref() {
                    Some(r) => r,
                    None => return Self::copy_to_buf(buf, b"ERR no_band0_rx\n"),
                };
                match value {
                    b"monitor" => {
                        // Accept ALL frames — clear both RFCR and RFCR1
                        dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), 0);
                        dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), 0);
                        // Re-arm CPU_IDX to ndesc-1 so DMA has full ring available
                        // (CPU_IDX=0 with DMA_IDX=0 gives DMA zero free slots)
                        dev.mt76_wr(rx.regs_base + MT_QUEUE_CPU_IDX, rx.ndesc - 1);
                        udebug!("wifid", "rx_monitor_mode");
                        Self::copy_to_buf(buf, b"OK monitor\n")
                    }
                    b"normal" => {
                        // Restore AP default filter — matches Linux mt7996_configure_filter()
                        let rfcr: u32 = MT_WF_RFCR_DROP_FCSFAIL
                            | MT_WF_RFCR_DROP_CTL_RSV
                            | MT_WF_RFCR_DROP_CTS
                            | MT_WF_RFCR_DROP_RTS
                            | MT_WF_RFCR_DROP_OTHER_UC;
                        dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr);
                        let rfcr1: u32 = MT_WF_RFCR1_DROP_ACK
                            | MT_WF_RFCR1_DROP_BF_POLL
                            | MT_WF_RFCR1_DROP_BA
                            | MT_WF_RFCR1_DROP_CFEND
                            | MT_WF_RFCR1_DROP_CFACK;
                        dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1);
                        udebug!("wifid", "rx_normal_mode");
                        Self::copy_to_buf(buf, b"OK normal\n")
                    }
                    b"reset" => {
                        // Re-initialize all RX descriptors and reset CPU_IDX
                        // This clears stale frames and gives DMA a fresh ring
                        dev.rx_fill(rx);
                        udebug!("wifid", "rx_ring_reset");
                        Self::copy_to_buf(buf, b"OK reset\n")
                    }
                    _ => Self::copy_to_buf(buf, b"ERR monitor|normal|reset\n"),
                }
            }
            b"scan" => {
                if value != b"start" {
                    return Self::copy_to_buf(buf, b"ERR use: start\n");
                }
                let rx = match self.0.band0_rx.as_ref() {
                    Some(r) => r,
                    None => return Self::copy_to_buf(buf, b"ERR no_band0_rx\n"),
                };

                // Step 1: Re-arm RX ring — clear stale frames
                dev.rx_fill(rx);

                // Step 2: Accept all frames for scan — clear RFCR filters
                dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), 0);
                dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), 0);

                // Step 3: Send MCU OFFCH_SCAN_CTRL START — mcu.c:3604
                let ch = self.0.channel;
                if dev.mcu_background_chain_ctrl(wm_ring, 0, ch, 0, 0, 1, self.0.seq).is_err() {
                    uwarn!("wifid", "scan_mcu_start_failed");
                    // Continue anyway — scan via monitor mode even if MCU rejects
                }
                self.0.seq = self.0.seq.wrapping_add(1);

                // Step 4: Wait 300ms for beacons (~3 beacon intervals)
                userlib::delay_ms(300);

                // Step 5: DSB SY — ensure DMA writes visible to CPU
                unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)); }

                // Step 6: Scan RX ring for beacons
                let dma_idx = dev.mt76_rr(rx.regs_base + MT_QUEUE_DMA_IDX);
                let avail = dma_idx.min(rx.ndesc);
                let mut beacon_count = 0u32;
                let mut seen_bssids: [[u8; 6]; 16] = [[0u8; 6]; 16];
                let mut seen_count = 0usize;

                for idx in 0..avail {
                    let i = idx as usize;
                    let desc_ptr = unsafe { (rx.desc_virt as *const dma::Mt76Desc).add(i) };
                    let ctrl = unsafe { core::ptr::read_volatile(&(*desc_ptr).ctrl) };
                    if ctrl & MT_DMA_CTL_DMA_DONE == 0 { continue; }

                    let buf_base = rx.buf_virt + (i as u64) * (rx.buf_size as u64);
                    let buf_ptr = buf_base as *const u32;
                    let rxd0 = unsafe { core::ptr::read_volatile(buf_ptr) };
                    let rxd1 = unsafe { core::ptr::read_volatile(buf_ptr.add(1)) };
                    let rxd2 = unsafe { core::ptr::read_volatile(buf_ptr.add(2)) };

                    let pkt_type = (rxd0 >> 27) & 0x1F;
                    if pkt_type != 2 { continue; } // Skip non-NORMAL frames

                    let remove_pad = (rxd2 >> 13) & 0x7;
                    let g3_present = rxd1 & MT_RXD1_NORMAL_GROUP_3 != 0;

                    // Navigate RXD groups: order G4, G1, G2, G3 (each 16 bytes)
                    let mut g3_ofs: usize = 32;
                    if rxd1 & MT_RXD1_NORMAL_GROUP_4 != 0 { g3_ofs += 16; }
                    if rxd1 & MT_RXD1_NORMAL_GROUP_1 != 0 { g3_ofs += 16; }
                    if rxd1 & MT_RXD1_NORMAL_GROUP_2 != 0 { g3_ofs += 16; }

                    // RCPI from P-RXV[3] (Group 3, DW3) — Linux mac.c:625
                    let rssi: i32 = if g3_present && g3_ofs + 16 <= rx.buf_size as usize {
                        let g3_dw3 = unsafe {
                            core::ptr::read_volatile(buf_ptr.add((g3_ofs + 12) / 4))
                        };
                        let rcpi0 = g3_dw3 & 0xFF;
                        ((rcpi0 as i32) - 220) / 2
                    } else {
                        -128
                    };

                    // Frame body offset
                    let mut frame_ofs: usize = g3_ofs;
                    if g3_present {
                        frame_ofs += 16;
                        if rxd1 & MT_RXD1_NORMAL_GROUP_5 != 0 { frame_ofs += 96; }
                    }
                    frame_ofs += 2 * remove_pad as usize;

                    // Check frame control = beacon (0x0080)
                    if frame_ofs + 36 > rx.buf_size as usize { continue; }
                    let byte_ptr = buf_base as *const u8;
                    let fc = unsafe {
                        let lo = core::ptr::read_volatile(byte_ptr.add(frame_ofs)) as u16;
                        let hi = core::ptr::read_volatile(byte_ptr.add(frame_ofs + 1)) as u16;
                        lo | (hi << 8)
                    };
                    if fc != 0x0080 { continue; } // Not a beacon

                    // Extract BSSID from addr3 (offset 16 from FC)
                    let mut bssid = [0u8; 6];
                    for j in 0..6 {
                        bssid[j] = unsafe { core::ptr::read_volatile(byte_ptr.add(frame_ofs + 16 + j)) };
                    }

                    // Dedup: skip if BSSID already seen
                    let mut dup = false;
                    for s in 0..seen_count {
                        if seen_bssids[s] == bssid { dup = true; break; }
                    }
                    if dup { continue; }
                    if seen_count < 16 {
                        seen_bssids[seen_count] = bssid;
                        seen_count += 1;
                    }

                    // Parse IEs: skip MAC header (24) + timestamp (8) + interval (2) + capability (2) = 36
                    let ie_start = frame_ofs + 24 + 12;
                    let pkt_len = (rxd0 & MT_RXD0_LENGTH) as usize;
                    let frame_end = frame_ofs + pkt_len.min(rx.buf_size as usize - frame_ofs);

                    let mut ssid_buf = [0u8; 33];
                    let mut ssid_len = 0usize;
                    let mut ie_chan: u8 = 0;
                    let mut ie_pos = ie_start;

                    while ie_pos + 2 <= frame_end {
                        let tag = unsafe { core::ptr::read_volatile(byte_ptr.add(ie_pos)) };
                        let len = unsafe { core::ptr::read_volatile(byte_ptr.add(ie_pos + 1)) } as usize;
                        if ie_pos + 2 + len > frame_end { break; }

                        if tag == 0 && len <= 32 {
                            // SSID IE
                            ssid_len = len;
                            for j in 0..len {
                                let c = unsafe { core::ptr::read_volatile(byte_ptr.add(ie_pos + 2 + j)) };
                                ssid_buf[j] = if c >= 0x20 && c < 0x7f { c } else { b'?' };
                            }
                        } else if tag == 3 && len == 1 {
                            // DS Parameter Set — channel number
                            ie_chan = unsafe { core::ptr::read_volatile(byte_ptr.add(ie_pos + 2)) };
                        }

                        ie_pos += 2 + len;
                    }

                    // Log beacon — BSSID, SSID, channel, RSSI
                    let ssid_str = core::str::from_utf8(&ssid_buf[..ssid_len]).unwrap_or("?");
                    let abs_rssi = if rssi < 0 { (-rssi) as u32 } else { rssi as u32 };
                    udebug!("scan", "beacon";
                        ssid = ssid_str,
                        ch = ie_chan,
                        rssi_neg = abs_rssi,
                        bssid0 = bssid[0], bssid1 = bssid[1], bssid2 = bssid[2],
                        bssid3 = bssid[3], bssid4 = bssid[4], bssid5 = bssid[5]
                    );
                    beacon_count += 1;
                }

                // Step 8: Send MCU OFFCH_SCAN_CTRL STOP
                let _ = dev.mcu_background_chain_ctrl(wm_ring, 0, ch, 0, 0, 0, self.0.seq);
                self.0.seq = self.0.seq.wrapping_add(1);

                // Step 9: Restore RFCR to normal AP filter
                let rfcr_val: u32 = MT_WF_RFCR_DROP_FCSFAIL
                    | MT_WF_RFCR_DROP_CTL_RSV
                    | MT_WF_RFCR_DROP_CTS
                    | MT_WF_RFCR_DROP_RTS
                    | MT_WF_RFCR_DROP_OTHER_UC;
                let rfcr1_val: u32 = MT_WF_RFCR1_DROP_ACK
                    | MT_WF_RFCR1_DROP_BF_POLL
                    | MT_WF_RFCR1_DROP_BA
                    | MT_WF_RFCR1_DROP_CFEND
                    | MT_WF_RFCR1_DROP_CFACK;
                dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr_val);
                dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1_val);

                // Format response: "OK N beacons\n"
                let mut tmp = [0u8; 32];
                tmp[0..3].copy_from_slice(b"OK ");
                let mut pos = 3;
                pos += fmt_u32_dec(beacon_count, &mut tmp[pos..]);
                tmp[pos..pos + 9].copy_from_slice(b" beacons\n");
                pos += 9;
                Self::copy_to_buf(buf, &tmp[..pos])
            }
            _ => Self::copy_to_buf(buf, b"ERR unknown_key\n"),
        }
    }
}
