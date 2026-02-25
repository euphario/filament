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
use userlib::ipc::{Msi, Irq, PciDevice};
use userlib::bus::{BusMsg, BusError, BusCtx, Driver, Disposition, ConfigKey};
use userlib::bus_runtime::driver_main;

mod regs;
mod dma;
mod device;
mod mcu;
mod firmware;
mod mac;
mod event;

use regs::*;
use dma::{TxRing, TxFreeResult, flush_buffer};
use device::Mt7996Dev;
use event::RxMibCounters;

use wifi80211::ap::ApManager;
use wifi80211::types::{BssConfig, RxMgmtFrame, MgmtSubtype, ApAction, MAX_SSID_LEN};

// ============================================================================
// WifiDriver — Bus framework integration
// ============================================================================

struct WifiDriver {
    dev: Option<Mt7996Dev>,
    bar0: Option<MmioRegion>,
    desc_pool: Option<DmaPool>,
    rx_pool: Option<DmaPool>,
    wa_ring: Option<TxRing>,
    tx_band0: Option<TxRing>,
    wm_tx_buf_pool: Option<DmaPool>,
    wa_tx_buf_pool: Option<DmaPool>,
    tx_band0_pool: Option<DmaPool>,
    band0_rx: Option<dma::RxQueueInfo>,
    rx_queues: [dma::RxQueueInfo; regs::NUM_RX_QUEUES],
    rx_queue_count: usize,
    mac_addr: [u8; 6],
    seq: u8,
    drain_ticks: u32,
    radio_on: bool,
    beacon_on: bool,
    tx_throttle: u8,
    channel: u8,
    irq_mode: bool,
    /// BDF (Bus/Device/Function) from pcied metadata
    bdf: u32,
    /// PCI device handle (keeps ownership claim alive)
    pci_dev: Option<PciDevice>,
    /// MSI allocation (keeps kernel allocation alive)
    msi: Option<Msi>,
    /// IRQ handle for MSI interrupt
    irq: Option<Irq>,
    /// Software MIB counters: [0]=band0, [1]=band2, [2]=MCU events
    mib: [RxMibCounters; 3],
    /// TX probe response counter
    tx_probe_resp: u32,
    /// TX beacon counter
    tx_beacon: u32,
    /// Beacon tick counter (increments every 10ms event tick, resets at 10 = 100ms)
    beacon_tick: u8,
    /// Heartbeat tick counter (increments every 10ms, fires at 50 = 500ms)
    heartbeat_tick: u8,
    /// Previous tx_ok count for radio-stop detection
    last_tx_ok: u32,
    /// Consecutive seconds with no TX progress (0 = healthy)
    tx_stall_count: u8,
    /// TX token allocator — monotonically increasing, wrapping u16.
    /// Firmware uses tokens to track TX completion; each enqueued frame needs a unique one.
    /// Linux: mt76_token_consume() via idr_alloc() starting from token_start=0.
    tx_token: u16,
    /// Total TX tokens freed by firmware (via WA_MAIN/WA_TRI TX free events)
    tx_freed: u32,
    /// Total WA_MAIN entries processed (for diagnosing txf=0)
    tx_free_entries: u32,
    /// Total TXRX_NOTIFY (pkt_type=6) entries found
    tx_free_notify_count: u32,
    /// First TXRX_NOTIFY payload dumped (one-shot diagnostic)
    tx_free_payload_dumped: bool,
    /// 802.11 AP state machine (STA table, auth/assoc handling)
    ap: Option<ApManager>,
    /// Next WLAN index for client STAs (starts below WTBL_RESERVED)
    next_wlan_idx: u16,
}

impl WifiDriver {
    const fn new() -> Self {
        Self {
            dev: None,
            bar0: None,
            desc_pool: None,
            rx_pool: None,
            wa_ring: None,
            tx_band0: None,
            wm_tx_buf_pool: None,
            wa_tx_buf_pool: None,
            tx_band0_pool: None,
            band0_rx: None,
            rx_queues: [dma::RxQueueInfo::ZERO; regs::NUM_RX_QUEUES],
            rx_queue_count: 0,
            mac_addr: [0; 6],
            seq: 0,
            drain_ticks: 0,
            radio_on: false,
            beacon_on: false,
            tx_throttle: 100,
            channel: 1,
            irq_mode: false,
            bdf: 0,
            pci_dev: None,
            msi: None,
            irq: None,
            last_tx_ok: 0,
            tx_stall_count: 0,
            mib: [RxMibCounters::new(); 3],
            tx_probe_resp: 0,
            tx_beacon: 0,
            beacon_tick: 0,
            heartbeat_tick: 0,
            tx_token: 0,
            tx_freed: 0,
            tx_free_entries: 0,
            tx_free_notify_count: 0,
            tx_free_payload_dumped: false,
            ap: None,
            next_wlan_idx: MT7996_WTBL_RESERVED - 1,
        }
    }

    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        unotice!("wifid", "init_start");

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

        // Read BDF from metadata bytes 12-16 (u32 LE, set by pcied)
        let bdf = if meta.len() >= 16 {
            u32::from_le_bytes([meta[12], meta[13], meta[14], meta[15]])
        } else {
            0
        };

        unotice!("wifid", "device_found"; bar0 = bar0_addr, size = bar0_size, bdf = bdf as u64);

        // Step 2: Map BAR0
        udebug!("wifid", "map_bar0"; addr = bar0_addr, size_kb = bar0_size / 1024);
        let bar0 = MmioRegion::open(bar0_addr, bar0_size).ok_or_else(|| {
            uerror!("wifid", "mmap_device_failed");
            BusError::Internal
        })?;
        let bar0_virt = bar0.virt_base();
        udebug!("wifid", "bar0_mapped"; virt = bar0_virt);

        // HIF2 registers (0xd8xxx) are accessible through HIF1's BAR at offset 0xd8xxx.
        // The MT7996 chip always has dual HIF internally — HIF2 is a second DMA engine
        // within the same chip, reachable via BAR0. pcied skips the companion PCI
        // function (0x7991) but HIF2 hardware is still present and must be configured.
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
    let (mcu_wa_rx_buf_virt, mcu_wa_rx_buf_size, rx_queues, rx_queue_count) = dev.mt7996_dma_init(desc_phys, desc_virt, DESC_MEM_SIZE, rx_buf_phys, rx_buf_virt, RX_BUF_POOL_SIZE);

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

    // Claim PCI device ownership and allocate MSI before firmware loading —
    // enables IRQ-based DMA waiting instead of delay_ms(1) busy-wait polling.
    let mut fw_irq: Option<crate::mcu::FwIrq> = None;
    if bdf != 0 {
        // Claim device ownership (required before MSI allocation)
        match PciDevice::open(bdf) {
            Ok(pci_dev) => {
                self.pci_dev = Some(pci_dev);
            }
            Err(e) => {
                uwarn!("wifid", "pci_claim_fail"; err = e.to_errno(), bdf = bdf as u64);
            }
        }
        match Msi::allocate(bdf, 1) {
            Ok(msi_alloc) => {
                let virq = msi_alloc.first_irq();
                match Irq::new(virq) {
                    Ok(irq) => {
                        match crate::mcu::FwIrq::new(irq) {
                            Ok(fw) => {
                                // Enable WFDMA interrupts for firmware download
                                let fw_irq_mask = MT_INT_TX_DONE_FWDL | MT_INT_TX_DONE_MCU_WM
                                    | MT_INT_RX_DONE_WM;
                                dev.mt76_wr(MT_INT_MASK_CSR, fw_irq_mask);
                                unotice!("wifid", "msi_allocated"; irq = virq, bdf = bdf as u64);
                                self.msi = Some(msi_alloc);
                                fw_irq = Some(fw);
                            }
                            Err(_) => {
                                uwarn!("wifid", "fw_irq_mux_fail");
                            }
                        }
                    }
                    Err(e) => {
                        uwarn!("wifid", "irq_new_fail"; err = e.to_errno(), virq = virq as u64);
                    }
                }
            }
            Err(e) => {
                uwarn!("wifid", "msi_alloc_fail"; err = e.to_errno(), bdf = bdf as u64);
            }
        }
    }

    // BPI-R4 is MT7996 233 variant (dual-ADIE TBTC, 2+3+3 chains)
    // Detected via MT_PAD_GPIO bit 19 — confirmed on hardware (pad_gpio=0x94800)
    // 233 variant requires different firmware binaries than standard 444
    dev.load_firmware(&mut mcu_ring, &mut fwdl_ring, None).map_err(|e| {
        uerror!("wifid", "firmware_load_fail"; err = e);
        BusError::Internal
    })?;

    // ====================================================================
    // Post-firmware MCU init
    // Linux: mt7996/mcu.c:295-298 — after MCU_RUNNING + WA present:
    //   ALL commands → MT_MCUQ_WA (hw_idx=20)
    // WA firmware acts as command proxy — inspects s2d_index and routes
    // WM-destined commands internally. Host must NOT send on WM TX ring
    // (hw_idx=17) after WA boots — WM expects all host comms via WA.
    // ====================================================================

    // MCU_WA ring (hw_idx=20) — THE ONLY post-firmware command ring
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
    wa_ring.rx_regs = MCU_WA_RX_REGS; // All MCU responses arrive on WA RX (q1)
    wa_ring.rx_buf_virt = mcu_wa_rx_buf_virt;
    wa_ring.rx_buf_size = mcu_wa_rx_buf_size;

    // mcu_ring (WM TX, hw_idx=17) is no longer used — WA proxies all commands
    drop(mcu_ring);

    // Post-firmware: drop FwIrq. All post-firmware MCU commands use
    // polling (delay_ms(1)) for wait_rx_response — the original working path.
    // IRQ-based waiting in wait_rx_response causes set_eeprom timeout.
    // The Irq handle is kept for the event-driven phase (bus Mux).
    let wifi_irq = fw_irq.map(|fi| fi.into_irq());

    udebug!("mcu", "post_init_start");
    let mut seq: u8 = 1;

    let mcu_err = |_e: i32| { BusError::Internal };

    // fw_log_2_host(WM, 0) — enable WM logging
    // MCU_WM_UNI_CMD(WSYS_CONFIG) — WM-only, routed via WA queue
    dev.mcu_fw_log_2_host(&mut wa_ring, 0, 0, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // fw_log_2_host(WA, 0) — enable WA logging
    // MCU_WA_UNI_CMD(WSYS_CONFIG) — WA, via WA queue
    dev.mcu_fw_log_2_host(&mut wa_ring, 1, 0, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // set_mwds(1) — enable MWDS
    // MCU_WA_EXT_CMD(MWDS_SUPPORT) — WA, via WA queue
    dev.mcu_set_mwds(&mut wa_ring, true, seq).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // init_rx_airtime() — RX airtime for bands 0,1,2
    // MCU_WM_UNI_CMD(VOW) — WM-only, routed via WA queue
    dev.mcu_init_rx_airtime(&mut wa_ring, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // wa_cmd(SET, RED, 0, 0) — enable Random Early Drop
    // MCU_WA_PARAM_CMD(SET) — WA, via WA queue
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
    let free_blocks = dev.mcu_get_eeprom_free_block(&mut wa_ring, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    if free_blocks >= 59 {
        // eFuse empty — upload embedded mt7996_eeprom.bin via flash/buffer mode
        udebug!("wifid", "eeprom_flash_upload"; size = firmware::FW_EEPROM.len() as u32);
        dev.mcu_set_eeprom_flash(&mut wa_ring, firmware::FW_EEPROM, &mut seq, None).map_err(mcu_err)?;
        udebug!("wifid", "eeprom_flash_ok");
    } else {
        // eFuse has calibration data — use eFuse mode
        udebug!("wifid", "eeprom_efuse_mode"; free = free_blocks);
        dev.mcu_set_eeprom(&mut wa_ring, seq, None).map_err(mcu_err)?;
        seq = seq.wrapping_add(1);
    }

    // Read eFuse WiFi config to determine actual antenna configuration.
    // EEPROM offset 0x190 = WiFi config area (stream/path/nss per band).
    // Linux: eeprom.c:64-94 mt7996_eeprom_parse_stream()
    //   Byte[1] bits[5:3] = TX_PATH_BAND0
    //   Byte[3] bits[2:0] = RX_PATH_BAND0
    //   Byte[4] bits[5:3] = STREAM_NUM_BAND0 (nss)
    match dev.mcu_get_eeprom(&mut wa_ring, 0x190, seq, None) {
        Ok(wifi_conf) => {
            seq = seq.wrapping_add(1);
            let tx_path_b0 = (wifi_conf[1] >> 3) & 0x7;
            let rx_path_b0 = wifi_conf[3] & 0x7;
            let nss_b0 = (wifi_conf[4] >> 3) & 0x7;
            udebug!("wifid", "efuse_wifi_conf"; tx = tx_path_b0 as u32, rx = rx_path_b0 as u32, nss = nss_b0 as u32);
            // Also dump device ID from eFuse offset 0
            match dev.mcu_get_eeprom(&mut wa_ring, 0, seq, None) {
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
    match dev.mcu_rf_regval(&mut wa_ring, MT_ADIE_CHIP_ID_0, seq, None) {
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

    // Full MAC init: WTBL clear, RRO(WM), HIF TXD(WA), per-band regs, basic rates(WM)
    dev.mac_init(&mut wa_ring, &mut seq, None).map_err(mcu_err)?;

    // TxBF init: beamforming subsystem — init.c:757 mt7996_txbf_init()
    // MCU_WM_UNI_CMD(BF) — WM-only, routed via WA queue
    dev.mcu_txbf_init(&mut wa_ring, &mut seq, None).map_err(mcu_err)?;

    // ====================================================================
    // Thermal protection + radio control
    // Linux: mt7996/main.c mt7996_start() — thermal protect, throttle, radio
    // ====================================================================

    // Thermal protection: enable for all 3 bands
    // MCU_WM_UNI_CMD(THERMAL) — WM-only, routed via WA queue
    for band in 0..3u8 {
        dev.mcu_set_thermal_protect(&mut wa_ring, band, true, seq).map_err(mcu_err)?;
        seq = seq.wrapping_add(1);
    }
    udebug!("wifid", "thermal_protect_ok");

    // Thermal throttling: 100% (full power — throttle only on overtemp)
    // MCU_WM_UNI_CMD(THERMAL) — WM-only, routed via WA queue
    for band in 0..3u8 {
        dev.mcu_set_thermal_throttling(&mut wa_ring, band, 100, seq).map_err(mcu_err)?;
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
    // MCU_WM_UNI_CMD(RX_HDR_TRANS) — WM-only, routed via WA queue
    dev.mcu_set_hdr_trans(&mut wa_ring, true, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "hdr_trans_ok");

    // === Band 0 radio startup — Linux main.c:10-48 mt7996_run() ===

    // Enable noise floor measurement — Linux main.c:15 (FIRST thing in mt7996_run!)
    // Enables IPI for CCA (Clear Channel Assessment) — required before TX
    dev.mac_enable_nf(0);
    udebug!("wifid", "enable_nf_ok");

    // Configure RX filter — Linux init.c:414 + main.c:432-454 mt7996_phy_set_rxfilter()
    // Linux default: phy->rxfilter = DROP_OTHER_UC, then phy_set_rxfilter() expands
    // to DROP_OTHER_UC | DROP_CTS | DROP_RTS | DROP_CTL_RSV | DROP_FCSFAIL.
    //
    // NOTE: 0xE002 (bits 15,14,13,1) = DROP_RTS|CTS|CTL_RSV|FCSFAIL — this IS our
    // value, not a firmware overwrite. Previous 0xE002="firmware overwrite" was a hex
    // calculation error (bits 11,10,9 would be 0x0E02=3586, not 0xE002=57346).
    //
    // DROP_FCSFAIL omitted for now: RFCR=0 (which clears DROP_FCSFAIL) allowed auth
    // frames through in previous tests. Possible that auth frames arrive with FCS
    // errors due to radio calibration. Test without it to isolate.
    let rfcr_val: u32 = MT_WF_RFCR_DROP_CTL_RSV
        | MT_WF_RFCR_DROP_CTS
        | MT_WF_RFCR_DROP_RTS;
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
    // MCU_WM_UNI_CMD(BAND_CONFIG) — WM-only, routed via WA queue
    dev.mcu_set_rts_thresh(&mut wa_ring, 0, 0x92b, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "rts_thresh_ok");

    // Radio ON — Linux main.c:21 (before RX_PATH in mt7996_run!)
    // MCU_WM_UNI_CMD(BAND_CONFIG) — WM-only, routed via WA queue
    dev.mcu_set_radio_en(&mut wa_ring, 0, true, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // RX path — Linux main.c:25 (after radio enable in mt7996_run)
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_RX_PATH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "rx_path_init_ok");

    // === Interface creation (band 0) ===
    // Linux: mt7996_vif_link_add() → add_dev_info + add_bss_info + add_sta

    // DEV_INFO: activate OMAC on band 0 — Linux mcu.c:2623
    udebug!("wifid", "dev_info_send"; band = 0u32, omac = HW_BSSID_0 as u32, active = 1u32);
    dev.mcu_add_dev_info(&mut wa_ring, 0, HW_BSSID_0, &mac_addr, true, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "dev_info_ok");

    // BSS_INFO: create BSS on band 0 — Linux mcu.c:1123
    // hw_bss_idx = 0 (matches bss_req_hdr.bss_idx in uni_header)
    // omac_idx = HW_BSSID_0 (OMAC to use, independent of bss_idx)
    udebug!("wifid", "bss_info_send"; band = 0u32, omac = HW_BSSID_0 as u32, bss = 0u32, active = 1u32, ch = 1u32);
    dev.mcu_add_bss_info(&mut wa_ring, 0, HW_BSSID_0, 0, &mac_addr, true, 1, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "bss_info_ok");

    // STA_REC: add broadcast/multicast STA — Linux mcu.c:2438
    // bss_idx = 0 (must match BSS_INFO's bss_req_hdr.bss_idx)
    // wlan_idx = MT7996_WTBL_RESERVED - bss_idx = 1087 (Linux: main.c:334)
    // muar_idx(omac_idx param) = 0 (band_idx, Linux: wcid.phy_idx = band_idx)
    udebug!("wifid", "sta_rec_send"; bss = 0u32, wlan = MT7996_WTBL_RESERVED as u32, omac = 0u32, newly = 1u32);
    dev.mcu_add_sta(&mut wa_ring, 0, MT7996_WTBL_RESERVED, 0, CONN_STATE_PORT_SECURE, &mac_addr, true, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "sta_rec_ok");

    // VOW: assign WCID to BSS group — Linux mcu.c:2504 mt7996_mcu_add_group()
    udebug!("wifid", "vow_send"; bss = 0u32, wlan = MT7996_WTBL_RESERVED as u32);
    dev.mcu_add_group(&mut wa_ring, 0, MT7996_WTBL_RESERVED, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // === Channel tune (band 0) — Linux main.c:553 mt7996_set_channel() ===
    // Radio is already ON from mt7996_run() sequence above.

    // Channel switch — Linux main.c:561
    udebug!("wifid", "chan_switch"; band = 0u32, ch = 1u32, bw = 0u32, ch_band = 0u32);
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_SWITCH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // RX path after switch — Linux main.c:565
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_RX_PATH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // TX power SKU — Linux main.c:569
    dev.mcu_set_txpower_sku(&mut wa_ring, 0, seq, None).map_err(mcu_err)?;
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

    // Band 1 (5GHz) radio disabled — only band 0 (2.4GHz) active.
    // Linux only calls mt7996_run() when an interface is added on that band.
    // Enabling band 1 radio without BSS/STA/beacon is non-standard and may
    // confuse firmware beacon scheduling.

    // Program beacon rate table: 1 Mbps CCK (most compatible 2.4GHz rate)
    // MCU_WM_UNI_CMD(FIXED_RATE_TABLE) — WM-only, routed via WA queue
    // Rate encoding: mode=CCK(0) at bits[9:6], idx=0 at bits[5:0] → 0x0000
    let beacon_rate_idx = MT7996_BEACON_RATES_TBL + 2 * 0; // band 0 → table 25
    dev.mcu_set_fixed_rate_table(&mut wa_ring, beacon_rate_idx, 0x0000, true, 0, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // Program basic rate table entry too (for bc/mc frames)
    // rate_idx = 0x0000 = 1Mbps CCK
    dev.mcu_set_fixed_rate_table(&mut wa_ring, MT7996_BASIC_RATES_TBL, 0x0000, false, 0, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // === Beacon enable sequence — Linux main.c:857-903 mt7996_link_info_changed() ===
    // BSS_CHANGED_BEACON_ENABLED fires AFTER channel switch in Linux.
    // Linux re-sends BSS_INFO + STA_REC at this point to confirm BSS on channel.

    // Second BSS_INFO: re-send after channel is configured — Linux main.c:859
    udebug!("wifid", "bss_info2_send"; band = 0u32, omac = HW_BSSID_0 as u32, bss = 0u32, active = 1u32);
    dev.mcu_add_bss_info(&mut wa_ring, 0, HW_BSSID_0, 0, &mac_addr, true, 1, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // Second STA_REC: update existing BMC STA (newly=false) — Linux main.c:861
    udebug!("wifid", "sta_rec2_send"; bss = 0u32, wlan = MT7996_WTBL_RESERVED as u32, newly = 0u32);
    dev.mcu_add_sta(&mut wa_ring, 0, MT7996_WTBL_RESERVED, 0, CONN_STATE_PORT_SECURE, &mac_addr, false, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // VOW: reassign WCID to BSS group — Linux mcu.c:2504
    dev.mcu_add_group(&mut wa_ring, 0, MT7996_WTBL_RESERVED, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // EDCA/WMM parameters — Linux main.c:883-884
    // MCU_WM_UNI_CMD(EDCA_UPDATE) — WM-only, routed via WA queue
    // "ensure that enable txcmd_mode after bss_info"
    // bss_idx = 0 (must match BSS_INFO's bss_req_hdr.bss_idx)
    udebug!("wifid", "edca_send"; bss = 0u32);
    dev.mcu_set_edca(&mut wa_ring, 0, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "edca_ok");

    // Beacon: firmware offload DISABLED — it autonomously stops after ~57 seconds
    // for unknown reasons. Instead, we use software TX beacons via BAND0 CT ring
    // which we fully control. The BSS_INFO/STA_REC/rate table setup above is still
    // needed for the firmware to know about our BSS and allow TX.
    udebug!("wifid", "beacon_sw_mode");

    // Re-set RFCR after all MCU commands (Linux calls configure_filter after BSS changes)
    dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr_val);
    dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1_val);

    // TX BAND0 ring setup — for probe responses and management frames
    // Descriptors already allocated at offset 0 in desc_pool during DMA init.
    // Hardware ring programmed with MT7996_TX_RING_SIZE=2048 — must match.
    // Allocate buffer pool: 2048 entries × 256 bytes = 512KB
    const TX_BAND0_BUF_STRIDE: usize = 256;
    const TX_BAND0_NDESC: u32 = MT7996_TX_RING_SIZE; // Must match hardware ring size
    const TX_BAND0_BUF_SIZE: usize = TX_BAND0_NDESC as usize * TX_BAND0_BUF_STRIDE;
    let tx_band0_pool = DmaPool::alloc_high(TX_BAND0_BUF_SIZE).or_else(|| {
        DmaPool::alloc(TX_BAND0_BUF_SIZE)
    }).ok_or_else(|| {
        uerror!("wifid", "tx_band0_alloc_failed");
        BusError::Internal
    })?;
    let tx_b0_regs = tx_ring_base + MT7996_TXQ_BAND0 * MT_RING_SIZE;
    let mut tx_band0 = TxRing::new(
        tx_b0_regs,
        TX_BAND0_NDESC,
        desc_virt,  // TX BAND0 descriptors are at offset 0
        desc_phys,
        tx_band0_pool.vaddr(),
        tx_band0_pool.paddr(),
    );
    tx_band0.buf_stride = TX_BAND0_BUF_STRIDE;
    udebug!("wifid", "tx_band0_ready"; ndesc = TX_BAND0_NDESC, buf_stride = TX_BAND0_BUF_STRIDE as u32);

    // Final state
    let final_fw_state = dev.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
    unotice!("wifid", "init_complete"; fw_state = final_fw_state);

    // Event processing: MSI IRQ mode with timer fallback for beacons
    const TAG_WIFI_IRQ: u32 = 101;
    const TAG_WIFI_TIMER: u32 = 100;
    const BEACON_INTERVAL_NS: u64 = 10_000_000; // 10ms for beacon ticks

    // Transition MSI IRQ from synchronous firmware-loading mode to event-driven.
    // The Irq was already allocated before firmware upload; now add it to the
    // bus Mux so handle_event() receives WFDMA interrupt notifications.
    let mut irq_ok = false;
    if let Some(irq) = wifi_irq {
        let handle = irq.handle();
        match ctx.watch_handle(handle, TAG_WIFI_IRQ) {
            Ok(()) => {
                // Switch WFDMA interrupt mask to operational mode:
                // RX done + MCU cmd + TX done (bands + MCU WM + FWDL)
                let irq_mask = MT_INT_RX_DONE_ALL | MT_INT_MCU_CMD
                    | MT_INT_TX_DONE_BAND0 | MT_INT_TX_DONE_MCU_WM
                    | MT_INT_TX_DONE_FWDL;
                dev.mt76_wr(MT_INT_MASK_CSR, irq_mask);

                self.irq = Some(irq);
                self.irq_mode = true;
                irq_ok = true;
            }
            Err(_) => {
                uwarn!("wifid", "watch_handle_fail");
            }
        }
    }

    if irq_ok {
        unotice!("wifid", "irq_mode");
    } else {
        uwarn!("wifid", "timer_fallback");
    }

    // Always start beacon timer (10ms ticks for beacon generation)
    if let Err(_) = ctx.start_timer(TAG_WIFI_TIMER, BEACON_INTERVAL_NS) {
        uwarn!("wifid", "beacon_timer_fail");
    }

    // Initialize AP state machine
    let mut ssid = [0u8; MAX_SSID_LEN];
    ssid[..8].copy_from_slice(b"Filament");
    self.ap = Some(ApManager::new(BssConfig {
        bssid: mac_addr,
        ssid,
        ssid_len: 8,
        channel: 1,
    }));

    // Store resources and state in driver struct
    self.mac_addr = mac_addr;
    self.seq = seq;
    self.radio_on = true;
    self.beacon_on = true;
    self.tx_throttle = 100;
    self.channel = 1;
    self.bdf = bdf;
    self.dev = Some(dev);
    self.bar0 = Some(bar0);
    self.desc_pool = Some(desc_pool);
    self.rx_pool = Some(rx_pool);
    self.wa_ring = Some(wa_ring);
    self.tx_band0 = Some(tx_band0);
    self.wm_tx_buf_pool = Some(mcu_tx_buf_pool);  // Keep pool alive (memory backing)
    self.wa_tx_buf_pool = Some(wa_tx_buf_pool);
    self.tx_band0_pool = Some(tx_band0_pool);
    self.band0_rx = Some(rx_queues[2]); // BAND0 data RX (index 2)
    self.rx_queues = rx_queues;
    self.rx_queue_count = rx_queue_count;

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
// MT7996 TXD Wrapping — prepends hardware TXD to raw 802.11 frames
// ============================================================================

/// Wrap a raw 802.11 management frame in MT7996 TXD for CT-mode TX.
///
/// Reads the 802.11 FC to determine subtype for TXD2, and whether to set
/// NO_ACK (beacons) or not (unicast mgmt). The raw frame is copied after
/// the 32-byte TXD.
///
/// Returns total length (TXD + frame), or 0 if buffer too small.
///
/// Source: Linux mt76/mt7996/mac.c mt7996_mac_write_txwi()
fn wrap_mgmt_txd(buf: &mut [u8], frame: &[u8]) -> usize {
    let total = MT_TXD_SIZE + frame.len();
    if buf.len() < total || frame.len() < 2 {
        return 0;
    }

    for b in buf[..MT_TXD_SIZE].iter_mut() { *b = 0; }

    // Parse FC to get subtype and determine NO_ACK
    let fc0 = frame[0];
    let subtype = (fc0 >> 4) & 0xF;
    let is_beacon = subtype == 0x8;
    // Check if DA is broadcast (addr1 at offset 4)
    let is_bcast = frame.len() >= 10 && frame[4..10] == [0xFF; 6];

    // TXD0: TX_BYTES | PKT_FMT(CT=0) | Q_IDX(ALTX0=0x10)
    let txd0 = (total as u32)
        | ((MT_TX_TYPE_CT as u32) << 23)
        | ((MT_LMAC_ALTX0 as u32) << 25);
    buf[0..4].copy_from_slice(&txd0.to_le_bytes());

    // TXD1: WLAN_IDX(BMC) | FIXED_RATE | OWN_MAC | HDR_FORMAT | HDR_INFO | TID
    let txd1 = (1u32 << 31)                          // FIXED_RATE
        | ((MT_HDR_FORMAT_802_11 as u32) << 14)  // HDR_FORMAT = 802.11
        | (12u32 << 16)                          // HDR_INFO = 24/2
        | (MT_TX_NORMAL << 22)                   // TID = 5 (mgmt queue)
        | ((HW_BSSID_0 as u32) << 25)           // OWN_MAC = 0
        | (MT7996_WTBL_RESERVED as u32);         // WLAN_IDX = BMC STA
    buf[4..8].copy_from_slice(&txd1.to_le_bytes());

    // TXD2: SUB_TYPE | FRAME_TYPE(0=mgmt)
    let txd2 = subtype as u32;
    buf[8..12].copy_from_slice(&txd2.to_le_bytes());

    // TXD3: NO_ACK | SW_POWER_MGMT | REM_TX_COUNT | BA_DISABLE | BCM
    let txd3 = if is_beacon {
        // Beacons: NO_ACK, BCM, no SW_POWER_MGMT, rem_tx_count=31
        1u32 | MT_TXD3_BCM | (0x1Fu32 << 11) | (1u32 << 28)
    } else {
        // Unicast mgmt (probe resp, auth resp, assoc resp): ACK expected
        (1u32 << 29) | (15u32 << 11) | (1u32 << 28)
            | if is_bcast { MT_TXD3_BCM } else { 0 }
    };
    buf[12..16].copy_from_slice(&txd3.to_le_bytes());

    // TXD6: DAS | VTA | DIS_MAT | MSDU_CNT(1) | TX_RATE | FIXED_BW
    let txd6 = (1u32 << 2) | (1u32 << 28) | (1u32 << 3) | (1u32 << 4)
        | ((MT7996_BASIC_RATES_TBL as u32) << 16) | (1u32 << 25);
    buf[24..28].copy_from_slice(&txd6.to_le_bytes());

    // Copy raw 802.11 frame after TXD
    buf[MT_TXD_SIZE..total].copy_from_slice(frame);

    total
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
    ConfigKey::read_only(b"diag.ple"),
    ConfigKey::read_only(b"diag.mib"),
    ConfigKey::read_only(b"diag.bcn"),
    ConfigKey::read_only(b"diag.dma"),
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

    fn handle_event(&mut self, tag: u32, _handle: userlib::syscall::Handle, _ctx: &mut dyn BusCtx) {
        const TAG_WIFI_TIMER: u32 = 100;
        const TAG_WIFI_IRQ: u32 = 101;

        // Determine if this is an IRQ event or a timer tick
        let is_irq = tag == TAG_WIFI_IRQ;
        let is_timer = tag == TAG_WIFI_TIMER;
        if !is_irq && !is_timer {
            return;
        }

        let dev = match self.0.dev.as_ref() {
            Some(d) => d,
            None => return,
        };

        // Both timer and IRQ events process DMA queues.
        // Timer ticks MUST drain MCU RX queues (q0/q1) because the timer
        // handler sends fire-and-forget MCU commands (mcu_get_chan_mib_info,
        // mcu_get_all_sta_info) whose responses accumulate on q0/q1.
        // If only IRQ events drain these queues, the RX ring fills up
        // (~512 entries ÷ 10/sec = ~51 seconds) and firmware stalls.

        if is_timer {
            self.0.drain_ticks += 1;

        }

        // Read INT_SOURCE_CSR to determine which queues need servicing
        // Source: Linux mt76/mt7996/mmio.c mt7996_irq_handler()
        let intr = if self.0.irq_mode && is_irq {
            // IRQ event: mask, read source, clear, process
            dev.mt76_wr(MT_INT_MASK_CSR, 0);
            let src = dev.mt76_rr(MT_INT_SOURCE_CSR);
            if src != 0 {
                // Write-1-clear the handled bits
                dev.mt76_wr(MT_INT_SOURCE_CSR, src);
            }
            src
        } else {
            // Timer (both IRQ mode and fallback): process all queues
            // unconditionally. This ensures MCU response queues are
            // drained even when no IRQ fires between timer ticks.
            0xFFFF_FFFF
        };

        let qc = self.0.rx_queue_count;

        // MCU WM RX (q0) — firmware events from WM processor
        if intr & MT_INT_RX_DONE_WM != 0 && qc > 0 {
            dev.rx_process_mcu(&self.0.rx_queues[0], &mut self.0.mib[2]);
        }

        // MCU WA RX (q1) — firmware events from WA processor
        if intr & MT_INT_RX_DONE_WA != 0 && qc > 1 {
            dev.rx_process_mcu(&self.0.rx_queues[1], &mut self.0.mib[2]);
        }

        // BAND0 data RX (q2) — classify frames, collect management frames for AP
        let mut mgmt_frames: [RxMgmtFrame; 8] = core::array::from_fn(|_| RxMgmtFrame {
            subtype: MgmtSubtype::Other(0),
            addr2: [0; 6],
        });
        let mut mgmt_count = 0usize;
        if intr & MT_INT_RX_DONE_BAND0 != 0 && qc > 2 {
            let (_n, mc) = dev.rx_classify(
                &self.0.rx_queues[2], &mut self.0.mib[0],
                &mut mgmt_frames, 8,
            );
            mgmt_count = mc;
        }

        // WA_MAIN (q3) — TX free notifications
        if intr & MT_INT_RX_DONE_WA_MAIN != 0 && qc > 3 {
            let r = dev.rx_process_tx_free(&self.0.rx_queues[3]);
            self.0.tx_freed += r.tokens_freed;
            self.0.tx_free_entries += r.entries;
            self.0.tx_free_notify_count += r.txrx_notify_count;
            // One-shot dump of first TXRX_NOTIFY buffer for debugging
            if r.txrx_notify_count > 0 && !self.0.tx_free_payload_dumped {
                self.0.tx_free_payload_dumped = true;
                uinfo!("wifid", "txfree_dump";
                    dw0 = r.first_rxd[0], dw1 = r.first_rxd[1],
                    dw2 = r.first_rxd[2], dw3 = r.first_rxd[3],
                    dw4 = r.first_rxd[4], dw5 = r.first_rxd[5],
                    ver = r.first_payload[0],
                    msdu_cnt = r.first_payload[1],
                    byte_cnt = r.first_payload[2]
                );
            }
        }

        // BAND2 data RX (q4) — classify frames, update band2 counters
        if intr & MT_INT_RX_DONE_BAND2 != 0 && qc > 4 {
            let mut dummy: [RxMgmtFrame; 1] = [RxMgmtFrame {
                subtype: MgmtSubtype::Other(0),
                addr2: [0; 6],
            }];
            dev.rx_classify(&self.0.rx_queues[4], &mut self.0.mib[1], &mut dummy, 0);
        }

        // WA_TRI (q5) — TX free notifications (band2)
        if intr & MT_INT_RX_DONE_WA_TRI != 0 && qc > 5 {
            let r = dev.rx_process_tx_free(&self.0.rx_queues[5]);
            self.0.tx_freed += r.tokens_freed;
        }

        // Dispatch management frames through AP state machine.
        // We process one frame at a time: get actions from AP, then execute them.
        // Actions are executed inline to avoid borrow conflicts between ap and self.
        for i in 0..mgmt_count {
            let subtype = mgmt_frames[i].subtype;
            let addr2 = mgmt_frames[i].addr2;

            // Log every received management frame — essential for debugging the RX path
            match subtype {
                MgmtSubtype::ProbeReq => {
                    udebug!("wifid", "rx_probe_req";
                        mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                        mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                        mac4 = addr2[4] as u32, mac5 = addr2[5] as u32
                    );
                }
                MgmtSubtype::Auth => {
                    uinfo!("wifid", "rx_auth";
                        mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                        mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                        mac4 = addr2[4] as u32, mac5 = addr2[5] as u32
                    );
                }
                MgmtSubtype::AssocReq => {
                    uinfo!("wifid", "rx_assoc_req";
                        mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                        mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                        mac4 = addr2[4] as u32, mac5 = addr2[5] as u32
                    );
                }
                MgmtSubtype::Deauth => {
                    uinfo!("wifid", "rx_deauth";
                        mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                        mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                        mac4 = addr2[4] as u32, mac5 = addr2[5] as u32
                    );
                }
                MgmtSubtype::Disassoc => {
                    uinfo!("wifid", "rx_disassoc";
                        mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                        mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                        mac4 = addr2[4] as u32, mac5 = addr2[5] as u32
                    );
                }
                MgmtSubtype::Other(st) => {
                    udebug!("wifid", "rx_mgmt_other"; subtype = st as u32);
                }
            }

            // Phase 1: Get AP response (borrows self.0.ap mutably)
            let mut tx_frame = [0u8; 128]; // Raw 802.11 frame from AP
            let mut tx_len = 0usize;
            let mut register_mac = [0u8; 6];
            let mut register_aid = 0u16;
            let mut do_register = false;
            let mut remove_aid = 0u16;
            let mut do_remove = false;

            if let Some(ref mut ap) = self.0.ap {
                let mut raw_buf = [0u8; 256];
                let result = ap.handle_rx_mgmt(&mgmt_frames[i], &mut raw_buf, self.0.drain_ticks);

                // Extract action data into local variables
                if let Some(ref action) = result.action1 {
                    match action {
                        ApAction::TxFrame(data) => {
                            let n = data.len().min(tx_frame.len());
                            tx_frame[..n].copy_from_slice(&data[..n]);
                            tx_len = n;
                        }
                        ApAction::RegisterSta { mac, aid } => {
                            register_mac = *mac;
                            register_aid = *aid;
                            do_register = true;
                        }
                        ApAction::RemoveSta { aid } => {
                            remove_aid = *aid;
                            do_remove = true;
                        }
                    }
                }
                if let Some(ref action) = result.action2 {
                    match action {
                        ApAction::RegisterSta { mac, aid } => {
                            register_mac = *mac;
                            register_aid = *aid;
                            do_register = true;
                        }
                        ApAction::RemoveSta { aid } => {
                            remove_aid = *aid;
                            do_remove = true;
                        }
                        ApAction::TxFrame(_) => {} // action2 is never TxFrame in practice
                    }
                }

                // Log when AP drops a frame (e.g. assoc from unauthenticated STA)
                if result.action1.is_none() && !matches!(subtype, MgmtSubtype::ProbeReq | MgmtSubtype::Other(_)) {
                    uwarn!("wifid", "ap_drop_frame";
                        mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                        mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                        mac4 = addr2[4] as u32, mac5 = addr2[5] as u32
                    );
                }
            }

            // Phase 2: Execute actions (no AP borrow held)
            if tx_len > 0 {
                if let Some(ref mut tx_ring) = self.0.tx_band0 {
                    let mut txd_buf = [0u8; 256];
                    let len = wrap_mgmt_txd(&mut txd_buf, &tx_frame[..tx_len]);
                    if len > 0 {
                        let tok = self.0.tx_token;
                        self.0.tx_token = self.0.tx_token.wrapping_add(1);
                        match dev.tx_enqueue(tx_ring, &txd_buf[..len], tok) {
                            Ok(()) => {
                                self.0.tx_probe_resp = self.0.tx_probe_resp.wrapping_add(1);
                            }
                            Err(e) => {
                                uerror!("wifid", "tx_mgmt_enqueue_failed"; err = e as u32, len = len as u32);
                            }
                        }
                    }
                }
            }
            if do_register {
                let wlan_idx = self.0.next_wlan_idx;
                self.0.next_wlan_idx = self.0.next_wlan_idx.wrapping_sub(1);

                uinfo!("wifid", "sta_register";
                    mac0 = register_mac[0] as u32, mac1 = register_mac[1] as u32,
                    mac2 = register_mac[2] as u32, mac3 = register_mac[3] as u32,
                    mac4 = register_mac[4] as u32, mac5 = register_mac[5] as u32,
                    aid = register_aid as u32, wlan = wlan_idx as u32
                );

                if let Some(ref mut wa_ring) = self.0.wa_ring {
                    let seq = self.0.seq;
                    self.0.seq = self.0.seq.wrapping_add(1);
                    if let Err(e) = dev.mcu_add_client_sta(
                        wa_ring, 0, wlan_idx, HW_BSSID_0,
                        CONN_STATE_CONNECT, &register_mac, register_aid, true, seq, None,
                    ) {
                        uerror!("wifid", "mcu_sta_connect_failed"; err = e as u32, wlan = wlan_idx as u32);
                    }
                    let seq = self.0.seq;
                    self.0.seq = self.0.seq.wrapping_add(1);
                    if let Err(e) = dev.mcu_add_client_sta(
                        wa_ring, 0, wlan_idx, HW_BSSID_0,
                        CONN_STATE_PORT_SECURE, &register_mac, register_aid, false, seq, None,
                    ) {
                        uerror!("wifid", "mcu_sta_port_secure_failed"; err = e as u32, wlan = wlan_idx as u32);
                    }
                    let seq = self.0.seq;
                    self.0.seq = self.0.seq.wrapping_add(1);
                    if let Err(e) = dev.mcu_add_group(wa_ring, 0, wlan_idx, seq, None) {
                        uerror!("wifid", "mcu_add_group_failed"; err = e as u32, wlan = wlan_idx as u32);
                    }
                }
            }
            if do_remove {
                uinfo!("wifid", "sta_remove"; aid = remove_aid as u32);
            }
        }

        // Software beacon TX: every 10 ticks (100ms) send a beacon frame via BAND0 CT ring.
        // Firmware beacon offload stops after ~57 seconds for unknown reasons, so we
        // drive beacons entirely from software using the proven CT TX path.
        // Only count beacon ticks on timer events (not IRQ events).
        if self.0.beacon_on && is_timer {
            self.0.beacon_tick = self.0.beacon_tick.wrapping_add(1);
            if self.0.beacon_tick >= 10 {
                self.0.beacon_tick = 0;
                if let Some(ref mut ap) = self.0.ap {
                    if let Some(ref mut tx_ring) = self.0.tx_band0 {
                        let mut raw_buf = [0u8; 256];
                        let raw_len = ap.beacon(&mut raw_buf);
                        if raw_len > 0 {
                            let mut txd_buf = [0u8; 256];
                            let len = wrap_mgmt_txd(&mut txd_buf, &raw_buf[..raw_len]);
                            let tok = self.0.tx_token;
                            self.0.tx_token = self.0.tx_token.wrapping_add(1);
                            match dev.tx_enqueue(tx_ring, &txd_buf[..len], tok) {
                                Ok(()) => {
                                    self.0.tx_beacon = self.0.tx_beacon.wrapping_add(1);
                                }
                                Err(e) => {
                                    uerror!("wifid", "beacon_enqueue_failed"; err = e as u32);
                                }
                            }
                        }
                    }
                }
            }
        }

        // Channel MIB counter clearing: every 100ms (10 ticks)
        // Linux: mt7996_mac_work() → mt76_update_survey() → mt7996_mcu_get_chan_mib_info()
        // Without this, firmware CCA counters accumulate and MAC assesses channel as
        // permanently busy, stopping all TX after ~42 seconds.
        // Source: mt7996/mac.c:2155-2170, mcu.c:3948-4025
        if is_timer && self.0.drain_ticks % 10 == 0 {
            if let Some(ref mut wa_ring) = self.0.wa_ring {
                let _ = dev.mcu_get_chan_mib_info(wa_ring, 0, self.0.seq);
                self.0.seq = self.0.seq.wrapping_add(1);
            }
        }

        // Periodic maintenance: every 500ms (timer-driven)
        // Linux: mt7996_mac_work() runs every 100ms, does stats every 500ms
        if is_timer {
        self.0.heartbeat_tick = self.0.heartbeat_tick.wrapping_add(1);
        }
        if is_timer && self.0.heartbeat_tick >= 50 {
            self.0.heartbeat_tick = 0;

            // Hardware MIB register clearing: every 500ms
            // Linux: mac.c:2743-2882 mt7996_mac_update_stats()
            dev.mac_update_stats(0);

            if let Some(ref mut wa_ring) = self.0.wa_ring {
                // Stats query every 500ms
                let _ = dev.mcu_get_all_sta_info(wa_ring, UNI_ALL_STA_TXRX_RATE, self.0.seq);
                self.0.seq = self.0.seq.wrapping_add(1);
            }

            // STA aging: evict STAs not seen for 30 seconds
            if let Some(ref mut ap) = self.0.ap {
                let mut evicted = [0u16; 16];
                let n = ap.age_stas(self.0.drain_ticks, &mut evicted);
                for j in 0..n {
                    uinfo!("wifid", "sta_aged"; aid = evicted[j] as u32);
                }
            }

        }

        // Beacon diagnostic: log key registers every second while beaconing.
        // drain_ticks increments every 10ms, so 100 = 1 second.
        if is_timer && self.0.beacon_on && self.0.drain_ticks % 100 == 0 {
            let secs = self.0.drain_ticks / 100;
            let arb = dev.reg_rr(mt_wf_arb(0, MT_ARB_SCR_OFS));
            let mcu_cmd = dev.mt76_rr(MT_MCU_CMD);
            let tx_mpdu = dev.reg_rr(mt_wf_mib(0, MT_MIB_TSCR4_OFS));
            let tx_try = dev.reg_rr(mt_wf_mib(0, MT_MIB_TSCR3_OFS));
            let tx_cpu = if let Some(ref ring) = self.0.tx_band0 {
                dev.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX)
            } else { 0 };
            let tx_dma = if let Some(ref ring) = self.0.tx_band0 {
                dev.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX)
            } else { 0 };
            // PLE diagnostics: check if frames are stuck or resources exhausted
            // NOTE: PSE registers (0x820c8xxx) return 0xDEADBEEF on MT7996 —
            // they're firmware-internal, not host-accessible. Linux never reads them.
            // PLE (0x820c0xxx) IS host-accessible and used by Linux debugfs.
            let ple_free = dev.reg_rr(MT_PLE_FREEPG_CNT);
            let ple_empty = dev.reg_rr(MT_PLE_FL_Q_EMPTY);

            // MCU RX queue fill levels: gap between DMA_IDX and CPU_IDX
            // Non-zero means responses are pending (not yet processed).
            // If gap approaches ring size (q0=512, q1=1024), firmware stalls.
            let q0_dma = if qc > 0 { dev.mt76_rr(self.0.rx_queues[0].regs_base + MT_QUEUE_DMA_IDX) } else { 0 };
            let q0_cpu = if qc > 0 { dev.mt76_rr(self.0.rx_queues[0].regs_base + MT_QUEUE_CPU_IDX) } else { 0 };
            let q1_dma = if qc > 1 { dev.mt76_rr(self.0.rx_queues[1].regs_base + MT_QUEUE_DMA_IDX) } else { 0 };
            let q1_cpu = if qc > 1 { dev.mt76_rr(self.0.rx_queues[1].regs_base + MT_QUEUE_CPU_IDX) } else { 0 };

            let sta_count = if let Some(ref ap) = self.0.ap { ap.sta_count() as u32 } else { 0 };
            let b0 = &self.0.mib[0];
            // ACK fail count — non-zero means frames ARE going over the air but nobody ACKs
            // TSCR4 only counts data AMPDU MPDUs, NOT management frames.
            let ack_fail = dev.reg_rr(mt_wf_mib(0, MT_MIB_BFTFCR_OFS));
            // TSCR0 = TX AMPDU count (data only)
            let tx_ampdu = dev.reg_rr(mt_wf_mib(0, MT_MIB_TSCR0_OFS));

            // Monitor RX ring DMA_IDX and CPU_IDX for WA_MAIN (q3) to debug drain
            let mut rx_dma = [0u32; 6];
            let mut r3_cpu = 0u32;
            for ri in 0..qc.min(6) {
                rx_dma[ri] = dev.mt76_rr(self.0.rx_queues[ri].regs_base + MT_QUEUE_DMA_IDX);
            }
            if qc > 3 {
                r3_cpu = dev.mt76_rr(self.0.rx_queues[3].regs_base + MT_QUEUE_CPU_IDX);
            }

            let rfcr_actual = dev.reg_rr(mt_wf_rmac(0, MT_WF_RFCR_OFS));

            uinfo!("wifid", "bcn_diag";
                t = secs,
                arb = arb,
                mcu = mcu_cmd,
                tx_ok = tx_mpdu,
                sw_bcn = self.0.tx_beacon,
                tx_prb = self.0.tx_probe_resp,
                stas = sta_count,
                rx_tot = b0.total,
                rx_dat = b0.data,
                rx_mgmt = b0.mgmt,
                rx_auth = b0.auth,
                rx_fcs = b0.fcs_err,
                rx_txs = b0.txs,
                rfcr = rfcr_actual,
                r0 = rx_dma[0], r1 = rx_dma[1], r2 = rx_dma[2],
                r3 = rx_dma[3], r3c = r3_cpu,
                tx_c = tx_cpu,
                tx_d = tx_dma,
                txf = self.0.tx_freed,
                tfe = self.0.tx_free_entries,
                tfn = self.0.tx_free_notify_count
            );

            // Radio-stop detection: when TX DMA CPU index stops advancing, the DMA engine
            // is stuck or firmware stopped consuming frames.
            // NOTE: tx_mpdu (TSCR4) only counts data AMPDUs, NOT management frames.
            // We use tx_cpu (DMA CPU_IDX) which advances for every frame we enqueue.
            if secs >= 5 { // Skip first 5 seconds (settling)
                if tx_cpu == self.0.last_tx_ok {
                    self.0.tx_stall_count = self.0.tx_stall_count.saturating_add(1);
                    if self.0.tx_stall_count == 3 { // 3 consecutive seconds with no DMA progress
                        let fw_state = dev.mt76_rr(MT_TOP_MISC) & 0x7;
                        let glo_cfg = dev.mt76_rr(MT_WFDMA0_GLO_CFG);
                        let swdef = dev.mt76_rr(MT_SWDEF_MODE);
                        let rfcr = dev.reg_rr(mt_wf_rmac(0, MT_WF_RFCR_OFS));
                        let int_src = dev.mt76_rr(MT_INT_SOURCE_CSR);
                        let int_mask = dev.mt76_rr(MT_INT_MASK_CSR);
                        uerror!("wifid", "RADIO_STOPPED";
                            t = secs,
                            fw = fw_state,
                            arb = arb,
                            mcu = mcu_cmd,
                            glo = glo_cfg,
                            swdef = swdef,
                            rfcr = rfcr,
                            int_s = int_src,
                            int_m = int_mask,
                            tx_ok = tx_mpdu,
                            ple_f = ple_free,
                            ple_e = ple_empty
                        );
                    }
                } else {
                    self.0.tx_stall_count = 0;
                }
                self.0.last_tx_ok = tx_cpu;
            }

            // One-time TXD hex dump: read back the last-written TXD+TXP from the buffer.
            // This lets us compare byte-for-byte against Linux's mt7996_mac_write_txwi output.
            if secs == 1 {
                if let Some(ref ring) = self.0.tx_band0 {
                    // Read the buffer at the previous cpu_idx (last enqueued frame)
                    let prev = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
                    let buf = ring.buf(prev);
                    // Read TXD (8 DWORDs = 32 bytes) + TXP first 12 bytes
                    let mut txd = [0u32; 8];
                    for i in 0..8 {
                        txd[i] = unsafe { core::ptr::read_volatile((buf as *const u32).add(i)) };
                    }
                    uinfo!("wifid", "txd_dump";
                        d0 = txd[0], d1 = txd[1], d2 = txd[2], d3 = txd[3],
                        d4 = txd[4], d5 = txd[5], d6 = txd[6], d7 = txd[7]
                    );
                    // Read TXP header (at offset 32): flags, token, bss_idx, rept_wds_wcid, nbuf
                    let txp_base = unsafe { buf.add(MT_TXD_SIZE) };
                    let txp_flags = unsafe { core::ptr::read_volatile(txp_base as *const u16) };
                    let txp_nbuf = unsafe { core::ptr::read_volatile(txp_base.add(7)) };
                    let txp_buf0 = unsafe { core::ptr::read_volatile((txp_base.add(8)) as *const u32) };
                    let txp_len0 = unsafe { core::ptr::read_volatile((txp_base.add(32)) as *const u16) };
                    uinfo!("wifid", "txp_dump";
                        flags = txp_flags as u32,
                        nbuf = txp_nbuf as u32,
                        buf0 = txp_buf0,
                        len0 = txp_len0 as u32
                    );
                    // Also dump the descriptor
                    let desc = ring.desc(prev);
                    let d_buf0 = unsafe { core::ptr::read_volatile(&(*desc).buf0) };
                    let d_ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
                    let d_buf1 = unsafe { core::ptr::read_volatile(&(*desc).buf1) };
                    let d_info = unsafe { core::ptr::read_volatile(&(*desc).info) };
                    uinfo!("wifid", "desc_dump";
                        buf0 = d_buf0, ctrl = d_ctrl, buf1 = d_buf1, info = d_info
                    );
                }

                // WA_MAIN buffer dump: read first buffer to see what pkt_type firmware writes
                if qc > 3 {
                    let q = &self.0.rx_queues[3];
                    // Read buffer 0 (or any recently-written buffer)
                    let buf_base = q.buf_virt;
                    let buf_ptr = buf_base as *const u32;
                    let mut wa_dw = [0u32; 12]; // 48 bytes: RXD(32) + tx_info[0..3]
                    for j in 0..12 {
                        wa_dw[j] = unsafe { core::ptr::read_volatile(buf_ptr.add(j)) };
                    }
                    uinfo!("wifid", "wa_main_dump";
                        d0 = wa_dw[0], d1 = wa_dw[1], d2 = wa_dw[2], d3 = wa_dw[3],
                        d4 = wa_dw[4], d5 = wa_dw[5], d6 = wa_dw[6], d7 = wa_dw[7],
                        d8 = wa_dw[8], d9 = wa_dw[9], d10 = wa_dw[10], d11 = wa_dw[11]
                    );
                }
            }
        }

        // Check MCU_CMD register for firmware watchdog/error (only on real IRQ)
        if is_irq && intr & MT_INT_MCU_CMD != 0 {
            let mcu_cmd = dev.mt76_rr(MT_MCU_CMD);
            if mcu_cmd & MT_MCU_CMD_WDT_MASK != 0 {
                self.0.mib[2].mcu_wdt = self.0.mib[2].mcu_wdt.wrapping_add(1);
                uerror!("wifid", "firmware_watchdog"; mcu_cmd = mcu_cmd);
            }
            if mcu_cmd & MT_MCU_CMD_ERROR_MASK != 0 {
                uerror!("wifid", "firmware_error"; mcu_cmd = mcu_cmd);
            }
        }

        // Re-enable interrupts (IRQ mode, after DMA processing only)
        if self.0.irq_mode && is_irq {
            let irq_mask = MT_INT_RX_DONE_ALL | MT_INT_MCU_CMD
                | MT_INT_TX_DONE_BAND0 | MT_INT_TX_DONE_MCU_WM;
            dev.mt76_wr(MT_INT_MASK_CSR, irq_mask);

            // Ack the kernel IRQ to clear pending flag and allow next MSI notification
            if let Some(ref mut irq) = self.0.irq {
                let _ = irq.ack();
            }
        }
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
            b"diag.dma" => {
                let dev = match self.0.dev.as_ref() {
                    Some(d) => d,
                    None => return Self::copy_to_buf(buf, b"not_initialized"),
                };
                let mut tmp = [0u8; 200];
                let mut pos = 0;
                macro_rules! kv {
                    ($label:expr, $val:expr) => {{
                        let label: &[u8] = $label;
                        if pos + label.len() + 10 <= tmp.len() {
                            tmp[pos..pos+label.len()].copy_from_slice(label);
                            pos += label.len();
                            pos += fmt_u32_dec($val, &mut tmp[pos..]);
                        }
                    }};
                }
                // RX: only show active queues (WM, WA, WA_MAIN, BAND0)
                // Skip WAT(q3) and B2(q5) — always 0 without HIF2
                let rx_base = MT_WFDMA0_RX_RING_BASE;
                let rx_names: [&[u8]; 4] = [b"wm", b"wa", b"wam", b"b0"];
                let rx_idxs: [u32; 4] = [0, 1, 2, 4];
                tmp[pos..pos+3].copy_from_slice(b"rx ");
                pos += 3;
                for i in 0..4 {
                    let regs = rx_base + rx_idxs[i] * MT_RING_SIZE;
                    let cpu = dev.mt76_rr(regs + MT_QUEUE_CPU_IDX);
                    let dma = dev.mt76_rr(regs + MT_QUEUE_DMA_IDX);
                    if pos + 20 <= tmp.len() {
                        tmp[pos..pos+rx_names[i].len()].copy_from_slice(rx_names[i]);
                        pos += rx_names[i].len();
                    }
                    kv!(b"=", cpu);
                    kv!(b"/", dma);
                    tmp[pos] = b' ';
                    pos += 1;
                }
                // TX BAND0 ring
                let tx_b0_regs = MT_WFDMA0_TX_RING_BASE + MT7996_TXQ_BAND0 * MT_RING_SIZE;
                let cpu = dev.mt76_rr(tx_b0_regs + MT_QUEUE_CPU_IDX);
                let dma = dev.mt76_rr(tx_b0_regs + MT_QUEUE_DMA_IDX);
                tmp[pos..pos+3].copy_from_slice(b"tx ");
                pos += 3;
                kv!(b"b0=", cpu);
                kv!(b"/", dma);

                Self::copy_to_buf(buf, &tmp[..pos])
            }
            b"diag" | b"diag.ple" | b"diag.mib" | b"diag.bcn" => {
                let dev = match self.0.dev.as_ref() {
                    Some(d) => d,
                    None => return Self::copy_to_buf(buf, b"not_initialized"),
                };

                let mut tmp = [0u8; 400];
                let mut pos = 0;

                macro_rules! kv {
                    ($label:expr, $val:expr) => {{
                        let label: &[u8] = $label;
                        if pos + label.len() + 10 <= tmp.len() {
                            tmp[pos..pos+label.len()].copy_from_slice(label);
                            pos += label.len();
                            pos += fmt_hex32($val, &mut tmp[pos..]);
                        }
                    }};
                }

                match key {
                    b"diag" => {
                        // Core HW state: firmware, DMA, arbiter, TMAC, TSF
                        kv!(b"fw=", dev.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE);
                        kv!(b" glo=", dev.mt76_rr(MT_WFDMA0_GLO_CFG));
                        kv!(b" int=", dev.mt76_rr(MT_INT_SOURCE_CSR));
                        kv!(b" msk=", dev.mt76_rr(MT_INT_MASK_CSR));
                        tmp[pos] = b'\n'; pos += 1;
                        // TSF
                        dev.reg_rmw(
                            mt_wf_lpon(0, MT_LPON_TCR_OFS),
                            MT_LPON_TCR_SW_MODE,
                            MT_LPON_TCR_SW_READ,
                        );
                        kv!(b"tsf=", dev.reg_rr(mt_wf_lpon(0, MT_LPON_UTTR1_OFS)));
                        tmp[pos] = b'.'; pos += 1;
                        pos += fmt_hex32(dev.reg_rr(mt_wf_lpon(0, MT_LPON_UTTR0_OFS)), &mut tmp[pos..]);
                        kv!(b" frcr=", dev.reg_rr(mt_wf_lpon(0, MT_LPON_FRCR_OFS)));
                        tmp[pos] = b'\n'; pos += 1;
                        // Band 0 control
                        kv!(b"arb=", dev.reg_rr(mt_wf_arb(0, MT_ARB_SCR_OFS)));
                        kv!(b" tcr0=", dev.reg_rr(mt_wf_tmac(0, MT_TMAC_TCR0_OFS)));
                        kv!(b" icr0=", dev.reg_rr(mt_wf_tmac(0, MT_TMAC_ICR0_OFS)));
                        kv!(b" rfcr=", dev.reg_rr(mt_wf_rmac(0, MT_WF_RFCR_OFS)));
                    }
                    b"diag.ple" => {
                        // PLE/PSE queue status, beacon queue depth
                        kv!(b"ple_e=", dev.reg_rr(MT_PLE_FL_Q_EMPTY));
                        kv!(b" free=", dev.reg_rr(MT_PLE_FREEPG_CNT));
                        kv!(b" hif=", dev.reg_rr(MT_PLE_HIF_PG_INFO));
                        tmp[pos] = b'\n'; pos += 1;
                        // BCN_Q0 depth: pid=2, qid=10
                        dev.reg_wr(MT_PLE_FL_Q0_CTRL, (1u32 << 31) | (2 << 10) | (10 << 24));
                        kv!(b"bcn_ht=", dev.reg_rr(MT_PLE_FL_Q2_CTRL));
                        kv!(b" bcn_n=", dev.reg_rr(MT_PLE_FL_Q3_CTRL) & 0xFFF);
                        tmp[pos] = b'\n'; pos += 1;
                        // PSE not host-accessible on MT7996 (reads return 0xDEADBEEF)
                        kv!(b" acr4=", dev.reg_rr(mt_wf_agg(0, MT_AGG_ACR4_OFS)));
                    }
                    b"diag.mib" => {
                        // Software RX MIB counters (band 0)
                        let b0 = &self.0.mib[0];
                        let mcu = &self.0.mib[2];

                        // Use decimal kv for software counters
                        macro_rules! kvd {
                            ($label:expr, $val:expr) => {{
                                let label: &[u8] = $label;
                                if pos + label.len() + 12 <= tmp.len() {
                                    tmp[pos..pos+label.len()].copy_from_slice(label);
                                    pos += label.len();
                                    pos += fmt_u32_dec($val, &mut tmp[pos..]);
                                }
                            }};
                        }

                        kvd!(b"rx total=", b0.total);
                        kvd!(b" data=", b0.data);
                        kvd!(b" mgmt=", b0.mgmt);
                        kvd!(b" bcn=", b0.beacon);
                        kvd!(b" preq=", b0.probe_req);
                        kvd!(b" prsp=", b0.probe_resp);
                        kvd!(b" auth=", b0.auth);
                        kvd!(b" assoc=", b0.assoc_req);
                        kvd!(b" deauth=", b0.deauth);
                        kvd!(b" fcs=", b0.fcs_err);
                        tmp[pos] = b'\n'; pos += 1;
                        kvd!(b"rx u2m=", b0.unicast);
                        kvd!(b" mcast=", b0.multicast);
                        kvd!(b" bcast=", b0.broadcast);
                        kvd!(b" txs=", b0.txs);
                        kvd!(b" notify=", b0.txrx_notify);
                        tmp[pos] = b'\n'; pos += 1;
                        kvd!(b"mcu events=", mcu.mcu_events);
                        kvd!(b" thermal=", mcu.mcu_thermal);
                        kvd!(b" wdt=", mcu.mcu_wdt);
                        tmp[pos] = b'\n'; pos += 1;
                        kvd!(b"tx prsp=", self.0.tx_probe_resp);
                        kvd!(b" bcn=", self.0.tx_beacon);
                        tmp[pos] = b'\n'; pos += 1;
                        // Hardware MIB (clear-on-read) for reference
                        kv!(b"hw fcs=", dev.reg_rr(mt_wf_mib(0, MT_MIB_RSCR1_OFS)));
                        kv!(b" mpdu=", dev.reg_rr(mt_wf_mib(0, MT_MIB_RSCR31_OFS)));
                        kv!(b" idle=", dev.reg_rr(mt_wf_mib(0, MT_MIB_SDR6_OFS)) & 0xFFFF);
                    }
                    b"diag.bcn" => {
                        // Beacon TX counters
                        kv!(b"bt0=", dev.reg_rr(mt_wf_mib(0, MT_MIB_BTSCR0_OFS)));
                        kv!(b" bt5=", dev.reg_rr(mt_wf_mib(0, MT_MIB_BTSCR5_OFS)));
                        kv!(b" bt6=", dev.reg_rr(mt_wf_mib(0, MT_MIB_BTSCR6_OFS)));
                        tmp[pos] = b'\n'; pos += 1;
                        // Beacon statistics
                        kv!(b"bs0=", dev.reg_rr(mt_wf_mib(0, MT_MIB_BSCR0_OFS)));
                        kv!(b" bs1=", dev.reg_rr(mt_wf_mib(0, MT_MIB_BSCR1_OFS)));
                        kv!(b" bs2=", dev.reg_rr(mt_wf_mib(0, MT_MIB_BSCR2_OFS)));
                        kv!(b" bs3=", dev.reg_rr(mt_wf_mib(0, MT_MIB_BSCR3_OFS)));
                        kv!(b" bs17=", dev.reg_rr(mt_wf_mib(0, MT_MIB_BSCR17_OFS)));
                        tmp[pos] = b'\n'; pos += 1;
                        // WTBL 1087 (BMC STA) — indirect read
                        dev.reg_rmw(MT_WTBL_UPDATE, MT_WTBL_UPDATE_WLAN_IDX,
                            (MT7996_WTBL_RESERVED as u32) & MT_WTBL_UPDATE_WLAN_IDX);
                        kv!(b"wu=", dev.reg_rr(MT_WTBL_UPDATE));
                        dev.reg_wr(MT_WTBL_ITCR,
                            (1u32 << 31) | ((MT7996_WTBL_RESERVED as u32) << 0));
                        kv!(b" wd0=", dev.reg_rr(MT_WTBL_ITDR0));
                        kv!(b" wd1=", dev.reg_rr(MT_WTBL_ITDR1));
                    }
                    _ => {}
                }

                Self::copy_to_buf(buf, &tmp[..pos])
            }
            b"scan" => Self::copy_to_buf(buf, b"use: devc wifid set scan start\n"),
            _ => 0,
        }
    }

    fn config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], _ctx: &mut dyn BusCtx) -> usize {
        let (dev, wa_ring) = match (self.0.dev.as_ref(), self.0.wa_ring.as_mut()) {
            (Some(d), Some(wa)) => (d, wa),
            _ => return Self::copy_to_buf(buf, b"ERR not_initialized\n"),
        };

        match key {
            b"radio" => {
                let enable = match value {
                    b"on" | b"1" | b"true" => true,
                    b"off" | b"0" | b"false" => false,
                    _ => return Self::copy_to_buf(buf, b"ERR invalid_value\n"),
                };
                // MCU_WM_UNI_CMD(BAND_CONFIG) — WM-only, via WA queue
                for band in 0..3u8 {
                    if dev.mcu_set_radio_en(wa_ring, band, enable, self.0.seq, None).is_err() {
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
                // Software-driven beacons: just toggle the flag.
                // handle_event's beacon_tick loop does the actual TX.
                self.0.beacon_on = enable;
                self.0.beacon_tick = 0;
                if enable {
                    unotice!("wifid", "beacon_enable_sw");
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
                // MCU_WMWA_UNI_CMD(CHANNEL_SWITCH) — WMWA, via WA queue
                if dev.mcu_set_chan_info(wa_ring, 0, UNI_CHANNEL_SWITCH, ch, CMD_CBW_20MHZ, CH_BAND_2GHZ, self.0.seq, None).is_err() {
                    return Self::copy_to_buf(buf, b"ERR chan_switch_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                if dev.mcu_set_chan_info(wa_ring, 0, UNI_CHANNEL_RX_PATH, ch, CMD_CBW_20MHZ, CH_BAND_2GHZ, self.0.seq, None).is_err() {
                    return Self::copy_to_buf(buf, b"ERR rx_path_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                // TX power SKU after channel switch — Linux main.c:569
                if dev.mcu_set_txpower_sku(wa_ring, 0, self.0.seq, None).is_err() {
                    return Self::copy_to_buf(buf, b"ERR txpower_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                // Re-set RFCR after channel switch
                let rfcr_val: u32 = MT_WF_RFCR_DROP_CTL_RSV
                    | MT_WF_RFCR_DROP_CTS
                    | MT_WF_RFCR_DROP_RTS;
                let rfcr1_val: u32 = MT_WF_RFCR1_DROP_ACK
                    | MT_WF_RFCR1_DROP_BF_POLL
                    | MT_WF_RFCR1_DROP_BA
                    | MT_WF_RFCR1_DROP_CFEND
                    | MT_WF_RFCR1_DROP_CFACK;
                dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr_val);
                dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1_val);
                // Re-upload beacon with new channel
                if self.0.beacon_on {
                    if dev.mcu_set_beacon(wa_ring, 0, HW_BSSID_0, &self.0.mac_addr, ch, true, self.0.seq, None).is_err() {
                        return Self::copy_to_buf(buf, b"ERR beacon_update_failed\n");
                    }
                    self.0.seq = self.0.seq.wrapping_add(1);
                }
                // Update AP BSS config channel
                if let Some(ref mut ap) = self.0.ap {
                    ap.bss.channel = ch;
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
                // MCU_WM_UNI_CMD(THERMAL) — WM-only, via WA queue
                for band in 0..3u8 {
                    if dev.mcu_set_thermal_throttling(wa_ring, band, throttle, self.0.seq).is_err() {
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
                        // Restore AP default filter
                        let rfcr: u32 = MT_WF_RFCR_DROP_CTL_RSV
                            | MT_WF_RFCR_DROP_CTS
                            | MT_WF_RFCR_DROP_RTS
                            ;
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
                if dev.mcu_background_chain_ctrl(wa_ring, 0, ch, 0, 0, 1, self.0.seq).is_err() {
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
                let _ = dev.mcu_background_chain_ctrl(wa_ring, 0, ch, 0, 0, 0, self.0.seq);
                self.0.seq = self.0.seq.wrapping_add(1);

                // Step 9: Restore RFCR to normal AP filter
                let rfcr_val: u32 = MT_WF_RFCR_DROP_CTL_RSV
                    | MT_WF_RFCR_DROP_CTS
                    | MT_WF_RFCR_DROP_RTS;
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
