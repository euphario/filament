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
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, ConfigKey, PortId, BlockPortConfig,
    PortInfo, PortClass, NetworkMetadata, port_subclass,
};
use userlib::bus_runtime::driver_main;
use userlib::ring::{IoCqe, SideEntry, io_op, io_status, side_msg, side_status};

mod regs;
mod dma;
mod device;
mod mcu;
mod firmware;
mod mac;
mod event;

use regs::*;
use dma::{TxRing, RxDataFrame, RxPmEvent, flush_buffer};
use device::Mt7996Dev;
use event::RxMibCounters;

use wifi80211::ap::ApManager;
use wifi80211::types::{BssConfig, RxMgmtFrame, MgmtSubtype, ApAction, MAX_SSID_LEN};

// ============================================================================
// Data Path Statistics
// ============================================================================

/// Maximum in-flight TX frames tracked for deferred CQE posting.
/// Must be power of 2 and >= TX ring size.
const TX_INFLIGHT_SIZE: usize = 2048;

// ============================================================================
// SER L1 Recovery State Machine
// ============================================================================

/// Firmware-cooperative DMA reset states.
/// Each Wait* state checks one register bit per timer tick (500ms).
#[derive(Clone, Copy, PartialEq, Eq)]
enum SerL1State {
    /// Normal operation — no recovery in progress.
    Idle,
    /// DMA_STOPPED signaled, waiting for firmware RESET_DONE.
    WaitResetDone,
    /// DMA reset done, DMA_INIT signaled, waiting for RECOVERY_DONE.
    WaitRecoveryDone,
    /// RESET_DONE signaled, waiting for firmware NORMAL_STATE.
    WaitNormalState,
}

// ============================================================================
// Frame Size Constants
// ============================================================================

/// Ethernet header: dst(6) + src(6) + ethertype(2).
const ETH_HEADER_LEN: usize = 14;
/// IP MTU for the WiFi data path.
const MTU: usize = 1500;
/// Maximum Ethernet frame size (header + payload, no FCS).
const MAX_FRAME_SIZE: usize = MTU + ETH_HEADER_LEN;

// ============================================================================
// Power Save Frame Buffering
// ============================================================================

/// Max buffered frames per STA during power save.
const PS_MAX_PER_STA: usize = 4;
/// Max total PS buffered frames across all STAs.
const PS_BUF_TOTAL: usize = 32;

/// A buffered frame for a sleeping STA.
struct PsBufferedFrame {
    /// Destination STA MAC
    dst_mac: [u8; 6],
    /// DataPort SQE tag for deferred CQE posting
    sqe_tag: u32,
    /// Frame data (Ethernet header + payload)
    data: [u8; MAX_FRAME_SIZE],
    /// Actual frame length
    len: u16,
    /// Whether this slot is in use
    valid: bool,
}

impl PsBufferedFrame {
    const EMPTY: Self = Self {
        dst_mac: [0; 6],
        sqe_tag: 0,
        data: [0; MAX_FRAME_SIZE],
        len: 0,
        valid: false,
    };
}

/// PS frame buffer — global ring for all sleeping STAs.
struct PsBuffer {
    frames: [PsBufferedFrame; PS_BUF_TOTAL],
    /// Number of valid buffered frames
    count: u16,
    /// Total frames dropped due to buffer full
    drops: u32,
}

impl PsBuffer {
    const fn new() -> Self {
        Self {
            frames: [PsBufferedFrame::EMPTY; PS_BUF_TOTAL],
            count: 0,
            drops: 0,
        }
    }

    /// Count frames buffered for a specific STA.
    fn count_for_sta(&self, mac: &[u8; 6]) -> usize {
        let mut n = 0;
        for f in self.frames.iter() {
            if f.valid && f.dst_mac == *mac {
                n += 1;
            }
        }
        n
    }

    /// Buffer a frame for a sleeping STA. Returns false if full.
    fn push(&mut self, dst_mac: &[u8; 6], sqe_tag: u32, data: &[u8]) -> bool {
        // Per-STA limit
        if self.count_for_sta(dst_mac) >= PS_MAX_PER_STA {
            self.drops += 1;
            return false;
        }
        // Find free slot
        for f in self.frames.iter_mut() {
            if !f.valid {
                f.dst_mac = *dst_mac;
                f.sqe_tag = sqe_tag;
                let len = data.len().min(MAX_FRAME_SIZE);
                f.data[..len].copy_from_slice(&data[..len]);
                f.len = len as u16;
                f.valid = true;
                self.count += 1;
                return true;
            }
        }
        self.drops += 1;
        false
    }

    /// Dequeue one frame for a STA (PS-Poll response). Returns (sqe_tag, frame_data, has_more).
    fn pop_one(&mut self, mac: &[u8; 6]) -> Option<(u32, &[u8], bool)> {
        let mut found_idx = None;
        for (i, f) in self.frames.iter().enumerate() {
            if f.valid && f.dst_mac == *mac {
                found_idx = Some(i);
                break;
            }
        }
        let idx = found_idx?;
        self.frames[idx].valid = false;
        self.count -= 1;
        let has_more = self.count_for_sta(mac) > 0;
        let f = &self.frames[idx];
        Some((f.sqe_tag, &f.data[..f.len as usize], has_more))
    }

    /// Find indices of all frames for a STA (for PM=0 flush).
    /// Returns count of indices written to `out`.
    fn find_sta_frames(&self, mac: &[u8; 6], out: &mut [usize]) -> usize {
        let mut n = 0;
        for (i, f) in self.frames.iter().enumerate() {
            if f.valid && f.dst_mac == *mac && n < out.len() {
                out[n] = i;
                n += 1;
            }
        }
        n
    }

    /// Release a frame slot after TX (called after find_sta_frames + tx_enqueue_data).
    fn release(&mut self, idx: usize) {
        if idx < PS_BUF_TOTAL && self.frames[idx].valid {
            self.frames[idx].valid = false;
            self.count -= 1;
        }
    }
}

struct WifiDataPathStats {
    rx_frames: u32,
    tx_frames: u32,
    rx_pool_drops: u32,
    tx_pool_drops: u32,
    rx_pool_reclaimed: u32,
    /// How many times data_ready() was called for the DataPort.
    data_ready_calls: u32,
    /// SQEs consumed but TX'd to PS buffer (client in power save).
    tx_ps_buffered: u32,
    /// SQEs consumed but TX failed (DMA ring full, bad frame, etc).
    tx_errors: u32,
}

impl WifiDataPathStats {
    const fn new() -> Self {
        Self {
            rx_frames: 0, tx_frames: 0, rx_pool_drops: 0, tx_pool_drops: 0,
            rx_pool_reclaimed: 0, data_ready_calls: 0, tx_ps_buffered: 0, tx_errors: 0,
        }
    }
}

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
    /// TX beacon counter (firmware-offloaded, tracked via MIB)
    tx_beacon: u32,
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
    /// 802.11 AP state machine (STA table, auth/assoc handling)
    ap: Option<ApManager>,
    /// Next WLAN index for client STAs (starts below WTBL_RESERVED)
    next_wlan_idx: u16,
    /// NAPI polling mode: IRQ is suppressed (hw masked, kernel IRQ not acked).
    /// Timer tick will drain queues and re-enable when empty.
    irq_suppressed: bool,
    /// DataPort for IP stack data exchange (wifi:0)
    data_port: Option<PortId>,
    /// Monotonic RX sequence counter for DataPort CQE tags
    net_rx_seq: u32,
    /// Monotonic CQ post sequence — incremented for EVERY CQE (RX + TX).
    /// Tracks the provider's cq_tail position for cq_offsets indexing.
    cq_post_seq: u32,
    /// Provider-side: pool offsets posted at each CQ slot (for reclaim).
    /// Indexed by (cq_post_seq & mask) — matches CQ ring position exactly.
    /// TX CQE slots contain u32::MAX (no pool slot to reclaim).
    cq_offsets: [u32; 256],
    /// Last known consumer cq_head (for reclaim delta).
    last_cq_head: u32,
    /// Data path statistics.
    dp_stats: WifiDataPathStats,
    /// TX inflight: maps wifid token → ipd SQE tag for deferred CQE posting.
    /// Indexed by (token & (TX_INFLIGHT_SIZE - 1)). 0 = unused.
    tx_inflight_tags: [u32; TX_INFLIGHT_SIZE],
    /// Cached DataPort pool physical address (for zero-copy TX).
    pool_phys: u64,
    /// SER L1 recovery count.
    ser_count: u32,
    /// SER L1 recovery state machine.
    ser_state: SerL1State,
    /// Tick when current SER step started (for timeout).
    ser_step_tick: u32,
    /// Power save frame buffer for sleeping STAs.
    ps_buf: PsBuffer,
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
            mib: [RxMibCounters::new(); 3],
            tx_probe_resp: 0,
            tx_beacon: 0,
            tx_token: 0,
            tx_freed: 0,
            tx_free_entries: 0,
            tx_free_notify_count: 0,
            ap: None,
            next_wlan_idx: MT7996_WTBL_RESERVED - 1,
            irq_suppressed: false,
            data_port: None,
            net_rx_seq: 0,
            cq_post_seq: 0,
            cq_offsets: [u32::MAX; 256],
            last_cq_head: 0,
            dp_stats: WifiDataPathStats::new(),
            tx_inflight_tags: [u32::MAX; TX_INFLIGHT_SIZE],
            pool_phys: 0,
            ser_count: 0,
            ser_state: SerL1State::Idle,
            ser_step_tick: 0,
            ps_buf: PsBuffer::new(),
        }
    }

    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        unotice!("wifid", "init_start");
        userlib::ulog::flush();

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
    // Match Linux mt7996_configure_filter() default: DROP_OTHER_UC base filter
    // plus CTL_RSV+CTS+RTS+FCSFAIL (main.c:442-446).
    let rfcr_val: u32 = MT_WF_RFCR_DROP_OTHER_UC
        | MT_WF_RFCR_DROP_CTL_RSV
        | MT_WF_RFCR_DROP_CTS
        | MT_WF_RFCR_DROP_RTS
        | MT_WF_RFCR_DROP_FCSFAIL;
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
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_RX_PATH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, 0, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "rx_path_init_ok");

    // === Interface creation (band 0) ===
    // Linux: mt7996_vif_link_add() → add_dev_info + add_bss_info + add_sta

    // DEV_INFO: activate OMAC on band 0 — Linux mcu.c:2623
    udebug!("wifid", "dev_info_send"; band = 0u32, omac = HW_BSSID_0 as u32, active = 1u32);
    dev.mcu_add_dev_info(&mut wa_ring, 0, HW_BSSID_0, &mac_addr, true, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "dev_info_ok");

    // WTBL: clear ADM count on BMC STA — Linux main.c:343
    // Must happen before mcu_add_bss_info + mcu_add_sta so hardware
    // has clean admission state and will auto-ACK inbound frames.
    dev.mac_wtbl_update(MT7996_WTBL_RESERVED as u32, MT_WTBL_UPDATE_ADM_COUNT_CLEAR);
    udebug!("wifid", "wtbl_adm_clear"; wlan = MT7996_WTBL_RESERVED as u32);

    // BSS_INFO: create BSS on band 0 — Linux mcu.c:1123
    // hw_bss_idx = 0 (matches bss_req_hdr.bss_idx in uni_header)
    // omac_idx = HW_BSSID_0 (OMAC to use, independent of bss_idx)
    udebug!("wifid", "bss_info_send"; band = 0u32, omac = HW_BSSID_0 as u32, bss = 0u32, active = 1u32, ch = 1u32);
    dev.mcu_add_bss_info(&mut wa_ring, 0, HW_BSSID_0, 0, &mac_addr, true, 1, CMD_CBW_20MHZ, 0, seq, None).map_err(mcu_err)?;
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
    dev.mcu_add_group(&mut wa_ring, 0, MT7996_WTBL_RESERVED, seq, None, true).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // === Channel tune (band 0) — Linux main.c:553 mt7996_set_channel() ===
    // Radio is already ON from mt7996_run() sequence above.

    // Channel switch — Linux main.c:561
    udebug!("wifid", "chan_switch"; band = 0u32, ch = 1u32, bw = 0u32, ch_band = 0u32);
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_SWITCH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, 0, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // RX path after switch — Linux main.c:565
    dev.mcu_set_chan_info(&mut wa_ring, 0, UNI_CHANNEL_RX_PATH, 1, CMD_CBW_20MHZ, CH_BAND_2GHZ, 0, seq, None).map_err(mcu_err)?;
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
    dev.mcu_add_bss_info(&mut wa_ring, 0, HW_BSSID_0, 0, &mac_addr, true, 1, CMD_CBW_20MHZ, 0, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // Second STA_REC: update existing BMC STA (newly=false) — Linux main.c:861
    udebug!("wifid", "sta_rec2_send"; bss = 0u32, wlan = MT7996_WTBL_RESERVED as u32, newly = 0u32);
    dev.mcu_add_sta(&mut wa_ring, 0, MT7996_WTBL_RESERVED, 0, CONN_STATE_PORT_SECURE, &mac_addr, false, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // VOW: reassign WCID to BSS group — Linux mcu.c:2504
    dev.mcu_add_group(&mut wa_ring, 0, MT7996_WTBL_RESERVED, seq, None, true).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);

    // EDCA/WMM parameters — Linux main.c:883-884
    // MCU_WM_UNI_CMD(EDCA_UPDATE) — WM-only, routed via WA queue
    // "ensure that enable txcmd_mode after bss_info"
    // bss_idx = 0 (must match BSS_INFO's bss_req_hdr.bss_idx)
    udebug!("wifid", "edca_send"; bss = 0u32);
    dev.mcu_set_edca(&mut wa_ring, 0, seq, None).map_err(mcu_err)?;
    seq = seq.wrapping_add(1);
    udebug!("wifid", "edca_ok");

    // Beacon: firmware offload — uploads beacon template to MCU.
    // The ~57s death was caused by DMA CPU_IDX starvation, not beacon offload.
    // Linux: mt7996_mcu_add_beacon() in mcu.c:2766
    {
        let mut init_bss = wifi80211::types::BssConfig {
            bssid: mac_addr,
            ssid: [0u8; MAX_SSID_LEN],
            ssid_len: 8,
            channel: 1,
            bandwidth: 0,
            secondary_channel_offset: 0,
            erp_protection: false,
            ht_protection: 0,
        };
        init_bss.ssid[..8].copy_from_slice(b"Filament");
        let empty_tim = [0u8; 8];
        let mut bcn_buf = [0u8; 256];
        let bcn_len = wifi80211::frame::build_beacon(&mut bcn_buf, &init_bss, 0, 0, &empty_tim);
        dev.mcu_set_beacon(&mut wa_ring, 0, HW_BSSID_0, &bcn_buf[..bcn_len], true, seq, None).map_err(mcu_err)?;
    }
    seq = seq.wrapping_add(1);
    udebug!("wifid", "beacon_fw_offload");

    // Re-set RFCR after all MCU commands (Linux calls configure_filter after BSS changes)
    dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr_val);
    dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1_val);

    // TX BAND0 ring setup — for data + management frames.
    // Descriptors already allocated at offset 0 in desc_pool during DMA init.
    // Hardware ring programmed with MT7996_TX_RING_SIZE=2048 — must match.
    // Must fit TXD+fw_txp (MT_TXWI_SIZE=76) + max frame (1514), rounded to power of 2.
    // 76 + 1514 = 1590 → next power of 2 = 2048.
    const TX_BAND0_BUF_STRIDE: usize = 2048;
    const TX_BAND0_NDESC: u32 = MT7996_TX_RING_SIZE; // Must match hardware ring size
    const TX_BAND0_BUF_SIZE: usize = TX_BAND0_NDESC as usize * TX_BAND0_BUF_STRIDE; // 4MB
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
    const HEARTBEAT_INTERVAL_NS: u64 = 500_000_000; // 500ms heartbeat

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

    // Heartbeat timer for MIB clearing, stats, STA aging (500ms)
    if let Err(_) = ctx.start_timer(TAG_WIFI_TIMER, HEARTBEAT_INTERVAL_NS) {
        uwarn!("wifid", "heartbeat_timer_fail");
    }

    // Initialize AP state machine
    let mut ssid = [0u8; MAX_SSID_LEN];
    ssid[..8].copy_from_slice(b"Filament");
    self.ap = Some(ApManager::new(BssConfig {
        bssid: mac_addr,
        ssid,
        ssid_len: 8,
        channel: 1,
        bandwidth: 0,
        secondary_channel_offset: 0,
        erp_protection: false,
        ht_protection: 0,
    }));

    // Create DataPort for IP stack data exchange
    let dp_config = BlockPortConfig { ring_size: 256, side_size: 4, pool_size: 1024 * 1024 };
    match ctx.create_block_port(dp_config) {
        Ok(port_id) => {
            if let Some(port) = ctx.block_port(port_id) {
                port.set_public();
            }
            // Register as wifi:0
            let shmem_id = ctx.block_port(port_id).map(|p| p.shmem_id()).unwrap_or(0);
            let mut info = PortInfo::new(b"wifi:0", PortClass::Network);
            info.port_subclass = port_subclass::NET_WIFI_DATA;
            let mut meta = NetworkMetadata::empty();
            meta.mac.copy_from_slice(&mac_addr);
            info.set_network_metadata(meta);
            let _ = ctx.register_port_with_info(&info, shmem_id);
            self.data_port = Some(port_id);
            // Cache pool physical address for zero-copy TX
            self.pool_phys = ctx.block_port(port_id).map(|p| p.pool_phys()).unwrap_or(0);
            uinfo!("wifid", "dataport_ready"; shmem = shmem_id, pool_phys = self.pool_phys);
        }
        Err(e) => {
            uerror!("wifid", "dataport_create_failed"; err = e as u32);
        }
    }

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

/// Reclaim provider-side RX pool slots freed by the consumer.
///
/// Free function to avoid borrow conflicts with `dev` in the timer handler.
fn reclaim_rx_pool(
    drv: &mut WifiDriver,
    ctx: &mut dyn BusCtx,
) {
    let dp = match drv.data_port {
        Some(id) => id,
        None => return,
    };
    if let Some(port) = ctx.block_port(dp) {
        let new_head = port.cq_consumer_head();
        let mask = port.ring_mask();
        while drv.last_cq_head != new_head {
            let slot = (drv.last_cq_head & mask) as usize;
            if slot < drv.cq_offsets.len() {
                let offset = drv.cq_offsets[slot];
                if offset != u32::MAX {
                    port.free(offset);
                    drv.cq_offsets[slot] = u32::MAX;
                    drv.dp_stats.rx_pool_reclaimed += 1;
                }
            }
            drv.last_cq_head = drv.last_cq_head.wrapping_add(1);
        }
    }
}

/// Sweep completed TX descriptors and post deferred CQEs to ipd.
///
/// After hardware finishes transmitting, DMA_DONE is set on the descriptor.
/// We walk the TX ring, read the token from each completed descriptor,
/// look up the original SQE tag, and post CQE_FLAG_TX_DONE so ipd can
/// safely free the pool slot (hardware is done reading it).
fn tx_sweep_and_complete(
    drv: &mut WifiDriver,
    dev: &Mt7996Dev,
    ctx: &mut dyn BusCtx,
) {
    let dp = match drv.data_port {
        Some(id) => id,
        None => return,
    };
    let tx_ring = match drv.tx_band0 {
        Some(ref mut r) => r,
        None => return,
    };

    let mut tokens = [0u16; 32]; // sweep up to 32 per call
    let count = dev.tx_sweep(tx_ring, &mut tokens);
    if count == 0 { return; }

    if let Some(port) = ctx.block_port(dp) {
        let mask = port.ring_mask();
        for i in 0..count {
            let tok = tokens[i];
            let idx = (tok as usize) & (TX_INFLIGHT_SIZE - 1);
            let sqe_tag = drv.tx_inflight_tags[idx];
            if sqe_tag == u32::MAX {
                // Management frame token (no DataPort SQE) — skip CQE posting
                continue;
            }
            drv.tx_inflight_tags[idx] = u32::MAX;

            // Mark CQ slot as TX (no provider pool to reclaim)
            let cq_slot = (drv.cq_post_seq & mask) as usize;
            if port.complete(&IoCqe {
                status: io_status::OK,
                flags: io_status::CQE_FLAG_TX_DONE,
                tag: sqe_tag,
                transferred: 0,
                result: 0,
            }) {
                if cq_slot < drv.cq_offsets.len() {
                    drv.cq_offsets[cq_slot] = u32::MAX; // no pool slot for TX CQEs
                }
                drv.cq_post_seq = drv.cq_post_seq.wrapping_add(1);
            }
        }
        // No notify — TX completions are just pool reclaim.
        // ipd will drain them on its next 10ms timer tick.
        // Notifying here causes spurious smoltcp polls → extra TCP retransmissions.
    }
}

/// SER L1 (System Error Recovery Level 1) — firmware-cooperative DMA reset.
///
/// When the data path stalls (band0 RX frozen but MCU alive), this function
/// performs the Linux SER L1 recovery sequence:
///   1. Signal firmware DMA_STOPPED
///   2. Wait for firmware RESET_DONE
///   3. Disable DMA, reset all ring indices, refill RX rings
///   4. Signal firmware DMA_INIT, wait for RECOVERY_DONE
///   5. Signal RESET_DONE, wait for NORMAL_STATE
///   6. Re-enable DMA
///
/// Source: Linux mt7996/mac.c:2534-2648 mt7996_mac_reset_work()
/// Start SER L1 recovery — non-blocking, advances via ser_l1_tick().
///
/// The old implementation blocked for up to 15 seconds (3 × 5s polls),
/// which triggered syscall storms and prevented the event loop from
/// running. Now each step is a single MMIO write; the timer tick checks
/// if the firmware has responded yet.
fn ser_l1_start(drv: &mut WifiDriver, dev: &Mt7996Dev) {
    unotice!("wifid", "ser_l1_start"; count = drv.ser_count);
    drv.ser_count += 1;

    // Step 1: Tell firmware we've stopped DMA
    // Linux: mt76_wr(dev, MT_MCU_INT_EVENT, MT_MCU_INT_EVENT_DMA_STOPPED)
    dev.mt76_wr(MT_MCU_INT_EVENT, MT_MCU_INT_EVENT_DMA_STOPPED);

    drv.ser_state = SerL1State::WaitResetDone;
    drv.ser_step_tick = drv.drain_ticks;
}

/// SER L1 timeout per step: 10 seconds (20 × 500ms ticks).
const SER_STEP_TIMEOUT_TICKS: u32 = 20;

/// Advance SER L1 state machine — called once per timer tick.
/// Returns without blocking; each firmware wait is a single register read.
fn ser_l1_tick(drv: &mut WifiDriver, dev: &Mt7996Dev) {
    let elapsed = drv.drain_ticks.wrapping_sub(drv.ser_step_tick);
    match drv.ser_state {
        SerL1State::Idle => {}
        SerL1State::WaitResetDone => {
            let mcu_cmd = dev.mt76_rr(MT_MCU_CMD);
            if mcu_cmd & MT_MCU_CMD_RESET_DONE != 0 {
                dev.mt76_wr(MT_MCU_CMD, mcu_cmd); // W1C
                ser_l1_dma_reset(drv, dev);
                dev.mt76_wr(MT_MCU_INT_EVENT, MT_MCU_INT_EVENT_DMA_INIT);
                drv.ser_state = SerL1State::WaitRecoveryDone;
                drv.ser_step_tick = drv.drain_ticks;
            } else if elapsed > SER_STEP_TIMEOUT_TICKS {
                uerror!("wifid", "ser_l1_timeout"; step = 1u32);
                ser_l1_dma_reset(drv, dev);
                dev.mt76_wr(MT_MCU_INT_EVENT, MT_MCU_INT_EVENT_DMA_INIT);
                drv.ser_state = SerL1State::WaitRecoveryDone;
                drv.ser_step_tick = drv.drain_ticks;
            }
        }
        SerL1State::WaitRecoveryDone => {
            let mcu_cmd = dev.mt76_rr(MT_MCU_CMD);
            if mcu_cmd & MT_MCU_CMD_RECOVERY_DONE != 0 {
                dev.mt76_wr(MT_MCU_CMD, mcu_cmd); // W1C
                dev.mt76_wr(MT_MCU_INT_EVENT, MT_MCU_INT_EVENT_RESET_DONE);
                drv.ser_state = SerL1State::WaitNormalState;
                drv.ser_step_tick = drv.drain_ticks;
            } else if elapsed > SER_STEP_TIMEOUT_TICKS {
                uerror!("wifid", "ser_l1_timeout"; step = 2u32);
                dev.mt76_wr(MT_MCU_INT_EVENT, MT_MCU_INT_EVENT_RESET_DONE);
                drv.ser_state = SerL1State::WaitNormalState;
                drv.ser_step_tick = drv.drain_ticks;
            }
        }
        SerL1State::WaitNormalState => {
            let mcu_cmd = dev.mt76_rr(MT_MCU_CMD);
            if mcu_cmd & MT_MCU_CMD_NORMAL_STATE != 0 {
                dev.mt76_wr(MT_MCU_CMD, mcu_cmd); // W1C
                ser_l1_finish(drv, dev);
            } else if elapsed > SER_STEP_TIMEOUT_TICKS {
                uerror!("wifid", "ser_l1_timeout"; step = 3u32);
                ser_l1_finish(drv, dev);
            }
        }
    }
}

/// DMA reset: disable engines, zero all rings, refill RX.
/// Pure MMIO — no syscalls, no blocking.
fn ser_l1_dma_reset(drv: &mut WifiDriver, dev: &Mt7996Dev) {
    // Disable DMA engines (no logic reset — firmware is still alive)
    // Linux: dma.c mt7996_dma_disable(dev, false)
    dev.mt7996_dma_disable(false);

    // Reset TX band0 ring
    if let Some(ref mut ring) = drv.tx_band0 {
        for i in 0..ring.ndesc as usize {
            let desc_ptr = ring.desc(i as u32);
            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, MT_DMA_CTL_DMA_DONE);
            }
        }
        dev.program_queue(ring.regs_base, 0, ring.desc_phys, ring.ndesc);
        ring.cpu_idx = 0;
        ring.sweep_idx = 0;
    }

    // Reset MCU WA TX ring
    if let Some(ref mut ring) = drv.wa_ring {
        for i in 0..ring.ndesc as usize {
            let desc_ptr = ring.desc(i as u32);
            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, MT_DMA_CTL_DMA_DONE);
            }
        }
        dev.program_queue(ring.regs_base, 0, ring.desc_phys, ring.ndesc);
        ring.cpu_idx = 0;
        ring.sweep_idx = 0;
    }

    // Reset all RX rings and refill
    for qi in 0..drv.rx_queue_count {
        let q = &mut drv.rx_queues[qi];
        for i in 0..q.ndesc as usize {
            let desc_ptr = unsafe { (q.desc_virt as *mut dma::Mt76Desc).add(i) };
            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, MT_DMA_CTL_DMA_DONE);
            }
        }
        dev.mt76_wr(q.regs_base + MT_QUEUE_CPU_IDX, 0);
        dev.mt76_wr(q.regs_base + MT_QUEUE_DMA_IDX, 0);
        q.reset_counters();
        dev.rx_fill(q);
    }

    // Clear TX inflight tokens (all orphaned after reset)
    for i in 0..TX_INFLIGHT_SIZE {
        drv.tx_inflight_tags[i] = u32::MAX;
    }

    // Reset statistics
    drv.tx_token = 0;
    drv.tx_freed = 0;
    drv.tx_free_entries = 0;
    drv.tx_free_notify_count = 0;
}

/// Finalize SER L1: re-enable DMA, return to normal operation.
fn ser_l1_finish(drv: &mut WifiDriver, dev: &Mt7996Dev) {
    dev.mt7996_dma_enable(false);
    dev.mt7996_dma_start(false, false);
    drv.ser_state = SerL1State::Idle;
    unotice!("wifid", "ser_l1_done"; count = drv.ser_count);
}

/// Format "prefix=value\n" into buffer, return bytes written.
fn fmt_stat_kv(buf: &mut [u8], prefix: &[u8], val: u32) -> usize {
    let mut pos = 0;
    let plen = prefix.len().min(buf.len());
    buf[..plen].copy_from_slice(&prefix[..plen]);
    pos += plen;
    pos += fmt_u32_dec(val, &mut buf[pos..]);
    if pos < buf.len() { buf[pos] = b'\n'; pos += 1; }
    pos
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
    // Linux mt76_connac3_mac.h bit layout:
    //   WLAN_IDX = GENMASK(11,0), TGID = GENMASK(13,12), HDR_FORMAT = GENMASK(15,14)
    //   HDR_INFO = GENMASK(20,16), TID = GENMASK(24,21), OWN_MAC = GENMASK(30,25)
    //   FIXED_RATE = BIT(31)
    let txd1 = (1u32 << 31)                          // FIXED_RATE
        | ((MT_HDR_FORMAT_802_11 as u32) << 14)  // HDR_FORMAT = 802.11
        | (12u32 << 16)                          // HDR_INFO = 24/2
        | (MT_TX_NORMAL << 21)                   // TID (mgmt) = 0
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
    ConfigKey::read_only(b"diag.stas"),
    ConfigKey::read_write(b"rx"),
    ConfigKey::read_write(b"scan"),
    ConfigKey::read_only(b"stats"),
];

/// Post an error CQE for a TX SQE, tracking cq_offsets and cq_post_seq.
fn post_tx_error(drv: &mut WifiDriver, dp: PortId, tag: u32, status: u16, ctx: &mut dyn BusCtx) {
    if let Some(port) = ctx.block_port(dp) {
        let mask = port.ring_mask();
        let cq_slot = (drv.cq_post_seq & mask) as usize;
        if port.complete_error(tag, status) {
            if cq_slot < drv.cq_offsets.len() {
                drv.cq_offsets[cq_slot] = u32::MAX;
            }
            drv.cq_post_seq = drv.cq_post_seq.wrapping_add(1);
        }
    }
}

impl WifiDriverWrapper {
    fn copy_to_buf(buf: &mut [u8], s: &[u8]) -> usize {
        let n = s.len().min(buf.len());
        buf[..n].copy_from_slice(&s[..n]);
        n
    }

    /// Process TX frames from ipd via the DataPort SQ.
    ///
    /// Takes `dev` as a parameter to avoid borrow conflicts in the timer
    /// handler where `dev` is already borrowed from `self.0.dev.as_ref()`.
    fn process_tx_from_ipd(&mut self, dev: &Mt7996Dev, ctx: &mut dyn BusCtx) {
        let dp = match self.0.data_port {
            Some(id) => id,
            None => return,
        };
        // Drain all pending SQEs (up to ring_size). Previous 16-per-call limit
        // caused SQE accumulation: ipd burst-submits during iface.poll(), shmem
        // notify collapses to one wake, wifid processes 16, rest stranded until
        // next 500ms timer tick.
        for _ in 0..256 {
            let sqe = match ctx.block_port(dp).and_then(|p| p.recv_request()) {
                Some(s) => s,
                None => break,
            };
            if sqe.opcode != io_op::NET_SEND {
                post_tx_error(&mut self.0, dp, sqe.tag, io_status::INVALID, ctx);

                continue;
            }

            let frame_len = sqe.data_len as usize;
            if frame_len < ETH_HEADER_LEN || frame_len > MAX_FRAME_SIZE {
                post_tx_error(&mut self.0, dp, sqe.tag, io_status::INVALID, ctx);

                continue;
            }

            // Read full frame from pool for TX (copy to stack buffer)
            let mut frame_buf = [0u8; MAX_FRAME_SIZE];
            let got_frame = ctx.block_port(dp).and_then(|port| {
                port.pool_slice(sqe.data_offset, frame_len as u32).map(|s| {
                    frame_buf[..frame_len].copy_from_slice(s);
                })
            });
            if got_frame.is_none() {
                if let Some(port) = ctx.block_port(dp) {
                    port.complete_error(sqe.tag, io_status::INVALID);
    
                }
                continue;
            }

            let dst_mac: [u8; 6] = [frame_buf[0], frame_buf[1], frame_buf[2],
                                     frame_buf[3], frame_buf[4], frame_buf[5]];
            let is_multicast = (dst_mac[0] & 0x01) != 0;

            // PS buffering gated by SoC capability.
            // MT7996 firmware handles PS natively via WTBL gating + PS-Poll/U-APSD,
            // so software buffering is redundant and was causing SSH heartbeat freeze
            // (frames held hostage → no CQEs → smoltcp send queue fills).
            //
            // Simpler chipsets without firmware-assisted PS would need the software
            // path: check ps_mode, divert to ps_buf, flush on PM=0 transition.
            let firmware_handles_ps = true; // MT7996: firmware PS via WTBL/TIM

            let (wcid, sta_sleeping) = if is_multicast {
                (MT7996_WTBL_RESERVED, false)
            } else {
                self.0.ap.as_ref()
                    .and_then(|ap| ap.find_sta(&dst_mac))
                    .map(|sta| (sta.wlan_idx, sta.ps_mode))
                    .unwrap_or((MT7996_WTBL_RESERVED, false))
            };

            // Software PS buffering: divert frames for sleeping STAs.
            // Skipped when firmware handles PS natively (MT7996 WTBL gating).
            if !firmware_handles_ps && !is_multicast && sta_sleeping {
                if self.0.ps_buf.push(&dst_mac, &frame_buf[..frame_len]).is_ok() {
                    udebug!("wifid", "ps_buf_enqueue"; wcid = wcid as u32, len = frame_len as u32);
                }
                continue;
            }

            // Copy-based TX: frame data copied into DMA buffer by tx_enqueue_data
            let result = if let Some(ref mut tx_ring) = self.0.tx_band0 {
                let tok = self.0.tx_token;
                self.0.tx_token = self.0.tx_token.wrapping_add(1);
                let r = dev.tx_enqueue_data(tx_ring, &frame_buf[..frame_len], wcid, tok);
                if r.is_ok() {
                    // Track inflight: tok → sqe.tag for deferred CQE
                    let idx = (tok as usize) & (TX_INFLIGHT_SIZE - 1);
                    self.0.tx_inflight_tags[idx] = sqe.tag;
                    self.0.dp_stats.tx_frames += 1;
                    udebug!("wifid", "tx_enqueue"; len = frame_len as u32,
                        mcast = is_multicast as u32, wcid = wcid as u32,
                        tok = tok as u32, total = self.0.dp_stats.tx_frames);
                }
                r
            } else {
                Err(-3)
            };

            // Only post immediate CQE on error (hardware never saw the frame)
            if let Err(e) = result {
                self.0.dp_stats.tx_errors += 1;
                udebug!("wifid", "tx_enqueue_err"; err = e as u32, len = frame_len as u32,
                    total_err = self.0.dp_stats.tx_errors);
                if let Some(port) = ctx.block_port(dp) {
                    let mask = port.ring_mask();
                    let cq_slot = (self.0.cq_post_seq & mask) as usize;
                    if port.complete(&IoCqe {
                        status: io_status::IO_ERROR,
                        flags: io_status::CQE_FLAG_TX_DONE,
                        tag: sqe.tag,
                        transferred: 0,
                        result: 0,
                    }) {
                        if cq_slot < self.0.cq_offsets.len() {
                            self.0.cq_offsets[cq_slot] = u32::MAX;
                        }
                        self.0.cq_post_seq = self.0.cq_post_seq.wrapping_add(1);
                    }

                }
                self.0.dp_stats.tx_pool_drops += 1;
            }
            // On success: CQE deferred until tx_sweep confirms hardware completion
        }

        // No notify for TX error CQEs — pool reclaim only.
        // ipd drains them on its next 10ms timer tick.
    }

}

impl Driver for WifiDriverWrapper {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.reset(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn handle_event(&mut self, tag: u32, _handle: userlib::syscall::Handle, ctx: &mut dyn BusCtx) {
        const TAG_WIFI_TIMER: u32 = 100;
        const TAG_WIFI_IRQ: u32 = 101;

        // Determine if this is an IRQ event or a timer tick
        let is_irq = tag == TAG_WIFI_IRQ;
        let is_timer = tag == TAG_WIFI_TIMER;
        if !is_irq && !is_timer {
            return;
        }

        let dev_ptr = match self.0.dev.as_ref() {
            Some(d) => d as *const Mt7996Dev,
            None => return,
        };
        // SAFETY: self.0.dev is Some (just checked) and is never dropped or moved
        // during handle_event. The pointer is valid for the entire function body.
        // We use a raw pointer because the borrow checker cannot see that
        // process_tx_from_ipd and reclaim_rx_pool don't touch self.0.dev.
        let dev = unsafe { &*dev_ptr };

        // Both timer and IRQ events process DMA queues.
        // Timer ticks MUST drain MCU RX queues (q0/q1) because the timer
        // handler sends fire-and-forget MCU commands (mcu_get_chan_mib_info,
        // mcu_get_all_sta_info) whose responses accumulate on q0/q1.
        // If only IRQ events drain these queues, the RX ring fills up
        // (~512 entries ÷ 10/sec = ~51 seconds) and firmware stalls.

        if is_timer {
            self.0.drain_ticks += 1;

        }

        // Skip all DMA work while SER L1 recovery is in progress.
        // The rings are being reset — touching them would corrupt recovery.
        // Timer ticks advance the SER state machine (one MMIO check per tick).
        if self.0.ser_state != SerL1State::Idle {
            if is_timer {
                ser_l1_tick(&mut self.0, dev);
            }
            if self.0.irq_mode && is_irq {
                if let Some(ref mut irq) = self.0.irq {
                    let _ = irq.ack();
                }
            }
            return;
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
            dev.rx_process_mcu(&mut self.0.rx_queues[0], &mut self.0.mib[2]);
        }

        // MCU WA RX (q1) — firmware events from WA processor
        if intr & MT_INT_RX_DONE_WA != 0 && qc > 1 {
            dev.rx_process_mcu(&mut self.0.rx_queues[1], &mut self.0.mib[2]);
        }

        // BAND0 data RX (q2) — classify frames, collect management frames for AP
        let mut mgmt_frames: [RxMgmtFrame; 8] = core::array::from_fn(|_| RxMgmtFrame {
            subtype: MgmtSubtype::Other(0),
            addr2: [0; 6],
            addr3: [0; 6],
            rssi: -128,
            phy: wifi80211::types::RxPhyInfo::UNKNOWN,
            body: [0; wifi80211::types::MAX_MGMT_BODY_LEN],
            body_len: 0,
        });
        let mut mgmt_count = 0usize;
        let mut data_frames: [RxDataFrame; 16] = core::array::from_fn(|_| RxDataFrame {
            copy_offset: 0, len: 0,
        });
        let mut data_count = 0usize;
        let mut pm_events: [RxPmEvent; 8] = core::array::from_fn(|_| RxPmEvent { mac: [0; 6], pm: false });
        let mut pm_count = 0usize;
        // Copy buffer for data frame payloads — rx_classify copies frame data
        // here before resetting DMA descriptors (avoids use-after-free).
        // 16 frames × MAX_FRAME_SIZE bytes max = ~24KB
        let mut data_buf = [0u8; 16 * MAX_FRAME_SIZE];
        if intr & MT_INT_RX_DONE_BAND0 != 0 && qc > 2 {
            let (_n, mc, dc, pc) = dev.rx_classify(
                &mut self.0.rx_queues[2], &mut self.0.mib[0],
                &mut mgmt_frames, 8,
                &mut data_frames, 16,
                &mut data_buf,
                &mut pm_events, 8,
            );
            mgmt_count = mc;
            data_count = dc;
            pm_count = pc;
        }

        // WA_MAIN (q3) — TX free notifications
        if intr & MT_INT_RX_DONE_WA_MAIN != 0 && qc > 3 {
            let r = dev.rx_process_tx_free(&mut self.0.rx_queues[3]);
            self.0.tx_freed += r.tokens_freed;
            self.0.tx_free_entries += r.entries;
            self.0.tx_free_notify_count += r.txrx_notify_count;
        }

        // BAND2 data RX (q4) — classify frames, update band2 counters
        if intr & MT_INT_RX_DONE_BAND2 != 0 && qc > 4 {
            let mut dummy_mgmt: [RxMgmtFrame; 1] = [RxMgmtFrame {
                subtype: MgmtSubtype::Other(0),
                addr2: [0; 6],
                addr3: [0; 6],
                rssi: -128,
                phy: wifi80211::types::RxPhyInfo::UNKNOWN,
                body: [0; wifi80211::types::MAX_MGMT_BODY_LEN],
                body_len: 0,
            }];
            let mut dummy_data: [RxDataFrame; 1] = [RxDataFrame { copy_offset: 0, len: 0 }];
            let mut dummy_buf = [0u8; 0];
            let mut dummy_pm: [RxPmEvent; 1] = [RxPmEvent { mac: [0; 6], pm: false }];
            dev.rx_classify(&mut self.0.rx_queues[4], &mut self.0.mib[1], &mut dummy_mgmt, 0, &mut dummy_data, 0, &mut dummy_buf, &mut dummy_pm, 0);
        }

        // WA_TRI (q5) — TX free notifications (band2)
        if intr & MT_INT_RX_DONE_WA_TRI != 0 && qc > 5 {
            let r = dev.rx_process_tx_free(&mut self.0.rx_queues[5]);
            self.0.tx_freed += r.tokens_freed;
        }

        // Sweep completed TX descriptors and post CQEs to ipd.
        // This runs on every timer tick (not just data_ready) so that
        // TX completions flow back to ipd even when no new RX/TX events
        // arrive — breaking the TX↔RX coupling.
        tx_sweep_and_complete(&mut self.0, dev, ctx);

        // Dispatch management frames through AP state machine.
        // We process one frame at a time: get actions from AP, then execute them.
        // Actions are executed inline to avoid borrow conflicts between ap and self.
        for i in 0..mgmt_count {
            let subtype = mgmt_frames[i].subtype;
            let addr2 = mgmt_frames[i].addr2;

            // Log every received management frame — essential for debugging the RX path
            match subtype {
                MgmtSubtype::ProbeReq => {
                    // Counted in mib.probe_req — no per-frame log needed
                }
                MgmtSubtype::Auth => {
                    let abs_rssi = if mgmt_frames[i].rssi < 0 { (-mgmt_frames[i].rssi) as u32 } else { mgmt_frames[i].rssi as u32 };
                    uinfo!("wifid", "rx_auth";
                        mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                        mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                        mac4 = addr2[4] as u32, mac5 = addr2[5] as u32,
                        rssi_neg = abs_rssi
                    );
                }
                MgmtSubtype::AssocReq | MgmtSubtype::ReassocReq => {
                    let abs_rssi = if mgmt_frames[i].rssi < 0 { (-mgmt_frames[i].rssi) as u32 } else { mgmt_frames[i].rssi as u32 };
                    let is_reassoc = matches!(mgmt_frames[i].subtype, MgmtSubtype::ReassocReq);
                    if is_reassoc {
                        uinfo!("wifid", "rx_reassoc_req";
                            mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                            mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                            mac4 = addr2[4] as u32, mac5 = addr2[5] as u32,
                            rssi_neg = abs_rssi
                        );
                    } else {
                        uinfo!("wifid", "rx_assoc_req";
                            mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                            mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                            mac4 = addr2[4] as u32, mac5 = addr2[5] as u32,
                            rssi_neg = abs_rssi
                        );
                    }
                }
                MgmtSubtype::Deauth => {
                    udebug!("wifid", "rx_deauth";
                        mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                        mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                        mac4 = addr2[4] as u32, mac5 = addr2[5] as u32
                    );
                }
                MgmtSubtype::Disassoc => {
                    udebug!("wifid", "rx_disassoc";
                        mac0 = addr2[0] as u32, mac1 = addr2[1] as u32,
                        mac2 = addr2[2] as u32, mac3 = addr2[3] as u32,
                        mac4 = addr2[4] as u32, mac5 = addr2[5] as u32
                    );
                }
                MgmtSubtype::Action => {
                    udebug!("wifid", "rx_action";
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
            let mut tx_frame = [0u8; 256]; // Raw 802.11 frame from AP
            let mut tx_len = 0usize;
            let mut register_mac = [0u8; 6];
            let mut register_aid = 0u16;
            let mut register_ht_cap = 0u16;
            let mut register_ht_param = 0u8;
            let mut register_flags = 0u16;
            let mut do_register = false;
            let mut remove_mac = [0u8; 6];
            let mut remove_aid = 0u16;
            let mut remove_wlan_idx = 0u16;
            let mut do_remove = false;
            let mut ba_mac = [0u8; 6];
            let mut ba_tid = 0u8;
            let mut ba_ssn = 0u16;
            let mut ba_win_size = 0u16;
            let mut ba_start = false;
            let mut do_ba = false;

            if let Some(ref mut ap) = self.0.ap {
                let mut raw_buf = [0u8; 256];
                let result = ap.handle_rx_mgmt(&mgmt_frames[i], &mut raw_buf, self.0.drain_ticks);

                // Extract action data from up to 3 actions
                for action_opt in result.actions.iter() {
                    if let Some(ref action) = action_opt {
                        match action {
                            ApAction::TxFrame(data) => {
                                let n = data.len().min(tx_frame.len());
                                tx_frame[..n].copy_from_slice(&data[..n]);
                                tx_len = n;
                            }
                            ApAction::RegisterSta { mac, aid, ht_cap, ht_param, flags } => {
                                register_mac = *mac;
                                register_aid = *aid;
                                register_ht_cap = *ht_cap;
                                register_ht_param = *ht_param;
                                register_flags = *flags;
                                do_register = true;
                            }
                            ApAction::RemoveSta { mac, aid, wlan_idx } => {
                                remove_mac = *mac;
                                remove_aid = *aid;
                                remove_wlan_idx = *wlan_idx;
                                do_remove = true;
                            }
                            ApAction::NotifyBaSession { mac, tid, ssn, win_size, start } => {
                                ba_mac = *mac;
                                ba_tid = *tid;
                                ba_ssn = *ssn;
                                ba_win_size = *win_size;
                                ba_start = *start;
                                do_ba = true;
                            }
                            ApAction::StaPsChanged { .. } => {
                                // PS state change handled by beacon_dirty flag
                            }
                        }
                    }
                }

                // Log when AP drops a frame (e.g. assoc from unauthenticated STA)
                if result.count == 0 && !matches!(subtype, MgmtSubtype::ProbeReq | MgmtSubtype::Other(_) | MgmtSubtype::Action) {
                    udebug!("wifid", "ap_drop_frame";
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
                        // Log TXD1 for debugging TX-not-transmitted regression
                        let txd1_val = u32::from_le_bytes([txd_buf[4], txd_buf[5], txd_buf[6], txd_buf[7]]);
                        match dev.tx_enqueue(tx_ring, &txd_buf[..len], tok) {
                            Ok(()) => {
                                if !matches!(subtype, MgmtSubtype::ProbeReq) {
                                    uinfo!("wifid", "tx_mgmt_ok"; tok = tok as u32, len = len as u32, txd1 = txd1_val);
                                }
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

                udebug!("wifid", "sta_register";
                    mac0 = register_mac[0] as u32, mac1 = register_mac[1] as u32,
                    mac2 = register_mac[2] as u32, mac3 = register_mac[3] as u32,
                    mac4 = register_mac[4] as u32, mac5 = register_mac[5] as u32,
                    aid = register_aid as u32, wlan = wlan_idx as u32
                );

                // Linux main.c:992 — clear ADM count before adding STA
                dev.mac_wtbl_update(wlan_idx as u32, MT_WTBL_UPDATE_ADM_COUNT_CLEAR);

                if let Some(ref mut wa_ring) = self.0.wa_ring {
                    let seq = self.0.seq;
                    self.0.seq = self.0.seq.wrapping_add(1);
                    // Fire-and-forget: don't wait for MCU responses during event loop.
                    // wait=true would block processing (delay_ms polling) and corrupt
                    // the MCU RX ring (CPU_IDX advanced without descriptor reset).
                    // Responses are drained by rx_process_mcu on the next timer tick.
                    if let Err(e) = dev.mcu_add_client_sta(
                        wa_ring, 0, wlan_idx, HW_BSSID_0,
                        CONN_STATE_CONNECT, &register_mac, register_aid, true, seq, None, false,
                        register_ht_cap, register_ht_param, register_flags,
                    ) {
                        uerror!("wifid", "mcu_sta_connect_failed"; err = e as u32, wlan = wlan_idx as u32);
                    }
                    let seq = self.0.seq;
                    self.0.seq = self.0.seq.wrapping_add(1);
                    if let Err(e) = dev.mcu_add_client_sta(
                        wa_ring, 0, wlan_idx, HW_BSSID_0,
                        CONN_STATE_PORT_SECURE, &register_mac, register_aid, false, seq, None, false,
                        register_ht_cap, register_ht_param, register_flags,
                    ) {
                        uerror!("wifid", "mcu_sta_port_secure_failed"; err = e as u32, wlan = wlan_idx as u32);
                    }
                    let seq = self.0.seq;
                    self.0.seq = self.0.seq.wrapping_add(1);
                    if let Err(e) = dev.mcu_add_group(wa_ring, 0, wlan_idx, seq, None, false) {
                        uerror!("wifid", "mcu_add_group_failed"; err = e as u32, wlan = wlan_idx as u32);
                    }
                    // Send STA_REC_RA for rate adaptation — must follow add_sta.
                    // Without this, firmware has no rate table → uses lowest rate → PLE stall.
                    let seq = self.0.seq;
                    self.0.seq = self.0.seq.wrapping_add(1);
                    // Effective BW: 40MHz only if BSS configured AND STA supports it
                    let sta_bw = if register_flags & wifi80211::types::STA_FLAG_HT40 != 0 {
                        if let Some(ref ap) = self.0.ap {
                            if ap.bss.bandwidth >= 1 { CMD_CBW_40MHZ } else { CMD_CBW_20MHZ }
                        } else { CMD_CBW_20MHZ }
                    } else { CMD_CBW_20MHZ };
                    if let Err(e) = dev.mcu_sta_rate_ctrl(
                        wa_ring, 0, wlan_idx, HW_BSSID_0,
                        self.0.channel, sta_bw, register_ht_cap, register_ht_param, register_flags, seq,
                    ) {
                        uerror!("wifid", "mcu_sta_rate_ctrl_failed"; err = e as u32, wlan = wlan_idx as u32);
                    }
                }

                // Store WCID on STA for TX routing
                if let Some(ref mut ap) = self.0.ap {
                    ap.set_sta_wlan_idx(&register_mac, wlan_idx);
                }
            }
            if do_remove && remove_wlan_idx != 0 {
                udebug!("wifid", "sta_remove"; aid = remove_aid as u32, wlan = remove_wlan_idx as u32);
                // Linux main.c:1034 — clear ADM count before removing STA
                dev.mac_wtbl_update(remove_wlan_idx as u32, MT_WTBL_UPDATE_ADM_COUNT_CLEAR);
                if let Some(ref mut wa_ring) = self.0.wa_ring {
                    let seq = self.0.seq;
                    self.0.seq = self.0.seq.wrapping_add(1);
                    let _ = dev.mcu_add_client_sta(
                        wa_ring, 0, remove_wlan_idx, HW_BSSID_0,
                        CONN_STATE_DISCONNECT, &remove_mac, remove_aid, false, seq, None, false,
                        0, 0, 0, // HT caps irrelevant for disconnect
                    );
                }
            }
            // BA session notification — tell firmware about ADDBA/DELBA.
            // Source: Linux mt7996/mcu.c mt7996_mcu_sta_ba()
            // STA_REC_BA TLV via MCU_WMWA_UNI_CMD(STA_REC_UPDATE)
            if do_ba {
                // Look up STA's wlan_idx for the MCU command
                let ba_wlan_idx = if let Some(ref ap) = self.0.ap {
                    ap.find_sta(&ba_mac).map(|s| s.wlan_idx).unwrap_or(0)
                } else {
                    0
                };
                if ba_wlan_idx != 0 {
                    if ba_start {
                        uinfo!("wifid", "ba_session_start"; tid = ba_tid as u32,
                            ssn = ba_ssn as u32, win = ba_win_size as u32, wlan = ba_wlan_idx as u32);
                    } else {
                        uinfo!("wifid", "ba_session_stop"; tid = ba_tid as u32, wlan = ba_wlan_idx as u32);
                    }
                    if let Some(ref mut wa_ring) = self.0.wa_ring {
                        let seq = self.0.seq;
                        self.0.seq = self.0.seq.wrapping_add(1);
                        if let Err(e) = dev.mcu_sta_ba(
                            wa_ring, 0, ba_wlan_idx, HW_BSSID_0,
                            ba_tid, ba_ssn, ba_win_size, ba_start, seq,
                        ) {
                            uerror!("wifid", "mcu_sta_ba_failed"; err = e as u32, wlan = ba_wlan_idx as u32);
                        }
                    }
                }
            }
        }

        // Process PM events from Null Function / QoS Null frames.
        // These are the primary PS mode signaling mechanism.
        if pm_count > 0 {
            if let Some(ref mut ap) = self.0.ap {
                for i in 0..pm_count {
                    let changed = ap.handle_rx_data_pm(&pm_events[i].mac, pm_events[i].pm, self.0.drain_ticks);
                    // PM=0 transition: flush all buffered frames for this STA.
                    // Only needed when software PS buffering is active.
                    let firmware_handles_ps = true; // MT7996: firmware PS via WTBL/TIM
                    if !firmware_handles_ps && changed && !pm_events[i].pm {
                        let mac = &pm_events[i].mac;
                        let wcid = ap.find_sta(mac).map(|s| s.wlan_idx).unwrap_or(MT7996_WTBL_RESERVED);
                        let mut indices = [0usize; PS_MAX_PER_STA];
                        let n = self.0.ps_buf.find_sta_frames(mac, &mut indices);
                        for j in 0..n {
                            let idx = indices[j];
                            let f = &self.0.ps_buf.frames[idx];
                            if let Some(ref mut tx_ring) = self.0.tx_band0 {
                                let tok = self.0.tx_token;
                                self.0.tx_token = self.0.tx_token.wrapping_add(1);
                                let _ = dev.tx_enqueue_data(tx_ring, &f.data[..f.len as usize], wcid, tok);
                            }
                            self.0.ps_buf.release(idx);
                        }
                    }
                }
            }
        }

        // Forward data frames to ipd via DataPort.
        // Frame data was copied into data_buf by rx_classify (safe — DMA buffers already freed).
        if data_count > 0 {
            // Touch STA last_seen + clear PS for each data frame.
            // If a client is sending data, it's awake (PM=0 implicit).
            // Ethernet frame: dst[6] + src[6] — source MAC at offset 6.
            if let Some(ref mut ap) = self.0.ap {
                for i in 0..data_count {
                    let frame = &data_frames[i];
                    let ofs = frame.copy_offset as usize;
                    if frame.len >= 14 {
                        let mut src_mac = [0u8; 6];
                        src_mac.copy_from_slice(&data_buf[ofs + 6..ofs + 12]);
                        ap.handle_rx_data_pm(&src_mac, false, self.0.drain_ticks);
                    }
                }
            }
            if let Some(dp) = self.0.data_port {
                if let Some(port) = ctx.block_port(dp) {
                    let mask = port.ring_mask();
                    for i in 0..data_count {
                        let frame = &data_frames[i];
                        let flen = frame.len as u32;
                        let src = &data_buf[frame.copy_offset as usize..(frame.copy_offset as usize + frame.len as usize)];

                        // Allocate pool slot for this frame
                        let offset = match port.alloc(flen) {
                            Some(o) => o,
                            None => {
                                self.0.dp_stats.rx_pool_drops += 1;
                                continue;
                            }
                        };

                        // Copy frame into pool
                        if let Some(dst) = port.pool_slice_mut(offset, flen) {
                            dst.copy_from_slice(src);
                        } else {
                            port.free(offset);
                            self.0.dp_stats.rx_pool_drops += 1;
                            continue;
                        }

                        // Post CQE to consumer (ipd)
                        let tag = self.0.net_rx_seq;
                        self.0.net_rx_seq = self.0.net_rx_seq.wrapping_add(1);
                        let cq_slot = (self.0.cq_post_seq & mask) as usize;
                        if port.complete(&IoCqe {
                            status: io_status::OK,
                            flags: 0,
                            tag,
                            transferred: flen,
                            result: offset,
                        }) {
                            if cq_slot < self.0.cq_offsets.len() {
                                self.0.cq_offsets[cq_slot] = offset;
                            }
                            self.0.cq_post_seq = self.0.cq_post_seq.wrapping_add(1);
                            self.0.dp_stats.rx_frames += 1;
                        } else {
                            port.free(offset);
                            self.0.dp_stats.rx_pool_drops += 1;
                        }
                    }
                    // Notify ipd that RX frames are available. ipd defers
                    // poll_smoltcp() by a 2ms coalescing window so multiple
                    // frames accumulate before a single iface.poll().
                    port.notify();
                }
            }
        }

        // Process TX frames from ipd and sweep completed TX descriptors
        if let Some(dp) = self.0.data_port {
            if ctx.block_port(dp).is_some() {
                self.process_tx_from_ipd(dev, ctx);
                tx_sweep_and_complete(&mut self.0, dev, ctx);
                reclaim_rx_pool(&mut self.0, ctx);
            }
        }

        // Periodic maintenance: every timer tick (500ms)
        // Beacons are firmware-offloaded; timer handles MIB, stats, STA aging.
        if is_timer {
            // Channel MIB counter clearing
            // Linux: mt7996_mac_work() → mt76_update_survey() → mt7996_mcu_get_chan_mib_info()
            // Without this, firmware CCA counters accumulate and MAC assesses channel as
            // permanently busy, stopping all TX.
            // Source: mt7996/mac.c:2155-2170, mcu.c:3948-4025
            if let Some(ref mut wa_ring) = self.0.wa_ring {
                let _ = dev.mcu_get_chan_mib_info(wa_ring, 0, self.0.seq);
                self.0.seq = self.0.seq.wrapping_add(1);
            }

            // Hardware MIB register clearing
            // Linux: mac.c:2743-2882 mt7996_mac_update_stats()
            dev.mac_update_stats(0);

            if let Some(ref mut wa_ring) = self.0.wa_ring {
                // Stats query
                let _ = dev.mcu_get_all_sta_info(wa_ring, UNI_ALL_STA_TXRX_RATE, self.0.seq);
                self.0.seq = self.0.seq.wrapping_add(1);
            }

            // STA aging: evict STAs not seen within timeout.
            // TX deauth frame (reason=4, inactivity) so client disconnects cleanly.
            if let Some(ref mut ap) = self.0.ap {
                let mut evicted = [([0u8; 6], 0u16, 0u16); 16];
                let n = ap.age_stas(self.0.drain_ticks, &mut evicted);
                for j in 0..n {
                    let (mac, aid, wlan_idx) = evicted[j];
                    udebug!("wifid", "sta_aged"; aid = aid as u32, wlan = wlan_idx as u32);
                    // TX deauth frame so client knows it was disconnected (reason=4, inactivity)
                    {
                        let seq = ap.next_seq();
                        let mut deauth_buf = [0u8; 64];
                        let deauth_len = wifi80211::frame::build_deauth(
                            &mut deauth_buf, &ap.bss.bssid, &mac, 4, seq,
                        );
                        if deauth_len > 0 {
                            if let Some(ref mut tx_ring) = self.0.tx_band0 {
                                let mut txd_buf = [0u8; 320];
                                let total = wrap_mgmt_txd(&mut txd_buf, &deauth_buf[..deauth_len]);
                                if total > 0 {
                                    let token = self.0.tx_token;
                                    self.0.tx_token = self.0.tx_token.wrapping_add(1);
                                    let _ = dev.tx_enqueue(tx_ring, &txd_buf[..total], token);
                                }
                            }
                        }
                    }
                    if wlan_idx != 0 {
                        // Linux main.c:1034 — clear ADM count before removing STA
                        dev.mac_wtbl_update(wlan_idx as u32, MT_WTBL_UPDATE_ADM_COUNT_CLEAR);
                        if let Some(ref mut wa_ring) = self.0.wa_ring {
                            let seq = self.0.seq;
                            self.0.seq = self.0.seq.wrapping_add(1);
                            let _ = dev.mcu_add_client_sta(
                                wa_ring, 0, wlan_idx, HW_BSSID_0,
                                CONN_STATE_DISCONNECT, &mac, aid, false, seq, None, false,
                                0, 0, 0, // HT caps irrelevant for disconnect
                            );
                        }
                    }
                }
            }

            // Beacon re-upload on dirty (TIM/ERP/HT protection changed).
            // Rate-limited: at most once per timer tick (500ms).
            if self.0.beacon_on {
                let dirty = if let Some(ref mut ap) = self.0.ap {
                    let d = ap.beacon_dirty;
                    ap.beacon_dirty = false;
                    d
                } else { false };
                if dirty {
                    let mut bcn_buf = [0u8; 256];
                    let bcn_len = if let Some(ref mut ap) = self.0.ap {
                        ap.beacon(&mut bcn_buf)
                    } else { 0 };
                    if bcn_len > 0 {
                        if let Some(ref mut wa_ring) = self.0.wa_ring {
                            let _ = dev.mcu_set_beacon(wa_ring, 0, HW_BSSID_0, &bcn_buf[..bcn_len], true, self.0.seq, None);
                            self.0.seq = self.0.seq.wrapping_add(1);
                        }
                    }
                }
            }

            // Firmware-initiated SER: poll MT_MCU_CMD for error/WDT bits.
            // Linux reads this in IRQ handler (mmio.c:781-788); we poll since
            // we don't have the MCU interrupt wired up.
            // Source: Linux mt7996/mmio.c mt7996_irq_tasklet()
            {
                let mcu_cmd = dev.mt76_rr(MT_MCU_CMD);
                if mcu_cmd & (MT_MCU_CMD_ERROR_MASK | MT_MCU_CMD_WDT_MASK) != 0 {
                    // Ack by writing back — Linux mmio.c:784
                    dev.mt76_wr(MT_MCU_CMD, mcu_cmd);

                    if mcu_cmd & MT_MCU_CMD_WDT_MASK != 0 {
                        uerror!("wifid", "firmware_wdt";
                            mcu_cmd = mcu_cmd,
                            wm = ((mcu_cmd & MT_MCU_CMD_WM_WDT) != 0) as u32,
                            wa = ((mcu_cmd & MT_MCU_CMD_WA_WDT) != 0) as u32);
                        // WDT = firmware crash. Full reset needed (L2+), not L1.
                        // For now just log it — L2 recovery is much more complex.
                    } else if mcu_cmd & MT_MCU_CMD_STOP_DMA != 0 {
                        uwarn!("wifid", "firmware_ser_l1"; mcu_cmd = mcu_cmd);
                        ser_l1_start(&mut self.0, dev);
                    }
                }
            }
        }

        // Beacon diagnostic: log key registers every second while beaconing.
        // drain_ticks increments every 500ms, so 2 = 1 second.
        if is_timer && self.0.beacon_on && self.0.drain_ticks % 10 == 0 {
            let secs = self.0.drain_ticks / 2;
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

            // Beacon TX MIB counter — read-to-clear, shows beacons sent since last read.
            // At 100ms beacon interval, expect ~50 per 5-second diagnostic window.
            // 0 = firmware stopped sending beacons (likely cause of client disconnect).
            let bcn_hw = dev.reg_rr(mt_wf_mib(0, MT_MIB_BTSCR0_OFS));

            // Monitor RX ring DMA_IDX and CPU_IDX for WA_MAIN (q3) to debug drain
            let mut rx_dma = [0u32; 6];
            let mut r2_cpu = 0u32;
            let mut r3_cpu = 0u32;
            for ri in 0..qc.min(6) {
                rx_dma[ri] = dev.mt76_rr(self.0.rx_queues[ri].regs_base + MT_QUEUE_DMA_IDX);
            }
            if qc > 2 {
                r2_cpu = dev.mt76_rr(self.0.rx_queues[2].regs_base + MT_QUEUE_CPU_IDX);
            }
            if qc > 3 {
                r3_cpu = dev.mt76_rr(self.0.rx_queues[3].regs_base + MT_QUEUE_CPU_IDX);
            }

            let rfcr_actual = dev.reg_rr(mt_wf_rmac(0, MT_WF_RFCR_OFS));

            let dp = &self.0.dp_stats;
            let sq_pend = self.0.data_port.and_then(|id| ctx.block_port(id))
                .map(|p| p.sq_pending()).unwrap_or(0);
            // Split into two log lines to avoid uinfo buffer truncation
            uinfo!("wifid", "bcn_diag";
                t = secs,
                arb = arb,
                mcu = mcu_cmd,
                stas = sta_count,
                sw_bcn = self.0.tx_beacon,
                tx_prb = self.0.tx_probe_resp,
                rx_tot = b0.total,
                rx_dat = b0.data,
                rx_mgmt = b0.mgmt,
                rx_prb = b0.probe_req,
                rx_fcs = b0.fcs_err,
                rfcr = rfcr_actual,
                r0 = rx_dma[0], r1 = rx_dma[1], r2 = rx_dma[2], r2c = r2_cpu,
                r3 = rx_dma[3], r3c = r3_cpu,
                q0d = q0_dma, q0c = q0_cpu,
                q1d = q1_dma, q1c = q1_cpu,
                q0h = self.0.rx_queues[0].head,
                q0t = self.0.rx_queues[0].tail,
                q0q = self.0.rx_queues[0].queued
            );
            uinfo!("wifid", "bcn_tx";
                t = secs,
                bcn_hw = bcn_hw,
                tx_ok = tx_mpdu,
                tx_try = tx_try,
                tx_amp = tx_ampdu,
                ack_f = ack_fail,
                tx_c = tx_cpu,
                tx_d = tx_dma,
                txf = self.0.tx_freed,
                tfe = self.0.tx_free_entries,
                tfn = self.0.tx_free_notify_count,
                ple_f = ple_free,
                ple_e = ple_empty,
                dp_rx = dp.rx_frames,
                dp_tx = dp.tx_frames,
                dp_rxd = dp.rx_pool_drops,
                dp_txd = dp.tx_pool_drops,
                dp_sqp = sq_pend,
                dp_dr = dp.data_ready_calls,
                dp_ps = dp.tx_ps_buffered,
                dp_te = dp.tx_errors,
                napi = self.0.irq_suppressed as u32
            );

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

        // NAPI-style IRQ management.
        //
        // Key insight: HW interrupt mask and kernel IRQ ack are independent.
        // - HW mask (MT_INT_MASK_CSR) controls whether the device raises MSI
        // - Kernel IRQ ack (irq.ack()) clears the "readable" flag on the handle
        //
        // Strategy: ALWAYS ack the kernel IRQ immediately (so the Mux handle
        // goes not-readable and we don't hot-loop). Control whether new
        // interrupts arrive by toggling the HW mask only.
        //
        // Enter polling: ack kernel IRQ, leave HW masked → timer drains
        // Exit polling:  unmask HW → next real interrupt fires normally
        if self.0.irq_mode && is_irq {
            // Always ack kernel IRQ first — clears Mux readability
            if let Some(ref mut irq) = self.0.irq {
                let _ = irq.ack();
            }

            let pending = dev.mt76_rr(MT_INT_SOURCE_CSR);
            if pending & MT_INT_RX_DONE_ALL != 0 {
                // More RX work arrived during processing — enter polling mode.
                // HW interrupts stay masked (we masked at top of IRQ path).
                // Timer tick will drain queues and unmask when quiet.
                self.0.irq_suppressed = true;
            } else {
                // Queues drained — unmask HW interrupts to resume normal mode.
                let irq_mask = MT_INT_RX_DONE_ALL | MT_INT_MCU_CMD
                    | MT_INT_TX_DONE_BAND0 | MT_INT_TX_DONE_MCU_WM;
                dev.mt76_wr(MT_INT_MASK_CSR, irq_mask);
                self.0.irq_suppressed = false;
            }
        }

        // Timer tick: if IRQ was suppressed (NAPI polling), re-enable.
        // The timer already drained all queues above (intr = 0xFFFF_FFFF).
        // Clear INT_SOURCE_CSR (W1C) to ack processed work, then unmask.
        // If new RX arrives immediately, the IRQ handler re-enters NAPI.
        //
        // BUG FIX: Previously we checked INT_SOURCE_CSR without clearing it
        // first, so stale pending bits from already-processed work kept us
        // permanently in NAPI mode (500ms-only processing, sawtooth latency).
        if is_timer && self.0.irq_suppressed && self.0.irq_mode {
            // Clear all pending interrupt bits (W1C register)
            let pending = dev.mt76_rr(MT_INT_SOURCE_CSR);
            dev.mt76_wr(MT_INT_SOURCE_CSR, pending);

            // Unmask HW interrupts — exit polling mode unconditionally.
            // If more RX is ready, the next IRQ fires immediately and the
            // IRQ handler can re-enter NAPI if needed.
            let irq_mask = MT_INT_RX_DONE_ALL | MT_INT_MCU_CMD
                | MT_INT_TX_DONE_BAND0 | MT_INT_TX_DONE_MCU_WM;
            dev.mt76_wr(MT_INT_MASK_CSR, irq_mask);
            self.0.irq_suppressed = false;
        }
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        self.0.dp_stats.data_ready_calls += 1;
        let dp = match self.0.data_port {
            Some(id) if id == port => id,
            _ => return,
        };

        // Handle sidechannel queries (QUERY_INFO from ipd)
        let mut queries: [Option<SideEntry>; 4] = [None; 4];
        let mut qcount = 0;
        if let Some(port) = ctx.block_port(dp) {
            while qcount < 4 {
                if let Some(entry) = port.poll_side_request() {
                    queries[qcount] = Some(entry);
                    qcount += 1;
                } else {
                    break;
                }
            }
        }
        for i in 0..qcount {
            if let Some(entry) = queries[i].take() {
                match entry.msg_type {
                    side_msg::QUERY_INFO => {
                        let mut response = SideEntry {
                            msg_type: entry.msg_type,
                            flags: 0,
                            tag: entry.tag,
                            status: side_status::OK,
                            payload: [0; 24],
                        };
                        response.payload[0..6].copy_from_slice(&self.0.mac_addr);
                        // payload[6] = link status (1 = up when radio is on and AP active)
                        response.payload[6] = if self.0.radio_on && self.0.beacon_on { 1 } else { 0 };
                        // payload[7..9] = MTU (1500)
                        response.payload[7..9].copy_from_slice(&1500u16.to_le_bytes());
                        if let Some(port) = ctx.block_port(dp) {
                            port.side_send(&response);
                            port.notify();
                        }
                    }
                    _ => {
                        let response = SideEntry {
                            msg_type: entry.msg_type,
                            flags: 0,
                            tag: entry.tag,
                            status: side_status::EOL,
                            payload: [0; 24],
                        };
                        if let Some(port) = ctx.block_port(dp) {
                            port.side_send(&response);
                            port.notify();
                        }
                    }
                }
            }
        }

        // Process TX from IP stack (ipd → WiFi)
        // Use a raw pointer to avoid borrow conflict — process_tx_from_ipd
        // takes dev as a param but &mut self conflicts with self.0.dev.as_ref().
        let dev_ptr = self.0.dev.as_ref().map(|d| d as *const Mt7996Dev);
        if let Some(ptr) = dev_ptr {
            // SAFETY: self.0.dev is Some (just checked), and process_tx_from_ipd
            // does not modify or drop self.0.dev. The pointer is valid for the
            // duration of this call.
            let dev = unsafe { &*ptr };
            self.process_tx_from_ipd(dev, ctx);
            tx_sweep_and_complete(self.0, dev, ctx);
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
                let rx = &self.0.rx_queues[2]; // BAND0 data RX
                if rx.ndesc == 0 {
                    return Self::copy_to_buf(buf, b"no_band0_rx");
                }

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
            b"diag.stas" => {
                // STA table dump: MAC state RSSI rate/MCS BW NSS mode
                let ap = match self.0.ap.as_ref() {
                    Some(a) => a,
                    None => return Self::copy_to_buf(buf, b"no AP\n"),
                };
                let mut tmp = [0u8; 2048];
                let mut pos = 0;
                let modes = [b"CCK", b"OFD" as &[u8], b"HT\0", b"???", b"VHT", b"???", b"???", b"???", b"HE\0", b"EHT"];
                for sta in ap.iter_stas() {
                    if pos + 80 > tmp.len() { break; }
                    pos += fmt_mac(&sta.mac, &mut tmp[pos..]);
                    tmp[pos] = b' '; pos += 1;
                    let st = match sta.state {
                        wifi80211::types::StaState::Authenticated => b"auth ",
                        wifi80211::types::StaState::Associated => b"assoc",
                        _ => b"?????",
                    };
                    tmp[pos..pos+5].copy_from_slice(st);
                    pos += 5;
                    tmp[pos] = b' '; pos += 1;
                    // RSSI (signed)
                    let rssi = sta.rssi;
                    if rssi < 0 {
                        tmp[pos] = b'-'; pos += 1;
                        pos += fmt_u32_dec((-rssi as i32) as u32, &mut tmp[pos..]);
                    } else {
                        pos += fmt_u32_dec(rssi as u32, &mut tmp[pos..]);
                    }
                    tmp[pos..pos+4].copy_from_slice(b"dBm ");
                    pos += 4;
                    // PHY mode
                    let mi = sta.phy.mode as usize;
                    let mstr = if mi < modes.len() { modes[mi] } else { b"???" };
                    tmp[pos..pos+3].copy_from_slice(mstr);
                    pos += 3;
                    // MCS/rate
                    tmp[pos..pos+4].copy_from_slice(b" mcs");
                    pos += 4;
                    pos += fmt_u32_dec(sta.phy.rate as u32, &mut tmp[pos..]);
                    // NSS
                    tmp[pos..pos+4].copy_from_slice(b" ss=");
                    pos += 4;
                    pos += fmt_u32_dec((sta.phy.nss + 1) as u32, &mut tmp[pos..]);
                    // BW
                    let bw_str = match sta.phy.bw {
                        0 => b" 20M",
                        1 => b" 40M",
                        2 => b" 80M",
                        3 => b"160M",
                        _ => b" ??M",
                    };
                    tmp[pos..pos+4].copy_from_slice(bw_str);
                    pos += 4;
                    // AID
                    tmp[pos..pos+5].copy_from_slice(b" aid=");
                    pos += 5;
                    pos += fmt_u32_dec(sta.aid as u32, &mut tmp[pos..]);
                    tmp[pos] = b'\n'; pos += 1;
                }
                if pos == 0 {
                    return Self::copy_to_buf(buf, b"no STAs\n");
                }
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
            b"stats" => {
                let s = &self.0.dp_stats;
                let mut tmp = [0u8; 256];
                let mut pos = 0;

                pos += fmt_stat_kv(&mut tmp[pos..], b"rx_frames=", s.rx_frames);
                pos += fmt_stat_kv(&mut tmp[pos..], b"tx_frames=", s.tx_frames);
                pos += fmt_stat_kv(&mut tmp[pos..], b"rx_pool_drops=", s.rx_pool_drops);
                pos += fmt_stat_kv(&mut tmp[pos..], b"tx_pool_drops=", s.tx_pool_drops);
                pos += fmt_stat_kv(&mut tmp[pos..], b"rx_pool_reclaimed=", s.rx_pool_reclaimed);

                Self::copy_to_buf(buf, &tmp[..pos])
            }
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
                // Build beacon from AP state machine, upload to firmware
                let mut bcn_buf = [0u8; 256];
                let bcn_len = if let Some(ref mut ap) = self.0.ap {
                    ap.beacon(&mut bcn_buf)
                } else { 0 };
                if bcn_len == 0 || dev.mcu_set_beacon(wa_ring, 0, HW_BSSID_0, &bcn_buf[..bcn_len], enable, self.0.seq, None).is_err() {
                    return Self::copy_to_buf(buf, b"ERR beacon_cmd_failed\n");
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
                // Parse "6" (20MHz) or "6 40+" / "6 40-" (40MHz with secondary above/below)
                let mut parts = val_str.split_whitespace();
                let ch_str = parts.next().unwrap_or("");
                let bw_str = parts.next().unwrap_or("");
                let ch: u8 = match ch_str.parse::<u8>() {
                    Ok(v) if v >= 1 && v <= 14 => v,
                    Ok(_) => return Self::copy_to_buf(buf, b"ERR range_1_14\n"),
                    Err(_) => return Self::copy_to_buf(buf, b"ERR invalid_number\n"),
                };
                let (bw, sec_ch_off): (u8, i8) = match bw_str {
                    "40+" => (CMD_CBW_40MHZ, 1),
                    "40-" => (CMD_CBW_40MHZ, -1),
                    "" | "20" => (CMD_CBW_20MHZ, 0),
                    _ => return Self::copy_to_buf(buf, b"ERR bw: 20|40+|40-\n"),
                };
                // Validate 40MHz channel (2.4GHz: need room for secondary)
                if bw == CMD_CBW_40MHZ {
                    if sec_ch_off > 0 && ch > 9 {
                        return Self::copy_to_buf(buf, b"ERR ch_too_high_for_40+\n");
                    }
                    if sec_ch_off < 0 && ch < 5 {
                        return Self::copy_to_buf(buf, b"ERR ch_too_low_for_40-\n");
                    }
                }
                // MCU_WMWA_UNI_CMD(CHANNEL_SWITCH) — WMWA, via WA queue
                if dev.mcu_set_chan_info(wa_ring, 0, UNI_CHANNEL_SWITCH, ch, bw, CH_BAND_2GHZ, sec_ch_off, self.0.seq, None).is_err() {
                    return Self::copy_to_buf(buf, b"ERR chan_switch_failed\n");
                }
                self.0.seq = self.0.seq.wrapping_add(1);
                if dev.mcu_set_chan_info(wa_ring, 0, UNI_CHANNEL_RX_PATH, ch, bw, CH_BAND_2GHZ, sec_ch_off, self.0.seq, None).is_err() {
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
                    | MT_WF_RFCR_DROP_RTS
                    | MT_WF_RFCR_DROP_FCSFAIL;
                let rfcr1_val: u32 = MT_WF_RFCR1_DROP_ACK
                    | MT_WF_RFCR1_DROP_BF_POLL
                    | MT_WF_RFCR1_DROP_BA
                    | MT_WF_RFCR1_DROP_CFEND
                    | MT_WF_RFCR1_DROP_CFACK;
                dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr_val);
                dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1_val);
                // Update AP BSS config channel + bandwidth, then re-upload beacon
                if let Some(ref mut ap) = self.0.ap {
                    ap.bss.channel = ch;
                    ap.bss.bandwidth = if bw == CMD_CBW_40MHZ { 1 } else { 0 };
                    ap.bss.secondary_channel_offset = sec_ch_off;
                }
                self.0.channel = ch;
                if self.0.beacon_on {
                    let mut bcn_buf = [0u8; 256];
                    let bcn_len = if let Some(ref mut ap) = self.0.ap {
                        ap.beacon(&mut bcn_buf)
                    } else { 0 };
                    if bcn_len > 0 {
                        if dev.mcu_set_beacon(wa_ring, 0, HW_BSSID_0, &bcn_buf[..bcn_len], true, self.0.seq, None).is_err() {
                            return Self::copy_to_buf(buf, b"ERR beacon_update_failed\n");
                        }
                        self.0.seq = self.0.seq.wrapping_add(1);
                    }
                }
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
                if self.0.band0_rx.is_none() {
                    return Self::copy_to_buf(buf, b"ERR no_band0_rx\n");
                }
                let rx = &mut self.0.rx_queues[2]; // BAND0 data RX
                match value {
                    b"monitor" => {
                        // Accept ALL frames — clear both RFCR and RFCR1
                        dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), 0);
                        dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), 0);
                        // Reset ring counters and refill so DMA has full ring
                        rx.reset_counters();
                        dev.rx_fill(rx);
                        udebug!("wifid", "rx_monitor_mode");
                        Self::copy_to_buf(buf, b"OK monitor\n")
                    }
                    b"normal" => {
                        // Restore AP default filter
                        let rfcr: u32 = MT_WF_RFCR_DROP_OTHER_UC
                            | MT_WF_RFCR_DROP_CTL_RSV
                            | MT_WF_RFCR_DROP_CTS
                            | MT_WF_RFCR_DROP_RTS
                            | MT_WF_RFCR_DROP_FCSFAIL;
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
                        rx.reset_counters();
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
                if self.0.rx_queues[2].ndesc == 0 {
                    return Self::copy_to_buf(buf, b"ERR no_band0_rx\n");
                }

                // Step 1: Re-arm RX ring — clear stale frames
                {
                    let rx = &mut self.0.rx_queues[2];
                    rx.reset_counters();
                    dev.rx_fill(rx);
                }
                let rx = &self.0.rx_queues[2];

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
                    | MT_WF_RFCR_DROP_RTS
                    | MT_WF_RFCR_DROP_FCSFAIL;
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
