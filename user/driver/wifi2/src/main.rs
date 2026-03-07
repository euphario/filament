//! wifi2 — Idiomatic MT7996 WiFi Driver
//!
//! References Linux mt76/mt7996 for hardware behavior, but uses clean Rust
//! idioms instead of C-shaped code. State machine driven, Result-based
//! error handling, structured logging throughout.

#![no_std]
#![no_main]

use userlib::{uinfo, unotice, uwarn, uerror, udebug, ulog};
use userlib::mmio::{MmioRegion, DmaPool};
use userlib::ipc::{Msi, Irq};
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, ConfigKey,
    PortInfo, PortClass, NetworkMetadata, port_subclass,
    BlockPortConfig, PortId,
};
use userlib::bus_runtime::driver_main;
use userlib::ring::{IoSqe, IoCqe, SideEntry, io_op, io_status, side_msg, side_status};

mod regs;
mod device;
mod dma;
mod mcu;
mod firmware;
mod config;
mod mac;
mod event;
mod wpa;

use regs::*;
use device::Mt76Device;
use dma::{TxRing, RxRing, flush_buffer};
use wifi80211::types::{BssConfig, MAX_SSID_LEN, RxMgmtFrame, RxPhyInfo,
                       HandshakeState, StaState, MAX_MGMT_BODY_LEN, ApAction};
use wifi80211::ap::ApManager;
use event::{RxFrameClass, RxEapolFrame};

// ============================================================================
// Driver State Machine
// ============================================================================

/// Driver lifecycle — explicit states, no bool flags.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DriverState {
    /// Just spawned, no hardware access yet.
    Init,
    /// BAR0 mapped, DMA pools allocated, hardware reset done.
    HardwareReady,
    /// Firmware loaded (patch + WM + DSP + WA), MCU responding.
    FirmwareLoaded,
    /// MAC configured, radio enabled, ready for AP/STA operations.
    RadioOn,
    /// AP active, beaconing, accepting STAs.
    ApActive,
}

impl DriverState {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Init => "init",
            Self::HardwareReady => "hw_ready",
            Self::FirmwareLoaded => "fw_loaded",
            Self::RadioOn => "radio_on",
            Self::ApActive => "ap_active",
        }
    }
}

// ============================================================================
// Timer/IRQ Tags
// ============================================================================

const TAG_WIFI_TIMER: u32 = 100;
const TAG_WIFI_IRQ: u32 = 101;

/// TX inflight table size (must be power of 2).
const TX_INFLIGHT_SIZE: usize = 256;
/// Sentinel value for unused TX inflight slots.
const TX_TAG_FREE: u32 = u32::MAX;
/// Maximum Ethernet frame size (header + MTU).
const MAX_FRAME_SIZE: usize = libf::net::eth::FRAME_MAX;
/// First STA WLAN index (0=reserved, 1=BMC → STAs start at 2).
const STA_WLAN_IDX_BASE: u16 = 2;
/// Maximum STA WLAN indices (2..17 = 16 STAs).
const MAX_STA_WLAN_IDX: u16 = 17;

// ============================================================================
// Driver struct
// ============================================================================

struct Wifi2 {
    state: DriverState,
    dev: Option<Mt76Device>,
    bar0: Option<MmioRegion>,
    channel: u8,

    // DMA pools (kept alive for lifetime — drop = unmap)
    _desc_pool: Option<DmaPool>,
    _rx_pool: Option<DmaPool>,
    _fwdl_pool: Option<DmaPool>,
    _mcu_pool: Option<DmaPool>,
    _wa_pool: Option<DmaPool>,
    _tx_band0_pool: Option<DmaPool>,
    _tx_band0_desc_pool: Option<DmaPool>,

    // RX queues
    rx_queues: [RxRing; NUM_RX_QUEUES],
    rx_queue_count: usize,

    // MCU rings
    mcu_ring: Option<TxRing>,
    wa_ring: Option<TxRing>,

    // TX band0 ring (data + management)
    tx_band0: Option<TxRing>,

    // IRQ
    irq: Option<Irq>,
    msi: Option<Msi>,
    irq_mode: bool,

    // MCU command sequence counter
    seq: u8,

    // MAC address
    mac_addr: [u8; 6],

    // AP state
    ap: Option<ApManager>,
    wpa_ctx: Option<wpa::HandshakeCtx>,

    // DataPort for IP stack
    data_port: Option<PortId>,
    pool_phys: u64,

    // TX token tracking
    tx_token: u16,
    /// Maps wifid token → ipd SQE tag for deferred CQE posting.
    tx_inflight_tags: [u32; TX_INFLIGHT_SIZE],

    // CQ tracking (provider side, for pool reclaim)
    cq_post_seq: u32,
    cq_offsets: [u32; TX_INFLIGHT_SIZE],
    last_cq_head: u32,

    // RX frame counter for CQ tags
    net_rx_seq: u32,

    // WLAN index allocation bitmap (bits 2..17 for client STAs)
    wlan_idx_used: u32,

    // Housekeeping tick counter (incremented every 500ms, used for STA aging)
    tick: u32,
    // Sub-tick counter (counts 50ms timer fires, resets at 10 = 500ms)
    subtick: u8,
}

impl Wifi2 {
    fn new() -> Self {
        Self {
            state: DriverState::Init,
            dev: None,
            bar0: None,
            channel: 0,
            _desc_pool: None,
            _rx_pool: None,
            _fwdl_pool: None,
            _mcu_pool: None,
            _wa_pool: None,
            _tx_band0_pool: None,
            _tx_band0_desc_pool: None,
            rx_queues: [RxRing::ZERO; NUM_RX_QUEUES],
            rx_queue_count: 0,
            mcu_ring: None,
            wa_ring: None,
            tx_band0: None,
            irq: None,
            msi: None,
            irq_mode: false,
            seq: 1,
            mac_addr: [0u8; 6],
            ap: None,
            wpa_ctx: None,
            data_port: None,
            pool_phys: 0,
            tx_token: 0,
            tx_inflight_tags: [TX_TAG_FREE; TX_INFLIGHT_SIZE],
            cq_post_seq: 0,
            cq_offsets: [0; TX_INFLIGHT_SIZE],
            last_cq_head: 0,
            net_rx_seq: 0,
            wlan_idx_used: 0,
            tick: 0,
            subtick: 0,
        }
    }

    /// Initialize hardware: map BAR0, allocate DMA, reset WFSYS.
    fn init_hardware(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        // Get BAR0 from spawn context (pcied metadata)
        let spawn_ctx = ctx.spawn_context().map_err(|_| {
            uerror!("wifi2", "no_spawn_context");
            BusError::Internal
        })?;

        let meta = spawn_ctx.metadata();
        if meta.len() < 12 {
            uerror!("wifi2", "metadata_too_short"; len = meta.len() as u32);
            return Err(BusError::Internal);
        }

        let bar0_addr = u64::from_le_bytes([
            meta[0], meta[1], meta[2], meta[3],
            meta[4], meta[5], meta[6], meta[7],
        ]);
        let bar0_size = u32::from_le_bytes([
            meta[8], meta[9], meta[10], meta[11],
        ]) as u64;

        let bdf = if meta.len() >= 16 {
            u32::from_le_bytes([meta[12], meta[13], meta[14], meta[15]])
        } else {
            0
        };

        unotice!("wifi2", "device_found"; bar0 = bar0_addr, size = bar0_size, bdf = bdf as u64);
        ulog::flush();

        // Map BAR0
        let bar0 = MmioRegion::open(bar0_addr, bar0_size).ok_or_else(|| {
            uerror!("wifi2", "mmap_failed");
            BusError::Internal
        })?;
        let bar0_virt = bar0.virt_base();

        // Allocate DMA descriptor memory (LOW — DESC_BASE is 32-bit)
        const DESC_MEM_SIZE: usize = 256 * 1024;
        let mut desc_pool = DmaPool::alloc(DESC_MEM_SIZE).ok_or_else(|| {
            uerror!("wifi2", "desc_pool_failed");
            BusError::Internal
        })?;
        let desc_virt = desc_pool.vaddr();
        let desc_phys = desc_pool.paddr();
        desc_pool.zero();
        flush_buffer(desc_virt, DESC_MEM_SIZE);

        // Allocate RX buffer pool (HIGH — 36-bit addresses OK)
        const RX_BUF_POOL_SIZE: usize = 13 * 1024 * 1024;
        let mut rx_pool = DmaPool::alloc_high(RX_BUF_POOL_SIZE)
            .or_else(|| DmaPool::alloc(RX_BUF_POOL_SIZE))
            .ok_or_else(|| {
                uerror!("wifi2", "rx_pool_failed");
                BusError::Internal
            })?;
        let rx_buf_virt = rx_pool.vaddr();
        let rx_buf_phys = rx_pool.paddr();
        rx_pool.zero();
        flush_buffer(rx_buf_virt, RX_BUF_POOL_SIZE);

        let dev = Mt76Device::new(bar0_virt, bar0_size, true);

        // ================================================================
        // Hardware init sequence (init.c:mt7996_init_hardware)
        // ================================================================

        // Fix HOST_CONFIG (bits 8,10-14)
        dev.set(MT_WFDMA_HOST_CONFIG, 0x7d00);

        // Fix PCIE_SETTING to match OpenWRT (0x00003180)
        dev.wr(MT_PCIE_SETTING, 0x3180);

        // Read HW revision via L1 remap (mmio.c:672)
        let hw_rev = dev.rr_remap(MT_HW_REV);
        uinfo!("wifi2", "hw_rev"; rev = hw_rev);

        // Clear interrupt mask before WFSYS reset
        // Source: wifid reset() — mask=0 before reset, then clear sources after
        dev.wr(MT_INT_MASK_CSR, 0);

        // WFSYS reset — clean hardware state
        // Source: init.c:762-769
        dev.wfsys_reset();

        // PCIe MAC interrupt enable — required for DMA completion signaling
        // Source: pci.c mt7996_pci_probe() — enables PCIe MAC IRQs on both HIFs
        dev.wr(MT_PCIE_MAC_INT_ENABLE, 0xff);
        dev.wr_remap(MT_PCIE1_MAC_INT_ENABLE_PHYS, 0xff);

        // Disable interrupt masks, clear pending sources
        dev.wr(MT_INT_MASK_CSR, 0);
        dev.wr(MT_INT1_MASK_CSR, 0);
        dev.wr(MT_INT_SOURCE_CSR, !0u32);

        // ================================================================
        // DMA init
        // ================================================================
        dma::dma_disable(&dev, true);

        // Initialize RX queues with Linux-matching ring sizes
        // HIF2 ring base for BAND2 — on second PCIE interface
        let rx_base_band2 = if dev.has_hif2 {
            MT_WFDMA0_RX_RING_BASE + HIF1_OFS
        } else {
            MT_WFDMA0_RX_RING_BASE
        };

        let rx_queue_configs: [(u32, u32, u32, u32); 6] = [
            // (hw_idx, ring_base, ndesc, buf_size)
            (MT7996_RXQ_MCU_WM,      MT_WFDMA0_RX_RING_BASE, MT7996_RX_MCU_RING_SIZE,    MT7996_RX_MCU_BUF_SIZE),
            (MT7996_RXQ_MCU_WA,      MT_WFDMA0_RX_RING_BASE, MT7996_RX_MCU_RING_SIZE,    MT7996_RX_MCU_BUF_SIZE),
            (MT7996_RXQ_MCU_WA_MAIN, MT_WFDMA0_RX_RING_BASE, MT7996_RX_MCU_RING_SIZE_WA, MT7996_RX_BUF_SIZE),
            (MT7996_RXQ_MCU_WA_TRI,  MT_WFDMA0_RX_RING_BASE, MT7996_RX_MCU_RING_SIZE_WA, MT7996_RX_BUF_SIZE),
            (MT7996_RXQ_BAND0,       MT_WFDMA0_RX_RING_BASE, MT7996_RX_RING_SIZE,         MT7996_RX_BUF_SIZE),
            (MT7996_RXQ_BAND2,       rx_base_band2,           MT7996_RX_RING_SIZE,         MT7996_RX_BUF_SIZE),
        ];

        let mut desc_offset: usize = 0;
        let mut rx_buf_offset: usize = 0;

        for (i, &(hw_idx, ring_base, ndesc, buf_size)) in rx_queue_configs.iter().enumerate() {
            let desc_size = ndesc as usize * core::mem::size_of::<dma::Descriptor>();
            // Align descriptor offset to 4KB
            desc_offset = (desc_offset + 0xFFF) & !0xFFF;

            let q_desc_virt = desc_virt + desc_offset as u64;
            let q_desc_phys = desc_phys + desc_offset as u64;

            let q_buf_virt = rx_buf_virt + rx_buf_offset as u64;
            let q_buf_phys = rx_buf_phys + rx_buf_offset as u64;

            let regs_base = ring_base + hw_idx * MT_RING_SIZE;

            dma::init_queue(&dev, ring_base, hw_idx, ndesc, q_desc_phys, q_desc_virt);

            self.rx_queues[i] = RxRing {
                hw_idx,
                regs_base,
                ndesc,
                desc_virt: q_desc_virt,
                buf_size,
                buf_phys: q_buf_phys,
                buf_virt: q_buf_virt,
                head: 0,
                tail: 0,
                queued: 0,
            };

            desc_offset += desc_size;
            rx_buf_offset += ndesc as usize * buf_size as usize;
        }

        self.rx_queue_count = 6;

        // Fill all RX queues
        for q in self.rx_queues[..self.rx_queue_count].iter_mut() {
            q.fill(&dev);
        }

        // DMA enable
        dma::dma_enable(&dev, false);
        uinfo!("wifi2", "dma_enabled");
        ulog::flush();

        // ================================================================
        // MCU init
        // ================================================================
        dev.reg_wr(MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE);

        dev.driver_own(0).map_err(|e| {
            uerror!("wifi2", "driver_own_0_failed"; err = e.name());
            BusError::Internal
        })?;
        dev.driver_own(1).map_err(|e| {
            uerror!("wifi2", "driver_own_1_failed"; err = e.name());
            BusError::Internal
        })?;
        uinfo!("wifi2", "driver_own_ok");
        ulog::flush();

        // ================================================================
        // Allocate MCU TX rings — each gets its own DMA buffer pool
        // (RX pool is fully used by RX queues; TX needs separate allocs)
        // ================================================================

        // FWDL ring (hw_idx=16) — firmware download chunks
        let fwdl_ndesc = MT7996_TX_FWDL_RING_SIZE;
        desc_offset = (desc_offset + 0xFFF) & !0xFFF;
        let fwdl_desc_virt = desc_virt + desc_offset as u64;
        let fwdl_desc_phys = desc_phys + desc_offset as u64;
        desc_offset += fwdl_ndesc as usize * core::mem::size_of::<dma::Descriptor>();

        let fwdl_buf_size = fwdl_ndesc as usize * MCU_FW_DL_BUF_SIZE;
        let fwdl_pool = DmaPool::alloc_high(fwdl_buf_size)
            .or_else(|| DmaPool::alloc(fwdl_buf_size))
            .ok_or_else(|| {
                uerror!("wifi2", "fwdl_pool_failed");
                BusError::Internal
            })?;
        let fwdl_buf_virt = fwdl_pool.vaddr();
        let fwdl_buf_phys = fwdl_pool.paddr();

        dma::init_queue(&dev, MT_WFDMA0_TX_RING_BASE, MT7996_TXQ_FWDL, fwdl_ndesc, fwdl_desc_phys, fwdl_desc_virt);
        let mut fwdl_ring = TxRing::new(
            MT_WFDMA0_TX_RING_BASE + MT7996_TXQ_FWDL * MT_RING_SIZE,
            fwdl_ndesc, fwdl_desc_virt, fwdl_desc_phys,
            fwdl_buf_virt, fwdl_buf_phys,
        );

        // MCU WM ring (hw_idx=17) — MCU commands
        let wm_ndesc = MT7996_TX_MCU_RING_SIZE;
        desc_offset = (desc_offset + 0xFFF) & !0xFFF;
        let wm_desc_virt = desc_virt + desc_offset as u64;
        let wm_desc_phys = desc_phys + desc_offset as u64;
        desc_offset += wm_ndesc as usize * core::mem::size_of::<dma::Descriptor>();

        let mcu_buf_size = wm_ndesc as usize * MCU_FW_DL_BUF_SIZE;
        let mcu_pool = DmaPool::alloc_high(mcu_buf_size)
            .or_else(|| DmaPool::alloc(mcu_buf_size))
            .ok_or_else(|| {
                uerror!("wifi2", "mcu_pool_failed");
                BusError::Internal
            })?;
        let wm_buf_virt = mcu_pool.vaddr();
        let wm_buf_phys = mcu_pool.paddr();

        dma::init_queue(&dev, MT_WFDMA0_TX_RING_BASE, MT7996_TXQ_MCU_WM, wm_ndesc, wm_desc_phys, wm_desc_virt);
        let mut mcu_ring = TxRing::new(
            MT_WFDMA0_TX_RING_BASE + MT7996_TXQ_MCU_WM * MT_RING_SIZE,
            wm_ndesc, wm_desc_virt, wm_desc_phys,
            wm_buf_virt, wm_buf_phys,
        );

        // Set up response RX queue pointers for send_and_wait
        mcu_ring.rx_regs = MCU_WM_RX_REGS;
        mcu_ring.rx_buf_virt = self.rx_queues[0].buf_virt;
        mcu_ring.rx_buf_size = self.rx_queues[0].buf_size;

        // MCU WA ring (hw_idx=20)
        let wa_ndesc = MT7996_TX_MCU_RING_SIZE;
        desc_offset = (desc_offset + 0xFFF) & !0xFFF;
        let wa_desc_virt = desc_virt + desc_offset as u64;
        let wa_desc_phys = desc_phys + desc_offset as u64;

        let wa_buf_size = wa_ndesc as usize * MCU_FW_DL_BUF_SIZE;
        let wa_pool = DmaPool::alloc_high(wa_buf_size)
            .or_else(|| DmaPool::alloc(wa_buf_size))
            .ok_or_else(|| {
                uerror!("wifi2", "wa_pool_failed");
                BusError::Internal
            })?;
        let wa_buf_virt = wa_pool.vaddr();
        let wa_buf_phys = wa_pool.paddr();

        dma::init_queue(&dev, MT_WFDMA0_TX_RING_BASE, MT7996_TXQ_MCU_WA, wa_ndesc, wa_desc_phys, wa_desc_virt);
        let mut wa_ring = TxRing::new(
            MT_WFDMA0_TX_RING_BASE + MT7996_TXQ_MCU_WA * MT_RING_SIZE,
            wa_ndesc, wa_desc_virt, wa_desc_phys,
            wa_buf_virt, wa_buf_phys,
        );
        wa_ring.rx_regs = MCU_WA_RX_REGS;
        wa_ring.rx_buf_virt = self.rx_queues[1].buf_virt;
        wa_ring.rx_buf_size = self.rx_queues[1].buf_size;

        uinfo!("wifi2", "tx_rings_ready";
            fwdl_buf = fwdl_buf_phys, mcu_buf = wm_buf_phys, wa_buf = wa_buf_phys);
        ulog::flush();

        // ================================================================
        // Try to get MSI IRQ
        // ================================================================
        if bdf != 0 {
            if let Ok(msi) = Msi::allocate(bdf, 1) {
                let irq_num = msi.first_irq();
                if let Ok(irq) = Irq::new(irq_num) {
                    uinfo!("wifi2", "msi_irq_ready"; irq = irq_num);
                    self.irq = Some(irq);
                    self.msi = Some(msi);
                    self.irq_mode = true;
                }
            }
        }

        // ================================================================
        // Load firmware
        // ================================================================
        // Phase 1: use polling for firmware load (IRQ path available for Phase 2)
        firmware::load_all(&dev, &mut mcu_ring, &mut fwdl_ring, None).map_err(|e| {
            uerror!("wifi2", "firmware_load_failed"; err = e.name());
            BusError::Internal
        })?;

        self.state = DriverState::FirmwareLoaded;
        unotice!("wifi2", "firmware_loaded");

        // Store state (pools kept alive to prevent DMA memory unmap)
        self.dev = Some(dev);
        self.bar0 = Some(bar0);
        self.mcu_ring = Some(mcu_ring);
        self.wa_ring = Some(wa_ring);
        self._desc_pool = Some(desc_pool);
        self._rx_pool = Some(rx_pool);
        self._fwdl_pool = Some(fwdl_pool);
        self._mcu_pool = Some(mcu_pool);
        self._wa_pool = Some(wa_pool);

        Ok(())
    }

    /// Post-firmware init: MCU subsystem, EEPROM, MAC, radio, interface, beacon.
    /// Takes driver from FirmwareLoaded → ApActive.
    /// Source: wifid/main.rs lines 716-1182
    fn init_radio(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        let dev = self.dev.as_ref().ok_or(BusError::Internal)?;
        let ring = self.wa_ring.as_mut().ok_or(BusError::Internal)?;
        let seq = &mut self.seq;

        let mcu_err = |_e: mcu::McuError| BusError::Internal;

        // ================================================================
        // MCU subsystem init
        // ================================================================

        // Enable WM + WA logging
        mcu::fw_log_2_host(dev, ring, 0, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);
        mcu::fw_log_2_host(dev, ring, 1, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // Enable MWDS
        mcu::set_mwds(dev, ring, true, *seq).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // RX airtime for bands 0,1,2
        mcu::init_rx_airtime(dev, ring, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // Random Early Drop enable
        mcu::wa_cmd(dev, ring, MCU_WA_PARAM_RED, 0, 0, *seq).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        udebug!("wifi2", "mcu_subsys_ok");

        // ================================================================
        // EEPROM init
        // ================================================================

        let free_blocks = mcu::get_eeprom_free_block(dev, ring, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        if free_blocks >= 59 {
            // eFuse empty — upload embedded eeprom.bin
            udebug!("wifi2", "eeprom_flash"; size = firmware::FW_EEPROM.len() as u32);
            mcu::set_eeprom_flash(dev, ring, firmware::FW_EEPROM, seq, None).map_err(mcu_err)?;
        } else {
            // eFuse has data — use eFuse mode
            udebug!("wifi2", "eeprom_efuse"; free = free_blocks as u32);
            mcu::set_eeprom(dev, ring, *seq, None).map_err(mcu_err)?;
            *seq = seq.wrapping_add(1);
        }

        // Read WiFi config from eFuse (antenna paths)
        match mcu::get_eeprom(dev, ring, 0x190, *seq, None) {
            Ok(wifi_conf) => {
                *seq = seq.wrapping_add(1);
                let tx = (wifi_conf[1] >> 3) & 0x7;
                let rx = wifi_conf[3] & 0x7;
                let nss = (wifi_conf[4] >> 3) & 0x7;
                udebug!("wifi2", "efuse_wifi_conf"; tx = tx as u32, rx = rx as u32, nss = nss as u32);
            }
            Err(e) => {
                uwarn!("wifi2", "efuse_read_fail"; err = e.name());
                *seq = seq.wrapping_add(1);
            }
        }

        // Read ADIE chip ID
        match mcu::rf_regval(dev, ring, MT_ADIE_CHIP_ID_0, *seq, None) {
            Ok(regval) => {
                let id = (regval >> 16) & 0xFFFF;
                let ver = regval & 0xFFFF;
                udebug!("wifi2", "adie_chip"; id = id, ver = ver);
            }
            Err(e) => {
                uwarn!("wifi2", "adie_read_fail"; err = e.name());
            }
        }
        *seq = seq.wrapping_add(1);

        // ================================================================
        // MAC init: WTBL clear, RRO, HIF TXD, per-band regs, basic rates
        // ================================================================

        mac::init(dev, ring, seq, None).map_err(mcu_err)?;
        udebug!("wifi2", "mac_init_ok");

        // TxBF init
        mcu::txbf_init(dev, ring, seq, None).map_err(mcu_err)?;

        // ================================================================
        // Thermal protection + throttling
        // ================================================================

        for band in 0..3u8 {
            mcu::set_thermal_protect(dev, ring, band, true, *seq).map_err(mcu_err)?;
            *seq = seq.wrapping_add(1);
        }

        for band in 0..3u8 {
            mcu::set_thermal_throttling(dev, ring, band, 100, *seq).map_err(mcu_err)?;
            *seq = seq.wrapping_add(1);
        }

        // ================================================================
        // Band 0 radio startup
        // ================================================================

        let mac_addr: [u8; 6] = [0x02, 0x0c, 0x43, 0x28, 0x80, 0x01];

        // Header translation (global, once)
        mcu::set_hdr_trans(dev, ring, true, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // Noise floor measurement
        mac::enable_nf(dev, 0);

        // RX filter — drop noise, keep management + data
        // (DROP_OTHER_UC drops unicast not addressed to our BSS, not from unknown STAs)
        let rfcr_val: u32 = MT_WF_RFCR_DROP_OTHER_UC
            | MT_WF_RFCR_DROP_CTL_RSV
            | MT_WF_RFCR_DROP_CTS
            | MT_WF_RFCR_DROP_RTS
            | MT_WF_RFCR_DROP_FCSFAIL;
        dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr_val);

        let rfcr1_val: u32 = MT_WF_RFCR1_DROP_ACK
            | MT_WF_RFCR1_DROP_BF_POLL
            | MT_WF_RFCR1_DROP_BA
            | MT_WF_RFCR1_DROP_CFEND
            | MT_WF_RFCR1_DROP_CFACK;
        dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1_val);

        // RTS threshold
        mcu::set_rts_thresh(dev, ring, 0, 0x92b, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // Radio ON
        mcu::set_radio_en(dev, ring, 0, true, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // RX path init
        mcu::set_chan_info(dev, ring, 0, UNI_CHANNEL_RX_PATH, 1,
            CMD_CBW_20MHZ, CH_BAND_2GHZ, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // ================================================================
        // Interface creation (band 0)
        // ================================================================

        // DEV_INFO: activate OMAC
        mcu::add_dev_info(dev, ring, 0, HW_BSSID_0, &mac_addr, true, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // WTBL: clear ADM count on BMC STA
        mac::wtbl_update(dev, MT7996_WTBL_RESERVED as u32, MT_WTBL_UPDATE_ADM_COUNT_CLEAR);

        // BSS_INFO: create BSS
        mcu::add_bss_info(dev, ring, 0, HW_BSSID_0, 0, &mac_addr, true,
            1, CMD_CBW_20MHZ, 0, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // STA_REC: add BMC STA
        mcu::add_sta(dev, ring, 0, MT7996_WTBL_RESERVED, 0,
            CONN_STATE_PORT_SECURE, &mac_addr, true, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // VOW group
        mcu::add_group(dev, ring, 0, MT7996_WTBL_RESERVED, *seq, None, true).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // ================================================================
        // Channel switch + TX power
        // ================================================================

        mcu::set_chan_info(dev, ring, 0, UNI_CHANNEL_SWITCH, 1,
            CMD_CBW_20MHZ, CH_BAND_2GHZ, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        mcu::set_chan_info(dev, ring, 0, UNI_CHANNEL_RX_PATH, 1,
            CMD_CBW_20MHZ, CH_BAND_2GHZ, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        mcu::set_txpower_sku(dev, ring, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        mac::cca_stats_reset(dev, 0);

        // Radio diagnostics
        let arb_scr = dev.reg_rr(mt_wf_arb(0, MT_ARB_SCR_OFS));
        if arb_scr & MT_ARB_SCR_RX_DISABLE != 0 {
            uerror!("wifi2", "rx_disabled_in_arb");
        }
        if arb_scr & MT_ARB_SCR_TX_DISABLE != 0 {
            uwarn!("wifi2", "tx_disabled_in_arb");
        }

        self.channel = 1;
        self.state = DriverState::RadioOn;
        unotice!("wifi2", "radio_on"; channel = 1u32);

        // ================================================================
        // Beacon rate tables
        // ================================================================

        // Beacon rate: 1 Mbps CCK
        let beacon_rate_idx = MT7996_BEACON_RATES_TBL + 2 * 0;
        mcu::set_fixed_rate_table(dev, ring, beacon_rate_idx, 0x0000, true, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // Basic rate: 1 Mbps CCK
        mcu::set_fixed_rate_table(dev, ring, MT7996_BASIC_RATES_TBL, 0x0000, false, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // ================================================================
        // Beacon enable sequence — re-send BSS + STA + group + EDCA + beacon
        // ================================================================

        // Second BSS_INFO
        mcu::add_bss_info(dev, ring, 0, HW_BSSID_0, 0, &mac_addr, true,
            1, CMD_CBW_20MHZ, 0, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // BSS cipher mode — tell firmware this BSS uses AES-CCMP (WPA2)
        // Must be set BEFORE any per-STA key installation.
        // Source: Linux mt7996/main.c:240-244 mt7996_set_key(), first group key path
        mcu::update_bss_sec(dev, ring, 0, MCU_CIPHER_AES_CCMP, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // Second STA_REC (newly=false, update existing)
        mcu::add_sta(dev, ring, 0, MT7996_WTBL_RESERVED, 0,
            CONN_STATE_PORT_SECURE, &mac_addr, false, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // VOW reassign
        mcu::add_group(dev, ring, 0, MT7996_WTBL_RESERVED, *seq, None, true).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // EDCA/WMM parameters
        mcu::set_edca(dev, ring, 0, *seq, None).map_err(mcu_err)?;
        *seq = seq.wrapping_add(1);

        // Beacon offload
        {
            let mut bss = BssConfig {
                bssid: mac_addr,
                ssid: [0u8; MAX_SSID_LEN],
                ssid_len: 8,
                channel: 1,
                bandwidth: 0,
                secondary_channel_offset: 0,
                erp_protection: false,
                ht_protection: 0,
                wpa2: true,
            };
            bss.ssid[..8].copy_from_slice(b"Filament");
            let empty_tim = [0u8; 8];
            let mut bcn_buf = [0u8; 256];
            let bcn_len = wifi80211::frame::build_beacon(&mut bcn_buf, &bss, 0, 0, &empty_tim);
            mcu::set_beacon(dev, ring, 0, HW_BSSID_0, &bcn_buf[..bcn_len], true, *seq, None).map_err(mcu_err)?;
        }
        *seq = seq.wrapping_add(1);

        // Re-set RFCR after all MCU commands
        dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR_OFS), rfcr_val);
        dev.reg_wr(mt_wf_rmac(0, MT_WF_RFCR1_OFS), rfcr1_val);

        // ================================================================
        // TX Band0 ring
        // ================================================================

        const TX_BAND0_BUF_STRIDE: usize = 2048;
        const TX_BAND0_NDESC: u32 = MT7996_TX_RING_SIZE;
        const TX_BAND0_BUF_SIZE: usize = TX_BAND0_NDESC as usize * TX_BAND0_BUF_STRIDE;

        let tx_band0_pool = DmaPool::alloc_high(TX_BAND0_BUF_SIZE)
            .or_else(|| DmaPool::alloc(TX_BAND0_BUF_SIZE))
            .ok_or_else(|| {
                uerror!("wifi2", "tx_band0_alloc_failed");
                BusError::Internal
            })?;

        // Separate descriptor pool — can't reuse _desc_pool at offset 0
        // because that memory is occupied by RX queue descriptors.
        let desc_size = TX_BAND0_NDESC as usize * core::mem::size_of::<dma::Descriptor>();
        let desc_alloc = (desc_size + 0xFFF) & !0xFFF; // page-aligned
        let tx_b0_desc_pool = DmaPool::alloc(desc_alloc).ok_or_else(|| {
            uerror!("wifi2", "tx_band0_desc_alloc_failed");
            BusError::Internal
        })?;
        let b0_desc_virt = tx_b0_desc_pool.vaddr();
        let b0_desc_phys = tx_b0_desc_pool.paddr();

        dma::init_queue(dev, MT_WFDMA0_TX_RING_BASE, MT7996_TXQ_BAND0, TX_BAND0_NDESC, b0_desc_phys, b0_desc_virt);

        let mut tx_band0 = TxRing::new(
            MT_WFDMA0_TX_RING_BASE + MT7996_TXQ_BAND0 * MT_RING_SIZE,
            TX_BAND0_NDESC,
            b0_desc_virt, b0_desc_phys,
            tx_band0_pool.vaddr(), tx_band0_pool.paddr(),
        );
        tx_band0.buf_stride = TX_BAND0_BUF_STRIDE;

        self.tx_band0 = Some(tx_band0);
        self._tx_band0_pool = Some(tx_band0_pool);
        self._tx_band0_desc_pool = Some(tx_b0_desc_pool);

        // ================================================================
        // IRQ transition to event-driven mode
        // ================================================================

        if let Some(ref irq) = self.irq {
            let handle = irq.handle();
            match ctx.watch_handle(handle, TAG_WIFI_IRQ) {
                Ok(()) => {
                    let irq_mask = MT_INT_RX_DONE_ALL | MT_INT_MCU_CMD
                        | MT_INT_TX_DONE_BAND0 | MT_INT_TX_DONE_MCU_WM
                        | MT_INT_TX_DONE_FWDL;
                    dev.wr(MT_INT_MASK_CSR, irq_mask);
                    unotice!("wifi2", "irq_mode");
                }
                Err(_) => {
                    uwarn!("wifi2", "watch_handle_fail");
                }
            }
        }

        // ================================================================
        // AP state machine + WPA2
        // ================================================================

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
            wpa2: true,
        }));

        // WPA2-PSK handshake context
        let mut rsn_ie = [0u8; 22];
        let rsn_ie_len = wifi80211::frame::build_rsn_ie(&mut rsn_ie);
        self.wpa_ctx = Some(wpa::HandshakeCtx::new(
            b"filament",
            b"Filament",
            &mac_addr,
            &rsn_ie,
            rsn_ie_len,
        ));

        // ================================================================
        // DataPort for IP stack data exchange
        // ================================================================

        let dp_config = BlockPortConfig { ring_size: 256, side_size: 4, pool_size: 1024 * 1024 };
        match ctx.create_block_port(dp_config) {
            Ok(port_id) => {
                if let Some(port) = ctx.block_port(port_id) {
                    port.set_public();
                }
                let shmem_id = ctx.block_port(port_id).map(|p| p.shmem_id()).unwrap_or(0);
                let mut info = PortInfo::new(b"wifi:0", PortClass::Network);
                info.port_subclass = port_subclass::NET_WIFI_DATA;
                let mut meta = NetworkMetadata::empty();
                meta.mac.copy_from_slice(&mac_addr);
                info.set_network_metadata(meta);
                let _ = ctx.register_port_with_info(&info, shmem_id);
                self.data_port = Some(port_id);
                self.pool_phys = ctx.block_port(port_id).map(|p| p.pool_phys()).unwrap_or(0);
                uinfo!("wifi2", "dataport_ready"; shmem = shmem_id as u64, pool_phys = self.pool_phys);
            }
            Err(e) => {
                uerror!("wifi2", "dataport_failed"; err = e as u32);
            }
        }

        // Initialize TX inflight table
        self.tx_token = 0;
        self.tx_inflight_tags = [TX_TAG_FREE; TX_INFLIGHT_SIZE];
        self.cq_post_seq = 0;
        self.cq_offsets = [0; TX_INFLIGHT_SIZE];
        self.last_cq_head = 0;
        self.net_rx_seq = 0;
        self.wlan_idx_used = 0;
        self.tick = 0;

        self.mac_addr = mac_addr;
        self.state = DriverState::ApActive;

        let fw_state = dev.rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
        unotice!("wifi2", "ap_active"; fw_state = fw_state, channel = 1u32);

        Ok(())
    }

    /// Allocate a WLAN index for a new client STA (range 2..17).
    fn alloc_wlan_idx(&mut self) -> Option<u16> {
        for idx in STA_WLAN_IDX_BASE..=MAX_STA_WLAN_IDX {
            if self.wlan_idx_used & (1 << idx) == 0 {
                self.wlan_idx_used |= 1 << idx;
                return Some(idx);
            }
        }
        None
    }

    /// Free a WLAN index back to the pool.
    fn free_wlan_idx(&mut self, idx: u16) {
        if idx >= STA_WLAN_IDX_BASE && idx <= MAX_STA_WLAN_IDX {
            self.wlan_idx_used &= !(1 << idx);
        }
    }

    /// Allocate a TX token and return it.
    fn next_tx_token(&mut self) -> u16 {
        let tok = self.tx_token;
        self.tx_token = self.tx_token.wrapping_add(1);
        tok
    }

    /// Extract RxPhyInfo from Group 3 (P-RXV) in an RX buffer.
    /// Source: Linux mt76/mac.c mt76_connac3_mac_fill_rx()
    fn extract_phy_info(buf: &[u8], rxd1: u32) -> RxPhyInfo {
        if rxd1 & MT_RXD1_NORMAL_GROUP_3 == 0 {
            return RxPhyInfo::UNKNOWN;
        }
        let mut g3_ofs: usize = 32;
        if rxd1 & MT_RXD1_NORMAL_GROUP_4 != 0 { g3_ofs += 16; }
        if rxd1 & MT_RXD1_NORMAL_GROUP_1 != 0 { g3_ofs += 16; }
        if rxd1 & MT_RXD1_NORMAL_GROUP_2 != 0 { g3_ofs += 16; }
        if g3_ofs + 16 > buf.len() {
            return RxPhyInfo::UNKNOWN;
        }
        let g3 = &buf[g3_ofs..g3_ofs + 16];
        let g3_dw0 = u32::from_le_bytes([g3[0], g3[1], g3[2], g3[3]]);
        let g3_dw1 = u32::from_le_bytes([g3[4], g3[5], g3[6], g3[7]]);
        let g3_dw3 = u32::from_le_bytes([g3[12], g3[13], g3[14], g3[15]]);
        let rcpi = [
            (g3_dw3 & 0xFF) as u8,
            ((g3_dw3 >> 8) & 0xFF) as u8,
            ((g3_dw3 >> 16) & 0xFF) as u8,
            ((g3_dw3 >> 24) & 0xFF) as u8,
        ];
        let rssi = (((rcpi[0] as i32) - 220) / 2).clamp(-128, 0) as i8;
        RxPhyInfo {
            rssi,
            rcpi,
            rate: (g3_dw0 & 0x7F) as u8,
            nss: ((g3_dw0 >> 7) & 0x7) as u8,
            bw: ((g3_dw1 >> 11) & 0xF) as u8,
            mode: ((g3_dw0 >> 12) & 0x7) as u8,
            gi: ((g3_dw0 >> 15) & 0x3) as u8,
            stbc: (g3_dw0 >> 22) & 1 != 0,
        }
    }
}

// ============================================================================
// RX frame collector — snapshot frames from DMA buffers during process()
// ============================================================================

/// Collected management frame (copied from DMA buffer before it's freed).
struct MgmtCollected {
    fc0: u8,
    addr2: [u8; 6],
    addr3: [u8; 6],
    rssi: i8,
    phy: RxPhyInfo,
    body: [u8; MAX_MGMT_BODY_LEN],
    body_len: u16,
}

/// Collected data frame — stored as (offset, len) into shared data_buf.
struct DataCollected {
    offset: usize,
    len: usize,
}

/// Accumulates RX frames during a single process() pass over the BAND0 queue.
///
/// DMA buffers are freed when the process() closure returns, so we snapshot
/// everything we need into stack arrays. The match arms in handle_event
/// call named methods: collect_mgmt, collect_eth_data, collect_raw_eapol.
struct RxCollector {
    mgmt_frames: [Option<MgmtCollected>; 8],
    mgmt_count: usize,

    data_frames: [Option<DataCollected>; 16],
    data_count: usize,
    data_buf: [u8; 24576],
    data_buf_pos: usize,

    eapol_frames: [Option<RxEapolFrame>; 4],
    eapol_count: usize,

    total: usize,
    dropped: usize,
    fcs_errors: usize,
}

impl RxCollector {
    fn new() -> Self {
        Self {
            mgmt_frames: [const { None }; 8],
            mgmt_count: 0,
            data_frames: [const { None }; 16],
            data_count: 0,
            data_buf: [0u8; 24576],
            data_buf_pos: 0,
            eapol_frames: [const { None }; 4],
            eapol_count: 0,
            total: 0,
            dropped: 0,
            fcs_errors: 0,
        }
    }

    /// Snapshot a management frame: extract addr2, addr3, body, PHY info.
    fn collect_mgmt(&mut self, buf: &[u8], rxd: &event::RxdInfo, frame_end: usize) {
        if self.mgmt_count >= self.mgmt_frames.len() { return; }
        let ofs = rxd.frame_ofs;
        if ofs + 24 > frame_end { return; }

        let fc0 = buf[ofs];
        let mut addr2 = [0u8; 6];
        let mut addr3 = [0u8; 6];
        addr2.copy_from_slice(&buf[ofs + 10..ofs + 16]);
        addr3.copy_from_slice(&buf[ofs + 16..ofs + 22]);

        // Body starts after 802.11 header (24 bytes)
        let body_start = ofs + 24;
        let body_len = frame_end.saturating_sub(body_start).min(MAX_MGMT_BODY_LEN);
        let mut body = [0u8; MAX_MGMT_BODY_LEN];
        if body_len > 0 && body_start + body_len <= buf.len() {
            body[..body_len].copy_from_slice(&buf[body_start..body_start + body_len]);
        }

        let phy = Wifi2::extract_phy_info(buf, rxd.rxd1);

        self.mgmt_frames[self.mgmt_count] = Some(MgmtCollected {
            fc0,
            addr2,
            addr3,
            rssi: rxd.rssi,
            phy,
            body,
            body_len: body_len as u16,
        });
        self.mgmt_count += 1;
    }

    /// Snapshot a header-translated (Ethernet) data frame.
    /// Intercepts EAPOL (EtherType 0x888E) before treating as data.
    fn collect_eth_data(&mut self, frame: &[u8]) {
        use libf::net::{eth, ethertype};

        // Check for EAPOL in header-translated frames
        if let Some(hdr) = eth::parse(frame) {
            if hdr.ethertype == ethertype::EAPOL {
                self.collect_eth_eapol(frame, &hdr.src);
                return;
            }
        }

        if self.data_count >= self.data_frames.len() { return; }
        let len = frame.len();
        if self.data_buf_pos + len > self.data_buf.len() { return; }

        let offset = self.data_buf_pos;
        self.data_buf[offset..offset + len].copy_from_slice(frame);
        self.data_buf_pos += len;

        self.data_frames[self.data_count] = Some(DataCollected { offset, len });
        self.data_count += 1;
    }

    /// Extract EAPOL from a header-translated Ethernet frame.
    fn collect_eth_eapol(&mut self, frame: &[u8], src: &libf::net::EthAddr) {
        if self.eapol_count >= self.eapol_frames.len() { return; }

        let body_start = libf::net::eth::HEADER_LEN;
        let body_len = frame.len().saturating_sub(body_start).min(256);
        let mut ef = RxEapolFrame::EMPTY;
        ef.src_mac = src.0;
        if body_len > 0 {
            ef.body[..body_len].copy_from_slice(&frame[body_start..body_start + body_len]);
        }
        ef.body_len = body_len as u16;

        self.eapol_frames[self.eapol_count] = Some(ef);
        self.eapol_count += 1;
    }

    /// Process a raw 802.11 (non-header-translated) data frame.
    ///
    /// EAPOL frames are extracted for WPA2 handshake.
    /// Non-EAPOL frames get software header translation: strip 802.11+LLC/SNAP,
    /// build Ethernet header, and collect as data for ipd forwarding.
    /// Firmware HDR_TRANS is not reliably enabled on all STAs.
    fn collect_raw_data(&mut self, buf: &[u8], rxd: &event::RxdInfo) {
        let ofs = rxd.frame_ofs;
        if ofs >= buf.len() { return; }

        let fc0 = buf[ofs];
        let byte_cnt = rxd.byte_cnt as usize;

        // Check for EAPOL first
        if let Some((eapol_start, eapol_len)) = event::detect_eapol(buf, ofs, fc0, byte_cnt) {
            if self.eapol_count >= self.eapol_frames.len() { return; }
            let mut ef = RxEapolFrame::EMPTY;
            if ofs + 10 + 6 <= buf.len() {
                ef.src_mac.copy_from_slice(&buf[ofs + 10..ofs + 16]);
            }
            let copy_len = eapol_len.min(256);
            ef.body[..copy_len].copy_from_slice(&buf[eapol_start..eapol_start + copy_len]);
            ef.body_len = copy_len as u16;

            self.eapol_frames[self.eapol_count] = Some(ef);
            self.eapol_count += 1;
            return;
        }

        // Software header translation: 802.11 QoS Data → Ethernet
        // ToDS frame (STA→AP): Addr1=BSSID, Addr2=SA, Addr3=DA
        let subtype = (fc0 >> 4) & 0xF;
        let is_qos = (subtype & 0x8) != 0;
        let hdr_len: usize = if is_qos { 26 } else { 24 };
        let llc_ofs = ofs + hdr_len;

        // Need at least: 802.11 header + LLC/SNAP (8 bytes)
        if llc_ofs + 8 > byte_cnt || byte_cnt > buf.len() {
            return;
        }
        if ofs + 22 > buf.len() { return; }

        // Check for CCMP header: if encrypted, bytes at llc_ofs look like
        // [PN0, PN1, rsvd, ExtIV|KeyID, PN2-5] instead of AA AA 03 ...
        // CCMP ExtIV bit = byte[3] bit 5 (0x20)
        let has_ccmp = buf[llc_ofs + 3] & 0x20 != 0;
        let actual_llc_ofs = if has_ccmp { llc_ofs + 8 } else { llc_ofs };

        // Re-check bounds with CCMP offset
        if actual_llc_ofs + 8 > byte_cnt { return; }

        // Verify LLC/SNAP header (AA AA 03 00 00 00)
        let llc = &buf[actual_llc_ofs..actual_llc_ofs + 6];
        let has_snap = llc[0] == 0xAA && llc[1] == 0xAA && llc[2] == 0x03
            && llc[3] == 0x00 && llc[4] == 0x00 && llc[5] == 0x00;
        if !has_snap {
            return;
        }

        // Extract addresses and EtherType
        let sa = &buf[ofs + 10..ofs + 16]; // Addr2 = SA
        let da = &buf[ofs + 16..ofs + 22]; // Addr3 = DA
        let ethertype = &buf[actual_llc_ofs + 6..actual_llc_ofs + 8]; // after LLC/SNAP
        let payload_start = actual_llc_ofs + 8;
        // CCMP MIC trailer is 8 bytes — exclude from payload
        let mic_len = if has_ccmp { 8 } else { 0 };
        let payload_len = byte_cnt.saturating_sub(payload_start + mic_len);

        // Build Ethernet frame: DA(6) + SA(6) + EtherType(2) + payload
        let eth_len = 14 + payload_len;
        if self.data_count >= self.data_frames.len() { return; }
        if self.data_buf_pos + eth_len > self.data_buf.len() { return; }

        let out = self.data_buf_pos;
        self.data_buf[out..out + 6].copy_from_slice(da);
        self.data_buf[out + 6..out + 12].copy_from_slice(sa);
        self.data_buf[out + 12..out + 14].copy_from_slice(ethertype);
        if payload_len > 0 {
            self.data_buf[out + 14..out + 14 + payload_len]
                .copy_from_slice(&buf[payload_start..payload_start + payload_len]);
        }
        self.data_buf_pos += eth_len;

        self.data_frames[self.data_count] = Some(DataCollected { offset: out, len: eth_len });
        self.data_count += 1;
    }

    /// Record a dropped frame for diagnostics.
    fn record_drop(&mut self, _buf: &[u8], _rxd: &event::RxdInfo, _class: RxFrameClass) {
        self.dropped += 1;
    }

    /// Log collection statistics if any frames were received.
    fn log_stats(&self) {
        if self.total > 0 {
            udebug!("wifi2", "rx_batch";
                total = self.total as u32,
                mgmt = self.mgmt_count as u32,
                data = self.data_count as u32,
                eapol = self.eapol_count as u32,
                dropped = self.dropped as u32,
                fcs = self.fcs_errors as u32);
        }
    }
}

// ============================================================================
// Driver trait impl
// ============================================================================

/// Wrapper to implement the Driver trait.
struct Wifi2Driver(Wifi2);

impl Driver for Wifi2Driver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.init_hardware(ctx)?;
        self.0.init_radio(ctx)?;

        // Start poll timer (50ms) — fast enough for STA auth/assoc timeouts
        ctx.start_timer(TAG_WIFI_TIMER, 50_000_000)?;

        unotice!("wifi2", "ready"; state = self.0.state.name());
        Ok(())
    }

    fn command(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) -> Disposition {
        Disposition::Forward
    }

    fn handle_event(&mut self, tag: u32, _handle: userlib::syscall::Handle, ctx: &mut dyn BusCtx) {
        let is_timer = tag == TAG_WIFI_TIMER;
        let is_irq = tag == TAG_WIFI_IRQ;
        if !is_timer && !is_irq { return; }
        if self.0.state != DriverState::ApActive { return; }

        // Use raw pointer for dev to avoid borrow conflicts with &mut self.0
        let dev_ptr = match self.0.dev.as_ref() {
            Some(d) => d as *const Mt76Device,
            None => return,
        };
        // SAFETY: self.0.dev is Some (checked above) and not modified during handle_event.
        let dev = unsafe { &*dev_ptr };

        // ================================================================
        // IRQ handling: mask, read source, clear, then process
        // ================================================================
        if is_irq {
            if let Some(ref mut irq) = self.0.irq {
                dev.wr(MT_INT_MASK_CSR, 0);
                let src = dev.rr(MT_INT_SOURCE_CSR);
                if src != 0 {
                    dev.wr(MT_INT_SOURCE_CSR, src);
                }
                let _ = irq.ack();
            }
        }

        // Housekeeping tick: 10 sub-ticks (50ms each) = 500ms
        let mut is_slow_tick = false;
        if is_timer {
            self.0.subtick += 1;
            if self.0.subtick >= 10 {
                self.0.subtick = 0;
                self.0.tick = self.0.tick.wrapping_add(1);
                is_slow_tick = true;
            }
        }

        // ================================================================
        // Queue 0 (MCU WM) + Queue 1 (MCU WA): drain only
        // ================================================================
        for q in self.0.rx_queues[..2.min(self.0.rx_queue_count)].iter_mut() {
            q.drain(dev);
        }

        // ================================================================
        // Queue 2 (WA_MAIN) + Queue 3 (WA_TRI): drain (TX free notifications)
        // ================================================================
        if self.0.rx_queue_count > 2 { self.0.rx_queues[2].drain(dev); }
        if self.0.rx_queue_count > 3 { self.0.rx_queues[3].drain(dev); }

        // ================================================================
        // Queue 5 (BAND2): drain
        // ================================================================
        if self.0.rx_queue_count > 5 { self.0.rx_queues[5].drain(dev); }

        // ================================================================
        // Queue 4 (BAND0 data): classify and collect frames
        // ================================================================
        // Collect frames on stack — DMA buffers are freed during process(),
        // so we snapshot everything we need before the closure returns.
        let mut rx = RxCollector::new();

        if self.0.rx_queue_count > 4 {
            self.0.rx_queues[4].process(dev, |buf| {
                let rxd = event::classify_rxd(buf);
                rx.total += 1;
                if rxd.fcs_err {
                    rx.fcs_errors += 1;
                    // Log first FCS error details for diagnosis
                    if rx.fcs_errors == 1 {
                        let rxd0 = if buf.len() >= 4 { u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) } else { 0 };
                        let rxd3 = if buf.len() >= 16 { u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]) } else { 0 };
                        uwarn!("wifi2", "fcs_err_detail";
                            rxd0 = rxd0,
                            rxd1 = rxd.rxd1,
                            rxd3 = rxd3,
                            hdr_trans = rxd.hdr_trans as u32,
                            wlan_idx = rxd.wlan_idx as u32,
                            byte_cnt = rxd.byte_cnt as u32);
                    }
                    return;
                }

                let mut class = rxd.class;

                // Reclassify non-HDR_TRANS frames by reading FC byte
                // (NDATA bit is unreliable for unicast management)
                if !rxd.hdr_trans && matches!(class, RxFrameClass::Data | RxFrameClass::Mgmt) {
                    if rxd.frame_ofs + 2 <= buf.len() {
                        class = event::reclassify_by_fc(buf[rxd.frame_ofs]);
                    }
                }

                let frame_end = (rxd.byte_cnt as usize).min(buf.len());

                match class {
                    RxFrameClass::Mgmt if !rxd.hdr_trans =>
                        rx.collect_mgmt(buf, &rxd, frame_end),

                    RxFrameClass::Data if rxd.hdr_trans =>
                        rx.collect_eth_data(&buf[rxd.frame_ofs..frame_end]),

                    RxFrameClass::Data if !rxd.hdr_trans =>
                        rx.collect_raw_data(buf, &rxd),

                    _ => {
                        // Log first drop details for diagnosis
                        if rx.dropped == 0 {
                            let rxd0 = if buf.len() >= 4 { u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) } else { 0 };
                            let pkt_type = (rxd0 >> 27) & 0x1f;
                            uwarn!("wifi2", "drop_detail";
                                pkt_type = pkt_type,
                                hdr_trans = rxd.hdr_trans as u32,
                                wlan_idx = rxd.wlan_idx as u32,
                                byte_cnt = rxd.byte_cnt as u32);
                        }
                        rx.record_drop(buf, &rxd, class);
                    }
                }
            });
        }

        rx.log_stats();

        // ================================================================
        // Management frame dispatch through AP state machine
        // ================================================================
        for i in 0..rx.mgmt_count {
            let mf = match rx.mgmt_frames[i].take() {
                Some(m) => m,
                None => continue,
            };

            let subtype = wifi80211::types::parse_mgmt_subtype(mf.fc0);
            let fc_subtype = (mf.fc0 >> 4) & 0xF;
            udebug!("wifi2", "rx_mgmt"; subtype = fc_subtype as u32,
                fc0 = mf.fc0 as u32, body_len = mf.body_len as u32, rssi = mf.rssi as i32);
            let frame = RxMgmtFrame {
                subtype,
                addr2: mf.addr2,
                addr3: mf.addr3,
                rssi: mf.rssi,
                phy: mf.phy,
                body: mf.body,
                body_len: mf.body_len,
            };

            let mut tx_buf = [0u8; 512];
            let ap = match self.0.ap.as_mut() {
                Some(a) => a,
                None => continue,
            };
            let result = ap.handle_rx_mgmt(&frame, &mut tx_buf, self.0.tick);
            // Log AP actions for debugging
            for j2 in 0..result.count {
                match &result.actions[j2] {
                    Some(ApAction::TxFrame(_)) => { udebug!("wifi2", "ap_action"; action = 1u32); }
                    Some(ApAction::RegisterSta { .. }) => { udebug!("wifi2", "ap_action"; action = 2u32); }
                    Some(ApAction::RemoveSta { .. }) => { udebug!("wifi2", "ap_action"; action = 3u32); }
                    Some(ApAction::NotifyBaSession { .. }) => { udebug!("wifi2", "ap_action"; action = 4u32); }
                    Some(ApAction::StaPsChanged { .. }) => { udebug!("wifi2", "ap_action"; action = 5u32); }
                    _ => {}
                }
            }

            for j in 0..result.count {
                let action = match &result.actions[j] {
                    Some(a) => a,
                    None => continue,
                };
                match action {
                    ApAction::TxFrame(data) => {
                        let mut txd_buf = [0u8; 576];
                        let total = dma::wrap_mgmt_txd(&mut txd_buf, data);
                        if total > 0 {
                            let tok = self.0.next_tx_token();
                            if let Some(ref mut ring) = self.0.tx_band0 {
                                match dma::tx_enqueue_mgmt(ring, dev, &txd_buf[..total], tok) {
                                    Ok(()) => {
                                        udebug!("wifi2", "tx_mgmt_ok"; tok = tok as u32, len = total as u32);
                                    }
                                    Err(_) => {
                                        uerror!("wifi2", "tx_mgmt_full"; tok = tok as u32);
                                    }
                                }
                            }
                        }
                    }
                    ApAction::RegisterSta { mac, aid, ht_cap, ht_param, flags } => {
                        uinfo!("wifi2", "register_sta";
                            aid = *aid as u32,
                            mac4 = u32::from_be_bytes([mac[0], mac[1], mac[2], mac[3]]));

                        if let Some(wlan_idx) = self.0.alloc_wlan_idx() {
                            let mut sta_ok = false;
                            if let Some(ref mut ring) = self.0.wa_ring {
                                let seq = &mut self.0.seq;
                                let r1 = mcu::add_client_sta(
                                    dev, ring, 0, wlan_idx, 0, CONN_STATE_CONNECT,
                                    mac, *aid, true, *ht_cap, *ht_param, *flags,
                                    *seq, None, true,
                                );
                                *seq = seq.wrapping_add(1);

                                if r1.is_ok() {
                                    let r2 = mcu::sta_rate_ctrl(
                                        dev, ring, 0, wlan_idx, 0,
                                        self.0.channel, 0, *ht_cap, *ht_param, *flags, *seq,
                                    );
                                    *seq = seq.wrapping_add(1);
                                    if r2.is_err() {
                                        uwarn!("wifi2", "sta_rate_ctrl_failed"; wlan_idx = wlan_idx as u32);
                                    }

                                    // Enable RX header translation (decap offload)
                                    // Linux: sta_set_decap_offload → wtbl_update_hdr_trans
                                    let r3 = mcu::wtbl_update_hdr_trans(
                                        dev, ring, 0, wlan_idx, 0, *seq,
                                    );
                                    *seq = seq.wrapping_add(1);
                                    if r3.is_err() {
                                        uwarn!("wifi2", "hdr_trans_enable_failed"; wlan_idx = wlan_idx as u32);
                                    }

                                    sta_ok = true;
                                } else {
                                    uerror!("wifi2", "register_sta_mcu_failed"; wlan_idx = wlan_idx as u32);
                                    self.0.free_wlan_idx(wlan_idx);
                                }
                            }

                            if sta_ok {
                                if let Some(ref mut ap) = self.0.ap {
                                    ap.set_sta_wlan_idx(mac, wlan_idx);
                                }
                                uinfo!("wifi2", "sta_registered"; wlan_idx = wlan_idx as u32);

                                // Start WPA2 handshake if enabled
                                if let Some(ref mut wpa_ctx) = self.0.wpa_ctx {
                                    let mut anonce = [0u8; 32];
                                    let mut replay: u64 = 0;
                                    let mut m1_buf = [0u8; 256];
                                    let m1_len = wpa_ctx.build_m1(mac, &mut anonce, &mut replay, &mut m1_buf);
                                    if m1_len > 0 {
                                        let tok = self.0.next_tx_token();
                                        if let Some(ref mut ring) = self.0.tx_band0 {
                                            match dma::tx_enqueue_data(ring, dev, &m1_buf[..m1_len], wlan_idx, tok, true) {
                                                Ok(()) => {
                                                    uinfo!("wifi2", "m1_sent"; wlan_idx = wlan_idx as u32,
                                                        tok = tok as u32, len = m1_len as u32);
                                                }
                                                Err(_) => {
                                                    uerror!("wifi2", "m1_tx_full"; wlan_idx = wlan_idx as u32);
                                                }
                                            }
                                        }
                                        // Record handshake state on STA
                                        if let Some(ref mut ap) = self.0.ap {
                                            if let Some(sta) = ap.find_sta_mut(mac) {
                                                sta.hs_state = HandshakeState::WaitM2;
                                                sta.anonce = anonce;
                                                sta.replay_counter = replay;
                                                sta.hs_retries = 0;
                                            }
                                        }
                                    } else {
                                        uerror!("wifi2", "m1_build_failed"; wlan_idx = wlan_idx as u32);
                                    }
                                }
                            }
                        } else {
                            uerror!("wifi2", "wlan_idx_exhausted");
                        }
                    }
                    ApAction::RemoveSta { mac, aid: _, wlan_idx } => {
                        if let Some(ref mut ring) = self.0.wa_ring {
                            let seq = &mut self.0.seq;
                            if mcu::add_client_sta(
                                dev, ring, 0, *wlan_idx, 0, CONN_STATE_DISCONNECT,
                                mac, 0, false, 0, 0, 0,
                                *seq, None, true,
                            ).is_err() {
                                uerror!("wifi2", "remove_sta_failed"; wlan_idx = *wlan_idx as u32);
                            }
                            *seq = seq.wrapping_add(1);
                        }
                        self.0.free_wlan_idx(*wlan_idx);
                        // Clear handshake state
                        if let Some(ref mut ap) = self.0.ap {
                            if let Some(sta) = ap.find_sta_mut(mac) {
                                sta.hs_state = HandshakeState::None;
                            }
                        }
                    }
                    ApAction::NotifyBaSession { mac: _, tid, ssn, win_size, start } => {
                        // Find wlan_idx for this STA
                        let wlan_idx = self.0.ap.as_ref()
                            .and_then(|ap| ap.find_sta(&mf.addr2))
                            .map(|s| s.wlan_idx)
                            .unwrap_or(0);
                        if wlan_idx >= STA_WLAN_IDX_BASE {
                            if let Some(ref mut ring) = self.0.wa_ring {
                                let seq = &mut self.0.seq;
                                if mcu::sta_ba(
                                    dev, ring, 0, wlan_idx, 0,
                                    *tid, *ssn, *win_size, *start, *seq,
                                ).is_err() {
                                    uerror!("wifi2", "ba_session_failed"; wlan_idx = wlan_idx as u32, tid = *tid as u32);
                                }
                                *seq = seq.wrapping_add(1);
                            }
                        }
                    }
                    ApAction::StaPsChanged { .. } => {
                        // MT7996 firmware handles PS natively — no software action needed
                    }
                }
            }
        }

        // ================================================================
        // EAPOL dispatch (WPA2 4-way handshake)
        // ================================================================
        for i in 0..rx.eapol_count {
            let ef = match rx.eapol_frames[i].take() {
                Some(e) => e,
                None => continue,
            };
            let eapol_body = &ef.body[..ef.body_len as usize];
            udebug!("wifi2", "rx_eapol"; len = ef.body_len as u32,
                src = u32::from_be_bytes([ef.src_mac[0], ef.src_mac[1], ef.src_mac[2], ef.src_mac[3]]));

            // Find STA's handshake state
            let sta_info = self.0.ap.as_ref().and_then(|ap| {
                ap.find_sta(&ef.src_mac).map(|s| (s.wlan_idx, s.hs_state, s.anonce, s.replay_counter))
            });
            let (wlan_idx, hs_state, anonce, mut replay) = match sta_info {
                Some(info) => info,
                None => continue,
            };

            match hs_state {
                HandshakeState::WaitM2 => {
                    let wpa_ctx = match self.0.wpa_ctx.as_mut() {
                        Some(w) => w,
                        None => continue,
                    };
                    let mut ptk = [0u8; 48];
                    let mut snonce = [0u8; 32];
                    if wpa_ctx.process_m2(&ef.src_mac, &anonce, replay, eapol_body, &mut ptk, &mut snonce) {
                        // Build and send M3
                        replay += 1;
                        let kck: &[u8; 16] = ptk[..16].try_into().unwrap();
                        let kek: &[u8; 16] = ptk[16..32].try_into().unwrap();
                        let mut m3_buf = [0u8; 512];
                        let m3_len = wpa_ctx.build_m3(&ef.src_mac, &anonce, &mut replay, kck, kek, &mut m3_buf);
                        if m3_len > 0 {
                            let tok = self.0.next_tx_token();
                            if let Some(ref mut ring) = self.0.tx_band0 {
                                let _ = dma::tx_enqueue_data(ring, dev, &m3_buf[..m3_len], wlan_idx, tok, true);
                            }
                        }
                        // Update STA state
                        if let Some(ref mut ap) = self.0.ap {
                            if let Some(sta) = ap.find_sta_mut(&ef.src_mac) {
                                sta.hs_state = HandshakeState::WaitM4;
                                sta.ptk = ptk;
                                sta.snonce = snonce;
                                sta.replay_counter = replay;
                                sta.hs_retries = 0;
                            }
                        }
                        uinfo!("wifi2", "wpa_m2_ok"; wlan_idx = wlan_idx as u32);
                    }
                }
                HandshakeState::WaitM4 => {
                    // Get PTK from STA
                    let ptk = self.0.ap.as_ref()
                        .and_then(|ap| ap.find_sta(&ef.src_mac))
                        .map(|s| s.ptk);
                    let ptk = match ptk {
                        Some(p) => p,
                        None => continue,
                    };
                    let kck = &ptk[..16];
                    let wpa_ctx = match self.0.wpa_ctx.as_ref() {
                        Some(w) => w,
                        None => continue,
                    };
                    if wpa_ctx.process_m4(kck, eapol_body) {
                        // Install keys via MCU
                        // Linux order: PORT_SECURE first (sta_state AUTHORIZED),
                        // then set_key() — firmware needs STA authorized before
                        // installing keys.
                        let ptk_tk: [u8; 16] = ptk[32..48].try_into().unwrap_or([0; 16]);
                        let mut keys_ok = false;
                        if let Some(ref mut ring) = self.0.wa_ring {
                            let seq = &mut self.0.seq;

                            // Linux AP mode key install order:
                            // 0. Re-send BSS_INFO with cipher (first key install only)
                            // 1. install_key (GTK + PTK)
                            // 2. PORT_SECURE
                            // Source: mt7996/main.c:240-244 mt7996_set_hw_key()

                            // 0. Re-send full BSS_INFO with cipher=CCMP
                            // Linux: mt7996_set_hw_key() calls mt7996_mcu_add_bss_info()
                            // when link->mt76.cipher was 0 (first key install).
                            // Firmware needs BSS cipher set before WTBL keys take effect.
                            let _ = mcu::add_bss_info(
                                dev, ring, 0, HW_BSSID_0, 0, &self.0.mac_addr,
                                true, self.0.channel, CMD_CBW_20MHZ, 0,
                                MCU_CIPHER_AES_CCMP, *seq, None,
                            );
                            *seq = seq.wrapping_add(1);

                            // 1. Install GTK (group key) on BMC STA
                            // muar_idx=0x0e for BMC (broadcast/multicast)
                            // Linux: mt76_connac_mcu.c:285-286 alloc_sta_req
                            let r1 = mcu::install_key(
                                dev, ring, 0, MT7996_WTBL_RESERVED, 0x0e,
                                wpa_ctx.gtk_idx(), wpa_ctx.gtk(), *seq, None,
                            );
                            *seq = seq.wrapping_add(1);

                            // 2. Install PTK (pairwise key) on this STA
                            uinfo!("wifi2", "ptk_install";
                                wlan_idx = wlan_idx as u32,
                                tk0 = u32::from_le_bytes([ptk_tk[0], ptk_tk[1], ptk_tk[2], ptk_tk[3]]),
                                tk4 = u32::from_le_bytes([ptk_tk[4], ptk_tk[5], ptk_tk[6], ptk_tk[7]]),
                                tk8 = u32::from_le_bytes([ptk_tk[8], ptk_tk[9], ptk_tk[10], ptk_tk[11]]),
                                tk12 = u32::from_le_bytes([ptk_tk[12], ptk_tk[13], ptk_tk[14], ptk_tk[15]]));
                            let r2 = mcu::install_key(
                                dev, ring, 0, wlan_idx, 0,
                                0, &ptk_tk, *seq, None,
                            );
                            *seq = seq.wrapping_add(1);

                            // 3. Transition STA to PORT_SECURE (after keys installed)
                            let r3 = mcu::add_client_sta(
                                dev, ring, 0, wlan_idx, 0, CONN_STATE_PORT_SECURE,
                                &ef.src_mac, 0, false, 0, 0, 0,
                                *seq, None, true,
                            );
                            *seq = seq.wrapping_add(1);

                            if r1.is_err() || r2.is_err() || r3.is_err() {
                                uerror!("wifi2", "wpa_key_install_failed"; wlan_idx = wlan_idx as u32,
                                    gtk = r1.is_err() as u32, ptk = r2.is_err() as u32, secure = r3.is_err() as u32);
                            } else {
                                keys_ok = true;
                            }
                        }

                        if keys_ok {
                            if let Some(ref mut ap) = self.0.ap {
                                if let Some(sta) = ap.find_sta_mut(&ef.src_mac) {
                                    sta.hs_state = HandshakeState::Complete;
                                }
                            }
                            unotice!("wifi2", "wpa_complete"; wlan_idx = wlan_idx as u32);
                        }
                    }
                }
                _ => {} // Not in handshake — ignore
            }
        }

        // ================================================================
        // Data frame forwarding to ipd via DataPort
        // ================================================================
        if rx.data_count > 0 {
            if let Some(dp) = self.0.data_port {
                let mut forwarded = 0u32;
                let mut alloc_fail = 0u32;
                let mut port_fail = 0u32;
                for i in 0..rx.data_count {
                    let df = match rx.data_frames[i].take() {
                        Some(d) => d,
                        None => continue,
                    };
                    let frame_data = &rx.data_buf[df.offset..df.offset + df.len];

                    // Touch STA last_seen for aging
                    if df.len >= 12 {
                        let mut src_mac = [0u8; 6];
                        src_mac.copy_from_slice(&frame_data[6..12]);
                        if let Some(ref mut ap) = self.0.ap {
                            ap.touch_sta(&src_mac, self.0.tick);
                        }
                    }

                    // Allocate pool space and post CQE
                    let port = match ctx.block_port(dp) {
                        Some(p) => p,
                        None => { port_fail += 1; continue; }
                    };
                    let offset = match port.alloc(df.len as u32) {
                        Some(o) => o,
                        None => { alloc_fail += 1; continue; }
                    };
                    if let Some(dst) = port.pool_slice_mut(offset, df.len as u32) {
                        dst.copy_from_slice(frame_data);
                    }
                    let tag = self.0.net_rx_seq;
                    self.0.net_rx_seq = self.0.net_rx_seq.wrapping_add(1);
                    let cqe = IoCqe {
                        status: io_status::OK,
                        flags: 0,
                        tag,
                        transferred: df.len as u32,
                        result: offset,
                    };
                    port.complete(&cqe);
                    // Track pool offset for reclaim
                    let mask = TX_INFLIGHT_SIZE - 1;
                    self.0.cq_offsets[(self.0.cq_post_seq as usize) & mask] = offset;
                    self.0.cq_post_seq = self.0.cq_post_seq.wrapping_add(1);
                    forwarded += 1;

                    // Log first frame details for debugging
                    if forwarded == 1 && df.len >= 14 {
                        let ethertype = u16::from_be_bytes([frame_data[12], frame_data[13]]);
                        uinfo!("wifi2", "rx_data_fwd";
                            count = rx.data_count as u32,
                            len = df.len as u32,
                            ethertype = ethertype as u32,
                            dst0 = frame_data[0] as u32,
                            dst1 = frame_data[1] as u32,
                            offset = offset);
                    }
                }
                if alloc_fail > 0 || port_fail > 0 {
                    uwarn!("wifi2", "rx_fwd_drop"; alloc_fail = alloc_fail, port_fail = port_fail);
                }
                // Notify consumer (ipd) that completions are available
                if forwarded > 0 {
                    if let Some(port) = ctx.block_port(dp) {
                        port.notify();
                    }
                }
            } else {
                uinfo!("wifi2", "rx_data_no_port"; count = rx.data_count as u32);
            }
        }

        // ================================================================
        // TX sweep: reclaim completed TX descriptors
        // ================================================================
        if let Some(ref mut ring) = self.0.tx_band0 {
            let mut tokens = [0u16; 32];
            let swept = dma::tx_sweep(ring, dev, &mut tokens);
            if swept > 0 {
                udebug!("wifi2", "tx_sweep"; swept = swept as u32,
                    cpu = ring.cpu_idx, sweep = ring.sweep_idx);
                if let Some(dp) = self.0.data_port {
                    let mask = TX_INFLIGHT_SIZE - 1;
                    for k in 0..swept {
                        let tok = tokens[k] as usize;
                        let tag = self.0.tx_inflight_tags[tok & mask];
                        if tag != TX_TAG_FREE {
                            // Post TX completion CQE to ipd
                            if let Some(port) = ctx.block_port(dp) {
                                port.complete_ok(tag, 0);
                            }
                            self.0.tx_inflight_tags[tok & mask] = TX_TAG_FREE;
                        }
                    }
                    if let Some(port) = ctx.block_port(dp) {
                        port.notify();
                    }
                }
            }
        }

        // ================================================================
        // Pool reclaim: free CQ offsets that consumer has acknowledged
        // ================================================================
        if let Some(dp) = self.0.data_port {
            if let Some(port) = ctx.block_port(dp) {
                let new_head = port.cq_consumer_head();
                let mask = TX_INFLIGHT_SIZE - 1;
                let mut h = self.0.last_cq_head;
                while h != new_head {
                    let offset = self.0.cq_offsets[(h as usize) & mask];
                    if offset != 0 {
                        port.free(offset);
                    }
                    h = h.wrapping_add(1);
                }
                self.0.last_cq_head = new_head;
            }
        }

        // ================================================================
        // Slow tick (500ms): MIB stats, STA aging
        // ================================================================
        if is_slow_tick {
            mac::update_stats(dev, 0);

            // STA aging
            if let Some(ref mut ap) = self.0.ap {
                let mut evicted = [([0u8; 6], 0u16, 0u16); 4];
                let evict_count = ap.age_stas(self.0.tick, &mut evicted);
                for k in 0..evict_count {
                    let (mac, _aid, wlan_idx) = evicted[k];
                    if let Some(ref mut ring) = self.0.wa_ring {
                        let seq = &mut self.0.seq;
                        if mcu::add_client_sta(
                            dev, ring, 0, wlan_idx, 0, CONN_STATE_DISCONNECT,
                            &mac, 0, false, 0, 0, 0, *seq, None, true,
                        ).is_err() {
                            uerror!("wifi2", "age_disconnect_failed"; wlan_idx = wlan_idx as u32);
                        }
                        *seq = seq.wrapping_add(1);
                    }
                    self.0.free_wlan_idx(wlan_idx);
                    udebug!("wifi2", "sta_aged"; wlan_idx = wlan_idx as u32);
                }
            }
        }

        // ================================================================
        // Re-enable IRQ mask
        // ================================================================
        if is_irq {
            let irq_mask = MT_INT_MCU_CMD
                | MT_INT_RX_DONE_MCU
                | MT_INT_TX_DONE_MCU
                | MT_INT_RX_DONE_BAND0
                | MT_INT_RX_DONE_WA_MAIN
                | MT_INT_TX_DONE_BAND0;
            dev.wr(MT_INT_MASK_CSR, irq_mask);
        }
    }

    fn data_ready(&mut self, port_id: PortId, ctx: &mut dyn BusCtx) {
        if self.0.state != DriverState::ApActive { return; }
        let dev_ptr = match self.0.dev.as_ref() {
            Some(d) => d as *const Mt76Device,
            None => return,
        };
        // SAFETY: self.0.dev is Some (checked above) and not modified during data_ready.
        let dev = unsafe { &*dev_ptr };
        let dp = match self.0.data_port {
            Some(id) if id == port_id => id,
            _ => return,
        };
        udebug!("wifi2", "data_ready"; port = port_id.0 as u32);

        // ================================================================
        // Handle sidechannel queries (QUERY_INFO from ipd)
        // ================================================================
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
        for k in 0..qcount {
            if let Some(entry) = queries[k].take() {
                match entry.msg_type {
                    side_msg::QUERY_INFO => {
                        uinfo!("wifi2", "side_query_info"; tag = entry.tag);
                        let mut response = SideEntry {
                            msg_type: entry.msg_type,
                            flags: 0,
                            tag: entry.tag,
                            status: side_status::OK,
                            payload: [0; 24],
                        };
                        response.payload[0..6].copy_from_slice(&self.0.mac_addr);
                        response.payload[6] = 1; // link up
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

        // ================================================================
        // Process TX from ipd (SQ drain)
        // ================================================================
        for _ in 0..256 {
            let sqe = match ctx.block_port(dp).and_then(|p| p.recv_request()) {
                Some(s) => s,
                None => break,
            };
            if sqe.opcode != io_op::NET_SEND {
                if let Some(port) = ctx.block_port(dp) {
                    port.complete_error(sqe.tag, io_status::INVALID);
                }
                continue;
            }

            let frame_len = sqe.data_len as usize;
            if frame_len < libf::net::eth::HEADER_LEN || frame_len > MAX_FRAME_SIZE {
                if let Some(port) = ctx.block_port(dp) {
                    port.complete_error(sqe.tag, io_status::INVALID);
                }
                continue;
            }

            // Read frame from pool
            let mut frame_buf = [0u8; MAX_FRAME_SIZE];
            let got = ctx.block_port(dp).and_then(|port| {
                port.pool_slice(sqe.data_offset, frame_len as u32).map(|s| {
                    frame_buf[..frame_len].copy_from_slice(s);
                })
            });
            if got.is_none() {
                if let Some(port) = ctx.block_port(dp) {
                    port.complete_error(sqe.tag, io_status::INVALID);
                }
                continue;
            }

            let dst_mac: [u8; 6] = [frame_buf[0], frame_buf[1], frame_buf[2],
                                     frame_buf[3], frame_buf[4], frame_buf[5]];
            let is_multicast = (dst_mac[0] & 0x01) != 0;

            let wcid = if is_multicast {
                MT7996_WTBL_RESERVED
            } else {
                self.0.ap.as_ref()
                    .and_then(|ap| ap.find_sta(&dst_mac))
                    .map(|sta| sta.wlan_idx)
                    .unwrap_or(MT7996_WTBL_RESERVED)
            };

            // Check WPA2 handshake state for cipher decision
            let no_cipher = if self.0.wpa_ctx.is_some() && !is_multicast {
                self.0.ap.as_ref()
                    .and_then(|ap| ap.find_sta(&dst_mac))
                    .map(|sta| sta.hs_state != HandshakeState::Complete)
                    .unwrap_or(true)
            } else if self.0.wpa_ctx.is_some() {
                // Multicast: check if GTK is installed (any STA completed handshake)
                let any_complete = self.0.ap.as_ref()
                    .map(|ap| ap.iter_stas().any(|s| s.hs_state == HandshakeState::Complete))
                    .unwrap_or(false);
                !any_complete
            } else {
                true // No WPA2
            };

            let tok = self.0.next_tx_token();
            uinfo!("wifi2", "tx_from_ipd";
                len = frame_len as u32,
                wcid = wcid as u32,
                no_cipher = no_cipher as u32,
                dst0 = dst_mac[0] as u32,
                dst1 = dst_mac[1] as u32,
                mcast = is_multicast as u32);
            if let Some(ref mut ring) = self.0.tx_band0 {
                match dma::tx_enqueue_data(ring, dev, &frame_buf[..frame_len], wcid, tok, no_cipher) {
                    Ok(()) => {
                        let mask = TX_INFLIGHT_SIZE - 1;
                        self.0.tx_inflight_tags[(tok as usize) & mask] = sqe.tag;
                    }
                    Err(_) => {
                        if let Some(port) = ctx.block_port(dp) {
                            port.complete_error(sqe.tag, io_status::NOT_READY);
                        }
                    }
                }
            }
        }

        // ================================================================
        // TX sweep + CQ post for completed TX tokens
        // ================================================================
        if let Some(ref mut ring) = self.0.tx_band0 {
            let mut tokens = [0u16; 32];
            let swept = dma::tx_sweep(ring, dev, &mut tokens);
            if swept > 0 {
                let mask = TX_INFLIGHT_SIZE - 1;
                for k in 0..swept {
                    let tok = tokens[k] as usize;
                    let tag = self.0.tx_inflight_tags[tok & mask];
                    if tag != TX_TAG_FREE {
                        if let Some(port) = ctx.block_port(dp) {
                            port.complete_ok(tag, 0);
                        }
                        self.0.tx_inflight_tags[tok & mask] = TX_TAG_FREE;
                    }
                }
                if let Some(port) = ctx.block_port(dp) {
                    port.notify();
                }
            }
        }
    }

    fn config_keys(&self) -> &[ConfigKey] {
        config::CONFIG_KEYS
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        config::config_get(&self.0.state, self.0.channel, key, buf)
    }
}

// ============================================================================
// Entry point
// ============================================================================

#[unsafe(no_mangle)]
fn main() {
    driver_main(b"wifi2", Wifi2Driver(Wifi2::new()));
}
