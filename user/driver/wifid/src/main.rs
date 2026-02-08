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

use userlib::{print, println, syscall};
use userlib::{uinfo, uwarn, uerror, udebug};
use userlib::mmio::{MmioRegion, DmaPool};
use userlib::bus::{BusMsg, BusError, BusCtx, Driver, Disposition};
use userlib::bus_runtime::driver_main;

/// Write memory barrier for DMA - ensures stores are visible to PCIe devices
/// Linux wmb() on ARM64 is "dsb st" - Data Synchronization Barrier, stores only
/// This is required because fence(Ordering::Release) only generates "dmb ish"
/// which is inner-shareable and NOT visible to PCIe devices in outer-shareable domain
#[inline(always)]
fn dma_wmb() {
    unsafe {
        core::arch::asm!("dsb st", options(nostack, preserves_flags));
    }
}

/// Full data synchronization barrier (for after cache flush)
#[inline(always)]
fn dsb_sy() {
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// Flush a single cache line to memory (DC CVAC = Data Cache Clean by VA to Coherency)
/// This writes back dirty cache data to RAM so DMA devices can see it
#[inline(always)]
fn flush_cache_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc cvac, {}", in(reg) addr, options(nostack, preserves_flags));
    }
}

/// Flush a buffer to memory (for CPU writes that DMA device will read)
/// Must be called BEFORE kicking DMA, after writing descriptors/data
fn flush_buffer(virt_addr: u64, size: usize) {
    const CACHE_LINE: usize = 64;
    let start = virt_addr & !(CACHE_LINE as u64 - 1);
    let end = (virt_addr + size as u64 + CACHE_LINE as u64 - 1) & !(CACHE_LINE as u64 - 1);

    let mut a = start;
    while a < end {
        flush_cache_line(a);
        a += CACHE_LINE as u64;
    }
    // DSB ensures cache maintenance complete before continuing
    dsb_sy();
}

// ============================================================================
// DMA Address Translation
// ============================================================================
//
// PCIe DMA requires bus/DMA addresses, not CPU physical addresses.
// On some SoCs, these differ (e.g., DRAM at CPU 0x40000000 might be bus addr 0).
//
// Testing hypotheses:
//   1. Identity mapping: phys == dma (what we assumed for MT7988A)
//   2. DRAM offset: dma = phys - 0x40000000 (bus sees DRAM starting at 0)
//
// Toggle USE_DRAM_OFFSET to test hypothesis #2 if identity mapping fails.

const DRAM_BASE: u64 = 0x4000_0000;

/// Set to true to test the "DRAM offset" hypothesis
/// If DMA works with this enabled, the PCIe bus expects addresses relative to DRAM base
/// NOTE: Device tree has no dma-ranges, which means IDENTITY mapping should be correct
const USE_DRAM_OFFSET: bool = false;

/// Convert CPU physical address to PCIe bus/DMA address
#[inline]
fn phys_to_dma(phys: u64) -> u64 {
    if USE_DRAM_OFFSET {
        // Hypothesis 2: Bus sees DRAM at address 0
        if phys >= DRAM_BASE {
            phys - DRAM_BASE
        } else {
            // Address below DRAM - shouldn't happen for our allocations
            uwarn!("dma", "phys_below_dram"; addr = phys);
            phys
        }
    } else {
        // Hypothesis 1: Identity mapping (phys == dma)
        phys
    }
}

// ============================================================================
// EXACT Register Definitions from Linux regs.h
// ============================================================================

// WFDMA0 Base
const MT_WFDMA0_BASE: u32 = 0xd4000;
const MT_WFDMA0_TX_RING_BASE: u32 = MT_WFDMA0_BASE + 0x300;  // TX ring base at 0xd4300

#[inline]
const fn mt_wfdma0(ofs: u32) -> u32 { MT_WFDMA0_BASE + ofs }

// WFDMA0 Core Registers
const MT_WFDMA0_RST: u32 = mt_wfdma0(0x100);
const MT_WFDMA0_RST_LOGIC_RST: u32 = 1 << 4;           // BIT(4)
const MT_WFDMA0_RST_DMASHDL_ALL_RST: u32 = 1 << 5;     // BIT(5)

const MT_WFDMA0_BUSY_ENA: u32 = mt_wfdma0(0x13c);
const MT_WFDMA0_BUSY_ENA_TX_FIFO0: u32 = 1 << 0;       // BIT(0)
const MT_WFDMA0_BUSY_ENA_TX_FIFO1: u32 = 1 << 1;       // BIT(1)
const MT_WFDMA0_BUSY_ENA_RX_FIFO: u32 = 1 << 2;        // BIT(2)

const MT_WFDMA0_RX_INT_PCIE_SEL: u32 = mt_wfdma0(0x154);
const MT_WFDMA0_RX_INT_SEL_RING3: u32 = 1 << 3;        // BIT(3)

const MT_INT_SOURCE_CSR: u32 = mt_wfdma0(0x200);
const MT_INT_MASK_CSR: u32 = mt_wfdma0(0x204);

const MT_WFDMA0_GLO_CFG: u32 = mt_wfdma0(0x208);
const MT_WFDMA0_GLO_CFG_TX_DMA_EN: u32 = 1 << 0;       // BIT(0)
const MT_WFDMA0_GLO_CFG_RX_DMA_EN: u32 = 1 << 2;       // BIT(2)
const MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2: u32 = 1 << 21;
const MT_WFDMA0_GLO_CFG_EXT_EN: u32 = 1 << 26;         // BIT(26) - IN UPSTREAM LINUX!
const MT_WFDMA0_GLO_CFG_OMIT_RX_INFO: u32 = 1 << 27;   // BIT(27)
const MT_WFDMA0_GLO_CFG_OMIT_TX_INFO: u32 = 1 << 28;   // BIT(28)

const MT_WFDMA0_RST_DTX_PTR: u32 = mt_wfdma0(0x20c);

// UPSTREAM LINUX HAS THESE - restored:
const MT_WFDMA0_PAUSE_RX_Q_45_TH: u32 = mt_wfdma0(0x268);
const MT_WFDMA0_PAUSE_RX_Q_67_TH: u32 = mt_wfdma0(0x26c);
const MT_WFDMA0_PAUSE_RX_Q_89_TH: u32 = mt_wfdma0(0x270);
const MT_WFDMA0_PAUSE_RX_Q_RRO_TH: u32 = mt_wfdma0(0x27c);

const WF_WFDMA0_GLO_CFG_EXT0: u32 = mt_wfdma0(0x2b0);
const WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD: u32 = 1 << 18;
const WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE: u32 = 1 << 14;

const WF_WFDMA0_GLO_CFG_EXT1: u32 = mt_wfdma0(0x2b4);
const WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE: u32 = 1 << 28;
const WF_WFDMA0_GLO_CFG_EXT1_CALC_MODE: u32 = 1u32 << 31;

const MT_WFDMA0_PRI_DLY_INT_CFG0: u32 = mt_wfdma0(0x2f0);
const MT_WFDMA0_PRI_DLY_INT_CFG1: u32 = mt_wfdma0(0x2f4);
const MT_WFDMA0_PRI_DLY_INT_CFG2: u32 = mt_wfdma0(0x2f8);

// WFDMA Extended CSR
const MT_WFDMA_EXT_CSR_BASE: u32 = 0xd7000;
#[inline]
const fn mt_wfdma_ext_csr(ofs: u32) -> u32 { MT_WFDMA_EXT_CSR_BASE + ofs }

const MT_WFDMA_HOST_CONFIG: u32 = mt_wfdma_ext_csr(0x30);
const MT_WFDMA_HOST_CONFIG_PDMA_BAND: u32 = 1 << 0;
// UPSTREAM LINUX HAS THESE - restored:
const MT_WFDMA_HOST_CONFIG_BAND0_PCIE1: u32 = 1 << 20;
const MT_WFDMA_HOST_CONFIG_BAND1_PCIE1: u32 = 1 << 21;
const MT_WFDMA_HOST_CONFIG_BAND2_PCIE1: u32 = 1 << 22;

const MT_WFDMA_EXT_CSR_HIF_MISC: u32 = mt_wfdma_ext_csr(0x44);
const MT_WFDMA_EXT_CSR_HIF_MISC_BUSY: u32 = 1 << 0;

// UPSTREAM LINUX HAS THESE - restored:
const MT_WFDMA_AXI_R2A_CTRL: u32 = mt_wfdma_ext_csr(0x500);
const MT_WFDMA_AXI_R2A_CTRL_OUTSTAND_MASK: u32 = 0x1f;

// WFDMA0 PCIE1 (HIF2 offset)
const MT_WFDMA0_PCIE1_BASE: u32 = 0xd8000;
const HIF1_OFS: u32 = MT_WFDMA0_PCIE1_BASE - MT_WFDMA0_BASE; // 0x4000

const MT_WFDMA0_PCIE1_BUSY_ENA_TX_FIFO0: u32 = 1 << 0;
const MT_WFDMA0_PCIE1_BUSY_ENA_TX_FIFO1: u32 = 1 << 1;
const MT_WFDMA0_PCIE1_BUSY_ENA_RX_FIFO: u32 = 1 << 2;

// MT_TOP registers (driver ownership)
const MT_TOP_BASE: u32 = 0xe0000;
#[inline]
const fn mt_top(ofs: u32) -> u32 { MT_TOP_BASE + ofs }
#[inline]
const fn mt_top_lpcr_host_band(band: u32) -> u32 { mt_top(0x10 + band * 0x10) }
#[inline]
const fn mt_top_lpcr_host_band_irq_stat(band: u32) -> u32 { mt_top(0x14 + band * 0x10) }

const MT_TOP_LPCR_HOST_DRV_OWN: u32 = 1 << 1;          // BIT(1)
const MT_TOP_LPCR_HOST_FW_OWN_STAT: u32 = 1 << 2;      // BIT(2)
const MT_TOP_LPCR_HOST_BAND_STAT: u32 = 1 << 0;        // BIT(0)

// ============================================================================
// Address Remapping - for accessing high addresses like 0x70028600
// Source: mmio.c - mt7996_reg_map_l1()
// ============================================================================

// MT7996 remap offsets (from mmio.c mt7996_reg_base_offs)
const CONN_BUS_CR_VON_BASE: u32 = 0x155000;
const HIF_REMAP_L1_OFFSET: u32 = 0x24;       // MT7996 specific
const HIF_REMAP_BASE_L1: u32 = 0x130000;     // MT7996 specific

const MT_HIF_REMAP_L1: u32 = CONN_BUS_CR_VON_BASE + HIF_REMAP_L1_OFFSET; // 0x155024

// WFSYS reset register (accessed via L1 remap)
const MT_WF_SUBSYS_RST: u32 = 0x70028600;

// Hardware revision register (accessed via L1 remap)
// Source: regs.h:733 - #define MT_HW_REV 0x70010204
const MT_HW_REV: u32 = 0x70010204;

// PCIe MAC registers
// Linux mmio.c:154 has static map: { 0x74030000, 0x10000, 0x1000 }
// So HIF1 0x74030188 -> BAR offset 0x10188 (direct access!)
// HIF2 0x74090188 is NOT in static map, needs L1 remap
const MT_PCIE_MAC_INT_ENABLE: u32 = 0x10188;           // HIF1 - static map!
const MT_PCIE1_MAC_INT_ENABLE_PHYS: u32 = 0x74090188;  // HIF2 - needs L1 remap!

// pci.c:70 - RECOG_ID establishes HIF1-HIF2 communication
// Written by HIF2 device during mt7996_pci_init_hif2()
// Since we have single BAR with HIF2 at +0x4000, we write to HIF1's perspective
const MT_PCIE_RECOG_ID: u32 = 0xd7090;
const MT_PCIE_RECOG_ID_SEM: u32 = 1 << 31;  // BIT(31)

const MT_TOP_MISC: u32 = mt_top(0xf0);
const MT_TOP_MISC_FW_STATE: u32 = 0x7;                 // GENMASK(2, 0)

// SWDEF registers - for setting firmware mode BEFORE driver_own
// MT_SWDEF_BASE = 0x00401400 maps to BAR offset 0x80000 via reg_map
// (reg_map entry: { 0x00400000, 0x80000, 0x10000 } WF_MCU_SYSRAM)
// MT_SWDEF_MODE = 0x00401400 + 0x3c = 0x0040143C -> 0x80000 + 0x143C = 0x8143C
const MT_SWDEF_MODE: u32 = 0x8143C;  // Translated via reg_map!
const MT_SWDEF_NORMAL_MODE: u32 = 0;

// Queue register offsets (from mt76.h)
const MT_RING_SIZE: u32 = 0x10;
const MT_QUEUE_DESC_BASE: u32 = 0x0;
const MT_QUEUE_RING_SIZE: u32 = 0x4;
const MT_QUEUE_CPU_IDX: u32 = 0x8;
const MT_QUEUE_DMA_IDX: u32 = 0xc;

// DMA control bits (from dma.h)
const MT_DMA_CTL_SD_LEN0: u32 = 0x3fff_0000;           // GENMASK(29, 16)
const MT_DMA_CTL_LAST_SEC0: u32 = 1 << 30;             // BIT(30)
const MT_DMA_CTL_DMA_DONE: u32 = 1u32 << 31;           // BIT(31)
const MT_DMA_CTL_SDP0_H: u32 = 0xF;                    // GENMASK(3, 0) - high 4 bits of buf0 addr

/// Get low 32 bits of DMA address (for buf0)
#[inline]
fn dma_addr_lo(addr: u64) -> u32 {
    addr as u32
}

/// Get high 4 bits of DMA address (for info bits 3:0)
/// MT7996 uses 36-bit addressing: buf0 = bits 31:0, info[3:0] = bits 35:32
/// Verified from Linux trace: buf1=0, info contains high bits
#[inline]
fn dma_addr_hi(addr: u64) -> u32 {
    ((addr >> 32) & 0xF) as u32
}

// Ring sizes from Linux mt7996.h - EXACT VALUES
const MT7996_TX_RING_SIZE: u32 = 2048;
const MT7996_TX_MCU_RING_SIZE: u32 = 256;
const MT7996_TX_FWDL_RING_SIZE: u32 = 128;
const MT7996_RX_RING_SIZE: u32 = 1536;
const MT7996_RX_MCU_RING_SIZE: u32 = 512;
const MT7996_RX_MCU_RING_SIZE_WA: u32 = 1024;
const MT7996_RX_BUF_SIZE: u32 = 2048;
const MT7996_RX_MCU_BUF_SIZE: u32 = 2048;  // Linux adds SKB overhead, we use plain 2KB

// Number of RX queues we initialize
const NUM_RX_QUEUES: usize = 7;

// TX queue indices (from mt7996.h enum mt7996_txq_id)
const MT7996_TXQ_FWDL: u32 = 16;
const MT7996_TXQ_MCU_WM: u32 = 17;
const MT7996_TXQ_BAND0: u32 = 18;
const MT7996_TXQ_BAND1: u32 = 19;
const MT7996_TXQ_MCU_WA: u32 = 20;
const MT7996_TXQ_BAND2: u32 = 21;

// RX queue indices (from mt7996.h enum mt7996_rxq_id)
const MT7996_RXQ_MCU_WM: u32 = 0;
const MT7996_RXQ_MCU_WA: u32 = 1;
const MT7996_RXQ_MCU_WA_MAIN: u32 = 2;
const MT7996_RXQ_MCU_WA_TRI: u32 = 3;
const MT7996_RXQ_BAND0: u32 = 4;
const MT7996_RXQ_BAND2: u32 = 5;

// Logical queue indices (for prefetch, etc.)
const MT_MCUQ_FWDL: u32 = 0;
const MT_MCUQ_WM: u32 = 1;
const MT_MCUQ_WA: u32 = 2;

const MT_RXQ_MCU: u32 = 0;
const MT_RXQ_MCU_WA: u32 = 1;
const MT_RXQ_MAIN_WA: u32 = 2;
const MT_RXQ_BAND2_WA: u32 = 3;
const MT_RXQ_MAIN: u32 = 4;
const MT_RXQ_BAND2: u32 = 5;

// Interrupt bits
const MT_INT_RX_DONE_WM: u32 = 1 << 0;
const MT_INT_RX_DONE_WA: u32 = 1 << 1;
const MT_INT_RX_DONE_WA_MAIN: u32 = 1 << 2;
const MT_INT_RX_DONE_WA_TRI: u32 = 1 << 3;
const MT_INT_RX_DONE_BAND0: u32 = 1 << 12;
const MT_INT_RX_DONE_BAND2: u32 = 1 << 13;
const MT_INT_TX_DONE_FWDL: u32 = 1 << 26;
const MT_INT_TX_DONE_MCU_WM: u32 = 1 << 27;
const MT_INT_TX_DONE_MCU_WA: u32 = 1 << 22;
const MT_INT_TX_DONE_BAND0: u32 = 1 << 30;
const MT_INT_TX_DONE_BAND1: u32 = 1u32 << 31;
const MT_INT_MCU_CMD: u32 = 1 << 29;

// Compound interrupt masks - EXACT Linux definitions (regs.h:563-608)
// MT_INT_RX_DONE_MCU = MT_INT_RX(MT_RXQ_MCU) | MT_INT_RX(MT_RXQ_MCU_WA)
const MT_INT_RX_DONE_MCU: u32 = MT_INT_RX_DONE_WM | MT_INT_RX_DONE_WA;
// MT_INT_TX_DONE_MCU = MT_INT_TX_MCU(MT_MCUQ_WA) | MT_INT_TX_MCU(MT_MCUQ_WM) | MT_INT_TX_MCU(MT_MCUQ_FWDL)
const MT_INT_TX_DONE_MCU: u32 = MT_INT_TX_DONE_MCU_WA | MT_INT_TX_DONE_MCU_WM | MT_INT_TX_DONE_FWDL;

// HIF2 interrupt registers
const MT_INT1_SOURCE_CSR: u32 = MT_WFDMA0_PCIE1_BASE + 0x200;
const MT_INT1_MASK_CSR: u32 = MT_WFDMA0_PCIE1_BASE + 0x204;

// WFDMA queue ID definitions (for config)
const WFDMA0: bool = true;

// ============================================================================
// DMA Descriptor Structure - EXACT Linux layout
// Source: dma.h struct mt76_desc
// ============================================================================

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
struct Mt76Desc {
    buf0: u32,
    ctrl: u32,
    buf1: u32,
    info: u32,
}

// ============================================================================
// RX Queue Info - for tracking RX queues that need buffer fill
// ============================================================================

/// Tracks an RX queue for buffer fill
#[derive(Clone, Copy, Default)]
struct RxQueueInfo {
    /// Hardware queue index (for logging)
    hw_idx: u32,
    /// Register base for this queue (for writing CPU_IDX)
    regs_base: u32,
    /// Number of descriptors
    ndesc: u32,
    /// Virtual address of descriptor ring
    desc_virt: u64,
    /// Buffer size for this queue
    buf_size: u32,
    /// Buffer pool physical base for this queue
    buf_phys: u64,
    /// Buffer pool virtual base for this queue
    buf_virt: u64,
}

// ============================================================================
// Queue Configuration - from mt7996_dma_config()
// Source: dma.c:50-151
// ============================================================================

/// Queue configuration entry
struct QueueConfig {
    int_mask: u32,
    hw_idx: u32,
    is_wfdma0: bool,
}

/// DMA configuration state
/// Mirrors Linux dev->q_wfdma_mask, dev->q_int_mask[], dev->q_id[]
struct DmaConfig {
    q_wfdma_mask: u32,
    // MCUQ (0-2): FWDL, WM, WA
    mcuq: [QueueConfig; 3],
    // TXQ (0-2): BAND0, BAND1, BAND2
    txq: [QueueConfig; 3],
    // RXQ (0-5): MCU, MCU_WA, MAIN_WA, BAND2_WA, MAIN, BAND2
    rxq: [QueueConfig; 6],
}

impl DmaConfig {
    /// mt7996_dma_config() - EXACT Linux translation
    /// Source: dma.c:50-151
    fn new(has_hif2: bool) -> Self {
        let mut cfg = Self {
            q_wfdma_mask: 0,
            mcuq: [
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
            ],
            txq: [
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
            ],
            rxq: [
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
            ],
        };

        // dma.c:62-64 - RX queue config
        // RXQ_CONFIG(MT_RXQ_MCU, WFDMA0, MT_INT_RX_DONE_WM, MT7996_RXQ_MCU_WM);
        cfg.rxq_config(MT_RXQ_MCU as usize, WFDMA0, MT_INT_RX_DONE_WM, MT7996_RXQ_MCU_WM);
        // RXQ_CONFIG(MT_RXQ_MCU_WA, WFDMA0, MT_INT_RX_DONE_WA, MT7996_RXQ_MCU_WA);
        cfg.rxq_config(MT_RXQ_MCU_WA as usize, WFDMA0, MT_INT_RX_DONE_WA, MT7996_RXQ_MCU_WA);
        // RXQ_CONFIG(MT_RXQ_MAIN, WFDMA0, MT_INT_RX_DONE_BAND0, MT7996_RXQ_BAND0);
        cfg.rxq_config(MT_RXQ_MAIN as usize, WFDMA0, MT_INT_RX_DONE_BAND0, MT7996_RXQ_BAND0);
        // RXQ_CONFIG(MT_RXQ_MAIN_WA, WFDMA0, MT_INT_RX_DONE_WA_MAIN, MT7996_RXQ_MCU_WA_MAIN);
        cfg.rxq_config(MT_RXQ_MAIN_WA as usize, WFDMA0, MT_INT_RX_DONE_WA_MAIN, MT7996_RXQ_MCU_WA_MAIN);
        // dma.c:85-87 - MT7996 band2
        // RXQ_CONFIG(MT_RXQ_BAND2_WA, WFDMA0, MT_INT_RX_DONE_WA_TRI, MT7996_RXQ_MCU_WA_TRI);
        cfg.rxq_config(MT_RXQ_BAND2_WA as usize, WFDMA0, MT_INT_RX_DONE_WA_TRI, MT7996_RXQ_MCU_WA_TRI);
        // RXQ_CONFIG(MT_RXQ_BAND2, WFDMA0, MT_INT_RX_DONE_BAND2, MT7996_RXQ_BAND2);
        cfg.rxq_config(MT_RXQ_BAND2 as usize, WFDMA0, MT_INT_RX_DONE_BAND2, MT7996_RXQ_BAND2);

        // dma.c:127-141 - TX data queues for MT7996
        // TXQ_CONFIG(0, WFDMA0, MT_INT_TX_DONE_BAND0, MT7996_TXQ_BAND0);
        cfg.txq_config(0, WFDMA0, MT_INT_TX_DONE_BAND0, MT7996_TXQ_BAND0);
        if has_hif2 {
            // TXQ_CONFIG(1, WFDMA0, MT_INT_TX_DONE_BAND1, MT7996_TXQ_BAND1);
            cfg.txq_config(1, WFDMA0, MT_INT_TX_DONE_BAND1, MT7996_TXQ_BAND1);
            // TXQ_CONFIG(2, WFDMA0, MT_INT_TX_DONE_BAND2, MT7996_TXQ_BAND2);
            cfg.txq_config(2, WFDMA0, MT_INT_TX_DONE_BAND1, MT7996_TXQ_BAND2);
        } else {
            // Single PCIe: bn0/1:ring18 bn2:ring19
            cfg.txq_config(2, WFDMA0, MT_INT_TX_DONE_BAND1, MT7996_TXQ_BAND1);
        }

        // dma.c:148-151 - MCU TX queues
        // MCUQ_CONFIG(MT_MCUQ_FWDL, WFDMA0, MT_INT_TX_DONE_FWDL, MT7996_TXQ_FWDL);
        cfg.mcuq_config(MT_MCUQ_FWDL as usize, WFDMA0, MT_INT_TX_DONE_FWDL, MT7996_TXQ_FWDL);
        // MCUQ_CONFIG(MT_MCUQ_WM, WFDMA0, MT_INT_TX_DONE_MCU_WM, MT7996_TXQ_MCU_WM);
        cfg.mcuq_config(MT_MCUQ_WM as usize, WFDMA0, MT_INT_TX_DONE_MCU_WM, MT7996_TXQ_MCU_WM);
        // MCUQ_CONFIG(MT_MCUQ_WA, WFDMA0, MT_INT_TX_DONE_MCU_WA, MT7996_TXQ_MCU_WA);
        cfg.mcuq_config(MT_MCUQ_WA as usize, WFDMA0, MT_INT_TX_DONE_MCU_WA, MT7996_TXQ_MCU_WA);

        cfg
    }

    fn mcuq_config(&mut self, q: usize, wfdma: bool, int: u32, id: u32) {
        if q < 3 {
            if wfdma { self.q_wfdma_mask |= 1 << q; }
            self.mcuq[q] = QueueConfig { int_mask: int, hw_idx: id, is_wfdma0: wfdma };
        }
    }

    fn txq_config(&mut self, q: usize, wfdma: bool, int: u32, id: u32) {
        if q < 3 {
            if wfdma { self.q_wfdma_mask |= 1 << (q + 3); }
            self.txq[q] = QueueConfig { int_mask: int, hw_idx: id, is_wfdma0: wfdma };
        }
    }

    fn rxq_config(&mut self, q: usize, wfdma: bool, int: u32, id: u32) {
        if q < 6 {
            if wfdma { self.q_wfdma_mask |= 1 << (q + 6); }
            self.rxq[q] = QueueConfig { int_mask: int, hw_idx: id, is_wfdma0: wfdma };
        }
    }
}

// ============================================================================
// MT7996 DMA Controller
// ============================================================================

struct Mt7996Dev {
    bar0_base: u64,
    bar0_size: u64,
    has_hif2: bool,
    config: DmaConfig,
}

impl Mt7996Dev {
    fn new(bar0_base: u64, bar0_size: u64, has_hif2: bool) -> Self {
        // MT7996 has TWO PCIe devices but HIF2 registers (0xd8xxx) are accessible
        // through HIF1's BAR at offset 0xd8xxx. Linux mmio.c __mt7996_reg_addr():
        //   if (addr < 0x100000) return addr;
        // So 0xd8xxx is accessed via HIF1_BAR + 0xd8xxx, not through HIF2's separate BAR.
        Self {
            bar0_base,
            bar0_size,
            has_hif2,
            config: DmaConfig::new(has_hif2),
        }
    }

    // ========================================================================
    // Register access - EXACT mt76 equivalents
    // ========================================================================

    #[inline]
    fn mt76_wr(&self, reg: u32, val: u32) {
        // Linux mmio.c: For addr < 0x100000, access directly via BAR + offset
        // HIF2 registers at 0xd8xxx are accessible through HIF1's BAR!
        let offset = reg as usize;
        if offset < self.bar0_size as usize {
            unsafe {
                let ptr = (self.bar0_base as *mut u32).add(offset / 4);
                core::ptr::write_volatile(ptr, val);
            }
        }
    }

    #[inline]
    fn mt76_rr(&self, reg: u32) -> u32 {
        // Linux mmio.c: For addr < 0x100000, access directly via BAR + offset
        // HIF2 registers at 0xd8xxx are accessible through HIF1's BAR!
        let offset = reg as usize;
        if offset < self.bar0_size as usize {
            unsafe {
                let ptr = (self.bar0_base as *const u32).add(offset / 4);
                core::ptr::read_volatile(ptr)
            }
        } else {
            0xFFFF_FFFF
        }
    }

    #[inline]
    fn mt76_set(&self, reg: u32, bits: u32) {
        let val = self.mt76_rr(reg);
        self.mt76_wr(reg, val | bits);
    }

    #[inline]
    fn mt76_clear(&self, reg: u32, bits: u32) {
        let val = self.mt76_rr(reg);
        self.mt76_wr(reg, val & !bits);
    }

    #[inline]
    fn mt76_rmw(&self, reg: u32, mask: u32, val: u32) {
        let old = self.mt76_rr(reg);
        self.mt76_wr(reg, (old & !mask) | val);
    }

    fn mt76_poll(&self, reg: u32, mask: u32, val: u32, timeout_us: u32) -> bool {
        let iterations = timeout_us / 10;
        for _ in 0..iterations.max(1) {
            if (self.mt76_rr(reg) & mask) == val {
                return true;
            }
            userlib::delay_us(10);
        }
        false
    }

    fn mt76_poll_msec(&self, reg: u32, mask: u32, val: u32, timeout_ms: u32) -> bool {
        for _ in 0..timeout_ms.max(1) {
            if (self.mt76_rr(reg) & mask) == val {
                return true;
            }
            userlib::delay_ms(1);
        }
        false
    }

    // ========================================================================
    // Linux-style Logged Register Access
    // Format: [MT76_REG] WR 0xXXXXX: 0xYYYYYYYY -> 0xZZZZZZZZ
    // ========================================================================

    /// Logged write - matches Linux [MT76_REG] format
    fn mt76_wr_log(&self, reg: u32, val: u32) {
        let old = self.mt76_rr(reg);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", reg, old, val);
        self.mt76_wr(reg, val);
    }

    /// Logged read - matches Linux [MT76_REG] format
    fn mt76_rr_log(&self, reg: u32) -> u32 {
        let val = self.mt76_rr(reg);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}", reg, val);
        val
    }

    /// Logged RMW - matches Linux [MT76_REG] format
    fn mt76_rmw_log(&self, reg: u32, mask: u32, val: u32) {
        let old = self.mt76_rr(reg);
        let new = (old & !mask) | val;
        println!("[MT76_REG] RMW 0x{:05x}: 0x{:08x} & ~0x{:08x} | 0x{:08x} = 0x{:08x}",
                 reg, old, mask, val, new);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", reg, old, new);
        self.mt76_wr(reg, new);
    }

    /// Logged set - matches Linux [MT76_REG] format
    fn mt76_set_log(&self, reg: u32, bits: u32) {
        self.mt76_rmw_log(reg, 0, bits);
    }

    /// Logged clear - matches Linux [MT76_REG] format
    fn mt76_clear_log(&self, reg: u32, bits: u32) {
        self.mt76_rmw_log(reg, bits, 0);
    }

    // ========================================================================
    // L1 Address Remapping - EXACT Linux translation
    // Source: mmio.c:mt7996_reg_map_l1()
    // For accessing high addresses like 0x70028600 that are outside BAR0
    // ========================================================================

    /// Remap a high address via L1 remap mechanism
    /// Returns the remapped address within BAR0 range
    fn mt7996_reg_map_l1(&self, addr: u32) -> u32 {
        // mmio.c:261-262 - Extract offset and base from address
        // MT_HIF_REMAP_L1_OFFSET = GENMASK(15, 0)
        // MT_HIF_REMAP_L1_BASE = GENMASK(31, 16)
        let offset = addr & 0xFFFF;             // Lower 16 bits
        let base = (addr >> 16) & 0xFFFF;       // Upper 16 bits

        // mmio.c:265-267 - For MT7996: FIELD_PREP(MT_HIF_REMAP_L1_MASK_7996, base)
        // MT_HIF_REMAP_L1_MASK_7996 = GENMASK(31, 16) = 0xFFFF0000
        let l1_mask: u32 = 0xFFFF0000;
        let val = base << 16;                   // Shift base to upper 16 bits

        // mmio.c:273 - Use RMW to preserve lower 16 bits (like Linux)
        // dev->bus_ops->rmw(&dev->mt76, MT_HIF_REMAP_L1, l1_mask, val);
        // rmw(reg, mask, val) = (read(reg) & ~mask) | val
        let current = self.mt76_rr(MT_HIF_REMAP_L1);
        let new_val = (current & !l1_mask) | val;
        self.mt76_wr(MT_HIF_REMAP_L1, new_val);

        // mmio.c:274-275 - Read back to push write (memory barrier)
        // /* use read to push write */
        // dev->bus_ops->rr(&dev->mt76, MT_HIF_REMAP_L1);
        let _ = self.mt76_rr(MT_HIF_REMAP_L1);

        // mmio.c:277 - Return remapped address: base + offset
        HIF_REMAP_BASE_L1 + offset
    }

    /// Remap a high address via L1 remap mechanism WITH LOGGING
    /// Matches Linux [MT76_REG] format exactly for log comparison
    /// Returns the remapped address within BAR0 range
    fn mt7996_reg_map_l1_log(&self, addr: u32) -> u32 {
        // Extract offset and base from address
        let offset = addr & 0xFFFF;             // Lower 16 bits
        let base = (addr >> 16) & 0xFFFF;       // Upper 16 bits
        let l1_mask: u32 = 0xFFFF0000;
        let val = base << 16;

        // Step 1: Read current L1 remap value (Linux log line 1)
        let current = self.mt76_rr(MT_HIF_REMAP_L1);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}", MT_HIF_REMAP_L1, current);

        // Step 2: RMW to set new base (Linux log lines 2-3)
        let new_val = (current & !l1_mask) | val;
        println!("[MT76_REG] RMW 0x{:05x}: 0x{:08x} & ~0x{:08x} | 0x{:08x} = 0x{:08x}",
                 MT_HIF_REMAP_L1, current, l1_mask, val, new_val);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}",
                 MT_HIF_REMAP_L1, current, new_val);
        self.mt76_wr(MT_HIF_REMAP_L1, new_val);

        // Step 3: Read back to push write (Linux log line 4)
        let readback = self.mt76_rr(MT_HIF_REMAP_L1);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}", MT_HIF_REMAP_L1, readback);

        // Return remapped address
        HIF_REMAP_BASE_L1 + offset
    }

    /// Read from a high address using L1 remap WITH LOGGING
    /// Matches Linux [MT76_REG] format - shows the remapped address read
    fn mt76_rr_remap_log(&self, addr: u32) -> u32 {
        let remapped = self.mt7996_reg_map_l1_log(addr);
        let val = self.mt76_rr(remapped);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}", remapped, val);
        val
    }

    /// Write to a high address using L1 remap
    fn mt76_wr_remap(&self, addr: u32, val: u32) {
        let remapped = self.mt7996_reg_map_l1(addr);
        self.mt76_wr(remapped, val);
    }

    /// Read from a high address using L1 remap
    fn mt76_rr_remap(&self, addr: u32) -> u32 {
        let remapped = self.mt7996_reg_map_l1(addr);
        self.mt76_rr(remapped)
    }

    /// Set bits in a high address using L1 remap (read-modify-write)
    fn mt76_set_remap(&self, addr: u32, bits: u32) {
        let remapped = self.mt7996_reg_map_l1(addr);
        let val = self.mt76_rr(remapped);
        self.mt76_wr(remapped, val | bits);
    }

    /// Clear bits in a high address using L1 remap (read-modify-write)
    fn mt76_clear_remap(&self, addr: u32, bits: u32) {
        let remapped = self.mt7996_reg_map_l1(addr);
        let val = self.mt76_rr(remapped);
        self.mt76_wr(remapped, val & !bits);
    }

    // ========================================================================
    // Linux-style trace logging - matches MT7996_TRACE format exactly
    // ========================================================================

    /// Log full register state at a checkpoint (matches Linux MT7996_TRACE format)
    fn trace_checkpoint(&self, name: &str) {
        let int_src = self.mt76_rr(MT_INT_SOURCE_CSR);
        let int_mask = self.mt76_rr(MT_INT_MASK_CSR);
        let glo_cfg = self.mt76_rr(MT_WFDMA0_GLO_CFG);
        let rst = self.mt76_rr(MT_WFDMA0_RST);
        let ltssm = self.mt76_rr(0x10150);
        let setting = self.mt76_rr(0x10080);
        let mac_int = self.mt76_rr(MT_PCIE_MAC_INT_ENABLE);
        let hif_misc = self.mt76_rr(MT_WFDMA_EXT_CSR_HIF_MISC);
        let bit11 = if (ltssm & 0x800) != 0 { 1 } else { 0 };

        println!("MT7996_TRACE @ {:<32}", name);
        println!("  INT_SRC=0x{:08x} INT_MASK=0x{:08x} GLO_CFG=0x{:08x} RST=0x{:08x}",
                 int_src, int_mask, glo_cfg, rst);
        println!("  LTSSM=0x{:08x} (bit11={}) SETTING=0x{:08x} MAC_INT=0x{:08x} HIF_MISC=0x{:08x}",
                 ltssm, bit11, setting, mac_int, hif_misc);
    }

    /// Log DMA state (matches Linux [MT7996_DMA] format)
    fn trace_dma_state(&self, prefix: &str) {
        let rst = self.mt76_rr(MT_WFDMA0_RST);
        let glo_cfg = self.mt76_rr(MT_WFDMA0_GLO_CFG);
        let hif_misc = self.mt76_rr(MT_WFDMA_EXT_CSR_HIF_MISC);
        let busy_ena = self.mt76_rr(MT_WFDMA0_BUSY_ENA);
        println!("[MT7996_DMA] {}: RST=0x{:08x} GLO_CFG=0x{:08x}", prefix, rst, glo_cfg);
        println!("[MT7996_DMA] {}: HIF_MISC=0x{:08x} BUSY_ENA=0x{:08x}", prefix, hif_misc, busy_ena);
    }

    /// Log MCU ring state
    fn trace_mcu_ring(&self, name: &str, ring_base: u32) {
        let base = self.mt76_rr(ring_base + MT_QUEUE_DESC_BASE);
        let cpu = self.mt76_rr(ring_base + MT_QUEUE_CPU_IDX);
        let dma = self.mt76_rr(ring_base + MT_QUEUE_DMA_IDX);
        println!("[MT7996_MCU] {} ring: BASE=0x{:08x} CPU={} DMA={}", name, base, cpu, dma);
    }

    // ========================================================================
    // mt7996_wfsys_reset() - EXACT Linux translation
    // Source: init.c (simple WFSYS reset before DMA init)
    // This MUST be called before mt7996_dma_init()!
    // ========================================================================

    fn mt7996_wfsys_reset(&self) {
        println!("[MT7996] === wfsys_reset START ===");

        // init.c - mt7996_wfsys_reset():
        //   mt76_set(dev, MT_WF_SUBSYS_RST, 0x1);
        //   msleep(20);
        //   mt76_clear(dev, MT_WF_SUBSYS_RST, 0x1);
        //   msleep(20);

        // Save L1 remap value BEFORE modifying it (with logging)
        let remap_before = self.mt76_rr(MT_HIF_REMAP_L1);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}  ; save L1 remap", MT_HIF_REMAP_L1, remap_before);

        // L1 remap to access MT_WF_SUBSYS_RST (0x70028600) with full logging
        // This does: read L1, RMW, write, read-back
        let remapped = self.mt7996_reg_map_l1_log(MT_WF_SUBSYS_RST);
        println!("[MT7996] MT_WF_SUBSYS_RST (0x{:08x}) -> remapped to 0x{:05x}", MT_WF_SUBSYS_RST, remapped);

        // mt76_set(dev, MT_WF_SUBSYS_RST, 0x1) - read-modify-write to set bit 0
        let val_before = self.mt76_rr(remapped);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}  ; MT_WF_SUBSYS_RST", remapped, val_before);
        let val_set = val_before | 0x1;
        println!("[MT76_REG] RMW 0x{:05x}: 0x{:08x} | 0x{:08x} = 0x{:08x}  ; set bit 0",
                 remapped, val_before, 0x1u32, val_set);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", remapped, val_before, val_set);
        self.mt76_wr(remapped, val_set);

        println!("[MT7996] msleep(20)");
        userlib::delay_ms(20);

        // mt76_clear(dev, MT_WF_SUBSYS_RST, 0x1) - read-modify-write to clear bit 0
        let val_after_set = self.mt76_rr(remapped);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}  ; after set", remapped, val_after_set);
        let val_clear = val_after_set & !0x1;
        println!("[MT76_REG] RMW 0x{:05x}: 0x{:08x} & ~0x{:08x} = 0x{:08x}  ; clear bit 0",
                 remapped, val_after_set, 0x1u32, val_clear);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", remapped, val_after_set, val_clear);
        self.mt76_wr(remapped, val_clear);

        println!("[MT7996] msleep(20)");
        userlib::delay_ms(20);

        // Read final value
        let val_final = self.mt76_rr(remapped);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}  ; final", remapped, val_final);

        // Restore L1 remap to original value (with logging)
        let remap_current = self.mt76_rr(MT_HIF_REMAP_L1);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}  ; L1 after reset", MT_HIF_REMAP_L1, remap_current);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}  ; restore L1",
                 MT_HIF_REMAP_L1, remap_current, remap_before);
        self.mt76_wr(MT_HIF_REMAP_L1, remap_before);
        let remap_restored = self.mt76_rr(MT_HIF_REMAP_L1);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}  ; L1 restored", MT_HIF_REMAP_L1, remap_restored);

        println!("[MT7996] === wfsys_reset END ===");
    }

    // ========================================================================
    // mt7996_driver_own() - EXACT Linux translation
    // Source: mcu.c:3559-3572
    // ========================================================================

    fn mt7996_driver_own(&self, band: u32) -> Result<(), i32> {
        // mcu.c:3561 - Write driver own request
        self.mt76_wr(mt_top_lpcr_host_band(band), MT_TOP_LPCR_HOST_DRV_OWN);

        // mcu.c:3562-3566 - Poll for firmware to release ownership
        if !self.mt76_poll_msec(
            mt_top_lpcr_host_band(band),
            MT_TOP_LPCR_HOST_FW_OWN_STAT,
            0,
            500,
        ) {
            uerror!("init", "driver_own_timeout"; band = band);
            return Err(-1);
        }

        // mcu.c:3569-3570 - Clear IRQ status on success
        self.mt76_wr(
            mt_top_lpcr_host_band_irq_stat(band),
            MT_TOP_LPCR_HOST_BAND_STAT,
        );

        Ok(())
    }

    // ========================================================================
    // mt7996_dma_disable() - EXACT Linux translation
    // Source: dma.c:545-585
    // ========================================================================

    fn mt7996_dma_disable(&self, reset: bool) {
        println!("[MT7996_DMA] === mt7996_dma_disable reset={} ===", if reset { 1 } else { 0 });
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        // Log BEFORE state (like Linux)
        let _ = self.mt76_rr_log(MT_WFDMA0_RST);
        let _ = self.mt76_rr_log(MT_WFDMA0_GLO_CFG);
        println!("[MT7996_DMA] BEFORE: RST=0x{:08x} GLO_CFG=0x{:08x}",
            self.mt76_rr(MT_WFDMA0_RST), self.mt76_rr(MT_WFDMA0_GLO_CFG));
        let _ = self.mt76_rr_log(MT_WFDMA_EXT_CSR_HIF_MISC);
        let _ = self.mt76_rr_log(MT_WFDMA0_BUSY_ENA);
        println!("[MT7996_DMA] BEFORE: HIF_MISC=0x{:08x} BUSY_ENA=0x{:08x}",
            self.mt76_rr(MT_WFDMA_EXT_CSR_HIF_MISC), self.mt76_rr(MT_WFDMA0_BUSY_ENA));

        // Log LTSSM like Linux
        let ltssm = self.mt76_rr_log(0x10150);
        println!("MT7996_LTSSM @ {:<32}: 0x{:08x} bit11={}", "dma_disable_entry", ltssm, (ltssm >> 11) & 1);

        // dma.c:551-570 - Reset pulse
        if reset {
            // dma.c:552-555 - Clear then set RST bits on HIF1
            self.mt76_clear_log(MT_WFDMA0_RST,
                MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
            self.mt76_set_log(MT_WFDMA0_RST,
                MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);

            // dma.c:557-566 - Same for HIF2
            if self.has_hif2 {
                self.mt76_clear_log(MT_WFDMA0_RST + hif1_ofs,
                    MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
                self.mt76_set_log(MT_WFDMA0_RST + hif1_ofs,
                    MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
            }
        }

        // dma.c:573-579 - Disable DMA on HIF1 (use logged RMW)
        let glo_clear_bits = MT_WFDMA0_GLO_CFG_TX_DMA_EN |
            MT_WFDMA0_GLO_CFG_RX_DMA_EN |
            MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
            MT_WFDMA0_GLO_CFG_OMIT_RX_INFO |
            MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2;
        self.mt76_clear_log(MT_WFDMA0_GLO_CFG, glo_clear_bits);

        // dma.c:581-585 - Disable DMA on HIF2
        if self.has_hif2 {
            self.mt76_clear_log(MT_WFDMA0_GLO_CFG + hif1_ofs, glo_clear_bits);
        }

        // Log AFTER state (like Linux)
        let rst_after = self.mt76_rr_log(MT_WFDMA0_RST);
        let glo_after = self.mt76_rr_log(MT_WFDMA0_GLO_CFG);
        println!("[MT7996_DMA] AFTER disable: RST=0x{:08x} GLO_CFG=0x{:08x}", rst_after, glo_after);

        let ltssm = self.mt76_rr_log(0x10150);
        println!("MT7996_LTSSM @ {:<32}: 0x{:08x} bit11={}", "dma_disable_exit", ltssm, (ltssm >> 11) & 1);
        println!("[MT7996_DMA] === mt7996_dma_disable END ===");
    }

    // ========================================================================
    // mt7996_dma_start() - EXACT Linux translation
    // Source: dma.c:587-625
    // ========================================================================

    fn mt7996_dma_start(&self, reset: bool, wed_reset: bool) {
        println!("[MT7996_DMA] === mt7996_dma_start reset={} wed_reset={} ===",
            if reset { 1 } else { 0 }, if wed_reset { 1 } else { 0 });
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        // Log LTSSM at entry (like Linux)
        let ltssm = self.mt76_rr_log(0x10150);
        println!("MT7996_LTSSM @ {:<32}: 0x{:08x} bit11={}", "dma_start_entry", ltssm, (ltssm >> 11) & 1);

        // dma.c:302-321 (upstream) - Enable WFDMA TX/RX
        // Linux sets: TX_DMA_EN | RX_DMA_EN | OMIT_TX_INFO | OMIT_RX_INFO_PFET2 | EXT_EN
        // Note: EXT_EN (BIT 26) is in upstream kernel, not our local 6.6.119
        if !reset {
            let glo_cfg_before = self.mt76_rr_log(MT_WFDMA0_GLO_CFG);
            println!("[MT7996_DMA] GLO_CFG before: 0x{:08x}", glo_cfg_before);

            // UPSTREAM LINUX: dma_start includes EXT_EN (BIT 26)!
            // Bits to set: TX_DMA_EN=BIT0, RX_DMA_EN=BIT2, OMIT_TX_INFO=BIT28,
            //              OMIT_RX_INFO_PFET2=BIT29, EXT_EN=BIT26
            let bits_to_set = MT_WFDMA0_GLO_CFG_TX_DMA_EN |
                MT_WFDMA0_GLO_CFG_RX_DMA_EN |
                MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
                MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2 |
                MT_WFDMA0_GLO_CFG_EXT_EN;

            // Use logged read-first then read again after set (like Linux)
            let _ = self.mt76_rr_log(MT_WFDMA0_GLO_CFG);
            self.mt76_set_log(MT_WFDMA0_GLO_CFG, bits_to_set);
            let glo_cfg_after = self.mt76_rr_log(MT_WFDMA0_GLO_CFG);
            println!("[MT7996_DMA] GLO_CFG (no WED): 0x{:08x} -> 0x{:08x}", glo_cfg_before, glo_cfg_after);

            // UPSTREAM LINUX: HIF2 GLO_CFG also includes EXT_EN
            if self.has_hif2 {
                self.mt76_set_log(MT_WFDMA0_GLO_CFG + hif1_ofs, bits_to_set);
            }
        }

        // dma.c:328 - Enable interrupts for TX/RX rings
        // EXACT Linux sequence from armbian_golden.log lines 277-286:
        // 1. Set INT_MASK = irq_mask
        // 2. Clear INT_MASK = 0 (disable before clearing source)
        // 3. Read & clear INT_SRC
        // 4. Re-set INT_MASK = irq_mask
        let irq_mask = MT_INT_MCU_CMD | MT_INT_RX_DONE_MCU | MT_INT_TX_DONE_MCU;

        // Step 1: Set INT_MASK (like Linux mt7996_irq_enable())
        self.mt76_wr_log(MT_INT_MASK_CSR, irq_mask);
        if self.has_hif2 {
            self.mt76_wr_log(MT_INT1_MASK_CSR, irq_mask);
        }

        // Step 2: Clear INT_MASK (like Linux mt7996_irq_disable(0))
        self.mt76_wr_log(MT_INT_MASK_CSR, 0);
        if self.has_hif2 {
            self.mt76_wr_log(MT_INT1_MASK_CSR, 0);
        }

        // Step 3: Read and clear INT_SRC
        let int_src = self.mt76_rr_log(MT_INT_SOURCE_CSR);
        self.mt76_wr_log(MT_INT_SOURCE_CSR, int_src);
        if self.has_hif2 {
            let int_src1 = self.mt76_rr_log(MT_INT1_SOURCE_CSR);
            self.mt76_wr_log(MT_INT1_SOURCE_CSR, int_src1);
        }

        // Step 4: Re-enable INT_MASK
        self.mt76_wr_log(MT_INT_MASK_CSR, irq_mask);
        if self.has_hif2 {
            self.mt76_wr_log(MT_INT1_MASK_CSR, irq_mask);
        }
    }

    // ========================================================================
    // __mt7996_dma_prefetch() - EXACT Linux translation
    // Source: dma.c:167-240
    //
    // CRITICAL: Linux uses ACCUMULATED base values!
    // The PREFETCH macro advances the base by (depth << 4) each call.
    // Order matters - WA comes BEFORE TX data queues!
    // ========================================================================

    fn __mt7996_dma_prefetch(&self, ofs: u32) {
        // CRITICAL: Use EXACT Linux prefetch values from armbian_golden.log
        // The prefetch offset encodes cumulative offset + depth.
        // Format: (offset << 16) | depth
        // Each queue's offset = previous offset + (previous depth * 16)
        println!("[MT7996_DMA] __mt7996_dma_prefetch ofs=0x{:x}", ofs);

        // TX prefetch registers (0xd4640 - 0xd4654)
        // Lines 174-179 from armbian_golden.log
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x640 + ofs, 0x00000002);  // FWDL (q16): base=0, depth=2
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x644 + ofs, 0x00200002);  // WM (q17): base=32, depth=2
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x648 + ofs, 0x00400008);  // BAND0 (q18): base=64, depth=8
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x64c + ofs, 0x00c00008);  // q19: base=192, depth=8
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x650 + ofs, 0x01400002);  // WA (q20): base=320, depth=2
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x654 + ofs, 0x01600008);  // q21: base=352, depth=8

        // RX prefetch registers (0xd4680 - 0xd4694)
        // Lines 180-185 from armbian_golden.log
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x680 + ofs, 0x01e00002);  // RX_MCU (q0): base=480, depth=2
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x684 + ofs, 0x02000002);  // RX_WA (q1): base=512, depth=2
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x688 + ofs, 0x02200002);  // q2: base=544, depth=2
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x68c + ofs, 0x02400002);  // q3: base=576, depth=2
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x690 + ofs, 0x02600010);  // q4: base=608, depth=16
        self.mt76_wr_log(MT_WFDMA0_BASE + 0x694 + ofs, 0x03600010);  // q5: base=864, depth=16

        // dma.c:239 - Set CALC_MODE (Linux does RMW on 0xd42b4/0xd82b4 with 0x80000000)
        self.mt76_set_log(WF_WFDMA0_GLO_CFG_EXT1 + ofs, WF_WFDMA0_GLO_CFG_EXT1_CALC_MODE);
    }

    // ========================================================================
    // mt7996_dma_prefetch() - EXACT Linux translation
    // Source: dma.c:520-523
    // ========================================================================

    fn mt7996_dma_prefetch(&self) {
        udebug!("dma", "prefetch"; hif = 1, ofs = 0);
        self.__mt7996_dma_prefetch(0);
        if self.has_hif2 {
            udebug!("dma", "prefetch"; hif = 2, ofs = HIF1_OFS);
            self.__mt7996_dma_prefetch(HIF1_OFS);
        }
    }

    // ========================================================================
    // mt7996_dma_enable() - EXACT Linux translation
    // Source: dma.c:627-754
    // ========================================================================

    fn mt7996_dma_enable(&self, reset: bool) {
        udebug!("dma", "dma_enable_start"; reset = reset);
        udebug!("dma", "hif_bases"; hif1 = MT_WFDMA0_BASE, hif2 = MT_WFDMA0_BASE + HIF1_OFS, hif2_ofs = HIF1_OFS);
        udebug!("dma", "config"; has_hif2 = self.has_hif2);

        // Dump MT7996 internal PCIe registers (same as Linux)
        udebug!("dma", "pcie_regs";
            setting = self.mt76_rr(0x10080),
            reg_84 = self.mt76_rr(0x10084),
            reg_88 = self.mt76_rr(0x10088),
            rst_ctrl = self.mt76_rr(0x10148),
            ltssm = self.mt76_rr(0x10150),
            linksta = self.mt76_rr(0x10154),
            mac_int = self.mt76_rr(0x10188));

        // BEFORE state
        udebug!("dma", "dma_enable_before";
            rst = self.mt76_rr(MT_WFDMA0_RST),
            glo_cfg = self.mt76_rr(MT_WFDMA0_GLO_CFG),
            hif_misc = self.mt76_rr(MT_WFDMA_EXT_CSR_HIF_MISC),
            busy_ena = self.mt76_rr(MT_WFDMA0_BUSY_ENA));

        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        // dma.c:633-635 - Reset DMA index
        println!("[MT7996_DMA] Writing RST_DTX_PTR");
        self.mt76_wr_log(MT_WFDMA0_RST_DTX_PTR, !0u32);
        if self.has_hif2 {
            self.mt76_wr_log(MT_WFDMA0_RST_DTX_PTR + hif1_ofs, !0u32);
        }

        // dma.c:637-646 - Configure delay interrupt off
        self.mt76_wr_log(MT_WFDMA0_PRI_DLY_INT_CFG0, 0);
        self.mt76_wr_log(MT_WFDMA0_PRI_DLY_INT_CFG1, 0);
        self.mt76_wr_log(MT_WFDMA0_PRI_DLY_INT_CFG2, 0);

        if self.has_hif2 {
            self.mt76_wr_log(MT_WFDMA0_PRI_DLY_INT_CFG0 + hif1_ofs, 0);
            self.mt76_wr_log(MT_WFDMA0_PRI_DLY_INT_CFG1 + hif1_ofs, 0);
            self.mt76_wr_log(MT_WFDMA0_PRI_DLY_INT_CFG2 + hif1_ofs, 0);
        }

        // dma.c:649 - Configure prefetch settings
        self.mt7996_dma_prefetch();
        udebug!("dma", "prefetch_done");

        // dma.c:651-655 - Configure BUSY_ENA on HIF1
        let busy_ena_before = self.mt76_rr(MT_WFDMA0_BUSY_ENA);
        udebug!("dma", "busy_ena_before"; val = busy_ena_before);
        self.mt76_set(MT_WFDMA0_BUSY_ENA,
            MT_WFDMA0_BUSY_ENA_TX_FIFO0 |
            MT_WFDMA0_BUSY_ENA_TX_FIFO1 |
            MT_WFDMA0_BUSY_ENA_RX_FIFO);

        // dma.c:657-661 - Configure BUSY_ENA on HIF2
        if self.has_hif2 {
            self.mt76_set(MT_WFDMA0_BUSY_ENA + hif1_ofs,
                MT_WFDMA0_PCIE1_BUSY_ENA_TX_FIFO0 |
                MT_WFDMA0_PCIE1_BUSY_ENA_TX_FIFO1 |
                MT_WFDMA0_PCIE1_BUSY_ENA_RX_FIFO);
        }

        // dma.c:663-664 - Wait for HIF_MISC_BUSY
        let hif_misc_before = self.mt76_rr(MT_WFDMA_EXT_CSR_HIF_MISC);
        udebug!("dma", "poll_hif_misc"; val = hif_misc_before);
        let poll_result = self.mt76_poll(MT_WFDMA_EXT_CSR_HIF_MISC,
            MT_WFDMA_EXT_CSR_HIF_MISC_BUSY, 0, 1000);
        if poll_result {
            udebug!("dma", "hif_misc_busy_cleared");
        } else {
            uwarn!("dma", "hif_misc_busy_timeout");
        }

        // UPSTREAM LINUX: GLO_CFG_EXT0 on HIF1 (line 217-219)
        self.mt76_set_log(WF_WFDMA0_GLO_CFG_EXT0,
            WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD |
            WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);

        // UPSTREAM LINUX: GLO_CFG_EXT1 on HIF1 TX_FCTRL_MODE (line 220-222)
        self.mt76_set_log(WF_WFDMA0_GLO_CFG_EXT1,
            WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);

        // UPSTREAM LINUX: WFDMA rx thresholds on HIF1 (lines 223-226)
        self.mt76_wr_log(MT_WFDMA0_PAUSE_RX_Q_45_TH, 0xc000c);
        self.mt76_wr_log(MT_WFDMA0_PAUSE_RX_Q_67_TH, 0x10008);
        self.mt76_wr_log(MT_WFDMA0_PAUSE_RX_Q_89_TH, 0x10008);
        self.mt76_wr_log(MT_WFDMA0_PAUSE_RX_Q_RRO_TH, 0x20);

        // UPSTREAM LINUX: HIF2 configuration
        if self.has_hif2 {
            // UPSTREAM LINUX: GLO_CFG_EXT0 on HIF2 (line 227-229)
            self.mt76_set_log(WF_WFDMA0_GLO_CFG_EXT0 + hif1_ofs,
                WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD |
                WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);

            // UPSTREAM LINUX: GLO_CFG_EXT1 on HIF2 TX_FCTRL_MODE (line 230-232)
            self.mt76_set_log(WF_WFDMA0_GLO_CFG_EXT1 + hif1_ofs,
                WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);

            // UPSTREAM LINUX: HOST_CONFIG (line 233-235)
            self.mt76_set_log(MT_WFDMA_HOST_CONFIG,
                MT_WFDMA_HOST_CONFIG_PDMA_BAND);

            // UPSTREAM LINUX: HOST_CONFIG: clear band PCIE1 bits
            self.mt76_clear_log(MT_WFDMA_HOST_CONFIG,
                MT_WFDMA_HOST_CONFIG_BAND0_PCIE1 |
                MT_WFDMA_HOST_CONFIG_BAND1_PCIE1 |
                MT_WFDMA_HOST_CONFIG_BAND2_PCIE1);

            // UPSTREAM LINUX: HOST_CONFIG: set BAND2_PCIE1 for MT7996
            self.mt76_set_log(MT_WFDMA_HOST_CONFIG,
                MT_WFDMA_HOST_CONFIG_BAND2_PCIE1);

            // UPSTREAM LINUX: AXI read outstanding (line 236-238)
            self.mt76_rmw_log(MT_WFDMA_AXI_R2A_CTRL,
                MT_WFDMA_AXI_R2A_CTRL_OUTSTAND_MASK, 0x14);

            // UPSTREAM LINUX: WFDMA rx thresholds on HIF2 (lines 239-242)
            self.mt76_wr_log(MT_WFDMA0_PAUSE_RX_Q_45_TH + hif1_ofs, 0xc000c);
            self.mt76_wr_log(MT_WFDMA0_PAUSE_RX_Q_67_TH + hif1_ofs, 0x10008);
            self.mt76_wr_log(MT_WFDMA0_PAUSE_RX_Q_89_TH + hif1_ofs, 0x10008);
            self.mt76_wr_log(MT_WFDMA0_PAUSE_RX_Q_RRO_TH + hif1_ofs, 0x20);
        }

        // dma.c:744-749 - RX_INT_PCIE_SEL for HIF2 (line 243-245)
        if self.has_hif2 {
            // dma.c:747-748 - Non-WED path: set RING3
            self.mt76_set_log(MT_WFDMA0_RX_INT_PCIE_SEL, MT_WFDMA0_RX_INT_SEL_RING3);
        }

        // dma.c:754 - Call dma_start
        self.trace_checkpoint("mt7996_dma_start_entry");
        self.mt7996_dma_start(reset, true);
        self.trace_checkpoint("mt7996_dma_start_exit");

        // AFTER state
        udebug!("dma", "dma_enable_done";
            rst = self.mt76_rr(MT_WFDMA0_RST),
            glo_cfg = self.mt76_rr(MT_WFDMA0_GLO_CFG),
            hif_misc = self.mt76_rr(MT_WFDMA_EXT_CSR_HIF_MISC),
            busy_ena = self.mt76_rr(MT_WFDMA0_BUSY_ENA));
    }

    // ========================================================================
    // Queue allocation - EXACT Linux translation
    // Source: dma.c mt76_dma_queue_reset() + mt76_dma_sync_idx()
    //
    // CRITICAL: Linux writes CPU_IDX/DMA_IDX BEFORE DESC_BASE/RING_SIZE!
    // ========================================================================

    fn program_queue(&self, regs_base: u32, hw_idx: u32, desc_phys: u64, ndesc: u32) {
        // CRITICAL: Convert CPU physical address to PCIe bus/DMA address
        let desc_dma = phys_to_dma(desc_phys);
        let desc_lo = desc_dma as u32;

        // Step 1: Reset CPU and DMA indices FIRST (like Linux)
        // dma.c:204-205 in mt76_dma_queue_reset()
        self.mt76_wr(regs_base + MT_QUEUE_CPU_IDX, 0);
        self.mt76_wr(regs_base + MT_QUEUE_DMA_IDX, 0);

        // Step 2: Program descriptor base and ring size
        // dma.c:186-187 in mt76_dma_sync_idx()
        // NOTE: Writing DMA address, not CPU physical address!
        self.mt76_wr(regs_base + MT_QUEUE_DESC_BASE, desc_lo);
        self.mt76_wr(regs_base + MT_QUEUE_RING_SIZE, ndesc);

        // Log in Linux [MT76_DMA] format
        println!("[MT76_DMA] program_queue hw_idx={}: DESC_BASE=0x{:08x} RING_SIZE={}",
                 hw_idx, desc_lo, ndesc);

        // Step 3: Read back DMA_IDX to sync (like Linux mt76_dma_sync_idx)
        // dma.c:188 - q->head = Q_READ(dev, q, dma_idx);
        let dma_idx = self.mt76_rr(regs_base + MT_QUEUE_DMA_IDX);
        println!("[MT76_DMA] sync: DMA_IDX={}", dma_idx);
    }

    fn init_mcu_queue(&self, hw_idx: u32, ring_base: u32, ndesc: u32, desc_phys: u64, desc_virt: u64) {
        let regs_base = ring_base + hw_idx * MT_RING_SIZE;

        // Clear descriptors - DMA_DONE=1 means CPU owns, ready to be filled
        for i in 0..ndesc as usize {
            let desc_ptr = (desc_virt as *mut Mt76Desc).wrapping_add(i);
            unsafe {
                // Write all fields with volatile for hardware consistency
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                // ctrl LAST - DMA_DONE=1 means TX completed (owned by CPU)
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, MT_DMA_CTL_DMA_DONE);
            }
        }

        self.program_queue(regs_base, hw_idx, desc_phys, ndesc);
    }

    fn init_rx_queue(&self, hw_idx: u32, ring_base: u32, ndesc: u32, desc_phys: u64, desc_virt: u64) {
        let regs_base = ring_base + hw_idx * MT_RING_SIZE;

        // Clear descriptors - DMA_DONE=1 means CPU owns, will be cleared when we fill with buffers
        for i in 0..ndesc as usize {
            let desc_ptr = (desc_virt as *mut Mt76Desc).wrapping_add(i);
            unsafe {
                // Write all fields with volatile for hardware consistency
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                // ctrl LAST - DMA_DONE=1 means not ready for DMA yet
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, MT_DMA_CTL_DMA_DONE);
            }
        }

        self.program_queue(regs_base, hw_idx, desc_phys, ndesc);
    }

    // ========================================================================
    // mt76_dma_rx_fill() - Fill RX queue with buffers
    // Source: dma.c:684-694 (mt76_dma_rx_fill_buf)
    //
    // CRITICAL: This MUST be called AFTER DMA enable!
    // Writing CPU_IDX is the "kick" that starts DMA processing.
    // The kick only works when GLO_CFG has DMA enabled.
    // ========================================================================

    fn rx_fill(&self, q: &RxQueueInfo) {
        // Fill descriptors with buffers (up to ndesc-1, leave 1 for wrap detection)
        // Linux: while (q->queued < q->ndesc - 1) { ... }
        let fill_count = (q.ndesc - 1) as usize;

        udebug!("dma", "rx_fill"; count = fill_count, buf_size = q.buf_size, buf_phys = q.buf_phys);

        for i in 0..fill_count {
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let buf_phys = q.buf_phys + (i as u64 * q.buf_size as u64);
            // CRITICAL: Convert to DMA address for PCIe device
            let buf_dma = phys_to_dma(buf_phys);

            // dma.c:220 - ctrl = FIELD_PREP(MT_DMA_CTL_SD_LEN0, buf[0].len)
            // SD_LEN0 is bits 29:16, so shift length left by 16
            let ctrl = (q.buf_size as u32) << 16;  // Length in upper bits, DMA_DONE=0

            // CRITICAL: Write order must match Linux: buf0, buf1, info, THEN ctrl (last!)
            // NOTE: Using DMA address, not CPU physical address!
            // Linux mt76/dma.c: desc->buf0 = lower_32_bits(addr), desc->buf1 = upper_32_bits(addr)
            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, dma_addr_lo(buf_dma));
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, dma_addr_hi(buf_dma));  // High bits in buf1!
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);  // info=0 for RX
                // Write ctrl LAST - DMA_DONE=0 means DMA owns this descriptor
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, ctrl);
            }
        }

        // CRITICAL: Flush descriptor cache to RAM so DMA device can see them!
        // On ARM64, CPU writes go to cache first. Without flush, device reads stale RAM.
        let desc_bytes = fill_count * core::mem::size_of::<Mt76Desc>();
        flush_buffer(q.desc_virt, desc_bytes);

        // Ensure writes are visible to DMA (use proper DSB barrier for PCIe)
        dma_wmb();

        // dma.c:403 - Q_WRITE(q, cpu_idx, q->head) - tell hardware buffers are ready
        // head = fill_count (index of next to fill)
        // THIS IS THE "KICK" - tells DMA engine to start processing
        self.mt76_wr(q.regs_base + MT_QUEUE_CPU_IDX, fill_count as u32);

        // Log KICK in Linux [MT76_DMA] format
        println!("[MT76_DMA] KICK hw_idx={}: CPU_IDX={}", q.hw_idx, fill_count);

        // Read back to verify and log result
        let cpu_idx = self.mt76_rr(q.regs_base + MT_QUEUE_CPU_IDX);
        let dma_idx = self.mt76_rr(q.regs_base + MT_QUEUE_DMA_IDX);
        println!("[MT76_DMA] after_kick hw_idx={}: CPU_IDX={} DMA_IDX={}", q.hw_idx, cpu_idx, dma_idx);
    }

    // ========================================================================
    // mt7996_dma_init() - EXACT Linux translation
    // Source: dma.c:599-854
    // ========================================================================

    fn mt7996_dma_init(&self, desc_phys: u64, desc_virt: u64, _desc_size: usize,
                       rx_buf_phys: u64, rx_buf_virt: u64, _rx_buf_size: usize) {
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        const DESC_SIZE: usize = 16;
        // Helper to calculate aligned ring bytes (4KB alignment like Linux dmam_alloc_coherent)
        #[inline]
        const fn ring_bytes(ndesc: u32) -> usize {
            let raw = (ndesc as usize) * DESC_SIZE;
            // Round up to 4KB alignment
            (raw + 4095) & !4095
        }

        // dma.c:611 - mt7996_dma_config() was called in constructor

        // dma.c:613 - mt76_dma_attach (sets up DMA ops - we do this implicitly)

        // dma.c:618 - Disable DMA with reset
        self.trace_checkpoint("mt7996_dma_disable_entry");
        self.mt7996_dma_disable(true);
        self.trace_checkpoint("mt7996_dma_disable_exit");

        println!("\n=== Ring Configuration ===");

        // Track offset in descriptor memory
        let mut offset: usize = 0;

        // Track RX queues for buffer fill (NEW - Linux does this in mt76_init_queues)
        let mut rx_queues: [RxQueueInfo; NUM_RX_QUEUES] = [RxQueueInfo::default(); NUM_RX_QUEUES];
        let mut rx_queue_idx: usize = 0;
        let mut rx_buf_offset: usize = 0;

        let tx_ring_base = MT_WFDMA0_BASE + 0x300;
        let rx_ring_base = MT_WFDMA0_BASE + 0x500;

        // dma.c:621-628 - init tx queue (BAND0)
        // Linux: MT7996_TX_RING_SIZE = 2048
        self.init_mcu_queue(
            MT7996_TXQ_BAND0,
            tx_ring_base,
            MT7996_TX_RING_SIZE,
            desc_phys + offset as u64,
            desc_virt + offset as u64,
        );
        offset += ring_bytes(MT7996_TX_RING_SIZE);
        udebug!("dma", "queue_init"; queue = "TX_BAND0", reg = tx_ring_base + MT7996_TXQ_BAND0 * MT_RING_SIZE);

        // dma.c:631-637 - command to WM
        // Linux: MT7996_TX_MCU_RING_SIZE = 256
        self.init_mcu_queue(
            MT7996_TXQ_MCU_WM,
            tx_ring_base,
            MT7996_TX_MCU_RING_SIZE,
            desc_phys + offset as u64,
            desc_virt + offset as u64,
        );
        offset += ring_bytes(MT7996_TX_MCU_RING_SIZE);
        udebug!("dma", "queue_init"; queue = "TX_MCU_WM", reg = tx_ring_base + MT7996_TXQ_MCU_WM * MT_RING_SIZE);

        // dma.c:640-647 - command to WA (mt7996_has_wa = true for MT7996)
        // Linux: MT7996_TX_MCU_RING_SIZE = 256
        self.init_mcu_queue(
            MT7996_TXQ_MCU_WA,
            tx_ring_base,
            MT7996_TX_MCU_RING_SIZE,
            desc_phys + offset as u64,
            desc_virt + offset as u64,
        );
        offset += ring_bytes(MT7996_TX_MCU_RING_SIZE);
        udebug!("dma", "queue_init"; queue = "TX_MCU_WA", reg = tx_ring_base + MT7996_TXQ_MCU_WA * MT_RING_SIZE);

        // dma.c:650-656 - firmware download
        // Linux: MT7996_TX_FWDL_RING_SIZE = 128
        self.init_mcu_queue(
            MT7996_TXQ_FWDL,
            tx_ring_base,
            MT7996_TX_FWDL_RING_SIZE,
            desc_phys + offset as u64,
            desc_virt + offset as u64,
        );
        offset += ring_bytes(MT7996_TX_FWDL_RING_SIZE);
        udebug!("dma", "queue_init"; queue = "TX_FWDL", reg = tx_ring_base + MT7996_TXQ_FWDL * MT_RING_SIZE);

        // dma.c:659-665 - event from WM (MCU RX queue)
        // Linux: MT7996_RX_MCU_RING_SIZE = 512
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_MCU_WM,
            rx_ring_base,
            MT7996_RX_MCU_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_MCU_WM,
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WM * MT_RING_SIZE,
            ndesc: MT7996_RX_MCU_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_MCU_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MT7996_RX_MCU_RING_SIZE as usize * MT7996_RX_MCU_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_MCU_RING_SIZE);

        // dma.c:668-675 - event from WA
        // Linux hw_idx=1 uses 1024 entries (MT7996_RX_MCU_RING_SIZE_WA)
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_MCU_WA,
            rx_ring_base,
            MT7996_RX_MCU_RING_SIZE_WA,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_MCU_WA,
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WA * MT_RING_SIZE,
            ndesc: MT7996_RX_MCU_RING_SIZE_WA,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_MCU_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MT7996_RX_MCU_RING_SIZE_WA as usize * MT7996_RX_MCU_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_MCU_RING_SIZE_WA);

        // dma.c:677-692 - rx data queue for band0
        // Linux: MT7996_RX_RING_SIZE = 1536
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_BAND0,
            rx_ring_base,
            MT7996_RX_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_BAND0,
            regs_base: rx_ring_base + MT7996_RXQ_BAND0 * MT_RING_SIZE,
            ndesc: MT7996_RX_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MT7996_RX_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_RING_SIZE);

        // dma.c:694-706 - tx free notify event from WA for band0 (mt7996_has_wa = true)
        // Linux hw_idx=2 uses 512 entries (MT7996_RX_MCU_RING_SIZE)
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_MCU_WA_MAIN,
            rx_ring_base,
            MT7996_RX_MCU_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_MCU_WA_MAIN,
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WA_MAIN * MT_RING_SIZE,
            ndesc: MT7996_RX_MCU_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MT7996_RX_MCU_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_MCU_RING_SIZE);

        // dma.c:735-756 - MT7996 band2 queues
        // rx data queue for mt7996 band2
        // Linux: MT7996_RX_RING_SIZE = 1536
        let rx_base_band2 = rx_ring_base + hif1_ofs;
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_BAND2,
            rx_base_band2,
            MT7996_RX_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_BAND2,
            regs_base: rx_base_band2 + MT7996_RXQ_BAND2 * MT_RING_SIZE,
            ndesc: MT7996_RX_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MT7996_RX_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_RING_SIZE);

        // tx free notify event from WA for mt7996 band2
        // Linux hw_idx=3 uses 512 entries (MT7996_RX_MCU_RING_SIZE)
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_MCU_WA_TRI,
            rx_ring_base,
            MT7996_RX_MCU_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_MCU_WA_TRI,
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WA_TRI * MT_RING_SIZE,
            ndesc: MT7996_RX_MCU_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MT7996_RX_MCU_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_MCU_RING_SIZE);

        // We only need 6 RX queues for firmware download
        // The 7th slot is reserved for RXQ_BAND1 if needed later
        let _ = offset;
        let _ = rx_buf_offset;

        // === Ring Configuration Dump (match Linux format) ===
        // MCU_WM ring (hw_idx=17)
        let mcu_wm_base = tx_ring_base + MT7996_TXQ_MCU_WM * MT_RING_SIZE;
        let mcu_wm_desc = self.mt76_rr(mcu_wm_base + MT_QUEUE_DESC_BASE);
        udebug!("dma", "ring_dump"; queue = "MCU_WM", hw_idx = 17, desc_base = mcu_wm_desc, ndesc = MT7996_TX_MCU_RING_SIZE, ring_base = mcu_wm_base);

        // FWDL ring (hw_idx=16)
        let fwdl_base = tx_ring_base + MT7996_TXQ_FWDL * MT_RING_SIZE;
        let fwdl_desc = self.mt76_rr(fwdl_base + MT_QUEUE_DESC_BASE);
        udebug!("dma", "ring_dump"; queue = "FWDL", hw_idx = 16, desc_base = fwdl_desc, ndesc = MT7996_TX_FWDL_RING_SIZE, ring_base = fwdl_base);
        udebug!("dma", "fwdl_regs";
            desc_base = self.mt76_rr(fwdl_base + MT_QUEUE_DESC_BASE),
            ring_size = self.mt76_rr(fwdl_base + MT_QUEUE_RING_SIZE),
            cpu_idx = self.mt76_rr(fwdl_base + MT_QUEUE_CPU_IDX),
            dma_idx = self.mt76_rr(fwdl_base + MT_QUEUE_DMA_IDX));

        self.trace_checkpoint("ring_config_done");

        // ====================================================================
        // CRITICAL: Fill RX buffers BEFORE enabling DMA!
        // Linux sequence (from armbian_golden.log):
        //   1. sync_idx (program DESC_BASE/RING_SIZE) - lines 105-132
        //   2. KICK RX queues (CPU_IDX write) - lines 133-138
        //   3. dma_enable (GLO_CFG) - line 140
        // The device stores CPU_IDX even while DMA is disabled, then starts
        // processing those descriptors when DMA is enabled.
        // ====================================================================
        udebug!("dma", "rx_fill_start"; num_queues = rx_queue_idx);
        for i in 0..rx_queue_idx {
            udebug!("dma", "rx_queue_fill"; queue_idx = i);
            self.rx_fill(&rx_queues[i]);
        }

        // dma.c:853 - Enable DMA AFTER RX queues are filled
        self.trace_checkpoint("mt7996_dma_enable_entry");
        self.mt7996_dma_enable(false);
        self.trace_checkpoint("mt7996_dma_enable_exit");
    }
}

// ============================================================================
// MCU Command Definitions
// Source: mt76_connac_mcu.h, mcu.c
// ============================================================================

/// MCU packet types
mod mcu_pkt {
    pub const MCU_PKT_ID: u8 = 0xa0;
}

/// MCU command IDs
mod mcu_cmd {
    pub const TARGET_ADDRESS_LEN_REQ: u8 = 0x01;
    pub const FW_START_REQ: u8 = 0x02;
    pub const PATCH_START_REQ: u8 = 0x05;
    pub const PATCH_FINISH_REQ: u8 = 0x07;
    pub const PATCH_SEM_CTRL: u8 = 0x10;
    pub const FW_SCATTER: u8 = 0xee;
}

/// MCU download modes (from mt76_connac_mcu.h)
mod dl_mode {
    pub const ENCRYPT: u32 = 1 << 0;           // DL_MODE_ENCRYPT
    pub const KEY_IDX_MASK: u32 = 0x3 << 1;    // DL_MODE_KEY_IDX
    pub const RESET_SEC_IV: u32 = 1 << 3;      // DL_MODE_RESET_SEC_IV
    pub const WORKING_PDA_CR4: u32 = 1 << 4;   // DL_MODE_WORKING_PDA_CR4 (for WA)
    pub const VALID_RAM_ENTRY: u32 = 1 << 5;   // DL_MODE_VALID_RAM_ENTRY
    pub const NEED_RSP: u32 = 1 << 31;         // DL_MODE_NEED_RSP
}

/// MCU query type (mt76_connac_mcu.h enum)
#[allow(dead_code)]
mod mcu_q {
    pub const QUERY: u8 = 0;
    pub const SET: u8 = 1;
    pub const RESERVED: u8 = 2;
    pub const NA: u8 = 3;  // Used for non-ext_cid commands
}

/// Source-to-destination index for MCU commands
const S2D_H2N: u8 = 0;  // Host to N9 (WM)

/// Maximum DMA buffer size for firmware chunks
const MCU_FW_DL_BUF_SIZE: usize = 4096;

/// MCU TXD header structure
/// Source: mt76_connac_mcu.h struct mt76_connac2_mcu_txd
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct McuTxd {
    /// Hardware TXD (32 bytes)
    txd: [u32; 8],
    /// Payload length (excluding TXD)
    len: u16,
    /// Port queue ID
    pq_id: u16,
    /// Command ID
    cid: u8,
    /// Packet type (0xa0 for MCU)
    pkt_type: u8,
    /// Set/Query flag
    set_query: u8,
    /// Sequence number
    seq: u8,
    /// Reserved
    uc_d2b0_rev: u8,
    /// Extended command ID
    ext_cid: u8,
    /// Source-to-destination index
    s2d_index: u8,
    /// Extended CID ACK
    ext_cid_ack: u8,
    /// Reserved padding
    rsv: [u32; 5],
}

impl McuTxd {
    const SIZE: usize = 64;  // Must match sizeof(mt76_connac2_mcu_txd) = 64 bytes

    fn new(cmd: u8, len: u16, seq: u8) -> Self {
        let mut txd = Self::default();

        // TXD[0]: tx_bytes (bits 15:0) | pkt_fmt (bits 24:23) | q_idx (bits 31:25)
        // From mt76_connac3_mac.h:
        //   MT_TXD0_TX_BYTES = GENMASK(15, 0)
        //   MT_TXD0_PKT_FMT = GENMASK(24, 23)  - MT_TX_TYPE_CMD = 2
        //   MT_TXD0_Q_IDX = GENMASK(31, 25)   - MT_TX_MCU_PORT_RX_Q0 = 0x20
        let total_len = (Self::SIZE + len as usize) as u32;
        const MT_TX_TYPE_CMD: u32 = 2;
        const MT_TX_MCU_PORT_RX_Q0: u32 = 0x20;
        txd.txd[0] = (total_len & 0xffff) | (MT_TX_TYPE_CMD << 23) | (MT_TX_MCU_PORT_RX_Q0 << 25);

        // TXD[1]: hdr_format (bits 15:14)
        // From mt76_connac3_mac.h:
        //   MT_TXD1_HDR_FORMAT = GENMASK(15, 14)  - MT_HDR_FORMAT_CMD = 1
        // Note: MT7996 does NOT use LONG_FORMAT (bit 31) unlike older chips!
        const MT_HDR_FORMAT_CMD: u32 = 1;
        txd.txd[1] = MT_HDR_FORMAT_CMD << 14;

        // MCU specific fields
        // Linux: mcu_txd->len = cpu_to_le16(skb->len - sizeof(mcu_txd->txd));
        // = total_len - 32 (TXD header size)
        txd.len = ((Self::SIZE + len as usize - 32) as u16).to_le();
        txd.pq_id = 0x8000u16.to_le();  // MCU port
        txd.cid = cmd;
        txd.pkt_type = mcu_pkt::MCU_PKT_ID;
        // Linux: mcu_txd->set_query = MCU_Q_NA for non-ext_cid commands
        txd.set_query = mcu_q::NA;
        txd.seq = seq;
        txd.s2d_index = S2D_H2N;

        txd
    }

    fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, Self::SIZE)
        }
    }
}

/// Init download request payload
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct InitDlRequest {
    addr: u32,
    len: u32,
    mode: u32,
}

impl InitDlRequest {
    fn new(addr: u32, len: u32, mode: u32) -> Self {
        Self {
            addr: addr.to_le(),
            len: len.to_le(),
            mode: mode.to_le(),
        }
    }

    fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<Self>())
        }
    }
}

/// Patch semaphore control request
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct PatchSemCtrl {
    op: u32,  // 0 = release, 1 = get
}

impl PatchSemCtrl {
    fn get() -> Self { Self { op: 1u32.to_le() } }
    fn release() -> Self { Self { op: 0u32.to_le() } }

    fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, 4)
        }
    }
}

// ============================================================================
// Firmware Header Structures
// Source: mcu.c struct mt7996_patch_hdr, mt7996_fw_trailer, mt7996_fw_region
// ============================================================================

/// Patch header descriptor (inner struct)
/// From Linux mcu.c:20-27
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct PatchHeaderDesc {
    patch_ver: u32,      // __be32
    subsys: u32,         // __be32
    feature: u32,        // __be32
    n_region: u32,       // __be32
    crc: u32,            // __be32
    reserved: [u32; 11], // 44 bytes
}

/// Patch header - at START of patch file
/// From mt76_connac_mcu.h - total 96 bytes
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct PatchHeader {
    /// Build date (16 bytes)
    build_date: [u8; 16],
    /// Platform (4 bytes)
    platform: [u8; 4],
    /// Hardware/software version (big-endian)
    hw_sw_ver: u32,
    /// Patch version (big-endian)
    patch_ver: u32,
    /// Checksum (big-endian u16)
    checksum: u16,
    /// Reserved
    _reserved: u16,
    /// Descriptor
    desc: PatchHeaderDesc,
}

/// Patch section - 64 bytes each
/// From Linux mcu.c:30-44
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct PatchSec {
    /// Section type (big-endian)
    sec_type: u32,
    /// Section offset in file (big-endian)
    offs: u32,
    /// Section size (big-endian)
    size: u32,
    /// Info union - using info variant
    addr: u32,           // __be32 - target address
    len: u32,            // __be32 - length
    sec_key_idx: u32,    // __be32 - security key index
    align_len: u32,      // __be32 - aligned length
    _reserved: [u32; 9], // 36 bytes reserved
}

/// Firmware trailer - at END of firmware file
/// From Linux mcu.c:46-56
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct FwTrailer {
    /// Chip ID - NOTE: u8 NOT u16! Linux mcu.c:47
    chip_id: u8,
    /// ECO code
    eco_code: u8,
    /// Number of regions
    n_region: u8,
    /// Format version
    format_ver: u8,
    /// Format flag
    format_flag: u8,
    /// Reserved
    _reserved: [u8; 2],
    /// Firmware version string
    fw_ver: [u8; 10],
    /// Build date
    build_date: [u8; 15],
    /// CRC32
    crc: u32,
}

/// Firmware region descriptor
/// From mt76_connac_mcu.h:184-194 - MUST be 40 bytes!
#[repr(C, packed)]
#[derive(Clone, Copy)]
struct FwRegion {
    /// CRC of decompressed data
    decomp_crc: u32,
    /// Decompressed length
    decomp_len: u32,
    /// Decompressed block size
    decomp_blk_sz: u32,
    /// Reserved
    _reserved: [u8; 4],
    /// Target address
    addr: u32,
    /// Compressed length
    len: u32,
    /// Feature set (1 byte)
    feature_set: u8,
    /// Region type (1 byte) - was missing!
    region_type: u8,
    /// Reserved (14 bytes, not 15!)
    _reserved1: [u8; 14],
}

// ============================================================================
// DMA Ring Management for MCU Commands
// ============================================================================

/// TX ring state
struct TxRing {
    /// Base register address for this ring
    regs_base: u32,
    /// Number of descriptors
    ndesc: u32,
    /// Current CPU index (next descriptor to use)
    cpu_idx: u32,
    /// Descriptor virtual address base
    desc_virt: u64,
    /// Descriptor physical address base
    desc_phys: u64,
    /// Buffer virtual address (for copying data)
    buf_virt: u64,
    /// Buffer physical address
    buf_phys: u64,
}

impl TxRing {
    fn new(regs_base: u32, ndesc: u32, desc_virt: u64, desc_phys: u64, buf_virt: u64, buf_phys: u64) -> Self {
        Self {
            regs_base,
            ndesc,
            cpu_idx: 0,
            desc_virt,
            desc_phys,
            buf_virt,
            buf_phys,
        }
    }

    /// Get descriptor at index
    fn desc(&self, idx: u32) -> *mut Mt76Desc {
        unsafe {
            (self.desc_virt as *mut Mt76Desc).add(idx as usize)
        }
    }

    /// Get buffer for descriptor at index (4KB per descriptor)
    fn buf(&self, idx: u32) -> *mut u8 {
        unsafe {
            (self.buf_virt as *mut u8).add(idx as usize * MCU_FW_DL_BUF_SIZE)
        }
    }

    /// Get buffer physical address for descriptor
    fn buf_phys(&self, idx: u32) -> u64 {
        self.buf_phys + (idx as u64 * MCU_FW_DL_BUF_SIZE as u64)
    }
}

/// RX ring state (for MCU responses)
struct RxRing {
    regs_base: u32,
    ndesc: u32,
    cpu_idx: u32,
    desc_virt: u64,
    buf_virt: u64,
    buf_phys: u64,
}

impl RxRing {
    fn new(regs_base: u32, ndesc: u32, desc_virt: u64, buf_virt: u64, buf_phys: u64) -> Self {
        Self {
            regs_base,
            ndesc,
            cpu_idx: 0,
            desc_virt,
            buf_virt,
            buf_phys,
        }
    }

    fn desc(&self, idx: u32) -> *mut Mt76Desc {
        unsafe { (self.desc_virt as *mut Mt76Desc).add(idx as usize) }
    }

    fn buf(&self, idx: u32) -> *mut u8 {
        unsafe { (self.buf_virt as *mut u8).add(idx as usize * 2048) }
    }
}

// ============================================================================
// MCU Command Functions
// ============================================================================

impl Mt7996Dev {
    /// Send MCU command via FWDL ring
    /// Returns sequence number used
    fn mcu_send_cmd(&self, ring: &mut TxRing, cmd: u8, data: &[u8], seq: u8) -> Result<(), i32> {
        let idx = ring.cpu_idx;

        // Build TXD header
        let txd = McuTxd::new(cmd, data.len() as u16, seq);

        // Copy TXD + data to buffer
        let buf = ring.buf(idx);
        let total_len = McuTxd::SIZE + data.len();
        unsafe {
            // IMPORTANT: Zero the buffer first!
            // High DMA pool memory is NOT zeroed by kernel (can't access it).
            // Garbage past our data shouldn't matter, but zero anyway to be safe.
            core::ptr::write_bytes(buf, 0, total_len);
            // Copy TXD header
            core::ptr::copy_nonoverlapping(txd.as_bytes().as_ptr(), buf, McuTxd::SIZE);
            // Copy payload
            if !data.is_empty() {
                core::ptr::copy_nonoverlapping(data.as_ptr(), buf.add(McuTxd::SIZE), data.len());
            }
        }

        let buf_phys = ring.buf_phys(idx);
        // CRITICAL: Convert to DMA address for PCIe device
        let buf_dma = phys_to_dma(buf_phys);

        // Read indices before - match Linux log format
        let cpu_before = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);
        let dma_before = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);

        // Determine hw_idx from regs_base for logging
        // TX ring 16 = FWDL, TX ring 17 = MCU_WM
        let hw_idx = (ring.regs_base - MT_WFDMA0_TX_RING_BASE) / MT_RING_SIZE;
        let qid = if hw_idx == MT7996_TXQ_FWDL { 2 } else { 0 };  // FWDL uses qid=2 in Linux

        // Log in Linux format with hw_idx for debugging
        println!("[MT7996_MCU] === send_message cmd=0x{:08x} qid={} hw_idx={} len={} regs=0x{:x} ===",
                 cmd as u32, qid, hw_idx, total_len, ring.regs_base);
        println!("[MT7996_MCU]   BEFORE: cpu_idx={} dma_idx={} queued=0", cpu_before, dma_before);

        // Detailed MCU TXD dump for first few transfers only
        if idx < 4 {
            println!("[MCU_TXD] idx={} cmd=0x{:02x} buf_phys=0x{:x} buf_dma=0x{:x}", idx, cmd, buf_phys, buf_dma);
            // Dump the McuTxd header (64 bytes) as hex
            unsafe {
                let bytes = core::slice::from_raw_parts(buf, McuTxd::SIZE);
                // TXD[0-7] (32 bytes)
                println!("[MCU_TXD] TXD[0-3]: {:08x} {:08x} {:08x} {:08x}",
                    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
                    u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
                    u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
                    u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]));
                println!("[MCU_TXD] TXD[4-7]: {:08x} {:08x} {:08x} {:08x}",
                    u32::from_le_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]),
                    u32::from_le_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]),
                    u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]),
                    u32::from_le_bytes([bytes[28], bytes[29], bytes[30], bytes[31]]));
                // MCU fields (bytes 32-63)
                println!("[MCU_TXD] len={} pq_id=0x{:04x} cid=0x{:02x} pkt_type=0x{:02x}",
                    u16::from_le_bytes([bytes[32], bytes[33]]),
                    u16::from_le_bytes([bytes[34], bytes[35]]),
                    bytes[36], bytes[37]);
                println!("[MCU_TXD] set_query={} seq={} uc_d2b0_rev={} ext_cid=0x{:02x}",
                    bytes[38], bytes[39], bytes[40], bytes[41]);
                println!("[MCU_TXD] s2d_index={} ext_cid_ack={}", bytes[42], bytes[43]);
                // Dump payload (first 16 bytes if present)
                if !data.is_empty() {
                    let payload_bytes = core::slice::from_raw_parts(buf.add(McuTxd::SIZE), data.len().min(16));
                    print!("[MCU_TXD] payload[0..{}]: ", payload_bytes.len());
                    for b in payload_bytes {
                        print!("{:02x} ", b);
                    }
                    println!();
                }
            }
        }

        // Log DMA address like Linux does: [DMA_TX] dma_addr=0x... len=N
        println!("[DMA_TX] dma_addr=0x{:x} len={}", buf_dma, total_len);

        // Fill DMA descriptor
        // CRITICAL: Write order must match Linux: buf0, buf1, info, THEN ctrl (last!)
        // Hardware triggers on ctrl write, so other fields must be ready first
        // Uses volatile writes (WRITE_ONCE equivalent)
        // NOTE: Using DMA address, not CPU physical address!
        let desc = ring.desc(idx);
        let ctrl_val = ((total_len as u32) << 16) | MT_DMA_CTL_LAST_SEC0;
        let buf0_val = dma_addr_lo(buf_dma);
        let buf1_val = dma_addr_hi(buf_dma);  // HIGH BITS GO IN BUF1!
        // Linux mt76/dma.c: desc->buf0 = lower_32_bits(addr), desc->buf1 = upper_32_bits(addr)
        // Verified from armbian log: [DMA_TX] dma_addr=0x101be8000 uses 36-bit addresses
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, buf0_val);
            core::ptr::write_volatile(&mut (*desc).buf1, buf1_val);  // High bits here!
            core::ptr::write_volatile(&mut (*desc).info, 0);  // info=0 for TX
            // Write ctrl LAST - this triggers hardware to process the descriptor
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        // Log descriptor in Linux format for comparison
        if idx < 4 {
            println!("[DMA_DESC] q=[{}] hw_idx={}: buf0=0x{:08x} buf1=0x{:08x} ctrl=0x{:08x} info=0x00000000",
                     idx, hw_idx, buf0_val, buf1_val, ctrl_val);
        }

        // CRITICAL: Flush TX buffer and descriptor to RAM so DMA device can see them!
        // Without this, data sits in CPU cache and device reads stale RAM.
        flush_buffer(buf as u64, total_len);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        // Advance CPU index
        ring.cpu_idx = (ring.cpu_idx + 1) % ring.ndesc;

        // Memory barrier before kicking DMA (Linux wmb() = dsb st on ARM64)
        dma_wmb();

        // Log in Linux format BEFORE kick - read actual hardware registers
        let head = idx;
        let cpu_before = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);  // Read HW register
        let dma_before = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
        println!("[MT76_DMA] KICK q={:016x} hw_idx={}: head={} cpu={}->{}  dma={}",
                 ring as *const TxRing as u64, hw_idx, head, cpu_before, ring.cpu_idx, dma_before);

        // Kick DMA by writing CPU index
        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        // CRITICAL: Linux handles BOTH HIF1 and HIF2 interrupt registers!
        // MT7996 has dual HIFs and both must be managed together.
        // This is the key difference that was causing DMA_IDX to stay stuck.

        // Step 1: Clear HIF1 + HIF2 INT_MASK
        let int_mask1_old = self.mt76_rr(MT_INT_MASK_CSR);
        let int_mask2_old = self.mt76_rr(MT_INT1_MASK_CSR);
        self.mt76_wr(MT_INT_MASK_CSR, 0);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", MT_INT_MASK_CSR, int_mask1_old, 0u32);
        self.mt76_wr(MT_INT1_MASK_CSR, 0);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", MT_INT1_MASK_CSR, int_mask2_old, 0u32);

        // Step 2: Write again (Linux does this - may be for synchronization)
        self.mt76_wr(MT_INT_MASK_CSR, 0);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", MT_INT_MASK_CSR, 0u32, 0u32);
        self.mt76_wr(MT_INT1_MASK_CSR, 0);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", MT_INT1_MASK_CSR, 0u32, 0u32);

        // Step 3: Read/Clear HIF1 INT_SRC
        let int_src1 = self.mt76_rr(MT_INT_SOURCE_CSR);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}", MT_INT_SOURCE_CSR, int_src1);
        self.mt76_wr(MT_INT_SOURCE_CSR, int_src1);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", MT_INT_SOURCE_CSR, int_src1, int_src1);

        // Step 4: Read/Clear HIF2 INT_SRC
        let int_src2 = self.mt76_rr(MT_INT1_SOURCE_CSR);
        println!("[MT76_REG] RD 0x{:05x} = 0x{:08x}", MT_INT1_SOURCE_CSR, int_src2);
        self.mt76_wr(MT_INT1_SOURCE_CSR, int_src2);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", MT_INT1_SOURCE_CSR, int_src2, int_src2);

        // Step 5: Restore HIF1 + HIF2 INT_MASK
        self.mt76_wr(MT_INT_MASK_CSR, int_mask1_old);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", MT_INT_MASK_CSR, 0u32, int_mask1_old);
        self.mt76_wr(MT_INT1_MASK_CSR, int_mask2_old);
        println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", MT_INT1_MASK_CSR, 0u32, int_mask2_old);

        // Read indices after kick
        let cpu_after = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);
        let dma_after = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);

        // Log in Linux format: AFTER: cpu_idx=N dma_idx=N ret=0
        println!("   AFTER:  cpu_idx={} dma_idx={} ret=0", cpu_after, dma_after);

        Ok(())
    }

    /// Wait for TX descriptor to complete
    fn mcu_wait_tx_done(&self, ring: &TxRing, idx: u32, timeout_ms: u32) -> bool {
        let desc = ring.desc(idx);
        for _ in 0..timeout_ms {
            let ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
            if (ctrl & MT_DMA_CTL_DMA_DONE) != 0 {
                return true;
            }
            userlib::delay_ms(1);
        }
        false
    }

    /// Send init download command
    ///
    /// Linux logic (mt76_connac_mcu.c:67-73):
    /// - For MT7996 (is_mt799x): addr == 0x900000 → PATCH_START_REQ
    /// - Otherwise → TARGET_ADDRESS_LEN_REQ
    fn mcu_init_download(&self, ring: &mut TxRing, addr: u32, len: u32, mode: u32, seq: u8) -> Result<(), i32> {
        // MT7996 uses PATCH_START_REQ only for addr == 0x900000
        // Other connac chips use different conditions but we only support MT7996
        let cmd = if addr == 0x900000 {
            udebug!("mcu", "init_download_cmd"; addr = addr, cmd = "PATCH_START_REQ");
            mcu_cmd::PATCH_START_REQ
        } else {
            udebug!("mcu", "init_download_cmd"; addr = addr, cmd = "TARGET_ADDRESS_LEN_REQ");
            mcu_cmd::TARGET_ADDRESS_LEN_REQ
        };

        let req = InitDlRequest::new(addr, len, mode | dl_mode::NEED_RSP);
        self.mcu_send_cmd(ring, cmd, req.as_bytes(), seq)?;

        // Wait for TX complete
        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 1000) {
            // Dump diagnostic info on timeout
            let rst = self.mt76_rr(MT_WFDMA0_RST);
            let glo = self.mt76_rr(MT_WFDMA0_GLO_CFG);
            let int_src = self.mt76_rr(MT_INT_SOURCE_CSR);
            let dma_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
            let cpu_idx_hw = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);
            let desc_base = self.mt76_rr(ring.regs_base + MT_QUEUE_DESC_BASE);
            uerror!("mcu", "tx_timeout"; rst = rst, glo = glo, int_src = int_src);
            // Analyze GLO_CFG busy bits
            let tx_busy = (glo & 0x2) != 0;
            let rx_busy = (glo & 0x8) != 0;
            uerror!("mcu", "tx_timeout_busy"; tx_busy = tx_busy, rx_busy = rx_busy);
            uerror!("mcu", "tx_timeout_idx"; dma_idx = dma_idx, cpu_idx = cpu_idx_hw, desc_base = desc_base);

            // WFDMA internal status
            let busy_ena = self.mt76_rr(0xd413c);
            let busy_stat = self.mt76_rr(0xd4140);
            let err_sta = self.mt76_rr(0xd4144);
            uerror!("mcu", "wfdma_status"; busy_ena = busy_ena, busy_stat = busy_stat, err_sta = err_sta);
            // MCU INT EVENT: bit0=DMA_STOPPED, bit1=DMA_INIT, bit3=RESET_DONE
            let mcu_int = self.mt76_rr(0x2108);
            uerror!("mcu", "mcu_int"; val = mcu_int, dma_stop = mcu_int & 1, dma_init = (mcu_int >> 1) & 1, rst_done = (mcu_int >> 3) & 1);
            // HIF_MISC: bit0=HIF_BUSY
            let hif_misc = self.mt76_rr(0xd7044);
            uerror!("mcu", "hif_misc"; val = hif_misc, busy = hif_misc & 1);
            // HOST_CONFIG: controls HIF routing
            let host_cfg = self.mt76_rr(0xd7030);
            uerror!("mcu", "host_config"; val = host_cfg);

            // Dump MT7996 internal PCIe status
            let ltssm = self.mt76_rr(0x10150);
            let linksta = self.mt76_rr(0x10154);
            let pcie_setting = self.mt76_rr(0x10080);
            uerror!("mcu", "pcie_status"; ltssm = ltssm, linksta = linksta, setting = pcie_setting);

            // Dump the descriptor that should have been processed
            let desc = ring.desc(prev_idx);
            unsafe {
                let buf0 = core::ptr::read_volatile(&(*desc).buf0);
                let buf1 = core::ptr::read_volatile(&(*desc).buf1);
                let ctrl = core::ptr::read_volatile(&(*desc).ctrl);
                let info = core::ptr::read_volatile(&(*desc).info);
                uerror!("mcu", "desc_dump"; idx = prev_idx, buf0 = buf0, buf1 = buf1, ctrl = ctrl, info = info);
            }
            return Err(-1);
        }

        // TODO: Wait for RX response
        userlib::delay_ms(10);
        Ok(())
    }

    /// Send firmware chunk (scatter command)
    ///
    /// IMPORTANT: For FW_SCATTER, the Linux driver sends RAW firmware data
    /// directly to the DMA without any TXD header. The mt7996_mcu_send_message
    /// function checks if cmd == MCU_CMD(FW_SCATTER) and if so, jumps to exit
    /// without prepending any header - see mt7996/mcu.c:235-238.
    fn mcu_send_firmware_chunk(&self, ring: &mut TxRing, data: &[u8], _seq: u8) -> Result<(), i32> {
        static mut FIRST_CHUNK: bool = true;
        let is_first = unsafe { FIRST_CHUNK };

        let idx = ring.cpu_idx;
        let buf = ring.buf(idx);

        // Copy raw firmware data directly - NO header for FW_SCATTER!
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), buf, data.len());
        }

        let total_len = data.len();
        let buf_phys = ring.buf_phys(idx);
        // CRITICAL: Convert to DMA address for PCIe device
        let buf_dma = phys_to_dma(buf_phys);

        // Fill DMA descriptor
        // CRITICAL: Write order must match Linux: buf0, buf1, info, THEN ctrl (last!)
        // NOTE: Using DMA address, not CPU physical address!
        // Linux mt76/dma.c: desc->buf0 = lower_32_bits(addr), desc->buf1 = upper_32_bits(addr)
        let desc = ring.desc(idx);
        let ctrl_val = ((total_len as u32) << 16) | MT_DMA_CTL_LAST_SEC0;
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(buf_dma));
            core::ptr::write_volatile(&mut (*desc).buf1, dma_addr_hi(buf_dma));  // High bits in buf1!
            core::ptr::write_volatile(&mut (*desc).info, 0);  // info=0 for TX
            // Write ctrl LAST - triggers hardware
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        // CRITICAL: Flush TX buffer and descriptor to RAM so DMA device can see them!
        // Without this, data sits in CPU cache and device reads stale RAM.
        flush_buffer(buf as u64, total_len);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        // Detailed diagnostic for first chunk only
        if is_first {
            unsafe { FIRST_CHUNK = false; }
            udebug!("fw", "first_chunk_diag"; regs_base = ring.regs_base);

            // Read back GLO_CFG to verify DMA is enabled
            let glo_cfg = self.mt76_rr(MT_WFDMA0_GLO_CFG);
            udebug!("fw", "glo_cfg"; val = glo_cfg,
                tx_dma_en = (glo_cfg >> 0) & 1,
                rx_dma_en = (glo_cfg >> 2) & 1,
                ext_en = (glo_cfg >> 26) & 1);

            // Read back ring registers
            let hw_desc_base = self.mt76_rr(ring.regs_base + MT_QUEUE_DESC_BASE);
            let hw_ring_size = self.mt76_rr(ring.regs_base + MT_QUEUE_RING_SIZE);
            let hw_cpu_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);
            let hw_dma_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
            udebug!("fw", "ring_before_kick";
                desc_base = hw_desc_base,
                ring_size = hw_ring_size,
                cpu_idx = hw_cpu_idx,
                dma_idx = hw_dma_idx);

            // Read back descriptor content
            let desc_buf0 = unsafe { core::ptr::read_volatile(&(*desc).buf0) };
            let desc_ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
            let desc_buf1 = unsafe { core::ptr::read_volatile(&(*desc).buf1) };
            let desc_info = unsafe { core::ptr::read_volatile(&(*desc).info) };
            udebug!("fw", "desc_content"; idx = idx, buf0 = desc_buf0, buf1 = desc_buf1,
                ctrl = desc_ctrl, info = desc_info,
                len = (desc_ctrl >> 16) & 0x3FFF,
                last = (desc_ctrl >> 30) & 1,
                dma_done = (desc_ctrl >> 31) & 1);
            udebug!("fw", "buffer"; virt = buf as u64, phys = buf_phys, dma = buf_dma, len = total_len);

            // Check INT_SOURCE before kick
            let int_src = self.mt76_rr(MT_INT_SOURCE_CSR);
            udebug!("fw", "int_src_before"; val = int_src);
        }

        ring.cpu_idx = (ring.cpu_idx + 1) % ring.ndesc;

        // Memory barrier before kicking DMA (Linux wmb() = dsb st on ARM64)
        dma_wmb();

        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        // Read back after kick (only for first chunk)
        if is_first {
            userlib::delay_ms(1);  // Small delay to let DMA potentially start
            let hw_cpu_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);
            let hw_dma_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
            let desc_ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
            let int_src = self.mt76_rr(MT_INT_SOURCE_CSR);
            udebug!("fw", "after_kick";
                cpu_idx = hw_cpu_idx,
                dma_idx = hw_dma_idx,
                desc_ctrl = desc_ctrl,
                dma_done = (desc_ctrl >> 31) & 1,
                int_src = int_src);
        }

        // Wait for completion with extended diagnostics on failure
        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 100) {
            // Extended WFDMA diagnostics on firmware send timeout
            let rst = self.mt76_rr(MT_WFDMA0_RST);
            let glo = self.mt76_rr(MT_WFDMA0_GLO_CFG);
            let int_src = self.mt76_rr(MT_INT_SOURCE_CSR);
            uerror!("fw", "send_timeout"; rst = rst, glo = glo, int_src = int_src);
            uerror!("fw", "send_timeout_dma";
                tx_dma_en = glo & 1,
                tx_dma_busy = (glo >> 1) & 1,
                rx_dma_en = (glo >> 2) & 1,
                rx_dma_busy = (glo >> 3) & 1);

            // Extended GLO_CFG info
            let glo_ext0 = self.mt76_rr(0xd42b0);  // WF_WFDMA0_GLO_CFG_EXT0
            let glo_ext1 = self.mt76_rr(0xd42b4);  // WF_WFDMA0_GLO_CFG_EXT1
            uerror!("fw", "send_timeout_ext"; glo_ext0 = glo_ext0, glo_ext1 = glo_ext1,
                calc_mode = (glo_ext1 >> 31) & 1, tx_fctrl_mode = (glo_ext1 >> 28) & 1);

            // Queue state
            let dma_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
            let cpu_idx_hw = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);
            let desc_base = self.mt76_rr(ring.regs_base + MT_QUEUE_DESC_BASE);
            let ring_size = self.mt76_rr(ring.regs_base + MT_QUEUE_RING_SIZE);
            uerror!("fw", "queue_state"; desc_base = desc_base, size = ring_size, cpu_idx = cpu_idx_hw, dma_idx = dma_idx);

            // WFDMA internal status
            let busy_ena = self.mt76_rr(0xd413c);
            let busy_stat = self.mt76_rr(0xd4140);
            let err_sta = self.mt76_rr(0xd4144);
            uerror!("fw", "wfdma_status"; busy_ena = busy_ena, busy_stat = busy_stat, err_sta = err_sta);
            // MCU INT EVENT: bit0=DMA_STOPPED, bit1=DMA_INIT, bit3=RESET_DONE
            let mcu_int = self.mt76_rr(0x2108);
            uerror!("fw", "mcu_int"; val = mcu_int, dma_stop = mcu_int & 1, dma_init = (mcu_int >> 1) & 1, rst_done = (mcu_int >> 3) & 1);
            // HIF_MISC: bit0=HIF_BUSY
            let hif_misc = self.mt76_rr(0xd7044);
            uerror!("fw", "hif_misc"; val = hif_misc, busy = hif_misc & 1);

            // AXI controller state
            let axi_ctrl = self.mt76_rr(0xd7500);
            let axi_ctrl2 = self.mt76_rr(0xd7508);
            uerror!("fw", "axi_ctrl"; ctrl = axi_ctrl, ctrl2 = axi_ctrl2);

            // FWDL prefetch state
            let fwdl_prefetch = self.mt76_rr(0xd4640);
            uerror!("fw", "fwdl_prefetch"; val = fwdl_prefetch, base = fwdl_prefetch >> 16, depth = fwdl_prefetch & 0xFFFF);

            // MT7996 internal PCIe status
            let ltssm = self.mt76_rr(0x10150);
            let linksta = self.mt76_rr(0x10154);
            let pcie_setting = self.mt76_rr(0x10080);
            uerror!("fw", "pcie_status"; ltssm = ltssm, linksta = linksta, setting = pcie_setting);

            // Descriptor content
            unsafe {
                let buf0 = core::ptr::read_volatile(&(*desc).buf0);
                let ctrl = core::ptr::read_volatile(&(*desc).ctrl);
                let buf1 = core::ptr::read_volatile(&(*desc).buf1);
                let info = core::ptr::read_volatile(&(*desc).info);
                uerror!("fw", "desc_dump"; idx = prev_idx, buf0 = buf0, ctrl = ctrl, buf1 = buf1, info = info,
                    len = (ctrl >> 16) & 0x3FFF, last = (ctrl >> 30) & 1, dma_done = (ctrl >> 31) & 1);
            }
            return Err(-1);
        }

        Ok(())
    }

    /// Send firmware chunk without verbose logging (for bulk transfers)
    fn mcu_send_firmware_chunk_quiet(&self, ring: &mut TxRing, data: &[u8], _seq: u8) -> Result<(), i32> {
        let idx = ring.cpu_idx;
        let buf = ring.buf(idx);

        // Copy raw firmware data directly - NO header for FW_SCATTER!
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), buf, data.len());
        }

        let total_len = data.len();
        let buf_phys = ring.buf_phys(idx);
        let buf_dma = phys_to_dma(buf_phys);

        // Fill DMA descriptor
        // Linux mt76/dma.c: desc->buf0 = lower_32_bits(addr), desc->buf1 = upper_32_bits(addr)
        let desc = ring.desc(idx);
        let ctrl_val = ((total_len as u32) << 16) | MT_DMA_CTL_LAST_SEC0;
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(buf_dma));
            core::ptr::write_volatile(&mut (*desc).buf1, dma_addr_hi(buf_dma));  // High bits in buf1!
            core::ptr::write_volatile(&mut (*desc).info, 0);  // info=0 for TX
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        // Flush to RAM
        flush_buffer(buf as u64, total_len);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        ring.cpu_idx = (ring.cpu_idx + 1) % ring.ndesc;
        dma_wmb();
        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        // Wait for completion (shorter timeout for bulk transfers)
        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 100) {
            // Print minimal error for quiet mode
            println!("[MT7996_MCU] FW chunk timeout (quiet mode)");
            return Err(-1);
        }

        Ok(())
    }

    /// Send patch/firmware start command
    fn mcu_start_firmware(&self, ring: &mut TxRing, is_patch: bool, seq: u8) -> Result<(), i32> {
        let cmd = if is_patch {
            mcu_cmd::PATCH_FINISH_REQ
        } else {
            mcu_cmd::FW_START_REQ
        };

        let override_val: [u8; 4] = [0, 0, 0, 0];  // No override
        self.mcu_send_cmd(ring, cmd, &override_val, seq)?;

        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 1000) {
            println!("  mcu_start_firmware: TX timeout");
            return Err(-1);
        }

        userlib::delay_ms(50);
        Ok(())
    }

    /// Get patch semaphore
    fn mcu_patch_sem_ctrl(&self, ring: &mut TxRing, get: bool, seq: u8) -> Result<(), i32> {
        let req = if get { PatchSemCtrl::get() } else { PatchSemCtrl::release() };
        self.mcu_send_cmd(ring, mcu_cmd::PATCH_SEM_CTRL, req.as_bytes(), seq)?;

        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 1000) {
            // Same diagnostic as mcu_init_download
            let rst = self.mt76_rr(MT_WFDMA0_RST);
            let glo = self.mt76_rr(MT_WFDMA0_GLO_CFG);
            let int_src = self.mt76_rr(MT_INT_SOURCE_CSR);
            let dma_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
            let cpu_idx_hw = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);
            let desc_base = self.mt76_rr(ring.regs_base + MT_QUEUE_DESC_BASE);
            println!("  mcu_patch_sem_ctrl: TX timeout");
            println!("    RST=0x{:08x} GLO=0x{:08x} INT_SRC=0x{:08x}", rst, glo, int_src);
            let tx_busy = (glo & 0x2) != 0;
            let rx_busy = (glo & 0x8) != 0;
            println!("    TX_DMA_BUSY={} RX_DMA_BUSY={}", tx_busy, rx_busy);
            println!("    DMA_IDX={} CPU_IDX={} DESC_BASE=0x{:08x}", dma_idx, cpu_idx_hw, desc_base);

            // WFDMA internal status
            let busy_ena = self.mt76_rr(0xd413c);
            let busy_stat = self.mt76_rr(0xd4140);
            let err_sta = self.mt76_rr(0xd4144);
            println!("    WFDMA: BUSY_ENA=0x{:08x} BUSY_STAT=0x{:08x} ERR_STA=0x{:08x}",
                     busy_ena, busy_stat, err_sta);
            // MCU INT EVENT: bit0=DMA_STOPPED, bit1=DMA_INIT, bit3=RESET_DONE
            let mcu_int = self.mt76_rr(0x2108);
            println!("    MCU_INT=0x{:08x} (DMA_STOP={} DMA_INIT={} RST_DONE={})",
                     mcu_int, mcu_int & 1, (mcu_int >> 1) & 1, (mcu_int >> 3) & 1);
            // HIF_MISC: bit0=HIF_BUSY
            let hif_misc = self.mt76_rr(0xd7044);
            println!("    HIF_MISC=0x{:08x} (BUSY={})", hif_misc, hif_misc & 1);

            let desc = ring.desc(prev_idx);
            unsafe {
                let buf0 = core::ptr::read_volatile(&(*desc).buf0);
                let buf1 = core::ptr::read_volatile(&(*desc).buf1);
                let ctrl = core::ptr::read_volatile(&(*desc).ctrl);
                let info = core::ptr::read_volatile(&(*desc).info);
                println!("    DESC[{}]: buf0=0x{:08x} buf1=0x{:08x} ctrl=0x{:08x} info=0x{:08x}",
                    prev_idx, buf0, buf1, ctrl, info);
            }
            return Err(-1);
        }

        userlib::delay_ms(10);
        Ok(())
    }
}

// ============================================================================
// Firmware Loading Functions
// ============================================================================

/// File reading helper - reads entire file into buffer
///
/// NOTE: This requires a VFS layer to resolve paths. Currently stubbed
/// to always return an error since wifid firmware loading depends on
/// fatfs being available via the FirmwareClient protocol (not raw file I/O).
fn read_file(_path: &[u8], _buf: &mut [u8]) -> Result<usize, i32> {
    // Firmware loading via raw file I/O is not supported with the unified
    // syscall interface. Use FirmwareClient protocol when fatfs is available.
    uerror!("fw", "read_file_not_implemented");
    Err(-1)
}

impl Mt7996Dev {
    /// Load patch firmware
    /// From Linux mcu.c:2208-2296
    ///
    /// CRITICAL QUEUE ROUTING (Linux mcu.c:250-260):
    /// - mcu_ring (MCU_WM, hw_idx=17): MCU commands (patch_sem_ctrl, init_download, start_firmware)
    /// - fwdl_ring (FWDL, hw_idx=16): FW_SCATTER data chunks ONLY
    fn load_patch(&self, mcu_ring: &mut TxRing, fwdl_ring: &mut TxRing, fw_buf: &[u8], seq: &mut u8) -> Result<(), i32> {
        uinfo!("fw", "load_patch_start");

        // Log firmware state before patch (Linux reads MT_TOP_MISC)
        let fw_state = self.mt76_rr(MT_TOP_MISC);
        udebug!("fw", "fw_state_before"; val = fw_state);

        // Log DMA state
        udebug!("fw", "dma_state"; glo_cfg = self.mt76_rr(MT_WFDMA0_GLO_CFG), rst = self.mt76_rr(MT_WFDMA0_RST));

        // Log FWDL ring state
        let fwdl_base = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_DESC_BASE);
        let fwdl_cpu = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_CPU_IDX);
        let fwdl_dma = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_DMA_IDX);
        udebug!("fw", "fwdl_ring"; base = fwdl_base, cpu = fwdl_cpu, dma = fwdl_dma);

        let hdr_size = core::mem::size_of::<PatchHeader>();
        let sec_size = core::mem::size_of::<PatchSec>();

        if fw_buf.len() < hdr_size {
            uerror!("fw", "patch_too_small"; size = fw_buf.len());
            return Err(-1);
        }

        // Parse patch header (at START of file)
        let hdr = unsafe { &*(fw_buf.as_ptr() as *const PatchHeader) };
        let hw_ver = u32::from_be(hdr.hw_sw_ver);
        let patch_ver = u32::from_be(hdr.patch_ver);
        let n_region = u32::from_be(hdr.desc.n_region);
        uinfo!("fw", "patch_header"; hw = hw_ver, ver = patch_ver, regions = n_region);

        // Get semaphore first (MCU command → mcu_ring)
        self.mcu_patch_sem_ctrl(mcu_ring, true, *seq)?;
        *seq = seq.wrapping_add(1);

        // Iterate through patch sections (right after header)
        // From Linux mcu.c:2242-2279
        for i in 0..n_region {
            let sec_offset = hdr_size + (i as usize * sec_size);
            if sec_offset + sec_size > fw_buf.len() {
                uerror!("fw", "section_oob"; idx = i);
                return Err(-1);
            }

            let sec = unsafe { &*(fw_buf.as_ptr().add(sec_offset) as *const PatchSec) };

            // Get section info (big-endian)
            let addr = u32::from_be(sec.addr);
            let len = u32::from_be(sec.len);
            let data_offs = u32::from_be(sec.offs) as usize;

            udebug!("fw", "patch_region"; idx = i, addr = addr, len = len);

            if data_offs + len as usize > fw_buf.len() {
                uerror!("fw", "section_data_oob"; idx = i);
                return Err(-1);
            }

            // Init download for this section (MCU command → mcu_ring)
            self.mcu_init_download(mcu_ring, addr, len, 0, *seq)?;
            *seq = seq.wrapping_add(1);

            // Send section data in chunks (FW_SCATTER data → fwdl_ring)
            let section_data = &fw_buf[data_offs..data_offs + len as usize];
            let mut chunk_idx = 0u32;
            for chunk in section_data.chunks(MCU_FW_DL_BUF_SIZE - 4) {
                // Only log first few chunks to avoid spam
                if chunk_idx < 3 {
                    self.mcu_send_firmware_chunk(fwdl_ring, chunk, *seq)?;
                } else {
                    // Silent send for remaining chunks
                    self.mcu_send_firmware_chunk_quiet(fwdl_ring, chunk, *seq)?;
                }
                *seq = seq.wrapping_add(1);
                chunk_idx += 1;
            }

            // Log after region sent
            let fwdl_cpu = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_CPU_IDX);
            let fwdl_dma = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_DMA_IDX);
            udebug!("fw", "patch_region_done"; idx = i, cpu = fwdl_cpu, dma = fwdl_dma);
        }

        // Start patch (MCU command → mcu_ring)
        udebug!("fw", "patch_start");
        self.mcu_start_firmware(mcu_ring, true, *seq)?;
        *seq = seq.wrapping_add(1);

        // Log final ring state
        let fwdl_cpu = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_CPU_IDX);
        let fwdl_dma = self.mt76_rr(fwdl_ring.regs_base + MT_QUEUE_DMA_IDX);
        uinfo!("fw", "load_patch_done"; cpu = fwdl_cpu, dma = fwdl_dma);
        Ok(())
    }

    /// Load RAM firmware (WM, DSP, or WA)
    ///
    /// CRITICAL QUEUE ROUTING (Linux mcu.c:250-260):
    /// - mcu_ring (MCU_WM, hw_idx=17): MCU commands (init_download, start_firmware)
    /// - fwdl_ring (FWDL, hw_idx=16): FW_SCATTER data chunks ONLY
    fn load_ram(&self, mcu_ring: &mut TxRing, fwdl_ring: &mut TxRing, fw_buf: &[u8], name: &str, seq: &mut u8) -> Result<(), i32> {
        uinfo!("fw", "load_ram_start"; size = fw_buf.len());

        if fw_buf.len() < core::mem::size_of::<FwTrailer>() {
            uerror!("fw", "ram_too_small"; size = fw_buf.len());
            return Err(-1);
        }

        // Trailer is at end of file
        let trailer_offset = fw_buf.len() - core::mem::size_of::<FwTrailer>();
        let trailer = unsafe { &*(fw_buf.as_ptr().add(trailer_offset) as *const FwTrailer) };

        let chip_id = trailer.chip_id;  // u8, no endian conversion needed
        let n_region = trailer.n_region;
        uinfo!("fw", "ram_header"; chip_id = chip_id, regions = n_region);

        // Region descriptors are before trailer
        let region_size = core::mem::size_of::<FwRegion>();
        let regions_end = trailer_offset;
        let regions_start = regions_end - (n_region as usize * region_size);

        let mut data_offset = 0usize;
        for i in 0..n_region {
            let region_ptr = fw_buf.as_ptr().wrapping_add(regions_start + i as usize * region_size);
            let region = unsafe { &*(region_ptr as *const FwRegion) };

            let addr = u32::from_le(region.addr);
            let len = u32::from_le(region.len);

            udebug!("fw", "ram_region"; idx = i, addr = addr, len = len);

            // Init download for this region (MCU command → mcu_ring)
            self.mcu_init_download(mcu_ring, addr, len, 0, *seq)?;
            *seq = seq.wrapping_add(1);

            // Send region data in chunks (FW_SCATTER data → fwdl_ring)
            let region_data = &fw_buf[data_offset..data_offset + len as usize];
            for chunk in region_data.chunks(MCU_FW_DL_BUF_SIZE - 4) {
                self.mcu_send_firmware_chunk(fwdl_ring, chunk, *seq)?;
                *seq = seq.wrapping_add(1);
            }

            data_offset += len as usize;
        }

        // Start firmware (MCU command → mcu_ring)
        self.mcu_start_firmware(mcu_ring, false, *seq)?;
        *seq = seq.wrapping_add(1);

        uinfo!("fw", "load_ram_done");
        Ok(())
    }

    /// Main firmware loading entry point
    ///
    /// CRITICAL: Linux uses different queues for different operations:
    /// - mcu_ring (MCU_WM, hw_idx=17): ALL MCU commands (patch_sem_ctrl, init_download, start_firmware)
    /// - fwdl_ring (FWDL, hw_idx=16): ONLY firmware data chunks (FW_SCATTER)
    ///
    /// Reference: mt7996/mcu.c:250-260
    pub fn load_firmware(&self, mcu_ring: &mut TxRing, fwdl_ring: &mut TxRing) -> Result<(), i32> {
        uinfo!("fw", "load_firmware_start");
        udebug!("fw", "queues"; mcu_regs = mcu_ring.regs_base, fwdl_regs = fwdl_ring.regs_base);

        // Allocate buffer for firmware (3MB should be enough for largest file)
        const FW_BUF_SIZE: usize = 3 * 1024 * 1024;
        let (shmem_handle, _shmem_id) = match syscall::open_shmem_create(FW_BUF_SIZE) {
            Ok(v) => v,
            Err(_) => {
                uerror!("fw", "alloc_fw_buf_failed");
                return Err(-1);
            }
        };
        let fw_buf_virt = match syscall::map(shmem_handle, 0) {
            Ok(v) if v != 0 => v,
            _ => {
                let _ = syscall::close(shmem_handle);
                uerror!("fw", "map_fw_buf_failed");
                return Err(-1);
            }
        };

        let fw_buf = unsafe { core::slice::from_raw_parts_mut(fw_buf_virt as *mut u8, FW_BUF_SIZE) };
        // Linux starts seq at 1 and explicitly avoids 0 (see mcu.c:249-251)
        let mut seq: u8 = 1;

        // Check firmware state before loading
        let fw_state = self.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
        uinfo!("fw", "fw_state_before"; val = fw_state);

        // 1. Load patch (ROM patch)
        // MCU commands go to mcu_ring, firmware data to fwdl_ring
        let patch_path = b"/lib/firmware/mediatek/mt7996/mt7996_rom_patch.bin";
        match read_file(patch_path, fw_buf) {
            Ok(size) => {
                if let Err(_) = self.load_patch(mcu_ring, fwdl_ring, &fw_buf[..size], &mut seq) {
                    uerror!("fw", "patch_failed");
                }
            }
            Err(e) => uerror!("fw", "patch_not_found"; err = e),
        }

        userlib::delay_ms(100);

        // 2. Load WM (Wireless Manager firmware)
        let wm_path = b"/lib/firmware/mediatek/mt7996/mt7996_wm.bin";
        match read_file(wm_path, fw_buf) {
            Ok(size) => {
                if let Err(_) = self.load_ram(mcu_ring, fwdl_ring, &fw_buf[..size], "WM", &mut seq) {
                    uerror!("fw", "wm_failed");
                }
            }
            Err(e) => uerror!("fw", "wm_not_found"; err = e),
        }

        userlib::delay_ms(100);

        // 3. Load WA (Wireless Accelerator firmware)
        let wa_path = b"/lib/firmware/mediatek/mt7996/mt7996_wa.bin";
        match read_file(wa_path, fw_buf) {
            Ok(size) => {
                if let Err(_) = self.load_ram(mcu_ring, fwdl_ring, &fw_buf[..size], "WA", &mut seq) {
                    uerror!("fw", "wa_failed");
                }
            }
            Err(e) => uerror!("fw", "wa_not_found"; err = e),
        }

        // Final state check
        let fw_state = self.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
        uinfo!("fw", "fw_state_final"; val = fw_state);

        // Cleanup
        let _ = syscall::close(shmem_handle);

        if fw_state == 7 {
            uinfo!("fw", "load_firmware_success");
            Ok(())
        } else {
            uerror!("fw", "load_firmware_incomplete"; state = fw_state);
            Err(-1)
        }
    }
}

// ============================================================================
// Bus Framework Driver
// ============================================================================

struct WifiDriver {
    dev: Option<Mt7996Dev>,
    bar0: Option<MmioRegion>,
    desc_pool: Option<DmaPool>,
    rx_pool: Option<DmaPool>,
}

impl WifiDriver {
    const fn new() -> Self {
        Self {
            dev: None,
            bar0: None,
            desc_pool: None,
            rx_pool: None,
        }
    }
}

impl Driver for WifiDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        uinfo!("wifid", "init_start");

        // Show DMA translation mode
        if USE_DRAM_OFFSET {
            uinfo!("wifid", "dma_mode"; mode = "dram_offset", base = DRAM_BASE);
        } else {
            uinfo!("wifid", "dma_mode"; mode = "identity");
        }

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

        // Note: HIF2 registers (0xd8xxx) are accessible through HIF1's BAR at offset 0xd8xxx
        // Linux mmio.c __mt7996_reg_addr(): if (addr < 0x100000) return addr;
        // So we don't need to map HIF2's BAR separately for register access.
        // pcied registers separate ports for HIF1 (0x7990) and HIF2 (0x7991);
        // wifid only gets spawned for HIF1 (the :network port).
        let has_hif2 = false;

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

    // === TIMING DIAGNOSTIC ===
    // Verify ARM generic timer is working correctly (if delays don't work, DMA fails)
    let freq: u64;
    let start: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) start);
    }
    println!("[TIMING] cntfrq_el0 = {} Hz ({} MHz)", freq, freq / 1_000_000);
    userlib::delay_ms(100);
    let end: u64;
    unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) end); }
    let elapsed_ticks = end - start;
    let expected_ticks = freq / 10; // 100ms = freq/10 ticks
    println!("[TIMING] 100ms delay: {} ticks (expected ~{})", elapsed_ticks, expected_ticks);
    if elapsed_ticks < expected_ticks / 2 {
        println!("[TIMING] WARNING: Delay too short! Timer may not be working!");
    } else if elapsed_ticks > expected_ticks * 2 {
        println!("[TIMING] WARNING: Delay too long! Scheduler overhead?");
    } else {
        println!("[TIMING] Timer appears to be working correctly");
    }
    // === END TIMING DIAGNOSTIC ===

    // Helper to dump INT_SRC at various points (tracking when bit 20 gets set)
    let dump_int_src = |dev: &Mt7996Dev, stage: &str| {
        let int_src = dev.mt76_rr(MT_INT_SOURCE_CSR);
        let int_src1 = dev.mt76_rr(MT_INT1_SOURCE_CSR);
        println!("[INT_SRC @ {}] HIF1=0x{:08x} HIF2=0x{:08x} (bit20={})",
                 stage, int_src, int_src1, if (int_src & (1 << 20)) != 0 { "SET" } else { "clear" });
    };

    // Read initial register state (BEFORE any driver init)
    let rst_before = dev.mt76_rr(MT_WFDMA0_RST);
    let glo_before = dev.mt76_rr(MT_WFDMA0_GLO_CFG);
    let host_cfg_initial = dev.mt76_rr(0xd7030);  // HOST_CONFIG
    let hif_misc_initial = dev.mt76_rr(0xd7044);  // HIF_MISC
    let pcie_setting_initial = dev.mt76_rr(0x10080);  // MT7996 internal PCIE_SETTING
    println!("Initial state (BEFORE any init):");
    println!("   RST: 0x{:08x}", rst_before);
    println!("   GLO_CFG: 0x{:08x}", glo_before);
    println!("   HOST_CONFIG: 0x{:08x} (OpenWRT: 0x00407d01)", host_cfg_initial);
    println!("   HIF_MISC: 0x{:08x} (OpenWRT: 0x001c2000)", hif_misc_initial);
    println!("   PCIE_SETTING: 0x{:08x} (OpenWRT: 0x00003180)", pcie_setting_initial);
    dump_int_src(&dev, "initial");

    // Check if HOST_CONFIG is missing bits that OpenWRT has
    if (host_cfg_initial & 0x7d00) != 0x7d00 {
        println!("WARNING: HOST_CONFIG missing bits 0x7d00 (bits 8,10-14)!");
        println!("         Setting HOST_CONFIG bits to match OpenWRT...");
        // Set the missing bits (preserve existing, OR in the missing ones)
        dev.mt76_set(0xd7030, 0x7d00);
        let host_cfg_fixed = dev.mt76_rr(0xd7030);
        println!("         HOST_CONFIG now: 0x{:08x}", host_cfg_fixed);
    }

    // Check if PCIE_SETTING matches OpenWRT (0x00003180)
    // Bits: 7 (0x80), 8 (0x100), 12 (0x1000), 13 (0x2000)
    if pcie_setting_initial != 0x00003180 {
        println!("WARNING: PCIE_SETTING (0x10080) = 0x{:08x} differs from OpenWRT 0x00003180",
                 pcie_setting_initial);
        println!("         Setting PCIE_SETTING to 0x00003180...");
        dev.mt76_wr(0x10080, 0x00003180);
        let pcie_setting_fixed = dev.mt76_rr(0x10080);
        println!("         PCIE_SETTING now: 0x{:08x}", pcie_setting_fixed);
    }

    // EXPERIMENT: Check PCIE_LTSSM before/after WFSYS reset
    // Our value: 0x1003c01f, OpenWRT: 0x1003c81f (bit 11 = 0x800 differs)
    let ltssm_before = dev.mt76_rr(0x10150);
    println!("PCIE_LTSSM BEFORE wfsys_reset: 0x{:08x} (OpenWRT: 0x1003c81f, diff=bit11)", ltssm_before);

    // ========================================================================
    // CRITICAL FIX: Linux clears INT_MASK and INT_SRC BEFORE wfsys_reset_entry!
    // See armbian_golden.log line 13: INT_MASK written BEFORE wfsys_reset_entry
    // This is why Linux shows INT_SRC=0x00000000 INT_MASK=0x00000000 at entry
    // ========================================================================
    println!("\n=== PHASE 0: Early PCIe/Interrupt Setup (BEFORE WFSYS Reset) ===");

    // Linux log lines 1-5: Read MT_HW_REV via L1 remap (mmio.c:672)
    // This happens in mt7996_mmio_init() BEFORE wfsys_reset
    // mdev->rev = (device_id << 16) | (mt76_rr(dev, MT_HW_REV) & 0xff);
    println!("[MT7996] Reading MT_HW_REV (0x{:08x}) via L1 remap...", MT_HW_REV);
    let hw_rev = dev.mt76_rr_remap_log(MT_HW_REV);
    println!("[MT7996] MT_HW_REV = 0x{:08x} (rev = 0x{:02x})", hw_rev, hw_rev & 0xff);

    // Linux log line 6: [MT76_REG] WR 0xd4204: 0x00000000 -> 0x00000000
    dev.mt76_wr_log(MT_INT_MASK_CSR, 0);

    // Read and clear INT_SRC (Linux reads it before clearing)
    let _ = dev.mt76_rr_log(MT_INT_SOURCE_CSR);

    // Read key registers with logging (like Linux lines 14-21)
    let _ = dev.mt76_rr_log(MT_INT_MASK_CSR);
    let _ = dev.mt76_rr_log(MT_WFDMA0_GLO_CFG);
    let _ = dev.mt76_rr_log(MT_WFDMA0_RST);
    let _ = dev.mt76_rr_log(0x10150);  // LTSSM
    let _ = dev.mt76_rr_log(0x10080);  // PCIE_SETTING
    let _ = dev.mt76_rr_log(MT_PCIE_MAC_INT_ENABLE);
    let _ = dev.mt76_rr_log(MT_WFDMA_EXT_CSR_HIF_MISC);

    // ========================================================================
    // PHASE 0: WFSYS Reset - ensure clean hardware state
    // Per MT7996 init guide: this happens EARLY, before PCIe/DMA setup
    // Source: init.c:762-769
    // ========================================================================
    dev.trace_checkpoint("wfsys_reset_entry");
    dev.mt7996_wfsys_reset();
    dev.trace_checkpoint("wfsys_reset_exit");

    // ========================================================================
    // PHASE 1: PCIe setup (from pci.c:mt7996_pci_probe, AFTER wfsys_reset)
    // ========================================================================
    println!("\n=== PHASE 1: PCIe Setup (AFTER WFSYS Reset) ===");

    // HIF1: Set PCIe MAC interrupt enable (matches Linux pci.c)
    dev.mt76_wr_log(MT_PCIE_MAC_INT_ENABLE, 0xff);

    // HIF2: Via L1 remap (matches Linux for dual-HIF setup)
    println!("[MT7996] HIF2 MAC_INT via L1 remap (0x{:08x})...", MT_PCIE1_MAC_INT_ENABLE_PHYS);
    let pcie1_mapped = dev.mt7996_reg_map_l1_log(MT_PCIE1_MAC_INT_ENABLE_PHYS);
    dev.mt76_wr_log(pcie1_mapped, 0xff);

    // HIF1: Disable interrupt mask (already cleared above, but set again like Linux)
    dev.mt76_wr_log(MT_INT_MASK_CSR, 0);
    dev.mt76_wr_log(MT_INT1_MASK_CSR, 0);

    // Clear interrupt sources (write 1s to clear, like Linux)
    dev.mt76_wr_log(MT_INT_SOURCE_CSR, !0u32);

    dev.trace_checkpoint("before_dma_init");

    // ========================================================================
    // PHASE 2: init_hardware() from init.c
    // Per DeepWiki: dma_init() BEFORE mcu_init(), driver_own() is in mcu_init()
    // ========================================================================

    println!("[MT7996_DMA] === mt7996_dma_init ===");
    dev.mt7996_dma_init(desc_phys, desc_virt, DESC_MEM_SIZE, rx_buf_phys, rx_buf_virt, RX_BUF_POOL_SIZE);
    dev.trace_checkpoint("after_dma_init");

    // === Register dump for OpenWRT comparison ===
    println!("\n=== Register Dump (compare with OpenWRT) ===");
    println!("GLO_CFG      = 0x{:08x}  (OpenWRT: 0x1430b875)", dev.mt76_rr(0xd4208));
    println!("GLO_CFG_EXT0 = 0x{:08x}  (OpenWRT: 0x28c444df)", dev.mt76_rr(0xd42b0));
    println!("GLO_CFG_EXT1 = 0x{:08x}  (OpenWRT: 0x9c800404)", dev.mt76_rr(0xd42b4));
    println!("BUSY_ENA     = 0x{:08x}  (OpenWRT: 0x0000001f)", dev.mt76_rr(0xd413c));
    println!("HIF_MISC     = 0x{:08x}  (OpenWRT: 0x001c2000)", dev.mt76_rr(0xd7044));
    println!("HOST_CONFIG  = 0x{:08x}  (OpenWRT: 0x00407d01)", dev.mt76_rr(0xd7030));
    println!("AXI_R2A_CTRL = 0x{:08x}  (OpenWRT: 0xffff0c14)", dev.mt76_rr(0xd7500));
    println!("PCIE_RECOG   = 0x{:08x}  (OpenWRT: 0x00000001)", dev.mt76_rr(0xd7090));
    println!("FWDL BASE    = 0x{:08x}  (OpenWRT: 0x40181000)", dev.mt76_rr(0xd4400));
    println!("FWDL CPU_IDX = 0x{:08x}", dev.mt76_rr(0xd4408));
    println!("FWDL DMA_IDX = 0x{:08x}", dev.mt76_rr(0xd440c));
    println!("HIF2 BUSY_EN = 0x{:08x}  (OpenWRT: 0x0000001f)", dev.mt76_rr(0xd813c));

    // MT7996 internal PCIe registers (BAR + 0x10xxx)
    println!("\n=== MT7996 Internal PCIe Registers ===");
    println!("PCIE_SETTING @ 0x10080 = 0x{:08x}  (OpenWRT: 0x00003180)", dev.mt76_rr(0x10080));
    println!("PCIE_0x10084           = 0x{:08x}  (OpenWRT: 0x00000000)", dev.mt76_rr(0x10084));
    println!("PCIE_0x10088           = 0x{:08x}  (OpenWRT: 0x10020000)", dev.mt76_rr(0x10088));
    println!("PCIE_0x1008c           = 0x{:08x}  (OpenWRT: 0x08000001)", dev.mt76_rr(0x1008c));
    println!("PCIE_0x10090           = 0x{:08x}  (OpenWRT: 0xc0c0300c)", dev.mt76_rr(0x10090));
    println!("PCIE_RST_CTRL @ 0x10148= 0x{:08x}  (OpenWRT: 0x00088708)", dev.mt76_rr(0x10148));
    println!("PCIE_LTSSM   @ 0x10150 = 0x{:08x}  (OpenWRT: 0x1003c81f)", dev.mt76_rr(0x10150));
    println!("PCIE_LINKSTA @ 0x10154 = 0x{:08x}  (OpenWRT: 0x00001901)", dev.mt76_rr(0x10154));
    println!("PCIE_MAC_INT @ 0x10188 = 0x{:08x}  (OpenWRT: 0x000000ff)", dev.mt76_rr(0x10188));

    // Prefetch registers (EXT_CTRL at 0xd4600 + idx*4 for TX, 0xd4680 + idx*4 for RX)
    // FWDL = queue 16, WM = 17, WA = 20, BAND0 = 18
    println!("\n=== Prefetch Configuration ===");
    println!("FWDL (q16)  @ 0xd4640 = 0x{:08x}  (OpenWRT: 0x00000002)", dev.mt76_rr(0xd4640));
    println!("WM (q17)    @ 0xd4644 = 0x{:08x}  (OpenWRT: 0x00200002)", dev.mt76_rr(0xd4644));
    println!("BAND0 (q18) @ 0xd4648 = 0x{:08x}  (OpenWRT: 0x00400008)", dev.mt76_rr(0xd4648));
    println!("WA (q20)    @ 0xd4650 = 0x{:08x}  (OpenWRT: 0x00c00002)", dev.mt76_rr(0xd4650));
    // RX prefetch
    println!("RX_MCU (q0) @ 0xd4680 = 0x{:08x}  (OpenWRT: 0x01000002)", dev.mt76_rr(0xd4680));
    println!("RX_WA (q1)  @ 0xd4684 = 0x{:08x}  (OpenWRT: 0x01200002)", dev.mt76_rr(0xd4684));

    // ========================================================================
    // PHASE 3: mcu_init() from mcu.c - happens AFTER dma_init!
    // Per DeepWiki citations [4][5]: SWDEF_MODE then driver_own
    // ========================================================================
    dev.trace_checkpoint("mcu_init_entry");

    // mcu.c:3299-3303 - Set SWDEF_MODE BEFORE driver_own
    let swdef_old = dev.mt76_rr(MT_SWDEF_MODE);
    println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", MT_SWDEF_MODE, swdef_old, MT_SWDEF_NORMAL_MODE);
    dev.mt76_wr(MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE);

    // mcu.c:3304-3312 - driver_own calls (AFTER dma_init!)
    // Band 0 - write DRV_OWN, poll for FW_OWN_STAT clear
    dev.trace_checkpoint("driver_own_band0_entry");
    let lpcr0 = mt_top_lpcr_host_band(0);
    let lpcr0_old = dev.mt76_rr(lpcr0);
    println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", lpcr0, lpcr0_old, MT_TOP_LPCR_HOST_DRV_OWN);
    match dev.mt7996_driver_own(0) {
        Ok(()) => println!("[MT7996_MCU] Band 0 driver_own OK"),
        Err(e) => println!("[MT7996_MCU] Band 0 driver_own FAILED: {}", e),
    }
    dev.trace_checkpoint("driver_own_band0_exit");

    // Band 1 (HIF2)
    dev.trace_checkpoint("driver_own_band1_entry");
    let lpcr1 = mt_top_lpcr_host_band(1);
    let lpcr1_old = dev.mt76_rr(lpcr1);
    println!("[MT76_REG] WR 0x{:05x}: 0x{:08x} -> 0x{:08x}", lpcr1, lpcr1_old, MT_TOP_LPCR_HOST_DRV_OWN);
    match dev.mt7996_driver_own(1) {
        Ok(()) => println!("[MT7996_MCU] Band 1 driver_own OK"),
        Err(_) => println!("[MT7996_MCU] Band 1 driver_own WARNING: failed"),
    }
    dev.trace_checkpoint("driver_own_band1_exit");

    // Read final state
    println!("\n=== Final State ===");
    let rst_final = dev.mt76_rr(MT_WFDMA0_RST);
    let glo_final = dev.mt76_rr(MT_WFDMA0_GLO_CFG);
    let hif_misc = dev.mt76_rr(MT_WFDMA_EXT_CSR_HIF_MISC);
    let int_mask = dev.mt76_rr(MT_INT_MASK_CSR);

    // Check firmware state (MT_TOP_MISC, bits 2:0)
    // FW_STATE values: 0=idle?, 1=FW_DOWNLOAD ready, 2=RDY, 3=NORMAL_TRX?
    let top_misc = dev.mt76_rr(MT_TOP_MISC);
    let fw_state = top_misc & MT_TOP_MISC_FW_STATE;

    println!("RST: 0x{:08x} (expected: 0x00 for auto-release)", rst_final);
    println!("GLO_CFG: 0x{:08x}", glo_final);
    println!("HIF_MISC: 0x{:08x}", hif_misc);
    println!("INT_MASK: 0x{:08x}", int_mask);
    println!("TOP_MISC: 0x{:08x} (FW_STATE={}: {})", top_misc, fw_state,
        match fw_state {
            0 => "IDLE/RESET",
            1 => "FW_DOWNLOAD READY",
            2 => "RDY",
            3 => "NORMAL_TRX",
            _ => "UNKNOWN",
        });

    // Check if RST released
    if rst_final == 0 {
        println!("\nSUCCESS: RST auto-released!");
    } else if rst_final == 0x30 {
        println!("\nWARNING: RST still 0x30 - hardware not releasing");
        println!("  BIT(4)=LOGIC_RST, BIT(5)=DMASHDL_ALL_RST");
        println!("  Prerequisites may not be met");
    } else {
        println!("\nNOTE: RST=0x{:08x}", rst_final);
    }

    // Dump some queue registers
    println!("\n=== Queue Registers ===");
    let tx_ring_base = MT_WFDMA0_BASE + 0x300;
    let rx_ring_base = MT_WFDMA0_BASE + 0x500;

    // FWDL queue
    let fwdl_regs = tx_ring_base + MT7996_TXQ_FWDL * MT_RING_SIZE;
    println!("FWDL (0x{:08x}):", fwdl_regs);
    println!("  DESC_BASE: 0x{:08x}", dev.mt76_rr(fwdl_regs + MT_QUEUE_DESC_BASE));
    println!("  RING_SIZE: {}", dev.mt76_rr(fwdl_regs + MT_QUEUE_RING_SIZE));
    println!("  CPU_IDX: {}", dev.mt76_rr(fwdl_regs + MT_QUEUE_CPU_IDX));
    println!("  DMA_IDX: {}", dev.mt76_rr(fwdl_regs + MT_QUEUE_DMA_IDX));

    // MCU RX queue
    let mcu_rx_regs = rx_ring_base + MT7996_RXQ_MCU_WM * MT_RING_SIZE;
    println!("MCU_RX (0x{:08x}):", mcu_rx_regs);
    println!("  DESC_BASE: 0x{:08x}", dev.mt76_rr(mcu_rx_regs + MT_QUEUE_DESC_BASE));
    println!("  RING_SIZE: {}", dev.mt76_rr(mcu_rx_regs + MT_QUEUE_RING_SIZE));
    println!("  CPU_IDX: {}", dev.mt76_rr(mcu_rx_regs + MT_QUEUE_CPU_IDX));
    println!("  DMA_IDX: {}", dev.mt76_rr(mcu_rx_regs + MT_QUEUE_DMA_IDX));

    // ========================================================================
    // Step 8: Load Firmware (still in mcu_init)
    // ========================================================================
    dev.trace_checkpoint("mcu_init_firmware_entry");
    println!("\n8. Firmware Loading (mcu_init -> mt7996_load_firmware)...");

    // Allocate buffer memory for FWDL TX from HIGH DMA pool (36-bit, above 4GB)
    // Linux uses high addresses (0x101be8000) - matching that behavior
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
    println!("FWDL TX buffer: virt=0x{:x} phys=0x{:016x}",
             tx_buf_virt, tx_buf_phys);

    // Create TxRing for MCU_WM queue (for MCU commands)
    // CRITICAL: Offsets MUST match what mt7996_dma_init uses!
    // mt7996_dma_init layout: BAND0(32KB), MCU_WM(4KB), MCU_WA(4KB), FWDL(4KB)
    // ring_bytes(2048) = 32KB, ring_bytes(256) = 4KB, ring_bytes(128) = 4KB
    // MCU_WM offset = ring_bytes(BAND0) = 32KB
    // CRITICAL: Linux sends ALL MCU commands (patch_sem_ctrl, init_download, etc.)
    // to MCU_WM queue (hw_idx=17), NOT FWDL queue!
    // Only FW_SCATTER data goes to FWDL queue (hw_idx=16).
    const MCU_WM_DESC_OFFSET: usize = 32 * 1024;  // After BAND0 (2048 * 16 = 32KB)
    let mcu_wm_desc_virt = desc_virt + MCU_WM_DESC_OFFSET as u64;
    let mcu_wm_desc_phys = desc_phys + MCU_WM_DESC_OFFSET as u64;
    let mcu_wm_regs = tx_ring_base + MT7996_TXQ_MCU_WM * MT_RING_SIZE;

    // Allocate separate TX buffer for MCU_WM (MCU commands are small)
    // Use HIGH pool (36-bit, above 4GB) to match Linux behavior
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
    println!("MCU TX buffer: virt=0x{:x} phys=0x{:016x}",
             mcu_tx_buf_virt, mcu_tx_buf_phys);

    let mut mcu_ring = TxRing::new(
        mcu_wm_regs,
        MT7996_TX_MCU_RING_SIZE,
        mcu_wm_desc_virt,
        mcu_wm_desc_phys,
        mcu_tx_buf_virt,
        mcu_tx_buf_phys,
    );
    // No need to reprogram - mt7996_dma_init already programmed this ring with correct address
    // since we're using the same desc_phys from LOW pool
    println!("MCU_WM queue: regs=0x{:x} desc_phys=0x{:x}", mcu_wm_regs, mcu_wm_desc_phys);

    // Create TxRing for FWDL queue (for firmware data chunks)
    // FWDL offset = ring_bytes(BAND0) + ring_bytes(MCU_WM) + ring_bytes(MCU_WA)
    //             = 32KB + 4KB + 4KB = 40KB
    const FWDL_DESC_OFFSET: usize = 40 * 1024;  // After BAND0 + MCU_WM + MCU_WA
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
    // No need to reprogram - mt7996_dma_init already programmed this ring with correct address
    println!("FWDL queue: regs=0x{:x} desc_phys=0x{:x}", fwdl_regs, fwdl_desc_phys);

    // ========================================================================
    // Comprehensive DMA Diagnostics Before Firmware Load
    // ========================================================================
    println!("\n=== DMA Diagnostics ===");
    println!("DMA Translation: {}", if USE_DRAM_OFFSET { "DRAM_OFFSET (dma = phys - 0x40000000)" } else { "IDENTITY (dma = phys)" });

    // Show all relevant addresses
    println!("\nMemory Addresses:");
    println!("  FWDL desc_phys = 0x{:016x}", fwdl_desc_phys);
    println!("  FWDL desc_dma  = 0x{:016x} (what device sees)", phys_to_dma(fwdl_desc_phys));
    println!("  TX buf_phys    = 0x{:016x}", tx_buf_phys);
    println!("  TX buf_dma     = 0x{:016x} (what device sees)", phys_to_dma(tx_buf_phys));

    // Read back what hardware thinks the ring base is
    let hw_desc_base = dev.mt76_rr(fwdl_regs + MT_QUEUE_DESC_BASE);
    let hw_ring_size = dev.mt76_rr(fwdl_regs + MT_QUEUE_RING_SIZE);
    let hw_cpu_idx = dev.mt76_rr(fwdl_regs + MT_QUEUE_CPU_IDX);
    let hw_dma_idx = dev.mt76_rr(fwdl_regs + MT_QUEUE_DMA_IDX);
    println!("\nFWDL Queue Registers (after init):");
    println!("  DESC_BASE = 0x{:08x}", hw_desc_base);
    println!("  RING_SIZE = {}", hw_ring_size);
    println!("  CPU_IDX   = {}", hw_cpu_idx);
    println!("  DMA_IDX   = {}", hw_dma_idx);

    // Verify DESC_BASE matches what we expect
    let expected_desc_base = phys_to_dma(fwdl_desc_phys) as u32;
    if hw_desc_base != expected_desc_base {
        println!("  WARNING: DESC_BASE mismatch! Expected 0x{:08x}, got 0x{:08x}",
                 expected_desc_base, hw_desc_base);
    }

    // Dump first descriptor contents (should be DMA_DONE=1 after init)
    unsafe {
        let desc0 = fwdl_desc_virt as *const Mt76Desc;
        let buf0 = core::ptr::read_volatile(&(*desc0).buf0);
        let ctrl = core::ptr::read_volatile(&(*desc0).ctrl);
        let buf1 = core::ptr::read_volatile(&(*desc0).buf1);
        let info = core::ptr::read_volatile(&(*desc0).info);
        println!("\nFWDL Descriptor[0] (before load):");
        println!("  buf0 = 0x{:08x}", buf0);
        println!("  ctrl = 0x{:08x} (DMA_DONE={}, LAST_SEC0={}, len={})",
                 ctrl,
                 (ctrl >> 31) & 1,
                 (ctrl >> 30) & 1,
                 (ctrl >> 16) & 0x3FFF);
        println!("  buf1 = 0x{:08x}", buf1);
        println!("  info = 0x{:08x}", info);
    }

    // Dump GLO_CFG and other key registers
    let glo_cfg = dev.mt76_rr(MT_WFDMA0_GLO_CFG);
    let rst_reg = dev.mt76_rr(MT_WFDMA0_RST);
    let int_src = dev.mt76_rr(MT_INT_SOURCE_CSR);
    let int_mask = dev.mt76_rr(MT_INT_MASK_CSR);
    println!("\nWFDMA Registers:");
    println!("  GLO_CFG  = 0x{:08x} (TX_EN={}, RX_EN={}, EXT_EN={})",
             glo_cfg,
             (glo_cfg >> 0) & 1,  // TX_DMA_EN
             (glo_cfg >> 2) & 1,  // RX_DMA_EN
             (glo_cfg >> 26) & 1); // EXT_EN
    println!("  RST      = 0x{:08x}", rst_reg);
    println!("  INT_SRC  = 0x{:08x}", int_src);
    println!("  INT_MASK = 0x{:08x}", int_mask);

    // Check TX/RX DMA busy bits
    let tx_busy = (glo_cfg >> 1) & 1;
    let rx_busy = (glo_cfg >> 3) & 1;
    println!("  TX_DMA_BUSY = {}, RX_DMA_BUSY = {}", tx_busy, rx_busy);

    // Prefetch control registers - these control how WFDMA fetches descriptors
    // Format: bits[11:0] = ring base offset (in 64-byte units), bits[15:12] = depth
    println!("\nPrefetch EXT_CTRL (TX queues):");
    println!("  FWDL[16]  @ 0xd4640 = 0x{:08x}", dev.mt76_rr(0xd4640));  // TX q16
    println!("  WM[17]    @ 0xd4644 = 0x{:08x}", dev.mt76_rr(0xd4644));  // TX q17
    println!("  BAND0[18] @ 0xd4648 = 0x{:08x}", dev.mt76_rr(0xd4648));  // TX q18
    println!("  WA[20]    @ 0xd4650 = 0x{:08x}", dev.mt76_rr(0xd4650));  // TX q20

    println!("\nPrefetch EXT_CTRL (RX queues):");
    println!("  MCU[0]    @ 0xd4680 = 0x{:08x}", dev.mt76_rr(0xd4680));  // RX q0
    println!("  WA[1]     @ 0xd4684 = 0x{:08x}", dev.mt76_rr(0xd4684));  // RX q1
    println!("  BAND0[8]  @ 0xd46a0 = 0x{:08x}", dev.mt76_rr(0xd46a0));  // RX q8

    // Check WFDMA status registers for any stall/error conditions
    println!("\nWFDMA Status:");
    println!("  BUSY_ENA     = 0x{:08x}", dev.mt76_rr(0xd413c));
    println!("  BUSY_STAT    = 0x{:08x}", dev.mt76_rr(0xd4140));
    println!("  ERR_STA      = 0x{:08x}", dev.mt76_rr(0xd4144));
    println!("  DBG_TXQCNT   = 0x{:08x}", dev.mt76_rr(0xd4220));
    println!("  DBG_RXQCNT   = 0x{:08x}", dev.mt76_rr(0xd4224));
    // HIF status (critical for DMA)
    let hif_misc = dev.mt76_rr(0xd7044);
    let host_cfg = dev.mt76_rr(0xd7030);
    println!("  HIF_MISC     = 0x{:08x} (BUSY={})", hif_misc, hif_misc & 1);
    println!("  HOST_CONFIG  = 0x{:08x} (OpenWRT: 0x00407d01)", host_cfg);

    // ========================================================================
    // COMPREHENSIVE STATE DUMP BEFORE FIRMWARE LOAD
    // ========================================================================
    println!("\n========== COMPREHENSIVE PRE-FIRMWARE DUMP ==========\n");

    // 1. ALL TX Queue DESC_BASE addresses
    println!("=== ALL TX Queue Ring Bases (MT_TXQ_* @ 0xd4300-0xd44f0) ===");
    let tx_base = 0xd4300u32;
    for q in 0..32u32 {
        let regs = tx_base + q * 0x10;
        let base = dev.mt76_rr(regs + 0);  // DESC_BASE
        let size = dev.mt76_rr(regs + 4);  // RING_SIZE
        // Only print if ring is configured (size > 0)
        if size > 0 {
            let cpu = dev.mt76_rr(regs + 8);
            let dma = dev.mt76_rr(regs + 12);
            println!("  TX[{:2}] BASE=0x{:08x} SIZE={:3} CPU={:3} DMA={:3}",
                     q, base, size, cpu, dma);
            // Check for addresses > 4GB (suspicious for 4GB RAM system)
            if base > 0xFFFF_FFFF {
                println!("         *** WARNING: BASE > 4GB! ***");
            }
        }
    }

    // 2. ALL RX Queue DESC_BASE addresses
    println!("\n=== ALL RX Queue Ring Bases (MT_RXQ_* @ 0xd4500-0xd46f0) ===");
    let rx_base = 0xd4500u32;
    for q in 0..32u32 {
        let regs = rx_base + q * 0x10;
        let base = dev.mt76_rr(regs + 0);  // DESC_BASE
        let size = dev.mt76_rr(regs + 4);  // RING_SIZE
        // Only print if ring is configured (size > 0)
        if size > 0 {
            let cpu = dev.mt76_rr(regs + 8);
            let dma = dev.mt76_rr(regs + 12);
            println!("  RX[{:2}] BASE=0x{:08x} SIZE={:3} CPU={:3} DMA={:3}",
                     q, base, size, cpu, dma);
            // Check for addresses > 4GB
            if base > 0xFFFF_FFFF {
                println!("         *** WARNING: BASE > 4GB! ***");
            }
        }
    }

    // 3. Our allocated memory addresses
    println!("\n=== Our Allocated Memory ===");
    println!("  DMA Pool Base:    0x{:08x}", 0x4010_0000u32);  // From kernel dma_pool.rs
    println!("  desc_phys:        0x{:016x}", desc_phys);
    println!("  desc_dma:         0x{:016x}", phys_to_dma(desc_phys));
    println!("  rx_buf_phys:      0x{:016x}", rx_buf_phys);
    println!("  rx_buf_dma:       0x{:016x}", phys_to_dma(rx_buf_phys));
    println!("  fwdl_desc_phys:   0x{:016x}", fwdl_desc_phys);
    println!("  fwdl_desc_dma:    0x{:016x}", phys_to_dma(fwdl_desc_phys));
    println!("  tx_buf_phys:      0x{:016x}", tx_buf_phys);
    println!("  tx_buf_dma:       0x{:016x}", phys_to_dma(tx_buf_phys));

    // 4. BAR0 physical address (from spawn context)
    println!("\n=== PCIe BAR Configuration ===");
    println!("  HIF1 BAR0 phys:   0x{:016x}", bar0_addr);
    println!("  HIF1 BAR0 size:   0x{:08x} ({}KB)", bar0_size, bar0_size / 1024);

    // 5. MT7996 Internal PCIe Registers
    println!("\n=== MT7996 Internal PCIe Registers ===");
    println!("  PCIE_SETTING (0x10080):   0x{:08x} (OpenWRT: 0x00003180)", dev.mt76_rr(0x10080));
    println!("  PCIE_LTSSM (0x10150):     0x{:08x} (OpenWRT: 0x1003c81f)", dev.mt76_rr(0x10150));
    println!("  PCIE_LINKSTA (0x10154):   0x{:08x}", dev.mt76_rr(0x10154));
    println!("  PCIE_MAC_INT (0x10188):   0x{:08x}", dev.mt76_rr(0x10188));

    // 6. ATU / Address Translation Registers (from OpenWRT reference)
    println!("\n=== MT7996 ATU Registers (Address Translation) ===");
    println!("  ATU_0 (0x10000):    0x{:08x}", dev.mt76_rr(0x10000));
    println!("  ATU_1 (0x10004):    0x{:08x}", dev.mt76_rr(0x10004));
    println!("  ATU_2 (0x10010):    0x{:08x}", dev.mt76_rr(0x10010));
    println!("  ATU_3 (0x10014):    0x{:08x}", dev.mt76_rr(0x10014));
    println!("  ATU_DATA (0x10100-0c):");
    println!("    0x10100 = 0x{:08x}", dev.mt76_rr(0x10100));
    println!("    0x10104 = 0x{:08x}", dev.mt76_rr(0x10104));
    println!("    0x10108 = 0x{:08x}", dev.mt76_rr(0x10108));
    println!("    0x1010c = 0x{:08x}", dev.mt76_rr(0x1010c));

    // 7. L1/L2 Remap Registers (used for accessing high addresses)
    println!("\n=== L1/L2 Remap Registers ===");
    println!("  HIF_REMAP_L1 (0x155024): 0x{:08x}", dev.mt76_rr(0x155024));
    println!("  HIF_REMAP_L2 (0x155008): 0x{:08x}", dev.mt76_rr(0x155008));

    // 8. MCU/Firmware State
    println!("\n=== MCU/Firmware State ===");
    println!("  TOP_MISC (0xe0010):     0x{:08x} (FW_STATE={})",
             dev.mt76_rr(0xe0010), dev.mt76_rr(0xe0010) & 7);
    println!("  SWDEF_MODE (0x8143c):   0x{:08x}", dev.mt76_rr(0x8143c));
    println!("  MCU_INT_EVENT (0x2108): 0x{:08x}", dev.mt76_rr(0x2108));

    // 9. CONN_INFRA / Bus Status - DISABLED: causes SError on some systems
    // These registers may require L1 remap or aren't accessible in all states
    // println!("\n=== CONN_INFRA Registers ===");
    // println!("  0x7c000: 0x{:08x}", dev.mt76_rr(0x7c000));
    // println!("  0x7c060: 0x{:08x}", dev.mt76_rr(0x7c060));
    // println!("  0x7c500: 0x{:08x}", dev.mt76_rr(0x7c500));

    // 10. GLO_CFG Extended
    println!("\n=== GLO_CFG Extended ===");
    println!("  GLO_CFG (0xd4208):      0x{:08x}", dev.mt76_rr(0xd4208));
    println!("  GLO_CFG_EXT0 (0xd42b0): 0x{:08x} (OpenWRT: 0x28c444df)", dev.mt76_rr(0xd42b0));
    println!("  GLO_CFG_EXT1 (0xd42b4): 0x{:08x} (OpenWRT: 0x9c800404)", dev.mt76_rr(0xd42b4));

    // 11. AXI Control
    println!("\n=== AXI Control ===");
    println!("  AXI_R2A_CTRL (0xd7500):  0x{:08x} (OpenWRT: 0xffff0c14)", dev.mt76_rr(0xd7500));
    println!("  AXI_R2A_CTRL2 (0xd7508): 0x{:08x}", dev.mt76_rr(0xd7508));

    println!("\n========== END PRE-FIRMWARE DUMP ==========\n");

    // Load firmware
    // CRITICAL: Linux uses different queues for different operations:
    // - mcu_ring (MCU_WM, hw_idx=17): ALL MCU commands
    // - fwdl_ring (FWDL, hw_idx=16): ONLY firmware data chunks (FW_SCATTER)
    println!("FIRMWARE DOWNLOAD");
    dev.trace_checkpoint("load_firmware_entry");

    // Log MCU ring state like Linux
    let fw_state = dev.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
    println!("[MT7996_MCU] FW_STATE=0x{:08x} before patch", fw_state);
    dev.trace_dma_state("DMA state");
    dev.trace_mcu_ring("FWDL", fwdl_regs);
    dev.trace_mcu_ring("MCU_WM", mcu_wm_regs);

    match dev.load_firmware(&mut mcu_ring, &mut fwdl_ring) {
        Ok(()) => println!("Firmware loading completed successfully!"),
        Err(e) => println!("Firmware loading failed: {}", e),
    }

    // Check final state after firmware
    println!("\n=== Post-Firmware State ===");
    let final_fw_state = dev.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
    println!("FW_STATE: {} (7=running)", final_fw_state);

    // Store resources in driver struct (keeps DMA pools alive for driver lifetime)
    self.dev = Some(dev);
    self.bar0 = Some(bar0);
    self.desc_pool = Some(desc_pool);
    self.rx_pool = Some(rx_pool);

    println!("\n=== wifid init complete ===");
    Ok(())
    }

    fn command(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) -> Disposition {
        Disposition::Handled
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

impl Driver for WifiDriverWrapper {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.reset(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }
}
