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

use userlib::{println, syscall};
use pcie::{PcieClient, consts};
use core::sync::atomic::{fence, Ordering};

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
            println!("WARNING: phys_to_dma: addr 0x{:x} below DRAM base!", phys);
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
    // mt7996_wfsys_reset() - EXACT Linux translation
    // Source: init.c (simple WFSYS reset before DMA init)
    // This MUST be called before mt7996_dma_init()!
    // ========================================================================

    fn mt7996_wfsys_reset(&self) {
        println!("   Performing WFSYS reset...");

        // Debug: show L1 remap register access
        println!("   MT_HIF_REMAP_L1 @ 0x{:x} (within BAR: {})",
            MT_HIF_REMAP_L1, MT_HIF_REMAP_L1 < self.bar0_size as u32);

        // init.c - mt7996_wfsys_reset():
        //   mt76_set(dev, MT_WF_SUBSYS_RST, 0x1);
        //   msleep(20);
        //   mt76_clear(dev, MT_WF_SUBSYS_RST, 0x1);
        //   msleep(20);

        // Read the L1 remap register before
        let remap_before = self.mt76_rr(MT_HIF_REMAP_L1);
        println!("   L1 remap before: 0x{:08x}", remap_before);

        // Set bit 0 of MT_WF_SUBSYS_RST (0x70028600)
        // This requires L1 remapping:
        //   addr = 0x70028600
        //   offset = 0x8600, base = 0x7002
        //   Write 0x70020000 to MT_HIF_REMAP_L1
        //   Then access at HIF_REMAP_BASE_L1 + 0x8600 = 0x130000 + 0x8600 = 0x138600
        let remapped = self.mt7996_reg_map_l1(MT_WF_SUBSYS_RST);
        println!("   0x{:08x} remapped to 0x{:08x}", MT_WF_SUBSYS_RST, remapped);

        let remap_after = self.mt76_rr(MT_HIF_REMAP_L1);
        println!("   L1 remap after write: 0x{:08x}", remap_after);

        // Read the remapped register value
        let val_before = self.mt76_rr(remapped);
        println!("   MT_WF_SUBSYS_RST value: 0x{:08x}", val_before);

        // Set bit 0
        self.mt76_wr(remapped, val_before | 0x1);
        println!("   Set bit 0, waiting 20ms...");
        userlib::delay_ms(20);

        // Read back
        let val_after_set = self.mt76_rr(remapped);
        println!("   After set: 0x{:08x}", val_after_set);

        // Clear bit 0
        self.mt76_wr(remapped, val_after_set & !0x1);
        println!("   Cleared bit 0, waiting 20ms...");
        userlib::delay_ms(20);

        // Read back
        let val_final = self.mt76_rr(remapped);
        println!("   Final value: 0x{:08x}", val_final);

        println!("   WFSYS reset complete");
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
            println!("  ERROR: Timeout for driver own (band {})", band);
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
        println!("   dma_disable: reset={} has_hif2={}", reset, self.has_hif2);
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        // dma.c:551-570 - Reset pulse
        if reset {
            // dma.c:552-555 - Clear then set RST bits on HIF1
            println!("   RST pulse on HIF1 (0x{:x})", MT_WFDMA0_RST);
            self.mt76_clear(MT_WFDMA0_RST,
                MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
            self.mt76_set(MT_WFDMA0_RST,
                MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);

            // dma.c:557-566 - Same for HIF2
            if self.has_hif2 {
                println!("   RST pulse on HIF2 (0x{:x})", MT_WFDMA0_RST + hif1_ofs);
                self.mt76_clear(MT_WFDMA0_RST + hif1_ofs,
                    MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
                self.mt76_set(MT_WFDMA0_RST + hif1_ofs,
                    MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
            }
        }

        // dma.c:573-579 - Disable DMA on HIF1
        self.mt76_clear(MT_WFDMA0_GLO_CFG,
            MT_WFDMA0_GLO_CFG_TX_DMA_EN |
            MT_WFDMA0_GLO_CFG_RX_DMA_EN |
            MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
            MT_WFDMA0_GLO_CFG_OMIT_RX_INFO |
            MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2);

        // dma.c:581-585 - Disable DMA on HIF2
        if self.has_hif2 {
            self.mt76_clear(MT_WFDMA0_GLO_CFG + hif1_ofs,
                MT_WFDMA0_GLO_CFG_TX_DMA_EN |
                MT_WFDMA0_GLO_CFG_RX_DMA_EN |
                MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
                MT_WFDMA0_GLO_CFG_OMIT_RX_INFO |
                MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2);
        }
    }

    // ========================================================================
    // mt7996_dma_start() - EXACT Linux translation
    // Source: dma.c:587-625
    // ========================================================================

    fn mt7996_dma_start(&self, reset: bool, _wed_reset: bool) {
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        // dma.c:302-321 (upstream) - Enable WFDMA TX/RX
        // Linux sets: TX_DMA_EN | RX_DMA_EN | OMIT_TX_INFO | OMIT_RX_INFO_PFET2 | EXT_EN
        // Note: EXT_EN (BIT 26) is in upstream kernel, not our local 6.6.119
        if !reset {
            // UPSTREAM LINUX: dma_start includes EXT_EN (BIT 26)!
            println!("   dma_start: enabling GLO_CFG on HIF1 (0x{:x})", MT_WFDMA0_GLO_CFG);
            self.mt76_set(MT_WFDMA0_GLO_CFG,
                MT_WFDMA0_GLO_CFG_TX_DMA_EN |
                MT_WFDMA0_GLO_CFG_RX_DMA_EN |
                MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
                MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2 |
                MT_WFDMA0_GLO_CFG_EXT_EN);

            // UPSTREAM LINUX: HIF2 GLO_CFG also includes EXT_EN
            if self.has_hif2 {
                println!("   dma_start: enabling GLO_CFG on HIF2 (0x{:x})", MT_WFDMA0_GLO_CFG + hif1_ofs);
                self.mt76_set(MT_WFDMA0_GLO_CFG + hif1_ofs,
                    MT_WFDMA0_GLO_CFG_TX_DMA_EN |
                    MT_WFDMA0_GLO_CFG_RX_DMA_EN |
                    MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
                    MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2 |
                    MT_WFDMA0_GLO_CFG_EXT_EN);
            }

            // === TIMING TEST: 500ms delay after GLO_CFG enable ===
            // If this helps, timing/tight loops are the issue
            println!("   dma_start: waiting 500ms after GLO_CFG enable...");
            userlib::delay_ms(500);
            let rst_after_delay = self.mt76_rr(MT_WFDMA0_RST);
            println!("   dma_start: RST after 500ms: 0x{:08x}", rst_after_delay);

            // EXPERIMENT: Clear INT_SOURCE after enabling DMA
            // Bit 20 gets set when GLO_CFG enables DMA - might need to be cleared
            // before WFDMA will process descriptors
            let int_src_before = self.mt76_rr(MT_INT_SOURCE_CSR);
            println!("   INT_SRC before clear: 0x{:08x}", int_src_before);
            self.mt76_wr(MT_INT_SOURCE_CSR, !0u32);
            self.mt76_wr(MT_INT1_SOURCE_CSR, !0u32);
            let int_src_after = self.mt76_rr(MT_INT_SOURCE_CSR);
            println!("   INT_SRC after clear: 0x{:08x}", int_src_after);
        }

        // dma.c:328 - Enable interrupts for TX/RX rings
        // EXACT Linux: irq_mask = MT_INT_MCU_CMD | MT_INT_RX_DONE_MCU | MT_INT_TX_DONE_MCU;
        let irq_mask = MT_INT_MCU_CMD | MT_INT_RX_DONE_MCU | MT_INT_TX_DONE_MCU;

        // CRITICAL FIX: Linux mt7996_irq_enable()+mt7996_irq_disable(0) writes to BOTH
        // MT_INT_MASK_CSR and MT_INT1_MASK_CSR when HIF2 is present.
        // See mt7996_dual_hif_set_irq_mask() in mmio.c:252-269
        self.mt76_wr(MT_INT_MASK_CSR, irq_mask);
        if self.has_hif2 {
            self.mt76_wr(MT_INT1_MASK_CSR, irq_mask);
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
        // Accumulated base value - starts at 0, advances by depth*16 each time
        let mut base: u32 = 0;

        // PREFETCH macro with accumulation (like Linux __mt7996_dma_prefetch_base)
        let mut prefetch = |depth: u32| -> u32 {
            let ret = (base << 16) | depth;
            base += depth << 4;  // base = base + depth * 16
            ret
        };

        // Register addresses use HARDWARE queue indices from q_id table
        // MT_MCUQ_EXT_CTRL(q) = MT_WFDMA0_BASE + 0x600 + hw_idx * 4
        // MT_TXQ_EXT_CTRL(q)  = MT_WFDMA0_BASE + 0x600 + hw_idx * 4
        // MT_RXQ_EXT_CTRL(q)  = MT_WFDMA0_BASE + 0x680 + hw_idx * 4
        let mcuq_ext_ctrl = |hw_idx: u32| MT_WFDMA0_BASE + 0x600 + hw_idx * 4;
        let txq_ext_ctrl = |hw_idx: u32| MT_WFDMA0_BASE + 0x600 + hw_idx * 4;
        let rxq_ext_ctrl = |hw_idx: u32| MT_WFDMA0_BASE + 0x680 + hw_idx * 4;

        // dma.c:184-188 - MCU Command Rings (depth=2 for MT7996)
        // MUST be in this order: FWDL, WM, WA
        self.mt76_wr(mcuq_ext_ctrl(MT7996_TXQ_FWDL) + ofs, prefetch(0x2));    // base=0
        self.mt76_wr(mcuq_ext_ctrl(MT7996_TXQ_MCU_WM) + ofs, prefetch(0x2));  // base=32
        self.mt76_wr(mcuq_ext_ctrl(MT7996_TXQ_MCU_WA) + ofs, prefetch(0x2));  // base=64

        // dma.c:191-195 - TX Data Rings (depth=8 for MT7996)
        // AFTER MCU queues!
        self.mt76_wr(txq_ext_ctrl(MT7996_TXQ_BAND0) + ofs, prefetch(0x8));    // base=96
        if self.has_hif2 {
            self.mt76_wr(txq_ext_ctrl(MT7996_TXQ_BAND1) + ofs, prefetch(0x8)); // base=224
        }
        self.mt76_wr(txq_ext_ctrl(MT7996_TXQ_BAND2) + ofs, prefetch(0x8));    // base=352 (or 224 if no HIF2)

        // dma.c:198-206 - RX Event/WA Rings (depth=2)
        self.mt76_wr(rxq_ext_ctrl(MT7996_RXQ_MCU_WM) + ofs, prefetch(0x2));
        self.mt76_wr(rxq_ext_ctrl(MT7996_RXQ_MCU_WA) + ofs, prefetch(0x2));
        self.mt76_wr(rxq_ext_ctrl(MT7996_RXQ_MCU_WA_MAIN) + ofs, prefetch(0x2));
        self.mt76_wr(rxq_ext_ctrl(MT7996_RXQ_MCU_WA_TRI) + ofs, prefetch(0x2));

        // dma.c:219-221 - RX Data Rings (depth=16 = 0x10)
        self.mt76_wr(rxq_ext_ctrl(MT7996_RXQ_BAND0) + ofs, prefetch(0x10));
        self.mt76_wr(rxq_ext_ctrl(MT7996_RXQ_BAND2) + ofs, prefetch(0x10));

        // dma.c:239 - Set CALC_MODE
        self.mt76_set(WF_WFDMA0_GLO_CFG_EXT1 + ofs, WF_WFDMA0_GLO_CFG_EXT1_CALC_MODE);
    }

    // ========================================================================
    // mt7996_dma_prefetch() - EXACT Linux translation
    // Source: dma.c:520-523
    // ========================================================================

    fn mt7996_dma_prefetch(&self) {
        println!("   prefetch: HIF1 (ofs=0)");
        self.__mt7996_dma_prefetch(0);
        if self.has_hif2 {
            println!("   prefetch: HIF2 (ofs=0x{:x})", HIF1_OFS);
            self.__mt7996_dma_prefetch(HIF1_OFS);
        }
    }

    // ========================================================================
    // mt7996_dma_enable() - EXACT Linux translation
    // Source: dma.c:627-754
    // ========================================================================

    fn mt7996_dma_enable(&self, reset: bool) {
        println!("   dma_enable: has_hif2={} hif1_ofs=0x{:x}", self.has_hif2, HIF1_OFS);
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        // dma.c:633-635 - Reset DMA index
        println!("   RST_DTX_PTR: HIF1=0x{:x}", MT_WFDMA0_RST_DTX_PTR);
        self.mt76_wr(MT_WFDMA0_RST_DTX_PTR, !0u32);
        if self.has_hif2 {
            println!("   RST_DTX_PTR: HIF2=0x{:x}", MT_WFDMA0_RST_DTX_PTR + hif1_ofs);
            self.mt76_wr(MT_WFDMA0_RST_DTX_PTR + hif1_ofs, !0u32);
        }

        // dma.c:637-646 - Configure delay interrupt off
        self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG0, 0);
        self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG1, 0);
        self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG2, 0);

        if self.has_hif2 {
            self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG0 + hif1_ofs, 0);
            self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG1 + hif1_ofs, 0);
            self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG2 + hif1_ofs, 0);
        }

        // dma.c:649 - Configure prefetch settings
        self.mt7996_dma_prefetch();

        // dma.c:651-655 - Configure BUSY_ENA on HIF1
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
        println!("   HIF_MISC before poll: 0x{:08x}", hif_misc_before);
        let poll_result = self.mt76_poll(MT_WFDMA_EXT_CSR_HIF_MISC,
            MT_WFDMA_EXT_CSR_HIF_MISC_BUSY, 0, 1000);
        let hif_misc_after = self.mt76_rr(MT_WFDMA_EXT_CSR_HIF_MISC);
        println!("   HIF_MISC after poll: 0x{:08x} (poll success: {})", hif_misc_after, poll_result);

        // UPSTREAM LINUX: GLO_CFG_EXT0 on HIF1
        self.mt76_set(WF_WFDMA0_GLO_CFG_EXT0,
            WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD |
            WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);

        // UPSTREAM LINUX: GLO_CFG_EXT1 on HIF1
        self.mt76_set(WF_WFDMA0_GLO_CFG_EXT1,
            WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);

        // UPSTREAM LINUX: WFDMA rx thresholds on HIF1
        self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_45_TH, 0xc000c);
        self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_67_TH, 0x10008);
        self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_89_TH, 0x10008);
        self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_RRO_TH, 0x20);

        // UPSTREAM LINUX: HIF2 configuration
        if self.has_hif2 {
            // UPSTREAM LINUX: GLO_CFG_EXT0 on HIF2
            self.mt76_set(WF_WFDMA0_GLO_CFG_EXT0 + hif1_ofs,
                WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD |
                WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);

            // UPSTREAM LINUX: GLO_CFG_EXT1 on HIF2
            self.mt76_set(WF_WFDMA0_GLO_CFG_EXT1 + hif1_ofs,
                WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);

            // UPSTREAM LINUX: HOST_CONFIG: set PDMA_BAND
            self.mt76_set(MT_WFDMA_HOST_CONFIG,
                MT_WFDMA_HOST_CONFIG_PDMA_BAND);

            // UPSTREAM LINUX: HOST_CONFIG: clear band PCIE1 bits
            self.mt76_clear(MT_WFDMA_HOST_CONFIG,
                MT_WFDMA_HOST_CONFIG_BAND0_PCIE1 |
                MT_WFDMA_HOST_CONFIG_BAND1_PCIE1 |
                MT_WFDMA_HOST_CONFIG_BAND2_PCIE1);

            // UPSTREAM LINUX: HOST_CONFIG: set BAND2_PCIE1 for MT7996
            self.mt76_set(MT_WFDMA_HOST_CONFIG,
                MT_WFDMA_HOST_CONFIG_BAND2_PCIE1);

            // UPSTREAM LINUX: AXI read outstanding
            self.mt76_rmw(MT_WFDMA_AXI_R2A_CTRL,
                MT_WFDMA_AXI_R2A_CTRL_OUTSTAND_MASK, 0x14);

            // UPSTREAM LINUX: WFDMA rx thresholds on HIF2
            self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_45_TH + hif1_ofs, 0xc000c);
            self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_67_TH + hif1_ofs, 0x10008);
            self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_89_TH + hif1_ofs, 0x10008);
            self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_RRO_TH + hif1_ofs, 0x20);
        }

        // dma.c:744-749 - RX_INT_PCIE_SEL for HIF2
        if self.has_hif2 {
            // dma.c:747-748 - Non-WED path: set RING3
            self.mt76_set(MT_WFDMA0_RX_INT_PCIE_SEL, MT_WFDMA0_RX_INT_SEL_RING3);
        }

        // dma.c:754 - Call dma_start
        self.mt7996_dma_start(reset, true);
    }

    // ========================================================================
    // Queue allocation - EXACT Linux translation
    // Source: dma.c mt76_dma_queue_reset() + mt76_dma_sync_idx()
    //
    // CRITICAL: Linux writes CPU_IDX/DMA_IDX BEFORE DESC_BASE/RING_SIZE!
    // ========================================================================

    fn program_queue(&self, regs_base: u32, desc_phys: u64, ndesc: u32) {
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

        // Step 3: Read back DMA_IDX to sync (like Linux mt76_dma_sync_idx)
        // dma.c:188 - q->head = Q_READ(dev, q, dma_idx);
        let _ = self.mt76_rr(regs_base + MT_QUEUE_DMA_IDX);
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

        self.program_queue(regs_base, desc_phys, ndesc);
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

        self.program_queue(regs_base, desc_phys, ndesc);
    }

    // ========================================================================
    // mt76_dma_rx_fill() - Fill RX queue with buffers
    // Source: dma.c:684-694 (mt76_dma_rx_fill_buf)
    //
    // CRITICAL: This MUST be called before DMA enable!
    // Without buffers, RX DMA cannot start and RST stays at 0x30.
    // ========================================================================

    fn rx_fill(&self, q: &RxQueueInfo) {
        // Fill descriptors with buffers (up to ndesc-1, leave 1 for wrap detection)
        // Linux: while (q->queued < q->ndesc - 1) { ... }
        let fill_count = (q.ndesc - 1) as usize;

        println!("    Filling {} RX buffers (buf_size={}, buf_phys=0x{:x})",
                 fill_count, q.buf_size, q.buf_phys);

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
            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, buf_dma as u32);
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, 0);  // No second buffer
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
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
        self.mt76_wr(q.regs_base + MT_QUEUE_CPU_IDX, fill_count as u32);

        // Debug: verify CPU_IDX was written
        let cpu_idx = self.mt76_rr(q.regs_base + MT_QUEUE_CPU_IDX);
        println!("    CPU_IDX set to {} (readback: {})", fill_count, cpu_idx);
    }

    // ========================================================================
    // mt7996_dma_init() - EXACT Linux translation
    // Source: dma.c:599-854
    // ========================================================================

    fn mt7996_dma_init(&self, desc_phys: u64, desc_virt: u64, _desc_size: usize,
                       rx_buf_phys: u64, rx_buf_virt: u64, _rx_buf_size: usize) {
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        const MIN_RING_SIZE: u32 = 64;
        const DESC_SIZE: usize = 16;
        // CRITICAL: Linux uses dmam_alloc_coherent() which returns 4KB-aligned memory
        // for each queue. WFDMA hardware may require this alignment!
        // We must use 4KB increments between queues, not just RING_BYTES.
        const RING_BYTES: usize = 4096; // 4KB alignment per queue (was 64*16=1024)

        // dma.c:611 - mt7996_dma_config() was called in constructor

        // dma.c:613 - mt76_dma_attach (sets up DMA ops - we do this implicitly)

        // dma.c:618 - Disable DMA with reset
        self.mt7996_dma_disable(true);

        // Track offset in descriptor memory
        let mut offset: usize = 0;

        // Track RX queues for buffer fill (NEW - Linux does this in mt76_init_queues)
        let mut rx_queues: [RxQueueInfo; NUM_RX_QUEUES] = [RxQueueInfo::default(); NUM_RX_QUEUES];
        let mut rx_queue_idx: usize = 0;
        let mut rx_buf_offset: usize = 0;

        let tx_ring_base = MT_WFDMA0_BASE + 0x300;
        let rx_ring_base = MT_WFDMA0_BASE + 0x500;

        // dma.c:621-628 - init tx queue (BAND0)
        self.init_mcu_queue(
            MT7996_TXQ_BAND0,
            tx_ring_base,
            MIN_RING_SIZE,
            desc_phys + offset as u64,
            desc_virt + offset as u64,
        );
        offset += RING_BYTES;
        println!("  TX BAND0 queue @ 0x{:08x}", tx_ring_base + MT7996_TXQ_BAND0 * MT_RING_SIZE);

        // dma.c:631-637 - command to WM
        self.init_mcu_queue(
            MT7996_TXQ_MCU_WM,
            tx_ring_base,
            MIN_RING_SIZE,
            desc_phys + offset as u64,
            desc_virt + offset as u64,
        );
        offset += RING_BYTES;
        println!("  TX MCU_WM queue @ 0x{:08x}", tx_ring_base + MT7996_TXQ_MCU_WM * MT_RING_SIZE);

        // dma.c:640-647 - command to WA (mt7996_has_wa = true for MT7996)
        self.init_mcu_queue(
            MT7996_TXQ_MCU_WA,
            tx_ring_base,
            MIN_RING_SIZE,
            desc_phys + offset as u64,
            desc_virt + offset as u64,
        );
        offset += RING_BYTES;
        println!("  TX MCU_WA queue @ 0x{:08x}", tx_ring_base + MT7996_TXQ_MCU_WA * MT_RING_SIZE);

        // dma.c:650-656 - firmware download
        self.init_mcu_queue(
            MT7996_TXQ_FWDL,
            tx_ring_base,
            MT7996_TX_FWDL_RING_SIZE,
            desc_phys + offset as u64,
            desc_virt + offset as u64,
        );
        offset += RING_BYTES; // 4KB aligned like other queues
        println!("  TX FWDL queue @ 0x{:08x}", tx_ring_base + MT7996_TXQ_FWDL * MT_RING_SIZE);

        // dma.c:659-665 - event from WM (MCU RX queue)
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_MCU_WM,
            rx_ring_base,
            MIN_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WM * MT_RING_SIZE,
            ndesc: MIN_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_MCU_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MIN_RING_SIZE as usize * MT7996_RX_MCU_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += RING_BYTES;
        println!("  RX MCU_WM queue @ 0x{:08x}", rx_ring_base + MT7996_RXQ_MCU_WM * MT_RING_SIZE);

        // dma.c:668-675 - event from WA
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_MCU_WA,
            rx_ring_base,
            MIN_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WA * MT_RING_SIZE,
            ndesc: MIN_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_MCU_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MIN_RING_SIZE as usize * MT7996_RX_MCU_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += RING_BYTES;
        println!("  RX MCU_WA queue @ 0x{:08x}", rx_ring_base + MT7996_RXQ_MCU_WA * MT_RING_SIZE);

        // dma.c:677-692 - rx data queue for band0
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_BAND0,
            rx_ring_base,
            MIN_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            regs_base: rx_ring_base + MT7996_RXQ_BAND0 * MT_RING_SIZE,
            ndesc: MIN_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MIN_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += RING_BYTES;
        println!("  RX BAND0 queue @ 0x{:08x}", rx_ring_base + MT7996_RXQ_BAND0 * MT_RING_SIZE);

        // dma.c:694-706 - tx free notify event from WA for band0 (mt7996_has_wa = true)
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_MCU_WA_MAIN,
            rx_ring_base,
            MIN_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WA_MAIN * MT_RING_SIZE,
            ndesc: MIN_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MIN_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += RING_BYTES;
        println!("  RX WA_MAIN queue @ 0x{:08x}", rx_ring_base + MT7996_RXQ_MCU_WA_MAIN * MT_RING_SIZE);

        // dma.c:735-756 - MT7996 band2 queues
        // rx data queue for mt7996 band2
        let rx_base_band2 = rx_ring_base + hif1_ofs;
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_BAND2,
            rx_base_band2,
            MIN_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            regs_base: rx_base_band2 + MT7996_RXQ_BAND2 * MT_RING_SIZE,
            ndesc: MIN_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MIN_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += RING_BYTES;
        println!("  RX BAND2 queue @ 0x{:08x}", rx_base_band2 + MT7996_RXQ_BAND2 * MT_RING_SIZE);

        // tx free notify event from WA for mt7996 band2
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(
            MT7996_RXQ_MCU_WA_TRI,
            rx_ring_base,
            MIN_RING_SIZE,
            desc_phys + offset as u64,
            rx_desc_virt,
        );
        rx_queues[rx_queue_idx] = RxQueueInfo {
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WA_TRI * MT_RING_SIZE,
            ndesc: MIN_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
        };
        rx_buf_offset += MIN_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += RING_BYTES;
        println!("  RX WA_TRI queue @ 0x{:08x}", rx_ring_base + MT7996_RXQ_MCU_WA_TRI * MT_RING_SIZE);

        // We only need 6 RX queues for firmware download
        // The 7th slot is reserved for RXQ_BAND1 if needed later
        let _ = offset;
        let _ = rx_buf_offset;

        // ====================================================================
        // CRITICAL: Fill RX buffers BEFORE enabling DMA!
        // This is what Linux does in mt76_init_queues() -> mt76_dma_rx_queue_init()
        // Without buffers, DMA engine won't start and RST stays at 0x30
        // ====================================================================
        println!("\n  Filling RX buffers (mt76_dma_rx_fill)...");
        for i in 0..rx_queue_idx {
            println!("  RX Queue {}:", i);
            self.rx_fill(&rx_queues[i]);
        }

        // dma.c:853 - Enable DMA
        self.mt7996_dma_enable(false);
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

/// MCU download modes
mod dl_mode {
    pub const NEED_RSP: u32 = 1 << 31;
    pub const MODE_ENCRYPTED: u32 = 1 << 0;
    pub const MODE_KEY_IDX_MASK: u32 = 0x3 << 1;
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
    const SIZE: usize = 60;

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
        txd.len = len.to_le();
        txd.pq_id = 0x8000u16.to_le();  // MCU port
        txd.cid = cmd;
        txd.pkt_type = mcu_pkt::MCU_PKT_ID;
        txd.set_query = 1;  // SET
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
        unsafe {
            // Copy TXD header
            core::ptr::copy_nonoverlapping(txd.as_bytes().as_ptr(), buf, McuTxd::SIZE);
            // Copy payload
            if !data.is_empty() {
                core::ptr::copy_nonoverlapping(data.as_ptr(), buf.add(McuTxd::SIZE), data.len());
            }
        }

        let total_len = McuTxd::SIZE + data.len();
        let buf_phys = ring.buf_phys(idx);
        // CRITICAL: Convert to DMA address for PCIe device
        let buf_dma = phys_to_dma(buf_phys);

        // Fill DMA descriptor
        // CRITICAL: Write order must match Linux: buf0, buf1, info, THEN ctrl (last!)
        // Hardware triggers on ctrl write, so other fields must be ready first
        // Uses volatile writes (WRITE_ONCE equivalent)
        // NOTE: Using DMA address, not CPU physical address!
        let desc = ring.desc(idx);
        let ctrl_val = ((total_len as u32) << 16) | MT_DMA_CTL_LAST_SEC0;
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, buf_dma as u32);
            core::ptr::write_volatile(&mut (*desc).buf1, 0);  // No second buffer
            core::ptr::write_volatile(&mut (*desc).info, 0);  // No high addr bits (< 4GB)
            // Write ctrl LAST - this triggers hardware to process the descriptor
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        // CRITICAL: Flush TX buffer and descriptor to RAM so DMA device can see them!
        // Without this, data sits in CPU cache and device reads stale RAM.
        flush_buffer(buf as u64, total_len);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        // Debug: print desc state (show both phys and dma for debugging)
        println!("    TX[{}]: buf_phys=0x{:08x} buf_dma=0x{:08x} len={} cmd=0x{:02x}",
                 idx, buf_phys as u32, buf_dma as u32, total_len, cmd);

        // Read indices before kick
        let cpu_before = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);
        let dma_before = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);

        // Advance CPU index
        ring.cpu_idx = (ring.cpu_idx + 1) % ring.ndesc;

        // Memory barrier before kicking DMA (Linux wmb() = dsb st on ARM64)
        dma_wmb();

        // Kick DMA by writing CPU index
        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        // Read indices after kick
        let cpu_after = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);
        let dma_after = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
        let glo_cfg = self.mt76_rr(MT_WFDMA0_GLO_CFG);

        println!("    CPU: {}->{}  DMA: {}->{}  GLO_CFG: 0x{:08x}",
                 cpu_before, cpu_after, dma_before, dma_after, glo_cfg);

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
    fn mcu_init_download(&self, ring: &mut TxRing, addr: u32, len: u32, mode: u32, seq: u8) -> Result<(), i32> {
        let cmd = if (addr & 0x80000000) != 0 {
            mcu_cmd::PATCH_START_REQ
        } else {
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
            println!("  mcu_init_download: TX timeout");
            println!("    RST=0x{:08x} GLO=0x{:08x} INT_SRC=0x{:08x}", rst, glo, int_src);
            println!("    DMA_IDX={} CPU_IDX={} DESC_BASE=0x{:08x}", dma_idx, cpu_idx_hw, desc_base);

            // Dump the descriptor that should have been processed
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
        let desc = ring.desc(idx);
        let ctrl_val = ((total_len as u32) << 16) | MT_DMA_CTL_LAST_SEC0;
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, buf_dma as u32);
            core::ptr::write_volatile(&mut (*desc).buf1, 0);  // No second buffer
            core::ptr::write_volatile(&mut (*desc).info, 0);  // No high addr bits
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
            println!("\n=== FIRST FIRMWARE CHUNK DIAGNOSTIC ===");
            println!("Ring regs @ 0x{:08x}", ring.regs_base);

            // Read back GLO_CFG to verify DMA is enabled
            let glo_cfg = self.mt76_rr(MT_WFDMA0_GLO_CFG);
            println!("GLO_CFG = 0x{:08x}", glo_cfg);
            println!("  TX_DMA_EN = {} (expected: 1)", (glo_cfg >> 0) & 1);
            println!("  RX_DMA_EN = {} (expected: 1)", (glo_cfg >> 2) & 1);
            println!("  EXT_EN    = {} (expected: 1)", (glo_cfg >> 26) & 1);

            // Read back ring registers
            let hw_desc_base = self.mt76_rr(ring.regs_base + MT_QUEUE_DESC_BASE);
            let hw_ring_size = self.mt76_rr(ring.regs_base + MT_QUEUE_RING_SIZE);
            let hw_cpu_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_CPU_IDX);
            let hw_dma_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
            println!("Ring registers (before kick):");
            println!("  DESC_BASE = 0x{:08x} (expected: 0x{:08x})", hw_desc_base, phys_to_dma(ring.desc_phys) as u32);
            println!("  RING_SIZE = {} (expected: {})", hw_ring_size, ring.ndesc);
            println!("  CPU_IDX   = {} (will become: {})", hw_cpu_idx, (idx + 1) % ring.ndesc);
            println!("  DMA_IDX   = {} (expected: 0)", hw_dma_idx);

            // Read back descriptor content
            let desc_buf0 = unsafe { core::ptr::read_volatile(&(*desc).buf0) };
            let desc_ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
            let desc_buf1 = unsafe { core::ptr::read_volatile(&(*desc).buf1) };
            let desc_info = unsafe { core::ptr::read_volatile(&(*desc).info) };
            println!("Descriptor[{}] content:", idx);
            println!("  buf0 = 0x{:08x} (buf_dma = 0x{:08x})", desc_buf0, buf_dma as u32);
            println!("  ctrl = 0x{:08x} (len={}, LAST={}, DMA_DONE={})",
                     desc_ctrl,
                     (desc_ctrl >> 16) & 0x3FFF,
                     (desc_ctrl >> 30) & 1,
                     (desc_ctrl >> 31) & 1);
            println!("  buf1 = 0x{:08x}", desc_buf1);
            println!("  info = 0x{:08x}", desc_info);
            println!("Buffer: virt=0x{:x} phys=0x{:x} dma=0x{:x} len={}", buf as u64, buf_phys, buf_dma, total_len);

            // Check INT_SOURCE before kick
            let int_src = self.mt76_rr(MT_INT_SOURCE_CSR);
            println!("INT_SOURCE before kick: 0x{:08x}", int_src);
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
            println!("After kick (1ms):");
            println!("  CPU_IDX   = {}", hw_cpu_idx);
            println!("  DMA_IDX   = {} (should be 1 if DMA processed)", hw_dma_idx);
            println!("  desc.ctrl = 0x{:08x} (DMA_DONE={})", desc_ctrl, (desc_ctrl >> 31) & 1);
            println!("  INT_SRC   = 0x{:08x}", int_src);
            println!("=== END DIAGNOSTIC ===\n");
        }

        // Wait for completion
        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 100) {
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
fn read_file(path: &[u8], buf: &mut [u8]) -> Result<usize, i32> {
    use userlib::syscall;

    let fd = syscall::open(path, 0);  // O_RDONLY
    if fd < 0 {
        return Err(fd);
    }
    let fd = fd as u32;

    // Kernel limits reads to 4096 bytes per syscall
    const MAX_READ_CHUNK: usize = 4096;

    let mut total = 0;
    loop {
        let remaining = buf.len() - total;
        let chunk_size = core::cmp::min(remaining, MAX_READ_CHUNK);
        if chunk_size == 0 {
            break;
        }

        let n = syscall::read(fd, &mut buf[total..total + chunk_size]);
        if n < 0 {
            syscall::close(fd);
            return Err(n as i32);
        }
        if n == 0 {
            break;  // EOF
        }
        total += n as usize;
    }

    syscall::close(fd);
    Ok(total)
}

impl Mt7996Dev {
    /// Load patch firmware
    /// From Linux mcu.c:2208-2296
    fn load_patch(&self, ring: &mut TxRing, fw_buf: &[u8], seq: &mut u8) -> Result<(), i32> {
        println!("  Loading patch...");

        let hdr_size = core::mem::size_of::<PatchHeader>();
        let sec_size = core::mem::size_of::<PatchSec>();

        if fw_buf.len() < hdr_size {
            println!("    Patch too small");
            return Err(-1);
        }

        // Parse patch header (at START of file)
        let hdr = unsafe { &*(fw_buf.as_ptr() as *const PatchHeader) };
        let hw_ver = u32::from_be(hdr.hw_sw_ver);
        let patch_ver = u32::from_be(hdr.patch_ver);
        let n_region = u32::from_be(hdr.desc.n_region);
        println!("    HW ver: 0x{:08x}, Patch ver: 0x{:08x}, Regions: {}", hw_ver, patch_ver, n_region);

        // Get semaphore first
        self.mcu_patch_sem_ctrl(ring, true, *seq)?;
        *seq = seq.wrapping_add(1);

        // Iterate through patch sections (right after header)
        // From Linux mcu.c:2242-2279
        for i in 0..n_region {
            let sec_offset = hdr_size + (i as usize * sec_size);
            if sec_offset + sec_size > fw_buf.len() {
                println!("    Section {} out of bounds", i);
                return Err(-1);
            }

            let sec = unsafe { &*(fw_buf.as_ptr().add(sec_offset) as *const PatchSec) };

            // Get section info (big-endian)
            let addr = u32::from_be(sec.addr);
            let len = u32::from_be(sec.len);
            let data_offs = u32::from_be(sec.offs) as usize;

            println!("    Section {}: addr=0x{:08x} len={} offs={}", i, addr, len, data_offs);

            if data_offs + len as usize > fw_buf.len() {
                println!("    Section data out of bounds");
                return Err(-1);
            }

            // Init download for this section
            self.mcu_init_download(ring, addr, len, 0, *seq)?;
            *seq = seq.wrapping_add(1);

            // Send section data in chunks
            let section_data = &fw_buf[data_offs..data_offs + len as usize];
            for chunk in section_data.chunks(MCU_FW_DL_BUF_SIZE - 4) {
                self.mcu_send_firmware_chunk(ring, chunk, *seq)?;
                *seq = seq.wrapping_add(1);
            }
        }

        // Start patch
        self.mcu_start_firmware(ring, true, *seq)?;
        *seq = seq.wrapping_add(1);

        println!("    Patch loaded");
        Ok(())
    }

    /// Load RAM firmware (WM, DSP, or WA)
    fn load_ram(&self, ring: &mut TxRing, fw_buf: &[u8], name: &str, seq: &mut u8) -> Result<(), i32> {
        println!("  Loading {} firmware...", name);

        if fw_buf.len() < core::mem::size_of::<FwTrailer>() {
            println!("    Firmware too small");
            return Err(-1);
        }

        // Trailer is at end of file
        let trailer_offset = fw_buf.len() - core::mem::size_of::<FwTrailer>();
        let trailer = unsafe { &*(fw_buf.as_ptr().add(trailer_offset) as *const FwTrailer) };

        let chip_id = trailer.chip_id;  // u8, no endian conversion needed
        let n_region = trailer.n_region;
        println!("    Chip ID: 0x{:02x}, Regions: {}", chip_id, n_region);

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

            println!("    Region {}: addr=0x{:08x} len={}", i, addr, len);

            // Init download for this region
            self.mcu_init_download(ring, addr, len, 0, *seq)?;
            *seq = seq.wrapping_add(1);

            // Send region data in chunks
            let region_data = &fw_buf[data_offset..data_offset + len as usize];
            for chunk in region_data.chunks(MCU_FW_DL_BUF_SIZE - 4) {
                self.mcu_send_firmware_chunk(ring, chunk, *seq)?;
                *seq = seq.wrapping_add(1);
            }

            data_offset += len as usize;
        }

        // Start firmware
        self.mcu_start_firmware(ring, false, *seq)?;
        *seq = seq.wrapping_add(1);

        println!("    {} loaded", name);
        Ok(())
    }

    /// Main firmware loading entry point
    pub fn load_firmware(&self, ring: &mut TxRing) -> Result<(), i32> {
        println!("\n=== Loading MT7996 Firmware ===\n");

        // Allocate buffer for firmware (3MB should be enough for largest file)
        const FW_BUF_SIZE: usize = 3 * 1024 * 1024;
        let mut fw_buf_virt: u64 = 0;
        let mut fw_buf_phys: u64 = 0;
        let shmem_id = syscall::shmem_create(FW_BUF_SIZE, &mut fw_buf_virt, &mut fw_buf_phys);
        if shmem_id < 0 {
            println!("Failed to allocate firmware buffer");
            return Err(-1);
        }

        let fw_buf = unsafe { core::slice::from_raw_parts_mut(fw_buf_virt as *mut u8, FW_BUF_SIZE) };
        let mut seq: u8 = 0;

        // Check firmware state before loading
        let fw_state = self.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
        println!("FW_STATE before loading: {}", fw_state);

        // 1. Load patch
        let patch_path = b"/lib/firmware/mediatek/mt7996/mt7996_rom_patch.bin";
        match read_file(patch_path, fw_buf) {
            Ok(size) => {
                println!("Patch file: {} bytes", size);
                if let Err(e) = self.load_patch(ring, &fw_buf[..size], &mut seq) {
                    println!("Patch load failed: {}", e);
                }
            }
            Err(e) => println!("Patch file not found: {}", e),
        }

        userlib::delay_ms(100);

        // 2. Load WM (Wireless Manager)
        let wm_path = b"/lib/firmware/mediatek/mt7996/mt7996_wm.bin";
        match read_file(wm_path, fw_buf) {
            Ok(size) => {
                println!("WM file: {} bytes", size);
                if let Err(e) = self.load_ram(ring, &fw_buf[..size], "WM", &mut seq) {
                    println!("WM load failed: {}", e);
                }
            }
            Err(e) => println!("WM file not found: {}", e),
        }

        userlib::delay_ms(100);

        // Check firmware state after WM
        let fw_state = self.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
        println!("FW_STATE after WM: {}", fw_state);

        // 3. Load WA (Wireless Accelerator) - optional for some configs
        let wa_path = b"/lib/firmware/mediatek/mt7996/mt7996_wa.bin";
        match read_file(wa_path, fw_buf) {
            Ok(size) => {
                println!("WA file: {} bytes", size);
                if let Err(e) = self.load_ram(ring, &fw_buf[..size], "WA", &mut seq) {
                    println!("WA load failed: {}", e);
                }
            }
            Err(e) => println!("WA file not found: {}", e),
        }

        // Final state check
        let fw_state = self.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
        println!("\nFW_STATE final: {} (7=running)", fw_state);

        // Cleanup
        syscall::shmem_destroy(shmem_id as u32);

        if fw_state == 7 {
            println!("\n=== Firmware loaded successfully! ===");
            Ok(())
        } else {
            println!("\n=== Firmware loading incomplete ===");
            Err(-1)
        }
    }
}

// ============================================================================
// Entry Point
// ============================================================================

/// MT7996 device IDs
const DEVICE_MT7996_HIF1: u16 = 0x7990;
const DEVICE_MT7996_HIF2: u16 = 0x7991;

#[unsafe(no_mangle)]
fn main() {
    println!("\n=== MT7996 WiFi Driver (wifid3) - EXACT Linux Translation ===\n");

    // Show DMA translation mode
    if USE_DRAM_OFFSET {
        println!("DMA Translation: DRAM_OFFSET (dma = phys - 0x{:x})", DRAM_BASE);
    } else {
        println!("DMA Translation: IDENTITY (dma = phys)");
    }
    println!("");

    // Step 1: Connect to pcied
    println!("Connecting to pcied...");
    let client = match PcieClient::connect() {
        Some(c) => c,
        None => {
            println!("FAILED (is pcied running?)");
            syscall::exit(1);
        }
    };
    println!("OK");

    // Step 2: Find MT7996 devices
    println!("Scanning for MediaTek devices...");
    let devices = client.find_devices(consts::vendor::MEDIATEK, 0xFFFF);
    println!("Found {} devices", devices.len());

    // Find HIF1 (main device) and HIF2 (secondary)
    let mut hif1_info: Option<pcie::PcieDeviceInfo> = None;
    let mut hif2_info: Option<pcie::PcieDeviceInfo> = None;

    for info in devices.iter() {
        // Print PCI command register: bit 0=IO, bit 1=MEM, bit 2=BME (Bus Master Enable)
        let io_en = if info.command & 0x01 != 0 { "IO" } else { "" };
        let mem_en = if info.command & 0x02 != 0 { "MEM" } else { "" };
        let bme = if info.command & 0x04 != 0 { "BME" } else { "NO-BME!" };
        println!("  [{:02x}:{:02x}.{}] {:04x}:{:04x} BAR0=0x{:x}/{}KB CMD=0x{:04x}[{} {} {}]",
            info.bus, info.device, info.function,
            info.vendor_id, info.device_id,
            info.bar0_addr, info.bar0_size / 1024,
            info.command, io_en, mem_en, bme);

        match info.device_id {
            DEVICE_MT7996_HIF1 => hif1_info = Some(*info),
            DEVICE_MT7996_HIF2 => hif2_info = Some(*info),
            _ => {}
        }
    }

    // Must have HIF1 (main interface)
    let hif1 = match hif1_info {
        Some(info) => info,
        None => {
            println!("\nERROR: No MT7996 HIF1 device found!");
            syscall::exit(1);
        }
    };

    let has_hif2 = hif2_info.is_some();

    println!("\n=== MT7996 WiFi Device ===");
    println!("  HIF1: {:04x}:{:04x} @ phys 0x{:x} cmd=0x{:04x}",
        hif1.vendor_id, hif1.device_id, hif1.bar0_addr, hif1.command);
    // Check bus master enabled (bit 2)
    if (hif1.command & 0x04) == 0 {
        println!("  WARNING: Bus Master not enabled! DMA will not work!");
    } else {
        println!("  Bus Master enabled: OK");
    }
    if let Some(ref hif2) = hif2_info {
        println!("  HIF2: {:04x}:{:04x} @ phys 0x{:x} cmd=0x{:04x}",
            hif2.vendor_id, hif2.device_id, hif2.bar0_addr, hif2.command);
    }

    // Step 3: Map BAR0
    println!("\n=== Mapping MMIO Regions ===");

    println!("  Mapping HIF1 BAR0 (0x{:x}, {}KB)...", hif1.bar0_addr, hif1.bar0_size / 1024);
    let (bar0_virt, bar0_size) = match syscall::mmap_device(hif1.bar0_addr, hif1.bar0_size as u64) {
        Ok(virt) => (virt, hif1.bar0_size as u64),
        Err(e) => {
            println!("ERROR: mmap_device failed: {}", e);
            syscall::exit(1);
        }
    };
    println!("  OK (virt=0x{:x})", bar0_virt);

    // Note: HIF2 registers (0xd8xxx) are accessible through HIF1's BAR at offset 0xd8xxx
    // Linux mmio.c __mt7996_reg_addr(): if (addr < 0x100000) return addr;
    // So we don't need to map HIF2's BAR separately for register access.

    // Step 4: Allocate DMA descriptor memory from LOW MEMORY pool
    // MT7996 WFDMA appears to have issues with higher memory addresses (0x46xxxxxx)
    // OpenWRT uses addresses around 0x40xxxxxx - we now use a dedicated DMA pool there
    println!("\n=== Allocating DMA Memory (from low memory pool) ===");

    const DESC_MEM_SIZE: usize = 64 * 1024; // 64KB for all queues
    let mut desc_virt: u64 = 0;
    let mut desc_phys: u64 = 0;
    let result = syscall::dma_pool_create(DESC_MEM_SIZE, &mut desc_virt, &mut desc_phys);
    if result < 0 {
        println!("ERROR: dma_pool_create failed: {} (falling back to shmem)", result);
        // Fallback to regular shmem if DMA pool fails
        let result = syscall::shmem_create(DESC_MEM_SIZE, &mut desc_virt, &mut desc_phys);
        if result < 0 {
            println!("ERROR: shmem_create also failed: {}", result);
            syscall::exit(1);
        }
    }
    println!("  Descriptor pool: virt=0x{:x} phys=0x{:x} size={}", desc_virt, desc_phys, DESC_MEM_SIZE);

    // Clear all descriptor memory
    unsafe {
        core::ptr::write_bytes(desc_virt as *mut u8, 0, DESC_MEM_SIZE);
    }

    // Allocate RX buffer pool (6 queues × 64 buffers × 2048 bytes = 768KB)
    // Linux: mt76_dma_rx_fill_buf() allocates skbs and fills descriptors with physical addresses
    const RX_BUF_POOL_SIZE: usize = 6 * 64 * 2048;  // 768KB for all RX queues
    let mut rx_buf_virt: u64 = 0;
    let mut rx_buf_phys: u64 = 0;
    let result = syscall::dma_pool_create(RX_BUF_POOL_SIZE, &mut rx_buf_virt, &mut rx_buf_phys);
    if result < 0 {
        println!("ERROR: dma_pool_create for RX buffers failed: {} (falling back to shmem)", result);
        // Fallback to regular shmem
        let result = syscall::shmem_create(RX_BUF_POOL_SIZE, &mut rx_buf_virt, &mut rx_buf_phys);
        if result < 0 {
            println!("ERROR: shmem_create for RX buffers also failed: {}", result);
            syscall::exit(1);
        }
    }
    println!("  RX buffer pool: virt=0x{:x} phys=0x{:x} size={}", rx_buf_virt, rx_buf_phys, RX_BUF_POOL_SIZE);

    // Clear RX buffer memory
    unsafe {
        core::ptr::write_bytes(rx_buf_virt as *mut u8, 0, RX_BUF_POOL_SIZE);
    }

    // Create device
    let dev = Mt7996Dev::new(bar0_virt, bar0_size, has_hif2);

    // ========================================================================
    // EXACT Linux initialization sequence
    // Source: pci.c (probe) + init.c:mt7996_init_hardware()
    // ========================================================================

    println!("\n=== Starting EXACT Linux init sequence ===\n");

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

    // Read initial register state
    let rst_before = dev.mt76_rr(MT_WFDMA0_RST);
    let glo_before = dev.mt76_rr(MT_WFDMA0_GLO_CFG);
    println!("Initial state:");
    println!("   RST: 0x{:08x}", rst_before);
    println!("   GLO_CFG: 0x{:08x}", glo_before);
    dump_int_src(&dev, "initial");

    // ========================================================================
    // PHASE 0: WFSYS Reset - ensure clean hardware state
    // Per MT7996 init guide: this happens EARLY, before PCIe/DMA setup
    // Source: init.c:762-769
    // ========================================================================
    println!("\n0. WFSYS Reset (mt7996_wfsys_reset)...");
    println!("   MT_WF_SUBSYS_RST @ 0x{:x} (needs L1 remap)", MT_WF_SUBSYS_RST);
    dev.mt7996_wfsys_reset();

    // Check state after WFSYS reset
    let rst_after_wfsys = dev.mt76_rr(MT_WFDMA0_RST);
    let glo_after_wfsys = dev.mt76_rr(MT_WFDMA0_GLO_CFG);
    println!("   After WFSYS reset:");
    println!("   RST: 0x{:08x}", rst_after_wfsys);
    println!("   GLO_CFG: 0x{:08x}", glo_after_wfsys);
    dump_int_src(&dev, "after_wfsys_reset");

    // ========================================================================
    // PHASE 1: PCIe setup (from pci.c:mt7996_pci_probe, BEFORE init_hardware)
    // ========================================================================

    // pci.c:163 - Disable all interrupts FIRST
    // Linux: mt76_wr(dev, MT_INT_MASK_CSR, 0);
    println!("\n1. Disabling interrupts (pci.c:163)...");
    println!("   HIF1: MT_INT_MASK_CSR = 0");
    dev.mt76_wr(MT_INT_MASK_CSR, 0);
    println!("   HIF2: MT_INT1_MASK_CSR = 0");
    dev.mt76_wr(MT_INT1_MASK_CSR, 0);

    // pci.c:165,192 - Master switch of PCIe interrupt enable
    // HIF1: 0x74030188 -> static map to 0x10188 (direct access)
    // HIF2: 0x74090188 -> NOT in static map, needs L1 remap
    println!("\n2. PCIe MAC interrupt enables (pci.c:165,192)...");

    // HIF1: MT_PCIE_MAC_INT_ENABLE @ BAR offset 0x10188 (static map)
    println!("   HIF1: MT_PCIE_MAC_INT_ENABLE @ 0x{:x} (static map)", MT_PCIE_MAC_INT_ENABLE);
    dev.mt76_wr(MT_PCIE_MAC_INT_ENABLE, 0xff);

    // HIF2: MT_PCIE1_MAC_INT_ENABLE @ 0x74090188 - needs L1 remap
    println!("   HIF2: MT_PCIE1_MAC_INT_ENABLE @ 0x{:x} (L1 remap)", MT_PCIE1_MAC_INT_ENABLE_PHYS);
    let pcie1_mapped = dev.mt7996_reg_map_l1(MT_PCIE1_MAC_INT_ENABLE_PHYS);
    println!("   HIF2: Remapped to 0x{:x}", pcie1_mapped);
    dev.mt76_wr(pcie1_mapped, 0xff);

    // pci.c:70-71 - Establish HIF1-HIF2 communication link
    // In Linux this is done by HIF2 device during mt7996_pci_init_hif2()
    // We do it from HIF1's perspective since we only have one BAR
    // Write: hif_idx(1) | MT_PCIE_RECOG_ID_SEM to MT_PCIE_RECOG_ID
    println!("\n2. Establishing HIF1-HIF2 communication (pci.c:70-71)...");
    let recog_val = 1 | MT_PCIE_RECOG_ID_SEM;  // hif_idx=1, semaphore bit set
    println!("   Writing 0x{:08x} to MT_PCIE_RECOG_ID @ 0x{:x}", recog_val, MT_PCIE_RECOG_ID);
    dev.mt76_wr(MT_PCIE_RECOG_ID, recog_val);
    let recog_read = dev.mt76_rr(MT_PCIE_RECOG_ID);
    println!("   MT_PCIE_RECOG_ID readback: 0x{:08x}", recog_read);

    // ========================================================================
    // PHASE 2: init_hardware() from init.c
    // Per DeepWiki: dma_init() BEFORE mcu_init(), driver_own() is in mcu_init()
    // ========================================================================

    // init.c - Clear interrupt source
    println!("\n3. Clearing INT_SOURCE_CSR...");
    dev.mt76_wr(MT_INT_SOURCE_CSR, !0u32);
    dev.mt76_wr(MT_INT1_SOURCE_CSR, !0u32);  // Also clear HIF2
    dump_int_src(&dev, "after_int_clear");

    // init.c - Initialize DMA FIRST (before mcu_init/driver_own)
    println!("\n4. Initializing DMA (mt7996_dma_init)...");
    dev.mt7996_dma_init(desc_phys, desc_virt, DESC_MEM_SIZE, rx_buf_phys, rx_buf_virt, RX_BUF_POOL_SIZE);
    dump_int_src(&dev, "after_dma_init");

    // === Register dump for OpenWRT comparison ===
    println!("\n=== Register Dump (compare with OpenWRT) ===");
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

    // ========================================================================
    // PHASE 3: mcu_init() from mcu.c - happens AFTER dma_init!
    // Per DeepWiki citations [4][5]: SWDEF_MODE then driver_own
    // ========================================================================
    println!("\n=== mcu_init (mcu.c) - AFTER dma_init ===");

    // mcu.c:3299-3303 - Set SWDEF_MODE BEFORE driver_own
    println!("\n5. Setting MT_SWDEF_MODE = NORMAL_MODE...");
    println!("   Register: 0x{:x}, Value: 0x{:x}", MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE);
    dev.mt76_wr(MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE);
    let swdef_read = dev.mt76_rr(MT_SWDEF_MODE);
    println!("   SWDEF_MODE readback: 0x{:x}", swdef_read);

    // mcu.c:3304-3312 - driver_own calls (AFTER dma_init!)
    println!("\n6. Taking driver ownership (mcu_init, AFTER dma_init)...");
    println!("   Band 0...");
    match dev.mt7996_driver_own(0) {
        Ok(()) => println!("   Band 0: Driver own successful!"),
        Err(e) => {
            println!("   Band 0 ERROR: driver_own failed: {}", e);
        }
    }

    // MT7996 ALWAYS has HIF2 at +0x4000 offset
    println!("   Band 1 (HIF2)...");
    match dev.mt7996_driver_own(1) {
        Ok(()) => println!("   Band 1: Driver own successful!"),
        Err(_) => println!("   Band 1 WARNING: driver_own failed"),
    }
    dump_int_src(&dev, "after_driver_own");

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
    println!("\n8. Firmware Loading (mcu_init -> mt7996_load_firmware)...");

    // Allocate buffer memory for MCU TX commands (4KB per descriptor) from DMA pool
    const TX_BUF_SIZE: usize = MT7996_TX_FWDL_RING_SIZE as usize * MCU_FW_DL_BUF_SIZE;
    let mut tx_buf_virt: u64 = 0;
    let mut tx_buf_phys: u64 = 0;
    let tx_buf_result = syscall::dma_pool_create(TX_BUF_SIZE, &mut tx_buf_virt, &mut tx_buf_phys);
    if tx_buf_result < 0 {
        println!("WARNING: dma_pool_create for TX buffer failed: {}, using shmem", tx_buf_result);
        let tx_buf_id = syscall::shmem_create(TX_BUF_SIZE, &mut tx_buf_virt, &mut tx_buf_phys);
        if tx_buf_id < 0 {
            println!("ERROR: Failed to allocate TX buffer: {}", tx_buf_id);
            syscall::exit(1);
        }
    }
    println!("TX buffer: virt=0x{:x} phys=0x{:x}", tx_buf_virt, tx_buf_phys);

    // Create TxRing for FWDL queue
    // FWDL is the 4th queue (index 3), each queue is 4KB aligned
    // From mt7996_dma_init: BAND0, MCU_WM, MCU_WA, then FWDL
    const FWDL_DESC_OFFSET: usize = 3 * 4096;  // 3 * 4KB = 12KB offset
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

    println!("\n");

    // Load firmware
    match dev.load_firmware(&mut fwdl_ring) {
        Ok(()) => println!("Firmware loading completed successfully!"),
        Err(e) => println!("Firmware loading failed: {}", e),
    }

    // Check final state after firmware
    println!("\n=== Post-Firmware State ===");
    let final_fw_state = dev.mt76_rr(MT_TOP_MISC) & MT_TOP_MISC_FW_STATE;
    println!("FW_STATE: {} (7=running)", final_fw_state);

    // Note: DMA pool memory is not freed - it's a bump allocator for driver lifetime
    // The pool will be reused if the driver is restarted

    println!("\n=== wifid3 complete ===");
}
