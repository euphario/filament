//! MT7996 WFDMA (WiFi DMA) Driver
//!
//! The MT7996 uses WFDMA for host-to-device communication including:
//! - Firmware download (FWDL queue)
//! - MCU commands (WM/WA queues)
//! - TX/RX data
//!
//! ## DMA Ring Structure
//!
//! Each ring consists of:
//! - Base address register (ring descriptor array physical address)
//! - Count register (number of descriptors)
//! - CPU index (host write pointer)
//! - DMA index (device read pointer)
//!
//! ## Register Layout (per ring at base + queue_offset)
//!
//! Linux struct mt76_queue_regs is exactly 16 bytes:
//! - 0x00: desc_base  (descriptor ring base address, 32-bit only!)
//! - 0x04: ring_size  (number of descriptors)
//! - 0x08: cpu_idx    (host write pointer)
//! - 0x0C: dma_idx    (device read pointer)
//!
//! IMPORTANT: There is NO 0x10 offset for base_hi! Offset 0x10 would be the
//! next ring's desc_base. Our physical addresses fit in 32 bits anyway.

use crate::device::Mt7996Device;
use crate::regs;
use userlib::{syscall, trace};

// Note: crate::dma_defs and crate::mt7996_defs are used via fully-qualified paths
// in the submodules below (queue_map, wfdma, glo_cfg, dma_ctl) to provide canonical
// definitions that were translated from Linux header files.

//=============================================================================
// Queue ID Mapping System (mirrors Linux mt76 exactly)
//=============================================================================
//
// Linux uses a q_id[] array to map logical queue indices to hardware ring indices.
// This allows the same code to work with different chip variants that may have
// different hardware ring assignments.
//
// The q_id[] array is indexed by three namespaces:
//   - MCU queues:     indices 0 .. __MT_MCUQ_MAX
//   - RX queues:      indices __MT_MCUQ_MAX .. (__MT_MCUQ_MAX + __MT_RXQ_MAX)
//   - Data TX queues: indices (__MT_MCUQ_MAX + __MT_RXQ_MAX) .. end
//
// Linux macros:
//   #define __RXQ(q)     ((q) + __MT_MCUQ_MAX)
//   #define __TXQ(q)     (__RXQ(q) + __MT_RXQ_MAX)
//   #define MT_MCUQ_ID(q)  dev->q_id[q]
//   #define MT_RXQ_ID(q)   dev->q_id[__RXQ(q)]
//   #define MT_TXQ_ID(q)   dev->q_id[__TXQ(q)]

/// MT7996 queue ID mapping - mirrors Linux q_id[] array system
pub mod queue_map {
    /// Ring size in bytes (16 bytes per ring register set) - from dma_defs
    pub const MT_RING_SIZE: u32 = crate::dma_defs::MT_RING_SIZE;

    //-------------------------------------------------------------------------
    // Logical Queue Indices (mirrors Linux enums)
    //-------------------------------------------------------------------------

    /// MCU queue logical indices (mt76.h: enum mt76_mcuq_id)
    pub mod mcuq {
        pub const WM: usize = 0;      // MT_MCUQ_WM
        pub const WA: usize = 1;      // MT_MCUQ_WA
        pub const FWDL: usize = 2;    // MT_MCUQ_FWDL
        pub const MAX: usize = 3;     // __MT_MCUQ_MAX
    }

    /// RX queue logical indices (mt76.h: enum mt76_rxq_id)
    /// Note: These are LOGICAL indices, not hardware ring indices!
    pub mod rxq {
        pub const MAIN: usize = 0;           // MT_RXQ_MAIN
        pub const MCU: usize = 1;            // MT_RXQ_MCU
        pub const MCU_WA: usize = 2;         // MT_RXQ_MCU_WA
        pub const BAND1: usize = 3;          // MT_RXQ_BAND1
        pub const BAND1_WA: usize = 4;       // MT_RXQ_BAND1_WA
        pub const MAIN_WA: usize = 5;        // MT_RXQ_MAIN_WA
        pub const BAND2: usize = 6;          // MT_RXQ_BAND2
        pub const BAND2_WA: usize = 7;       // MT_RXQ_BAND2_WA
        pub const RRO_BAND0: usize = 8;      // MT_RXQ_RRO_BAND0
        pub const RRO_BAND1: usize = 9;      // MT_RXQ_RRO_BAND1
        pub const RRO_BAND2: usize = 10;     // MT_RXQ_RRO_BAND2
        pub const MSDU_PAGE_BAND0: usize = 11;
        pub const MSDU_PAGE_BAND1: usize = 12;
        pub const MSDU_PAGE_BAND2: usize = 13;
        pub const TXFREE_BAND0: usize = 14;  // MT_RXQ_TXFREE_BAND0
        pub const TXFREE_BAND1: usize = 15;  // MT_RXQ_TXFREE_BAND1
        pub const TXFREE_BAND2: usize = 16;  // MT_RXQ_TXFREE_BAND2
        pub const RRO_IND: usize = 17;
        pub const RRO_RXDMAD_C: usize = 18;
        pub const NPU0: usize = 19;
        pub const NPU1: usize = 20;
        pub const MAX: usize = 21;           // __MT_RXQ_MAX
    }

    /// Data TX queue logical indices (0, 1, 2 for BAND0, BAND1, BAND2)
    pub mod txq {
        pub const BAND0: usize = 0;
        pub const BAND1: usize = 1;
        pub const BAND2: usize = 2;
        pub const MAX: usize = 3;
    }

    //-------------------------------------------------------------------------
    // Hardware Ring Indices for MT7996 (mt7996.h enums)
    //-------------------------------------------------------------------------

    /// MT7996 TX queue hardware ring indices - from mt7996_defs::Mt7996TxqId
    pub mod mt7996_txq {
        use crate::mt7996_defs::Mt7996TxqId;
        pub const FWDL: u32 = Mt7996TxqId::Fwdl as u32;
        pub const MCU_WM: u32 = Mt7996TxqId::McuWm as u32;
        pub const BAND0: u32 = Mt7996TxqId::Band0 as u32;
        pub const BAND1: u32 = Mt7996TxqId::Band1 as u32;
        pub const MCU_WA: u32 = Mt7996TxqId::McuWa as u32;
        pub const BAND2: u32 = Mt7996TxqId::Band2 as u32;
    }

    /// MT7996 RX queue hardware ring indices - from mt7996_defs::rxq_id
    pub mod mt7996_rxq {
        use crate::mt7996_defs::rxq_id;
        pub const MCU_WM: u32 = rxq_id::MCU_WM;
        pub const MCU_WA: u32 = rxq_id::MCU_WA;
        pub const MCU_WA_MAIN: u32 = rxq_id::MCU_WA_MAIN;
        pub const MCU_WA_TRI: u32 = rxq_id::MCU_WA_TRI;
        pub const BAND0: u32 = rxq_id::BAND0;
        pub const BAND2: u32 = rxq_id::BAND2;
        pub const RRO_BAND2: u32 = rxq_id::RRO_BAND2;
        pub const TXFREE2: u32 = rxq_id::TXFREE2;
        pub const RRO_BAND0: u32 = rxq_id::RRO_BAND0;
        pub const TXFREE0: u32 = rxq_id::TXFREE0;
        pub const RRO_BAND1: u32 = rxq_id::RRO_BAND1;
        pub const MSDU_PG_BAND0: u32 = rxq_id::MSDU_PG_BAND0;
        pub const MSDU_PG_BAND1: u32 = rxq_id::MSDU_PG_BAND1;
        pub const MSDU_PG_BAND2: u32 = rxq_id::MSDU_PG_BAND2;
    }

    //-------------------------------------------------------------------------
    // Index Transformation (mirrors Linux macros)
    //-------------------------------------------------------------------------

    /// Convert RX logical index to q_id[] array index
    /// Linux: #define __RXQ(q) ((q) + __MT_MCUQ_MAX)
    #[inline]
    pub const fn rxq_to_qid_idx(rxq: usize) -> usize {
        rxq + mcuq::MAX
    }

    /// Convert Data TX logical index to q_id[] array index
    /// Linux: #define __TXQ(q) (__RXQ(q) + __MT_RXQ_MAX)
    #[inline]
    pub const fn txq_to_qid_idx(txq: usize) -> usize {
        rxq_to_qid_idx(txq) + rxq::MAX
    }

    /// Total size of q_id[] array
    pub const QID_ARRAY_SIZE: usize = mcuq::MAX + rxq::MAX + txq::MAX;

    //-------------------------------------------------------------------------
    // Queue ID Mapping Structure
    //-------------------------------------------------------------------------

    /// Queue ID mapping table - mirrors Linux dev->q_id[]
    /// Populated at init time based on chip type
    pub struct QueueIds {
        /// Maps logical queue index to hardware ring index
        pub q_id: [u32; QID_ARRAY_SIZE],
    }

    impl QueueIds {
        /// Create queue ID mapping for MT7996
        /// This mirrors mt7996_dma_config() in Linux
        pub const fn new_mt7996() -> Self {
            let mut q_id = [0u32; QID_ARRAY_SIZE];

            // MCU queues (index 0..3)
            q_id[mcuq::WM] = mt7996_txq::MCU_WM;
            q_id[mcuq::WA] = mt7996_txq::MCU_WA;
            q_id[mcuq::FWDL] = mt7996_txq::FWDL;

            // RX queues (index 3..24) - offset by mcuq::MAX
            q_id[rxq_to_qid_idx(rxq::MCU)] = mt7996_rxq::MCU_WM;
            q_id[rxq_to_qid_idx(rxq::MCU_WA)] = mt7996_rxq::MCU_WA;
            q_id[rxq_to_qid_idx(rxq::MAIN_WA)] = mt7996_rxq::MCU_WA_MAIN;
            q_id[rxq_to_qid_idx(rxq::BAND2_WA)] = mt7996_rxq::MCU_WA_TRI;
            q_id[rxq_to_qid_idx(rxq::MAIN)] = mt7996_rxq::BAND0;
            q_id[rxq_to_qid_idx(rxq::BAND2)] = mt7996_rxq::BAND2;
            q_id[rxq_to_qid_idx(rxq::RRO_BAND0)] = mt7996_rxq::RRO_BAND0;
            q_id[rxq_to_qid_idx(rxq::RRO_BAND2)] = mt7996_rxq::RRO_BAND2;
            q_id[rxq_to_qid_idx(rxq::TXFREE_BAND0)] = mt7996_rxq::TXFREE0;
            q_id[rxq_to_qid_idx(rxq::TXFREE_BAND2)] = mt7996_rxq::TXFREE2;

            // Data TX queues (index 24..27) - offset by mcuq::MAX + rxq::MAX
            q_id[txq_to_qid_idx(txq::BAND0)] = mt7996_txq::BAND0;
            q_id[txq_to_qid_idx(txq::BAND1)] = mt7996_txq::BAND1;
            q_id[txq_to_qid_idx(txq::BAND2)] = mt7996_txq::BAND2;

            Self { q_id }
        }

        //---------------------------------------------------------------------
        // ID Lookup Methods (mirror Linux MT_*_ID macros)
        //---------------------------------------------------------------------

        /// Get MCU queue hardware ring index
        /// Linux: #define MT_MCUQ_ID(q) dev->q_id[q]
        #[inline]
        pub const fn mcuq_id(&self, q: usize) -> u32 {
            self.q_id[q]
        }

        /// Get RX queue hardware ring index
        /// Linux: #define MT_RXQ_ID(q) dev->q_id[__RXQ(q)]
        #[inline]
        pub const fn rxq_id(&self, q: usize) -> u32 {
            self.q_id[rxq_to_qid_idx(q)]
        }

        /// Get Data TX queue hardware ring index
        /// Linux: #define MT_TXQ_ID(q) dev->q_id[__TXQ(q)]
        #[inline]
        pub const fn txq_id(&self, q: usize) -> u32 {
            self.q_id[txq_to_qid_idx(q)]
        }

        //---------------------------------------------------------------------
        // Ring Base Address Methods (mirror Linux MT_*_RING_BASE macros)
        //---------------------------------------------------------------------

        /// MCU queue ring base address (relative to WFDMA base)
        /// Linux: #define MT_MCUQ_RING_BASE(q) (MT_Q_BASE(q) + 0x300)
        /// Returns offset from WFDMA base (we always use WFDMA0)
        #[inline]
        pub const fn mcuq_ring_base(&self, q: usize) -> u32 {
            0x300 + self.mcuq_id(q) * MT_RING_SIZE
        }

        /// RX queue ring base address (relative to WFDMA base)
        /// Linux: #define MT_RXQ_RING_BASE(q) (MT_Q_BASE(__RXQ(q)) + 0x500)
        #[inline]
        pub const fn rxq_ring_base(&self, q: usize) -> u32 {
            0x500 + self.rxq_id(q) * MT_RING_SIZE
        }

        /// Data TX queue ring base address (relative to WFDMA base)
        /// Linux: #define MT_TXQ_RING_BASE(q) (MT_Q_BASE(__TXQ(q)) + 0x300)
        #[inline]
        pub const fn txq_ring_base(&self, q: usize) -> u32 {
            0x300 + self.txq_id(q) * MT_RING_SIZE
        }

        //---------------------------------------------------------------------
        // Prefetch Control Methods (mirror Linux MT_*_EXT_CTRL macros)
        //---------------------------------------------------------------------

        /// MCU queue prefetch control register (relative to WFDMA base)
        /// Linux: #define MT_MCUQ_EXT_CTRL(q) (MT_Q_BASE(q) + 0x600 + MT_MCUQ_ID(q) * 0x4)
        #[inline]
        pub const fn mcuq_ext_ctrl(&self, q: usize) -> u32 {
            0x600 + self.mcuq_id(q) * 4
        }

        /// RX queue prefetch control register (relative to WFDMA base)
        /// Linux: #define MT_RXQ_EXT_CTRL(q) (MT_Q_BASE(__RXQ(q)) + 0x680 + MT_RXQ_ID(q) * 0x4)
        #[inline]
        pub const fn rxq_ext_ctrl(&self, q: usize) -> u32 {
            0x680 + self.rxq_id(q) * 4
        }

        /// Data TX queue prefetch control register (relative to WFDMA base)
        /// Linux: #define MT_TXQ_EXT_CTRL(q) (MT_Q_BASE(__TXQ(q)) + 0x600 + MT_TXQ_ID(q) * 0x4)
        #[inline]
        pub const fn txq_ext_ctrl(&self, q: usize) -> u32 {
            0x600 + self.txq_id(q) * 4
        }
    }

    /// Static queue ID mapping for MT7996
    pub static MT7996_QUEUE_IDS: QueueIds = QueueIds::new_mt7996();

    //-------------------------------------------------------------------------
    // Self-Tests (run at init to validate all calculations match expected)
    //-------------------------------------------------------------------------

    /// Validate queue ID mappings match expected values
    /// Call this at driver init to catch any math errors
    pub fn validate_mt7996_queue_ids() -> Result<(), &'static str> {
        let q = &MT7996_QUEUE_IDS;

        // Validate MCU queue IDs
        if q.mcuq_id(mcuq::FWDL) != 16 { return Err("FWDL hw_id != 16"); }
        if q.mcuq_id(mcuq::WM) != 17 { return Err("WM hw_id != 17"); }
        if q.mcuq_id(mcuq::WA) != 20 { return Err("WA hw_id != 20"); }

        // Validate Data TX queue IDs
        if q.txq_id(txq::BAND0) != 18 { return Err("TXQ_BAND0 hw_id != 18"); }
        if q.txq_id(txq::BAND1) != 19 { return Err("TXQ_BAND1 hw_id != 19"); }
        if q.txq_id(txq::BAND2) != 21 { return Err("TXQ_BAND2 hw_id != 21"); }

        // Validate RX queue IDs
        if q.rxq_id(rxq::MCU) != 0 { return Err("RXQ_MCU hw_id != 0"); }
        if q.rxq_id(rxq::MCU_WA) != 1 { return Err("RXQ_MCU_WA hw_id != 1"); }
        if q.rxq_id(rxq::MAIN_WA) != 2 { return Err("RXQ_MAIN_WA hw_id != 2"); }
        if q.rxq_id(rxq::MAIN) != 4 { return Err("RXQ_MAIN hw_id != 4"); }
        if q.rxq_id(rxq::BAND2) != 5 { return Err("RXQ_BAND2 hw_id != 5"); }
        if q.rxq_id(rxq::RRO_BAND0) != 8 { return Err("RXQ_RRO_BAND0 hw_id != 8"); }
        if q.rxq_id(rxq::TXFREE_BAND0) != 9 { return Err("RXQ_TXFREE_BAND0 hw_id != 9"); }
        if q.rxq_id(rxq::TXFREE_BAND2) != 7 { return Err("RXQ_TXFREE_BAND2 hw_id != 7"); }

        // Validate ring base calculations
        if q.mcuq_ring_base(mcuq::FWDL) != 0x400 { return Err("FWDL ring_base != 0x400"); }
        if q.mcuq_ring_base(mcuq::WM) != 0x410 { return Err("WM ring_base != 0x410"); }
        if q.mcuq_ring_base(mcuq::WA) != 0x440 { return Err("WA ring_base != 0x440"); }
        if q.txq_ring_base(txq::BAND0) != 0x420 { return Err("TXQ_BAND0 ring_base != 0x420"); }
        if q.txq_ring_base(txq::BAND1) != 0x430 { return Err("TXQ_BAND1 ring_base != 0x430"); }
        if q.txq_ring_base(txq::BAND2) != 0x450 { return Err("TXQ_BAND2 ring_base != 0x450"); }
        if q.rxq_ring_base(rxq::MCU) != 0x500 { return Err("RXQ_MCU ring_base != 0x500"); }
        if q.rxq_ring_base(rxq::MAIN) != 0x540 { return Err("RXQ_MAIN ring_base != 0x540"); }
        if q.rxq_ring_base(rxq::TXFREE_BAND0) != 0x590 { return Err("RXQ_TXFREE0 ring_base != 0x590"); }

        // Validate prefetch control calculations
        if q.mcuq_ext_ctrl(mcuq::FWDL) != 0x640 { return Err("FWDL ext_ctrl != 0x640"); }
        if q.mcuq_ext_ctrl(mcuq::WM) != 0x644 { return Err("WM ext_ctrl != 0x644"); }
        if q.mcuq_ext_ctrl(mcuq::WA) != 0x650 { return Err("WA ext_ctrl != 0x650"); }
        if q.txq_ext_ctrl(txq::BAND0) != 0x648 { return Err("TXQ_BAND0 ext_ctrl != 0x648"); }
        if q.txq_ext_ctrl(txq::BAND1) != 0x64C { return Err("TXQ_BAND1 ext_ctrl != 0x64C"); }
        if q.txq_ext_ctrl(txq::BAND2) != 0x654 { return Err("TXQ_BAND2 ext_ctrl != 0x654"); }
        if q.rxq_ext_ctrl(rxq::MCU) != 0x680 { return Err("RXQ_MCU ext_ctrl != 0x680"); }
        if q.rxq_ext_ctrl(rxq::MAIN) != 0x690 { return Err("RXQ_MAIN ext_ctrl != 0x690"); }
        if q.rxq_ext_ctrl(rxq::TXFREE_BAND0) != 0x6A4 { return Err("RXQ_TXFREE0 ext_ctrl != 0x6A4"); }

        Ok(())
    }

    /// Print all queue mappings for debugging
    pub fn dump_queue_ids() {
        let q = &MT7996_QUEUE_IDS;

        userlib::println!("[QueueMap] MT7996 Queue ID Mapping:");
        userlib::println!("[QueueMap]   MCU TX queues:");
        userlib::println!("[QueueMap]     FWDL: hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.mcuq_id(mcuq::FWDL), q.mcuq_ring_base(mcuq::FWDL), q.mcuq_ext_ctrl(mcuq::FWDL));
        userlib::println!("[QueueMap]     WM:   hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.mcuq_id(mcuq::WM), q.mcuq_ring_base(mcuq::WM), q.mcuq_ext_ctrl(mcuq::WM));
        userlib::println!("[QueueMap]     WA:   hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.mcuq_id(mcuq::WA), q.mcuq_ring_base(mcuq::WA), q.mcuq_ext_ctrl(mcuq::WA));

        userlib::println!("[QueueMap]   Data TX queues:");
        userlib::println!("[QueueMap]     BAND0: hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.txq_id(txq::BAND0), q.txq_ring_base(txq::BAND0), q.txq_ext_ctrl(txq::BAND0));
        userlib::println!("[QueueMap]     BAND1: hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.txq_id(txq::BAND1), q.txq_ring_base(txq::BAND1), q.txq_ext_ctrl(txq::BAND1));
        userlib::println!("[QueueMap]     BAND2: hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.txq_id(txq::BAND2), q.txq_ring_base(txq::BAND2), q.txq_ext_ctrl(txq::BAND2));

        userlib::println!("[QueueMap]   RX queues:");
        userlib::println!("[QueueMap]     MCU:        hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.rxq_id(rxq::MCU), q.rxq_ring_base(rxq::MCU), q.rxq_ext_ctrl(rxq::MCU));
        userlib::println!("[QueueMap]     MCU_WA:     hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.rxq_id(rxq::MCU_WA), q.rxq_ring_base(rxq::MCU_WA), q.rxq_ext_ctrl(rxq::MCU_WA));
        userlib::println!("[QueueMap]     MAIN:       hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.rxq_id(rxq::MAIN), q.rxq_ring_base(rxq::MAIN), q.rxq_ext_ctrl(rxq::MAIN));
        userlib::println!("[QueueMap]     TXFREE0:    hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.rxq_id(rxq::TXFREE_BAND0), q.rxq_ring_base(rxq::TXFREE_BAND0), q.rxq_ext_ctrl(rxq::TXFREE_BAND0));
        userlib::println!("[QueueMap]     TXFREE2:    hw_id={} ring=0x{:03x} ext_ctrl=0x{:03x}",
            q.rxq_id(rxq::TXFREE_BAND2), q.rxq_ring_base(rxq::TXFREE_BAND2), q.rxq_ext_ctrl(rxq::TXFREE_BAND2));
    }
}

/// Cache line size (64 bytes for Cortex-A73)
const CACHE_LINE: usize = 64;

/// Flush (clean) a cache line to memory.
/// Use before DMA reads (hardware reading data that CPU wrote).
#[inline]
fn flush_cache_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc cvac, {}", in(reg) addr, options(nostack, preserves_flags));
    }
}

/// Data Synchronization Barrier
#[inline]
fn dsb() {
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// Flush a buffer to memory (for DMA reads by hardware)
#[inline]
fn flush_buffer(addr: u64, size: usize) {
    let start = addr & !(CACHE_LINE as u64 - 1);
    let end = (addr + size as u64 + CACHE_LINE as u64 - 1) & !(CACHE_LINE as u64 - 1);
    let mut a = start;
    while a < end {
        flush_cache_line(a);
        a += CACHE_LINE as u64;
    }
    dsb();
}

/// Address remapping registers (for accessing addresses > 0x100000)
pub mod remap {
    /// CONN_BUS_CR_VON_BASE
    pub const CONN_BUS_CR_VON_BASE: u32 = 0x155000;

    /// HIF_REMAP_L1 register offset (MT7990)
    pub const HIF_REMAP_L1_OFFSET: u32 = 0x8;
    /// Full HIF_REMAP_L1 address
    pub const HIF_REMAP_L1: u32 = CONN_BUS_CR_VON_BASE + HIF_REMAP_L1_OFFSET;

    /// Where remapped region appears in BAR space (MT7990)
    pub const HIF_REMAP_BASE_L1: u32 = 0x40000;

    /// L1 remap granularity (256KB)
    pub const L1_REMAP_MASK: u32 = 0x3ffff;
    pub const L1_BASE_MASK: u32 = !L1_REMAP_MASK;

    /// CBTOP remap for MT7990 (for addresses in 0x70000000 range)
    pub const HIF_REMAP_CBTOP: u32 = 0x1f6554;
    /// Where CBTOP remapped region appears in BAR space
    pub const HIF_REMAP_BASE_CBTOP: u32 = 0x1c0000;
    /// CBTOP remap granularity (64KB)
    pub const CBTOP_REMAP_MASK: u32 = 0xffff;

    /// CBTOP1 physical address range start
    pub const CBTOP1_PHY_START: u32 = 0x70000000;

    /// Wireless subsystem reset register (in CBTOP1 range, needs CBTOP remap)
    pub const WF_SUBSYS_RST: u32 = 0x70028600;
}

/// WFDMA base addresses (relative to BAR0)
pub mod wfdma {
    /// WFDMA0 base - from regs::wfdma0
    pub const WFDMA0_BASE: u32 = crate::regs::wfdma0::BASE;
    /// WFDMA1 base - from regs::wfdma1
    pub const WFDMA1_BASE: u32 = crate::regs::wfdma1::BASE;
    /// WFDMA0 on PCIe1 (for dual-band) - from regs::wfdma0_pcie1
    pub const WFDMA0_PCIE1_BASE: u32 = crate::regs::wfdma0_pcie1::BASE;

    /// Reset register offset - computed from regs::wfdma0::RST - BASE
    pub const RST: u32 = crate::regs::wfdma0::RST - crate::regs::wfdma0::BASE;
    /// Reset bits - from regs::wfdma0
    pub const RST_LOGIC_RST: u32 = crate::regs::wfdma0::RST_LOGIC_RST;
    pub const RST_DMASHDL_ALL_RST: u32 = crate::regs::wfdma0::RST_DMASHDL_ALL_RST;

    /// Busy enable register offset - from regs::wfdma0
    pub const BUSY_ENA: u32 = crate::regs::wfdma0::BUSY_ENA - crate::regs::wfdma0::BASE;
    pub const BUSY_ENA_TX_FIFO0: u32 = crate::regs::wfdma0::BUSY_ENA_TX_FIFO0;
    pub const BUSY_ENA_TX_FIFO1: u32 = crate::regs::wfdma0::BUSY_ENA_TX_FIFO1;
    pub const BUSY_ENA_RX_FIFO: u32 = crate::regs::wfdma0::BUSY_ENA_RX_FIFO;
    /// PCIE1 (HIF2) BUSY_ENA bits - from regs::wfdma0_pcie1
    pub const PCIE1_BUSY_ENA_TX_FIFO0: u32 = crate::regs::wfdma0_pcie1::BUSY_ENA_TX_FIFO0;
    pub const PCIE1_BUSY_ENA_TX_FIFO1: u32 = crate::regs::wfdma0_pcie1::BUSY_ENA_TX_FIFO1;
    pub const PCIE1_BUSY_ENA_RX_FIFO: u32 = crate::regs::wfdma0_pcie1::BUSY_ENA_RX_FIFO;

    /// MCU command register offset - from regs::wfdma0
    pub const MCU_CMD: u32 = crate::regs::wfdma0::MCU_CMD - crate::regs::wfdma0::BASE;

    /// Interrupt source offset - from regs::wfdma0
    pub const INT_SRC: u32 = crate::regs::wfdma0::INT_SOURCE_CSR - crate::regs::wfdma0::BASE;
    /// Interrupt mask offset - from regs::wfdma0
    pub const INT_MASK: u32 = crate::regs::wfdma0::INT_MASK_CSR - crate::regs::wfdma0::BASE;

    /// Global config register offset - from regs::wfdma0
    pub const GLO_CFG: u32 = crate::regs::wfdma0::GLO_CFG - crate::regs::wfdma0::BASE;

    /// Reset DTX pointer offset - from regs::wfdma0
    pub const RST_DTX_PTR: u32 = crate::regs::wfdma0::RST_DTX_PTR - crate::regs::wfdma0::BASE;

    /// Interrupt delay configuration register offsets - from regs::wfdma0
    pub const PRI_DLY_INT_CFG0: u32 = crate::regs::wfdma0::PRI_DLY_INT_CFG0 - crate::regs::wfdma0::BASE;
    pub const PRI_DLY_INT_CFG1: u32 = crate::regs::wfdma0::PRI_DLY_INT_CFG1 - crate::regs::wfdma0::BASE;
    pub const PRI_DLY_INT_CFG2: u32 = crate::regs::wfdma0::PRI_DLY_INT_CFG2 - crate::regs::wfdma0::BASE;

    /// Extended global config register offsets - from regs::wfdma0
    pub const GLO_CFG_EXT0: u32 = crate::regs::wfdma0::GLO_CFG_EXT0 - crate::regs::wfdma0::BASE;
    pub const GLO_CFG_EXT1: u32 = crate::regs::wfdma0::GLO_CFG_EXT1 - crate::regs::wfdma0::BASE;

    /// GLO_CFG_EXT0 bits - from regs::wfdma0
    pub const GLO_CFG_EXT0_RX_WB_RXD: u32 = crate::regs::wfdma0::GLO_CFG_EXT0_RX_WB_RXD;
    pub const GLO_CFG_EXT0_WED_MERGE_MODE: u32 = crate::regs::wfdma0::GLO_CFG_EXT0_WED_MERGE_MODE;

    /// RX pause threshold register offsets - from regs::wfdma0
    /// Note: NOT sequential! 0x274-0x278 reserved, RRO at 0x27c
    pub const PAUSE_RX_Q_45_TH: u32 = crate::regs::wfdma0::PAUSE_RX_Q_45_TH - crate::regs::wfdma0::BASE;
    pub const PAUSE_RX_Q_67_TH: u32 = crate::regs::wfdma0::PAUSE_RX_Q_67_TH - crate::regs::wfdma0::BASE;
    pub const PAUSE_RX_Q_89_TH: u32 = crate::regs::wfdma0::PAUSE_RX_Q_89_TH - crate::regs::wfdma0::BASE;
    pub const PAUSE_RX_Q_RRO_TH: u32 = crate::regs::wfdma0::PAUSE_RX_Q_RRO_TH - crate::regs::wfdma0::BASE;

    /// WFDMA Extended CSR base - from regs::wfdma_ext_csr
    pub const EXT_CSR_BASE: u32 = crate::regs::wfdma_ext_csr::BASE;
    /// HIF misc status register (relative to BAR0) - from regs::wfdma_ext_csr
    pub const EXT_CSR_HIF_MISC: u32 = crate::regs::wfdma_ext_csr::HIF_MISC;
    /// BUSY bit in HIF_MISC - from regs::wfdma_ext_csr
    pub const EXT_CSR_HIF_MISC_BUSY: u32 = crate::regs::wfdma_ext_csr::HIF_MISC_BUSY;

    /// Host config register (relative to BAR0) - from regs::wfdma_ext_csr
    pub const HOST_CONFIG: u32 = crate::regs::wfdma_ext_csr::HOST_CONFIG;
    /// HOST_CONFIG bits - from regs::wfdma_ext_csr
    pub const HOST_CONFIG_PDMA_BAND: u32 = crate::regs::wfdma_ext_csr::HOST_CONFIG_PDMA_BAND;
    pub const HOST_CONFIG_BAND0_PCIE1: u32 = crate::regs::wfdma_ext_csr::HOST_CONFIG_BAND0_PCIE1;
    pub const HOST_CONFIG_BAND1_PCIE1: u32 = crate::regs::wfdma_ext_csr::HOST_CONFIG_BAND1_PCIE1;
    pub const HOST_CONFIG_BAND2_PCIE1: u32 = crate::regs::wfdma_ext_csr::HOST_CONFIG_BAND2_PCIE1;

    /// AXI R2A control register - from regs::wfdma_ext_csr
    pub const AXI_R2A_CTRL: u32 = crate::regs::wfdma_ext_csr::AXI_R2A_CTRL;
    pub const AXI_R2A_CTRL_OUTSTAND_MASK: u32 = crate::regs::wfdma_ext_csr::AXI_R2A_CTRL_OUTSTAND_MASK;

    /// RX interrupt PCIe select register - from regs::wfdma0
    pub const RX_INT_PCIE_SEL: u32 = crate::regs::wfdma0::RX_INT_PCIE_SEL - crate::regs::wfdma0::BASE;
    pub const RX_INT_SEL_RING3: u32 = 1 << 3;
    pub const RX_INT_SEL_RING5: u32 = 1 << 5;
    pub const RX_INT_SEL_RING6: u32 = 1 << 6;
    pub const RX_INT_SEL_RING9: u32 = 1 << 9;

    /// GLO_CFG_EXT1 bits - from regs::wfdma0
    pub const GLO_CFG_EXT1_TX_FCTRL_MODE: u32 = crate::regs::wfdma0::GLO_CFG_EXT1_TX_FCTRL_MODE;
    /// CALC_MODE bit - MUST be set after prefetch configuration
    pub const GLO_CFG_EXT1_CALC_MODE: u32 = crate::regs::wfdma0::GLO_CFG_EXT1_CALC_MODE;

    /// TX ring base offset (from WFDMA base) - from regs::wfdma0
    /// All TX rings (MCU and data) use this base + hw_idx * 0x10
    pub const MCU_RING_BASE: u32 = crate::regs::wfdma0::TX_RING_BASE;
    /// RX queue ring base offset - from regs::wfdma0
    pub const RX_RING_BASE: u32 = crate::regs::wfdma0::RX_RING_BASE;

    /// Data TX queue hardware indices - from mt7996_defs::Mt7996TxqId
    /// All TX rings use MCU_RING_BASE + hw_idx * 0x10
    pub const TXQ_BAND0: u32 = crate::mt7996_defs::Mt7996TxqId::Band0 as u32;  // 18
    pub const TXQ_BAND1: u32 = crate::mt7996_defs::Mt7996TxqId::Band1 as u32;  // 19 (HIF2)
    pub const TXQ_BAND2: u32 = crate::mt7996_defs::Mt7996TxqId::Band2 as u32;  // 21 (HIF2)
    /// Data TX ring size - from mt7996_defs (MT7996_TX_RING_SIZE)
    pub const TX_RING_SIZE: u32 = crate::mt7996_defs::MT7996_TX_RING_SIZE;
    /// TX MCU ring size - from mt7996_defs (MT7996_TX_MCU_RING_SIZE)
    pub const TX_MCU_RING_SIZE: u32 = crate::mt7996_defs::MT7996_TX_MCU_RING_SIZE;

    /// RX MCU queue IDs - from mt7996_defs::rxq_id
    pub const RXQ_MCU: u32 = crate::mt7996_defs::rxq_id::MCU_WM;
    pub const RXQ_MCU_WA: u32 = crate::mt7996_defs::rxq_id::MCU_WA;
    pub const RXQ_MCU_WA_MAIN: u32 = crate::mt7996_defs::rxq_id::MCU_WA_MAIN;
    pub const RXQ_MCU_WA_TRI: u32 = crate::mt7996_defs::rxq_id::MCU_WA_TRI;
    /// RX data queue IDs - from mt7996_defs::rxq_id
    pub const RXQ_BAND0: u32 = crate::mt7996_defs::rxq_id::BAND0;
    pub const RXQ_BAND2: u32 = crate::mt7996_defs::rxq_id::BAND2;
    /// RRO queue IDs - from mt7996_defs::rxq_id
    pub const RXQ_RRO_BAND2: u32 = crate::mt7996_defs::rxq_id::RRO_BAND2;
    pub const RXQ_RRO_BAND0: u32 = crate::mt7996_defs::rxq_id::RRO_BAND0;
    /// RX TXFREE queue IDs - from mt7996_defs::rxq_id
    /// CRITICAL: Required for DMA init, hardware validates before RST release!
    pub const RXQ_TXFREE_BAND0: u32 = crate::mt7996_defs::rxq_id::TXFREE0;
    pub const RXQ_TXFREE_BAND2: u32 = crate::mt7996_defs::rxq_id::TXFREE2;

    /// RX MCU ring size - from mt7996_defs (MT7996_RX_MCU_RING_SIZE)
    pub const RX_MCU_RING_SIZE: u32 = crate::mt7996_defs::MT7996_RX_MCU_RING_SIZE;
    /// RX MCU WA ring size - from mt7996_defs (MT7996_RX_MCU_RING_SIZE_WA)
    pub const RX_MCU_RING_SIZE_WA: u32 = crate::mt7996_defs::MT7996_RX_MCU_RING_SIZE_WA;
    /// RX data ring size - from mt7996_defs (MT7996_RX_RING_SIZE)
    pub const RX_DATA_RING_SIZE: u32 = crate::mt7996_defs::MT7996_RX_RING_SIZE;
    /// RX TXFREE ring size - uses RX data size per Linux
    pub const RX_TXFREE_RING_SIZE: u32 = crate::mt7996_defs::MT7996_RX_RING_SIZE;
    /// RX RRO ring size - matches RX data size
    pub const RX_RRO_RING_SIZE: u32 = crate::mt7996_defs::MT7996_RX_RING_SIZE;

    /// Ring register offsets (16 bytes per ring, NO base_hi!)
    pub const RING_BASE: u32 = 0x00;     // desc_base (32-bit address)
    pub const RING_MAX_CNT: u32 = 0x04;  // ring_size
    pub const RING_CPU_IDX: u32 = 0x08;  // cpu_idx
    pub const RING_DMA_IDX: u32 = 0x0c;  // dma_idx
    // Note: offset 0x10 is the NEXT ring's desc_base, not base_hi!

    /// Ring size for firmware download - from mt7996_defs (MT7996_TX_FWDL_RING_SIZE)
    pub const FWDL_RING_SIZE: u32 = crate::mt7996_defs::MT7996_TX_FWDL_RING_SIZE;

    /// Extended control register offsets (relative to WFDMA base)
    /// All TX queue prefetch control (MCU AND Data TX share the same base!)
    /// Linux: MT_MCUQ_EXT_CTRL(q) = MT_Q_BASE(q) + 0x600 + MT_MCUQ_ID(q) * 0x4
    /// Linux: MT_TXQ_EXT_CTRL(q) = MT_Q_BASE(__TXQ(q)) + 0x600 + MT_TXQ_ID(q) * 0x4
    /// Both use 0x600 base - the hw ring index (16-21) determines the actual offset
    pub const TX_EXT_CTRL_BASE: u32 = 0x600;
    /// Alias for backwards compatibility
    pub const MCUQ_EXT_CTRL_BASE: u32 = TX_EXT_CTRL_BASE;
    /// REMOVED: TXQ_EXT_CTRL_BASE was 0x640 which is WRONG!
    /// Linux uses 0x600 for both MCU and Data TX queues
    pub const TXQ_EXT_CTRL_BASE: u32 = TX_EXT_CTRL_BASE;  // Was 0x640, now fixed to 0x600
    /// RX queue prefetch control
    /// Linux: MT_RXQ_EXT_CTRL(q) = MT_Q_BASE(__RXQ(q)) + 0x680 + MT_RXQ_ID(q) * 0x4
    pub const RXQ_EXT_CTRL_BASE: u32 = 0x680;

    /// For backwards compatibility
    pub const EXT_CTRL_BASE: u32 = MCUQ_EXT_CTRL_BASE;

    /// Prefetch depth for MT7990 MCU queues
    pub const PREFETCH_DEPTH_MCU_MT7990: u32 = 4;

    /// Validate wfdma constants against regs.rs (catches drift)
    pub fn validate_against_regs() {
        use crate::regs;

        // WFDMA0 base addresses
        assert_eq!(WFDMA0_BASE, regs::wfdma0::BASE, "WFDMA0_BASE mismatch with regs.rs");
        assert_eq!(WFDMA0_PCIE1_BASE, regs::wfdma0_pcie1::BASE, "WFDMA0_PCIE1_BASE mismatch");

        // Reset register
        assert_eq!(WFDMA0_BASE + RST, regs::wfdma0::RST, "RST address mismatch");
        assert_eq!(RST_LOGIC_RST, regs::wfdma0::RST_LOGIC_RST, "RST_LOGIC_RST mismatch");
        assert_eq!(RST_DMASHDL_ALL_RST, regs::wfdma0::RST_DMASHDL_ALL_RST, "RST_DMASHDL_ALL_RST mismatch");

        // GLO_CFG
        assert_eq!(WFDMA0_BASE + GLO_CFG, regs::wfdma0::GLO_CFG, "GLO_CFG address mismatch");

        // Pause RX thresholds (the one we fixed!)
        assert_eq!(WFDMA0_BASE + PAUSE_RX_Q_45_TH, regs::wfdma0::PAUSE_RX_Q_45_TH, "PAUSE_RX_Q_45_TH mismatch");
        assert_eq!(WFDMA0_BASE + PAUSE_RX_Q_67_TH, regs::wfdma0::PAUSE_RX_Q_67_TH, "PAUSE_RX_Q_67_TH mismatch");
        assert_eq!(WFDMA0_BASE + PAUSE_RX_Q_89_TH, regs::wfdma0::PAUSE_RX_Q_89_TH, "PAUSE_RX_Q_89_TH mismatch");
        assert_eq!(WFDMA0_BASE + PAUSE_RX_Q_RRO_TH, regs::wfdma0::PAUSE_RX_Q_RRO_TH, "PAUSE_RX_Q_RRO_TH mismatch");

        // GLO_CFG extensions
        assert_eq!(WFDMA0_BASE + GLO_CFG_EXT0, regs::wfdma0::GLO_CFG_EXT0, "GLO_CFG_EXT0 address mismatch");
        assert_eq!(WFDMA0_BASE + GLO_CFG_EXT1, regs::wfdma0::GLO_CFG_EXT1, "GLO_CFG_EXT1 address mismatch");
        assert_eq!(GLO_CFG_EXT1_CALC_MODE, regs::wfdma0::GLO_CFG_EXT1_CALC_MODE, "GLO_CFG_EXT1_CALC_MODE mismatch");

        // Extended CSR
        assert_eq!(EXT_CSR_HIF_MISC, regs::wfdma_ext_csr::HIF_MISC, "EXT_CSR_HIF_MISC mismatch");
        assert_eq!(HOST_CONFIG, regs::wfdma_ext_csr::HOST_CONFIG, "HOST_CONFIG mismatch");

        // Prefetch control bases - the bug we found!
        // Both MCU and Data TX use 0x600, RX uses 0x680
        assert_eq!(TX_EXT_CTRL_BASE, regs::wfdma0::TX_EXT_CTRL_BASE, "TX_EXT_CTRL_BASE mismatch");
        assert_eq!(RXQ_EXT_CTRL_BASE, regs::wfdma0::RX_EXT_CTRL_BASE, "RXQ_EXT_CTRL_BASE mismatch");

        // Ring bases
        assert_eq!(MCU_RING_BASE, regs::wfdma0::TX_RING_BASE, "MCU_RING_BASE mismatch");
        assert_eq!(RX_RING_BASE, regs::wfdma0::RX_RING_BASE, "RX_RING_BASE mismatch");

        // Busy enable bits
        assert_eq!(BUSY_ENA_TX_FIFO0, regs::wfdma0::BUSY_ENA_TX_FIFO0, "BUSY_ENA_TX_FIFO0 mismatch");
        assert_eq!(BUSY_ENA_TX_FIFO1, regs::wfdma0::BUSY_ENA_TX_FIFO1, "BUSY_ENA_TX_FIFO1 mismatch");
        assert_eq!(BUSY_ENA_RX_FIFO, regs::wfdma0::BUSY_ENA_RX_FIFO, "BUSY_ENA_RX_FIFO mismatch");
    }
}

/// Interrupt mask bits for WFDMA
/// From Linux drivers/net/wireless/mediatek/mt76/mt7996/regs.h
pub mod int_mask {
    // RX done interrupts
    pub const RX_DONE_WM: u32 = 1 << 0;
    pub const RX_DONE_WA: u32 = 1 << 1;
    pub const RX_DONE_WA_MAIN: u32 = 1 << 2;
    pub const RX_DONE_WA_TRI: u32 = 1 << 3;
    pub const RX_DONE_BAND0: u32 = 1 << 12;
    pub const RX_DONE_BAND1: u32 = 1 << 13;
    pub const RX_DONE_BAND2: u32 = 1 << 13;
    pub const RX_DONE_BAND2_EXT: u32 = 1 << 23;

    // TX done interrupts
    pub const TX_DONE_MCU_WM: u32 = 1 << 27;
    pub const TX_DONE_MCU_WA: u32 = 1 << 22;
    pub const TX_DONE_FWDL: u32 = 1 << 26;
    pub const TX_DONE_BAND0: u32 = 1 << 30;
    pub const TX_DONE_BAND1: u32 = 1 << 31;
    pub const TX_DONE_BAND2: u32 = 1 << 15;

    // MCU command interrupt
    pub const MCU_CMD: u32 = 1 << 29;

    /// Minimal mask for firmware download (TX_DONE_FWDL + RX_DONE_WM)
    pub const FWDL_MASK: u32 = TX_DONE_FWDL | RX_DONE_WM;

    /// Full MCU interrupt mask (matches OpenWrt 0x2cc0b00f)
    pub const MCU_FULL: u32 = RX_DONE_WM | RX_DONE_WA | RX_DONE_WA_MAIN | RX_DONE_WA_TRI |
                              RX_DONE_BAND0 | RX_DONE_BAND1 | TX_DONE_BAND2 |
                              TX_DONE_MCU_WA | RX_DONE_BAND2_EXT |
                              TX_DONE_FWDL | TX_DONE_MCU_WM | MCU_CMD;

    /// Validate interrupt bits against regs.rs
    pub fn validate_against_regs() {
        use crate::regs;

        assert_eq!(RX_DONE_WM, regs::int::RX_DONE_WM, "RX_DONE_WM mismatch");
        assert_eq!(RX_DONE_WA, regs::int::RX_DONE_WA, "RX_DONE_WA mismatch");
        assert_eq!(RX_DONE_WA_MAIN, regs::int::RX_DONE_WA_MAIN, "RX_DONE_WA_MAIN mismatch");
        assert_eq!(RX_DONE_WA_TRI, regs::int::RX_DONE_WA_TRI, "RX_DONE_WA_TRI mismatch");
        assert_eq!(RX_DONE_BAND0, regs::int::RX_DONE_BAND0, "RX_DONE_BAND0 mismatch");
        assert_eq!(RX_DONE_BAND2_EXT, regs::int::RX_DONE_BAND2_EXT, "RX_DONE_BAND2_EXT mismatch");

        assert_eq!(TX_DONE_MCU_WM, regs::int::TX_DONE_MCU_WM, "TX_DONE_MCU_WM mismatch");
        assert_eq!(TX_DONE_MCU_WA, regs::int::TX_DONE_MCU_WA, "TX_DONE_MCU_WA mismatch");
        assert_eq!(TX_DONE_FWDL, regs::int::TX_DONE_FWDL, "TX_DONE_FWDL mismatch");
        assert_eq!(TX_DONE_BAND0, regs::int::TX_DONE_BAND0, "TX_DONE_BAND0 mismatch");
        assert_eq!(TX_DONE_BAND1, regs::int::TX_DONE_BAND1, "TX_DONE_BAND1 mismatch");
        assert_eq!(TX_DONE_BAND2, regs::int::TX_DONE_BAND2, "TX_DONE_BAND2 mismatch");

        assert_eq!(MCU_CMD, regs::int::MCU_CMD, "MCU_CMD mismatch");
    }
}

/// MT7996 MCU TX queue IDs (for TXD packet header Q_IDX field)
/// These go into the TXD0 Q_IDX field and indicate the destination RX queue on MCU.
/// Linux: MT_TX_MCU_PORT_RX_Q0 = 0x20 is used for all MCU commands.
/// The physical TX queue (FWDL/WM/WA) is selected by which ring the descriptor goes to.
pub mod txq_id {
    /// MCU RX queue 0 - used for all MCU commands
    pub const MCU_RX_Q0: u32 = 0x20;  // MT_TX_MCU_PORT_RX_Q0

    /// Firmware download - still route to MCU RX Q0
    pub const FWDL: u32 = MCU_RX_Q0;
    pub const WM: u32 = MCU_RX_Q0;
    pub const WA: u32 = MCU_RX_Q0;
}

/// MT7996 MCU queue logical IDs (matches Linux mt76_mcuq_id enum)
/// These are software indices (0, 1, 2) that map to hardware ring indices.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McuQueue {
    /// WM (WiFi Manager) - logical index 0
    Wm = 0,
    /// WA (WiFi Agent) - logical index 1
    Wa = 1,
    /// Firmware download - logical index 2
    Fwdl = 2,
}

/// MT7996 hardware TX queue indices - from mt7996_defs::Mt7996TxqId
/// These are the actual hardware ring numbers used for register access.
pub mod hw_ring_idx {
    use crate::mt7996_defs::Mt7996TxqId;
    /// Firmware download - hardware ring 16
    pub const FWDL: u32 = Mt7996TxqId::Fwdl as u32;
    /// WM (WiFi Manager) - hardware ring 17
    pub const WM: u32 = Mt7996TxqId::McuWm as u32;
    /// WA (WiFi Agent) - hardware ring 20
    pub const WA: u32 = Mt7996TxqId::McuWa as u32;
}

impl McuQueue {
    /// Get hardware ring index for this MCU queue
    /// Linux: dev->q_id[MT_MCUQ_*] = MT7996_TXQ_*
    #[inline]
    pub const fn hw_ring_idx(self) -> u32 {
        match self {
            McuQueue::Wm => hw_ring_idx::WM,
            McuQueue::Wa => hw_ring_idx::WA,
            McuQueue::Fwdl => hw_ring_idx::FWDL,
        }
    }

    /// Get ring register base offset from WFDMA base
    /// Linux: q->regs = base + ring_base + idx * MT_RING_SIZE
    /// where ring_base = MT_MCUQ_RING_BASE(q) = MT_Q_BASE(q) + 0x300
    /// and idx = MT_MCUQ_ID(q) = hardware ring index
    #[inline]
    pub const fn ring_offset(self) -> u32 {
        wfdma::MCU_RING_BASE + self.hw_ring_idx() * 0x10
    }

    /// Get prefetch control register offset from WFDMA base
    /// Linux: MT_MCUQ_EXT_CTRL(q) = MT_Q_BASE(q) + 0x600 + MT_MCUQ_ID(q) * 0x4
    #[inline]
    pub const fn ext_ctrl_offset(self) -> u32 {
        wfdma::MCUQ_EXT_CTRL_BASE + self.hw_ring_idx() * 4
    }

    /// Get TXD queue ID for this MCU queue (for TXD0 Q_IDX field)
    #[inline]
    pub const fn txq_id(self) -> u32 {
        match self {
            McuQueue::Wm => txq_id::WM,
            McuQueue::Wa => txq_id::WA,
            McuQueue::Fwdl => txq_id::FWDL,
        }
    }
}

/// Typed TX queue index (prevents mixing with RX queues or raw offsets)
/// Uses hardware ring indices (16, 17, 20) for correct register access.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxQ(pub u8);

impl TxQ {
    /// Hardware ring indices (for register access)
    /// These match Linux mt7996_txq_id enum values
    pub const WM: TxQ = TxQ(hw_ring_idx::WM as u8);
    pub const WA: TxQ = TxQ(hw_ring_idx::WA as u8);
    pub const FWDL: TxQ = TxQ(hw_ring_idx::FWDL as u8);

    /// Get ring register base offset from WFDMA base
    #[inline]
    pub const fn ring_offset(self) -> u32 {
        wfdma::MCU_RING_BASE + (self.0 as u32) * 0x10
    }
}

/// Typed RX queue index
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RxQ(pub u8);

impl RxQ {
    pub const MCU: RxQ = RxQ(0);

    /// Get ring register base offset from WFDMA base
    #[inline]
    pub const fn ring_offset(self) -> u32 {
        wfdma::RX_RING_BASE + (self.0 as u32) * 0x10
    }

    /// Get prefetch control register offset from WFDMA base
    #[inline]
    pub const fn ext_ctrl_offset(self) -> u32 {
        wfdma::RXQ_EXT_CTRL_BASE + (self.0 as u32) * 4
    }
}

/// WFDMA controller state machine
///
/// Explicit states prevent "works only if you call X before Y" bugs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WfdmaState {
    /// Not initialized - just created
    Uninit,
    /// Reset complete, ready for ring allocation
    Reset,
    /// Rings allocated and configured
    RingsConfigured,
    /// DMA engines enabled, ready for transfers
    Ready,
    /// Error state - needs full reset
    Error,
}

/// MCU command flags (for MCU_CMD register at WFDMA0+0x1f0)
pub mod mcu_cmd {
    /// Stop DMA
    pub const STOP_DMA: u32 = 1 << 2;
    /// Reset done
    pub const RESET_DONE: u32 = 1 << 3;
    /// Recovery done
    pub const RECOVERY_DONE: u32 = 1 << 4;
    /// Normal state (firmware running)
    pub const NORMAL_STATE: u32 = 1 << 5;
    /// Error mask (bits 5-1)
    pub const ERROR_MASK: u32 = 0x3e;
    /// WM watchdog timer
    pub const WM_WDT: u32 = 1 << 30;
    /// WA watchdog timer
    pub const WA_WDT: u32 = 1 << 31;
    /// Combined WDT mask
    pub const WDT_MASK: u32 = WM_WDT | WA_WDT;
}

/// MCU interrupt event register (at 0x2108)
pub mod mcu_int {
    /// MCU interrupt event register address
    pub const EVENT: u32 = 0x2108;
    /// DMA stopped
    pub const DMA_STOPPED: u32 = 1 << 0;
    /// DMA init
    pub const DMA_INIT: u32 = 1 << 1;
    /// Reset done
    pub const RESET_DONE: u32 = 1 << 3;
}

/// MCU command IDs
pub mod mcu {
    pub const CMD_FW_SCATTER: u8 = 0xee;
    pub const CMD_TARGET_ADDRESS_LEN_REQ: u8 = 0x01;
    pub const CMD_FW_START_REQ: u8 = 0x02;
    pub const CMD_PATCH_START_REQ: u8 = 0x05;
    pub const CMD_PATCH_SEM_CTRL: u8 = 0x11;
    pub const CMD_PATCH_FINISH_REQ: u8 = 0x07;

    /// MCU packet type
    pub const PKT_ID: u8 = 0xa0;

    /// Source to destination index (host to MCU)
    pub const S2D_H2N: u8 = 0x00;

    /// Patch semaphore operations
    pub const PATCH_SEM_GET: u32 = 0x01;
    pub const PATCH_SEM_RELEASE: u32 = 0x00;

    /// Patch load mode
    pub const PATCH_MODE_ENCRYPTED: u32 = 0x01;
    pub const PATCH_MODE_NORMAL: u32 = 0x00;
}

/// Patch header structure (mt76_connac2_patch_hdr)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PatchHeader {
    pub build_date: [u8; 16],
    pub platform: [u8; 4],
    pub hw_sw_ver: u32,  // big-endian
    pub patch_ver: u32,  // big-endian
    pub checksum: u16,   // big-endian
    pub rsv: u16,
    // descriptor
    pub desc_patch_ver: u32,  // big-endian
    pub desc_subsys: u32,     // big-endian
    pub desc_feature: u32,    // big-endian
    pub desc_n_region: u32,   // big-endian
    pub desc_crc: u32,
    pub desc_rsv: [u32; 11],
}

impl PatchHeader {
    /// Get number of regions
    pub fn n_region(&self) -> u32 {
        u32::from_be(self.desc_n_region)
    }
}

/// Patch section structure (mt76_connac2_patch_sec)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PatchSection {
    pub sec_type: u32,  // big-endian
    pub offs: u32,      // big-endian
    pub size: u32,      // big-endian
    // info union
    pub addr: u32,          // big-endian - target address
    pub len: u32,           // big-endian - data length
    pub sec_key_idx: u32,   // big-endian - security key index
    pub align_len: u32,     // big-endian
    pub rsv: [u32; 9],
}

impl PatchSection {
    /// Get target address
    pub fn target_addr(&self) -> u32 {
        u32::from_be(self.addr)
    }

    /// Get data length
    pub fn data_len(&self) -> u32 {
        u32::from_be(self.len)
    }

    /// Get section offset in file
    pub fn offset(&self) -> u32 {
        u32::from_be(self.offs)
    }

    /// Get section size in file
    pub fn sec_size(&self) -> u32 {
        u32::from_be(self.size)
    }
}

/// TXD field definitions (MT7996 connac2/connac3 bit positions)
/// Verified via deepwiki 2024-12-30
pub mod txd {
    /// TXD0: TX_BYTES field (bits 15:0)
    pub const TXD0_TX_BYTES_MASK: u32 = 0xFFFF;
    /// TXD0: PKT_FMT field (bits 24:23) - 2 = CMD
    pub const TXD0_PKT_FMT_CMD: u32 = 2 << 23;
    /// TXD0: Q_IDX field (bits 31:25)
    pub const TXD0_Q_IDX_SHIFT: u32 = 25;

    /// TXD1: LONG_FORMAT (bit 31) - must be set for command packets
    pub const TXD1_LONG_FORMAT: u32 = 1 << 31;
    /// TXD1: HDR_FORMAT field (bits 17:16) - 1 = CMD
    /// Note: NOT bits 15:14! That was for older chipsets.
    pub const TXD1_HDR_FORMAT_CMD: u32 = 1 << 16;
    /// Combined TXD1 value for command packets
    pub const TXD1_CMD: u32 = TXD1_LONG_FORMAT | TXD1_HDR_FORMAT_CMD;
}

/// Global config register bits - from regs::wfdma0
pub mod glo_cfg {
    /// Enable TX DMA - from regs::wfdma0
    pub const TX_DMA_EN: u32 = crate::regs::wfdma0::GLO_CFG_TX_DMA_EN;
    /// Enable RX DMA - from regs::wfdma0
    pub const RX_DMA_EN: u32 = crate::regs::wfdma0::GLO_CFG_RX_DMA_EN;
    /// TX DMA busy
    pub const TX_DMA_BUSY: u32 = 1 << 1;
    /// RX DMA busy
    pub const RX_DMA_BUSY: u32 = 1 << 3;
    /// Byte swap - MUST be cleared for little-endian descriptors!
    pub const BYTE_SWAP: u32 = 1 << 4;
    /// DMA descriptor size (0 = 8 DWORDs, 1 = 16 DWORDs)
    pub const DESC_SIZE_16DW: u32 = 1 << 8;
    /// Omit RX info in prefetch - from regs::wfdma0
    pub const OMIT_RX_INFO_PFET2: u32 = crate::regs::wfdma0::GLO_CFG_OMIT_RX_INFO_PFET2;
    /// External enable (required for firmware loading!) - from regs::wfdma0
    pub const EXT_EN: u32 = crate::regs::wfdma0::GLO_CFG_EXT_EN;
    /// Omit RX info - from regs::wfdma0
    pub const OMIT_RX_INFO: u32 = crate::regs::wfdma0::GLO_CFG_OMIT_RX_INFO;
    /// Omit TX info - from regs::wfdma0
    pub const OMIT_TX_INFO: u32 = crate::regs::wfdma0::GLO_CFG_OMIT_TX_INFO;
}

/// Ring descriptor control field bits - from dma_defs
pub mod dma_ctl {
    /// Segment data length 0 (bits 29:16) - from dma_defs
    pub const SD_LEN0_SHIFT: u32 = 16;
    pub const SD_LEN0_MASK: u32 = crate::dma_defs::MT_DMA_CTL_SD_LEN0;
    /// Last segment indicator for buffer 0 - from dma_defs
    pub const LAST_SEC0: u32 = crate::dma_defs::MT_DMA_CTL_LAST_SEC0;
    /// DMA done flag - from dma_defs
    pub const DMA_DONE: u32 = crate::dma_defs::MT_DMA_CTL_DMA_DONE;

    /// RRO magic counter constants - from dma_defs
    /// MT_DMA_MAGIC_CNT = 16, initial value is CNT-1 = 15
    pub const RRO_MAGIC_CNT: u32 = crate::dma_defs::MT_DMA_MAGIC_CNT;
    pub const RRO_MAGIC_CNT_INIT: u32 = crate::dma_defs::MT_DMA_MAGIC_CNT - 1;
    /// Magic counter mask in info field (bits 31:28 per Linux dma.h MT_DMA_MAGIC_MASK)
    pub const RRO_MAGIC_SHIFT: u32 = 28;  // Corrected: was 12, should be 28
    pub const RRO_MAGIC_MASK: u32 = crate::dma_defs::MT_DMA_MAGIC_MASK;
}

/// DMA ring descriptor (mt76_desc format - 16 bytes)
/// This is what goes in the DMA ring to point to the actual packet buffer
#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct TxDesc {
    /// Buffer 0 address (low 32 bits)
    pub buf0: u32,
    /// Control: SD_LEN0(29:16), LAST_SEC0(30), DMA_DONE(31)
    pub ctrl: u32,
    /// Buffer 1 address / high bits
    pub buf1: u32,
    /// Info field
    pub info: u32,
}

/// MCU command TXD header (prepended to MCU command packets)
/// 32 bytes TXD + 28 bytes MCU fields = 60 bytes total
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct McuTxd {
    /// TXD header (8 DWORDs)
    pub txd: [u32; 8],
    /// Payload length (excluding this header)
    pub len: u16,
    /// Port/Queue ID
    pub pq_id: u16,
    /// Command ID
    pub cid: u8,
    /// Packet type (0xa0 for MCU)
    pub pkt_type: u8,
    /// Set/Query flag
    pub set_query: u8,
    /// Sequence number
    pub seq: u8,
    /// Reserved
    pub uc_d2b0_rev: u8,
    /// Extended command ID
    pub ext_cid: u8,
    /// Source to destination index
    pub s2d_index: u8,
    /// Extended CID ack
    pub ext_cid_ack: u8,
    /// Reserved
    pub rsv: [u32; 5],
}

impl Default for McuTxd {
    fn default() -> Self {
        Self {
            txd: [0; 8],
            len: 0,
            pq_id: 0,
            cid: 0,
            pkt_type: 0,
            set_query: 0,
            seq: 0,
            uc_d2b0_rev: 0,
            ext_cid: 0,
            s2d_index: 0,
            ext_cid_ack: 0,
            rsv: [0; 5],
        }
    }
}

impl McuTxd {
    /// Build MCU TXD for a command
    /// queue_id: MCU destination RX queue (0x20 = MT_TX_MCU_PORT_RX_Q0 for all MCU commands)
    /// The physical TX ring (FWDL/WM/WA) is selected by which ring the descriptor is placed in.
    pub fn new(cmd_id: u8, payload_len: usize, seq: u8, queue_id: u32) -> Self {
        let total_len = core::mem::size_of::<McuTxd>() + payload_len;
        let mut txd = Self::default();

        // TXD0: length | PKT_FMT=CMD | Q_IDX
        // Q_IDX must match the hardware queue we're submitting to!
        txd.txd[0] = ((total_len as u32) & txd::TXD0_TX_BYTES_MASK)
            | txd::TXD0_PKT_FMT_CMD
            | (queue_id << txd::TXD0_Q_IDX_SHIFT);

        // TXD1: LONG_FORMAT + HDR_FORMAT=CMD
        // MT7996 (connac2/connac3): LONG_FORMAT=bit31, HDR_FORMAT=bits17:16
        txd.txd[1] = txd::TXD1_CMD;

        // MCU fields
        txd.len = (payload_len as u16).to_le();
        txd.pq_id = 0; // Will be set by hardware
        txd.cid = cmd_id;
        txd.pkt_type = mcu::PKT_ID;
        txd.seq = seq;
        txd.s2d_index = mcu::S2D_H2N;

        txd
    }
}

impl TxDesc {
    /// Create a new descriptor for MCU command
    pub fn for_firmware(buf_phys: u64, len: usize, last: bool) -> Self {
        let mut desc = Self::default();

        // Buffer 0 address
        desc.buf0 = buf_phys as u32;

        // Control: length in bits 29:16, last segment flag in bit 30
        desc.ctrl = ((len as u32) << dma_ctl::SD_LEN0_SHIFT) & dma_ctl::SD_LEN0_MASK;
        if last {
            desc.ctrl |= dma_ctl::LAST_SEC0;
        }

        // Buffer 1 (high bits of 64-bit address)
        desc.buf1 = (buf_phys >> 32) as u32;

        // Info field (unused for simple TX)
        desc.info = 0;

        desc
    }
}

/// Size of MCU command buffer (header + max payload)
const MCU_CMD_BUF_SIZE: usize = core::mem::size_of::<McuTxd>() + 4096;

/// DMA ring for firmware download
pub struct FwdlRing {
    /// Virtual address of descriptor ring
    pub desc_vaddr: u64,
    /// Physical address of descriptor ring
    pub desc_paddr: u64,
    /// Virtual address of command buffer
    pub cmd_vaddr: u64,
    /// Physical address of command buffer
    pub cmd_paddr: u64,
    /// Number of descriptors
    pub size: u32,
    /// Current CPU index (write pointer)
    pub cpu_idx: u32,
    /// Command sequence number
    pub seq: u8,
}

impl FwdlRing {
    /// Create a new firmware download ring with default size
    pub fn new() -> Option<Self> {
        Self::new_with_size(wfdma::FWDL_RING_SIZE)
    }

    /// Create a new ring with specified size
    pub fn new_with_size(size: u32) -> Option<Self> {
        let size = size;
        let desc_size = (size as usize) * core::mem::size_of::<TxDesc>();

        // Allocate DMA memory for descriptors
        let mut desc_paddr: u64 = 0;
        let desc_vaddr = syscall::mmap_dma(desc_size, &mut desc_paddr);
        // Check for error (negative) AND invalid address (0 or too low)
        if desc_vaddr <= 0 || (desc_vaddr as u64) < 0x1000 {
            userlib::println!("[FwdlRing] desc alloc failed: ret={}, size={}", desc_vaddr, desc_size);
            return None;
        }

        // Zero the descriptor ring
        unsafe {
            core::ptr::write_bytes(desc_vaddr as *mut u8, 0, desc_size);
        }

        // Allocate DMA memory for command buffer
        let mut cmd_paddr: u64 = 0;
        let cmd_vaddr = syscall::mmap_dma(MCU_CMD_BUF_SIZE, &mut cmd_paddr);
        // Check for error (negative) AND invalid address (0 or too low)
        if cmd_vaddr <= 0 || (cmd_vaddr as u64) < 0x1000 {
            userlib::println!("[FwdlRing] cmd alloc failed: ret={}, size={}", cmd_vaddr, MCU_CMD_BUF_SIZE);
            syscall::munmap(desc_vaddr as u64, desc_size);
            return None;
        }

        Some(Self {
            desc_vaddr: desc_vaddr as u64,
            desc_paddr,
            cmd_vaddr: cmd_vaddr as u64,
            cmd_paddr,
            size,
            cpu_idx: 0,
            seq: 0,
        })
    }

    /// Get a descriptor by index
    pub fn get_desc(&self, idx: u32) -> &mut TxDesc {
        let descs = self.desc_vaddr as *mut TxDesc;
        unsafe { &mut *descs.add(idx as usize) }
    }

    /// Advance CPU index
    pub fn advance(&mut self) {
        self.cpu_idx = (self.cpu_idx + 1) % self.size;
    }

    /// Get next sequence number
    pub fn next_seq(&mut self) -> u8 {
        let s = self.seq;
        self.seq = self.seq.wrapping_add(1);
        s
    }

    /// Build FW_SCATTER raw data in buffer (NO TXD header!)
    /// Per deepwiki: FW_SCATTER is the ONLY MCU command that skips TXD header.
    /// The `goto exit` in mt7996_mcu_send_message() bypasses TXD preparation.
    /// Returns physical address and total length of raw data.
    pub fn build_fw_scatter(&mut self, fw_data: &[u8]) -> (u64, usize) {
        let data_len = fw_data.len();

        // Copy raw firmware data directly to command buffer (NO TXD header!)
        let buf_ptr = self.cmd_vaddr as *mut u8;
        unsafe {
            core::ptr::copy_nonoverlapping(fw_data.as_ptr(), buf_ptr, data_len);
        }

        // Flush buffer to RAM so DMA can see it
        flush_buffer(self.cmd_vaddr, data_len);

        // Advance sequence number (for consistency, though not used in packet)
        let _ = self.next_seq();

        (self.cmd_paddr, data_len)
    }

    /// Build TARGET_ADDRESS_LEN_REQ command
    /// This tells the MCU where to load the firmware data
    /// Returns physical address and total length of command
    pub fn build_init_download(&mut self, addr: u32, len: u32, mode: u32) -> (u64, usize) {
        // Payload is 3 u32s: addr, len, mode
        let payload_len = 12;

        // For MT799x at address 0x900000 (ROM patch), use PATCH_START_REQ
        // Otherwise use TARGET_ADDRESS_LEN_REQ
        let cmd_id = if addr == 0x900000 {
            mcu::CMD_PATCH_START_REQ
        } else {
            mcu::CMD_TARGET_ADDRESS_LEN_REQ
        };

        // Use TXQ ID (16) for TXD header, NOT ring index (2)
        let header = McuTxd::new(cmd_id, payload_len, self.next_seq(), txq_id::FWDL);

        // Write header to command buffer
        let buf_ptr = self.cmd_vaddr as *mut McuTxd;
        unsafe {
            core::ptr::write(buf_ptr, header);
        }

        // Write payload (addr, len, mode) as little-endian
        let data_ptr = (self.cmd_vaddr as usize + core::mem::size_of::<McuTxd>()) as *mut u32;
        unsafe {
            core::ptr::write(data_ptr, addr.to_le());
            core::ptr::write(data_ptr.add(1), len.to_le());
            core::ptr::write(data_ptr.add(2), mode.to_le());
        }

        let total_len = core::mem::size_of::<McuTxd>() + payload_len;

        // Flush command buffer to RAM so DMA can see it
        flush_buffer(self.cmd_vaddr, total_len);

        (self.cmd_paddr, total_len)
    }

    /// Build PATCH_SEM_CTRL command (acquire/release semaphore)
    pub fn build_patch_sem_ctrl(&mut self, get: bool) -> (u64, usize) {
        // Payload is 1 u32: operation (GET=1, RELEASE=0)
        let payload_len = 4;
        // Use TXQ ID (16) for TXD header, NOT ring index (2)
        let header = McuTxd::new(mcu::CMD_PATCH_SEM_CTRL, payload_len, self.next_seq(), txq_id::FWDL);

        let buf_ptr = self.cmd_vaddr as *mut McuTxd;
        unsafe {
            core::ptr::write(buf_ptr, header);
        }

        let data_ptr = (self.cmd_vaddr as usize + core::mem::size_of::<McuTxd>()) as *mut u32;
        let op = if get { mcu::PATCH_SEM_GET } else { mcu::PATCH_SEM_RELEASE };
        unsafe {
            core::ptr::write(data_ptr, op.to_le());
        }

        let total_len = core::mem::size_of::<McuTxd>() + payload_len;

        // Flush command buffer to RAM so DMA can see it
        flush_buffer(self.cmd_vaddr, total_len);

        (self.cmd_paddr, total_len)
    }

    /// Build PATCH_FINISH_REQ command
    pub fn build_patch_finish(&mut self) -> (u64, usize) {
        // No payload
        // Use TXQ ID (16) for TXD header, NOT ring index (2)
        let header = McuTxd::new(mcu::CMD_PATCH_FINISH_REQ, 0, self.next_seq(), txq_id::FWDL);

        let buf_ptr = self.cmd_vaddr as *mut McuTxd;
        unsafe {
            core::ptr::write(buf_ptr, header);
        }

        let total_len = core::mem::size_of::<McuTxd>();

        // Flush command buffer to RAM so DMA can see it
        flush_buffer(self.cmd_vaddr, total_len);

        (self.cmd_paddr, total_len)
    }
}

impl Drop for FwdlRing {
    fn drop(&mut self) {
        let desc_size = (self.size as usize) * core::mem::size_of::<TxDesc>();
        syscall::munmap(self.desc_vaddr, desc_size);
        syscall::munmap(self.cmd_vaddr, MCU_CMD_BUF_SIZE);
    }
}

/// RX buffer size for MCU events
const RX_BUF_SIZE: usize = 2048;

/// Simple RX ring for MCU responses
pub struct RxRing {
    /// Virtual address of descriptor ring
    pub desc_vaddr: u64,
    /// Physical address of descriptor ring
    pub desc_paddr: u64,
    /// Virtual address of RX buffers
    pub buf_vaddr: u64,
    /// Physical address of RX buffers
    pub buf_paddr: u64,
    /// Number of descriptors
    pub size: u32,
}

impl RxRing {
    /// Create a new RX ring
    pub fn new(size: u32) -> Option<Self> {
        let desc_size = (size as usize) * core::mem::size_of::<TxDesc>();
        let buf_total = (size as usize) * RX_BUF_SIZE;

        userlib::println!("[RxRing::new] size={}, desc_size={}, buf_total={}", size, desc_size, buf_total);

        // Allocate DMA memory for descriptors
        let mut desc_paddr: u64 = 0;
        let desc_vaddr = syscall::mmap_dma(desc_size, &mut desc_paddr);
        userlib::println!("[RxRing::new] desc alloc: vaddr=0x{:x} paddr=0x{:x}", desc_vaddr, desc_paddr);
        // Check for error (negative) AND invalid address (0 or too low)
        if desc_vaddr <= 0 || (desc_vaddr as u64) < 0x1000 {
            userlib::println!("[RxRing] desc alloc failed: ret={} (0x{:x}), size={}",
                desc_vaddr, desc_vaddr as u64, desc_size);
            return None;
        }

        // Allocate DMA memory for RX buffers
        userlib::println!("[RxRing::new] about to alloc buf, size={}", buf_total);
        let mut buf_paddr: u64 = 0;
        userlib::println!("[RxRing::new] calling mmap_dma for buf...");
        let buf_vaddr = syscall::mmap_dma(buf_total, &mut buf_paddr);
        userlib::println!("[RxRing::new] buf alloc: vaddr=0x{:x} paddr=0x{:x}", buf_vaddr, buf_paddr);
        // Check for error (negative) AND invalid address (0 or too low)
        if buf_vaddr <= 0 || (buf_vaddr as u64) < 0x1000 {
            userlib::println!("[RxRing] buf alloc failed: ret={} (0x{:x}), size={}",
                buf_vaddr, buf_vaddr as u64, buf_total);
            syscall::munmap(desc_vaddr as u64, desc_size);
            return None;
        }

        // Zero everything
        unsafe {
            core::ptr::write_bytes(desc_vaddr as *mut u8, 0, desc_size);
            core::ptr::write_bytes(buf_vaddr as *mut u8, 0, buf_total);
        }

        // Initialize descriptors to point to RX buffers
        // Per Linux mt76_dma_add_rx_buf(): each descriptor gets buf0/buf1 = physical addr
        // ctrl field gets buffer length (and optionally token)
        let descs = desc_vaddr as *mut TxDesc;
        for i in 0..size {
            let buf_addr = buf_paddr + (i as u64 * RX_BUF_SIZE as u64);
            unsafe {
                let desc = &mut *descs.add(i as usize);
                desc.buf0 = buf_addr as u32;
                desc.buf1 = (buf_addr >> 32) as u32;
                // Set buffer length in ctrl field for RX
                // DMA_DONE (bit 31) = 0 means "owned by hardware" (ready to receive)
                desc.ctrl = (RX_BUF_SIZE as u32) << dma_ctl::SD_LEN0_SHIFT;
                desc.info = 0;
            }
        }

        // Debug: show first descriptor to verify population
        unsafe {
            let first = &*descs;
            userlib::println!("[RxRing] Populated {} descriptors, first: buf0=0x{:08x} buf1=0x{:08x} ctrl=0x{:08x}",
                size, first.buf0, first.buf1, first.ctrl);
        }

        // Flush all RX descriptors to RAM so DMA can see them
        flush_buffer(desc_vaddr as u64, desc_size);

        Some(Self {
            desc_vaddr: desc_vaddr as u64,
            desc_paddr,
            buf_vaddr: buf_vaddr as u64,
            buf_paddr,
            size,
        })
    }

    /// Create a new RRO (Receive Reorder) ring with magic counter initialization
    /// RRO rings require magic counter in info field for hardware-software synchronization
    pub fn new_rro(size: u32) -> Option<Self> {
        let desc_size = (size as usize) * core::mem::size_of::<TxDesc>();
        let buf_total = (size as usize) * RX_BUF_SIZE;

        // Allocate DMA memory for descriptors
        let mut desc_paddr: u64 = 0;
        let desc_vaddr = syscall::mmap_dma(desc_size, &mut desc_paddr);
        if desc_vaddr <= 0 || (desc_vaddr as u64) < 0x1000 {
            userlib::println!("[RxRing/RRO] desc alloc failed: ret={} (0x{:x}), size={}",
                desc_vaddr, desc_vaddr as u64, desc_size);
            return None;
        }

        // Allocate DMA memory for RX buffers
        let mut buf_paddr: u64 = 0;
        let buf_vaddr = syscall::mmap_dma(buf_total, &mut buf_paddr);
        if buf_vaddr <= 0 || (buf_vaddr as u64) < 0x1000 {
            userlib::println!("[RxRing/RRO] buf alloc failed: ret={} (0x{:x}), size={}",
                buf_vaddr, buf_vaddr as u64, buf_total);
            syscall::munmap(desc_vaddr as u64, desc_size);
            return None;
        }

        // Zero everything
        unsafe {
            core::ptr::write_bytes(desc_vaddr as *mut u8, 0, desc_size);
            core::ptr::write_bytes(buf_vaddr as *mut u8, 0, buf_total);
        }

        // Initialize RRO descriptors with magic counter in info field
        // Per Linux mt76: magic counter starts at MT_DMA_MAGIC_CNT-1 = 15
        let magic_info = dma_ctl::RRO_MAGIC_CNT_INIT << dma_ctl::RRO_MAGIC_SHIFT;
        let descs = desc_vaddr as *mut TxDesc;
        for i in 0..size {
            let buf_addr = buf_paddr + (i as u64 * RX_BUF_SIZE as u64);
            unsafe {
                let desc = &mut *descs.add(i as usize);
                desc.buf0 = buf_addr as u32;
                desc.buf1 = (buf_addr >> 32) as u32;
                desc.ctrl = (RX_BUF_SIZE as u32) << dma_ctl::SD_LEN0_SHIFT;
                desc.info = magic_info;  // RRO magic counter for synchronization
            }
        }

        userlib::println!("[RxRing/RRO] Initialized {} descriptors with magic_info=0x{:08x}",
            size, magic_info);

        // Flush all RRO descriptors to RAM so DMA can see them
        flush_buffer(desc_vaddr as u64, desc_size);

        Some(Self {
            desc_vaddr: desc_vaddr as u64,
            desc_paddr,
            buf_vaddr: buf_vaddr as u64,
            buf_paddr,
            size,
        })
    }
}

impl Drop for RxRing {
    fn drop(&mut self) {
        let desc_size = (self.size as usize) * core::mem::size_of::<TxDesc>();
        let buf_total = (self.size as usize) * RX_BUF_SIZE;
        syscall::munmap(self.desc_vaddr, desc_size);
        syscall::munmap(self.buf_vaddr, buf_total);
    }
}

/// WFDMA controller
pub struct Wfdma<'a> {
    dev: &'a Mt7996Device,
    /// Base address of the WFDMA0 window for primary interface (HIF1)
    /// Always 0xD4000 for the primary device
    wfdma0_base: u32,
    /// WM ring - for MCU commands (INIT_DOWNLOAD, SEM_CTRL, etc.) with TXD header
    /// Ring 0 at offset 0x300
    wm_ring: Option<FwdlRing>,
    /// WA ring - for WA MCU commands (used by firmware post-boot)
    /// Ring 1 at offset 0x310
    wa_ring: Option<FwdlRing>,
    /// FWDL ring - for raw firmware data (FW_SCATTER) without TXD header
    /// Ring 2 at offset 0x320
    fwdl_ring: Option<FwdlRing>,
    /// RX MCU ring - for receiving WM responses (RXQ 0)
    rx_mcu_ring: Option<RxRing>,
    /// RX WA ring - for receiving WA event responses (RXQ 1)
    rx_wa_ring: Option<RxRing>,
    /// RX WA MAIN ring - TXFREE notifications via WA (RXQ 2)
    rx_wa_main_ring: Option<RxRing>,
    /// RX WA TRI ring - Band2 WA events (RXQ 3, on HIF1!)
    rx_wa_tri_ring: Option<RxRing>,
    /// Data TX ring (band0) - required for DMA operation per deepwiki
    tx_data_ring: Option<FwdlRing>,
    /// RX MAIN data ring (band0) - required for DMA operation per deepwiki (RXQ 4)
    rx_main_ring: Option<RxRing>,
    /// RX TXFREE ring (band0) - TX completion notifications (RXQ 9)
    /// CRITICAL: Required for RST auto-release! Hardware validates all queues.
    rx_txfree_ring: Option<RxRing>,
    /// RX RRO ring (band0) - Hardware Receive Reorder (RXQ 8)
    rx_rro_band0_ring: Option<RxRing>,
    /// HIF2 RX TXFREE ring (band2) - Required for dual-HIF RST release (RXQ 7)
    hif2_rx_txfree_ring: Option<RxRing>,
    /// HIF2 RX data ring (band2) - Required for dual-HIF RST release (RXQ 5)
    hif2_rx_data_ring: Option<RxRing>,
    /// HIF2 RX RRO ring (band2) - Hardware Receive Reorder (RXQ 6)
    hif2_rx_rro_band2_ring: Option<RxRing>,
    /// HIF2 TX data ring (band1) - Required for dual-HIF RST release
    hif2_tx_band1_ring: Option<FwdlRing>,
    /// HIF2 TX data ring (band2) - Required for dual-HIF RST release
    hif2_tx_band2_ring: Option<FwdlRing>,
    state: WfdmaState,
}

impl<'a> Wfdma<'a> {
    /// Create new WFDMA controller
    ///
    /// This always operates on the primary device (HIF1). HIF2 is accessed
    /// via the device's hif2_* methods when needed.
    pub fn new(dev: &'a Mt7996Device) -> Self {
        // Always use primary WFDMA base for primary device
        let wfdma0_base = wfdma::WFDMA0_BASE;

        userlib::println!("[WFDMA] Using WFDMA0 base 0x{:x} (HIF2 available: {})",
            wfdma0_base, dev.has_hif2());

        Self {
            dev,
            wfdma0_base,
            wm_ring: None,
            wa_ring: None,
            fwdl_ring: None,
            rx_mcu_ring: None,
            rx_wa_ring: None,
            rx_wa_main_ring: None,
            rx_wa_tri_ring: None,
            tx_data_ring: None,
            rx_main_ring: None,
            rx_txfree_ring: None,
            rx_rro_band0_ring: None,
            hif2_rx_txfree_ring: None,
            hif2_rx_data_ring: None,
            hif2_rx_rro_band2_ring: None,
            hif2_tx_band1_ring: None,
            hif2_tx_band2_ring: None,
            state: WfdmaState::Uninit,
        }
    }

    /// Get current state
    pub fn state(&self) -> WfdmaState {
        self.state
    }

    /// Check if ready for transfers
    pub fn is_ready(&self) -> bool {
        self.state == WfdmaState::Ready
    }

    /// Transition to error state
    fn set_error(&mut self) {
        self.state = WfdmaState::Error;
    }

    /// Read WFDMA0 register (relative to WFDMA0_BASE)
    fn read(&self, offset: u32) -> u32 {
        self.dev.read32_raw(self.wfdma0_base + offset)
    }

    /// Write WFDMA0 register (relative to WFDMA0_BASE)
    fn write(&self, offset: u32, value: u32) {
        self.dev.write32_raw(self.wfdma0_base + offset, value);
    }

    /// Read register at absolute BAR offset
    fn read_bar(&self, offset: u32) -> u32 {
        self.dev.read32_raw(offset)
    }

    /// Write register at absolute BAR offset
    fn write_bar(&self, offset: u32, value: u32) {
        self.dev.write32_raw(offset, value);
    }

    /// Read HIF2 WFDMA0 register (relative to WFDMA0_BASE on HIF2)
    ///
    /// **IMPORTANT**: HIF2 registers must be accessed via HIF1's BAR + 0x4000 offset,
    /// NOT via HIF2's separate BAR. The separate BAR appears to be a "shadow"
    /// that doesn't control the actual hardware.
    fn hif2_read(&self, offset: u32) -> Option<u32> {
        // Access HIF2 via HIF1's BAR + offset (how Linux does it)
        Some(self.dev.hif2_via_hif1_read32(wfdma::WFDMA0_BASE + offset))
    }

    /// Write HIF2 WFDMA0 register (relative to WFDMA0_BASE on HIF2)
    ///
    /// **IMPORTANT**: HIF2 registers must be accessed via HIF1's BAR + 0x4000 offset.
    fn hif2_write(&self, offset: u32, value: u32) {
        self.dev.hif2_via_hif1_write32(wfdma::WFDMA0_BASE + offset, value);
    }

    /// Reset HIF2 WFDMA (if available)
    /// This should be called BEFORE resetting the primary WFDMA
    ///
    /// **IMPORTANT**: Uses HIF1+offset access method (not HIF2's separate BAR)
    fn reset_hif2(&self) {
        if !self.dev.has_hif2() {
            userlib::println!("[DMA] No HIF2 available, skipping HIF2 reset");
            return;
        }

        userlib::println!("[DMA] Resetting HIF2 WFDMA (via HIF1+0x4000)...");

        // Read current RST value via HIF1+offset
        let rst_offset = wfdma::WFDMA0_BASE + wfdma::RST;
        let rst_before = self.dev.hif2_via_hif1_read32(rst_offset);
        userlib::println!("[DMA]   HIF2 RST before: 0x{:08x}", rst_before);

        // Clear RST first using RMW (same pattern as HIF1, matches Linux mt76_clear)
        let rst_mask = wfdma::RST_DMASHDL_ALL_RST | wfdma::RST_LOGIC_RST;
        let rst_val = rst_before;
        self.dev.hif2_via_hif1_write32(rst_offset, rst_val & !rst_mask);
        unsafe { core::arch::asm!("dsb sy"); }
        let rst_after_clear = self.dev.hif2_via_hif1_read32(rst_offset);
        userlib::println!("[DMA]   HIF2 RST after clear: 0x{:08x}", rst_after_clear);

        // Short delay between clear and set
        userlib::delay_us(100);

        // Now assert reset using RMW (matches Linux mt76_set)
        let rst_val = self.dev.hif2_via_hif1_read32(rst_offset);
        self.dev.hif2_via_hif1_write32(rst_offset, rst_val | rst_mask);
        unsafe { core::arch::asm!("dsb sy"); }
        let rst_after_set = self.dev.hif2_via_hif1_read32(rst_offset);
        userlib::println!("[DMA]   HIF2 RST after set(0x30): 0x{:08x}", rst_after_set);

        // Actual time delay for HIF2 reset to complete
        userlib::delay_ms(10);

        // Read back RST
        let rst_after = self.dev.hif2_via_hif1_read32(rst_offset);
        userlib::println!("[DMA]   HIF2 RST after 10ms: 0x{:08x}", rst_after);

        // Clear GLO_CFG DMA enables on HIF2 - match Linux mt7996_dma_disable() exactly
        let glo_offset = wfdma::WFDMA0_BASE + wfdma::GLO_CFG;
        let glo = self.dev.hif2_via_hif1_read32(glo_offset);
        let clear_mask = glo_cfg::TX_DMA_EN | glo_cfg::RX_DMA_EN |
                         glo_cfg::OMIT_TX_INFO | glo_cfg::OMIT_RX_INFO | glo_cfg::OMIT_RX_INFO_PFET2;
        let glo_clean = glo & !clear_mask;  // Just clear, don't set anything back
        self.dev.hif2_via_hif1_write32(glo_offset, glo_clean);
        userlib::println!("[DMA]   HIF2 GLO_CFG: 0x{:08x} -> 0x{:08x}", glo, glo_clean);

        userlib::println!("[DMA] HIF2 WFDMA reset complete");
    }

    /// Read register via L1 remapping (for addresses > 0x100000 but < 0x70000000)
    fn read_remap_l1(&self, addr: u32) -> u32 {
        // Calculate offset within remap window and base address
        let offset = addr & remap::L1_REMAP_MASK;
        let base = addr & remap::L1_BASE_MASK;

        // Write base to HIF_REMAP_L1 register
        self.write_bar(remap::HIF_REMAP_L1, base >> 18);

        // Read back to ensure write completes
        let _ = self.read_bar(remap::HIF_REMAP_L1);

        // Read via remap window
        self.read_bar(remap::HIF_REMAP_BASE_L1 + offset)
    }

    /// Write register via L1 remapping (for addresses > 0x100000 but < 0x70000000)
    fn write_remap_l1(&self, addr: u32, value: u32) {
        let offset = addr & remap::L1_REMAP_MASK;
        let base = addr & remap::L1_BASE_MASK;

        // Write base to HIF_REMAP_L1 register
        self.write_bar(remap::HIF_REMAP_L1, base >> 18);

        // Read back to ensure write completes
        let _ = self.read_bar(remap::HIF_REMAP_L1);

        // Write via remap window
        self.write_bar(remap::HIF_REMAP_BASE_L1 + offset, value);
    }

    /// Read register via CBTOP remapping (for addresses in 0x70000000 range - MT7990)
    fn read_remap_cbtop(&self, addr: u32) -> u32 {
        // CBTOP uses 64KB windows with 16-bit offset/base
        let offset = addr & remap::CBTOP_REMAP_MASK;
        let base = (addr >> 16) & 0xffff;

        userlib::println!("[DMA] CBTOP remap read: addr=0x{:08x} -> base=0x{:04x} offset=0x{:04x}",
            addr, base, offset);
        userlib::println!("[DMA]   HIF_REMAP_CBTOP=0x{:x}, window at 0x{:x}",
            remap::HIF_REMAP_CBTOP, remap::HIF_REMAP_BASE_CBTOP);

        // Write base to HIF_REMAP_CBTOP register
        self.write_bar(remap::HIF_REMAP_CBTOP, base);

        // Read back to ensure write completes
        let readback = self.read_bar(remap::HIF_REMAP_CBTOP);
        userlib::println!("[DMA]   Wrote base=0x{:04x}, readback=0x{:08x}", base, readback);

        // Read via remap window
        let result = self.read_bar(remap::HIF_REMAP_BASE_CBTOP + offset);
        userlib::println!("[DMA]   Read from 0x{:x} = 0x{:08x}",
            remap::HIF_REMAP_BASE_CBTOP + offset, result);
        result
    }

    /// Write register via CBTOP remapping (for addresses in 0x70000000 range - MT7990)
    fn write_remap_cbtop(&self, addr: u32, value: u32) {
        // CBTOP uses 64KB windows with 16-bit offset/base
        let offset = addr & remap::CBTOP_REMAP_MASK;
        let base = (addr >> 16) & 0xffff;

        // Write base to HIF_REMAP_CBTOP register
        self.write_bar(remap::HIF_REMAP_CBTOP, base);

        // Read back to ensure write completes
        let _ = self.read_bar(remap::HIF_REMAP_CBTOP);

        // Write via remap window
        self.write_bar(remap::HIF_REMAP_BASE_CBTOP + offset, value);
    }

    /// Configure WFDMA prefetch settings
    ///
    /// This configures the EXT_CTRL registers for each queue to set up SRAM prefetch
    /// boundaries. This is REQUIRED before ring registers become writable!
    ///
    /// Based on Linux mt7996_dma_prefetch(). The prefetch value format is:
    /// (base << 16) | depth, where base accumulates by (depth << 4) for each queue.
    fn configure_prefetch(&self) {
        // Prefetch depths used by Linux for MT7996:
        // - MCU command queues (FWDL/WM/WA): depth = 2
        // - TX data queues: depth = 8
        // - RX data queues: depth = 16
        // For firmware download, we only need MCU queues.
        //
        // This function mirrors Linux __mt7996_dma_prefetch() exactly.
        // Uses queue_map to compute register offsets the same way Linux macros do.

        use queue_map::{mcuq, rxq, txq, MT7996_QUEUE_IDS};
        let q = &MT7996_QUEUE_IDS;

        let mut base: u32 = 0;

        // Helper to compute prefetch value and advance base
        // Linux: __mt7996_dma_prefetch_base(&base, depth)
        // Formula: (base << 16) | depth, then base += depth << 4
        let prefetch = |b: &mut u32, depth: u32| -> u32 {
            let val = (*b << 16) | depth;
            *b += depth << 4;  // Advance base by depth * 16
            val
        };

        // MT7996 command queue depth (vs 4 for MT7990)
        let cmd_depth = 2u32;

        //---------------------------------------------------------------------
        // MCU TX Command Queues
        // Linux: mt76_wr(dev, MT_MCUQ_EXT_CTRL(MT_MCUQ_FWDL), PREFETCH(val))
        // Order: FWDL, WM, WA (FWDL gets base=0)
        //---------------------------------------------------------------------
        let fwdl_prefetch = prefetch(&mut base, cmd_depth);
        self.write(q.mcuq_ext_ctrl(mcuq::FWDL), fwdl_prefetch);

        let wm_prefetch = prefetch(&mut base, cmd_depth);
        self.write(q.mcuq_ext_ctrl(mcuq::WM), wm_prefetch);

        let wa_prefetch = prefetch(&mut base, cmd_depth);
        self.write(q.mcuq_ext_ctrl(mcuq::WA), wa_prefetch);

        userlib::println!("[DMA]   MCUQ prefetch @0x{:x},0x{:x},0x{:x}: FWDL=0x{:08x} WM=0x{:08x} WA=0x{:08x}",
            q.mcuq_ext_ctrl(mcuq::FWDL), q.mcuq_ext_ctrl(mcuq::WM), q.mcuq_ext_ctrl(mcuq::WA),
            fwdl_prefetch, wm_prefetch, wa_prefetch);

        //---------------------------------------------------------------------
        // Data TX Queues
        // Linux: mt76_wr(dev, MT_TXQ_EXT_CTRL(0), PREFETCH(0x8))
        //---------------------------------------------------------------------
        let tx_data_depth = 8u32;
        let tx_band0_prefetch = prefetch(&mut base, tx_data_depth);
        self.write(q.txq_ext_ctrl(txq::BAND0), tx_band0_prefetch);

        userlib::println!("[DMA]   TXQ data prefetch @0x{:x}: BAND0=0x{:08x}",
            q.txq_ext_ctrl(txq::BAND0), tx_band0_prefetch);

        //---------------------------------------------------------------------
        // RX Event Queues (MCU)
        // Linux: mt76_wr(dev, MT_RXQ_EXT_CTRL(MT_RXQ_MCU), PREFETCH(val))
        //---------------------------------------------------------------------
        let rx_mcu_prefetch = prefetch(&mut base, cmd_depth);
        self.write(q.rxq_ext_ctrl(rxq::MCU), rx_mcu_prefetch);

        let rx_wa_prefetch = prefetch(&mut base, cmd_depth);
        self.write(q.rxq_ext_ctrl(rxq::MCU_WA), rx_wa_prefetch);

        let rx_wa_main_prefetch = prefetch(&mut base, cmd_depth);
        self.write(q.rxq_ext_ctrl(rxq::MAIN_WA), rx_wa_main_prefetch);

        let rx_wa_tri_prefetch = prefetch(&mut base, cmd_depth);
        self.write(q.rxq_ext_ctrl(rxq::BAND2_WA), rx_wa_tri_prefetch);

        userlib::println!("[DMA]   RXQ MCU prefetch @0x{:x},0x{:x},0x{:x},0x{:x}: MCU=0x{:08x} WA=0x{:08x} WA_MAIN=0x{:08x} WA_TRI=0x{:08x}",
            q.rxq_ext_ctrl(rxq::MCU), q.rxq_ext_ctrl(rxq::MCU_WA),
            q.rxq_ext_ctrl(rxq::MAIN_WA), q.rxq_ext_ctrl(rxq::BAND2_WA),
            rx_mcu_prefetch, rx_wa_prefetch, rx_wa_main_prefetch, rx_wa_tri_prefetch);

        //---------------------------------------------------------------------
        // RX Data Queues
        // Linux: mt76_wr(dev, MT_RXQ_EXT_CTRL(MT_RXQ_MAIN), PREFETCH(0x10))
        //---------------------------------------------------------------------
        let rx_data_depth = 16u32;
        let rx_main_prefetch = prefetch(&mut base, rx_data_depth);
        self.write(q.rxq_ext_ctrl(rxq::MAIN), rx_main_prefetch);

        userlib::println!("[DMA]   RXQ data prefetch @0x{:x}: MAIN=0x{:08x}",
            q.rxq_ext_ctrl(rxq::MAIN), rx_main_prefetch);

        //---------------------------------------------------------------------
        // RX TXFREE Queue (TX completion notifications - CRITICAL!)
        // Linux: mt76_wr(dev, MT_RXQ_EXT_CTRL(MT_RXQ_TXFREE_BAND0), PREFETCH(0x4))
        //---------------------------------------------------------------------
        let rx_txfree_depth = 4u32;
        let rx_txfree_prefetch = prefetch(&mut base, rx_txfree_depth);
        self.write(q.rxq_ext_ctrl(rxq::TXFREE_BAND0), rx_txfree_prefetch);

        userlib::println!("[DMA]   RXQ TXFREE prefetch @0x{:x}: 0x{:08x}",
            q.rxq_ext_ctrl(rxq::TXFREE_BAND0), rx_txfree_prefetch);

        //---------------------------------------------------------------------
        // RX RRO Queue (Hardware Receive Reorder)
        // Linux: mt76_wr(dev, MT_RXQ_EXT_CTRL(MT_RXQ_RRO_BAND0), PREFETCH(0x10))
        //---------------------------------------------------------------------
        let rx_rro_prefetch = prefetch(&mut base, rx_data_depth);
        self.write(q.rxq_ext_ctrl(rxq::RRO_BAND0), rx_rro_prefetch);

        userlib::println!("[DMA]   RXQ RRO prefetch @0x{:x}: 0x{:08x}",
            q.rxq_ext_ctrl(rxq::RRO_BAND0), rx_rro_prefetch);

        //---------------------------------------------------------------------
        // HIF2 prefetch configuration (for dual-PCIe devices)
        // Linux calls __mt7996_dma_prefetch() with hif2 offset, continuing base
        //---------------------------------------------------------------------
        if self.dev.has_hif2() {
            userlib::println!("[DMA]   HIF2 prefetch configuration (continuing from base=0x{:x}):", base);

            // HIF2 RX data (band2)
            let hif2_rx_data_prefetch = prefetch(&mut base, rx_data_depth);
            let hif2_rx_data_reg = wfdma::WFDMA0_BASE + q.rxq_ext_ctrl(rxq::BAND2);
            self.dev.hif2_via_hif1_write32(hif2_rx_data_reg, hif2_rx_data_prefetch);
            userlib::println!("[DMA]     HIF2 RX data @0x{:x}: 0x{:08x}", hif2_rx_data_reg, hif2_rx_data_prefetch);

            // HIF2 RX TXFREE (band2)
            let hif2_txfree_prefetch = prefetch(&mut base, rx_txfree_depth);
            let hif2_txfree_reg = wfdma::WFDMA0_BASE + q.rxq_ext_ctrl(rxq::TXFREE_BAND2);
            self.dev.hif2_via_hif1_write32(hif2_txfree_reg, hif2_txfree_prefetch);
            userlib::println!("[DMA]     HIF2 TXFREE @0x{:x}: 0x{:08x}", hif2_txfree_reg, hif2_txfree_prefetch);

            // HIF2 RX RRO (band2)
            let hif2_rro_prefetch = prefetch(&mut base, rx_data_depth);
            let hif2_rro_reg = wfdma::WFDMA0_BASE + q.rxq_ext_ctrl(rxq::RRO_BAND2);
            self.dev.hif2_via_hif1_write32(hif2_rro_reg, hif2_rro_prefetch);
            userlib::println!("[DMA]     HIF2 RRO @0x{:x}: 0x{:08x}", hif2_rro_reg, hif2_rro_prefetch);

            // HIF2 TX data (band1) - uses txq_ext_ctrl
            let hif2_tx_band1_prefetch = prefetch(&mut base, tx_data_depth);
            let hif2_tx_band1_reg = wfdma::WFDMA0_BASE + q.txq_ext_ctrl(txq::BAND1);
            self.dev.hif2_via_hif1_write32(hif2_tx_band1_reg, hif2_tx_band1_prefetch);
            userlib::println!("[DMA]     HIF2 TX band1 @0x{:x}: 0x{:08x}", hif2_tx_band1_reg, hif2_tx_band1_prefetch);

            // HIF2 TX data (band2)
            let hif2_tx_band2_prefetch = prefetch(&mut base, tx_data_depth);
            let hif2_tx_band2_reg = wfdma::WFDMA0_BASE + q.txq_ext_ctrl(txq::BAND2);
            self.dev.hif2_via_hif1_write32(hif2_tx_band2_reg, hif2_tx_band2_prefetch);
            userlib::println!("[DMA]     HIF2 TX band2 @0x{:x}: 0x{:08x}", hif2_tx_band2_reg, hif2_tx_band2_prefetch);

            // Set CALC_MODE on HIF2 as well
            let hif2_ext1_reg = wfdma::WFDMA0_BASE + wfdma::GLO_CFG_EXT1;
            let hif2_ext1 = self.dev.hif2_via_hif1_read32(hif2_ext1_reg);
            self.dev.hif2_via_hif1_write32(hif2_ext1_reg, hif2_ext1 | wfdma::GLO_CFG_EXT1_CALC_MODE);
            userlib::println!("[DMA]     HIF2 GLO_CFG_EXT1 CALC_MODE set");
        }

        // Set CALC_MODE bit in GLO_CFG_EXT1 to finalize prefetch configuration
        let ext1 = self.read(wfdma::GLO_CFG_EXT1);
        self.write(wfdma::GLO_CFG_EXT1, ext1 | wfdma::GLO_CFG_EXT1_CALC_MODE);
        let ext1_after = self.read(wfdma::GLO_CFG_EXT1);
        userlib::println!("[DMA]   GLO_CFG_EXT1: 0x{:08x} -> 0x{:08x} (CALC_MODE={})",
            ext1, ext1_after, if (ext1_after & wfdma::GLO_CFG_EXT1_CALC_MODE) != 0 { "set" } else { "NOT set" });
    }

    /// Perform wireless subsystem reset
    pub fn wfsys_reset(&self) {
        userlib::println!("[DMA] Performing wfsys_reset (WF_SUBSYS_RST=0x{:08x})...", remap::WF_SUBSYS_RST);

        // WF_SUBSYS_RST is in CBTOP1 range (0x70000000+), use CBTOP remap
        userlib::println!("[DMA] Reading WF_SUBSYS_RST via CBTOP remap...");
        let val = self.read_remap_cbtop(remap::WF_SUBSYS_RST);
        userlib::println!("[DMA]   WF_SUBSYS_RST initial: 0x{:08x}", val);

        // Set reset bit
        userlib::println!("[DMA] Asserting reset (writing 0x{:08x})...", val | 0x1);
        self.write_remap_cbtop(remap::WF_SUBSYS_RST, val | 0x1);

        // Readback to verify write
        let val_during = self.read_remap_cbtop(remap::WF_SUBSYS_RST);
        userlib::println!("[DMA]   WF_SUBSYS_RST during reset: 0x{:08x}", val_during);

        // Wait 50ms (reset must be held long enough)
        userlib::delay_ms(50);

        // Clear reset bit
        userlib::println!("[DMA] Releasing reset (writing 0x{:08x})...", val & !0x1);
        self.write_remap_cbtop(remap::WF_SUBSYS_RST, val & !0x1);

        // Wait 100ms for ROM to boot and reach download state
        // The MT7996 ROM needs time to initialize after reset
        userlib::delay_ms(100);

        let val_after = self.read_remap_cbtop(remap::WF_SUBSYS_RST);
        userlib::println!("[DMA]   WF_SUBSYS_RST after: 0x{:08x}", val_after);
    }

    /// Initialize WFDMA for firmware download
    pub fn init(&mut self) -> bool {
        trace!(DMA, "Initializing WFDMA...");

        // Validate queue ID mappings (catches math errors at init time)
        if let Err(e) = queue_map::validate_mt7996_queue_ids() {
            userlib::println!("[DMA] FATAL: Queue ID validation failed: {}", e);
            return false;
        }
        userlib::println!("[DMA] Queue ID validation passed");

        // Validate regs.rs constants match Linux regs.h
        regs::validate_registers();
        userlib::println!("[DMA] Register address validation passed");

        // Validate dma.rs wfdma constants match regs.rs (catches drift)
        wfdma::validate_against_regs();
        userlib::println!("[DMA] WFDMA constants cross-validation passed");

        // Validate interrupt bits match regs.rs
        int_mask::validate_against_regs();
        userlib::println!("[DMA] Interrupt mask cross-validation passed");

        // === TIMING DIAGNOSTIC ===
        // Verify ARM generic timer is working correctly (if delays don't work, DMA fails)
        let freq: u64;
        let start: u64;
        unsafe {
            core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
            core::arch::asm!("mrs {}, cntpct_el0", out(reg) start);
        }
        userlib::println!("[TIMING] cntfrq_el0 = {} Hz ({} MHz)", freq, freq / 1_000_000);
        userlib::delay_ms(100);
        let end: u64;
        unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) end); }
        let elapsed_ticks = end - start;
        let expected_ticks = freq / 10; // 100ms = freq/10 ticks
        userlib::println!("[TIMING] 100ms delay: {} ticks (expected ~{})", elapsed_ticks, expected_ticks);
        if elapsed_ticks < expected_ticks / 2 {
            userlib::println!("[TIMING] WARNING: Delay too short! Timer may not be working!");
        } else if elapsed_ticks > expected_ticks * 2 {
            userlib::println!("[TIMING] WARNING: Delay too long! Scheduler overhead?");
        } else {
            userlib::println!("[TIMING] Timer appears to be working correctly");
        }
        // === END TIMING DIAGNOSTIC ===

        // Dump queue mappings for debugging
        queue_map::dump_queue_ids();

        // Print BAR0 info for diagnostics
        let bar0_size = self.dev.bar0_size();
        let bar0_virt = self.dev.bar0_virt();
        userlib::println!("[DMA] HIF1 BAR0: virt=0x{:x} size=0x{:x} ({}KB)",
            bar0_virt, bar0_size, bar0_size / 1024);

        // Print HIF2 info if available
        if let Some(hif2_virt) = self.dev.hif2_bar0_virt() {
            let hif2_size = self.dev.hif2_bar0_size();
            userlib::println!("[DMA] HIF2 BAR0: virt=0x{:x} size=0x{:x} ({}KB)",
                hif2_virt, hif2_size, hif2_size / 1024);
        }

        // NOTE: HIF2 reset moved to AFTER HIF1 reset to match Linux mt7996_dma_disable() order:
        // Linux does: HIF1 RST → HIF1 GLO_CFG → HIF2 RST → HIF2 GLO_CFG

        // Print PCI command register (bus master check)
        let cmd = self.dev.command();
        let mem_space = (cmd & 0x02) != 0;
        let bus_master = (cmd & 0x04) != 0;
        userlib::println!("[DMA] HIF1 PCI CMD: 0x{:04x} (MemSpace={} BusMaster={})",
            cmd, mem_space, bus_master);
        if !bus_master {
            userlib::println!("[DMA] ERROR: HIF1 Bus Master not enabled! DMA will not work.");
        }

        // Print HIF2 PCI command register (bus master check)
        if self.dev.has_hif2() {
            let hif2_cmd = self.dev.hif2_command();
            let hif2_mem_space = (hif2_cmd & 0x02) != 0;
            let hif2_bus_master = (hif2_cmd & 0x04) != 0;
            userlib::println!("[DMA] HIF2 PCI CMD: 0x{:04x} (MemSpace={} BusMaster={})",
                hif2_cmd, hif2_mem_space, hif2_bus_master);
            if !hif2_bus_master {
                userlib::println!("[DMA] ERROR: HIF2 Bus Master not enabled! HIF2 DMA will not work.");
            }

            // HIF2 access method: always via HIF1 BAR + 0x4000 offset (per Linux mt76)
            // Direct HIF2 BAR access is NOT used - it may return different values and
            // doesn't control the actual hardware the same way.
            userlib::println!("[DMA] HIF2 accessed via HIF1+0x4000 (Linux mt76 method)");
        }

        // Test which register ranges are writable
        userlib::println!("[DMA] Testing register writability (WFDMA0_BASE=0x{:x}):", self.wfdma0_base);

        // Scan test: try writing to various offsets in 0x100-0x400 range
        // to find where writes start failing
        // CRITICAL: Do NOT test RST (0x100) or GLO_CFG (0x208) - writing to these
        // corrupts DMA state and can prevent RST auto-release!
        let test_offsets = [
            // (0x100, "RST"),  // NEVER touch - controls DMA reset state
            // (0x13c, "BUSY_ENA"),  // Skip - affects DMA operation
            (0x1f0, "MCU_CMD"),
            // (0x200, "INT_SRC"),  // Read-only status
            (0x204, "INT_MASK"),
            // (0x208, "GLO_CFG"),  // NEVER touch - controls DMA enable
            // (0x20c, "RST_DTX_PTR"),  // Write-only reset
            (0x2b0, "GLO_CFG_EXT0"),
            (0x2f0, "PRI_DLY_INT0"),
            (0x300, "Ring0_BASE"),
            (0x304, "Ring0_CNT"),
            (0x320, "Ring2_BASE"),
            (0x324, "Ring2_CNT"),
            (0x500, "RX0_BASE"),
            (0x600, "MCUQ_EXT_CTRL"),
        ];

        for (offset, name) in test_offsets.iter() {
            let before = self.read(*offset);
            // Write a distinct test pattern
            let test_val = 0xABCD_0000 | (*offset as u32);
            self.write(*offset, test_val);
            let after = self.read(*offset);
            // Restore original
            self.write(*offset, before);
            let status = if after == test_val { "OK" } else if after == before { "UNCHANGED" } else { "PARTIAL" };
            userlib::println!("[DMA]   0x{:03x} {}: before=0x{:08x} after=0x{:08x} {}",
                offset, name, before, after, status);
        }

        // Step 0: Check and clear WDT (watchdog) bits
        // If WDT is set, the device experienced a firmware crash
        let mcu_cmd = self.read(wfdma::MCU_CMD);
        trace!(DMA, "MCU_CMD initial: 0x{:08x}", mcu_cmd);

        if (mcu_cmd & mcu_cmd::WDT_MASK) != 0 {
            trace!(DMA, "WDT detected, clearing...");
            // Clear WDT bits by writing them
            self.write(wfdma::MCU_CMD, mcu_cmd & !mcu_cmd::WDT_MASK);
            let mcu_cmd_after = self.read(wfdma::MCU_CMD);
            trace!(DMA, "MCU_CMD after WDT clear: 0x{:08x}", mcu_cmd_after);
        }

        // Step 1: WFDMA reset - pulse RST then initialize control registers
        // IMPORTANT: RST pulse clears ring bases/prefetch but NOT control registers!
        // We must explicitly initialize GLO_CFG, INT_MASK, PRI_DLY_INT after RST.
        userlib::println!("[DMA] WFDMA reset...");

        let rst_before = self.read(wfdma::RST);
        userlib::println!("[DMA] RST before pulse: 0x{:08x}", rst_before);

        // Pulse RST: clear then set (Linux mt7996_dma_disable pattern)
        // CRITICAL: Use RMW like Linux mt76_clear/mt76_set to preserve any other bits!
        let rst_mask = wfdma::RST_DMASHDL_ALL_RST | wfdma::RST_LOGIC_RST;
        let rst_val = self.read(wfdma::RST);
        self.write(wfdma::RST, rst_val & !rst_mask);  // mt76_clear
        unsafe { core::arch::asm!("dsb sy"); }
        userlib::delay_us(100);
        let rst_val = self.read(wfdma::RST);
        self.write(wfdma::RST, rst_val | rst_mask);   // mt76_set
        unsafe { core::arch::asm!("dsb sy"); }
        userlib::delay_ms(10);

        let rst_after = self.read(wfdma::RST);
        userlib::println!("[DMA] RST after pulse: 0x{:08x}", rst_after);

        // NOTE: Linux does NOT touch INT_MASK or PRI_DLY_INT in dma_disable!
        // - PRI_DLY_INT is cleared in dma_enable (after driver_own)
        // - INT_MASK is set in dma_start (after GLO_CFG enable)
        // We should NOT touch them here.

        // NOTE: Do NOT manually clear RST here!
        // RST=0x30 stays asserted during ring programming and auto-releases when GLO_CFG enables DMA.

        // Step 2: Clean GLO_CFG - clear EXACTLY the bits Linux clears in mt7996_dma_disable()!
        // Linux mt76_clear: TX_DMA_EN, RX_DMA_EN, OMIT_TX_INFO, OMIT_RX_INFO, OMIT_RX_INFO_PFET2
        // Do NOT set any bits here - they get set during enable phase.
        // The transition from "all cleared" to "all set" may be important for RST auto-release.
        let glo = self.read(wfdma::GLO_CFG);
        userlib::println!("[DMA] GLO_CFG initial: 0x{:08x}", glo);
        let clear_mask = glo_cfg::TX_DMA_EN | glo_cfg::RX_DMA_EN |
                         glo_cfg::OMIT_TX_INFO | glo_cfg::OMIT_RX_INFO | glo_cfg::OMIT_RX_INFO_PFET2;
        let glo_clean = glo & !clear_mask;  // Just clear, don't set anything back
        self.write(wfdma::GLO_CFG, glo_clean);
        userlib::println!("[DMA] GLO_CFG after clean: 0x{:08x}", self.read(wfdma::GLO_CFG));

        // Step 2b: Reset HIF2 AFTER HIF1 - this is Linux's mt7996_dma_disable() order
        // Linux: HIF1 RST → HIF1 GLO_CFG → HIF2 RST → HIF2 GLO_CFG
        self.reset_hif2();

        // NOTE: BUSY_ENA moved to enable phase (after RST_DTX_PTR, per Linux order)
        // NOTE: Prefetch moved to enable phase (after PRI_DLY_INT clear, per Linux order)

        // Step 3: RST stays 0x30 during ring programming (auto-releases when GLO_CFG enables DMA)
        let rst_val = self.read(wfdma::RST);
        userlib::println!("[DMA] RST before ring programming: 0x{:08x} (should be 0x30)", rst_val);

        // State: Reset complete, ready for ring programming
        self.state = WfdmaState::Reset;

        // Allocate WM ring (Ring 0) - for MCU commands with TXD header
        // Linux: mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_WM, ...)
        userlib::println!("[DMA] Allocating WM ring (Ring 0 for MCU commands)...");
        self.wm_ring = FwdlRing::new();
        if self.wm_ring.is_none() {
            userlib::println!("[DMA] Failed to allocate WM ring!");
            self.set_error();
            return false;
        }

        let wm_ring = self.wm_ring.as_ref().unwrap();
        userlib::println!("[DMA] WM ring allocated: vaddr=0x{:x}, paddr=0x{:x}, cnt={}",
            wm_ring.desc_vaddr, wm_ring.desc_paddr, wm_ring.size);

        // Configure WM ring (hardware ring 17)
        let wm_ring_base = McuQueue::Wm.ring_offset();
        userlib::println!("[DMA] WM Ring@WFDMA+0x{:x} programming...", wm_ring_base);
        self.write(wm_ring_base + wfdma::RING_BASE, wm_ring.desc_paddr as u32);
        self.write(wm_ring_base + wfdma::RING_MAX_CNT, wm_ring.size);
        self.write(wm_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(wm_ring_base + wfdma::RING_DMA_IDX, 0);
        let wm_base_rb = self.read(wm_ring_base + wfdma::RING_BASE);
        let wm_cnt_rb = self.read(wm_ring_base + wfdma::RING_MAX_CNT);
        userlib::println!("[DMA] WM Ring after write: base=0x{:08x} cnt={}", wm_base_rb, wm_cnt_rb);

        // Allocate WA ring (Ring 1) - for WA MCU commands (required for RST auto-release)
        // Linux: mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_WA, ...)
        // Per deepwiki: DMA engine expects ALL configured queues to have valid descriptors
        userlib::println!("[DMA] Allocating WA ring (Ring 1 for WA MCU commands)...");
        self.wa_ring = FwdlRing::new();
        if self.wa_ring.is_none() {
            userlib::println!("[DMA] Failed to allocate WA ring!");
            self.set_error();
            return false;
        }

        let wa_ring = self.wa_ring.as_ref().unwrap();
        userlib::println!("[DMA] WA ring allocated: vaddr=0x{:x}, paddr=0x{:x}, cnt={}",
            wa_ring.desc_vaddr, wa_ring.desc_paddr, wa_ring.size);

        // Configure WA ring (hardware ring 20)
        let wa_ring_base = McuQueue::Wa.ring_offset();
        userlib::println!("[DMA] WA Ring@WFDMA+0x{:x} programming...", wa_ring_base);
        self.write(wa_ring_base + wfdma::RING_BASE, wa_ring.desc_paddr as u32);
        self.write(wa_ring_base + wfdma::RING_MAX_CNT, wa_ring.size);
        self.write(wa_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(wa_ring_base + wfdma::RING_DMA_IDX, 0);
        let wa_base_rb = self.read(wa_ring_base + wfdma::RING_BASE);
        let wa_cnt_rb = self.read(wa_ring_base + wfdma::RING_MAX_CNT);
        userlib::println!("[DMA] WA Ring after write: base=0x{:08x} cnt={}", wa_base_rb, wa_cnt_rb);

        // Allocate firmware download ring (Ring 2) - for raw FW data WITHOUT TXD
        // Linux: mt76_init_mcu_queue(&dev->mt76, MT_MCUQ_FWDL, ...)
        userlib::println!("[DMA] Allocating FWDL ring (Ring 2 for raw FW data)...");
        self.fwdl_ring = FwdlRing::new();
        if self.fwdl_ring.is_none() {
            userlib::println!("[DMA] Failed to allocate FWDL ring!");
            self.set_error();
            return false;
        }

        let ring = self.fwdl_ring.as_ref().unwrap();
        let desc_bytes = (ring.size as usize) * core::mem::size_of::<TxDesc>();
        userlib::println!("[DMA] FWDL ring allocated: vaddr=0x{:x}, paddr=0x{:x}, cnt={}, bytes={}",
            ring.desc_vaddr, ring.desc_paddr, ring.size, desc_bytes);

        // CRITICAL: Verify DMA addresses are below 4GB (MT7996 has 32-bit DMA on some paths)
        if (ring.desc_paddr >> 32) != 0 {
            trace!(DMA, "ERROR: FWDL ring paddr 0x{:x} is above 4GB!", ring.desc_paddr);
            self.set_error();
            return false;
        }

        // Configure FWDL ring (hardware ring 16)
        let ring_base = McuQueue::Fwdl.ring_offset();

        // Debug: dump ring register space before config
        userlib::println!("[DMA] FWDL Ring@WFDMA+0x{:x} before write:", ring_base);
        userlib::println!("[DMA]   regs: {:08x} {:08x} {:08x} {:08x}",
            self.read(ring_base + 0x00), self.read(ring_base + 0x04),
            self.read(ring_base + 0x08), self.read(ring_base + 0x0c));

        // Program ring registers (works because RST=0x30 is still asserted)
        self.write(ring_base + wfdma::RING_BASE, ring.desc_paddr as u32);
        self.write(ring_base + wfdma::RING_MAX_CNT, ring.size);
        self.write(ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(ring_base + wfdma::RING_DMA_IDX, 0);

        // Verify ring configuration
        let rb_base = self.read(ring_base + wfdma::RING_BASE);
        let rb_cnt = self.read(ring_base + wfdma::RING_MAX_CNT);
        userlib::println!("[DMA] FWDL Ring after write: base=0x{:08x} cnt={} {}",
            rb_base, rb_cnt,
            if rb_base == ring.desc_paddr as u32 { "OK" } else { "FAILED!" });

        if rb_base != ring.desc_paddr as u32 {
            userlib::println!("[DMA] ERROR: Ring base write failed! DMA won't work.");
            self.set_error();
            return false;
        }

        // NOTE: Linux does NOT create rings at TX indices 0-15 (only 16-21 are used).
        // Ring indices 16=FWDL, 17=WM, 18=BAND0, 19=BAND1, 20=WA, 21=BAND2

        // NOTE: PRI_DLY_INT and GLO_CFG_EXT0/EXT1 moved to enable phase
        // Per Linux mt7996_dma_enable order:
        // 1. RST_DTX_PTR
        // 2. PRI_DLY_INT clear
        // 3. Prefetch (configure_prefetch)
        // 4. BUSY_ENA
        // 5. HIF_MISC_BUSY poll
        // 6. GLO_CFG_EXT0/EXT1
        // 7. RX pause thresholds
        // 8. GLO_CFG enable
        // 9. INT_MASK enable

        // Step 4: Set up RX MCU queue for receiving MCU responses
        // This is required for the MCU to send responses to our commands
        userlib::println!("[DMA] Setting up RX MCU queue...");
        self.rx_mcu_ring = RxRing::new(wfdma::RX_MCU_RING_SIZE);
        if self.rx_mcu_ring.is_none() {
            userlib::println!("[DMA] ERROR: Failed to allocate RX MCU ring");
            self.set_error();
            return false;
        }

        let rx_ring = self.rx_mcu_ring.as_ref().unwrap();
        userlib::println!("[DMA] RX MCU ring: vaddr=0x{:x}, paddr=0x{:x}, size={}",
            rx_ring.desc_vaddr, rx_ring.desc_paddr, rx_ring.size);

        // CRITICAL: Verify DMA addresses are below 4GB
        if (rx_ring.desc_paddr >> 32) != 0 {
            trace!(DMA, "ERROR: RX ring paddr 0x{:x} is above 4GB!", rx_ring.desc_paddr);
            self.set_error();
            return false;
        }

        // State: Rings configured
        self.state = WfdmaState::RingsConfigured;

        // Configure RX MCU ring registers
        // RX rings use hardware indices directly (MCU=0 at offset 0x500)
        let rx_ring_base = wfdma::RX_RING_BASE + wfdma::RXQ_MCU * 0x10;
        trace!(DMA, "RX ring@WFDMA+0x{:x} before write:", rx_ring_base);
        trace!(DMA, "  regs: {:08x} {:08x} {:08x} {:08x}",
            self.read(rx_ring_base + 0x00), self.read(rx_ring_base + 0x04),
            self.read(rx_ring_base + 0x08), self.read(rx_ring_base + 0x0c));

        // Write RX ring configuration (Linux order: base, size, then indices)
        // Note: No BASE_HI write - it would corrupt next ring's desc_base!
        self.write(rx_ring_base + wfdma::RING_BASE, rx_ring.desc_paddr as u32);
        self.write(rx_ring_base + wfdma::RING_MAX_CNT, rx_ring.size);
        self.write(rx_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(rx_ring_base + wfdma::RING_DMA_IDX, 0);

        // Verify RX ring configuration
        trace!(DMA, "RX ring@WFDMA+0x{:x} after write:", rx_ring_base);
        let rx_rb_lo = self.read(rx_ring_base + wfdma::RING_BASE);
        let rx_rb_cnt = self.read(rx_ring_base + wfdma::RING_MAX_CNT);
        let rx_rb_cpu = self.read(rx_ring_base + wfdma::RING_CPU_IDX);
        let rx_rb_dma = self.read(rx_ring_base + wfdma::RING_DMA_IDX);
        trace!(DMA, "  base=0x{:08x}, cnt={}, cpu={}, dma={}",
            rx_rb_lo, rx_rb_cnt, rx_rb_cpu, rx_rb_dma);

        // For RX, we need to set CPU_IDX to size-1 to indicate all buffers are available
        // (Linux does this with mt76_queue_update_read_idx)
        self.write(rx_ring_base + wfdma::RING_CPU_IDX, rx_ring.size - 1);
        trace!(DMA, "RX MCU ring cpu_idx set to {} (all buffers available)", rx_ring.size - 1);

        // Allocate RX WA ring (rx ring 1) - for receiving WA event responses
        // Per deepwiki: ALL configured queues need valid descriptors for RST auto-release
        userlib::println!("[DMA] Allocating RX WA ring (rx ring 1)...");
        self.rx_wa_ring = RxRing::new(wfdma::RX_MCU_RING_SIZE);
        if self.rx_wa_ring.is_none() {
            userlib::println!("[DMA] ERROR: Failed to allocate RX WA ring");
            self.set_error();
            return false;
        }

        let rx_wa_ring = self.rx_wa_ring.as_ref().unwrap();
        userlib::println!("[DMA] RX WA ring: vaddr=0x{:x}, paddr=0x{:x}, size={}",
            rx_wa_ring.desc_vaddr, rx_wa_ring.desc_paddr, rx_wa_ring.size);

        // Configure RX WA ring registers (rx ring 1)
        // RX rings use hardware indices directly
        let rx_wa_ring_base = wfdma::RX_RING_BASE + (wfdma::RXQ_MCU_WA) * 0x10;
        userlib::println!("[DMA] RX WA Ring@WFDMA+0x{:x} programming...", rx_wa_ring_base);
        self.write(rx_wa_ring_base + wfdma::RING_BASE, rx_wa_ring.desc_paddr as u32);
        self.write(rx_wa_ring_base + wfdma::RING_MAX_CNT, rx_wa_ring.size);
        self.write(rx_wa_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(rx_wa_ring_base + wfdma::RING_DMA_IDX, 0);
        // Set CPU_IDX to size-1 to indicate all buffers available
        self.write(rx_wa_ring_base + wfdma::RING_CPU_IDX, rx_wa_ring.size - 1);
        let rx_wa_base_rb = self.read(rx_wa_ring_base + wfdma::RING_BASE);
        let rx_wa_cnt_rb = self.read(rx_wa_ring_base + wfdma::RING_MAX_CNT);
        userlib::println!("[DMA] RX WA Ring after write: base=0x{:08x} cnt={}", rx_wa_base_rb, rx_wa_cnt_rb);

        // Allocate RX WA MAIN ring (RXQ 2) - TXFREE via WA
        // Per deepwiki: ALL configured queues need valid descriptors for RST auto-release
        userlib::println!("[DMA] Allocating RX WA MAIN ring (rx ring 2)...");
        self.rx_wa_main_ring = RxRing::new(wfdma::RX_MCU_RING_SIZE);
        if self.rx_wa_main_ring.is_none() {
            userlib::println!("[DMA] ERROR: Failed to allocate RX WA MAIN ring");
            self.set_error();
            return false;
        }
        let rx_wa_main = self.rx_wa_main_ring.as_ref().unwrap();
        userlib::println!("[DMA] RX WA MAIN ring: paddr=0x{:x}, size={}", rx_wa_main.desc_paddr, rx_wa_main.size);
        // Configure RX WA MAIN ring - hardware queue index 2, offset 0x520
        let rx_wa_main_ring_base = wfdma::RX_RING_BASE + wfdma::RXQ_MCU_WA_MAIN * 0x10;
        userlib::println!("[DMA] RX WA MAIN Ring@WFDMA+0x{:x} programming...", rx_wa_main_ring_base);
        self.write(rx_wa_main_ring_base + wfdma::RING_BASE, rx_wa_main.desc_paddr as u32);
        self.write(rx_wa_main_ring_base + wfdma::RING_MAX_CNT, rx_wa_main.size);
        self.write(rx_wa_main_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(rx_wa_main_ring_base + wfdma::RING_DMA_IDX, 0);
        self.write(rx_wa_main_ring_base + wfdma::RING_CPU_IDX, rx_wa_main.size - 1);
        let rb = self.read(rx_wa_main_ring_base + wfdma::RING_BASE);
        userlib::println!("[DMA] RX WA MAIN Ring after write: base=0x{:08x}", rb);

        // Allocate RX WA TRI ring (RXQ 3) - Band2 WA events (on HIF1!)
        userlib::println!("[DMA] Allocating RX WA TRI ring (rx ring 3)...");
        self.rx_wa_tri_ring = RxRing::new(wfdma::RX_MCU_RING_SIZE);
        if self.rx_wa_tri_ring.is_none() {
            userlib::println!("[DMA] ERROR: Failed to allocate RX WA TRI ring");
            self.set_error();
            return false;
        }
        let rx_wa_tri = self.rx_wa_tri_ring.as_ref().unwrap();
        userlib::println!("[DMA] RX WA TRI ring: paddr=0x{:x}, size={}", rx_wa_tri.desc_paddr, rx_wa_tri.size);
        // Configure RX WA TRI ring - hardware queue index 3, offset 0x530
        let rx_wa_tri_ring_base = wfdma::RX_RING_BASE + wfdma::RXQ_MCU_WA_TRI * 0x10;
        userlib::println!("[DMA] RX WA TRI Ring@WFDMA+0x{:x} programming...", rx_wa_tri_ring_base);
        self.write(rx_wa_tri_ring_base + wfdma::RING_BASE, rx_wa_tri.desc_paddr as u32);
        self.write(rx_wa_tri_ring_base + wfdma::RING_MAX_CNT, rx_wa_tri.size);
        self.write(rx_wa_tri_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(rx_wa_tri_ring_base + wfdma::RING_DMA_IDX, 0);
        self.write(rx_wa_tri_ring_base + wfdma::RING_CPU_IDX, rx_wa_tri.size - 1);
        let rb = self.read(rx_wa_tri_ring_base + wfdma::RING_BASE);
        userlib::println!("[DMA] RX WA TRI Ring after write: base=0x{:08x}", rb);

        // Allocate Data TX ring (band0) - REQUIRED per deepwiki
        // All TX rings use MCU_RING_BASE + hw_idx * 0x10 (TXQ_BAND0 = 18 → offset 0x420)
        userlib::println!("[DMA] Allocating Data TX ring (band0)...");
        self.tx_data_ring = FwdlRing::new_with_size(wfdma::TX_RING_SIZE);
        if self.tx_data_ring.is_none() {
            userlib::println!("[DMA] ERROR: Failed to allocate Data TX ring");
            self.set_error();
            return false;
        }

        let tx_data = self.tx_data_ring.as_ref().unwrap();
        userlib::println!("[DMA] Data TX ring: vaddr=0x{:x}, paddr=0x{:x}, size={}",
            tx_data.desc_vaddr, tx_data.desc_paddr, tx_data.size);

        // Configure Data TX ring registers (at MCU_RING_BASE + TXQ_BAND0 * 0x10 = 0x300 + 18*0x10 = 0x420)
        let tx_data_ring_base = wfdma::MCU_RING_BASE + wfdma::TXQ_BAND0 * 0x10;
        userlib::println!("[DMA] Data TX Ring@WFDMA+0x{:x} programming...", tx_data_ring_base);
        self.write(tx_data_ring_base + wfdma::RING_BASE, tx_data.desc_paddr as u32);
        self.write(tx_data_ring_base + wfdma::RING_MAX_CNT, tx_data.size);
        self.write(tx_data_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(tx_data_ring_base + wfdma::RING_DMA_IDX, 0);
        let tx_data_base_rb = self.read(tx_data_ring_base + wfdma::RING_BASE);
        let tx_data_cnt_rb = self.read(tx_data_ring_base + wfdma::RING_MAX_CNT);
        userlib::println!("[DMA] Data TX Ring after write: base=0x{:08x} cnt={}", tx_data_base_rb, tx_data_cnt_rb);

        // Allocate RX MAIN data ring (band0) - REQUIRED per deepwiki
        // RX MAIN is at RX ring index 4 (RXQ_BAND0)
        userlib::println!("[DMA] Allocating RX MAIN ring (band0)...");
        self.rx_main_ring = RxRing::new(wfdma::RX_DATA_RING_SIZE);
        if self.rx_main_ring.is_none() {
            userlib::println!("[DMA] ERROR: Failed to allocate RX MAIN ring");
            self.set_error();
            return false;
        }

        let rx_main = self.rx_main_ring.as_ref().unwrap();
        userlib::println!("[DMA] RX MAIN ring: vaddr=0x{:x}, paddr=0x{:x}, size={}",
            rx_main.desc_vaddr, rx_main.desc_paddr, rx_main.size);

        // Configure RX MAIN ring registers - hw ring 4 at offset 0x540
        let rx_main_ring_base = wfdma::RX_RING_BASE + (wfdma::RXQ_BAND0) * 0x10;
        userlib::println!("[DMA] RX MAIN Ring@WFDMA+0x{:x} programming...", rx_main_ring_base);
        self.write(rx_main_ring_base + wfdma::RING_BASE, rx_main.desc_paddr as u32);
        self.write(rx_main_ring_base + wfdma::RING_MAX_CNT, rx_main.size);
        self.write(rx_main_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(rx_main_ring_base + wfdma::RING_DMA_IDX, 0);
        // Set CPU_IDX to size-1 to indicate all buffers available
        self.write(rx_main_ring_base + wfdma::RING_CPU_IDX, rx_main.size - 1);
        let rx_main_base_rb = self.read(rx_main_ring_base + wfdma::RING_BASE);
        let rx_main_cnt_rb = self.read(rx_main_ring_base + wfdma::RING_MAX_CNT);
        userlib::println!("[DMA] RX MAIN Ring after write: base=0x{:08x} cnt={}", rx_main_base_rb, rx_main_cnt_rb);

        // Allocate RX TXFREE ring (band0) - CRITICAL for RST auto-release!
        // HW ring 9 at offset 0x500 + 9*0x10 = 0x590
        userlib::println!("[DMA] Allocating RX TXFREE ring (band0)...");
        self.rx_txfree_ring = RxRing::new(wfdma::RX_TXFREE_RING_SIZE);
        if self.rx_txfree_ring.is_none() {
            userlib::println!("[DMA] ERROR: Failed to allocate RX TXFREE ring");
            return false;
        }
        let rx_txfree = self.rx_txfree_ring.as_ref().unwrap();
        userlib::println!("[DMA] RX TXFREE ring: vaddr=0x{:x}, paddr=0x{:x}, size={}",
            rx_txfree.desc_vaddr, rx_txfree.desc_paddr, rx_txfree.size);

        // Configure RX TXFREE ring registers at hw ring 9
        let rx_txfree_ring_base = wfdma::RX_RING_BASE + wfdma::RXQ_TXFREE_BAND0 * 0x10;
        userlib::println!("[DMA] RX TXFREE Ring@WFDMA+0x{:x} programming...", rx_txfree_ring_base);
        self.write(rx_txfree_ring_base + wfdma::RING_BASE, rx_txfree.desc_paddr as u32);
        self.write(rx_txfree_ring_base + wfdma::RING_MAX_CNT, rx_txfree.size);
        self.write(rx_txfree_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(rx_txfree_ring_base + wfdma::RING_DMA_IDX, 0);
        // Set CPU_IDX to size-1 to indicate all buffers available
        self.write(rx_txfree_ring_base + wfdma::RING_CPU_IDX, rx_txfree.size - 1);
        let rx_txfree_base_rb = self.read(rx_txfree_ring_base + wfdma::RING_BASE);
        let rx_txfree_cnt_rb = self.read(rx_txfree_ring_base + wfdma::RING_MAX_CNT);
        userlib::println!("[DMA] RX TXFREE Ring after write: base=0x{:08x} cnt={}", rx_txfree_base_rb, rx_txfree_cnt_rb);

        // Allocate RX RRO ring (band0) - Hardware Receive Reorder
        // HW ring 8 at offset 0x500 + 8*0x10 = 0x580
        userlib::println!("[DMA] Allocating RX RRO BAND0 ring...");
        self.rx_rro_band0_ring = RxRing::new_rro(wfdma::RX_RRO_RING_SIZE);
        if self.rx_rro_band0_ring.is_none() {
            userlib::println!("[DMA] ERROR: Failed to allocate RX RRO BAND0 ring");
            return false;
        }
        let rx_rro = self.rx_rro_band0_ring.as_ref().unwrap();
        userlib::println!("[DMA] RX RRO BAND0 ring: paddr=0x{:x}, size={}", rx_rro.desc_paddr, rx_rro.size);
        // Configure RX RRO BAND0 ring at hw ring 8
        let rx_rro_ring_base = wfdma::RX_RING_BASE + wfdma::RXQ_RRO_BAND0 * 0x10;
        userlib::println!("[DMA] RX RRO BAND0 Ring@WFDMA+0x{:x} programming...", rx_rro_ring_base);
        self.write(rx_rro_ring_base + wfdma::RING_BASE, rx_rro.desc_paddr as u32);
        self.write(rx_rro_ring_base + wfdma::RING_MAX_CNT, rx_rro.size);
        self.write(rx_rro_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(rx_rro_ring_base + wfdma::RING_DMA_IDX, 0);
        self.write(rx_rro_ring_base + wfdma::RING_CPU_IDX, rx_rro.size - 1);
        let rb = self.read(rx_rro_ring_base + wfdma::RING_BASE);
        userlib::println!("[DMA] RX RRO BAND0 Ring after write: base=0x{:08x}", rb);

        // === HIF2 Ring Configuration (CRITICAL for dual-HIF RST release!) ===
        // HIF2 needs dedicated rings separate from HIF1. Without these, RST won't release.
        if self.dev.has_hif2() {
            userlib::println!("[DMA] === HIF2 Ring Configuration ===");

            // Allocate HIF2 RX TXFREE ring (band2)
            // Hardware queue index 7 (RXQ_TXFREE_BAND2), ring offset = 0x500 + 7*0x10 = 0x570
            userlib::println!("[DMA] Allocating HIF2 RX TXFREE ring (band2)...");
            self.hif2_rx_txfree_ring = RxRing::new(wfdma::RX_TXFREE_RING_SIZE);
            if let Some(ref hif2_txfree) = self.hif2_rx_txfree_ring {
                userlib::println!("[DMA] HIF2 TXFREE ring: paddr=0x{:x}, size={}",
                    hif2_txfree.desc_paddr, hif2_txfree.size);

                // Configure HIF2 RX TXFREE ring via HIF1+0x4000 offset
                let hif2_txfree_ring_base = wfdma::WFDMA0_BASE + wfdma::RX_RING_BASE + wfdma::RXQ_TXFREE_BAND2 * 0x10;
                userlib::println!("[DMA] HIF2 TXFREE Ring@0x{:x} programming (via HIF1+0x4000)...", hif2_txfree_ring_base);
                self.dev.hif2_via_hif1_write32(hif2_txfree_ring_base + wfdma::RING_BASE, hif2_txfree.desc_paddr as u32);
                self.dev.hif2_via_hif1_write32(hif2_txfree_ring_base + wfdma::RING_MAX_CNT, hif2_txfree.size);
                self.dev.hif2_via_hif1_write32(hif2_txfree_ring_base + wfdma::RING_CPU_IDX, 0);
                self.dev.hif2_via_hif1_write32(hif2_txfree_ring_base + wfdma::RING_DMA_IDX, 0);
                self.dev.hif2_via_hif1_write32(hif2_txfree_ring_base + wfdma::RING_CPU_IDX, hif2_txfree.size - 1);

                let base_rb = self.dev.hif2_via_hif1_read32(hif2_txfree_ring_base + wfdma::RING_BASE);
                userlib::println!("[DMA] HIF2 TXFREE Ring readback: base=0x{:08x}", base_rb);
            } else {
                userlib::println!("[DMA] WARNING: Failed to allocate HIF2 TXFREE ring");
            }

            // Allocate HIF2 RX data ring (band2)
            // Hardware queue index 5 (RXQ_BAND2), ring offset = 0x500 + 5*0x10 = 0x550
            userlib::println!("[DMA] Allocating HIF2 RX data ring (band2)...");
            self.hif2_rx_data_ring = RxRing::new(wfdma::RX_DATA_RING_SIZE);
            if let Some(ref hif2_rx_data) = self.hif2_rx_data_ring {
                userlib::println!("[DMA] HIF2 RX data ring: paddr=0x{:x}, size={}",
                    hif2_rx_data.desc_paddr, hif2_rx_data.size);

                // Configure HIF2 RX data ring via HIF1+0x4000 offset
                let hif2_rx_ring_base = wfdma::WFDMA0_BASE + wfdma::RX_RING_BASE + wfdma::RXQ_BAND2 * 0x10;
                userlib::println!("[DMA] HIF2 RX data Ring@0x{:x} programming...", hif2_rx_ring_base);
                self.dev.hif2_via_hif1_write32(hif2_rx_ring_base + wfdma::RING_BASE, hif2_rx_data.desc_paddr as u32);
                self.dev.hif2_via_hif1_write32(hif2_rx_ring_base + wfdma::RING_MAX_CNT, hif2_rx_data.size);
                self.dev.hif2_via_hif1_write32(hif2_rx_ring_base + wfdma::RING_CPU_IDX, 0);
                self.dev.hif2_via_hif1_write32(hif2_rx_ring_base + wfdma::RING_DMA_IDX, 0);
                self.dev.hif2_via_hif1_write32(hif2_rx_ring_base + wfdma::RING_CPU_IDX, hif2_rx_data.size - 1);

                let base_rb = self.dev.hif2_via_hif1_read32(hif2_rx_ring_base + wfdma::RING_BASE);
                userlib::println!("[DMA] HIF2 RX data Ring readback: base=0x{:08x}", base_rb);
            } else {
                userlib::println!("[DMA] WARNING: Failed to allocate HIF2 RX data ring");
            }

            // Allocate HIF2 TX data ring (band1) - ring 19
            // Ring 19: MCU_RING_BASE + 19*0x10 = 0x300 + 0x130 = 0x430
            userlib::println!("[DMA] Allocating HIF2 TX data ring (band1, ring19)...");
            self.hif2_tx_band1_ring = FwdlRing::new_with_size(wfdma::TX_RING_SIZE);
            if let Some(ref hif2_tx_band1) = self.hif2_tx_band1_ring {
                userlib::println!("[DMA] HIF2 TX band1 ring: paddr=0x{:x}, size={}",
                    hif2_tx_band1.desc_paddr, hif2_tx_band1.size);

                // Configure HIF2 TX ring via HIF1+0x4000 offset
                let hif2_tx_band1_base = wfdma::WFDMA0_BASE + wfdma::MCU_RING_BASE + wfdma::TXQ_BAND1 * 0x10;
                userlib::println!("[DMA] HIF2 TX band1 Ring@0x{:x} programming...", hif2_tx_band1_base);
                self.dev.hif2_via_hif1_write32(hif2_tx_band1_base + wfdma::RING_BASE, hif2_tx_band1.desc_paddr as u32);
                self.dev.hif2_via_hif1_write32(hif2_tx_band1_base + wfdma::RING_MAX_CNT, hif2_tx_band1.size);
                self.dev.hif2_via_hif1_write32(hif2_tx_band1_base + wfdma::RING_CPU_IDX, 0);
                self.dev.hif2_via_hif1_write32(hif2_tx_band1_base + wfdma::RING_DMA_IDX, 0);

                let base_rb = self.dev.hif2_via_hif1_read32(hif2_tx_band1_base + wfdma::RING_BASE);
                userlib::println!("[DMA] HIF2 TX band1 Ring readback: base=0x{:08x}", base_rb);
            } else {
                userlib::println!("[DMA] WARNING: Failed to allocate HIF2 TX band1 ring");
            }

            // Allocate HIF2 TX data ring (band2) - ring 21
            // Ring 21: MCU_RING_BASE + 21*0x10 = 0x300 + 0x150 = 0x450
            userlib::println!("[DMA] Allocating HIF2 TX data ring (band2, ring21)...");
            self.hif2_tx_band2_ring = FwdlRing::new_with_size(wfdma::TX_RING_SIZE);
            if let Some(ref hif2_tx_band2) = self.hif2_tx_band2_ring {
                userlib::println!("[DMA] HIF2 TX band2 ring: paddr=0x{:x}, size={}",
                    hif2_tx_band2.desc_paddr, hif2_tx_band2.size);

                // Configure HIF2 TX ring via HIF1+0x4000 offset
                let hif2_tx_band2_base = wfdma::WFDMA0_BASE + wfdma::MCU_RING_BASE + wfdma::TXQ_BAND2 * 0x10;
                userlib::println!("[DMA] HIF2 TX band2 Ring@0x{:x} programming...", hif2_tx_band2_base);
                self.dev.hif2_via_hif1_write32(hif2_tx_band2_base + wfdma::RING_BASE, hif2_tx_band2.desc_paddr as u32);
                self.dev.hif2_via_hif1_write32(hif2_tx_band2_base + wfdma::RING_MAX_CNT, hif2_tx_band2.size);
                self.dev.hif2_via_hif1_write32(hif2_tx_band2_base + wfdma::RING_CPU_IDX, 0);
                self.dev.hif2_via_hif1_write32(hif2_tx_band2_base + wfdma::RING_DMA_IDX, 0);

                let base_rb = self.dev.hif2_via_hif1_read32(hif2_tx_band2_base + wfdma::RING_BASE);
                userlib::println!("[DMA] HIF2 TX band2 Ring readback: base=0x{:08x}", base_rb);
            } else {
                userlib::println!("[DMA] WARNING: Failed to allocate HIF2 TX band2 ring");
            }

            // Allocate HIF2 RX RRO ring (band2) - Hardware Receive Reorder
            // Hardware queue index 6 (RXQ_RRO_BAND2), ring offset = 0x500 + 6*0x10 = 0x560
            userlib::println!("[DMA] Allocating HIF2 RX RRO ring (band2)...");
            self.hif2_rx_rro_band2_ring = RxRing::new_rro(wfdma::RX_RRO_RING_SIZE);
            if let Some(ref hif2_rro) = self.hif2_rx_rro_band2_ring {
                userlib::println!("[DMA] HIF2 RRO band2 ring: paddr=0x{:x}, size={}",
                    hif2_rro.desc_paddr, hif2_rro.size);

                // Configure HIF2 RX RRO ring via HIF1+0x4000 offset
                let hif2_rro_ring_base = wfdma::WFDMA0_BASE + wfdma::RX_RING_BASE + wfdma::RXQ_RRO_BAND2 * 0x10;
                userlib::println!("[DMA] HIF2 RRO band2 Ring@0x{:x} programming (via HIF1+0x4000)...", hif2_rro_ring_base);
                self.dev.hif2_via_hif1_write32(hif2_rro_ring_base + wfdma::RING_BASE, hif2_rro.desc_paddr as u32);
                self.dev.hif2_via_hif1_write32(hif2_rro_ring_base + wfdma::RING_MAX_CNT, hif2_rro.size);
                self.dev.hif2_via_hif1_write32(hif2_rro_ring_base + wfdma::RING_CPU_IDX, 0);
                self.dev.hif2_via_hif1_write32(hif2_rro_ring_base + wfdma::RING_DMA_IDX, 0);
                self.dev.hif2_via_hif1_write32(hif2_rro_ring_base + wfdma::RING_CPU_IDX, hif2_rro.size - 1);

                let base_rb = self.dev.hif2_via_hif1_read32(hif2_rro_ring_base + wfdma::RING_BASE);
                userlib::println!("[DMA] HIF2 RRO band2 Ring readback: base=0x{:08x}", base_rb);
            } else {
                userlib::println!("[DMA] WARNING: Failed to allocate HIF2 RRO band2 ring");
            }

            userlib::println!("[DMA] HIF2 ring configuration complete");
        }

        // NOTE: All the following moved to enable phase (after HIF_MISC poll) per Linux order:
        // - GLO_CFG_EXT0/EXT1
        // - RX pause thresholds
        // - HOST_CONFIG, AXI_R2A_CTRL, RX_INT_PCIE_SEL

        // Dump ALL rings BEFORE enabling DMA
        userlib::println!("[DMA] Ring dump BEFORE enable:");
        userlib::println!("[DMA]   TX MCU rings:");
        // FWDL=16, WM=17, WA=20 (using hw_ring_idx from McuQueue)
        for queue in [McuQueue::Fwdl, McuQueue::Wm, McuQueue::Wa] {
            let rbase = queue.ring_offset();
            let base = self.read(rbase);
            let cnt = self.read(rbase + 4);
            let name = match queue {
                McuQueue::Fwdl => "FWDL",
                McuQueue::Wm => "WM",
                McuQueue::Wa => "WA",
            };
            userlib::println!("[DMA]     {} (hw ring {}) @0x{:x}: base=0x{:08x} cnt={}",
                name, queue.hw_ring_idx(), rbase, base, cnt);
        }
        userlib::println!("[DMA]   TX Data ring BAND0 (hw ring 18):");
        {
            let rbase = wfdma::MCU_RING_BASE + wfdma::TXQ_BAND0 * 0x10;
            let base = self.read(rbase);
            let cnt = self.read(rbase + 4);
            userlib::println!("[DMA]     BAND0 @0x{:x}: base=0x{:08x} cnt={}", rbase, base, cnt);
        }
        userlib::println!("[DMA]   RX MCU rings (hw indices 0-3):");
        for i in 0..4u32 {
            // MCU RX rings use hardware indices 0, 1, 2, 3 directly
            let rbase = wfdma::RX_RING_BASE + i * 0x10;
            let base = self.read(rbase);
            let cnt = self.read(rbase + 4);
            let cpu = self.read(rbase + 8);
            let name = match i { 0 => "MCU", 1 => "WA", 2 => "WA_MAIN", 3 => "WA_TRI", _ => "?" };
            userlib::println!("[DMA]     {} (ring{}) @0x{:x}: base=0x{:08x} cnt={} cpu={}", name, i, rbase, base, cnt, cpu);
        }
        userlib::println!("[DMA]   RX Data rings:");
        {
            // MAIN at hw ring 4 → 0x540
            let rbase = wfdma::RX_RING_BASE + wfdma::RXQ_BAND0 * 0x10;
            let base = self.read(rbase);
            let cnt = self.read(rbase + 4);
            let cpu = self.read(rbase + 8);
            let ring_num = wfdma::RXQ_BAND0;
            userlib::println!("[DMA]     MAIN (ring{}) @0x{:x}: base=0x{:08x} cnt={} cpu={}", ring_num, rbase, base, cnt, cpu);
        }
        {
            // TXFREE at hw ring 9 → 0x590
            let rbase = wfdma::RX_RING_BASE + wfdma::RXQ_TXFREE_BAND0 * 0x10;
            let base = self.read(rbase);
            let cnt = self.read(rbase + 4);
            let cpu = self.read(rbase + 8);
            userlib::println!("[DMA]     TXFREE (rx12) @0x{:x}: base=0x{:08x} cnt={} cpu={}", rbase, base, cnt, cpu);
        }
        {
            // RRO BAND0 at hw ring 8 → 0x580
            let rbase = wfdma::RX_RING_BASE + wfdma::RXQ_RRO_BAND0 * 0x10;
            let base = self.read(rbase);
            let cnt = self.read(rbase + 4);
            let cpu = self.read(rbase + 8);
            userlib::println!("[DMA]     RRO_B0 (rx11) @0x{:x}: base=0x{:08x} cnt={} cpu={}", rbase, base, cnt, cpu);
        }

        // RST should still be 0x30 (Linux keeps it asserted during ring programming)
        let rst_val = self.read(wfdma::RST);
        userlib::println!("[DMA] RST after ring programming: 0x{:08x} (should be 0x30)", rst_val);

        // === HIF2 band-to-PCIe mapping (per Linux dma_init, NOT dma_enable!) ===
        // These are configured in mt7996_dma_init() before driver ownership
        if self.dev.has_hif2() {
            // MT_WFDMA_HOST_CONFIG (0xd7030) - band-to-PCIe mapping
            userlib::println!("[DMA] Configuring WFDMA_HOST_CONFIG (init phase)...");
            let host_cfg = self.dev.read32_raw(wfdma::HOST_CONFIG);
            userlib::println!("[DMA]   HOST_CONFIG before: 0x{:08x}", host_cfg);
            let host_cfg_step1 = host_cfg | wfdma::HOST_CONFIG_PDMA_BAND;
            self.dev.write32_raw(wfdma::HOST_CONFIG, host_cfg_step1);
            let clear_mask = wfdma::HOST_CONFIG_BAND0_PCIE1 |
                             wfdma::HOST_CONFIG_BAND1_PCIE1 |
                             wfdma::HOST_CONFIG_BAND2_PCIE1;
            let host_cfg_step2 = self.dev.read32_raw(wfdma::HOST_CONFIG) & !clear_mask;
            self.dev.write32_raw(wfdma::HOST_CONFIG, host_cfg_step2);
            let host_cfg_final = self.dev.read32_raw(wfdma::HOST_CONFIG) | wfdma::HOST_CONFIG_BAND2_PCIE1;
            self.dev.write32_raw(wfdma::HOST_CONFIG, host_cfg_final);
            let host_cfg_verify = self.dev.read32_raw(wfdma::HOST_CONFIG);
            userlib::println!("[DMA]   HOST_CONFIG after: 0x{:08x} (expected 0x400001)", host_cfg_verify);

            // MT_WFDMA_AXI_R2A_CTRL (0xd7500)
            userlib::println!("[DMA] Configuring WFDMA_AXI_R2A_CTRL (init phase)...");
            let axi_ctrl = self.dev.read32_raw(wfdma::AXI_R2A_CTRL);
            userlib::println!("[DMA]   AXI_R2A_CTRL before: 0x{:08x}", axi_ctrl);
            let axi_ctrl_new = (axi_ctrl & !wfdma::AXI_R2A_CTRL_OUTSTAND_MASK) | 0x14;
            self.dev.write32_raw(wfdma::AXI_R2A_CTRL, axi_ctrl_new);
            userlib::println!("[DMA]   AXI_R2A_CTRL after: 0x{:08x}", self.dev.read32_raw(wfdma::AXI_R2A_CTRL));

            // MT_WFDMA0_RX_INT_PCIE_SEL (0xd4154)
            userlib::println!("[DMA] Configuring RX_INT_PCIE_SEL (init phase)...");
            let rx_int_sel = self.read(wfdma::RX_INT_PCIE_SEL);
            userlib::println!("[DMA]   RX_INT_PCIE_SEL before: 0x{:08x}", rx_int_sel);
            let rx_int_sel_new = rx_int_sel | wfdma::RX_INT_SEL_RING3;
            self.write(wfdma::RX_INT_PCIE_SEL, rx_int_sel_new);
            userlib::println!("[DMA]   RX_INT_PCIE_SEL after: 0x{:08x}", self.read(wfdma::RX_INT_PCIE_SEL));
        }

        // IMPORTANT: Do NOT manually clear RST bits!
        // Per deepwiki: "Reset bits are never explicitly cleared - the hardware automatically
        // releases them when the DMA engine transitions from disabled to enabled state
        // through the MT_WFDMA0_GLO_CFG register."

        // Ring allocation complete - caller should call enable() after driver ownership
        self.state = WfdmaState::Reset;
        userlib::println!("[DMA] Ring allocation complete. Call enable() after driver ownership.");
        true
    }

    /// Enable WFDMA after driver ownership acquired
    ///
    /// Linux order: dma_enable() is called AFTER driver_own() in load_firmware()
    /// This function completes the DMA initialization sequence.
    pub fn enable(&mut self) -> bool {
        userlib::println!("[DMA] Enabling WFDMA (after driver ownership)...");

        // FWDL ring base for debugging (hw ring 16 = offset 0x100 from MCU_RING_BASE)
        let ring_base = McuQueue::Fwdl.ring_offset();

        // STEP 1: Write RST_DTX_PTR first (resets DMA indices)
        userlib::println!("[DMA] Writing RST_DTX_PTR=0xFFFFFFFF (both interfaces)...");
        self.write(wfdma::RST_DTX_PTR, 0xFFFF_FFFF);
        if self.dev.has_hif2() {
            self.dev.hif2_via_hif1_write32(wfdma::WFDMA0_BASE + wfdma::RST_DTX_PTR, 0xFFFF_FFFF);
            userlib::println!("[DMA]   HIF2 RST_DTX_PTR written");
        }

        // Check FWDL ring state after RST_DTX_PTR (should still be intact)
        let fwdl_after_dtp = self.read(ring_base + wfdma::RING_BASE);
        userlib::println!("[DMA] FWDL ring base after RST_DTX_PTR: 0x{:08x}", fwdl_after_dtp);

        // STEP 2: Clear delay interrupt configs (per deepwiki dma.c:368-377)
        userlib::println!("[DMA] Clearing PRI_DLY_INT_CFG0/1/2...");
        self.write(wfdma::PRI_DLY_INT_CFG0, 0);
        self.write(wfdma::PRI_DLY_INT_CFG1, 0);
        self.write(wfdma::PRI_DLY_INT_CFG2, 0);
        if self.dev.has_hif2() {
            self.dev.hif2_via_hif1_write32(wfdma::WFDMA0_BASE + wfdma::PRI_DLY_INT_CFG0, 0);
            self.dev.hif2_via_hif1_write32(wfdma::WFDMA0_BASE + wfdma::PRI_DLY_INT_CFG1, 0);
            self.dev.hif2_via_hif1_write32(wfdma::WFDMA0_BASE + wfdma::PRI_DLY_INT_CFG2, 0);
        }

        // STEP 3: Configure prefetch (per Linux mt7996_dma_enable -> mt7996_dma_prefetch)
        userlib::println!("[DMA] Configuring prefetch (Linux order: after PRI_DLY_INT clear)...");
        self.configure_prefetch();

        // STEP 4: Configure BUSY_ENA (per deepwiki: AFTER prefetch, BEFORE HIF_MISC_BUSY wait)
        userlib::println!("[DMA] Configuring BUSY_ENA...");
        let busy_bits = wfdma::BUSY_ENA_TX_FIFO0 | wfdma::BUSY_ENA_TX_FIFO1 | wfdma::BUSY_ENA_RX_FIFO;
        let busy_old = self.read(wfdma::BUSY_ENA);
        self.write(wfdma::BUSY_ENA, busy_old | busy_bits);
        userlib::println!("[DMA] BUSY_ENA: 0x{:08x} -> 0x{:08x}", busy_old, self.read(wfdma::BUSY_ENA));

        // Configure BUSY_ENA on HIF2 as well (PCIE1 uses different RX_FIFO bit!)
        if self.dev.has_hif2() {
            let hif2_busy_bits = wfdma::PCIE1_BUSY_ENA_TX_FIFO0 |
                                 wfdma::PCIE1_BUSY_ENA_TX_FIFO1 |
                                 wfdma::PCIE1_BUSY_ENA_RX_FIFO;
            let hif2_busy_offset = wfdma::WFDMA0_BASE + wfdma::BUSY_ENA;
            let hif2_busy_old = self.dev.hif2_via_hif1_read32(hif2_busy_offset);
            self.dev.hif2_via_hif1_write32(hif2_busy_offset, hif2_busy_old | hif2_busy_bits);
            let hif2_busy_new = self.dev.hif2_via_hif1_read32(hif2_busy_offset);
            userlib::println!("[DMA] HIF2 BUSY_ENA: 0x{:08x} -> 0x{:08x}", hif2_busy_old, hif2_busy_new);
        }

        // Wait for HIF_MISC_BUSY to clear (prerequisite for RST auto-release)
        // Per deepwiki: DMA engine must be idle before RST bits will auto-clear
        // Pattern: delay FIRST, then check (avoids tight loops starving hardware)
        userlib::println!("[DMA] Waiting for HIF_MISC_BUSY to clear...");
        let hif_misc_addr = wfdma::EXT_CSR_HIF_MISC;
        let mut busy_timeout = false;
        for i in 0..1000 {
            userlib::delay_ms(1);  // Delay FIRST before checking
            let hif_misc = self.read_bar(hif_misc_addr);
            if (hif_misc & wfdma::EXT_CSR_HIF_MISC_BUSY) == 0 {
                userlib::println!("[DMA] HIF_MISC idle after {}ms: 0x{:08x}", i + 1, hif_misc);
                break;
            }
            if i == 999 {
                userlib::println!("[DMA] WARNING: HIF_MISC_BUSY timeout: 0x{:08x}", hif_misc);
                busy_timeout = true;
            }
        }

        // Also check HIF2's HIF_MISC_BUSY (CRITICAL for dual-HIF RST release!)
        if self.dev.has_hif2() {
            userlib::println!("[DMA] Waiting for HIF2 HIF_MISC_BUSY to clear...");
            let hif2_hif_misc_offset = wfdma::EXT_CSR_HIF_MISC;
            for i in 0..1000 {
                userlib::delay_ms(1);  // Delay FIRST before checking
                let hif2_misc = self.dev.hif2_via_hif1_read32(hif2_hif_misc_offset);
                if (hif2_misc & wfdma::EXT_CSR_HIF_MISC_BUSY) == 0 {
                    userlib::println!("[DMA] HIF2 HIF_MISC idle after {}ms: 0x{:08x}", i + 1, hif2_misc);
                    break;
                }
                if i == 999 {
                    userlib::println!("[DMA] WARNING: HIF2 HIF_MISC_BUSY timeout: 0x{:08x}", hif2_misc);
                }
            }
        }

        // STEP 6: Configure GLO_CFG_EXT0/EXT1 (per Linux mt7996_dma_enable: AFTER HIF_MISC poll)
        userlib::println!("[DMA] Configuring GLO_CFG_EXT0/EXT1 (Linux order: after HIF_MISC poll)...");
        let ext0 = self.read(wfdma::GLO_CFG_EXT0);
        userlib::println!("[DMA]   GLO_CFG_EXT0 before: 0x{:08x}", ext0);
        let ext0_new = ext0 | wfdma::GLO_CFG_EXT0_RX_WB_RXD | wfdma::GLO_CFG_EXT0_WED_MERGE_MODE;
        self.write(wfdma::GLO_CFG_EXT0, ext0_new);
        let ext0_after = self.read(wfdma::GLO_CFG_EXT0);
        userlib::println!("[DMA]   GLO_CFG_EXT0 after: 0x{:08x} (set RX_WB_RXD+WED_MERGE_MODE)", ext0_after);
        let ext1 = self.read(wfdma::GLO_CFG_EXT1);
        self.write(wfdma::GLO_CFG_EXT1, ext1 | wfdma::GLO_CFG_EXT1_TX_FCTRL_MODE);
        userlib::println!("[DMA]   GLO_CFG_EXT1: 0x{:08x} -> 0x{:08x}",
            ext1, self.read(wfdma::GLO_CFG_EXT1));

        // Configure HIF2 GLO_CFG_EXT0/EXT1 as well (Linux dma.c:413-420)
        if self.dev.has_hif2() {
            let hif1_ofs = wfdma::WFDMA0_BASE;
            let ext0_offset = hif1_ofs + wfdma::GLO_CFG_EXT0;
            let ext1_offset = hif1_ofs + wfdma::GLO_CFG_EXT1;
            let hif2_ext0 = self.dev.hif2_via_hif1_read32(ext0_offset);
            let hif2_ext0_new = hif2_ext0 | wfdma::GLO_CFG_EXT0_RX_WB_RXD |
                                wfdma::GLO_CFG_EXT0_WED_MERGE_MODE;
            self.dev.hif2_via_hif1_write32(ext0_offset, hif2_ext0_new);
            let hif2_ext1 = self.dev.hif2_via_hif1_read32(ext1_offset);
            self.dev.hif2_via_hif1_write32(ext1_offset, hif2_ext1 | wfdma::GLO_CFG_EXT1_TX_FCTRL_MODE);
            userlib::println!("[DMA] HIF2 GLO_CFG_EXT0/EXT1 configured");
        }

        // STEP 7: Set RX pause thresholds (per Linux mt7996_dma_enable: AFTER GLO_CFG_EXT)
        userlib::println!("[DMA] Setting RX pause thresholds (Linux values from dma.c:407-410)...");
        self.write(wfdma::PAUSE_RX_Q_45_TH, 0xc000c);
        self.write(wfdma::PAUSE_RX_Q_67_TH, 0x10008);
        self.write(wfdma::PAUSE_RX_Q_89_TH, 0x10008);
        self.write(wfdma::PAUSE_RX_Q_RRO_TH, 0x20);

        // Set HIF2 thresholds as well (Linux dma.c:466-469)
        if self.dev.has_hif2() {
            let hif1_ofs = wfdma::WFDMA0_BASE;
            self.dev.hif2_via_hif1_write32(hif1_ofs + wfdma::PAUSE_RX_Q_45_TH, 0xc000c);
            self.dev.hif2_via_hif1_write32(hif1_ofs + wfdma::PAUSE_RX_Q_67_TH, 0x10008);
            self.dev.hif2_via_hif1_write32(hif1_ofs + wfdma::PAUSE_RX_Q_89_TH, 0x10008);
            self.dev.hif2_via_hif1_write32(hif1_ofs + wfdma::PAUSE_RX_Q_RRO_TH, 0x20);
            userlib::println!("[DMA] HIF2 RX pause thresholds set");
        }

        // STEP 8: Enable DMA - hardware auto-releases RST when GLO_CFG enables DMA
        // CRITICAL: mt76 uses mt76_set() which only ORs in bits, never clears.
        // Do NOT use & !BYTE_SWAP or any clearing - that can gate reset FSM completion.
        // NOTE: HOST_CONFIG, AXI_R2A_CTRL, RX_INT_PCIE_SEL already configured in init()
        userlib::println!("[DMA] Enabling DMA engines (both interfaces)...");

        // Check RST before GLO_CFG enable (should still be 0x30)
        let rst_before = self.read(wfdma::RST);
        userlib::println!("[DMA] RST before GLO_CFG enable: 0x{:08x} (expected 0x30)", rst_before);

        let glo_val = self.read(wfdma::GLO_CFG);
        userlib::println!("[DMA] HIF1 GLO_CFG before enable: 0x{:08x}", glo_val);
        // EXACT bits from Linux mt7996_dma_start() dma.c:555-560
        let enable_bits = glo_cfg::TX_DMA_EN | glo_cfg::RX_DMA_EN |
                          glo_cfg::OMIT_TX_INFO | glo_cfg::OMIT_RX_INFO_PFET2 |
                          glo_cfg::EXT_EN;  // BIT(26) - CRITICAL for RST auto-release!
        let new_glo = glo_val | enable_bits;  // mt76_set semantics: ONLY set, never clear
        self.write(wfdma::GLO_CFG, new_glo);
        unsafe { core::arch::asm!("dsb sy"); }

        // Read back HIF1's ACTUAL value - hardware may modify bits
        let hif1_final = self.read(wfdma::GLO_CFG);
        userlib::println!("[DMA] HIF1 GLO_CFG after enable: 0x{:08x}", hif1_final);

        // Enable GLO_CFG on HIF2 as well (Linux does this in mt7996_dma_start)
        // CRITICAL: Do RMW on HIF2's own current value, NOT copy HIF1's value!
        // mt76 uses mt76_set() semantics: read HIF2, OR in enable bits, write back
        if self.dev.has_hif2() {
            let hif2_glo_offset = wfdma::WFDMA0_BASE + wfdma::GLO_CFG;
            let hif2_glo_before = self.dev.hif2_via_hif1_read32(hif2_glo_offset);
            userlib::println!("[DMA] HIF2 GLO_CFG before enable: 0x{:08x}", hif2_glo_before);
            // Same enable bits as HIF1, applied to HIF2's current value
            let hif2_glo_new = hif2_glo_before | enable_bits;
            self.dev.hif2_via_hif1_write32(hif2_glo_offset, hif2_glo_new);
            unsafe { core::arch::asm!("dsb sy"); }
            let hif2_glo_after = self.dev.hif2_via_hif1_read32(hif2_glo_offset);
            userlib::println!("[DMA] HIF2 GLO_CFG after enable: 0x{:08x}", hif2_glo_after);
        }

        // Give hardware time to process GLO_CFG enable and auto-release RST
        // (USB driver had similar issues with tight loops preventing HW completion)
        // Using 500ms to rule out timing issues - if this works, timing is the culprit
        userlib::println!("[DMA] Waiting 500ms after GLO_CFG enable for hardware to settle...");
        userlib::delay_ms(500);

        // Check ring state after GLO_CFG enable (hardware should have auto-released RST)
        let ring2_after_glo = self.read(ring_base + wfdma::RING_BASE);
        let rst_after_glo = self.read(wfdma::RST);
        userlib::println!("[DMA] Ring2 base after GLO_CFG enable: 0x{:08x}", ring2_after_glo);
        userlib::println!("[DMA] HIF1 RST after GLO_CFG enable: 0x{:08x} (expected 0x00)", rst_after_glo);

        // Also check HIF2's RST status - both need to auto-release
        let hif2_rst = if self.dev.has_hif2() {
            let rst_offset = wfdma::WFDMA0_BASE + wfdma::RST;
            let hif2_rst_val = self.dev.hif2_via_hif1_read32(rst_offset);
            userlib::println!("[DMA] HIF2 RST after GLO_CFG enable: 0x{:08x} (expected 0x00)", hif2_rst_val);
            hif2_rst_val
        } else {
            0
        };

        // IMPORTANT: DO NOT manually clear RST!
        // RST is designed to AUTO-RELEASE when GLO_CFG is properly configured.
        // It's a pulse at the beginning - hardware releases it when everything is in place.
        // If RST stays 0x30, it means we're missing a prerequisite, NOT that we should force it.
        // Forcing RST clear would mask the real problem and likely cause other issues.
        if rst_after_glo != 0 || hif2_rst != 0 {
            userlib::println!("[DMA] WARNING: RST didn't auto-release! HIF1=0x{:08x} HIF2=0x{:08x}",
                rst_after_glo, hif2_rst);
            userlib::println!("[DMA]   This means a prerequisite is missing. Check:");
            userlib::println!("[DMA]   - All required rings programmed with valid descriptors");
            userlib::println!("[DMA]   - Prefetch configuration complete");
            userlib::println!("[DMA]   - HIF_MISC_BUSY cleared");
            userlib::println!("[DMA]   - BUSY_ENA configured");
            userlib::println!("[DMA]   - GLO_CFG_EXT0/EXT1 configured");
        }

        // Dump ALL MCU TX rings AFTER enabling DMA
        userlib::println!("[DMA] Ring dump AFTER enable:");
        for i in 0..5u32 {
            let rbase = wfdma::MCU_RING_BASE + i * 0x10;
            let base = self.read(rbase);
            let cnt = self.read(rbase + 4);
            let cpu = self.read(rbase + 8);
            let dma = self.read(rbase + 12);
            userlib::println!("[DMA]   Ring{} @0x{:x}: base=0x{:08x} cnt={} cpu={} dma={}",
                i, rbase, base, cnt, cpu, dma);
        }

        // Final check on FWDL ring
        let rb_final = self.read(ring_base + wfdma::RING_BASE);
        if rb_final == 0 {
            userlib::println!("[DMA] ERROR: FWDL ring base is 0!");
        }

        // STEP 3: Enable interrupts - LAST step per Linux mt7996_dma_start()
        // This must be done AFTER GLO_CFG enable for proper DMA operation
        userlib::println!("[DMA] Enabling interrupts (final step)...");
        let int_mask_old = self.read(wfdma::INT_MASK);
        let int_mask_new = int_mask_old | int_mask::MCU_FULL;
        self.write(wfdma::INT_MASK, int_mask_new);
        userlib::println!("[DMA] INT_MASK: 0x{:08x} -> 0x{:08x}",
            int_mask_old, self.read(wfdma::INT_MASK));

        // Also enable interrupts on HIF2 if present
        if self.dev.has_hif2() {
            let hif2_int_mask_offset = wfdma::WFDMA0_BASE + wfdma::INT_MASK;
            let hif2_int_old = self.dev.hif2_via_hif1_read32(hif2_int_mask_offset);
            self.dev.hif2_via_hif1_write32(hif2_int_mask_offset, hif2_int_old | int_mask::MCU_FULL);
            let hif2_int_new = self.dev.hif2_via_hif1_read32(hif2_int_mask_offset);
            userlib::println!("[DMA] HIF2 INT_MASK: 0x{:08x} -> 0x{:08x}", hif2_int_old, hif2_int_new);
        }

        // FINAL RST CHECK: After all configuration including interrupts
        // (User requested: "could we check the RST status AFTER the interrupt config?")
        userlib::delay_ms(5);  // Give hardware time to settle
        let rst_final = self.read(wfdma::RST);
        userlib::println!("[DMA] **FINAL** RST after INT_MASK enable: 0x{:08x} (expected 0x00)", rst_final);
        if self.dev.has_hif2() {
            let hif2_rst_final = self.dev.hif2_via_hif1_read32(wfdma::WFDMA0_BASE + wfdma::RST);
            userlib::println!("[DMA] **FINAL** HIF2 RST after INT_MASK enable: 0x{:08x} (expected 0x00)", hif2_rst_final);
        }

        // State: Ready for transfers
        self.state = WfdmaState::Ready;
        trace!(DMA, "WFDMA initialized, state={:?}", self.state);
        true
    }

    /// Send firmware data via FWDL queue using MCU FW_SCATTER command
    pub fn send_fw_scatter(&mut self, fw_data: &[u8]) -> bool {
        // State check: must be Ready
        if self.state != WfdmaState::Ready {
            trace!(DMA, "send_fw_scatter: wrong state {:?}", self.state);
            return false;
        }

        // Build MCU command in ring's command buffer
        let (cmd_paddr, cmd_len) = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };
            ring.build_fw_scatter(fw_data)
        };

        // Get cpu_idx
        let cpu_idx = match self.fwdl_ring.as_ref() {
            Some(r) => r.cpu_idx,
            None => return false,
        };

        // Fill descriptor to point at command buffer
        let new_cpu_idx = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };

            let desc = ring.get_desc(cpu_idx);
            *desc = TxDesc::for_firmware(cmd_paddr, cmd_len, true);

            // Flush descriptor to RAM so DMA can see it
            flush_buffer(desc as *const _ as u64, core::mem::size_of::<TxDesc>());

            // Debug: dump descriptor
            if cpu_idx == 0 {
                trace!(DMA, "First descriptor:");
                trace!(DMA, "  buf0=0x{:08x} ctrl=0x{:08x}", desc.buf0, desc.ctrl);
                trace!(DMA, "  buf1=0x{:08x} info=0x{:08x}", desc.buf1, desc.info);
                trace!(DMA, "  cmd_paddr=0x{:x} cmd_len={}", cmd_paddr, cmd_len);

                // Dump first 32 bytes of command buffer (MCU TXD header)
                let cmd_ptr = ring.cmd_vaddr as *const u32;
                unsafe {
                    trace!(DMA, "  TXD[0-3]: {:08x} {:08x} {:08x} {:08x}",
                        *cmd_ptr, *cmd_ptr.add(1), *cmd_ptr.add(2), *cmd_ptr.add(3));
                    trace!(DMA, "  TXD[4-7]: {:08x} {:08x} {:08x} {:08x}",
                        *cmd_ptr.add(4), *cmd_ptr.add(5), *cmd_ptr.add(6), *cmd_ptr.add(7));
                    trace!(DMA, "  MCU[0-3]: {:08x} {:08x} {:08x} {:08x}",
                        *cmd_ptr.add(8), *cmd_ptr.add(9), *cmd_ptr.add(10), *cmd_ptr.add(11));
                }
            }

            // Advance CPU index
            ring.advance();
            ring.cpu_idx
        };

        // Update CPU index register (rings doorbell) - WFDMA relative
        let ring_base = McuQueue::Fwdl.ring_offset();

        // Debug: check state before doorbell
        if cpu_idx == 0 {
            let pre_cpu = self.read(ring_base + wfdma::RING_CPU_IDX);
            let glo_cfg = self.read(wfdma::GLO_CFG);
            trace!(DMA, "Pre-doorbell: cpu_idx={}, GLO_CFG=0x{:08x}", pre_cpu, glo_cfg);
        }

        // CRITICAL: DSB SY ensures descriptor writes are visible to DMA engine
        // before we ring the doorbell. fence(SeqCst) is not sufficient for device DMA.
        unsafe { core::arch::asm!("dsb sy"); }

        // Ring the doorbell
        self.write(ring_base + wfdma::RING_CPU_IDX, new_cpu_idx);

        // Debug: verify ring state after doorbell
        if cpu_idx == 0 {
            let after1 = self.read(ring_base + wfdma::RING_CPU_IDX);

            // Try writing again with barrier
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            self.write(ring_base + wfdma::RING_CPU_IDX, new_cpu_idx);
            let after2 = self.read(ring_base + wfdma::RING_CPU_IDX);

            let rb_dma = self.read(ring_base + wfdma::RING_DMA_IDX);
            trace!(DMA, "Post-doorbell: cpu_idx={}/{}, dma_idx={}", after1, after2, rb_dma);

            // Dump all FWDL ring registers to verify configuration
            trace!(DMA, "FWDL ring regs: {:08x} {:08x} {:08x} {:08x} {:08x}",
                self.read(ring_base + 0x00),
                self.read(ring_base + 0x04),
                self.read(ring_base + 0x08),
                self.read(ring_base + 0x0c),
                self.read(ring_base + 0x10));
        }

        true
    }

    /// Send a generic command through DMA
    fn send_command(&mut self, cmd_paddr: u64, cmd_len: usize, debug_name: &str) -> bool {
        // State check: must be Ready
        if self.state != WfdmaState::Ready {
            trace!(DMA, "send_command({}): wrong state {:?}", debug_name, self.state);
            return false;
        }

        let cpu_idx = match self.fwdl_ring.as_ref() {
            Some(r) => r.cpu_idx,
            None => return false,
        };

        let new_cpu_idx = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };

            let desc = ring.get_desc(cpu_idx);
            *desc = TxDesc::for_firmware(cmd_paddr, cmd_len, true);

            // Debug: dump descriptor content before flush
            userlib::println!("[DMA] Desc[{}]: buf0=0x{:08x} ctrl=0x{:08x} buf1=0x{:08x} info=0x{:08x}",
                cpu_idx, desc.buf0, desc.ctrl, desc.buf1, desc.info);
            userlib::println!("[DMA]   desc_vaddr=0x{:x} desc_paddr=0x{:x}",
                desc as *const _ as u64, ring.desc_paddr + (cpu_idx as u64) * 16);

            // Flush descriptor to RAM so DMA can see it
            flush_buffer(desc as *const _ as u64, core::mem::size_of::<TxDesc>());

            // Debug for first command
            trace!(DMA, "Sending {}: buf=0x{:x} len={}", debug_name, cmd_paddr, cmd_len);

            ring.advance();
            ring.cpu_idx
        };

        let ring_base = McuQueue::Fwdl.ring_offset();

        // CRITICAL: DSB SY ensures descriptor writes are visible to DMA engine
        // before we ring the doorbell. Matches send_fw_scatter() barrier.
        unsafe { core::arch::asm!("dsb sy"); }

        self.write(ring_base + wfdma::RING_CPU_IDX, new_cpu_idx);

        true
    }

    /// Send command via WM ring (ring 0)
    /// Used for MCU commands like PATCH_SEM_CTRL that go to WM queue
    fn send_command_wm(&mut self, cmd_paddr: u64, cmd_len: usize, debug_name: &str) -> bool {
        // State check: must be Ready
        if self.state != WfdmaState::Ready {
            trace!(DMA, "send_command_wm({}): wrong state {:?}", debug_name, self.state);
            return false;
        }

        let cpu_idx = match self.wm_ring.as_ref() {
            Some(r) => r.cpu_idx,
            None => {
                userlib::println!("[DMA] send_command_wm: no WM ring!");
                return false;
            }
        };

        let new_cpu_idx = {
            let ring = match self.wm_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };

            let desc = ring.get_desc(cpu_idx);
            *desc = TxDesc::for_firmware(cmd_paddr, cmd_len, true);

            // Debug: dump descriptor content before flush
            userlib::println!("[DMA] WM Desc[{}]: buf0=0x{:08x} ctrl=0x{:08x} buf1=0x{:08x} info=0x{:08x}",
                cpu_idx, desc.buf0, desc.ctrl, desc.buf1, desc.info);

            // Flush descriptor to RAM so DMA can see it
            flush_buffer(desc as *const _ as u64, core::mem::size_of::<TxDesc>());

            trace!(DMA, "Sending {} via WM ring: buf=0x{:x} len={}", debug_name, cmd_paddr, cmd_len);

            ring.advance();
            ring.cpu_idx
        };

        // Write to WM ring (ring 0) CPU index
        let ring_base = McuQueue::Wm.ring_offset();

        // CRITICAL: DSB SY ensures descriptor writes are visible to DMA engine
        // before we ring the doorbell. Matches send_fw_scatter() barrier.
        unsafe { core::arch::asm!("dsb sy"); }

        self.write(ring_base + wfdma::RING_CPU_IDX, new_cpu_idx);

        true
    }

    /// Send TARGET_ADDRESS_LEN_REQ (init_download) command
    pub fn send_init_download(&mut self, addr: u32, len: u32, mode: u32) -> bool {
        let (cmd_paddr, cmd_len) = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };
            ring.build_init_download(addr, len, mode)
        };

        userlib::println!("[DMA] init_download: addr=0x{:08x} len={} mode=0x{:x} cmd_paddr=0x{:x}",
            addr, len, mode, cmd_paddr);
        self.send_command(cmd_paddr, cmd_len, "TARGET_ADDRESS_LEN_REQ")
    }

    /// Send patch semaphore control command
    /// Per deepwiki: PATCH_SEM_CTRL goes to WM queue (ring 0), NOT FWDL queue
    pub fn send_patch_sem_ctrl(&mut self, get: bool) -> bool {
        let (cmd_paddr, cmd_len) = {
            // Use WM ring for semaphore control (not FWDL!)
            let ring = match self.wm_ring.as_mut() {
                Some(r) => r,
                None => {
                    userlib::println!("[DMA] send_patch_sem_ctrl: no WM ring!");
                    return false;
                }
            };
            ring.build_patch_sem_ctrl(get)
        };

        // Send via WM ring
        self.send_command_wm(cmd_paddr, cmd_len, if get { "SEM_GET" } else { "SEM_RELEASE" })
    }

    /// Send patch finish command
    pub fn send_patch_finish(&mut self) -> bool {
        let (cmd_paddr, cmd_len) = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };
            ring.build_patch_finish()
        };

        self.send_command(cmd_paddr, cmd_len, "PATCH_FINISH")
    }

    /// Wait for command completion - checks for MCU response or state change
    pub fn wait_dma_done(&mut self, timeout_ms: u32) -> bool {
        let ring = match self.fwdl_ring.as_ref() {
            Some(r) => r,
            None => return false,
        };

        let tx_ring_base = McuQueue::Fwdl.ring_offset();
        // RX rings use hardware indices directly
        let rx_ring_base = wfdma::RX_RING_BASE + (wfdma::RXQ_MCU) * 0x10;

        let initial_dma_idx = self.read(tx_ring_base + wfdma::RING_DMA_IDX);
        let initial_rx_dma_idx = self.read(rx_ring_base + wfdma::RING_DMA_IDX);

        for i in 0..(timeout_ms * 10) {
            // Delay FIRST before checking (avoids tight loops starving hardware)
            userlib::delay_us(100);

            // Check TX DMA progress
            let tx_dma_idx = self.read(tx_ring_base + wfdma::RING_DMA_IDX);

            // Check RX DMA progress (MCU responses)
            let rx_dma_idx = self.read(rx_ring_base + wfdma::RING_DMA_IDX);

            // Check interrupt status
            let int_src = self.read(wfdma::INT_SRC);
            let mcu_int = self.read_bar(mcu_int::EVENT);

            // If any progress or interrupt, report and continue
            if tx_dma_idx != initial_dma_idx || rx_dma_idx != initial_rx_dma_idx ||
               int_src != 0 || mcu_int != 0 {
                trace!(DMA, "Progress @{}ms: TX dma_idx={} RX dma_idx={} INT_SRC=0x{:x} MCU_INT=0x{:x}",
                    i / 10, tx_dma_idx, rx_dma_idx, int_src, mcu_int);

                // Clear interrupts
                if int_src != 0 {
                    self.write(wfdma::INT_SRC, int_src);
                }

                // Check if TX completed
                if tx_dma_idx == ring.cpu_idx {
                    return true;
                }
            }

            // Poll MCU_CMD for state changes
            let mcu_cmd = self.read(wfdma::MCU_CMD);
            if (mcu_cmd & mcu_cmd::NORMAL_STATE) != 0 {
                trace!(DMA, "MCU reached normal state: 0x{:08x}", mcu_cmd);
            }
        }

        // Final status dump
        let tx_dma_idx = self.read(tx_ring_base + wfdma::RING_DMA_IDX);
        let rx_dma_idx = self.read(rx_ring_base + wfdma::RING_DMA_IDX);
        let int_src = self.read(wfdma::INT_SRC);
        let mcu_cmd = self.read(wfdma::MCU_CMD);
        let glo_cfg = self.read(wfdma::GLO_CFG);

        userlib::println!("[DMA] Timeout after {}ms:", timeout_ms);
        userlib::println!("[DMA]   TX: cpu_idx={}, dma_idx={}", ring.cpu_idx, tx_dma_idx);
        userlib::println!("[DMA]   RX: dma_idx={}", rx_dma_idx);
        userlib::println!("[DMA]   INT_SRC=0x{:08x} MCU_CMD=0x{:08x} GLO_CFG=0x{:08x}",
            int_src, mcu_cmd, glo_cfg);

        // Also check ring base configuration
        let ring_base = self.read(tx_ring_base + wfdma::RING_BASE);
        let ring_cnt = self.read(tx_ring_base + wfdma::RING_MAX_CNT);
        userlib::println!("[DMA]   TX ring: base=0x{:08x}, cnt={}", ring_base, ring_cnt);

        // Return false on timeout - DMA didn't complete
        false
    }

    /// Wait for WM ring command completion
    pub fn wait_dma_done_wm(&mut self, timeout_ms: u32) -> bool {
        let ring = match self.wm_ring.as_ref() {
            Some(r) => r,
            None => {
                userlib::println!("[DMA] wait_dma_done_wm: no WM ring!");
                return false;
            }
        };

        let tx_ring_base = McuQueue::Wm.ring_offset();
        // RX rings use hardware indices directly
        let rx_ring_base = wfdma::RX_RING_BASE + (wfdma::RXQ_MCU) * 0x10;

        let initial_dma_idx = self.read(tx_ring_base + wfdma::RING_DMA_IDX);

        for i in 0..(timeout_ms * 10) {
            // Delay FIRST before checking (avoids tight loops starving hardware)
            userlib::delay_us(100);

            let tx_dma_idx = self.read(tx_ring_base + wfdma::RING_DMA_IDX);
            let rx_dma_idx = self.read(rx_ring_base + wfdma::RING_DMA_IDX);
            let int_src = self.read(wfdma::INT_SRC);

            if tx_dma_idx != initial_dma_idx || int_src != 0 {
                trace!(DMA, "WM Progress @{}ms: TX dma_idx={} RX dma_idx={} INT_SRC=0x{:x}",
                    i / 10, tx_dma_idx, rx_dma_idx, int_src);

                if int_src != 0 {
                    self.write(wfdma::INT_SRC, int_src);
                }

                if tx_dma_idx == ring.cpu_idx {
                    return true;
                }
            }
        }

        // Timeout - dump status
        let tx_dma_idx = self.read(tx_ring_base + wfdma::RING_DMA_IDX);
        let int_src = self.read(wfdma::INT_SRC);
        let mcu_cmd = self.read(wfdma::MCU_CMD);
        let glo_cfg = self.read(wfdma::GLO_CFG);
        let rst = self.read(wfdma::RST);

        userlib::println!("[DMA] WM Timeout after {}ms:", timeout_ms);
        userlib::println!("[DMA]   TX: cpu_idx={}, dma_idx={}", ring.cpu_idx, tx_dma_idx);
        userlib::println!("[DMA]   INT_SRC=0x{:08x} MCU_CMD=0x{:08x} GLO_CFG=0x{:08x} RST=0x{:08x}",
            int_src, mcu_cmd, glo_cfg, rst);

        let ring_base = self.read(tx_ring_base + wfdma::RING_BASE);
        let ring_cnt = self.read(tx_ring_base + wfdma::RING_MAX_CNT);
        userlib::println!("[DMA]   WM ring: base=0x{:08x}, cnt={}", ring_base, ring_cnt);

        false
    }

    /// Check MCU state
    pub fn mcu_state(&self) -> u32 {
        self.read(wfdma::MCU_CMD)
    }

    /// Set MCU command
    pub fn set_mcu_cmd(&self, cmd: u32) {
        self.write(wfdma::MCU_CMD, cmd);
    }
}
