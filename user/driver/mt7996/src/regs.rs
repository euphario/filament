//! MT7996 Register Definitions
//!
//! Direct translation of mt76/mt7996/regs.h from Linux.
//! Each section matches the C header layout exactly.
//!
//! IMPORTANT: Do not "improve" this file. Keep it 1:1 with Linux.
//! The goal is correctness through mechanical translation, not elegance.

#![allow(dead_code)]

// ============================================================================
// Helper macros as const fn
// ============================================================================

/// BIT(n) = 1 << n
pub const fn bit(n: u32) -> u32 {
    1 << n
}

/// GENMASK(hi, lo) = mask from bit lo to bit hi inclusive
pub const fn genmask(hi: u32, lo: u32) -> u32 {
    let bits = hi - lo + 1;
    (((1u64 << bits) - 1) as u32) << lo
}

// ============================================================================
// Device IDs and Variant Detection (preserved from original)
// ============================================================================

/// Device IDs
pub mod device_id {
    /// MT7996 primary device
    pub const MT7996: u16 = 0x7990;
    /// MT7996 secondary (HIF2)
    pub const MT7996_2: u16 = 0x7991;
    /// MT7992 primary device
    pub const MT7992: u16 = 0x7992;
    /// MT7992 secondary
    pub const MT7992_2: u16 = 0x799a;
    /// MT7990 primary
    pub const MT7990: u16 = 0x7993;
    /// MT7990 secondary
    pub const MT7990_2: u16 = 0x799b;
}

/// Chip variant types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipVariant {
    /// MT7996 with 4+4+4 antenna configuration
    Mt7996_444,
    /// MT7996 with 2+3+3 antenna configuration
    Mt7996_233,
    /// MT7992 variant
    Mt7992,
    /// MT7990 variant
    Mt7990,
    /// Unknown variant
    Unknown,
}

impl ChipVariant {
    /// Detect variant from device ID
    pub fn from_device_id(device_id: u16) -> Self {
        match device_id {
            device_id::MT7996 | device_id::MT7996_2 => ChipVariant::Mt7996_444,
            device_id::MT7992 | device_id::MT7992_2 => ChipVariant::Mt7992,
            device_id::MT7990 | device_id::MT7990_2 => ChipVariant::Mt7990,
            _ => ChipVariant::Unknown,
        }
    }
}

// ============================================================================
// Address ranges for L1 remapping
// ============================================================================

/// Address ranges for layer 1 remapping
pub mod l1_range {
    /// Infra range start
    pub const INFRA_START: u32 = 0x18000000;
    pub const INFRA_END: u32 = 0x18ffffff;
    /// WFSYS range start
    pub const WFSYS_START: u32 = 0x7c000000;
    pub const WFSYS_END: u32 = 0x7fffffff;
}

/// CBTOP range for PCIe-specific remapping
pub mod cbtop {
    pub const CBTOP1_PHY_START: u32 = 0x70000000;
    pub const CBTOP1_PHY_END: u32 = 0x77ffffff;
    pub const CBTOP2_PHY_START: u32 = 0x78000000;
    pub const CBTOP2_PHY_END: u32 = 0x7fffffff;
}

// ============================================================================
// RRO TOP - Reorder Buffer
// ============================================================================

pub mod rro {
    use super::{bit, genmask};

    pub const BASE: u32 = 0xA000;
    pub const fn rro(ofs: u32) -> u32 { BASE + ofs }

    pub const BA_BITMAP_BASE0: u32 = rro(0x8);
    pub const BA_BITMAP_BASE1: u32 = rro(0xC);
    pub const AXI_MST_CFG: u32 = rro(0xB8);
    pub const AXI_MST_CFG_DIDX_OK: u32 = bit(12);

    pub const ADDR_ARRAY_BASE0: u32 = rro(0x30);
    pub const ADDR_ARRAY_BASE1: u32 = rro(0x34);
    pub const ADDR_ARRAY_ELEM_ADDR_SEG_MODE: u32 = bit(31);

    pub const IND_CMD_SIGNATURE_BASE0: u32 = rro(0x38);
    pub const IND_CMD_SIGNATURE_BASE1: u32 = rro(0x3C);
    pub const IND_CMD_0_CTRL0: u32 = rro(0x40);
    pub const IND_CMD_SIGNATURE_BASE1_EN: u32 = bit(31);

    pub const PARTICULAR_CFG0: u32 = rro(0x5C);
    pub const PARTICULAR_CFG1: u32 = rro(0x60);
    pub const PARTICULAR_CONFG_EN: u32 = bit(31);
    pub const PARTICULAR_SID: u32 = genmask(30, 16);

    pub const BA_BITMAP_BASE_EXT0: u32 = rro(0x70);
    pub const BA_BITMAP_BASE_EXT1: u32 = rro(0x74);
    pub const HOST_INT_ENA: u32 = rro(0x204);
    pub const HOST_INT_ENA_HOST_RRO_DONE_ENA: u32 = bit(0);

    pub const ADDR_ELEM_SEG_ADDR0: u32 = rro(0x400);

    pub const EMU_CONF_3_0: u32 = rro(0x600);
    pub const EMU_CONF_EN_MASK: u32 = bit(11);

    pub const GLOBAL_CONFIG_3_1: u32 = rro(0x604);
    pub const GLOBAL_CONFIG_RXDMAD_SEL: u32 = bit(6);
    pub const GLOBAL_CONFIG_RX_CIDX_RD_EN: u32 = bit(3);
    pub const GLOBAL_CONFIG_RX_DIDX_WR_EN: u32 = bit(2);
    pub const GLOBAL_CONFIG_INTERLEAVE_EN: u32 = bit(0);

    pub const MSDU_PG_SEG_ADDR0: u32 = rro(0x620);
    pub const RX_RING_AP_CIDX_ADDR: u32 = rro(0x6f0);
    pub const RX_RING_AP_DIDX_ADDR: u32 = rro(0x6f4);

    pub const ACK_SN_CTRL: u32 = rro(0x50);
    pub const ACK_SN_CTRL_SN_MASK: u32 = genmask(27, 16);
    pub const ACK_SN_CTRL_SESSION_MASK: u32 = genmask(11, 0);

    pub const DBG_RD_CTRL: u32 = rro(0xe0);
    pub const DBG_RD_ADDR: u32 = genmask(15, 0);
    pub const DBG_RD_EXEC: u32 = bit(31);

    pub const fn dbg_rdat_dw(n: u32) -> u32 { rro(0xf0 + n * 0x4) }

    // Ring bases for RRO
    pub const RXQ_RRO_IND_RING_BASE: u32 = rro(0x40);
    pub const RXQ_RRO_AP_RING_BASE: u32 = rro(0x650);
}

// ============================================================================
// MCU Interrupt Event
// ============================================================================

pub mod mcu {
    use super::bit;

    pub const INT_EVENT: u32 = 0x2108;
    pub const INT_EVENT_DMA_STOPPED: u32 = bit(0);
    pub const INT_EVENT_DMA_INIT: u32 = bit(1);
    pub const INT_EVENT_RESET_DONE: u32 = bit(3);
}

// ============================================================================
// PLE - Packet List Engine
// ============================================================================

pub mod ple {
    pub const BASE: u32 = 0x820c0000;
    pub const fn ple(ofs: u32) -> u32 { BASE + ofs }

    pub const FL_Q_EMPTY: u32 = ple(0x360);
    pub const FL_Q0_CTRL: u32 = ple(0x3e0);
    pub const FL_Q2_CTRL: u32 = ple(0x3e8);
    pub const FL_Q3_CTRL: u32 = ple(0x3ec);

    pub const FREEPG_CNT: u32 = ple(0x380);
    pub const FREEPG_HEAD_TAIL: u32 = ple(0x384);
    pub const PG_HIF_GROUP: u32 = ple(0x00c);
    pub const HIF_PG_INFO: u32 = ple(0x388);

    pub const fn ac_qempty(ac: u32, n: u32) -> u32 { ple(0x600 + 0x80 * ac + (n << 2)) }
    pub const fn amsdu_pack_msdu_cnt(n: u32) -> u32 { ple(0x10e0 + (n << 2)) }
}

// ============================================================================
// MDP - MAC DMA Processor
// ============================================================================

pub mod mdp {
    use super::bit;

    pub const BASE: u32 = 0x820cc000;
    pub const fn mdp(ofs: u32) -> u32 { BASE + ofs }

    pub const DCR2: u32 = mdp(0x8e8);
    pub const DCR2_RX_TRANS_SHORT: u32 = bit(2);
}

// ============================================================================
// UMIB - Unified MIB
// ============================================================================

pub mod umib {
    pub const BASE: u32 = 0x820cd000;
    pub const fn umib(ofs: u32) -> u32 { BASE + ofs }

    pub const fn rpdcr(band: u32) -> u32 { umib(0x594) + band * 0x164 }
}

// ============================================================================
// WTBLON TOP - WTBL Online
// ============================================================================

pub mod wtblon {
    use super::{bit, genmask};

    pub const BASE: u32 = 0x820d4000;
    pub const fn wtblon(ofs: u32) -> u32 { BASE + ofs }

    // Note: These use __OFFS() in Linux, meaning they're chip-variant dependent
    // For MT7996, the base offsets are documented in mt7996.h
    // WTBLON_WDUCR offset for MT7996 = 0x200
    pub const WDUCR: u32 = wtblon(0x200);
    pub const WDUCR_GROUP: u32 = genmask(4, 0);

    // WTBL_UPDATE offset for MT7996 = 0x230
    pub const UPDATE: u32 = wtblon(0x230);
    pub const UPDATE_WLAN_IDX: u32 = genmask(11, 0);
    pub const UPDATE_ADM_COUNT_CLEAR: u32 = bit(14);
    pub const UPDATE_BUSY: u32 = bit(31);

    // WTBL_ITCR offset for MT7996 = 0x3b0
    pub const ITCR: u32 = wtblon(0x3b0);
    pub const ITCR_WR: u32 = bit(16);
    pub const ITCR_EXEC: u32 = bit(31);
    pub const ITDR0: u32 = wtblon(0x3b8);
    pub const ITDR1: u32 = wtblon(0x3bc);
    pub const SPE_IDX_SEL: u32 = bit(6);
}

// ============================================================================
// WTBL - Wireless Table
// ============================================================================

pub mod wtbl {
    use super::genmask;

    pub const BASE: u32 = 0x820d8000;
    pub const LMAC_ID: u32 = genmask(14, 8);
    pub const LMAC_DW: u32 = genmask(7, 2);

    pub const fn lmac_offs(id: u32, dw: u32) -> u32 {
        BASE | ((id & 0x7F) << 8) | ((dw & 0x3F) << 2)
    }
}

// ============================================================================
// WFDMA0 - WiFi DMA Controller 0
// ============================================================================

pub mod wfdma0 {
    use super::{bit, genmask};

    pub const BASE: u32 = 0xd4000;
    pub const fn wfdma0(ofs: u32) -> u32 { BASE + ofs }

    // Reset control
    pub const RST: u32 = wfdma0(0x100);
    pub const RST_LOGIC_RST: u32 = bit(4);
    pub const RST_DMASHDL_ALL_RST: u32 = bit(5);

    // Busy enable
    pub const BUSY_ENA: u32 = wfdma0(0x13c);
    pub const BUSY_ENA_TX_FIFO0: u32 = bit(0);
    pub const BUSY_ENA_TX_FIFO1: u32 = bit(1);
    pub const BUSY_ENA_RX_FIFO: u32 = bit(2);

    // RX interrupt PCIe select
    pub const RX_INT_PCIE_SEL: u32 = wfdma0(0x154);
    pub const RX_INT_SEL_RING3: u32 = bit(3);
    pub const RX_INT_SEL_RING5: u32 = bit(5);
    pub const RX_INT_SEL_RING6: u32 = bit(6);
    pub const RX_INT_SEL_RING9: u32 = bit(9);

    // MCU host interrupt enable
    pub const MCU_HOST_INT_ENA: u32 = wfdma0(0x1f4);

    // Global config
    pub const GLO_CFG: u32 = wfdma0(0x208);
    pub const GLO_CFG_TX_DMA_EN: u32 = bit(0);
    pub const GLO_CFG_RX_DMA_EN: u32 = bit(2);
    pub const GLO_CFG_OMIT_RX_INFO_PFET2: u32 = bit(21);
    pub const GLO_CFG_EXT_EN: u32 = bit(26);
    pub const GLO_CFG_OMIT_RX_INFO: u32 = bit(27);
    pub const GLO_CFG_OMIT_TX_INFO: u32 = bit(28);

    // RX pause thresholds - NOTE: NOT SEQUENTIAL! Gap before RRO_TH
    pub const PAUSE_RX_Q_45_TH: u32 = wfdma0(0x268);
    pub const PAUSE_RX_Q_67_TH: u32 = wfdma0(0x26c);
    pub const PAUSE_RX_Q_89_TH: u32 = wfdma0(0x270);
    // 0x274, 0x278 reserved
    pub const PAUSE_RX_Q_RRO_TH: u32 = wfdma0(0x27c);

    // Global config extensions
    pub const GLO_CFG_EXT0: u32 = wfdma0(0x2b0);
    pub const GLO_CFG_EXT0_OUTSTAND_MASK: u32 = genmask(27, 24);
    pub const GLO_CFG_EXT0_RX_WB_RXD: u32 = bit(18);
    pub const GLO_CFG_EXT0_WED_MERGE_MODE: u32 = bit(14);

    pub const GLO_CFG_EXT1: u32 = wfdma0(0x2b4);
    pub const GLO_CFG_EXT1_CALC_MODE: u32 = bit(31);
    pub const GLO_CFG_EXT1_TX_FCTRL_MODE: u32 = bit(28);

    // Reset and delay
    pub const RST_DTX_PTR: u32 = wfdma0(0x20c);
    pub const PRI_DLY_INT_CFG0: u32 = wfdma0(0x2f0);
    pub const PRI_DLY_INT_CFG1: u32 = wfdma0(0x2f4);
    pub const PRI_DLY_INT_CFG2: u32 = wfdma0(0x2f8);

    // Interrupt registers
    pub const INT_SOURCE_CSR: u32 = wfdma0(0x200);
    pub const INT_MASK_CSR: u32 = wfdma0(0x204);

    // MCU command
    pub const MCU_CMD: u32 = wfdma0(0x1f0);
    pub const MCU_CMD_STOP_DMA: u32 = bit(2);
    pub const MCU_CMD_RESET_DONE: u32 = bit(3);
    pub const MCU_CMD_RECOVERY_DONE: u32 = bit(4);
    pub const MCU_CMD_NORMAL_STATE: u32 = bit(5);
    pub const MCU_CMD_ERROR_MASK: u32 = genmask(5, 1);
    pub const MCU_CMD_WA_WDT: u32 = bit(31);
    pub const MCU_CMD_WM_WDT: u32 = bit(30);
    pub const MCU_CMD_WDT_MASK: u32 = genmask(31, 30);

    // Ring bases (relative to WFDMA0_BASE)
    // TX rings at +0x300, RX rings at +0x500
    pub const TX_RING_BASE: u32 = 0x300;
    pub const RX_RING_BASE: u32 = 0x500;

    // Prefetch control bases
    pub const TX_EXT_CTRL_BASE: u32 = 0x600;
    pub const RX_EXT_CTRL_BASE: u32 = 0x680;
}

// ============================================================================
// WFDMA1 - WiFi DMA Controller 1
// ============================================================================

pub mod wfdma1 {
    pub const BASE: u32 = 0xd5000;
}

// ============================================================================
// WFDMA0 PCIE1 (HIF2) - Second PCIe interface
// ============================================================================

pub mod wfdma0_pcie1 {
    use super::bit;

    pub const BASE: u32 = 0xd8000;
    pub const fn pcie1(ofs: u32) -> u32 { BASE + ofs }

    /// HIF2 offset when accessed via HIF1's BAR
    /// Linux uses: MT_WFDMA0_PCIE1(0) - MT_WFDMA0(0) = 0xd8000 - 0xd4000 = 0x4000
    pub const HIF1_OFS: u32 = BASE - super::wfdma0::BASE;  // 0x4000

    pub const INT_SOURCE_CSR_EXT: u32 = pcie1(0x118);
    pub const INT_MASK_CSR: u32 = pcie1(0x11c);

    pub const BUSY_ENA: u32 = pcie1(0x13c);
    pub const BUSY_ENA_TX_FIFO0: u32 = bit(0);
    pub const BUSY_ENA_TX_FIFO1: u32 = bit(1);
    pub const BUSY_ENA_RX_FIFO: u32 = bit(2);

    pub const INT1_SOURCE_CSR: u32 = pcie1(0x200);
    pub const INT1_MASK_CSR: u32 = pcie1(0x204);
}

// ============================================================================
// WFDMA Extended CSR
// ============================================================================

pub mod wfdma_ext_csr {
    use super::{bit, genmask};

    pub const BASE: u32 = 0xd7000;
    pub const fn ext_csr(ofs: u32) -> u32 { BASE + ofs }

    pub const HOST_CONFIG: u32 = ext_csr(0x30);
    pub const HOST_CONFIG_PDMA_BAND: u32 = bit(0);
    pub const HOST_CONFIG_BAND0_PCIE1: u32 = bit(20);
    pub const HOST_CONFIG_BAND1_PCIE1: u32 = bit(21);
    pub const HOST_CONFIG_BAND2_PCIE1: u32 = bit(22);

    pub const HIF_MISC: u32 = ext_csr(0x44);
    pub const HIF_MISC_BUSY: u32 = bit(0);

    pub const AXI_R2A_CTRL: u32 = ext_csr(0x500);
    pub const AXI_R2A_CTRL_OUTSTAND_MASK: u32 = genmask(4, 0);

    pub const AXI_R2A_CTRL2: u32 = ext_csr(0x508);
    pub const AXI_R2A_CTRL2_OUTSTAND_MASK: u32 = genmask(31, 28);
}

// ============================================================================
// PCIe Recognition ID
// ============================================================================

pub mod pcie {
    use super::{bit, genmask};

    pub const RECOG_ID: u32 = 0xd7090;
    pub const RECOG_ID_MASK: u32 = genmask(30, 0);
    pub const RECOG_ID_SEM: u32 = bit(31);
}

// ============================================================================
// Queue Index System - Mirrors Linux __RXQ(), __TXQ(), etc.
// ============================================================================

pub mod queue {
    //! Queue index calculation system.
    //!
    //! Linux uses three namespaces:
    //! - MCU queues: 0..MCUQ_MAX (0, 1, 2)
    //! - RX queues: MCUQ_MAX..(MCUQ_MAX + RXQ_MAX)
    //! - TX queues: (MCUQ_MAX + RXQ_MAX)..(MCUQ_MAX + RXQ_MAX + TXQ_MAX)
    //!
    //! The q_id[] array maps logical indices to hardware ring indices.

    // MCU queue logical indices
    pub mod mcuq {
        pub const WM: usize = 0;
        pub const WA: usize = 1;
        pub const FWDL: usize = 2;
        pub const MAX: usize = 3;
    }

    // RX queue logical indices (after transformation via __RXQ)
    pub mod rxq {
        pub const MAIN: usize = 0;
        pub const MCU: usize = 1;
        pub const MCU_WA: usize = 2;
        pub const MAIN_WA: usize = 3;
        pub const TXFREE_BAND0: usize = 4;  // MT_RXQ_BAND1_WA in 7996
        pub const BAND1: usize = 5;
        pub const BAND1_WA: usize = 6;
        pub const BAND2: usize = 7;
        pub const BAND2_WA: usize = 8;
        pub const RRO_BAND0: usize = 9;
        pub const RRO_BAND1: usize = 10;
        pub const RRO_BAND2: usize = 11;
        pub const RRO_IND: usize = 12;
        pub const RRO_RXDMAD_C: usize = 12;  // Same as RRO_IND
        pub const MSDU_PAGE_BAND0: usize = 13;
        pub const MSDU_PAGE_BAND1: usize = 14;
        pub const MSDU_PAGE_BAND2: usize = 15;
        pub const TXFREE_BAND1: usize = 16;
        pub const TXFREE_BAND2: usize = 17;
        pub const TXFREE: usize = 18;
        pub const CHANGE_BAND0: usize = 19;
        pub const CHANGE_BAND1: usize = 20;
        pub const MAX: usize = 21;
    }

    // Data TX queue logical indices (after transformation via __TXQ)
    pub mod txq {
        pub const BAND0: usize = 0;
        pub const BAND1: usize = 1;
        pub const BAND2: usize = 2;
        pub const MAX: usize = 3;
    }

    // MT7996 hardware ring indices (from q_id[] array in mt7996.h)
    pub mod mt7996_hw {
        // MCU TX rings
        pub const FWDL: u32 = 16;
        pub const MCU_WM: u32 = 17;

        // Data TX rings
        pub const BAND0: u32 = 18;
        pub const BAND1: u32 = 19;

        // MCU TX ring (continued)
        pub const MCU_WA: u32 = 20;

        // Data TX ring (continued)
        pub const BAND2: u32 = 21;

        // RX rings
        pub const RX_MCU_WM: u32 = 0;
        pub const RX_MCU_WA: u32 = 1;
        pub const RX_MCU_WA_MAIN: u32 = 2;
        pub const RX_DATA_BAND0: u32 = 4;
        pub const RX_DATA_BAND1: u32 = 5;
        pub const RX_DATA_BAND2: u32 = 6;
        pub const RX_TXFREE0: u32 = 3;
        pub const RX_TXFREE1: u32 = 7;
    }

    /// Convert logical MCU queue index to hardware index
    /// Linux: MT_MCUQ_ID(q) = dev->q_id[q]
    pub const fn mcuq_id(q: usize) -> u32 {
        match q {
            mcuq::WM => mt7996_hw::MCU_WM,
            mcuq::WA => mt7996_hw::MCU_WA,
            mcuq::FWDL => mt7996_hw::FWDL,
            _ => 0,
        }
    }

    /// Convert logical TX queue index to hardware index
    /// Linux: MT_TXQ_ID(q) = dev->q_id[__TXQ(q)]
    pub const fn txq_id(q: usize) -> u32 {
        match q {
            txq::BAND0 => mt7996_hw::BAND0,
            txq::BAND1 => mt7996_hw::BAND1,
            txq::BAND2 => mt7996_hw::BAND2,
            _ => 0,
        }
    }

    /// Convert logical RX queue index to hardware index
    /// Linux: MT_RXQ_ID(q) = dev->q_id[__RXQ(q)]
    pub const fn rxq_id(q: usize) -> u32 {
        match q {
            rxq::MAIN => mt7996_hw::RX_DATA_BAND0,
            rxq::MCU => mt7996_hw::RX_MCU_WM,
            rxq::MCU_WA => mt7996_hw::RX_MCU_WA,
            rxq::MAIN_WA => mt7996_hw::RX_MCU_WA_MAIN,
            rxq::TXFREE_BAND0 => mt7996_hw::RX_TXFREE0,
            rxq::BAND1 => mt7996_hw::RX_DATA_BAND1,
            rxq::BAND2 => mt7996_hw::RX_DATA_BAND2,
            rxq::TXFREE_BAND1 => mt7996_hw::RX_TXFREE1,
            _ => 0,
        }
    }

    /// MCU queue prefetch control register
    /// Linux: MT_MCUQ_EXT_CTRL(q) = MT_Q_BASE(q) + 0x600 + MT_MCUQ_ID(q) * 0x4
    pub const fn mcuq_ext_ctrl(q: usize) -> u32 {
        super::wfdma0::BASE + 0x600 + mcuq_id(q) * 4
    }

    /// TX queue prefetch control register
    /// Linux: MT_TXQ_EXT_CTRL(q) = MT_Q_BASE(__TXQ(q)) + 0x600 + MT_TXQ_ID(q) * 0x4
    pub const fn txq_ext_ctrl(q: usize) -> u32 {
        super::wfdma0::BASE + 0x600 + txq_id(q) * 4
    }

    /// RX queue prefetch control register
    /// Linux: MT_RXQ_EXT_CTRL(q) = MT_Q_BASE(__RXQ(q)) + 0x680 + MT_RXQ_ID(q) * 0x4
    pub const fn rxq_ext_ctrl(q: usize) -> u32 {
        super::wfdma0::BASE + 0x680 + rxq_id(q) * 4
    }

    /// MCU queue ring base address
    /// Linux: MT_MCUQ_RING_BASE(q) = MT_Q_BASE(q) + 0x300
    pub const fn mcuq_ring_base(q: usize) -> u32 {
        super::wfdma0::BASE + 0x300 + mcuq_id(q) * 0x10
    }

    /// TX queue ring base address
    /// Linux: MT_TXQ_RING_BASE(q) = MT_Q_BASE(__TXQ(q)) + 0x300
    pub const fn txq_ring_base(q: usize) -> u32 {
        super::wfdma0::BASE + 0x300 + txq_id(q) * 0x10
    }

    /// RX queue ring base address
    /// Linux: MT_RXQ_RING_BASE(q) = MT_Q_BASE(__RXQ(q)) + 0x500
    pub const fn rxq_ring_base(q: usize) -> u32 {
        super::wfdma0::BASE + 0x500 + rxq_id(q) * 0x10
    }

    // Ring register offsets (within a ring's 0x10 byte block)
    pub const RING_BASE: u32 = 0x00;
    pub const RING_CNT: u32 = 0x04;
    pub const RING_CPU_IDX: u32 = 0x08;
    pub const RING_DMA_IDX: u32 = 0x0c;
}

// ============================================================================
// Interrupt Bits
// ============================================================================

pub mod int {
    use super::bit;

    // RX done interrupts
    pub const RX_DONE_BAND0: u32 = bit(12);
    pub const RX_DONE_BAND1: u32 = bit(13);  // mt7992
    pub const RX_DONE_BAND2: u32 = bit(13);
    pub const RX_DONE_WM: u32 = bit(0);
    pub const RX_DONE_WA: u32 = bit(1);
    pub const RX_DONE_WA_MAIN: u32 = bit(2);
    pub const RX_DONE_WA_EXT: u32 = bit(3);   // mt7992
    pub const RX_DONE_WA_TRI: u32 = bit(3);

    // TX free interrupts
    pub const RX_TXFREE_MAIN: u32 = bit(17);
    pub const RX_TXFREE_BAND1: u32 = bit(15);
    pub const RX_TXFREE_TRI: u32 = bit(15);
    pub const RX_TXFREE_BAND1_EXT: u32 = bit(19);  // mt7992 two PCIE
    pub const RX_TXFREE_BAND0_MT7990: u32 = bit(14);
    pub const RX_TXFREE_BAND1_MT7990: u32 = bit(15);
    pub const RX_DONE_BAND2_EXT: u32 = bit(23);
    pub const RX_TXFREE_EXT: u32 = bit(26);

    // MCU command interrupt
    pub const MCU_CMD: u32 = bit(29);

    // RRO interrupts
    pub const RX_DONE_RRO_BAND0: u32 = bit(16);
    pub const RX_DONE_RRO_BAND1: u32 = bit(17);
    pub const RX_DONE_RRO_BAND2: u32 = bit(14);
    pub const RX_DONE_RRO_IND: u32 = bit(11);
    pub const RX_DONE_RRO_RXDMAD_C: u32 = bit(11);
    pub const RX_DONE_MSDU_PG_BAND0: u32 = bit(18);
    pub const RX_DONE_MSDU_PG_BAND1: u32 = bit(19);
    pub const RX_DONE_MSDU_PG_BAND2: u32 = bit(23);

    // TX done interrupts
    pub const TX_DONE_FWDL: u32 = bit(26);
    pub const TX_DONE_MCU_WM: u32 = bit(27);
    pub const TX_DONE_MCU_WA: u32 = bit(22);
    pub const TX_DONE_BAND0: u32 = bit(30);
    pub const TX_DONE_BAND1: u32 = bit(31);
    pub const TX_DONE_BAND2: u32 = bit(15);
}

// ============================================================================
// MT TOP - Top-level control
// ============================================================================

pub mod top {
    use super::{bit, genmask};

    pub const BASE: u32 = 0xe0000;
    pub const fn top(ofs: u32) -> u32 { BASE + ofs }

    pub const fn lpcr_host_band(band: u32) -> u32 { top(0x10 + band * 0x10) }
    pub const LPCR_HOST_FW_OWN: u32 = bit(0);
    pub const LPCR_HOST_DRV_OWN: u32 = bit(1);
    pub const LPCR_HOST_FW_OWN_STAT: u32 = bit(2);

    pub const fn lpcr_host_band_irq_stat(band: u32) -> u32 { top(0x14 + band * 0x10) }
    pub const LPCR_HOST_BAND_STAT: u32 = bit(0);

    pub const MISC: u32 = top(0xf0);
    pub const MISC_FW_STATE: u32 = genmask(2, 0);
}

// ============================================================================
// HIF Remap
// ============================================================================

pub mod hif_remap {
    use super::genmask;

    pub const CONN_BUS_CR_VON_BASE: u32 = 0x155000;

    // MT7996 specific offsets (from mt7996.h offs array)
    pub const L1_OFFSET: u32 = 0x24;
    pub const BASE_L1_OFFSET: u32 = 0x28;
    pub const L2_OFFSET: u32 = 0x1b4;
    pub const BASE_L2_OFFSET: u32 = 0x1b8;

    pub const L1: u32 = CONN_BUS_CR_VON_BASE + L1_OFFSET;
    pub const L1_MASK_7996: u32 = genmask(31, 16);
    pub const L1_MASK: u32 = genmask(15, 0);
    pub const L1_OFFSET_MASK: u32 = genmask(15, 0);
    pub const L1_BASE: u32 = genmask(31, 16);
    pub const BASE_L1: u32 = BASE_L1_OFFSET;

    pub const L2_MASK: u32 = genmask(19, 0);
    pub const L2_OFFSET_MASK: u32 = genmask(11, 0);
    pub const L2_BASE: u32 = genmask(31, 12);
}

// ============================================================================
// Memory regions
// ============================================================================

pub mod mem {
    pub const INFRA_BASE: u32 = 0x18000000;
    pub const WFSYS0_PHY_START: u32 = 0x18400000;
    pub const WFSYS1_PHY_START: u32 = 0x18800000;
    pub const WFSYS1_PHY_END: u32 = 0x18bfffff;
    pub const CBTOP1_PHY_START: u32 = 0x70000000;
    pub const CBTOP2_PHY_START: u32 = 0xf0000000;
    pub const INFRA_MCU_START: u32 = 0x7c000000;

    // MT7996 specific
    pub const CBTOP1_PHY_END: u32 = 0x77ffffff;
    pub const INFRA_MCU_END: u32 = 0x7c3fffff;
}

// ============================================================================
// Firmware state
// ============================================================================

pub mod fw {
    pub const ASSERT_CNT: u32 = 0x02208274;
    pub const DUMP_STATE: u32 = 0x02209e90;

    pub const SWDEF_BASE: u32 = 0x00401400;
    pub const fn swdef(ofs: u32) -> u32 { SWDEF_BASE + ofs }

    pub const SWDEF_MODE: u32 = swdef(0x3c);
    pub const SWDEF_NORMAL_MODE: u32 = 0;

    pub const SWDEF_SER_STATS: u32 = swdef(0x040);
    pub const SWDEF_PLE_STATS: u32 = swdef(0x044);
    pub const SWDEF_PLE1_STATS: u32 = swdef(0x048);
    pub const SWDEF_PLE_AMSDU_STATS: u32 = swdef(0x04c);
    pub const SWDEF_PSE_STATS: u32 = swdef(0x050);
    pub const SWDEF_PSE1_STATS: u32 = swdef(0x054);
}

// ============================================================================
// LED
// ============================================================================

pub mod led {
    use super::{bit, genmask};

    pub const TOP_BASE: u32 = 0x18013000;
    pub const fn led_phys(n: u32) -> u32 { TOP_BASE + n }

    pub const fn ctrl(n: u32) -> u32 { led_phys(0x00 + n * 4) }
    pub const CTRL_KICK: u32 = bit(7);
    pub const CTRL_BLINK_BAND_SEL: u32 = bit(4);
    pub const CTRL_BLINK_MODE: u32 = bit(2);
    pub const CTRL_POLARITY: u32 = bit(1);

    pub const fn tx_blink(n: u32) -> u32 { led_phys(0x10 + n * 4) }
    pub const TX_BLINK_ON_MASK: u32 = genmask(7, 0);
    pub const TX_BLINK_OFF_MASK: u32 = genmask(15, 8);

    pub const fn en(n: u32) -> u32 { led_phys(0x40 + n * 4) }

    pub const GPIO_MUX2: u32 = 0x70005058;  // GPIO 18
    pub const GPIO_MUX3: u32 = 0x7000505C;  // GPIO 26
    pub const GPIO_SEL_MASK: u32 = genmask(11, 8);
}

// ============================================================================
// PCIe MAC
// ============================================================================

pub mod pcie_mac {
    pub const BASE: u32 = 0x74030000;
    pub const fn pcie_mac(ofs: u32) -> u32 { BASE + ofs }

    pub const INT_ENABLE: u32 = pcie_mac(0x188);

    pub const BASE1: u32 = 0x74090000;
    pub const fn pcie1_mac(ofs: u32) -> u32 { BASE1 + ofs }

    pub const INT_ENABLE1: u32 = pcie1_mac(0x188);
}

// ============================================================================
// Hardware revision
// ============================================================================

pub mod hw {
    use super::genmask;

    pub const REV: u32 = 0x70010204;
    pub const REV1: u32 = 0x8a00;
    pub const CHIPID: u32 = 0x70010200;

    pub const WF_SUBSYS_RST: u32 = 0x70028600;

    pub const ADIE_CHIP_ID_0: u32 = 0x0f00002c;
    pub const ADIE_CHIP_ID_1: u32 = 0x1f00002c;
    pub const ADIE_VERSION_MASK: u32 = genmask(15, 0);
    pub const ADIE_CHIP_ID_MASK: u32 = genmask(31, 16);

    pub const PAD_GPIO: u32 = 0x700056f0;
    pub const PAD_GPIO_ADIE_COMB: u32 = genmask(16, 15);
    pub const PAD_GPIO_2ADIE_TBTC: u32 = 1 << 19;
}

// ============================================================================
// Legacy aliases (for compatibility with existing code)
// ============================================================================

// These match the old regs.rs naming for backward compatibility
pub const MT_HW_REV: u32 = hw::REV;
pub const MT_HW_CHIPID: u32 = hw::CHIPID;
pub const MT_PAD_GPIO: u32 = hw::PAD_GPIO;
pub const MT_TOP_CFG: u32 = 0x70020000;
pub const MT_WFDMA0_BASE: u32 = wfdma0::BASE;
pub const MT_HIF1_OFS: u32 = wfdma0_pcie1::HIF1_OFS;
pub const MT_WFDMA0_BUSY_ENA: u32 = wfdma0::BUSY_ENA;
pub const MT_PCIE_MAC_INT_ENABLE: u32 = pcie_mac::INT_ENABLE;

// Remap constants (from original regs.rs)
pub const MT_INFRA_REMAP_L1_BASE: u32 = 0x700d02f8;
pub const MT_INFRA_REMAP_L2_BASE: u32 = 0x700d0300;
pub const MT_INFRA_REMAP_CBTOP1_BASE: u32 = 0x0228;
pub const MT_INFRA_REMAP_CBTOP2_BASE: u32 = 0x022c;
pub const MT_REMAP_L1_MASK: u32 = 0x7fff_ffff;
pub const MT_REMAP_L1_OFFSET: u32 = 18;
pub const MT_REMAP_L1_BASE_MASK: u32 = 0x3ffff;

// ============================================================================
// Self-test: Verify key register addresses match Linux
// ============================================================================

/// Validate that our register addresses match Linux regs.h
/// Call this at driver init to catch any translation errors
pub fn validate_registers() {
    // WFDMA0 addresses
    debug_assert_eq!(wfdma0::BASE, 0xd4000, "WFDMA0_BASE mismatch");
    debug_assert_eq!(wfdma0::RST, 0xd4100, "WFDMA0_RST mismatch");
    debug_assert_eq!(wfdma0::GLO_CFG, 0xd4208, "WFDMA0_GLO_CFG mismatch");
    debug_assert_eq!(wfdma0::RST_DTX_PTR, 0xd420c, "WFDMA0_RST_DTX_PTR mismatch");
    debug_assert_eq!(wfdma0::PAUSE_RX_Q_45_TH, 0xd4268, "PAUSE_RX_Q_45_TH mismatch");
    debug_assert_eq!(wfdma0::PAUSE_RX_Q_67_TH, 0xd426c, "PAUSE_RX_Q_67_TH mismatch");
    debug_assert_eq!(wfdma0::PAUSE_RX_Q_89_TH, 0xd4270, "PAUSE_RX_Q_89_TH mismatch");
    debug_assert_eq!(wfdma0::PAUSE_RX_Q_RRO_TH, 0xd427c, "PAUSE_RX_Q_RRO_TH mismatch (NOT 0x274!)");
    debug_assert_eq!(wfdma0::GLO_CFG_EXT0, 0xd42b0, "GLO_CFG_EXT0 mismatch");
    debug_assert_eq!(wfdma0::GLO_CFG_EXT1, 0xd42b4, "GLO_CFG_EXT1 mismatch");
    debug_assert_eq!(wfdma0::INT_SOURCE_CSR, 0xd4200, "INT_SOURCE_CSR mismatch");
    debug_assert_eq!(wfdma0::INT_MASK_CSR, 0xd4204, "INT_MASK_CSR mismatch");

    // WFDMA0 PCIE1 (HIF2)
    debug_assert_eq!(wfdma0_pcie1::BASE, 0xd8000, "WFDMA0_PCIE1_BASE mismatch");
    debug_assert_eq!(wfdma0_pcie1::HIF1_OFS, 0x4000, "HIF1_OFS mismatch");

    // Extended CSR
    debug_assert_eq!(wfdma_ext_csr::BASE, 0xd7000, "WFDMA_EXT_CSR_BASE mismatch");
    debug_assert_eq!(wfdma_ext_csr::HOST_CONFIG, 0xd7030, "HOST_CONFIG mismatch");
    debug_assert_eq!(wfdma_ext_csr::HIF_MISC, 0xd7044, "HIF_MISC mismatch");

    // Queue prefetch control addresses
    // FWDL = hw idx 16, so 0xd4000 + 0x600 + 16*4 = 0xd4640
    debug_assert_eq!(queue::mcuq_ext_ctrl(queue::mcuq::FWDL), 0xd4640, "FWDL ext_ctrl mismatch");
    // WM = hw idx 17
    debug_assert_eq!(queue::mcuq_ext_ctrl(queue::mcuq::WM), 0xd4644, "WM ext_ctrl mismatch");
    // WA = hw idx 20
    debug_assert_eq!(queue::mcuq_ext_ctrl(queue::mcuq::WA), 0xd4650, "WA ext_ctrl mismatch");
    // BAND0 = hw idx 18
    debug_assert_eq!(queue::txq_ext_ctrl(queue::txq::BAND0), 0xd4648, "BAND0 ext_ctrl mismatch");

    // Ring bases
    // FWDL ring: 0xd4000 + 0x300 + 16*0x10 = 0xd4400
    debug_assert_eq!(queue::mcuq_ring_base(queue::mcuq::FWDL), 0xd4400, "FWDL ring_base mismatch");
}

/// Print all register addresses for debugging
#[allow(dead_code)]
pub fn dump_register_map() {
    userlib::println!("=== MT7996 Register Map ===");
    userlib::println!("WFDMA0_BASE:        0x{:05x}", wfdma0::BASE);
    userlib::println!("WFDMA0_RST:         0x{:05x}", wfdma0::RST);
    userlib::println!("WFDMA0_GLO_CFG:     0x{:05x}", wfdma0::GLO_CFG);
    userlib::println!("WFDMA0_RST_DTX_PTR: 0x{:05x}", wfdma0::RST_DTX_PTR);
    userlib::println!("PAUSE_RX_Q_45_TH:   0x{:05x}", wfdma0::PAUSE_RX_Q_45_TH);
    userlib::println!("PAUSE_RX_Q_67_TH:   0x{:05x}", wfdma0::PAUSE_RX_Q_67_TH);
    userlib::println!("PAUSE_RX_Q_89_TH:   0x{:05x}", wfdma0::PAUSE_RX_Q_89_TH);
    userlib::println!("PAUSE_RX_Q_RRO_TH:  0x{:05x}", wfdma0::PAUSE_RX_Q_RRO_TH);
    userlib::println!("GLO_CFG_EXT0:       0x{:05x}", wfdma0::GLO_CFG_EXT0);
    userlib::println!("GLO_CFG_EXT1:       0x{:05x}", wfdma0::GLO_CFG_EXT1);
    userlib::println!("");
    userlib::println!("WFDMA0_PCIE1_BASE:  0x{:05x}", wfdma0_pcie1::BASE);
    userlib::println!("HIF1_OFS:           0x{:05x}", wfdma0_pcie1::HIF1_OFS);
    userlib::println!("");
    userlib::println!("EXT_CSR_BASE:       0x{:05x}", wfdma_ext_csr::BASE);
    userlib::println!("EXT_CSR_HIF_MISC:   0x{:05x}", wfdma_ext_csr::HIF_MISC);
    userlib::println!("");
    userlib::println!("=== Queue Prefetch Control ===");
    userlib::println!("FWDL ext_ctrl:      0x{:05x}", queue::mcuq_ext_ctrl(queue::mcuq::FWDL));
    userlib::println!("WM ext_ctrl:        0x{:05x}", queue::mcuq_ext_ctrl(queue::mcuq::WM));
    userlib::println!("WA ext_ctrl:        0x{:05x}", queue::mcuq_ext_ctrl(queue::mcuq::WA));
    userlib::println!("BAND0 ext_ctrl:     0x{:05x}", queue::txq_ext_ctrl(queue::txq::BAND0));
    userlib::println!("BAND1 ext_ctrl:     0x{:05x}", queue::txq_ext_ctrl(queue::txq::BAND1));
    userlib::println!("BAND2 ext_ctrl:     0x{:05x}", queue::txq_ext_ctrl(queue::txq::BAND2));
    userlib::println!("");
    userlib::println!("=== Ring Bases ===");
    userlib::println!("FWDL ring_base:     0x{:05x}", queue::mcuq_ring_base(queue::mcuq::FWDL));
    userlib::println!("WM ring_base:       0x{:05x}", queue::mcuq_ring_base(queue::mcuq::WM));
    userlib::println!("BAND0 ring_base:    0x{:05x}", queue::txq_ring_base(queue::txq::BAND0));
}
