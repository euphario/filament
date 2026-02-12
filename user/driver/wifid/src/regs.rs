//! MT7996 Register Definitions
//!
//! EXACT register definitions from Linux regs.h, mmio.c, dma.h.
//! All constants are direct translations from the Linux mt76/mt7996 driver.

// ============================================================================
// WFDMA0 Base
// ============================================================================

pub const MT_WFDMA0_BASE: u32 = 0xd4000;
pub const MT_WFDMA0_TX_RING_BASE: u32 = MT_WFDMA0_BASE + 0x300;
pub const MT_WFDMA0_RX_RING_BASE: u32 = MT_WFDMA0_BASE + 0x500;

#[inline]
pub const fn mt_wfdma0(ofs: u32) -> u32 { MT_WFDMA0_BASE + ofs }

// WFDMA0 Core Registers
pub const MT_WFDMA0_RST: u32 = mt_wfdma0(0x100);
pub const MT_WFDMA0_RST_LOGIC_RST: u32 = 1 << 4;
pub const MT_WFDMA0_RST_DMASHDL_ALL_RST: u32 = 1 << 5;

pub const MT_WFDMA0_BUSY_ENA: u32 = mt_wfdma0(0x13c);
pub const MT_WFDMA0_BUSY_ENA_TX_FIFO0: u32 = 1 << 0;
pub const MT_WFDMA0_BUSY_ENA_TX_FIFO1: u32 = 1 << 1;
pub const MT_WFDMA0_BUSY_ENA_RX_FIFO: u32 = 1 << 2;

pub const MT_WFDMA0_RX_INT_PCIE_SEL: u32 = mt_wfdma0(0x154);
pub const MT_WFDMA0_RX_INT_SEL_RING3: u32 = 1 << 3;

pub const MCU_WM_RX_REGS: u32 = MT_WFDMA0_BASE + 0x500;  // RX queue 0
pub const MCU_WA_RX_REGS: u32 = MT_WFDMA0_BASE + 0x500 + MT_RING_SIZE;  // RX queue 1

pub const MT_INT_SOURCE_CSR: u32 = mt_wfdma0(0x200);
pub const MT_INT_MASK_CSR: u32 = mt_wfdma0(0x204);

pub const MT_WFDMA0_GLO_CFG: u32 = mt_wfdma0(0x208);
pub const MT_WFDMA0_GLO_CFG_TX_DMA_EN: u32 = 1 << 0;
pub const MT_WFDMA0_GLO_CFG_RX_DMA_EN: u32 = 1 << 2;
pub const MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2: u32 = 1 << 21;
pub const MT_WFDMA0_GLO_CFG_EXT_EN: u32 = 1 << 26;
pub const MT_WFDMA0_GLO_CFG_OMIT_RX_INFO: u32 = 1 << 27;
pub const MT_WFDMA0_GLO_CFG_OMIT_TX_INFO: u32 = 1 << 28;

pub const MT_WFDMA0_RST_DTX_PTR: u32 = mt_wfdma0(0x20c);

pub const MT_WFDMA0_PAUSE_RX_Q_45_TH: u32 = mt_wfdma0(0x268);
pub const MT_WFDMA0_PAUSE_RX_Q_67_TH: u32 = mt_wfdma0(0x26c);
pub const MT_WFDMA0_PAUSE_RX_Q_89_TH: u32 = mt_wfdma0(0x270);
pub const MT_WFDMA0_PAUSE_RX_Q_RRO_TH: u32 = mt_wfdma0(0x27c);

pub const WF_WFDMA0_GLO_CFG_EXT0: u32 = mt_wfdma0(0x2b0);
pub const WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD: u32 = 1 << 18;
pub const WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE: u32 = 1 << 14;

pub const WF_WFDMA0_GLO_CFG_EXT1: u32 = mt_wfdma0(0x2b4);
pub const WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE: u32 = 1 << 28;
pub const WF_WFDMA0_GLO_CFG_EXT1_CALC_MODE: u32 = 1u32 << 31;

pub const MT_WFDMA0_PRI_DLY_INT_CFG0: u32 = mt_wfdma0(0x2f0);
pub const MT_WFDMA0_PRI_DLY_INT_CFG1: u32 = mt_wfdma0(0x2f4);
pub const MT_WFDMA0_PRI_DLY_INT_CFG2: u32 = mt_wfdma0(0x2f8);

// ============================================================================
// WFDMA Extended CSR
// ============================================================================

const MT_WFDMA_EXT_CSR_BASE: u32 = 0xd7000;
#[inline]
const fn mt_wfdma_ext_csr(ofs: u32) -> u32 { MT_WFDMA_EXT_CSR_BASE + ofs }

pub const MT_WFDMA_HOST_CONFIG: u32 = mt_wfdma_ext_csr(0x30);
pub const MT_WFDMA_HOST_CONFIG_PDMA_BAND: u32 = 1 << 0;
pub const MT_WFDMA_HOST_CONFIG_BAND0_PCIE1: u32 = 1 << 20;
pub const MT_WFDMA_HOST_CONFIG_BAND1_PCIE1: u32 = 1 << 21;
pub const MT_WFDMA_HOST_CONFIG_BAND2_PCIE1: u32 = 1 << 22;

pub const MT_WFDMA_EXT_CSR_HIF_MISC: u32 = mt_wfdma_ext_csr(0x44);
pub const MT_WFDMA_EXT_CSR_HIF_MISC_BUSY: u32 = 1 << 0;

pub const MT_WFDMA_AXI_R2A_CTRL: u32 = mt_wfdma_ext_csr(0x500);
pub const MT_WFDMA_AXI_R2A_CTRL_OUTSTAND_MASK: u32 = 0x1f;

// ============================================================================
// WFDMA0 PCIE1 (HIF2 offset)
// ============================================================================

pub const MT_WFDMA0_PCIE1_BASE: u32 = 0xd8000;
pub const HIF1_OFS: u32 = MT_WFDMA0_PCIE1_BASE - MT_WFDMA0_BASE;

pub const MT_WFDMA0_PCIE1_BUSY_ENA_TX_FIFO0: u32 = 1 << 0;
pub const MT_WFDMA0_PCIE1_BUSY_ENA_TX_FIFO1: u32 = 1 << 1;
pub const MT_WFDMA0_PCIE1_BUSY_ENA_RX_FIFO: u32 = 1 << 2;

// ============================================================================
// MT_TOP registers (driver ownership)
// ============================================================================

const MT_TOP_BASE: u32 = 0xe0000;
#[inline]
const fn mt_top(ofs: u32) -> u32 { MT_TOP_BASE + ofs }

#[inline]
pub const fn mt_top_lpcr_host_band(band: u32) -> u32 { mt_top(0x10 + band * 0x10) }
#[inline]
pub const fn mt_top_lpcr_host_band_irq_stat(band: u32) -> u32 { mt_top(0x14 + band * 0x10) }

pub const MT_TOP_LPCR_HOST_DRV_OWN: u32 = 1 << 1;
pub const MT_TOP_LPCR_HOST_FW_OWN_STAT: u32 = 1 << 2;
pub const MT_TOP_LPCR_HOST_BAND_STAT: u32 = 1 << 0;

pub const MT_TOP_MISC: u32 = mt_top(0xf0);
pub const MT_TOP_MISC_FW_STATE: u32 = 0x7;

// ============================================================================
// Address Remapping
// Source: mmio.c - mt7996_reg_map_l1()
// ============================================================================

pub const CONN_BUS_CR_VON_BASE: u32 = 0x155000;
pub const HIF_REMAP_L1_OFFSET: u32 = 0x24;
pub const HIF_REMAP_BASE_L1: u32 = 0x130000;
pub const MT_HIF_REMAP_L1: u32 = CONN_BUS_CR_VON_BASE + HIF_REMAP_L1_OFFSET;

// WFSYS reset register (accessed via L1 remap)
pub const MT_WF_SUBSYS_RST: u32 = 0x70028600;

// Hardware revision register (accessed via L1 remap)
pub const MT_HW_REV: u32 = 0x70010204;

// PCIe MAC registers
pub const MT_PCIE_MAC_INT_ENABLE: u32 = 0x10188;
pub const MT_PCIE1_MAC_INT_ENABLE_PHYS: u32 = 0x74090188;

pub const MT_PCIE_RECOG_ID: u32 = 0xd7090;
pub const MT_PCIE_RECOG_ID_SEM: u32 = 1 << 31;

// SWDEF registers
pub const MT_SWDEF_MODE: u32 = 0x8143C;
pub const MT_SWDEF_NORMAL_MODE: u32 = 0;

// ============================================================================
// Queue register offsets (from mt76.h)
// ============================================================================

pub const MT_RING_SIZE: u32 = 0x10;
pub const MT_QUEUE_DESC_BASE: u32 = 0x0;
pub const MT_QUEUE_RING_SIZE: u32 = 0x4;
pub const MT_QUEUE_CPU_IDX: u32 = 0x8;
pub const MT_QUEUE_DMA_IDX: u32 = 0xc;

// ============================================================================
// DMA control bits (from dma.h)
// ============================================================================

pub const MT_DMA_CTL_SD_LEN0: u32 = 0x3fff_0000;
pub const MT_DMA_CTL_LAST_SEC0: u32 = 1 << 30;
pub const MT_DMA_CTL_DMA_DONE: u32 = 1u32 << 31;
pub const MT_DMA_CTL_SDP0_H: u32 = 0xF;

// ============================================================================
// Ring sizes from Linux mt7996.h
// ============================================================================

pub const MT7996_TX_RING_SIZE: u32 = 2048;
pub const MT7996_TX_MCU_RING_SIZE: u32 = 256;
pub const MT7996_TX_FWDL_RING_SIZE: u32 = 128;
pub const MT7996_RX_RING_SIZE: u32 = 1536;
pub const MT7996_RX_MCU_RING_SIZE: u32 = 512;
pub const MT7996_RX_MCU_RING_SIZE_WA: u32 = 1024;
pub const MT7996_RX_BUF_SIZE: u32 = 2048;
pub const MT7996_RX_MCU_BUF_SIZE: u32 = 2048;

pub const NUM_RX_QUEUES: usize = 7;

// TX queue indices (from mt7996.h enum mt7996_txq_id)
pub const MT7996_TXQ_FWDL: u32 = 16;
pub const MT7996_TXQ_MCU_WM: u32 = 17;
pub const MT7996_TXQ_BAND0: u32 = 18;
pub const MT7996_TXQ_BAND1: u32 = 19;
pub const MT7996_TXQ_MCU_WA: u32 = 20;
pub const MT7996_TXQ_BAND2: u32 = 21;

// RX queue indices (from mt7996.h enum mt7996_rxq_id)
pub const MT7996_RXQ_MCU_WM: u32 = 0;
pub const MT7996_RXQ_MCU_WA: u32 = 1;
pub const MT7996_RXQ_MCU_WA_MAIN: u32 = 2;
pub const MT7996_RXQ_MCU_WA_TRI: u32 = 3;
pub const MT7996_RXQ_BAND0: u32 = 4;
pub const MT7996_RXQ_BAND2: u32 = 5;

// Logical queue indices
pub const MT_MCUQ_FWDL: u32 = 0;
pub const MT_MCUQ_WM: u32 = 1;
pub const MT_MCUQ_WA: u32 = 2;

pub const MT_RXQ_MCU: u32 = 0;
pub const MT_RXQ_MCU_WA: u32 = 1;
pub const MT_RXQ_MAIN_WA: u32 = 2;
pub const MT_RXQ_BAND2_WA: u32 = 3;
pub const MT_RXQ_MAIN: u32 = 4;
pub const MT_RXQ_BAND2: u32 = 5;

// ============================================================================
// Interrupt bits
// ============================================================================

pub const MT_INT_RX_DONE_WM: u32 = 1 << 0;
pub const MT_INT_RX_DONE_WA: u32 = 1 << 1;
pub const MT_INT_RX_DONE_WA_MAIN: u32 = 1 << 2;
pub const MT_INT_RX_DONE_WA_TRI: u32 = 1 << 3;
pub const MT_INT_RX_DONE_BAND0: u32 = 1 << 12;
pub const MT_INT_RX_DONE_BAND2: u32 = 1 << 13;
pub const MT_INT_TX_DONE_FWDL: u32 = 1 << 26;
pub const MT_INT_TX_DONE_MCU_WM: u32 = 1 << 27;
pub const MT_INT_TX_DONE_MCU_WA: u32 = 1 << 22;
pub const MT_INT_TX_DONE_BAND0: u32 = 1 << 30;
pub const MT_INT_TX_DONE_BAND1: u32 = 1u32 << 31;
pub const MT_INT_MCU_CMD: u32 = 1 << 29;

pub const MT_INT_RX_DONE_MCU: u32 = MT_INT_RX_DONE_WM | MT_INT_RX_DONE_WA;
pub const MT_INT_TX_DONE_MCU: u32 = MT_INT_TX_DONE_MCU_WA | MT_INT_TX_DONE_MCU_WM | MT_INT_TX_DONE_FWDL;

// HIF2 interrupt registers
pub const MT_INT1_SOURCE_CSR: u32 = MT_WFDMA0_PCIE1_BASE + 0x200;
pub const MT_INT1_MASK_CSR: u32 = MT_WFDMA0_PCIE1_BASE + 0x204;

// WFDMA queue ID flag
pub const WFDMA0: bool = true;

// ============================================================================
// DMA address helpers
// ============================================================================

/// Get low 32 bits of DMA address (for buf0)
#[inline]
pub fn dma_addr_lo(addr: u64) -> u32 {
    addr as u32
}

/// Get high 4 bits of DMA address (for info bits 3:0)
/// MT7996 uses 36-bit addressing: buf0 = bits 31:0, SDP0_H = bits 35:32
///
/// CRITICAL: TX and RX descriptors use DIFFERENT fields for SDP0_H:
///   TX: info[3:0] = SDP0_H, buf1 = 0       (Linux dma.c:339, mt76_dma_add_buf)
///   RX: buf1[3:0] = SDP0_H, info = 0       (Linux dma.c:256, mt76_dma_rx_fill)
#[inline]
pub fn dma_addr_hi(addr: u64) -> u32 {
    ((addr >> 32) & 0xF) as u32
}

/// Maximum DMA buffer size for firmware chunks
pub const MCU_FW_DL_BUF_SIZE: usize = 4096;
