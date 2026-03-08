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

/// PCIe SETTING register — controls PCIe bus parameters.
/// Written to 0x3180 to match OpenWRT configuration.
pub const MT_PCIE_SETTING: u32 = 0x10080;

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

// L2 remap — Source: mmio.c mt7996_reg_map_l2(), regs.h:630-637
// MT7996 offsets from mt7996_offs[]: HIF_REMAP_L2=0x1b4, HIF_REMAP_BASE_L2=0x1000
pub const MT_HIF_REMAP_L2: u32 = 0x1b4;
pub const HIF_REMAP_BASE_L2: u32 = 0x1000;

// Address range constants for L1/L2 routing
// Source: regs.h:646-654, mmio.c:340-362 __mt7996_reg_remap_addr()
pub const MT_WFSYS1_PHY_END: u32 = 0x18bfffff;
pub const MT_INFRA_MCU_START: u32 = 0x7c000000;
pub const MT_INFRA_MCU_END: u32 = 0x7c3fffff;
pub const MT_INFRA_BASE: u32 = 0x18000000;
pub const MT_CBTOP1_PHY_END: u32 = 0x77ffffff;

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

// ADIE (Analog Die) registers — accessed via MCU rf_regval
// regs.h:722-724
pub const MT_ADIE_CHIP_ID_0: u32 = 0x0f00002c;
pub const MT_ADIE_CHIP_ID_1: u32 = 0x1f00002c;

// GPIO pad register — init.c:1115-1143 variant detection
pub const MT_PAD_GPIO: u32 = 0x700056f0;
pub const MT_PAD_GPIO_ADIE_COMB_MASK: u32 = 0x3 << 15;
pub const MT_PAD_GPIO_2ADIE_TBTC: u32 = 1 << 19; // BIT(19) — set = 233 variant

// MCU REG_ACCESS command — mt76_connac_mcu.h:1278
pub const MCU_UNI_CMD_REG_ACCESS: u16 = 0x0d;
pub const UNI_CMD_ACCESS_RF_REG_BASIC: u16 = 1;

// PHYRX register for IPI enable readback — regs.h
pub const MT_WF_PHYRX_BAND_RX_CTRL1_IPI_EN_MASK: u32 = 0x7;

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
pub const MT_DMA_CTL_SDP0_H: u32 = 0xF;           // GENMASK(3,0) — buf0 addr[35:32]
pub const MT_DMA_CTL_SDP1_H: u32 = 0xF << 16;     // GENMASK(19,16) — buf1 addr[35:32]

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

// MCU command register — firmware WDT and error status
// Linux: mt7996/regs.h:610-619
pub const MT_MCU_CMD: u32 = mt_wfdma0(0x1f0);
pub const MT_MCU_CMD_STOP_DMA: u32 = 1 << 2;
pub const MT_MCU_CMD_RESET_DONE: u32 = 1 << 3;
pub const MT_MCU_CMD_RECOVERY_DONE: u32 = 1 << 4;
pub const MT_MCU_CMD_NORMAL_STATE: u32 = 1 << 5;
pub const MT_MCU_CMD_ERROR_MASK: u32 = 0x3E; // GENMASK(5,1) — firmware error bits
pub const MT_MCU_CMD_WM_WDT: u32 = 1u32 << 30;
pub const MT_MCU_CMD_WA_WDT: u32 = 1u32 << 31;
pub const MT_MCU_CMD_WDT_MASK: u32 = MT_MCU_CMD_WA_WDT | MT_MCU_CMD_WM_WDT;

// MCU interrupt event register — host writes to notify firmware during SER L1 recovery
// Linux: regs.h:136-139
pub const MT_MCU_INT_EVENT: u32 = 0x2108;
pub const MT_MCU_INT_EVENT_DMA_STOPPED: u32 = 1 << 0;
pub const MT_MCU_INT_EVENT_DMA_INIT: u32 = 1 << 1;
pub const MT_MCU_INT_EVENT_RESET_DONE: u32 = 1 << 3;

// MCU host interrupt enable — for WDT mask control during full reset
// Linux: regs.h:434
pub const MT_WFDMA0_MCU_HOST_INT_ENA: u32 = mt_wfdma0(0x1f4);

pub const MT_INT_RX_DONE_MCU: u32 = MT_INT_RX_DONE_WM | MT_INT_RX_DONE_WA;

/// Combined mask of all RX done interrupt bits
pub const MT_INT_RX_DONE_ALL: u32 = MT_INT_RX_DONE_WM | MT_INT_RX_DONE_WA
    | MT_INT_RX_DONE_WA_MAIN | MT_INT_RX_DONE_WA_TRI
    | MT_INT_RX_DONE_BAND0 | MT_INT_RX_DONE_BAND2;
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

// ============================================================================
// Per-band register bases — from regs.h and mmio.c mt7996_reg_base[]
// Band 0: 0x820eXXXX, Band 1: 0x820fXXXX, Band 2: 0x830eXXXX
// ============================================================================

/// WF_RMAC per-band base addresses
pub const WF_RMAC_BASE: [u32; 3] = [0x820e5000, 0x820f5000, 0x830e5000];
/// WF_AGG per-band base addresses
pub const WF_AGG_BASE: [u32; 3] = [0x820e2000, 0x820f2000, 0x830e2000];
/// WF_WTBLOFF per-band base addresses
pub const WF_WTBLOFF_BASE: [u32; 3] = [0x820e9000, 0x820f9000, 0x830e9000];

// WF_ARB (Arbiter) — mmio.c:22, regs.h:351-356
pub const WF_ARB_BASE: [u32; 3] = [0x820e3000, 0x820f3000, 0x830e3000];
// ARB_SCR — regs.h:354 — controls TX/RX enable/disable
pub const MT_ARB_SCR_OFS: u32 = 0x000;
pub const MT_ARB_SCR_TX_DISABLE: u32 = 1 << 8;
pub const MT_ARB_SCR_RX_DISABLE: u32 = 1 << 9;

// PLE (Packet List Engine) — debugfs.c:687-691
// Base: 0x820c0000
pub const MT_PLE_BASE: u32 = 0x820c0000;
// FL_Q_EMPTY: bit 10 = BCN_Q0 empty, bit 14 = BCN_Q1 empty — debugfs.c:688
pub const MT_PLE_FL_Q_EMPTY: u32 = MT_PLE_BASE + 0x360;
// FL_Q_CTRL: write (pid<<10 | qid<<24 | BIT(31)) to query queue depth
pub const MT_PLE_FL_Q0_CTRL: u32 = MT_PLE_BASE + 0x3e0;
pub const MT_PLE_FL_Q2_CTRL: u32 = MT_PLE_BASE + 0x3e8;
pub const MT_PLE_FL_Q3_CTRL: u32 = MT_PLE_BASE + 0x3ec;
// Free page counters — debugfs.c
pub const MT_PLE_FREEPG_CNT: u32 = MT_PLE_BASE + 0x380;
pub const MT_PLE_FREEPG_HEAD_TAIL: u32 = MT_PLE_BASE + 0x384;
pub const MT_PLE_HIF_PG_INFO: u32 = MT_PLE_BASE + 0x388;

// WF_TMAC (TX MAC) — mmio.c:23, regs.h:368-383
pub const WF_TMAC_BASE: [u32; 3] = [0x820e4000, 0x820f4000, 0x830e4000];
// TCR0 — TX control register — regs.h:368
pub const MT_TMAC_TCR0_OFS: u32 = 0x0;
// Timeout/IFS timing registers — mac.c:2010-2040 mt7996_mac_set_timing()
pub const MT_TMAC_CDTR_OFS: u32 = 0x0c8;  // CCK detection timeout — regs.h:374
pub const MT_TMAC_ODTR_OFS: u32 = 0x0cc;  // OFDM detection timeout — regs.h:377
pub const MT_TMAC_ICR0_OFS: u32 = 0x014;  // IFS control 0 (EIFS/RIFS/SIFS/SLOT) — regs.h:382
pub const MT_TMAC_ICR1_OFS: u32 = 0x018;  // IFS control 1 (EIFS_CCK) — regs.h:390

// RMAC offsets + masks — from regs.h

// RFCR (RX Filter Control Register) — regs.h:362-389
// MT_WF_RFCR(_band) = MT_WF_RMAC(_band, 0x000)
pub const MT_WF_RFCR_OFS: u32 = 0x000;
pub const MT_WF_RFCR_DROP_STBC_MULTI: u32 = 1 << 0;
pub const MT_WF_RFCR_DROP_FCSFAIL: u32 = 1 << 1;
pub const MT_WF_RFCR_DROP_PROBEREQ: u32 = 1 << 4;
pub const MT_WF_RFCR_DROP_MCAST: u32 = 1 << 5;
pub const MT_WF_RFCR_DROP_BCAST: u32 = 1 << 6;
pub const MT_WF_RFCR_DROP_MCAST_FILTERED: u32 = 1 << 7;
pub const MT_WF_RFCR_DROP_A3_MAC: u32 = 1 << 8;
pub const MT_WF_RFCR_DROP_A3_BSSID: u32 = 1 << 9;
pub const MT_WF_RFCR_DROP_A2_BSSID: u32 = 1 << 10;
pub const MT_WF_RFCR_DROP_OTHER_BEACON: u32 = 1 << 11;
pub const MT_WF_RFCR_DROP_FRAME_REPORT: u32 = 1 << 12;
pub const MT_WF_RFCR_DROP_CTL_RSV: u32 = 1 << 13;
pub const MT_WF_RFCR_DROP_CTS: u32 = 1 << 14;
pub const MT_WF_RFCR_DROP_RTS: u32 = 1 << 15;
pub const MT_WF_RFCR_DROP_DUPLICATE: u32 = 1 << 16;
pub const MT_WF_RFCR_DROP_OTHER_BSS: u32 = 1 << 17;
pub const MT_WF_RFCR_DROP_OTHER_UC: u32 = 1 << 18;
pub const MT_WF_RFCR_DROP_OTHER_TIM: u32 = 1 << 19;
pub const MT_WF_RFCR_DROP_NDPA: u32 = 1 << 20;
pub const MT_WF_RFCR_DROP_UNWANTED_CTL: u32 = 1 << 21;

// RFCR1 (secondary filter) — regs.h:391-398
pub const MT_WF_RFCR1_OFS: u32 = 0x004;
pub const MT_WF_RFCR1_DROP_ACK: u32 = 1 << 4;
pub const MT_WF_RFCR1_DROP_BF_POLL: u32 = 1 << 5;
pub const MT_WF_RFCR1_DROP_BA: u32 = 1 << 6;
pub const MT_WF_RFCR1_DROP_CFEND: u32 = 1 << 7;
pub const MT_WF_RFCR1_DROP_CFACK: u32 = 1 << 8;

pub const MT_WF_RMAC_RSVD0_OFS: u32 = 0x03e0;
pub const MT_WF_RMAC_RSVD0_EIFS_CLR: u32 = 1 << 21;

pub const MT_WF_RMAC_MIB_AIRTIME0_OFS: u32 = 0x0380;
pub const MT_WF_RMAC_MIB_OBSS_BACKOFF: u32 = 0xFFFF;          // GENMASK(15,0)
pub const MT_WF_RMAC_MIB_ED_OFFSET: u32 = 0x1F << 16;         // GENMASK(20,16)

pub const MT_WF_RMAC_MIB_AIRTIME1_OFS: u32 = 0x0384;
pub const MT_WF_RMAC_MIB_NONQOSD_BACKOFF: u32 = 0xFFFF << 16; // GENMASK(31,16)

pub const MT_WF_RMAC_MIB_AIRTIME3_OFS: u32 = 0x038c;
pub const MT_WF_RMAC_MIB_QOS01_BACKOFF: u32 = 0xFFFFFFFF;     // GENMASK(31,0)

pub const MT_WF_RMAC_MIB_AIRTIME4_OFS: u32 = 0x0390;
pub const MT_WF_RMAC_MIB_QOS23_BACKOFF: u32 = 0xFFFFFFFF;     // GENMASK(31,0)

// WTBLOFF offsets + masks — from regs.h
pub const MT_WTBLOFF_ACR_OFS: u32 = 0x010;
pub const MT_WTBLOFF_ADM_BACKOFFTIME: u32 = 1 << 29;

pub const MT_WTBLOFF_RSCR_OFS: u32 = 0x008;
pub const MT_WTBLOFF_RSCR_RCPI_MODE: u32 = 0x3 << 30;         // GENMASK(31,30)
pub const MT_WTBLOFF_RSCR_RCPI_PARAM: u32 = 0x3 << 24;        // GENMASK(25,24)

// AGG offsets + masks — from regs.h
pub const MT_AGG_ACR4_OFS: u32 = 0x3c;
pub const MT_AGG_ACR_PPDU_TXS2H: u32 = 1 << 1;

// NOTE: PSE registers (0x820c8xxx) are NOT host-accessible on MT7996.
// Reads return 0xDEADBEEF. Linux's MT7996 driver never defines PSE register addresses.
// PSE is firmware-internal; use PLE registers (0x820c0xxx) for host diagnostics.

// MDP (global) — from regs.h
pub const MT_MDP_BASE: u32 = 0x820cc000;
pub const MT_MDP_DCR2: u32 = MT_MDP_BASE + 0x8e8;
pub const MT_MDP_DCR2_RX_TRANS_SHORT: u32 = 1 << 2;

// WTBL (global) — from regs.h
pub const MT_WTBLON_TOP_BASE: u32 = 0x820d4000;
/// MT7996 uses __OFFS(WTBL_UPDATE) = 0x380 — regs.h
pub const MT_WTBL_UPDATE: u32 = MT_WTBLON_TOP_BASE + 0x380;
// WTBL indirect read — regs.h
pub const MT_WTBL_ITCR: u32 = MT_WTBLON_TOP_BASE + 0x3b0;
pub const MT_WTBL_ITDR0: u32 = MT_WTBLON_TOP_BASE + 0x3b8;
pub const MT_WTBL_ITDR1: u32 = MT_WTBLON_TOP_BASE + 0x3bc;
pub const MT_WTBL_UPDATE_WLAN_IDX: u32 = 0xFFF;               // GENMASK(11,0)
pub const MT_WTBL_UPDATE_ADM_COUNT_CLEAR: u32 = 1 << 14;
pub const MT_WTBL_UPDATE_BUSY: u32 = 1u32 << 31;

// WTBL size defaults for MT7996 — from mt7996.h
pub const MT7996_WTBL_SIZE_GROUP: u32 = 4;
pub const MT7996_WTBL_BMC_SIZE: u32 = 64;
// mt7996_wtbl_size(dev) = (group << 8) + bmc_size = 1088
// MT7996_WTBL_RESERVED = wtbl_size - 1 = 1087 — BMC STA goes here
// Linux: mt7996.h:18 — mlink->wcid.idx = MT7996_WTBL_RESERVED - mlink->idx
pub const MT7996_WTBL_RESERVED: u16 = (MT7996_WTBL_SIZE_GROUP * 256 + MT7996_WTBL_BMC_SIZE - 1) as u16;

// Rate table — from mt7996/mt7996.h:105-106
pub const MT7996_BEACON_RATES_TBL: u8 = 25;
pub const MT7996_BASIC_RATES_TBL: u8 = 31;

// Rate encoding — from mt76.h
pub const MT_TX_RATE_MODE: u32 = 0x3C0;                        // GENMASK(9,6)
pub const MT_TX_RATE_IDX: u32 = 0x3F;                          // GENMASK(5,0)

// PHY mode bits — mt76_connac_mcu.h:915-920
pub const PHY_MODE_A: u8 = 1 << 0;   // BIT(0) = 0x01 — 11a OFDM (5GHz)
pub const PHY_MODE_B: u8 = 1 << 1;   // BIT(1) = 0x02 — 11b CCK (2.4GHz)
pub const PHY_MODE_G: u8 = 1 << 2;   // BIT(2) = 0x04 — 11g OFDM (2.4GHz)
pub const PHY_MODE_GN: u8 = 1 << 3;  // BIT(3) = 0x08 — 11n HT (2.4GHz)
pub const PHY_MODE_AN: u8 = 1 << 4;  // BIT(4) = 0x10 — 11n HT (5GHz)

// ============================================================================
// MCU UNI command IDs — from mt76_connac_mcu.h enum
// ============================================================================

pub const MCU_UNI_CMD_EDCA_UPDATE: u16 = 0x04;
pub const MCU_UNI_CMD_BAND_CONFIG: u16 = 0x08;
pub const MCU_UNI_CMD_BF: u16 = 0x33;
pub const MCU_UNI_CMD_THERMAL: u16 = 0x35;
pub const MCU_UNI_CMD_FIXED_RATE_TABLE: u16 = 0x40;
pub const MCU_UNI_CMD_OFFCH_SCAN_CTRL: u16 = 0x58;

// Thermal tags — mcu.h enum UNI_CMD_THERMAL_*
pub const UNI_CMD_THERMAL_PROTECT_ENABLE: u16 = 0x6;
pub const UNI_CMD_THERMAL_PROTECT_DISABLE: u16 = 0x7;
pub const UNI_CMD_THERMAL_PROTECT_DUTY_CONFIG: u16 = 0x8;

// Band config tags — mcu.h enum UNI_BAND_CONFIG_*
pub const UNI_BAND_CONFIG_RADIO_ENABLE: u16 = 0;
pub const UNI_BAND_CONFIG_RTS_THRESHOLD: u16 = 0x08;

// Thermal defaults — mt7996.h
pub const MT7996_CRIT_TEMP: u32 = 110;   // restore temp (°C)
pub const MT7996_MAX_TEMP: u32 = 120;    // trigger temp (°C)
pub const MT7996_THERMAL_THROTTLE_MAX: u8 = 100;

// ============================================================================
// TX Power — mcu.h, mt76_connac_mcu.h
// ============================================================================

// mt76_connac_mcu.h:1293
pub const MCU_UNI_CMD_TXPOWER: u16 = 0x2b;
// mt7996/mcu.h:906
pub const UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL: u16 = 4;
// mt7996/mt7996.h:97
pub const MT7996_SKU_PATH_NUM: usize = 494;

// ============================================================================
// Channel switch — mcu.h, mt76_connac_mcu.h
// ============================================================================

pub const MCU_UNI_CMD_CHANNEL_SWITCH: u16 = 0x34;
pub const UNI_CHANNEL_SWITCH: u16 = 0;
pub const UNI_CHANNEL_RX_PATH: u16 = 1;

// Bandwidth encoding — mt76_connac.h CMD_CBW_*
pub const CMD_CBW_20MHZ: u8 = 0;
#[allow(dead_code)]
pub const CMD_CBW_40MHZ: u8 = 1;
#[allow(dead_code)]
pub const CMD_CBW_80MHZ: u8 = 2;
#[allow(dead_code)]
pub const CMD_CBW_160MHZ: u8 = 3;

// Channel switch reasons — mt76_connac_mcu.h
pub const CH_SWITCH_NORMAL: u8 = 0;
#[allow(dead_code)]
pub const CH_SWITCH_SCAN_BYPASS_DPD: u8 = 9;

// Channel band encoding
pub const CH_BAND_2GHZ: u8 = 0;
pub const CH_BAND_5GHZ: u8 = 1;
#[allow(dead_code)]
pub const CH_BAND_6GHZ: u8 = 2;

// ============================================================================
// Header translation — mcu.h
// ============================================================================

pub const MCU_UNI_CMD_RX_HDR_TRANS: u16 = 0x12;
pub const UNI_HDR_TRANS_EN: u16 = 0;
pub const UNI_HDR_TRANS_VLAN: u16 = 1;
pub const UNI_HDR_TRANS_BLACKLIST: u16 = 2;

// ETH_P_PAE for hdr_trans blacklist
pub const ETH_P_PAE: u16 = 0x888E;

// ============================================================================
// Interface management — mcu.h, mt76_connac_mcu.h
// ============================================================================

pub const MCU_UNI_CMD_DEV_INFO_UPDATE: u16 = 0x01;
pub const MCU_UNI_CMD_BSS_INFO_UPDATE: u16 = 0x02;
pub const MCU_UNI_CMD_STA_REC_UPDATE: u16 = 0x03;

// DEV_INFO TLV tags
pub const DEV_INFO_ACTIVE: u16 = 0;

// BSS_INFO TLV tags — mt76_connac_mcu.h:1363-1382
pub const UNI_BSS_INFO_BASIC: u16 = 0;
pub const UNI_BSS_INFO_RA: u16 = 1;
pub const UNI_BSS_INFO_RLM: u16 = 2;
pub const UNI_BSS_INFO_BCN_CONTENT: u16 = 7;
pub const UNI_BSS_INFO_RATE: u16 = 11;
pub const UNI_BSS_INFO_SEC: u16 = 16;
pub const UNI_BSS_INFO_TXCMD: u16 = 18;
pub const UNI_BSS_INFO_IFS_TIME: u16 = 23;
pub const UNI_BSS_INFO_MLD: u16 = 26;

// STA_REC TLV tags — mt76_connac_mcu.h:802-836
pub const STA_REC_BASIC: u16 = 0;
pub const STA_REC_RA: u16 = 1;            // Rate adaptation — mt76_connac_mcu.h:804
pub const STA_REC_BA: u16 = 6;            // Block Ack session — mt76_connac_mcu.h:809
pub const STA_REC_TX_PROC: u16 = 8;       // for hdr trans and CSO in CR4
pub const STA_REC_HT: u16 = 9;            // HT capabilities — mt76_connac_mcu.h:812
pub const STA_REC_KEY_V2: u16 = 0x11;    // Key installation — mt76_connac_mcu.h:820 (enum index 17)
pub const STA_REC_HDRT: u16 = 0x28;      // Connac3 HDR trans mode — mt76_connac_mcu.h:834
pub const STA_REC_HDR_TRANS: u16 = 0x2B;

// STA capability flags for STA_REC_RA sta_cap field — mt76_connac_mcu.h:1128-1146
pub const STA_CAP_WMM: u32      = 1 << 0;
pub const STA_CAP_SGI_20: u32   = 1 << 4;
pub const STA_CAP_SGI_40: u32   = 1 << 5;
pub const STA_CAP_TX_STBC: u32  = 1 << 6;
pub const STA_CAP_RX_STBC: u32  = 1 << 7;
pub const STA_CAP_HT: u32       = 1 << 11;
pub const STA_CAP_LDPC: u32     = 1 << 15;

// RCPI initial value — mt76_connac_mcu.c INIT_RCPI
pub const INIT_RCPI: u8 = 180;

// VOW (airtime fairness) — mcu.h:860
pub const UNI_VOW_DRR_CTRL: u16 = 0;

// Connection types — mt76_connac_mcu.h:860-876
pub const STA_TYPE_STA: u32 = 1 << 0;    // BIT(0)
pub const STA_TYPE_AP: u32 = 1 << 1;     // BIT(1)
pub const NETWORK_INFRA: u32 = 1 << 16;  // BIT(16)
pub const CONNECTION_INFRA_STA: u32 = STA_TYPE_STA | NETWORK_INFRA;  // 0x10001
pub const STA_TYPE_BC: u32 = 1 << 5;    // BIT(5)
pub const CONNECTION_INFRA_AP: u32 = STA_TYPE_AP | NETWORK_INFRA;    // 0x10002
pub const CONNECTION_INFRA_BC: u32 = STA_TYPE_BC | NETWORK_INFRA;    // 0x10020
pub const CONN_STATE_DISCONNECT: u8 = 0;
pub const CONN_STATE_CONNECT: u8 = 1;
pub const CONN_STATE_PORT_SECURE: u8 = 2;

/// Cipher suites for STA_REC_KEY_V2 — mt76_connac_mcu.h MCU_CIPHER_*
pub const MCU_CIPHER_AES_CCMP: u8 = 5; // AES-CCMP (WPA2) — mt76_connac_mcu.h:1129

// EXTRA_INFO flags for STA_REC
pub const EXTRA_INFO_VER: u16 = 1 << 0;
pub const EXTRA_INFO_NEW: u16 = 1 << 1;

// OMAC indices
#[allow(dead_code)]
pub const HW_BSSID_0: u8 = 0;
pub const HW_BSSID_1: u8 = 1;  // First STA interface

// ============================================================================
// Beacon TX constants — from mt76_connac3_mac.h
// ============================================================================

/// Hardware TXD size (8 × u32) — mt76_connac3_mac.h
pub const MT_TXD_SIZE: usize = 32;

/// Beacon queue index — mt76_connac3_mac.h:16
pub const MT_LMAC_BCN0: u8 = 0x12;

/// TX packet format: cut-through — mt76_connac3_mac.h:175
pub const MT_TX_TYPE_CT: u8 = 0;
/// TX packet format: firmware — mt76_connac3_mac.h:172
pub const MT_TX_TYPE_FW: u8 = 3;

/// Header format: 802.11 — mt76_connac3_mac.h:167
pub const MT_HDR_FORMAT_802_11: u8 = 2;

/// TXD1 TID for management frames — mt76_connac3_mac.h enum tx_mgnt_type
/// Linux enum: MT_TX_NORMAL = 0, MT_TX_TIMING = 1, MT_TX_ADDBA = 2
/// Used with FIELD_PREP(MT_TXD1_TID, tid) where TID = GENMASK(24,21)
/// Source: mt7996/mac.c:812 `tid = MT_TX_NORMAL;`
pub const MT_TX_NORMAL: u32 = 0;

// NOTE: There is NO LONG_FORMAT bit in connac3 (MT7996).
// All TXDs are 8 DWORDs (32 bytes). BIT(13) of TXD1 is TGID[0] (band index).
// Setting BIT(13) routes frames to band 1 instead of band 0!
// Source: mt76_connac3_mac.h — MT_TXD1_TGID = GENMASK(13,12)

// NOTE: VTA is in TXD6 BIT(28) (MT_TXD6_VTA), NOT in TXD1!
// TXD1 bit 28 falls within OWN_MAC GENMASK(30,25). Setting it corrupts
// OWN_MAC from 0 to 8, causing firmware to drop all frames.
// The old MT_TXD1_VTA constant was WRONG and has been removed.

/// ALTX queue index for management frames — mt76_connac3_mac.h:15
pub const MT_LMAC_ALTX0: u8 = 0x10;

/// MCU port RX queue index — mt76.h:78 MT_TX_MCU_PORT_RX_Q0
pub const MCU_PORT_RX: u8 = 0x20;

// ============================================================================
// CT mode TXP (TX Packet) — mt76_connac3_mac.h, mt76_connac.h
// In CT mode (PKT_FMT=0), a fw_txp follows the 32-byte TXD. The MAC reads
// the TXP to find buffer pointers for the actual frame data.
// ============================================================================

/// CT info flags — mt76_connac3_mac.h:207-212
pub const MT_CT_INFO_APPLY_TXD: u16 = 1 << 0;
pub const MT_CT_INFO_MGMT_FRAME: u16 = 1 << 2;
pub const MT_CT_INFO_NONE_CIPHER_FRAME: u16 = 1 << 3;
pub const MT_CT_INFO_FROM_HOST: u16 = 1 << 7;

/// CT parse length — first N bytes of frame copied to descriptor buf1
/// for firmware header parsing. Source: mt76_connac3_mac.h:292
pub const MT_CT_PARSE_LEN: usize = 72;

/// fw_txp structure size (packed) — mt76_connac.h:135-143
/// Layout: flags(2) + token(2) + bss_idx(1) + rept_wds_wcid(2) + nbuf(1) +
///         buf[6](24) + len[6](12) = 44 bytes
pub const MT_FW_TXP_SIZE: usize = 44;

/// Total TXWI size = TXD + TXP — mt7996/mmio.c:820-821
pub const MT_TXWI_SIZE: usize = MT_TXD_SIZE + MT_FW_TXP_SIZE; // 76

/// DMA descriptor ctrl — SD_LEN1 and LAST_SEC1 for 2-buffer mode
/// ctrl[13:0] = SD_LEN1, ctrl[14] = LAST_SEC1
pub const MT_DMA_CTL_SD_LEN1_MASK: u32 = 0x3FFF;
pub const MT_DMA_CTL_LAST_SEC1: u32 = 1 << 14;

// TXD0 bitfields — mt76_connac3_mac.h:216-219
pub const MT_TXD0_Q_IDX_SHIFT: u32 = 25;       // GENMASK(31,25)
pub const MT_TXD0_PKT_FMT_SHIFT: u32 = 23;     // GENMASK(24,23)

// TXD1 bitfields — mt76_connac3_mac.h:221-229
pub const MT_TXD1_FIXED_RATE: u32 = 1 << 31;
pub const MT_TXD1_OWN_MAC_SHIFT: u32 = 25;     // GENMASK(30,25)
pub const MT_TXD1_TID_SHIFT: u32 = 21;          // GENMASK(24,21)
pub const MT_TXD1_HDR_INFO_SHIFT: u32 = 16;     // GENMASK(20,16)
pub const MT_TXD1_HDR_FORMAT_SHIFT: u32 = 14;   // GENMASK(15,14)

// TXD3 bitfields — mt76_connac3_mac.h:243-255
pub const MT_TXD3_SW_POWER_MGMT: u32 = 1 << 29;
pub const MT_TXD3_BA_DISABLE: u32 = 1 << 28;
pub const MT_TXD3_REM_TX_COUNT_SHIFT: u32 = 11; // GENMASK(15,11)
pub const MT_TXD3_BCM: u32 = 1 << 4;
pub const MT_TXD3_NO_ACK: u32 = 1 << 0;

// TXD6 bitfields — mt76_connac3_mac.h:269-281
pub const MT_TXD6_VTA: u32 = 1 << 28;
pub const MT_TXD6_FIXED_BW: u32 = 1 << 25;
pub const MT_TXD6_TX_RATE_SHIFT: u32 = 16;      // GENMASK(21,16)
pub const MT_TXD6_MSDU_CNT_SHIFT: u32 = 4;      // GENMASK(9,4)
pub const MT_TXD6_DIS_MAT: u32 = 1 << 3;
pub const MT_TXD6_DAS: u32 = 1 << 2;

// ============================================================================
// MIB counter registers — from regs.h
// WF_MIB_BASE: band 0 = 0x820ed000, band 1 = 0x820fd000, band 2 = 0x830ed000
// ============================================================================

pub const WF_MIB_BASE: [u32; 3] = [0x820ed000, 0x820fd000, 0x830ed000];

#[inline]
pub const fn mt_wf_mib(band: usize, ofs: u32) -> u32 { WF_MIB_BASE[band] + ofs }

// Channel idle count — regs.h:259
pub const MT_MIB_SDR6_OFS: u32 = 0x020;

// TX AMPDU count — regs.h:268
pub const MT_MIB_TSCR0_OFS: u32 = 0x6b0;

// TX MPDU attempts (all mpdus in ampdu, regardless of success) — regs.h:272
pub const MT_MIB_TSCR3_OFS: u32 = 0x6bc;

// TX MPDU success — regs.h:275
pub const MT_MIB_TSCR4_OFS: u32 = 0x6c0;

// RX FCS error count — mmio.c:38 offset 0x7ac for MT7996
pub const MT_MIB_RSCR1_OFS: u32 = 0x7ac;

// RX vector mismatch — mmio.c:34 offset 0x720 for MT7996
pub const MT_MIB_RVSR0_OFS: u32 = 0x720;

// RX MPDU count — mmio.c:43 offset 0x964 for MT7996
pub const MT_MIB_RSCR31_OFS: u32 = 0x964;

// RX FIFO full — mmio.c:44 offset 0x96c for MT7996
pub const MT_MIB_RSCR33_OFS: u32 = 0x96c;

// RX delimiter fail — mmio.c:45 offset 0x974 for MT7996
pub const MT_MIB_RSCR35_OFS: u32 = 0x974;

// Beacon TX counters — debugfs.c
pub const MT_MIB_BTSCR0_OFS: u32 = 0x5e0;
pub const MT_MIB_BTSCR5_OFS: u32 = 0x788;
pub const MT_MIB_BTSCR6_OFS: u32 = 0x798;

// Beacon statistics — debugfs.c
pub const MT_MIB_BSCR0_OFS: u32 = 0x9cc;
pub const MT_MIB_BSCR1_OFS: u32 = 0x9d0;
pub const MT_MIB_BSCR2_OFS: u32 = 0x9d4;
pub const MT_MIB_BSCR3_OFS: u32 = 0x9d8;
pub const MT_MIB_BSCR4_OFS: u32 = 0x9dc;
pub const MT_MIB_BSCR17_OFS: u32 = 0xa10;

// TX AMPDU attempts — regs.h
pub const MT_MIB_TSCR5_OFS: u32 = 0x6c4;
pub const MT_MIB_TSCR6_OFS: u32 = 0x6c8;
pub const MT_MIB_TSCR7_OFS: u32 = 0x6d0;

// Additional MIB registers — mac.c:2743-2882 mt7996_mac_update_stats()
// All offsets from mt7996_offs[] in mmio.c (MT7996 variant)
pub const MT_MIB_TSCR1_OFS: u32 = 0x6b4;   // RX BA count
pub const MT_MIB_TSCR2_OFS: u32 = 0x6b8;   // TX stop queue empty
pub const MT_MIB_SDR27_OFS: u32 = 0x080;    // TX RWP fail
pub const MT_MIB_SDR28_OFS: u32 = 0x084;    // TX RWP need
pub const MT_MIB_RSCR27_OFS: u32 = 0x954;   // RX AMPDU count
pub const MT_MIB_RSCR28_OFS: u32 = 0x958;   // RX AMPDU bytes
pub const MT_MIB_RSCR29_OFS: u32 = 0x95c;   // RX AMPDU valid subframe
pub const MT_MIB_RSCR30_OFS: u32 = 0x960;   // RX AMPDU valid subframe bytes
pub const MT_MIB_RSCR36_OFS: u32 = 0x978;   // RX len mismatch
pub const MT_MIB_RVSR1_OFS: u32 = 0x724;    // RX vec queue overflow
pub const MT_MIB_BSCR5_OFS: u32 = 0x9e0;    // Beamforming HE
pub const MT_MIB_BSCR6_OFS: u32 = 0x9e4;    // Beamforming EHT
pub const MT_MIB_BSCR7_OFS: u32 = 0x9e8;    // BF trigger
pub const MT_MIB_BFTFCR_OFS: u32 = 0x5d0;   // ACK fail count
pub const MT_MIB_TRDR1_OFS: u32 = 0xa28;    // TX AGG count base

// UMIB (unified MIB) — regs.h:312-315
pub const MT_WF_UMIB_BASE: u32 = 0x820cd000;
/// UMIB_RPDCR per-band: base + 0x594 + band * 0x164
#[inline]
pub const fn mt_umib_rpdcr(band: u32) -> u32 { MT_WF_UMIB_BASE + 0x594 + band * 0x164 }

// ETBF (e-BF) per-band base — mmio.c:27
pub const WF_ETBF_BASE: [u32; 3] = [0x820ea000, 0x820fa000, 0x830ea000];
/// ETBF_RX_FB_CONT — regs.h:211
#[inline]
pub const fn mt_etbf_rx_fb_cont(band: usize) -> u32 { WF_ETBF_BASE[band] + 0x100 }

// ============================================================================
// LPON (TSF timer) — regs.h:217-228
// WF_LPON_BASE band 0 = 0x820eb000 (in FIXED_MAP, size 0x0400)
// ============================================================================

pub const WF_LPON_BASE: [u32; 3] = [0x820eb000, 0x820fb000, 0x830eb000];

#[inline]
pub const fn mt_wf_lpon(band: usize, ofs: u32) -> u32 { WF_LPON_BASE[band] + ofs }

// Free-running counter — debugfs.c
pub const MT_LPON_FRCR_OFS: u32 = 0x37c;
// TSF read: write SW_READ to TCR, then read UTTR0/UTTR1
// regs.h:220-228
pub const MT_LPON_UTTR0_OFS: u32 = 0x360;
pub const MT_LPON_UTTR1_OFS: u32 = 0x364;
pub const MT_LPON_TCR_OFS: u32 = 0x0a8;  // TCR(band, n=0) for BSS 0
pub const MT_LPON_TCR_SW_MODE: u32 = 0x3;     // GENMASK(1,0)
pub const MT_LPON_TCR_SW_READ: u32 = 0x3;     // GENMASK(1,0) — both bits set

// ============================================================================
// PHYRX registers — from regs.h:749-774
// These are accessed via L2 remap (0x83xxxxxx > CBTOP1_PHY_END)
// ============================================================================

// PHYRX BAND base — regs.h:756
// MT_WF_PHYRX_BAND(_band, ofs) = 0x83080000 + (band << 20) + ofs
pub const MT_WF_PHYRX_BAND_BASE: u32 = 0x83080000;

#[inline]
pub const fn mt_wf_phyrx_band(band: u32, ofs: u32) -> u32 {
    MT_WF_PHYRX_BAND_BASE + (band << 20) + ofs
}

// RX_CTRL1 — regs.h:767-768
// MT_WF_PHYRX_BAND_RX_CTRL1(_band) = MT_WF_PHYRX_BAND(_band, 0x2004)
pub const MT_WF_PHYRX_BAND_RX_CTRL1_OFS: u32 = 0x2004;
pub const MT_WF_PHYRX_BAND_RX_CTRL1_IPI_EN: u32 = 0x7; // GENMASK(2,0)
pub const MT_WF_PHYRX_BAND_RX_CTRL1_STSCNT_EN: u32 = 0x7 << 9; // GENMASK(11,9)

// CSD_BAND_RXTD12 — regs.h:772-774
// MT_WF_PHYRX_CSD_BAND_RXTD12(_band) = MT_WF_PHYRX_BAND(_band, 0x8230)
pub const MT_WF_PHYRX_CSD_BAND_RXTD12_OFS: u32 = 0x8230;
pub const MT_WF_PHYRX_CSD_BAND_RXTD12_IRPI_SW_CLR_ONLY: u32 = 1 << 18;
pub const MT_WF_PHYRX_CSD_BAND_RXTD12_IRPI_SW_CLR: u32 = 1 << 29;

// ============================================================================
// RXD field definitions — mt76_connac3_mac.h
// Used for parsing received frames on BAND0 data RX ring
// ============================================================================

pub const MT_RXD0_LENGTH: u32 = 0xFFFF;           // GENMASK(15,0) — packet length
/// Packet type field: GENMASK(31,27) of RXD0 — mt76_connac3_mac.h
pub const MT_RXD0_PKT_TYPE_SHIFT: u32 = 27;
pub const MT_RXD0_PKT_TYPE_MASK: u32 = 0xF800_0000; // GENMASK(31,27)
pub const MT_RXD1_NORMAL_GROUP_1: u32 = 1 << 16;  // IV (4 bytes)
pub const MT_RXD1_NORMAL_GROUP_2: u32 = 1 << 17;  // Timestamp (4 bytes)
pub const MT_RXD1_NORMAL_GROUP_3: u32 = 1 << 18;  // P-RXV (16 bytes, RSSI/RCPI)
pub const MT_RXD1_NORMAL_GROUP_4: u32 = 1 << 19;  // FC/QoS/Seq (16 bytes)
pub const MT_RXD1_NORMAL_GROUP_5: u32 = 1 << 20;  // C-RXV (24 bytes, HE/EHT)
/// WLAN index: GENMASK(11,0) of RXD1
pub const MT_RXD1_NORMAL_WLAN_IDX: u32 = 0xFFF;
/// Band index: GENMASK(28,27) of RXD1
pub const MT_RXD1_NORMAL_BAND_IDX_SHIFT: u32 = 27;
pub const MT_RXD1_NORMAL_BAND_IDX_MASK: u32 = 0x1800_0000; // GENMASK(28,27)
pub const MT_RXD2_NORMAL_HDR_TRANS: u32 = 1 << 7;
pub const MT_RXD2_NORMAL_HDR_OFFSET: u32 = 0x7 << 13; // GENMASK(15,13) — pad units×2
/// Non-data frame: 1 = management/control, 0 = data — mt76_connac3_mac.h
pub const MT_RXD2_NORMAL_NDATA: u32 = 1 << 29;
/// Address type: GENMASK(17,16) of RXD3 — U2M=0, MCAST=1, BCAST=2, OTHER=3
pub const MT_RXD3_NORMAL_ADDR_TYPE_SHIFT: u32 = 16;
pub const MT_RXD3_NORMAL_ADDR_TYPE_MASK: u32 = 0x3_0000; // GENMASK(17,16)
pub const MT_RXD3_NORMAL_FCS_ERR: u32 = 1 << 24;

// ============================================================================
// RXD packet types — mt76_connac3_mac.h PKT_TYPE_*
// ============================================================================

pub const PKT_TYPE_TXS: u32 = 0;           // TX status
pub const PKT_TYPE_TXRXV: u32 = 1;         // TX/RX vector
pub const PKT_TYPE_NORMAL: u32 = 2;        // Normal data/mgmt frame
pub const PKT_TYPE_TXRX_NOTIFY: u32 = 6;   // TX/RX notification
pub const PKT_TYPE_RX_EVENT: u32 = 7;      // MCU RX event
pub const PKT_TYPE_RX_FW_MONITOR: u32 = 0x0c; // FW monitor

// ============================================================================
// TX Free event format — mt76_connac3_mac.h
// Source: mt76/mt76_connac3_mac.h:312-323
// ============================================================================

/// tx_info[0] fields (first u32 AFTER 32-byte RXD)
pub const MT_TXFREE0_MSDU_CNT_MASK: u32 = 0x03FF_0000; // GENMASK(25,16)
pub const MT_TXFREE0_MSDU_CNT_SHIFT: u32 = 16;

/// tx_info[1] fields
pub const MT_TXFREE1_VER_MASK: u32 = 0x000F_0000; // GENMASK(19,16)
pub const MT_TXFREE1_VER_SHIFT: u32 = 16;

/// tx_info[2..] entry flags
pub const MT_TXFREE_INFO_PAIR: u32 = 1 << 31;     // BIT(31) — new WCID context
pub const MT_TXFREE_INFO_HEADER: u32 = 1 << 30;   // BIT(30) — retry/status header
pub const MT_TXFREE_INFO_MSDU_ID: u32 = 0x7FFF;   // GENMASK(14,0) — 15-bit token
pub const MT_TXFREE_INFO_MSDU_ID_V3: u32 = 0xFFF; // GENMASK(11,0) — 12-bit for ver<4

// ============================================================================
// MCU UNI event EIDs — mt76_connac_mcu.h / mt7996/mcu.h
// ============================================================================

pub const MCU_UNI_EVENT_FW_LOG: u8 = 0x04;
pub const MCU_UNI_EVENT_IE_COUNTDOWN: u8 = 0x09;
pub const MCU_UNI_EVENT_RDD_REPORT: u8 = 0x11;
pub const MCU_UNI_EVENT_THERMAL: u8 = 0x35;
pub const MCU_UNI_EVENT_ALL_STA_INFO: u8 = 0x6e;

/// MCU_UNI_CMD_ALL_STA_INFO = 0x6e — mt76_connac_mcu.h:1309
pub const MCU_UNI_CMD_ALL_STA_INFO: u8 = 0x6e;
/// UNI_ALL_STA_TXRX_RATE = 0 — mt76_connac_mcu.h:1396
pub const UNI_ALL_STA_TXRX_RATE: u16 = 0;

/// MCU_UNI_CMD_GET_MIB_INFO = 0x22 — mt76_connac_mcu.h:1287
pub const MCU_UNI_CMD_GET_MIB_INFO: u16 = 0x22;
/// UNI_CMD_MIB_DATA = 0 — mt7996/mcu.h:866 (first enum member)
pub const UNI_CMD_MIB_DATA: u16 = 0;
/// Channel MIB offsets — mt7996/mcu.h:278-281
pub const UNI_MIB_OBSS_AIRTIME: u32 = 26;
pub const UNI_MIB_NON_WIFI_TIME: u32 = 27;
pub const UNI_MIB_TX_TIME: u32 = 28;
pub const UNI_MIB_RX_TIME: u32 = 29;

// ============================================================================
// PCIe IRQ number — GIC SPI 171 + 32 (MT7996 on pcie3)
// ============================================================================

pub const PCIE3_IRQ: u32 = 203;

// ============================================================================
// MCU command sub-tags and constants
// ============================================================================

// WSYS_CONFIG — mcu.h
pub const UNI_WSYS_CONFIG_FW_LOG_CTRL: u16 = 0;

// WA EXT commands — mt76_connac_mcu.h
pub const MCU_EXT_CMD_MWDS_SUPPORT: u8 = 0x80;

// VOW RX airtime — mcu.h
pub const UNI_VOW_RX_AT_AIRTIME_CLR_EN: u16 = 0x0e;
pub const UNI_VOW_RX_AT_AIRTIME_EN: u16 = 0x0b;

// WA PARAM — mt76_connac_mcu.h
pub const MCU_WA_PARAM_RED: u32 = 0x0e;
pub const MCU_WA_PARAM_HW_PATH_HIF_VER: u32 = 0x2f;

// RRO — mcu.h
pub const UNI_RRO_SET_PLATFORM_TYPE: u16 = 0x2;
pub const UNI_RRO_SET_BYPASS_MODE: u16 = 0x4;
pub const UNI_RRO_SET_TXFREE_PATH: u16 = 0x5;
pub const UNI_RRO_SET_FLUSH_TIMEOUT: u16 = 0x7;

// EFUSE — mcu.h
pub const UNI_EFUSE_ACCESS: u16 = 1;
pub const UNI_EFUSE_BUFFER_MODE: u16 = 2;
pub const UNI_EFUSE_FREE_BLOCK: u16 = 3;
pub const EE_MODE_EFUSE: u8 = 0;
pub const EE_MODE_BUFFER: u8 = 1;
pub const EE_FORMAT_WHOLE: u8 = 1;
pub const PER_PAGE_SIZE: usize = 1024;

// Per-band register address helpers
#[inline]
pub const fn mt_wf_rmac(band: usize, ofs: u32) -> u32 { WF_RMAC_BASE[band] + ofs }
#[inline]
pub const fn mt_wf_agg(band: usize, ofs: u32) -> u32 { WF_AGG_BASE[band] + ofs }
#[inline]
pub const fn mt_wtbloff(band: usize, ofs: u32) -> u32 { WF_WTBLOFF_BASE[band] + ofs }
#[inline]
pub const fn mt_wf_arb(band: usize, ofs: u32) -> u32 { WF_ARB_BASE[band] + ofs }
#[inline]
pub const fn mt_wf_tmac(band: usize, ofs: u32) -> u32 { WF_TMAC_BASE[band] + ofs }
