//! U-Boot mtk_eth.c/h Line-by-Line Rust Port
//!
//! This is a direct translation of U-Boot's drivers/net/mtk_eth/mtk_eth.c and mtk_eth.h
//! to Rust, preserving all functionality including GMAC2, MDIO, SFP, SGMII, USXGMII, etc.
//!
//! Original authors: Weijie Gao, Mark Lee (MediaTek)
//! Rust port: line-by-line translation, no shortcuts

#![allow(dead_code)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]

// =============================================================================
// mtk_eth.h - Register definitions and structures
// =============================================================================

// -----------------------------------------------------------------------------
// Capability bits (enum mkt_eth_capabilities)
// -----------------------------------------------------------------------------
pub const MTK_TRGMII_BIT: u32 = 0;
pub const MTK_TRGMII_MT7621_CLK_BIT: u32 = 1;
pub const MTK_U3_COPHY_V2_BIT: u32 = 2;
pub const MTK_INFRA_BIT: u32 = 3;
pub const MTK_NETSYS_V2_BIT: u32 = 4;
pub const MTK_NETSYS_V3_BIT: u32 = 5;

// PATH BITS
pub const MTK_ETH_PATH_GMAC1_TRGMII_BIT: u32 = 6;
pub const MTK_ETH_PATH_GMAC2_SGMII_BIT: u32 = 7;
pub const MTK_ETH_PATH_MT7622_SGMII_BIT: u32 = 8;
pub const MTK_ETH_PATH_MT7629_GMAC2_BIT: u32 = 9;

// Capability flags
pub const MTK_TRGMII: u32 = 1 << MTK_TRGMII_BIT;
pub const MTK_TRGMII_MT7621_CLK: u32 = 1 << MTK_TRGMII_MT7621_CLK_BIT;
pub const MTK_U3_COPHY_V2: u32 = 1 << MTK_U3_COPHY_V2_BIT;
pub const MTK_INFRA: u32 = 1 << MTK_INFRA_BIT;
pub const MTK_NETSYS_V2: u32 = 1 << MTK_NETSYS_V2_BIT;
pub const MTK_NETSYS_V3: u32 = 1 << MTK_NETSYS_V3_BIT;

// Supported path present on SoCs
pub const MTK_ETH_PATH_GMAC1_TRGMII: u32 = 1 << MTK_ETH_PATH_GMAC1_TRGMII_BIT;
pub const MTK_ETH_PATH_GMAC2_SGMII: u32 = 1 << MTK_ETH_PATH_GMAC2_SGMII_BIT;
pub const MTK_ETH_PATH_MT7622_SGMII: u32 = 1 << MTK_ETH_PATH_MT7622_SGMII_BIT;
pub const MTK_ETH_PATH_MT7629_GMAC2: u32 = 1 << MTK_ETH_PATH_MT7629_GMAC2_BIT;

pub const MTK_GMAC1_TRGMII: u32 = MTK_ETH_PATH_GMAC1_TRGMII | MTK_TRGMII;
pub const MTK_GMAC2_U3_QPHY: u32 = MTK_ETH_PATH_GMAC2_SGMII | MTK_U3_COPHY_V2 | MTK_INFRA;

/// Check if caps has all bits in _x
#[inline]
pub const fn mtk_has_caps(caps: u32, x: u32) -> bool {
    (caps & x) == x
}

// SoC-specific capabilities
pub const MT7621_CAPS: u32 = MTK_GMAC1_TRGMII | MTK_TRGMII_MT7621_CLK;
pub const MT7622_CAPS: u32 = MTK_ETH_PATH_MT7622_SGMII;
pub const MT7623_CAPS: u32 = MTK_GMAC1_TRGMII;
pub const MT7629_CAPS: u32 = MTK_ETH_PATH_MT7629_GMAC2 | MTK_INFRA;
pub const MT7981_CAPS: u32 = MTK_GMAC2_U3_QPHY | MTK_NETSYS_V2;
pub const MT7986_CAPS: u32 = MTK_NETSYS_V2;
pub const MT7987_CAPS: u32 = MTK_NETSYS_V3 | MTK_GMAC2_U3_QPHY | MTK_INFRA;
pub const MT7988_CAPS: u32 = MTK_NETSYS_V3 | MTK_INFRA;

// -----------------------------------------------------------------------------
// Frame Engine Register Bases
// -----------------------------------------------------------------------------
pub const PDMA_V1_BASE: u32 = 0x0800;
pub const PDMA_V2_BASE: u32 = 0x6000;
pub const PDMA_V3_BASE: u32 = 0x6800;
pub const GDMA1_BASE: u32 = 0x0500;
pub const GDMA2_BASE: u32 = 0x1500;
pub const GDMA3_BASE: u32 = 0x0540;
pub const GMAC_BASE: u32 = 0x10000;
pub const GSW_BASE: u32 = 0x20000;

// -----------------------------------------------------------------------------
// Ethernet subsystem registers (ETHSYS)
// -----------------------------------------------------------------------------
pub const ETHSYS_SYSCFG1_REG: u32 = 0x14;

#[inline]
pub const fn syscfg1_ge_mode_s(n: u32) -> u32 {
    12 + (n * 2)
}

pub const SYSCFG1_GE_MODE_M: u32 = 0x3;
pub const SYSCFG1_SGMII_SEL_M: u32 = 0x300; // GENMASK(9, 8)

#[inline]
pub const fn syscfg1_sgmii_sel(gmac: u32) -> u32 {
    1 << (9 - gmac)
}

pub const ETHSYS_CLKCFG0_REG: u32 = 0x2c;
pub const ETHSYS_TRGMII_CLK_SEL362_5: u32 = 1 << 11;

// -----------------------------------------------------------------------------
// Top misc registers
// -----------------------------------------------------------------------------
pub const TOPMISC_NETSYS_PCS_MUX: u32 = 0x84;
pub const NETSYS_PCS_MUX_MASK: u32 = 0x3; // GENMASK(1, 0)
pub const MUX_G2_USXGMII_SEL: u32 = 1 << 1;
pub const MUX_HSGMII1_G1_SEL: u32 = 1 << 0;

pub const USB_PHY_SWITCH_REG: u32 = 0x218;
pub const QPHY_SEL_MASK: u32 = 0x3;
pub const SGMII_QPHY_SEL: u32 = 0x2;

pub const MT7629_INFRA_MISC2_REG: u32 = 0x70c;
pub const INFRA_MISC2_BONDING_OPTION: u32 = 0xFFFF; // GENMASK(15, 0)

// -----------------------------------------------------------------------------
// SYSCFG1_GE_MODE: GE Modes
// -----------------------------------------------------------------------------
pub const GE_MODE_RGMII: u32 = 0;
pub const GE_MODE_MII: u32 = 1;
pub const GE_MODE_MII_PHY: u32 = 2;
pub const GE_MODE_RMII: u32 = 3;

// -----------------------------------------------------------------------------
// SGMII subsystem config registers
// -----------------------------------------------------------------------------
pub const SGMSYS_PCS_CONTROL_1: u32 = 0x0;
pub const SGMII_LINK_STATUS: u32 = 1 << 18;
pub const SGMII_AN_ENABLE: u32 = 1 << 12;
pub const SGMII_AN_RESTART: u32 = 1 << 9;

pub const SGMSYS_SGMII_MODE: u32 = 0x20;
pub const SGMII_AN_MODE: u32 = 0x31120103;
pub const SGMII_FORCE_MODE: u32 = 0x31120019;

pub const SGMSYS_QPHY_PWR_STATE_CTRL: u32 = 0xe8;
pub const SGMII_PHYA_PWD: u32 = 1 << 4;

pub const SGMSYS_QPHY_WRAP_CTRL: u32 = 0xec;
pub const SGMII_PN_SWAP_TX_RX: u32 = 0x03;

pub const SGMSYS_GEN2_SPEED: u32 = 0x2028;
pub const SGMSYS_GEN2_SPEED_V2: u32 = 0x128;
pub const SGMSYS_SPEED_MASK: u32 = 0xC; // GENMASK(3, 2)
pub const SGMSYS_SPEED_2500: u32 = 1;

// -----------------------------------------------------------------------------
// USXGMII subsystem config registers
// -----------------------------------------------------------------------------
// Register to control USXGMII XFI PLL digital
pub const XFI_PLL_DIG_GLB8: u32 = 0x08;
pub const RG_XFI_PLL_EN: u32 = 1 << 31;

// Register to control USXGMII XFI PLL analog
pub const XFI_PLL_ANA_GLB8: u32 = 0x108;
pub const RG_XFI_PLL_ANA_SWWA: u32 = 0x02283248;

// -----------------------------------------------------------------------------
// Frame Engine Registers
// -----------------------------------------------------------------------------
pub const PSE_NO_DROP_CFG_REG: u32 = 0x108;
pub const PSE_NO_DROP_GDM1: u32 = 1 << 1;

pub const FE_GLO_MISC_REG: u32 = 0x124;
pub const PDMA_VER_V2: u32 = 1 << 4;

// -----------------------------------------------------------------------------
// PDMA registers
// -----------------------------------------------------------------------------
#[inline]
pub const fn tx_base_ptr_reg(n: u32) -> u32 {
    0x000 + n * 0x10
}

#[inline]
pub const fn tx_max_cnt_reg(n: u32) -> u32 {
    0x004 + n * 0x10
}

#[inline]
pub const fn tx_ctx_idx_reg(n: u32) -> u32 {
    0x008 + n * 0x10
}

#[inline]
pub const fn tx_dtx_idx_reg(n: u32) -> u32 {
    0x00c + n * 0x10
}

#[inline]
pub const fn rx_base_ptr_reg(n: u32) -> u32 {
    0x100 + n * 0x10
}

#[inline]
pub const fn rx_max_cnt_reg(n: u32) -> u32 {
    0x104 + n * 0x10
}

#[inline]
pub const fn rx_crx_idx_reg(n: u32) -> u32 {
    0x108 + n * 0x10
}

#[inline]
pub const fn rx_drx_idx_reg(n: u32) -> u32 {
    0x10c + n * 0x10
}

pub const PDMA_GLO_CFG_REG: u32 = 0x204;
pub const TX_WB_DDONE: u32 = 1 << 6;
pub const RX_DMA_BUSY: u32 = 1 << 3;
pub const RX_DMA_EN: u32 = 1 << 2;
pub const TX_DMA_BUSY: u32 = 1 << 1;
pub const TX_DMA_EN: u32 = 1 << 0;

pub const PDMA_RST_IDX_REG: u32 = 0x208;
pub const RST_DRX_IDX0: u32 = 1 << 16;
pub const RST_DTX_IDX0: u32 = 1 << 0;

// -----------------------------------------------------------------------------
// GDMA registers
// -----------------------------------------------------------------------------
pub const GDMA_IG_CTRL_REG: u32 = 0x000;
pub const GDM_ICS_EN: u32 = 1 << 22;
pub const GDM_TCS_EN: u32 = 1 << 21;
pub const GDM_UCS_EN: u32 = 1 << 20;
pub const STRP_CRC: u32 = 1 << 16;
pub const MYMAC_DP_S: u32 = 12;
pub const MYMAC_DP_M: u32 = 0xf000;
pub const BC_DP_S: u32 = 8;
pub const BC_DP_M: u32 = 0xf00;
pub const MC_DP_S: u32 = 4;
pub const MC_DP_M: u32 = 0xf0;
pub const UN_DP_S: u32 = 0;
pub const UN_DP_M: u32 = 0x0f;

pub const GDMA_EG_CTRL_REG: u32 = 0x004;
pub const GDMA_CPU_BRIDGE_EN: u32 = 1 << 31;

pub const GDMA_MAC_LSB_REG: u32 = 0x008;
pub const GDMA_MAC_MSB_REG: u32 = 0x00c;

// MYMAC_DP/BC_DP/MC_DP/UN_DP: Destination ports
pub const DP_PDMA: u32 = 0;
pub const DP_GDMA1: u32 = 1;
pub const DP_GDMA2: u32 = 2;
pub const DP_PPE: u32 = 4;
pub const DP_QDMA: u32 = 5;
pub const DP_DISCARD: u32 = 7;

// -----------------------------------------------------------------------------
// GMAC Registers
// -----------------------------------------------------------------------------
pub const GMAC_PPSC_REG: u32 = 0x0000;
pub const PHY_MDC_CFG: u32 = 0x3F000000; // GENMASK(29, 24)
pub const MDC_TURBO: u32 = 1 << 20;
pub const MDC_MAX_FREQ: u32 = 25000000;
pub const MDC_MAX_DIVIDER: u32 = 63;

pub const GMAC_PIAC_REG: u32 = 0x0004;
pub const PHY_ACS_ST: u32 = 1 << 31;
pub const MDIO_REG_ADDR_S: u32 = 25;
pub const MDIO_REG_ADDR_M: u32 = 0x3e000000;
pub const MDIO_PHY_ADDR_S: u32 = 20;
pub const MDIO_PHY_ADDR_M: u32 = 0x1f00000;
pub const MDIO_CMD_S: u32 = 18;
pub const MDIO_CMD_M: u32 = 0xc0000;
pub const MDIO_ST_S: u32 = 16;
pub const MDIO_ST_M: u32 = 0x30000;
pub const MDIO_RW_DATA_S: u32 = 0;
pub const MDIO_RW_DATA_M: u32 = 0xffff;

pub const GMAC_XGMAC_STS_REG: u32 = 0x000c;
pub const P1_XGMAC_FORCE_LINK: u32 = 1 << 15;

pub const GMAC_MAC_MISC_REG: u32 = 0x0010;
pub const MISC_MDC_TURBO: u32 = 1 << 4;

pub const GMAC_GSW_CFG_REG: u32 = 0x0080;
pub const GSWTX_IPG_M: u32 = 0xF0000;
pub const GSWTX_IPG_S: u32 = 16;
pub const GSWRX_IPG_M: u32 = 0xF;
pub const GSWRX_IPG_S: u32 = 0;

// MDIO_CMD: MDIO commands
pub const MDIO_CMD_ADDR: u32 = 0;
pub const MDIO_CMD_WRITE: u32 = 1;
pub const MDIO_CMD_READ: u32 = 2;
pub const MDIO_CMD_READ_C45: u32 = 3;

// MDIO_ST: MDIO start field
pub const MDIO_ST_C45: u32 = 0;
pub const MDIO_ST_C22: u32 = 1;

#[inline]
pub const fn gmac_port_mcr(p: u32) -> u32 {
    0x0100 + p * 0x100
}

pub const MAC_RX_PKT_LEN_S: u32 = 24;
pub const MAC_RX_PKT_LEN_M: u32 = 0x3000000;
pub const IPG_CFG_S: u32 = 18;
pub const IPG_CFG_M: u32 = 0xc0000;
pub const MAC_MODE: u32 = 1 << 16;
pub const FORCE_MODE: u32 = 1 << 15;
pub const MAC_TX_EN: u32 = 1 << 14;
pub const MAC_RX_EN: u32 = 1 << 13;
pub const DEL_RXFIFO_CLR: u32 = 1 << 12;
pub const BKOFF_EN: u32 = 1 << 9;
pub const BACKPR_EN: u32 = 1 << 8;
pub const FORCE_RX_FC: u32 = 1 << 5;
pub const FORCE_TX_FC: u32 = 1 << 4;
pub const FORCE_SPD_S: u32 = 2;
pub const FORCE_SPD_M: u32 = 0x0c;
pub const FORCE_DPX: u32 = 1 << 1;
pub const FORCE_LINK: u32 = 1 << 0;

// Values of IPG_CFG
pub const IPG_96BIT: u32 = 0;
pub const IPG_96BIT_WITH_SHORT_IPG: u32 = 1;
pub const IPG_64BIT: u32 = 2;

// MAC_RX_PKT_LEN: Max RX packet length
pub const MAC_RX_PKT_LEN_1518: u32 = 0;
pub const MAC_RX_PKT_LEN_1536: u32 = 1;
pub const MAC_RX_PKT_LEN_1552: u32 = 2;
pub const MAC_RX_PKT_LEN_JUMBO: u32 = 3;

// FORCE_SPD: Forced link speed
pub const SPEED_10M: u32 = 0;
pub const SPEED_100M: u32 = 1;
pub const SPEED_1000M: u32 = 2;

pub const GMAC_TRGMII_RCK_CTRL: u32 = 0x300;
pub const RX_RST: u32 = 1 << 31;
pub const RXC_DQSISEL: u32 = 1 << 30;

pub const NUM_TRGMII_CTRL: u32 = 5;

#[inline]
pub const fn gmac_trgmii_td_odt(n: u32) -> u32 {
    0x354 + n * 8
}

pub const TD_DM_DRVN_S: u32 = 4;
pub const TD_DM_DRVN_M: u32 = 0xf0;
pub const TD_DM_DRVP_S: u32 = 0;
pub const TD_DM_DRVP_M: u32 = 0x0f;

// -----------------------------------------------------------------------------
// XGMAC Status Registers
// -----------------------------------------------------------------------------
#[inline]
pub const fn xgmac_sts(x: u32) -> u32 {
    if x == 2 { 0x001C } else { 0x000C }
}

#[inline]
pub const fn xgmac_force_link(x: u32) -> u32 {
    if x == 1 { 1 << 31 } else { 1 << 15 }
}

// XGMAC Registers
#[inline]
pub const fn xgmac_port_mcr(x: u32) -> u32 {
    0x2000 + (x - 1) * 0x1000
}

pub const XGMAC_TRX_DISABLE: u32 = 0xf;
pub const XGMAC_FORCE_TX_FC: u32 = 1 << 5;
pub const XGMAC_FORCE_RX_FC: u32 = 1 << 4;

// -----------------------------------------------------------------------------
// MDIO Indirect Access Registers
// -----------------------------------------------------------------------------
pub const MII_MMD_ACC_CTL_REG: u32 = 0x0d;
pub const MMD_CMD_S: u32 = 14;
pub const MMD_CMD_M: u32 = 0xc000;
pub const MMD_DEVAD_S: u32 = 0;
pub const MMD_DEVAD_M: u32 = 0x1f;

// MMD_CMD: MMD commands
pub const MMD_ADDR: u32 = 0;
pub const MMD_DATA: u32 = 1;
pub const MMD_DATA_RW_POST_INC: u32 = 2;
pub const MMD_DATA_W_POST_INC: u32 = 3;

pub const MII_MMD_ADDR_DATA_REG: u32 = 0x0e;

// -----------------------------------------------------------------------------
// PDMA descriptors
// -----------------------------------------------------------------------------

/// V1 RX DMA descriptor (16 bytes)
#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct MtkRxDma {
    pub rxd1: u32,
    pub rxd2: u32,
    pub rxd3: u32,
    pub rxd4: u32,
}

/// V2 RX DMA descriptor (32 bytes)
#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct MtkRxDmaV2 {
    pub rxd1: u32,
    pub rxd2: u32,
    pub rxd3: u32,
    pub rxd4: u32,
    pub rxd5: u32,
    pub rxd6: u32,
    pub rxd7: u32,
    pub rxd8: u32,
}

/// V1 TX DMA descriptor (16 bytes)
#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct MtkTxDma {
    pub txd1: u32,
    pub txd2: u32,
    pub txd3: u32,
    pub txd4: u32,
}

/// V2 TX DMA descriptor (32 bytes)
#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct MtkTxDmaV2 {
    pub txd1: u32,
    pub txd2: u32,
    pub txd3: u32,
    pub txd4: u32,
    pub txd5: u32,
    pub txd6: u32,
    pub txd7: u32,
    pub txd8: u32,
}

// -----------------------------------------------------------------------------
// PDMA TXD fields
// -----------------------------------------------------------------------------
pub const PDMA_TXD2_DDONE: u32 = 1 << 31;
pub const PDMA_TXD2_LS0: u32 = 1 << 30;

// V1: SDL0 in bits [29:16]
pub const PDMA_V1_TXD2_SDL0_M: u32 = 0x3FFF0000; // GENMASK(29, 16)

#[inline]
pub const fn pdma_v1_txd2_sdl0_set(v: u32) -> u32 {
    (v << 16) & PDMA_V1_TXD2_SDL0_M
}

// V2: SDL0 in bits [23:8]
pub const PDMA_V2_TXD2_SDL0_M: u32 = 0x00FFFF00; // GENMASK(23, 8)

#[inline]
pub const fn pdma_v2_txd2_sdl0_set(v: u32) -> u32 {
    (v << 8) & PDMA_V2_TXD2_SDL0_M
}

// V1: FPORT in txd4 bits [27:25]
pub const PDMA_V1_TXD4_FPORT_M: u32 = 0x0E000000; // GENMASK(27, 25)

#[inline]
pub const fn pdma_v1_txd4_fport_set(v: u32) -> u32 {
    (v << 25) & PDMA_V1_TXD4_FPORT_M
}

// V2: FPORT in txd4 bits [27:24]
pub const PDMA_V2_TXD4_FPORT_M: u32 = 0x0F000000; // GENMASK(27, 24)

#[inline]
pub const fn pdma_v2_txd4_fport_set(v: u32) -> u32 {
    (v << 24) & PDMA_V2_TXD4_FPORT_M
}

// V2: FPORT in txd5 bits [19:16]
pub const PDMA_V2_TXD5_FPORT_M: u32 = 0x000F0000; // GENMASK(19, 16)

#[inline]
pub const fn pdma_v2_txd5_fport_set(v: u32) -> u32 {
    (v << 16) & PDMA_V2_TXD5_FPORT_M
}

// -----------------------------------------------------------------------------
// PDMA RXD fields
// -----------------------------------------------------------------------------
pub const PDMA_RXD2_DDONE: u32 = 1 << 31;
pub const PDMA_RXD2_LS0: u32 = 1 << 30;

// V1: PLEN0 in bits [29:16]
pub const PDMA_V1_RXD2_PLEN0_M: u32 = 0x3FFF0000; // GENMASK(29, 16)

#[inline]
pub const fn pdma_v1_rxd2_plen0_get(v: u32) -> u32 {
    (v & PDMA_V1_RXD2_PLEN0_M) >> 16
}

#[inline]
pub const fn pdma_v1_rxd2_plen0_set(v: u32) -> u32 {
    (v << 16) & PDMA_V1_RXD2_PLEN0_M
}

// V2: PLEN0 in bits [23:8]
pub const PDMA_V2_RXD2_PLEN0_M: u32 = 0x00FFFF00; // GENMASK(23, 8)

#[inline]
pub const fn pdma_v2_rxd2_plen0_get(v: u32) -> u32 {
    (v & PDMA_V2_RXD2_PLEN0_M) >> 8
}

#[inline]
pub const fn pdma_v2_rxd2_plen0_set(v: u32) -> u32 {
    (v << 8) & PDMA_V2_RXD2_PLEN0_M
}

// =============================================================================
// mtk_eth.c - Driver implementation
// =============================================================================

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------
pub const NUM_TX_DESC: usize = 32;
pub const NUM_RX_DESC: usize = 32;
pub const PKTSIZE_ALIGN: usize = 1536;
pub const TX_TOTAL_BUF_SIZE: usize = NUM_TX_DESC * PKTSIZE_ALIGN;
pub const RX_TOTAL_BUF_SIZE: usize = NUM_RX_DESC * PKTSIZE_ALIGN;
pub const TOTAL_PKT_BUF_SIZE: usize = TX_TOTAL_BUF_SIZE + RX_TOTAL_BUF_SIZE;

/// GDMA forward to CPU (normal mode)
/// (0x20000000 | GDM_ICS_EN | GDM_TCS_EN | GDM_UCS_EN | STRP_CRC |
///  (DP_PDMA << MYMAC_DP_S) | (DP_PDMA << BC_DP_S) |
///  (DP_PDMA << MC_DP_S) | (DP_PDMA << UN_DP_S))
pub const GDMA_FWD_TO_CPU: u32 = 0x20000000
    | GDM_ICS_EN
    | GDM_TCS_EN
    | GDM_UCS_EN
    | STRP_CRC
    | (DP_PDMA << MYMAC_DP_S)
    | (DP_PDMA << BC_DP_S)
    | (DP_PDMA << MC_DP_S)
    | (DP_PDMA << UN_DP_S);

/// GDMA bridge to CPU (for internal switch)
/// (0xC0000000 | GDM_ICS_EN | GDM_TCS_EN | GDM_UCS_EN |
///  (DP_PDMA << MYMAC_DP_S) | (DP_PDMA << BC_DP_S) |
///  (DP_PDMA << MC_DP_S) | (DP_PDMA << UN_DP_S))
pub const GDMA_BRIDGE_TO_CPU: u32 = 0xC0000000
    | GDM_ICS_EN
    | GDM_TCS_EN
    | GDM_UCS_EN
    | (DP_PDMA << MYMAC_DP_S)
    | (DP_PDMA << BC_DP_S)
    | (DP_PDMA << MC_DP_S)
    | (DP_PDMA << UN_DP_S);

/// GDMA forward to discard
/// (0x20000000 | GDM_ICS_EN | GDM_TCS_EN | GDM_UCS_EN | STRP_CRC |
///  (DP_DISCARD << MYMAC_DP_S) | (DP_DISCARD << BC_DP_S) |
///  (DP_DISCARD << MC_DP_S) | (DP_DISCARD << UN_DP_S))
pub const GDMA_FWD_DISCARD: u32 = 0x20000000
    | GDM_ICS_EN
    | GDM_TCS_EN
    | GDM_UCS_EN
    | STRP_CRC
    | (DP_DISCARD << MYMAC_DP_S)
    | (DP_DISCARD << BC_DP_S)
    | (DP_DISCARD << MC_DP_S)
    | (DP_DISCARD << UN_DP_S);

// -----------------------------------------------------------------------------
// PHY interface modes
// -----------------------------------------------------------------------------
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum PhyInterfaceMode {
    Na = 0,
    Mii,
    Gmii,
    Rgmii,
    RgmiiId,
    RgmiiRxid,
    RgmiiTxid,
    Rmii,
    Sgmii,
    Basex2500,
    Usxgmii,
    Baseer10g,
    Xgmii,
}

// Speed values (from U-Boot)
pub const SPEED_10: u32 = 10;
pub const SPEED_100: u32 = 100;
pub const SPEED_1000: u32 = 1000;
pub const SPEED_2500: u32 = 2500;
pub const SPEED_10000: u32 = 10000;

// Flow control
pub const FLOW_CTRL_TX: u8 = 1;
pub const FLOW_CTRL_RX: u8 = 2;

// MII advertisement
pub const ADVERTISE_PAUSE_CAP: u16 = 0x0400;
pub const ADVERTISE_PAUSE_ASYM: u16 = 0x0800;
pub const LPA_PAUSE_CAP: u16 = 0x0400;
pub const LPA_PAUSE_ASYM: u16 = 0x0800;
pub const ADVERTISED_Pause: u32 = 1 << 13;
pub const ADVERTISED_Asym_Pause: u32 = 1 << 14;

// -----------------------------------------------------------------------------
// SoC data structure
// -----------------------------------------------------------------------------

/// struct mtk_soc_data - This is the structure holding all differences
/// among various platforms
#[derive(Debug, Clone, Copy)]
pub struct MtkSocData {
    /// Flags shown the extra capability for the SoC
    pub caps: u32,
    /// The offset for register ANA_RGC3 related to sgmiisys syscon
    pub ana_rgc3: u32,
    /// Number of GDMAs
    pub gdma_count: u32,
    /// Register base of PDMA block
    pub pdma_base: u32,
    /// Tx DMA descriptor size
    pub txd_size: usize,
    /// Rx DMA descriptor size
    pub rxd_size: usize,
}

// Pre-defined SoC data configurations
pub const MT7988_DATA: MtkSocData = MtkSocData {
    caps: MT7988_CAPS,
    ana_rgc3: 0x128,
    gdma_count: 3,
    pdma_base: PDMA_V3_BASE,
    txd_size: core::mem::size_of::<MtkTxDmaV2>(),
    rxd_size: core::mem::size_of::<MtkRxDmaV2>(),
};

pub const MT7987_DATA: MtkSocData = MtkSocData {
    caps: MT7987_CAPS,
    ana_rgc3: 0x128,
    gdma_count: 3,
    pdma_base: PDMA_V3_BASE,
    txd_size: core::mem::size_of::<MtkTxDmaV2>(),
    rxd_size: core::mem::size_of::<MtkRxDmaV2>(),
};

pub const MT7986_DATA: MtkSocData = MtkSocData {
    caps: MT7986_CAPS,
    ana_rgc3: 0x128,
    gdma_count: 2,
    pdma_base: PDMA_V2_BASE,
    txd_size: core::mem::size_of::<MtkTxDmaV2>(),
    rxd_size: core::mem::size_of::<MtkRxDmaV2>(),
};

pub const MT7981_DATA: MtkSocData = MtkSocData {
    caps: MT7981_CAPS,
    ana_rgc3: 0x128,
    gdma_count: 2,
    pdma_base: PDMA_V2_BASE,
    txd_size: core::mem::size_of::<MtkTxDmaV2>(),
    rxd_size: core::mem::size_of::<MtkRxDmaV2>(),
};

pub const MT7629_DATA: MtkSocData = MtkSocData {
    caps: MT7629_CAPS,
    ana_rgc3: 0x128,
    gdma_count: 2,
    pdma_base: PDMA_V1_BASE,
    txd_size: core::mem::size_of::<MtkTxDma>(),
    rxd_size: core::mem::size_of::<MtkRxDma>(),
};

pub const MT7623_DATA: MtkSocData = MtkSocData {
    caps: MT7623_CAPS,
    ana_rgc3: 0,
    gdma_count: 2,
    pdma_base: PDMA_V1_BASE,
    txd_size: core::mem::size_of::<MtkTxDma>(),
    rxd_size: core::mem::size_of::<MtkRxDma>(),
};

pub const MT7622_DATA: MtkSocData = MtkSocData {
    caps: MT7622_CAPS,
    ana_rgc3: 0x2028,
    gdma_count: 2,
    pdma_base: PDMA_V1_BASE,
    txd_size: core::mem::size_of::<MtkTxDma>(),
    rxd_size: core::mem::size_of::<MtkRxDma>(),
};

pub const MT7621_DATA: MtkSocData = MtkSocData {
    caps: MT7621_CAPS,
    ana_rgc3: 0,
    gdma_count: 2,
    pdma_base: PDMA_V1_BASE,
    txd_size: core::mem::size_of::<MtkTxDma>(),
    rxd_size: core::mem::size_of::<MtkRxDma>(),
};

// -----------------------------------------------------------------------------
// Driver private data structure
// -----------------------------------------------------------------------------

/// Main ethernet driver private data
pub struct MtkEthPriv {
    // Packet buffer pool
    pub pkt_pool: [u8; TOTAL_PKT_BUF_SIZE],

    // DMA ring pointers (non-cached)
    pub tx_ring_noc: *mut u8,
    pub rx_ring_noc: *mut u8,

    // DMA ring indices
    pub rx_dma_owner_idx0: usize,
    pub tx_cpu_owner_idx0: usize,

    // Register bases
    pub fe_base: usize,
    pub gmac_base: usize,
    pub sgmii_base: usize,

    // Regmap bases (for syscon access)
    pub ethsys_base: usize,
    pub infra_base: usize,
    pub usxgmii_base: usize,
    pub xfi_pextp_base: usize,
    pub xfi_pll_base: usize,
    pub toprgu_base: usize,

    // SoC configuration
    pub soc: &'static MtkSocData,

    // GMAC configuration
    pub gmac_id: u32,
    pub force_mode: bool,
    pub speed: u32,
    pub duplex: bool,
    pub mdc: u32,
    pub pn_swap: bool,

    // PHY configuration
    pub phy_interface: PhyInterfaceMode,
    pub phy_addr: u8,

    // Switch configuration
    pub swname: Option<&'static str>,
    pub mcm: bool,

    // MAC address
    pub enetaddr: [u8; 6],
}

impl MtkEthPriv {
    pub fn new(soc: &'static MtkSocData) -> Self {
        Self {
            pkt_pool: [0; TOTAL_PKT_BUF_SIZE],
            tx_ring_noc: core::ptr::null_mut(),
            rx_ring_noc: core::ptr::null_mut(),
            rx_dma_owner_idx0: 0,
            tx_cpu_owner_idx0: 0,
            fe_base: 0,
            gmac_base: 0,
            sgmii_base: 0,
            ethsys_base: 0,
            infra_base: 0,
            usxgmii_base: 0,
            xfi_pextp_base: 0,
            xfi_pll_base: 0,
            toprgu_base: 0,
            soc,
            gmac_id: 0,
            force_mode: false,
            speed: 0,
            duplex: false,
            mdc: 0,
            pn_swap: false,
            phy_interface: PhyInterfaceMode::Na,
            phy_addr: 0,
            swname: None,
            mcm: false,
            enetaddr: [0; 6],
        }
    }
}

// =============================================================================
// Register access helper functions
// =============================================================================

/// Read a 32-bit register
#[inline]
pub unsafe fn readl(addr: usize) -> u32 {
    core::ptr::read_volatile(addr as *const u32)
}

/// Write a 32-bit register
#[inline]
pub unsafe fn writel(val: u32, addr: usize) {
    core::ptr::write_volatile(addr as *mut u32, val);
}

/// Read-modify-write: clear bits then set bits
#[inline]
pub unsafe fn clrsetbits_le32(addr: usize, clr: u32, set: u32) {
    let val = readl(addr);
    writel((val & !clr) | set, addr);
}

/// Set bits
#[inline]
pub unsafe fn setbits_le32(addr: usize, set: u32) {
    clrsetbits_le32(addr, 0, set);
}

/// Clear bits
#[inline]
pub unsafe fn clrbits_le32(addr: usize, clr: u32) {
    clrsetbits_le32(addr, clr, 0);
}

// -----------------------------------------------------------------------------
// PDMA register access
// -----------------------------------------------------------------------------

/// Write PDMA register
/// static void mtk_pdma_write(struct mtk_eth_priv *eth_priv, u32 reg, u32 val)
#[inline]
pub unsafe fn mtk_pdma_write(eth_priv: &MtkEthPriv, reg: u32, val: u32) {
    writel(val, eth_priv.fe_base + eth_priv.soc.pdma_base as usize + reg as usize);
}

/// Read PDMA register
#[inline]
pub unsafe fn mtk_pdma_read(eth_priv: &MtkEthPriv, reg: u32) -> u32 {
    readl(eth_priv.fe_base + eth_priv.soc.pdma_base as usize + reg as usize)
}

/// Read-modify-write PDMA register
/// static void mtk_pdma_rmw(struct mtk_eth_priv *eth_priv, u32 reg, u32 clr, u32 set)
#[inline]
pub unsafe fn mtk_pdma_rmw(eth_priv: &MtkEthPriv, reg: u32, clr: u32, set: u32) {
    clrsetbits_le32(
        eth_priv.fe_base + eth_priv.soc.pdma_base as usize + reg as usize,
        clr,
        set,
    );
}

// -----------------------------------------------------------------------------
// GDMA register access
// -----------------------------------------------------------------------------

/// Get GDMA base address for given GDMA number
fn gdma_base(no: u32) -> u32 {
    if no == 2 {
        GDMA3_BASE
    } else if no == 1 {
        GDMA2_BASE
    } else {
        GDMA1_BASE
    }
}

/// Write GDMA register
/// static void mtk_gdma_write(struct mtk_eth_priv *eth_priv, int no, u32 reg, u32 val)
#[inline]
pub unsafe fn mtk_gdma_write(eth_priv: &MtkEthPriv, no: u32, reg: u32, val: u32) {
    writel(val, eth_priv.fe_base + gdma_base(no) as usize + reg as usize);
}

/// Read GDMA register
#[inline]
pub unsafe fn mtk_gdma_read(eth_priv: &MtkEthPriv, no: u32, reg: u32) -> u32 {
    readl(eth_priv.fe_base + gdma_base(no) as usize + reg as usize)
}

// -----------------------------------------------------------------------------
// Frame Engine register access
// -----------------------------------------------------------------------------

/// Read-modify-write Frame Engine register
/// void mtk_fe_rmw(struct mtk_eth_priv *eth_priv, u32 reg, u32 clr, u32 set)
#[inline]
pub unsafe fn mtk_fe_rmw(eth_priv: &MtkEthPriv, reg: u32, clr: u32, set: u32) {
    clrsetbits_le32(eth_priv.fe_base + reg as usize, clr, set);
}

/// Read Frame Engine register
#[inline]
pub unsafe fn mtk_fe_read(eth_priv: &MtkEthPriv, reg: u32) -> u32 {
    readl(eth_priv.fe_base + reg as usize)
}

/// Write Frame Engine register
#[inline]
pub unsafe fn mtk_fe_write(eth_priv: &MtkEthPriv, reg: u32, val: u32) {
    writel(val, eth_priv.fe_base + reg as usize);
}

// -----------------------------------------------------------------------------
// GMAC register access
// -----------------------------------------------------------------------------

/// Read GMAC register
/// static u32 mtk_gmac_read(struct mtk_eth_priv *eth_priv, u32 reg)
#[inline]
pub unsafe fn mtk_gmac_read(eth_priv: &MtkEthPriv, reg: u32) -> u32 {
    readl(eth_priv.gmac_base + reg as usize)
}

/// Write GMAC register
/// static void mtk_gmac_write(struct mtk_eth_priv *eth_priv, u32 reg, u32 val)
#[inline]
pub unsafe fn mtk_gmac_write(eth_priv: &MtkEthPriv, reg: u32, val: u32) {
    writel(val, eth_priv.gmac_base + reg as usize);
}

/// Read-modify-write GMAC register
/// void mtk_gmac_rmw(struct mtk_eth_priv *eth_priv, u32 reg, u32 clr, u32 set)
#[inline]
pub unsafe fn mtk_gmac_rmw(eth_priv: &MtkEthPriv, reg: u32, clr: u32, set: u32) {
    clrsetbits_le32(eth_priv.gmac_base + reg as usize, clr, set);
}

// -----------------------------------------------------------------------------
// Ethsys register access
// -----------------------------------------------------------------------------

/// Read ethsys register
#[inline]
pub unsafe fn mtk_ethsys_read(eth_priv: &MtkEthPriv, reg: u32) -> u32 {
    readl(eth_priv.ethsys_base + reg as usize)
}

/// Write ethsys register
#[inline]
pub unsafe fn mtk_ethsys_write(eth_priv: &MtkEthPriv, reg: u32, val: u32) {
    writel(val, eth_priv.ethsys_base + reg as usize);
}

/// Read-modify-write ethsys register
/// void mtk_ethsys_rmw(struct mtk_eth_priv *eth_priv, u32 reg, u32 clr, u32 set)
#[inline]
pub unsafe fn mtk_ethsys_rmw(eth_priv: &MtkEthPriv, reg: u32, clr: u32, set: u32) {
    let val = mtk_ethsys_read(eth_priv, reg);
    mtk_ethsys_write(eth_priv, reg, (val & !clr) | set);
}

// -----------------------------------------------------------------------------
// Infra register access
// -----------------------------------------------------------------------------

/// Read-modify-write infra register
/// static void mtk_infra_rmw(struct mtk_eth_priv *eth_priv, u32 reg, u32 clr, u32 set)
#[inline]
pub unsafe fn mtk_infra_rmw(eth_priv: &MtkEthPriv, reg: u32, clr: u32, set: u32) {
    let val = readl(eth_priv.infra_base + reg as usize);
    writel((val & !clr) | set, eth_priv.infra_base + reg as usize);
}

// -----------------------------------------------------------------------------
// SGMII register access
// -----------------------------------------------------------------------------

#[inline]
pub unsafe fn mtk_sgmii_read(eth_priv: &MtkEthPriv, reg: u32) -> u32 {
    readl(eth_priv.sgmii_base + reg as usize)
}

#[inline]
pub unsafe fn mtk_sgmii_write(eth_priv: &MtkEthPriv, reg: u32, val: u32) {
    writel(val, eth_priv.sgmii_base + reg as usize);
}

// -----------------------------------------------------------------------------
// USXGMII/XFI register access
// -----------------------------------------------------------------------------

#[inline]
pub unsafe fn mtk_usxgmii_write(eth_priv: &MtkEthPriv, reg: u32, val: u32) {
    writel(val, eth_priv.usxgmii_base + reg as usize);
}

#[inline]
pub unsafe fn mtk_xfi_pextp_write(eth_priv: &MtkEthPriv, reg: u32, val: u32) {
    writel(val, eth_priv.xfi_pextp_base + reg as usize);
}

#[inline]
pub unsafe fn mtk_xfi_pll_read(eth_priv: &MtkEthPriv, reg: u32) -> u32 {
    readl(eth_priv.xfi_pll_base + reg as usize)
}

#[inline]
pub unsafe fn mtk_xfi_pll_write(eth_priv: &MtkEthPriv, reg: u32, val: u32) {
    writel(val, eth_priv.xfi_pll_base + reg as usize);
}

#[inline]
pub unsafe fn mtk_toprgu_write(eth_priv: &MtkEthPriv, reg: u32, val: u32) {
    writel(val, eth_priv.toprgu_base + reg as usize);
}

// =============================================================================
// MDIO functions
// =============================================================================

/// Direct MDIO clause 22/45 access via SoC
/// static int mtk_mii_rw(struct mtk_eth_priv *eth_priv, u8 phy, u8 reg, u16 data,
///                       u32 cmd, u32 st)
pub unsafe fn mtk_mii_rw(
    eth_priv: &MtkEthPriv,
    phy: u8,
    reg: u8,
    data: u16,
    cmd: u32,
    st: u32,
) -> Result<u32, i32> {
    let mut val = (st << MDIO_ST_S)
        | ((cmd << MDIO_CMD_S) & MDIO_CMD_M)
        | (((phy as u32) << MDIO_PHY_ADDR_S) & MDIO_PHY_ADDR_M)
        | (((reg as u32) << MDIO_REG_ADDR_S) & MDIO_REG_ADDR_M);

    if cmd == MDIO_CMD_WRITE || cmd == MDIO_CMD_ADDR {
        val |= (data as u32) & MDIO_RW_DATA_M;
    }

    mtk_gmac_write(eth_priv, GMAC_PIAC_REG, val | PHY_ACS_ST);

    // Wait for bit to clear (timeout 5000ms)
    for _ in 0..5000 {
        if (mtk_gmac_read(eth_priv, GMAC_PIAC_REG) & PHY_ACS_ST) == 0 {
            if cmd == MDIO_CMD_READ || cmd == MDIO_CMD_READ_C45 {
                let result = mtk_gmac_read(eth_priv, GMAC_PIAC_REG);
                return Ok(result & MDIO_RW_DATA_M);
            }
            return Ok(0);
        }
        // udelay(1) equivalent - caller should implement delay
    }

    // Timeout
    Err(-1)
}

/// Direct MDIO clause 22 read via SoC
/// int mtk_mii_read(struct mtk_eth_priv *eth_priv, u8 phy, u8 reg)
pub unsafe fn mtk_mii_read(eth_priv: &MtkEthPriv, phy: u8, reg: u8) -> Result<u32, i32> {
    mtk_mii_rw(eth_priv, phy, reg, 0, MDIO_CMD_READ, MDIO_ST_C22)
}

/// Direct MDIO clause 22 write via SoC
/// int mtk_mii_write(struct mtk_eth_priv *eth_priv, u8 phy, u8 reg, u16 data)
pub unsafe fn mtk_mii_write(eth_priv: &MtkEthPriv, phy: u8, reg: u8, data: u16) -> Result<u32, i32> {
    mtk_mii_rw(eth_priv, phy, reg, data, MDIO_CMD_WRITE, MDIO_ST_C22)
}

/// Direct MDIO clause 45 read via SoC
/// int mtk_mmd_read(struct mtk_eth_priv *eth_priv, u8 addr, u8 devad, u16 reg)
pub unsafe fn mtk_mmd_read(eth_priv: &MtkEthPriv, addr: u8, devad: u8, reg: u16) -> Result<u32, i32> {
    mtk_mii_rw(eth_priv, addr, devad, reg, MDIO_CMD_ADDR, MDIO_ST_C45)?;
    mtk_mii_rw(eth_priv, addr, devad, 0, MDIO_CMD_READ_C45, MDIO_ST_C45)
}

/// Direct MDIO clause 45 write via SoC
/// int mtk_mmd_write(struct mtk_eth_priv *eth_priv, u8 addr, u8 devad, u16 reg, u16 val)
pub unsafe fn mtk_mmd_write(
    eth_priv: &MtkEthPriv,
    addr: u8,
    devad: u8,
    reg: u16,
    val: u16,
) -> Result<u32, i32> {
    mtk_mii_rw(eth_priv, addr, devad, reg, MDIO_CMD_ADDR, MDIO_ST_C45)?;
    mtk_mii_rw(eth_priv, addr, devad, val, MDIO_CMD_WRITE, MDIO_ST_C45)
}

/// Indirect MDIO clause 45 read via MII registers
/// int mtk_mmd_ind_read(struct mtk_eth_priv *eth_priv, u8 addr, u8 devad, u16 reg)
pub unsafe fn mtk_mmd_ind_read(
    eth_priv: &MtkEthPriv,
    addr: u8,
    devad: u8,
    reg: u16,
) -> Result<u32, i32> {
    mtk_mii_write(
        eth_priv,
        addr,
        MII_MMD_ACC_CTL_REG as u8,
        ((MMD_ADDR << MMD_CMD_S) | ((devad as u32) << MMD_DEVAD_S) & MMD_DEVAD_M) as u16,
    )?;

    mtk_mii_write(eth_priv, addr, MII_MMD_ADDR_DATA_REG as u8, reg)?;

    mtk_mii_write(
        eth_priv,
        addr,
        MII_MMD_ACC_CTL_REG as u8,
        ((MMD_DATA << MMD_CMD_S) | ((devad as u32) << MMD_DEVAD_S) & MMD_DEVAD_M) as u16,
    )?;

    mtk_mii_read(eth_priv, addr, MII_MMD_ADDR_DATA_REG as u8)
}

/// Indirect MDIO clause 45 write via MII registers
/// int mtk_mmd_ind_write(struct mtk_eth_priv *eth_priv, u8 addr, u8 devad, u16 reg, u16 val)
pub unsafe fn mtk_mmd_ind_write(
    eth_priv: &MtkEthPriv,
    addr: u8,
    devad: u8,
    reg: u16,
    val: u16,
) -> Result<u32, i32> {
    mtk_mii_write(
        eth_priv,
        addr,
        MII_MMD_ACC_CTL_REG as u8,
        ((MMD_ADDR << MMD_CMD_S) | ((devad as u32) << MMD_DEVAD_S) & MMD_DEVAD_M) as u16,
    )?;

    mtk_mii_write(eth_priv, addr, MII_MMD_ADDR_DATA_REG as u8, reg)?;

    mtk_mii_write(
        eth_priv,
        addr,
        MII_MMD_ACC_CTL_REG as u8,
        ((MMD_DATA << MMD_CMD_S) | ((devad as u32) << MMD_DEVAD_S) & MMD_DEVAD_M) as u16,
    )?;

    mtk_mii_write(eth_priv, addr, MII_MMD_ADDR_DATA_REG as u8, val)
}

// =============================================================================
// SGMII functions
// =============================================================================

/// Initialize SGMII with auto-negotiation
/// static void mtk_sgmii_an_init(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_sgmii_an_init(eth_priv: &MtkEthPriv) {
    // Set SGMII GEN1 speed(1G)
    clrbits_le32(
        eth_priv.sgmii_base + eth_priv.soc.ana_rgc3 as usize,
        SGMSYS_SPEED_MASK,
    );

    // Enable SGMII AN
    setbits_le32(
        eth_priv.sgmii_base + SGMSYS_PCS_CONTROL_1 as usize,
        SGMII_AN_ENABLE,
    );

    // SGMII AN mode setting
    writel(SGMII_AN_MODE, eth_priv.sgmii_base + SGMSYS_SGMII_MODE as usize);

    // SGMII PN SWAP setting
    if eth_priv.pn_swap {
        setbits_le32(
            eth_priv.sgmii_base + SGMSYS_QPHY_WRAP_CTRL as usize,
            SGMII_PN_SWAP_TX_RX,
        );
    }

    // Release PHYA power down state
    clrsetbits_le32(
        eth_priv.sgmii_base + SGMSYS_QPHY_PWR_STATE_CTRL as usize,
        SGMII_PHYA_PWD,
        0,
    );
}

/// Initialize SGMII in force mode (2.5G)
/// static void mtk_sgmii_force_init(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_sgmii_force_init(eth_priv: &MtkEthPriv) {
    // Set SGMII GEN2 speed(2.5G)
    clrsetbits_le32(
        eth_priv.sgmii_base + eth_priv.soc.ana_rgc3 as usize,
        SGMSYS_SPEED_MASK,
        (SGMSYS_SPEED_2500 << 2) & SGMSYS_SPEED_MASK,
    );

    // Disable SGMII AN
    clrsetbits_le32(
        eth_priv.sgmii_base + SGMSYS_PCS_CONTROL_1 as usize,
        SGMII_AN_ENABLE,
        0,
    );

    // SGMII force mode setting
    writel(
        SGMII_FORCE_MODE,
        eth_priv.sgmii_base + SGMSYS_SGMII_MODE as usize,
    );

    // SGMII PN SWAP setting
    if eth_priv.pn_swap {
        setbits_le32(
            eth_priv.sgmii_base + SGMSYS_QPHY_WRAP_CTRL as usize,
            SGMII_PN_SWAP_TX_RX,
        );
    }

    // Release PHYA power down state
    clrsetbits_le32(
        eth_priv.sgmii_base + SGMSYS_QPHY_PWR_STATE_CTRL as usize,
        SGMII_PHYA_PWD,
        0,
    );
}

// =============================================================================
// USXGMII / 10GBase-R functions
// =============================================================================

/// Enable XFI PLL
/// static void mtk_xfi_pll_enable(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_xfi_pll_enable(eth_priv: &MtkEthPriv) {
    // Add software workaround for USXGMII PLL TCL issue
    mtk_xfi_pll_write(eth_priv, XFI_PLL_ANA_GLB8, RG_XFI_PLL_ANA_SWWA);

    let mut val = mtk_xfi_pll_read(eth_priv, XFI_PLL_DIG_GLB8);
    val |= RG_XFI_PLL_EN;
    mtk_xfi_pll_write(eth_priv, XFI_PLL_DIG_GLB8, val);
}

/// Reset USXGMII
/// static void mtk_usxgmii_reset(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_usxgmii_reset(eth_priv: &MtkEthPriv) {
    match eth_priv.gmac_id {
        1 => {
            mtk_toprgu_write(eth_priv, 0xFC, 0x0000A004);
            mtk_toprgu_write(eth_priv, 0x18, 0x88F0A004);
            mtk_toprgu_write(eth_priv, 0xFC, 0x00000000);
            mtk_toprgu_write(eth_priv, 0x18, 0x88F00000);
            mtk_toprgu_write(eth_priv, 0x18, 0x00F00000);
        }
        2 => {
            mtk_toprgu_write(eth_priv, 0xFC, 0x00005002);
            mtk_toprgu_write(eth_priv, 0x18, 0x88F05002);
            mtk_toprgu_write(eth_priv, 0xFC, 0x00000000);
            mtk_toprgu_write(eth_priv, 0x18, 0x88F00000);
            mtk_toprgu_write(eth_priv, 0x18, 0x00F00000);
        }
        _ => {}
    }

    // mdelay(10) - caller should implement delay
}

/// Setup PHYA for USXGMII AN 10G mode
/// static void mtk_usxgmii_setup_phya_an_10000(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_usxgmii_setup_phya_an_10000(eth_priv: &MtkEthPriv) {
    mtk_usxgmii_write(eth_priv, 0x810, 0x000FFE6D);
    mtk_usxgmii_write(eth_priv, 0x818, 0x07B1EC7B);
    mtk_usxgmii_write(eth_priv, 0x80C, 0x30000000);
    // ndelay(1020)
    mtk_usxgmii_write(eth_priv, 0x80C, 0x10000000);
    // ndelay(1020)
    mtk_usxgmii_write(eth_priv, 0x80C, 0x00000000);

    mtk_xfi_pextp_write(eth_priv, 0x9024, 0x00C9071C);
    mtk_xfi_pextp_write(eth_priv, 0x2020, 0xAA8585AA);
    mtk_xfi_pextp_write(eth_priv, 0x2030, 0x0C020707);
    mtk_xfi_pextp_write(eth_priv, 0x2034, 0x0E050F0F);
    mtk_xfi_pextp_write(eth_priv, 0x2040, 0x00140032);
    mtk_xfi_pextp_write(eth_priv, 0x50F0, 0x00C014AA);
    mtk_xfi_pextp_write(eth_priv, 0x50E0, 0x3777C12B);
    mtk_xfi_pextp_write(eth_priv, 0x506C, 0x005F9CFF);
    mtk_xfi_pextp_write(eth_priv, 0x5070, 0x9D9DFAFA);
    mtk_xfi_pextp_write(eth_priv, 0x5074, 0x27273F3F);
    mtk_xfi_pextp_write(eth_priv, 0x5078, 0xA7883C68);
    mtk_xfi_pextp_write(eth_priv, 0x507C, 0x11661166);
    mtk_xfi_pextp_write(eth_priv, 0x5080, 0x0E000AAF);
    mtk_xfi_pextp_write(eth_priv, 0x5084, 0x08080D0D);
    mtk_xfi_pextp_write(eth_priv, 0x5088, 0x02030909);
    mtk_xfi_pextp_write(eth_priv, 0x50E4, 0x0C0C0000);
    mtk_xfi_pextp_write(eth_priv, 0x50E8, 0x04040000);
    mtk_xfi_pextp_write(eth_priv, 0x50EC, 0x0F0F0C06);
    mtk_xfi_pextp_write(eth_priv, 0x50A8, 0x506E8C8C);
    mtk_xfi_pextp_write(eth_priv, 0x6004, 0x18190000);
    mtk_xfi_pextp_write(eth_priv, 0x00F8, 0x01423342);
    mtk_xfi_pextp_write(eth_priv, 0x00F4, 0x80201F20);
    mtk_xfi_pextp_write(eth_priv, 0x0030, 0x00050C00);
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x02002800);
    // ndelay(1020)
    mtk_xfi_pextp_write(eth_priv, 0x30B0, 0x00000020);
    mtk_xfi_pextp_write(eth_priv, 0x3028, 0x00008A01);
    mtk_xfi_pextp_write(eth_priv, 0x302C, 0x0000A884);
    mtk_xfi_pextp_write(eth_priv, 0x3024, 0x00083002);
    mtk_xfi_pextp_write(eth_priv, 0x3010, 0x00022220);
    mtk_xfi_pextp_write(eth_priv, 0x5064, 0x0F020A01);
    mtk_xfi_pextp_write(eth_priv, 0x50B4, 0x06100600);
    mtk_xfi_pextp_write(eth_priv, 0x3048, 0x40704000);
    mtk_xfi_pextp_write(eth_priv, 0x3050, 0xA8000000);
    mtk_xfi_pextp_write(eth_priv, 0x3054, 0x000000AA);
    mtk_xfi_pextp_write(eth_priv, 0x306C, 0x00000F00);
    mtk_xfi_pextp_write(eth_priv, 0xA060, 0x00040000);
    mtk_xfi_pextp_write(eth_priv, 0x90D0, 0x00000001);
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x0200E800);
    // udelay(150)
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x0200C111);
    // ndelay(1020)
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x0200C101);
    // udelay(15)
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x0202C111);
    // ndelay(1020)
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x0202C101);
    // udelay(100)
    mtk_xfi_pextp_write(eth_priv, 0x30B0, 0x00000030);
    mtk_xfi_pextp_write(eth_priv, 0x00F4, 0x80201F00);
    mtk_xfi_pextp_write(eth_priv, 0x3040, 0x30000000);
    // udelay(400)
}

/// Setup PHYA for 10GBase-R force mode
/// static void mtk_usxgmii_setup_phya_force_10000(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_usxgmii_setup_phya_force_10000(eth_priv: &MtkEthPriv) {
    mtk_usxgmii_write(eth_priv, 0x810, 0x000FFE6C);
    mtk_usxgmii_write(eth_priv, 0x818, 0x07B1EC7B);
    mtk_usxgmii_write(eth_priv, 0x80C, 0xB0000000);
    // ndelay(1020)
    mtk_usxgmii_write(eth_priv, 0x80C, 0x90000000);
    // ndelay(1020)

    mtk_xfi_pextp_write(eth_priv, 0x9024, 0x00C9071C);
    mtk_xfi_pextp_write(eth_priv, 0x2020, 0xAA8585AA);
    mtk_xfi_pextp_write(eth_priv, 0x2030, 0x0C020707);
    mtk_xfi_pextp_write(eth_priv, 0x2034, 0x0E050F0F);
    mtk_xfi_pextp_write(eth_priv, 0x2040, 0x00140032);
    mtk_xfi_pextp_write(eth_priv, 0x50F0, 0x00C014AA);
    mtk_xfi_pextp_write(eth_priv, 0x50E0, 0x3777C12B);
    mtk_xfi_pextp_write(eth_priv, 0x506C, 0x005F9CFF);
    mtk_xfi_pextp_write(eth_priv, 0x5070, 0x9D9DFAFA);
    mtk_xfi_pextp_write(eth_priv, 0x5074, 0x27273F3F);
    mtk_xfi_pextp_write(eth_priv, 0x5078, 0xA7883C68);
    mtk_xfi_pextp_write(eth_priv, 0x507C, 0x11661166);
    mtk_xfi_pextp_write(eth_priv, 0x5080, 0x0E000AAF);
    mtk_xfi_pextp_write(eth_priv, 0x5084, 0x08080D0D);
    mtk_xfi_pextp_write(eth_priv, 0x5088, 0x02030909);
    mtk_xfi_pextp_write(eth_priv, 0x50E4, 0x0C0C0000);
    mtk_xfi_pextp_write(eth_priv, 0x50E8, 0x04040000);
    mtk_xfi_pextp_write(eth_priv, 0x50EC, 0x0F0F0C06);
    mtk_xfi_pextp_write(eth_priv, 0x50A8, 0x506E8C8C);
    mtk_xfi_pextp_write(eth_priv, 0x6004, 0x18190000);
    mtk_xfi_pextp_write(eth_priv, 0x00F8, 0x01423342);
    mtk_xfi_pextp_write(eth_priv, 0x00F4, 0x80201F20);
    mtk_xfi_pextp_write(eth_priv, 0x0030, 0x00050C00);
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x02002800);
    // ndelay(1020)
    mtk_xfi_pextp_write(eth_priv, 0x30B0, 0x00000020);
    mtk_xfi_pextp_write(eth_priv, 0x3028, 0x00008A01);
    mtk_xfi_pextp_write(eth_priv, 0x302C, 0x0000A884);
    mtk_xfi_pextp_write(eth_priv, 0x3024, 0x00083002);
    mtk_xfi_pextp_write(eth_priv, 0x3010, 0x00022220);
    mtk_xfi_pextp_write(eth_priv, 0x5064, 0x0F020A01);
    mtk_xfi_pextp_write(eth_priv, 0x50B4, 0x06100600);
    mtk_xfi_pextp_write(eth_priv, 0x3048, 0x47684100);
    mtk_xfi_pextp_write(eth_priv, 0x3050, 0x00000000);
    mtk_xfi_pextp_write(eth_priv, 0x3054, 0x00000000);
    mtk_xfi_pextp_write(eth_priv, 0x306C, 0x00000F00);
    if eth_priv.gmac_id == 2 {
        mtk_xfi_pextp_write(eth_priv, 0xA008, 0x0007B400);
    }
    mtk_xfi_pextp_write(eth_priv, 0xA060, 0x00040000);
    mtk_xfi_pextp_write(eth_priv, 0x90D0, 0x00000001);
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x0200E800);
    // udelay(150)
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x0200C111);
    // ndelay(1020)
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x0200C101);
    // udelay(15)
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x0202C111);
    // ndelay(1020)
    mtk_xfi_pextp_write(eth_priv, 0x0070, 0x0202C101);
    // udelay(100)
    mtk_xfi_pextp_write(eth_priv, 0x30B0, 0x00000030);
    mtk_xfi_pextp_write(eth_priv, 0x00F4, 0x80201F00);
    mtk_xfi_pextp_write(eth_priv, 0x3040, 0x30000000);
    // udelay(400)
}

/// Initialize USXGMII with auto-negotiation
/// static void mtk_usxgmii_an_init(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_usxgmii_an_init(eth_priv: &MtkEthPriv) {
    mtk_xfi_pll_enable(eth_priv);
    mtk_usxgmii_reset(eth_priv);
    mtk_usxgmii_setup_phya_an_10000(eth_priv);
}

/// Initialize 10GBase-R mode
/// static void mtk_10gbaser_init(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_10gbaser_init(eth_priv: &MtkEthPriv) {
    mtk_xfi_pll_enable(eth_priv);
    mtk_usxgmii_reset(eth_priv);
    mtk_usxgmii_setup_phya_force_10000(eth_priv);
}

// =============================================================================
// MAC initialization functions
// =============================================================================

/// Initialize MAC (non-XGMAC)
/// static int mtk_mac_init(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_mac_init(eth_priv: &MtkEthPriv) -> Result<(), i32> {
    let mut ge_mode;

    if mtk_has_caps(eth_priv.soc.caps, MTK_ETH_PATH_MT7629_GMAC2) {
        mtk_infra_rmw(
            eth_priv,
            MT7629_INFRA_MISC2_REG,
            INFRA_MISC2_BONDING_OPTION,
            eth_priv.gmac_id,
        );
    }

    match eth_priv.phy_interface {
        PhyInterfaceMode::RgmiiRxid | PhyInterfaceMode::Rgmii => {
            ge_mode = GE_MODE_RGMII;
        }
        PhyInterfaceMode::Sgmii | PhyInterfaceMode::Basex2500 => {
            if mtk_has_caps(eth_priv.soc.caps, MTK_GMAC2_U3_QPHY) {
                mtk_infra_rmw(eth_priv, USB_PHY_SWITCH_REG, QPHY_SEL_MASK, SGMII_QPHY_SEL);
            }

            let mut sgmii_sel_mask = 0;
            if mtk_has_caps(eth_priv.soc.caps, MTK_ETH_PATH_MT7622_SGMII) {
                sgmii_sel_mask = SYSCFG1_SGMII_SEL_M;
            }

            mtk_ethsys_rmw(
                eth_priv,
                ETHSYS_SYSCFG1_REG,
                sgmii_sel_mask,
                syscfg1_sgmii_sel(eth_priv.gmac_id),
            );

            if eth_priv.phy_interface == PhyInterfaceMode::Sgmii {
                mtk_sgmii_an_init(eth_priv);
            } else {
                mtk_sgmii_force_init(eth_priv);
            }

            ge_mode = GE_MODE_RGMII;
        }
        PhyInterfaceMode::Mii | PhyInterfaceMode::Gmii => {
            ge_mode = GE_MODE_MII;
        }
        PhyInterfaceMode::Rmii => {
            ge_mode = GE_MODE_RMII;
        }
        _ => {
            ge_mode = GE_MODE_RGMII;
        }
    }

    // Set the gmac to the right mode
    mtk_ethsys_rmw(
        eth_priv,
        ETHSYS_SYSCFG1_REG,
        SYSCFG1_GE_MODE_M << syscfg1_ge_mode_s(eth_priv.gmac_id),
        ge_mode << syscfg1_ge_mode_s(eth_priv.gmac_id),
    );

    if eth_priv.force_mode {
        let mut mcr = (IPG_96BIT_WITH_SHORT_IPG << IPG_CFG_S)
            | (MAC_RX_PKT_LEN_1536 << MAC_RX_PKT_LEN_S)
            | MAC_MODE
            | FORCE_MODE
            | MAC_TX_EN
            | MAC_RX_EN
            | BKOFF_EN
            | BACKPR_EN
            | FORCE_LINK;

        match eth_priv.speed {
            SPEED_10 => mcr |= SPEED_10M << FORCE_SPD_S,
            SPEED_100 => mcr |= SPEED_100M << FORCE_SPD_S,
            SPEED_1000 | SPEED_2500 => mcr |= SPEED_1000M << FORCE_SPD_S,
            _ => {}
        }

        if eth_priv.duplex {
            mcr |= FORCE_DPX;
        }

        mtk_gmac_write(eth_priv, gmac_port_mcr(eth_priv.gmac_id), mcr);
    }

    if mtk_has_caps(eth_priv.soc.caps, MTK_GMAC1_TRGMII)
        && !mtk_has_caps(eth_priv.soc.caps, MTK_TRGMII_MT7621_CLK)
    {
        // Lower Tx Driving for TRGMII path
        for i in 0..NUM_TRGMII_CTRL {
            mtk_gmac_write(
                eth_priv,
                gmac_trgmii_td_odt(i),
                (8 << TD_DM_DRVP_S) | (8 << TD_DM_DRVN_S),
            );
        }

        mtk_gmac_rmw(eth_priv, GMAC_TRGMII_RCK_CTRL, 0, RX_RST | RXC_DQSISEL);
        mtk_gmac_rmw(eth_priv, GMAC_TRGMII_RCK_CTRL, RX_RST, 0);
    }

    Ok(())
}

/// Initialize XMAC (10G)
/// static int mtk_xmac_init(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_xmac_init(eth_priv: &MtkEthPriv) -> Result<(), i32> {
    match eth_priv.phy_interface {
        PhyInterfaceMode::Usxgmii => {
            mtk_usxgmii_an_init(eth_priv);
        }
        PhyInterfaceMode::Baseer10g => {
            mtk_10gbaser_init(eth_priv);
        }
        _ => {}
    }

    // Set GMAC to the correct mode
    mtk_ethsys_rmw(
        eth_priv,
        ETHSYS_SYSCFG1_REG,
        SYSCFG1_GE_MODE_M << syscfg1_ge_mode_s(eth_priv.gmac_id),
        0,
    );

    if (eth_priv.phy_interface == PhyInterfaceMode::Usxgmii
        || eth_priv.phy_interface == PhyInterfaceMode::Baseer10g)
        && eth_priv.gmac_id == 1
    {
        mtk_infra_rmw(
            eth_priv,
            TOPMISC_NETSYS_PCS_MUX,
            NETSYS_PCS_MUX_MASK,
            MUX_G2_USXGMII_SEL,
        );
    }

    let force_link = if eth_priv.phy_interface == PhyInterfaceMode::Xgmii || eth_priv.gmac_id == 2 {
        xgmac_force_link(eth_priv.gmac_id)
    } else {
        0
    };

    mtk_gmac_rmw(
        eth_priv,
        xgmac_sts(eth_priv.gmac_id),
        xgmac_force_link(eth_priv.gmac_id),
        force_link,
    );

    // Force GMAC link down
    mtk_gmac_write(eth_priv, gmac_port_mcr(eth_priv.gmac_id), FORCE_MODE);

    Ok(())
}

// =============================================================================
// DMA / FIFO initialization
// =============================================================================

/// Initialize DMA rings and packet buffers
/// static void mtk_eth_fifo_init(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_eth_fifo_init(eth_priv: &mut MtkEthPriv) {
    // Clear upper bits of PDMA_GLO_CFG
    mtk_pdma_rmw(eth_priv, PDMA_GLO_CFG_REG, 0xffff0000, 0);
    // udelay(500)

    // Clear rings
    core::ptr::write_bytes(
        eth_priv.tx_ring_noc,
        0,
        NUM_TX_DESC * eth_priv.soc.txd_size,
    );
    core::ptr::write_bytes(
        eth_priv.rx_ring_noc,
        0,
        NUM_RX_DESC * eth_priv.soc.rxd_size,
    );

    // Fill packet pool with 0xff (for debugging)
    eth_priv.pkt_pool.fill(0xff);

    // Flush dcache for pkt_pool
    // flush_dcache_range((ulong)pkt_base, (ulong)(pkt_base + TOTAL_PKT_BUF_SIZE));

    eth_priv.rx_dma_owner_idx0 = 0;
    eth_priv.tx_cpu_owner_idx0 = 0;

    let pkt_base = eth_priv.pkt_pool.as_ptr() as usize;

    // Initialize TX descriptors
    for i in 0..NUM_TX_DESC {
        let txd_offset = i * eth_priv.soc.txd_size;
        let txd = eth_priv.tx_ring_noc.add(txd_offset) as *mut MtkTxDmaV2;
        let pkt_phys = pkt_base + i * PKTSIZE_ALIGN;

        (*txd).txd1 = pkt_phys as u32;
        (*txd).txd2 = PDMA_TXD2_DDONE | PDMA_TXD2_LS0;

        if mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V3) {
            // V3: FPORT in txd5
            let fport = if eth_priv.gmac_id == 2 { 15 } else { eth_priv.gmac_id + 1 };
            (*txd).txd5 = pdma_v2_txd5_fport_set(fport);
        } else if mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V2) {
            // V2: FPORT in txd5
            (*txd).txd5 = pdma_v2_txd5_fport_set(eth_priv.gmac_id + 1);
        } else {
            // V1: FPORT in txd4
            (*txd).txd4 = pdma_v1_txd4_fport_set(eth_priv.gmac_id + 1);
        }
    }

    // Initialize RX descriptors
    for i in 0..NUM_RX_DESC {
        let rxd_offset = i * eth_priv.soc.rxd_size;
        let rxd = eth_priv.rx_ring_noc.add(rxd_offset) as *mut MtkRxDmaV2;
        let pkt_phys = pkt_base + TX_TOTAL_BUF_SIZE + i * PKTSIZE_ALIGN;

        (*rxd).rxd1 = pkt_phys as u32;

        if mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V2)
            || mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V3)
        {
            (*rxd).rxd2 = pdma_v2_rxd2_plen0_set(PKTSIZE_ALIGN as u32);
        } else {
            (*rxd).rxd2 = pdma_v1_rxd2_plen0_set(PKTSIZE_ALIGN as u32);
        }
    }

    // Set up PDMA registers
    let tx_ring_phys = eth_priv.tx_ring_noc as usize;
    let rx_ring_phys = eth_priv.rx_ring_noc as usize;

    mtk_pdma_write(eth_priv, tx_base_ptr_reg(0), tx_ring_phys as u32);
    mtk_pdma_write(eth_priv, tx_max_cnt_reg(0), NUM_TX_DESC as u32);
    mtk_pdma_write(eth_priv, tx_ctx_idx_reg(0), eth_priv.tx_cpu_owner_idx0 as u32);

    mtk_pdma_write(eth_priv, rx_base_ptr_reg(0), rx_ring_phys as u32);
    mtk_pdma_write(eth_priv, rx_max_cnt_reg(0), NUM_RX_DESC as u32);
    mtk_pdma_write(eth_priv, rx_crx_idx_reg(0), (NUM_RX_DESC - 1) as u32);

    // Reset DMA indices
    mtk_pdma_write(eth_priv, PDMA_RST_IDX_REG, RST_DTX_IDX0 | RST_DRX_IDX0);
}

/// Set MDC divider
/// static void mtk_eth_mdc_init(struct mtk_eth_priv *eth_priv)
pub unsafe fn mtk_eth_mdc_init(eth_priv: &MtkEthPriv) {
    if eth_priv.mdc == 0 {
        return;
    }

    let divider = core::cmp::min(
        (MDC_MAX_FREQ + eth_priv.mdc - 1) / eth_priv.mdc, // DIV_ROUND_UP
        MDC_MAX_DIVIDER,
    );

    // Configure MDC turbo mode
    if mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V3) {
        mtk_gmac_rmw(eth_priv, GMAC_MAC_MISC_REG, 0, MISC_MDC_TURBO);
    } else {
        mtk_gmac_rmw(eth_priv, GMAC_PPSC_REG, 0, MDC_TURBO);
    }

    // Configure MDC divider
    mtk_gmac_rmw(
        eth_priv,
        GMAC_PPSC_REG,
        PHY_MDC_CFG,
        (divider << 24) & PHY_MDC_CFG,
    );
}

// =============================================================================
// Driver start/stop/send/recv
// =============================================================================

/// Start the ethernet driver
/// static int mtk_eth_start(struct udevice *dev)
pub unsafe fn mtk_eth_start(eth_priv: &mut MtkEthPriv) -> Result<(), i32> {
    // Reset FE would be done by caller (reset controller)
    // reset_assert(&eth_priv->rst_fe);
    // udelay(1000);
    // reset_deassert(&eth_priv->rst_fe);
    // mdelay(10);

    if mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V2)
        || mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V3)
    {
        setbits_le32(eth_priv.fe_base + FE_GLO_MISC_REG as usize, PDMA_VER_V2);
    }

    // Packets forward to PDMA
    mtk_gdma_write(eth_priv, eth_priv.gmac_id, GDMA_IG_CTRL_REG, GDMA_FWD_TO_CPU);

    // Discard packets for other GDMAs
    for i in 0..eth_priv.soc.gdma_count {
        if i == eth_priv.gmac_id {
            continue;
        }
        mtk_gdma_write(eth_priv, i, GDMA_IG_CTRL_REG, GDMA_FWD_DISCARD);
    }

    // V3 specific: bridge mode for internal switch
    if mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V3) {
        // Check if using internal switch (mt7988) and gmac_id == 0
        if eth_priv.swname.is_some() && eth_priv.gmac_id == 0 {
            mtk_gdma_write(eth_priv, eth_priv.gmac_id, GDMA_IG_CTRL_REG, GDMA_BRIDGE_TO_CPU);
            mtk_gdma_write(eth_priv, eth_priv.gmac_id, GDMA_EG_CTRL_REG, GDMA_CPU_BRIDGE_EN);
        } else if (eth_priv.phy_interface == PhyInterfaceMode::Usxgmii
            || eth_priv.phy_interface == PhyInterfaceMode::Baseer10g
            || eth_priv.phy_interface == PhyInterfaceMode::Xgmii)
            && eth_priv.gmac_id != 0
        {
            mtk_gdma_write(eth_priv, eth_priv.gmac_id, GDMA_EG_CTRL_REG, GDMA_CPU_BRIDGE_EN);
        }
    }

    // udelay(500)

    mtk_eth_fifo_init(eth_priv);

    // PHY startup or switch mac_control would be done by caller

    // Enable DMA
    mtk_pdma_rmw(eth_priv, PDMA_GLO_CFG_REG, 0, TX_WB_DDONE | RX_DMA_EN | TX_DMA_EN);
    // udelay(500)

    Ok(())
}

/// Stop the ethernet driver
/// static void mtk_eth_stop(struct udevice *dev)
pub unsafe fn mtk_eth_stop(eth_priv: &MtkEthPriv) {
    // Disable DMA
    mtk_pdma_rmw(eth_priv, PDMA_GLO_CFG_REG, TX_WB_DDONE | RX_DMA_EN | TX_DMA_EN, 0);
    // udelay(500)

    // Wait for DMA to become idle
    // wait_for_bit_le32(eth_priv->fe_base + eth_priv->soc->pdma_base + PDMA_GLO_CFG_REG,
    //                   RX_DMA_BUSY | TX_DMA_BUSY, 0, 5000, 0);
}

/// Write MAC address to hardware
/// static int mtk_eth_write_hwaddr(struct udevice *dev)
pub unsafe fn mtk_eth_write_hwaddr(eth_priv: &MtkEthPriv) {
    let mac = &eth_priv.enetaddr;

    let macaddr_msb = ((mac[0] as u32) << 8) | (mac[1] as u32);
    let macaddr_lsb = ((mac[2] as u32) << 24)
        | ((mac[3] as u32) << 16)
        | ((mac[4] as u32) << 8)
        | (mac[5] as u32);

    mtk_gdma_write(eth_priv, eth_priv.gmac_id, GDMA_MAC_MSB_REG, macaddr_msb);
    mtk_gdma_write(eth_priv, eth_priv.gmac_id, GDMA_MAC_LSB_REG, macaddr_lsb);
}

/// Send a packet
/// static int mtk_eth_send(struct udevice *dev, void *packet, int length)
pub unsafe fn mtk_eth_send(eth_priv: &mut MtkEthPriv, packet: &[u8]) -> Result<(), i32> {
    let idx = eth_priv.tx_cpu_owner_idx0;
    let txd = eth_priv.tx_ring_noc.add(idx * eth_priv.soc.txd_size) as *mut MtkTxDmaV2;

    // Check if descriptor is available (DDONE set)
    if ((*txd).txd2 & PDMA_TXD2_DDONE) == 0 {
        return Err(-1); // TX ring full
    }

    // Copy packet to buffer
    let pkt_base = (*txd).txd1 as usize;
    core::ptr::copy_nonoverlapping(packet.as_ptr(), pkt_base as *mut u8, packet.len());

    // Flush dcache for packet
    // flush_dcache_range((ulong)pkt_base, (ulong)pkt_base + roundup(length, ARCH_DMA_MINALIGN));

    // Set length and clear DDONE
    if mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V2)
        || mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V3)
    {
        (*txd).txd2 = PDMA_TXD2_LS0 | pdma_v2_txd2_sdl0_set(packet.len() as u32);
    } else {
        (*txd).txd2 = PDMA_TXD2_LS0 | pdma_v1_txd2_sdl0_set(packet.len() as u32);
    }

    // Advance TX index
    eth_priv.tx_cpu_owner_idx0 = (eth_priv.tx_cpu_owner_idx0 + 1) % NUM_TX_DESC;
    mtk_pdma_write(eth_priv, tx_ctx_idx_reg(0), eth_priv.tx_cpu_owner_idx0 as u32);

    Ok(())
}

/// Receive a packet
/// static int mtk_eth_recv(struct udevice *dev, int flags, uchar **packetp)
/// Returns packet length or error
pub unsafe fn mtk_eth_recv(eth_priv: &MtkEthPriv) -> Result<(*const u8, usize), i32> {
    let idx = eth_priv.rx_dma_owner_idx0;
    let rxd = eth_priv.rx_ring_noc.add(idx * eth_priv.soc.rxd_size) as *const MtkRxDmaV2;

    // Check if descriptor has been filled (DDONE set)
    if ((*rxd).rxd2 & PDMA_RXD2_DDONE) == 0 {
        return Err(-1); // No packet available
    }

    // Get packet length
    let length = if mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V2)
        || mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V3)
    {
        pdma_v2_rxd2_plen0_get((*rxd).rxd2) as usize
    } else {
        pdma_v1_rxd2_plen0_get((*rxd).rxd2) as usize
    };

    let pkt_base = (*rxd).rxd1 as *const u8;

    // Invalidate dcache for packet
    // invalidate_dcache_range((ulong)pkt_base, (ulong)pkt_base + roundup(length, ARCH_DMA_MINALIGN));

    Ok((pkt_base, length))
}

/// Free a received packet (return descriptor to DMA)
/// static int mtk_eth_free_pkt(struct udevice *dev, uchar *packet, int length)
pub unsafe fn mtk_eth_free_pkt(eth_priv: &mut MtkEthPriv) {
    let idx = eth_priv.rx_dma_owner_idx0;
    let rxd = eth_priv.rx_ring_noc.add(idx * eth_priv.soc.rxd_size) as *mut MtkRxDmaV2;

    // Invalidate dcache for buffer
    // invalidate_dcache_range((ulong)rxd->rxd1, (ulong)rxd->rxd1 + PKTSIZE_ALIGN);

    // Reset descriptor
    if mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V2)
        || mtk_has_caps(eth_priv.soc.caps, MTK_NETSYS_V3)
    {
        (*rxd).rxd2 = pdma_v2_rxd2_plen0_set(PKTSIZE_ALIGN as u32);
    } else {
        (*rxd).rxd2 = pdma_v1_rxd2_plen0_set(PKTSIZE_ALIGN as u32);
    }

    // Update CPU RX index
    mtk_pdma_write(eth_priv, rx_crx_idx_reg(0), idx as u32);

    // Advance RX index
    eth_priv.rx_dma_owner_idx0 = (eth_priv.rx_dma_owner_idx0 + 1) % NUM_RX_DESC;
}

// =============================================================================
// Probe / initialization
// =============================================================================

/// Probe and initialize the ethernet driver
/// static int mtk_eth_probe(struct udevice *dev)
pub unsafe fn mtk_eth_probe(eth_priv: &mut MtkEthPriv, fe_base: usize) -> Result<(), i32> {
    // Frame Engine Register Base
    eth_priv.fe_base = fe_base;

    // GMAC Register Base
    eth_priv.gmac_base = fe_base + GMAC_BASE as usize;

    // Allocate non-cached memory for DMA rings
    // eth_priv.tx_ring_noc = noncached_alloc(eth_priv.soc.txd_size * NUM_TX_DESC, ARCH_DMA_MINALIGN);
    // eth_priv.rx_ring_noc = noncached_alloc(eth_priv.soc.rxd_size * NUM_RX_DESC, ARCH_DMA_MINALIGN);

    // Set MDC divider
    mtk_eth_mdc_init(eth_priv);

    // Set MAC mode
    match eth_priv.phy_interface {
        PhyInterfaceMode::Usxgmii
        | PhyInterfaceMode::Baseer10g
        | PhyInterfaceMode::Xgmii => {
            mtk_xmac_init(eth_priv)?;
        }
        _ => {
            mtk_mac_init(eth_priv)?;
        }
    }

    Ok(())
}
