//! MT7988 Native Ethernet Driver
//!
//! GMAC1 + Internal MT7530 Switch driver using the bus framework with DataPort
//! integration for zero-copy packet I/O. Auto-spawned by devd when platform
//! registers an eth0 bus port.
//!
//! Architecture:
//! - Platform registers /kernel/bus/eth0 at boot
//! - devd connects to claim ownership, spawns ethd with MMIO metadata
//! - ethd initializes GMAC1/QDMA/PDMA, creates DataPort for packet I/O
//! - TX: dequeues DataPort SQ, fills QDMA descriptors, kicks hardware
//! - RX: timer-polled PDMA used ring, posts to DataPort CQ
//!
//! Hardware:
//! - GMAC1 connects to internal MT7530 switch (4x RJ45 GbE ports)
//! - No external PHY/MDIO needed — force mode for internal switch
//! - QDMA for TX (hardware queue 0)
//! - PDMA for RX (ring 0)

#![no_std]
#![no_main]

use userlib::syscall::Handle;
use userlib::mmio::{MmioRegion, DmaPool, delay_us};
use userlib::ipc::Timer;
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId,
    BlockPortConfig, bus_msg,
};
use userlib::bus_runtime::driver_main;
use userlib::ring::{IoSqe, IoCqe, io_op, io_status, side_msg, side_status, SideEntry};
use userlib::devd::PortType;
use userlib::{uinfo, uerror, uwarn};

// =============================================================================
// MT7988 Frame Engine Register Definitions
// =============================================================================

/// Frame Engine base address
const FE_BASE: u64 = 0x1510_0000;

/// Frame Engine size (512KB)
const FE_SIZE: u64 = 0x8_0000;

/// Internal Switch (GSW) base address (MT7531 in MT7988)
/// Accessed via memory-mapped I/O, NOT MDIO
const GSW_BASE: u64 = 0x1502_0000;

/// GSW size (32KB)
const GSW_SIZE: u64 = 0x8000;

/// ETHSYS base address (Ethernet System Control - clock/reset)
/// CRITICAL: This is SEPARATE from FE_BASE! DMA_AG_MAP lives here.
const ETHSYS_BASE: u64 = 0x1500_0000;

/// ETHSYS size (4KB is enough for our registers)
const ETHSYS_SIZE: u64 = 0x1000;

/// Register offsets from FE_BASE
mod fe_regs {
    // Frame Engine global registers
    pub const FE_GLO_CFG: usize = 0x0000;
    pub const FE_RST_GL: usize = 0x0004;
    pub const FE_INT_STATUS: usize = 0x0020;
    pub const FE_INT_ENABLE: usize = 0x0028;

    // FE global misc (NETSYSv3)
    pub const FE_GLO_MISC: usize = 0x0124;
    pub const FE_GLO_MISC_PDMA_V2: u32 = 1 << 4;  // Enable PDMAv2 mode

    // Reset bits
    pub const RST_FE: u32 = 1 << 31;

    // PSE (Packet Scheduling Engine) registers - required for V3!
    pub const PSE_DROP_CFG: usize = 0x0108;
    pub const PSE_DUMY_REQ: usize = 0x010c;
    pub const PSE_PPE_DROP0: usize = 0x0110;
    pub const PSE_PPE_DROP1: usize = 0x0114;
    pub const PSE_PPE_DROP2: usize = 0x0118;
    pub const PSE_FQFC_CFG2: usize = 0x0104;  // Free queue flow control config 2
    // PSE_IQ_REV(x) = 0x140 + (x-1)*4 - Input Queue Reservation
    pub const PSE_IQ_REV1: usize = 0x0140;  // 0x140 + 0*4
    pub const PSE_IQ_REV2: usize = 0x0144;  // 0x140 + 1*4
    pub const PSE_IQ_REV3: usize = 0x0148;  // 0x140 + 2*4
    pub const PSE_IQ_REV4: usize = 0x014c;  // 0x140 + 3*4
    pub const PSE_IQ_REV5: usize = 0x0150;  // 0x140 + 4*4
    pub const PSE_IQ_REV6: usize = 0x0154;  // 0x140 + 5*4
    pub const PSE_IQ_REV7: usize = 0x0158;  // 0x140 + 6*4
    pub const PSE_IQ_REV8: usize = 0x015c;  // 0x140 + 7*4
    // PSE_OQ_TH(x) = 0x160 + (x-1)*4 - Output Queue Threshold
    pub const PSE_OQ_TH1: usize = 0x0160;  // 0x160 + 0*4
    pub const PSE_OQ_TH2: usize = 0x0164;  // 0x160 + 1*4
    pub const PSE_OQ_TH3: usize = 0x0168;  // 0x160 + 2*4
    pub const PSE_OQ_TH4: usize = 0x016c;  // 0x160 + 3*4
    pub const PSE_OQ_TH5: usize = 0x0170;  // 0x160 + 4*4
    pub const PSE_OQ_TH6: usize = 0x0174;  // 0x160 + 5*4
    pub const PSE_OQ_TH7: usize = 0x0178;  // 0x160 + 6*4
    pub const PSE_OQ_TH8: usize = 0x017c;  // 0x160 + 7*4

    // PSE Dummy Page bits
    pub const PSE_DUMMY_WORK_GDM1: u32 = 1 << 17;
    pub const PSE_DUMMY_WORK_GDM2: u32 = 1 << 18;
    pub const PSE_DUMMY_WORK_GDM3: u32 = 1 << 19;
    pub const PSE_DUMMY_PAGE_THR: u32 = 1;

    // GDM and CDM thresholds (V3) - Linux mtk_hw_init NETSYS_V3 section
    pub const MTK_GDM2_THRES: usize = 0x1648;
    pub const MTK_CDMW0_THRES: usize = 0x164c;
    pub const MTK_CDMW1_THRES: usize = 0x1650;
    pub const MTK_CDME0_THRES: usize = 0x1654;
    pub const MTK_CDME1_THRES: usize = 0x1658;
    pub const MTK_CDMM_THRES: usize = 0x165c;

    // DMA address generation map (confusingly named ETHSYS in Linux but at FE_BASE!)
    // CRITICAL for DMA to work on MT7988/NETSYS_V3
    pub const DMA_AG_MAP: usize = 0x0408;
    pub const DMA_AG_MAP_PDMA: u32 = 1 << 0;       // Enable PDMA address generation
    pub const DMA_AG_MAP_QDMA: u32 = 1 << 1;       // Enable QDMA address generation
    pub const DMA_AG_MAP_PPE: u32 = 1 << 2;        // Enable PPE address generation
    pub const DMA_AG_MAP_PPE1_BYPASS: u32 = 1 << 3; // Bypass PPE1 for NETSYS_V3

    // CDMQ ingress control (for V3 CDM)
    pub const MTK_CDMQ_IG_CTRL: usize = 0x1400;
    pub const MTK_CDMQ_STAG_EN: u32 = 1 << 0;

    // Interrupt grouping registers (from Linux mtk_hw_init)
    // Even for polling mode, these may need to be configured
    pub const FE_INT_GRP: usize = 0x0020;           // FE interrupt grouping
    pub const TX_IRQ_MASK: usize = 0x461c;          // QDMA TX IRQ mask
    pub const PDMA_IRQ_MASK: usize = 0x6a28;        // PDMA RX IRQ mask
    pub const PDMA_INT_GRP0: usize = 0x6a50;        // PDMA interrupt group 0
    pub const PDMA_INT_GRP1: usize = 0x6a54;        // PDMA interrupt group 1
    pub const QDMA_INT_GRP0: usize = 0x4620;        // QDMA interrupt group 0
    pub const QDMA_INT_GRP1: usize = 0x4624;        // QDMA interrupt group 1

    // Interrupt grouping values (from Linux)
    pub const TX_DONE_INT: u32 = 1 << 28;           // MTK_TX_DONE_INT
    pub const RX_DONE_INT_V2: u32 = 1 << 14;        // RX done mask for V2
    pub const FE_INT_GRP_VALUE: u32 = 0x21021000;   // FE interrupt grouping value
}

/// ETHSYS registers (at ETHSYS_BASE = 0x15000000)
/// These control clock/reset and DMA address generation
mod ethsys_regs {
    /// Clock gate control register
    /// From clk-mt7988-eth.c: ethdma_cg_regs at offset 0x30
    /// Write 1 to enable clock, 0 to disable (inverted in hardware)
    pub const CLK_GATE: usize = 0x30;
    pub const CLK_FE_EN: u32 = 1 << 6;      // Frame Engine clock
    pub const CLK_GP2_EN: u32 = 1 << 7;     // GMAC Port 2 clock
    pub const CLK_GP1_EN: u32 = 1 << 8;     // GMAC Port 1 clock
    pub const CLK_GP3_EN: u32 = 1 << 10;    // GMAC Port 3 clock
    pub const CLK_ESW_EN: u32 = 1 << 16;    // Ethernet Switch clock

    /// Reset idle check enable - must disable before reset, re-enable after
    pub const FE_RST_CHK_IDLE_EN: usize = 0x28;

    /// Reset control register
    pub const RSTCTRL: usize = 0x34;

    /// DMA Address Generation Map - CRITICAL for DMA!
    /// This is at ETHSYS_BASE + 0x408 (confirmed from Linux regmap_update_bits(eth->ethsys, ...))
    pub const DMA_AG_MAP: usize = 0x408;
    pub const DMA_AG_MAP_PDMA: u32 = 1 << 0;   // Enable PDMA address generation
    pub const DMA_AG_MAP_QDMA: u32 = 1 << 1;   // Enable QDMA address generation
    pub const DMA_AG_MAP_PPE: u32 = 1 << 2;    // Enable PPE address generation

    // RSTCTRL bits (for V3 / MT7988)
    pub const RSTCTRL_FE: u32 = 1 << 6;        // Frame Engine reset
    pub const RSTCTRL_ETH: u32 = 1 << 23;      // Ethernet reset
    pub const RSTCTRL_PPE0_V3: u32 = 1 << 29;  // PPE0 reset (V3)
    pub const RSTCTRL_PPE1_V3: u32 = 1 << 30;  // PPE1 reset (V3)
    pub const RSTCTRL_WDMA0: u32 = 1 << 24;    // WDMA0 reset
    pub const RSTCTRL_WDMA1: u32 = 1 << 25;    // WDMA1 reset
    pub const RSTCTRL_WDMA2: u32 = 1 << 26;    // WDMA2 reset
}

/// GDM (GMAC Data Module) registers — forwards packets to correct DMA
mod gdm_regs {
    /// GDM1 forward config — controls where RX packets go
    /// Offset from FE_BASE
    pub const GDM1_FWD_CFG: usize = 0x0500;

    /// Forward to PDMA (value 0 = PDMA)
    pub const GDM_TO_PDMA: u32 = 0;

    /// Enable IP/TCP/UDP checksum generation for TX
    pub const GDM_ICS_EN: u32 = 1 << 22;
    pub const GDM_TCS_EN: u32 = 1 << 21;
    pub const GDM_UCS_EN: u32 = 1 << 20;
}

/// MAC registers (GMAC1)
mod mac_regs {
    /// Base offset for GMAC1 within FE
    pub const GMAC1_BASE: usize = 0x10100;

    /// MAC Control Register
    pub const MAC_MCR: usize = 0x00;

    /// MAC address registers
    pub const MAC_ADRH: usize = 0x0C;  // [47:32]
    pub const MAC_ADRL: usize = 0x10;  // [31:0]

    // MCR bits (from Linux mtk_eth_soc.h)
    /// Force mode (bypass AN)
    pub const MCR_FORCE_MODE: u32 = 1 << 15;
    /// TX enable
    pub const MCR_TX_EN: u32 = 1 << 14;
    /// RX enable
    pub const MCR_RX_EN: u32 = 1 << 13;
    /// Backoff enable
    pub const MCR_BACKOFF_EN: u32 = 1 << 9;
    /// Backpressure enable
    pub const MCR_BACKPR_EN: u32 = 1 << 8;
    /// Force RX flow control
    pub const MCR_FORCE_RX_FC: u32 = 1 << 5;
    /// Force TX flow control
    pub const MCR_FORCE_TX_FC: u32 = 1 << 4;
    /// Speed (bits 2-3): 00=10M, 01=100M, 10=1000M
    pub const MCR_SPEED_1000: u32 = 2 << 2;
    pub const MCR_SPEED_100: u32 = 1 << 2;
    /// Full duplex (bit 1, NOT bit 8!)
    pub const MCR_FORCE_DPX: u32 = 1 << 1;
    /// Force link up
    pub const MCR_FORCE_LINK: u32 = 1 << 0;
}

/// GMAC misc (MUX control)
mod mac_misc {
    /// MAC misc register (V3)
    pub const MAC_MISC_V3: usize = 0x10010;

    /// Route GMAC1 to internal switch (ESW)
    pub const MUX_TO_ESW: u32 = 1 << 0;
}

/// MDIO (PHY Management) registers
mod mdio_regs {
    /// PHY Indirect Access Control register (within GMAC block)
    /// Offset from FE_BASE
    pub const PHY_IAC: usize = 0x10004;

    // PHY_IAC bits
    pub const PHY_ACS_ST: u32 = 1 << 31;      // Access start/busy
    pub const PHY_RW_WRITE: u32 = 1 << 30;    // 1=write, 0=read
    pub const PHY_RW_READ: u32 = 0 << 30;
    pub const MDIO_ST_C22: u32 = 1 << 28;     // Clause 22

    pub const fn phy_addr(addr: u8) -> u32 {
        ((addr as u32) & 0x1f) << 20
    }

    pub const fn phy_reg(reg: u8) -> u32 {
        ((reg as u32) & 0x1f) << 25
    }

    pub const fn phy_data(data: u16) -> u32 {
        data as u32
    }
}

/// MT7531 Switch registers (memory-mapped in MT7988)
/// Based on Linux drivers/net/dsa/mt7530.c and mt7530.h
mod mt7531_regs {
    // Switch core registers
    /// System Control Register
    pub const SYS_CTRL: u32 = 0x7000;
    pub const SYS_CTRL_PHY_RST: u32 = 1 << 2;  // Reset PHYs
    pub const SYS_CTRL_SW_RST: u32 = 1 << 1;   // Software reset
    pub const SYS_CTRL_REG_RST: u32 = 1 << 0;  // Register reset

    /// Port Control Register base (per-port: + port * 0x100)
    pub const PCR_BASE: u32 = 0x2004;
    /// Port matrix (which ports can forward to which)
    pub const fn pcr_matrix(ports: u8) -> u32 {
        ((ports as u32) & 0x7f) << 16
    }
    /// Port VLAN and port matrix control
    pub const PCR_PORT_VLAN_MASK: u32 = 0x3 << 0;
    pub const PCR_PORT_VLAN_USER: u32 = 0 << 0;  // User port mode

    /// Port MAC Control Register base
    pub const PMCR_BASE: u32 = 0x3000;

    // MT7531 Force Mode bits (high bits to ENABLE forcing each parameter)
    // These are different from MT7530!
    pub const MT7531_FORCE_MODE_LNK: u32 = 1 << 31;    // Enable force link
    pub const MT7531_FORCE_MODE_SPD: u32 = 1 << 30;    // Enable force speed
    pub const MT7531_FORCE_MODE_DPX: u32 = 1 << 29;    // Enable force duplex
    pub const MT7531_FORCE_MODE_RX_FC: u32 = 1 << 28;  // Enable force RX flow control
    pub const MT7531_FORCE_MODE_TX_FC: u32 = 1 << 27;  // Enable force TX flow control

    // PMCR value bits (same for MT7530 and MT7531)
    pub const PMCR_FORCE_LNK: u32 = 1 << 0;        // Link up value
    pub const PMCR_FORCE_DPX: u32 = 1 << 1;        // Full duplex value
    pub const PMCR_FORCE_SPD_1000: u32 = 2 << 2;   // Speed = 1000Mbps
    pub const PMCR_FORCE_SPD_100: u32 = 1 << 2;    // Speed = 100Mbps
    pub const PMCR_TX_FC_EN: u32 = 1 << 4;         // TX flow control value
    pub const PMCR_RX_FC_EN: u32 = 1 << 5;         // RX flow control value

    // PMCR control bits
    pub const PMCR_BACKPR_EN: u32 = 1 << 9;
    pub const PMCR_BKOFF_EN: u32 = 1 << 10;
    pub const PMCR_RX_EN: u32 = 1 << 13;
    pub const PMCR_TX_EN: u32 = 1 << 14;
    pub const PMCR_MAC_MODE: u32 = 1 << 16;  // MAC mode (vs PHY mode)

    /// Port MAC Status Register
    pub const PMSR_BASE: u32 = 0x3008;
    pub const PMSR_LINK: u32 = 1 << 0;
    pub const PMSR_DPX: u32 = 1 << 1;
    pub const PMSR_SPD_MASK: u32 = 3 << 2;

    /// Port Security Control Register
    pub const PSC_BASE: u32 = 0x200c;
    pub const PSC_DIS_PORT: u32 = 1 << 31;  // Port disable bit

    /// Internal PHY Indirect Access Control (MT7531 specific)
    /// Different encoding from MT7530!
    pub const PHY_IAC: u32 = 0x701C;
    pub const PHY_ACS_ST: u32 = 1 << 31;     // Access start/busy

    // MT7531 command encoding (bits 18-19)
    pub const MDIO_CMD_WRITE: u32 = 0x1 << 18;  // C22 write
    pub const MDIO_CMD_READ: u32 = 0x2 << 18;   // C22 read

    // MT7531 clause type (bits 16-17)
    pub const MDIO_ST_C22: u32 = 0x1 << 16;     // Clause 22

    pub const fn phy_addr(addr: u8) -> u32 {
        ((addr as u32) & 0x1f) << 20
    }
    pub const fn phy_reg(reg: u8) -> u32 {
        ((reg as u32) & 0x1f) << 25
    }
    pub const fn phy_data(data: u16) -> u32 {
        data as u32
    }

    /// CPU port number (port 6 connects to GMAC1)
    pub const CPU_PORT: u8 = 6;

    /// Number of user ports (0-3 are RJ45, 4 is SFP)
    pub const NUM_USER_PORTS: u8 = 4;

    /// GMAC Mux Control
    pub const TOP_SIG_CTRL: u32 = 0x7808;
    pub const TOP_SIG_CTRL_NORMAL: u32 = 0;  // Normal switch mode

    /// PHY control registers (MT7531 specific)
    pub const PLLGP_EN: u32 = 0x7820;     // PLL enable
    pub const PLLGP_CR0: u32 = 0x78a8;    // PLL config
    pub const PHY_CNTL: u32 = 0x7a10;     // PHY power control
    pub const PHY_EN_MASK: u32 = 0x1F;    // PHY 0-4 enable bits
}

/// QDMA (Queue DMA) registers — for TX (NETSYS V3)
mod qdma_regs {
    /// QDMA base offset from FE_BASE
    pub const QDMA_BASE: usize = 0x4400;

    /// TX queue config (per queue, +0x10 each) - at QDMA_BASE
    pub const QDMA_QTX_CFG: usize = 0x0000;  // 0x4400
    /// TX scheduler config
    pub const QDMA_QTX_SCH: usize = 0x0004;  // 0x4404

    /// GLO_CFG register (V2/V3 offset)
    pub const QDMA_GLO_CFG: usize = 0x0204;  // 0x4604

    /// TX ring registers (MT7988 offsets)
    /// TX context pointer (ring base) - 0x4400+0x300=0x4700
    pub const QDMA_CTX_PTR: usize = 0x0300;
    /// TX DMA pointer (HW read position)
    pub const QDMA_DTX_PTR: usize = 0x0304;
    /// TX ring count
    pub const QDMA_TRING_CNT: usize = 0x0308;
    /// TX CPU release pointer (kick DMA)
    pub const QDMA_CRX_PTR: usize = 0x0310;
    /// TX DMA release pointer
    pub const QDMA_DRX_PTR: usize = 0x0314;

    /// Free queue registers (V2/V3 offset)
    pub const QDMA_FQ_HEAD: usize = 0x0320;
    pub const QDMA_FQ_TAIL: usize = 0x0324;
    pub const QDMA_FQ_COUNT: usize = 0x0328;
    pub const QDMA_FQ_BLEN: usize = 0x032C;

    /// QDMA RX ring registers (for TX completion feedback)
    /// Linux: qdma.rx_ptr = 0x4500, rx_cnt_cfg = 0x4504, qcrx_ptr = 0x4508
    pub const QDMA_RX_PTR: usize = 0x0100;      // 0x4400 + 0x100 = 0x4500
    pub const QDMA_RX_CNT_CFG: usize = 0x0104;  // 0x4504
    pub const QDMA_QCRX_PTR: usize = 0x0108;    // 0x4508

    /// TX scheduler rate (per queue, +4 each)
    pub const QDMA_TX_SCH_RATE: usize = 0x0398;  // 0x4798

    /// Reset registers
    pub const QDMA_RST_IDX: usize = 0x0208;

    /// Flow control (Linux: QDMA_FC_TH, QDMA_HRED)
    pub const QDMA_FC_TH: usize = 0x0210;  // 0x4610
    pub const QDMA_HRED: usize = 0x0244;   // 0x4644

    // Flow control bits
    pub const FC_THRES_DROP_MODE: u32 = 1 << 20;
    pub const FC_THRES_DROP_EN: u32 = 7 << 16;
    pub const FC_THRES_MIN: u32 = 0x4444;

    // GLO_CFG bits - from Linux mtk_eth_soc.h
    pub const TX_DMA_EN: u32 = 1 << 0;
    pub const TX_DMA_BUSY: u32 = 1 << 1;
    pub const RX_DMA_EN: u32 = 1 << 2;
    pub const RX_DMA_BUSY: u32 = 1 << 3;
    pub const TX_BT_32DWORDS: u32 = 3 << 4;   // bits 5-4
    pub const TX_WB_DDONE: u32 = 1 << 6;      // bit 6 (NOT 8!)
    pub const NDP_CO_PRO: u32 = 1 << 10;      // Next Descriptor Pointer Coherent
    pub const RX_BT_32DWORDS: u32 = 3 << 11;  // bits 12-11 (NOT 6-7!)
    pub const MULTI_CNT: u32 = 0x4 << 12;     // bits 15-12
    pub const RESV_BUF: u32 = 0x40 << 16;     // bits 22-16
    pub const WCOMP_EN: u32 = 1 << 24;        // Write Complete Enable (V2+)
    pub const DMAD_WR_WDONE: u32 = 1 << 26;   // DMA Descriptor Write DONE (V2+)
    pub const CHK_DDONE_EN: u32 = 1 << 28;    // Check Descriptor DONE Enable (V2+)
    pub const RX_2B_OFFSET: u32 = 1 << 31;    // bit 31 (NOT 7!)

    /// Resource threshold for TX queue config
    pub const QDMA_RES_THRES: u32 = 4;

    // TX scheduler bits
    pub const QTX_SCH_MIN_RATE_EN: u32 = 1 << 27;
    pub const QTX_SCH_MIN_RATE_MAN: u32 = 1 << 20;  // mantissa=1
    pub const QTX_SCH_MIN_RATE_EXP: u32 = 4 << 16;  // exponent=4 (10Mbps min)
    pub const QTX_SCH_LEAKY_BUCKET_SIZE: u32 = 0x3 << 28;

    // TX scheduler rate
    pub const QDMA_TX_SCH_MAX_WFQ: u32 = 1 << 15;
}

/// PDMA (Packet DMA) registers — for TX and RX
/// NOTE: For NETSYS_V3 (MT7988), PDMA base is 0x6800, not 0x6000!
mod pdma_regs {
    /// PDMA base offset from FE_BASE (NETSYS_V3)
    pub const PDMA_BASE: usize = 0x6800;

    // TX ring registers (at PDMA_BASE + 0x000)
    // U-Boot: TX_BASE_PTR_REG(n) = 0x000 + n*0x10
    pub const PDMA_TX_PTR: usize = 0x0000;      // TX ring base pointer
    pub const PDMA_TX_MAX_CNT: usize = 0x0004;  // TX ring max count
    pub const PDMA_TX_CPU_IDX: usize = 0x0008;  // TX CPU index (write to kick)
    pub const PDMA_TX_DMA_IDX: usize = 0x000C;  // TX DMA index (completion)

    /// RX ring 0 registers (at PDMA_BASE + 0x100)
    pub const PDMA_RX_PTR: usize = 0x0100;      // Was 0x0900 with old base
    pub const PDMA_RX_MAX_CNT: usize = 0x0104;
    pub const PDMA_RX_CPU_IDX: usize = 0x0108;
    pub const PDMA_RX_DMA_IDX: usize = 0x010C;

    /// Global config (at PDMA_BASE + 0x204)
    pub const PDMA_GLO_CFG: usize = 0x0204;
    /// Reset index
    pub const PDMA_RST_IDX: usize = 0x0208;

    // GLO_CFG bits (from Linux mtk_eth_soc.h)
    pub const TX_DMA_EN: u32 = 1 << 0;
    pub const TX_DMA_BUSY: u32 = 1 << 1;
    pub const RX_DMA_EN: u32 = 1 << 2;
    pub const RX_DMA_BUSY: u32 = 1 << 3;
    pub const TX_BT_32DWORDS: u32 = 3 << 4;
    pub const TX_WB_DDONE: u32 = 1 << 6;
    pub const RX_BT_32DWORDS: u32 = 3 << 11;  // bits 12-11
    pub const MULTI_EN: u32 = 1 << 10;  // MTK_MULTI_EN BIT(10) - enable multiple DMA bursts
    pub const RX_2B_OFFSET: u32 = 1 << 31;

    // PDMA TX descriptor bits (V2 format)
    pub const TXD2_DDONE: u32 = 1 << 31;   // Descriptor done
    pub const TXD2_LS0: u32 = 1 << 30;     // Last segment
    pub const TXD2_SDL0_SHIFT: u32 = 8;    // Length field shift (bits 23:8)
    pub const TXD5_FPORT_SHIFT: u32 = 16;  // FPORT field shift (bits 19:16)
}

// =============================================================================
// V2 Descriptor Format (32 bytes)
// =============================================================================

/// TX descriptor (V2 format, 32 bytes)
#[repr(C, align(32))]
#[derive(Clone, Copy)]
struct TxDesc {
    /// txd1: Buffer address (low 32 bits)
    txd1: u32,
    /// txd2: Buffer address (high bits) or next desc pointer
    txd2: u32,
    /// txd3: Control/status word
    txd3: u32,
    /// txd4: VLAN/port info
    txd4: u32,
    /// txd5-8: Reserved
    txd5: u32,
    txd6: u32,
    txd7: u32,
    txd8: u32,
}

impl TxDesc {
    const fn zeroed() -> Self {
        Self {
            txd1: 0, txd2: 0, txd3: 0, txd4: 0,
            txd5: 0, txd6: 0, txd7: 0, txd8: 0,
        }
    }
}

// TX descriptor bits (txd3) - Linux mtk_eth_soc.h
const TX_DMA_OWNER_CPU: u32 = 1 << 31;  // BIT(31) - CPU owns descriptor
const TX_DMA_LS0: u32 = 1 << 30;        // BIT(30) - Last segment
const TX_DMA_PLEN0_MASK: u32 = 0xFFFF;  // For V2: max_len=0xFFFF
const TX_DMA_PLEN0_SHIFT: u32 = 8;      // For V2: dma_len_offset=8

fn tx_dma_plen0(len: usize) -> u32 {
    // Linux V2: TX_DMA_PLEN0(x) = ((x) & 0xFFFF) << 8
    ((len as u32) & TX_DMA_PLEN0_MASK) << TX_DMA_PLEN0_SHIFT
}

// 36-bit DMA support for MT7988 (MTK_36BIT_DMA capability)
// High 4 address bits [35:32] go into txd3/rxd3 bits [3:0]
// Linux: TX_DMA_ADDR64_MASK = GENMASK(3, 0) = 0xF
// Linux: TX_DMA_PREP_ADDR64(x) = ((x >> 32) & 0xF)
// Linux: RX_DMA_ADDR64_MASK = GENMASK(3, 0) = 0xF
// Linux: RX_DMA_PREP_ADDR64(x) = ((x >> 32) & 0xF)

/// Extract high 4 bits [35:32] of 64-bit address for txd3
fn tx_dma_prep_addr64(addr: u64) -> u32 {
    ((addr >> 32) & 0xF) as u32
}

/// Extract high 4 bits [35:32] of 64-bit address for rxd3
fn rx_dma_prep_addr64(addr: u64) -> u32 {
    ((addr >> 32) & 0xF) as u32
}

// TX descriptor bits (txd4) - V2/V3 format
// Linux: TX_DMA_FPORT_SHIFT_V2 = 8, FPORT in bits 8-11
// Linux: TX_DMA_SWC_V2 = BIT(30) - must be set for TX to work
// Linux: QID_BITS_V2(x) = ((x) & 0x3f) << 16 - queue ID in bits 21:16
const TX_DMA_SWC_V2: u32 = 1 << 30;
const TX_DMA_FPORT_SHIFT_V2: u32 = 8;
const TX_DMA_QID_SHIFT_V2: u32 = 16;
const TX_DMA_QID_MASK_V2: u32 = 0x3f;

/// Build QID bits for txd4 (V2/V3 format)
/// Linux: QID_BITS_V2(x) = ((x) & 0x3f) << 16
fn qid_bits_v2(qid: u8) -> u32 {
    ((qid as u32) & TX_DMA_QID_MASK_V2) << TX_DMA_QID_SHIFT_V2
}

/// Build txd4 for V2/V3 descriptor format
fn tx_dma_txd4_v2(pse_port: u8, qid: u8) -> u32 {
    TX_DMA_SWC_V2 | (((pse_port as u32) & 0xf) << TX_DMA_FPORT_SHIFT_V2) | qid_bits_v2(qid)
}

// PSE port numbers (from Linux mtk_pse_port enum)
/// PSE forward ports (from U-Boot: fport = gmac_id + 1)
// GMAC numbering: GMAC0=first MAC (4x RJ45), GMAC1=second MAC (SFP+)
const PSE_GDM1_PORT: u8 = 1;   // GMAC0 (first MAC, 4x GbE RJ45 via internal switch)
#[allow(dead_code)]
const PSE_GDM2_PORT: u8 = 2;   // GMAC1 (second MAC, SFP+)
#[allow(dead_code)]
const PSE_GDM3_PORT: u8 = 15;  // GMAC2 (third MAC, special port)

/// RX descriptor (V2 format, 32 bytes)
#[repr(C, align(32))]
#[derive(Clone, Copy)]
struct RxDesc {
    /// rxd1: Buffer address (low 32 bits)
    rxd1: u32,
    /// rxd2: Status/length
    rxd2: u32,
    /// rxd3: More status
    rxd3: u32,
    /// rxd4: VLAN/port info
    rxd4: u32,
    /// rxd5-8: Reserved
    rxd5: u32,
    rxd6: u32,
    rxd7: u32,
    rxd8: u32,
}

impl RxDesc {
    const fn zeroed() -> Self {
        Self {
            rxd1: 0, rxd2: 0, rxd3: 0, rxd4: 0,
            rxd5: 0, rxd6: 0, rxd7: 0, rxd8: 0,
        }
    }
}

// RX descriptor bits (rxd2)
const RX_DMA_DONE: u32 = 1 << 31;
const RX_DMA_LS0: u32 = 1 << 30;

// Packet length extraction:
// MT7988 NETSYS_V3 RX descriptor format (from Linux mtk_eth_soc.c):
// - Length is in rxd2 (NOT rxd5!)
// - mt7988_data: rx.dma_len_offset = 8, rx.dma_max_len = 0xffff
// - RX_DMA_GET_PLEN0(rxd2) = (rxd2 >> 8) & 0xffff
// - RX_DMA_DONE = BIT(31) in rxd2

/// RX length mask for MT7988 (MTK_TX_DMA_BUF_LEN_V2)
const RX_DMA_PLEN_MASK_V2: u32 = 0xFFFF;

/// Extract RX packet length from rxd2 for MT7988
/// Linux: RX_DMA_GET_PLEN0(x) = (x >> dma_len_offset) & dma_max_len
/// MT7988: dma_len_offset=8, dma_max_len=0xffff
fn rx_dma_plen_v2(rxd2: u32) -> usize {
    ((rxd2 >> 8) & RX_DMA_PLEN_MASK_V2) as usize
}

// =============================================================================
// Constants
// =============================================================================

/// Number of TX descriptors
const TX_RING_SIZE: usize = 64;
/// Number of RX descriptors (PDMA)
const RX_RING_SIZE: usize = 64;
/// Free queue size (for QDMA)
const FQ_SIZE: usize = 64;
/// QDMA RX ring size (for TX completion feedback - Linux allocates this)
const QDMA_RX_RING_SIZE: usize = 64;

/// Buffer size for each RX descriptor (2KB, must accommodate 1514 + padding)
const RX_BUF_SIZE: usize = 2048;
/// Buffer size for TX (max Ethernet frame)
const TX_BUF_SIZE: usize = 2048;

/// RX buffer offset to Ethernet frame:
/// - 2 bytes: IP alignment padding (from RX_2B_OFFSET in PDMA_GLO_CFG)
/// Linux skb_reserve(skb, NET_SKB_PAD + NET_IP_ALIGN) then skb_put(skb, pktlen)
/// The DMA writes directly to buffer with 2-byte prefix for alignment
const RX_FRAME_OFFSET: usize = 2;

/// DMA pool layout:
/// - TX descriptors: TX_RING_SIZE * 32 = 2KB
/// - RX descriptors: RX_RING_SIZE * 32 = 2KB
/// - Free queue descriptors: FQ_SIZE * 32 = 2KB
/// - QDMA RX descriptors: QDMA_RX_RING_SIZE * 32 = 2KB
/// - TX buffers: TX_RING_SIZE * 2KB = 128KB
/// - RX buffers: RX_RING_SIZE * 2KB = 128KB
/// - Free queue buffers: FQ_SIZE * 2KB = 128KB
/// - QDMA RX buffers: QDMA_RX_RING_SIZE * 2KB = 128KB
/// Total: ~520KB
const DMA_POOL_SIZE: usize = (TX_RING_SIZE + RX_RING_SIZE + FQ_SIZE + QDMA_RX_RING_SIZE) * 32
    + (TX_RING_SIZE + RX_RING_SIZE + FQ_SIZE + QDMA_RX_RING_SIZE) * 2048;

/// Network opcodes (using io_op::NET_BASE = 0x60)
mod net_op {
    pub const TX: u8 = super::io_op::NET_BASE;       // 0x60 - transmit packet
    pub const RX: u8 = super::io_op::NET_BASE + 1;   // 0x61 - received packet (CQ only)
}

/// Sidechannel query: network info
const SIDE_QUERY_NET_INFO: u16 = side_msg::QUERY_INFO;

/// RX poll timer: 10ms interval (until MSI-X interrupt support)
const RX_POLL_INTERVAL_NS: u64 = 10_000_000;
const TAG_RX_POLL: u32 = 1;

// =============================================================================
// EthDriver State
// =============================================================================

struct EthDriver {
    /// Frame Engine MMIO region
    fe: Option<MmioRegion>,
    /// ETHSYS MMIO region (clock/reset/DMA_AG_MAP)
    ethsys: Option<MmioRegion>,
    /// Internal Switch (GSW) MMIO region
    gsw: Option<MmioRegion>,
    /// DMA memory pool
    dma: Option<DmaPool>,

    /// TX ring CPU index (next to fill)
    tx_head: usize,
    /// TX ring HW index (next to reclaim)
    tx_tail: usize,

    /// RX ring CPU index (next to check)
    rx_head: usize,

    /// Our MAC address
    mac: [u8; 6],

    /// DataPort for consumer
    port_id: Option<PortId>,

    /// Sequence number for RX packets
    rx_seq: u32,

    /// RX poll timer
    rx_poll_timer: Option<Timer>,

    /// Link status (always up for internal switch)
    link_up: bool,
}

/// DMA pool offsets
struct DmaLayout {
    tx_ring_off: usize,
    rx_ring_off: usize,
    fq_ring_off: usize,
    qdma_rx_ring_off: usize,
    tx_buf_off: usize,
    rx_buf_off: usize,
    fq_buf_off: usize,
    qdma_rx_buf_off: usize,
}

impl DmaLayout {
    const fn new() -> Self {
        let tx_ring_off = 0;
        let rx_ring_off = tx_ring_off + TX_RING_SIZE * 32;
        let fq_ring_off = rx_ring_off + RX_RING_SIZE * 32;
        let qdma_rx_ring_off = fq_ring_off + FQ_SIZE * 32;
        let tx_buf_off = qdma_rx_ring_off + QDMA_RX_RING_SIZE * 32;
        let rx_buf_off = tx_buf_off + TX_RING_SIZE * TX_BUF_SIZE;
        let fq_buf_off = rx_buf_off + RX_RING_SIZE * RX_BUF_SIZE;
        let qdma_rx_buf_off = fq_buf_off + FQ_SIZE * TX_BUF_SIZE;
        Self {
            tx_ring_off,
            rx_ring_off,
            fq_ring_off,
            qdma_rx_ring_off,
            tx_buf_off,
            rx_buf_off,
            fq_buf_off,
            qdma_rx_buf_off,
        }
    }
}

static LAYOUT: DmaLayout = DmaLayout::new();

impl EthDriver {
    const fn new() -> Self {
        Self {
            fe: None,
            ethsys: None,
            gsw: None,
            dma: None,
            tx_head: 0,
            tx_tail: 0,
            rx_head: 0,
            mac: [0x02, 0x12, 0xce, 0x6f, 0xbb, 0xf1], // Locally-administered MAC
            port_id: None,
            rx_seq: 0,
            rx_poll_timer: None,
            link_up: false,
        }
    }

    /// Reset the Frame Engine using ETHSYS (Linux mtk_hw_reset)
    fn reset_fe(&self) {
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };
        let ethsys = match &self.ethsys {
            Some(e) => e,
            None => return,
        };

        uinfo!("ethd", "reset_fe_start";);

        // NOTE: 100ms delay is now in init() BEFORE this function is called
        // (Linux mtk_hw_init:4125 does msleep(100) before reset)

        // Step 1: Disable idle check (Linux mtk_hw_reset:3902)
        // ETHSYS+0x28 = 0
        ethsys.write32(ethsys_regs::FE_RST_CHK_IDLE_EN, 0);

        // Step 3: Build reset mask for V3 (Linux mtk_hw_reset:3904-3923)
        let reset_mask = ethsys_regs::RSTCTRL_FE
            | ethsys_regs::RSTCTRL_ETH
            | ethsys_regs::RSTCTRL_PPE0_V3
            | ethsys_regs::RSTCTRL_PPE1_V3
            | ethsys_regs::RSTCTRL_WDMA0
            | ethsys_regs::RSTCTRL_WDMA1
            | ethsys_regs::RSTCTRL_WDMA2;

        // Step 4: Assert reset (Linux ethsys_reset via regmap_update_bits)
        let old_rst = ethsys.read32(ethsys_regs::RSTCTRL);
        ethsys.write32(ethsys_regs::RSTCTRL, old_rst | reset_mask);
        uinfo!("ethd", "reset_assert"; mask = userlib::ulog::hex32(reset_mask));

        // Step 5: Wait 1ms (Linux usleep_range(1000, 1100))
        delay_us(1000);

        // Step 6: Deassert reset
        ethsys.write32(ethsys_regs::RSTCTRL, old_rst & !reset_mask);

        // Step 7: Wait 10ms (Linux ethsys_reset:3781 does mdelay(10))
        delay_us(10_000);

        // Step 8: Re-enable idle check (Linux mtk_hw_reset:3925-3927)
        // ETHSYS+0x28 = 0x6f8ff (specific value from Linux, not 0xFFFFFFFF!)
        ethsys.write32(ethsys_regs::FE_RST_CHK_IDLE_EN, 0x6f8ff);

        // NOTE: Removed FE_RST_GL (FE_BASE+0x04) reset - Linux doesn't do this!
        // The ETHSYS reset above is sufficient.

        // Enable PDMAv2 mode (required for NETSYSv3 / MT7988)
        let misc = fe.read32(fe_regs::FE_GLO_MISC);
        fe.write32(fe_regs::FE_GLO_MISC, misc | fe_regs::FE_GLO_MISC_PDMA_V2);

        // Configure PSE (Packet Scheduling Engine) for V3
        // CRITICAL: Without this, packets will be dropped!
        // Values from Linux mtk_hw_init() for NETSYS_V3:

        // PSE should not drop port8, port9 and port13 packets (from WDMA Tx)
        fe.write32(fe_regs::PSE_DROP_CFG, 0x00002300);

        // PSE should drop packets to port8, port9, port13 on WDMA Rx ring full
        fe.write32(fe_regs::PSE_PPE_DROP0, 0x00002300);
        fe.write32(fe_regs::PSE_PPE_DROP1, 0x00002300);
        fe.write32(fe_regs::PSE_PPE_DROP2, 0x00002300);

        // PSE Free Queue Flow Control (Linux mtk_hw_init:4236)
        fe.write32(fe_regs::PSE_FQFC_CFG2, 0x01fa01f4);

        // PSE config input queue threshold (Linux mtk_hw_init:4238-4246)
        // CRITICAL: These must match Linux exactly for packet flow!
        fe.write32(fe_regs::PSE_IQ_REV1, 0x001a000e);
        fe.write32(fe_regs::PSE_IQ_REV2, 0x01ff001a);
        fe.write32(fe_regs::PSE_IQ_REV3, 0x000e01ff);
        fe.write32(fe_regs::PSE_IQ_REV4, 0x000e000e);
        fe.write32(fe_regs::PSE_IQ_REV5, 0x000e000e);
        fe.write32(fe_regs::PSE_IQ_REV6, 0x000e000e);
        fe.write32(fe_regs::PSE_IQ_REV7, 0x000e000e);
        fe.write32(fe_regs::PSE_IQ_REV8, 0x000e000e);

        // PSE config output queue threshold (Linux mtk_hw_init:4248-4256)
        // CRITICAL: Without these, packets get STUCK in PSE output queues!
        fe.write32(fe_regs::PSE_OQ_TH1, 0x000f000a);
        fe.write32(fe_regs::PSE_OQ_TH2, 0x001a000f);
        fe.write32(fe_regs::PSE_OQ_TH3, 0x000f001a);
        fe.write32(fe_regs::PSE_OQ_TH4, 0x01ff000f);
        fe.write32(fe_regs::PSE_OQ_TH5, 0x000f000f);
        fe.write32(fe_regs::PSE_OQ_TH6, 0x0006000f);
        fe.write32(fe_regs::PSE_OQ_TH7, 0x00060006);
        fe.write32(fe_regs::PSE_OQ_TH8, 0x00060006);

        // GDM and CDM thresholds (V3) - Linux mtk_hw_init:4258-4264
        fe.write32(fe_regs::MTK_GDM2_THRES, 0x00000004);
        fe.write32(fe_regs::MTK_CDMW0_THRES, 0x00000004);
        fe.write32(fe_regs::MTK_CDMW1_THRES, 0x00000004);
        fe.write32(fe_regs::MTK_CDME0_THRES, 0x00000004);
        fe.write32(fe_regs::MTK_CDME1_THRES, 0x00000004);
        fe.write32(fe_regs::MTK_CDMM_THRES, 0x00000004);

        // CDMQ ingress control - parse MTK special tag from CPU
        // Linux: mtk_w32(eth, val | MTK_CDMQ_STAG_EN, MTK_CDMQ_IG_CTRL);
        let cdmq = fe.read32(fe_regs::MTK_CDMQ_IG_CTRL);
        fe.write32(fe_regs::MTK_CDMQ_IG_CTRL, cdmq | fe_regs::MTK_CDMQ_STAG_EN);

        // PSE Dummy Page Mechanism (V3 CRITICAL!)
        // Linux: PSE_DUMY_REQ = PSE_DUMMY_WORK_GDM(1) | PSE_DUMMY_WORK_GDM(2) |
        //                       PSE_DUMMY_WORK_GDM(3) | DUMMY_PAGE_THR
        let pse_dummy = fe_regs::PSE_DUMMY_WORK_GDM1
            | fe_regs::PSE_DUMMY_WORK_GDM2
            | fe_regs::PSE_DUMMY_WORK_GDM3
            | fe_regs::PSE_DUMMY_PAGE_THR;
        fe.write32(fe_regs::PSE_DUMY_REQ, pse_dummy);

        // Interrupt grouping setup (Linux mtk_hw_init lines 4185-4193)
        // Even though we use polling, these registers may need to be configured
        // for the hardware to work correctly.

        // Disable all interrupts first
        fe.write32(fe_regs::TX_IRQ_MASK, 0);      // Clear TX IRQ mask
        fe.write32(fe_regs::PDMA_IRQ_MASK, 0);    // Clear RX IRQ mask

        // Configure interrupt grouping (routes interrupts to correct groups)
        fe.write32(fe_regs::PDMA_INT_GRP0, fe_regs::TX_DONE_INT);     // 0x10000000
        fe.write32(fe_regs::PDMA_INT_GRP1, fe_regs::RX_DONE_INT_V2);  // 0x4000
        fe.write32(fe_regs::QDMA_INT_GRP0, fe_regs::TX_DONE_INT);     // 0x10000000
        fe.write32(fe_regs::QDMA_INT_GRP1, fe_regs::RX_DONE_INT_V2);  // 0x4000
        fe.write32(fe_regs::FE_INT_GRP, fe_regs::FE_INT_GRP_VALUE);   // 0x21021000

        uinfo!("ethd", "reset_fe_done"; pse_drop = userlib::ulog::hex32(0x00002300), pse_dummy = userlib::ulog::hex32(pse_dummy));
    }

    /// Initialize DMA address generation - CRITICAL for DMA to work!
    /// This enables DMA address generation for PDMA and QDMA.
    /// Without this, DMA engines cannot properly translate descriptor addresses.
    /// Enable Ethernet clocks via ETHSYS clock gate register
    /// This must be called FIRST, before any other initialization!
    /// From clk-mt7988-eth.c: clock gates are at ETHSYS+0x30
    /// Linux enables these via the clock framework in mtk_clk_enable()
    fn enable_clocks(&self) {
        let ethsys = match &self.ethsys {
            Some(e) => e,
            None => return,
        };

        // Read current clock gate value
        let old_val = ethsys.read32(ethsys_regs::CLK_GATE);
        uinfo!("ethd", "clk_gate_before"; val = userlib::ulog::hex32(old_val));

        // Enable required clocks: FE, GP1, ESW (minimum for GMAC1 + internal switch)
        // From clk-mt7988-eth.c ethdma_clks array
        let new_val = old_val
            | ethsys_regs::CLK_FE_EN    // Bit 6: Frame Engine
            | ethsys_regs::CLK_GP1_EN   // Bit 8: GMAC Port 1
            | ethsys_regs::CLK_ESW_EN;  // Bit 16: Ethernet Switch
        ethsys.write32(ethsys_regs::CLK_GATE, new_val);

        // Verify
        let verify = ethsys.read32(ethsys_regs::CLK_GATE);
        uinfo!("ethd", "clk_gate_after"; val = userlib::ulog::hex32(verify));
    }

    /// Initialize DMA address generation - CRITICAL for DMA to work!
    /// This enables DMA address generation for PDMA and QDMA.
    /// Without this, DMA engines cannot properly translate descriptor addresses.
    /// IMPORTANT: This register is at ETHSYS_BASE+0x408, NOT FE_BASE!
    fn init_dma_ag_map(&self) {
        let ethsys = match &self.ethsys {
            Some(e) => e,
            None => return,
        };

        // Read current DMA_AG_MAP (at ETHSYS_BASE + 0x408 = 0x15000408)
        let old_val = ethsys.read32(ethsys_regs::DMA_AG_MAP);
        uinfo!("ethd", "dma_ag_map_before"; val = userlib::ulog::hex32(old_val));

        // Enable DMA address generation for PDMA, QDMA, and PPE
        // Linux: PDMA | QDMA | PPE = 0x7
        let new_val = old_val
            | ethsys_regs::DMA_AG_MAP_PDMA   // BIT(0)
            | ethsys_regs::DMA_AG_MAP_QDMA   // BIT(1)
            | ethsys_regs::DMA_AG_MAP_PPE;   // BIT(2)
        ethsys.write32(ethsys_regs::DMA_AG_MAP, new_val);

        // Verify
        let verify = ethsys.read32(ethsys_regs::DMA_AG_MAP);
        uinfo!("ethd", "dma_ag_map_after"; val = userlib::ulog::hex32(verify));
    }

    /// Wait for DMA to be idle before initializing rings (Linux mtk_dma_busy_wait)
    fn dma_busy_wait(&self) -> bool {
        let fe = match &self.fe {
            Some(f) => f,
            None => return false,
        };

        // Poll QDMA GLO_CFG for DMA not busy (timeout 5ms with 5us intervals)
        let qdma_base = qdma_regs::QDMA_BASE;
        for _ in 0..1000 {
            let val = fe.read32(qdma_base + qdma_regs::QDMA_GLO_CFG);
            if (val & (qdma_regs::RX_DMA_BUSY | qdma_regs::TX_DMA_BUSY)) == 0 {
                return true;
            }
            delay_us(5);
        }
        uerror!("ethd", "dma_busy_timeout";);
        false
    }

    /// Configure GDM1 to forward RX to PDMA with checksum offload
    fn configure_gdm(&self) {
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };

        // GDM1_FWD_CFG: Forward RX to PDMA + enable checksum offload
        // Linux mtk_gdm_config() does read-modify-write:
        //   1. val = mtk_r32(GDM_FWD_CFG)
        //   2. val &= ~0xffff           // Clear low 16 bits (forward port)
        //   3. val |= ICS_EN | TCS_EN | UCS_EN
        //   4. val |= config            // GDMA_TO_PDMA = 0x0
        //   5. mtk_w32(GDM_FWD_CFG, val)
        let mut val = fe.read32(gdm_regs::GDM1_FWD_CFG);
        val &= !0xffff; // Clear low 16 bits (forward port config)
        val |= gdm_regs::GDM_ICS_EN
            | gdm_regs::GDM_TCS_EN
            | gdm_regs::GDM_UCS_EN
            | gdm_regs::GDM_TO_PDMA;
        fe.write32(gdm_regs::GDM1_FWD_CFG, val);

        uinfo!("ethd", "gdm_configured"; val = userlib::ulog::hex32(val));
    }

    /// Configure MAC (GMAC1) for internal switch
    fn configure_mac(&self) {
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };

        // Set MAC address
        let mac_h = ((self.mac[0] as u32) << 8) | (self.mac[1] as u32);
        let mac_l = ((self.mac[2] as u32) << 24)
            | ((self.mac[3] as u32) << 16)
            | ((self.mac[4] as u32) << 8)
            | (self.mac[5] as u32);

        let gmac_base = mac_regs::GMAC1_BASE;
        fe.write32(gmac_base + mac_regs::MAC_ADRH, mac_h);
        fe.write32(gmac_base + mac_regs::MAC_ADRL, mac_l);

        // Configure MCR: force mode, 100Mbps (match switch ports), full duplex, link up, TX/RX enable
        // Bits from Linux: FORCE_MODE | TX_EN | RX_EN | BACKOFF_EN | BACKPR_EN | SPEED | DPX | LINK
        let mcr = mac_regs::MCR_FORCE_MODE
            | mac_regs::MCR_TX_EN
            | mac_regs::MCR_RX_EN
            | mac_regs::MCR_BACKOFF_EN
            | mac_regs::MCR_BACKPR_EN
            | mac_regs::MCR_SPEED_1000  // Internal MAC-to-switch: always 1Gbps
            | mac_regs::MCR_FORCE_DPX  // bit 1 = full duplex
            | mac_regs::MCR_FORCE_LINK;
        fe.write32(gmac_base + mac_regs::MAC_MCR, mcr);

        // Route GMAC1 to internal switch
        fe.set32(mac_misc::MAC_MISC_V3, mac_misc::MUX_TO_ESW);

        uinfo!("ethd", "mac_configured"; mcr = userlib::ulog::hex32(mcr));
    }

    // =========================================================================
    // MDIO Access
    // =========================================================================

    /// Wait for MDIO access to complete
    fn mdio_wait(&self) -> bool {
        let fe = match &self.fe {
            Some(f) => f,
            None => return false,
        };

        for _ in 0..1000 {
            let val = fe.read32(mdio_regs::PHY_IAC);
            if (val & mdio_regs::PHY_ACS_ST) == 0 {
                return true;
            }
            delay_us(10);
        }
        false
    }

    /// Read a PHY register via MDIO
    fn mdio_read(&self, phy_addr: u8, reg: u8) -> Option<u16> {
        let fe = match &self.fe {
            Some(f) => f,
            None => return None,
        };

        if !self.mdio_wait() {
            return None;
        }

        let cmd = mdio_regs::PHY_ACS_ST
            | mdio_regs::MDIO_ST_C22
            | mdio_regs::PHY_RW_READ
            | mdio_regs::phy_addr(phy_addr)
            | mdio_regs::phy_reg(reg);

        fe.write32(mdio_regs::PHY_IAC, cmd);

        if !self.mdio_wait() {
            return None;
        }

        let val = fe.read32(mdio_regs::PHY_IAC);
        Some((val & 0xFFFF) as u16)
    }

    /// Write a PHY register via MDIO
    fn mdio_write(&self, phy_addr: u8, reg: u8, data: u16) -> bool {
        let fe = match &self.fe {
            Some(f) => f,
            None => return false,
        };

        if !self.mdio_wait() {
            return false;
        }

        let cmd = mdio_regs::PHY_ACS_ST
            | mdio_regs::MDIO_ST_C22
            | mdio_regs::PHY_RW_WRITE
            | mdio_regs::phy_addr(phy_addr)
            | mdio_regs::phy_reg(reg)
            | mdio_regs::phy_data(data);

        fe.write32(mdio_regs::PHY_IAC, cmd);
        self.mdio_wait()
    }

    // =========================================================================
    // MT7531 Switch Access (via memory-mapped I/O)
    // MT7988's internal switch is memory-mapped at GSW_BASE, not MDIO
    // =========================================================================

    /// Read a switch register via memory-mapped I/O
    fn sw_read(&self, addr: u32) -> Option<u32> {
        let gsw = self.gsw.as_ref()?;
        Some(gsw.read32(addr as usize))
    }

    /// Write a switch register via memory-mapped I/O
    fn sw_write(&self, addr: u32, val: u32) {
        if let Some(gsw) = &self.gsw {
            gsw.write32(addr as usize, val);
        }
    }

    // =========================================================================
    // MT7531 Switch Initialization (Memory-Mapped)
    // =========================================================================

    /// Wait for switch internal PHY access to complete
    fn sw_phy_wait(&self) -> bool {
        for _ in 0..1000 {
            if let Some(val) = self.sw_read(mt7531_regs::PHY_IAC) {
                if val & mt7531_regs::PHY_ACS_ST == 0 {
                    return true;
                }
            }
            delay_us(10);
        }
        false
    }

    /// Read internal PHY register via switch's PHY_IAC (MT7531 encoding)
    fn sw_phy_read(&self, phy_addr: u8, reg: u8) -> Option<u16> {
        if !self.sw_phy_wait() {
            uwarn!("ethd", "phy_wait_timeout_pre"; addr = phy_addr as u32, reg = reg as u32);
            return None;
        }

        // MT7531: ACS_ST | CMD_READ | ST_C22 | phy_addr | phy_reg
        let cmd = mt7531_regs::PHY_ACS_ST
            | mt7531_regs::MDIO_CMD_READ
            | mt7531_regs::MDIO_ST_C22
            | mt7531_regs::phy_addr(phy_addr)
            | mt7531_regs::phy_reg(reg);

        self.sw_write(mt7531_regs::PHY_IAC, cmd);

        if !self.sw_phy_wait() {
            uwarn!("ethd", "phy_wait_timeout_post"; addr = phy_addr as u32, reg = reg as u32);
            return None;
        }

        let result = self.sw_read(mt7531_regs::PHY_IAC);
        if let Some(val) = result {
            uinfo!("ethd", "phy_read"; addr = phy_addr as u32, reg = reg as u32, val = userlib::ulog::hex32(val));
        }
        result.map(|v| (v & 0xFFFF) as u16)
    }

    /// Write internal PHY register via switch's PHY_IAC (MT7531 encoding)
    fn sw_phy_write(&self, phy_addr: u8, reg: u8, data: u16) -> bool {
        if !self.sw_phy_wait() {
            return false;
        }

        // MT7531: ACS_ST | CMD_WRITE | ST_C22 | phy_addr | phy_reg | data
        let cmd = mt7531_regs::PHY_ACS_ST
            | mt7531_regs::MDIO_CMD_WRITE
            | mt7531_regs::MDIO_ST_C22
            | mt7531_regs::phy_addr(phy_addr)
            | mt7531_regs::phy_reg(reg)
            | mt7531_regs::phy_data(data);

        self.sw_write(mt7531_regs::PHY_IAC, cmd);
        self.sw_phy_wait()
    }

    /// Initialize the internal MT7531 switch
    fn init_switch(&self) {
        uinfo!("ethd", "switch_init_start";);

        // Read switch ID to verify memory mapping works
        if let Some(sys_ctrl) = self.sw_read(mt7531_regs::SYS_CTRL) {
            uinfo!("ethd", "switch_sys_ctrl"; val = userlib::ulog::hex32(sys_ctrl));
        } else {
            uerror!("ethd", "switch_read_failed";);
            return;
        }

        // NOTE: Skip switch PHY reset - Linux does this via reset controller,
        // and doing it here might interfere with already-established links.
        // The PHYs should already be initialized by the bootloader.
        // let sys_ctrl = self.sw_read(mt7531_regs::SYS_CTRL).unwrap_or(0);
        // self.sw_write(mt7531_regs::SYS_CTRL, sys_ctrl | mt7531_regs::SYS_CTRL_PHY_RST);
        // delay_us(50000);
        // self.sw_write(mt7531_regs::SYS_CTRL, sys_ctrl & !mt7531_regs::SYS_CTRL_PHY_RST);
        // delay_us(100000);
        uinfo!("ethd", "switch_phy_skip_reset";);

        // Configure port 6 (CPU port) - connects to GMAC1
        // Force mode: 1000Mbps (internal switch backplane), full duplex, link up, TX/RX enabled
        // MT7531 uses high bits (31-27) to ENABLE forcing, low bits for values
        // NOTE: CPU port runs at 1Gbps regardless of external PHY speeds - switch handles conversion
        let cpu_pmcr = mt7531_regs::MT7531_FORCE_MODE_LNK     // Enable force link
            | mt7531_regs::MT7531_FORCE_MODE_SPD              // Enable force speed
            | mt7531_regs::MT7531_FORCE_MODE_DPX              // Enable force duplex
            | mt7531_regs::MT7531_FORCE_MODE_TX_FC            // Enable force TX FC
            | mt7531_regs::MT7531_FORCE_MODE_RX_FC            // Enable force RX FC
            | mt7531_regs::PMCR_FORCE_LNK                     // Link up
            | mt7531_regs::PMCR_FORCE_DPX                     // Full duplex
            | mt7531_regs::PMCR_FORCE_SPD_1000                // 1Gbps (internal backplane)
            | mt7531_regs::PMCR_TX_EN
            | mt7531_regs::PMCR_RX_EN
            | mt7531_regs::PMCR_BACKPR_EN
            | mt7531_regs::PMCR_BKOFF_EN;

        let cpu_port = mt7531_regs::CPU_PORT;
        let cpu_pmcr_addr = mt7531_regs::PMCR_BASE + (cpu_port as u32) * 0x100;
        self.sw_write(cpu_pmcr_addr, cpu_pmcr);
        uinfo!("ethd", "switch_cpu_port"; port = cpu_port as u32, pmcr = userlib::ulog::hex32(cpu_pmcr));

        // Configure user ports 0-3 (RJ45 ports)
        for port in 0..mt7531_regs::NUM_USER_PORTS {
            // Port Security Control - clear port disable bit
            let psc_addr = mt7531_regs::PSC_BASE + (port as u32) * 0x100;
            if let Some(psc) = self.sw_read(psc_addr) {
                uinfo!("ethd", "port_psc_before"; port = port as u32, val = userlib::ulog::hex32(psc));
                if psc & mt7531_regs::PSC_DIS_PORT != 0 {
                    // Clear disable bit
                    self.sw_write(psc_addr, psc & !mt7531_regs::PSC_DIS_PORT);
                }
            }

            // Port Control Register - set forwarding matrix
            // Allow forwarding to CPU port and all other user ports
            let pcr_addr = mt7531_regs::PCR_BASE + (port as u32) * 0x100;
            let matrix = (1u8 << cpu_port) | 0x0F; // CPU + ports 0-3
            let pcr_val = mt7531_regs::pcr_matrix(matrix);
            self.sw_write(pcr_addr, pcr_val);

            // Port MAC Control Register - force mode to match PHY (100Mbps FD)
            // MT7531 uses high bits to ENABLE forcing, low bits for values
            let pmcr_addr = mt7531_regs::PMCR_BASE + (port as u32) * 0x100;
            let pmcr_val = mt7531_regs::MT7531_FORCE_MODE_LNK     // Enable force link
                | mt7531_regs::MT7531_FORCE_MODE_SPD              // Enable force speed
                | mt7531_regs::MT7531_FORCE_MODE_DPX              // Enable force duplex
                | mt7531_regs::PMCR_FORCE_LNK                     // Link up
                | mt7531_regs::PMCR_FORCE_DPX                     // Full duplex
                | mt7531_regs::PMCR_FORCE_SPD_100                 // 100Mbps
                | mt7531_regs::PMCR_TX_EN
                | mt7531_regs::PMCR_RX_EN
                | mt7531_regs::PMCR_BACKPR_EN
                | mt7531_regs::PMCR_BKOFF_EN;
            self.sw_write(pmcr_addr, pmcr_val);

            uinfo!("ethd", "switch_user_port"; port = port as u32);
        }

        // CPU port matrix - allow forwarding to all user ports
        let cpu_pcr_addr = mt7531_regs::PCR_BASE + (cpu_port as u32) * 0x100;
        let cpu_matrix = 0x0F; // Ports 0-3
        self.sw_write(cpu_pcr_addr, mt7531_regs::pcr_matrix(cpu_matrix));

        // Initialize internal PHYs for ports 0-3
        for port in 0..mt7531_regs::NUM_USER_PORTS {
            self.init_phy(port);
        }

        // Longer delay for PHY auto-negotiation to complete
        delay_us(500_000); // 500ms for AN

        // Read back port and PHY status
        for port in 0..mt7531_regs::NUM_USER_PORTS {
            // Read PHY BMSR (reg 1) TWICE - first read clears latched status
            let _ = self.sw_phy_read(port, 1); // Clear latch
            let bmsr = self.sw_phy_read(port, 1).unwrap_or(0); // Real status
            let phy_link = if bmsr & 0x0004 != 0 { "up" } else { "down" };

            // Read switch port status
            let pmsr_addr = mt7531_regs::PMSR_BASE + (port as u32) * 0x100;
            let pmsr = self.sw_read(pmsr_addr).unwrap_or(0);
            let sw_link = if pmsr & mt7531_regs::PMSR_LINK != 0 { "up" } else { "down" };

            uinfo!("ethd", "port_status"; port = port as u32, phy_link = phy_link, sw_link = sw_link, bmsr = userlib::ulog::hex32(bmsr as u32), pmsr = userlib::ulog::hex32(pmsr));
        }

        // Check CPU port (port 6) status - CRITICAL for packet flow!
        let cpu_pmsr_addr = mt7531_regs::PMSR_BASE + (cpu_port as u32) * 0x100;
        let cpu_pmsr = self.sw_read(cpu_pmsr_addr).unwrap_or(0xDEAD);
        let cpu_pmcr_addr = mt7531_regs::PMCR_BASE + (cpu_port as u32) * 0x100;
        let cpu_pmcr = self.sw_read(cpu_pmcr_addr).unwrap_or(0xDEAD);
        uinfo!("ethd", "cpu_port"; port = cpu_port as u32, pmsr = userlib::ulog::hex32(cpu_pmsr), pmcr = userlib::ulog::hex32(cpu_pmcr));

        // Also check GMAC1 MCR status
        if let Some(fe) = &self.fe {
            let mcr = fe.read32(mac_regs::GMAC1_BASE + mac_regs::MAC_MCR);
            uinfo!("ethd", "gmac1_mcr"; val = userlib::ulog::hex32(mcr));
        }

        uinfo!("ethd", "switch_init_done";);
    }

    /// Initialize an internal PHY for auto-negotiation
    fn init_phy(&self, phy_addr: u8) {
        // Read PHY ID to verify presence (via switch's internal MDIO)
        let id_hi = self.sw_phy_read(phy_addr, 2).unwrap_or(0);
        let id_lo = self.sw_phy_read(phy_addr, 3).unwrap_or(0);
        let phy_id = ((id_hi as u32) << 16) | (id_lo as u32);

        if phy_id == 0 || phy_id == 0xFFFFFFFF {
            uwarn!("ethd", "phy_not_found"; addr = phy_addr as u32);
            return;
        }

        uinfo!("ethd", "phy_found"; addr = phy_addr as u32, id = userlib::ulog::hex32(phy_id));

        // Don't reconfigure PHY - bootloader should have set it up
        // Just read current status
        let bmcr = self.sw_phy_read(phy_addr, 0).unwrap_or(0);
        let anar = self.sw_phy_read(phy_addr, 4).unwrap_or(0);
        let bmsr = self.sw_phy_read(phy_addr, 1).unwrap_or(0);

        // Check power-down bit (bit 11) in BMCR
        let power_down = if bmcr & 0x0800 != 0 { "yes" } else { "no" };
        let link = if bmsr & 0x0004 != 0 { "up" } else { "down" };

        uinfo!("ethd", "phy_status"; addr = phy_addr as u32, bmcr = userlib::ulog::hex32(bmcr as u32), anar = userlib::ulog::hex32(anar as u32), link = link, pdown = power_down);
    }

    /// Initialize TX ring (QDMA)
    fn init_tx_ring(&mut self) {
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };
        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };

        let ring_paddr = dma.paddr() + LAYOUT.tx_ring_off as u64;
        let ring_vaddr = dma.vaddr() + LAYOUT.tx_ring_off as u64;

        // Initialize TX descriptors as LINKED LIST via txd2
        // CRITICAL: txd2 = next descriptor physical address (NOT buffer high bits!)
        // Linux maintains linked list for QDMA navigation
        for i in 0..TX_RING_SIZE {
            let desc_ptr = (ring_vaddr + (i * 32) as u64) as *mut TxDesc;
            let next_idx = (i + 1) % TX_RING_SIZE;
            let next_paddr = ring_paddr + (next_idx * 32) as u64;

            unsafe {
                let desc = &mut *desc_ptr;
                *desc = TxDesc::zeroed();
                // txd1 = 0 initially (buffer address filled at TX time)
                desc.txd1 = 0;
                // txd2 = NEXT DESCRIPTOR POINTER (linked list!)
                desc.txd2 = next_paddr as u32;
                // txd3 = LS0 | OWNER_CPU (Linux mtk_tx_alloc:2665)
                desc.txd3 = TX_DMA_LS0 | TX_DMA_OWNER_CPU;
                desc.txd4 = 0;
                desc.txd5 = 0;
                desc.txd6 = 0;
                desc.txd7 = 0;
                desc.txd8 = 0;
            }
        }

        // Configure QDMA TX ring
        let qdma_base = qdma_regs::QDMA_BASE;

        // Reset QDMA TX indices (Linux: mtk_w32(eth, RST_TX_MASK, rst_idx))
        fe.write32(qdma_base + qdma_regs::QDMA_RST_IDX, 0xFFFF);
        delay_us(100);
        fe.write32(qdma_base + qdma_regs::QDMA_RST_IDX, 0);

        // TX ring base pointers
        fe.write32(qdma_base + qdma_regs::QDMA_CTX_PTR, ring_paddr as u32);
        fe.write32(qdma_base + qdma_regs::QDMA_DTX_PTR, ring_paddr as u32);

        // CRX/DRX point to LAST descriptor (Linux does this!)
        let last_desc_paddr = ring_paddr + ((TX_RING_SIZE - 1) * 32) as u64;
        fe.write32(qdma_base + qdma_regs::QDMA_CRX_PTR, last_desc_paddr as u32);
        fe.write32(qdma_base + qdma_regs::QDMA_DRX_PTR, last_desc_paddr as u32);

        // Debug: read back to verify writes
        let ctx_rb = fe.read32(qdma_base + qdma_regs::QDMA_CTX_PTR);
        let dtx_rb = fe.read32(qdma_base + qdma_regs::QDMA_DTX_PTR);
        let crx_rb = fe.read32(qdma_base + qdma_regs::QDMA_CRX_PTR);
        let drx_rb = fe.read32(qdma_base + qdma_regs::QDMA_DRX_PTR);
        uinfo!("ethd", "tx_ptrs_after_init"; ctx = userlib::ulog::hex32(ctx_rb), dtx = userlib::ulog::hex32(dtx_rb), crx = userlib::ulog::hex32(crx_rb), drx = userlib::ulog::hex32(drx_rb));

        self.tx_head = 0;
        self.tx_tail = 0;

        uinfo!("ethd", "tx_ring_init"; paddr = userlib::ulog::hex64(ring_paddr), size = TX_RING_SIZE as u32);
    }

    /// Initialize free queue (QDMA requires this)
    fn init_free_queue(&self) {
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };
        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };

        let fq_ring_paddr = dma.paddr() + LAYOUT.fq_ring_off as u64;
        let fq_ring_vaddr = dma.vaddr() + LAYOUT.fq_ring_off as u64;

        // Initialize free queue descriptors as a LINEAR list (not circular!)
        // Linux mtk_init_fq_dma(): txd3 = buffer length, last desc txd2=0
        for i in 0..FQ_SIZE {
            let desc_ptr = (fq_ring_vaddr + (i * 32) as u64) as *mut TxDesc;
            let buf_paddr = dma.paddr() + (LAYOUT.fq_buf_off + i * TX_BUF_SIZE) as u64;

            unsafe {
                let desc = &mut *desc_ptr;
                *desc = TxDesc::zeroed();
                desc.txd1 = buf_paddr as u32;
                // Link to next, but LAST descriptor has txd2=0 (end of list)
                if i < FQ_SIZE - 1 {
                    desc.txd2 = (fq_ring_paddr + ((i + 1) * 32) as u64) as u32;
                } else {
                    desc.txd2 = 0; // End of list
                }
                // txd3 = buffer length | ADDR64 (high 4 address bits for 36-bit DMA)
                // Linux: TX_DMA_PLEN0(PAGE_SIZE) | TX_DMA_PREP_ADDR64(dma_addr)
                desc.txd3 = tx_dma_plen0(TX_BUF_SIZE) | tx_dma_prep_addr64(buf_paddr);
                // txd4-8 = 0 for V2+
                desc.txd4 = 0;
                desc.txd5 = 0;
                desc.txd6 = 0;
                desc.txd7 = 0;
                desc.txd8 = 0;
            }
        }

        // Configure free queue registers
        // Format from Linux mtk_eth_soc.c:mtk_init_fq_dma():
        //   fq_count = (cnt << 16) | cnt  (high = num allocated, low = current count)
        //   fq_blen  = buf_size << 16     (buffer length in high 16 bits)
        let qdma_base = qdma_regs::QDMA_BASE;
        fe.write32(qdma_base + qdma_regs::QDMA_FQ_HEAD, fq_ring_paddr as u32);
        let fq_tail_paddr = fq_ring_paddr + ((FQ_SIZE - 1) * 32) as u64;
        fe.write32(qdma_base + qdma_regs::QDMA_FQ_TAIL, fq_tail_paddr as u32);
        let fq_count_val = ((FQ_SIZE as u32) << 16) | (FQ_SIZE as u32);
        // FQ_BLEN: buffer size in bits 31-16
        // Linux: MTK_QDMA_PAGE_SIZE << 16 = 2048 << 16 = 0x08000000
        let fq_blen_val = (TX_BUF_SIZE as u32) << 16;  // 2048 << 16 = 0x08000000
        fe.write32(qdma_base + qdma_regs::QDMA_FQ_COUNT, fq_count_val);
        fe.write32(qdma_base + qdma_regs::QDMA_FQ_BLEN, fq_blen_val);

        uinfo!("ethd", "fq_init"; paddr = userlib::ulog::hex64(fq_ring_paddr), size = FQ_SIZE as u32, count = userlib::ulog::hex32(fq_count_val), blen = userlib::ulog::hex32(fq_blen_val));
    }

    /// Initialize RX ring (PDMA)
    fn init_rx_ring(&mut self) {
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };
        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };

        let ring_paddr = dma.paddr() + LAYOUT.rx_ring_off as u64;
        let ring_vaddr = dma.vaddr() + LAYOUT.rx_ring_off as u64;

        // Initialize RX descriptors
        // Linux: rxd2 = RX_DMA_PREP_PLEN0(buf_size) = (buf_size & dma_max_len) << dma_len_offset
        // MT7988: dma_len_offset=8, dma_max_len=0xffff (MTK_TX_DMA_BUF_LEN_V2)
        let rxd2_prep = ((RX_BUF_SIZE as u32) & 0xffff) << 8;

        for i in 0..RX_RING_SIZE {
            let desc_ptr = (ring_vaddr + (i * 32) as u64) as *mut RxDesc;
            let buf_paddr = dma.paddr() + (LAYOUT.rx_buf_off + i * RX_BUF_SIZE) as u64;

            unsafe {
                let desc = &mut *desc_ptr;
                *desc = RxDesc::zeroed();
                desc.rxd1 = buf_paddr as u32;
                // rxd2: buffer size in bits 8-23 (Linux RX_DMA_PREP_PLEN0 for MT7988)
                // DMA owns descriptor when DONE bit (31) is clear
                desc.rxd2 = rxd2_prep;
                // rxd3: high 4 address bits for 36-bit DMA (MTK_36BIT_DMA)
                // Linux: RX_DMA_PREP_ADDR64(dma_addr)
                desc.rxd3 = rx_dma_prep_addr64(buf_paddr);
            }
        }

        uinfo!("ethd", "rx_desc_prep"; rxd2 = userlib::ulog::hex32(rxd2_prep));

        // Configure PDMA RX ring
        let pdma_base = pdma_regs::PDMA_BASE;

        // Reset PDMA RX index first (Linux: MTK_PST_DRX_IDX_CFG)
        fe.write32(pdma_base + pdma_regs::PDMA_RST_IDX, 1 << 16); // Reset RX ring 0
        delay_us(100);

        fe.write32(pdma_base + pdma_regs::PDMA_RX_PTR, ring_paddr as u32);
        fe.write32(pdma_base + pdma_regs::PDMA_RX_MAX_CNT, RX_RING_SIZE as u32);
        fe.write32(pdma_base + pdma_regs::PDMA_RX_CPU_IDX, (RX_RING_SIZE - 1) as u32);

        self.rx_head = 0;

        uinfo!("ethd", "rx_ring_init"; paddr = userlib::ulog::hex64(ring_paddr), size = RX_RING_SIZE as u32);
    }

    /// Initialize QDMA RX ring (for TX completion feedback)
    /// Linux allocates this in mtk_rx_alloc() with MTK_RX_FLAGS_QDMA.
    /// The ring is set up but never polled - hardware may use it internally.
    fn init_qdma_rx_ring(&self) {
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };
        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };

        let ring_paddr = dma.paddr() + LAYOUT.qdma_rx_ring_off as u64;
        let ring_vaddr = dma.vaddr() + LAYOUT.qdma_rx_ring_off as u64;

        // Initialize QDMA RX descriptors (same format as PDMA RX)
        // Linux: rxd2 = RX_DMA_PREP_PLEN0(buf_size) = (buf_size & dma_max_len) << dma_len_offset
        // MT7988: dma_len_offset=8, dma_max_len=0xffff
        let rxd2_prep = ((RX_BUF_SIZE as u32) & 0xffff) << 8;

        for i in 0..QDMA_RX_RING_SIZE {
            let desc_ptr = (ring_vaddr + (i * 32) as u64) as *mut RxDesc;
            let buf_paddr = dma.paddr() + (LAYOUT.qdma_rx_buf_off + i * RX_BUF_SIZE) as u64;

            unsafe {
                let desc = &mut *desc_ptr;
                *desc = RxDesc::zeroed();
                desc.rxd1 = buf_paddr as u32;
                desc.rxd2 = rxd2_prep;
                // rxd3: high 4 address bits for 36-bit DMA (MTK_36BIT_DMA)
                desc.rxd3 = rx_dma_prep_addr64(buf_paddr);
            }
        }

        // Configure QDMA RX ring registers
        // Linux: qdma.rx_ptr, qdma.rx_cnt_cfg, qdma.rst_idx, qcrx_ptr
        let qdma_base = qdma_regs::QDMA_BASE;

        // Reset QDMA RX index (Linux: MTK_PST_DRX_IDX_CFG(0) = BIT(16))
        fe.write32(qdma_base + qdma_regs::QDMA_RST_IDX, 1 << 16);
        delay_us(100);

        // Set ring base and size
        fe.write32(qdma_base + qdma_regs::QDMA_RX_PTR, ring_paddr as u32);
        fe.write32(qdma_base + qdma_regs::QDMA_RX_CNT_CFG, QDMA_RX_RING_SIZE as u32);

        // Set CPU index to last descriptor (Linux: ring->calc_idx = rx_dma_size - 1)
        fe.write32(qdma_base + qdma_regs::QDMA_QCRX_PTR, (QDMA_RX_RING_SIZE - 1) as u32);

        uinfo!("ethd", "qdma_rx_ring_init"; paddr = userlib::ulog::hex64(ring_paddr), size = QDMA_RX_RING_SIZE as u32);
    }

    /// Enable DMA engines
    fn enable_dma(&mut self) {
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };

        let qdma_base = qdma_regs::QDMA_BASE;

        // Initialize TX queue config and scheduler for all 16 queues
        // Linux: both qtx_cfg AND qtx_sch must be configured!
        let qtx_cfg_val = (qdma_regs::QDMA_RES_THRES << 8) | qdma_regs::QDMA_RES_THRES;
        let qtx_sch_val = qdma_regs::QTX_SCH_MIN_RATE_EN
            | qdma_regs::QTX_SCH_MIN_RATE_MAN
            | qdma_regs::QTX_SCH_MIN_RATE_EXP
            | qdma_regs::QTX_SCH_LEAKY_BUCKET_SIZE;

        for i in 0..16 {
            fe.write32(qdma_base + qdma_regs::QDMA_QTX_CFG + i * 0x10, qtx_cfg_val);
            fe.write32(qdma_base + qdma_regs::QDMA_QTX_SCH + i * 0x10, qtx_sch_val);
        }

        // TX scheduler rate (Linux: MTK_QDMA_TX_SCH_MAX_WFQ | (same << 16))
        let tx_sch_rate = qdma_regs::QDMA_TX_SCH_MAX_WFQ
            | (qdma_regs::QDMA_TX_SCH_MAX_WFQ << 16);
        fe.write32(qdma_base + qdma_regs::QDMA_TX_SCH_RATE, tx_sch_rate);
        fe.write32(qdma_base + qdma_regs::QDMA_TX_SCH_RATE + 4, tx_sch_rate);

        uinfo!("ethd", "tx_queues_configured"; qtx_cfg = userlib::ulog::hex32(qtx_cfg_val), qtx_sch = userlib::ulog::hex32(qtx_sch_val));

        // QDMA flow control (Linux: QDMA_FC_TH, QDMA_HRED)
        let fc_th = qdma_regs::FC_THRES_DROP_MODE
            | qdma_regs::FC_THRES_DROP_EN
            | qdma_regs::FC_THRES_MIN;
        fe.write32(qdma_base + qdma_regs::QDMA_FC_TH, fc_th);
        fe.write32(qdma_base + qdma_regs::QDMA_HRED, 0);

        uinfo!("ethd", "qdma_flow_ctrl"; fc_th = userlib::ulog::hex32(fc_th));

        // Enable QDMA (TX) with V2+ flags from Linux mtk_start_dma()
        let qdma_cfg = qdma_regs::TX_DMA_EN
            | qdma_regs::RX_DMA_EN  // Linux enables both
            | qdma_regs::TX_BT_32DWORDS
            | qdma_regs::NDP_CO_PRO
            | qdma_regs::RX_2B_OFFSET
            | qdma_regs::TX_WB_DDONE
            // V2+ specific flags (critical for TX completion!)
            | qdma_regs::MULTI_CNT
            | qdma_regs::RESV_BUF
            | qdma_regs::WCOMP_EN
            | qdma_regs::DMAD_WR_WDONE
            | qdma_regs::CHK_DDONE_EN;
        fe.write32(qdma_base + qdma_regs::QDMA_GLO_CFG, qdma_cfg);

        uinfo!("ethd", "qdma_glo_cfg"; val = userlib::ulog::hex32(qdma_cfg));

        // Enable PDMA (RX) - Linux mtk_start_dma:3481-3484
        let pdma_base = pdma_regs::PDMA_BASE;
        let pdma_cfg = pdma_regs::RX_DMA_EN
            | pdma_regs::RX_BT_32DWORDS
            | pdma_regs::RX_2B_OFFSET
            | pdma_regs::MULTI_EN;  // MTK_MULTI_EN - required by Linux
        fe.write32(pdma_base + pdma_regs::PDMA_GLO_CFG, pdma_cfg);

        // Enable TX/RX interrupts - Linux mtk_open() does this
        // Even though we use polling, DMA engine might need IRQs enabled to function
        // mtk_tx_irq_enable(eth, MTK_TX_DONE_INT): tx_irq_mask |= BIT(28)
        let tx_irq_mask = fe.read32(fe_regs::TX_IRQ_MASK);
        fe.write32(fe_regs::TX_IRQ_MASK, tx_irq_mask | fe_regs::TX_DONE_INT);

        // mtk_rx_irq_enable(eth, MTK_RX_DONE_INT_V2): pdma.irq_mask |= BIT(14)
        let rx_irq_mask = fe.read32(fe_regs::PDMA_IRQ_MASK);
        fe.write32(fe_regs::PDMA_IRQ_MASK, rx_irq_mask | fe_regs::RX_DONE_INT_V2);

        uinfo!("ethd", "irq_enabled"; tx_mask = userlib::ulog::hex32(tx_irq_mask | fe_regs::TX_DONE_INT), rx_mask = userlib::ulog::hex32(rx_irq_mask | fe_regs::RX_DONE_INT_V2));

        self.link_up = true;

        // Debug: read Frame Engine and QDMA status
        let fe_int_status = fe.read32(fe_regs::FE_INT_STATUS);
        let gdm1_cfg = fe.read32(gdm_regs::GDM1_FWD_CFG);
        let mac_misc = fe.read32(mac_misc::MAC_MISC_V3);
        let qdma_glo = fe.read32(qdma_base + qdma_regs::QDMA_GLO_CFG);
        let pdma_glo = fe.read32(pdma_base + pdma_regs::PDMA_GLO_CFG);
        uinfo!("ethd", "fe_final"; fe_int = userlib::ulog::hex32(fe_int_status), gdm1 = userlib::ulog::hex32(gdm1_cfg), mac_misc = userlib::ulog::hex32(mac_misc));
        uinfo!("ethd", "dma_final"; qdma = userlib::ulog::hex32(qdma_glo), pdma = userlib::ulog::hex32(pdma_glo));

        // Re-write TX ring pointers AFTER enabling DMA
        // (enabling DMA may reset them)
        let dma = match &self.dma {
            Some(d) => d,
            None => {
                uinfo!("ethd", "dma_enabled";);
                return;
            }
        };
        let ring_paddr = dma.paddr() + LAYOUT.tx_ring_off as u64;
        let last_desc_paddr = ring_paddr + ((TX_RING_SIZE - 1) * 32) as u64;
        fe.write32(qdma_base + qdma_regs::QDMA_CTX_PTR, ring_paddr as u32);
        fe.write32(qdma_base + qdma_regs::QDMA_DTX_PTR, ring_paddr as u32);
        fe.write32(qdma_base + qdma_regs::QDMA_CRX_PTR, last_desc_paddr as u32);
        fe.write32(qdma_base + qdma_regs::QDMA_DRX_PTR, last_desc_paddr as u32);

        // Debug: verify pointers after re-write
        let ctx_rb = fe.read32(qdma_base + qdma_regs::QDMA_CTX_PTR);
        let dtx_rb = fe.read32(qdma_base + qdma_regs::QDMA_DTX_PTR);
        uinfo!("ethd", "tx_ptrs_after_dma_en"; ctx = userlib::ulog::hex32(ctx_rb), dtx = userlib::ulog::hex32(dtx_rb));

        uinfo!("ethd", "dma_enabled";);
    }

    /// Poll RX ring for received packets
    fn poll_rx(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = match self.port_id {
            Some(id) => id,
            None => return,
        };
        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };

        let ring_vaddr = dma.vaddr() + LAYOUT.rx_ring_off as u64;
        let mut processed = 0;

        // Debug: read first RX descriptor and PDMA status (once per 100 polls)
        static mut POLL_COUNT: u32 = 0;
        unsafe {
            POLL_COUNT += 1;
            if POLL_COUNT % 100 == 1 {
                let desc_ptr = (ring_vaddr) as *const RxDesc;
                let desc = &*desc_ptr;
                let pdma_base = pdma_regs::PDMA_BASE;
                let glo_cfg = fe.read32(pdma_base + pdma_regs::PDMA_GLO_CFG);
                let cpu_idx = fe.read32(pdma_base + pdma_regs::PDMA_RX_CPU_IDX);
                let dma_idx = fe.read32(pdma_base + pdma_regs::PDMA_RX_DMA_IDX);
                // Also check QDMA DTX to see if it changes during RX poll
                let qdma_base = qdma_regs::QDMA_BASE;
                let qdma_dtx = fe.read32(qdma_base + qdma_regs::QDMA_DTX_PTR);
                uinfo!("ethd", "pdma_poll"; glo = userlib::ulog::hex32(glo_cfg), cpu = cpu_idx, dma = dma_idx, rxd2 = userlib::ulog::hex32(desc.rxd2), qdma_dtx = userlib::ulog::hex32(qdma_dtx));
            }
        }

        loop {
            let desc_ptr = (ring_vaddr + (self.rx_head * 32) as u64) as *const RxDesc;
            let desc = unsafe { &*desc_ptr };

            // Check if DMA has written this descriptor
            if (desc.rxd2 & RX_DMA_DONE) == 0 {
                break;
            }

            processed += 1;

            // Get packet length from rxd2 (MT7988 NETSYS_V3 format)
            // Linux: pktlen = RX_DMA_GET_PLEN0(trxd.rxd2) at mtk_poll_rx:2240
            // MT7988: (rxd2 >> 8) & 0xffff
            // Note: Length INCLUDES metadata (2-byte padding + 16-byte RX info = 18 bytes)
            let raw_len = rx_dma_plen_v2(desc.rxd2);

            // Debug: dump bytes from OFFSET 0 to see actual DMA layout
            let buf_base = dma.vaddr() + (LAYOUT.rx_buf_off + self.rx_head * RX_BUF_SIZE) as u64;
            let raw_bytes = unsafe { core::slice::from_raw_parts(buf_base as *const u8, 40.min(raw_len)) };
            let mut raw_hex = [0u8; 120];
            for (i, &b) in raw_bytes.iter().take(40).enumerate() {
                const HEX: &[u8; 16] = b"0123456789abcdef";
                raw_hex[i * 3] = HEX[(b >> 4) as usize];
                raw_hex[i * 3 + 1] = HEX[(b & 0xf) as usize];
                raw_hex[i * 3 + 2] = b' ';
            }
            let raw_str = core::str::from_utf8(&raw_hex[..raw_bytes.len().min(40) * 3]).unwrap_or("?");
            // Also show descriptor rxd1 (buffer paddr) and expected buf_paddr
            let buf_paddr = dma.paddr() + (LAYOUT.rx_buf_off + self.rx_head * RX_BUF_SIZE) as u64;
            uinfo!("ethd", "rx_raw"; idx = self.rx_head as u32, raw_len = raw_len as u32, rxd1 = userlib::ulog::hex32(desc.rxd1), buf_p = userlib::ulog::hex32(buf_paddr as u32), bytes = raw_str);

            let frame_vaddr = buf_base + RX_FRAME_OFFSET as u64;
            let frame_len = raw_len.saturating_sub(RX_FRAME_OFFSET);

            // Use frame_len for validation (after stripping metadata)
            let len = frame_len;
            if len == 0 || len > RX_BUF_SIZE - RX_FRAME_OFFSET {
                // Invalid length, skip
                self.reclaim_rx_desc(self.rx_head);
                self.rx_head = (self.rx_head + 1) % RX_RING_SIZE;
                continue;
            }

            // Copy frame to DataPort pool for consumer
            if let Some(port) = ctx.block_port(port_id) {
                if let Some(pool_offset) = port.alloc(len as u32) {
                    // Copy from DMA buffer (skip 18-byte header: 2-byte padding + 16-byte RX info)
                    let frame_data = unsafe {
                        core::slice::from_raw_parts(frame_vaddr as *const u8, len)
                    };
                    port.pool_write(pool_offset, frame_data);

                    // Post CQE to notify consumer
                    let tag = self.rx_seq;
                    self.rx_seq = self.rx_seq.wrapping_add(1);
                    let cqe = IoCqe {
                        status: io_status::OK,
                        flags: 0,
                        tag,
                        transferred: frame_len as u32,
                        result: pool_offset,
                    };
                    port.complete(&cqe);
                }
            }

            // Reclaim descriptor
            self.reclaim_rx_desc(self.rx_head);

            // Advance
            let old_head = self.rx_head;
            self.rx_head = (self.rx_head + 1) % RX_RING_SIZE;

            // Update CPU index
            let pdma_base = pdma_regs::PDMA_BASE;
            fe.write32(pdma_base + pdma_regs::PDMA_RX_CPU_IDX, old_head as u32);

            if processed >= 16 {
                break;
            }
        }

        if processed > 0 {
            if let Some(port) = ctx.block_port(port_id) {
                port.notify();
            }
        }
    }

    /// Reclaim an RX descriptor for reuse
    fn reclaim_rx_desc(&self, idx: usize) {
        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };

        let ring_vaddr = dma.vaddr() + LAYOUT.rx_ring_off as u64;
        let desc_ptr = (ring_vaddr + (idx * 32) as u64) as *mut RxDesc;
        let buf_paddr = dma.paddr() + (LAYOUT.rx_buf_off + idx * RX_BUF_SIZE) as u64;

        // Linux: rxd2 = RX_DMA_PREP_PLEN0(buf_size) with DONE bit clear
        // MT7988: dma_len_offset=8, dma_max_len=0xffff
        let rxd2_prep = ((RX_BUF_SIZE as u32) & 0xffff) << 8;

        unsafe {
            let desc = &mut *desc_ptr;
            desc.rxd1 = buf_paddr as u32;
            desc.rxd2 = rxd2_prep; // Buffer size in bits 8-23, DONE clear = DMA owns
            // rxd3: high 4 address bits for 36-bit DMA (MTK_36BIT_DMA)
            desc.rxd3 = rx_dma_prep_addr64(buf_paddr);
        }
    }

    /// Process TX requests from DataPort SQ
    fn process_tx(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = match self.port_id {
            Some(id) => id,
            None => return,
        };
        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };

        // Collect TX requests
        let mut requests: [Option<IoSqe>; 8] = [None; 8];
        let mut req_count = 0;

        if let Some(port) = ctx.block_port(port_id) {
            while req_count < 8 {
                if let Some(sqe) = port.recv_request() {
                    requests[req_count] = Some(sqe);
                    req_count += 1;
                } else {
                    break;
                }
            }
        }

        if req_count == 0 {
            return;
        }

        uinfo!("ethd", "tx_req"; count = req_count as u32);

        // Debug: read QDMA status
        let qdma_base = qdma_regs::QDMA_BASE;
        let glo_cfg = fe.read32(qdma_base + qdma_regs::QDMA_GLO_CFG);

        // Debug: check DTX at start of TX handler - is it already 0?
        let dtx_at_start = fe.read32(qdma_base + qdma_regs::QDMA_DTX_PTR);
        let ctx_at_start = fe.read32(qdma_base + qdma_regs::QDMA_CTX_PTR);
        uinfo!("ethd", "qdma_status"; glo_cfg = userlib::ulog::hex32(glo_cfg), ctx = userlib::ulog::hex32(ctx_at_start), dtx = userlib::ulog::hex32(dtx_at_start));

        let ring_vaddr = dma.vaddr() + LAYOUT.tx_ring_off as u64;
        let ring_paddr = dma.paddr() + LAYOUT.tx_ring_off as u64;

        for i in 0..req_count {
            let sqe = match requests[i].take() {
                Some(s) => s,
                None => continue,
            };

            if sqe.opcode != net_op::TX {
                if let Some(port) = ctx.block_port(port_id) {
                    port.complete_error(sqe.tag, io_status::INVALID);
                    port.notify();
                }
                continue;
            }

            let packet_len = sqe.data_len as usize;
            if packet_len == 0 || packet_len > 1514 {
                if let Some(port) = ctx.block_port(port_id) {
                    port.complete_error(sqe.tag, io_status::INVALID);
                    port.notify();
                }
                continue;
            }

            // Check if TX ring is full
            let next_head = (self.tx_head + 1) % TX_RING_SIZE;
            if next_head == self.tx_tail {
                // Ring full
                if let Some(port) = ctx.block_port(port_id) {
                    port.complete_error(sqe.tag, io_status::NO_SPACE);
                    port.notify();
                }
                continue;
            }

            // Get descriptor
            let desc_ptr = (ring_vaddr + (self.tx_head * 32) as u64) as *mut TxDesc;
            let buf_vaddr = dma.vaddr() + (LAYOUT.tx_buf_off + self.tx_head * TX_BUF_SIZE) as u64;
            let buf_paddr = dma.paddr() + (LAYOUT.tx_buf_off + self.tx_head * TX_BUF_SIZE) as u64;

            // Copy packet from pool to TX buffer
            if let Some(port) = ctx.block_port(port_id) {
                if let Some(src) = port.pool_slice(sqe.data_offset, sqe.data_len) {
                    unsafe {
                        core::ptr::copy_nonoverlapping(
                            src.as_ptr(),
                            buf_vaddr as *mut u8,
                            packet_len,
                        );
                    }
                } else {
                    port.complete_error(sqe.tag, io_status::INVALID);
                    port.notify();
                    continue;
                }
            }

            // Debug: show first 20 bytes of TX packet
            let tx_hdr = unsafe { core::slice::from_raw_parts(buf_vaddr as *const u8, 20.min(packet_len)) };
            let mut tx_hex = [0u8; 60];
            for (i, &b) in tx_hdr.iter().take(20).enumerate() {
                const HEX: &[u8; 16] = b"0123456789abcdef";
                tx_hex[i * 3] = HEX[(b >> 4) as usize];
                tx_hex[i * 3 + 1] = HEX[(b & 0xf) as usize];
                tx_hex[i * 3 + 2] = b' ';
            }
            let tx_str = core::str::from_utf8(&tx_hex[..tx_hdr.len().min(20) * 3]).unwrap_or("?");
            uinfo!("ethd", "tx_pkt"; idx = self.tx_head as u32, len = packet_len as u32, hdr = tx_str);

            // Fill descriptor (V2/V3 format for MT7988)
            // Based on Linux mtk_tx_set_dma_desc_v2() and audit
            unsafe {
                let desc = &mut *desc_ptr;
                // txd1 = buffer address (low 32 bits)
                desc.txd1 = buf_paddr as u32;
                // txd2 = DO NOT TOUCH - it's the next descriptor pointer!
                // txd3 = LS0 | PLEN0 | ADDR64 (high 4 address bits for 36-bit DMA)
                // Linux: TX_DMA_PREP_ADDR64(addr) for MTK_36BIT_DMA systems
                desc.txd3 = TX_DMA_LS0 | tx_dma_plen0(packet_len) | tx_dma_prep_addr64(buf_paddr);
                // txd4 = FPORT | SWC_V2 | QID (FPORT in bits 11:8, QID in bits 21:16)
                // Linux: data = PSE_GDM1_PORT << TX_DMA_FPORT_SHIFT_V2 | TX_DMA_SWC_V2 | QID_BITS_V2(qid)
                desc.txd4 = tx_dma_txd4_v2(PSE_GDM1_PORT, 0);  // Queue 0
                // txd5 = 0 (TSO/CHKSUM flags, we don't use them)
                desc.txd5 = 0;
                desc.txd6 = 0;
                desc.txd7 = 0;
                desc.txd8 = 0;
            }

            // Debug: show descriptor contents after fill
            let desc = unsafe { &*(desc_ptr as *const TxDesc) };
            uinfo!("ethd", "tx_desc"; txd1 = userlib::ulog::hex32(desc.txd1), txd2 = userlib::ulog::hex32(desc.txd2), txd3 = userlib::ulog::hex32(desc.txd3), txd4 = userlib::ulog::hex32(desc.txd4), txd5 = userlib::ulog::hex32(desc.txd5));

            // Advance head
            self.tx_head = next_head;

            // Memory barrier to ensure descriptor writes complete before kick
            core::sync::atomic::fence(core::sync::atomic::Ordering::Release);

            // Kick QDMA by writing next descriptor address to CTX_PTR
            // This tells QDMA "here's a new descriptor to process"
            let qdma_base = qdma_regs::QDMA_BASE;
            let next_desc_paddr = ring_paddr + (self.tx_head * 32) as u64;
            fe.write32(qdma_base + qdma_regs::QDMA_CTX_PTR, next_desc_paddr as u32);

            // Debug: Read back TX registers and check for errors
            let ctx_ptr = fe.read32(qdma_base + qdma_regs::QDMA_CTX_PTR);
            let dtx_ptr = fe.read32(qdma_base + qdma_regs::QDMA_DTX_PTR);
            let crx_ptr = fe.read32(qdma_base + qdma_regs::QDMA_CRX_PTR);
            let drx_ptr = fe.read32(qdma_base + qdma_regs::QDMA_DRX_PTR);

            // Read FE interrupt status and free queue count for debug
            let fe_int = fe.read32(fe_regs::FE_INT_STATUS);
            let fq_cnt = fe.read32(qdma_base + qdma_regs::QDMA_FQ_COUNT);

            uinfo!("ethd", "tx_kick"; ctx = userlib::ulog::hex32(ctx_ptr), dtx = userlib::ulog::hex32(dtx_ptr), crx = userlib::ulog::hex32(crx_ptr), drx = userlib::ulog::hex32(drx_ptr), len = packet_len as u32);
            uinfo!("ethd", "tx_status"; fe_int = userlib::ulog::hex32(fe_int), fq_cnt = userlib::ulog::hex32(fq_cnt));

            // Debug: Read MAC TX counters to see if packet reached MAC
            // MT7988 GDM1 counters at FE_BASE + 0x1c00
            const GDM1_CNT_BASE: usize = 0x1c00;
            let tx_pkts = fe.read32(GDM1_CNT_BASE + 0x48);  // TX good packets
            let tx_bytes = fe.read32(GDM1_CNT_BASE + 0x40); // TX good bytes
            let tx_drop = fe.read32(GDM1_CNT_BASE + 0x60);  // TX drop packets

            // PSE queue status registers (Linux: PSE_OQ_STA, PSE_IQ_STA)
            let pse_oq_sta = fe.read32(0x01a0);   // Output queue status
            let pse_iq_sta = fe.read32(0x01b0);   // Input queue status

            // PSE Free Queue Flow Control (Linux: PSE_FQFC_CFG)
            let pse_fqfc = fe.read32(0x0100);    // Free queue config

            // PSE port drop counters - check if PSE is dropping our packet
            // PSE_MQ(x)_DROP at 0x1b8 + x*4 shows drops per port
            let pse_mq0_drop = fe.read32(0x01b8);  // Port 0 drops
            let pse_mq1_drop = fe.read32(0x01bc);  // Port 1 (GDM1) drops

            // QDMA debug: FSM state and error status
            let qdma_fsm = fe.read32(qdma_base + 0x0200);  // QDMA_FSM
            let qdma_err = fe.read32(qdma_base + 0x0204);  // QDMA_ERR (if exists)

            uinfo!("ethd", "mac_tx"; pkts = tx_pkts, bytes = tx_bytes, drop = tx_drop);
            uinfo!("ethd", "pse_status"; oq = userlib::ulog::hex32(pse_oq_sta), iq = userlib::ulog::hex32(pse_iq_sta), fqfc = userlib::ulog::hex32(pse_fqfc));
            uinfo!("ethd", "pse_drops"; mq0 = pse_mq0_drop, mq1 = pse_mq1_drop);
            uinfo!("ethd", "qdma_debug"; fsm = userlib::ulog::hex32(qdma_fsm), err = userlib::ulog::hex32(qdma_err));

            // Complete to consumer
            if let Some(port) = ctx.block_port(port_id) {
                port.complete_ok(sqe.tag, sqe.data_len);
                port.notify();
            }
        }
    }

    /// Reclaim completed TX descriptors
    /// QDMA uses DRX pointer for completion, not DONE bit in descriptor
    fn poll_tx_completions(&mut self) {
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };
        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };

        let ring_paddr = dma.paddr() + LAYOUT.tx_ring_off as u64;
        let ring_vaddr = dma.vaddr() + LAYOUT.tx_ring_off as u64;
        let qdma_base = qdma_regs::QDMA_BASE;

        // Read DRX pointer - where DMA has released/completed up to
        let drx_ptr = fe.read32(qdma_base + qdma_regs::QDMA_DRX_PTR);

        // Calculate DRX index from pointer
        if drx_ptr < ring_paddr as u32 {
            return; // Invalid pointer
        }
        let drx_offset = drx_ptr - ring_paddr as u32;
        let drx_idx = (drx_offset / 32) as usize;
        if drx_idx >= TX_RING_SIZE {
            return; // Out of range
        }

        let mut completed = 0u32;

        // Reclaim all descriptors from tx_tail up to (but not including) drx_idx
        // Linux: walks linked list via txd2 and checks OWNER_CPU bit
        // We use index increment but add OWNER_CPU sanity check
        while self.tx_tail != drx_idx && self.tx_tail != self.tx_head {
            // P4.1: Check OWNER_CPU bit as sanity check (like Linux mtk_poll_tx_qdma:2467)
            // When DMA completes, it sets OWNER_CPU=1 (returns ownership to CPU)
            // If OWNER_CPU=0, DMA still owns it - shouldn't happen if drx_ptr is valid, but check anyway
            let desc_vaddr = ring_vaddr + (self.tx_tail * 32) as u64;
            let desc = unsafe { &*(desc_vaddr as *const TxDesc) };
            if (desc.txd3 & TX_DMA_OWNER_CPU) == 0 {
                // DMA still owns this descriptor - stop reclaiming
                uinfo!("ethd", "tx_owner_dma"; idx = self.tx_tail as u32, txd3 = userlib::ulog::hex32(desc.txd3));
                break;
            }

            completed += 1;
            self.tx_tail = (self.tx_tail + 1) % TX_RING_SIZE;
        }

        if completed > 0 {
            // Update CRX pointer to acknowledge we've reclaimed these
            let new_crx = ring_paddr + (self.tx_tail * 32) as u64;
            fe.write32(qdma_base + qdma_regs::QDMA_CRX_PTR, new_crx as u32);
            uinfo!("ethd", "tx_completed"; count = completed);
        } else if self.tx_tail != self.tx_head {
            // Debug: show why no completion
            let dtx_ptr = fe.read32(qdma_base + qdma_regs::QDMA_DTX_PTR);
            uinfo!("ethd", "tx_pending"; tail = self.tx_tail as u32, drx_idx = drx_idx as u32, drx = userlib::ulog::hex32(drx_ptr), dtx = userlib::ulog::hex32(dtx_ptr));
        }
    }

    /// Handle sidechannel queries
    fn handle_side_queries(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = match self.port_id {
            Some(id) => id,
            None => return,
        };

        let mut queries: [Option<SideEntry>; 4] = [None; 4];
        let mut query_count = 0;

        if let Some(port) = ctx.block_port(port_id) {
            while query_count < 4 {
                if let Some(entry) = port.poll_side_request() {
                    queries[query_count] = Some(entry);
                    query_count += 1;
                } else {
                    break;
                }
            }
        }

        for i in 0..query_count {
            if let Some(entry) = queries[i].take() {
                match entry.msg_type {
                    SIDE_QUERY_NET_INFO => {
                        // Respond with MAC address and link status
                        let mut response = SideEntry {
                            msg_type: entry.msg_type,
                            flags: 0,
                            tag: entry.tag,
                            status: side_status::OK,
                            payload: [0; 24],
                        };
                        // payload[0..6] = MAC address
                        response.payload[0..6].copy_from_slice(&self.mac);
                        // payload[6] = link status
                        response.payload[6] = if self.link_up { 1 } else { 0 };
                        // payload[7..9] = MTU (1500)
                        response.payload[7..9].copy_from_slice(&1500u16.to_le_bytes());

                        if let Some(port) = ctx.block_port(port_id) {
                            port.side_send(&response);
                            port.notify();
                        }
                    }
                    _ => {
                        // Unknown query — respond with EOL
                        let response = SideEntry {
                            msg_type: entry.msg_type,
                            flags: 0,
                            tag: entry.tag,
                            status: side_status::EOL,
                            payload: [0; 24],
                        };
                        if let Some(port) = ctx.block_port(port_id) {
                            port.side_send(&response);
                            port.notify();
                        }
                    }
                }
            }
        }
    }

    fn format_mac(&self) -> [u8; 17] {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut buf = [0u8; 17];
        for i in 0..6 {
            buf[i * 3] = HEX[(self.mac[i] >> 4) as usize];
            buf[i * 3 + 1] = HEX[(self.mac[i] & 0xf) as usize];
            if i < 5 {
                buf[i * 3 + 2] = b':';
            }
        }
        buf
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for EthDriver {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        uinfo!("ethd", "init";);

        // 1. Map Frame Engine MMIO
        let fe = MmioRegion::open(FE_BASE, FE_SIZE).ok_or_else(|| {
            uerror!("ethd", "mmio_map_failed"; addr = userlib::ulog::hex64(FE_BASE));
            BusError::Internal
        })?;
        self.fe = Some(fe);

        // 2. Map ETHSYS MMIO (clock/reset/DMA_AG_MAP)
        let ethsys = MmioRegion::open(ETHSYS_BASE, ETHSYS_SIZE).ok_or_else(|| {
            uerror!("ethd", "ethsys_map_failed"; addr = userlib::ulog::hex64(ETHSYS_BASE));
            BusError::Internal
        })?;
        self.ethsys = Some(ethsys);

        // 4. Map Internal Switch (GSW) MMIO
        let gsw = MmioRegion::open(GSW_BASE, GSW_SIZE).ok_or_else(|| {
            uerror!("ethd", "gsw_map_failed"; addr = userlib::ulog::hex64(GSW_BASE));
            BusError::Internal
        })?;
        self.gsw = Some(gsw);

        // 5. Allocate DMA pool
        let mut dma = DmaPool::alloc(DMA_POOL_SIZE).ok_or_else(|| {
            uerror!("ethd", "dma_alloc_failed"; size = DMA_POOL_SIZE as u32);
            BusError::Internal
        })?;
        dma.zero();

        // Log DMA pool address - critical for 36-bit DMA debugging
        // If paddr > 0xFFFFFFFF, the high bits MUST be set in descriptors
        let dma_paddr = dma.paddr();
        let dma_hi = (dma_paddr >> 32) as u32;  // Should be non-zero if >4GB
        uinfo!("ethd", "dma_pool"; size = DMA_POOL_SIZE as u32, paddr = userlib::ulog::hex64(dma_paddr), hi32 = userlib::ulog::hex32(dma_hi));

        self.dma = Some(dma);

        // Linux order from mtk_hw_init (corrected after finding clock setup):
        // 0. mtk_clk_enable() - enables clocks via ETHSYS+0x30 (via clock framework)
        // 1. DMA_AG_MAP (line 4103-4105) - ETHSYS+0x408
        // 2. msleep(100) (line 4125)
        // 3. Reset sequence (mtk_hw_reset)

        // Step 0: Enable clocks FIRST! Without this, registers may not respond.
        // Linux does this via mtk_clk_enable() which goes through clock framework
        // to write ETHSYS+0x30 (clock gate register)
        self.enable_clocks();

        // Step 1: Initialize DMA address generation (before reset!)
        // This is at ETHSYS_BASE+0x408, enables PDMA/QDMA/PPE address generation
        self.init_dma_ag_map();

        // Step 2: 100ms delay before reset (Linux mtk_hw_init:4125)
        delay_us(100_000);

        // Step 3: Reset Frame Engine
        self.reset_fe();

        // 6. Configure GDM1 (forward RX to PDMA)
        self.configure_gdm();

        // 6. Configure MAC (GMAC1)
        self.configure_mac();

        // 7. Initialize MT7531 switch and internal PHYs
        self.init_switch();

        // 8. Wait for DMA to be idle before ring init (Linux mtk_dma_busy_wait)
        if !self.dma_busy_wait() {
            uerror!("ethd", "dma_busy_wait_failed";);
            // Continue anyway - may work
        }

        // 9. Initialize TX ring (QDMA)
        self.init_tx_ring();

        // 9. Initialize free queue (required by QDMA)
        self.init_free_queue();

        // 10. Initialize QDMA RX ring (for TX completion feedback - Linux does this)
        self.init_qdma_rx_ring();

        // 11. Initialize RX ring (PDMA)
        self.init_rx_ring();

        // 11. Enable DMA
        self.enable_dma();

        // 12. Create DataPort for network I/O
        let config = BlockPortConfig {
            ring_size: 64,
            side_size: 4,
            pool_size: 128 * 1024,
        };
        let port_id = ctx.create_block_port(config).map_err(|e| {
            uerror!("ethd", "create_block_port_failed";);
            e
        })?;
        if let Some(port) = ctx.block_port(port_id) {
            port.set_public();
        }
        self.port_id = Some(port_id);

        // 13. Register with devd
        let shmem_id = ctx.block_port(port_id).map(|p| p.shmem_id()).unwrap_or(0);
        let _ = ctx.register_port_with_metadata(b"net0", PortType::Network, shmem_id, None, &self.mac);

        let mac_str = self.format_mac();
        uinfo!("ethd", "ready"; mac = core::str::from_utf8(&mac_str).unwrap_or("?"));

        // 14. Start RX poll timer
        if let Ok(mut timer) = Timer::new() {
            let _ = timer.set(RX_POLL_INTERVAL_NS);
            let handle = timer.handle();
            let _ = ctx.watch_handle(handle, TAG_RX_POLL);
            self.rx_poll_timer = Some(timer);
        }

        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::QUERY_INFO => {
                let mac_str = self.format_mac();
                let mut info = [0u8; 128];
                let mut pos = 0;
                let prefix = b"mt7988-eth mac=";
                info[..prefix.len()].copy_from_slice(prefix);
                pos += prefix.len();
                info[pos..pos + mac_str.len()].copy_from_slice(&mac_str);
                pos += mac_str.len();
                let link_str: &[u8] = if self.link_up { b" link=up" } else { b" link=down" };
                info[pos..pos + link_str.len()].copy_from_slice(link_str);
                pos += link_str.len();
                let _ = ctx.respond_info(msg.seq_id, &info[..pos]);
                Disposition::Handled
            }
            _ => Disposition::Forward,
        }
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        if self.port_id != Some(port) {
            return;
        }

        // Process TX requests from consumer
        self.process_tx(ctx);

        // Check for received packets
        self.poll_rx(ctx);

        // Reclaim completed TX descriptors
        self.poll_tx_completions();

        // Handle sidechannel queries
        self.handle_side_queries(ctx);
    }

    fn handle_event(&mut self, tag: u32, _handle: Handle, ctx: &mut dyn BusCtx) {
        if tag == TAG_RX_POLL {
            // Read timer to clear fired state
            if let Some(ref mut timer) = self.rx_poll_timer {
                let _ = timer.wait();
            }

            // Poll RX and reclaim TX
            self.poll_rx(ctx);
            self.poll_tx_completions();

            // Re-arm timer
            if let Some(ref mut timer) = self.rx_poll_timer {
                let _ = timer.set(RX_POLL_INTERVAL_NS);
            }
        }
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: EthDriver = EthDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"ethd", EthDriverWrapper(driver));
}

struct EthDriverWrapper(&'static mut EthDriver);

impl Driver for EthDriverWrapper {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.init(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        self.0.data_ready(port, ctx)
    }

    fn handle_event(&mut self, tag: u32, handle: Handle, ctx: &mut dyn BusCtx) {
        self.0.handle_event(tag, handle, ctx)
    }
}
