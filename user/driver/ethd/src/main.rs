//! MT7988 Native Ethernet Driver (U-Boot style)
//!
//! Direct port of U-Boot's mtk_eth driver logic for GMAC0 + internal switch.
//! Uses PDMA only (no QDMA) - same as U-Boot.

#![no_std]
#![no_main]

mod uboot_mtk_eth;

use uboot_mtk_eth::*;
use userlib::syscall::Handle;
use userlib::mmio::{MmioRegion, DmaPool, delay_us, cache_clean, cache_invalidate};
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
// Hardware Constants
// =============================================================================

const FE_BASE: u64 = 0x1510_0000;
const FE_SIZE: u64 = 0x8_0000;

// MT7988 Internal Switch - SEPARATE memory region (not inside FE!)
// Reference: Linux DTS mt7988a.dtsi - switch@15020000
const SWITCH_BASE: u64 = 0x1502_0000;
const SWITCH_SIZE: u64 = 0x8000;

// Ethwarp clock/reset controller - controls switch reset
// Reference: Linux clk-mt7988-eth.c
const ETHWARP_BASE: u64 = 0x1503_1000;
const ETHWARP_SIZE: u64 = 0x1000;
const ETHWARP_RST_OFS: usize = 0x8;  // Reset register offset
const ETHWARP_RST_SWITCH_BIT: u32 = 1 << 9;  // Bit 9 = switch reset

// ETHSYS - System control for Frame Engine
// Reference: Linux mt7988a.dtsi - ethsys: syscon@15000000
// Reference: Linux clk-mt7988-eth.c - clock gate definitions
const ETHSYS_BASE: u64 = 0x1500_0000;
const ETHSYS_SIZE: u64 = 0x1000;
const ETHSYS_DUMMY_REG: usize = 0x0c;  // Dummy register with SRAM enable
const ETHSYS_SRAM_EN: u32 = 1 << 15;   // BIT(15) enables SRAM access (older chips)

// ETHSYS clock gate register at offset 0x30
// Reference: Linux clk-mt7988-eth.c ethdma_cg_regs
// 1 = clock enabled (inverted logic: mtk_clk_gate_ops_no_setclr_inv)
const ETHSYS_CLK_GATE_REG: usize = 0x30;
const ETHSYS_CLK_XGP1_EN: u32 = 1 << 0;   // 2.5G GMAC1
const ETHSYS_CLK_XGP2_EN: u32 = 1 << 1;   // 2.5G GMAC2
const ETHSYS_CLK_XGP3_EN: u32 = 1 << 2;   // 2.5G GMAC3
const ETHSYS_CLK_FE_EN: u32 = 1 << 6;     // Frame Engine (critical!)
const ETHSYS_CLK_GP2_EN: u32 = 1 << 7;    // GMAC2
const ETHSYS_CLK_GP1_EN: u32 = 1 << 8;    // GMAC1
const ETHSYS_CLK_GP3_EN: u32 = 1 << 10;   // GMAC3
const ETHSYS_CLK_ESW_EN: u32 = 1 << 16;   // Embedded Switch (critical!)
const ETHSYS_CLK_CRYPT0_EN: u32 = 1 << 29; // Crypto engine

// Pinctrl (GPIO) - for LED pin muxing
// Reference: Linux mt7988a.dtsi - pinctrl@1001f000
const PINCTRL_BASE: u64 = 0x1001_f000;
const PINCTRL_SIZE: u64 = 0x1000;
// Pin mode registers at GPIO_BASE + 0x300, 4 bits per pin, 8 pins per register
// GPIO 64-67 (GBE LEDs) are in register at offset 0x300 + (64/8)*0x10 = 0x380
const PINCTRL_MODE_OFS: usize = 0x300;
const PINCTRL_LED_REG_OFS: usize = PINCTRL_MODE_OFS + 0x80;  // 0x380 for GPIO 64-71

// FE SRAM - MT7988 uses SEPARATE SRAM region (NOT at FE_BASE + 0x40000!)
// Reference: linux/arch/arm64/boot/dts/mediatek/mt7988a.dtsi
//   eth_sram: sram@15400000 { reg = <0 0x15400000 0 0x200000>; }
// The MTK_ETH_SRAM_OFFSET (0x40000) is for older NETSYS V2 chips, not MT7988!
const SRAM_BASE: u64 = 0x1540_0000;     // MT7988 SRAM at 0x15400000
const SRAM_SIZE: u64 = 0x20_0000;       // 2MB

// Legacy offset (for reference only - NOT used on MT7988)
const FE_SRAM_OFFSET: u64 = 0x4_0000;   // MTK_ETH_SRAM_OFFSET - OLD CHIPS ONLY!

// MT7531 Internal Switch (memory-mapped in MT7988)
// Reference: Linux drivers/net/dsa/mt7530.h
const GSW_BASE: u32 = 0x20000;

// Switch port registers (per-port, 0x100 stride)
const MT7530_PCR_P: fn(u32) -> u32 = |p| 0x2004 + p * 0x100;  // Port Control Register
const MT7530_PVC_P: fn(u32) -> u32 = |p| 0x2010 + p * 0x100;  // Port VLAN Control
const MT7530_PMCR_P: fn(u32) -> u32 = |p| 0x3000 + p * 0x100; // Port MAC Control Register

// PCR (Port Control Register) bits - Linux: PCR_MATRIX(x) = ((x) & 0xff) << 16
const PCR_MATRIX_SHIFT: u32 = 16;        // Port matrix at bits [23:16]
const PCR_MATRIX_MASK: u32 = 0xFF << 16; // Port matrix mask
const fn pcr_matrix(ports: u32) -> u32 { (ports & 0xFF) << 16 }
const PCR_PORT_VLAN_MASK: u32 = 0x3;     // Bits [1:0]
const MT7530_PORT_MATRIX_MODE: u32 = 0;  // Matrix-based forwarding
const MT7530_PORT_FALLBACK_MODE: u32 = 3; // Fallback mode for VLAN

// MFC (Forward Control) - flooding configuration
const MT753X_MFC: u32 = 0x10;
const MFC_BC_FFP_SHIFT: u32 = 24;        // Broadcast flood port bits [31:24]
const MFC_UNM_FFP_SHIFT: u32 = 16;       // Unknown multicast flood [23:16]
const MFC_UNU_FFP_SHIFT: u32 = 8;        // Unknown unicast flood [15:8]

// CFC (CPU Forward Control) - MT7531/MT7988 specific
const MT7531_CFC: u32 = 0x4;
const MT7531_CPU_PMAP_SHIFT: u32 = 8;    // CPU port map bits [15:8]

// PVC (Port VLAN Control) bits
const PORT_SPEC_TAG: u32 = 1 << 5;       // Enable Mediatek header mode

// PMCR (Port MAC Control Register) bits
const PMCR_FORCE_MODE: u32 = 1 << 15;    // Force mode enable
const PMCR_IFG_XMIT: u32 = 1 << 18;      // TX IFG
const PMCR_MAC_MODE: u32 = 1 << 16;      // MAC mode (1G)
const PMCR_FORCE_SPEED_1000: u32 = 2 << 2;  // Force 1Gbps
const PMCR_FORCE_DPX: u32 = 1 << 1;      // Force full duplex
const PMCR_FORCE_LNK: u32 = 1 << 0;      // Force link up
const PMCR_TX_EN: u32 = 1 << 14;         // TX enable
const PMCR_RX_EN: u32 = 1 << 13;         // RX enable
const PMCR_BACKOFF_EN: u32 = 1 << 9;     // Backoff enable
const PMCR_BACKPR_EN: u32 = 1 << 8;      // Backpressure enable
const PMCR_FORCE_RX_FC_EN: u32 = 1 << 5; // RX flow control (Linux: flow control rx/tx)
const PMCR_FORCE_TX_FC_EN: u32 = 1 << 4; // TX flow control

// System control
const MT7530_SYS_CTRL: u32 = 0x7000;
const SYS_CTRL_SW_RST: u32 = 1 << 1;
const SYS_CTRL_REG_RST: u32 = 1 << 0;

// CPU port (port 6 in MT7531/MT7988)
const MT7531_CPU_PORT: u32 = 6;
// User ports (ports 0-3, the 4x RJ45)
const MT7531_USER_PORTS: u32 = 0x0F;  // Ports 0-3

// =============================================================================
// MT7531 VLAN Table Registers (for config API)
// Reference: Linux drivers/net/dsa/mt7530.h
// =============================================================================

const MT7530_VTCR: u32 = 0x90;        // VLAN table control
const MT7530_VAWD1: u32 = 0x94;       // VLAN data word 1
const MT7530_VAWD2: u32 = 0x98;       // VLAN data word 2

// VTCR bits
const VTCR_BUSY: u32 = 1 << 31;
const VTCR_INVALID: u32 = 1 << 16;
const VTCR_FUNC_RD_VID: u32 = 0 << 12;  // Read VLAN entry
const VTCR_FUNC_WR_VID: u32 = 1 << 12;  // Write VLAN entry
const VTCR_FUNC_INV_VID: u32 = 2 << 12; // Invalidate VLAN entry

// VAWD1 bits
const VAWD1_IVL_MAC: u32 = 1 << 30;     // Independent VLAN learning
const VAWD1_VTAG_EN: u32 = 1 << 28;     // VLAN tag enable
const VAWD1_PORT_MEM_SHIFT: u32 = 16;   // Port membership bits [23:16]
const VAWD1_PORT_MEM_MASK: u32 = 0xFF << 16;
const VAWD1_VLAN_VALID: u32 = 1 << 0;   // Entry valid

// VAWD2: Egress tag control, 2 bits per port
// 0 = untag, 1 = tag (use PVID), 2 = tag (use VID), 3 = stack tag
const VAWD2_PORT_ETAG_SHIFT: u32 = 0;   // Bits [13:0] for ports 0-6

// =============================================================================
// MT7531 FDB (Forwarding Database) Registers
// =============================================================================

const MT7530_ATA1: u32 = 0x74;        // Address table MAC bytes [47:16]
const MT7530_ATA2: u32 = 0x78;        // Address table MAC bytes [15:0] + flags
const MT7530_ATWD: u32 = 0x7c;        // Address table write data
const MT7530_ATC: u32 = 0x80;         // Address table control
const MT7530_TSRA1: u32 = 0x84;       // Table search result 1
const MT7530_TSRA2: u32 = 0x88;       // Table search result 2
const MT7530_ATRD: u32 = 0x8c;        // Address table read data

// ATC bits
const ATC_BUSY: u32 = 1 << 15;
const ATC_SRCH_END: u32 = 1 << 14;
const ATC_SRCH_HIT: u32 = 1 << 13;
const ATC_SAT: u32 = 1 << 2;          // Static address table

// ATC commands
const ATC_CMD_READ: u32 = 0;
const ATC_CMD_WRITE: u32 = 1;
const ATC_CMD_CLEAN: u32 = 2;
const ATC_CMD_SEARCH_START: u32 = 4;
const ATC_CMD_SEARCH_NEXT: u32 = 5;

// ATWD bits
const ATWD_PORT_MASK_SHIFT: u32 = 4;
const ATWD_LIVE: u32 = 3 << 0;        // Entry type: 3 = static

// =============================================================================
// MT7531 STP State Registers (per-port)
// =============================================================================

const MT7530_SSP_P: fn(u32) -> u32 = |p| 0x2000 + p * 0x100;  // Port Spanning State
// SSP bits [1:0]: 0=disabled, 1=blocking, 2=learning, 3=forwarding

// =============================================================================
// MT7531 MIB Counter Registers (per-port)
// =============================================================================

const MT7530_MIB_PORT_BASE: fn(u32) -> u32 = |p| 0x4000 + p * 0x100;
// MIB offsets within port base:
const MIB_TX_DROP: u32 = 0x00;
const MIB_TX_CRC: u32 = 0x04;
const MIB_TX_UNI: u32 = 0x08;
const MIB_TX_MULTI: u32 = 0x0c;
const MIB_TX_BCAST: u32 = 0x10;
const MIB_TX_COL: u32 = 0x14;
const MIB_TX_BYTES_LO: u32 = 0x24;
const MIB_TX_BYTES_HI: u32 = 0x28;
const MIB_RX_DROP: u32 = 0x50;
const MIB_RX_FILTER: u32 = 0x54;
const MIB_RX_UNI: u32 = 0x58;
const MIB_RX_MULTI: u32 = 0x5c;
const MIB_RX_BCAST: u32 = 0x60;
const MIB_RX_ALIGN_ERR: u32 = 0x64;
const MIB_RX_CRC: u32 = 0x68;
const MIB_RX_BYTES_LO: u32 = 0x80;
const MIB_RX_BYTES_HI: u32 = 0x84;

// =============================================================================
// MT7531 Port Mirror Registers
// =============================================================================

const MT7530_MFC: u32 = 0x10;         // Mirror Forward Control (shared)
const MFC_MIRROR_EN: u32 = 1 << 3;
const MFC_MIRROR_PORT_SHIFT: u32 = 4;
const MFC_MIRROR_PORT_MASK: u32 = 0x7 << 4;

const MT7530_PCR_MIR: fn(u32) -> u32 = |p| 0x2004 + p * 0x100;  // Port Control (mirror bits)
const PCR_TX_MIR: u32 = 1 << 9;       // TX mirror enable
const PCR_RX_MIR: u32 = 1 << 8;       // RX mirror enable

// =============================================================================
// MT7531 Age Control
// =============================================================================

const MT7530_AAC: u32 = 0xa0;         // Address Aging Control
const AAC_AGE_CNT_SHIFT: u32 = 12;    // Aging count bits [19:12]
const AAC_AGE_UNIT_SHIFT: u32 = 0;    // Aging unit bits [11:0]

// Ring sizes - MUST match Linux mtk_eth_soc exactly (MT7988)
// Source: linux/drivers/net/ethernet/mediatek/mtk_eth_soc.c mt7988_data
const NUM_TX_DESC: usize = 2048;      // MTK_DMA_SIZE(2K) for tx.dma_size
const NUM_RX_DESC: usize = 2048;      // MTK_DMA_SIZE(2K) for rx.dma_size
const NUM_FQ_DESC: usize = 4096;      // MTK_DMA_SIZE(4K) for tx.fq_dma_size
const PKTSIZE_ALIGN: usize = 2048;    // MTK_QDMA_PAGE_SIZE

// DMA memory layout - following Linux mtk_eth_soc exactly
// Descriptor rings go to FE SRAM (at FE_BASE + FE_SRAM_OFFSET)
// Data buffers go to regular DMA pool
const TX_DESC_SIZE: usize = 32;        // sizeof(struct mtk_tx_dma_v2)
const RX_DESC_SIZE: usize = 32;        // sizeof(struct mtk_rx_dma_v2)
const TX_RING_SIZE: usize = NUM_TX_DESC * TX_DESC_SIZE;  // 2048 * 32 = 64KB
const RX_RING_SIZE: usize = NUM_RX_DESC * RX_DESC_SIZE;  // 2048 * 32 = 64KB
const FQ_RING_SIZE: usize = NUM_FQ_DESC * TX_DESC_SIZE;  // 4096 * 32 = 128KB
const TX_BUF_SIZE: usize = NUM_TX_DESC * PKTSIZE_ALIGN;  // 2048 * 2048 = 4MB
const RX_BUF_SIZE: usize = NUM_RX_DESC * PKTSIZE_ALIGN;  // 2048 * 2048 = 4MB

// SRAM layout for descriptor rings (256KB total)
const SRAM_TX_RING_OFF: usize = 0;
const SRAM_RX_RING_OFF: usize = TX_RING_SIZE;                    // 64KB
const SRAM_FQ_RING_OFF: usize = TX_RING_SIZE + RX_RING_SIZE;     // 128KB
const SRAM_TOTAL_SIZE: usize = TX_RING_SIZE + RX_RING_SIZE + FQ_RING_SIZE;  // 256KB

// DMA pool layout
// When USE_SRAM_FOR_RINGS=true: descriptors in SRAM, buffers in DMA pool
// When USE_SRAM_FOR_RINGS=false: everything in DMA pool (like U-Boot)
//
// Layout when not using SRAM:
//   0x00000000: TX ring (64KB)
//   0x00010000: RX ring (64KB)
//   0x00020000: TX buffers (4MB)
//   0x00420000: RX buffers (4MB)
//   0x00820000: QDMA FQ ring (128KB) - only when USE_QDMA=true
//   0x00840000: QDMA FQ buffers (8MB) - only when USE_QDMA=true
const DMA_RING_OFF: usize = 0;                                      // Rings at start
const DMA_TX_RING_OFF: usize = DMA_RING_OFF;                        // 0
const DMA_RX_RING_OFF: usize = DMA_RING_OFF + TX_RING_SIZE;         // 64KB
const DMA_RINGS_SIZE: usize = TX_RING_SIZE + RX_RING_SIZE;          // 128KB
const DMA_TX_BUF_OFF: usize = DMA_RINGS_SIZE;                       // 128KB
const DMA_RX_BUF_OFF: usize = DMA_RINGS_SIZE + TX_BUF_SIZE;         // 128KB + 4MB

// QDMA Free Queue layout (after TX+RX buffers) - Linux sizes
// Linux MT7988: fq_dma_size = MTK_DMA_SIZE(4K) = 4096 descriptors
const QDMA_FQ_NUM_DESCS: usize = NUM_FQ_DESC;                                 // 4096 (Linux)
const QDMA_FQ_RING_OFF: usize = DMA_RINGS_SIZE + TX_BUF_SIZE + RX_BUF_SIZE;  // After RX buffers
const QDMA_FQ_RING_SIZE: usize = QDMA_FQ_NUM_DESCS * TX_DESC_SIZE;           // 4096 * 32 = 128KB
const QDMA_FQ_BUF_OFF: usize = QDMA_FQ_RING_OFF + QDMA_FQ_RING_SIZE;         // After FQ ring
const QDMA_FQ_BUF_SIZE: usize = QDMA_FQ_NUM_DESCS * QDMA_PAGE_SIZE;          // 4096 * 2048 = 8MB
const QDMA_FQ_TOTAL_SIZE: usize = QDMA_FQ_RING_SIZE + QDMA_FQ_BUF_SIZE;      // ~8.125MB

// Total DMA pool size - include FQ space when QDMA is enabled
const DMA_POOL_BASE_SIZE: usize = DMA_RINGS_SIZE + TX_BUF_SIZE + RX_BUF_SIZE;  // ~8.125MB
const DMA_POOL_SIZE: usize = if USE_QDMA { DMA_POOL_BASE_SIZE + QDMA_FQ_TOTAL_SIZE } else { DMA_POOL_BASE_SIZE };

// DMA mode configuration
// Set to true to use streaming (cacheable) DMA for data buffers.
// Streaming DMA provides better CPU performance but requires explicit cache sync.
// Set to false to use coherent (non-cacheable) DMA for data buffers (default).
const USE_STREAMING_DMA: bool = false;  // Use coherent (non-cacheable) DMA for simplicity

// SRAM vs DMA memory for descriptor rings
// MT7988 SRAM is at 0x15400000 (separate from FE), requires clocks to be enabled.
// Set to true to test SRAM, false to use DMA memory (like U-Boot does).
const USE_SRAM_FOR_RINGS: bool = true;  // Test SRAM with clock enable

// Debug verbosity flag
// Set to true to enable detailed register dumps and diagnostics during init.
// Set to false for production (reduces log spam significantly).
const DEBUG_VERBOSE: bool = false;

// QDMA vs PDMA for TX
// Linux uses QDMA for TX with a free queue (FQ) for automatic buffer recycling.
// U-Boot uses PDMA for simplicity. QDMA provides better throughput for high-load scenarios.
// Set to true to use QDMA (Linux style), false to use PDMA (U-Boot style, current default).
const USE_QDMA: bool = true;

// Frame Engine feature flags - enable/disable dynamic configuration
// These control whether the driver exposes config keys for runtime tuning.
// Set to true to enable runtime configuration via `devc ethd set fe.*`
const USE_CHECKSUM_OFFLOAD: bool = true;    // RX checksum offload (IP/TCP/UDP)
const USE_INTERRUPT_COALESCING: bool = true; // RX/TX interrupt coalescing
const USE_FLOW_CONTROL: bool = true;         // Pause frame (802.3x flow control)

// =============================================================================
// Frame Engine Delay Interrupt Registers (MT7988 / NETSYS V3)
// Reference: Linux mtk_eth_soc.c mt7988_reg_map
// =============================================================================

// PDMA delay interrupt register (relative to PDMA_V3_BASE)
const PDMA_DELAY_IRQ: u32 = 0x20c;  // 0x6a0c - PDMA_V3_BASE(0x6800) = 0x20c

// QDMA delay interrupt register (absolute offset from FE_BASE)
// Already defined: QDMA_DELAY_IRQ = 0x460c

// Delay interrupt register bits (Linux mtk_eth_soc.h)
const DLY_RX_EN: u32 = 1 << 15;          // RX delay enable
const DLY_RX_PINT_SHIFT: u32 = 8;        // RX packet interrupt count [14:8]
const DLY_RX_PTIME_SHIFT: u32 = 0;       // RX packet time delay [7:0]
const DLY_TX_EN: u32 = 1 << 31;          // TX delay enable
const DLY_TX_PINT_SHIFT: u32 = 24;       // TX packet interrupt count [30:24]
const DLY_TX_PTIME_SHIFT: u32 = 16;      // TX packet time delay [23:16]
const DLY_PINT_MASK: u32 = 0x7f;         // Max 127 packets
const DLY_PTIME_MASK: u32 = 0xff;        // Max 255 units (20µs each = 5.1ms max)

// Default coalescing values (Linux dim_default_profile)
const DEFAULT_COALESCE_USEC: u32 = 50;   // 50µs default
const DEFAULT_COALESCE_PKTS: u32 = 8;    // 8 packets default

// =============================================================================
// QDMA Register Map (MT7988 / NETSYS V3)
// Reference: Linux mtk_eth_soc.c mt7988_reg_map
// =============================================================================

const QDMA_BASE: u32 = 0x4400;  // QDMA register base offset from FE_BASE

// QDMA TX scheduling
const QDMA_QTX_CFG: u32 = 0x4400;
const QDMA_QTX_SCH: u32 = 0x4404;

// QDMA RX (not used - we use PDMA for RX)
const QDMA_RX_PTR: u32 = 0x4500;
const QDMA_RX_CNT_CFG: u32 = 0x4504;
const QDMA_QCRX_PTR: u32 = 0x4508;

// QDMA global config
const QDMA_GLO_CFG: u32 = 0x4604;
const QDMA_RST_IDX: u32 = 0x4608;
const QDMA_DELAY_IRQ: u32 = 0x460c;
const QDMA_FC_TH: u32 = 0x4610;
const QDMA_INT_GRP: u32 = 0x4620;
const QDMA_HRED: u32 = 0x4644;

// QDMA TX ring pointers
const QDMA_CTX_PTR: u32 = 0x4700;  // TX CPU pointer (where CPU writes)
const QDMA_DTX_PTR: u32 = 0x4704;  // TX DMA pointer (where HW reads)
const QDMA_CRX_PTR: u32 = 0x4710;  // RX CPU pointer
const QDMA_DRX_PTR: u32 = 0x4714;  // RX DMA pointer

// QDMA Free Queue (FQ) - scratch memory for QDMA internal buffer management
const QDMA_FQ_HEAD: u32 = 0x4720;
const QDMA_FQ_TAIL: u32 = 0x4724;
const QDMA_FQ_COUNT: u32 = 0x4728;
const QDMA_FQ_BLEN: u32 = 0x472c;

// QDMA TX scheduling rate
const QDMA_TX_SCH_RATE: u32 = 0x4798;

// QDMA GLO_CFG bits (Linux mtk_eth_soc.h)
const QDMA_TX_DMA_EN: u32 = 1 << 0;
const QDMA_TX_DMA_BUSY: u32 = 1 << 1;
const QDMA_RX_DMA_EN: u32 = 1 << 2;
const QDMA_RX_DMA_BUSY: u32 = 1 << 3;
const QDMA_TX_BT_32DWORDS: u32 = 3 << 4;   // TX burst size 32 DWORDs
const QDMA_TX_WB_DDONE: u32 = 1 << 6;
const QDMA_NDP_CO_PRO: u32 = 1 << 10;      // No-drop co-processor
const QDMA_RX_BT_32DWORDS: u32 = 3 << 11;  // RX burst size 32 DWORDs

// QDMA flow control threshold bits
const QDMA_FC_THRES_DROP_MODE: u32 = 1 << 21;
const QDMA_FC_THRES_DROP_EN: u32 = 1 << 20;
const QDMA_FC_THRES_MIN: u32 = 0x4444;  // Linux default

// QDMA descriptor bits (V2 format)
// Different from PDMA! QDMA uses txd2 as NEXT POINTER (linked list), not for length/flags.
// txd1: buffer physical address (low 32 bits)
// txd2: next descriptor physical address (linked list pointer)
// txd3: TX_DMA_PLEN0 | TX_DMA_LS0 | TX_DMA_OWNER_CPU
// txd4: FPORT | TX_DMA_SWC_V2 | QID
// txd5-8: optional features (TSO, checksum, VLAN)

// txd3 bits for QDMA V2 (from Linux mtk_eth_soc.h)
const QDMA_TX_DMA_LS0: u32 = 1 << 30;        // Last segment (Linux: TX_DMA_LS0 = BIT(30))
const QDMA_TX_DMA_OWNER_CPU: u32 = 1 << 31;  // CPU owns descriptor (Linux: TX_DMA_OWNER_CPU = BIT(31))

// For MT7988 (NETSYS V3): dma_max_len = 0xFFFF, dma_len_offset = 8
// TX_DMA_PLEN0(x) = (x & dma_max_len) << dma_len_offset
const fn qdma_tx_plen0(len: u32) -> u32 { (len & 0xFFFF) << 8 }

// txd4 bits for QDMA V2
const QDMA_TX_DMA_FPORT_SHIFT: u32 = 8;  // TX_DMA_FPORT_SHIFT_V2
const QDMA_TX_DMA_SWC: u32 = 1 << 30;    // TX_DMA_SWC_V2 (switch control)
const fn qdma_tx_fport(port: u32) -> u32 { port << QDMA_TX_DMA_FPORT_SHIFT }

// PSE port numbers (from Linux mtk_eth_soc.h enum mtk_pse_port)
const PSE_GDM1_PORT: u32 = 1;
const PSE_GDM2_PORT: u32 = 2;
const PSE_GDM3_PORT: u32 = 15;

// FQ sizes (from Linux)
const QDMA_PAGE_SIZE: usize = 2048;  // MTK_QDMA_PAGE_SIZE
const QDMA_FQ_DMA_SIZE: usize = 4096;  // MTK_DMA_SIZE(4K) for MT7988 FQ
const QDMA_TX_RING_SIZE: usize = 2048;  // MTK_QDMA_RING_SIZE

// QDMA scheduling constants
const QDMA_RES_THRES: u32 = 4;  // From Linux
const QDMA_NUM_QUEUES: usize = 16;  // MTK_QDMA_NUM_QUEUES
const QDMA_QTX_OFFSET: usize = 0x10;  // MTK_QTX_OFFSET

// QDMA TX Queue Scheduler Configuration (from Linux mtk_eth_soc.h)
// These bits go in QDMA_QTX_SCH register (0x4404 + queue * 0x10)
const MTK_QTX_SCH_MIN_RATE_EN: u32 = 1 << 27;       // Enable minimum rate
const MTK_QTX_SCH_MIN_RATE_MAN_SHIFT: u32 = 20;     // Min rate mantissa bits [26:20]
const MTK_QTX_SCH_MIN_RATE_EXP_SHIFT: u32 = 16;     // Min rate exponent bits [19:16]
const MTK_QTX_SCH_LEAKY_BUCKET_SIZE: u32 = 0x3 << 28; // Leaky bucket size bits [29:28]
// Note: LEAKY_BUCKET_EN (bit 30) only for NETSYS V1

// QDMA TX Scheduler Rate Control (from Linux mtk_eth_soc.h)
// This goes in tx_sch_rate register (0x4798)
const MTK_QDMA_TX_SCH_MAX_WFQ: u32 = 1 << 15;

// Legacy offsets for compatibility (will be removed)
const TX_RING_OFF: usize = 0;
const RX_RING_OFF: usize = TX_RING_SIZE;
const TX_BUF_OFF: usize = TX_RING_SIZE + RX_RING_SIZE;
const RX_BUF_OFF: usize = TX_RING_SIZE + RX_RING_SIZE + TX_BUF_SIZE;

// Network opcodes: use io_op::NET_SEND from userlib::ring (already imported)

// =============================================================================
// V2 Descriptor Structures (32 bytes each)
// =============================================================================

#[repr(C)]
#[derive(Clone, Copy)]
struct TxDescV2 {
    txd1: u32,  // Buffer address low
    txd2: u32,  // DDONE, LS0, SDL0
    txd3: u32,  // Buffer address high (for 36-bit)
    txd4: u32,  // V1 FPORT
    txd5: u32,  // V2/V3 FPORT
    txd6: u32,
    txd7: u32,
    txd8: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct RxDescV2 {
    rxd1: u32,  // Buffer address low
    rxd2: u32,  // DDONE, LS0, PLEN0
    rxd3: u32,  // Buffer address high
    rxd4: u32,
    rxd5: u32,
    rxd6: u32,
    rxd7: u32,
    rxd8: u32,
}

// =============================================================================
// Driver State
// =============================================================================

struct EthDriver {
    fe: Option<MmioRegion>,
    sw: Option<MmioRegion>,       // Internal switch at 0x15020000
    ethwarp: Option<MmioRegion>,  // Reset controller at 0x15031000
    ethsys: Option<MmioRegion>,   // System control at 0x15000000 (SRAM enable)
    pinctrl: Option<MmioRegion>,  // Pinctrl at 0x1001f000 (LED pin mux)
    dma: Option<DmaPool>,         // DMA pool for data buffers (8MB)

    // SRAM for descriptor rings (within FE MMIO at FE_BASE + 0x40000)
    // Linux: eth->scratch_ring = eth->base + MTK_ETH_SRAM_OFFSET
    sram_vaddr: u64,   // Virtual address of SRAM region (FE vaddr + 0x40000)
    sram_paddr: u64,   // Physical address for DMA programming (0x15140000)

    // DMA pool for data buffers
    buf_vaddr: u64,    // Virtual address of buffer region
    buf_paddr: u64,    // Physical address of buffer region

    tx_idx: usize,
    rx_idx: usize,

    port_id: Option<PortId>,
    poll_timer: Option<Timer>,
    poll_tag: u32,
    rx_seq: u64,

    mac: [u8; 6],
    link_up: bool,

    // GMAC configuration
    gmac_id: u32,           // 0, 1, or 2
    use_bridge_mode: bool,  // true for internal switch (GMAC0)

    // Debug
    poll_count: u32,

    // QDMA state (when USE_QDMA is true)
    // FQ = Free Queue (scratch memory for QDMA internal buffer management)
    // TX ring is separate from FQ
    qdma_fq_paddr: u64,      // FQ ring physical address
    qdma_fq_count: usize,    // Number of FQ descriptors
    qdma_tx_paddr: u64,      // TX ring physical address
    qdma_tx_next: u32,       // Next TX descriptor to fill (physical addr)

    // Frame Engine runtime configuration state
    // Checksum offload (bits in GDMA_IG_CTRL_REG)
    csum_rx_en: bool,        // RX checksum offload enabled (IP+TCP+UDP)

    // Interrupt coalescing (delay interrupt register)
    coalesce_rx_usec: u32,   // RX coalesce time in microseconds
    coalesce_rx_pkts: u32,   // RX coalesce packet count
    coalesce_tx_usec: u32,   // TX coalesce time in microseconds
    coalesce_tx_pkts: u32,   // TX coalesce packet count

    // Flow control (pause frames)
    pause_rx_en: bool,       // RX pause frame handling
    pause_tx_en: bool,       // TX pause frame generation
}

impl EthDriver {
    // =========================================================================
    // Register Access (following U-Boot's mtk_pdma_write/read pattern)
    // =========================================================================

    fn pdma_write(&self, reg: u32, val: u32) {
        if let Some(fe) = &self.fe {
            fe.write32(PDMA_V3_BASE as usize + reg as usize, val);
        }
    }

    fn pdma_read(&self, reg: u32) -> u32 {
        self.fe.as_ref().map(|fe| fe.read32(PDMA_V3_BASE as usize + reg as usize)).unwrap_or(0)
    }

    fn pdma_rmw(&self, reg: u32, clr: u32, set: u32) {
        let val = self.pdma_read(reg);
        self.pdma_write(reg, (val & !clr) | set);
    }

    // =========================================================================
    // QDMA Register Access (Linux mtk_eth_soc pattern)
    // =========================================================================

    fn qdma_write(&self, reg: u32, val: u32) {
        if let Some(fe) = &self.fe {
            fe.write32(reg as usize, val);
        }
    }

    fn qdma_read(&self, reg: u32) -> u32 {
        self.fe.as_ref().map(|fe| fe.read32(reg as usize)).unwrap_or(0)
    }

    fn qdma_rmw(&self, reg: u32, clr: u32, set: u32) {
        let val = self.qdma_read(reg);
        self.qdma_write(reg, (val & !clr) | set);
    }

    fn gdma_write(&self, gmac_id: u32, reg: u32, val: u32) {
        if let Some(fe) = &self.fe {
            let base = match gmac_id {
                0 => GDMA1_BASE,
                1 => GDMA2_BASE,
                _ => GDMA3_BASE,
            };
            fe.write32(base as usize + reg as usize, val);
        }
    }

    fn fe_write(&self, reg: u32, val: u32) {
        if let Some(fe) = &self.fe {
            fe.write32(reg as usize, val);
        }
    }

    fn fe_read(&self, reg: u32) -> u32 {
        self.fe.as_ref().map(|fe| fe.read32(reg as usize)).unwrap_or(0)
    }

    fn gmac_write(&self, reg: u32, val: u32) {
        if let Some(fe) = &self.fe {
            fe.write32(GMAC_BASE as usize + reg as usize, val);
        }
    }

    // =========================================================================
    // MT7988 Internal Switch Register Access (at 0x15020000)
    // =========================================================================

    fn gsw_write(&self, reg: u32, val: u32) {
        if let Some(sw) = &self.sw {
            sw.write32(reg as usize, val);
        }
    }

    fn gsw_read(&self, reg: u32) -> u32 {
        self.sw.as_ref().map(|sw| sw.read32(reg as usize)).unwrap_or(0)
    }

    fn gsw_rmw(&self, reg: u32, clr: u32, set: u32) {
        let val = self.gsw_read(reg);
        self.gsw_write(reg, (val & !clr) | set);
    }

    // =========================================================================
    // Ethwarp Reset Controller
    // =========================================================================

    fn reset_switch(&self) {
        let ethwarp = match &self.ethwarp {
            Some(e) => e,
            None => {
                uinfo!("ethd", "reset_switch_skip"; reason = "no_ethwarp");
                return;
            }
        };

        uinfo!("ethd", "reset_switch_start";);

        // Read current reset state
        let rst = ethwarp.read32(ETHWARP_RST_OFS);
        uinfo!("ethd", "ethwarp_rst_before"; val = userlib::ulog::hex32(rst));

        // Assert reset (set bit 9)
        ethwarp.write32(ETHWARP_RST_OFS, rst | ETHWARP_RST_SWITCH_BIT);
        delay_us(1000);  // 1ms

        // Deassert reset (clear bit 9)
        ethwarp.write32(ETHWARP_RST_OFS, rst & !ETHWARP_RST_SWITCH_BIT);

        // Wait for switch to stabilize (U-Boot waits 1000ms)
        delay_us(100_000);  // 100ms should be enough

        let rst_after = ethwarp.read32(ETHWARP_RST_OFS);
        uinfo!("ethd", "reset_switch_done"; val = userlib::ulog::hex32(rst_after));
    }

    // =========================================================================
    // MDIO PHY Access (via GMAC registers)
    // =========================================================================

    fn mdio_read(&self, phy: u8, reg: u8) -> Option<u16> {
        // Use switch's internal PHY_IAC register at 0x701C (not GMAC's MDIO!)
        // Reference: Linux mt7530.c - MT7531_PHY_IAC
        let sw = self.sw.as_ref()?;

        const PHY_IAC: usize = 0x701C;
        const PHY_ACS_ST: u32 = 1 << 31;
        const MDIO_CL22_READ: u32 = 2 << 18;  // C22 read command
        const MDIO_ST_C22: u32 = 1 << 16;

        let cmd = PHY_ACS_ST
            | MDIO_ST_C22
            | MDIO_CL22_READ
            | ((phy as u32) << 20)   // PHY_ADDR
            | ((reg as u32) << 25);  // REG_ADDR

        sw.write32(PHY_IAC, cmd);

        // Wait for PHY_ACS_ST to clear (use fewer iterations with longer delays to avoid syscall storm)
        for _ in 0..50 {
            let val = sw.read32(PHY_IAC);
            if (val & PHY_ACS_ST) == 0 {
                return Some((val & 0xFFFF) as u16);
            }
            delay_us(100);
        }
        None  // Timeout
    }

    fn mdio_write(&self, phy: u8, reg: u8, data: u16) -> bool {
        let sw = match self.sw.as_ref() {
            Some(s) => s,
            None => return false,
        };

        const PHY_IAC: usize = 0x701C;
        const PHY_ACS_ST: u32 = 1 << 31;
        const MDIO_CL22_WRITE: u32 = 1 << 18;  // C22 write command
        const MDIO_ST_C22: u32 = 1 << 16;

        let cmd = PHY_ACS_ST
            | MDIO_ST_C22
            | MDIO_CL22_WRITE
            | ((phy as u32) << 20)   // PHY_ADDR
            | ((reg as u32) << 25)   // REG_ADDR
            | (data as u32);         // DATA

        sw.write32(PHY_IAC, cmd);

        // Wait for PHY_ACS_ST to clear (use fewer iterations with longer delays to avoid syscall storm)
        for _ in 0..50 {
            let val = sw.read32(PHY_IAC);
            if (val & PHY_ACS_ST) == 0 {
                return true;
            }
            delay_us(100);
        }
        false  // Timeout
    }

    // MMD (MDIO Managed Device) register access for extended PHY registers
    // Uses Clause 22 indirect access: reg 13 = control, reg 14 = data
    fn mmd_read(&self, phy: u8, devad: u8, reg: u16) -> Option<u16> {
        const MMD_CTRL: u8 = 13;
        const MMD_DATA: u8 = 14;

        // Step 1: Set MMD device address
        if !self.mdio_write(phy, MMD_CTRL, devad as u16) { return None; }
        // Step 2: Set MMD register address
        if !self.mdio_write(phy, MMD_DATA, reg) { return None; }
        // Step 3: Set access mode to data (no post-increment)
        if !self.mdio_write(phy, MMD_CTRL, 0x4000 | (devad as u16)) { return None; }
        // Step 4: Read data
        self.mdio_read(phy, MMD_DATA)
    }

    fn mmd_write(&self, phy: u8, devad: u8, reg: u16, data: u16) -> bool {
        const MMD_CTRL: u8 = 13;
        const MMD_DATA: u8 = 14;

        // Step 1: Set MMD device address
        if !self.mdio_write(phy, MMD_CTRL, devad as u16) { return false; }
        // Step 2: Set MMD register address
        if !self.mdio_write(phy, MMD_DATA, reg) { return false; }
        // Step 3: Set access mode to data (no post-increment)
        if !self.mdio_write(phy, MMD_CTRL, 0x4000 | (devad as u16)) { return false; }
        // Step 4: Write data
        self.mdio_write(phy, MMD_DATA, data)
    }

    // Configure pinctrl to mux GPIO 64-67 to LED function
    // These are the GBE PHY LED pins on MT7988:
    //   GPIO 64 = GBE0_LED0 (port 0 LED)
    //   GPIO 65 = GBE1_LED0 (port 1 LED)
    //   GPIO 66 = GBE2_LED0 (port 2 LED)
    //   GPIO 67 = GBE3_LED0 (port 3 LED)
    // Each pin uses 4 bits in the mode register, function 1 = LED mode
    fn pinctrl_led_mux(&self) {
        let pc = match &self.pinctrl {
            Some(pc) => pc,
            None => {
                uwarn!("ethd", "pinctrl_led_mux"; status = "no_pinctrl");
                return;
            }
        };

        // GPIO 64-67 are in register at offset 0x380 (0x300 + 64/8 * 0x10)
        // Each pin is 4 bits: GPIO64 = [3:0], GPIO65 = [7:4], GPIO66 = [11:8], GPIO67 = [15:12]
        // Function 1 = LED mode (from pinctrl-mt7988.c)
        let old_val = pc.read32(PINCTRL_LED_REG_OFS);

        // Set function 1 for GPIO 64-67 (bits 15:0)
        // Keep upper bits (GPIO 68-71) unchanged
        let led_func: u32 = 0x1111;  // Function 1 for each of 4 pins
        let new_val = (old_val & 0xFFFF_0000) | led_func;

        pc.write32(PINCTRL_LED_REG_OFS, new_val);
        let verify = pc.read32(PINCTRL_LED_REG_OFS);

        uinfo!("ethd", "pinctrl_led_mux"; reg_ofs = userlib::ulog::hex32(PINCTRL_LED_REG_OFS as u32),
               old = userlib::ulog::hex32(old_val), new = userlib::ulog::hex32(new_val),
               verify = userlib::ulog::hex32(verify));
    }

    // Configure PHY LEDs for link/activity indication
    fn phy_led_init(&self) {
        // First, mux the LED pins via pinctrl
        self.pinctrl_led_mux();

        // MediaTek internal PHY LED registers (in MMD VEND2 = 31)
        const MDIO_MMD_VEND2: u8 = 31;

        // LED ON control registers
        const LED0_ON_CTRL: u16 = 0x24;
        const LED1_ON_CTRL: u16 = 0x26;

        // LED BLINK control registers
        const LED0_BLINK_CTRL: u16 = 0x25;
        const LED1_BLINK_CTRL: u16 = 0x27;

        // ON control bits
        const LED_ON_LINK1000: u16 = 1 << 0;
        const LED_ON_LINK100: u16 = 1 << 1;
        const LED_ON_LINK10: u16 = 1 << 2;
        const LED_ON_POLARITY: u16 = 1 << 14;  // Active-low if set
        const LED_ON_ENABLE: u16 = 1 << 15;

        // BLINK control bits (activity)
        const LED_BLINK_1000TX: u16 = 1 << 0;
        const LED_BLINK_1000RX: u16 = 1 << 1;
        const LED_BLINK_100TX: u16 = 1 << 2;
        const LED_BLINK_100RX: u16 = 1 << 3;
        const LED_BLINK_10TX: u16 = 1 << 4;
        const LED_BLINK_10RX: u16 = 1 << 5;

        // LED0: Link indicator (ON when link is up at any speed)
        // Try with polarity bit - MT7988 may need active-low depending on bootstrap
        let led0_on = LED_ON_ENABLE | LED_ON_POLARITY | LED_ON_LINK1000 | LED_ON_LINK100 | LED_ON_LINK10;
        let led0_blink: u16 = 0;  // No blinking for link LED

        // LED1: Activity indicator (blink on TX/RX)
        let led1_on = LED_ON_ENABLE | LED_ON_POLARITY;
        let led1_blink = LED_BLINK_1000TX | LED_BLINK_1000RX
                       | LED_BLINK_100TX | LED_BLINK_100RX
                       | LED_BLINK_10TX | LED_BLINK_10RX;

        for phy in 0..4u8 {
            // Configure LED0 (link)
            let old_led0 = self.mmd_read(phy, MDIO_MMD_VEND2, LED0_ON_CTRL).unwrap_or(0);
            self.mmd_write(phy, MDIO_MMD_VEND2, LED0_ON_CTRL, led0_on);
            self.mmd_write(phy, MDIO_MMD_VEND2, LED0_BLINK_CTRL, led0_blink);

            // Configure LED1 (activity)
            self.mmd_write(phy, MDIO_MMD_VEND2, LED1_ON_CTRL, led1_on);
            self.mmd_write(phy, MDIO_MMD_VEND2, LED1_BLINK_CTRL, led1_blink);

            let new_led0 = self.mmd_read(phy, MDIO_MMD_VEND2, LED0_ON_CTRL).unwrap_or(0);
            uinfo!("ethd", "phy_led_init"; phy = phy, old = userlib::ulog::hex32(old_led0 as u32),
                   new = userlib::ulog::hex32(new_led0 as u32));
        }

        uinfo!("ethd", "phy_leds_enabled";);
    }

    // =========================================================================
    // MT7531 Internal Switch Initialization
    // =========================================================================

    fn switch_init(&mut self) {
        uinfo!("ethd", "switch_init_start";);

        // Check if switch MMIO was mapped
        if self.sw.is_none() {
            uinfo!("ethd", "switch_init_skipped"; reason = "no_mmio");
            return;
        }

        // Probe switch registers - MT7988's internal switch has different layout
        // sys_ctrl at 0x7000 returns real data, 0x7ffc/0x7800 return deadbeef
        let sys_ctrl = self.gsw_read(0x7000);
        let pll_7500 = self.gsw_read(0x7500);  // MT7531_PLLGP_EN
        let pll_7504 = self.gsw_read(0x7504);  // PLL config
        let clk_7508 = self.gsw_read(0x7508);  // Clock config
        let misc_7804 = self.gsw_read(0x7804); // Trap register
        uinfo!("ethd", "switch_regs"; sys = userlib::ulog::hex32(sys_ctrl), pll0 = userlib::ulog::hex32(pll_7500),
               pll1 = userlib::ulog::hex32(pll_7504), clk = userlib::ulog::hex32(clk_7508), misc = userlib::ulog::hex32(misc_7804));

        // PRESERVE U-Boot's configuration - just read and log, don't write!
        // U-Boot should have already configured MDC and MUX_TO_ESW
        let mac_misc = self.fe.as_ref().map(|fe| fe.read32(GMAC_BASE as usize + 0x10)).unwrap_or(0);
        let ppsc = self.fe.as_ref().map(|fe| fe.read32(GMAC_BASE as usize + 0x00)).unwrap_or(0);
        uinfo!("ethd", "mac_misc_v3_uboot"; val = userlib::ulog::hex32(mac_misc),
               mux_to_esw = if mac_misc & 1 != 0 { 1u32 } else { 0u32 },
               mdc_turbo = if mac_misc & 0x10 != 0 { 1u32 } else { 0u32 });
        uinfo!("ethd", "ppsc_uboot"; val = userlib::ulog::hex32(ppsc));

        // Probe and initialize internal PHYs via MDIO (addresses 0-3 for ports 0-3)
        // PHY register definitions (IEEE 802.3):
        const BMCR: u8 = 0;       // Basic Mode Control Register
        const BMSR: u8 = 1;       // Basic Mode Status Register
        const PHYID1: u8 = 2;     // PHY ID 1
        const PHYID2: u8 = 3;     // PHY ID 2
        const ANAR: u8 = 4;       // Auto-Negotiation Advertisement Register

        // BMCR bits
        const BMCR_RESET: u16 = 1 << 15;
        const BMCR_AUTONEG_EN: u16 = 1 << 12;
        const BMCR_RESTART_AN: u16 = 1 << 9;
        const BMCR_POWER_DOWN: u16 = 1 << 11;

        // ANAR bits (advertise all capabilities)
        const ANAR_10_HALF: u16 = 1 << 5;
        const ANAR_10_FULL: u16 = 1 << 6;
        const ANAR_100_HALF: u16 = 1 << 7;
        const ANAR_100_FULL: u16 = 1 << 8;
        const ANAR_SELECTOR: u16 = 0x0001;  // 802.3

        for phy in 0..4u8 {
            // Read PHY ID registers (reg 2 and 3)
            let id1 = self.mdio_read(phy, PHYID1).unwrap_or(0xFFFF);
            let id2 = self.mdio_read(phy, PHYID2).unwrap_or(0xFFFF);
            // Read status register (reg 1) - bit 2 = link status
            let status = self.mdio_read(phy, BMSR).unwrap_or(0);
            let link = (status & 0x04) != 0;
            uinfo!("ethd", "phy"; addr = phy, id1 = userlib::ulog::hex32(id1 as u32), id2 = userlib::ulog::hex32(id2 as u32),
                   status = userlib::ulog::hex32(status as u32), link = if link { 1u32 } else { 0u32 });

            // Read current BMCR
            let bmcr = self.mdio_read(phy, BMCR).unwrap_or(0);
            uinfo!("ethd", "phy_bmcr"; addr = phy, bmcr = userlib::ulog::hex32(bmcr as u32),
                   power_down = if (bmcr & BMCR_POWER_DOWN) != 0 { 1u32 } else { 0u32 },
                   autoneg = if (bmcr & BMCR_AUTONEG_EN) != 0 { 1u32 } else { 0u32 });

            // If PHY is powered down, bring it up
            if (bmcr & BMCR_POWER_DOWN) != 0 {
                uinfo!("ethd", "phy_power_up"; addr = phy);
                self.mdio_write(phy, BMCR, bmcr & !BMCR_POWER_DOWN);
                delay_us(10_000);  // 10ms for PHY to power up
            }

            // Advertise all capabilities (10/100 half/full)
            let anar = ANAR_10_HALF | ANAR_10_FULL | ANAR_100_HALF | ANAR_100_FULL | ANAR_SELECTOR;
            self.mdio_write(phy, ANAR, anar);

            // For GbE, also set 1000BASE-T advertisement (reg 9)
            const GBCR: u8 = 9;  // 1000BASE-T Control Register
            const GBCR_1000_FULL: u16 = 1 << 9;
            const GBCR_1000_HALF: u16 = 1 << 8;
            self.mdio_write(phy, GBCR, GBCR_1000_FULL | GBCR_1000_HALF);

            // Enable auto-negotiation and restart it
            let new_bmcr = BMCR_AUTONEG_EN | BMCR_RESTART_AN;
            self.mdio_write(phy, BMCR, new_bmcr);
            uinfo!("ethd", "phy_autoneg_restart"; addr = phy);
        }

        // Wait for auto-negotiation to complete (up to 3 seconds)
        delay_us(100_000);  // 100ms initial wait

        // Re-check link status
        for phy in 0..4u8 {
            let status = self.mdio_read(phy, BMSR).unwrap_or(0);
            let link = (status & 0x04) != 0;
            let an_complete = (status & 0x20) != 0;
            uinfo!("ethd", "phy_status"; addr = phy, link = if link { 1u32 } else { 0u32 },
                   an_done = if an_complete { 1u32 } else { 0u32 });
        }

        // DON'T configure switch - trust U-Boot's setup
        // Just log current state for debugging
        let cpu_pcr = self.gsw_read(MT7530_PCR_P(MT7531_CPU_PORT));
        let cpu_pmcr = self.gsw_read(MT7530_PMCR_P(MT7531_CPU_PORT));
        uinfo!("ethd", "cpu_port_uboot"; pcr = userlib::ulog::hex32(cpu_pcr), pmcr = userlib::ulog::hex32(cpu_pmcr));

        for port in 0..4 {
            let pcr = self.gsw_read(MT7530_PCR_P(port));
            let pmcr = self.gsw_read(MT7530_PMCR_P(port));
            uinfo!("ethd", "user_port_uboot"; port = port, pcr = userlib::ulog::hex32(pcr), pmcr = userlib::ulog::hex32(pmcr));
        }

        // Configure switch CPU port (port 6) following Linux mt7530 driver
        // Reference: linux/drivers/net/dsa/mt7530.c - mt753x_cpu_port_enable()
        uinfo!("ethd", "cpu_port_config_start";);

        // Step 1: Configure MFC - enable flooding of BC/unknown UC/MC to CPU port
        // Linux: mt7530_set(priv, MT753X_MFC, BC_FFP(BIT(port)) | UNM_FFP(BIT(port)) | UNU_FFP(BIT(port)))
        let cpu_bit = 1u32 << MT7531_CPU_PORT;  // BIT(6) = 0x40
        let mfc_val = (cpu_bit << MFC_BC_FFP_SHIFT)    // Broadcast flood to CPU
                    | (cpu_bit << MFC_UNM_FFP_SHIFT)   // Unknown multicast to CPU
                    | (cpu_bit << MFC_UNU_FFP_SHIFT);  // Unknown unicast to CPU
        let mfc_before = self.gsw_read(MT753X_MFC);
        self.gsw_write(MT753X_MFC, mfc_before | mfc_val);
        let mfc_after = self.gsw_read(MT753X_MFC);
        uinfo!("ethd", "mfc_configured"; before = userlib::ulog::hex32(mfc_before),
               after = userlib::ulog::hex32(mfc_after));

        // Step 2: DON'T modify CFC - U-Boot leaves it at 0x00000000
        // U-Boot: pfc/cfc=0x00000000
        // We were setting CPU_PMAP which changes forwarding behavior
        let cfc_before = self.gsw_read(MT7531_CFC);
        // Don't modify - keep U-Boot's setting
        let cfc_after = cfc_before;
        uinfo!("ethd", "cfc_configured"; before = userlib::ulog::hex32(cfc_before),
               after = userlib::ulog::hex32(cfc_after));

        // Step 3: DON'T modify PVC - U-Boot leaves it at 0x81000000
        // U-Boot: pvc=0x81000000
        // Ours was setting PORT_SPEC_TAG (0x20) which changes the behavior
        let pvc_before = self.gsw_read(MT7530_PVC_P(MT7531_CPU_PORT));
        // Don't modify - keep U-Boot's setting
        let pvc_after = pvc_before;
        uinfo!("ethd", "pvc_configured"; before = userlib::ulog::hex32(pvc_before),
               after = userlib::ulog::hex32(pvc_after));

        // Step 4: Configure PCR - connect CPU port to ports 0-5 (matching U-Boot)
        // U-Boot: pcr=0x003f0000 → PORT_MATRIX = 0x3f (ports 0-5), no VLAN mode
        // We were using ports 0-3 only, but U-Boot includes port 5 (internal 2.5G PHY)
        let port_matrix = 0x3Fu32;  // Ports 0-5 (matching U-Boot)
        let pcr_val = pcr_matrix(port_matrix);  // No FALLBACK_MODE
        self.gsw_write(MT7530_PCR_P(MT7531_CPU_PORT), pcr_val);
        let pcr_after = self.gsw_read(MT7530_PCR_P(MT7531_CPU_PORT));
        uinfo!("ethd", "cpu_pcr_configured"; val = userlib::ulog::hex32(pcr_val),
               readback = userlib::ulog::hex32(pcr_after));

        // Step 5: Configure CPU port PMCR for internal 10G link
        // U-Boot leaves PMCR at 0x80000000 (only MT7531_FORCE_MODE_LNK).
        // However, we need TX_EN and RX_EN to actually forward packets.
        // User ports have PMCR=0x00056330 which includes TX_EN (bit 14) and RX_EN (bit 13).
        //
        // For the internal 10G CPU port, we'll enable:
        // - MT7531_FORCE_MODE_LNK (bit 31) - force mode for internal link
        // - TX_EN (bit 14) - enable TX
        // - RX_EN (bit 13) - enable RX
        // - FORCE_LNK (bit 0) - force link up
        const MT7531_FORCE_MODE_LNK: u32 = 1 << 31;
        let pmcr_before = self.gsw_read(MT7530_PMCR_P(MT7531_CPU_PORT));
        let pmcr_new = MT7531_FORCE_MODE_LNK | PMCR_TX_EN | PMCR_RX_EN | PMCR_FORCE_LNK;
        self.gsw_write(MT7530_PMCR_P(MT7531_CPU_PORT), pmcr_new);
        let pmcr_after = self.gsw_read(MT7530_PMCR_P(MT7531_CPU_PORT));
        uinfo!("ethd", "cpu_pmcr_configured"; before = userlib::ulog::hex32(pmcr_before),
               after = userlib::ulog::hex32(pmcr_after),
               tx_en = if pmcr_after & PMCR_TX_EN != 0 { 1u32 } else { 0 },
               rx_en = if pmcr_after & PMCR_RX_EN != 0 { 1u32 } else { 0 });

        // Step 6: DON'T modify user ports - U-Boot already configured them
        // U-Boot: port 0-3 pcr=0x00400000 → PORT_MATRIX = 0x40 (just CPU port)
        // The U-Boot config works, so don't change it
        for port in 0..4u32 {
            let port_pcr = self.gsw_read(MT7530_PCR_P(port));
            // Don't modify - keep U-Boot's setting
            uinfo!("ethd", "user_pcr_configured"; port = port,
                   before = userlib::ulog::hex32(port_pcr),
                   after = userlib::ulog::hex32(port_pcr));
        }

        uinfo!("ethd", "cpu_port_config_done";);

        // Initialize PHY LEDs for link/activity indication
        self.phy_led_init();

        // Enable switch-level LED controller (MT7530/MT7531 registers)
        // 0x7d00 = LED_EN (1 = enable LEDs)
        // 0x7d04 = LED_IO_MODE (1 = PHY mode, 0 = GPIO mode)
        const LED_EN_REG: u32 = 0x7d00;
        const LED_IO_MODE_REG: u32 = 0x7d04;

        let led_en_before = self.gsw_read(LED_EN_REG);
        let led_io_before = self.gsw_read(LED_IO_MODE_REG);
        uinfo!("ethd", "switch_led_before"; led_en = userlib::ulog::hex32(led_en_before),
               led_io = userlib::ulog::hex32(led_io_before));

        // Enable LEDs and set to PHY mode (not GPIO)
        self.gsw_write(LED_EN_REG, 0xFFFFFFFF);      // Enable all LED outputs
        self.gsw_write(LED_IO_MODE_REG, 0xFFFFFFFF); // PHY mode for all

        let led_en_after = self.gsw_read(LED_EN_REG);
        let led_io_after = self.gsw_read(LED_IO_MODE_REG);
        uinfo!("ethd", "switch_led_after"; led_en = userlib::ulog::hex32(led_en_after),
               led_io = userlib::ulog::hex32(led_io_after));

        uinfo!("ethd", "switch_init_done";);
    }

    // =========================================================================
    // Linux mtk_eth_soc: mtk_dma_init / mtk_init_fq_dma
    // =========================================================================

    fn fifo_init(&mut self) {
        let fe = match &self.fe {
            Some(f) => f,
            None => return,
        };
        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };

        // Calculate ring and buffer addresses based on USE_SRAM_FOR_RINGS config
        let (tx_ring_vaddr, tx_ring_paddr, rx_ring_vaddr, rx_ring_paddr): (u64, u64, u64, u64);
        let use_sram = USE_SRAM_FOR_RINGS && self.sram_vaddr != 0;

        if use_sram {
            // Use SRAM at 0x15400000 (already mapped and verified in init())
            // DON'T overwrite sram_vaddr/sram_paddr - they were set correctly in init()!
            tx_ring_vaddr = self.sram_vaddr + SRAM_TX_RING_OFF as u64;
            rx_ring_vaddr = self.sram_vaddr + SRAM_RX_RING_OFF as u64;
            tx_ring_paddr = self.sram_paddr + SRAM_TX_RING_OFF as u64;
            rx_ring_paddr = self.sram_paddr + SRAM_RX_RING_OFF as u64;

            // Buffer addresses in DMA pool (rings in SRAM, buffers in DMA)
            self.buf_vaddr = dma.vaddr();
            self.buf_paddr = dma.paddr();

            uinfo!("ethd", "fifo_init_sram";
                   sram_vaddr = userlib::ulog::hex64(self.sram_vaddr),
                   sram_paddr = userlib::ulog::hex64(self.sram_paddr),
                   buf_paddr = userlib::ulog::hex64(self.buf_paddr));
        } else {
            // Use DMA memory for both rings and buffers (like U-Boot)
            if USE_SRAM_FOR_RINGS {
                uerror!("ethd", "sram_fallback"; reason = "sram_not_mapped");
            }

            // Rings at start of DMA pool, buffers after rings
            self.sram_vaddr = 0;  // Not using SRAM
            self.sram_paddr = 0;

            tx_ring_vaddr = dma.vaddr() + DMA_TX_RING_OFF as u64;
            rx_ring_vaddr = dma.vaddr() + DMA_RX_RING_OFF as u64;
            tx_ring_paddr = dma.paddr() + DMA_TX_RING_OFF as u64;
            rx_ring_paddr = dma.paddr() + DMA_RX_RING_OFF as u64;

            // Buffer addresses in DMA pool (after rings)
            self.buf_vaddr = dma.vaddr() + DMA_RINGS_SIZE as u64;
            self.buf_paddr = dma.paddr() + DMA_RINGS_SIZE as u64;

            uinfo!("ethd", "fifo_init_dma";
                   tx_ring_paddr = userlib::ulog::hex64(tx_ring_paddr),
                   rx_ring_paddr = userlib::ulog::hex64(rx_ring_paddr),
                   buf_paddr = userlib::ulog::hex64(self.buf_paddr));
        }

        uinfo!("ethd", "ring_sizes";
               tx_desc = NUM_TX_DESC as u32,
               rx_desc = NUM_RX_DESC as u32,
               tx_ring_kb = (TX_RING_SIZE / 1024) as u32,
               rx_ring_kb = (RX_RING_SIZE / 1024) as u32);

        // CRITICAL: Proper PDMA reconfiguration sequence
        // U-Boot may have left PDMA pointing to old ring. We must:
        // 1. Disable DMA
        // 2. Wait for DMA to become idle
        // 3. Clear/reset state
        // 4. Configure new rings
        // 5. Re-enable DMA (done later in eth_start)

        let glo_before = self.pdma_read(PDMA_GLO_CFG_REG);
        uinfo!("ethd", "pdma_stop_start"; glo = userlib::ulog::hex32(glo_before));

        // Step 1: Disable DMA
        self.pdma_rmw(PDMA_GLO_CFG_REG, TX_DMA_EN | RX_DMA_EN, 0);
        delay_us(100);

        // Step 2: Wait for DMA busy to clear (up to 5ms)
        for _ in 0..50 {
            let glo = self.pdma_read(PDMA_GLO_CFG_REG);
            if (glo & (TX_DMA_BUSY | RX_DMA_BUSY)) == 0 {
                break;
            }
            delay_us(100);
        }

        // Step 3: Clear upper bits and reset (following U-Boot mtk_eth_fifo_init)
        self.pdma_rmw(PDMA_GLO_CFG_REG, 0xffff0000, 0);
        delay_us(100);

        let glo_after = self.pdma_read(PDMA_GLO_CFG_REG);
        uinfo!("ethd", "pdma_stopped"; glo = userlib::ulog::hex32(glo_after));

        // Zero descriptor rings
        if use_sram {
            // Zero SRAM rings using volatile writes (MMIO memory)
            unsafe {
                for i in 0..(TX_RING_SIZE + RX_RING_SIZE) {
                    core::ptr::write_volatile((self.sram_vaddr + i as u64) as *mut u8, 0);
                }
            }
            // Zero DMA pool (buffers only)
            unsafe {
                core::ptr::write_bytes(dma.vaddr() as *mut u8, 0, TX_BUF_SIZE + RX_BUF_SIZE);
            }
        } else {
            // Zero entire DMA pool (rings + buffers)
            unsafe {
                core::ptr::write_bytes(dma.vaddr() as *mut u8, 0, DMA_POOL_SIZE);
            }
        }

        // If using streaming DMA, clean cache after zeroing
        if USE_STREAMING_DMA {
            cache_clean(dma.vaddr(), DMA_POOL_SIZE);
        }

        self.tx_idx = 0;
        self.rx_idx = 0;

        // Initialize TX descriptors
        for i in 0..NUM_TX_DESC {
            let desc_vaddr = tx_ring_vaddr + (i * TX_DESC_SIZE) as u64;
            let buf_paddr = self.buf_paddr + (i * PKTSIZE_ALIGN) as u64;  // TX buffers at buf_paddr

            unsafe {
                let txd = desc_vaddr as *mut TxDescV2;
                // Use volatile writes (needed for both SRAM and DMA memory)
                core::ptr::write_volatile(&mut (*txd).txd1, buf_paddr as u32);
                core::ptr::write_volatile(&mut (*txd).txd2, PDMA_TXD2_DDONE | PDMA_TXD2_LS0);
                core::ptr::write_volatile(&mut (*txd).txd3, (buf_paddr >> 32) as u32);
                core::ptr::write_volatile(&mut (*txd).txd4, 0);
                // V2/V3: FPORT in txd5 - GMAC1 = port 1
                core::ptr::write_volatile(&mut (*txd).txd5, pdma_v2_txd5_fport_set(self.gmac_id + 1));
                core::ptr::write_volatile(&mut (*txd).txd6, 0);
                core::ptr::write_volatile(&mut (*txd).txd7, 0);
                core::ptr::write_volatile(&mut (*txd).txd8, 0);
            }
        }

        // Initialize RX descriptors
        for i in 0..NUM_RX_DESC {
            let desc_vaddr = rx_ring_vaddr + (i * RX_DESC_SIZE) as u64;
            // RX buffers after TX buffers
            let buf_paddr = self.buf_paddr + (TX_BUF_SIZE + i * PKTSIZE_ALIGN) as u64;

            unsafe {
                let rxd = desc_vaddr as *mut RxDescV2;
                core::ptr::write_volatile(&mut (*rxd).rxd1, buf_paddr as u32);
                core::ptr::write_volatile(&mut (*rxd).rxd2, pdma_v2_rxd2_plen0_set(PKTSIZE_ALIGN as u32));
                core::ptr::write_volatile(&mut (*rxd).rxd3, (buf_paddr >> 32) as u32);
                core::ptr::write_volatile(&mut (*rxd).rxd4, 0);
                core::ptr::write_volatile(&mut (*rxd).rxd5, 0);
                core::ptr::write_volatile(&mut (*rxd).rxd6, 0);
                core::ptr::write_volatile(&mut (*rxd).rxd7, 0);
                core::ptr::write_volatile(&mut (*rxd).rxd8, 0);
            }
        }

        // For streaming DMA: clean cache after writing descriptors
        if USE_STREAMING_DMA && !USE_SRAM_FOR_RINGS {
            cache_clean(tx_ring_vaddr, TX_RING_SIZE);
            cache_clean(rx_ring_vaddr, RX_RING_SIZE);
        }

        if USE_QDMA {
            // QDMA TX + PDMA RX (Linux pattern)
            // Initialize QDMA Free Queue and TX ring
            self.qdma_init_fq();
            self.qdma_init_tx_ring();

            // Still set up PDMA RX (Linux uses PDMA for RX even with QDMA TX)
            self.pdma_write(rx_base_ptr_reg(0), rx_ring_paddr as u32);
            self.pdma_write(rx_max_cnt_reg(0), NUM_RX_DESC as u32);
            self.pdma_write(rx_crx_idx_reg(0), (NUM_RX_DESC - 1) as u32);

            // Reset RX DMA index only (TX is handled by QDMA)
            self.pdma_write(PDMA_RST_IDX_REG, RST_DRX_IDX0);

            uinfo!("ethd", "fifo_init_qdma"; mode = "qdma_tx_pdma_rx");
        } else {
            // PDMA for both TX and RX (U-Boot pattern)
            self.pdma_write(tx_base_ptr_reg(0), tx_ring_paddr as u32);
            self.pdma_write(tx_max_cnt_reg(0), NUM_TX_DESC as u32);
            self.pdma_write(tx_ctx_idx_reg(0), 0);

            self.pdma_write(rx_base_ptr_reg(0), rx_ring_paddr as u32);
            self.pdma_write(rx_max_cnt_reg(0), NUM_RX_DESC as u32);
            self.pdma_write(rx_crx_idx_reg(0), (NUM_RX_DESC - 1) as u32);

            // Reset DMA indices
            self.pdma_write(PDMA_RST_IDX_REG, RST_DTX_IDX0 | RST_DRX_IDX0);
        }

        // Verify configuration
        let verify_tx_base = self.pdma_read(tx_base_ptr_reg(0));
        let verify_tx_max = self.pdma_read(tx_max_cnt_reg(0));
        let verify_tx_ctx = self.pdma_read(tx_ctx_idx_reg(0));
        let verify_tx_dtx = self.pdma_read(tx_dtx_idx_reg(0));
        let verify_rx_base = self.pdma_read(rx_base_ptr_reg(0));
        let verify_rx_max = self.pdma_read(rx_max_cnt_reg(0));

        uinfo!("ethd", "fifo_init_done";
               tx_ring = userlib::ulog::hex64(tx_ring_paddr),
               rx_ring = userlib::ulog::hex64(rx_ring_paddr));

        if DEBUG_VERBOSE {
            uinfo!("ethd", "verify_tx";
                   base = userlib::ulog::hex32(verify_tx_base),
                   max = verify_tx_max,
                   ctx = verify_tx_ctx,
                   dtx = verify_tx_dtx);
            uinfo!("ethd", "verify_rx";
                   base = userlib::ulog::hex32(verify_rx_base),
                   max = verify_rx_max);

            // Verify ring writes by reading back first TX descriptor
            unsafe {
                let txd = tx_ring_vaddr as *const TxDescV2;
                // For streaming DMA, invalidate before reading
                if USE_STREAMING_DMA && !USE_SRAM_FOR_RINGS {
                    cache_invalidate(tx_ring_vaddr, TX_DESC_SIZE);
                }
                let txd1 = core::ptr::read_volatile(&(*txd).txd1);
                let txd2 = core::ptr::read_volatile(&(*txd).txd2);
                let txd3 = core::ptr::read_volatile(&(*txd).txd3);
                let txd5 = core::ptr::read_volatile(&(*txd).txd5);
                uinfo!("ethd", "verify_tx0";
                       txd1 = userlib::ulog::hex32(txd1),
                       txd2 = userlib::ulog::hex32(txd2),
                       txd3 = userlib::ulog::hex32(txd3),
                       txd5 = userlib::ulog::hex32(txd5));

                // Also verify RX descriptor 0
                if USE_STREAMING_DMA && !USE_SRAM_FOR_RINGS {
                    cache_invalidate(rx_ring_vaddr, RX_DESC_SIZE);
                }
                let rxd = rx_ring_vaddr as *const RxDescV2;
                let rxd1 = core::ptr::read_volatile(&(*rxd).rxd1);
                let rxd2 = core::ptr::read_volatile(&(*rxd).rxd2);
                uinfo!("ethd", "verify_rx0";
                       rxd1 = userlib::ulog::hex32(rxd1),
                       rxd2 = userlib::ulog::hex32(rxd2));
            }
        }
    }

    // =========================================================================
    // QDMA Initialization (Linux mtk_eth_soc: mtk_init_fq_dma + mtk_tx_alloc)
    // =========================================================================

    /// Stop QDMA TX engine and wait for idle.
    /// Must be called before reconfiguring QDMA.
    fn qdma_stop(&mut self) {
        let glo_before = self.qdma_read(QDMA_GLO_CFG);
        uinfo!("ethd", "qdma_stop_start"; glo = userlib::ulog::hex32(glo_before));

        // Disable TX DMA
        self.qdma_rmw(QDMA_GLO_CFG, QDMA_TX_DMA_EN | QDMA_RX_DMA_EN, 0);
        delay_us(100);

        // Wait for TX DMA busy to clear (up to 5ms)
        for _ in 0..50 {
            let glo = self.qdma_read(QDMA_GLO_CFG);
            if (glo & QDMA_TX_DMA_BUSY) == 0 {
                break;
            }
            delay_us(100);
        }

        // Clear upper bits (reset various states)
        self.qdma_rmw(QDMA_GLO_CFG, 0xffff0000, 0);
        delay_us(100);

        // Reset QDMA indices
        self.qdma_write(QDMA_RST_IDX, 0xffffffff);  // Reset all indices
        delay_us(100);

        let glo_after = self.qdma_read(QDMA_GLO_CFG);
        uinfo!("ethd", "qdma_stopped"; glo = userlib::ulog::hex32(glo_after));
    }

    /// Initialize QDMA Free Queue (FQ) - scratch memory for QDMA internal buffer management.
    /// This is called from fifo_init() when USE_QDMA is true.
    /// Reference: Linux mtk_init_fq_dma()
    fn qdma_init_fq(&mut self) {
        // First stop QDMA to clear any U-Boot state
        self.qdma_stop();

        // FQ is a linked list of descriptors, each pointing to a scratch buffer.
        // QDMA uses FQ for temporary storage during packet processing.

        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };

        // Use module-level constants for FQ layout
        let fq_ring_vaddr = dma.vaddr() + QDMA_FQ_RING_OFF as u64;
        let fq_ring_paddr = dma.paddr() + QDMA_FQ_RING_OFF as u64;
        let fq_buf_paddr = dma.paddr() + QDMA_FQ_BUF_OFF as u64;

        // Zero FQ descriptors
        unsafe {
            core::ptr::write_bytes(fq_ring_vaddr as *mut u8, 0, QDMA_FQ_RING_SIZE);
        }

        // Initialize FQ descriptors (linked list)
        for i in 0..QDMA_FQ_NUM_DESCS {
            let desc_vaddr = fq_ring_vaddr + (i * TX_DESC_SIZE) as u64;
            let buf_paddr = fq_buf_paddr + (i * QDMA_PAGE_SIZE) as u64;
            let next_desc_paddr = if i < QDMA_FQ_NUM_DESCS - 1 {
                fq_ring_paddr + ((i + 1) * TX_DESC_SIZE) as u64
            } else {
                0  // Last descriptor, no next
            };

            unsafe {
                let txd = desc_vaddr as *mut TxDescV2;
                // txd1 = buffer physical address
                core::ptr::write_volatile(&mut (*txd).txd1, buf_paddr as u32);
                // txd2 = next descriptor physical address (linked list)
                core::ptr::write_volatile(&mut (*txd).txd2, next_desc_paddr as u32);
                // txd3 = TX_DMA_PLEN0(QDMA_PAGE_SIZE) - buffer length
                core::ptr::write_volatile(&mut (*txd).txd3, qdma_tx_plen0(QDMA_PAGE_SIZE as u32));
                core::ptr::write_volatile(&mut (*txd).txd4, 0);
                core::ptr::write_volatile(&mut (*txd).txd5, 0);
                core::ptr::write_volatile(&mut (*txd).txd6, 0);
                core::ptr::write_volatile(&mut (*txd).txd7, 0);
                core::ptr::write_volatile(&mut (*txd).txd8, 0);
            }
        }

        // Cache clean if streaming DMA
        if USE_STREAMING_DMA {
            cache_clean(fq_ring_vaddr, QDMA_FQ_RING_SIZE);
        }

        // Configure FQ registers
        let fq_tail_paddr = fq_ring_paddr + ((QDMA_FQ_NUM_DESCS - 1) * TX_DESC_SIZE) as u64;
        self.qdma_write(QDMA_FQ_HEAD, fq_ring_paddr as u32);
        self.qdma_write(QDMA_FQ_TAIL, fq_tail_paddr as u32);
        // fq_count: high 16 bits = total count, low 16 bits = available count
        self.qdma_write(QDMA_FQ_COUNT, ((QDMA_FQ_NUM_DESCS as u32) << 16) | (QDMA_FQ_NUM_DESCS as u32));
        // fq_blen: buffer length in high 16 bits
        self.qdma_write(QDMA_FQ_BLEN, (QDMA_PAGE_SIZE as u32) << 16);

        self.qdma_fq_paddr = fq_ring_paddr;
        self.qdma_fq_count = QDMA_FQ_NUM_DESCS;

        uinfo!("ethd", "qdma_fq_init";
               fq_head = userlib::ulog::hex64(fq_ring_paddr),
               fq_tail = userlib::ulog::hex64(fq_tail_paddr),
               count = QDMA_FQ_NUM_DESCS as u32);
    }

    /// Initialize QDMA TX ring.
    /// Reference: Linux mtk_tx_alloc() for QDMA
    fn qdma_init_tx_ring(&mut self) {
        let dma = match &self.dma {
            Some(d) => d,
            None => return,
        };

        // TX ring layout - use same area as PDMA TX ring for simplicity
        // (We only use one or the other, never both)
        let use_sram = USE_SRAM_FOR_RINGS && self.sram_vaddr != 0;
        let (tx_ring_vaddr, tx_ring_paddr) = if use_sram {
            (self.sram_vaddr + SRAM_TX_RING_OFF as u64,
             self.sram_paddr + SRAM_TX_RING_OFF as u64)
        } else {
            (dma.vaddr() + DMA_TX_RING_OFF as u64,
             dma.paddr() + DMA_TX_RING_OFF as u64)
        };

        // Initialize TX descriptors (circular linked list)
        for i in 0..NUM_TX_DESC {
            let desc_vaddr = tx_ring_vaddr + (i * TX_DESC_SIZE) as u64;
            let next_idx = (i + 1) % NUM_TX_DESC;
            let next_desc_paddr = tx_ring_paddr + (next_idx * TX_DESC_SIZE) as u64;
            let buf_paddr = self.buf_paddr + (i * PKTSIZE_ALIGN) as u64;

            unsafe {
                let txd = desc_vaddr as *mut TxDescV2;
                // txd1 = buffer address (will be overwritten on each TX)
                core::ptr::write_volatile(&mut (*txd).txd1, buf_paddr as u32);
                // txd2 = next descriptor physical address (circular linked list)
                core::ptr::write_volatile(&mut (*txd).txd2, next_desc_paddr as u32);
                // txd3 = LS0 | OWNER_CPU (CPU owns descriptor, ready for use)
                core::ptr::write_volatile(&mut (*txd).txd3, QDMA_TX_DMA_LS0 | QDMA_TX_DMA_OWNER_CPU);
                core::ptr::write_volatile(&mut (*txd).txd4, 0);
                core::ptr::write_volatile(&mut (*txd).txd5, 0);
                core::ptr::write_volatile(&mut (*txd).txd6, 0);
                core::ptr::write_volatile(&mut (*txd).txd7, 0);
                core::ptr::write_volatile(&mut (*txd).txd8, 0);
            }
        }

        // Cache clean if streaming DMA and not using SRAM
        if USE_STREAMING_DMA && !use_sram {
            cache_clean(tx_ring_vaddr, TX_RING_SIZE);
        }

        // Configure QDMA TX ring registers
        // ctx_ptr = dtx_ptr = ring base (both start at same place)
        self.qdma_write(QDMA_CTX_PTR, tx_ring_paddr as u32);
        self.qdma_write(QDMA_DTX_PTR, tx_ring_paddr as u32);
        // crx_ptr = drx_ptr = last descriptor (for RX done indication, not used for TX-only)
        let last_desc_paddr = tx_ring_paddr + ((NUM_TX_DESC - 1) * TX_DESC_SIZE) as u64;
        self.qdma_write(QDMA_CRX_PTR, last_desc_paddr as u32);
        self.qdma_write(QDMA_DRX_PTR, last_desc_paddr as u32);

        // Configure TX scheduling (from Linux mtk_tx_alloc)
        // Each TX queue needs both QTX_CFG and QTX_SCH configured
        for i in 0..QDMA_NUM_QUEUES {
            let ofs = i * QDMA_QTX_OFFSET;

            // QTX_CFG: resource threshold (high byte = res_thres, low byte = res_thres)
            self.qdma_write(QDMA_QTX_CFG + ofs as u32, (QDMA_RES_THRES << 8) | QDMA_RES_THRES);

            // QTX_SCH: scheduling config
            // Linux sets:
            // - MTK_QTX_SCH_MIN_RATE_EN (bit 27)
            // - MTK_QTX_SCH_MIN_RATE_MAN = 1 (minimum: 10 Mbps)
            // - MTK_QTX_SCH_MIN_RATE_EXP = 4
            // - MTK_QTX_SCH_LEAKY_BUCKET_SIZE (bits 29:28)
            // Note: LEAKY_BUCKET_EN only for NETSYS V1, skip for MT7988 (V3)
            let qtx_sch = MTK_QTX_SCH_MIN_RATE_EN
                | (1 << MTK_QTX_SCH_MIN_RATE_MAN_SHIFT)   // mantissa = 1
                | (4 << MTK_QTX_SCH_MIN_RATE_EXP_SHIFT)   // exponent = 4 (10 Mbps min)
                | MTK_QTX_SCH_LEAKY_BUCKET_SIZE;
            self.qdma_write(QDMA_QTX_SCH + ofs as u32, qtx_sch);
        }

        // Configure global TX scheduler rate control (tx_sch_rate register)
        // Linux: val = MTK_QDMA_TX_SCH_MAX_WFQ | (MTK_QDMA_TX_SCH_MAX_WFQ << 16)
        // For NETSYS V2+ also write to tx_sch_rate + 4
        let tx_sch_val = MTK_QDMA_TX_SCH_MAX_WFQ | (MTK_QDMA_TX_SCH_MAX_WFQ << 16);
        self.qdma_write(QDMA_TX_SCH_RATE, tx_sch_val);
        self.qdma_write(QDMA_TX_SCH_RATE + 4, tx_sch_val);  // NETSYS V3 has two rate registers

        uinfo!("ethd", "qdma_tx_sch"; qtx_sch = userlib::ulog::hex32(
            MTK_QTX_SCH_MIN_RATE_EN | (1 << MTK_QTX_SCH_MIN_RATE_MAN_SHIFT)
            | (4 << MTK_QTX_SCH_MIN_RATE_EXP_SHIFT) | MTK_QTX_SCH_LEAKY_BUCKET_SIZE),
            tx_sch_rate = userlib::ulog::hex32(tx_sch_val));

        self.qdma_tx_paddr = tx_ring_paddr;
        self.qdma_tx_next = tx_ring_paddr as u32;

        uinfo!("ethd", "qdma_tx_init";
               tx_ring = userlib::ulog::hex64(tx_ring_paddr),
               ctx_ptr = userlib::ulog::hex32(tx_ring_paddr as u32),
               num_desc = NUM_TX_DESC as u32);
    }

    /// Enable QDMA TX DMA engine
    fn qdma_start(&mut self) {
        // Read current GLO_CFG
        let glo = self.qdma_read(QDMA_GLO_CFG);
        uinfo!("ethd", "qdma_start_before"; glo = userlib::ulog::hex32(glo));

        // Enable TX DMA with all the bits Linux uses:
        // - TX_DMA_EN: enable TX
        // - TX_WB_DDONE: write-back done descriptor
        // - TX_BT_32DWORDS: 32 DWORD burst
        // - NDP_CO_PRO: no-drop co-processor
        // Note: We don't enable RX DMA here - we still use PDMA for RX (like Linux does)
        let enable_bits = QDMA_TX_DMA_EN | QDMA_TX_WB_DDONE | QDMA_TX_BT_32DWORDS | QDMA_NDP_CO_PRO;
        self.qdma_rmw(QDMA_GLO_CFG, 0, enable_bits);

        let glo_after = self.qdma_read(QDMA_GLO_CFG);
        uinfo!("ethd", "qdma_start_after"; glo = userlib::ulog::hex32(glo_after));
    }

    /// Send a packet via QDMA TX.
    /// Returns true on success, false if TX ring is full.
    fn qdma_send(&mut self, packet: &[u8]) -> bool {
        if self.buf_vaddr == 0 {
            return false;
        }

        // Get current TX descriptor
        let use_sram = USE_SRAM_FOR_RINGS && self.sram_vaddr != 0;
        let tx_ring_vaddr = if use_sram {
            self.sram_vaddr + SRAM_TX_RING_OFF as u64
        } else {
            self.dma.as_ref().map(|d| d.vaddr() + DMA_TX_RING_OFF as u64).unwrap_or(0)
        };
        if tx_ring_vaddr == 0 {
            return false;
        }

        let desc_vaddr = tx_ring_vaddr + (self.tx_idx * TX_DESC_SIZE) as u64;

        unsafe {
            let txd = desc_vaddr as *mut TxDescV2;

            // Check if CPU owns this descriptor (OWNER_CPU bit set in txd3)
            let txd3 = core::ptr::read_volatile(&(*txd).txd3);
            if (txd3 & QDMA_TX_DMA_OWNER_CPU) == 0 {
                // DMA still owns this descriptor, ring is full
                uinfo!("ethd", "qdma_tx_full"; idx = self.tx_idx as u32);
                return false;
            }

            // Calculate buffer addresses
            let buf_offset = self.tx_idx * PKTSIZE_ALIGN;
            let buf_vaddr = self.buf_vaddr + buf_offset as u64;
            let buf_paddr = self.buf_paddr + buf_offset as u64;

            // Copy packet to buffer
            core::ptr::copy_nonoverlapping(packet.as_ptr(), buf_vaddr as *mut u8, packet.len());

            // Cache clean if streaming DMA
            if USE_STREAMING_DMA {
                cache_clean(buf_vaddr, packet.len());
            }

            // Memory barrier before descriptor update
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            // Fill descriptor
            // txd1 = buffer physical address
            core::ptr::write_volatile(&mut (*txd).txd1, buf_paddr as u32);
            // txd2 = next descriptor (already set during init, don't change)
            // txd3 = PLEN0 | LS0 (clear OWNER_CPU to give to DMA)
            core::ptr::write_volatile(&mut (*txd).txd3, qdma_tx_plen0(packet.len() as u32) | QDMA_TX_DMA_LS0);
            // txd4 = FPORT (GMAC1 = port 1) | SWC
            let fport = match self.gmac_id {
                0 => PSE_GDM1_PORT,
                1 => PSE_GDM2_PORT,
                2 => PSE_GDM3_PORT,
                _ => PSE_GDM1_PORT,
            };
            core::ptr::write_volatile(&mut (*txd).txd4, qdma_tx_fport(fport) | QDMA_TX_DMA_SWC);
            // txd5-8 = 0 (no offloads)
            core::ptr::write_volatile(&mut (*txd).txd5, 0);
            core::ptr::write_volatile(&mut (*txd).txd6, 0);
            core::ptr::write_volatile(&mut (*txd).txd7, 0);
            core::ptr::write_volatile(&mut (*txd).txd8, 0);

            // Memory barrier after descriptor update
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Calculate next descriptor physical address
        let next_idx = (self.tx_idx + 1) % NUM_TX_DESC;
        let tx_ring_paddr = self.qdma_tx_paddr;
        let next_desc_paddr = tx_ring_paddr + (next_idx * TX_DESC_SIZE) as u64;

        // Update CPU TX pointer to kick QDMA
        self.qdma_write(QDMA_CTX_PTR, next_desc_paddr as u32);

        // Advance TX index
        self.tx_idx = next_idx;
        self.qdma_tx_next = next_desc_paddr as u32;

        let dtx = self.qdma_read(QDMA_DTX_PTR);
        let ctx = self.qdma_read(QDMA_CTX_PTR);

        // Read back what we wrote to the descriptor for debugging
        let use_sram = USE_SRAM_FOR_RINGS && self.sram_vaddr != 0;
        let tx_ring_vaddr = if use_sram {
            self.sram_vaddr + SRAM_TX_RING_OFF as u64
        } else {
            self.dma.as_ref().map(|d| d.vaddr() + DMA_TX_RING_OFF as u64).unwrap_or(0)
        };
        let prev_idx = if self.tx_idx == 0 { NUM_TX_DESC - 1 } else { self.tx_idx - 1 };
        let desc_vaddr = tx_ring_vaddr + (prev_idx * TX_DESC_SIZE) as u64;
        let (txd1, txd2, txd3, txd4) = unsafe {
            let txd = desc_vaddr as *const TxDescV2;
            (
                core::ptr::read_volatile(&(*txd).txd1),
                core::ptr::read_volatile(&(*txd).txd2),
                core::ptr::read_volatile(&(*txd).txd3),
                core::ptr::read_volatile(&(*txd).txd4),
            )
        };

        uinfo!("ethd", "qdma_tx_sent"; idx = self.tx_idx as u32, len = packet.len() as u32,
               dtx = userlib::ulog::hex32(dtx), ctx = userlib::ulog::hex32(ctx),
               txd1 = userlib::ulog::hex32(txd1), txd2 = userlib::ulog::hex32(txd2),
               txd3 = userlib::ulog::hex32(txd3), txd4 = userlib::ulog::hex32(txd4));

        true
    }

    // =========================================================================
    // U-Boot: mtk_eth_start
    // =========================================================================

    // =========================================================================
    // Comprehensive Register Dump (to compare U-Boot vs our state)
    // =========================================================================

    fn dump_all_registers(&self, prefix: &str) {
        // This dumps ALL relevant FE/GMAC/Switch registers so we can compare
        // U-Boot's working state with our non-working state

        // FE global
        let fe_glo_misc = self.fe_read(FE_GLO_MISC_REG);
        let fe_glo_cfg = self.fe_read(0x00);  // FE_GLO_CFG at 0x00
        uinfo!("ethd", "dump_fe_glo"; prefix = prefix, glo_misc = userlib::ulog::hex32(fe_glo_misc),
               glo_cfg = userlib::ulog::hex32(fe_glo_cfg));

        // PSE registers (Packet Switching Engine)
        let pse_fqfc = self.fe_read(0x100);   // PSE_FQFC_CFG
        let pse_iqfc = self.fe_read(0x104);   // PSE_IQ_CFG
        let pse_drop = self.fe_read(0x108);   // PSE_DROP_CFG
        let pse_dumy = self.fe_read(0x10c);   // PSE_DUMY_REQ
        let pse_buf_ctrl = self.fe_read(0x110); // PSE_BUF_CTRL
        uinfo!("ethd", "dump_pse"; prefix = prefix, fqfc = userlib::ulog::hex32(pse_fqfc),
               iqfc = userlib::ulog::hex32(pse_iqfc), drop = userlib::ulog::hex32(pse_drop),
               dumy = userlib::ulog::hex32(pse_dumy));
        uinfo!("ethd", "dump_pse2"; prefix = prefix, buf_ctrl = userlib::ulog::hex32(pse_buf_ctrl));

        // PSE IQ_REV registers (free buffer thresholds per port)
        for i in 0..9u32 {
            let iq_rev = self.fe_read(0x118 + i * 4);
            if iq_rev != 0 {
                uinfo!("ethd", "dump_pse_iq"; prefix = prefix, idx = i, val = userlib::ulog::hex32(iq_rev));
            }
        }

        // PSE output queue status
        let pse_oq0 = self.fe_read(0x1a0);  // PSE_OQ_STA0
        let pse_oq1 = self.fe_read(0x1a4);  // PSE_OQ_STA1
        let pse_oq2 = self.fe_read(0x1b0);  // PSE_OQ_STA2
        uinfo!("ethd", "dump_pse_oq"; prefix = prefix, oq0 = userlib::ulog::hex32(pse_oq0),
               oq1 = userlib::ulog::hex32(pse_oq1), oq2 = userlib::ulog::hex32(pse_oq2));

        // GDM registers (all 3)
        for gdm in 0..3u32 {
            let base = match gdm {
                0 => GDMA1_BASE,
                1 => GDMA2_BASE,
                _ => GDMA3_BASE,
            };
            let ig = self.fe_read(base + GDMA_IG_CTRL_REG);
            let eg = self.fe_read(base + GDMA_EG_CTRL_REG);
            let fwd = self.fe_read(base + 0x00);  // Forward config
            let shp = self.fe_read(base + 0x04);  // Shaper config
            uinfo!("ethd", "dump_gdm"; prefix = prefix, gdm = gdm, ig = userlib::ulog::hex32(ig),
                   eg = userlib::ulog::hex32(eg), fwd = userlib::ulog::hex32(fwd), shp = userlib::ulog::hex32(shp));
        }

        // GMAC MCR and related (all 3 ports)
        for port in 0..3u32 {
            let mcr_ofs = GMAC_BASE as usize + gmac_port_mcr(port) as usize;
            let mcr = self.fe.as_ref().map(|fe| fe.read32(mcr_ofs)).unwrap_or(0);
            uinfo!("ethd", "dump_gmac"; prefix = prefix, port = port, mcr = userlib::ulog::hex32(mcr));
        }

        // MAC_MISC registers (including MUX_TO_ESW)
        let mac_misc_v3 = self.fe.as_ref().map(|fe| fe.read32(GMAC_BASE as usize + 0x10)).unwrap_or(0);
        let ppsc = self.fe.as_ref().map(|fe| fe.read32(GMAC_BASE as usize + 0x00)).unwrap_or(0);
        uinfo!("ethd", "dump_mac_misc"; prefix = prefix, mac_misc = userlib::ulog::hex32(mac_misc_v3),
               ppsc = userlib::ulog::hex32(ppsc));

        // FE reset and clock state
        let fe_rst = self.fe_read(0x04);  // FE_RST_GLO
        let fe_crc = self.fe_read(0x20);  // FE CRC config
        uinfo!("ethd", "dump_fe_rst"; prefix = prefix, rst = userlib::ulog::hex32(fe_rst),
               crc = userlib::ulog::hex32(fe_crc));

        // SGMII/path control (for GMAC to switch connection)
        // Linux: MTK_MAC_MISC = 0x10010 within FE (not GMAC offset)
        // Contains MUX_TO_ESW, path selection bits
        let mac_misc_fe = self.fe_read(0x10010);
        uinfo!("ethd", "dump_path"; prefix = prefix, mac_misc_fe = userlib::ulog::hex32(mac_misc_fe));

        // XGMAC registers for internal 10G link diagnostics
        // MTK_XGMAC_STS(0) = 0x1000C - status/force bits
        // GSW_CFG = 0x10080 - bridge IPG
        let xgmac_sts = self.fe_read(0x1000C);
        let gsw_cfg = self.fe_read(0x10080);
        uinfo!("ethd", "dump_xgmac"; prefix = prefix,
               sts = userlib::ulog::hex32(xgmac_sts),
               gsw_cfg = userlib::ulog::hex32(gsw_cfg),
               link = if xgmac_sts & 1 != 0 { 1u32 } else { 0 },
               force_link = if xgmac_sts & (1 << 11) != 0 { 1u32 } else { 0 },
               force_mode = if xgmac_sts & (1 << 15) != 0 { 1u32 } else { 0 });

        // XGMAC counters if available (offsets from Linux mtk_eth_soc.h)
        // Check for any TX/RX counters or error indicators
        // MTK_STAT_OFFSET = 0x40 for some stat registers
        let gdm0_tx_ok = self.fe_read(GDMA1_BASE + 0x40);  // TX OK count
        let gdm0_tx_drop = self.fe_read(GDMA1_BASE + 0x44); // TX drop count
        let gdm0_rx_ok = self.fe_read(GDMA1_BASE + 0x50);  // RX OK count
        let gdm0_rx_drop = self.fe_read(GDMA1_BASE + 0x54); // RX drop count
        uinfo!("ethd", "dump_gdm_stats"; prefix = prefix,
               tx_ok = gdm0_tx_ok, tx_drop = gdm0_tx_drop,
               rx_ok = gdm0_rx_ok, rx_drop = gdm0_rx_drop);

        // Ethwarp reset state
        if let Some(ew) = &self.ethwarp {
            let ew_rst = ew.read32(ETHWARP_RST_OFS);
            uinfo!("ethd", "dump_ethwarp"; prefix = prefix, rst = userlib::ulog::hex32(ew_rst));
        }

        // PDMA global config
        let pdma_glo = self.pdma_read(PDMA_GLO_CFG_REG);
        let pdma_rst = self.pdma_read(PDMA_RST_IDX_REG);
        let pdma_int_sta = self.pdma_read(0x20);  // INT status
        let pdma_int_en = self.pdma_read(0x28);   // INT enable
        uinfo!("ethd", "dump_pdma"; prefix = prefix, glo = userlib::ulog::hex32(pdma_glo),
               rst = userlib::ulog::hex32(pdma_rst), int_sta = userlib::ulog::hex32(pdma_int_sta),
               int_en = userlib::ulog::hex32(pdma_int_en));

        // PDMA TX ring 0 state
        let tx_base = self.pdma_read(tx_base_ptr_reg(0));
        let tx_max = self.pdma_read(tx_max_cnt_reg(0));
        let tx_ctx = self.pdma_read(tx_ctx_idx_reg(0));
        let tx_dtx = self.pdma_read(tx_dtx_idx_reg(0));
        uinfo!("ethd", "dump_tx_ring"; prefix = prefix, base = userlib::ulog::hex32(tx_base),
               max = tx_max, ctx = tx_ctx, dtx = tx_dtx);

        // QDMA registers (at FE_BASE + 0x4400) - maybe U-Boot uses QDMA for TX?
        const QDMA_BASE: u32 = 0x4400;
        let qdma_glo = self.fe_read(QDMA_BASE + 0x204);  // GLO_CFG
        let qdma_tx_base = self.fe_read(QDMA_BASE + 0x300);  // TX_BASE_PTR_0
        let qdma_tx_max = self.fe_read(QDMA_BASE + 0x304);   // TX_MAX_CNT_0
        let qdma_tx_ctx = self.fe_read(QDMA_BASE + 0x308);   // TX_CTX_IDX_0
        let qdma_tx_dtx = self.fe_read(QDMA_BASE + 0x30c);   // TX_DTX_IDX_0
        uinfo!("ethd", "dump_qdma"; prefix = prefix, glo = userlib::ulog::hex32(qdma_glo),
               tx_base = userlib::ulog::hex32(qdma_tx_base), tx_max = qdma_tx_max,
               tx_ctx = qdma_tx_ctx, tx_dtx = qdma_tx_dtx);

        // Switch CPU port (6) state
        if self.sw.is_some() {
            let cpu_pcr = self.gsw_read(MT7530_PCR_P(MT7531_CPU_PORT));
            let cpu_pmcr = self.gsw_read(MT7530_PMCR_P(MT7531_CPU_PORT));
            let cpu_pvc = self.gsw_read(MT7530_PVC_P(MT7531_CPU_PORT));
            uinfo!("ethd", "dump_sw_cpu"; prefix = prefix, pcr = userlib::ulog::hex32(cpu_pcr),
                   pmcr = userlib::ulog::hex32(cpu_pmcr), pvc = userlib::ulog::hex32(cpu_pvc));

            // User ports 0-3
            for port in 0..4u32 {
                let pcr = self.gsw_read(MT7530_PCR_P(port));
                let pmcr = self.gsw_read(MT7530_PMCR_P(port));
                uinfo!("ethd", "dump_sw_port"; prefix = prefix, port = port,
                       pcr = userlib::ulog::hex32(pcr), pmcr = userlib::ulog::hex32(pmcr));
            }

            // Switch system registers
            let sys_ctrl = self.gsw_read(0x7000);
            let mfc = self.gsw_read(0x0010);  // MFC - frame control
            let pfc = self.gsw_read(0x0004);  // PFC - PHY indirect
            uinfo!("ethd", "dump_sw_sys"; prefix = prefix, sys = userlib::ulog::hex32(sys_ctrl),
                   mfc = userlib::ulog::hex32(mfc), pfc = userlib::ulog::hex32(pfc));
        }
    }

    fn eth_start(&mut self) {
        // FIRST: Dump U-Boot's state BEFORE we change anything
        if DEBUG_VERBOSE {
            uinfo!("ethd", "dump_uboot_state";);
            self.dump_all_registers("UBOOT");
        }

        // Read MAC address from U-Boot's GDM registers before we overwrite them
        // U-Boot sets the MAC from EFUSE or environment, so we inherit it
        let mac_msb = self.fe_read(GDMA1_BASE + GDMA_MAC_MSB_REG);
        let mac_lsb = self.fe_read(GDMA1_BASE + GDMA_MAC_LSB_REG);
        if mac_msb != 0 || mac_lsb != 0 {
            // U-Boot left a valid MAC - use it
            // Format: MSB = [0][1], LSB = [2][3][4][5]
            self.mac[0] = ((mac_msb >> 8) & 0xFF) as u8;
            self.mac[1] = (mac_msb & 0xFF) as u8;
            self.mac[2] = ((mac_lsb >> 24) & 0xFF) as u8;
            self.mac[3] = ((mac_lsb >> 16) & 0xFF) as u8;
            self.mac[4] = ((mac_lsb >> 8) & 0xFF) as u8;
            self.mac[5] = (mac_lsb & 0xFF) as u8;
            uinfo!("ethd", "mac_from_uboot";
                   mac0 = self.mac[0], mac1 = self.mac[1], mac2 = self.mac[2],
                   mac3 = self.mac[3], mac4 = self.mac[4], mac5 = self.mac[5]);
        } else {
            uinfo!("ethd", "mac_using_default"; reason = "uboot_mac_zero");
        }

        // DON'T reset Frame Engine - preserve U-Boot's working state
        // The FE reset clears internal 10G SerDes and bridge configuration
        // that U-Boot sets up. Since U-Boot's BOOTP/TFTP works, we should
        // preserve that state rather than reset and try to reconfigure.
        //
        // If we need to reset in the future, we'd need to fully reinitialize
        // the internal 10G link including SerDes, which requires more
        // register configuration than we currently do.
        const ETHSYS_RSTCTRL: usize = 0x34;
        if let Some(ref ethsys) = self.ethsys {
            let rst_state = ethsys.read32(ETHSYS_RSTCTRL);
            uinfo!("ethd", "fe_reset_skipped"; rst_state = userlib::ulog::hex32(rst_state),
                   reason = "preserve_uboot_state");
        }

        // Do what U-Boot does: configure GDM for bridge mode
        // U-Boot's eth_halt() disables DMA but keeps GDM/PSE config.
        // However, we need to ensure GDM is properly configured for bridge mode.

        uinfo!("ethd", "configuring_gdm";);

        // MT7988 with MUX_TO_ESW=1: GMAC1 is connected to internal switch via USXGMII.
        // This means we need to use GDM1, not GDM0!
        //
        // TX path: QDMA (FPORT=1) → PSE → GDM1 → GMAC1 → USXGMII → Switch
        // RX path: Switch → USXGMII → GMAC1 → GDM1 → PDMA
        //
        // Configure GDM1 for bridge mode (matching U-Boot's mtk_eth_start with gmac_id=1)
        // U-Boot: gmac_id=1 with internal switch
        self.gdma_write(1, GDMA_IG_CTRL_REG, GDMA_BRIDGE_TO_CPU);  // 0xC0000000
        self.gdma_write(1, GDMA_EG_CTRL_REG, GDMA_CPU_BRIDGE_EN);  // 0x80000000

        // Also configure GDM0 for RX (packets come from switch via port 6 which goes through GDM0)
        // Actually on MT7988 with internal switch, RX path is: Switch → USXGMII → GDM1 → PDMA
        // But some packets might come through GDM0 depending on switch config
        self.gdma_write(0, GDMA_IG_CTRL_REG, GDMA_BRIDGE_TO_CPU);  // 0xC0000000
        self.gdma_write(0, GDMA_EG_CTRL_REG, GDMA_CPU_BRIDGE_EN);  // 0x80000000

        // Disable GDM2 (not used)
        self.gdma_write(2, GDMA_IG_CTRL_REG, GDMA_FWD_DISCARD);

        // Verify GDM config
        // NOTE: Our naming is confusing. Linux hardware naming:
        //   GDMA1_BASE (0x500) = GDM1 (for GMAC1) = where FPORT=1 sends
        //   GDMA2_BASE (0x1500) = GDM2 (for GMAC2) = where FPORT=2 sends
        // Our gdma_write(0, ...) uses GDMA1_BASE, gdma_write(1, ...) uses GDMA2_BASE
        let gdm1_ig = self.fe_read(GDMA1_BASE + GDMA_IG_CTRL_REG);  // GDM1 at 0x500 (FPORT=1 target)
        let gdm1_eg = self.fe_read(GDMA1_BASE + GDMA_EG_CTRL_REG);
        uinfo!("ethd", "gdm1_0x500_configured"; ig = userlib::ulog::hex32(gdm1_ig),
               eg = userlib::ulog::hex32(gdm1_eg));

        let gdm2_ig = self.fe_read(GDMA2_BASE + GDMA_IG_CTRL_REG);  // GDM2 at 0x1500
        let gdm2_eg = self.fe_read(GDMA2_BASE + GDMA_EG_CTRL_REG);
        uinfo!("ethd", "gdm2_0x1500_configured"; ig = userlib::ulog::hex32(gdm2_ig),
               eg = userlib::ulog::hex32(gdm2_eg));

        // Initialize switch and PHYs (this now does PHY autoneg restart)
        self.switch_init();

        // Log switch state (don't modify)
        if self.sw.is_some() {
            let cpu_pcr = self.gsw_read(MT7530_PCR_P(MT7531_CPU_PORT));
            let cpu_pmcr = self.gsw_read(MT7530_PMCR_P(MT7531_CPU_PORT));
            uinfo!("ethd", "switch_cpu_port"; pcr = userlib::ulog::hex32(cpu_pcr), pmcr = userlib::ulog::hex32(cpu_pmcr));
        }

        // Initialize DMA rings
        self.fifo_init();

        // Write MAC address to all GDMs
        let mac_msb = ((self.mac[0] as u32) << 8) | (self.mac[1] as u32);
        let mac_lsb = ((self.mac[2] as u32) << 24)
            | ((self.mac[3] as u32) << 16)
            | ((self.mac[4] as u32) << 8)
            | (self.mac[5] as u32);
        for i in 0..3 {
            self.gdma_write(i, GDMA_MAC_MSB_REG, mac_msb);
            self.gdma_write(i, GDMA_MAC_LSB_REG, mac_lsb);
        }

        // CRITICAL: For internal 10G switch mode (XGMII), the 1G MAC must be DISABLED.
        // Traffic goes through XGMAC, not the 1G MAC.
        // Reference: Linux mtk_eth_soc.c mtk_mac_config() for XGMII mode:
        //   mtk_w32(mac->hw, MAC_MCR_FORCE_LINK_DOWN, MTK_MAC_MCR(mac->id));
        // where MAC_MCR_FORCE_LINK_DOWN = MAC_MCR_FORCE_MODE (only bit 15)
        //
        // U-Boot mtk_xmac_init() also does:
        //   mtk_gmac_write(eth_priv, gmac_port_mcr(eth_priv.gmac_id), FORCE_MODE);
        {
            let mcr0_addr = GMAC_BASE as usize + gmac_port_mcr(0) as usize;
            let mcr0 = self.fe.as_ref().map(|fe| fe.read32(mcr0_addr)).unwrap_or(0);
            uinfo!("ethd", "gmac0_mcr_before"; mcr = userlib::ulog::hex32(mcr0));

            // For XGMII/internal 10G mode: set ONLY FORCE_MODE, nothing else!
            // This forces the 1G MAC link DOWN while XGMAC handles traffic.
            const MAC_FORCE_MODE: u32 = 1 << 15;  // Force link parameters
            const MAC_TX_EN: u32 = 1 << 14;
            const MAC_RX_EN: u32 = 1 << 13;
            const MAC_FORCE_LINK: u32 = 1 << 0;

            // Set only FORCE_MODE - forces 1G MAC link down, traffic goes through XGMAC
            let new_mcr0 = MAC_FORCE_MODE;
            self.fe.as_ref().map(|fe| fe.write32(mcr0_addr, new_mcr0));

            let mcr0_after = self.fe.as_ref().map(|fe| fe.read32(mcr0_addr)).unwrap_or(0);
            uinfo!("ethd", "gmac0_mcr_after"; mcr = userlib::ulog::hex32(mcr0_after),
                   tx_en = if mcr0_after & MAC_TX_EN != 0 { 1u32 } else { 0 },
                   rx_en = if mcr0_after & MAC_RX_EN != 0 { 1u32 } else { 0 },
                   force_link = if mcr0_after & MAC_FORCE_LINK != 0 { 1u32 } else { 0 });

            // CRITICAL: Force XGMAC link up for internal switch connection
            // Reference: Linux mtk_setup_bridge_switch()
            // MTK_XGMAC_STS(GMAC1_ID=0) = 0x1000C
            // MTK_XGMAC_FORCE_MODE(GMAC1_ID=0) = BIT(15)
            const XGMAC_STS_REG: usize = 0x1000C;
            const XGMAC_FORCE_MODE: u32 = 1 << 15;
            const XGMAC_FORCE_LINK: u32 = 1 << 11;

            let xgmac_before = self.fe.as_ref().map(|fe| fe.read32(XGMAC_STS_REG)).unwrap_or(0);
            // Set FORCE_MODE and FORCE_LINK bits
            let xgmac_new = xgmac_before | XGMAC_FORCE_MODE | XGMAC_FORCE_LINK;
            self.fe.as_ref().map(|fe| fe.write32(XGMAC_STS_REG, xgmac_new));
            let xgmac_after = self.fe.as_ref().map(|fe| fe.read32(XGMAC_STS_REG)).unwrap_or(0);
            uinfo!("ethd", "xgmac_sts"; before = userlib::ulog::hex32(xgmac_before),
                   after = userlib::ulog::hex32(xgmac_after),
                   force_mode = if xgmac_after & XGMAC_FORCE_MODE != 0 { 1u32 } else { 0 },
                   force_link = if xgmac_after & XGMAC_FORCE_LINK != 0 { 1u32 } else { 0 });

            // Adjust GSW bridge IPG to 11 (Linux: mtk_setup_bridge_switch)
            // MTK_GSW_CFG = 0x10080
            // GSWTX_IPG at bits [19:16], GSWRX_IPG at bits [3:0] (assumed)
            const GSW_CFG_REG: usize = 0x10080;
            const GSWTX_IPG_MASK: u32 = 0xF << 16;
            const GSWRX_IPG_MASK: u32 = 0xF << 0;
            const GSW_IPG_11: u32 = 11;

            let gsw_before = self.fe.as_ref().map(|fe| fe.read32(GSW_CFG_REG)).unwrap_or(0);
            let gsw_new = (gsw_before & !(GSWTX_IPG_MASK | GSWRX_IPG_MASK))
                        | (GSW_IPG_11 << 16) | (GSW_IPG_11 << 0);
            self.fe.as_ref().map(|fe| fe.write32(GSW_CFG_REG, gsw_new));
            let gsw_after = self.fe.as_ref().map(|fe| fe.read32(GSW_CFG_REG)).unwrap_or(0);
            uinfo!("ethd", "gsw_cfg"; before = userlib::ulog::hex32(gsw_before),
                   after = userlib::ulog::hex32(gsw_after));
        }

        for i in 1..3u32 {
            let mcr = self.fe.as_ref().map(|fe| fe.read32(GMAC_BASE as usize + gmac_port_mcr(i) as usize)).unwrap_or(0);
            uinfo!("ethd", "gmac_mcr_other"; port = i, mcr = userlib::ulog::hex32(mcr));
        }

        // Enable DMA
        if USE_QDMA {
            // QDMA TX + PDMA RX (Linux pattern)
            self.qdma_start();  // Enable QDMA TX
            self.pdma_rmw(PDMA_GLO_CFG_REG, 0, TX_WB_DDONE | RX_DMA_EN);  // PDMA RX only
            delay_us(500);

            let pdma_glo = self.pdma_read(PDMA_GLO_CFG_REG);
            let qdma_glo = self.qdma_read(QDMA_GLO_CFG);
            uinfo!("ethd", "dma_enabled_qdma";
                   pdma_glo = userlib::ulog::hex32(pdma_glo),
                   qdma_glo = userlib::ulog::hex32(qdma_glo));
        } else {
            // PDMA for both TX and RX (U-Boot style)
            self.pdma_rmw(PDMA_GLO_CFG_REG, 0, TX_WB_DDONE | RX_DMA_EN | TX_DMA_EN);
            delay_us(500);

            let glo_cfg = self.pdma_read(PDMA_GLO_CFG_REG);
            uinfo!("ethd", "dma_enabled"; glo_cfg = userlib::ulog::hex32(glo_cfg));
        }

        // LAST: Dump state AFTER all our changes
        if DEBUG_VERBOSE {
            uinfo!("ethd", "dump_our_state";);
            self.dump_all_registers("OURS");
        }

        self.link_up = true;
    }

    // =========================================================================
    // Linux mtk_eth_soc: mtk_start_xmit
    // =========================================================================

    fn eth_send(&mut self, packet: &[u8]) -> bool {
        // Use QDMA TX when enabled
        if USE_QDMA {
            return self.qdma_send(packet);
        }

        // PDMA TX path (U-Boot style)

        // Verify DMA pool is initialized
        if self.buf_vaddr == 0 {
            return false;
        }

        // Calculate TX ring address based on mode
        let tx_ring_vaddr = if USE_SRAM_FOR_RINGS && self.sram_vaddr != 0 {
            self.sram_vaddr + SRAM_TX_RING_OFF as u64
        } else {
            self.dma.as_ref().map(|d| d.vaddr() + DMA_TX_RING_OFF as u64).unwrap_or(0)
        };
        if tx_ring_vaddr == 0 {
            return false;
        }
        let desc_vaddr = tx_ring_vaddr + (self.tx_idx * TX_DESC_SIZE) as u64;

        unsafe {
            let txd = desc_vaddr as *mut TxDescV2;

            // Check if descriptor is available (DDONE set means CPU owns it)
            // SRAM requires volatile read!
            let txd2 = core::ptr::read_volatile(&(*txd).txd2);
            if (txd2 & PDMA_TXD2_DDONE) == 0 {
                uinfo!("ethd", "tx_full"; idx = self.tx_idx as u32);
                return false;
            }

            // Calculate buffer addresses
            // buf_vaddr/buf_paddr point to start of TX buffers
            let buf_offset = self.tx_idx * PKTSIZE_ALIGN;
            let buf_vaddr = self.buf_vaddr + buf_offset as u64;
            let buf_paddr = self.buf_paddr + buf_offset as u64;

            // Copy packet to buffer
            core::ptr::copy_nonoverlapping(packet.as_ptr(), buf_vaddr as *mut u8, packet.len());

            // For streaming DMA: clean cache to push data to RAM before DMA reads it
            if USE_STREAMING_DMA {
                cache_clean(buf_vaddr, packet.len());
            }

            // Memory barrier to ensure write completes before descriptor update
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            // Update descriptor - SRAM requires volatile writes!
            core::ptr::write_volatile(&mut (*txd).txd1, buf_paddr as u32);
            core::ptr::write_volatile(&mut (*txd).txd3, (buf_paddr >> 32) as u32);
            // FPORT should already be set during init, but ensure it's correct
            core::ptr::write_volatile(&mut (*txd).txd5, pdma_v2_txd5_fport_set(self.gmac_id + 1));
            // Set length and clear DDONE (gives descriptor to DMA)
            core::ptr::write_volatile(&mut (*txd).txd2, PDMA_TXD2_LS0 | pdma_v2_txd2_sdl0_set(packet.len() as u32));

            // Memory barrier after descriptor update
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Advance TX index
        self.tx_idx = (self.tx_idx + 1) % NUM_TX_DESC;
        self.pdma_write(tx_ctx_idx_reg(0), self.tx_idx as u32);

        // Debug logging
        let dtx = self.pdma_read(tx_dtx_idx_reg(0));
        let ctx = self.pdma_read(tx_ctx_idx_reg(0));
        uinfo!("ethd", "tx_sent"; idx = self.tx_idx as u32, len = packet.len() as u32, dtx = dtx, ctx = ctx);

        true
    }

    // =========================================================================
    // Linux mtk_eth_soc: mtk_poll_rx
    // =========================================================================

    fn eth_recv(&mut self) -> Option<(u64, usize)> {
        // Calculate RX ring address based on mode
        let rx_ring_vaddr = if USE_SRAM_FOR_RINGS && self.sram_vaddr != 0 {
            self.sram_vaddr + SRAM_RX_RING_OFF as u64
        } else {
            self.dma.as_ref().map(|d| d.vaddr() + DMA_RX_RING_OFF as u64)?
        };
        let desc_vaddr = rx_ring_vaddr + (self.rx_idx * RX_DESC_SIZE) as u64;

        unsafe {
            let rxd = desc_vaddr as *const RxDescV2;

            // Check if descriptor has been filled (DDONE set by DMA)
            // SRAM requires volatile read!
            let rxd2 = core::ptr::read_volatile(&(*rxd).rxd2);
            if (rxd2 & PDMA_RXD2_DDONE) == 0 {
                return None;
            }

            // Get packet length (V2/V3: PLEN0 in bits [23:8])
            let length = pdma_v2_rxd2_plen0_get(rxd2) as usize;

            // Get buffer virtual address
            // buf_vaddr points to start of TX buffers, RX buffers are after TX buffers
            let buf_vaddr = self.buf_vaddr + (TX_BUF_SIZE + self.rx_idx * PKTSIZE_ALIGN) as u64;

            // For streaming DMA: invalidate cache to discard stale data before CPU reads
            if USE_STREAMING_DMA {
                cache_invalidate(buf_vaddr, length);
            }

            Some((buf_vaddr, length))
        }
    }

    fn eth_free_pkt(&mut self) {
        // Calculate RX ring address based on mode
        let rx_ring_vaddr = if USE_SRAM_FOR_RINGS && self.sram_vaddr != 0 {
            self.sram_vaddr + SRAM_RX_RING_OFF as u64
        } else {
            match self.dma.as_ref() {
                Some(d) => d.vaddr() + DMA_RX_RING_OFF as u64,
                None => return,
            }
        };
        let desc_vaddr = rx_ring_vaddr + (self.rx_idx * RX_DESC_SIZE) as u64;

        unsafe {
            let rxd = desc_vaddr as *mut RxDescV2;

            // Reset descriptor for reuse (set PLEN0, clear DDONE)
            // SRAM requires volatile write!
            core::ptr::write_volatile(&mut (*rxd).rxd2, pdma_v2_rxd2_plen0_set(PKTSIZE_ALIGN as u32));
        }

        // Advance RX index and update hardware
        self.rx_idx = (self.rx_idx + 1) % NUM_RX_DESC;
        self.pdma_write(rx_crx_idx_reg(0), self.rx_idx as u32);
    }

    // =========================================================================
    // TX Processing (from DataPort)
    // =========================================================================

    fn process_tx(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = match self.port_id {
            Some(id) => id,
            None => {
                uinfo!("ethd", "process_tx_no_port";);
                return;
            }
        };

        // Process up to 8 TX requests
        let mut count = 0u32;
        for _ in 0..8 {
            let sqe = match ctx.block_port(port_id).and_then(|p| p.recv_request()) {
                Some(s) => s,
                None => break,
            };
            count += 1;
            uinfo!("ethd", "tx_request"; opcode = sqe.opcode, tag = sqe.tag, len = sqe.data_len);

            if sqe.opcode != io_op::NET_SEND {
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

            // Get packet from pool
            if let Some(port) = ctx.block_port(port_id) {
                if let Some(pkt_data) = port.pool_slice(sqe.data_offset, sqe.data_len) {
                    // Copy to local buffer and send
                    let mut buf = [0u8; 1536];
                    buf[..packet_len].copy_from_slice(pkt_data);

                    if self.eth_send(&buf[..packet_len]) {
                        port.complete_ok(sqe.tag, sqe.data_len);
                    } else {
                        port.complete_error(sqe.tag, io_status::NO_SPACE);
                    }
                    port.notify();
                } else {
                    port.complete_error(sqe.tag, io_status::INVALID);
                    port.notify();
                }
            }
        }
        if count > 0 {
            uinfo!("ethd", "process_tx_done"; count = count);
        }
    }

    // =========================================================================
    // RX Polling (timer-driven)
    // =========================================================================

    fn poll_rx(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = match self.port_id {
            Some(id) => id,
            None => return,
        };

        // Process up to 8 RX packets
        for _ in 0..8 {
            let (buf_vaddr, length) = match self.eth_recv() {
                Some(r) => r,
                None => break,
            };

            // Skip tiny packets
            if length < 14 {
                self.eth_free_pkt();
                continue;
            }

            // Copy packet to DataPort pool
            if let Some(port) = ctx.block_port(port_id) {
                if let Some(pool_offset) = port.alloc(length as u32) {
                    // Copy from DMA buffer to pool
                    let src = unsafe { core::slice::from_raw_parts(buf_vaddr as *const u8, length) };
                    port.pool_write(pool_offset, src);

                    // Post CQE
                    let cqe = IoCqe {
                        status: io_status::OK,
                        flags: 0,
                        tag: self.rx_seq as u32,
                        transferred: length as u32,
                        result: pool_offset,
                    };
                    port.complete(&cqe);
                    self.rx_seq += 1;
                }
                port.notify();
            }

            // Return descriptor to DMA
            self.eth_free_pkt();
        }
    }

    // =========================================================================
    // Sidechannel Handling (for ipd to query MAC/MTU)
    // =========================================================================

    fn handle_side_queries(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = match self.port_id {
            Some(id) => id,
            None => return,
        };

        // Process up to 4 sidechannel queries
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
                    side_msg::QUERY_INFO => {
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
                        // payload[6] = link status (1 = up)
                        response.payload[6] = if self.link_up { 1 } else { 0 };
                        // payload[7..9] = MTU (1500)
                        response.payload[7..9].copy_from_slice(&1500u16.to_le_bytes());

                        if let Some(port) = ctx.block_port(port_id) {
                            port.side_send(&response);
                            port.notify();
                        }
                        uinfo!("ethd", "side_query"; msg = "info_response");
                    }
                    _ => {
                        // Unknown query — respond with error
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

    // =========================================================================
    // Switch Configuration Helpers (for devc config API)
    // =========================================================================

    /// Wait for VTCR busy bit to clear
    fn wait_vtcr_ready(&self) -> bool {
        for _ in 0..1000 {
            let val = self.gsw_read(MT7530_VTCR);
            if val & VTCR_BUSY == 0 {
                return (val & VTCR_INVALID) == 0;
            }
            delay_us(20);
        }
        false
    }

    /// Wait for ATC busy bit to clear
    fn wait_atc_ready(&self) -> bool {
        for _ in 0..1000 {
            let val = self.gsw_read(MT7530_ATC);
            if val & ATC_BUSY == 0 {
                return true;
            }
            delay_us(20);
        }
        false
    }

    /// Read a VLAN entry. Returns (members, untagged) or None if invalid.
    fn vlan_read(&self, vid: u16) -> Option<(u8, u8)> {
        // Issue read command
        self.gsw_write(MT7530_VTCR, VTCR_BUSY | VTCR_FUNC_RD_VID | (vid as u32));
        if !self.wait_vtcr_ready() {
            return None;
        }

        let vawd1 = self.gsw_read(MT7530_VAWD1);
        if vawd1 & VAWD1_VLAN_VALID == 0 {
            return None;
        }

        let members = ((vawd1 >> VAWD1_PORT_MEM_SHIFT) & 0x7f) as u8;
        let vawd2 = self.gsw_read(MT7530_VAWD2);

        // Extract untagged ports from VAWD2 (2 bits per port, 0=untag)
        let mut untagged = 0u8;
        for p in 0..7 {
            if (vawd2 >> (p * 2)) & 3 == 0 {
                untagged |= 1 << p;
            }
        }
        Some((members, untagged))
    }

    /// Write a VLAN entry
    fn vlan_write(&self, vid: u16, members: u8, untagged: u8) -> bool {
        // Build VAWD1: IVL + members + valid
        let vawd1 = VAWD1_IVL_MAC | ((members as u32) << VAWD1_PORT_MEM_SHIFT) | VAWD1_VLAN_VALID;

        // Build VAWD2: egress tag control (2 bits per port)
        // 0 = untag, 2 = tag with VID
        let mut vawd2 = 0u32;
        for p in 0..7 {
            let tag_mode = if (untagged >> p) & 1 != 0 { 0 } else { 2 };
            vawd2 |= tag_mode << (p * 2);
        }

        self.gsw_write(MT7530_VAWD1, vawd1);
        self.gsw_write(MT7530_VAWD2, vawd2);
        self.gsw_write(MT7530_VTCR, VTCR_BUSY | VTCR_FUNC_WR_VID | (vid as u32));
        self.wait_vtcr_ready()
    }

    /// Delete a VLAN entry
    fn vlan_delete(&self, vid: u16) -> bool {
        self.gsw_write(MT7530_VTCR, VTCR_BUSY | VTCR_FUNC_INV_VID | (vid as u32));
        self.wait_vtcr_ready()
    }

    /// List all valid VLANs, formatting into buffer
    fn switch_vlan_list(&self, buf: &mut [u8]) -> usize {
        let mut offset = 0;
        for vid in 1..4096u16 {
            if let Some((members, untagged)) = self.vlan_read(vid) {
                if members != 0 {
                    let s = format_args!("vid={} members=0x{:02x} untagged=0x{:02x}\n",
                                         vid, members, untagged);
                    let n = fmt_to_buf(&mut buf[offset..], s);
                    offset += n;
                    if offset + 64 > buf.len() {
                        break; // Buffer full
                    }
                }
            }
        }
        offset
    }

    /// Get a single VLAN entry
    fn switch_vlan_get(&self, vid: u16, buf: &mut [u8]) -> usize {
        match self.vlan_read(vid) {
            Some((members, untagged)) => {
                fmt_to_buf(buf, format_args!("vid={} members=0x{:02x} untagged=0x{:02x}\n",
                                             vid, members, untagged))
            }
            None => fmt_to_buf(buf, format_args!("ERR VLAN {} not found\n", vid)),
        }
    }

    /// Set/create a VLAN entry. Value format: "members,untagged" (hex)
    fn switch_vlan_set(&self, vid: u16, value: &str, buf: &mut [u8]) -> usize {
        // Parse "0x0f,0x07" or "15,7"
        let mut parts = value.split(',');
        let members = match parts.next().and_then(|s| parse_u8(s.trim())) {
            Some(m) => m,
            None => return fmt_to_buf(buf, format_args!("ERR invalid members\n")),
        };
        let untagged = match parts.next().and_then(|s| parse_u8(s.trim())) {
            Some(u) => u,
            None => return fmt_to_buf(buf, format_args!("ERR invalid untagged\n")),
        };

        if self.vlan_write(vid, members, untagged) {
            fmt_to_buf(buf, format_args!("OK\n"))
        } else {
            fmt_to_buf(buf, format_args!("ERR write failed\n"))
        }
    }

    /// Delete a VLAN entry
    fn switch_vlan_del(&self, vid: u16, buf: &mut [u8]) -> usize {
        if self.vlan_delete(vid) {
            fmt_to_buf(buf, format_args!("OK\n"))
        } else {
            fmt_to_buf(buf, format_args!("ERR delete failed\n"))
        }
    }

    /// Dump the FDB (learned MAC addresses)
    fn switch_fdb_dump(&self, buf: &mut [u8]) -> usize {
        let mut offset = 0;

        // Start search
        self.gsw_write(MT7530_ATC, ATC_BUSY | ATC_CMD_SEARCH_START);
        if !self.wait_atc_ready() {
            return fmt_to_buf(buf, format_args!("ERR timeout\n"));
        }

        loop {
            let atc = self.gsw_read(MT7530_ATC);
            if atc & ATC_SRCH_END != 0 {
                break; // No more entries
            }
            if atc & ATC_SRCH_HIT != 0 {
                // Read the entry
                let tsra1 = self.gsw_read(MT7530_TSRA1);
                let tsra2 = self.gsw_read(MT7530_TSRA2);
                let atrd = self.gsw_read(MT7530_ATRD);

                // Extract MAC address
                let mac = [
                    ((tsra1 >> 24) & 0xff) as u8,
                    ((tsra1 >> 16) & 0xff) as u8,
                    ((tsra1 >> 8) & 0xff) as u8,
                    (tsra1 & 0xff) as u8,
                    ((tsra2 >> 24) & 0xff) as u8,
                    ((tsra2 >> 16) & 0xff) as u8,
                ];
                let vid = (tsra2 & 0xfff) as u16;
                let port_mask = ((atrd >> 4) & 0x7f) as u8;
                let is_static = (atrd & ATWD_LIVE) == ATWD_LIVE;

                let n = fmt_to_buf(&mut buf[offset..], format_args!(
                    "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} vid={} port=0x{:02x} {}\n",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                    vid, port_mask, if is_static { "static" } else { "dynamic" }
                ));
                offset += n;
                if offset + 80 > buf.len() {
                    break; // Buffer full
                }
            }

            // Next entry
            self.gsw_write(MT7530_ATC, ATC_BUSY | ATC_CMD_SEARCH_NEXT);
            if !self.wait_atc_ready() {
                break;
            }
        }

        if offset == 0 {
            fmt_to_buf(buf, format_args!("(empty)\n"))
        } else {
            offset
        }
    }

    /// Add a static FDB entry. Value: "mac,vid,port"
    fn switch_fdb_add(&self, value: &str, buf: &mut [u8]) -> usize {
        // Parse "00:11:22:33:44:55,1,0"
        let mut parts = value.split(',');
        let mac_str = match parts.next() {
            Some(s) => s.trim(),
            None => return fmt_to_buf(buf, format_args!("ERR missing MAC\n")),
        };
        let vid = match parts.next().and_then(|s| s.trim().parse::<u16>().ok()) {
            Some(v) => v,
            None => return fmt_to_buf(buf, format_args!("ERR invalid VID\n")),
        };
        let port = match parts.next().and_then(|s| parse_u8(s.trim())) {
            Some(p) => p,
            None => return fmt_to_buf(buf, format_args!("ERR invalid port\n")),
        };

        // Parse MAC address
        let mac = match parse_mac(mac_str) {
            Some(m) => m,
            None => return fmt_to_buf(buf, format_args!("ERR invalid MAC format\n")),
        };

        // Write ATA1: MAC bytes [47:16]
        let ata1 = ((mac[0] as u32) << 24) | ((mac[1] as u32) << 16)
                 | ((mac[2] as u32) << 8) | (mac[3] as u32);

        // Write ATA2: MAC bytes [15:0] + IVL + VID
        let ata2 = ((mac[4] as u32) << 24) | ((mac[5] as u32) << 16)
                 | (1 << 15)  // IVL bit
                 | (vid as u32);

        // Write ATWD: port mask + static entry
        let atwd = ((1u32 << port) << ATWD_PORT_MASK_SHIFT) | ATWD_LIVE;

        self.gsw_write(MT7530_ATA1, ata1);
        self.gsw_write(MT7530_ATA2, ata2);
        self.gsw_write(MT7530_ATWD, atwd);
        self.gsw_write(MT7530_ATC, ATC_BUSY | ATC_CMD_WRITE);

        if self.wait_atc_ready() {
            fmt_to_buf(buf, format_args!("OK\n"))
        } else {
            fmt_to_buf(buf, format_args!("ERR write failed\n"))
        }
    }

    /// Delete a FDB entry. Value: "mac,vid"
    fn switch_fdb_del(&self, value: &str, buf: &mut [u8]) -> usize {
        let mut parts = value.split(',');
        let mac_str = match parts.next() {
            Some(s) => s.trim(),
            None => return fmt_to_buf(buf, format_args!("ERR missing MAC\n")),
        };
        let vid = match parts.next().and_then(|s| s.trim().parse::<u16>().ok()) {
            Some(v) => v,
            None => return fmt_to_buf(buf, format_args!("ERR invalid VID\n")),
        };

        let mac = match parse_mac(mac_str) {
            Some(m) => m,
            None => return fmt_to_buf(buf, format_args!("ERR invalid MAC format\n")),
        };

        let ata1 = ((mac[0] as u32) << 24) | ((mac[1] as u32) << 16)
                 | ((mac[2] as u32) << 8) | (mac[3] as u32);
        let ata2 = ((mac[4] as u32) << 24) | ((mac[5] as u32) << 16)
                 | (1 << 15) | (vid as u32);

        self.gsw_write(MT7530_ATA1, ata1);
        self.gsw_write(MT7530_ATA2, ata2);
        self.gsw_write(MT7530_ATWD, 0); // Clear port mask
        self.gsw_write(MT7530_ATC, ATC_BUSY | ATC_CMD_WRITE);

        if self.wait_atc_ready() {
            fmt_to_buf(buf, format_args!("OK\n"))
        } else {
            fmt_to_buf(buf, format_args!("ERR delete failed\n"))
        }
    }

    /// Flush FDB. Value: "all" or port number
    fn switch_fdb_flush(&self, value: &str, buf: &mut [u8]) -> usize {
        let value = value.trim();
        if value == "all" || value.is_empty() {
            // Flush all
            self.gsw_write(MT7530_ATC, ATC_BUSY | ATC_CMD_CLEAN);
        } else {
            // Flush specific port (not directly supported, would need iteration)
            return fmt_to_buf(buf, format_args!("ERR per-port flush not supported\n"));
        }

        if self.wait_atc_ready() {
            fmt_to_buf(buf, format_args!("OK\n"))
        } else {
            fmt_to_buf(buf, format_args!("ERR flush failed\n"))
        }
    }

    /// Get STP state for a port
    fn switch_stp_get(&self, port: u8, buf: &mut [u8]) -> usize {
        if port > 6 {
            return fmt_to_buf(buf, format_args!("ERR invalid port\n"));
        }
        let ssp = self.gsw_read(MT7530_SSP_P(port as u32));
        let state = ssp & 0x3;
        let state_name = match state {
            0 => "disabled",
            1 => "blocking",
            2 => "learning",
            3 => "forwarding",
            _ => "unknown",
        };
        fmt_to_buf(buf, format_args!("{}\n", state_name))
    }

    /// Set STP state for a port
    fn switch_stp_set(&self, port: u8, value: &str, buf: &mut [u8]) -> usize {
        if port > 6 {
            return fmt_to_buf(buf, format_args!("ERR invalid port\n"));
        }
        let state = match value.trim() {
            "disabled" | "0" => 0,
            "blocking" | "1" => 1,
            "learning" | "2" => 2,
            "forwarding" | "3" => 3,
            _ => return fmt_to_buf(buf, format_args!("ERR invalid state\n")),
        };

        self.gsw_rmw(MT7530_SSP_P(port as u32), 0x3, state);
        fmt_to_buf(buf, format_args!("OK\n"))
    }

    /// Get port statistics
    fn switch_stats_get(&self, port: u8, buf: &mut [u8]) -> usize {
        if port > 6 {
            return fmt_to_buf(buf, format_args!("ERR invalid port\n"));
        }
        let base = MT7530_MIB_PORT_BASE(port as u32);

        let tx_bytes = ((self.gsw_read(base + MIB_TX_BYTES_HI) as u64) << 32)
                     | (self.gsw_read(base + MIB_TX_BYTES_LO) as u64);
        let rx_bytes = ((self.gsw_read(base + MIB_RX_BYTES_HI) as u64) << 32)
                     | (self.gsw_read(base + MIB_RX_BYTES_LO) as u64);
        let tx_uni = self.gsw_read(base + MIB_TX_UNI);
        let tx_bcast = self.gsw_read(base + MIB_TX_BCAST);
        let tx_multi = self.gsw_read(base + MIB_TX_MULTI);
        let rx_uni = self.gsw_read(base + MIB_RX_UNI);
        let rx_bcast = self.gsw_read(base + MIB_RX_BCAST);
        let rx_multi = self.gsw_read(base + MIB_RX_MULTI);
        let tx_drop = self.gsw_read(base + MIB_TX_DROP);
        let rx_drop = self.gsw_read(base + MIB_RX_DROP);
        let tx_crc = self.gsw_read(base + MIB_TX_CRC);
        let rx_crc = self.gsw_read(base + MIB_RX_CRC);
        let tx_col = self.gsw_read(base + MIB_TX_COL);

        fmt_to_buf(buf, format_args!(
            "port={}\n\
             tx_bytes={} rx_bytes={}\n\
             tx_uni={} tx_bcast={} tx_multi={}\n\
             rx_uni={} rx_bcast={} rx_multi={}\n\
             tx_drop={} rx_drop={}\n\
             tx_crc={} rx_crc={} tx_col={}\n",
            port, tx_bytes, rx_bytes,
            tx_uni, tx_bcast, tx_multi,
            rx_uni, rx_bcast, rx_multi,
            tx_drop, rx_drop,
            tx_crc, rx_crc, tx_col
        ))
    }

    /// Get mirror configuration
    fn switch_mirror_get(&self, buf: &mut [u8]) -> usize {
        let mfc = self.gsw_read(MT7530_MFC);
        let enabled = (mfc & MFC_MIRROR_EN) != 0;
        let dest = ((mfc & MFC_MIRROR_PORT_MASK) >> MFC_MIRROR_PORT_SHIFT) as u8;

        // Collect source ports
        let mut sources = 0u8;
        for p in 0..7 {
            let pcr = self.gsw_read(MT7530_PCR_MIR(p));
            if (pcr & (PCR_TX_MIR | PCR_RX_MIR)) != 0 {
                sources |= 1 << p;
            }
        }

        if enabled {
            fmt_to_buf(buf, format_args!("dest={} sources=0x{:02x}\n", dest, sources))
        } else {
            fmt_to_buf(buf, format_args!("disabled\n"))
        }
    }

    /// Set mirror configuration. Value: "dest,sources,mode"
    fn switch_mirror_set(&self, value: &str, buf: &mut [u8]) -> usize {
        let mut parts = value.split(',');
        let dest = match parts.next().and_then(|s| parse_u8(s.trim())) {
            Some(d) if d <= 6 => d,
            _ => return fmt_to_buf(buf, format_args!("ERR invalid dest port\n")),
        };
        let sources = match parts.next().and_then(|s| parse_u8(s.trim())) {
            Some(s) => s,
            None => return fmt_to_buf(buf, format_args!("ERR invalid sources\n")),
        };
        let mode = parts.next().map(|s| s.trim()).unwrap_or("both");

        let (rx_en, tx_en) = match mode {
            "rx" => (true, false),
            "tx" => (false, true),
            "both" => (true, true),
            _ => return fmt_to_buf(buf, format_args!("ERR invalid mode\n")),
        };

        // Set MFC mirror port and enable
        let mfc = self.gsw_read(MT7530_MFC);
        let mfc_new = (mfc & !MFC_MIRROR_PORT_MASK)
                    | ((dest as u32) << MFC_MIRROR_PORT_SHIFT)
                    | MFC_MIRROR_EN;
        self.gsw_write(MT7530_MFC, mfc_new);

        // Configure source ports
        for p in 0..7 {
            let mut pcr = self.gsw_read(MT7530_PCR_MIR(p));
            pcr &= !(PCR_TX_MIR | PCR_RX_MIR);
            if (sources >> p) & 1 != 0 {
                if rx_en { pcr |= PCR_RX_MIR; }
                if tx_en { pcr |= PCR_TX_MIR; }
            }
            self.gsw_write(MT7530_PCR_MIR(p), pcr);
        }

        fmt_to_buf(buf, format_args!("OK\n"))
    }

    /// Disable mirroring
    fn switch_mirror_off(&self, buf: &mut [u8]) -> usize {
        // Clear MFC mirror enable
        self.gsw_rmw(MT7530_MFC, MFC_MIRROR_EN, 0);

        // Clear all port mirror bits
        for p in 0..7 {
            self.gsw_rmw(MT7530_PCR_MIR(p), PCR_TX_MIR | PCR_RX_MIR, 0);
        }

        fmt_to_buf(buf, format_args!("OK\n"))
    }

    /// Get age time
    fn switch_age_get(&self, buf: &mut [u8]) -> usize {
        let aac = self.gsw_read(MT7530_AAC);
        let cnt = (aac >> AAC_AGE_CNT_SHIFT) & 0xff;
        let unit = aac & 0xfff;
        // Age time = cnt * unit * 2 seconds (approximate)
        let seconds = (cnt as u32) * (unit as u32) * 2;
        fmt_to_buf(buf, format_args!("{}\n", seconds))
    }

    /// Set age time in seconds
    fn switch_age_set(&self, value: &str, buf: &mut [u8]) -> usize {
        let seconds = match value.trim().parse::<u32>() {
            Ok(s) => s,
            Err(_) => return fmt_to_buf(buf, format_args!("ERR invalid value\n")),
        };

        // Convert to count/unit (simplified: use unit=1, count=seconds/2)
        let unit = 1u32;
        let cnt = (seconds / 2).min(255);

        let aac = (cnt << AAC_AGE_CNT_SHIFT) | unit;
        self.gsw_write(MT7530_AAC, aac);

        fmt_to_buf(buf, format_args!("OK\n"))
    }

    /// Get DMA mode
    fn switch_dma_get(&self, buf: &mut [u8]) -> usize {
        let mode = if USE_QDMA { "qdma" } else { "pdma" };
        fmt_to_buf(buf, format_args!("{}\n", mode))
    }

    /// Set DMA mode (informational only - requires recompile)
    fn switch_dma_set(&self, _value: &str, buf: &mut [u8]) -> usize {
        fmt_to_buf(buf, format_args!("ERR DMA mode requires recompile (USE_QDMA={})\n", USE_QDMA))
    }

    // =========================================================================
    // Combined Statistics (top-level `stats` key)
    // =========================================================================

    /// Get all statistics (FE + switch ports)
    fn stats_get_all(&self, buf: &mut [u8]) -> usize {
        use core::fmt::Write;
        struct BufWriter<'a> { buf: &'a mut [u8], pos: usize }
        impl Write for BufWriter<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                let bytes = s.as_bytes();
                let remaining = self.buf.len() - self.pos;
                let to_write = bytes.len().min(remaining);
                self.buf[self.pos..self.pos + to_write].copy_from_slice(&bytes[..to_write]);
                self.pos += to_write;
                Ok(())
            }
        }
        let mut w = BufWriter { buf, pos: 0 };

        let _ = core::write!(w,
            "=== Switch Ports ===\n\
             port  tx_bytes      rx_bytes      tx_pkts  rx_pkts  drop\n"
        );

        // Switch port stats (compact summary)
        for port in 0..7u8 {
            let base = MT7530_MIB_PORT_BASE(port as u32);
            let tx_bytes = ((self.gsw_read(base + MIB_TX_BYTES_HI) as u64) << 32)
                         | (self.gsw_read(base + MIB_TX_BYTES_LO) as u64);
            let rx_bytes = ((self.gsw_read(base + MIB_RX_BYTES_HI) as u64) << 32)
                         | (self.gsw_read(base + MIB_RX_BYTES_LO) as u64);
            let tx_pkts = self.gsw_read(base + MIB_TX_UNI)
                        + self.gsw_read(base + MIB_TX_BCAST)
                        + self.gsw_read(base + MIB_TX_MULTI);
            let rx_pkts = self.gsw_read(base + MIB_RX_UNI)
                        + self.gsw_read(base + MIB_RX_BCAST)
                        + self.gsw_read(base + MIB_RX_MULTI);
            let drop = self.gsw_read(base + MIB_TX_DROP)
                     + self.gsw_read(base + MIB_RX_DROP);

            let _ = core::write!(w,
                "{:<5} {:<13} {:<13} {:<8} {:<8} {}\n",
                port, tx_bytes, rx_bytes, tx_pkts, rx_pkts, drop
            );
        }

        w.pos
    }

    // =========================================================================
    // Frame Engine Config Helpers (fe.*)
    // =========================================================================

    /// Show FE config help
    fn fe_help(&self, buf: &mut [u8]) -> usize {
        fmt_to_buf(buf, format_args!(
            "Frame Engine configuration:\n\
             fe.csum       - RX checksum offload (on/off)\n\
             fe.coalesce   - Interrupt coalescing (rx_usec,rx_pkts,tx_usec,tx_pkts)\n\
             fe.pause      - Flow control/pause frames (rx_en,tx_en)\n\
             fe.regs       - Register dump (debug)\n\n\
             Use 'devc ethd get stats' for combined FE+switch statistics.\n\n\
             Examples:\n\
             devc ethd set fe.csum on\n\
             devc ethd set fe.coalesce 100,16,100,16\n\
             devc ethd set fe.pause on,on\n"
        ))
    }

    /// Get checksum offload status
    fn fe_csum_get(&self, buf: &mut [u8]) -> usize {
        if !USE_CHECKSUM_OFFLOAD {
            return fmt_to_buf(buf, format_args!("disabled (compile-time)\n"));
        }

        // Read current GDMA_IG_CTRL setting
        let ig_ctrl = self.fe_read(GDMA1_BASE + GDMA_IG_CTRL_REG);
        let ics = (ig_ctrl & GDM_ICS_EN) != 0;
        let tcs = (ig_ctrl & GDM_TCS_EN) != 0;
        let ucs = (ig_ctrl & GDM_UCS_EN) != 0;

        fmt_to_buf(buf, format_args!(
            "rx_csum={} ip={} tcp={} udp={}\n",
            if self.csum_rx_en { "on" } else { "off" },
            if ics { "on" } else { "off" },
            if tcs { "on" } else { "off" },
            if ucs { "on" } else { "off" }
        ))
    }

    /// Set checksum offload (on/off)
    fn fe_csum_set(&mut self, value: &str, buf: &mut [u8]) -> usize {
        if !USE_CHECKSUM_OFFLOAD {
            return fmt_to_buf(buf, format_args!("ERR disabled (compile-time)\n"));
        }

        let enable = match value.trim() {
            "on" | "1" | "true" => true,
            "off" | "0" | "false" => false,
            _ => return fmt_to_buf(buf, format_args!("ERR use 'on' or 'off'\n")),
        };

        self.csum_rx_en = enable;

        // Update GDMA_IG_CTRL for GMAC0 (internal switch)
        let csum_bits = GDM_ICS_EN | GDM_TCS_EN | GDM_UCS_EN;
        let mut ig_ctrl = self.fe_read(GDMA1_BASE + GDMA_IG_CTRL_REG);
        if enable {
            ig_ctrl |= csum_bits;
        } else {
            ig_ctrl &= !csum_bits;
        }
        self.fe_write(GDMA1_BASE + GDMA_IG_CTRL_REG, ig_ctrl);

        fmt_to_buf(buf, format_args!("OK rx_csum={}\n", if enable { "on" } else { "off" }))
    }

    /// Get interrupt coalescing settings
    fn fe_coalesce_get(&self, buf: &mut [u8]) -> usize {
        if !USE_INTERRUPT_COALESCING {
            return fmt_to_buf(buf, format_args!("disabled (compile-time)\n"));
        }

        // Read current delay interrupt register
        let dly = self.pdma_read(PDMA_DELAY_IRQ);
        let rx_en = (dly & DLY_RX_EN) != 0;
        let tx_en = (dly & DLY_TX_EN) != 0;
        let rx_ptime = (dly >> DLY_RX_PTIME_SHIFT) & DLY_PTIME_MASK;
        let rx_pint = (dly >> DLY_RX_PINT_SHIFT) & DLY_PINT_MASK;
        let tx_ptime = (dly >> DLY_TX_PTIME_SHIFT) & DLY_PTIME_MASK;
        let tx_pint = (dly >> DLY_TX_PINT_SHIFT) & DLY_PINT_MASK;

        fmt_to_buf(buf, format_args!(
            "rx_usec={} rx_pkts={} rx_en={}\n\
             tx_usec={} tx_pkts={} tx_en={}\n\
             (state: rx_usec={} rx_pkts={} tx_usec={} tx_pkts={})\n",
            rx_ptime * 20, rx_pint, if rx_en { "on" } else { "off" },
            tx_ptime * 20, tx_pint, if tx_en { "on" } else { "off" },
            self.coalesce_rx_usec, self.coalesce_rx_pkts,
            self.coalesce_tx_usec, self.coalesce_tx_pkts
        ))
    }

    /// Set interrupt coalescing (rx_usec,rx_pkts,tx_usec,tx_pkts)
    fn fe_coalesce_set(&mut self, value: &str, buf: &mut [u8]) -> usize {
        if !USE_INTERRUPT_COALESCING {
            return fmt_to_buf(buf, format_args!("ERR disabled (compile-time)\n"));
        }

        // Parse "rx_usec,rx_pkts,tx_usec,tx_pkts" or "off"
        let value = value.trim();
        if value == "off" || value == "0" {
            // Disable coalescing
            let dly = self.pdma_read(PDMA_DELAY_IRQ);
            self.pdma_write(PDMA_DELAY_IRQ, dly & !(DLY_RX_EN | DLY_TX_EN));
            if USE_QDMA {
                let qdly = self.qdma_read(QDMA_DELAY_IRQ);
                self.qdma_write(QDMA_DELAY_IRQ, qdly & !(DLY_RX_EN | DLY_TX_EN));
            }
            return fmt_to_buf(buf, format_args!("OK coalescing disabled\n"));
        }

        let mut parts = value.split(',');
        let rx_usec: u32 = parts.next().and_then(|s| s.trim().parse().ok()).unwrap_or(0);
        let rx_pkts: u32 = parts.next().and_then(|s| s.trim().parse().ok()).unwrap_or(0);
        let tx_usec: u32 = parts.next().and_then(|s| s.trim().parse().ok()).unwrap_or(0);
        let tx_pkts: u32 = parts.next().and_then(|s| s.trim().parse().ok()).unwrap_or(0);

        // Convert microseconds to 20µs units
        let rx_ptime = (rx_usec / 20).min(DLY_PTIME_MASK);
        let rx_pint = rx_pkts.min(DLY_PINT_MASK);
        let tx_ptime = (tx_usec / 20).min(DLY_PTIME_MASK);
        let tx_pint = tx_pkts.min(DLY_PINT_MASK);

        // Build delay register value
        let mut dly_val = 0u32;
        if rx_usec > 0 || rx_pkts > 0 {
            dly_val |= DLY_RX_EN | (rx_pint << DLY_RX_PINT_SHIFT) | (rx_ptime << DLY_RX_PTIME_SHIFT);
        }
        if tx_usec > 0 || tx_pkts > 0 {
            dly_val |= DLY_TX_EN | (tx_pint << DLY_TX_PINT_SHIFT) | (tx_ptime << DLY_TX_PTIME_SHIFT);
        }

        self.pdma_write(PDMA_DELAY_IRQ, dly_val);
        if USE_QDMA {
            self.qdma_write(QDMA_DELAY_IRQ, dly_val);
        }

        // Save state
        self.coalesce_rx_usec = rx_usec;
        self.coalesce_rx_pkts = rx_pkts;
        self.coalesce_tx_usec = tx_usec;
        self.coalesce_tx_pkts = tx_pkts;

        fmt_to_buf(buf, format_args!(
            "OK rx_usec={} rx_pkts={} tx_usec={} tx_pkts={}\n",
            rx_ptime * 20, rx_pint, tx_ptime * 20, tx_pint
        ))
    }

    /// Get flow control (pause frame) status
    fn fe_pause_get(&self, buf: &mut [u8]) -> usize {
        if !USE_FLOW_CONTROL {
            return fmt_to_buf(buf, format_args!("disabled (compile-time)\n"));
        }

        // Read MAC control register for GMAC0
        let mcr = self.fe_read(GMAC_BASE + gmac_port_mcr(0));
        let rx_fc = (mcr & FORCE_RX_FC) != 0;
        let tx_fc = (mcr & FORCE_TX_FC) != 0;

        fmt_to_buf(buf, format_args!(
            "rx_pause={} tx_pause={}\n\
             (state: rx={} tx={})\n",
            if rx_fc { "on" } else { "off" },
            if tx_fc { "on" } else { "off" },
            if self.pause_rx_en { "on" } else { "off" },
            if self.pause_tx_en { "on" } else { "off" }
        ))
    }

    /// Set flow control (rx_en,tx_en or on/off)
    fn fe_pause_set(&mut self, value: &str, buf: &mut [u8]) -> usize {
        if !USE_FLOW_CONTROL {
            return fmt_to_buf(buf, format_args!("ERR disabled (compile-time)\n"));
        }

        let value = value.trim();

        // Parse "rx,tx" or "on"/"off" for both
        let (rx_en, tx_en) = if value == "on" || value == "1" {
            (true, true)
        } else if value == "off" || value == "0" {
            (false, false)
        } else {
            let mut parts = value.split(',');
            let rx = match parts.next().map(|s| s.trim()) {
                Some("on") | Some("1") => true,
                Some("off") | Some("0") => false,
                _ => return fmt_to_buf(buf, format_args!("ERR use 'on,on' or 'off,off'\n")),
            };
            let tx = match parts.next().map(|s| s.trim()) {
                Some("on") | Some("1") => true,
                Some("off") | Some("0") => false,
                _ => return fmt_to_buf(buf, format_args!("ERR use 'on,on' or 'off,off'\n")),
            };
            (rx, tx)
        };

        self.pause_rx_en = rx_en;
        self.pause_tx_en = tx_en;

        // Update MAC control register for GMAC0
        let mut mcr = self.fe_read(GMAC_BASE + gmac_port_mcr(0));
        if rx_en {
            mcr |= FORCE_RX_FC;
        } else {
            mcr &= !FORCE_RX_FC;
        }
        if tx_en {
            mcr |= FORCE_TX_FC;
        } else {
            mcr &= !FORCE_TX_FC;
        }
        self.fe_write(GMAC_BASE + gmac_port_mcr(0), mcr);

        fmt_to_buf(buf, format_args!(
            "OK rx_pause={} tx_pause={}\n",
            if rx_en { "on" } else { "off" },
            if tx_en { "on" } else { "off" }
        ))
    }

    /// Get Frame Engine (GDMA) statistics - redirect to combined stats
    fn fe_stats_get(&self, buf: &mut [u8]) -> usize {
        // Note: GDMA doesn't have packet counters. Real stats are in switch MIB.
        fmt_to_buf(buf, format_args!(
            "Use 'devc ethd get stats' for packet statistics.\n\
             (GDMA provides config, not counters - stats are in switch MIB)\n"
        ))
    }

    /// Dump Frame Engine registers (debug)
    fn fe_regs_get(&self, buf: &mut [u8]) -> usize {
        let gdma_ig = self.fe_read(GDMA1_BASE + GDMA_IG_CTRL_REG);
        let gdma_eg = self.fe_read(GDMA1_BASE + GDMA_EG_CTRL_REG);
        let mcr = self.fe_read(GMAC_BASE + gmac_port_mcr(0));
        let pdma_dly = self.pdma_read(PDMA_DELAY_IRQ);
        let qdma_dly = if USE_QDMA { self.qdma_read(QDMA_DELAY_IRQ) } else { 0 };

        fmt_to_buf(buf, format_args!(
            "GDMA1_IG_CTRL=0x{:08x} (csum: ics={} tcs={} ucs={})\n\
             GDMA1_EG_CTRL=0x{:08x}\n\
             GMAC0_MCR=0x{:08x} (rx_fc={} tx_fc={})\n\
             PDMA_DELAY=0x{:08x} (rx_en={} tx_en={})\n\
             QDMA_DELAY=0x{:08x}\n",
            gdma_ig,
            (gdma_ig >> 22) & 1, (gdma_ig >> 21) & 1, (gdma_ig >> 20) & 1,
            gdma_eg,
            mcr, (mcr >> 5) & 1, (mcr >> 4) & 1,
            pdma_dly, (pdma_dly >> 15) & 1, (pdma_dly >> 31) & 1,
            qdma_dly
        ))
    }
}

/// Format arguments to buffer, returns bytes written
fn fmt_to_buf(buf: &mut [u8], args: core::fmt::Arguments<'_>) -> usize {
    use core::fmt::Write;
    struct BufWriter<'a> { buf: &'a mut [u8], pos: usize }
    impl Write for BufWriter<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let bytes = s.as_bytes();
            let remaining = self.buf.len() - self.pos;
            let to_write = bytes.len().min(remaining);
            self.buf[self.pos..self.pos + to_write].copy_from_slice(&bytes[..to_write]);
            self.pos += to_write;
            Ok(())
        }
    }
    let mut w = BufWriter { buf, pos: 0 };
    let _ = core::fmt::write(&mut w, args);
    w.pos
}

/// Parse a u8 value from string (supports "0x" prefix)
fn parse_u8(s: &str) -> Option<u8> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u8::from_str_radix(hex, 16).ok()
    } else {
        s.parse().ok()
    }
}

/// Parse MAC address from "xx:xx:xx:xx:xx:xx" format
fn parse_mac(s: &str) -> Option<[u8; 6]> {
    let mut mac = [0u8; 6];
    let mut idx = 0;
    for part in s.split(':') {
        if idx >= 6 {
            return None;
        }
        mac[idx] = u8::from_str_radix(part, 16).ok()?;
        idx += 1;
    }
    if idx != 6 {
        return None;
    }
    Some(mac)
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for EthDriver {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        uinfo!("ethd", "init";);

        // Map Frame Engine MMIO
        let fe = MmioRegion::open(FE_BASE, FE_SIZE).ok_or_else(|| {
            uerror!("ethd", "fe_mmio_failed";);
            BusError::Internal
        })?;

        // DIAGNOSTIC: Test FE register write capability
        // We need to verify MMIO writes work, not just reads.
        // Test using FE_GLO_MISC at offset 0x124
        {
            let glo_misc = fe.read32(FE_GLO_MISC_REG as usize);
            uinfo!("ethd", "fe_glo_misc_uboot"; val = userlib::ulog::hex32(glo_misc),
                   pdma_v2 = if (glo_misc & PDMA_VER_V2) != 0 { 1u32 } else { 0u32 });

            // Test: Try setting BIT(4) if not already set
            if (glo_misc & PDMA_VER_V2) == 0 {
                uinfo!("ethd", "set_pdma_v2"; reason = "not_set_by_uboot");
                fe.write32(FE_GLO_MISC_REG as usize, glo_misc | PDMA_VER_V2);
                let glo_misc_after = fe.read32(FE_GLO_MISC_REG as usize);
                uinfo!("ethd", "fe_glo_misc_after"; val = userlib::ulog::hex32(glo_misc_after),
                       pdma_v2 = if (glo_misc_after & PDMA_VER_V2) != 0 { 1u32 } else { 0u32 });
            }

            // Test: Write to PSE_DUMY_REQ register (offset 0x10C) - should be writable
            // This is a scratch/dummy register often used for testing
            let pse_dumy_before = fe.read32(0x10c);
            fe.write32(0x10c, 0x12345678);
            let pse_dumy_after = fe.read32(0x10c);
            uinfo!("ethd", "fe_write_test"; reg = "pse_dumy", before = userlib::ulog::hex32(pse_dumy_before),
                   wrote = userlib::ulog::hex32(0x12345678), after = userlib::ulog::hex32(pse_dumy_after));

            // Restore original value
            fe.write32(0x10c, pse_dumy_before);

            // Test: Direct SRAM access at FE offset 0x40000
            // SRAM should be within FE region (FE_SIZE = 0x80000 = 512KB)
            let sram_test_offset = FE_SRAM_OFFSET as usize;
            let sram_before = fe.read32(sram_test_offset);
            uinfo!("ethd", "sram_via_fe_before"; offset = userlib::ulog::hex32(sram_test_offset as u32),
                   val = userlib::ulog::hex32(sram_before));

            fe.write32(sram_test_offset, 0xCAFEBABE);
            let sram_after = fe.read32(sram_test_offset);
            uinfo!("ethd", "sram_via_fe_after"; offset = userlib::ulog::hex32(sram_test_offset as u32),
                   wrote = userlib::ulog::hex32(0xCAFEBABE), read = userlib::ulog::hex32(sram_after));

            // If sram_after != 0xCAFEBABE, SRAM writes are being ignored!
            if sram_after != 0xCAFEBABE {
                uerror!("ethd", "sram_write_failed"; expected = userlib::ulog::hex32(0xCAFEBABE),
                        got = userlib::ulog::hex32(sram_after));
            }
        }

        self.fe = Some(fe);

        // Map Internal Switch MMIO (separate region at 0x15020000)
        match MmioRegion::open(SWITCH_BASE, SWITCH_SIZE) {
            Some(sw) => {
                uinfo!("ethd", "switch_mmio_ok"; base = userlib::ulog::hex64(SWITCH_BASE));
                self.sw = Some(sw);
            }
            None => {
                uwarn!("ethd", "switch_mmio_failed"; base = userlib::ulog::hex64(SWITCH_BASE));
                // Continue without switch - SFP+ might still work
            }
        }

        // Map Ethwarp reset controller (0x15031000)
        match MmioRegion::open(ETHWARP_BASE, ETHWARP_SIZE) {
            Some(ew) => {
                uinfo!("ethd", "ethwarp_mmio_ok"; base = userlib::ulog::hex64(ETHWARP_BASE));
                self.ethwarp = Some(ew);
            }
            None => {
                uwarn!("ethd", "ethwarp_mmio_failed"; base = userlib::ulog::hex64(ETHWARP_BASE));
            }
        }

        // Map ETHSYS system control (0x15000000)
        // Reference: Linux clk-mt7988-eth.c for clock gate definitions
        match MmioRegion::open(ETHSYS_BASE, ETHSYS_SIZE) {
            Some(es) => {
                uinfo!("ethd", "ethsys_mmio_ok"; base = userlib::ulog::hex64(ETHSYS_BASE));

                // Read current clock gate status
                let clk_gate_before = es.read32(ETHSYS_CLK_GATE_REG);
                uinfo!("ethd", "ethsys_clk_gate_before"; val = userlib::ulog::hex32(clk_gate_before));

                // Enable all necessary clocks for Ethernet + SRAM
                // Linux enables these via mtk_clk_enable() in mtk_hw_init()
                // Inverted logic: 1 = enabled
                let clk_enable_mask = ETHSYS_CLK_FE_EN       // Frame Engine (required)
                                    | ETHSYS_CLK_GP1_EN     // GMAC1 port clock
                                    | ETHSYS_CLK_GP2_EN     // GMAC2 port clock
                                    | ETHSYS_CLK_GP3_EN     // GMAC3 port clock
                                    | ETHSYS_CLK_ESW_EN     // Embedded Switch (required for internal switch)
                                    | ETHSYS_CLK_XGP1_EN    // 2.5G mode for GMAC1
                                    | ETHSYS_CLK_XGP2_EN    // 2.5G mode for GMAC2
                                    | ETHSYS_CLK_XGP3_EN;   // 2.5G mode for GMAC3

                let new_clk_gate = clk_gate_before | clk_enable_mask;
                es.write32(ETHSYS_CLK_GATE_REG, new_clk_gate);

                // Verify the write took effect
                let clk_gate_after = es.read32(ETHSYS_CLK_GATE_REG);
                uinfo!("ethd", "ethsys_clk_gate_after"; val = userlib::ulog::hex32(clk_gate_after),
                       fe_en = if (clk_gate_after & ETHSYS_CLK_FE_EN) != 0 { 1u32 } else { 0u32 },
                       esw_en = if (clk_gate_after & ETHSYS_CLK_ESW_EN) != 0 { 1u32 } else { 0u32 });

                // Small delay after clock enable for HW to stabilize
                delay_us(100);

                // Also read other ETHSYS registers for debugging
                let dummy_reg = es.read32(ETHSYS_DUMMY_REG);
                let sys_cfg = es.read32(0x10);   // ETHSYS_SYSCFG
                let sys_cfg0 = es.read32(0x14);  // ETHSYS_SYSCFG0
                uinfo!("ethd", "ethsys_regs"; dummy = userlib::ulog::hex32(dummy_reg),
                       syscfg = userlib::ulog::hex32(sys_cfg), syscfg0 = userlib::ulog::hex32(sys_cfg0));

                self.ethsys = Some(es);
            }
            None => {
                uwarn!("ethd", "ethsys_mmio_failed"; base = userlib::ulog::hex64(ETHSYS_BASE));
            }
        }

        // Map Pinctrl (GPIO) for LED pin muxing
        match MmioRegion::open(PINCTRL_BASE, PINCTRL_SIZE) {
            Some(pc) => {
                uinfo!("ethd", "pinctrl_mmio_ok"; base = userlib::ulog::hex64(PINCTRL_BASE));
                self.pinctrl = Some(pc);
            }
            None => {
                uwarn!("ethd", "pinctrl_mmio_failed"; base = userlib::ulog::hex64(PINCTRL_BASE));
            }
        }

        // MT7988 SRAM is at separate address 0x15400000 (NOT FE_BASE + 0x40000!)
        // Reference: linux/arch/arm64/boot/dts/mediatek/mt7988a.dtsi
        // The SRAM requires Frame Engine clocks to be enabled (done above via ETHSYS_CLK_GATE_REG)
        if USE_SRAM_FOR_RINGS {
            match MmioRegion::open(SRAM_BASE, SRAM_SIZE) {
                Some(sr) => {
                    // Quick SRAM verification - write and read back
                    sr.write32(0, 0xCAFEBABE);
                    let test_val = sr.read32(0);

                    if test_val == 0xCAFEBABE {
                        uinfo!("ethd", "sram_ok"; base = userlib::ulog::hex64(SRAM_BASE),
                               size = userlib::ulog::hex64(SRAM_SIZE));
                        self.sram_vaddr = sr.virt_base();
                        self.sram_paddr = SRAM_BASE;
                        // Keep the mapping alive
                        core::mem::forget(sr);
                    } else {
                        uerror!("ethd", "sram_write_failed"; expected = userlib::ulog::hex32(0xCAFEBABE),
                                got = userlib::ulog::hex32(test_val));
                        // Fall back to DMA memory
                    }
                }
                None => {
                    uerror!("ethd", "sram_mmio_failed"; base = userlib::ulog::hex64(SRAM_BASE));
                }
            }
        }

        uinfo!("ethd", "sram_config";
               use_sram = if USE_SRAM_FOR_RINGS && self.sram_vaddr != 0 { 1u32 } else { 0u32 },
               paddr = userlib::ulog::hex64(if self.sram_vaddr != 0 { self.sram_paddr } else { 0 }));

        // Allocate DMA pool for data buffers (8MB: 4MB TX + 4MB RX)
        // Descriptor rings go to SRAM, only buffers go to DMA pool
        // Use streaming (cacheable) or coherent (non-cacheable) mode based on config
        let mut dma = if USE_STREAMING_DMA {
            uinfo!("ethd", "dma_mode"; mode = "streaming");
            DmaPool::alloc_streaming(DMA_POOL_SIZE)
        } else {
            uinfo!("ethd", "dma_mode"; mode = "coherent");
            DmaPool::alloc(DMA_POOL_SIZE)
        }.ok_or_else(|| {
            uerror!("ethd", "dma_failed"; size = DMA_POOL_SIZE as u64);
            BusError::Internal
        })?;
        dma.zero();

        let paddr = dma.paddr();
        uinfo!("ethd", "dma_pool";
               paddr = userlib::ulog::hex64(paddr),
               size_mb = (DMA_POOL_SIZE / (1024 * 1024)) as u32);
        self.dma = Some(dma);

        // Initialize hardware (U-Boot style)
        self.eth_start();

        // Create DataPort
        let config = BlockPortConfig {
            ring_size: 64,
            side_size: 4,
            pool_size: 128 * 1024,
        };
        let port_id = ctx.create_block_port(config).map_err(|e| {
            uerror!("ethd", "port_failed";);
            e
        })?;
        // Make port publicly accessible so ipd can connect
        if let Some(port) = ctx.block_port(port_id) {
            port.set_public();
        }
        self.port_id = Some(port_id);

        // Register as net0
        let shmem_id = ctx.block_port(port_id).map(|p| p.shmem_id()).unwrap_or(0);
        let _ = ctx.register_port_with_metadata(b"net0", PortType::Network, shmem_id, None, &self.mac);
        uinfo!("ethd", "registered"; name = "net0");

        // Start RX poll timer (10ms)
        if let Ok(mut timer) = Timer::new() {
            let _ = timer.set(10_000_000);  // 10ms in ns
            let handle = timer.handle();
            let _ = ctx.watch_handle(handle, self.poll_tag);
            self.poll_timer = Some(timer);
        }

        uinfo!("ethd", "init_complete";);
        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::QUERY_INFO => {
                // Format: "net0 LINK=up/down"
                let mut info = [0u8; 64];
                let status_str = if self.link_up {
                    b"net0 LINK=up" as &[u8]
                } else {
                    b"net0 LINK=down" as &[u8]
                };
                info[..status_str.len()].copy_from_slice(status_str);
                let _ = ctx.respond_info(msg.seq_id, &info[..status_str.len()]);
                Disposition::Handled
            }
            _ => Disposition::Forward,
        }
    }

    fn data_ready(&mut self, _port: PortId, ctx: &mut dyn BusCtx) {
        uinfo!("ethd", "data_ready";);
        self.process_tx(ctx);
        self.handle_side_queries(ctx);
    }

    fn handle_event(&mut self, tag: u32, _handle: Handle, ctx: &mut dyn BusCtx) {
        if tag == self.poll_tag {
            self.poll_rx(ctx);
            self.handle_side_queries(ctx);

            // Every 100 polls (~1 second), dump TX descriptor state and MIB counters
            self.poll_count += 1;
            if DEBUG_VERBOSE && self.poll_count % 100 == 0 {
                if USE_QDMA {
                    // QDMA TX status
                    let ctx = self.qdma_read(QDMA_CTX_PTR);
                    let dtx = self.qdma_read(QDMA_DTX_PTR);
                    let glo = self.qdma_read(QDMA_GLO_CFG);
                    let fq_cnt = self.qdma_read(QDMA_FQ_COUNT);
                    uinfo!("ethd", "qdma_status"; ctx = userlib::ulog::hex32(ctx), dtx = userlib::ulog::hex32(dtx),
                           glo = userlib::ulog::hex32(glo), fq_cnt = userlib::ulog::hex32(fq_cnt), poll = self.poll_count);

                    // Also show PDMA RX status (we use PDMA for RX)
                    let pdma_glo = self.pdma_read(PDMA_GLO_CFG_REG);
                    let rx_drx = self.pdma_read(rx_drx_idx_reg(0));
                    let rx_crx = self.pdma_read(rx_crx_idx_reg(0));
                    uinfo!("ethd", "pdma_rx_status"; glo = userlib::ulog::hex32(pdma_glo),
                           rx_drx = rx_drx, rx_crx = rx_crx);
                } else if self.sram_vaddr != 0 {
                    let tx_ring_vaddr = self.sram_vaddr + SRAM_TX_RING_OFF as u64;
                    let dtx = self.pdma_read(tx_dtx_idx_reg(0));
                    let ctx_idx = self.pdma_read(tx_ctx_idx_reg(0));

                    // Read txd2 of first 8 descriptors to see DDONE state (sample only, ring is 2048)
                    let mut ddone_bits = 0u32;
                    for i in 0..8 {
                        let txd2 = unsafe {
                            let txd = &*((tx_ring_vaddr + (i * TX_DESC_SIZE) as u64) as *const TxDescV2);
                            txd.txd2
                        };
                        if txd2 & PDMA_TXD2_DDONE != 0 {
                            ddone_bits |= 1 << i;
                        }
                    }
                    uinfo!("ethd", "tx_status"; dtx = dtx, ctx = ctx_idx, ddone_mask = userlib::ulog::hex32(ddone_bits), poll = self.poll_count);
                } else {
                    // Regular DMA mode (no SRAM, no QDMA)
                    let pdma_glo = self.pdma_read(PDMA_GLO_CFG_REG);
                    let dtx = self.pdma_read(tx_dtx_idx_reg(0));
                    let ctx_idx = self.pdma_read(tx_ctx_idx_reg(0));
                    uinfo!("ethd", "pdma_status"; glo = userlib::ulog::hex32(pdma_glo),
                           dtx = dtx, ctx = ctx_idx, poll = self.poll_count);
                }

                // Read switch MIB counters for ALL ports (0-6)
                // MIB base = 0x4000, per-port = port * 0x100
                // TX counters: 0x00=TxDrop, 0x04=TxCRC, 0x08=TxUnicast, 0x0C=TxMulticast, 0x10=TxBroadcast
                //              0x24=TxOctets_lo, 0x28=TxOctets_hi
                // RX counters: 0x50=RxDrop, 0x54=RxFiltering, 0x58=RxUnicast, 0x5C=RxMulticast, 0x60=RxBroadcast
                //              0x80=RxOctets_lo, 0x84=RxOctets_hi
                for port in 0..7u32 {
                    let mib_base = 0x4000 + port * 0x100;
                    let tx_oct = self.gsw_read(mib_base + 0x24);  // TX bytes (low 32 bits)
                    let rx_oct = self.gsw_read(mib_base + 0x80);  // RX bytes (low 32 bits)
                    let tx_uni = self.gsw_read(mib_base + 0x08);
                    let tx_bcast = self.gsw_read(mib_base + 0x10);
                    let rx_uni = self.gsw_read(mib_base + 0x58);
                    let rx_bcast = self.gsw_read(mib_base + 0x60);
                    let tx_drop = self.gsw_read(mib_base + 0x00);
                    let rx_drop = self.gsw_read(mib_base + 0x50);
                    uinfo!("ethd", "mib"; port = port,
                           tx_oct = tx_oct, rx_oct = rx_oct,
                           tx_u = tx_uni, tx_b = tx_bcast,
                           rx_u = rx_uni, rx_b = rx_bcast,
                           tx_drop = tx_drop, rx_drop = rx_drop);
                }

                // Check PHY link status on all 4 ports
                let mut link_mask = 0u32;
                for phy in 0..4u8 {
                    if let Some(status) = self.mdio_read(phy, 1) {  // BMSR
                        if (status & 0x04) != 0 {  // Link bit
                            link_mask |= 1 << phy;
                        }
                    }
                }
                uinfo!("ethd", "phy_links"; mask = userlib::ulog::hex32(link_mask));

                // Read FE status registers to debug packet path
                // PSE buffer status, GDM counters
                let pse_fq = self.fe_read(0x100);  // PSE_FQFC_CFG - free queue config
                let pse_drop = self.fe_read(0x108); // PSE drop config
                let pse_oqc = self.fe_read(0x1b0);  // PSE_OQ_STA - output queue status

                // GDM counters for NETSYS V3 (MT7988) - base at 0x1c00
                // Reference: Linux mtk_eth_soc.c: .gdm1_cnt = 0x1c00, MTK_STAT_OFFSET = 0x40
                const GDM_CNT_BASE: u32 = 0x1c00;

                // Dump all 3 GMACs counters
                for gmac in 0..3u32 {
                    let offs = gmac * 0x40;  // MTK_STAT_OFFSET
                    let rx_bytes = self.fe_read(GDM_CNT_BASE + offs + 0x00);
                    let rx_pkts = self.fe_read(GDM_CNT_BASE + offs + 0x08);
                    let tx_bytes = self.fe_read(GDM_CNT_BASE + offs + 0x40);
                    let tx_pkts = self.fe_read(GDM_CNT_BASE + offs + 0x44);
                    let drop = self.fe_read(GDM_CNT_BASE + offs + 0x54);
                    uinfo!("ethd", "gdm_cnt"; gmac = gmac, rx_b = rx_bytes, rx_p = rx_pkts,
                           tx_b = tx_bytes, tx_p = tx_pkts, drop = drop);
                }

                // Dump all 3 GMAC MCR registers
                for gmac in 0..3u32 {
                    let mcr = self.fe_read(GMAC_BASE + gmac_port_mcr(gmac));
                    uinfo!("ethd", "gmac_mcr"; port = gmac, mcr = userlib::ulog::hex32(mcr));
                }

                // PDMA status
                let pdma_glo = self.pdma_read(PDMA_GLO_CFG_REG);
                let pdma_tx_ptr = self.pdma_read(tx_base_ptr_reg(0));
                let pdma_tx_ctx = self.pdma_read(tx_ctx_idx_reg(0));
                let pdma_tx_dtx = self.pdma_read(tx_dtx_idx_reg(0));
                uinfo!("ethd", "pdma_state"; glo = userlib::ulog::hex32(pdma_glo),
                       tx_ptr = userlib::ulog::hex32(pdma_tx_ptr), ctx = pdma_tx_ctx, dtx = pdma_tx_dtx);

                uinfo!("ethd", "fe_status"; pse_fq = userlib::ulog::hex32(pse_fq), pse_drop = userlib::ulog::hex32(pse_drop),
                       pse_oq = userlib::ulog::hex32(pse_oqc));
            }

            // Re-arm the timer
            if let Some(ref mut timer) = self.poll_timer {
                let _ = timer.set(10_000_000);  // 10ms
            }
        }
    }

    // =========================================================================
    // Config API (devc ethd get/set switch.* and fe.*)
    // =========================================================================

    fn config_keys(&self) -> &[userlib::bus::ConfigKey] {
        use userlib::bus::ConfigKey;
        static KEYS: [ConfigKey; 11] = [
            // Top-level
            ConfigKey::read_only(b"stats"),         // Combined FE + switch stats
            // Switch management keys
            ConfigKey::read_only(b"switch.vlan"),
            ConfigKey::read_only(b"switch.fdb"),
            ConfigKey::read_only(b"switch.mirror"),
            ConfigKey::read_only(b"switch.age"),
            ConfigKey::read_only(b"switch.dma"),
            // Frame Engine keys
            ConfigKey::read_write(b"fe.csum"),      // Checksum offload
            ConfigKey::read_write(b"fe.coalesce"),  // Interrupt coalescing
            ConfigKey::read_write(b"fe.pause"),     // Flow control
            ConfigKey::read_only(b"fe.stats"),      // GDMA statistics
            ConfigKey::read_only(b"fe.regs"),       // Register dump (debug)
        ];
        &KEYS
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        let key_str = core::str::from_utf8(key).unwrap_or("");

        // Handle empty key (summary)
        if key_str.is_empty() {
            return fmt_to_buf(buf, format_args!(
                "keys: stats switch.* fe.*\n\
                 switch: vlan fdb stp.<port> stats.<port> mirror age dma\n\
                 fe: csum coalesce pause regs\n"
            ));
        }

        // Top-level stats (combined FE + switch)
        if key_str == "stats" {
            return self.stats_get_all(buf);
        }

        // Frame Engine keys
        if key_str == "fe" || key_str == "fe.help" {
            return self.fe_help(buf);
        }
        if key_str == "fe.csum" {
            return self.fe_csum_get(buf);
        }
        if key_str == "fe.coalesce" {
            return self.fe_coalesce_get(buf);
        }
        if key_str == "fe.pause" {
            return self.fe_pause_get(buf);
        }
        if key_str == "fe.stats" {
            return self.fe_stats_get(buf);
        }
        if key_str == "fe.regs" {
            return self.fe_regs_get(buf);
        }

        // Switch keys
        if key_str == "switch.vlan" {
            return self.switch_vlan_list(buf);
        }
        if let Some(rest) = key_str.strip_prefix("switch.vlan.") {
            if let Ok(vid) = rest.parse::<u16>() {
                return self.switch_vlan_get(vid, buf);
            }
        }
        if key_str == "switch.fdb" {
            return self.switch_fdb_dump(buf);
        }
        if let Some(rest) = key_str.strip_prefix("switch.stp.") {
            if let Ok(port) = rest.parse::<u8>() {
                return self.switch_stp_get(port, buf);
            }
        }
        if let Some(rest) = key_str.strip_prefix("switch.stats.") {
            if let Ok(port) = rest.parse::<u8>() {
                return self.switch_stats_get(port, buf);
            }
        }
        if key_str == "switch.mirror" {
            return self.switch_mirror_get(buf);
        }
        if key_str == "switch.age" {
            return self.switch_age_get(buf);
        }
        if key_str == "switch.dma" {
            return self.switch_dma_get(buf);
        }

        0 // Unknown key
    }

    fn config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], _ctx: &mut dyn BusCtx) -> usize {
        let key_str = core::str::from_utf8(key).unwrap_or("");
        let val_str = core::str::from_utf8(value).unwrap_or("");

        // Frame Engine operations
        if key_str == "fe.csum" {
            return self.fe_csum_set(val_str, buf);
        }
        if key_str == "fe.coalesce" {
            return self.fe_coalesce_set(val_str, buf);
        }
        if key_str == "fe.pause" {
            return self.fe_pause_set(val_str, buf);
        }

        // VLAN operations
        if let Some(rest) = key_str.strip_prefix("switch.vlan.") {
            if rest == "del" {
                if let Ok(vid) = val_str.trim().parse::<u16>() {
                    return self.switch_vlan_del(vid, buf);
                }
                return fmt_to_buf(buf, format_args!("ERR invalid VID\n"));
            }
            if let Ok(vid) = rest.parse::<u16>() {
                return self.switch_vlan_set(vid, val_str, buf);
            }
        }

        // FDB operations
        if key_str == "switch.fdb.add" {
            return self.switch_fdb_add(val_str, buf);
        }
        if key_str == "switch.fdb.del" {
            return self.switch_fdb_del(val_str, buf);
        }
        if key_str == "switch.fdb.flush" {
            return self.switch_fdb_flush(val_str, buf);
        }

        // STP operations
        if let Some(rest) = key_str.strip_prefix("switch.stp.") {
            if let Ok(port) = rest.parse::<u8>() {
                return self.switch_stp_set(port, val_str, buf);
            }
        }

        // Mirror operations
        if key_str == "switch.mirror" {
            return self.switch_mirror_set(val_str, buf);
        }
        if key_str == "switch.mirror.off" {
            return self.switch_mirror_off(buf);
        }

        // Stats reset
        if key_str == "switch.stats.reset" {
            // MIB reset not implemented (would need CCR register)
            return fmt_to_buf(buf, format_args!("ERR not implemented\n"));
        }

        // Age time
        if key_str == "switch.age" {
            return self.switch_age_set(val_str, buf);
        }

        // DMA mode
        if key_str == "switch.dma" {
            return self.switch_dma_set(val_str, buf);
        }

        fmt_to_buf(buf, format_args!("ERR unknown key\n"))
    }
}

// =============================================================================
// Entry Point
// =============================================================================

static mut DRIVER: EthDriver = EthDriver {
    fe: None,
    sw: None,
    ethwarp: None,
    ethsys: None,
    pinctrl: None,
    dma: None,
    sram_vaddr: 0,
    sram_paddr: 0,
    buf_vaddr: 0,
    buf_paddr: 0,
    tx_idx: 0,
    rx_idx: 0,
    port_id: None,
    poll_timer: None,
    poll_tag: 1,
    rx_seq: 0,
    mac: [0x02, 0x00, 0x00, 0x00, 0x00, 0x01],
    link_up: false,
    gmac_id: 0,             // GMAC0 for internal switch (Linux DTS: gmac0 -> switch port 6)
    use_bridge_mode: true,  // Default to bridge mode for internal switch
    poll_count: 0,

    // QDMA state (initialized when USE_QDMA is true)
    qdma_fq_paddr: 0,
    qdma_fq_count: 0,
    qdma_tx_paddr: 0,
    qdma_tx_next: 0,

    // Frame Engine runtime config (defaults)
    csum_rx_en: true,                          // RX checksum enabled by default
    coalesce_rx_usec: DEFAULT_COALESCE_USEC,   // 50µs
    coalesce_rx_pkts: DEFAULT_COALESCE_PKTS,   // 8 packets
    coalesce_tx_usec: DEFAULT_COALESCE_USEC,   // 50µs
    coalesce_tx_pkts: DEFAULT_COALESCE_PKTS,   // 8 packets
    pause_rx_en: true,                         // RX pause enabled
    pause_tx_en: true,                         // TX pause enabled
};

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
    fn config_keys(&self) -> &[userlib::bus::ConfigKey] {
        self.0.config_keys()
    }
    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        self.0.config_get(key, buf)
    }
    fn config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], ctx: &mut dyn BusCtx) -> usize {
        self.0.config_set(key, value, buf, ctx)
    }
}

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"ethd", EthDriverWrapper(driver));
}
