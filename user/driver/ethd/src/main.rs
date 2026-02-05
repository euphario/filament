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
const ETHSYS_BASE: u64 = 0x1500_0000;
const ETHSYS_SIZE: u64 = 0x1000;
const ETHSYS_DUMMY_REG: usize = 0x0c;  // Dummy register with SRAM enable
const ETHSYS_SRAM_EN: u32 = 1 << 15;   // BIT(15) enables SRAM access

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
const DMA_RING_OFF: usize = 0;                                      // Rings at start
const DMA_TX_RING_OFF: usize = DMA_RING_OFF;                        // 0
const DMA_RX_RING_OFF: usize = DMA_RING_OFF + TX_RING_SIZE;         // 64KB
const DMA_RINGS_SIZE: usize = TX_RING_SIZE + RX_RING_SIZE;          // 128KB
const DMA_TX_BUF_OFF: usize = DMA_RINGS_SIZE;                       // 128KB
const DMA_RX_BUF_OFF: usize = DMA_RINGS_SIZE + TX_BUF_SIZE;         // 128KB + 4MB
const DMA_POOL_SIZE: usize = DMA_RINGS_SIZE + TX_BUF_SIZE + RX_BUF_SIZE;  // ~8.125MB

// DMA mode configuration
// Set to true to use streaming (cacheable) DMA for data buffers.
// Streaming DMA provides better CPU performance but requires explicit cache sync.
// Set to false to use coherent (non-cacheable) DMA for data buffers (default).
const USE_STREAMING_DMA: bool = false;  // Use coherent (non-cacheable) DMA for simplicity

// SRAM vs DMA memory for descriptor rings
// MT7988 SRAM is at 0x15400000 (separate from FE), requires clocks to be enabled.
// Set to true to test SRAM, false to use DMA memory (like U-Boot does).
const USE_SRAM_FOR_RINGS: bool = false;  // DMA can't access SRAM - use regular DMA memory like U-Boot

// Debug verbosity flag
// Set to true to enable detailed register dumps and diagnostics during init.
// Set to false for production (reduces log spam significantly).
const DEBUG_VERBOSE: bool = false;

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

        // Set up PDMA registers
        self.pdma_write(tx_base_ptr_reg(0), tx_ring_paddr as u32);
        self.pdma_write(tx_max_cnt_reg(0), NUM_TX_DESC as u32);
        self.pdma_write(tx_ctx_idx_reg(0), 0);

        self.pdma_write(rx_base_ptr_reg(0), rx_ring_paddr as u32);
        self.pdma_write(rx_max_cnt_reg(0), NUM_RX_DESC as u32);
        self.pdma_write(rx_crx_idx_reg(0), (NUM_RX_DESC - 1) as u32);

        // Reset DMA indices
        self.pdma_write(PDMA_RST_IDX_REG, RST_DTX_IDX0 | RST_DRX_IDX0);

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

        // Set GDM0 for bridge mode (matching U-Boot's mtk_eth_start)
        // U-Boot code in mtk_eth_start() for V3 with internal switch:
        //   mtk_gdma_write(eth_priv, gmac_id, GDMA_IG_CTRL_REG, GDMA_BRIDGE_TO_CPU);  // 0xC0000000
        //   mtk_gdma_write(eth_priv, gmac_id, GDMA_EG_CTRL_REG, GDMA_CPU_BRIDGE_EN);  // 0x80000000
        //
        // The U-Boot dump showing eg=0x00000000 is from BEFORE mtk_eth_start or after eth_halt.
        // During active operation, U-Boot sets both registers.
        self.gdma_write(0, GDMA_IG_CTRL_REG, GDMA_BRIDGE_TO_CPU);  // 0xC0000000
        self.gdma_write(0, GDMA_EG_CTRL_REG, GDMA_CPU_BRIDGE_EN);  // 0x80000000

        // Disable other GDMs (discard their traffic)
        self.gdma_write(1, GDMA_IG_CTRL_REG, GDMA_FWD_DISCARD);
        self.gdma_write(2, GDMA_IG_CTRL_REG, GDMA_FWD_DISCARD);

        // Verify GDM config
        let gdm0_ig = self.fe_read(GDMA1_BASE + GDMA_IG_CTRL_REG);
        let gdm0_eg = self.fe_read(GDMA1_BASE + GDMA_EG_CTRL_REG);
        uinfo!("ethd", "gdm0_configured"; ig = userlib::ulog::hex32(gdm0_ig),
               eg = userlib::ulog::hex32(gdm0_eg));

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

        // Enable DMA (U-Boot style)
        self.pdma_rmw(PDMA_GLO_CFG_REG, 0, TX_WB_DDONE | RX_DMA_EN | TX_DMA_EN);
        delay_us(500);

        let glo_cfg = self.pdma_read(PDMA_GLO_CFG_REG);
        uinfo!("ethd", "dma_enabled"; glo_cfg = userlib::ulog::hex32(glo_cfg));

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
        // NOTE: Previous code tried to set BIT(15) at offset 0x0c (ETHSYS_DUMMY_REG) but
        // this appears to be for older chips, not MT7988. For MT7988 (NETSYS V3), SRAM
        // is enabled via BIT(4) in FE_GLO_MISC (already done above).
        match MmioRegion::open(ETHSYS_BASE, ETHSYS_SIZE) {
            Some(es) => {
                uinfo!("ethd", "ethsys_mmio_ok"; base = userlib::ulog::hex64(ETHSYS_BASE));

                // Dump ETHSYS registers for debugging
                let dummy_reg = es.read32(ETHSYS_DUMMY_REG);
                let clk_cfg0 = es.read32(0x00);  // CLK_CFG_0
                let clk_cfg1 = es.read32(0x04);  // CLK_CFG_1
                uinfo!("ethd", "ethsys_regs"; dummy = userlib::ulog::hex32(dummy_reg),
                       clk0 = userlib::ulog::hex32(clk_cfg0), clk1 = userlib::ulog::hex32(clk_cfg1));

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
        // NOTE: SRAM may require specific clocks to be enabled - Linux enables ~20 clocks!
        if USE_SRAM_FOR_RINGS {
            match MmioRegion::open(SRAM_BASE, SRAM_SIZE) {
                Some(sr) => {
                    uinfo!("ethd", "sram_mmio_ok"; base = userlib::ulog::hex64(SRAM_BASE));

                    // Test SRAM write capability at the correct address
                    let test_before = sr.read32(0);
                    sr.write32(0, 0xCAFEBABE);
                    let test_after = sr.read32(0);
                    uinfo!("ethd", "sram_real_test"; before = userlib::ulog::hex32(test_before),
                           wrote = userlib::ulog::hex32(0xCAFEBABE), after = userlib::ulog::hex32(test_after));

                    if test_after == 0xCAFEBABE {
                        uinfo!("ethd", "sram_working";);
                        self.sram_vaddr = sr.virt_base();
                        self.sram_paddr = SRAM_BASE;
                        // Keep the mapping alive by storing handle (handled via Drop)
                        core::mem::forget(sr);  // Leak the handle to keep mapping
                    } else {
                        uerror!("ethd", "sram_write_failed"; reason = "clocks_not_enabled?");
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
                if self.sram_vaddr != 0 {
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
}

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"ethd", EthDriverWrapper(driver));
}
