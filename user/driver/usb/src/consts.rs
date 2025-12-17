//! USB driver constants and hardware addresses

// =============================================================================
// MT7988A SSUSB1 hardware addresses
// =============================================================================

// Physical addresses (for MMIO scheme)
// MT7988A has two USB controllers: SSUSB0 and SSUSB1

// SSUSB0 (first USB controller) - from MT7988A DTS
pub const SSUSB0_MAC_PHYS: u64 = 0x1119_0000;
pub const SSUSB0_PHY_PHYS: u64 = 0x11f2_0000;  // XFI T-PHY (xfi_tphy0), NOT 0x11e10000!
pub const SSUSB0_IRQ: u32 = 173 + 32;  // GIC SPI 173 = interrupt ID 205

// SSUSB1 (second USB controller)
pub const SSUSB1_MAC_PHYS: u64 = 0x1120_0000;
pub const SSUSB1_PHY_PHYS: u64 = 0x11c5_0000;  // T-PHY v2
pub const SSUSB1_IRQ: u32 = 172 + 32;  // GIC SPI 172 = interrupt ID 204

// =============================================================================
// Clock and Reset Control (INFRACFG_AO)
// =============================================================================
// MT7988A uses infracfg_ao for clock gates and resets
// Clock gates: bit=1 means GATED (disabled), bit=0 means ENABLED
// Resets: bit=1 means RESET asserted, bit=0 means RESET deasserted

pub const INFRACFG_AO_BASE: u64 = 0x1000_1000;
pub const INFRACFG_AO_SIZE: u64 = 0x1000;

// Module reset registers (active high - write 1 to assert reset)
pub const INFRA_RST0_SET: usize = 0x30;   // Write 1 to assert reset
pub const INFRA_RST0_CLR: usize = 0x34;   // Write 1 to deassert reset
pub const INFRA_RST0_STA: usize = 0x38;   // Read reset status

pub const INFRA_RST1_SET: usize = 0x40;
pub const INFRA_RST1_CLR: usize = 0x44;
pub const INFRA_RST1_STA: usize = 0x48;

// Clock gate registers (active high - bit=1 means gated/disabled)
pub const INFRA_CG0_SET: usize = 0x80;    // Write 1 to gate (disable) clock
pub const INFRA_CG0_CLR: usize = 0x84;    // Write 1 to ungate (enable) clock
pub const INFRA_CG0_STA: usize = 0x90;    // Read clock gate status

pub const INFRA_CG1_SET: usize = 0x88;
pub const INFRA_CG1_CLR: usize = 0x8C;
pub const INFRA_CG1_STA: usize = 0x94;

pub const INFRA_CG2_SET: usize = 0xA4;
pub const INFRA_CG2_CLR: usize = 0xA8;
pub const INFRA_CG2_STA: usize = 0xAC;

// USB reset bits (in RST1)
pub const RST1_SSUSB_TOP0: u32 = 1 << 7;    // SSUSB0 top reset
pub const RST1_SSUSB_TOP1: u32 = 1 << 8;    // SSUSB1 top reset

// USB clock gate bits (need to find correct register/bits)
// These may be in CG0, CG1, or CG2 depending on the specific IP
pub const CG1_SSUSB0: u32 = 1 << 18;        // SSUSB0 bus clock (approximate)
pub const CG1_SSUSB1: u32 = 1 << 19;        // SSUSB1 bus clock (approximate)

// Common sizes
pub const SSUSB_MAC_SIZE: u64 = 0x4000;       // Includes IPPC at offset 0x3e00
pub const SSUSB_IPPC_OFFSET: usize = 0x3e00;  // IPPC is within MAC region
pub const SSUSB0_PHY_SIZE: u64 = 0x10000;     // XFI T-PHY is 64KB
pub const SSUSB1_PHY_SIZE: u64 = 0x1000;      // Regular T-PHY is 4KB

// =============================================================================
// GPIO IPC Protocol (for requesting USB VBUS enable)
// =============================================================================

pub const GPIO_CMD_USB_VBUS: u8 = 3;   // Control USB VBUS: [3, enable]
