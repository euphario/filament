//! MediaTek MT7988A USB SoC Wrapper
//!
//! The MT7988A has two USB host controllers:
//!
//! ## SSUSB0 (XFI T-PHY)
//! - MAC base: 0x11190000
//! - PHY base: 0x11F20000 (XFI T-PHY)
//! - Combo PHY (USB/PCIe/XFI)
//! - Connects to M.2 slot on BPI-R4
//!
//! ## SSUSB1 (T-PHY v2)
//! - MAC base: 0x11200000
//! - PHY base: 0x11C50000 (USB2 at +0, USB3 at +0x700)
//! - Dedicated USB PHY
//! - Connects to VL822 hub -> USB-A ports on BPI-R4
//!
//! ## IPPC (IP Port Control)
//! MediaTek's USB IP wrapper provides:
//! - Power and clock control
//! - Port enable/disable
//! - USB2/USB3 port count readback
//! - PHY PLL control

use super::{SocUsb, SocError, PortCount};
use crate::mmio::MmioRegion;

/// IPPC register offsets (from MAC base + IPPC_OFFSET)
mod ippc {
    /// IP Power/Clock control 0
    pub const IP_PW_CTRL0: usize = 0x00;
    /// IP Power/Clock control 1
    pub const IP_PW_CTRL1: usize = 0x04;
    /// IP Power/Clock control 2
    pub const IP_PW_CTRL2: usize = 0x08;
    /// IP Power status 1 (port count)
    pub const IP_PW_STS1: usize = 0x10;
    /// IP Power status 2
    pub const IP_PW_STS2: usize = 0x14;
    /// USB3 control 0
    pub const U3_CTRL_0P: usize = 0x30;
    /// USB2 control 0
    pub const U2_CTRL_0P: usize = 0x50;
    /// USB2 PHY PLL control
    pub const U2_PHY_PLL: usize = 0x7C;

    /// Power control bits
    pub mod ctrl {
        /// Power down IP
        pub const IP_SW_RST: u32 = 1 << 0;
        /// Reference clock gate
        pub const REF_CK_GATE: u32 = 1 << 1;
        /// SYS clock gate enable
        pub const SYS_CK_GATE_DIS: u32 = 1 << 2;
    }

    /// Status bits
    pub mod sts {
        /// USB3 port count shift
        pub const U3_PORT_NUM_SHIFT: u32 = 8;
        /// USB3 port count mask
        pub const U3_PORT_NUM_MASK: u32 = 0xFF << 8;
        /// USB2 port count shift
        pub const U2_PORT_NUM_SHIFT: u32 = 0;
        /// USB2 port count mask
        pub const U2_PORT_NUM_MASK: u32 = 0xFF;
        /// SYSPLL stable
        pub const SYSPLL_STABLE: u32 = 1 << 0;
        /// REF clock stable
        pub const REF_CLK_STABLE: u32 = 1 << 8;
    }
}

/// MT7988A USB controller addresses
pub mod addrs {
    /// SSUSB0 MAC base address
    pub const SSUSB0_MAC: u64 = 0x11190000;
    /// SSUSB0 PHY base address (XFI T-PHY)
    pub const SSUSB0_PHY: u64 = 0x11E10000;
    /// SSUSB0 PHY size
    pub const SSUSB0_PHY_SIZE: usize = 0x10000;

    /// SSUSB1 MAC base address
    pub const SSUSB1_MAC: u64 = 0x11200000;
    /// SSUSB1 PHY base address (T-PHY v2)
    /// USB2 PHY at 0x11c50000 (size 0x700), USB3 PHY at 0x11c50700 (size 0x900)
    pub const SSUSB1_PHY: u64 = 0x11C50000;
    /// SSUSB1 PHY size
    pub const SSUSB1_PHY_SIZE: usize = 0x10000;

    /// MAC region size (includes IPPC)
    pub const MAC_SIZE: usize = 0x4000;

    /// IPPC offset from MAC base
    pub const IPPC_OFFSET: usize = 0x3E00;

    /// IRQ numbers (GIC: SPI + 32)
    pub const SSUSB0_IRQ: u32 = 173 + 32;  // SPI 173 = IRQ 205
    pub const SSUSB1_IRQ: u32 = 172 + 32;  // SPI 172 = IRQ 204

    // INFRACFG_AO - Clock and Reset Control
    pub const INFRACFG_AO_BASE: u64 = 0x1000_1000;
    pub const INFRACFG_AO_SIZE: u64 = 0x1000;

    // Reset registers
    pub const INFRA_RST1_CLR: usize = 0x44;  // Write 1 to deassert reset

    // Clock gate registers
    pub const INFRA_CG1_CLR: usize = 0x8C;   // Write 1 to ungate (enable) clock

    // USB reset bits (in RST1)
    pub const RST1_SSUSB_TOP0: u32 = 1 << 7;
    pub const RST1_SSUSB_TOP1: u32 = 1 << 8;

    // USB clock gate bits (in CG1)
    pub const CG1_SSUSB0: u32 = 1 << 18;
    pub const CG1_SSUSB1: u32 = 1 << 19;
}

/// MT7988A USB controller index
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerId {
    /// SSUSB0 - XFI T-PHY (M.2 slot)
    Ssusb0 = 0,
    /// SSUSB1 - T-PHY v2 (USB-A ports via VL822)
    Ssusb1 = 1,
}

/// MediaTek MT7988A USB SoC wrapper
///
/// Handles IPPC initialization for the MT7988A USB controllers.
pub struct Mt7988aSoc {
    /// Controller ID (0 or 1)
    id: ControllerId,
    /// MAC MMIO region (includes IPPC)
    mac: Option<MmioRegion>,
    /// Cached port counts
    usb3_ports: u8,
    usb2_ports: u8,
}

impl Mt7988aSoc {
    /// Create a new MT7988A SoC wrapper for the specified controller
    pub fn new(id: ControllerId) -> Self {
        Self {
            id,
            mac: None,
            usb3_ports: 0,
            usb2_ports: 0,
        }
    }

    /// Create wrapper for SSUSB0
    pub fn ssusb0() -> Self {
        Self::new(ControllerId::Ssusb0)
    }

    /// Create wrapper for SSUSB1
    pub fn ssusb1() -> Self {
        Self::new(ControllerId::Ssusb1)
    }

    /// Set the MAC MMIO region
    pub fn set_mac(&mut self, mmio: MmioRegion) {
        self.mac = Some(mmio);
    }

    /// Get MAC base address for this controller
    pub fn mac_base(&self) -> u64 {
        match self.id {
            ControllerId::Ssusb0 => addrs::SSUSB0_MAC,
            ControllerId::Ssusb1 => addrs::SSUSB1_MAC,
        }
    }

    /// Get PHY base address for this controller
    pub fn phy_base(&self) -> u64 {
        match self.id {
            ControllerId::Ssusb0 => addrs::SSUSB0_PHY,
            ControllerId::Ssusb1 => addrs::SSUSB1_PHY,
        }
    }

    /// Get PHY region size for this controller
    pub fn phy_size(&self) -> usize {
        match self.id {
            ControllerId::Ssusb0 => addrs::SSUSB0_PHY_SIZE,
            ControllerId::Ssusb1 => addrs::SSUSB1_PHY_SIZE,
        }
    }

    /// Global USB initialization - deassert resets and enable clocks
    ///
    /// This must be called once before initializing any USB controller.
    /// It configures the INFRACFG_AO registers to:
    /// - Deassert USB controller resets
    /// - Enable USB clock gates
    ///
    /// Returns true on success, false on failure.
    pub fn global_init() -> bool {
        use crate::mmio::delay;

        // Map INFRACFG_AO region using unified object interface
        let mmio = match MmioRegion::open(addrs::INFRACFG_AO_BASE, addrs::INFRACFG_AO_SIZE) {
            Some(m) => m,
            None => return false,
        };

        // Deassert USB resets for both controllers
        let usb_rst_bits = addrs::RST1_SSUSB_TOP0 | addrs::RST1_SSUSB_TOP1;
        mmio.write32(addrs::INFRA_RST1_CLR, usb_rst_bits);
        delay(1000);

        // Enable USB clocks for both controllers
        let usb_cg_bits = addrs::CG1_SSUSB0 | addrs::CG1_SSUSB1;
        mmio.write32(addrs::INFRA_CG1_CLR, usb_cg_bits);
        delay(10000);

        // mmio dropped here, region unmapped
        true
    }

    /// Read IPPC register
    fn ippc_read32(&self, offset: usize) -> Result<u32, SocError> {
        self.mac.as_ref()
            .map(|m| m.read32(addrs::IPPC_OFFSET + offset))
            .ok_or(SocError::NoMmio)
    }

    /// Write IPPC register
    fn ippc_write32(&self, offset: usize, value: u32) -> Result<(), SocError> {
        self.mac.as_ref()
            .map(|m| m.write32(addrs::IPPC_OFFSET + offset, value))
            .ok_or(SocError::NoMmio)
    }

    /// Initialize IPPC (called during pre_init)
    fn ippc_init(&mut self) -> Result<(), SocError> {
        use crate::mmio::delay;
        use userlib::println;

        // Read initial status to get port counts
        let sts1 = self.ippc_read32(ippc::IP_PW_STS1)?;
        println!("  IPPC STS1: 0x{:08x}", sts1);

        // Extract port counts (cap to reasonable values)
        let mut usb3_raw = ((sts1 & ippc::sts::U3_PORT_NUM_MASK) >> ippc::sts::U3_PORT_NUM_SHIFT) as u8;
        let mut usb2_raw = (sts1 & ippc::sts::U2_PORT_NUM_MASK) as u8;
        println!("  Port counts: USB3={}, USB2={}", usb3_raw, usb2_raw);
        if usb3_raw > 4 { usb3_raw = 1; }
        if usb2_raw > 4 { usb2_raw = 1; }
        self.usb3_ports = usb3_raw;
        self.usb2_ports = usb2_raw;

        // Clear power down and reset bits
        let ctrl0 = self.ippc_read32(ippc::IP_PW_CTRL0)?;
        let new_ctrl0 = ctrl0 & !(ippc::ctrl::IP_SW_RST | ippc::ctrl::REF_CK_GATE);
        self.ippc_write32(ippc::IP_PW_CTRL0, new_ctrl0)?;

        // Wait for clocks to stabilize
        delay(10000);

        // Port control bits
        const PORT_DIS: u32 = 1 << 0;
        const PORT_PDN: u32 = 1 << 1;
        const PORT_HOST_SEL: u32 = 1 << 2;

        // Enable USB3 ports: clear DIS/PDN, set HOST_SEL
        for p in 0..self.usb3_ports {
            let offset = ippc::U3_CTRL_0P + (p as usize * 0x08);
            let ctrl = self.ippc_read32(offset)?;
            let new_ctrl = (ctrl & !(PORT_DIS | PORT_PDN)) | PORT_HOST_SEL;
            self.ippc_write32(offset, new_ctrl)?;
            println!("  U3_CTRL[{}]: 0x{:08x} -> 0x{:08x}", p, ctrl, new_ctrl);
        }

        // Enable USB2 ports: clear DIS/PDN, set HOST_SEL
        for p in 0..self.usb2_ports {
            let offset = ippc::U2_CTRL_0P + (p as usize * 0x08);
            let ctrl = self.ippc_read32(offset)?;
            let new_ctrl = (ctrl & !(PORT_DIS | PORT_PDN)) | PORT_HOST_SEL;
            self.ippc_write32(offset, new_ctrl)?;
            println!("  U2_CTRL[{}]: 0x{:08x} -> 0x{:08x}", p, ctrl, new_ctrl);
        }

        // Wait for PHY power-up
        delay(10000);

        Ok(())
    }
}

impl SocUsb for Mt7988aSoc {
    fn xhci_base(&self) -> u64 {
        self.mac_base()
    }

    fn xhci_size(&self) -> usize {
        addrs::MAC_SIZE
    }

    fn irq_number(&self) -> u32 {
        match self.id {
            ControllerId::Ssusb0 => addrs::SSUSB0_IRQ,
            ControllerId::Ssusb1 => addrs::SSUSB1_IRQ,
        }
    }

    fn enable_clocks(&mut self) -> Result<(), SocError> {
        // Clocks are enabled via IPPC in pre_init
        Ok(())
    }

    fn deassert_reset(&mut self) -> Result<(), SocError> {
        // Reset is de-asserted via IPPC in pre_init
        Ok(())
    }

    fn pre_init(&mut self) -> Result<(), SocError> {
        // IPPC initialization must happen before xHCI access
        self.ippc_init()
    }

    fn post_init(&mut self) -> Result<(), SocError> {
        // No post-init needed for MT7988A
        Ok(())
    }

    fn port_count(&self) -> PortCount {
        PortCount {
            usb3: self.usb3_ports,
            usb2: self.usb2_ports,
        }
    }

    fn force_usb2_phy_power(&mut self) -> Result<(), SocError> {
        use crate::mmio::delay;

        let pll = self.ippc_read32(ippc::U2_PHY_PLL)?;
        // Set bit 0 to force PHY power on
        self.ippc_write32(ippc::U2_PHY_PLL, pll | 1)?;
        delay(1000);
        Ok(())
    }
}
