//! Hardware Abstraction Layer (HAL) for USB Controllers
//!
//! This module defines traits that abstract SoC-specific operations.
//! Implementations allow the USB driver to work across different ARM SoCs.
//!
//! The xHCI register interface itself is standardized, so most of the
//! driver is portable. Only these SoC-specific parts need adaptation:
//! - PHY initialization
//! - Clock and reset control
//! - Base addresses
//! - Vendor quirks

use crate::mmio::MmioRegion;

/// SoC-specific hardware operations
///
/// Implement this trait for each supported SoC to provide:
/// - PHY initialization sequences
/// - Clock/reset control
/// - Base addresses and configuration
pub trait SocHal {
    /// Get the xHCI MAC base address for this controller
    fn mac_base(&self) -> usize;

    /// Get the PHY base address for this controller
    fn phy_base(&self) -> usize;

    /// Get the PHY region size for this controller
    fn phy_size(&self) -> usize;

    /// Get the IRQ number for this controller
    fn irq_number(&self) -> u32;

    /// Initialize clocks for the USB controller
    /// Called before any other initialization
    fn clock_enable(&mut self) -> bool;

    /// De-assert resets for the USB controller
    fn reset_deassert(&mut self) -> bool;

    /// Initialize the PHY (T-PHY, XS-PHY, DWC3, etc.)
    /// This is highly SoC-specific
    fn phy_init(&mut self) -> bool;

    /// Perform any vendor-specific quirks or workarounds
    fn apply_quirks(&mut self);

    /// Get the IPPC offset from MAC base (MediaTek specific)
    /// Returns None if this SoC doesn't have IPPC
    fn ippc_offset(&self) -> Option<usize> {
        None
    }

    /// Check if this is a USB3-capable controller
    fn is_usb3(&self) -> bool {
        true
    }
}

/// Controller instance identification
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ControllerId {
    pub index: u32,
    pub name: &'static str,
}

/// xHCI controller abstraction
///
/// This struct wraps the MMIO regions and provides a common interface
/// for xHCI operations regardless of SoC.
pub struct XhciController {
    /// MMIO region for MAC (xHCI) registers
    pub mac: MmioRegion,
    /// MMIO region for PHY registers (optional for some SoCs)
    pub phy: Option<MmioRegion>,
    /// Controller identification
    pub id: ControllerId,
    /// xHCI capability length
    pub caplength: u32,
    /// Operational register base offset
    pub op_base: usize,
    /// Runtime register base offset
    pub rt_base: usize,
    /// Doorbell register base offset
    pub db_base: usize,
    /// Port register base offset
    pub port_base: usize,
    /// Number of ports
    pub num_ports: u32,
    /// Maximum slots supported
    pub max_slots: u32,
    /// IPPC offset (MediaTek specific)
    pub ippc_offset: Option<usize>,
}

impl XhciController {
    /// Read from IPPC registers (MediaTek specific)
    /// Returns 0 if IPPC is not available
    #[inline]
    pub fn ippc_read32(&self, offset: usize) -> u32 {
        match self.ippc_offset {
            Some(ippc) => self.mac.read32(ippc + offset),
            None => 0,
        }
    }

    /// Write to IPPC registers (MediaTek specific)
    /// No-op if IPPC is not available
    #[inline]
    pub fn ippc_write32(&self, offset: usize, value: u32) {
        if let Some(ippc) = self.ippc_offset {
            self.mac.write32(ippc + offset, value);
        }
    }

    /// Configure T-PHY for host mode (MediaTek specific)
    ///
    /// This must be called AFTER xHCI reset.
    /// Sets host mode bits in the T-PHY DTM1 register.
    pub fn configure_tphy_host_mode(&self) {
        use crate::phy_regs::tphy;
        use crate::mmio::{delay, print_hex32};
        use userlib::{print, println};

        let phy = match &self.phy {
            Some(p) => p,
            None => return,
        };

        println!();
        println!("=== PHY Initialization ===");

        let phy_name = if self.id.index == 0 { "XFI T-PHY" } else { "T-PHY v2" };
        println!("  Configuring {} for host mode...", phy_name);

        // T-PHY v2 has USB2 PHY at COM bank offset 0x300
        let offsets_to_try: &[(usize, &str)] = &[
            (0x300, "COM+0x300"),
            (0x000, "base+0x000"),
            (0x100, "base+0x100"),
            (0x400, "base+0x400"),
        ];

        let mut found_offset: Option<usize> = None;
        for &(base_off, name) in offsets_to_try {
            let dtm1_off = base_off + 0x6C;
            let dtm1 = phy.read32(dtm1_off);
            if dtm1 != 0 || base_off == 0x300 {
                print!("    {} DTM1@0x{:x}=0x", name, dtm1_off);
                print_hex32(dtm1);
                println!();
                if found_offset.is_none() {
                    found_offset = Some(base_off);
                }
            }
        }

        let com_offset = found_offset.unwrap_or(0x300);
        let dtm1_offset = com_offset + 0x6C;

        let dtm1 = phy.read32(dtm1_offset);
        print!("    Using offset 0x{:x}, DTM1 before: 0x", dtm1_offset);
        print_hex32(dtm1);
        println!();

        // Set host mode bits
        let new_dtm1 = dtm1 | tphy::HOST_MODE;
        phy.write32(dtm1_offset, new_dtm1);

        delay(1000);

        let dtm1_after = phy.read32(dtm1_offset);
        print!("    After:  DTM1=0x");
        print_hex32(dtm1_after);
        if dtm1_after != dtm1 {
            println!(" (host mode configured)");
        } else {
            println!(" (WARNING: register unchanged!)");
        }
    }

    /// Force USB2 PHY power on (MediaTek specific)
    ///
    /// This must be called AFTER xHCI reset for proper device detection.
    /// Sets bit 0 of U2_PHY_PLL IPPC register.
    pub fn force_usb2_phy_power(&self) {
        use crate::phy_regs::ippc;
        use crate::mmio::{delay, print_hex32};
        use userlib::{print, println};

        if self.ippc_offset.is_none() {
            return;
        }

        println!();
        println!("=== Forcing USB2 PHY Power ===");
        let pll = self.ippc_read32(ippc::U2_PHY_PLL);
        print!("  U2_PHY_PLL before: 0x");
        print_hex32(pll);
        println!();
        self.ippc_write32(ippc::U2_PHY_PLL, pll | 1);  // Set bit 0 to force power on
        delay(1000);
        let pll_after = self.ippc_read32(ippc::U2_PHY_PLL);
        print!("  U2_PHY_PLL after:  0x");
        print_hex32(pll_after);
        println!(" (PHY power forced on)");
    }

    /// Read operational register
    #[inline]
    pub fn op_read32(&self, offset: usize) -> u32 {
        self.mac.read32(self.op_base + offset)
    }

    /// Write operational register
    #[inline]
    pub fn op_write32(&self, offset: usize, value: u32) {
        self.mac.write32(self.op_base + offset, value);
    }

    /// Write 64-bit operational register
    #[inline]
    pub fn op_write64(&self, offset: usize, value: u64) {
        self.mac.write64(self.op_base + offset, value);
    }

    /// Read runtime register
    #[inline]
    pub fn rt_read32(&self, offset: usize) -> u32 {
        self.mac.read32(self.rt_base + offset)
    }

    /// Read 64-bit runtime register
    #[inline]
    pub fn rt_read64(&self, offset: usize) -> u64 {
        self.mac.read64(self.rt_base + offset)
    }

    /// Write runtime register
    #[inline]
    pub fn rt_write32(&self, offset: usize, value: u32) {
        self.mac.write32(self.rt_base + offset, value);
    }

    /// Write 64-bit runtime register
    #[inline]
    pub fn rt_write64(&self, offset: usize, value: u64) {
        self.mac.write64(self.rt_base + offset, value);
    }

    /// Ring doorbell with proper memory barriers
    ///
    /// Includes memory barrier and PCIe serialization to ensure DMA writes
    /// are visible before the doorbell is rung.
    #[inline]
    pub fn ring_doorbell(&self, slot: u32, target: u32) {
        // Data synchronization barrier
        unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)) };

        // Force PCIe write serialization by reading from device
        let _ = self.mac.read32(self.db_base);

        // Write the doorbell
        self.mac.write32(self.db_base + (slot as usize * 4), target);
    }

    /// Read port status register (PORTSC)
    #[inline]
    pub fn read_portsc(&self, port: u32) -> u32 {
        if port == 0 || port > self.num_ports {
            return 0;
        }
        let offset = self.port_base + ((port - 1) as usize * 0x10);
        self.mac.read32(offset)
    }

    /// Write port status register (PORTSC)
    #[inline]
    pub fn write_portsc(&self, port: u32, value: u32) {
        if port == 0 || port > self.num_ports {
            return;
        }
        let offset = self.port_base + ((port - 1) as usize * 0x10);
        self.mac.write32(offset, value);
    }
}

// =============================================================================
// MediaTek MT7988A HAL Implementation
// =============================================================================

/// MT7988A SSUSB controller configuration and HAL
pub mod mt7988a {
    use super::{SocHal, XhciController, ControllerId, MmioRegion};
    use crate::phy_regs::ippc;
    use crate::mmio::{delay, print_hex32};
    use userlib::{print, println};

    /// Controller indices for MT7988A
    pub const SSUSB0: u32 = 0;  // XS-PHY controller (XFI T-PHY)
    pub const SSUSB1: u32 = 1;  // T-PHY v2 controller

    /// IPPC offset from MAC base
    pub const IPPC_OFFSET: usize = 0x3E00;

    /// Default MAC region size
    pub const MAC_SIZE: usize = 0x4000;

    /// MT7988A physical addresses
    pub const SSUSB0_MAC_PHYS: u64 = 0x1119_0000;
    pub const SSUSB0_PHY_PHYS: u64 = 0x11f2_0000;  // XFI T-PHY
    pub const SSUSB0_IRQ: u32 = 173 + 32;  // GIC SPI 173

    pub const SSUSB1_MAC_PHYS: u64 = 0x1120_0000;
    pub const SSUSB1_PHY_PHYS: u64 = 0x11c5_0000;  // T-PHY v2
    pub const SSUSB1_IRQ: u32 = 172 + 32;  // GIC SPI 172

    pub const SSUSB0_PHY_SIZE: usize = 0x10000;  // XFI T-PHY is 64KB
    pub const SSUSB1_PHY_SIZE: usize = 0x1000;   // T-PHY v2 is 4KB

    /// MT7988A SSUSB HAL implementation
    ///
    /// Handles MediaTek-specific IPPC and PHY initialization.
    pub struct Mt7988aHal {
        controller_id: u32,
        mac: Option<MmioRegion>,
        phy: Option<MmioRegion>,
    }

    impl Mt7988aHal {
        /// Create a new MT7988A HAL for the specified controller
        pub fn new(controller_id: u32) -> Self {
            Self {
                controller_id,
                mac: None,
                phy: None,
            }
        }

        /// Set MAC MMIO region (call after mapping)
        pub fn set_mac(&mut self, mac: MmioRegion) {
            self.mac = Some(mac);
        }

        /// Set PHY MMIO region (call after mapping)
        pub fn set_phy(&mut self, phy: MmioRegion) {
            self.phy = Some(phy);
        }

        /// Get MAC region reference
        pub fn mac(&self) -> Option<&MmioRegion> {
            self.mac.as_ref()
        }

        /// Get PHY region reference
        pub fn phy(&self) -> Option<&MmioRegion> {
            self.phy.as_ref()
        }

        /// Read IPPC register
        fn ippc_read32(&self, offset: usize) -> u32 {
            self.mac.as_ref().map_or(0, |m| m.read32(IPPC_OFFSET + offset))
        }

        /// Write IPPC register
        fn ippc_write32(&self, offset: usize, value: u32) {
            if let Some(m) = &self.mac {
                m.write32(IPPC_OFFSET + offset, value);
            }
        }

        /// Initialize IPPC (IP Port Control) - MediaTek specific
        ///
        /// This handles:
        /// - Software reset
        /// - Host power on / device power down
        /// - Port configuration (U3/U2)
        /// - Clock stabilization wait
        ///
        /// Returns (u3_ports, u2_ports) on success, None on failure
        pub fn ippc_init(&mut self) -> Option<(u32, u32)> {
            println!();
            println!("=== IPPC Initialization ===");

            // Software reset
            println!("  Software reset...");
            self.ippc_write32(ippc::IP_PW_CTRL0, ippc::IP_SW_RST);
            delay(1000);
            self.ippc_write32(ippc::IP_PW_CTRL0, 0);
            delay(1000);

            // Power on host, power down device
            println!("  Host power on...");
            self.ippc_write32(ippc::IP_PW_CTRL2, ippc::IP_DEV_PDN);
            self.ippc_write32(ippc::IP_PW_CTRL1, 0); // Clear host PDN

            // Read port counts
            let cap = self.ippc_read32(ippc::IP_XHCI_CAP);
            let u3_ports = (cap >> 8) & 0xF;
            let u2_ports = cap & 0xF;
            println!("  Ports: {} USB3, {} USB2", u3_ports, u2_ports);

            // Configure U3 ports (if any)
            for i in 0..u3_ports {
                let offset = ippc::U3_CTRL_P0 + (i as usize * 8);
                let mut ctrl = self.ippc_read32(offset);
                ctrl &= !(ippc::PORT_PDN | ippc::PORT_DIS);
                ctrl |= ippc::PORT_HOST_SEL;
                self.ippc_write32(offset, ctrl);
            }

            // Configure U2 ports
            for i in 0..u2_ports {
                let offset = ippc::U2_CTRL_P0 + (i as usize * 8);
                let mut ctrl = self.ippc_read32(offset);
                ctrl &= !(ippc::PORT_PDN | ippc::PORT_DIS);
                ctrl |= ippc::PORT_HOST_SEL;
                self.ippc_write32(offset, ctrl);
            }

            // Wait for clocks
            println!("  Waiting for clocks...");
            let required = ippc::SYSPLL_STABLE | ippc::REF_RST | ippc::SYS125_RST | ippc::XHCI_RST;
            for i in 0..1000 {
                let sts = self.ippc_read32(ippc::IP_PW_STS1);
                if (sts & required) == required {
                    println!("  Clocks stable (after {} iterations)", i);
                    return Some((u3_ports, u2_ports));
                }
                if i == 999 {
                    print!("  Clock timeout! IP_PW_STS1=0x");
                    print_hex32(sts);
                    println!();
                    return None;
                }
                delay(1000);
            }

            Some((u3_ports, u2_ports))
        }

        /// Create an XhciController from this HAL (after init)
        ///
        /// Call this after IPPC and PHY initialization to get an XhciController
        /// that can be used for generic xHCI operations.
        pub fn into_controller(self) -> Option<XhciController> {
            let mac = self.mac?;
            Some(XhciController {
                mac,
                phy: self.phy,
                id: ControllerId {
                    index: self.controller_id,
                    name: if self.controller_id == 0 { "SSUSB0" } else { "SSUSB1" },
                },
                caplength: 0,
                op_base: 0,
                rt_base: 0,
                db_base: 0,
                port_base: 0,
                num_ports: 0,
                max_slots: 0,
                ippc_offset: Some(IPPC_OFFSET),
            })
        }
    }

    impl SocHal for Mt7988aHal {
        fn mac_base(&self) -> usize {
            match self.controller_id {
                0 => SSUSB0_MAC_PHYS as usize,
                _ => SSUSB1_MAC_PHYS as usize,
            }
        }

        fn phy_base(&self) -> usize {
            match self.controller_id {
                0 => SSUSB0_PHY_PHYS as usize,
                _ => SSUSB1_PHY_PHYS as usize,
            }
        }

        fn phy_size(&self) -> usize {
            match self.controller_id {
                0 => SSUSB0_PHY_SIZE,
                _ => SSUSB1_PHY_SIZE,
            }
        }

        fn irq_number(&self) -> u32 {
            match self.controller_id {
                0 => SSUSB0_IRQ,
                _ => SSUSB1_IRQ,
            }
        }

        fn clock_enable(&mut self) -> bool {
            // MT7988A clocks are typically enabled by the bootloader
            // If needed, this would use INFRACFG_AO registers
            true
        }

        fn reset_deassert(&mut self) -> bool {
            // MT7988A resets are typically handled by bootloader
            // The IPPC software reset is done in ippc_init()
            true
        }

        fn phy_init(&mut self) -> bool {
            // Only IPPC init here - this must complete before xHCI can be accessed
            // T-PHY host mode and USB2 PHY power are done AFTER xHCI reset
            self.ippc_init().is_some()
        }

        fn apply_quirks(&mut self) {
            // No additional quirks needed for MT7988A currently
        }

        fn ippc_offset(&self) -> Option<usize> {
            Some(IPPC_OFFSET)
        }

        fn is_usb3(&self) -> bool {
            true
        }
    }
}
