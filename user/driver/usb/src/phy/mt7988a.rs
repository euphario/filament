//! MediaTek MT7988A USB PHY Drivers
//!
//! The MT7988A has two USB host controllers with different PHYs:
//!
//! ## SSUSB1 - T-PHY v2
//! - USB 2.0 + USB 3.0
//! - PHY base: 0x11C50000 (USB2 at +0, USB3 at +0x700)
//! - USB2 PHY at offset 0x0 within the region
//! - DTM1 register at offset 0x6C controls host mode
//!
//! ## SSUSB0 - XFI T-PHY (XS-PHY)
//! - Combo PHY (USB 3.0 / PCIe / XFI)
//! - PHY base: 0x11E10000
//! - Different register layout

use super::{PhyDriver, PhyError, PhyStatus};
use crate::mmio::MmioRegion;
use userlib::{print, println};

/// T-PHY v2 register definitions
mod tphy_regs {
    /// DTM1 register offset (U3P_U2PHYDTM1)
    /// Controls host/device mode via VBUS and A-valid signals
    pub const DTM1: usize = 0x6C;

    /// Host mode bits for DTM1:
    /// - FORCE_VBUSVALID (bit 5): Force VBUS valid signal
    /// - FORCE_AVALID (bit 4): Force A-valid signal
    /// - RG_VBUSVALID (bit 1): Set VBUS valid state
    /// - RG_AVALID (bit 0): Set A-valid state
    pub const FORCE_VBUSVALID: u32 = 1 << 5;
    pub const FORCE_AVALID: u32 = 1 << 4;
    pub const RG_VBUSVALID: u32 = 1 << 1;
    pub const RG_AVALID: u32 = 1 << 0;
    pub const HOST_MODE: u32 = FORCE_VBUSVALID | FORCE_AVALID | RG_VBUSVALID | RG_AVALID;  // 0x33
}

/// MediaTek T-PHY v2 Driver (for SSUSB1)
///
/// This PHY is used on the MT7988A SSUSB1 controller which connects
/// to the USB-A ports via the VL822 hub on the BPI-R4.
pub struct Mt7988aTphy {
    mmio: Option<MmioRegion>,
    powered: bool,
    host_mode: bool,
}

impl Mt7988aTphy {
    /// Create a new T-PHY driver
    pub fn new() -> Self {
        Self {
            mmio: None,
            powered: false,
            host_mode: false,
        }
    }

    /// Set the MMIO region for PHY registers
    pub fn set_mmio(&mut self, mmio: MmioRegion) {
        self.mmio = Some(mmio);
    }

    /// Get PHY MMIO region (for diagnostics)
    pub fn mmio(&self) -> Option<&MmioRegion> {
        self.mmio.as_ref()
    }

    /// Read a PHY register
    fn read32(&self, offset: usize) -> Result<u32, PhyError> {
        self.mmio.as_ref()
            .map(|m| m.read32(offset))
            .ok_or(PhyError::NoMmio)
    }

    /// Write a PHY register
    fn write32(&self, offset: usize, value: u32) -> Result<(), PhyError> {
        self.mmio.as_ref()
            .map(|m| m.write32(offset, value))
            .ok_or(PhyError::NoMmio)
    }
}

impl Default for Mt7988aTphy {
    fn default() -> Self {
        Self::new()
    }
}

impl PhyDriver for Mt7988aTphy {
    fn init(&mut self) -> Result<(), PhyError> {
        // T-PHY doesn't need explicit init - it's ready after reset
        // The main configuration is done in set_host_mode()
        println!("  T-PHY v2: initialized");
        Ok(())
    }

    fn set_host_mode(&mut self) -> Result<(), PhyError> {
        use crate::mmio::{delay_ms, print_hex32};

        println!();
        println!("=== T-PHY v2 Host Mode Configuration ===");

        // DTM1 is at offset 0x6C from USB2 PHY base (0x11c50000)
        // Set host mode: force VBUS valid and A-valid signals
        let dtm1 = self.read32(tphy_regs::DTM1)?;
        print!("    DTM1@0x{:x} before: 0x", tphy_regs::DTM1);
        print_hex32(dtm1);
        println!();

        // Set host mode bits (0x33):
        // - FORCE_VBUSVALID (bit 5) + RG_VBUSVALID (bit 1)
        // - FORCE_AVALID (bit 4) + RG_AVALID (bit 0)
        let new_dtm1 = dtm1 | tphy_regs::HOST_MODE;
        self.write32(tphy_regs::DTM1, new_dtm1)?;

        delay_ms(1);  // Wait for register to take effect

        let dtm1_after = self.read32(tphy_regs::DTM1)?;
        print!("    DTM1 after: 0x");
        print_hex32(dtm1_after);
        if dtm1_after != dtm1 {
            println!(" (host mode configured)");
        } else {
            println!(" (WARNING: register unchanged!)");
        }

        self.host_mode = true;
        Ok(())
    }

    fn power_on(&mut self) -> Result<(), PhyError> {
        // T-PHY power is controlled via IPPC, not here
        // This is handled by the SoC wrapper
        self.powered = true;
        Ok(())
    }

    fn power_off(&mut self) -> Result<(), PhyError> {
        self.powered = false;
        Ok(())
    }

    fn status(&self) -> PhyStatus {
        PhyStatus {
            powered: self.powered,
            host_mode: self.host_mode,
            link_up: self.powered && self.host_mode,
        }
    }
}

/// MediaTek XS-PHY Driver (for SSUSB0)
///
/// This is the XFI T-PHY used on SSUSB0, which is a combo PHY
/// supporting USB 3.0, PCIe, and XFI modes.
///
/// On the BPI-R4, SSUSB0 connects to the M.2 slot for WiFi cards.
pub struct Mt7988aXsphy {
    mmio: Option<MmioRegion>,
    powered: bool,
    host_mode: bool,
}

impl Mt7988aXsphy {
    /// Create a new XS-PHY driver
    pub fn new() -> Self {
        Self {
            mmio: None,
            powered: false,
            host_mode: false,
        }
    }

    /// Set the MMIO region for PHY registers
    pub fn set_mmio(&mut self, mmio: MmioRegion) {
        self.mmio = Some(mmio);
    }
}

impl Default for Mt7988aXsphy {
    fn default() -> Self {
        Self::new()
    }
}

impl PhyDriver for Mt7988aXsphy {
    fn init(&mut self) -> Result<(), PhyError> {
        println!("  XFI T-PHY (XS-PHY): initialized");
        Ok(())
    }

    fn set_host_mode(&mut self) -> Result<(), PhyError> {
        // XS-PHY configuration is more complex due to combo nature
        // For now, assume bootloader configured it
        println!("  XS-PHY: host mode (using bootloader config)");
        self.host_mode = true;
        Ok(())
    }

    fn power_on(&mut self) -> Result<(), PhyError> {
        self.powered = true;
        Ok(())
    }

    fn power_off(&mut self) -> Result<(), PhyError> {
        self.powered = false;
        Ok(())
    }

    fn status(&self) -> PhyStatus {
        PhyStatus {
            powered: self.powered,
            host_mode: self.host_mode,
            link_up: self.powered && self.host_mode,
        }
    }
}
