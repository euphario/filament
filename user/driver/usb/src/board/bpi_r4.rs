//! Banana Pi BPI-R4 Board Configuration
//!
//! The BPI-R4 is based on the MediaTek MT7988A SoC with the following
//! USB configuration:
//!
//! ## SSUSB0 (XFI T-PHY)
//! - MAC: 0x11190000
//! - PHY: 0x11E10000 (XFI T-PHY, combo PHY)
//! - Connected to: M.2 Key B slot (for WiFi cards)
//! - Can be used for USB or PCIe depending on M.2 card
//!
//! ## SSUSB1 (T-PHY v2)
//! - MAC: 0x11200000
//! - PHY: 0x11C50000 (T-PHY v2: USB2 PHY at +0, USB3 PHY at +0x700)
//! - Connected to: VL822 USB 3.0 hub
//!   - 4x USB 3.0 Type-A ports on the board
//!
//! ## VBUS Control
//! - SSUSB0: Controlled by M.2 slot power (not direct GPIO)
//! - SSUSB1: VL822 hub manages downstream VBUS

use super::{Board, BoardError, UsbControllerConfig};
use crate::soc::{Mt7988aSoc, Mt7988aControllerId as ControllerId};
use crate::phy::Mt7988aTphy;
use userlib::println;

/// USB controller configurations for BPI-R4
const BPI_R4_USB_CONTROLLERS: &[UsbControllerConfig] = &[
    // SSUSB0 - M.2 slot (usually WiFi, not commonly used for USB)
    UsbControllerConfig {
        name: "SSUSB0 (M.2)",
        index: 0,
        mac_base: 0x11190000,
        mac_size: 0x4000,
        phy_base: 0x11E10000,
        phy_size: 0x10000,
        irq: 243,
    },
    // SSUSB1 - Main USB ports via VL822 hub
    UsbControllerConfig {
        name: "SSUSB1 (USB-A)",
        index: 1,
        mac_base: 0x11200000,
        mac_size: 0x4000,
        phy_base: 0x11C50000,  // T-PHY v2: USB2 at +0 (0x700), USB3 at +0x700 (0x900)
        phy_size: 0x1000,      // USB2 + USB3 = 0x700 + 0x900 = 0x1000
        irq: 244,
    },
];

/// Banana Pi BPI-R4 board configuration
pub struct BpiR4 {
    /// Currently selected controller (for single-controller init)
    default_controller: u8,
}

impl BpiR4 {
    /// Create a new BPI-R4 board configuration
    ///
    /// Defaults to SSUSB1 (the USB-A ports) which is the most common use case.
    pub fn new() -> Self {
        Self {
            default_controller: 1,  // SSUSB1 by default
        }
    }

    /// Create configuration for SSUSB0 (M.2 slot)
    pub fn ssusb0() -> Self {
        Self {
            default_controller: 0,
        }
    }

    /// Create configuration for SSUSB1 (USB-A ports)
    pub fn ssusb1() -> Self {
        Self {
            default_controller: 1,
        }
    }

    /// Get the default controller index
    pub fn default_controller(&self) -> u8 {
        self.default_controller
    }

    /// Get controller config by index
    pub fn controller_config(&self, index: u8) -> Option<&UsbControllerConfig> {
        BPI_R4_USB_CONTROLLERS.get(index as usize)
    }
}

impl Default for BpiR4 {
    fn default() -> Self {
        Self::new()
    }
}

impl Board for BpiR4 {
    type Soc = Mt7988aSoc;
    type Phy = Mt7988aTphy;

    fn name(&self) -> &'static str {
        "Banana Pi BPI-R4"
    }

    fn usb_controllers(&self) -> &[UsbControllerConfig] {
        BPI_R4_USB_CONTROLLERS
    }

    fn create_soc(&self, index: u8) -> Option<Self::Soc> {
        match index {
            0 => Some(Mt7988aSoc::new(ControllerId::Ssusb0)),
            1 => Some(Mt7988aSoc::new(ControllerId::Ssusb1)),
            _ => None,
        }
    }

    fn create_phy(&self, index: u8) -> Option<Self::Phy> {
        match index {
            0 | 1 => Some(Mt7988aTphy::new()),
            _ => None,
        }
    }

    fn pre_init(&mut self) -> Result<(), BoardError> {
        println!("=== {} Board Init ===", self.name());
        // No special pre-init needed for BPI-R4
        // Power rails are managed by the board's power management
        Ok(())
    }

    fn enable_vbus(&mut self, index: u8) -> Result<(), BoardError> {
        match index {
            0 => {
                // SSUSB0 (M.2): Power managed by M.2 slot
                println!("  SSUSB0: M.2 slot power (managed by slot)");
            }
            1 => {
                // SSUSB1: VL822 hub manages downstream VBUS
                println!("  SSUSB1: VBUS via VL822 hub");
            }
            _ => return Err(BoardError::InvalidController),
        }
        Ok(())
    }

    fn disable_vbus(&mut self, _index: u8) -> Result<(), BoardError> {
        // VBUS disable not implemented for BPI-R4
        // The VL822 hub manages power for downstream ports
        Ok(())
    }
}
