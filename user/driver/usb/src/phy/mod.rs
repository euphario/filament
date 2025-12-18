//! USB PHY Driver Abstraction
//!
//! This module defines the `PhyDriver` trait for USB PHY initialization.
//!
//! ## Why PHY is Board-Specific
//!
//! The same SoC on different boards may have different PHY configurations:
//! - Different GPIO pins for VBUS enable
//! - Different external PHY chips (TUSB1310, etc.)
//! - Different power sequencing requirements
//! - Different tuning parameters
//!
//! ## Implementations
//!
//! - `mt7988a_tphy` - MediaTek T-PHY v2 (used on BPI-R4 SSUSB1)
//! - `mt7988a_xsphy` - MediaTek XS-PHY (used on BPI-R4 SSUSB0)
//! - `none` - No-op for platforms that don't need PHY configuration

mod mt7988a;

pub use mt7988a::{Mt7988aTphy, Mt7988aXsphy};

/// PHY initialization error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhyError {
    /// PHY hardware not responding
    NotResponding,
    /// PHY initialization timeout
    Timeout,
    /// Invalid configuration
    InvalidConfig,
    /// MMIO region not set
    NoMmio,
}

/// PHY status information
#[derive(Debug, Clone, Copy)]
pub struct PhyStatus {
    /// PHY is powered on
    pub powered: bool,
    /// PHY is in host mode (vs device/OTG)
    pub host_mode: bool,
    /// PHY link is up
    pub link_up: bool,
}

impl Default for PhyStatus {
    fn default() -> Self {
        Self {
            powered: false,
            host_mode: false,
            link_up: false,
        }
    }
}

/// USB PHY Driver trait
///
/// Implement this trait for each PHY type to provide initialization
/// and configuration for the electrical layer.
///
/// ## Responsibilities
/// - Configure PHY registers for USB operation
/// - Set host mode (vs device/OTG mode)
/// - Power on/off the PHY
/// - Apply tuning parameters
///
/// ## What does NOT belong here
/// - xHCI operations (that's in the `xhci` module)
/// - SoC wrapper operations (IPPC, etc. - that's in `soc` module)
/// - GPIO control for VBUS (that's board-specific)
pub trait PhyDriver {
    /// Initialize the PHY hardware
    ///
    /// This is called early in the USB initialization sequence.
    /// May be called before or after xHCI reset depending on hardware.
    fn init(&mut self) -> Result<(), PhyError>;

    /// Configure PHY for host mode
    ///
    /// Sets up the PHY for host operation (vs device/OTG mode).
    /// This typically involves setting control registers to enable
    /// VBUS sensing, session management, etc.
    fn set_host_mode(&mut self) -> Result<(), PhyError>;

    /// Power on the PHY
    ///
    /// Enables the PHY's internal power and clocks.
    fn power_on(&mut self) -> Result<(), PhyError>;

    /// Power off the PHY
    ///
    /// Disables the PHY to save power.
    fn power_off(&mut self) -> Result<(), PhyError>;

    /// Get current PHY status
    fn status(&self) -> PhyStatus;

    /// Apply tuning parameters (optional)
    ///
    /// Override to apply board-specific tuning like:
    /// - Eye diagram optimization
    /// - Impedance tuning
    /// - Signal quality adjustments
    fn apply_tuning(&mut self) -> Result<(), PhyError> {
        Ok(())
    }
}

/// No-op PHY driver for platforms that don't need PHY configuration
///
/// Use this when:
/// - PHY is handled by firmware/bootloader
/// - Platform uses external PHY with its own driver
/// - Testing without real hardware
pub struct NoPhy;

impl PhyDriver for NoPhy {
    fn init(&mut self) -> Result<(), PhyError> {
        Ok(())
    }

    fn set_host_mode(&mut self) -> Result<(), PhyError> {
        Ok(())
    }

    fn power_on(&mut self) -> Result<(), PhyError> {
        Ok(())
    }

    fn power_off(&mut self) -> Result<(), PhyError> {
        Ok(())
    }

    fn status(&self) -> PhyStatus {
        PhyStatus {
            powered: true,
            host_mode: true,
            link_up: true,
        }
    }
}
