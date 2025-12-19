//! SoC USB IP Wrapper Abstraction
//!
//! This module defines the `SocUsb` trait for SoC-specific USB IP wrappers.
//!
//! ## Why SoC Wrappers Exist
//!
//! Most SoCs wrap the standard xHCI controller with vendor-specific logic:
//! - **MediaTek**: IPPC (IP Port Control) for power/clock management
//! - **Synopsys DWC3**: GCTL, GUSB2PHYCFG, GUSB3PIPECTL registers
//! - **Intel/AMD**: Platform-specific power management
//!
//! The xHCI controller itself is standard, but accessing it requires
//! going through these wrapper layers first.
//!
//! ## Responsibilities
//! - Enable clocks and power
//! - De-assert resets
//! - Configure port counts
//! - Handle vendor-specific quirks
//!
//! ## What does NOT belong here
//! - PHY initialization (that's in `phy` module)
//! - Standard xHCI operations (that's in `xhci` module)
//! - Board-specific GPIO/power (that's in `board` module)

pub mod mt7988a;

pub use mt7988a::{Mt7988aSoc, ControllerId as Mt7988aControllerId};

/// SoC wrapper initialization error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SocError {
    /// Clock enable failed
    ClockFailed,
    /// Reset de-assert failed
    ResetFailed,
    /// Power on failed
    PowerFailed,
    /// Timeout waiting for hardware
    Timeout,
    /// Invalid controller ID
    InvalidController,
    /// MMIO region not set
    NoMmio,
}

/// USB port count information
#[derive(Debug, Clone, Copy)]
pub struct PortCount {
    /// Number of USB 3.0 ports
    pub usb3: u8,
    /// Number of USB 2.0 ports
    pub usb2: u8,
}

/// SoC USB IP Wrapper trait
///
/// Implement this trait for each SoC to handle vendor-specific
/// USB controller initialization.
///
/// ## Responsibilities
/// - Enable clocks and power for USB
/// - De-assert USB reset signals
/// - Read port count from SoC registers
/// - Handle any pre/post xHCI init quirks
///
/// ## What does NOT belong here
/// - PHY configuration (use `PhyDriver` trait)
/// - xHCI register access (use `xhci::Controller`)
/// - Board-specific GPIO (use `Board` trait)
pub trait SocUsb {
    /// Get the xHCI MMIO base address
    fn xhci_base(&self) -> u64;

    /// Get the xHCI MMIO region size
    fn xhci_size(&self) -> usize;

    /// Get the IRQ number for this USB controller
    fn irq_number(&self) -> u32;

    /// Enable clocks for USB controller
    fn enable_clocks(&mut self) -> Result<(), SocError>;

    /// De-assert reset for USB controller
    fn deassert_reset(&mut self) -> Result<(), SocError>;

    /// Pre-xHCI initialization
    ///
    /// Called before xHCI controller is accessed.
    /// Use for vendor-specific power-on sequences.
    fn pre_init(&mut self) -> Result<(), SocError>;

    /// Post-xHCI initialization
    ///
    /// Called after xHCI reset but before controller start.
    /// Use for quirks that need xHCI to be reset first.
    fn post_init(&mut self) -> Result<(), SocError>;

    /// Get port count from SoC wrapper
    ///
    /// Many SoCs have registers indicating actual port count.
    fn port_count(&self) -> PortCount;

    /// Force USB2 PHY power on (if supported)
    ///
    /// Some SoCs need explicit PHY power control.
    /// Default implementation does nothing.
    fn force_usb2_phy_power(&mut self) -> Result<(), SocError> {
        Ok(())
    }
}

/// Generic SoC wrapper for plain xHCI without vendor wrapper
///
/// Use this when:
/// - xHCI is directly accessible without vendor wrapper
/// - Bootloader/firmware handles all initialization
pub struct GenericSoc {
    xhci_base: u64,
    xhci_size: usize,
    irq: u32,
}

impl GenericSoc {
    /// Create a generic SoC wrapper
    pub fn new(xhci_base: u64, xhci_size: usize, irq: u32) -> Self {
        Self {
            xhci_base,
            xhci_size,
            irq,
        }
    }
}

impl SocUsb for GenericSoc {
    fn xhci_base(&self) -> u64 {
        self.xhci_base
    }

    fn xhci_size(&self) -> usize {
        self.xhci_size
    }

    fn irq_number(&self) -> u32 {
        self.irq
    }

    fn enable_clocks(&mut self) -> Result<(), SocError> {
        // Assume clocks are already enabled
        Ok(())
    }

    fn deassert_reset(&mut self) -> Result<(), SocError> {
        // Assume reset is already de-asserted
        Ok(())
    }

    fn pre_init(&mut self) -> Result<(), SocError> {
        Ok(())
    }

    fn post_init(&mut self) -> Result<(), SocError> {
        Ok(())
    }

    fn port_count(&self) -> PortCount {
        // Unknown - will be read from xHCI capabilities
        PortCount { usb3: 0, usb2: 0 }
    }
}
