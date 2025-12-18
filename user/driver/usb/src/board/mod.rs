//! Board Configuration Abstraction
//!
//! This module defines the `Board` trait that ties together:
//! - SoC wrapper selection
//! - PHY driver selection
//! - Board-specific GPIO and power control
//!
//! ## Why Board-Specific Configuration?
//!
//! The same SoC can appear on different boards with different:
//! - VBUS enable GPIO pins
//! - Power sequencing requirements
//! - External USB hubs or PHYs
//! - Available USB ports
//!
//! ## Example: MT7988A Boards
//!
//! - **BPI-R4**: SSUSB1 -> VL822 hub -> 4x USB-A
//! - **Other board**: Same SoC, different GPIO, different port layout

mod bpi_r4;

pub use bpi_r4::BpiR4;

use crate::soc::SocUsb;
use crate::phy::PhyDriver;

/// Board initialization error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardError {
    /// Failed to enable VBUS
    VbusFailed,
    /// Failed to configure power rails
    PowerFailed,
    /// Failed to map MMIO
    MmioFailed,
    /// Invalid USB controller
    InvalidController,
}

/// USB controller configuration from board
#[derive(Debug, Clone)]
pub struct UsbControllerConfig {
    /// Human-readable name
    pub name: &'static str,
    /// Controller index (0-based)
    pub index: u8,
    /// MAC (xHCI) MMIO base
    pub mac_base: u64,
    /// MAC MMIO size
    pub mac_size: usize,
    /// PHY MMIO base
    pub phy_base: u64,
    /// PHY MMIO size
    pub phy_size: usize,
    /// IRQ number
    pub irq: u32,
}

/// Board configuration trait
///
/// Implement this trait for each supported board to provide:
/// - Which SoC wrapper to use
/// - Which PHY driver to use
/// - Board-specific initialization (GPIO, power)
///
/// ## Generic Parameters
/// - `S`: SoC wrapper type implementing `SocUsb`
/// - `P`: PHY driver type implementing `PhyDriver`
pub trait Board {
    /// SoC wrapper type
    type Soc: SocUsb;
    /// PHY driver type
    type Phy: PhyDriver;

    /// Get board name
    fn name(&self) -> &'static str;

    /// Get available USB controllers on this board
    fn usb_controllers(&self) -> &[UsbControllerConfig];

    /// Create SoC wrapper for a controller
    fn create_soc(&self, index: u8) -> Option<Self::Soc>;

    /// Create PHY driver for a controller
    fn create_phy(&self, index: u8) -> Option<Self::Phy>;

    /// Board-specific pre-initialization
    ///
    /// Called before any USB initialization. Use for:
    /// - Enabling power rails
    /// - Configuring muxes
    /// - Early GPIO setup
    fn pre_init(&mut self) -> Result<(), BoardError> {
        Ok(())
    }

    /// Enable VBUS power for a controller
    ///
    /// This is board-specific because VBUS enable GPIO
    /// varies between boards.
    fn enable_vbus(&mut self, _index: u8) -> Result<(), BoardError> {
        Ok(())
    }

    /// Disable VBUS power for a controller
    fn disable_vbus(&mut self, _index: u8) -> Result<(), BoardError> {
        Ok(())
    }

    /// Board-specific post-initialization
    ///
    /// Called after USB controllers are initialized.
    fn post_init(&mut self) -> Result<(), BoardError> {
        Ok(())
    }
}

/// USB initialization helper
///
/// This struct orchestrates USB initialization using the proper
/// layer separation: Board -> SoC -> PHY -> xHCI
pub struct UsbInit<B: Board> {
    board: B,
}

impl<B: Board> UsbInit<B> {
    /// Create a new USB initializer for a board
    pub fn new(board: B) -> Self {
        Self { board }
    }

    /// Get board reference
    pub fn board(&self) -> &B {
        &self.board
    }

    /// Get mutable board reference
    pub fn board_mut(&mut self) -> &mut B {
        &mut self.board
    }
}
