//! Board Configuration Abstraction
//!
//! This module defines the `Board` trait that ties together:
//! - SoC wrapper selection
//! - Slot descriptions (which physical slot is each port)
//! - Board-specific power/GPIO control
//!
//! ## Why Board-Specific Configuration?
//!
//! The same SoC can appear on different boards with different:
//! - Physical slot layouts (M.2, mPCIe, full-size PCIe)
//! - Power enable GPIO pins
//! - Perst# signal GPIO
//! - Available ports (some may be unpopulated)
//!
//! ## Example: MT7988A Boards
//!
//! - **BPI-R4**: pcie0=mPCIe SIM2, pcie1=mPCIe SIM3, pcie2=M.2 Key-B, pcie3=M.2 Key-M
//! - **Other board**: Same SoC, different slot layout

pub mod bpi_r4;

pub use bpi_r4::BpiR4;

use crate::soc::SocPcie;

/// Board initialization error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardError {
    /// Failed to initialize SoC wrapper
    SocInitFailed,
    /// Failed to enable power for a slot
    PowerFailed,
    /// Invalid port number
    InvalidPort,
}

/// Extended port information with board-specific details
#[derive(Debug, Clone, Copy)]
pub struct SlotInfo {
    /// Port index
    pub port: u8,
    /// Physical slot description (e.g., "mPCIe SIM2", "M.2 Key-M")
    pub slot_name: &'static str,
    /// Whether this slot is populated/available
    pub available: bool,
    /// Expected device type (optional, for documentation)
    pub expected_device: Option<&'static str>,
}

/// Board configuration trait
///
/// Implement this trait for each supported board to provide:
/// - Which SoC wrapper to use
/// - Board-specific slot information
/// - Power/GPIO control
///
/// ## Generic Parameters
/// - `S`: SoC wrapper type implementing `SocPcie`
pub trait Board {
    /// SoC wrapper type
    type Soc: SocPcie;

    /// Get board name
    fn name(&self) -> &'static str;

    /// Get the SoC wrapper
    fn soc(&self) -> &Self::Soc;

    /// Get mutable SoC wrapper
    fn soc_mut(&mut self) -> &mut Self::Soc;

    /// Get slot information for a port
    fn slot_info(&self, port: u8) -> Option<SlotInfo>;

    /// Get all available slots
    fn slots(&self) -> &[SlotInfo];

    /// Board-specific pre-initialization
    ///
    /// Called before any PCIe initialization. Use for:
    /// - Enabling power rails
    /// - Configuring GPIOs
    fn pre_init(&mut self) -> Result<(), BoardError> {
        Ok(())
    }

    /// Enable power for a slot
    ///
    /// This is board-specific because power enable GPIO
    /// varies between boards.
    fn enable_slot_power(&mut self, _port: u8) -> Result<(), BoardError> {
        Ok(())
    }

    /// Assert PERST# for a slot (put device in reset)
    fn assert_perst(&mut self, _port: u8) -> Result<(), BoardError> {
        Ok(())
    }

    /// Deassert PERST# for a slot (release device from reset)
    fn deassert_perst(&mut self, _port: u8) -> Result<(), BoardError> {
        Ok(())
    }

    /// Board-specific post-initialization
    fn post_init(&mut self) -> Result<(), BoardError> {
        Ok(())
    }
}

/// PCIe initialization helper
///
/// This struct orchestrates PCIe initialization using the proper
/// layer separation: Board -> SoC -> Controller
pub struct PcieInit<B: Board> {
    board: B,
}

impl<B: Board> PcieInit<B> {
    /// Create a new PCIe initializer for a board
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

    /// Initialize the board and SoC for PCIe
    ///
    /// Performs the following sequence:
    /// 1. Board pre-init (power rails, GPIOs)
    /// 2. Deassert PHY reset
    /// 3. Enable clocks
    /// 4. Board post-init
    pub fn init(&mut self) -> Result<(), BoardError> {
        // Board pre-init
        self.board.pre_init()?;

        // Deassert PHY reset first (required before clock enable)
        self.board.soc().deassert_phy_reset()
            .map_err(|_| BoardError::SocInitFailed)?;

        // Small delay for reset to take effect
        crate::delay_ms(10);

        // Enable clocks
        self.board.soc().enable_clocks()
            .map_err(|_| BoardError::SocInitFailed)?;

        // Wait for clocks to stabilize
        crate::delay_ms(10);

        // Board post-init
        self.board.post_init()?;

        Ok(())
    }
}
