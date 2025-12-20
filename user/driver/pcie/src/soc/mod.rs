//! SoC PCIe Wrapper Abstraction
//!
//! This module defines the `SocPcie` trait for SoC-specific PCIe initialization.
//!
//! ## Why SoC Wrappers Exist
//!
//! Most SoCs wrap the PCIe controller with vendor-specific logic:
//! - **MediaTek**: INFRACFG_AO for clock gates, reset controller
//! - **Qualcomm**: PARF (PCIe Abstraction Register File)
//! - **Rockchip**: GRF (General Register Files) for PHY control
//!
//! The PCIe configuration space access is often vendor-specific too:
//! - **Standard**: ECAM (Enhanced Configuration Access Mechanism)
//! - **MediaTek Gen3**: TLP-based access through MAC registers
//!
//! ## Responsibilities
//! - Enable clocks for PCIe controllers
//! - Control reset signals (PEXTP PHY, MAC, bridge)
//! - Provide port configuration (addresses, IRQs)
//!
//! ## What does NOT belong here
//! - Standard PCIe config space operations (that's in `config` module)
//! - Board-specific GPIO/power (that's in `board` module)

pub mod mt7988a;

pub use mt7988a::Mt7988aSoc;

/// SoC initialization error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SocError {
    /// Clock enable failed
    ClockFailed,
    /// Reset de-assert failed
    ResetFailed,
    /// MMIO mapping failed
    MmioFailed,
    /// Invalid port number
    InvalidPort,
    /// Timeout waiting for hardware
    Timeout,
}

/// PCIe port configuration from SoC
#[derive(Debug, Clone, Copy)]
pub struct PciePortConfig {
    /// Port index (0-based)
    pub index: u8,
    /// MAC controller base address
    pub mac_base: u64,
    /// MAC controller region size
    pub mac_size: u64,
    /// Memory window base address (for BAR allocation)
    pub mem_base: u64,
    /// Memory window size
    pub mem_size: u64,
    /// IRQ number (GIC)
    pub irq: u32,
    /// Human-readable description
    pub desc: &'static str,
}

/// SoC PCIe Wrapper trait
///
/// Implement this trait for each SoC to handle vendor-specific
/// PCIe initialization.
///
/// ## Responsibilities
/// - Enable clocks for PCIe ports
/// - Control PHY and MAC reset signals
/// - Provide port configuration
///
/// ## What does NOT belong here
/// - PCIe config space access (use `PcieConfigSpace`)
/// - Board-specific GPIO (use `Board` trait)
pub trait SocPcie {
    /// Get available port count
    fn port_count(&self) -> u8;

    /// Get port configuration by index
    fn port_config(&self, index: u8) -> Option<PciePortConfig>;

    /// Enable clocks for all PCIe ports
    fn enable_clocks(&self) -> Result<(), SocError>;

    /// Enable clocks for a specific port
    fn enable_port_clocks(&self, index: u8) -> Result<(), SocError>;

    /// Deassert PHY reset (release PHY from reset)
    fn deassert_phy_reset(&self) -> Result<(), SocError>;

    /// Assert PHY reset (put PHY in reset state)
    fn assert_phy_reset(&self) -> Result<(), SocError>;

    /// Check if PHY reset is asserted
    fn is_phy_reset_asserted(&self) -> bool;

    /// Get clock status for debugging (implementation-specific format)
    fn clock_status(&self) -> (u32, u32) {
        (0, 0)
    }

    /// Get reset status for debugging (implementation-specific format)
    fn reset_status(&self) -> u32 {
        0
    }
}

/// Generic SoC for platforms that don't need special initialization
///
/// Use this when:
/// - PCIe is already initialized by bootloader/firmware
/// - Platform uses standard ECAM without vendor wrapper
pub struct GenericSoc {
    ports: &'static [PciePortConfig],
}

impl GenericSoc {
    /// Create a generic SoC wrapper
    pub const fn new(ports: &'static [PciePortConfig]) -> Self {
        Self { ports }
    }
}

impl SocPcie for GenericSoc {
    fn port_count(&self) -> u8 {
        self.ports.len() as u8
    }

    fn port_config(&self, index: u8) -> Option<PciePortConfig> {
        self.ports.get(index as usize).copied()
    }

    fn enable_clocks(&self) -> Result<(), SocError> {
        // Assume clocks already enabled
        Ok(())
    }

    fn enable_port_clocks(&self, _index: u8) -> Result<(), SocError> {
        Ok(())
    }

    fn deassert_phy_reset(&self) -> Result<(), SocError> {
        // Assume reset already deasserted
        Ok(())
    }

    fn assert_phy_reset(&self) -> Result<(), SocError> {
        Ok(())
    }

    fn is_phy_reset_asserted(&self) -> bool {
        false
    }
}
