//! Platform Bus Configuration
//!
//! Runtime platform configuration for buses. At boot, the kernel matches
//! the DTB compatible string against known platforms and selects the
//! appropriate configuration.
//!
//! This allows a single kernel binary to support multiple hardware variants
//! (e.g., BPI-R4 lite/normal/pro).

use crate::{kinfo, kwarn};

/// Bus configuration for a platform
#[derive(Clone, Copy)]
pub struct BusConfig {
    /// Compatible strings that match this platform (null-terminated list)
    pub compatible: &'static [&'static str],
    /// Platform name for logging
    pub name: &'static str,

    // PCIe configuration
    /// PCIe MAC base addresses (for MT7988-style) or ECAM base (for QEMU)
    pub pcie_bases: &'static [usize],
    /// INFRACFG_AO base address (for PCIe clocks/resets), 0 = not available
    pub infracfg_base: usize,
    /// True if PCIe uses ECAM (QEMU virt), false if MAC-based (MT7988)
    pub pcie_ecam_based: bool,

    // PCIe MMIO window (for BAR allocation on ECAM platforms)
    /// PCIe MMIO window base address (0 = firmware-assigned, no kernel allocation)
    pub pcie_mmio_base: u64,
    /// PCIe MMIO window end address (inclusive)
    pub pcie_mmio_end: u64,

    // USB configuration
    /// USB/xHCI MAC base addresses
    pub usb_bases: &'static [usize],
}

impl BusConfig {
    /// Number of PCIe ports
    pub fn pcie_port_count(&self) -> usize {
        self.pcie_bases.len()
    }

    /// Number of USB controllers
    pub fn usb_controller_count(&self) -> usize {
        self.usb_bases.len()
    }

    /// Get PCIe base address for a port
    pub fn pcie_base(&self, index: usize) -> Option<usize> {
        self.pcie_bases.get(index).copied()
    }

    /// Get USB base address for a controller
    pub fn usb_base(&self, index: usize) -> Option<usize> {
        self.usb_bases.get(index).copied()
    }

    /// Check if PCIe is ECAM-based (QEMU) vs MAC-based (MT7988)
    pub fn is_pcie_ecam_based(&self) -> bool {
        self.pcie_ecam_based
    }

    /// Get PCIe MMIO window (base, end) for BAR allocation
    /// Returns None if no kernel BAR allocation (firmware-assigned)
    pub fn pcie_mmio_window(&self) -> Option<(u64, u64)> {
        if self.pcie_mmio_base != 0 && self.pcie_mmio_end > self.pcie_mmio_base {
            Some((self.pcie_mmio_base, self.pcie_mmio_end))
        } else {
            None
        }
    }
}

// =============================================================================
// Platform Configurations
// =============================================================================

/// MT7988A (BPI-R4 standard)
/// - 4 PCIe ports (MAC-based access)
/// - 2 USB controllers
static MT7988A_CONFIG: BusConfig = BusConfig {
    compatible: &["mediatek,mt7988a", "mediatek,mt7988", "bananapi,bpi-r4"],
    name: "MT7988A",
    pcie_bases: &[
        0x1130_0000,  // PCIe0
        0x1131_0000,  // PCIe1
        0x1128_0000,  // PCIe2
        0x1129_0000,  // PCIe3
    ],
    infracfg_base: 0x1000_1000,
    pcie_ecam_based: false,
    pcie_mmio_base: 0,  // Firmware-assigned BARs on MT7988A
    pcie_mmio_end: 0,
    usb_bases: &[
        0x1119_0000,  // SSUSB0 (M.2 slot)
        0x1120_0000,  // SSUSB1 (USB-A ports via VL822)
    ],
};

/// MT7988A Lite variant (hypothetical)
/// - 2 PCIe ports (MAC-based access)
/// - 1 USB controller
#[allow(dead_code)]
static MT7988A_LITE_CONFIG: BusConfig = BusConfig {
    compatible: &["bananapi,bpi-r4-lite"],
    name: "MT7988A-Lite",
    pcie_bases: &[
        0x1130_0000,  // PCIe0
        0x1131_0000,  // PCIe1
    ],
    infracfg_base: 0x1000_1000,
    pcie_ecam_based: false,
    pcie_mmio_base: 0,
    pcie_mmio_end: 0,
    usb_bases: &[
        0x1119_0000,  // SSUSB0
    ],
};

/// QEMU virt machine
/// - 1 PCIe bus (ECAM-based at 0x4010000000)
/// - No USB (uses virtio instead)
static QEMU_VIRT_CONFIG: BusConfig = BusConfig {
    compatible: &["linux,dummy-virt", "arm,virt"],
    name: "QEMU-virt",
    pcie_bases: &[
        0x40_1000_0000,  // ECAM base (config space)
    ],
    infracfg_base: 0,
    pcie_ecam_based: true,
    pcie_mmio_base: 0x1000_0000,   // QEMU virt PCI MMIO window
    pcie_mmio_end: 0x3EFF_FFFF,
    usb_bases: &[],
};

/// All known platform configurations
static PLATFORMS: &[&BusConfig] = &[
    &MT7988A_CONFIG,
    // &MT7988A_LITE_CONFIG,  // Uncomment when BPI-R4 Lite is available
    &QEMU_VIRT_CONFIG,
];

/// Default configuration (used when no DTB match or DTB not available)
#[cfg(feature = "platform-mt7988a")]
static DEFAULT_CONFIG: &BusConfig = &MT7988A_CONFIG;

#[cfg(feature = "platform-qemu-virt")]
static DEFAULT_CONFIG: &BusConfig = &QEMU_VIRT_CONFIG;

#[cfg(not(any(feature = "platform-mt7988a", feature = "platform-qemu-virt")))]
static DEFAULT_CONFIG: &BusConfig = &MT7988A_CONFIG;

// =============================================================================
// Runtime Platform Selection
// =============================================================================

/// Currently selected platform configuration
static mut SELECTED_CONFIG: Option<&'static BusConfig> = None;

/// Select platform based on DTB compatible string
/// Called early in boot before buses are initialized
pub fn select_platform_from_fdt(fdt: &crate::kernel::fdt::Fdt) {
    // Try to get compatible strings from root node
    if let Some(compat_iter) = fdt.get_root_compatible() {
        for compat in compat_iter {
            // Try to match against known platforms
            for platform in PLATFORMS {
                if platform.compatible.contains(&compat) {
                    kinfo!("bus", "platform_matched";
                        name = platform.name,
                        compat = compat);
                    unsafe {
                        SELECTED_CONFIG = Some(*platform);
                    }
                    return;
                }
            }
        }

        // No match found, log what we saw
        kwarn!("bus", "platform_no_match"; using_default = DEFAULT_CONFIG.name);
    } else {
        kwarn!("bus", "fdt_no_compatible"; using_default = DEFAULT_CONFIG.name);
    }

    // Use default
    unsafe {
        SELECTED_CONFIG = Some(DEFAULT_CONFIG);
    }
}

/// Use the compile-time default platform (no DTB available)
pub fn use_default_platform() {
    kinfo!("bus", "using_default_platform"; name = DEFAULT_CONFIG.name);
    unsafe {
        SELECTED_CONFIG = Some(DEFAULT_CONFIG);
    }
}

/// Get the current bus configuration
/// Panics if called before platform selection
pub fn bus_config() -> &'static BusConfig {
    unsafe {
        SELECTED_CONFIG.unwrap_or_else(|| {
            // Not yet selected, use default
            SELECTED_CONFIG = Some(DEFAULT_CONFIG);
            DEFAULT_CONFIG
        })
    }
}

/// Check if platform has been selected
pub fn is_platform_selected() -> bool {
    // Use addr_of! to avoid creating shared reference to mutable static (Rust 2024)
    unsafe { (*core::ptr::addr_of!(SELECTED_CONFIG)).is_some() }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mt7988a_config() {
        let config = &MT7988A_CONFIG;
        assert_eq!(config.pcie_port_count(), 4);
        assert_eq!(config.usb_controller_count(), 2);
        assert_eq!(config.pcie_base(0), Some(0x1130_0000));
        assert_eq!(config.usb_base(1), Some(0x1120_0000));
        assert!(!config.is_pcie_ecam_based());
    }

    #[test]
    fn test_qemu_config() {
        let config = &QEMU_VIRT_CONFIG;
        assert_eq!(config.pcie_port_count(), 1);
        assert_eq!(config.usb_controller_count(), 0);
        assert!(config.is_pcie_ecam_based());
        assert_eq!(config.pcie_base(0), Some(0x40_1000_0000));
    }
}
