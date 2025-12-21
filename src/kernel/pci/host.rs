//! PCI Host Controller Trait
//!
//! Defines the interface that platform-specific PCI host controllers must implement.

use super::device::PciBdf;

/// PCI Host Controller trait
///
/// This trait defines the interface for accessing PCI configuration space
/// and device resources. Platform-specific implementations (e.g., MT7988A)
/// implement this trait.
pub trait PciHost {
    /// Initialize the host controller
    fn init(&mut self) -> Result<(), &'static str>;

    /// Get number of root ports
    fn port_count(&self) -> usize;

    /// Check if a port has link up
    fn link_up(&self, port: usize) -> bool;

    /// Enumerate devices on a port
    fn enumerate_port(&mut self, port: usize) -> usize;

    /// Read 32-bit value from config space
    fn config_read32(&self, bdf: PciBdf, offset: u16) -> u32;

    /// Write 32-bit value to config space
    fn config_write32(&self, bdf: PciBdf, offset: u16, value: u32);

    /// Read 16-bit value from config space
    fn config_read16(&self, bdf: PciBdf, offset: u16) -> u16 {
        let val32 = self.config_read32(bdf, offset & !0x3);
        let shift = ((offset & 0x2) as u32) * 8;
        ((val32 >> shift) & 0xFFFF) as u16
    }

    /// Write 16-bit value to config space
    fn config_write16(&self, bdf: PciBdf, offset: u16, value: u16) {
        let aligned = offset & !0x3;
        let shift = ((offset & 0x2) as u32) * 8;
        let mask = 0xFFFF << shift;
        let val32 = self.config_read32(bdf, aligned);
        let new_val = (val32 & !mask) | ((value as u32) << shift);
        self.config_write32(bdf, aligned, new_val);
    }

    /// Read 8-bit value from config space
    fn config_read8(&self, bdf: PciBdf, offset: u16) -> u8 {
        let val32 = self.config_read32(bdf, offset & !0x3);
        let shift = ((offset & 0x3) as u32) * 8;
        ((val32 >> shift) & 0xFF) as u8
    }

    /// Write 8-bit value to config space
    fn config_write8(&self, bdf: PciBdf, offset: u16, value: u8) {
        let aligned = offset & !0x3;
        let shift = ((offset & 0x3) as u32) * 8;
        let mask = 0xFF << shift;
        let val32 = self.config_read32(bdf, aligned);
        let new_val = (val32 & !mask) | ((value as u32) << shift);
        self.config_write32(bdf, aligned, new_val);
    }

    /// Get BAR physical address and size
    fn bar_info(&self, bdf: PciBdf, bar: u8) -> Option<(u64, u64)>;

    /// Check if device supports MSI
    fn msi_capable(&self, bdf: PciBdf) -> bool {
        self.find_capability(bdf, 0x05).is_some()
    }

    /// Check if device supports MSI-X
    fn msix_capable(&self, bdf: PciBdf) -> bool {
        self.find_capability(bdf, 0x11).is_some()
    }

    /// Find a capability by ID
    fn find_capability(&self, bdf: PciBdf, cap_id: u8) -> Option<u8> {
        // Check if capabilities are supported
        let status = self.config_read16(bdf, 0x06);
        if (status & 0x10) == 0 {
            return None; // No capabilities list
        }

        // Get capabilities pointer
        let mut cap_ptr = self.config_read8(bdf, 0x34) & 0xFC;

        // Walk the capability list (max 48 to avoid infinite loops)
        for _ in 0..48 {
            if cap_ptr == 0 {
                break;
            }

            let id = self.config_read8(bdf, cap_ptr as u16);
            if id == cap_id {
                return Some(cap_ptr);
            }

            cap_ptr = self.config_read8(bdf, (cap_ptr + 1) as u16) & 0xFC;
        }

        None
    }
}

/// Simplified operations trait (for trait objects without generics)
pub trait PciHostOps {
    fn port_count(&self) -> usize;
    fn link_up(&self, port: usize) -> bool;
    fn config_read32(&self, bdf: PciBdf, offset: u16) -> u32;
    fn config_write32(&self, bdf: PciBdf, offset: u16, value: u32);
    fn bar_info(&self, bdf: PciBdf, bar: u8) -> Option<(u64, u64)>;
}
