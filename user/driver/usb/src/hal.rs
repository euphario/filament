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
// MediaTek MT7988A HAL Implementation (Reference)
// =============================================================================

/// MT7988A SSUSB controller configuration
pub mod mt7988a {
    /// Controller indices for MT7988A
    pub const SSUSB0: u32 = 0;  // XS-PHY controller
    pub const SSUSB1: u32 = 1;  // T-PHY controller

    /// IPPC offset from MAC base
    pub const IPPC_OFFSET: usize = 0x3E00;

    /// Default MAC region size
    pub const MAC_SIZE: usize = 0x4000;
}
