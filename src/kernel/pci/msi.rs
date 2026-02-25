//! MSI Vector Allocation
//!
//! MT7988A PCIe MSI is handled per-port inside the PCIe MAC registers,
//! not through the GIC ITS. Each port has 8 MSI sets × 32 vectors = 256 vectors.
//!
//! Virtual IRQ numbering: 1024 + port*256 + set*32 + vector
//!
//! ## Separation of Concerns
//!
//! `MsiAllocator` is a pure bitmap allocator — it tracks which vectors are
//! in use and computes virtual IRQ numbers. It has NO hardware knowledge.
//!
//! All hardware operations (MAC MSI enable, endpoint programming, GIC SPI
//! registration) live in `msi_alloc()` in `mod.rs`, which has access to
//! the PCI host and platform layer.
//!
//! References:
//!   Linux pcie-mediatek-gen3.c: mtk_pcie_irq_handler(), mtk_pcie_msi_handler()

use super::device::PciBdf;
use super::{PciError, PciResult};
use crate::kdebug;

/// First virtual IRQ number for MSI vectors
pub const MSI_IRQ_BASE: u32 = 1024;

/// MSI sets per port
pub const MSI_SET_COUNT: usize = 8;

/// Vectors per set
pub const MSI_VECTORS_PER_SET: usize = 32;

/// Vectors per port
pub const MSI_VECTORS_PER_PORT: usize = MSI_SET_COUNT * MSI_VECTORS_PER_SET;

/// Number of PCIe ports on MT7988A
pub const PORT_COUNT: usize = 4;

// MAC register offsets (from Linux pcie-mediatek-gen3.c)
pub const PCIE_INT_ENABLE_REG: usize = 0x180;
pub const PCIE_INT_STATUS_REG: usize = 0x184;
pub const PCIE_MSI_SET_ENABLE_REG: usize = 0x190;
pub const PCIE_MSI_SET_BASE_REG: usize = 0xc00;
pub const PCIE_MSI_SET_ADDR_HI_BASE: usize = 0xc80;
pub const MSI_SET_OFFSET: usize = 0x10;
pub const MSI_SET_STATUS_OFFSET: usize = 0x04;
pub const MSI_SET_ENABLE_OFFSET: usize = 0x08;
pub const MSI_ADDR_HI_OFFSET: usize = 0x04;

/// MSI enable bits in INT_ENABLE register (bits 15:8)
pub const MSI_INT_ENABLE_BITS: u32 = 0xFF << 8;

/// MSI set enable bits in MSI_SET_ENABLE register (bits 7:0)
pub const MSI_SET_ENABLE_BITS: u32 = 0xFF;

/// Result of an MSI allocation
pub struct MsiAllocation {
    pub virtual_irq: u32,
    pub port: u8,
    pub set: u8,
    pub vector: u8,
}

/// MSI vector allocator — pure bitmap tracker, no hardware access.
///
/// Per-port state: 8 MSI sets × 32 vectors = 256 vectors per port.
/// Tracks which vectors are allocated and whether each port's hardware
/// has been initialized (so `msi_alloc()` can do first-time MAC setup).
pub struct MsiAllocator {
    ports: [PortBitmaps; PORT_COUNT],
}

/// Per-port allocation bitmaps and initialization flags.
///
/// Hardware init state lives here because the allocator is the single
/// source of truth for "has this port had MSI set up". The actual
/// hardware writes happen in `msi_alloc()` which reads these flags.
struct PortBitmaps {
    /// Allocation bitmaps — bit N in set S means vector N of set S is in use
    allocated: [u32; MSI_SET_COUNT],
    /// Whether MAC MSI registers have been programmed for this port
    mac_initialized: bool,
    /// Whether the port's GIC SPI has been enabled for MSI dispatch
    gic_spi_enabled: bool,
}

impl PortBitmaps {
    const fn new() -> Self {
        Self {
            allocated: [0; MSI_SET_COUNT],
            mac_initialized: false,
            gic_spi_enabled: false,
        }
    }
}

impl MsiAllocator {
    pub const fn new() -> Self {
        Self {
            ports: [
                PortBitmaps::new(), PortBitmaps::new(),
                PortBitmaps::new(), PortBitmaps::new(),
            ],
        }
    }

    /// Allocate a single MSI vector from the bitmap for the given port.
    ///
    /// Caller must validate that the port is valid and has a MAC base
    /// before calling. This function only does bitmap accounting.
    pub fn allocate(&mut self, port: usize, count: u8) -> PciResult<MsiAllocation> {
        if count != 1 {
            return Err(PciError::NoMsiVectors);
        }
        if port >= PORT_COUNT {
            return Err(PciError::NoMsiVectors);
        }

        let bitmaps = &mut self.ports[port];

        for set in 0..MSI_SET_COUNT {
            if bitmaps.allocated[set] != 0xFFFF_FFFF {
                let free = (!bitmaps.allocated[set]).trailing_zeros();
                if free < 32 {
                    bitmaps.allocated[set] |= 1 << free;
                    let virtual_irq = MSI_IRQ_BASE
                        + (port as u32) * MSI_VECTORS_PER_PORT as u32
                        + (set as u32) * MSI_VECTORS_PER_SET as u32
                        + free;

                    kdebug!("msi", "vector_allocated"; port = port as u64,
                        set = set as u64, vector = free as u64, virq = virtual_irq as u64);

                    return Ok(MsiAllocation {
                        virtual_irq,
                        port: port as u8,
                        set: set as u8,
                        vector: free as u8,
                    });
                }
            }
        }

        Err(PciError::NoMsiVectors)
    }

    /// Free all MSI vectors for a device on a given port.
    pub fn free(&mut self, _bdf: PciBdf) {
        // TODO: track per-device allocations for proper cleanup
    }

    /// Whether this port's MAC MSI registers need first-time setup.
    pub fn needs_mac_init(&self, port: usize) -> bool {
        port < PORT_COUNT && !self.ports[port].mac_initialized
    }

    /// Mark port MAC MSI as initialized.
    pub fn mark_mac_initialized(&mut self, port: usize) {
        if port < PORT_COUNT {
            self.ports[port].mac_initialized = true;
        }
    }

    /// Whether this port's GIC SPI needs to be enabled.
    pub fn needs_gic_spi_enable(&self, port: usize) -> bool {
        port < PORT_COUNT && !self.ports[port].gic_spi_enabled
    }

    /// Mark port GIC SPI as enabled.
    pub fn mark_gic_spi_enabled(&mut self, port: usize) {
        if port < PORT_COUNT {
            self.ports[port].gic_spi_enabled = true;
        }
    }

    // ========================================================================
    // Pure utility functions (no state access)
    // ========================================================================

    /// Extract port index from BDF (upper nibble of bus = port on MT7988A)
    pub fn port_for_bdf(bdf: &PciBdf) -> usize {
        (bdf.bus >> 4) as usize
    }

    /// Check if a virtual IRQ is an MSI virtual IRQ
    pub fn is_msi_irq(irq_num: u32) -> bool {
        irq_num >= MSI_IRQ_BASE
    }

    /// Decode virtual IRQ → (port, set, vector)
    pub fn decode_virq(virtual_irq: u32) -> Option<(usize, usize, usize)> {
        if virtual_irq < MSI_IRQ_BASE {
            return None;
        }
        let offset = virtual_irq - MSI_IRQ_BASE;
        let port = (offset / MSI_VECTORS_PER_PORT as u32) as usize;
        let rem = (offset % MSI_VECTORS_PER_PORT as u32) as usize;
        let set = rem / MSI_VECTORS_PER_SET;
        let vector = rem % MSI_VECTORS_PER_SET;

        if port < PORT_COUNT && set < MSI_SET_COUNT && vector < 32 {
            Some((port, set, vector))
        } else {
            None
        }
    }
}
