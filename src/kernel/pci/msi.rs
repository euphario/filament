//! MSI/MSI-X Vector Allocation (stub)
//!
//! On MT7988A, MSI is handled per-port inside the PCIe MAC registers,
//! not through the GIC ITS. Each port has 8 MSI sets x 32 vectors = 256 vectors.
//! The MSI capture addresses, enable registers, and status registers are all
//! within the MAC MMIO region (offsets 0xc00+).
//!
//! MSI allocation and IRQ dispatch are managed by the userspace pcied driver
//! via `pcie::msi::MsiController`. The kernel does not need to track MSI state.
//!
//! This module is a stub that keeps the kernel PCI subsystem interface intact
//! while returning NoMsiVectors for allocation requests. Endpoint MSI capability
//! programming is done by the userspace driver that owns the device.

use super::{PciError, PciResult};
use super::device::PciBdf;

/// MSI allocator stub
///
/// MSI is per-port in the PCIe MAC on MT7988A. This stub satisfies the
/// PciSubsystem interface. Actual MSI management is in userspace
/// (`user/driver/pcie/src/msi.rs`).
pub struct MsiAllocator;

impl MsiAllocator {
    pub const fn new() -> Self {
        Self
    }

    /// Allocation is handled in userspace — always returns NoMsiVectors.
    pub fn allocate(&mut self, _bdf: PciBdf, _count: u8) -> PciResult<u32> {
        Err(PciError::NoMsiVectors)
    }

    /// No-op — no kernel-side MSI state to free.
    pub fn free(&mut self, _bdf: PciBdf) {}
}
