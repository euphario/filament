//! Per-Port MSI Controller
//!
//! MT7988A PCIe MSI is handled entirely within the PCIe MAC registers,
//! not the GIC ITS. Each port has 8 MSI sets × 32 vectors = 256 vectors.
//!
//! IRQ flow:
//!   GIC SPI fires → pcied reads INT_STATUS → for each MSI set bit,
//!   read set status → dispatch per-vector → clear status
//!
//! References:
//!   Linux pcie-mediatek-gen3.c: mtk_pcie_irq_handler(), mtk_pcie_msi_handler()

use crate::MmioRegion;
use crate::regs::{self, msi, PCIE_INT_STATUS_REG, PCIE_MSI_SET_BASE_REG};

/// Maximum MSI sets per port
pub const MSI_SET_COUNT: usize = msi::SET_NUM;

/// Maximum vectors per set
pub const MSI_VECTORS_PER_SET: usize = 32;

/// Total MSI vectors per port
pub const MSI_VECTORS_TOTAL: usize = MSI_SET_COUNT * MSI_VECTORS_PER_SET;

/// MSI vector assignment
#[derive(Clone, Copy, Debug)]
pub struct MsiAssignment {
    /// MSI set index (0..7)
    pub set: u8,
    /// Vector within set (0..31)
    pub vector: u8,
    /// Physical address endpoint devices write to trigger this MSI
    pub msg_addr: u64,
    /// Data value endpoint devices write
    pub msg_data: u32,
}

/// Per-port MSI controller state
pub struct MsiController {
    /// MAC physical base (for computing MSI capture addresses)
    mac_phys_base: u64,
    /// Allocation bitmap: bit N = vector N in use
    allocated: [u32; MSI_SET_COUNT],
    /// Total allocated count
    count: usize,
}

impl MsiController {
    /// Create a new MSI controller for a port.
    ///
    /// `mac_phys_base` is the physical base of the PCIe MAC (e.g., 0x11300000).
    /// MSI capture addresses are computed relative to this base.
    pub const fn new(mac_phys_base: u64) -> Self {
        Self {
            mac_phys_base,
            allocated: [0; MSI_SET_COUNT],
            count: 0,
        }
    }

    /// Allocate an MSI vector.
    ///
    /// Returns the assignment (set, vector, msg_addr, msg_data) for programming
    /// the endpoint's MSI capability registers.
    pub fn allocate(&mut self) -> Option<MsiAssignment> {
        for set in 0..MSI_SET_COUNT {
            if self.allocated[set] != 0xFFFF_FFFF {
                // Find first free bit
                let free = (!self.allocated[set]).trailing_zeros();
                if free < 32 {
                    self.allocated[set] |= 1 << free;
                    self.count += 1;

                    // MSI capture address: MAC_BASE + 0xc00 + set * 0x10
                    // This is the physical address the endpoint writes to trigger MSI
                    let msg_addr = self.mac_phys_base
                        + PCIE_MSI_SET_BASE_REG as u64
                        + (set * msi::SET_OFFSET) as u64;

                    return Some(MsiAssignment {
                        set: set as u8,
                        vector: free as u8,
                        msg_addr,
                        msg_data: free, // Vector number within set
                    });
                }
            }
        }
        None
    }

    /// Free an MSI vector.
    pub fn free(&mut self, set: u8, vector: u8) {
        if (set as usize) < MSI_SET_COUNT && vector < 32 {
            let mask = 1u32 << vector;
            if self.allocated[set as usize] & mask != 0 {
                self.allocated[set as usize] &= !mask;
                self.count -= 1;
            }
        }
    }

    /// Number of allocated vectors.
    pub fn allocated_count(&self) -> usize {
        self.count
    }

    /// Process MSI interrupts.
    ///
    /// Reads INT_STATUS to find which MSI sets fired, then reads per-set
    /// status registers to find which vectors fired. Clears status bits
    /// and returns a list of (set, vector) pairs that fired.
    ///
    /// The caller (pcied IRQ handler) dispatches to the appropriate device
    /// handler based on the (set, vector) mapping.
    ///
    /// Returns the number of MSI events written to `events`.
    pub fn process_irq(&self, mac: &MmioRegion, events: &mut [MsiEvent]) -> usize {
        let int_status = mac.read32(PCIE_INT_STATUS_REG);

        // MSI set bits are bits 15:8 in INT_STATUS
        let msi_bits = (int_status >> 8) & 0xFF;
        if msi_bits == 0 {
            return 0;
        }

        let mut event_count = 0;

        for set in 0..MSI_SET_COUNT {
            if (msi_bits & (1 << set)) == 0 {
                continue;
            }

            // Read per-set status register
            let set_status_reg = PCIE_MSI_SET_BASE_REG + set * msi::SET_OFFSET + msi::SET_STATUS_OFFSET;
            let set_status = mac.read32(set_status_reg);

            if set_status == 0 {
                continue;
            }

            // Clear status (W1C)
            mac.write32(set_status_reg, set_status);

            // Dispatch each fired vector
            let mut remaining = set_status;
            while remaining != 0 {
                let vector = remaining.trailing_zeros();
                remaining &= !(1 << vector);

                if event_count < events.len() {
                    events[event_count] = MsiEvent {
                        set: set as u8,
                        vector: vector as u8,
                    };
                    event_count += 1;
                }
            }
        }

        event_count
    }

    /// Enable a specific MSI vector in the per-set enable register.
    pub fn enable_vector(&self, mac: &MmioRegion, set: u8, vector: u8) {
        if (set as usize) >= MSI_SET_COUNT || vector >= 32 {
            return;
        }
        let enable_reg = PCIE_MSI_SET_BASE_REG + (set as usize) * msi::SET_OFFSET + msi::SET_ENABLE_OFFSET;
        let val = mac.read32(enable_reg);
        mac.write32(enable_reg, val | (1 << vector));
    }

    /// Disable a specific MSI vector in the per-set enable register.
    pub fn disable_vector(&self, mac: &MmioRegion, set: u8, vector: u8) {
        if (set as usize) >= MSI_SET_COUNT || vector >= 32 {
            return;
        }
        let enable_reg = PCIE_MSI_SET_BASE_REG + (set as usize) * msi::SET_OFFSET + msi::SET_ENABLE_OFFSET;
        let val = mac.read32(enable_reg);
        mac.write32(enable_reg, val & !(1 << vector));
    }
}

/// An MSI event from process_irq()
#[derive(Clone, Copy, Debug)]
pub struct MsiEvent {
    /// MSI set index (0..7)
    pub set: u8,
    /// Vector within set (0..31)
    pub vector: u8,
}
