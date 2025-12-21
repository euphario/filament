//! MSI/MSI-X Vector Allocation
//!
//! Manages allocation of MSI vectors to PCI devices.

use super::{PciError, PciResult, MAX_MSI_VECTORS};
use super::device::PciBdf;

/// An allocated MSI vector
#[derive(Debug, Clone, Copy)]
pub struct MsiVector {
    /// GIC IRQ number (SPI base + offset)
    pub irq: u32,
    /// MSI address (for device programming)
    pub address: u64,
    /// MSI data (for device programming)
    pub data: u32,
}

/// MSI allocation entry
#[derive(Clone, Copy)]
struct MsiEntry {
    /// Device that owns this vector
    bdf: PciBdf,
    /// IRQ number
    irq: u32,
    /// Is this entry in use?
    in_use: bool,
}

impl MsiEntry {
    const fn empty() -> Self {
        Self {
            bdf: PciBdf::new(0, 0, 0),
            irq: 0,
            in_use: false,
        }
    }
}

/// MSI Vector Allocator
///
/// Manages a pool of MSI vectors and tracks which device owns each one.
pub struct MsiAllocator {
    /// Allocation entries
    entries: [MsiEntry; MAX_MSI_VECTORS],
    /// Base IRQ for MSI vectors (GIC SPI number)
    base_irq: u32,
    /// MSI target address (GIC ITS or direct)
    target_address: u64,
    /// Number of allocated vectors
    allocated: usize,
}

impl MsiAllocator {
    /// Create new allocator
    pub const fn new() -> Self {
        Self {
            entries: [MsiEntry::empty(); MAX_MSI_VECTORS],
            // MT7988A: MSI vectors start at SPI 236 (IRQ 268)
            // GIC-600 supports MSI through ITS at 0x0B000000
            base_irq: 268,
            target_address: 0x0B00_0040, // GIC ITS TRANSLATER register
            allocated: 0,
        }
    }

    /// Set base IRQ for MSI vectors
    pub fn set_base_irq(&mut self, irq: u32) {
        self.base_irq = irq;
    }

    /// Set MSI target address
    pub fn set_target_address(&mut self, addr: u64) {
        self.target_address = addr;
    }

    /// Allocate MSI vectors for a device
    ///
    /// Returns the first IRQ number allocated.
    pub fn allocate(&mut self, bdf: PciBdf, count: u8) -> PciResult<u32> {
        if count == 0 {
            return Err(PciError::NoMsiVectors);
        }

        // Find contiguous free slots
        let count = count as usize;
        let mut start = None;
        let mut run = 0;

        for i in 0..MAX_MSI_VECTORS {
            if !self.entries[i].in_use {
                if run == 0 {
                    start = Some(i);
                }
                run += 1;
                if run >= count {
                    break;
                }
            } else {
                start = None;
                run = 0;
            }
        }

        let start_idx = start.ok_or(PciError::NoMsiVectors)?;
        if run < count {
            return Err(PciError::NoMsiVectors);
        }

        // Allocate the vectors
        let first_irq = self.base_irq + start_idx as u32;
        for i in 0..count {
            self.entries[start_idx + i] = MsiEntry {
                bdf,
                irq: first_irq + i as u32,
                in_use: true,
            };
        }
        self.allocated += count;

        Ok(first_irq)
    }

    /// Free all MSI vectors for a device
    pub fn free(&mut self, bdf: PciBdf) {
        for entry in self.entries.iter_mut() {
            if entry.in_use && entry.bdf == bdf {
                entry.in_use = false;
                self.allocated -= 1;
            }
        }
    }

    /// Get MSI vector info for programming the device
    pub fn get_vector(&self, irq: u32) -> Option<MsiVector> {
        let idx = irq.checked_sub(self.base_irq)? as usize;
        if idx >= MAX_MSI_VECTORS {
            return None;
        }

        let entry = &self.entries[idx];
        if !entry.in_use {
            return None;
        }

        Some(MsiVector {
            irq,
            address: self.target_address,
            data: irq, // Simple: data = IRQ number
        })
    }

    /// Find which device owns an IRQ
    pub fn irq_owner(&self, irq: u32) -> Option<PciBdf> {
        let idx = irq.checked_sub(self.base_irq)? as usize;
        if idx >= MAX_MSI_VECTORS {
            return None;
        }

        let entry = &self.entries[idx];
        if entry.in_use {
            Some(entry.bdf)
        } else {
            None
        }
    }

    /// Get number of allocated vectors
    pub fn allocated_count(&self) -> usize {
        self.allocated
    }

    /// Get number of free vectors
    pub fn free_count(&self) -> usize {
        MAX_MSI_VECTORS - self.allocated
    }
}

/// Program a device's MSI capability
pub fn program_msi<F>(
    msi_cap_offset: u8,
    vector: &MsiVector,
    write32: F,
) where
    F: Fn(u16, u32),
{
    // MSI capability layout:
    // +0: Capability header (ID, Next, Control)
    // +4: Message Address (lower 32 bits)
    // +8: Message Address (upper 32 bits) - if 64-bit capable
    // +8/+12: Message Data

    // Write address (assuming 64-bit capable for simplicity)
    write32((msi_cap_offset + 4) as u16, vector.address as u32);
    write32((msi_cap_offset + 8) as u16, (vector.address >> 32) as u32);

    // Write data
    write32((msi_cap_offset + 12) as u16, vector.data);
}

/// Program a device's MSI-X capability
pub fn program_msix_entry<F>(
    table_base: u64,
    entry_index: u16,
    vector: &MsiVector,
    write32: F,
) where
    F: Fn(u64, u32),
{
    // MSI-X table entry layout (16 bytes each):
    // +0: Message Address (lower 32 bits)
    // +4: Message Address (upper 32 bits)
    // +8: Message Data
    // +12: Vector Control (bit 0 = masked)

    let entry_offset = (entry_index as u64) * 16;
    let entry_addr = table_base + entry_offset;

    write32(entry_addr, vector.address as u32);
    write32(entry_addr + 4, (vector.address >> 32) as u32);
    write32(entry_addr + 8, vector.data);
    write32(entry_addr + 12, 0); // Unmask
}
