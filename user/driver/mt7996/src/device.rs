//! MT7996 Device Access
//!
//! Handles BAR mapping and register I/O for the MT7996 WiFi chipset.
//!
//! ## Dual PCIe Interface (HIF1/HIF2)
//!
//! MT7996 uses two PCIe devices for tri-band operation:
//! - HIF1 (primary): Device ID 0x7990 - handles bands 0 and 1
//! - HIF2 (secondary): Device ID 0x7991 - handles band 2
//!
//! Both interfaces must be initialized for proper operation.

use pcie::{PcieConfigSpace, PcieBdf, PcieDevice, PcieDeviceInfo};
use pcie::regs::cfg;
use crate::regs::{self, ChipVariant};

/// MT7996 initialization error
#[derive(Debug, Clone, Copy)]
pub enum Mt7996Error {
    /// Failed to map BAR
    BarMapFailed,
    /// Invalid BAR (not memory-mapped or too small)
    InvalidBar,
    /// Device not responding
    DeviceNotResponding,
    /// Unsupported chip variant
    UnsupportedVariant,
}

/// MT7996 device handle
pub struct Mt7996Device {
    /// Mapped BAR0 region (primary/HIF1)
    bar0: pcie::MmioRegion,
    /// BAR0 size
    bar0_size: u64,
    /// Device variant
    variant: ChipVariant,
    /// BDF address
    bdf: PcieBdf,
    /// PCI command register (for verifying bus master)
    command: u16,
    /// HIF2 BAR0 region (secondary interface, optional)
    hif2_bar0: Option<pcie::MmioRegion>,
    /// HIF2 BAR0 size
    hif2_bar0_size: u64,
    /// HIF2 PCI command register (for verifying bus master on HIF2)
    hif2_command: u16,
}

impl Mt7996Device {
    /// Initialize the MT7996 device
    ///
    /// Maps BAR0 and verifies the device responds.
    pub fn init(
        pcie_dev: &PcieDevice,
        cfg: &PcieConfigSpace,
    ) -> Result<Self, Mt7996Error> {
        let bdf = pcie_dev.bdf;
        let variant = ChipVariant::from_device_id(pcie_dev.id.device_id);

        // Read BAR0
        let bar0_low = cfg.read32(bdf, cfg::BAR0);

        // Check if it's a memory BAR (bit 0 = 0)
        if (bar0_low & 0x1) != 0 {
            return Err(Mt7996Error::InvalidBar);
        }

        // Check if 64-bit BAR (bits 2:1 = 10)
        let is_64bit = ((bar0_low >> 1) & 0x3) == 0x2;

        // Calculate BAR0 address
        let bar0_addr = if is_64bit {
            let bar0_high = cfg.read32(bdf, cfg::BAR0 + 4);
            ((bar0_high as u64) << 32) | ((bar0_low & !0xF) as u64)
        } else {
            (bar0_low & !0xF) as u64
        };

        // Size the BAR by writing all 1s and reading back
        // First save original value
        let orig_bar = bar0_low;

        // Write all 1s to BAR (excluding type bits)
        cfg.write32(bdf, cfg::BAR0, 0xFFFF_FFFF);
        let size_low = cfg.read32(bdf, cfg::BAR0);

        // Restore original value
        cfg.write32(bdf, cfg::BAR0, orig_bar);

        // Calculate size from read value
        let size_mask = size_low & !0xF;
        let bar0_size = if size_mask == 0 {
            // BAR not implemented or error
            0x100000 // Default to 1MB if we can't size
        } else {
            ((!size_mask) + 1) as u64
        };

        // Map BAR0
        let bar0 = pcie::MmioRegion::open(bar0_addr, bar0_size)
            .ok_or(Mt7996Error::BarMapFailed)?;

        let device = Self {
            bar0,
            bar0_size,
            variant,
            bdf,
            command: 0, // Not available via this path
            hif2_bar0: None,
            hif2_bar0_size: 0,
            hif2_command: 0,
        };

        // Verify device responds by reading a known register
        // Try reading the first DWORD which should not be 0xFFFFFFFF
        let test_read = device.read32_raw(0);
        if test_read == 0xFFFF_FFFF {
            return Err(Mt7996Error::DeviceNotResponding);
        }

        Ok(device)
    }

    /// Initialize the MT7996 device from IPC-provided device info
    ///
    /// Uses pre-discovered BAR0 info from pcied instead of reading config space.
    /// Optional HIF2 device info can be provided for dual-interface operation.
    pub fn init_from_info(info: &PcieDeviceInfo) -> Result<Self, Mt7996Error> {
        Self::init_with_hif2(info, None)
    }

    /// Initialize the MT7996 device with optional HIF2 interface
    ///
    /// For proper MT7996 operation, both HIF1 (primary) and HIF2 (secondary)
    /// interfaces should be initialized.
    pub fn init_with_hif2(info: &PcieDeviceInfo, hif2_info: Option<&PcieDeviceInfo>) -> Result<Self, Mt7996Error> {
        let bdf = PcieBdf::new(info.bus, info.device, info.function);
        let variant = ChipVariant::from_device_id(info.device_id);

        // Validate BAR0 info
        if info.bar0_addr == 0 || info.bar0_size == 0 {
            return Err(Mt7996Error::InvalidBar);
        }

        // Map BAR0 (primary/HIF1)
        let bar0 = pcie::MmioRegion::open(info.bar0_addr, info.bar0_size as u64)
            .ok_or(Mt7996Error::BarMapFailed)?;

        // Map HIF2 BAR if provided
        let (hif2_bar0, hif2_bar0_size, hif2_command) = if let Some(hif2) = hif2_info {
            if hif2.bar0_addr != 0 && hif2.bar0_size != 0 {
                let hif2_region = pcie::MmioRegion::open(hif2.bar0_addr, hif2.bar0_size as u64)
                    .ok_or(Mt7996Error::BarMapFailed)?;
                (Some(hif2_region), hif2.bar0_size as u64, hif2.command)
            } else {
                (None, 0, 0)
            }
        } else {
            (None, 0, 0)
        };

        let device = Self {
            bar0,
            bar0_size: info.bar0_size as u64,
            variant,
            bdf,
            command: info.command,
            hif2_bar0,
            hif2_bar0_size,
            hif2_command,
        };

        // Verify device responds by reading a known register
        let test_read = device.read32_raw(0);
        if test_read == 0xFFFF_FFFF {
            return Err(Mt7996Error::DeviceNotResponding);
        }

        Ok(device)
    }

    /// Read 32-bit value at raw offset (no remapping)
    #[inline]
    pub fn read32_raw(&self, offset: u32) -> u32 {
        if (offset as u64) < self.bar0_size {
            self.bar0.read32(offset as usize)
        } else {
            0xFFFF_FFFF
        }
    }

    /// Write 32-bit value at raw offset (no remapping)
    #[inline]
    pub fn write32_raw(&self, offset: u32, value: u32) {
        if (offset as u64) < self.bar0_size {
            self.bar0.write32(offset as usize, value);
        }
    }

    /// Read 32-bit value with Layer 1 remapping
    ///
    /// MT7996 uses remapping to access the full address space through
    /// a limited BAR window.
    pub fn read32(&self, addr: u32) -> u32 {
        if let Some(offset) = self.remap_addr(addr) {
            self.read32_raw(offset)
        } else {
            // Address doesn't need remapping, try direct
            self.read32_raw(addr)
        }
    }

    /// Remap an address if needed
    ///
    /// Returns the remapped offset within BAR0, or None if no remapping needed.
    fn remap_addr(&self, addr: u32) -> Option<u32> {
        // Check if address is in Layer 1 remapping ranges
        if addr >= regs::l1_range::INFRA_START && addr <= regs::l1_range::INFRA_END {
            // Infra range - needs L1 remapping
            Some(self.remap_l1(addr, regs::l1_range::INFRA_START))
        } else if addr >= regs::l1_range::WFSYS_START && addr <= regs::l1_range::WFSYS_END {
            // WFSYS range - needs L1 remapping
            Some(self.remap_l1(addr, regs::l1_range::WFSYS_START))
        } else if addr >= regs::cbtop::CBTOP1_PHY_START && addr <= regs::cbtop::CBTOP1_PHY_END {
            // CBTOP1 range - needs CBTOP remapping
            Some(self.remap_cbtop(addr, 1))
        } else if addr >= regs::cbtop::CBTOP2_PHY_START && addr <= regs::cbtop::CBTOP2_PHY_END {
            // CBTOP2 range
            Some(self.remap_cbtop(addr, 2))
        } else if addr < 0x10_0000 {
            // Small offset - direct access
            None
        } else {
            // Unknown range - try direct
            None
        }
    }

    /// Layer 1 remapping
    fn remap_l1(&self, addr: u32, base: u32) -> u32 {
        // Calculate the base and offset
        let offset = addr - base;
        let remap_base = (offset >> regs::MT_REMAP_L1_OFFSET) & regs::MT_REMAP_L1_BASE_MASK;
        let remap_offset = offset & ((1 << regs::MT_REMAP_L1_OFFSET) - 1);

        // Write remap base (simplified - real driver has locking)
        self.write32_raw(regs::MT_INFRA_REMAP_L1_BASE & 0xFFFF, remap_base | base);

        // Return the remapped offset within the L1 window
        // L1 window is at fixed offset in BAR
        0x40000 + remap_offset
    }

    /// CBTOP remapping for PCIe devices
    fn remap_cbtop(&self, addr: u32, bank: u8) -> u32 {
        let base = if bank == 1 {
            regs::cbtop::CBTOP1_PHY_START
        } else {
            regs::cbtop::CBTOP2_PHY_START
        };

        let offset = addr - base;
        let remap_reg = if bank == 1 {
            regs::MT_INFRA_REMAP_CBTOP1_BASE
        } else {
            regs::MT_INFRA_REMAP_CBTOP2_BASE
        };

        // Set remap base
        let remap_base = (offset >> 20) << 20;
        self.write32_raw(remap_reg, remap_base);

        // Return offset within CBTOP window
        let window_base = if bank == 1 { 0x60000 } else { 0x70000 };
        window_base + (offset & 0xFFFFF)
    }

    /// Get chip variant
    pub fn variant(&self) -> ChipVariant {
        self.variant
    }

    /// Get BDF address
    pub fn bdf(&self) -> PcieBdf {
        self.bdf
    }

    /// Get BAR0 size
    pub fn bar0_size(&self) -> u64 {
        self.bar0_size
    }

    /// Get BAR0 virtual address (for debugging)
    pub fn bar0_virt(&self) -> u64 {
        self.bar0.virt_base()
    }

    /// Get PCI command register value
    pub fn command(&self) -> u16 {
        self.command
    }

    /// Read hardware revision
    pub fn hw_rev(&self) -> u32 {
        self.read32(regs::MT_HW_REV)
    }

    /// Read chip ID
    pub fn chip_id(&self) -> u32 {
        self.read32(regs::MT_HW_CHIPID)
    }

    /// Check if HIF2 is available
    pub fn has_hif2(&self) -> bool {
        self.hif2_bar0.is_some()
    }

    /// Get HIF2 BAR0 virtual address (for debugging)
    pub fn hif2_bar0_virt(&self) -> Option<u64> {
        self.hif2_bar0.as_ref().map(|r| r.virt_base())
    }

    /// Get HIF2 BAR0 size
    pub fn hif2_bar0_size(&self) -> u64 {
        self.hif2_bar0_size
    }

    /// Read 32-bit value from HIF2 at raw offset
    #[inline]
    pub fn hif2_read32_raw(&self, offset: u32) -> Option<u32> {
        self.hif2_bar0.as_ref().map(|bar| {
            if (offset as u64) < self.hif2_bar0_size {
                bar.read32(offset as usize)
            } else {
                0xFFFF_FFFF
            }
        })
    }

    /// Write 32-bit value to HIF2 at raw offset
    #[inline]
    pub fn hif2_write32_raw(&self, offset: u32, value: u32) {
        if let Some(ref bar) = self.hif2_bar0 {
            if (offset as u64) < self.hif2_bar0_size {
                bar.write32(offset as usize, value);
            }
        }
    }

    /// Get HIF2 PCI command register value
    pub fn hif2_command(&self) -> u16 {
        self.hif2_command
    }

    /// Read 32-bit value from HIF2 via HIF1's BAR + offset
    ///
    /// This is how Linux accesses HIF2 registers: through HIF1's BAR with
    /// a 0x4000 offset added. This may behave differently than accessing
    /// HIF2 via its own BAR.
    #[inline]
    pub fn hif2_via_hif1_read32(&self, offset: u32) -> u32 {
        let hif2_offset = offset + regs::MT_HIF1_OFS;
        if (hif2_offset as u64) < self.bar0_size {
            self.bar0.read32(hif2_offset as usize)
        } else {
            0xFFFF_FFFF
        }
    }

    /// Write 32-bit value to HIF2 via HIF1's BAR + offset
    ///
    /// This is how Linux accesses HIF2 registers: through HIF1's BAR with
    /// a 0x4000 offset added.
    #[inline]
    pub fn hif2_via_hif1_write32(&self, offset: u32, value: u32) {
        let hif2_offset = offset + regs::MT_HIF1_OFS;
        if (hif2_offset as u64) < self.bar0_size {
            self.bar0.write32(hif2_offset as usize, value);
        }
    }
}
