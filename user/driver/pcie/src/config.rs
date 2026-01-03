//! PCIe Configuration Space Access
//!
//! MediaTek-specific configuration access via TLP mechanism.
//! Config space is accessed through MAC registers, NOT standard ECAM.

use crate::MmioRegion;
use crate::regs::{cfg, cfgnum, PCIE_CFGNUM_REG, PCIE_CFG_OFFSET_ADDR};

/// PCIe Bus/Device/Function address
#[derive(Debug, Clone, Copy, Default)]
pub struct PcieBdf {
    pub bus: u8,
    pub device: u8,
    pub function: u8,
}

impl PcieBdf {
    /// Create new BDF address
    pub const fn new(bus: u8, device: u8, function: u8) -> Self {
        Self { bus, device, function }
    }

    /// Convert BDF + register to ECAM offset
    ///
    /// ECAM format: Bus[27:20] | Device[19:15] | Function[14:12] | Register[11:0]
    pub fn to_ecam_offset(&self, reg: u16) -> usize {
        ((self.bus as usize) << 20) |
        ((self.device as usize) << 15) |
        ((self.function as usize) << 12) |
        ((reg as usize) & 0xFFF)
    }
}

/// PCIe device identification
#[derive(Debug, Clone, Copy, Default)]
pub struct PcieDeviceId {
    /// Vendor ID (e.g., 0x14C3 for MediaTek)
    pub vendor_id: u16,
    /// Device ID (e.g., 0x7996 for MT7996)
    pub device_id: u16,
    /// Class code (24-bit: class << 16 | subclass << 8 | prog_if)
    pub class_code: u32,
    /// Revision ID
    pub revision: u8,
}

impl PcieDeviceId {
    /// Get the base class (upper 8 bits of class code)
    pub fn base_class(&self) -> u8 {
        ((self.class_code >> 16) & 0xFF) as u8
    }

    /// Get the subclass (middle 8 bits)
    pub fn subclass(&self) -> u8 {
        ((self.class_code >> 8) & 0xFF) as u8
    }

    /// Get the programming interface (lower 8 bits)
    pub fn prog_if(&self) -> u8 {
        (self.class_code & 0xFF) as u8
    }

    /// Get human-readable class name
    pub fn class_name(&self) -> &'static str {
        match (self.base_class(), self.subclass()) {
            (0x00, _) => "Unclassified device",
            (0x01, 0x00) => "SCSI storage controller",
            (0x01, 0x01) => "IDE controller",
            (0x01, 0x06) => "SATA controller",
            (0x01, 0x08) => "NVMe controller",
            (0x01, _) => "Storage controller",
            (0x02, 0x00) => "Ethernet controller",
            (0x02, 0x80) => "Network controller",
            (0x02, _) => "Network controller",
            (0x03, _) => "Display controller",
            (0x04, _) => "Multimedia controller",
            (0x05, _) => "Memory controller",
            (0x06, 0x00) => "Host bridge",
            (0x06, 0x01) => "ISA bridge",
            (0x06, 0x04) => "PCI bridge",
            (0x06, _) => "Bridge device",
            (0x07, _) => "Communication controller",
            (0x08, _) => "System peripheral",
            (0x0C, 0x03) => "USB controller",
            (0x0C, _) => "Serial bus controller",
            (0x0D, 0x00) => "IRDA controller",
            (0x0D, 0x11) => "Bluetooth controller",
            (0x0D, _) => "Wireless controller",
            _ => "Unknown device",
        }
    }
}

/// PCIe configuration space accessor
/// Uses MediaTek TLP-based config access through MAC registers
pub struct PcieConfigSpace<'a> {
    /// MAC MMIO region (NOT a separate config window)
    mac: &'a MmioRegion,
}

impl<'a> PcieConfigSpace<'a> {
    /// Create new config space accessor using MAC region
    pub fn new(mac: &'a MmioRegion) -> Self {
        Self { mac }
    }

    /// Set up TLP header for config access
    fn setup_tlp(&self, bdf: PcieBdf, where_: u16, size: u32) {
        // Calculate byte enable mask based on access size and offset
        // For 32-bit access at aligned offset: 0xF
        // For 16-bit access: 0x3 << (where & 0x3)
        // For 8-bit access: 0x1 << (where & 0x3)
        let bytes = ((1u32 << size) - 1) << (where_ & 0x3) as u32;

        // Build devfn: device in bits 7:3, function in bits 2:0
        let devfn = ((bdf.device as u32) << 3) | (bdf.function as u32);

        // Build config number register value
        let cfgnum = cfgnum::FORCE_BYTE_EN
            | ((bytes & 0xF) << cfgnum::BYTE_EN_SHIFT)
            | ((bdf.bus as u32) << cfgnum::BUS_SHIFT)
            | (devfn & cfgnum::DEVFN_MASK);

        self.mac.write32(PCIE_CFGNUM_REG, cfgnum);
    }

    /// Read 32-bit value from config space
    pub fn read32(&self, bdf: PcieBdf, reg: u16) -> u32 {
        self.setup_tlp(bdf, reg, 4);
        let offset = PCIE_CFG_OFFSET_ADDR + ((reg as usize) & !0x3);
        self.mac.read32(offset)
    }

    /// Read 16-bit value from config space
    pub fn read16(&self, bdf: PcieBdf, reg: u16) -> u16 {
        self.setup_tlp(bdf, reg, 2);
        let offset = PCIE_CFG_OFFSET_ADDR + ((reg as usize) & !0x3);
        let val32 = self.mac.read32(offset);
        let shift = ((reg & 0x2) as u32) * 8;
        ((val32 >> shift) & 0xFFFF) as u16
    }

    /// Read 8-bit value from config space
    pub fn read8(&self, bdf: PcieBdf, reg: u16) -> u8 {
        self.setup_tlp(bdf, reg, 1);
        let offset = PCIE_CFG_OFFSET_ADDR + ((reg as usize) & !0x3);
        let val32 = self.mac.read32(offset);
        let shift = ((reg & 0x3) as u32) * 8;
        ((val32 >> shift) & 0xFF) as u8
    }

    /// Write 32-bit value to config space
    pub fn write32(&self, bdf: PcieBdf, reg: u16, value: u32) {
        self.setup_tlp(bdf, reg, 4);
        let offset = PCIE_CFG_OFFSET_ADDR + ((reg as usize) & !0x3);
        self.mac.write32(offset, value);
    }

    /// Write 16-bit value to config space
    pub fn write16(&self, bdf: PcieBdf, reg: u16, value: u16) {
        // For 16-bit writes, we need to read-modify-write
        // to avoid clobbering adjacent 16-bit register
        let aligned_reg = reg & !0x3;
        self.setup_tlp(bdf, aligned_reg, 4);
        let offset = PCIE_CFG_OFFSET_ADDR + (aligned_reg as usize);

        let val32 = self.mac.read32(offset);
        let shift = ((reg & 0x2) as u32) * 8;
        let mask = !(0xFFFFu32 << shift);
        let new_val = (val32 & mask) | ((value as u32) << shift);

        self.setup_tlp(bdf, aligned_reg, 4);
        self.mac.write32(offset, new_val);
    }

    /// Find a PCI capability by ID
    /// Returns the capability's base offset in config space, or None if not found
    pub fn find_capability(&self, bdf: PcieBdf, cap_id: u8) -> Option<u16> {
        use crate::regs::cfg;

        // Check if device has capabilities (status register bit 4)
        let status = self.read16(bdf, cfg::STATUS);
        if (status & (1 << 4)) == 0 {
            return None;  // No capabilities
        }

        // Get capabilities pointer
        let mut cap_ptr = self.read8(bdf, cfg::CAP_PTR) & !0x3;  // Must be dword aligned
        if cap_ptr == 0 {
            return None;
        }

        // Walk capability list (max 48 to avoid infinite loops)
        for _ in 0..48 {
            if cap_ptr == 0 || cap_ptr == 0xFF {
                break;
            }

            let id = self.read8(bdf, cap_ptr as u16);
            if id == cap_id {
                return Some(cap_ptr as u16);
            }

            // Next pointer is at offset +1
            cap_ptr = self.read8(bdf, cap_ptr as u16 + 1);
        }

        None
    }

    /// Find PCIe Express Capability
    pub fn find_pcie_capability(&self, bdf: PcieBdf) -> Option<u16> {
        use crate::regs::cap_id;
        self.find_capability(bdf, cap_id::PCIE)
    }

    /// Disable ASPM (Active State Power Management) on a device
    ///
    /// This is critical for WiFi devices like MT7996 which may not work
    /// correctly with power management enabled.
    ///
    /// Returns true if ASPM was disabled, false if PCIe capability not found
    pub fn disable_aspm(&self, bdf: PcieBdf) -> bool {
        use crate::regs::{pcie_cap, link_ctl};

        let pcie_cap_base = match self.find_pcie_capability(bdf) {
            Some(base) => base,
            None => return false,
        };

        // Read Link Control register
        let link_ctl_reg = pcie_cap_base + pcie_cap::LINK_CTL;
        let link_ctl_val = self.read16(bdf, link_ctl_reg);

        // Check if ASPM is enabled
        if (link_ctl_val & link_ctl::ASPM_MASK) != 0 {
            // Clear ASPM bits
            let new_val = link_ctl_val & !link_ctl::ASPM_MASK;
            self.write16(bdf, link_ctl_reg, new_val);

            // Verify
            let verify = self.read16(bdf, link_ctl_reg);
            return (verify & link_ctl::ASPM_MASK) == 0;
        }

        true  // ASPM already disabled
    }

    /// Check if a device exists at the given BDF
    pub fn device_exists(&self, bdf: PcieBdf) -> bool {
        let vendor = self.read16(bdf, cfg::VENDOR_ID);
        vendor != 0xFFFF && vendor != 0x0000
    }

    /// Read device identification
    pub fn read_device_id(&self, bdf: PcieBdf) -> Option<PcieDeviceId> {
        let vendor_id = self.read16(bdf, cfg::VENDOR_ID);

        // 0xFFFF means no device present
        if vendor_id == 0xFFFF {
            return None;
        }

        let device_id = self.read16(bdf, cfg::DEVICE_ID);
        let class_rev = self.read32(bdf, cfg::REVISION_ID);

        Some(PcieDeviceId {
            vendor_id,
            device_id,
            class_code: class_rev >> 8,
            revision: (class_rev & 0xFF) as u8,
        })
    }

    /// Read header type
    pub fn read_header_type(&self, bdf: PcieBdf) -> u8 {
        self.read8(bdf, cfg::HEADER_TYPE)
    }

    /// Check if device is multi-function
    pub fn is_multifunction(&self, bdf: PcieBdf) -> bool {
        (self.read_header_type(bdf) & 0x80) != 0
    }

    /// Check if device is a PCI bridge
    pub fn is_bridge(&self, bdf: PcieBdf) -> bool {
        (self.read_header_type(bdf) & 0x7F) == 0x01
    }

    /// Read secondary bus number (for bridges)
    pub fn read_secondary_bus(&self, bdf: PcieBdf) -> u8 {
        self.read8(bdf, cfg::SECONDARY_BUS)
    }

    /// Read BAR value
    pub fn read_bar(&self, bdf: PcieBdf, bar_num: u8) -> u32 {
        let reg = cfg::BAR0 + (bar_num as u16) * 4;
        self.read32(bdf, reg)
    }

    /// Write BAR value
    pub fn write_bar(&self, bdf: PcieBdf, bar_num: u8, value: u32) {
        let reg = cfg::BAR0 + (bar_num as u16) * 4;
        self.write32(bdf, reg, value);
    }

    /// Read BAR0 info (address and size)
    ///
    /// Returns (physical_address, size_in_bytes)
    /// Uses standard BAR probing: write all 1s, read back, count cleared bits
    pub fn read_bar0_info(&self, bdf: PcieBdf) -> (u64, u32) {
        // Read BAR0
        let bar0 = self.read_bar(bdf, 0);

        // Check if BAR is memory (bit 0 = 0) or I/O (bit 0 = 1)
        if (bar0 & 0x1) != 0 {
            // I/O BAR - not typically used for modern devices
            return (0, 0);
        }

        // Check BAR type: bits 2:1
        // 00 = 32-bit, 10 = 64-bit
        let is_64bit = ((bar0 >> 1) & 0x3) == 2;

        // Read current address
        let addr_lo = bar0 & !0xF;  // Mask out type bits
        let addr = if is_64bit {
            let bar1 = self.read_bar(bdf, 1);
            (addr_lo as u64) | ((bar1 as u64) << 32)
        } else {
            addr_lo as u64
        };

        // Probe size: write all 1s, read back, restore
        self.write_bar(bdf, 0, 0xFFFF_FFFF);
        let size_mask = self.read_bar(bdf, 0);
        self.write_bar(bdf, 0, bar0);  // Restore

        if is_64bit {
            self.write_bar(bdf, 1, 0xFFFF_FFFF);
            let _size_mask_hi = self.read_bar(bdf, 1);
            let bar1 = self.read_bar(bdf, 1);
            self.write_bar(bdf, 1, bar1);  // Restore
        }

        // Size is determined by lowest set bit in mask (after masking type bits)
        let size_bits = size_mask & !0xF;
        if size_bits == 0 {
            return (addr, 0);
        }

        // Size = ~(size_bits - 1) & size_bits, but simpler: size = lowest set bit
        let size = (!size_bits).wrapping_add(1) & size_bits;

        (addr, size)
    }
}
