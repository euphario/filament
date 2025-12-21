//! PCI Configuration Space Constants and Helpers

/// Standard PCI configuration space registers
pub mod reg {
    pub const VENDOR_ID: u16 = 0x00;
    pub const DEVICE_ID: u16 = 0x02;
    pub const COMMAND: u16 = 0x04;
    pub const STATUS: u16 = 0x06;
    pub const REVISION_ID: u16 = 0x08;
    pub const CLASS_CODE: u16 = 0x09;
    pub const CACHE_LINE: u16 = 0x0C;
    pub const LATENCY: u16 = 0x0D;
    pub const HEADER_TYPE: u16 = 0x0E;
    pub const BIST: u16 = 0x0F;
    pub const BAR0: u16 = 0x10;
    pub const BAR1: u16 = 0x14;
    pub const BAR2: u16 = 0x18;
    pub const BAR3: u16 = 0x1C;
    pub const BAR4: u16 = 0x20;
    pub const BAR5: u16 = 0x24;
    pub const CARDBUS_CIS: u16 = 0x28;
    pub const SUBSYSTEM_VENDOR: u16 = 0x2C;
    pub const SUBSYSTEM_ID: u16 = 0x2E;
    pub const ROM_BASE: u16 = 0x30;
    pub const CAP_PTR: u16 = 0x34;
    pub const INTERRUPT_LINE: u16 = 0x3C;
    pub const INTERRUPT_PIN: u16 = 0x3D;
    pub const MIN_GNT: u16 = 0x3E;
    pub const MAX_LAT: u16 = 0x3F;

    // Bridge-specific (Type 1 header)
    pub const PRIMARY_BUS: u16 = 0x18;
    pub const SECONDARY_BUS: u16 = 0x19;
    pub const SUBORDINATE_BUS: u16 = 0x1A;
}

/// PCI Command register bits
pub mod cmd {
    pub const IO_SPACE: u16 = 1 << 0;
    pub const MEMORY_SPACE: u16 = 1 << 1;
    pub const BUS_MASTER: u16 = 1 << 2;
    pub const SPECIAL_CYCLES: u16 = 1 << 3;
    pub const MEM_WR_INVAL: u16 = 1 << 4;
    pub const VGA_SNOOP: u16 = 1 << 5;
    pub const PARITY_ERR: u16 = 1 << 6;
    pub const SERR: u16 = 1 << 8;
    pub const FAST_B2B: u16 = 1 << 9;
    pub const INTX_DISABLE: u16 = 1 << 10;
}

/// PCI Capability IDs
pub mod cap {
    pub const PM: u8 = 0x01;        // Power Management
    pub const AGP: u8 = 0x02;       // AGP
    pub const VPD: u8 = 0x03;       // Vital Product Data
    pub const SLOT_ID: u8 = 0x04;   // Slot Identification
    pub const MSI: u8 = 0x05;       // Message Signaled Interrupts
    pub const PCIE: u8 = 0x10;      // PCI Express
    pub const MSIX: u8 = 0x11;      // MSI-X
}

/// MSI capability structure
pub struct MsiCapability {
    /// Capability offset in config space
    pub offset: u8,
    /// Message control register value
    pub control: u16,
    /// Is 64-bit address capable?
    pub is_64bit: bool,
    /// Supports per-vector masking?
    pub per_vector_mask: bool,
    /// Maximum vectors (log2)
    pub max_vectors_log2: u8,
}

impl MsiCapability {
    /// Read MSI capability from config space
    pub fn read<F>(offset: u8, read32: F) -> Self
    where
        F: Fn(u16) -> u32,
    {
        let control = (read32(offset as u16) >> 16) as u16;
        let is_64bit = (control & (1 << 7)) != 0;
        let per_vector_mask = (control & (1 << 8)) != 0;
        let max_vectors_log2 = ((control >> 1) & 0x7) as u8;

        Self {
            offset,
            control,
            is_64bit,
            per_vector_mask,
            max_vectors_log2,
        }
    }

    /// Get maximum supported vectors
    pub fn max_vectors(&self) -> u8 {
        1 << self.max_vectors_log2
    }
}

/// MSI-X capability structure
pub struct MsixCapability {
    /// Capability offset in config space
    pub offset: u8,
    /// Table size (number of entries - 1)
    pub table_size: u16,
    /// Table BAR
    pub table_bar: u8,
    /// Table offset within BAR
    pub table_offset: u32,
    /// PBA BAR
    pub pba_bar: u8,
    /// PBA offset within BAR
    pub pba_offset: u32,
}

impl MsixCapability {
    /// Read MSI-X capability from config space
    pub fn read<F>(offset: u8, read32: F) -> Self
    where
        F: Fn(u16) -> u32,
    {
        let control = read32(offset as u16);
        let table_size = ((control >> 16) & 0x7FF) as u16;

        let table_reg = read32((offset + 4) as u16);
        let table_bar = (table_reg & 0x7) as u8;
        let table_offset = table_reg & !0x7;

        let pba_reg = read32((offset + 8) as u16);
        let pba_bar = (pba_reg & 0x7) as u8;
        let pba_offset = pba_reg & !0x7;

        Self {
            offset,
            table_size,
            table_bar,
            table_offset,
            pba_bar,
            pba_offset,
        }
    }

    /// Get number of MSI-X entries
    pub fn entry_count(&self) -> u16 {
        self.table_size + 1
    }
}

/// PCIe capability structure
pub struct PcieCapability {
    /// Capability offset
    pub offset: u8,
    /// Device type
    pub device_type: PcieDeviceType,
    /// Max link speed
    pub max_link_speed: u8,
    /// Max link width
    pub max_link_width: u8,
    /// Current link speed
    pub current_link_speed: u8,
    /// Current link width
    pub current_link_width: u8,
}

/// PCIe device types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PcieDeviceType {
    Endpoint,
    LegacyEndpoint,
    RootPort,
    UpstreamSwitch,
    DownstreamSwitch,
    PcieToPciBridge,
    PciToPcieBridge,
    RootComplexIntegrated,
    RootComplexEventCollector,
    Unknown(u8),
}

impl PcieCapability {
    /// Read PCIe capability from config space
    pub fn read<F>(offset: u8, read32: F) -> Self
    where
        F: Fn(u16) -> u32,
    {
        let caps = read32(offset as u16);
        let device_type_bits = ((caps >> 20) & 0xF) as u8;

        let device_type = match device_type_bits {
            0 => PcieDeviceType::Endpoint,
            1 => PcieDeviceType::LegacyEndpoint,
            4 => PcieDeviceType::RootPort,
            5 => PcieDeviceType::UpstreamSwitch,
            6 => PcieDeviceType::DownstreamSwitch,
            7 => PcieDeviceType::PcieToPciBridge,
            8 => PcieDeviceType::PciToPcieBridge,
            9 => PcieDeviceType::RootComplexIntegrated,
            10 => PcieDeviceType::RootComplexEventCollector,
            x => PcieDeviceType::Unknown(x),
        };

        let link_caps = read32((offset + 0x0C) as u16);
        let max_link_speed = (link_caps & 0xF) as u8;
        let max_link_width = ((link_caps >> 4) & 0x3F) as u8;

        let link_status = read32((offset + 0x12) as u16);
        let current_link_speed = (link_status & 0xF) as u8;
        let current_link_width = ((link_status >> 4) & 0x3F) as u8;

        Self {
            offset,
            device_type,
            max_link_speed,
            max_link_width,
            current_link_speed,
            current_link_width,
        }
    }
}

/// Placeholder types for module exports
pub struct PciConfigSpace;
pub struct PciCapability;
