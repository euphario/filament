//! Standard PCIe Constants
//!
//! Well-known vendor IDs, device IDs, and class codes.
//! These are standard across all PCIe implementations.

/// Well-known PCIe vendor IDs
pub mod vendor {
    pub const MEDIATEK: u16 = 0x14C3;
    pub const INTEL: u16 = 0x8086;
    pub const REALTEK: u16 = 0x10EC;
    pub const QUALCOMM: u16 = 0x17CB;
    pub const MICRON: u16 = 0x1344;
    pub const SAMSUNG: u16 = 0x144D;
    pub const NVIDIA: u16 = 0x10DE;
    pub const AMD: u16 = 0x1022;
    pub const BROADCOM: u16 = 0x14E4;
    pub const MARVELL: u16 = 0x1B4B;
}

/// Well-known PCIe device IDs
pub mod device {
    // MediaTek WiFi devices
    /// MediaTek MT7996 WiFi 7 (main)
    pub const MT7996: u16 = 0x7996;
    /// MediaTek MT7996 WiFi 7 endpoint A
    pub const MT7990: u16 = 0x7990;
    /// MediaTek MT7996 WiFi 7 endpoint B
    pub const MT7991: u16 = 0x7991;
    /// MediaTek MT7915 WiFi 6
    pub const MT7915: u16 = 0x7915;
    /// MediaTek MT7988 PCIe Root Complex
    pub const MT7988_RC: u16 = 0x7988;
}

/// PCIe class codes (base class, upper 8 bits of 24-bit class code)
pub mod class {
    /// Unclassified device
    pub const UNCLASSIFIED: u8 = 0x00;
    /// Mass storage controller
    pub const STORAGE: u8 = 0x01;
    /// Network controller
    pub const NETWORK: u8 = 0x02;
    /// Display controller
    pub const DISPLAY: u8 = 0x03;
    /// Multimedia controller
    pub const MULTIMEDIA: u8 = 0x04;
    /// Memory controller
    pub const MEMORY: u8 = 0x05;
    /// Bridge device
    pub const BRIDGE: u8 = 0x06;
    /// Simple communication controller
    pub const COMM: u8 = 0x07;
    /// Base system peripheral
    pub const SYSTEM: u8 = 0x08;
    /// Input device controller
    pub const INPUT: u8 = 0x09;
    /// Docking station
    pub const DOCKING: u8 = 0x0A;
    /// Processor
    pub const PROCESSOR: u8 = 0x0B;
    /// Serial bus controller
    pub const SERIAL: u8 = 0x0C;
    /// Wireless controller
    pub const WIRELESS: u8 = 0x0D;
    /// Intelligent controller
    pub const INTELLIGENT: u8 = 0x0E;
    /// Satellite communication controller
    pub const SATELLITE: u8 = 0x0F;
    /// Encryption controller
    pub const ENCRYPTION: u8 = 0x10;
    /// Signal processing controller
    pub const SIGNAL: u8 = 0x11;
}

/// Storage subclass codes
pub mod storage_subclass {
    pub const SCSI: u8 = 0x00;
    pub const IDE: u8 = 0x01;
    pub const FLOPPY: u8 = 0x02;
    pub const IPI: u8 = 0x03;
    pub const RAID: u8 = 0x04;
    pub const ATA: u8 = 0x05;
    pub const SATA: u8 = 0x06;
    pub const SAS: u8 = 0x07;
    pub const NVME: u8 = 0x08;
}

/// Bridge subclass codes
pub mod bridge_subclass {
    pub const HOST: u8 = 0x00;
    pub const ISA: u8 = 0x01;
    pub const EISA: u8 = 0x02;
    pub const MCA: u8 = 0x03;
    pub const PCI_PCI: u8 = 0x04;
    pub const PCMCIA: u8 = 0x05;
    pub const NUBUS: u8 = 0x06;
    pub const CARDBUS: u8 = 0x07;
    pub const RACEWAY: u8 = 0x08;
    pub const SEMI_PCI: u8 = 0x09;
    pub const INFINIBAND: u8 = 0x0A;
}
