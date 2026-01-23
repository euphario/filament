//! PCI Device Registry
//!
//! Tracks discovered PCI devices and their capabilities.

use super::{PciError, PciResult, MAX_PCI_DEVICES};

/// PCI Bus/Device/Function address
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct PciBdf {
    /// Bus number (0-255)
    pub bus: u8,
    /// Device number (0-31)
    pub device: u8,
    /// Function number (0-7)
    pub function: u8,
    /// Port number (for multi-port controllers)
    pub port: u8,
}

impl PciBdf {
    /// Create new BDF address
    pub const fn new(bus: u8, device: u8, function: u8) -> Self {
        Self { bus, device, function, port: 0 }
    }

    /// Create BDF with port
    pub const fn with_port(port: u8, bus: u8, device: u8, function: u8) -> Self {
        Self { bus, device, function, port }
    }

    /// Convert to u32 for syscall passing
    /// Format: port(8) | bus(8) | device(5) | function(3)
    pub const fn to_u32(&self) -> u32 {
        ((self.port as u32) << 24)
            | ((self.bus as u32) << 16)
            | ((self.device as u32) << 8)
            | (self.function as u32)
    }

    /// Create from u32
    pub const fn from_u32(val: u32) -> Self {
        Self {
            port: ((val >> 24) & 0xFF) as u8,
            bus: ((val >> 16) & 0xFF) as u8,
            device: ((val >> 8) & 0x1F) as u8,
            function: (val & 0x07) as u8,
        }
    }

    /// Check if valid
    pub const fn is_valid(&self) -> bool {
        self.device <= 31 && self.function <= 7
    }
}

/// A discovered PCI device
#[derive(Debug, Clone, Copy)]
pub struct PciDevice {
    /// Bus/Device/Function address (immutable after creation)
    pub bdf: PciBdf,
    /// Vendor ID (immutable after creation)
    pub vendor_id: u16,
    /// Device ID (immutable after creation)
    pub device_id: u16,
    /// Class code (24-bit: class << 16 | subclass << 8 | prog_if)
    pub class_code: u32,
    /// Revision ID
    pub revision: u8,
    /// Header type
    pub header_type: u8,
    /// Is this a bridge?
    pub is_bridge: bool,
    /// BAR0 physical address
    pub bar0_addr: u64,
    /// BAR0 size
    pub bar0_size: u64,
    /// MSI capability offset (0 if not supported)
    pub msi_cap: u8,
    /// MSI-X capability offset (0 if not supported)
    pub msix_cap: u8,
    /// Owning process ID (0 = unclaimed)
    /// Private to enforce access control - use owner() and set_owner()
    owner_pid: u32,
}

impl PciDevice {
    /// Create empty device
    pub const fn empty() -> Self {
        Self {
            bdf: PciBdf::new(0, 0, 0),
            vendor_id: 0xFFFF,
            device_id: 0xFFFF,
            class_code: 0,
            revision: 0,
            header_type: 0,
            is_bridge: false,
            bar0_addr: 0,
            bar0_size: 0,
            msi_cap: 0,
            msix_cap: 0,
            owner_pid: 0,
        }
    }

    /// Check if slot is empty
    pub const fn is_empty(&self) -> bool {
        self.vendor_id == 0xFFFF
    }

    /// Get base class
    pub const fn base_class(&self) -> u8 {
        ((self.class_code >> 16) & 0xFF) as u8
    }

    /// Get subclass
    pub const fn subclass(&self) -> u8 {
        ((self.class_code >> 8) & 0xFF) as u8
    }

    /// Get human-readable class name
    pub fn class_name(&self) -> &'static str {
        match (self.base_class(), self.subclass()) {
            (0x00, _) => "Unclassified",
            (0x01, 0x00) => "SCSI Controller",
            (0x01, 0x01) => "IDE Controller",
            (0x01, 0x06) => "SATA Controller",
            (0x01, 0x08) => "NVMe Controller",
            (0x01, _) => "Storage Controller",
            (0x02, 0x00) => "Ethernet Controller",
            (0x02, 0x80) => "WiFi Controller",
            (0x02, _) => "Network Controller",
            (0x03, _) => "Display Controller",
            (0x04, _) => "Multimedia Controller",
            (0x05, _) => "Memory Controller",
            (0x06, 0x00) => "Host Bridge",
            (0x06, 0x04) => "PCI Bridge",
            (0x06, _) => "Bridge Device",
            (0x07, _) => "Communication Controller",
            (0x08, _) => "System Peripheral",
            (0x0C, 0x03) => "USB Controller",
            (0x0C, _) => "Serial Bus Controller",
            (0x0D, 0x00) => "IRDA Controller",
            (0x0D, 0x11) => "Bluetooth Controller",
            (0x0D, _) => "Wireless Controller",
            _ => "Unknown Device",
        }
    }

    /// Check if device supports MSI
    pub const fn has_msi(&self) -> bool {
        self.msi_cap != 0
    }

    /// Check if device supports MSI-X
    pub const fn has_msix(&self) -> bool {
        self.msix_cap != 0
    }

    /// Get the owning process ID (0 = unclaimed)
    pub const fn owner(&self) -> u32 {
        self.owner_pid
    }

    /// Check if device is claimed by a process
    pub const fn is_claimed(&self) -> bool {
        self.owner_pid != 0
    }

    /// Set the owner (kernel-internal, use claim/release methods on registry)
    pub(crate) fn set_owner(&mut self, pid: u32) {
        self.owner_pid = pid;
    }
}

/// Registry of discovered PCI devices
pub struct PciDeviceRegistry {
    devices: [PciDevice; MAX_PCI_DEVICES],
    count: usize,
}

impl PciDeviceRegistry {
    /// Create empty registry
    pub const fn new() -> Self {
        Self {
            devices: [PciDevice::empty(); MAX_PCI_DEVICES],
            count: 0,
        }
    }

    /// Register a device
    pub fn register(&mut self, device: PciDevice) -> PciResult<()> {
        // Check for duplicates
        for i in 0..self.count {
            if self.devices[i].bdf == device.bdf {
                // Update existing
                self.devices[i] = device;
                return Ok(());
            }
        }

        // Add new
        if self.count >= MAX_PCI_DEVICES {
            return Err(PciError::OutOfMemory);
        }

        self.devices[self.count] = device;
        self.count += 1;
        Ok(())
    }

    /// Find device by BDF
    pub fn find_by_bdf(&self, bdf: PciBdf) -> Option<&PciDevice> {
        for i in 0..self.count {
            if self.devices[i].bdf == bdf {
                return Some(&self.devices[i]);
            }
        }
        None
    }

    /// Find device by vendor/device ID
    pub fn find_by_id(&self, vendor_id: u16, device_id: u16) -> Option<&PciDevice> {
        for i in 0..self.count {
            let dev = &self.devices[i];
            if dev.vendor_id == vendor_id && dev.device_id == device_id {
                return Some(dev);
            }
        }
        None
    }

    /// Find all devices matching vendor ID
    pub fn find_all_by_vendor(&self, vendor_id: u16) -> impl Iterator<Item = &PciDevice> {
        self.devices[..self.count]
            .iter()
            .filter(move |d| d.vendor_id == vendor_id)
    }

    /// Find all devices matching class
    pub fn find_all_by_class(&self, base_class: u8, subclass: u8) -> impl Iterator<Item = &PciDevice> {
        self.devices[..self.count]
            .iter()
            .filter(move |d| d.base_class() == base_class && d.subclass() == subclass)
    }

    /// Get device by index
    pub fn get(&self, index: usize) -> Option<&PciDevice> {
        if index < self.count {
            Some(&self.devices[index])
        } else {
            None
        }
    }

    /// Get mutable device by BDF
    pub fn get_mut(&mut self, bdf: PciBdf) -> Option<&mut PciDevice> {
        for i in 0..self.count {
            if self.devices[i].bdf == bdf {
                return Some(&mut self.devices[i]);
            }
        }
        None
    }

    /// Get device count
    pub fn len(&self) -> usize {
        self.count
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Claim a device for a process
    pub fn claim(&mut self, bdf: PciBdf, pid: u32) -> PciResult<()> {
        let dev = self.get_mut(bdf).ok_or(PciError::NotFound)?;
        let current_owner = dev.owner();
        if current_owner != 0 && current_owner != pid {
            return Err(PciError::PermissionDenied);
        }
        dev.set_owner(pid);
        Ok(())
    }

    /// Release a device
    pub fn release(&mut self, bdf: PciBdf, pid: u32) -> PciResult<()> {
        let dev = self.get_mut(bdf).ok_or(PciError::NotFound)?;
        if dev.owner() != pid {
            return Err(PciError::PermissionDenied);
        }
        dev.set_owner(0);
        Ok(())
    }

    /// Release all devices owned by a process
    /// Returns BDFs of released devices (for MSI cleanup)
    pub fn release_all(&mut self, pid: u32) -> [Option<PciBdf>; MAX_PCI_DEVICES] {
        let mut released = [None; MAX_PCI_DEVICES];
        for i in 0..self.count {
            if self.devices[i].owner() == pid {
                released[i] = Some(self.devices[i].bdf);
                self.devices[i].set_owner(0);
            }
        }
        released
    }
}
