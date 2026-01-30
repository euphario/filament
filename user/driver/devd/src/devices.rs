//! Device Registry
//!
//! Tracks devices registered by drivers for the hierarchical query model.
//! Uses trait-based design for testability.

use userlib::query::{DeviceRegister, DeviceEntry, state};

// =============================================================================
// Constants
// =============================================================================

pub const MAX_DEVICES: usize = 32;
pub const MAX_PATH_LEN: usize = 64;

// =============================================================================
// Device
// =============================================================================

/// A registered device
#[derive(Clone, Copy)]
pub struct Device {
    /// Device ID assigned by devd
    pub id: u32,
    /// Device class
    pub device_class: u16,
    /// Device subclass
    pub device_subclass: u16,
    /// Vendor ID
    pub vendor_id: u16,
    /// Product ID
    pub product_id: u16,
    /// Current state
    pub state: u8,
    /// Service index that owns this device
    pub owner_service_idx: u8,
    /// Device path (e.g., "/usb/port1/msc0")
    pub path: [u8; MAX_PATH_LEN],
    /// Path length
    pub path_len: u8,
}

impl Device {
    pub const fn empty() -> Self {
        Self {
            id: 0,
            device_class: 0,
            device_subclass: 0,
            vendor_id: 0,
            product_id: 0,
            state: state::DISCOVERED,
            owner_service_idx: 0,
            path: [0; MAX_PATH_LEN],
            path_len: 0,
        }
    }

    /// Get device path as slice
    pub fn path(&self) -> &[u8] {
        &self.path[..self.path_len as usize]
    }

    /// Convert to DeviceEntry for query responses
    pub fn to_entry(&self) -> DeviceEntry {
        DeviceEntry {
            device_id: self.id,
            device_class: self.device_class,
            device_subclass: self.device_subclass,
            vendor_id: self.vendor_id,
            product_id: self.product_id,
            state: self.state,
            _pad: [0; 3],
        }
    }
}

// =============================================================================
// DeviceStore Trait
// =============================================================================

/// Device registry trait - enables testing with mocks
pub trait DeviceStore {
    /// Register a device, returns assigned ID
    fn register(
        &mut self,
        info: &DeviceRegister,
        path: &[u8],
        owner: u8,
    ) -> Option<u32>;

    /// Unregister a device by ID
    fn unregister(&mut self, device_id: u32) -> bool;

    /// Update device state
    fn update_state(&mut self, device_id: u32, new_state: u8) -> bool;

    /// Get device by ID
    fn get(&self, device_id: u32) -> Option<&Device>;

    /// Count devices, optionally filtered by class
    fn count(&self, class_filter: Option<u16>) -> usize;

    /// Iterate devices (for list command)
    fn for_each<F: FnMut(&Device)>(&self, class_filter: Option<u16>, f: F);

    /// Remove all devices owned by a service (for crash cleanup)
    fn remove_by_owner(&mut self, owner: u8) -> usize;
}

// =============================================================================
// DeviceRegistry Implementation
// =============================================================================

/// Concrete implementation of DeviceStore
pub struct DeviceRegistry {
    devices: [Option<Device>; MAX_DEVICES],
    next_id: u32,
    count: usize,
}

impl DeviceRegistry {
    pub const fn new() -> Self {
        Self {
            devices: [const { None }; MAX_DEVICES],
            next_id: 1,
            count: 0,
        }
    }

    /// Find slot by device ID
    fn find_slot(&self, device_id: u32) -> Option<usize> {
        (0..MAX_DEVICES).find(|&i| {
            self.devices[i]
                .as_ref()
                .map(|d| d.id == device_id)
                .unwrap_or(false)
        })
    }

    /// Find empty slot
    fn find_empty_slot(&self) -> Option<usize> {
        (0..MAX_DEVICES).find(|&i| self.devices[i].is_none())
    }
}

impl DeviceStore for DeviceRegistry {
    fn register(
        &mut self,
        info: &DeviceRegister,
        path: &[u8],
        owner: u8,
    ) -> Option<u32> {
        let slot = self.find_empty_slot()?;

        let mut device = Device::empty();
        device.id = self.next_id;
        device.device_class = info.device_class;
        device.device_subclass = info.device_subclass;
        device.vendor_id = info.vendor_id;
        device.product_id = info.product_id;
        device.state = state::DISCOVERED;
        device.owner_service_idx = owner;

        // Copy path
        let path_len = path.len().min(MAX_PATH_LEN);
        device.path[..path_len].copy_from_slice(&path[..path_len]);
        device.path_len = path_len as u8;

        self.devices[slot] = Some(device);
        self.next_id += 1;
        self.count += 1;

        Some(device.id)
    }

    fn unregister(&mut self, device_id: u32) -> bool {
        if let Some(slot) = self.find_slot(device_id) {
            self.devices[slot] = None;
            self.count = self.count.saturating_sub(1);
            true
        } else {
            false
        }
    }

    fn update_state(&mut self, device_id: u32, new_state: u8) -> bool {
        if let Some(slot) = self.find_slot(device_id) {
            if let Some(device) = &mut self.devices[slot] {
                device.state = new_state;
                return true;
            }
        }
        false
    }

    fn get(&self, device_id: u32) -> Option<&Device> {
        self.find_slot(device_id)
            .and_then(|slot| self.devices[slot].as_ref())
    }

    fn count(&self, class_filter: Option<u16>) -> usize {
        match class_filter {
            Some(class) => self.devices.iter()
                .flatten()
                .filter(|d| d.device_class == class)
                .count(),
            None => self.count,
        }
    }

    fn for_each<F: FnMut(&Device)>(&self, class_filter: Option<u16>, mut f: F) {
        for device in self.devices.iter().flatten() {
            match class_filter {
                Some(class) if device.device_class != class => continue,
                _ => f(device),
            }
        }
    }

    fn remove_by_owner(&mut self, owner: u8) -> usize {
        let mut removed = 0;
        for slot in &mut self.devices {
            if let Some(device) = slot {
                if device.owner_service_idx == owner {
                    *slot = None;
                    removed += 1;
                }
            }
        }
        self.count = self.count.saturating_sub(removed);
        removed
    }

}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use userlib::query::{QueryHeader, msg, class};

    fn make_register(device_class: u16, vendor_id: u16, product_id: u16) -> DeviceRegister {
        DeviceRegister {
            header: QueryHeader::new(msg::REGISTER_DEVICE, 1),
            device_class,
            device_subclass: 0,
            vendor_id,
            product_id,
            path_len: 0,
            name_len: 0,
        }
    }

    #[test]
    fn test_register_unregister() {
        let mut registry = DeviceRegistry::new();

        let info = make_register(class::MASS_STORAGE, 0x1234, 0x5678);
        let id = registry.register(&info, b"/usb/msc0", 0).unwrap();

        assert_eq!(id, 1);
        assert_eq!(registry.count(None), 1);

        let device = registry.get(id).unwrap();
        assert_eq!(device.vendor_id, 0x1234);
        assert_eq!(device.product_id, 0x5678);
        assert_eq!(device.path(), b"/usb/msc0");

        assert!(registry.unregister(id));
        assert_eq!(registry.count(None), 0);
        assert!(registry.get(id).is_none());
    }

    #[test]
    fn test_update_state() {
        let mut registry = DeviceRegistry::new();

        let info = make_register(class::MASS_STORAGE, 0x1234, 0x5678);
        let id = registry.register(&info, b"", 0).unwrap();

        assert_eq!(registry.get(id).unwrap().state, state::DISCOVERED);

        assert!(registry.update_state(id, state::OPERATIONAL));
        assert_eq!(registry.get(id).unwrap().state, state::OPERATIONAL);
    }

    #[test]
    fn test_class_filter() {
        let mut registry = DeviceRegistry::new();

        let msc = make_register(class::MASS_STORAGE, 0x1234, 0x5678);
        let hub = make_register(class::HUB, 0x1234, 0x9999);

        registry.register(&msc, b"", 0);
        registry.register(&hub, b"", 0);
        registry.register(&msc, b"", 0);

        assert_eq!(registry.count(None), 3);
        assert_eq!(registry.count(Some(class::MASS_STORAGE)), 2);
        assert_eq!(registry.count(Some(class::HUB)), 1);
        assert_eq!(registry.count(Some(class::HID)), 0);
    }

    #[test]
    fn test_remove_by_owner() {
        let mut registry = DeviceRegistry::new();

        let info = make_register(class::MASS_STORAGE, 0x1234, 0x5678);

        registry.register(&info, b"", 1); // owner 1
        registry.register(&info, b"", 2); // owner 2
        registry.register(&info, b"", 1); // owner 1

        assert_eq!(registry.count(None), 3);

        let removed = registry.remove_by_owner(1);
        assert_eq!(removed, 2);
        assert_eq!(registry.count(None), 1);
    }
}
