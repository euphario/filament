//! USB Device Class Trait
//!
//! Abstraction layer for USB device classes. Each device class (MSC, Hub, etc.)
//! implements this trait, allowing usbd to work with any supported device type.

#![allow(dead_code)]  // Types reserved for future use

/// USB device class error
#[derive(Debug, Clone, Copy)]
pub enum DeviceError {
    /// Device not supported by this driver
    NotSupported,
    /// Failed to configure device
    ConfigFailed,
    /// Communication error
    CommError,
    /// Device not ready
    NotReady,
    /// Transfer timeout
    Timeout,
}

/// USB device state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceState {
    /// Device detected but not configured
    Detected,
    /// Device configured and ready
    Ready,
    /// Device in error state
    Error,
}

/// Information about a USB device
#[derive(Debug, Clone, Copy)]
pub struct UsbDeviceInfo {
    pub slot_id: u32,
    pub vendor_id: u16,
    pub product_id: u16,
    pub device_class: u8,
    pub device_subclass: u8,
    pub device_protocol: u8,
    pub device_type: &'static str,
}

/// Trait for USB device class drivers
///
/// This trait abstracts USB device classes (MSC, Hub, HID, etc.) allowing
/// the USB daemon to work with any supported device type.
pub trait UsbDeviceClass {
    /// Check if this driver supports the given device descriptor
    fn supports(class: u8, subclass: u8, protocol: u8) -> bool where Self: Sized;

    /// Get device info
    fn device_info(&self) -> UsbDeviceInfo;

    /// Get current device state
    fn state(&self) -> DeviceState;

    /// Get device type name
    fn device_type(&self) -> &'static str;

    /// Get slot ID
    fn slot_id(&self) -> u32;
}

// Note: The actual MscDevice and HubDevice implementations remain in main.rs
// for now, as they are tightly coupled with UsbDriver's ring management.
// This trait provides the abstraction layer for future separation.

/// Enum of all supported USB device classes (static dispatch, no allocator needed)
pub enum AnyUsbDevice {
    Msc(super::MscDeviceInfo),
    Hub(super::HubDeviceInfo),
}

impl AnyUsbDevice {
    /// Get device info
    pub fn device_info(&self) -> UsbDeviceInfo {
        match self {
            AnyUsbDevice::Msc(d) => UsbDeviceInfo {
                slot_id: d.slot_id,
                vendor_id: 0,  // Could be stored if needed
                product_id: 0,
                device_class: 0x08,  // Mass Storage
                device_subclass: 0x06,  // SCSI
                device_protocol: 0x50,  // BBB
                device_type: "Mass Storage",
            },
            AnyUsbDevice::Hub(d) => UsbDeviceInfo {
                slot_id: d.slot_id,
                vendor_id: 0,
                product_id: 0,
                device_class: 0x09,  // Hub
                device_subclass: 0,
                device_protocol: 0,
                device_type: "Hub",
            },
        }
    }

    /// Get current device state
    pub fn state(&self) -> DeviceState {
        // For now, if we have the device info, it's ready
        DeviceState::Ready
    }

    /// Get device type name
    pub fn device_type(&self) -> &'static str {
        match self {
            AnyUsbDevice::Msc(_) => "Mass Storage",
            AnyUsbDevice::Hub(_) => "Hub",
        }
    }

    /// Get slot ID
    pub fn slot_id(&self) -> u32 {
        match self {
            AnyUsbDevice::Msc(d) => d.slot_id,
            AnyUsbDevice::Hub(d) => d.slot_id,
        }
    }

    /// Check if this is an MSC device
    pub fn is_msc(&self) -> bool {
        matches!(self, AnyUsbDevice::Msc(_))
    }

    /// Check if this is a Hub device
    pub fn is_hub(&self) -> bool {
        matches!(self, AnyUsbDevice::Hub(_))
    }

    /// Get as MSC device if applicable
    pub fn as_msc(&self) -> Option<&super::MscDeviceInfo> {
        match self {
            AnyUsbDevice::Msc(d) => Some(d),
            _ => None,
        }
    }

    /// Get as MSC device mutably if applicable
    pub fn as_msc_mut(&mut self) -> Option<&mut super::MscDeviceInfo> {
        match self {
            AnyUsbDevice::Msc(d) => Some(d),
            _ => None,
        }
    }

    /// Get as Hub device if applicable
    pub fn as_hub(&self) -> Option<&super::HubDeviceInfo> {
        match self {
            AnyUsbDevice::Hub(d) => Some(d),
            _ => None,
        }
    }

    /// Get as Hub device mutably if applicable
    pub fn as_hub_mut(&mut self) -> Option<&mut super::HubDeviceInfo> {
        match self {
            AnyUsbDevice::Hub(d) => Some(d),
            _ => None,
        }
    }
}

/// Check if a device class/subclass/protocol matches MSC BBB
pub fn is_msc_bbb(class: u8, subclass: u8, protocol: u8) -> bool {
    class == 0x08 && subclass == 0x06 && protocol == 0x50
}

/// Check if a device class matches Hub
pub fn is_hub_class(class: u8) -> bool {
    class == 0x09
}
