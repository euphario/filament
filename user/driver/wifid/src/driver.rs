//! WiFi Driver Trait
//!
//! Abstraction layer for WiFi device drivers. Each chip family implements
//! this trait, allowing wifid to work with any supported hardware.

use pcie::PcieDeviceInfo;

/// WiFi driver initialization error
#[derive(Debug, Clone, Copy)]
pub enum DriverError {
    /// Device not supported by this driver
    NotSupported,
    /// Failed to map device memory
    MapFailed,
    /// Firmware not found
    FirmwareNotFound,
    /// Firmware loading failed
    FirmwareLoadFailed,
    /// Device initialization failed
    InitFailed,
}

/// WiFi device state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceState {
    /// Just discovered, not initialized
    Discovered,
    /// Memory mapped, registers accessible
    Mapped,
    /// Firmware loaded
    FirmwareLoaded,
    /// Fully initialized and ready
    Ready,
    /// Error state
    Error,
}

/// Information about a detected WiFi device
#[derive(Debug, Clone, Copy)]
pub struct WifiDeviceInfo {
    pub vendor_id: u16,
    pub device_id: u16,
    pub chip_name: &'static str,
}

/// Trait for WiFi device drivers
pub trait WifiDriver {
    /// Check if this driver supports the given device
    fn supports(info: &PcieDeviceInfo) -> bool where Self: Sized;

    /// Create driver instance for the device
    fn probe(info: &PcieDeviceInfo) -> Result<Self, DriverError> where Self: Sized;

    /// Get device info
    fn device_info(&self) -> WifiDeviceInfo;

    /// Get current device state
    fn state(&self) -> DeviceState;

    /// Check if firmware is available
    fn firmware_available(&self) -> bool;

    /// Load firmware to device
    fn load_firmware(&mut self) -> Result<(), DriverError>;

    /// Initialize device after firmware load
    fn init(&mut self) -> Result<(), DriverError>;

    /// Get driver name
    fn driver_name(&self) -> &'static str;
}

/// Enum of all supported drivers (static dispatch, no allocator needed)
pub enum AnyWifiDriver {
    Mt7996(super::drivers::mt7996::Mt7996Driver),
    // Add more variants as drivers are added
}

impl AnyWifiDriver {
    /// Try to find a driver for the given device
    pub fn probe(info: &PcieDeviceInfo) -> Option<Self> {
        use super::drivers::mt7996::Mt7996Driver;

        if Mt7996Driver::supports(info) {
            if let Ok(driver) = Mt7996Driver::probe(info) {
                return Some(AnyWifiDriver::Mt7996(driver));
            }
        }

        None
    }

    pub fn device_info(&self) -> WifiDeviceInfo {
        match self {
            AnyWifiDriver::Mt7996(d) => d.device_info(),
        }
    }

    pub fn state(&self) -> DeviceState {
        match self {
            AnyWifiDriver::Mt7996(d) => d.state(),
        }
    }

    pub fn firmware_available(&self) -> bool {
        match self {
            AnyWifiDriver::Mt7996(d) => d.firmware_available(),
        }
    }

    pub fn load_firmware(&mut self) -> Result<(), DriverError> {
        match self {
            AnyWifiDriver::Mt7996(d) => d.load_firmware(),
        }
    }

    pub fn init(&mut self) -> Result<(), DriverError> {
        match self {
            AnyWifiDriver::Mt7996(d) => d.init(),
        }
    }

    pub fn driver_name(&self) -> &'static str {
        match self {
            AnyWifiDriver::Mt7996(d) => d.driver_name(),
        }
    }
}
