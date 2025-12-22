//! MT7996 WiFi Driver
//!
//! Implements WifiDriver trait for MediaTek MT7996/MT7992/MT7990 chips.
//! Currently links directly to the mt7996 library; can be replaced with
//! IPC calls to a separate mt7996d process.

use crate::driver::{WifiDriver, DriverError, DeviceState, WifiDeviceInfo};
use pcie::PcieDeviceInfo;
use mt7996::{Mt7996Device, mcu::Mcu, firmware, regs};

/// MT7996 driver implementation
pub struct Mt7996Driver {
    device: Mt7996Device,
    device_id: u16,
    state: DeviceState,
}

impl Mt7996Driver {
    /// Get chip name from device ID
    fn chip_name(device_id: u16) -> &'static str {
        match device_id {
            regs::device_id::MT7996 | regs::device_id::MT7996_2 => "MT7996",
            regs::device_id::MT7992 | regs::device_id::MT7992_2 => "MT7992",
            regs::device_id::MT7990 | regs::device_id::MT7990_2 => "MT7990",
            _ => "Unknown",
        }
    }

    /// Check if device ID is MT7996 family
    fn is_supported_device(device_id: u16) -> bool {
        matches!(device_id,
            regs::device_id::MT7996 |
            regs::device_id::MT7996_2 |
            regs::device_id::MT7992 |
            regs::device_id::MT7992_2 |
            regs::device_id::MT7990 |
            regs::device_id::MT7990_2
        )
    }
}

impl WifiDriver for Mt7996Driver {
    fn supports(info: &PcieDeviceInfo) -> bool {
        // Check vendor and device ID
        info.vendor_id == pcie::consts::vendor::MEDIATEK
            && Self::is_supported_device(info.device_id)
    }

    fn probe(info: &PcieDeviceInfo) -> Result<Self, DriverError> {
        let device = Mt7996Device::init_from_info(info)
            .map_err(|_| DriverError::MapFailed)?;

        Ok(Self {
            device,
            device_id: info.device_id,
            state: DeviceState::Mapped,
        })
    }

    fn device_info(&self) -> WifiDeviceInfo {
        WifiDeviceInfo {
            vendor_id: pcie::consts::vendor::MEDIATEK,
            device_id: self.device_id,
            chip_name: Self::chip_name(self.device_id),
        }
    }

    fn state(&self) -> DeviceState {
        self.state
    }

    fn firmware_available(&self) -> bool {
        // Check ramfs first, then USB
        firmware::check_firmware_files() || firmware::check_usb_available()
    }

    fn load_firmware(&mut self) -> Result<(), DriverError> {
        if self.state != DeviceState::Mapped {
            return Err(DriverError::InitFailed);
        }

        // Create MCU interface (temporary - lifetime tied to this scope)
        let mut mcu = Mcu::new(&self.device);

        // TODO: Try USB firmware first if ramfs not available
        // For now, just use the existing ramfs loader
        mcu.load_firmware()
            .map_err(|_| DriverError::FirmwareLoadFailed)?;

        self.state = DeviceState::FirmwareLoaded;
        Ok(())
    }

    fn init(&mut self) -> Result<(), DriverError> {
        if self.state != DeviceState::FirmwareLoaded {
            return Err(DriverError::InitFailed);
        }

        // TODO: Additional initialization after firmware load
        // - Configure MAC
        // - Set up queues
        // - Enable interrupts

        self.state = DeviceState::Ready;
        Ok(())
    }

    fn driver_name(&self) -> &'static str {
        "mt7996"
    }
}

// Extra methods specific to MT7996
impl Mt7996Driver {
    /// Get raw register value (for debugging)
    pub fn read_reg(&self, offset: u32) -> u32 {
        self.device.read32_raw(offset)
    }

    /// Get BAR0 info
    pub fn bar0_info(&self) -> (u64, u64) {
        (self.device.bar0_virt(), self.device.bar0_size())
    }

    /// Get device variant
    pub fn variant(&self) -> mt7996::ChipVariant {
        self.device.variant()
    }
}
