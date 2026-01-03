//! MT7996 WiFi Driver
//!
//! Implements WifiDriver trait for MediaTek MT7996/MT7992/MT7990 chips.
//! Currently links directly to the mt7996 library; can be replaced with
//! IPC calls to a separate mt7996d process.
//!
//! ## Dual Interface Support
//!
//! MT7996 family chips use two PCIe devices:
//! - Primary (HIF1): 0x7990, 0x7992, 0x7993 - main interface
//! - Secondary (HIF2): 0x7991, 0x799a, 0x799b - for tri-band operation
//!
//! Both interfaces must be initialized for proper operation.

use crate::driver::{WifiDriver, DriverError, DeviceState, WifiDeviceInfo};
use pcie::PcieDeviceInfo;
use mt7996::{Mt7996Device, mcu::Mcu, firmware, regs};

/// MT7996 driver implementation
pub struct Mt7996Driver {
    device: Mt7996Device,
    device_id: u16,
    state: DeviceState,
    /// Whether HIF2 interface was found and initialized
    has_hif2: bool,
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

    /// Check if device ID is MT7996 family PRIMARY device
    /// HIF2/secondary devices should NOT be probed - they don't need firmware loading
    pub fn is_supported_device(device_id: u16) -> bool {
        // Only support primary devices, not HIF2/secondary
        matches!(device_id,
            regs::device_id::MT7996 |  // 0x7990 - primary
            regs::device_id::MT7992 |  // 0x7992 - primary
            regs::device_id::MT7990    // 0x7993 - primary
        )
    }

    /// Check if device ID is a HIF2/secondary device
    pub fn is_hif2_device(device_id: u16) -> bool {
        matches!(device_id,
            regs::device_id::MT7996_2 |  // 0x7991 - HIF2
            regs::device_id::MT7992_2 |  // 0x799a - HIF2
            regs::device_id::MT7990_2    // 0x799b - HIF2
        )
    }

    /// Get the HIF2 device ID that corresponds to a primary device ID
    pub fn hif2_device_id(primary_device_id: u16) -> Option<u16> {
        match primary_device_id {
            regs::device_id::MT7996 => Some(regs::device_id::MT7996_2),  // 0x7990 -> 0x7991
            regs::device_id::MT7992 => Some(regs::device_id::MT7992_2),  // 0x7992 -> 0x799a
            regs::device_id::MT7990 => Some(regs::device_id::MT7990_2),  // 0x7993 -> 0x799b
            _ => None,
        }
    }
}

impl WifiDriver for Mt7996Driver {
    fn supports(info: &PcieDeviceInfo) -> bool {
        // Check vendor and device ID
        info.vendor_id == pcie::consts::vendor::MEDIATEK
            && Self::is_supported_device(info.device_id)
    }

    fn probe(info: &PcieDeviceInfo) -> Result<Self, DriverError> {
        // Probe without HIF2 (backwards compatible)
        Self::probe_with_hif2(info, None)
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

impl Mt7996Driver {
    /// Probe with optional HIF2 device info
    ///
    /// For proper MT7996 operation, both HIF1 (primary) and HIF2 (secondary)
    /// interfaces should be initialized. If hif2_info is None, only the
    /// primary interface will be used (may not work correctly).
    pub fn probe_with_hif2(info: &PcieDeviceInfo, hif2_info: Option<&PcieDeviceInfo>) -> Result<Self, DriverError> {
        let has_hif2 = hif2_info.is_some();

        if has_hif2 {
            userlib::println!("[mt7996] Initializing with HIF2 interface");
        } else {
            userlib::println!("[mt7996] WARNING: No HIF2 interface found!");
        }

        let device = Mt7996Device::init_with_hif2(info, hif2_info)
            .map_err(|_| DriverError::MapFailed)?;

        Ok(Self {
            device,
            device_id: info.device_id,
            state: DeviceState::Mapped,
            has_hif2,
        })
    }

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
