//! PCIe Client for Device Discovery
//!
//! Connects to pcied to query discovered PCIe devices.
//!
//! # Protocol
//!
//! The client sends a query and receives device info:
//! - Query: vendor_id (2), device_id (2), class_mask (3)
//! - Response: count, then for each device: port, bdf, ids, bar0_addr, bar0_size

use userlib::ipc::Channel;
use userlib::syscall;

/// Maximum devices in a query response
pub const MAX_DEVICES: usize = 16;

/// Device info returned from pcied
#[derive(Debug, Clone, Copy, Default)]
pub struct PcieDeviceInfo {
    /// PCIe port number (0-3)
    pub port: u8,
    /// Bus number
    pub bus: u8,
    /// Device number
    pub device: u8,
    /// Function number
    pub function: u8,
    /// Vendor ID
    pub vendor_id: u16,
    /// Device ID
    pub device_id: u16,
    /// Class code (24-bit)
    pub class_code: u32,
    /// BAR0 physical address
    pub bar0_addr: u64,
    /// BAR0 size in bytes
    pub bar0_size: u32,
    /// PCI command register (for verifying bus master enabled)
    pub command: u16,
}

/// PCIe client for querying devices from pcied
pub struct PcieClient {
    channel: Channel,
}

impl PcieClient {
    /// Connect to pcied
    pub fn connect() -> Option<Self> {
        let channel = Channel::connect(b"pcie:").ok()?;
        Some(Self { channel })
    }

    /// Find devices matching the given criteria
    ///
    /// - vendor_id: 0xFFFF means any
    /// - device_id: 0xFFFF means any
    ///
    /// Returns list of matching devices.
    pub fn find_devices(&mut self, vendor_id: u16, device_id: u16) -> DeviceList {
        let mut buf = [0u8; 128];

        // Build query: cmd(1) + vendor(2) + device(2)
        buf[0] = 0x01; // FIND_DEVICE command
        buf[1..3].copy_from_slice(&vendor_id.to_le_bytes());
        buf[3..5].copy_from_slice(&device_id.to_le_bytes());

        // Send query
        let _ = self.channel.send(&buf[..5]);

        // Wait for response
        let mut devices = DeviceList::new();

        for _ in 0..100 {
            if let Ok(len) = self.channel.recv(&mut buf) {
                if len > 0 {
                    // Parse response
                    let count = buf[0] as usize;
                    let mut offset = 1;

                    for _ in 0..count {
                        if offset + 25 > len {
                            break;
                        }

                        let info = PcieDeviceInfo {
                            port: buf[offset],
                            bus: buf[offset + 1],
                            device: buf[offset + 2],
                            function: buf[offset + 3],
                            vendor_id: u16::from_le_bytes([buf[offset + 4], buf[offset + 5]]),
                            device_id: u16::from_le_bytes([buf[offset + 6], buf[offset + 7]]),
                            class_code: u32::from_le_bytes([
                                buf[offset + 8], buf[offset + 9], buf[offset + 10], 0
                            ]),
                            bar0_addr: u64::from_le_bytes([
                                buf[offset + 11], buf[offset + 12], buf[offset + 13], buf[offset + 14],
                                buf[offset + 15], buf[offset + 16], buf[offset + 17], buf[offset + 18],
                            ]),
                            bar0_size: u32::from_le_bytes([
                                buf[offset + 19], buf[offset + 20], buf[offset + 21], buf[offset + 22],
                            ]),
                            command: u16::from_le_bytes([buf[offset + 23], buf[offset + 24]]),
                        };

                        devices.push(info);
                        offset += 25;
                    }

                    return devices;
                }
            }
            syscall::sleep_ms(1);
        }

        devices
    }

    /// Find all MediaTek WiFi devices (MT7996 family)
    pub fn find_mt7996_devices(&mut self) -> DeviceList {
        // Query for MediaTek vendor, any device
        self.find_devices(0x14C3, 0xFFFF)
    }

    /// Reset a PCIe port (for driver cleanup before exit)
    ///
    /// This asserts PERST# to reset the endpoint device on the specified port,
    /// returning it to power-on state for the next driver.
    pub fn reset_port(&mut self, port: u8) -> bool {
        let mut buf = [0u8; 16];

        // Build request: cmd(1) + port(1)
        buf[0] = 0x02; // RESET_PORT command
        buf[1] = port;

        // Send request
        let _ = self.channel.send(&buf[..2]);

        // Wait for response
        for _ in 0..100 {
            if let Ok(len) = self.channel.recv(&mut buf) {
                if len > 0 {
                    return buf[0] == 1;
                }
            }
            syscall::sleep_ms(1);
        }

        false
    }

    /// Enable Bus Master on a device and its parent bridge
    ///
    /// This must be called before DMA operations. It enables BME on:
    /// 1. The specified device
    /// 2. The parent bridge (if any) to allow DMA traffic through
    ///
    /// Returns true on success.
    pub fn enable_bus_master(&mut self, port: u8, bus: u8, device: u8, function: u8) -> bool {
        let mut buf = [0u8; 16];

        // Build request: cmd(1) + port(1) + bus(1) + device(1) + function(1)
        buf[0] = 0x03; // ENABLE_BUS_MASTER command
        buf[1] = port;
        buf[2] = bus;
        buf[3] = device;
        buf[4] = function;

        // Send request
        let _ = self.channel.send(&buf[..5]);

        // Wait for response
        for _ in 0..100 {
            if let Ok(len) = self.channel.recv(&mut buf) {
                if len > 0 {
                    return buf[0] == 1;
                }
            }
            syscall::sleep_ms(1);
        }

        false
    }

    /// Read PCIe Device Status register
    ///
    /// Returns the Device Status register value (16-bit), or None on error.
    /// Key bits for DMA debugging:
    /// - Bit 0: Correctable Error Detected
    /// - Bit 1: Non-Fatal Error Detected
    /// - Bit 2: Fatal Error Detected
    /// - Bit 3: Unsupported Request Detected (indicates bad DMA address!)
    /// - Bit 5: Transactions Pending
    pub fn read_device_status(&mut self, port: u8, bus: u8, device: u8, function: u8) -> Option<u16> {
        let mut buf = [0u8; 16];

        // Build request: cmd(1) + port(1) + bus(1) + device(1) + function(1)
        buf[0] = 0x04; // READ_DEVICE_STATUS command
        buf[1] = port;
        buf[2] = bus;
        buf[3] = device;
        buf[4] = function;

        // Send request
        let _ = self.channel.send(&buf[..5]);

        // Wait for response
        for _ in 0..100 {
            if let Ok(len) = self.channel.recv(&mut buf) {
                if len >= 3 {
                    if buf[0] == 0xFE {
                        // DeviceStatus marker (from pcie.rs protocol): status in bytes 1-2
                        return Some(u16::from_le_bytes([buf[1], buf[2]]));
                    }
                    return None; // Error
                }
            }
            syscall::sleep_ms(1);
        }

        None
    }

    /// Clear PCIe Device Status error bits
    ///
    /// Clears all error bits (Correctable, Non-Fatal, Fatal, Unsupported Request).
    /// Returns true on success.
    pub fn clear_device_status(&mut self, port: u8, bus: u8, device: u8, function: u8) -> bool {
        let mut buf = [0u8; 16];

        // Build request: cmd(1) + port(1) + bus(1) + device(1) + function(1)
        buf[0] = 0x05; // CLEAR_DEVICE_STATUS command
        buf[1] = port;
        buf[2] = bus;
        buf[3] = device;
        buf[4] = function;

        // Send request
        let _ = self.channel.send(&buf[..5]);

        // Wait for response
        for _ in 0..100 {
            if let Ok(len) = self.channel.recv(&mut buf) {
                if len > 0 {
                    return buf[0] == 1;
                }
            }
            syscall::sleep_ms(1);
        }

        false
    }
}

// Channel drops automatically, no need for explicit Drop impl

/// List of device info (fixed-size, no heap)
pub struct DeviceList {
    devices: [Option<PcieDeviceInfo>; MAX_DEVICES],
    count: usize,
}

impl DeviceList {
    /// Create empty list
    pub fn new() -> Self {
        Self {
            devices: [None; MAX_DEVICES],
            count: 0,
        }
    }

    /// Add a device
    pub fn push(&mut self, info: PcieDeviceInfo) {
        if self.count < MAX_DEVICES {
            self.devices[self.count] = Some(info);
            self.count += 1;
        }
    }

    /// Get count
    pub fn len(&self) -> usize {
        self.count
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Iterate
    pub fn iter(&self) -> impl Iterator<Item = &PcieDeviceInfo> {
        self.devices[..self.count].iter().filter_map(|d| d.as_ref())
    }
}
