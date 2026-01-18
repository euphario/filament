//! PCIe Protocol
//!
//! Type-safe IPC protocol for communicating with pcied.
//!
//! ## Commands
//!
//! - `FindDevice`: Query for PCIe devices by vendor/device ID
//! - `ResetPort`: Reset a PCIe port (assert PERST#)
//!
//! ## Example
//!
//! ```rust
//! use userlib::ipc::{Client, protocols::PcieProtocol};
//!
//! let mut client = Client::<PcieProtocol>::connect()?;
//! let response = client.request(&PcieRequest::FindDevice {
//!     vendor_id: 0x14C3,  // MediaTek
//!     device_id: 0xFFFF,  // Any
//! })?;
//!
//! if let PcieResponse::Devices(devices) = response {
//!     for dev in devices.iter() {
//!         println!("Found: {:04x}:{:04x}", dev.vendor_id, dev.device_id);
//!     }
//! }
//! ```

use super::super::error::{IpcError, IpcResult};
use super::super::protocol::{Protocol, Message, Writer, Reader};

/// Maximum devices in a response
pub const MAX_DEVICES: usize = 8;

/// PCIe service protocol
pub struct PcieProtocol;

impl Protocol for PcieProtocol {
    type Request = PcieRequest;
    type Response = PcieResponse;
    const PORT_NAME: &'static [u8] = b"pcie";
}

/// PCIe device information
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
    /// PCI command register
    pub command: u16,
}

impl PcieDeviceInfo {
    /// Serialized size: 1+1+1+1+2+2+3+8+4+2 = 25 bytes
    pub const SERIALIZED_SIZE: usize = 25;
}

/// PCIe request messages
#[derive(Debug, Clone)]
pub enum PcieRequest {
    /// Find devices matching vendor/device ID
    /// Use 0xFFFF as wildcard for either field
    FindDevice {
        vendor_id: u16,
        device_id: u16,
    },
    /// Reset a PCIe port (assert PERST#)
    ResetPort {
        port: u8,
    },
    /// Enable Bus Master on a device and its parent bridge
    /// Required before DMA operations. Also enables BME on the parent bridge
    /// to allow DMA traffic through.
    EnableBusMaster {
        port: u8,
        bus: u8,
        device: u8,
        function: u8,
    },
    /// Read PCIe Device Status register
    /// Returns status bits for error detection (UR, Fatal, Non-Fatal, Correctable)
    ReadDeviceStatus {
        port: u8,
        bus: u8,
        device: u8,
        function: u8,
    },
    /// Clear PCIe Device Status error bits (W1C)
    ClearDeviceStatus {
        port: u8,
        bus: u8,
        device: u8,
        function: u8,
    },
    /// Read a register from device BAR0 (for debugging)
    /// offset: Offset from BAR0 base address
    ReadRegister {
        port: u8,
        bus: u8,
        device: u8,
        function: u8,
        offset: u32,
    },
    /// Write a register to device BAR0 (for debugging)
    /// offset: Offset from BAR0 base address
    /// value: 32-bit value to write
    WriteRegister {
        port: u8,
        bus: u8,
        device: u8,
        function: u8,
        offset: u32,
        value: u32,
    },
}

/// Command codes (must match pcied)
const CMD_FIND_DEVICE: u8 = 0x01;
const CMD_RESET_PORT: u8 = 0x02;
const CMD_ENABLE_BUS_MASTER: u8 = 0x03;
const CMD_READ_DEVICE_STATUS: u8 = 0x04;
const CMD_CLEAR_DEVICE_STATUS: u8 = 0x05;
const CMD_READ_REGISTER: u8 = 0x06;
const CMD_WRITE_REGISTER: u8 = 0x07;

impl Message for PcieRequest {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        let mut w = Writer::new(buf);
        match self {
            PcieRequest::FindDevice { vendor_id, device_id } => {
                w.write_u8(CMD_FIND_DEVICE)?;
                w.write_u16(*vendor_id)?;
                w.write_u16(*device_id)?;
            }
            PcieRequest::ResetPort { port } => {
                w.write_u8(CMD_RESET_PORT)?;
                w.write_u8(*port)?;
            }
            PcieRequest::EnableBusMaster { port, bus, device, function } => {
                w.write_u8(CMD_ENABLE_BUS_MASTER)?;
                w.write_u8(*port)?;
                w.write_u8(*bus)?;
                w.write_u8(*device)?;
                w.write_u8(*function)?;
            }
            PcieRequest::ReadDeviceStatus { port, bus, device, function } => {
                w.write_u8(CMD_READ_DEVICE_STATUS)?;
                w.write_u8(*port)?;
                w.write_u8(*bus)?;
                w.write_u8(*device)?;
                w.write_u8(*function)?;
            }
            PcieRequest::ClearDeviceStatus { port, bus, device, function } => {
                w.write_u8(CMD_CLEAR_DEVICE_STATUS)?;
                w.write_u8(*port)?;
                w.write_u8(*bus)?;
                w.write_u8(*device)?;
                w.write_u8(*function)?;
            }
            PcieRequest::ReadRegister { port, bus, device, function, offset } => {
                w.write_u8(CMD_READ_REGISTER)?;
                w.write_u8(*port)?;
                w.write_u8(*bus)?;
                w.write_u8(*device)?;
                w.write_u8(*function)?;
                w.write_u32(*offset)?;
            }
            PcieRequest::WriteRegister { port, bus, device, function, offset, value } => {
                w.write_u8(CMD_WRITE_REGISTER)?;
                w.write_u8(*port)?;
                w.write_u8(*bus)?;
                w.write_u8(*device)?;
                w.write_u8(*function)?;
                w.write_u32(*offset)?;
                w.write_u32(*value)?;
            }
        }
        Ok(w.finish())
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        let mut r = Reader::new(buf);
        let cmd = r.read_u8()?;

        let msg = match cmd {
            CMD_FIND_DEVICE => PcieRequest::FindDevice {
                vendor_id: r.read_u16()?,
                device_id: r.read_u16()?,
            },
            CMD_RESET_PORT => PcieRequest::ResetPort {
                port: r.read_u8()?,
            },
            CMD_ENABLE_BUS_MASTER => PcieRequest::EnableBusMaster {
                port: r.read_u8()?,
                bus: r.read_u8()?,
                device: r.read_u8()?,
                function: r.read_u8()?,
            },
            CMD_READ_DEVICE_STATUS => PcieRequest::ReadDeviceStatus {
                port: r.read_u8()?,
                bus: r.read_u8()?,
                device: r.read_u8()?,
                function: r.read_u8()?,
            },
            CMD_CLEAR_DEVICE_STATUS => PcieRequest::ClearDeviceStatus {
                port: r.read_u8()?,
                bus: r.read_u8()?,
                device: r.read_u8()?,
                function: r.read_u8()?,
            },
            CMD_READ_REGISTER => PcieRequest::ReadRegister {
                port: r.read_u8()?,
                bus: r.read_u8()?,
                device: r.read_u8()?,
                function: r.read_u8()?,
                offset: r.read_u32()?,
            },
            CMD_WRITE_REGISTER => PcieRequest::WriteRegister {
                port: r.read_u8()?,
                bus: r.read_u8()?,
                device: r.read_u8()?,
                function: r.read_u8()?,
                offset: r.read_u32()?,
                value: r.read_u32()?,
            },
            _ => return Err(IpcError::UnexpectedMessage),
        };
        Ok((msg, r.position()))
    }

    fn serialized_size(&self) -> usize {
        match self {
            PcieRequest::FindDevice { .. } => 5,
            PcieRequest::ResetPort { .. } => 2,
            PcieRequest::EnableBusMaster { .. } => 5,
            PcieRequest::ReadDeviceStatus { .. } => 5,
            PcieRequest::ClearDeviceStatus { .. } => 5,
            PcieRequest::ReadRegister { .. } => 9,
            PcieRequest::WriteRegister { .. } => 13,
        }
    }
}

/// PCIe response messages
#[derive(Debug, Clone)]
pub enum PcieResponse {
    /// List of devices found
    Devices(DeviceList),
    /// Reset result (success/failure)
    ResetResult(bool),
    /// Device Status register value (16-bit)
    /// Bit 0: Correctable Error Detected
    /// Bit 1: Non-Fatal Error Detected
    /// Bit 2: Fatal Error Detected
    /// Bit 3: Unsupported Request Detected (indicates bad DMA address!)
    /// Bit 5: Transactions Pending
    DeviceStatus(u16),
    /// Register value (32-bit) from ReadRegister request
    RegisterValue(u32),
    /// Error response
    Error(i32),
}

impl Message for PcieResponse {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        match self {
            PcieResponse::Devices(list) => {
                let count = list.len();
                let needed = 1 + count * PcieDeviceInfo::SERIALIZED_SIZE;
                if buf.len() < needed {
                    return Err(IpcError::MessageTooLarge);
                }

                buf[0] = count as u8;
                let mut offset = 1;

                for dev in list.iter() {
                    buf[offset] = dev.port;
                    buf[offset + 1] = dev.bus;
                    buf[offset + 2] = dev.device;
                    buf[offset + 3] = dev.function;
                    buf[offset + 4..offset + 6].copy_from_slice(&dev.vendor_id.to_le_bytes());
                    buf[offset + 6..offset + 8].copy_from_slice(&dev.device_id.to_le_bytes());
                    // Class code is 24-bit, only use lower 3 bytes
                    buf[offset + 8..offset + 11].copy_from_slice(&dev.class_code.to_le_bytes()[..3]);
                    buf[offset + 11..offset + 19].copy_from_slice(&dev.bar0_addr.to_le_bytes());
                    buf[offset + 19..offset + 23].copy_from_slice(&dev.bar0_size.to_le_bytes());
                    buf[offset + 23..offset + 25].copy_from_slice(&dev.command.to_le_bytes());
                    offset += PcieDeviceInfo::SERIALIZED_SIZE;
                }

                Ok(offset)
            }
            PcieResponse::ResetResult(success) => {
                if buf.is_empty() {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = if *success { 1 } else { 0 };
                Ok(1)
            }
            PcieResponse::DeviceStatus(status) => {
                if buf.len() < 3 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = 0xFE; // DeviceStatus marker (distinct from error 0xFF)
                buf[1..3].copy_from_slice(&status.to_le_bytes());
                Ok(3)
            }
            PcieResponse::RegisterValue(value) => {
                if buf.len() < 5 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = 0xFD; // RegisterValue marker
                buf[1..5].copy_from_slice(&value.to_le_bytes());
                Ok(5)
            }
            PcieResponse::Error(code) => {
                if buf.len() < 5 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = 0xFF; // Error marker
                buf[1..5].copy_from_slice(&code.to_le_bytes());
                Ok(5)
            }
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }

        // Check for error marker
        if buf[0] == 0xFF {
            if buf.len() < 5 {
                return Err(IpcError::Truncated);
            }
            let code = i32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
            return Ok((PcieResponse::Error(code), 5));
        }

        // Check for DeviceStatus marker
        if buf[0] == 0xFE {
            if buf.len() < 3 {
                return Err(IpcError::Truncated);
            }
            let status = u16::from_le_bytes([buf[1], buf[2]]);
            return Ok((PcieResponse::DeviceStatus(status), 3));
        }

        // Check for RegisterValue marker
        if buf[0] == 0xFD {
            if buf.len() < 5 {
                return Err(IpcError::Truncated);
            }
            let value = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
            return Ok((PcieResponse::RegisterValue(value), 5));
        }

        // If count is 0 or 1 and buffer is exactly 1 byte, it's a reset result
        if buf.len() == 1 {
            return Ok((PcieResponse::ResetResult(buf[0] == 1), 1));
        }

        // Otherwise, it's a device list
        let count = buf[0] as usize;
        if count > MAX_DEVICES {
            return Err(IpcError::MalformedMessage);
        }

        let expected_len = 1 + count * PcieDeviceInfo::SERIALIZED_SIZE;
        if buf.len() < expected_len {
            return Err(IpcError::Truncated);
        }

        let mut list = DeviceList::new();
        let mut offset = 1;

        for _ in 0..count {
            let dev = PcieDeviceInfo {
                port: buf[offset],
                bus: buf[offset + 1],
                device: buf[offset + 2],
                function: buf[offset + 3],
                vendor_id: u16::from_le_bytes([buf[offset + 4], buf[offset + 5]]),
                device_id: u16::from_le_bytes([buf[offset + 6], buf[offset + 7]]),
                class_code: u32::from_le_bytes([buf[offset + 8], buf[offset + 9], buf[offset + 10], 0]),
                bar0_addr: u64::from_le_bytes([
                    buf[offset + 11], buf[offset + 12], buf[offset + 13], buf[offset + 14],
                    buf[offset + 15], buf[offset + 16], buf[offset + 17], buf[offset + 18],
                ]),
                bar0_size: u32::from_le_bytes([
                    buf[offset + 19], buf[offset + 20], buf[offset + 21], buf[offset + 22],
                ]),
                command: u16::from_le_bytes([buf[offset + 23], buf[offset + 24]]),
            };
            list.push(dev);
            offset += PcieDeviceInfo::SERIALIZED_SIZE;
        }

        Ok((PcieResponse::Devices(list), offset))
    }

    fn serialized_size(&self) -> usize {
        match self {
            PcieResponse::Devices(list) => 1 + list.len() * PcieDeviceInfo::SERIALIZED_SIZE,
            PcieResponse::ResetResult(_) => 1,
            PcieResponse::DeviceStatus(_) => 3,
            PcieResponse::RegisterValue(_) => 5,
            PcieResponse::Error(_) => 5,
        }
    }
}

/// Fixed-size device list (no heap allocation)
#[derive(Debug, Clone)]
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
    pub fn push(&mut self, dev: PcieDeviceInfo) {
        if self.count < MAX_DEVICES {
            self.devices[self.count] = Some(dev);
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

    /// Iterate over devices
    pub fn iter(&self) -> impl Iterator<Item = &PcieDeviceInfo> {
        self.devices[..self.count].iter().filter_map(|d| d.as_ref())
    }

    /// Get first device
    pub fn first(&self) -> Option<&PcieDeviceInfo> {
        self.devices.first().and_then(|d| d.as_ref())
    }
}

impl Default for DeviceList {
    fn default() -> Self {
        Self::new()
    }
}

/// Convenience client for PCIe operations
///
/// Wraps Client<PcieProtocol> with helper methods.
pub struct PcieClient {
    inner: super::super::Client<PcieProtocol>,
}

impl PcieClient {
    /// Connect to pcied
    pub fn connect() -> IpcResult<Self> {
        let inner = super::super::Client::<PcieProtocol>::connect()?;
        Ok(Self { inner })
    }

    /// Find devices by vendor/device ID
    ///
    /// Use 0xFFFF as wildcard for either field.
    pub fn find_devices(&mut self, vendor_id: u16, device_id: u16) -> IpcResult<DeviceList> {
        let request = PcieRequest::FindDevice { vendor_id, device_id };
        let response = self.inner.request(&request)?;

        match response {
            PcieResponse::Devices(list) => Ok(list),
            PcieResponse::Error(code) => Err(IpcError::ServerError(code)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Find all MediaTek WiFi devices
    pub fn find_mediatek_devices(&mut self) -> IpcResult<DeviceList> {
        self.find_devices(0x14C3, 0xFFFF)
    }

    /// Reset a PCIe port
    pub fn reset_port(&mut self, port: u8) -> IpcResult<bool> {
        let request = PcieRequest::ResetPort { port };
        let response = self.inner.request(&request)?;

        match response {
            PcieResponse::ResetResult(success) => Ok(success),
            PcieResponse::Error(code) => Err(IpcError::ServerError(code)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Enable Bus Master on a device and its parent bridge
    ///
    /// This must be called before DMA operations. It enables BME on:
    /// 1. The specified device
    /// 2. The parent bridge (if any) to allow DMA traffic through the PCIe hierarchy
    pub fn enable_bus_master(&mut self, port: u8, bus: u8, device: u8, function: u8) -> IpcResult<bool> {
        let request = PcieRequest::EnableBusMaster { port, bus, device, function };
        let response = self.inner.request(&request)?;

        match response {
            PcieResponse::ResetResult(success) => Ok(success), // Reusing same response type
            PcieResponse::Error(code) => Err(IpcError::ServerError(code)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Read PCIe Device Status register
    ///
    /// Returns the Device Status register value (16-bit).
    /// Key bits for DMA debugging:
    /// - Bit 0: Correctable Error Detected
    /// - Bit 1: Non-Fatal Error Detected
    /// - Bit 2: Fatal Error Detected
    /// - Bit 3: Unsupported Request Detected (indicates bad DMA address!)
    /// - Bit 5: Transactions Pending
    pub fn read_device_status(&mut self, port: u8, bus: u8, device: u8, function: u8) -> IpcResult<u16> {
        let request = PcieRequest::ReadDeviceStatus { port, bus, device, function };
        let response = self.inner.request(&request)?;

        match response {
            PcieResponse::DeviceStatus(status) => Ok(status),
            PcieResponse::Error(code) => Err(IpcError::ServerError(code)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Clear PCIe Device Status error bits
    ///
    /// Clears all error bits (Correctable, Non-Fatal, Fatal, Unsupported Request).
    pub fn clear_device_status(&mut self, port: u8, bus: u8, device: u8, function: u8) -> IpcResult<bool> {
        let request = PcieRequest::ClearDeviceStatus { port, bus, device, function };
        let response = self.inner.request(&request)?;

        match response {
            PcieResponse::ResetResult(success) => Ok(success),
            PcieResponse::Error(code) => Err(IpcError::ServerError(code)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Read a 32-bit register from device BAR0
    ///
    /// offset: Offset from BAR0 base address (must be 4-byte aligned)
    /// Returns the 32-bit register value
    pub fn read_register(&mut self, port: u8, bus: u8, device: u8, function: u8, offset: u32) -> IpcResult<u32> {
        let request = PcieRequest::ReadRegister { port, bus, device, function, offset };
        let response = self.inner.request(&request)?;

        match response {
            PcieResponse::RegisterValue(value) => Ok(value),
            PcieResponse::Error(code) => Err(IpcError::ServerError(code)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Write a 32-bit register to device BAR0
    ///
    /// offset: Offset from BAR0 base address (must be 4-byte aligned)
    /// value: 32-bit value to write
    pub fn write_register(&mut self, port: u8, bus: u8, device: u8, function: u8, offset: u32, value: u32) -> IpcResult<bool> {
        let request = PcieRequest::WriteRegister { port, bus, device, function, offset, value };
        let response = self.inner.request(&request)?;

        match response {
            PcieResponse::ResetResult(success) => Ok(success),
            PcieResponse::Error(code) => Err(IpcError::ServerError(code)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }
}
