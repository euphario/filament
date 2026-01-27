//! Device Query Protocol
//!
//! Message types for hierarchical driver queries.
//! Used for communication between devd, drivers, and clients.
//!
//! # Protocol Overview
//!
//! ## Device Registration (driver → devd)
//! When a driver discovers a device, it registers it with devd:
//! ```text
//! Driver → devd: REGISTER_DEVICE { class, vendor, product, path }
//! devd → Driver: DEVICE_INFO { device_id }
//! ```
//!
//! ## Device Queries (client → devd → driver)
//! ```text
//! Client → devd: LIST_DEVICES { class_filter }
//! devd → Client: DEVICE_LIST { devices[] }
//!
//! Client → devd: QUERY_DRIVER { device_id, query_type, payload }
//! devd → Driver: QUERY_DRIVER { ... }
//! Driver → devd: QUERY_RESULT { ... }
//! devd → Client: QUERY_RESULT { ... }
//! ```

// =============================================================================
// Message Header
// =============================================================================

/// Query message header (8 bytes)
///
/// All query messages start with this header.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct QueryHeader {
    /// Message type (see `msg` module)
    pub msg_type: u16,
    /// Flags (reserved, set to 0)
    pub flags: u16,
    /// Sequence ID for matching requests to responses
    pub seq_id: u32,
}

impl QueryHeader {
    pub const SIZE: usize = 8;

    pub const fn new(msg_type: u16, seq_id: u32) -> Self {
        Self {
            msg_type,
            flags: 0,
            seq_id,
        }
    }

    /// Serialize header to bytes
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..2].copy_from_slice(&self.msg_type.to_le_bytes());
        buf[2..4].copy_from_slice(&self.flags.to_le_bytes());
        buf[4..8].copy_from_slice(&self.seq_id.to_le_bytes());
        buf
    }

    /// Deserialize header from bytes
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            msg_type: u16::from_le_bytes([buf[0], buf[1]]),
            flags: u16::from_le_bytes([buf[2], buf[3]]),
            seq_id: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        })
    }
}

// =============================================================================
// Message Types
// =============================================================================

/// Message type constants
pub mod msg {
    // Registration messages (driver → devd)
    /// Register a new device with devd
    pub const REGISTER_DEVICE: u16 = 0x0100;
    /// Unregister a device from devd
    pub const UNREGISTER_DEVICE: u16 = 0x0101;
    /// Update device state
    pub const UPDATE_STATE: u16 = 0x0102;
    /// Register a port (with optional parent and type)
    pub const REGISTER_PORT: u16 = 0x0103;
    /// Unregister a port
    pub const UNREGISTER_PORT: u16 = 0x0104;

    // Query messages (client → devd → driver)
    /// List all registered devices
    pub const LIST_DEVICES: u16 = 0x0200;
    /// Get detailed info about a specific device
    pub const GET_DEVICE_INFO: u16 = 0x0201;
    /// Pass-through query to driver
    pub const QUERY_DRIVER: u16 = 0x0202;

    // Response messages
    /// Response containing device list
    pub const DEVICE_LIST: u16 = 0x0300;
    /// Response containing device info
    pub const DEVICE_INFO: u16 = 0x0301;
    /// Response from driver query
    pub const QUERY_RESULT: u16 = 0x0302;
    /// Error response
    pub const ERROR: u16 = 0x03FF;
}

// =============================================================================
// Device Classes (USB-style)
// =============================================================================

/// Device class constants (based on USB class codes)
pub mod class {
    /// Human Interface Device (keyboard, mouse, etc.)
    pub const HID: u16 = 0x03;
    /// Mass Storage (USB drives, etc.)
    pub const MASS_STORAGE: u16 = 0x08;
    /// USB Hub
    pub const HUB: u16 = 0x09;
    /// Network controller
    pub const NETWORK: u16 = 0xE0;
    /// NVMe storage
    pub const NVME: u16 = 0x0108; // PCI class
    /// Unknown/other
    pub const UNKNOWN: u16 = 0xFF;
}

// =============================================================================
// Device States
// =============================================================================

/// Device state constants
pub mod state {
    /// Device discovered but not yet initialized
    pub const DISCOVERED: u8 = 0;
    /// Device bound to a driver
    pub const BOUND: u8 = 1;
    /// Device fully operational
    pub const OPERATIONAL: u8 = 2;
    /// Device in error state
    pub const ERROR: u8 = 3;
    /// Device removed/disconnected
    pub const REMOVED: u8 = 4;
}

// =============================================================================
// Registration Messages
// =============================================================================

/// Device registration message (driver → devd)
///
/// Sent when a driver discovers a new device.
/// Variable-length: header + fixed fields + path + name
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DeviceRegister {
    pub header: QueryHeader,
    /// Device class (see `class` module)
    pub device_class: u16,
    /// Device subclass
    pub device_subclass: u16,
    /// Vendor ID
    pub vendor_id: u16,
    /// Product ID
    pub product_id: u16,
    /// Length of path string (follows fixed fields)
    pub path_len: u16,
    /// Length of name string (follows path)
    pub name_len: u16,
    // Followed by: path bytes, then name bytes
}

impl DeviceRegister {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 12;

    pub fn new(
        seq_id: u32,
        device_class: u16,
        device_subclass: u16,
        vendor_id: u16,
        product_id: u16,
    ) -> Self {
        Self {
            header: QueryHeader::new(msg::REGISTER_DEVICE, seq_id),
            device_class,
            device_subclass,
            vendor_id,
            product_id,
            path_len: 0,
            name_len: 0,
        }
    }

    /// Serialize to buffer, returns total length written
    pub fn write_to(&self, buf: &mut [u8], path: &[u8], name: &[u8]) -> Option<usize> {
        let total_len = Self::FIXED_SIZE + path.len() + name.len();
        if buf.len() < total_len {
            return None;
        }

        // Header
        buf[0..8].copy_from_slice(&self.header.to_bytes());

        // Fixed fields
        buf[8..10].copy_from_slice(&self.device_class.to_le_bytes());
        buf[10..12].copy_from_slice(&self.device_subclass.to_le_bytes());
        buf[12..14].copy_from_slice(&self.vendor_id.to_le_bytes());
        buf[14..16].copy_from_slice(&self.product_id.to_le_bytes());
        buf[16..18].copy_from_slice(&(path.len() as u16).to_le_bytes());
        buf[18..20].copy_from_slice(&(name.len() as u16).to_le_bytes());

        // Variable fields
        let path_start = Self::FIXED_SIZE;
        buf[path_start..path_start + path.len()].copy_from_slice(path);
        let name_start = path_start + path.len();
        buf[name_start..name_start + name.len()].copy_from_slice(name);

        Some(total_len)
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8], &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }

        let header = QueryHeader::from_bytes(buf)?;

        let device_class = u16::from_le_bytes([buf[8], buf[9]]);
        let device_subclass = u16::from_le_bytes([buf[10], buf[11]]);
        let vendor_id = u16::from_le_bytes([buf[12], buf[13]]);
        let product_id = u16::from_le_bytes([buf[14], buf[15]]);
        let path_len = u16::from_le_bytes([buf[16], buf[17]]) as usize;
        let name_len = u16::from_le_bytes([buf[18], buf[19]]) as usize;

        let total_len = Self::FIXED_SIZE + path_len + name_len;
        if buf.len() < total_len {
            return None;
        }

        let path_start = Self::FIXED_SIZE;
        let path = &buf[path_start..path_start + path_len];
        let name_start = path_start + path_len;
        let name = &buf[name_start..name_start + name_len];

        Some((
            Self {
                header,
                device_class,
                device_subclass,
                vendor_id,
                product_id,
                path_len: path_len as u16,
                name_len: name_len as u16,
            },
            path,
            name,
        ))
    }
}

// =============================================================================
// Port Registration Messages
// =============================================================================

/// Port type constants (matches devd/ports.rs PortType enum)
pub mod port_type {
    pub const UNKNOWN: u8 = 0;
    pub const BLOCK: u8 = 1;
    pub const PARTITION: u8 = 2;
    pub const FILESYSTEM: u8 = 3;
    pub const USB: u8 = 4;
    pub const NETWORK: u8 = 5;
    pub const CONSOLE: u8 = 6;
    pub const SERVICE: u8 = 7;
}

/// Port registration message (driver → devd)
///
/// Used by drivers to dynamically register ports with hierarchy information.
/// This enables the composable driver stack where each layer can spawn the next.
///
/// Variable-length: header + fixed fields + port_name + parent_name
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PortRegister {
    pub header: QueryHeader,
    /// Port type (see `port_type` module)
    pub port_type: u8,
    /// Length of port name (follows fixed fields)
    pub name_len: u8,
    /// Length of parent name (follows port name, 0 = no parent)
    pub parent_len: u8,
    /// Reserved
    pub _pad: u8,
    /// Shared memory ID for DataPort (0 = no data port)
    pub shmem_id: u32,
    // Followed by: port name bytes, then parent name bytes
}

impl PortRegister {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 8;

    pub fn new(seq_id: u32, port_type: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::REGISTER_PORT, seq_id),
            port_type,
            name_len: 0,
            parent_len: 0,
            _pad: 0,
            shmem_id: 0,
        }
    }

    /// Serialize to buffer, returns total length written
    pub fn write_to(
        &self,
        buf: &mut [u8],
        name: &[u8],
        parent: Option<&[u8]>,
        shmem_id: u32,
    ) -> Option<usize> {
        let parent_bytes = parent.unwrap_or(&[]);
        let total_len = Self::FIXED_SIZE + name.len() + parent_bytes.len();
        if buf.len() < total_len || name.len() > 255 || parent_bytes.len() > 255 {
            return None;
        }

        // Header
        buf[0..8].copy_from_slice(&self.header.to_bytes());

        // Fixed fields
        buf[8] = self.port_type;
        buf[9] = name.len() as u8;
        buf[10] = parent_bytes.len() as u8;
        buf[11] = 0;
        buf[12..16].copy_from_slice(&shmem_id.to_le_bytes());

        // Variable fields
        let name_start = Self::FIXED_SIZE;
        buf[name_start..name_start + name.len()].copy_from_slice(name);
        if !parent_bytes.is_empty() {
            let parent_start = name_start + name.len();
            buf[parent_start..parent_start + parent_bytes.len()].copy_from_slice(parent_bytes);
        }

        Some(total_len)
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8], Option<&[u8]>)> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }

        let header = QueryHeader::from_bytes(buf)?;
        let port_type = buf[8];
        let name_len = buf[9] as usize;
        let parent_len = buf[10] as usize;
        let shmem_id = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);

        let total_len = Self::FIXED_SIZE + name_len + parent_len;
        if buf.len() < total_len {
            return None;
        }

        let name_start = Self::FIXED_SIZE;
        let name = &buf[name_start..name_start + name_len];
        let parent = if parent_len > 0 {
            let parent_start = name_start + name_len;
            Some(&buf[parent_start..parent_start + parent_len])
        } else {
            None
        };

        Some((
            Self {
                header,
                port_type,
                name_len: name_len as u8,
                parent_len: parent_len as u8,
                _pad: 0,
                shmem_id,
            },
            name,
            parent,
        ))
    }
}

/// Port registration response
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PortRegisterResponse {
    pub header: QueryHeader,
    /// 0 = success, negative = error
    pub result: i32,
}

impl PortRegisterResponse {
    pub const SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, result: i32) -> Self {
        Self {
            header: QueryHeader::new(msg::REGISTER_PORT, seq_id),
            result,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.result.to_le_bytes());
        buf
    }
}

// =============================================================================
// Query Messages
// =============================================================================

/// List devices request
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ListDevices {
    pub header: QueryHeader,
    /// Filter by class (0 = all classes)
    pub class_filter: u16,
    pub _pad: u16,
}

impl ListDevices {
    pub const SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, class_filter: Option<u16>) -> Self {
        Self {
            header: QueryHeader::new(msg::LIST_DEVICES, seq_id),
            class_filter: class_filter.unwrap_or(0),
            _pad: 0,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..10].copy_from_slice(&self.class_filter.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            class_filter: u16::from_le_bytes([buf[8], buf[9]]),
            _pad: 0,
        })
    }
}

/// Get device info request
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GetDeviceInfo {
    pub header: QueryHeader,
    /// Device ID to query
    pub device_id: u32,
}

impl GetDeviceInfo {
    pub const SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, device_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::GET_DEVICE_INFO, seq_id),
            device_id,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.device_id.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            device_id: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        })
    }
}

/// Pass-through query to driver
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DriverQuery {
    pub header: QueryHeader,
    /// Device ID to query
    pub device_id: u32,
    /// Driver-specific query type
    pub query_type: u32,
    /// Payload length (follows this struct)
    pub payload_len: u16,
    pub _pad: u16,
    // Followed by: payload bytes
}

impl DriverQuery {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 12;

    pub fn new(seq_id: u32, device_id: u32, query_type: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::QUERY_DRIVER, seq_id),
            device_id,
            query_type,
            payload_len: 0,
            _pad: 0,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], payload: &[u8]) -> Option<usize> {
        let total_len = Self::FIXED_SIZE + payload.len();
        if buf.len() < total_len {
            return None;
        }

        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.device_id.to_le_bytes());
        buf[12..16].copy_from_slice(&self.query_type.to_le_bytes());
        buf[16..18].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        buf[18..20].copy_from_slice(&[0, 0]);
        buf[Self::FIXED_SIZE..Self::FIXED_SIZE + payload.len()].copy_from_slice(payload);

        Some(total_len)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }

        let header = QueryHeader::from_bytes(buf)?;
        let device_id = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let query_type = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);
        let payload_len = u16::from_le_bytes([buf[16], buf[17]]) as usize;

        let total_len = Self::FIXED_SIZE + payload_len;
        if buf.len() < total_len {
            return None;
        }

        let payload = &buf[Self::FIXED_SIZE..total_len];

        Some((
            Self {
                header,
                device_id,
                query_type,
                payload_len: payload_len as u16,
                _pad: 0,
            },
            payload,
        ))
    }
}

// =============================================================================
// Response Messages
// =============================================================================

/// Device info in list response
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DeviceEntry {
    /// Device ID assigned by devd
    pub device_id: u32,
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
    pub _pad: [u8; 3],
}

impl DeviceEntry {
    pub const SIZE: usize = 16;

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..4].copy_from_slice(&self.device_id.to_le_bytes());
        buf[4..6].copy_from_slice(&self.device_class.to_le_bytes());
        buf[6..8].copy_from_slice(&self.device_subclass.to_le_bytes());
        buf[8..10].copy_from_slice(&self.vendor_id.to_le_bytes());
        buf[10..12].copy_from_slice(&self.product_id.to_le_bytes());
        buf[12] = self.state;
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            device_id: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            device_class: u16::from_le_bytes([buf[4], buf[5]]),
            device_subclass: u16::from_le_bytes([buf[6], buf[7]]),
            vendor_id: u16::from_le_bytes([buf[8], buf[9]]),
            product_id: u16::from_le_bytes([buf[10], buf[11]]),
            state: buf[12],
            _pad: [0; 3],
        })
    }
}

/// Device list response header
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DeviceListResponse {
    pub header: QueryHeader,
    /// Number of devices in list
    pub count: u16,
    pub _pad: u16,
    // Followed by: count * DeviceEntry
}

impl DeviceListResponse {
    pub const HEADER_SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, count: u16) -> Self {
        Self {
            header: QueryHeader::new(msg::DEVICE_LIST, seq_id),
            count,
            _pad: 0,
        }
    }

    pub fn write_header(&self, buf: &mut [u8]) -> Option<()> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..10].copy_from_slice(&self.count.to_le_bytes());
        buf[10..12].copy_from_slice(&[0, 0]);
        Some(())
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            count: u16::from_le_bytes([buf[8], buf[9]]),
            _pad: 0,
        })
    }
}

/// Device info response
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DeviceInfoResponse {
    pub header: QueryHeader,
    pub entry: DeviceEntry,
    /// Driver port name length
    pub driver_port_len: u8,
    pub _pad: [u8; 3],
    // Followed by: driver port name bytes
}

impl DeviceInfoResponse {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + DeviceEntry::SIZE + 4;

    pub fn new(seq_id: u32, entry: DeviceEntry) -> Self {
        Self {
            header: QueryHeader::new(msg::DEVICE_INFO, seq_id),
            entry,
            driver_port_len: 0,
            _pad: [0; 3],
        }
    }

    pub fn write_to(&self, buf: &mut [u8], driver_port: &[u8]) -> Option<usize> {
        let total_len = Self::FIXED_SIZE + driver_port.len();
        if buf.len() < total_len {
            return None;
        }

        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..24].copy_from_slice(&self.entry.to_bytes());
        buf[24] = driver_port.len() as u8;
        buf[25..28].copy_from_slice(&[0, 0, 0]);
        buf[Self::FIXED_SIZE..total_len].copy_from_slice(driver_port);

        Some(total_len)
    }
}

/// Error response
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ErrorResponse {
    pub header: QueryHeader,
    /// Error code
    pub error_code: i32,
}

impl ErrorResponse {
    pub const SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, error_code: i32) -> Self {
        Self {
            header: QueryHeader::new(msg::ERROR, seq_id),
            error_code,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.error_code.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            error_code: i32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        })
    }
}

// =============================================================================
// Driver-Specific Query Types (for MSC)
// =============================================================================

/// MSC (Mass Storage Class) query types
pub mod msc {
    /// Get block device info (size, block size)
    pub const GET_BLOCK_INFO: u32 = 1;
    /// Read partition table
    pub const GET_PARTITION_TABLE: u32 = 2;
    /// Get device capacity
    pub const GET_CAPACITY: u32 = 3;
}

/// Block info response (for MSC GET_BLOCK_INFO)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlockInfo {
    /// Block size in bytes
    pub block_size: u32,
    /// Total number of blocks
    pub block_count: u64,
}

impl BlockInfo {
    pub const SIZE: usize = 12;

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..4].copy_from_slice(&self.block_size.to_le_bytes());
        buf[4..12].copy_from_slice(&self.block_count.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            block_size: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            block_count: u64::from_le_bytes([
                buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
            ]),
        })
    }
}

/// Partition entry (for MSC GET_PARTITION_TABLE)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PartitionEntry {
    /// Partition type (0x0B = FAT32, 0x83 = Linux, etc.)
    pub partition_type: u8,
    /// Bootable flag
    pub bootable: u8,
    pub _pad: [u8; 2],
    /// Start LBA
    pub start_lba: u32,
    /// Size in sectors
    pub size_sectors: u32,
}

impl PartitionEntry {
    pub const SIZE: usize = 12;

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0] = self.partition_type;
        buf[1] = self.bootable;
        buf[4..8].copy_from_slice(&self.start_lba.to_le_bytes());
        buf[8..12].copy_from_slice(&self.size_sectors.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            partition_type: buf[0],
            bootable: buf[1],
            _pad: [0; 2],
            start_lba: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            size_sectors: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        })
    }
}

// =============================================================================
// Error Codes
// =============================================================================

/// Query error codes
pub mod error {
    /// Success (no error)
    pub const OK: i32 = 0;
    /// Device not found
    pub const NOT_FOUND: i32 = -1;
    /// Invalid request format
    pub const INVALID_REQUEST: i32 = -2;
    /// Driver not available
    pub const NO_DRIVER: i32 = -3;
    /// Device in error state
    pub const DEVICE_ERROR: i32 = -4;
    /// Permission denied
    pub const PERMISSION_DENIED: i32 = -5;
    /// Query not supported
    pub const NOT_SUPPORTED: i32 = -6;
    /// Timeout waiting for response
    pub const TIMEOUT: i32 = -7;
}
