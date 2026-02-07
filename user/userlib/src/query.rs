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
    // 0x0103, 0x0104 - reserved (legacy REGISTER_PORT, UNREGISTER_PORT)
    /// Driver state change notification (driver → devd)
    pub const STATE_CHANGE: u16 = 0x0105;
    /// Get spawn context (child → devd) - returns the port that triggered this spawn
    pub const GET_SPAWN_CONTEXT: u16 = 0x0106;
    /// Query port info by name (returns shmem_id if DataPort-backed)
    pub const QUERY_PORT: u16 = 0x0107;
    /// Update shmem_id for an existing port
    pub const UPDATE_PORT_SHMEM_ID: u16 = 0x0108;
    /// Report discovered partitions (partd → devd)
    pub const REPORT_PARTITIONS: u16 = 0x0109;
    /// Partition ready notification (partd → devd)
    pub const PARTITION_READY: u16 = 0x010A;
    /// Mount ready notification (reserved)
    pub const MOUNT_READY: u16 = 0x010B;
    /// Register port with full PortInfo (unified enumeration)
    pub const REGISTER_PORT_INFO: u16 = 0x010C;
    /// Set port state (driver → devd)
    pub const SET_PORT_STATE: u16 = 0x010D;

    // Query messages (client → devd → driver)
    /// List all registered devices
    pub const LIST_DEVICES: u16 = 0x0200;
    /// Get detailed info about a specific device
    pub const GET_DEVICE_INFO: u16 = 0x0201;
    /// Pass-through query to driver
    pub const QUERY_DRIVER: u16 = 0x0202;
    /// List all registered ports (for introspection)
    pub const LIST_PORTS: u16 = 0x0203;
    /// List all services (for introspection)
    pub const LIST_SERVICES: u16 = 0x0204;
    /// Query detailed info from a service by name
    pub const QUERY_SERVICE_INFO: u16 = 0x0205;
    /// Query log history from devd
    pub const LOG_QUERY: u16 = 0x0206;
    /// Control logging (on/off)
    pub const LOG_CONTROL: u16 = 0x0207;

    // Log messages (driver → devd)
    /// Driver sends log message to devd
    pub const LOG_MESSAGE: u16 = 0x0500;

    // Commands (devd → driver)
    /// Tell driver to spawn a child process
    pub const SPAWN_CHILD: u16 = 0x0400;
    /// Tell driver to stop a child process
    pub const STOP_CHILD: u16 = 0x0401;
    /// Acknowledgement of spawn/stop command
    pub const SPAWN_ACK: u16 = 0x0402;
    /// Attach a disk to partd (devd → partd)
    pub const ATTACH_DISK: u16 = 0x0410;
    /// Register a partition with assigned name (devd → partd)
    pub const REGISTER_PARTITION: u16 = 0x0411;
    /// Mount a partition (reserved)
    pub const MOUNT_PARTITION: u16 = 0x0412;
    /// Get a configuration value from driver (response via SERVICE_INFO_RESULT)
    pub const CONFIG_GET: u16 = 0x0420;
    /// Set a configuration value on driver (response via SERVICE_INFO_RESULT)
    pub const CONFIG_SET: u16 = 0x0421;

    // Response messages
    /// Response containing device list
    pub const DEVICE_LIST: u16 = 0x0300;
    /// Response containing device info
    pub const DEVICE_INFO: u16 = 0x0301;
    /// Response from driver query
    pub const QUERY_RESULT: u16 = 0x0302;
    /// Response containing spawn context (port name that triggered spawn)
    pub const SPAWN_CONTEXT: u16 = 0x0303;
    /// Response containing port info (including shmem_id)
    pub const PORT_INFO: u16 = 0x0304;
    /// Response containing list of ports
    pub const PORTS_LIST: u16 = 0x0305;
    /// Response containing list of services
    pub const SERVICES_LIST: u16 = 0x0306;
    /// Response containing service info text
    pub const SERVICE_INFO_RESULT: u16 = 0x0307;
    /// Response containing log history
    pub const LOG_HISTORY: u16 = 0x0308;
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

/// Wire protocol port type constants
pub mod port_type {
    pub const UNKNOWN: u8 = 0;
    pub const BLOCK: u8 = 1;
    pub const PARTITION: u8 = 2;
    pub const FILESYSTEM: u8 = 3;
    pub const USB: u8 = 4;
    pub const NETWORK: u8 = 5;
    pub const CONSOLE: u8 = 6;
    pub const SERVICE: u8 = 7;
    pub const STORAGE: u8 = 8;
}

/// Filesystem type hints for partition mounting
pub mod fs_hint {
    pub const UNKNOWN: u8 = 0;
    pub const FAT12: u8 = 1;
    pub const FAT16: u8 = 2;
    pub const FAT32: u8 = 3;
    pub const EXFAT: u8 = 4;
    pub const EXT2: u8 = 5;
    pub const EXT4: u8 = 6;
    pub const NTFS: u8 = 7;
    pub const SWAP: u8 = 8;

    /// Convert MBR partition type to filesystem hint
    pub fn from_mbr_type(mbr_type: u8) -> u8 {
        match mbr_type {
            0x01 => FAT12,
            0x04 | 0x06 | 0x0E => FAT16,
            0x0B | 0x0C => FAT32,
            0x07 => NTFS,
            0x82 => SWAP,
            0x83 => EXT4,
            _ => UNKNOWN,
        }
    }
}

/// Partition scheme types
pub mod part_scheme {
    pub const UNKNOWN: u8 = 0;
    pub const MBR: u8 = 1;
    pub const GPT: u8 = 2;
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
            header: QueryHeader::new(msg::REGISTER_PORT_INFO, seq_id),
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
// Port Registration with PortInfo (unified enumeration)
// =============================================================================

/// Register port with full PortInfo (driver → devd)
///
/// Uses the unified PortInfo struct from abi crate for type-safe,
/// structured port metadata. This replaces the legacy PortRegister
/// for new drivers.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PortRegisterInfo {
    pub header: QueryHeader,
    /// Shared memory ID for DataPort (0 = no data port)
    pub shmem_id: u32,
    /// Padding for alignment
    pub _pad: u32,
    // Followed by PortInfo bytes (112 bytes)
}

impl PortRegisterInfo {
    /// Fixed header size before PortInfo
    pub const HEADER_SIZE: usize = QueryHeader::SIZE + 8;
    /// Total message size including PortInfo
    pub const SIZE: usize = Self::HEADER_SIZE + 112; // PortInfo is 112 bytes

    pub fn new(seq_id: u32, shmem_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::REGISTER_PORT_INFO, seq_id),
            shmem_id,
            _pad: 0,
        }
    }

    /// Serialize to buffer with PortInfo, returns total length written
    pub fn write_to(&self, buf: &mut [u8], info: &abi::PortInfo) -> Option<usize> {
        if buf.len() < Self::SIZE {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.shmem_id.to_le_bytes());
        buf[12..16].copy_from_slice(&self._pad.to_le_bytes());
        // Copy PortInfo bytes
        let info_bytes: &[u8; 112] = unsafe { &*(info as *const abi::PortInfo as *const [u8; 112]) };
        buf[16..128].copy_from_slice(info_bytes);
        Some(Self::SIZE)
    }

    /// Deserialize header from bytes (PortInfo follows at offset 16)
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let header = QueryHeader::from_bytes(&buf[0..8])?;
        let shmem_id = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let info_bytes = &buf[16..128];
        Some((
            Self { header, shmem_id, _pad: 0 },
            info_bytes,
        ))
    }
}

// =============================================================================
// State Change Messages (driver → devd)
// =============================================================================

/// Driver state constants
pub mod driver_state {
    /// Driver is initializing
    pub const INITIALIZING: u8 = 0;
    /// Driver is ready to receive commands
    pub const READY: u8 = 1;
    /// Driver encountered an error
    pub const ERROR: u8 = 2;
    /// Driver is stopping
    pub const STOPPING: u8 = 3;
}

/// State change notification (driver → devd)
///
/// Sent when a driver's state changes. devd uses this to track
/// driver lifecycle and trigger policy decisions.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct StateChange {
    pub header: QueryHeader,
    /// New state (see `driver_state` module)
    pub new_state: u8,
    pub _pad: [u8; 3],
}

impl StateChange {
    pub const SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, new_state: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::STATE_CHANGE, seq_id),
            new_state,
            _pad: [0; 3],
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8] = self.new_state;
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            new_state: buf[8],
            _pad: [0; 3],
        })
    }
}

// =============================================================================
// Spawn Context Messages (child → devd)
// =============================================================================

/// Get spawn context request (child → devd)
///
/// Sent by a newly spawned child to learn which port triggered its spawn.
/// devd tracks this based on the caller's PID.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct GetSpawnContext {
    pub header: QueryHeader,
}

impl GetSpawnContext {
    pub const SIZE: usize = QueryHeader::SIZE;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::GET_SPAWN_CONTEXT, seq_id),
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        self.header.to_bytes()
    }
}

/// Spawn context response (devd → child)
///
/// Contains the port name that triggered this driver's spawn.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SpawnContextResponse {
    pub header: QueryHeader,
    /// Result code (0 = success, negative = error)
    pub result: i32,
    /// Port type that triggered spawn
    pub port_type: u8,
    /// Length of port name
    pub port_name_len: u8,
    /// Length of metadata
    pub metadata_len: u8,
    pub _pad: u8,
    // Followed by: port_name bytes, then metadata bytes
}

impl SpawnContextResponse {
    pub const HEADER_SIZE: usize = QueryHeader::SIZE + 8;
    pub const MAX_PORT_NAME: usize = 64;

    pub fn new(seq_id: u32, result: i32, port_type: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::SPAWN_CONTEXT, seq_id),
            result,
            port_type,
            port_name_len: 0,
            metadata_len: 0,
            _pad: 0,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], port_name: &[u8]) -> Option<usize> {
        self.write_to_with_metadata(buf, port_name, &[])
    }

    pub fn write_to_with_metadata(&self, buf: &mut [u8], port_name: &[u8], metadata: &[u8]) -> Option<usize> {
        if port_name.len() > 255 || metadata.len() > 255 {
            return None;
        }
        let total = Self::HEADER_SIZE + port_name.len() + metadata.len();
        if buf.len() < total {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.result.to_le_bytes());
        buf[12] = self.port_type;
        buf[13] = port_name.len() as u8;
        buf[14] = metadata.len() as u8;
        buf[15] = 0;
        buf[16..16 + port_name.len()].copy_from_slice(port_name);
        let meta_start = 16 + port_name.len();
        buf[meta_start..meta_start + metadata.len()].copy_from_slice(metadata);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8], &[u8])> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let header = QueryHeader::from_bytes(buf)?;
        let result = i32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let port_type = buf[12];
        let port_name_len = buf[13] as usize;
        let metadata_len = buf[14] as usize;

        let total = Self::HEADER_SIZE + port_name_len + metadata_len;
        if buf.len() < total {
            return None;
        }

        let resp = Self {
            header,
            result,
            port_type,
            port_name_len: port_name_len as u8,
            metadata_len: metadata_len as u8,
            _pad: 0,
        };
        let port_name = &buf[Self::HEADER_SIZE..Self::HEADER_SIZE + port_name_len];
        let meta_start = Self::HEADER_SIZE + port_name_len;
        let metadata = &buf[meta_start..meta_start + metadata_len];
        Some((resp, port_name, metadata))
    }
}

// =============================================================================
// Port Query Messages (client → devd)
// =============================================================================

/// Query port information by name
///
/// Allows consumers to discover DataPort shmem_id for a named port.
/// This replaces hardcoded shmem_id constants in drivers.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct QueryPort {
    pub header: QueryHeader,
    /// Length of port name
    pub name_len: u8,
    pub _pad: [u8; 3],
    // Followed by: port name bytes
}

impl QueryPort {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::QUERY_PORT, seq_id),
            name_len: 0,
            _pad: [0; 3],
        }
    }

    /// Serialize to buffer, returns total length written
    pub fn write_to(&self, buf: &mut [u8], name: &[u8]) -> Option<usize> {
        let total_len = Self::FIXED_SIZE + name.len();
        if buf.len() < total_len || name.len() > 255 {
            return None;
        }

        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8] = name.len() as u8;
        buf[9..12].copy_from_slice(&[0, 0, 0]);
        buf[Self::FIXED_SIZE..total_len].copy_from_slice(name);

        Some(total_len)
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }

        let header = QueryHeader::from_bytes(buf)?;
        let name_len = buf[8] as usize;

        let total_len = Self::FIXED_SIZE + name_len;
        if buf.len() < total_len {
            return None;
        }

        let name = &buf[Self::FIXED_SIZE..total_len];

        Some((
            Self {
                header,
                name_len: name_len as u8,
                _pad: [0; 3],
            },
            name,
        ))
    }
}

/// Update port shmem_id message
///
/// Allows drivers to set/update the DataPort shmem_id for an existing port.
/// Used when the DataPort is created after initial port registration.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct UpdatePortShmemId {
    pub header: QueryHeader,
    /// New shmem_id for the port
    pub shmem_id: u32,
    /// Length of port name
    pub name_len: u8,
    pub _pad: [u8; 3],
    // Followed by: port name bytes
}

impl UpdatePortShmemId {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 8;

    pub fn new(seq_id: u32, shmem_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::UPDATE_PORT_SHMEM_ID, seq_id),
            shmem_id,
            name_len: 0,
            _pad: [0; 3],
        }
    }

    /// Serialize to buffer, returns total length written
    pub fn write_to(&self, buf: &mut [u8], name: &[u8]) -> Option<usize> {
        let total_len = Self::FIXED_SIZE + name.len();
        if buf.len() < total_len || name.len() > 255 {
            return None;
        }

        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.shmem_id.to_le_bytes());
        buf[12] = name.len() as u8;
        buf[13..16].copy_from_slice(&[0, 0, 0]);
        buf[Self::FIXED_SIZE..total_len].copy_from_slice(name);

        Some(total_len)
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }

        let header = QueryHeader::from_bytes(buf)?;
        let shmem_id = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let name_len = buf[12] as usize;

        let total_len = Self::FIXED_SIZE + name_len;
        if buf.len() < total_len {
            return None;
        }

        let name = &buf[Self::FIXED_SIZE..total_len];

        Some((
            Self {
                header,
                shmem_id,
                name_len: name_len as u8,
                _pad: [0; 3],
            },
            name,
        ))
    }
}

/// Set port state message (driver → devd)
///
/// Transitions a port to a new state. Rules are checked when
/// a port transitions to Ready state.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SetPortState {
    pub header: QueryHeader,
    /// New state (see abi::PortState)
    pub state: u8,
    /// Length of port name
    pub name_len: u8,
    pub _pad: [u8; 2],
    // Followed by: port name bytes
}

impl SetPortState {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, state: abi::PortState) -> Self {
        Self {
            header: QueryHeader::new(msg::SET_PORT_STATE, seq_id),
            state: state as u8,
            name_len: 0,
            _pad: [0; 2],
        }
    }

    /// Serialize to buffer, returns total length written
    pub fn write_to(&self, buf: &mut [u8], name: &[u8]) -> Option<usize> {
        let total_len = Self::FIXED_SIZE + name.len();
        if buf.len() < total_len || name.len() > 255 {
            return None;
        }

        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8] = self.state;
        buf[9] = name.len() as u8;
        buf[10..12].copy_from_slice(&[0, 0]);
        buf[Self::FIXED_SIZE..total_len].copy_from_slice(name);

        Some(total_len)
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }

        let header = QueryHeader::from_bytes(buf)?;
        let state = buf[8];
        let name_len = buf[9] as usize;

        let total_len = Self::FIXED_SIZE + name_len;
        if buf.len() < total_len {
            return None;
        }

        let name = &buf[Self::FIXED_SIZE..total_len];

        Some((
            Self {
                header,
                state,
                name_len: name_len as u8,
                _pad: [0; 2],
            },
            name,
        ))
    }
}

/// Port info flags
pub mod port_flags {
    /// Port has a DataPort backing (shmem_id is valid)
    pub const HAS_DATAPORT: u8 = 0x01;
    /// Port is available/ready
    pub const AVAILABLE: u8 = 0x02;
}

/// Port info response (devd → client)
///
/// Contains port information including DataPort shmem_id if applicable.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PortInfoResponse {
    pub header: QueryHeader,
    /// Result code (0 = success, negative = error)
    pub result: i32,
    /// Port type (see `port_type` module)
    pub port_type: u8,
    /// Flags (see `port_flags` module)
    pub flags: u8,
    pub _pad: u16,
    /// DataPort shared memory ID (0 if no DataPort)
    pub shmem_id: u32,
    /// Owner process ID
    pub owner_pid: u32,
}

impl PortInfoResponse {
    pub const SIZE: usize = QueryHeader::SIZE + 16;

    pub fn new(seq_id: u32, result: i32) -> Self {
        Self {
            header: QueryHeader::new(msg::PORT_INFO, seq_id),
            result,
            port_type: 0,
            flags: 0,
            _pad: 0,
            shmem_id: 0,
            owner_pid: 0,
        }
    }

    pub fn success(
        seq_id: u32,
        port_type: u8,
        flags: u8,
        shmem_id: u32,
        owner_pid: u32,
    ) -> Self {
        Self {
            header: QueryHeader::new(msg::PORT_INFO, seq_id),
            result: error::OK,
            port_type,
            flags,
            _pad: 0,
            shmem_id,
            owner_pid,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.result.to_le_bytes());
        buf[12] = self.port_type;
        buf[13] = self.flags;
        buf[14..16].copy_from_slice(&[0, 0]);
        buf[16..20].copy_from_slice(&self.shmem_id.to_le_bytes());
        buf[20..24].copy_from_slice(&self.owner_pid.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            result: i32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            port_type: buf[12],
            flags: buf[13],
            _pad: 0,
            shmem_id: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            owner_pid: u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
        })
    }

    /// Check if port has a DataPort
    pub fn has_dataport(&self) -> bool {
        self.flags & port_flags::HAS_DATAPORT != 0
    }

    /// Check if port is available
    pub fn is_available(&self) -> bool {
        self.flags & port_flags::AVAILABLE != 0
    }
}

// =============================================================================
// Spawn Command Messages (devd → driver)
// =============================================================================

// =============================================================================
// Filter Types for Queries and Commands
// =============================================================================

/// Filter match mode
pub mod filter_mode {
    /// Exact name match (e.g., "part0:")
    pub const EXACT: u8 = 0;
    /// Match by port type (e.g., all Partition ports)
    pub const BY_TYPE: u8 = 1;
    /// Match all children of a port (e.g., "disk0:.*")
    pub const CHILDREN_OF: u8 = 2;
    /// Match all (broadcast)
    pub const ALL: u8 = 3;
}

/// Filter for targeting ports/devices
///
/// Used in spawn commands and queries to select targets.
/// Same mechanism handles directed (1 match) and fan-out (N matches).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PortFilter {
    /// Filter mode (see `filter_mode` module)
    pub mode: u8,
    /// Port type filter (for BY_TYPE mode)
    pub port_type: u8,
    /// Device class filter (0 = any)
    pub class_filter: u16,
    /// Length of name/pattern string
    pub pattern_len: u8,
    pub _pad: [u8; 3],
    // Followed by: pattern bytes (for EXACT or CHILDREN_OF modes)
}

impl PortFilter {
    pub const FIXED_SIZE: usize = 8;

    /// Create an exact name match filter
    pub fn exact(name: &[u8]) -> (Self, &[u8]) {
        (Self {
            mode: filter_mode::EXACT,
            port_type: 0,
            class_filter: 0,
            pattern_len: name.len() as u8,
            _pad: [0; 3],
        }, name)
    }

    /// Create a port type filter
    pub fn by_type(port_type: u8) -> Self {
        Self {
            mode: filter_mode::BY_TYPE,
            port_type,
            class_filter: 0,
            pattern_len: 0,
            _pad: [0; 3],
        }
    }

    /// Create a children-of filter
    pub fn children_of(parent: &[u8]) -> (Self, &[u8]) {
        (Self {
            mode: filter_mode::CHILDREN_OF,
            port_type: 0,
            class_filter: 0,
            pattern_len: parent.len() as u8,
            _pad: [0; 3],
        }, parent)
    }

    /// Create a broadcast filter (all)
    pub fn all() -> Self {
        Self {
            mode: filter_mode::ALL,
            port_type: 0,
            class_filter: 0,
            pattern_len: 0,
            _pad: [0; 3],
        }
    }

    /// Write filter to buffer, returns bytes written
    pub fn write_to(&self, buf: &mut [u8], pattern: &[u8]) -> Option<usize> {
        let total = Self::FIXED_SIZE + pattern.len();
        if buf.len() < total {
            return None;
        }
        buf[0] = self.mode;
        buf[1] = self.port_type;
        buf[2..4].copy_from_slice(&self.class_filter.to_le_bytes());
        buf[4] = pattern.len() as u8;
        buf[5..8].copy_from_slice(&[0, 0, 0]);
        buf[Self::FIXED_SIZE..total].copy_from_slice(pattern);
        Some(total)
    }

    /// Parse filter from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }
        let pattern_len = buf[4] as usize;
        let total = Self::FIXED_SIZE + pattern_len;
        if buf.len() < total {
            return None;
        }
        let pattern = &buf[Self::FIXED_SIZE..total];
        Some((Self {
            mode: buf[0],
            port_type: buf[1],
            class_filter: u16::from_le_bytes([buf[2], buf[3]]),
            pattern_len: pattern_len as u8,
            _pad: [0; 3],
        }, pattern))
    }
}

// =============================================================================
// Spawn Child Command
// =============================================================================

/// Spawn child command (devd → driver)
///
/// Tells a driver to spawn child process(es) for ports matching a filter.
/// The driver spawns children with its own restricted capabilities.
///
/// Examples:
/// - Filter "part0:" (exact) → spawn 1 child for that port
/// - Filter "*.Partition" (by_type) → spawn N children for all partition ports
///
/// Variable-length: header + fixed fields + binary_name + filter
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SpawnChild {
    pub header: QueryHeader,
    /// Length of binary name
    pub binary_len: u8,
    /// Length of filter (PortFilter + pattern)
    pub filter_len: u8,
    pub _pad: [u8; 2],
    /// Capability bits for spawned child (0 = inherit parent's)
    pub caps: u64,
    // Followed by: binary name bytes, then PortFilter + pattern
}

impl SpawnChild {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 4 + 8;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::SPAWN_CHILD, seq_id),
            binary_len: 0,
            filter_len: 0,
            _pad: [0; 2],
            caps: 0,
        }
    }

    pub fn with_caps(seq_id: u32, caps: u64) -> Self {
        Self {
            header: QueryHeader::new(msg::SPAWN_CHILD, seq_id),
            binary_len: 0,
            filter_len: 0,
            _pad: [0; 2],
            caps,
        }
    }

    /// Serialize to buffer with exact port name filter
    pub fn write_exact(
        &self,
        buf: &mut [u8],
        binary: &[u8],
        port_name: &[u8],
    ) -> Option<usize> {
        let (filter, pattern) = PortFilter::exact(port_name);
        self.write_to_internal(buf, binary, &filter, pattern)
    }

    /// Serialize to buffer with type filter
    pub fn write_by_type(
        &self,
        buf: &mut [u8],
        binary: &[u8],
        port_type: u8,
    ) -> Option<usize> {
        let filter = PortFilter::by_type(port_type);
        self.write_to_internal(buf, binary, &filter, &[])
    }

    /// Serialize to buffer with custom filter
    pub fn write_to_internal(
        &self,
        buf: &mut [u8],
        binary: &[u8],
        filter: &PortFilter,
        pattern: &[u8],
    ) -> Option<usize> {
        let filter_total = PortFilter::FIXED_SIZE + pattern.len();
        let total_len = Self::FIXED_SIZE + binary.len() + filter_total;
        if buf.len() < total_len || binary.len() > 255 || filter_total > 255 {
            return None;
        }

        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8] = binary.len() as u8;
        buf[9] = filter_total as u8;
        buf[10] = 0;
        buf[11] = 0;
        buf[12..20].copy_from_slice(&self.caps.to_le_bytes());

        let binary_start = Self::FIXED_SIZE;
        buf[binary_start..binary_start + binary.len()].copy_from_slice(binary);

        let filter_start = binary_start + binary.len();
        filter.write_to(&mut buf[filter_start..], pattern)?;

        Some(total_len)
    }

    /// Legacy: Serialize with port name (backwards compat, uses exact filter)
    pub fn write_to(&self, buf: &mut [u8], binary: &[u8], trigger_port: &[u8]) -> Option<usize> {
        self.write_exact(buf, binary, trigger_port)
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8], &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }

        let header = QueryHeader::from_bytes(buf)?;
        let binary_len = buf[8] as usize;
        let filter_len = buf[9] as usize;

        let total_len = Self::FIXED_SIZE + binary_len + filter_len;
        if buf.len() < total_len {
            return None;
        }

        let binary_start = Self::FIXED_SIZE;
        let binary = &buf[binary_start..binary_start + binary_len];
        let filter_start = binary_start + binary_len;
        let filter_bytes = &buf[filter_start..filter_start + filter_len];

        let caps = if buf.len() >= 20 {
            u64::from_le_bytes([
                buf[12], buf[13], buf[14], buf[15],
                buf[16], buf[17], buf[18], buf[19],
            ])
        } else {
            0
        };

        Some((
            Self {
                header,
                binary_len: binary_len as u8,
                filter_len: filter_len as u8,
                _pad: [0; 2],
                caps,
            },
            binary,
            filter_bytes,
        ))
    }

    /// Parse the filter from raw filter bytes
    pub fn parse_filter(filter_bytes: &[u8]) -> Option<(PortFilter, &[u8])> {
        PortFilter::from_bytes(filter_bytes)
    }
}

/// Spawn acknowledgement (driver → devd)
///
/// Sent by driver after processing SPAWN_CHILD command.
/// Reports how many children were spawned (filter can match N ports).
///
/// Variable-length: header + fixed fields + child PIDs array
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SpawnAck {
    pub header: QueryHeader,
    /// 0 = success, negative = error code
    pub result: i32,
    /// Number of children spawned
    pub spawn_count: u8,
    /// Number of filter matches (may be > spawn_count if some failed)
    pub match_count: u8,
    pub _pad: [u8; 2],
    // Followed by: spawn_count * u32 (child PIDs)
}

impl SpawnAck {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 8;
    pub const MAX_CHILDREN: usize = 16;

    /// Create a single-spawn ack (backwards compat)
    pub fn new(seq_id: u32, result: i32, child_pid: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::SPAWN_ACK, seq_id),
            result,
            spawn_count: if child_pid != 0 { 1 } else { 0 },
            match_count: 1,
            _pad: [0; 2],
        }
    }

    /// Create a multi-spawn ack
    pub fn new_multi(seq_id: u32, result: i32, match_count: u8, spawn_count: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::SPAWN_ACK, seq_id),
            result,
            spawn_count,
            match_count,
            _pad: [0; 2],
        }
    }

    /// Write to buffer with child PIDs
    pub fn write_to(&self, buf: &mut [u8], child_pids: &[u32]) -> Option<usize> {
        let total = Self::FIXED_SIZE + child_pids.len() * 4;
        if buf.len() < total || child_pids.len() > Self::MAX_CHILDREN {
            return None;
        }

        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.result.to_le_bytes());
        buf[12] = child_pids.len() as u8;
        buf[13] = self.match_count;
        buf[14..16].copy_from_slice(&[0, 0]);

        for (i, &pid) in child_pids.iter().enumerate() {
            let offset = Self::FIXED_SIZE + i * 4;
            buf[offset..offset + 4].copy_from_slice(&pid.to_le_bytes());
        }

        Some(total)
    }

    /// Legacy: to_bytes for single spawn (backwards compat)
    pub fn to_bytes(&self) -> [u8; 20] {
        let mut buf = [0u8; 20];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.result.to_le_bytes());
        buf[12] = self.spawn_count;
        buf[13] = self.match_count;
        buf[14..16].copy_from_slice(&[0, 0]);
        // No PIDs in legacy format
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            result: i32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            spawn_count: buf[12],
            match_count: buf[13],
            _pad: [0; 2],
        })
    }

    /// Parse child PIDs from buffer (after from_bytes)
    pub fn parse_pids(buf: &[u8], count: usize) -> Option<[u32; Self::MAX_CHILDREN]> {
        if buf.len() < Self::FIXED_SIZE + count * 4 {
            return None;
        }
        let mut pids = [0u32; Self::MAX_CHILDREN];
        for i in 0..count.min(Self::MAX_CHILDREN) {
            let offset = Self::FIXED_SIZE + i * 4;
            pids[i] = u32::from_le_bytes([
                buf[offset], buf[offset + 1], buf[offset + 2], buf[offset + 3]
            ]);
        }
        Some(pids)
    }
}

/// Stop child command (devd → driver)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct StopChild {
    pub header: QueryHeader,
    /// PID of child to stop
    pub child_pid: u32,
}

impl StopChild {
    pub const SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, child_pid: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::STOP_CHILD, seq_id),
            child_pid,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.child_pid.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            child_pid: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        })
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
// Introspection Messages
// =============================================================================

/// List ports request (client → devd)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ListPorts {
    pub header: QueryHeader,
}

impl ListPorts {
    pub const SIZE: usize = QueryHeader::SIZE;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::LIST_PORTS, seq_id),
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        self.header.to_bytes()
    }
}

/// List services request (client → devd)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ListServices {
    pub header: QueryHeader,
}

impl ListServices {
    pub const SIZE: usize = QueryHeader::SIZE;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::LIST_SERVICES, seq_id),
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        self.header.to_bytes()
    }
}

/// Port entry in ports list response (32 bytes per entry)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PortEntry {
    /// Port name (null-padded)
    pub name: [u8; 20],
    /// Port type (see `port_type` module)
    pub port_type: u8,
    /// Port flags (see `port_flags` module)
    pub flags: u8,
    /// Owner service index (0xFF = devd itself)
    pub owner_idx: u8,
    /// Parent port index (0xFF = no parent / root)
    pub parent_idx: u8,
    /// DataPort shared memory ID (0 if no DataPort)
    pub shmem_id: u32,
    /// Owner process PID
    pub owner_pid: u32,
}

impl PortEntry {
    pub const SIZE: usize = 32;

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..20].copy_from_slice(&self.name);
        buf[20] = self.port_type;
        buf[21] = self.flags;
        buf[22] = self.owner_idx;
        buf[23] = self.parent_idx;
        buf[24..28].copy_from_slice(&self.shmem_id.to_le_bytes());
        buf[28..32].copy_from_slice(&self.owner_pid.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let mut name = [0u8; 20];
        name.copy_from_slice(&buf[0..20]);
        Some(Self {
            name,
            port_type: buf[20],
            flags: buf[21],
            owner_idx: buf[22],
            parent_idx: buf[23],
            shmem_id: u32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]),
            owner_pid: u32::from_le_bytes([buf[28], buf[29], buf[30], buf[31]]),
        })
    }

    /// Get port name as slice (up to null terminator)
    pub fn name_str(&self) -> &[u8] {
        let len = self.name.iter().position(|&b| b == 0).unwrap_or(20);
        &self.name[..len]
    }
}

/// Service state constants for introspection
pub mod service_state {
    /// Waiting for dependencies to be satisfied
    pub const PENDING: u8 = 0;
    /// Spawned, waiting for service to connect
    pub const STARTING: u8 = 1;
    /// Service is ready
    pub const READY: u8 = 2;
    /// Service exited cleanly
    pub const STOPPED: u8 = 3;
    /// Service crashed, pending restart
    pub const CRASHED: u8 = 4;
    /// Service failed permanently
    pub const FAILED: u8 = 5;
}

/// Service entry in services list response (64 bytes per entry)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ServiceEntry {
    /// Service binary name (null-padded)
    pub name: [u8; 16],
    /// Process ID (0 if not running)
    pub pid: u32,
    /// Service state (see `service_state` module)
    pub state: u8,
    /// Service index
    pub index: u8,
    /// Parent service index (0xFF = devd)
    pub parent_idx: u8,
    /// Number of children
    pub child_count: u8,
    /// Total restart count
    pub total_restarts: u32,
    /// Last state change timestamp (ms since boot)
    pub last_change: u32,
    /// Bus/port path that triggered this service's spawn (null-padded)
    /// e.g., "/kernel/bus/uart0" for bus drivers
    pub bus_path: [u8; 32],
}

impl ServiceEntry {
    pub const SIZE: usize = 64;

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..16].copy_from_slice(&self.name);
        buf[16..20].copy_from_slice(&self.pid.to_le_bytes());
        buf[20] = self.state;
        buf[21] = self.index;
        buf[22] = self.parent_idx;
        buf[23] = self.child_count;
        buf[24..28].copy_from_slice(&self.total_restarts.to_le_bytes());
        buf[28..32].copy_from_slice(&self.last_change.to_le_bytes());
        buf[32..64].copy_from_slice(&self.bus_path);
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let mut name = [0u8; 16];
        name.copy_from_slice(&buf[0..16]);
        let mut bus_path = [0u8; 32];
        bus_path.copy_from_slice(&buf[32..64]);
        Some(Self {
            name,
            pid: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            state: buf[20],
            index: buf[21],
            parent_idx: buf[22],
            child_count: buf[23],
            total_restarts: u32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]),
            last_change: u32::from_le_bytes([buf[28], buf[29], buf[30], buf[31]]),
            bus_path,
        })
    }

    /// Get service name as slice (up to null terminator)
    pub fn name_str(&self) -> &[u8] {
        let len = self.name.iter().position(|&b| b == 0).unwrap_or(16);
        &self.name[..len]
    }

    /// Get state as string
    pub fn state_str(&self) -> &'static str {
        match self.state {
            service_state::PENDING => "pending",
            service_state::STARTING => "starting",
            service_state::READY => "ready",
            service_state::STOPPED => "stopped",
            service_state::CRASHED => "crashed",
            service_state::FAILED => "failed",
            _ => "unknown",
        }
    }

    /// Get bus path as slice (up to null terminator)
    pub fn bus_path_str(&self) -> &[u8] {
        let len = self.bus_path.iter().position(|&b| b == 0).unwrap_or(32);
        &self.bus_path[..len]
    }

    /// Check if this service has a bus path (was spawned for a kernel bus)
    pub fn has_bus_path(&self) -> bool {
        self.bus_path[0] != 0
    }
}

/// Ports list response header (devd → client)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PortsListResponse {
    pub header: QueryHeader,
    /// Number of port entries following
    pub count: u16,
    pub _pad: u16,
    // Followed by: count * PortEntry
}

impl PortsListResponse {
    pub const HEADER_SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, count: u16) -> Self {
        Self {
            header: QueryHeader::new(msg::PORTS_LIST, seq_id),
            count,
            _pad: 0,
        }
    }

    pub fn header_to_bytes(&self) -> [u8; Self::HEADER_SIZE] {
        let mut buf = [0u8; Self::HEADER_SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..10].copy_from_slice(&self.count.to_le_bytes());
        buf[10..12].copy_from_slice(&[0, 0]);
        buf
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

/// Services list response header (devd → client)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ServicesListResponse {
    pub header: QueryHeader,
    /// Number of service entries following
    pub count: u16,
    pub _pad: u16,
    // Followed by: count * ServiceEntry
}

impl ServicesListResponse {
    pub const HEADER_SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, count: u16) -> Self {
        Self {
            header: QueryHeader::new(msg::SERVICES_LIST, seq_id),
            count,
            _pad: 0,
        }
    }

    pub fn header_to_bytes(&self) -> [u8; Self::HEADER_SIZE] {
        let mut buf = [0u8; Self::HEADER_SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..10].copy_from_slice(&self.count.to_le_bytes());
        buf[10..12].copy_from_slice(&[0, 0]);
        buf
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

// =============================================================================
// Service Info Query (shell → devd → driver)
// =============================================================================

/// Query service info by name (client → devd)
///
/// Requests detailed info from a service. devd forwards this to the driver,
/// which responds with text describing its state.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct QueryServiceInfo {
    pub header: QueryHeader,
    /// Length of service name
    pub name_len: u8,
    pub _pad: [u8; 3],
    // Followed by: service name bytes
}

impl QueryServiceInfo {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::QUERY_SERVICE_INFO, seq_id),
            name_len: 0,
            _pad: [0; 3],
        }
    }

    /// Serialize to buffer, returns total length written
    pub fn write_to(&self, buf: &mut [u8], name: &[u8]) -> Option<usize> {
        let total_len = Self::FIXED_SIZE + name.len();
        if buf.len() < total_len || name.len() > 255 {
            return None;
        }

        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8] = name.len() as u8;
        buf[9..12].copy_from_slice(&[0, 0, 0]);
        buf[Self::FIXED_SIZE..total_len].copy_from_slice(name);

        Some(total_len)
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }

        let header = QueryHeader::from_bytes(buf)?;
        let name_len = buf[8] as usize;

        let total_len = Self::FIXED_SIZE + name_len;
        if buf.len() < total_len {
            return None;
        }

        let name = &buf[Self::FIXED_SIZE..total_len];

        Some((
            Self {
                header,
                name_len: name_len as u8,
                _pad: [0; 3],
            },
            name,
        ))
    }
}

/// Service info response (driver → devd → client)
///
/// Contains UTF-8 text describing the service's current state and configuration.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ServiceInfoResult {
    pub header: QueryHeader,
    /// Result code (0 = success, negative = error)
    pub result: i32,
    /// Length of info text
    pub info_len: u16,
    pub _pad: u16,
    // Followed by: UTF-8 info text bytes
}

impl ServiceInfoResult {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 8;

    pub fn new(seq_id: u32, result: i32) -> Self {
        Self {
            header: QueryHeader::new(msg::SERVICE_INFO_RESULT, seq_id),
            result,
            info_len: 0,
            _pad: 0,
        }
    }

    pub fn success(seq_id: u32, info_len: u16) -> Self {
        Self {
            header: QueryHeader::new(msg::SERVICE_INFO_RESULT, seq_id),
            result: error::OK,
            info_len,
            _pad: 0,
        }
    }

    /// Serialize to buffer with info text, returns total length written
    pub fn write_to(&self, buf: &mut [u8], info: &[u8]) -> Option<usize> {
        let total_len = Self::FIXED_SIZE + info.len();
        if buf.len() < total_len {
            return None;
        }

        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.result.to_le_bytes());
        buf[12..14].copy_from_slice(&(info.len() as u16).to_le_bytes());
        buf[14..16].copy_from_slice(&[0, 0]);
        buf[Self::FIXED_SIZE..total_len].copy_from_slice(info);

        Some(total_len)
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }

        let header = QueryHeader::from_bytes(buf)?;
        let result = i32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let info_len = u16::from_le_bytes([buf[12], buf[13]]) as usize;

        let total_len = Self::FIXED_SIZE + info_len;
        if buf.len() < total_len {
            return None;
        }

        let info = &buf[Self::FIXED_SIZE..total_len];

        Some((
            Self {
                header,
                result,
                info_len: info_len as u16,
                _pad: 0,
            },
            info,
        ))
    }
}

// =============================================================================
// Log Messages
// =============================================================================

/// Log level constants
pub mod log_level {
    pub const ERROR: u8 = 0;
    pub const WARN: u8 = 1;
    pub const INFO: u8 = 2;
    pub const DEBUG: u8 = 3;
}

/// Log message (driver → devd)
///
/// Drivers send log messages to devd for buffering and optional display.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LogMessage {
    pub header: QueryHeader,
    /// Log level (see log_level module)
    pub level: u8,
    /// Length of message text
    pub msg_len: u8,
    pub _pad: [u8; 2],
    // Followed by: message text (UTF-8)
}

impl LogMessage {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 4;
    pub const MAX_MSG_LEN: usize = 200;

    pub fn new(seq_id: u32, level: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::LOG_MESSAGE, seq_id),
            level,
            msg_len: 0,
            _pad: [0; 2],
        }
    }

    pub fn write_to(&self, buf: &mut [u8], message: &[u8]) -> Option<usize> {
        let len = message.len().min(Self::MAX_MSG_LEN);
        let total = Self::FIXED_SIZE + len;
        if buf.len() < total {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8] = self.level;
        buf[9] = len as u8;
        buf[10..12].copy_from_slice(&[0, 0]);
        buf[Self::FIXED_SIZE..Self::FIXED_SIZE + len].copy_from_slice(&message[..len]);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }
        let header = QueryHeader::from_bytes(buf)?;
        let level = buf[8];
        let msg_len = buf[9] as usize;
        let total = Self::FIXED_SIZE + msg_len;
        if buf.len() < total {
            return None;
        }
        let message = &buf[Self::FIXED_SIZE..total];
        Some((Self { header, level, msg_len: msg_len as u8, _pad: [0; 2] }, message))
    }
}

/// Log query (shell → devd)
///
/// Request log history from devd.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LogQuery {
    pub header: QueryHeader,
    /// Maximum number of log entries to return
    pub max_count: u8,
    pub _pad: [u8; 3],
}

impl LogQuery {
    pub const SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, max_count: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::LOG_QUERY, seq_id),
            max_count,
            _pad: [0; 3],
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8] = self.max_count;
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            max_count: buf[8],
            _pad: [0; 3],
        })
    }
}

/// Log control commands
pub mod log_cmd {
    /// Enable live logging to console
    pub const ENABLE: u8 = 1;
    /// Disable live logging
    pub const DISABLE: u8 = 2;
}

/// Log control (shell → devd)
///
/// Control logging behavior (enable/disable live output).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LogControl {
    pub header: QueryHeader,
    /// Command (see log_cmd module)
    pub command: u8,
    pub _pad: [u8; 3],
}

impl LogControl {
    pub const SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, command: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::LOG_CONTROL, seq_id),
            command,
            _pad: [0; 3],
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8] = self.command;
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            command: buf[8],
            _pad: [0; 3],
        })
    }
}

/// Log history response (devd → shell)
///
/// Contains buffered log messages.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct LogHistory {
    pub header: QueryHeader,
    /// Result code (0 = success)
    pub result: i32,
    /// Number of log entries in response
    pub count: u8,
    /// Whether live logging is currently enabled
    pub live_enabled: u8,
    /// Total length of log text following header
    pub text_len: u16,
    // Followed by: log text (newline-separated entries)
}

impl LogHistory {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 8;

    pub fn new(seq_id: u32, count: u8, live_enabled: bool) -> Self {
        Self {
            header: QueryHeader::new(msg::LOG_HISTORY, seq_id),
            result: 0,
            count,
            live_enabled: if live_enabled { 1 } else { 0 },
            text_len: 0,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], text: &[u8]) -> Option<usize> {
        let total = Self::FIXED_SIZE + text.len();
        if buf.len() < total {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.result.to_le_bytes());
        buf[12] = self.count;
        buf[13] = self.live_enabled;
        buf[14..16].copy_from_slice(&(text.len() as u16).to_le_bytes());
        buf[Self::FIXED_SIZE..total].copy_from_slice(text);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }
        let header = QueryHeader::from_bytes(buf)?;
        let result = i32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let count = buf[12];
        let live_enabled = buf[13];
        let text_len = u16::from_le_bytes([buf[14], buf[15]]) as usize;
        let total = Self::FIXED_SIZE + text_len;
        if buf.len() < total {
            return None;
        }
        let text = &buf[Self::FIXED_SIZE..total];
        Some((Self { header, result, count, live_enabled, text_len: text_len as u16 }, text))
    }
}

// =============================================================================
// Orchestration Messages
// =============================================================================

/// Rich partition information for ReportPartitions
///
/// Contains all partition metadata needed by devd to make policy decisions.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PartitionInfoMsg {
    /// Partition index (0-based position in partition table)
    pub index: u8,
    /// MBR partition type (0x0B=FAT32, 0x83=Linux) or GPT type index
    pub part_type: u8,
    /// Bootable flag (1 = bootable, 0 = not)
    pub bootable: u8,
    /// Reserved padding
    pub _pad: u8,
    /// Start LBA (64-bit for GPT support)
    pub start_lba: u64,
    /// Size in sectors (64-bit for GPT support)
    pub size_sectors: u64,
    /// GPT partition GUID (zeros for MBR)
    pub guid: [u8; 16],
    /// GPT partition label (zeros for MBR, null-terminated UTF-8)
    pub label: [u8; 36],
}

impl PartitionInfoMsg {
    pub const SIZE: usize = 4 + 8 + 8 + 16 + 36; // 72 bytes

    pub fn new() -> Self {
        Self {
            index: 0,
            part_type: 0,
            bootable: 0,
            _pad: 0,
            start_lba: 0,
            size_sectors: 0,
            guid: [0; 16],
            label: [0; 36],
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0] = self.index;
        buf[1] = self.part_type;
        buf[2] = self.bootable;
        buf[3] = self._pad;
        buf[4..12].copy_from_slice(&self.start_lba.to_le_bytes());
        buf[12..20].copy_from_slice(&self.size_sectors.to_le_bytes());
        buf[20..36].copy_from_slice(&self.guid);
        buf[36..72].copy_from_slice(&self.label);
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            index: buf[0],
            part_type: buf[1],
            bootable: buf[2],
            _pad: buf[3],
            start_lba: u64::from_le_bytes([
                buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
            ]),
            size_sectors: u64::from_le_bytes([
                buf[12], buf[13], buf[14], buf[15], buf[16], buf[17], buf[18], buf[19],
            ]),
            guid: {
                let mut g = [0u8; 16];
                g.copy_from_slice(&buf[20..36]);
                g
            },
            label: {
                let mut l = [0u8; 36];
                l.copy_from_slice(&buf[36..72]);
                l
            },
        })
    }

    /// Get label as string slice
    pub fn label_str(&self) -> &str {
        let len = self.label.iter().position(|&b| b == 0).unwrap_or(36);
        core::str::from_utf8(&self.label[..len]).unwrap_or("")
    }
}

/// Attach disk command (devd → partd)
///
/// Tells partd to connect to a block device and scan for partitions.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct AttachDisk {
    pub header: QueryHeader,
    /// DataPort shared memory ID
    pub shmem_id: u32,
    /// Length of source port name
    pub source_len: u8,
    /// Reserved padding
    pub _pad: [u8; 3],
    /// Block size in bytes
    pub block_size: u32,
    /// Total number of blocks
    pub block_count: u64,
    // Followed by: source port name bytes (e.g., "disk0:")
}

impl AttachDisk {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 4 + 4 + 4 + 8; // 28 bytes

    pub fn new(seq_id: u32, shmem_id: u32, block_size: u32, block_count: u64) -> Self {
        Self {
            header: QueryHeader::new(msg::ATTACH_DISK, seq_id),
            shmem_id,
            source_len: 0,
            _pad: [0; 3],
            block_size,
            block_count,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], source: &[u8]) -> Option<usize> {
        let total = Self::FIXED_SIZE + source.len();
        if buf.len() < total || source.len() > 255 {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.shmem_id.to_le_bytes());
        buf[12] = source.len() as u8;
        buf[13..16].copy_from_slice(&[0; 3]);
        buf[16..20].copy_from_slice(&self.block_size.to_le_bytes());
        buf[20..28].copy_from_slice(&self.block_count.to_le_bytes());
        buf[28..total].copy_from_slice(source);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }
        let header = QueryHeader::from_bytes(buf)?;
        let shmem_id = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let source_len = buf[12] as usize;
        let block_size = u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]);
        let block_count = u64::from_le_bytes([
            buf[20], buf[21], buf[22], buf[23], buf[24], buf[25], buf[26], buf[27],
        ]);
        let total = Self::FIXED_SIZE + source_len;
        if buf.len() < total {
            return None;
        }
        let source = &buf[Self::FIXED_SIZE..total];
        Some((
            Self {
                header,
                shmem_id,
                source_len: source_len as u8,
                _pad: [0; 3],
                block_size,
                block_count,
            },
            source,
        ))
    }
}

/// Mount ready notification (devd → vfsd)
///
/// Sent when a Filesystem port is registered, so vfsd can connect to it.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MountReady {
    pub header: QueryHeader,
    /// DataPort shared memory ID of the filesystem port
    pub shmem_id: u32,
    /// Length of port name
    pub name_len: u8,
    /// Reserved padding
    pub _pad: [u8; 3],
    // Followed by: port name bytes (e.g., "fat0:")
}

impl MountReady {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 4 + 4; // 16 bytes

    pub fn new(seq_id: u32, shmem_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::MOUNT_READY, seq_id),
            shmem_id,
            name_len: 0,
            _pad: [0; 3],
        }
    }

    pub fn write_to(&self, buf: &mut [u8], port_name: &[u8]) -> Option<usize> {
        let total = Self::FIXED_SIZE + port_name.len();
        if buf.len() < total || port_name.len() > 255 {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.shmem_id.to_le_bytes());
        buf[12] = port_name.len() as u8;
        buf[13..16].copy_from_slice(&[0; 3]);
        buf[Self::FIXED_SIZE..total].copy_from_slice(port_name);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }
        let header = QueryHeader::from_bytes(buf)?;
        let shmem_id = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let name_len = buf[12] as usize;
        let total = Self::FIXED_SIZE + name_len;
        if buf.len() < total {
            return None;
        }
        let port_name = &buf[Self::FIXED_SIZE..total];
        Some((
            Self {
                header,
                shmem_id,
                name_len: name_len as u8,
                _pad: [0; 3],
            },
            port_name,
        ))
    }
}

/// Report partitions message (partd → devd)
///
/// partd sends this after scanning a disk's partition table.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ReportPartitions {
    pub header: QueryHeader,
    /// Source disk's DataPort shmem_id
    pub source_shmem_id: u32,
    /// Number of partitions found
    pub count: u8,
    /// Partition scheme (see part_scheme module)
    pub scheme: u8,
    /// Reserved padding
    pub _pad: [u8; 2],
    // Followed by: count * PartitionInfoMsg entries
}

impl ReportPartitions {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 8;

    pub fn new(seq_id: u32, source_shmem_id: u32, count: u8, scheme: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::REPORT_PARTITIONS, seq_id),
            source_shmem_id,
            count,
            scheme,
            _pad: [0; 2],
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::FIXED_SIZE] {
        let mut buf = [0u8; Self::FIXED_SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..12].copy_from_slice(&self.source_shmem_id.to_le_bytes());
        buf[12] = self.count;
        buf[13] = self.scheme;
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            source_shmem_id: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            count: buf[12],
            scheme: buf[13],
            _pad: [0; 2],
        })
    }
}

/// Register partition command (devd → partd)
///
/// devd tells partd to create a DataPort for a partition with an assigned name.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RegisterPartitionMsg {
    pub header: QueryHeader,
    /// Partition index from ReportPartitions
    pub index: u8,
    /// Length of assigned name
    pub name_len: u8,
    /// Reserved padding
    pub _pad: [u8; 2],
    // Followed by: assigned name bytes (e.g., "part0:")
}

impl RegisterPartitionMsg {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32, index: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::REGISTER_PARTITION, seq_id),
            index,
            name_len: 0,
            _pad: [0; 2],
        }
    }

    pub fn write_to(&self, buf: &mut [u8], name: &[u8]) -> Option<usize> {
        let total = Self::FIXED_SIZE + name.len();
        if buf.len() < total || name.len() > 255 {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8] = self.index;
        buf[9] = name.len() as u8;
        buf[10..12].copy_from_slice(&[0; 2]);
        buf[12..total].copy_from_slice(name);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }
        let header = QueryHeader::from_bytes(buf)?;
        let index = buf[8];
        let name_len = buf[9] as usize;
        let total = Self::FIXED_SIZE + name_len;
        if buf.len() < total {
            return None;
        }
        let name = &buf[Self::FIXED_SIZE..total];
        Some((
            Self {
                header,
                index,
                name_len: name_len as u8,
                _pad: [0; 2],
            },
            name,
        ))
    }
}

/// Partition ready notification (partd → devd)
///
/// partd confirms a partition DataPort is ready.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PartitionReadyMsg {
    pub header: QueryHeader,
    /// Length of partition name
    pub name_len: u8,
    /// Reserved padding
    pub _pad: [u8; 3],
    /// DataPort shared memory ID
    pub shmem_id: u32,
    // Followed by: partition name bytes
}

impl PartitionReadyMsg {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 8;

    pub fn new(seq_id: u32, shmem_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::PARTITION_READY, seq_id),
            name_len: 0,
            _pad: [0; 3],
            shmem_id,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], name: &[u8]) -> Option<usize> {
        let total = Self::FIXED_SIZE + name.len();
        if buf.len() < total || name.len() > 255 {
            return None;
        }
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8] = name.len() as u8;
        buf[9..12].copy_from_slice(&[0; 3]);
        buf[12..16].copy_from_slice(&self.shmem_id.to_le_bytes());
        buf[16..total].copy_from_slice(name);
        Some(total)
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }
        let header = QueryHeader::from_bytes(buf)?;
        let name_len = buf[8] as usize;
        let shmem_id = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);
        let total = Self::FIXED_SIZE + name_len;
        if buf.len() < total {
            return None;
        }
        let name = &buf[Self::FIXED_SIZE..total];
        Some((
            Self {
                header,
                name_len: name_len as u8,
                _pad: [0; 3],
                shmem_id,
            },
            name,
        ))
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
