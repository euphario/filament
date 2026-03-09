//! Query Protocol
//!
//! Message types for communication between devd, drivers, and clients.
//! Used for port registration, service introspection, spawn commands,
//! and logging.

// =============================================================================
// Message Header
// =============================================================================

/// Query header flags (u16 bitfield).
pub mod query_flags {
    /// Response is end-of-line: this node has no answer and no children to forward to.
    pub const EOL: u16 = 0x0001;
    /// Response contains an error.
    pub const ERROR: u16 = 0x0002;
    /// More responses will follow for this seq_id (partial delivery).
    pub const PARTIAL: u16 = 0x0004;
    /// Payload contains an address prefix (address_len + address before key).
    pub const ADDRESSED: u16 = 0x0008;
}

/// Query message header (8 bytes)
///
/// All query messages start with this header.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct QueryHeader {
    /// Message type (see `msg` module)
    pub msg_type: u16,
    /// Flags (see `query_flags` module)
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

    pub const fn with_flags(msg_type: u16, flags: u16, seq_id: u32) -> Self {
        Self { msg_type, flags, seq_id }
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
    // 0x0100-0x0102 - removed (legacy device registry)
    // 0x0103, 0x0104 - reserved (legacy REGISTER_PORT, UNREGISTER_PORT)
    /// Driver state change notification (driver → devd)
    pub const STATE_CHANGE: u16 = 0x0105;
    /// Get spawn context (child → devd) - returns the port that triggered this spawn
    pub const GET_SPAWN_CONTEXT: u16 = 0x0106;
    /// Query port info by name (returns shmem_id if DataPort-backed)
    pub const QUERY_PORT: u16 = 0x0107;
    /// Update shmem_id for an existing port
    pub const UPDATE_PORT_SHMEM_ID: u16 = 0x0108;
    /// Register a filesystem mount point (driver → devd)
    pub const REGISTER_MOUNT: u16 = 0x0109;
    /// Resolve a path to a mount transport (client → devd)
    pub const RESOLVE_PATH: u16 = 0x010A;
    /// List all registered mount points (client → devd)
    pub const LIST_MOUNTS: u16 = 0x010B;
    /// Register port with full PortInfo (unified enumeration)
    pub const REGISTER_PORT_INFO: u16 = 0x010C;
    /// Set port state (driver → devd)
    pub const SET_PORT_STATE: u16 = 0x010D;

    // Query messages (client → devd)
    // 0x0200-0x0202 - removed (legacy device queries)
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
    /// Get a configuration value from driver (response via SERVICE_INFO_RESULT)
    pub const CONFIG_GET: u16 = 0x0420;
    /// Set a configuration value on driver (response via SERVICE_INFO_RESULT)
    pub const CONFIG_SET: u16 = 0x0421;

    // Response messages
    // 0x0300-0x0302 - removed (legacy device responses)
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
    /// Response containing mount list
    pub const MOUNTS_LIST: u16 = 0x0309;
    /// Response containing path resolution result
    pub const RESOLVE_RESULT: u16 = 0x030A;
    /// Error response
    pub const ERROR: u16 = 0x03FF;
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
    pub const PCIE: u8 = 9;
    pub const UART: u8 = 10;
    pub const KLOG: u8 = 11;
    pub const ETHERNET: u8 = 12;
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
        use crate::wire::WireWriter;
        let info_bytes: &[u8; 112] = unsafe { &*(info as *const abi::PortInfo as *const [u8; 112]) };
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_u32(self.shmem_id)?;
        w.put_u32(0)?; // pad
        w.put_bytes(info_bytes)?;
        Some(w.finish())
    }

    /// Deserialize header from bytes (PortInfo follows at offset 16)
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let shmem_id = r.u32()?;
        r.skip(4)?; // pad
        let info_bytes = r.bytes(112)?;
        Some((Self { header, shmem_id, _pad: 0 }, info_bytes))
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
    /// Port ID of the trigger port (monotonic, from port registry)
    pub port_id: u8,
    // Followed by: port_name bytes, then metadata bytes,
    // then context_kv_count (u8), then KV entries (each: key_len(u8) + key + value_len(u8) + value)
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
            port_id: 0xFF,
        }
    }

    pub fn write_to(&self, buf: &mut [u8], port_name: &[u8]) -> Option<usize> {
        self.write_to_with_metadata(buf, port_name, &[])
    }

    pub fn write_to_with_metadata(&self, buf: &mut [u8], port_name: &[u8], metadata: &[u8]) -> Option<usize> {
        self.write_to_full(buf, port_name, metadata, &[])
    }

    /// Serialize with port name, metadata, and context key-value pairs.
    ///
    /// KV pairs are appended as: kv_count(u8) + (key_len(u8) + key + value_len(u8) + value)*
    pub fn write_to_full(
        &self,
        buf: &mut [u8],
        port_name: &[u8],
        metadata: &[u8],
        context_kvs: &[(&[u8], &[u8])],
    ) -> Option<usize> {
        use crate::wire::WireWriter;
        if port_name.len() > 255 || metadata.len() > 255 {
            return None;
        }
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_i32(self.result)?;
        w.put_u8(self.port_type)?;
        w.put_u8(port_name.len() as u8)?;
        w.put_u8(metadata.len() as u8)?;
        w.put_u8(self.port_id)?;
        w.put_bytes(port_name)?;
        w.put_bytes(metadata)?;
        w.put_kv_list(context_kvs)?;
        Some(w.finish())
    }

    /// Parse header, port_name, and metadata from bytes.
    ///
    /// Context KV pairs (if present) can be parsed separately via `parse_context_kvs()`.
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8], &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let result = r.i32()?;
        let port_type = r.u8()?;
        let port_name_len = r.u8()?;
        let metadata_len = r.u8()?;
        let port_id = r.u8()?;
        let port_name = r.bytes(port_name_len as usize)?;
        let metadata = r.bytes(metadata_len as usize)?;

        let resp = Self {
            header,
            result,
            port_type,
            port_name_len,
            metadata_len,
            port_id,
        };
        Some((resp, port_name, metadata))
    }

    /// Parse context key-value pairs from the trailing bytes of a spawn context response.
    ///
    /// Call after `from_bytes()`. `buf` is the full message buffer.
    /// Returns a small fixed-size array of (key, value) pairs.
    pub fn parse_context_kvs(buf: &[u8], port_name_len: usize, metadata_len: usize)
        -> ([([u8; 32], u8, [u8; 64], u8); 4], usize)
    {
        use crate::wire::WireReader;
        let mut result = [([0u8; 32], 0u8, [0u8; 64], 0u8); 4];
        let kv_start = Self::HEADER_SIZE + port_name_len + metadata_len;
        if buf.len() <= kv_start {
            return (result, 0);
        }
        let mut r = WireReader::new(&buf[kv_start..]);
        let mut count = 0;
        let parsed = r.kv_list(|key, value| {
            if count < 4 {
                let kl = key.len().min(32);
                result[count].0[..kl].copy_from_slice(&key[..kl]);
                result[count].1 = kl as u8;
                let vl = value.len().min(64);
                result[count].2[..vl].copy_from_slice(&value[..vl]);
                result[count].3 = vl as u8;
            }
            count += 1;
        });
        let n = match parsed {
            Some(_) => count.min(4),
            None => count.min(4),
        };
        (result, n)
    }

    /// Parse trailing shmem_id appended after the KV section by parent relay.
    ///
    /// The parent relay appends 4 bytes of shmem_id (LE) after the standard
    /// SpawnContextResponse + KV data. Root-mode devd responses omit this,
    /// so returns 0 if there aren't enough trailing bytes.
    pub fn parse_trailing_shmem_id(buf: &[u8], port_name_len: usize, metadata_len: usize) -> u32 {
        use crate::wire::WireReader;
        let kv_start = Self::HEADER_SIZE + port_name_len + metadata_len;
        if buf.len() <= kv_start {
            return 0;
        }
        let mut r = WireReader::new(&buf[kv_start..]);
        // Skip past the KV list
        if r.kv_list(|_, _| {}).is_none() {
            return 0;
        }
        // Read trailing shmem_id
        r.u32().unwrap_or(0)
    }
}

// =============================================================================
// Port Query Messages (client → devd)
// =============================================================================

/// Query port information by name or port_id
///
/// Allows consumers to discover DataPort shmem_id for a named port.
/// This replaces hardcoded shmem_id constants in drivers.
///
/// When `port_id != 0xFF`, devd uses port_id for lookup (unambiguous).
/// Otherwise falls back to name-based lookup.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct QueryPort {
    pub header: QueryHeader,
    /// Length of port name
    pub name_len: u8,
    /// Port ID hint (0xFF = use name-based lookup)
    pub port_id: u8,
    pub _pad: [u8; 2],
    // Followed by: port name bytes
}

impl QueryPort {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 4;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::QUERY_PORT, seq_id),
            name_len: 0,
            port_id: 0xFF,
            _pad: [0; 2],
        }
    }

    /// Create a query that looks up by port_id (unambiguous)
    pub fn by_id(seq_id: u32, port_id: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::QUERY_PORT, seq_id),
            name_len: 0,
            port_id,
            _pad: [0; 2],
        }
    }

    /// Serialize to buffer, returns total length written
    pub fn write_to(&self, buf: &mut [u8], name: &[u8]) -> Option<usize> {
        use crate::wire::WireWriter;
        if name.len() > 255 { return None; }
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_u8(name.len() as u8)?;
        w.put_u8(self.port_id)?;
        w.put_zeros(2)?;
        w.put_bytes(name)?;
        Some(w.finish())
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let name_len = r.u8()?;
        let port_id = r.u8()?;
        r.skip(2)?;
        let name = r.bytes(name_len as usize)?;
        Some((Self { header, name_len, port_id, _pad: [0; 2] }, name))
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
        use crate::wire::WireWriter;
        if name.len() > 255 { return None; }
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_u32(self.shmem_id)?;
        w.put_u8(name.len() as u8)?;
        w.put_zeros(3)?;
        w.put_bytes(name)?;
        Some(w.finish())
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let shmem_id = r.u32()?;
        let name_len = r.u8()?;
        r.skip(3)?;
        let name = r.bytes(name_len as usize)?;
        Some((Self { header, shmem_id, name_len, _pad: [0; 3] }, name))
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
        use crate::wire::WireWriter;
        if name.len() > 255 { return None; }
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_u8(self.state)?;
        w.put_u8(name.len() as u8)?;
        w.put_zeros(2)?;
        w.put_bytes(name)?;
        Some(w.finish())
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let state = r.u8()?;
        let name_len = r.u8()?;
        r.skip(2)?;
        let name = r.bytes(name_len as usize)?;
        Some((Self { header, state, name_len, _pad: [0; 2] }, name))
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
        use crate::wire::WireWriter;
        let mut w = WireWriter::new(buf);
        w.put_u8(self.mode)?;
        w.put_u8(self.port_type)?;
        w.put_u16(self.class_filter)?;
        w.put_u8(pattern.len() as u8)?;
        w.put_zeros(3)?;
        w.put_bytes(pattern)?;
        Some(w.finish())
    }

    /// Parse filter from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let mode = r.u8()?;
        let port_type = r.u8()?;
        let class_filter = r.u16()?;
        let pattern_len = r.u8()?;
        r.skip(3)?;
        let pattern = r.bytes(pattern_len as usize)?;
        Some((Self { mode, port_type, class_filter, pattern_len, _pad: [0; 3] }, pattern))
    }
}

// =============================================================================
// Spawn Child Context (embedded in SPAWN_CHILD message)
// =============================================================================

/// Context data embedded in a SPAWN_CHILD command so the parent driver
/// can answer GET_SPAWN_CONTEXT locally without querying devd.
#[derive(Clone, Copy)]
pub struct SpawnChildContext {
    /// Port type of the trigger port
    pub port_type: u8,
    /// Port ID of the trigger port
    pub port_id: u8,
    /// Length of metadata bytes
    pub metadata_len: u8,
    /// Number of context KV pairs
    pub kv_count: u8,
    /// Shared memory ID for the trigger port's DataPort
    pub shmem_id: u32,
    /// Metadata bytes from port registration
    pub metadata: [u8; 64],
    /// Context key names
    pub kv_keys: [[u8; 32]; 4],
    /// Context key name lengths
    pub kv_keys_len: [u8; 4],
    /// Context values
    pub kv_values: [[u8; 64]; 4],
    /// Context value lengths
    pub kv_values_len: [u8; 4],
}

impl SpawnChildContext {
    pub const fn empty() -> Self {
        Self {
            port_type: 0,
            port_id: 0xFF,
            metadata_len: 0,
            kv_count: 0,
            shmem_id: 0,
            metadata: [0u8; 64],
            kv_keys: [[0u8; 32]; 4],
            kv_keys_len: [0u8; 4],
            kv_values: [[0u8; 64]; 4],
            kv_values_len: [0u8; 4],
        }
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
/// Variable-length: header + fixed fields + binary_name + filter + context
///
/// When `context_len > 0`, a context section follows the filter:
/// ```text
/// context_section: port_type(1) port_id(1) metadata_len(1) kv_count(1) shmem_id(4)
///                  metadata(metadata_len bytes)
///                  kv_entries: (key_len(1) + key + value_len(1) + value) * kv_count
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SpawnChild {
    pub header: QueryHeader,
    /// Length of binary name
    pub binary_len: u8,
    /// Length of filter (PortFilter + pattern)
    pub filter_len: u8,
    /// Length of context section (0 = no context)
    pub context_len: u8,
    /// Priority for spawned child (abi::priority::*, INHERIT=0xFF means use parent's)
    pub priority: u8,
    /// Capability bits for spawned child (0 = inherit parent's)
    pub caps: u64,
    // Followed by: binary name bytes, then PortFilter + pattern, then context section
}

impl SpawnChild {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 4 + 8;

    pub fn new(seq_id: u32) -> Self {
        Self {
            header: QueryHeader::new(msg::SPAWN_CHILD, seq_id),
            binary_len: 0,
            filter_len: 0,
            context_len: 0,
            priority: abi::priority::INHERIT,
            caps: 0,
        }
    }

    pub fn with_caps(seq_id: u32, caps: u64) -> Self {
        Self {
            header: QueryHeader::new(msg::SPAWN_CHILD, seq_id),
            binary_len: 0,
            filter_len: 0,
            context_len: 0,
            priority: abi::priority::INHERIT,
            caps,
        }
    }

    pub fn with_caps_and_priority(seq_id: u32, caps: u64, priority: u8) -> Self {
        Self {
            header: QueryHeader::new(msg::SPAWN_CHILD, seq_id),
            binary_len: 0,
            filter_len: 0,
            context_len: 0,
            priority,
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
        use crate::wire::WireWriter;
        let filter_total = PortFilter::FIXED_SIZE + pattern.len();
        if binary.len() > 255 || filter_total > 255 {
            return None;
        }
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_u8(binary.len() as u8)?;
        w.put_u8(filter_total as u8)?;
        w.put_u8(self.context_len)?;
        w.put_u8(self.priority)?;
        w.put_u64(self.caps)?;
        w.put_bytes(binary)?;
        // Reserve space for PortFilter — written below via its own write_to
        w.put_zeros(filter_total)?;
        let header_end = w.finish();
        // PortFilter writes into the reserved region
        filter.write_to(&mut buf[header_end - filter_total..], pattern)?;
        Some(header_end)
    }

    /// Serialize to buffer with filter and context section.
    ///
    /// Context section layout:
    /// ```text
    /// port_type(1) port_id(1) metadata_len(1) kv_count(1) shmem_id(4)
    /// metadata(metadata_len bytes)
    /// kv_entries: (key_len(1) + key + value_len(1) + value) * kv_count
    /// ```
    pub fn write_to_with_context(
        &self,
        buf: &mut [u8],
        binary: &[u8],
        filter: &PortFilter,
        pattern: &[u8],
        ctx: &SpawnChildContext,
    ) -> Option<usize> {
        use crate::wire::WireWriter;
        // Calculate context section size for the header's context_len field
        let mut ctx_size: usize = 8; // port_type + port_id + metadata_len + kv_count + shmem_id
        ctx_size += ctx.metadata_len as usize;
        for i in 0..ctx.kv_count as usize {
            ctx_size += 1 + ctx.kv_keys_len[i] as usize + 1 + ctx.kv_values_len[i] as usize;
        }
        if ctx_size > 255 { return None; }

        let filter_total = PortFilter::FIXED_SIZE + pattern.len();
        if binary.len() > 255 || filter_total > 255 {
            return None;
        }

        // Write header + binary
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_u8(binary.len() as u8)?;
        w.put_u8(filter_total as u8)?;
        w.put_u8(ctx_size as u8)?;
        w.put_u8(self.priority)?;
        w.put_u64(self.caps)?;
        w.put_bytes(binary)?;
        // Reserve space for filter (written via its own write_to below)
        w.put_zeros(filter_total)?;
        let filter_end = w.position();

        // Write context section
        w.put_u8(ctx.port_type)?;
        w.put_u8(ctx.port_id)?;
        w.put_u8(ctx.metadata_len)?;
        w.put_u8(ctx.kv_count)?;
        w.put_u32(ctx.shmem_id)?;
        let mlen = ctx.metadata_len as usize;
        w.put_bytes(&ctx.metadata[..mlen])?;
        for i in 0..ctx.kv_count as usize {
            let klen = ctx.kv_keys_len[i] as usize;
            w.put_len_u8_bytes(&ctx.kv_keys[i][..klen])?;
            let vlen = ctx.kv_values_len[i] as usize;
            w.put_len_u8_bytes(&ctx.kv_values[i][..vlen])?;
        }
        let total = w.finish();

        // Write PortFilter into the reserved region
        filter.write_to(&mut buf[filter_end - filter_total..], pattern)?;

        Some(total)
    }

    /// Legacy: Serialize with port name (backwards compat, uses exact filter)
    pub fn write_to(&self, buf: &mut [u8], binary: &[u8], trigger_port: &[u8]) -> Option<usize> {
        self.write_exact(buf, binary, trigger_port)
    }

    /// Parse from buffer.
    ///
    /// Returns (header, binary_name, filter_bytes). Use `parse_context()` to
    /// extract the optional context section.
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8], &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let binary_len = r.u8()?;
        let filter_len = r.u8()?;
        let context_len = r.u8()?;
        let priority = r.u8()?;
        let caps = r.u64()?;
        let binary = r.bytes(binary_len as usize)?;
        let filter_bytes = r.bytes(filter_len as usize)?;
        // Validate that context section is also present
        r.skip(context_len as usize)?;

        Some((
            Self {
                header,
                binary_len,
                filter_len,
                context_len,
                priority,
                caps,
            },
            binary,
            filter_bytes,
        ))
    }

    /// Parse the context section from a SPAWN_CHILD message buffer.
    ///
    /// Call after `from_bytes()`. Returns None if context_len == 0.
    pub fn parse_context(buf: &[u8]) -> Option<SpawnChildContext> {
        use crate::wire::WireReader;
        if buf.len() < Self::FIXED_SIZE {
            return None;
        }
        let binary_len = buf[8] as usize;
        let filter_len = buf[9] as usize;
        let context_len = buf[10] as usize;
        if context_len == 0 {
            return None;
        }
        let ctx_start = Self::FIXED_SIZE + binary_len + filter_len;
        if buf.len() < ctx_start + context_len {
            return None;
        }

        let mut r = WireReader::new(&buf[ctx_start..ctx_start + context_len]);
        let port_type = r.u8()?;
        let port_id = r.u8()?;
        let metadata_len = r.u8()?;
        let kv_count = r.u8()?;
        let shmem_id = r.u32()?;

        let mut result = SpawnChildContext {
            port_type, port_id, metadata_len, kv_count, shmem_id,
            metadata: [0u8; 64],
            kv_keys: [[0u8; 32]; 4],
            kv_keys_len: [0u8; 4],
            kv_values: [[0u8; 64]; 4],
            kv_values_len: [0u8; 4],
        };

        let mlen = (metadata_len as usize).min(64);
        let meta = r.bytes(mlen)?;
        result.metadata[..mlen].copy_from_slice(meta);

        let kv_n = (kv_count as usize).min(4);
        for i in 0..kv_n {
            let key = r.len_u8_bytes()?;
            let klen = key.len().min(32);
            result.kv_keys[i][..klen].copy_from_slice(&key[..klen]);
            result.kv_keys_len[i] = klen as u8;
            let value = r.len_u8_bytes()?;
            let vlen = value.len().min(64);
            result.kv_values[i][..vlen].copy_from_slice(&value[..vlen]);
            result.kv_values_len[i] = vlen as u8;
        }

        Some(result)
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
        use crate::wire::WireWriter;
        if child_pids.len() > Self::MAX_CHILDREN { return None; }
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_i32(self.result)?;
        w.put_u8(child_pids.len() as u8)?;
        w.put_u8(self.match_count)?;
        w.put_zeros(2)?;
        for &pid in child_pids {
            w.put_u32(pid)?;
        }
        Some(w.finish())
    }

    /// Legacy: to_bytes for single spawn (backwards compat)
    pub fn to_bytes(&self) -> [u8; 20] {
        use crate::wire::WireWriter;
        let mut buf = [0u8; 20];
        let mut w = WireWriter::new(&mut buf);
        w.put_query_header(&self.header).unwrap();
        w.put_i32(self.result).unwrap();
        w.put_u8(self.spawn_count).unwrap();
        w.put_u8(self.match_count).unwrap();
        w.put_zeros(2).unwrap();
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let result = r.i32()?;
        let spawn_count = r.u8()?;
        let match_count = r.u8()?;
        r.skip(2)?;
        Some(Self { header, result, spawn_count, match_count, _pad: [0; 2] })
    }

    /// Parse child PIDs from buffer (after from_bytes)
    pub fn parse_pids(buf: &[u8], count: usize) -> Option<[u32; Self::MAX_CHILDREN]> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        r.skip(Self::FIXED_SIZE)?; // skip header
        let mut pids = [0u32; Self::MAX_CHILDREN];
        for i in 0..count.min(Self::MAX_CHILDREN) {
            pids[i] = r.u32()?;
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
// Response Messages
// =============================================================================

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
    pub name: [u8; 18],
    /// Monotonic port ID (assigned at registration)
    pub port_id: u8,
    /// Port type (see `port_type` module)
    pub port_type: u8,
    /// Port flags (see `port_flags` module)
    pub flags: u8,
    /// Owner service index (0xFF = devd itself)
    pub owner_idx: u8,
    /// Parent port ID (0xFF = no parent / root)
    pub parent_idx: u8,
    /// Reserved padding
    pub _pad: u8,
    /// DataPort shared memory ID (0 if no DataPort)
    pub shmem_id: u32,
    /// Owner process PID
    pub owner_pid: u32,
}

impl PortEntry {
    pub const SIZE: usize = 32;

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..18].copy_from_slice(&self.name);
        buf[18] = self.port_id;
        buf[19] = self.port_type;
        buf[20] = self.flags;
        buf[21] = self.owner_idx;
        buf[22] = self.parent_idx;
        buf[23] = self._pad;
        buf[24..28].copy_from_slice(&self.shmem_id.to_le_bytes());
        buf[28..32].copy_from_slice(&self.owner_pid.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        let mut name = [0u8; 18];
        name.copy_from_slice(&buf[0..18]);
        Some(Self {
            name,
            port_id: buf[18],
            port_type: buf[19],
            flags: buf[20],
            owner_idx: buf[21],
            parent_idx: buf[22],
            _pad: buf[23],
            shmem_id: u32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]),
            owner_pid: u32::from_le_bytes([buf[28], buf[29], buf[30], buf[31]]),
        })
    }

    /// Get port name as slice (up to null terminator)
    pub fn name_str(&self) -> &[u8] {
        let len = self.name.iter().position(|&b| b == 0).unwrap_or(18);
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
    /// e.g., "/uart:0" for bus drivers
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
///
/// When more than `MAX_PER_MSG` entries exist, devd sends multiple messages
/// with the same `seq_id`.  Each carries `count` entries; `total` is the grand
/// total across all chunks.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PortsListResponse {
    pub header: QueryHeader,
    /// Number of port entries in this message
    pub count: u16,
    /// Total number of port entries across all chunks
    pub total: u16,
    // Followed by: count * PortEntry
}

impl PortsListResponse {
    pub const HEADER_SIZE: usize = QueryHeader::SIZE + 4;
    /// Max entries that fit in one IPC message: (576 - 12) / 32 = 17
    pub const MAX_PER_MSG: usize = 17;

    pub fn new(seq_id: u32, count: u16, total: u16) -> Self {
        Self {
            header: QueryHeader::new(msg::PORTS_LIST, seq_id),
            count,
            total,
        }
    }

    pub fn header_to_bytes(&self) -> [u8; Self::HEADER_SIZE] {
        let mut buf = [0u8; Self::HEADER_SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..10].copy_from_slice(&self.count.to_le_bytes());
        buf[10..12].copy_from_slice(&self.total.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            count: u16::from_le_bytes([buf[8], buf[9]]),
            total: u16::from_le_bytes([buf[10], buf[11]]),
        })
    }
}

/// Services list response header (devd → client)
///
/// When more than `MAX_PER_MSG` entries exist, devd sends multiple messages
/// with the same `seq_id`.  Each message carries `count` entries in this chunk
/// and `total` = total entries across all chunks.  The client accumulates until
/// it has received `total` entries.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ServicesListResponse {
    pub header: QueryHeader,
    /// Number of service entries in this message
    pub count: u16,
    /// Total number of service entries across all chunks
    pub total: u16,
    // Followed by: count * ServiceEntry
}

impl ServicesListResponse {
    pub const HEADER_SIZE: usize = QueryHeader::SIZE + 4;
    /// Max entries that fit in one IPC message: (576 - 12) / 64 = 8
    pub const MAX_PER_MSG: usize = 8;

    pub fn new(seq_id: u32, count: u16, total: u16) -> Self {
        Self {
            header: QueryHeader::new(msg::SERVICES_LIST, seq_id),
            count,
            total,
        }
    }

    pub fn header_to_bytes(&self) -> [u8; Self::HEADER_SIZE] {
        let mut buf = [0u8; Self::HEADER_SIZE];
        buf[0..8].copy_from_slice(&self.header.to_bytes());
        buf[8..10].copy_from_slice(&self.count.to_le_bytes());
        buf[10..12].copy_from_slice(&self.total.to_le_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        Some(Self {
            header: QueryHeader::from_bytes(buf)?,
            count: u16::from_le_bytes([buf[8], buf[9]]),
            total: u16::from_le_bytes([buf[10], buf[11]]),
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
        use crate::wire::WireWriter;
        if name.len() > 255 { return None; }
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_u8(name.len() as u8)?;
        w.put_zeros(3)?;
        w.put_bytes(name)?;
        Some(w.finish())
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let name_len = r.u8()?;
        r.skip(3)?;
        let name = r.bytes(name_len as usize)?;
        Some((Self { header, name_len, _pad: [0; 3] }, name))
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

    /// Create an end-of-line response: this node has no answer and no children.
    pub fn eol(seq_id: u32) -> Self {
        Self {
            header: QueryHeader::with_flags(msg::SERVICE_INFO_RESULT, query_flags::EOL, seq_id),
            result: error::OK,
            info_len: 0,
            _pad: 0,
        }
    }

    /// Serialize to buffer with info text, returns total length written
    pub fn write_to(&self, buf: &mut [u8], info: &[u8]) -> Option<usize> {
        use crate::wire::WireWriter;
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_i32(self.result)?;
        w.put_u16(info.len() as u16)?;
        w.put_u16(0)?;
        w.put_bytes(info)?;
        Some(w.finish())
    }

    /// Parse from buffer
    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let result = r.i32()?;
        let info_len = r.u16()?;
        r.skip(2)?;
        let info = r.bytes(info_len as usize)?;
        Some((Self { header, result, info_len, _pad: 0 }, info))
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
        use crate::wire::WireWriter;
        let len = message.len().min(Self::MAX_MSG_LEN);
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_u8(self.level)?;
        w.put_u8(len as u8)?;
        w.put_zeros(2)?;
        w.put_bytes(&message[..len])?;
        Some(w.finish())
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let level = r.u8()?;
        let msg_len = r.u8()?;
        r.skip(2)?;
        let message = r.bytes(msg_len as usize)?;
        Some((Self { header, level, msg_len, _pad: [0; 2] }, message))
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
        use crate::wire::WireWriter;
        let mut w = WireWriter::new(buf);
        w.put_query_header(&self.header)?;
        w.put_i32(self.result)?;
        w.put_u8(self.count)?;
        w.put_u8(self.live_enabled)?;
        w.put_u16(text.len() as u16)?;
        w.put_bytes(text)?;
        Some(w.finish())
    }

    pub fn from_bytes(buf: &[u8]) -> Option<(Self, &[u8])> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let header = r.query_header()?;
        let result = r.i32()?;
        let count = r.u8()?;
        let live_enabled = r.u8()?;
        let text_len = r.u16()?;
        let text = r.bytes(text_len as usize)?;
        Some((Self { header, result, count, live_enabled, text_len }, text))
    }
}

// =============================================================================
// Mount Protocol Messages
// =============================================================================

/// Transport type constants for mount protocol
pub mod mount_transport {
    pub const PORT: u8 = 0;
    pub const DATAPORT: u8 = 1;
}

/// Register a filesystem mount point (driver → devd)
///
/// Wire format:
/// ```text
/// [0..8]:    QueryHeader (msg_type=REGISTER_MOUNT)
/// [8]:       transport_type (0=Port, 1=DataPort)
/// [9]:       prefix_len
/// [10]:      port_name_len (if Port) OR 0
/// [11]:      _pad
/// [12..16]:  shmem_id (u32 LE, if DataPort) OR 0
/// [16..80]:  prefix (up to 64 bytes)
/// [80..112]: port_name (up to 32 bytes, if Port)
/// ```
pub struct RegisterMount;

impl RegisterMount {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 8; // 16 bytes header
    pub const MAX_SIZE: usize = 112;

    /// Serialize a DataPort mount registration.
    pub fn write_dataport(buf: &mut [u8], seq_id: u32, prefix: &[u8], shmem_id: u32) -> Option<usize> {
        use crate::wire::WireWriter;
        if prefix.len() > 64 || buf.len() < Self::MAX_SIZE { return None; }
        let header = QueryHeader::new(msg::REGISTER_MOUNT, seq_id);
        let mut w = WireWriter::new(buf);
        w.put_query_header(&header)?;
        w.put_u8(mount_transport::DATAPORT)?;
        w.put_u8(prefix.len() as u8)?;
        w.put_u8(0)?; // no port name
        w.put_u8(0)?;
        w.put_u32(shmem_id)?;
        w.put_padded(prefix, 64)?;
        w.put_zeros(32)?; // port_name field
        Some(w.finish())
    }

    /// Serialize a Port-based mount registration.
    pub fn write_port(buf: &mut [u8], seq_id: u32, prefix: &[u8], port_name: &[u8]) -> Option<usize> {
        use crate::wire::WireWriter;
        if prefix.len() > 64 || port_name.len() > 32 || buf.len() < Self::MAX_SIZE { return None; }
        let header = QueryHeader::new(msg::REGISTER_MOUNT, seq_id);
        let mut w = WireWriter::new(buf);
        w.put_query_header(&header)?;
        w.put_u8(mount_transport::PORT)?;
        w.put_u8(prefix.len() as u8)?;
        w.put_u8(port_name.len() as u8)?;
        w.put_u8(0)?;
        w.put_u32(0)?; // no shmem_id
        w.put_padded(prefix, 64)?;
        w.put_padded(port_name, 32)?;
        Some(w.finish())
    }

    /// Parse from buffer. Returns (transport_type, prefix, port_name, shmem_id).
    pub fn from_bytes(buf: &[u8]) -> Option<(u8, &[u8], &[u8], u32)> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        r.skip(QueryHeader::SIZE)?; // skip header (caller already parsed it)
        let transport_type = r.u8()?;
        let prefix_len = r.u8()? as usize;
        let port_name_len = r.u8()? as usize;
        r.skip(1)?; // pad
        let shmem_id = r.u32()?;
        if prefix_len > 64 || port_name_len > 32 { return None; }
        // prefix lives at fixed offset 16, port_name at fixed offset 80
        // Read the full prefix field (64 bytes), then slice to actual length
        let prefix_field = r.bytes(64)?;
        let prefix = &prefix_field[..prefix_len];
        let port_name_field = r.bytes(32)?;
        let port_name = if port_name_len > 0 {
            &port_name_field[..port_name_len]
        } else {
            &[]
        };
        Some((transport_type, prefix, port_name, shmem_id))
    }
}

/// Resolve a path to a mount transport (client → devd)
///
/// Wire format:
/// ```text
/// [0..8]:   QueryHeader (msg_type=RESOLVE_PATH)
/// [8]:      path_len
/// [9..73]:  path (up to 64 bytes)
/// ```
pub struct ResolvePath;

impl ResolvePath {
    pub const FIXED_SIZE: usize = QueryHeader::SIZE + 1; // 9 bytes
    pub const MAX_SIZE: usize = QueryHeader::SIZE + 1 + 64; // 73 bytes

    pub fn write_to(buf: &mut [u8], seq_id: u32, path: &[u8]) -> Option<usize> {
        use crate::wire::WireWriter;
        let path_len = path.len().min(64);
        let header = QueryHeader::new(msg::RESOLVE_PATH, seq_id);
        let mut w = WireWriter::new(buf);
        w.put_query_header(&header)?;
        w.put_u8(path_len as u8)?;
        w.put_bytes(&path[..path_len])?;
        Some(w.finish())
    }

    pub fn from_bytes(buf: &[u8]) -> Option<&[u8]> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        r.skip(QueryHeader::SIZE)?; // skip header
        let path_len = r.u8()? as usize;
        if path_len > 64 { return None; }
        r.bytes(path_len)
    }
}

/// Response for RESOLVE_PATH
///
/// Wire format:
/// ```text
/// [0..8]:    QueryHeader (msg_type=RESOLVE_RESULT)
/// [8]:       result (0=ok, negative=error)
/// [9]:       transport_type (0=Port, 1=DataPort)
/// [10]:      port_name_len
/// [11]:      remaining_path_len
/// [12..16]:  shmem_id (if DataPort)
/// [16..48]:  port_name (if Port)
/// [48..112]: remaining_path (up to 64 bytes)
/// ```
pub struct ResolvePathResponse;

impl ResolvePathResponse {
    pub const SIZE: usize = 112;

    pub fn write_not_found(buf: &mut [u8], seq_id: u32) -> usize {
        use crate::wire::WireWriter;
        let header = QueryHeader::new(msg::RESOLVE_RESULT, seq_id);
        let mut w = WireWriter::new(buf);
        w.put_query_header(&header).unwrap();
        w.put_u8(error::NOT_FOUND as u8).unwrap();
        w.put_zeros(Self::SIZE - 9).unwrap();
        w.finish()
    }

    pub fn write_dataport(buf: &mut [u8], seq_id: u32, shmem_id: u32, remaining: &[u8]) -> usize {
        use crate::wire::WireWriter;
        let header = QueryHeader::new(msg::RESOLVE_RESULT, seq_id);
        let rem_len = remaining.len().min(64);
        let mut w = WireWriter::new(buf);
        w.put_query_header(&header).unwrap();
        w.put_u8(0).unwrap(); // result OK
        w.put_u8(mount_transport::DATAPORT).unwrap();
        w.put_u8(0).unwrap(); // no port name
        w.put_u8(rem_len as u8).unwrap();
        w.put_u32(shmem_id).unwrap();
        w.put_zeros(32).unwrap(); // port_name field (empty)
        w.put_padded(&remaining[..rem_len], 64).unwrap();
        w.finish()
    }

    pub fn write_port(buf: &mut [u8], seq_id: u32, port_name: &[u8], remaining: &[u8]) -> usize {
        use crate::wire::WireWriter;
        let header = QueryHeader::new(msg::RESOLVE_RESULT, seq_id);
        let pn_len = port_name.len().min(32);
        let rem_len = remaining.len().min(64);
        let mut w = WireWriter::new(buf);
        w.put_query_header(&header).unwrap();
        w.put_u8(0).unwrap(); // result OK
        w.put_u8(mount_transport::PORT).unwrap();
        w.put_u8(pn_len as u8).unwrap();
        w.put_u8(rem_len as u8).unwrap();
        w.put_u32(0).unwrap(); // no shmem_id
        w.put_padded(&port_name[..pn_len], 32).unwrap();
        w.put_padded(&remaining[..rem_len], 64).unwrap();
        w.finish()
    }

    /// Parse response. Returns (result, transport_type, port_name, remaining_path, shmem_id).
    pub fn from_bytes(buf: &[u8]) -> Option<(i8, u8, &[u8], &[u8], u32)> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        r.skip(QueryHeader::SIZE)?; // skip header
        let result = r.u8()? as i8;
        let transport_type = r.u8()?;
        let port_name_len = r.u8()? as usize;
        let remaining_len = r.u8()? as usize;
        let shmem_id = r.u32()?;
        // port_name is in a 32-byte fixed field
        let pn_field = r.bytes(32)?;
        let port_name = if port_name_len > 0 && port_name_len <= 32 {
            &pn_field[..port_name_len]
        } else {
            &[]
        };
        // remaining_path is in a 64-byte fixed field
        let rem_field = r.bytes(64)?;
        let remaining = if remaining_len > 0 && remaining_len <= 64 {
            &rem_field[..remaining_len]
        } else {
            &[]
        };
        Some((result, transport_type, port_name, remaining, shmem_id))
    }
}

/// Mount list entry (used in LIST_MOUNTS response)
///
/// 104 bytes per entry:
/// ```text
/// [0]:      transport_type
/// [1]:      prefix_len
/// [2]:      port_name_len
/// [3]:      _pad
/// [4..8]:   shmem_id
/// [8..72]:  prefix (64 bytes)
/// [72..104]: port_name (32 bytes)
/// ```
pub struct MountListEntry;

impl MountListEntry {
    pub const SIZE: usize = 104;
    /// Max entries per IPC message: (576 - 12) / 104 = 5
    pub const MAX_PER_MSG: usize = 5;

    pub fn write_to(buf: &mut [u8], transport_type: u8, prefix: &[u8], port_name: &[u8], shmem_id: u32) -> bool {
        use crate::wire::WireWriter;
        let mut w = WireWriter::new(buf);
        let pl = prefix.len().min(64);
        let pnl = port_name.len().min(32);
        if w.put_u8(transport_type).is_none() { return false; }
        if w.put_u8(pl as u8).is_none() { return false; }
        if w.put_u8(pnl as u8).is_none() { return false; }
        if w.put_u8(0).is_none() { return false; }
        if w.put_u32(shmem_id).is_none() { return false; }
        if w.put_padded(&prefix[..pl], 64).is_none() { return false; }
        if w.put_padded(&port_name[..pnl], 32).is_none() { return false; }
        true
    }

    /// Parse entry from buffer. Returns (transport_type, prefix, port_name, shmem_id).
    pub fn from_bytes(buf: &[u8]) -> Option<(u8, &[u8], &[u8], u32)> {
        use crate::wire::WireReader;
        let mut r = WireReader::new(buf);
        let transport_type = r.u8()?;
        let prefix_len = r.u8()? as usize;
        let port_name_len = r.u8()? as usize;
        r.skip(1)?; // pad
        let shmem_id = r.u32()?;
        let prefix_field = r.bytes(64)?;
        let port_name_field = r.bytes(32)?;
        let prefix = &prefix_field[..prefix_len.min(64)];
        let port_name = &port_name_field[..port_name_len.min(32)];
        Some((transport_type, prefix, port_name, shmem_id))
    }
}

/// LIST_MOUNTS response header
///
/// ```text
/// [0..8]:  QueryHeader (msg_type=MOUNTS_LIST)
/// [8..10]: count (u16) - entries in this chunk
/// [10..12]: total (u16) - total entries across all chunks
/// ```
pub struct MountsListResponse;

impl MountsListResponse {
    pub const HEADER_SIZE: usize = QueryHeader::SIZE + 4; // 12 bytes

    pub fn write_header(buf: &mut [u8], seq_id: u32, count: u16, total: u16) -> usize {
        let header = QueryHeader::new(msg::MOUNTS_LIST, seq_id);
        buf[0..8].copy_from_slice(&header.to_bytes());
        buf[8..10].copy_from_slice(&count.to_le_bytes());
        buf[10..12].copy_from_slice(&total.to_le_bytes());
        Self::HEADER_SIZE
    }

    /// Parse header. Returns (count, total).
    pub fn parse_header(buf: &[u8]) -> Option<(u16, u16)> {
        if buf.len() < Self::HEADER_SIZE {
            return None;
        }
        let count = u16::from_le_bytes([buf[8], buf[9]]);
        let total = u16::from_le_bytes([buf[10], buf[11]]);
        Some((count, total))
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
    /// Duplicate entry (e.g., mount prefix already exists)
    pub const DUPLICATE: i32 = -8;
    /// Table full (e.g., mount table)
    pub const TABLE_FULL: i32 = -9;
}
