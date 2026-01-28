//! Device daemon client library
//!
//! Provides a clean, state-machine-based API for drivers to communicate
//! with devd. Handles bidirectional communication for:
//! - Driver → devd: state changes, port/device registration
//! - devd → Driver: spawn commands (driver spawns its own children)
//!
//! ## Architecture
//!
//! ```text
//! devd (policy holder)
//!  ↕ bidirectional channel
//! Driver (uses DevdClient)
//!  ↓ spawns children with restricted capabilities
//! Child drivers
//! ```
//!
//! ## Example
//!
//! ```rust
//! use userlib::devd::{DevdClient, PortType, DeviceClass, DriverState};
//!
//! // Connect to devd
//! let mut client = DevdClient::connect()?;
//!
//! // Report ready state
//! client.report_state(DriverState::Ready)?;
//!
//! // Register a block port
//! client.register_port(b"disk0:", PortType::Block, None)?;
//!
//! // In event loop, check for commands from devd
//! if let Some(cmd) = client.poll_command()? {
//!     match cmd {
//!         DevdCommand::SpawnChild { binary, trigger } => {
//!             // Spawn the child with restricted capabilities
//!             let pid = spawn_with_caps(binary, my_caps.restrict(...))?;
//!             client.ack_spawn(cmd.seq_id, 0, pid)?;
//!         }
//!         _ => {}
//!     }
//! }
//! ```

use crate::error::SysError;
use crate::ipc::{Channel, ObjHandle};
use crate::syscall;
use crate::query::{
    QueryHeader, PortRegister, PortRegisterResponse, DeviceRegister,
    ListDevices, DeviceListResponse, StateChange, SpawnChild, SpawnAck,
    PortFilter, filter_mode, GetSpawnContext, SpawnContextResponse,
    QueryPort, PortInfoResponse, port_flags, UpdatePortShmemId,
    ListPorts, PortsListResponse, PortEntry,
    ListServices, ServicesListResponse, ServiceEntry,
    QueryServiceInfo, ServiceInfoResult,
    msg, port_type, error, class, driver_state, service_state,
};

// =============================================================================
// Port Types (re-export for convenience)
// =============================================================================

/// Port type for registration
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PortType {
    /// Console port (keyboard/display)
    Console = port_type::CONSOLE,
    /// Block device port (disk)
    Block = port_type::BLOCK,
    /// Network port
    Network = port_type::NETWORK,
    /// Partition port
    Partition = port_type::PARTITION,
    /// USB port
    Usb = port_type::USB,
    /// Filesystem port
    Filesystem = port_type::FILESYSTEM,
    /// Service port
    Service = port_type::SERVICE,
}

// =============================================================================
// Device Classes (re-export for convenience)
// =============================================================================

/// Device class for registration
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum DeviceClass {
    /// Mass storage device
    MassStorage = class::MASS_STORAGE,
    /// USB hub
    Hub = class::HUB,
    /// Human interface device
    Hid = class::HID,
    /// Other class
    Other(u16),
}

impl DeviceClass {
    pub fn as_u16(self) -> u16 {
        match self {
            DeviceClass::MassStorage => class::MASS_STORAGE,
            DeviceClass::Hub => class::HUB,
            DeviceClass::Hid => class::HID,
            DeviceClass::Other(c) => c,
        }
    }
}

// =============================================================================
// Driver State
// =============================================================================

/// Driver state for reporting to devd
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum DriverState {
    /// Driver is initializing
    Initializing = driver_state::INITIALIZING,
    /// Driver is ready to receive commands
    Ready = driver_state::READY,
    /// Driver encountered an error
    Error = driver_state::ERROR,
    /// Driver is stopping
    Stopping = driver_state::STOPPING,
}

// =============================================================================
// Device Info
// =============================================================================

/// Device information for registration
#[derive(Clone, Copy, Debug)]
pub struct DeviceInfo<'a> {
    /// Port name this device is associated with
    pub port_name: &'a [u8],
    /// Device class
    pub class: DeviceClass,
    /// Device subclass
    pub subclass: u16,
    /// Vendor ID (0 if unknown)
    pub vendor_id: u16,
    /// Product ID (0 if unknown)
    pub product_id: u16,
    /// Human-readable device name
    pub name: &'a [u8],
}

impl Default for DeviceInfo<'_> {
    fn default() -> Self {
        Self {
            port_name: b"",
            class: DeviceClass::Other(0),
            subclass: 0,
            vendor_id: 0,
            product_id: 0,
            name: b"Unknown Device",
        }
    }
}

// =============================================================================
// Commands from devd
// =============================================================================

/// Parsed spawn filter
#[derive(Clone, Copy, Debug)]
pub struct SpawnFilter {
    /// Filter mode
    pub mode: u8,
    /// Port type (for BY_TYPE mode)
    pub port_type: u8,
    /// Class filter
    pub class_filter: u16,
    /// Pattern (for EXACT or CHILDREN_OF modes)
    pub pattern: [u8; 64],
    pub pattern_len: usize,
}

impl SpawnFilter {
    /// Check if this is an exact name match
    pub fn is_exact(&self) -> bool {
        self.mode == filter_mode::EXACT
    }

    /// Check if this matches by port type
    pub fn is_by_type(&self) -> bool {
        self.mode == filter_mode::BY_TYPE
    }

    /// Check if this matches children of a port
    pub fn is_children_of(&self) -> bool {
        self.mode == filter_mode::CHILDREN_OF
    }

    /// Check if this is a broadcast (all)
    pub fn is_all(&self) -> bool {
        self.mode == filter_mode::ALL
    }

    /// Get the pattern as bytes (for EXACT or CHILDREN_OF)
    pub fn pattern_bytes(&self) -> &[u8] {
        &self.pattern[..self.pattern_len]
    }
}

/// Command received from devd
#[derive(Clone, Debug)]
pub enum DevdCommand {
    /// Spawn child driver process(es) matching a filter
    SpawnChild {
        /// Sequence ID for acknowledgement
        seq_id: u32,
        /// Binary name to spawn
        binary: [u8; 64],
        binary_len: usize,
        /// Filter for selecting target ports
        filter: SpawnFilter,
    },
    /// Stop a child driver process
    StopChild {
        /// Sequence ID for acknowledgement
        seq_id: u32,
        /// PID of child to stop
        child_pid: u32,
    },
    /// Query for service info (devd is asking driver for info text)
    QueryInfo {
        /// Sequence ID for response
        seq_id: u32,
    },
}

impl DevdCommand {
    /// Get the binary name as a string slice
    pub fn binary_str(&self) -> Option<&str> {
        match self {
            DevdCommand::SpawnChild { binary, binary_len, .. } => {
                core::str::from_utf8(&binary[..*binary_len]).ok()
            }
            _ => None,
        }
    }

    /// Get the filter (for SpawnChild)
    pub fn filter(&self) -> Option<&SpawnFilter> {
        match self {
            DevdCommand::SpawnChild { filter, .. } => Some(filter),
            _ => None,
        }
    }

    /// Get the exact port name if filter is EXACT mode
    pub fn exact_port(&self) -> Option<&[u8]> {
        match self {
            DevdCommand::SpawnChild { filter, .. } if filter.is_exact() => {
                Some(filter.pattern_bytes())
            }
            _ => None,
        }
    }
}

// =============================================================================
// Client State
// =============================================================================

/// State of the devd client connection
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClientState {
    /// Not connected
    Disconnected,
    /// Connected and ready
    Connected,
    /// Connection failed
    Failed,
}

// =============================================================================
// DevdClient
// =============================================================================

/// Client for bidirectional communication with devd
///
/// Provides API for drivers to:
/// - Report state changes to devd
/// - Register ports and devices
/// - Receive spawn commands from devd
/// - Spawn children with restricted capabilities
pub struct DevdClient {
    channel: Option<Channel>,
    state: ClientState,
    next_seq: u32,
}

impl DevdClient {
    /// Maximum retries for IPC operations
    const MAX_RETRIES: usize = 50;
    /// Delay between retries in milliseconds
    const RETRY_DELAY_MS: u32 = 10;

    /// Create a new disconnected client
    pub const fn new() -> Self {
        Self {
            channel: None,
            state: ClientState::Disconnected,
            next_seq: 1,
        }
    }

    /// Connect to devd-query port
    ///
    /// Returns a connected client ready for communication.
    pub fn connect() -> Result<Self, SysError> {
        let channel = Channel::connect(b"devd-query:")?;
        Ok(Self {
            channel: Some(channel),
            state: ClientState::Connected,
            next_seq: 1,
        })
    }

    /// Get current connection state
    pub fn state(&self) -> ClientState {
        self.state
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.state == ClientState::Connected
    }

    /// Get the channel handle for event loop integration
    ///
    /// Use this to add the devd channel to your event loop so you can
    /// receive commands from devd.
    pub fn handle(&self) -> Option<ObjHandle> {
        self.channel.as_ref().map(|c| c.handle())
    }

    /// Get next sequence ID
    fn next_seq(&mut self) -> u32 {
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        seq
    }

    // =========================================================================
    // State Reporting (driver → devd)
    // =========================================================================

    /// Report driver state change to devd
    ///
    /// Call this when your driver's state changes (initializing → ready, etc.)
    /// devd uses this to track driver lifecycle and trigger policy decisions.
    pub fn report_state(&mut self, new_state: DriverState) -> Result<(), SysError> {
        let seq_id = self.next_seq();
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let msg = StateChange::new(seq_id, new_state as u8);
        channel.send(&msg.to_bytes())?;

        // State changes don't need a response - fire and forget
        Ok(())
    }

    // =========================================================================
    // Spawn Context Query (child driver → devd)
    // =========================================================================

    /// Get spawn context - returns the port that triggered this driver's spawn
    ///
    /// Call this when your driver starts to learn which port to connect to.
    /// devd tracks spawn context by PID and returns the trigger port name.
    ///
    /// Returns (port_name, port_type) if this driver was spawned by devd rules,
    /// or None if no context available (e.g., manually started driver).
    pub fn get_spawn_context(&mut self) -> Result<Option<([u8; 64], usize, PortType)>, SysError> {
        let seq_id = self.next_seq();
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let req = GetSpawnContext::new(seq_id);
        channel.send(&req.to_bytes())?;

        // Wait for response
        let mut resp_buf = [0u8; 128];
        for _ in 0..Self::MAX_RETRIES {
            match channel.recv(&mut resp_buf) {
                Ok(resp_len) if resp_len >= SpawnContextResponse::HEADER_SIZE => {
                    if let Some((resp, port_name)) = SpawnContextResponse::from_bytes(&resp_buf[..resp_len]) {
                        if resp.header.msg_type == msg::SPAWN_CONTEXT {
                            if resp.result == error::OK && resp.port_name_len > 0 {
                                let mut name = [0u8; 64];
                                let len = (resp.port_name_len as usize).min(64);
                                name[..len].copy_from_slice(&port_name[..len]);
                                let ptype = match resp.port_type {
                                    port_type::BLOCK => PortType::Block,
                                    port_type::PARTITION => PortType::Partition,
                                    port_type::NETWORK => PortType::Network,
                                    port_type::CONSOLE => PortType::Console,
                                    port_type::USB => PortType::Usb,
                                    port_type::FILESYSTEM => PortType::Filesystem,
                                    _ => PortType::Service,
                                };
                                return Ok(Some((name, len, ptype)));
                            }
                            // No context available (result != OK or empty name)
                            return Ok(None);
                        }
                    }
                    return Err(SysError::IoError);
                }
                Ok(_) => return Err(SysError::IoError),
                Err(SysError::WouldBlock) => {
                    syscall::sleep_ms(Self::RETRY_DELAY_MS);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        Err(SysError::Timeout)
    }

    // =========================================================================
    // Port/Device Registration (driver → devd)
    // =========================================================================

    /// Register a port with devd
    ///
    /// # Arguments
    /// * `name` - Port name (e.g., b"disk0:")
    /// * `port_type` - Type of port
    /// * `parent` - Optional parent port name
    pub fn register_port(
        &mut self,
        name: &[u8],
        port_type: PortType,
        parent: Option<&[u8]>,
    ) -> Result<(), SysError> {
        self.register_port_with_dataport(name, port_type, 0, parent)
    }

    /// Register a port with devd including DataPort shmem_id
    ///
    /// # Arguments
    /// * `name` - Port name (e.g., b"disk0:")
    /// * `port_type` - Type of port
    /// * `shmem_id` - DataPort shared memory ID (0 = no DataPort)
    /// * `parent` - Optional parent port name
    pub fn register_port_with_dataport(
        &mut self,
        name: &[u8],
        port_type: PortType,
        shmem_id: u32,
        parent: Option<&[u8]>,
    ) -> Result<(), SysError> {
        let seq_id = self.next_seq();
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let reg = PortRegister::new(seq_id, port_type as u8);
        let mut buf = [0u8; 128];
        let len = reg.write_to(&mut buf, name, parent, shmem_id)
            .ok_or(SysError::InvalidArgument)?;

        channel.send(&buf[..len])?;

        // Wait for response
        let mut resp_buf = [0u8; 64];
        for _ in 0..Self::MAX_RETRIES {
            match channel.recv(&mut resp_buf) {
                Ok(resp_len) if resp_len >= PortRegisterResponse::SIZE => {
                    let header = QueryHeader::from_bytes(&resp_buf)
                        .ok_or(SysError::IoError)?;

                    if header.msg_type == msg::REGISTER_PORT {
                        let result = i32::from_le_bytes([
                            resp_buf[8], resp_buf[9], resp_buf[10], resp_buf[11]
                        ]);
                        if result == error::OK {
                            return Ok(());
                        }
                        return Err(SysError::PermissionDenied);
                    } else if header.msg_type == msg::ERROR {
                        return Err(SysError::PermissionDenied);
                    }
                }
                Ok(_) => return Err(SysError::IoError),
                Err(SysError::WouldBlock) => {
                    syscall::sleep_ms(Self::RETRY_DELAY_MS);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        Err(SysError::Timeout)
    }

    /// Update shmem_id for an existing port
    ///
    /// Use this when the DataPort is created after the initial port registration.
    /// This updates the shmem_id so consumers can discover it via query_port.
    pub fn update_port_shmem_id(
        &mut self,
        name: &[u8],
        shmem_id: u32,
    ) -> Result<(), SysError> {
        let seq_id = self.next_seq();
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let update = UpdatePortShmemId::new(seq_id, shmem_id);
        let mut buf = [0u8; 64];
        let len = update.write_to(&mut buf, name)
            .ok_or(SysError::InvalidArgument)?;

        channel.send(&buf[..len])?;

        // Wait for response
        let mut resp_buf = [0u8; 64];
        for _ in 0..Self::MAX_RETRIES {
            match channel.recv(&mut resp_buf) {
                Ok(resp_len) if resp_len >= PortRegisterResponse::SIZE => {
                    let header = QueryHeader::from_bytes(&resp_buf)
                        .ok_or(SysError::IoError)?;

                    if header.msg_type == msg::REGISTER_PORT {
                        let result = i32::from_le_bytes([
                            resp_buf[8], resp_buf[9], resp_buf[10], resp_buf[11]
                        ]);
                        if result == error::OK {
                            return Ok(());
                        }
                        return Err(SysError::NotFound);
                    } else if header.msg_type == msg::ERROR {
                        return Err(SysError::NotFound);
                    }
                }
                Ok(_) => return Err(SysError::IoError),
                Err(SysError::WouldBlock) => {
                    syscall::sleep_ms(Self::RETRY_DELAY_MS);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        Err(SysError::Timeout)
    }

    /// Register a device with devd
    pub fn register_device(&mut self, info: DeviceInfo) -> Result<u32, SysError> {
        let seq_id = self.next_seq();
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let reg = DeviceRegister::new(
            seq_id,
            info.class.as_u16(),
            info.subclass,
            info.vendor_id,
            info.product_id,
        );

        let mut buf = [0u8; 128];
        let len = reg.write_to(&mut buf, info.port_name, info.name)
            .ok_or(SysError::InvalidArgument)?;

        channel.send(&buf[..len])?;

        let mut resp_buf = [0u8; 64];
        for _ in 0..Self::MAX_RETRIES {
            match channel.recv(&mut resp_buf) {
                Ok(resp_len) if resp_len >= 12 => {
                    let header = QueryHeader::from_bytes(&resp_buf)
                        .ok_or(SysError::IoError)?;

                    if header.msg_type == msg::DEVICE_INFO {
                        let device_id = u32::from_le_bytes([
                            resp_buf[8], resp_buf[9], resp_buf[10], resp_buf[11]
                        ]);
                        return Ok(device_id);
                    } else if header.msg_type == msg::ERROR {
                        return Err(SysError::PermissionDenied);
                    }
                }
                Ok(_) => return Err(SysError::IoError),
                Err(SysError::WouldBlock) => {
                    syscall::sleep_ms(Self::RETRY_DELAY_MS);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        Err(SysError::Timeout)
    }

    /// Register both a port and device in one call
    pub fn register_port_and_device<'a>(
        &mut self,
        name: &'a [u8],
        port_type: PortType,
        mut device_info: DeviceInfo<'a>,
    ) -> Result<Option<u32>, SysError> {
        self.register_port(name, port_type, None)?;
        device_info.port_name = name;
        match self.register_device(device_info) {
            Ok(id) => Ok(Some(id)),
            Err(_) => Ok(None),
        }
    }

    // =========================================================================
    // Command Reception (devd → driver)
    // =========================================================================

    /// Poll for commands from devd (non-blocking)
    ///
    /// Call this in your event loop when the devd channel becomes readable.
    /// Returns `Ok(Some(cmd))` if a command was received, `Ok(None)` if no
    /// command is pending, or an error.
    pub fn poll_command(&mut self) -> Result<Option<DevdCommand>, SysError> {
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let mut buf = [0u8; 256];
        match channel.recv(&mut buf) {
            Ok(len) if len >= QueryHeader::SIZE => {
                let header = QueryHeader::from_bytes(&buf)
                    .ok_or(SysError::IoError)?;

                match header.msg_type {
                    msg::SPAWN_CHILD => {
                        let (_, binary, filter_bytes) = SpawnChild::from_bytes(&buf[..len])
                            .ok_or(SysError::IoError)?;

                        // Parse binary name
                        let mut cmd_binary = [0u8; 64];
                        let binary_len = binary.len().min(64);
                        cmd_binary[..binary_len].copy_from_slice(&binary[..binary_len]);

                        // Parse filter
                        let filter = if !filter_bytes.is_empty() {
                            let (pf, pattern) = PortFilter::from_bytes(filter_bytes)
                                .ok_or(SysError::IoError)?;
                            let mut pattern_buf = [0u8; 64];
                            let pattern_len = pattern.len().min(64);
                            pattern_buf[..pattern_len].copy_from_slice(&pattern[..pattern_len]);
                            SpawnFilter {
                                mode: pf.mode,
                                port_type: pf.port_type,
                                class_filter: pf.class_filter,
                                pattern: pattern_buf,
                                pattern_len,
                            }
                        } else {
                            // No filter = broadcast all
                            SpawnFilter {
                                mode: filter_mode::ALL,
                                port_type: 0,
                                class_filter: 0,
                                pattern: [0u8; 64],
                                pattern_len: 0,
                            }
                        };

                        Ok(Some(DevdCommand::SpawnChild {
                            seq_id: header.seq_id,
                            binary: cmd_binary,
                            binary_len,
                            filter,
                        }))
                    }
                    msg::STOP_CHILD => {
                        let child_pid = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
                        Ok(Some(DevdCommand::StopChild {
                            seq_id: header.seq_id,
                            child_pid,
                        }))
                    }
                    msg::QUERY_SERVICE_INFO => {
                        // devd is forwarding a service info query
                        Ok(Some(DevdCommand::QueryInfo {
                            seq_id: header.seq_id,
                        }))
                    }
                    _ => {
                        // Unknown message type - ignore
                        Ok(None)
                    }
                }
            }
            Ok(_) => Ok(None),
            Err(SysError::WouldBlock) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Acknowledge a spawn command (single child)
    ///
    /// Call this after processing a SPAWN_CHILD command to report the result.
    pub fn ack_spawn(&mut self, seq_id: u32, result: i32, child_pid: u32) -> Result<(), SysError> {
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;
        let ack = SpawnAck::new(seq_id, result, child_pid);
        let mut buf = [0u8; 64];
        let pids = if child_pid != 0 { &[child_pid][..] } else { &[][..] };
        let len = ack.write_to(&mut buf, pids).ok_or(SysError::IoError)?;
        channel.send(&buf[..len])?;
        Ok(())
    }

    /// Acknowledge a spawn command (multiple children)
    ///
    /// Call this when a filter matched multiple ports and spawned multiple children.
    pub fn ack_spawn_multi(
        &mut self,
        seq_id: u32,
        result: i32,
        match_count: u8,
        child_pids: &[u32],
    ) -> Result<(), SysError> {
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;
        let ack = SpawnAck::new_multi(seq_id, result, match_count, child_pids.len() as u8);
        let mut buf = [0u8; 128];
        let len = ack.write_to(&mut buf, child_pids).ok_or(SysError::IoError)?;
        channel.send(&buf[..len])?;
        Ok(())
    }

    // =========================================================================
    // Queries
    // =========================================================================

    /// List devices of a specific class
    pub fn list_devices(&mut self, class_filter: Option<DeviceClass>) -> Result<usize, SysError> {
        let seq_id = self.next_seq();
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let class_val = class_filter.map(|c| c.as_u16());
        let req = ListDevices::new(seq_id, class_val);
        channel.send(&req.to_bytes())?;

        let mut resp_buf = [0u8; 512];
        for _ in 0..Self::MAX_RETRIES {
            match channel.recv(&mut resp_buf) {
                Ok(resp_len) if resp_len >= DeviceListResponse::HEADER_SIZE => {
                    let header = QueryHeader::from_bytes(&resp_buf)
                        .ok_or(SysError::IoError)?;

                    if header.msg_type == msg::ERROR {
                        return Ok(0);
                    }

                    let resp = DeviceListResponse::from_bytes(&resp_buf)
                        .ok_or(SysError::IoError)?;
                    return Ok(resp.count as usize);
                }
                Ok(_) => return Ok(0),
                Err(SysError::WouldBlock) => {
                    syscall::sleep_ms(Self::RETRY_DELAY_MS);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        Err(SysError::Timeout)
    }

    /// Query port info by name
    ///
    /// Returns port info including DataPort shmem_id if applicable.
    /// Use this to discover the shmem_id for a named port instead of
    /// hardcoding shmem_id constants.
    pub fn query_port(&mut self, port_name: &[u8]) -> Result<PortInfoResponse, SysError> {
        let seq_id = self.next_seq();
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let req = QueryPort::new(seq_id);
        let mut buf = [0u8; 128];
        let len = req.write_to(&mut buf, port_name)
            .ok_or(SysError::InvalidArgument)?;
        channel.send(&buf[..len])?;

        let mut resp_buf = [0u8; 64];
        for _ in 0..Self::MAX_RETRIES {
            match channel.recv(&mut resp_buf) {
                Ok(resp_len) if resp_len >= PortInfoResponse::SIZE => {
                    let resp = PortInfoResponse::from_bytes(&resp_buf)
                        .ok_or(SysError::IoError)?;

                    if resp.result == error::OK {
                        return Ok(resp);
                    } else {
                        return Err(SysError::NotFound);
                    }
                }
                Ok(_) => return Err(SysError::IoError),
                Err(SysError::WouldBlock) => {
                    syscall::sleep_ms(Self::RETRY_DELAY_MS);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        Err(SysError::Timeout)
    }

    /// Query port and return its shmem_id
    ///
    /// Convenience method that queries port info and extracts the shmem_id.
    /// Returns None if the port doesn't have a DataPort.
    pub fn query_port_shmem_id(&mut self, port_name: &[u8]) -> Result<Option<u32>, SysError> {
        let info = self.query_port(port_name)?;
        if info.has_dataport() {
            Ok(Some(info.shmem_id))
        } else {
            Ok(None)
        }
    }

    // =========================================================================
    // Introspection (client → devd)
    // =========================================================================

    /// List all registered ports
    ///
    /// Returns a list of PortEntry structures describing all ports
    /// currently registered with devd.
    pub fn list_ports(&mut self) -> Result<([PortEntry; 32], usize), SysError> {
        use crate::query::{ListPorts, PortsListResponse, PortEntry};

        let seq_id = self.next_seq();
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let req = ListPorts::new(seq_id);
        channel.send(&req.to_bytes())?;

        // Buffer for response: header + up to 32 entries
        let mut resp_buf = [0u8; 12 + 32 * PortEntry::SIZE];
        for _ in 0..Self::MAX_RETRIES {
            match channel.recv(&mut resp_buf) {
                Ok(len) if len >= PortsListResponse::HEADER_SIZE => {
                    let header = QueryHeader::from_bytes(&resp_buf)
                        .ok_or(SysError::IoError)?;

                    if header.msg_type == msg::PORTS_LIST {
                        let resp = PortsListResponse::from_bytes(&resp_buf)
                            .ok_or(SysError::IoError)?;

                        let count = (resp.count as usize).min(32);
                        let mut entries = [PortEntry {
                            name: [0u8; 20],
                            port_type: 0,
                            flags: 0,
                            owner_idx: 0,
                            parent_idx: 0xFF,
                            shmem_id: 0,
                            owner_pid: 0,
                        }; 32];

                        let data_start = PortsListResponse::HEADER_SIZE;
                        for i in 0..count {
                            let offset = data_start + i * PortEntry::SIZE;
                            if offset + PortEntry::SIZE <= len {
                                if let Some(entry) = PortEntry::from_bytes(&resp_buf[offset..]) {
                                    entries[i] = entry;
                                }
                            }
                        }

                        return Ok((entries, count));
                    } else if header.msg_type == msg::ERROR {
                        return Err(SysError::NotFound);
                    }
                }
                Ok(_) => return Err(SysError::IoError),
                Err(SysError::WouldBlock) => {
                    syscall::sleep_ms(Self::RETRY_DELAY_MS);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        Err(SysError::Timeout)
    }

    /// List all services
    ///
    /// Returns a list of ServiceEntry structures describing all services
    /// tracked by devd.
    pub fn list_services(&mut self) -> Result<([ServiceEntry; 16], usize), SysError> {
        use crate::query::{ListServices, ServicesListResponse, ServiceEntry};

        let seq_id = self.next_seq();
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let req = ListServices::new(seq_id);
        channel.send(&req.to_bytes())?;

        // Buffer for response: header + up to 16 entries
        let mut resp_buf = [0u8; 12 + 16 * ServiceEntry::SIZE];
        for _ in 0..Self::MAX_RETRIES {
            match channel.recv(&mut resp_buf) {
                Ok(len) if len >= ServicesListResponse::HEADER_SIZE => {
                    let header = QueryHeader::from_bytes(&resp_buf)
                        .ok_or(SysError::IoError)?;

                    if header.msg_type == msg::SERVICES_LIST {
                        let resp = ServicesListResponse::from_bytes(&resp_buf)
                            .ok_or(SysError::IoError)?;

                        let count = (resp.count as usize).min(16);
                        let mut entries = [ServiceEntry {
                            name: [0u8; 16],
                            pid: 0,
                            state: 0,
                            index: 0,
                            parent_idx: 0xFF,
                            child_count: 0,
                            total_restarts: 0,
                            last_change: 0,
                        }; 16];

                        let data_start = ServicesListResponse::HEADER_SIZE;
                        for i in 0..count {
                            let offset = data_start + i * ServiceEntry::SIZE;
                            if offset + ServiceEntry::SIZE <= len {
                                if let Some(entry) = ServiceEntry::from_bytes(&resp_buf[offset..]) {
                                    entries[i] = entry;
                                }
                            }
                        }

                        return Ok((entries, count));
                    } else if header.msg_type == msg::ERROR {
                        return Err(SysError::NotFound);
                    }
                }
                Ok(_) => return Err(SysError::IoError),
                Err(SysError::WouldBlock) => {
                    syscall::sleep_ms(Self::RETRY_DELAY_MS);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        Err(SysError::Timeout)
    }

    /// Query detailed info from a service by name
    ///
    /// Sends a query to devd which forwards it to the named service.
    /// The service responds with UTF-8 text describing its state.
    /// Returns the info text and its length.
    pub fn query_service_info(&mut self, service_name: &[u8]) -> Result<([u8; 1024], usize), SysError> {
        let seq_id = self.next_seq();
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let req = QueryServiceInfo::new(seq_id);
        let mut buf = [0u8; 128];
        let len = req.write_to(&mut buf, service_name)
            .ok_or(SysError::InvalidArgument)?;
        channel.send(&buf[..len])?;

        // Wait for response (larger buffer for info text)
        let mut resp_buf = [0u8; 1100];
        for _ in 0..Self::MAX_RETRIES {
            match channel.recv(&mut resp_buf) {
                Ok(resp_len) if resp_len >= ServiceInfoResult::FIXED_SIZE => {
                    let header = QueryHeader::from_bytes(&resp_buf)
                        .ok_or(SysError::IoError)?;

                    if header.msg_type == msg::SERVICE_INFO_RESULT {
                        if let Some((resp, info)) = ServiceInfoResult::from_bytes(&resp_buf[..resp_len]) {
                            if resp.result == error::OK {
                                let mut result = [0u8; 1024];
                                let info_len = info.len().min(1024);
                                result[..info_len].copy_from_slice(&info[..info_len]);
                                return Ok((result, info_len));
                            } else {
                                return Err(SysError::NotFound);
                            }
                        }
                        return Err(SysError::IoError);
                    } else if header.msg_type == msg::ERROR {
                        return Err(SysError::NotFound);
                    }
                }
                Ok(_) => return Err(SysError::IoError),
                Err(SysError::WouldBlock) => {
                    syscall::sleep_ms(Self::RETRY_DELAY_MS);
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        Err(SysError::Timeout)
    }

    /// Respond to a service info query
    ///
    /// Drivers call this when they receive a QueryInfo command from devd.
    /// The info text should be UTF-8 formatted text describing the driver's
    /// current state and configuration.
    pub fn respond_info(&mut self, seq_id: u32, info: &[u8]) -> Result<(), SysError> {
        let channel = self.channel.as_mut().ok_or(SysError::ConnectionRefused)?;

        let resp = ServiceInfoResult::success(seq_id, info.len() as u16);
        let mut buf = [0u8; 1100];
        let len = resp.write_to(&mut buf, info)
            .ok_or(SysError::InvalidArgument)?;
        channel.send(&buf[..len])?;

        Ok(())
    }

    /// Disconnect from devd
    pub fn disconnect(&mut self) {
        self.channel = None;
        self.state = ClientState::Disconnected;
    }
}

impl Default for DevdClient {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for DevdClient {
    fn drop(&mut self) {
        self.disconnect();
    }
}

// =============================================================================
// Convenience Functions
// =============================================================================

/// Register a block device port with devd
pub fn register_block_device(name: &[u8]) -> Result<DevdClient, SysError> {
    let mut client = DevdClient::connect()?;
    client.register_port(name, PortType::Block, None)?;
    client.register_device(DeviceInfo {
        port_name: name,
        class: DeviceClass::MassStorage,
        subclass: 0x06,
        name: b"Block Device",
        ..Default::default()
    })?;
    Ok(client)
}

/// Register a partition port with devd
pub fn register_partition(name: &[u8], parent: &[u8]) -> Result<DevdClient, SysError> {
    let mut client = DevdClient::connect()?;
    client.register_port(name, PortType::Partition, Some(parent))?;
    Ok(client)
}

// =============================================================================
// Driver Service Loop Pattern
// =============================================================================

/// Result of spawning a child process
#[derive(Clone, Copy, Debug)]
pub struct SpawnResult {
    /// Child process ID (0 if spawn failed)
    pub pid: u32,
    /// Error code (0 on success, negative on error)
    pub error: i32,
}

impl SpawnResult {
    pub fn success(pid: u32) -> Self {
        Self { pid, error: 0 }
    }

    pub fn failure(error: i32) -> Self {
        Self { pid: 0, error }
    }

    pub fn is_ok(&self) -> bool {
        self.error == 0 && self.pid > 0
    }
}

impl DevdClient {
    /// Handle a spawn command by executing the binary and sending acknowledgement
    ///
    /// This is the common pattern for drivers receiving SPAWN_CHILD commands.
    /// Returns the spawn result after sending the acknowledgement to devd.
    ///
    /// # Arguments
    /// * `seq_id` - Sequence ID from the command
    /// * `binary` - Binary name to spawn
    ///
    /// # Example
    /// ```rust
    /// match client.poll_command()? {
    ///     Some(DevdCommand::SpawnChild { seq_id, binary, binary_len, .. }) => {
    ///         let binary_name = core::str::from_utf8(&binary[..binary_len]).ok()?;
    ///         let result = client.spawn_and_ack(seq_id, binary_name);
    ///         if result.is_ok() {
    ///             // Track the child PID
    ///         }
    ///     }
    ///     _ => {}
    /// }
    /// ```
    pub fn spawn_and_ack(&mut self, seq_id: u32, binary: &str) -> SpawnResult {
        let pid = syscall::exec(binary);
        if pid > 0 {
            let _ = self.ack_spawn(seq_id, 0, pid as u32);
            SpawnResult::success(pid as u32)
        } else {
            let _ = self.ack_spawn(seq_id, pid as i32, 0);
            SpawnResult::failure(pid as i32)
        }
    }

    /// Handle a spawn command with multiple potential spawns
    ///
    /// Spawns multiple instances if count > 1 (e.g., for fan-out).
    pub fn spawn_multi_and_ack(&mut self, seq_id: u32, binary: &str, count: usize) -> (u8, [u32; 16]) {
        let mut pids = [0u32; 16];
        let mut spawned = 0u8;

        for i in 0..count.min(16) {
            let pid = syscall::exec(binary);
            if pid > 0 {
                pids[i] = pid as u32;
                spawned += 1;
            }
        }

        let result = if spawned > 0 { 0 } else { -1 };
        let _ = self.ack_spawn_multi(seq_id, result, count as u8, &pids[..spawned as usize]);
        (spawned, pids)
    }
}

/// Trait for drivers that handle spawn commands
///
/// Implement this trait to customize spawn behavior (e.g., filtering,
/// tracking children, passing arguments).
pub trait SpawnHandler {
    /// Handle a spawn command from devd
    ///
    /// Called when a SPAWN_CHILD command is received. The handler should:
    /// 1. Validate the binary name and filter
    /// 2. Spawn the child process(es)
    /// 3. Return the PIDs of spawned children
    ///
    /// The default implementation spawns a single child unconditionally.
    fn handle_spawn(&mut self, binary: &str, filter: &SpawnFilter) -> (u8, [u32; 16]) {
        let _ = filter; // Default ignores filter
        let mut pids = [0u32; 16];
        let pid = syscall::exec(binary);
        if pid > 0 {
            pids[0] = pid as u32;
            (1, pids)
        } else {
            (0, pids)
        }
    }

    /// Called when a STOP_CHILD command is received
    fn handle_stop(&mut self, pid: u32) {
        let _ = syscall::kill(pid);
    }

    /// Get service info text for introspection queries
    ///
    /// Called when a QueryInfo command is received. Return UTF-8 text
    /// describing the driver's current state and configuration.
    /// The default implementation returns a placeholder message.
    fn get_info(&self) -> &[u8] {
        b"No info available"
    }
}

/// Run the driver service loop
///
/// This is the generic pattern for drivers that receive commands from devd.
/// After registering ports and reporting ready state, call this to enter
/// the service loop.
///
/// # Type Parameters
/// * `H` - A handler implementing `SpawnHandler` for custom spawn behavior
///
/// # Arguments
/// * `client` - Connected DevdClient
/// * `handler` - Spawn handler for custom behavior
/// * `poll_interval_ms` - How often to poll for commands (use 0 for no delay)
///
/// # Example
/// ```rust
/// struct MyDriver { children: Vec<u32> }
/// impl SpawnHandler for MyDriver {
///     fn handle_spawn(&mut self, binary: &str, _filter: &SpawnFilter) -> (u8, [u32; 16]) {
///         let pid = syscall::exec(binary);
///         if pid > 0 {
///             self.children.push(pid as u32);
///             ([pid as u32, 0, 0, ...], 1)
///         } else {
///             ([0; 16], 0)
///         }
///     }
/// }
///
/// let client = DevdClient::connect()?;
/// client.report_state(DriverState::Ready)?;
/// run_driver_loop(client, MyDriver::new());
/// ```
pub fn run_driver_loop<H: SpawnHandler>(
    mut client: DevdClient,
    mut handler: H,
) -> ! {
    // Get the channel handle for event-driven blocking
    let handle = match client.handle() {
        Some(h) => h,
        None => {
            // No channel - can't run event loop
            loop { syscall::sleep_ms(1000); }
        }
    };

    loop {
        // Block until channel is readable (Sleeping state, no deadline)
        let _ = crate::ipc::wait_one(handle);

        // Process all pending commands
        loop {
            match client.poll_command() {
                Ok(Some(cmd)) => {
                    handle_command(&mut client, &mut handler, cmd);
                }
                Ok(None) => break,  // No more commands
                Err(_) => break,    // Error - exit inner loop
            }
        }
    }
}

/// Process a single command from devd
fn handle_command<H: SpawnHandler>(
    client: &mut DevdClient,
    handler: &mut H,
    cmd: DevdCommand,
) {
    match cmd {
        DevdCommand::SpawnChild { seq_id, binary, binary_len, filter } => {
            let binary_name = match core::str::from_utf8(&binary[..binary_len]) {
                Ok(s) => s,
                Err(_) => {
                    let _ = client.ack_spawn(seq_id, -1, 0);
                    return;
                }
            };

            let (spawned, pids) = handler.handle_spawn(binary_name, &filter);
            let result = if spawned > 0 { 0 } else { -1 };
            let _ = client.ack_spawn_multi(seq_id, result, spawned, &pids[..spawned as usize]);
        }
        DevdCommand::StopChild { child_pid, .. } => {
            handler.handle_stop(child_pid);
        }
        DevdCommand::QueryInfo { seq_id } => {
            // Get info from handler and respond
            let info = handler.get_info();
            let _ = client.respond_info(seq_id, info);
        }
    }
}

/// Default spawn handler that spawns children unconditionally
pub struct DefaultSpawnHandler;

impl SpawnHandler for DefaultSpawnHandler {
    // Uses default implementations
}
