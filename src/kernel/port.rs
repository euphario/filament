//! Named Port Registry

#![allow(dead_code)]  // Infrastructure for future use
//!
//! Allows services to register named ports (e.g., "uart:", "fs:", "net:")
//! and clients to connect to them by name.
//!
//! This is the service discovery mechanism for the microkernel.
//! When a client connects to a port, the kernel creates a channel pair
//! and gives one end to the client and one to the service.

use super::ipc::{self, ChannelId, Message, MessageType};
use super::process::Pid;
use crate::logln;

/// Maximum port name length
pub const MAX_PORT_NAME: usize = 32;

/// Maximum number of registered ports
pub const MAX_PORTS: usize = 32;

/// Maximum pending connections per port
pub const MAX_PENDING: usize = 8;

/// Port state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortState {
    /// Slot is free
    Free,
    /// Port is registered and accepting connections
    Open,
    /// Port is closed
    Closed,
}

/// A pending connection request
#[derive(Clone, Copy)]
pub struct PendingConnection {
    /// Client PID
    pub client_pid: Pid,
    /// Client's channel endpoint
    pub client_channel: ChannelId,
}

/// A registered port
pub struct Port {
    /// Port name (null-terminated)
    pub name: [u8; MAX_PORT_NAME],
    /// Owner process ID
    pub owner: Pid,
    /// Channel for receiving connection requests
    pub listen_channel: ChannelId,
    /// State
    pub state: PortState,
    /// Pending connection requests
    pub pending: [Option<PendingConnection>; MAX_PENDING],
}

impl Port {
    pub const fn new() -> Self {
        Self {
            name: [0; MAX_PORT_NAME],
            owner: 0,
            listen_channel: 0,
            state: PortState::Free,
            pending: [None; MAX_PENDING],
        }
    }

    /// Get port name as string
    pub fn name_str(&self) -> &str {
        let len = self.name.iter().position(|&c| c == 0).unwrap_or(MAX_PORT_NAME);
        core::str::from_utf8(&self.name[..len]).unwrap_or("???")
    }

    /// Add a pending connection
    pub fn add_pending(&mut self, conn: PendingConnection) -> bool {
        for slot in self.pending.iter_mut() {
            if slot.is_none() {
                *slot = Some(conn);
                return true;
            }
        }
        false
    }

    /// Remove and return a pending connection
    pub fn take_pending(&mut self) -> Option<PendingConnection> {
        for slot in self.pending.iter_mut() {
            if slot.is_some() {
                return slot.take();
            }
        }
        None
    }

    /// Count pending connections
    pub fn pending_count(&self) -> usize {
        self.pending.iter().filter(|p| p.is_some()).count()
    }
}

/// Port registry
pub struct PortRegistry {
    ports: [Port; MAX_PORTS],
}

impl PortRegistry {
    pub const fn new() -> Self {
        const EMPTY: Port = Port::new();
        Self {
            ports: [EMPTY; MAX_PORTS],
        }
    }

    /// Find a port by name
    fn find_by_name(&self, name: &str) -> Option<usize> {
        self.ports.iter().position(|p| {
            p.state == PortState::Open && p.name_str() == name
        })
    }

    /// Find a free slot
    fn find_free(&self) -> Option<usize> {
        self.ports.iter().position(|p| p.state == PortState::Free)
    }

    /// Register a new port
    /// Returns the listen channel ID on success
    pub fn register(&mut self, name: &str, owner: Pid) -> Result<ChannelId, PortError> {
        // Check if name already exists
        if self.find_by_name(name).is_some() {
            return Err(PortError::AlreadyExists);
        }

        // Find free slot
        let slot = self.find_free().ok_or(PortError::NoSpace)?;

        // Create a channel for this port (owner gets both ends initially)
        let (listen_ch, _connect_ch) = unsafe {
            ipc::channel_table().create_pair(owner, owner)
                .ok_or(PortError::NoChannels)?
        };

        // Set up the port
        let port = &mut self.ports[slot];
        port.name = [0; MAX_PORT_NAME];
        let name_bytes = name.as_bytes();
        let copy_len = core::cmp::min(name_bytes.len(), MAX_PORT_NAME - 1);
        port.name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);
        port.owner = owner;
        port.listen_channel = listen_ch;
        port.state = PortState::Open;
        port.pending = [None; MAX_PENDING];

        Ok(listen_ch)
    }

    /// Unregister a port
    pub fn unregister(&mut self, name: &str, caller: Pid) -> Result<(), PortError> {
        let slot = self.find_by_name(name).ok_or(PortError::NotFound)?;

        if self.ports[slot].owner != caller {
            return Err(PortError::PermissionDenied);
        }

        // Close the listen channel
        unsafe {
            ipc::channel_table().close(self.ports[slot].listen_channel);
        }

        // Mark as free
        self.ports[slot].state = PortState::Free;
        Ok(())
    }

    /// Connect to a port by name
    /// Returns the client's channel endpoint
    pub fn connect(&mut self, name: &str, client_pid: Pid) -> Result<ChannelId, PortError> {
        let slot = self.find_by_name(name).ok_or(PortError::NotFound)?;
        let port = &mut self.ports[slot];

        // Create a channel pair for this connection
        let (client_ch, server_ch) = unsafe {
            ipc::channel_table().create_pair(client_pid, port.owner)
                .ok_or(PortError::NoChannels)?
        };

        // Send connection notification to the service via its listen channel
        // The service will receive the server_ch endpoint
        let mut connect_msg = Message::new();
        connect_msg.header.msg_type = MessageType::Connect;
        connect_msg.header.sender = client_pid;
        connect_msg.header.payload_len = 4;
        connect_msg.payload[0..4].copy_from_slice(&server_ch.to_le_bytes());

        unsafe {
            let table = ipc::channel_table();
            if let Some(slot) = table.endpoints.iter().position(|e| e.id == port.listen_channel) {
                let peer_id = table.endpoints[slot].peer;
                if peer_id != 0 {
                    let _ = table.send(peer_id, connect_msg);
                }
            }
        }

        Ok(client_ch)
    }

    /// Accept a connection (for service side)
    /// Returns the server's channel endpoint for the new connection
    pub fn accept(&mut self, listen_channel: ChannelId, owner: Pid) -> Result<ChannelId, PortError> {
        // Find the port by listen channel
        let slot = self.ports.iter().position(|p| {
            p.state == PortState::Open && p.listen_channel == listen_channel
        }).ok_or(PortError::NotFound)?;

        if self.ports[slot].owner != owner {
            return Err(PortError::PermissionDenied);
        }

        // Try to receive a Connect message from the listen channel
        unsafe {
            match ipc::channel_table().receive(listen_channel) {
                Ok(msg) if msg.header.msg_type == MessageType::Connect => {
                    // Extract server channel from payload
                    if msg.header.payload_len >= 4 {
                        let server_ch = u32::from_le_bytes([
                            msg.payload[0], msg.payload[1],
                            msg.payload[2], msg.payload[3]
                        ]);
                        Ok(server_ch)
                    } else {
                        Err(PortError::InvalidMessage)
                    }
                }
                Ok(_) => Err(PortError::InvalidMessage),
                Err(_) => Err(PortError::WouldBlock),
            }
        }
    }

    /// List all registered ports
    pub fn list(&self) -> impl Iterator<Item = &Port> {
        self.ports.iter().filter(|p| p.state == PortState::Open)
    }

    /// Print registry info
    pub fn print_info(&self) {
        logln!("  Registered ports:");
        let mut count = 0;
        for port in self.ports.iter() {
            if port.state == PortState::Open {
                logln!("    {} (owner: PID {}, pending: {})",
                    port.name_str(), port.owner, port.pending_count());
                count += 1;
            }
        }
        if count == 0 {
            logln!("    (none)");
        }
    }
}

/// Port errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortError {
    /// Port name already registered
    AlreadyExists,
    /// Port not found
    NotFound,
    /// No space in registry
    NoSpace,
    /// No channels available
    NoChannels,
    /// Permission denied
    PermissionDenied,
    /// Would block (no pending connections)
    WouldBlock,
    /// Invalid message format
    InvalidMessage,
}

impl PortError {
    pub fn to_errno(self) -> i32 {
        match self {
            PortError::AlreadyExists => -17,  // EEXIST
            PortError::NotFound => -2,        // ENOENT
            PortError::NoSpace => -28,        // ENOSPC
            PortError::NoChannels => -23,     // ENFILE
            PortError::PermissionDenied => -13, // EACCES
            PortError::WouldBlock => -11,     // EAGAIN
            PortError::InvalidMessage => -22, // EINVAL
        }
    }
}

/// Global port registry
static mut PORT_REGISTRY: PortRegistry = PortRegistry::new();

/// Get the global port registry
/// # Safety
/// Must ensure proper synchronization
pub unsafe fn port_registry() -> &'static mut PortRegistry {
    &mut *core::ptr::addr_of_mut!(PORT_REGISTRY)
}

/// Clean up all ports owned by a process (called on process exit)
pub fn process_cleanup(pid: Pid) {
    unsafe {
        let registry = port_registry();
        for port in registry.ports.iter_mut() {
            if port.state == PortState::Open && port.owner == pid {
                logln!("  Port cleanup: unregistering '{}' (owned by PID {})",
                       port.name_str(), pid);
                // Close the listen channel
                ipc::channel_table().close(port.listen_channel);
                // Mark as free
                port.state = PortState::Free;
            }
        }
    }
}

// ============================================================================
// Syscall wrappers
// ============================================================================

/// Register a port (syscall)
pub fn sys_port_register(name_ptr: u64, name_len: usize, caller: Pid) -> Result<ChannelId, PortError> {
    if name_len == 0 || name_len > MAX_PORT_NAME - 1 {
        return Err(PortError::InvalidMessage);
    }

    let name = unsafe {
        let slice = core::slice::from_raw_parts(name_ptr as *const u8, name_len);
        core::str::from_utf8(slice).map_err(|_| PortError::InvalidMessage)?
    };

    unsafe { port_registry().register(name, caller) }
}

/// Unregister a port (syscall)
pub fn sys_port_unregister(name_ptr: u64, name_len: usize, caller: Pid) -> Result<(), PortError> {
    if name_len == 0 || name_len > MAX_PORT_NAME - 1 {
        return Err(PortError::InvalidMessage);
    }

    let name = unsafe {
        let slice = core::slice::from_raw_parts(name_ptr as *const u8, name_len);
        core::str::from_utf8(slice).map_err(|_| PortError::InvalidMessage)?
    };

    unsafe { port_registry().unregister(name, caller) }
}

/// Connect to a port (syscall)
pub fn sys_port_connect(name_ptr: u64, name_len: usize, caller: Pid) -> Result<ChannelId, PortError> {
    if name_len == 0 || name_len > MAX_PORT_NAME - 1 {
        return Err(PortError::InvalidMessage);
    }

    let name = unsafe {
        let slice = core::slice::from_raw_parts(name_ptr as *const u8, name_len);
        core::str::from_utf8(slice).map_err(|_| PortError::InvalidMessage)?
    };

    unsafe { port_registry().connect(name, caller) }
}

/// Accept a connection (syscall)
pub fn sys_port_accept(listen_channel: ChannelId, caller: Pid) -> Result<ChannelId, PortError> {
    unsafe { port_registry().accept(listen_channel, caller) }
}

// ============================================================================
// Buffer-based syscall wrappers (for use after copy_from_user)
// These take kernel buffers directly instead of user pointers
// ============================================================================

/// Register a port using a kernel buffer
pub fn sys_port_register_buf(name: &[u8], caller: Pid) -> Result<ChannelId, PortError> {
    if name.is_empty() || name.len() > MAX_PORT_NAME - 1 {
        return Err(PortError::InvalidMessage);
    }

    let name_str = core::str::from_utf8(name).map_err(|_| PortError::InvalidMessage)?;
    unsafe { port_registry().register(name_str, caller) }
}

/// Unregister a port using a kernel buffer
pub fn sys_port_unregister_buf(name: &[u8], caller: Pid) -> Result<(), PortError> {
    if name.is_empty() || name.len() > MAX_PORT_NAME - 1 {
        return Err(PortError::InvalidMessage);
    }

    let name_str = core::str::from_utf8(name).map_err(|_| PortError::InvalidMessage)?;
    unsafe { port_registry().unregister(name_str, caller) }
}

/// Connect to a port using a kernel buffer
pub fn sys_port_connect_buf(name: &[u8], caller: Pid) -> Result<ChannelId, PortError> {
    if name.is_empty() || name.len() > MAX_PORT_NAME - 1 {
        return Err(PortError::InvalidMessage);
    }

    let name_str = core::str::from_utf8(name).map_err(|_| PortError::InvalidMessage)?;
    unsafe { port_registry().connect(name_str, caller) }
}

// ============================================================================
// Testing
// ============================================================================

/// Test port registry
pub fn test() {
    logln!("  Testing port registry...");

    unsafe {
        let registry = port_registry();

        // Register a test port
        match registry.register("uart:", 1) {
            Ok(listen_ch) => {
                logln!("    Registered 'uart:' port, listen channel: {}", listen_ch);
            }
            Err(e) => {
                logln!("    [!!] Failed to register port: {:?}", e);
                return;
            }
        }

        // Register another port
        match registry.register("echo:", 1) {
            Ok(listen_ch) => {
                logln!("    Registered 'echo:' port, listen channel: {}", listen_ch);
            }
            Err(e) => {
                logln!("    [!!] Failed to register port: {:?}", e);
            }
        }

        // List ports
        registry.print_info();

        // Try to connect to uart:
        match registry.connect("uart:", 2) {
            Ok(client_ch) => {
                logln!("    PID 2 connected to 'uart:', got channel: {}", client_ch);

                // The service (PID 1) should now have a Connect message
                // Let's simulate accepting it
                let uart_port = registry.ports.iter()
                    .find(|p| p.name_str() == "uart:")
                    .unwrap();
                let listen_ch = uart_port.listen_channel;

                match registry.accept(listen_ch, 1) {
                    Ok(server_ch) => {
                        logln!("    Service accepted connection, got channel: {}", server_ch);

                        // Now client_ch and server_ch form a connected pair
                        // Let's test sending a message
                        let test_msg = b"Hello service!";
                        match ipc::sys_send(client_ch, 2, test_msg) {
                            Ok(()) => logln!("    Client sent message"),
                            Err(e) => logln!("    [!!] Client send failed: {:?}", e),
                        }

                        // Service receives
                        match ipc::sys_receive(server_ch, 1) {
                            Ok(msg) => {
                                let payload = msg.payload_slice();
                                if payload == test_msg {
                                    logln!("    Service received correct message!");
                                }
                            }
                            Err(e) => logln!("    [!!] Service receive failed: {:?}", e),
                        }
                    }
                    Err(e) => logln!("    [!!] Accept failed: {:?}", e),
                }
            }
            Err(e) => {
                logln!("    [!!] Connect failed: {:?}", e);
            }
        }

        // Unregister ports
        let _ = registry.unregister("uart:", 1);
        let _ = registry.unregister("echo:", 1);
        logln!("    Ports unregistered");
    }

    logln!("    [OK] Port registry test passed");
}
