//! Port State Machine and Registry
//!
//! Ports are named listening endpoints that allow service discovery.
//! Clients connect to ports by name, servers accept connections.
//!
//! # Port Lifecycle
//!
//! ```text
//!     ┌─────────────┐
//!     │  Listening  │◄─── register()
//!     └──────┬──────┘
//!            │
//!            │ unregister() or owner exits
//!            ▼
//!     ┌─────────────┐
//!     │   Closed    │
//!     └─────────────┘
//! ```
//!
//! # Connection Flow
//!
//! ```text
//! Client                    Port                     Server
//!   │                         │                         │
//!   │──── connect(name) ─────>│                         │
//!   │                         │──── wake(Accepted) ────>│
//!   │                         │                         │
//!   │                         │<──── accept() ──────────│
//!   │<═══════════════════════>│<═══════════════════════>│
//!   │     client_channel      │     server_channel      │
//! ```
//!
//! # Invariants
//!
//! 1. Port names are unique
//! 2. Only owner can accept connections
//! 3. Pending connections timeout after PENDING_TIMEOUT
//! 4. Ports use sorted array (not hash table) for simplicity

use super::types::{ChannelId, TaskId, PortId, MAX_PORTS, MAX_PORT_NAME, MAX_PENDING_PER_PORT};
use super::table::ChannelTable;
use super::traits::{Subscriber, WakeReason, Waitable, Closable, CloseAction};
use super::waker::{SubscriberSet, WakeList};
use super::error::IpcError;

// Re-export PortInfo from abi crate
pub use abi::{PortInfo, PortClass, port_subclass, port_caps};

/// Timeout for pending connections (in ticks)
pub const PENDING_TIMEOUT: u64 = 30_000; // ~30 seconds at 1kHz

// ============================================================================
// Port State
// ============================================================================

/// Port lifecycle state
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PortState {
    /// Port is listening for connections
    Listening {
        /// Number of pending connections
        pending_count: u8,
    },

    /// Port is closed
    Closed,
}

impl PortState {
    /// Check if listening
    pub fn is_listening(&self) -> bool {
        matches!(self, PortState::Listening { .. })
    }

    /// Check if closed
    pub fn is_closed(&self) -> bool {
        matches!(self, PortState::Closed)
    }

    /// Get pending count
    pub fn pending_count(&self) -> u8 {
        match self {
            PortState::Listening { pending_count } => *pending_count,
            PortState::Closed => 0,
        }
    }
}

// ============================================================================
// Pending Connection
// ============================================================================

/// A pending connection request waiting for accept
#[derive(Clone, Debug)]
pub struct PendingConnection {
    /// Client task ID
    pub client_pid: TaskId,
    /// Client's channel endpoint
    pub client_channel: ChannelId,
    /// Server's channel endpoint (pre-created)
    pub server_channel: ChannelId,
    /// Tick when connection was requested
    pub requested_at: u64,
}

impl PendingConnection {
    /// Create a new pending connection
    pub fn new(client_pid: TaskId, client_channel: ChannelId, server_channel: ChannelId, now: u64) -> Self {
        Self {
            client_pid,
            client_channel,
            server_channel,
            requested_at: now,
        }
    }

    /// Check if this connection has timed out
    pub fn is_expired(&self, now: u64) -> bool {
        now.saturating_sub(self.requested_at) > PENDING_TIMEOUT
    }
}

// ============================================================================
// Port
// ============================================================================

/// A named listening endpoint
pub struct Port {
    /// Port ID
    id: PortId,

    /// Port name (null-terminated or full)
    name: [u8; MAX_PORT_NAME],

    /// Name length
    name_len: u8,

    /// Owner task ID
    owner: TaskId,

    /// Current state
    state: PortState,

    /// Listen channel (server receives connect notifications here)
    listen_channel: ChannelId,

    /// Pending connections waiting for accept
    pending: [Option<PendingConnection>; MAX_PENDING_PER_PORT],

    /// Subscribers for events
    subscribers: SubscriberSet,

    /// Structured port info (optional - None for legacy ports)
    port_info: Option<PortInfo>,
}

impl Port {
    /// Create a new port (legacy - no PortInfo)
    pub fn new(id: PortId, name: &[u8], owner: TaskId, listen_channel: ChannelId) -> Self {
        let mut name_arr = [0u8; MAX_PORT_NAME];
        let len = core::cmp::min(name.len(), MAX_PORT_NAME);
        name_arr[..len].copy_from_slice(&name[..len]);

        Self {
            id,
            name: name_arr,
            name_len: len as u8,
            owner,
            state: PortState::Listening { pending_count: 0 },
            listen_channel,
            pending: [const { None }; MAX_PENDING_PER_PORT],
            subscribers: SubscriberSet::new(),
            port_info: None,
        }
    }

    /// Create a new port with structured PortInfo
    pub fn new_with_info(id: PortId, name: &[u8], owner: TaskId, listen_channel: ChannelId, info: PortInfo) -> Self {
        let mut name_arr = [0u8; MAX_PORT_NAME];
        let len = core::cmp::min(name.len(), MAX_PORT_NAME);
        name_arr[..len].copy_from_slice(&name[..len]);

        Self {
            id,
            name: name_arr,
            name_len: len as u8,
            owner,
            state: PortState::Listening { pending_count: 0 },
            listen_channel,
            pending: [const { None }; MAX_PENDING_PER_PORT],
            subscribers: SubscriberSet::new(),
            port_info: Some(info),
        }
    }

    /// Create a closed placeholder port
    pub const fn closed() -> Self {
        Self {
            id: 0,
            name: [0; MAX_PORT_NAME],
            name_len: 0,
            owner: 0,
            state: PortState::Closed,
            listen_channel: 0,
            pending: [const { None }; MAX_PENDING_PER_PORT],
            subscribers: SubscriberSet::new(),
            port_info: None,
        }
    }

    // ========================================================================
    // Getters
    // ========================================================================

    /// Get port ID
    pub fn id(&self) -> PortId {
        self.id
    }

    /// Get port name as slice
    pub fn name(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }

    /// Get owner task ID
    pub fn owner(&self) -> TaskId {
        self.owner
    }

    /// Get current state
    pub fn state(&self) -> &PortState {
        &self.state
    }

    /// Get listen channel ID
    pub fn listen_channel(&self) -> ChannelId {
        self.listen_channel
    }

    /// Check if names match
    pub fn name_matches(&self, name: &[u8]) -> bool {
        self.name_len as usize == name.len() && self.name() == name
    }

    /// Get structured port info (if available)
    pub fn port_info(&self) -> Option<&PortInfo> {
        self.port_info.as_ref()
    }

    /// Set structured port info
    pub fn set_port_info(&mut self, info: PortInfo) {
        self.port_info = Some(info);
    }

    // ========================================================================
    // Operations
    // ========================================================================

    /// Add a pending connection
    ///
    /// Returns subscribers to wake if connection was added.
    pub fn add_pending(&mut self, conn: PendingConnection) -> Result<WakeList, IpcError> {
        match &mut self.state {
            PortState::Listening { pending_count } => {
                // Find empty slot
                for slot in self.pending.iter_mut() {
                    if slot.is_none() {
                        *slot = Some(conn);
                        *pending_count += 1;
                        // Wake subscribers waiting for Accepted
                        return Ok(self.subscribers.get_for(WakeReason::Accepted));
                    }
                }
                Err(IpcError::PendingFull)
            }
            PortState::Closed => Err(IpcError::Closed),
        }
    }

    /// Accept a pending connection
    ///
    /// Returns (server_channel, client_pid) if there was a pending connection.
    pub fn accept(&mut self) -> Result<(ChannelId, TaskId), IpcError> {
        match &mut self.state {
            PortState::Listening { pending_count } => {
                // Find first pending
                for slot in self.pending.iter_mut() {
                    if let Some(conn) = slot.take() {
                        *pending_count -= 1;
                        return Ok((conn.server_channel, conn.client_pid));
                    }
                }
                Err(IpcError::NoPending)
            }
            PortState::Closed => Err(IpcError::Closed),
        }
    }

    /// Close this port
    ///
    /// Returns subscribers to wake.
    pub fn do_close(&mut self) -> WakeList {
        let subs = self.subscribers.drain();
        self.state = PortState::Closed;

        // Clear pending connections (they will timeout eventually)
        for slot in self.pending.iter_mut() {
            *slot = None;
        }

        subs
    }

    /// Expire old pending connections
    ///
    /// Returns array of channel IDs that need to be closed (max 2 * MAX_PENDING_PER_PORT).
    /// Returns (array, count) where count is the number of valid entries.
    pub fn expire_pending(&mut self, now: u64) -> ([ChannelId; MAX_PENDING_PER_PORT * 2], usize) {
        let mut expired = [0u32; MAX_PENDING_PER_PORT * 2];
        let mut count = 0;

        if let PortState::Listening { pending_count } = &mut self.state {
            for slot in self.pending.iter_mut() {
                if let Some(conn) = slot {
                    if conn.is_expired(now) {
                        if count < expired.len() {
                            expired[count] = conn.client_channel;
                            count += 1;
                        }
                        if count < expired.len() {
                            expired[count] = conn.server_channel;
                            count += 1;
                        }
                        *slot = None;
                        *pending_count -= 1;
                    }
                }
            }
        }

        (expired, count)
    }

    /// Remove a specific task from subscriber list
    ///
    /// Called when a task dies to prevent stale wakes.
    pub fn unsubscribe_by_task(&mut self, task_id: TaskId) {
        self.subscribers.remove_by_task(task_id);
    }
}

// ============================================================================
// Trait Implementations
// ============================================================================

impl Waitable for Port {
    fn poll(&self, filter: WakeReason) -> bool {
        match filter {
            WakeReason::Accepted => {
                matches!(&self.state, PortState::Listening { pending_count } if *pending_count > 0)
            }
            WakeReason::Closed => self.state.is_closed(),
            _ => false,
        }
    }

    fn subscribe(&mut self, sub: Subscriber, filter: WakeReason) -> Result<(), ()> {
        self.subscribers.add(sub, filter)
    }

    fn unsubscribe(&mut self, sub: Subscriber) {
        self.subscribers.remove(sub);
    }
}

impl Closable for Port {
    fn close(&mut self, _owner: TaskId) -> CloseAction {
        self.do_close();
        CloseAction::UnregisterPort { port_id: self.id }
    }
}

// ============================================================================
// Port Registry
// ============================================================================

/// Registry of all ports
///
/// Uses a simple sorted array for O(log n) lookup by name.
/// This is simpler and more testable than a hash table.
pub struct PortRegistry {
    /// Port slots
    ports: [Option<Port>; MAX_PORTS],

    /// Number of active ports
    active_count: usize,

    /// Next port ID
    next_id: PortId,
}

impl PortRegistry {
    /// Create empty registry
    pub const fn new() -> Self {
        Self {
            ports: [const { None }; MAX_PORTS],
            active_count: 0,
            next_id: 1,
        }
    }

    /// Get number of active ports
    pub fn active_count(&self) -> usize {
        self.active_count
    }

    /// Find port by name
    fn find_by_name(&self, name: &[u8]) -> Option<usize> {
        self.ports.iter().position(|p| {
            p.as_ref().map(|port| port.name_matches(name)).unwrap_or(false)
        })
    }

    /// Find port by ID
    fn find_by_id(&self, id: PortId) -> Option<usize> {
        self.ports.iter().position(|p| {
            p.as_ref().map(|port| port.id() == id).unwrap_or(false)
        })
    }

    /// Find free slot
    fn find_free_slot(&self) -> Option<usize> {
        self.ports.iter().position(|p| p.is_none())
    }

    // ========================================================================
    // Operations
    // ========================================================================

    /// Register a new port (legacy - no PortInfo)
    ///
    /// Creates a listen channel for the port.
    /// Returns (port_id, listen_channel_id).
    pub fn register(&mut self, name: &[u8], owner: TaskId, channel_table: &mut ChannelTable) -> Result<(PortId, ChannelId), IpcError> {
        self.register_inner(name, owner, channel_table, None)
    }

    /// Register a new port with structured PortInfo
    ///
    /// Creates a listen channel for the port and attaches device classification info.
    /// Returns (port_id, listen_channel_id).
    pub fn register_with_info(&mut self, name: &[u8], owner: TaskId, channel_table: &mut ChannelTable, info: PortInfo) -> Result<(PortId, ChannelId), IpcError> {
        self.register_inner(name, owner, channel_table, Some(info))
    }

    /// Internal registration implementation
    fn register_inner(&mut self, name: &[u8], owner: TaskId, channel_table: &mut ChannelTable, info: Option<PortInfo>) -> Result<(PortId, ChannelId), IpcError> {
        // Check name length
        if name.is_empty() || name.len() > MAX_PORT_NAME {
            return Err(IpcError::port_exists(name)); // Use exists for bad name too
        }

        // Check for duplicate
        if self.find_by_name(name).is_some() {
            return Err(IpcError::port_exists(name));
        }

        // Find free slot
        let slot = self.find_free_slot().ok_or(IpcError::NoPortSpace)?;

        // Create listen channel pair (owner talks to self for internal signaling)
        let (listen_ch, _peer_ch) = channel_table.create_pair(owner, owner)?;

        // Allocate port ID
        let port_id = self.next_id;
        self.next_id += 1;

        // Create port (with or without PortInfo)
        self.ports[slot] = Some(match info {
            Some(pi) => Port::new_with_info(port_id, name, owner, listen_ch, pi),
            None => Port::new(port_id, name, owner, listen_ch),
        });
        self.active_count += 1;

        Ok((port_id, listen_ch))
    }

    /// Connect to a port by name
    ///
    /// Creates a channel pair and adds a pending connection.
    /// Returns (client_channel, wake_list, port_owner).
    pub fn connect(&mut self, name: &[u8], client: TaskId, channel_table: &mut ChannelTable) -> Result<(ChannelId, WakeList, TaskId), IpcError> {
        // Find port
        let slot = self.find_by_name(name).ok_or(IpcError::PortNotFound)?;

        let port = self.ports[slot].as_mut().ok_or(IpcError::PortNotFound)?;

        if !port.state().is_listening() {
            return Err(IpcError::Closed);
        }

        let server_owner = port.owner();

        // Create channel pair: client <-> server
        let (client_ch, server_ch) = channel_table.create_pair(client, server_owner)?;

        // Add pending connection
        let now = crate::platform::current::timer::ticks();
        let conn = PendingConnection::new(client, client_ch, server_ch, now);
        let wake_list = port.add_pending(conn)?;

        Ok((client_ch, wake_list, server_owner))
    }

    /// Accept a pending connection on a port
    ///
    /// Returns (server_channel, client_pid).
    pub fn accept(&mut self, port_id: PortId, caller: TaskId) -> Result<(ChannelId, TaskId), IpcError> {
        let slot = self.find_by_id(port_id).ok_or(IpcError::InvalidPort { id: port_id })?;

        let port = self.ports[slot].as_mut().ok_or(IpcError::InvalidPort { id: port_id })?;

        if port.owner() != caller {
            return Err(IpcError::port_not_owner(port_id, port.owner(), caller));
        }

        port.accept()
    }

    /// Connect to a port and immediately accept (for kernel-owned ports)
    ///
    /// This is used for kernel bus ports where there's no userspace process
    /// to accept connections. Returns (client_channel, server_channel, port_owner).
    pub fn connect_and_accept(&mut self, name: &[u8], client: TaskId, channel_table: &mut ChannelTable) -> Result<(ChannelId, ChannelId, TaskId), IpcError> {
        // Find port
        let slot = self.find_by_name(name).ok_or(IpcError::PortNotFound)?;

        let port = self.ports[slot].as_mut().ok_or(IpcError::PortNotFound)?;

        if !port.state().is_listening() {
            return Err(IpcError::Closed);
        }

        let server_owner = port.owner();

        // Create channel pair: client <-> server
        let (client_ch, server_ch) = channel_table.create_pair(client, server_owner)?;

        // Add pending connection
        let now = crate::platform::current::timer::ticks();
        let conn = PendingConnection::new(client, client_ch, server_ch, now);
        let _ = port.add_pending(conn)?;

        // Immediately accept for kernel-owned ports
        let (accepted_server_ch, _) = port.accept()?;

        Ok((client_ch, accepted_server_ch, server_owner))
    }

    /// Unregister a port
    ///
    /// Returns subscribers to wake.
    pub fn unregister(&mut self, port_id: PortId, caller: TaskId) -> Result<WakeList, IpcError> {
        let slot = self.find_by_id(port_id).ok_or(IpcError::InvalidPort { id: port_id })?;

        let port = self.ports[slot].as_mut().ok_or(IpcError::InvalidPort { id: port_id })?;

        if port.owner() != caller {
            return Err(IpcError::port_not_owner(port_id, port.owner(), caller));
        }

        let wake_list = port.do_close();
        self.ports[slot] = None;
        self.active_count -= 1;

        Ok(wake_list)
    }

    /// Clean up all ports owned by a task
    pub fn cleanup_task(&mut self, task_id: TaskId) -> WakeList {
        let mut wake_list = WakeList::new();

        for slot in 0..MAX_PORTS {
            if let Some(port) = &mut self.ports[slot] {
                if port.owner() == task_id {
                    let wl = port.do_close();
                    wake_list.merge(&wl);
                    self.ports[slot] = None;
                    self.active_count -= 1;
                }
            }
        }

        wake_list
    }

    /// Get port by ID (for handle system)
    pub fn get(&self, port_id: PortId) -> Option<&Port> {
        let slot = self.find_by_id(port_id)?;
        self.ports[slot].as_ref()
    }

    /// Get port mutably by ID
    pub fn get_mut(&mut self, port_id: PortId) -> Option<&mut Port> {
        let slot = self.find_by_id(port_id)?;
        self.ports[slot].as_mut()
    }

    /// Subscribe to port events
    ///
    /// Returns error if subscriber set is full (never silently drops).
    pub fn subscribe(&mut self, port_id: PortId, caller: TaskId, sub: Subscriber, filter: WakeReason) -> Result<(), IpcError> {
        let slot = self.find_by_id(port_id).ok_or(IpcError::InvalidPort { id: port_id })?;

        let port = self.ports[slot].as_mut().ok_or(IpcError::InvalidPort { id: port_id })?;

        if port.owner() != caller {
            return Err(IpcError::port_not_owner(port_id, port.owner(), caller));
        }

        // SECURITY FIX: Propagate error if subscriber set is full
        port.subscribe(sub, filter).map_err(|()| IpcError::SubscribersFull)
    }

    /// Unregister port by channel ID (for cleanup)
    pub fn unregister_by_channel(&mut self, channel_id: ChannelId) {
        for slot in 0..MAX_PORTS {
            if let Some(port) = &self.ports[slot] {
                if port.listen_channel() == channel_id {
                    if let Some(p) = self.ports[slot].as_mut() {
                        p.do_close();
                    }
                    self.ports[slot] = None;
                    self.active_count -= 1;
                    return;
                }
            }
        }
    }

    /// Check if a port has pending connections by listen channel
    pub fn has_pending_accept(&self, channel_id: ChannelId) -> bool {
        for slot in 0..MAX_PORTS {
            if let Some(port) = &self.ports[slot] {
                if port.listen_channel() == channel_id {
                    return port.state().pending_count() > 0;
                }
            }
        }
        false
    }

    /// Get port owner and listen channel by name
    pub fn get_owner_by_name(&self, name: &[u8]) -> Option<(TaskId, ChannelId)> {
        for slot in 0..MAX_PORTS {
            if let Some(port) = &self.ports[slot] {
                if port.name_matches(name) {
                    return Some((port.owner(), port.listen_channel()));
                }
            }
        }
        None
    }

    /// Find port by listen channel (for accept_by_channel compatibility)
    fn find_by_listen_channel(&self, channel_id: ChannelId) -> Option<usize> {
        self.ports.iter().position(|p| {
            p.as_ref().map(|port| port.listen_channel() == channel_id).unwrap_or(false)
        })
    }

    /// Get mutable port by listen channel (for waker registration)
    pub fn get_by_listen_channel_mut(&mut self, channel_id: ChannelId) -> Option<&mut Port> {
        let slot = self.find_by_listen_channel(channel_id)?;
        self.ports[slot].as_mut()
    }

    /// Accept a connection using listen_channel (legacy API compatibility)
    pub fn accept_by_channel(&mut self, listen_channel: ChannelId, caller: TaskId) -> Result<(ChannelId, TaskId), IpcError> {
        let slot = self.find_by_listen_channel(listen_channel)
            .ok_or(IpcError::PortNotFound)?;

        let port = self.ports[slot].as_mut()
            .ok_or(IpcError::PortNotFound)?;

        if port.owner() != caller {
            return Err(IpcError::port_not_owner(port.id(), port.owner(), caller));
        }

        port.accept()
    }

    /// Unregister a port by name (legacy API compatibility)
    pub fn unregister_by_name(&mut self, name: &[u8], caller: TaskId) -> Result<WakeList, IpcError> {
        let slot = self.find_by_name(name)
            .ok_or(IpcError::PortNotFound)?;

        let port = self.ports[slot].as_mut()
            .ok_or(IpcError::PortNotFound)?;

        if port.owner() != caller {
            return Err(IpcError::port_not_owner(port.id(), port.owner(), caller));
        }

        let wake_list = port.do_close();
        self.ports[slot] = None;
        self.active_count -= 1;

        Ok(wake_list)
    }

    /// Remove a task from ALL port subscriber lists
    ///
    /// Called when a task dies to prevent stale wakes.
    pub fn remove_subscriber_from_all(&mut self, task_id: TaskId) {
        for slot in 0..MAX_PORTS {
            if let Some(ref mut port) = self.ports[slot] {
                port.unsubscribe_by_task(task_id);
            }
        }
    }
}

impl Default for PortRegistry {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_channel_table() -> ChannelTable {
        ChannelTable::new()
    }

    #[test]
    fn test_port_state() {
        let listening = PortState::Listening { pending_count: 0 };
        assert!(listening.is_listening());
        assert!(!listening.is_closed());
        assert_eq!(listening.pending_count(), 0);

        let closed = PortState::Closed;
        assert!(!closed.is_listening());
        assert!(closed.is_closed());
    }

    #[test]
    fn test_pending_connection() {
        let conn = PendingConnection::new(1, 10, 20, 1000);
        assert!(!conn.is_expired(1000));
        assert!(!conn.is_expired(1000 + PENDING_TIMEOUT - 1));
        assert!(conn.is_expired(1000 + PENDING_TIMEOUT + 1));
    }

    #[test]
    fn test_port_add_accept() {
        let mut port = Port::new(1, b"test:", 1, 100);

        // Add pending
        let conn = PendingConnection::new(2, 10, 20, 0);
        port.add_pending(conn).unwrap();

        assert_eq!(port.state().pending_count(), 1);

        // Accept
        let (server_ch, client_pid) = port.accept().unwrap();
        assert_eq!(server_ch, 20);
        assert_eq!(client_pid, 2);
        assert_eq!(port.state().pending_count(), 0);
    }

    #[test]
    fn test_port_poll() {
        let mut port = Port::new(1, b"test:", 1, 100);

        // No pending - not ready for accept
        assert!(!port.poll(WakeReason::Accepted));

        // Add pending
        let conn = PendingConnection::new(2, 10, 20, 0);
        port.add_pending(conn).unwrap();

        // Now ready
        assert!(port.poll(WakeReason::Accepted));
    }

    #[test]
    fn test_registry_register() {
        let mut reg = PortRegistry::new();
        let mut table = make_test_channel_table();

        let (port_id, listen_ch) = reg.register(b"devd:", 1, &mut table).unwrap();

        assert!(port_id > 0);
        assert!(listen_ch > 0);
        assert_eq!(reg.active_count(), 1);
    }

    #[test]
    fn test_registry_duplicate_name() {
        let mut reg = PortRegistry::new();
        let mut table = make_test_channel_table();

        reg.register(b"devd:", 1, &mut table).unwrap();

        // Duplicate should fail
        let result = reg.register(b"devd:", 2, &mut table);
        assert!(matches!(result, Err(IpcError::PortExists { .. })));
    }

    #[test]
    fn test_registry_connect_accept() {
        let mut reg = PortRegistry::new();
        let mut table = make_test_channel_table();

        // Server registers port
        let (port_id, _listen_ch) = reg.register(b"svc:", 1, &mut table).unwrap();

        // Client connects
        let (client_ch, wake_list) = reg.connect(b"svc:", 2, &mut table).unwrap();
        assert!(client_ch > 0);

        // Server accepts
        let (server_ch, client_pid) = reg.accept(port_id, 1).unwrap();
        assert!(server_ch > 0);
        assert_eq!(client_pid, 2);
    }

    #[test]
    fn test_registry_connect_not_found() {
        let mut reg = PortRegistry::new();
        let mut table = make_test_channel_table();

        let result = reg.connect(b"nonexistent:", 1, &mut table);
        assert!(matches!(result, Err(IpcError::PortNotFound)));
    }

    #[test]
    fn test_registry_accept_not_owner() {
        let mut reg = PortRegistry::new();
        let mut table = make_test_channel_table();

        let (port_id, _) = reg.register(b"svc:", 1, &mut table).unwrap();

        // Non-owner tries to accept
        let result = reg.accept(port_id, 2);
        assert!(matches!(result, Err(IpcError::PortNotOwner { .. })));
    }

    #[test]
    fn test_registry_cleanup_task() {
        let mut reg = PortRegistry::new();
        let mut table = make_test_channel_table();

        reg.register(b"port1:", 1, &mut table).unwrap();
        reg.register(b"port2:", 1, &mut table).unwrap();
        reg.register(b"port3:", 2, &mut table).unwrap();

        assert_eq!(reg.active_count(), 3);

        reg.cleanup_task(1);

        assert_eq!(reg.active_count(), 1);
    }
}
