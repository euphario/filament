//! IPC Trait Definitions
//!
//! These traits define the contract for IPC operations.
//! The object layer and syscall handlers use these traits exclusively.
//!
//! # Key Principle: Single Source of Truth
//!
//! The implementation owns all state. Callers NEVER cache state locally.
//! Every state query goes through the trait method.
//!
//! ```ignore
//! // WRONG: Caching state
//! struct ChannelWrapper {
//!     channel: Arc<dyn IpcChannel>,
//!     cached_state: ChannelState,  // NO! State can get stale
//! }
//!
//! // RIGHT: Always query
//! struct ChannelWrapper {
//!     channel: Arc<dyn IpcChannel>,
//! }
//! impl ChannelWrapper {
//!     fn state(&self) -> ChannelState {
//!         self.channel.state()  // Always authoritative
//!     }
//! }
//! ```

pub use super::waker::{Subscriber, WakeReason, WakeList};

/// Channel identifier
pub type ChannelId = u32;

/// Port identifier
pub type PortId = u32;

/// Task identifier
pub type TaskId = u32;

// ============================================================================
// Message Type (simplified for trait boundary)
// ============================================================================

/// Message for IPC - simplified version for trait boundary
///
/// The actual Message struct lives in ipc::types, but this trait
/// uses a simpler interface to avoid tight coupling.
#[derive(Clone)]
pub struct Message {
    /// Sender task ID
    pub sender: TaskId,
    /// Payload data
    pub payload: [u8; 256],
    /// Actual payload length
    pub len: usize,
}

impl Message {
    /// Create a new message
    pub fn new(sender: TaskId, data: &[u8]) -> Self {
        let mut payload = [0u8; 256];
        let len = data.len().min(256);
        payload[..len].copy_from_slice(&data[..len]);
        Self { sender, payload, len }
    }

    /// Get payload slice
    pub fn data(&self) -> &[u8] {
        &self.payload[..self.len]
    }
}

// ============================================================================
// Error Types
// ============================================================================

/// IPC errors for trait boundary
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IpcError {
    /// Channel/port not found
    NotFound,
    /// Caller is not the owner
    NotOwner,
    /// Peer has closed
    PeerClosed,
    /// Would block (no data available)
    WouldBlock,
    /// Queue is full
    QueueFull,
    /// No slots available
    NoSlots,
    /// Port is closed
    PortClosed,
    /// No pending connections
    NoPending,
    /// Subscriber set is full
    SubscribersFull,
    /// Invalid argument
    InvalidArg,
}

impl IpcError {
    /// Convert to errno for syscall return
    pub fn to_errno(self) -> i64 {
        match self {
            IpcError::NotFound => -2,      // ENOENT
            IpcError::NotOwner => -1,      // EPERM
            IpcError::PeerClosed => -104,  // ECONNRESET
            IpcError::WouldBlock => -11,   // EAGAIN
            IpcError::QueueFull => -11,    // EAGAIN
            IpcError::NoSlots => -12,      // ENOMEM
            IpcError::PortClosed => -111,  // ECONNREFUSED
            IpcError::NoPending => -11,    // EAGAIN
            IpcError::SubscribersFull => -12, // ENOMEM
            IpcError::InvalidArg => -22,   // EINVAL
        }
    }
}

// ============================================================================
// Channel State
// ============================================================================

/// Channel lifecycle state
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ChannelState {
    /// Channel is open with connected peer
    Open {
        peer_id: ChannelId,
        peer_owner: TaskId,
    },
    /// Peer closed, draining remaining messages
    HalfClosed {
        messages_remaining: u8,
    },
    /// Fully closed
    Closed,
}

impl ChannelState {
    pub fn is_open(&self) -> bool {
        matches!(self, ChannelState::Open { .. })
    }

    pub fn is_closed(&self) -> bool {
        matches!(self, ChannelState::Closed)
    }

    pub fn can_send(&self) -> bool {
        matches!(self, ChannelState::Open { .. })
    }

    pub fn can_receive(&self) -> bool {
        match self {
            ChannelState::Open { .. } => true,
            ChannelState::HalfClosed { messages_remaining } => *messages_remaining > 0,
            ChannelState::Closed => false,
        }
    }

    pub fn peer_id(&self) -> Option<ChannelId> {
        match self {
            ChannelState::Open { peer_id, .. } => Some(*peer_id),
            _ => None,
        }
    }
}

// ============================================================================
// Port State
// ============================================================================

/// Port lifecycle state
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PortState {
    /// Listening for connections
    Listening { pending_count: u8 },
    /// Closed
    Closed,
}

impl PortState {
    pub fn is_listening(&self) -> bool {
        matches!(self, PortState::Listening { .. })
    }

    pub fn pending_count(&self) -> u8 {
        match self {
            PortState::Listening { pending_count } => *pending_count,
            PortState::Closed => 0,
        }
    }
}

// ============================================================================
// IPC Channel Trait
// ============================================================================

/// Trait for IPC channel operations
///
/// # Contract
///
/// 1. `state()` always returns the authoritative state
/// 2. `send()`/`receive()` are atomic with state updates
/// 3. Subscribers are woken via returned WakeList, NEVER inside the call
/// 4. `subscribe()` returns error if subscriber set is full (never silent drop)
///
/// # Thread Safety
///
/// Implementations must be thread-safe. The trait uses `&self` for most
/// operations because internal locking handles mutation.
pub trait IpcChannel: Send + Sync {
    /// Get channel ID
    fn id(&self) -> ChannelId;

    /// Get owner task ID
    fn owner(&self) -> TaskId;

    /// Get current state (always authoritative)
    fn state(&self) -> ChannelState;

    /// Send a message to peer
    ///
    /// Returns subscribers to wake on success.
    /// Caller MUST wake them after this call returns.
    fn send(&self, msg: &Message) -> Result<WakeList, IpcError>;

    /// Receive a message
    ///
    /// Returns the next message or WouldBlock if none available.
    fn receive(&self) -> Result<Message, IpcError>;

    /// Check if ready for operation
    fn poll(&self, filter: WakeReason) -> bool;

    /// Subscribe to events
    ///
    /// # Errors
    /// Returns `IpcError::SubscribersFull` if subscriber set is full.
    /// NEVER silently drops subscriptions.
    fn subscribe(&self, sub: Subscriber, reason: WakeReason) -> Result<(), IpcError>;

    /// Unsubscribe from events
    fn unsubscribe(&self, sub: Subscriber);

    /// Close this channel
    ///
    /// Returns subscribers to wake (peer will be notified).
    fn close(&self) -> Result<WakeList, IpcError>;

    /// Check if channel has messages
    fn has_messages(&self) -> bool;

    /// Check if queue is full
    fn is_full(&self) -> bool;
}

// ============================================================================
// IPC Port Trait
// ============================================================================

/// Trait for IPC port operations
///
/// Ports are named endpoints that accept connections.
pub trait IpcPort: Send + Sync {
    /// Get port ID
    fn id(&self) -> PortId;

    /// Get port name
    fn name(&self) -> &[u8];

    /// Get owner task ID
    fn owner(&self) -> TaskId;

    /// Get current state (always authoritative)
    fn state(&self) -> PortState;

    /// Accept a pending connection
    ///
    /// Returns (server_channel_id, client_task_id) on success.
    fn accept(&self) -> Result<(ChannelId, TaskId), IpcError>;

    /// Check if ready for operation
    fn poll(&self, filter: WakeReason) -> bool;

    /// Subscribe to events
    ///
    /// # Errors
    /// Returns `IpcError::SubscribersFull` if subscriber set is full.
    fn subscribe(&self, sub: Subscriber, reason: WakeReason) -> Result<(), IpcError>;

    /// Unsubscribe from events
    fn unsubscribe(&self, sub: Subscriber);

    /// Close this port
    fn close(&self) -> Result<WakeList, IpcError>;
}

// ============================================================================
// IPC Backend Trait
// ============================================================================

/// Factory and registry for IPC objects
///
/// This is the main entry point for IPC operations.
/// The kernel holds a single instance implementing this trait.
///
/// # Design Note
///
/// All operations go through this trait directly rather than returning
/// references to individual channel/port objects. This allows the
/// implementation to use internal locking without lifetime issues.
///
/// The `IpcChannel` and `IpcPort` traits above are documentation of the
/// expected behavior, not interfaces that need to be implemented directly.
pub trait IpcBackend: Send + Sync {
    // ========================================================================
    // Channel Pair Creation
    // ========================================================================

    /// Create a channel pair
    ///
    /// Returns (channel_a, channel_b) where messages sent on A
    /// arrive on B and vice versa.
    fn create_channel_pair(
        &self,
        owner_a: TaskId,
        owner_b: TaskId,
    ) -> Result<(ChannelId, ChannelId), IpcError>;

    // ========================================================================
    // Port Operations
    // ========================================================================

    /// Register a new port
    ///
    /// Returns (port_id, listen_channel_id).
    fn register_port(
        &self,
        name: &[u8],
        owner: TaskId,
    ) -> Result<(PortId, ChannelId), IpcError>;

    /// Connect to a named port
    ///
    /// Returns (client_channel_id, wake_list).
    fn connect_to_port(
        &self,
        name: &[u8],
        client: TaskId,
    ) -> Result<(ChannelId, WakeList), IpcError>;

    /// Accept connection on port
    fn accept_port(
        &self,
        port_id: PortId,
        owner: TaskId,
    ) -> Result<(ChannelId, TaskId), IpcError>;

    /// Close port
    fn close_port(
        &self,
        port_id: PortId,
        owner: TaskId,
    ) -> Result<WakeList, IpcError>;

    /// Get port state (no ownership check)
    fn port_state(&self, port_id: PortId) -> Option<PortState>;

    // ========================================================================
    // Channel Operations
    // ========================================================================

    /// Send on channel with ownership check
    fn send(
        &self,
        channel_id: ChannelId,
        msg: &Message,
        caller: TaskId,
    ) -> Result<WakeList, IpcError>;

    /// Receive from channel with ownership check
    fn receive(
        &self,
        channel_id: ChannelId,
        caller: TaskId,
    ) -> Result<Message, IpcError>;

    /// Close channel with ownership check
    fn close_channel(
        &self,
        channel_id: ChannelId,
        caller: TaskId,
    ) -> Result<WakeList, IpcError>;

    /// Subscribe to channel events
    fn subscribe_channel(
        &self,
        channel_id: ChannelId,
        caller: TaskId,
        sub: Subscriber,
        reason: WakeReason,
    ) -> Result<(), IpcError>;

    /// Poll channel readiness
    fn poll_channel(
        &self,
        channel_id: ChannelId,
        caller: TaskId,
        filter: WakeReason,
    ) -> Result<bool, IpcError>;

    /// Check if channel has messages (no ownership check)
    fn channel_has_messages(&self, channel_id: ChannelId) -> bool;

    /// Check if channel is closed (no ownership check)
    fn channel_is_closed(&self, channel_id: ChannelId) -> bool;

    /// Get channel state (no ownership check)
    fn channel_state(&self, channel_id: ChannelId) -> Option<ChannelState>;

    // ========================================================================
    // Cleanup
    // ========================================================================

    /// Clean up all IPC resources for a task
    fn cleanup_task(&self, task_id: TaskId) -> WakeList;

    /// Remove a task from all subscriber lists
    fn remove_subscriber(&self, task_id: TaskId);
}

