//! Channel State Machine
//!
//! Channels are bidirectional message-passing endpoints. Each channel
//! has exactly one peer (the other end of the connection).
//!
//! # State Diagram
//!
//! ```text
//!     ┌────────────┐
//!     │    Open    │◄─── create_pair()
//!     └─────┬──────┘
//!           │
//!           │ peer closes
//!           ▼
//!     ┌────────────┐
//!     │ HalfClosed │  (can still receive pending messages)
//!     └─────┬──────┘
//!           │
//!           │ queue empty OR we close
//!           ▼
//!     ┌────────────┐
//!     │   Closed   │
//!     └────────────┘
//! ```
//!
//! # Invariants
//!
//! 1. A channel pair is always created together (atomic)
//! 2. HalfClosed only entered when peer closes
//! 3. Subscribers are notified on every state transition
//! 4. Messages can be drained from HalfClosed state
//! 5. Send fails immediately on HalfClosed or Closed

use super::types::{ChannelId, TaskId, Message};
use super::queue::MessageQueue;
use super::traits::{Closable, CloseAction};
use super::error::IpcError;

// ============================================================================
// Channel State
// ============================================================================

/// Channel lifecycle state - SINGLE source of truth
///
/// No more `blocked_receiver` + `peer_closed` + `state` mess.
/// This enum IS the complete state.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ChannelState {
    /// Channel is open and connected to peer
    Open {
        /// Peer channel ID
        peer_id: ChannelId,
        /// Peer owner task ID
        peer_owner: TaskId,
    },

    /// Peer has closed, but messages may still be in queue
    /// Allows draining remaining messages before full close
    HalfClosed {
        /// Number of messages remaining in queue
        messages_remaining: u8,
    },

    /// Channel is fully closed - no operations allowed
    Closed,
}

impl ChannelState {
    /// Check if state is open
    pub fn is_open(&self) -> bool {
        matches!(self, ChannelState::Open { .. })
    }

    /// Check if state is half-closed
    pub fn is_half_closed(&self) -> bool {
        matches!(self, ChannelState::HalfClosed { .. })
    }

    /// Check if state is closed
    pub fn is_closed(&self) -> bool {
        matches!(self, ChannelState::Closed)
    }

    /// Check if we can still receive messages
    pub fn can_receive(&self) -> bool {
        match self {
            ChannelState::Open { .. } => true,
            ChannelState::HalfClosed { messages_remaining } => *messages_remaining > 0,
            ChannelState::Closed => false,
        }
    }

    /// Check if we can still send messages
    pub fn can_send(&self) -> bool {
        matches!(self, ChannelState::Open { .. })
    }

    /// Valid state transitions
    pub fn can_transition_to(&self, new: &ChannelState) -> bool {
        match (self, new) {
            // Open -> HalfClosed (peer closes)
            (ChannelState::Open { .. }, ChannelState::HalfClosed { .. }) => true,
            // Open -> Closed (we close)
            (ChannelState::Open { .. }, ChannelState::Closed) => true,
            // HalfClosed -> Closed (queue drained or we close)
            (ChannelState::HalfClosed { .. }, ChannelState::Closed) => true,
            // All other transitions are invalid
            _ => false,
        }
    }

    /// Get peer ID if open
    pub fn peer_id(&self) -> Option<ChannelId> {
        match self {
            ChannelState::Open { peer_id, .. } => Some(*peer_id),
            _ => None,
        }
    }
}

// ============================================================================
// Channel
// ============================================================================

/// How writes on a channel are dispatched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DispatchMode {
    /// Normal IPC — messages queued for peer
    Peer,
    /// Kernel bus dispatch — writes are intercepted by kernel bus controller.
    /// Set for client channels connected to /kernel/bus/* ports.
    KernelBus,
}

/// A complete channel endpoint
///
/// Contains the state machine and message queue.
/// Wake notifications are handled by the Object layer's WaitQueue, not here.
pub struct Channel {
    /// This channel's ID
    id: ChannelId,

    /// Owner task ID
    owner: TaskId,

    /// Current state (single source of truth)
    state: ChannelState,

    /// Incoming message queue
    queue: MessageQueue,

    /// How writes on this channel are dispatched.
    dispatch_mode: DispatchMode,
}

impl Channel {
    /// Create a new channel in Open state
    pub fn new(id: ChannelId, owner: TaskId, peer_id: ChannelId, peer_owner: TaskId) -> Self {
        Self {
            id,
            owner,
            state: ChannelState::Open { peer_id, peer_owner },
            queue: MessageQueue::new(),
            dispatch_mode: DispatchMode::Peer,
        }
    }

    /// Create a new channel in Closed state (placeholder)
    pub const fn closed() -> Self {
        Self {
            id: 0,
            owner: 0,
            state: ChannelState::Closed,
            queue: MessageQueue::new(),
            dispatch_mode: DispatchMode::Peer,
        }
    }

    // ========================================================================
    // Getters
    // ========================================================================

    /// Get channel ID
    pub fn id(&self) -> ChannelId {
        self.id
    }

    /// Get owner task ID
    pub fn owner(&self) -> TaskId {
        self.owner
    }

    /// Get current state
    pub fn state(&self) -> &ChannelState {
        &self.state
    }

    /// Get peer channel ID (if open)
    pub fn peer_id(&self) -> Option<ChannelId> {
        self.state.peer_id()
    }

    /// Check if writes on this channel dispatch to a kernel bus controller
    pub fn is_kernel_dispatch(&self) -> bool {
        self.dispatch_mode == DispatchMode::KernelBus
    }

    /// Mark this channel for kernel bus dispatch
    pub fn set_kernel_dispatch(&mut self) {
        self.dispatch_mode = DispatchMode::KernelBus;
    }

    /// Get message queue reference
    pub fn queue(&self) -> &MessageQueue {
        &self.queue
    }

    /// Get message queue mutably
    pub fn queue_mut(&mut self) -> &mut MessageQueue {
        &mut self.queue
    }

    /// Check if queue has messages
    pub fn has_messages(&self) -> bool {
        !self.queue.is_empty()
    }

    /// Check if queue has space
    pub fn has_space(&self) -> bool {
        !self.queue.is_full()
    }

    // ========================================================================
    // Operations
    // ========================================================================

    /// Push a message to this channel's queue (called by peer's send)
    ///
    /// Just pushes the message. Wake notifications are handled by the caller
    /// via the Object layer's WaitQueue.
    pub fn deliver(&mut self, msg: Message) -> Result<(), IpcError> {
        if self.state.is_closed() {
            return Err(IpcError::Closed);
        }

        if !self.queue.push(msg) {
            return Err(IpcError::QueueFull);
        }

        Ok(())
    }

    /// Receive a message from queue
    pub fn receive(&mut self) -> Result<Message, IpcError> {
        // Try to pop a message
        if let Some(msg) = self.queue.pop() {
            // If half-closed and queue now empty, transition to Closed
            if let ChannelState::HalfClosed { .. } = &self.state {
                if self.queue.is_empty() {
                    // SAFETY: HalfClosed -> Closed is always valid when queue drains
                    self.state = ChannelState::Closed;
                } else {
                    // SAFETY: HalfClosed can always update messages_remaining (same state variant)
                    self.state = ChannelState::HalfClosed {
                        messages_remaining: self.queue.len() as u8,
                    };
                }
            }
            Ok(msg)
        } else {
            // No messages - return appropriate error based on state
            match &self.state {
                ChannelState::Open { .. } => Err(IpcError::WouldBlock),
                ChannelState::HalfClosed { .. } => Err(IpcError::PeerClosed),
                ChannelState::Closed => Err(IpcError::Closed),
            }
        }
    }

    /// Notify that peer has closed
    ///
    /// Transitions to HalfClosed or Closed state.
    /// Wake notifications are handled by the caller via the Object layer's WaitQueue.
    pub fn notify_peer_closed(&mut self) {
        match &self.state {
            ChannelState::Open { .. } => {
                let remaining = self.queue.len() as u8;
                if remaining > 0 {
                    // SAFETY: Open -> HalfClosed is always valid per state machine
                    self.state = ChannelState::HalfClosed { messages_remaining: remaining };
                } else {
                    // SAFETY: Open -> Closed is valid when no pending messages
                    self.state = ChannelState::Closed;
                }
            }
            _ => {} // Already closed
        }
    }

    /// Close this channel
    ///
    /// Wake notifications are handled by the caller via the Object layer's WaitQueue.
    pub fn do_close(&mut self) {
        // SAFETY: Any state -> Closed is always valid (terminal state)
        self.state = ChannelState::Closed;
    }
}

// ============================================================================
// Trait Implementations
// ============================================================================

impl Closable for Channel {
    fn close(&mut self, _owner: TaskId) -> CloseAction {
        // Get peer ID before closing
        let peer_id = self.peer_id();
        self.do_close();

        // Return action to close peer
        if let Some(peer) = peer_id {
            CloseAction::ClosePeer { peer_id: peer }
        } else {
            CloseAction::None
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_channel_state_transitions() {
        let open = ChannelState::Open { peer_id: 2, peer_owner: 1 };
        let half = ChannelState::HalfClosed { messages_remaining: 3 };
        let closed = ChannelState::Closed;

        // Valid transitions
        assert!(open.can_transition_to(&half));
        assert!(open.can_transition_to(&closed));
        assert!(half.can_transition_to(&closed));

        // Invalid transitions
        assert!(!closed.can_transition_to(&open));
        assert!(!closed.can_transition_to(&half));
        assert!(!half.can_transition_to(&open));
    }

    #[test]
    fn test_channel_state_queries() {
        let open = ChannelState::Open { peer_id: 2, peer_owner: 1 };
        assert!(open.is_open());
        assert!(!open.is_closed());
        assert!(open.can_send());
        assert!(open.can_receive());

        let half = ChannelState::HalfClosed { messages_remaining: 3 };
        assert!(half.is_half_closed());
        assert!(!half.can_send());
        assert!(half.can_receive());

        let half_empty = ChannelState::HalfClosed { messages_remaining: 0 };
        assert!(!half_empty.can_receive());

        let closed = ChannelState::Closed;
        assert!(closed.is_closed());
        assert!(!closed.can_send());
        assert!(!closed.can_receive());
    }

    #[test]
    fn test_channel_deliver_receive() {
        let mut ch = Channel::new(1, 10, 2, 20);

        // Deliver a message
        let msg = Message::data(20, b"hello");
        ch.deliver(msg).unwrap();

        // Deliver another
        let msg2 = Message::data(20, b"world");
        ch.deliver(msg2).unwrap();

        // Receive messages
        let received1 = ch.receive().unwrap();
        assert_eq!(received1.payload_slice(), b"hello");

        let received2 = ch.receive().unwrap();
        assert_eq!(received2.payload_slice(), b"world");

        // No more messages
        assert_eq!(ch.receive(), Err(IpcError::WouldBlock));
    }

    #[test]
    fn test_channel_half_closed_drain() {
        let mut ch = Channel::new(1, 10, 2, 20);

        // Deliver some messages
        ch.deliver(Message::data(20, b"msg1")).unwrap();
        ch.deliver(Message::data(20, b"msg2")).unwrap();

        // Peer closes
        ch.notify_peer_closed();
        assert!(ch.state().is_half_closed());

        // Can still receive
        let msg1 = ch.receive().unwrap();
        assert_eq!(msg1.payload_slice(), b"msg1");

        let msg2 = ch.receive().unwrap();
        assert_eq!(msg2.payload_slice(), b"msg2");

        // Now fully closed
        assert!(ch.state().is_closed());
        assert_eq!(ch.receive(), Err(IpcError::Closed));
    }

    #[test]
    fn test_channel_deliver_to_closed() {
        let mut ch = Channel::new(1, 10, 2, 20);
        ch.do_close();

        let result = ch.deliver(Message::data(20, b"test"));
        assert_eq!(result, Err(IpcError::Closed));
    }
}
