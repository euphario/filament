//! IPC Traits and Core Types
//!
//! This module defines the contracts that kernel objects must implement:
//!
//! ## State Machine Traits
//! - `StateMachine`: Explicit state transitions with validation
//!
//! ## Waitable Traits
//! - `Subscriber`: Who to wake when an event occurs
//! - `WakeReason`: Why we're waking someone
//! - `Waitable`: Any object that can be waited on
//! - `Closable`: Objects that need cleanup on close
//!
//! # Design Philosophy
//!
//! 1. **Explicit State Machines**: Every object has a state enum with validated transitions
//! 2. **Uniform Interface**: All waitable objects implement the same traits
//! 3. **Wake Outside Locks**: Operations return subscribers to wake, caller wakes outside lock
//!
//! # State Machine Pattern
//!
//! All state enums should implement `StateMachine`:
//!
//! ```ignore
//! impl StateMachine for ChannelState {
//!     fn can_transition_to(&self, new: &Self) -> bool {
//!         match (self, new) {
//!             (Open, HalfClosed) => true,
//!             (Open, Closed) => true,
//!             (HalfClosed, Closed) => true,
//!             _ => false,
//!         }
//!     }
//!     fn name(&self) -> &'static str { ... }
//! }
//! ```

use super::types::TaskId;

// ============================================================================
// State Machine Trait
// ============================================================================

/// Trait for explicit state machine validation
///
/// All state enums should implement this to enforce valid transitions.
/// This catches bugs at runtime where invalid transitions are attempted.
///
/// # Contract
///
/// 1. `can_transition_to()` returns true only for valid state transitions
/// 2. `name()` returns a human-readable state name for logging
/// 3. Implementations should be exhaustive (handle all state pairs)
///
/// # Example
///
/// ```ignore
/// #[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// pub enum ServiceState {
///     Created,
///     Running,
///     Stopped,
/// }
///
/// impl StateMachine for ServiceState {
///     fn can_transition_to(&self, new: &Self) -> bool {
///         use ServiceState::*;
///         matches!((self, new),
///             (Created, Running) |
///             (Running, Stopped)
///         )
///     }
///
///     fn name(&self) -> &'static str {
///         match self {
///             Self::Created => "created",
///             Self::Running => "running",
///             Self::Stopped => "stopped",
///         }
///     }
/// }
/// ```
pub trait StateMachine: Sized {
    /// Check if transition from self to new state is valid
    fn can_transition_to(&self, new: &Self) -> bool;

    /// Get human-readable state name for logging
    fn name(&self) -> &'static str;

    /// Attempt transition, returning Ok if valid, Err with both states if invalid
    fn transition_to(&mut self, new: Self) -> Result<(), (Self, Self)>
    where
        Self: Clone,
    {
        if self.can_transition_to(&new) {
            *self = new;
            Ok(())
        } else {
            Err((self.clone(), new))
        }
    }
}

// ============================================================================
// Subscriber
// ============================================================================

/// A subscriber waiting for events on an object
///
/// The generation field prevents stale wakes when a task exits and
/// its slot is reused by a new task.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Subscriber {
    /// Task ID to wake
    pub task_id: TaskId,
    /// Generation counter to detect stale subscribers
    /// Must match the task's current generation
    pub generation: u32,
}

impl Subscriber {
    /// Create a new subscriber
    pub const fn new(task_id: TaskId, generation: u32) -> Self {
        Self { task_id, generation }
    }

    /// Create a subscriber for a task (generation 0 bypasses stale PID detection)
    ///
    /// Use this when you don't have access to task generation,
    /// but be aware it may cause spurious wakes to reused PIDs.
    /// Prefer using Subscriber::new() with proper generation when possible.
    pub const fn simple(task_id: TaskId) -> Self {
        Self { task_id, generation: 0 }
    }
}

// ============================================================================
// WakeReason
// ============================================================================

/// Reason for waking a subscriber
///
/// This allows subscribers to filter which events they care about.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WakeReason {
    /// Message available to read
    Readable,

    /// Queue has space for writing
    Writable,

    /// Object has been closed
    Closed,

    /// Connection accepted (for ports)
    Accepted,

    /// Timeout/deadline expired
    Timeout,

    /// Generic signal/notification
    Signal,

    /// Child process exited
    ChildExit,
}

impl WakeReason {
    /// Check if this reason matches a filter
    ///
    /// Used when a subscriber only wants certain events.
    pub fn matches(&self, filter: WakeReason) -> bool {
        *self == filter
    }

    /// Convert to a bitmask bit
    pub fn to_bit(&self) -> u8 {
        match self {
            WakeReason::Readable => 1 << 0,
            WakeReason::Writable => 1 << 1,
            WakeReason::Closed => 1 << 2,
            WakeReason::Accepted => 1 << 3,
            WakeReason::Timeout => 1 << 4,
            WakeReason::Signal => 1 << 5,
            WakeReason::ChildExit => 1 << 6,
        }
    }

    /// Create from bitmask (returns first matching reason)
    pub fn from_bits(bits: u8) -> Option<Self> {
        if bits & (1 << 0) != 0 { return Some(WakeReason::Readable); }
        if bits & (1 << 1) != 0 { return Some(WakeReason::Writable); }
        if bits & (1 << 2) != 0 { return Some(WakeReason::Closed); }
        if bits & (1 << 3) != 0 { return Some(WakeReason::Accepted); }
        if bits & (1 << 4) != 0 { return Some(WakeReason::Timeout); }
        if bits & (1 << 5) != 0 { return Some(WakeReason::Signal); }
        if bits & (1 << 6) != 0 { return Some(WakeReason::ChildExit); }
        None
    }
}

// ============================================================================
// CloseAction
// ============================================================================

/// Action to take after closing an object
///
/// Some objects require cleanup that must happen outside the lock.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CloseAction {
    /// No special action needed
    None,

    /// Unregister this port from the global registry
    UnregisterPort { port_id: u32 },

    /// Close peer channel
    ClosePeer { peer_id: u32 },

    /// Custom action with data
    Custom { action_type: u32, data: u32 },
}

// ============================================================================
// Waitable Trait
// ============================================================================

/// Trait for objects that can be waited on
///
/// # Contract
///
/// 1. `poll()` returns true if object is ready for the given operation
/// 2. `subscribe()` registers a subscriber for future events
/// 3. `unsubscribe()` removes a subscriber
/// 4. Object MUST wake all relevant subscribers when state changes
///
/// # Example Implementation
///
/// ```ignore
/// impl Waitable for Channel {
///     fn poll(&self, filter: WakeReason) -> bool {
///         match filter {
///             WakeReason::Readable => !self.queue.is_empty(),
///             WakeReason::Closed => self.state.is_closed(),
///             _ => false,
///         }
///     }
///
///     fn subscribe(&mut self, sub: Subscriber, filter: WakeReason) {
///         self.subscribers.add(sub, filter);
///     }
///
///     fn unsubscribe(&mut self, sub: Subscriber) {
///         self.subscribers.remove(sub);
///     }
/// }
/// ```
pub trait Waitable {
    /// Check if object is ready for the given operation
    ///
    /// This is a non-blocking poll. Returns true if the operation
    /// would succeed without blocking.
    fn poll(&self, filter: WakeReason) -> bool;

    /// Subscribe to events on this object
    ///
    /// The subscriber will be woken when events matching the filter occur.
    /// Returns Err(()) if subscriber set is full (resource exhaustion).
    fn subscribe(&mut self, sub: Subscriber, filter: WakeReason) -> Result<(), ()>;

    /// Unsubscribe from events
    ///
    /// The subscriber will no longer be woken for events on this object.
    fn unsubscribe(&mut self, sub: Subscriber);
}

// ============================================================================
// Closable Trait
// ============================================================================

/// Trait for objects that need cleanup on close
///
/// # Contract
///
/// 1. `close()` performs object-specific cleanup
/// 2. Returns a `CloseAction` indicating what else needs to happen
/// 3. Caller must perform the CloseAction outside any locks
pub trait Closable {
    /// Close this object
    ///
    /// Returns an action that may need to be performed outside locks.
    fn close(&mut self, owner: TaskId) -> CloseAction;
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_subscriber_new() {
        let sub = Subscriber::new(42, 5);
        assert_eq!(sub.task_id, 42);
        assert_eq!(sub.generation, 5);
    }

    #[test]
    fn test_subscriber_simple() {
        let sub = Subscriber::simple(42);
        assert_eq!(sub.task_id, 42);
        assert_eq!(sub.generation, 0);
    }

    #[test]
    fn test_subscriber_equality() {
        let a = Subscriber::new(1, 1);
        let b = Subscriber::new(1, 1);
        let c = Subscriber::new(1, 2);
        let d = Subscriber::new(2, 1);

        assert_eq!(a, b);
        assert_ne!(a, c);
        assert_ne!(a, d);
    }

    #[test]
    fn test_wake_reason_to_bit() {
        assert_eq!(WakeReason::Readable.to_bit(), 1);
        assert_eq!(WakeReason::Writable.to_bit(), 2);
        assert_eq!(WakeReason::Closed.to_bit(), 4);
        assert_eq!(WakeReason::Accepted.to_bit(), 8);
    }

    #[test]
    fn test_wake_reason_from_bits() {
        assert_eq!(WakeReason::from_bits(1), Some(WakeReason::Readable));
        assert_eq!(WakeReason::from_bits(2), Some(WakeReason::Writable));
        assert_eq!(WakeReason::from_bits(4), Some(WakeReason::Closed));
        assert_eq!(WakeReason::from_bits(0), None);
    }

    #[test]
    fn test_wake_reason_matches() {
        assert!(WakeReason::Readable.matches(WakeReason::Readable));
        assert!(!WakeReason::Readable.matches(WakeReason::Writable));
    }

    #[test]
    fn test_close_action() {
        let action = CloseAction::UnregisterPort { port_id: 42 };
        match action {
            CloseAction::UnregisterPort { port_id } => assert_eq!(port_id, 42),
            _ => panic!("wrong action"),
        }
    }
}
