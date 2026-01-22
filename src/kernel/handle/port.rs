//! Port Handle Object
//!
//! Represents a named listening endpoint for IPC connections.
//!
//! ## State Machine
//!
//! ```text
//!     ┌─────────────┐
//!     │  Listening  │◄──── new()
//!     └──────┬──────┘
//!            │
//!            │ close()
//!            ▼
//!     ┌─────────────┐
//!     │   Closed    │
//!     └─────────────┘
//! ```
//!
//! ## Events
//!
//! - `Readable`: A client connection is pending (accept will succeed)
//! - `Closed`: Port was closed

use super::{Handle, Waitable, Closable, WaitFilter, WaitResult, CloseAction};
use crate::kernel::task::TaskId;

// ============================================================================
// Port State
// ============================================================================

/// Port lifecycle state
///
/// State machine:
/// ```text
///   Listening ──────► Closed
/// ```
/// Once closed, a port cannot be reopened.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortState {
    /// Port is listening for connections
    Listening,
    /// Port has been closed
    Closed,
}

impl PortState {
    /// Check if transition to new state is valid
    pub fn can_transition_to(&self, new: &PortState) -> bool {
        match (self, new) {
            // Listening → Closed (normal close)
            (PortState::Listening, PortState::Closed) => true,
            // Cannot reopen a closed port
            _ => false,
        }
    }

    /// Get state name for logging
    pub fn name(&self) -> &'static str {
        match self {
            PortState::Listening => "Listening",
            PortState::Closed => "Closed",
        }
    }
}

// ============================================================================
// Port Error
// ============================================================================

/// Errors from port operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortError {
    /// Port is closed
    Closed,
    /// No pending connections
    WouldBlock,
    /// Internal error
    Internal(i64),
}

// ============================================================================
// Port Object
// ============================================================================

/// Port object - a named listening endpoint
///
/// Wraps the kernel's port system with a handle-based interface.
/// Pure state machine - all syscalls happen in the executor.
pub struct PortObject {
    /// Current state
    state: PortState,

    /// Underlying listen channel ID
    listen_channel: u32,

    /// Port name (for debugging/logging)
    name: [u8; 32],
    name_len: u8,

    /// Tasks waiting for connections
    waiters: [TaskId; 4],
    waiter_count: u8,
}

impl PortObject {
    /// Maximum name length
    pub const MAX_NAME_LEN: usize = 32;

    /// Create a new port object
    ///
    /// Called after port_register succeeds. The listen_channel is the
    /// channel ID returned by the kernel port system.
    pub fn new(listen_channel: u32, name: &[u8]) -> Self {
        let mut port_name = [0u8; 32];
        let len = name.len().min(Self::MAX_NAME_LEN);
        port_name[..len].copy_from_slice(&name[..len]);

        Self {
            state: PortState::Listening,
            listen_channel,
            name: port_name,
            name_len: len as u8,
            waiters: [0; 4],
            waiter_count: 0,
        }
    }

    /// Get current state
    #[inline]
    pub fn state(&self) -> PortState {
        self.state
    }

    /// Get underlying listen channel ID
    #[inline]
    pub fn listen_channel(&self) -> u32 {
        self.listen_channel
    }

    /// Get port name as str (for logging)
    pub fn name_str(&self) -> &str {
        core::str::from_utf8(&self.name[..self.name_len as usize]).unwrap_or("<invalid>")
    }

    /// Check if a connection is pending
    ///
    /// Pure function - doesn't perform syscalls.
    /// Caller must check with the actual port system.
    pub fn has_pending_connection(&self, pending: bool) -> Option<WaitResult> {
        match self.state {
            PortState::Listening if pending => {
                Some(WaitResult::new(Handle::INVALID, WaitFilter::Readable, 0))
            }
            PortState::Closed => {
                Some(WaitResult::new(Handle::INVALID, WaitFilter::Closed, 0))
            }
            _ => None,
        }
    }

    /// Transition: accept a connection
    ///
    /// Returns the client channel ID on success.
    /// Pure state check - actual accept happens in executor.
    pub fn can_accept(&self) -> Result<(), PortError> {
        match self.state {
            PortState::Listening => Ok(()),
            PortState::Closed => Err(PortError::Closed),
        }
    }

    /// Transition: close the port
    ///
    /// Moves to Closed state. Actual cleanup happens via CloseAction.
    pub fn close(&mut self) {
        self.state = PortState::Closed;
    }

    /// Wake all waiting tasks
    ///
    /// Called when a connection arrives. The event system handles waking
    /// via IpcReady events pushed to the port owner's event queue.
    pub fn wake_waiters(&self) {
        // Waking is handled by the IPC/event system when connections arrive.
        // The scheduler wakes tasks with pending events when they become ready.
        // See kernel/port.rs connect() which pushes IpcReady events.
    }
}

// ============================================================================
// Waitable Implementation
// ============================================================================

impl Waitable for PortObject {
    fn poll(&self, filter: WaitFilter) -> Option<WaitResult> {
        match (self.state, filter) {
            // Can only wait for Readable (connection pending) on listening port
            (PortState::Listening, WaitFilter::Readable) => {
                // Check with port system if connection pending
                let pending = crate::kernel::ipc::has_pending_accept(self.listen_channel);
                if pending {
                    Some(WaitResult::new(Handle::INVALID, WaitFilter::Readable, 0))
                } else {
                    None
                }
            }

            // Closed port always signals Closed
            (PortState::Closed, WaitFilter::Closed) => {
                Some(WaitResult::new(Handle::INVALID, WaitFilter::Closed, 0))
            }

            // Any filter on closed port returns Closed
            (PortState::Closed, _) => {
                Some(WaitResult::new(Handle::INVALID, WaitFilter::Closed, 0))
            }

            // Other combinations: would block
            _ => None,
        }
    }

    fn register_waker(&mut self, task_id: TaskId) {
        if (self.waiter_count as usize) < self.waiters.len() {
            // Check not already registered
            for i in 0..self.waiter_count as usize {
                if self.waiters[i] == task_id {
                    return;
                }
            }
            self.waiters[self.waiter_count as usize] = task_id;
            self.waiter_count += 1;

            // Also register on the listen channel so port.rs::connect() can wake us
            crate::kernel::ipc::channel_register_waker(self.listen_channel, task_id);
        }
    }

    fn unregister_waker(&mut self, task_id: TaskId) {
        for i in 0..self.waiter_count as usize {
            if self.waiters[i] == task_id {
                // Shift remaining elements
                for j in i..self.waiter_count as usize - 1 {
                    self.waiters[j] = self.waiters[j + 1];
                }
                self.waiter_count -= 1;
                self.waiters[self.waiter_count as usize] = 0;

                // Also unregister from the listen channel
                crate::kernel::ipc::channel_unregister_waker(self.listen_channel, task_id);
                return;
            }
        }
    }
}

// ============================================================================
// Closable Implementation
// ============================================================================

impl Closable for PortObject {
    fn close(&self, _owner_pid: u32) -> CloseAction {
        CloseAction::UnregisterPort { port_id: self.listen_channel }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_port_is_listening() {
        let port = PortObject::new(42, b"test:");
        assert_eq!(port.state(), PortState::Listening);
        assert_eq!(port.listen_channel(), 42);
    }

    #[test]
    fn test_close_transitions_to_closed() {
        let mut port = PortObject::new(42, b"test:");
        port.close();
        assert_eq!(port.state(), PortState::Closed);
    }

    #[test]
    fn test_can_accept_when_listening() {
        let port = PortObject::new(42, b"test:");
        assert!(port.can_accept().is_ok());
    }

    #[test]
    fn test_cannot_accept_when_closed() {
        let mut port = PortObject::new(42, b"test:");
        port.close();
        assert_eq!(port.can_accept(), Err(PortError::Closed));
    }

    #[test]
    fn test_has_pending_connection() {
        let port = PortObject::new(42, b"test:");

        // No pending connection
        assert!(port.has_pending_connection(false).is_none());

        // Pending connection
        let result = port.has_pending_connection(true);
        assert!(result.is_some());
        assert_eq!(result.unwrap().filter, WaitFilter::Readable);
    }

    #[test]
    fn test_closed_port_returns_closed_event() {
        let mut port = PortObject::new(42, b"test:");
        port.close();

        let result = port.has_pending_connection(true);
        assert!(result.is_some());
        assert_eq!(result.unwrap().filter, WaitFilter::Closed);
    }

    #[test]
    fn test_name_str() {
        let port = PortObject::new(42, b"pcie:");
        assert_eq!(port.name_str(), "pcie:");
    }
}
