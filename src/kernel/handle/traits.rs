//! Handle Traits
//!
//! Defines traits for handle capabilities:
//! - Waitable: Objects that can be waited on (channels, timers, etc.)
//! - Closable: Objects that need cleanup on close

use super::Handle;
use crate::kernel::task::TaskId;

// ============================================================================
// Wait Filter
// ============================================================================

/// Filter for what events to wait for
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum WaitFilter {
    /// Data available to read
    Readable = 0,

    /// Space available to write
    Writable = 1,

    /// Peer closed / EOF
    Closed = 2,

    /// Error condition
    Error = 3,

    /// Child task exited
    ChildExit = 4,

    /// Timer expired
    Timer = 5,

    /// IRQ fired
    Irq = 6,

    /// Signal received
    Signal = 7,
}

impl WaitFilter {
    /// Create from raw u8 value
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(WaitFilter::Readable),
            1 => Some(WaitFilter::Writable),
            2 => Some(WaitFilter::Closed),
            3 => Some(WaitFilter::Error),
            4 => Some(WaitFilter::ChildExit),
            5 => Some(WaitFilter::Timer),
            6 => Some(WaitFilter::Irq),
            7 => Some(WaitFilter::Signal),
            _ => None,
        }
    }
}

// ============================================================================
// Wait Result
// ============================================================================

/// Result of a wait operation
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct WaitResult {
    /// Handle that became ready
    pub handle: Handle,

    /// Type of event that occurred
    pub filter: WaitFilter,

    /// Event-specific data
    /// - ChildExit: exit code
    /// - Readable: bytes available (if known)
    /// - Timer: deadline tick
    pub data: u64,
}

impl WaitResult {
    /// Create a new wait result
    pub const fn new(handle: Handle, filter: WaitFilter, data: u64) -> Self {
        Self { handle, filter, data }
    }

    /// Create an empty result (for arrays)
    pub const fn empty() -> Self {
        Self {
            handle: Handle::INVALID,
            filter: WaitFilter::Readable,
            data: 0,
        }
    }
}

// ============================================================================
// Waitable Trait
// ============================================================================

/// Objects that can be waited on via sys_wait
///
/// Implements poll-based waiting with task wake registration.
pub trait Waitable {
    /// Check if the object is ready
    ///
    /// Returns Some(WaitResult) if ready, None if would block.
    /// The returned handle field may be INVALID - caller fills it in.
    fn poll(&self, filter: WaitFilter) -> Option<WaitResult>;

    /// Register interest in this object becoming ready
    ///
    /// Called when sys_wait blocks waiting for this handle.
    /// When the object becomes ready, it should call wake_task().
    fn register_waker(&mut self, task_id: TaskId);

    /// Unregister interest
    ///
    /// Called when task is woken (by this or another handle) or times out.
    fn unregister_waker(&mut self, task_id: TaskId);
}

// ============================================================================
// Close Action
// ============================================================================

/// Action to take when closing a handle
#[derive(Clone, Copy, Debug)]
pub enum CloseAction {
    /// No special cleanup needed
    None,

    /// Close peer channel
    CloseChannel { channel_id: u32 },

    /// Release shmem reference
    ReleaseShmem { shmem_id: u32 },

    /// Disable IRQ
    DisableIrq { irq_num: u32 },

    /// Unregister port
    UnregisterPort { port_id: u32 },
}

// ============================================================================
// Closable Trait
// ============================================================================

/// Objects that need cleanup when their handle is closed
pub trait Closable {
    /// Perform cleanup for this object
    ///
    /// Called when the handle is closed (explicitly or on task exit).
    /// Returns any deferred action needed.
    fn close(&self, owner_pid: u32) -> CloseAction;
}

// ============================================================================
// Wait Request
// ============================================================================

/// A single wait request (handle + filter)
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct WaitRequest {
    /// Handle to wait on
    pub handle: Handle,

    /// What to wait for
    pub filter: WaitFilter,

    /// Padding for alignment
    pub _pad: [u8; 3],
}

impl WaitRequest {
    /// Create a new wait request
    pub const fn new(handle: Handle, filter: WaitFilter) -> Self {
        Self {
            handle,
            filter,
            _pad: [0; 3],
        }
    }
}
