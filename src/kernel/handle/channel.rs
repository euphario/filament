//! Channel Handle Object
//!
//! Wraps the existing IPC channel system with the handle abstraction.

use super::traits::{Waitable, Closable, WaitResult, WaitFilter, CloseAction};
use crate::kernel::task::TaskId;
use crate::kernel::ipc::ChannelId;

/// Channel object for handle system
///
/// Wraps an IPC channel endpoint. The actual channel state lives in
/// the global ChannelTable - this just holds the reference.
#[derive(Clone, Copy, Debug)]
pub struct ChannelObject {
    /// Global channel ID
    pub channel_id: ChannelId,
}

impl ChannelObject {
    /// Create a new channel object
    pub const fn new(channel_id: ChannelId) -> Self {
        Self { channel_id }
    }

    /// Get the channel ID
    pub const fn id(&self) -> ChannelId {
        self.channel_id
    }

    /// Check if messages are available
    pub fn has_messages(&self) -> bool {
        crate::kernel::ipc::channel_has_messages(self.channel_id)
    }

    /// Check if peer is closed
    pub fn is_peer_closed(&self) -> bool {
        crate::kernel::ipc::channel_is_closed(self.channel_id)
    }
}

impl Waitable for ChannelObject {
    fn poll(&self, filter: WaitFilter) -> Option<WaitResult> {
        match filter {
            WaitFilter::Readable => {
                // Check if messages available
                if self.has_messages() {
                    Some(WaitResult::new(
                        super::Handle::INVALID, // Caller fills in
                        WaitFilter::Readable,
                        0,
                    ))
                } else if crate::kernel::ipc::has_pending_accept(self.channel_id) {
                    // This channel is a port's listen channel with pending connections
                    Some(WaitResult::new(
                        super::Handle::INVALID,
                        WaitFilter::Readable,
                        0,
                    ))
                } else if self.is_peer_closed() {
                    // Peer closed - return Closed event
                    Some(WaitResult::new(
                        super::Handle::INVALID,
                        WaitFilter::Closed,
                        0,
                    ))
                } else {
                    None
                }
            }
            WaitFilter::Writable => {
                // Check if we can send (queue not full)
                if crate::kernel::ipc::channel_can_send(self.channel_id) {
                    Some(WaitResult::new(
                        super::Handle::INVALID,
                        WaitFilter::Writable,
                        0,
                    ))
                } else {
                    None
                }
            }
            WaitFilter::Closed => {
                if self.is_peer_closed() {
                    Some(WaitResult::new(
                        super::Handle::INVALID,
                        WaitFilter::Closed,
                        0,
                    ))
                } else {
                    None
                }
            }
            _ => None, // Other filters not supported for channels
        }
    }

    fn register_waker(&mut self, task_id: TaskId) {
        // Register with the channel table to be woken when message arrives
        crate::kernel::ipc::channel_register_waker(self.channel_id, task_id);
        // Also register for port notifications (if this is a listen channel)
        crate::kernel::ipc::port_register_waker_for_channel(self.channel_id, task_id);
    }

    fn unregister_waker(&mut self, task_id: TaskId) {
        crate::kernel::ipc::channel_unregister_waker(self.channel_id, task_id);
        crate::kernel::ipc::port_unregister_waker_for_channel(self.channel_id, task_id);
    }
}

impl Closable for ChannelObject {
    fn close(&self, owner_pid: u32) -> CloseAction {
        // Close the channel in the global table
        let _ = crate::kernel::ipc::sys_channel_close(self.channel_id, owner_pid);
        CloseAction::CloseChannel { channel_id: self.channel_id }
    }
}
