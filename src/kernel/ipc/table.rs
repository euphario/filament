//! Channel Table
//!
//! Global table of all channel endpoints. Provides allocation, lookup,
//! and operations on channels.
//!
//! # Design
//!
//! - Fixed-size array of channel slots
//! - Channel IDs are indices with generation counters
//! - Operations return WakeLists - caller wakes outside lock
//! - Atomic pair creation (both or neither)
//!
//! # Thread Safety
//!
//! The table is protected by a SpinLock in mod.rs. All operations
//! assume the lock is held.

use super::types::{ChannelId, TaskId, Message, MAX_CHANNELS};
use super::channel::{Channel, ChannelState};
use super::traits::{Subscriber, WakeReason, Waitable};
use super::waker::WakeList;
use super::error::IpcError;

/// Channel table entry state
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SlotState {
    /// Slot is free for allocation
    Free,
    /// Slot contains an active channel
    Active,
}

/// Channel table managing all endpoints
pub struct ChannelTable {
    /// Channel slots
    channels: [Option<Channel>; MAX_CHANNELS],

    /// Slot states for fast lookup
    states: [SlotState; MAX_CHANNELS],

    /// Generation counters per slot (for stale ID detection)
    generations: [u8; MAX_CHANNELS],

    /// Next channel ID counter
    next_id: u32,

    /// Number of active channels
    active_count: usize,
}

impl ChannelTable {
    /// Create a new empty channel table
    pub const fn new() -> Self {
        const NONE: Option<Channel> = None;
        Self {
            channels: [NONE; MAX_CHANNELS],
            states: [SlotState::Free; MAX_CHANNELS],
            generations: [1u8; MAX_CHANNELS], // Start at 1 so 0 is invalid
            next_id: 1,
            active_count: 0,
        }
    }

    /// Get number of active channels
    pub fn active_count(&self) -> usize {
        self.active_count
    }

    /// Find a free slot
    fn find_free_slot(&self) -> Option<usize> {
        self.states.iter().position(|s| *s == SlotState::Free)
    }

    /// Find slot by channel ID
    fn find_slot(&self, id: ChannelId) -> Option<usize> {
        // Channel ID encodes slot index in low bits and generation in high bits
        let slot = (id & 0x00FFFFFF) as usize;
        let gen = (id >> 24) as u8;

        if slot >= MAX_CHANNELS {
            return None;
        }

        if self.states[slot] == SlotState::Active && self.generations[slot] == gen {
            Some(slot)
        } else {
            None
        }
    }

    /// Allocate a new channel ID for a slot
    fn alloc_id(&mut self, slot: usize) -> ChannelId {
        let gen = self.generations[slot];
        ((gen as u32) << 24) | (slot as u32)
    }

    /// Bump generation for a slot (on close)
    fn bump_generation(&mut self, slot: usize) {
        self.generations[slot] = self.generations[slot].wrapping_add(1);
        if self.generations[slot] == 0 {
            self.generations[slot] = 1; // Skip 0
        }
    }

    /// Close a channel slot and notify peer
    ///
    /// This is the unified cleanup path - called by close(), close_unchecked(),
    /// and cleanup_task(). Assumes slot is valid and contains a channel.
    ///
    /// Returns a wake list containing subscribers from both this channel and peer.
    fn close_slot(&mut self, slot: usize) -> WakeList {
        // Get peer ID before closing
        let peer_id = self.channels[slot].as_ref()
            .and_then(|ch| ch.peer_id());

        // Close this channel and collect its subscribers
        let mut wake_list = if let Some(channel) = self.channels[slot].as_mut() {
            channel.do_close()
        } else {
            WakeList::new()
        };

        // Free the slot
        self.channels[slot] = None;
        self.states[slot] = SlotState::Free;
        self.bump_generation(slot);
        self.active_count -= 1;

        // Notify peer that we've closed
        if let Some(peer) = peer_id {
            if let Some(peer_channel) = self.get_unchecked_mut(peer) {
                let peer_wake = peer_channel.notify_peer_closed();
                wake_list.merge(&peer_wake);
            }
        }

        wake_list
    }

    // ========================================================================
    // Channel Operations
    // ========================================================================

    /// Create a channel pair between two tasks
    ///
    /// This is atomic - either both channels are created or neither.
    pub fn create_pair(&mut self, owner_a: TaskId, owner_b: TaskId) -> Result<(ChannelId, ChannelId), IpcError> {
        // Find two free slots BEFORE modifying anything
        let slot_a = self.find_free_slot().ok_or(IpcError::NoChannelSpace)?;

        // Find second slot, excluding slot_a
        let slot_b = self.states.iter()
            .enumerate()
            .position(|(i, s)| i != slot_a && *s == SlotState::Free)
            .ok_or(IpcError::NoChannelSpace)?;

        // Allocate IDs
        let id_a = self.alloc_id(slot_a);
        let id_b = self.alloc_id(slot_b);

        // Create channels pointing at each other
        self.channels[slot_a] = Some(Channel::new(id_a, owner_a, id_b, owner_b));
        self.channels[slot_b] = Some(Channel::new(id_b, owner_b, id_a, owner_a));
        self.states[slot_a] = SlotState::Active;
        self.states[slot_b] = SlotState::Active;
        self.active_count += 2;

        Ok((id_a, id_b))
    }

    /// Get channel reference (with ownership check)
    pub fn get(&self, id: ChannelId, caller: TaskId) -> Result<&Channel, IpcError> {
        let slot = self.find_slot(id).ok_or(IpcError::InvalidChannel { id })?;

        let channel = self.channels[slot].as_ref()
            .ok_or(IpcError::InvalidChannel { id })?;

        if channel.owner() != caller {
            return Err(IpcError::not_owner(id, channel.owner(), caller));
        }

        Ok(channel)
    }

    /// Get channel mutable reference (with ownership check)
    pub fn get_mut(&mut self, id: ChannelId, caller: TaskId) -> Result<&mut Channel, IpcError> {
        let slot = self.find_slot(id).ok_or(IpcError::InvalidChannel { id })?;

        let channel = self.channels[slot].as_mut()
            .ok_or(IpcError::InvalidChannel { id })?;

        if channel.owner() != caller {
            return Err(IpcError::not_owner(id, channel.owner(), caller));
        }

        Ok(channel)
    }

    /// Get channel without ownership check (for peer operations)
    fn get_unchecked_mut(&mut self, id: ChannelId) -> Option<&mut Channel> {
        let slot = self.find_slot(id)?;
        self.channels[slot].as_mut()
    }

    /// Get channel reference without ownership check (for state queries)
    pub fn get_unchecked(&self, id: ChannelId) -> Option<&Channel> {
        let slot = self.find_slot(id)?;
        self.channels[slot].as_ref()
    }

    /// Send a message on a channel
    ///
    /// Returns subscribers to wake.
    pub fn send(&mut self, id: ChannelId, msg: Message, caller: TaskId) -> Result<WakeList, IpcError> {
        // Validate sender's channel
        let slot = self.find_slot(id).ok_or(IpcError::InvalidChannel { id })?;

        let channel = self.channels[slot].as_ref()
            .ok_or(IpcError::InvalidChannel { id })?;

        if channel.owner() != caller {
            return Err(IpcError::not_owner(id, channel.owner(), caller));
        }

        // Check sender's state
        match channel.state() {
            ChannelState::Open { peer_id, .. } => {
                let peer = *peer_id;
                // Get peer and deliver message
                let peer_channel = self.get_unchecked_mut(peer)
                    .ok_or(IpcError::PeerClosed)?;

                peer_channel.deliver(msg)
            }
            ChannelState::HalfClosed { .. } => Err(IpcError::PeerClosed),
            ChannelState::Closed => Err(IpcError::Closed),
        }
    }

    /// Receive a message from a channel
    pub fn receive(&mut self, id: ChannelId, caller: TaskId) -> Result<Message, IpcError> {
        let channel = self.get_mut(id, caller)?;
        channel.receive()
    }

    /// Close a channel
    ///
    /// Returns subscribers to wake (from both this channel and peer).
    pub fn close(&mut self, id: ChannelId, caller: TaskId) -> Result<WakeList, IpcError> {
        let slot = self.find_slot(id).ok_or(IpcError::InvalidChannel { id })?;

        // Validate ownership
        {
            let channel = self.channels[slot].as_ref()
                .ok_or(IpcError::InvalidChannel { id })?;

            if channel.owner() != caller {
                return Err(IpcError::not_owner(id, channel.owner(), caller));
            }
        }

        // Use unified cleanup path
        Ok(self.close_slot(slot))
    }

    /// Subscribe to events on a channel
    pub fn subscribe(&mut self, id: ChannelId, caller: TaskId, sub: Subscriber, filter: WakeReason) -> Result<(), IpcError> {
        let channel = self.get_mut(id, caller)?;
        channel.subscribe(sub, filter).map_err(|_| IpcError::SubscribersFull)
    }

    /// Unsubscribe from events on a channel
    pub fn unsubscribe(&mut self, id: ChannelId, caller: TaskId, sub: Subscriber) -> Result<(), IpcError> {
        let channel = self.get_mut(id, caller)?;
        channel.unsubscribe(sub);
        Ok(())
    }

    /// Poll channel readiness
    pub fn poll(&self, id: ChannelId, caller: TaskId, filter: WakeReason) -> Result<bool, IpcError> {
        let channel = self.get(id, caller)?;
        Ok(channel.poll(filter))
    }

    /// Poll channel readiness without ownership check (for handle system)
    pub fn poll_unchecked(&self, id: ChannelId, filter: WakeReason) -> bool {
        let slot = match self.find_slot(id) {
            Some(s) => s,
            None => return false,
        };

        match &self.channels[slot] {
            Some(channel) => channel.poll(filter),
            None => false,
        }
    }

    /// Clean up all channels owned by a task
    ///
    /// Returns all subscribers to wake.
    pub fn cleanup_task(&mut self, task_id: TaskId) -> WakeList {
        let mut wake_list = WakeList::new();

        for slot in 0..MAX_CHANNELS {
            if self.states[slot] != SlotState::Active {
                continue;
            }

            let owner = self.channels[slot].as_ref()
                .map(|ch| ch.owner())
                .unwrap_or(0);

            if owner == task_id {
                // Use unified cleanup path
                let slot_wake = self.close_slot(slot);
                wake_list.merge(&slot_wake);
            }
        }

        wake_list
    }

    /// Get queue status for a channel
    pub fn queue_status(&self, id: ChannelId, caller: TaskId) -> Result<super::queue::QueueStatus, IpcError> {
        let channel = self.get(id, caller)?;
        Ok(channel.queue().status())
    }

    /// Check if a channel exists and is open
    pub fn is_open(&self, id: ChannelId) -> bool {
        if let Some(slot) = self.find_slot(id) {
            if let Some(channel) = &self.channels[slot] {
                return channel.state().is_open();
            }
        }
        false
    }

    /// Get owner of a channel (without ownership check)
    pub fn get_owner(&self, id: ChannelId) -> Option<TaskId> {
        let slot = self.find_slot(id)?;
        self.channels[slot].as_ref().map(|ch| ch.owner())
    }

    /// Get peer owner of a channel
    pub fn get_peer_owner(&self, id: ChannelId, caller: TaskId) -> Result<TaskId, IpcError> {
        let channel = self.get(id, caller)?;
        if let Some(peer_id) = channel.peer_id() {
            self.get_owner(peer_id).ok_or(IpcError::PeerClosed)
        } else {
            Err(IpcError::PeerClosed)
        }
    }

    /// Register a waker for a channel (for handle system compatibility)
    pub fn register_waker(&mut self, id: ChannelId, task_id: TaskId) -> Result<(), IpcError> {
        // Find the channel without ownership check (for handle system)
        let slot = self.find_slot(id).ok_or(IpcError::InvalidChannel { id })?;

        let channel = self.channels[slot].as_mut()
            .ok_or(IpcError::InvalidChannel { id })?;

        channel.subscribe(Subscriber::simple(task_id), WakeReason::Readable)
            .map_err(|_| IpcError::SubscribersFull)
    }

    /// Unregister a waker for a channel
    pub fn unregister_waker(&mut self, id: ChannelId, task_id: TaskId) -> Result<(), IpcError> {
        let slot = self.find_slot(id).ok_or(IpcError::InvalidChannel { id })?;

        let channel = self.channels[slot].as_mut()
            .ok_or(IpcError::InvalidChannel { id })?;

        channel.unsubscribe(Subscriber::simple(task_id));
        Ok(())
    }

    // ========================================================================
    // Liveness Support Functions
    // ========================================================================

    /// Send a message directly to a channel (bypass ownership check)
    /// Used by kernel components to send messages on kernel-owned channels.
    /// Like regular send(), this delivers to the PEER channel.
    pub fn send_direct(&mut self, id: ChannelId, msg: Message) -> Result<WakeList, IpcError> {
        let slot = self.find_slot(id).ok_or(IpcError::InvalidChannel { id })?;

        let channel = self.channels[slot].as_ref()
            .ok_or(IpcError::InvalidChannel { id })?;

        // Get peer ID from sender's channel state
        let peer_id = match channel.state() {
            ChannelState::Open { peer_id, .. } => *peer_id,
            ChannelState::HalfClosed { .. } => return Err(IpcError::PeerClosed),
            ChannelState::Closed => return Err(IpcError::Closed),
        };

        // Deliver to peer channel
        let peer_channel = self.get_unchecked_mut(peer_id)
            .ok_or(IpcError::PeerClosed)?;

        peer_channel.deliver(msg)
    }

    /// Close a channel without ownership check (for liveness recovery)
    pub fn close_unchecked(&mut self, id: ChannelId) -> Result<WakeList, IpcError> {
        let slot = self.find_slot(id).ok_or(IpcError::InvalidChannel { id })?;

        // Use unified cleanup path
        Ok(self.close_slot(slot))
    }

    /// Get peer channel ID for a channel
    pub fn get_peer_id(&self, id: ChannelId) -> Option<ChannelId> {
        let slot = self.find_slot(id)?;
        self.channels[slot].as_ref()?.peer_id()
    }

    /// Remove a task from ALL subscriber lists
    ///
    /// Called when a task dies to prevent stale wakes.
    /// This scans all channels and removes the task from their subscriber sets.
    pub fn remove_subscriber_from_all(&mut self, task_id: TaskId) {
        for slot in 0..MAX_CHANNELS {
            if let Some(ref mut channel) = self.channels[slot] {
                channel.unsubscribe_by_task(task_id);
            }
        }
    }
}

impl Default for ChannelTable {
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

    #[test]
    fn test_table_create_pair() {
        let mut table = ChannelTable::new();

        let (id_a, id_b) = table.create_pair(1, 2).unwrap();

        assert_ne!(id_a, id_b);
        assert!(table.is_open(id_a));
        assert!(table.is_open(id_b));
        assert_eq!(table.active_count(), 2);
    }

    #[test]
    fn test_table_ownership_check() {
        let mut table = ChannelTable::new();

        let (id_a, _id_b) = table.create_pair(1, 2).unwrap();

        // Owner can access
        assert!(table.get(id_a, 1).is_ok());

        // Non-owner cannot
        let err = table.get(id_a, 2).unwrap_err();
        match err {
            IpcError::NotOwner { channel, owner, caller } => {
                assert_eq!(channel, id_a);
                assert_eq!(owner, 1);
                assert_eq!(caller, 2);
            }
            _ => panic!("wrong error type"),
        }
    }

    #[test]
    fn test_table_send_receive() {
        let mut table = ChannelTable::new();

        let (id_a, id_b) = table.create_pair(1, 2).unwrap();

        // Task 1 sends on id_a, message goes to id_b's queue
        let msg = Message::data(1, b"hello");
        let wake_list = table.send(id_a, msg, 1).unwrap();
        assert!(wake_list.is_empty()); // No subscribers

        // Task 2 receives on id_b
        let received = table.receive(id_b, 2).unwrap();
        assert_eq!(received.payload_slice(), b"hello");
    }

    #[test]
    fn test_table_close() {
        let mut table = ChannelTable::new();

        let (id_a, id_b) = table.create_pair(1, 2).unwrap();

        // Close id_a
        table.close(id_a, 1).unwrap();

        // id_a is gone
        assert!(!table.is_open(id_a));

        // id_b is half-closed
        let channel_b = table.get(id_b, 2).unwrap();
        assert!(channel_b.state().is_half_closed() || channel_b.state().is_closed());
    }

    #[test]
    fn test_table_cleanup_task() {
        let mut table = ChannelTable::new();

        // Create multiple channels for task 1
        let (id1, _) = table.create_pair(1, 2).unwrap();
        let (id2, _) = table.create_pair(1, 3).unwrap();
        let (_id3, _) = table.create_pair(2, 3).unwrap(); // Not owned by 1

        assert_eq!(table.active_count(), 6);

        // Cleanup task 1
        table.cleanup_task(1);

        // Task 1's channels are gone
        assert!(!table.is_open(id1));
        assert!(!table.is_open(id2));

        // Others still have their channels
        assert_eq!(table.active_count(), 2);
    }

    #[test]
    fn test_table_generation_prevents_stale_access() {
        let mut table = ChannelTable::new();

        let (id_a, _id_b) = table.create_pair(1, 2).unwrap();

        // Close the channel
        table.close(id_a, 1).unwrap();

        // Try to access with old ID - should fail
        assert!(table.get(id_a, 1).is_err());

        // Create new pair - may reuse same slot
        let (new_id, _) = table.create_pair(1, 2).unwrap();

        // Old ID still invalid even if slot is reused
        assert!(table.get(id_a, 1).is_err());

        // New ID works
        assert!(table.get(new_id, 1).is_ok());
    }

    #[test]
    fn test_table_send_to_closed() {
        let mut table = ChannelTable::new();

        let (id_a, id_b) = table.create_pair(1, 2).unwrap();

        // Close id_b (the receiver)
        table.close(id_b, 2).unwrap();

        // Try to send on id_a - peer is closed
        let result = table.send(id_a, Message::data(1, b"test"), 1);
        assert!(matches!(result, Err(IpcError::PeerClosed)));
    }

    #[test]
    fn test_table_subscribe_wake() {
        let mut table = ChannelTable::new();

        let (id_a, id_b) = table.create_pair(1, 2).unwrap();

        // Subscribe task 2 to id_b for Readable
        table.subscribe(id_b, 2, Subscriber::new(2, 1), WakeReason::Readable).unwrap();

        // Send message - should return wake list with task 2
        let wake_list = table.send(id_a, Message::data(1, b"test"), 1).unwrap();
        assert_eq!(wake_list.len(), 1);

        let sub = wake_list.iter().next().unwrap();
        assert_eq!(sub.task_id, 2);
    }
}
