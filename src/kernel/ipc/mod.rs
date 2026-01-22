//! IPC2 - Rewritten Inter-Process Communication
//!
//! This module provides a clean, state-machine-driven IPC system with:
//! - Explicit state machines for channels and ports
//! - Single, unified wake mechanism via subscribers
//! - Trait-based contracts (Waitable, Closable)
//! - Comprehensive unit tests
//!
//! # Module Structure
//!
//! ```text
//! ipc/
//! ├── mod.rs      - This file: re-exports, global table access
//! ├── types.rs    - Message, MessageHeader, constants
//! ├── queue.rs    - MessageQueue with ring buffer
//! ├── channel.rs  - Channel state machine
//! ├── table.rs    - ChannelTable (allocation, lookup)
//! ├── port.rs     - Port state machine + registry
//! ├── waker.rs    - Unified wake mechanism
//! ├── traits.rs   - Waitable, Closable, Subscriber
//! └── error.rs    - Typed errors
//! ```
//!
//! # Design Principles
//!
//! 1. **Single source of truth**: Each object has ONE state enum
//! 2. **Explicit transitions**: State changes only via defined methods
//! 3. **Subscriber-based waking**: Tasks subscribe to events, get woken
//! 4. **No implicit state**: No `blocked_receiver` + `peer_closed` + `state` mess
//! 5. **Testable**: Core logic separated from syscall glue

pub mod types;
pub mod queue;
pub mod error;
pub mod traits;
pub mod waker;
pub mod channel;
pub mod table;
pub mod port;

#[cfg(test)]
mod tests;

// Re-export commonly used types
pub use types::{Message, MessageHeader, MessageType, ChannelId, MAX_INLINE_PAYLOAD, MAX_QUEUE_SIZE, MAX_PORT_NAME};
pub use queue::MessageQueue;
pub use error::IpcError;
pub use traits::{Subscriber, WakeReason, Waitable, Closable, CloseAction};
pub use waker::SubscriberSet;
pub use channel::{Channel, ChannelState};
pub use table::ChannelTable;
pub use port::{Port, PortState, PortRegistry, PendingConnection};

use super::lock::SpinLock;

/// Global channel table
static CHANNEL_TABLE: SpinLock<ChannelTable> = SpinLock::new(ChannelTable::new());

/// Global port registry
static PORT_REGISTRY: SpinLock<PortRegistry> = SpinLock::new(PortRegistry::new());

/// Access the global channel table (IRQ-safe)
///
/// # Example
/// ```ignore
/// let subs = with_channel_table(|table| {
///     table.send(channel_id, msg, caller_pid)
/// })?;
/// waker::wake(&subs, WakeReason::Readable);
/// ```
pub fn with_channel_table<F, R>(f: F) -> R
where
    F: FnOnce(&mut ChannelTable) -> R,
{
    let mut guard = CHANNEL_TABLE.lock();
    f(&mut guard)
}

/// Access the global port registry (IRQ-safe)
pub fn with_port_registry<F, R>(f: F) -> R
where
    F: FnOnce(&mut PortRegistry) -> R,
{
    let mut guard = PORT_REGISTRY.lock();
    f(&mut guard)
}

/// Access both tables atomically (for operations that span both)
///
/// This is needed for operations like port_connect which:
/// 1. Looks up port in registry
/// 2. Creates channel pair in channel table
/// 3. Adds pending connection to port
pub fn with_both_tables<F, R>(f: F) -> R
where
    F: FnOnce(&mut ChannelTable, &mut PortRegistry) -> R,
{
    // Always lock channel table first to prevent deadlock
    let mut chan_guard = CHANNEL_TABLE.lock();
    let mut port_guard = PORT_REGISTRY.lock();
    f(&mut chan_guard, &mut port_guard)
}

// ============================================================================
// Convenience functions for common operations
// ============================================================================

/// Create a channel pair between two tasks
pub fn create_channel_pair(owner_a: u32, owner_b: u32) -> Result<(ChannelId, ChannelId), IpcError> {
    with_channel_table(|table| table.create_pair(owner_a, owner_b))
}

/// Send a message on a channel
/// Returns subscribers to wake (caller must wake outside lock)
pub fn send(channel_id: ChannelId, msg: Message, caller: u32) -> Result<waker::WakeList, IpcError> {
    with_channel_table(|table| table.send(channel_id, msg, caller))
}

/// Receive a message from a channel
pub fn receive(channel_id: ChannelId, caller: u32) -> Result<Message, IpcError> {
    with_channel_table(|table| table.receive(channel_id, caller))
}

/// Close a channel
/// Returns subscribers to wake
pub fn close_channel(channel_id: ChannelId, caller: u32) -> Result<waker::WakeList, IpcError> {
    with_channel_table(|table| table.close(channel_id, caller))
}

/// Subscribe to events on a channel
pub fn subscribe(channel_id: ChannelId, caller: u32, sub: Subscriber, filter: WakeReason) -> Result<(), IpcError> {
    with_channel_table(|table| table.subscribe(channel_id, caller, sub, filter))
}

/// Unsubscribe from events on a channel
pub fn unsubscribe(channel_id: ChannelId, caller: u32, sub: Subscriber) -> Result<(), IpcError> {
    with_channel_table(|table| table.unsubscribe(channel_id, caller, sub))
}

/// Check if channel is ready for an operation
pub fn poll(channel_id: ChannelId, caller: u32, filter: WakeReason) -> Result<bool, IpcError> {
    with_channel_table(|table| table.poll(channel_id, caller, filter))
}

/// Clean up all channels owned by a task
pub fn cleanup_task(task_id: u32) -> waker::WakeList {
    with_channel_table(|table| table.cleanup_task(task_id))
}

/// Remove a dying task from ALL subscriber lists
///
/// Called when a task dies to prevent stale wakes.
/// Scans all channels and ports to remove the task from their subscriber sets.
pub fn remove_subscriber_from_all(task_id: u32) {
    with_both_tables(|chan_table, port_reg| {
        chan_table.remove_subscriber_from_all(task_id);
        port_reg.remove_subscriber_from_all(task_id);
    })
}

// ============================================================================
// Port operations
// ============================================================================

/// Register a new port
pub fn port_register(name: &[u8], owner: u32) -> Result<(u32, ChannelId), IpcError> {
    with_both_tables(|chan_table, port_reg| {
        port_reg.register(name, owner, chan_table)
    })
}

/// Connect to a port
/// Returns (client_channel, subscribers_to_wake)
pub fn port_connect(name: &[u8], client: u32) -> Result<(ChannelId, waker::WakeList), IpcError> {
    with_both_tables(|chan_table, port_reg| {
        port_reg.connect(name, client, chan_table)
    })
}

/// Accept a pending connection on a port
pub fn port_accept(port_id: u32, owner: u32) -> Result<(ChannelId, u32), IpcError> {
    with_port_registry(|reg| reg.accept(port_id, owner))
}

/// Unregister a port
pub fn port_unregister(port_id: u32, owner: u32) -> Result<waker::WakeList, IpcError> {
    with_port_registry(|reg| reg.unregister(port_id, owner))
}

/// Accept using listen channel (legacy API compatibility)
pub fn port_accept_by_channel(listen_channel: ChannelId, owner: u32) -> Result<(ChannelId, u32), IpcError> {
    with_port_registry(|reg| reg.accept_by_channel(listen_channel, owner))
}

/// Unregister a port by name (legacy API compatibility)
pub fn port_unregister_by_name(name: &[u8], owner: u32) -> Result<waker::WakeList, IpcError> {
    with_port_registry(|reg| reg.unregister_by_name(name, owner))
}

/// Clean up all ports owned by a task
pub fn port_cleanup_task(task_id: u32) -> waker::WakeList {
    with_port_registry(|reg| reg.cleanup_task(task_id))
}

/// Unregister a port by its listen channel ID
pub fn port_unregister_by_channel(channel_id: ChannelId) {
    with_port_registry(|reg| reg.unregister_by_channel(channel_id))
}

/// Register port from buffer (old API compatibility)
/// Returns listen channel on success
pub fn sys_port_register_buf(name: &[u8], owner: u32) -> Result<ChannelId, IpcError> {
    let (_port_id, listen_channel) = port_register(name, owner)?;
    Ok(listen_channel)
}

/// Accept connection on port (old API compatibility)
pub fn sys_port_accept(listen_channel: ChannelId, owner: u32) -> Result<(ChannelId, u32), IpcError> {
    port_accept_by_channel(listen_channel, owner)
}

/// Check if port has pending connections
pub fn has_pending_accept(listen_channel: ChannelId) -> bool {
    with_port_registry(|reg| reg.has_pending_accept(listen_channel))
}

/// Get port owner and listen channel by name
pub fn get_port_owner_by_name(name: &[u8]) -> Option<(u32, ChannelId)> {
    with_port_registry(|reg| reg.get_owner_by_name(name))
}

/// Get peer owner of a channel
pub fn get_peer_owner(channel_id: ChannelId, caller: u32) -> Result<u32, IpcError> {
    with_channel_table(|table| table.get_peer_owner(channel_id, caller))
}

// ============================================================================
// Handle system compatibility functions
// ============================================================================

/// Check if channel has messages available
pub fn channel_has_messages(channel_id: ChannelId) -> bool {
    with_channel_table(|table| table.poll_unchecked(channel_id, WakeReason::Readable))
}

/// Check if channel's peer is closed
pub fn channel_is_closed(channel_id: ChannelId) -> bool {
    with_channel_table(|table| {
        // Closed if not open OR if peer closed (check Closed filter)
        !table.is_open(channel_id) || table.poll_unchecked(channel_id, WakeReason::Closed)
    })
}

/// Check if a channel exists and is open
pub fn channel_exists(channel_id: ChannelId) -> bool {
    with_channel_table(|table| table.is_open(channel_id))
}

/// Check if channel can send (not full)
pub fn channel_can_send(channel_id: ChannelId) -> bool {
    with_channel_table(|table| table.poll_unchecked(channel_id, WakeReason::Writable))
}

/// Check if channel queue is full (cannot send)
pub fn channel_queue_full(channel_id: ChannelId) -> bool {
    !channel_can_send(channel_id)
}

/// Check if a port has pending connections
pub fn port_has_pending(port_id: u32) -> bool {
    with_port_registry(|reg| {
        if let Some(port) = reg.get(port_id) {
            port.state().pending_count() > 0
        } else {
            false
        }
    })
}

/// Register a waker for a channel (handle system compatibility)
pub fn channel_register_waker(channel_id: ChannelId, task_id: u32) {
    let _ = with_channel_table(|table| table.register_waker(channel_id, task_id));
}

/// Unregister a waker for a channel
pub fn channel_unregister_waker(channel_id: ChannelId, task_id: u32) {
    let _ = with_channel_table(|table| table.unregister_waker(channel_id, task_id));
}

/// Register a waker for port Accepted events (when channel is a listen channel)
pub fn port_register_waker_for_channel(channel_id: ChannelId, task_id: u32) {
    with_port_registry(|reg| {
        if let Some(port) = reg.get_by_listen_channel_mut(channel_id) {
            let sub = traits::Subscriber { task_id, generation: 0 };
            port.subscribe(sub, traits::WakeReason::Accepted);
        }
    });
}

/// Unregister a waker for port events
pub fn port_unregister_waker_for_channel(channel_id: ChannelId, task_id: u32) {
    with_port_registry(|reg| {
        if let Some(port) = reg.get_by_listen_channel_mut(channel_id) {
            let sub = traits::Subscriber { task_id, generation: 0 };
            port.unsubscribe(sub);
        }
    });
}

// ============================================================================
// Liveness support functions
// ============================================================================

/// Send a message directly to a channel (bypass ownership check)
/// Used by liveness checker to send ping messages
pub fn send_direct(channel_id: ChannelId, msg: Message) -> Result<waker::WakeList, IpcError> {
    with_channel_table(|table| table.send_direct(channel_id, msg))
}

/// Close a channel without ownership check (for liveness recovery)
pub fn close_unchecked(channel_id: ChannelId) -> Result<waker::WakeList, IpcError> {
    with_channel_table(|table| table.close_unchecked(channel_id))
}

/// Get peer channel ID
pub fn get_peer_id(channel_id: ChannelId) -> Option<ChannelId> {
    with_channel_table(|table| table.get_peer_id(channel_id))
}

// ============================================================================
// Syscall-style wrappers (for backward compatibility)
// ============================================================================

/// Create a channel pair (syscall-style wrapper)
/// Returns (channel_a, channel_b) wrapped in Result for old API compatibility
pub fn sys_channel_create(owner: u32) -> Result<(ChannelId, ChannelId), IpcError> {
    create_channel_pair(owner, owner)
}

/// Close a channel (syscall-style wrapper)
/// Returns 0 on success, negative error on failure
pub fn sys_channel_close(channel_id: ChannelId, caller: u32) -> i64 {
    match close_channel(channel_id, caller) {
        Ok(wake_list) => {
            waker::wake(&wake_list, WakeReason::Closed);
            0
        }
        Err(e) => e.to_errno(),
    }
}

/// Clean up all IPC resources for a process
pub fn process_cleanup(pid: u32) -> waker::WakeList {
    cleanup_task(pid)
}

/// Send data on a channel (old API compatibility)
/// This is the kernel-internal API used by handle system
pub fn sys_send(channel_id: ChannelId, caller: u32, data: &[u8]) -> Result<(), IpcError> {
    let msg = Message::data(caller, data);
    let wake_list = send(channel_id, msg, caller)?;
    waker::wake(&wake_list, WakeReason::Readable);
    Ok(())
}

/// Receive message from a channel (old API compatibility)
/// Returns the message directly
pub fn sys_receive(channel_id: ChannelId, caller: u32) -> Result<Message, IpcError> {
    receive(channel_id, caller)
}

/// Receive blocking (old API compatibility)
/// This is the same as sys_receive - actual blocking is handled by syscall layer
pub fn sys_receive_blocking(channel_id: ChannelId, caller: u32) -> Result<Message, IpcError> {
    receive(channel_id, caller)
}

/// Get peer owner for a channel (for scheme.rs compatibility)
/// Does NOT require ownership - used for looking up daemon channels
pub fn get_peer_owner_unchecked(channel_id: ChannelId) -> Option<u32> {
    with_channel_table(|table| {
        if let Some(peer_id) = table.get_peer_id(channel_id) {
            table.get_owner(peer_id)
        } else {
            None
        }
    })
}

/// Send to a channel without ownership check (for scheme.rs connecting to daemon)
pub fn send_unchecked(channel_id: ChannelId, msg: Message) -> Result<waker::WakeList, IpcError> {
    with_channel_table(|table| table.send_direct(channel_id, msg))
}
