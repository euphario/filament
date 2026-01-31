//! Inter-Process Communication (IPC)
//!
//! Kernel-internal message passing system with channels and ports.
//!
//! # Overview
//!
//! This module provides the low-level IPC primitives used by the kernel.
//! It is NOT directly exposed to userspace - syscalls go through the
//! [`object`](super::object) module which wraps these primitives.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                      Channel Table                              │
//! │  - Global table of all channels (MAX_CHANNELS slots)           │
//! │  - Each channel has peer, state, message queue, subscribers    │
//! └────────────────────────────────────────┬────────────────────────┘
//!                                          │
//!          ┌───────────────────────────────┼───────────────────────┐
//!          │                               │                       │
//!          ▼                               ▼                       ▼
//!    ┌──────────┐                   ┌──────────┐            ┌──────────┐
//!    │ Channel  │ ◀────paired────▶ │ Channel  │            │  Port    │
//!    │  ch_id=1 │                   │  ch_id=2 │            │ "devd:"  │
//!    │ owner=10 │                   │ owner=20 │            │ owner=1  │
//!    └──────────┘                   └──────────┘            └──────────┘
//! ```
//!
//! # Public API
//!
//! ## Types (re-exported)
//! - [`Message`] - Message buffer with header and payload
//! - [`MessageHeader`] - Message type and metadata
//! - [`ChannelId`] - Channel identifier (u32)
//! - [`IpcError`] - Error type for IPC operations
//! - [`Subscriber`] - Task subscription for wake notifications
//! - [`WakeReason`] - Why a subscriber is being woken
//!
//! ## Channel Operations
//! - [`create_channel_pair()`] - Create a connected channel pair
//! - [`send()`] - Send message on a channel
//! - [`receive()`] - Receive message from a channel
//! - [`close_channel()`] - Close a channel (notifies peer)
//! - [`subscribe()`] - Register for wake notifications
//!
//! ## Port Operations
//! - [`port_register()`] - Register a named port
//! - [`port_connect()`] - Connect to a named port (creates channel pair)
//! - [`port_accept()`] - Accept pending connection on a port
//! - [`port_unregister()`] - Unregister a port
//!
//! ## Query Operations
//! - [`channel_has_messages()`] - Check if channel has pending messages
//! - [`channel_is_closed()`] - Check if channel or peer is closed
//! - [`port_has_pending()`] - Check if port has pending connections
//!
//! # Usage Example
//!
//! ```ignore
//! // Create channel pair between two tasks
//! let (ch_a, ch_b) = ipc::create_channel_pair(pid_a, pid_b)?;
//!
//! // Send a message
//! let msg = Message::new(MessageType::Data, b"hello");
//! let wake_list = ipc::send(ch_a, msg, pid_a)?;
//! waker::wake(&wake_list, WakeReason::Readable);
//!
//! // Receive on the other end
//! let received = ipc::receive(ch_b, pid_b)?;
//! ```
//!
//! # Design Principles
//!
//! 1. **State machines** - Channels and ports have explicit states with valid transitions
//! 2. **Subscriber-based waking** - Tasks subscribe to events, collected in WakeList
//! 3. **Wake outside locks** - Collect subscribers under lock, wake after release
//! 4. **IRQ-safe** - All operations use spinlocks, safe from interrupt context
//!
//! # Module Structure
//!
//! - [`types`] - Message, MessageHeader, constants
//! - [`queue`] - MessageQueue ring buffer
//! - [`channel`] - Channel state machine
//! - [`table`] - ChannelTable (internal, allocation/lookup)
//! - [`port`] - Port state machine and registry
//! - [`waker`] - Wake mechanism (WakeList, wake())
//! - [`traits`] - Subscriber, WakeReason, Waitable, Closable
//! - [`error`] - IpcError enum

pub mod types;
pub mod queue;
pub mod error;
pub mod traits;
pub mod waker;
pub mod channel;
pub mod table;
pub mod port;
pub mod backend;

#[cfg(test)]
mod tests;

// Re-export types used by other kernel modules
pub use types::{Message, MessageHeader, MessageType, ChannelId, MAX_INLINE_PAYLOAD};
pub use error::IpcError;
pub use traits::{WakeReason, Waitable, Closable};
pub use table::{PeerInfo, PeerInfoList};
// Note: KernelIpcBackend exists in backend.rs for trait-based testing
// but is not re-exported since ObjectService handles operations directly

// Internal imports (not re-exported)
use table::ChannelTable;
use port::PortRegistry;

use super::lock::SpinLock;

/// Global channel table
static CHANNEL_TABLE: SpinLock<ChannelTable> = SpinLock::new(ChannelTable::new());

/// Global port registry
static PORT_REGISTRY: SpinLock<PortRegistry> = SpinLock::new(PortRegistry::new());

/// Access the global channel table (IRQ-safe)
/// Internal only - use the typed API functions below
fn with_channel_table<F, R>(f: F) -> R
where
    F: FnOnce(&mut ChannelTable) -> R,
{
    let mut guard = CHANNEL_TABLE.lock();
    f(&mut guard)
}

/// Access the global port registry (IRQ-safe)
/// Internal only - use the typed API functions below
fn with_port_registry<F, R>(f: F) -> R
where
    F: FnOnce(&mut PortRegistry) -> R,
{
    let mut guard = PORT_REGISTRY.lock();
    f(&mut guard)
}

/// Access both tables atomically (for operations that span both)
/// Internal only - use port_connect() instead
fn with_both_tables<F, R>(f: F) -> R
where
    F: FnOnce(&mut ChannelTable, &mut PortRegistry) -> R,
{
    // Always lock channel table first to prevent deadlock
    let mut chan_guard = CHANNEL_TABLE.lock();
    let mut port_guard = PORT_REGISTRY.lock();
    f(&mut chan_guard, &mut port_guard)
}

// ============================================================================
// Channel API - Message passing between tasks
// ============================================================================

/// Create a bidirectional channel pair between two tasks.
///
/// Returns `(channel_a, channel_b)` where messages sent on `channel_a`
/// are received on `channel_b` and vice versa.
///
/// # Arguments
/// * `owner_a` - PID of task that will own channel_a
/// * `owner_b` - PID of task that will own channel_b (can be same as owner_a)
///
/// # Errors
/// - `IpcError::NoSlots` - No free channel slots available
pub fn create_channel_pair(owner_a: u32, owner_b: u32) -> Result<(ChannelId, ChannelId), IpcError> {
    with_channel_table(|table| table.create_pair(owner_a, owner_b))
}

/// Send a message on a channel.
///
/// Returns peer info so the caller can wake the peer's ChannelObject
/// via ObjectService after releasing the lock.
///
/// # Arguments
/// * `channel_id` - Channel to send on
/// * `msg` - Message to send
/// * `caller` - PID of sending task (must be owner)
///
/// # Errors
/// - `IpcError::NotFound` - Channel doesn't exist
/// - `IpcError::NotOwner` - Caller doesn't own this channel
/// - `IpcError::PeerClosed` - Peer channel is closed
/// - `IpcError::QueueFull` - Message queue is full
pub fn send(channel_id: ChannelId, msg: Message, caller: u32) -> Result<PeerInfo, IpcError> {
    with_channel_table(|table| table.send(channel_id, msg, caller))
}

/// Receive a message from a channel.
///
/// Blocks conceptually (caller should check/subscribe before calling).
///
/// # Arguments
/// * `channel_id` - Channel to receive from
/// * `caller` - PID of receiving task (must be owner)
///
/// # Errors
/// - `IpcError::NotFound` - Channel doesn't exist
/// - `IpcError::NotOwner` - Caller doesn't own this channel
/// - `IpcError::WouldBlock` - No messages available
/// - `IpcError::PeerClosed` - Peer closed and queue empty
pub fn receive(channel_id: ChannelId, caller: u32) -> Result<Message, IpcError> {
    with_channel_table(|table| table.receive(channel_id, caller))
}

/// Close a channel.
///
/// Notifies peer that this end is closed. Returns peer info for the caller
/// to wake via ObjectService.
///
/// # Arguments
/// * `channel_id` - Channel to close
/// * `caller` - PID of calling task (must be owner)
///
/// # Errors
/// - `IpcError::NotFound` - Channel doesn't exist
/// - `IpcError::NotOwner` - Caller doesn't own this channel
pub fn close_channel(channel_id: ChannelId, caller: u32) -> Result<Option<PeerInfo>, IpcError> {
    with_channel_table(|table| table.close(channel_id, caller))
}

/// Check if channel is ready for an operation (non-blocking poll).
///
/// # Arguments
/// * `filter` - What to check: `Readable` (has messages), `Writable` (queue not full)
///
/// # Returns
/// `Ok(true)` if ready, `Ok(false)` if not ready
pub fn poll(channel_id: ChannelId, caller: u32, filter: WakeReason) -> Result<bool, IpcError> {
    with_channel_table(|table| table.poll(channel_id, caller, filter))
}

/// Clean up all channels owned by a task
pub fn cleanup_task(task_id: u32) -> PeerInfoList {
    with_channel_table(|table| table.cleanup_task(task_id))
}

/// Remove a dying task from ALL port subscriber lists
///
/// Called when a task dies to prevent stale wakes.
/// Channel subscribers are managed by the Object layer's WaitQueue.
pub fn remove_subscriber_from_all(task_id: u32) {
    with_port_registry(|port_reg| {
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
/// Returns (client_channel, subscribers_to_wake, port_owner_id)
pub fn port_connect(name: &[u8], client: u32) -> Result<(ChannelId, waker::WakeList, u32), IpcError> {
    // Check if this is a kernel bus port (/kernel/bus/*)
    // These are special: auto-accept and call bus handler
    const BUS_PREFIX: &[u8] = b"/kernel/bus/";
    if name.starts_with(BUS_PREFIX) && name.len() > BUS_PREFIX.len() {
        return connect_to_bus_port(name, client);
    }

    with_both_tables(|chan_table, port_reg| {
        port_reg.connect(name, client, chan_table)
    })
}

/// Connect to a kernel bus port
/// Bus ports are kernel-owned, so we auto-accept and call the bus handler
fn connect_to_bus_port(name: &[u8], client: u32) -> Result<(ChannelId, waker::WakeList, u32), IpcError> {
    const BUS_PREFIX: &[u8] = b"/kernel/bus/";

    // Get suffix (e.g., "pcie0", "usb0")
    let suffix = &name[BUS_PREFIX.len()..];
    let suffix_str = core::str::from_utf8(suffix).map_err(|_| IpcError::PortNotFound)?;

    // Connect and auto-accept (kernel port has no process to accept)
    let (client_channel, server_channel, port_owner) = with_both_tables(|chan_table, port_reg| {
        port_reg.connect_and_accept(name, client, chan_table)
    })?;

    // Mark client channel for kernel bus dispatch — writes on this channel
    // are synchronously dispatched to the bus controller
    set_kernel_dispatch(client_channel);

    // Call bus handler to send StateSnapshot
    match crate::kernel::bus::handle_port_connect(suffix_str, server_channel, client) {
        Ok(()) => {
            // Success - bus handler sent StateSnapshot
        }
        Err(e) => {
            // Bus handler failed - close channels and return error
            crate::kerror!("ipc", "bus_connect_failed"; suffix = suffix_str);
            let _ = close_channel(client_channel, client);
            let _ = close_unchecked(server_channel);
            return Err(match e {
                crate::kernel::bus::BusError::NotFound => IpcError::PortNotFound,
                crate::kernel::bus::BusError::AlreadyClaimed => IpcError::Busy,
                _ => IpcError::Internal,
            });
        }
    }

    Ok((client_channel, waker::WakeList::new(), port_owner))
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

/// Check if a port exists and is listening
pub fn port_is_listening(port_id: u32) -> bool {
    with_port_registry(|reg| {
        if let Some(port) = reg.get(port_id) {
            port.state().is_listening()
        } else {
            false
        }
    })
}

/// Check if a port exists
pub fn port_exists(port_id: u32) -> bool {
    with_port_registry(|reg| reg.get(port_id).is_some())
}

/// Register a waker for port Accepted events (when channel is a listen channel)
///
/// Returns true on success, false if port not found or subscriber set full.
pub fn port_register_waker_for_channel(channel_id: ChannelId, task_id: u32) -> bool {
    with_port_registry(|reg| {
        if let Some(port) = reg.get_by_listen_channel_mut(channel_id) {
            let sub = traits::Subscriber { task_id, generation: 0 };
            // SECURITY FIX: Don't silently drop - return error if full
            port.subscribe(sub, traits::WakeReason::Accepted).is_ok()
        } else {
            false
        }
    })
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
pub fn send_direct(channel_id: ChannelId, msg: Message) -> Result<PeerInfo, IpcError> {
    with_channel_table(|table| table.send_direct(channel_id, msg))
}

/// Close a channel without ownership check (for liveness recovery)
pub fn close_unchecked(channel_id: ChannelId) -> Result<Option<PeerInfo>, IpcError> {
    with_channel_table(|table| table.close_unchecked(channel_id))
}

/// Get peer channel ID
pub fn get_peer_id(channel_id: ChannelId) -> Option<ChannelId> {
    with_channel_table(|table| table.get_peer_id(channel_id))
}

/// Mark a channel for kernel bus dispatch
///
/// Writes on this channel will be dispatched synchronously to the
/// kernel bus controller instead of just being queued.
pub fn set_kernel_dispatch(channel_id: ChannelId) {
    with_channel_table(|table| table.set_kernel_dispatch(channel_id))
}

/// Check if a channel has kernel bus dispatch enabled
pub fn is_kernel_dispatch(channel_id: ChannelId) -> bool {
    with_channel_table(|table| table.is_kernel_dispatch(channel_id))
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
        Ok(peer_opt) => {
            if let Some(peer) = peer_opt {
                let wake_list = crate::kernel::object_service::object_service()
                    .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::CLOSED);
                waker::wake(&wake_list, WakeReason::Closed);
            }
            0
        }
        Err(e) => e.to_errno(),
    }
}

/// Clean up all IPC resources for a process
pub fn process_cleanup(pid: u32) -> PeerInfoList {
    cleanup_task(pid)
}

/// Send data on a channel (old API compatibility)
/// This is the kernel-internal API used by handle system
pub fn sys_send(channel_id: ChannelId, caller: u32, data: &[u8]) -> Result<(), IpcError> {
    let msg = Message::data(caller, data);
    let peer = send(channel_id, msg, caller)?;
    let wake_list = crate::kernel::object_service::object_service()
        .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::READABLE);
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
pub fn send_unchecked(channel_id: ChannelId, msg: Message) -> Result<PeerInfo, IpcError> {
    with_channel_table(|table| table.send_direct(channel_id, msg))
}
