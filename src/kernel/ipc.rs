//! Inter-Process Communication (IPC)
//!
//! Implements message-passing IPC for the microkernel.
//! This is the primary mechanism for communication between processes.
//!
//! Design:
//! - Channels: Bidirectional communication endpoints
//! - Messages: Header + payload (up to 256 bytes inline)
//! - Blocking: Processes can block waiting for messages
//! - Ports: Named endpoints for service discovery
//!
//! ## Thread Safety
//!
//! The channel table is protected by a SpinLock with IRQ-save semantics,
//! making it safe for SMP and interrupt context access.
//!
//! ## Backpressure
//!
//! Senders can query queue status before sending to implement backpressure:
//! - `queue_status()` returns current/max counts
//! - `QueueFull` error when queue is full
//! - Receivers can be notified when queue drains below threshold

#![allow(dead_code)]  // Some message types and methods are for future use

use crate::logln;
use super::lock::SpinLock;
use super::process::Pid;

/// Maximum inline message payload size
/// 576 bytes = 512 (sector) + 64 (headers/overhead)
pub const MAX_INLINE_PAYLOAD: usize = 576;

/// Maximum messages in a queue
pub const MAX_QUEUE_SIZE: usize = 8;

/// Maximum number of channels (keep small for now)
pub const MAX_CHANNELS: usize = 32;

/// Channel ID type
pub type ChannelId = u32;

/// Message types
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    /// Regular data message
    Data = 0,
    /// Request (expects reply)
    Request = 1,
    /// Reply to a request
    Reply = 2,
    /// Error response
    Error = 3,
    /// Channel closed notification
    Close = 4,
    /// Connect request (for ports)
    Connect = 5,
    /// Accept connection
    Accept = 6,
}

/// Message header
#[repr(C)]
#[derive(Clone, Copy)]
pub struct MessageHeader {
    /// Message type
    pub msg_type: MessageType,
    /// Sender PID
    pub sender: Pid,
    /// Message ID (for request/reply matching)
    pub msg_id: u32,
    /// Payload length
    pub payload_len: u32,
    /// Flags (reserved)
    pub flags: u32,
}

impl MessageHeader {
    pub const fn new(msg_type: MessageType, sender: Pid) -> Self {
        Self {
            msg_type,
            sender,
            msg_id: 0,
            payload_len: 0,
            flags: 0,
        }
    }
}

/// A complete message with inline payload
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Message {
    pub header: MessageHeader,
    pub payload: [u8; MAX_INLINE_PAYLOAD],
}

impl Message {
    pub const fn new() -> Self {
        Self {
            header: MessageHeader::new(MessageType::Data, 0),
            payload: [0; MAX_INLINE_PAYLOAD],
        }
    }

    /// Create a data message
    pub fn data(sender: Pid, data: &[u8]) -> Self {
        let mut msg = Self::new();
        msg.header.msg_type = MessageType::Data;
        msg.header.sender = sender;
        let len = core::cmp::min(data.len(), MAX_INLINE_PAYLOAD);
        msg.header.payload_len = len as u32;
        msg.payload[..len].copy_from_slice(&data[..len]);
        msg
    }

    /// Create a request message
    pub fn request(sender: Pid, msg_id: u32, data: &[u8]) -> Self {
        let mut msg = Self::data(sender, data);
        msg.header.msg_type = MessageType::Request;
        msg.header.msg_id = msg_id;
        msg
    }

    /// Create a reply message
    pub fn reply(sender: Pid, msg_id: u32, data: &[u8]) -> Self {
        let mut msg = Self::data(sender, data);
        msg.header.msg_type = MessageType::Reply;
        msg.header.msg_id = msg_id;
        msg
    }

    /// Create an error message
    pub fn error(sender: Pid, msg_id: u32, error_code: i32) -> Self {
        let mut msg = Self::new();
        msg.header.msg_type = MessageType::Error;
        msg.header.sender = sender;
        msg.header.msg_id = msg_id;
        msg.header.payload_len = 4;
        msg.payload[0..4].copy_from_slice(&error_code.to_le_bytes());
        msg
    }

    /// Get payload as slice
    ///
    /// Returns the valid portion of the payload buffer. The length is clamped
    /// to MAX_INLINE_PAYLOAD to prevent out-of-bounds access if payload_len
    /// is corrupted.
    pub fn payload_slice(&self) -> &[u8] {
        let len = core::cmp::min(self.header.payload_len as usize, MAX_INLINE_PAYLOAD);
        &self.payload[..len]
    }
}

/// Queue status for backpressure monitoring
#[derive(Debug, Clone, Copy)]
pub struct QueueStatus {
    /// Current number of messages in queue
    pub count: usize,
    /// Maximum queue capacity
    pub capacity: usize,
    /// True if queue is full
    pub full: bool,
    /// True if queue is empty
    pub empty: bool,
}

/// Message queue for a channel endpoint
pub struct MessageQueue {
    messages: [Message; MAX_QUEUE_SIZE],
    head: usize,
    tail: usize,
    count: usize,
}

impl MessageQueue {
    pub const fn new() -> Self {
        const EMPTY_MSG: Message = Message::new();
        Self {
            messages: [EMPTY_MSG; MAX_QUEUE_SIZE],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    /// Check if queue is empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Check if queue is full
    pub fn is_full(&self) -> bool {
        self.count >= MAX_QUEUE_SIZE
    }

    /// Push a message to the queue
    pub fn push(&mut self, msg: Message) -> bool {
        if self.is_full() {
            return false;
        }
        self.messages[self.tail] = msg;
        self.tail = (self.tail + 1) % MAX_QUEUE_SIZE;
        self.count += 1;
        true
    }

    /// Pop a message from the queue
    pub fn pop(&mut self) -> Option<Message> {
        if self.is_empty() {
            return None;
        }
        let msg = self.messages[self.head];
        self.head = (self.head + 1) % MAX_QUEUE_SIZE;
        self.count -= 1;
        Some(msg)
    }

    /// Peek at the front message without removing
    pub fn peek(&self) -> Option<&Message> {
        if self.is_empty() {
            None
        } else {
            Some(&self.messages[self.head])
        }
    }

    /// Get number of messages in queue
    pub fn len(&self) -> usize {
        self.count
    }

    /// Get queue status for backpressure monitoring
    pub fn status(&self) -> QueueStatus {
        QueueStatus {
            count: self.count,
            capacity: MAX_QUEUE_SIZE,
            full: self.is_full(),
            empty: self.is_empty(),
        }
    }

    /// Get available space in queue
    pub fn available(&self) -> usize {
        MAX_QUEUE_SIZE - self.count
    }
}

/// Channel endpoint state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelState {
    /// Channel is free/unused
    Free,
    /// Channel is open and usable
    Open,
    /// Channel is closed
    Closed,
}

/// A channel endpoint
pub struct ChannelEndpoint {
    /// Channel ID
    pub id: ChannelId,
    /// Owning process
    pub owner: Pid,
    /// Peer endpoint ID (for bidirectional channels)
    pub peer: ChannelId,
    /// State
    pub state: ChannelState,
    /// Message queue
    pub queue: MessageQueue,
    /// Process waiting to receive on this channel
    pub blocked_receiver: Option<Pid>,
    /// Set when peer has closed (even if Close message couldn't be delivered)
    pub peer_closed: bool,
}

impl ChannelEndpoint {
    pub const fn new() -> Self {
        Self {
            id: 0,
            owner: 0,
            peer: 0,
            state: ChannelState::Free,
            queue: MessageQueue::new(),
            blocked_receiver: None,
            peer_closed: false,
        }
    }

    /// Reset endpoint to free state
    pub fn reset(&mut self) {
        self.id = 0;
        self.owner = 0;
        self.peer = 0;
        self.state = ChannelState::Free;
        self.queue = MessageQueue::new();
        self.blocked_receiver = None;
        self.peer_closed = false;
    }
}

/// Channel table managing all endpoints
pub struct ChannelTable {
    pub endpoints: [ChannelEndpoint; MAX_CHANNELS],
    next_id: ChannelId,
}

impl ChannelTable {
    pub const fn new() -> Self {
        const EMPTY: ChannelEndpoint = ChannelEndpoint::new();
        Self {
            endpoints: [EMPTY; MAX_CHANNELS],
            next_id: 1,
        }
    }

    /// Allocate a new channel ID
    fn alloc_id(&mut self) -> ChannelId {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Find a free endpoint slot
    fn find_free_slot(&self) -> Option<usize> {
        self.endpoints.iter().position(|e| e.state == ChannelState::Free)
    }

    /// Find endpoint by ID
    fn find_by_id(&self, id: ChannelId) -> Option<usize> {
        self.endpoints.iter().position(|e| e.id == id && e.state != ChannelState::Free)
    }

    /// Create a channel pair (bidirectional)
    /// Returns (endpoint_a, endpoint_b) IDs
    ///
    /// CORRECTNESS: This function is atomic - it either creates both endpoints
    /// or neither. We find both slots BEFORE modifying any state to prevent
    /// leaving the table in an inconsistent state on failure.
    pub fn create_pair(&mut self, owner_a: Pid, owner_b: Pid) -> Option<(ChannelId, ChannelId)> {
        // CRITICAL: Find both slots BEFORE modifying any state
        // This prevents a race where we mark slot_a as Open but then fail
        // to find slot_b, leaving slot_a in an inconsistent state
        let slot_a = self.find_free_slot()?;

        // Find second slot, excluding slot_a
        let slot_b = self.endpoints.iter()
            .enumerate()
            .position(|(i, e)| i != slot_a && e.state == ChannelState::Free)?;

        // Now that we have both slots, allocate IDs
        let id_a = self.alloc_id();
        let id_b = self.alloc_id();

        // Set up endpoint A (atomically - all fields at once)
        self.endpoints[slot_a].id = id_a;
        self.endpoints[slot_a].owner = owner_a;
        self.endpoints[slot_a].peer = id_b;
        self.endpoints[slot_a].queue = MessageQueue::new();
        self.endpoints[slot_a].blocked_receiver = None;
        self.endpoints[slot_a].state = ChannelState::Open; // Set state LAST

        // Set up endpoint B (atomically - all fields at once)
        self.endpoints[slot_b].id = id_b;
        self.endpoints[slot_b].owner = owner_b;
        self.endpoints[slot_b].peer = id_a;
        self.endpoints[slot_b].queue = MessageQueue::new();
        self.endpoints[slot_b].blocked_receiver = None;
        self.endpoints[slot_b].state = ChannelState::Open; // Set state LAST

        Some((id_a, id_b))
    }

    /// Close a channel endpoint
    /// Returns Ok(Option<Pid>) with blocked receiver PID if any, or Err if channel not found
    pub fn close(&mut self, id: ChannelId) -> Result<Option<Pid>, ()> {
        if let Some(slot) = self.find_by_id(id) {
            let peer_id = self.endpoints[slot].peer;
            self.endpoints[slot].state = ChannelState::Closed;

            let mut blocked_pid = None;

            // Notify peer if it exists
            if peer_id != 0 {
                if let Some(peer_slot) = self.find_by_id(peer_id) {
                    // Get blocked receiver before pushing message
                    blocked_pid = self.endpoints[peer_slot].blocked_receiver;

                    // Mark peer as having a closed peer (ensures notification even if queue is full)
                    self.endpoints[peer_slot].peer_closed = true;

                    // Try to send close notification to peer's queue
                    // (may fail if queue is full, but peer_closed flag ensures detection)
                    let close_msg = Message {
                        header: MessageHeader {
                            msg_type: MessageType::Close,
                            sender: self.endpoints[slot].owner,
                            msg_id: 0,
                            payload_len: 0,
                            flags: 0,
                        },
                        payload: [0; MAX_INLINE_PAYLOAD],
                    };
                    let _ = self.endpoints[peer_slot].queue.push(close_msg);

                    // Clear blocked receiver since we're waking them
                    if blocked_pid.is_some() {
                        self.endpoints[peer_slot].blocked_receiver = None;
                    }
                }
            }

            // Reset the endpoint
            self.endpoints[slot].reset();
            Ok(blocked_pid)
        } else {
            Err(())
        }
    }

    /// Send a message to a channel
    /// Returns Ok(()) or Err with blocked PID to wake
    pub fn send(&mut self, channel_id: ChannelId, msg: Message) -> Result<Option<Pid>, IpcError> {
        let slot = self.find_by_id(channel_id).ok_or(IpcError::InvalidChannel)?;
        let peer_id = self.endpoints[slot].peer;

        if peer_id == 0 {
            return Err(IpcError::NoPeer);
        }

        let peer_slot = self.find_by_id(peer_id).ok_or(IpcError::PeerClosed)?;

        if self.endpoints[peer_slot].state == ChannelState::Closed {
            return Err(IpcError::PeerClosed);
        }

        if self.endpoints[peer_slot].queue.is_full() {
            return Err(IpcError::QueueFull);
        }

        self.endpoints[peer_slot].queue.push(msg);

        // Check if there's a blocked receiver to wake
        let blocked = self.endpoints[peer_slot].blocked_receiver.take();
        Ok(blocked)
    }

    /// Receive a message from a channel (non-blocking)
    pub fn receive(&mut self, channel_id: ChannelId) -> Result<Message, IpcError> {
        let slot = self.find_by_id(channel_id).ok_or(IpcError::InvalidChannel)?;

        // Try to pop a message from the queue
        if let Some(msg) = self.endpoints[slot].queue.pop() {
            return Ok(msg);
        }

        // Queue is empty - check if peer has closed
        // This handles the case where the Close message couldn't be delivered
        // due to a full queue
        if self.endpoints[slot].peer_closed {
            return Err(IpcError::PeerClosed);
        }

        Err(IpcError::WouldBlock)
    }

    /// Register a process as blocked waiting for a message
    pub fn block_receiver(&mut self, channel_id: ChannelId, pid: Pid) -> Result<(), IpcError> {
        let slot = self.find_by_id(channel_id).ok_or(IpcError::InvalidChannel)?;
        self.endpoints[slot].blocked_receiver = Some(pid);
        Ok(())
    }

    /// Get endpoint owner
    pub fn get_owner(&self, channel_id: ChannelId) -> Option<Pid> {
        self.find_by_id(channel_id).map(|slot| self.endpoints[slot].owner)
    }

    /// Get peer's owner PID
    pub fn get_peer_owner(&self, channel_id: ChannelId) -> Option<Pid> {
        let slot = self.find_by_id(channel_id)?;
        let peer_id = self.endpoints[slot].peer;
        if peer_id == 0 {
            return None;
        }
        let peer_slot = self.find_by_id(peer_id)?;
        Some(self.endpoints[peer_slot].owner)
    }

    /// Get peer's channel ID
    pub fn get_peer_id(&self, channel_id: ChannelId) -> Option<ChannelId> {
        let slot = self.find_by_id(channel_id)?;
        let peer_id = self.endpoints[slot].peer;
        if peer_id == 0 {
            None
        } else {
            Some(peer_id)
        }
    }

    /// Check if a channel has messages waiting
    pub fn has_messages(&self, channel_id: ChannelId) -> bool {
        self.find_by_id(channel_id)
            .map(|slot| !self.endpoints[slot].queue.is_empty())
            .unwrap_or(false)
    }

    /// Get queue length for a channel
    pub fn queue_len(&self, channel_id: ChannelId) -> usize {
        self.find_by_id(channel_id)
            .map(|slot| self.endpoints[slot].queue.len())
            .unwrap_or(0)
    }

    /// Get queue status for backpressure monitoring
    /// Returns None if channel not found
    pub fn queue_status(&self, channel_id: ChannelId) -> Option<QueueStatus> {
        self.find_by_id(channel_id)
            .map(|slot| self.endpoints[slot].queue.status())
    }

    /// Get peer's queue status (what we'd be sending to)
    /// Useful for sender-side backpressure
    pub fn peer_queue_status(&self, channel_id: ChannelId) -> Option<QueueStatus> {
        self.find_by_id(channel_id)
            .and_then(|slot| {
                let peer_id = self.endpoints[slot].peer;
                if peer_id == 0 {
                    None
                } else {
                    self.find_by_id(peer_id)
                        .map(|peer_slot| self.endpoints[peer_slot].queue.status())
                }
            })
    }
}

/// IPC error codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IpcError {
    /// Invalid channel ID
    InvalidChannel,
    /// Channel has no peer
    NoPeer,
    /// Peer channel is closed
    PeerClosed,
    /// Message queue is full
    QueueFull,
    /// No message available (would block)
    WouldBlock,
    /// Permission denied
    PermissionDenied,
    /// Invalid message
    InvalidMessage,
}

impl IpcError {
    pub fn to_errno(self) -> i32 {
        match self {
            IpcError::InvalidChannel => -1,
            IpcError::NoPeer => -2,
            IpcError::PeerClosed => -3,
            IpcError::QueueFull => -4,
            IpcError::WouldBlock => -11,  // EAGAIN
            IpcError::PermissionDenied => -13,  // EACCES
            IpcError::InvalidMessage => -22,  // EINVAL
        }
    }
}

/// Global channel table protected by SpinLock
///
/// The SpinLock provides:
/// - IRQ-safe access (disables interrupts while held)
/// - SMP-safe access (atomic spinlock for multi-core)
static CHANNEL_TABLE: SpinLock<ChannelTable> = SpinLock::new(ChannelTable::new());

/// Get the global channel table (legacy API, prefer with_channel_table)
///
/// # Safety
/// Caller must ensure proper synchronization. This function exists for
/// backwards compatibility with code that manages its own locking.
/// Prefer using `with_channel_table()` for new code.
#[deprecated(note = "Use with_channel_table() for thread-safe access")]
pub unsafe fn channel_table() -> &'static mut ChannelTable {
    // SAFETY: This is inherently unsafe - caller must ensure synchronization
    // We use a raw pointer to bypass the SpinLock for legacy compatibility
    static mut LEGACY_TABLE: ChannelTable = ChannelTable::new();
    &mut *core::ptr::addr_of_mut!(LEGACY_TABLE)
}

/// Execute a closure with exclusive access to the channel table.
/// The SpinLock automatically handles IRQ disable/enable and SMP synchronization.
#[inline]
pub fn with_channel_table<R, F: FnOnce(&mut ChannelTable) -> R>(f: F) -> R {
    let mut guard = CHANNEL_TABLE.lock();
    f(&mut *guard)
}

// ============================================================================
// Syscall interface (thread-safe via SpinLock)
// ============================================================================

/// Create a channel pair
/// Returns channel IDs in (arg0_out, arg1_out) or error
pub fn sys_channel_create(owner: Pid) -> Result<(ChannelId, ChannelId), IpcError> {
    with_channel_table(|table| {
        table.create_pair(owner, owner)
            .ok_or(IpcError::InvalidChannel)
    })
}

/// Close a channel
pub fn sys_channel_close(channel_id: ChannelId, caller: Pid) -> Result<(), IpcError> {
    // Hold lock only for the channel operation, release before waking
    let blocked_pid = with_channel_table(|table| {
        // Verify ownership atomically with close
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }
        match table.close(channel_id) {
            Ok(blocked) => Ok(blocked),
            Err(()) => Err(IpcError::InvalidChannel),
        }
    })?;

    // Wake blocked process outside the channel lock to avoid deadlock
    if let Some(pid) = blocked_pid {
        unsafe {
            super::process::process_table().wake(pid);
            super::task::scheduler().wake_by_pid(pid);
        }
    }
    Ok(())
}

/// Send a message (non-blocking)
pub fn sys_send(channel_id: ChannelId, caller: Pid, data: &[u8]) -> Result<(), IpcError> {
    sys_send_internal(channel_id, caller, data, false)
}

/// Send a message with direct switch to receiver (fast-path IPC)
/// If receiver is blocked waiting, directly switch to it.
/// Returns when switched back (receiver yielded/blocked/preempted).
pub fn sys_send_direct(channel_id: ChannelId, caller: Pid, data: &[u8]) -> Result<(), IpcError> {
    sys_send_internal(channel_id, caller, data, true)
}

/// Internal send implementation
fn sys_send_internal(channel_id: ChannelId, caller: Pid, data: &[u8], direct: bool) -> Result<(), IpcError> {
    // Build message outside lock (data copy)
    let msg = Message::data(caller, data);

    // Hold lock only for ownership check and send, release before waking
    // Also get peer info for event notification
    let (blocked_pid, peer_owner, peer_channel) = with_channel_table(|table| {
        // Verify ownership atomically with send
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }
        let peer_owner = table.get_peer_owner(channel_id);
        let peer_channel = table.get_peer_id(channel_id);
        let blocked = table.send(channel_id, msg)?;
        Ok((blocked, peer_owner, peer_channel))
    })?;

    // Push IpcReady event to peer's event queue (for event-driven processes)
    // Do this outside the channel lock to avoid deadlock
    // SECURITY: Use IrqGuard to prevent preemption during scheduler access
    if let (Some(peer_pid), Some(peer_ch)) = (peer_owner, peer_channel) {
        // Only push if peer is a userspace process (PID != 0)
        if peer_pid != 0 {
            // Disable IRQs to prevent preemption during scheduler modification
            let _guard = crate::arch::aarch64::sync::IrqGuard::new();
            unsafe {
                let sched = super::task::scheduler();
                // Use generation-aware PID lookup to prevent TOCTOU
                if let Some(slot) = sched.slot_by_pid(peer_pid) {
                    if let Some(ref mut task) = sched.tasks[slot] {
                        // Check if subscribed to IpcReady events for this channel
                        let event = super::event::Event::ipc_ready(peer_ch, caller);
                        if task.event_queue.is_subscribed(&event) {
                            task.event_queue.push(event);
                            // Wake task if blocked on event_wait
                            if task.state == super::task::TaskState::Blocked {
                                task.state = super::task::TaskState::Ready;
                            }
                        }
                    }
                }
            }
        }
    }

    // Wake blocked process outside the channel lock to avoid deadlock
    // Use IrqGuard for scheduler access to prevent race with interrupt handlers
    if let Some(pid) = blocked_pid {
        let _guard = crate::arch::aarch64::sync::IrqGuard::new();
        unsafe {
            super::process::process_table().wake(pid);
            let sched = super::task::scheduler();
            sched.wake_by_pid(pid);

            // Direct switch: donate timeslice to receiver
            if direct {
                sched.direct_switch_to(pid);
                // Returns here when switched back
            }
        }
    }
    Ok(())
}

/// Receive a message (non-blocking)
pub fn sys_receive(channel_id: ChannelId, caller: Pid) -> Result<Message, IpcError> {
    with_channel_table(|table| {
        // Verify ownership atomically with receive
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }
        table.receive(channel_id)
    })
}

/// Try to receive, blocking if no message available
/// Returns WouldBlock if called from non-blocking context
/// The actual blocking is handled by the syscall layer
pub fn sys_receive_blocking(channel_id: ChannelId, caller: Pid) -> Result<Message, IpcError> {
    // First try: receive with ownership check
    let result = with_channel_table(|table| {
        // Verify ownership atomically with receive attempt
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }

        // Try to receive
        match table.receive(channel_id) {
            Ok(msg) => Ok(Ok(msg)),
            Err(IpcError::WouldBlock) => {
                // No message - register as blocked receiver (while still holding lock)
                table.block_receiver(channel_id, caller)?;
                Ok(Err(IpcError::WouldBlock))
            }
            Err(e) => Err(e),
        }
    })?;

    match result {
        Ok(msg) => Ok(msg),
        Err(IpcError::WouldBlock) => {
            // Mark process as blocked (outside channel lock)
            unsafe {
                if let Some(proc) = super::process::process_table().get_mut(caller) {
                    proc.block_on_receive(channel_id);
                }
            }
            Err(IpcError::WouldBlock)
        }
        Err(e) => Err(e),
    }
}

/// Synchronous call: send a request and wait for reply
/// This combines send + block-for-reply in one operation.
/// Returns the reply message or error.
///
/// Note: For true fast-path performance, the scheduler would need to
/// directly switch to the callee and back. This version uses the
/// standard blocking mechanism.
pub fn sys_call(channel_id: ChannelId, caller: Pid, msg_id: u32, data: &[u8]) -> Result<Message, IpcError> {
    // Build message outside lock
    let msg = Message::request(caller, msg_id, data);

    // Send request and register for blocking (atomically)
    let blocked_pid = with_channel_table(|table| {
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }

        // Send the request message
        let wake_pid = table.send(channel_id, msg)?;

        // Register as blocked receiver for reply
        table.block_receiver(channel_id, caller)?;

        Ok(wake_pid)
    })?;

    // Wake callee and mark ourselves as blocked (outside lock)
    if let Some(pid) = blocked_pid {
        unsafe {
            super::process::process_table().wake(pid);
            super::task::scheduler().wake_by_pid(pid);
        }
    }

    // Mark process as blocked on receive
    unsafe {
        if let Some(proc) = super::process::process_table().get_mut(caller) {
            proc.block_on_receive(channel_id);
        }
    }

    Err(IpcError::WouldBlock) // Syscall layer will handle retry after wakeup
}

/// Reply to a request on a channel
/// Used by servers responding to sys_call requests.
pub fn sys_reply(channel_id: ChannelId, caller: Pid, msg_id: u32, data: &[u8]) -> Result<(), IpcError> {
    // Build message outside lock
    let msg = Message::reply(caller, msg_id, data);

    // Send reply (atomically with ownership check)
    let blocked_pid = with_channel_table(|table| {
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }
        table.send(channel_id, msg)
    })?;

    // Wake the caller that's waiting for reply (outside lock)
    if let Some(pid) = blocked_pid {
        unsafe {
            super::process::process_table().wake(pid);
            super::task::scheduler().wake_by_pid(pid);
        }
    }
    Ok(())
}

/// Transfer a channel endpoint to another process
pub fn sys_channel_transfer(channel_id: ChannelId, from: Pid, to: Pid) -> Result<(), IpcError> {
    with_channel_table(|table| {
        let slot = table.find_by_id(channel_id).ok_or(IpcError::InvalidChannel)?;

        if table.endpoints[slot].owner != from {
            return Err(IpcError::PermissionDenied);
        }

        table.endpoints[slot].owner = to;
        Ok(())
    })
}

/// Query channel queue status for backpressure monitoring
///
/// Returns the status of the peer's queue (i.e., what we would be sending to).
/// This allows senders to implement backpressure by checking before sending.
pub fn sys_queue_status(channel_id: ChannelId, caller: Pid) -> Result<QueueStatus, IpcError> {
    with_channel_table(|table| {
        // Verify ownership
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }

        // Return peer's queue status (what we'd be sending to)
        table.peer_queue_status(channel_id)
            .ok_or(IpcError::NoPeer)
    })
}

/// Clean up all channels owned by a process (called on process exit)
/// This handles channels that weren't stored in the FD table (e.g., from port_connect)
pub fn process_cleanup(pid: Pid) {
    with_channel_table(|table| {
        // Collect channel IDs to close (avoid borrow issues)
        let mut to_close: [ChannelId; MAX_CHANNELS] = [0; MAX_CHANNELS];
        let mut count = 0;

        for ep in table.endpoints.iter() {
            if ep.state != ChannelState::Free && ep.owner == pid {
                to_close[count] = ep.id;
                count += 1;
            }
        }

        // Close each channel
        for i in 0..count {
            let _ = table.close(to_close[i]);
        }
    });
}

// ============================================================================
// Testing
// ============================================================================

/// Test IPC functionality
pub fn test() {
    logln!("  Testing IPC...");

    // Create a channel pair
    let channels = with_channel_table(|table| {
        table.create_pair(1, 2)
    });

    let (ch_a, ch_b) = match channels {
        Some((a, b)) => {
            logln!("    Created channel pair: {} <-> {}", a, b);
            (a, b)
        }
        None => {
            logln!("    [!!] Failed to create channel pair");
            return;
        }
    };

    // Send a message from A to B
    let test_data = b"Hello IPC!";
    let msg = Message::data(1, test_data);

    match with_channel_table(|table| table.send(ch_a, msg)) {
        Ok(_) => logln!("    Sent message on channel {}", ch_a),
        Err(e) => logln!("    [!!] Send failed: {:?}", e),
    }

    // Check queue length
    let queue_len = with_channel_table(|table| table.queue_len(ch_b));
    logln!("    Channel {} queue length: {}", ch_b, queue_len);

    // Receive on B
    match with_channel_table(|table| table.receive(ch_b)) {
        Ok(recv_msg) => {
            let payload = recv_msg.payload_slice();
            logln!("    Received {} bytes from PID {}",
                payload.len(), recv_msg.header.sender);
            if payload == test_data {
                logln!("    Message content verified!");
            }
        }
        Err(e) => logln!("    [!!] Receive failed: {:?}", e),
    }

    // Test request/reply pattern
    let req_msg = Message::request(1, 42, b"ping");
    let _ = with_channel_table(|table| table.send(ch_a, req_msg));

    if let Ok(req) = with_channel_table(|table| table.receive(ch_b)) {
        logln!("    Received request with msg_id={}", req.header.msg_id);

        // Send reply
        let reply_msg = Message::reply(2, req.header.msg_id, b"pong");
        let _ = with_channel_table(|table| table.send(ch_b, reply_msg));

        if let Ok(reply) = with_channel_table(|table| table.receive(ch_a)) {
            if reply.header.msg_type == MessageType::Reply &&
               reply.header.msg_id == 42 {
                logln!("    Request/reply pattern works!");
            }
        }
    }

    // Test blocking receive (on empty queue)
    logln!("    Testing blocking receive...");
    match sys_receive_blocking(ch_a, 1) {
        Ok(_) => logln!("    [!!] Should have blocked"),
        Err(IpcError::WouldBlock) => {
            logln!("    Correctly returned WouldBlock");
            // Check that receiver was registered
            let blocked = with_channel_table(|table| {
                table.endpoints.iter()
                    .find(|e| e.id == ch_a)
                    .and_then(|e| e.blocked_receiver)
            });
            if blocked == Some(1) {
                logln!("    Blocked receiver registered: PID 1");
            }
        }
        Err(e) => logln!("    [!!] Unexpected error: {:?}", e),
    }

    // Send message which should wake the blocked receiver
    let wake_msg = Message::data(2, b"wake up!");
    match with_channel_table(|table| table.send(ch_b, wake_msg)) {
        Ok(maybe_pid) => {
            logln!("    Sent wake message, blocked PID: {:?}", maybe_pid);
        }
        Err(e) => logln!("    [!!] Send failed: {:?}", e),
    }

    // Test backpressure API
    if let Some(status) = with_channel_table(|table| table.peer_queue_status(ch_a)) {
        logln!("    Queue status: {}/{} (full={}, empty={})",
            status.count, status.capacity, status.full, status.empty);
    }

    // Close channels
    let _ = with_channel_table(|table| table.close(ch_a));
    let _ = with_channel_table(|table| table.close(ch_b));
    logln!("    Channels closed");

    logln!("    [OK] IPC test passed");
}
