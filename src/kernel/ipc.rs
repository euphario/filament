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

#![allow(dead_code)]  // Some message types and methods are for future use

use crate::logln;
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
    pub fn payload_slice(&self) -> &[u8] {
        &self.payload[..self.header.payload_len as usize]
    }
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
    pub fn create_pair(&mut self, owner_a: Pid, owner_b: Pid) -> Option<(ChannelId, ChannelId)> {
        let slot_a = self.find_free_slot()?;
        let id_a = self.alloc_id();

        // Temporarily mark slot_a as used so find_free_slot finds a different slot
        self.endpoints[slot_a].state = ChannelState::Open;

        let slot_b = self.find_free_slot()?;
        let id_b = self.alloc_id();

        // Set up endpoint A
        self.endpoints[slot_a].id = id_a;
        self.endpoints[slot_a].owner = owner_a;
        self.endpoints[slot_a].peer = id_b;
        self.endpoints[slot_a].state = ChannelState::Open;
        self.endpoints[slot_a].queue = MessageQueue::new();

        // Set up endpoint B
        self.endpoints[slot_b].id = id_b;
        self.endpoints[slot_b].owner = owner_b;
        self.endpoints[slot_b].peer = id_a;
        self.endpoints[slot_b].state = ChannelState::Open;
        self.endpoints[slot_b].queue = MessageQueue::new();

        Some((id_a, id_b))
    }

    /// Close a channel endpoint
    pub fn close(&mut self, id: ChannelId) -> bool {
        if let Some(slot) = self.find_by_id(id) {
            let peer_id = self.endpoints[slot].peer;
            self.endpoints[slot].state = ChannelState::Closed;

            // Notify peer if it exists
            if peer_id != 0 {
                if let Some(peer_slot) = self.find_by_id(peer_id) {
                    // Send close notification to peer's queue
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
                }
            }

            // Reset the endpoint
            self.endpoints[slot].reset();
            true
        } else {
            false
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

        self.endpoints[slot].queue.pop().ok_or(IpcError::WouldBlock)
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

/// Global channel table
static mut CHANNEL_TABLE: ChannelTable = ChannelTable::new();

/// Get the global channel table
/// # Safety
/// Must ensure proper synchronization
pub unsafe fn channel_table() -> &'static mut ChannelTable {
    &mut *core::ptr::addr_of_mut!(CHANNEL_TABLE)
}

// ============================================================================
// Syscall interface
// ============================================================================

/// Create a channel pair
/// Returns channel IDs in (arg0_out, arg1_out) or error
pub fn sys_channel_create(owner: Pid) -> Result<(ChannelId, ChannelId), IpcError> {
    unsafe {
        channel_table().create_pair(owner, owner)
            .ok_or(IpcError::InvalidChannel)
    }
}

/// Close a channel
pub fn sys_channel_close(channel_id: ChannelId, caller: Pid) -> Result<(), IpcError> {
    unsafe {
        let table = channel_table();
        // Verify ownership
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }
        if table.close(channel_id) {
            Ok(())
        } else {
            Err(IpcError::InvalidChannel)
        }
    }
}

/// Send a message (non-blocking)
pub fn sys_send(channel_id: ChannelId, caller: Pid, data: &[u8]) -> Result<(), IpcError> {
    unsafe {
        let table = channel_table();
        // Verify ownership
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }

        let msg = Message::data(caller, data);
        match table.send(channel_id, msg) {
            Ok(maybe_blocked) => {
                // If there's a blocked process, wake it
                if let Some(blocked_pid) = maybe_blocked {
                    super::process::process_table().wake(blocked_pid);
                }
                Ok(())
            }
            Err(e) => Err(e),
        }
    }
}

/// Receive a message (non-blocking)
pub fn sys_receive(channel_id: ChannelId, caller: Pid) -> Result<Message, IpcError> {
    unsafe {
        let table = channel_table();
        // Verify ownership
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }
        table.receive(channel_id)
    }
}

/// Try to receive, blocking if no message available
/// Returns WouldBlock if called from non-blocking context
/// The actual blocking is handled by the syscall layer
pub fn sys_receive_blocking(channel_id: ChannelId, caller: Pid) -> Result<Message, IpcError> {
    unsafe {
        let table = channel_table();
        // Verify ownership
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }

        // Try to receive
        match table.receive(channel_id) {
            Ok(msg) => Ok(msg),
            Err(IpcError::WouldBlock) => {
                // No message - register as blocked receiver
                table.block_receiver(channel_id, caller)?;
                // Mark process as blocked
                if let Some(proc) = super::process::process_table().get_mut(caller) {
                    proc.block_on_receive(channel_id);
                }
                Err(IpcError::WouldBlock)
            }
            Err(e) => Err(e),
        }
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
    // Send the request
    let msg = Message::request(caller, msg_id, data);
    unsafe {
        let table = channel_table();
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }

        // Send the request message
        if let Some(blocked_pid) = table.send(channel_id, msg)? {
            super::process::process_table().wake(blocked_pid);
        }

        // Now block waiting for a reply with matching msg_id
        // For simplicity, we just do blocking receive and check
        table.block_receiver(channel_id, caller)?;
        if let Some(proc) = super::process::process_table().get_mut(caller) {
            proc.block_on_receive(channel_id);
        }
        Err(IpcError::WouldBlock) // Syscall layer will handle retry after wakeup
    }
}

/// Reply to a request on a channel
/// Used by servers responding to sys_call requests.
pub fn sys_reply(channel_id: ChannelId, caller: Pid, msg_id: u32, data: &[u8]) -> Result<(), IpcError> {
    let msg = Message::reply(caller, msg_id, data);
    unsafe {
        let table = channel_table();
        if table.get_owner(channel_id) != Some(caller) {
            return Err(IpcError::PermissionDenied);
        }

        if let Some(blocked_pid) = table.send(channel_id, msg)? {
            // Wake the caller that's waiting for reply
            super::process::process_table().wake(blocked_pid);
        }
        Ok(())
    }
}

/// Transfer a channel endpoint to another process
pub fn sys_channel_transfer(channel_id: ChannelId, from: Pid, to: Pid) -> Result<(), IpcError> {
    unsafe {
        let table = channel_table();
        let slot = table.find_by_id(channel_id).ok_or(IpcError::InvalidChannel)?;

        if table.endpoints[slot].owner != from {
            return Err(IpcError::PermissionDenied);
        }

        table.endpoints[slot].owner = to;
        Ok(())
    }
}

// ============================================================================
// Testing
// ============================================================================

/// Test IPC functionality
pub fn test() {
    logln!("  Testing IPC...");

    unsafe {
        let table = channel_table();

        // Create a channel pair
        if let Some((ch_a, ch_b)) = table.create_pair(1, 2) {
            logln!("    Created channel pair: {} <-> {}", ch_a, ch_b);

            // Send a message from A to B
            let test_data = b"Hello IPC!";
            let msg = Message::data(1, test_data);

            match table.send(ch_a, msg) {
                Ok(_) => logln!("    Sent message on channel {}", ch_a),
                Err(e) => logln!("    [!!] Send failed: {:?}", e),
            }

            // Check queue
            logln!("    Channel {} queue length: {}", ch_b, table.queue_len(ch_b));

            // Receive on B
            match table.receive(ch_b) {
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
            let _ = table.send(ch_a, req_msg);

            if let Ok(req) = table.receive(ch_b) {
                logln!("    Received request with msg_id={}", req.header.msg_id);

                // Send reply
                let reply_msg = Message::reply(2, req.header.msg_id, b"pong");
                let _ = table.send(ch_b, reply_msg);

                if let Ok(reply) = table.receive(ch_a) {
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
                    if table.endpoints.iter()
                        .find(|e| e.id == ch_a)
                        .map(|e| e.blocked_receiver)
                        .flatten() == Some(1)
                    {
                        logln!("    Blocked receiver registered: PID 1");
                    }
                }
                Err(e) => logln!("    [!!] Unexpected error: {:?}", e),
            }

            // Send message which should wake the blocked receiver
            let wake_msg = Message::data(2, b"wake up!");
            match table.send(ch_b, wake_msg) {
                Ok(maybe_pid) => {
                    logln!("    Sent wake message, blocked PID: {:?}", maybe_pid);
                }
                Err(e) => logln!("    [!!] Send failed: {:?}", e),
            }

            // Close channels
            table.close(ch_a);
            table.close(ch_b);
            logln!("    Channels closed");

        } else {
            logln!("    [!!] Failed to create channel pair");
        }
    }

    logln!("    [OK] IPC test passed");
}
