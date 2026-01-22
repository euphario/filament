//! BPI-R4 Kernel Library
//!
//! This crate provides the testable pure-logic components of the kernel.
//! It's used for running unit tests with `cargo test`.
//!
//! ## Usage
//!
//! ```sh
//! ./test.sh
//! ```
//!
//! The kernel uses `aarch64-unknown-none` target which conflicts with host tests.
//! Use `./test.sh` which temporarily disables the cargo config for testing.
//!
//! ## What's Testable
//!
//! Only pure-logic modules that don't depend on:
//! - Hardware (MMIO, interrupts, timers)
//! - Architecture-specific code (assembly, MMU)
//! - Global mutable state (statics requiring synchronization)
//!
//! Currently testable:
//! - IPC data structures (MessageQueue, ChannelTable, Message)
//! - Handle types (Handle, HandleRights)
//!
//! ## Adding Tests
//!
//! To add tests to a kernel module:
//! 1. Ensure the code is pure logic (no hardware/arch deps)
//! 2. Add `#[cfg(test)] mod tests { ... }` at the end of the module
//! 3. Re-export the module here if needed

// Only build library for tests (not for kernel binary)
#![cfg_attr(not(test), no_std)]

// For tests, we need the standard library
#[cfg(test)]
extern crate std;

// Re-export testable modules
// These modules contain pure logic that can run on the host

/// IPC data structures and channel management
pub mod ipc {
    // Type alias for process IDs (matches kernel definition)
    pub type Pid = u32;

    /// Maximum inline message payload size
    pub const MAX_INLINE_PAYLOAD: usize = 576;

    /// Maximum messages in a queue
    pub const MAX_QUEUE_SIZE: usize = 8;

    /// Maximum number of channels
    pub const MAX_CHANNELS: usize = 64;

    /// Channel ID type
    pub type ChannelId = u32;

    /// Message types
    #[repr(u32)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum MessageType {
        Data = 0,
        Request = 1,
        Reply = 2,
        Error = 3,
        Close = 4,
        Connect = 5,
        Accept = 6,
    }

    /// Message header
    #[repr(C)]
    #[derive(Clone, Copy)]
    pub struct MessageHeader {
        pub msg_type: MessageType,
        pub sender: Pid,
        pub msg_id: u32,
        pub payload_len: u32,
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

        pub fn data(sender: Pid, data: &[u8]) -> Self {
            let mut msg = Self::new();
            msg.header.msg_type = MessageType::Data;
            msg.header.sender = sender;
            let len = core::cmp::min(data.len(), MAX_INLINE_PAYLOAD);
            msg.header.payload_len = len as u32;
            msg.payload[..len].copy_from_slice(&data[..len]);
            msg
        }

        pub fn request(sender: Pid, msg_id: u32, data: &[u8]) -> Self {
            let mut msg = Self::data(sender, data);
            msg.header.msg_type = MessageType::Request;
            msg.header.msg_id = msg_id;
            msg
        }

        pub fn reply(sender: Pid, msg_id: u32, data: &[u8]) -> Self {
            let mut msg = Self::data(sender, data);
            msg.header.msg_type = MessageType::Reply;
            msg.header.msg_id = msg_id;
            msg
        }

        pub fn error(sender: Pid, msg_id: u32, error_code: i32) -> Self {
            let mut msg = Self::new();
            msg.header.msg_type = MessageType::Error;
            msg.header.sender = sender;
            msg.header.msg_id = msg_id;
            msg.header.payload_len = 4;
            msg.payload[0..4].copy_from_slice(&error_code.to_le_bytes());
            msg
        }

        pub fn payload_slice(&self) -> &[u8] {
            let len = core::cmp::min(self.header.payload_len as usize, MAX_INLINE_PAYLOAD);
            &self.payload[..len]
        }
    }

    /// Queue status for backpressure monitoring
    #[derive(Debug, Clone, Copy)]
    pub struct QueueStatus {
        pub count: usize,
        pub capacity: usize,
        pub full: bool,
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

        pub fn is_empty(&self) -> bool {
            self.count == 0
        }

        pub fn is_full(&self) -> bool {
            self.count >= MAX_QUEUE_SIZE
        }

        pub fn push(&mut self, msg: Message) -> bool {
            if self.is_full() {
                return false;
            }
            self.messages[self.tail] = msg;
            self.tail = (self.tail + 1) % MAX_QUEUE_SIZE;
            self.count += 1;
            true
        }

        pub fn pop(&mut self) -> Option<Message> {
            if self.is_empty() {
                return None;
            }
            let msg = self.messages[self.head];
            self.head = (self.head + 1) % MAX_QUEUE_SIZE;
            self.count -= 1;
            Some(msg)
        }

        pub fn peek(&self) -> Option<&Message> {
            if self.is_empty() {
                None
            } else {
                Some(&self.messages[self.head])
            }
        }

        pub fn len(&self) -> usize {
            self.count
        }

        pub fn status(&self) -> QueueStatus {
            QueueStatus {
                count: self.count,
                capacity: MAX_QUEUE_SIZE,
                full: self.is_full(),
                empty: self.is_empty(),
            }
        }
    }

    /// Channel endpoint state
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ChannelState {
        Free,
        Open,
        Closed,
    }

    /// A channel endpoint
    pub struct ChannelEndpoint {
        pub id: ChannelId,
        pub owner: Pid,
        pub peer: ChannelId,
        pub state: ChannelState,
        pub queue: MessageQueue,
        pub blocked_receiver: Option<Pid>,
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

    /// Channel error types
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ChannelError {
        InvalidChannel,
        NoPeer,
        PeerClosed,
        QueueFull,
        WouldBlock,
        PermissionDenied,
        InvalidMessage,
    }

    impl ChannelError {
        pub fn to_errno(self) -> i32 {
            match self {
                ChannelError::InvalidChannel => -9,
                ChannelError::NoPeer => -104,
                ChannelError::PeerClosed => -104,
                ChannelError::QueueFull => -11,
                ChannelError::WouldBlock => -11,
                ChannelError::PermissionDenied => -13,
                ChannelError::InvalidMessage => -22,
            }
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

        fn alloc_id(&mut self) -> ChannelId {
            let id = self.next_id;
            self.next_id += 1;
            id
        }

        fn find_free_slot(&self) -> Option<usize> {
            self.endpoints.iter().position(|e| e.state == ChannelState::Free)
        }

        pub fn find_by_id(&self, id: ChannelId) -> Option<usize> {
            self.endpoints.iter().position(|e| e.id == id && e.state != ChannelState::Free)
        }

        pub fn create_pair(&mut self, owner_a: Pid, owner_b: Pid) -> Option<(ChannelId, ChannelId)> {
            let slot_a = self.find_free_slot()?;
            let slot_b = self.endpoints.iter()
                .enumerate()
                .position(|(i, e)| i != slot_a && e.state == ChannelState::Free)?;

            let id_a = self.alloc_id();
            let id_b = self.alloc_id();

            self.endpoints[slot_a].id = id_a;
            self.endpoints[slot_a].owner = owner_a;
            self.endpoints[slot_a].peer = id_b;
            self.endpoints[slot_a].queue = MessageQueue::new();
            self.endpoints[slot_a].blocked_receiver = None;
            self.endpoints[slot_a].state = ChannelState::Open;

            self.endpoints[slot_b].id = id_b;
            self.endpoints[slot_b].owner = owner_b;
            self.endpoints[slot_b].peer = id_a;
            self.endpoints[slot_b].queue = MessageQueue::new();
            self.endpoints[slot_b].blocked_receiver = None;
            self.endpoints[slot_b].state = ChannelState::Open;

            Some((id_a, id_b))
        }

        pub fn close(&mut self, id: ChannelId) -> Result<Option<Pid>, ()> {
            if let Some(slot) = self.find_by_id(id) {
                let peer_id = self.endpoints[slot].peer;
                self.endpoints[slot].state = ChannelState::Closed;

                let mut blocked_pid = None;

                if peer_id != 0 {
                    if let Some(peer_slot) = self.find_by_id(peer_id) {
                        blocked_pid = self.endpoints[peer_slot].blocked_receiver;
                        self.endpoints[peer_slot].peer_closed = true;

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

                        if blocked_pid.is_some() {
                            self.endpoints[peer_slot].blocked_receiver = None;
                        }
                    }
                }

                self.endpoints[slot].reset();
                Ok(blocked_pid)
            } else {
                Err(())
            }
        }

        pub fn send(&mut self, channel_id: ChannelId, msg: Message) -> Result<Option<Pid>, ChannelError> {
            let slot = self.find_by_id(channel_id).ok_or(ChannelError::InvalidChannel)?;
            let peer_id = self.endpoints[slot].peer;

            if peer_id == 0 {
                return Err(ChannelError::NoPeer);
            }

            let peer_slot = self.find_by_id(peer_id).ok_or(ChannelError::PeerClosed)?;

            if self.endpoints[peer_slot].state == ChannelState::Closed {
                return Err(ChannelError::PeerClosed);
            }

            if self.endpoints[peer_slot].queue.is_full() {
                return Err(ChannelError::QueueFull);
            }

            self.endpoints[peer_slot].queue.push(msg);
            let blocked = self.endpoints[peer_slot].blocked_receiver.take();
            Ok(blocked)
        }

        pub fn receive(&mut self, channel_id: ChannelId) -> Result<Message, ChannelError> {
            let slot = self.find_by_id(channel_id).ok_or(ChannelError::InvalidChannel)?;

            if let Some(msg) = self.endpoints[slot].queue.pop() {
                return Ok(msg);
            }

            if self.endpoints[slot].peer_closed {
                return Err(ChannelError::PeerClosed);
            }

            Err(ChannelError::WouldBlock)
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_queue_new_is_empty() {
            let queue = MessageQueue::new();
            assert!(queue.is_empty());
            assert!(!queue.is_full());
            assert_eq!(queue.len(), 0);
        }

        #[test]
        fn test_queue_push_pop() {
            let mut queue = MessageQueue::new();
            let msg = Message::data(1, b"hello");

            assert!(queue.push(msg));
            assert!(!queue.is_empty());
            assert_eq!(queue.len(), 1);

            let popped = queue.pop();
            assert!(popped.is_some());
            assert_eq!(popped.unwrap().header.sender, 1);
            assert!(queue.is_empty());
        }

        #[test]
        fn test_queue_fifo_order() {
            let mut queue = MessageQueue::new();

            for i in 1..=3 {
                let msg = Message::data(i, b"test");
                assert!(queue.push(msg));
            }

            assert_eq!(queue.pop().unwrap().header.sender, 1);
            assert_eq!(queue.pop().unwrap().header.sender, 2);
            assert_eq!(queue.pop().unwrap().header.sender, 3);
            assert!(queue.pop().is_none());
        }

        #[test]
        fn test_queue_full() {
            let mut queue = MessageQueue::new();

            for i in 0..MAX_QUEUE_SIZE {
                let msg = Message::data(i as u32, b"test");
                assert!(queue.push(msg));
            }

            assert!(queue.is_full());
            assert!(!queue.push(Message::data(99, b"overflow")));
        }

        #[test]
        fn test_message_data() {
            let msg = Message::data(123, b"hello world");
            assert_eq!(msg.header.msg_type, MessageType::Data);
            assert_eq!(msg.header.sender, 123);
            assert_eq!(msg.header.payload_len, 11);
            assert_eq!(&msg.payload[..11], b"hello world");
        }

        #[test]
        fn test_channel_table_create_pair() {
            let mut table = ChannelTable::new();
            let (id_a, id_b) = table.create_pair(1, 2).unwrap();

            assert_ne!(id_a, id_b);

            let slot_a = table.find_by_id(id_a).unwrap();
            let slot_b = table.find_by_id(id_b).unwrap();

            assert_eq!(table.endpoints[slot_a].peer, id_b);
            assert_eq!(table.endpoints[slot_b].peer, id_a);
        }

        #[test]
        fn test_channel_table_send_receive() {
            let mut table = ChannelTable::new();
            let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

            let msg = Message::data(1, b"hello");
            assert!(table.send(ch_a, msg).is_ok());

            let recv_msg = table.receive(ch_b).unwrap();
            assert_eq!(recv_msg.header.sender, 1);
            assert_eq!(recv_msg.payload_slice(), b"hello");
        }

        #[test]
        fn test_channel_error_to_errno() {
            assert_eq!(ChannelError::InvalidChannel.to_errno(), -9);
            assert_eq!(ChannelError::WouldBlock.to_errno(), -11);
            assert_eq!(ChannelError::PeerClosed.to_errno(), -104);
        }
    }
}

/// Handle types for capability-based access
pub mod handle {
    // Handle and HandleRights imported from abi crate (single source of truth)
    pub use abi::{Handle, HandleRights};

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_handle_encoding() {
            let h = Handle::new(42, 7);
            assert_eq!(h.index(), 42);
            assert_eq!(h.generation(), 7);

            let h2 = Handle::new(0x00FFFFFF, 255);
            assert_eq!(h2.index(), 0x00FFFFFF);
            assert_eq!(h2.generation(), 255);
        }

        #[test]
        fn test_handle_invalid() {
            assert!(!Handle::INVALID.is_valid());
            assert!(Handle::new(1, 1).is_valid());
        }

        #[test]
        fn test_handle_raw_roundtrip() {
            let h = Handle::new(123, 45);
            let raw = h.raw();
            let h2 = Handle::from_raw(raw);
            assert_eq!(h, h2);
            assert_eq!(h.index(), h2.index());
            assert_eq!(h.generation(), h2.generation());
        }

        #[test]
        fn test_rights_operations() {
            let rw = HandleRights::READ.or(HandleRights::WRITE);
            assert!(rw.has(HandleRights::READ));
            assert!(rw.has(HandleRights::WRITE));
            assert!(!rw.has(HandleRights::WAIT));

            let restricted = rw.and(HandleRights::READ);
            assert!(restricted.has(HandleRights::READ));
            assert!(!restricted.has(HandleRights::WRITE));
        }

        #[test]
        fn test_rights_full() {
            let full = HandleRights::FULL;
            assert!(full.has(HandleRights::READ));
            assert!(full.has(HandleRights::WRITE));
            assert!(full.has(HandleRights::WAIT));
            assert!(full.has(HandleRights::SIGNAL));
            assert!(full.has(HandleRights::CLOSE));
            assert!(full.has(HandleRights::TRANSFER));
            assert!(full.has(HandleRights::DUPLICATE));
            assert!(full.has(HandleRights::MAP));
        }
    }
}
