//! Bus Client — connection to busd for message routing
//!
//! Provides a high-level API for connecting to the bus daemon (busd) and
//! sending/receiving BusMsg messages. Used by both bus_runtime (drivers)
//! and devd v2.
//!
//! Transport is abstracted — currently uses Channel, swappable to shmem
//! rings later without changing the API.

use abi::{BusMsgHeader, BusMsgBuilder, bus_msg_type, bus_msg_flags};
use libsys::ipc::Channel;
use libsys::syscall::{self, Handle};
use libsys::error::SysError;

const MSG_BUF_SIZE: usize = 576;

// =============================================================================
// BusClient
// =============================================================================

/// Connection to busd. Provides fire-and-forget messaging.
pub struct BusClient {
    channel: Channel,
    next_seq: u16,
}

impl BusClient {
    /// Connect to the bus daemon via the `bus:` port.
    pub fn connect() -> Result<Self, SysError> {
        let channel = Channel::connect(b"bus:")?;
        Ok(Self { channel, next_seq: 1 })
    }

    /// Wrap an existing channel as a bus client (e.g. from exec_with_channel).
    pub fn from_channel(channel: Channel) -> Self {
        Self { channel, next_seq: 1 }
    }

    /// Get the underlying channel handle (for Mux registration).
    pub fn handle(&self) -> Handle {
        self.channel.handle()
    }

    /// Allocate a sequence ID for request tracking.
    fn alloc_seq(&mut self) -> u16 {
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        if self.next_seq == 0 { self.next_seq = 1; }
        seq
    }

    // =========================================================================
    // Registration
    // =========================================================================

    /// Register this client with busd (name, path, class, subscribe_events).
    pub fn register(
        &mut self,
        name: &[u8],
        path: &[u8],
        class: u8,
        subscribe_events: bool,
    ) -> Result<(), SysError> {
        let mut buf = [0u8; MSG_BUF_SIZE];
        let seq = self.alloc_seq();
        let mut value = [0u8; 2];
        value[0] = class;
        value[1] = if subscribe_events { 1 } else { 0 };

        let len = BusMsgBuilder::new(&mut buf, bus_msg_type::REGISTER, seq)
            .addr(name)
            .key(path)
            .value(&value)
            .finish();

        self.channel.send(&buf[..len])
    }

    // =========================================================================
    // Fire-and-forget sends
    // =========================================================================

    /// Emit an event (fire and forget). busd routes to all subscribers.
    pub fn emit(&mut self, key: &[u8], value: &[u8]) -> Result<(), SysError> {
        let mut buf = [0u8; MSG_BUF_SIZE];
        let seq = self.alloc_seq();
        let len = BusMsgBuilder::new(&mut buf, bus_msg_type::EVENT, seq)
            .key(key)
            .value(value)
            .finish();

        self.channel.send(&buf[..len])
    }

    /// Send a lifecycle command to a target address.
    pub fn send_lifecycle(
        &mut self,
        addr: &[u8],
        key: &[u8],
        value: &[u8],
    ) -> Result<u16, SysError> {
        let mut buf = [0u8; MSG_BUF_SIZE];
        let seq = self.alloc_seq();
        let len = BusMsgBuilder::new(&mut buf, bus_msg_type::LIFECYCLE, seq)
            .addr(addr)
            .key(key)
            .value(value)
            .finish();

        self.channel.send(&buf[..len])?;
        Ok(seq)
    }

    /// Send a GET request to a target address.
    pub fn send_get(
        &mut self,
        addr: &[u8],
        key: &[u8],
    ) -> Result<u16, SysError> {
        let mut buf = [0u8; MSG_BUF_SIZE];
        let seq = self.alloc_seq();
        let len = BusMsgBuilder::new(&mut buf, bus_msg_type::GET, seq)
            .addr(addr)
            .key(key)
            .finish();

        self.channel.send(&buf[..len])?;
        Ok(seq)
    }

    /// Send a SET request to a target address.
    pub fn send_set(
        &mut self,
        addr: &[u8],
        key: &[u8],
        value: &[u8],
    ) -> Result<u16, SysError> {
        let mut buf = [0u8; MSG_BUF_SIZE];
        let seq = self.alloc_seq();
        let len = BusMsgBuilder::new(&mut buf, bus_msg_type::SET, seq)
            .addr(addr)
            .key(key)
            .value(value)
            .finish();

        self.channel.send(&buf[..len])?;
        Ok(seq)
    }

    /// Send a reply to a request (matching seq_id).
    pub fn send_reply(
        &mut self,
        seq_id: u16,
        value: &[u8],
    ) -> Result<(), SysError> {
        let mut buf = [0u8; MSG_BUF_SIZE];
        let len = BusMsgBuilder::new(&mut buf, bus_msg_type::REPLY, seq_id)
            .flags(bus_msg_flags::EOL)
            .value(value)
            .finish();

        self.channel.send(&buf[..len])
    }

    /// Send an error reply to a request.
    pub fn send_error_reply(
        &mut self,
        seq_id: u16,
        error_msg: &[u8],
    ) -> Result<(), SysError> {
        let mut buf = [0u8; MSG_BUF_SIZE];
        let len = BusMsgBuilder::new(&mut buf, bus_msg_type::REPLY, seq_id)
            .flags(bus_msg_flags::EOL | bus_msg_flags::ERROR)
            .value(error_msg)
            .finish();

        self.channel.send(&buf[..len])
    }

    // =========================================================================
    // Receive
    // =========================================================================

    /// Try to receive a message (non-blocking).
    /// Returns None if no message available (WouldBlock).
    /// Returns the raw bytes and length on success.
    pub fn try_recv(&self, buf: &mut [u8; MSG_BUF_SIZE]) -> Result<Option<usize>, SysError> {
        match syscall::read(self.channel.handle(), buf) {
            Ok(n) => Ok(Some(n)),
            Err(SysError::WouldBlock) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Receive a message (blocking).
    pub fn recv(&mut self, buf: &mut [u8; MSG_BUF_SIZE]) -> Result<usize, SysError> {
        self.channel.recv(buf)
    }

    /// Blocking query: send GET, wait for REPLY with matching seq_id.
    /// Times out after `timeout_ns` nanoseconds.
    pub fn query(
        &mut self,
        addr: &[u8],
        key: &[u8],
        timeout_ns: u64,
    ) -> Result<QueryReply, SysError> {
        let seq = self.send_get(addr, key)?;
        self.wait_reply(seq, timeout_ns)
    }

    /// Wait for a reply matching the given seq_id with timeout.
    /// Discards non-matching messages (events, other replies) and retries
    /// up to 8 times within the timeout window.
    pub fn wait_reply(&mut self, seq_id: u16, timeout_ns: u64) -> Result<QueryReply, SysError> {
        for _ in 0..8 {
            let mut buf = [0u8; MSG_BUF_SIZE];
            let len = self.channel.recv_deadline(&mut buf, timeout_ns)?;

            if len < BusMsgHeader::SIZE {
                continue;
            }

            let header = match BusMsgHeader::read_from(&buf[..len]) {
                Some(h) => h,
                None => continue,
            };

            if header.msg_type != bus_msg_type::REPLY || header.seq_id != seq_id {
                // Non-matching message (event, stale reply) — discard and retry
                continue;
            }

            let is_error = (header.flags & bus_msg_flags::ERROR) != 0;
            let payload_start = BusMsgHeader::SIZE;
            let value_start = payload_start + header.addr_len as usize + header.key_len as usize;
            let value_end = (value_start + header.value_len as usize).min(len);

            return Ok(QueryReply {
                value_start,
                value_end,
                is_error,
                buf,
            });
        }

        Err(SysError::Timeout)
    }
}

// =============================================================================
// QueryReply
// =============================================================================

/// Result of a blocking query. Holds the reply buffer and value location.
pub struct QueryReply {
    buf: [u8; MSG_BUF_SIZE],
    value_start: usize,
    value_end: usize,
    pub is_error: bool,
}

impl QueryReply {
    /// Get the reply value bytes.
    pub fn value(&self) -> &[u8] {
        if self.value_start <= self.value_end && self.value_end <= self.buf.len() {
            &self.buf[self.value_start..self.value_end]
        } else {
            &[]
        }
    }
}
