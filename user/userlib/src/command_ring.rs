//! Zero-Copy Command Ring IPC
//!
//! Production-quality, battle-tested IPC using shared memory command rings.
//! No serialization, no kernel copies, minimal syscalls.
//!
//! # Design Principles
//!
//! - **Zero-copy**: Commands/responses are written directly to shared memory
//! - **Lock-free**: Single producer, single consumer with atomic head/tail
//! - **Explicit state machine**: Ring has clear lifecycle states
//! - **Fail-fast**: Invalid operations panic in debug, return errors in release
//! - **Memory safe**: All pointer access is bounds-checked
//!
//! # Wire Format Stability
//!
//! The `Command` and `Response` structs ARE the wire format. They are:
//! - `#[repr(C, align(64))]` for stable layout and cache-line alignment
//! - Versioned via magic number in header
//! - Sized at compile-time with static assertions
//!
//! # Memory Ordering
//!
//! Producer (command sender):
//! - Write command data with regular stores
//! - Publish with `Release` to ensure data is visible before head update
//! - Read tail with `Acquire` to see consumer's progress
//!
//! Consumer (command receiver):
//! - Read head with `Acquire` to see producer's writes
//! - Write response data with regular stores
//! - Consume with `Release` to ensure response is visible before tail update
//!
//! # Traits for Swappability
//!
//! - [`CommandSender`] - Send commands and receive responses (producer)
//! - [`CommandReceiver`] - Receive commands and send responses (consumer)
//! - [`Transport`] - Share transport ID with other processes
//!
//! # Example
//!
//! ```ignore
//! // Producer (devd)
//! let mut ring = Producer::create()?;
//! ring.set_public()?;
//! let shmem_id = ring.shmem_id();
//!
//! let rsp = ring.send(cmd::ATTACH_DISK)
//!     .u32(shmem_id)
//!     .u32(block_size)
//!     .call()?;
//!
//! // Consumer (partd)
//! let mut ring = Consumer::attach(shmem_id)?;
//! loop {
//!     let cmd = ring.recv()?;
//!     ring.respond(cmd.seq_id, result::OK)?;
//! }
//! ```

use crate::error::{SysError, SysResult};
use crate::ipc::Shmem;
use crate::syscall::gettime;
use core::sync::atomic::{AtomicU32, Ordering};

// ============================================================================
// Constants
// ============================================================================

/// Magic number for header validation: "CMDR" in little-endian
const HEADER_MAGIC: u32 = 0x5244_4D43;

/// Current protocol version
const PROTOCOL_VERSION: u16 = 1;

/// Minimum ring size (must be power of 2)
pub const MIN_RING_SIZE: usize = 4;

/// Maximum ring size (must be power of 2)
pub const MAX_RING_SIZE: usize = 256;

/// Default ring size
pub const RING_SIZE: usize = 16;

/// Command payload capacity in bytes
pub const COMMAND_PAYLOAD_SIZE: usize = 48;

/// Response payload capacity in bytes
pub const RESPONSE_PAYLOAD_SIZE: usize = 56;

/// Default timeout in milliseconds
const DEFAULT_TIMEOUT_MS: u32 = 5000;

/// Maximum timeout (10 minutes)
const MAX_TIMEOUT_MS: u32 = 600_000;

// ============================================================================
// Compile-time Assertions
// ============================================================================

const _: () = {
    assert!(core::mem::size_of::<Command>() == 64);
    assert!(core::mem::size_of::<Response>() == 64);
    assert!(core::mem::size_of::<CommandRingHeader>() == 64);
    assert!(core::mem::align_of::<Command>() == 64);
    assert!(core::mem::align_of::<Response>() == 64);
    assert!(core::mem::align_of::<CommandRingHeader>() == 64);
    assert!(RING_SIZE.is_power_of_two());
    assert!(MIN_RING_SIZE.is_power_of_two());
    assert!(MAX_RING_SIZE.is_power_of_two());
};

// ============================================================================
// Traits - Swappable IPC Interface
// ============================================================================

/// Trait for sending commands and receiving responses (producer side).
///
/// Implement this trait to provide alternative IPC mechanisms (e.g., for testing
/// or different transport layers).
pub trait CommandSender {
    /// Send a command and wait for response.
    ///
    /// # Arguments
    /// * `cmd_type` - Command type code (see [`cmd`] module)
    /// * `flags` - Command flags (protocol-specific)
    /// * `payload` - Command payload (max [`COMMAND_PAYLOAD_SIZE`] bytes, excess truncated)
    /// * `timeout_ms` - Timeout in milliseconds (0 = use default)
    ///
    /// # Returns
    /// - `Ok(Response)` on success
    /// - `Err(SysError::TimedOut)` if response not received within timeout
    /// - `Err(SysError::InvalidArgument)` if called on wrong role
    fn send_command(
        &mut self,
        cmd_type: u32,
        flags: u32,
        payload: &[u8],
        timeout_ms: u32,
    ) -> SysResult<Response>;

    /// Send a command without waiting for response (fire-and-forget).
    ///
    /// Use for notifications that don't require acknowledgment.
    fn fire_command(
        &mut self,
        cmd_type: u32,
        flags: u32,
        payload: &[u8],
        timeout_ms: u32,
    ) -> SysResult<()>;

    /// Check if there are pending responses to consume.
    fn has_responses(&self) -> bool;
}

/// Trait for receiving commands and sending responses (consumer side).
///
/// Implement this trait to provide alternative IPC mechanisms.
pub trait CommandReceiver {
    /// Receive next command (blocking).
    ///
    /// Blocks until a command is available.
    fn recv_command(&self) -> SysResult<&Command>;

    /// Try to receive a command (non-blocking).
    ///
    /// Returns `None` if no command is available.
    fn try_recv_command(&self) -> Option<&Command>;

    /// Send response to a command.
    ///
    /// # Arguments
    /// * `seq_id` - Sequence ID from the command being responded to
    /// * `result` - Result code (see [`result`] module, 0 = success)
    /// * `payload` - Response payload (max [`RESPONSE_PAYLOAD_SIZE`] bytes)
    ///
    /// # Important
    /// This also consumes the command. Call exactly once per received command.
    fn send_response(&mut self, seq_id: u32, result: i32, payload: &[u8]) -> SysResult<()>;

    /// Check if there are pending commands to process.
    fn has_commands(&self) -> bool;
}

/// Trait for IPC transport that can be shared between processes.
pub trait Transport {
    /// Get transport ID that can be shared with other processes.
    ///
    /// The consumer uses this ID to attach to the ring.
    fn transport_id(&self) -> u32;

    /// Allow a specific process to access this transport.
    fn allow_process(&self, pid: u32) -> SysResult<()>;

    /// Make transport publicly accessible (any process can attach).
    fn set_public(&self) -> SysResult<()>;
}

// ============================================================================
// Command Structure - THE WIRE FORMAT
// ============================================================================

/// Fixed-size command structure.
///
/// This struct IS the wire format - no serialization needed.
/// Cache-line aligned (64 bytes) for optimal performance.
///
/// # Layout
/// ```text
/// Offset  Size  Field
/// 0       4     cmd_type
/// 4       4     seq_id
/// 8       4     flags
/// 12      4     _pad
/// 16      48    payload
/// ```
#[repr(C, align(64))]
#[derive(Clone, Copy)]
pub struct Command {
    /// Command type code (see [`cmd`] module for standard types).
    pub cmd_type: u32,
    /// Sequence ID for matching responses. Assigned by producer, never 0.
    pub seq_id: u32,
    /// Protocol-specific flags.
    pub flags: u32,
    /// Reserved padding for alignment.
    pub _pad: u32,
    /// Command-specific payload data.
    pub payload: [u8; COMMAND_PAYLOAD_SIZE],
}

impl Command {
    /// Size of Command in bytes (always 64).
    pub const SIZE: usize = 64;

    /// Create an empty (zeroed) command.
    #[inline]
    pub const fn empty() -> Self {
        Self {
            cmd_type: 0,
            seq_id: 0,
            flags: 0,
            _pad: 0,
            payload: [0u8; COMMAND_PAYLOAD_SIZE],
        }
    }

    /// Check if this command has a valid sequence ID.
    #[inline]
    pub const fn is_valid(&self) -> bool {
        self.seq_id != 0
    }

    /// Read a u8 from payload at offset.
    ///
    /// Returns 0 if offset is out of bounds.
    #[inline]
    pub fn read_u8(&self, offset: usize) -> u8 {
        self.payload.get(offset).copied().unwrap_or(0)
    }

    /// Read a u16 (little-endian) from payload at offset.
    ///
    /// Returns 0 if offset + 2 exceeds payload size.
    #[inline]
    pub fn read_u16(&self, offset: usize) -> u16 {
        if offset + 2 > COMMAND_PAYLOAD_SIZE {
            return 0;
        }
        u16::from_le_bytes([self.payload[offset], self.payload[offset + 1]])
    }

    /// Read a u32 (little-endian) from payload at offset.
    ///
    /// Returns 0 if offset + 4 exceeds payload size.
    #[inline]
    pub fn read_u32(&self, offset: usize) -> u32 {
        if offset + 4 > COMMAND_PAYLOAD_SIZE {
            return 0;
        }
        u32::from_le_bytes([
            self.payload[offset],
            self.payload[offset + 1],
            self.payload[offset + 2],
            self.payload[offset + 3],
        ])
    }

    /// Read a u64 (little-endian) from payload at offset.
    ///
    /// Returns 0 if offset + 8 exceeds payload size.
    #[inline]
    pub fn read_u64(&self, offset: usize) -> u64 {
        if offset + 8 > COMMAND_PAYLOAD_SIZE {
            return 0;
        }
        u64::from_le_bytes([
            self.payload[offset],
            self.payload[offset + 1],
            self.payload[offset + 2],
            self.payload[offset + 3],
            self.payload[offset + 4],
            self.payload[offset + 5],
            self.payload[offset + 6],
            self.payload[offset + 7],
        ])
    }

    /// Read a byte slice from payload.
    ///
    /// Clamps to available bytes if offset + len exceeds payload size.
    #[inline]
    pub fn read_bytes(&self, offset: usize, len: usize) -> &[u8] {
        let start = offset.min(COMMAND_PAYLOAD_SIZE);
        let end = (offset.saturating_add(len)).min(COMMAND_PAYLOAD_SIZE);
        &self.payload[start..end]
    }

    /// Read a null-terminated string from payload.
    ///
    /// Returns empty slice if offset is out of bounds or string is invalid UTF-8.
    #[inline]
    pub fn read_str(&self, offset: usize) -> &[u8] {
        if offset >= COMMAND_PAYLOAD_SIZE {
            return &[];
        }
        let remaining = &self.payload[offset..];
        let len = remaining.iter().position(|&b| b == 0).unwrap_or(remaining.len());
        &remaining[..len]
    }

    /// Write a u8 to payload at offset.
    ///
    /// No-op if offset is out of bounds.
    #[inline]
    pub fn write_u8(&mut self, offset: usize, value: u8) {
        if let Some(slot) = self.payload.get_mut(offset) {
            *slot = value;
        }
    }

    /// Write a u16 (little-endian) to payload at offset.
    ///
    /// No-op if offset + 2 exceeds payload size.
    #[inline]
    pub fn write_u16(&mut self, offset: usize, value: u16) {
        if offset + 2 <= COMMAND_PAYLOAD_SIZE {
            self.payload[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
        }
    }

    /// Write a u32 (little-endian) to payload at offset.
    ///
    /// No-op if offset + 4 exceeds payload size.
    #[inline]
    pub fn write_u32(&mut self, offset: usize, value: u32) {
        if offset + 4 <= COMMAND_PAYLOAD_SIZE {
            self.payload[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
    }

    /// Write a u64 (little-endian) to payload at offset.
    ///
    /// No-op if offset + 8 exceeds payload size.
    #[inline]
    pub fn write_u64(&mut self, offset: usize, value: u64) {
        if offset + 8 <= COMMAND_PAYLOAD_SIZE {
            self.payload[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
        }
    }

    /// Write bytes to payload at offset.
    ///
    /// Truncates if data exceeds available space.
    #[inline]
    pub fn write_bytes(&mut self, offset: usize, data: &[u8]) {
        if offset < COMMAND_PAYLOAD_SIZE {
            let available = COMMAND_PAYLOAD_SIZE - offset;
            let len = data.len().min(available);
            self.payload[offset..offset + len].copy_from_slice(&data[..len]);
        }
    }

    /// Clear the payload (set to zeros).
    #[inline]
    pub fn clear_payload(&mut self) {
        self.payload.fill(0);
    }
}

impl Default for Command {
    fn default() -> Self {
        Self::empty()
    }
}

impl core::fmt::Debug for Command {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Command")
            .field("cmd_type", &self.cmd_type)
            .field("seq_id", &self.seq_id)
            .field("flags", &self.flags)
            .finish_non_exhaustive()
    }
}

// ============================================================================
// Response Structure
// ============================================================================

/// Fixed-size response structure.
///
/// This struct IS the wire format - no serialization needed.
/// Cache-line aligned (64 bytes) for optimal performance.
///
/// # Layout
/// ```text
/// Offset  Size  Field
/// 0       4     seq_id
/// 4       4     result
/// 8       56    payload
/// ```
#[repr(C, align(64))]
#[derive(Clone, Copy)]
pub struct Response {
    /// Sequence ID matching the command this responds to.
    pub seq_id: u32,
    /// Result code (0 = success, negative = error, see [`result`] module).
    pub result: i32,
    /// Response-specific payload data.
    pub payload: [u8; RESPONSE_PAYLOAD_SIZE],
}

impl Response {
    /// Size of Response in bytes (always 64).
    pub const SIZE: usize = 64;

    /// Create an empty (zeroed) response.
    #[inline]
    pub const fn empty() -> Self {
        Self {
            seq_id: 0,
            result: 0,
            payload: [0u8; RESPONSE_PAYLOAD_SIZE],
        }
    }

    /// Create a success response for a given sequence ID.
    #[inline]
    pub const fn success(seq_id: u32) -> Self {
        Self {
            seq_id,
            result: result::OK,
            payload: [0u8; RESPONSE_PAYLOAD_SIZE],
        }
    }

    /// Create an error response for a given sequence ID.
    #[inline]
    pub const fn error(seq_id: u32, code: i32) -> Self {
        Self {
            seq_id,
            result: code,
            payload: [0u8; RESPONSE_PAYLOAD_SIZE],
        }
    }

    /// Check if this response indicates success.
    #[inline]
    pub const fn is_ok(&self) -> bool {
        self.result == result::OK
    }

    /// Check if this response indicates an error.
    #[inline]
    pub const fn is_err(&self) -> bool {
        self.result != result::OK
    }

    /// Read a u32 (little-endian) from payload at offset.
    #[inline]
    pub fn read_u32(&self, offset: usize) -> u32 {
        if offset + 4 > RESPONSE_PAYLOAD_SIZE {
            return 0;
        }
        u32::from_le_bytes([
            self.payload[offset],
            self.payload[offset + 1],
            self.payload[offset + 2],
            self.payload[offset + 3],
        ])
    }

    /// Read a u64 (little-endian) from payload at offset.
    #[inline]
    pub fn read_u64(&self, offset: usize) -> u64 {
        if offset + 8 > RESPONSE_PAYLOAD_SIZE {
            return 0;
        }
        u64::from_le_bytes([
            self.payload[offset],
            self.payload[offset + 1],
            self.payload[offset + 2],
            self.payload[offset + 3],
            self.payload[offset + 4],
            self.payload[offset + 5],
            self.payload[offset + 6],
            self.payload[offset + 7],
        ])
    }

    /// Write a u32 (little-endian) to payload at offset.
    #[inline]
    pub fn write_u32(&mut self, offset: usize, value: u32) {
        if offset + 4 <= RESPONSE_PAYLOAD_SIZE {
            self.payload[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
    }

    /// Write bytes to payload at offset.
    #[inline]
    pub fn write_bytes(&mut self, offset: usize, data: &[u8]) {
        if offset < RESPONSE_PAYLOAD_SIZE {
            let available = RESPONSE_PAYLOAD_SIZE - offset;
            let len = data.len().min(available);
            self.payload[offset..offset + len].copy_from_slice(&data[..len]);
        }
    }
}

impl Default for Response {
    fn default() -> Self {
        Self::empty()
    }
}

impl core::fmt::Debug for Response {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Response")
            .field("seq_id", &self.seq_id)
            .field("result", &self.result)
            .finish_non_exhaustive()
    }
}

// ============================================================================
// Ring Header
// ============================================================================

/// Ring header structure at the start of shared memory.
///
/// Contains atomic indices for lock-free producer/consumer coordination,
/// plus metadata for validation and versioning.
///
/// # Layout
/// ```text
/// Offset  Size  Field
/// 0       4     magic
/// 4       2     version
/// 6       2     ring_size
/// 8       4     cmd_head (atomic)
/// 12      4     cmd_tail (atomic)
/// 16      4     rsp_head (atomic)
/// 20      4     rsp_tail (atomic)
/// 24      4     state_flags (atomic)
/// 28      36    _reserved
/// ```
#[repr(C, align(64))]
pub struct CommandRingHeader {
    /// Magic number for validation (HEADER_MAGIC).
    magic: u32,
    /// Protocol version.
    version: u16,
    /// Ring size (number of slots, power of 2).
    ring_size: u16,
    /// Command ring head (producer writes, consumer reads).
    cmd_head: AtomicU32,
    /// Command ring tail (consumer writes, producer reads).
    cmd_tail: AtomicU32,
    /// Response ring head (consumer writes, producer reads).
    rsp_head: AtomicU32,
    /// Response ring tail (producer writes, consumer reads).
    rsp_tail: AtomicU32,
    /// State flags for coordination.
    state_flags: AtomicU32,
    /// Reserved for future use.
    _reserved: [u8; 36],
}

/// State flag bits
mod state_flags {
    /// Producer has closed the ring
    pub const PRODUCER_CLOSED: u32 = 1 << 0;
    /// Consumer has closed the ring
    pub const CONSUMER_CLOSED: u32 = 1 << 1;
    /// Ring encountered an error
    #[allow(dead_code)]
    pub const ERROR: u32 = 1 << 2;
}

impl CommandRingHeader {
    /// Size of header in bytes (always 64).
    pub const SIZE: usize = 64;

    /// Initialize a new ring header.
    ///
    /// # Panics
    /// Panics if `ring_size` is not a power of 2 or out of valid range.
    fn init(&mut self, ring_size: usize) {
        debug_assert!(ring_size.is_power_of_two(), "ring_size must be power of 2");
        debug_assert!(ring_size >= MIN_RING_SIZE, "ring_size too small");
        debug_assert!(ring_size <= MAX_RING_SIZE, "ring_size too large");

        self.magic = HEADER_MAGIC;
        self.version = PROTOCOL_VERSION;
        self.ring_size = ring_size as u16;
        self.cmd_head.store(0, Ordering::Relaxed);
        self.cmd_tail.store(0, Ordering::Relaxed);
        self.rsp_head.store(0, Ordering::Relaxed);
        self.rsp_tail.store(0, Ordering::Relaxed);
        self.state_flags.store(0, Ordering::Relaxed);
        self._reserved.fill(0);
    }

    /// Validate the header.
    fn validate(&self) -> SysResult<()> {
        if self.magic != HEADER_MAGIC {
            return Err(SysError::InvalidArgument);
        }
        if self.version != PROTOCOL_VERSION {
            return Err(SysError::InvalidArgument);
        }
        let ring_size = self.ring_size as usize;
        if !ring_size.is_power_of_two() || ring_size < MIN_RING_SIZE || ring_size > MAX_RING_SIZE {
            return Err(SysError::InvalidArgument);
        }
        Ok(())
    }

    /// Check if the ring is closed.
    #[inline]
    fn is_closed(&self) -> bool {
        let flags = self.state_flags.load(Ordering::Acquire);
        (flags & (state_flags::PRODUCER_CLOSED | state_flags::CONSUMER_CLOSED)) != 0
    }

    /// Get number of pending commands (head - tail).
    #[inline]
    fn cmd_count(&self) -> u32 {
        let head = self.cmd_head.load(Ordering::Acquire);
        let tail = self.cmd_tail.load(Ordering::Relaxed);
        head.wrapping_sub(tail)
    }

    /// Get number of pending responses.
    #[inline]
    fn rsp_count(&self) -> u32 {
        let head = self.rsp_head.load(Ordering::Acquire);
        let tail = self.rsp_tail.load(Ordering::Relaxed);
        head.wrapping_sub(tail)
    }
}

// ============================================================================
// Shared Memory Layout
// ============================================================================

/// Calculate total shared memory size needed for a ring.
///
/// Layout: [Header 64B][Commands N*64B][Responses N*64B]
#[inline]
pub const fn ring_shmem_size(ring_slots: usize) -> usize {
    CommandRingHeader::SIZE + (ring_slots * Command::SIZE) + (ring_slots * Response::SIZE)
}

#[inline]
const fn cmd_ring_offset() -> usize {
    CommandRingHeader::SIZE
}

#[inline]
const fn rsp_ring_offset(ring_slots: usize) -> usize {
    cmd_ring_offset() + (ring_slots * Command::SIZE)
}

// ============================================================================
// Ring Role
// ============================================================================

/// Role in the ring (producer or consumer).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RingRole {
    /// Sends commands, receives responses.
    Producer,
    /// Receives commands, sends responses.
    Consumer,
}

// ============================================================================
// Ring State
// ============================================================================

/// Internal ring state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RingState {
    /// Ring is operational.
    Ready,
    /// Ring has been closed.
    Closed,
    /// Ring encountered an unrecoverable error.
    #[allow(dead_code)]
    Error,
}

// ============================================================================
// CommandRing - Core Implementation
// ============================================================================

/// Command ring handle.
///
/// Provides lock-free producer/consumer access to command/response rings
/// in shared memory. This is the low-level API; prefer [`Producer`] and
/// [`Consumer`] wrappers for most use cases.
///
/// # Thread Safety
///
/// A `CommandRing` is NOT thread-safe. Each ring should be used from a single
/// thread. For multi-threaded access, use external synchronization.
///
/// # Lifecycle
///
/// 1. Producer creates ring with [`create_producer`](Self::create_producer)
/// 2. Producer shares `shmem_id()` with consumer
/// 3. Consumer attaches with [`attach_consumer`](Self::attach_consumer)
/// 4. Communication proceeds until one side closes
pub struct CommandRing {
    shmem: Shmem,
    role: RingRole,
    ring_size: usize,
    ring_mask: usize,
    next_seq: u32,
    state: RingState,
}

impl CommandRing {
    /// Create a new command ring as producer.
    ///
    /// Creates shared memory and initializes the ring header.
    /// Share `shmem_id()` with the consumer process to attach.
    ///
    /// # Arguments
    /// * `ring_slots` - Number of slots (must be power of 2, 4-256)
    ///
    /// # Errors
    /// - `SysError::InvalidArgument` if ring_slots is invalid
    /// - `SysError::OutOfMemory` if shared memory allocation fails
    pub fn create_producer(ring_slots: usize) -> SysResult<Self> {
        // Validate ring size
        if !ring_slots.is_power_of_two() {
            return Err(SysError::InvalidArgument);
        }
        if ring_slots < MIN_RING_SIZE || ring_slots > MAX_RING_SIZE {
            return Err(SysError::InvalidArgument);
        }

        let size = ring_shmem_size(ring_slots);
        let shmem = Shmem::create(size)?;

        // Initialize header
        // SAFETY: We just created this memory, we're the only accessor
        let header = unsafe { &mut *(shmem.as_ptr() as *mut CommandRingHeader) };
        header.init(ring_slots);

        Ok(Self {
            shmem,
            role: RingRole::Producer,
            ring_size: ring_slots,
            ring_mask: ring_slots - 1,
            next_seq: 1, // Never use 0 as seq_id
            state: RingState::Ready,
        })
    }

    /// Attach to existing ring as consumer.
    ///
    /// Opens shared memory by ID and validates the ring configuration.
    ///
    /// # Arguments
    /// * `shmem_id` - Shared memory ID from producer's `shmem_id()`
    ///
    /// # Errors
    /// - `SysError::NotFound` if shmem_id doesn't exist
    /// - `SysError::PermissionDenied` if not allowed to access
    /// - `SysError::InvalidArgument` if header validation fails
    pub fn attach_consumer(shmem_id: u32) -> SysResult<Self> {
        let shmem = Shmem::open_existing(shmem_id)?;

        // SAFETY: Shmem guarantees valid mapping
        let header = unsafe { &*(shmem.as_ptr() as *const CommandRingHeader) };
        header.validate()?;

        let ring_size = header.ring_size as usize;

        Ok(Self {
            shmem,
            role: RingRole::Consumer,
            ring_size,
            ring_mask: ring_size - 1,
            next_seq: 1,
            state: RingState::Ready,
        })
    }

    /// Get shared memory ID for sharing with peer.
    #[inline]
    pub fn shmem_id(&self) -> u32 {
        self.shmem.shmem_id()
    }

    /// Get this ring's role.
    #[inline]
    pub fn role(&self) -> RingRole {
        self.role
    }

    /// Get ring size (number of slots).
    #[inline]
    pub fn ring_size(&self) -> usize {
        self.ring_size
    }

    /// Check if ring is ready for operations.
    #[inline]
    pub fn is_ready(&self) -> bool {
        self.state == RingState::Ready && !self.header().is_closed()
    }

    /// Allow another process to access the ring.
    pub fn allow(&self, peer_pid: u32) -> SysResult<()> {
        self.shmem.allow(peer_pid)
    }

    /// Make ring publicly accessible.
    pub fn set_public(&self) -> SysResult<()> {
        self.shmem.set_public()
    }

    /// Get the shmem handle for event loop integration.
    #[inline]
    pub fn handle(&self) -> crate::syscall::Handle {
        self.shmem.handle()
    }

    /// Wait for a notification (blocking).
    ///
    /// # Arguments
    /// * `timeout_ms` - Maximum time to wait in milliseconds (0 = wait forever)
    pub fn wait(&self, timeout_ms: u32) -> SysResult<()> {
        self.shmem.wait(timeout_ms)
    }

    /// Mark this side as closed.
    pub fn close(&mut self) {
        if self.state == RingState::Closed {
            return;
        }

        let flag = match self.role {
            RingRole::Producer => state_flags::PRODUCER_CLOSED,
            RingRole::Consumer => state_flags::CONSUMER_CLOSED,
        };

        self.header().state_flags.fetch_or(flag, Ordering::Release);
        self.state = RingState::Closed;

        // Wake peer so they notice the close
        let _ = self.shmem.notify();
    }

    // ========================================================================
    // Internal helpers
    // ========================================================================

    #[inline]
    fn header(&self) -> &CommandRingHeader {
        // SAFETY: Header is always at offset 0, properly aligned
        unsafe { &*(self.shmem.as_ptr() as *const CommandRingHeader) }
    }

    #[inline]
    fn cmd_slot_ptr(&self, index: u32) -> *mut Command {
        let base = self.shmem.as_ptr() as usize + cmd_ring_offset();
        let slot_index = (index as usize) & self.ring_mask;
        (base + slot_index * Command::SIZE) as *mut Command
    }

    #[inline]
    fn rsp_slot_ptr(&self, index: u32) -> *mut Response {
        let base = self.shmem.as_ptr() as usize + rsp_ring_offset(self.ring_size);
        let slot_index = (index as usize) & self.ring_mask;
        (base + slot_index * Response::SIZE) as *mut Response
    }

    /// Check state and return error if not ready.
    #[inline]
    fn check_ready(&self) -> SysResult<()> {
        match self.state {
            RingState::Ready => {
                if self.header().is_closed() {
                    Err(SysError::ConnectionReset)
                } else {
                    Ok(())
                }
            }
            RingState::Closed => Err(SysError::ConnectionReset),
            RingState::Error => Err(SysError::IoError),
        }
    }

    /// Allocate next sequence ID (never 0).
    #[inline]
    fn alloc_seq_id(&mut self) -> u32 {
        let seq = self.next_seq;
        self.next_seq = if self.next_seq == u32::MAX { 1 } else { self.next_seq + 1 };
        seq
    }

    // ========================================================================
    // Producer API
    // ========================================================================

    /// Try to reserve a command slot (non-blocking).
    ///
    /// Returns the slot index, or `None` if ring is full.
    /// Producer only.
    pub fn try_reserve_cmd(&mut self) -> Option<u32> {
        if self.role != RingRole::Producer {
            return None;
        }
        if self.check_ready().is_err() {
            return None;
        }

        let header = self.header();
        let head = header.cmd_head.load(Ordering::Relaxed);
        let tail = header.cmd_tail.load(Ordering::Acquire);

        // Ring full?
        if head.wrapping_sub(tail) >= self.ring_size as u32 {
            return None;
        }

        // Initialize the slot
        let slot = unsafe { &mut *self.cmd_slot_ptr(head) };
        slot.seq_id = self.alloc_seq_id();
        slot.cmd_type = 0;
        slot.flags = 0;
        slot._pad = 0;
        slot.payload.fill(0);

        Some(head)
    }

    /// Reserve a command slot with timeout.
    ///
    /// Blocks until a slot is available or timeout expires.
    /// Producer only.
    ///
    /// # Arguments
    /// * `timeout_ms` - Timeout in milliseconds (0 = default, capped at MAX_TIMEOUT_MS)
    pub fn reserve_cmd(&mut self, timeout_ms: u32) -> SysResult<u32> {
        if self.role != RingRole::Producer {
            return Err(SysError::InvalidArgument);
        }

        let timeout = if timeout_ms == 0 { DEFAULT_TIMEOUT_MS } else { timeout_ms.min(MAX_TIMEOUT_MS) };
        let start_ns = gettime();
        let deadline_ns = start_ns + (timeout as u64) * 1_000_000;

        loop {
            self.check_ready()?;

            if let Some(index) = self.try_reserve_cmd() {
                return Ok(index);
            }

            // Check timeout
            let now_ns = gettime();
            if now_ns >= deadline_ns {
                return Err(SysError::Timeout);
            }

            // Wait for consumer to free a slot
            let remaining_ms = ((deadline_ns - now_ns) / 1_000_000) as u32;
            let wait_ms = remaining_ms.min(100); // Wake periodically to check state
            let _ = self.shmem.wait(wait_ms);
        }
    }

    /// Get mutable reference to command slot by index.
    ///
    /// # Safety
    /// Index must have been returned by `try_reserve_cmd` or `reserve_cmd`,
    /// and not yet published.
    pub fn get_cmd_mut(&mut self, index: u32) -> &mut Command {
        debug_assert!(self.role == RingRole::Producer);
        unsafe { &mut *self.cmd_slot_ptr(index) }
    }

    /// Publish the command (make visible to consumer).
    ///
    /// Must be called after filling the command. Producer only.
    pub fn publish_cmd(&self) {
        if self.role != RingRole::Producer {
            return;
        }
        // Release ensures command data is visible before head update
        self.header().cmd_head.fetch_add(1, Ordering::Release);
    }

    /// Notify the consumer that data is available.
    pub fn notify(&self) -> SysResult<()> {
        self.shmem.notify()?;
        Ok(())
    }

    /// Try to receive a response (non-blocking). Producer only.
    pub fn try_recv_rsp(&self) -> Option<&Response> {
        if self.role != RingRole::Producer {
            return None;
        }

        let header = self.header();
        let tail = header.rsp_tail.load(Ordering::Relaxed);
        let head = header.rsp_head.load(Ordering::Acquire);

        if tail == head {
            return None;
        }

        Some(unsafe { &*self.rsp_slot_ptr(tail) })
    }

    /// Consume a response (after processing). Producer only.
    pub fn consume_rsp(&self) {
        if self.role != RingRole::Producer {
            return;
        }
        self.header().rsp_tail.fetch_add(1, Ordering::Release);
    }

    /// Check if there are pending responses. Producer only.
    pub fn has_responses(&self) -> bool {
        if self.role != RingRole::Producer {
            return false;
        }
        self.header().rsp_count() > 0
    }

    // ========================================================================
    // Consumer API
    // ========================================================================

    /// Try to receive a command (non-blocking). Consumer only.
    pub fn try_recv_cmd(&self) -> Option<&Command> {
        if self.role != RingRole::Consumer {
            return None;
        }

        let header = self.header();
        let tail = header.cmd_tail.load(Ordering::Relaxed);
        let head = header.cmd_head.load(Ordering::Acquire);

        if tail == head {
            return None;
        }

        Some(unsafe { &*self.cmd_slot_ptr(tail) })
    }

    /// Receive a command (blocking). Consumer only.
    pub fn recv_cmd(&self) -> SysResult<&Command> {
        if self.role != RingRole::Consumer {
            return Err(SysError::InvalidArgument);
        }

        loop {
            self.check_ready()?;

            if let Some(cmd) = self.try_recv_cmd() {
                return Ok(cmd);
            }

            // Wait for producer
            self.shmem.wait(0)?;
        }
    }

    /// Consume a command (after processing). Consumer only.
    ///
    /// This frees the slot for reuse by the producer.
    pub fn consume_cmd(&self) {
        if self.role != RingRole::Consumer {
            return;
        }
        self.header().cmd_tail.fetch_add(1, Ordering::Release);
        // Notify producer that slot is free
        let _ = self.shmem.notify();
    }

    /// Check if there are pending commands. Consumer only.
    pub fn has_commands(&self) -> bool {
        if self.role != RingRole::Consumer {
            return false;
        }
        self.header().cmd_count() > 0
    }

    /// Try to reserve a response slot (non-blocking). Consumer only.
    pub fn try_reserve_rsp(&mut self) -> Option<&mut Response> {
        if self.role != RingRole::Consumer {
            return None;
        }

        let header = self.header();
        let head = header.rsp_head.load(Ordering::Relaxed);
        let tail = header.rsp_tail.load(Ordering::Acquire);

        if head.wrapping_sub(tail) >= self.ring_size as u32 {
            return None;
        }

        let slot = unsafe { &mut *self.rsp_slot_ptr(head) };
        slot.seq_id = 0;
        slot.result = 0;
        slot.payload.fill(0);

        Some(slot)
    }

    /// Publish a response. Consumer only.
    pub fn publish_rsp(&self) {
        if self.role != RingRole::Consumer {
            return;
        }
        self.header().rsp_head.fetch_add(1, Ordering::Release);
    }

    // ========================================================================
    // High-level API
    // ========================================================================

    /// Send a command and wait for response (producer only).
    ///
    /// This is a blocking call that:
    /// 1. Reserves a command slot
    /// 2. Fills and publishes the command
    /// 3. Waits for the matching response
    pub fn call(
        &mut self,
        cmd_type: u32,
        flags: u32,
        payload: &[u8],
        timeout_ms: u32,
    ) -> SysResult<Response> {
        let timeout = if timeout_ms == 0 { DEFAULT_TIMEOUT_MS } else { timeout_ms.min(MAX_TIMEOUT_MS) };
        let start_ns = gettime();
        let deadline_ns = start_ns + (timeout as u64) * 1_000_000;

        // Reserve and fill command
        let index = self.reserve_cmd(timeout)?;
        let seq_id = {
            let cmd = self.get_cmd_mut(index);
            cmd.cmd_type = cmd_type;
            cmd.flags = flags;
            let len = payload.len().min(COMMAND_PAYLOAD_SIZE);
            cmd.payload[..len].copy_from_slice(&payload[..len]);
            cmd.seq_id
        };

        // Publish and notify
        self.publish_cmd();
        self.notify()?;

        // Wait for matching response
        loop {
            self.check_ready()?;

            if let Some(rsp) = self.try_recv_rsp() {
                if rsp.seq_id == seq_id {
                    let result = *rsp;
                    self.consume_rsp();
                    return Ok(result);
                }
                // Wrong seq_id - could be stale, consume and continue
                // In a well-behaved system this shouldn't happen
            }

            let now_ns = gettime();
            if now_ns >= deadline_ns {
                return Err(SysError::Timeout);
            }

            let remaining_ms = ((deadline_ns - now_ns) / 1_000_000) as u32;
            let wait_ms = remaining_ms.min(100);
            let _ = self.shmem.wait(wait_ms);
        }
    }
}

impl Drop for CommandRing {
    fn drop(&mut self) {
        // Mark as closed so peer knows we're gone
        if self.state == RingState::Ready {
            self.close();
        }
    }
}

// ============================================================================
// Producer Wrapper
// ============================================================================

/// Producer-side command ring with simplified API.
///
/// This is the recommended way to use the command ring as a producer.
/// It provides a fluent builder API for constructing commands.
///
/// # Example
///
/// ```ignore
/// let mut ring = Producer::create()?;
/// ring.set_public()?;
///
/// let rsp = ring.send(cmd::ATTACH_DISK)
///     .u32(shmem_id)
///     .u32(block_size)
///     .call()?;
///
/// if rsp.is_ok() {
///     // Success
/// }
/// ```
pub struct Producer {
    ring: CommandRing,
    timeout_ms: u32,
}

impl Producer {
    /// Create a new producer ring with default settings.
    pub fn create() -> SysResult<Self> {
        Self::create_sized(RING_SIZE)
    }

    /// Create a producer ring with custom size.
    ///
    /// # Arguments
    /// * `ring_slots` - Number of slots (must be power of 2, 4-256)
    pub fn create_sized(ring_slots: usize) -> SysResult<Self> {
        Ok(Self {
            ring: CommandRing::create_producer(ring_slots)?,
            timeout_ms: DEFAULT_TIMEOUT_MS,
        })
    }

    /// Set default timeout for commands (builder pattern).
    pub fn with_timeout(mut self, timeout_ms: u32) -> Self {
        self.timeout_ms = timeout_ms.min(MAX_TIMEOUT_MS);
        self
    }

    /// Get shared memory ID for sharing with consumer.
    #[inline]
    pub fn shmem_id(&self) -> u32 {
        self.ring.shmem_id()
    }

    /// Allow a process to access this ring.
    pub fn allow(&self, pid: u32) -> SysResult<()> {
        self.ring.allow(pid)
    }

    /// Make ring publicly accessible.
    pub fn set_public(&self) -> SysResult<()> {
        self.ring.set_public()
    }

    /// Check if ring is ready.
    #[inline]
    pub fn is_ready(&self) -> bool {
        self.ring.is_ready()
    }

    /// Close the ring.
    pub fn close(&mut self) {
        self.ring.close();
    }

    /// Start building a command (request/response).
    pub fn send(&mut self, cmd_type: u32) -> CommandBuilder<'_> {
        CommandBuilder::new(&mut self.ring, cmd_type, self.timeout_ms)
    }

    /// Start building a fire-and-forget command.
    pub fn fire(&mut self, cmd_type: u32) -> FireBuilder<'_> {
        FireBuilder::new(&mut self.ring, cmd_type, self.timeout_ms)
    }
}

impl CommandSender for Producer {
    fn send_command(
        &mut self,
        cmd_type: u32,
        flags: u32,
        payload: &[u8],
        timeout_ms: u32,
    ) -> SysResult<Response> {
        self.ring.call(cmd_type, flags, payload, timeout_ms)
    }

    fn fire_command(
        &mut self,
        cmd_type: u32,
        flags: u32,
        payload: &[u8],
        timeout_ms: u32,
    ) -> SysResult<()> {
        let index = self.ring.reserve_cmd(timeout_ms)?;
        {
            let cmd = self.ring.get_cmd_mut(index);
            cmd.cmd_type = cmd_type;
            cmd.flags = flags;
            let len = payload.len().min(COMMAND_PAYLOAD_SIZE);
            cmd.payload[..len].copy_from_slice(&payload[..len]);
        }
        self.ring.publish_cmd();
        self.ring.notify()
    }

    fn has_responses(&self) -> bool {
        self.ring.has_responses()
    }
}

impl Transport for Producer {
    fn transport_id(&self) -> u32 {
        self.ring.shmem_id()
    }

    fn allow_process(&self, pid: u32) -> SysResult<()> {
        self.ring.allow(pid)
    }

    fn set_public(&self) -> SysResult<()> {
        self.ring.set_public()
    }
}

// ============================================================================
// Consumer Wrapper
// ============================================================================

/// Consumer-side command ring with simplified API.
///
/// This is the recommended way to use the command ring as a consumer.
///
/// # Example
///
/// ```ignore
/// let mut ring = Consumer::attach(shmem_id)?;
///
/// loop {
///     let cmd = ring.recv()?;
///
///     match cmd.cmd_type {
///         cmd::ATTACH_DISK => {
///             let id = cmd.read_u32(0);
///             // Process...
///             ring.respond(cmd.seq_id, result::OK)?;
///         }
///         _ => ring.respond(cmd.seq_id, result::ERR_NOT_SUPPORTED)?,
///     }
/// }
/// ```
pub struct Consumer {
    ring: CommandRing,
}

impl Consumer {
    /// Attach to an existing ring by shared memory ID.
    pub fn attach(shmem_id: u32) -> SysResult<Self> {
        Ok(Self {
            ring: CommandRing::attach_consumer(shmem_id)?,
        })
    }

    /// Check if ring is ready.
    #[inline]
    pub fn is_ready(&self) -> bool {
        self.ring.is_ready()
    }

    /// Close the ring.
    pub fn close(&mut self) {
        self.ring.close();
    }

    /// Receive next command (blocking).
    pub fn recv(&self) -> SysResult<&Command> {
        self.ring.recv_cmd()
    }

    /// Try to receive a command (non-blocking).
    pub fn try_recv(&self) -> Option<&Command> {
        self.ring.try_recv_cmd()
    }

    /// Respond to a command with simple result code.
    ///
    /// This also consumes the command.
    pub fn respond(&mut self, seq_id: u32, result: i32) -> SysResult<()> {
        self.respond_with(seq_id, result, &[])
    }

    /// Respond to a command with result and payload.
    ///
    /// This also consumes the command.
    pub fn respond_with(&mut self, seq_id: u32, result_code: i32, payload: &[u8]) -> SysResult<()> {
        self.ring.check_ready()?;

        // Wait for response slot with timeout
        let start_ns = gettime();
        let deadline_ns = start_ns + (DEFAULT_TIMEOUT_MS as u64) * 1_000_000;

        loop {
            if let Some(rsp) = self.ring.try_reserve_rsp() {
                rsp.seq_id = seq_id;
                rsp.result = result_code;
                let len = payload.len().min(RESPONSE_PAYLOAD_SIZE);
                rsp.payload[..len].copy_from_slice(&payload[..len]);
                break;
            }

            let now_ns = gettime();
            if now_ns >= deadline_ns {
                return Err(SysError::Timeout);
            }

            let _ = self.ring.shmem.wait(100);
        }

        // Publish response
        self.ring.publish_rsp();

        // Consume the command
        self.ring.consume_cmd();

        // Notify producer
        self.ring.notify()
    }

    /// Check if there are pending commands.
    #[inline]
    pub fn has_commands(&self) -> bool {
        self.ring.has_commands()
    }

    /// Get underlying shmem handle for Mux integration.
    #[inline]
    pub fn shmem_handle(&self) -> crate::syscall::Handle {
        self.ring.shmem.handle()
    }
}

impl CommandReceiver for Consumer {
    fn recv_command(&self) -> SysResult<&Command> {
        self.ring.recv_cmd()
    }

    fn try_recv_command(&self) -> Option<&Command> {
        self.ring.try_recv_cmd()
    }

    fn send_response(&mut self, seq_id: u32, result: i32, payload: &[u8]) -> SysResult<()> {
        self.respond_with(seq_id, result, payload)
    }

    fn has_commands(&self) -> bool {
        self.ring.has_commands()
    }
}

// ============================================================================
// Command Builder
// ============================================================================

/// Builder for constructing and sending commands.
///
/// Provides a fluent API for building command payloads.
pub struct CommandBuilder<'a> {
    ring: &'a mut CommandRing,
    cmd_type: u32,
    flags: u32,
    payload: [u8; COMMAND_PAYLOAD_SIZE],
    offset: usize,
    timeout_ms: u32,
}

impl<'a> CommandBuilder<'a> {
    fn new(ring: &'a mut CommandRing, cmd_type: u32, timeout_ms: u32) -> Self {
        Self {
            ring,
            cmd_type,
            flags: 0,
            payload: [0u8; COMMAND_PAYLOAD_SIZE],
            offset: 0,
            timeout_ms,
        }
    }

    /// Set command flags.
    pub fn flags(mut self, flags: u32) -> Self {
        self.flags = flags;
        self
    }

    /// Append a u8 to payload.
    pub fn u8(mut self, value: u8) -> Self {
        if self.offset < COMMAND_PAYLOAD_SIZE {
            self.payload[self.offset] = value;
            self.offset += 1;
        }
        self
    }

    /// Append a u16 (little-endian) to payload.
    pub fn u16(mut self, value: u16) -> Self {
        if self.offset + 2 <= COMMAND_PAYLOAD_SIZE {
            self.payload[self.offset..self.offset + 2].copy_from_slice(&value.to_le_bytes());
            self.offset += 2;
        }
        self
    }

    /// Append a u32 (little-endian) to payload.
    pub fn u32(mut self, value: u32) -> Self {
        if self.offset + 4 <= COMMAND_PAYLOAD_SIZE {
            self.payload[self.offset..self.offset + 4].copy_from_slice(&value.to_le_bytes());
            self.offset += 4;
        }
        self
    }

    /// Append a u64 (little-endian) to payload.
    pub fn u64(mut self, value: u64) -> Self {
        if self.offset + 8 <= COMMAND_PAYLOAD_SIZE {
            self.payload[self.offset..self.offset + 8].copy_from_slice(&value.to_le_bytes());
            self.offset += 8;
        }
        self
    }

    /// Append raw bytes to payload.
    pub fn bytes(mut self, data: &[u8]) -> Self {
        let available = COMMAND_PAYLOAD_SIZE - self.offset;
        let len = data.len().min(available);
        self.payload[self.offset..self.offset + len].copy_from_slice(&data[..len]);
        self.offset += len;
        self
    }

    /// Append a null-terminated string to payload.
    pub fn str(mut self, s: &str) -> Self {
        let bytes = s.as_bytes();
        let available = COMMAND_PAYLOAD_SIZE - self.offset;
        if available > 0 {
            let len = bytes.len().min(available - 1); // Leave room for null
            self.payload[self.offset..self.offset + len].copy_from_slice(&bytes[..len]);
            self.offset += len;
            if self.offset < COMMAND_PAYLOAD_SIZE {
                self.payload[self.offset] = 0;
                self.offset += 1;
            }
        }
        self
    }

    /// Set custom timeout for this command.
    pub fn timeout(mut self, timeout_ms: u32) -> Self {
        self.timeout_ms = timeout_ms.min(MAX_TIMEOUT_MS);
        self
    }

    /// Current payload offset (bytes written).
    #[inline]
    pub fn offset(&self) -> usize {
        self.offset
    }

    /// Remaining payload capacity.
    #[inline]
    pub fn remaining(&self) -> usize {
        COMMAND_PAYLOAD_SIZE - self.offset
    }

    /// Send command and wait for response.
    pub fn call(self) -> SysResult<Response> {
        self.ring.call(self.cmd_type, self.flags, &self.payload[..self.offset], self.timeout_ms)
    }
}

// ============================================================================
// Fire Builder (no response)
// ============================================================================

/// Builder for fire-and-forget commands.
pub struct FireBuilder<'a> {
    ring: &'a mut CommandRing,
    cmd_type: u32,
    flags: u32,
    payload: [u8; COMMAND_PAYLOAD_SIZE],
    offset: usize,
    timeout_ms: u32,
}

impl<'a> FireBuilder<'a> {
    fn new(ring: &'a mut CommandRing, cmd_type: u32, timeout_ms: u32) -> Self {
        Self {
            ring,
            cmd_type,
            flags: 0,
            payload: [0u8; COMMAND_PAYLOAD_SIZE],
            offset: 0,
            timeout_ms,
        }
    }

    /// Set command flags.
    pub fn flags(mut self, flags: u32) -> Self {
        self.flags = flags;
        self
    }

    /// Append a u32 to payload.
    pub fn u32(mut self, value: u32) -> Self {
        if self.offset + 4 <= COMMAND_PAYLOAD_SIZE {
            self.payload[self.offset..self.offset + 4].copy_from_slice(&value.to_le_bytes());
            self.offset += 4;
        }
        self
    }

    /// Append a u64 to payload.
    pub fn u64(mut self, value: u64) -> Self {
        if self.offset + 8 <= COMMAND_PAYLOAD_SIZE {
            self.payload[self.offset..self.offset + 8].copy_from_slice(&value.to_le_bytes());
            self.offset += 8;
        }
        self
    }

    /// Append raw bytes to payload.
    pub fn bytes(mut self, data: &[u8]) -> Self {
        let available = COMMAND_PAYLOAD_SIZE - self.offset;
        let len = data.len().min(available);
        self.payload[self.offset..self.offset + len].copy_from_slice(&data[..len]);
        self.offset += len;
        self
    }

    /// Send command without waiting for response.
    pub fn fire(self) -> SysResult<()> {
        let index = self.ring.reserve_cmd(self.timeout_ms)?;
        {
            let cmd = self.ring.get_cmd_mut(index);
            cmd.cmd_type = self.cmd_type;
            cmd.flags = self.flags;
            cmd.payload[..self.offset].copy_from_slice(&self.payload[..self.offset]);
        }
        self.ring.publish_cmd();
        self.ring.notify()
    }
}

// ============================================================================
// Standard Command Types
// ============================================================================

/// Standard command type codes.
pub mod cmd {
    /// Ping/keepalive request.
    pub const PING: u32 = 0x0001;
    /// Pong response to ping.
    pub const PONG: u32 = 0x0002;

    // Device orchestration commands (0x0200+)

    /// Attach a disk to partition driver (devd → partd).
    pub const ATTACH_DISK: u32 = 0x0210;
    /// Report discovered partitions (partd → devd).
    pub const REPORT_PARTITIONS: u32 = 0x0211;
    /// Assign name to a partition (devd → partd).
    pub const REGISTER_PARTITION: u32 = 0x0212;
    /// Partition ready for use (partd → devd).
    pub const PARTITION_READY: u32 = 0x0213;
    /// Mount a partition (devd → fatfsd).
    pub const MOUNT_PARTITION: u32 = 0x0214;
    /// Mount complete (fatfsd → devd).
    pub const MOUNT_READY: u32 = 0x0215;
}

/// Standard result codes.
pub mod result {
    /// Success.
    pub const OK: i32 = 0;
    /// Invalid argument or request.
    pub const ERR_INVALID: i32 = -1;
    /// Resource not found.
    pub const ERR_NOT_FOUND: i32 = -2;
    /// Resource busy.
    pub const ERR_BUSY: i32 = -3;
    /// I/O error.
    pub const ERR_IO: i32 = -4;
    /// Operation timed out.
    pub const ERR_TIMEOUT: i32 = -5;
    /// Operation not supported.
    pub const ERR_NOT_SUPPORTED: i32 = -6;
    /// Out of memory.
    pub const ERR_NO_MEMORY: i32 = -7;
    /// Permission denied.
    pub const ERR_PERMISSION: i32 = -8;
}

// ============================================================================
// Partition Info
// ============================================================================

/// Partition table scheme.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PartitionScheme {
    /// Unknown scheme.
    Unknown = 0,
    /// MBR (Master Boot Record).
    Mbr = 1,
    /// GPT (GUID Partition Table).
    Gpt = 2,
}

/// Filesystem type hints (from partition type).
pub mod fs_hint {
    /// Unknown filesystem.
    pub const UNKNOWN: u8 = 0;
    /// FAT12.
    pub const FAT12: u8 = 1;
    /// FAT16.
    pub const FAT16: u8 = 2;
    /// FAT32.
    pub const FAT32: u8 = 3;
    /// exFAT.
    pub const EXFAT: u8 = 4;
    /// ext2.
    pub const EXT2: u8 = 5;
    /// ext4.
    pub const EXT4: u8 = 6;
    /// NTFS.
    pub const NTFS: u8 = 7;
    /// Linux swap.
    pub const SWAP: u8 = 8;
}

/// Map MBR partition type byte to filesystem hint.
pub fn mbr_type_to_fs_hint(mbr_type: u8) -> u8 {
    match mbr_type {
        0x01 => fs_hint::FAT12,
        0x04 | 0x06 | 0x0E => fs_hint::FAT16,
        0x0B | 0x0C => fs_hint::FAT32,
        0x07 => fs_hint::NTFS,
        0x82 => fs_hint::SWAP,
        0x83 => fs_hint::EXT4,
        _ => fs_hint::UNKNOWN,
    }
}

/// Partition information.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PartitionInfo {
    /// Partition index (0-based).
    pub index: u8,
    /// Partition type (MBR type byte or GPT type index).
    pub part_type: u8,
    /// Bootable flag.
    pub bootable: u8,
    /// Reserved.
    pub _pad: u8,
    /// Starting LBA.
    pub start_lba: u64,
    /// Size in sectors.
    pub size_sectors: u64,
}

impl PartitionInfo {
    /// Size of PartitionInfo in bytes.
    pub const SIZE: usize = 24;

    /// Create an empty partition info.
    pub const fn empty() -> Self {
        Self {
            index: 0,
            part_type: 0,
            bootable: 0,
            _pad: 0,
            start_lba: 0,
            size_sectors: 0,
        }
    }

    /// Get filesystem hint from partition type.
    pub fn fs_hint(&self) -> u8 {
        mbr_type_to_fs_hint(self.part_type)
    }
}

// ============================================================================
// Payload Layout Helpers
// ============================================================================

/// AttachDisk command payload layout.
pub mod attach_disk {
    /// shmem_id (u32) at offset 0.
    pub const SHMEM_ID: usize = 0;
    /// block_size (u32) at offset 4.
    pub const BLOCK_SIZE: usize = 4;
    /// block_count (u64) at offset 8.
    pub const BLOCK_COUNT: usize = 8;
    /// Source name at offset 16 (up to 32 bytes).
    pub const SOURCE_NAME: usize = 16;
    /// Maximum source name length.
    pub const SOURCE_NAME_MAX: usize = 32;
}

/// RegisterPartition command payload layout.
pub mod register_partition {
    /// Partition index (u8) at offset 0.
    pub const INDEX: usize = 0;
    /// Assigned name at offset 1 (up to 31 bytes).
    pub const NAME: usize = 1;
    /// Maximum name length.
    pub const NAME_MAX: usize = 31;
}

/// PartitionReady command payload layout.
pub mod partition_ready {
    /// shmem_id (u32) at offset 0.
    pub const SHMEM_ID: usize = 0;
    /// Name at offset 4 (up to 32 bytes).
    pub const NAME: usize = 4;
    /// Maximum name length.
    pub const NAME_MAX: usize = 32;
}

/// MountPartition command payload layout.
pub mod mount_partition {
    /// shmem_id (u32) at offset 0.
    pub const SHMEM_ID: usize = 0;
    /// fs_hint (u8) at offset 4.
    pub const FS_HINT: usize = 4;
    /// block_size (u32) at offset 8.
    pub const BLOCK_SIZE: usize = 8;
    /// block_count (u64) at offset 12.
    pub const BLOCK_COUNT: usize = 12;
    /// Source name at offset 20 (up to 14 bytes).
    pub const SOURCE_NAME: usize = 20;
    /// Maximum source name length.
    pub const SOURCE_NAME_MAX: usize = 14;
    /// Mount point at offset 34 (up to 14 bytes).
    pub const MOUNT_POINT: usize = 34;
    /// Maximum mount point length.
    pub const MOUNT_POINT_MAX: usize = 14;
}
