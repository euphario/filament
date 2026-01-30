//! Shared Ring Buffer Library
//!
//! Provides lock-free single-producer single-consumer ring buffers
//! built on top of shared memory primitives. Used for high-performance
//! IPC between userspace drivers.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                    Shared Memory Region                     │
//! ├─────────────────────────────────────────────────────────────┤
//! │  RingHeader (64 bytes, cache-line aligned)                  │
//! │    magic, version, entry_size, entry_count                  │
//! │    sq_head, sq_tail (submission queue indices)              │
//! │    cq_head, cq_tail (completion queue indices)              │
//! ├─────────────────────────────────────────────────────────────┤
//! │  Submission Queue Entries [entry_count]                     │
//! ├─────────────────────────────────────────────────────────────┤
//! │  Completion Queue Entries [entry_count]                     │
//! ├─────────────────────────────────────────────────────────────┤
//! │  Data Buffer (for bulk data transfer)                       │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```no_run
//! // Server side (creates the ring)
//! let ring = Ring::<BlockRequest, BlockResponse>::create(64, 1024 * 1024)?;
//! ring.allow(client_pid)?;
//! // Send shmem_id to client via existing IPC
//!
//! // Client side (maps the ring)
//! let ring = Ring::<BlockRequest, BlockResponse>::map(shmem_id)?;
//!
//! // Client submits request
//! ring.submit(&BlockRequest { lba: 0, count: 1, buf_offset: 0 })?;
//! ring.notify();
//!
//! // Server processes
//! ring.wait(0);
//! while let Some(req) = ring.next_request() {
//!     // Process request, DMA to data buffer
//!     ring.complete(&BlockResponse { status: 0, bytes: 512 });
//! }
//! ring.notify();
//! ```

use core::sync::atomic::{AtomicU32, Ordering};
use core::marker::PhantomData;

// ============================================================================
// Cache management for shared memory coherency
// ============================================================================

/// Data synchronization barrier
#[inline]
fn dsb() {
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// Flush (clean) a cache line to memory
#[inline]
fn flush_cache_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc cvac, {}", in(reg) addr, options(nostack, preserves_flags));
    }
}

/// Invalidate a cache line
#[inline]
fn invalidate_cache_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc civac, {}", in(reg) addr, options(nostack, preserves_flags));
    }
}

/// Flush a buffer to memory (after CPU writes, before other process reads)
#[inline]
fn flush_buffer(addr: u64, size: usize) {
    const CACHE_LINE: usize = 64;
    let start = addr & !(CACHE_LINE as u64 - 1);
    let end = (addr + size as u64 + CACHE_LINE as u64 - 1) & !(CACHE_LINE as u64 - 1);
    let mut a = start;
    while a < end {
        flush_cache_line(a);
        a += CACHE_LINE as u64;
    }
    dsb();
}

/// Invalidate a buffer from cache (before CPU reads data written by other process)
#[inline]
fn invalidate_buffer(addr: u64, size: usize) {
    const CACHE_LINE: usize = 64;
    let start = addr & !(CACHE_LINE as u64 - 1);
    let end = (addr + size as u64 + CACHE_LINE as u64 - 1) & !(CACHE_LINE as u64 - 1);
    dsb();
    let mut a = start;
    while a < end {
        invalidate_cache_line(a);
        a += CACHE_LINE as u64;
    }
    dsb();
}

/// Ring buffer magic number
pub const RING_MAGIC: u32 = 0x52494E47; // "RING"

/// Ring buffer version
pub const RING_VERSION: u32 = 1;

/// Default entry count (power of 2)
pub const DEFAULT_ENTRY_COUNT: u32 = 64;

/// Ring header - shared between producer and consumer
/// Aligned to cache line (64 bytes) to avoid false sharing
#[repr(C, align(64))]
pub struct RingHeader {
    /// Magic number for validation
    pub magic: u32,
    /// Version number
    pub version: u32,
    /// Size of each entry in bytes
    pub entry_size: u32,
    /// Number of entries (power of 2)
    pub entry_count: u32,
    /// Size of data buffer in bytes
    pub data_size: u32,
    /// Offset to submission queue from start of region
    pub sq_offset: u32,
    /// Offset to completion queue from start of region
    pub cq_offset: u32,
    /// Offset to data buffer from start of region
    pub data_offset: u32,

    // Submission queue indices (producer: client, consumer: server)
    /// SQ head - updated by consumer (server) after processing
    pub sq_head: AtomicU32,
    /// SQ tail - updated by producer (client) after submitting
    pub sq_tail: AtomicU32,

    // Completion queue indices (producer: server, consumer: client)
    /// CQ head - updated by consumer (client) after reading
    pub cq_head: AtomicU32,
    /// CQ tail - updated by producer (server) after completing
    pub cq_tail: AtomicU32,

    /// Padding to 64 bytes
    _padding: [u32; 4],
}

impl RingHeader {
    /// Check if the header is valid
    pub fn is_valid(&self) -> bool {
        self.magic == RING_MAGIC &&
        self.version == RING_VERSION &&
        self.is_power_of_two()
    }

    /// Check if entry_count is a power of 2 (required for mask() to work)
    fn is_power_of_two(&self) -> bool {
        self.entry_count > 0 && (self.entry_count & (self.entry_count - 1)) == 0
    }

    /// Get the mask for index wrapping (entry_count - 1)
    /// SAFETY: Only valid if entry_count is power of 2 (checked by is_valid)
    pub fn mask(&self) -> u32 {
        self.entry_count - 1
    }
}

/// Shared ring buffer
pub struct Ring<S, C> {
    /// Shared memory wrapper (handles lifecycle)
    shmem: crate::ipc::Shmem,
    /// Phantom data for request/response types
    _marker: PhantomData<(S, C)>,
}

// Ring is Send because we own the memory exclusively per side
unsafe impl<S, C> Send for Ring<S, C> {}

impl<S: Copy, C: Copy> Ring<S, C> {
    /// Create a new ring buffer
    ///
    /// # Arguments
    /// * `entry_count` - Number of entries (will be rounded to power of 2)
    /// * `data_size` - Size of the data buffer in bytes
    ///
    /// Returns None if parameters would cause overflow or allocation fails.
    pub fn create(entry_count: u32, data_size: usize) -> Option<Self> {
        // Validate entry_count won't overflow when rounded up
        // next_power_of_two() returns 0 on overflow for u32
        let entry_count = entry_count.checked_next_power_of_two()?;

        // Calculate sizes with overflow checks
        let header_size = core::mem::size_of::<RingHeader>();
        let sq_size = (entry_count as usize).checked_mul(core::mem::size_of::<S>())?;
        let cq_size = (entry_count as usize).checked_mul(core::mem::size_of::<C>())?;

        // Align each section to cache line (with overflow checks)
        let sq_offset = header_size.checked_add(63)? & !63;
        let cq_offset = sq_offset.checked_add(sq_size)?.checked_add(63)? & !63;
        let data_offset = cq_offset.checked_add(cq_size)?.checked_add(63)? & !63;
        let total_size = data_offset.checked_add(data_size)?;

        // Create shared memory region
        let shmem = crate::ipc::Shmem::create(total_size).ok()?;
        let base = shmem.as_ptr();

        // Initialize header
        unsafe {
            let header = base as *mut RingHeader;
            (*header).magic = RING_MAGIC;
            (*header).version = RING_VERSION;
            (*header).entry_size = core::mem::size_of::<S>() as u32;
            (*header).entry_count = entry_count;
            (*header).data_size = data_size as u32;
            (*header).sq_offset = sq_offset as u32;
            (*header).cq_offset = cq_offset as u32;
            (*header).data_offset = data_offset as u32;
            (*header).sq_head = AtomicU32::new(0);
            (*header).sq_tail = AtomicU32::new(0);
            (*header).cq_head = AtomicU32::new(0);
            (*header).cq_tail = AtomicU32::new(0);
            (*header)._padding = [0; 4];
        }

        // No cache flush needed - shmem uses non-cacheable memory

        Some(Self {
            shmem,
            _marker: PhantomData,
        })
    }

    /// Map an existing ring buffer
    pub fn map(shmem_id: u32) -> Option<Self> {
        let shmem = crate::ipc::Shmem::open_existing(shmem_id).ok()?;
        let base = shmem.as_ptr();

        // No cache invalidation needed - shmem uses non-cacheable memory

        // Validate header
        unsafe {
            let header = base as *const RingHeader;
            if !(*header).is_valid() {
                return None;
            }

            Some(Self {
                shmem,
                _marker: PhantomData,
            })
        }
    }

    /// Get the shared memory ID (to send to peer)
    pub fn shmem_id(&self) -> u32 {
        self.shmem.shmem_id()
    }

    /// Get the physical address of the data buffer (for DMA)
    pub fn data_phys(&self) -> u64 {
        let header = self.header();
        self.shmem.paddr() + header.data_offset as u64
    }

    /// Get the data buffer size
    pub fn data_size(&self) -> usize {
        self.header().data_size as usize
    }

    /// Allow another process to map this ring
    pub fn allow(&self, peer_pid: u32) -> bool {
        self.shmem.allow(peer_pid).is_ok()
    }

    /// Make ring public (accessible by any process)
    pub fn set_public(&self) -> bool {
        self.shmem.set_public().is_ok()
    }

    /// Wait for notification
    pub fn wait(&self, timeout_ms: u32) -> bool {
        self.shmem.wait(timeout_ms).is_ok()
    }

    /// Notify waiters
    pub fn notify(&self) -> u32 {
        self.shmem.notify().unwrap_or(0)
    }

    // === Private helpers ===

    fn base(&self) -> *mut u8 {
        self.shmem.as_ptr()
    }

    fn header(&self) -> &RingHeader {
        unsafe { &*(self.base() as *const RingHeader) }
    }

    fn sq_ptr(&self) -> *mut S {
        unsafe { self.base().add(self.header().sq_offset as usize) as *mut S }
    }

    fn cq_ptr(&self) -> *mut C {
        unsafe { self.base().add(self.header().cq_offset as usize) as *mut C }
    }

    /// Get pointer to data buffer
    pub fn data_ptr(&self) -> *mut u8 {
        unsafe { self.base().add(self.header().data_offset as usize) }
    }

    /// Get base address (for debugging)
    pub fn base_addr(&self) -> u64 {
        self.base() as u64
    }

    /// Get data offset from header (for debugging)
    pub fn data_offset(&self) -> u32 {
        self.header().data_offset
    }

    /// Get slice of data buffer
    pub fn data(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                self.data_ptr(),
                self.header().data_size as usize
            )
        }
    }

    /// Get mutable slice of data buffer
    pub fn data_mut(&mut self) -> &mut [u8] {
        unsafe {
            core::slice::from_raw_parts_mut(
                self.data_ptr(),
                self.header().data_size as usize
            )
        }
    }

    // === DMA Cache Management ===

    /// Prepare data buffer region for DMA read (device writes to memory)
    /// Call BEFORE starting DMA to ensure no dirty cache lines interfere
    pub fn prepare_dma_read(&self, offset: usize, len: usize) {
        let addr = self.data_ptr() as u64 + offset as u64;
        invalidate_buffer(addr, len);
    }

    /// Complete DMA read (device wrote to memory)
    /// Call AFTER DMA completes to ensure CPU sees fresh data
    pub fn complete_dma_read(&self, offset: usize, len: usize) {
        let addr = self.data_ptr() as u64 + offset as u64;
        invalidate_buffer(addr, len);
    }

    /// Prepare data buffer region for DMA write (device reads from memory)
    /// Call BEFORE starting DMA to flush CPU writes to memory
    pub fn prepare_dma_write(&self, offset: usize, len: usize) {
        let addr = self.data_ptr() as u64 + offset as u64;
        flush_buffer(addr, len);
    }

    // === Submission Queue (Client -> Server) ===

    /// Check if submission queue has space
    pub fn sq_has_space(&self) -> bool {
        let header = self.header();
        let head = header.sq_head.load(Ordering::Acquire);
        let tail = header.sq_tail.load(Ordering::Relaxed);
        let next_tail = (tail + 1) & header.mask();
        next_tail != head
    }

    /// Submit a request to the submission queue
    /// Returns true on success, false if queue is full
    pub fn submit(&self, request: &S) -> bool {
        let header = self.header();
        let head = header.sq_head.load(Ordering::Acquire);
        let tail = header.sq_tail.load(Ordering::Relaxed);
        let next_tail = (tail + 1) & header.mask();

        if next_tail == head {
            return false; // Queue full
        }

        // Write request
        unsafe {
            let slot = self.sq_ptr().add(tail as usize);
            core::ptr::write_volatile(slot, *request);
        }

        // Memory barrier before updating tail
        core::sync::atomic::fence(Ordering::Release);

        // Update tail
        header.sq_tail.store(next_tail, Ordering::Release);

        true
    }

    /// Get next request from submission queue (server side)
    /// Returns None if queue is empty
    pub fn next_request(&self) -> Option<S> {
        let header = self.header();
        let head = header.sq_head.load(Ordering::Relaxed);
        let tail = header.sq_tail.load(Ordering::Acquire);

        if head == tail {
            return None; // Queue empty
        }

        // Read request
        let request = unsafe {
            let slot = self.sq_ptr().add(head as usize);
            core::ptr::read_volatile(slot)
        };

        // Update head
        let next_head = (head + 1) & header.mask();
        header.sq_head.store(next_head, Ordering::Release);

        Some(request)
    }

    /// Check if there are pending requests
    pub fn sq_pending(&self) -> u32 {
        let header = self.header();
        let head = header.sq_head.load(Ordering::Acquire);
        let tail = header.sq_tail.load(Ordering::Acquire);
        if tail >= head {
            tail - head
        } else {
            header.entry_count - head + tail
        }
    }

    // === Completion Queue (Server -> Client) ===

    /// Check if completion queue has space
    pub fn cq_has_space(&self) -> bool {
        let header = self.header();
        let head = header.cq_head.load(Ordering::Acquire);
        let tail = header.cq_tail.load(Ordering::Relaxed);
        let next_tail = (tail + 1) & header.mask();
        next_tail != head
    }

    /// Complete a request (server side)
    /// Returns true on success, false if queue is full
    pub fn complete(&self, response: &C) -> bool {
        let header = self.header();
        let head = header.cq_head.load(Ordering::Acquire);
        let tail = header.cq_tail.load(Ordering::Relaxed);
        let next_tail = (tail + 1) & header.mask();

        if next_tail == head {
            return false; // Queue full
        }

        // Write response
        unsafe {
            let slot = self.cq_ptr().add(tail as usize);
            core::ptr::write_volatile(slot, *response);
        }

        // Memory barrier before updating tail
        core::sync::atomic::fence(Ordering::Release);

        // Update tail
        header.cq_tail.store(next_tail, Ordering::Release);

        true
    }

    /// Get next completion from completion queue (client side)
    /// Returns None if queue is empty
    pub fn next_completion(&self) -> Option<C> {
        let header = self.header();
        let head = header.cq_head.load(Ordering::Relaxed);
        let tail = header.cq_tail.load(Ordering::Acquire);

        if head == tail {
            return None; // Queue empty
        }

        // Read response
        let response = unsafe {
            let slot = self.cq_ptr().add(head as usize);
            core::ptr::read_volatile(slot)
        };

        // Update head
        let next_head = (head + 1) & header.mask();
        header.cq_head.store(next_head, Ordering::Release);

        Some(response)
    }

    /// Check if there are pending completions
    pub fn cq_pending(&self) -> u32 {
        let header = self.header();
        let head = header.cq_head.load(Ordering::Acquire);
        let tail = header.cq_tail.load(Ordering::Acquire);
        if tail >= head {
            tail - head
        } else {
            header.entry_count - head + tail
        }
    }
}

// Ring uses Shmem which handles its own Drop

// =============================================================================
// Block Device Protocol
// =============================================================================

/// Block read/write request (submission queue entry)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlockRequest {
    /// Command: 0 = Read, 1 = Write, 2 = Info
    pub cmd: u8,
    /// Reserved
    pub _reserved: [u8; 3],
    /// Request tag (for matching responses)
    pub tag: u32,
    /// Logical block address
    pub lba: u64,
    /// Number of blocks
    pub count: u32,
    /// Offset into data buffer
    pub buf_offset: u32,
}

impl BlockRequest {
    pub const CMD_READ: u8 = 0;
    pub const CMD_WRITE: u8 = 1;
    pub const CMD_INFO: u8 = 2;

    pub fn read(tag: u32, lba: u64, count: u32, buf_offset: u32) -> Self {
        Self {
            cmd: Self::CMD_READ,
            _reserved: [0; 3],
            tag,
            lba,
            count,
            buf_offset,
        }
    }

    pub fn write(tag: u32, lba: u64, count: u32, buf_offset: u32) -> Self {
        Self {
            cmd: Self::CMD_WRITE,
            _reserved: [0; 3],
            tag,
            lba,
            count,
            buf_offset,
        }
    }

    pub fn info(tag: u32) -> Self {
        Self {
            cmd: Self::CMD_INFO,
            _reserved: [0; 3],
            tag,
            lba: 0,
            count: 0,
            buf_offset: 0,
        }
    }
}

/// Block operation response (completion queue entry)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct BlockResponse {
    /// Status: 0 = OK, negative = error
    pub status: i32,
    /// Request tag (matches request)
    pub tag: u32,
    /// Bytes transferred (for read/write)
    pub bytes: u32,
    /// Block size (for info response)
    pub block_size: u32,
    /// Block count (for info response)
    pub block_count: u64,
}

impl BlockResponse {
    pub fn ok(tag: u32, bytes: u32) -> Self {
        Self {
            status: 0,
            tag,
            bytes,
            block_size: 0,
            block_count: 0,
        }
    }

    pub fn error(tag: u32, status: i32) -> Self {
        Self {
            status,
            tag,
            bytes: 0,
            block_size: 0,
            block_count: 0,
        }
    }

    pub fn info(tag: u32, block_size: u32, block_count: u64) -> Self {
        Self {
            status: 0,
            tag,
            bytes: 0,
            block_size,
            block_count,
        }
    }
}

/// Convenience type for block device rings
pub type BlockRing = Ring<BlockRequest, BlockResponse>;

// =============================================================================
// Layered I/O Protocol (Composable Driver Stack)
// =============================================================================
//
// These structures support the zero-copy layered driver model where:
// - Each layer is a separate process
// - Data flows through rings without copying
// - Queries flow through sidechannel for metadata/control
//
// ```text
// fatfs → partition → msc → usbd → [hardware]
//    └──────────────────────────────────────┘
//              same data pool
// ```

/// Layered I/O Submission Queue Entry (SQE) - 32 bytes
///
/// Requests flow down the stack (app → hardware).
/// The actual data is in the shared pool at `data_offset`.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct IoSqe {
    /// Operation code (see `io_op` module)
    pub opcode: u8,
    /// Flags (see `io_sqe_flags` module)
    pub flags: u8,
    /// Priority (0 = normal, higher = more urgent)
    pub priority: u16,
    /// Request tag (for matching completions)
    pub tag: u32,
    /// Logical block address (for block ops)
    pub lba: u64,
    /// Offset into shared data pool (NOT the data itself)
    pub data_offset: u32,
    /// Data length in bytes
    pub data_len: u32,
    /// Layer-specific parameter (e.g., SCSI LUN, encryption key ID)
    pub param: u64,
}

/// Layered I/O Completion Queue Entry (CQE) - 16 bytes
///
/// Completions flow up the stack (hardware → app).
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct IoCqe {
    /// Completion status (see `io_status` module)
    pub status: u16,
    /// Flags
    pub flags: u16,
    /// Original request tag
    pub tag: u32,
    /// Bytes transferred (may be less than requested)
    pub transferred: u32,
    /// Layer-specific result
    pub result: u32,
}

/// Sidechannel Entry - 32 bytes
///
/// For queries and control messages that flow through the layer stack.
/// Each layer can answer, pass down, or return EOL.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct SideEntry {
    /// Message type (see `side_msg` module)
    pub msg_type: u16,
    /// Flags
    pub flags: u16,
    /// Request tag (for matching responses)
    pub tag: u16,
    /// Status (see `side_status` module)
    pub status: u16,
    /// Payload (query-specific data)
    pub payload: [u8; 24],
}

impl Default for SideEntry {
    fn default() -> Self {
        Self {
            msg_type: 0,
            flags: 0,
            tag: 0,
            status: 0,
            payload: [0; 24],
        }
    }
}

/// Layered I/O operation codes
pub mod io_op {
    pub const NOP: u8 = 0;
    pub const READ: u8 = 1;
    pub const WRITE: u8 = 2;
    pub const FLUSH: u8 = 3;
    pub const DISCARD: u8 = 4;
    pub const SYNC: u8 = 5;

    // Layer-specific ranges (each layer gets 32 opcodes)
    pub const SCSI_BASE: u8 = 0x20;
    pub const CRYPTO_BASE: u8 = 0x40;
    pub const NET_BASE: u8 = 0x60;
    pub const FS_BASE: u8 = 0x80;
}

/// Layered I/O completion status codes
pub mod io_status {
    pub const OK: u16 = 0;
    pub const IO_ERROR: u16 = 1;
    pub const INVALID: u16 = 2;
    pub const NOT_READY: u16 = 3;
    pub const TIMEOUT: u16 = 4;
    pub const CANCELLED: u16 = 5;
    pub const NO_SPACE: u16 = 6;
    pub const NOT_FOUND: u16 = 7;
}

/// Layered I/O SQE flags
pub mod io_sqe_flags {
    /// Drain: wait for all prior ops to complete first
    pub const DRAIN: u8 = 1 << 0;
    /// Link: if this fails, cancel subsequent linked entries
    pub const LINK: u8 = 1 << 1;
    /// Async: don't block waiting for hardware
    pub const ASYNC: u8 = 1 << 2;
}

/// Sidechannel message types
pub mod side_msg {
    // Queries (request info) - 0x00xx
    pub const QUERY_INFO: u16 = 0x0001;        // Get device/layer info
    pub const QUERY_GEOMETRY: u16 = 0x0002;    // Get block geometry
    pub const QUERY_PARTITION: u16 = 0x0003;   // Get partition table
    pub const QUERY_CAPS: u16 = 0x0004;        // Get capabilities

    // Control (change state) - 0x01xx
    pub const CTRL_FLUSH: u16 = 0x0100;        // Flush caches
    pub const CTRL_RESET: u16 = 0x0101;        // Reset layer
    pub const CTRL_EJECT: u16 = 0x0102;        // Eject media

    // Notifications (from lower to upper) - 0x02xx
    pub const NOTIFY_MEDIA_CHANGE: u16 = 0x0200;
    pub const NOTIFY_ERROR: u16 = 0x0201;

    // VFS registration (fatfsd → vfsd) - 0x03xx
    pub const REGISTER_MOUNT: u16 = 0x0300;
}

/// Sidechannel status codes
pub mod side_status {
    /// Query answered successfully
    pub const OK: u16 = 0;
    /// Not for me, forwarded downstream (layer should pass to next)
    pub const PASS_DOWN: u16 = 1;
    /// End of line - nobody could answer
    pub const EOL: u16 = 2;
    /// Error processing query
    pub const ERROR: u16 = 3;
    /// Request (not yet answered) - used to distinguish requests from responses
    /// in the shared sidechannel queue
    pub const REQUEST: u16 = 0xFFFF;
}

// =============================================================================
// Layered Ring with Sidechannel
// =============================================================================

/// Extended ring header for layered I/O with sidechannel
///
/// Memory layout:
/// ```text
/// offset 0x000: LayeredRingHeader (64 bytes)
/// offset 0x040: SQE array (ring_size * 32 bytes)
/// offset 0x040 + sq_size: CQE array (ring_size * 16 bytes)
/// offset after CQ: Sidechannel array (side_size * 32 bytes)
/// offset pool_offset: Data buffer pool (pool_size bytes)
/// ```
#[repr(C, align(64))]
pub struct LayeredRingHeader {
    /// Magic number ("LRIO")
    pub magic: u32,
    /// Protocol version
    pub version: u16,
    /// Ring size (power of 2)
    pub ring_size: u16,

    // Submission queue pointers
    /// SQ head - consumer (lower layer) advances
    pub sq_head: AtomicU32,
    /// SQ tail - producer (upper layer) advances
    pub sq_tail: AtomicU32,

    // Completion queue pointers
    /// CQ head - consumer (upper layer) advances
    pub cq_head: AtomicU32,
    /// CQ tail - producer (lower layer) advances
    pub cq_tail: AtomicU32,

    // Sidechannel pointers
    /// Side head - consumer advances
    pub side_head: AtomicU32,
    /// Side tail - producer advances
    pub side_tail: AtomicU32,
    /// Sidechannel size (may differ from ring_size)
    pub side_size: u16,
    /// Reserved
    _pad0: u16,

    /// Offset to data buffer pool
    pub pool_offset: u32,
    /// Size of data buffer pool
    pub pool_size: u32,

    /// Doorbell flags (for notification)
    pub doorbell: AtomicU32,

    /// Reserved
    _reserved: [u32; 3],
}

/// Magic for layered ring ("LRIO")
pub const LAYERED_RING_MAGIC: u32 = 0x4C52494F;
/// Layered ring version
pub const LAYERED_RING_VERSION: u16 = 1;

impl LayeredRingHeader {
    /// Offset to SQ array
    pub const fn sq_offset() -> usize {
        64
    }

    /// Offset to CQ array
    pub fn cq_offset(&self) -> usize {
        Self::sq_offset() + (self.ring_size as usize * core::mem::size_of::<IoSqe>())
    }

    /// Offset to sidechannel array
    pub fn side_offset(&self) -> usize {
        self.cq_offset() + (self.ring_size as usize * core::mem::size_of::<IoCqe>())
    }

    /// Total size for ring structures (before pool)
    pub fn struct_size(&self) -> usize {
        self.side_offset() + (self.side_size as usize * core::mem::size_of::<SideEntry>())
    }

    /// Mask for index wrapping
    pub fn ring_mask(&self) -> u32 {
        self.ring_size as u32 - 1
    }

    /// Mask for sidechannel index wrapping
    pub fn side_mask(&self) -> u32 {
        self.side_size as u32 - 1
    }

    /// Validate header
    pub fn is_valid(&self) -> bool {
        self.magic == LAYERED_RING_MAGIC
            && self.version == LAYERED_RING_VERSION
            && self.ring_size > 0
            && (self.ring_size & (self.ring_size - 1)) == 0
            && (self.side_size == 0 || (self.side_size & (self.side_size - 1)) == 0)
    }
}

/// Layered I/O ring with sidechannel
pub struct LayeredRing {
    /// Shared memory wrapper
    shmem: crate::ipc::Shmem,
}

unsafe impl Send for LayeredRing {}

impl LayeredRing {
    /// Create a new layered ring
    ///
    /// # Arguments
    /// * `ring_size` - Number of SQ/CQ entries (power of 2)
    /// * `side_size` - Number of sidechannel entries (power of 2, 0 to disable)
    /// * `pool_size` - Size of data buffer pool in bytes
    pub fn create(ring_size: u16, side_size: u16, pool_size: u32) -> Option<Self> {
        // Validate
        if ring_size == 0 || (ring_size & (ring_size - 1)) != 0 {
            return None;
        }
        if side_size != 0 && (side_size & (side_size - 1)) != 0 {
            return None;
        }

        // Calculate sizes
        let header_size = 64;
        let sq_size = ring_size as usize * core::mem::size_of::<IoSqe>();
        let cq_size = ring_size as usize * core::mem::size_of::<IoCqe>();
        let side_bytes = side_size as usize * core::mem::size_of::<SideEntry>();
        let struct_size = header_size + sq_size + cq_size + side_bytes;
        let total_size = struct_size + pool_size as usize;

        // Allocate shared memory
        let shmem = crate::ipc::Shmem::create(total_size).ok()?;
        let base = shmem.as_ptr();

        // Initialize header
        unsafe {
            let header = base as *mut LayeredRingHeader;
            (*header).magic = LAYERED_RING_MAGIC;
            (*header).version = LAYERED_RING_VERSION;
            (*header).ring_size = ring_size;
            (*header).sq_head = AtomicU32::new(0);
            (*header).sq_tail = AtomicU32::new(0);
            (*header).cq_head = AtomicU32::new(0);
            (*header).cq_tail = AtomicU32::new(0);
            (*header).side_head = AtomicU32::new(0);
            (*header).side_tail = AtomicU32::new(0);
            (*header).side_size = side_size;
            (*header)._pad0 = 0;
            (*header).pool_offset = struct_size as u32;
            (*header).pool_size = pool_size;
            (*header).doorbell = AtomicU32::new(0);
            (*header)._reserved = [0; 3];
        }

        Some(Self { shmem })
    }

    /// Map an existing layered ring
    pub fn map(shmem_id: u32) -> Option<Self> {
        let shmem = crate::ipc::Shmem::open_existing(shmem_id).ok()?;

        let base = shmem.as_ptr();

        // Validate header
        unsafe {
            let header = base as *const LayeredRingHeader;
            if !(*header).is_valid() {
                return None;
            }
        }

        Some(Self { shmem })
    }

    /// Get shared memory ID
    pub fn shmem_id(&self) -> u32 {
        self.shmem.shmem_id()
    }

    /// Allow another process to map this ring
    pub fn allow(&self, peer_pid: u32) -> bool {
        self.shmem.allow(peer_pid).is_ok()
    }

    /// Make ring public (accessible by any process)
    pub fn set_public(&self) -> bool {
        self.shmem.set_public().is_ok()
    }

    /// Wait for notification
    pub fn wait(&self, timeout_ms: u32) -> bool {
        self.shmem.wait(timeout_ms).is_ok()
    }

    /// Notify waiters
    pub fn notify(&self) {
        let _ = self.shmem.notify();
    }

    /// Get the shmem handle for Mux registration
    pub fn shmem_handle(&self) -> crate::syscall::Handle {
        self.shmem.handle()
    }

    fn base(&self) -> *mut u8 {
        self.shmem.as_ptr()
    }

    fn header(&self) -> &LayeredRingHeader {
        unsafe { &*(self.base() as *const LayeredRingHeader) }
    }

    // === Submission Queue ===

    /// Space available in SQ
    pub fn sq_space(&self) -> u32 {
        let h = self.header();
        let head = h.sq_head.load(Ordering::Acquire);
        let tail = h.sq_tail.load(Ordering::Relaxed);
        h.ring_size as u32 - tail.wrapping_sub(head)
    }

    /// Submit a request
    pub fn sq_submit(&self, sqe: &IoSqe) -> bool {
        let h = self.header();
        if self.sq_space() == 0 {
            return false;
        }

        let tail = h.sq_tail.load(Ordering::Relaxed);
        let idx = (tail & h.ring_mask()) as usize;

        unsafe {
            let sq = self.base().add(LayeredRingHeader::sq_offset()) as *mut IoSqe;
            core::ptr::write_volatile(sq.add(idx), *sqe);
        }

        core::sync::atomic::fence(Ordering::Release);
        h.sq_tail.store(tail.wrapping_add(1), Ordering::Release);
        true
    }

    /// Pending SQ entries
    pub fn sq_pending(&self) -> u32 {
        let h = self.header();
        let head = h.sq_head.load(Ordering::Relaxed);
        let tail = h.sq_tail.load(Ordering::Acquire);
        tail.wrapping_sub(head)
    }

    /// Consume next SQ entry
    pub fn sq_consume(&self) -> Option<IoSqe> {
        let h = self.header();
        if self.sq_pending() == 0 {
            return None;
        }

        let head = h.sq_head.load(Ordering::Relaxed);
        let idx = (head & h.ring_mask()) as usize;

        let sqe = unsafe {
            let sq = self.base().add(LayeredRingHeader::sq_offset()) as *const IoSqe;
            core::ptr::read_volatile(sq.add(idx))
        };

        core::sync::atomic::fence(Ordering::Acquire);
        h.sq_head.store(head.wrapping_add(1), Ordering::Release);
        Some(sqe)
    }

    // === Completion Queue ===

    /// Post a completion
    pub fn cq_complete(&self, cqe: &IoCqe) -> bool {
        let h = self.header();
        let tail = h.cq_tail.load(Ordering::Relaxed);
        let idx = (tail & h.ring_mask()) as usize;

        unsafe {
            let cq = self.base().add(h.cq_offset()) as *mut IoCqe;
            core::ptr::write_volatile(cq.add(idx), *cqe);
        }

        core::sync::atomic::fence(Ordering::Release);
        h.cq_tail.store(tail.wrapping_add(1), Ordering::Release);
        true
    }

    /// Pending CQ entries
    pub fn cq_pending(&self) -> u32 {
        let h = self.header();
        let head = h.cq_head.load(Ordering::Relaxed);
        let tail = h.cq_tail.load(Ordering::Acquire);
        tail.wrapping_sub(head)
    }

    /// Consume next CQ entry
    pub fn cq_consume(&self) -> Option<IoCqe> {
        let h = self.header();
        if self.cq_pending() == 0 {
            return None;
        }

        let head = h.cq_head.load(Ordering::Relaxed);
        let idx = (head & h.ring_mask()) as usize;

        let cqe = unsafe {
            let cq = self.base().add(h.cq_offset()) as *const IoCqe;
            core::ptr::read_volatile(cq.add(idx))
        };

        core::sync::atomic::fence(Ordering::Acquire);
        h.cq_head.store(head.wrapping_add(1), Ordering::Release);
        Some(cqe)
    }

    // === Sidechannel ===

    /// Is sidechannel enabled?
    pub fn has_sidechannel(&self) -> bool {
        self.header().side_size > 0
    }

    /// Space in sidechannel
    pub fn side_space(&self) -> u32 {
        let h = self.header();
        if h.side_size == 0 {
            return 0;
        }
        let head = h.side_head.load(Ordering::Acquire);
        let tail = h.side_tail.load(Ordering::Relaxed);
        h.side_size as u32 - tail.wrapping_sub(head)
    }

    /// Send sidechannel entry
    pub fn side_send(&self, entry: &SideEntry) -> bool {
        let h = self.header();
        if h.side_size == 0 || self.side_space() == 0 {
            return false;
        }

        let tail = h.side_tail.load(Ordering::Relaxed);
        let idx = (tail & h.side_mask()) as usize;

        unsafe {
            let side = self.base().add(h.side_offset()) as *mut SideEntry;
            core::ptr::write_volatile(side.add(idx), *entry);
        }

        core::sync::atomic::fence(Ordering::Release);
        h.side_tail.store(tail.wrapping_add(1), Ordering::Release);
        true
    }

    /// Pending sidechannel entries
    pub fn side_pending(&self) -> u32 {
        let h = self.header();
        if h.side_size == 0 {
            return 0;
        }
        let head = h.side_head.load(Ordering::Relaxed);
        let tail = h.side_tail.load(Ordering::Acquire);
        tail.wrapping_sub(head)
    }

    /// Receive sidechannel entry (consumes it)
    pub fn side_recv(&self) -> Option<SideEntry> {
        let h = self.header();
        if h.side_size == 0 || self.side_pending() == 0 {
            return None;
        }

        let head = h.side_head.load(Ordering::Relaxed);
        let idx = (head & h.side_mask()) as usize;

        let entry = unsafe {
            let side = self.base().add(h.side_offset()) as *const SideEntry;
            core::ptr::read_volatile(side.add(idx))
        };

        core::sync::atomic::fence(Ordering::Acquire);
        h.side_head.store(head.wrapping_add(1), Ordering::Release);
        Some(entry)
    }

    /// Peek at next sidechannel entry without consuming
    pub fn side_peek(&self) -> Option<SideEntry> {
        let h = self.header();
        if h.side_size == 0 || self.side_pending() == 0 {
            return None;
        }

        let head = h.side_head.load(Ordering::Relaxed);
        let idx = (head & h.side_mask()) as usize;

        let entry = unsafe {
            let side = self.base().add(h.side_offset()) as *const SideEntry;
            core::ptr::read_volatile(side.add(idx))
        };

        core::sync::atomic::fence(Ordering::Acquire);
        // Don't advance head - just peek
        Some(entry)
    }

    // === Data Pool ===

    /// Get data pool base pointer
    pub fn pool_ptr(&self) -> *mut u8 {
        let h = self.header();
        unsafe { self.base().add(h.pool_offset as usize) }
    }

    /// Get data pool size
    pub fn pool_size(&self) -> u32 {
        self.header().pool_size
    }

    /// Get data pool physical address (for DMA)
    pub fn pool_phys(&self) -> u64 {
        self.shmem.paddr() + self.header().pool_offset as u64
    }

    /// Get slice at offset in pool
    pub fn pool_slice(&self, offset: u32, len: u32) -> Option<&[u8]> {
        let h = self.header();
        if offset + len > h.pool_size {
            return None;
        }
        unsafe {
            Some(core::slice::from_raw_parts(
                self.pool_ptr().add(offset as usize),
                len as usize,
            ))
        }
    }

    /// Get mutable slice at offset in pool
    pub fn pool_slice_mut(&self, offset: u32, len: u32) -> Option<&mut [u8]> {
        let h = self.header();
        if offset + len > h.pool_size {
            return None;
        }
        unsafe {
            Some(core::slice::from_raw_parts_mut(
                self.pool_ptr().add(offset as usize),
                len as usize,
            ))
        }
    }
}

// LayeredRing uses Shmem which handles its own Drop

// =============================================================================
// Pool Allocator (simple bump allocator)
// =============================================================================

/// Simple bump allocator for data pool
///
/// For production, use a proper free-list allocator.
pub struct PoolAlloc {
    base_offset: u32,
    size: u32,
    next: u32,
}

impl PoolAlloc {
    /// Create allocator for a pool region
    pub fn new(base_offset: u32, size: u32) -> Self {
        Self {
            base_offset,
            size,
            next: 0,
        }
    }

    /// Allocate buffer, returns offset from pool base
    pub fn alloc(&mut self, len: u32) -> Option<u32> {
        // Align to 64 bytes
        let aligned = (len + 63) & !63;
        if self.next + aligned > self.size {
            return None;
        }
        let offset = self.base_offset + self.next;
        self.next += aligned;
        Some(offset)
    }

    /// Reset (free all)
    pub fn reset(&mut self) {
        self.next = 0;
    }

    /// Remaining space
    pub fn remaining(&self) -> u32 {
        self.size - self.next
    }
}

// =============================================================================
// Helper: Calculate total shmem size needed
// =============================================================================

/// Calculate minimum shared memory size for a layered ring
pub const fn layered_ring_size(ring_size: u16, side_size: u16, pool_size: u32) -> usize {
    let header = 64;
    let sq = ring_size as usize * 32; // IoSqe is 32 bytes
    let cq = ring_size as usize * 16; // IoCqe is 16 bytes
    let side = side_size as usize * 32; // SideEntry is 32 bytes
    header + sq + cq + side + pool_size as usize
}
