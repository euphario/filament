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
use crate::syscall;

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
        self.magic == RING_MAGIC && self.version == RING_VERSION
    }

    /// Get the mask for index wrapping (entry_count - 1)
    pub fn mask(&self) -> u32 {
        self.entry_count - 1
    }
}

/// Shared ring buffer
pub struct Ring<S, C> {
    /// Shared memory ID
    shmem_id: u32,
    /// Virtual address of the shared region
    base: *mut u8,
    /// Physical address (for DMA)
    phys: u64,
    /// Total size of the region
    size: usize,
    /// Are we the owner (creator)?
    is_owner: bool,
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
    pub fn create(entry_count: u32, data_size: usize) -> Option<Self> {
        // Round entry count to power of 2
        let entry_count = entry_count.next_power_of_two();

        // Calculate sizes
        let header_size = core::mem::size_of::<RingHeader>();
        let sq_size = (entry_count as usize) * core::mem::size_of::<S>();
        let cq_size = (entry_count as usize) * core::mem::size_of::<C>();

        // Align each section to cache line
        let sq_offset = (header_size + 63) & !63;
        let cq_offset = (sq_offset + sq_size + 63) & !63;
        let data_offset = (cq_offset + cq_size + 63) & !63;
        let total_size = data_offset + data_size;

        // Create shared memory region
        let mut vaddr: u64 = 0;
        let mut paddr: u64 = 0;
        let shmem_id = syscall::shmem_create(total_size, &mut vaddr, &mut paddr);
        if shmem_id < 0 {
            return None;
        }

        let base = vaddr as *mut u8;

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

        Some(Self {
            shmem_id: shmem_id as u32,
            base,
            phys: paddr,
            size: total_size,
            is_owner: true,
            _marker: PhantomData,
        })
    }

    /// Map an existing ring buffer
    pub fn map(shmem_id: u32) -> Option<Self> {
        let mut vaddr: u64 = 0;
        let mut paddr: u64 = 0;
        let result = syscall::shmem_map(shmem_id, &mut vaddr, &mut paddr);
        if result < 0 {
            return None;
        }

        let base = vaddr as *mut u8;

        // Validate header
        unsafe {
            let header = base as *const RingHeader;
            if !(*header).is_valid() {
                return None;
            }

            // Calculate total size from header
            let data_offset = (*header).data_offset as usize;
            let data_size = (*header).data_size as usize;
            let total_size = data_offset + data_size;

            Some(Self {
                shmem_id,
                base,
                phys: paddr,
                size: total_size,
                is_owner: false,
                _marker: PhantomData,
            })
        }
    }

    /// Get the shared memory ID (to send to peer)
    pub fn shmem_id(&self) -> u32 {
        self.shmem_id
    }

    /// Get the physical address of the data buffer (for DMA)
    pub fn data_phys(&self) -> u64 {
        let header = self.header();
        self.phys + header.data_offset as u64
    }

    /// Get the data buffer size
    pub fn data_size(&self) -> usize {
        self.header().data_size as usize
    }

    /// Allow another process to map this ring
    pub fn allow(&self, peer_pid: u32) -> bool {
        syscall::shmem_allow(self.shmem_id, peer_pid) >= 0
    }

    /// Wait for notification
    pub fn wait(&self, timeout_ms: u32) -> bool {
        syscall::shmem_wait(self.shmem_id, timeout_ms) >= 0
    }

    /// Notify waiters
    pub fn notify(&self) -> u32 {
        let result = syscall::shmem_notify(self.shmem_id);
        if result >= 0 { result as u32 } else { 0 }
    }

    // === Private helpers ===

    fn header(&self) -> &RingHeader {
        unsafe { &*(self.base as *const RingHeader) }
    }

    fn sq_ptr(&self) -> *mut S {
        unsafe { self.base.add(self.header().sq_offset as usize) as *mut S }
    }

    fn cq_ptr(&self) -> *mut C {
        unsafe { self.base.add(self.header().cq_offset as usize) as *mut C }
    }

    /// Get pointer to data buffer
    pub fn data_ptr(&self) -> *mut u8 {
        unsafe { self.base.add(self.header().data_offset as usize) }
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

impl<S, C> Drop for Ring<S, C> {
    fn drop(&mut self) {
        if self.is_owner {
            syscall::shmem_destroy(self.shmem_id);
        }
    }
}

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
