//! Zero-copy ring protocol for layered block I/O
//!
//! Each layer is a separate process. Layers communicate through:
//! - Submission ring: requests flow down (app → disk)
//! - Completion ring: completions flow up (disk → app)
//! - Shared data pool: actual bytes (never copied between layers)
//!
//! ```text
//!                      ┌─────────────────────────────────┐
//!                      │        Shared Memory            │
//!  ┌────────┐          │  ┌─────────────────────────┐    │
//!  │ fatfs  │          │  │     Data Buffer Pool    │    │
//!  │        │ ───SQ───→│  │  ┌─────┬─────┬─────┐    │    │
//!  │        │ ←──CQ────│  │  │ buf │ buf │ buf │    │    │
//!  └────────┘          │  │  │  0  │  1  │  2  │    │    │
//!       ↕              │  │  └──┬──┴──┬──┴──┬──┘    │    │
//!  ┌────────┐          │  │     │     │     │       │    │
//!  │  msc   │          │  │  (data never moves,     │    │
//!  │        │ ───SQ───→│  │   only descriptors)     │    │
//!  │        │ ←──CQ────│  │                         │    │
//!  └────────┘          │  └─────────────────────────┘    │
//!       ↕              │                                 │
//!  ┌────────┐          │  Ring entries point to pool:    │
//!  │  usbd  │          │    { op: READ,                  │
//!  │        │ ───SQ───→│      data_offset: 0x1000,       │
//!  │        │ ←──CQ────│      data_len: 512 }            │
//!  └────────┘          │                                 │
//!       ↕              └─────────────────────────────────┘
//!  [hardware]
//! ```
//!
//! ## Zero-Copy Flow
//!
//! 1. App allocates buffer in pool, gets offset 0x1000
//! 2. App posts SQE: {READ, lba=100, data=0x1000, len=512}
//! 3. Layer 1 transforms: {SCSI_READ10, cdb=[...], data=0x1000}
//! 4. Layer 2 transforms: {TRB pointing to phys(0x1000)}
//! 5. DMA writes directly to 0x1000
//! 6. CQEs flow back up, app reads from 0x1000
//! 7. **Data never copied between layers**
//!
//! ## Network Storage
//!
//! Just another layer! Instead of passing to usbd:
//! ```text
//! msc → nbd_client → [network] → nbd_server → msc → usbd
//! ```
//! Same ring protocol, data still at same offset (nbd sends it over network).
//!
//! ## Encryption Layer
//!
//! ```text
//! fatfs → encrypt → msc → usbd
//!              ↓
//!         (encrypts data in-place or to second buffer,
//!          posts completion when done)
//! ```

use core::sync::atomic::{AtomicU32, Ordering};

// =============================================================================
// Ring Entry Types
// =============================================================================

/// Submission Queue Entry (SQE) - request from upper layer
///
/// Fixed 32 bytes. The actual data is in shared memory at `data_offset`.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Sqe {
    /// Operation code
    pub opcode: u8,
    /// Flags (e.g., DRAIN, LINK)
    pub flags: u8,
    /// Priority
    pub priority: u16,
    /// Request tag (for matching completions)
    pub tag: u32,
    /// Logical block address (for block ops)
    pub lba: u64,
    /// Offset into shared data pool
    pub data_offset: u32,
    /// Data length in bytes
    pub data_len: u32,
    /// Layer-specific parameter (e.g., SCSI LUN, encryption key ID)
    pub param: u64,
}

/// Completion Queue Entry (CQE) - response from lower layer
///
/// Fixed 16 bytes. Matches an SQE by tag.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Cqe {
    /// Completion status
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

/// Operation codes (generic across layers)
pub mod op {
    pub const NOP: u8 = 0;
    pub const READ: u8 = 1;
    pub const WRITE: u8 = 2;
    pub const FLUSH: u8 = 3;
    pub const DISCARD: u8 = 4;
    pub const SYNC: u8 = 5;

    // Layer-specific ranges
    pub const SCSI_BASE: u8 = 0x20;
    pub const CRYPTO_BASE: u8 = 0x40;
    pub const NET_BASE: u8 = 0x60;
}

/// Completion status codes
pub mod status {
    pub const OK: u16 = 0;
    pub const IO_ERROR: u16 = 1;
    pub const INVALID: u16 = 2;
    pub const NOT_READY: u16 = 3;
    pub const TIMEOUT: u16 = 4;
    pub const CANCELLED: u16 = 5;
}

/// SQE flags
pub mod sqe_flags {
    /// Drain: wait for all prior ops to complete first
    pub const DRAIN: u8 = 1 << 0;
    /// Link: if this fails, cancel subsequent linked entries
    pub const LINK: u8 = 1 << 1;
    /// Async: don't block waiting for hardware
    pub const ASYNC: u8 = 1 << 2;
}

// =============================================================================
// Ring Structure (in shared memory)
// =============================================================================

/// Ring header - sits at start of ring's shmem region
///
/// Layout in shared memory:
/// ```text
/// offset 0x000: RingHeader
/// offset 0x040: SQE array (ring_size entries)
/// offset 0x040 + sq_size: CQE array (ring_size entries)
/// ```
#[repr(C)]
pub struct RingHeader {
    /// Magic number for validation
    pub magic: u32,
    /// Ring size (power of 2, e.g., 64, 128, 256)
    pub ring_size: u32,

    // Submission queue pointers (modified by different sides)
    /// SQ head - consumer (lower layer) advances this
    pub sq_head: AtomicU32,
    /// SQ tail - producer (upper layer) advances this
    pub sq_tail: AtomicU32,

    // Completion queue pointers
    /// CQ head - consumer (upper layer) advances this
    pub cq_head: AtomicU32,
    /// CQ tail - producer (lower layer) advances this
    pub cq_tail: AtomicU32,

    /// Offset to data buffer pool (from start of shmem)
    pub data_pool_offset: u32,
    /// Size of data buffer pool
    pub data_pool_size: u32,

    /// Doorbell flags (for notification)
    pub sq_doorbell: AtomicU32,
    pub cq_doorbell: AtomicU32,

    /// Reserved for future use
    _reserved: [u32; 8],
}

pub const RING_MAGIC: u32 = 0x52494E47; // "RING"

impl RingHeader {
    /// Calculate offset to SQ array
    pub fn sq_offset(&self) -> usize {
        64 // Header is 64 bytes
    }

    /// Calculate offset to CQ array
    pub fn cq_offset(&self) -> usize {
        self.sq_offset() + (self.ring_size as usize * core::mem::size_of::<Sqe>())
    }
}

// =============================================================================
// Ring Operations (for a single side)
// =============================================================================

/// Producer side of a ring (submits entries)
pub struct RingProducer<'a> {
    header: &'a RingHeader,
    sq: &'a mut [Sqe],
    mask: u32,
}

impl<'a> RingProducer<'a> {
    /// Create producer from shared memory region
    ///
    /// # Safety
    /// Caller must ensure shmem points to valid, properly initialized ring memory
    pub unsafe fn from_shmem(shmem: *mut u8, ring_size: u32) -> Option<Self> {
        let header = &*(shmem as *const RingHeader);
        if header.magic != RING_MAGIC || header.ring_size != ring_size {
            return None;
        }

        let sq_ptr = shmem.add(header.sq_offset()) as *mut Sqe;
        let sq = core::slice::from_raw_parts_mut(sq_ptr, ring_size as usize);

        Some(Self {
            header,
            sq,
            mask: ring_size - 1,
        })
    }

    /// Number of free slots in submission queue
    pub fn sq_space(&self) -> u32 {
        let head = self.header.sq_head.load(Ordering::Acquire);
        let tail = self.header.sq_tail.load(Ordering::Relaxed);
        self.header.ring_size - (tail.wrapping_sub(head))
    }

    /// Submit an entry (returns false if queue full)
    pub fn submit(&mut self, sqe: Sqe) -> bool {
        if self.sq_space() == 0 {
            return false;
        }

        let tail = self.header.sq_tail.load(Ordering::Relaxed);
        let idx = (tail & self.mask) as usize;

        self.sq[idx] = sqe;

        // Memory barrier: ensure SQE is visible before tail update
        core::sync::atomic::fence(Ordering::Release);

        self.header.sq_tail.store(tail.wrapping_add(1), Ordering::Release);
        true
    }

    /// Ring doorbell to notify consumer
    pub fn doorbell(&self) {
        self.header.sq_doorbell.fetch_add(1, Ordering::Release);
    }
}

/// Consumer side of a ring (receives entries)
pub struct RingConsumer<'a> {
    header: &'a RingHeader,
    sq: &'a [Sqe],
    cq: &'a mut [Cqe],
    mask: u32,
}

impl<'a> RingConsumer<'a> {
    /// Create consumer from shared memory region
    ///
    /// # Safety
    /// Caller must ensure shmem points to valid, properly initialized ring memory
    pub unsafe fn from_shmem(shmem: *mut u8, ring_size: u32) -> Option<Self> {
        let header = &*(shmem as *const RingHeader);
        if header.magic != RING_MAGIC || header.ring_size != ring_size {
            return None;
        }

        let sq_ptr = shmem.add(header.sq_offset()) as *const Sqe;
        let sq = core::slice::from_raw_parts(sq_ptr, ring_size as usize);

        let cq_ptr = shmem.add(header.cq_offset()) as *mut Cqe;
        let cq = core::slice::from_raw_parts_mut(cq_ptr, ring_size as usize);

        Some(Self {
            header,
            sq,
            cq,
            mask: ring_size - 1,
        })
    }

    /// Number of pending submissions
    pub fn sq_pending(&self) -> u32 {
        let head = self.header.sq_head.load(Ordering::Relaxed);
        let tail = self.header.sq_tail.load(Ordering::Acquire);
        tail.wrapping_sub(head)
    }

    /// Peek at next submission (doesn't consume)
    pub fn peek(&self) -> Option<&Sqe> {
        if self.sq_pending() == 0 {
            return None;
        }
        let head = self.header.sq_head.load(Ordering::Relaxed);
        let idx = (head & self.mask) as usize;
        Some(&self.sq[idx])
    }

    /// Consume next submission
    pub fn consume(&mut self) -> Option<Sqe> {
        if self.sq_pending() == 0 {
            return None;
        }

        let head = self.header.sq_head.load(Ordering::Relaxed);
        let idx = (head & self.mask) as usize;

        // Read entry before advancing head
        let sqe = self.sq[idx];

        // Memory barrier: ensure we read SQE before advancing head
        core::sync::atomic::fence(Ordering::Acquire);

        self.header.sq_head.store(head.wrapping_add(1), Ordering::Release);
        Some(sqe)
    }

    /// Post a completion
    pub fn complete(&mut self, cqe: Cqe) -> bool {
        // CQ always has space (same size as SQ, and we complete what we consumed)
        let tail = self.header.cq_tail.load(Ordering::Relaxed);
        let idx = (tail & self.mask) as usize;

        self.cq[idx] = cqe;

        core::sync::atomic::fence(Ordering::Release);

        self.header.cq_tail.store(tail.wrapping_add(1), Ordering::Release);
        true
    }

    /// Ring completion doorbell
    pub fn doorbell(&self) {
        self.header.cq_doorbell.fetch_add(1, Ordering::Release);
    }
}

// =============================================================================
// Data Pool (buffer allocation in shared memory)
// =============================================================================

/// Simple bump allocator for data buffers in shared memory
///
/// Real implementation would have free list, but this shows the concept.
pub struct DataPool {
    base_offset: u32,
    size: u32,
    next_offset: u32,
}

impl DataPool {
    pub fn new(base_offset: u32, size: u32) -> Self {
        Self {
            base_offset,
            size,
            next_offset: 0,
        }
    }

    /// Allocate a buffer, returns offset from shmem base
    pub fn alloc(&mut self, len: u32) -> Option<u32> {
        // Align to 64 bytes
        let aligned_len = (len + 63) & !63;

        if self.next_offset + aligned_len > self.size {
            return None;
        }

        let offset = self.base_offset + self.next_offset;
        self.next_offset += aligned_len;
        Some(offset)
    }

    /// Reset pool (free all allocations)
    pub fn reset(&mut self) {
        self.next_offset = 0;
    }
}

// =============================================================================
// Layer Interface
// =============================================================================

/// Trait for a processing layer
///
/// Each layer:
/// - Consumes from input ring (requests from above)
/// - Produces to output ring (requests to below)
/// - Handles completions flowing back up
pub trait Layer {
    /// Process pending requests
    fn process(&mut self);

    /// Handle a completion from below
    fn complete(&mut self, cqe: Cqe);
}

// =============================================================================
// Example: How layers connect
// =============================================================================

/*
Example setup for: fatfs → msc → usbd

1. devd creates shmem regions:
   - fatfs_msc_ring (fatfs → msc)
   - msc_usbd_ring (msc → usbd)
   - data_pool (shared by all)

2. devd spawns processes, gives handles:
   - fatfs: fatfs_msc_ring (producer), data_pool
   - msc: fatfs_msc_ring (consumer), msc_usbd_ring (producer)
   - usbd: msc_usbd_ring (consumer), hardware access

3. Request flow (READ):
   fatfs: alloc buffer at offset 0x1000
   fatfs: submit {READ, lba=100, data=0x1000} to fatfs_msc_ring
   msc:   consume, translate to SCSI
   msc:   submit {SCSI_READ10, data=0x1000} to msc_usbd_ring
   usbd:  consume, build TRB with DMA to phys(shmem + 0x1000)
   usbd:  hardware DMA writes to 0x1000
   usbd:  complete {OK, tag} to msc_usbd_ring
   msc:   complete {OK, tag} to fatfs_msc_ring
   fatfs: read data from 0x1000

4. Data at 0x1000 was written once (by DMA), never copied!

Adding network storage:

   fatfs → msc → nbd_client ──[network]──→ nbd_server → msc → usbd
                     │                          │
                     └── same ring protocol ────┘

Adding encryption:

   fatfs → encrypt → msc → usbd
               │
               └── encrypts in-place (or to second buffer)
                   same ring protocol
*/
