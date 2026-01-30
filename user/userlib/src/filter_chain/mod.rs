//! Zero-Copy Filter Chain
//!
//! Inspired by BIG-IP TMM's HUD filter chain. Data stays in one DMA buffer,
//! only descriptors flow through layers.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                    SHARED DMA BUFFER                        │
//! │  ┌────────┬────────┬────────┬──────────────────────────┐   │
//! │  │ Hdr 3  │ Hdr 2  │ Hdr 1  │       PAYLOAD            │   │
//! │  └────────┴────────┴────────┴──────────────────────────┘   │
//! │      ↑        ↑        ↑            ↑                      │
//! │   Layer3   Layer2   Layer1      Hardware                   │
//! └─────────────────────────────────────────────────────────────┘
//!
//! Descriptors (tiny metadata) flow through rings:
//!   DOWN ring: App → Layer3 → Layer2 → Layer1 → Hardware
//!   UP ring:   Hardware → Layer1 → Layer2 → Layer3 → App
//! ```
//!
//! ## Zero Copy Guarantee
//!
//! - Payload is written ONCE by hardware (DMA) or application
//! - Each layer only reads/writes its own header region
//! - Descriptors contain offsets, not data copies

pub mod layers;

use core::sync::atomic::{AtomicU32, Ordering};

// =============================================================================
// Descriptor - The unit of work flowing through the chain
// =============================================================================

/// Descriptor flags
pub mod desc_flags {
    /// Direction: 0 = read (device→host), 1 = write (host→device)
    pub const WRITE: u32 = 1 << 0;
    /// Descriptor is valid and ready for processing
    pub const VALID: u32 = 1 << 1;
    /// Processing complete
    pub const COMPLETE: u32 = 1 << 2;
    /// Error occurred
    pub const ERROR: u32 = 1 << 3;
    /// Last descriptor in a chain
    pub const LAST: u32 = 1 << 4;
}

/// A descriptor pointing into the shared DMA buffer
///
/// Size: 64 bytes (fits in one cache line)
#[repr(C, align(64))]
#[derive(Clone, Copy)]
pub struct Descriptor {
    // === Buffer location (8 bytes) ===
    /// Offset into DMA pool where data starts
    pub data_offset: u32,
    /// Length of data
    pub data_len: u32,

    // === Control (8 bytes) ===
    /// Flags (direction, valid, complete, error)
    pub flags: u32,
    /// Unique ID for tracking
    pub id: u32,

    // === Per-layer metadata (32 bytes) ===
    /// Layer-specific data - each layer knows its slot
    /// Layer 0 (hardware): physical address, etc.
    /// Layer 1 (xHCI): TRB pointer, slot/endpoint
    /// Layer 2 (MSC): SCSI command, LBA
    /// Layer 3 (FS): inode, block number
    pub layer_data: [u64; 4],

    // === Result (8 bytes) ===
    /// Bytes actually transferred (set on completion)
    pub transferred: u32,
    /// Error code if ERROR flag set
    pub error_code: u32,

    // === Reserved (8 bytes for future use) ===
    pub _reserved: [u32; 2],
}

impl Descriptor {
    pub const fn new() -> Self {
        Self {
            data_offset: 0,
            data_len: 0,
            flags: 0,
            id: 0,
            layer_data: [0; 4],
            transferred: 0,
            error_code: 0,
            _reserved: [0; 2],
        }
    }

    #[inline]
    pub fn is_write(&self) -> bool {
        (self.flags & desc_flags::WRITE) != 0
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        (self.flags & desc_flags::VALID) != 0
    }

    #[inline]
    pub fn is_complete(&self) -> bool {
        (self.flags & desc_flags::COMPLETE) != 0
    }

    #[inline]
    pub fn is_error(&self) -> bool {
        (self.flags & desc_flags::ERROR) != 0
    }

    #[inline]
    pub fn set_complete(&mut self, transferred: u32) {
        self.transferred = transferred;
        self.flags |= desc_flags::COMPLETE;
    }

    #[inline]
    pub fn set_error(&mut self, code: u32) {
        self.error_code = code;
        self.flags |= desc_flags::ERROR | desc_flags::COMPLETE;
    }
}

// =============================================================================
// Descriptor Ring - Lock-free SPSC queue
// =============================================================================

/// Ring buffer size (must be power of 2)
pub const RING_SIZE: usize = 64;
const RING_MASK: u32 = (RING_SIZE - 1) as u32;

/// Single-producer single-consumer ring for descriptors
///
/// Lock-free: producer writes to tail, consumer reads from head
#[repr(C, align(64))]
pub struct DescriptorRing {
    /// Producer writes here (cache line 1)
    tail: AtomicU32,
    _pad1: [u32; 15],

    /// Consumer reads from here (cache line 2)
    head: AtomicU32,
    _pad2: [u32; 15],

    /// Descriptor storage
    descriptors: [Descriptor; RING_SIZE],
}

impl DescriptorRing {
    pub const fn new() -> Self {
        Self {
            tail: AtomicU32::new(0),
            _pad1: [0; 15],
            head: AtomicU32::new(0),
            _pad2: [0; 15],
            descriptors: [Descriptor::new(); RING_SIZE],
        }
    }

    /// Try to enqueue a descriptor (producer side)
    pub fn try_push(&mut self, desc: Descriptor) -> bool {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);

        // Check if full
        if tail.wrapping_sub(head) >= RING_SIZE as u32 {
            return false;
        }

        let idx = (tail & RING_MASK) as usize;
        self.descriptors[idx] = desc;

        // Memory barrier: ensure descriptor is written before tail advances
        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        true
    }

    /// Try to dequeue a descriptor (consumer side)
    pub fn try_pop(&mut self) -> Option<Descriptor> {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);

        // Check if empty
        if head == tail {
            return None;
        }

        let idx = (head & RING_MASK) as usize;
        let desc = self.descriptors[idx];

        // Memory barrier: ensure descriptor is read before head advances
        self.head.store(head.wrapping_add(1), Ordering::Release);
        Some(desc)
    }

    /// Check if ring is empty
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.head.load(Ordering::Acquire) == self.tail.load(Ordering::Acquire)
    }

    /// Number of pending descriptors
    #[inline]
    pub fn len(&self) -> usize {
        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Acquire);
        tail.wrapping_sub(head) as usize
    }
}

// =============================================================================
// Filter Chain - The shared region between processes
// =============================================================================

/// Memory layout for a filter chain
///
/// This structure lives in shared DMA memory, mapped into all participants.
#[repr(C)]
pub struct FilterChainRegion {
    // === Header (64 bytes) ===
    /// Magic number for validation
    pub magic: u32,
    /// Version
    pub version: u32,
    /// Number of layers in this chain
    pub layer_count: u32,
    /// Size of DMA data pool
    pub data_pool_size: u32,
    /// Offset to data pool from start of region
    pub data_pool_offset: u32,
    /// Next descriptor ID to allocate
    pub next_desc_id: AtomicU32,
    _header_pad: [u32; 10],

    // === Rings ===
    /// Requests going down toward hardware
    pub down_ring: DescriptorRing,
    /// Completions going up toward application
    pub up_ring: DescriptorRing,

    // Note: Data pool follows this structure in memory
}

pub const FILTER_CHAIN_MAGIC: u32 = 0x46494C54; // "FILT"
pub const FILTER_CHAIN_VERSION: u32 = 1;

impl FilterChainRegion {
    /// Initialize a new filter chain region
    ///
    /// # Safety
    /// Caller must ensure `ptr` points to a valid, zeroed memory region
    /// of at least `total_size` bytes.
    pub unsafe fn init(ptr: *mut u8, total_size: usize, layer_count: u32) -> &'static mut Self {
        // SAFETY: Caller guarantees ptr points to valid memory of sufficient size
        let region = unsafe { &mut *(ptr as *mut Self) };

        region.magic = FILTER_CHAIN_MAGIC;
        region.version = FILTER_CHAIN_VERSION;
        region.layer_count = layer_count;

        // Data pool starts after the header and rings
        let header_size = core::mem::size_of::<Self>();
        region.data_pool_offset = header_size as u32;
        region.data_pool_size = (total_size - header_size) as u32;

        region.next_desc_id = AtomicU32::new(1);
        region.down_ring = DescriptorRing::new();
        region.up_ring = DescriptorRing::new();

        region
    }

    /// Validate this is a properly initialized region
    pub fn validate(&self) -> bool {
        self.magic == FILTER_CHAIN_MAGIC && self.version == FILTER_CHAIN_VERSION
    }

    /// Get pointer to data pool
    pub fn data_pool(&self) -> *mut u8 {
        let base = self as *const Self as *const u8;
        unsafe { base.add(self.data_pool_offset as usize) as *mut u8 }
    }

    /// Allocate a descriptor ID
    pub fn alloc_desc_id(&self) -> u32 {
        self.next_desc_id.fetch_add(1, Ordering::Relaxed)
    }
}

// =============================================================================
// Filter Layer Trait - What each layer implements
// =============================================================================

/// Result of processing a descriptor
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterResult {
    /// Continue to next layer
    Continue,
    /// Descriptor consumed, don't pass further
    Consume,
    /// Error occurred
    Error(u32),
    /// Need to wait for async completion (e.g., hardware)
    Pending,
}

/// A layer in the filter chain
pub trait FilterLayer {
    /// Process descriptor going DOWN (toward hardware)
    ///
    /// Layer should:
    /// - Read `desc.data_offset` and `desc.data_len`
    /// - Add its headers before data_offset if needed
    /// - Update `desc.layer_data[self.layer_index()]` with its metadata
    /// - Return Continue to pass to next layer
    fn filter_down(&mut self, desc: &mut Descriptor, data_pool: *mut u8) -> FilterResult;

    /// Process descriptor going UP (toward application)
    ///
    /// Layer should:
    /// - Check completion status
    /// - Parse any response data at its offset
    /// - Return Continue to pass to upper layer
    fn filter_up(&mut self, desc: &mut Descriptor, data_pool: *mut u8) -> FilterResult;

    /// This layer's index (0 = closest to hardware)
    fn layer_index(&self) -> usize;

    /// Human-readable name for debugging
    fn name(&self) -> &'static str;
}

// =============================================================================
// Buffer Allocator - Simple bump allocator for the data pool
// =============================================================================

/// Simple allocator for the data pool
///
/// Each request gets a chunk of the pool. For now, simple bump allocation.
/// In production, would want free list or slab allocator.
pub struct DataPoolAllocator {
    pool_base: *mut u8,
    pool_size: usize,
    next_offset: AtomicU32,
}

impl DataPoolAllocator {
    pub fn new(pool_base: *mut u8, pool_size: usize) -> Self {
        Self {
            pool_base,
            pool_size,
            next_offset: AtomicU32::new(0),
        }
    }

    /// Allocate a chunk from the pool
    ///
    /// Returns offset into pool, or None if full.
    /// Alignment is 64 bytes for cache line alignment.
    pub fn alloc(&self, size: usize) -> Option<u32> {
        let aligned_size = (size + 63) & !63;

        loop {
            let current = self.next_offset.load(Ordering::Relaxed);
            let new_offset = current + aligned_size as u32;

            if new_offset as usize > self.pool_size {
                return None; // Pool exhausted
            }

            if self
                .next_offset
                .compare_exchange(current, new_offset, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                return Some(current);
            }
            // CAS failed, retry
        }
    }

    /// Reset allocator (e.g., after all descriptors complete)
    pub fn reset(&self) {
        self.next_offset.store(0, Ordering::Release);
    }

    /// Get pointer to data at offset
    #[inline]
    pub fn ptr_at(&self, offset: u32) -> *mut u8 {
        unsafe { self.pool_base.add(offset as usize) }
    }
}

// =============================================================================
// Size calculations
// =============================================================================

/// Calculate total size needed for a filter chain region
pub const fn filter_chain_size(data_pool_size: usize) -> usize {
    core::mem::size_of::<FilterChainRegion>() + data_pool_size
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor_size() {
        assert_eq!(core::mem::size_of::<Descriptor>(), 64);
    }

    #[test]
    fn ring_operations() {
        let mut ring = DescriptorRing::new();

        assert!(ring.is_empty());

        let mut desc = Descriptor::new();
        desc.id = 42;

        assert!(ring.try_push(desc));
        assert!(!ring.is_empty());
        assert_eq!(ring.len(), 1);

        let popped = ring.try_pop().unwrap();
        assert_eq!(popped.id, 42);
        assert!(ring.is_empty());
    }
}
