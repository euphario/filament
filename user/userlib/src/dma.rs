//! DMA Buffer Abstraction
//!
//! Type-safe wrappers for DMA buffers that enforce correct cache synchronization.
//!
//! # Two kinds of DMA memory
//!
//! | Type | Backing | CPU perf | Sync cost |
//! |------|---------|----------|-----------|
//! | **CoherentBuf** | Non-cacheable | Slower reads | None (always coherent) |
//! | **StreamingBuf** | Cacheable | Fast reads | Explicit `sync_for_device` / `sync_for_cpu` |
//!
//! # Usage
//!
//! ```ignore
//! // Coherent: descriptors, control structures
//! let desc = DmaPool::alloc(4096)?.into_coherent();
//! desc.write32(0, 0xDEADBEEF); // No sync needed
//!
//! // Streaming: bulk data (RX/TX buffers)
//! let mut rxbuf = DmaPool::alloc_streaming(4096)?.into_streaming(DmaDirection::FromDevice);
//! rxbuf.sync_for_device(); // Invalidate before device writes
//! // ... DMA transfer ...
//! rxbuf.sync_for_cpu(); // Invalidate before CPU reads
//! let data = rxbuf.read32(0);
//! ```

use crate::mmio::{DmaPool, cache_clean, cache_invalidate, cache_clean_invalidate};

/// Direction of DMA transfer (follows Linux DMA API semantics)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DmaDirection {
    /// CPU writes, device reads (TX)
    ToDevice,
    /// Device writes, CPU reads (RX)
    FromDevice,
    /// Both directions
    Bidirectional,
}

/// A DMA buffer with type-safe sync semantics.
///
/// All DMA buffers implement this trait. The sync methods are no-ops for
/// coherent buffers and perform cache maintenance for streaming buffers.
pub trait DmaBuf {
    /// Virtual address of the buffer (CPU-accessible)
    fn vaddr(&self) -> u64;
    /// Physical address of the buffer (for programming DMA descriptors)
    fn paddr(&self) -> u64;
    /// Size of the buffer in bytes
    fn size(&self) -> usize;

    /// Call BEFORE device accesses the buffer.
    ///
    /// For ToDevice: cleans cache (CPU writes → RAM so device can read)
    /// For FromDevice: invalidates cache (stale lines discarded before device writes)
    /// For Bidirectional: clean + invalidate
    fn sync_for_device(&self);

    /// Call AFTER device DMA completes, BEFORE CPU reads.
    ///
    /// For ToDevice: no-op (CPU doesn't read after TX)
    /// For FromDevice: invalidates cache (device wrote to RAM, discard stale lines)
    /// For Bidirectional: invalidate
    fn sync_for_cpu(&self);

    /// Raw pointer to buffer start
    fn as_ptr(&self) -> *const u8 {
        self.vaddr() as *const u8
    }

    /// Mutable raw pointer to buffer start
    fn as_mut_ptr(&self) -> *mut u8 {
        self.vaddr() as *mut u8
    }

    /// Read a u32 at byte offset (volatile)
    fn read32(&self, offset: usize) -> u32 {
        debug_assert!(offset + 4 <= self.size());
        unsafe { core::ptr::read_volatile((self.vaddr() + offset as u64) as *const u32) }
    }

    /// Write a u32 at byte offset (volatile)
    fn write32(&self, offset: usize, val: u32) {
        debug_assert!(offset + 4 <= self.size());
        unsafe { core::ptr::write_volatile((self.vaddr() + offset as u64) as *mut u32, val) }
    }
}

/// Non-cacheable DMA buffer — sync is always a no-op.
///
/// Use for descriptor rings, control structures, and anything where the device
/// and CPU access the same memory frequently. Simpler but slower CPU reads.
pub struct CoherentBuf {
    pool: DmaPool,
}

impl CoherentBuf {
    /// Consume a DmaPool and wrap it as a coherent buffer.
    ///
    /// The pool must have been allocated with `DmaPool::alloc()` (non-cacheable).
    pub fn new(pool: DmaPool) -> Self {
        Self { pool }
    }

    /// Get the underlying DmaPool (for APIs that need the raw pool)
    pub fn pool(&self) -> &DmaPool {
        &self.pool
    }
}

impl DmaBuf for CoherentBuf {
    #[inline]
    fn vaddr(&self) -> u64 { self.pool.vaddr() }
    #[inline]
    fn paddr(&self) -> u64 { self.pool.paddr() }
    #[inline]
    fn size(&self) -> usize { self.pool.size() }
    #[inline]
    fn sync_for_device(&self) { /* no-op: non-cacheable memory is always coherent */ }
    #[inline]
    fn sync_for_cpu(&self) { /* no-op: non-cacheable memory is always coherent */ }
}

/// Cacheable DMA buffer — sync performs cache maintenance.
///
/// Use for bulk data buffers (packet RX/TX, block I/O) where CPU read performance
/// matters. Requires explicit sync before/after DMA transfers.
///
/// # Sync semantics (following Linux DMA API)
///
/// | Direction | `sync_for_device()` | `sync_for_cpu()` |
/// |-----------|---------------------|-------------------|
/// | ToDevice | clean (DC CVAC) | no-op |
/// | FromDevice | invalidate (DC CIVAC) | invalidate (DC CIVAC) |
/// | Bidirectional | clean+invalidate | invalidate |
pub struct StreamingBuf {
    pool: DmaPool,
    direction: DmaDirection,
}

impl StreamingBuf {
    /// Consume a DmaPool and wrap it as a streaming buffer.
    ///
    /// The pool should have been allocated with `DmaPool::alloc_streaming()`.
    pub fn new(pool: DmaPool, direction: DmaDirection) -> Self {
        Self { pool, direction }
    }

    /// Get the DMA direction
    pub fn direction(&self) -> DmaDirection {
        self.direction
    }

    /// Get the underlying DmaPool (for APIs that need the raw pool)
    pub fn pool(&self) -> &DmaPool {
        &self.pool
    }
}

impl DmaBuf for StreamingBuf {
    #[inline]
    fn vaddr(&self) -> u64 { self.pool.vaddr() }
    #[inline]
    fn paddr(&self) -> u64 { self.pool.paddr() }
    #[inline]
    fn size(&self) -> usize { self.pool.size() }

    fn sync_for_device(&self) {
        match self.direction {
            DmaDirection::ToDevice => {
                cache_clean(self.pool.vaddr(), self.pool.size());
            }
            DmaDirection::FromDevice => {
                cache_invalidate(self.pool.vaddr(), self.pool.size());
            }
            DmaDirection::Bidirectional => {
                cache_clean_invalidate(self.pool.vaddr(), self.pool.size());
            }
        }
    }

    fn sync_for_cpu(&self) {
        match self.direction {
            DmaDirection::ToDevice => { /* no-op: CPU doesn't read TX buffers */ }
            DmaDirection::FromDevice | DmaDirection::Bidirectional => {
                cache_invalidate(self.pool.vaddr(), self.pool.size());
            }
        }
    }
}

// Conversion methods on DmaPool
impl DmaPool {
    /// Convert this pool into a coherent (non-cacheable) DMA buffer.
    ///
    /// Consumes the pool. Use for descriptor rings and control structures.
    pub fn into_coherent(self) -> CoherentBuf {
        CoherentBuf::new(self)
    }

    /// Convert this pool into a streaming (cacheable) DMA buffer.
    ///
    /// Consumes the pool. Use for bulk data buffers that require explicit sync.
    pub fn into_streaming(self, direction: DmaDirection) -> StreamingBuf {
        StreamingBuf::new(self, direction)
    }
}
