//! User Memory Mapping Types
//!
//! This module defines types for user-space memory management:
//! - `MappingKind`: Ownership semantics for mapped pages
//! - `HeapMapping`: Tracks a single heap mapping
//! - `MapFlags`: Attributes for page mapping
//! - `MapSource`: Where physical memory comes from
//! - `MapResult`: Result of a mapping operation
//!
//! The actual mapping implementation remains on `Task` since it needs
//! access to the address space and page tables.

// ============================================================================
// Constants
// ============================================================================

/// Maximum number of heap mappings per task
pub const MAX_HEAP_MAPPINGS: usize = 64;

/// Start of user heap region
pub const USER_HEAP_START: u64 = 0x5000_0000;

/// End of user heap region
pub const USER_HEAP_END: u64 = 0x7000_0000;

// ============================================================================
// Mapping Types
// ============================================================================

/// Kind of memory mapping - determines ownership and cleanup behavior.
///
/// This is critical for proper resource management:
/// - OwnedAnon/OwnedDma: Task allocated these pages, must free on unmap/exit
/// - BorrowedShmem: Pages owned by shmem system, never free here
/// - DeviceMmio: Physical MMIO addresses, NEVER free (not RAM!)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MappingKind {
    /// Anonymous memory allocated by mmap() - kernel owns pages
    OwnedAnon,
    /// DMA-capable memory allocated by mmap_dma() - kernel owns pages
    OwnedDma,
    /// Shared memory mapped via mmap_phys() - pages owned elsewhere
    BorrowedShmem,
    /// Device MMIO mapped via mmap_device() - not RAM, never free
    DeviceMmio,
}

/// A single heap mapping entry
#[derive(Clone, Copy)]
pub struct HeapMapping {
    /// Virtual address of mapping
    pub virt_addr: u64,
    /// Physical address of mapping
    pub phys_addr: u64,
    /// Number of pages
    pub num_pages: usize,
    /// Ownership kind - determines if pages should be freed on unmap
    pub kind: MappingKind,
}

impl HeapMapping {
    pub const fn empty() -> Self {
        Self {
            virt_addr: 0,
            phys_addr: 0,
            num_pages: 0,
            kind: MappingKind::OwnedAnon, // Default doesn't matter for empty
        }
    }

    pub fn is_empty(&self) -> bool {
        self.num_pages == 0
    }

    /// Returns true if this mapping owns its physical pages (should free on unmap)
    pub fn owns_pages(&self) -> bool {
        matches!(self.kind, MappingKind::OwnedAnon | MappingKind::OwnedDma)
    }
}

// ============================================================================
// Mapping Flags
// ============================================================================

/// Flags for map_region()
#[derive(Clone, Copy, Debug)]
pub struct MapFlags {
    /// Page is writable
    pub writable: bool,
    /// Page is executable
    pub executable: bool,
    /// Zero pages after allocation
    pub zero: bool,
    /// Flush cache after zeroing (for DMA)
    pub flush_cache: bool,
    /// Use device memory attributes (nGnRnE) instead of normal cacheable
    pub device: bool,
}

impl MapFlags {
    /// Anonymous memory mapping (normal cacheable, zeroed)
    pub const fn anon(writable: bool, executable: bool) -> Self {
        Self {
            writable,
            executable,
            zero: true,
            flush_cache: false,
            device: false,
        }
    }

    /// DMA memory mapping (cacheable, zeroed, cache flushed)
    pub const fn dma() -> Self {
        Self {
            writable: true,
            executable: false,
            zero: true,
            flush_cache: true,
            device: false,
        }
    }

    /// High DMA memory mapping (for 36-bit addresses above 4GB)
    /// No kernel-side zeroing or cache flush since kernel doesn't have mapping
    pub const fn dma_high() -> Self {
        Self {
            writable: true,
            executable: false,
            zero: false,      // Can't zero - kernel has no mapping for high memory
            flush_cache: false, // Can't flush - kernel has no mapping
            device: false,
        }
    }

    /// Shared memory mapping (no allocation, no zeroing)
    pub const fn shared() -> Self {
        Self {
            writable: true,
            executable: false,
            zero: false,
            flush_cache: false,
            device: false,
        }
    }

    /// Mapping existing shmem (no zeroing - data already initialized)
    /// Uses non-cacheable (device memory) for cross-process coherency.
    /// This ensures writes are immediately visible to other processes without
    /// requiring userspace memory barriers.
    pub const fn shmem_map() -> Self {
        Self {
            writable: true,
            executable: false,
            zero: false,       // CRITICAL: Don't zero - data already there!
            flush_cache: false, // No flush needed - NC memory bypasses cache
            device: true,      // Non-cacheable for cross-process safety
        }
    }

    /// Device MMIO mapping (non-cacheable, no zeroing)
    pub const fn device() -> Self {
        Self {
            writable: true,
            executable: false,
            zero: false,
            flush_cache: false,
            device: true,
        }
    }
}

// ============================================================================
// Mapping Source & Result
// ============================================================================

/// Source of physical memory for mapping
#[derive(Clone, Copy, Debug)]
pub enum MapSource {
    /// Allocate new physical pages
    Allocate,
    /// Use specified physical address (don't allocate)
    Fixed(u64),
}

/// Result of map_region
#[derive(Clone, Copy, Debug)]
pub struct MapResult {
    /// Virtual address of the mapping
    pub virt_addr: u64,
    /// Physical address of the mapping
    pub phys_addr: u64,
}
