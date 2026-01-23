//! Address Space Trait Definitions
//!
//! These traits define the contract for address space operations.
//! This enables mock implementations for testing without hardware MMU.
//!
//! # Design Philosophy
//!
//! The address space trait abstracts virtual memory management,
//! allowing tests to verify page mapping logic without actual
//! page table manipulation.

/// ASID (Address Space Identifier) type
pub type Asid = u16;

/// Virtual address type
pub type VirtAddr = u64;

/// Physical address type
pub type PhysAddr = u64;

// ============================================================================
// Memory Attributes
// ============================================================================

/// Memory type for page mappings
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MemoryType {
    /// Normal cacheable memory
    Normal,
    /// Device memory (non-cacheable, no reordering)
    Device,
    /// Normal non-cacheable (for DMA coherency)
    NonCacheable,
}

/// Page permissions
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PageFlags {
    /// Page is writable
    pub writable: bool,
    /// Page is executable
    pub executable: bool,
    /// Memory type
    pub memory_type: MemoryType,
}

impl PageFlags {
    /// Create flags for normal code page (read-only, executable)
    pub fn code() -> Self {
        Self {
            writable: false,
            executable: true,
            memory_type: MemoryType::Normal,
        }
    }

    /// Create flags for normal data page (read-write, not executable)
    pub fn data() -> Self {
        Self {
            writable: true,
            executable: false,
            memory_type: MemoryType::Normal,
        }
    }

    /// Create flags for read-only data page
    pub fn rodata() -> Self {
        Self {
            writable: false,
            executable: false,
            memory_type: MemoryType::Normal,
        }
    }

    /// Create flags for device MMIO page
    pub fn device() -> Self {
        Self {
            writable: true,
            executable: false,
            memory_type: MemoryType::Device,
        }
    }

    /// Create flags for DMA buffer page
    pub fn dma() -> Self {
        Self {
            writable: true,
            executable: false,
            memory_type: MemoryType::NonCacheable,
        }
    }
}

// ============================================================================
// Address Space Backend Trait
// ============================================================================

/// Trait for address space operations
///
/// This trait abstracts the MMU and page table operations.
/// The kernel holds a single instance implementing this trait.
///
/// # Contract
///
/// 1. All operations are thread-safe (internal locking)
/// 2. ASID 0 is reserved for kernel
/// 3. Page mappings are 4KB aligned
/// 4. TLB invalidation happens automatically on unmap
pub trait AddressSpaceBackend: Send + Sync {
    // ========================================================================
    // ASID Operations
    // ========================================================================

    /// Allocate a new ASID
    ///
    /// Returns None if all ASIDs are exhausted (max 255 user spaces).
    fn alloc_asid(&self) -> Option<Asid>;

    /// Free an ASID
    ///
    /// This also invalidates all TLB entries for the ASID.
    fn free_asid(&self, asid: Asid);

    /// Get count of allocated ASIDs (for debugging)
    fn asid_count(&self) -> usize;

    // ========================================================================
    // Page Table Operations
    // ========================================================================

    /// Map a page in an address space
    ///
    /// # Arguments
    /// * `asid` - Address space identifier
    /// * `virt` - Virtual address (must be 4KB aligned)
    /// * `phys` - Physical address (must be 4KB aligned)
    /// * `flags` - Page permissions and memory type
    ///
    /// # Returns
    /// true if mapping succeeded, false otherwise.
    fn map_page(
        &self,
        asid: Asid,
        virt: VirtAddr,
        phys: PhysAddr,
        flags: PageFlags,
    ) -> bool;

    /// Unmap a page from an address space
    ///
    /// # Returns
    /// Physical address that was mapped, or None if not mapped.
    fn unmap_page(&self, asid: Asid, virt: VirtAddr) -> Option<PhysAddr>;

    /// Check if a virtual address is mapped
    fn is_mapped(&self, asid: Asid, virt: VirtAddr) -> bool;

    // ========================================================================
    // TLB Operations
    // ========================================================================

    /// Invalidate TLB entry for a virtual address in an ASID
    fn invalidate_page(&self, asid: Asid, virt: VirtAddr);

    /// Invalidate all TLB entries for an ASID
    fn invalidate_asid(&self, asid: Asid);

    // ========================================================================
    // Address Space Activation
    // ========================================================================

    /// Activate an address space (switch TTBR0)
    ///
    /// # Arguments
    /// * `ttbr0` - TTBR0 value including ASID in bits [63:48]
    fn activate(&self, ttbr0: u64);

    /// Get the current active TTBR0 value
    fn current_ttbr0(&self) -> u64;
}

// ============================================================================
// Error Types
// ============================================================================

/// Address space errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddrSpaceError {
    /// No free ASIDs available
    NoAsid,
    /// No free page table entries
    NoPageTables,
    /// Address not aligned to page boundary
    NotAligned,
    /// Address out of valid range
    OutOfRange,
    /// Page already mapped
    AlreadyMapped,
    /// Page not mapped
    NotMapped,
}

impl AddrSpaceError {
    pub fn to_errno(self) -> i64 {
        match self {
            AddrSpaceError::NoAsid => -12,       // ENOMEM
            AddrSpaceError::NoPageTables => -12, // ENOMEM
            AddrSpaceError::NotAligned => -22,   // EINVAL
            AddrSpaceError::OutOfRange => -22,   // EINVAL
            AddrSpaceError::AlreadyMapped => -17, // EEXIST
            AddrSpaceError::NotMapped => -2,     // ENOENT
        }
    }
}
