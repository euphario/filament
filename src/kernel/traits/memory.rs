//! Memory Management Trait Definitions
//!
//! These traits define contracts for memory allocation and shared memory.
//!
//! # Key Fix: Mapping Tracker
//!
//! The `MappingTracker` trait solves the double-free vulnerability in shmem.
//! Instead of just tracking ref_count, we track actual (region_id, pid) pairs.
//!
//! ```ignore
//! // OLD (vulnerable):
//! region.ref_count += 1;  // map()
//! region.ref_count -= 1;  // unmap() - can call multiple times!
//!
//! // NEW (safe):
//! tracker.add_mapping(region_id, pid)?;  // map() - fails if already mapped
//! tracker.remove_mapping(region_id, pid)?;  // unmap() - fails if not mapped
//! ```

use super::waker::WakeList;

/// Physical address type
pub type PhysAddr = u64;

/// Virtual address type
pub type VirtAddr = u64;

/// Process ID
pub type Pid = u32;

/// Shared memory region ID
pub type ShmemId = u32;

// ============================================================================
// Error Types
// ============================================================================

/// Shared memory errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShmemError {
    /// Region not found
    NotFound,
    /// Caller is not the owner
    NotOwner,
    /// Process not allowed to access
    NotAllowed,
    /// Region is being destroyed
    Dying,
    /// No free slots
    NoSlots,
    /// Already mapped by this process
    AlreadyMapped,
    /// Not mapped by this process
    NotMapped,
    /// Size overflow
    SizeOverflow,
    /// Invalid size (zero or too large)
    InvalidSize,
    /// Out of memory
    OutOfMemory,
    /// Wait timeout
    Timeout,
}

impl ShmemError {
    pub fn to_errno(self) -> i64 {
        match self {
            ShmemError::NotFound => -2,       // ENOENT
            ShmemError::NotOwner => -1,       // EPERM
            ShmemError::NotAllowed => -13,    // EACCES
            ShmemError::Dying => -6,          // ENXIO
            ShmemError::NoSlots => -12,       // ENOMEM
            ShmemError::AlreadyMapped => -17, // EEXIST
            ShmemError::NotMapped => -22,     // EINVAL
            ShmemError::SizeOverflow => -22,  // EINVAL
            ShmemError::InvalidSize => -22,   // EINVAL
            ShmemError::OutOfMemory => -12,   // ENOMEM
            ShmemError::Timeout => -110,      // ETIMEDOUT
        }
    }
}

// ============================================================================
// Mapping Tracker Trait
// ============================================================================

/// Maximum mappings per region
pub const MAX_MAPPINGS_PER_REGION: usize = 16;

/// Tracks actual (region, pid) mappings to prevent double-free
///
/// # Contract
///
/// 1. `add_mapping()` fails if pid already has this region mapped
/// 2. `remove_mapping()` fails if pid doesn't have this region mapped
/// 3. `has_mapping()` returns true only if pid has active mapping
/// 4. `mapping_count()` returns exact number of active mappings
///
/// This solves the ref_count underflow vulnerability by tracking
/// actual mappings instead of just a counter.
pub trait MappingTracker: Send + Sync {
    /// Add a mapping for (region_id, pid)
    ///
    /// # Errors
    /// - `AlreadyMapped` if pid already has this region mapped
    /// - `NoSlots` if maximum mappings reached
    fn add_mapping(&self, region_id: ShmemId, pid: Pid) -> Result<(), ShmemError>;

    /// Remove a mapping for (region_id, pid)
    ///
    /// # Errors
    /// - `NotMapped` if pid doesn't have this region mapped
    fn remove_mapping(&self, region_id: ShmemId, pid: Pid) -> Result<(), ShmemError>;

    /// Check if pid has region mapped
    fn has_mapping(&self, region_id: ShmemId, pid: Pid) -> bool;

    /// Get count of active mappings for a region
    fn mapping_count(&self, region_id: ShmemId) -> usize;

    /// Get all PIDs that have region mapped
    fn get_mapped_pids(&self, region_id: ShmemId) -> [Pid; MAX_MAPPINGS_PER_REGION];

    /// Remove all mappings for a pid (process cleanup)
    fn remove_all_for_pid(&self, pid: Pid);

    /// Remove all mappings for a region (region destruction)
    fn remove_all_for_region(&self, region_id: ShmemId);
}

// ============================================================================
// Shared Memory State
// ============================================================================

/// Shared memory region state
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShmemState {
    /// Region is active and usable
    Active,
    /// Region is being destroyed (no new mappings allowed)
    Dying,
}

// ============================================================================
// Shared Memory Trait
// ============================================================================

/// Trait for shared memory region operations
///
/// # Thread Safety
///
/// All operations use internal locking.
pub trait SharedMemory: Send + Sync {
    /// Get region ID
    fn id(&self) -> ShmemId;

    /// Get owner PID
    fn owner(&self) -> Pid;

    /// Get physical address
    fn phys_addr(&self) -> PhysAddr;

    /// Get size in bytes
    fn size(&self) -> usize;

    /// Get current state
    fn state(&self) -> ShmemState;

    /// Allow a process to map this region
    ///
    /// Only owner can call this.
    fn allow(&self, pid: Pid, caller: Pid) -> Result<(), ShmemError>;

    /// Check if a process is allowed
    fn is_allowed(&self, pid: Pid) -> bool;

    /// Notify waiting processes
    ///
    /// Returns wake list and count of woken waiters.
    fn notify(&self, caller: Pid) -> Result<(WakeList, u32), ShmemError>;

    /// Begin destruction (owner exiting)
    ///
    /// Transitions to Dying state, preventing new mappings.
    fn begin_dying(&self) -> Result<(), ShmemError>;
}

// ============================================================================
// Physical Allocator Trait
// ============================================================================

/// Trait for physical memory allocation
pub trait PhysicalAllocator: Send + Sync {
    /// Allocate contiguous physical pages
    ///
    /// # Arguments
    /// * `count` - Number of 4KB pages to allocate
    ///
    /// # Returns
    /// Physical address of first page, or None if out of memory
    fn alloc_pages(&self, count: usize) -> Option<PhysAddr>;

    /// Free contiguous physical pages
    ///
    /// # Safety
    /// Caller must ensure pages are not in use.
    fn free_pages(&self, addr: PhysAddr, count: usize);

    /// Allocate a single page
    fn alloc_page(&self) -> Option<PhysAddr> {
        self.alloc_pages(1)
    }

    /// Free a single page
    fn free_page(&self, addr: PhysAddr) {
        self.free_pages(addr, 1)
    }

    /// Get total available pages
    fn available_pages(&self) -> usize;

    /// Get total pages in system
    fn total_pages(&self) -> usize;
}

// ============================================================================
// Shmem Backend Trait
// ============================================================================

/// Factory and registry for shared memory
pub trait ShmemBackend: Send + Sync {
    /// Create a new shared memory region
    ///
    /// # Arguments
    /// * `owner` - Owner PID
    /// * `size` - Size in bytes (will be page-aligned)
    ///
    /// # Returns
    /// (region_id, phys_addr, size) on success
    ///
    /// # Errors
    /// - `InvalidSize` if size is 0
    /// - `SizeOverflow` if size calculation overflows
    /// - `OutOfMemory` if allocation fails
    /// - `NoSlots` if no free region slots
    fn create(&self, owner: Pid, size: usize) -> Result<(ShmemId, PhysAddr, usize), ShmemError>;

    /// Get a shared memory region
    fn get(&self, id: ShmemId) -> Option<&dyn SharedMemory>;

    /// Map a region into a process
    ///
    /// Uses MappingTracker to prevent double-map.
    fn map(&self, id: ShmemId, pid: Pid) -> Result<(PhysAddr, usize), ShmemError>;

    /// Unmap a region from a process
    ///
    /// Uses MappingTracker to prevent double-free.
    fn unmap(&self, id: ShmemId, pid: Pid) -> Result<(), ShmemError>;

    /// Destroy a region (owner only)
    fn destroy(&self, id: ShmemId, owner: Pid) -> Result<(), ShmemError>;

    /// Clean up all regions for a process
    fn cleanup_process(&self, pid: Pid);

    /// Get the mapping tracker
    fn mapping_tracker(&self) -> &dyn MappingTracker;
}

// ============================================================================
// Size Calculation Helpers
// ============================================================================

/// Page size constant
pub const PAGE_SIZE: usize = 4096;

/// Safely align size to page boundary with overflow checking
///
/// Returns None if alignment would overflow.
pub fn align_to_page_checked(size: usize) -> Option<usize> {
    size.checked_add(PAGE_SIZE - 1)
        .map(|s| s & !(PAGE_SIZE - 1))
}

/// Safely calculate number of pages with overflow checking
///
/// Returns None if calculation would overflow.
pub fn pages_for_size_checked(size: usize) -> Option<usize> {
    align_to_page_checked(size).map(|aligned| aligned / PAGE_SIZE)
}

