//! Shared Memory Backend Implementation
//!
//! Implements the `ShmemBackend`, `MappingTracker`, and `SharedMemory` traits
//! by delegating to the existing shmem module. This provides a clean trait
//! boundary without changing the internal implementation.
//!
//! # Thread Safety
//!
//! All operations go through the shmem module's SpinLock, ensuring proper
//! serialization on SMP systems.

use crate::kernel::traits::memory::{
    ShmemBackend, ShmemError, MappingTracker, SharedMemory, ShmemState as TraitShmemState,
    PhysAddr, Pid, ShmemId, MAX_MAPPINGS_PER_REGION,
};
use crate::kernel::traits::waker::WakeList;
use crate::kernel::shmem::{self, RegionState, SharedMem};

// ============================================================================
// Type Conversions
// ============================================================================

/// Convert internal RegionState to trait ShmemState
fn convert_state(state: RegionState) -> TraitShmemState {
    match state {
        RegionState::Active => TraitShmemState::Active,
        RegionState::Dying => TraitShmemState::Dying,
    }
}

/// Convert errno to ShmemError
fn errno_to_error(errno: i64) -> ShmemError {
    match errno {
        -1 => ShmemError::NotOwner,      // EPERM
        -2 => ShmemError::NotFound,      // ENOENT
        -11 => ShmemError::Dying,        // EAGAIN (used for dying state)
        -12 => ShmemError::OutOfMemory,  // ENOMEM
        -13 => ShmemError::NotAllowed,   // EACCES
        -16 => ShmemError::NotMapped,    // EBUSY (still mapped)
        -17 => ShmemError::AlreadyMapped, // EEXIST
        -22 => ShmemError::InvalidSize,  // EINVAL
        -110 => ShmemError::Timeout,     // ETIMEDOUT
        _ => ShmemError::NotFound,       // Default
    }
}

// ============================================================================
// Kernel Mapping Tracker Implementation
// ============================================================================

/// Kernel mapping tracker implementation
///
/// A zero-sized type that implements `MappingTracker` by delegating to the
/// shmem module's MappingTable.
pub struct KernelMappingTracker;

impl KernelMappingTracker {
    /// Create a new kernel mapping tracker instance
    pub const fn new() -> Self {
        Self
    }
}

impl MappingTracker for KernelMappingTracker {
    fn add_mapping(&self, region_id: ShmemId, pid: Pid) -> Result<(), ShmemError> {
        shmem::mapping_add(region_id, pid).map_err(errno_to_error)
    }

    fn remove_mapping(&self, region_id: ShmemId, pid: Pid) -> Result<(), ShmemError> {
        shmem::mapping_remove(region_id, pid).map_err(errno_to_error)
    }

    fn has_mapping(&self, region_id: ShmemId, pid: Pid) -> bool {
        shmem::mapping_has(region_id, pid)
    }

    fn mapping_count(&self, region_id: ShmemId) -> usize {
        shmem::mapping_count(region_id)
    }

    fn get_mapped_pids(&self, region_id: ShmemId) -> [Pid; MAX_MAPPINGS_PER_REGION] {
        // The trait uses MAX_MAPPINGS_PER_REGION (16), shmem uses MAX_ALLOWED_PIDS (8)
        // We need to handle this difference
        let mut result = [0; MAX_MAPPINGS_PER_REGION];
        let pids = shmem::mapping_get_pids(region_id);
        // Copy available PIDs (MAX_ALLOWED_PIDS = 8)
        for (i, &pid) in pids.iter().enumerate() {
            if i < MAX_MAPPINGS_PER_REGION {
                result[i] = pid;
            }
        }
        result
    }

    fn remove_all_for_pid(&self, pid: Pid) {
        shmem::mapping_remove_all_for_pid(pid);
    }

    fn remove_all_for_region(&self, region_id: ShmemId) {
        shmem::mapping_remove_all_for_region(region_id);
    }
}

// ============================================================================
// Kernel Shared Memory Region Implementation
// ============================================================================

/// Wrapper for SharedMem that implements the SharedMemory trait
///
/// Note: This is a snapshot of the region's state. For authoritative state,
/// always query through ShmemBackend.
pub struct KernelSharedMemory {
    /// Cached region data
    region: SharedMem,
}

impl KernelSharedMemory {
    /// Create from a SharedMem snapshot
    fn from_region(region: SharedMem) -> Self {
        Self { region }
    }
}

impl SharedMemory for KernelSharedMemory {
    fn id(&self) -> ShmemId {
        self.region.id
    }

    fn owner(&self) -> Pid {
        self.region.owner_pid
    }

    fn phys_addr(&self) -> PhysAddr {
        self.region.phys_addr
    }

    fn size(&self) -> usize {
        self.region.size
    }

    fn state(&self) -> TraitShmemState {
        convert_state(self.region.state())
    }

    fn allow(&self, pid: Pid, caller: Pid) -> Result<(), ShmemError> {
        shmem::allow(caller, self.region.id, pid).map_err(errno_to_error)
    }

    fn is_allowed(&self, pid: Pid) -> bool {
        self.region.is_allowed(pid)
    }

    fn notify(&self, caller: Pid) -> Result<(WakeList, u32), ShmemError> {
        // notify() returns count of woken tasks
        // We need to convert to WakeList - but the shmem module wakes directly
        // For trait compliance, we call notify and return empty wake list since waking is done
        let count = shmem::notify(caller, self.region.id).map_err(errno_to_error)?;
        Ok((WakeList::new(), count))
    }

    fn begin_dying(&self) -> Result<(), ShmemError> {
        // This needs to be called on the actual region in the table
        shmem::begin_region_dying(self.region.id).map_err(errno_to_error)
    }
}

// ============================================================================
// Kernel Shmem Backend Implementation
// ============================================================================

/// Kernel shmem backend implementation
///
/// A zero-sized type that implements `ShmemBackend` by delegating to the
/// global shmem module.
pub struct KernelShmemBackend;

impl KernelShmemBackend {
    /// Create a new kernel shmem backend instance
    pub const fn new() -> Self {
        Self
    }
}

impl ShmemBackend for KernelShmemBackend {
    fn create(&self, owner: Pid, size: usize) -> Result<(ShmemId, PhysAddr, usize), ShmemError> {
        shmem::create(owner, size)
            .map(|(id, _vaddr, paddr)| (id, paddr, size))
            .map_err(errno_to_error)
    }

    fn get(&self, _id: ShmemId) -> Option<&dyn SharedMemory> {
        // Note: This is tricky because we can't return a reference to a temporary.
        // The trait design assumes we have persistent objects, but shmem module
        // returns copies. For now, this method is not usable in this implementation.
        // Callers should use the other methods directly.
        None
    }

    fn map(&self, id: ShmemId, pid: Pid) -> Result<(PhysAddr, usize), ShmemError> {
        shmem::map(pid, id)
            .map(|(_vaddr, paddr)| {
                // Get the region size
                let size = shmem::get_size(id).unwrap_or(0);
                (paddr, size)
            })
            .map_err(errno_to_error)
    }

    fn unmap(&self, id: ShmemId, pid: Pid) -> Result<(), ShmemError> {
        shmem::unmap(pid, id).map_err(errno_to_error)
    }

    fn destroy(&self, id: ShmemId, owner: Pid) -> Result<(), ShmemError> {
        shmem::destroy(owner, id).map_err(errno_to_error)
    }

    fn cleanup_process(&self, pid: Pid) {
        shmem::process_cleanup(pid);
    }

    fn mapping_tracker(&self) -> &dyn MappingTracker {
        &MAPPING_TRACKER
    }
}

// ============================================================================
// Global Instances
// ============================================================================

/// Global kernel mapping tracker
pub static MAPPING_TRACKER: KernelMappingTracker = KernelMappingTracker::new();

/// Global kernel shmem backend
pub static SHMEM_BACKEND: KernelShmemBackend = KernelShmemBackend::new();

/// Get a reference to the global shmem backend
pub fn shmem_backend() -> &'static dyn ShmemBackend {
    &SHMEM_BACKEND
}

/// Get a reference to the global mapping tracker
pub fn mapping_tracker() -> &'static dyn MappingTracker {
    &MAPPING_TRACKER
}

// ============================================================================
// Mock Implementation for Testing
// ============================================================================

#[cfg(test)]
pub mod mock {
    use super::*;
    use core::cell::RefCell;

    /// Mock region for testing
    #[derive(Clone)]
    pub struct MockRegion {
        pub id: ShmemId,
        pub owner: Pid,
        pub phys_addr: PhysAddr,
        pub size: usize,
        pub state: TraitShmemState,
        pub allowed: Vec<Pid>,
    }

    /// Mock mapping tracker for testing
    pub struct MockMappingTracker {
        mappings: RefCell<Vec<(ShmemId, Pid)>>,
    }

    impl MockMappingTracker {
        pub fn new() -> Self {
            Self {
                mappings: RefCell::new(Vec::new()),
            }
        }

        /// Get all mappings for verification
        pub fn all_mappings(&self) -> Vec<(ShmemId, Pid)> {
            self.mappings.borrow().clone()
        }
    }

    impl MappingTracker for MockMappingTracker {
        fn add_mapping(&self, region_id: ShmemId, pid: Pid) -> Result<(), ShmemError> {
            let mut mappings = self.mappings.borrow_mut();
            if mappings.iter().any(|(r, p)| *r == region_id && *p == pid) {
                return Err(ShmemError::AlreadyMapped);
            }
            mappings.push((region_id, pid));
            Ok(())
        }

        fn remove_mapping(&self, region_id: ShmemId, pid: Pid) -> Result<(), ShmemError> {
            let mut mappings = self.mappings.borrow_mut();
            if let Some(idx) = mappings.iter().position(|(r, p)| *r == region_id && *p == pid) {
                mappings.remove(idx);
                Ok(())
            } else {
                Err(ShmemError::NotMapped)
            }
        }

        fn has_mapping(&self, region_id: ShmemId, pid: Pid) -> bool {
            self.mappings.borrow().iter().any(|(r, p)| *r == region_id && *p == pid)
        }

        fn mapping_count(&self, region_id: ShmemId) -> usize {
            self.mappings.borrow().iter().filter(|(r, _)| *r == region_id).count()
        }

        fn get_mapped_pids(&self, region_id: ShmemId) -> [Pid; MAX_MAPPINGS_PER_REGION] {
            let mut result = [0; MAX_MAPPINGS_PER_REGION];
            let mappings = self.mappings.borrow();
            for (i, (r, p)) in mappings.iter().filter(|(r, _)| *r == region_id).enumerate() {
                if i < MAX_MAPPINGS_PER_REGION {
                    result[i] = *p;
                }
            }
            result
        }

        fn remove_all_for_pid(&self, pid: Pid) {
            self.mappings.borrow_mut().retain(|(_, p)| *p != pid);
        }

        fn remove_all_for_region(&self, region_id: ShmemId) {
            self.mappings.borrow_mut().retain(|(r, _)| *r != region_id);
        }
    }

    /// Mock shmem backend for testing
    pub struct MockShmemBackend {
        regions: RefCell<Vec<MockRegion>>,
        next_id: RefCell<ShmemId>,
        mapping_tracker: MockMappingTracker,
    }

    impl MockShmemBackend {
        pub fn new() -> Self {
            Self {
                regions: RefCell::new(Vec::new()),
                next_id: RefCell::new(1),
                mapping_tracker: MockMappingTracker::new(),
            }
        }

        /// Get all regions for verification
        pub fn all_regions(&self) -> Vec<MockRegion> {
            self.regions.borrow().clone()
        }

        /// Inject a pre-existing region for testing
        pub fn inject_region(&self, region: MockRegion) {
            self.regions.borrow_mut().push(region);
        }
    }

    impl ShmemBackend for MockShmemBackend {
        fn create(&self, owner: Pid, size: usize) -> Result<(ShmemId, PhysAddr, usize), ShmemError> {
            if size == 0 {
                return Err(ShmemError::InvalidSize);
            }
            let id = *self.next_id.borrow();
            *self.next_id.borrow_mut() += 1;

            // Fake physical address
            let phys_addr = 0x4000_0000 + (id as u64 * 0x1000);
            let aligned_size = (size + 0xFFF) & !0xFFF;

            let region = MockRegion {
                id,
                owner,
                phys_addr,
                size: aligned_size,
                state: TraitShmemState::Active,
                allowed: vec![owner],
            };
            self.regions.borrow_mut().push(region);

            // Add owner's mapping
            self.mapping_tracker.add_mapping(id, owner)?;

            Ok((id, phys_addr, aligned_size))
        }

        fn get(&self, id: ShmemId) -> Option<&dyn SharedMemory> {
            // Mock doesn't support this - return None
            None
        }

        fn map(&self, id: ShmemId, pid: Pid) -> Result<(PhysAddr, usize), ShmemError> {
            let regions = self.regions.borrow();
            let region = regions.iter().find(|r| r.id == id).ok_or(ShmemError::NotFound)?;

            if region.state == TraitShmemState::Dying {
                return Err(ShmemError::Dying);
            }
            if !region.allowed.contains(&pid) {
                return Err(ShmemError::NotAllowed);
            }

            // Add to mapping tracker (will fail if already mapped)
            drop(regions);
            self.mapping_tracker.add_mapping(id, pid)?;

            let regions = self.regions.borrow();
            let region = regions.iter().find(|r| r.id == id).unwrap();
            Ok((region.phys_addr, region.size))
        }

        fn unmap(&self, id: ShmemId, pid: Pid) -> Result<(), ShmemError> {
            self.mapping_tracker.remove_mapping(id, pid)
        }

        fn destroy(&self, id: ShmemId, owner: Pid) -> Result<(), ShmemError> {
            let mut regions = self.regions.borrow_mut();
            let idx = regions.iter().position(|r| r.id == id).ok_or(ShmemError::NotFound)?;

            if regions[idx].owner != owner {
                return Err(ShmemError::NotOwner);
            }

            // Check no other mappings
            let mapping_count = self.mapping_tracker.mapping_count(id);
            if mapping_count > 1 {
                return Err(ShmemError::NotMapped); // Still mapped by others
            }

            self.mapping_tracker.remove_all_for_region(id);
            regions.remove(idx);
            Ok(())
        }

        fn cleanup_process(&self, pid: Pid) {
            // Remove all mappings for this pid
            self.mapping_tracker.remove_all_for_pid(pid);

            // Destroy regions owned by this pid
            self.regions.borrow_mut().retain(|r| r.owner != pid);
        }

        fn mapping_tracker(&self) -> &dyn MappingTracker {
            &self.mapping_tracker
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::mock::*;

    #[test]
    fn test_mock_create_region() {
        let backend = MockShmemBackend::new();
        let result = backend.create(1, 4096);
        assert!(result.is_ok());
        let (id, phys, size) = result.unwrap();
        assert!(id > 0);
        assert!(phys > 0);
        assert_eq!(size, 4096);
    }

    #[test]
    fn test_mock_create_zero_size_fails() {
        let backend = MockShmemBackend::new();
        let result = backend.create(1, 0);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ShmemError::InvalidSize);
    }

    #[test]
    fn test_mock_mapping_tracker_double_map() {
        let tracker = MockMappingTracker::new();

        // First add should succeed
        assert!(tracker.add_mapping(1, 100).is_ok());
        assert!(tracker.has_mapping(1, 100));

        // Second add should fail with AlreadyMapped
        let result = tracker.add_mapping(1, 100);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ShmemError::AlreadyMapped);
    }

    #[test]
    fn test_mock_mapping_tracker_double_free() {
        let tracker = MockMappingTracker::new();

        // Add mapping
        tracker.add_mapping(1, 100).unwrap();

        // First remove should succeed
        assert!(tracker.remove_mapping(1, 100).is_ok());

        // Second remove should fail with NotMapped
        let result = tracker.remove_mapping(1, 100);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ShmemError::NotMapped);
    }

    #[test]
    fn test_mock_map_unmap_flow() {
        let backend = MockShmemBackend::new();

        // Create region as owner
        let (id, _phys, _size) = backend.create(1, 4096).unwrap();

        // Owner is already mapped from create
        assert!(backend.mapping_tracker().has_mapping(id, 1));

        // Allow another process
        {
            let mut regions = backend.regions.borrow_mut();
            regions[0].allowed.push(2);
        }

        // Map as pid 2
        let result = backend.map(id, 2);
        assert!(result.is_ok());
        assert!(backend.mapping_tracker().has_mapping(id, 2));

        // Unmap as pid 2
        assert!(backend.unmap(id, 2).is_ok());
        assert!(!backend.mapping_tracker().has_mapping(id, 2));
    }

    #[test]
    fn test_mock_destroy_while_mapped() {
        let backend = MockShmemBackend::new();

        // Create region
        let (id, _phys, _size) = backend.create(1, 4096).unwrap();

        // Allow and map as pid 2
        {
            let mut regions = backend.regions.borrow_mut();
            regions[0].allowed.push(2);
        }
        backend.map(id, 2).unwrap();

        // Try to destroy - should fail because pid 2 has it mapped
        let result = backend.destroy(id, 1);
        assert!(result.is_err());

        // Unmap pid 2
        backend.unmap(id, 2).unwrap();

        // Now destroy should succeed (only owner has it)
        assert!(backend.destroy(id, 1).is_ok());
    }

    #[test]
    fn test_mock_cleanup_process() {
        let backend = MockShmemBackend::new();

        // Create region as pid 1
        let (id1, _, _) = backend.create(1, 4096).unwrap();

        // Create region as pid 2
        let (id2, _, _) = backend.create(2, 4096).unwrap();

        // Cleanup pid 1 - should remove id1
        backend.cleanup_process(1);

        // id1 should be gone
        assert!(backend.all_regions().iter().find(|r| r.id == id1).is_none());

        // id2 should still exist
        assert!(backend.all_regions().iter().find(|r| r.id == id2).is_some());
    }

    // ========================================================================
    // Additional Lifecycle Tests
    // ========================================================================

    #[test]
    fn test_shmem_full_lifecycle() {
        let backend = MockShmemBackend::new();

        // 1. Create region
        let (id, phys, size) = backend.create(1, 4096).unwrap();
        assert!(id > 0);
        assert!(phys > 0);
        assert_eq!(size, 4096);

        // 2. Allow another process
        {
            let mut regions = backend.regions.borrow_mut();
            regions[0].allowed.push(2);
        }

        // 3. Second process maps
        let result = backend.map(id, 2);
        assert!(result.is_ok());
        let (mapped_phys, mapped_size) = result.unwrap();
        assert_eq!(mapped_phys, phys);
        assert_eq!(mapped_size, size);

        // 4. Verify mapping count
        assert_eq!(backend.mapping_tracker().mapping_count(id), 2);

        // 5. Second process unmaps
        assert!(backend.unmap(id, 2).is_ok());
        assert_eq!(backend.mapping_tracker().mapping_count(id), 1);

        // 6. Owner destroys
        assert!(backend.destroy(id, 1).is_ok());

        // 7. Region is gone
        assert!(backend.all_regions().is_empty());
    }

    #[test]
    fn test_only_owner_can_destroy() {
        let backend = MockShmemBackend::new();

        // Create region as pid 1
        let (id, _, _) = backend.create(1, 4096).unwrap();

        // Pid 2 tries to destroy - should fail
        let result = backend.destroy(id, 2);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ShmemError::NotOwner);

        // Owner can destroy
        assert!(backend.destroy(id, 1).is_ok());
    }

    #[test]
    fn test_dying_state_blocks_mapping() {
        let backend = MockShmemBackend::new();

        // Create region
        let (id, _, _) = backend.create(1, 4096).unwrap();

        // Allow pid 2
        {
            let mut regions = backend.regions.borrow_mut();
            regions[0].allowed.push(2);
        }

        // Set to dying state
        {
            let mut regions = backend.regions.borrow_mut();
            regions[0].state = TraitShmemState::Dying;
        }

        // Pid 2 tries to map - should fail because dying
        let result = backend.map(id, 2);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ShmemError::Dying);
    }

    #[test]
    fn test_map_without_permission() {
        let backend = MockShmemBackend::new();

        // Create region as pid 1
        let (id, _, _) = backend.create(1, 4096).unwrap();

        // Pid 2 NOT allowed, tries to map
        let result = backend.map(id, 2);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ShmemError::NotAllowed);
    }

    #[test]
    fn test_map_nonexistent_region() {
        let backend = MockShmemBackend::new();

        // Try to map nonexistent region
        let result = backend.map(999, 1);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ShmemError::NotFound);
    }

    #[test]
    fn test_destroy_nonexistent_region() {
        let backend = MockShmemBackend::new();

        // Try to destroy nonexistent region
        let result = backend.destroy(999, 1);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ShmemError::NotFound);
    }

    #[test]
    fn test_multiple_regions_same_owner() {
        let backend = MockShmemBackend::new();

        // Create multiple regions as same owner
        let (id1, _, _) = backend.create(1, 4096).unwrap();
        let (id2, _, _) = backend.create(1, 8192).unwrap();
        let (id3, _, _) = backend.create(1, 16384).unwrap();

        // All should be different
        assert_ne!(id1, id2);
        assert_ne!(id2, id3);
        assert_ne!(id1, id3);

        // All should exist
        assert_eq!(backend.all_regions().len(), 3);

        // Cleanup removes all for this owner
        backend.cleanup_process(1);
        assert!(backend.all_regions().is_empty());
    }

    #[test]
    fn test_mapping_tracker_remove_all_for_region() {
        let tracker = MockMappingTracker::new();

        // Add mappings for region 1
        tracker.add_mapping(1, 100).unwrap();
        tracker.add_mapping(1, 101).unwrap();
        tracker.add_mapping(1, 102).unwrap();

        // Add mapping for region 2
        tracker.add_mapping(2, 100).unwrap();

        // Remove all for region 1
        tracker.remove_all_for_region(1);

        // Region 1 mappings gone
        assert!(!tracker.has_mapping(1, 100));
        assert!(!tracker.has_mapping(1, 101));
        assert!(!tracker.has_mapping(1, 102));

        // Region 2 mapping still there
        assert!(tracker.has_mapping(2, 100));
    }

    #[test]
    fn test_mapping_tracker_remove_all_for_pid() {
        let tracker = MockMappingTracker::new();

        // Pid 100 maps multiple regions
        tracker.add_mapping(1, 100).unwrap();
        tracker.add_mapping(2, 100).unwrap();
        tracker.add_mapping(3, 100).unwrap();

        // Pid 101 maps one region
        tracker.add_mapping(1, 101).unwrap();

        // Remove all for pid 100
        tracker.remove_all_for_pid(100);

        // Pid 100 mappings gone
        assert!(!tracker.has_mapping(1, 100));
        assert!(!tracker.has_mapping(2, 100));
        assert!(!tracker.has_mapping(3, 100));

        // Pid 101 mapping still there
        assert!(tracker.has_mapping(1, 101));
    }

    #[test]
    fn test_mapping_count() {
        let tracker = MockMappingTracker::new();

        // Initially zero
        assert_eq!(tracker.mapping_count(1), 0);

        // Add mappings
        tracker.add_mapping(1, 100).unwrap();
        assert_eq!(tracker.mapping_count(1), 1);

        tracker.add_mapping(1, 101).unwrap();
        assert_eq!(tracker.mapping_count(1), 2);

        tracker.add_mapping(1, 102).unwrap();
        assert_eq!(tracker.mapping_count(1), 3);

        // Remove one
        tracker.remove_mapping(1, 101).unwrap();
        assert_eq!(tracker.mapping_count(1), 2);
    }

    #[test]
    fn test_get_mapped_pids() {
        let tracker = MockMappingTracker::new();

        // Add mappings
        tracker.add_mapping(1, 100).unwrap();
        tracker.add_mapping(1, 101).unwrap();
        tracker.add_mapping(1, 102).unwrap();

        let pids = tracker.get_mapped_pids(1);

        // First 3 elements should be our pids (order may vary)
        let expected: Vec<Pid> = vec![100, 101, 102];
        for pid in expected {
            assert!(pids.contains(&pid), "Missing pid {}", pid);
        }
    }

    #[test]
    fn test_size_alignment() {
        let backend = MockShmemBackend::new();

        // Create with non-aligned size
        let (_, _, size) = backend.create(1, 100).unwrap();

        // Should be page-aligned (4096)
        assert_eq!(size, 4096);

        // Create with slightly over one page
        let (_, _, size) = backend.create(1, 4097).unwrap();

        // Should be 2 pages
        assert_eq!(size, 8192);
    }
}
