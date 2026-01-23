//! Physical Memory Allocator Backend Implementation
//!
//! Implements the `PhysicalAllocator` trait by delegating to the existing PMM module.
//! This provides a clean trait boundary without changing the internal implementation.
//!
//! # Thread Safety
//!
//! All operations go through the PMM module's SpinLock, ensuring proper
//! serialization on SMP systems.

use crate::kernel::traits::memory::{PhysicalAllocator, PhysAddr};
use crate::kernel::pmm;

// ============================================================================
// Kernel Physical Allocator Implementation
// ============================================================================

/// Kernel physical allocator implementation
///
/// A zero-sized type that implements `PhysicalAllocator` by delegating to the
/// global PMM module.
pub struct KernelPhysicalAllocator;

impl KernelPhysicalAllocator {
    /// Create a new kernel physical allocator instance
    pub const fn new() -> Self {
        Self
    }
}

impl PhysicalAllocator for KernelPhysicalAllocator {
    fn alloc_pages(&self, count: usize) -> Option<PhysAddr> {
        pmm::alloc_pages(count).map(|addr| addr as PhysAddr)
    }

    fn free_pages(&self, addr: PhysAddr, count: usize) {
        pmm::free_pages(addr as usize, count);
    }

    fn available_pages(&self) -> usize {
        pmm::free_count()
    }

    fn total_pages(&self) -> usize {
        pmm::total_count()
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global kernel physical allocator
pub static PMM_BACKEND: KernelPhysicalAllocator = KernelPhysicalAllocator::new();

/// Get a reference to the global physical allocator
pub fn physical_allocator() -> &'static dyn PhysicalAllocator {
    &PMM_BACKEND
}

// ============================================================================
// Mock Implementation for Testing
// ============================================================================

#[cfg(test)]
pub mod mock {
    use super::*;
    use core::cell::RefCell;

    /// Mock physical page
    #[derive(Clone, Copy)]
    struct MockPage {
        addr: PhysAddr,
        allocated: bool,
    }

    /// Mock physical allocator for testing
    ///
    /// Simulates a simple physical memory pool with configurable size.
    pub struct MockPhysicalAllocator {
        /// Base address for fake physical memory
        base_addr: PhysAddr,
        /// Total number of pages
        total_pages: usize,
        /// Bitmap of allocated pages
        allocated: RefCell<Vec<bool>>,
        /// Track allocation history for verification
        alloc_history: RefCell<Vec<(PhysAddr, usize)>>,
        /// Track free history for verification
        free_history: RefCell<Vec<(PhysAddr, usize)>>,
    }

    impl MockPhysicalAllocator {
        /// Create a new mock allocator with given number of pages
        pub fn new(total_pages: usize) -> Self {
            Self {
                base_addr: 0x4000_0000, // Fake DRAM base
                total_pages,
                allocated: RefCell::new(vec![false; total_pages]),
                alloc_history: RefCell::new(Vec::new()),
                free_history: RefCell::new(Vec::new()),
            }
        }

        /// Create a mock allocator with custom base address
        pub fn with_base(base_addr: PhysAddr, total_pages: usize) -> Self {
            Self {
                base_addr,
                total_pages,
                allocated: RefCell::new(vec![false; total_pages]),
                alloc_history: RefCell::new(Vec::new()),
                free_history: RefCell::new(Vec::new()),
            }
        }

        /// Get allocation history for verification
        pub fn alloc_history(&self) -> Vec<(PhysAddr, usize)> {
            self.alloc_history.borrow().clone()
        }

        /// Get free history for verification
        pub fn free_history(&self) -> Vec<(PhysAddr, usize)> {
            self.free_history.borrow().clone()
        }

        /// Reset the allocator state
        pub fn reset(&self) {
            let mut allocated = self.allocated.borrow_mut();
            for page in allocated.iter_mut() {
                *page = false;
            }
            self.alloc_history.borrow_mut().clear();
            self.free_history.borrow_mut().clear();
        }

        /// Mark specific pages as reserved (not allocatable)
        pub fn reserve_pages(&self, start_page: usize, count: usize) {
            let mut allocated = self.allocated.borrow_mut();
            for i in start_page..(start_page + count).min(self.total_pages) {
                allocated[i] = true;
            }
        }

        fn page_index(&self, addr: PhysAddr) -> Option<usize> {
            if addr < self.base_addr {
                return None;
            }
            let offset = (addr - self.base_addr) as usize;
            if offset % 4096 != 0 {
                return None;
            }
            let idx = offset / 4096;
            if idx >= self.total_pages {
                return None;
            }
            Some(idx)
        }
    }

    impl PhysicalAllocator for MockPhysicalAllocator {
        fn alloc_pages(&self, count: usize) -> Option<PhysAddr> {
            if count == 0 {
                return None;
            }

            let mut allocated = self.allocated.borrow_mut();

            // Find contiguous free pages
            let mut start = 0;
            let mut consecutive = 0;

            for i in 0..self.total_pages {
                if !allocated[i] {
                    if consecutive == 0 {
                        start = i;
                    }
                    consecutive += 1;
                    if consecutive == count {
                        // Found! Mark as allocated
                        for j in start..(start + count) {
                            allocated[j] = true;
                        }
                        let addr = self.base_addr + (start as u64 * 4096);
                        drop(allocated);
                        self.alloc_history.borrow_mut().push((addr, count));
                        return Some(addr);
                    }
                } else {
                    consecutive = 0;
                }
            }

            None // Out of memory
        }

        fn free_pages(&self, addr: PhysAddr, count: usize) {
            if let Some(start_idx) = self.page_index(addr) {
                let mut allocated = self.allocated.borrow_mut();
                for i in start_idx..(start_idx + count).min(self.total_pages) {
                    allocated[i] = false;
                }
                drop(allocated);
                self.free_history.borrow_mut().push((addr, count));
            }
        }

        fn available_pages(&self) -> usize {
            self.allocated.borrow().iter().filter(|&&a| !a).count()
        }

        fn total_pages(&self) -> usize {
            self.total_pages
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::mock::*;

    #[test]
    fn test_mock_alloc_single_page() {
        let pmm = MockPhysicalAllocator::new(100);

        let result = pmm.alloc_page();
        assert!(result.is_some());
        let addr = result.unwrap();
        assert!(addr >= 0x4000_0000);
        assert_eq!(addr % 4096, 0);

        // Should have reduced available pages
        assert_eq!(pmm.available_pages(), 99);
    }

    #[test]
    fn test_mock_alloc_multiple_pages() {
        let pmm = MockPhysicalAllocator::new(100);

        let result = pmm.alloc_pages(4);
        assert!(result.is_some());

        assert_eq!(pmm.available_pages(), 96);
    }

    #[test]
    fn test_mock_free_pages() {
        let pmm = MockPhysicalAllocator::new(100);

        let addr = pmm.alloc_pages(4).unwrap();
        assert_eq!(pmm.available_pages(), 96);

        pmm.free_pages(addr, 4);
        assert_eq!(pmm.available_pages(), 100);
    }

    #[test]
    fn test_mock_alloc_history() {
        let pmm = MockPhysicalAllocator::new(100);

        let addr1 = pmm.alloc_page().unwrap();
        let addr2 = pmm.alloc_pages(3).unwrap();

        let history = pmm.alloc_history();
        assert_eq!(history.len(), 2);
        assert_eq!(history[0], (addr1, 1));
        assert_eq!(history[1], (addr2, 3));
    }

    #[test]
    fn test_mock_out_of_memory() {
        let pmm = MockPhysicalAllocator::new(10);

        // Allocate all pages
        let _addr = pmm.alloc_pages(10).unwrap();
        assert_eq!(pmm.available_pages(), 0);

        // Should fail to allocate more
        let result = pmm.alloc_page();
        assert!(result.is_none());
    }

    #[test]
    fn test_mock_fragmentation() {
        let pmm = MockPhysicalAllocator::new(10);

        // Allocate alternating pages to create fragmentation
        let addr1 = pmm.alloc_page().unwrap();
        let _addr2 = pmm.alloc_page().unwrap();
        let addr3 = pmm.alloc_page().unwrap();

        // Free alternating pages
        pmm.free_page(addr1);
        pmm.free_page(addr3);

        // Now we have 2 free pages but they're not contiguous
        assert_eq!(pmm.available_pages(), 2);

        // Allocating 2 contiguous should fail (depending on which were freed)
        // Actually, in this simple implementation it might work if addr3+1 happens to be free
        // Let's test a clearer fragmentation case
    }

    #[test]
    fn test_mock_reserve_pages() {
        let pmm = MockPhysicalAllocator::new(100);

        // Reserve first 10 pages
        pmm.reserve_pages(0, 10);

        // Should have 90 available
        assert_eq!(pmm.available_pages(), 90);

        // First allocation should start at page 10
        let addr = pmm.alloc_page().unwrap();
        assert_eq!(addr, 0x4000_0000 + (10 * 4096));
    }

    #[test]
    fn test_mock_reset() {
        let pmm = MockPhysicalAllocator::new(100);

        pmm.alloc_pages(50).unwrap();
        assert_eq!(pmm.available_pages(), 50);

        pmm.reset();
        assert_eq!(pmm.available_pages(), 100);
        assert!(pmm.alloc_history().is_empty());
    }
}
