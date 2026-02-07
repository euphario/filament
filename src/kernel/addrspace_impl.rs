//! Address Space Backend Implementation
//!
//! Implements the `AddressSpaceBackend` trait by delegating to the existing
//! addrspace module. This provides a clean trait boundary for testing.
//!
//! # Design Notes
//!
//! The kernel's address space design uses per-process AddressSpace structs,
//! while the trait abstracts operations by ASID. The kernel implementation
//! delegates ASID/TLB operations to the global modules, but page mapping
//! operations require the caller to have access to the AddressSpace.
//!
//! For testing, the MockAddressSpaceBackend tracks all operations in memory.

use crate::kernel::traits::addrspace::{
    AddressSpaceBackend, Asid, VirtAddr, PhysAddr, PageFlags,
};
use crate::kernel::addrspace;
use crate::kernel::arch::{mmu, tlb};

// ============================================================================
// Kernel Address Space Backend
// ============================================================================

/// Kernel address space backend implementation
///
/// A zero-sized type that implements `AddressSpaceBackend` by delegating to
/// the existing address space and TLB modules.
///
/// # Limitations
///
/// The page mapping operations (map_page, unmap_page, is_mapped) are not
/// fully implemented here because the kernel stores page tables per-process
/// in AddressSpace structs, not globally by ASID. These operations return
/// failure/None since the trait doesn't have access to process state.
///
/// For actual page mapping, use the Process's AddressSpace directly.
/// This backend is primarily useful for:
/// - ASID allocation/deallocation
/// - TLB invalidation
/// - TTBR0 activation
pub struct KernelAddressSpaceBackend;

impl KernelAddressSpaceBackend {
    /// Create a new kernel address space backend instance
    pub const fn new() -> Self {
        Self
    }
}

impl AddressSpaceBackend for KernelAddressSpaceBackend {
    // ========================================================================
    // ASID Operations - fully supported
    // ========================================================================

    fn alloc_asid(&self) -> Option<Asid> {
        addrspace::alloc_asid_external()
    }

    fn free_asid(&self, asid: Asid) {
        addrspace::free_asid_external(asid);
    }

    fn asid_count(&self) -> usize {
        addrspace::asid_count_external()
    }

    // ========================================================================
    // Page Table Operations - limited support
    //
    // These operations require an AddressSpace instance that we don't have
    // access to through this trait. The kernel stores page tables per-process.
    // These are primarily useful in the mock for testing.
    // ========================================================================

    fn map_page(
        &self,
        _asid: Asid,
        _virt: VirtAddr,
        _phys: PhysAddr,
        _flags: PageFlags,
    ) -> bool {
        // Cannot implement without AddressSpace reference.
        // Use Process.addrspace.map_page() directly in kernel code.
        false
    }

    fn unmap_page(&self, _asid: Asid, _virt: VirtAddr) -> Option<PhysAddr> {
        // Cannot implement without AddressSpace reference.
        // Use Process.addrspace.unmap_page() directly in kernel code.
        None
    }

    fn is_mapped(&self, _asid: Asid, _virt: VirtAddr) -> bool {
        // Cannot implement without AddressSpace reference.
        false
    }

    // ========================================================================
    // TLB Operations - fully supported
    // ========================================================================

    fn invalidate_page(&self, asid: Asid, virt: VirtAddr) {
        tlb::invalidate_va(asid, virt);
    }

    fn invalidate_asid(&self, asid: Asid) {
        tlb::invalidate_asid(asid);
    }

    // ========================================================================
    // Address Space Activation - fully supported
    // ========================================================================

    fn activate(&self, ttbr0: u64) {
        unsafe {
            mmu::switch_user_space(ttbr0);
        }
    }

    fn current_ttbr0(&self) -> u64 {
        mmu::ttbr0()
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global kernel address space backend
pub static ADDRSPACE_BACKEND: KernelAddressSpaceBackend = KernelAddressSpaceBackend::new();

/// Get a reference to the global address space backend
pub fn addrspace_backend() -> &'static dyn AddressSpaceBackend {
    &ADDRSPACE_BACKEND
}

// ============================================================================
// Mock Implementation for Testing
// ============================================================================

#[cfg(test)]
pub mod mock {
    use super::*;
    use core::cell::RefCell;

    /// Page entry for mock tracking
    #[derive(Clone, Debug)]
    pub struct MockPageEntry {
        pub phys: PhysAddr,
        pub flags: PageFlags,
    }

    /// Mock address space backend for testing
    ///
    /// Tracks all operations in memory for verification in tests.
    pub struct MockAddressSpaceBackend {
        /// Allocated ASIDs (true = allocated)
        asids: RefCell<[bool; 256]>,
        /// Next ASID hint for allocation
        next_asid: RefCell<u16>,
        /// Page mappings: (asid, virt_page) -> entry
        /// Uses page number (virt >> 12) as key
        pages: RefCell<[(Option<(Asid, u64)>, Option<MockPageEntry>); 1024]>,
        /// Number of pages mapped
        page_count: RefCell<usize>,
        /// Current TTBR0 value
        current_ttbr0: RefCell<u64>,
        /// TLB invalidation history for verification
        tlb_invalidations: RefCell<Vec<TlbInvalidation>>,
    }

    /// TLB invalidation record
    #[derive(Clone, Debug, PartialEq)]
    pub enum TlbInvalidation {
        Page { asid: Asid, virt: VirtAddr },
        Asid { asid: Asid },
    }

    impl MockAddressSpaceBackend {
        pub fn new() -> Self {
            Self {
                asids: RefCell::new([false; 256]),
                next_asid: RefCell::new(1),
                pages: RefCell::new([(None, None); 1024]),
                page_count: RefCell::new(0),
                current_ttbr0: RefCell::new(0),
                tlb_invalidations: RefCell::new(Vec::new()),
            }
        }

        /// Find a page slot by key
        fn find_page_slot(&self, asid: Asid, virt_page: u64) -> Option<usize> {
            let pages = self.pages.borrow();
            for (i, (key, _)) in pages.iter().enumerate() {
                if let Some((a, v)) = key {
                    if *a == asid && *v == virt_page {
                        return Some(i);
                    }
                }
            }
            None
        }

        /// Find an empty page slot
        fn find_empty_slot(&self) -> Option<usize> {
            let pages = self.pages.borrow();
            for (i, (key, _)) in pages.iter().enumerate() {
                if key.is_none() {
                    return Some(i);
                }
            }
            None
        }

        /// Get TLB invalidation history
        pub fn tlb_history(&self) -> Vec<TlbInvalidation> {
            self.tlb_invalidations.borrow().clone()
        }

        /// Clear TLB history
        pub fn clear_tlb_history(&self) {
            self.tlb_invalidations.borrow_mut().clear();
        }

        /// Reset all state
        pub fn reset(&self) {
            *self.asids.borrow_mut() = [false; 256];
            *self.next_asid.borrow_mut() = 1;
            *self.pages.borrow_mut() = [(None, None); 1024];
            *self.page_count.borrow_mut() = 0;
            *self.current_ttbr0.borrow_mut() = 0;
            self.tlb_invalidations.borrow_mut().clear();
        }

        /// Get count of mapped pages for an ASID
        pub fn page_count_for_asid(&self, asid: Asid) -> usize {
            let pages = self.pages.borrow();
            pages.iter().filter(|(key, _)| {
                key.map_or(false, |(a, _)| a == asid)
            }).count()
        }

        /// Check if a specific mapping exists
        pub fn has_mapping(&self, asid: Asid, virt: VirtAddr, phys: PhysAddr) -> bool {
            let virt_page = virt >> 12;
            if let Some(idx) = self.find_page_slot(asid, virt_page) {
                let pages = self.pages.borrow();
                if let Some(entry) = &pages[idx].1 {
                    return entry.phys == phys;
                }
            }
            false
        }
    }

    impl AddressSpaceBackend for MockAddressSpaceBackend {
        fn alloc_asid(&self) -> Option<Asid> {
            let mut asids = self.asids.borrow_mut();
            let mut next = *self.next_asid.borrow();

            // ASID 0 is reserved
            for _ in 0..255 {
                if next == 0 {
                    next = 1;
                }
                if next > 255 {
                    next = 1;
                }

                if !asids[next as usize] {
                    asids[next as usize] = true;
                    *self.next_asid.borrow_mut() = next + 1;
                    return Some(next);
                }
                next = if next < 255 { next + 1 } else { 1 };
            }
            None
        }

        fn free_asid(&self, asid: Asid) {
            if asid > 0 && asid <= 255 {
                self.asids.borrow_mut()[asid as usize] = false;
            }
        }

        fn asid_count(&self) -> usize {
            self.asids.borrow().iter().filter(|&&x| x).count()
        }

        fn map_page(
            &self,
            asid: Asid,
            virt: VirtAddr,
            phys: PhysAddr,
            flags: PageFlags,
        ) -> bool {
            let virt_page = virt >> 12;

            // Check if already mapped
            if self.find_page_slot(asid, virt_page).is_some() {
                return false; // Already mapped
            }

            // Find empty slot
            let slot = match self.find_empty_slot() {
                Some(s) => s,
                None => return false, // Out of slots
            };

            // Map it
            let mut pages = self.pages.borrow_mut();
            pages[slot] = (
                Some((asid, virt_page)),
                Some(MockPageEntry { phys, flags }),
            );
            *self.page_count.borrow_mut() += 1;

            true
        }

        fn unmap_page(&self, asid: Asid, virt: VirtAddr) -> Option<PhysAddr> {
            let virt_page = virt >> 12;

            let slot = self.find_page_slot(asid, virt_page)?;

            let mut pages = self.pages.borrow_mut();
            let phys = pages[slot].1.as_ref().map(|e| e.phys);
            pages[slot] = (None, None);
            *self.page_count.borrow_mut() -= 1;

            phys
        }

        fn is_mapped(&self, asid: Asid, virt: VirtAddr) -> bool {
            let virt_page = virt >> 12;
            self.find_page_slot(asid, virt_page).is_some()
        }

        fn invalidate_page(&self, asid: Asid, virt: VirtAddr) {
            self.tlb_invalidations.borrow_mut().push(
                TlbInvalidation::Page { asid, virt }
            );
        }

        fn invalidate_asid(&self, asid: Asid) {
            self.tlb_invalidations.borrow_mut().push(
                TlbInvalidation::Asid { asid }
            );
        }

        fn activate(&self, ttbr0: u64) {
            *self.current_ttbr0.borrow_mut() = ttbr0;
        }

        fn current_ttbr0(&self) -> u64 {
            *self.current_ttbr0.borrow()
        }
    }
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use super::mock::*;
    use crate::kernel::traits::addrspace::MemoryType;

    #[test]
    fn test_mock_asid_allocation() {
        let backend = MockAddressSpaceBackend::new();

        // Allocate some ASIDs
        let asid1 = backend.alloc_asid();
        let asid2 = backend.alloc_asid();
        let asid3 = backend.alloc_asid();

        assert!(asid1.is_some());
        assert!(asid2.is_some());
        assert!(asid3.is_some());

        // All different
        assert_ne!(asid1, asid2);
        assert_ne!(asid2, asid3);
        assert_ne!(asid1, asid3);

        // Count should be 3
        assert_eq!(backend.asid_count(), 3);

        // Free one
        backend.free_asid(asid2.unwrap());
        assert_eq!(backend.asid_count(), 2);

        // Allocate again - should reuse freed ASID eventually
        let asid4 = backend.alloc_asid();
        assert!(asid4.is_some());
        assert_eq!(backend.asid_count(), 3);
    }

    #[test]
    fn test_mock_asid_exhaustion() {
        let backend = MockAddressSpaceBackend::new();

        // Allocate all 255 ASIDs (0 is reserved)
        for _ in 0..255 {
            assert!(backend.alloc_asid().is_some());
        }

        // Next allocation should fail
        assert!(backend.alloc_asid().is_none());
        assert_eq!(backend.asid_count(), 255);

        // Free one, should be able to allocate again
        backend.free_asid(100);
        assert_eq!(backend.asid_count(), 254);

        let asid = backend.alloc_asid();
        assert!(asid.is_some());
        assert_eq!(backend.asid_count(), 255);
    }

    #[test]
    fn test_mock_page_mapping() {
        let backend = MockAddressSpaceBackend::new();
        let asid = backend.alloc_asid().unwrap();

        let virt: VirtAddr = 0x4000_0000;
        let phys: PhysAddr = 0x8000_0000;
        let flags = PageFlags::code();

        // Map a page
        assert!(backend.map_page(asid, virt, phys, flags));

        // Check it's mapped
        assert!(backend.is_mapped(asid, virt));
        assert!(backend.has_mapping(asid, virt, phys));

        // Can't map again at same address
        assert!(!backend.map_page(asid, virt, 0x9000_0000, flags));

        // Unmap it
        let unmapped = backend.unmap_page(asid, virt);
        assert_eq!(unmapped, Some(phys));

        // No longer mapped
        assert!(!backend.is_mapped(asid, virt));

        // Can map again
        assert!(backend.map_page(asid, virt, 0xA000_0000, PageFlags::data()));
        assert!(backend.has_mapping(asid, virt, 0xA000_0000));
    }

    #[test]
    fn test_mock_multiple_pages() {
        let backend = MockAddressSpaceBackend::new();
        let asid = backend.alloc_asid().unwrap();

        // Map multiple pages
        for i in 0..10 {
            let virt = 0x4000_0000 + (i as u64 * 0x1000);
            let phys = 0x8000_0000 + (i as u64 * 0x1000);
            assert!(backend.map_page(asid, virt, phys, PageFlags::data()));
        }

        assert_eq!(backend.page_count_for_asid(asid), 10);

        // Unmap every other page
        for i in (0..10).step_by(2) {
            let virt = 0x4000_0000 + (i as u64 * 0x1000);
            assert!(backend.unmap_page(asid, virt).is_some());
        }

        assert_eq!(backend.page_count_for_asid(asid), 5);
    }

    #[test]
    fn test_mock_different_asids() {
        let backend = MockAddressSpaceBackend::new();
        let asid1 = backend.alloc_asid().unwrap();
        let asid2 = backend.alloc_asid().unwrap();

        let virt: VirtAddr = 0x4000_0000;

        // Map same virtual address in different ASIDs
        assert!(backend.map_page(asid1, virt, 0x1000_0000, PageFlags::data()));
        assert!(backend.map_page(asid2, virt, 0x2000_0000, PageFlags::data()));

        // Both are mapped independently
        assert!(backend.has_mapping(asid1, virt, 0x1000_0000));
        assert!(backend.has_mapping(asid2, virt, 0x2000_0000));

        // Unmap from asid1 doesn't affect asid2
        backend.unmap_page(asid1, virt);
        assert!(!backend.is_mapped(asid1, virt));
        assert!(backend.is_mapped(asid2, virt));
    }

    #[test]
    fn test_mock_tlb_invalidation() {
        let backend = MockAddressSpaceBackend::new();
        let asid = backend.alloc_asid().unwrap();

        // Invalidate a page
        backend.invalidate_page(asid, 0x4000_0000);
        backend.invalidate_page(asid, 0x4000_1000);

        // Invalidate entire ASID
        backend.invalidate_asid(asid);

        // Check history
        let history = backend.tlb_history();
        assert_eq!(history.len(), 3);
        assert_eq!(history[0], TlbInvalidation::Page { asid, virt: 0x4000_0000 });
        assert_eq!(history[1], TlbInvalidation::Page { asid, virt: 0x4000_1000 });
        assert_eq!(history[2], TlbInvalidation::Asid { asid });

        // Clear history
        backend.clear_tlb_history();
        assert!(backend.tlb_history().is_empty());
    }

    #[test]
    fn test_mock_ttbr0() {
        let backend = MockAddressSpaceBackend::new();

        // Initial value is 0
        assert_eq!(backend.current_ttbr0(), 0);

        // Activate with a value
        backend.activate(0x1234_5678_9ABC_DEF0);
        assert_eq!(backend.current_ttbr0(), 0x1234_5678_9ABC_DEF0);

        // Change it
        backend.activate(0xDEAD_BEEF_CAFE_0000);
        assert_eq!(backend.current_ttbr0(), 0xDEAD_BEEF_CAFE_0000);
    }

    #[test]
    fn test_mock_page_flags() {
        let backend = MockAddressSpaceBackend::new();
        let asid = backend.alloc_asid().unwrap();

        // Map with different flag types
        assert!(backend.map_page(asid, 0x4000_0000, 0x1000_0000, PageFlags::code()));
        assert!(backend.map_page(asid, 0x4000_1000, 0x2000_0000, PageFlags::data()));
        assert!(backend.map_page(asid, 0x4000_2000, 0x3000_0000, PageFlags::rodata()));
        assert!(backend.map_page(asid, 0x4000_3000, 0x4000_0000, PageFlags::device()));
        assert!(backend.map_page(asid, 0x4000_4000, 0x5000_0000, PageFlags::dma()));

        // All mapped
        assert_eq!(backend.page_count_for_asid(asid), 5);
    }

    #[test]
    fn test_mock_reset() {
        let backend = MockAddressSpaceBackend::new();

        // Set up some state
        let asid = backend.alloc_asid().unwrap();
        backend.map_page(asid, 0x4000_0000, 0x1000_0000, PageFlags::data());
        backend.invalidate_page(asid, 0x4000_0000);
        backend.activate(0x1234);

        // Reset
        backend.reset();

        // All state should be cleared
        assert_eq!(backend.asid_count(), 0);
        assert!(!backend.is_mapped(asid, 0x4000_0000));
        assert!(backend.tlb_history().is_empty());
        assert_eq!(backend.current_ttbr0(), 0);
    }
}
