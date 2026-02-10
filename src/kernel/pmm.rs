//! Physical Memory Manager (PMM)
//!
//! External metadata allocator - production-quality page frame management.
//!
//! ## Design
//!
//! Each physical page has a corresponding `PageFrame` struct stored in an
//! external array (not in the page data itself). This allows:
//! - Page content to be modified without corrupting the allocator
//! - Reference counting for shared pages
//! - Page flags (kernel, user, DMA, reserved)
//! - O(1) allocation and freeing
//!
//! The PageFrame array is allocated at boot from reserved memory after the
//! kernel, not embedded in the kernel image. This keeps the kernel small
//! and allows adaptation to different RAM sizes.
//!
//! ## Memory Layout
//!
//! ```text
//! DRAM_BASE ──────────────────────────────────────────
//!            │ Reserved (U-Boot, DTB, etc.)        │
//!            ├─────────────────────────────────────────
//!            │ Kernel code + data + BSS            │
//! __kernel_end ────────────────────────────────────────
//!            │ Kernel reserved (stack margin)      │
//! PMM_FRAMES_BASE ─────────────────────────────────────
//!            │ PageFrame array (8 bytes per page)  │
//!            │ For 4GB: 1M pages × 8 = 8MB         │
//! ─────────────────────────────────────────────────────
//!            │ Free memory for allocation          │
//! DRAM_END ──────────────────────────────────────────
//! ```
//!
//! ## Thread Safety
//!
//! All operations are protected by a SpinLock with IRQ-save semantics.
//! Safe to call from both process and interrupt context.

use crate::{kinfo, kwarn, kdebug, print_direct, klog};
use crate::kernel::arch::mmu;
use crate::platform::current::{DRAM_BASE, DRAM_END, KERNEL_PHYS_BASE,
    DMA_POOL_BASE, DMA_POOL_SIZE, DMA_POOL_HIGH_BASE, DMA_POOL_HIGH_SIZE};
use super::lock::SpinLock;

/// Page size (4KB) - re-exported for other modules
pub const PAGE_SIZE: usize = crate::platform::current::PAGE_SIZE;

/// Sentinel value for end of free list
const FREE_LIST_END: u32 = u32::MAX;

// Linker symbols for kernel boundaries
extern "C" {
    static __kernel_start: u8;
    static __kernel_end: u8;
}

/// Get kernel size in bytes
fn kernel_size() -> usize {
    unsafe {
        let virt_end = &__kernel_end as *const u8 as usize;
        let virt_start = &__kernel_start as *const u8 as usize;
        virt_end - virt_start
    }
}

// ============================================================================
// Page Frame Metadata
// ============================================================================

/// Page state flags
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PageState {
    /// Page is free and available for allocation
    Free = 0,
    /// Page is allocated and in use
    Used = 1,
    /// Page is reserved (kernel, metadata, U-Boot, etc.)
    Reserved = 2,
}

/// External metadata for each physical page frame
///
/// This struct lives OUTSIDE the page data, so page content can be
/// freely modified without affecting the allocator.
#[derive(Clone, Copy)]
#[repr(C)]
pub struct PageFrame {
    /// Index of next free page (FREE_LIST_END = end of list)
    next_free: u32,
    /// Page state
    state: PageState,
    /// Reference count (for shared pages, copy-on-write)
    ref_count: u8,
    /// Reserved for future use
    _reserved: [u8; 2],
}

impl PageFrame {
    const fn new() -> Self {
        Self {
            next_free: FREE_LIST_END,
            state: PageState::Free,
            ref_count: 0,
            _reserved: [0; 2],
        }
    }
}

// Compile-time check: PageFrame should be 8 bytes
const _: () = assert!(core::mem::size_of::<PageFrame>() == 8);

// ============================================================================
// Physical Memory Manager
// ============================================================================

/// Physical page frame allocator with external metadata
pub struct PhysicalMemoryManager {
    /// Pointer to PageFrame array (allocated at boot, not in kernel image)
    frames: *mut PageFrame,
    /// Number of page frames
    num_frames: usize,
    /// Head of free list (page index, FREE_LIST_END = empty)
    free_list_head: u32,
    /// Number of free pages
    free_pages: usize,
    /// First allocatable page index (after metadata region)
    first_free_page: usize,
    /// Whether PMM is initialized
    initialized: bool,
}

// Safety: PMM is protected by SpinLock and only accessed through lock
unsafe impl Send for PhysicalMemoryManager {}
unsafe impl Sync for PhysicalMemoryManager {}

impl PhysicalMemoryManager {
    pub const fn new() -> Self {
        Self {
            frames: core::ptr::null_mut(),
            num_frames: 0,
            free_list_head: FREE_LIST_END,
            free_pages: 0,
            first_free_page: 0,
            initialized: false,
        }
    }

    /// Get a frame by index
    #[inline]
    fn frame(&self, idx: usize) -> &PageFrame {
        debug_assert!(idx < self.num_frames);
        unsafe { &*self.frames.add(idx) }
    }

    /// Get a mutable frame by index
    #[inline]
    fn frame_mut(&mut self, idx: usize) -> &mut PageFrame {
        debug_assert!(idx < self.num_frames);
        unsafe { &mut *self.frames.add(idx) }
    }

    /// Initialize the allocator
    ///
    /// Memory layout after init:
    /// - Pages 0..16: Reserved (U-Boot, DTB)
    /// - Pages kernel_start..kernel_end+margin: Reserved (kernel)
    /// - Pages metadata_start..metadata_end: Reserved (PageFrame array)
    /// - Pages metadata_end..: Free for allocation
    pub fn init(&mut self) {
        if self.initialized {
            return;
        }

        // Calculate number of pages in system
        let total_pages = (DRAM_END - DRAM_BASE) / PAGE_SIZE;
        self.num_frames = total_pages;

        // Calculate kernel reserved region (with 256KB safety margin)
        let k_size = kernel_size();
        let k_reserved_pages = (k_size + PAGE_SIZE - 1) / PAGE_SIZE + 64;
        let kernel_start_page = (KERNEL_PHYS_BASE - DRAM_BASE) / PAGE_SIZE;
        let kernel_end_page = kernel_start_page + k_reserved_pages;

        // Calculate PageFrame array size and location
        // Place it right after kernel reserved region
        let frames_size = total_pages * core::mem::size_of::<PageFrame>();
        let frames_pages = (frames_size + PAGE_SIZE - 1) / PAGE_SIZE;
        let frames_start_page = kernel_end_page;
        let frames_end_page = frames_start_page + frames_pages;

        // Physical address of PageFrame array
        let frames_phys = DRAM_BASE + frames_start_page * PAGE_SIZE;
        self.frames = mmu::phys_to_virt(frames_phys as u64) as *mut PageFrame;

        // First page available for allocation
        self.first_free_page = frames_end_page;

        // DMA pool reserved regions (managed by dma_pool.rs bump allocator)
        let dma_start_page = (DMA_POOL_BASE as usize).saturating_sub(DRAM_BASE) / PAGE_SIZE;
        let dma_end_page = dma_start_page + DMA_POOL_SIZE / PAGE_SIZE;
        let dma_high_start_page = (DMA_POOL_HIGH_BASE as usize).saturating_sub(DRAM_BASE) / PAGE_SIZE;
        let dma_high_end_page = dma_high_start_page + DMA_POOL_HIGH_SIZE / PAGE_SIZE;

        // Initialize all frames
        for i in 0..total_pages {
            let frame = self.frame_mut(i);
            frame.next_free = FREE_LIST_END;
            frame.ref_count = 0;

            if i < 16 {
                // Reserved: U-Boot, DTB, etc.
                frame.state = PageState::Reserved;
            } else if i >= kernel_start_page && i < kernel_end_page {
                // Reserved: Kernel
                frame.state = PageState::Reserved;
            } else if i >= frames_start_page && i < frames_end_page {
                // Reserved: PageFrame array itself
                frame.state = PageState::Reserved;
            } else if i >= dma_start_page && i < dma_end_page {
                // Reserved: DMA pool (managed by dma_pool.rs)
                frame.state = PageState::Reserved;
            } else if i >= dma_high_start_page && i < dma_high_end_page {
                // Reserved: DMA pool high (managed by dma_pool.rs)
                frame.state = PageState::Reserved;
            } else {
                frame.state = PageState::Free;
            }
        }

        // Count free pages and build free list
        self.free_pages = 0;
        self.free_list_head = FREE_LIST_END;

        // Build free list in reverse order (lower addresses at head)
        for i in (self.first_free_page..total_pages).rev() {
            if self.frame(i).state == PageState::Free {
                self.push_free_list(i);
                self.free_pages += 1;
            }
        }

        self.initialized = true;

        let free_mb = self.free_pages * PAGE_SIZE / (1024 * 1024);

        // Debug: direct UART output
        print_direct!("PMM: {} free pages, {} MB free\r\n", self.free_pages, free_mb);

        kinfo!("pmm", "init_ok"; free_mb = free_mb as u64);
    }

    /// Push a page onto the free list (O(1))
    #[inline]
    fn push_free_list(&mut self, page_idx: usize) {
        debug_assert!(page_idx < self.num_frames);
        self.frame_mut(page_idx).next_free = self.free_list_head;
        self.free_list_head = page_idx as u32;
    }

    /// Pop a page from the free list (O(1))
    #[inline]
    fn pop_free_list(&mut self) -> Option<usize> {
        if self.free_list_head == FREE_LIST_END {
            return None;
        }

        let page_idx = self.free_list_head as usize;
        debug_assert!(page_idx < self.num_frames);

        self.free_list_head = self.frame(page_idx).next_free;
        self.frame_mut(page_idx).next_free = FREE_LIST_END;

        Some(page_idx)
    }

    /// Allocate a single page (O(1))
    pub fn alloc_page(&mut self) -> Option<usize> {
        let page_idx = self.pop_free_list()?;
        self.frame_mut(page_idx).state = PageState::Used;
        self.frame_mut(page_idx).ref_count = 1;
        self.free_pages -= 1;

        Some(DRAM_BASE + page_idx * PAGE_SIZE)
    }

    /// Allocate contiguous pages (O(n) scan)
    pub fn alloc_pages(&mut self, count: usize) -> Option<usize> {
        if count == 0 {
            return None;
        }
        if count == 1 {
            return self.alloc_page();
        }

        // Scan for contiguous free pages
        let mut start_page = self.first_free_page;
        let mut consecutive = 0;
        let mut max_consecutive = 0;
        let mut scanned = 0;

        for page_idx in self.first_free_page..self.num_frames {
            scanned += 1;
            if self.frame(page_idx).state == PageState::Free {
                if consecutive == 0 {
                    start_page = page_idx;
                }
                consecutive += 1;
                if consecutive > max_consecutive {
                    max_consecutive = consecutive;
                }
                if consecutive == count {
                    // Found enough! Remove from free list and mark as used
                    self.remove_range_from_free_list(start_page, count);

                    for i in start_page..(start_page + count) {
                        self.frame_mut(i).state = PageState::Used;
                        self.frame_mut(i).ref_count = 1;
                    }
                    self.free_pages -= count;

                    return Some(DRAM_BASE + start_page * PAGE_SIZE);
                }
            } else {
                consecutive = 0;
            }
        }

        // Debug: log failure for large allocations
        if count >= 256 {
            kwarn!("pmm", "alloc_failed";
                requested = count as u64,
                scanned = scanned as u64,
                max_consecutive = max_consecutive as u64,
                free = self.free_pages as u64,
                first_free = self.first_free_page as u64
            );
        }
        None
    }

    /// Remove a range of pages from the free list
    fn remove_range_from_free_list(&mut self, start: usize, count: usize) {
        let end = start + count;

        // Handle head of list
        while self.free_list_head != FREE_LIST_END {
            let head = self.free_list_head as usize;
            if head >= start && head < end {
                self.free_list_head = self.frame(head).next_free;
                self.frame_mut(head).next_free = FREE_LIST_END;
            } else {
                break;
            }
        }

        // Walk the rest
        if self.free_list_head != FREE_LIST_END {
            let mut prev = self.free_list_head as usize;
            while self.frame(prev).next_free != FREE_LIST_END {
                let curr = self.frame(prev).next_free as usize;
                if curr >= start && curr < end {
                    self.frame_mut(prev).next_free = self.frame(curr).next_free;
                    self.frame_mut(curr).next_free = FREE_LIST_END;
                } else {
                    prev = curr;
                }
            }
        }
    }

    /// Free a single page (O(1))
    pub fn free_page(&mut self, phys_addr: usize) {
        if phys_addr < DRAM_BASE || phys_addr >= DRAM_END {
            return;
        }
        if phys_addr & (PAGE_SIZE - 1) != 0 {
            return;
        }

        let page_idx = (phys_addr - DRAM_BASE) / PAGE_SIZE;
        if page_idx >= self.num_frames {
            return;
        }

        if self.frame(page_idx).state == PageState::Used {
            let old_ref = self.frame(page_idx).ref_count;
            if old_ref == 0 {
                crate::kwarn!("pmm", "double_free"; addr = phys_addr as u64, page = page_idx as u64);
                return;
            }
            let new_ref = old_ref - 1;
            self.frame_mut(page_idx).ref_count = new_ref;

            if new_ref == 0 {
                self.frame_mut(page_idx).state = PageState::Free;
                self.push_free_list(page_idx);
                self.free_pages += 1;
            }
        }
    }

    /// Free contiguous pages
    pub fn free_pages(&mut self, phys_addr: usize, count: usize) {
        for i in 0..count {
            self.free_page(phys_addr + i * PAGE_SIZE);
        }
    }

    /// Get free page count
    pub fn free_count(&self) -> usize {
        self.free_pages
    }

    /// Get total page count
    pub fn total_count(&self) -> usize {
        self.num_frames
    }

    /// Get free memory in bytes
    pub fn free_memory(&self) -> usize {
        self.free_pages * PAGE_SIZE
    }

    /// Increment reference count for a page
    #[allow(dead_code)]
    pub fn inc_ref(&mut self, phys_addr: usize) -> bool {
        if phys_addr < DRAM_BASE || phys_addr >= DRAM_END {
            return false;
        }
        let page_idx = (phys_addr - DRAM_BASE) / PAGE_SIZE;
        if page_idx >= self.num_frames {
            return false;
        }
        if self.frame(page_idx).state == PageState::Used {
            let frame = self.frame_mut(page_idx);
            frame.ref_count = frame.ref_count.saturating_add(1);
            true
        } else {
            false
        }
    }

    /// Get reference count for a page
    #[allow(dead_code)]
    pub fn ref_count(&self, phys_addr: usize) -> u8 {
        if phys_addr < DRAM_BASE || phys_addr >= DRAM_END {
            return 0;
        }
        let page_idx = (phys_addr - DRAM_BASE) / PAGE_SIZE;
        if page_idx >= self.num_frames {
            return 0;
        }
        self.frame(page_idx).ref_count
    }
}

// ============================================================================
// Global PMM Instance
// ============================================================================

/// Global PMM protected by SpinLock
static PMM: SpinLock<PhysicalMemoryManager> = SpinLock::new(crate::kernel::lock::lock_class::RESOURCE, PhysicalMemoryManager::new());

/// Initialize the physical memory manager
pub fn init() {
    let mut guard = PMM.lock();
    guard.init();
}

/// Allocate a single page
pub fn alloc_page() -> Option<usize> {
    let mut guard = PMM.lock();
    guard.alloc_page()
}

/// Allocate contiguous pages
pub fn alloc_pages(count: usize) -> Option<usize> {
    let mut guard = PMM.lock();
    guard.alloc_pages(count)
}

/// Free a single page
pub fn free_page(addr: usize) {
    let mut guard = PMM.lock();
    guard.free_page(addr);
}

/// Free contiguous pages
pub fn free_pages(addr: usize, count: usize) {
    let mut guard = PMM.lock();
    guard.free_pages(addr, count);
}

/// Allocate contiguous pages (alias for DMA)
pub fn alloc_contiguous(count: usize) -> Option<usize> {
    alloc_pages(count)
}

/// Free contiguous pages (alias)
pub fn free_contiguous(addr: usize, count: usize) {
    free_pages(addr, count)
}

/// Get free page count
pub fn free_count() -> usize {
    let guard = PMM.lock();
    guard.free_count()
}

/// Get total page count
pub fn total_count() -> usize {
    let guard = PMM.lock();
    guard.total_count()
}

/// Print memory info
pub fn print_info() {
    let guard = PMM.lock();
    let free_mb = guard.free_memory() / (1024 * 1024);
    let total_mb = (guard.total_count() * PAGE_SIZE) / (1024 * 1024);
    print_direct!("  Total:  {} pages ({} MB)\n", guard.total_count(), total_mb);
    print_direct!("  Free:   {} pages ({} MB)\n", guard.free_count(), free_mb);
}

/// Execute a closure with exclusive PMM access
#[inline]
#[allow(dead_code)]
pub fn with_pmm<R, F: FnOnce(&mut PhysicalMemoryManager) -> R>(f: F) -> R {
    let mut guard = PMM.lock();
    f(&mut *guard)
}

/// Test the allocator
#[allow(dead_code)]
pub fn test() {
    kdebug!("pmm", "test_start");

    if let Some(addr) = alloc_page() {
        kdebug!("pmm", "alloc_page_ok"; addr = klog::hex64(addr as u64));
        free_page(addr);
        kdebug!("pmm", "free_page_ok");
    } else {
        kwarn!("pmm", "test_alloc_failed");
        return;
    }

    if let Some(addr) = alloc_pages(4) {
        kdebug!("pmm", "alloc_pages_ok"; addr = klog::hex64(addr as u64), count = 4);
        free_pages(addr, 4);
        kdebug!("pmm", "free_pages_ok");
    } else {
        kwarn!("pmm", "test_contiguous_failed");
        return;
    }

    kinfo!("pmm", "test_ok");
}
