//! Physical Memory Manager (PMM)
//!
//! Simple bitmap-based page frame allocator.
//! Each bit represents a 4KB page: 1 = used, 0 = free

use crate::logln;

/// Page size (4KB)
pub const PAGE_SIZE: usize = 4096;

/// Start of usable DRAM
const DRAM_START: usize = 0x40000000;

/// End of usable DRAM (4GB total, but we'll use 1GB for now)
const DRAM_END: usize = 0x80000000; // 1GB usable

/// Kernel load address
const KERNEL_START: usize = 0x46000000;

/// Kernel reserved size (2MB for kernel + embedded initrd + IPC buffers)
const KERNEL_SIZE: usize = 0x200000;

/// Number of pages we manage
const NUM_PAGES: usize = (DRAM_END - DRAM_START) / PAGE_SIZE;

/// Bitmap size in bytes (1 bit per page)
const BITMAP_SIZE: usize = NUM_PAGES / 8;

/// Physical page frame allocator
pub struct PhysicalMemoryManager {
    /// Bitmap: 1 = used, 0 = free
    bitmap: [u8; BITMAP_SIZE],
    /// Total pages
    total_pages: usize,
    /// Free pages count
    free_pages: usize,
}

impl PhysicalMemoryManager {
    pub const fn new() -> Self {
        Self {
            bitmap: [0; BITMAP_SIZE],
            total_pages: NUM_PAGES,
            free_pages: 0,
        }
    }

    /// Initialize the allocator
    pub fn init(&mut self) {
        // Mark all pages as free initially
        for byte in self.bitmap.iter_mut() {
            *byte = 0;
        }
        self.free_pages = self.total_pages;

        // Mark kernel region as used
        let kernel_start_page = (KERNEL_START - DRAM_START) / PAGE_SIZE;
        let kernel_pages = KERNEL_SIZE / PAGE_SIZE;
        for i in kernel_start_page..(kernel_start_page + kernel_pages) {
            self.mark_used(i);
        }

        // Mark first few pages as used (safety buffer)
        for i in 0..16 {
            self.mark_used(i);
        }
    }

    /// Allocate a single page, returns physical address or None
    pub fn alloc_page(&mut self) -> Option<usize> {
        // Find first free page
        for (byte_idx, byte) in self.bitmap.iter_mut().enumerate() {
            if *byte != 0xFF {
                // Found a byte with at least one free bit
                for bit in 0..8 {
                    if (*byte & (1 << bit)) == 0 {
                        // Found free page
                        *byte |= 1 << bit;
                        self.free_pages -= 1;
                        let page_idx = byte_idx * 8 + bit;
                        let phys_addr = DRAM_START + page_idx * PAGE_SIZE;
                        return Some(phys_addr);
                    }
                }
            }
        }
        None
    }

    /// Allocate contiguous pages, returns starting physical address or None
    pub fn alloc_pages(&mut self, count: usize) -> Option<usize> {
        if count == 0 {
            return None;
        }
        if count == 1 {
            return self.alloc_page();
        }

        // Find contiguous free pages (simple linear search)
        let mut start_page = 0;
        let mut consecutive = 0;

        for page_idx in 0..self.total_pages {
            if self.is_free(page_idx) {
                if consecutive == 0 {
                    start_page = page_idx;
                }
                consecutive += 1;
                if consecutive == count {
                    // Found enough contiguous pages, mark them used
                    for i in start_page..(start_page + count) {
                        self.mark_used(i);
                    }
                    let phys_addr = DRAM_START + start_page * PAGE_SIZE;
                    return Some(phys_addr);
                }
            } else {
                consecutive = 0;
            }
        }
        None
    }

    /// Free a single page
    pub fn free_page(&mut self, phys_addr: usize) {
        if phys_addr < DRAM_START || phys_addr >= DRAM_END {
            return; // Invalid address
        }
        let page_idx = (phys_addr - DRAM_START) / PAGE_SIZE;
        self.mark_free(page_idx);
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
        self.total_pages
    }

    /// Get free memory in bytes
    pub fn free_memory(&self) -> usize {
        self.free_pages * PAGE_SIZE
    }

    #[inline]
    fn is_free(&self, page_idx: usize) -> bool {
        let byte_idx = page_idx / 8;
        let bit = page_idx % 8;
        (self.bitmap[byte_idx] & (1 << bit)) == 0
    }

    #[inline]
    fn mark_used(&mut self, page_idx: usize) {
        let byte_idx = page_idx / 8;
        let bit = page_idx % 8;
        if self.is_free(page_idx) {
            self.bitmap[byte_idx] |= 1 << bit;
            self.free_pages -= 1;
        }
    }

    #[inline]
    fn mark_free(&mut self, page_idx: usize) {
        let byte_idx = page_idx / 8;
        let bit = page_idx % 8;
        if !self.is_free(page_idx) {
            self.bitmap[byte_idx] &= !(1 << bit);
            self.free_pages += 1;
        }
    }
}

/// Global PMM instance
static mut PMM: PhysicalMemoryManager = PhysicalMemoryManager::new();

/// Initialize the physical memory manager
pub fn init() {
    unsafe {
        (*core::ptr::addr_of_mut!(PMM)).init();
    }
}

/// Allocate a page
pub fn alloc_page() -> Option<usize> {
    unsafe { (*core::ptr::addr_of_mut!(PMM)).alloc_page() }
}

/// Allocate contiguous pages
pub fn alloc_pages(count: usize) -> Option<usize> {
    unsafe { (*core::ptr::addr_of_mut!(PMM)).alloc_pages(count) }
}

/// Free a page
pub fn free_page(addr: usize) {
    unsafe { (*core::ptr::addr_of_mut!(PMM)).free_page(addr) }
}

/// Free contiguous pages
pub fn free_pages(addr: usize, count: usize) {
    unsafe { (*core::ptr::addr_of_mut!(PMM)).free_pages(addr, count) }
}

/// Allocate contiguous physical pages (alias for alloc_pages)
/// For shared memory / DMA buffers that need physical contiguity
pub fn alloc_contiguous(count: usize) -> Option<usize> {
    alloc_pages(count)
}

/// Free contiguous physical pages (alias for free_pages)
pub fn free_contiguous(addr: usize, count: usize) {
    free_pages(addr, count)
}

/// Get free page count
pub fn free_count() -> usize {
    unsafe { (*core::ptr::addr_of!(PMM)).free_count() }
}

/// Get total page count
pub fn total_count() -> usize {
    unsafe { (*core::ptr::addr_of!(PMM)).total_count() }
}

/// Print memory info
pub fn print_info() {
    unsafe {
        let pmm = &*core::ptr::addr_of!(PMM);
        let free_mb = pmm.free_memory() / (1024 * 1024);
        let total_mb = (pmm.total_count() * PAGE_SIZE) / (1024 * 1024);
        logln!("  Total:  {} pages ({} MB)", pmm.total_count(), total_mb);
        logln!("  Free:   {} pages ({} MB)", pmm.free_count(), free_mb);
    }
}

/// Test the allocator
pub fn test() {
    logln!("  Testing allocation...");

    // Allocate a page
    if let Some(addr) = alloc_page() {
        logln!("    Allocated page at 0x{:08x}", addr);

        // Write test pattern
        unsafe {
            let ptr = addr as *mut u64;
            core::ptr::write_volatile(ptr, 0xDEADBEEF_CAFEBABE);
            let val = core::ptr::read_volatile(ptr);
            if val == 0xDEADBEEF_CAFEBABE {
                logln!("    Write/read test: OK");
            } else {
                logln!("    Write/read test: FAILED");
            }
        }

        // Free it
        free_page(addr);
        logln!("    Freed page");
    } else {
        logln!("    Allocation failed!");
    }
}
