//! Address Space Manager
//!
//! Manages per-process virtual address spaces using the PMM for page table allocation.
//! Each process gets its own page tables loaded into TTBR0 during context switch.
//!
//! ## Memory Layout (User Space via TTBR0)
//!
//! ```text
//! 0x0000_0000_0000_0000 - 0x0000_0000_3FFF_FFFF : Reserved/device (1GB)
//! 0x0000_0000_4000_0000 - 0x0000_0000_7FFF_FFFF : Code + data (1GB)
//! 0x0000_0000_8000_0000 - 0x0000_0000_BFFF_FFFF : Available (1GB)
//! 0x0000_0000_C000_0000 - 0x0000_0000_FFFF_FFFF : Available (1GB)
//! ```
//!
//! ## Page Table Structure (4KB granule, 48-bit VA)
//!
//! ```text
//! L0 (512 entries x 512GB each) -> L1 (512 entries x 1GB each)
//!                                       -> L2 (512 entries x 2MB each)
//!                                              -> L3 (512 entries x 4KB pages)
//! ```
//!
//! ASID (Address Space ID) support:
//! - Each address space gets a unique 8-bit ASID (256 values, 0 reserved)
//! - ASID is encoded in TTBR0 bits [63:48] for hardware tagging
//! - Allows per-ASID TLB invalidation instead of global flush
//!
//! ## Thread Safety
//!
//! The ASID allocator is protected by a SpinLock with IRQ-save semantics,
//! making it safe to call from both process and interrupt context.

#![allow(dead_code)]

use crate::kernel::arch::mmu::{self, flags, attr};
use crate::kernel::arch::tlb;
use super::pmm;
use super::lock::SpinLock;
use crate::print_direct;

// ============================================================================
// DEBUG: L3 canary tracking for devd's stack
// ============================================================================

// ============================================================================
// ASID Allocator
// ============================================================================

/// Maximum ASID value (8-bit ASIDs, 0 reserved for kernel)
const MAX_ASID: u16 = 255;

/// ASID allocator - simple bitmap for 255 ASIDs (ASID 0 reserved)
///
/// This allocator uses a rotor-style hint to find free ASIDs quickly.
/// Thread-safe when accessed through the SpinLock wrapper.
struct AsidAllocator {
    /// Bitmap: bit N = 1 if ASID N+1 is in use
    bitmap: [u64; 4],  // 256 bits
    /// Next ASID to try (for faster allocation)
    next_hint: u16,
    /// Count of allocated ASIDs (for debugging)
    allocated_count: u16,
}

impl AsidAllocator {
    const fn new() -> Self {
        Self {
            bitmap: [0; 4],
            next_hint: 1,  // Start at ASID 1 (0 reserved)
            allocated_count: 0,
        }
    }

    /// Allocate an ASID, returns None if all exhausted
    fn alloc(&mut self) -> Option<u16> {
        // Try from hint first
        for _ in 0..MAX_ASID {
            let asid = self.next_hint;
            if asid == 0 || asid > MAX_ASID {
                self.next_hint = 1;
                continue;
            }

            let word = (asid - 1) as usize / 64;
            let bit = (asid - 1) as usize % 64;

            if (self.bitmap[word] & (1u64 << bit)) == 0 {
                // Found free ASID
                self.bitmap[word] |= 1u64 << bit;
                self.next_hint = if asid < MAX_ASID { asid + 1 } else { 1 };
                self.allocated_count += 1;
                return Some(asid);
            }

            self.next_hint = if asid < MAX_ASID { asid + 1 } else { 1 };
        }
        None  // All ASIDs in use
    }

    /// Free an ASID
    fn free(&mut self, asid: u16) {
        if asid == 0 || asid > MAX_ASID {
            return;
        }
        let word = (asid - 1) as usize / 64;
        let bit = (asid - 1) as usize % 64;

        // Only decrement count if actually allocated
        if (self.bitmap[word] & (1u64 << bit)) != 0 {
            self.bitmap[word] &= !(1u64 << bit);
            self.allocated_count = self.allocated_count.saturating_sub(1);
        }
    }

    /// Get count of allocated ASIDs
    #[allow(dead_code)]
    fn count(&self) -> u16 {
        self.allocated_count
    }
}

/// Global ASID allocator protected by SpinLock
///
/// The SpinLock provides:
/// - IRQ-safe access (disables interrupts while held)
/// - SMP-safe access (atomic spinlock for multi-core)
static ASID_ALLOCATOR: SpinLock<AsidAllocator> = SpinLock::new(crate::kernel::lock::lock_class::RESOURCE, AsidAllocator::new());

/// Allocate an ASID (thread-safe)
fn alloc_asid() -> Option<u16> {
    let mut guard = ASID_ALLOCATOR.lock();
    guard.alloc()
}

/// Free an ASID (thread-safe)
fn free_asid(asid: u16) {
    let mut guard = ASID_ALLOCATOR.lock();
    guard.free(asid);
}

// ============================================================================
// External API for Trait Implementation
// ============================================================================

/// Allocate an ASID (external API for trait boundary)
pub fn alloc_asid_external() -> Option<u16> {
    alloc_asid()
}

/// Free an ASID (external API for trait boundary)
pub fn free_asid_external(asid: u16) {
    free_asid(asid);
}

/// Get count of allocated ASIDs (external API for trait boundary)
pub fn asid_count_external() -> usize {
    let guard = ASID_ALLOCATOR.lock();
    guard.count() as usize
}

/// Convert a physical address to a kernel virtual address for access via TTBR1
#[inline(always)]
fn phys_to_virt(phys: u64) -> *mut u64 {
    mmu::phys_to_virt(phys) as *mut u64
}

// ============================================================================
// Virtual Address Space Layout Constants
// ============================================================================
//
// These constants define the userspace memory layout for AArch64 with 4KB
// pages and 48-bit virtual addresses. The layout is:
//
//   L1 Index 0: 0x00000000-0x3FFFFFFF (1GB) - Reserved for kernel/device
//   L1 Index 1: 0x40000000-0x7FFFFFFF (1GB) - Code + initial data
//   L1 Index 2: 0x80000000-0xBFFFFFFF (1GB) - Heap growth
//   L1 Index 3: 0xC0000000-0xFFFFFFFF (1GB) - Stack + mmap
//
// This provides 4GB of user virtual address space, which is sufficient
// for embedded systems while keeping page table overhead minimal.
// ============================================================================

/// Maximum number of L1 (1GB) entries for user address space.
/// Value 4 = 4GB total user VA space (indices 0-3).
/// L1 index 0 is reserved (device memory region), so usable space is 3GB.
const MAX_USER_L1_ENTRIES: usize = 4;

/// User space start address.
/// 0x4000_0000 = 1GB = L1 index 1. This skips the first 1GB which
/// contains device MMIO mappings in the kernel's address space.
/// User code/data starts here.
pub const USER_SPACE_START: u64 = 0x0000_0000_4000_0000;

/// User space end address (exclusive).
/// 0x1_0000_0000 = 4GB. This is the upper limit of the 4 L1 entries.
/// Addresses at or above this are invalid for user processes.
pub const USER_SPACE_END: u64 = 0x0000_0001_0000_0000;

/// Represents a process's virtual address space
pub struct AddressSpace {
    /// Physical address of L0 page table (loaded into TTBR0)
    pub ttbr0: u64,
    /// ASID for this address space (1-255, 0 = none allocated)
    asid: u16,
    /// Physical addresses of allocated page tables (for cleanup).
    ///
    /// Array size 64 = L0(1) + L1(1) + L2(up to 4) + L3(up to 58).
    /// Each L3 table covers 2MB (512 x 4KB pages), so 58 L3 tables
    /// support up to 116MB of fine-grained mapped memory.
    ///
    /// This is sufficient for typical embedded processes. Processes
    /// needing more memory would use 2MB block mappings at L2 level.
    page_tables: [u64; 64],
    num_tables: usize,
}

impl AddressSpace {
    /// Create a new empty address space
    pub fn new() -> Option<Self> {
        // Allocate ASID for this address space
        let asid = alloc_asid().unwrap_or(0);

        // Allocate L0 table
        let l0_phys = match pmm::alloc_page() {
            Some(addr) => addr,
            None => {
                // Clean up ASID on failure
                if asid != 0 {
                    free_asid(asid);
                }
                return None;
            }
        };

        // Allocate L1 table
        let l1_phys = match pmm::alloc_page() {
            Some(addr) => addr,
            None => {
                // Clean up L0 and ASID on failure
                pmm::free_page(l0_phys);
                if asid != 0 {
                    free_asid(asid);
                }
                return None;
            }
        };

        // Zero out the tables (must use TTBR1 mapping since MMU is enabled)
        unsafe {
            let l0_ptr = phys_to_virt(l0_phys as u64);
            let l1_ptr = phys_to_virt(l1_phys as u64);
            for i in 0..512 {
                core::ptr::write_volatile(l0_ptr.add(i), 0);
                core::ptr::write_volatile(l1_ptr.add(i), 0);
            }

            // Set up L0 -> L1 table entry
            // Entry 0: points to L1 table
            core::ptr::write_volatile(
                l0_ptr,
                l1_phys as u64 | flags::VALID | flags::TABLE
            );
        }

        // Note: We don't map device memory in user address spaces.
        // User processes access devices through syscalls (microkernel design).
        // L1 entries start empty, allowing map_page() to create L2/L3 tables.

        Some(Self {
            ttbr0: l0_phys as u64,
            asid,
            page_tables: {
                let mut tables = [0u64; 64];
                tables[0] = l0_phys as u64;
                tables[1] = l1_phys as u64;
                tables
            },
            num_tables: 2,
        })
    }

    /// Map a region of physical memory into user space
    /// Returns true on success
    pub fn map_region(
        &mut self,
        virt_start: u64,
        phys_start: u64,
        size: usize,
        writable: bool,
        executable: bool,
    ) -> bool {
        // For now, only support 1GB block mappings at L1 level.
        // This is a simplification - full implementation would use L2/L3 for fine-grained mapping.
        //
        // 0x4000_0000 = 1GB = 2^30 bytes = size of one L1 block entry.
        // L1 block entries map 1GB of contiguous physical memory to 1GB of virtual memory.
        if size != 0x4000_0000 {
            // Only 1GB blocks supported for now
            return false;
        }

        // Calculate L1 index
        let l1_index = ((virt_start >> 30) & 0x1FF) as usize;
        if l1_index >= MAX_USER_L1_ENTRIES {
            return false;
        }

        // Get L1 table address
        let l1_phys = self.page_tables[1];

        // Set up the block entry
        let ap = if writable { flags::AP_RW_ALL } else { flags::AP_RO_ALL };
        let xn = if executable { 0 } else { flags::UXN };

        unsafe {
            let l1_ptr = phys_to_virt(l1_phys);
            core::ptr::write_volatile(
                l1_ptr.add(l1_index),
                phys_start
                    | flags::VALID
                    | flags::AF
                    | flags::SH_INNER
                    | attr::NORMAL
                    | ap
                    | xn
                    | flags::PXN  // Kernel cannot execute user code
            );
        }

        true
    }

    // ========================================================================
    // Page Mapping - Core Implementation
    // ========================================================================

    /// Mask to extract physical address from a page table entry.
    ///
    /// AArch64 page table entry format (4KB granule):
    ///   Bits [47:12] = Physical address (36 bits, 4KB aligned)
    ///   Bits [11:0]  = Flags (VALID, TABLE/BLOCK, AP, SH, etc.)
    ///
    /// This mask (0x0000_FFFF_FFFF_F000) extracts bits [47:12]:
    ///   - Upper 16 bits cleared (bits [63:48] are ASID/reserved)
    ///   - Lower 12 bits cleared (flags)
    ///   - Result is 4KB-aligned physical address
    const PHYS_ADDR_MASK: u64 = 0x0000_FFFF_FFFF_F000;

    /// Internal helper: Get or allocate a next-level table
    /// Returns the physical address of the table, or None on failure
    fn get_or_alloc_table(&mut self, parent_phys: u64, index: usize) -> Option<u64> {
        unsafe {
            let parent_ptr = phys_to_virt(parent_phys);
            let entry = core::ptr::read_volatile(parent_ptr.add(index));

            if entry == 0 {
                // Allocate new table
                if self.num_tables >= 64 {
                    return None; // Out of tracking space
                }
                let new_table = pmm::alloc_page()? as u64;

                // Zero the new table
                let new_ptr = phys_to_virt(new_table);
                core::ptr::write_bytes(new_ptr, 0, 512);

                // Set parent entry to point to new table
                core::ptr::write_volatile(
                    parent_ptr.add(index),
                    new_table | flags::VALID | flags::TABLE
                );
                self.page_tables[self.num_tables] = new_table;
                self.num_tables += 1;
                Some(new_table)
            } else if (entry & flags::TABLE) != 0 {
                // Already a table entry - extract physical address
                Some(entry & Self::PHYS_ADDR_MASK)
            } else {
                // It's a block entry, can't map pages here
                None
            }
        }
    }

    /// Internal helper: Map a page with specified attributes
    /// memory_attr: attr::NORMAL, attr::DEVICE, or attr::NORMAL_NC
    /// executable: only applies to NORMAL memory (others are always XN)
    fn map_page_internal(
        &mut self,
        virt_addr: u64,
        phys_addr: u64,
        writable: bool,
        executable: bool,
        memory_attr: u64,
    ) -> bool {
        // Extract table indices from virtual address
        let l1_index = ((virt_addr >> 30) & 0x1FF) as usize;
        let l2_index = ((virt_addr >> 21) & 0x1FF) as usize;
        let l3_index = ((virt_addr >> 12) & 0x1FF) as usize;

        // Walk/create page tables: L1 -> L2 -> L3
        let l1_phys = self.page_tables[1];
        let l2_phys = match self.get_or_alloc_table(l1_phys, l1_index) {
            Some(addr) => addr,
            None => return false,
        };
        let l3_phys = match self.get_or_alloc_table(l2_phys, l2_index) {
            Some(addr) => addr,
            None => return false,
        };

        // DEBUG: Verify L2 entry actually points to L3
        // Check addresses in 0x5000_0000-0x5FFF_FFFF range (heap region at 1.25-1.5GB)
        // This is a common area for heap allocations and has had issues with page table setup
        if (virt_addr >> 28) == 5 {
            unsafe {
                let l2_ptr = phys_to_virt(l2_phys);
                let l2_entry = core::ptr::read_volatile(l2_ptr.add(l2_index));
                let l2_points_to = l2_entry & 0x0000_FFFF_FFFF_F000;
                if l2_points_to != l3_phys {
                    crate::kwarn!("addrspace", "l2_mismatch";
                        va = virt_addr,
                        l2_entry = l2_entry,
                        l2_points_to = l2_points_to,
                        expected_l3 = l3_phys);
                }
            }
        }

        // Build page entry flags
        let ap = if writable { flags::AP_RW_ALL } else { flags::AP_RO_ALL };
        let xn = if executable && memory_attr == attr::NORMAL { 0 } else { flags::UXN };

        unsafe {
            let l3_ptr = phys_to_virt(l3_phys);

            // DEBUG: Check if there's already a mapping here
            let old_pte = core::ptr::read_volatile(l3_ptr.add(l3_index));
            if old_pte != 0 {
                crate::kwarn!("addrspace", "overwriting_pte";
                    va = virt_addr,
                    old_pte = old_pte,
                    old_pa = old_pte & 0x0000_FFFF_FFFF_F000);
            }

            let pte = phys_addr
                    | flags::VALID
                    | flags::PAGE
                    | flags::AF
                    | flags::SH_INNER
                    | memory_attr
                    | ap
                    | xn
                    | flags::PXN;

            core::ptr::write_volatile(l3_ptr.add(l3_index), pte);

            // Clean cache line containing the PTE to ensure page table walker sees it
            let pte_addr = l3_ptr.add(l3_index) as u64;
            core::arch::asm!(
                "dc cvac, {pte}",   // Clean data cache by VA to PoC
                "dsb ish",          // Ensure clean completes
                pte = in(reg) pte_addr,
                options(nostack, preserves_flags)
            );

            // CRITICAL: Invalidate TLB entry for this virtual address + ASID
            // Use vale1is (VA + ASID, last level, inner shareable) to avoid
            // unnecessarily invalidating other address spaces' TLB entries.
            let tlbi_val = ((self.asid as u64) << 48) | (virt_addr >> 12);
            core::arch::asm!(
                "tlbi vale1is, {va}",
                "dsb ish",
                "isb",
                va = in(reg) tlbi_val,
                options(nostack, preserves_flags)
            );

        }

        true
    }

    /// Map a 4KB page at a specific virtual address (normal cacheable memory)
    #[inline]
    pub fn map_page(
        &mut self,
        virt_addr: u64,
        phys_addr: u64,
        writable: bool,
        executable: bool,
    ) -> bool {
        self.map_page_internal(virt_addr, phys_addr, writable, executable, attr::NORMAL)
    }

    /// Map a 4KB device page (non-cacheable, for MMIO)
    #[inline]
    pub fn map_device_page(
        &mut self,
        virt_addr: u64,
        phys_addr: u64,
        writable: bool,
    ) -> bool {
        self.map_page_internal(virt_addr, phys_addr, writable, false, attr::DEVICE)
    }

    /// Map a 4KB DMA page (using non-cacheable memory for coherency)
    #[inline]
    pub fn map_dma_page(
        &mut self,
        virt_addr: u64,
        phys_addr: u64,
        writable: bool,
    ) -> bool {
        self.map_page_internal(virt_addr, phys_addr, writable, false, attr::NORMAL_NC)
    }

    /// Unmap a 4KB page at a specific virtual address
    /// Returns the physical address that was mapped, or None if not mapped
    pub fn unmap_page(&mut self, virt_addr: u64) -> Option<u64> {
        // Extract table indices from virtual address
        let l1_index = ((virt_addr >> 30) & 0x1FF) as usize;
        let l2_index = ((virt_addr >> 21) & 0x1FF) as usize;
        let l3_index = ((virt_addr >> 12) & 0x1FF) as usize;

        let l1_phys = self.page_tables[1];

        unsafe {
            // Check L1 entry
            let l1_ptr = phys_to_virt(l1_phys);
            let l1_entry = core::ptr::read_volatile(l1_ptr.add(l1_index));

            if (l1_entry & flags::VALID) == 0 {
                return None; // Not mapped
            }

            if (l1_entry & flags::TABLE) == 0 {
                // It's a 1GB block entry - we don't support unmapping these
                return None;
            }

            // Get L2 table
            let l2_phys = l1_entry & Self::PHYS_ADDR_MASK;
            let l2_ptr = phys_to_virt(l2_phys);
            let l2_entry = core::ptr::read_volatile(l2_ptr.add(l2_index));

            if (l2_entry & flags::VALID) == 0 {
                return None; // Not mapped
            }

            if (l2_entry & flags::TABLE) == 0 {
                // It's a 2MB block entry - we don't support unmapping these
                return None;
            }

            // Get L3 table
            let l3_phys = l2_entry & Self::PHYS_ADDR_MASK;
            let l3_ptr = phys_to_virt(l3_phys);
            let l3_entry = core::ptr::read_volatile(l3_ptr.add(l3_index));

            if (l3_entry & flags::VALID) == 0 {
                return None; // Not mapped
            }

            // Get the physical address before clearing
            let phys_addr = l3_entry & Self::PHYS_ADDR_MASK;

            // Clear the L3 entry (mark as invalid)
            core::ptr::write_volatile(l3_ptr.add(l3_index), 0);

            // Ensure the store is visible to page table walkers on all CPUs
            core::arch::asm!("dsb ishst");

            // Invalidate TLB for this VA on all CPUs (inner shareable)
            // Without this, other CPUs may continue accessing the old physical
            // page through stale TLB entries — a use-after-free via TLB.
            tlb::invalidate_va(self.asid, virt_addr);

            Some(phys_addr)
        }
    }

    /// Activate this address space (switch TTBR0)
    pub unsafe fn activate(&self) {
        // IMPORTANT: Must include ASID in TTBR0 for proper TLB isolation
        mmu::switch_user_space(self.get_ttbr0());
    }

    /// Get the TTBR0 value for this address space (includes ASID in bits [63:48])
    pub fn get_ttbr0(&self) -> u64 {
        // Encode ASID in upper 16 bits of TTBR0
        // Hardware uses bits [63:48] for ASID when TCR_EL1.A1 = 0
        self.ttbr0 | ((self.asid as u64) << 48)
    }

    /// Get the ASID for this address space
    pub fn get_asid(&self) -> u16 {
        self.asid
    }

    /// Debug: Dump PTE for a given virtual address
    /// Returns (l1_entry, l2_entry, l3_entry, phys_addr) or None if not mapped
    pub fn dump_pte(&self, virt_addr: u64) -> Option<(u64, u64, u64, u64)> {
        let l1_index = ((virt_addr >> 30) & 0x1FF) as usize;
        let l2_index = ((virt_addr >> 21) & 0x1FF) as usize;
        let l3_index = ((virt_addr >> 12) & 0x1FF) as usize;

        unsafe {
            // Check L1
            let l1_ptr = phys_to_virt(self.page_tables[1]);
            let l1_entry = core::ptr::read_volatile(l1_ptr.add(l1_index));
            if (l1_entry & flags::VALID) == 0 {
                return None;
            }
            if (l1_entry & flags::TABLE) == 0 {
                // Block entry at L1 (1GB block)
                let phys = l1_entry & Self::PHYS_ADDR_MASK;
                return Some((l1_entry, 0, 0, phys));
            }

            // Get L2
            let l2_phys = l1_entry & Self::PHYS_ADDR_MASK;
            let l2_ptr = phys_to_virt(l2_phys);
            let l2_entry = core::ptr::read_volatile(l2_ptr.add(l2_index));
            if (l2_entry & flags::VALID) == 0 {
                return None;
            }
            if (l2_entry & flags::TABLE) == 0 {
                // Block entry at L2 (2MB block)
                let phys = l2_entry & Self::PHYS_ADDR_MASK;
                return Some((l1_entry, l2_entry, 0, phys));
            }

            // Get L3
            let l3_phys = l2_entry & Self::PHYS_ADDR_MASK;
            let l3_ptr = phys_to_virt(l3_phys);
            let l3_entry = core::ptr::read_volatile(l3_ptr.add(l3_index));
            if (l3_entry & flags::VALID) == 0 {
                return None;
            }

            let phys = l3_entry & Self::PHYS_ADDR_MASK;
            Some((l1_entry, l2_entry, l3_entry, phys))
        }
    }
}

impl Drop for AddressSpace {
    fn drop(&mut self) {
        // Invalidate TLB entries for this ASID before freeing
        if self.asid != 0 {
            tlb::invalidate_asid(self.asid);
            free_asid(self.asid);
        }

        // Free all allocated page tables
        for i in 0..self.num_tables {
            if self.page_tables[i] != 0 {
                pmm::free_page(self.page_tables[i] as usize);
            }
        }
    }
}

/// Test the address space manager
pub fn test() {
    print_direct!("  Testing address space creation...\n");

    if let Some(mut addr_space) = AddressSpace::new() {
        print_direct!("    Created address space, TTBR0: 0x{:016x}\n", addr_space.ttbr0);

        // Allocate a physical page for user code
        if let Some(user_page) = pmm::alloc_page() {
            print_direct!("    Allocated user page at 0x{:08x}\n", user_page);

            // Map it into the address space at USER_SPACE_START (1GB mark)
            let user_virt = USER_SPACE_START;
            if addr_space.map_page(user_virt, user_page as u64, true, true) {
                print_direct!("    Mapped 0x{:08x} -> 0x{:016x}\n", user_page, user_virt);
                print_direct!("    [OK] Address space test passed\n");
            } else {
                print_direct!("    [!!] Failed to map page\n");
            }

            pmm::free_page(user_page);
        } else {
            print_direct!("    [!!] Failed to allocate user page\n");
        }

        // addr_space will be dropped here, freeing page tables
        print_direct!("    Address space cleaned up\n");
    } else {
        print_direct!("    [!!] Failed to create address space\n");
    }
}
