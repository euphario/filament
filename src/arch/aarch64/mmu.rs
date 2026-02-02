//! AArch64 MMU (Memory Management Unit) driver
//!
//! Implements 4-level page tables with 4KB pages for kernel/user separation.
//!
//! The kernel is linked at virtual address 0xFFFF_0000_4600_0000 (TTBR1).
//! MMU is enabled in boot.S before any Rust code runs.
//!
//! Address Space Layout:
//! - TTBR0_EL1: User space (0x0000_0000_0000_0000 - 0x0000_FFFF_FFFF_FFFF)
//! - TTBR1_EL1: Kernel space (0xFFFF_0000_0000_0000 - 0xFFFF_FFFF_FFFF_FFFF)
//!
//! Virtual Address Layout (48-bit):
//! [63:48] - Sign extension (must match bit 47)
//! [47:39] - L0 index (9 bits, 512 entries)
//! [38:30] - L1 index (9 bits, 512 entries)
//! [29:21] - L2 index (9 bits, 512 entries)
//! [20:12] - L3 index (9 bits, 512 entries)
//! [11:0]  - Page offset (12 bits, 4KB)

#![allow(dead_code)]  // Many items are infrastructure for future use

use crate::{kdebug, klog};

/// Page size (4KB)
pub const PAGE_SIZE: usize = 4096;

/// Number of entries per table (512 for 4KB pages)
const ENTRIES_PER_TABLE: usize = 512;

/// Kernel virtual base address (upper half)
/// Maps physical 0x0 to virtual 0xFFFF_0000_0000_0000
pub const KERNEL_VIRT_BASE: u64 = 0xFFFF_0000_0000_0000;

/// Physical to kernel virtual address conversion
#[inline]
pub fn phys_to_virt(phys: u64) -> u64 {
    phys | KERNEL_VIRT_BASE
}

/// Kernel virtual to physical address conversion
#[inline]
pub fn virt_to_phys(virt: u64) -> u64 {
    virt & !KERNEL_VIRT_BASE
}

/// Page table entry flags
pub mod flags {
    pub const VALID: u64 = 1 << 0;
    pub const TABLE: u64 = 1 << 1;  // For L0-L2: points to next level table
    pub const PAGE: u64 = 1 << 1;   // For L3: this is a page entry
    pub const AF: u64 = 1 << 10;    // Access flag (must be 1)
    pub const SH_NONE: u64 = 0 << 8;   // Non-shareable
    pub const SH_OUTER: u64 = 2 << 8;  // Outer shareable (for DMA with bus masters)
    pub const SH_INNER: u64 = 3 << 8;  // Inner shareable
    pub const AP_RW_EL1: u64 = 0 << 6;  // Read-write at EL1 only
    pub const AP_RW_ALL: u64 = 1 << 6;  // Read-write at all ELs
    pub const AP_RO_EL1: u64 = 2 << 6;  // Read-only at EL1 only
    pub const AP_RO_ALL: u64 = 3 << 6;  // Read-only at all ELs
    pub const UXN: u64 = 1 << 54;   // Unprivileged execute never
    pub const PXN: u64 = 1 << 53;   // Privileged execute never
    pub const NS: u64 = 1 << 5;     // Non-secure (for block/page)
}

/// Memory attribute indices (for MAIR_EL1)
pub mod attr {
    pub const DEVICE: u64 = 0 << 2;     // Attr index 0: Device-nGnRnE
    pub const NORMAL: u64 = 1 << 2;     // Attr index 1: Normal memory (cacheable)
    pub const NORMAL_NC: u64 = 2 << 2;  // Attr index 2: Normal Non-Cacheable (for DMA buffers)
}

// Note: MMU configuration (MAIR, TCR, page tables) is set up in boot.S
// before any Rust code runs. This module provides helper functions for
// runtime page table manipulation.

/// Page table (512 entries, 4KB aligned)
#[repr(C, align(4096))]
pub struct PageTable {
    pub entries: [u64; ENTRIES_PER_TABLE],
}

impl PageTable {
    pub const fn new() -> Self {
        Self {
            entries: [0; ENTRIES_PER_TABLE],
        }
    }

    /// Clear all entries
    pub fn clear(&mut self) {
        for entry in self.entries.iter_mut() {
            *entry = 0;
        }
    }

    /// Set an entry pointing to next level table
    pub fn set_table(&mut self, index: usize, table_addr: u64) {
        self.entries[index] = table_addr | flags::VALID | flags::TABLE;
    }

    /// Set a block entry (1GB at L1, 2MB at L2)
    pub fn set_block(&mut self, index: usize, phys_addr: u64, is_device: bool) {
        let attr = if is_device { attr::DEVICE } else { attr::NORMAL };
        self.entries[index] = phys_addr
            | flags::VALID
            | flags::AF
            | flags::SH_INNER
            | attr;
        // Note: no TABLE flag = block descriptor
    }

    /// Set a block entry with full control over flags
    pub fn set_block_with_flags(&mut self, index: usize, phys_addr: u64, extra_flags: u64, attr_idx: u64) {
        self.entries[index] = phys_addr
            | flags::VALID
            | flags::AF
            | flags::SH_INNER
            | attr_idx
            | extra_flags;
    }

    /// Set a page entry (4KB at L3)
    pub fn set_page(&mut self, index: usize, phys_addr: u64, is_device: bool) {
        let attr = if is_device { attr::DEVICE } else { attr::NORMAL };
        self.entries[index] = phys_addr
            | flags::VALID
            | flags::PAGE
            | flags::AF
            | flags::SH_INNER
            | attr;
    }

    /// Set a page entry with user access
    pub fn set_user_page(&mut self, index: usize, phys_addr: u64, writable: bool, executable: bool) {
        let ap = if writable { flags::AP_RW_ALL } else { flags::AP_RO_ALL };
        let xn = if executable { 0 } else { flags::UXN };
        self.entries[index] = phys_addr
            | flags::VALID
            | flags::PAGE
            | flags::AF
            | flags::SH_INNER
            | attr::NORMAL
            | ap
            | xn
            | flags::PXN;  // Kernel can't execute user code
    }

    /// Get physical address of this table
    /// Note: Returns virtual address when kernel is linked at virtual address.
    /// Use virt_to_phys() to convert if needed.
    pub fn phys_addr(&self) -> u64 {
        virt_to_phys(self as *const _ as u64)
    }
}

// Boot page table addresses from boot.S (physical addresses)
extern "C" {
    pub static BOOT_TTBR0: u64;
    pub static BOOT_TTBR1: u64;
    // L1 table for TTBR1 (512 entries, each covers 1GB)
    static boot_l1_ttbr1: [u64; 512];
}

/// Map a 1GB-aligned physical region as device memory in the kernel address space.
///
/// Writes an L1 block descriptor into the kernel's TTBR1 page table so that
/// `phys_base | KERNEL_VIRT_BASE` becomes accessible as device memory.
///
/// # Safety
/// - `phys_base` must be 1GB-aligned
/// - The L1 slot must not already be in use for a different mapping
/// - Must be called after MMU is enabled
pub fn map_kernel_device_1gb(phys_base: u64) {
    assert!(phys_base & 0x3FFF_FFFF == 0, "phys_base must be 1GB-aligned");

    let l1_index = ((phys_base >> 30) & 0x1FF) as usize;

    // Build L1 block descriptor: device memory, no-execute, kernel-only
    let descriptor = phys_base
        | flags::VALID
        | flags::AF
        | attr::DEVICE
        | (flags::UXN | flags::PXN);
    // Note: no TABLE flag = block descriptor at L1 level

    // Write to the L1 table (identity-mapped in kernel VA space)
    let l1_virt = unsafe { &boot_l1_ttbr1 as *const _ as u64 } ;
    let entry_ptr = (l1_virt + (l1_index as u64) * 8) as *mut u64;
    unsafe {
        core::ptr::write_volatile(entry_ptr, descriptor);
        // Invalidate TLB for all addresses on all CPUs (inner shareable).
        // Another CPU may have cached the old (invalid/zero) L1 entry,
        // so we must ensure it re-walks the page table.
        core::arch::asm!(
            "dsb ishst",
            "tlbi vmalle1is",
            "dsb ish",
            "isb",
            options(nostack, nomem)
        );
    }
}

/// Get the boot TTBR0 physical address (identity mapping)
pub fn boot_ttbr0() -> u64 {
    unsafe { BOOT_TTBR0 }
}

/// Get the boot TTBR1 physical address (kernel mapping)
pub fn boot_ttbr1() -> u64 {
    unsafe { BOOT_TTBR1 }
}

/// Switch to a new user address space (change TTBR0)
/// With ASID support, TTBR0 should include the ASID in bits [63:48].
/// TLB entries are tagged with ASID, so no flush needed on context switch.
/// # Safety
/// The page table must be valid and properly set up
pub unsafe fn switch_user_space(ttbr0_with_asid: u64) {
    core::arch::asm!(
        // MEMORY BARRIER: Ensure all prior stores are visible before
        // switching address spaces. Critical for SMP correctness.
        "dsb sy",
        "msr ttbr0_el1, {0}",
        "isb",
        // No TLB flush - entries are ASID-tagged
        in(reg) ttbr0_with_asid
    );
}

// NOTE: TLB invalidation wrappers removed - use arch::aarch64::tlb directly

/// Get current TTBR0 value
pub fn ttbr0() -> u64 {
    let val: u64;
    unsafe {
        core::arch::asm!("mrs {}, ttbr0_el1", out(reg) val);
    }
    val
}

/// Get current TTBR1 value
pub fn ttbr1() -> u64 {
    let val: u64;
    unsafe {
        core::arch::asm!("mrs {}, ttbr1_el1", out(reg) val);
    }
    val
}

/// Check if MMU is enabled
pub fn is_enabled() -> bool {
    let sctlr: u64;
    unsafe {
        core::arch::asm!("mrs {}, sctlr_el1", out(reg) sctlr);
    }
    (sctlr & 1) != 0
}

/// Print MMU info
pub fn print_info() {
    kdebug!("mmu", "info"; ttbr0 = klog::hex64(ttbr0()), ttbr1 = klog::hex64(ttbr1()), enabled = is_enabled());
}

// ============================================================================
// Guard Page Support
// ============================================================================

/// L3 page table for splitting a 2MB block into 4KB pages (for guard pages).
/// Statically allocated to avoid PMM dependency during early boot.
static mut GUARD_L3_TABLE: PageTable = PageTable::new();

/// Set up guard pages for boot stack and secondary CPU stacks.
///
/// This splits the relevant 2MB L2 block(s) into L3 4KB pages,
/// then marks guard pages as invalid (unmapped). Any stack overflow
/// that reaches the guard page will trigger a data abort.
///
/// Must be called after MMU is enabled and before userspace starts.
pub fn setup_guard_pages() {
    // Boot stack guard page
    extern "C" {
        static __boot_stack_guard: u8;
    }
    let guard_vaddr = unsafe { &__boot_stack_guard as *const u8 as u64 };
    unmap_kernel_page(guard_vaddr);

    // Secondary CPU stack guard pages
    setup_cpu_stack_guards();
}

/// Unmap a single 4KB page in the kernel (TTBR1) address space.
///
/// Splits the containing 2MB L2 block into L3 4KB pages, then
/// marks the target page as invalid. Uses the static GUARD_L3_TABLE.
///
/// Note: Currently supports unmapping one 2MB block region only
/// (the one containing all guard pages, which are in the BSS/stack area).
fn unmap_kernel_page(vaddr: u64) {
    // Convert to physical address
    let phys = virt_to_phys(vaddr);

    // Find which L2 block this belongs to
    let l2_block_base = phys & !0x1F_FFFF; // 2MB aligned
    let l1_index = ((phys >> 30) & 0x1FF) as usize;
    let l2_index = ((phys >> 21) & 0x1FF) as usize;
    let l3_index = ((phys >> 12) & 0x1FF) as usize;

    // Get physical address of the L3 table
    let l3_phys = unsafe { virt_to_phys(core::ptr::addr_of!(GUARD_L3_TABLE) as u64) };

    // Check if L3 table is already populated (split already done)
    let l3_populated = unsafe { GUARD_L3_TABLE.entries[0] != 0 };

    if !l3_populated {
        // Populate L3 table: 512 x 4KB pages matching the original 2MB block
        for i in 0..ENTRIES_PER_TABLE {
            let page_phys = l2_block_base + (i as u64 * PAGE_SIZE as u64);
            // Normal memory, kernel-only, inner shareable, UXN (EL1 can execute)
            unsafe {
                GUARD_L3_TABLE.entries[i] = page_phys
                    | flags::VALID
                    | flags::PAGE
                    | flags::AF
                    | flags::SH_INNER
                    | attr::NORMAL
                    | (0x0040u64 << 48); // UXN only (EL1 can execute)
            }
        }
    }

    // Mark the guard page as invalid (unmapped)
    unsafe {
        GUARD_L3_TABLE.entries[l3_index] = 0;
    }

    // Point the L2 entry to the L3 table instead of a 2MB block.
    // We need to find the L2 table that contains this entry.
    // For TTBR1, L1 entry points to L2 table.
    extern "C" {
        static boot_l2_ttbr1_0: [u64; 512];
        static boot_l2_ttbr1_1: [u64; 512];
    }

    let l2_table_ptr: *mut u64 = if l1_index == 0 {
        unsafe { boot_l2_ttbr1_0.as_ptr() as *mut u64 }
    } else if l1_index == 1 {
        unsafe { boot_l2_ttbr1_1.as_ptr() as *mut u64 }
    } else {
        // Guard page is in an L1 region we don't have an L2 table for
        return;
    };

    // Replace the L2 block descriptor with a table descriptor pointing to L3
    unsafe {
        let entry_ptr = l2_table_ptr.add(l2_index);
        core::ptr::write_volatile(entry_ptr, l3_phys | flags::VALID | flags::TABLE);

        // Full TLB invalidation
        core::arch::asm!(
            "dsb ishst",
            "tlbi vmalle1is",
            "dsb ish",
            "isb",
            options(nostack, nomem)
        );
    }

    kdebug!("mmu", "guard_page_unmapped"; vaddr = klog::hex64(vaddr), phys = klog::hex64(phys));
}

/// Set up guard pages for secondary CPU stacks.
///
/// The CPU stacks are laid out as [16KB stack][16KB stack]...
/// Guard pages would go at the base of each stack. However, since
/// secondary stacks use a separate static array (CPU_STACKS), and
/// the stacks are in BSS, they may be in a different 2MB block than
/// the boot stack. For simplicity, we only set up the boot stack
/// guard page now, and document that secondary stack guards would
/// need additional L3 tables.
fn setup_cpu_stack_guards() {
    // Secondary CPU stack guard pages require additional L3 tables
    // (one per 2MB block containing stack memory). This is deferred
    // until we have a PMM-backed L3 table allocator.
    // For now, secondary CPUs use the full CPU_STACK_SIZE without guards.
}
