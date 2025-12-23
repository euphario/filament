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

use crate::logln;

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
    logln!("  TTBR0_EL1: 0x{:016x} (user/identity)", ttbr0());
    logln!("  TTBR1_EL1: 0x{:016x} (kernel)", ttbr1());
    logln!("  MMU:       {}", if is_enabled() { "enabled" } else { "disabled" });
}
