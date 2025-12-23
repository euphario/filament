//! TLB (Translation Lookaside Buffer) Management
//!
//! Centralized API for all TLB operations on AArch64. All TLB invalidations
//! should go through this module to ensure consistency and proper barriers.
//!
//! ## Key Operations
//!
//! - `invalidate_va(asid, va)` - Invalidate a single VA for an ASID
//! - `invalidate_va_range(asid, va, len)` - Invalidate a range of VAs
//! - `invalidate_asid(asid)` - Invalidate all entries for an ASID
//! - `invalidate_all()` - Invalidate all TLB entries (heavyweight, avoid)
//!
//! ## Usage Guidelines
//!
//! 1. After unmapping a page: use `invalidate_va()`
//! 2. After destroying an address space: use `invalidate_asid()`
//! 3. After changing kernel mappings: use `invalidate_kernel_va()`
//! 4. Never use `invalidate_all()` except during initialization
//!
//! ## ASID Notes
//!
//! - ASID 0 is reserved for the kernel
//! - User processes get ASIDs 1-255
//! - TLB entries are tagged with ASIDs, allowing per-process invalidation
//! - TTBR0 bits [63:48] contain the ASID for hardware tagging

#![allow(dead_code)]

/// Invalidate TLB entry for a single user virtual address with ASID
///
/// Uses `tlbi vale1is` (VA, Last level, EL1, Inner Shareable) which:
/// - Invalidates only the final page table level
/// - Broadcasts to all CPUs (Inner Shareable)
/// - Includes the ASID in the invalidation
///
/// # Arguments
/// * `asid` - Address Space ID (1-255, 0 for kernel)
/// * `va` - Virtual address (must be page-aligned for efficiency)
///
/// # Safety
/// This is safe to call but should only be used after unmapping a page.
#[inline]
pub fn invalidate_va(asid: u16, va: u64) {
    // TLBI VAE1IS format:
    // - Bits [63:48]: ASID
    // - Bits [47:12]: VA[47:12] (VA divided by page size)
    // - Bits [11:0]: Reserved
    let asid_val = (asid as u64) << 48;
    let va_val = (va >> 12) & 0x0000_FFFF_FFFF_FFFF;
    let tlbi_val = asid_val | va_val;

    unsafe {
        core::arch::asm!(
            "tlbi vale1is, {0}",
            "dsb ish",
            in(reg) tlbi_val,
            options(nostack, preserves_flags)
        );
    }
}

/// Invalidate TLB entries for a range of user virtual addresses
///
/// More efficient than calling `invalidate_va()` in a loop because
/// it batches the DSB barrier at the end.
///
/// # Arguments
/// * `asid` - Address Space ID (1-255, 0 for kernel)
/// * `va` - Starting virtual address
/// * `num_pages` - Number of 4KB pages to invalidate
#[inline]
pub fn invalidate_va_range(asid: u16, va: u64, num_pages: usize) {
    let asid_val = (asid as u64) << 48;

    unsafe {
        for i in 0..num_pages {
            let page_va = va + (i as u64 * 4096);
            let va_val = (page_va >> 12) & 0x0000_FFFF_FFFF_FFFF;
            let tlbi_val = asid_val | va_val;
            core::arch::asm!(
                "tlbi vale1is, {0}",
                in(reg) tlbi_val,
                options(nostack, preserves_flags)
            );
        }
        // Single barrier after all invalidations
        core::arch::asm!(
            "dsb ish",
            "isb",
            options(nostack, preserves_flags)
        );
    }
}

/// Invalidate all TLB entries for a specific ASID
///
/// Uses `tlbi aside1is` (ASID, Inner Shareable) to invalidate all
/// entries tagged with the given ASID across all CPUs.
///
/// Call this when destroying an address space.
///
/// # Arguments
/// * `asid` - Address Space ID (1-255)
#[inline]
pub fn invalidate_asid(asid: u16) {
    // TLBI ASIDE1IS format:
    // - Bits [63:48]: ASID
    // - Bits [47:0]: Reserved (SBZ)
    let asid_val = (asid as u64) << 48;

    unsafe {
        core::arch::asm!(
            "tlbi aside1is, {0}",
            "dsb ish",
            "isb",
            in(reg) asid_val,
            options(nostack, preserves_flags)
        );
    }
}

/// Invalidate TLB entry for a kernel virtual address (ASID 0)
///
/// For kernel mappings which use ASID 0 or are in TTBR1 space.
///
/// # Arguments
/// * `va` - Kernel virtual address
#[inline]
pub fn invalidate_kernel_va(va: u64) {
    // TLBI VAAE1IS format: VA only, affects all ASIDs
    // This is used for kernel addresses which may be shared
    let va_val = (va >> 12) & 0x0000_FFFF_FFFF_FFFF;

    unsafe {
        core::arch::asm!(
            "tlbi vaae1is, {0}",
            "dsb ish",
            in(reg) va_val,
            options(nostack, preserves_flags)
        );
    }
}

/// Invalidate ALL TLB entries
///
/// **WARNING**: This is a heavyweight operation that flushes the entire TLB
/// across all CPUs. Only use during:
/// - Initial boot setup
/// - Catastrophic recovery
/// - When ASIDs are exhausted and being recycled
///
/// For normal operation, prefer `invalidate_va()` or `invalidate_asid()`.
#[inline]
pub fn invalidate_all() {
    unsafe {
        core::arch::asm!(
            "tlbi vmalle1is",
            "dsb ish",
            "isb",
            options(nostack, preserves_flags)
        );
    }
}

/// Invalidate TLB entries for current ASID only (local CPU)
///
/// Non-broadcast version for local-only invalidation.
/// Use when you know the address space is only active on this CPU.
///
/// # Arguments
/// * `asid` - Address Space ID (1-255)
#[inline]
pub fn invalidate_asid_local(asid: u16) {
    let asid_val = (asid as u64) << 48;

    unsafe {
        core::arch::asm!(
            "tlbi aside1, {0}",
            "dsb nsh",
            "isb",
            in(reg) asid_val,
            options(nostack, preserves_flags)
        );
    }
}

/// Invalidate single VA for current ASID (local CPU only)
///
/// Non-broadcast version - use only when certain the mapping
/// isn't cached on other CPUs.
///
/// # Arguments
/// * `asid` - Address Space ID
/// * `va` - Virtual address
#[inline]
pub fn invalidate_va_local(asid: u16, va: u64) {
    let asid_val = (asid as u64) << 48;
    let va_val = (va >> 12) & 0x0000_FFFF_FFFF_FFFF;
    let tlbi_val = asid_val | va_val;

    unsafe {
        core::arch::asm!(
            "tlbi vale1, {0}",
            "dsb nsh",
            in(reg) tlbi_val,
            options(nostack, preserves_flags)
        );
    }
}

/// Ensure all TLB maintenance operations are complete
///
/// Call this after a sequence of TLB operations if you need to
/// ensure they're visible before proceeding.
#[inline]
pub fn barrier() {
    unsafe {
        core::arch::asm!(
            "dsb ish",
            "isb",
            options(nostack, preserves_flags)
        );
    }
}
