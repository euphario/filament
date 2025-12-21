//! Memory-Mapped I/O Abstraction
//!
//! Provides safe(r) abstractions for MMIO register access with proper
//! volatile semantics and memory barriers.
//!
//! All device register access should go through this module.
//!
//! After MMU is enabled, MMIO addresses are accessed via TTBR1 (kernel virtual
//! addresses) to avoid dependency on TTBR0 which may be pointing to user space.

#![allow(dead_code)]  // Many methods are infrastructure for drivers

use core::ptr::{read_volatile, write_volatile};
use crate::arch::aarch64::mmu;

/// A memory-mapped I/O region.
///
/// Wraps a base address and provides methods for reading/writing registers
/// with correct volatile semantics and optional barriers.
///
/// Before MMU is enabled, uses physical addresses directly.
/// After MMU is enabled, converts to TTBR1 virtual addresses.
///
/// # Example
/// ```
/// let uart = MmioRegion::new(platform::UART0_BASE);
/// uart.write32(0x00, 0x42);       // Write to offset 0
/// let val = uart.read32(0x14);    // Read from offset 0x14
/// uart.write32_flush(0x00, 0x43); // Write + readback barrier
/// ```
#[derive(Clone, Copy)]
pub struct MmioRegion {
    /// Physical base address
    base: usize,
}

impl MmioRegion {
    /// Create a new MMIO region at the given physical base address.
    #[inline]
    pub const fn new(base: usize) -> Self {
        Self { base }
    }

    /// Get the effective address for MMIO access.
    /// Before MMU: returns physical address
    /// After MMU: returns TTBR1 virtual address
    #[inline]
    fn effective_addr(&self, offset: usize) -> usize {
        let phys = self.base + offset;
        if mmu::is_enabled() {
            phys | mmu::KERNEL_VIRT_BASE as usize
        } else {
            phys
        }
    }

    /// Get the base address of this region.
    #[inline]
    pub const fn base(&self) -> usize {
        self.base
    }

    // =========================================================================
    // 8-bit access
    // =========================================================================

    /// Read an 8-bit register at the given offset.
    #[inline]
    pub fn read8(&self, offset: usize) -> u8 {
        unsafe { read_volatile(self.effective_addr(offset) as *const u8) }
    }

    /// Write an 8-bit register at the given offset.
    #[inline]
    pub fn write8(&self, offset: usize, value: u8) {
        unsafe { write_volatile(self.effective_addr(offset) as *mut u8, value) }
    }

    // =========================================================================
    // 16-bit access
    // =========================================================================

    /// Read a 16-bit register at the given offset.
    #[inline]
    pub fn read16(&self, offset: usize) -> u16 {
        unsafe { read_volatile(self.effective_addr(offset) as *const u16) }
    }

    /// Write a 16-bit register at the given offset.
    #[inline]
    pub fn write16(&self, offset: usize, value: u16) {
        unsafe { write_volatile(self.effective_addr(offset) as *mut u16, value) }
    }

    // =========================================================================
    // 32-bit access
    // =========================================================================

    /// Read a 32-bit register at the given offset.
    #[inline]
    pub fn read32(&self, offset: usize) -> u32 {
        unsafe { read_volatile(self.effective_addr(offset) as *const u32) }
    }

    /// Write a 32-bit register at the given offset.
    #[inline]
    pub fn write32(&self, offset: usize, value: u32) {
        unsafe { write_volatile(self.effective_addr(offset) as *mut u32, value) }
    }

    /// Write a 32-bit register and read it back (for ordering).
    ///
    /// Some devices require a readback to ensure the write has completed
    /// before proceeding. This is stronger than a memory barrier.
    #[inline]
    pub fn write32_flush(&self, offset: usize, value: u32) {
        self.write32(offset, value);
        let _ = self.read32(offset);
    }

    /// Modify a 32-bit register with a read-modify-write sequence.
    ///
    /// Reads the current value, applies the closure, and writes back.
    #[inline]
    pub fn modify32<F>(&self, offset: usize, f: F)
    where
        F: FnOnce(u32) -> u32,
    {
        let val = self.read32(offset);
        self.write32(offset, f(val));
    }

    /// Set bits in a 32-bit register (read-modify-write).
    #[inline]
    pub fn set_bits32(&self, offset: usize, bits: u32) {
        self.modify32(offset, |v| v | bits);
    }

    /// Clear bits in a 32-bit register (read-modify-write).
    #[inline]
    pub fn clear_bits32(&self, offset: usize, bits: u32) {
        self.modify32(offset, |v| v & !bits);
    }

    // =========================================================================
    // 64-bit access
    // =========================================================================

    /// Read a 64-bit register at the given offset.
    #[inline]
    pub fn read64(&self, offset: usize) -> u64 {
        unsafe { read_volatile(self.effective_addr(offset) as *const u64) }
    }

    /// Write a 64-bit register at the given offset.
    #[inline]
    pub fn write64(&self, offset: usize, value: u64) {
        unsafe { write_volatile(self.effective_addr(offset) as *mut u64, value) }
    }

    // =========================================================================
    // Polling helpers
    // =========================================================================

    /// Poll a 32-bit register until a condition is met or timeout.
    ///
    /// Returns `true` if condition was met, `false` if timeout.
    #[inline]
    pub fn poll32<F>(&self, offset: usize, max_polls: u32, condition: F) -> bool
    where
        F: Fn(u32) -> bool,
    {
        for _ in 0..max_polls {
            if condition(self.read32(offset)) {
                return true;
            }
            core::hint::spin_loop();
        }
        false
    }

    /// Wait for specific bits to be set in a 32-bit register.
    #[inline]
    pub fn wait_bits_set32(&self, offset: usize, bits: u32, max_polls: u32) -> bool {
        self.poll32(offset, max_polls, |v| (v & bits) == bits)
    }

    /// Wait for specific bits to be clear in a 32-bit register.
    #[inline]
    pub fn wait_bits_clear32(&self, offset: usize, bits: u32, max_polls: u32) -> bool {
        self.poll32(offset, max_polls, |v| (v & bits) == 0)
    }

    // =========================================================================
    // Subregion access
    // =========================================================================

    /// Create a subregion at an offset from this region.
    ///
    /// Useful for devices with multiple register blocks.
    #[inline]
    pub const fn subregion(&self, offset: usize) -> MmioRegion {
        MmioRegion::new(self.base + offset)
    }
}

// =============================================================================
// Memory Barriers
// =============================================================================

/// Data Memory Barrier (DMB) - ensures ordering of memory accesses.
///
/// Use after writes that must complete before subsequent reads/writes.
#[inline]
pub fn dmb() {
    unsafe {
        core::arch::asm!("dmb sy");
    }
}

/// Data Synchronization Barrier (DSB) - ensures completion of memory accesses.
///
/// Stronger than DMB - waits for all memory accesses to complete.
/// Use before changing system registers or after DMA setup.
#[inline]
pub fn dsb() {
    unsafe {
        core::arch::asm!("dsb sy");
    }
}

/// Instruction Synchronization Barrier (ISB).
///
/// Flushes the pipeline. Use after modifying system registers.
#[inline]
pub fn isb() {
    unsafe {
        core::arch::asm!("isb");
    }
}

/// DSB + ISB sequence.
///
/// Common pattern after modifying system configuration.
#[inline]
pub fn dsb_isb() {
    dsb();
    isb();
}

// =============================================================================
// Cache Operations (for DMA)
// =============================================================================

/// Clean a cache line (write back to memory).
#[inline]
pub fn cache_clean_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc cvac, {}", in(reg) addr);
    }
}

/// Invalidate a cache line (discard cached data).
#[inline]
pub fn cache_invalidate_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc ivac, {}", in(reg) addr);
    }
}

/// Clean and invalidate a cache line.
#[inline]
pub fn cache_clean_invalidate_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc civac, {}", in(reg) addr);
    }
}

/// Clean a memory range (write back to memory).
pub fn cache_clean_range(start: u64, size: usize) {
    let end = start + size as u64;
    let mut addr = start & !63; // Align to cache line
    while addr < end {
        cache_clean_line(addr);
        addr += 64;
    }
    dsb();
}

/// Invalidate a memory range (discard cached data).
pub fn cache_invalidate_range(start: u64, size: usize) {
    let end = start + size as u64;
    let mut addr = start & !63; // Align to cache line
    while addr < end {
        cache_invalidate_line(addr);
        addr += 64;
    }
    dsb();
}

/// Clean and invalidate a memory range.
pub fn cache_clean_invalidate_range(start: u64, size: usize) {
    let end = start + size as u64;
    let mut addr = start & !63; // Align to cache line
    while addr < end {
        cache_clean_invalidate_line(addr);
        addr += 64;
    }
    dsb();
}

// =============================================================================
// Delay helpers
// =============================================================================

/// Spin delay for approximately `us` microseconds.
///
/// Uses the ARM generic timer counter for timing.
/// Note: This is a busy-wait, not suitable for long delays.
pub fn delay_us(us: u64) {
    let freq: u64;
    let start: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) start);
    }

    // Calculate ticks to wait
    let ticks = (freq * us) / 1_000_000;
    let target = start + ticks;

    loop {
        let now: u64;
        unsafe {
            core::arch::asm!("mrs {}, cntpct_el0", out(reg) now);
        }
        if now >= target {
            break;
        }
        core::hint::spin_loop();
    }
}

/// Spin delay for approximately `ms` milliseconds.
#[inline]
pub fn delay_ms(ms: u64) {
    delay_us(ms * 1000);
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
pub fn test() {
    crate::logln!("  MMIO module ready");
}
