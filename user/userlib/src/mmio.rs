//! MMIO helpers for memory-mapped I/O access
//!
//! Provides a safe-ish wrapper around memory-mapped I/O regions.
//! Uses the kernel's mmio scheme for mapping physical addresses.

use crate::syscall;

/// MMIO region handle
///
/// Represents a mapped memory-mapped I/O region. The region is unmapped
/// when the handle is dropped.
pub struct MmioRegion {
    fd: i32,
    base: u64,
    size: u64,
}

impl MmioRegion {
    /// Open an MMIO region at the given physical address
    ///
    /// Returns None if the mapping fails.
    pub fn open(phys_addr: u64, size: u64) -> Option<Self> {
        let mut url_buf = [0u8; 64];
        let url_len = format_mmio_url(&mut url_buf, phys_addr, size);

        let url_str = core::str::from_utf8(&url_buf[..url_len]).ok()?;
        let fd = syscall::scheme_open(url_str, 0);
        if fd < 0 {
            return None;
        }

        // Read virtual address from kernel
        let mut virt_buf = [0u8; 8];
        let n = syscall::read(fd as u32, &mut virt_buf);
        if n != 8 {
            syscall::close(fd as u32);
            return None;
        }

        let base = u64::from_le_bytes(virt_buf);
        Some(Self { fd, base, size })
    }

    /// Get the virtual base address
    #[inline]
    pub fn virt_base(&self) -> u64 {
        self.base
    }

    /// Get the mapped size
    #[inline]
    pub fn size(&self) -> u64 {
        self.size
    }

    /// Read 8-bit value at offset
    #[inline]
    pub fn read8(&self, offset: usize) -> u8 {
        debug_assert!(offset < self.size as usize);
        unsafe {
            core::ptr::read_volatile((self.base + offset as u64) as *const u8)
        }
    }

    /// Write 8-bit value at offset
    #[inline]
    pub fn write8(&self, offset: usize, value: u8) {
        debug_assert!(offset < self.size as usize);
        unsafe {
            core::ptr::write_volatile((self.base + offset as u64) as *mut u8, value)
        }
    }

    /// Read 16-bit value at offset
    #[inline]
    pub fn read16(&self, offset: usize) -> u16 {
        debug_assert!(offset + 1 < self.size as usize);
        unsafe {
            core::ptr::read_volatile((self.base + offset as u64) as *const u16)
        }
    }

    /// Write 16-bit value at offset
    #[inline]
    pub fn write16(&self, offset: usize, value: u16) {
        debug_assert!(offset + 1 < self.size as usize);
        unsafe {
            core::ptr::write_volatile((self.base + offset as u64) as *mut u16, value)
        }
    }

    /// Read 32-bit value at offset
    #[inline]
    pub fn read32(&self, offset: usize) -> u32 {
        debug_assert!(offset + 3 < self.size as usize);
        unsafe {
            core::ptr::read_volatile((self.base + offset as u64) as *const u32)
        }
    }

    /// Write 32-bit value at offset
    #[inline]
    pub fn write32(&self, offset: usize, value: u32) {
        debug_assert!(offset + 3 < self.size as usize);
        unsafe {
            core::ptr::write_volatile((self.base + offset as u64) as *mut u32, value)
        }
    }

    /// Read 64-bit value at offset
    #[inline]
    pub fn read64(&self, offset: usize) -> u64 {
        debug_assert!(offset + 7 < self.size as usize);
        unsafe {
            core::ptr::read_volatile((self.base + offset as u64) as *const u64)
        }
    }

    /// Write 64-bit value at offset
    #[inline]
    pub fn write64(&self, offset: usize, value: u64) {
        debug_assert!(offset + 7 < self.size as usize);
        unsafe {
            core::ptr::write_volatile((self.base + offset as u64) as *mut u64, value)
        }
    }

    /// Set bits in a 32-bit register (read-modify-write)
    #[inline]
    pub fn set32(&self, offset: usize, bits: u32) {
        let val = self.read32(offset);
        self.write32(offset, val | bits);
    }

    /// Clear bits in a 32-bit register (read-modify-write)
    #[inline]
    pub fn clr32(&self, offset: usize, bits: u32) {
        let val = self.read32(offset);
        self.write32(offset, val & !bits);
    }

    /// Modify bits in a 32-bit register: clear mask, then set bits
    #[inline]
    pub fn modify32(&self, offset: usize, clear: u32, set: u32) {
        let val = self.read32(offset);
        self.write32(offset, (val & !clear) | set);
    }
}

impl Drop for MmioRegion {
    fn drop(&mut self) {
        if self.fd >= 0 {
            syscall::close(self.fd as u32);
        }
    }
}

/// Format MMIO URL: "mmio:ADDR/SIZE"
fn format_mmio_url(buf: &mut [u8], addr: u64, size: u64) -> usize {
    let prefix = b"mmio:";
    let mut pos = 0;

    for &b in prefix {
        buf[pos] = b;
        pos += 1;
    }

    pos += format_hex(&mut buf[pos..], addr);
    buf[pos] = b'/';
    pos += 1;
    pos += format_hex(&mut buf[pos..], size);

    pos
}

/// Format u64 as hex string, return length
fn format_hex(buf: &mut [u8], val: u64) -> usize {
    const HEX: &[u8] = b"0123456789abcdef";

    if val == 0 {
        buf[0] = b'0';
        return 1;
    }

    let mut v = val;
    let mut digits = 0;
    while v > 0 {
        digits += 1;
        v >>= 4;
    }

    let mut v = val;
    for i in (0..digits).rev() {
        buf[i] = HEX[(v & 0xF) as usize];
        v >>= 4;
    }

    digits
}

/// Delay for approximately `ms` milliseconds using the ARM timer
///
/// This yields to the scheduler between timer checks, allowing other tasks to run.
pub fn delay_ms(ms: u32) {
    let freq: u64;
    let start: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) start);
    }
    let ticks_needed = (freq / 1000) * ms as u64;
    let end = start + ticks_needed;
    loop {
        let now: u64;
        unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) now); }
        if now >= end {
            break;
        }
        // Yield to scheduler instead of busy-spinning
        syscall::yield_now();
    }
}

/// Delay for approximately `us` microseconds using the ARM timer
///
/// For very short delays (<100us), this uses spin_loop() to avoid scheduler overhead.
/// For longer delays, yields to the scheduler between checks.
pub fn delay_us(us: u32) {
    let freq: u64;
    let start: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) start);
    }
    let ticks_needed = (freq / 1_000_000) * us as u64;
    let end = start + ticks_needed;

    // For very short delays, spin to avoid scheduler overhead
    // For longer delays, yield to allow other tasks to run
    let use_yield = us >= 100;

    loop {
        let now: u64;
        unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) now); }
        if now >= end {
            break;
        }
        if use_yield {
            syscall::yield_now();
        } else {
            core::hint::spin_loop();
        }
    }
}

/// Poll a condition with timeout
///
/// Calls `condition()` repeatedly until it returns `true` or `timeout_ms` expires.
/// Returns `true` if condition was met, `false` on timeout.
///
/// The `label` is used for debug output on timeout.
pub fn poll_until<F>(label: &str, timeout_ms: u32, mut condition: F) -> bool
where
    F: FnMut() -> bool,
{
    let freq: u64;
    let start: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) start);
    }
    let ticks_needed = (freq / 1000) * timeout_ms as u64;
    let end = start + ticks_needed;

    loop {
        if condition() {
            return true;
        }

        let now: u64;
        unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) now); }
        if now >= end {
            // Timeout - could log here if we had a trace! macro
            crate::println!("poll_until '{}' timeout after {}ms", label, timeout_ms);
            return false;
        }

        // Yield to scheduler between polls
        syscall::yield_now();
    }
}

/// Poll with interval - calls condition every `interval_ms`
pub fn poll_interval<F>(label: &str, timeout_ms: u32, interval_ms: u32, mut condition: F) -> bool
where
    F: FnMut() -> bool,
{
    let iterations = timeout_ms / interval_ms.max(1);
    for _ in 0..iterations {
        if condition() {
            return true;
        }
        delay_ms(interval_ms);
    }
    crate::println!("poll_interval '{}' timeout after {}ms", label, timeout_ms);
    false
}
