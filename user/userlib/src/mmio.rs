//! MMIO helpers for memory-mapped I/O access
//!
//! Provides a safe-ish wrapper around memory-mapped I/O regions.
//! Uses the unified object interface for mapping physical addresses.

use crate::syscall::{self, Handle, ObjectType};

/// MMIO region handle
///
/// Represents a mapped memory-mapped I/O region. The region is unmapped
/// when the handle is dropped.
pub struct MmioRegion {
    handle: Handle,
    base: u64,
    size: u64,
}

impl MmioRegion {
    /// Open an MMIO region at the given physical address
    ///
    /// Returns None if the mapping fails.
    pub fn open(phys_addr: u64, size: u64) -> Option<Self> {
        // Build params: u64 phys_addr, u64 size (16 bytes)
        let mut params = [0u8; 16];
        params[0..8].copy_from_slice(&phys_addr.to_le_bytes());
        params[8..16].copy_from_slice(&size.to_le_bytes());

        // Open MMIO object (kernel maps it during open)
        let handle = syscall::open(ObjectType::Mmio, &params).ok()?;

        // Get virtual address via map syscall
        let base = syscall::map(handle, 0).ok()?;
        if base == 0 {
            let _ = syscall::close(handle);
            return None;
        }

        Some(Self { handle, base, size })
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
            core::ptr::write_volatile((self.base + offset as u64) as *mut u32, value);
            // dsb sy ensures write completes before any subsequent operation
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
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
        let _ = syscall::close(self.handle);
    }
}

// =============================================================================
// DMA Pool
// =============================================================================

/// DMA-capable memory pool
///
/// Allocates physically contiguous, non-cacheable memory suitable for DMA.
/// Use this for USB rings, command buffers, etc.
pub struct DmaPool {
    handle: Handle,
    vaddr: u64,
    paddr: u64,
    size: usize,
}

impl DmaPool {
    /// Allocate a DMA pool of the given size
    ///
    /// Returns None if allocation fails.
    /// The memory is physically contiguous and non-cacheable.
    pub fn alloc(size: usize) -> Option<Self> {
        // Build params: u64 size, u32 flags (16 bytes)
        // flags bit 0: 1 = high memory pool, 0 = low memory pool
        let mut params = [0u8; 16];
        params[0..8].copy_from_slice(&(size as u64).to_le_bytes());
        params[8..12].copy_from_slice(&0u32.to_le_bytes()); // low memory

        // Open DMA pool object
        let handle = syscall::open(ObjectType::DmaPool, &params).ok()?;

        // Get virtual address via map syscall
        let vaddr = match syscall::map(handle, 0) {
            Ok(v) if v != 0 => v,
            _ => {
                let _ = syscall::close(handle);
                return None;
            }
        };

        // Get physical address via read syscall (returns paddr + size)
        let mut info = [0u8; 16];
        if syscall::read(handle, &mut info).ok()? != 16 {
            let _ = syscall::close(handle);
            return None;
        }

        let paddr = u64::from_le_bytes([
            info[0], info[1], info[2], info[3],
            info[4], info[5], info[6], info[7],
        ]);

        Some(Self { handle, vaddr, paddr, size })
    }

    /// Get the virtual base address
    #[inline]
    pub fn vaddr(&self) -> u64 {
        self.vaddr
    }

    /// Get the physical base address (for DMA)
    #[inline]
    pub fn paddr(&self) -> u64 {
        self.paddr
    }

    /// Get the allocation size
    #[inline]
    pub fn size(&self) -> usize {
        self.size
    }

    /// Get a pointer to the memory
    #[inline]
    pub fn as_ptr(&self) -> *mut u8 {
        self.vaddr as *mut u8
    }

    /// Get a slice of the memory
    ///
    /// # Safety
    /// Caller must ensure no concurrent DMA access to this region.
    #[inline]
    pub unsafe fn as_slice(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.vaddr as *const u8, self.size) }
    }

    /// Get a mutable slice of the memory
    ///
    /// # Safety
    /// Caller must ensure no concurrent DMA access to this region.
    #[inline]
    pub unsafe fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.vaddr as *mut u8, self.size) }
    }

    /// Zero the memory
    pub fn zero(&mut self) {
        unsafe {
            core::ptr::write_bytes(self.vaddr as *mut u8, 0, self.size);
        }
    }

    /// Read a u32 at offset
    #[inline]
    pub fn read32(&self, offset: usize) -> u32 {
        debug_assert!(offset + 4 <= self.size);
        unsafe {
            core::ptr::read_volatile((self.vaddr + offset as u64) as *const u32)
        }
    }

    /// Write a u32 at offset
    #[inline]
    pub fn write32(&self, offset: usize, value: u32) {
        debug_assert!(offset + 4 <= self.size);
        unsafe {
            core::ptr::write_volatile((self.vaddr + offset as u64) as *mut u32, value);
        }
    }

    /// Read a u64 at offset
    #[inline]
    pub fn read64(&self, offset: usize) -> u64 {
        debug_assert!(offset + 8 <= self.size);
        unsafe {
            core::ptr::read_volatile((self.vaddr + offset as u64) as *const u64)
        }
    }

    /// Write a u64 at offset
    #[inline]
    pub fn write64(&self, offset: usize, value: u64) {
        debug_assert!(offset + 8 <= self.size);
        unsafe {
            core::ptr::write_volatile((self.vaddr + offset as u64) as *mut u64, value);
        }
    }
}

impl Drop for DmaPool {
    fn drop(&mut self) {
        let _ = syscall::close(self.handle);
    }
}

/// Format MMIO URL: "mmio:ADDR/SIZE"
#[allow(dead_code)]
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
#[allow(dead_code)]
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

/// Delay for approximately `ms` milliseconds
///
/// Uses light oneshot sleep syscall - task sleeps, scheduler can idle CPU.
pub fn delay_ms(ms: u32) {
    if ms == 0 {
        return;
    }
    syscall::sleep_ms(ms);
}

/// Delay for approximately `us` microseconds
///
/// < 100us: spin (hardware precision, syscall overhead not worth it)
/// >= 100us: sleep syscall (scheduler can idle CPU)
pub fn delay_us(us: u32) {
    if us == 0 {
        return;
    }
    if us >= 100 {
        syscall::sleep_us(us);
    } else {
        // Short delay - spin for precision
        let start = syscall::gettime();
        let end = start + (us as u64) * 1_000;
        while syscall::gettime() < end {
            core::hint::spin_loop();
        }
    }
}

/// Poll a condition with timeout
///
/// Calls `condition()` repeatedly until it returns `true` or `timeout_ms` expires.
/// Returns `true` if condition was met, `false` on timeout.
///
/// Sleeps 1ms between polls to avoid busy-waiting.
/// The `label` is used for debug output on timeout.
pub fn poll_until<F>(label: &str, timeout_ms: u32, mut condition: F) -> bool
where
    F: FnMut() -> bool,
{
    let start = syscall::gettime();
    let deadline = start + (timeout_ms as u64) * 1_000_000;

    loop {
        if condition() {
            return true;
        }

        if syscall::gettime() >= deadline {
            crate::println!("poll_until '{}' timeout after {}ms", label, timeout_ms);
            return false;
        }

        // Sleep 1ms between polls
        syscall::sleep_ms(1);
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
