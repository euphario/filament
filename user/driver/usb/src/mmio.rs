//! MMIO helpers for memory-mapped I/O access

use userlib::syscall;

pub struct MmioRegion {
    pub fd: i32,
    pub base: u64,
}

impl MmioRegion {
    pub fn open(phys_addr: u64, size: u64) -> Option<Self> {
        // Format URL as hex without 0x prefix
        let mut url_buf = [0u8; 32];
        let url_len = format_mmio_url(&mut url_buf, phys_addr, size);
        let url = unsafe { core::str::from_utf8_unchecked(&url_buf[..url_len]) };

        let fd = syscall::scheme_open(url, 0);
        if fd < 0 {
            return None;
        }

        // Read virtual address
        let mut virt_buf = [0u8; 8];
        if syscall::read(fd as u32, &mut virt_buf) < 8 {
            syscall::close(fd as u32);
            return None;
        }

        let base = u64::from_le_bytes(virt_buf);
        Some(Self { fd, base })
    }

    #[inline(always)]
    pub fn read32(&self, offset: usize) -> u32 {
        unsafe {
            let ptr = (self.base + offset as u64) as *const u32;
            core::ptr::read_volatile(ptr)
        }
    }

    #[inline(always)]
    pub fn write32(&self, offset: usize, value: u32) {
        unsafe {
            let ptr = (self.base + offset as u64) as *mut u32;
            core::ptr::write_volatile(ptr, value);
        }
    }

    #[inline(always)]
    pub fn read64(&self, offset: usize) -> u64 {
        unsafe {
            let ptr = (self.base + offset as u64) as *const u64;
            core::ptr::read_volatile(ptr)
        }
    }

    #[inline(always)]
    pub fn write64(&self, offset: usize, value: u64) {
        unsafe {
            let ptr = (self.base + offset as u64) as *mut u64;
            core::ptr::write_volatile(ptr, value);
        }
    }
}

impl Drop for MmioRegion {
    fn drop(&mut self) {
        syscall::close(self.fd as u32);
    }
}

pub fn format_mmio_url(buf: &mut [u8], phys: u64, size: u64) -> usize {
    // Format: "mmio:ADDR/SIZE" in hex
    let prefix = b"mmio:";
    buf[..5].copy_from_slice(prefix);

    let mut pos = 5;
    pos += format_hex(&mut buf[pos..], phys);
    buf[pos] = b'/';
    pos += 1;
    pos += format_hex(&mut buf[pos..], size);
    pos
}

pub fn format_hex(buf: &mut [u8], val: u64) -> usize {
    let hex = b"0123456789abcdef";
    let mut v = val;
    let mut len = 0;

    // Find number of digits
    let mut temp = val;
    loop {
        len += 1;
        temp >>= 4;
        if temp == 0 {
            break;
        }
    }

    // Write digits in reverse
    for i in (0..len).rev() {
        buf[i] = hex[(v & 0xF) as usize];
        v >>= 4;
    }
    len
}

/// Delay for approximately `ms` milliseconds using the ARM timer
/// This reads the hardware counter and CAN'T be optimized away
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
    }
}

pub fn delay(iterations: u32) {
    for i in 0..iterations {
        unsafe { core::arch::asm!("nop", options(nostack, nomem, preserves_flags)); }
        core::hint::black_box(i);
    }
}

pub fn print_hex64(val: u64) {
    print_hex32((val >> 32) as u32);
    print_hex32(val as u32);
}

pub fn print_hex32(val: u32) {
    let hex = b"0123456789abcdef";
    let mut buf = [b'0'; 8];
    let mut n = val;
    for i in (0..8).rev() {
        buf[i] = hex[(n & 0xF) as usize];
        n >>= 4;
    }
    let _ = syscall::write(syscall::STDOUT, &buf);
}

pub fn print_hex8(val: u8) {
    let hex = b"0123456789abcdef";
    let buf = [hex[(val >> 4) as usize], hex[(val & 0xF) as usize]];
    let _ = syscall::write(syscall::STDOUT, &buf);
}
