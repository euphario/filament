//! MMIO helpers for memory-mapped I/O access
//!
//! Re-exports MmioRegion and delay_ms from userlib, plus USB-specific helpers.

use userlib::syscall;

// Re-export from userlib (now uses unified object interface)
pub use userlib::mmio::{MmioRegion, DmaPool, delay_ms, delay_us, poll_until, poll_interval};

/// Format MMIO URL: "mmio:ADDR/SIZE" in hex
/// (kept for compatibility with existing code)
pub fn format_mmio_url(buf: &mut [u8], phys: u64, size: u64) -> usize {
    let prefix = b"mmio:";
    buf[..5].copy_from_slice(prefix);

    let mut pos = 5;
    pos += format_hex(&mut buf[pos..], phys);
    buf[pos] = b'/';
    pos += 1;
    pos += format_hex(&mut buf[pos..], size);
    pos
}

/// Format u64 as hex string, return length
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
    let _ = syscall::write(syscall::Handle::STDOUT, &buf);
}

pub fn print_hex8(val: u8) {
    let hex = b"0123456789abcdef";
    let buf = [hex[(val >> 4) as usize], hex[(val & 0xF) as usize]];
    let _ = syscall::write(syscall::Handle::STDOUT, &buf);
}
