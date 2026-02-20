//! MT7988A True Random Number Generator (TRNG)
//!
//! Hardware RNG at base 0x1020f000. Used to seed the kernel's xorshift64
//! PRNG at boot for better entropy than a timer counter alone.
//!
//! Register map:
//!   RNG_CTRL (0x00): bit 0 = EN, bit 31 = READY
//!   RNG_DATA (0x08): 32-bit random output

use super::KERNEL_VIRT_BASE;
use super::clocks;

/// TRNG base address
const TRNG_BASE: usize = 0x1020_F000;

/// Register offsets
mod reg {
    pub const RNG_CTRL: usize = 0x00;
    pub const RNG_DATA: usize = 0x08;
}

/// Control register bits
mod ctrl {
    pub const EN: u32 = 1 << 0;
    pub const READY: u32 = 1 << 31;
}

#[inline]
fn read_reg(offset: usize) -> u32 {
    let addr = (KERNEL_VIRT_BASE as usize | TRNG_BASE) + offset;
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline]
fn write_reg(offset: usize, value: u32) {
    let addr = (KERNEL_VIRT_BASE as usize | TRNG_BASE) + offset;
    unsafe { core::ptr::write_volatile(addr as *mut u32, value); }
}

/// Enable the TRNG. Must be called before read_u32().
fn enable() {
    let val = read_reg(reg::RNG_CTRL);
    write_reg(reg::RNG_CTRL, val | ctrl::EN);
}

/// Disable the TRNG (save power after seeding).
fn disable() {
    let val = read_reg(reg::RNG_CTRL);
    write_reg(reg::RNG_CTRL, val & !ctrl::EN);
}

/// Read a single 32-bit random value.
///
/// Polls READY with a ~60us timeout (30 iterations × 2us).
/// Returns None on timeout.
fn read_u32() -> Option<u32> {
    for _ in 0..30 {
        if (read_reg(reg::RNG_CTRL) & ctrl::READY) != 0 {
            return Some(read_reg(reg::RNG_DATA));
        }
        // ~2us delay (simple spin; no timer needed at boot)
        for _ in 0..200 {
            core::hint::spin_loop();
        }
    }
    None
}

/// Read 4 hardware random words to seed the kernel PRNG.
///
/// Enables the TRNG, reads 4 words, disables it.
/// Returns None if the TRNG fails to produce data (hardware not present
/// or not responding).
pub fn seed_words() -> Option<[u32; 4]> {
    // Enable TRNG peripheral clock before any register access
    clocks::enable_trng_clock();

    enable();

    let mut words = [0u32; 4];
    for w in words.iter_mut() {
        *w = read_u32()?;
    }

    disable();
    Some(words)
}
