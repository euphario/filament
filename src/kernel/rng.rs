//! Kernel PRNG
//!
//! Simple xorshift64 PRNG seeded from the hardware timer counter.
//! Used for ASLR randomization of user task load addresses, stacks, and heaps.

use crate::kernel::lock::{SpinLock, lock_class};

static RNG: SpinLock<u64> = SpinLock::new(lock_class::UNORDERED, 0);

/// Initialize the PRNG with entropy from the hardware timer counter.
/// Must be called after timer init.
pub fn init() {
    // Seed from hardware counter, ensure non-zero (xorshift requires it)
    let seed = crate::platform::current::timer::counter() | 1;
    *RNG.lock() = seed;
}

/// Mix hardware entropy into the PRNG state.
///
/// XORs 4 hardware random words into the state for better entropy
/// than a timer counter alone. Called once at boot if the platform
/// has a hardware RNG.
pub fn seed_from_hardware(words: [u32; 4]) {
    let mut state = RNG.lock();
    let hw = ((words[0] as u64) << 32) | (words[1] as u64);
    let hw2 = ((words[2] as u64) << 32) | (words[3] as u64);
    *state ^= hw;
    *state ^= hw2;
    // Ensure non-zero (xorshift requires it)
    if *state == 0 {
        *state = 0xDEAD_BEEF_CAFE_BABE;
    }
}

/// Generate a pseudo-random u64.
pub fn next_u64() -> u64 {
    let mut state = RNG.lock();
    let mut s = *state;
    s ^= s << 13;
    s ^= s >> 7;
    s ^= s << 17;
    *state = s;
    s
}

/// Generate a random offset aligned to `align`, in range [0, max_offset).
/// `align` must be a power of two.
pub fn random_offset(max_offset: u64, align: u64) -> u64 {
    if max_offset == 0 {
        return 0;
    }
    let n = next_u64() % max_offset;
    n & !(align - 1)
}
