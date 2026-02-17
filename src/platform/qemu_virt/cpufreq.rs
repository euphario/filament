//! QEMU virt CPU Frequency Scaling (stub)
//!
//! Virtual CPUs have no real frequency states. Single OPP, no-op set_opp.

use core::sync::atomic::{AtomicU8, Ordering};

const VIRT_FREQ_KHZ: u32 = 2_000_000; // 2 GHz (nominal virtual CPU)

static CURRENT_OPP: AtomicU8 = AtomicU8::new(0);

pub fn init() {}

pub fn opp_count() -> usize { 1 }

pub fn opp_freq_khz(index: usize) -> u32 {
    if index == 0 { VIRT_FREQ_KHZ } else { 0 }
}

pub fn current_opp() -> usize {
    CURRENT_OPP.load(Ordering::Relaxed) as usize
}

pub fn current_freq_khz() -> u32 { VIRT_FREQ_KHZ }

pub fn set_opp(index: usize) {
    if index == 0 {
        CURRENT_OPP.store(0, Ordering::Relaxed);
    }
}
