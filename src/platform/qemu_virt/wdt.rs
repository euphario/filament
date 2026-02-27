//! QEMU Software Watchdog
//!
//! No hardware WDT on QEMU virt. Instead, track the last kick time using
//! the ARM generic counter and check from the timer tick handler. If no
//! kick arrives for TIMEOUT_SECONDS, call the soft NMI handler.

use core::sync::atomic::{AtomicU64, AtomicBool, Ordering};

/// Last kick timestamp (raw counter ticks).
static LAST_KICK: AtomicU64 = AtomicU64::new(0);

/// Whether the watchdog is enabled.
static ENABLED: AtomicBool = AtomicBool::new(false);

/// Timeout in seconds before soft NMI fires.
const TIMEOUT_SECONDS: u64 = 15;

/// Read the ARM generic counter (CNTPCT_EL0).
#[inline]
fn counter() -> u64 {
    let cnt: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) cnt);
    }
    cnt
}

/// Read the counter frequency (CNTFRQ_EL0).
#[inline]
fn frequency() -> u64 {
    let freq: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
    }
    freq
}

/// Initialize and enable the software watchdog.
pub fn init() {
    LAST_KICK.store(counter(), Ordering::Release);
    ENABLED.store(true, Ordering::Release);
}

/// Kick the watchdog — call periodically from timer tick.
#[inline]
pub fn kick() {
    LAST_KICK.store(counter(), Ordering::Release);
}

/// Check if the watchdog has timed out. Called from CPU 0's timer tick.
/// Returns true if timeout expired (caller should trigger soft NMI).
pub fn check_timeout() -> bool {
    if !ENABLED.load(Ordering::Acquire) {
        return false;
    }
    let last = LAST_KICK.load(Ordering::Acquire);
    let now = counter();
    let elapsed = now.wrapping_sub(last);
    let threshold = frequency() * TIMEOUT_SECONDS;
    elapsed >= threshold
}
