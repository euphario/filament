//! MediaTek MT7988A Watchdog Timer (WDT) Driver
//!
//! The TOPRGU (Top Reset Generation Unit) provides:
//! - Hardware watchdog that resets the system if not kicked
//! - Software reset capability
//!
//! Default timeout is ~30 seconds. Call kick() periodically to prevent reset.

use super::KERNEL_VIRT_BASE;

/// TOPRGU base address
const WDT_BASE: usize = 0x1001_C000;

/// WDT register offsets
mod reg {
    /// Watchdog mode register
    pub const WDT_MODE: usize = 0x00;
    /// Watchdog length (timeout) register
    pub const WDT_LENGTH: usize = 0x04;
    /// Watchdog restart (kick) register
    pub const WDT_RESTART: usize = 0x08;
    /// Watchdog status register
    pub const WDT_STATUS: usize = 0x0C;
    /// Watchdog software reset register
    pub const WDT_SWRST: usize = 0x14;
}

/// WDT mode bits
mod mode {
    /// Enable watchdog timer
    pub const ENABLE: u32 = 1 << 0;
    /// External reset enable
    pub const EXTEN: u32 = 1 << 2;
    /// External reset polarity
    pub const EXT_POL: u32 = 1 << 3;
    /// IRQ enable (instead of reset)
    pub const IRQ: u32 = 1 << 3;
    /// Dual mode (IRQ then reset)
    pub const DUAL_MODE: u32 = 1 << 6;
    /// Key to unlock register writes
    pub const KEY: u32 = 0x22 << 8;
}

/// WDT restart key to kick the watchdog
const WDT_RESTART_KEY: u32 = 0x1971;

/// Write to a WDT register
#[inline]
fn write_reg(offset: usize, value: u32) {
    let addr = (KERNEL_VIRT_BASE as usize | WDT_BASE) + offset;
    unsafe {
        core::ptr::write_volatile(addr as *mut u32, value);
    }
}

/// Read a WDT register
#[inline]
fn read_reg(offset: usize) -> u32 {
    let addr = (KERNEL_VIRT_BASE as usize | WDT_BASE) + offset;
    unsafe {
        core::ptr::read_volatile(addr as *const u32)
    }
}

/// Initialize watchdog with default timeout (~30 seconds)
/// Does NOT enable the watchdog - call enable() to start it.
pub fn init() {
    // Set default timeout (0x0800 = ~30 seconds at 32KHz)
    write_reg(reg::WDT_LENGTH, 0x0800_0000 | 0x0008);
}

/// Enable the watchdog timer
pub fn enable() {
    let mode = mode::ENABLE | mode::EXTEN | mode::KEY;
    write_reg(reg::WDT_MODE, mode);
}

/// Disable the watchdog timer
pub fn disable() {
    write_reg(reg::WDT_MODE, mode::KEY); // KEY with no ENABLE clears it
}

/// Kick (feed) the watchdog to prevent reset
/// Call this periodically from the timer IRQ or main loop.
#[inline]
pub fn kick() {
    write_reg(reg::WDT_RESTART, WDT_RESTART_KEY);
}

/// Check if the last reset was caused by watchdog
pub fn was_wdt_reset() -> bool {
    let status = read_reg(reg::WDT_STATUS);
    status != 0
}

/// Clear the watchdog reset status
pub fn clear_status() {
    write_reg(reg::WDT_STATUS, 0);
}

/// Trigger an immediate software reset
pub fn software_reset() -> ! {
    write_reg(reg::WDT_SWRST, 0x1209);
    loop {
        core::hint::spin_loop();
    }
}

/// Set watchdog timeout in seconds (approximate)
/// Valid range: 1-512 seconds
pub fn set_timeout(seconds: u32) {
    let seconds = seconds.clamp(1, 512);
    // Each tick is ~1/32768 seconds, timeout = (value + 1) * 512 / 32768
    // Simplified: ticks = seconds * 64
    let ticks = (seconds * 64) as u32;
    let length = (ticks << 5) | 0x0008; // 0x0008 is the unlock key
    write_reg(reg::WDT_LENGTH, length);
}
