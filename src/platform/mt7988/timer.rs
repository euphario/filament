//! ARM Generic Timer driver
//!
//! Uses the EL1 Physical Timer (CNTP_*) which generates the timer PPI.

use crate::logln;
use super::{gic, irq};

/// Timer control bits
mod ctl {
    pub const ENABLE: u64 = 1 << 0;   // Timer enabled
    pub const IMASK: u64 = 1 << 1;    // Interrupt mask (1 = masked)
    pub const ISTATUS: u64 = 1 << 2;  // Interrupt status (read-only)
}

pub struct Timer {
    frequency: u64,
    tick_count: u64,
    /// Current time slice in milliseconds
    time_slice_ms: u64,
}

impl Timer {
    pub const fn new() -> Self {
        Self {
            frequency: 0,
            tick_count: 0,
            time_slice_ms: 10, // Default 10ms time slice
        }
    }

    /// Initialize the timer
    pub fn init(&mut self) {
        // Read counter frequency
        self.frequency = Self::read_frequency();

        // Disable timer while configuring
        Self::write_ctl(0);

        // Enable EL0 access to virtual counter (CNTVCT_EL0)
        // CNTKCTL_EL1: bit 1 = EL0VCTEN (enable virtual counter for EL0)
        //              bit 0 = EL0PCTEN (enable physical counter for EL0)
        unsafe {
            core::arch::asm!("msr cntkctl_el1, {}", in(reg) 0x3u64);
            core::arch::asm!("isb");
        }

        // Enable timer interrupt in GIC
        gic::enable_irq(irq::TIMER_PPI);
    }

    /// Start the timer with an interval in milliseconds
    pub fn start(&mut self, interval_ms: u64) {
        self.time_slice_ms = interval_ms;

        // Calculate ticks for the interval
        let ticks = (self.frequency * interval_ms) / 1000;

        // Set timer value (countdown)
        Self::write_tval(ticks);

        // Enable timer, unmask interrupt
        Self::write_ctl(ctl::ENABLE);
    }

    /// Stop the timer
    pub fn stop(&self) {
        Self::write_ctl(0);
    }

    /// Handle timer interrupt - returns true if it was a timer interrupt
    pub fn handle_irq(&mut self) -> bool {
        let ctl = Self::read_ctl();

        if (ctl & ctl::ISTATUS) != 0 {
            self.tick_count += 1;

            // Reload timer for next time slice
            let ticks = (self.frequency * self.time_slice_ms) / 1000;
            Self::write_tval(ticks);

            // Re-enable (clearing ISTATUS implicitly)
            Self::write_ctl(ctl::ENABLE);

            true
        } else {
            false
        }
    }

    /// Get current tick count
    pub fn ticks(&self) -> u64 {
        self.tick_count
    }

    /// Get timer frequency in Hz
    pub fn frequency(&self) -> u64 {
        self.frequency
    }

    /// Read current counter value
    pub fn counter(&self) -> u64 {
        Self::read_cntpct()
    }

    // System register access

    fn read_frequency() -> u64 {
        let freq: u64;
        unsafe {
            core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
        }
        freq
    }

    fn read_ctl() -> u64 {
        let ctl: u64;
        unsafe {
            core::arch::asm!("mrs {}, cntp_ctl_el0", out(reg) ctl);
        }
        ctl
    }

    fn write_ctl(val: u64) {
        unsafe {
            core::arch::asm!("msr cntp_ctl_el0, {}", in(reg) val);
            core::arch::asm!("isb");
        }
    }

    fn write_tval(val: u64) {
        unsafe {
            core::arch::asm!("msr cntp_tval_el0, {}", in(reg) val);
        }
    }

    fn read_cntpct() -> u64 {
        let cnt: u64;
        unsafe {
            core::arch::asm!("mrs {}, cntpct_el0", out(reg) cnt);
        }
        cnt
    }
}

/// Global timer instance
static mut TIMER: Timer = Timer::new();

/// Initialize the global timer
pub fn init() {
    unsafe {
        (*core::ptr::addr_of_mut!(TIMER)).init();
    }
}

/// Start timer with interval in ms
pub fn start(interval_ms: u64) {
    unsafe {
        (*core::ptr::addr_of_mut!(TIMER)).start(interval_ms);
    }
}

/// Handle timer IRQ - call from IRQ handler
pub fn handle_irq() -> bool {
    unsafe { (*core::ptr::addr_of_mut!(TIMER)).handle_irq() }
}

/// Get tick count
pub fn ticks() -> u64 {
    unsafe { (*core::ptr::addr_of!(TIMER)).ticks() }
}

/// Get frequency
pub fn frequency() -> u64 {
    unsafe { (*core::ptr::addr_of!(TIMER)).frequency() }
}

/// Get raw counter value
pub fn counter() -> u64 {
    unsafe { (*core::ptr::addr_of!(TIMER)).counter() }
}

/// Print timer info
pub fn print_info() {
    let freq = frequency();
    logln!("  Frequency: {} Hz ({} MHz)", freq, freq / 1_000_000);
}
