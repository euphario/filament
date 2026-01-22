//! ARM Generic Timer driver
//!
//! Uses the EL1 Physical Timer (CNTP_*) which generates the timer PPI.
//! Implements the HAL Timer trait for portability.

use crate::kdebug;
use crate::hal::Timer as TimerTrait;
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

            // Update per-CPU tick counts for CPU usage tracking
            let cpu_data = crate::kernel::percpu::cpu_local();
            cpu_data.tick();
            if cpu_data.is_idle() {
                cpu_data.idle_tick();
            }

            // Kick the watchdog to prevent system reset
            super::wdt::kick();


            // Drain klog ring directly to UART
            crate::klog::try_drain(8);

            // Flush UART output buffer (userspace console output)
            // Use try_flush to avoid deadlock if syscall is holding UART lock
            super::uart::try_flush_buffer();

            // Check for timed-out blocked tasks and wake them
            unsafe {
                crate::kernel::task::scheduler().check_timeouts(self.tick_count);
            }

            // Run liveness checks (pings blocked tasks periodically)
            crate::kernel::sched::timer_tick(self.tick_count);

            // Reap terminated tasks (orphan processes that exited)
            // Safe to do here since we're not running on the terminated task's stack
            unsafe {
                crate::kernel::task::scheduler().reap_terminated(self.tick_count);
            }

            // Continue bus initialization (one bus per tick until all Safe)
            // This runs in parallel with devd - devd gets notified as each bus becomes Safe
            crate::kernel::bus::continue_init();

            // Check if liveness system killed devd - trigger recovery
            // (Done here to avoid recursive scheduler access from within liveness check)
            if crate::DEVD_LIVENESS_KILLED.swap(false, core::sync::atomic::Ordering::SeqCst) {
                crate::recover_devd();
            }

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

// ============================================================================
// HAL Timer Implementation
// ============================================================================

impl TimerTrait for Timer {
    fn init(&mut self) {
        self.frequency = Self::read_frequency();
        Self::write_ctl(0);

        // Enable EL0 access to virtual counter
        unsafe {
            core::arch::asm!("msr cntkctl_el1, {}", in(reg) 0x3u64);
            core::arch::asm!("isb");
        }

        gic::enable_irq(irq::TIMER_PPI);
    }

    fn start(&mut self, interval_ms: u64) {
        self.time_slice_ms = interval_ms;
        let ticks = (self.frequency * interval_ms) / 1000;
        Self::write_tval(ticks);
        Self::write_ctl(ctl::ENABLE);
    }

    fn stop(&self) {
        Self::write_ctl(0);
    }

    fn handle_irq(&mut self) -> bool {
        let ctl_val = Self::read_ctl();

        if (ctl_val & ctl::ISTATUS) != 0 {
            self.tick_count += 1;

            super::wdt::kick();
            crate::klog::try_drain(8);
            super::uart::try_flush_buffer();

            unsafe {
                crate::kernel::task::scheduler().check_timeouts(self.tick_count);
                crate::kernel::task::scheduler().reap_terminated(self.tick_count);
            }

            let ticks = (self.frequency * self.time_slice_ms) / 1000;
            Self::write_tval(ticks);
            Self::write_ctl(ctl::ENABLE);

            true
        } else {
            false
        }
    }

    fn ticks(&self) -> u64 {
        self.tick_count
    }

    fn frequency(&self) -> u64 {
        self.frequency
    }

    fn counter(&self) -> u64 {
        Self::read_cntpct()
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
    kdebug!("timer", "info"; freq_hz = freq, freq_mhz = freq / 1_000_000);
}

/// Get a mutable reference to the timer as a Timer trait object
/// # Safety
/// Must only be called after init()
pub fn as_timer() -> &'static mut dyn TimerTrait {
    unsafe { &mut *core::ptr::addr_of_mut!(TIMER) }
}
