//! ARM Generic Timer driver for QEMU virt
//!
//! Uses the EL1 Physical Timer (CNTP_*) - same as real hardware.

use crate::hal::Timer as TimerTrait;
use super::{gic, irq};

mod ctl {
    pub const ENABLE: u64 = 1 << 0;
    pub const IMASK: u64 = 1 << 1;
    pub const ISTATUS: u64 = 1 << 2;
}

pub struct Timer {
    frequency: u64,
    tick_count: u64,
    time_slice_ms: u64,
}

impl Timer {
    pub const fn new() -> Self {
        Self {
            frequency: 0,
            tick_count: 0,
            time_slice_ms: 10,
        }
    }

    pub fn init(&mut self) {
        self.frequency = Self::read_frequency();
        Self::write_ctl(0);

        // Enable EL0 access to virtual counter
        unsafe {
            core::arch::asm!("msr cntkctl_el1, {}", in(reg) 0x3u64);
            core::arch::asm!("isb");
        }

        gic::enable_irq(irq::TIMER_PPI);
    }

    pub fn start(&mut self, interval_ms: u64) {
        self.time_slice_ms = interval_ms;
        let ticks = (self.frequency * interval_ms) / 1000;
        Self::write_tval(ticks);
        Self::write_ctl(ctl::ENABLE);
    }

    pub fn stop(&self) {
        Self::write_ctl(0);
    }

    pub fn handle_irq(&mut self) -> bool {
        let ctl_val = Self::read_ctl();

        if (ctl_val & ctl::ISTATUS) != 0 {
            self.tick_count += 1;

            // Update per-CPU tick counts
            let cpu_data = crate::kernel::percpu::cpu_local();
            cpu_data.tick();
            if cpu_data.is_idle() {
                cpu_data.idle_tick();
            }

            // Drain klog (limit to 2 records per tick to avoid UART saturation)
            // At 115200 baud, UART can only output ~115 bytes per 10ms tick
            // Each formatted log line is ~100-200 bytes with ANSI codes
            crate::klog::try_drain(2);

            // Check timeouts using hardware counter (canonical time source)
            // All deadlines in the system use counter units for consistency
            let current_counter = Self::read_cntpct();
            crate::kernel::sched::timer_tick(current_counter);

            unsafe {
                crate::kernel::task::scheduler().reap_terminated(self.tick_count);
            }

            // Continue bus init (no-op for QEMU without PCIe)
            crate::kernel::bus::continue_init();

            // Check devd liveness
            if crate::DEVD_LIVENESS_KILLED.swap(false, core::sync::atomic::Ordering::SeqCst) {
                crate::recover_devd();
            }

            // Use simple fixed interval (tickless deferred for future work)
            let ticks = (self.frequency * self.time_slice_ms) / 1000;
            Self::write_tval(ticks);
            Self::write_ctl(ctl::ENABLE);

            true
        } else {
            false
        }
    }

    pub fn ticks(&self) -> u64 {
        self.tick_count
    }

    pub fn frequency(&self) -> u64 {
        self.frequency
    }

    pub fn counter(&self) -> u64 {
        Self::read_cntpct()
    }

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

impl TimerTrait for Timer {
    fn init(&mut self) {
        Timer::init(self);
    }

    fn start(&mut self, interval_ms: u64) {
        Timer::start(self, interval_ms);
    }

    fn stop(&self) {
        Timer::stop(self);
    }

    fn handle_irq(&mut self) -> bool {
        Timer::handle_irq(self)
    }

    fn ticks(&self) -> u64 {
        Timer::ticks(self)
    }

    fn frequency(&self) -> u64 {
        Timer::frequency(self)
    }

    fn counter(&self) -> u64 {
        Timer::counter(self)
    }
}

// ============================================================================
// Global Instance and Public API
// ============================================================================

static mut TIMER: Timer = Timer::new();

pub fn init() {
    unsafe { (*core::ptr::addr_of_mut!(TIMER)).init(); }
}

pub fn start(interval_ms: u64) {
    unsafe { (*core::ptr::addr_of_mut!(TIMER)).start(interval_ms); }
}

pub fn handle_irq() -> bool {
    unsafe { (*core::ptr::addr_of_mut!(TIMER)).handle_irq() }
}

pub fn ticks() -> u64 {
    unsafe { (*core::ptr::addr_of!(TIMER)).ticks() }
}

pub fn frequency() -> u64 {
    unsafe { (*core::ptr::addr_of!(TIMER)).frequency() }
}

pub fn counter() -> u64 {
    unsafe { (*core::ptr::addr_of!(TIMER)).counter() }
}

/// Get logical tick count (computed from hardware counter, not IRQ-incremented)
/// More reliable for storm protection and rate limiting.
pub fn logical_ticks() -> u64 {
    unsafe { (*core::ptr::addr_of!(TIMER)).logical_ticks() }
}

// ============================================================================
// Unified Clock API
// ============================================================================

/// Get current time in nanoseconds
pub fn now_ns() -> u64 {
    unsafe { (*core::ptr::addr_of!(TIMER)).now_ns() }
}

/// Create a deadline N nanoseconds from now
pub fn deadline_ns(duration_ns: u64) -> u64 {
    unsafe { (*core::ptr::addr_of!(TIMER)).deadline_ns(duration_ns) }
}

/// Check if a deadline has expired
pub fn is_expired(deadline: u64) -> bool {
    unsafe { (*core::ptr::addr_of!(TIMER)).is_expired(deadline) }
}

pub fn as_timer() -> &'static mut dyn TimerTrait {
    unsafe { &mut *core::ptr::addr_of_mut!(TIMER) }
}

/// Print timer info (frequency, etc.)
pub fn print_info() {
    let freq = frequency();
    crate::kinfo!("timer", "info"; freq_hz = freq);
}
