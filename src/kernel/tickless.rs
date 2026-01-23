//! Tickless Timer Support
//!
//! Instead of a fixed periodic timer, dynamically calculate
//! the next needed wakeup time and sleep until then.
//!
//! Benefits:
//! - CPU can sleep for long periods when idle
//! - Reduces power consumption significantly
//! - Timer only fires when actually needed
//!
//! Deadlines to track:
//! - Task timers (kevent timers with deadlines)
//! - Task timeouts (blocking syscalls with timeout)
//! - Liveness checks (periodic probing of blocked tasks)
//! - Watchdog (must kick periodically)

use super::task::scheduler;
use super::liveness::LIVENESS_CHECK_INTERVAL;

/// Maximum time to sleep (in ticks) - ensures watchdog doesn't expire
/// and liveness checks happen periodically
const MAX_SLEEP_TICKS: u64 = LIVENESS_CHECK_INTERVAL;

/// Minimum sleep time (avoid thrashing for very short sleeps)
const MIN_SLEEP_TICKS: u64 = 1;

/// Calculate the next timer deadline
/// Returns the tick count when the timer should fire next
pub fn next_deadline(current_tick: u64) -> u64 {
    // Get the scheduler's tracked next deadline
    let sched_deadline = unsafe {
        scheduler().get_next_deadline()
    };

    // Use scheduler's deadline if it's valid and sooner
    let soonest = if sched_deadline > current_tick && sched_deadline < current_tick + MAX_SLEEP_TICKS {
        sched_deadline
    } else {
        current_tick + MAX_SLEEP_TICKS
    };

    // Ensure we don't return a deadline in the past
    if soonest <= current_tick {
        current_tick + MIN_SLEEP_TICKS
    } else {
        soonest
    }
}

/// Calculate how many ticks until the next deadline
pub fn ticks_until_next(current_tick: u64) -> u64 {
    let deadline = next_deadline(current_tick);
    deadline.saturating_sub(current_tick).max(MIN_SLEEP_TICKS)
}

/// Check if we should use tickless mode
/// Returns false if there's a reason to maintain periodic ticks
pub fn can_go_tickless() -> bool {
    // For now, always allow tickless
    // Could add conditions like:
    // - Don't go tickless during bus init
    // - Don't go tickless if there are running tasks
    true
}

// =============================================================================
// Timer Reprogramming
// =============================================================================

/// Reprogram the timer for the next deadline
/// Called after processing timer events to set up the next wakeup
pub fn reprogram_timer(current_tick: u64) {
    if !can_go_tickless() {
        // Fall back to periodic mode
        return;
    }

    let sleep_ticks = ticks_until_next(current_tick);

    // Convert ticks to timer countdown value
    // The platform timer will use this to set CNTP_TVAL
    set_next_timer(sleep_ticks);
}

/// Platform-specific: set the timer to fire after `ticks` timer ticks
fn set_next_timer(ticks: u64) {
    // Convert scheduler ticks (10ms each) to timer ticks
    let freq = crate::platform::current::timer::frequency();
    // Each scheduler tick is 10ms, so multiply by 10 and convert to timer ticks
    let timer_ticks = (ticks * freq * 10) / 1000;

    // Clamp to reasonable bounds
    let timer_ticks = timer_ticks.max(freq / 100).min(freq * 10); // 10ms to 10s

    // Program the timer
    unsafe {
        core::arch::asm!("msr cntp_tval_el0, {}", in(reg) timer_ticks);
        core::arch::asm!("msr cntp_ctl_el0, {}", in(reg) 1u64); // Enable
        core::arch::asm!("isb");
    }

    // Track stats
    unsafe {
        STATS.tickless_entries += 1;
        if ticks > STATS.longest_sleep {
            STATS.longest_sleep = ticks;
        }
    }
}

// =============================================================================
// Statistics
// =============================================================================

/// Track tickless statistics for debugging
pub struct TicklessStats {
    /// Number of times we went tickless
    pub tickless_entries: u64,
    /// Total ticks saved by sleeping longer
    pub ticks_saved: u64,
    /// Longest single sleep (in ticks)
    pub longest_sleep: u64,
}

static mut STATS: TicklessStats = TicklessStats {
    tickless_entries: 0,
    ticks_saved: 0,
    longest_sleep: 0,
};

/// Get tickless statistics
pub fn stats() -> &'static TicklessStats {
    unsafe { &STATS }
}
