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

use super::task::{scheduler, TaskState, MAX_TASKS};
use super::liveness::LIVENESS_CHECK_INTERVAL;

/// Maximum time to sleep (in ticks) - ensures watchdog doesn't expire
/// and liveness checks happen periodically
const MAX_SLEEP_TICKS: u64 = LIVENESS_CHECK_INTERVAL;

/// Minimum sleep time (avoid thrashing for very short sleeps)
const MIN_SLEEP_TICKS: u64 = 1;

/// Calculate the next timer deadline
/// Returns the tick count when the timer should fire next
pub fn next_deadline(current_tick: u64) -> u64 {
    let mut soonest = current_tick + MAX_SLEEP_TICKS;

    unsafe {
        let sched = scheduler();

        for slot in 0..MAX_TASKS {
            if let Some(ref task) = sched.tasks[slot] {
                // Check Waiting state deadline (request-response with deadline)
                if let TaskState::Waiting { deadline, .. } = task.state {
                    if deadline < soonest {
                        soonest = deadline;
                    }
                }

                // Check Dying state deadline (cleanup deadline)
                if let TaskState::Dying { until, .. } = task.state {
                    if until < soonest {
                        soonest = until;
                    }
                }

                // Check all task timers
                for timer in &task.timers {
                    if timer.is_active() && timer.deadline < soonest {
                        soonest = timer.deadline;
                    }
                }
            }
        }
    }

    // Ensure we don't return a deadline in the past
    if soonest <= current_tick {
        soonest = current_tick + MIN_SLEEP_TICKS;
    }

    soonest
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
    // This would call into platform/mt7988/timer.rs
    // to reprogram CNTP_TVAL with the calculated value
    //
    // For now, this is a stub - the actual implementation
    // would replace the fixed reload in handle_irq
    let _ = ticks;
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
