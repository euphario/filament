//! Timer Handle Object
//!
//! Per-handle timer objects that can be waited on.

use super::traits::{Waitable, Closable, WaitResult, WaitFilter, CloseAction};
use crate::kernel::task::TaskId;

/// Timer object for handle system
///
/// Each timer has a deadline and optional interval for recurring timers.
#[derive(Clone, Copy, Debug)]
pub struct TimerObject {
    /// Deadline tick (0 = not armed)
    pub deadline: u64,

    /// Interval for recurring timers (0 = one-shot)
    pub interval: u64,

    /// Task to wake when timer fires (0 = none)
    waker_task: TaskId,
}

impl TimerObject {
    /// Create a new one-shot timer
    pub const fn one_shot(deadline: u64) -> Self {
        Self {
            deadline,
            interval: 0,
            waker_task: 0,
        }
    }

    /// Create a new recurring timer
    pub const fn recurring(deadline: u64, interval: u64) -> Self {
        Self {
            deadline,
            interval,
            waker_task: 0,
        }
    }

    /// Create an unarmed timer
    pub const fn new() -> Self {
        Self {
            deadline: 0,
            interval: 0,
            waker_task: 0,
        }
    }

    /// Arm the timer (set deadline)
    pub fn arm(&mut self, deadline: u64) {
        self.deadline = deadline;
    }

    /// Arm with interval
    pub fn arm_recurring(&mut self, deadline: u64, interval: u64) {
        self.deadline = deadline;
        self.interval = interval;
    }

    /// Disarm the timer
    pub fn disarm(&mut self) {
        self.deadline = 0;
    }

    /// Check if timer is armed
    pub fn is_armed(&self) -> bool {
        self.deadline > 0
    }

    /// Check if timer has expired
    pub fn is_expired(&self, current_tick: u64) -> bool {
        self.deadline > 0 && current_tick >= self.deadline
    }

    /// Rearm for next interval (for recurring timers)
    /// Returns new deadline, or 0 if one-shot
    pub fn rearm(&mut self, current_tick: u64) -> u64 {
        if self.interval > 0 {
            self.deadline = current_tick + self.interval;
            self.deadline
        } else {
            self.deadline = 0;
            0
        }
    }
}

impl Waitable for TimerObject {
    fn poll(&self, filter: WaitFilter) -> Option<WaitResult> {
        match filter {
            WaitFilter::Timer | WaitFilter::Readable => {
                let current_tick = crate::platform::mt7988::timer::ticks();
                if self.is_expired(current_tick) {
                    Some(WaitResult::new(
                        super::Handle::INVALID,
                        WaitFilter::Timer,
                        self.deadline,
                    ))
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn register_waker(&mut self, task_id: TaskId) {
        self.waker_task = task_id;
        // Timer waking is handled by the scheduler's check_timeouts
        // We just store the task ID for reference
    }

    fn unregister_waker(&mut self, _task_id: TaskId) {
        self.waker_task = 0;
    }
}

impl Closable for TimerObject {
    fn close(&self, _owner_pid: u32) -> CloseAction {
        // Timers don't need special cleanup
        CloseAction::None
    }
}

impl Default for TimerObject {
    fn default() -> Self {
        Self::new()
    }
}
