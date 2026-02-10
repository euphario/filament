//! Process Liveness Checker
//!
//! Detects and recovers from stuck processes using two strategies:
//!
//! ## Sleeping vs Waiting
//!
//! The system distinguishes between two blocked states:
//!
//! **Sleeping** (TaskState::Sleeping):
//! - Event-loop processes waiting for ANY event (no deadline)
//! - Examples: devd waiting for child exits, consoled waiting for input
//! - Strategy: Periodic probing (ping/pong) to verify responsiveness
//!
//! **Waiting** (TaskState::Waiting):
//! - Specific request awaiting response (HAS deadline)
//! - Examples: IPC call with timeout, waitpid with timeout
//! - Strategy: No probing needed - just enforce deadline
//!   (scheduler's check_timeouts wakes them; if deadline passes, request failed)
//!
//! ## Probe Mechanism (for Sleeping)
//!
//! For sleeping tasks:
//! - After PING_INTERVAL_NS of idle, probe the task
//! - For EventLoop sleep: wake the task (implicit pong via next syscall)
//! - For IPC sleep: send MSG_TYPE_PING message to channel
//! - If no response after PING_TIMEOUT_NS → escalate
//!
//! ## Timeouts
//!
//! - PING_INTERVAL: How often to probe sleeping processes (30s)
//! - PING_TIMEOUT: How long to wait for response before escalation (10s)
//! - KILL_TIMEOUT: How long after escalation before killing (10s)
//!
//! ## Future Work
//!
//! - Add liveness checking for shared memory rings (ByteRing, BlockRing)
//! - Add waiting channel and liveness status to ps output

use super::ipc::{Message, MessageHeader, MessageType, MAX_INLINE_PAYLOAD, waker, WakeReason};
use super::task::SleepReason;

/// How often to check liveness (in timer IRQs, ~100/sec)
/// Check every ~1 second for quicker detection
pub const LIVENESS_CHECK_INTERVAL: u64 = 100; // Every ~1 second

/// How long a process can be idle-waiting before we ping it (in nanoseconds)
pub const PING_INTERVAL_NS: u64 = 10 * 1_000_000_000; // 10 seconds

/// How long to wait for pong response (in nanoseconds)
pub const PING_TIMEOUT_NS: u64 = 10 * 1_000_000_000; // 10 seconds

/// How long after closing channel before killing (in nanoseconds)
pub const KILL_TIMEOUT_NS: u64 = 10 * 1_000_000_000; // 10 seconds

/// Convert hardware counter value to nanoseconds
fn counter_to_ns(counter: u64) -> u64 {
    let freq = crate::platform::current::timer::frequency();
    if freq == 0 { return 0; }
    // counter * 1_000_000_000 / freq, avoiding overflow
    ((counter as u128 * 1_000_000_000) / freq as u128) as u64
}

/// Convert hardware counter value to milliseconds (for logging)
fn counter_to_ms(counter: u64) -> u64 {
    let freq = crate::platform::current::timer::frequency();
    if freq == 0 { return 0; }
    // counter * 1000 / freq
    ((counter as u128 * 1000) / freq as u128) as u64
}

/// Special message type for liveness ping (kernel-internal)
pub const MSG_TYPE_PING: u32 = 0xFFFE;
/// Special message type for liveness pong (kernel-internal)
pub const MSG_TYPE_PONG: u32 = 0xFFFF;

/// Liveness state for a task
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LivenessState {
    /// Process is healthy or not being tracked
    Normal,
    /// Ping sent, waiting for response
    /// (channel_id, sent_at_tick)
    PingSent { channel: u32, sent_at: u64 },
    /// Channel closed, waiting to see if process recovers
    /// (channel_id, closed_at_tick)
    ClosePending { channel: u32, closed_at: u64 },
}

impl Default for LivenessState {
    fn default() -> Self {
        LivenessState::Normal
    }
}

impl LivenessState {
    /// State as numeric code for logging
    fn code(&self) -> u8 {
        match self {
            LivenessState::Normal => 0,
            LivenessState::PingSent { .. } => 1,
            LivenessState::ClosePending { .. } => 2,
        }
    }

    /// Check if transition from self to new_state is valid
    fn can_transition_to(&self, new_state: &LivenessState) -> bool {
        use LivenessState::*;
        match (self, new_state) {
            // Any state can reset to Normal (task activity/response)
            (_, Normal) => true,
            // Normal → PingSent (ping sent or implicit wake)
            (Normal, PingSent { .. }) => true,
            // PingSent → ClosePending (timeout, escalate)
            (PingSent { .. }, ClosePending { .. }) => true,
            // All other transitions are invalid
            _ => false,
        }
    }

    /// Attempt to transition to new state, returns Ok if valid, Err if invalid
    /// This is the ONLY way state transitions should occur (except reset to Normal)
    pub fn transition_to(
        &mut self,
        new_state: LivenessState,
        pid: u32,
    ) -> Result<(), LivenessTransitionError> {
        if !self.can_transition_to(&new_state) {
            crate::kwarn!("liveness", "invalid_transition";
                pid = pid as u64,
                from = self.code() as u64,
                to = new_state.code() as u64
            );
            return Err(LivenessTransitionError::InvalidTransition {
                from: *self,
                to: new_state,
            });
        }

        // Log the transition
        let from_code = self.code();
        let to_code = new_state.code();

        // Only log non-trivial transitions (skip Normal→Normal)
        if from_code != to_code {
            match &new_state {
                LivenessState::Normal => {
                    // State reset is frequent (every syscall), don't log
                }
                LivenessState::PingSent { channel, sent_at } => {
                    crate::kdebug!("liveness", "ping_sent";
                        pid = pid as u64,
                        channel = *channel as u64,
                        ms = counter_to_ms(*sent_at)
                    );
                }
                LivenessState::ClosePending { channel, closed_at } => {
                    crate::kinfo!("liveness", "close_pending";
                        pid = pid as u64,
                        channel = *channel as u64,
                        ms = counter_to_ms(*closed_at)
                    );
                }
            }
        }

        *self = new_state;
        Ok(())
    }

    /// Force reset to Normal state (always valid, used for task activity)
    pub fn reset(&mut self, pid: u32) {
        if *self != LivenessState::Normal {
            // Only log reset from problematic states (PingSent, ClosePending)
            // Skip logging for frequent resets
            let _ = pid; // silence unused warning
            *self = LivenessState::Normal;
        }
    }
}

/// Error type for invalid state transitions
#[derive(Debug, Clone, Copy)]
pub enum LivenessTransitionError {
    InvalidTransition {
        from: LivenessState,
        to: LivenessState,
    },
}

/// Check liveness of all waiting tasks
/// Called periodically from timer tick (not every tick, use counter)
///
/// Returns number of actions taken (pings sent, channels closed, tasks killed)
///
/// IMPORTANT: This runs in IRQ context. Must NOT acquire ObjectService locks
/// (the interrupted task may hold one → deadlock). Killed tasks go through the
/// normal Phase 1 → Phase 2 cleanup pipeline which runs in task context and
/// handles parent notification safely.
pub fn check_liveness(current_tick: u64) -> usize {
    // Execute liveness check with scheduler lock
    super::task::with_scheduler(|sched| {
        let mut actions = 0usize;

        // Collect slots that need to be woken (can't call wake_task during iteration)
        // Must be >= max sleeping EventLoop tasks; 8 was too small for 9+ sleepers,
        // causing the last task in iteration order to get PingSent but never woken.
        let mut wake_slots: [Option<usize>; 16] = [None; 16];
        let mut wake_count = 0usize;

        for (slot, task_opt) in sched.iter_tasks_mut() {
            if let Some(task) = task_opt {
                // Reset liveness state for non-blocked tasks
                if !task.is_blocked() {
                    task.liveness_state.reset(task.id);
                    continue;
                }

                // WAITING tasks have deadlines - no probing needed
                // Scheduler's check_timeouts handles deadline enforcement
                if task.state().is_waiting() {
                    // Just reset liveness - deadline is the enforcement
                    task.liveness_state.reset(task.id);
                    continue;
                }

                // Only probe SLEEPING tasks (event loop processes)
                if !task.state().is_sleeping() {
                    continue;
                }

                let sleep_reason = task.state().sleep_reason();

                // Convert counter values to nanoseconds for comparison
                let current_ns = counter_to_ns(current_tick);

                match task.liveness_state {
                    LivenessState::Normal => {
                        // Check if task has been sleeping too long
                        let wait_started = task.last_activity_tick;
                        let wait_started_ns = counter_to_ns(wait_started);
                        if wait_started > 0 && current_ns > wait_started_ns + PING_INTERVAL_NS {
                            // For EventLoop sleep (handle_wait), use implicit pong:
                            // Just wake the task - its next syscall proves it's alive
                            if matches!(sleep_reason, Some(SleepReason::EventLoop)) {
                                // Mark slot to wake after iteration (can't call wake_task during iteration)
                                // Set liveness state now while we have the task reference
                                let new_state = LivenessState::PingSent {
                                    channel: 0, // No channel - implicit pong via syscall
                                    sent_at: current_tick,
                                };
                                let _ = task.liveness_state.transition_to(new_state, task.id);
                                if wake_count < wake_slots.len() {
                                    wake_slots[wake_count] = Some(slot);
                                    wake_count += 1;
                                }
                                actions += 1;
                                crate::kdebug!("live", "wake"; pid = task.id as u64);
                            } else if matches!(sleep_reason, Some(SleepReason::Irq)) {
                                // For IRQ sleep, send ping message via channel if available
                                if let Some(channel) = find_wait_channel(task) {
                                    if send_ping(channel, task.id) {
                                        let new_state = LivenessState::PingSent {
                                            channel,
                                            sent_at: current_tick,
                                        };
                                        let _ = task.liveness_state.transition_to(new_state, task.id);
                                        actions += 1;
                                        crate::kdebug!("live", "ping"; pid = task.id as u64, ch = channel as u64);
                                    } else {
                                        crate::kwarn!("live", "ping_fail"; pid = task.id as u64, ch = channel as u64);
                                    }
                                } else {
                                    crate::kwarn!("live", "no_channel"; pid = task.id as u64);
                                }
                            }
                        }
                    }

                    LivenessState::PingSent { channel, sent_at } => {
                        // Check if ping timed out
                        let sent_at_ns = counter_to_ns(sent_at);
                        if current_ns > sent_at_ns + PING_TIMEOUT_NS {
                            if channel == 0 {
                                // Implicit pong case (handle_wait) - no channel to close
                                // Task was woken but didn't call any syscall - skip to kill
                                let new_state = LivenessState::ClosePending {
                                    channel: 0,
                                    closed_at: current_tick,
                                };
                                let _ = task.liveness_state.transition_to(new_state, task.id);
                                actions += 1;
                                crate::kwarn!("live", "unresponsive"; pid = task.id as u64);
                            } else {
                                // Channel-based ping - close the channel
                                close_channel_for_liveness(channel);
                                let new_state = LivenessState::ClosePending {
                                    channel,
                                    closed_at: current_tick,
                                };
                                let _ = task.liveness_state.transition_to(new_state, task.id);
                                actions += 1;
                                crate::kwarn!("live", "close_channel"; pid = task.id as u64, ch = channel as u64);
                            }
                        }
                    }

                    LivenessState::ClosePending { channel: _, closed_at } => {
                        // Check if task recovered (should have woken up from channel close)
                        // If still blocked after timeout, kill it
                        let closed_at_ns = counter_to_ns(closed_at);
                        if current_ns > closed_at_ns + KILL_TIMEOUT_NS {
                            // Task didn't recover - kill it
                            let pid = task.id;
                            let is_init = task.is_init;
                            crate::kerror!("live", "kill"; pid = pid as u64, init = is_init);

                            // Mark for termination (actual kill happens in scheduler)
                            crate::transition_or_evict!(task, set_exiting, -9); // SIGKILL equivalent
                            // Reset liveness state (transition logging handled by reset())
                            task.liveness_state.reset(pid);
                            actions += 1;

                            // Special case: if this is devd (init), trigger full recovery
                            if is_init {
                                // Defer recovery to avoid recursive scheduler access
                                // The timer tick handler will check and trigger recovery
                                crate::DEVD_LIVENESS_KILLED.store(true, core::sync::atomic::Ordering::SeqCst);
                            }
                            // Parent notification handled by Phase 2 microtask
                            // (NotifyParentExit) — NOT here, because we're in IRQ
                            // context and can't safely acquire ObjectService locks.
                        }
                    }
                }
            }
        }

        // Now wake the collected slots using the proper wake mechanism
        // This ensures state transition, kernel_stack_owner cleared, and task added to ready queue
        for i in 0..wake_count {
            if let Some(slot) = wake_slots[i] {
                // Check if still blocked, transition state, clear kernel_stack_owner
                let should_notify = if let Some(task) = sched.task_mut(slot) {
                    if task.is_blocked() {
                        // Transition state: Sleeping -> Ready
                        crate::transition_or_log!(task, wake);
                        // Clear kernel_stack_owner so task is selectable
                        task.clear_kernel_stack_owner();
                        true
                    } else {
                        false
                    }
                } else {
                    false
                };
                // Notify scheduler (handles set_on_runq and policy.on_task_ready)
                if should_notify {
                    sched.notify_ready(slot);
                }
            }
        }
        // Set need_resched if any tasks were woken
        if wake_count > 0 {
            crate::kernel::arch::sync::cpu_flags().set_need_resched();
        }

        actions
    })
}


/// Find the channel a task is waiting on
fn find_wait_channel(_task: &super::task::Task) -> Option<u32> {
    // With the unified object system, tasks block via Mux which subscribes
    // to channels through the WaitQueue mechanism. Liveness checking doesn't
    // need to know the specific channel — it just checks if the task is blocked.
    None
}

/// Send a ping message to a channel
fn send_ping(channel_id: u32, _target_pid: u32) -> bool {
    // Create ping message using ipc types
    // MSG_TYPE_PING (0xFFFE) maps to a custom message type
    let ping = Message {
        header: MessageHeader {
            msg_type: MessageType::Data, // Use Data type, check payload for ping marker
            sender: 0, // Kernel
            msg_id: MSG_TYPE_PING, // Store ping marker in msg_id
            payload_len: 0,
            flags: 0,
        },
        payload: [0; MAX_INLINE_PAYLOAD],
    };

    // Send directly to the channel (bypass ownership for kernel liveness probe)
    match super::ipc::send_direct(channel_id, ping) {
        Ok(peer) => {
            let wake_list = super::object_service::object_service()
                .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::READABLE);
            waker::wake(&wake_list, WakeReason::Readable);
            true
        }
        Err(_) => false,
    }
}

/// Close a channel for liveness recovery
fn close_channel_for_liveness(channel_id: u32) {
    // Close channel without ownership check (kernel recovery action)
    match super::ipc::close_unchecked(channel_id) {
        Ok(Some(peer)) => {
            let wake_list = super::object_service::object_service()
                .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::CLOSED);
            waker::wake(&wake_list, WakeReason::Closed);
        }
        Ok(None) | Err(_) => {
            // Channel may already be closed or had no peer
        }
    }
}

/// Called when a task makes a syscall - reset liveness tracking
/// This proves the task is alive and responsive
pub fn task_activity(task: &mut super::task::Task, current_tick: u64) {
    task.last_activity_tick = current_tick;

    // If we were tracking this task, it responded - reset to Normal
    task.liveness_state.reset(task.id);
}

/// Check if a message is a liveness ping and auto-respond
/// Returns true if it was a ping (caller should discard it)
pub fn handle_ping_message(msg: &Message, receiver_channel: u32) -> bool {
    // Check msg_id for ping/pong marker (we use msg_id since MessageType is an enum)
    let msg_id = msg.header.msg_id;

    if msg_id == MSG_TYPE_PING {
        // Auto-respond with pong
        let pong = Message {
            header: MessageHeader {
                msg_type: MessageType::Data,
                sender: 0,
                msg_id: MSG_TYPE_PONG, // Pong marker
                payload_len: 0,
                flags: 0,
            },
            payload: [0; MAX_INLINE_PAYLOAD],
        };

        // Send pong back (to peer channel)
        if let Some(peer_id) = super::ipc::get_peer_id(receiver_channel) {
            // Send directly to peer
            if let Ok(peer) = super::ipc::send_direct(peer_id, pong) {
                let wake_list = super::object_service::object_service()
                    .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::READABLE);
                waker::wake(&wake_list, WakeReason::Readable);
            }
        }

        return true; // Consumed the ping
    }

    if msg_id == MSG_TYPE_PONG {
        // Discard pong messages - they're just acknowledgments
        return true;
    }

    false // Normal message, don't consume
}

/// Liveness checker test
pub fn test() {
    use crate::print_direct;
    print_direct!("  Testing liveness checker...\n");

    // Basic state machine test
    let mut state = LivenessState::default();
    assert_eq!(state, LivenessState::Normal);

    // Test valid transitions
    // Normal → PingSent (valid)
    let result = state.transition_to(
        LivenessState::PingSent { channel: 1, sent_at: 100 },
        999, // test pid
    );
    assert!(result.is_ok());
    assert!(matches!(state, LivenessState::PingSent { channel: 1, sent_at: 100 }));

    // PingSent → ClosePending (valid)
    let result = state.transition_to(
        LivenessState::ClosePending { channel: 1, closed_at: 200 },
        999,
    );
    assert!(result.is_ok());
    assert!(matches!(state, LivenessState::ClosePending { channel: 1, closed_at: 200 }));

    // ClosePending → Normal (valid - any state can reset to Normal)
    let result = state.transition_to(LivenessState::Normal, 999);
    assert!(result.is_ok());
    assert_eq!(state, LivenessState::Normal);

    // Test invalid transitions
    // Normal → ClosePending (invalid - must go through PingSent)
    let result = state.transition_to(
        LivenessState::ClosePending { channel: 1, closed_at: 300 },
        999,
    );
    assert!(result.is_err());
    assert_eq!(state, LivenessState::Normal); // State unchanged

    // PingSent → PingSent (invalid - can't ping again)
    state = LivenessState::PingSent { channel: 1, sent_at: 100 };
    let result = state.transition_to(
        LivenessState::PingSent { channel: 2, sent_at: 200 },
        999,
    );
    assert!(result.is_err());

    // Test reset() always works
    state = LivenessState::ClosePending { channel: 5, closed_at: 500 };
    state.reset(999);
    assert_eq!(state, LivenessState::Normal);

    print_direct!("    [OK] Liveness state machine test passed\n");
}
