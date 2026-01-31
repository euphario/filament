//! Task Eviction
//!
//! Kernel-initiated forced termination of tasks when state corruption
//! or violations are detected. Provides production-quality error handling
//! for scheduler state machine failures.
//!
//! # Design
//!
//! Eviction is a two-phase process:
//! 1. **Mark**: Task is marked for eviction (safe from any context, including IRQ)
//! 2. **Process**: Pending evictions are processed at safe points
//!
//! This separation ensures eviction can be triggered from interrupt context
//! without risking deadlocks or reentrancy issues.
//!
//! # Usage
//!
//! ```ignore
//! // From syscall/scheduler code when transition fails:
//! eviction::mark_for_eviction(task_id, EvictionReason::InvalidStateTransition);
//!
//! // At safe points (syscall exit, timer tick):
//! eviction::process_pending_evictions();
//! ```

use super::state::EvictionReason;
use super::TaskId;
use crate::kerror;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

/// Maximum number of pending evictions
const MAX_PENDING_EVICTIONS: usize = 8;

/// A pending eviction record
#[repr(C)]
struct PendingEviction {
    /// Task ID to evict (0 = slot empty)
    task_id: AtomicU32,
    /// Eviction reason (as u8 for atomic access)
    reason: AtomicU8,
}

impl PendingEviction {
    const fn new() -> Self {
        Self {
            task_id: AtomicU32::new(0),
            reason: AtomicU8::new(0),
        }
    }
}

/// Global pending eviction queue
static PENDING_EVICTIONS: [PendingEviction; MAX_PENDING_EVICTIONS] = [
    PendingEviction::new(),
    PendingEviction::new(),
    PendingEviction::new(),
    PendingEviction::new(),
    PendingEviction::new(),
    PendingEviction::new(),
    PendingEviction::new(),
    PendingEviction::new(),
];

/// Convert EvictionReason to u8 for atomic storage
fn reason_to_u8(reason: EvictionReason) -> u8 {
    match reason {
        EvictionReason::InvalidStateTransition => 1,
        EvictionReason::StateCorruption => 2,
        EvictionReason::SyscallPanic => 3,
        EvictionReason::LivenessTimeout => 4,
        EvictionReason::ResourceExhaustion => 5,
    }
}

/// Convert u8 back to EvictionReason
fn u8_to_reason(val: u8) -> Option<EvictionReason> {
    match val {
        1 => Some(EvictionReason::InvalidStateTransition),
        2 => Some(EvictionReason::StateCorruption),
        3 => Some(EvictionReason::SyscallPanic),
        4 => Some(EvictionReason::LivenessTimeout),
        5 => Some(EvictionReason::ResourceExhaustion),
        _ => None,
    }
}

/// Mark a task for eviction.
///
/// Safe to call from any context including IRQ handlers.
/// The actual eviction happens later when `process_pending_evictions()` is called.
///
/// If the eviction queue is full, logs an error but doesn't panic.
pub fn mark_for_eviction(task_id: TaskId, reason: EvictionReason) {
    let reason_val = reason_to_u8(reason);

    // Find an empty slot
    for slot in PENDING_EVICTIONS.iter() {
        // Try to claim this slot with compare-exchange
        // If task_id is 0, slot is empty
        if slot.task_id.compare_exchange(
            0,
            task_id,
            Ordering::SeqCst,
            Ordering::SeqCst,
        ).is_ok() {
            // Successfully claimed slot, store reason
            slot.reason.store(reason_val, Ordering::SeqCst);
            kerror!("evict", "marked";
                pid = task_id as u64,
                reason = reason_val as u64
            );
            return;
        }
    }

    // Queue full - log but don't panic
    kerror!("evict", "queue_full";
        pid = task_id as u64,
        reason = reason_val as u64
    );
}

/// Process all pending evictions.
///
/// Must be called from a safe context (scheduler lock not held).
/// Typically called at syscall exit or timer tick handler exit.
///
/// CRITICAL: Eviction must complete atomically - no preemption during cleanup.
/// This ensures scheduler and task state remain consistent.
///
/// Returns the number of tasks evicted.
pub fn process_pending_evictions() -> usize {
    // Guarantee atomicity - no preemption during eviction
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    let mut evicted = 0;

    for slot in PENDING_EVICTIONS.iter() {
        // Atomically take the task_id (and clear the slot)
        let task_id = slot.task_id.swap(0, Ordering::SeqCst);
        if task_id == 0 {
            continue;
        }

        let reason_val = slot.reason.swap(0, Ordering::SeqCst);
        let reason = u8_to_reason(reason_val).unwrap_or(EvictionReason::StateCorruption);

        // Actually evict the task
        if do_evict_task(task_id, reason) {
            evicted += 1;
        }
    }

    evicted
}

/// Actually perform the eviction of a single task.
///
/// This transitions the task to Evicting state and marks it for cleanup.
fn do_evict_task(task_id: TaskId, reason: EvictionReason) -> bool {
    use super::with_scheduler;

    with_scheduler(|sched| {
        if let Some(slot) = sched.slot_by_pid(task_id) {
            if let Some(task) = sched.task_mut(slot) {
                // Check if already terminated
                if task.is_terminated() {
                    kerror!("evict", "already_terminated";
                        pid = task_id as u64,
                        state = task.state().name()
                    );
                    return false;
                }

                // Log the eviction with full context
                kerror!("evict", "evicting";
                    pid = task_id as u64,
                    name = task.name_str(),
                    from_state = task.state().name(),
                    reason = reason_to_u8(reason) as u64,
                    parent = task.parent_id as u64
                );

                // Transition to Evicting
                if let Err(e) = task.evict(reason) {
                    // This shouldn't happen since we checked is_terminated
                    kerror!("evict", "transition_failed";
                        pid = task_id as u64,
                        from = e.from,
                        to = e.to
                    );
                    return false;
                }

                true
            } else {
                false
            }
        } else {
            // Task not found (maybe already reaped)
            kerror!("evict", "not_found"; pid = task_id as u64);
            false
        }
    })
}

/// Check if any evictions are pending.
///
/// Useful for deciding whether to call `process_pending_evictions()`.
pub fn has_pending_evictions() -> bool {
    PENDING_EVICTIONS.iter().any(|slot| {
        slot.task_id.load(Ordering::SeqCst) != 0
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reason_conversion() {
        let reasons = [
            EvictionReason::InvalidStateTransition,
            EvictionReason::StateCorruption,
            EvictionReason::SyscallPanic,
            EvictionReason::LivenessTimeout,
            EvictionReason::ResourceExhaustion,
        ];

        for reason in reasons {
            let val = reason_to_u8(reason);
            let back = u8_to_reason(val);
            assert_eq!(back, Some(reason));
        }
    }

    #[test]
    fn test_invalid_reason() {
        assert_eq!(u8_to_reason(0), None);
        assert_eq!(u8_to_reason(255), None);
    }
}
