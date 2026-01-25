//! Scheduling Policies
//!
//! This module provides trait-based scheduling algorithms that can be
//! swapped without changing the rest of the kernel.
//!
//! # Design
//!
//! The `SchedulingPolicy` trait defines the interface for task selection.
//! Implementations provide different scheduling strategies:
//!
//! - `PriorityRoundRobin` - Current default: priority levels with round-robin
//! - Future: `PerCpuQueues` - SMP-aware with per-CPU run queues
//! - Future: `RealtimePolicy` - Hard real-time guarantees
//!
//! # SMP Considerations
//!
//! For future SMP support, policies can:
//! - Track per-CPU ready queues
//! - Implement work stealing
//! - Respect CPU affinity/pinning
//! - Isolate high-performance tasks to dedicated CPUs

use super::{Task, TaskState, Priority, MAX_TASKS};

/// Scheduling policy trait
///
/// Defines the interface for task selection algorithms. The scheduler
/// calls `select_next()` to pick which task runs next.
///
/// Implementations are notified of state changes via callback methods,
/// allowing them to maintain internal data structures (e.g., per-priority
/// run queues) without scanning the full task array each time.
pub trait SchedulingPolicy {
    /// Select the next task slot to run
    ///
    /// # Arguments
    /// - `current_slot` - The currently running task's slot
    /// - `tasks` - The task array (read-only view)
    ///
    /// # Returns
    /// - `Some(slot)` - The slot of the next task to run
    /// - `None` - No runnable tasks (caller should idle)
    fn select_next(&mut self, current_slot: usize, tasks: &[Option<Task>; MAX_TASKS]) -> Option<usize>;

    /// Called when a task becomes ready (woken or created)
    ///
    /// Policies that maintain ready queues can use this to add the task.
    /// Default implementation does nothing (stateless policies).
    #[inline]
    fn on_task_ready(&mut self, _slot: usize, _priority: Priority) {}

    /// Called when a task blocks (sleeping/waiting)
    ///
    /// Policies that maintain ready queues can use this to remove the task.
    /// Default implementation does nothing (stateless policies).
    #[inline]
    fn on_task_blocked(&mut self, _slot: usize) {}

    /// Called when a task's priority changes
    ///
    /// Policies that use priority queues can reposition the task.
    /// Default implementation does nothing.
    #[inline]
    fn on_priority_change(&mut self, _slot: usize, _old: Priority, _new: Priority) {}

    /// Called when a task terminates
    ///
    /// Policies can clean up any tracking state for this task.
    /// Default implementation does nothing.
    #[inline]
    fn on_task_exit(&mut self, _slot: usize) {}

    /// Get the name of this policy (for debugging/logging)
    fn name(&self) -> &'static str;
}

// ============================================================================
// Priority Round-Robin Policy
// ============================================================================

/// Priority-based round-robin scheduling
///
/// This is the default scheduler policy:
/// - Higher priority tasks (lower Priority value) always run first
/// - Among same-priority tasks, uses round-robin for fairness
/// - Scans task array starting from current+1 for round-robin behavior
///
/// # Complexity
/// - `select_next`: O(n) where n = MAX_TASKS
/// - Could be optimized with per-priority ready queues if needed
pub struct PriorityRoundRobin {
    /// Slot 0 is reserved for idle task - skip it in normal scheduling
    idle_slot: usize,
}

impl PriorityRoundRobin {
    pub const fn new() -> Self {
        Self { idle_slot: 0 }
    }
}

impl SchedulingPolicy for PriorityRoundRobin {
    fn select_next(&mut self, current_slot: usize, tasks: &[Option<Task>; MAX_TASKS]) -> Option<usize> {
        let mut best_slot: Option<usize> = None;
        let mut best_priority = Priority::Low;

        // Scan from current+1 for round-robin fairness
        let start = (current_slot + 1) % MAX_TASKS;
        for i in 0..MAX_TASKS {
            let slot = (start + i) % MAX_TASKS;

            // Skip idle slot in first pass (use it as fallback)
            if slot == self.idle_slot {
                continue;
            }

            if let Some(ref task) = tasks[slot] {
                if *task.state() == TaskState::Ready {
                    // Lower priority value = higher priority
                    if best_slot.is_none() || task.priority < best_priority {
                        best_slot = Some(slot);
                        best_priority = task.priority;

                        // Early exit for highest priority
                        if best_priority == Priority::High {
                            break;
                        }
                    }
                }
            }
        }

        // If we found a ready task, return it
        if best_slot.is_some() {
            return best_slot;
        }

        // Check if current task is still runnable (it might be Running, not Ready)
        if let Some(ref task) = tasks[current_slot] {
            if task.is_runnable() {
                // Log if pid=2 (devd) is being returned as current
                if task.id == 2 {
                    crate::kdebug!("sched", "select_current"; slot = current_slot, pid = 2, state = task.state().name());
                }
                return Some(current_slot);
            }
        }

        // Fall back to idle task if nothing else
        if let Some(ref task) = tasks[self.idle_slot] {
            if task.is_runnable() {
                crate::kdebug!("sched", "select_idle"; from = current_slot);
                return Some(self.idle_slot);
            }
        }

        crate::kerror!("sched", "select_none"; from = current_slot);
        None
    }

    fn name(&self) -> &'static str {
        "PriorityRoundRobin"
    }
}

// ============================================================================
// Future: Per-CPU Queues (SMP-aware)
// ============================================================================

// /// Per-CPU queue scheduling for SMP systems
// ///
// /// Each CPU has its own ready queue, reducing lock contention.
// /// Work stealing allows idle CPUs to take tasks from busy ones.
// ///
// /// # Features (TODO)
// /// - Per-CPU ready queues
// /// - Work stealing with configurable threshold
// /// - CPU affinity support
// /// - High-performance (HP) task isolation
// pub struct PerCpuQueues {
//     // queues: [ReadyQueue; MAX_CPUS],
//     // affinity: [CpuMask; MAX_TASKS],
// }

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Tests would require mock Task structures
    // For now, this validates the trait design compiles
}
