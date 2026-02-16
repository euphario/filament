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
//! - `PerCpuQueues` - Default: SMP-aware with per-CPU ready bitsets and work stealing
//!
//! # SMP Design
//!
//! Each task is assigned to a CPU. The scheduler first checks the local
//! CPU's ready bitset, then steals from other CPUs if empty.

use super::{Task, TaskState, Priority, NUM_PRIORITIES, MAX_TASKS};
use crate::kernel::percpu;

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

    /// Assign a task to the next CPU via round-robin
    ///
    /// Called at spawn time to distribute tasks across CPUs.
    /// Default does nothing.
    #[inline]
    fn assign_task_round_robin(&mut self, _slot: usize) {}

    /// Get which CPU a task is assigned to
    ///
    /// Returns `None` if the task is unassigned. Default returns None.
    #[inline]
    fn task_cpu(&self, _slot: usize) -> Option<u32> { None }

    /// Get the name of this policy (for debugging/logging)
    fn name(&self) -> &'static str;
}

// ============================================================================
// Per-CPU Queues (SMP-aware)
// ============================================================================

/// Unassigned CPU sentinel
const UNASSIGNED: u8 = 0xFF;

/// 256-bit ready bitset (4 × u64 words)
type ReadyBitset = [u64; 4];

#[inline]
fn bitset_set(bits: &mut ReadyBitset, slot: usize) {
    bits[slot / 64] |= 1u64 << (slot % 64);
}

#[inline]
fn bitset_clear(bits: &mut ReadyBitset, slot: usize) {
    bits[slot / 64] &= !(1u64 << (slot % 64));
}

/// Per-CPU, per-priority queue scheduling with work stealing
///
/// Each CPU has NUM_PRIORITIES ready bitsets — one per priority level.
/// Task selection is O(1): iterate priorities 0..7, find first non-empty
/// bitset, pick the first ready task from it via `trailing_zeros`.
///
/// # Data Structures
/// - `ready[cpu][priority]`: [u64; 4] bitset — bit N set = task slot N ready at this priority
/// - `cpu_assign[slot]`: which CPU owns this task (0xFF = unassigned)
/// - `task_prio[slot]`: cached priority level for this task (for fast removal)
///
/// # Complexity
/// - `select_next` with local work: O(NUM_PRIORITIES) priority scan + O(1) bit scan
/// - `select_next` with stealing: O(MAX_CPUS × NUM_PRIORITIES) worst case
pub struct PerCpuQueues {
    /// Per-CPU, per-priority ready bitsets.
    /// ready[cpu][priority] has bit N set if task N is ready at that priority.
    ready: [[ReadyBitset; NUM_PRIORITIES]; percpu::MAX_CPUS],
    /// Which CPU each task is assigned to (UNASSIGNED = not assigned)
    cpu_assign: [u8; MAX_TASKS],
    /// Cached priority level per task (for O(1) removal without scanning all levels)
    task_prio: [u8; MAX_TASKS],
    /// Round-robin counter for task placement
    next_cpu: u8,
    /// Number of CPUs to distribute across
    num_cpus: u8,
}

impl PerCpuQueues {
    pub const fn new() -> Self {
        Self {
            ready: [[[0u64; 4]; NUM_PRIORITIES]; percpu::MAX_CPUS],
            cpu_assign: [UNASSIGNED; MAX_TASKS],
            task_prio: [0xFF; MAX_TASKS],
            next_cpu: 0,
            num_cpus: 1,
        }
    }

    /// Update the number of CPUs for round-robin distribution.
    /// Called after secondary CPUs come online.
    pub fn set_num_cpus(&mut self, n: u8) {
        if n > 0 && (n as usize) <= percpu::MAX_CPUS {
            self.num_cpus = n;
        }
    }

    /// Find the first ready task from a CPU's per-priority bitsets.
    /// Iterates from highest priority (0) to lowest (7).
    ///
    /// # SMP Safety
    /// Only considers tasks where:
    /// - State is Ready
    /// - Kernel stack is available (not being used by another CPU during switch)
    fn first_ready_from_cpu(&self, cpu: usize, tasks: &[Option<Task>; MAX_TASKS]) -> Option<usize> {
        for prio in 0..NUM_PRIORITIES {
            if let Some(slot) = self.first_ready_in_bitset(&self.ready[cpu][prio], tasks) {
                return Some(slot);
            }
        }
        None
    }

    /// Find the first ready task in a single bitset via trailing_zeros scan.
    fn first_ready_in_bitset(&self, bitset: &ReadyBitset, tasks: &[Option<Task>; MAX_TASKS]) -> Option<usize> {
        for (word_idx, &word) in bitset.iter().enumerate() {
            let mut bits = word;
            while bits != 0 {
                let bit = bits.trailing_zeros() as usize;
                let slot = word_idx * 64 + bit;
                bits &= bits - 1; // Clear lowest set bit

                if slot < MAX_TASKS {
                    if let Some(ref task) = tasks[slot] {
                        if *task.state() == TaskState::Ready && task.kernel_stack_available() {
                            return Some(slot);
                        }
                    }
                }
            }
        }
        None
    }
}

impl SchedulingPolicy for PerCpuQueues {
    fn select_next(&mut self, current_slot: usize, tasks: &[Option<Task>; MAX_TASKS]) -> Option<usize> {
        let my_cpu = percpu::cpu_id() as usize;
        let my_idle = my_cpu;

        // 1. Check local CPU's per-priority ready queues (highest priority first)
        if let Some(slot) = self.first_ready_from_cpu(my_cpu, tasks) {
            return Some(slot);
        }

        // 2. Check if current task is still runnable (not idle)
        if current_slot >= percpu::MAX_CPUS {
            if let Some(ref task) = tasks[current_slot] {
                if task.is_runnable() && task.kernel_stack_available() {
                    return Some(current_slot);
                }
            }
        }

        // 3. Work stealing — try other CPUs' queues (round-robin victim selection)
        for offset in 1..percpu::MAX_CPUS {
            let victim = (my_cpu + offset) % percpu::MAX_CPUS;
            if let Some(slot) = self.first_ready_from_cpu(victim, tasks) {
                return Some(slot);
            }
        }

        // 4. Fall back to this CPU's idle task
        if let Some(ref task) = tasks[my_idle] {
            if task.is_runnable() && task.kernel_stack_available() {
                return Some(my_idle);
            }
        }

        None
    }

    fn on_task_ready(&mut self, slot: usize, priority: Priority) {
        if slot < MAX_TASKS {
            let prio = priority as u8 as usize;
            self.task_prio[slot] = prio as u8;
            let cpu = self.cpu_assign[slot];
            if cpu != UNASSIGNED && (cpu as usize) < percpu::MAX_CPUS {
                bitset_set(&mut self.ready[cpu as usize][prio], slot);
            }
        }
    }

    fn on_task_blocked(&mut self, slot: usize) {
        if slot < MAX_TASKS {
            let prio = self.task_prio[slot] as usize;
            if prio < NUM_PRIORITIES {
                // Fast path: clear from cached priority level only
                for cpu_bitsets in self.ready.iter_mut() {
                    bitset_clear(&mut cpu_bitsets[prio], slot);
                }
            } else {
                // Fallback: clear from all levels
                for cpu_bitsets in self.ready.iter_mut() {
                    for prio_bitset in cpu_bitsets.iter_mut() {
                        bitset_clear(prio_bitset, slot);
                    }
                }
            }
        }
    }

    fn on_priority_change(&mut self, slot: usize, old: Priority, new: Priority) {
        if slot < MAX_TASKS {
            let old_prio = old as u8 as usize;
            let new_prio = new as u8 as usize;
            if old_prio == new_prio {
                return;
            }
            self.task_prio[slot] = new_prio as u8;
            // Move bit from old priority level to new
            let cpu = self.cpu_assign[slot];
            if cpu != UNASSIGNED && (cpu as usize) < percpu::MAX_CPUS {
                let cpu_idx = cpu as usize;
                bitset_clear(&mut self.ready[cpu_idx][old_prio], slot);
                bitset_set(&mut self.ready[cpu_idx][new_prio], slot);
            }
        }
    }

    fn on_task_exit(&mut self, slot: usize) {
        if slot < MAX_TASKS {
            // Clear from all priority levels on all CPUs
            for cpu_bitsets in self.ready.iter_mut() {
                for prio_bitset in cpu_bitsets.iter_mut() {
                    bitset_clear(prio_bitset, slot);
                }
            }
            self.cpu_assign[slot] = UNASSIGNED;
            self.task_prio[slot] = 0xFF;
        }
    }

    fn assign_task_round_robin(&mut self, slot: usize) {
        if slot < MAX_TASKS {
            self.cpu_assign[slot] = self.next_cpu;
            self.next_cpu = (self.next_cpu + 1) % self.num_cpus;
        }
    }

    fn task_cpu(&self, slot: usize) -> Option<u32> {
        if slot < MAX_TASKS && self.cpu_assign[slot] != UNASSIGNED {
            Some(self.cpu_assign[slot] as u32)
        } else {
            None
        }
    }

    fn name(&self) -> &'static str {
        "PerCpuQueues"
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Tests would require mock Task structures
    // For now, this validates the trait design compiles
}
