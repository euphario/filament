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

use super::{Task, TaskState, Priority, MAX_TASKS};
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

    /// Assign a task to a specific CPU
    ///
    /// Used for initial task placement. Default does nothing.
    #[inline]
    fn assign_task_to_cpu(&mut self, _slot: usize, _cpu: u32) {}

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

#[inline]
fn bitset_clear_all(bits: &mut ReadyBitset) {
    for w in bits.iter_mut() {
        *w = 0;
    }
}

/// Per-CPU queue scheduling with work stealing
///
/// Each CPU has a ready bitset tracking which task slots are ready on it.
/// Tasks are assigned to CPUs at spawn time (round-robin).
/// When a CPU's queue is empty, it steals from other CPUs.
///
/// # Data Structures
/// - `ready[cpu]`: [u64; 4] bitset — bit N set = task in slot N is ready on this CPU
/// - `cpu_assign[slot]`: which CPU owns this task (0xFF = unassigned)
///
/// # Complexity
/// - `select_next` with local work: O(1) via `trailing_zeros`
/// - `select_next` with stealing: O(MAX_CPUS) in worst case
pub struct PerCpuQueues {
    /// Per-CPU ready bitsets. Bit N = task slot N is ready on this CPU.
    ready: [ReadyBitset; percpu::MAX_CPUS],
    /// Which CPU each task is assigned to (UNASSIGNED = not assigned)
    cpu_assign: [u8; MAX_TASKS],
    /// Round-robin counter for task placement
    next_cpu: u8,
    /// Number of CPUs to distribute across
    num_cpus: u8,
}

impl PerCpuQueues {
    pub const fn new() -> Self {
        Self {
            ready: [[0u64; 4]; percpu::MAX_CPUS],
            cpu_assign: [UNASSIGNED; MAX_TASKS],
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

    /// Find highest-priority ready task from a bitset
    ///
    /// # SMP Safety
    /// Only considers tasks where:
    /// - State is Ready
    /// - Kernel stack is available (not being used by another CPU during switch)
    fn best_from_bitset(&self, bitset: &ReadyBitset, tasks: &[Option<Task>; MAX_TASKS]) -> Option<usize> {
        let mut best_slot: Option<usize> = None;
        let mut best_priority = Priority::Low;

        for (word_idx, &word) in bitset.iter().enumerate() {
            let mut bits = word;
            while bits != 0 {
                let bit = bits.trailing_zeros() as usize;
                let slot = word_idx * 64 + bit;
                bits &= bits - 1; // Clear lowest set bit

                if slot < MAX_TASKS {
                    if let Some(ref task) = tasks[slot] {
                        // SMP EXCLUSIVITY: Skip tasks whose kernel stack is still
                        // owned by another CPU (mid-context-switch). This prevents
                        // running a task on two CPUs simultaneously.
                        if *task.state() == TaskState::Ready && task.kernel_stack_available() {
                            if best_slot.is_none() || task.priority < best_priority {
                                best_slot = Some(slot);
                                best_priority = task.priority;
                                if best_priority == Priority::High {
                                    return Some(slot);
                                }
                            }
                        }
                    }
                }
            }
        }

        best_slot
    }
}

impl SchedulingPolicy for PerCpuQueues {
    fn select_next(&mut self, current_slot: usize, tasks: &[Option<Task>; MAX_TASKS]) -> Option<usize> {
        let my_cpu = percpu::cpu_id() as usize;
        let my_idle = my_cpu;

        // 1. Check local CPU's ready queue
        if let Some(slot) = self.best_from_bitset(&self.ready[my_cpu], tasks) {
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
            if let Some(slot) = self.best_from_bitset(&self.ready[victim], tasks) {
                return Some(slot);
            }
        }

        // 4. Fall back to this CPU's idle task
        // Note: idle tasks should never have kernel_stack_owner set since they
        // never context switch away (they wait for interrupts). But we check
        // for consistency.
        if let Some(ref task) = tasks[my_idle] {
            if task.is_runnable() && task.kernel_stack_available() {
                return Some(my_idle);
            }
        }

        None
    }

    fn on_task_ready(&mut self, slot: usize, _priority: Priority) {
        if slot < MAX_TASKS {
            let cpu = self.cpu_assign[slot];
            if cpu != UNASSIGNED && (cpu as usize) < percpu::MAX_CPUS {
                bitset_set(&mut self.ready[cpu as usize], slot);
            }
        }
    }

    fn on_task_blocked(&mut self, slot: usize) {
        if slot < MAX_TASKS {
            for r in self.ready.iter_mut() {
                bitset_clear(r, slot);
            }
        }
    }

    fn on_task_exit(&mut self, slot: usize) {
        if slot < MAX_TASKS {
            for r in self.ready.iter_mut() {
                bitset_clear(r, slot);
            }
            self.cpu_assign[slot] = UNASSIGNED;
        }
    }

    fn assign_task_to_cpu(&mut self, slot: usize, cpu: u32) {
        if slot < MAX_TASKS && (cpu as usize) < percpu::MAX_CPUS {
            self.cpu_assign[slot] = cpu as u8;
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
