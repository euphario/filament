//! Task Management and Scheduling
//!
//! This module provides the core task/process abstraction for the microkernel.
//! Each task has its own saved CPU state, address space, and resource tracking.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                      Scheduler                               │
//! │  - Global task table (MAX_TASKS slots)                      │
//! │  - Per-CPU current slot tracking                            │
//! │  - Schedule policy (priority round-robin)                   │
//! └─────────────────────────────────────────────────────────────┘
//!                              │
//!                    ┌────────┴────────┐
//!                    ▼                 ▼
//!              ┌─────────┐       ┌─────────┐
//!              │  Idle   │       │  User   │  ...
//!              │ slot 0  │       │ slot N  │  (N = MAX_CPUS)
//!              └────┬────┘       └────┬────┘
//!                   │                 │
//!        ┌─────────┴────┐    ┌──────┴───────┐
//!        ▼              ▼    ▼              ▼
//!   TaskState      TrapFrame    AddressSpace   ObjectTable
//! ```
//!
//! # Public API
//!
//! ## Types
//! - [`TaskState`] - State machine with validated transitions
//! - [`SleepReason`] - Why a task is sleeping (EventLoop, Request, etc.)
//! - [`WaitReason`] - Why a task is waiting with a deadline
//! - [`Priority`] - Task scheduling priority (High, Normal, Low)
//! - [`TaskId`] - Task identifier (generation-based PID)
//!
//! ## Global Functions
//! - [`scheduler()`] - Get mutable reference to global scheduler (unsafe)
//! - [`current_slot()`] - Get current CPU's running task slot
//! - [`set_current_slot()`] - Set current CPU's running task slot
//!
//! ## Scheduler Methods
//! - [`Scheduler::spawn_kernel_task()`] - Create a kernel-mode task
//! - [`Scheduler::spawn_user_task()`] - Create a user-mode task
//! - [`Scheduler::schedule()`] - Pick next ready task to run
//! - [`Scheduler::task()`] / [`Scheduler::task_mut()`] - Access task by slot
//! - [`Scheduler::current_task()`] / [`Scheduler::current_task_mut()`] - Access running task
//! - [`Scheduler::slot_by_pid()`] - Find slot by PID (O(1) with generation check)
//! - [`Scheduler::wake_by_pid()`] - Wake a sleeping/waiting task
//!
//! ## Task Methods
//! - [`Task::state`] - Current state (via accessor, use state machine for changes)
//! - [`Task::trap_frame()`] / [`Task::trap_frame_mut()`] - Access saved registers
//! - [`Task::mmap_*()`] - Map memory regions into task's address space
//!
//! # State Machine
//!
//! Tasks follow a strict state machine (see [`state`] module):
//!
//! ```text
//!                    ┌──────────┐
//!      spawn() ────▶ │  Ready   │ ◀──── wake()
//!                    └────┬─────┘
//!                         │ schedule()
//!                         ▼
//!                    ┌──────────┐
//!                    │ Running  │
//!                    └────┬─────┘
//!          ┌──────────────┼──────────────┐
//!          │              │              │
//!          ▼              ▼              ▼
//!     ┌─────────┐   ┌──────────┐   ┌──────────┐
//!     │Sleeping │   │ Waiting  │   │ Exiting  │
//!     └─────────┘   └──────────┘   └────┬─────┘
//!                                       │
//!                                       ▼
//!                                  ┌─────────┐
//!                                  │  Dying  │
//!                                  └────┬────┘
//!                                       │
//!                                       ▼
//!                                  ┌─────────┐
//!                                  │  Dead   │
//!                                  └─────────┘
//! ```
//!
//! # Module Structure
//!
//! - [`state`] - Task state machine with validated transitions
//! - [`lifecycle`] - Centralized lifecycle operations (exit, kill, wait, wake)
//! - [`policy`] - Pluggable scheduling policies (trait + implementations)
//!
//! # Thread Safety
//!
//! - Global scheduler protected by spinlock (IRQ-safe)
//! - Per-CPU current slot uses atomic operations
//! - Task access requires holding scheduler lock

#![allow(dead_code)]  // Some wait reasons and methods are for future use

// Submodules
pub mod state;
pub mod lifecycle;
pub mod policy;
pub mod tcb;
pub mod eviction;

// Re-export core types from state module
pub use state::{TaskState, SleepReason, WaitReason, EvictionReason};

// Re-export scheduling policy trait and implementations
pub use policy::{SchedulingPolicy, PerCpuQueues};

// Re-export TCB types
pub use tcb::{
    Priority, TrapFrame, CpuContext, TaskId, Task,
    MAX_CHILDREN, MAX_CHANNELS_PER_TASK, CLEANUP_GRACE_TICKS,
    enter_usermode, context_switch,
};

use crate::{kerror, print_direct};

// ============================================================================
// Error Handling Macros for State Transitions
// ============================================================================

/// State transition that evicts task on failure (critical transitions).
///
/// Use this for transitions that MUST succeed for correct operation.
/// If the transition fails, the task is marked for eviction and will be
/// cleaned up at the next safe point.
///
/// Returns `true` if transition succeeded, `false` if task was marked for eviction.
#[macro_export]
macro_rules! transition_or_evict {
    ($task:expr, $method:ident $(, $arg:expr)*) => {{
        match $task.$method($($arg),*) {
            Ok(_) => true,
            Err(e) => {
                $crate::kerror!("state", "transition_fail";
                    pid = $task.id as u64,
                    from = e.from,
                    to = e.to
                );
                $crate::kernel::task::eviction::mark_for_eviction(
                    $task.id,
                    $crate::kernel::task::EvictionReason::InvalidStateTransition,
                );
                false
            }
        }
    }};
}

/// State transition that logs but continues (idempotent operations).
///
/// Use this for transitions that may reasonably fail without indicating
/// corruption (e.g., waking a task that's already Ready).
#[macro_export]
macro_rules! transition_or_log {
    ($task:expr, $method:ident $(, $arg:expr)*) => {{
        if let Err(e) = $task.$method($($arg),*) {
            $crate::kdebug!("state", "transition_skip";
                pid = $task.id as u64,
                from = e.from,
                to = e.to
            );
        }
    }};
}

/// Maximum number of tasks (slots 0..MAX_CPUS reserved for idle)
pub const MAX_TASKS: usize = 256;

/// Scheduler state
///
/// The scheduler manages all tasks and their state. For SMP safety:
/// - Task array modifications are protected by IrqGuard (single-core) or SpinLock (SMP)
/// - Current task index is stored per-CPU via percpu module (use `current_slot()`)
///
/// # Scheduling Policy
///
/// The scheduler uses a pluggable `SchedulingPolicy` trait for task selection.
/// The default policy is `PerCpuQueues` - priority levels with round-robin
/// fairness within each level. Different policies can be swapped for:
/// - SMP-aware scheduling with per-CPU queues
/// - Real-time scheduling with deadline guarantees
/// - Custom workload-specific policies (e.g., WiFi router optimization)
pub struct Scheduler {
    /// All tasks (shared across CPUs) - access via task()/task_mut() only
    tasks: [Option<Task>; MAX_TASKS],
    /// Generation counter per slot - increments when slot is reused
    /// PID = (slot + 1) | (generation << 8)
    /// This ensures stale PIDs from terminated tasks don't match new tasks
    generations: [u32; MAX_TASKS],
    /// Next deadline across all tasks (for tickless optimization)
    /// Set to u64::MAX when no deadlines are active.
    /// This allows check_timeouts to early-return most ticks.
    next_deadline: u64,
    /// Scheduling policy for task selection
    policy: PerCpuQueues,
}

/// Get current CPU's running task slot index (SMP-safe)
#[inline]
pub fn current_slot() -> usize {
    super::percpu::cpu_local().get_current_slot()
}

/// Set current CPU's running task slot index (SMP-safe)
#[inline]
pub fn set_current_slot(slot: usize) {
    super::percpu::cpu_local().set_current_slot(slot);
}

impl Scheduler {
    pub const fn new() -> Self {
        const NONE: Option<Task> = None;
        Self {
            tasks: [NONE; MAX_TASKS],
            generations: [0; MAX_TASKS],
            next_deadline: u64::MAX,
            policy: PerCpuQueues::new(),
        }
    }

    /// Update next_deadline if the given deadline is sooner
    /// Called when arming a timer or entering Waiting state with deadline
    #[inline]
    pub fn note_deadline(&mut self, deadline: u64) {
        if deadline < self.next_deadline {
            self.next_deadline = deadline;
        }
    }

    /// Wake parent task if it's sleeping (helper for cleanup code)
    ///
    /// This allows EventLoop to poll ProcessObjects and see child exits.
    /// Called during task termination to notify parent.
    fn wake_parent_if_sleeping(&mut self, slot_idx: usize) {
        let parent_id = match self.tasks[slot_idx].as_ref() {
            Some(task) => task.parent_id,
            None => return,
        };
        if parent_id == 0 {
            return;
        }
        if let Some(parent_slot) = self.slot_by_pid(parent_id) {
            if let Some(ref mut parent) = self.tasks[parent_slot] {
                if parent.state().is_sleeping() {
                    crate::transition_or_log!(parent, wake);
                }
            }
        }
    }

    /// Recalculate next_deadline by scanning all active timers and waiting tasks
    /// Called after a deadline fires to find the next one
    pub fn recalculate_next_deadline(&mut self) {
        let mut soonest = u64::MAX;

        for task_opt in self.tasks.iter() {
            if let Some(ref task) = task_opt {
                // Skip terminated tasks
                if task.is_terminated() {
                    continue;
                }

                // Check Waiting state deadline
                if let TaskState::Waiting { deadline, .. } = *task.state() {
                    if deadline < soonest {
                        soonest = deadline;
                    }
                }

                // Check Dying state deadline
                if let TaskState::Dying { until, .. } = *task.state() {
                    if until < soonest {
                        soonest = until;
                    }
                }

            }
        }

        self.next_deadline = soonest;
    }

    /// Number of bits used for slot encoding in PID
    const PID_SLOT_BITS: u32 = 9;
    /// Mask for extracting slot bits from PID
    const PID_SLOT_MASK: u32 = (1 << 9) - 1; // 0x1FF

    /// Generate a PID for a given slot
    /// PID format: bits[8:0] = slot + 1 (1-256), bits[31:9] = generation (23 bits)
    ///
    /// Idle tasks (slots 0..MAX_CPUS) get PID 0. User tasks start at PID 1.
    /// This ensures devd (first user task) gets PID 1.
    ///
    /// The generation is masked to 23 bits to prevent overflow when shifted.
    /// After 2^23 task creations in a slot, generation wraps to 0, which is
    /// acceptable since the probability of a stale reference surviving that
    /// long is negligible.
    fn make_pid(&self, slot: usize) -> TaskId {
        use crate::kernel::percpu::MAX_CPUS;
        if slot < MAX_CPUS {
            // Idle tasks get PID 0 - they're internal kernel tasks
            return 0;
        }
        // User tasks: slot MAX_CPUS → PID 1, slot MAX_CPUS+1 → PID 2, etc.
        let slot_bits = (slot - MAX_CPUS + 1) as u32;
        let gen = self.generations[slot] & ((1 << (32 - Self::PID_SLOT_BITS)) - 1);
        slot_bits | (gen << Self::PID_SLOT_BITS)
    }

    /// Extract slot from PID (returns None if invalid)
    fn slot_from_pid(pid: TaskId) -> Option<usize> {
        use crate::kernel::percpu::MAX_CPUS;
        if pid == 0 {
            // PID 0 is reserved for idle - but we can't map it back to a specific slot
            // since all idle tasks share PID 0. Return None.
            return None;
        }
        let slot_bits = (pid & Self::PID_SLOT_MASK) as usize;
        if slot_bits == 0 || slot_bits + MAX_CPUS > MAX_TASKS {
            return None;
        }
        Some(slot_bits + MAX_CPUS - 1)
    }

    /// Extract generation from PID
    /// Used for subscriber tracking to detect stale references
    pub fn generation_from_pid(pid: TaskId) -> u32 {
        pid >> Self::PID_SLOT_BITS
    }

    /// Increment generation for a slot (call when task terminates)
    pub fn bump_generation(&mut self, slot: usize) {
        self.generations[slot] = self.generations[slot].wrapping_add(1);
    }

    /// Look up slot by PID - O(1) with generation verification
    /// Returns Some(slot) if found and valid, None if stale/invalid
    pub fn slot_by_pid(&self, pid: TaskId) -> Option<usize> {
        let slot = Self::slot_from_pid(pid)?;
        // Verify the task exists and PID matches (generation check)
        if let Some(ref task) = self.tasks[slot] {
            if task.id == pid {
                return Some(slot);
            }
        }
        None
    }

    /// Notify the scheduling policy that a task became ready.
    /// Also tracks on_runq invariant in debug builds.
    pub fn notify_ready(&mut self, slot: usize) {
        if let Some(ref mut task) = self.tasks[slot] {
            task.set_on_runq(true);
            self.policy.on_task_ready(slot, task.priority);
        }
    }

    /// Notify the scheduling policy that a task blocked.
    /// Also tracks on_runq invariant in debug builds.
    pub fn notify_blocked(&mut self, slot: usize) {
        if let Some(ref mut task) = self.tasks[slot] {
            task.set_on_runq(false);
        }
        self.policy.on_task_blocked(slot);
    }

    /// Notify the scheduling policy that a task exited.
    /// Also tracks on_runq invariant in debug builds.
    pub fn notify_exit(&mut self, slot: usize) {
        if let Some(ref mut task) = self.tasks[slot] {
            // Only clear if on runq (task may have been blocked when it exited)
            if task.is_on_runq() {
                task.set_on_runq(false);
            }
        }
        self.policy.on_task_exit(slot);
    }

    /// Wake a task by slot index - internal unified wake function
    ///
    /// This is THE central place for waking tasks. All wake paths should
    /// eventually call this function to ensure:
    /// 1. State transition is valid (blocked → ready)
    /// 2. Liveness state is reset to Normal
    ///
    /// Returns true if task was actually woken (was blocked).
    pub fn wake_task(&mut self, slot: usize) -> bool {
        if let Some(ref mut task) = self.tasks[slot] {
            // Only wake if task is blocked
            if !task.is_blocked() {
                return false;
            }

            // Transition to Ready via state machine
            crate::transition_or_log!(task, wake);

            // Reset liveness state - task responded/became active
            task.liveness_state = super::liveness::LivenessState::Normal;

            // Clear kernel_stack_owner so the task is selectable.
            // When a task blocked and context-switched away, it had
            // kernel_stack_owner set. Now that it's being woken, it's
            // ready to be scheduled again.
            task.clear_kernel_stack_owner();

            // Track runq invariant in debug builds
            task.set_on_runq(true);

            // Notify scheduling policy
            self.policy.on_task_ready(slot, task.priority);

            // Set need_resched so the woken task gets scheduled
            // when the current syscall returns
            crate::arch::aarch64::sync::cpu_flags().set_need_resched();

            true
        } else {
            false
        }
    }

    /// Wake a task by PID - O(1) lookup
    /// Returns true if task was woken
    pub fn wake_by_pid(&mut self, pid: TaskId) -> bool {
        if let Some(slot) = self.slot_by_pid(pid) {
            self.wake_task(slot)
        } else {
            false
        }
    }

    // NOTE: Old heartbeat timeout constant removed - using liveness ping/pong system instead

    /// Check for timed-out blocked tasks, deliver timer events, and wake them
    /// Called from timer tick handler
    /// Returns number of tasks woken
    pub fn check_timeouts(&mut self, current_tick: u64) -> usize {
        // Tickless optimization: early return if no deadlines are due
        if current_tick < self.next_deadline {
            return 0;
        }

        let mut woken = 0;
        let mut need_recalculate = false;

        // Collect slots woken by deadline expiry for policy notification
        let mut woken_slots = [0u16; 64];
        let mut woken_slot_count = 0usize;

        // Collect TimerObject subscribers to wake (from Mux watches)
        use crate::kernel::ipc::traits::Subscriber;
        let mut timer_subscribers: [Subscriber; 32] = [Subscriber { task_id: 0, generation: 0 }; 32];
        let mut timer_sub_count = 0;

        for (slot_idx, task_opt) in self.tasks.iter_mut().enumerate() {
            if let Some(ref mut task) = task_opt {
                // Skip terminated tasks - their timers should not fire
                if task.is_terminated() {
                    continue;
                }

                // Check waiting deadline (for request-response patterns)
                // Sleeping tasks have no deadline - only Waiting tasks do
                if let TaskState::Waiting { deadline, .. } = *task.state() {
                    if current_tick >= deadline {
                        crate::transition_or_log!(task, wake);
                        if woken_slot_count < woken_slots.len() {
                            woken_slots[woken_slot_count] = slot_idx as u16;
                            woken_slot_count += 1;
                        }
                        woken += 1;
                        need_recalculate = true;
                    }
                }

                // Collect task ID for TimerObject checking via ObjectService
                if timer_sub_count < timer_subscribers.len() {
                    timer_subscribers[timer_sub_count] = Subscriber {
                        task_id: task.id,
                        generation: Scheduler::generation_from_pid(task.id),
                    };
                    timer_sub_count += 1;
                } else {
                    crate::kwarn!("timer", "subscriber_overflow"; dropped = 1);
                }
            }
        }

        // Notify policy about deadline-woken tasks and track runq invariant
        for i in 0..woken_slot_count {
            let slot = woken_slots[i] as usize;
            if let Some(ref mut task) = self.tasks[slot] {
                // Clear kernel_stack_owner so task is selectable
                task.clear_kernel_stack_owner();
                task.set_on_runq(true);
                self.policy.on_task_ready(slot, task.priority);
            }
        }

        // Check TimerObjects via ObjectService (outside task iteration)
        let mut all_subscribers: [Subscriber; 32] = [Subscriber { task_id: 0, generation: 0 }; 32];
        let mut all_sub_count = 0;

        for i in 0..timer_sub_count {
            let task_id = timer_subscribers[i].task_id;
            let (wake_list, _) = super::object_service::object_service()
                .check_timers_for_task(task_id, current_tick, 8);

            for sub in wake_list.iter() {
                if all_sub_count < all_subscribers.len() {
                    all_subscribers[all_sub_count] = sub;
                    all_sub_count += 1;
                } else {
                    crate::kwarn!("timer", "wake_list_overflow"; dropped = 1);
                }
            }
        }

        // Wake all collected TimerObject subscribers
        for i in 0..all_sub_count {
            let subscriber = all_subscribers[i];
            if let Some(slot) = self.slot_by_pid(subscriber.task_id) {
                if let Some(ref mut task) = self.tasks[slot] {
                    if task.is_blocked() {
                        crate::transition_or_log!(task, wake);
                        task.liveness_state.reset(task.id);
                        // Clear kernel_stack_owner so task is selectable
                        task.clear_kernel_stack_owner();
                        task.set_on_runq(true);
                        self.policy.on_task_ready(slot, task.priority);
                        woken += 1;
                    }
                }
            }
        }

        // Recalculate next deadline if any fired
        if need_recalculate {
            self.recalculate_next_deadline();
        }

        woken
    }

    // NOTE: check_devd_heartbeat removed - using liveness ping/pong system instead

    /// Add a kernel task to the scheduler
    pub fn add_kernel_task(&mut self, entry: fn() -> !, name: &str) -> Option<TaskId> {
        // Find empty slot (skip slots 0..MAX_CPUS reserved for per-CPU idle tasks)
        let slot = self.tasks.iter().enumerate()
            .position(|(i, t)| i >= super::percpu::MAX_CPUS && t.is_none())?;

        // Generate PID with current generation for this slot
        let id = self.make_pid(slot);

        self.tasks[slot] = Task::new_kernel(id, entry, name);
        if self.tasks[slot].is_some() {
            self.policy.assign_task_round_robin(slot);
            self.notify_ready(slot);
            // Create parallel object table in ObjectService (Phase 1: not used yet)
            super::object_service::object_service().create_task_table(id, false);
            Some(id)
        } else {
            None
        }
    }

    /// Add a user task to the scheduler
    pub fn add_user_task(&mut self, name: &str) -> Option<(TaskId, usize)> {
        // Skip slots 0..MAX_CPUS reserved for per-CPU idle tasks
        let slot = self.tasks.iter().enumerate()
            .position(|(i, t)| i >= super::percpu::MAX_CPUS && t.is_none())?;

        // Generate PID with current generation for this slot
        let id = self.make_pid(slot);

        self.tasks[slot] = Task::new_user(id, name);
        if self.tasks[slot].is_some() {
            self.policy.assign_task_round_robin(slot);
            self.notify_ready(slot);
            // Create parallel object table in ObjectService
            super::object_service::object_service().create_task_table(id, true);
            Some((id, slot))
        } else {
            crate::kerror!("task", "add_user_fail"; slot = slot as u64, pid = id as u64);
            None
        }
    }

    // ========================================================================
    // Task Access API - Primary interface for accessing tasks
    // ========================================================================

    /// Get task by slot index (read-only).
    ///
    /// Returns `None` if slot is out of bounds or empty.
    ///
    /// # Example
    /// ```ignore
    /// if let Some(task) = sched.task(slot) {
    ///     println!("Task {} state: {:?}", task.id, task.state);
    /// }
    /// ```
    pub fn task(&self, slot: usize) -> Option<&Task> {
        self.tasks.get(slot).and_then(|t| t.as_ref())
    }

    /// Get task by slot index mutably.
    ///
    /// Returns `None` if slot is out of bounds or empty.
    /// Use this for modifying task state, trap frame, etc.
    ///
    /// # Example
    /// ```ignore
    /// if let Some(task) = sched.task_mut(slot) {
    ///     task.state.wake(); // Transition to Ready state
    /// }
    /// ```
    pub fn task_mut(&mut self, slot: usize) -> Option<&mut Task> {
        self.tasks.get_mut(slot).and_then(|t| t.as_mut())
    }

    /// Get currently running task on this CPU (read-only).
    ///
    /// Returns `None` if no task is running (shouldn't happen after boot).
    pub fn current_task(&self) -> Option<&Task> {
        self.tasks[current_slot()].as_ref()
    }

    /// Get currently running task on this CPU mutably.
    ///
    /// Returns `None` if no task is running.
    /// Common use: modifying trap frame registers for syscall return.
    pub fn current_task_mut(&mut self) -> Option<&mut Task> {
        self.tasks[current_slot()].as_mut()
    }

    /// Get ID of currently running task on this CPU.
    ///
    /// Convenience method for getting current PID without full task access.
    pub fn current_task_id(&self) -> Option<TaskId> {
        self.tasks[current_slot()].as_ref().map(|t| t.id)
    }

    /// Iterate over all task slots with their indices.
    ///
    /// Yields `(slot_index, Option<&Task>)` for each slot.
    /// Empty slots yield `None`.
    ///
    /// # Example
    /// ```ignore
    /// for (slot, task_opt) in sched.iter_tasks() {
    ///     if let Some(task) = task_opt {
    ///         if task.state == TaskState::Ready {
    ///             // Found a ready task
    ///         }
    ///     }
    /// }
    /// ```
    pub fn iter_tasks(&self) -> impl Iterator<Item = (usize, Option<&Task>)> {
        self.tasks.iter().enumerate().map(|(i, opt)| (i, opt.as_ref()))
    }

    /// Iterate over all task slots mutably with their indices.
    ///
    /// Yields `(slot_index, Option<&mut Task>)` for each slot.
    /// Use for bulk operations like timeout checking.
    pub fn iter_tasks_mut(&mut self) -> impl Iterator<Item = (usize, Option<&mut Task>)> {
        self.tasks.iter_mut().enumerate().map(|(i, opt)| (i, opt.as_mut()))
    }

    /// Clear a task slot, removing the task entirely.
    ///
    /// Used for cleanup after task death or for crash recovery.
    ///
    /// **Note**: Does not bump generation counter. Call [`bump_generation()`]
    /// separately if the slot will be reused to prevent stale PID collisions.
    pub fn clear_slot(&mut self, slot: usize) {
        if slot < MAX_TASKS {
            self.tasks[slot] = None;
        }
    }

    /// Terminate current task (uses per-CPU slot)
    pub fn terminate_current(&mut self, exit_code: i32) {
        let slot = current_slot();
        if let Some(ref mut task) = self.tasks[slot] {
            crate::transition_or_evict!(task, set_exiting, exit_code);
            // Clear is_init flag so new devd can be the init
            task.is_init = false;
        }
        self.policy.on_task_exit(slot);
    }

    /// Perform IPC cleanup for a terminating task.
    ///
    /// This handles:
    /// Phase 2 / final cleanup: free remaining resources for a dead task.
    ///
    /// Called from reap_terminated (Phase 2 + Evicting) and reap_child.
    /// Does NOT require scheduler lock — only touches subsystem-specific state.
    fn do_final_cleanup(pid: TaskId) {
        super::shmem::finalize_cleanup(pid);
        super::irq::process_cleanup(pid);
        super::pci::release_all_devices(pid);

        let port_wake_list = super::ipc::port_cleanup_task(pid);
        super::ipc::waker::wake(&port_wake_list, super::ipc::WakeReason::Closed);

        super::object_service::object_service().remove_task_table(pid);
    }

    /// Clear per-CPU trap frame if it still points to this task's frame.
    /// Prevents use-after-free if the task slot is about to be cleared.
    fn clear_stale_trap_frame(task: &Task, slot_idx: usize, pid: TaskId) {
        let task_trap_ptr = &task.trap_frame as *const TrapFrame;
        let current_ptr = super::percpu::get_trap_frame();
        if current_ptr == task_trap_ptr as *mut TrapFrame {
            crate::kwarn!("task", "clearing_stale_trap_frame"; slot = slot_idx as u64, pid = pid as u64);
            let cpu = super::percpu::cpu_id() as usize;
            unsafe {
                super::percpu::set_trap_frame(
                    core::ptr::addr_of_mut!(EARLY_BOOT_TRAP_FRAMES[cpu])
                );
            }
        }
    }

    /// - Stopping DMA (critical for safety)
    /// - Removing from subscriber lists (prevents stale wakes)
    /// - Notifying IPC peers (sends Close messages, wakes blocked receivers)
    ///
    /// Returns the peer info list for IPC peers to wake via ObjectService.
    fn do_ipc_cleanup(pid: TaskId) -> super::ipc::PeerInfoList {
        // Stop DMA first (critical for safety)
        super::bus::process_cleanup(pid);

        // Remove dying task from ALL port subscriber lists (prevent stale wakes)
        super::ipc::remove_subscriber_from_all(pid);

        // Notify IPC peers (sends Close messages)
        super::ipc::process_cleanup(pid)
    }

    /// Remove terminated tasks and free resources
    /// Uses two-phase cleanup to give servers a chance to release shared resources gracefully:
    /// - Exiting: Phase 1 - Send notifications (IPC Close, ShmemInvalid events), transition to Dying
    /// - Dying: Grace period (~100ms) for servers to react, then Phase 2
    /// - Dead: Task slot can be reused
    pub fn reap_terminated(&mut self, current_tick: u64) {
        // CRITICAL: Cleanup must be atomic - no preemption during dying cycle.
        // This is typically called from IRQ context (IRQs already disabled),
        // but we take IrqGuard defensively to guarantee atomicity.
        let _guard = crate::arch::aarch64::sync::IrqGuard::new();

        // CRITICAL: Never reap the current task - it's still executing!
        // If it needs to die, let it reach a safe point first (syscall return, reschedule).
        let current = current_slot();

        for slot_idx in 0..MAX_TASKS {
            // Skip current task - can't reap while still running
            if slot_idx == current {
                continue;
            }

            let state = self.tasks[slot_idx]
                .as_ref()
                .map(|t| (*t.state(), t.id))
                .unwrap_or((TaskState::Dead, 0));

            match state {
                (TaskState::Exiting { code: _ }, pid) => {
                    // ============================================================
                    // Phase 1: Notify servers about dying resources
                    // ============================================================

                    // Reparent children to init (PID 1) or kill them if init doesn't exist
                    // Collect children first to avoid borrow issues
                    let children_to_reparent: [TaskId; MAX_CHILDREN] = self.tasks[slot_idx]
                        .as_ref()
                        .map(|t| t.children)
                        .unwrap_or([0; MAX_CHILDREN]);

                    // Check if init (PID 1) exists before the mutable borrow
                    let has_init = self.slot_by_pid(1).is_some();

                    for child_pid in children_to_reparent {
                        if child_pid == 0 {
                            continue;
                        }
                        if let Some(child_slot) = self.slot_by_pid(child_pid) {
                            if let Some(ref mut child) = self.tasks[child_slot] {
                                // Reparent to init (PID 1) - devd
                                // If init doesn't exist (shouldn't happen), leave orphan with parent_id=0
                                if has_init {
                                    child.parent_id = 1;
                                } else {
                                    child.parent_id = 0;  // Orphan
                                }
                            }
                        }
                    }

                    // Wake parent task if it's sleeping (for Process watch)
                    self.wake_parent_if_sleeping(slot_idx);

                    // Perform IPC cleanup (DMA stop, subscriber removal, peer notification)
                    let ipc_peers = Self::do_ipc_cleanup(pid);
                    for peer in ipc_peers.iter() {
                        let wake_list = super::object_service::object_service()
                            .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::CLOSED);
                        super::ipc::waker::wake(&wake_list, super::ipc::WakeReason::Closed);
                    }

                    // Notify shmem mappers (marks regions as Dying, sends ShmemInvalid events)
                    super::shmem::begin_cleanup(pid);

                    // Transition to Dying with grace period deadline
                    let grace_until = current_tick + CLEANUP_GRACE_TICKS as u64;
                    if let Some(ref mut task) = self.tasks[slot_idx] {
                        crate::transition_or_evict!(task, set_dying, grace_until);
                    }
                }
                (TaskState::Dying { code: _, until }, pid) if current_tick >= until => {
                    // ============================================================
                    // Phase 2: Force cleanup and reap (grace period expired)
                    // ============================================================
                    Self::do_final_cleanup(pid);

                    if let Some(ref task) = self.tasks[slot_idx] {
                        Self::clear_stale_trap_frame(task, slot_idx, pid);
                    }

                    self.notify_exit(slot_idx);
                    self.bump_generation(slot_idx);
                    self.tasks[slot_idx] = None;
                }
                (TaskState::Dying { .. }, _) => {
                    // Still in grace period - let servers process notifications
                }
                (TaskState::Evicting { reason }, pid) => {
                    // ============================================================
                    // Evicted task: Immediate cleanup, no grace period
                    // ============================================================
                    kerror!("evict", "reaping";
                        pid = pid as u64,
                        reason = reason as u8 as u64
                    );

                    self.wake_parent_if_sleeping(slot_idx);

                    let ipc_peers = Self::do_ipc_cleanup(pid);
                    for peer in ipc_peers.iter() {
                        let wake_list = super::object_service::object_service()
                            .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::CLOSED);
                        super::ipc::waker::wake(&wake_list, super::ipc::WakeReason::Closed);
                    }

                    super::shmem::begin_cleanup(pid);
                    Self::do_final_cleanup(pid);

                    if let Some(ref mut task) = self.tasks[slot_idx] {
                        let _ = task.finalize(); // Evicting → Dead
                    }

                    if let Some(ref task) = self.tasks[slot_idx] {
                        Self::clear_stale_trap_frame(task, slot_idx, pid);
                    }

                    self.notify_exit(slot_idx);
                    self.bump_generation(slot_idx);
                    self.tasks[slot_idx] = None;
                }
                _ => {}
            }
        }
    }

    /// Prepare a task for running and return data needed for usermode entry.
    ///
    /// This extracts the trap frame pointer, ttbr0, and kernel stack top,
    /// setting up globals. The caller MUST release the scheduler lock before
    /// calling enter_usermode().
    ///
    /// # Returns
    /// Some((trap_frame_ptr, ttbr0, kernel_stack_top)) on success, None if task isn't valid.
    ///
    /// # Safety
    /// Task must be properly set up with valid trap frame and address space.
    pub unsafe fn prepare_user_task(&mut self, slot: usize) -> Option<(*const TrapFrame, u64, u64)> {
        // Set current task slot for this CPU
        set_current_slot(slot);

        let task = self.tasks[slot].as_mut()?;
        let cpu = super::percpu::cpu_id();

        // Task must be Ready to enter Running state
        if !crate::transition_or_evict!(task, set_running, cpu) {
            // Transition failed - task was evicted
            kerror!("task", "run_user_task_failed"; slot = slot as u64, pid = task.id as u64, state = task.state().name());

            // Reset to idle task - update per-CPU data directly since we already hold the lock
            set_current_slot(0);
            if let Some(ref mut idle) = self.tasks[0] {
                let trap_ptr = &mut idle.trap_frame as *mut TrapFrame;
                super::percpu::set_trap_frame(trap_ptr);
                if let Some(ref addr_space) = idle.address_space {
                    super::percpu::set_ttbr0(addr_space.get_ttbr0());
                }
            }
            return None;
        }

        let addr_space = task.address_space.as_ref()?;
        let ttbr0 = addr_space.get_ttbr0();
        let trap_frame = &mut task.trap_frame as *mut TrapFrame;

        // Kernel stack top (virtual) - becomes SP_EL1 for this task's exceptions
        let kstack_top = crate::arch::aarch64::mmu::phys_to_virt(
            task.kernel_stack + task.kernel_stack_size as u64
        );

        // Set per-CPU data for exception handler
        super::percpu::set_trap_frame(trap_frame);
        super::percpu::set_ttbr0(ttbr0);

        Some((trap_frame as *const TrapFrame, ttbr0, kstack_top))
    }

    /// Get the next deadline (for tickless timer reprogramming)
    pub fn get_next_deadline(&self) -> u64 {
        self.next_deadline
    }

    /// Select the next task to run using the current scheduling policy
    ///
    /// Delegates to `self.policy.select_next()` for the actual selection.
    /// The default policy is `PerCpuQueues` - priority levels with
    /// round-robin fairness within each level.
    pub fn schedule(&mut self) -> Option<usize> {
        let my_slot = current_slot();
        self.policy.select_next(my_slot, &self.tasks)
    }

    /// Get which CPU a task slot is assigned to
    pub fn task_cpu(&self, slot: usize) -> Option<u32> {
        self.policy.task_cpu(slot)
    }

    /// Update the number of CPUs for round-robin task distribution.
    pub fn set_num_cpus(&mut self, n: u32) {
        self.policy.set_num_cpus(n as u8);
    }

    /// Print scheduler state
    pub fn print_info(&self) {
        let current = current_slot();
        print_direct!("  Tasks:\n");
        for (i, slot) in self.tasks.iter().enumerate() {
            if let Some(ref task) = slot {
                let state_str = match *task.state() {
                    TaskState::Ready => "ready",
                    TaskState::Running { .. } => "RUNNING",
                    TaskState::Sleeping { .. } => "sleeping",
                    TaskState::Waiting { .. } => "waiting",
                    TaskState::Exiting { .. } => "exiting",
                    TaskState::Dying { .. } => "dying",
                    TaskState::Evicting { .. } => "evicting",
                    TaskState::Dead => "dead",
                };
                let marker = if i == current { ">" } else { " " };
                print_direct!("    {} [{}] {} ({})\n", marker, task.id, task.name_str(), state_str);
            }
        }
    }
}

use super::lock::SpinLock;

/// Global scheduler instance protected by SpinLock for SMP safety.
///
/// # Synchronization
///
/// Access is serialized via SpinLock which:
/// 1. Disables IRQs on the current CPU (prevents interrupt handler races)
/// 2. Uses atomic spinlock (prevents cross-CPU races on SMP)
///
/// # Three-Phase Scheduling
///
/// The lock MUST NOT be held across context_switch(). This is achieved via
/// three-phase scheduling in reschedule():
/// 1. Phase 1: Acquire lock, make decision, extract SwitchDecision, RELEASE LOCK
/// 2. Phase 2: Context switch with NO LOCK (only IRQs disabled)
/// 3. Phase 3: Reacquire lock for finalization
///
/// # Public API
///
/// Most code should use the public API functions which hide locking:
/// - `with_task()` / `with_task_mut()` - access a task by slot
/// - `reap_terminated()` - cleanup dead tasks
/// - `check_timeouts()` - wake timed-out tasks
/// - `current_task_id()` - get current task's PID
///
/// See docs/architecture/SCHEDULER_DESIGN_V2.md for details.
static SCHEDULER: SpinLock<Scheduler> = SpinLock::new(Scheduler::new());

/// Initialize scheduler (call once at boot)
/// This explicitly resets all task slots to None to handle static initialization issues
/// on some platforms (e.g., QEMU where .data section may not be properly initialized).
///
/// Also creates the idle task in slot 0 - this is an internal implementation detail.
pub fn init_scheduler() {
    // Clear all task slots (handles static init issues on some platforms)
    {
        let mut sched = SCHEDULER.lock();
        for i in 0..MAX_TASKS {
            sched.tasks[i] = None;
        }
        sched.next_deadline = u64::MAX;
    }

    // Initialize per-CPU current slot to idle (slot 0)
    set_current_slot(0);

    // Set early boot trap frame for this CPU (before any tasks exist)
    let cpu = super::percpu::cpu_id() as usize;
    unsafe {
        let early_frame = core::ptr::addr_of_mut!(EARLY_BOOT_TRAP_FRAMES[cpu]);
        super::percpu::set_trap_frame(early_frame);
    }

    // Create idle task in slot 0 - runs WFI loop when no tasks ready
    // Uses static stack (no PMM allocation) since PMM isn't initialized yet
    {
        let mut sched = SCHEDULER.lock();
        let idle_id = sched.make_pid(0);
        // SAFETY: idle_entry is a valid function pointer, idle_stack_top() returns valid stack
        let idle_task = unsafe { Task::new_idle(idle_id, crate::kernel::idle::idle_entry, 0) };
        sched.tasks[0] = Some(idle_task);
    }
}

/// Initialize scheduler for a secondary CPU.
///
/// Creates an idle task in slot `cpu` for the given CPU.
/// Must be called from the secondary CPU after percpu::init_secondary_cpu().
pub fn init_secondary_scheduler(cpu: u32) {
    // Set per-CPU current slot to this CPU's idle slot
    set_current_slot(cpu as usize);

    // Set early boot trap frame for this CPU
    unsafe {
        let early_frame = core::ptr::addr_of_mut!(EARLY_BOOT_TRAP_FRAMES[cpu as usize]);
        super::percpu::set_trap_frame(early_frame);
    }

    // Create idle task in slot `cpu`
    let mut sched = SCHEDULER.lock();
    let idle_id = sched.make_pid(cpu as usize);
    let idle_task = unsafe { Task::new_idle(idle_id, crate::kernel::idle::idle_entry, cpu) };
    sched.tasks[cpu as usize] = Some(idle_task);
}

// Per-CPU early boot trap frames - used before any tasks are created.
// Each CPU gets its own early boot trap frame.
static mut EARLY_BOOT_TRAP_FRAMES: [TrapFrame; super::percpu::MAX_CPUS] = [
    TrapFrame::new(),
    TrapFrame::new(),
    TrapFrame::new(),
    TrapFrame::new(),
];

// NOTE: CURRENT_TRAP_FRAME, CURRENT_TTBR0, and SYSCALL_SWITCHED_TASK have been
// moved to per-CPU CpuData fields (accessed via percpu::set_trap_frame() etc.)
// for SMP safety. See percpu.rs for the new API.

/// Get exclusive access to the scheduler.
///
/// Returns a guard that holds the SpinLock. The lock is released when
/// the guard is dropped.
///
/// # Warning
/// NEVER hold the guard across context_switch()! Use the three-phase
/// pattern in reschedule() instead.
///
/// # Usage
/// ```
/// let mut sched = scheduler();
/// sched.wake_task(slot);
/// // Guard dropped, lock released
/// ```
///
/// # Warning
///
/// This function is `pub(crate)` to prevent external code from holding the
/// lock across function calls, which can cause deadlocks. External code should
/// use `with_scheduler()` or `try_with_scheduler()` instead.
#[inline]
pub(crate) fn scheduler() -> super::lock::SpinLockGuard<'static, Scheduler> {
    SCHEDULER.lock()
}

/// Try to get exclusive access to the scheduler without blocking.
///
/// Returns None if the scheduler lock is already held.
/// Useful for preventing deadlock when logging inside scheduler operations.
///
/// # Warning
///
/// This function is `pub(crate)` to prevent external code from holding the
/// lock across function calls. External code should use `try_with_scheduler()`.
#[inline]
pub(crate) fn try_scheduler() -> Option<super::lock::SpinLockGuard<'static, Scheduler>> {
    SCHEDULER.try_lock()
}

/// Execute a closure with exclusive access to the scheduler.
///
/// This is the preferred pattern for most operations:
/// ```
/// with_scheduler(|sched| {
///     sched.wake_task(slot);
/// });
/// ```
///
/// The SpinLock is acquired before calling the closure and released after.
/// IRQs are disabled for the duration (SpinLock does this automatically).
///
/// # Deadlock Prevention
///
/// The closure-based API ensures the lock is always released when the closure
/// returns. This prevents the common bug of holding the scheduler lock while
/// calling into other subsystems (uaccess, IPC, etc.) that might also need it.
#[inline]
pub fn with_scheduler<R, F: FnOnce(&mut Scheduler) -> R>(f: F) -> R {
    let mut guard = SCHEDULER.lock();
    f(&mut *guard)
}

/// Try to execute a closure with scheduler access, returning None if lock is held.
///
/// This is useful when:
/// - Called from code that might already hold the scheduler lock
/// - Called from IRQ context where blocking is not acceptable
/// - Need to fall back to a lock-free alternative (e.g., deferred wake queue)
///
/// # Example
/// ```
/// if let Some(result) = try_with_scheduler(|sched| {
///     sched.wake_by_pid(pid)
/// }) {
///     // Lock acquired, operation completed
/// } else {
///     // Lock was held, use fallback (e.g., request_wake)
///     cpu_flags().request_wake(pid);
/// }
/// ```
#[inline]
pub fn try_with_scheduler<R, F: FnOnce(&mut Scheduler) -> R>(f: F) -> Option<R> {
    SCHEDULER.try_lock().map(|mut guard| f(&mut *guard))
}

// ============================================================================
// Public API - Locking Hidden Inside
// ============================================================================

/// Process any pending wake requests queued by IRQ handlers.
///
/// Must be called from a safe point where the scheduler lock is NOT held.
/// This is typically called from `irq_exit_resched()` after the interrupt
/// handler returns but before returning to the interrupted code.
///
/// IRQ handlers use `cpu_flags().request_wake(pid)` to queue wakes because
/// they cannot safely acquire the scheduler lock (it may be held by the
/// interrupted code, causing deadlock). This function processes those
/// queued requests at a point where it's safe to acquire the lock.
pub fn process_pending_wakes() {
    let pending = crate::arch::aarch64::sync::cpu_flags().drain_pending_wakes();
    let mut any_woken = false;

    for pid in pending {
        if pid != 0 {
            with_scheduler(|sched| {
                if sched.wake_by_pid(pid) {
                    any_woken = true;
                }
            });
        }
    }

    // Ensure need_resched is set if we woke anyone
    if any_woken {
        crate::arch::aarch64::sync::cpu_flags().set_need_resched();
    }
}

/// Access a task by slot (read-only).
///
/// Acquires scheduler lock, calls closure with task reference, releases lock.
/// Returns None if slot is empty.
#[inline]
pub fn with_task<R, F: FnOnce(&Task) -> R>(slot: usize, f: F) -> Option<R> {
    let sched = SCHEDULER.lock();
    sched.tasks[slot].as_ref().map(f)
}

/// Access a task by slot (mutable).
///
/// Acquires scheduler lock, calls closure with task reference, releases lock.
/// Returns None if slot is empty.
#[inline]
pub fn with_task_mut<R, F: FnOnce(&mut Task) -> R>(slot: usize, f: F) -> Option<R> {
    let mut sched = SCHEDULER.lock();
    sched.tasks[slot].as_mut().map(f)
}

/// Get the current task's PID.
#[inline]
pub fn current_task_id() -> Option<TaskId> {
    let sched = SCHEDULER.lock();
    sched.tasks[current_slot()].as_ref().map(|t| t.id)
}

/// Remove terminated tasks and free resources.
///
/// Called from timer interrupt to clean up dead tasks.
#[inline]
pub fn reap_terminated(current_tick: u64) {
    let mut sched = SCHEDULER.lock();
    sched.reap_terminated(current_tick);
}

/// Check for timed-out tasks and wake them.
///
/// Returns the number of tasks woken.
#[inline]
pub fn check_timeouts(current_time: u64) -> usize {
    let mut sched = SCHEDULER.lock();
    sched.check_timeouts(current_time)
}

/// Record a deadline for the next wake time.
#[inline]
pub fn note_deadline(deadline: u64) {
    let mut sched = SCHEDULER.lock();
    sched.note_deadline(deadline);
}

/// Spawn a new user task.
///
/// Returns (pid, slot) on success, None if no free slots.
#[inline]
pub fn spawn_user_task(name: &str) -> Option<(TaskId, usize)> {
    let mut sched = SCHEDULER.lock();
    sched.add_user_task(name)
}

// ============================================================================
// Updating Task Globals - Pattern Documentation
// ============================================================================
//
// When switching tasks, update CURRENT_TRAP_FRAME and CURRENT_TTBR0 while
// STILL HOLDING the scheduler lock. Do NOT call a helper function that
// re-acquires the lock (this causes deadlock since SpinLock is not reentrant).
//
// ## Correct Pattern
//
// ```rust
// with_scheduler(|sched| {
//     // ... scheduling decision ...
//     task::set_current_slot(next_slot);
//
//     // Update globals while still holding lock:
//     if let Some(next) = sched.task_mut(next_slot) {
//         crate::transition_or_evict!(next, set_running);
//         let trap_ptr = &mut next.trap_frame as *mut task::TrapFrame;
//         task::CURRENT_TRAP_FRAME.store(trap_ptr, core::sync::atomic::Ordering::Release);
//         if let Some(ref addr_space) = next.address_space {
//             task::CURRENT_TTBR0.store(addr_space.get_ttbr0(), core::sync::atomic::Ordering::Release);
//         }
//     }
// });
// ```
//
// ## Why Not a Helper Function?
//
// A helper function that acquires SCHEDULER.lock() internally will deadlock
// if called from inside with_scheduler(). The closure-based API ensures the
// lock is always released, but only if you don't call functions that
// re-acquire it.
// ============================================================================

/// Check NEED_RESCHED flag and perform reschedule if needed.
/// Called at safe points (syscall exit, exception return).
///
/// This is the "bottom half" of preemption - the timer IRQ just sets a flag,
/// and this function does the actual work.
///
/// Also reschedules if current task is Blocked (e.g., after blocking syscall).
///
/// This function ONLY handles scheduling - no logging or other side effects.
/// Log flushing and other deferred work should be done separately.
///
/// # Safety
/// Must be called from kernel context (not IRQ context).
#[no_mangle]
pub unsafe extern "C" fn do_resched_if_needed() {
    // First, process any pending wake requests from IRQ context.
    // This is critical because IRQ handlers can't acquire the scheduler lock
    // directly (to avoid deadlock), so they queue wakes via request_wake().
    // We must process these before checking need_resched.
    process_pending_wakes();

    // Process any pending task evictions.
    // This is critical for storm detection: when a task is marked for eviction
    // during syscall handling, we must process it before returning to userspace.
    // Otherwise the task keeps making syscalls and hitting storm detection again.
    eviction::process_pending_evictions();

    // Check and clear the flag atomically (before taking lock)
    let need_resched = crate::arch::aarch64::sync::cpu_flags().check_and_clear_resched();

    // Check if we need to reschedule (must hold IrqGuard for SMP safety)
    let (should_resched, blocked_pid) = if need_resched {
        (true, 0)
    } else {
        // Check if current task is blocked or terminated - requires lock for safe access
        with_scheduler(|sched| {
            let my_slot = current_slot();
            match sched.tasks[my_slot].as_ref() {
                Some(t) if t.is_blocked() => (true, t.id),
                // Terminated tasks (including evicted) must yield CPU
                Some(t) if t.is_terminated() => (true, t.id),
                _ => (false, 0),
            }
        })
    };

    if !should_resched {
        return;
    }

    // Debug: log when we're about to reschedule due to blocked task
    if blocked_pid != 0 {
        crate::kdebug!("resched", "blocked_resched"; pid = blocked_pid as u64);
    }

    // Delegate to sched::reschedule() which properly handles:
    // - Variable-based switching for user->user preemption
    // - Context switching for kernel tasks and blocked tasks
    crate::kernel::sched::reschedule();
}

// NOTE: yield_cpu() and yield_cpu_locked() have been removed.
// They used context_switch() which is for kernel-to-kernel switching,
// not user tasks. For user task scheduling, use:
// - sched::yield_current() - kernel-internal yield
// - sched::reschedule() - kernel-internal reschedule
// - sys_yield() in syscall.rs - syscall entry point

/// Test context switching
pub fn test() {
    print_direct!("  Context switch test:\n");
    print_direct!("    TrapFrame size: {} bytes\n", core::mem::size_of::<TrapFrame>());
    print_direct!("    CpuContext size: {} bytes\n", core::mem::size_of::<CpuContext>());
    print_direct!("    Task size: {} bytes\n", core::mem::size_of::<Task>());
    print_direct!("    [OK] Structures initialized\n");
}
