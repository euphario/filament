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
//!              │  Task   │       │  Task   │  ...
//!              │ slot 0  │       │ slot 1  │
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
pub use policy::{SchedulingPolicy, PriorityRoundRobin};

// Re-export TCB types
pub use tcb::{
    Priority, TrapFrame, CpuContext, TaskId, Task,
    MAX_CHILDREN, MAX_CHANNELS_PER_TASK, CLEANUP_GRACE_TICKS,
    enter_usermode, context_switch,
};

use crate::{kinfo, kerror, print_direct, klog};

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

/// Maximum number of tasks
pub const MAX_TASKS: usize = 16;

/// Scheduler state
///
/// The scheduler manages all tasks and their state. For SMP safety:
/// - Task array modifications are protected by IrqGuard (single-core) or SpinLock (SMP)
/// - Current task index is stored per-CPU via percpu module (use `current_slot()`)
///
/// # Scheduling Policy
///
/// The scheduler uses a pluggable `SchedulingPolicy` trait for task selection.
/// The default policy is `PriorityRoundRobin` - priority levels with round-robin
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
    policy: PriorityRoundRobin,
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
            policy: PriorityRoundRobin::new(),
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

                // Check all task timers
                for timer in &task.timers {
                    if timer.is_active() && timer.deadline < soonest {
                        soonest = timer.deadline;
                    }
                }
            }
        }

        self.next_deadline = soonest;
    }

    /// Generate a PID for a given slot
    /// PID format: bits[7:0] = slot + 1 (1-16), bits[31:8] = generation (24 bits)
    ///
    /// The generation is masked to 24 bits to prevent overflow when shifted.
    /// After 2^24 task creations in a slot, generation wraps to 0, which is
    /// acceptable since the probability of a stale reference surviving that
    /// long is negligible.
    fn make_pid(&self, slot: usize) -> TaskId {
        let slot_bits = (slot + 1) as u32;  // +1 to avoid PID 0
        let gen = self.generations[slot] & 0xFF_FFFF;  // Mask to 24 bits
        let gen_bits = gen << 8;
        slot_bits | gen_bits
    }

    /// Extract slot from PID (returns None if invalid)
    fn slot_from_pid(pid: TaskId) -> Option<usize> {
        let slot_bits = (pid & 0xFF) as usize;
        if slot_bits == 0 || slot_bits > MAX_TASKS {
            return None;
        }
        Some(slot_bits - 1)
    }

    /// Extract generation from PID
    /// Used for subscriber tracking to detect stale references
    pub fn generation_from_pid(pid: TaskId) -> u32 {
        (pid >> 8) & 0xFF_FFFF
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

        // Collect TimerObject subscribers to wake (from Mux watches)
        use crate::kernel::ipc::traits::Subscriber;
        let mut timer_subscribers: [Subscriber; 32] = [Subscriber { task_id: 0, generation: 0 }; 32];
        let mut timer_sub_count = 0;

        for task_opt in self.tasks.iter_mut() {
            if let Some(ref mut task) = task_opt {
                // Skip terminated tasks - their timers should not fire
                if task.is_terminated() {
                    continue;
                }

                // Check waiting deadline (for request-response patterns)
                // Sleeping tasks have no deadline - only Waiting tasks do
                if let TaskState::Waiting { deadline, .. } = *task.state() {
                    if current_tick >= deadline {
                        // Deadline expired - wake the task
                        // Note: Don't reset liveness state for timeout - task didn't respond
                        crate::kdebug!("sched", "deadline_wake"; pid = task.id as u64);
                        crate::transition_or_log!(task, wake);
                        woken += 1;
                        need_recalculate = true;
                    }
                }

                // Check all timers for this task (BSD kqueue style)
                // Multiple timers can fire per tick, each generates a separate event
                let mut should_wake = false;
                for timer in task.timers.iter_mut() {
                    if timer.is_active() && current_tick >= timer.deadline {
                        // Timer fired - deliver event with timer ID
                        let timer_event = super::event::Event::timer_with_id(timer.id, timer.deadline);
                        if !task.event_queue.push(timer_event) {
                            // Queue full - log for diagnostics (timer still fires, event lost)
                            crate::kwarn!("timer", "event_dropped";
                                pid = task.id as u64,
                                timer_id = timer.id as u64
                            );
                        }

                        if timer.interval > 0 {
                            // Recurring timer - reset deadline
                            // Use saturating_add to prevent overflow on very long uptime
                            timer.deadline = timer.deadline.saturating_add(timer.interval);
                        } else {
                            // One-shot timer - clear it
                            timer.deadline = 0;
                        }

                        should_wake = true;
                        need_recalculate = true;
                    }
                }

                // Wake task if any timer fired and task was blocked
                if should_wake && task.is_blocked() {
                    crate::transition_or_log!(task, wake);
                    // Reset liveness - timer event means task is active
                    task.liveness_state = super::liveness::LivenessState::Normal;
                    woken += 1;
                }

                // Collect task ID for TimerObject checking via ObjectService
                if timer_sub_count < timer_subscribers.len() {
                    // Store task_id in the subscriber array (repurposed temporarily)
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

        // Check TimerObjects via ObjectService (outside task iteration)
        // Collect all subscribers to wake
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
            // Find and wake the subscriber's task
            if let Some(slot) = self.slot_by_pid(subscriber.task_id) {
                if let Some(ref mut task) = self.tasks[slot] {
                    // Wake if task is blocked
                    if task.is_blocked() {
                        crate::transition_or_log!(task, wake);
                        // Reset liveness - timer event means task is active
                        task.liveness_state.reset(task.id);
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
        // Find empty slot
        let slot = self.tasks.iter().position(|t| t.is_none())?;

        // Generate PID with current generation for this slot
        let id = self.make_pid(slot);

        self.tasks[slot] = Task::new_kernel(id, entry, name);
        if self.tasks[slot].is_some() {
            // Create parallel object table in ObjectService (Phase 1: not used yet)
            super::object_service::object_service().create_task_table(id, false);
            Some(id)
        } else {
            None
        }
    }

    /// Add a user task to the scheduler
    pub fn add_user_task(&mut self, name: &str) -> Option<(TaskId, usize)> {
        let slot = self.tasks.iter().position(|t| t.is_none())?;

        // Generate PID with current generation for this slot
        let id = self.make_pid(slot);

        self.tasks[slot] = Task::new_user(id, name);
        if self.tasks[slot].is_some() {
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
            // Clear timers so they don't fire for terminated task
            for timer in task.timers.iter_mut() {
                timer.deadline = 0;
                timer.interval = 0;
            }
            // Clear is_init flag so new devd can be the init
            task.is_init = false;
        }
    }

    /// Perform IPC cleanup for a terminating task.
    ///
    /// This handles:
    /// - Stopping DMA (critical for safety)
    /// - Removing from subscriber lists (prevents stale wakes)
    /// - Notifying IPC peers (sends Close messages, wakes blocked receivers)
    ///
    /// Returns the wake list for IPC peers.
    fn do_ipc_cleanup(pid: TaskId) -> super::ipc::waker::WakeList {
        // Stop DMA first (critical for safety)
        super::bus::process_cleanup(pid);

        // Remove dying task from ALL subscriber lists (prevent stale wakes)
        super::ipc::remove_subscriber_from_all(pid);

        // Notify IPC peers (sends Close messages, wakes blocked receivers)
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
                    let ipc_wake_list = Self::do_ipc_cleanup(pid);
                    super::ipc::waker::wake(&ipc_wake_list, super::ipc::WakeReason::Closed);

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
                    // Finalize shmem (force unmap from any remaining mappers, free memory)
                    super::shmem::finalize_cleanup(pid);

                    // Clean up remaining subsystems
                    super::irq::process_cleanup(pid);
                    super::pci::release_all_devices(pid);
                    // Clean up ports owned by this task
                    let port_wake_list = super::ipc::port_cleanup_task(pid);
                    super::ipc::waker::wake(&port_wake_list, super::ipc::WakeReason::Closed);

                    // Remove object table from ObjectService (Phase 1: parallel structure)
                    super::object_service::object_service().remove_task_table(pid);

                    // SAFETY: Clear CURRENT_TRAP_FRAME if it points to this task.
                    // This shouldn't happen (we skip current task), but prevents use-after-free.
                    if let Some(ref task) = self.tasks[slot_idx] {
                        let task_trap_ptr = &task.trap_frame as *const TrapFrame;
                        let current_ptr = CURRENT_TRAP_FRAME.load(Ordering::Acquire);
                        if current_ptr == task_trap_ptr as *mut TrapFrame {
                            crate::kwarn!("task", "clearing_stale_trap_frame"; slot = slot_idx as u64, pid = pid as u64);
                            CURRENT_TRAP_FRAME.store(
                                unsafe { core::ptr::addr_of_mut!(EARLY_BOOT_TRAP_FRAME) },
                                Ordering::Release
                            );
                        }
                    }

                    // Bump generation so any stale PIDs for this slot become invalid
                    self.bump_generation(slot_idx);
                    // Drop the task (frees kernel stack and heap mappings)
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

                    // Wake parent task if it's sleeping (for Process watch)
                    self.wake_parent_if_sleeping(slot_idx);

                    // Perform IPC cleanup (DMA stop, subscriber removal, peer notification)
                    let ipc_wake_list = Self::do_ipc_cleanup(pid);
                    super::ipc::waker::wake(&ipc_wake_list, super::ipc::WakeReason::Closed);

                    // Finalize shmem immediately (no grace period)
                    super::shmem::begin_cleanup(pid);
                    super::shmem::finalize_cleanup(pid);

                    // Clean up remaining subsystems
                    super::irq::process_cleanup(pid);
                    super::pci::release_all_devices(pid);

                    // Clean up ports
                    let port_wake_list = super::ipc::port_cleanup_task(pid);
                    super::ipc::waker::wake(&port_wake_list, super::ipc::WakeReason::Closed);

                    // Remove object table
                    super::object_service::object_service().remove_task_table(pid);

                    // Transition to Dead via state machine
                    if let Some(ref mut task) = self.tasks[slot_idx] {
                        let _ = task.finalize(); // Evicting → Dead
                    }

                    // SAFETY: Clear CURRENT_TRAP_FRAME if it points to this task.
                    // This shouldn't happen (we skip current task), but prevents use-after-free.
                    if let Some(ref task) = self.tasks[slot_idx] {
                        let task_trap_ptr = &task.trap_frame as *const TrapFrame;
                        let current_ptr = CURRENT_TRAP_FRAME.load(Ordering::Acquire);
                        if current_ptr == task_trap_ptr as *mut TrapFrame {
                            crate::kwarn!("task", "clearing_stale_trap_frame"; slot = slot_idx as u64, pid = pid as u64);
                            CURRENT_TRAP_FRAME.store(
                                unsafe { core::ptr::addr_of_mut!(EARLY_BOOT_TRAP_FRAME) },
                                Ordering::Release
                            );
                        }
                    }

                    // Bump generation and remove task
                    self.bump_generation(slot_idx);
                    self.tasks[slot_idx] = None;
                }
                _ => {}
            }
        }
    }

    /// Run a specific user task (enter user mode)
    /// This never returns to the caller - it erets to user mode
    /// # Safety
    /// Task must be properly set up with valid trap frame and address space
    pub unsafe fn run_user_task(&mut self, slot: usize) -> ! {
        // This should only be called ONCE at boot from kernel_main
        kinfo!("task", "run_user_task_entry"; slot = slot as u64);

        // Set current task slot for this CPU
        set_current_slot(slot);

        if let Some(ref mut task) = self.tasks[slot] {
            // Task must be Ready to enter Running state
            // If task is in another state (Waiting, Sleeping, etc), we can't run it
            if !crate::transition_or_evict!(task, set_running) {
                // Transition failed - task was evicted
                // This should NEVER happen at boot - devd should be Ready
                kerror!("task", "run_user_task_failed"; slot = slot as u64, pid = task.id as u64, state = task.state().name());

                // Reset to idle task before entering idle loop
                set_current_slot(0);  // Idle is always slot 0
                update_current_task_globals();

                // Enter idle - will wait for IRQ to wake a task
                crate::kernel::idle::idle_entry();
            }

            if let Some(ref addr_space) = task.address_space {
                let ttbr0 = addr_space.get_ttbr0();
                let trap_frame = &mut task.trap_frame as *mut TrapFrame;

                // Set globals for exception handler (atomic for SMP safety)
                CURRENT_TRAP_FRAME.store(trap_frame, Ordering::Release);
                CURRENT_TTBR0.store(ttbr0, Ordering::Release);

                kinfo!("task", "enter_user";
                    entry = klog::hex64(task.trap_frame.elr_el1),
                    stack = klog::hex64(task.trap_frame.sp_el0),
                    ttbr0 = klog::hex64(ttbr0)
                );

                // Flush log buffer before entering userspace
                crate::klog::flush();

                // Start timer preemption just before entering userspace.
                // We can't start this earlier in boot because timer IRQs
                // would trigger rescheduling before run_user_task is called,
                // corrupting the boot flow.
                crate::plat::timer::start(10);
                kinfo!("timer", "preemption_started"; slice_ms = 10u64);

                enter_usermode(trap_frame as *const TrapFrame, ttbr0);
            }
        }

        // Should never reach here
        panic!("run_user_task: invalid task or no address space");
    }

    /// Get the next deadline (for tickless timer reprogramming)
    pub fn get_next_deadline(&self) -> u64 {
        self.next_deadline
    }

    /// Select the next task to run using the current scheduling policy
    ///
    /// Delegates to `self.policy.select_next()` for the actual selection.
    /// The default policy is `PriorityRoundRobin` - priority levels with
    /// round-robin fairness within each level.
    pub fn schedule(&mut self) -> Option<usize> {
        let my_slot = current_slot();
        self.policy.select_next(my_slot, &self.tasks)
    }

    /// Print scheduler state
    pub fn print_info(&self) {
        let current = current_slot();
        print_direct!("  Tasks:\n");
        for (i, slot) in self.tasks.iter().enumerate() {
            if let Some(ref task) = slot {
                let state_str = match *task.state() {
                    TaskState::Ready => "ready",
                    TaskState::Running => "RUNNING",
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

use core::sync::atomic::{AtomicU64, AtomicPtr, Ordering};
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

    // Create idle task in slot 0 - runs WFI loop when no tasks ready
    // Uses static stack (no PMM allocation) since PMM isn't initialized yet
    {
        let mut sched = SCHEDULER.lock();
        let idle_id = sched.make_pid(0);
        // SAFETY: idle_entry is a valid function pointer, idle_stack_top() returns valid stack
        let idle_task = unsafe { Task::new_idle(idle_id, crate::kernel::idle::idle_entry) };
        sched.tasks[0] = Some(idle_task);
    }
}

/// Early boot trap frame - used before any tasks are created
/// This ensures exception handlers have a valid place to save state
/// even if an exception occurs during very early boot.
static mut EARLY_BOOT_TRAP_FRAME: TrapFrame = TrapFrame::new();

/// Current task's trap frame pointer - used by exception handler
/// Atomic to ensure safe access from multiple CPUs (SMP safety)
/// Initialized to early boot trap frame to handle exceptions before tasks exist.
#[no_mangle]
pub static CURRENT_TRAP_FRAME: AtomicPtr<TrapFrame> = AtomicPtr::new(
    unsafe { core::ptr::addr_of_mut!(EARLY_BOOT_TRAP_FRAME) }
);

/// Current task's TTBR0 value - used by exception handler
/// Atomic to ensure safe access from multiple CPUs (SMP safety)
#[no_mangle]
pub static CURRENT_TTBR0: AtomicU64 = AtomicU64::new(0);

/// Flag: set to 1 when syscall switched tasks, 0 otherwise
/// Assembly checks this to skip storing return value when switched
/// Atomic to ensure safe access from multiple CPUs (SMP safety)
#[no_mangle]
pub static SYSCALL_SWITCHED_TASK: AtomicU64 = AtomicU64::new(0);

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
#[inline]
pub fn scheduler() -> super::lock::SpinLockGuard<'static, Scheduler> {
    SCHEDULER.lock()
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
#[inline]
pub fn with_scheduler<R, F: FnOnce(&mut Scheduler) -> R>(f: F) -> R {
    let mut guard = SCHEDULER.lock();
    f(&mut *guard)
}

// ============================================================================
// Public API - Locking Hidden Inside
// ============================================================================

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

/// Update the current task globals after scheduling decision
///
/// # Safety
/// Must be called with valid current task.
///
/// # Note
/// This function acquires the scheduler lock. It should NOT be called
/// while already holding the lock. For three-phase reschedule, extract
/// the needed data (trap_frame ptr, ttbr0) upfront in Phase 1.
pub unsafe fn update_current_task_globals() {
    let mut sched = SCHEDULER.lock();
    let slot = current_slot();

    // Defensive check: validate current slot is in bounds
    if slot >= MAX_TASKS {
        crate::platform::current::uart::print("[PANIC] update_current_task_globals: current slot out of bounds!\r\n");
        loop { core::arch::asm!("wfe"); }
    }

    if let Some(ref mut task) = sched.tasks[slot] {
        let trap_ptr = &mut task.trap_frame as *mut TrapFrame;
        let trap_addr = trap_ptr as u64;

        // Defensive check: trap frame should be in kernel space (0xFFFF...)
        if trap_addr < 0xFFFF_0000_0000_0000 {
            crate::platform::current::uart::print("[PANIC] update_current_task_globals: trap_frame not in kernel space!\r\n");
            loop { core::arch::asm!("wfe"); }
        }

        CURRENT_TRAP_FRAME.store(trap_ptr, Ordering::Release);

        if let Some(ref addr_space) = task.address_space {
            let ttbr0 = addr_space.get_ttbr0();
            // Extract physical address (bits [47:0], mask out ASID in bits [63:48])
            let ttbr0_phys = ttbr0 & 0x0000_FFFF_FFFF_FFFF;

            // Defensive check: physical address should be valid DRAM (>= 0x40000000)
            // and properly aligned (4KB page table)
            if ttbr0_phys < 0x4000_0000 || ttbr0_phys >= 0x1_0000_0000 || (ttbr0_phys & 0xFFF) != 0 {
                crate::platform::current::uart::print("[PANIC] update_current_task_globals: invalid TTBR0=0x");
                // Print hex value
                for i in (0..16).rev() {
                    let nibble = ((ttbr0 >> (i * 4)) & 0xf) as u8;
                    let c = if nibble < 10 { b'0' + nibble } else { b'a' + nibble - 10 };
                    crate::platform::current::uart::putc(c as char);
                }
                crate::platform::current::uart::print("\r\n");
                loop { core::arch::asm!("wfe"); }
            }

            CURRENT_TTBR0.store(ttbr0, Ordering::Release);
        }
        // Note: Kernel tasks (no address_space) keep existing TTBR0
    } else {
        // No task at current slot - this is also a bug
        crate::platform::current::uart::print("[PANIC] update_current_task_globals: no task at current slot!\r\n");
        loop { core::arch::asm!("wfe"); }
    }
}

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
    // Check and clear the flag atomically (before taking lock)
    let need_resched = crate::arch::aarch64::sync::cpu_flags().check_and_clear_resched();

    // Check if we need to reschedule (must hold IrqGuard for SMP safety)
    let should_resched = if need_resched {
        true
    } else {
        // Check if current task is blocked - requires lock for safe access
        with_scheduler(|sched| {
            let my_slot = current_slot();
            sched.tasks[my_slot]
                .as_ref()
                .map(|t| t.is_blocked())
                .unwrap_or(false)
        })
    };

    if !should_resched {
        return;
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
