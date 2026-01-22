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

// Re-export core types from state module
pub use state::{TaskState, SleepReason, WaitReason};

// Re-export scheduling policy trait and implementations
pub use policy::{SchedulingPolicy, PriorityRoundRobin};

// Re-export TCB types
pub use tcb::{
    Priority, TrapFrame, CpuContext, TaskId, Task,
    MAX_CHILDREN, MAX_CHANNELS_PER_TASK, CLEANUP_GRACE_TICKS,
    enter_usermode, context_switch,
};

use crate::{kinfo, print_direct, klog};

/// Maximum number of tasks
pub const MAX_TASKS: usize = 16;

/// Scheduler state
///
/// The scheduler manages all tasks and their state. For SMP safety:
/// - Task array modifications are protected by IrqGuard (single-core) or SpinLock (SMP)
/// - Current task index is stored per-CPU via percpu module
/// - The `current` field is deprecated - use `current_slot()` function instead
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
    /// Currently running task index - DEPRECATED, use current_slot()
    /// Kept for backwards compatibility during migration
    current: usize,
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
            current: 0,
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
    /// 3. IPC return value is set correctly if needed
    ///
    /// Returns true if task was actually woken (was blocked).
    pub fn wake_task(&mut self, slot: usize) -> bool {
        if let Some(ref mut task) = self.tasks[slot] {
            // Only wake if task is blocked
            if !task.is_blocked() {
                return false;
            }

            // Check if task was blocked on IPC
            let is_ipc = match task.state() {
                TaskState::Sleeping { reason: SleepReason::Ipc } => true,
                TaskState::Waiting { reason: WaitReason::IpcCall, .. } => true,
                _ => false,
            };

            // If task was blocked on IPC, set x0 to EAGAIN (-11)
            // so it retries the receive and gets the message.
            // This is needed because receive_timeout pre-sets x0 to
            // ETIMEDOUT (-110) before blocking.
            if is_ipc {
                task.trap_frame.x0 = (-11i64) as u64; // EAGAIN
            }

            // Transition to Ready via state machine
            let _ = task.wake();

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
                        let _ = task.wake();
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
                        let _pushed = task.event_queue.push(timer_event);

                        if timer.interval > 0 {
                            // Recurring timer - reset deadline
                            // Avoid drift by adding interval to previous deadline
                            timer.deadline += timer.interval;
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
                    let _ = task.wake();
                    // Reset liveness - timer event means task is active
                    task.liveness_state = super::liveness::LivenessState::Normal;
                    woken += 1;
                }

                // Also check TimerObjects in object_table (for Mux subscribers)
                for entry in task.object_table.entries_mut() {
                    if let crate::kernel::object::Object::Timer(ref mut t) = entry.object {
                        // Check if timer has fired and has a subscriber
                        if t.check(current_tick) {
                            if let Some(subscriber) = t.subscriber() {
                                // Collect subscriber to wake
                                timer_subscribers[timer_sub_count] = subscriber;
                                timer_sub_count += 1;
                                if timer_sub_count >= timer_subscribers.len() {
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        // Wake all collected TimerObject subscribers (outside scheduler lock ideally)
        for i in 0..timer_sub_count {
            let subscriber = timer_subscribers[i];
            // Find and wake the subscriber's task
            if let Some(slot) = self.slot_by_pid(subscriber.task_id) {
                if let Some(ref mut task) = self.tasks[slot] {
                    // Wake if task is blocked (generation 0 = always valid, skip check)
                    if task.is_blocked() {
                        let _ = task.wake();
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
            Some((id, slot))
        } else {
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

    /// Check if any task besides `exclude_slot` is ready to run.
    ///
    /// Used by blocking operations to decide whether to yield or spin.
    /// Returns `true` if there's useful work to switch to.
    pub fn has_ready_task_besides(&self, exclude_slot: usize) -> bool {
        self.tasks.iter().enumerate().any(|(i, slot)| {
            if i == exclude_slot { return false; }
            if let Some(ref task) = slot {
                *task.state() == TaskState::Ready
            } else {
                false
            }
        })
    }

    /// Check if a slot is occupied by a task.
    ///
    /// Note: Task may be in any state including Dead/Dying.
    pub fn slot_occupied(&self, slot: usize) -> bool {
        self.tasks.get(slot).map(|t| t.is_some()).unwrap_or(false)
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
            let _ = task.set_exiting(exit_code);
            // Clear timers so they don't fire for terminated task
            for timer in task.timers.iter_mut() {
                timer.deadline = 0;
                timer.interval = 0;
            }
            // Clear is_init flag so new devd can be the init
            task.is_init = false;
        }
    }

    /// Remove terminated tasks and free resources
    /// Uses two-phase cleanup to give servers a chance to release shared resources gracefully:
    /// - Exiting: Phase 1 - Send notifications (IPC Close, ShmemInvalid events), transition to Dying
    /// - Dying: Grace period (~100ms) for servers to react, then Phase 2
    /// - Dead: Task slot can be reused
    pub fn reap_terminated(&mut self, current_tick: u64) {
        for slot_idx in 0..MAX_TASKS {
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
                    // This allows EventLoop to poll ProcessObjects and see the exit
                    if let Some(ref task) = self.tasks[slot_idx] {
                        let parent_id = task.parent_id;
                        if parent_id != 0 {
                            if let Some(parent_slot) = self.slot_by_pid(parent_id) {
                                if let Some(ref mut parent) = self.tasks[parent_slot] {
                                    if parent.state().is_sleeping() {
                                        let _ = parent.wake();
                                    }
                                }
                            }
                        }
                    }

                    // Stop DMA first (critical for safety)
                    super::bus::process_cleanup(pid);

                    // Remove dying task from ALL subscriber lists (prevent stale wakes)
                    super::ipc::remove_subscriber_from_all(pid);

                    // Notify IPC peers (sends Close messages, wakes blocked receivers)
                    let ipc_wake_list = super::ipc::process_cleanup(pid);
                    super::ipc::waker::wake(&ipc_wake_list, super::ipc::WakeReason::Closed);

                    // Notify shmem mappers (marks regions as Dying, sends ShmemInvalid events)
                    super::shmem::begin_cleanup(pid);

                    // Transition to Dying with grace period deadline
                    let grace_until = current_tick + CLEANUP_GRACE_TICKS as u64;
                    if let Some(ref mut task) = self.tasks[slot_idx] {
                        let _ = task.set_dying(grace_until);
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

                    // Bump generation so any stale PIDs for this slot become invalid
                    self.bump_generation(slot_idx);
                    // Drop the task (frees kernel stack and heap mappings)
                    self.tasks[slot_idx] = None;
                }
                (TaskState::Dying { .. }, _) => {
                    // Still in grace period - let servers process notifications
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
        // Update both per-CPU and legacy field
        set_current_slot(slot);
        self.current = slot;

        if let Some(ref mut task) = self.tasks[slot] {
            let _ = task.set_running();

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
                super::log::flush();

                enter_usermode(trap_frame as *const TrapFrame, ttbr0);
            }
        }

        // Should never reach here
        panic!("run_user_task: invalid task or no address space");
    }

    /// Check if there are any runnable tasks
    pub fn has_runnable_tasks(&self) -> bool {
        self.tasks.iter().any(|slot| {
            if let Some(ref task) = slot {
                task.is_runnable()
            } else {
                false
            }
        })
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

    /// Perform a context switch to the next ready task
    /// # Safety
    /// Must be called with interrupts disabled
    pub unsafe fn switch_to_next(&mut self) {
        let my_slot = current_slot();  // Use per-CPU current slot

        if let Some(next_idx) = self.schedule() {
            if next_idx != my_slot {
                let caller_slot = my_slot;  // Save our slot before switching

                // Mark current as ready (if still running)
                if let Some(ref mut current) = self.tasks[caller_slot] {
                    if *current.state() == TaskState::Running {
                        let _ = current.set_ready();
                    }
                }

                // Get pointers to current and next contexts
                let current_ctx = if let Some(ref mut t) = self.tasks[caller_slot] {
                    &mut t.context as *mut CpuContext
                } else {
                    return;
                };

                let next_ctx = if let Some(ref t) = self.tasks[next_idx] {
                    &t.context as *const CpuContext
                } else {
                    return;
                };

                // Switch address space if next task has one
                if let Some(ref task) = self.tasks[next_idx] {
                    if let Some(ref addr_space) = task.address_space {
                        addr_space.activate();
                    }
                }

                // Update current index (both per-CPU and legacy field)
                set_current_slot(next_idx);
                self.current = next_idx;

                // Mark next as running
                if let Some(ref mut t) = self.tasks[next_idx] {
                    let _ = t.set_running();
                }

                // CRITICAL: Update globals BEFORE context_switch so the target task
                // uses the correct trap frame when returning to user mode.
                update_current_task_globals();
                SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);

                // Actually switch
                context_switch(current_ctx, next_ctx);

                // We return here when switched back to this task.
                // Restore our slot (was saved on stack before switch).
                set_current_slot(caller_slot);
                self.current = caller_slot;

                // Mark ourselves as Running again
                if let Some(ref mut t) = self.tasks[caller_slot] {
                    let _ = t.set_running();
                }
                // Restore our address space and globals
                if let Some(ref task) = self.tasks[caller_slot] {
                    if let Some(ref addr_space) = task.address_space {
                        addr_space.activate();
                    }
                }
                update_current_task_globals();
            }
        }
    }

    /// Direct switch to a specific task by PID (for IPC fast-path)
    /// Returns true if switch happened, false if target not found/not ready
    ///
    /// This bypasses normal scheduling for synchronous IPC - the sender
    /// directly donates its timeslice to the receiver.
    ///
    /// # Safety
    /// Must be called with interrupts disabled
    pub unsafe fn direct_switch_to(&mut self, target_pid: TaskId) -> bool {
        let my_slot = current_slot();  // Use per-CPU current slot

        // Find target slot
        let target_slot = match self.slot_by_pid(target_pid) {
            Some(slot) => slot,
            None => return false,
        };

        // Don't switch to self
        if target_slot == my_slot {
            return false;
        }

        // Verify target is ready (or we just woke it)
        if let Some(ref task) = self.tasks[target_slot] {
            if !task.is_runnable() {
                return false;
            }
        } else {
            return false;
        }

        let caller_slot = my_slot;  // Save our slot before switching

        // Mark current as ready (donating timeslice)
        if let Some(ref mut current) = self.tasks[caller_slot] {
            if *current.state() == TaskState::Running {
                let _ = current.set_ready();
            }
        }

        // Get context pointers
        let current_ctx = if let Some(ref mut t) = self.tasks[caller_slot] {
            &mut t.context as *mut CpuContext
        } else {
            return false;
        };

        let next_ctx = if let Some(ref t) = self.tasks[target_slot] {
            &t.context as *const CpuContext
        } else {
            return false;
        };

        // Switch address space
        if let Some(ref task) = self.tasks[target_slot] {
            if let Some(ref addr_space) = task.address_space {
                addr_space.activate();
            }
        }

        // Update current (both per-CPU and legacy field) and mark target as running
        set_current_slot(target_slot);
        self.current = target_slot;
        if let Some(ref mut t) = self.tasks[target_slot] {
            let _ = t.set_running();
        }

        // CRITICAL: Update globals BEFORE context_switch so the target task
        // uses the correct trap frame when returning to user mode.
        // The target task will reload CURRENT_TRAP_FRAME from the global
        // in svc_handler/irq_from_user before eret.
        update_current_task_globals();
        SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);

        // Perform the switch
        context_switch(current_ctx, next_ctx);

        // Returned here when switched back to this task.
        // Restore our slot (was saved on stack before switch).
        set_current_slot(caller_slot);
        self.current = caller_slot;

        // Mark ourselves as Running again
        if let Some(ref mut t) = self.tasks[caller_slot] {
            let _ = t.set_running();
        }
        // Restore our address space and globals
        if let Some(ref task) = self.tasks[caller_slot] {
            if let Some(ref addr_space) = task.address_space {
                addr_space.activate();
            }
        }
        update_current_task_globals();

        true
    }

    /// Print scheduler state
    pub fn print_info(&self) {
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
                    TaskState::Dead => "dead",
                };
                let marker = if i == self.current { ">" } else { " " };
                print_direct!("    {} [{}] {} ({})\n", marker, task.id, task.name_str(), state_str);
            }
        }
    }
}

use core::sync::atomic::{AtomicU64, AtomicPtr, Ordering};

/// Global scheduler instance
static mut SCHEDULER: Scheduler = Scheduler::new();

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

/// Get the global scheduler
/// # Safety
/// Must ensure proper synchronization (interrupts disabled)
///
/// NOTE: Prefer `with_scheduler()` for safe access with automatic IRQ guard.
/// This raw accessor is `pub(crate)` to limit exposure.
pub(crate) unsafe fn scheduler() -> &'static mut Scheduler {
    &mut *core::ptr::addr_of_mut!(SCHEDULER)
}

/// Execute a closure with exclusive access to the scheduler.
/// Automatically disables interrupts for the duration.
///
/// Use this instead of `unsafe { scheduler() }` for safe access:
/// ```
/// with_scheduler(|sched| {
///     sched.wake(pid);
/// });
/// ```
#[inline]
pub fn with_scheduler<R, F: FnOnce(&mut Scheduler) -> R>(f: F) -> R {
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();
    unsafe { f(scheduler()) }
}

/// Update the current task globals after scheduling decision
/// # Safety
/// Must be called with valid current task
pub unsafe fn update_current_task_globals() {
    let sched = scheduler();

    // Defensive check: validate current slot is in bounds
    if sched.current >= MAX_TASKS {
        crate::platform::mt7988::uart::print("[PANIC] update_current_task_globals: current slot out of bounds!\r\n");
        loop { core::arch::asm!("wfe"); }
    }

    if let Some(ref mut task) = sched.tasks[sched.current] {
        let trap_ptr = &mut task.trap_frame as *mut TrapFrame;
        let trap_addr = trap_ptr as u64;

        // Defensive check: trap frame should be in kernel space (0xFFFF...)
        if trap_addr < 0xFFFF_0000_0000_0000 {
            crate::platform::mt7988::uart::print("[PANIC] update_current_task_globals: trap_frame not in kernel space!\r\n");
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
                crate::platform::mt7988::uart::print("[PANIC] update_current_task_globals: invalid TTBR0=0x");
                // Print hex value
                for i in (0..16).rev() {
                    let nibble = ((ttbr0 >> (i * 4)) & 0xf) as u8;
                    let c = if nibble < 10 { b'0' + nibble } else { b'a' + nibble - 10 };
                    crate::platform::mt7988::uart::putc(c as char);
                }
                crate::platform::mt7988::uart::print("\r\n");
                loop { core::arch::asm!("wfe"); }
            }

            CURRENT_TTBR0.store(ttbr0, Ordering::Release);
        }
    } else {
        // No task at current slot - this is also a bug
        crate::platform::mt7988::uart::print("[PANIC] update_current_task_globals: no task at current slot!\r\n");
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

    // Do the reschedule with IRQs disabled for the critical section
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();
    let sched = scheduler();
    let my_slot = current_slot();  // Use per-CPU slot for consistency with schedule()

    // Check if current task is blocked - must switch away immediately
    let current_blocked = if let Some(ref task) = sched.tasks[my_slot] {
        task.is_blocked()
    } else {
        false
    };

    // Skip if no reason to reschedule
    if !need_resched && !current_blocked {
        return;
    }

    // Mark current as ready (it was running) - but only if it was Running
    // (don't change Blocked tasks)
    if let Some(ref mut current) = sched.tasks[my_slot] {
        if *current.state() == TaskState::Running {
            let _ = current.set_ready();
        }
    }

    // Find next task
    if let Some(next_slot) = sched.schedule() {
        if next_slot != my_slot {
            // Update both per-CPU slot and legacy field
            set_current_slot(next_slot);
            sched.current = next_slot;
            if let Some(ref mut next_task) = sched.tasks[next_slot] {
                let _ = next_task.set_running();
            }
            update_current_task_globals();
            // Signal to assembly that we switched tasks
            SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);
        } else {
            // Same task selected - only mark as running if it was Ready
            if let Some(ref mut current) = sched.tasks[my_slot] {
                if *current.state() == TaskState::Ready {
                    let _ = current.set_running();
                }
            }
        }
    } else if current_blocked {
        // Current task is blocked but no other task is ready
        // We MUST wait for an interrupt to wake something up
        drop(_guard);

        loop {
            // CRITICAL: Explicitly enable IRQs before WFI!
            // After syscall, hardware leaves IRQs disabled (PSTATE.I set).
            // IrqGuard saw them disabled, so drop() didn't re-enable them.
            // We must enable IRQs here or WFI will wait forever.
            core::arch::asm!("msr daifclr, #2");  // Clear I bit = enable IRQs

            // Mark CPU as idle for usage tracking (set before WFI)
            crate::kernel::percpu::cpu_local().set_idle();

            // Wait for interrupt - timer tick or device IRQ will wake a task
            core::arch::asm!("wfi");

            // CPU woke up - no longer idle
            crate::kernel::percpu::cpu_local().clear_idle();

            // Disable IRQs to check scheduler state
            let _guard2 = crate::arch::aarch64::sync::IrqGuard::new();
            let sched2 = scheduler();

            if let Some(next_slot) = sched2.schedule() {
                // Found a ready task - switch to it
                // Update both per-CPU slot and legacy field
                set_current_slot(next_slot);
                sched2.current = next_slot;
                if let Some(ref mut next_task) = sched2.tasks[next_slot] {
                    let _ = next_task.set_running();
                }
                update_current_task_globals();
                SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);
                // Note: _guard2 will be dropped here, re-enabling IRQs
                return;
            }
            // No ready task yet, _guard2 drops here and we loop back to WFI
        }
    }
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
