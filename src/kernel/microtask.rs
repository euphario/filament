//! Microtask Queue — Deferred Auxiliary Work
//!
//! Moves non-scheduling work (IPC cleanup, resource release, parent notification,
//! eviction) out of the scheduler hot path into a bounded FIFO queue.
//!
//! # Design
//!
//! - **Bounded ring buffer** (64 entries) protected by SpinLock at lock class 15
//! - **Enqueue from any context** holding SCHEDULER(10) — lock ordering 10→15 is valid
//! - **Drain at safe points** — irq_exit_resched, do_resched_if_needed, exit()
//! - **Execute without holding microtask lock** — dequeue one item, release lock,
//!   execute, repeat. Executor can freely acquire OBJ_SERVICE(20+).
//!
//! # Phases
//!
//! Task exit cleanup is split into two phases with a grace period:
//!
//! 1. **Phase 1** (Exiting): NotifyParentExit, IpcCleanup, ShmemNotify, ReparentChildren
//! 2. **Grace period**: Servers process Close/ShmemInvalid notifications
//! 3. **Phase 2** (after grace): FinalCleanup, SlotReap

use crate::kernel::lock::{SpinLock, lock_class};
use crate::kernel::task::TaskId;
use core::mem::MaybeUninit;

// ============================================================================
// MicroTask Variants
// ============================================================================

/// A unit of deferred work. Each variant is small (≤16 bytes) and does bounded work.
#[derive(Debug, Clone, Copy)]
pub enum MicroTask {
    // --- Notifications (run first) ---

    /// Notify parent that a child exited (ObjectService ProcessObject wake)
    NotifyParentExit { parent_id: TaskId, child_pid: TaskId, code: i32 },

    /// Wake a task by PID
    Wake { pid: TaskId },

    // --- Phase 1 cleanup (IPC teardown) ---

    /// DMA stop + subscriber removal + peer notification + shmem begin_cleanup
    IpcCleanup { pid: TaskId },

    /// Snapshot children under brief sched lock, reparent to init
    ReparentChildren { pid: TaskId },

    // --- Phase 2 cleanup (resource release, after grace period) ---

    /// shmem::finalize + irq + pci + ports + object table removal
    FinalCleanup { pid: TaskId },

    /// Brief sched lock: clear_stale_trap_frame, bump_generation, free slot
    SlotReap { pid: TaskId, slot: u16 },

    // --- Eviction ---

    /// Kernel-initiated forced termination
    Evict { pid: TaskId, reason: u8 },
}

// ============================================================================
// Queue
// ============================================================================

const QUEUE_CAPACITY: usize = 64;

struct MicroTaskQueue {
    ring: [MaybeUninit<MicroTask>; QUEUE_CAPACITY],
    head: usize, // next write position
    tail: usize, // next read position
    len: usize,
}

impl MicroTaskQueue {
    const fn new() -> Self {
        Self {
            ring: [const { MaybeUninit::uninit() }; QUEUE_CAPACITY],
            head: 0,
            tail: 0,
            len: 0,
        }
    }

    fn enqueue(&mut self, task: MicroTask) -> Result<(), MicroTask> {
        if self.len >= QUEUE_CAPACITY {
            return Err(task);
        }
        self.ring[self.head] = MaybeUninit::new(task);
        self.head = (self.head + 1) % QUEUE_CAPACITY;
        self.len += 1;
        Ok(())
    }

    fn dequeue(&mut self) -> Option<MicroTask> {
        if self.len == 0 {
            return None;
        }
        // SAFETY: tail..head contains initialized values
        let task = unsafe { self.ring[self.tail].assume_init_read() };
        self.tail = (self.tail + 1) % QUEUE_CAPACITY;
        self.len -= 1;
        Some(task)
    }

    fn is_empty(&self) -> bool {
        self.len == 0
    }
}

// SAFETY: MicroTaskQueue is only accessed behind SpinLock (IRQs disabled)
unsafe impl Send for MicroTaskQueue {}

static MICROTASK_QUEUE: SpinLock<MicroTaskQueue> =
    SpinLock::new(lock_class::MICROTASK, MicroTaskQueue::new());

// ============================================================================
// Public API
// ============================================================================

/// Enqueue a microtask. Safe to call while holding SCHEDULER lock (class 10→15).
///
/// Returns Err(task) if the queue is full. Callers should either:
/// - Retry on next timer tick
/// - For Wake: fallback to set_need_resched()
pub fn enqueue(task: MicroTask) -> Result<(), MicroTask> {
    let mut q = MICROTASK_QUEUE.lock();
    q.enqueue(task)
}

/// Drain and execute all pending microtasks.
///
/// Must be called at safe points (NOT under scheduler or ObjectService locks).
/// Dequeues one item at a time, releases the microtask lock, executes, repeats.
/// This allows executors to freely acquire any lock (OBJ_SERVICE, SUBSYSTEM, RESOURCE).
pub fn drain() {
    loop {
        // Dequeue one item under lock
        let task = {
            let mut q = MICROTASK_QUEUE.lock();
            match q.dequeue() {
                Some(t) => t,
                None => return,
            }
        }; // lock released

        // Execute without holding microtask lock
        execute(task);
    }
}

/// Check if the queue has pending work (for debugging/stats).
pub fn has_pending() -> bool {
    let q = MICROTASK_QUEUE.lock();
    !q.is_empty()
}

// ============================================================================
// Executors
// ============================================================================

/// Dispatch a microtask to the appropriate executor.
fn execute(task: MicroTask) {
    match task {
        MicroTask::NotifyParentExit { parent_id, child_pid, code } => {
            exec_notify_parent_exit(parent_id, child_pid, code);
        }
        MicroTask::Wake { pid } => {
            exec_wake(pid);
        }
        MicroTask::IpcCleanup { pid } => {
            exec_ipc_cleanup(pid);
        }
        MicroTask::ReparentChildren { pid } => {
            exec_reparent_children(pid);
        }
        MicroTask::FinalCleanup { pid } => {
            exec_final_cleanup(pid);
        }
        MicroTask::SlotReap { pid, slot } => {
            exec_slot_reap(pid, slot as usize);
        }
        MicroTask::Evict { pid, reason } => {
            exec_evict(pid, reason);
        }
    }
}

/// Notify parent that a child exited via ObjectService ProcessObject wake.
fn exec_notify_parent_exit(parent_id: TaskId, child_pid: TaskId, code: i32) {
    use crate::kernel::task::lifecycle::{ExitInfo, complete_exit_notification};
    complete_exit_notification(ExitInfo { parent_id, child_pid, code });
}

/// Wake a task by PID.
fn exec_wake(pid: TaskId) {
    crate::kernel::task::with_scheduler(|sched| {
        sched.wake_by_pid(pid);
    });
}

/// Phase 1 IPC cleanup: DMA stop, subscriber removal, peer notification, shmem begin.
fn exec_ipc_cleanup(pid: TaskId) {
    use crate::kernel::ipc::{waker, traits::WakeReason};

    // DMA stop + subscriber removal + peer notification
    let ipc_peers = crate::kernel::task::Scheduler::do_ipc_cleanup(pid);
    for peer in ipc_peers.iter() {
        let wake_list = crate::kernel::object_service::object_service()
            .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::CLOSED);
        waker::wake(&wake_list, WakeReason::Closed);
    }

    // Begin shmem cleanup (marks regions Dying, sends ShmemInvalid events)
    crate::kernel::shmem::begin_cleanup(pid);
}

/// Reparent children to init (PID 1), then set cleanup_phase to GracePeriod.
fn exec_reparent_children(pid: TaskId) {
    use crate::kernel::task::tcb::CleanupPhase;

    crate::kernel::task::with_scheduler(|sched| {
        let slot = match sched.slot_by_pid(pid) {
            Some(s) => s,
            None => return,
        };

        // Snapshot children
        let children: [TaskId; crate::kernel::task::tcb::MAX_CHILDREN] = sched.task(slot)
            .map(|t| t.children)
            .unwrap_or([0; crate::kernel::task::tcb::MAX_CHILDREN]);

        let has_init = sched.slot_by_pid(1).is_some();

        for child_pid in children {
            if child_pid == 0 {
                continue;
            }
            if let Some(child_slot) = sched.slot_by_pid(child_pid) {
                if let Some(child) = sched.task_mut(child_slot) {
                    if has_init {
                        child.parent_id = 1;
                    } else {
                        child.parent_id = 0;
                    }
                }
            }
        }

        // Wake parent if sleeping (for Process watch)
        sched.wake_parent_if_sleeping(slot);

        // Transition: Phase1Enqueued → GracePeriod
        // 100ms grace period in hardware counter units
        let grace_until = crate::platform::current::timer::deadline_ns(100_000_000);
        if let Some(task) = sched.task_mut(slot) {
            task.cleanup_phase = CleanupPhase::GracePeriod { until: grace_until };
        }
    });
}

/// Phase 2 final cleanup: shmem finalize, IRQ, PCI, ports, object table removal.
fn exec_final_cleanup(pid: TaskId) {
    crate::kernel::task::Scheduler::do_final_cleanup(pid);
}

/// Slot reap: clear stale trap frame, bump generation, free slot.
fn exec_slot_reap(pid: TaskId, slot: usize) {
    crate::kernel::task::with_scheduler(|sched| {
        // Verify the slot still belongs to this pid
        match sched.task(slot) {
            Some(task) if task.id == pid => {}
            _ => return, // Already reaped (e.g. by wait_child) or PID mismatch
        }

        if let Some(task) = sched.task(slot) {
            crate::kernel::task::Scheduler::clear_stale_trap_frame(task, slot, pid);
        }

        sched.notify_exit(slot);
        sched.bump_generation(slot);
        sched.clear_slot(slot);
    });
}

/// Evict a task (kernel-initiated forced termination).
fn exec_evict(pid: TaskId, reason_val: u8) {
    use crate::kernel::task::eviction;
    let reason = eviction::u8_to_reason(reason_val)
        .unwrap_or(crate::kernel::task::EvictionReason::StateCorruption);
    eviction::do_evict_task(pid, reason);
}
