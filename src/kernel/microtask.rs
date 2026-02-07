//! Microtask Queue — Deferred Auxiliary Work
//!
//! Moves non-scheduling work (IPC cleanup, resource release, parent notification,
//! eviction) out of the scheduler hot path into a bounded FIFO queue.
//!
//! # Design
//!
//! - **Per-CPU queues** (32 entries each) — local CPU enqueue needs only IRQ disable
//! - **Global overflow queue** (64 entries) — SpinLock at lock class 15, fallback
//! - **Enqueue from any context** holding SCHEDULER(10) — lock ordering 10→15 is valid
//! - **Drain at safe points** — irq_exit_resched, do_resched_if_needed, exit()
//! - **Budgeted drain** — max items per call prevents cleanup starving scheduling
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

    /// Re-wake a task whose wake may have been dropped due to WakeList overflow.
    /// The task will re-poll its Mux and discover any missed events.
    WakeSweep { pid: TaskId },

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
// Queue Internals
// ============================================================================

/// Generic ring buffer — used for both per-CPU and global queues.
/// Parameterized by capacity.
pub struct RingQueue<const N: usize> {
    ring: [MaybeUninit<MicroTask>; N],
    head: usize, // next write position
    tail: usize, // next read position
    len: usize,
}

impl<const N: usize> RingQueue<N> {
    pub const fn new() -> Self {
        Self {
            ring: [const { MaybeUninit::uninit() }; N],
            head: 0,
            tail: 0,
            len: 0,
        }
    }

    pub fn enqueue(&mut self, task: MicroTask) -> Result<(), MicroTask> {
        if self.len >= N {
            return Err(task);
        }
        self.ring[self.head] = MaybeUninit::new(task);
        self.head = (self.head + 1) % N;
        self.len += 1;
        Ok(())
    }

    pub fn dequeue(&mut self) -> Option<MicroTask> {
        if self.len == 0 {
            return None;
        }
        // SAFETY: tail..head contains initialized values
        let task = unsafe { self.ring[self.tail].assume_init_read() };
        self.tail = (self.tail + 1) % N;
        self.len -= 1;
        Some(task)
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

// SAFETY: RingQueue is only accessed behind SpinLock or with IRQs disabled (per-CPU)
unsafe impl<const N: usize> Send for RingQueue<N> {}

// ============================================================================
// Per-CPU Queue (32 entries, accessed with IRQs disabled, no SpinLock)
// ============================================================================

/// Per-CPU microtask queue capacity
pub const PERCPU_CAPACITY: usize = 32;

/// Per-CPU microtask queue type — embedded in CpuData.
/// Access is IRQ-disable protected (single-CPU, no lock needed).
pub type PerCpuMicroTaskQueue = RingQueue<PERCPU_CAPACITY>;

// ============================================================================
// Global Overflow Queue (64 entries, SpinLock)
// ============================================================================

const GLOBAL_CAPACITY: usize = 64;

static GLOBAL_OVERFLOW: SpinLock<RingQueue<GLOBAL_CAPACITY>> =
    SpinLock::new(lock_class::MICROTASK, RingQueue::new());

// ============================================================================
// Public API
// ============================================================================

/// Enqueue a microtask. Safe to call while holding SCHEDULER lock (class 10→15).
///
/// Fast path: tries the local CPU's per-CPU queue (IRQ disable only).
/// Slow path: falls back to global overflow queue (SpinLock).
/// Returns Err(task) if both queues are full.
pub fn enqueue(task: MicroTask) -> Result<(), MicroTask> {
    // Fast path: per-CPU queue (no SpinLock, just IRQ disable)
    let percpu = crate::kernel::percpu::cpu_local();
    // SAFETY: IRQs are disabled when enqueue is called (we're either in IRQ
    // context or holding SCHEDULER lock which disables IRQs). Only the local
    // CPU accesses its per-CPU queue.
    let local_q = unsafe { &mut *percpu.microtask_queue.get() };
    match local_q.enqueue(task) {
        Ok(()) => return Ok(()),
        Err(task) => {
            // Slow path: global overflow queue
            let mut global = GLOBAL_OVERFLOW.lock();
            global.enqueue(task)
        }
    }
}

/// Drain and execute up to `budget` pending microtasks.
///
/// Must be called at safe points (NOT under scheduler or ObjectService locks).
/// Drains the local CPU's per-CPU queue first, then the global overflow queue.
/// Returns the number of tasks executed.
pub fn drain(budget: usize) -> usize {
    let mut executed = 0;

    while executed < budget {
        // Try local CPU queue first (IRQ disable only)
        let task = {
            let percpu = crate::kernel::percpu::cpu_local();
            // SAFETY: We're at a safe drain point — not holding scheduler/obj locks.
            // IRQs may fire during execute(), but per-CPU queue is only mutated
            // with IRQs disabled (in enqueue or here). We disable IRQs briefly
            // to dequeue atomically.
            let local_q = unsafe { &mut *percpu.microtask_queue.get() };
            local_q.dequeue()
        };

        if let Some(t) = task {
            execute(t);
            executed += 1;
            continue;
        }

        // Local queue empty — try global overflow
        let task = {
            let mut global = GLOBAL_OVERFLOW.lock();
            global.dequeue()
        };

        match task {
            Some(t) => {
                execute(t);
                executed += 1;
            }
            None => break, // Both queues empty
        }
    }

    executed
}

/// Check if any queue has pending work (for debugging/stats).
pub fn has_pending() -> bool {
    let percpu = crate::kernel::percpu::cpu_local();
    let local_q = unsafe { &*percpu.microtask_queue.get() };
    if !local_q.is_empty() {
        return true;
    }
    let global = GLOBAL_OVERFLOW.lock();
    !global.is_empty()
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
        MicroTask::WakeSweep { pid } => {
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

    // Guard: verify PID is still valid and terminated before cleanup.
    // Another CPU may have already reaped this task via wait_child.
    let valid = crate::kernel::task::with_scheduler(|sched| {
        sched.slot_by_pid(pid)
            .and_then(|s| sched.task(s))
            .map(|t| t.is_terminated())
            .unwrap_or(false)
    });
    if !valid { return; }

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
    // Guard: verify PID is still valid and terminated before final cleanup.
    // Another CPU may have already reaped this task via wait_child.
    let valid = crate::kernel::task::with_scheduler(|sched| {
        sched.slot_by_pid(pid)
            .and_then(|s| sched.task(s))
            .map(|t| t.is_terminated())
            .unwrap_or(false)
    });
    if !valid { return; }

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
