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
//! 1. **Phase 1** (Exiting): IpcCleanup, ShmemNotify, KillChildren
//! 2. **Grace period**: Servers process Close/ShmemInvalid notifications
//! 3. **Phase 2** (after grace): NotifyParentExit, FinalCleanup, SlotReap
//!
//! NotifyParentExit is in Phase 2 so the parent (e.g. devd) learns about
//! child exit only after all IPC/bus cleanup is complete — safe to respawn.

use crate::kernel::lock::{SpinLock, lock_class};
use crate::kernel::task::TaskId;
use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

// ============================================================================
// MicroTask Variants
// ============================================================================

/// A unit of deferred work. Each variant is small (≤16 bytes) and does bounded work.
#[derive(Debug, Clone, Copy)]
pub enum MicroTask {
    // --- Immediate notifications ---

    /// Wake a task by PID
    Wake { pid: TaskId },

    /// Re-wake a task whose wake may have been dropped due to WakeList overflow.
    /// The task will re-poll its Mux and discover any missed events.
    WakeSweep { pid: TaskId },

    // --- Phase 1 cleanup (IPC teardown) ---

    /// DMA stop + subscriber removal + peer notification + shmem begin_cleanup
    IpcCleanup { pid: TaskId },

    /// Kill children of dying task (cascading teardown)
    KillChildren { pid: TaskId },

    // --- Phase 2 cleanup (after grace period) ---

    /// Notify parent that a child exited (ObjectService ProcessObject wake).
    /// Deferred to Phase 2: parent learns about exit only after IPC/bus cleanup
    /// is complete, so it's safe to immediately spawn a replacement.
    NotifyParentExit { parent_id: TaskId, child_pid: TaskId, code: i32 },

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
// Lock-Free MPSC Inbox (multi-producer, single-consumer per CPU)
// ============================================================================

/// Capacity of the per-CPU MPSC remote inbox.
pub const INBOX_CAPACITY: usize = 16;

/// A single slot in the MPSC inbox ring.
///
/// Layout: 1-byte ready flag + 7 padding + MicroTask data.
/// The `ready` flag prevents the consumer from reading partially-written data.
#[repr(C)]
pub struct InboxSlot {
    /// 0 = empty (slot available for write), 1 = data ready to read
    ready: AtomicU8,
    _pad: [u8; 7],
    data: UnsafeCell<MaybeUninit<MicroTask>>,
}

impl InboxSlot {
    const fn new() -> Self {
        Self {
            ready: AtomicU8::new(0),
            _pad: [0; 7],
            data: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }
}

/// Lock-free multi-producer single-consumer ring buffer.
///
/// Multiple CPUs can enqueue concurrently via CAS on `write_head`.
/// Only the owning CPU dequeues (with IRQs disabled).
///
/// # Protocol
///
/// **Enqueue (any CPU):**
/// 1. CAS-loop on `write_head` to claim a slot index
/// 2. Check capacity: if `claimed - read_tail >= CAPACITY`, slot is full → return Err
/// 3. Write MicroTask data into `slots[idx].data`
/// 4. Release-store `slots[idx].ready = 1`
///
/// **Dequeue (owning CPU, IRQs disabled):**
/// 1. Acquire-load `slots[read_tail % CAPACITY].ready`
/// 2. If 0 → empty
/// 3. Read data, Release-store `ready = 0`, advance `read_tail`
pub struct MpscInbox {
    slots: [InboxSlot; INBOX_CAPACITY],
    /// Next slot to be claimed by a producer. Monotonically increasing.
    write_head: AtomicU32,
    /// Next slot to be consumed by the owner. Only the owner CPU touches this.
    read_tail: u32,
}

impl MpscInbox {
    pub const fn new() -> Self {
        Self {
            slots: [const { InboxSlot::new() }; INBOX_CAPACITY],
            write_head: AtomicU32::new(0),
            read_tail: 0,
        }
    }

    /// Lock-free enqueue from any CPU.
    ///
    /// Uses a CAS loop on `write_head` to claim a slot, then writes data
    /// and sets the ready flag. Returns Err if the inbox is full.
    pub fn enqueue(&self, task: MicroTask) -> Result<(), MicroTask> {
        loop {
            let head = self.write_head.load(Ordering::Acquire);
            // Check capacity: read_tail is only advanced by the consumer,
            // so reading it from another CPU is safe — it can only increase,
            // meaning we may see a stale (smaller) value which is conservative
            // (we might think the queue is fuller than it is, never emptier).
            //
            // SAFETY: read_tail is a plain u32 on MpscInbox which lives in
            // UnsafeCell in CpuData. We read it via a volatile read to avoid
            // tearing on 32-bit values (guaranteed atomic on AArch64).
            let tail = unsafe {
                core::ptr::read_volatile(&self.read_tail)
            };
            if head.wrapping_sub(tail) >= INBOX_CAPACITY as u32 {
                return Err(task); // Full
            }

            // Try to claim this slot
            match self.write_head.compare_exchange_weak(
                head,
                head.wrapping_add(1),
                Ordering::AcqRel,
                Ordering::Relaxed,
            ) {
                Ok(_) => {
                    // Slot claimed. Write data then publish.
                    let idx = (head as usize) % INBOX_CAPACITY;
                    // SAFETY: We own this slot exclusively (CAS succeeded).
                    // No other producer will write here until we set ready=1
                    // and the consumer later sets ready=0.
                    unsafe {
                        (*self.slots[idx].data.get()).write(task);
                    }
                    // Publish: consumer can now see this data
                    self.slots[idx].ready.store(1, Ordering::Release);
                    return Ok(());
                }
                Err(_) => {
                    // CAS failed — another producer claimed this slot, retry
                    continue;
                }
            }
        }
    }

    /// Single-consumer dequeue (owning CPU only, IRQs must be disabled).
    ///
    /// Returns None if no data is ready.
    pub fn dequeue(&mut self) -> Option<MicroTask> {
        let idx = (self.read_tail as usize) % INBOX_CAPACITY;
        // Check if this slot has been published
        if self.slots[idx].ready.load(Ordering::Acquire) == 0 {
            return None;
        }
        // SAFETY: ready=1 means the producer finished writing data.
        let task = unsafe { (*self.slots[idx].data.get()).assume_init_read() };
        // Release slot for reuse
        self.slots[idx].ready.store(0, Ordering::Release);
        self.read_tail = self.read_tail.wrapping_add(1);
        Some(task)
    }

    /// Check if any slots have pending data (without consuming).
    pub fn has_pending(&self) -> bool {
        let idx = (self.read_tail as usize) % INBOX_CAPACITY;
        self.slots[idx].ready.load(Ordering::Acquire) != 0
    }
}

// SAFETY: MpscInbox is designed for cross-CPU access. Producers use atomics (CAS + ready flag).
// Consumer access is IRQ-disabled on owning CPU.
unsafe impl Send for MpscInbox {}
unsafe impl Sync for MpscInbox {}

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

/// Enqueue a microtask on the local CPU. Safe to call while holding SCHEDULER lock (class 10→15).
///
/// Fast path: tries the local CPU's per-CPU queue (IRQ disable only).
/// Slow path: falls back to global overflow queue (SpinLock).
/// Returns Err(task) if both queues are full.
///
/// For Wake/WakeSweep variants, also sets need_resched so the wake is
/// processed at the next safe point (irq_exit_resched / do_resched_if_needed).
pub fn enqueue(task: MicroTask) -> Result<(), MicroTask> {
    // Set need_resched for wake variants so the scheduler runs at the next safe point
    if matches!(task, MicroTask::Wake { .. } | MicroTask::WakeSweep { .. }) {
        crate::kernel::percpu::cpu_local().set_need_resched();
    }

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

/// Enqueue a microtask to a specific CPU's MPSC inbox (lock-free).
///
/// This is for cross-CPU enqueue. The task goes into the target CPU's
/// lock-free inbox. Falls back to the global overflow queue if the inbox
/// is full. Sends an IPI (SGI 0) if the target CPU is idle.
///
/// Returns Err(task) if both inbox and global overflow are full.
pub fn enqueue_remote(target_cpu: u32, task: MicroTask) -> Result<(), MicroTask> {
    let target = match crate::kernel::percpu::try_get_cpu_data(target_cpu as usize) {
        Some(cpu) => cpu,
        None => return Err(task),
    };

    // Try the target CPU's MPSC inbox (lock-free)
    // SAFETY: remote_inbox is designed for cross-CPU access via atomics.
    let inbox = unsafe { &*target.remote_inbox.get() };
    match inbox.enqueue(task) {
        Ok(()) => {
            // Poke target CPU if it's idle so it drains the inbox
            if target.is_idle() {
                crate::platform::current::gic::send_sgi(target_cpu, 0);
            }
            Ok(())
        }
        Err(task) => {
            // Inbox full — fall back to global overflow
            let mut global = GLOBAL_OVERFLOW.lock();
            global.enqueue(task)
        }
    }
}

/// Drain and execute up to `budget` pending microtasks.
///
/// Must be called at safe points (NOT under scheduler or ObjectService locks).
/// Drain order: local_queue → remote_inbox → global_overflow.
/// Returns the number of tasks executed.
pub fn drain(budget: usize) -> usize {
    let mut executed = 0;

    while executed < budget {
        // 1. Try local CPU queue first (IRQ disable only, fastest)
        let task = {
            let percpu = crate::kernel::percpu::cpu_local();
            // SAFETY: We're at a safe drain point — not holding scheduler/obj locks.
            // IRQs may fire during execute(), but per-CPU queue is only mutated
            // with IRQs disabled (in enqueue or here).
            let local_q = unsafe { &mut *percpu.microtask_queue.get() };
            local_q.dequeue()
        };

        if let Some(t) = task {
            execute(t);
            executed += 1;
            continue;
        }

        // 2. Local empty — try MPSC remote inbox (items enqueued by other CPUs)
        let task = {
            let percpu = crate::kernel::percpu::cpu_local();
            // SAFETY: Only the owning CPU dequeues from its inbox.
            // IRQs are disabled at safe points (we're in IRQ-exit or reschedule path).
            let inbox = unsafe { &mut *percpu.remote_inbox.get() };
            inbox.dequeue()
        };

        if let Some(t) = task {
            execute(t);
            executed += 1;
            continue;
        }

        // 3. Inbox empty — try global overflow queue (SpinLock)
        let task = {
            let mut global = GLOBAL_OVERFLOW.lock();
            global.dequeue()
        };

        match task {
            Some(t) => {
                execute(t);
                executed += 1;
            }
            None => break, // All three queues empty
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
    let inbox = unsafe { &*percpu.remote_inbox.get() };
    if inbox.has_pending() {
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
        MicroTask::KillChildren { pid } => {
            exec_kill_children(pid);
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

/// Kill children of dying task, then set cleanup_phase to GracePeriod.
///
/// Cascading kill: each killed child gets its own IpcCleanup + KillChildren
/// microtasks enqueued, so the entire subtree is torn down.
fn exec_kill_children(pid: TaskId) {
    use crate::kernel::task::tcb::{CleanupPhase, MAX_CHILDREN};
    use crate::kernel::task::lifecycle;

    let mut killed_pids: [TaskId; MAX_CHILDREN] = [0; MAX_CHILDREN];
    let mut killed_count = 0usize;

    crate::kernel::task::with_scheduler(|sched| {
        let slot = match sched.slot_by_pid(pid) {
            Some(s) => s,
            None => return,
        };

        // Snapshot children array
        let children: [TaskId; MAX_CHILDREN] = sched.task(slot)
            .map(|t| t.children)
            .unwrap_or([0; MAX_CHILDREN]);

        for child_pid in children {
            if child_pid == 0 {
                continue;
            }

            // Kill child — permission check passes since pid IS the child's parent
            match lifecycle::kill(sched, child_pid, pid) {
                Ok(_info) => {
                    // Set cleanup phase on the killed child
                    if let Some(child_slot) = sched.slot_by_pid(child_pid) {
                        if let Some(child) = sched.task_mut(child_slot) {
                            child.cleanup_phase = CleanupPhase::Phase1Enqueued;
                        }
                    }
                    if killed_count < MAX_CHILDREN {
                        killed_pids[killed_count] = child_pid;
                        killed_count += 1;
                    }
                }
                Err(_) => {
                    // Child already exiting/dead, skip
                }
            }
        }

        // Wake parent of the dying task if sleeping (for Process watch)
        sched.wake_parent_if_sleeping(slot);

        // Transition dying task: Phase1Enqueued → GracePeriod
        // 100ms grace period in hardware counter units
        let grace_until = crate::platform::current::timer::deadline_ns(100_000_000);
        if let Some(task) = sched.task_mut(slot) {
            task.cleanup_phase = CleanupPhase::GracePeriod { until: grace_until };
        }
    });

    // Enqueue Phase 1 cleanup for each killed child (outside scheduler lock).
    // NotifyParentExit is deferred — each child's own Phase 2 will notify its parent.
    for i in 0..killed_count {
        let child_pid = killed_pids[i];
        let _ = enqueue(MicroTask::IpcCleanup { pid: child_pid });
        let _ = enqueue(MicroTask::KillChildren { pid: child_pid }); // recursive cascade
    }
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
            Some(task) if task.id == pid => {
                #[cfg(debug_assertions)]
                debug_assert!(
                    task.is_terminated(),
                    "SlotReap: task {} in slot {} is {:?}, expected terminated",
                    pid, slot, task.state().name()
                );
                // Verify stack canary before freeing (catches corruption that happened during lifetime)
                #[cfg(debug_assertions)]
                if !task.verify_stack_canary() {
                    crate::kwarn!("reap", "stack_canary_corrupt"; pid = pid as u64, slot = slot as u64);
                }
            }
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
