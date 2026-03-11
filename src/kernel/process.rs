//! Process Metadata Table
//!
//! Owns all per-task metadata that is NOT scheduling state. This separates
//! "who is this process" from "when does it run" — external code accesses
//! ProcessInfo without touching the scheduler lock.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────┐
//! │         Scheduler               │  Owns: state, context, priority,
//! │    (task::with_scheduler)       │  trap_frame, address_space, stack
//! └─────────────────────────────────┘
//!
//! ┌─────────────────────────────────┐
//! │       ProcessTable              │  Owns: name, capabilities, signals,
//! │    (process::with/with_mut)     │  resource counts, children, liveness
//! └─────────────────────────────────┘
//!
//! ┌─────────────────────────────────┐
//! │       ObjectService             │  Owns: handle tables (channels,
//! │    (object_service::with_table) │  ports, timers, mux objects)
//! └─────────────────────────────────┘
//! ```
//!
//! # Lock Ordering
//!
//! ```text
//! SCHEDULER(10) → PROCESS(12) → MICROTASK(15) → OBJ_SERVICE(20) → ...
//! ```
//!
//! The scheduler NEVER needs process metadata during scheduling decisions.
//! External code acquires the ProcessTable lock directly.

/// Process ID type (legacy re-export — equivalent to TaskId)
pub type Pid = u32;

use crate::kernel::lock::{SpinLock, lock_class};
use crate::kernel::task::{TaskId, MAX_TASKS};
use crate::kernel::task::tcb::{
    WakeData, ResourceLimits, CleanupPhase, MAX_CHILDREN,
    MAX_SIGNAL_SENDERS, MAX_PENDING_SIGNALS,
};
use crate::kernel::caps::Capabilities;
use crate::kernel::liveness::LivenessState;
use crate::kernel::storm::StormState;

/// Process metadata — everything about a task that isn't scheduling state.
///
/// Created when a task spawns, destroyed when a task is reaped.
pub struct ProcessInfo {
    // ========================================================================
    // Identity
    // ========================================================================

    /// Task ID (duplicated from Task for convenience — always matches)
    pub id: TaskId,
    /// Parent task ID (0 for orphan/init)
    pub parent_id: TaskId,
    /// Is this the init process (devd)?
    pub is_init: bool,
    /// Is this the probed process (boot-time bus discovery)?
    pub is_probed: bool,
    /// Task name for debugging
    pub name: [u8; 16],

    // ========================================================================
    // Security
    // ========================================================================

    /// Capability set for this task
    pub capabilities: Capabilities,
    /// Signal allowlist — PIDs allowed to send signals
    pub signal_allowlist: [u32; MAX_SIGNAL_SENDERS],
    /// Number of entries in signal allowlist
    pub signal_allowlist_count: usize,

    // ========================================================================
    // Resource accounting
    // ========================================================================

    /// Per-process resource limits
    pub limits: ResourceLimits,
    /// Per-process resource counters
    pub channel_count: u16,
    pub port_count: u16,
    pub shmem_count: u16,

    // ========================================================================
    // Process tree
    // ========================================================================

    /// Child task IDs
    pub children: [TaskId; MAX_CHILDREN],
    /// Number of children
    pub num_children: usize,

    // ========================================================================
    // Liveness & storm protection
    // ========================================================================

    /// Liveness tracking state
    pub liveness_state: LivenessState,
    /// Last activity tick (syscall made)
    pub last_activity_tick: u64,
    /// Storm protection state (syscall/wake rate limiting)
    pub storm: StormState,

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /// Cleanup phase for exiting tasks (microtask-driven)
    pub cleanup_phase: CleanupPhase,

    // ========================================================================
    // Metrics
    // ========================================================================

    /// IPC messages sent by this task (saturating counter)
    pub ipc_sent: u32,
    /// IPC messages received by this task (saturating counter)
    pub ipc_recv: u32,
    /// Cumulative syscall count (saturating)
    pub total_syscalls: u64,
    /// Page faults handled for this task (demand paging)
    pub page_faults: u32,

    // ========================================================================
    // Mux / wake fast path
    // ========================================================================

    /// Data delivered with the last wake (for Mux fast path)
    pub wake_data: Option<WakeData>,

    // ========================================================================
    // Exception handling
    // ========================================================================

    /// Exception channel: if set, user faults freeze the task and deliver
    /// fault info on this channel instead of killing. (parent_task_id, channel_id)
    pub exception_channel: Option<(TaskId, u32)>,
    /// Deadline (counter ticks) for frozen task timeout — kill if exceeded
    pub frozen_deadline: u64,

    // ========================================================================
    // Signal queue
    // ========================================================================

    /// Per-task signal queue (fixed ring buffer)
    pub signal_queue: [abi::PendingSignal; MAX_PENDING_SIGNALS],
    pub signal_head: u8,  // next write position
    pub signal_tail: u8,  // next read position
    pub signal_count: u8, // current count
}

impl ProcessInfo {
    /// Create a new ProcessInfo with default values.
    pub fn new(id: TaskId, parent_id: TaskId, name: [u8; 16], caps: Capabilities) -> Self {
        Self {
            id,
            parent_id,
            is_init: false,
            is_probed: false,
            name,
            capabilities: caps,
            signal_allowlist: [0; MAX_SIGNAL_SENDERS],
            signal_allowlist_count: 0,
            limits: ResourceLimits::default(),
            channel_count: 0,
            port_count: 0,
            shmem_count: 0,
            children: [0; MAX_CHILDREN],
            num_children: 0,
            liveness_state: LivenessState::Normal,
            last_activity_tick: 0,
            storm: StormState::new(),
            cleanup_phase: CleanupPhase::None,
            ipc_sent: 0,
            ipc_recv: 0,
            total_syscalls: 0,
            page_faults: 0,
            wake_data: None,
            exception_channel: None,
            frozen_deadline: 0,
            signal_queue: [abi::PendingSignal { event: 0, value: 0 }; MAX_PENDING_SIGNALS],
            signal_head: 0,
            signal_tail: 0,
            signal_count: 0,
        }
    }

    /// Get task name as a string slice.
    pub fn name_str(&self) -> &str {
        let len = self.name.iter().position(|&c| c == 0).unwrap_or(16);
        core::str::from_utf8(&self.name[..len]).unwrap_or("???")
    }

    // ========================================================================
    // Resource counter methods
    // ========================================================================

    pub fn can_create_channel(&self) -> bool {
        self.channel_count < self.limits.max_channels
    }

    pub fn add_channel(&mut self) {
        self.channel_count = self.channel_count.saturating_add(1);
    }

    pub fn remove_channel(&mut self) {
        self.channel_count = self.channel_count.saturating_sub(1);
    }

    pub fn can_create_port(&self) -> bool {
        self.port_count < self.limits.max_ports
    }

    pub fn add_port(&mut self) {
        self.port_count = self.port_count.saturating_add(1);
    }

    pub fn remove_port(&mut self) {
        self.port_count = self.port_count.saturating_sub(1);
    }

    pub fn can_create_shmem(&self) -> bool {
        self.shmem_count < self.limits.max_shmem
    }

    pub fn add_shmem(&mut self) {
        self.shmem_count = self.shmem_count.saturating_add(1);
    }

    pub fn remove_shmem(&mut self) {
        self.shmem_count = self.shmem_count.saturating_sub(1);
    }

    // ========================================================================
    // Capability methods
    // ========================================================================

    pub fn has_capability(&self, cap: Capabilities) -> bool {
        self.capabilities.has(cap)
    }

    pub fn get_capabilities_bits(&self) -> u64 {
        self.capabilities.bits()
    }

    pub fn set_capabilities(&mut self, caps: Capabilities) {
        self.capabilities = caps;
    }

    // ========================================================================
    // Signal queue methods
    // ========================================================================

    /// Enqueue a signal into the task's signal queue.
    ///
    /// When the queue is full, coalesces by ORing event bits into the newest
    /// entry. This ensures no event *type* is lost, though the value may be
    /// from a different event. Returns true always (coalescing never fails).
    pub fn enqueue_signal(&mut self, event: u32, value: u64) -> bool {
        debug_assert!(event <= 0xFFFF, "signal event {:#x} exceeds u16 range for MuxEvent delivery", event);
        if self.signal_count >= MAX_PENDING_SIGNALS as u8 {
            // Coalesce: OR event bits into the newest (head-1) entry
            let tail_idx = (self.signal_head + MAX_PENDING_SIGNALS as u8 - 1) % MAX_PENDING_SIGNALS as u8;
            self.signal_queue[tail_idx as usize].event |= event;
            return true;
        }
        self.signal_queue[self.signal_head as usize] = abi::PendingSignal { event, value };
        self.signal_head = (self.signal_head + 1) % MAX_PENDING_SIGNALS as u8;
        self.signal_count += 1;
        true
    }

    /// Dequeue one signal. Returns None if empty.
    pub fn dequeue_signal(&mut self) -> Option<abi::PendingSignal> {
        if self.signal_count == 0 {
            return None;
        }
        let sig = self.signal_queue[self.signal_tail as usize];
        self.signal_tail = (self.signal_tail + 1) % MAX_PENDING_SIGNALS as u8;
        self.signal_count -= 1;
        Some(sig)
    }

    /// Check if there are pending signals.
    pub fn has_pending_signals(&self) -> bool {
        self.signal_count > 0
    }

    // ========================================================================
    // Signal allowlist methods
    // ========================================================================

    pub fn can_receive_signal_from(&self, sender: TaskId) -> bool {
        // Allow from parent always
        if sender == self.parent_id {
            return true;
        }
        // Check allowlist
        for i in 0..self.signal_allowlist_count {
            if self.signal_allowlist[i] == sender {
                return true;
            }
        }
        false
    }

    pub fn allow_signals_from(&mut self, sender: TaskId) {
        if self.signal_allowlist_count < MAX_SIGNAL_SENDERS {
            self.signal_allowlist[self.signal_allowlist_count] = sender;
            self.signal_allowlist_count += 1;
        }
    }

    // ========================================================================
    // Children methods
    // ========================================================================

    pub fn add_child(&mut self, child_id: TaskId) -> bool {
        if self.num_children >= MAX_CHILDREN {
            return false;
        }
        self.children[self.num_children] = child_id;
        self.num_children += 1;
        true
    }

    pub fn remove_child(&mut self, child_id: TaskId) {
        for i in 0..self.num_children {
            if self.children[i] == child_id {
                // Swap with last
                self.children[i] = self.children[self.num_children - 1];
                self.children[self.num_children - 1] = 0;
                self.num_children -= 1;
                return;
            }
        }
    }

    // ========================================================================
    // Activity / liveness
    // ========================================================================

    pub fn record_activity(&mut self, tick: u64) {
        self.last_activity_tick = tick;
    }

    pub fn reset_liveness_if_implicit_pong(&mut self) {
        if matches!(self.liveness_state, LivenessState::PingSent { .. }) {
            self.liveness_state = LivenessState::Normal;
        }
    }

    /// Record a syscall for storm protection and return action to take
    #[inline]
    pub fn record_storm_syscall(&mut self, tick: u64, config: &crate::kernel::storm::StormConfig) -> crate::kernel::storm::StormAction {
        self.storm.record_syscall(tick, config)
    }

    /// Get activity age in milliseconds since last syscall.
    #[inline]
    pub fn get_activity_age_ms(&self, current_counter: u64) -> u32 {
        if self.last_activity_tick == 0 {
            0
        } else {
            let delta = current_counter.saturating_sub(self.last_activity_tick);
            let freq = crate::platform::current::timer::frequency();
            if freq == 0 { return 0; }
            (delta * 1000 / freq) as u32
        }
    }

    /// Get liveness status code for ABI
    #[inline]
    pub fn get_liveness_status_code(&self) -> u8 {
        use abi::liveness_status;
        match self.liveness_state {
            LivenessState::Normal => liveness_status::NORMAL,
            LivenessState::PingSent { .. } => liveness_status::PING_SENT,
            LivenessState::ClosePending { .. } => liveness_status::CLOSE_PENDING,
        }
    }

    /// Check if task can spawn another child
    #[inline]
    pub fn can_add_child(&self) -> bool {
        (self.num_children as u16) < self.limits.max_children
    }

    /// Check if task has children
    pub fn has_children(&self) -> bool {
        self.num_children > 0
    }

    /// Reset per-task statistics counters to zero.
    pub fn reset_stats(&mut self) {
        self.ipc_sent = 0;
        self.ipc_recv = 0;
        self.total_syscalls = 0;
        self.page_faults = 0;
    }
}

// ============================================================================
// ProcessTable — Global table with its own lock
// ============================================================================

/// Global process metadata table.
///
/// Indexed by task slot (same slots as the scheduler). Lock class is PROCESS(12),
/// between SCHEDULER(10) and MICROTASK(15).
pub struct ProcessTable {
    entries: SpinLock<[Option<ProcessInfo>; MAX_TASKS]>,
}

impl ProcessTable {
    const fn new() -> Self {
        const NONE: Option<ProcessInfo> = None;
        Self {
            entries: SpinLock::new(lock_class::PROCESS, [NONE; MAX_TASKS]),
        }
    }

    /// Access process info by slot (read-only).
    #[inline]
    pub fn with<R, F: FnOnce(&ProcessInfo) -> R>(&self, slot: usize, f: F) -> Option<R> {
        let guard = self.entries.lock();
        guard[slot].as_ref().map(f)
    }

    /// Access process info by slot (mutable).
    #[inline]
    pub fn with_mut<R, F: FnOnce(&mut ProcessInfo) -> R>(&self, slot: usize, f: F) -> Option<R> {
        let mut guard = self.entries.lock();
        guard[slot].as_mut().map(f)
    }

    /// Create ProcessInfo for a task slot.
    pub fn create(&self, slot: usize, info: ProcessInfo) {
        let mut guard = self.entries.lock();
        guard[slot] = Some(info);
    }

    /// Destroy ProcessInfo when a task is reaped.
    pub fn destroy(&self, slot: usize) {
        let mut guard = self.entries.lock();
        guard[slot] = None;
    }

    /// Iterate all occupied slots (read-only).
    ///
    /// Acquires the lock for the duration of the closure. The closure receives
    /// an iterator of (slot, &ProcessInfo) pairs.
    pub fn for_each<F: FnMut(usize, &ProcessInfo)>(&self, mut f: F) {
        let guard = self.entries.lock();
        for (slot, entry) in guard.iter().enumerate() {
            if let Some(info) = entry {
                f(slot, info);
            }
        }
    }

    /// Iterate all occupied slots (mutable).
    pub fn for_each_mut<F: FnMut(usize, &mut ProcessInfo)>(&self, mut f: F) {
        let mut guard = self.entries.lock();
        for (slot, entry) in guard.iter_mut().enumerate() {
            if let Some(info) = entry {
                f(slot, info);
            }
        }
    }
}

static PROCESS_TABLE: ProcessTable = ProcessTable::new();

/// Get the global process table.
#[inline]
pub fn process_table() -> &'static ProcessTable {
    &PROCESS_TABLE
}
