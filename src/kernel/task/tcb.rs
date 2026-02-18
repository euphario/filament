//! Task Control Block
//!
//! Contains the Task struct, CPU contexts, and related types.
//! This is the per-task state, separate from the scheduler.

use crate::kernel::addrspace::AddressSpace;
use crate::kernel::pmm;
use crate::kernel::arch::{mmu, tlb};

use super::state::TaskState;
use crate::kernel::memory::{
    MAX_HEAP_MAPPINGS, USER_HEAP_START, USER_HEAP_END,
    MappingKind, HeapMapping, MapFlags, MapSource, MapResult,
};

/// Magic value written to the bottom of every kernel stack at allocation time.
/// Checked at context switch and slot reap to detect stack overflow/corruption.
pub const STACK_CANARY: u64 = 0xCA4A_B1E5_DEAD_BEEF;

/// Number of priority levels
pub const NUM_PRIORITIES: usize = 8;

/// Task priority levels (8 levels, lower number = higher priority)
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Priority {
    /// Realtime - hardware interrupt handlers (future)
    Realtime = 0,
    /// Critical - devd, consoled — must always respond
    Critical = 1,
    /// High - device drivers (pcied, usbd, wifid, nvmed)
    High = 2,
    /// Above normal - driver children (partd, fatfsd, switchd)
    AboveNorm = 3,
    /// Normal - default for shell, user apps
    Normal = 4,
    /// Below normal - background services
    BelowNorm = 5,
    /// Low - batch / best-effort
    Low = 6,
    /// Idle - only runs when nothing else ready
    Idle = 7,
}

impl Priority {
    /// Convert from u8, returning None for invalid values
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(Priority::Realtime),
            1 => Some(Priority::Critical),
            2 => Some(Priority::High),
            3 => Some(Priority::AboveNorm),
            4 => Some(Priority::Normal),
            5 => Some(Priority::BelowNorm),
            6 => Some(Priority::Low),
            7 => Some(Priority::Idle),
            _ => None,
        }
    }
}

impl Default for Priority {
    fn default() -> Self {
        Priority::Normal
    }
}

/// Maximum concurrent priority inheritance boosters per task.
/// Bounded to prevent unbounded chains — deeper PI is rare in practice.
pub const MAX_PI_BOOSTERS: usize = 4;

/// A priority inheritance boost from a blocked higher-priority task.
#[derive(Debug, Clone, Copy)]
pub struct PiBooster {
    /// PID of the task that caused this boost (INVALID_PID = empty slot)
    pub task_id: u32,
    /// The effective priority of the boosting task at boost time
    pub priority: Priority,
}

impl PiBooster {
    /// Sentinel value for empty booster slots.
    /// Must NOT be a valid PID (0 is used by idle tasks).
    const INVALID_PID: u32 = u32::MAX;

    pub const EMPTY: Self = Self { task_id: Self::INVALID_PID, priority: Priority::Idle };

    pub fn is_empty(&self) -> bool {
        self.task_id == Self::INVALID_PID
    }
}

/// Trap frame saved on kernel stack during exceptions
/// Must match the layout in boot.S exactly!
/// This captures the full user-mode state.
#[repr(C)]
pub struct TrapFrame {
    // General purpose registers (saved in reverse order by stp)
    pub x0: u64,
    pub x1: u64,
    pub x2: u64,
    pub x3: u64,
    pub x4: u64,
    pub x5: u64,
    pub x6: u64,
    pub x7: u64,
    pub x8: u64,
    pub x9: u64,
    pub x10: u64,
    pub x11: u64,
    pub x12: u64,
    pub x13: u64,
    pub x14: u64,
    pub x15: u64,
    pub x16: u64,
    pub x17: u64,
    pub x18: u64,
    pub x19: u64,
    pub x20: u64,
    pub x21: u64,
    pub x22: u64,
    pub x23: u64,
    pub x24: u64,
    pub x25: u64,
    pub x26: u64,
    pub x27: u64,
    pub x28: u64,
    pub x29: u64,   // Frame pointer
    pub x30: u64,   // Link register
    // Special registers
    pub sp_el0: u64,   // User stack pointer
    pub elr_el1: u64,  // Exception return address (user PC)
    pub spsr_el1: u64, // Saved program status
}

impl TrapFrame {
    pub const fn new() -> Self {
        Self {
            x0: 0, x1: 0, x2: 0, x3: 0, x4: 0, x5: 0, x6: 0, x7: 0,
            x8: 0, x9: 0, x10: 0, x11: 0, x12: 0, x13: 0, x14: 0, x15: 0,
            x16: 0, x17: 0, x18: 0, x19: 0, x20: 0, x21: 0, x22: 0, x23: 0,
            x24: 0, x25: 0, x26: 0, x27: 0, x28: 0, x29: 0, x30: 0,
            sp_el0: 0, elr_el1: 0, spsr_el1: 0,
        }
    }

    /// Initialize for user mode entry
    /// SPSR: EL0t (user mode), all interrupts enabled
    pub fn init_user(&mut self, entry: u64, user_stack: u64) {
        self.elr_el1 = entry;
        self.sp_el0 = user_stack;
        // SPSR for EL0t: M[4:0] = 0b00000 (EL0t), DAIF = 0 (interrupts enabled)
        self.spsr_el1 = 0;
    }
}

/// Saved CPU context for kernel-to-kernel context switching
/// Used for kernel threads (not user processes)
#[repr(C)]
pub struct CpuContext {
    // Callee-saved registers (x19-x30)
    pub x19: u64,
    pub x20: u64,
    pub x21: u64,
    pub x22: u64,
    pub x23: u64,
    pub x24: u64,
    pub x25: u64,
    pub x26: u64,
    pub x27: u64,
    pub x28: u64,
    pub x29: u64,  // Frame pointer
    pub x30: u64,  // Link register (return address)
    pub sp: u64,   // Stack pointer
    pub pc: u64,   // Program counter (entry point for new tasks)
}

impl CpuContext {
    pub const fn new() -> Self {
        Self {
            x19: 0, x20: 0, x21: 0, x22: 0,
            x23: 0, x24: 0, x25: 0, x26: 0,
            x27: 0, x28: 0, x29: 0, x30: 0,
            sp: 0, pc: 0,
        }
    }
}

/// Task ID type
pub type TaskId = u32;

/// Stack size for kernel tasks (64KB)
/// Needs to be large enough for functions that create large local variables
/// (e.g., MessageQueue is ~4800 bytes, Channel is ~5000 bytes)
pub const KERNEL_STACK_SIZE: usize = 64 * 1024;

/// Guard page size (4KB) - placed at bottom of kernel stack
pub const GUARD_PAGE_SIZE: usize = 4096;

/// Maximum number of children a process can have
pub const MAX_CHILDREN: usize = 16;

/// Maximum number of PIDs allowed to send signals to this process
pub const MAX_SIGNAL_SENDERS: usize = 8;

/// Maximum pending signals per task
pub const MAX_PENDING_SIGNALS: usize = 8;

/// Per-process resource limits (prevent exhaustion attacks)
pub const MAX_CHANNELS_PER_TASK: u16 = 32;
pub const MAX_PORTS_PER_TASK: u16 = 4;
pub const MAX_SHMEM_PER_TASK: u16 = 16;

/// Number of timer ticks to wait between Phase 1 (notify) and Phase 2 (force cleanup)
/// At 100 ticks/sec, 10 ticks = 100ms grace period for servers to release resources
pub const CLEANUP_GRACE_TICKS: u8 = 10;

/// Cleanup phase for exiting tasks — tracks progress through microtask-driven cleanup.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CleanupPhase {
    /// Not exiting
    None,
    /// Phase 1 microtasks submitted (IpcCleanup, KillChildren, etc.)
    Phase1Enqueued,
    /// Phase 1 done, waiting for servers to process notifications
    GracePeriod { until: u64 },
    /// Phase 2 microtasks submitted (FinalCleanup, SlotReap)
    Phase2Enqueued,
}

/// Data delivered with a wake — allows Mux fast path to skip re-poll.
#[derive(Debug, Clone, Copy)]
pub struct WakeData {
    /// Which handle triggered the event
    pub handle: u32,
    /// Event bitmask (READABLE, WRITABLE, CLOSED, etc.)
    pub events: u8,
}

/// Context restore state — tracks whether a task's kernel context needs restoring.
///
/// Replaces the old `needs_context_restore: bool` with explicit states:
/// - `Fresh` — task was just created or its context was restored
/// - `Saved` — task's kernel context was saved by context_switch and needs restoring
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ContextRestoreState {
    /// Task was just created or context was restored — no pending restore needed
    Fresh,
    /// Task's kernel context was saved by context_switch and needs restoring
    Saved,
}

/// Task Control Block
///
/// Fields are private to enforce access through methods.
/// Use accessor methods for reading and state machine methods for transitions.
pub struct Task {
    // ========================================================================
    // Immutable identifiers (public - safe to read directly)
    // ========================================================================

    /// Unique task ID
    pub id: TaskId,
    /// Parent task ID (0 for orphan/init)
    pub parent_id: TaskId,
    /// Is this a user-mode task?
    pub is_user: bool,
    /// Kernel stack base (physical address, after guard page)
    pub kernel_stack: u64,
    /// Kernel stack size (not including guard)
    pub kernel_stack_size: usize,
    /// Guard page physical address (for debugging overflows)
    pub guard_page: u64,

    // ========================================================================
    // Private state - access via methods only
    // ========================================================================

    /// Current state - use state() accessor and transition methods
    /// PRIVATE: all access must go through state machine methods
    state: TaskState,
    /// Base scheduling priority - set at spawn, changed by supervisor
    pub(crate) base_priority: Priority,
    /// Effective scheduling priority - min(base, PI boosts) — used by scheduler
    pub(crate) effective_priority: Priority,
    /// Priority inheritance boosters (tasks whose blocked read boosted us)
    pub(crate) pi_boosters: [PiBooster; MAX_PI_BOOSTERS],
    /// Number of active PI boosters
    pub(crate) pi_booster_count: u8,
    /// PIDs of tasks we are currently boosting (for fast unboost on wake)
    pub(crate) pi_targets: [u32; MAX_PI_BOOSTERS],
    /// Number of active PI targets
    pub(crate) pi_target_count: u8,
    /// Saved CPU context (for kernel threads) - internal use only
    pub(crate) context: CpuContext,
    /// Saved trap frame (for user processes) - use trap_frame() / trap_frame_mut()
    pub(crate) trap_frame: TrapFrame,
    /// User address space (None for kernel tasks)
    pub(crate) address_space: Option<AddressSpace>,
    /// Is this the init process (devd)?
    pub(crate) is_init: bool,
    /// Is this the probed process (boot-time bus discovery)?
    pub is_probed: bool,
    /// Task name for debugging
    pub(crate) name: [u8; 16],
    /// Heap mappings for mmap/munmap tracking
    pub(crate) heap_mappings: [HeapMapping; MAX_HEAP_MAPPINGS],
    /// Next heap address for bump allocation
    pub(crate) heap_next: u64,
    // NOTE: object_table removed - now owned by ObjectService
    /// Per-process resource counters
    pub(crate) channel_count: u16,
    pub(crate) port_count: u16,
    pub(crate) shmem_count: u16,
    /// Child task IDs
    pub(crate) children: [TaskId; MAX_CHILDREN],
    /// Number of children
    pub(crate) num_children: usize,
    /// Capability set for this task
    pub(crate) capabilities: crate::kernel::caps::Capabilities,
    /// Signal allowlist - PIDs allowed to send signals
    pub(crate) signal_allowlist: [u32; MAX_SIGNAL_SENDERS],
    /// Number of entries in signal allowlist
    pub(crate) signal_allowlist_count: usize,
    /// Liveness tracking state
    pub(crate) liveness_state: crate::kernel::liveness::LivenessState,
    /// Last activity tick (syscall made)
    pub(crate) last_activity_tick: u64,
    /// Storm protection state (syscall/wake rate limiting)
    pub(crate) storm: crate::kernel::storm::StormState,
    /// Whether this task needs its kernel context restored via context_switch.
    /// Set to Saved when task blocks and context_switch saves its context.
    /// Set to Fresh when context_switch restores its context.
    context_restore: ContextRestoreState,

    /// SMP exclusivity: which CPU is currently using this task's kernel stack?
    ///
    /// Set in Phase 1 of context switch (before releasing scheduler lock).
    /// Cleared in Phase 3 of context switch (after switch completes).
    ///
    /// A task is selectable iff:
    /// - `state == Ready`
    /// - `kernel_stack_owner.is_none()`
    ///
    /// This prevents the bug where CPU 0 is mid-context-switch using a task's
    /// kernel stack while CPU 1 tries to select and run the same task.
    kernel_stack_owner: Option<u32>,

    /// Cleanup phase for exiting tasks (microtask-driven)
    pub(crate) cleanup_phase: CleanupPhase,

    /// IPC messages sent by this task (saturating counter)
    pub(crate) ipc_sent: u32,
    /// IPC messages received by this task (saturating counter)
    pub(crate) ipc_recv: u32,
    /// Context switches into this task (saturating counter)
    pub(crate) context_switches: u32,
    /// Page faults handled for this task (demand paging)
    pub(crate) page_faults: u32,

    /// Total CPU time consumed by this task (nanoseconds)
    pub(crate) cpu_time_ns: u64,
    /// Counter snapshot when this task was last scheduled to run
    pub(crate) last_scheduled_at: u64,

    /// Data delivered with the last wake (for Mux fast path)
    pub(crate) wake_data: Option<WakeData>,

    /// Per-task signal queue (fixed ring buffer)
    pub(crate) signal_queue: [abi::PendingSignal; MAX_PENDING_SIGNALS],
    pub(crate) signal_head: u8,  // next write position
    pub(crate) signal_tail: u8,  // next read position
    pub(crate) signal_count: u8, // current count

    /// Debug invariant: is this task currently in a ready queue?
    /// Used to assert that tasks are enqueued/dequeued correctly.
    #[cfg(debug_assertions)]
    on_runq: bool,
}

/// Trampoline for new user tasks.
///
/// When a new user task is first scheduled via context_switch, it ret's here.
/// This function sets up the task globals and erets into userspace.
///
/// # Safety
/// Must only be called via context_switch with proper scheduler state set up.
fn user_task_trampoline() -> ! {
    // CRITICAL: Process pending_stack_release from the context_switch that
    // brought us here. Since this is a new task (first schedule), context_switch
    // ret'd to this trampoline instead of back to reschedule_inner's Phase 3.
    // Without this, the from-task stays stuck (Running never→Ready, or
    // kernel_stack_owner never cleared).
    crate::kernel::sched::process_pending_from_switch();

    // Get current slot (was set by switch_to_task before context_switch)
    let slot = crate::kernel::percpu::cpu_local().get_current_slot();

    // CRITICAL: Extract data while holding lock, then RELEASE LOCK before enter_usermode.
    // If we call enter_usermode while holding the lock, it never returns (eret),
    // so the lock would never be released, causing deadlock on next scheduler() call.
    let (trap_frame, ttbr0, kstack_top) = {
        let mut sched = super::scheduler();

        let task = match sched.tasks[slot].as_mut() {
            Some(t) => t,
            None => panic!("user_task_trampoline: no task in slot {}", slot),
        };

        let addr_space = match task.address_space.as_ref() {
            Some(a) => a,
            None => panic!("user_task_trampoline: no address space for task {}", task.id),
        };

        // Extract data needed for usermode entry
        let ttbr0 = addr_space.get_ttbr0();
        let trap_frame = &mut task.trap_frame as *mut TrapFrame;

        // Kernel stack top (virtual) - becomes SP_EL1 for this task's exceptions
        let kstack_top = crate::kernel::arch::mmu::phys_to_virt(
            task.kernel_stack + task.kernel_stack_size as u64
        );

        // Set per-CPU data for exception handler
        crate::kernel::percpu::set_trap_frame(trap_frame);
        crate::kernel::percpu::set_ttbr0(ttbr0);
        crate::kernel::percpu::set_kernel_stack_top(kstack_top);

        (trap_frame as *const TrapFrame, ttbr0, kstack_top)
        // sched guard dropped here - lock released
    };

    // Enter userspace via eret (this never returns)
    unsafe {
        super::enter_usermode(trap_frame, ttbr0, kstack_top);
    }
}

impl Task {
    /// Create a new kernel task
    pub fn new_kernel(id: TaskId, entry: fn() -> !, name: &str) -> Option<Self> {
        // Allocate kernel stack + guard page
        let total_pages = (KERNEL_STACK_SIZE / 4096) + 1;
        let alloc_base = pmm::alloc_pages(total_pages)?;

        let guard_page = alloc_base;
        let stack_base = alloc_base + GUARD_PAGE_SIZE;
        let stack_top = stack_base + KERNEL_STACK_SIZE;

        // Write canary at the bottom of the stack to detect overflow/corruption
        unsafe {
            let canary_virt = mmu::phys_to_virt(stack_base as u64) as *mut u64;
            core::ptr::write_volatile(canary_virt, STACK_CANARY);
        }

        let mut context = CpuContext::new();
        context.sp = stack_top as u64;
        context.pc = entry as u64;
        context.x30 = entry as u64;

        let mut task_name = [0u8; 16];
        let name_bytes = name.as_bytes();
        let copy_len = core::cmp::min(name_bytes.len(), 15);
        task_name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        Some(Self {
            id,
            state: TaskState::Ready,
            base_priority: Priority::Normal,
            effective_priority: Priority::Normal,
            pi_boosters: [PiBooster::EMPTY; MAX_PI_BOOSTERS],
            pi_booster_count: 0,
            pi_targets: [0; MAX_PI_BOOSTERS],
            pi_target_count: 0,
            context,
            trap_frame: TrapFrame::new(),
            kernel_stack: stack_base as u64,
            kernel_stack_size: KERNEL_STACK_SIZE,
            guard_page: guard_page as u64,
            address_space: None,
            is_user: false,
            is_init: false,
            is_probed: false,
            name: task_name,
            heap_mappings: [HeapMapping::empty(); MAX_HEAP_MAPPINGS],
            heap_next: USER_HEAP_START,
            channel_count: 0,
            port_count: 0,
            shmem_count: 0,

            parent_id: 0,
            children: [0; MAX_CHILDREN],
            num_children: 0,

            capabilities: crate::kernel::caps::Capabilities::ALL,
            signal_allowlist: [0; MAX_SIGNAL_SENDERS],
            signal_allowlist_count: 0,
            liveness_state: crate::kernel::liveness::LivenessState::Normal,
            last_activity_tick: 0,
            storm: crate::kernel::storm::StormState::new(),
            context_restore: ContextRestoreState::Saved,  // Kernel task always uses CpuContext
            kernel_stack_owner: None,
            cleanup_phase: CleanupPhase::None,
            ipc_sent: 0,
            ipc_recv: 0,
            context_switches: 0,
            page_faults: 0,
            cpu_time_ns: 0,
            last_scheduled_at: 0,
            wake_data: None,
            signal_queue: [abi::PendingSignal { event: 0, value: 0 }; MAX_PENDING_SIGNALS],
            signal_head: 0,
            signal_tail: 0,
            signal_count: 0,
            #[cfg(debug_assertions)]
            on_runq: false,
        })
    }

    /// Create the idle task with a static stack (no PMM allocation)
    ///
    /// This is special because:
    /// 1. The idle task is created before PMM is initialized
    /// 2. It uses a statically allocated stack from kernel/idle.rs
    /// 3. There's only ever one idle task (in slot 0)
    ///
    /// # Safety
    /// Must only be called once, during scheduler initialization.
    pub unsafe fn new_idle(id: TaskId, entry: fn() -> !, cpu: u32) -> Self {
        // Use per-CPU static stack from idle module
        let stack_top = crate::kernel::idle::idle_stack_top_for_cpu(cpu);

        let mut context = CpuContext::new();
        context.sp = stack_top as u64;
        context.pc = entry as u64;
        context.x30 = entry as u64;

        Self {
            id,
            state: TaskState::Ready,
            base_priority: Priority::Idle,
            effective_priority: Priority::Idle,
            pi_boosters: [PiBooster::EMPTY; MAX_PI_BOOSTERS],
            pi_booster_count: 0,
            pi_targets: [0; MAX_PI_BOOSTERS],
            pi_target_count: 0,
            context,
            trap_frame: TrapFrame::new(),
            kernel_stack: 0,  // Static stack, not from PMM
            kernel_stack_size: 4096,  // IDLE_STACK size
            guard_page: 0,  // No guard page for static stack
            address_space: None,
            is_user: false,
            is_init: false,
            is_probed: false,
            name: *b"idle\0\0\0\0\0\0\0\0\0\0\0\0",
            heap_mappings: [HeapMapping::empty(); MAX_HEAP_MAPPINGS],
            heap_next: USER_HEAP_START,
            channel_count: 0,
            port_count: 0,
            shmem_count: 0,

            parent_id: 0,
            children: [0; MAX_CHILDREN],
            num_children: 0,

            capabilities: crate::kernel::caps::Capabilities::NONE,  // No capabilities needed
            signal_allowlist: [0; MAX_SIGNAL_SENDERS],
            signal_allowlist_count: 0,
            liveness_state: crate::kernel::liveness::LivenessState::Normal,
            last_activity_tick: 0,
            storm: crate::kernel::storm::StormState::new(),
            context_restore: ContextRestoreState::Saved,  // Kernel task always uses CpuContext
            kernel_stack_owner: None,
            cleanup_phase: CleanupPhase::None,
            ipc_sent: 0,
            ipc_recv: 0,
            context_switches: 0,
            page_faults: 0,
            cpu_time_ns: 0,
            last_scheduled_at: 0,
            wake_data: None,
            signal_queue: [abi::PendingSignal { event: 0, value: 0 }; MAX_PENDING_SIGNALS],
            signal_head: 0,
            signal_tail: 0,
            signal_count: 0,
            #[cfg(debug_assertions)]
            on_runq: false,
        }
    }

    /// Create a new user task with its own address space
    ///
    /// The task's CpuContext is initialized to point to `user_task_trampoline`,
    /// which will set up globals and eret to userspace when first scheduled.
    pub fn new_user(id: TaskId, name: &str) -> Option<Self> {
        let total_pages = (KERNEL_STACK_SIZE / 4096) + 1;
        let alloc_base = pmm::alloc_pages(total_pages)?;

        let guard_page = alloc_base;
        let stack_base = alloc_base + GUARD_PAGE_SIZE;
        let stack_top = stack_base + KERNEL_STACK_SIZE;

        // Write canary at the bottom of the stack to detect overflow/corruption
        unsafe {
            let canary_virt = mmu::phys_to_virt(stack_base as u64) as *mut u64;
            core::ptr::write_volatile(canary_virt, STACK_CANARY);
        }

        // Convert to kernel virtual address (MMU is enabled)
        let stack_top_virt = crate::kernel::arch::mmu::phys_to_virt(stack_top as u64);

        let address_space = match AddressSpace::new() {
            Some(addr_space) => addr_space,
            None => {
                pmm::free_pages(alloc_base, total_pages);
                return None;
            }
        };

        // Initialize CpuContext to point to the user task entry trampoline.
        // When context_switched to, ret will jump to the trampoline which
        // sets up globals and erets to userspace.
        let mut context = CpuContext::new();
        context.x30 = user_task_trampoline as *const () as u64;
        context.sp = stack_top_virt;

        let trap_frame = TrapFrame::new();

        let mut task_name = [0u8; 16];
        let name_bytes = name.as_bytes();
        let copy_len = core::cmp::min(name_bytes.len(), 15);
        task_name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        Some(Self {
            id,
            state: TaskState::Ready,
            base_priority: Priority::Normal,
            effective_priority: Priority::Normal,
            pi_boosters: [PiBooster::EMPTY; MAX_PI_BOOSTERS],
            pi_booster_count: 0,
            pi_targets: [0; MAX_PI_BOOSTERS],
            pi_target_count: 0,
            context,
            trap_frame,
            kernel_stack: stack_base as u64,
            kernel_stack_size: KERNEL_STACK_SIZE,
            guard_page: guard_page as u64,
            address_space: Some(address_space),
            is_user: true,
            is_init: false,
            is_probed: false,
            name: task_name,
            heap_mappings: [HeapMapping::empty(); MAX_HEAP_MAPPINGS],
            heap_next: USER_HEAP_START,
            channel_count: 0,
            port_count: 0,
            shmem_count: 0,

            parent_id: 0,
            children: [0; MAX_CHILDREN],
            num_children: 0,

            capabilities: crate::kernel::caps::Capabilities::DRIVER_DEFAULT,
            signal_allowlist: [0; MAX_SIGNAL_SENDERS],
            signal_allowlist_count: 0,
            liveness_state: crate::kernel::liveness::LivenessState::Normal,
            last_activity_tick: 0,
            storm: crate::kernel::storm::StormState::new(),
            context_restore: ContextRestoreState::Saved,
            kernel_stack_owner: None,
            cleanup_phase: CleanupPhase::None,
            ipc_sent: 0,
            ipc_recv: 0,
            context_switches: 0,
            page_faults: 0,
            cpu_time_ns: 0,
            last_scheduled_at: 0,
            wake_data: None,
            signal_queue: [abi::PendingSignal { event: 0, value: 0 }; MAX_PENDING_SIGNALS],
            signal_head: 0,
            signal_tail: 0,
            signal_count: 0,
            #[cfg(debug_assertions)]
            on_runq: false,
        })
    }

    /// Set task capabilities
    pub fn set_capabilities(&mut self, caps: crate::kernel::caps::Capabilities) {
        self.capabilities = caps;
    }

    /// Check if task has a capability
    pub fn has_capability(&self, cap: crate::kernel::caps::Capabilities) -> bool {
        self.capabilities.has(cap)
    }

    /// Check if task can create another channel
    #[inline]
    pub fn can_create_channel(&self) -> bool {
        self.channel_count < MAX_CHANNELS_PER_TASK
    }

    #[inline]
    pub fn add_channel(&mut self) {
        self.channel_count = self.channel_count.saturating_add(1);
    }

    #[inline]
    pub fn remove_channel(&mut self) {
        self.channel_count = self.channel_count.saturating_sub(1);
    }

    #[inline]
    pub fn can_create_port(&self) -> bool {
        self.port_count < MAX_PORTS_PER_TASK
    }

    #[inline]
    pub fn add_port(&mut self) {
        self.port_count = self.port_count.saturating_add(1);
    }

    #[inline]
    pub fn remove_port(&mut self) {
        self.port_count = self.port_count.saturating_sub(1);
    }

    #[inline]
    pub fn can_create_shmem(&self) -> bool {
        self.shmem_count < MAX_SHMEM_PER_TASK
    }

    #[inline]
    pub fn add_shmem(&mut self) {
        self.shmem_count = self.shmem_count.saturating_add(1);
    }

    #[inline]
    pub fn remove_shmem(&mut self) {
        self.shmem_count = self.shmem_count.saturating_sub(1);
    }

    /// Add a PID to the signal allowlist
    pub fn allow_signals_from(&mut self, sender_pid: u32) -> bool {
        for i in 0..self.signal_allowlist_count {
            if self.signal_allowlist[i] == sender_pid {
                return true;
            }
        }

        if self.signal_allowlist_count < MAX_SIGNAL_SENDERS {
            self.signal_allowlist[self.signal_allowlist_count] = sender_pid;
            self.signal_allowlist_count += 1;
            true
        } else {
            false
        }
    }

    /// Check if a sender PID is allowed to send signals
    pub fn can_receive_signal_from(&self, sender_pid: u32) -> bool {
        if self.signal_allowlist_count == 0 {
            return true;
        }

        if sender_pid == self.parent_id && self.parent_id != 0 {
            return true;
        }

        for i in 0..self.signal_allowlist_count {
            if self.signal_allowlist[i] == sender_pid {
                return true;
            }
        }

        false
    }

    /// Set base priority. Recomputes effective priority.
    /// Returns the old effective priority (for policy notification).
    pub fn set_priority(&mut self, priority: Priority) -> Priority {
        let old_effective = self.effective_priority;
        self.base_priority = priority;
        self.recompute_effective_priority();
        old_effective
    }

    /// Get the effective priority (used by scheduler for task selection).
    #[inline]
    pub fn effective_priority(&self) -> Priority {
        self.effective_priority
    }

    /// Get the base priority.
    #[inline]
    pub fn base_priority(&self) -> Priority {
        self.base_priority
    }

    /// Recompute effective_priority from base_priority and PI boosts.
    /// Lower enum value = higher priority, so we use min().
    fn recompute_effective_priority(&mut self) {
        let mut best = self.base_priority;
        for i in 0..self.pi_booster_count as usize {
            if !self.pi_boosters[i].is_empty() && self.pi_boosters[i].priority < best {
                best = self.pi_boosters[i].priority;
            }
        }
        self.effective_priority = best;
    }

    /// Add a priority inheritance boost from a blocking task.
    /// Returns true if effective priority changed (caller should notify policy).
    pub fn add_pi_boost(&mut self, booster_pid: u32, booster_priority: Priority) -> bool {
        // Don't boost if booster has same or lower priority
        if booster_priority >= self.effective_priority {
            return false;
        }
        // Check for existing boost from this task
        for i in 0..self.pi_booster_count as usize {
            if self.pi_boosters[i].task_id == booster_pid {
                // Update existing boost
                self.pi_boosters[i].priority = booster_priority;
                let old = self.effective_priority;
                self.recompute_effective_priority();
                return self.effective_priority != old;
            }
        }
        // Add new booster if space available
        if (self.pi_booster_count as usize) < MAX_PI_BOOSTERS {
            self.pi_boosters[self.pi_booster_count as usize] = PiBooster {
                task_id: booster_pid,
                priority: booster_priority,
            };
            self.pi_booster_count += 1;
            let old = self.effective_priority;
            self.recompute_effective_priority();
            return self.effective_priority != old;
        }
        false
    }

    /// Remove a priority inheritance boost from a specific task.
    /// Returns true if effective priority changed (caller should notify policy).
    pub fn remove_pi_boost(&mut self, booster_pid: u32) -> bool {
        for i in 0..self.pi_booster_count as usize {
            if self.pi_boosters[i].task_id == booster_pid {
                // Swap-remove
                let last = self.pi_booster_count as usize - 1;
                self.pi_boosters[i] = self.pi_boosters[last];
                self.pi_boosters[last] = PiBooster::EMPTY;
                self.pi_booster_count -= 1;
                let old = self.effective_priority;
                self.recompute_effective_priority();
                return self.effective_priority != old;
            }
        }
        false
    }

    /// Remove all PI boosts on us (used during task exit cleanup).
    pub fn clear_pi_boosts(&mut self) {
        self.pi_boosters = [PiBooster::EMPTY; MAX_PI_BOOSTERS];
        self.pi_booster_count = 0;
        self.recompute_effective_priority();
    }

    /// Record that we are boosting target_pid. Called on the blocker task.
    pub fn add_pi_target(&mut self, target_pid: u32) {
        if (self.pi_target_count as usize) < MAX_PI_BOOSTERS {
            self.pi_targets[self.pi_target_count as usize] = target_pid;
            self.pi_target_count += 1;
        }
    }

    /// Get the list of tasks we are currently boosting.
    pub fn pi_targets(&self) -> &[u32] {
        &self.pi_targets[..self.pi_target_count as usize]
    }

    /// Clear our PI target list (called when we wake up).
    pub fn clear_pi_targets(&mut self) {
        self.pi_targets = [0; MAX_PI_BOOSTERS];
        self.pi_target_count = 0;
    }

    /// Record that this task is starting to run now.
    #[inline]
    pub fn mark_scheduled(&mut self, now_ns: u64) {
        self.last_scheduled_at = now_ns;
    }

    /// Account CPU time since last scheduled, return elapsed ns.
    #[inline]
    pub fn account_cpu_time(&mut self, now_ns: u64) -> u64 {
        let elapsed = now_ns.saturating_sub(self.last_scheduled_at);
        self.cpu_time_ns += elapsed;
        elapsed
    }

    /// Get total CPU time in nanoseconds.
    #[inline]
    pub fn cpu_time_ns(&self) -> u64 {
        self.cpu_time_ns
    }

    /// Reset all per-task statistics counters.
    pub fn reset_stats(&mut self) {
        self.cpu_time_ns = 0;
        self.ipc_sent = 0;
        self.ipc_recv = 0;
        self.context_switches = 0;
        self.page_faults = 0;
        self.last_activity_tick = 0;
    }

    pub fn set_parent(&mut self, parent_id: TaskId) {
        self.parent_id = parent_id;
    }

    pub fn add_child(&mut self, child_id: TaskId) -> bool {
        if self.num_children >= MAX_CHILDREN {
            return false;
        }
        self.children[self.num_children] = child_id;
        self.num_children += 1;
        true
    }

    pub fn remove_child(&mut self, child_id: TaskId) -> bool {
        for i in 0..self.num_children {
            if self.children[i] == child_id {
                for j in i..self.num_children - 1 {
                    self.children[j] = self.children[j + 1];
                }
                self.children[self.num_children - 1] = 0;
                self.num_children -= 1;
                return true;
            }
        }
        false
    }

    pub fn has_children(&self) -> bool {
        self.num_children > 0
    }

    pub fn set_user_entry(&mut self, entry: u64, user_stack: u64) {
        self.trap_frame.init_user(entry, user_stack);
    }

    pub fn name_str(&self) -> &str {
        let len = self.name.iter().position(|&c| c == 0).unwrap_or(16);
        core::str::from_utf8(&self.name[..len]).unwrap_or("???")
    }

    pub fn address_space_mut(&mut self) -> Option<&mut AddressSpace> {
        self.address_space.as_mut()
    }

    /// Verify that the stack canary at the bottom of this task's kernel stack is intact.
    /// Returns true if the canary is valid (or this is a static-stack idle task).
    /// Returns false if the canary has been corrupted (stack overflow or use-after-free).
    pub fn verify_stack_canary(&self) -> bool {
        // Idle tasks have static stacks (kernel_stack == 0), no canary written
        if self.kernel_stack == 0 {
            return true;
        }
        unsafe {
            let canary_virt = mmu::phys_to_virt(self.kernel_stack) as *const u64;
            core::ptr::read_volatile(canary_virt) == STACK_CANARY
        }
    }

    /// Take wake data (consumes it — returns None on second call).
    pub fn take_wake_data(&mut self) -> Option<WakeData> {
        self.wake_data.take()
    }

    // ========================================================================
    // Signal Queue API
    // ========================================================================

    /// Enqueue a signal into this task's signal queue. Returns false if full.
    pub fn enqueue_signal(&mut self, event: u32, value: u64) -> bool {
        debug_assert!(event <= 255, "signal event {} exceeds u8 range for MuxEvent delivery", event);
        if self.signal_count >= MAX_PENDING_SIGNALS as u8 {
            return false;
        }
        self.signal_queue[self.signal_head as usize] = abi::PendingSignal { event, value };
        self.signal_head = (self.signal_head + 1) % MAX_PENDING_SIGNALS as u8;
        self.signal_count += 1;
        true
    }

    /// Dequeue a signal from this task's signal queue.
    pub fn dequeue_signal(&mut self) -> Option<abi::PendingSignal> {
        if self.signal_count == 0 {
            return None;
        }
        let sig = self.signal_queue[self.signal_tail as usize];
        self.signal_tail = (self.signal_tail + 1) % MAX_PENDING_SIGNALS as u8;
        self.signal_count -= 1;
        Some(sig)
    }

    /// Check if this task has pending signals.
    pub fn has_pending_signals(&self) -> bool {
        self.signal_count > 0
    }

    // ========================================================================
    // State Machine API
    // ========================================================================
    // All state changes MUST go through these methods.
    // Direct access to the state field is not allowed.

    /// Get current task state (read-only)
    #[inline]
    pub fn state(&self) -> &TaskState {
        &self.state
    }

    /// Check if task is runnable (Ready or Running)
    #[inline]
    pub fn is_runnable(&self) -> bool {
        self.state.is_runnable()
    }

    /// Check if task is blocked (Sleeping or Waiting)
    #[inline]
    pub fn is_blocked(&self) -> bool {
        self.state.is_blocked()
    }

    /// Check if task is terminated (Exiting, Dying, or Dead)
    #[inline]
    pub fn is_terminated(&self) -> bool {
        self.state.is_terminated()
    }

    /// Transition: Ready → Running (scheduled to run on specific CPU)
    #[inline]
    pub fn set_running(&mut self, cpu: u32) -> Result<(), super::state::InvalidTransition> {
        self.state.schedule(cpu)
    }

    /// Transition: Running → Ready (yield or preempt)
    #[inline]
    pub fn set_ready(&mut self) -> Result<(), super::state::InvalidTransition> {
        self.state.yield_cpu()
    }

    /// Transition: Running → Sleeping (block for event, no deadline)
    #[inline]
    pub fn set_sleeping(&mut self, reason: super::state::SleepReason) -> Result<(), super::state::InvalidTransition> {
        self.state.sleep(reason)
    }

    /// Transition: Running → Waiting (block with deadline)
    #[inline]
    pub fn set_waiting(&mut self, reason: super::state::WaitReason, deadline: u64) -> Result<(), super::state::InvalidTransition> {
        self.state.wait(reason, deadline)
    }

    /// Transition: Sleeping/Waiting → Ready (wake up)
    #[inline]
    pub fn wake(&mut self) -> Result<(), super::state::InvalidTransition> {
        self.state.wake()
    }

    /// Transition: Any runnable/blocked → Exiting
    #[inline]
    pub fn set_exiting(&mut self, code: i32) -> Result<(), super::state::InvalidTransition> {
        self.state.exit(code)
    }

    /// Transition: Exiting → Dying (start grace period)
    #[inline]
    pub fn set_dying(&mut self, until: u64) -> Result<i32, super::state::InvalidTransition> {
        self.state.start_dying(until)
    }

    /// Transition: Exiting/Dying → Dead (cleanup complete)
    #[inline]
    pub fn set_dead(&mut self) -> Result<i32, super::state::InvalidTransition> {
        self.state.finalize()
    }

    /// Transition: Any non-terminal → Evicting (kernel-initiated forced termination)
    #[inline]
    pub fn evict(&mut self, reason: super::state::EvictionReason) -> Result<(), super::state::InvalidTransition> {
        self.state.evict(reason)
    }

    /// Transition: Exiting/Dying/Evicting → Dead (cleanup complete)
    #[inline]
    pub fn finalize(&mut self) -> Result<i32, super::state::InvalidTransition> {
        self.state.finalize()
    }

    /// Force state to Ready (for task slot reuse after Dead)
    /// This bypasses the state machine - only use for slot reinitialization
    #[inline]
    pub(crate) fn reset_state_for_reuse(&mut self) {
        self.state = TaskState::Ready;
    }

    // ========================================================================
    // Syscall-Facing Encapsulation API
    // ========================================================================
    // These methods hide internal field access from syscall handlers.
    // Syscalls should use these instead of accessing fields directly.

    // ========================================================================
    // Context Restore State API
    // ========================================================================

    /// Mark that this task's kernel context was saved and needs restoring.
    #[inline]
    pub(crate) fn mark_context_saved(&mut self) {
        self.context_restore = ContextRestoreState::Saved;
    }

    /// Mark that this task's kernel context was restored (or is fresh).
    #[inline]
    pub(crate) fn mark_context_restored(&mut self) {
        self.context_restore = ContextRestoreState::Fresh;
    }

    /// Check whether this task needs its kernel context restored via context_switch.
    #[inline]
    pub(crate) fn needs_context_restore(&self) -> bool {
        self.context_restore == ContextRestoreState::Saved
    }

    // ========================================================================
    // Kernel Stack Ownership API (SMP Exclusivity)
    // ========================================================================

    /// Set the CPU that owns this task's kernel stack.
    /// Called in Phase 1 of context switch before releasing the scheduler lock.
    #[inline]
    pub(crate) fn set_kernel_stack_owner(&mut self, cpu: u32) {
        self.kernel_stack_owner = Some(cpu);
    }

    /// Clear kernel stack ownership.
    /// Called in Phase 3 of context switch after the switch completes.
    #[inline]
    pub(crate) fn clear_kernel_stack_owner(&mut self) {
        self.kernel_stack_owner = None;
    }

    /// Check if this task's kernel stack is available (not in use by any CPU).
    /// A task is selectable iff state == Ready AND kernel_stack_owner.is_none().
    #[inline]
    pub fn kernel_stack_available(&self) -> bool {
        self.kernel_stack_owner.is_none()
    }

    /// Get which CPU owns this task's kernel stack, if any.
    #[inline]
    pub fn kernel_stack_owner(&self) -> Option<u32> {
        self.kernel_stack_owner
    }

    // ========================================================================
    // Debug Invariants (Run Queue Tracking)
    // ========================================================================

    /// Mark task as being on a run queue (debug builds only).
    #[cfg(debug_assertions)]
    #[inline]
    pub(crate) fn set_on_runq(&mut self, on: bool) {
        debug_assert!(
            on != self.on_runq,
            "Task {} on_runq invariant violated: was {}, setting to {}",
            self.id, self.on_runq, on
        );
        self.on_runq = on;
    }

    /// Check if task is on a run queue (debug builds only).
    #[cfg(debug_assertions)]
    #[inline]
    pub fn is_on_runq(&self) -> bool {
        self.on_runq
    }

    /// No-op for release builds.
    #[cfg(not(debug_assertions))]
    #[inline]
    pub(crate) fn set_on_runq(&mut self, _on: bool) {}

    /// Always false for release builds.
    #[cfg(not(debug_assertions))]
    #[inline]
    pub fn is_on_runq(&self) -> bool {
        false
    }

    /// Record syscall activity for liveness tracking
    /// Called at syscall entry to prove task is responsive
    #[inline]
    pub fn record_activity(&mut self, tick: u64) {
        self.last_activity_tick = tick;
    }

    /// Reset liveness state if task is in implicit pong state
    /// A syscall itself proves the task is alive - no explicit pong needed
    #[inline]
    pub fn reset_liveness_if_implicit_pong(&mut self) {
        if let crate::kernel::liveness::LivenessState::PingSent { channel: 0, .. } = self.liveness_state {
            self.liveness_state = crate::kernel::liveness::LivenessState::Normal;
        }
    }

    /// Record a syscall for storm protection and return action to take
    #[inline]
    pub fn record_storm_syscall(&mut self, tick: u64, config: &crate::kernel::storm::StormConfig) -> crate::kernel::storm::StormAction {
        self.storm.record_syscall(tick, config)
    }

    /// Set deferred return value for syscall (written to trap_frame.x0)
    /// Used when syscall return value needs to be set after blocking
    #[inline]
    pub fn set_deferred_return(&mut self, value: i64) {
        self.trap_frame.x0 = value as u64;
    }

    /// Detach task from its parent, returning the old parent ID
    /// Used by daemonize to orphan a process
    #[inline]
    pub fn detach_from_parent(&mut self) -> TaskId {
        let old_parent = self.parent_id;
        self.parent_id = 0;
        old_parent
    }

    /// Total heap pages across all heap mappings
    pub fn total_heap_pages(&self) -> u32 {
        let mut total: u32 = 0;
        for m in &self.heap_mappings {
            if !m.is_empty() {
                total = total.saturating_add(m.num_pages as u32);
            }
        }
        total
    }

    /// Count of non-empty heap mappings
    pub fn mapping_count(&self) -> u8 {
        let mut count: u8 = 0;
        for m in &self.heap_mappings {
            if !m.is_empty() {
                count = count.saturating_add(1);
            }
        }
        count
    }

    /// Get capabilities as raw bits for syscall return
    #[inline]
    pub fn get_capabilities_bits(&self) -> u64 {
        self.capabilities.bits()
    }

    /// Get activity age in milliseconds since last syscall.
    /// `current_counter` must be a raw hardware counter value (same units as last_activity_tick).
    #[inline]
    pub fn get_activity_age_ms(&self, current_counter: u64) -> u32 {
        if self.last_activity_tick == 0 {
            0
        } else {
            let delta = current_counter.saturating_sub(self.last_activity_tick);
            let freq = crate::platform::current::timer::frequency();
            if freq == 0 { return 0; }
            // delta * 1000 / freq — safe for up to ~9 years at 62.5MHz
            (delta * 1000 / freq) as u32
        }
    }

    /// Get liveness status code for ABI
    #[inline]
    pub fn get_liveness_status_code(&self) -> u8 {
        use abi::liveness_status;
        match self.liveness_state {
            crate::kernel::liveness::LivenessState::Normal => liveness_status::NORMAL,
            crate::kernel::liveness::LivenessState::PingSent { .. } => liveness_status::PING_SENT,
            crate::kernel::liveness::LivenessState::ClosePending { .. } => liveness_status::CLOSE_PENDING,
        }
    }

    // ========================================================================
    // Unified Memory Mapping API
    // ========================================================================

    /// Map a region of memory into user address space
    pub fn map_region(
        &mut self,
        size: usize,
        source: MapSource,
        flags: MapFlags,
        kind: MappingKind,
    ) -> Option<MapResult> {
        // OVERFLOW CHECK: Calculate number of pages safely
        let num_pages = size.checked_add(4095)? / 4096;
        if num_pages == 0 {
            return None;
        }

        let slot = self.heap_mappings.iter().position(|m| m.is_empty())?;

        let virt_addr = self.heap_next;
        // OVERFLOW CHECK: Calculate mapping size safely
        let mapping_size = num_pages.checked_mul(4096)? as u64;
        // OVERFLOW CHECK: Check end address safely
        let end_addr = virt_addr.checked_add(mapping_size)?;
        if end_addr > USER_HEAP_END {
            return None;
        }

        let phys_addr = match source {
            MapSource::Allocate => pmm::alloc_pages(num_pages)? as u64,
            MapSource::Fixed(addr) => addr,
        };

        if flags.zero {
            unsafe {
                let kva = mmu::phys_to_virt(phys_addr) as *mut u8;
                for i in 0..(num_pages * 4096) {
                    core::ptr::write_volatile(kva.add(i), 0);
                }
            }
        }

        if flags.flush_cache {
            unsafe {
                for page in 0..num_pages {
                    let page_kva = mmu::phys_to_virt(phys_addr + (page * 4096) as u64);
                    for offset in (0..4096).step_by(64) {
                        let addr = page_kva + offset;
                        core::arch::asm!("dc civac, {}", in(reg) addr);
                    }
                }
                core::arch::asm!("dsb sy");
            }
        }

        let addr_space = self.address_space.as_mut()?;
        let map_result = if flags.device {
            for i in 0..num_pages {
                let page_virt = virt_addr + (i * 4096) as u64;
                let page_phys = phys_addr + (i * 4096) as u64;
                if !addr_space.map_device_page(page_virt, page_phys, flags.writable) {
                    if matches!(source, MapSource::Allocate) {
                        pmm::free_pages(phys_addr as usize, num_pages);
                    }
                    return None;
                }
            }
            true
        } else if kind == MappingKind::OwnedAnon && !flags.flush_cache {
            for i in 0..num_pages {
                let page_virt = virt_addr + (i * 4096) as u64;
                let page_phys = phys_addr + (i * 4096) as u64;
                if !addr_space.map_page(page_virt, page_phys, flags.writable, flags.executable) {
                    if matches!(source, MapSource::Allocate) {
                        pmm::free_pages(phys_addr as usize, num_pages);
                    }
                    return None;
                }
            }
            true
        } else {
            for i in 0..num_pages {
                let page_virt = virt_addr + (i * 4096) as u64;
                let page_phys = phys_addr + (i * 4096) as u64;
                if !addr_space.map_dma_page(page_virt, page_phys, flags.writable) {
                    if matches!(source, MapSource::Allocate) {
                        pmm::free_pages(phys_addr as usize, num_pages);
                    }
                    return None;
                }
            }
            true
        };

        if !map_result {
            return None;
        }

        let asid = self.address_space.as_ref().map(|a| a.get_asid()).unwrap_or(0);
        tlb::invalidate_va_range(asid, virt_addr, num_pages);

        self.heap_mappings[slot] = HeapMapping {
            virt_addr,
            phys_addr,
            num_pages,
            kind,
        };

        self.heap_next = virt_addr + mapping_size;

        Some(MapResult { virt_addr, phys_addr })
    }

    // ========================================================================
    // Legacy Mapping Functions
    // ========================================================================

    /// Reserve a virtual region without allocating physical pages.
    /// Pages are allocated on demand when accessed (page fault handler).
    pub fn mmap_lazy(&mut self, size: usize) -> Option<u64> {
        let num_pages = size.checked_add(4095)? / 4096;
        if num_pages == 0 {
            return None;
        }

        let slot = self.heap_mappings.iter().position(|m| m.is_empty())?;

        let virt_addr = self.heap_next;
        let mapping_size = num_pages.checked_mul(4096)? as u64;
        let end_addr = virt_addr.checked_add(mapping_size)?;
        if end_addr > USER_HEAP_END {
            return None;
        }

        // No physical allocation — pages filled in by fault handler
        self.heap_mappings[slot] = HeapMapping {
            virt_addr,
            phys_addr: 0,
            num_pages,
            kind: MappingKind::LazyAnon,
        };

        self.heap_next = virt_addr + mapping_size;
        Some(virt_addr)
    }

    #[inline]
    pub fn mmap(&mut self, size: usize, writable: bool, executable: bool) -> Option<u64> {
        self.map_region(
            size,
            MapSource::Allocate,
            MapFlags::anon(writable, executable),
            MappingKind::OwnedAnon,
        ).map(|r| r.virt_addr)
    }

    #[inline]
    pub fn mmap_dma(&mut self, size: usize) -> Option<(u64, u64)> {
        self.map_region(
            size,
            MapSource::Allocate,
            MapFlags::dma(),
            MappingKind::OwnedDma,
        ).map(|r| (r.virt_addr, r.phys_addr))
    }

    #[inline]
    pub fn mmap_phys(&mut self, phys_addr: u64, size: usize) -> Option<u64> {
        self.map_region(
            size,
            MapSource::Fixed(phys_addr),
            MapFlags::shared(),
            MappingKind::BorrowedShmem,
        ).map(|r| r.virt_addr)
    }

    #[inline]
    pub fn mmap_shmem_dma(&mut self, phys_addr: u64, size: usize) -> Option<u64> {
        self.map_region(
            size,
            MapSource::Fixed(phys_addr),
            MapFlags::shmem_map(),
            MappingKind::BorrowedShmem,
        ).map(|r| r.virt_addr)
    }

    #[inline]
    pub fn mmap_shmem_dma_high(&mut self, phys_addr: u64, size: usize) -> Option<u64> {
        self.map_region(
            size,
            MapSource::Fixed(phys_addr),
            MapFlags::dma_high(),
            MappingKind::BorrowedShmem,
        ).map(|r| r.virt_addr)
    }

    /// Map DMA memory with streaming (cacheable) attributes
    /// Use this for data buffers that benefit from caching.
    /// Requires explicit cache sync operations via userspace.
    #[inline]
    pub fn mmap_shmem_dma_streaming(&mut self, phys_addr: u64, size: usize) -> Option<u64> {
        self.map_region(
            size,
            MapSource::Fixed(phys_addr),
            MapFlags::streaming_dma(),
            MappingKind::BorrowedShmem,
        ).map(|r| r.virt_addr)
    }

    #[inline]
    pub fn mmap_device(&mut self, phys_addr: u64, size: usize) -> Option<u64> {
        self.map_region(
            size,
            MapSource::Fixed(phys_addr),
            MapFlags::device(),
            MappingKind::DeviceMmio,
        ).map(|r| r.virt_addr)
    }

    pub fn munmap(&mut self, addr: u64, size: usize) -> bool {
        // OVERFLOW CHECK: Calculate number of pages safely
        let num_pages = match size.checked_add(4095) {
            Some(s) => s / 4096,
            None => return false, // Size overflow
        };
        if num_pages == 0 {
            return false;
        }

        let slot = match self.heap_mappings.iter().position(|m| {
            m.virt_addr == addr && m.num_pages == num_pages
        }) {
            Some(s) => s,
            None => return false,
        };

        let mapping = self.heap_mappings[slot];

        if let Some(ref mut addr_space) = self.address_space {
            if mapping.kind == MappingKind::LazyAnon {
                // Demand-paged: free each individually-allocated page
                for i in 0..mapping.num_pages {
                    let page_virt = mapping.virt_addr + (i * 4096) as u64;
                    if let Some(phys) = addr_space.translate(page_virt) {
                        let page_phys = phys & !0xFFF;
                        pmm::free_page(page_phys as usize);
                    }
                    addr_space.unmap_page(page_virt);
                }
            } else {
                for i in 0..mapping.num_pages {
                    let page_virt = mapping.virt_addr + (i * 4096) as u64;
                    addr_space.unmap_page(page_virt);
                }
            }
        }

        let asid = self.address_space.as_ref().map(|a| a.get_asid()).unwrap_or(0);
        tlb::invalidate_va_range(asid, mapping.virt_addr, mapping.num_pages);

        if mapping.owns_pages() {
            pmm::free_pages(mapping.phys_addr as usize, mapping.num_pages);
        }

        self.heap_mappings[slot] = HeapMapping::empty();

        true
    }

}

impl Drop for Task {
    fn drop(&mut self) {
        for mapping in &self.heap_mappings {
            if mapping.is_empty() {
                continue;
            }
            if mapping.kind == MappingKind::LazyAnon {
                // Demand-paged: walk each page and free if mapped
                if let Some(ref addr_space) = self.address_space {
                    for i in 0..mapping.num_pages {
                        let vaddr = mapping.virt_addr + (i * 4096) as u64;
                        if let Some(phys) = addr_space.translate(vaddr) {
                            let page_phys = phys & !0xFFF;
                            pmm::free_page(page_phys as usize);
                        }
                    }
                }
            } else if mapping.owns_pages() {
                pmm::free_pages(mapping.phys_addr as usize, mapping.num_pages);
            }
        }

        // Don't free static idle stacks (guard_page == 0 means not from PMM)
        if self.guard_page != 0 {
            // Poison the stack before freeing so any use-after-free reads garbage
            #[cfg(debug_assertions)]
            unsafe {
                let stack_virt = mmu::phys_to_virt(self.kernel_stack) as *mut u8;
                core::ptr::write_bytes(stack_virt, 0xDF, self.kernel_stack_size);
            }

            let total_pages = (self.kernel_stack_size / 4096) + 1;
            pmm::free_pages(self.guard_page as usize, total_pages);
        }
    }
}

// ============================================================================
// Context Switch Assembly
// ============================================================================

core::arch::global_asm!(r#"
.global context_switch_asm
.type context_switch_asm, @function
context_switch_asm:
    // MEMORY BARRIER: Ensure all stores from outgoing context are visible
    // to other CPUs before we switch. Critical for SMP correctness.
    dsb     sy

    // Save callee-saved registers for current task
    stp     x19, x20, [x0, #0]
    stp     x21, x22, [x0, #16]
    stp     x23, x24, [x0, #32]
    stp     x25, x26, [x0, #48]
    stp     x27, x28, [x0, #64]
    stp     x29, x30, [x0, #80]
    mov     x9, sp
    str     x9, [x0, #96]

    // Restore callee-saved registers for next task
    ldp     x19, x20, [x1, #0]
    ldp     x21, x22, [x1, #16]
    ldp     x23, x24, [x1, #32]
    ldp     x25, x26, [x1, #48]
    ldp     x27, x28, [x1, #64]
    ldp     x29, x30, [x1, #80]
    ldr     x9, [x1, #96]
    mov     sp, x9

    // MEMORY BARRIER: Ensure all loads complete before returning
    // to the new context. Prevents stale reads in SMP.
    dsb     sy
    isb

    ret

.global enter_usermode_asm
.type enter_usermode_asm, @function
enter_usermode_asm:
    // x0 = trap_frame, x1 = ttbr0, x2 = kernel_stack_top
    mov     x9, x1
    mov     x19, x2         // save kernel_stack_top (x19 is callee-saved, safe to use here since we never return)

    ldr     x2, [x0, #248]
    ldr     x3, [x0, #256]
    ldr     x4, [x0, #264]

    msr     sp_el0, x2
    msr     elr_el1, x3
    msr     spsr_el1, x4

    adr     x10, 1f
    movz    x11, #0xFFFF, lsl #48
    orr     x10, x10, x11
    br      x10

1:
    msr     ttbr0_el1, x9
    isb
    tlbi    vmalle1is
    dsb     ish
    isb
    // CRITICAL: Invalidate I-cache - different processes use same VA with different PA
    ic      ialluis         // Invalidate all I-caches in inner shareable domain
    dsb     ish
    isb

    // Set SP to task's kernel stack top BEFORE eret.
    // This becomes SP_EL1 for subsequent exceptions from EL0.
    // Without this, the boot stack is used for all syscalls,
    // which overflows BSS and corrupts kernel globals.
    mov     sp, x19

    mov     x0, #0
    mov     x1, #0
    mov     x2, #0
    mov     x3, #0
    mov     x4, #0
    mov     x5, #0
    mov     x6, #0
    mov     x7, #0
    mov     x8, #0
    mov     x9, #0
    mov     x10, #0
    mov     x11, #0
    mov     x12, #0
    mov     x13, #0
    mov     x14, #0
    mov     x15, #0
    mov     x16, #0
    mov     x17, #0
    mov     x18, #0
    mov     x19, #0
    mov     x20, #0
    mov     x21, #0
    mov     x22, #0
    mov     x23, #0
    mov     x24, #0
    mov     x25, #0
    mov     x26, #0
    mov     x27, #0
    mov     x28, #0
    mov     x29, #0
    mov     x30, #0

    eret
"#);

extern "C" {
    fn context_switch_asm(current: *mut CpuContext, next: *const CpuContext);
    fn enter_usermode_asm(trap_frame: *const TrapFrame, ttbr0: u64, kernel_stack_top: u64) -> !;
}

/// Perform a raw context switch between two CPU contexts
/// # Safety
/// Both pointers must point to valid CpuContext structures
pub unsafe fn context_switch(current: *mut CpuContext, next: *const CpuContext) {
    context_switch_asm(current, next);
}

/// Perform a context switch between two tasks
/// # Safety
/// Must be called with interrupts disabled
pub unsafe fn switch_context(current: &mut Task, next: &Task) {
    if let Some(ref addr_space) = next.address_space {
        addr_space.activate();
    }

    context_switch_asm(&mut current.context, &next.context);
}

/// Enter user mode with given trap frame and address space
/// # Safety
/// trap_frame must point to valid TrapFrame, ttbr0 must be valid page table,
/// kernel_stack_top must be a valid kernel virtual address for SP_EL1.
pub unsafe fn enter_usermode(trap_frame: *const TrapFrame, ttbr0: u64, kernel_stack_top: u64) -> ! {
    enter_usermode_asm(trap_frame, ttbr0, kernel_stack_top)
}
