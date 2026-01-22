//! Task Control Block
//!
//! Contains the Task struct, CPU contexts, and related types.
//! This is the per-task state, separate from the scheduler.

use crate::kernel::addrspace::AddressSpace;
use crate::kernel::event::EventQueue;
use crate::kernel::pmm;
use crate::arch::aarch64::{mmu, tlb};

use super::state::TaskState;
use crate::kernel::memory::{
    MAX_HEAP_MAPPINGS, USER_HEAP_START, USER_HEAP_END,
    MappingKind, HeapMapping, MapFlags, MapSource, MapResult,
};

/// Task priority levels
/// Lower number = higher priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Priority {
    /// High priority - drivers and critical services
    High = 0,
    /// Normal priority - regular applications
    Normal = 1,
    /// Low priority - background tasks
    Low = 2,
}

impl Default for Priority {
    fn default() -> Self {
        Priority::Normal
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

/// Stack size for kernel tasks (16KB)
pub const KERNEL_STACK_SIZE: usize = 16 * 1024;

/// Guard page size (4KB) - placed at bottom of kernel stack
pub const GUARD_PAGE_SIZE: usize = 4096;

/// Maximum number of children a process can have
pub const MAX_CHILDREN: usize = 16;

/// Maximum number of PIDs allowed to send signals to this process
pub const MAX_SIGNAL_SENDERS: usize = 8;

/// Maximum number of timers per task (BSD kqueue style)
pub const MAX_TIMERS_PER_TASK: usize = 8;

/// Per-process resource limits (prevent exhaustion attacks)
pub const MAX_CHANNELS_PER_TASK: u16 = 32;
pub const MAX_PORTS_PER_TASK: u16 = 4;
pub const MAX_SHMEM_PER_TASK: u16 = 16;

/// Number of timer ticks to wait between Phase 1 (notify) and Phase 2 (force cleanup)
/// At 100 ticks/sec, 10 ticks = 100ms grace period for servers to release resources
pub const CLEANUP_GRACE_TICKS: u8 = 10;

/// Timer descriptor for unified event system
/// Supports multiple independent timers per task, including recurring.
#[derive(Clone, Copy)]
pub struct TimerDesc {
    /// Timer ID (1-8 for user timers, 0 reserved for legacy timer_set)
    pub id: u32,
    /// Interval in ticks (0 = one-shot, >0 = recurring)
    pub interval: u64,
    /// Next deadline tick (0 = inactive)
    pub deadline: u64,
}

impl TimerDesc {
    pub const fn empty() -> Self {
        Self {
            id: 0,
            interval: 0,
            deadline: 0,
        }
    }

    pub fn is_active(&self) -> bool {
        self.deadline != 0
    }
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
    /// Scheduling priority - use priority() and set_priority()
    pub(crate) priority: Priority,
    /// Saved CPU context (for kernel threads) - internal use only
    pub(crate) context: CpuContext,
    /// Saved trap frame (for user processes) - use trap_frame() / trap_frame_mut()
    pub(crate) trap_frame: TrapFrame,
    /// User address space (None for kernel tasks)
    pub(crate) address_space: Option<AddressSpace>,
    /// Is this the init process (devd)?
    pub(crate) is_init: bool,
    /// Task name for debugging
    pub(crate) name: [u8; 16],
    /// Heap mappings for mmap/munmap tracking
    pub(crate) heap_mappings: [HeapMapping; MAX_HEAP_MAPPINGS],
    /// Next heap address for bump allocation
    pub(crate) heap_next: u64,
    /// Object table (unified 5-syscall system)
    pub(crate) object_table: crate::kernel::object::HandleTable,
    /// Per-process resource counters
    pub(crate) channel_count: u16,
    pub(crate) port_count: u16,
    pub(crate) shmem_count: u16,
    /// Event queue for async notifications
    pub(crate) event_queue: EventQueue,
    /// Child task IDs
    pub(crate) children: [TaskId; MAX_CHILDREN],
    /// Number of children
    pub(crate) num_children: usize,
    /// Multiple timers per task (BSD kqueue style)
    pub(crate) timers: [TimerDesc; MAX_TIMERS_PER_TASK],
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
            priority: Priority::Normal,
            context,
            trap_frame: TrapFrame::new(),
            kernel_stack: stack_base as u64,
            kernel_stack_size: KERNEL_STACK_SIZE,
            guard_page: guard_page as u64,
            address_space: None,
            is_user: false,
            is_init: false,
            name: task_name,
            heap_mappings: [HeapMapping::empty(); MAX_HEAP_MAPPINGS],
            heap_next: USER_HEAP_START,
            object_table: crate::kernel::object::HandleTable::new(),
            channel_count: 0,
            port_count: 0,
            shmem_count: 0,
            event_queue: EventQueue::new(),
            parent_id: 0,
            children: [0; MAX_CHILDREN],
            num_children: 0,
            timers: [TimerDesc::empty(); MAX_TIMERS_PER_TASK],
            capabilities: crate::kernel::caps::Capabilities::ALL,
            signal_allowlist: [0; MAX_SIGNAL_SENDERS],
            signal_allowlist_count: 0,
            liveness_state: crate::kernel::liveness::LivenessState::Normal,
            last_activity_tick: 0,
        })
    }

    /// Create a new user task with its own address space
    pub fn new_user(id: TaskId, name: &str) -> Option<Self> {
        let total_pages = (KERNEL_STACK_SIZE / 4096) + 1;
        let alloc_base = pmm::alloc_pages(total_pages)?;

        let guard_page = alloc_base;
        let stack_base = alloc_base + GUARD_PAGE_SIZE;

        let address_space = match AddressSpace::new() {
            Some(addr_space) => addr_space,
            None => {
                pmm::free_pages(alloc_base, total_pages);
                return None;
            }
        };

        let context = CpuContext::new();
        let trap_frame = TrapFrame::new();

        let mut task_name = [0u8; 16];
        let name_bytes = name.as_bytes();
        let copy_len = core::cmp::min(name_bytes.len(), 15);
        task_name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        Some(Self {
            id,
            state: TaskState::Ready,
            priority: Priority::Normal,
            context,
            trap_frame,
            kernel_stack: stack_base as u64,
            kernel_stack_size: KERNEL_STACK_SIZE,
            guard_page: guard_page as u64,
            address_space: Some(address_space),
            is_user: true,
            is_init: false,
            name: task_name,
            heap_mappings: [HeapMapping::empty(); MAX_HEAP_MAPPINGS],
            heap_next: USER_HEAP_START,
            object_table: crate::kernel::object::HandleTable::new_with_stdio(),
            channel_count: 0,
            port_count: 0,
            shmem_count: 0,
            event_queue: EventQueue::new(),
            parent_id: 0,
            children: [0; MAX_CHILDREN],
            num_children: 0,
            timers: [TimerDesc::empty(); MAX_TIMERS_PER_TASK],
            capabilities: crate::kernel::caps::Capabilities::DRIVER_DEFAULT,
            signal_allowlist: [0; MAX_SIGNAL_SENDERS],
            signal_allowlist_count: 0,
            liveness_state: crate::kernel::liveness::LivenessState::Normal,
            last_activity_tick: 0,
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

    pub fn set_priority(&mut self, priority: Priority) {
        self.priority = priority;
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

    /// Transition: Ready → Running (scheduled to run)
    #[inline]
    pub fn set_running(&mut self) -> Result<(), super::state::InvalidTransition> {
        self.state.schedule()
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

    /// Force state to Ready (for task slot reuse after Dead)
    /// This bypasses the state machine - only use for slot reinitialization
    #[inline]
    pub(crate) fn reset_state_for_reuse(&mut self) {
        self.state = TaskState::Ready;
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
        let num_pages = (size + 4095) / 4096;
        if num_pages == 0 {
            return None;
        }

        let slot = self.heap_mappings.iter().position(|m| m.is_empty())?;

        let virt_addr = self.heap_next;
        let mapping_size = (num_pages * 4096) as u64;
        if virt_addr + mapping_size > USER_HEAP_END {
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
        let num_pages = (size + 4095) / 4096;
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
            for i in 0..mapping.num_pages {
                let page_virt = mapping.virt_addr + (i * 4096) as u64;
                addr_space.unmap_page(page_virt);
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
            if !mapping.is_empty() && mapping.owns_pages() {
                pmm::free_pages(mapping.phys_addr as usize, mapping.num_pages);
            }
        }

        let total_pages = (self.kernel_stack_size / 4096) + 1;
        pmm::free_pages(self.guard_page as usize, total_pages);
    }
}

// ============================================================================
// Context Switch Assembly
// ============================================================================

core::arch::global_asm!(r#"
.global context_switch_asm
.type context_switch_asm, @function
context_switch_asm:
    stp     x19, x20, [x0, #0]
    stp     x21, x22, [x0, #16]
    stp     x23, x24, [x0, #32]
    stp     x25, x26, [x0, #48]
    stp     x27, x28, [x0, #64]
    stp     x29, x30, [x0, #80]
    mov     x9, sp
    str     x9, [x0, #96]

    ldp     x19, x20, [x1, #0]
    ldp     x21, x22, [x1, #16]
    ldp     x23, x24, [x1, #32]
    ldp     x25, x26, [x1, #48]
    ldp     x27, x28, [x1, #64]
    ldp     x29, x30, [x1, #80]
    ldr     x9, [x1, #96]
    mov     sp, x9

    ret

.global enter_usermode_asm
.type enter_usermode_asm, @function
enter_usermode_asm:
    mov     x9, x1

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
    tlbi    vmalle1
    dsb     sy
    isb

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
    fn enter_usermode_asm(trap_frame: *const TrapFrame, ttbr0: u64) -> !;
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
/// trap_frame must point to valid TrapFrame, ttbr0 must be valid page table
pub unsafe fn enter_usermode(trap_frame: *const TrapFrame, ttbr0: u64) -> ! {
    enter_usermode_asm(trap_frame, ttbr0)
}
