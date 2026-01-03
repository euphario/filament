//! Task Management and Context Switching
//!
//! Provides task structures and context switching for the microkernel.
//! Each task has its own saved CPU state and address space.

#![allow(dead_code)]  // Some wait reasons and methods are for future use

use super::addrspace::AddressSpace;
use super::event::EventQueue;
use super::fd::FdTable;
use super::pmm;
use crate::arch::aarch64::{mmu, tlb};
use crate::logln;

/// Task states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TaskState {
    /// Task is ready to run
    Ready = 0,
    /// Task is currently running
    Running = 1,
    /// Task is blocked (waiting for IPC, etc.)
    Blocked = 2,
    /// Task has exited
    Terminated = 3,
}

/// Reason for blocking (helps notify know what to check)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitReason {
    /// Waiting for IPC message
    Ipc,
    /// Waiting for child process
    Child,
    /// Waiting for event
    Event,
    /// Waiting for shared memory notification
    ShmemNotify(u32),
}

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
/// If stack overflows, it will write to this page which can be detected
pub const GUARD_PAGE_SIZE: usize = 4096;

/// Maximum number of tasks
pub const MAX_TASKS: usize = 16;

/// Maximum number of heap mappings per task
/// WiFi driver needs ~20 rings × 2 mappings each = 40+
pub const MAX_HEAP_MAPPINGS: usize = 64;

/// User heap region start (after code/stack regions)
pub const USER_HEAP_START: u64 = 0x5000_0000;

/// User heap region end
pub const USER_HEAP_END: u64 = 0x7000_0000;

/// Mapping ownership kind - determines cleanup behavior
///
/// CRITICAL: This enum prevents memory corruption bugs:
/// - OwnedAnon/OwnedDma: Kernel allocated pages, free on unmap
/// - BorrowedShmem: Pages owned by shmem region, just unmap PTEs
/// - DeviceMmio: Physical MMIO addresses, NEVER free (not RAM!)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MappingKind {
    /// Anonymous memory allocated by mmap() - kernel owns pages
    OwnedAnon,
    /// DMA-capable memory allocated by mmap_dma() - kernel owns pages
    OwnedDma,
    /// Shared memory mapped via mmap_phys() - pages owned elsewhere
    BorrowedShmem,
    /// Device MMIO mapped via mmap_device() - not RAM, never free
    DeviceMmio,
}

/// A single heap mapping entry
#[derive(Clone, Copy)]
pub struct HeapMapping {
    /// Virtual address of mapping
    pub virt_addr: u64,
    /// Physical address of mapping
    pub phys_addr: u64,
    /// Number of pages
    pub num_pages: usize,
    /// Ownership kind - determines if pages should be freed on unmap
    pub kind: MappingKind,
}

impl HeapMapping {
    pub const fn empty() -> Self {
        Self {
            virt_addr: 0,
            phys_addr: 0,
            num_pages: 0,
            kind: MappingKind::OwnedAnon, // Default doesn't matter for empty
        }
    }

    pub fn is_empty(&self) -> bool {
        self.num_pages == 0
    }

    /// Returns true if this mapping owns its physical pages (should free on unmap)
    pub fn owns_pages(&self) -> bool {
        matches!(self.kind, MappingKind::OwnedAnon | MappingKind::OwnedDma)
    }
}

// ============================================================================
// Unified Mapping API
// ============================================================================

/// Flags for map_region()
#[derive(Clone, Copy, Debug)]
pub struct MapFlags {
    /// Page is writable
    pub writable: bool,
    /// Page is executable
    pub executable: bool,
    /// Zero pages after allocation
    pub zero: bool,
    /// Flush cache after zeroing (for DMA)
    pub flush_cache: bool,
    /// Use device memory attributes (nGnRnE) instead of normal cacheable
    pub device: bool,
}

impl MapFlags {
    /// Anonymous memory mapping (normal cacheable, zeroed)
    pub const fn anon(writable: bool, executable: bool) -> Self {
        Self {
            writable,
            executable,
            zero: true,
            flush_cache: false,
            device: false,
        }
    }

    /// DMA memory mapping (cacheable, zeroed, cache flushed)
    pub const fn dma() -> Self {
        Self {
            writable: true,
            executable: false,
            zero: true,
            flush_cache: true,
            device: false,
        }
    }

    /// Shared memory mapping (no allocation, no zeroing)
    pub const fn shared() -> Self {
        Self {
            writable: true,
            executable: false,
            zero: false,
            flush_cache: false,
            device: false,
        }
    }

    /// Device MMIO mapping (non-cacheable, no zeroing)
    pub const fn device() -> Self {
        Self {
            writable: true,
            executable: false,
            zero: false,
            flush_cache: false,
            device: true,
        }
    }
}

/// Source of physical memory for mapping
#[derive(Clone, Copy, Debug)]
pub enum MapSource {
    /// Allocate new physical pages
    Allocate,
    /// Use specified physical address (don't allocate)
    Fixed(u64),
}

/// Result of map_region
#[derive(Clone, Copy, Debug)]
pub struct MapResult {
    /// Virtual address of the mapping
    pub virt_addr: u64,
    /// Physical address of the mapping
    pub phys_addr: u64,
}

/// Maximum number of children a process can have
pub const MAX_CHILDREN: usize = 16;

/// Maximum number of PIDs allowed to send signals to this process
/// Used for receiver opt-in rate limiting
pub const MAX_SIGNAL_SENDERS: usize = 8;

/// Task Control Block
pub struct Task {
    /// Unique task ID
    pub id: TaskId,
    /// Current state
    pub state: TaskState,
    /// Scheduling priority
    pub priority: Priority,
    /// Saved CPU context (for kernel threads)
    pub context: CpuContext,
    /// Saved trap frame (for user processes) - at top of kernel stack
    pub trap_frame: TrapFrame,
    /// Kernel stack base (physical address, after guard page)
    pub kernel_stack: u64,
    /// Kernel stack size (not including guard)
    pub kernel_stack_size: usize,
    /// Guard page physical address (for debugging overflows)
    pub guard_page: u64,
    /// User address space (None for kernel tasks)
    pub address_space: Option<AddressSpace>,
    /// Is this a user-mode task?
    pub is_user: bool,
    /// Task name for debugging
    pub name: [u8; 16],
    /// Heap mappings for mmap/munmap tracking
    pub heap_mappings: [HeapMapping; MAX_HEAP_MAPPINGS],
    /// Next heap address for bump allocation
    pub heap_next: u64,
    /// File descriptor table
    pub fd_table: FdTable,
    /// Event queue for async notifications
    pub event_queue: EventQueue,
    /// Parent task ID (0 for orphan/init)
    pub parent_id: TaskId,
    /// Child task IDs
    pub children: [TaskId; MAX_CHILDREN],
    /// Number of children
    pub num_children: usize,
    /// Exit code (valid when state is Terminated)
    pub exit_code: i32,
    /// Reason for blocking (when state is Blocked)
    pub wait_reason: Option<WaitReason>,
    /// Timer tick count at which to wake this task (0 = no timeout)
    pub wake_at: u64,
    /// Timer tick count at which to deliver Timer event (0 = no timer)
    /// Set via timer_set syscall, delivers EventType::Timer when reached
    pub timer_deadline: u64,
    /// Last heartbeat tick count (0 = never sent heartbeat)
    /// Privileged processes should send heartbeats periodically
    pub last_heartbeat: u64,
    /// Capability set for this task
    pub capabilities: super::caps::Capabilities,
    /// Signal allowlist - PIDs allowed to send signals to this process
    /// Empty allowlist (count=0) means allow from all (backwards compat)
    pub signal_allowlist: [u32; MAX_SIGNAL_SENDERS],
    /// Number of entries in signal allowlist
    pub signal_allowlist_count: usize,
}

impl Task {
    /// Create a new kernel task
    pub fn new_kernel(id: TaskId, entry: fn() -> !, name: &str) -> Option<Self> {
        // Allocate kernel stack + guard page
        // Layout: [guard page][stack pages...]
        let total_pages = (KERNEL_STACK_SIZE / 4096) + 1; // +1 for guard
        let alloc_base = pmm::alloc_pages(total_pages)?;

        // Guard page is at the base
        let guard_page = alloc_base;
        // Usable stack starts after guard page
        let stack_base = alloc_base + GUARD_PAGE_SIZE;
        // Stack grows down, so SP starts at top
        let stack_top = stack_base + KERNEL_STACK_SIZE;

        // Initialize context
        let mut context = CpuContext::new();
        context.sp = stack_top as u64;
        context.pc = entry as u64;
        context.x30 = entry as u64;  // Return address for initial ret

        // Copy name
        let mut task_name = [0u8; 16];
        let name_bytes = name.as_bytes();
        let copy_len = core::cmp::min(name_bytes.len(), 15);
        task_name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        Some(Self {
            id,
            state: TaskState::Ready,
            priority: Priority::Normal,  // Kernel tasks default to Normal
            context,
            trap_frame: TrapFrame::new(),
            kernel_stack: stack_base as u64,
            kernel_stack_size: KERNEL_STACK_SIZE,
            guard_page: guard_page as u64,
            address_space: None,
            is_user: false,
            name: task_name,
            heap_mappings: [HeapMapping::empty(); MAX_HEAP_MAPPINGS],
            heap_next: USER_HEAP_START,
            fd_table: FdTable::new(),
            event_queue: EventQueue::new(),
            parent_id: 0,
            children: [0; MAX_CHILDREN],
            num_children: 0,
            exit_code: 0,
            wait_reason: None,
            wake_at: 0,
            timer_deadline: 0,
            last_heartbeat: 0,
            capabilities: super::caps::Capabilities::ALL,  // Kernel tasks get all capabilities
            signal_allowlist: [0; MAX_SIGNAL_SENDERS],
            signal_allowlist_count: 0,  // Allow from all (kernel tasks)
        })
    }

    /// Create a new user task with its own address space
    /// Entry point and user stack must be set separately via set_user_entry()
    pub fn new_user(id: TaskId, name: &str) -> Option<Self> {
        // Allocate kernel stack for syscall handling + guard page
        let total_pages = (KERNEL_STACK_SIZE / 4096) + 1; // +1 for guard
        let alloc_base = pmm::alloc_pages(total_pages)?;

        // Guard page at base, usable stack after it
        let guard_page = alloc_base;
        let stack_base = alloc_base + GUARD_PAGE_SIZE;

        // Create address space (clean up stack on failure)
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
            priority: Priority::Normal,  // User tasks default to Normal
            context,
            trap_frame,
            kernel_stack: stack_base as u64,
            kernel_stack_size: KERNEL_STACK_SIZE,
            guard_page: guard_page as u64,
            address_space: Some(address_space),
            is_user: true,
            name: task_name,
            heap_mappings: [HeapMapping::empty(); MAX_HEAP_MAPPINGS],
            heap_next: USER_HEAP_START,
            fd_table: FdTable::new(),
            event_queue: EventQueue::new(),
            parent_id: 0,
            children: [0; MAX_CHILDREN],
            num_children: 0,
            exit_code: 0,
            wait_reason: None,
            wake_at: 0,
            timer_deadline: 0,
            last_heartbeat: 0,
            capabilities: super::caps::Capabilities::DRIVER_DEFAULT,  // User tasks default to driver caps
            signal_allowlist: [0; MAX_SIGNAL_SENDERS],
            signal_allowlist_count: 0,  // Allow from all (backwards compat)
        })
    }

    /// Set task capabilities
    pub fn set_capabilities(&mut self, caps: super::caps::Capabilities) {
        self.capabilities = caps;
    }

    /// Check if task has a capability
    pub fn has_capability(&self, cap: super::caps::Capabilities) -> bool {
        self.capabilities.has(cap)
    }

    /// Add a PID to the signal allowlist
    /// Returns true if added, false if list is full or already present
    pub fn allow_signals_from(&mut self, sender_pid: u32) -> bool {
        // Check if already in allowlist
        for i in 0..self.signal_allowlist_count {
            if self.signal_allowlist[i] == sender_pid {
                return true; // Already allowed
            }
        }

        // Add if space available
        if self.signal_allowlist_count < MAX_SIGNAL_SENDERS {
            self.signal_allowlist[self.signal_allowlist_count] = sender_pid;
            self.signal_allowlist_count += 1;
            true
        } else {
            false // List full
        }
    }

    /// Check if a sender PID is allowed to send signals to this task
    /// Returns true if:
    /// - Allowlist is empty (count=0) - allows all (backwards compat)
    /// - Sender is in allowlist
    /// - Sender is parent (always allowed)
    pub fn can_receive_signal_from(&self, sender_pid: u32) -> bool {
        // Empty allowlist means accept from anyone (backwards compatibility)
        if self.signal_allowlist_count == 0 {
            return true;
        }

        // Parent can always send signals
        if sender_pid == self.parent_id && self.parent_id != 0 {
            return true;
        }

        // Check allowlist
        for i in 0..self.signal_allowlist_count {
            if self.signal_allowlist[i] == sender_pid {
                return true;
            }
        }

        false
    }

    /// Set task priority
    pub fn set_priority(&mut self, priority: Priority) {
        self.priority = priority;
    }

    /// Set parent ID
    pub fn set_parent(&mut self, parent_id: TaskId) {
        self.parent_id = parent_id;
    }

    /// Add a child task ID
    pub fn add_child(&mut self, child_id: TaskId) -> bool {
        if self.num_children >= MAX_CHILDREN {
            return false;
        }
        self.children[self.num_children] = child_id;
        self.num_children += 1;
        true
    }

    /// Remove a child task ID (when child is reaped)
    pub fn remove_child(&mut self, child_id: TaskId) -> bool {
        for i in 0..self.num_children {
            if self.children[i] == child_id {
                // Shift remaining children down
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

    /// Check if this task has any children
    pub fn has_children(&self) -> bool {
        self.num_children > 0
    }

    /// Set user-mode entry point and stack
    pub fn set_user_entry(&mut self, entry: u64, user_stack: u64) {
        self.trap_frame.init_user(entry, user_stack);
    }

    /// Get task name as string
    pub fn name_str(&self) -> &str {
        let len = self.name.iter().position(|&c| c == 0).unwrap_or(16);
        core::str::from_utf8(&self.name[..len]).unwrap_or("???")
    }

    /// Get mutable reference to address space
    pub fn address_space_mut(&mut self) -> Option<&mut AddressSpace> {
        self.address_space.as_mut()
    }

    // ========================================================================
    // Unified Memory Mapping API
    // ========================================================================

    /// Map a region of memory into user address space
    ///
    /// This is the unified mapping function that handles all cases:
    /// - Anonymous memory (allocate + zero)
    /// - DMA memory (allocate + zero + cache flush)
    /// - Shared memory (fixed address, no allocation)
    /// - Device MMIO (fixed address, non-cacheable)
    ///
    /// # Arguments
    /// * `size` - Size of the region in bytes (will be rounded up to page size)
    /// * `source` - Where to get physical memory from
    /// * `flags` - Mapping attributes
    /// * `kind` - Ownership kind (determines cleanup behavior)
    ///
    /// # Returns
    /// MapResult with virtual and physical addresses on success
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

        // Find free mapping slot
        let slot = self.heap_mappings.iter().position(|m| m.is_empty())?;

        // Check heap space available
        let virt_addr = self.heap_next;
        let mapping_size = (num_pages * 4096) as u64;
        if virt_addr + mapping_size > USER_HEAP_END {
            return None;
        }

        // Get or allocate physical memory
        let phys_addr = match source {
            MapSource::Allocate => pmm::alloc_pages(num_pages)? as u64,
            MapSource::Fixed(addr) => addr,
        };

        // Zero pages if requested (using kernel virtual address via TTBR1)
        if flags.zero {
            unsafe {
                let kva = mmu::phys_to_virt(phys_addr) as *mut u8;
                for i in 0..(num_pages * 4096) {
                    core::ptr::write_volatile(kva.add(i), 0);
                }
            }
        }

        // Flush cache if requested (for DMA)
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

        // Map pages into address space with appropriate attributes
        let addr_space = self.address_space.as_mut()?;
        let map_result = if flags.device {
            // Device memory (nGnRnE)
            for i in 0..num_pages {
                let page_virt = virt_addr + (i * 4096) as u64;
                let page_phys = phys_addr + (i * 4096) as u64;
                if !addr_space.map_device_page(page_virt, page_phys, flags.writable) {
                    // Cleanup on failure if we allocated
                    if matches!(source, MapSource::Allocate) {
                        pmm::free_pages(phys_addr as usize, num_pages);
                    }
                    return None;
                }
            }
            true
        } else if kind == MappingKind::OwnedAnon && !flags.flush_cache {
            // Normal cacheable memory
            for i in 0..num_pages {
                let page_virt = virt_addr + (i * 4096) as u64;
                let page_phys = phys_addr + (i * 4096) as u64;
                if !addr_space.map_page(page_virt, page_phys, flags.writable, flags.executable) {
                    // Cleanup on failure if we allocated
                    if matches!(source, MapSource::Allocate) {
                        pmm::free_pages(phys_addr as usize, num_pages);
                    }
                    return None;
                }
            }
            true
        } else {
            // DMA or shared memory (cacheable but with DMA attributes)
            for i in 0..num_pages {
                let page_virt = virt_addr + (i * 4096) as u64;
                let page_phys = phys_addr + (i * 4096) as u64;
                if !addr_space.map_dma_page(page_virt, page_phys, flags.writable) {
                    // Cleanup on failure if we allocated
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

        // Invalidate TLB entries for newly mapped pages
        let asid = self.address_space.as_ref().map(|a| a.get_asid()).unwrap_or(0);
        tlb::invalidate_va_range(asid, virt_addr, num_pages);

        // Record mapping
        self.heap_mappings[slot] = HeapMapping {
            virt_addr,
            phys_addr,
            num_pages,
            kind,
        };

        // Bump heap pointer
        self.heap_next = virt_addr + mapping_size;

        Some(MapResult { virt_addr, phys_addr })
    }

    // ========================================================================
    // Legacy Mapping Functions (wrappers around map_region)
    // ========================================================================

    /// Allocate and map memory pages into user heap
    /// Returns virtual address on success, None on failure
    #[inline]
    pub fn mmap(&mut self, size: usize, writable: bool, executable: bool) -> Option<u64> {
        self.map_region(
            size,
            MapSource::Allocate,
            MapFlags::anon(writable, executable),
            MappingKind::OwnedAnon,
        ).map(|r| r.virt_addr)
    }

    /// Allocate DMA-capable memory, returning both virtual and physical addresses
    #[inline]
    pub fn mmap_dma(&mut self, size: usize) -> Option<(u64, u64)> {
        self.map_region(
            size,
            MapSource::Allocate,
            MapFlags::dma(),
            MappingKind::OwnedDma,
        ).map(|r| (r.virt_addr, r.phys_addr))
    }

    /// Map existing physical memory into user address space (for shared memory)
    /// Does NOT allocate physical memory - just creates the mapping
    /// Returns virtual address on success
    #[inline]
    pub fn mmap_phys(&mut self, phys_addr: u64, size: usize) -> Option<u64> {
        self.map_region(
            size,
            MapSource::Fixed(phys_addr),
            MapFlags::shared(),
            MappingKind::BorrowedShmem,
        ).map(|r| r.virt_addr)
    }

    /// Map device memory (MMIO/PCI BARs) into user address space
    /// Uses non-cacheable device memory attributes (nGnRnE)
    /// Returns virtual address on success
    #[inline]
    pub fn mmap_device(&mut self, phys_addr: u64, size: usize) -> Option<u64> {
        self.map_region(
            size,
            MapSource::Fixed(phys_addr),
            MapFlags::device(),
            MappingKind::DeviceMmio,
        ).map(|r| r.virt_addr)
    }

    /// Unmap and free memory pages from user heap
    /// Returns true on success
    pub fn munmap(&mut self, addr: u64, size: usize) -> bool {
        let num_pages = (size + 4095) / 4096;
        if num_pages == 0 {
            return false;
        }

        // Find the mapping
        let slot = match self.heap_mappings.iter().position(|m| {
            m.virt_addr == addr && m.num_pages == num_pages
        }) {
            Some(s) => s,
            None => return false,
        };

        let mapping = self.heap_mappings[slot];

        // Unmap pages from page tables
        if let Some(ref mut addr_space) = self.address_space {
            for i in 0..mapping.num_pages {
                let page_virt = mapping.virt_addr + (i * 4096) as u64;
                addr_space.unmap_page(page_virt);
            }
        }

        // Invalidate TLB entries for the unmapped pages
        // Use centralized TLB API with proper ASID tagging
        let asid = self.address_space.as_ref().map(|a| a.get_asid()).unwrap_or(0);
        tlb::invalidate_va_range(asid, mapping.virt_addr, mapping.num_pages);

        // Only free physical pages if we own them
        // CRITICAL: BorrowedShmem and DeviceMmio must NOT free pages!
        // - BorrowedShmem: pages belong to the shmem region owner
        // - DeviceMmio: physical addresses are hardware registers, not RAM
        if mapping.owns_pages() {
            pmm::free_pages(mapping.phys_addr as usize, mapping.num_pages);
        }

        // Clear mapping slot
        self.heap_mappings[slot] = HeapMapping::empty();

        true
    }
}

impl Drop for Task {
    fn drop(&mut self) {
        // Free heap mappings - but only if we own the pages
        for mapping in &self.heap_mappings {
            if !mapping.is_empty() && mapping.owns_pages() {
                pmm::free_pages(mapping.phys_addr as usize, mapping.num_pages);
            }
            // Note: BorrowedShmem and DeviceMmio mappings are NOT freed here
            // - BorrowedShmem: pages owned by shmem region, will be freed when region destroyed
            // - DeviceMmio: not RAM, never free
        }

        // Free kernel stack + guard page (allocated together)
        // The guard page is at guard_page, stack follows it
        let total_pages = (self.kernel_stack_size / 4096) + 1; // +1 for guard
        pmm::free_pages(self.guard_page as usize, total_pages);
        // address_space is automatically dropped
    }
}

// Context switch assembly for kernel threads
// Switches from current task to next task
// Arguments:
//   x0 = pointer to current task's CpuContext
//   x1 = pointer to next task's CpuContext
core::arch::global_asm!(r#"
.global context_switch
.type context_switch, @function
context_switch:
    // Save current task's callee-saved registers
    stp     x19, x20, [x0, #0]
    stp     x21, x22, [x0, #16]
    stp     x23, x24, [x0, #32]
    stp     x25, x26, [x0, #48]
    stp     x27, x28, [x0, #64]
    stp     x29, x30, [x0, #80]
    mov     x9, sp
    str     x9, [x0, #96]        // Save SP

    // Load next task's callee-saved registers
    ldp     x19, x20, [x1, #0]
    ldp     x21, x22, [x1, #16]
    ldp     x23, x24, [x1, #32]
    ldp     x25, x26, [x1, #48]
    ldp     x27, x28, [x1, #64]
    ldp     x29, x30, [x1, #80]
    ldr     x9, [x1, #96]        // Load SP
    mov     sp, x9

    // Return to next task (x30 has return address)
    ret

// Enter user mode from kernel
// Arguments:
//   x0 = pointer to TrapFrame
//   x1 = TTBR0 value for user address space
// This function never returns - it erets to user mode
.global enter_usermode
.type enter_usermode, @function
enter_usermode:
    // Save TTBR0 value (x1 arg) before we use other registers
    mov     x9, x1

    // Load special registers from trap frame
    // TrapFrame layout: x0-x30 (31*8=248), sp_el0, elr_el1, spsr_el1
    ldr     x2, [x0, #248]      // sp_el0
    ldr     x3, [x0, #256]      // elr_el1
    ldr     x4, [x0, #264]      // spsr_el1

    msr     sp_el0, x2
    msr     elr_el1, x3
    msr     spsr_el1, x4

    // Jump to TTBR1 address space before switching TTBR0
    // This allows us to safely flush TLB without losing access to kernel code
    adr     x10, 1f                 // Get PC-relative address of trampoline
    movz    x11, #0xFFFF, lsl #48   // KERNEL_VIRT_BASE = 0xFFFF000000000000
    orr     x10, x10, x11           // Ensure TTBR1 virtual address
    br      x10                     // Jump to TTBR1 space

1:
    // Now executing from TTBR1 space - safe to modify TTBR0
    msr     ttbr0_el1, x9
    isb
    tlbi    vmalle1                 // Flush all TLB entries
    dsb     sy
    isb

    // UART is accessible via TTBR1: 0xFFFF_0000_1100_0000

    // Zero all registers for clean user entry
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

    // Return to user mode
    eret
"#);

extern "C" {
    /// Switch CPU context from current to next task
    /// # Safety
    /// Both pointers must point to valid CpuContext structures
    fn context_switch(current: *mut CpuContext, next: *const CpuContext);

    /// Enter user mode with given trap frame and address space
    /// # Safety
    /// trap_frame must point to valid TrapFrame, ttbr0 must be valid page table
    fn enter_usermode(trap_frame: *const TrapFrame, ttbr0: u64) -> !;
}

/// Perform a context switch between two tasks
/// # Safety
/// Must be called with interrupts disabled
pub unsafe fn switch_context(current: &mut Task, next: &Task) {
    // Switch address space if needed
    if let Some(ref addr_space) = next.address_space {
        addr_space.activate();
    }

    // Switch CPU context
    context_switch(&mut current.context, &next.context);
}

/// Simple scheduler state
///
/// The scheduler manages all tasks and their state. For SMP safety:
/// - Task array modifications are protected by IrqGuard (single-core) or SpinLock (SMP)
/// - Current task index is stored per-CPU via percpu module
/// - The `current` field is deprecated - use `current_slot()` function instead
pub struct Scheduler {
    /// All tasks (shared across CPUs)
    pub tasks: [Option<Task>; MAX_TASKS],
    /// Currently running task index (DEPRECATED - use current_slot())
    /// Kept for backwards compatibility during migration
    pub current: usize,
    /// Generation counter per slot - increments when slot is reused
    /// PID = (slot + 1) | (generation << 8)
    /// This ensures stale PIDs from terminated tasks don't match new tasks
    generations: [u32; MAX_TASKS],
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
        }
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

    /// Wake a task by PID - O(1) lookup
    /// Returns true if task was woken
    pub fn wake_by_pid(&mut self, pid: TaskId) -> bool {
        if let Some(slot) = self.slot_by_pid(pid) {
            if let Some(ref mut task) = self.tasks[slot] {
                if task.state == TaskState::Blocked {
                    // If task was blocked on IPC, set x0 to EAGAIN (-11)
                    // so it retries the receive and gets the message.
                    // This is needed because receive_timeout pre-sets x0 to
                    // ETIMEDOUT (-110) before blocking.
                    if task.wait_reason == Some(WaitReason::Ipc) {
                        task.trap_frame.x0 = (-11i64) as u64; // EAGAIN
                    }
                    task.state = TaskState::Ready;
                    task.wait_reason = None;
                    task.wake_at = 0;
                    return true;
                }
            }
        }
        false
    }

    /// Check for timed-out blocked tasks, deliver timer events, and wake them
    /// Called from timer tick handler
    /// Returns number of tasks woken
    pub fn check_timeouts(&mut self, current_tick: u64) -> usize {
        let mut woken = 0;
        for task_opt in self.tasks.iter_mut() {
            if let Some(ref mut task) = task_opt {
                // Check blocking timeout (for receive with timeout, etc.)
                if task.state == TaskState::Blocked && task.wake_at != 0 {
                    if current_tick >= task.wake_at {
                        // Timeout expired - wake the task
                        task.state = TaskState::Ready;
                        task.wait_reason = None;
                        task.wake_at = 0;
                        woken += 1;
                    }
                }

                // Check timer_deadline - deliver Timer event if reached
                // This works for both Blocked and Ready tasks
                if task.timer_deadline != 0 && current_tick >= task.timer_deadline {
                    // Deliver Timer event to task's event queue
                    let timer_event = super::event::Event::timer(task.timer_deadline);
                    task.event_queue.push(timer_event);
                    task.timer_deadline = 0;  // Clear (one-shot timer)

                    // Wake task if blocked
                    if task.state == TaskState::Blocked {
                        task.state = TaskState::Ready;
                        woken += 1;
                    }
                }
            }
        }
        woken
    }

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

    /// Get task by slot index mutably
    pub fn task_mut(&mut self, slot: usize) -> Option<&mut Task> {
        self.tasks.get_mut(slot).and_then(|t| t.as_mut())
    }

    /// Get current task (uses per-CPU slot)
    pub fn current_task(&self) -> Option<&Task> {
        self.tasks[current_slot()].as_ref()
    }

    /// Get current task mutably (uses per-CPU slot)
    pub fn current_task_mut(&mut self) -> Option<&mut Task> {
        self.tasks[current_slot()].as_mut()
    }

    /// Get current task ID (uses per-CPU slot)
    pub fn current_task_id(&self) -> Option<TaskId> {
        self.tasks[current_slot()].as_ref().map(|t| t.id)
    }

    /// Terminate current task (uses per-CPU slot)
    pub fn terminate_current(&mut self, _exit_code: i32) {
        let slot = current_slot();
        if let Some(ref mut task) = self.tasks[slot] {
            task.state = TaskState::Terminated;
        }
    }

    /// Remove terminated tasks and free resources
    pub fn reap_terminated(&mut self) {
        for slot_idx in 0..MAX_TASKS {
            let should_reap = self.tasks[slot_idx]
                .as_ref()
                .map(|t| t.state == TaskState::Terminated)
                .unwrap_or(false);

            if should_reap {
                // Bump generation so any stale PIDs for this slot become invalid
                self.bump_generation(slot_idx);
                // Drop the task (frees resources)
                self.tasks[slot_idx] = None;
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
            task.state = TaskState::Running;

            if let Some(ref addr_space) = task.address_space {
                let ttbr0 = addr_space.get_ttbr0();
                let trap_frame = &mut task.trap_frame as *mut TrapFrame;

                // Set globals for exception handler (atomic for SMP safety)
                CURRENT_TRAP_FRAME.store(trap_frame, Ordering::Release);
                CURRENT_TTBR0.store(ttbr0, Ordering::Release);

                logln!("  Entering user mode:");
                logln!("    Entry:  0x{:016x}", task.trap_frame.elr_el1);
                logln!("    Stack:  0x{:016x}", task.trap_frame.sp_el0);
                logln!("    TTBR0:  0x{:016x}", ttbr0);

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
                task.state == TaskState::Ready || task.state == TaskState::Running
            } else {
                false
            }
        })
    }

    /// Priority-based scheduling with round-robin within same priority
    /// Higher priority tasks (lower Priority value) always run first.
    /// Among same-priority tasks, uses round-robin for fairness.
    pub fn schedule(&mut self) -> Option<usize> {
        let my_slot = current_slot();  // Use per-CPU current slot
        let mut best_slot: Option<usize> = None;
        let mut best_priority = Priority::Low;  // Start with lowest priority

        // First pass: find the highest priority ready task
        // Use round-robin starting from current+1 for fairness
        let start = (my_slot + 1) % MAX_TASKS;
        for i in 0..MAX_TASKS {
            let slot = (start + i) % MAX_TASKS;
            if let Some(ref task) = self.tasks[slot] {
                if task.state == TaskState::Ready {
                    // Lower priority value = higher priority
                    if best_slot.is_none() || task.priority < best_priority {
                        best_slot = Some(slot);
                        best_priority = task.priority;
                        // If we found a High priority task, no need to look further
                        if best_priority == Priority::High {
                            break;
                        }
                    }
                }
            }
        }

        // If we found a ready task, return it
        if best_slot.is_some() {
            return best_slot;
        }

        // Check if current task is still runnable (fallback)
        if let Some(ref task) = self.tasks[my_slot] {
            if task.state == TaskState::Running || task.state == TaskState::Ready {
                return Some(my_slot);
            }
        }

        None
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
                    if current.state == TaskState::Running {
                        current.state = TaskState::Ready;
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
                    t.state = TaskState::Running;
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
                    t.state = TaskState::Running;
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
            if task.state != TaskState::Ready && task.state != TaskState::Running {
                return false;
            }
        } else {
            return false;
        }

        let caller_slot = my_slot;  // Save our slot before switching

        // Mark current as ready (donating timeslice)
        if let Some(ref mut current) = self.tasks[caller_slot] {
            if current.state == TaskState::Running {
                current.state = TaskState::Ready;
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
            t.state = TaskState::Running;
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
            t.state = TaskState::Running;
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
        logln!("  Tasks:");
        for (i, slot) in self.tasks.iter().enumerate() {
            if let Some(ref task) = slot {
                let state_str = match task.state {
                    TaskState::Ready => "ready",
                    TaskState::Running => "RUNNING",
                    TaskState::Blocked => "blocked",
                    TaskState::Terminated => "terminated",
                };
                let marker = if i == self.current { ">" } else { " " };
                logln!("    {} [{}] {} ({})", marker, task.id, task.name_str(), state_str);
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
pub unsafe fn scheduler() -> &'static mut Scheduler {
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
        task.state == TaskState::Blocked
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
        if current.state == TaskState::Running {
            current.state = TaskState::Ready;
        }
    }

    // Find next task
    if let Some(next_slot) = sched.schedule() {
        if next_slot != my_slot {
            // Update both per-CPU slot and legacy field
            set_current_slot(next_slot);
            sched.current = next_slot;
            if let Some(ref mut next_task) = sched.tasks[next_slot] {
                next_task.state = TaskState::Running;
            }
            update_current_task_globals();
            // Signal to assembly that we switched tasks
            SYSCALL_SWITCHED_TASK.store(1, Ordering::Release);
        } else {
            // Same task selected - only mark as running if it was Ready
            if let Some(ref mut current) = sched.tasks[my_slot] {
                if current.state == TaskState::Ready {
                    current.state = TaskState::Running;
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

            // Wait for interrupt - timer tick or device IRQ will wake a task
            core::arch::asm!("wfi");

            // Disable IRQs to check scheduler state
            let _guard2 = crate::arch::aarch64::sync::IrqGuard::new();
            let sched2 = scheduler();

            if let Some(next_slot) = sched2.schedule() {
                // Found a ready task - switch to it
                // Update both per-CPU slot and legacy field
                set_current_slot(next_slot);
                sched2.current = next_slot;
                if let Some(ref mut next_task) = sched2.tasks[next_slot] {
                    next_task.state = TaskState::Running;
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
    logln!("  Context switch test:");
    logln!("    TrapFrame size: {} bytes", core::mem::size_of::<TrapFrame>());
    logln!("    CpuContext size: {} bytes", core::mem::size_of::<CpuContext>());
    logln!("    Task size: {} bytes", core::mem::size_of::<Task>());
    logln!("    [OK] Structures initialized");
}
