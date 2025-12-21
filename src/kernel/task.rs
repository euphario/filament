//! Task Management and Context Switching
//!
//! Provides task structures and context switching for the microkernel.
//! Each task has its own saved CPU state and address space.

#![allow(dead_code)]  // Some wait reasons and methods are for future use

use super::addrspace::AddressSpace;
use super::event::EventQueue;
use super::fd::FdTable;
use super::pmm;
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
pub const MAX_HEAP_MAPPINGS: usize = 32;

/// User heap region start (after code/stack regions)
pub const USER_HEAP_START: u64 = 0x5000_0000;

/// User heap region end
pub const USER_HEAP_END: u64 = 0x7000_0000;

/// A single heap mapping entry
#[derive(Clone, Copy)]
pub struct HeapMapping {
    /// Virtual address of mapping
    pub virt_addr: u64,
    /// Physical address of mapping
    pub phys_addr: u64,
    /// Number of pages
    pub num_pages: usize,
}

impl HeapMapping {
    pub const fn empty() -> Self {
        Self {
            virt_addr: 0,
            phys_addr: 0,
            num_pages: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.num_pages == 0
    }
}

/// Maximum number of children a process can have
pub const MAX_CHILDREN: usize = 16;

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
    /// Capability set for this task
    pub capabilities: super::caps::Capabilities,
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
            capabilities: super::caps::Capabilities::ALL,  // Kernel tasks get all capabilities
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

        // Create address space
        let address_space = AddressSpace::new()?;

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
            capabilities: super::caps::Capabilities::DRIVER_DEFAULT,  // User tasks default to driver caps
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

    /// Allocate and map memory pages into user heap
    /// Returns virtual address on success, None on failure
    pub fn mmap(&mut self, size: usize, writable: bool, executable: bool) -> Option<u64> {
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

        // Allocate physical pages
        let phys_addr = pmm::alloc_pages(num_pages)? as u64;

        // Zero the pages using kernel virtual address (via TTBR1)
        // During syscalls, TTBR0 points to user page tables
        unsafe {
            let kva = (crate::arch::aarch64::mmu::KERNEL_VIRT_BASE | phys_addr) as *mut u8;
            for i in 0..(num_pages * 4096) {
                core::ptr::write_volatile(kva.add(i), 0);
            }
        }

        // Map pages into address space
        let addr_space = self.address_space.as_mut()?;
        for i in 0..num_pages {
            let page_virt = virt_addr + (i * 4096) as u64;
            let page_phys = phys_addr + (i * 4096) as u64;
            if !addr_space.map_page(page_virt, page_phys, writable, executable) {
                // Cleanup on failure
                pmm::free_pages(phys_addr as usize, num_pages);
                return None;
            }
        }

        // Record mapping
        self.heap_mappings[slot] = HeapMapping {
            virt_addr,
            phys_addr,
            num_pages,
        };

        // Bump heap pointer
        self.heap_next = virt_addr + mapping_size;

        Some(virt_addr)
    }

    /// Allocate DMA-capable memory, returning both virtual and physical addresses
    pub fn mmap_dma(&mut self, size: usize) -> Option<(u64, u64)> {
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

        // Allocate physical pages
        let phys_addr = pmm::alloc_pages(num_pages)? as u64;

        // Zero the pages using kernel's TTBR1 mapping (high addresses)
        // During syscalls, TTBR0 points to user page tables, so we must use kernel VA
        // Then flush cache so DMA device sees zeros
        unsafe {
            // Use kernel virtual address (via TTBR1) to access physical memory
            let kva = (crate::arch::aarch64::mmu::KERNEL_VIRT_BASE | phys_addr) as *mut u8;
            for i in 0..(num_pages * 4096) {
                core::ptr::write_volatile(kva.add(i), 0);
            }
            // Flush cache for the zeroed memory - DMA devices need to see clean memory
            for page in 0..num_pages {
                let page_kva = crate::arch::aarch64::mmu::KERNEL_VIRT_BASE | (phys_addr + (page * 4096) as u64);
                for offset in (0..4096).step_by(64) {
                    let addr = page_kva + offset;
                    core::arch::asm!("dc civac, {}", in(reg) addr);
                }
            }
            core::arch::asm!("dsb sy");
        }

        // Map pages into address space (cacheable, userspace handles cache ops)
        let addr_space = self.address_space.as_mut()?;
        for i in 0..num_pages {
            let page_virt = virt_addr + (i * 4096) as u64;
            let page_phys = phys_addr + (i * 4096) as u64;
            // DMA memory uses cacheable attributes - userspace must do cache maintenance
            if !addr_space.map_dma_page(page_virt, page_phys, true) {
                // Cleanup on failure
                pmm::free_pages(phys_addr as usize, num_pages);
                return None;
            }
        }

        // Invalidate TLB entries for the newly mapped pages
        // This ensures the CPU uses the new device memory attributes
        unsafe {
            for i in 0..num_pages {
                let page_virt = virt_addr + (i * 4096) as u64;
                // TLBI expects VA[55:12] in bits[43:0] of the operand
                let tlbi_addr = page_virt >> 12;
                core::arch::asm!(
                    "tlbi vale1is, {addr}",
                    addr = in(reg) tlbi_addr,
                );
            }
            core::arch::asm!("dsb ish", "isb");
        }

        // Record mapping
        self.heap_mappings[slot] = HeapMapping {
            virt_addr,
            phys_addr,
            num_pages,
        };

        // Bump heap pointer
        self.heap_next = virt_addr + mapping_size;

        Some((virt_addr, phys_addr))
    }

    /// Map existing physical memory into user address space (for shared memory)
    /// Does NOT allocate physical memory - just creates the mapping
    /// Returns virtual address on success
    pub fn mmap_phys(&mut self, phys_addr: u64, size: usize) -> Option<u64> {
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

        // Map pages into address space (cacheable, userspace handles cache ops)
        let addr_space = self.address_space.as_mut()?;
        for i in 0..num_pages {
            let page_virt = virt_addr + (i * 4096) as u64;
            let page_phys = phys_addr + (i * 4096) as u64;
            // Shared memory uses cacheable attributes - userspace does cache maintenance
            if !addr_space.map_dma_page(page_virt, page_phys, true) {
                return None;
            }
        }

        // Invalidate TLB entries for the newly mapped pages
        unsafe {
            for i in 0..num_pages {
                let page_virt = virt_addr + (i * 4096) as u64;
                let tlbi_addr = page_virt >> 12;
                core::arch::asm!(
                    "tlbi vale1is, {addr}",
                    addr = in(reg) tlbi_addr,
                );
            }
            core::arch::asm!("dsb ish", "isb");
        }

        // Record mapping (phys_addr is the shared region's physical address)
        self.heap_mappings[slot] = HeapMapping {
            virt_addr,
            phys_addr,
            num_pages,
        };

        // Bump heap pointer
        self.heap_next = virt_addr + mapping_size;

        Some(virt_addr)
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
        // Use tlbi vale1is (VA, Last level, EL1, Inner Shareable) for each page
        for i in 0..mapping.num_pages {
            let page_virt = mapping.virt_addr + (i * 4096) as u64;
            // TLBI expects VA[55:12] in bits[43:0] of the operand
            let tlbi_addr = page_virt >> 12;
            unsafe {
                core::arch::asm!(
                    "tlbi vale1is, {addr}",
                    addr = in(reg) tlbi_addr,
                );
            }
        }

        // Ensure TLB invalidation completes before freeing physical memory
        unsafe {
            core::arch::asm!(
                "dsb ish",  // Ensure TLB invalidation completes
                "isb",      // Synchronize instruction stream
            );
        }

        // Free physical pages (safe now that TLB is invalidated)
        pmm::free_pages(mapping.phys_addr as usize, mapping.num_pages);

        // Clear mapping slot
        self.heap_mappings[slot] = HeapMapping::empty();

        true
    }
}

impl Drop for Task {
    fn drop(&mut self) {
        // Free heap mappings
        for mapping in &self.heap_mappings {
            if !mapping.is_empty() {
                pmm::free_pages(mapping.phys_addr as usize, mapping.num_pages);
            }
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
    // Save TTBR0 value
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
    adr     x10, 1f                 // Get physical address of trampoline
    movz    x11, #0xFFFF, lsl #48   // KERNEL_VIRT_BASE = 0xFFFF000000000000
    orr     x10, x10, x11           // Convert to TTBR1 virtual address
    br      x10                     // Jump to TTBR1 space

1:
    // Now executing from TTBR1 space - safe to modify TTBR0
    msr     ttbr0_el1, x9
    isb
    tlbi    vmalle1                 // Flush all TLB entries
    dsb     sy
    isb

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
pub struct Scheduler {
    /// All tasks
    pub tasks: [Option<Task>; MAX_TASKS],
    /// Currently running task index
    pub current: usize,
    /// Generation counter per slot - increments when slot is reused
    /// PID = (slot + 1) | (generation << 8)
    /// This ensures stale PIDs from terminated tasks don't match new tasks
    generations: [u32; MAX_TASKS],
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
    /// PID format: bits[7:0] = slot + 1 (1-16), bits[31:8] = generation
    fn make_pid(&self, slot: usize) -> TaskId {
        let slot_bits = (slot + 1) as u32;  // +1 to avoid PID 0
        let gen_bits = self.generations[slot] << 8;
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
    fn bump_generation(&mut self, slot: usize) {
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
                    task.state = TaskState::Ready;
                    return true;
                }
            }
        }
        false
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

    /// Get current task
    pub fn current_task(&self) -> Option<&Task> {
        self.tasks[self.current].as_ref()
    }

    /// Get current task mutably
    pub fn current_task_mut(&mut self) -> Option<&mut Task> {
        self.tasks[self.current].as_mut()
    }

    /// Get current task ID
    pub fn current_task_id(&self) -> Option<TaskId> {
        self.tasks[self.current].as_ref().map(|t| t.id)
    }

    /// Terminate current task
    pub fn terminate_current(&mut self, _exit_code: i32) {
        if let Some(ref mut task) = self.tasks[self.current] {
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
        self.current = slot;

        if let Some(ref mut task) = self.tasks[slot] {
            task.state = TaskState::Running;

            if let Some(ref addr_space) = task.address_space {
                let ttbr0 = addr_space.get_ttbr0();
                let trap_frame = &mut task.trap_frame as *mut TrapFrame;

                // Set globals for exception handler
                CURRENT_TRAP_FRAME = trap_frame;
                CURRENT_TTBR0 = ttbr0;

                logln!("  Entering user mode:");
                logln!("    Entry:  0x{:016x}", task.trap_frame.elr_el1);
                logln!("    Stack:  0x{:016x}", task.trap_frame.sp_el0);
                logln!("    TTBR0:  0x{:016x}", ttbr0);

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
        let mut best_slot: Option<usize> = None;
        let mut best_priority = Priority::Low;  // Start with lowest priority

        // First pass: find the highest priority ready task
        // Use round-robin starting from current+1 for fairness
        let start = (self.current + 1) % MAX_TASKS;
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
        if let Some(ref task) = self.tasks[self.current] {
            if task.state == TaskState::Running || task.state == TaskState::Ready {
                return Some(self.current);
            }
        }

        None
    }

    /// Perform a context switch to the next ready task
    /// # Safety
    /// Must be called with interrupts disabled
    pub unsafe fn switch_to_next(&mut self) {
        if let Some(next_idx) = self.schedule() {
            if next_idx != self.current {
                // Mark current as ready (if still running)
                if let Some(ref mut current) = self.tasks[self.current] {
                    if current.state == TaskState::Running {
                        current.state = TaskState::Ready;
                    }
                }

                // Get pointers to current and next contexts
                let current_ctx = if let Some(ref mut t) = self.tasks[self.current] {
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

                // Update current index
                self.current = next_idx;

                // Mark next as running
                if let Some(ref mut t) = self.tasks[next_idx] {
                    t.state = TaskState::Running;
                }

                // Actually switch
                context_switch(current_ctx, next_ctx);

                // We return here when switched back to this task
            }
        }
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

/// Global scheduler instance
static mut SCHEDULER: Scheduler = Scheduler::new();

/// Current task's trap frame pointer - used by exception handler
#[no_mangle]
pub static mut CURRENT_TRAP_FRAME: *mut TrapFrame = core::ptr::null_mut();

/// Current task's TTBR0 value - used by exception handler
#[no_mangle]
pub static mut CURRENT_TTBR0: u64 = 0;

/// Flag: set to 1 when syscall switched tasks, 0 otherwise
/// Assembly checks this to skip storing return value when switched
#[no_mangle]
pub static mut SYSCALL_SWITCHED_TASK: u64 = 0;

/// Get the global scheduler
/// # Safety
/// Must ensure proper synchronization (interrupts disabled)
pub unsafe fn scheduler() -> &'static mut Scheduler {
    &mut *core::ptr::addr_of_mut!(SCHEDULER)
}

/// Update the current task globals after scheduling decision
/// # Safety
/// Must be called with valid current task
pub unsafe fn update_current_task_globals() {
    let sched = scheduler();
    if let Some(ref mut task) = sched.tasks[sched.current] {
        CURRENT_TRAP_FRAME = &mut task.trap_frame as *mut TrapFrame;
        if let Some(ref addr_space) = task.address_space {
            CURRENT_TTBR0 = addr_space.get_ttbr0();
        }
    }
}

/// Check NEED_RESCHED flag and perform reschedule if needed.
/// Called at safe points (syscall exit, exception return).
///
/// This is the "bottom half" of preemption - the timer IRQ just sets a flag,
/// and this function does the actual work.
///
/// This function ONLY handles scheduling - no logging or other side effects.
/// Log flushing and other deferred work should be done separately.
///
/// # Safety
/// Must be called from kernel context (not IRQ context).
pub unsafe fn do_resched_if_needed() {
    // Check and clear the flag atomically
    if !crate::arch::aarch64::sync::cpu_flags().check_and_clear_resched() {
        return; // No reschedule needed
    }

    // Do the reschedule with IRQs disabled for the critical section
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();
    let sched = scheduler();

    // Mark current as ready (it was running)
    if let Some(ref mut current) = sched.tasks[sched.current] {
        if current.state == TaskState::Running {
            current.state = TaskState::Ready;
        }
    }

    // Find next task
    if let Some(next_slot) = sched.schedule() {
        if next_slot != sched.current {
            sched.current = next_slot;
            if let Some(ref mut next_task) = sched.tasks[next_slot] {
                next_task.state = TaskState::Running;
            }
            update_current_task_globals();
            // Signal to assembly that we switched tasks
            SYSCALL_SWITCHED_TASK = 1;
        } else {
            // Same task, mark as running again
            if let Some(ref mut current) = sched.tasks[sched.current] {
                current.state = TaskState::Running;
            }
        }
    }
}

/// Yield CPU to another task (callable from kernel code like schemes)
/// This switches to another ready task while keeping current task in its current state
/// (Ready or Blocked depending on what caller set).
///
/// When the yielded task is scheduled again, this function returns.
///
/// This is the safe variant that handles IRQ disabling internally.
/// # Safety
/// Must be called from kernel context (not IRQ handlers).
pub unsafe fn yield_cpu() {
    // Use IrqGuard to ensure IRQs are disabled during the switch.
    // The guard will re-enable IRQs after we return (if they were enabled).
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();
    yield_cpu_locked();
}

/// Yield CPU with IRQs already disabled.
///
/// Use this variant when you're already in a critical section with IRQs disabled
/// (e.g., holding an IrqGuard). Avoids redundant IRQ masking.
///
/// # Safety
/// - Must be called with IRQs disabled
/// - Must be called from kernel context (not IRQ handlers)
pub unsafe fn yield_cpu_locked() {
    let sched = scheduler();
    let caller_slot = sched.current;

    // Mark current task as Ready so it can be scheduled again
    if let Some(ref mut t) = sched.tasks[caller_slot] {
        if t.state == TaskState::Running {
            t.state = TaskState::Ready;
        }
    }

    // Find next ready task
    if let Some(next_slot) = sched.schedule() {
        if next_slot != caller_slot {
            // Get contexts for switch
            let current_ctx = if let Some(ref mut t) = sched.tasks[caller_slot] {
                &mut t.context as *mut CpuContext
            } else {
                return;
            };

            let next_ctx = if let Some(ref t) = sched.tasks[next_slot] {
                &t.context as *const CpuContext
            } else {
                return;
            };

            // Switch address space if next task has one
            if let Some(ref task) = sched.tasks[next_slot] {
                if let Some(ref addr_space) = task.address_space {
                    addr_space.activate();
                }
            }

            // Update current index
            sched.current = next_slot;

            // Mark next as running
            if let Some(ref mut t) = sched.tasks[next_slot] {
                t.state = TaskState::Running;
            }

            // Actually switch contexts
            context_switch(current_ctx, next_ctx);

            // We return here when switched back to this task
            // Mark ourselves as Running again
            if let Some(ref mut t) = sched.tasks[sched.current] {
                t.state = TaskState::Running;
            }
            // Restore our address space
            if let Some(ref task) = sched.tasks[sched.current] {
                if let Some(ref addr_space) = task.address_space {
                    addr_space.activate();
                }
            }
        }
    }
    // If no other task to run, just return and continue current task
}

/// Test context switching
pub fn test() {
    logln!("  Context switch test:");
    logln!("    TrapFrame size: {} bytes", core::mem::size_of::<TrapFrame>());
    logln!("    CpuContext size: {} bytes", core::mem::size_of::<CpuContext>());
    logln!("    Task size: {} bytes", core::mem::size_of::<Task>());
    logln!("    [OK] Structures initialized");
}
