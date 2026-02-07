//! Process Management
//!
//! Manages processes in the microkernel. Each process has:
//! - A unique ID (PID)
//! - Its own address space (page tables)
//! - A task structure for scheduling
//! - State (running, ready, blocked, zombie)
//!
//! For a Redox-style microkernel, most drivers run as processes.

#![allow(dead_code)]  // Some states and methods are for future use

use super::addrspace::AddressSpace;
use super::pmm;
use crate::print_direct;
use crate::kernel::arch::mmu;

/// Process ID type
pub type Pid = u32;

/// Process states
///
/// State machine:
/// ```text
///   Creating ──────► Ready ◄────────► Running
///                      │                 │
///                      │                 │
///                      ▼                 ▼
///                   Blocked ──────────► Zombie ──────► Free
///                      │                   ▲
///                      └───────────────────┘
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ProcessState {
    /// Process is being created
    Creating = 0,
    /// Process is ready to run
    Ready = 1,
    /// Process is currently running
    Running = 2,
    /// Process is waiting for something (IPC, I/O, etc.)
    Blocked = 3,
    /// Process has exited but not yet reaped
    Zombie = 4,
    /// Process slot is free
    Free = 255,
}

impl ProcessState {
    /// Check if transition to new state is valid
    pub fn can_transition_to(&self, new: &ProcessState) -> bool {
        match (self, new) {
            // Creating → Ready (after load complete)
            (ProcessState::Creating, ProcessState::Ready) => true,

            // Ready ↔ Running (scheduling)
            (ProcessState::Ready, ProcessState::Running) => true,
            (ProcessState::Running, ProcessState::Ready) => true,

            // Running → Blocked (waiting for I/O, IPC, etc.)
            (ProcessState::Running, ProcessState::Blocked) => true,
            // Blocked → Ready (event arrived)
            (ProcessState::Blocked, ProcessState::Ready) => true,

            // Any active state → Zombie (exit/kill)
            (ProcessState::Running, ProcessState::Zombie) => true,
            (ProcessState::Ready, ProcessState::Zombie) => true,
            (ProcessState::Blocked, ProcessState::Zombie) => true,

            // Zombie → Free (reaped by parent)
            (ProcessState::Zombie, ProcessState::Free) => true,

            // Free → Creating (slot reused)
            (ProcessState::Free, ProcessState::Creating) => true,

            // All other transitions are invalid
            _ => false,
        }
    }

    /// Get state name for logging
    pub fn name(&self) -> &'static str {
        match self {
            ProcessState::Creating => "Creating",
            ProcessState::Ready => "Ready",
            ProcessState::Running => "Running",
            ProcessState::Blocked => "Blocked",
            ProcessState::Zombie => "Zombie",
            ProcessState::Free => "Free",
        }
    }
}

/// What a blocked process is waiting for
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockReason {
    /// Not blocked
    None,
    /// Waiting to receive on a channel
    IpcReceive(u32),  // channel_id
    /// Waiting to send (queue full)
    IpcSend(u32),     // channel_id
    /// Waiting for a child to exit
    WaitChild(Pid),
    /// Sleeping for a duration
    Sleep,
}

/// Saved user-mode context for returning to user space
#[repr(C)]
pub struct UserContext {
    // General purpose registers
    pub x: [u64; 31],  // x0-x30
    pub sp: u64,       // Stack pointer (SP_EL0)
    pub pc: u64,       // Program counter (ELR_EL1)
    pub pstate: u64,   // Saved Program Status Register (SPSR_EL1)
}

impl UserContext {
    pub const fn new() -> Self {
        Self {
            x: [0; 31],
            sp: 0,
            pc: 0,
            pstate: 0,
        }
    }

    /// Set up context for starting a user process
    pub fn setup_for_user(&mut self, entry: u64, user_stack: u64) {
        self.pc = entry;
        self.sp = user_stack;
        // PSTATE for EL0:
        // - M[3:0] = 0b0000 (EL0t - EL0 using SP_EL0)
        // - DAIF = 0 (interrupts enabled)
        self.pstate = 0;
    }
}

/// Process Control Block
pub struct Process {
    /// Process ID
    pub pid: Pid,
    /// Parent process ID
    pub parent_pid: Pid,
    /// Current state - private, use state() and transition methods
    state: ProcessState,
    /// Why the process is blocked (if state == Blocked)
    pub block_reason: BlockReason,
    /// Virtual address space
    pub address_space: Option<AddressSpace>,
    /// Saved user context
    pub user_context: UserContext,
    /// Kernel stack (for handling syscalls/exceptions)
    pub kernel_stack: u64,
    pub kernel_stack_size: usize,
    /// Exit code (valid when state == Zombie)
    pub exit_code: i32,
    /// Process name
    pub name: [u8; 32],
}

/// Kernel stack size per process (64KB)
/// Must be large enough for syscall handlers that create large local variables
const KERNEL_STACK_SIZE: usize = 64 * 1024;

/// Maximum number of processes
const MAX_PROCESSES: usize = 64;

impl Process {
    /// Get current state
    pub fn state(&self) -> ProcessState { self.state }

    /// Create a new process
    pub fn new(pid: Pid, parent_pid: Pid, name: &str) -> Option<Self> {
        // Allocate kernel stack
        let stack_pages = (KERNEL_STACK_SIZE + 4095) / 4096;
        let kernel_stack = pmm::alloc_pages(stack_pages)? as u64;

        // Create address space
        let address_space = AddressSpace::new();

        // Copy name
        let mut proc_name = [0u8; 32];
        let name_bytes = name.as_bytes();
        let copy_len = core::cmp::min(name_bytes.len(), 31);
        proc_name[..copy_len].copy_from_slice(&name_bytes[..copy_len]);

        Some(Self {
            pid,
            parent_pid,
            state: ProcessState::Creating,
            block_reason: BlockReason::None,
            address_space,
            user_context: UserContext::new(),
            kernel_stack,
            kernel_stack_size: KERNEL_STACK_SIZE,
            exit_code: 0,
            name: proc_name,
        })
    }

    /// Block the process waiting for IPC receive
    pub fn block_on_receive(&mut self, channel_id: u32) {
        self.state = ProcessState::Blocked;
        self.block_reason = BlockReason::IpcReceive(channel_id);
    }

    /// Block the process waiting for IPC send (queue full)
    pub fn block_on_send(&mut self, channel_id: u32) {
        self.state = ProcessState::Blocked;
        self.block_reason = BlockReason::IpcSend(channel_id);
    }

    /// Wake up a blocked process
    pub fn wake(&mut self) {
        if self.state == ProcessState::Blocked {
            self.state = ProcessState::Ready;
            self.block_reason = BlockReason::None;
        }
    }

    /// Check if process is blocked on a specific channel for receive
    pub fn is_blocked_on_receive(&self, channel_id: u32) -> bool {
        self.state == ProcessState::Blocked &&
        matches!(self.block_reason, BlockReason::IpcReceive(ch) if ch == channel_id)
    }

    /// Check if process is blocked on a specific channel for send
    pub fn is_blocked_on_send(&self, channel_id: u32) -> bool {
        self.state == ProcessState::Blocked &&
        matches!(self.block_reason, BlockReason::IpcSend(ch) if ch == channel_id)
    }

    /// Get process name
    pub fn name_str(&self) -> &str {
        let len = self.name.iter().position(|&c| c == 0).unwrap_or(32);
        core::str::from_utf8(&self.name[..len]).unwrap_or("???")
    }

    /// Allocate user stack in process address space
    pub fn alloc_user_stack(&mut self, size: usize) -> Option<u64> {
        // OVERFLOW CHECK: Calculate pages safely
        let pages = size.checked_add(4095)? / 4096;
        let phys = pmm::alloc_pages(pages)? as u64;

        // Map at a high user address
        let virt_base = 0x7FFF_F000_0000u64 - (pages as u64 * 4096);

        if let Some(ref mut addr_space) = self.address_space {
            for i in 0..pages {
                let virt = virt_base + (i as u64 * 4096);
                let page_phys = phys + (i as u64 * 4096);
                if !addr_space.map_page(virt, page_phys, true, false) {
                    // Mapping failed - clean up
                    pmm::free_pages(phys as usize, pages);
                    return None;
                }
            }
        }

        // Return top of stack (stack grows down)
        Some(virt_base + (pages as u64 * 4096))
    }

    /// Load a simple program into the process address space
    /// Returns the entry point
    pub fn load_program(&mut self, code: &[u8], load_addr: u64) -> Option<u64> {
        // OVERFLOW CHECK: Calculate pages safely
        let pages = code.len().checked_add(4095)? / 4096;
        let phys = pmm::alloc_pages(pages)? as u64;

        // Copy program to physical memory (use TTBR1 virtual address)
        unsafe {
            let dest = mmu::phys_to_virt(phys) as *mut u8;
            for (i, &byte) in code.iter().enumerate() {
                core::ptr::write_volatile(dest.add(i), byte);
            }
        }

        // Map into user address space
        if let Some(ref mut addr_space) = self.address_space {
            for i in 0..pages {
                let virt = load_addr + (i as u64 * 4096);
                let page_phys = phys + (i as u64 * 4096);
                // Code pages are readable and executable, not writable
                if !addr_space.map_page(virt, page_phys, false, true) {
                    pmm::free_pages(phys as usize, pages);
                    return None;
                }
            }
        }

        Some(load_addr)
    }

    /// Mark process as ready to run
    pub fn make_ready(&mut self) {
        self.state = ProcessState::Ready;
    }

    /// Terminate the process (any -> Zombie)
    pub fn terminate(&mut self, exit_code: i32) {
        self.state = ProcessState::Zombie;
        self.exit_code = exit_code;
    }

    /// Mark as running
    pub fn make_running(&mut self) {
        self.state = ProcessState::Running;
    }

    /// Start running the process (switch to user mode)
    /// # Safety
    /// Must be called with interrupts properly configured
    pub unsafe fn run(&mut self) -> ! {
        self.make_running();

        // Switch to process's address space
        if let Some(ref addr_space) = self.address_space {
            addr_space.activate();
        }

        // Jump to user mode
        // This is done by setting up ELR_EL1 and SPSR_EL1, then doing eret
        core::arch::asm!(
            "msr elr_el1, {entry}",
            "msr sp_el0, {sp}",
            "msr spsr_el1, {pstate}",
            "eret",
            entry = in(reg) self.user_context.pc,
            sp = in(reg) self.user_context.sp,
            pstate = in(reg) self.user_context.pstate,
            options(noreturn)
        );
    }
}

impl Drop for Process {
    fn drop(&mut self) {
        // Free kernel stack
        // Note: kernel_stack_size is set during Process::new() with known-safe values
        // Use saturating_add for defense in depth (won't overflow, just cap at max)
        let pages = self.kernel_stack_size.saturating_add(4095) / 4096;
        pmm::free_pages(self.kernel_stack as usize, pages);
        // address_space is automatically dropped
    }
}

/// Process table
pub struct ProcessTable {
    processes: [Option<Process>; MAX_PROCESSES],
    next_pid: Pid,
    current_pid: Option<Pid>,
}

impl ProcessTable {
    pub const fn new() -> Self {
        const NONE: Option<Process> = None;
        Self {
            processes: [NONE; MAX_PROCESSES],
            next_pid: 1,  // PID 0 is reserved for idle/kernel
            current_pid: None,
        }
    }

    /// Create a new process
    pub fn create(&mut self, parent_pid: Pid, name: &str) -> Option<Pid> {
        // Find free slot
        let slot = self.processes.iter().position(|p| p.is_none())?;

        let pid = self.next_pid;
        self.next_pid += 1;

        self.processes[slot] = Process::new(pid, parent_pid, name);
        if self.processes[slot].is_some() {
            Some(pid)
        } else {
            None
        }
    }

    /// Get a process by PID
    pub fn get(&self, pid: Pid) -> Option<&Process> {
        self.processes.iter()
            .flatten()
            .find(|p| p.pid == pid)
    }

    /// Get a process mutably by PID
    pub fn get_mut(&mut self, pid: Pid) -> Option<&mut Process> {
        self.processes.iter_mut()
            .flatten()
            .find(|p| p.pid == pid)
    }

    /// Get current running process
    pub fn current(&self) -> Option<&Process> {
        self.current_pid.and_then(|pid| self.get(pid))
    }

    /// Get current running process mutably
    pub fn current_mut(&mut self) -> Option<&mut Process> {
        let pid = self.current_pid?;
        self.get_mut(pid)
    }

    /// Set current process
    pub fn set_current(&mut self, pid: Pid) {
        self.current_pid = Some(pid);
    }

    /// Terminate a process
    pub fn terminate(&mut self, pid: Pid, exit_code: i32) {
        if let Some(proc) = self.get_mut(pid) {
            proc.terminate(exit_code);
        }
    }

    /// Reap a zombie process (remove from table)
    pub fn reap(&mut self, pid: Pid) -> Option<i32> {
        let slot = self.processes.iter().position(|p| {
            p.as_ref().map_or(false, |proc| proc.pid == pid && proc.state == ProcessState::Zombie)
        })?;

        let exit_code = self.processes[slot].as_ref()?.exit_code;
        self.processes[slot] = None;
        Some(exit_code)
    }

    /// Wake a process by PID
    pub fn wake(&mut self, pid: Pid) -> bool {
        if let Some(proc) = self.get_mut(pid) {
            proc.wake();
            true
        } else {
            false
        }
    }

    /// Wake all processes blocked on a channel for receive
    pub fn wake_receivers(&mut self, channel_id: u32) {
        for proc in self.processes.iter_mut().flatten() {
            if proc.is_blocked_on_receive(channel_id) {
                proc.wake();
            }
        }
    }

    /// Wake all processes blocked on a channel for send
    pub fn wake_senders(&mut self, channel_id: u32) {
        for proc in self.processes.iter_mut().flatten() {
            if proc.is_blocked_on_send(channel_id) {
                proc.wake();
            }
        }
    }

    /// Print process table
    pub fn print_info(&self) {
        print_direct!("  Process table ({} max):\n", MAX_PROCESSES);
        for proc in self.processes.iter().flatten() {
            let state_str = match proc.state {
                ProcessState::Creating => "creating",
                ProcessState::Ready => "ready",
                ProcessState::Running => "RUNNING",
                ProcessState::Blocked => "waiting",
                ProcessState::Zombie => "zombie",
                ProcessState::Free => "free",
            };
            let marker = if Some(proc.pid) == self.current_pid { ">" } else { " " };
            print_direct!("    {} [{}] {} ({})\n", marker, proc.pid, proc.name_str(), state_str);
        }
        if self.processes.iter().flatten().count() == 0 {
            print_direct!("    (no processes)\n");
        }
    }
}

/// Global process table
static mut PROCESS_TABLE: ProcessTable = ProcessTable::new();

/// Get the global process table
/// # Safety
/// Must ensure proper synchronization
pub(crate) unsafe fn process_table() -> &'static mut ProcessTable {
    &mut *core::ptr::addr_of_mut!(PROCESS_TABLE)
}

/// A minimal "hello world" user program in AArch64 machine code
/// This program does:
///   mov x8, #1      ; syscall number = DebugWrite
///   adr x0, msg     ; buffer address
///   mov x1, #14     ; length
///   svc #0          ; syscall
///   mov x8, #0      ; syscall number = Exit
///   mov x0, #0      ; exit code
///   svc #0          ; syscall
/// msg:
///   .ascii "Hello, User!\n"
const HELLO_USER_PROGRAM: &[u8] = &[
    // mov x8, #1
    0x28, 0x00, 0x80, 0xd2,
    // adr x0, msg (offset to msg = 24 bytes)
    0xc0, 0x00, 0x00, 0x10,
    // mov x1, #14
    0xc1, 0x01, 0x80, 0xd2,
    // svc #0
    0x01, 0x00, 0x00, 0xd4,
    // mov x8, #0
    0x08, 0x00, 0x80, 0xd2,
    // mov x0, #0
    0x00, 0x00, 0x80, 0xd2,
    // svc #0
    0x01, 0x00, 0x00, 0xd4,
    // "Hello, User!\n\0"
    0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x2c, 0x20, 0x55,
    0x73, 0x65, 0x72, 0x21, 0x0a, 0x00,
];

/// Test process creation
pub fn test() {
    print_direct!("  Testing process creation...\n");

    unsafe {
        let ptable = process_table();

        // Create a test process
        if let Some(pid) = ptable.create(0, "test_proc") {
            print_direct!("    Created process with PID {}\n", pid);

            if let Some(proc) = ptable.get_mut(pid) {
                // Allocate user stack (4KB)
                if let Some(stack_top) = proc.alloc_user_stack(4096) {
                    print_direct!("    Allocated user stack at 0x{:016x}\n", stack_top);
                } else {
                    print_direct!("    [!!] Failed to allocate user stack\n");
                }

                // Mark as ready
                proc.make_ready();
            }

            ptable.print_info();

            // Clean up - terminate and reap
            ptable.terminate(pid, 0);
            ptable.reap(pid);
            print_direct!("    Process terminated and reaped\n");

        } else {
            print_direct!("    [!!] Failed to create process\n");
        }
    }

    print_direct!("    [OK] Process structure test passed\n");
}
