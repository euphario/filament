//! Process Operations Backend Implementation
//!
//! Implements the `ProcessOps` trait for process lifecycle operations.
//! This is a minimal interface: exit, kill, spawn.
//!
//! # Design Notes
//!
//! - No `wait()` syscall - child exit generates an IPC message to parent
//! - No `yield()` - tasks block on IPC when waiting
//! - No `sleep_for_ipc()` - blocking is implicit in IPC operations

use crate::kernel::traits::process_ops::{ProcessOps, ProcessError};
use crate::kernel::traits::task::{TaskId, Capabilities as TraitCapabilities};
use crate::kernel::task::{self, lifecycle};
use crate::kernel::caps::Capabilities;
use crate::kernel::elf;
use crate::{kinfo, kwarn, print_direct};

// ============================================================================
// Kernel Process Operations Implementation
// ============================================================================

/// Kernel process operations backend implementation
pub struct KernelProcessOps;

impl KernelProcessOps {
    /// Create a new kernel process operations instance
    pub const fn new() -> Self {
        Self
    }
}

impl ProcessOps for KernelProcessOps {
    fn exit(&self, task_id: TaskId, code: i32) -> ! {
        // Flush UART buffer so pending output from this process appears first
        while crate::platform::current::uart::has_buffered_output() {
            crate::platform::current::uart::flush_buffer();
        }

        print_direct!("\n========================================\n");
        print_direct!("  Process {} exited with code: {}\n", task_id, code);
        print_direct!("========================================\n");

        unsafe {
            let mut sched = task::scheduler();

            // Delegate state transition and parent notification to lifecycle module
            if let Err(e) = lifecycle::exit(&mut *sched, task_id, code) {
                kinfo!("process_ops", "exit_lifecycle_error"; pid = task_id as u64, err = e as i64);
            }

            // Schedule next task
            if let Some(next_slot) = sched.schedule() {
                kinfo!("process_ops", "exit_scheduled"; next = next_slot);

                // Update scheduler state - actual TTBR0 switch happens in assembly
                // at exception return (assembly loads per-CPU TTBR0 and switches)
                task::set_current_slot(next_slot);
                if let Some(next) = sched.task_mut(next_slot) {
                    crate::transition_or_evict!(next, set_running);
                    // Update per-CPU data directly (can't call update_current_task_globals
                    // here because we already hold the scheduler lock)
                    let trap_ptr = &mut next.trap_frame as *mut task::TrapFrame;
                    crate::kernel::percpu::set_trap_frame(trap_ptr);
                    if let Some(ref addr_space) = next.address_space {
                        crate::kernel::percpu::set_ttbr0(addr_space.get_ttbr0());
                    }
                }
                // Only set flag for user tasks (not idle) - this tells IRQ handler
                // to use user return path instead of kernel return path
                if !crate::kernel::sched::is_idle_slot(next_slot) {
                    crate::kernel::percpu::set_syscall_switched(1);
                }

                // Return to the new task
                // This is a diverging function, so we need to jump to context switch
                crate::kernel::sched::reschedule();
            }

            // No more tasks - halt
            kinfo!("process_ops", "halt_no_tasks");
            loop {
                core::arch::asm!("wfi");
            }
        }
    }

    fn kill(&self, killer: TaskId, target: TaskId) -> Result<(), ProcessError> {
        // Never allow killing init (PID 1)
        if target == 1 {
            kwarn!("security", "kill_init_denied"; caller = killer as u64);
            return Err(ProcessError::PermissionDenied);
        }

        unsafe {
            let mut sched = task::scheduler();

            // Delegate to lifecycle module for permission check, state transition, notification
            match lifecycle::kill(&mut *sched, target, killer) {
                Ok(()) => {
                    // If killing self, need to reschedule
                    if target == killer {
                        if let Some(next_slot) = sched.schedule() {
                            // Update scheduler state - actual TTBR0 switch happens in assembly
                            // at exception return (assembly loads per-CPU TTBR0 and switches)
                            task::set_current_slot(next_slot);
                            if let Some(next) = sched.task_mut(next_slot) {
                                crate::transition_or_evict!(next, set_running);
                                // Update per-CPU data directly (can't call update_current_task_globals
                                // here because we already hold the scheduler lock)
                                let trap_ptr = &mut next.trap_frame as *mut task::TrapFrame;
                                crate::kernel::percpu::set_trap_frame(trap_ptr);
                                if let Some(ref addr_space) = next.address_space {
                                    crate::kernel::percpu::set_ttbr0(addr_space.get_ttbr0());
                                }
                            }
                            // Only set flag for user tasks (not idle) - this tells IRQ handler
                            // to use user return path instead of kernel return path
                            if !crate::kernel::sched::is_idle_slot(next_slot) {
                                crate::kernel::percpu::set_syscall_switched(1);
                            }
                        }
                    }
                    Ok(())
                }
                Err(lifecycle::LifecycleError::NotFound) => Err(ProcessError::NotFound),
                Err(lifecycle::LifecycleError::PermissionDenied) => {
                    kwarn!("security", "kill_denied"; caller = killer as u64, target = target as u64);
                    Err(ProcessError::PermissionDenied)
                }
                Err(lifecycle::LifecycleError::InvalidState) => Err(ProcessError::InvalidState),
                Err(lifecycle::LifecycleError::NoChildren) => Err(ProcessError::InvalidState),
            }
        }
    }

    fn spawn(
        &self,
        parent: TaskId,
        elf_path: &str,
        caps: TraitCapabilities,
    ) -> Result<TaskId, ProcessError> {
        // Convert trait capabilities to kernel capabilities
        let kernel_caps = Capabilities::from_bits(caps.bits());

        // Try to spawn from ramfs path
        match elf::spawn_from_path_with_caps_find(elf_path, parent, kernel_caps) {
            Ok((child_id, _slot)) => Ok(child_id),
            Err(elf::ElfError::NotExecutable) => Err(ProcessError::InvalidElf),
            Err(elf::ElfError::BadMagic) => Err(ProcessError::InvalidElf),
            Err(elf::ElfError::WrongArch) => Err(ProcessError::InvalidElf),
            Err(elf::ElfError::Not64Bit) => Err(ProcessError::InvalidElf),
            Err(elf::ElfError::NotLittleEndian) => Err(ProcessError::InvalidElf),
            Err(_) => Err(ProcessError::NoSlots),
        }
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global kernel process operations backend
pub static PROCESS_OPS_BACKEND: KernelProcessOps = KernelProcessOps::new();

/// Get a reference to the global process operations backend
pub fn process_ops_backend() -> &'static dyn ProcessOps {
    &PROCESS_OPS_BACKEND
}
