//! Process Management Syscalls
//!
//! This module contains syscall handlers for process lifecycle management:
//! - Process creation (spawn, exec, exec_with_caps, exec_mem)
//! - Process termination (exit, kill)
//! - Process synchronization (wait, daemonize)
//! - Process information (ps_info, get_capabilities)
//!
//! # Design
//!
//! Syscalls are thin wrappers that:
//! 1. Validate user input (pointers, capabilities)
//! 2. Call lifecycle functions for state transitions
//! 3. Handle scheduling if task state changed
//!
//! Core lifecycle logic is in `kernel::task::lifecycle`.
//!
//! SECURITY: All user pointers are validated before access using the uaccess module.

use crate::{kwarn, kinfo, print_direct};
use super::super::uaccess;
use super::super::caps::Capabilities;
use super::super::task::{self, lifecycle};
use super::super::syscall_ctx_impl::create_syscall_context;
use super::super::traits::syscall_ctx::SyscallContext;
use super::uaccess_to_errno;
use crate::kernel::error::KernelError;

/// Wait flags
pub const WNOHANG: u32 = 1;  // Don't block if no child has exited

// Re-export ProcessInfo from abi crate (single source of truth)
pub use abi::ProcessInfo;

/// Exit current process
///
/// Delegates to lifecycle::exit() for state transition and parent notification,
/// then handles scheduling.
pub(super) fn sys_exit(code: i32) -> i64 {
    let ctx = create_syscall_context();

    // Flush UART buffer so pending output from this process appears first
    while crate::platform::current::uart::has_buffered_output() {
        crate::platform::current::uart::flush_buffer();
    }

    print_direct!("\n========================================\n");
    print_direct!("  Process exited with code: {}\n", code);
    print_direct!("========================================\n");

    let pid = match ctx.current_task_id() {
        Some(id) => id,
        None => {
            kinfo!("sys_exit", "no_current_task");
            return KernelError::NoProcess.to_errno();
        }
    };

    task::with_scheduler(|sched| {
        let current_slot = task::current_slot();

        // Delegate state transition and parent notification to lifecycle module
        // NOTE: Resource cleanup is deferred to reap_terminated() for two-phase cleanup.
        if let Err(e) = lifecycle::exit(sched, pid, code) {
            kinfo!("sys_exit", "lifecycle_error"; err = e as i64);
        }

        // Debug: show task states before scheduling
        print_direct!("  Task states at exit (current_slot={}):\n", current_slot);
        for (i, task_opt) in sched.iter_tasks() {
            if let Some(task) = task_opt {
                print_direct!("    [{}] {} - {}\n", i, task.name_str(), task.state().name());
            }
        }

        // Schedule next task
        if let Some(next_slot) = sched.schedule() {
            // Internal scheduling detail, don't log

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
            0
        } else {
            // No more tasks - halt
            kinfo!("sys_exit", "halt_no_tasks");
            loop {
                unsafe { core::arch::asm!("wfi"); }
            }
        }
    })
}

/// Spawn a new process from a built-in ELF binary
/// Args: elf_id (built-in ELF identifier), name_ptr, name_len
/// Returns: child PID on success, negative error on failure
/// SECURITY: Copies user name buffer via page table translation
pub(super) fn sys_spawn(elf_id: u32, name_ptr: u64, name_len: usize) -> i64 {
    let ctx = create_syscall_context();

    // Check SPAWN capability
    if let Err(_) = ctx.require_capability(Capabilities::SPAWN.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    // Validate name length
    if name_len == 0 || name_len > 15 {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy name from user space
    let mut name_buf = [0u8; 16];
    match uaccess::copy_from_user(&mut name_buf[..name_len], name_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Get the ELF binary by ID
    let elf_data = match super::super::elf::get_elf_by_id(elf_id) {
        Some(data) => data,
        None => return KernelError::NotFound.to_errno(),
    };

    // Parse name from kernel buffer
    let name = core::str::from_utf8(&name_buf[..name_len]).unwrap_or("child");

    // Get current PID as parent
    let parent_id = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    // Spawn the child process
    match super::super::elf::spawn_from_elf_with_parent(elf_data, name, parent_id) {
        Ok((child_id, _slot)) => child_id as i64,
        Err(_) => KernelError::OutOfMemory.to_errno(),
    }
}

/// Execute a program from a path (searches ramfs)
/// Args: path_ptr, path_len
/// Returns: child PID on success, negative error on failure
/// SECURITY: Copies user path buffer via page table translation
pub(super) fn sys_exec(path_ptr: u64, path_len: usize) -> i64 {
    let ctx = create_syscall_context();

    // Check SPAWN capability
    if let Err(_) = ctx.require_capability(Capabilities::SPAWN.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    // Validate path length
    if path_len == 0 || path_len > 127 {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy path from user space
    let mut path_buf = [0u8; 128];
    match uaccess::copy_from_user(&mut path_buf[..path_len], path_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Parse path from kernel buffer
    let path = match core::str::from_utf8(&path_buf[..path_len]) {
        Ok(s) => s,
        Err(_) => return KernelError::InvalidArg.to_errno(),
    };

    // Get current PID as parent
    let parent_id = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    // Try to spawn from ramfs path
    match super::super::elf::spawn_from_path_with_parent(path, parent_id) {
        Ok((child_id, _slot)) => child_id as i64,
        Err(super::super::elf::ElfError::NotExecutable) => KernelError::NotFound.to_errno(),
        Err(_) => KernelError::OutOfMemory.to_errno(),
    }
}

/// Execute a program with explicit capability grant (privilege separation)
/// Args: path_ptr, path_len, capabilities (bitmask)
/// Returns: child PID on success, negative error on failure
/// SECURITY: Child capabilities are intersected with parent's - can't escalate
pub(super) fn sys_exec_with_caps(path_ptr: u64, path_len: usize, capabilities: u64) -> i64 {
    let ctx = create_syscall_context();

    // Require SPAWN capability
    if let Err(_) = ctx.require_capability(Capabilities::SPAWN.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    // Require GRANT capability to delegate capabilities to children
    if let Err(_) = ctx.require_capability(Capabilities::GRANT.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    // Validate path length
    if path_len == 0 || path_len > 127 {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy path from user space
    let mut path_buf = [0u8; 128];
    match uaccess::copy_from_user(&mut path_buf[..path_len], path_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Parse path from kernel buffer
    let path = match core::str::from_utf8(&path_buf[..path_len]) {
        Ok(s) => s,
        Err(_) => return KernelError::InvalidArg.to_errno(),
    };

    // Get current PID as parent
    let parent_id = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    // Convert capabilities bitmask
    let requested_caps = Capabilities::from_bits(capabilities);

    // Try to spawn from ramfs path with explicit capabilities
    match super::super::elf::spawn_from_path_with_caps_find(path, parent_id, requested_caps) {
        Ok((child_id, _slot)) => child_id as i64,
        Err(super::super::elf::ElfError::NotExecutable) => KernelError::NotFound.to_errno(),
        Err(_) => KernelError::OutOfMemory.to_errno(),
    }
}

/// Execute ELF binary from a memory buffer
/// Args:
///   elf_ptr: Pointer to ELF data in userspace
///   elf_len: Size of ELF data in bytes
///   name_ptr: Pointer to process name (or 0 for default)
///   name_len: Length of process name
/// Returns: PID of new process on success, negative error on failure
/// SECURITY: Reads ELF data via page table translation
pub(super) fn sys_exec_mem(elf_ptr: u64, elf_len: usize, name_ptr: u64, name_len: usize) -> i64 {
    let ctx = create_syscall_context();

    // Check SPAWN capability
    if let Err(_) = ctx.require_capability(Capabilities::SPAWN.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    // Validate ELF size (reasonable bounds: at least an ELF header, at most 16MB)
    if elf_len < 64 || elf_len > 16 * 1024 * 1024 {
        return KernelError::InvalidArg.to_errno();
    }

    // Validate and read ELF data from userspace
    // We need to copy to kernel memory since the ELF loader works with slices
    if let Err(e) = uaccess::validate_user_read(elf_ptr, elf_len) {
        return uaccess_to_errno(e);
    }

    // Allocate kernel buffer for ELF data
    // Use the PMM for larger allocations
    let num_pages = (elf_len + 4095) / 4096;
    let elf_phys = match super::super::pmm::alloc_pages(num_pages) {
        Some(addr) => addr,
        None => return KernelError::OutOfMemory.to_errno(),
    };

    // Get kernel virtual address for the allocation
    let elf_virt = crate::arch::aarch64::mmu::phys_to_virt(elf_phys as u64) as *mut u8;

    // Copy ELF data from userspace to kernel buffer
    let elf_slice = unsafe { core::slice::from_raw_parts_mut(elf_virt, elf_len) };
    if let Err(e) = uaccess::copy_from_user(elf_slice, elf_ptr) {
        super::super::pmm::free_pages(elf_phys, num_pages);
        return uaccess_to_errno(e);
    }

    // Parse process name
    let mut name_buf = [0u8; 32];
    let name = if name_ptr != 0 && name_len > 0 {
        let actual_len = name_len.min(31);
        if let Err(e) = uaccess::copy_from_user(&mut name_buf[..actual_len], name_ptr) {
            super::super::pmm::free_pages(elf_phys, num_pages);
            return uaccess_to_errno(e);
        }
        core::str::from_utf8(&name_buf[..actual_len]).unwrap_or("exec_mem")
    } else {
        "exec_mem"
    };

    // Get current PID as parent
    let parent_id = match ctx.current_task_id() {
        Some(id) => id,
        None => {
            super::super::pmm::free_pages(elf_phys, num_pages);
            return KernelError::NoProcess.to_errno();
        }
    };

    // Spawn from the ELF data
    let result = match super::super::elf::spawn_from_elf_with_parent(elf_slice, name, parent_id) {
        Ok((child_id, _slot)) => child_id as i64,
        Err(super::super::elf::ElfError::BadMagic) => KernelError::InvalidArg.to_errno(),
        Err(super::super::elf::ElfError::NotExecutable) => KernelError::InvalidArg.to_errno(),
        Err(super::super::elf::ElfError::WrongArch) => KernelError::InvalidArg.to_errno(),
        Err(_) => KernelError::OutOfMemory.to_errno(),
    };

    // Free the kernel buffer (ELF data has been copied to process address space)
    super::super::pmm::free_pages(elf_phys, num_pages);

    result
}

/// Wait for a child process to exit
///
/// Delegates to lifecycle::wait_child() for child lookup and reaping,
/// then handles user pointer write and blocking if needed.
///
/// Args: pid (-1 = any child, >0 = specific child), status_ptr (pointer to i32 for exit status), flags
/// flags: 0 = block until child exits, WNOHANG (1) = return immediately if no child exited
/// Returns: PID of exited child on success, 0 if WNOHANG and no child exited, negative error
/// SECURITY: Writes to user status pointer via page table translation
pub(super) fn sys_wait(pid: i32, status_ptr: u64, flags: u32) -> i64 {
    let ctx = create_syscall_context();
    let caller_pid = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };
    let no_hang = (flags & WNOHANG) != 0;

    // Validate status pointer if provided
    if status_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(status_ptr, core::mem::size_of::<i32>()) {
            return uaccess_to_errno(e);
        }
    }

    task::with_scheduler(|sched| {
        let caller_slot = task::current_slot();

        // Delegate to lifecycle module for child lookup and reaping
        match lifecycle::wait_child(sched, caller_pid, pid, no_hang) {
            lifecycle::WaitResult::Exited { pid: child_pid, code } => {
                // Write exit status to user space if pointer provided
                if status_ptr != 0 {
                    if let Err(e) = uaccess::put_user::<i32>(status_ptr, code) {
                        return uaccess_to_errno(e);
                    }
                }
                // Return (pid << 32 | exit_code) as documented
                ((child_pid as i64) << 32) | ((code as i64) & 0xFFFFFFFF)
            }

            lifecycle::WaitResult::WouldBlock => {
                // If WNOHANG, return 0 without blocking
                if no_hang {
                    return 0;
                }

                // Block caller waiting for child exit (via state machine)
                if let Err(_e) = lifecycle::sleep_current(
                    sched,
                    caller_slot,
                    task::SleepReason::EventLoop,
                ) {
                    return KernelError::InvalidArg.to_errno();
                }

                // Pre-store return value in caller's trap frame
                if let Some(parent) = sched.task_mut(caller_slot) {
                    parent.set_deferred_return(KernelError::WouldBlock.to_errno());
                }

                // Reschedule to another task
                super::super::sched::reschedule();

                KernelError::WouldBlock.to_errno()
            }

            lifecycle::WaitResult::NoChildren => KernelError::NoChild.to_errno(),
            lifecycle::WaitResult::NotChild => KernelError::InvalidArg.to_errno(),
        }
    })
}

/// Daemonize - detach from parent and become a daemon
pub(super) fn sys_daemonize() -> i64 {
    let ctx = create_syscall_context();
    let caller_pid = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    task::with_scheduler(|sched| {
        // Find the calling task and detach from parent
        let parent_id = if let Some(slot) = sched.slot_by_pid(caller_pid) {
            if let Some(task) = sched.task_mut(slot) {
                task.detach_from_parent()
            } else {
                0
            }
        } else {
            0
        };

        // Remove from parent's children list and wake parent if blocked
        if parent_id != 0 {
            if let Some(parent_slot) = sched.slot_by_pid(parent_id) {
                if let Some(parent) = sched.task_mut(parent_slot) {
                    parent.remove_child(caller_pid);
                }
                // Wake up parent if it's blocked using unified wake function
                sched.wake_task(parent_slot);
            }
        }
    });

    0  // Success
}

/// Kill a process by PID
///
/// Delegates to lifecycle::kill() for state transition and parent notification,
/// then handles scheduling if killing self.
///
/// SECURITY: Only allows killing:
/// - The caller itself (suicide)
/// - Children of the caller
/// - Processes in caller's signal allowlist
/// - Processes with CAP_KILL capability can kill any process except init (PID 1)
pub(super) fn sys_kill(pid: u32) -> i64 {
    let ctx = create_syscall_context();

    if pid == 0 {
        return KernelError::InvalidArg.to_errno();
    }

    // SECURITY: Never allow killing idle (slot 0, pid 1)
    if pid == 1 {
        let caller = ctx.current_task_id().unwrap_or(0);
        kwarn!("security", "kill_idle_denied"; caller = caller as u64);
        return KernelError::PermDenied.to_errno();
    }

    // NOTE: Killing devd (is_init=true) is allowed for testing - recovery will be triggered.
    // The lifecycle::kill() function sets DEVD_LIVENESS_KILLED flag which causes
    // the timer handler to respawn devd and kill all other tasks.
    // TODO: For production, consider requiring CAP_KILL capability for killing init.

    let caller_pid = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    task::with_scheduler(|sched| {
        // Delegate to lifecycle module for permission check, state transition, and notification
        // NOTE: Resource cleanup is deferred to reap_terminated() for two-phase cleanup.
        match lifecycle::kill(sched, pid, caller_pid) {
            Ok(()) => {}
            Err(lifecycle::LifecycleError::NotFound) => return KernelError::NoProcess.to_errno(),
            Err(lifecycle::LifecycleError::PermissionDenied) => {
                kwarn!("security", "kill_denied"; caller = caller_pid as u64, target = pid as u64);
                return KernelError::PermDenied.to_errno();
            }
            Err(lifecycle::LifecycleError::InvalidState) => return KernelError::InvalidArg.to_errno(),
            Err(lifecycle::LifecycleError::NoChildren) => return KernelError::InvalidArg.to_errno(),
        }

        // If killing current task, the reschedule will be handled by do_resched_if_needed
        // after this syscall returns. The current task is now in Exiting state, which
        // will cause should_resched = true (is_terminated check in do_resched_if_needed).
        //
        // IMPORTANT: We must NOT do manual task switching here because:
        // 1. The target task might need needs_context_restore (was context-switched out)
        // 2. Proper context_switch requires releasing the scheduler lock first
        // 3. do_resched_if_needed and reschedule() handle this correctly
        //
        // The flow after self-kill:
        // 1. sys_kill returns 0
        // 2. svc_handler sees SYSCALL_SWITCHED_TASK = 0 (we didn't set it)
        // 3. svc_handler stores return value to current trap frame (dying task's)
        // 4. svc_handler calls do_resched_if_needed
        // 5. do_resched_if_needed sees current task is terminated -> should_resched = true
        // 6. reschedule() properly handles context switch to next task

        0  // Success
    })
}

/// Get process info list
/// Args: buf_ptr (pointer to ProcessInfo array), max_entries
/// Returns: number of entries written
///
/// NOTE: This is a temporary syscall - pure microkernel will move this to init service.
pub(super) fn sys_ps_info(buf_ptr: u64, max_entries: usize) -> i64 {
    if max_entries == 0 {
        return 0;
    }

    // Validate user pointer
    let entry_size = core::mem::size_of::<ProcessInfo>();
    if let Err(e) = uaccess::validate_user_write(buf_ptr, entry_size * max_entries) {
        return uaccess_to_errno(e);
    }

    task::with_scheduler(|sched| {
        let mut count = 0usize;

        for (slot, task_opt) in sched.iter_tasks() {
            // Skip idle tasks (slots 0..MAX_CPUS) - internal kernel tasks
            if crate::kernel::sched::is_idle_slot(slot) {
                continue;
            }

            if count >= max_entries {
                break;
            }

            if let Some(task) = task_opt {
                // Calculate activity age and liveness status via encapsulated accessors
                let current_tick = crate::platform::current::timer::ticks();

                let info = ProcessInfo {
                    pid: task.id,
                    ppid: task.parent_id,
                    state: task.state().state_code(),
                    liveness_status: task.get_liveness_status_code(),
                    cpu: sched.task_cpu(slot).map_or(0xFF, |c| c as u8),
                    _pad: 0,
                    activity_age_ms: task.get_activity_age_ms(current_tick),
                    name: task.name,
                };

                // Write to user buffer
                let offset = (count * entry_size) as u64;
                let info_bytes = unsafe {
                    core::slice::from_raw_parts(
                        &info as *const ProcessInfo as *const u8,
                        entry_size
                    )
                };

                if let Err(_) = uaccess::copy_to_user(buf_ptr + offset, info_bytes) {
                    break;
                }

                count += 1;
            }
        }

        count as i64
    })
}

/// Get capabilities of a process
/// Args: pid (0 = current process)
/// Returns: capability bits as i64 on success, negative error on failure
///
/// This allows protocol handlers to verify caller permissions before
/// executing privileged operations. Any process can query any other
/// process's capabilities (they are not secret).
pub(super) fn sys_get_capabilities(pid: u32) -> i64 {
    let ctx = create_syscall_context();
    let target_pid = if pid == 0 {
        match ctx.current_task_id() {
            Some(id) => id,
            None => return KernelError::NoProcess.to_errno(),
        }
    } else {
        pid
    };

    task::with_scheduler(|sched| {
        // Find the task by PID
        if let Some(slot) = sched.slot_by_pid(target_pid) {
            if let Some(task) = sched.task(slot) {
                return task.get_capabilities_bits() as i64;
            }
        }

        // Process not found
        KernelError::NoProcess.to_errno()
    })
}
