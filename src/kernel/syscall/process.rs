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
//! 2. Call ProcessOps trait methods for state transitions
//! 3. Handle scheduling if task state changed
//!
//! Core lifecycle logic is in `kernel::process_ops_impl`.
//!
//! SECURITY: All user pointers are validated before access using the uaccess module.

use crate::{kwarn, kinfo};
use super::super::uaccess;
use super::super::caps::Capabilities;
use crate::kernel::traits::process_ops::WaitChildResult;
use super::super::syscall_ctx_impl::create_syscall_context;
use super::super::traits::syscall_ctx::SyscallContext;
use super::super::traits::process_ops::SpawnSource;
use super::uaccess_to_errno;
use crate::kernel::error::KernelError;

/// Wait flags
pub const WNOHANG: u32 = 1;  // Don't block if no child has exited

// Re-export ProcessInfo from abi crate (single source of truth)
pub use abi::ProcessInfo;

/// Exit current process
///
/// Delegates to ProcessOps::exit() for state transition, parent notification,
/// and scheduling.
pub(super) fn sys_exit(code: i32) -> i64 {
    let ctx = create_syscall_context();

    let pid = match ctx.current_task_id() {
        Some(id) => id,
        None => {
            kinfo!("sys_exit", "no_current_task");
            return KernelError::NoProcess.to_errno();
        }
    };

    ctx.process().exit(pid, code);
    // Never reached — exit() -> !
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

    // Parse name from kernel buffer
    let name = core::str::from_utf8(&name_buf[..name_len]).unwrap_or("child");

    // Get current PID as parent
    let parent_id = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    match ctx.process().spawn(parent_id, SpawnSource::ElfId(elf_id, name)) {
        Ok(child_id) => child_id as i64,
        Err(e) => e.to_errno(),
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

    match ctx.process().spawn(parent_id, SpawnSource::Path(path)) {
        Ok(child_id) => child_id as i64,
        Err(e) => e.to_errno(),
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

    // Convert capabilities bitmask to trait type
    let trait_caps = crate::kernel::traits::task::Capabilities(capabilities);

    match ctx.process().spawn(parent_id, SpawnSource::PathWithCaps(path, trait_caps)) {
        Ok(child_id) => child_id as i64,
        Err(e) => e.to_errno(),
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

    // Spawn via ProcessOps trait
    let result = match ctx.process().spawn(parent_id, SpawnSource::Memory(elf_slice, name)) {
        Ok(child_id) => child_id as i64,
        Err(e) => e.to_errno(),
    };

    // Free the kernel buffer (ELF data has been copied to process address space)
    super::super::pmm::free_pages(elf_phys, num_pages);

    result
}

/// Wait for a child process to exit
///
/// Delegates to ProcessOps::wait_child() for the blocking retry loop.
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

    match ctx.process().wait_child(caller_pid, pid, no_hang) {
        WaitChildResult::Exited { pid: child_pid, code } => {
            if status_ptr != 0 {
                if let Err(e) = uaccess::put_user::<i32>(status_ptr, code) {
                    return uaccess_to_errno(e);
                }
            }
            ((child_pid as i64) << 32) | ((code as i64) & 0xFFFFFFFF)
        }
        WaitChildResult::WouldBlock => 0,
        WaitChildResult::NoChildren => KernelError::NoChild.to_errno(),
        WaitChildResult::NotChild => KernelError::InvalidArg.to_errno(),
    }
}

/// Daemonize - detach from parent and become a daemon
///
/// Delegates to ProcessOps::daemonize() for parent-child detachment.
pub(super) fn sys_daemonize() -> i64 {
    let ctx = create_syscall_context();
    let caller_pid = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    match ctx.process().daemonize(caller_pid) {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// Kill a process by PID
///
/// Delegates to ProcessOps::kill() for state transition and parent notification.
///
/// SECURITY: Only allows killing:
/// - The caller itself (suicide)
/// - Children of the caller
/// - Processes in caller's signal allowlist
/// - Processes with CAP_KILL capability can kill any process except init (PID 1)
pub(super) fn sys_kill(pid: u32) -> i64 {
    let ctx = create_syscall_context();

    // PID 0 is reserved for idle tasks - never allow killing them
    if pid == 0 {
        let caller = ctx.current_task_id().unwrap_or(0);
        kwarn!("security", "kill_idle_denied"; caller = caller as u64);
        return KernelError::PermDenied.to_errno();
    }

    // NOTE: Killing devd (PID 1, is_init=true) is allowed for testing - recovery will be triggered.
    // The lifecycle::kill() function sets DEVD_LIVENESS_KILLED flag which causes
    // the timer handler to respawn devd and kill all other tasks.

    let caller_pid = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    match ctx.process().kill(caller_pid, pid) {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
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

    let entry_size = core::mem::size_of::<ProcessInfo>();
    if let Err(e) = uaccess::validate_user_write(buf_ptr, entry_size * max_entries) {
        return uaccess_to_errno(e);
    }

    let ctx = create_syscall_context();

    // Use stack buffer to collect entries via trait
    // Limit to 64 to avoid excessive stack usage
    let actual_max = max_entries.min(64);
    let mut info_buf = [ProcessInfo {
        pid: 0, ppid: 0, state: 0, liveness_status: 0, cpu: 0, _pad: 0,
        activity_age_ms: 0, name: [0u8; 16],
    }; 64];

    let count = ctx.process().list_processes(&mut info_buf[..actual_max], actual_max);

    // Copy entries to userspace
    for i in 0..count {
        let offset = (i * entry_size) as u64;
        let info_bytes = unsafe {
            core::slice::from_raw_parts(
                &info_buf[i] as *const ProcessInfo as *const u8,
                entry_size,
            )
        };
        if uaccess::copy_to_user(buf_ptr + offset, info_bytes).is_err() {
            return i as i64;
        }
    }

    count as i64
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

    match ctx.process().get_capabilities(target_pid) {
        Ok(caps) => caps as i64,
        Err(e) => e.to_errno(),
    }
}
