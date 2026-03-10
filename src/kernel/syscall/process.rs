//! Process Management Syscalls
//!
//! This module contains syscall handlers for process lifecycle management:
//! - Process creation (exec, exec_with_mailbox)
//! - Process termination (exit, kill)
//! - Process synchronization (wait)
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

use crate::{kwarn, kdebug};
use super::super::uaccess;
use super::super::caps::Capabilities;
use crate::kernel::traits::process_ops::{WaitChildResult, SpawnSource};
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
/// Delegates to ProcessOps::exit() for state transition, parent notification,
/// and scheduling.
pub(super) fn sys_exit(code: i32) -> i64 {
    let ctx = create_syscall_context();

    let pid = match ctx.current_task_id() {
        Some(id) => id,
        None => {
            kdebug!("sys_exit", "no_current_task");
            return KernelError::NoProcess.to_errno();
        }
    };

    ctx.process().exit(pid, code);
    // Never reached — exit() -> !
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

/// Spawn a child process with a mailbox shmem page
/// Args:
///   path_ptr: Pointer to executable name in ramfs
///   path_len: Length of path
///   capabilities: Bitmask of capabilities to grant
///   mailbox_ptr: Pointer to mailbox content (up to 4KB) in parent's user memory
///   mailbox_len: Length of mailbox content
/// Returns: (parent_mailbox_handle << 32) | child_pid on success, negative error on failure
///
/// The kernel:
/// 1. Allocates a 4KB shmem page
/// 2. Copies mailbox_content from parent's user memory into shmem
/// 3. Maps shmem in child's address space
/// 4. Child gets Handle::MAILBOX at slot 5 (shmem handle)
/// 5. Parent gets a shmem handle to the same page (for post-spawn writes)
pub(super) fn sys_exec_with_mailbox(path_ptr: u64, path_len: usize, capabilities: u64, mailbox_ptr: u64, mailbox_len: usize) -> i64 {
    use crate::kernel::caps::Capabilities as KernelCaps;

    let ctx = create_syscall_context();

    // Require SPAWN + GRANT capabilities
    if let Err(_) = ctx.require_capability(Capabilities::SPAWN.bits()) {
        return KernelError::PermDenied.to_errno();
    }
    if let Err(_) = ctx.require_capability(Capabilities::GRANT.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    // Validate path
    if path_len == 0 || path_len > 127 {
        return KernelError::InvalidArg.to_errno();
    }

    // Validate mailbox
    if mailbox_len > 4096 {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy path from user space
    let mut path_buf = [0u8; 128];
    match uaccess::copy_from_user(&mut path_buf[..path_len], path_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    let path = match core::str::from_utf8(&path_buf[..path_len]) {
        Ok(s) => s,
        Err(_) => return KernelError::InvalidArg.to_errno(),
    };

    // Copy mailbox content from user space
    let mut mailbox_buf = [0u8; 4096];
    if mailbox_len > 0 {
        match uaccess::copy_from_user(&mut mailbox_buf[..mailbox_len], mailbox_ptr) {
            Ok(_) => {}
            Err(e) => return uaccess_to_errno(e),
        }
    }

    let parent_id = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    // Check per-task children limit
    let can_spawn = crate::kernel::task::with_scheduler(|sched| {
        let slot = crate::kernel::task::current_slot();
        sched.task(slot).map(|t| t.can_add_child()).unwrap_or(false)
    });

    if !can_spawn {
        return KernelError::OutOfHandles.to_errno();
    }

    let kernel_caps = KernelCaps::from_bits(capabilities);

    // Call elf directly to get both child PID and parent's mailbox handle
    match crate::kernel::elf::spawn_from_path_with_caps_and_mailbox(
        path, parent_id, kernel_caps, &mailbox_buf[..mailbox_len]
    ) {
        Ok(result) => {
            // Pack two values into 64 bits:
            //   bits  0-15: child_pid (u16, max 256 task slots)
            //   bits 16-31: shmem_handle packed (gen:8 << 8 | index:8)
            // Handle raw format: (gen << 24) | index
            // Packed format: (gen << 8) | (index & 0xFF) — fits in u16
            let shmem_packed = ((result.parent_handle_raw >> 16) & 0xFF00)
                | (result.parent_handle_raw & 0xFF);
            ((shmem_packed as i64) << 16)
                | (result.child_id as i64 & 0xFFFF)
        }
        Err(crate::kernel::elf::ElfError::NotExecutable) => KernelError::NotFound.to_errno(),
        Err(crate::kernel::elf::ElfError::BadMagic) => KernelError::InvalidArg.to_errno(),
        Err(_) => KernelError::OutOfMemory.to_errno(),
    }
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
    let mut info_buf = [ProcessInfo::empty(); 64];

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

/// Get extended process info list (with resource accounting)
/// Args: buf_ptr (pointer to ProcessInfoEx array), max_entries
/// Returns: number of entries written
pub(super) fn sys_ps_info_ex(buf_ptr: u64, max_entries: usize) -> i64 {
    if max_entries == 0 {
        return 0;
    }

    let entry_size = core::mem::size_of::<abi::ProcessInfoEx>();
    if let Err(e) = uaccess::validate_user_write(buf_ptr, entry_size * max_entries) {
        return uaccess_to_errno(e);
    }

    let ctx = create_syscall_context();

    // Use stack buffer — 64 bytes × 32 = 2KB, safe for kernel stack
    let actual_max = max_entries.min(32);
    let mut info_buf = [abi::ProcessInfoEx::empty(); 32];

    let count = ctx.process().list_processes_ex(&mut info_buf[..actual_max], actual_max);

    // Copy entries to userspace
    for i in 0..count {
        let offset = (i * entry_size) as u64;
        let info_bytes = unsafe {
            core::slice::from_raw_parts(
                &info_buf[i] as *const abi::ProcessInfoEx as *const u8,
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

/// Send an async signal to another task
/// Args: target_pid, event (signal_event::*), value
/// Returns: 0 on success, negative error on failure
pub(super) fn sys_signal(target_pid: u32, event: u32, value: u64) -> i64 {
    use crate::kernel::task;
    use crate::kernel::microtask;

    let caller_pid = super::current_pid();

    // Validate target exists and is allowed to receive signals from caller
    let allowed = task::with_scheduler(|sched| {
        if let Some(slot) = sched.slot_by_pid(target_pid) {
            if let Some(target) = sched.task(slot) {
                if target.is_terminated() {
                    return false;
                }
                return target.can_receive_signal_from(caller_pid);
            }
        }
        false
    });

    if !allowed {
        return KernelError::PermDenied.to_errno();
    }

    // Enqueue signal delivery via microtask (deferred, safe from any lock context)
    match microtask::enqueue(crate::kernel::microtask::MicroTask::Signal {
        target: target_pid,
        event,
        value,
    }) {
        Ok(()) => 0,
        Err(_) => KernelError::OutOfMemory.to_errno(),
    }
}

/// Peek at pending signals for a target process (non-consuming read)
/// Args: target_pid, buf_ptr (pointer to PendingSignal array), max_count
/// Returns: number of signals copied, or negative error
pub(super) fn sys_signal_peek(target_pid: u32, buf_ptr: u64, max_count: usize) -> i64 {
    use crate::kernel::task;

    if max_count == 0 {
        return 0;
    }

    let caller_pid = super::current_pid();

    // Validate output buffer
    let entry_size = core::mem::size_of::<abi::PendingSignal>();
    if let Err(e) = uaccess::validate_user_write(buf_ptr, entry_size * max_count) {
        return uaccess_to_errno(e);
    }

    // Copy signals to stack buffer under scheduler lock, then copy_to_user after release
    const MAX_STACK_SIGNALS: usize = 16;
    let mut sig_buf = [abi::PendingSignal { event: 0, value: 0 }; MAX_STACK_SIGNALS];
    let actual_max = max_count.min(MAX_STACK_SIGNALS);

    let result: Result<usize, i64> = task::with_scheduler(|sched| {
        let slot = sched.slot_by_pid(target_pid).ok_or(KernelError::NoProcess.to_errno())?;
        let target = sched.task(slot).ok_or(KernelError::NoProcess.to_errno())?;

        if target.is_terminated() {
            return Err(KernelError::NoProcess.to_errno());
        }

        // Permission: caller is parent, or caller==target, or caller has CAP_ADMIN
        let allowed = caller_pid == target_pid
            || target.parent_id == caller_pid
            || target.can_receive_signal_from(caller_pid);
        if !allowed {
            return Err(KernelError::PermDenied.to_errno());
        }

        // Copy to stack buffer under lock
        let count = (target.signal_count as usize).min(actual_max);
        for i in 0..count {
            let idx = ((target.signal_tail as usize) + i) % crate::kernel::task::tcb::MAX_PENDING_SIGNALS;
            sig_buf[i] = target.signal_queue[idx];
        }

        Ok(count)
    });

    // Copy to userspace outside scheduler lock
    match result {
        Ok(count) => {
            for i in 0..count {
                let offset = (i * entry_size) as u64;
                let sig_bytes = unsafe {
                    core::slice::from_raw_parts(
                        &sig_buf[i] as *const abi::PendingSignal as *const u8,
                        entry_size,
                    )
                };
                if uaccess::copy_to_user(buf_ptr + offset, sig_bytes).is_err() {
                    return i as i64;
                }
            }
            count as i64
        }
        Err(e) => e,
    }
}

/// Flush (clear) all pending signals for a target process
/// Args: target_pid
/// Returns: number of signals flushed, or negative error
pub(super) fn sys_signal_flush(target_pid: u32) -> i64 {
    use crate::kernel::task;

    let caller_pid = super::current_pid();

    task::with_scheduler(|sched| {
        let slot = match sched.slot_by_pid(target_pid) {
            Some(s) => s,
            None => return KernelError::NoProcess.to_errno(),
        };
        let target = match sched.task_mut(slot) {
            Some(t) => t,
            None => return KernelError::NoProcess.to_errno(),
        };

        if target.is_terminated() {
            return KernelError::NoProcess.to_errno();
        }

        // Permission: caller is parent, or caller==target, or caller has CAP_ADMIN
        let allowed = caller_pid == target_pid
            || target.parent_id == caller_pid
            || target.can_receive_signal_from(caller_pid);
        if !allowed {
            return KernelError::PermDenied.to_errno();
        }

        let flushed = target.signal_count as i64;
        target.signal_head = 0;
        target.signal_tail = 0;
        target.signal_count = 0;
        flushed
    })
}

/// Reset per-task statistics counters
/// Args: target_pid (0 = all tasks)
/// Returns: 0 on success, negative error
pub(super) fn sys_reset_stats(target_pid: u32) -> i64 {
    use crate::kernel::task;

    let caller_pid = super::current_pid();

    task::with_scheduler(|sched| {
        if target_pid == 0 {
            // Reset all — only allowed for tasks with CAP_ADMIN or as a convenience for any caller
            // (ps -r is a diagnostic tool, not a security boundary)
            for (_slot, task_opt) in sched.iter_tasks_mut() {
                if let Some(t) = task_opt {
                    t.reset_stats();
                }
            }
            return 0;
        }

        let slot = match sched.slot_by_pid(target_pid) {
            Some(s) => s,
            None => return KernelError::NoProcess.to_errno(),
        };
        let target = match sched.task_mut(slot) {
            Some(t) => t,
            None => return KernelError::NoProcess.to_errno(),
        };

        // Permission: caller is parent, caller==target
        if caller_pid != target_pid && target.parent_id != caller_pid {
            return KernelError::PermDenied.to_errno();
        }

        target.reset_stats();
        0
    })
}
