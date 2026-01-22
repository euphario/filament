//! Shared Memory Syscall Handlers
//!
//! This module implements the syscall handlers for shared memory operations:
//! - `sys_shmem_create`: Create a new shared memory region
//! - `sys_shmem_map`: Map an existing shared memory region into caller's address space
//! - `sys_shmem_allow`: Grant another process permission to map a shared memory region
//! - `sys_shmem_wait`: Wait for a notification on a shared memory region
//! - `sys_shmem_notify`: Wake up processes waiting on a shared memory region
//! - `sys_shmem_destroy`: Destroy a shared memory region (owner only)
//! - `sys_shmem_unmap`: Unmap a shared memory region from caller's address space
//!
//! Shared memory provides zero-copy IPC between processes. The owner process
//! creates the region and can grant access to peer processes via `shmem_allow`.

use super::super::shmem;
use super::super::task;
use super::super::uaccess;
use super::{current_pid, uaccess_to_errno, SyscallError};

/// Create a new shared memory region
/// Args: size, vaddr_ptr, paddr_ptr
/// Returns: shmem_id on success, negative error on failure
pub(super) fn sys_shmem_create(size: usize, vaddr_ptr: u64, paddr_ptr: u64) -> i64 {
    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate output pointers
    if vaddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(vaddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }
    if paddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(paddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }

    let caller_pid = current_pid();

    match shmem::create(caller_pid, size) {
        Ok((shmem_id, vaddr, paddr)) => {
            // Write output values
            if vaddr_ptr != 0 {
                if let Err(e) = uaccess::put_user::<u64>(vaddr_ptr, vaddr) {
                    return uaccess_to_errno(e);
                }
            }
            if paddr_ptr != 0 {
                if let Err(e) = uaccess::put_user::<u64>(paddr_ptr, paddr) {
                    return uaccess_to_errno(e);
                }
            }
            shmem_id as i64
        }
        Err(e) => e,
    }
}

/// Map an existing shared memory region
/// Args: shmem_id, vaddr_ptr, paddr_ptr
/// Returns: 0 on success, negative error on failure
pub(super) fn sys_shmem_map(shmem_id: u32, vaddr_ptr: u64, paddr_ptr: u64) -> i64 {
    // Validate output pointers
    if vaddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(vaddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }
    if paddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(paddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }

    let caller_pid = current_pid();

    match shmem::map(caller_pid, shmem_id) {
        Ok((vaddr, paddr)) => {
            // Write output values
            if vaddr_ptr != 0 {
                if let Err(e) = uaccess::put_user::<u64>(vaddr_ptr, vaddr) {
                    return uaccess_to_errno(e);
                }
            }
            if paddr_ptr != 0 {
                if let Err(e) = uaccess::put_user::<u64>(paddr_ptr, paddr) {
                    return uaccess_to_errno(e);
                }
            }
            0
        }
        Err(e) => e,
    }
}

/// Allow another process to map shared memory
/// Args: shmem_id, peer_pid
/// Returns: 0 on success, negative error on failure
pub(super) fn sys_shmem_allow(shmem_id: u32, peer_pid: u32) -> i64 {
    let caller_pid = current_pid();

    match shmem::allow(caller_pid, shmem_id, peer_pid) {
        Ok(()) => 0,
        Err(e) => e,
    }
}

/// Wait for shared memory notification
/// Args: shmem_id, timeout_ms
/// Returns: 0 on notify, negative error on failure/timeout
pub(super) fn sys_shmem_wait(shmem_id: u32, timeout_ms: u32) -> i64 {
    let caller_pid = current_pid();

    match shmem::wait(caller_pid, shmem_id, timeout_ms) {
        Ok(()) => 0,
        Err(-11) => {
            // EAGAIN = blocked, pre-store success return value
            // When woken by notify, task will resume and see 0
            unsafe {
                let sched = task::scheduler();
                if let Some(task) = sched.current_task_mut() {
                    task.trap_frame.x0 = 0;
                }
            }
            SyscallError::WouldBlock as i64
        }
        Err(e) => e,
    }
}

/// Notify shared memory waiters
/// Args: shmem_id
/// Returns: number of waiters woken, or negative error
pub(super) fn sys_shmem_notify(shmem_id: u32) -> i64 {
    let caller_pid = current_pid();

    match shmem::notify(caller_pid, shmem_id) {
        Ok(woken) => woken as i64,
        Err(e) => e,
    }
}

/// Destroy a shared memory region
/// Args: shmem_id
/// Returns: 0 on success, negative error on failure
pub(super) fn sys_shmem_destroy(shmem_id: u32) -> i64 {
    let caller_pid = current_pid();

    match shmem::destroy(caller_pid, shmem_id) {
        Ok(()) => 0,
        Err(e) => e,
    }
}

/// Unmap a shared memory region from this process
/// Args: shmem_id
/// Returns: 0 on success, negative error on failure
pub(super) fn sys_shmem_unmap(shmem_id: u32) -> i64 {
    let caller_pid = current_pid();

    match shmem::unmap(caller_pid, shmem_id) {
        Ok(()) => 0,
        Err(e) => e,
    }
}
