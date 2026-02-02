//! Memory-related system calls
//!
//! - `sys_mmap` - Map anonymous memory pages into user address space
//! - `sys_munmap` - Unmap memory pages from user address space
//!
//! DMA and device MMIO mapping use the unified object interface:
//! `open(ObjectType::DmaPool, size) + map(handle)` and
//! `open(ObjectType::Mmio, phys_addr) + map(handle)`.

use super::super::uaccess;
use super::super::syscall_ctx_impl::create_syscall_context;
use super::super::traits::syscall_ctx::SyscallContext;
use crate::kernel::error::KernelError;

/// Map memory pages into user address space
/// Args: addr (hint, ignored for now), size, prot
/// Returns: virtual address on success, negative error on failure
pub(super) fn sys_mmap(_addr: u64, size: usize, prot: u32) -> i64 {
    if size == 0 {
        return KernelError::InvalidArg.to_errno();
    }

    // Parse protection flags (simplified: bit 0 = write, bit 1 = exec)
    let writable = (prot & 0x2) != 0;  // PROT_WRITE
    let executable = (prot & 0x4) != 0; // PROT_EXEC

    // Use trait-based context
    let ctx = create_syscall_context();
    let task_id = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::InvalidArg.to_errno(),
    };

    match ctx.memory().mmap(task_id, size, writable, executable) {
        Ok(vaddr) => vaddr as i64,
        Err(_) => KernelError::OutOfMemory.to_errno(),
    }
}

/// Unmap memory pages from user address space
/// Args: addr, size
/// Returns: 0 on success, negative error on failure
pub(super) fn sys_munmap(addr: u64, size: usize) -> i64 {
    if size == 0 {
        return KernelError::InvalidArg.to_errno();
    }

    // Validate that the address is in user space (not kernel)
    if !uaccess::is_user_address(addr) {
        return KernelError::BadAddress.to_errno();
    }

    // Use trait-based context
    let ctx = create_syscall_context();
    let task_id = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::InvalidArg.to_errno(),
    };

    match ctx.memory().munmap(task_id, addr, size) {
        Ok(()) => 0i64,
        Err(_) => KernelError::InvalidArg.to_errno(),
    }
}

// Legacy syscalls removed:
// - sys_mmap_dma (36) → use open(DmaPool) + map()
// - sys_mmap_device (60) → use open(Mmio) + map()
// - sys_dma_pool_create (61) → use open(DmaPool) + map()
// - sys_dma_pool_create_high (62) → use open(DmaPool, HIGH) + map()
