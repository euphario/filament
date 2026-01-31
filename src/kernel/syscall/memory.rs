//! Memory-related system calls
//!
//! This module contains syscall handlers for memory management operations:
//! - `sys_mmap` - Map memory pages into user address space
//! - `sys_mmap_dma` - Allocate DMA-capable memory
//! - `sys_munmap` - Unmap memory pages from user address space
//! - `sys_mmap_device` - Map device MMIO into process address space
//! - `sys_dma_pool_create` - Allocate from DMA pool (low memory for PCIe devices)
//! - `sys_dma_pool_create_high` - Allocate from high DMA pool (36-bit addresses)
//!
//! # Trait-based Architecture
//!
//! Core memory operations (mmap/munmap) use the MemoryOps trait for clean separation.
//! DMA/device mapping still uses direct implementation for now (will migrate to ObjectOps).

#[allow(unused_imports)]  // Used for phys_to_dma method on dyn Platform
use crate::hal::Platform;
use super::super::uaccess;
use super::super::caps::Capabilities;
use super::super::syscall_ctx_impl::create_syscall_context;
use super::super::traits::syscall_ctx::SyscallContext;
use super::uaccess_to_errno;
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

/// Allocate DMA-capable memory
/// Args: size, dma_ptr (pointer to u64 for DMA address)
/// Returns: virtual address on success, writes DMA address to dma_ptr
///
/// The DMA address is what should be programmed into device descriptors for
/// bus-mastering DMA operations. On platforms with identity-mapped PCIe inbound
/// windows, this equals the CPU physical address. On platforms with an offset,
/// the appropriate translation is applied.
///
/// SECURITY: Writes to user pointer via page table translation
///
/// NOTE: Legacy syscall - pure microkernel uses open(DmaPool) + map() instead.
pub(super) fn sys_mmap_dma(size: usize, dma_ptr: u64) -> i64 {
    // Check DMA capability
    let ctx = create_syscall_context();
    if let Err(_) = ctx.require_capability(Capabilities::DMA.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    if size == 0 {
        return KernelError::InvalidArg.to_errno();
    }

    // Validate dma_ptr
    if dma_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(dma_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }

    super::super::task::with_scheduler(|sched| {
        let task = match sched.current_task_mut() {
            Some(t) => t,
            None => return KernelError::InvalidArg.to_errno(),
        };

        match task.mmap_dma(size) {
            Some((virt_addr, phys_addr)) => {
                // Convert physical address to DMA address using platform HAL
                let platform = crate::platform::current::platform::platform();
                let dma_addr = platform.phys_to_dma(phys_addr);

                // Write DMA address to user pointer
                if dma_ptr != 0 {
                    if let Err(e) = uaccess::put_user::<u64>(dma_ptr, dma_addr) {
                        // Unmap the allocation since we couldn't return the dma addr
                        task.munmap(virt_addr, size);
                        return uaccess_to_errno(e);
                    }
                }
                virt_addr as i64
            }
            None => KernelError::OutOfMemory.to_errno(),
        }
    })
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

/// Map device MMIO into process address space
/// Generic syscall - kernel doesn't track devices, just maps physical memory
/// phys_addr: physical address of device MMIO region
/// size: size in bytes to map
/// Returns: virtual address on success, negative error on failure
///
/// NOTE: Legacy syscall - pure microkernel uses open(Mmio) + map() instead.
pub(super) fn sys_mmap_device(phys_addr: u64, size: u64) -> i64 {
    // Require MMIO capability for device memory mapping
    let ctx = create_syscall_context();
    if let Err(_) = ctx.require_capability(Capabilities::MMIO.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    // Validate size
    if size == 0 || size > 16 * 1024 * 1024 {
        // Max 16MB per mapping
        return KernelError::InvalidArg.to_errno();
    }

    // Map into calling process's address space
    super::super::task::with_scheduler(|sched| {
        match sched.current_task_mut() {
            Some(task) => {
                match task.mmap_device(phys_addr, size as usize) {
                    Some(vaddr) => vaddr as i64,
                    None => KernelError::OutOfMemory.to_errno(),
                }
            }
            None => KernelError::NoProcess.to_errno(),
        }
    })
}

/// Allocate from DMA pool (low memory for PCIe devices)
/// Args: size, vaddr_ptr, paddr_ptr
/// Returns: 0 on success, negative error on failure
///
/// This allocates from a pre-reserved pool of low memory (0x40100000-0x40300000)
/// that some PCIe devices require for DMA operations. Use this instead of
/// shmem_create when the device can't access higher memory addresses.
///
/// NOTE: Legacy syscall - pure microkernel uses open(DmaPool) + map() instead.
pub(super) fn sys_dma_pool_create(size: usize, vaddr_ptr: u64, paddr_ptr: u64) -> i64 {
    // Require DMA capability
    let ctx = create_syscall_context();
    if let Err(_) = ctx.require_capability(Capabilities::DMA.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    if size == 0 {
        return KernelError::InvalidArg.to_errno();
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

    let caller_pid = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    // Allocate from DMA pool
    let phys_addr = match super::super::dma_pool::alloc(size) {
        Ok(addr) => addr,
        Err(_) => return KernelError::OutOfMemory.to_errno(),
    };

    // Map into process address space
    let virt_addr = match super::super::dma_pool::map_into_process(caller_pid, phys_addr, size) {
        Ok(addr) => addr,
        Err(_) => return KernelError::OutOfMemory.to_errno(),
    };

    // Write output values
    if vaddr_ptr != 0 {
        if let Err(e) = uaccess::put_user::<u64>(vaddr_ptr, virt_addr) {
            return uaccess_to_errno(e);
        }
    }
    if paddr_ptr != 0 {
        if let Err(e) = uaccess::put_user::<u64>(paddr_ptr, phys_addr) {
            return uaccess_to_errno(e);
        }
    }

    0 // Success
}

/// Allocate from high DMA pool (36-bit addresses, > 4GB)
///
/// Same interface as sys_dma_pool_create but allocates from high memory
/// for devices that use 36-bit DMA addressing (like MT7996 TX buffers).
///
/// NOTE: Legacy syscall - pure microkernel uses open(DmaPool, HIGH) + map() instead.
pub(super) fn sys_dma_pool_create_high(size: usize, vaddr_ptr: u64, paddr_ptr: u64) -> i64 {
    let ctx = create_syscall_context();
    if let Err(_) = ctx.require_capability(Capabilities::DMA.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    if size == 0 {
        return KernelError::InvalidArg.to_errno();
    }

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

    let caller_pid = match ctx.current_task_id() {
        Some(id) => id,
        None => return KernelError::NoProcess.to_errno(),
    };

    // Allocate from HIGH DMA pool (36-bit addresses)
    let phys_addr = match super::super::dma_pool::alloc_high(size) {
        Ok(addr) => addr,
        Err(_) => return KernelError::OutOfMemory.to_errno(),
    };

    // Map into process address space
    let virt_addr = match super::super::dma_pool::map_into_process_high(caller_pid, phys_addr, size) {
        Ok(addr) => addr,
        Err(_) => return KernelError::OutOfMemory.to_errno(),
    };

    if vaddr_ptr != 0 {
        if let Err(e) = uaccess::put_user::<u64>(vaddr_ptr, virt_addr) {
            return uaccess_to_errno(e);
        }
    }
    if paddr_ptr != 0 {
        if let Err(e) = uaccess::put_user::<u64>(paddr_ptr, phys_addr) {
            return uaccess_to_errno(e);
        }
    }

    0
}
