//! Memory-related system calls
//!
//! This module contains syscall handlers for memory management operations:
//! - `sys_mmap` - Map memory pages into user address space
//! - `sys_mmap_dma` - Allocate DMA-capable memory
//! - `sys_munmap` - Unmap memory pages from user address space
//! - `sys_mmap_device` - Map device MMIO into process address space
//! - `sys_dma_pool_create` - Allocate from DMA pool (low memory for PCIe devices)
//! - `sys_dma_pool_create_high` - Allocate from high DMA pool (36-bit addresses)

#[allow(unused_imports)]  // Used for phys_to_dma method on dyn Platform
use crate::hal::Platform;
use super::super::uaccess;
use super::super::caps::Capabilities;
use super::{SyscallError, current_pid, require_capability, uaccess_to_errno};

/// Map memory pages into user address space
/// Args: addr (hint, ignored for now), size, prot
/// Returns: virtual address on success, negative error on failure
pub(super) fn sys_mmap(_addr: u64, size: usize, prot: u32) -> i64 {
    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Parse protection flags (simplified: bit 0 = write, bit 1 = exec)
    let writable = (prot & 0x2) != 0;  // PROT_WRITE
    let executable = (prot & 0x4) != 0; // PROT_EXEC

    unsafe {
        let sched = super::super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            match task.mmap(size, writable, executable) {
                Some(virt_addr) => virt_addr as i64,
                None => SyscallError::OutOfMemory as i64,
            }
        } else {
            SyscallError::InvalidArgument as i64
        }
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
pub(super) fn sys_mmap_dma(size: usize, dma_ptr: u64) -> i64 {
    // Check DMA capability
    if let Err(e) = require_capability(Capabilities::DMA) {
        return e;
    }

    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate dma_ptr
    if dma_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(dma_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }

    unsafe {
        let sched = super::super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            match task.mmap_dma(size) {
                Some((virt_addr, phys_addr)) => {
                    // Convert physical address to DMA address using platform HAL
                    let platform = crate::platform::mt7988::platform::platform();
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
                None => SyscallError::OutOfMemory as i64,
            }
        } else {
            SyscallError::InvalidArgument as i64
        }
    }
}

/// Unmap memory pages from user address space
/// Args: addr, size
/// Returns: 0 on success, negative error on failure
pub(super) fn sys_munmap(addr: u64, size: usize) -> i64 {
    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate that the address is in user space (not kernel)
    if !uaccess::is_user_address(addr) {
        return SyscallError::BadAddress as i64;
    }

    unsafe {
        let sched = super::super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if task.munmap(addr, size) {
                SyscallError::Success as i64
            } else {
                SyscallError::InvalidArgument as i64
            }
        } else {
            SyscallError::InvalidArgument as i64
        }
    }
}

/// Map device MMIO into process address space
/// Generic syscall - kernel doesn't track devices, just maps physical memory
/// phys_addr: physical address of device MMIO region
/// size: size in bytes to map
/// Returns: virtual address on success, negative error on failure
pub(super) fn sys_mmap_device(phys_addr: u64, size: u64) -> i64 {
    // Require MMIO capability for device memory mapping
    if let Err(e) = require_capability(Capabilities::MMIO) {
        return e;
    }

    // Validate size
    if size == 0 || size > 16 * 1024 * 1024 {
        // Max 16MB per mapping
        return SyscallError::InvalidArgument as i64;
    }

    // Map into calling process's address space
    let pid = current_pid();
    unsafe {
        let sched = super::super::task::scheduler();
        for task_opt in sched.tasks.iter_mut() {
            if let Some(ref mut task) = task_opt {
                if task.id == pid {
                    match task.mmap_device(phys_addr, size as usize) {
                        Some(vaddr) => return vaddr as i64,
                        None => return SyscallError::OutOfMemory as i64,
                    }
                }
            }
        }
    }

    SyscallError::NoProcess as i64
}

/// Allocate from DMA pool (low memory for PCIe devices)
/// Args: size, vaddr_ptr, paddr_ptr
/// Returns: 0 on success, negative error on failure
///
/// This allocates from a pre-reserved pool of low memory (0x40100000-0x40300000)
/// that some PCIe devices require for DMA operations. Use this instead of
/// shmem_create when the device can't access higher memory addresses.
pub(super) fn sys_dma_pool_create(size: usize, vaddr_ptr: u64, paddr_ptr: u64) -> i64 {
    // Require DMA capability
    if let Err(e) = require_capability(Capabilities::DMA) {
        return e;
    }

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

    // Allocate from DMA pool
    let phys_addr = match super::super::dma_pool::alloc(size) {
        Ok(addr) => addr,
        Err(_) => return SyscallError::OutOfMemory as i64,
    };

    // Map into process address space
    let virt_addr = match super::super::dma_pool::map_into_process(caller_pid, phys_addr, size) {
        Ok(addr) => addr,
        Err(_) => return SyscallError::OutOfMemory as i64,
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
pub(super) fn sys_dma_pool_create_high(size: usize, vaddr_ptr: u64, paddr_ptr: u64) -> i64 {
    if let Err(e) = require_capability(Capabilities::DMA) {
        return e;
    }

    if size == 0 {
        return SyscallError::InvalidArgument as i64;
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

    let caller_pid = current_pid();

    // Allocate from HIGH DMA pool (36-bit addresses)
    let phys_addr = match super::super::dma_pool::alloc_high(size) {
        Ok(addr) => addr,
        Err(_) => return SyscallError::OutOfMemory as i64,
    };

    // Map into process address space
    let virt_addr = match super::super::dma_pool::map_into_process_high(caller_pid, phys_addr, size) {
        Ok(addr) => addr,
        Err(_) => return SyscallError::OutOfMemory as i64,
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
