//! Shared Memory Subsystem

#![allow(dead_code)]  // Infrastructure for future use
//!
//! Provides DMA-capable shared memory regions that can be mapped into
//! multiple processes. Used as a building block for high-performance
//! IPC mechanisms like ring buffers.
//!
//! ## Design
//!
//! The kernel manages shared memory regions and access control.
//! Ring buffer structure and protocol are handled in userspace.
//!
//! ## Syscalls
//!
//! - `shmem_create(size, &vaddr, &paddr)` - Create new region
//! - `shmem_map(id, &vaddr, &paddr)` - Map existing region
//! - `shmem_allow(id, peer_pid)` - Grant access to peer
//! - `shmem_wait(id, timeout_ms)` - Wait for notification
//! - `shmem_notify(id)` - Wake all waiters
//! - `shmem_destroy(id)` - Destroy region

use super::pmm;
use super::process::Pid;
use crate::logln;

/// Maximum number of shared memory regions
const MAX_SHMEM_REGIONS: usize = 32;

/// Maximum processes that can be granted access to a region
const MAX_ALLOWED_PIDS: usize = 8;

/// Maximum waiters per region
const MAX_WAITERS: usize = 8;

/// Invalid/no PID marker
const NO_PID: Pid = 0;

/// Shared memory region descriptor
#[derive(Clone, Copy)]
pub struct SharedMem {
    /// Unique ID for this region
    pub id: u32,
    /// Owner process ID (creator)
    pub owner_pid: Pid,
    /// Physical address of the region
    pub phys_addr: u64,
    /// Size in bytes
    pub size: usize,
    /// Processes allowed to map this region
    pub allowed_pids: [Pid; MAX_ALLOWED_PIDS],
    /// Processes currently waiting for notification
    pub waiters: [Pid; MAX_WAITERS],
    /// Reference count (number of mappings)
    pub ref_count: u32,
}

impl SharedMem {
    /// Create a new shared memory region
    const fn new() -> Self {
        Self {
            id: 0,
            owner_pid: NO_PID,
            phys_addr: 0,
            size: 0,
            allowed_pids: [NO_PID; MAX_ALLOWED_PIDS],
            waiters: [NO_PID; MAX_WAITERS],
            ref_count: 0,
        }
    }

    /// Check if a PID is allowed to access this region
    pub fn is_allowed(&self, pid: Pid) -> bool {
        // Owner always allowed
        if pid == self.owner_pid {
            return true;
        }
        // Check allowed list
        for &allowed in &self.allowed_pids {
            if allowed == pid {
                return true;
            }
        }
        false
    }

    /// Add a PID to the allowed list
    pub fn allow_pid(&mut self, pid: Pid) -> bool {
        // Check if already allowed
        if self.is_allowed(pid) {
            return true;
        }
        // Find empty slot
        for slot in &mut self.allowed_pids {
            if *slot == NO_PID {
                *slot = pid;
                return true;
            }
        }
        false // No space
    }

    /// Add a waiter
    pub fn add_waiter(&mut self, pid: Pid) -> bool {
        for slot in &mut self.waiters {
            if *slot == NO_PID {
                *slot = pid;
                return true;
            }
        }
        false // No space
    }

    /// Remove a waiter
    pub fn remove_waiter(&mut self, pid: Pid) {
        for slot in &mut self.waiters {
            if *slot == pid {
                *slot = NO_PID;
                return;
            }
        }
    }

    /// Get all waiters and clear the list
    pub fn drain_waiters(&mut self) -> [Pid; MAX_WAITERS] {
        let waiters = self.waiters;
        self.waiters = [NO_PID; MAX_WAITERS];
        waiters
    }
}

/// Global shared memory table
static mut SHMEM_TABLE: [Option<SharedMem>; MAX_SHMEM_REGIONS] = [None; MAX_SHMEM_REGIONS];

/// Next ID to assign
static mut NEXT_SHMEM_ID: u32 = 1;

/// Initialize shared memory subsystem
pub fn init() {
    unsafe {
        for slot in (*core::ptr::addr_of_mut!(SHMEM_TABLE)).iter_mut() {
            *slot = None;
        }
        *core::ptr::addr_of_mut!(NEXT_SHMEM_ID) = 1;
    }
    logln!("[shmem] Initialized ({} max regions)", MAX_SHMEM_REGIONS);
}

/// Create a new shared memory region
/// Returns (shmem_id, vaddr, paddr) or error
pub fn create(owner_pid: Pid, size: usize) -> Result<(u32, u64, u64), i64> {
    if size == 0 {
        return Err(-22); // EINVAL
    }

    // Round up to page size
    let page_size = 4096usize;
    let aligned_size = (size + page_size - 1) & !(page_size - 1);
    let num_pages = aligned_size / page_size;

    // CRITICAL: Find free slot BEFORE allocating physical memory
    // This prevents memory leak if no slots are available
    let (slot_idx, shmem_id) = unsafe {
        let mut found = None;
        for (i, slot) in (*core::ptr::addr_of!(SHMEM_TABLE)).iter().enumerate() {
            if slot.is_none() {
                found = Some(i);
                break;
            }
        }
        let idx = found.ok_or(-12i64)?; // ENOMEM (no slots)
        let id = *core::ptr::addr_of!(NEXT_SHMEM_ID);
        *core::ptr::addr_of_mut!(NEXT_SHMEM_ID) += 1;
        (idx, id)
    };

    // Now allocate contiguous physical pages for DMA
    // If this fails, we haven't modified any state yet
    let phys_addr = pmm::alloc_contiguous(num_pages)
        .ok_or(-12i64)?; // ENOMEM

    // Create the region descriptor
    let mut region = SharedMem::new();
    region.id = shmem_id;
    region.owner_pid = owner_pid;
    region.phys_addr = phys_addr as u64;
    region.size = aligned_size;
    region.ref_count = 1; // Owner's mapping

    // Map into owner's address space
    let vaddr = map_into_process(owner_pid, phys_addr as u64, aligned_size)?;

    // Zero the memory through kernel's direct-map
    unsafe {
        let kern_vaddr = crate::arch::aarch64::mmu::phys_to_virt(phys_addr as u64);
        core::ptr::write_bytes(kern_vaddr as *mut u8, 0, aligned_size);

        // CRITICAL: Flush kernel cache to physical memory and invalidate
        // Userspace maps this as non-cacheable, so we must ensure kernel's
        // cached zeros are written to memory and won't later overwrite
        // userspace writes when evicted
        for offset in (0..aligned_size).step_by(64) {
            let addr = kern_vaddr + offset as u64;
            core::arch::asm!(
                "dc civac, {addr}",  // Clean and Invalidate by VA to PoC
                addr = in(reg) addr,
                options(nostack, preserves_flags)
            );
        }
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }

    // Store in table
    unsafe {
        (*core::ptr::addr_of_mut!(SHMEM_TABLE))[slot_idx] = Some(region);
    }

    Ok((shmem_id, vaddr, phys_addr as u64))
}

/// Map an existing shared memory region into a process
/// Returns (vaddr, paddr) or error
pub fn map(pid: Pid, shmem_id: u32) -> Result<(u64, u64), i64> {
    let (phys_addr, size) = unsafe {
        let region = find_region_mut(shmem_id)?;

        // Check if allowed
        if !region.is_allowed(pid) {
            return Err(-13); // EACCES
        }

        region.ref_count += 1;
        (region.phys_addr, region.size)
    };

    // Map into process's address space
    let vaddr = map_into_process(pid, phys_addr, size)?;

    Ok((vaddr, phys_addr))
}

/// Grant another process permission to map this shared memory
pub fn allow(owner_pid: Pid, shmem_id: u32, peer_pid: Pid) -> Result<(), i64> {
    unsafe {
        let region = find_region_mut(shmem_id)?;

        // Only owner can grant access
        if region.owner_pid != owner_pid {
            return Err(-1); // EPERM
        }

        if !region.allow_pid(peer_pid) {
            return Err(-12); // ENOMEM (no space in allowed list)
        }
    }
    Ok(())
}

/// Wait for notification on shared memory
/// Returns Err(-EAGAIN) to signal caller should block.
/// The syscall layer handles actual blocking and rescheduling.
/// TODO: timeout_ms is currently ignored
pub fn wait(pid: Pid, shmem_id: u32, _timeout_ms: u32) -> Result<(), i64> {
    // Add to waiters list and mark task as blocked
    unsafe {
        let region = find_region_mut(shmem_id)?;

        // Check access
        if !region.is_allowed(pid) {
            return Err(-13); // EACCES
        }

        if !region.add_waiter(pid) {
            return Err(-12); // ENOMEM (no space in waiters)
        }
    }

    // Mark task as blocked - syscall layer will handle scheduling
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            task.state = super::task::TaskState::Blocked;
            task.wait_reason = Some(super::task::WaitReason::ShmemNotify(shmem_id));
        }
    }

    // Return "would block" - syscall exit path will reschedule
    Err(-11) // EAGAIN
}

/// Notify all waiters on shared memory
/// Uses O(1) wake via PID→slot map
pub fn notify(pid: Pid, shmem_id: u32) -> Result<u32, i64> {
    let waiters = unsafe {
        let region = find_region_mut(shmem_id)?;

        // Check access (anyone with access can notify)
        if !region.is_allowed(pid) {
            return Err(-13); // EACCES
        }

        region.drain_waiters()
    };

    // Wake all waiters using O(1) PID→slot lookup
    let mut woken = 0u32;
    unsafe {
        let sched = super::task::scheduler();
        for waiter_pid in waiters.iter() {
            if *waiter_pid == NO_PID {
                continue;
            }
            if sched.wake_by_pid(*waiter_pid) {
                woken += 1;
            }
        }
    }

    Ok(woken)
}

/// Destroy a shared memory region (only owner can do this)
pub fn destroy(owner_pid: Pid, shmem_id: u32) -> Result<(), i64> {
    unsafe {
        let slot_idx = find_region_slot(shmem_id)?;

        if let Some(ref region) = SHMEM_TABLE[slot_idx] {
            // Only owner can destroy
            if region.owner_pid != owner_pid {
                return Err(-1); // EPERM
            }

            let phys_addr = region.phys_addr;
            let size = region.size;
            let num_pages = size / 4096;

            // CRITICAL: Unmap from ALL processes that have this shmem mapped
            // This prevents use-after-free when we free the physical memory
            unmap_from_all_processes(phys_addr, size);

            // Now safe to free physical memory - no more mappings exist
            pmm::free_contiguous(phys_addr as usize, num_pages);
        }

        SHMEM_TABLE[slot_idx] = None;
    }
    Ok(())
}

/// Unmap a physical address range from all processes
/// Used when destroying shared memory to prevent use-after-free
unsafe fn unmap_from_all_processes(phys_addr: u64, size: usize) {
    let sched = super::task::scheduler();

    for task_opt in sched.tasks.iter_mut() {
        if let Some(ref mut task) = task_opt {
            // Search task's heap mappings for this physical address
            // First find the virtual address, then unmap (avoids borrow issues)
            let mut virt_to_unmap = None;
            for mapping in task.heap_mappings.iter() {
                if !mapping.is_empty() && mapping.phys_addr == phys_addr {
                    virt_to_unmap = Some(mapping.virt_addr);
                    break; // Each task should only have one mapping to this phys
                }
            }

            // Now unmap if we found a mapping
            // Note: munmap() won't free pages because kind is BorrowedShmem
            if let Some(virt_addr) = virt_to_unmap {
                task.munmap(virt_addr, size);
            }
        }
    }
}

/// Find a region by ID and return mutable reference
unsafe fn find_region_mut(shmem_id: u32) -> Result<&'static mut SharedMem, i64> {
    for slot in (*core::ptr::addr_of_mut!(SHMEM_TABLE)).iter_mut() {
        if let Some(ref mut region) = slot {
            if region.id == shmem_id {
                return Ok(region);
            }
        }
    }
    Err(-2) // ENOENT
}

/// Find a region's slot index by ID
unsafe fn find_region_slot(shmem_id: u32) -> Result<usize, i64> {
    for (i, slot) in (*core::ptr::addr_of!(SHMEM_TABLE)).iter().enumerate() {
        if let Some(ref region) = slot {
            if region.id == shmem_id {
                return Ok(i);
            }
        }
    }
    Err(-2) // ENOENT
}

/// Map physical memory into a process's address space
fn map_into_process(pid: Pid, phys_addr: u64, size: usize) -> Result<u64, i64> {
    unsafe {
        let sched = super::task::scheduler();

        // Find the task
        for task_opt in sched.tasks.iter_mut() {
            if let Some(ref mut task) = task_opt {
                if task.id == pid {
                    // Use the task's address space to allocate a virtual address
                    // and create the mapping
                    return task.mmap_phys(phys_addr, size)
                        .ok_or(-12i64); // ENOMEM
                }
            }
        }
    }
    Err(-3) // ESRCH - no such process
}

/// Called when a process exits - clean up any shared memory it owns
pub fn process_cleanup(pid: Pid) {
    unsafe {
        for slot in (*core::ptr::addr_of_mut!(SHMEM_TABLE)).iter_mut() {
            if let Some(ref mut region) = slot {
                // Remove from waiters if present
                region.remove_waiter(pid);

                // If owner, destroy the region (same as destroy() but without permission check)
                if region.owner_pid == pid {
                    let phys_addr = region.phys_addr;
                    let size = region.size;
                    let num_pages = size / 4096;

                    // CRITICAL: Unmap from ALL other processes before freeing
                    unmap_from_all_processes(phys_addr, size);

                    pmm::free_contiguous(phys_addr as usize, num_pages);
                    *slot = None;
                }
            }
        }
    }
}
