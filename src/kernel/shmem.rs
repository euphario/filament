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
use super::lock::SpinLock;
use crate::{kinfo, kwarn};

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

/// Shared memory subsystem state (protected by SpinLock for SMP safety)
struct ShmemState {
    table: [Option<SharedMem>; MAX_SHMEM_REGIONS],
    next_id: u32,
}

impl ShmemState {
    const fn new() -> Self {
        // Use array initialization that's const-compatible
        const NONE: Option<SharedMem> = None;
        Self {
            table: [NONE; MAX_SHMEM_REGIONS],
            next_id: 1,
        }
    }
}

/// Global shared memory table protected by SpinLock (SMP-safe)
static SHMEM: SpinLock<ShmemState> = SpinLock::new(ShmemState::new());

/// Execute a closure with exclusive access to the shmem table.
/// SpinLock provides both IRQ and SMP safety.
#[inline]
#[allow(dead_code)]
pub fn with_shmem_table<R, F: FnOnce(&mut [Option<SharedMem>; MAX_SHMEM_REGIONS]) -> R>(f: F) -> R {
    let mut guard = SHMEM.lock();
    f(&mut guard.table)
}

/// Initialize shared memory subsystem
pub fn init() {
    let mut guard = SHMEM.lock();
    for slot in guard.table.iter_mut() {
        *slot = None;
    }
    guard.next_id = 1;
    drop(guard);
    kinfo!("shmem", "init_ok"; max_regions = MAX_SHMEM_REGIONS as u64);
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
    let (slot_idx, shmem_id) = {
        let mut guard = SHMEM.lock();
        let mut found = None;
        for (i, slot) in guard.table.iter().enumerate() {
            if slot.is_none() {
                found = Some(i);
                break;
            }
        }
        let idx = found.ok_or(-12i64)?; // ENOMEM (no slots)
        let id = guard.next_id;
        guard.next_id += 1;
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

    // Map into owner's address space (clean up phys pages on failure)
    let vaddr = match map_into_process(owner_pid, phys_addr as u64, aligned_size) {
        Ok(addr) => addr,
        Err(e) => {
            pmm::free_contiguous(phys_addr, num_pages);
            return Err(e);
        }
    };

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
    {
        let mut guard = SHMEM.lock();
        guard.table[slot_idx] = Some(region);
    }

    Ok((shmem_id, vaddr, phys_addr as u64))
}

/// Map an existing shared memory region into a process
/// Returns (vaddr, paddr) or error
pub fn map(pid: Pid, shmem_id: u32) -> Result<(u64, u64), i64> {
    let (phys_addr, size) = {
        let mut guard = SHMEM.lock();
        let mut found = None;
        for slot in guard.table.iter_mut() {
            if let Some(ref mut region) = slot {
                if region.id == shmem_id {
                    // Check if allowed
                    if !region.is_allowed(pid) {
                        return Err(-13); // EACCES
                    }
                    region.ref_count += 1;
                    found = Some((region.phys_addr, region.size));
                    break;
                }
            }
        }
        found.ok_or(-2i64)? // ENOENT
    };

    // Map into process's address space
    let vaddr = map_into_process(pid, phys_addr, size)?;

    Ok((vaddr, phys_addr))
}

/// Unmap shared memory from a process (decrements ref_count)
/// Called when a process unmaps shmem or exits
pub fn unmap(pid: Pid, shmem_id: u32) -> Result<(), i64> {
    let mut guard = SHMEM.lock();
    for slot in guard.table.iter_mut() {
        if let Some(ref mut region) = slot {
            if region.id == shmem_id {
                // Check if this process has it mapped (must be allowed)
                if !region.is_allowed(pid) {
                    return Err(-13); // EACCES
                }

                // Decrement ref_count (but never below 0)
                if region.ref_count > 0 {
                    region.ref_count -= 1;
                }

                // Also remove from waiters if waiting
                region.remove_waiter(pid);
                return Ok(());
            }
        }
    }
    Err(-2) // ENOENT
}

/// Grant another process permission to map this shared memory
pub fn allow(owner_pid: Pid, shmem_id: u32, peer_pid: Pid) -> Result<(), i64> {
    let mut guard = SHMEM.lock();
    for slot in guard.table.iter_mut() {
        if let Some(ref mut region) = slot {
            if region.id == shmem_id {
                // Only owner can grant access
                if region.owner_pid != owner_pid {
                    return Err(-1); // EPERM
                }

                if !region.allow_pid(peer_pid) {
                    return Err(-12); // ENOMEM (no space in allowed list)
                }
                return Ok(());
            }
        }
    }
    Err(-2) // ENOENT
}

/// Wait for notification on shared memory
/// Returns Err(-EAGAIN) to signal caller should block.
/// The syscall layer handles actual blocking and rescheduling.
/// If timeout_ms > 0, the wait will automatically complete after the timeout.
/// timeout_ms = 0 means wait indefinitely.
pub fn wait(pid: Pid, shmem_id: u32, timeout_ms: u32) -> Result<(), i64> {
    // Add to waiters list under shmem lock
    {
        let mut guard = SHMEM.lock();
        let mut found = false;
        for slot in guard.table.iter_mut() {
            if let Some(ref mut region) = slot {
                if region.id == shmem_id {
                    // Check access
                    if !region.is_allowed(pid) {
                        return Err(-13); // EACCES
                    }

                    if !region.add_waiter(pid) {
                        return Err(-12); // ENOMEM (no space in waiters)
                    }
                    found = true;
                    break;
                }
            }
        }
        if !found {
            return Err(-2); // ENOENT
        }
    }
    // shmem lock released here before touching scheduler

    // Mark task as blocked - syscall layer will handle scheduling
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            task.state = super::task::TaskState::Blocked;
            task.wait_reason = Some(super::task::WaitReason::ShmemNotify(shmem_id));

            // Set wake timeout if specified
            if timeout_ms > 0 {
                // Timer runs at 100 Hz (10ms per tick), so timeout_ms / 10 = ticks
                // Add 1 to round up and avoid 0-tick timeouts
                let timeout_ticks = (timeout_ms as u64 + 9) / 10;
                let current_tick = crate::platform::mt7988::timer::ticks();
                // Use saturating_add to prevent overflow (would cause immediate wake)
                task.wake_at = current_tick.saturating_add(timeout_ticks);
            } else {
                task.wake_at = 0; // No timeout
            }
        }
    }

    // Return "would block" - syscall exit path will reschedule
    Err(-11) // EAGAIN
}

/// Notify all waiters on shared memory
/// Uses O(1) wake via PID→slot map
pub fn notify(pid: Pid, shmem_id: u32) -> Result<u32, i64> {
    // Drain waiters under shmem lock
    let waiters = {
        let mut guard = SHMEM.lock();
        let mut found_waiters = None;
        for slot in guard.table.iter_mut() {
            if let Some(ref mut region) = slot {
                if region.id == shmem_id {
                    // Check access (anyone with access can notify)
                    if !region.is_allowed(pid) {
                        return Err(-13); // EACCES
                    }
                    found_waiters = Some(region.drain_waiters());
                    break;
                }
            }
        }
        found_waiters.ok_or(-2i64)? // ENOENT
    };
    // shmem lock released here before touching scheduler

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
///
/// SECURITY: Refuses to destroy if ref_count > 1 (other processes still have it mapped).
/// This prevents use-after-free where destroying shared memory while another process
/// is actively using it would cause memory corruption.
pub fn destroy(owner_pid: Pid, shmem_id: u32) -> Result<(), i64> {
    // Extract info and validate under shmem lock
    let (phys_addr, size, slot_idx) = {
        let guard = SHMEM.lock();
        let mut found = None;
        for (i, slot) in guard.table.iter().enumerate() {
            if let Some(ref region) = slot {
                if region.id == shmem_id {
                    // Only owner can destroy
                    if region.owner_pid != owner_pid {
                        return Err(-1); // EPERM
                    }

                    // SECURITY: Refuse to destroy if other processes still have it mapped
                    // ref_count of 1 means only the owner has it mapped (safe to destroy)
                    // ref_count > 1 means other processes still have active mappings
                    if region.ref_count > 1 {
                        kwarn!("shmem", "destroy_refused_busy"; id = shmem_id as u64, ref_count = region.ref_count as u64);
                        return Err(-16); // EBUSY - resource busy
                    }

                    found = Some((region.phys_addr, region.size, i));
                    break;
                }
            }
        }
        found.ok_or(-2i64)? // ENOENT
    };
    // shmem lock released here before touching scheduler

    // Safe to unmap - only owner has it mapped
    unsafe {
        unmap_from_all_processes(phys_addr, size);
    }

    // Clear slot and free physical memory
    {
        let mut guard = SHMEM.lock();
        guard.table[slot_idx] = None;
    }

    let num_pages = size / 4096;
    pmm::free_contiguous(phys_addr as usize, num_pages);

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

/// Map physical memory into a process's address space for DMA
/// Uses non-cacheable device memory attributes (nGnRnE) because shmem
/// is primarily used for DMA rings where CPU cache coherency with
/// PCIe/USB devices is critical.
fn map_into_process(pid: Pid, phys_addr: u64, size: usize) -> Result<u64, i64> {
    unsafe {
        let sched = super::task::scheduler();

        // Find the task
        for task_opt in sched.tasks.iter_mut() {
            if let Some(ref mut task) = task_opt {
                if task.id == pid {
                    // Use mmap_shmem_dma: non-cacheable for DMA coherency,
                    // but BorrowedShmem kind so pages are managed by shmem subsystem
                    return task.mmap_shmem_dma(phys_addr, size)
                        .ok_or(-12i64); // ENOMEM
                }
            }
        }
    }
    Err(-3) // ESRCH - no such process
}

/// Called when a process exits - clean up any shared memory it owns or has mapped
pub fn process_cleanup(pid: Pid) {
    // Collect regions owned by this process (need to unmap outside lock)
    let mut owned_regions: [(u64, usize, usize); MAX_SHMEM_REGIONS] = [(0, 0, 0); MAX_SHMEM_REGIONS];
    let mut owned_count = 0;

    // First pass: update ref counts and collect owned regions
    {
        let mut guard = SHMEM.lock();
        for (i, slot) in guard.table.iter_mut().enumerate() {
            if let Some(ref mut region) = slot {
                // Remove from waiters if present
                region.remove_waiter(pid);

                // If this process had it mapped (is in allowed list), decrement ref_count
                // This handles non-owner processes that had mapped the shmem
                if region.is_allowed(pid) && region.owner_pid != pid {
                    if region.ref_count > 0 {
                        region.ref_count -= 1;
                    }
                }

                // If owner, mark for destruction
                if region.owner_pid == pid {
                    owned_regions[owned_count] = (region.phys_addr, region.size, i);
                    owned_count += 1;
                }
            }
        }
    }
    // shmem lock released before touching scheduler

    // Second pass: destroy owned regions (unmap from other processes)
    for i in 0..owned_count {
        let (phys_addr, size, slot_idx) = owned_regions[i];
        let num_pages = size / 4096;

        // CRITICAL: Unmap from ALL other processes before freeing
        unsafe {
            unmap_from_all_processes(phys_addr, size);
        }

        // Clear the slot
        {
            let mut guard = SHMEM.lock();
            guard.table[slot_idx] = None;
        }

        pmm::free_contiguous(phys_addr as usize, num_pages);
    }
}

// ============================================================================
// Self-tests
// ============================================================================

#[cfg(feature = "selftest")]
pub fn test() {
    use crate::{kdebug, kinfo};

    kdebug!("shmem", "test_start");

    // Test 1: Shmem manager is initialized
    {
        let guard = SHMEM.lock();
        // Check that table has expected capacity
        assert!(guard.table.len() > 0, "Shmem table should have capacity");
        drop(guard);
        kdebug!("shmem", "init_ok");
    }

    // Test 2: Create small shmem region
    // Note: create(owner_pid, size) returns (id, phys_addr, virt_addr)
    {
        let result = create(0, 4096); // 1 page, owner PID 0
        assert!(result.is_ok(), "create(0, 4096) should succeed");
        let (id, phys, _virt) = result.unwrap();
        assert!(id > 0, "Shmem ID should be positive");
        assert!(phys >= 0x4000_0000, "Physical address should be in DRAM");
        assert_eq!(phys % 4096, 0, "Physical address should be page-aligned");

        // Verify the region exists in table
        let guard = SHMEM.lock();
        let found = guard.table.iter().any(|slot| {
            if let Some(ref r) = slot {
                r.id == id
            } else {
                false
            }
        });
        assert!(found, "Created region should be in table");
        drop(guard);

        kdebug!("shmem", "create_ok"; id = id as u64, phys = crate::klog::hex64(phys));
    }

    // Test 3: Zero-size creation fails
    {
        let result = create(0, 0);
        assert!(result.is_err(), "Zero-size create should fail");
        kdebug!("shmem", "zero_create_rejected");
    }

    // Test 4: Region lookup works
    {
        let result = create(0, 8192); // 2 pages
        assert!(result.is_ok(), "create should succeed");
        let (id, phys, _virt) = result.unwrap();

        // Look it up
        let guard = SHMEM.lock();
        let found = guard.table.iter().find_map(|slot| {
            if let Some(ref r) = slot {
                if r.id == id {
                    Some((r.phys_addr, r.size))
                } else {
                    None
                }
            } else {
                None
            }
        });
        drop(guard);

        assert!(found.is_some(), "Should find created region");
        let (found_phys, found_size) = found.unwrap();
        assert_eq!(found_phys, phys, "Physical address should match");
        assert_eq!(found_size, 8192, "Size should match");
        kdebug!("shmem", "lookup_ok"; id = id as u64);
    }

    // Test 5: ID generation is unique
    {
        let r1 = create(0, 4096).unwrap();
        let r2 = create(0, 4096).unwrap();
        assert_ne!(r1.0, r2.0, "IDs should be unique");
        kdebug!("shmem", "unique_ids_ok"; id1 = r1.0 as u64, id2 = r2.0 as u64);
    }

    kinfo!("shmem", "test_ok");
}
