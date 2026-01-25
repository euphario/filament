//! Shared Memory Subsystem
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
//! ## Key Fix: Mapping Tracking
//!
//! Instead of using a simple ref_count that can be manipulated, we now
//! track actual (region_id, pid) pairs in a MappingTable. This prevents:
//! - Double-free (unmap called twice)
//! - Ref-count underflow (malicious repeated unmap)
//! - Use-after-free (destroy while others have mappings)
//!
//! ## Syscalls (via unified interface)
//!
//! - `open(Shmem, [size])` - Create new region
//! - `open(Shmem, [flags=1, id])` - Map existing region
//! - `write(handle, [cmd=0, pid])` - Allow peer (ALLOW command)
//! - `write(handle, [cmd=1])` - Notify waiters (NOTIFY command)
//! - `read(handle, [timeout])` - Wait for notification
//! - `close(handle)` - Unmap/destroy region

#![allow(dead_code)]  // Infrastructure for future use

use super::pmm;
use super::process::Pid;
use super::lock::SpinLock;
use crate::{kinfo, kwarn, kerror};

/// Maximum number of shared memory regions
const MAX_SHMEM_REGIONS: usize = 32;

/// Maximum processes that can be granted access to a region
const MAX_ALLOWED_PIDS: usize = 8;

/// Maximum waiters per region
const MAX_WAITERS: usize = 8;

/// Maximum total mappings tracked globally (prevents unbounded growth)
const MAX_TOTAL_MAPPINGS: usize = MAX_SHMEM_REGIONS * MAX_ALLOWED_PIDS;

/// Invalid/no PID marker
const NO_PID: Pid = 0;

/// Page size constant
const PAGE_SIZE: usize = 4096;

/// State of a shared memory region
///
/// State machine:
/// ```text
/// ┌────────┐    owner exit    ┌────────┐   finalize   ┌─────────┐
/// │ Active │ ─────────────> │ Dying  │ ──────────> │ (freed) │
/// └────────┘                 └────────┘             └─────────┘
///                               │
///                               │ all mappers unmap early
///                               ▼
///                          (can be freed)
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RegionState {
    /// Region is active and can be used for mapping and notifications
    Active,
    /// Owner is dying - ShmemInvalid events sent, waiting for mappers to release.
    /// New mappings are NOT allowed in this state.
    Dying,
}

impl RegionState {
    /// Check if transition to new state is valid
    pub fn can_transition_to(&self, new: &RegionState) -> bool {
        match (self, new) {
            // Active → Dying (owner starts exit sequence)
            (RegionState::Active, RegionState::Dying) => true,
            // All other transitions are invalid (including Dying → Active)
            _ => false,
        }
    }

    /// Get state name for logging
    pub fn name(&self) -> &'static str {
        match self {
            RegionState::Active => "active",
            RegionState::Dying => "dying",
        }
    }
}

impl super::ipc::traits::StateMachine for RegionState {
    fn can_transition_to(&self, new: &Self) -> bool {
        RegionState::can_transition_to(self, new)
    }

    fn name(&self) -> &'static str {
        RegionState::name(self)
    }
}

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
    /// Region state (Active or Dying) - private, use state() and transition methods
    state: RegionState,
}

impl SharedMem {
    /// Get current state
    pub fn state(&self) -> RegionState {
        self.state
    }
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
            state: RegionState::Active,
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

    /// Transition to Dying state (owner is exiting)
    /// Returns true if transition succeeded
    pub fn begin_dying(&mut self) -> bool {
        if self.state.can_transition_to(&RegionState::Dying) {
            self.state = RegionState::Dying;
            true
        } else {
            false
        }
    }
}

// ============================================================================
// Mapping Table - Tracks actual (region_id, pid) pairs
// ============================================================================

/// A single mapping entry tracking which pid has which region mapped
#[derive(Clone, Copy)]
struct MappingEntry {
    region_id: u32,
    pid: Pid,
}

impl MappingEntry {
    const fn empty() -> Self {
        Self { region_id: 0, pid: NO_PID }
    }

    fn is_empty(&self) -> bool {
        self.pid == NO_PID
    }
}

/// Mapping table that tracks actual mappings to prevent double-free
///
/// This solves the ref_count underflow vulnerability:
/// - add_mapping() fails if pid already has this region mapped
/// - remove_mapping() fails if pid doesn't have this region mapped
struct MappingTable {
    entries: [MappingEntry; MAX_TOTAL_MAPPINGS],
    count: usize,
}

impl MappingTable {
    const fn new() -> Self {
        Self {
            entries: [MappingEntry::empty(); MAX_TOTAL_MAPPINGS],
            count: 0,
        }
    }

    /// Add a mapping for (region_id, pid)
    ///
    /// Returns Ok(()) on success, Err(-EEXIST) if already mapped, Err(-ENOMEM) if full
    fn add_mapping(&mut self, region_id: u32, pid: Pid) -> Result<(), i64> {
        // Check if already mapped (prevents double-map)
        for entry in &self.entries[..self.count] {
            if entry.region_id == region_id && entry.pid == pid {
                return Err(-17); // EEXIST - already mapped
            }
        }

        // Find empty slot
        if self.count < MAX_TOTAL_MAPPINGS {
            self.entries[self.count] = MappingEntry { region_id, pid };
            self.count += 1;
            Ok(())
        } else {
            Err(-12) // ENOMEM - no slots
        }
    }

    /// Remove a mapping for (region_id, pid)
    ///
    /// Returns Ok(()) on success, Err(-EINVAL) if not mapped
    fn remove_mapping(&mut self, region_id: u32, pid: Pid) -> Result<(), i64> {
        for i in 0..self.count {
            if self.entries[i].region_id == region_id && self.entries[i].pid == pid {
                // Swap with last and decrement count
                if i < self.count - 1 {
                    self.entries[i] = self.entries[self.count - 1];
                }
                self.entries[self.count - 1] = MappingEntry::empty();
                self.count -= 1;
                return Ok(());
            }
        }
        Err(-22) // EINVAL - not mapped (prevents double-free)
    }

    /// Check if pid has region mapped
    fn has_mapping(&self, region_id: u32, pid: Pid) -> bool {
        for entry in &self.entries[..self.count] {
            if entry.region_id == region_id && entry.pid == pid {
                return true;
            }
        }
        false
    }

    /// Get count of mappings for a region
    fn mapping_count(&self, region_id: u32) -> usize {
        let mut count = 0;
        for entry in &self.entries[..self.count] {
            if entry.region_id == region_id {
                count += 1;
            }
        }
        count
    }

    /// Remove all mappings for a pid (process cleanup)
    fn remove_all_for_pid(&mut self, pid: Pid) {
        let mut i = 0;
        while i < self.count {
            if self.entries[i].pid == pid {
                if i < self.count - 1 {
                    self.entries[i] = self.entries[self.count - 1];
                }
                self.entries[self.count - 1] = MappingEntry::empty();
                self.count -= 1;
                // Don't increment i - check the swapped entry
            } else {
                i += 1;
            }
        }
    }

    /// Remove all mappings for a region (region destruction)
    fn remove_all_for_region(&mut self, region_id: u32) {
        let mut i = 0;
        while i < self.count {
            if self.entries[i].region_id == region_id {
                if i < self.count - 1 {
                    self.entries[i] = self.entries[self.count - 1];
                }
                self.entries[self.count - 1] = MappingEntry::empty();
                self.count -= 1;
            } else {
                i += 1;
            }
        }
    }

    /// Get all PIDs that have a region mapped
    fn get_mapped_pids(&self, region_id: u32) -> [Pid; MAX_ALLOWED_PIDS] {
        let mut pids = [NO_PID; MAX_ALLOWED_PIDS];
        let mut idx = 0;
        for entry in &self.entries[..self.count] {
            if entry.region_id == region_id && idx < MAX_ALLOWED_PIDS {
                pids[idx] = entry.pid;
                idx += 1;
            }
        }
        pids
    }
}

// ============================================================================
// Shmem State
// ============================================================================

/// Shared memory subsystem state (protected by SpinLock for SMP safety)
struct ShmemState {
    table: [Option<SharedMem>; MAX_SHMEM_REGIONS],
    mappings: MappingTable,
    next_id: u32,
}

impl ShmemState {
    const fn new() -> Self {
        // Use array initialization that's const-compatible
        const NONE: Option<SharedMem> = None;
        Self {
            table: [NONE; MAX_SHMEM_REGIONS],
            mappings: MappingTable::new(),
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

/// Get a copy of a shared memory region by ID
/// Returns None if the region doesn't exist
pub fn get_region(shmem_id: u32) -> Option<SharedMem> {
    let guard = SHMEM.lock();
    for slot in guard.table.iter() {
        if let Some(region) = slot {
            if region.id == shmem_id {
                return Some(*region);
            }
        }
    }
    None
}

/// Get size of a shared memory region
pub fn get_size(shmem_id: u32) -> Option<usize> {
    get_region(shmem_id).map(|r| r.size)
}

/// Create a new shared memory region
/// Returns (shmem_id, vaddr, paddr) or error
pub fn create(owner_pid: Pid, size: usize) -> Result<(u32, u64, u64), i64> {
    if size == 0 {
        return Err(-22); // EINVAL
    }

    // OVERFLOW CHECK: Align size to page boundary safely
    let aligned_size = size.checked_add(PAGE_SIZE - 1)
        .map(|s| s & !(PAGE_SIZE - 1))
        .ok_or_else(|| {
            kerror!("shmem", "size_overflow"; size = size as u64);
            -22i64 // EINVAL - size overflow
        })?;

    // OVERFLOW CHECK: Calculate number of pages
    let num_pages = aligned_size / PAGE_SIZE;
    if num_pages == 0 || num_pages > 1024 {
        // Sanity limit: max 4MB per region
        return Err(-22); // EINVAL
    }

    // CRITICAL: Find free slot and add mapping BEFORE allocating physical memory
    // This prevents memory leak if no slots or mappings available
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

        // Pre-register owner's mapping in tracking table
        guard.mappings.add_mapping(id, owner_pid)?;

        guard.next_id = guard.next_id.wrapping_add(1);
        if guard.next_id == 0 { guard.next_id = 1; } // Skip 0
        (idx, id)
    };

    // Now allocate contiguous physical pages for DMA
    // If this fails, remove the pre-registered mapping
    let phys_addr = match pmm::alloc_contiguous(num_pages) {
        Some(addr) => addr,
        None => {
            // Rollback: remove the mapping we added
            let mut guard = SHMEM.lock();
            let _ = guard.mappings.remove_mapping(shmem_id, owner_pid);
            return Err(-12); // ENOMEM
        }
    };

    // Create the region descriptor
    // Note: ref_count is now derived from mapping table, not stored here
    let mut region = SharedMem::new();
    region.id = shmem_id;
    region.owner_pid = owner_pid;
    region.phys_addr = phys_addr as u64;
    region.size = aligned_size;
    region.ref_count = 1; // For compatibility - real tracking is in MappingTable

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

        // SECURITY FIX: Check for double-map FIRST via MappingTable
        // This prevents double-map attacks
        if guard.mappings.has_mapping(shmem_id, pid) {
            kwarn!("shmem", "map_already_mapped";
                caller = pid as u64,
                shmem = shmem_id as u64);
            return Err(-17); // EEXIST - already mapped
        }

        // Find the region and validate
        let mut found_info: Option<(u64, usize, bool, Pid, [Pid; MAX_ALLOWED_PIDS])> = None;
        for slot in guard.table.iter() {
            if let Some(ref region) = slot {
                if region.id == shmem_id {
                    found_info = Some((
                        region.phys_addr,
                        region.size,
                        region.state() == RegionState::Dying,
                        region.owner_pid,
                        region.allowed_pids,
                    ));
                    break;
                }
            }
        }

        let (phys, sz, is_dying, owner, allowed) = match found_info {
            Some(info) => info,
            None => {
                kwarn!("shmem", "map_not_found"; caller = pid as u64, shmem = shmem_id as u64);
                return Err(-2); // ENOENT
            }
        };

        // Reject mapping if region is dying (owner exiting)
        if is_dying {
            kwarn!("shmem", "map_dying";
                caller = pid as u64,
                shmem = shmem_id as u64);
            return Err(-11); // EAGAIN - resource temporarily unavailable
        }

        // Check if allowed
        let is_allowed = pid == owner || allowed.iter().any(|&p| p == pid);
        if !is_allowed {
            kwarn!("shmem", "map_denied";
                caller = pid as u64,
                shmem = shmem_id as u64,
                owner = owner as u64,
                allowed0 = allowed[0] as u64,
                allowed1 = allowed[1] as u64);
            return Err(-13); // EACCES
        }

        // Add to tracking table (will fail if full)
        guard.mappings.add_mapping(shmem_id, pid)?;

        // Update ref_count for backward compatibility
        for slot in guard.table.iter_mut() {
            if let Some(ref mut region) = slot {
                if region.id == shmem_id {
                    region.ref_count += 1;
                    break;
                }
            }
        }

        (phys, sz)
    };

    // Map into process's address space
    let vaddr = match map_into_process(pid, phys_addr, size) {
        Ok(addr) => addr,
        Err(e) => {
            // Rollback: remove the mapping we added
            let mut guard = SHMEM.lock();
            let _ = guard.mappings.remove_mapping(shmem_id, pid);
            // Also decrement ref_count
            for slot in guard.table.iter_mut() {
                if let Some(ref mut region) = slot {
                    if region.id == shmem_id && region.ref_count > 0 {
                        region.ref_count -= 1;
                        break;
                    }
                }
            }
            return Err(e);
        }
    };

    Ok((vaddr, phys_addr))
}

/// Unmap shared memory from a process
/// Called when a process unmaps shmem or exits
///
/// SECURITY FIX: Uses MappingTable to track actual mappings.
/// Prevents double-free via ref_count underflow.
pub fn unmap(pid: Pid, shmem_id: u32) -> Result<(), i64> {
    let mut guard = SHMEM.lock();

    // SECURITY FIX: First verify the mapping actually exists in our tracking table
    // This prevents double-free attacks where unmap is called repeatedly
    guard.mappings.remove_mapping(shmem_id, pid)?;

    for slot in guard.table.iter_mut() {
        if let Some(ref mut region) = slot {
            if region.id == shmem_id {
                // Check if this process has access (defense in depth)
                if !region.is_allowed(pid) {
                    // This shouldn't happen since we verified mapping above
                    // but re-add the mapping to maintain consistency
                    let _ = guard.mappings.add_mapping(shmem_id, pid);
                    return Err(-13); // EACCES
                }

                // Update ref_count for backward compatibility
                // Real authority is the MappingTable which we already updated
                if region.ref_count > 0 {
                    region.ref_count -= 1;
                }

                // Also remove from waiters if waiting
                region.remove_waiter(pid);
                return Ok(());
            }
        }
    }

    // Region not found - re-add the mapping we removed to maintain consistency
    let _ = guard.mappings.add_mapping(shmem_id, pid);
    Err(-2) // ENOENT
}

/// Grant another process permission to map this shared memory
pub fn allow(owner_pid: Pid, shmem_id: u32, peer_pid: Pid) -> Result<(), i64> {
    let mut guard = SHMEM.lock();
    for slot in guard.table.iter_mut() {
        if let Some(ref mut region) = slot {
            if region.id == shmem_id {
                // Reject if region is dying
                if region.state() == RegionState::Dying {
                    return Err(-11); // EAGAIN - resource temporarily unavailable
                }

                // Only owner can grant access
                if region.owner_pid != owner_pid {
                    kwarn!("shmem", "allow_denied_not_owner";
                        caller = owner_pid as u64,
                        actual_owner = region.owner_pid as u64);
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
    // Calculate deadline first, then set state, then notify scheduler
    let deadline_to_notify: Option<u64> = super::task::with_scheduler(|sched| {
        if let Some(task) = sched.current_task_mut() {
            // Set wake timeout if specified
            let timeout = if timeout_ms > 0 {
                // Use unified clock API - convert ms to ns, then to hardware counter deadline
                let timeout_ns = timeout_ms as u64 * 1_000_000;
                Some(crate::platform::current::timer::deadline_ns(timeout_ns))
            } else {
                None // No timeout
            };

            // Set state based on whether we have a timeout (via state machine)
            match timeout {
                Some(deadline) => {
                    let _ = task.set_waiting(
                        super::task::WaitReason::ShmemNotify { shmem_id },
                        deadline,
                    );
                }
                None => {
                    let _ = task.set_sleeping(super::task::SleepReason::EventLoop);
                }
            };
            timeout
        } else {
            None
        }
    });

    // Notify scheduler of deadline for tickless optimization (outside task borrow)
    if let Some(deadline) = deadline_to_notify {
        super::task::with_scheduler(|sched| {
            sched.note_deadline(deadline);
        });
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
    let woken = super::task::with_scheduler(|sched| {
        let mut woken = 0u32;
        for waiter_pid in waiters.iter() {
            if *waiter_pid == NO_PID {
                continue;
            }
            if sched.wake_by_pid(*waiter_pid) {
                woken += 1;
            }
        }
        woken
    });

    Ok(woken)
}

/// Destroy a shared memory region (only owner can do this)
///
/// SECURITY: Refuses to destroy if other processes still have active mappings.
/// Uses MappingTable for authoritative count (not ref_count which can be manipulated).
/// This prevents use-after-free where destroying shared memory while another process
/// is actively using it would cause memory corruption.
pub fn destroy(owner_pid: Pid, shmem_id: u32) -> Result<(), i64> {
    // Extract info and validate under shmem lock
    let (phys_addr, size, slot_idx) = {
        let mut guard = SHMEM.lock();
        let mut found = None;
        for (i, slot) in guard.table.iter().enumerate() {
            if let Some(ref region) = slot {
                if region.id == shmem_id {
                    // Only owner can destroy
                    if region.owner_pid != owner_pid {
                        return Err(-1); // EPERM
                    }

                    // SECURITY FIX: Use MappingTable for authoritative count
                    // This prevents manipulation of ref_count to bypass the check
                    let active_mappings = guard.mappings.mapping_count(shmem_id);

                    // mapping_count of 1 means only the owner has it mapped (safe to destroy)
                    // mapping_count > 1 means other processes still have active mappings
                    if active_mappings > 1 {
                        kwarn!("shmem", "destroy_refused_busy";
                            id = shmem_id as u64,
                            mappings = active_mappings as u64);
                        return Err(-16); // EBUSY - resource busy
                    }

                    found = Some((region.phys_addr, region.size, i));
                    break;
                }
            }
        }

        // If we found the region, remove owner's mapping from table
        if found.is_some() {
            let _ = guard.mappings.remove_mapping(shmem_id, owner_pid);
        }

        found.ok_or(-2i64)? // ENOENT
    };
    // shmem lock released here before touching scheduler

    // Safe to unmap - only owner has it mapped
    unmap_from_all_processes(phys_addr, size);

    // Clear slot and free physical memory
    {
        let mut guard = SHMEM.lock();
        guard.table[slot_idx] = None;
        // Also clean up any remaining mappings for this region (defense in depth)
        guard.mappings.remove_all_for_region(shmem_id);
    }

    let num_pages = size / 4096;
    pmm::free_contiguous(phys_addr as usize, num_pages);

    Ok(())
}

/// Unmap a physical address range from all processes
/// Used when destroying shared memory to prevent use-after-free
fn unmap_from_all_processes(phys_addr: u64, size: usize) {
    super::task::with_scheduler(|sched| {
        for (_slot, task_opt) in sched.iter_tasks_mut() {
            if let Some(task) = task_opt {
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
    });
}

/// Map physical memory into a process's address space for DMA
/// Uses non-cacheable device memory attributes (nGnRnE) because shmem
/// is primarily used for DMA rings where CPU cache coherency with
/// PCIe/USB devices is critical.
fn map_into_process(pid: Pid, phys_addr: u64, size: usize) -> Result<u64, i64> {
    super::task::with_scheduler(|sched| {
        // Find the task
        if let Some(slot) = sched.slot_by_pid(pid) {
            if let Some(task) = sched.task_mut(slot) {
                // Use mmap_shmem_dma: non-cacheable for DMA coherency,
                // but BorrowedShmem kind so pages are managed by shmem subsystem
                return task.mmap_shmem_dma(phys_addr, size)
                    .ok_or(-12i64); // ENOMEM
            }
        }
        Err(-3) // ESRCH - no such process
    })
}

/// Phase 1: Begin cleanup - mark regions as Dying and notify mappers
/// This gives servers a chance to release their mappings gracefully.
/// Call finalize_cleanup() later to actually free the memory.
pub fn begin_cleanup(pid: Pid) {
    // Collect regions owned by this process and pids to notify
    let mut to_notify: [(u32, Pid); MAX_SHMEM_REGIONS * MAX_ALLOWED_PIDS] = [(0, 0); MAX_SHMEM_REGIONS * MAX_ALLOWED_PIDS];
    let mut notify_count = 0;

    // Collect shmem IDs owned by this pid first (to get mappers from table)
    let mut owned_ids: [u32; MAX_SHMEM_REGIONS] = [0; MAX_SHMEM_REGIONS];
    let mut owned_count = 0;

    {
        let mut guard = SHMEM.lock();

        // First pass: collect owned region IDs and get their mapped PIDs
        for slot in guard.table.iter() {
            if let Some(ref region) = slot {
                if region.owner_pid == pid && region.state() == RegionState::Active {
                    let shmem_id = region.id;
                    if owned_count < MAX_SHMEM_REGIONS {
                        owned_ids[owned_count] = shmem_id;
                        owned_count += 1;
                    }

                    // Get actual mapped PIDs from MappingTable
                    let mapped_pids = guard.mappings.get_mapped_pids(shmem_id);
                    for mapper_pid in mapped_pids {
                        if mapper_pid != NO_PID && mapper_pid != pid {
                            if notify_count < to_notify.len() {
                                to_notify[notify_count] = (shmem_id, mapper_pid);
                                notify_count += 1;
                            }
                        }
                    }
                }
            }
        }

        // SECURITY FIX: Remove all mappings for this pid from the tracking table
        // This happens regardless of whether they're owner or just mapper
        guard.mappings.remove_all_for_pid(pid);

        // Second pass: mark regions as dying and update ref_counts
        for slot in guard.table.iter_mut() {
            if let Some(ref mut region) = slot {
                // Remove dying pid from waiters
                region.remove_waiter(pid);

                // If this process had it mapped (non-owner), decrement ref_count for compat
                if region.is_allowed(pid) && region.owner_pid != pid {
                    if region.ref_count > 0 {
                        region.ref_count -= 1;
                    }
                }

                // If owner, mark as Dying
                if region.owner_pid == pid && region.state() == RegionState::Active {
                    let _ = region.begin_dying();
                }
            }
        }
    }
    // shmem lock released before touching scheduler/events

    // Send ShmemInvalid events to all mappers (outside lock)
    for i in 0..notify_count {
        let (shmem_id, mapper_pid) = to_notify[i];
        let event = super::event::Event::shmem_invalid(shmem_id, pid);
        let _ = super::event::deliver_event_to_task(mapper_pid, event);
    }
}

/// Phase 2: Finalize cleanup - force unmap and free memory
/// Called after servers had a chance to release their mappings.
pub fn finalize_cleanup(pid: Pid) {
    // Collect Dying regions owned by this process
    let mut owned_regions: [(u64, usize, usize, u32); MAX_SHMEM_REGIONS] = [(0, 0, 0, 0); MAX_SHMEM_REGIONS];
    let mut owned_count = 0;

    {
        let guard = SHMEM.lock();
        for (i, slot) in guard.table.iter().enumerate() {
            if let Some(ref region) = slot {
                // Collect Dying regions owned by this pid
                if region.owner_pid == pid && region.state() == RegionState::Dying {
                    owned_regions[owned_count] = (region.phys_addr, region.size, i, region.id);
                    owned_count += 1;
                }
            }
        }
    }

    // Destroy owned regions (unmap from other processes)
    for i in 0..owned_count {
        let (phys_addr, size, slot_idx, shmem_id) = owned_regions[i];
        let num_pages = size / 4096;

        // CRITICAL: Unmap from ALL other processes before freeing
        unmap_from_all_processes(phys_addr, size);

        // Clear the slot and clean up all mappings for this region
        {
            let mut guard = SHMEM.lock();
            guard.table[slot_idx] = None;
            // SECURITY FIX: Remove all mapping entries for this region
            guard.mappings.remove_all_for_region(shmem_id);
        }

        pmm::free_contiguous(phys_addr as usize, num_pages);
    }
}

/// Legacy single-phase cleanup (for backwards compatibility)
/// Calls both phases immediately - use begin_cleanup + finalize_cleanup for graceful cleanup
pub fn process_cleanup(pid: Pid) {
    begin_cleanup(pid);
    finalize_cleanup(pid);
}

// ============================================================================
// Trait Boundary Helpers
// ============================================================================
//
// These functions provide a stable interface for the trait implementations
// in shmem_impl.rs. They delegate to the internal MappingTable.

/// Add a mapping for (region_id, pid) - for trait boundary
pub fn mapping_add(region_id: u32, pid: Pid) -> Result<(), i64> {
    let mut guard = SHMEM.lock();
    guard.mappings.add_mapping(region_id, pid)
}

/// Remove a mapping for (region_id, pid) - for trait boundary
pub fn mapping_remove(region_id: u32, pid: Pid) -> Result<(), i64> {
    let mut guard = SHMEM.lock();
    guard.mappings.remove_mapping(region_id, pid)
}

/// Check if pid has region mapped - for trait boundary
pub fn mapping_has(region_id: u32, pid: Pid) -> bool {
    let guard = SHMEM.lock();
    guard.mappings.has_mapping(region_id, pid)
}

/// Get count of mappings for a region - for trait boundary
pub fn mapping_count(region_id: u32) -> usize {
    let guard = SHMEM.lock();
    guard.mappings.mapping_count(region_id)
}

/// Get all PIDs that have a region mapped - for trait boundary
pub fn mapping_get_pids(region_id: u32) -> [Pid; MAX_ALLOWED_PIDS] {
    let guard = SHMEM.lock();
    guard.mappings.get_mapped_pids(region_id)
}

/// Remove all mappings for a pid - for trait boundary
pub fn mapping_remove_all_for_pid(pid: Pid) {
    let mut guard = SHMEM.lock();
    guard.mappings.remove_all_for_pid(pid);
}

/// Remove all mappings for a region - for trait boundary
pub fn mapping_remove_all_for_region(region_id: u32) {
    let mut guard = SHMEM.lock();
    guard.mappings.remove_all_for_region(region_id);
}

/// Transition a region to Dying state - for trait boundary
pub fn begin_region_dying(region_id: u32) -> Result<(), i64> {
    let mut guard = SHMEM.lock();
    for slot in guard.table.iter_mut() {
        if let Some(ref mut region) = slot {
            if region.id == region_id {
                if region.begin_dying() {
                    return Ok(());
                } else {
                    return Err(-22); // EINVAL - invalid state transition
                }
            }
        }
    }
    Err(-2) // ENOENT
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
