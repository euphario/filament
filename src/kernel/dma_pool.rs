//! DMA Pool - Reserved Low Memory for PCIe DMA
//!
//! Some PCIe devices (like MT7996 WiFi) may have issues accessing higher
//! memory addresses. This module reserves a chunk of low DRAM at boot
//! for DMA descriptor rings and buffers.
//!
//! ## Memory Layout
//!
//! ```text
//! 0x40000000  DRAM_BASE
//! 0x40100000  DMA_POOL_BASE (1MB into DRAM)
//!             │ DMA Pool (2MB)                │
//! 0x40300000  DMA_POOL_END
//!             │ ... other memory ...          │
//! 0x43000000  ARM TF reserved
//! ```
//!
//! ## Usage
//!
//! Userspace drivers call `shmem_create_dma()` syscall to allocate from
//! this pool instead of regular shmem.

use crate::logln;
use crate::arch::aarch64::mmu;
use super::lock::SpinLock;
use super::process::Pid;

/// DMA pool base address (1MB into DRAM, avoiding any U-Boot stuff at start)
pub const DMA_POOL_BASE: u64 = 0x4010_0000;

/// DMA pool size (2MB - enough for MT7996 descriptors + buffers)
pub const DMA_POOL_SIZE: usize = 2 * 1024 * 1024;

/// DMA pool end address
pub const DMA_POOL_END: u64 = DMA_POOL_BASE + DMA_POOL_SIZE as u64;

/// Page size for alignment
const PAGE_SIZE: usize = 4096;

/// DMA pool state
struct DmaPool {
    /// Next allocation offset (bump allocator)
    next_offset: usize,
    /// Whether pool has been initialized
    initialized: bool,
}

impl DmaPool {
    const fn new() -> Self {
        Self {
            next_offset: 0,
            initialized: false,
        }
    }
}

/// Global DMA pool (protected by spinlock)
static DMA_POOL: SpinLock<DmaPool> = SpinLock::new(DmaPool::new());

/// Initialize the DMA pool
///
/// Called during kernel boot to:
/// 1. Verify pool region is accessible
/// 2. Zero the pool memory
///
/// Note: The kernel already has identity mapping for all DRAM from boot.S,
/// so we don't need to create a new mapping. The pool is in low DRAM
/// (0x40100000) which is below the kernel load address (0x46000000).
pub fn init() {
    let mut pool = DMA_POOL.lock();

    if pool.initialized {
        logln!("[DMA Pool] Already initialized");
        return;
    }

    logln!("[DMA Pool] Reserving low memory for PCIe DMA");
    logln!("  Base: 0x{:08x}", DMA_POOL_BASE);
    logln!("  Size: {} KB", DMA_POOL_SIZE / 1024);
    logln!("  End:  0x{:08x}", DMA_POOL_END);

    // The kernel's boot.S sets up identity mapping for all DRAM,
    // so we can access the pool directly at its physical address.
    // Convert to kernel virtual address for access.
    let pool_virt = mmu::phys_to_virt(DMA_POOL_BASE);

    // Zero the pool memory
    unsafe {
        core::ptr::write_bytes(pool_virt as *mut u8, 0, DMA_POOL_SIZE);
    }

    pool.initialized = true;
    logln!("[DMA Pool] Initialized successfully");
}

/// Allocate memory from DMA pool
///
/// Returns (physical_address, size) on success, or error code on failure.
/// Memory is page-aligned and zeroed.
pub fn alloc(size: usize) -> Result<u64, i64> {
    if size == 0 {
        return Err(-1); // EINVAL
    }

    // Round up to page size
    let aligned_size = (size + PAGE_SIZE - 1) & !(PAGE_SIZE - 1);

    let mut pool = DMA_POOL.lock();

    if !pool.initialized {
        logln!("[DMA Pool] ERROR: Pool not initialized");
        return Err(-2); // ENODEV
    }

    // Check if we have enough space
    if pool.next_offset + aligned_size > DMA_POOL_SIZE {
        logln!("[DMA Pool] ERROR: Out of memory (requested={}, available={})",
               aligned_size, DMA_POOL_SIZE - pool.next_offset);
        return Err(-3); // ENOMEM
    }

    let phys_addr = DMA_POOL_BASE + pool.next_offset as u64;
    pool.next_offset += aligned_size;

    logln!("[DMA Pool] Allocated {} bytes at 0x{:08x} (pool usage: {}/{})",
           aligned_size, phys_addr, pool.next_offset, DMA_POOL_SIZE);

    Ok(phys_addr)
}

/// Map DMA pool memory into a process's address space
///
/// The physical memory is already allocated from the pool.
/// This creates a virtual mapping in the process using device memory attributes.
pub fn map_into_process(pid: Pid, phys_addr: u64, size: usize) -> Result<u64, i64> {
    // Verify address is within pool
    if phys_addr < DMA_POOL_BASE || phys_addr + size as u64 > DMA_POOL_END {
        logln!("[DMA Pool] ERROR: Address 0x{:x} not in pool", phys_addr);
        return Err(-1);
    }

    unsafe {
        let sched = super::task::scheduler();

        // Find the task and map the memory
        for task_opt in sched.tasks.iter_mut() {
            if let Some(ref mut task) = task_opt {
                if task.id == pid {
                    // Use mmap_shmem_dma: device memory attributes for DMA coherency
                    // BorrowedShmem kind means pages won't be freed when unmapped
                    // (they're owned by the DMA pool)
                    return task.mmap_shmem_dma(phys_addr, size)
                        .ok_or(-12i64); // ENOMEM
                }
            }
        }
    }
    Err(-3) // ESRCH - no such process
}

/// Get pool statistics
pub fn stats() -> (usize, usize) {
    let pool = DMA_POOL.lock();
    (pool.next_offset, DMA_POOL_SIZE)
}
