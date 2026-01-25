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

use crate::{kinfo, kerror, klog};
use crate::arch::aarch64::mmu;
use crate::platform::current;
use super::lock::SpinLock;
use super::process::Pid;

/// DMA pool base address (platform-specific)
/// NOTE: Descriptor rings MUST be below 4GB (DESC_BASE is 32-bit register)
/// NOTE: Must not overlap with kernel image
pub const DMA_POOL_BASE: u64 = current::DMA_POOL_BASE;

/// DMA pool size (platform-specific)
pub const DMA_POOL_SIZE: usize = current::DMA_POOL_SIZE;

/// DMA pool end address
pub const DMA_POOL_END: u64 = DMA_POOL_BASE + DMA_POOL_SIZE as u64;

/// High DMA pool base address (platform-specific, for 36-bit buffer addresses)
pub const DMA_POOL_HIGH_BASE: u64 = current::DMA_POOL_HIGH_BASE;

/// High DMA pool size (platform-specific)
pub const DMA_POOL_HIGH_SIZE: usize = current::DMA_POOL_HIGH_SIZE;

/// High DMA pool end address
pub const DMA_POOL_HIGH_END: u64 = DMA_POOL_HIGH_BASE + DMA_POOL_HIGH_SIZE as u64;

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

/// Global DMA pool (protected by spinlock) - for descriptors (< 4GB)
static DMA_POOL: SpinLock<DmaPool> = SpinLock::new(DmaPool::new());

/// Global high DMA pool (protected by spinlock) - for buffers (> 4GB, 36-bit)
static DMA_POOL_HIGH: SpinLock<DmaPool> = SpinLock::new(DmaPool::new());

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
        return;
    }

    kinfo!("dma_pool", "init_start";
        base = klog::hex64(DMA_POOL_BASE),
        size_kb = (DMA_POOL_SIZE / 1024) as u64,
        end = klog::hex64(DMA_POOL_END)
    );

    // The kernel's boot.S sets up identity mapping for all DRAM,
    // so we can access the pool directly at its physical address.
    // Convert to kernel virtual address for access.
    let pool_virt = mmu::phys_to_virt(DMA_POOL_BASE);

    // Zero the pool memory
    unsafe {
        core::ptr::write_bytes(pool_virt as *mut u8, 0, DMA_POOL_SIZE);
    }

    // CRITICAL: Flush the cache to RAM!
    // The kernel maps DRAM as cacheable, but userspace DMA mappings use
    // non-cacheable (NORMAL_NC). Without this flush, the zeros sit in
    // kernel cache and could overwrite userspace descriptor writes when
    // cache lines are evicted later.
    const CACHE_LINE: u64 = 64;
    let end = pool_virt + DMA_POOL_SIZE as u64;
    let mut addr = pool_virt;
    while addr < end {
        unsafe {
            // DC CIVAC: Clean and Invalidate by VA to Point of Coherency
            // This writes dirty data to RAM and invalidates the cache line,
            // so future kernel accesses will re-read from RAM.
            core::arch::asm!("dc civac, {}", in(reg) addr, options(nostack, preserves_flags));
        }
        addr += CACHE_LINE;
    }
    // DSB ensures all cache maintenance operations complete
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }

    pool.initialized = true;
    kinfo!("dma_pool", "init_ok");
}

/// Initialize the high DMA pool (for 36-bit buffer addresses)
///
/// Called during kernel boot after init().
/// NOTE: We don't zero this memory from kernel space because the boot page tables
/// only cover lower memory. The memory will be zeroed when mapped into userspace.
pub fn init_high() {
    let mut pool = DMA_POOL_HIGH.lock();

    if pool.initialized {
        return;
    }

    kinfo!("dma_pool", "init_high_start";
        base = klog::hex64(DMA_POOL_HIGH_BASE),
        size_kb = (DMA_POOL_HIGH_SIZE / 1024) as u64,
        end = klog::hex64(DMA_POOL_HIGH_END)
    );

    pool.initialized = true;
    kinfo!("dma_pool", "init_high_ok");
}

/// Allocate memory from high DMA pool (36-bit addresses)
///
/// Returns physical_address on success, or error code on failure.
/// Memory is page-aligned and zeroed.
pub fn alloc_high(size: usize) -> Result<u64, i64> {
    if size == 0 {
        return Err(-1);
    }

    // OVERFLOW CHECK: Align size safely
    let aligned_size = size.checked_add(PAGE_SIZE - 1)
        .map(|s| s & !(PAGE_SIZE - 1))
        .ok_or_else(|| {
            kerror!("dma_pool", "alloc_high_size_overflow"; size = size as u64);
            -22i64 // EINVAL - size overflow
        })?;

    let mut pool = DMA_POOL_HIGH.lock();

    if !pool.initialized {
        kerror!("dma_pool", "alloc_high_not_init");
        return Err(-2);
    }

    // OVERFLOW CHECK: Verify addition doesn't overflow
    let new_offset = pool.next_offset.checked_add(aligned_size).ok_or_else(|| {
        kerror!("dma_pool", "alloc_high_offset_overflow");
        -22i64 // EINVAL
    })?;

    if new_offset > DMA_POOL_HIGH_SIZE {
        kerror!("dma_pool", "alloc_high_oom"; requested = aligned_size as u64, available = (DMA_POOL_HIGH_SIZE - pool.next_offset) as u64);
        return Err(-3);
    }

    let phys_addr = DMA_POOL_HIGH_BASE + pool.next_offset as u64;
    pool.next_offset = new_offset;

    kinfo!("dma_pool", "alloc_high_ok";
        size = aligned_size as u64,
        addr = klog::hex64(phys_addr),
        usage = pool.next_offset as u64,
        total = DMA_POOL_HIGH_SIZE as u64
    );

    Ok(phys_addr)
}

/// Allocate memory from DMA pool
///
/// Returns (physical_address, size) on success, or error code on failure.
/// Memory is page-aligned and zeroed.
pub fn alloc(size: usize) -> Result<u64, i64> {
    if size == 0 {
        return Err(-1); // EINVAL
    }

    // OVERFLOW CHECK: Round up to page size safely
    let aligned_size = size.checked_add(PAGE_SIZE - 1)
        .map(|s| s & !(PAGE_SIZE - 1))
        .ok_or_else(|| {
            kerror!("dma_pool", "alloc_size_overflow"; size = size as u64);
            -22i64 // EINVAL - size overflow
        })?;

    let mut pool = DMA_POOL.lock();

    if !pool.initialized {
        kerror!("dma_pool", "alloc_not_init");
        return Err(-2); // ENODEV
    }

    // OVERFLOW CHECK: Verify addition doesn't overflow
    let new_offset = pool.next_offset.checked_add(aligned_size).ok_or_else(|| {
        kerror!("dma_pool", "alloc_offset_overflow");
        -22i64 // EINVAL
    })?;

    // Check if we have enough space
    if new_offset > DMA_POOL_SIZE {
        kerror!("dma_pool", "alloc_oom"; requested = aligned_size as u64, available = (DMA_POOL_SIZE - pool.next_offset) as u64);
        return Err(-3); // ENOMEM
    }

    let phys_addr = DMA_POOL_BASE + pool.next_offset as u64;
    pool.next_offset = new_offset;

    kinfo!("dma_pool", "alloc_ok";
        size = aligned_size as u64,
        addr = klog::hex64(phys_addr),
        usage = pool.next_offset as u64,
        total = DMA_POOL_SIZE as u64
    );

    Ok(phys_addr)
}

/// Map DMA pool memory into a process's address space
///
/// The physical memory is already allocated from the pool.
/// This creates a virtual mapping in the process using device memory attributes.
pub fn map_into_process(pid: Pid, phys_addr: u64, size: usize) -> Result<u64, i64> {
    // OVERFLOW CHECK: Verify address bounds safely
    let end_addr = phys_addr.checked_add(size as u64).ok_or_else(|| {
        kerror!("dma_pool", "map_addr_overflow"; addr = klog::hex64(phys_addr), size = size as u64);
        -22i64 // EINVAL
    })?;

    // Verify address is within pool
    if phys_addr < DMA_POOL_BASE || end_addr > DMA_POOL_END {
        kerror!("dma_pool", "map_addr_invalid"; addr = klog::hex64(phys_addr));
        return Err(-1);
    }

    super::task::with_scheduler(|sched| {
        // Find the task and map the memory
        if let Some(slot) = sched.slot_by_pid(pid) {
            if let Some(task) = sched.task_mut(slot) {
                // Use mmap_shmem_dma: device memory attributes for DMA coherency
                // BorrowedShmem kind means pages won't be freed when unmapped
                // (they're owned by the DMA pool)
                return task.mmap_shmem_dma(phys_addr, size)
                    .ok_or(-12i64); // ENOMEM
            }
        }
        Err(-3) // ESRCH - no such process
    })
}

/// Map high DMA pool memory into a process's address space (36-bit addresses)
/// Uses special mapping that doesn't try to access memory from kernel space
pub fn map_into_process_high(pid: Pid, phys_addr: u64, size: usize) -> Result<u64, i64> {
    // OVERFLOW CHECK: Verify address bounds safely
    let end_addr = phys_addr.checked_add(size as u64).ok_or_else(|| {
        kerror!("dma_pool", "map_high_addr_overflow"; addr = klog::hex64(phys_addr), size = size as u64);
        -22i64 // EINVAL
    })?;

    // Verify address is within high pool
    if phys_addr < DMA_POOL_HIGH_BASE || end_addr > DMA_POOL_HIGH_END {
        kerror!("dma_pool", "map_high_addr_invalid"; addr = klog::hex64(phys_addr));
        return Err(-1);
    }

    super::task::with_scheduler(|sched| {
        if let Some(slot) = sched.slot_by_pid(pid) {
            if let Some(task) = sched.task_mut(slot) {
                // Use dma_high mapping - no kernel-side zeroing/flush
                return task.mmap_shmem_dma_high(phys_addr, size)
                    .ok_or(-12i64);
            }
        }
        Err(-3)
    })
}

/// Get pool statistics
pub fn stats() -> (usize, usize) {
    let pool = DMA_POOL.lock();
    (pool.next_offset, DMA_POOL_SIZE)
}

/// Get high pool statistics
pub fn stats_high() -> (usize, usize) {
    let pool = DMA_POOL_HIGH.lock();
    (pool.next_offset, DMA_POOL_HIGH_SIZE)
}

// ============================================================================
// Self-tests
// ============================================================================

#[cfg(feature = "selftest")]
pub fn test() {
    use crate::{kdebug, kinfo};

    kdebug!("dma_pool", "test_start");

    // Test 1: Pool constants are valid
    {
        assert!(DMA_POOL_BASE >= 0x4000_0000, "DMA_POOL_BASE should be in DRAM");
        assert!(DMA_POOL_BASE < 0x1_0000_0000, "DMA_POOL_BASE must be < 4GB for 32-bit DESC_BASE");
        assert!(DMA_POOL_SIZE > 0, "DMA_POOL_SIZE must be positive");
        assert_eq!(DMA_POOL_SIZE % PAGE_SIZE, 0, "DMA_POOL_SIZE must be page-aligned");
        assert!(DMA_POOL_END == DMA_POOL_BASE + DMA_POOL_SIZE as u64, "DMA_POOL_END calculation");
        kdebug!("dma_pool", "constants_ok");
    }

    // Test 2: High pool constants are valid
    {
        assert!(DMA_POOL_HIGH_BASE >= 0x1_0000_0000, "DMA_POOL_HIGH_BASE should be > 4GB");
        assert!(DMA_POOL_HIGH_SIZE > 0, "DMA_POOL_HIGH_SIZE must be positive");
        assert_eq!(DMA_POOL_HIGH_SIZE % PAGE_SIZE, 0, "DMA_POOL_HIGH_SIZE must be page-aligned");
        kdebug!("dma_pool", "high_constants_ok");
    }

    // Test 3: Pool initialized
    {
        let pool = DMA_POOL.lock();
        assert!(pool.initialized, "DMA pool should be initialized");
        drop(pool);
        kdebug!("dma_pool", "init_ok");
    }

    // Test 4: Allocation returns valid address
    {
        let (used_before, _) = stats();
        let result = alloc(PAGE_SIZE);
        assert!(result.is_ok(), "alloc should succeed");
        let addr = result.unwrap();
        assert!(addr >= DMA_POOL_BASE, "Allocated address below pool base");
        assert!(addr < DMA_POOL_END, "Allocated address above pool end");
        assert_eq!(addr % PAGE_SIZE as u64, 0, "Allocated address not page-aligned");

        let (used_after, _) = stats();
        assert_eq!(used_after, used_before + PAGE_SIZE, "Pool usage should increase by PAGE_SIZE");
        kdebug!("dma_pool", "alloc_ok"; addr = crate::klog::hex64(addr));
    }

    // Test 5: Zero-size allocation fails
    {
        let result = alloc(0);
        assert!(result.is_err(), "Zero-size alloc should fail");
        kdebug!("dma_pool", "zero_alloc_rejected");
    }

    // Test 6: High pool allocation
    {
        let pool = DMA_POOL_HIGH.lock();
        if pool.initialized {
            drop(pool);
            let result = alloc_high(PAGE_SIZE);
            assert!(result.is_ok(), "alloc_high should succeed");
            let addr = result.unwrap();
            assert!(addr >= DMA_POOL_HIGH_BASE, "High alloc below base");
            assert!(addr < DMA_POOL_HIGH_END, "High alloc above end");
            kdebug!("dma_pool", "alloc_high_ok"; addr = crate::klog::hex64(addr));
        } else {
            drop(pool);
            kdebug!("dma_pool", "high_pool_not_init_skipped");
        }
    }

    // Test 7: Stats function works
    {
        let (used, total) = stats();
        assert!(used <= total, "Used should not exceed total");
        assert_eq!(total, DMA_POOL_SIZE, "Total should match DMA_POOL_SIZE");
        kdebug!("dma_pool", "stats_ok"; used = used as u64, total = total as u64);
    }

    kinfo!("dma_pool", "test_ok");
}
