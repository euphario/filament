//! Per-CPU Data Infrastructure
//!
//! Provides per-CPU storage for data that should not be shared between CPUs.
//! This eliminates cache-line bouncing and the need for locks on per-CPU data.
//!
//! ## Design
//!
//! - Each CPU has its own `CpuData` structure
//! - TPIDR_EL1 register points to the current CPU's data
//! - Access is via `cpu_local()` which reads TPIDR_EL1
//!
//! ## Current Implementation
//!
//! This is a single-core implementation that can be extended to SMP.
//! For now, we have a single static CpuData and cpu_local() just returns it.
//!
//! ## Future SMP Extension
//!
//! 1. Allocate one CpuData per core during boot
//! 2. Set TPIDR_EL1 to point to that core's CpuData
//! 3. cpu_local() reads TPIDR_EL1 to find CpuData

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// Maximum number of CPUs supported
pub const MAX_CPUS: usize = 4;

/// Per-CPU data structure
///
/// This structure is instantiated once per CPU and provides thread-local
/// storage without requiring locks.
///
/// # Safety
///
/// Fields in this structure can be accessed without locks because each CPU
/// has its own copy. However, be careful when storing pointers to shared data.
#[repr(C)]
pub struct CpuData {
    /// CPU ID (0-based)
    pub cpu_id: u32,

    /// Pointer to currently running task (or null if idle)
    /// Updated on context switch.
    pub current_task: AtomicU64,

    /// Scheduler slot index of current task
    pub current_slot: AtomicU32,

    /// Nest count for IRQ disable (for nested critical sections)
    pub irq_disable_count: UnsafeCell<u32>,

    /// Flag: need to reschedule at next safe point
    pub need_resched: AtomicU32,

    /// Timer tick count (for this CPU)
    pub tick_count: AtomicU64,

    /// Idle time counter (ticks spent in idle)
    pub idle_ticks: AtomicU64,

    /// Flag: CPU is currently in idle (WFI)
    pub in_idle: AtomicU32,

    /// Last syscall number (for debugging)
    pub last_syscall: AtomicU32,

    /// Kernel stack for this CPU (used during boot/exceptions)
    pub kernel_stack_top: AtomicU64,
}

impl CpuData {
    /// Create a new CpuData for the given CPU ID.
    pub const fn new(cpu_id: u32) -> Self {
        Self {
            cpu_id,
            current_task: AtomicU64::new(0),
            current_slot: AtomicU32::new(0),
            irq_disable_count: UnsafeCell::new(0),
            need_resched: AtomicU32::new(0),
            tick_count: AtomicU64::new(0),
            idle_ticks: AtomicU64::new(0),
            in_idle: AtomicU32::new(0),
            last_syscall: AtomicU32::new(0),
            kernel_stack_top: AtomicU64::new(0),
        }
    }

    /// Get the current task slot index
    #[inline]
    pub fn get_current_slot(&self) -> usize {
        self.current_slot.load(Ordering::Relaxed) as usize
    }

    /// Set the current task slot index
    #[inline]
    pub fn set_current_slot(&self, slot: usize) {
        self.current_slot.store(slot as u32, Ordering::Relaxed);
    }

    /// Check if reschedule is needed
    #[inline]
    pub fn need_resched(&self) -> bool {
        self.need_resched.load(Ordering::Acquire) != 0
    }

    /// Request a reschedule at the next safe point
    #[inline]
    pub fn set_need_resched(&self) {
        self.need_resched.store(1, Ordering::Release);
    }

    /// Clear the need_resched flag and return if it was set
    #[inline]
    pub fn clear_need_resched(&self) -> bool {
        self.need_resched.swap(0, Ordering::AcqRel) != 0
    }

    /// Increment tick count
    #[inline]
    pub fn tick(&self) {
        self.tick_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Get tick count
    #[inline]
    pub fn ticks(&self) -> u64 {
        self.tick_count.load(Ordering::Relaxed)
    }

    /// Record that we're entering idle
    #[inline]
    pub fn enter_idle(&self) {
        // Will count idle ticks on next tick
    }

    /// Record a tick in idle state
    #[inline]
    pub fn idle_tick(&self) {
        self.idle_ticks.fetch_add(1, Ordering::Relaxed);
    }

    /// Get idle tick count
    #[inline]
    pub fn get_idle_ticks(&self) -> u64 {
        self.idle_ticks.load(Ordering::Relaxed)
    }

    /// Mark CPU as entering idle (before WFI)
    #[inline]
    pub fn set_idle(&self) {
        self.in_idle.store(1, Ordering::Release);
    }

    /// Mark CPU as exiting idle (after WFI, found work)
    #[inline]
    pub fn clear_idle(&self) {
        self.in_idle.store(0, Ordering::Release);
    }

    /// Check if CPU is currently idle
    #[inline]
    pub fn is_idle(&self) -> bool {
        self.in_idle.load(Ordering::Acquire) != 0
    }
}

// SAFETY: CpuData is designed for per-CPU access patterns
unsafe impl Sync for CpuData {}

/// Static array of per-CPU data (for now, single core)
static mut CPU_DATA: [CpuData; MAX_CPUS] = [
    CpuData::new(0),
    CpuData::new(1),
    CpuData::new(2),
    CpuData::new(3),
];

/// Initialize per-CPU infrastructure for the boot CPU.
///
/// Must be called early in boot, before any per-CPU data is accessed.
pub fn init_boot_cpu() {
    let cpu_id = read_cpu_id();

    // Set TPIDR_EL1 to point to this CPU's data
    let cpu_data_ptr = unsafe { &CPU_DATA[cpu_id as usize] as *const CpuData as u64 };

    unsafe {
        core::arch::asm!(
            "msr tpidr_el1, {}",
            in(reg) cpu_data_ptr,
            options(nostack, preserves_flags)
        );
    }

    // Initialize this CPU's data
    unsafe {
        CPU_DATA[cpu_id as usize].cpu_id = cpu_id;
    }
}

/// Initialize per-CPU infrastructure for a secondary CPU.
///
/// Called by secondary CPUs during their boot sequence.
#[allow(dead_code)]
pub fn init_secondary_cpu(cpu_id: u32) {
    if cpu_id as usize >= MAX_CPUS {
        panic!("CPU ID {} exceeds MAX_CPUS {}", cpu_id, MAX_CPUS);
    }

    // Set TPIDR_EL1 to point to this CPU's data
    let cpu_data_ptr = unsafe { &CPU_DATA[cpu_id as usize] as *const CpuData as u64 };

    unsafe {
        core::arch::asm!(
            "msr tpidr_el1, {}",
            in(reg) cpu_data_ptr,
            options(nostack, preserves_flags)
        );

        CPU_DATA[cpu_id as usize].cpu_id = cpu_id;
    }
}

/// Get a reference to the current CPU's data.
///
/// This reads TPIDR_EL1 to find the per-CPU data pointer.
/// Must be called after init_boot_cpu() or init_secondary_cpu().
#[inline]
pub fn cpu_local() -> &'static CpuData {
    let ptr: u64;
    unsafe {
        core::arch::asm!(
            "mrs {}, tpidr_el1",
            out(reg) ptr,
            options(nostack, preserves_flags)
        );
    }

    if ptr == 0 {
        // Fallback for early boot before init_boot_cpu()
        // SAFETY: Single-core access during boot
        unsafe { &CPU_DATA[0] }
    } else {
        // SAFETY: ptr was set by init_*_cpu() to point to a valid CpuData
        unsafe { &*(ptr as *const CpuData) }
    }
}

/// Get the current CPU ID.
///
/// Uses TPIDR_EL1 if available, otherwise reads MPIDR_EL1.
#[inline]
pub fn cpu_id() -> u32 {
    let ptr: u64;
    unsafe {
        core::arch::asm!(
            "mrs {}, tpidr_el1",
            out(reg) ptr,
            options(nostack, preserves_flags)
        );
    }

    if ptr != 0 {
        // Fast path: read from CpuData
        unsafe { (*(ptr as *const CpuData)).cpu_id }
    } else {
        // Slow path: read MPIDR
        read_cpu_id()
    }
}

/// Read CPU ID from MPIDR_EL1 register.
#[inline]
fn read_cpu_id() -> u32 {
    let mpidr: u64;
    unsafe {
        core::arch::asm!(
            "mrs {}, mpidr_el1",
            out(reg) mpidr,
            options(nostack, preserves_flags)
        );
    }
    // Extract Aff0 (CPU ID within cluster)
    // For MT7988A with 4 Cortex-A73 cores, this gives 0-3
    (mpidr & 0xFF) as u32
}

/// Get the number of online CPUs.
///
/// Currently always returns 1 (single-core implementation).
#[inline]
pub fn num_online_cpus() -> u32 {
    1
}

/// Check if we're on the boot CPU.
#[inline]
pub fn is_boot_cpu() -> bool {
    cpu_id() == 0
}

/// Macro for defining per-CPU variables with convenient access syntax.
///
/// Usage:
/// ```
/// define_percpu!(MY_COUNTER: AtomicU32 = AtomicU32::new(0));
///
/// fn increment() {
///     MY_COUNTER.with(|c| c.fetch_add(1, Ordering::Relaxed));
/// }
/// ```
#[macro_export]
macro_rules! define_percpu {
    ($name:ident : $ty:ty = $init:expr) => {
        // For now, just a regular static since we're single-core
        // In SMP, this would be an array indexed by cpu_id()
        static $name: $ty = $init;
    };
}
