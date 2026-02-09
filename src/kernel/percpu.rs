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
//! - Exception handlers (boot.S) access CpuData fields directly via
//!   TPIDR_EL1 + offset, avoiding global statics that aren't SMP-safe

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use super::microtask::{MpscInbox, PerCpuMicroTaskQueue, RingQueue};

/// Cached count of online CPUs. Starts at 1 (boot CPU), incremented by
/// each secondary CPU in `init_secondary_cpu()`.
static ONLINE_CPU_COUNT: AtomicU32 = AtomicU32::new(1);

/// CPU lifecycle state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum CpuState {
    Offline = 0,
    Starting = 1,
    Online = 2,
    Halted = 3,
}

impl CpuState {
    pub fn from_u32(v: u32) -> Self {
        match v {
            0 => CpuState::Offline,
            1 => CpuState::Starting,
            2 => CpuState::Online,
            _ => CpuState::Halted,
        }
    }
}

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

    /// Current task's kernel stack top (virtual address for SP_EL1).
    /// Updated during simple preemption and task entry so assembly
    /// exception return paths can set SP before ERET.
    pub kernel_stack_top: AtomicU64,

    /// Current task's trap frame pointer - used by exception handler (boot.S)
    /// Replaces the former global CURRENT_TRAP_FRAME for SMP safety.
    pub trap_frame_ptr: AtomicU64,

    /// Current task's TTBR0 value - used by exception handler (boot.S)
    /// Replaces the former global CURRENT_TTBR0 for SMP safety.
    pub ttbr0: AtomicU64,

    /// Flag: set to 1 when syscall switched tasks, 0 otherwise
    /// Assembly checks this to skip storing return value when switched.
    /// Replaces the former global SYSCALL_SWITCHED_TASK for SMP safety.
    pub syscall_switched: AtomicU64,

    // --- Fields below are NOT accessed from assembly (offset 88+) ---

    /// CPU lifecycle state (Offline/Starting/Online/Halted)
    pub state: AtomicU32,
    /// Padding for 8-byte alignment
    _pad_state: u32,
    /// Stack top address set during SMP boot
    pub stack_top: AtomicU64,

    /// Slot of task whose kernel_stack_owner should be cleared after context_switch.
    /// Set before context_switch, read after (same CPU, different stack).
    /// 0xFFFFFFFF = no pending release.
    pub pending_stack_release: AtomicU32,
    /// Padding for 8-byte alignment
    _pad_release: u32,

    // --- Per-CPU microtask queue (after all assembly-visible fields) ---

    /// Per-CPU microtask queue. Accessed with IRQs disabled (no SpinLock needed).
    /// Enqueue tries this first, falls back to global overflow queue.
    pub microtask_queue: UnsafeCell<PerCpuMicroTaskQueue>,

    /// Lock-free MPSC inbox for cross-CPU microtask enqueue.
    /// Other CPUs enqueue via CAS; only this CPU dequeues (IRQs disabled).
    pub remote_inbox: UnsafeCell<MpscInbox>,
}

// Assembly offsets into CpuData (must match #[repr(C)] layout above)
// Layout: cpu_id(u32:0) _pad(4) current_task(u64:8) current_slot(u32:16)
//         irq_disable_count(u32:20) need_resched(u32:24) _pad(4)
//         tick_count(u64:32) idle_ticks(u64:40) in_idle(u32:48)
//         last_syscall(u32:52) kernel_stack_top(u64:56)
//         trap_frame_ptr(u64:64) ttbr0(u64:72) syscall_switched(u64:80)
pub const CPUDATA_OFFSET_KSTACK_TOP: usize = 56;
pub const CPUDATA_OFFSET_TRAP_FRAME: usize = 64;
pub const CPUDATA_OFFSET_TTBR0: usize = 72;
pub const CPUDATA_OFFSET_SWITCHED: usize = 80;

// Compile-time verification that assembly offsets match actual layout.
// If any field is added/reordered, these will fail to compile.
const _: () = {
    assert!(core::mem::offset_of!(CpuData, kernel_stack_top) == CPUDATA_OFFSET_KSTACK_TOP);
    assert!(core::mem::offset_of!(CpuData, trap_frame_ptr) == CPUDATA_OFFSET_TRAP_FRAME);
    assert!(core::mem::offset_of!(CpuData, ttbr0) == CPUDATA_OFFSET_TTBR0);
    assert!(core::mem::offset_of!(CpuData, syscall_switched) == CPUDATA_OFFSET_SWITCHED);
    // New fields are after assembly-visible region — verify they don't overlap
    assert!(core::mem::offset_of!(CpuData, state) == 88);
    assert!(core::mem::offset_of!(CpuData, stack_top) == 96);
};

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
            trap_frame_ptr: AtomicU64::new(0),
            ttbr0: AtomicU64::new(0),
            syscall_switched: AtomicU64::new(0),
            state: AtomicU32::new(CpuState::Offline as u32),
            _pad_state: 0,
            stack_top: AtomicU64::new(0),
            pending_stack_release: AtomicU32::new(0xFFFFFFFF),
            _pad_release: 0,
            microtask_queue: UnsafeCell::new(RingQueue::new()),
            remote_inbox: UnsafeCell::new(MpscInbox::new()),
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

    /// Get CPU lifecycle state
    #[inline]
    pub fn get_state(&self) -> CpuState {
        CpuState::from_u32(self.state.load(Ordering::Acquire))
    }

    /// Set CPU lifecycle state
    #[inline]
    pub fn set_state(&self, state: CpuState) {
        self.state.store(state as u32, Ordering::Release);
    }

    /// Get stack top address
    #[inline]
    pub fn get_stack_top(&self) -> u64 {
        self.stack_top.load(Ordering::Relaxed)
    }

    /// Set stack top address
    #[inline]
    pub fn set_stack_top(&self, addr: u64) {
        self.stack_top.store(addr, Ordering::Relaxed);
    }

    /// Set the pending stack release slot.
    /// Called before context_switch to record which task needs finalization
    /// after context_switch completes (transition to Ready or clear kernel_stack_owner).
    #[inline]
    pub fn set_pending_release(&self, slot: u32) {
        self.pending_stack_release.store(slot, Ordering::Release);
    }

    /// Take the pending stack release slot, resetting to 0xFFFFFFFF (none).
    /// Called after context_switch to retrieve the from-task slot for finalization.
    #[inline]
    pub fn take_pending_release(&self) -> u32 {
        self.pending_stack_release.swap(0xFFFFFFFF, Ordering::AcqRel)
    }
}

// ============================================================================
// Per-CPU Trap Frame / TTBR0 / Switched Accessors
// ============================================================================
// These replace the former global statics CURRENT_TRAP_FRAME, CURRENT_TTBR0,
// and SYSCALL_SWITCHED_TASK for SMP safety.

/// Set the current task's kernel stack top (per-CPU).
/// Used by assembly exception return paths to set SP_EL1 correctly
/// when simple preemption switches to a different task.
#[inline]
pub fn set_kernel_stack_top(addr: u64) {
    #[cfg(debug_assertions)]
    {
        // Must be in kernel VA range (high half) and 16-byte aligned for ARM stack
        debug_assert!(
            addr >= 0xFFFF_0000_0000_0000 || addr == 0,
            "set_kernel_stack_top: 0x{:x} not in kernel VA range", addr
        );
        debug_assert!(
            addr & 0xF == 0,
            "set_kernel_stack_top: 0x{:x} not 16-byte aligned", addr
        );
    }
    cpu_local().kernel_stack_top.store(addr, Ordering::Release);
}

/// Set the current task's trap frame pointer (per-CPU)
#[inline]
pub fn set_trap_frame(ptr: *mut super::task::TrapFrame) {
    cpu_local().trap_frame_ptr.store(ptr as u64, Ordering::Release);
}

/// Get the current task's trap frame pointer (per-CPU)
#[inline]
pub fn get_trap_frame() -> *mut super::task::TrapFrame {
    cpu_local().trap_frame_ptr.load(Ordering::Acquire) as *mut _
}

/// Set the current task's TTBR0 value (per-CPU)
#[inline]
pub fn set_ttbr0(val: u64) {
    cpu_local().ttbr0.store(val, Ordering::Release);
}

/// Get the current task's TTBR0 value (per-CPU)
#[inline]
pub fn get_ttbr0() -> u64 {
    cpu_local().ttbr0.load(Ordering::Acquire)
}

/// Set the syscall-switched flag (per-CPU)
#[inline]
pub fn set_syscall_switched(val: u64) {
    cpu_local().syscall_switched.store(val, Ordering::Release);
}

/// Get the syscall-switched flag (per-CPU)
#[inline]
pub fn get_syscall_switched() -> u64 {
    cpu_local().syscall_switched.load(Ordering::Acquire)
}

/// Set the pending stack release slot (per-CPU).
/// Records which from-task needs finalization after context_switch.
#[inline]
pub fn set_pending_release(slot: u32) {
    cpu_local().set_pending_release(slot);
}

/// Take the pending stack release slot (per-CPU), resetting to none.
/// Returns 0xFFFFFFFF if no pending release.
#[inline]
pub fn take_pending_release() -> u32 {
    cpu_local().take_pending_release()
}

// SAFETY: CpuData is designed for per-CPU access patterns
unsafe impl Sync for CpuData {}

/// Static array of per-CPU data
pub static mut CPU_DATA: [CpuData; MAX_CPUS] = [
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

    // Increment cached online CPU count
    ONLINE_CPU_COUNT.fetch_add(1, Ordering::Release);
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

/// Get the number of online CPUs (cached, O(1)).
#[inline]
pub fn num_online_cpus() -> u32 {
    ONLINE_CPU_COUNT.load(Ordering::Acquire)
}

/// Get a reference to a specific CPU's data, returning `None` if
/// `cpu_id` is out of bounds.
#[inline]
pub fn try_get_cpu_data(cpu_id: usize) -> Option<&'static CpuData> {
    if cpu_id >= MAX_CPUS {
        return None;
    }
    // SAFETY: CPU_DATA is initialized at boot; index is bounds-checked above.
    Some(unsafe { &CPU_DATA[cpu_id] })
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
