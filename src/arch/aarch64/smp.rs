//! SMP (Symmetric Multi-Processing) Support
//!
//! Provides multi-core support for MT7988A's 4x Cortex-A73 cores.
//! Uses PSCI (Power State Coordination Interface) for CPU power management.

#![allow(dead_code)]  // SMP infrastructure for future multi-core support

use core::sync::atomic::{AtomicU32, AtomicU64, AtomicBool, Ordering};
use crate::logln;

/// Maximum number of CPUs supported
pub const MAX_CPUS: usize = 4;

/// Stack size per CPU (16KB)
pub const CPU_STACK_SIZE: usize = 16 * 1024;

/// PSCI function IDs (SMC32 convention)
pub mod psci {
    pub const PSCI_VERSION: u32 = 0x84000000;
    pub const CPU_ON: u32 = 0x84000003;
    pub const CPU_OFF: u32 = 0x84000002;
    pub const CPU_SUSPEND: u32 = 0x84000001;
    pub const AFFINITY_INFO: u32 = 0x84000004;
    pub const SYSTEM_RESET: u32 = 0x84000009;

    // PSCI return codes
    pub const SUCCESS: i32 = 0;
    pub const NOT_SUPPORTED: i32 = -1;
    pub const INVALID_PARAMS: i32 = -2;
    pub const DENIED: i32 = -3;
    pub const ALREADY_ON: i32 = -4;
    pub const ON_PENDING: i32 = -5;
}

/// CPU state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum CpuState {
    Offline = 0,
    Starting = 1,
    Online = 2,
    Halted = 3,
}

/// Per-CPU data structure
#[repr(C, align(64))]  // Cache line aligned
pub struct PerCpu {
    /// CPU ID (0-3)
    pub cpu_id: u32,
    /// Current state
    pub state: AtomicU32,
    /// Stack pointer for this CPU
    pub stack_top: u64,
    /// Current task slot (for scheduler)
    pub current_task: AtomicU32,
    /// Number of ticks processed
    pub tick_count: AtomicU64,
    /// Is this CPU idle?
    pub idle: AtomicBool,
    /// Padding to cache line
    _pad: [u8; 32],
}

impl PerCpu {
    pub const fn new(cpu_id: u32) -> Self {
        Self {
            cpu_id,
            state: AtomicU32::new(CpuState::Offline as u32),
            stack_top: 0,
            current_task: AtomicU32::new(0xFFFFFFFF),
            tick_count: AtomicU64::new(0),
            idle: AtomicBool::new(true),
            _pad: [0; 32],
        }
    }

    pub fn get_state(&self) -> CpuState {
        match self.state.load(Ordering::Acquire) {
            0 => CpuState::Offline,
            1 => CpuState::Starting,
            2 => CpuState::Online,
            _ => CpuState::Halted,
        }
    }

    pub fn set_state(&self, state: CpuState) {
        self.state.store(state as u32, Ordering::Release);
    }
}

/// Per-CPU data array
static mut PER_CPU: [PerCpu; MAX_CPUS] = [
    PerCpu::new(0),
    PerCpu::new(1),
    PerCpu::new(2),
    PerCpu::new(3),
];

/// Secondary CPU entry mailbox - each CPU polls its slot
/// Contains the entry point address (0 = not ready)
#[no_mangle]
pub static SECONDARY_MAILBOX: [AtomicU64; MAX_CPUS] = [
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
];

/// Secondary CPU stacks (allocated statically)
/// Exported for use by boot.S
#[repr(C, align(4096))]
pub struct CpuStacks {
    pub stacks: [[u8; CPU_STACK_SIZE]; MAX_CPUS],
}

#[no_mangle]
pub static mut CPU_STACKS: CpuStacks = CpuStacks {
    stacks: [[0; CPU_STACK_SIZE]; MAX_CPUS],
};

/// Get current CPU ID
#[inline]
pub fn cpu_id() -> u32 {
    let mpidr: u64;
    unsafe {
        core::arch::asm!("mrs {}, mpidr_el1", out(reg) mpidr);
    }
    (mpidr & 0xFF) as u32
}

/// Get per-CPU data for current CPU
pub fn this_cpu() -> &'static PerCpu {
    unsafe { &PER_CPU[cpu_id() as usize] }
}

/// Get per-CPU data for specific CPU
pub fn get_cpu(id: u32) -> Option<&'static PerCpu> {
    if (id as usize) < MAX_CPUS {
        unsafe { Some(&PER_CPU[id as usize]) }
    } else {
        None
    }
}

/// Get mutable per-CPU data (unsafe)
pub unsafe fn get_cpu_mut(id: u32) -> Option<&'static mut PerCpu> {
    if (id as usize) < MAX_CPUS {
        Some(&mut (*core::ptr::addr_of_mut!(PER_CPU))[id as usize])
    } else {
        None
    }
}

/// Call PSCI function via SMC
#[inline]
unsafe fn psci_call(func: u32, arg0: u64, arg1: u64, arg2: u64) -> i64 {
    let ret: i64;
    core::arch::asm!(
        "smc #0",
        inout("x0") func as u64 => ret,
        in("x1") arg0,
        in("x2") arg1,
        in("x3") arg2,
        options(nomem, nostack)
    );
    ret
}

/// Get PSCI version
pub fn psci_version() -> Option<(u16, u16)> {
    let ret = unsafe { psci_call(psci::PSCI_VERSION, 0, 0, 0) };
    if ret < 0 {
        None
    } else {
        Some(((ret >> 16) as u16, ret as u16))
    }
}

/// Start a secondary CPU
/// entry_point: Physical address of entry point
/// context_id: Passed to secondary CPU in x0
pub fn cpu_on(cpu: u32, entry_point: u64, context_id: u64) -> Result<(), i32> {
    if cpu as usize >= MAX_CPUS {
        return Err(psci::INVALID_PARAMS);
    }

    // MPIDR format for MT7988A: Aff0 = CPU ID within cluster
    let mpidr = cpu as u64;

    let ret = unsafe { psci_call(psci::CPU_ON, mpidr, entry_point, context_id) };

    match ret as i32 {
        0 => Ok(()),
        e => Err(e),
    }
}

/// Check CPU affinity state
pub fn cpu_affinity_info(cpu: u32) -> i32 {
    let mpidr = cpu as u64;
    unsafe { psci_call(psci::AFFINITY_INFO, mpidr, 0, 0) as i32 }
}

/// Initialize SMP - called from primary CPU
pub fn init() {
    let cpu = cpu_id();
    logln!("  Primary CPU: {}", cpu);

    // Check PSCI version
    if let Some((major, minor)) = psci_version() {
        logln!("  PSCI version: {}.{}", major, minor);
    } else {
        logln!("  PSCI not available (may be in EL1 without EL2/EL3)");
    }

    // Initialize primary CPU's per-CPU data
    unsafe {
        if let Some(pcpu) = get_cpu_mut(cpu) {
            pcpu.stack_top = CPU_STACKS.stacks[cpu as usize].as_ptr() as u64 + CPU_STACK_SIZE as u64;
            pcpu.set_state(CpuState::Online);
            logln!("  CPU {} online", cpu);
        } else {
            logln!("  [!!] CPU {} init failed - invalid CPU ID", cpu);
        }
    }
}

/// Secondary CPU entry point (called from assembly)
#[no_mangle]
pub extern "C" fn secondary_cpu_entry(cpu_id: u64) {
    let cpu = cpu_id as u32;

    // Mark CPU as online
    unsafe {
        if let Some(pcpu) = get_cpu_mut(cpu) {
            pcpu.set_state(CpuState::Online);
        }
    }

    // Initialize GIC CPU interface for this CPU
    crate::gic::init_cpu();

    // Enable timer for this CPU
    unsafe {
        // Enable physical timer interrupt
        let ctl: u64 = 1; // ENABLE
        core::arch::asm!("msr cntp_ctl_el0, {}", in(reg) ctl);
    }

    logln!("  CPU {} online", cpu);

    // Enter idle loop - scheduler will assign tasks
    loop {
        unsafe {
            core::arch::asm!("wfe");
        }

        // Check if we have work to do
        let pcpu = this_cpu();
        let task = pcpu.current_task.load(Ordering::Acquire);
        if task != 0xFFFFFFFF {
            // We have a task - run scheduler
            // For now, just acknowledge and go back to idle
            pcpu.idle.store(false, Ordering::Release);
        }
    }
}

/// Start secondary CPUs
pub fn start_secondary_cpus() {
    let primary = cpu_id();
    logln!("  Starting secondary CPUs from CPU {}...", primary);

    // Get the physical address of secondary_start from boot.S
    extern "C" {
        fn secondary_start();
    }
    let entry = secondary_start as *const () as u64;

    for cpu in 0..MAX_CPUS as u32 {
        if cpu == primary {
            continue;
        }

        // Set up stack for this CPU
        let setup_ok = unsafe {
            if let Some(pcpu) = get_cpu_mut(cpu) {
                pcpu.stack_top = CPU_STACKS.stacks[cpu as usize].as_ptr() as u64 + CPU_STACK_SIZE as u64;
                pcpu.set_state(CpuState::Starting);
                true
            } else {
                logln!("    [!!] CPU {} has invalid ID, skipping", cpu);
                false
            }
        };
        if !setup_ok {
            continue;
        }

        // Store entry point in mailbox
        SECONDARY_MAILBOX[cpu as usize].store(entry, Ordering::Release);

        // Try to wake CPU via PSCI
        match cpu_on(cpu, entry, cpu as u64) {
            Ok(()) => {
                logln!("    CPU {} start requested", cpu);
            }
            Err(psci::ALREADY_ON) => {
                logln!("    CPU {} already on", cpu);
            }
            Err(psci::NOT_SUPPORTED) => {
                // PSCI not available, try sending event
                logln!("    CPU {} PSCI not supported, sending SEV", cpu);
                unsafe {
                    core::arch::asm!("sev");
                }
            }
            Err(e) => {
                logln!("    CPU {} start failed: {}", cpu, e);
            }
        }
    }

    // Wait briefly for CPUs to come online
    for _ in 0..1000000 {
        core::hint::spin_loop();
    }

    // Report status
    let mut online = 0;
    for cpu in 0..MAX_CPUS as u32 {
        if let Some(pcpu) = get_cpu(cpu) {
            if pcpu.get_state() == CpuState::Online {
                online += 1;
            }
        }
    }
    logln!("  {} CPUs online", online);
}

/// Get number of online CPUs
pub fn online_cpus() -> u32 {
    let mut count = 0;
    for cpu in 0..MAX_CPUS as u32 {
        if let Some(pcpu) = get_cpu(cpu) {
            if pcpu.get_state() == CpuState::Online {
                count += 1;
            }
        }
    }
    count
}

// ============================================================================
// Spinlock Implementation
// ============================================================================

/// A simple ticket spinlock
#[repr(C)]
pub struct SpinLock {
    next_ticket: AtomicU32,
    now_serving: AtomicU32,
}

impl SpinLock {
    pub const fn new() -> Self {
        Self {
            next_ticket: AtomicU32::new(0),
            now_serving: AtomicU32::new(0),
        }
    }

    /// Acquire the lock
    #[inline]
    pub fn lock(&self) {
        let ticket = self.next_ticket.fetch_add(1, Ordering::Relaxed);
        while self.now_serving.load(Ordering::Acquire) != ticket {
            core::hint::spin_loop();
        }
    }

    /// Release the lock
    #[inline]
    pub fn unlock(&self) {
        self.now_serving.fetch_add(1, Ordering::Release);
    }

    /// Try to acquire the lock without blocking
    pub fn try_lock(&self) -> bool {
        let current = self.now_serving.load(Ordering::Relaxed);
        self.next_ticket
            .compare_exchange(current, current + 1, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
    }
}

/// RAII lock guard
pub struct SpinLockGuard<'a> {
    lock: &'a SpinLock,
}

impl<'a> SpinLockGuard<'a> {
    pub fn new(lock: &'a SpinLock) -> Self {
        lock.lock();
        Self { lock }
    }
}

impl Drop for SpinLockGuard<'_> {
    fn drop(&mut self) {
        self.lock.unlock();
    }
}

/// Test SMP functionality
pub fn test() {
    logln!("  Testing SMP...");

    let cpu = cpu_id();
    logln!("    Current CPU: {}", cpu);

    // Test spinlock
    static TEST_LOCK: SpinLock = SpinLock::new();
    static TEST_COUNTER: AtomicU32 = AtomicU32::new(0);

    {
        let _guard = SpinLockGuard::new(&TEST_LOCK);
        TEST_COUNTER.fetch_add(1, Ordering::Relaxed);
    }
    logln!("    Spinlock test: counter = {}", TEST_COUNTER.load(Ordering::Relaxed));

    // Show online CPUs
    logln!("    Online CPUs: {}", online_cpus());

    logln!("    [OK] SMP test passed");
}
