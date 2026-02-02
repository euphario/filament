//! SMP (Symmetric Multi-Processing) Support
//!
//! Provides multi-core support for MT7988A's 4x Cortex-A73 cores.
//! Uses PSCI (Power State Coordination Interface) for CPU power management.
//!
//! Per-CPU state is managed by `kernel::percpu::CpuData`. This module
//! handles PSCI calls, secondary CPU boot, and stack allocation.

use core::sync::atomic::{AtomicU64, Ordering};
use crate::{kinfo, kwarn, kdebug};
use crate::kernel::percpu::{self, CpuState, MAX_CPUS};

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

/// Call PSCI function via HVC (QEMU virt — no EL3 firmware).
#[cfg(feature = "platform-qemu-virt")]
#[inline]
unsafe fn psci_call(func: u32, arg0: u64, arg1: u64, arg2: u64) -> i64 {
    let ret: i64;
    core::arch::asm!(
        "hvc #0",
        inout("x0") func as u64 => ret,
        in("x1") arg0,
        in("x2") arg1,
        in("x3") arg2,
        options(nomem, nostack)
    );
    ret
}

/// Call PSCI function via SMC (real hardware with EL3 firmware).
#[cfg(not(feature = "platform-qemu-virt"))]
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
    let cpu = percpu::cpu_id();
    kdebug!("smp", "init"; primary_cpu = cpu as u64);

    // Check PSCI version
    if let Some((major, minor)) = psci_version() {
        kinfo!("smp", "psci_version"; major = major as u64, minor = minor as u64);
    } else {
        kwarn!("smp", "psci_unavailable");
    }

    // Initialize primary CPU's state via CpuData
    unsafe {
        let stack_top = CPU_STACKS.stacks[cpu as usize].as_ptr() as u64 + CPU_STACK_SIZE as u64;
        percpu::CPU_DATA[cpu as usize].set_stack_top(stack_top);
        percpu::CPU_DATA[cpu as usize].set_state(CpuState::Online);
    }
    kinfo!("smp", "cpu_online"; cpu = cpu as u64);
}

/// Secondary CPU entry point (called from assembly)
///
/// Sets up per-CPU data, GIC, timer, creates idle task, then enters idle loop.
#[no_mangle]
pub extern "C" fn secondary_cpu_entry(cpu_id_arg: u64) {
    let cpu = cpu_id_arg as u32;

    // 1. Initialize per-CPU data (sets TPIDR_EL1)
    percpu::init_secondary_cpu(cpu);

    // 2. Mark CPU as online via CpuData
    percpu::cpu_local().set_state(CpuState::Online);

    // 3. Initialize GIC CPU interface for this CPU
    crate::platform::current::gic::init_cpu();

    // 4. Create idle task for this CPU in the scheduler
    crate::kernel::task::init_secondary_scheduler(cpu);

    // 5. Set current slot to this CPU's idle task
    crate::kernel::task::set_current_slot(cpu as usize);

    // 6. Start timer for this CPU
    crate::platform::current::timer::start(10);

    kinfo!("smp", "cpu_online"; cpu = cpu as u64);

    // 7. Enter idle loop (never returns)
    crate::kernel::idle::idle_entry();
}

/// Start secondary CPUs
pub fn start_secondary_cpus() {
    let primary = percpu::cpu_id();
    kdebug!("smp", "starting_secondary"; primary = primary as u64);

    // Get the physical address of secondary_start from boot.S
    // PSCI CPU_ON requires a physical address — secondary CPU starts with MMU off
    extern "C" {
        fn secondary_start();
    }
    let entry = crate::arch::aarch64::mmu::virt_to_phys(secondary_start as *const () as u64);

    for cpu in 0..MAX_CPUS as u32 {
        if cpu == primary {
            continue;
        }

        // Set up stack and state for this CPU via CpuData
        unsafe {
            let stack_top = CPU_STACKS.stacks[cpu as usize].as_ptr() as u64 + CPU_STACK_SIZE as u64;
            percpu::CPU_DATA[cpu as usize].set_stack_top(stack_top);
            percpu::CPU_DATA[cpu as usize].set_state(CpuState::Starting);
        }

        // Store entry point in mailbox
        SECONDARY_MAILBOX[cpu as usize].store(entry, Ordering::Release);

        // Try to wake CPU via PSCI
        match cpu_on(cpu, entry, cpu as u64) {
            Ok(()) => {
                kdebug!("smp", "cpu_start_requested"; cpu = cpu as u64);
            }
            Err(psci::ALREADY_ON) => {
                kdebug!("smp", "cpu_already_on"; cpu = cpu as u64);
            }
            Err(psci::NOT_SUPPORTED) => {
                // PSCI not available, try sending event
                kdebug!("smp", "psci_not_supported_sev"; cpu = cpu as u64);
                unsafe {
                    core::arch::asm!("sev");
                }
            }
            Err(e) => {
                kwarn!("smp", "cpu_start_failed"; cpu = cpu as u64, err = e as i64);
            }
        }
    }

    // Wait briefly for CPUs to come online
    for _ in 0..1000000 {
        core::hint::spin_loop();
    }

    // Report status
    let online_count = percpu::num_online_cpus();
    kinfo!("smp", "cpus_online"; count = online_count as u64);

    // Update scheduler's round-robin to distribute across all online CPUs
    if online_count > 1 {
        crate::kernel::task::with_scheduler(|sched| {
            sched.set_num_cpus(online_count);
        });
    }
}

/// Test SMP functionality
pub fn test() {
    kdebug!("smp", "test_start");

    let cpu = percpu::cpu_id();
    kdebug!("smp", "current_cpu"; cpu = cpu as u64);

    // Test kernel SpinLock (ticket lock from kernel::lock)
    static TEST_LOCK: crate::kernel::lock::SpinLock<u32> = crate::kernel::lock::SpinLock::new(0);

    {
        let mut guard = TEST_LOCK.lock();
        *guard += 1;
    }
    let counter = *TEST_LOCK.lock();
    kdebug!("smp", "spinlock_test"; counter = counter as u64);

    // Show online CPUs
    let online = percpu::num_online_cpus();
    kdebug!("smp", "online_cpus"; count = online as u64);

    kinfo!("smp", "test_ok");
}
