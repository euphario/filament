//! BPI-R4 Bare-Metal Kernel
//!
//! A microkernel for the Banana Pi BPI-R4 running on the MediaTek MT7988A SoC.
//!
//! # Module Organization
//!
//! - `arch/aarch64/` - Architecture-specific code (boot, MMU, barriers, SMP)
//! - `platform/mt7988/` - SoC-specific drivers and constants
//! - `kernel/` - Portable kernel core (tasks, IPC, syscalls, schemes)

#![no_std]
#![no_main]

// Architecture support (AArch64)
mod arch {
    pub mod aarch64;
}

// Platform support (MT7988A)
mod platform {
    pub mod mt7988;
}

// Kernel core
mod kernel;

// Remaining top-level modules (to be organized later)
mod initrd;
mod log;
mod panic;
mod ramfs;

// Convenient aliases - use full paths to avoid ambiguity
use arch::aarch64::{mmu, sync, smp, mmio};
use platform::mt7988::{gic, uart, timer, eth, sd, i2c};
use kernel::{task, process, syscall, ipc, scheme, port, event, fd, shmem, elf, pmm, addrspace, uaccess};

// Alias for platform constants (platform::mt7988::INITRD_ADDR, etc.)
use platform::mt7988 as plat;

/// Kernel entry point - called from boot.S after dropping to EL1
#[no_mangle]
pub extern "C" fn kmain() -> ! {
    // Initialize UART
    uart::init();

    // Initialize logging (after UART so we can print)
    log::init();

    // Print banner
    println!();
    println!("========================================");
    println!("  BPI-R4 Bare-Metal Kernel");
    println!("  MediaTek MT7988A (Cortex-A73)");
    println!("========================================");
    println!();
    println!("Hello World from BPI-R4!");
    println!();

    // Read CPU info
    let midr: u64;
    let current_el: u64;
    let vbar: u64;

    unsafe {
        core::arch::asm!("mrs {}, midr_el1", out(reg) midr);
        core::arch::asm!("mrs {}, currentel", out(reg) current_el);
        core::arch::asm!("mrs {}, vbar_el1", out(reg) vbar);
    }

    let el = (current_el >> 2) & 0x3;

    println!("CPU Info:");
    println!("  MIDR_EL1:   0x{:016x}", midr);
    println!("  CurrentEL:  EL{}", el);
    println!("  VBAR_EL1:   0x{:016x}", vbar);
    println!();

    if el == 1 {
        println!("[OK] Successfully dropped from EL2 to EL1!");
    } else if el == 2 {
        println!("[!!] Still at EL2 - drop failed");
    }

    println!();
    println!("Initializing GIC...");
    gic::init();

    let (product, variant, num_irqs) = gic::info();
    println!("[OK] GIC initialized");
    println!("  Product:  0x{:02x}", product);
    println!("  Variant:  0x{:x}", variant);
    println!("  IRQ lines: {}", num_irqs);

    // Initialize MMU with kernel/user separation
    println!();
    println!("Initializing MMU...");
    mmu::init();
    println!("  Address space layout:");
    println!("    TTBR0 (user):   0x0000_xxxx -> identity mapped");
    println!("    TTBR1 (kernel): 0xFFFF_xxxx -> physical memory");
    println!("  L1 blocks (1GB each):");
    println!("    [0] 0x00000000: Device (UART, GIC)");
    println!("    [1] 0x40000000: Normal (DRAM)");
    println!("    [2] 0x80000000: Normal (DRAM)");
    println!("    [3] 0xC0000000: Normal (DRAM)");
    println!("Enabling MMU...");
    mmu::enable();
    println!("[OK] MMU enabled");
    mmu::print_info();

    // Initialize physical memory manager
    println!();
    println!("Initializing physical memory manager...");
    pmm::init();
    println!("[OK] PMM initialized");
    pmm::print_info();

    // Initialize shared memory subsystem
    shmem::init();

    // Initialize scheme system (needed for userspace drivers)
    println!();
    println!("Initializing scheme system...");
    scheme::init();

    // Initialize timer
    println!();
    println!("Initializing timer...");
    timer::init();
    println!("[OK] Timer initialized");
    timer::print_info();

    // Initialize SMP (but don't start secondary CPUs yet)
    println!();
    println!("Initializing SMP...");
    smp::init();

    // =========================================================================
    // Self-tests (only when compiled with --features selftest)
    // =========================================================================
    #[cfg(feature = "selftest")]
    {
        println!();
        println!("========================================");
        println!("  Running Self-Tests");
        println!("========================================");

        println!();
        println!("Testing PMM...");
        pmm::test();

        println!();
        println!("Testing address space manager...");
        addrspace::test();

        println!();
        println!("Testing task structures...");
        task::test();

        println!();
        println!("Testing syscall infrastructure...");
        syscall::test();

        println!();
        println!("Testing process management...");
        process::test();

        println!();
        println!("Testing IPC...");
        ipc::test();

        println!();
        println!("Testing port registry...");
        port::test();

        println!();
        println!("Testing event system...");
        event::test();

        println!();
        println!("Testing scheme system...");
        scheme::test();

        println!();
        println!("Testing ELF loader...");
        elf::test();

        println!();
        println!("Testing SMP...");
        smp::test();

        println!();
        println!("Testing Ethernet (register access)...");
        eth::test();

        println!();
        println!("Testing SD/eMMC (register access)...");
        sd::test();

        println!();
        println!("========================================");
        println!("  Self-Tests Complete");
        println!("========================================");
    }

    #[cfg(not(feature = "selftest"))]
    {
        println!();
        println!("  (self-tests skipped, use --features selftest to enable)");
    }

    // Note: USB is now handled by userspace driver (bin/usbtest)
    // The userspace driver uses MMIO and IRQ schemes to access hardware

    // Initialize ramfs (initrd)
    // The initrd address/size would typically be passed by U-Boot
    // For now, check if an initrd was loaded at a known address
    println!();
    println!("========================================");
    println!("  Initializing ramfs");
    println!("========================================");
    init_ramfs();

    // Enable IRQs before entering user mode
    println!();
    println!("Enabling interrupts...");
    unsafe {
        core::arch::asm!("msr daifclr, #2"); // Clear I bit to enable IRQs
    }
    println!("[OK] Interrupts enabled");

    // Spawn shell process
    println!();
    println!("========================================");
    println!("  Spawning shell");
    println!("========================================");

    // Spawn interactive shell from ramfs
    let slot1 = match elf::spawn_from_path("bin/shell") {
        Ok((_task_id, slot)) => {
            println!("  Process 'shell' spawned at slot {}", slot);
            Some(slot)
        }
        Err(e) => {
            println!("  [!!] Failed to spawn shell: {:?}", e);
            None
        }
    };

    // Note: mmap test can be added here once we have a proper test binary
    // For now, mmap/munmap syscalls are implemented and ready for use

    // Run the first process
    if let Some(slot) = slot1 {
        println!();
        println!("  Starting scheduler with process at slot {}...", slot);

        // Debug: print VBAR_EL1 to verify it's the kernel VA
        let vbar: u64;
        unsafe {
            core::arch::asm!("mrs {}, vbar_el1", out(reg) vbar);
        }
        println!("  VBAR_EL1:  0x{:016x}", vbar);

        // Start timer for preemption (10ms time slice)
        timer::start(10);
        println!("  Timer started: 10ms time slice");
        println!();

        // Enter user mode - scheduler will switch between processes
        unsafe {
            task::scheduler().run_user_task(slot);
        }
    } else {
        println!("  No processes to run - halting.");
        loop {
            unsafe { core::arch::asm!("wfi"); }
        }
    }
}

/// IRQ handler called from assembly
/// from_user: true if IRQ came from user mode (EL0), false if from kernel (EL1)
///
/// DESIGN: This handler does minimal work:
/// 1. Ack the interrupt
/// 2. Set flags for deferred processing
/// 3. EOI and return
///
/// Actual scheduling happens at safe points (syscall exit, exception return).
/// This prevents reentrancy issues and scheduler corruption.
#[no_mangle]
pub extern "C" fn irq_handler_rust(_from_user: u64) {
    // Acknowledge the interrupt from GIC
    let irq = gic::ack_irq();

    // Check for spurious interrupt
    if irq >= plat::irq::SPURIOUS_THRESHOLD {
        return; // Spurious, ignore
    }

    // Handle timer interrupt (PPI 30)
    if irq == plat::irq::TIMER_PPI {
        if timer::handle_irq() {
            // Timer tick - just set the flag, don't reschedule here
            sync::cpu_flags().tick();
            sync::cpu_flags().set_need_resched();
        }
    } else {
        // Check if this IRQ is registered by a userspace driver
        if let Some(owner_pid) = scheme::irq_notify(irq) {
            // Wake the owner process if blocked
            // This is a minimal state change - actual switch happens at safe point
            unsafe {
                let _guard = sync::IrqGuard::new(); // Ensure atomicity
                let sched = task::scheduler();
                for task_opt in sched.tasks.iter_mut() {
                    if let Some(ref mut task) = task_opt {
                        if task.id == owner_pid && task.state == task::TaskState::Blocked {
                            task.state = task::TaskState::Ready;
                            // Also set need_resched so we switch to woken task
                            sync::cpu_flags().set_need_resched();
                            break;
                        }
                    }
                }
            }
        } else {
            // Record unhandled IRQ (don't print in IRQ context!)
            sync::cpu_flags().record_unhandled_irq(irq);
        }
    }

    // Signal end of interrupt
    gic::eoi(irq);
}

/// Called from assembly after IRQ handler, at the safe point before eret.
/// This is where deferred scheduling actually happens for timer preemption.
#[no_mangle]
pub extern "C" fn irq_exit_resched() {
    unsafe {
        task::do_resched_if_needed();
    }

    // Safe point: flush deferred log buffer before returning to user
    kernel::log::flush();
}

/// Exception handler called from assembly
#[no_mangle]
pub extern "C" fn exception_handler_rust(esr: u64, elr: u64, far: u64) -> ! {
    // Force UART output
    for c in "\n=== EXCEPTION ===\n".chars() {
        crate::uart::putc(c);
    }
    println!();
    println!("  ESR_EL1: 0x{:016x}", esr);
    println!("  ELR_EL1: 0x{:016x}", elr);
    println!("  FAR_EL1: 0x{:016x}", far);

    // Decode exception class
    let ec = (esr >> 26) & 0x3f;
    let exception_name = match ec {
        0b000000 => "Unknown reason",
        0b000001 => "Trapped WFI/WFE",
        0b001110 => "Illegal execution state",
        0b010101 => "SVC instruction (syscall)",
        0b100000 => "Instruction Abort from lower EL",
        0b100001 => "Instruction Abort from same EL",
        0b100010 => "PC alignment fault",
        0b100100 => "Data Abort from lower EL",
        0b100101 => "Data Abort from same EL",
        0b100110 => "SP alignment fault",
        0b101100 => "FP/SIMD exception",
        0b111100 => "BRK instruction (breakpoint)",
        _ => "Other",
    };
    println!("  EC:      0x{:02x} ({})", ec, exception_name);
    println!("=================");
    println!();
    println!("System halted.");

    loop {
        unsafe { core::arch::asm!("wfe"); }
    }
}

/// Initialize ramfs from embedded or external initrd
fn init_ramfs() {
    // First, try embedded initrd (compiled into kernel)
    if let Some((addr, size)) = initrd::get_embedded_initrd() {
        println!("  Found embedded initrd at 0x{:08x} ({} bytes)", addr, size);
        let count = ramfs::init(addr, size);
        println!("  Loaded {} files from embedded initrd", count);
        ramfs::list();
        return;
    }

    // Fall back to external initrd at fixed address (from platform constants)
    // Check for TAR magic (ustar at offset 257)
    let magic_check = unsafe {
        let ptr = plat::INITRD_ADDR as *const u8;
        let ustar_magic = core::slice::from_raw_parts(ptr.add(257), 5);
        ustar_magic == b"ustar"
    };

    if magic_check {
        let count = ramfs::init(plat::INITRD_ADDR, plat::INITRD_MAX_SIZE);
        println!("  Loaded {} files from external initrd at 0x{:08x}", count, plat::INITRD_ADDR);
        ramfs::list();
    } else {
        println!("  No initrd found (build with: cd user && ./mkinitrd.sh)");
    }
}
