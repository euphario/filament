//! BPI-R4 Bare-Metal Kernel
//!
//! A minimal "Hello World" kernel for the Banana Pi BPI-R4
//! running on the MediaTek MT7988A SoC.

#![no_std]
#![no_main]

mod addrspace;
mod elf;
mod event;
mod fd;
mod gic;
mod ipc;
mod mmu;
mod panic;
mod pmm;
mod port;
mod process;
mod scheme;
mod syscall;
mod task;
mod timer;
mod uart;

use core::arch::global_asm;

// Include the assembly startup code
global_asm!(include_str!("boot.S"));

/// Kernel entry point - called from boot.S after dropping to EL1
#[no_mangle]
pub extern "C" fn kmain() -> ! {
    // Initialize UART
    uart::init();

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
    pmm::test();

    // Test address space management
    println!();
    println!("Testing address space manager...");
    addrspace::test();

    // Test task/context switch structures
    println!();
    println!("Testing task structures...");
    task::test();

    // Test syscall infrastructure
    println!();
    println!("Testing syscall infrastructure...");
    syscall::test();

    // Test process management
    println!();
    println!("Testing process management...");
    process::test();

    // Test IPC
    println!();
    println!("Testing IPC...");
    ipc::test();

    // Test port registry
    println!();
    println!("Testing port registry...");
    port::test();

    // Test event system
    println!();
    println!("Testing event system...");
    event::test();

    // Initialize scheme system
    println!();
    println!("Initializing scheme system...");
    scheme::init();
    scheme::test();

    // Test ELF loader
    println!();
    println!("Testing ELF loader...");
    elf::test();

    // Initialize timer (but don't start it - user process will control flow)
    println!();
    println!("Initializing timer...");
    timer::init();
    println!("[OK] Timer initialized");
    timer::print_info();

    // Enable IRQs before entering user mode
    println!();
    println!("Enabling interrupts...");
    unsafe {
        core::arch::asm!("msr daifclr, #2"); // Clear I bit to enable IRQs
    }
    println!("[OK] Interrupts enabled");

    // Spawn multiple processes
    println!();
    println!("========================================");
    println!("  Spawning user processes");
    println!("========================================");

    // Spawn first process
    let slot1 = match elf::spawn_from_elf(elf::get_test_elf(), "init") {
        Ok((_task_id, slot)) => {
            println!("  Process 'init' spawned at slot {}", slot);
            Some(slot)
        }
        Err(e) => {
            println!("  [!!] Failed to spawn init: {:?}", e);
            None
        }
    };

    // Spawn second process
    let _slot2 = match elf::spawn_from_elf(elf::get_test_elf2(), "worker") {
        Ok((_task_id, slot)) => {
            println!("  Process 'worker' spawned at slot {}", slot);
            Some(slot)
        }
        Err(e) => {
            println!("  [!!] Failed to spawn worker: {:?}", e);
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
#[no_mangle]
pub extern "C" fn irq_handler_rust(from_user: u64) {
    // Acknowledge the interrupt from GIC
    let irq = gic::ack_irq();

    // Check for spurious interrupt
    if irq >= 1020 {
        return; // Spurious, ignore
    }

    // Handle timer interrupt (PPI 30)
    if irq == 30 {
        if timer::handle_irq() {
            // Timer tick - potentially preempt user process
            if from_user != 0 {
                // Preempt: schedule next task
                unsafe {
                    let sched = task::scheduler();

                    // Mark current as ready (it was running)
                    if let Some(ref mut current) = sched.tasks[sched.current] {
                        if current.state == task::TaskState::Running {
                            current.state = task::TaskState::Ready;
                        }
                    }

                    // Find next task
                    if let Some(next_slot) = sched.schedule() {
                        if next_slot != sched.current {
                            sched.current = next_slot;
                            if let Some(ref mut next_task) = sched.tasks[next_slot] {
                                next_task.state = task::TaskState::Running;
                            }
                            task::update_current_task_globals();
                        } else {
                            // Same task, mark as running again
                            if let Some(ref mut current) = sched.tasks[sched.current] {
                                current.state = task::TaskState::Running;
                            }
                        }
                    }
                }
            }
        }
    } else {
        println!("[IRQ] Unhandled interrupt: {}", irq);
    }

    // Signal end of interrupt
    gic::eoi(irq);
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
