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
#![allow(dead_code)]        // Kernel functions reserved for SMP, drivers, etc.
#![allow(unused_unsafe)]    // Some unsafe blocks are for future-proofing
#![allow(unused_must_use)]  // Some Results intentionally ignored

// Architecture support (AArch64)
mod arch {
    pub mod aarch64;
}

// Platform support - conditional compilation
mod platform;

// Hardware Abstraction Layer
mod hal;

// Kernel core
mod kernel;

// Remaining top-level modules (to be organized later)
mod dtb;
mod initrd;
mod klog;      // Unified logging framework
mod ktrace;    // Unified tracing framework (spans, call depth)
mod log;       // Legacy logging (used by platform drivers, to migrate later)
mod serialize; // Shared serialization for klog/ktrace
mod panic;
mod ramfs;

// Convenient aliases - use full paths to avoid ambiguity
use arch::aarch64::{mmu, sync, smp};
use kernel::{task, irq, shmem, dma_pool, elf, pmm, pci, bus};
use kernel::percpu;

// Platform-specific imports - use platform::current for portability
use platform::current::{gic, uart, timer};
use platform::current as plat;

#[cfg(feature = "platform-mt7988a")]
use platform::mt7988::wdt;

/// Kernel entry point - called from boot.S after dropping to EL1
#[no_mangle]
pub extern "C" fn kmain() -> ! {
    // =========================================================================
    // Phase 1: Early init (no logging yet)
    // =========================================================================
    uart::init();

    // Reset terminal state first - ensures clean display on remote terminal
    // ESC c = Full Reset (RIS) - clears screen, resets all modes
    print_direct!("\x1bc");

    percpu::init_boot_cpu();
    task::init_scheduler();  // Explicitly init scheduler for platforms with static init issues
    sync::verify_pan_enabled();

    // Initialize logging/tracing infrastructure
    klog::init();      // Structured logging (LOG_RING → UART)
    ktrace::init();    // Span-based tracing (TRACE_RING)

    // Boot banner (direct UART - useful for early debug)
    print_direct!("\r\n========================================\r\n");
    #[cfg(feature = "platform-mt7988a")]
    print_direct!("  BPI-R4 Bare-Metal Kernel\r\n");
    #[cfg(feature = "platform-mt7988a")]
    print_direct!("  MediaTek MT7988A (Cortex-A73)\r\n");
    #[cfg(feature = "platform-qemu-virt")]
    print_direct!("  Bare-Metal Kernel (QEMU virt)\r\n");
    #[cfg(feature = "platform-qemu-virt")]
    print_direct!("  ARM Cortex-A72\r\n");
    print_direct!("========================================\r\n\r\n");

    // =========================================================================
    // Phase 2: Kernel initialization (with logging and tracing)
    // =========================================================================

    // =========================================================================
    // Phase 2: Kernel initialization (with logging and tracing)
    // =========================================================================

    // Start boot trace span
    let _boot_span = span!("kernel", "boot");

    // Read CPU info
    let (midr, current_el, vbar) = unsafe {
        let midr: u64;
        let el: u64;
        let vbar: u64;
        core::arch::asm!("mrs {}, midr_el1", out(reg) midr);
        core::arch::asm!("mrs {}, currentel", out(reg) el);
        core::arch::asm!("mrs {}, vbar_el1", out(reg) vbar);
        (midr, (el >> 2) & 0x3, vbar)
    };

    kinfo!("kernel", "cpu_info"; midr = klog::hex64(midr), el = current_el, vbar = klog::hex64(vbar));

    if current_el != 1 {
        kerror!("kernel", "el_check_failed"; expected = 1u64, actual = current_el);
    }

    // Device tree parsing
    {
        let _span = span!("fdt", "init");
        kernel::fdt::init();
    }

    // GIC (interrupt controller)
    {
        let _span = span!("gic", "init");
        gic::init();
        let (product, variant, num_irqs) = gic::info();
        kinfo!("gic", "init_ok"; product = klog::hex32(product as u32), variant = variant, irqs = num_irqs);

        uart::enable_rx_interrupt();
        gic::enable_irq(plat::irq::UART0);
    }

    // MMU info (already initialized by boot.S)
    kinfo!("mmu", "status"; ttbr0 = "user", ttbr1 = "kernel");
    mmu::print_info();

    // Set up guard pages for boot stack (splits 2MB L2 block into L3 4KB pages)
    mmu::setup_guard_pages();

    // Physical memory manager
    {
        let _span = span!("pmm", "init");
        pmm::init();
        // Note: pmm::init() logs init_ok with details
    }

    // Switch from linker-script boot stack to a PMM-allocated stack with guard page.
    // The boot stack sits right above BSS with no guard page — any overflow silently
    // corrupts kernel globals. Per-task kernel stacks (allocated later) all have guard
    // pages, so we use the same pattern here for the boot/idle stack.
    {
        use kernel::task::tcb::{KERNEL_STACK_SIZE, GUARD_PAGE_SIZE};
        let total_pages = (KERNEL_STACK_SIZE / 4096) + 1; // +1 for guard page
        let alloc_base = pmm::alloc_pages(total_pages)
            .expect("failed to allocate boot kernel stack");
        // Layout: [guard page | usable stack (64KB) ]
        //         alloc_base   alloc_base+4K          alloc_base+4K+64K
        let stack_top_phys = alloc_base + GUARD_PAGE_SIZE + KERNEL_STACK_SIZE;
        let stack_top_virt = arch::aarch64::mmu::phys_to_virt(stack_top_phys as u64);
        unsafe {
            core::arch::asm!(
                "mov sp, {new_sp}",
                new_sp = in(reg) stack_top_virt,
            );
        }
    }

    // DMA pools
    {
        let _span = span!("dma", "init");
        dma_pool::init();
        dma_pool::init_high();
        kinfo!("dma", "pools_ready");
    }

    // Shared memory
    {
        let _span = span!("shmem", "init");
        shmem::init();
        // Note: shmem::init() logs init_ok with details
    }

    // PCI subsystem
    {
        let _span = span!("pci", "init");
        pci::init();
        kinfo!("pci", "init_ok");
    }

    // Bus controllers — registry only, no buses registered yet
    // Buses are created by probed (userspace) via open(Bus) syscall
    {
        let _span = span!("bus", "init");
        bus::init(0);  // Initialize registry (kernel PID = 0), no buses
        kinfo!("bus", "init_ok");
    }

    // Watchdog (MT7988A only)
    #[cfg(feature = "platform-mt7988a")]
    {
        let _span = span!("wdt", "init");
        wdt::init();
        kinfo!("wdt", "init_ok"; enabled = false);
    }

    // Timer
    {
        let _span = span!("timer", "init");
        timer::init();
    }

    // SMP
    {
        let _span = span!("smp", "init");
        smp::init();
    }

    // Flush logs before self-tests
    klog::flush();
    ktrace::flush();

    // =========================================================================
    // Self-tests (only when compiled with --features selftest)
    // =========================================================================
    #[cfg(feature = "selftest")]
    {
        let _span = span!("kernel", "selftest");
        kinfo!("kernel", "selftest_start");

        kernel::lock::test();  // Test locks early - other modules depend on them
        pmm::test();
        kernel::dma_pool::test();  // Test DMA pool after PMM
        kernel::shmem::test();  // Test shmem after PMM
        kernel::addrspace::test();
        task::test();
        kernel::syscall::test();
        kernel::process::test();
        // IPC tests covered by object system tests in kernel::syscall::test()
        elf::test();
        smp::test();

        // Platform-specific tests (MT7988A only)
        #[cfg(feature = "platform-mt7988a")]
        {
            plat::eth::test();
            plat::sd::test();
        }

        kinfo!("kernel", "selftest_complete");
    }

    #[cfg(not(feature = "selftest"))]
    {
        kdebug!("kernel", "selftest_skipped");
    }

    // =========================================================================
    // Phase 3: Ramfs and userspace init
    // =========================================================================

    // Initialize ramfs
    {
        let _span = span!("ramfs", "init");
        init_ramfs();
    }

    // Flush logs after ramfs init (before timer starts)
    klog::flush();

    // Enable interrupts
    unsafe {
        core::arch::asm!("msr daifclr, #2");
    }
    kinfo!("kernel", "irq_enabled");

    // =========================================================================
    // Phase 4: Start scheduler
    // =========================================================================

    // NOTE: Timer preemption is started in run_user_task(), not here.
    // Starting it here would allow timer IRQs to fire during boot and
    // trigger a context switch BEFORE run_user_task is called, which
    // corrupts the boot flow.

    // Note: idle task is created automatically by init_scheduler() in slot 0
    kinfo!("kernel", "scheduler_ready");

    // Spawn probed (bus discovery) - runs first, exits quickly, then devd is spawned
    kinfo!("kernel", "spawning_probed");
    let slot1 = {
        let _span = span!("elf", "spawn"; path = "bin/probed");
        match elf::spawn_from_path("bin/probed") {
            Ok((_task_id, slot)) => {
                kinfo!("kernel", "probed_spawned"; slot = slot);
                klog::flush();
                // Give probed PROBED capabilities and mark as probed process
                task::with_scheduler(|sched| {
                    if let Some(task) = sched.task_mut(slot) {
                        task.set_capabilities(kernel::caps::Capabilities::PROBED);
                        task.is_probed = true;
                    }
                });
                Some(slot)
            }
            Err(e) => {
                let err_str = match e {
                    elf::ElfError::BadMagic => "bad_magic",
                    elf::ElfError::Not64Bit => "not_64bit",
                    elf::ElfError::NotLittleEndian => "not_le",
                    elf::ElfError::NotExecutable => "not_exec",
                    elf::ElfError::WrongArch => "wrong_arch",
                    elf::ElfError::TooSmall => "too_small",
                    elf::ElfError::OutOfMemory => "oom",
                    elf::ElfError::InvalidSegment => "invalid_seg",
                    elf::ElfError::SignatureInvalid => "sig_invalid",
                };
                kerror!("kernel", "probed_spawn_failed"; err = err_str);
                None
            }
        }
    };

    if let Some(slot) = slot1 {
        // Start secondary CPUs now that scheduler is ready with idle tasks
        smp::start_secondary_cpus();

        // Flush all logs before entering userspace
        kinfo!("kernel", "entering_userspace"; slot = slot);
        klog::flush();
        ktrace::flush();

        // Enter user mode - scheduler will switch between processes
        // (boot span will be auto-closed when we context switch)
        //
        // CRITICAL: We must release the scheduler lock BEFORE entering usermode.
        // Otherwise, the lock is never released (eret doesn't return) and all
        // subsequent scheduler() calls will deadlock.
        let (trap_frame, ttbr0, kstack_top) = unsafe {
            let mut sched = task::scheduler();
            match sched.prepare_user_task(slot) {
                Some(data) => data,
                None => {
                    // Task preparation failed - enter idle instead
                    drop(sched);
                    kerror!("kernel", "task_prep_failed"; slot = slot);
                    kernel::idle::idle_entry();
                }
            }
            // sched guard is dropped here, releasing the lock
        };

        // Set per-CPU kernel stack top so exception return paths can
        // restore SP_EL1 correctly after simple preemption switches tasks.
        kernel::percpu::set_kernel_stack_top(kstack_top);

        // Start timer preemption just before entering userspace.
        // We can't start this earlier in boot because timer IRQs
        // would trigger rescheduling before we're ready.
        crate::platform::current::timer::start(10);

        // Enter usermode - this never returns
        unsafe {
            task::enter_usermode(trap_frame, ttbr0, kstack_top);
        }
    } else {
        kerror!("kernel", "no_init_process");
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

    // Handle SGI 0 (reschedule IPI from another CPU)
    if irq < 16 {
        // SGI - just set need_resched, EOI, and return
        // The actual reschedule happens at IRQ exit via do_resched_if_needed
        sync::cpu_flags().set_need_resched();
        gic::eoi(irq);
        return;
    }

    // Handle timer interrupt (PPI 30)
    if irq == plat::irq::TIMER_PPI {
        if timer::handle_irq() {
            // Timer tick - just set the flag, don't reschedule here
            sync::cpu_flags().tick();
            sync::cpu_flags().set_need_resched();
        }
    } else if irq == plat::irq::UART0 {
        // UART RX interrupt - data available on console
        // IRQ handler drains FIFO into ring buffer, so IRQ won't re-trigger
        if uart::handle_rx_irq() {
            // Wake any process blocked waiting for console input.
            // Deferred via microtask queue — processed at irq_exit_resched.
            let blocked_pid = uart::get_blocked_pid();
            if blocked_pid != 0 {
                let _ = crate::kernel::microtask::enqueue(
                    crate::kernel::microtask::MicroTask::Wake { pid: blocked_pid },
                );
                uart::clear_blocked();
            }
        }
    } else {
        // Check if this IRQ is registered by a userspace driver
        if let Some(owner_pid) = irq::notify(irq) {
            // Wake the owner process — deferred via microtask queue
            let _ = crate::kernel::microtask::enqueue(
                crate::kernel::microtask::MicroTask::Wake { pid: owner_pid },
            );
        } else {
            // Record unhandled IRQ (don't print in IRQ context!)
            sync::cpu_flags().record_unhandled_irq(irq);
        }
    }

    // Signal end of interrupt
    gic::eoi(irq);
}

/// Called from assembly after IRQ handler, at the safe point before eret.
/// This is where deferred work happens:
/// 1. Drain microtask queue (deferred wakes, cleanup, evictions)
/// 2. Reschedule if timer set the flag
/// 3. Log any deferred messages (e.g., unhandled IRQs)
/// 4. Flush log buffer before returning to user
#[no_mangle]
pub extern "C" fn irq_exit_resched() {
    // 1. Drain microtask queue (cleanup, evictions, deferred wakes).
    // IRQ handlers enqueue Wake microtasks instead of directly acquiring the
    // scheduler lock (which may be held by interrupted code).
    crate::kernel::microtask::drain(16);

    // 2. Handle deferred reschedule
    unsafe {
        task::do_resched_if_needed();
    }

    // 3. Process any pending task evictions (legacy — kept for lock-free IRQ path)
    task::eviction::process_pending_evictions();

    // 4. Log any unhandled IRQs from interrupt context (deferred logging)
    let (unhandled_count, last_irq) = sync::cpu_flags().get_unhandled_stats();
    if unhandled_count > 0 {
        kwarn!("irq", "unhandled"; count = unhandled_count as u64, last = last_irq as u64);
        sync::cpu_flags().clear_unhandled_stats();
    }

    // 4. Don't flush all logs here - let timer tick drain gradually
    // Calling flush() here caused UART saturation feedback loop in QEMU
    // where output took longer than timer interval, causing cascading delays
}

/// Print a hex value directly to UART (no allocations, no logging)
pub fn print_hex_uart(val: u64) {
    for i in (0..16).rev() {
        let nibble = ((val >> (i * 4)) & 0xf) as u8;
        let c = if nibble < 10 { b'0' + nibble } else { b'a' + nibble - 10 };
        uart::putc(c as char);
    }
}

pub fn print_str_uart(s: &str) {
    for c in s.chars() {
        uart::putc(c);
    }
}

/// Diagnostic: called from assembly when CURRENT_TTBR0 is 0 before eret
/// caller_id: 1=exception_from_user, 2=irq_kernel_to_user, 3=irq_from_user, 4=svc_handler
#[no_mangle]
pub extern "C" fn ttbr0_zero_diagnostic(caller_id: u64) {
    print_str_uart("\r\n=== TTBR0 ZERO DETECTED ===\r\n");
    print_str_uart("  Caller: ");
    let caller_name = match caller_id {
        1 => "exception_from_user",
        2 => "irq_kernel_to_user",
        3 => "irq_from_user",
        4 => "svc_handler",
        _ => "unknown",
    };
    print_str_uart(caller_name);
    print_str_uart("\r\n");

    // Read current TTBR0 value from hardware
    let hw_ttbr0: u64;
    unsafe { core::arch::asm!("mrs {}, ttbr0_el1", out(reg) hw_ttbr0); }
    print_str_uart("  HW TTBR0: ");
    print_hex_uart(hw_ttbr0);
    print_str_uart("\r\n");

    // Read per-CPU TTBR0
    let atomic_val = kernel::percpu::get_ttbr0();
    print_str_uart("  Per-CPU TTBR0: ");
    print_hex_uart(atomic_val);
    print_str_uart("\r\n");

    // Read per-CPU TRAP_FRAME
    let trap_ptr = kernel::percpu::get_trap_frame();
    print_str_uart("  Per-CPU TRAP_FRAME: ");
    print_hex_uart(trap_ptr as u64);
    print_str_uart("\r\n");

    // Read current slot
    let slot = kernel::task::current_slot();
    print_str_uart("  Current slot: ");
    print_hex_uart(slot as u64);
    print_str_uart("\r\n");

    // Read per-CPU SYSCALL_SWITCHED
    let switched = kernel::percpu::get_syscall_switched();
    print_str_uart("  SYSCALL_SWITCHED: ");
    print_hex_uart(switched);
    print_str_uart("\r\n");

    // Try to get task info from scheduler
    if let Some(sched) = unsafe { kernel::task::try_scheduler() } {
        if let Some(task) = sched.task(slot) {
            print_str_uart("  Task PID: ");
            print_hex_uart(task.id as u64);
            print_str_uart("\r\n");
            print_str_uart("  Task state: ");
            print_str_uart(task.state().name());
            print_str_uart("\r\n");
            print_str_uart("  Task name: ");
            for &c in &task.name {
                if c == 0 { break; }
                uart::putc(c as char);
            }
            print_str_uart("\r\n");
            if let Some(ref addr_space) = task.address_space {
                print_str_uart("  addr_space.get_ttbr0(): ");
                print_hex_uart(addr_space.get_ttbr0());
                print_str_uart("\r\n");
            } else {
                print_str_uart("  addr_space: NONE\r\n");
            }
            print_str_uart("  needs_context_restore: ");
            print_hex_uart(task.needs_context_restore() as u64);
            print_str_uart("\r\n");
        } else {
            print_str_uart("  Slot is EMPTY\r\n");
        }
    } else {
        print_str_uart("  Cannot acquire scheduler lock (held by another context)\r\n");
    }

    // Print CPU ID for SMP debugging
    let cpu = kernel::percpu::cpu_id();
    print_str_uart("  CPU ID: ");
    print_hex_uart(cpu as u64);
    print_str_uart("\r\n");

    print_str_uart("=== HALTING ===\r\n");
    loop {
        unsafe { core::arch::asm!("wfe"); }
    }
}

/// Exception from user mode - terminate task and switch to next
/// Called from assembly when a userspace task faults (page fault, illegal instruction, etc.)
#[no_mangle]
pub extern "C" fn exception_from_user_rust(esr: u64, elr: u64, far: u64) {
    // Flush any buffered UART output first
    while uart::has_buffered_output() {
        uart::flush_buffer();
    }

    // Get current task info
    let (pid, parent_id, task_name, is_init) = unsafe {
        let sched = kernel::task::scheduler();
        let slot = kernel::task::current_slot();
        if let Some(task) = sched.task(slot) {
            let mut name = [0u8; 16];
            let name_len = task.name.len().min(16);
            name[..name_len].copy_from_slice(&task.name[..name_len]);
            (task.id, task.parent_id, name, task.is_init)
        } else {
            (0, 0, [0u8; 16], false)
        }
    };

    // Decode exception class
    let ec = (esr >> 26) & 0x3f;
    let exception_name = match ec {
        0b000000 => "Unknown",
        0b100000 => "Instruction Abort",
        0b100100 => "Data Abort",
        0b100010 => "PC Alignment",
        0b100110 => "SP Alignment",
        0b111100 => "BRK (breakpoint)",
        _ => "Exception",
    };

    // Print crash info
    print_str_uart("\r\n=== USER FAULT ===\r\n");
    print_str_uart("  PID: ");
    print_hex_uart(pid as u64);
    print_str_uart(" (");
    for &c in &task_name {
        if c == 0 { break; }
        uart::putc(c as char);
    }
    print_str_uart(")\r\n  ");
    print_str_uart(exception_name);
    print_str_uart(" at PC=0x");
    print_hex_uart(elr);
    print_str_uart(" FAR=0x");
    print_hex_uart(far);
    print_str_uart("\r\n");

    // Dump key registers from trap frame for crash diagnosis
    let trap_ptr = kernel::percpu::get_trap_frame();
    if !trap_ptr.is_null() {
        let tf = unsafe { &*trap_ptr };
        print_str_uart("  LR=0x");
        print_hex_uart(tf.x30);
        print_str_uart(" FP=0x");
        print_hex_uart(tf.x29);
        print_str_uart("\r\n");
        // Dump x0-x3 (arguments) and x19-x21 (callee-saved)
        print_str_uart("  x0=0x");
        print_hex_uart(tf.x0);
        print_str_uart(" x1=0x");
        print_hex_uart(tf.x1);
        print_str_uart(" x2=0x");
        print_hex_uart(tf.x2);
        print_str_uart(" x3=0x");
        print_hex_uart(tf.x3);
        print_str_uart("\r\n");
        print_str_uart("  x19=0x");
        print_hex_uart(tf.x19);
        print_str_uart(" x20=0x");
        print_hex_uart(tf.x20);
        print_str_uart(" x21=0x");
        print_hex_uart(tf.x21);
        print_str_uart("\r\n");
    }

    // Debug: Print TTBR0 and user sp for crash diagnosis
    print_str_uart("  TTBR0=0x");
    let ttbr0: u64;
    unsafe { core::arch::asm!("mrs {0}, ttbr0_el1", out(reg) ttbr0, options(nomem, nostack)); }
    print_hex_uart(ttbr0);
    print_str_uart(" SP_EL0=0x");
    let sp_el0: u64;
    unsafe { core::arch::asm!("mrs {0}, sp_el0", out(reg) sp_el0, options(nomem, nostack)); }
    print_hex_uart(sp_el0);
    print_str_uart("\r\n");

    // Debug: Walk page tables to see what's mapped for FAR
    print_str_uart("  Page table walk for FAR:\r\n");
    let l0_phys = ttbr0 & 0x0000_FFFF_FFFF_F000;
    let l0_idx = ((far >> 39) & 0x1FF) as usize;
    let l1_idx = ((far >> 30) & 0x1FF) as usize;
    let l2_idx = ((far >> 21) & 0x1FF) as usize;
    let l3_idx = ((far >> 12) & 0x1FF) as usize;
    print_str_uart("    L0_phys=0x");
    print_hex_uart(l0_phys);
    print_str_uart(" indices=");
    print_hex_uart(l0_idx as u64);
    print_str_uart(",");
    print_hex_uart(l1_idx as u64);
    print_str_uart(",");
    print_hex_uart(l2_idx as u64);
    print_str_uart(",");
    print_hex_uart(l3_idx as u64);
    print_str_uart("\r\n");

    // Read L0 entry
    let l0_virt = crate::arch::aarch64::mmu::phys_to_virt(l0_phys);
    let l0_entry = unsafe { core::ptr::read_volatile((l0_virt as *const u64).add(l0_idx)) };
    print_str_uart("    L0[");
    print_hex_uart(l0_idx as u64);
    print_str_uart("]=0x");
    print_hex_uart(l0_entry);
    if (l0_entry & 0x3) == 0x3 { print_str_uart(" (TABLE)"); }
    else if (l0_entry & 0x1) == 0 { print_str_uart(" (INVALID!)"); }
    print_str_uart("\r\n");

    if (l0_entry & 0x3) == 0x3 {
        let l1_phys = l0_entry & 0x0000_FFFF_FFFF_F000;
        let l1_virt = crate::arch::aarch64::mmu::phys_to_virt(l1_phys);
        let l1_entry = unsafe { core::ptr::read_volatile((l1_virt as *const u64).add(l1_idx)) };
        print_str_uart("    L1[");
        print_hex_uart(l1_idx as u64);
        print_str_uart("]=0x");
        print_hex_uart(l1_entry);
        if (l1_entry & 0x3) == 0x3 { print_str_uart(" (TABLE)"); }
        else if (l1_entry & 0x3) == 0x1 { print_str_uart(" (BLOCK)"); }
        else if (l1_entry & 0x1) == 0 { print_str_uart(" (INVALID!)"); }
        print_str_uart("\r\n");

        if (l1_entry & 0x3) == 0x3 {
            let l2_phys = l1_entry & 0x0000_FFFF_FFFF_F000;
            let l2_virt = crate::arch::aarch64::mmu::phys_to_virt(l2_phys);
            let l2_entry = unsafe { core::ptr::read_volatile((l2_virt as *const u64).add(l2_idx)) };
            print_str_uart("    L2[");
            print_hex_uart(l2_idx as u64);
            print_str_uart("]=0x");
            print_hex_uart(l2_entry);
            if (l2_entry & 0x3) == 0x3 { print_str_uart(" (TABLE)"); }
            else if (l2_entry & 0x3) == 0x1 { print_str_uart(" (BLOCK)"); }
            else if (l2_entry & 0x1) == 0 { print_str_uart(" (INVALID!)"); }
            print_str_uart("\r\n");

            if (l2_entry & 0x3) == 0x3 {
                let l3_phys = l2_entry & 0x0000_FFFF_FFFF_F000;
                let l3_virt = crate::arch::aarch64::mmu::phys_to_virt(l3_phys);

                // DEBUG: Read L3 entry BEFORE cache invalidate
                let l3_entry_cached = unsafe { core::ptr::read_volatile((l3_virt as *const u64).add(l3_idx)) };
                print_str_uart("    L3[");
                print_hex_uart(l3_idx as u64);
                print_str_uart("] cached=0x");
                print_hex_uart(l3_entry_cached);

                // Invalidate cache line to see true memory value
                let pte_addr = unsafe { (l3_virt as *const u64).add(l3_idx) } as u64;
                unsafe {
                    core::arch::asm!(
                        "dc ivac, {addr}",  // Invalidate data cache by VA to PoC
                        "dsb ish",
                        addr = in(reg) pte_addr,
                        options(nostack, preserves_flags)
                    );
                }

                // Re-read AFTER cache invalidate
                let l3_entry = unsafe { core::ptr::read_volatile((l3_virt as *const u64).add(l3_idx)) };
                print_str_uart(" after_invalidate=0x");
                print_hex_uart(l3_entry);
                if (l3_entry & 0x3) == 0x3 { print_str_uart(" (PAGE)"); }
                else if (l3_entry & 0x1) == 0 { print_str_uart(" (INVALID!)"); }
                print_str_uart("\r\n");
            }
        }
    }

    // Debug: Print expected stack range
    print_str_uart("  Expected stack: 0x7FFC0000-0x80000000 (256KB)\r\n");
    if sp_el0 < 0x7FFC0000 || sp_el0 > 0x80000000 {
        print_str_uart("  WARNING: SP_EL0 outside expected stack range!\r\n");
    }
    if far < 0x7FFC0000 || far > 0x80000000 {
        print_str_uart("  WARNING: FAR outside expected stack range!\r\n");
    } else {
        print_str_uart("  FAR is within stack range - page table issue?\r\n");
    }

    // Special handling for init process (devd)
    if is_init {
        print_str_uart("  CRITICAL: init (devd) crashed!\r\n");
        print_str_uart("  Attempting recovery...\r\n");

        // Step 1: Kill all tasks and free slots immediately
        // (Safe because we're on exception stack, not any task's stack)
        print_str_uart("  Killing all tasks...\r\n");
        unsafe {
            let mut sched = kernel::task::scheduler();
            // Skip idle slots (0..MAX_CPUS) - idle tasks must never be killed
            for slot_idx in kernel::percpu::MAX_CPUS..kernel::task::MAX_TASKS {
                if let Some(task) = sched.task(slot_idx) {
                    let pid = task.id;
                    print_str_uart("  slot[");
                    print_hex_uart(slot_idx as u64);
                    print_str_uart("] pid=");
                    print_hex_uart(pid as u64);
                    print_str_uart(" '");
                    for c in task.name.iter().take_while(|&&c| c != 0) {
                        uart::putc(*c as char);
                    }
                    print_str_uart("'\r\n");

                    if slot_idx != 0 {  // Extra safety: skip idle (slot 0)
                        // Cleanup resources
                        kernel::bus::process_cleanup(pid);
                        kernel::shmem::begin_cleanup(pid);
                kernel::shmem::finalize_cleanup(pid);
                        kernel::irq::process_cleanup(pid);
                        kernel::pci::release_all_devices(pid);
                        let port_wake = kernel::ipc::port_cleanup_task(pid);
                        kernel::ipc::waker::wake(&port_wake, kernel::ipc::WakeReason::Closed);
                        let ipc_peers = kernel::ipc::process_cleanup(pid);
                        for peer in ipc_peers.iter() {
                            let wake_list = kernel::object_service::object_service()
                                .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::CLOSED);
                            kernel::ipc::waker::wake(&wake_list, kernel::ipc::WakeReason::Closed);
                        }

                        // Free slot immediately (bump generation to invalidate stale PIDs)
                        sched.bump_generation(slot_idx);
                        sched.clear_slot(slot_idx);
                    }
                }
            }
        }

        // Step 2: Respawn devd
        // (Buses already reset to Safe by process_cleanup above)
        print_str_uart("  Respawning devd...\r\n");
        match elf::spawn_from_path("bin/devd") {
            Ok((new_pid, slot)) => {
                print_str_uart("  devd restarted as PID ");
                print_hex_uart(new_pid as u64);
                print_str_uart("\r\n");

                // Give devd ALL capabilities, high priority, and mark as init
                unsafe {
                    if let Some(task) = kernel::task::scheduler().task_mut(slot) {
                        task.set_capabilities(kernel::caps::Capabilities::ALL);
                        task.set_priority(kernel::task::Priority::High);  // devd is critical
                        task.is_init = true;  // Mark as init for heartbeat watchdog
                    }
                }

                // Schedule devd
                unsafe {
                    let mut sched = kernel::task::scheduler();
                    kernel::task::set_current_slot(slot);
                    let cpu = kernel::percpu::cpu_id();
                    if let Some(task) = sched.task_mut(slot) {
                        let _ = task.set_running(cpu);
                        // Update globals directly (can't call update_current_task_globals
                        // here because we already hold the scheduler lock)
                        let trap_ptr = &mut task.trap_frame as *mut kernel::task::TrapFrame;
                        kernel::percpu::set_trap_frame(trap_ptr);
                        if let Some(ref addr_space) = task.address_space {
                            kernel::percpu::set_ttbr0(addr_space.get_ttbr0());
                        }
                        // Set kernel stack top so assembly can set SP_EL1 before eret
                        let kstack_top = crate::arch::aarch64::mmu::phys_to_virt(
                            task.kernel_stack + task.kernel_stack_size as u64
                        );
                        kernel::percpu::set_kernel_stack_top(kstack_top);
                    }
                }

                print_str_uart("  Recovery complete, continuing.\r\n");
                return;  // Return to assembly, will eret to new devd
            }
            Err(e) => {
                print_str_uart("  FATAL: Failed to respawn devd: ");
                let err_str = match e {
                    elf::ElfError::BadMagic => "bad_magic",
                    elf::ElfError::Not64Bit => "not_64bit",
                    elf::ElfError::NotLittleEndian => "not_le",
                    elf::ElfError::NotExecutable => "not_exec",
                    elf::ElfError::WrongArch => "wrong_arch",
                    elf::ElfError::TooSmall => "too_small",
                    elf::ElfError::OutOfMemory => "oom",
                    elf::ElfError::InvalidSegment => "invalid_seg",
                    elf::ElfError::SignatureInvalid => "sig_invalid",
                };
                print_str_uart(err_str);
                print_str_uart("\r\n  System halted.\r\n");
                loop {
                    unsafe { core::arch::asm!("wfe"); }
                }
            }
        }
    }

    // Exit code: negative signal number (like Unix)
    // -11 = SIGSEGV for memory faults, -4 = SIGILL for illegal instruction
    let exit_code = match ec {
        0b100000 | 0b100100 => -11i32,  // SIGSEGV
        0b100010 | 0b100110 => -7i32,   // SIGBUS
        _ => -6i32,                      // SIGABRT
    };

    unsafe {
        let mut sched = kernel::task::scheduler();
        let current_slot = kernel::task::current_slot();

        // NOTE: Resource cleanup is deferred to reap_terminated() to avoid double-cleanup.
        // Just mark the task as Terminated here; reap_terminated() will handle:
        // - bus cleanup (stops DMA)
        // - shmem, scheme, pci, port, ipc cleanup
        // - closing file descriptors
        // - freeing task slot

        // Kill all children of this task
        for (_slot, task_opt) in sched.iter_tasks_mut() {
            if let Some(task) = task_opt {
                if task.parent_id == pid && !task.is_terminated() {
                    // Mark child as terminated
                    let _ = task.set_exiting(-9);  // SIGKILL
                }
            }
        }

        // Set exit code and mark as terminated
        if let Some(task) = sched.task_mut(current_slot) {
            let _ = task.set_exiting(exit_code);
        }

        // Wake parent if blocked (child exit notification via ProcessObject/Mux)
        if parent_id != 0 {
            if let Some(parent_slot) = sched.slot_by_pid(parent_id) {
                if let Some(parent) = sched.task_mut(parent_slot) {
                    if parent.is_blocked() {
                        let _ = parent.wake();
                    }
                }
            }
        }

        // Release scheduler lock before ObjectService notification
        drop(sched);

        // Notify parent via ObjectService (for Process watcher handles used by devd)
        if parent_id != 0 {
            use kernel::ipc::{waker, traits::WakeReason};
            let wake_list = kernel::object_service::object_service().notify_child_exit(
                parent_id, pid, exit_code,
            );
            waker::wake(&wake_list, WakeReason::ChildExit);
        }

        // Reacquire scheduler lock for task switching
        let mut sched = kernel::task::scheduler();

        print_str_uart("  Task terminated, switching to next...\r\n");

        // Schedule next task
        let cpu = kernel::percpu::cpu_id();
        if let Some(next_slot) = sched.schedule() {
            // Idle task is a kernel task - can't eret to it
            // Instead, enter the idle loop directly
            let my_idle = cpu as usize;
            if kernel::sched::is_idle_slot(next_slot) {
                print_str_uart("  Next task is idle, entering idle loop...\r\n");
                kernel::task::set_current_slot(my_idle);
                if let Some(task) = sched.task_mut(my_idle) {
                    let _ = task.set_running(cpu);
                }
                // Drop scheduler lock before entering idle (idle_entry never
                // returns, so the MutexGuard would never be dropped, deadlocking
                // when timer IRQ tries to acquire the scheduler lock).
                drop(sched);
                kernel::idle::idle_entry();
            }

            // Check if the next task has saved kernel context (was blocked in kernel mode).
            // If so, we can't eret to it - eret goes to userspace, but the task needs to
            // resume its kernel execution. Fall through to idle which will properly
            // context_switch to it.
            let needs_context_switch = sched.task(next_slot)
                .map(|t| t.needs_context_restore())
                .unwrap_or(false);

            if needs_context_switch {
                print_str_uart("  Next task needs context_switch, entering idle...\r\n");
                kernel::task::set_current_slot(my_idle);
                if let Some(task) = sched.task_mut(my_idle) {
                    let _ = task.set_running(cpu);
                }
                // Drop scheduler lock before entering idle (idle_entry never
                // returns, so the MutexGuard would never be dropped).
                drop(sched);
                kernel::idle::idle_entry();
            }

            // Switch address space BEFORE changing current slot
            if let Some(next) = sched.task(next_slot) {
                if let Some(ref addr_space) = next.address_space {
                    addr_space.activate();
                }
            }

            kernel::task::set_current_slot(next_slot);
            if let Some(task) = sched.task_mut(next_slot) {
                let _ = task.set_running(cpu);
                let trap_ptr = &mut task.trap_frame as *mut kernel::task::TrapFrame;
                kernel::percpu::set_trap_frame(trap_ptr);
                if let Some(ref addr_space) = task.address_space {
                    kernel::percpu::set_ttbr0(addr_space.get_ttbr0());
                }
                // Set kernel stack top so assembly can set SP_EL1 before eret
                let kstack_top = crate::arch::aarch64::mmu::phys_to_virt(
                    task.kernel_stack + task.kernel_stack_size as u64
                );
                kernel::percpu::set_kernel_stack_top(kstack_top);
            }
            // Return to assembly which will eret to next task
        } else {
            print_str_uart("  No more tasks - halting.\r\n");
            loop {
                core::arch::asm!("wfe");
            }
        }
    }
}

/// Exception handler called from assembly (kernel mode - fatal)
#[no_mangle]
pub extern "C" fn exception_handler_rust(esr: u64, elr: u64, far: u64) -> ! {
    // Flush any buffered UART output first so we don't lose pre-crash messages
    while uart::has_buffered_output() {
        uart::flush_buffer();
    }

    // Use only direct UART output - no println! which may fault
    // Use \r\n for proper terminal line endings
    print_str_uart("\r\n=== EXCEPTION ===\r\n");
    print_str_uart("  ESR: 0x");
    print_hex_uart(esr);
    print_str_uart("\r\n  ELR: 0x");
    print_hex_uart(elr);
    print_str_uart("\r\n  FAR: 0x");
    print_hex_uart(far);

    // Print SP_EL0 (user stack pointer)
    let sp_el0: u64;
    unsafe { core::arch::asm!("mrs {}, sp_el0", out(reg) sp_el0); }
    print_str_uart("\r\n  SP:  0x");
    print_hex_uart(sp_el0);

    // Print current slot and PID
    let slot = kernel::task::current_slot();
    print_str_uart("\r\n  SLOT: ");
    print_hex_uart(slot as u64);
    let pid = unsafe {
        kernel::task::scheduler().current_task_id().unwrap_or(0)
    };
    print_str_uart("\r\n  PID: ");
    print_hex_uart(pid as u64);

    // Decode exception class
    let ec = (esr >> 26) & 0x3f;
    print_str_uart("\r\n  EC:  0x");
    print_hex_uart(ec);
    print_str_uart(" = ");
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
    print_str_uart(exception_name);
    print_str_uart("\r\n=================\r\nSystem halted.\r\n");

    loop {
        unsafe { core::arch::asm!("wfe"); }
    }
}

/// Static flag to prevent re-entry during devd recovery
static DEVD_RECOVERY_IN_PROGRESS: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Flag set by liveness checker when devd is killed (checked by timer tick)
#[no_mangle]
pub static DEVD_LIVENESS_KILLED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Recover from devd failure (liveness timeout or other critical failure)
/// This kills all processes, resets buses, and respawns devd
#[no_mangle]
/// Recover from devd failure by resetting system state and respawning devd.
///
/// This follows the same pattern as initial boot:
/// 1. Kill all tasks (except idle)
/// 2. Reset buses to Safe state
/// 3. Spawn devd in Ready state
/// 4. Request reschedule - scheduler will pick up devd naturally
///
/// We don't enter devd directly - we let the normal scheduler handle it.
pub extern "C" fn recover_devd() {
    use core::sync::atomic::Ordering;

    // Prevent re-entry
    if DEVD_RECOVERY_IN_PROGRESS.swap(true, Ordering::SeqCst) {
        return;
    }

    print_str_uart("\r\n=== DEVD RECOVERY ===\r\n");
    print_str_uart("  Attempting recovery...\r\n");

    // Step 1: Collect PIDs under scheduler lock, then clean up without it.
    // Cleanup functions (shmem, ipc, bus) internally acquire the scheduler
    // lock, so we must NOT hold it during cleanup — that would deadlock.
    print_str_uart("  Killing all tasks...\r\n");

    // Phase 1a: Collect PIDs and clear slots under lock
    let mut pids_to_clean: [u32; kernel::task::MAX_TASKS] = [0; kernel::task::MAX_TASKS];
    let mut count = 0usize;
    unsafe {
        let mut sched = kernel::task::scheduler();
        for slot_idx in 1..kernel::task::MAX_TASKS {
            if let Some(task) = sched.task(slot_idx) {
                pids_to_clean[count] = task.id;
                count += 1;
                sched.bump_generation(slot_idx);
                sched.clear_slot(slot_idx);
            }
        }
    }
    // Scheduler lock released here

    // Phase 1b: Clean up resources for each PID without holding scheduler lock
    for i in 0..count {
        let pid = pids_to_clean[i];
        kernel::bus::process_cleanup(pid);
        kernel::shmem::begin_cleanup(pid);
                kernel::shmem::finalize_cleanup(pid);
        kernel::irq::process_cleanup(pid);
        kernel::pci::release_all_devices(pid);
        let port_wake = kernel::ipc::port_cleanup_task(pid);
        kernel::ipc::waker::wake(&port_wake, kernel::ipc::WakeReason::Closed);
        let ipc_peers = kernel::ipc::process_cleanup(pid);
        for peer in ipc_peers.iter() {
            let wake_list = kernel::object_service::object_service()
                .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::CLOSED);
            kernel::ipc::waker::wake(&wake_list, kernel::ipc::WakeReason::Closed);
        }
        kernel::object_service::object_service().remove_task_table(pid);
    }

    // Step 2: Reset all buses to Safe state
    print_str_uart("  Resetting buses to Safe...\r\n");
    kernel::bus::reset_all_buses();

    // Step 3: Respawn devd - same as boot
    print_str_uart("  Respawning devd...\r\n");
    match elf::spawn_from_path("bin/devd") {
        Ok((new_pid, slot)) => {
            print_str_uart("  devd spawned as PID ");
            print_hex_uart(new_pid as u64);
            print_str_uart(" slot ");
            print_hex_uart(slot as u64);
            print_str_uart("\r\n");

            // Configure devd (same as boot)
            unsafe {
                if let Some(task) = kernel::task::scheduler().task_mut(slot) {
                    task.set_capabilities(kernel::caps::Capabilities::ALL);
                    task.set_priority(kernel::task::Priority::High);
                    task.is_init = true;
                }
            }

            // Step 4: Request reschedule - scheduler will pick up devd
            // The task is in Ready state, scheduler will switch to it
            crate::arch::aarch64::sync::cpu_flags().set_need_resched();

            print_str_uart("  Recovery complete, scheduler will pick up devd.\r\n");
            DEVD_RECOVERY_IN_PROGRESS.store(false, Ordering::SeqCst);
        }
        Err(e) => {
            print_str_uart("  FATAL: Failed to respawn devd: ");
            let err_str = match e {
                elf::ElfError::BadMagic => "bad_magic",
                elf::ElfError::Not64Bit => "not_64bit",
                elf::ElfError::NotLittleEndian => "not_le",
                elf::ElfError::NotExecutable => "not_exec",
                elf::ElfError::WrongArch => "wrong_arch",
                elf::ElfError::TooSmall => "too_small",
                elf::ElfError::OutOfMemory => "oom",
                elf::ElfError::InvalidSegment => "invalid_seg",
                elf::ElfError::SignatureInvalid => "sig_invalid",
            };
            print_str_uart(err_str);
            print_str_uart("\r\n");

            // Hardware reboot (never returns)
            print_str_uart("  Attempting system reboot...\r\n");
            kernel::syscall::hardware_reset();
        }
    }
}

/// Initialize ramfs from embedded or external initrd
fn init_ramfs() {
    // First, try embedded initrd (compiled into kernel)
    if let Some((addr, size)) = initrd::get_embedded_initrd() {
        let count = ramfs::init(addr, size);
        kinfo!("ramfs", "loaded"; source = "embedded", files = count);
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
        kinfo!("ramfs", "loaded"; source = "external", files = count);
    } else {
        kwarn!("ramfs", "not_found"; hint = "build with mkinitrd.sh");
    }
}
