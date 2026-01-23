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
        kinfo!("gic", "uart_irq_enabled"; irq = plat::irq::UART0);
    }

    // MMU info (already initialized by boot.S)
    kinfo!("mmu", "status"; ttbr0 = "user", ttbr1 = "kernel");
    mmu::print_info();

    // Physical memory manager
    {
        let _span = span!("pmm", "init");
        pmm::init();
        // Note: pmm::init() logs init_ok with details
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

    // Bus controllers
    {
        let _span = span!("bus", "init");
        bus::init(0);  // Kernel PID = 0
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
        kinfo!("timer", "init_ok");
        timer::print_info();
    }

    // SMP
    {
        let _span = span!("smp", "init");
        smp::init();
        kinfo!("smp", "init_ok");
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
        // IPC tests via ipc module
        // kernel::ipc::test(); // TODO: Add ipc unit tests
        kernel::event::test();
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
    klog::flush();  // DEBUG: flush after irq enable

    // =========================================================================
    // Phase 4: Start scheduler
    // =========================================================================

    // Start timer for preemption (10ms time slice)
    timer::start(10);
    kinfo!("timer", "preemption_started"; slice_ms = 10u64);
    klog::flush();  // DEBUG: flush after timer start
    kinfo!("kernel", "scheduler_ready");
    klog::flush();  // DEBUG: flush after scheduler ready

    // Spawn devd (init process) - after scheduler is ready
    kinfo!("kernel", "spawning_devd");
    klog::flush();
    let slot1 = {
        let _span = span!("elf", "spawn"; path = "bin/devd");
        match elf::spawn_from_path("bin/devd") {
            Ok((_task_id, slot)) => {
                kinfo!("kernel", "devd_spawned"; slot = slot);
                klog::flush();
                // Give devd ALL capabilities, high priority, and mark as init process
                unsafe {
                    if let Some(task) = task::scheduler().task_mut(slot) {
                        task.set_capabilities(kernel::caps::Capabilities::ALL);
                        task.set_priority(task::Priority::High);  // devd is critical
                        task.is_init = true;  // Mark as init for heartbeat watchdog
                    }
                }
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
                kerror!("kernel", "devd_spawn_failed"; err = err_str);
                None
            }
        }
    };

    if let Some(slot) = slot1 {
        // Flush all logs before entering userspace
        kinfo!("kernel", "entering_userspace"; slot = slot);
        klog::flush();
        ktrace::flush();

        // Enter user mode - scheduler will switch between processes
        // (boot span will be auto-closed when we context switch)
        unsafe {
            task::scheduler().run_user_task(slot);
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
            // Broadcast FdReadable event to all subscribers of STDIN (fd 0)
            kernel::event::broadcast_event(kernel::event::Event::fd_readable(0));

            // Also wake any process blocked in legacy blocking read syscall
            let blocked_pid = uart::get_blocked_pid();
            if blocked_pid != 0 {
                unsafe {
                    let sched = task::scheduler();
                    if sched.wake_by_pid(blocked_pid) {
                        uart::clear_blocked();
                        sync::cpu_flags().set_need_resched();
                    }
                }
            }
        }
    } else {
        // Check if this IRQ is registered by a userspace driver
        if let Some(owner_pid) = irq::notify(irq) {
            // Wake the owner process if blocked - O(1) lookup via pid_to_slot map
            unsafe {
                let _guard = sync::IrqGuard::new(); // Ensure atomicity
                let sched = task::scheduler();
                if sched.wake_by_pid(owner_pid) {
                    // Task was woken, set need_resched so we switch to it
                    sync::cpu_flags().set_need_resched();
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
/// This is where deferred work happens:
/// 1. Reschedule if timer set the flag
/// 2. Log any deferred messages (e.g., unhandled IRQs)
/// 3. Flush log buffer before returning to user
#[no_mangle]
pub extern "C" fn irq_exit_resched() {
    // 1. Handle deferred reschedule
    unsafe {
        task::do_resched_if_needed();
    }

    // 2. Log any unhandled IRQs from interrupt context (deferred logging)
    let (unhandled_count, last_irq) = sync::cpu_flags().get_unhandled_stats();
    if unhandled_count > 0 {
        kwarn!("irq", "unhandled"; count = unhandled_count as u64, last = last_irq as u64);
        sync::cpu_flags().clear_unhandled_stats();
    }

    // 3. Flush log buffer before returning to user
    klog::flush();
}

/// Print a hex value directly to UART (no allocations, no logging)
fn print_hex_uart(val: u64) {
    for i in (0..16).rev() {
        let nibble = ((val >> (i * 4)) & 0xf) as u8;
        let c = if nibble < 10 { b'0' + nibble } else { b'a' + nibble - 10 };
        uart::putc(c as char);
    }
}

fn print_str_uart(s: &str) {
    for c in s.chars() {
        uart::putc(c);
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
    let (pid, parent_id, task_name) = unsafe {
        let sched = kernel::task::scheduler();
        let slot = kernel::task::current_slot();
        if let Some(task) = sched.task(slot) {
            let mut name = [0u8; 16];
            let name_len = task.name.len().min(16);
            name[..name_len].copy_from_slice(&task.name[..name_len]);
            (task.id, task.parent_id, name)
        } else {
            (0, 0, [0u8; 16])
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

    // Special handling for PID 1 (devd/init)
    if pid == 1 {
        print_str_uart("  CRITICAL: init (devd) crashed!\r\n");
        print_str_uart("  Attempting recovery...\r\n");

        // Step 1: Kill all tasks and free slots immediately
        // (Safe because we're on exception stack, not any task's stack)
        print_str_uart("  Killing all tasks...\r\n");
        unsafe {
            let sched = kernel::task::scheduler();
            for slot_idx in 0..kernel::task::MAX_TASKS {
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

                    if pid != 0 {  // Don't kill kernel (PID 0 if it exists)
                        // Cleanup resources
                        kernel::bus::process_cleanup(pid);
                        kernel::shmem::process_cleanup(pid);
                        kernel::irq::process_cleanup(pid);
                        kernel::pci::release_all_devices(pid);
                        let port_wake = kernel::ipc::port_cleanup_task(pid);
                        kernel::ipc::waker::wake(&port_wake, kernel::ipc::WakeReason::Closed);
                        let ipc_wake = kernel::ipc::process_cleanup(pid);
                        kernel::ipc::waker::wake(&ipc_wake, kernel::ipc::WakeReason::Closed);

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
                    let sched = kernel::task::scheduler();
                    kernel::task::set_current_slot(slot);
                    if let Some(task) = sched.task_mut(slot) {
                        let _ = task.set_running();
                    }
                    kernel::task::update_current_task_globals();
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
        let sched = kernel::task::scheduler();
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

        // Notify parent
        if parent_id != 0 {
            if let Some(parent_slot) = sched.slot_by_pid(parent_id) {
                if let Some(parent) = sched.task_mut(parent_slot) {
                    let event = kernel::event::Event::child_exit(pid, exit_code);
                    parent.event_queue.push(event);
                    if parent.is_blocked() {
                        let _ = parent.wake();
                    }
                }
            }
        }

        print_str_uart("  Task terminated, switching to next...\r\n");

        // Schedule next task
        if let Some(next_slot) = sched.schedule() {
            kernel::task::set_current_slot(next_slot);
            if let Some(task) = sched.task_mut(next_slot) {
                let _ = task.set_running();
            }
            kernel::task::update_current_task_globals();
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

    // Print current PID (from scheduler)
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
pub extern "C" fn recover_devd() {
    use core::sync::atomic::Ordering;

    // Prevent re-entry
    if DEVD_RECOVERY_IN_PROGRESS.swap(true, Ordering::SeqCst) {
        return;
    }

    print_str_uart("\r\n=== DEVD LIVENESS TIMEOUT ===\r\n");
    print_str_uart("  devd failed to respond to liveness probe\r\n");
    print_str_uart("  Attempting recovery...\r\n");

    // Step 1: Kill all tasks and free slots immediately
    // (Safe because we're on exception/interrupt stack, not any task's stack)
    print_str_uart("  Killing all tasks...\r\n");
    unsafe {
        let sched = kernel::task::scheduler();
        for slot_idx in 0..kernel::task::MAX_TASKS {
            if let Some(task) = sched.task(slot_idx) {
                let pid = task.id;
                if pid != 0 {  // Don't kill kernel (PID 0 if it exists)
                    // Cleanup resources
                    kernel::bus::process_cleanup(pid);
                    kernel::shmem::process_cleanup(pid);
                    kernel::irq::process_cleanup(pid);
                    kernel::pci::release_all_devices(pid);
                    let port_wake = kernel::ipc::port_cleanup_task(pid);
                    kernel::ipc::waker::wake(&port_wake, kernel::ipc::WakeReason::Closed);
                    let ipc_wake = kernel::ipc::process_cleanup(pid);
                    kernel::ipc::waker::wake(&ipc_wake, kernel::ipc::WakeReason::Closed);

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
                let sched = kernel::task::scheduler();
                kernel::task::set_current_slot(slot);
                if let Some(task) = sched.task_mut(slot) {
                    let _ = task.set_running();
                }
                kernel::task::update_current_task_globals();
            }

            print_str_uart("  Recovery complete.\r\n");
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
        kinfo!("ramfs", "loaded"; source = "embedded", addr = klog::hex64(addr as u64), files = count);
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
        kinfo!("ramfs", "loaded"; source = "external", addr = klog::hex64(plat::INITRD_ADDR as u64), files = count);
        ramfs::list();
    } else {
        kwarn!("ramfs", "not_found"; hint = "build with mkinitrd.sh");
    }
}
