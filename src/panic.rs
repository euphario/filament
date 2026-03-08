//! Panic handler for bare-metal kernel

use core::panic::PanicInfo;
use crate::println_direct;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Flush any pending log messages before panic output
    crate::klog::flush();

    // Flush UART output buffer (userspace writes)
    while crate::platform::current::uart::has_buffered_output() {
        crate::platform::current::uart::flush_buffer();
    }

    // Use direct UART output for panic - we need immediate visibility
    println_direct!();
    println_direct!("=== KERNEL PANIC ===");

    if let Some(location) = info.location() {
        println_direct!(
            "  at {}:{}:{}",
            location.file(),
            location.line(),
            location.column()
        );
    }

    // message() returns fmt::Arguments — as_str() only works for literals,
    // so always use Display formatting to handle panic!("fmt {}", val)
    println_direct!("  {}", info.message());
    println_direct!();

    // CPU state
    let elr: u64;
    let spsr: u64;
    let sp_el0: u64;
    let lr: u64;
    let fp: u64;
    let current_el: u64;
    unsafe {
        core::arch::asm!("mrs {}, ELR_EL1", out(reg) elr);
        core::arch::asm!("mrs {}, SPSR_EL1", out(reg) spsr);
        core::arch::asm!("mrs {}, SP_EL0", out(reg) sp_el0);
        core::arch::asm!("mov {}, lr", out(reg) lr);
        core::arch::asm!("mov {}, x29", out(reg) fp);
        core::arch::asm!("mrs {}, CurrentEL", out(reg) current_el);
    }
    let el = (current_el >> 2) & 3;
    println_direct!("  CPU state:");
    println_direct!("    EL:      {}", el);
    println_direct!("    ELR_EL1: {:#018x}", elr);
    println_direct!("    SPSR:    {:#018x}", spsr);
    println_direct!("    SP_EL0:  {:#018x}", sp_el0);
    println_direct!("    LR:      {:#018x}", lr);
    println_direct!("    FP:      {:#018x}", fp);

    // Current task info
    let slot = crate::kernel::task::current_slot();
    let (pid, name) = crate::kernel::task::with_scheduler(|sched| {
        if let Some(t) = sched.task(slot) {
            (t.id, t.name)
        } else {
            (0, [0u8; 16])
        }
    });
    println_direct!("    SLOT:    {}", slot);
    println_direct!("    PID:     {}", pid);
    let name_len = name.iter().position(|&b| b == 0).unwrap_or(name.len());
    if name_len > 0 {
        if let Ok(s) = core::str::from_utf8(&name[..name_len]) {
            println_direct!("    NAME:    {}", s);
        }
    }

    // Uptime
    let ns = crate::platform::current::timer::now_ns();
    let secs = ns / 1_000_000_000;
    let ms = (ns % 1_000_000_000) / 1_000_000;
    println_direct!("    UPTIME:  {}.{:03}s", secs, ms);

    // Stack backtrace (frame pointer chain)
    println_direct!();
    println_direct!("  Backtrace (FP chain):");
    let mut frame = fp;
    for i in 0..16 {
        if frame == 0 || frame & 7 != 0 {
            break;
        }
        // Each frame: [fp+0] = prev_fp, [fp+8] = return_addr
        let prev_fp = unsafe { *(frame as *const u64) };
        let ret_addr = unsafe { *((frame + 8) as *const u64) };
        if ret_addr == 0 {
            break;
        }
        println_direct!("    #{:2}: {:#018x}", i, ret_addr);
        frame = prev_fp;
    }

    println_direct!();
    println_direct!("====================");
    println_direct!("System halted.");

    // Halt the CPU
    loop {
        unsafe {
            core::arch::asm!("wfe");
        }
    }
}
