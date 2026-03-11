//! Panic handler for bare-metal kernel

use core::panic::PanicInfo;
use crate::println_direct;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Prevent re-entrant panics from looping forever.
    // If we panic inside the panic handler, halt immediately.
    use core::sync::atomic::{AtomicBool, Ordering};
    static PANICKING: AtomicBool = AtomicBool::new(false);
    if PANICKING.swap(true, Ordering::Relaxed) {
        // Re-entrant panic — halt without touching any locks
        loop {
            unsafe { core::arch::asm!("wfe"); }
        }
    }

    // Try to flush pending log messages — skip if lock is held
    crate::klog::try_flush();

    // Flush UART output buffer (userspace writes, lock-free)
    while crate::platform::current::uart::has_buffered_output() {
        crate::platform::current::uart::flush_buffer();
    }

    // Reset terminal so panic output is visible above shell decorations:
    // - Reset scroll region (CSI r)
    // - Clear screen (CSI 2J)
    // - Move cursor to top-left (CSI H)
    // - Show cursor if hidden (CSI ?25h)
    crate::platform::current::uart::write_bytes(b"\x1b[r\x1b[2J\x1b[H\x1b[?25h");

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

    // Current task info — use try_with_scheduler to avoid deadlock
    // if the scheduler lock is already held
    let slot = crate::kernel::task::current_slot();
    let task_info = crate::kernel::task::try_with_scheduler(|sched| {
        if let Some(t) = sched.task(slot) {
            t.id
        } else {
            0
        }
    });
    println_direct!("    SLOT:    {}", slot);
    if let Some(pid) = task_info {
        println_direct!("    PID:     {}", pid);
        let name: [u8; 16] = crate::kernel::process::process_table()
            .with(slot, |p| p.name)
            .unwrap_or([0u8; 16]);
        let name_len = name.iter().position(|&b| b == 0).unwrap_or(name.len());
        if name_len > 0 {
            if let Ok(s) = core::str::from_utf8(&name[..name_len]) {
                println_direct!("    NAME:    {}", s);
            }
        }
    } else {
        println_direct!("    PID:     (scheduler locked)");
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
