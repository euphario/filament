//! Panic handler for bare-metal kernel

use core::panic::PanicInfo;
use crate::println;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Flush any pending log messages before panic output
    crate::kernel::log::flush();

    // Flush UART output buffer (userspace writes)
    while crate::platform::mt7988::uart::has_buffered_output() {
        crate::platform::mt7988::uart::flush_buffer();
    }

    println!();
    println!("=== KERNEL PANIC ===");

    if let Some(location) = info.location() {
        println!(
            "  at {}:{}:{}",
            location.file(),
            location.line(),
            location.column()
        );
    }

    if let Some(message) = info.message().as_str() {
        println!("  message: {}", message);
    }

    println!("====================");
    println!();
    println!("System halted.");

    // Halt the CPU
    loop {
        unsafe {
            core::arch::asm!("wfe");
        }
    }
}
