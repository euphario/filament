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

    if let Some(message) = info.message().as_str() {
        println_direct!("  message: {}", message);
    }

    println_direct!("====================");
    println_direct!();
    println_direct!("System halted.");

    // Halt the CPU
    loop {
        unsafe {
            core::arch::asm!("wfe");
        }
    }
}
