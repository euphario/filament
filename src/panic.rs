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
