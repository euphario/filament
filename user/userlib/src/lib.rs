//! User-space runtime library for BPI-R4 kernel
//!
//! Provides syscall wrappers, entry point, and panic handler for user programs.

#![no_std]

pub mod syscall;
pub mod io;
pub mod ring;
pub mod mmio;
pub mod trace;

pub use syscall::*;
pub use io::{Stdout, Stdin, Stderr};
pub use ring::{Ring, BlockRing, BlockRequest, BlockResponse};
pub use mmio::{MmioRegion, delay_ms, delay_us, poll_until, poll_interval};

// Entry point - called by _start
unsafe extern "Rust" {
    fn main();
}

/// Program entry point
/// The kernel sets up SP before jumping here, so we just call main and exit.
#[unsafe(no_mangle)]
#[unsafe(naked)]
#[unsafe(link_section = ".text._start")]
pub extern "C" fn _start() -> ! {
    core::arch::naked_asm!(
        // Stack is already set up by kernel
        // Call main
        "bl {main}",
        // Exit with return value from main (in x0)
        "mov x8, #0",    // SYS_EXIT
        "svc #0",
        // Should never reach here
        "b .",
        main = sym main,
    )
}

/// Panic handler - prints message and exits
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;
    let _ = write!(&mut io::Stderr, "PANIC: ");
    if let Some(location) = info.location() {
        let _ = writeln!(&mut io::Stderr, "{}:{}", location.file(), location.line());
    } else {
        let _ = writeln!(&mut io::Stderr, "unknown location");
    }
    syscall::exit(1)
}
