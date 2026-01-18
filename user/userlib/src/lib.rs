//! User-space runtime library for BPI-R4 kernel
//!
//! Provides syscall wrappers, entry point, and panic handler for user programs.

#![no_std]

pub mod error;
pub mod syscall;
pub mod io;
pub mod log;
pub mod ulog;      // Unified logging framework
pub mod utrace;    // Unified tracing framework (spans, call depth)
pub mod serialize; // Shared serialization for ulog/utrace
pub mod ansi;      // ANSI terminal colors and styles
pub mod ring;
pub mod byte_ring; // Simple byte FIFO for console I/O
pub mod mmio;
pub mod trace;
pub mod firmware;
pub mod ipc;

pub use error::{SysError, SysResult, check_errno, check_errno_i32};
pub use syscall::*;
pub use syscall::{EventFilter, kevent_subscribe, kevent_unsubscribe, kevent_timer, kevent_wait};
pub use io::{Stdout, Stdin, Stderr};
pub use log::flush as flush_log;
pub use ring::{Ring, BlockRing, BlockRequest, BlockResponse};
pub use byte_ring::{ByteRing, ByteRingError};
pub use mmio::{MmioRegion, delay_ms, delay_us, poll_until, poll_interval};
pub use firmware::{FirmwareClient, FirmwareRequest, FirmwareReply};

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
