//! User-space runtime library for BPI-R4 kernel
//!
//! Minimal library for unified 5-syscall interface.

#![no_std]

pub mod error;
pub mod syscall;
pub mod ipc;
pub mod io;

pub use error::{SysError, SysResult};
pub use syscall::{LogLevel, Handle, ObjHandle, ObjectType};
pub use syscall::{exit, exec, klog};
pub use syscall::{open, read, write, map, close, channel_pair};
pub use ipc::{Channel, Port, Timer, Mux, MuxFilter, MuxEvent, Process, Message, ObjHandle as IpcHandle, EventLoop, Shmem, PciDevice, Msi, MsiInfo};

// Entry point - called by _start
unsafe extern "Rust" {
    fn main();
}

/// Program entry point
#[unsafe(no_mangle)]
#[unsafe(naked)]
#[unsafe(link_section = ".text._start")]
pub extern "C" fn _start() -> ! {
    core::arch::naked_asm!(
        // Debug: write "_ST\n" to UART via syscall before anything else
        // Store message on stack: "_ST\n" = 0x5f 0x53 0x54 0x0a
        "sub sp, sp, #16",           // Allocate stack space
        "mov x0, #0x5f",             // '_'
        "strb w0, [sp]",
        "mov x0, #0x53",             // 'S'
        "strb w0, [sp, #1]",
        "mov x0, #0x54",             // 'T'
        "strb w0, [sp, #2]",
        "mov x0, #0x0a",             // '\n'
        "strb w0, [sp, #3]",
        // Syscall: debug_write(buf=sp, len=4)
        "mov x0, sp",                // buf pointer
        "mov x1, #4",                // length
        "mov x8, #1",                // DEBUG_WRITE syscall
        "svc #0",
        // Clean up stack
        "add sp, sp, #16",
        // Now call main
        "bl {main}",
        "mov x8, #0",
        "svc #0",
        "b .",
        main = sym main,
    )
}

/// Panic handler
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Minimal panic - just log and exit
    let mut buf = [0u8; 64];
    let msg = if let Some(loc) = info.location() {
        let file = loc.file().as_bytes();
        let len = file.len().min(40);
        buf[..7].copy_from_slice(b"PANIC: ");
        buf[7..7+len].copy_from_slice(&file[..len]);
        &buf[..7+len]
    } else {
        b"PANIC: unknown"
    };
    syscall::klog(syscall::LogLevel::Error, msg);
    syscall::exit(1)
}
