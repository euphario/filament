//! Kernel interface library for BPI-R4 kernel
//!
//! Thin boundary layer: syscalls, IPC primitives, error types, allocator.
//! Driver SDK lives in `libos`.

#![no_std]

extern crate alloc;

pub mod allocator;

#[global_allocator]
static ALLOCATOR: allocator::FilamentAlloc = allocator::FilamentAlloc::new();

pub mod error;
pub mod syscall;
pub mod ipc;
pub mod io;
pub mod half_ring;
pub mod ring;
pub mod data_port;
pub mod mailbox;
pub mod sync;

pub use error::{SysError, SysResult};
pub use syscall::{LogLevel, Handle, ObjHandle, ObjectType};
pub use syscall::{exit, exec, klog};
pub use syscall::{open, read, write, map, close, channel_pair};
pub use ipc::{Channel, Port, Timer, Mux, MuxFilter, MuxEvent, Process, Message, ObjHandle as IpcHandle, EventLoop, Shmem, PciDevice, Msi, MsiInfo, PipeRing};
pub use ring::{Ring, LayeredRing, IoSqe, IoCqe, SideEntry, PoolAlloc, io_op, io_status, side_msg, side_status, Doorbell, ChannelDoorbell};
pub use data_port::{DataPort, DataPortConfig, PortRole, Layer, ConnectedLayer, GeometryInfo};

// Panic flush callback — libos registers ulog::flush here at startup
static mut PANIC_FLUSH: Option<fn()> = None;

/// Register a function to call before panic output (e.g., ulog::flush).
pub fn set_panic_flush(f: fn()) {
    unsafe { PANIC_FLUSH = Some(f); }
}

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
    // Flush any buffered log records before they're lost
    if let Some(flush) = unsafe { PANIC_FLUSH } {
        flush();
    }

    // Log location: "PANIC: src/mcu.rs:123"
    let mut buf = [0u8; 80];
    let loc_len = if let Some(loc) = info.location() {
        let file = loc.file().as_bytes();
        let flen = file.len().min(50);
        buf[..7].copy_from_slice(b"PANIC: ");
        buf[7..7 + flen].copy_from_slice(&file[..flen]);
        let mut pos = 7 + flen;
        // Append :line
        buf[pos] = b':';
        pos += 1;
        pos += fmt_u32(&mut buf[pos..], loc.line());
        pos
    } else {
        buf[..14].copy_from_slice(b"PANIC: unknown");
        14
    };
    syscall::klog(syscall::LogLevel::Error, &buf[..loc_len]);

    // Log panic message if available (second klog call)
    let mut msg_buf = [0u8; 128];
    struct BufWriter<'a> { buf: &'a mut [u8], pos: usize }
    impl core::fmt::Write for BufWriter<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let bytes = s.as_bytes();
            let n = bytes.len().min(self.buf.len() - self.pos);
            self.buf[self.pos..self.pos + n].copy_from_slice(&bytes[..n]);
            self.pos += n;
            Ok(())
        }
    }
    let msg_len = {
        let mut w = BufWriter { buf: &mut msg_buf, pos: 0 };
        use core::fmt::Write;
        let _ = write!(w, "{}", info.message());
        w.pos
    };
    if msg_len > 0 {
        syscall::klog(syscall::LogLevel::Error, &msg_buf[..msg_len]);
    }

    syscall::exit(1)
}

/// Format u32 as decimal into buffer, return bytes written.
fn fmt_u32(buf: &mut [u8], mut n: u32) -> usize {
    if n == 0 && !buf.is_empty() {
        buf[0] = b'0';
        return 1;
    }
    let mut tmp = [0u8; 10];
    let mut i = 0;
    while n > 0 && i < 10 {
        tmp[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let len = i.min(buf.len());
    for j in 0..len {
        buf[j] = tmp[i - 1 - j];
    }
    len
}
