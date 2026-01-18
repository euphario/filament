//! I/O primitives for user programs
//!
//! Provides Stdout, Stdin, Stderr types and print!/println! macros.

use crate::syscall::{read, write, STDIN, STDOUT, STDERR};
use core::fmt::{self, Write};
use core::sync::atomic::{AtomicBool, Ordering};

/// Global flag to enable/disable stdout output
/// Disabled by daemons to avoid corrupting consoled's display
static STDOUT_ENABLED: AtomicBool = AtomicBool::new(true);

/// Disable stdout output (print!/println! become no-ops)
pub fn disable_stdout() {
    STDOUT_ENABLED.store(false, Ordering::Release);
}

/// Enable stdout output
pub fn enable_stdout() {
    STDOUT_ENABLED.store(true, Ordering::Release);
}

/// Check if stdout is enabled
pub fn stdout_enabled() -> bool {
    STDOUT_ENABLED.load(Ordering::Acquire)
}

/// Standard output writer
pub struct Stdout;

impl Write for Stdout {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        if !STDOUT_ENABLED.load(Ordering::Acquire) {
            return Ok(()); // Silently discard
        }
        let bytes = s.as_bytes();
        let mut written = 0;
        while written < bytes.len() {
            let n = write(STDOUT, &bytes[written..]);
            if n < 0 {
                return Err(fmt::Error);
            }
            written += n as usize;
        }
        Ok(())
    }
}

/// Standard error writer
pub struct Stderr;

impl Write for Stderr {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let mut written = 0;
        while written < bytes.len() {
            let n = write(STDERR, &bytes[written..]);
            if n < 0 {
                return Err(fmt::Error);
            }
            written += n as usize;
        }
        Ok(())
    }
}

/// Standard input reader
pub struct Stdin;

impl Stdin {
    /// Read a single byte (blocking)
    pub fn read_byte(&self) -> Option<u8> {
        let mut buf = [0u8; 1];
        let n = read(STDIN, &mut buf);
        if n > 0 {
            Some(buf[0])
        } else {
            None
        }
    }

    /// Read into a buffer (blocking for first byte)
    pub fn read(&self, buf: &mut [u8]) -> isize {
        read(STDIN, buf)
    }

    /// Read a line into a buffer (until newline or buffer full)
    /// Returns the number of bytes read (including newline if present)
    pub fn read_line(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        while pos < buf.len() {
            let mut byte = [0u8; 1];
            let n = read(STDIN, &mut byte);
            if n <= 0 {
                break;
            }
            buf[pos] = byte[0];
            pos += 1;
            if byte[0] == b'\n' || byte[0] == b'\r' {
                break;
            }
        }
        pos
    }
}

/// Print without newline - uses core::fmt
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let _ = write!(&mut $crate::io::Stdout, $($arg)*);
    }};
}

/// Print with newline - uses core::fmt
#[macro_export]
macro_rules! println {
    () => {{
        if $crate::io::stdout_enabled() {
            let _ = $crate::syscall::write($crate::syscall::STDOUT, b"\r\n");
        }
    }};
    ($($arg:tt)*) => {{
        if $crate::io::stdout_enabled() {
            use core::fmt::Write;
            // writeln! adds \n, we need to also add \r for serial terminals
            let _ = write!(&mut $crate::io::Stdout, $($arg)*);
            let _ = $crate::syscall::write($crate::syscall::STDOUT, b"\r\n");
        }
    }};
}

/// Print to stderr without newline
#[macro_export]
macro_rules! eprint {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let _ = write!(&mut $crate::io::Stderr, $($arg)*);
    }};
}

/// Print to stderr with newline
#[macro_export]
macro_rules! eprintln {
    () => {{
        let _ = $crate::syscall::write($crate::syscall::STDERR, b"\n");
    }};
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let _ = writeln!(&mut $crate::io::Stderr, $($arg)*);
    }};
}
