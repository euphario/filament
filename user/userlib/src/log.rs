//! Buffered logging for userspace drivers
//!
//! Provides non-blocking logging that accumulates in a ring buffer.
//! Messages are flushed to UART at explicit flush points, avoiding
//! the timing delays of direct UART output.
//!
//! Usage:
//!   logln!("message");          // Deferred, non-blocking
//!   traceln!("debug info");     // Debug output (can be compiled out)
//!   flush_log();                // Call at safe points to drain buffer
//!   println!("critical");       // Direct output for critical messages

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicUsize, Ordering};

/// Log buffer size (4KB)
const LOG_BUFFER_SIZE: usize = 4096;

/// Ring buffer for log messages
pub struct LogBuffer {
    buffer: [u8; LOG_BUFFER_SIZE],
    write_pos: AtomicUsize,
    read_pos: AtomicUsize,
}

impl LogBuffer {
    pub const fn new() -> Self {
        Self {
            buffer: [0; LOG_BUFFER_SIZE],
            write_pos: AtomicUsize::new(0),
            read_pos: AtomicUsize::new(0),
        }
    }

    /// Write a byte to the ring buffer
    /// Returns false if buffer is full
    #[inline]
    fn push(&mut self, byte: u8) -> bool {
        let write = self.write_pos.load(Ordering::Relaxed);
        let read = self.read_pos.load(Ordering::Acquire);

        let next_write = (write + 1) % LOG_BUFFER_SIZE;

        if next_write == read {
            return false; // Buffer full
        }

        self.buffer[write] = byte;
        self.write_pos.store(next_write, Ordering::Release);
        true
    }

    /// Check if buffer has data
    #[inline]
    pub fn has_data(&self) -> bool {
        self.write_pos.load(Ordering::Acquire) != self.read_pos.load(Ordering::Acquire)
    }

    /// Get number of bytes in buffer
    #[inline]
    pub fn len(&self) -> usize {
        let write = self.write_pos.load(Ordering::Acquire);
        let read = self.read_pos.load(Ordering::Acquire);
        if write >= read {
            write - read
        } else {
            LOG_BUFFER_SIZE - read + write
        }
    }

    /// Drain buffer contents into a slice, return bytes written
    pub fn drain(&mut self, out: &mut [u8]) -> usize {
        let mut count = 0;
        let write = self.write_pos.load(Ordering::Acquire);
        let mut read = self.read_pos.load(Ordering::Relaxed);

        while read != write && count < out.len() {
            out[count] = self.buffer[read];
            read = (read + 1) % LOG_BUFFER_SIZE;
            count += 1;
        }

        self.read_pos.store(read, Ordering::Release);
        count
    }
}

impl Write for LogBuffer {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            if byte == b'\n' {
                let _ = self.push(b'\r'); // Add CR for terminals
            }
            if !self.push(byte) {
                // Buffer full - drop remaining characters
                break;
            }
        }
        Ok(())
    }
}

/// Global log buffer
pub static mut LOG_BUFFER: LogBuffer = LogBuffer::new();

/// Flush log buffer to UART via syscall
/// Call this at safe points (after completing a request, during idle, etc.)
pub fn flush() {
    // Drain in chunks to avoid large stack allocation
    let mut chunk = [0u8; 256];

    loop {
        let n = unsafe {
            let buffer = &mut *core::ptr::addr_of_mut!(LOG_BUFFER);
            buffer.drain(&mut chunk)
        };

        if n == 0 {
            break;
        }

        // Write chunk to stdout via syscall
        crate::syscall::write(crate::syscall::STDOUT, &chunk[..n]);
    }
}

/// Check if log buffer has pending data
pub fn has_pending() -> bool {
    unsafe { (*core::ptr::addr_of!(LOG_BUFFER)).has_data() }
}

/// Get number of bytes pending in log buffer
pub fn pending_bytes() -> usize {
    unsafe { (*core::ptr::addr_of!(LOG_BUFFER)).len() }
}

/// Log formatted output to the ring buffer (non-blocking, deferred)
#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        #[allow(unused_unsafe)]
        unsafe {
            let _ = write!(&mut *core::ptr::addr_of_mut!($crate::log::LOG_BUFFER), $($arg)*);
        }
    }};
}

/// Log with newline (non-blocking, deferred)
#[macro_export]
macro_rules! logln {
    () => ($crate::log!("\n"));
    ($($arg:tt)*) => {{
        $crate::log!($($arg)*);
        $crate::log!("\n");
    }};
}
