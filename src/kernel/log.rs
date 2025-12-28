//! Ring buffer logging for IRQ-safe deferred output
//!
//! This module provides a lock-free ring buffer for log messages.
//! Messages are accumulated in the buffer and flushed to UART at safe points
//! (syscall exit, idle loop, panic handler).
//!
//! Usage:
//!   log!("message");              // Deferred, IRQ-safe
//!   flush_log();                  // Called at safe points
//!   println!("message");          // Direct output (NOT IRQ-safe)

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicUsize, Ordering};

/// Log buffer size (16KB - enough for verbose boot output)
const LOG_BUFFER_SIZE: usize = 16384;

/// Ring buffer for log messages
/// Using raw array + atomic indices for lock-free operation
/// pub for macro access
pub struct LogBuffer {
    buffer: [u8; LOG_BUFFER_SIZE],
    /// Write position (only modified by log!, read by flush)
    write_pos: AtomicUsize,
    /// Read position (only modified by flush)
    read_pos: AtomicUsize,
}

impl LogBuffer {
    const fn new() -> Self {
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

        // Buffer full?
        if next_write == read {
            return false;
        }

        // SAFETY: We're the only writer (single CPU or with IRQs disabled)
        self.buffer[write] = byte;
        self.write_pos.store(next_write, Ordering::Release);
        true
    }

    /// Read a byte from the ring buffer
    /// Returns None if buffer is empty
    #[inline]
    fn pop(&mut self) -> Option<u8> {
        let write = self.write_pos.load(Ordering::Acquire);
        let read = self.read_pos.load(Ordering::Relaxed);

        // Buffer empty?
        if read == write {
            return None;
        }

        let byte = self.buffer[read];
        let next_read = (read + 1) % LOG_BUFFER_SIZE;
        self.read_pos.store(next_read, Ordering::Release);
        Some(byte)
    }

    /// Check if buffer has data
    #[inline]
    #[allow(dead_code)] // Infrastructure for future use
    fn has_data(&self) -> bool {
        self.write_pos.load(Ordering::Acquire) != self.read_pos.load(Ordering::Acquire)
    }

    /// Get number of bytes in buffer
    #[inline]
    #[allow(dead_code)] // Infrastructure for future use
    fn len(&self) -> usize {
        let write = self.write_pos.load(Ordering::Acquire);
        let read = self.read_pos.load(Ordering::Acquire);
        if write >= read {
            write - read
        } else {
            LOG_BUFFER_SIZE - read + write
        }
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
                // Could add overflow counter here
                break;
            }
        }
        Ok(())
    }
}

/// Global log buffer
/// pub for macro access
pub static mut LOG_BUFFER: LogBuffer = LogBuffer::new();

/// Flush log buffer to UART
/// Call this at safe points (syscall exit, idle loop, etc.)
pub fn flush() {
    // SAFETY: Single CPU, called from safe context
    let buffer = unsafe { &mut *core::ptr::addr_of_mut!(LOG_BUFFER) };

    while let Some(byte) = buffer.pop() {
        crate::platform::mt7988::uart::putc(byte as char);
    }
}

/// Try to flush log buffer to UART (non-blocking, IRQ-safe)
/// Returns true if flush was performed, false if UART lock was held
/// Safe to call from timer interrupt
pub fn try_flush() -> bool {
    // Try to acquire UART lock - if held by syscall, skip this tick
    let Some(uart) = crate::platform::mt7988::uart::UART.try_lock() else {
        return false;
    };

    // SAFETY: Single CPU, called from safe context
    let buffer = unsafe { &mut *core::ptr::addr_of_mut!(LOG_BUFFER) };

    while let Some(byte) = buffer.pop() {
        uart.putc(byte);
    }
    true
}

/// Check if log buffer has pending data
#[allow(dead_code)] // Infrastructure for future use
pub fn has_pending() -> bool {
    unsafe { (*core::ptr::addr_of!(LOG_BUFFER)).has_data() }
}

/// Get number of bytes pending in log buffer
#[allow(dead_code)] // Infrastructure for future use
pub fn pending_bytes() -> usize {
    unsafe { (*core::ptr::addr_of!(LOG_BUFFER)).len() }
}

/// Log formatted output (alias for print! - both use LOG_BUFFER)
#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => { $crate::print!($($arg)*) };
}

/// Log with newline (alias for println! - both use LOG_BUFFER)
#[macro_export]
macro_rules! logln {
    () => { $crate::println!() };
    ($($arg:tt)*) => { $crate::println!($($arg)*) };
}
