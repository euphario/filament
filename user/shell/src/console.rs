//! Console I/O via consoled
//!
//! All shell I/O goes through consoled channel - no direct UART access.
//! Shell must connect to consoled before producing any output.
//!
//! Uses the unified 5-syscall interface (open, read, write, map, close).

use userlib::ipc::Channel;

/// Receive buffer size - holds one complete message from consoled
const RX_BUFFER_SIZE: usize = 64;

/// Console I/O state
pub struct Console {
    /// Channel to consoled (if connected)
    channel: Option<Channel>,
    /// Terminal dimensions
    pub cols: u16,
    pub rows: u16,
    /// Log split enabled
    pub log_split: bool,
    /// Receive buffer for partial message handling
    rx_buf: [u8; RX_BUFFER_SIZE],
    /// Read position in rx_buf
    rx_read: usize,
    /// Write position in rx_buf (amount of data available)
    rx_write: usize,
}

impl Console {
    /// Create new console I/O (does not connect yet)
    pub const fn new() -> Self {
        Self {
            channel: None,
            cols: 80,
            rows: 24,
            log_split: false,
            rx_buf: [0u8; RX_BUFFER_SIZE],
            rx_read: 0,
            rx_write: 0,
        }
    }

    /// Connect to consoled
    /// Returns true if connected, false if falling back to direct UART
    pub fn connect(&mut self) -> bool {
        match Channel::connect(b"console:") {
            Ok(mut channel) => {
                // Query terminal size (query-response protocol)
                if channel.send(b"GETSIZE\n").is_ok() {
                    // Wait for SIZE response using event-driven blocking
                    use userlib::ipc::{Mux, MuxFilter};

                    if let Ok(mux) = Mux::new() {
                        // Add channel to mux, wait for readable
                        if mux.add(channel.handle(), MuxFilter::Readable).is_ok() {
                            // Block until channel is readable
                            if mux.wait().is_ok() {
                                // Now read the response
                                let mut buf = [0u8; 32];
                                if let Ok(n) = channel.recv(&mut buf) {
                                    self.parse_size_msg(&buf[..n]);
                                }
                            }
                        }
                    }
                }

                self.channel = Some(channel);
                true
            }
            Err(_) => {
                // Can't connect to consoled
                false
            }
        }
    }

    /// Parse "SIZE cols rows\n" message
    fn parse_size_msg(&mut self, data: &[u8]) {
        if data.len() < 5 || &data[..5] != b"SIZE " {
            return;
        }

        let rest = &data[5..];

        // Find space between cols and rows
        let mut cols: u16 = 0;
        let mut rows: u16 = 0;
        let mut i = 0;

        // Parse cols
        while i < rest.len() && rest[i] >= b'0' && rest[i] <= b'9' {
            cols = cols.saturating_mul(10).saturating_add((rest[i] - b'0') as u16);
            i += 1;
        }

        // Skip space
        if i < rest.len() && rest[i] == b' ' {
            i += 1;
        }

        // Parse rows
        while i < rest.len() && rest[i] >= b'0' && rest[i] <= b'9' {
            rows = rows.saturating_mul(10).saturating_add((rest[i] - b'0') as u16);
            i += 1;
        }

        if cols >= 10 && cols <= 500 && rows >= 10 && rows <= 500 {
            self.cols = cols;
            self.rows = rows;
        }
    }

    /// Check if connected to consoled
    pub fn is_connected(&self) -> bool {
        self.channel.is_some()
    }

    /// Read a single byte (blocking)
    /// All input comes from consoled channel - no direct UART access
    /// Uses internal buffer to handle message-oriented IPC where kernel
    /// delivers complete messages that may contain multiple bytes.
    pub fn read_byte(&mut self) -> Option<u8> {
        use userlib::ipc::{Mux, MuxFilter};
        use userlib::error::SysError;

        // First, check if we have buffered data
        if self.rx_read < self.rx_write {
            let byte = self.rx_buf[self.rx_read];
            self.rx_read += 1;
            return Some(byte);
        }

        // Buffer empty - need to receive more data
        let channel = self.channel.as_mut()?;

        // Loop until we get data - kernel returns WouldBlock if nothing queued
        loop {
            // Reset buffer positions before receiving
            self.rx_read = 0;
            self.rx_write = 0;

            match channel.recv(&mut self.rx_buf) {
                Ok(n) if n > 0 => {
                    // Got data - return first byte, keep rest in buffer
                    self.rx_write = n;
                    self.rx_read = 1;
                    return Some(self.rx_buf[0]);
                }
                Ok(_) => return None, // EOF
                Err(SysError::WouldBlock) => {
                    // No data yet - wait for channel to be readable using Mux
                    if let Ok(mux) = Mux::new() {
                        if mux.add(channel.handle(), MuxFilter::Readable).is_ok() {
                            let _ = mux.wait(); // Block until readable
                        }
                    }
                    // Loop back and try recv again
                }
                Err(_) => {
                    // Connection lost (not WouldBlock)
                    self.channel = None;
                    return None;
                }
            }
        }
    }

    /// Write bytes to console
    /// All output goes through consoled channel - no direct UART access
    pub fn write(&self, data: &[u8]) {
        if let Some(channel) = &self.channel {
            // Write to consoled channel only - retry if queue full
            let mut retries = 0u32;
            loop {
                match channel.send(data) {
                    Ok(()) => {
                        // Log if we had many retries (indicates backpressure)
                        if retries > 10 {
                            userlib::syscall::klog(
                                userlib::syscall::LogLevel::Warn,
                                b"[shell] write retries: high"
                            );
                        }
                        break;
                    }
                    Err(userlib::error::SysError::WouldBlock) => {
                        // Queue full - yield and retry
                        retries += 1;
                        userlib::syscall::sleep_us(100);
                    }
                    Err(_) => {
                        // Log unexpected errors
                        userlib::syscall::klog(
                            userlib::syscall::LogLevel::Error,
                            b"[shell] write failed"
                        );
                        break;
                    }
                }
            }
        }
        // If not connected, output is silently dropped
        // Shell must connect to consoled before producing output
    }

    /// Write a string to console
    pub fn write_str(&self, s: &str) {
        self.write(s.as_bytes());
    }

    /// Query terminal size from consoled
    /// Returns (cols, rows) on success
    pub fn query_size(&mut self) -> Option<(u16, u16)> {
        // For now, just return cached size
        // In the future, could send a query message to consoled
        Some((self.cols, self.rows))
    }

    /// Set log split (placeholder - no protocol for this yet)
    pub fn set_log_split(&mut self, enabled: bool) {
        self.log_split = enabled;
        // Future: send config message to consoled
    }

    /// Set log lines (placeholder)
    pub fn set_log_lines(&self, _lines: u8) {
        // Future: send config message to consoled
    }

    /// Connect/disconnect from logd (placeholder)
    pub fn set_logd_connected(&self, _connected: bool) {
        // Future: send config message to consoled
    }
}

/// Global console instance
static mut CONSOLE: Console = Console::new();

/// Initialize and connect to console (with retry)
pub fn init() -> bool {
    // Retry a few times - consoled might not be ready yet
    for _ in 0..10 {
        if unsafe { (*core::ptr::addr_of_mut!(CONSOLE)).connect() } {
            return true;
        }
        // Small delay before retry
        for _ in 0..100000 {
            core::hint::spin_loop();
        }
    }
    false
}

/// Get reference to global console
pub fn console() -> &'static Console {
    unsafe { &*core::ptr::addr_of!(CONSOLE) }
}

/// Get mutable reference to global console
pub fn console_mut() -> &'static mut Console {
    unsafe { &mut *core::ptr::addr_of_mut!(CONSOLE) }
}

/// Write bytes to console (convenience function)
pub fn write(data: &[u8]) {
    console().write(data);
}

/// Write string to console (convenience function)
pub fn write_str(s: &str) {
    console().write_str(s);
}

/// Read a byte from console (convenience function)
pub fn read_byte() -> Option<u8> {
    console_mut().read_byte()
}

/// Writer for core::fmt::Write trait
/// Used by the shell's print!/println! macros
pub struct ConsoleWriter;

impl core::fmt::Write for ConsoleWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        write(s.as_bytes());
        Ok(())
    }
}
