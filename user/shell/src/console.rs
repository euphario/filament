//! Console I/O via ring buffer
//!
//! High-performance I/O using ConsoleRing shared memory.
//! Shell connects to consoled port, receives shmem_id, maps the ring.
//!
//! ## Protocol
//!
//! 1. Connect to "console:" port (Channel)
//! 2. Receive "RING" + shmem_id + cols + rows message
//! 3. Map the shmem as ConsoleRing
//! 4. Use TX ring for output, RX ring for input

use userlib::ipc::{Channel, Mux, MuxFilter};
use userlib::console_ring::ConsoleRing;

/// Console I/O state
pub struct Console {
    /// Ring buffer for I/O (primary path once connected)
    ring: Option<ConsoleRing>,
    /// Channel used during handshake (kept alive to maintain connection)
    handshake_channel: Option<Channel>,
    /// Terminal dimensions
    pub cols: u16,
    pub rows: u16,
    /// Log split enabled
    pub log_split: bool,
}

impl Console {
    /// Create new console I/O (does not connect yet)
    pub const fn new() -> Self {
        Self {
            ring: None,
            handshake_channel: None,
            cols: 80,
            rows: 24,
            log_split: false,
        }
    }

    /// Connect to consoled
    /// Returns true if connected, false if falling back to direct UART
    pub fn connect(&mut self) -> bool {
        // Connect to consoled port
        let mut channel = match Channel::connect(b"console:") {
            Ok(ch) => ch,
            Err(_) => return false,
        };

        // Wait for RING message using mux
        let mux = match Mux::new() {
            Ok(m) => m,
            Err(_) => return false,
        };

        if mux.add(channel.handle(), MuxFilter::Readable).is_err() {
            return false;
        }

        // Block until channel is readable
        if mux.wait().is_err() {
            return false;
        }

        // Read the RING message
        let mut buf = [0u8; 16];
        let n = match channel.recv(&mut buf) {
            Ok(n) => n,
            Err(_) => return false,
        };

        // Parse: "RING" + shmem_id (4 bytes) + cols (2 bytes) + rows (2 bytes)
        if n < 12 || &buf[..4] != b"RING" {
            return false;
        }

        let shmem_id = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
        let cols = u16::from_le_bytes([buf[8], buf[9]]);
        let rows = u16::from_le_bytes([buf[10], buf[11]]);

        // Map the ring
        let ring = match ConsoleRing::map(shmem_id) {
            Some(r) => r,
            None => return false,
        };

        self.ring = Some(ring);
        self.handshake_channel = Some(channel);
        self.cols = cols;
        self.rows = rows;

        true
    }

    /// Check if connected to consoled
    pub fn is_connected(&self) -> bool {
        self.ring.is_some()
    }

    /// Read a single byte (blocking)
    /// Uses RX ring (consoled -> shell)
    pub fn read_byte(&mut self) -> Option<u8> {
        let ring = self.ring.as_ref()?;

        // First check if data is available
        loop {
            let available = ring.rx_available();
            if available > 0 {
                let mut byte = [0u8; 1];
                let n = ring.rx_read(&mut byte);
                if n > 0 {
                    return Some(byte[0]);
                }
            }

            // No data - wait indefinitely for notification
            // The shmem pending_for mechanism ensures we don't miss notifications
            ring.wait(0);
        }
    }

    /// Write bytes to console
    /// Uses TX ring (shell -> consoled)
    pub fn write(&self, data: &[u8]) {
        let ring = match &self.ring {
            Some(r) => r,
            None => return, // Not connected, drop output
        };

        let mut offset = 0;
        while offset < data.len() {
            let written = ring.tx_write(&data[offset..]);

            if written > 0 {
                offset += written;
                // Notify consoled that data is available
                ring.notify();
            } else {
                // TX ring full - wait a bit for consoled to drain
                if !ring.wait(1) {
                    userlib::syscall::sleep_us(1_000); // 1ms backoff
                }
            }
        }
    }

    /// Write a string to console
    pub fn write_str(&self, s: &str) {
        self.write(s.as_bytes());
    }

    /// Query terminal size from consoled
    /// Returns (cols, rows) on success
    pub fn query_size(&mut self) -> Option<(u16, u16)> {
        let ring = self.ring.as_ref()?;

        // Send GETSIZE query via TX ring
        ring.tx_write(b"GETSIZE\n");
        ring.notify();

        // Wait for SIZE response in RX ring
        for _ in 0..100 {
            if ring.rx_available() >= 5 {
                let mut buf = [0u8; 32];
                let n = ring.rx_read(&mut buf);
                if n >= 5 && &buf[..5] == b"SIZE " {
                    self.parse_size_msg(&buf[..n]);
                    return Some((self.cols, self.rows));
                }
            }
            // If wait fails, use sleep_us backoff to prevent storm
            if !ring.wait(10) {
                userlib::syscall::sleep_us(10_000); // 10ms backoff
            }
        }

        // Timeout - return cached size
        Some((self.cols, self.rows))
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
        userlib::syscall::sleep_us(100_000); // 100ms
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
