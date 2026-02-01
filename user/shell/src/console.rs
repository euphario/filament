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
use userlib::syscall::Handle;

/// IPC poll callback type — called during read_byte() when the shell-cmd port
/// has a pending connection. This lets the shell service remote commands even
/// while blocked waiting for console input.
pub type IpcPollFn = fn();

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
    /// Number of log lines (top region)
    pub log_lines: u16,
    /// IPC port handle to watch during read_byte() (for shell-cmd port)
    ipc_port_handle: Option<Handle>,
    /// Callback to drain IPC commands when port has activity
    ipc_poll_fn: Option<IpcPollFn>,
    /// Mux for watching both ring and IPC port (created once, reused)
    read_mux: Option<Mux>,
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
            log_lines: 5,
            ipc_port_handle: None,
            ipc_poll_fn: None,
            read_mux: None,
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
    /// Uses RX ring (consoled -> shell).
    /// If an IPC port handle is registered via set_ipc_poll(), also watches
    /// it and calls the poll callback when the port has activity (allowing
    /// remote shell commands to be serviced while waiting for console input).
    pub fn read_byte(&mut self) -> Option<u8> {
        let ring = self.ring.as_ref()?;

        // Fast path: data already available
        if ring.rx_available() > 0 {
            let mut byte = [0u8; 1];
            if ring.rx_read(&mut byte) > 0 {
                return Some(byte[0]);
            }
        }

        // Mux-based wait: watches both ring and IPC port
        if let (Some(mux), Some(port_handle), Some(poll_fn)) =
            (&self.read_mux, self.ipc_port_handle, self.ipc_poll_fn)
        {
            loop {
                if ring.rx_available() > 0 {
                    let mut byte = [0u8; 1];
                    if ring.rx_read(&mut byte) > 0 {
                        return Some(byte[0]);
                    }
                }

                match mux.wait() {
                    Ok(event) => {
                        if event.handle.0 == port_handle.0 {
                            poll_fn();
                        }
                    }
                    Err(_) => {
                        userlib::syscall::sleep_us(100_000);
                    }
                }
            }
        }

        // Fallback: no IPC port registered, simple ring.wait()
        loop {
            if ring.rx_available() > 0 {
                let mut byte = [0u8; 1];
                if ring.rx_read(&mut byte) > 0 {
                    return Some(byte[0]);
                }
            }

            if !ring.wait(0) {
                userlib::syscall::sleep_us(100_000);
            }
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

    /// Set log split mode
    pub fn set_log_split(&mut self, enabled: bool) {
        let ring = match &self.ring {
            Some(r) => r,
            None => return,
        };

        if enabled {
            // Send SPLIT command with current log_lines setting
            // For now use default 5 lines
            ring.tx_write(b"SPLIT 5\n");
        } else {
            ring.tx_write(b"NOSPLIT\n");
        }
        ring.notify();

        // Wait for OK response
        for _ in 0..50 {
            if ring.rx_available() >= 2 {
                let mut buf = [0u8; 16];
                let n = ring.rx_read(&mut buf);
                if n >= 2 && &buf[..2] == b"OK" {
                    self.log_split = enabled;
                    return;
                }
            }
            ring.wait(10);
        }
        // Timeout - assume it worked
        self.log_split = enabled;
    }

    /// Set number of log lines (only effective when split is enabled)
    pub fn set_log_lines(&mut self, lines: u8) {
        let ring = match &self.ring {
            Some(r) => r,
            None => return,
        };

        // Send SPLIT command with specified lines
        let mut cmd = [0u8; 16];
        cmd[..6].copy_from_slice(b"SPLIT ");
        let mut pos = 6;
        if lines >= 10 {
            cmd[pos] = b'0' + (lines / 10);
            pos += 1;
        }
        cmd[pos] = b'0' + (lines % 10);
        pos += 1;
        cmd[pos] = b'\n';
        pos += 1;

        ring.tx_write(&cmd[..pos]);
        ring.notify();

        // Wait for OK response
        for _ in 0..50 {
            if ring.rx_available() >= 2 {
                let mut buf = [0u8; 16];
                ring.rx_read(&mut buf);
                break;
            }
            ring.wait(10);
        }
    }

    /// Connect/disconnect from logd (placeholder - logd integration not yet implemented)
    pub fn set_logd_connected(&self, _connected: bool) {
        // Future: tell consoled to connect/disconnect from logd
    }

    /// Register an IPC port handle and poll callback. During read_byte(),
    /// the console will watch this handle alongside the ring and call the
    /// callback when the port has activity (pending connection).
    /// Creates a persistent Mux for efficient multiplexed waiting.
    pub fn set_ipc_poll(&mut self, handle: Handle, poll_fn: IpcPollFn) {
        self.ipc_port_handle = Some(handle);
        self.ipc_poll_fn = Some(poll_fn);

        // Create Mux and add both handles now (ring + port)
        if let Some(ref ring) = self.ring {
            if let Ok(mux) = Mux::new() {
                let _ = mux.add(ring.handle(), MuxFilter::Readable);
                let _ = mux.add(handle, MuxFilter::Readable);
                self.read_mux = Some(mux);
            }
        }
    }
}

/// Capture buffer for redirecting shell output to IPC responses.
/// Used when executing commands on behalf of remote shell clients.
pub struct CaptureBuffer {
    data: [u8; 8192],
    len: usize,
}

impl CaptureBuffer {
    pub const fn new() -> Self {
        Self {
            data: [0u8; 8192],
            len: 0,
        }
    }

    pub fn push(&mut self, bytes: &[u8]) {
        let available = self.data.len() - self.len;
        let to_copy = bytes.len().min(available);
        self.data[self.len..self.len + to_copy].copy_from_slice(&bytes[..to_copy]);
        self.len += to_copy;
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len]
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }
}

/// Global console instance
static mut CONSOLE: Console = Console::new();

/// Pointer to active capture buffer (set during IPC command execution)
static mut CAPTURE_BUF: Option<*mut CaptureBuffer> = None;

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

/// Begin capturing output into the given buffer.
/// While capturing, write() pushes to the buffer instead of ConsoleRing.
///
/// SAFETY: Caller must ensure the buffer outlives the capture session
/// and that end_capture() is called before the buffer is dropped.
pub fn begin_capture(buf: &mut CaptureBuffer) {
    unsafe {
        *core::ptr::addr_of_mut!(CAPTURE_BUF) = Some(buf as *mut CaptureBuffer);
    }
}

/// End output capture. Returns to normal console output.
pub fn end_capture() {
    unsafe {
        *core::ptr::addr_of_mut!(CAPTURE_BUF) = None;
    }
}

/// Check if output is currently being captured (for ANSI escape suppression).
pub fn is_capturing() -> bool {
    unsafe { (*core::ptr::addr_of!(CAPTURE_BUF)).is_some() }
}

/// Write bytes to console (convenience function)
pub fn write(data: &[u8]) {
    unsafe {
        if let Some(ptr) = *core::ptr::addr_of!(CAPTURE_BUF) {
            (*ptr).push(data);
            return;
        }
    }
    console().write(data);
}

/// Write string to console (convenience function)
pub fn write_str(s: &str) {
    write(s.as_bytes());
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
