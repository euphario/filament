//! Console I/O via consoled protocol
//!
//! Provides input/output through the console daemon (consoled) which manages
//! terminal regions for logs and shell interaction.
//!
//! ## Output Architecture
//!
//! Shell output uses a ring buffer for efficiency:
//! ```text
//! ┌─────────┐  write()   ┌──────────┐  Doorbell  ┌──────────┐
//! │  Shell  │ ──────────▶│ ByteRing │ ──────────▶│ consoled │
//! └─────────┘            └──────────┘  (IPC)     └──────────┘
//! ```
//!
//! - Shell creates ByteRing and sends shmem_id to consoled via SetupOutputRing
//! - Shell writes data directly to ring (no IPC per-write)
//! - Shell sends Doorbell message to wake consoled when data available
//! - consoled reads from ring and writes to display

use userlib::ipc::protocols::{
    ConsoleClient, ConsoleRequest, ConsoleResponse, InputState,
    MAX_INPUT_SIZE,
};
use userlib::ipc::Message;
use userlib::{syscall, ByteRing};

/// Console I/O state
pub struct Console {
    client: Option<ConsoleClient>,
    /// Output ring buffer (shell writes, consoled reads)
    output_ring: Option<ByteRing>,
    /// Input ring buffer (consoled writes, shell reads)
    input_ring: Option<ByteRing>,
    /// IPC channel for doorbell (separate from client for non-blocking sends)
    doorbell_channel: Option<u32>,
    /// Input buffer for data received from consoled (legacy or ring overflow)
    input_buf: [u8; MAX_INPUT_SIZE],
    input_len: usize,
    input_pos: usize,
    /// Fallback mode - use direct UART if consoled not available
    fallback: bool,
    /// Terminal dimensions
    pub cols: u16,
    pub rows: u16,
    /// Log split enabled
    pub log_split: bool,
}

/// Size of output ring buffer (4KB should be plenty for terminal output)
const OUTPUT_RING_SIZE: usize = 4096;

impl Console {
    /// Create new console I/O (does not connect yet)
    pub const fn new() -> Self {
        Self {
            client: None,
            output_ring: None,
            input_ring: None,
            doorbell_channel: None,
            input_buf: [0u8; MAX_INPUT_SIZE],
            input_len: 0,
            input_pos: 0,
            fallback: true,
            cols: 80,
            rows: 24,
            log_split: true,
        }
    }

    /// Connect to consoled
    /// Returns true if connected, false if falling back to direct UART
    pub fn connect(&mut self) -> bool {
        match ConsoleClient::connect() {
            Ok(client) => {
                // Send ready and get initial config
                match client.ready() {
                    Ok(ConsoleResponse::Ready { cols, rows, log_split }) => {
                        self.cols = cols;
                        self.rows = rows;
                        self.log_split = log_split;

                        // Create output ring buffer
                        if let Ok(ring) = ByteRing::create(OUTPUT_RING_SIZE) {
                            // Open a second channel for doorbell messages
                            // (ConsoleClient blocks on receive, we need non-blocking doorbell)
                            let doorbell_ch = syscall::port_connect(b"console");
                            if doorbell_ch >= 0 {
                                let ch = doorbell_ch as u32;

                                // Allow consoled to map our ring
                                // Get consoled's PID via channel peer lookup
                                let peer_pid = syscall::channel_get_peer(ch);
                                if peer_pid > 0 {
                                    let _ = ring.allow(peer_pid as u32);
                                }

                                // Send SetupOutputRing to tell consoled about the ring
                                let setup = ConsoleRequest::SetupOutputRing {
                                    shmem_id: ring.shmem_id(),
                                };
                                let mut buf = [0u8; 16];
                                if let Ok(len) = setup.serialize(&mut buf) {
                                    let _ = syscall::send(ch, &buf[..len]);
                                    // Wait for acknowledgment
                                    let mut resp = [0u8; 8];
                                    let _ = syscall::receive(ch, &mut resp);
                                }

                                self.output_ring = Some(ring);
                                self.doorbell_channel = Some(ch);
                            }
                        }

                        self.client = Some(client);
                        self.fallback = false;
                        true
                    }
                    _ => {
                        // Didn't get proper ready response, fall back
                        self.fallback = true;
                        false
                    }
                }
            }
            Err(_) => {
                // Can't connect to consoled, use direct UART
                self.fallback = true;
                false
            }
        }
    }

    /// Check if connected to consoled
    pub fn is_connected(&self) -> bool {
        !self.fallback && self.client.is_some()
    }

    /// Read a single byte (blocking)
    /// Compatible with Stdin interface for readline
    pub fn read_byte(&mut self) -> Option<u8> {
        // Return from buffer if we have data
        if self.input_pos < self.input_len {
            let b = self.input_buf[self.input_pos];
            self.input_pos += 1;
            return Some(b);
        }

        // Try to read from input ring first
        if let Some(ring) = &self.input_ring {
            let n = ring.read(&mut self.input_buf);
            if n > 0 {
                self.input_len = n;
                self.input_pos = 1;
                return Some(self.input_buf[0]);
            }
        }

        // Need more data
        if self.fallback {
            // Direct UART read
            let mut buf = [0u8; 1];
            let n = syscall::read(syscall::STDIN, &mut buf);
            if n > 0 {
                Some(buf[0])
            } else {
                None
            }
        } else if let Some(client) = &self.client {
            // Receive from consoled
            loop {
                match client.receive() {
                    Ok(ConsoleResponse::Input { data, len }) => {
                        // Legacy IPC input (fallback when ring not available)
                        let len = len as usize;
                        if len > 0 {
                            // Buffer the input
                            let copy_len = len.min(MAX_INPUT_SIZE);
                            self.input_buf[..copy_len].copy_from_slice(&data[..copy_len]);
                            self.input_len = copy_len;
                            self.input_pos = 1;
                            return Some(self.input_buf[0]);
                        }
                    }
                    Ok(ConsoleResponse::SetupInputRing { shmem_id }) => {
                        // consoled created an input ring - map it
                        if let Ok(ring) = ByteRing::map(shmem_id) {
                            self.input_ring = Some(ring);
                        }
                        // Continue waiting for input
                    }
                    Ok(ConsoleResponse::Doorbell) => {
                        // consoled notified us that input is available in ring
                        if let Some(ring) = &self.input_ring {
                            let n = ring.read(&mut self.input_buf);
                            if n > 0 {
                                self.input_len = n;
                                self.input_pos = 1;
                                return Some(self.input_buf[0]);
                            }
                        }
                        // Continue if nothing read (spurious doorbell)
                    }
                    Ok(ConsoleResponse::Resize { cols, rows }) => {
                        self.cols = cols;
                        self.rows = rows;
                        // Continue waiting for input
                    }
                    Ok(_) => {
                        // Other responses, continue
                    }
                    Err(_) => {
                        // Error - fall back to direct UART
                        return None;
                    }
                }
            }
        } else {
            None
        }
    }

    /// Write bytes to console
    pub fn write(&self, data: &[u8]) {
        if self.fallback {
            let _ = syscall::write(syscall::STDOUT, data);
        } else if let Some(ring) = &self.output_ring {
            // Write to ring buffer
            let mut offset = 0;
            while offset < data.len() {
                let written = ring.write(&data[offset..]);
                if written == 0 {
                    // Ring full - send doorbell and wait a bit for consoled to drain
                    self.send_doorbell();
                    syscall::yield_now();
                } else {
                    offset += written;
                }
            }
            // Send doorbell to notify consoled
            self.send_doorbell();
        } else if let Some(client) = &self.client {
            // Fallback to legacy IPC write (no ring available)
            let _ = client.write(data);
        }
    }

    /// Send doorbell to wake consoled
    fn send_doorbell(&self) {
        if let Some(ch) = self.doorbell_channel {
            let doorbell = ConsoleRequest::Doorbell;
            let mut buf = [0u8; 4];
            if let Ok(len) = doorbell.serialize(&mut buf) {
                // Non-blocking send - don't wait for response
                let _ = syscall::send(ch, &buf[..len]);
                // Drain any pending responses to avoid filling channel buffer
                let mut resp = [0u8; 8];
                let _ = syscall::receive_nonblock(ch, &mut resp);
            }
        }
    }

    /// Write a string to console
    pub fn write_str(&self, s: &str) {
        self.write(s.as_bytes());
    }

    /// Set input state (for dynamic region management)
    pub fn set_input_state(&self, state: InputState) {
        if let Some(client) = &self.client {
            let _ = client.set_input_state(state);
        }
    }

    /// Toggle log split
    pub fn set_log_split(&mut self, enabled: bool) {
        self.log_split = enabled;
        if let Some(client) = &self.client {
            let _ = client.set_log_split(enabled);
        }
    }

    /// Set log lines
    pub fn set_log_lines(&self, lines: u8) {
        if let Some(client) = &self.client {
            let _ = client.set_log_lines(lines);
        }
    }

    /// Connect/disconnect from logd
    pub fn set_logd_connected(&self, connected: bool) {
        if let Some(client) = &self.client {
            let _ = client.set_logd_connected(connected);
        }
    }

    /// Query terminal size from consoled (triggers re-detection)
    /// Returns (cols, rows) on success
    pub fn query_size(&mut self) -> Option<(u16, u16)> {
        if let Some(client) = &self.client {
            match client.query_size() {
                Ok((cols, rows)) => {
                    self.cols = cols;
                    self.rows = rows;
                    Some((cols, rows))
                }
                Err(_) => None,
            }
        } else {
            None
        }
    }
}

/// Global console instance
static mut CONSOLE: Console = Console::new();

/// Initialize and connect to console
pub fn init() -> bool {
    unsafe { (*core::ptr::addr_of_mut!(CONSOLE)).connect() }
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

/// Set input state (convenience function)
pub fn set_input_state(state: InputState) {
    console().set_input_state(state);
}
