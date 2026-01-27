//! Console Daemon (consoled)
//!
//! High-performance console multiplexer using ring buffer IPC.
//! Forwards UART input to shell, shell output to UART.
//!
//! ## Design
//!
//! Uses ConsoleRing for lock-free bidirectional I/O with shell:
//! - TX ring: shell output -> consoled -> UART (large buffer for burst writes)
//! - RX ring: UART input -> consoled -> shell (small buffer)
//!
//! Shell connects to "console:" port, receives shmem_id, maps the ring.

#![no_std]
#![no_main]

use userlib::syscall::{self, LogLevel, Handle, ObjectType};
use userlib::ipc::{Port, Channel, Mux, MuxFilter, ObjHandle};
use userlib::console_ring::ConsoleRing;
use userlib::error::{SysError, SysResult};

// =============================================================================
// Logging
// =============================================================================

macro_rules! clog {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let mut buf = [0u8; 128];
        let mut pos = 0;
        for &b in b"[consoled] " { if pos < buf.len() { buf[pos] = b; pos += 1; } }
        struct W<'a> { b: &'a mut [u8], p: &'a mut usize }
        impl core::fmt::Write for W<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                for &b in s.as_bytes() {
                    if *self.p < self.b.len() { self.b[*self.p] = b; *self.p += 1; }
                }
                Ok(())
            }
        }
        let _ = write!(W { b: &mut buf, p: &mut pos }, $($arg)*);
        syscall::klog(LogLevel::Info, &buf[..pos]);
    }};
}

macro_rules! cerror {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let mut buf = [0u8; 128];
        let mut pos = 0;
        for &b in b"[consoled] " { if pos < buf.len() { buf[pos] = b; pos += 1; } }
        struct W<'a> { b: &'a mut [u8], p: &'a mut usize }
        impl core::fmt::Write for W<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                for &b in s.as_bytes() {
                    if *self.p < self.b.len() { self.b[*self.p] = b; *self.p += 1; }
                }
                Ok(())
            }
        }
        let _ = write!(W { b: &mut buf, p: &mut pos }, $($arg)*);
        syscall::klog(LogLevel::Error, &buf[..pos]);
    }};
}

// =============================================================================
// ANSI helpers
// =============================================================================

mod ansi {
    use userlib::syscall::{self, Handle};

    /// Query terminal size using cursor position report (CPR)
    /// Returns (cols, rows) or None if detection fails
    pub fn query_screen_size() -> Option<(u16, u16)> {
        // Save cursor, move to bottom-right corner, query position
        write(b"\x1b[s\x1b[9999;9999H\x1b[6n");

        // Read response: ESC [ rows ; cols R
        let mut buf = [0u8; 32];
        let mut len = 0;

        for _ in 0..100 {
            let mut byte = [0u8; 1];
            let n = syscall::read(Handle::STDIN, &mut byte).unwrap_or(0);
            if n > 0 {
                if len < buf.len() {
                    buf[len] = byte[0];
                    len += 1;
                }
                if byte[0] == b'R' {
                    break;
                }
            } else {
                for _ in 0..10000 {
                    core::hint::spin_loop();
                }
            }
        }

        // Restore cursor position
        write(b"\x1b[u");

        // Parse response: ESC [ rows ; cols R
        if len < 6 {
            return None;
        }

        let mut start = 0;
        while start + 2 < len && !(buf[start] == 0x1b && buf[start + 1] == b'[') {
            start += 1;
        }

        if start + 2 >= len {
            return None;
        }

        let mut i = start + 2;
        let mut rows: u16 = 0;
        while i < len && buf[i] >= b'0' && buf[i] <= b'9' {
            rows = rows.saturating_mul(10).saturating_add((buf[i] - b'0') as u16);
            i += 1;
        }

        if i >= len || buf[i] != b';' {
            return None;
        }
        i += 1;

        let mut cols: u16 = 0;
        while i < len && buf[i] >= b'0' && buf[i] <= b'9' {
            cols = cols.saturating_mul(10).saturating_add((buf[i] - b'0') as u16);
            i += 1;
        }

        if i >= len || buf[i] != b'R' {
            return None;
        }

        if rows >= 10 && rows <= 500 && cols >= 40 && cols <= 500 {
            Some((cols, rows))
        } else {
            None
        }
    }

    fn write(data: &[u8]) {
        let _ = syscall::write(Handle::STDOUT, data);
    }
}

// =============================================================================
// Console State
// =============================================================================

/// Console state
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConsoleState {
    /// Waiting for shell to connect
    WaitingForShell,
    /// Shell connected, forwarding I/O via ring
    Connected,
}

pub struct Consoled {
    state: ConsoleState,
    port: Option<Port>,
    /// Ring buffer for shell I/O (replaces Channel)
    shell_ring: Option<ConsoleRing>,
    /// Channel used during handshake (to send shmem_id)
    handshake_channel: Option<Channel>,
    mux: Option<Mux>,
    stdin_handle: ObjHandle,
    shmem_handle: ObjHandle,
    cols: u16,
    rows: u16,
    /// Channel to devd (announces we're ready)
    devd_channel: Option<Channel>,
    /// Split screen mode enabled
    split_enabled: bool,
    /// Number of lines for log region (top)
    log_lines: u16,
    /// Current cursor row in output region
    output_row: u16,
}

impl Consoled {
    pub const fn new() -> Self {
        Self {
            state: ConsoleState::WaitingForShell,
            port: None,
            shell_ring: None,
            handshake_channel: None,
            mux: None,
            stdin_handle: Handle::INVALID,
            shmem_handle: Handle::INVALID,
            cols: 80,
            rows: 24,
            devd_channel: None,
            split_enabled: false,
            log_lines: 5,
            output_row: 6,  // First row after log region
        }
    }

    pub fn init(&mut self) -> SysResult<()> {
        // Skip terminal size detection at boot - causes blank output due to ANSI
        // escape sequences interleaving with kernel logs. Shell can query via
        // GETSIZE if needed. Defaults: 80x24

        // Open stdin handle for use with Mux
        self.stdin_handle = syscall::open(ObjectType::Stdin, &[])?;

        // Drain any stale input that accumulated before we were ready
        let mut buf = [0u8; 64];
        loop {
            match syscall::read(self.stdin_handle, &mut buf) {
                Ok(n) if n > 0 => {}
                _ => break,
            }
        }

        // Create event multiplexer
        let mux = Mux::new()?;

        // Add stdin to mux (readable)
        mux.add(self.stdin_handle, MuxFilter::Readable)?;

        // Register console port (limit: 1 shell connection)
        let port = Port::with_limit(b"console:", 1)?;

        // Add port to mux (readable = accept pending)
        mux.add(port.handle(), MuxFilter::Readable)?;

        self.port = Some(port);
        self.mux = Some(mux);

        // Announce ready to devd
        if let Ok(devd_channel) = Channel::connect(b"devd:") {
            // Keep the channel open - devd uses it to know we're alive
            self.devd_channel = Some(devd_channel);
        }

        Ok(())
    }

    // =========================================================================
    // Split Screen Management
    // =========================================================================

    /// Enable split screen mode
    ///
    /// Layout:
    /// - Lines 1 to log_lines: Log region (fixed, doesn't scroll)
    /// - Line log_lines+1: Top separator (dashes)
    /// - Lines log_lines+2 to rows-2: Output region (scrolls)
    /// - Line rows-1: Bottom separator (dashes)
    /// - Line rows: Prompt line (in scroll region for readline compat)
    fn enable_split(&mut self) {
        if self.split_enabled {
            return;
        }
        self.split_enabled = true;

        let mut buf = [0u8; 700];
        let mut pos = 0;

        // Clear entire screen
        buf[pos..pos+4].copy_from_slice(b"\x1b[2J");
        pos += 4;

        // Draw TOP separator at log_lines+1
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += Self::write_u16_to_buf(&mut buf[pos..], self.log_lines + 1);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        let line_chars = (self.cols as usize).min(200);
        for _ in 0..line_chars {
            buf[pos] = b'-';
            pos += 1;
        }

        // Draw BOTTOM separator at rows-1
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += Self::write_u16_to_buf(&mut buf[pos..], self.rows - 1);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        for _ in 0..line_chars {
            buf[pos] = b'-';
            pos += 1;
        }

        // Set scroll region: output area + prompt line
        // From log_lines+2 to rows (includes prompt for readline)
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += Self::write_u16_to_buf(&mut buf[pos..], self.log_lines + 2);
        buf[pos] = b';';
        pos += 1;
        pos += Self::write_u16_to_buf(&mut buf[pos..], self.rows);
        buf[pos] = b'r';
        pos += 1;

        // Move cursor to prompt line (bottom)
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += Self::write_u16_to_buf(&mut buf[pos..], self.rows);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        self.write_uart(&buf[..pos]);
    }

    /// Write u16 to buffer, return bytes written
    fn write_u16_to_buf(buf: &mut [u8], n: u16) -> usize {
        if n == 0 {
            buf[0] = b'0';
            return 1;
        }
        let mut tmp = [0u8; 5];
        let mut val = n;
        let mut len = 0;
        while val > 0 {
            tmp[len] = b'0' + (val % 10) as u8;
            val /= 10;
            len += 1;
        }
        for i in 0..len {
            buf[i] = tmp[len - 1 - i];
        }
        len
    }

    /// Disable split screen mode
    fn disable_split(&mut self) {
        if !self.split_enabled {
            return;
        }
        self.split_enabled = false;

        // Reset scroll region to full screen, clear, home cursor
        self.write_uart(b"\x1b[r\x1b[2J\x1b[H");
    }

    /// Write to log region (top, scrolls independently)
    fn write_log(&mut self, data: &[u8]) {
        if !self.split_enabled {
            return;
        }

        // Batch all output into single buffer to avoid storm detection
        let mut buf = [0u8; 600];
        let mut pos = 0;

        // Save cursor: ESC[s
        buf[pos..pos+3].copy_from_slice(b"\x1b[s");
        pos += 3;

        // Set scroll region to log area only: ESC[1;<log_lines>r
        buf[pos..pos+4].copy_from_slice(b"\x1b[1;");
        pos += 4;
        pos += Self::write_u16_to_buf(&mut buf[pos..], self.log_lines);
        buf[pos] = b'r';
        pos += 1;

        // Move to last line of log region: ESC[<log_lines>;1H
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += Self::write_u16_to_buf(&mut buf[pos..], self.log_lines);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        // Copy log data (limited by buffer)
        let data_len = data.len().min(500);
        buf[pos..pos+data_len].copy_from_slice(&data[..data_len]);
        pos += data_len;

        // Reset scroll region to full screen: ESC[r
        buf[pos..pos+3].copy_from_slice(b"\x1b[r");
        pos += 3;

        // Restore cursor: ESC[u
        buf[pos..pos+3].copy_from_slice(b"\x1b[u");
        pos += 3;

        self.write_uart(&buf[..pos]);
    }

    /// Write to shell output region
    ///
    /// In split mode, the scroll region is set so normal writes go to the
    /// shell area and scroll naturally within that region.
    fn write_output(&mut self, data: &[u8]) {
        // Just pass through - scroll region handles positioning
        self.write_uart(data);
    }

    /// Move cursor to prompt line (bottom)
    fn goto_prompt(&mut self) {
        if !self.split_enabled {
            return;
        }
        self.write_uart(b"\x1b[");
        self.write_u16(self.rows);
        self.write_uart(b";1H\x1b[K");  // Go to last row, clear line
    }

    /// Write a u16 as decimal to UART
    fn write_u16(&self, n: u16) {
        let mut buf = [0u8; 5];
        let len = format_u16(&mut buf, n);
        self.write_uart(&buf[..len]);
    }

    /// Write directly to UART
    fn write_uart(&self, data: &[u8]) {
        let _ = syscall::write(Handle::STDOUT, data);
    }

    pub fn run(&mut self) -> ! {
        loop {
            // If connected via ring, use polling loop
            if self.shell_ring.is_some() {
                self.run_ring_loop();
            } else {
                // Not connected - wait for connection on port
                let mux = self.mux.as_ref().expect("consoled: mux not initialized");
                let wait_result = mux.wait();

                // Wait for event (kernel blocks internally)
                let event = match wait_result {
                    Ok(e) => e,
                    Err(e) => {
                        cerror!("mux wait error: {:?}", e);
                        continue;
                    }
                };

                // Check for new connection on port
                if let Some(port) = &self.port {
                    if event.handle == port.handle() {
                        self.handle_port();
                    }
                }

                // Drain stdin to prevent spinning when not connected
                // (stdin data has nowhere to go without a shell)
                if event.handle == self.stdin_handle {
                    let mut buf = [0u8; 64];
                    while let Ok(n) = syscall::read(self.stdin_handle, &mut buf) {
                        if n == 0 { break; }
                    }
                }
            }
        }
    }

    /// Main loop when shell is connected via ring
    ///
    /// Strategy:
    /// - Wait on BOTH stdin (UART input) AND shmem (shell TX notifications)
    /// - This ensures we wake immediately when user types, not just when shell outputs
    fn run_ring_loop(&mut self) {
        let mut tx_buf = [0u8; 512];  // Buffer for reading from TX ring
        let mut rx_buf = [0u8; 64];   // Buffer for reading from UART

        // Create a new mux for ring mode that includes both stdin and shmem
        let ring_mux = match Mux::new() {
            Ok(m) => m,
            Err(e) => {
                cerror!("failed to create ring mux: {:?}", e);
                return;
            }
        };

        // Add stdin to mux (wake when UART has input)
        if let Err(e) = ring_mux.add(self.stdin_handle, MuxFilter::Readable) {
            cerror!("failed to add stdin to ring mux: {:?}", e);
            return;
        }

        // Add shmem handle to mux (wake when shell notifies)
        if let Some(ring) = &self.shell_ring {
            if let Err(e) = ring_mux.add(ring.handle(), MuxFilter::Readable) {
                cerror!("failed to add shmem to ring mux: {:?}", e);
                return;
            }
        }

        // Add handshake channel to mux (wake when shell disconnects)
        let channel_handle = if let Some(ch) = &self.handshake_channel {
            let h = ch.handle();
            // Use Readable filter - closed channels become readable with error
            if let Err(e) = ring_mux.add(h, MuxFilter::Readable) {
                cerror!("failed to add channel to ring mux: {:?}", e);
                return;
            }
            Some(h)
        } else {
            None
        };

        loop {
            // Check if still connected
            if self.shell_ring.is_none() {
                return;
            }

            // Process TX ring (shell output -> UART)
            // Read data first, then process (to avoid borrow conflicts)
            let n = {
                let ring = self.shell_ring.as_ref().unwrap();
                let tx_avail = ring.tx_available();
                if tx_avail == 0 {
                    0
                } else {
                    let to_read = tx_avail.min(tx_buf.len());
                    ring.tx_read(&mut tx_buf[..to_read])
                }
            };

            if n > 0 {
                // Find command start (skip leading whitespace/newlines which may be echo)
                let mut cmd_start = 0;
                while cmd_start < n && (tx_buf[cmd_start] == b'\n' || tx_buf[cmd_start] == b'\r') {
                    // Output the newline first (it's echo from shell)
                    if self.split_enabled {
                        self.write_output(&tx_buf[cmd_start..cmd_start+1]);
                    } else {
                        self.write_uart(&tx_buf[cmd_start..cmd_start+1]);
                    }
                    cmd_start += 1;
                }

                // Process the remaining data
                let cmd_buf = &tx_buf[cmd_start..n];
                let cmd_len = n - cmd_start;

                // Process the data we read (using cmd_buf which has leading newlines stripped)
                if cmd_len >= 5 && &cmd_buf[..5] == b"SPLIT" {
                    // Parse optional line count: "SPLIT 5\n" or just "SPLIT\n"
                    if cmd_len > 6 && cmd_buf[5] == b' ' {
                        let mut lines: u16 = 0;
                        let mut i = 6;
                        while i < cmd_len && cmd_buf[i] >= b'0' && cmd_buf[i] <= b'9' {
                            lines = lines.saturating_mul(10).saturating_add((cmd_buf[i] - b'0') as u16);
                            i += 1;
                        }
                        if lines >= 1 && lines <= 20 {
                            self.log_lines = lines;
                        }
                    }
                    self.enable_split();
                    // Send OK response
                    if let Some(ring) = &self.shell_ring {
                        ring.rx_write(b"OK\n");
                        ring.notify();
                    }
                } else if cmd_len >= 7 && &cmd_buf[..7] == b"NOSPLIT" {
                    self.disable_split();
                    if let Some(ring) = &self.shell_ring {
                        ring.rx_write(b"OK\n");
                        ring.notify();
                    }
                } else if cmd_len >= 7 && &cmd_buf[..7] == b"GETSIZE" {
                    // Temporarily disable scroll region for size detection
                    if self.split_enabled {
                        self.write_uart(b"\x1b[r");  // Reset scroll region
                    }

                    match ansi::query_screen_size() {
                        Some((cols, rows)) => {
                            // Log detected size
                            self.write_uart(b"\r\n[consoled] detected size: ");
                            let mut nbuf = [0u8; 8];
                            let nlen = Self::write_u16_to_buf(&mut nbuf, cols);
                            self.write_uart(&nbuf[..nlen]);
                            self.write_uart(b"x");
                            let nlen = Self::write_u16_to_buf(&mut nbuf, rows);
                            self.write_uart(&nbuf[..nlen]);
                            self.write_uart(b"\r\n");

                            self.cols = cols;
                            self.rows = rows;
                        }
                        None => {
                            self.write_uart(b"\r\n[consoled] size detection failed, using cached\r\n");
                        }
                    }

                    // Re-enable scroll region if split mode
                    if self.split_enabled {
                        // Re-setup the split screen with new size
                        self.split_enabled = false;
                        self.enable_split();
                    }

                    let mut msg = [0u8; 32];
                    let len = format_size_msg(&mut msg, self.cols, self.rows);
                    if let Some(ring) = &self.shell_ring {
                        ring.rx_write(&msg[..len]);
                        ring.notify();
                    }
                } else if cmd_len > 0 {
                    // Regular output (not a command)
                    if self.split_enabled {
                        self.write_output(cmd_buf);
                    } else {
                        self.write_uart(cmd_buf);
                    }
                }
                continue; // Check for more data
            }

            // Check stdin for user input (UART -> RX ring)
            match syscall::read(self.stdin_handle, &mut rx_buf) {
                Ok(n) if n > 0 => {
                    if let Some(ring) = &self.shell_ring {
                        let written = ring.rx_write(&rx_buf[..n]);
                        if written > 0 {
                            ring.notify();
                        }
                    }
                    continue;
                }
                _ => {}
            }

            // Wait for EITHER stdin OR shmem OR channel to become readable
            // This ensures we wake immediately when user types or shell disconnects
            match ring_mux.wait() {
                Ok(event) => {
                    // Check if this is the handshake channel signaling closure
                    if let Some(ch_handle) = channel_handle {
                        if event.handle == ch_handle {
                            // Channel fired - try to recv to check if it's closed
                            if let Some(ch) = &mut self.handshake_channel {
                                let mut probe = [0u8; 1];
                                match ch.recv(&mut probe) {
                                    Err(SysError::PeerClosed) | Err(SysError::ConnectionReset) => {
                                        clog!("shell disconnected (channel closed)");
                                        self.disconnect_shell();
                                        return;
                                    }
                                    _ => {} // Got data or would block - shell still alive
                                }
                            }
                        }
                    }
                }
                Err(_) => {
                    syscall::sleep_us(50_000); // 50ms backoff if wait fails
                }
            }
        }
    }

    fn handle_port(&mut self) {
        if self.shell_ring.is_some() {
            return; // Already have a shell
        }

        if let Some(port) = &mut self.port {
            match port.accept_with_pid() {
                Ok((channel, client_pid)) => {
                    // Create ring for shell communication
                    let ring = match ConsoleRing::create() {
                        Some(r) => r,
                        None => {
                            cerror!("failed to create ring");
                            return;
                        }
                    };

                    // Allow shell to map the ring using its actual PID
                    ring.allow(client_pid);

                    // Send shmem_id to shell via channel
                    let shmem_id = ring.shmem_id();
                    let mut msg = [0u8; 16];
                    msg[..4].copy_from_slice(b"RING");
                    msg[4..8].copy_from_slice(&shmem_id.to_le_bytes());
                    // Also send terminal size
                    msg[8..10].copy_from_slice(&self.cols.to_le_bytes());
                    msg[10..12].copy_from_slice(&self.rows.to_le_bytes());

                    if channel.send(&msg[..12]).is_err() {
                        cerror!("failed to send ring info");
                        return;
                    }

                    clog!("shell pid={} connected", client_pid);

                    // Drain any stale UART input before shell starts
                    let mut buf = [0u8; 64];
                    loop {
                        match syscall::read(self.stdin_handle, &mut buf) {
                            Ok(n) if n > 0 => {}
                            _ => break,
                        }
                    }

                    // Store ring and handshake channel
                    self.shell_ring = Some(ring);
                    self.handshake_channel = Some(channel);
                    self.state = ConsoleState::Connected;
                }
                Err(SysError::WouldBlock) => {} // No pending connections
                Err(e) => {
                    cerror!("accept error: {:?}", e);
                }
            }
        }
    }

    fn disconnect_shell(&mut self) {
        self.shell_ring = None;
        self.handshake_channel = None;

        // Notify port that connection is closed (allows new connections)
        if let Some(port) = &mut self.port {
            port.connection_closed();
        }
        self.state = ConsoleState::WaitingForShell;
        clog!("shell disconnected");
    }
}

/// Format size message: "SIZE cols rows\n"
fn format_size_msg(buf: &mut [u8], cols: u16, rows: u16) -> usize {
    let mut i = 0;

    // "SIZE "
    for &b in b"SIZE " {
        if i < buf.len() { buf[i] = b; i += 1; }
    }

    // cols
    i += format_u16(&mut buf[i..], cols);

    // " "
    if i < buf.len() { buf[i] = b' '; i += 1; }

    // rows
    i += format_u16(&mut buf[i..], rows);

    // "\n"
    if i < buf.len() { buf[i] = b'\n'; i += 1; }

    i
}

fn format_u16(buf: &mut [u8], n: u16) -> usize {
    if n == 0 {
        if !buf.is_empty() { buf[0] = b'0'; }
        return 1;
    }
    let mut tmp = [0u8; 5];
    let mut val = n;
    let mut len = 0;
    while val > 0 {
        tmp[len] = b'0' + (val % 10) as u8;
        val /= 10;
        len += 1;
    }
    for i in 0..len {
        if i < buf.len() {
            buf[i] = tmp[len - 1 - i];
        }
    }
    len
}

// =============================================================================
// Main
// =============================================================================

static mut CONSOLED: Consoled = Consoled::new();

#[unsafe(no_mangle)]
#[allow(static_mut_refs)]
fn main() -> ! {
    let consoled = unsafe { &mut CONSOLED };

    if let Err(e) = consoled.init() {
        cerror!("init failed: {:?}", e);
        syscall::exit(1);
    }

    consoled.run()
}
