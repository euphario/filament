//! Console Daemon (consoled)
//!
//! Simple console multiplexer - forwards UART input to shell, shell output to UART.
//! Uses the unified 5-syscall interface (open, read, write, map, close).
//!
//! ## Design
//!
//! Minimal design - no split screen, just a pipe between UART and shell.
//! Shell connects to "console:" port, then bidirectional forwarding begins.
//!
//! Uses Mux for event-driven waiting - properly sleeps until events arrive.

#![no_std]
#![no_main]

use userlib::syscall::{self, LogLevel, Handle, ObjectType};
use userlib::ipc::{Port, Channel, Mux, MuxFilter, ObjHandle};
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
    /// Shell connected, forwarding I/O
    Connected,
}

pub struct Consoled {
    state: ConsoleState,
    port: Option<Port>,
    shell: Option<Channel>,
    mux: Option<Mux>,
    stdin_handle: ObjHandle,
    cols: u16,
    rows: u16,
}

impl Consoled {
    pub const fn new() -> Self {
        Self {
            state: ConsoleState::WaitingForShell,
            port: None,
            shell: None,
            mux: None,
            stdin_handle: Handle::INVALID,
            cols: 80,
            rows: 24,
        }
    }

    pub fn init(&mut self) -> SysResult<()> {
        // Probe terminal size once at startup
        if let Some((cols, rows)) = ansi::query_screen_size() {
            clog!("screen {}x{}", cols, rows);
            self.cols = cols;
            self.rows = rows;
        } else {
            clog!("screen default 80x24");
        }

        // Open stdin handle for use with Mux
        self.stdin_handle = syscall::open(ObjectType::Stdin, &[])?;
        clog!("stdin handle={}", self.stdin_handle.0);

        // Drain any stale input that accumulated before we were ready
        let mut buf = [0u8; 64];
        let mut drained = 0usize;
        loop {
            match syscall::read(self.stdin_handle, &mut buf) {
                Ok(n) if n > 0 => drained += n,
                _ => break,
            }
        }
        if drained > 0 {
            clog!("drained {} stale bytes", drained);
        }

        // Create event multiplexer
        let mux = Mux::new()?;
        clog!("mux handle={}", mux.handle().0);

        // Add stdin to mux (readable)
        mux.add(self.stdin_handle, MuxFilter::Readable)?;

        // Register console port
        let port = Port::register(b"console:")?;
        clog!("port handle={}", port.handle().0);

        // Add port to mux (readable = accept pending)
        mux.add(port.handle(), MuxFilter::Readable)?;

        self.port = Some(port);
        self.mux = Some(mux);

        Ok(())
    }

    pub fn run(&mut self) -> ! {
        clog!("running");

        // Log handles for debugging
        clog!("stdin={} port={}", self.stdin_handle.0,
              self.port.as_ref().map(|p| p.handle().0).unwrap_or(0));

        loop {
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

            // Always poll ALL sources after any event to avoid missing data.
            // mux.wait() returns one event, but multiple handles may be ready.
            // Process stdin (UART input -> shell)
            self.handle_stdin();

            // Process shell output (shell -> UART)
            if self.shell.is_some() {
                self.handle_shell();
            }
        }
    }

    fn handle_stdin(&mut self) {
        // Read from UART (via stdin handle)
        let mut buf = [0u8; 64];
        match syscall::read(self.stdin_handle, &mut buf) {
            Ok(n) if n > 0 => {
                // Forward to shell if connected
                if let Some(shell) = &self.shell {
                    if shell.send(&buf[..n]).is_err() {
                        cerror!("shell send failed, disconnecting");
                        self.disconnect_shell();
                    }
                }
            }
            Ok(_) => {} // No data
            Err(SysError::WouldBlock) => {} // Expected
            Err(_) => {} // Ignore other errors
        }
    }

    fn handle_shell(&mut self) {
        // Check if shell is still connected
        let shell_handle = match &self.shell {
            Some(s) => s.handle(),
            None => return,
        };

        // Drain all available data from shell (multiple messages may be queued)
        let mut buf = [0u8; 256];
        loop {
            let result = syscall::read(shell_handle, &mut buf);

            match result {
                Ok(n) if n > 0 => {
                    // Check for GETSIZE query
                    if n >= 7 && &buf[..7] == b"GETSIZE" {
                        // Respond with SIZE message
                        let mut msg = [0u8; 32];
                        let len = format_size_msg(&mut msg, self.cols, self.rows);
                        if let Some(shell) = &self.shell {
                            let _ = shell.send(&msg[..len]);
                        }
                    } else {
                        // Forward to UART
                        let _ = syscall::write(Handle::STDOUT, &buf[..n]);
                    }
                }
                Ok(_) | Err(SysError::WouldBlock) => break, // No more data
                Err(SysError::ConnectionReset) | Err(SysError::BadFd) => {
                    clog!("shell disconnected");
                    self.disconnect_shell();
                    break;
                }
                Err(_) => break, // Other errors
            }
        }
    }

    fn handle_port(&mut self) {
        if self.shell.is_some() {
            return; // Already have a shell
        }

        if let Some(port) = &self.port {
            match port.accept() {
                Ok(channel) => {
                    clog!("shell connected");

                    // Drain any stale UART input before shell starts
                    let mut buf = [0u8; 64];
                    loop {
                        match syscall::read(self.stdin_handle, &mut buf) {
                            Ok(n) if n > 0 => {}
                            _ => break,
                        }
                    }

                    // Add shell channel to mux
                    if let Some(mux) = &self.mux {
                        let _ = mux.add(channel.handle(), MuxFilter::Readable);
                    }

                    self.shell = Some(channel);
                    self.state = ConsoleState::Connected;
                    // Shell will send GETSIZE to request terminal size
                }
                Err(SysError::WouldBlock) => {} // No pending connections
                Err(e) => {
                    cerror!("accept error: {:?}", e);
                }
            }
        }
    }

    fn disconnect_shell(&mut self) {
        if let Some(shell) = self.shell.take() {
            // Remove shell from mux
            if let Some(mux) = &self.mux {
                let _ = mux.remove(shell.handle());
            }
            // shell is dropped here, closing the handle
        }
        self.state = ConsoleState::WaitingForShell;
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
    cerror!("BOOT v2.0-unified");

    let consoled = unsafe { &mut CONSOLED };

    if let Err(e) = consoled.init() {
        cerror!("init failed: {:?}", e);
        syscall::exit(1);
    }

    cerror!("init complete");
    consoled.run()
}
