//! Console Daemon (consoled)
//!
//! Bus-framework driver that manages the serial console.
//! Connects to kernel UART bus, exposes `console:` port for shell.
//!
//! ## Architecture
//!
//! ```text
//! ┌────────────┐     ┌──────────┐     ┌────────────┐
//! │   Shell    │ ←── │ consoled │ ←── │ Kernel     │
//! │ (connects  │ ring│ (driver) │uart │ UART bus   │
//! │  console:) │     │          │     │ /kernel/   │
//! └────────────┘     └──────────┘     │ bus/uart0  │
//!                                     └────────────┘
//! ```
//!
//! Uses ConsoleRing for lock-free bidirectional I/O with shell:
//! - TX ring: shell output -> consoled -> UART
//! - RX ring: UART input -> consoled -> shell

#![no_std]
#![no_main]

use userlib::syscall::{self, Handle, ObjectType};
use userlib::ipc::{Port, Channel, ObjHandle};
use userlib::console_ring::ConsoleRing;
use userlib::error::SysError;
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition,
    PortInfo, PortClass, PortState, port_subclass,
};
use userlib::bus_runtime::driver_main;
use userlib::{uinfo, uerror};

// =============================================================================
// Handle Tags
// =============================================================================

const TAG_STDIN: u32 = 1;
const TAG_PORT: u32 = 2;
const TAG_SHMEM: u32 = 3;
const TAG_HANDSHAKE: u32 = 4;

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
// Console Driver State
// =============================================================================

/// Console state
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConsoleState {
    /// Waiting for shell to connect
    WaitingForShell,
    /// Shell connected, forwarding I/O via ring
    Connected,
}

pub struct ConsoledDriver {
    state: ConsoleState,
    port: Option<Port>,
    /// Ring buffer for shell I/O
    shell_ring: Option<ConsoleRing>,
    /// Channel used during handshake (to send shmem_id)
    handshake_channel: Option<Channel>,
    stdin_handle: ObjHandle,
    cols: u16,
    rows: u16,
    /// Split screen mode enabled
    split_enabled: bool,
    /// Number of lines for log region (top)
    log_lines: u16,
}

impl ConsoledDriver {
    pub const fn new() -> Self {
        Self {
            state: ConsoleState::WaitingForShell,
            port: None,
            shell_ring: None,
            handshake_channel: None,
            stdin_handle: Handle::INVALID,
            cols: 80,
            rows: 24,
            split_enabled: false,
            log_lines: 5,
        }
    }

    // =========================================================================
    // Split Screen Management
    // =========================================================================

    /// Enable split screen mode
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
        pos += write_u16_to_buf(&mut buf[pos..], self.log_lines + 1);
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
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 1);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        for _ in 0..line_chars {
            buf[pos] = b'-';
            pos += 1;
        }

        // Set scroll region: output area + prompt line
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.log_lines + 2);
        buf[pos] = b';';
        pos += 1;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows);
        buf[pos] = b'r';
        pos += 1;

        // Move cursor to prompt line (bottom)
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        self.write_uart(&buf[..pos]);
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

    /// Write to shell output region
    fn write_output(&mut self, data: &[u8]) {
        // Just pass through - scroll region handles positioning
        self.write_uart(data);
    }

    /// Write directly to UART
    fn write_uart(&self, data: &[u8]) {
        let _ = syscall::write(Handle::STDOUT, data);
    }

    // =========================================================================
    // Port Connection Handling
    // =========================================================================

    fn handle_port_readable(&mut self, ctx: &mut dyn BusCtx) {
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
                            uerror!("consoled", "ring_create_failed";);
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
                    msg[8..10].copy_from_slice(&self.cols.to_le_bytes());
                    msg[10..12].copy_from_slice(&self.rows.to_le_bytes());

                    if channel.send(&msg[..12]).is_err() {
                        uerror!("consoled", "send_ring_info_failed";);
                        return;
                    }

                    uinfo!("consoled", "shell_connected"; pid = client_pid);

                    // Drain any stale UART input before shell starts
                    let mut buf = [0u8; 64];
                    loop {
                        match syscall::read(self.stdin_handle, &mut buf) {
                            Ok(n) if n > 0 => {}
                            _ => break,
                        }
                    }

                    // Watch shmem and handshake channel for events
                    let _ = ctx.watch_handle(ring.handle(), TAG_SHMEM);
                    let _ = ctx.watch_handle(channel.handle(), TAG_HANDSHAKE);

                    // Store ring and handshake channel
                    self.shell_ring = Some(ring);
                    self.handshake_channel = Some(channel);
                    self.state = ConsoleState::Connected;
                }
                Err(SysError::WouldBlock) => {} // No pending connections
                Err(_e) => {
                    uerror!("consoled", "accept_error";);
                }
            }
        }
    }

    fn handle_stdin_readable(&mut self, ctx: &mut dyn BusCtx) {
        let mut rx_buf = [0u8; 64];

        match syscall::read(self.stdin_handle, &mut rx_buf) {
            Ok(n) if n > 0 => {
                // Check if shell is still connected by probing the handshake channel
                if self.shell_ring.is_some() {
                    if let Some(ch) = &mut self.handshake_channel {
                        // Non-blocking probe: try to receive
                        let mut probe = [0u8; 1];
                        match ch.try_recv(&mut probe) {
                            Err(SysError::ConnectionReset) => {
                                uinfo!("consoled", "shell_dead_on_stdin";);
                                self.disconnect_shell(ctx);
                                return;
                            }
                            _ => {} // Ok(None)=WouldBlock, Ok(Some)=got data - shell alive
                        }
                    }
                }

                if let Some(ring) = &self.shell_ring {
                    let written = ring.rx_write(&rx_buf[..n]);
                    if written > 0 {
                        ring.notify();
                    }
                }
                // If no shell connected, discard input
            }
            _ => {}
        }
    }

    fn handle_shmem_readable(&mut self, ctx: &mut dyn BusCtx) {
        let mut tx_buf = [0u8; 512];

        // Read data from TX ring
        let n = {
            let ring = match self.shell_ring.as_ref() {
                Some(r) => r,
                None => return,
            };
            let tx_avail = ring.tx_available();
            if tx_avail == 0 {
                return;
            }
            let to_read = tx_avail.min(tx_buf.len());
            ring.tx_read(&mut tx_buf[..to_read])
        };

        if n == 0 {
            return;
        }

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
                self.write_uart(b"\x1b[r");
            }

            match ansi::query_screen_size() {
                Some((cols, rows)) => {
                    self.cols = cols;
                    self.rows = rows;
                }
                None => {}
            }

            // Re-enable scroll region if split mode
            if self.split_enabled {
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

        // Check for more data (recursively, but bounded by buffer)
        self.handle_shmem_readable(ctx);
    }

    fn handle_channel_readable(&mut self, ctx: &mut dyn BusCtx) {
        // Channel fired - try to recv to check if it's closed
        if let Some(ch) = &mut self.handshake_channel {
            let mut probe = [0u8; 1];
            match ch.recv(&mut probe) {
                Err(SysError::PeerClosed) | Err(SysError::ConnectionReset) => {
                    uinfo!("consoled", "shell_disconnected";);
                    self.disconnect_shell(ctx);
                }
                _ => {} // Got data or would block - shell still alive
            }
        }
    }

    fn disconnect_shell(&mut self, ctx: &mut dyn BusCtx) {
        // Unwatch handles before dropping
        if let Some(ring) = &self.shell_ring {
            let _ = ctx.unwatch_handle(ring.handle());
        }
        if let Some(ch) = &self.handshake_channel {
            let _ = ctx.unwatch_handle(ch.handle());
        }

        self.shell_ring = None;
        self.handshake_channel = None;

        // Notify port that connection is closed (allows new connections)
        if let Some(port) = &mut self.port {
            port.connection_closed();
        }
        self.state = ConsoleState::WaitingForShell;

        // Notify devd: port briefly Registered then Ready to trigger spawn rule
        let _ = ctx.set_port_state(b"console:", PortState::Registered);
        let _ = ctx.set_port_state(b"console:", PortState::Ready);
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for ConsoledDriver {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        uinfo!("consoled", "init";);

        // Claim the kernel UART bus
        let uart_path = b"/kernel/bus/uart0";
        match ctx.claim_kernel_bus(uart_path) {
            Ok((bus_id, info)) => {
                uinfo!("consoled", "uart_claimed"; bus = bus_id.0, state = info.state as u8);
            }
            Err(e) => {
                uerror!("consoled", "uart_claim_failed"; err = e as u8);
                return Err(e);
            }
        }

        // Get stdin handle
        self.stdin_handle = syscall::open(ObjectType::Stdin, &[])
            .map_err(|_| BusError::Internal)?;

        // Drain any stale input
        let mut buf = [0u8; 64];
        loop {
            match syscall::read(self.stdin_handle, &mut buf) {
                Ok(n) if n > 0 => {}
                _ => break,
            }
        }

        // Create console port
        let port = Port::with_limit(b"console:", 1)
            .map_err(|_| BusError::Internal)?;

        // Watch stdin and port for events
        ctx.watch_handle(self.stdin_handle, TAG_STDIN)?;
        ctx.watch_handle(port.handle(), TAG_PORT)?;

        self.port = Some(port);

        // Register console: port with devd
        let mut info = PortInfo::empty();
        info.set_name(b"console:");
        info.port_class = PortClass::Console;
        info.port_subclass = port_subclass::CONSOLE_SERIAL;

        ctx.register_port_with_info(&info, 0)?;
        ctx.set_port_state(b"console:", PortState::Ready)?;

        uinfo!("consoled", "ready";);
        Ok(())
    }

    fn command(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) -> Disposition {
        // consoled doesn't handle any bus commands
        Disposition::Handled
    }

    fn handle_event(&mut self, tag: u32, _handle: Handle, ctx: &mut dyn BusCtx) {
        match tag {
            TAG_STDIN => self.handle_stdin_readable(ctx),
            TAG_PORT => self.handle_port_readable(ctx),
            TAG_SHMEM => self.handle_shmem_readable(ctx),
            TAG_HANDSHAKE => self.handle_channel_readable(ctx),
            _ => {}
        }
    }
}

// =============================================================================
// Helpers
// =============================================================================

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

/// Format size message: "SIZE cols rows\n"
fn format_size_msg(buf: &mut [u8], cols: u16, rows: u16) -> usize {
    let mut i = 0;

    for &b in b"SIZE " {
        if i < buf.len() { buf[i] = b; i += 1; }
    }

    i += format_u16(&mut buf[i..], cols);

    if i < buf.len() { buf[i] = b' '; i += 1; }

    i += format_u16(&mut buf[i..], rows);

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
// Wrapper for driver_main
// =============================================================================

struct ConsoledWrapper(&'static mut ConsoledDriver);

impl Driver for ConsoledWrapper {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.init(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn handle_event(&mut self, tag: u32, handle: Handle, ctx: &mut dyn BusCtx) {
        self.0.handle_event(tag, handle, ctx)
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: ConsoledDriver = ConsoledDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"consoled", ConsoledWrapper(driver));
}
