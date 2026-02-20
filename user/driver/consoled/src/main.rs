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
use userlib::{uinfo, unotice, uerror};

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
                // Short wait for terminal CPR response (~1ms)
                syscall::sleep_us(1000);
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
    /// PID of the connected shell (for signal delivery)
    shell_pid: Option<u32>,
    /// Input rendering mode (consoled owns the input line)
    input_mode: bool,
    /// Prompt for input mode (e.g., "\x1b[1m\x1b[34m/\x1b[32m > \x1b[0m")
    input_prompt: [u8; 48],
    input_prompt_len: u8,
    /// Visible prompt width (excluding ANSI escapes) for cursor positioning
    input_prompt_visible: u8,
    /// Last seen InputState sequence number
    last_input_seq: u32,
    /// Column position in scroll region after last output write (1-based).
    /// Used to resume multi-chunk output at the correct position.
    output_col: u16,
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
            shell_pid: None,
            input_mode: false,
            input_prompt: [0u8; 48],
            input_prompt_len: 0,
            input_prompt_visible: 0,
            last_input_seq: 0,
            output_col: 1,
        }
    }

    // =========================================================================
    // Input Line Rendering (consoled-owned input area)
    // =========================================================================

    /// Check InputState flags and enter/exit input mode accordingly.
    /// Called on every shmem event.
    fn check_input_flags(&mut self) {
        let ring = match self.shell_ring.as_ref() {
            Some(r) => r,
            None => return,
        };

        let input = ring.input_state();
        let flags = input.flags;
        let active = (flags & userlib::console_ring::input_flags::ACTIVE) != 0;

        if active && !self.input_mode {
            // Read prompt from InputState
            let plen = (input.prompt_len as usize).min(48);
            self.input_prompt[..plen].copy_from_slice(&input.prompt[..plen]);
            self.input_prompt_len = plen as u8;
            self.input_prompt_visible = input.prompt_visible_len;
            self.enter_input_mode();
        } else if !active && self.input_mode {
            self.exit_input_mode();
        }
    }

    /// Enter input mode: set up scroll region, render prompt on input row.
    ///
    /// Output position is tracked via `output_col` — no ESC[s/ESC[u needed.
    /// This is resilient to external UART writes (klog) that would corrupt
    /// terminal save/restore state.
    fn enter_input_mode(&mut self) {
        if self.input_mode {
            return;
        }
        self.last_input_seq = 0;
        self.output_col = 1;

        // Disable split if it was on — input mode replaces it
        if self.split_enabled {
            self.split_enabled = false;
        }

        // Scroll screen up to make room for separator + input lines.
        // Without this, short output (e.g., `uptime`) at the bottom of the
        // screen gets overwritten by the separator drawn at rows-1.
        let mut scroll = [0u8; 16];
        let mut spos = 0;
        // Move to bottom of screen
        scroll[spos..spos+2].copy_from_slice(b"\x1b[");
        spos += 2;
        spos += write_u16_to_buf(&mut scroll[spos..], self.rows);
        scroll[spos..spos+3].copy_from_slice(b";1H");
        spos += 3;
        // Print newlines to push content up (2 = separator + input)
        scroll[spos] = b'\n';
        scroll[spos+1] = b'\n';
        spos += 2;
        self.write_uart(&scroll[..spos]);

        let mut buf = [0u8; 64];
        let mut pos = 0;

        // Set scroll region: rows 1..rows-2 for output
        // Row rows-1 = separator line, row rows = input
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        buf[pos..pos+2].copy_from_slice(b"1;");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 2);
        buf[pos] = b'r';
        pos += 1;

        self.write_uart(&buf[..pos]);

        // Draw separator line on row rows-1
        self.draw_separator();

        // Move cursor to input row and draw prompt
        let mut buf2 = [0u8; 32];
        let mut pos2 = 0;
        buf2[pos2..pos2+2].copy_from_slice(b"\x1b[");
        pos2 += 2;
        pos2 += write_u16_to_buf(&mut buf2[pos2..], self.rows);
        buf2[pos2..pos2+3].copy_from_slice(b";1H");
        pos2 += 3;
        buf2[pos2..pos2+3].copy_from_slice(b"\x1b[K");
        pos2 += 3;
        self.write_uart(&buf2[..pos2]);

        // Draw prompt
        self.write_uart(&self.input_prompt[..self.input_prompt_len as usize]);

        self.input_mode = true;
    }

    /// Draw horizontal separator line on row rows-1
    fn draw_separator(&mut self) {
        let mut buf = [0u8; 512];
        let mut pos = 0;

        // Move to separator row
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 1);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        // Dim color for separator
        buf[pos..pos+4].copy_from_slice(b"\x1b[2m");
        pos += 4;

        // Fill with ─ (U+2500 = 0xE2 0x94 0x80), 3 bytes per character
        let max_chars = ((buf.len() - pos - 8) / 3).min(self.cols as usize);
        for _ in 0..max_chars {
            buf[pos] = 0xe2;
            buf[pos+1] = 0x94;
            buf[pos+2] = 0x80;
            pos += 3;
        }

        // Reset color
        buf[pos..pos+4].copy_from_slice(b"\x1b[0m");
        pos += 4;

        self.write_uart(&buf[..pos]);
    }

    /// Exit input mode: reset scroll region, position cursor after output.
    fn exit_input_mode(&mut self) {
        if !self.input_mode {
            return;
        }
        self.input_mode = false;

        let mut buf = [0u8; 64];
        let mut pos = 0;

        // Clear separator line (row=rows-1)
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 1);
        buf[pos..pos+6].copy_from_slice(b";1H\x1b[K");
        pos += 6;

        // Clear the input line (row=rows)
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows);
        buf[pos..pos+6].copy_from_slice(b";1H\x1b[K");
        pos += 6;

        // Reset scroll region (ESC[r moves cursor to 1,1 — VT100 spec)
        buf[pos..pos+3].copy_from_slice(b"\x1b[r");
        pos += 3;

        // Reposition cursor to where output was (ESC[r reset it to 1,1)
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 2);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        self.write_uart(&buf[..pos]);
    }

    /// Render the input line from InputState in shmem.
    ///
    /// Moves directly to the input row (row=rows) without save/restore —
    /// ESC[s/ESC[u is reserved for the output cursor position.
    fn render_input_line(&mut self) {
        use core::sync::atomic::Ordering;

        let ring = match self.shell_ring.as_ref() {
            Some(r) => r,
            None => return,
        };

        let input = ring.input_state();
        let seq = input.seq.load(Ordering::Acquire);
        if seq == self.last_input_seq {
            return; // No change
        }
        self.last_input_seq = seq;

        // Check if input mode should change
        let active = (input.flags & userlib::console_ring::input_flags::ACTIVE) != 0;
        if !active {
            if self.input_mode {
                self.exit_input_mode();
            }
            return;
        }

        let len = (input.len as usize).min(128);
        let cursor = (input.cursor as usize).min(len);
        let prompt_bytes = self.input_prompt_len as usize;
        let prompt_vis = self.input_prompt_visible as usize;

        // Build the entire UART write in one buffer to minimize syscalls
        let mut buf = [0u8; 256];
        let mut pos = 0;

        // Move to input row, column 1 (no save/restore — ESC[s is for output pos)
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        // Write prompt (may contain ANSI escapes)
        let prompt = &self.input_prompt[..prompt_bytes];
        let copy_len = prompt.len().min(buf.len() - pos);
        buf[pos..pos+copy_len].copy_from_slice(&prompt[..copy_len]);
        pos += copy_len;

        // Write input text
        let text_copy = len.min(buf.len() - pos);
        buf[pos..pos+text_copy].copy_from_slice(&input.buf[..text_copy]);
        pos += text_copy;

        // Clear to end of line
        buf[pos..pos+3].copy_from_slice(b"\x1b[K");
        pos += 3;

        // Position cursor using visible prompt width (not byte length)
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows);
        buf[pos] = b';';
        pos += 1;
        pos += write_u16_to_buf(&mut buf[pos..], (prompt_vis + cursor + 1) as u16);
        buf[pos] = b'H';
        pos += 1;

        self.write_uart(&buf[..pos]);
    }

    /// Write output while in input mode — output goes into scroll region,
    /// then cursor returns to input line position.
    ///
    /// Uses explicit column tracking (`output_col`) instead of ESC[s/ESC[u,
    /// which is resilient to external UART writes (klog) that corrupt
    /// terminal save/restore state.
    fn write_output_input_mode(&mut self, data: &[u8]) {
        let ring = match self.shell_ring.as_ref() {
            Some(r) => r,
            None => return,
        };

        let input = ring.input_state();
        let len = (input.len as usize).min(128);
        let cursor = (input.cursor as usize).min(len);
        let prompt_vis = self.input_prompt_visible as usize;

        // Move to output position in scroll region (bottom row, tracked column)
        let mut pre = [0u8; 16];
        let mut ppos = 0;
        pre[ppos..ppos+2].copy_from_slice(b"\x1b[");
        ppos += 2;
        ppos += write_u16_to_buf(&mut pre[ppos..], self.rows - 2);
        pre[ppos] = b';';
        ppos += 1;
        ppos += write_u16_to_buf(&mut pre[ppos..], self.output_col);
        pre[ppos] = b'H';
        ppos += 1;
        self.write_uart(&pre[..ppos]);

        // Write the output data (scroll region handles scrolling)
        self.write_uart(data);

        // Update output_col by scanning data, properly skipping ANSI escapes
        let cols = self.cols;
        let mut esc: u8 = 0; // 0=normal, 1=after ESC, 2=in CSI
        for &b in data {
            match esc {
                1 => {
                    // After ESC: '[' starts CSI, anything else = 2-char escape
                    esc = if b == b'[' { 2 } else { 0 };
                }
                2 => {
                    // In CSI: terminated by 0x40..=0x7e (letter)
                    if b >= 0x40 && b <= 0x7e {
                        esc = 0;
                    }
                }
                _ => match b {
                    0x1b => esc = 1,
                    b'\n' | b'\r' => self.output_col = 1,
                    0x20..=0x7e => {
                        self.output_col += 1;
                        if self.output_col > cols {
                            self.output_col = 1; // Line wrap
                        }
                    }
                    _ => {}
                },
            }
        }

        // Return cursor to input line
        let mut post = [0u8; 16];
        let mut qpos = 0;
        post[qpos..qpos+2].copy_from_slice(b"\x1b[");
        qpos += 2;
        qpos += write_u16_to_buf(&mut post[qpos..], self.rows);
        post[qpos] = b';';
        qpos += 1;
        qpos += write_u16_to_buf(&mut post[qpos..], (prompt_vis + cursor + 1) as u16);
        post[qpos] = b'H';
        qpos += 1;
        self.write_uart(&post[..qpos]);
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

                    // Grant shell access to shmem and send shmem_id
                    if !ring.allow(client_pid) {
                        uerror!("consoled", "shmem_allow_failed";);
                        return;
                    }
                    let shmem_id = ring.shmem_id();
                    let mut msg = [0u8; 12];
                    msg[..4].copy_from_slice(b"RING");
                    msg[4..8].copy_from_slice(&shmem_id.to_le_bytes());
                    msg[8..10].copy_from_slice(&self.cols.to_le_bytes());
                    msg[10..12].copy_from_slice(&self.rows.to_le_bytes());

                    if channel.send(&msg[..12]).is_err() {
                        uerror!("consoled", "send_ring_info_failed";);
                        return;
                    }

                    self.shell_pid = Some(client_pid);
                    unotice!("consoled", "shell_connected"; pid = client_pid);

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
                                unotice!("consoled", "shell_dead_on_stdin";);
                                self.disconnect_shell(ctx);
                                return;
                            }
                            _ => {} // Ok(None)=WouldBlock, Ok(Some)=got data - shell alive
                        }
                    }
                }

                // Check for Ctrl+C (0x03) — signal shell instead of writing to ring
                if let Some(pid) = self.shell_pid {
                    if rx_buf[..n].iter().any(|&b| b == 0x03) {
                        // Scan entire buffer: write non-Ctrl+C segments, signal for each 0x03
                        let mut seg_start = 0;
                        for i in 0..n {
                            if rx_buf[i] == 0x03 {
                                // Write segment before this Ctrl+C
                                if i > seg_start {
                                    if let Some(ring) = &self.shell_ring {
                                        let written = ring.rx_write(&rx_buf[seg_start..i]);
                                        if written > 0 { ring.notify(); }
                                    }
                                }
                                let _ = syscall::signal(pid, 2, 0);
                                seg_start = i + 1;
                            }
                        }
                        // Write any remaining bytes after last Ctrl+C
                        if seg_start < n {
                            if let Some(ring) = &self.shell_ring {
                                let written = ring.rx_write(&rx_buf[seg_start..n]);
                                if written > 0 { ring.notify(); }
                            }
                        }
                        return;
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
                0 // No TX data — handle below after borrow ends
            } else {
                let to_read = tx_avail.min(tx_buf.len());
                ring.tx_read(&mut tx_buf[..to_read])
            }
        };
        // Borrow of self.shell_ring is released here

        if n == 0 {
            // No TX data — check InputState flags and render
            self.check_input_flags();
            if self.input_mode {
                self.render_input_line();
            }
            return;
        }

        // Check InputState flags for input mode transitions (flag-based, not in-band)
        self.check_input_flags();

        // Find command start (skip leading whitespace/newlines which may be echo)
        let mut cmd_start = 0;
        while cmd_start < n && (tx_buf[cmd_start] == b'\n' || tx_buf[cmd_start] == b'\r') {
            // Output the newline first (it's echo from shell)
            if self.input_mode {
                self.write_output_input_mode(&tx_buf[cmd_start..cmd_start+1]);
            } else if self.split_enabled {
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
            let was_input = self.input_mode;
            if was_input {
                // Temporarily remove scroll region
                self.write_uart(b"\x1b[r");
            } else if self.split_enabled {
                self.write_uart(b"\x1b[r");
            }

            match ansi::query_screen_size() {
                Some((cols, rows)) => {
                    self.cols = cols;
                    self.rows = rows;
                }
                None => {}
            }

            // Re-enable scroll region
            if was_input {
                self.input_mode = false;
                self.enter_input_mode();
            } else if self.split_enabled {
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
            if self.input_mode {
                self.write_output_input_mode(cmd_buf);
            } else if self.split_enabled {
                self.write_output(cmd_buf);
            } else {
                self.write_uart(cmd_buf);
            }
        }

        // If in input mode, check InputState after processing TX data
        if self.input_mode {
            self.render_input_line();
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
                    unotice!("consoled", "shell_disconnected";);
                    self.disconnect_shell(ctx);
                }
                _ => {} // Got data or would block - shell still alive
            }
        }
    }

    fn disconnect_shell(&mut self, ctx: &mut dyn BusCtx) {
        // Exit input mode if active
        if self.input_mode {
            self.exit_input_mode();
        }

        // Unwatch handles before dropping
        if let Some(ring) = &self.shell_ring {
            let _ = ctx.unwatch_handle(ring.handle());
        }
        if let Some(ch) = &self.handshake_channel {
            let _ = ctx.unwatch_handle(ch.handle());
        }

        self.shell_ring = None;
        self.handshake_channel = None;
        self.shell_pid = None;

        // Notify port that connection is closed (allows new connections)
        if let Some(port) = &mut self.port {
            port.connection_closed();
        }
        self.state = ConsoleState::WaitingForShell;

        // Transition port Safe → devd fires spawn rule → new shell
        let _ = ctx.set_port_state(b"console:0", PortState::Safe);

        unotice!("consoled", "awaiting_shell";);
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for ConsoledDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        unotice!("consoled", "init";);

        // Claim the kernel UART bus. With Phase 2 NotifyParentExit, devd only
        // restarts consoled after the previous instance's bus handle is released,
        // so uart0 is guaranteed Safe here — no retry needed.
        let uart_path = b"/uart:0";
        match ctx.claim_kernel_bus(uart_path) {
            Ok((_bus_id, _info)) => {
                unotice!("consoled", "uart_claimed";);
            }
            Err(e) => {
                uerror!("consoled", "uart_claim_failed"; err = e as u8);
                return Err(e);
            }
        }

        // Get stdin handle
        self.stdin_handle = match syscall::open(ObjectType::Stdin, &[]) {
            Ok(h) => h,
            Err(_) => {
                uerror!("consoled", "stdin_open_failed";);
                return Err(BusError::Internal);
            }
        };

        // Drain any stale input
        let mut buf = [0u8; 64];
        loop {
            match syscall::read(self.stdin_handle, &mut buf) {
                Ok(n) if n > 0 => {}
                _ => break,
            }
        }

        // Detect terminal size via CPR
        if let Some((cols, rows)) = ansi::query_screen_size() {
            self.cols = cols;
            self.rows = rows;
            unotice!("consoled", "terminal_size"; cols = cols, rows = rows);
        }

        // Create console port
        let port = match Port::with_limit(b"console:0", 1) {
            Ok(p) => p,
            Err(_) => {
                uerror!("consoled", "port_create_failed";);
                return Err(BusError::Internal);
            }
        };

        // Watch stdin and port for events
        if let Err(e) = ctx.watch_handle(self.stdin_handle, TAG_STDIN) {
            uerror!("consoled", "watch_stdin_failed";);
            return Err(e);
        }
        if let Err(e) = ctx.watch_handle(port.handle(), TAG_PORT) {
            uerror!("consoled", "watch_port_failed";);
            return Err(e);
        }

        self.port = Some(port);

        // Register console: port with devd
        let mut info = PortInfo::empty();
        info.set_name(b"console:0");
        info.port_class = PortClass::Console;
        info.port_subclass = port_subclass::CONSOLE_SERIAL;

        if let Err(e) = ctx.register_port_with_info(&info, 0) {
            uerror!("consoled", "register_port_failed";);
            return Err(e);
        }
        // Port starts Safe. Supervisor (devd) fires spawn rules when port transitions to Claimed.

        unotice!("consoled", "ready";);
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
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.reset(ctx)
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
