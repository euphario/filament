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
use userlib::ipc::{Port, Channel, Timer, ObjHandle};
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
const TAG_STATUS_TIMER: u32 = 5;

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
    /// 1-second periodic timer for status bar updates
    status_timer: Option<Timer>,
    /// Alternates heartbeat symbol each tick
    heartbeat_toggle: bool,
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
            status_timer: None,
            heartbeat_toggle: false,
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

        // Scroll screen up to make room for separator + input + status lines.
        // Without this, short output (e.g., `uptime`) at the bottom of the
        // screen gets overwritten by the separator drawn at rows-2.
        let mut scroll = [0u8; 16];
        let mut spos = 0;
        // Move to bottom of screen
        scroll[spos..spos+2].copy_from_slice(b"\x1b[");
        spos += 2;
        spos += write_u16_to_buf(&mut scroll[spos..], self.rows);
        scroll[spos..spos+3].copy_from_slice(b";1H");
        spos += 3;
        // Print newlines to push content up (3 = separator + input + status)
        scroll[spos] = b'\n';
        scroll[spos+1] = b'\n';
        scroll[spos+2] = b'\n';
        spos += 3;
        self.write_uart(&scroll[..spos]);

        let mut buf = [0u8; 64];
        let mut pos = 0;

        // Set scroll region: rows 1..rows-3 for output
        // Row rows-2 = separator, row rows-1 = input, row rows = status
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        buf[pos..pos+2].copy_from_slice(b"1;");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 3);
        buf[pos] = b'r';
        pos += 1;

        self.write_uart(&buf[..pos]);

        // Draw separator line on row rows-2
        self.draw_separator();

        // Move cursor to input row (rows-1) and draw prompt
        let mut buf2 = [0u8; 32];
        let mut pos2 = 0;
        buf2[pos2..pos2+2].copy_from_slice(b"\x1b[");
        pos2 += 2;
        pos2 += write_u16_to_buf(&mut buf2[pos2..], self.rows - 1);
        buf2[pos2..pos2+3].copy_from_slice(b";1H");
        pos2 += 3;
        buf2[pos2..pos2+3].copy_from_slice(b"\x1b[K");
        pos2 += 3;
        self.write_uart(&buf2[..pos2]);

        // Draw prompt
        self.write_uart(&self.input_prompt[..self.input_prompt_len as usize]);

        self.input_mode = true;

        // Draw initial status line
        self.render_status_line();
    }

    /// Draw horizontal separator line on row rows-2
    fn draw_separator(&mut self) {
        let mut buf = [0u8; 1024];
        let mut pos = 0;

        // Move to separator row
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 2);
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

        let mut buf = [0u8; 80];
        let mut pos = 0;

        // Clear separator line (row=rows-2)
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 2);
        buf[pos..pos+6].copy_from_slice(b";1H\x1b[K");
        pos += 6;

        // Clear the input line (row=rows-1)
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 1);
        buf[pos..pos+6].copy_from_slice(b";1H\x1b[K");
        pos += 6;

        // Clear the status line (row=rows)
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
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 3);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        self.write_uart(&buf[..pos]);
    }

    /// Render the input line from InputState in shmem.
    ///
    /// Moves directly to the input row (row=rows-1) without save/restore —
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

        // Move to input row (rows-1), column 1
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 1);
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
        pos += write_u16_to_buf(&mut buf[pos..], self.rows - 1);
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

        // Move to output position in scroll region (bottom row = rows-3, tracked column)
        let mut pre = [0u8; 16];
        let mut ppos = 0;
        pre[ppos..ppos+2].copy_from_slice(b"\x1b[");
        ppos += 2;
        ppos += write_u16_to_buf(&mut pre[ppos..], self.rows - 3);
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

        // Return cursor to input line (rows-1)
        let mut post = [0u8; 16];
        let mut qpos = 0;
        post[qpos..qpos+2].copy_from_slice(b"\x1b[");
        qpos += 2;
        qpos += write_u16_to_buf(&mut post[qpos..], self.rows - 1);
        post[qpos] = b';';
        qpos += 1;
        qpos += write_u16_to_buf(&mut post[qpos..], (prompt_vis + cursor + 1) as u16);
        post[qpos] = b'H';
        qpos += 1;
        self.write_uart(&post[..qpos]);
    }

    /// Render the status line on row=rows (below input line).
    /// Shows uptime and heartbeat indicator, returns cursor to input line.
    fn render_status_line(&mut self) {
        let now_ns = syscall::gettime();
        let total_secs = (now_ns / 1_000_000_000) as u32;
        let hours = total_secs / 3600;
        let mins = (total_secs % 3600) / 60;
        let secs = total_secs % 60;

        // ♥ = U+2665 (E2 99 A5), ♡ = U+2661 (E2 99 A1)
        let heart: &[u8] = if self.heartbeat_toggle {
            &[0xe2, 0x99, 0xa5] // ♥
        } else {
            &[0xe2, 0x99, 0xa1] // ♡
        };

        let mut buf = [0u8; 1024];
        let mut pos = 0;

        // Hide cursor during status line render to prevent flicker
        buf[pos..pos+6].copy_from_slice(b"\x1b[?25l");
        pos += 6;

        // Move to status row
        buf[pos..pos+2].copy_from_slice(b"\x1b[");
        pos += 2;
        pos += write_u16_to_buf(&mut buf[pos..], self.rows);
        buf[pos..pos+3].copy_from_slice(b";1H");
        pos += 3;

        // Dim color
        buf[pos..pos+4].copy_from_slice(b"\x1b[2m");
        pos += 4;

        // Leading "── up "
        // ─ = U+2500 (E2 94 80)
        buf[pos..pos+3].copy_from_slice(&[0xe2, 0x94, 0x80]);
        pos += 3;
        buf[pos..pos+3].copy_from_slice(&[0xe2, 0x94, 0x80]);
        pos += 3;
        buf[pos..pos+4].copy_from_slice(b" up ");
        pos += 4;

        // Format uptime
        if hours > 0 {
            pos += write_u16_to_buf(&mut buf[pos..], hours as u16);
            buf[pos] = b'h';
            pos += 1;
        }
        pos += write_u16_to_buf(&mut buf[pos..], mins as u16);
        buf[pos] = b'm';
        pos += 1;
        // Zero-pad seconds
        if secs < 10 {
            buf[pos] = b'0';
            pos += 1;
        }
        pos += write_u16_to_buf(&mut buf[pos..], secs as u16);
        buf[pos] = b's';
        pos += 1;

        // " ── "
        buf[pos] = b' ';
        pos += 1;
        buf[pos..pos+3].copy_from_slice(&[0xe2, 0x94, 0x80]);
        pos += 3;
        buf[pos..pos+3].copy_from_slice(&[0xe2, 0x94, 0x80]);
        pos += 3;
        buf[pos] = b' ';
        pos += 1;

        // Heartbeat (reset dim, show heart in red or dim red)
        buf[pos..pos+4].copy_from_slice(b"\x1b[0m"); // reset
        pos += 4;
        if self.heartbeat_toggle {
            buf[pos..pos+5].copy_from_slice(b"\x1b[31m"); // red
            pos += 5;
        } else {
            buf[pos..pos+7].copy_from_slice(b"\x1b[2;31m"); // dim red
            pos += 7;
        }
        buf[pos..pos+3].copy_from_slice(heart);
        pos += 3;
        buf[pos..pos+4].copy_from_slice(b"\x1b[0m"); // reset
        pos += 4;
        buf[pos..pos+4].copy_from_slice(b"\x1b[2m"); // dim again
        pos += 4;

        // " ──────..." fill remaining with ─
        buf[pos] = b' ';
        pos += 1;

        // Calculate how many chars we've drawn so far (visible)
        // "── up XhYYmZZs ── ♥ " = ~20-25 chars depending on uptime
        // Each ─ and ♥ is 1 display column despite multi-byte UTF-8
        let used_cols: u16 = {
            // "── up " = 6
            let mut c: u16 = 6;
            if hours > 0 {
                c += if hours >= 10 { 3 } else { 2 }; // Nh
            }
            c += if mins >= 10 { 3 } else { 2 }; // Nm or NNm
            c += 3; // NNs (always 2 digits + s)
            c += 6; // " ── ♥ "
            c
        };

        let remaining = if self.cols > used_cols {
            (self.cols - used_cols) as usize
        } else {
            0
        };
        let fill = remaining.min((buf.len() - pos - 8) / 3);
        for _ in 0..fill {
            buf[pos..pos+3].copy_from_slice(&[0xe2, 0x94, 0x80]);
            pos += 3;
        }

        // Reset color
        buf[pos..pos+4].copy_from_slice(b"\x1b[0m");
        pos += 4;

        self.write_uart(&buf[..pos]);

        // Return cursor to input line (rows-1)
        if self.input_mode {
            let ring = match self.shell_ring.as_ref() {
                Some(r) => r,
                None => return,
            };
            let input = ring.input_state();
            let len = (input.len as usize).min(128);
            let cursor = (input.cursor as usize).min(len);
            let prompt_vis = self.input_prompt_visible as usize;

            let mut ret = [0u8; 32];
            let mut rpos = 0;
            ret[rpos..rpos+2].copy_from_slice(b"\x1b[");
            rpos += 2;
            rpos += write_u16_to_buf(&mut ret[rpos..], self.rows - 1);
            ret[rpos] = b';';
            rpos += 1;
            rpos += write_u16_to_buf(&mut ret[rpos..], (prompt_vis + cursor + 1) as u16);
            ret[rpos] = b'H';
            rpos += 1;
            // Show cursor again after repositioning
            ret[rpos..rpos+6].copy_from_slice(b"\x1b[?25h");
            rpos += 6;
            self.write_uart(&ret[..rpos]);
        } else {
            // Not in input mode — just show cursor
            self.write_uart(b"\x1b[?25h");
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

    /// Write directly to UART (blocking — retries on partial write)
    ///
    /// With event-driven TX drain, write(STDOUT) pushes into a 4KB kernel
    /// ring buffer. The TX interrupt drains the ring to the UART FIFO
    /// asynchronously. When the ring is full, write returns partial or
    /// WouldBlock. We sleep briefly and retry — the TX IRQ frees space
    /// within ~1ms at 115200 baud.
    fn write_uart(&self, data: &[u8]) {
        let mut offset = 0;
        while offset < data.len() {
            match syscall::write(Handle::STDOUT, &data[offset..]) {
                Ok(n) if n > 0 => {
                    offset += n;
                }
                _ => {
                    // WouldBlock or 0 bytes: UART ring full — wait for TX IRQ to drain
                    syscall::sleep_us(500); // ~0.5ms, enough for 16 bytes at 115200
                }
            }
        }
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

                // Ctrl+C (0x03): INTERRUPT signal + pass byte to ring (readline ^C cancel)
                // Ctrl+\ (0x1c): SHUTDOWN signal, strip from ring (hard kill)
                if let Some(pid) = self.shell_pid {
                    if rx_buf[..n].iter().any(|&b| b == 0x03 || b == 0x1c) {
                        let mut seg_start = 0;
                        for i in 0..n {
                            if rx_buf[i] == 0x03 {
                                // Send signal; keep 0x03 in stream for readline
                                let _ = syscall::signal(pid, 4, 0);
                            } else if rx_buf[i] == 0x1c {
                                // Write segment before Ctrl+\, skip the byte itself
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
                        // Write remaining (includes Ctrl+C bytes, excludes Ctrl+\)
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

    /// Poll STDIN for Ctrl+C / Ctrl+\ between TX drain chunks.
    /// Returns true if a kill signal was sent (caller should stop output).
    fn poll_stdin_for_ctrl(&mut self) -> bool {
        let mut byte = [0u8; 1];
        // Non-blocking read — drain any pending input
        loop {
            match syscall::read(self.stdin_handle, &mut byte) {
                Ok(1) => {
                    if let Some(pid) = self.shell_pid {
                        if byte[0] == 0x03 {
                            // Ctrl+C: INTERRUPT + pass to ring for readline
                            let _ = syscall::signal(pid, 4, 0);
                            if let Some(ring) = &self.shell_ring {
                                ring.rx_write(&byte);
                                ring.notify();
                            }
                            return true;
                        } else if byte[0] == 0x1c {
                            // Ctrl+\: SHUTDOWN (hard kill), strip from ring
                            let _ = syscall::signal(pid, 2, 0);
                            return true;
                        } else {
                            // Normal byte — write to ring
                            if let Some(ring) = &self.shell_ring {
                                ring.rx_write(&byte);
                                ring.notify();
                            }
                        }
                    }
                }
                _ => break,
            }
        }
        false
    }

    /// Handle shmem event: drain TX ring to UART.
    ///
    /// Tight loop is fine — write_uart() blocks at UART speed, which IS
    /// the natural pacing. write_uart() retries internally when the UART
    /// ring is full, yielding CPU via sleep (not spinning). The TX IRQ
    /// drains the UART ring to the FIFO asynchronously.
    ///
    /// After reading each chunk, if the shmem ring drops below 50% full,
    /// notify the shell so it can resume writing (back-pressure).
    ///
    /// Drains ALL available data from the shmem ring into a 4KB accumulation
    /// buffer before writing to UART. This prevents ANSI escape sequences in
    /// the shell's output from being split across multiple write_output calls,
    /// where consoled's cursor-positioning ANSI would be inserted between
    /// chunks, breaking the terminal's escape sequence parser.
    fn handle_shmem_readable(&mut self, _ctx: &mut dyn BusCtx) {
        // Outer loop: process all available data in 4KB chunks.
        // Each chunk is accumulated fully before writing, preventing ANSI
        // escape sequences from being split by consoled's cursor-positioning.
        //
        // Carry buffer: if a chunk ends mid-ANSI-escape (e.g. \x1b[2;3 at
        // the 4KB boundary), the incomplete tail is carried to the next chunk
        // so it isn't split by cursor-positioning sequences between writes.
        let mut did_output = false;
        let mut carry = [0u8; 16];
        let mut carry_len: usize = 0;

        loop {
            let mut acc = [0u8; 4096];
            let mut acc_len: usize = 0;

            // Prepend any carried-over bytes from previous chunk
            if carry_len > 0 {
                acc[..carry_len].copy_from_slice(&carry[..carry_len]);
                acc_len = carry_len;
                carry_len = 0;
            }

            // Accumulate up to 4KB from ring
            loop {
                let ring = match self.shell_ring.as_ref() {
                    Some(r) => r,
                    None => return,
                };
                let tx_avail = ring.tx_available();
                if tx_avail == 0 {
                    break;
                }
                let space = acc.len() - acc_len;
                if space == 0 {
                    break;
                }
                let to_read = tx_avail.min(space);
                let n = ring.tx_read(&mut acc[acc_len..acc_len + to_read]);
                if n == 0 {
                    break;
                }
                acc_len += n;

                // Notify shell after each drain chunk so it can refill
                ring.notify();
            }

            if acc_len == 0 {
                break; // Ring empty — exit outer loop
            }

            // Check if ring has more data (we'll loop again).
            // If so, scan for an incomplete ANSI escape at the end and carry it over.
            let more_data = match self.shell_ring.as_ref() {
                Some(r) => r.tx_available() > 0 || acc_len == acc.len(),
                None => false,
            };
            if more_data {
                // Find last ESC (0x1B) in the final 16 bytes
                let scan_start = if acc_len > 16 { acc_len - 16 } else { 0 };
                let mut last_esc = None;
                for i in (scan_start..acc_len).rev() {
                    if acc[i] == 0x1b {
                        last_esc = Some(i);
                        break;
                    }
                }
                if let Some(esc_pos) = last_esc {
                    // Check if the escape sequence is complete (terminated by a letter)
                    let mut complete = false;
                    for j in (esc_pos + 1)..acc_len {
                        if acc[j] >= b'A' && acc[j] <= b'z' {
                            complete = true;
                            break;
                        }
                    }
                    if !complete {
                        // Incomplete ANSI escape — carry it to next chunk
                        let tail_len = acc_len - esc_pos;
                        if tail_len <= carry.len() {
                            carry[..tail_len].copy_from_slice(&acc[esc_pos..acc_len]);
                            carry_len = tail_len;
                            acc_len = esc_pos;
                            if acc_len == 0 {
                                continue; // Entire chunk was just a partial escape
                            }
                        }
                    }
                }
            }

            did_output = true;

            // Poll STDIN so Ctrl+C/Ctrl+\ remain responsive between chunks
            if self.poll_stdin_for_ctrl() {
                break;
            }

            // Check InputState flags for input mode transitions
            self.check_input_flags();

            // First chunk: scan for commands (SPLIT, NOSPLIT, GETSIZE).
            // Commands are only sent as complete messages from the shell, never
            // interleaved with bulk output data. They start at byte 0 after
            // any leading newlines.
            let mut data_start = 0;
            while data_start < acc_len && (acc[data_start] == b'\n' || acc[data_start] == b'\r') {
                data_start += 1;
            }

            // Write leading newlines
            if data_start > 0 {
                if self.input_mode {
                    self.write_output_input_mode(&acc[..data_start]);
                } else if self.split_enabled {
                    self.write_output(&acc[..data_start]);
                } else {
                    self.write_uart(&acc[..data_start]);
                }
            }

            let cmd_buf = &acc[data_start..acc_len];
            let cmd_len = acc_len - data_start;

            if cmd_len >= 5 && &cmd_buf[..5] == b"SPLIT" {
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
                let was_input = self.input_mode;
                if was_input {
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
                // Bulk data output — write accumulated chunk at once.
                if self.input_mode {
                    self.write_output_input_mode(cmd_buf);
                } else if self.split_enabled {
                    self.write_output(cmd_buf);
                } else {
                    self.write_uart(cmd_buf);
                }
            }
        } // end outer loop

        if !did_output {
            // No TX data — shell may have only updated InputState (echo).
            // Check flags and render input line from shared memory.
            self.check_input_flags();
        }

        if self.input_mode {
            self.render_input_line();
        }
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

        // Create 1-second status bar timer
        if let Ok(timer) = Timer::new() {
            if let Err(e) = ctx.watch_handle(timer.handle(), TAG_STATUS_TIMER) {
                uerror!("consoled", "watch_timer_failed";);
                return Err(e);
            }
            self.status_timer = Some(timer);
            // Arm for 1 second
            if let Some(t) = &mut self.status_timer {
                let _ = t.set(1_000_000_000);
            }
        }

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
            TAG_STATUS_TIMER => {
                // Re-arm timer for 1 second
                if let Some(t) = &mut self.status_timer {
                    let _ = t.set(1_000_000_000);
                }
                // Toggle heartbeat and render if in input mode
                self.heartbeat_toggle = !self.heartbeat_toggle;
                if self.input_mode {
                    self.render_status_line();
                }
            }
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
