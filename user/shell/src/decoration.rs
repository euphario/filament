//! Shell decoration — separator line, status bar with uptime + heartbeat
//!
//! Layout (bottom 3 rows):
//! ```text
//! Row 1..rows-3  : Output area (DECSTBM scroll region)
//! Row rows-2     : ─── dim separator ────────────────────
//! Row rows-1     : / > command input (cursor lives here)
//! Row rows       : ── up 1h23m ── ♥ ─────────────────────
//! ```

use crate::console;
use libsys::syscall;

/// Heartbeat toggle state
static mut HEARTBEAT_SOLID: bool = true;

/// Whether decoration is active
static mut ACTIVE: bool = false;

/// Cached dimensions
static mut COLS: u16 = 80;
static mut ROWS: u16 = 24;

/// Current cursor column on the prompt row (1-based).
/// Updated by set_prompt_cursor() from readline.
static mut PROMPT_COL: u16 = 1;

/// Set up decoration: scroll region + separator + status bar.
pub fn setup(cols: u16, rows: u16) {
    if rows < 8 || cols < 20 {
        return;
    }

    unsafe {
        COLS = cols;
        ROWS = rows;
        ACTIVE = true;
    }

    // Clear screen, set scroll region, position cursor at top
    let mut buf = [0u8; 32];
    let mut pos = 0;
    pos += put(&mut buf[pos..], b"\x1b[2J");
    // Set scroll region: 1..(rows-3)
    pos += put(&mut buf[pos..], b"\x1b[1;");
    pos += put_u16(&mut buf[pos..], rows - 3);
    pos += put(&mut buf[pos..], b"r");
    pos += put(&mut buf[pos..], b"\x1b[1;1H");
    console::write(&buf[..pos]);

    // Draw separator at rows-2
    draw_separator();

    // Draw status bar at rows
    draw_status_bar();

    // Return cursor to top of scroll region (1;1) for banner text
    console::write(b"\x1b[1;1H");
}

/// Redraw decoration after screen clear.
pub fn redraw() {
    if !unsafe { ACTIVE } {
        return;
    }
    let cols = unsafe { COLS };
    let rows = unsafe { ROWS };
    setup(cols, rows);
}

/// Tear down decoration: reset scroll region, clear screen.
pub fn teardown() {
    unsafe { ACTIVE = false; }
    // Reset scroll region to full screen, clear, home
    console::write(b"\x1b[r\x1b[2J\x1b[1;1H");
}

/// Whether decoration is currently active.
pub fn is_active() -> bool {
    unsafe { ACTIVE }
}

/// Move cursor to the prompt row (rows-1), clear it.
pub fn move_to_input() {
    if !unsafe { ACTIVE } {
        return;
    }
    let rows = unsafe { ROWS };
    let mut buf = [0u8; 16];
    let mut pos = 0;
    pos += put(&mut buf[pos..], b"\x1b[");
    pos += put_u16(&mut buf[pos..], rows - 1);
    pos += put(&mut buf[pos..], b";1H\x1b[2K");
    console::write(&buf[..pos]);
}

/// Move cursor back into the scroll region for command output.
pub fn move_to_output() {
    if !unsafe { ACTIVE } {
        console::write(b"\r\n");
        return;
    }
    let rows = unsafe { ROWS };
    let mut buf = [0u8; 16];
    let mut pos = 0;
    pos += put(&mut buf[pos..], b"\x1b[");
    pos += put_u16(&mut buf[pos..], rows - 3);
    pos += put(&mut buf[pos..], b";1H\r\n");
    console::write(&buf[..pos]);
}

/// Update the tracked cursor column on the prompt row.
/// Called from readline whenever the cursor position changes.
pub fn set_prompt_cursor(col: u16) {
    unsafe { PROMPT_COL = col; }
}

/// Refresh the status bar without disturbing the cursor.
/// Uses explicit cursor repositioning (not ESC[s/u which is unreliable
/// through the SharedPipe → consoled relay).
pub fn refresh() {
    if !unsafe { ACTIVE } {
        return;
    }
    let ssh = console::console().ssh_mode;
    let pipe_before = if ssh { console::console().pipe_writable() } else { 0 };

    draw_status_bar();
    // Move cursor back to prompt row at tracked column
    let rows = unsafe { ROWS };
    let col = unsafe { PROMPT_COL };
    let mut buf = [0u8; 16];
    let mut pos = 0;
    pos += put(&mut buf[pos..], b"\x1b[");
    pos += put_u16(&mut buf[pos..], rows - 1);
    pos += put(&mut buf[pos..], b";");
    pos += put_u16(&mut buf[pos..], col);
    pos += put(&mut buf[pos..], b"H");
    console::write(&buf[..pos]);

    if ssh {
        let pipe_after = console::console().pipe_writable();
        let bytes_out = pipe_before.saturating_sub(pipe_after);
        libsys::udebug!("shell", "heartbeat"; bytes = bytes_out as u32, pipe_free = pipe_after as u32);
    }
}

/// Draw horizontal separator line on row rows-2.
fn draw_separator() {
    let cols = unsafe { COLS };
    let rows = unsafe { ROWS };

    let mut buf = [0u8; 768];
    let mut pos = 0;

    // Move to separator row
    pos += put(&mut buf[pos..], b"\x1b[");
    pos += put_u16(&mut buf[pos..], rows - 2);
    pos += put(&mut buf[pos..], b";1H\x1b[2m");

    // ─ (U+2500), 3 bytes per char
    let max_chars = (cols as usize).min((buf.len() - pos - 8) / 3);
    for _ in 0..max_chars {
        pos += put(&mut buf[pos..], b"\xe2\x94\x80");
    }

    pos += put(&mut buf[pos..], b"\x1b[0m");
    console::write(&buf[..pos]);
}

/// Draw full-width status bar on row rows.
/// Format: ── up 1h23m45s ── ♥ ────────────────
fn draw_status_bar() {
    let cols = unsafe { COLS };
    let rows = unsafe { ROWS };

    let secs = (syscall::gettime() / 1_000_000_000) as u32;

    let mut buf = [0u8; 128];
    let mut pos = 0;

    // Move to status bar row, clear it
    pos += put(&mut buf[pos..], b"\x1b[");
    pos += put_u16(&mut buf[pos..], rows);
    pos += put(&mut buf[pos..], b";1H\x1b[2K");

    // Dim
    pos += put(&mut buf[pos..], b"\x1b[2m");
    pos += put(&mut buf[pos..], b"\xe2\x94\x80\xe2\x94\x80 up ");

    pos += fmt_uptime(&mut buf[pos..], secs);

    pos += put(&mut buf[pos..], b" \xe2\x94\x80\xe2\x94\x80 ");
    pos += put(&mut buf[pos..], b"\x1b[0m");

    // Heartbeat
    let solid = unsafe {
        HEARTBEAT_SOLID = !HEARTBEAT_SOLID;
        !HEARTBEAT_SOLID
    };
    if solid {
        pos += put(&mut buf[pos..], b"\x1b[91m\xe2\x99\xa5\x1b[0m");
    } else {
        pos += put(&mut buf[pos..], b"\x1b[2;31m\xe2\x99\xa1\x1b[0m");
    }

    // Write preamble
    console::write(&buf[..pos]);

    // Trailing ── fill (separate write to avoid overflow)
    let uptime_chars = fmt_uptime_len(secs);
    let used = 6 + uptime_chars + 4 + 1; // "── up " + time + " ── " + heart
    let remaining = if (cols as usize) > used + 1 { (cols as usize) - used - 1 } else { 0 };

    // Write fill in a separate buffer
    let mut fill_buf = [0u8; 768];
    let mut fpos = 0;
    fpos += put(&mut fill_buf[fpos..], b"\x1b[2m ");
    let fill = remaining.min((fill_buf.len() - fpos - 8) / 3);
    for _ in 0..fill {
        fpos += put(&mut fill_buf[fpos..], b"\xe2\x94\x80");
    }
    fpos += put(&mut fill_buf[fpos..], b"\x1b[0m");
    console::write(&fill_buf[..fpos]);
}

fn fmt_uptime(buf: &mut [u8], total_secs: u32) -> usize {
    let mut pos = 0;
    let hours = total_secs / 3600;
    let mins = (total_secs % 3600) / 60;
    let secs = total_secs % 60;
    if hours > 0 {
        pos += put_u16(&mut buf[pos..], hours as u16);
        pos += put(&mut buf[pos..], b"h");
    }
    if hours > 0 || mins > 0 {
        pos += put_u16(&mut buf[pos..], mins as u16);
        pos += put(&mut buf[pos..], b"m");
    }
    pos += put_u16(&mut buf[pos..], secs as u16);
    pos += put(&mut buf[pos..], b"s");
    pos
}

fn fmt_uptime_len(total_secs: u32) -> usize {
    let hours = total_secs / 3600;
    let mins = (total_secs % 3600) / 60;
    let secs = total_secs % 60;
    let mut len = 0;
    if hours > 0 { len += num_digits(hours as u16) + 1; }
    if hours > 0 || mins > 0 { len += num_digits(mins as u16) + 1; }
    len += num_digits(secs as u16) + 1;
    len
}

fn num_digits(n: u16) -> usize {
    if n == 0 { return 1; }
    let mut count = 0;
    let mut v = n;
    while v > 0 { count += 1; v /= 10; }
    count
}

fn put(buf: &mut [u8], src: &[u8]) -> usize {
    let n = src.len().min(buf.len());
    buf[..n].copy_from_slice(&src[..n]);
    n
}

fn put_u16(buf: &mut [u8], val: u16) -> usize {
    if val == 0 {
        if !buf.is_empty() { buf[0] = b'0'; }
        return 1;
    }
    let mut tmp = [0u8; 5];
    let mut v = val;
    let mut len = 0;
    while v > 0 {
        tmp[len] = b'0' + (v % 10) as u8;
        v /= 10;
        len += 1;
    }
    let n = len.min(buf.len());
    for i in 0..n {
        buf[i] = tmp[len - 1 - i];
    }
    n
}
