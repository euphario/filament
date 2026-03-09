//! Console Daemon (consoled)
//!
//! Bus-framework driver that manages the serial console.
//! Thin transport: UART ↔ SharedPipe. Spawns shell as a child process.
//!
//! ## Architecture
//!
//! ```text
//! ┌────────────┐     ┌──────────┐     ┌────────────┐
//! │   Shell    │ ←── │ consoled │ ←── │ Kernel     │
//! │  (child)   │ ring│ (driver) │uart │ UART bus   │
//! └────────────┘     └──────────┘     └────────────┘
//! ```
//!
//! Shell is spawned via exec_with_mailbox. The mailbox carries the
//! SharedPipe shmem_id + terminal dimensions. No port discovery needed.

#![no_std]
#![no_main]

use libsys::syscall::{self, Handle, ObjectType};
use libsys::ipc::{Timer, ObjHandle};
use libf::sync::SharedPipe;
use libf::time::Duration;
use libos::supervision::SupervisionHandle;
use libos::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition,
    PortInfo, PortClass, PortState, port_subclass,
};
use libos::bus_runtime::driver_main;
use libos::{uinfo, unotice, uerror};

// =============================================================================
// Handle Tags
// =============================================================================

const TAG_STDIN: u32 = 1;
const TAG_SHMEM: u32 = 3;
const TAG_SUPERQ: u32 = 6;

// =============================================================================
// ANSI helpers
// =============================================================================

mod ansi {
    use libsys::syscall::{self, Handle};

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

pub struct ConsoledDriver {
    /// Bidirectional pipe for shell I/O
    shell_ring: Option<SharedPipe>,
    /// Supervision handle for watching child shell
    superq: Option<SupervisionHandle>,
    stdin_handle: ObjHandle,
    cols: u16,
    rows: u16,
    /// Split screen mode enabled
    split_enabled: bool,
    /// Number of lines for log region (top)
    log_lines: u16,
    /// PID of the child shell (for signal delivery)
    shell_pid: Option<u32>,
}

impl ConsoledDriver {
    pub const fn new() -> Self {
        Self {
            shell_ring: None,
            superq: None,
            stdin_handle: Handle::INVALID,
            cols: 80,
            rows: 24,
            split_enabled: false,
            log_lines: 5,
            shell_pid: None,
        }
    }

    // =========================================================================
    // Shell Spawn / Exit
    // =========================================================================

    /// Spawn shell as a child process via exec_with_mailbox.
    fn spawn_shell(&mut self, ctx: &mut dyn BusCtx) {
        // Create shared pipe for shell I/O (64K output, 4K input)
        let ring = match SharedPipe::console() {
            Some(r) => r,
            None => {
                uerror!("consoled", "ring_create_failed";);
                return;
            }
        };

        // Build mailbox: MailboxHeader (64 bytes) + shmem_id(4) + cols(2) + rows(2)
        let mut mailbox = [0u8; 72];
        // MailboxHeader magic
        mailbox[0..4].copy_from_slice(&0x4D424F58u32.to_le_bytes()); // "MBOX"
        mailbox[4..6].copy_from_slice(&1u16.to_le_bytes()); // version
        // Console fields at offset 64
        mailbox[64..68].copy_from_slice(&ring.shmem_id().to_le_bytes());
        mailbox[68..70].copy_from_slice(&self.cols.to_le_bytes());
        mailbox[70..72].copy_from_slice(&self.rows.to_le_bytes());

        let caps = libos::devd::caps::USER_ADMIN;
        match syscall::exec_with_mailbox("shell", caps, &mailbox) {
            Ok((child_pid, _parent_mb_handle, parent_superq_handle)) => {
                // Grant child access to the ring shmem
                ring.allow(child_pid);

                unotice!("consoled", "shell_spawned"; pid = child_pid);

                // Watch ring shmem for shell TX data
                let _ = ctx.watch_handle(ring.handle(), TAG_SHMEM);

                // Watch SuperQ for child exit
                let superq = SupervisionHandle::from_handle(parent_superq_handle);
                let _ = ctx.watch_handle(parent_superq_handle, TAG_SUPERQ);

                self.shell_ring = Some(ring);
                self.superq = Some(superq);
                self.shell_pid = Some(child_pid);
            }
            Err(e) => {
                uerror!("consoled", "shell_spawn_failed"; err = e as i32);
            }
        }
    }

    /// Handle child exit — respawn shell.
    /// Idempotent: safe to call from both SuperQ event and CHILD_EXIT signal.
    fn handle_child_exit(&mut self, ctx: &mut dyn BusCtx) {
        if self.shell_pid.is_none() {
            return; // Already handled
        }

        if let Some(superq) = &self.superq {
            // Drain EXIT note
            let _ = superq.try_recv();
        }

        unotice!("consoled", "shell_exited";);

        // Clean up
        if let Some(ring) = &self.shell_ring {
            let _ = ctx.unwatch_handle(ring.handle());
        }
        if let Some(superq) = &self.superq {
            let _ = ctx.unwatch_handle(superq.handle());
        }
        self.shell_ring = None;
        self.superq = None;
        self.shell_pid = None;

        // Respawn after brief delay
        libf::time::sleep(Duration::from_millis(100));

        // Drain stale UART input
        let mut buf = [0u8; 64];
        loop {
            match syscall::read(self.stdin_handle, &mut buf) {
                Ok(n) if n > 0 => {}
                _ => break,
            }
        }

        self.spawn_shell(ctx);
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
        self.write_uart(b"\x1b[r\x1b[2J\x1b[H");
    }

    /// Write directly to UART (blocking)
    fn write_uart(&self, data: &[u8]) {
        let mut offset = 0;
        while offset < data.len() {
            match syscall::write(Handle::STDOUT, &data[offset..]) {
                Ok(n) if n > 0 => {
                    offset += n;
                }
                _ => {
                    syscall::sleep_us(500);
                }
            }
        }
    }

    // =========================================================================
    // STDIN Handling
    // =========================================================================

    fn handle_stdin_readable(&mut self, _ctx: &mut dyn BusCtx) {
        let mut rx_buf = [0u8; 64];

        match syscall::read(self.stdin_handle, &mut rx_buf) {
            Ok(n) if n > 0 => {
                // Signal relay: Ctrl+C and Ctrl+\
                if let Some(pid) = self.shell_pid {
                    if rx_buf[..n].iter().any(|&b| b == 0x03 || b == 0x1c) {
                        let mut seg_start = 0;
                        for i in 0..n {
                            if rx_buf[i] == 0x03 {
                                // Ctrl+C: INTERRUPT signal, keep byte in stream
                                let _ = syscall::signal(pid, syscall::signal_event::INTERRUPT, 0);
                            } else if rx_buf[i] == 0x1c {
                                // Ctrl+\: SHUTDOWN signal, strip byte
                                if i > seg_start {
                                    if let Some(ring) = &self.shell_ring {
                                        let written = ring.push(&rx_buf[seg_start..i]);
                                        if written > 0 { ring.notify(); }
                                    }
                                }
                                let _ = syscall::signal(pid, syscall::signal_event::SHUTDOWN, 0);
                                seg_start = i + 1;
                            }
                        }
                        // Write remaining (includes Ctrl+C bytes, excludes Ctrl+\)
                        if seg_start < n {
                            if let Some(ring) = &self.shell_ring {
                                let written = ring.push(&rx_buf[seg_start..n]);
                                if written > 0 { ring.notify(); }
                            }
                        }
                        return;
                    }
                }

                if let Some(ring) = &self.shell_ring {
                    let written = ring.push(&rx_buf[..n]);
                    if written > 0 {
                        ring.notify();
                    }
                }
            }
            _ => {}
        }
    }

    /// Poll STDIN for Ctrl+C / Ctrl+\ between TX drain chunks.
    fn poll_stdin_for_ctrl(&mut self) -> bool {
        let mut byte = [0u8; 1];
        loop {
            match syscall::read(self.stdin_handle, &mut byte) {
                Ok(1) => {
                    if let Some(pid) = self.shell_pid {
                        if byte[0] == 0x03 {
                            let _ = syscall::signal(pid, syscall::signal_event::INTERRUPT, 0);
                            if let Some(ring) = &self.shell_ring {
                                ring.push(&byte);
                                ring.notify();
                            }
                            return true;
                        } else if byte[0] == 0x1c {
                            let _ = syscall::signal(pid, syscall::signal_event::SHUTDOWN, 0);
                            return true;
                        } else {
                            if let Some(ring) = &self.shell_ring {
                                ring.push(&byte);
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

    // =========================================================================
    // Shmem TX Drain
    // =========================================================================

    /// Drain TX ring to UART. Accumulates 4KB chunks to avoid splitting ANSI.
    fn handle_shmem_readable(&mut self, _ctx: &mut dyn BusCtx) {
        let mut carry = [0u8; 16];
        let mut carry_len: usize = 0;

        loop {
            let mut acc = [0u8; 4096];
            let mut acc_len: usize = 0;

            // Prepend carried-over bytes
            if carry_len > 0 {
                acc[..carry_len].copy_from_slice(&carry[..carry_len]);
                acc_len = carry_len;
                carry_len = 0;
            }

            // Accumulate from ring
            loop {
                let ring = match self.shell_ring.as_ref() {
                    Some(r) => r,
                    None => return,
                };
                let tx_avail = ring.readable();
                if tx_avail == 0 { break; }
                let space = acc.len() - acc_len;
                if space == 0 { break; }
                let to_read = tx_avail.min(space);
                let n = ring.pull(&mut acc[acc_len..acc_len + to_read]);
                if n == 0 { break; }
                acc_len += n;
                ring.notify();
            }

            if acc_len == 0 { break; }

            // Check for incomplete ANSI escape at end
            let more_data = match self.shell_ring.as_ref() {
                Some(r) => r.readable() > 0 || acc_len == acc.len(),
                None => false,
            };
            if more_data {
                let scan_start = if acc_len > 16 { acc_len - 16 } else { 0 };
                let mut last_esc = None;
                for i in (scan_start..acc_len).rev() {
                    if acc[i] == 0x1b {
                        last_esc = Some(i);
                        break;
                    }
                }
                if let Some(esc_pos) = last_esc {
                    let mut complete = false;
                    for j in (esc_pos + 1)..acc_len {
                        if acc[j] >= b'A' && acc[j] <= b'z' {
                            complete = true;
                            break;
                        }
                    }
                    if !complete {
                        let tail_len = acc_len - esc_pos;
                        if tail_len <= carry.len() {
                            carry[..tail_len].copy_from_slice(&acc[esc_pos..acc_len]);
                            carry_len = tail_len;
                            acc_len = esc_pos;
                            if acc_len == 0 { continue; }
                        }
                    }
                }
            }

            // Poll STDIN for Ctrl+C/Ctrl+\ between chunks
            if self.poll_stdin_for_ctrl() {
                break;
            }

            // Scan for commands (SPLIT, NOSPLIT, GETSIZE)
            let mut data_start = 0;
            while data_start < acc_len && (acc[data_start] == b'\n' || acc[data_start] == b'\r') {
                data_start += 1;
            }

            if data_start > 0 {
                if self.split_enabled {
                    self.write_uart(&acc[..data_start]);
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
                    ring.push(b"OK\n");
                    ring.notify();
                }
            } else if cmd_len >= 7 && &cmd_buf[..7] == b"NOSPLIT" {
                self.disable_split();
                if let Some(ring) = &self.shell_ring {
                    ring.push(b"OK\n");
                    ring.notify();
                }
            } else if cmd_len >= 7 && &cmd_buf[..7] == b"GETSIZE" {
                if self.split_enabled {
                    self.write_uart(b"\x1b[r");
                }

                if let Some((cols, rows)) = ansi::query_screen_size() {
                    self.cols = cols;
                    self.rows = rows;
                }

                if self.split_enabled {
                    self.split_enabled = false;
                    self.enable_split();
                }

                let mut msg = [0u8; 32];
                let len = format_size_msg(&mut msg, self.cols, self.rows);
                if let Some(ring) = &self.shell_ring {
                    ring.push(&msg[..len]);
                    ring.notify();
                }
            } else if cmd_len > 0 {
                if self.split_enabled {
                    self.write_uart(cmd_buf);
                } else {
                    self.write_uart(cmd_buf);
                }
            }
        }
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for ConsoledDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        unotice!("consoled", "init";);

        // Claim the kernel UART bus
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

        // Drain stale input
        let mut buf = [0u8; 64];
        loop {
            match syscall::read(self.stdin_handle, &mut buf) {
                Ok(n) if n > 0 => {}
                _ => break,
            }
        }

        // Detect terminal size
        if let Some((cols, rows)) = ansi::query_screen_size() {
            self.cols = cols;
            self.rows = rows;
            unotice!("consoled", "terminal_size"; cols = cols, rows = rows);
        }

        // Watch stdin for events
        if let Err(e) = ctx.watch_handle(self.stdin_handle, TAG_STDIN) {
            uerror!("consoled", "watch_stdin_failed";);
            return Err(e);
        }

        // Register console port with devd (for port tree visibility, no spawn rule)
        let mut info = PortInfo::empty();
        info.set_name(b"console:0");
        info.port_class = PortClass::Console;
        info.port_subclass = port_subclass::CONSOLE_SERIAL;

        if let Err(e) = ctx.register_port_with_info(&info, 0) {
            uerror!("consoled", "register_port_failed";);
            return Err(e);
        }

        // Spawn shell directly as child
        self.spawn_shell(ctx);

        unotice!("consoled", "ready";);
        Ok(())
    }

    fn command(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) -> Disposition {
        Disposition::Handled
    }

    fn signal(&mut self, signal_event: u16, signal_value: u64, ctx: &mut dyn BusCtx) {
        if signal_event as u32 & libsys::syscall::signal_event::CHILD_EXIT != 0 {
            let child_pid = (signal_value >> 32) as u32;
            if self.shell_pid == Some(child_pid) {
                self.handle_child_exit(ctx);
            }
        }
    }

    fn handle_event(&mut self, tag: u32, _handle: Handle, ctx: &mut dyn BusCtx) {
        match tag {
            TAG_STDIN => self.handle_stdin_readable(ctx),
            TAG_SHMEM => self.handle_shmem_readable(ctx),
            TAG_SUPERQ => self.handle_child_exit(ctx),
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

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: ConsoledDriver = ConsoledDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"consoled", driver);
}
