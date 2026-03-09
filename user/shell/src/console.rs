//! Console I/O via shared pipe
//!
//! Shell reads its SharedPipe shmem_id from the mailbox page provided by
//! its parent (consoled or sshd) via exec_with_mailbox.
//!
//! ## Mailbox format (offset 64+)
//!
//! | Offset | Size | Field                           |
//! |--------|------|---------------------------------|
//! | 64     | 4    | console_shmem_id (u32 LE)       |
//! | 68     | 2    | cols (u16 LE)                   |
//! | 70     | 2    | rows (u16 LE)                   |

use libf::sync::SharedPipe;
use libf::time::{Duration, Instant};
use libsys::syscall;
use libsys::Handle;

/// Console I/O state
pub struct Console {
    /// Bidirectional pipe for I/O with parent (consoled or sshd)
    pipe: Option<SharedPipe>,
    /// Terminal dimensions
    pub cols: u16,
    pub rows: u16,
    /// Log split enabled
    pub log_split: bool,
    /// True when connected via SSH (enables data path diagnostics).
    /// Set from mailbox flags byte (offset 72, bit 0).
    pub ssh_mode: bool,
}

impl Console {
    pub const fn new() -> Self {
        Self {
            pipe: None,
            cols: 80,
            rows: 24,
            log_split: false,
            ssh_mode: false,
        }
    }

    /// Connect by reading shmem_id from Handle::MAILBOX.
    /// Returns true if connected.
    pub fn connect(&mut self) -> bool {
        let addr = match syscall::map(Handle::MAILBOX, 0) {
            Ok(a) if a != 0 => a,
            _ => return false,
        };

        // Read console fields at offset 64
        let page = addr as *const u8;
        let shmem_id = unsafe {
            let mut buf = [0u8; 4];
            core::ptr::copy_nonoverlapping(page.add(64), buf.as_mut_ptr(), 4);
            u32::from_le_bytes(buf)
        };
        let cols = unsafe {
            let mut buf = [0u8; 2];
            core::ptr::copy_nonoverlapping(page.add(68), buf.as_mut_ptr(), 2);
            u16::from_le_bytes(buf)
        };
        let rows = unsafe {
            let mut buf = [0u8; 2];
            core::ptr::copy_nonoverlapping(page.add(70), buf.as_mut_ptr(), 2);
            u16::from_le_bytes(buf)
        };
        let flags = unsafe { *page.add(72) };

        if shmem_id == 0 {
            return false;
        }

        // Retry shmem map — on SMP the parent may not have called allow()
        // yet when we start running on another core.
        let pipe = {
            let mut p = None;
            for _ in 0..50 {
                if let Some(pipe) = SharedPipe::open(shmem_id) {
                    p = Some(pipe);
                    break;
                }
                libf::time::sleep(Duration::from_millis(1));
            }
            match p {
                Some(pipe) => pipe,
                None => return false,
            }
        };

        self.pipe = Some(pipe);
        if cols >= 10 && cols <= 500 {
            self.cols = cols;
        }
        if rows >= 10 && rows <= 500 {
            self.rows = rows;
        }
        self.ssh_mode = (flags & 0x01) != 0;

        true
    }

    /// Check if connected
    pub fn is_connected(&self) -> bool {
        self.pipe.is_some()
    }

    /// Bytes available in the outgoing pipe (for diagnostic delta).
    pub fn pipe_writable(&self) -> usize {
        self.pipe.as_ref().map(|p| p.writable()).unwrap_or(0)
    }

    /// Read a single byte (blocking)
    pub fn read_byte(&mut self) -> Option<u8> {
        let pipe = self.pipe.as_ref()?;
        let mut byte = [0u8; 1];

        // Fast path: data already available
        if pipe.pull(&mut byte) > 0 {
            return Some(byte[0]);
        }

        // Blocking wait with safety-net timeout against missed wakes
        loop {
            pipe.wait(5000);
            if pipe.pull(&mut byte) > 0 {
                return Some(byte[0]);
            }
        }
    }

    /// Read a single byte with timeout.
    /// Returns `None` if no data arrives within `timeout_ms` milliseconds.
    pub fn read_byte_timeout(&mut self, timeout_ms: u32) -> Option<u8> {
        let pipe = self.pipe.as_ref()?;
        let mut byte = [0u8; 1];

        // Fast path: data already available
        if pipe.pull(&mut byte) > 0 {
            return Some(byte[0]);
        }

        // Wait with deadline — shmem wakes on any notify (including our own TX),
        // so we must loop until the deadline actually expires.
        let deadline = Instant::now() + Duration::from_millis(timeout_ms as u64);
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return None;
            }
            let remaining_ms = remaining.as_millis() as u32;
            pipe.wait(remaining_ms.max(1));

            if pipe.pull(&mut byte) > 0 {
                return Some(byte[0]);
            }
        }
    }

    /// Write bytes to console.
    /// Blocking: writes all data, waiting when the pipe is full.
    pub fn write(&self, data: &[u8]) {
        let pipe = match &self.pipe {
            Some(p) => p,
            None => return,
        };

        let writable_before = pipe.writable();
        if data.len() > writable_before {
            libsys::udebug!("shell", "pipe_write_backpressure"; len = data.len(), writable = writable_before, capacity = pipe.outgoing().capacity());
            libsys::ulog::flush();
        }

        // SharedPipe::push_all handles the write loop with backpressure.
        pipe.push_all(data);

        if data.len() > writable_before {
            libsys::udebug!("shell", "pipe_write_resumed"; len = data.len());
            libsys::ulog::flush();
        }
    }

    /// Write a string to console
    pub fn write_str(&self, s: &str) {
        self.write(s.as_bytes());
    }

    /// Query terminal size from parent transport.
    /// Returns (cols, rows) on success.
    pub fn query_size(&mut self) -> Option<(u16, u16)> {
        let pipe = self.pipe.as_ref()?;

        // Send GETSIZE query to parent (consoled/sshd reads this)
        pipe.push(b"GETSIZE\n");
        pipe.notify();

        // Wait for SIZE response from parent
        for _ in 0..100 {
            if pipe.readable() >= 5 {
                let mut buf = [0u8; 32];
                let n = pipe.pull(&mut buf);
                if n >= 5 && &buf[..5] == b"SIZE " {
                    self.parse_size_msg(&buf[..n]);
                    return Some((self.cols, self.rows));
                }
            }
            if !pipe.wait(10) {
                syscall::sleep_us(10_000);
            }
        }

        Some((self.cols, self.rows))
    }

    /// Parse "SIZE cols rows\n" message
    fn parse_size_msg(&mut self, data: &[u8]) {
        if data.len() < 5 || &data[..5] != b"SIZE " {
            return;
        }

        let rest = &data[5..];
        let mut cols: u16 = 0;
        let mut rows: u16 = 0;
        let mut i = 0;

        while i < rest.len() && rest[i] >= b'0' && rest[i] <= b'9' {
            cols = cols.saturating_mul(10).saturating_add((rest[i] - b'0') as u16);
            i += 1;
        }

        if i < rest.len() && rest[i] == b' ' {
            i += 1;
        }

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
        let pipe = match &self.pipe {
            Some(p) => p,
            None => return,
        };

        if enabled {
            pipe.push(b"SPLIT 5\n");
        } else {
            pipe.push(b"NOSPLIT\n");
        }
        pipe.notify();

        // Wait for OK response
        for _ in 0..50 {
            if pipe.readable() >= 2 {
                let mut buf = [0u8; 16];
                pipe.pull(&mut buf);
                break;
            }
            pipe.wait(10);
        }
        self.log_split = enabled;
    }

    /// Set number of log lines
    pub fn set_log_lines(&mut self, lines: u8) {
        let pipe = match &self.pipe {
            Some(p) => p,
            None => return,
        };

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

        pipe.push(&cmd[..pos]);
        pipe.notify();

        for _ in 0..50 {
            if pipe.readable() >= 2 {
                let mut buf = [0u8; 16];
                pipe.pull(&mut buf);
                break;
            }
            pipe.wait(10);
        }
    }

    /// Connect/disconnect from logd (placeholder)
    pub fn set_logd_connected(&self, _connected: bool) {
        // Future: tell parent transport to connect/disconnect from logd
    }
}

/// Global console instance
static mut CONSOLE: Console = Console::new();

/// Initialize and connect to console via mailbox
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
    write(s.as_bytes());
}

/// Read a byte from console (convenience function)
pub fn read_byte() -> Option<u8> {
    console_mut().read_byte()
}

/// Read a byte with timeout (convenience function)
pub fn read_byte_timeout(timeout_ms: u32) -> Option<u8> {
    console_mut().read_byte_timeout(timeout_ms)
}

/// Writer for core::fmt::Write trait
pub struct ConsoleWriter;

impl core::fmt::Write for ConsoleWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        write(s.as_bytes());
        Ok(())
    }
}
