//! Remote Shell — minimal command interpreter over TCP port 23.
//!
//! Provides a subset of shell builtins using available syscalls:
//!   help, ps, uptime, ls, kill, net, exit
//!
//! Single connection at a time. Line-buffered input.

use smoltcp::iface::SocketSet;
use smoltcp::socket::tcp;
use smoltcp::iface::SocketHandle;

use userlib::syscall::{self, ProcessInfo};

/// Remote shell state.
#[derive(Clone, Copy, PartialEq, Eq)]
enum RshState {
    /// Listening for a new connection.
    Idle,
    /// Connected, waiting for input.
    Connected,
}

/// Remote shell command interpreter.
pub struct RemoteShell {
    state: RshState,
    line_buf: [u8; 256],
    line_len: usize,
    /// Current IP address (updated by ipd when IP state changes).
    pub ip: [u8; 4],
    /// IP source: 0=none, 1=dhcp, 2=static
    pub ip_source: u8,
}

impl RemoteShell {
    pub const fn new() -> Self {
        Self {
            state: RshState::Idle,
            line_buf: [0u8; 256],
            line_len: 0,
            ip: [0; 4],
            ip_source: 0,
        }
    }

    /// Process the remote shell TCP socket. Called every poll cycle.
    pub fn process(&mut self, sockets: &mut SocketSet<'static>, handle: SocketHandle) {
        let sock = sockets.get_mut::<tcp::Socket>(handle);

        // Re-listen after disconnect
        if !sock.is_active() && !sock.is_listening() {
            self.state = RshState::Idle;
            self.line_len = 0;
            sock.listen(23).ok();
            return;
        }

        // Detect new connection
        if self.state == RshState::Idle && sock.may_send() && sock.may_recv() {
            self.state = RshState::Connected;
            self.line_len = 0;
            // Send banner + prompt
            let banner = b"BPI-R4 remote shell\r\nType 'help' for commands.\r\n$ ";
            let _ = sock.send_slice(banner);
            return;
        }

        if self.state != RshState::Connected {
            return;
        }

        // Connection lost
        if !sock.may_recv() && !sock.may_send() {
            self.state = RshState::Idle;
            self.line_len = 0;
            return;
        }

        // Read available data
        if !sock.may_recv() {
            return;
        }

        // Read bytes one at a time for line editing
        let mut process_line = false;
        sock.recv(|data| {
            for i in 0..data.len() {
                let b = data[i];
                match b {
                    b'\r' | b'\n' => {
                        process_line = true;
                    }
                    // Backspace / DEL
                    0x08 | 0x7F => {
                        if self.line_len > 0 {
                            self.line_len -= 1;
                        }
                    }
                    // Ignore control chars
                    0..=0x1F => {}
                    _ => {
                        if self.line_len < self.line_buf.len() {
                            self.line_buf[self.line_len] = b;
                            self.line_len += 1;
                        }
                    }
                }
            }
            (data.len(), ())
        });

        if process_line {
            let cmd_len = self.line_len;
            self.line_len = 0;

            let mut resp = [0u8; 2048];
            let resp_len = self.dispatch(&self.line_buf[..cmd_len], &mut resp);

            if resp_len > 0 {
                let _ = sock.send_slice(&resp[..resp_len]);
            }

            // Check if exit was requested
            if cmd_len >= 4 && &self.line_buf[..4] == b"exit" {
                sock.close();
                self.state = RshState::Idle;
                return;
            }

            // Send prompt
            let _ = sock.send_slice(b"$ ");
        }
    }

    /// Dispatch a command line to the appropriate handler.
    fn dispatch(&self, line: &[u8], out: &mut [u8]) -> usize {
        let cmd = trim(line);
        if cmd.is_empty() {
            return 0;
        }

        // Split command and arguments at first space
        let (verb, args) = split_first_word(cmd);

        match verb {
            b"help" | b"?" => self.cmd_help(out),
            b"ps" => self.cmd_ps(out),
            b"uptime" => self.cmd_uptime(out),
            b"ls" => self.cmd_ls(out),
            b"kill" => self.cmd_kill(args, out),
            b"net" => self.cmd_net(out),
            b"exit" | b"quit" => {
                copy_str(out, "Goodbye.\r\n")
            }
            _ => {
                let mut pos = 0;
                pos += copy_str(&mut out[pos..], "Unknown command: ");
                pos += copy_bytes(&mut out[pos..], verb);
                pos += copy_str(&mut out[pos..], "\r\nType 'help' for commands.\r\n");
                pos
            }
        }
    }

    fn cmd_help(&self, out: &mut [u8]) -> usize {
        copy_str(out, concat!(
            "Commands:\r\n",
            "  help     Show this help\r\n",
            "  ps       Process list\r\n",
            "  uptime   System uptime\r\n",
            "  ls       List ramfs files\r\n",
            "  kill N   Kill process by PID\r\n",
            "  net      Network info\r\n",
            "  exit     Close connection\r\n",
        ))
    }

    fn cmd_ps(&self, out: &mut [u8]) -> usize {
        let mut procs = [ProcessInfo::empty(); 32];
        let count = syscall::ps_info(&mut procs);

        let mut pos = 0;
        pos += copy_str(&mut out[pos..], "PID  PPID  CPU  STATE     NAME\r\n");

        for i in 0..count {
            let p = &procs[i];
            // PID (right-aligned in 3 chars)
            pos += fmt_u32_pad(&mut out[pos..], p.pid, 3);
            pos += copy_str(&mut out[pos..], "  ");
            // PPID
            pos += fmt_u32_pad(&mut out[pos..], p.ppid, 4);
            pos += copy_str(&mut out[pos..], "  ");
            // CPU
            pos += fmt_u32_pad(&mut out[pos..], p.cpu as u32, 3);
            pos += copy_str(&mut out[pos..], "  ");
            // State
            let state = p.state_str();
            pos += copy_str(&mut out[pos..], state);
            // Pad to 10 chars
            for _ in state.len()..10 {
                if pos < out.len() {
                    out[pos] = b' ';
                    pos += 1;
                }
            }
            // Name
            let name = name_str(&p.name);
            pos += copy_bytes(&mut out[pos..], name);
            pos += copy_str(&mut out[pos..], "\r\n");

            if pos > out.len() - 64 {
                break;
            }
        }
        pos
    }

    fn cmd_uptime(&self, out: &mut [u8]) -> usize {
        let ns = syscall::gettime();
        let secs = ns / 1_000_000_000;
        let mins = secs / 60;
        let hours = mins / 60;

        let mut pos = 0;
        pos += copy_str(&mut out[pos..], "Up ");
        if hours > 0 {
            pos += fmt_u64(&mut out[pos..], hours);
            pos += copy_str(&mut out[pos..], "h ");
        }
        if mins > 0 || hours > 0 {
            pos += fmt_u64(&mut out[pos..], mins % 60);
            pos += copy_str(&mut out[pos..], "m ");
        }
        pos += fmt_u64(&mut out[pos..], secs % 60);
        pos += copy_str(&mut out[pos..], "s\r\n");
        pos
    }

    fn cmd_ls(&self, out: &mut [u8]) -> usize {
        let mut entries = [syscall::RamfsListEntry::empty(); 32];
        let count = syscall::ramfs_list(&mut entries);
        if count < 0 {
            return copy_str(out, "ramfs_list failed\r\n");
        }

        let mut pos = 0;
        for i in 0..count as usize {
            let e = &entries[i];
            let name = e.name_str();
            pos += copy_bytes(&mut out[pos..], name);
            // Pad to 20 chars
            for _ in name.len()..20 {
                if pos < out.len() {
                    out[pos] = b' ';
                    pos += 1;
                }
            }
            pos += fmt_u64(&mut out[pos..], e.size);
            pos += copy_str(&mut out[pos..], "\r\n");
            if pos > out.len() - 128 {
                break;
            }
        }
        if count == 0 {
            pos += copy_str(&mut out[pos..], "(empty)\r\n");
        }
        pos
    }

    fn cmd_kill(&self, args: &[u8], out: &mut [u8]) -> usize {
        let args = trim(args);
        if args.is_empty() {
            return copy_str(out, "Usage: kill <pid>\r\n");
        }

        let pid = parse_u32(args);
        if pid == 0 {
            return copy_str(out, "Invalid PID\r\n");
        }

        let ret = syscall::kill(pid);
        if ret == 0 {
            let mut pos = 0;
            pos += copy_str(&mut out[pos..], "Killed PID ");
            pos += fmt_u64(&mut out[pos..], pid as u64);
            pos += copy_str(&mut out[pos..], "\r\n");
            pos
        } else {
            let mut pos = 0;
            pos += copy_str(&mut out[pos..], "kill failed: ");
            pos += fmt_i64(&mut out[pos..], ret);
            pos += copy_str(&mut out[pos..], "\r\n");
            pos
        }
    }

    fn cmd_net(&self, out: &mut [u8]) -> usize {
        let ns = syscall::gettime();
        let secs = ns / 1_000_000_000;

        let mut pos = 0;
        pos += copy_str(&mut out[pos..], "IP:      ");
        pos += fmt_ip(&mut out[pos..], self.ip);
        pos += copy_str(&mut out[pos..], " (");
        pos += copy_str(&mut out[pos..], match self.ip_source {
            1 => "dhcp",
            2 => "static",
            _ => "none",
        });
        pos += copy_str(&mut out[pos..], ")\r\n");
        pos += copy_str(&mut out[pos..], "Stack:   smoltcp 0.12\r\n");
        pos += copy_str(&mut out[pos..], "Sockets: UDP:69 TCP:80,23\r\n");
        pos += copy_str(&mut out[pos..], "Uptime:  ");
        pos += fmt_u64(&mut out[pos..], secs);
        pos += copy_str(&mut out[pos..], "s\r\n");
        pos
    }
}

// =============================================================================
// Helpers
// =============================================================================

fn trim(s: &[u8]) -> &[u8] {
    let start = s.iter().position(|&b| b != b' ' && b != b'\t').unwrap_or(s.len());
    let end = s.iter().rposition(|&b| b != b' ' && b != b'\t').map(|p| p + 1).unwrap_or(start);
    &s[start..end]
}

fn split_first_word(s: &[u8]) -> (&[u8], &[u8]) {
    if let Some(pos) = s.iter().position(|&b| b == b' ' || b == b'\t') {
        (&s[..pos], &s[pos + 1..])
    } else {
        (s, &[])
    }
}

fn copy_str(dst: &mut [u8], s: &str) -> usize {
    let len = s.len().min(dst.len());
    dst[..len].copy_from_slice(&s.as_bytes()[..len]);
    len
}

fn copy_bytes(dst: &mut [u8], s: &[u8]) -> usize {
    let len = s.len().min(dst.len());
    dst[..len].copy_from_slice(&s[..len]);
    len
}

fn fmt_u64(buf: &mut [u8], val: u64) -> usize {
    if val == 0 {
        if !buf.is_empty() { buf[0] = b'0'; }
        return 1;
    }
    let mut tmp = [0u8; 20];
    let mut v = val;
    let mut i = 0;
    while v > 0 {
        tmp[i] = b'0' + (v % 10) as u8;
        v /= 10;
        i += 1;
    }
    let len = i.min(buf.len());
    for j in 0..len {
        buf[j] = tmp[i - 1 - j];
    }
    len
}

fn fmt_i64(buf: &mut [u8], val: i64) -> usize {
    if val < 0 {
        if buf.is_empty() { return 0; }
        buf[0] = b'-';
        1 + fmt_u64(&mut buf[1..], (-(val as i128)) as u64)
    } else {
        fmt_u64(buf, val as u64)
    }
}

fn fmt_u32_pad(buf: &mut [u8], val: u32, width: usize) -> usize {
    let mut tmp = [0u8; 10];
    let digits = fmt_u64(&mut tmp, val as u64);
    let mut pos = 0;
    // Pad with spaces
    if digits < width {
        for _ in 0..(width - digits) {
            if pos < buf.len() {
                buf[pos] = b' ';
                pos += 1;
            }
        }
    }
    let copy_len = digits.min(buf.len() - pos);
    buf[pos..pos + copy_len].copy_from_slice(&tmp[..copy_len]);
    pos + copy_len
}

fn fmt_ip(buf: &mut [u8], ip: [u8; 4]) -> usize {
    let mut pos = 0;
    for i in 0..4 {
        if i > 0 {
            pos += copy_str(&mut buf[pos..], ".");
        }
        pos += fmt_u64(&mut buf[pos..], ip[i] as u64);
    }
    pos
}

fn parse_u32(s: &[u8]) -> u32 {
    let mut val: u32 = 0;
    for &b in s {
        if b >= b'0' && b <= b'9' {
            val = val.wrapping_mul(10).wrapping_add((b - b'0') as u32);
        } else {
            break;
        }
    }
    val
}

fn name_str(name: &[u8; 16]) -> &[u8] {
    let len = name.iter().position(|&b| b == 0).unwrap_or(16);
    &name[..len]
}
