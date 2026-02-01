//! Remote Shell — TCP port 23 proxy to shell command server.
//!
//! Forwards commands to the real shell process via IPC ("shell-cmd:" port),
//! so all 50+ shell commands work over both console and remote shell.
//!
//! Local-only commands (net, exit) are handled directly since they need
//! ipd-internal state or TCP socket control.
//!
//! Fallback: if shell is not running (connect fails), uses built-in
//! implementations for ps, uptime, ls, kill, help.
//!
//! Single connection at a time. Line-buffered input.

use smoltcp::iface::SocketSet;
use smoltcp::socket::tcp;
use smoltcp::iface::SocketHandle;

use userlib::ipc::Channel;
use userlib::syscall::{self, ProcessInfo, Handle};

/// Remote shell state.
#[derive(Clone, Copy, PartialEq, Eq)]
enum RshState {
    /// Listening for a new connection.
    Idle,
    /// Connected, waiting for input.
    Connected,
}

/// Pending IPC command state.
enum RshPending {
    /// No pending command.
    None,
    /// Waiting for shell to respond on the channel.
    WaitingResponse {
        channel: Channel,
        resp_buf: [u8; 4096],
        resp_len: usize,
    },
}

/// Remote shell command interpreter.
pub struct RemoteShell {
    state: RshState,
    line_buf: [u8; 256],
    line_len: usize,
    pending: RshPending,
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
            pending: RshPending::None,
            ip: [0; 4],
            ip_source: 0,
        }
    }

    /// Get the IPC channel handle if we're waiting for a response.
    /// Used by ipd to register with the bus framework for wake events.
    pub fn pending_handle(&self) -> Option<Handle> {
        match &self.pending {
            RshPending::WaitingResponse { channel, .. } => Some(channel.handle()),
            RshPending::None => None,
        }
    }

    /// Process the remote shell TCP socket. Called every poll cycle.
    pub fn process(&mut self, sockets: &mut SocketSet<'static>, handle: SocketHandle) {
        let sock = sockets.get_mut::<tcp::Socket>(handle);

        // Re-listen after disconnect
        if !sock.is_active() && !sock.is_listening() {
            self.state = RshState::Idle;
            self.line_len = 0;
            self.pending = RshPending::None;
            sock.listen(23).ok();
            return;
        }

        // Detect new connection
        if self.state == RshState::Idle && sock.may_send() && sock.may_recv() {
            self.state = RshState::Connected;
            self.line_len = 0;
            self.pending = RshPending::None;
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
            self.pending = RshPending::None;
            return;
        }

        // Check for pending IPC response
        if self.poll_pending_response(sock) {
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

            // Make a copy of the command for dispatch (needed because self is borrowed)
            let mut cmd_copy = [0u8; 256];
            cmd_copy[..cmd_len].copy_from_slice(&self.line_buf[..cmd_len]);
            let cmd = trim(&cmd_copy[..cmd_len]);

            if cmd.is_empty() {
                let _ = sock.send_slice(b"$ ");
                return;
            }

            let (verb, _args) = split_first_word(cmd);

            // Local commands (need ipd state or TCP socket control)
            match verb {
                b"exit" | b"quit" => {
                    let _ = sock.send_slice(b"Goodbye.\r\n");
                    sock.close();
                    self.state = RshState::Idle;
                    return;
                }
                b"net" => {
                    let mut resp = [0u8; 512];
                    let resp_len = self.cmd_net(&mut resp);
                    let _ = sock.send_slice(&resp[..resp_len]);
                    let _ = sock.send_slice(b"$ ");
                    return;
                }
                _ => {}
            }

            // Try to forward to shell via IPC
            if self.forward_to_shell(cmd) {
                // Command sent to shell, response will arrive later
                return;
            }

            // Fallback: shell not running, use local implementations
            let mut resp = [0u8; 2048];
            let resp_len = self.dispatch_local(cmd, &mut resp);
            if resp_len > 0 {
                let _ = sock.send_slice(&resp[..resp_len]);
            }
            let _ = sock.send_slice(b"$ ");
        }
    }

    /// Try to forward a command to the shell process via IPC.
    /// Returns true if the command was sent (response pending).
    fn forward_to_shell(&mut self, cmd: &[u8]) -> bool {
        let mut channel = match Channel::connect(b"shell-cmd:") {
            Ok(ch) => ch,
            Err(_) => return false,
        };

        if channel.send(cmd).is_err() {
            return false;
        }

        self.pending = RshPending::WaitingResponse {
            channel,
            resp_buf: [0u8; 4096],
            resp_len: 0,
        };
        true
    }

    /// Poll the pending IPC channel for a response.
    /// Returns true if we're still waiting (caller should not process more input).
    fn poll_pending_response(&mut self, sock: &mut tcp::Socket<'_>) -> bool {
        let (done, data_to_send) = match &mut self.pending {
            RshPending::None => return false,
            RshPending::WaitingResponse { channel, resp_buf, resp_len } => {
                // Try to receive data from shell
                let mut tmp = [0u8; 512];
                loop {
                    match channel.try_recv(&mut tmp) {
                        Ok(Some(n)) => {
                            // Append to response buffer
                            let available = resp_buf.len() - *resp_len;
                            let to_copy = n.min(available);
                            resp_buf[*resp_len..*resp_len + to_copy]
                                .copy_from_slice(&tmp[..to_copy]);
                            *resp_len += to_copy;
                            // Keep reading - more data may be available
                        }
                        Ok(None) => {
                            // No data yet, still waiting
                            return true;
                        }
                        Err(_) => {
                            // Channel closed (ConnectionReset) = response complete
                            let len = *resp_len;
                            // Need to extract data before resetting pending
                            break (true, Some(len));
                        }
                    }
                }
            }
        };

        if done {
            if let Some(len) = data_to_send {
                // Extract response data before resetting pending state
                if let RshPending::WaitingResponse { resp_buf, .. } = &self.pending {
                    // Convert \n to \r\n for telnet clients
                    let mut converted = [0u8; 6144];
                    let mut cpos = 0;
                    for i in 0..len {
                        if resp_buf[i] == b'\n' && (i == 0 || resp_buf[i - 1] != b'\r') {
                            if cpos < converted.len() {
                                converted[cpos] = b'\r';
                                cpos += 1;
                            }
                        }
                        if cpos < converted.len() {
                            converted[cpos] = resp_buf[i];
                            cpos += 1;
                        }
                    }
                    let _ = sock.send_slice(&converted[..cpos]);
                }
            }
            self.pending = RshPending::None;
            let _ = sock.send_slice(b"$ ");
        }

        done
    }

    /// Dispatch to local fallback commands (when shell is not running).
    fn dispatch_local(&self, line: &[u8], out: &mut [u8]) -> usize {
        let cmd = trim(line);
        if cmd.is_empty() {
            return 0;
        }

        let (verb, args) = split_first_word(cmd);

        match verb {
            b"help" | b"?" => self.cmd_help_local(out),
            b"ps" => self.cmd_ps(out),
            b"uptime" => self.cmd_uptime(out),
            b"ls" => self.cmd_ls(out),
            b"kill" => self.cmd_kill(args, out),
            b"net" => self.cmd_net(out),
            _ => {
                let mut pos = 0;
                pos += copy_str(&mut out[pos..], "Shell not running. Limited commands available.\r\n");
                pos += copy_str(&mut out[pos..], "Type 'help' for available commands.\r\n");
                pos
            }
        }
    }

    fn cmd_help_local(&self, out: &mut [u8]) -> usize {
        copy_str(out, concat!(
            "Limited mode (shell not running):\r\n",
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
            pos += fmt_u32_pad(&mut out[pos..], p.pid, 3);
            pos += copy_str(&mut out[pos..], "  ");
            pos += fmt_u32_pad(&mut out[pos..], p.ppid, 4);
            pos += copy_str(&mut out[pos..], "  ");
            pos += fmt_u32_pad(&mut out[pos..], p.cpu as u32, 3);
            pos += copy_str(&mut out[pos..], "  ");
            let state = p.state_str();
            pos += copy_str(&mut out[pos..], state);
            for _ in state.len()..10 {
                if pos < out.len() {
                    out[pos] = b' ';
                    pos += 1;
                }
            }
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
