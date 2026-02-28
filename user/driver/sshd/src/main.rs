//! sshd — Minimal SSH-2 Server
//!
//! Bus framework driver that provides remote shell access over SSH.
//!
//! Architecture:
//!   1. Connects to ipd's "tcp:" socket service to listen on port 22
//!   2. Accepts TCP connections → gets DataPort for each
//!   3. Runs SSH-2 protocol (KEX, auth, channel) over the DataPort
//!   4. Bridges authenticated sessions to shell via "shell-cmd:" port
//!
//! Cipher suite (modern minimum):
//!   - Key exchange: curve25519-sha256
//!   - Host key: ssh-ed25519
//!   - Encryption: chacha20-poly1305@openssh.com
//!   - Auth: password (v1)

#![no_std]
#![no_main]

extern crate alloc;

mod ssh;

use alloc::vec::Vec;

use libf::crypto;
use libf::io::{Read, Write};
use libf::net::{TcpListener, TcpStream, SocketAddr, Ipv4Addr};

use userlib::syscall;
use userlib::{uinfo, udebug, uerror};

// =============================================================================
// Constants
// =============================================================================

const SSH_PORT: u16 = 22;
/// Hardcoded host key seed for development.
/// In production, this would be generated on first boot and stored.
const DEV_HOST_KEY_SEED: [u8; 32] = [
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];

/// Default password for development (plaintext comparison).
const DEV_PASSWORD: &[u8] = b"filament";

// =============================================================================
// SSH Session State Machine
// =============================================================================

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum SshState {
    /// Waiting for client version string.
    ProtocolExchange,
    /// Sent/received KEXINIT, waiting for KEX_ECDH_INIT.
    AlgorithmNegotiation,
    /// Processing key exchange.
    KeyExchange,
    /// Sent NEWKEYS, waiting for client NEWKEYS.
    NewKeys,
    /// Waiting for service request (ssh-userauth).
    ServiceRequest,
    /// Waiting for authentication.
    Authentication,
    /// Waiting for channel open.
    ChannelOpen,
    /// Shell session active, forwarding data.
    ShellRunning,
    /// Connection closing.
    Closing,
}

/// Per-session SSH state.
struct SshSession {
    state: SshState,
    stream: TcpStream,

    // Protocol exchange
    client_version: [u8; 256],
    client_version_len: usize,

    // Key exchange state
    server_kexinit_payload: Vec<u8>,
    client_kexinit_payload: Vec<u8>,
    server_ephemeral_secret: [u8; 32],
    server_ephemeral_public: [u8; 32],
    session_id: [u8; 32],
    session_id_set: bool,

    // Host key
    host_key_seed: [u8; 32],
    host_key_public: [u8; 32],

    // Encryption state
    tx_transport: Option<ssh::EncryptedTransport>,
    rx_transport: Option<ssh::EncryptedTransport>,
    encrypted: bool,

    // Channel state
    remote_channel_id: u32,
    local_channel_id: u32,
    remote_window: u32,

    // Sequence numbers (count all binary packets, not version exchange)
    tx_seqno: u64,
    rx_seqno: u64,

    // Read buffer for accumulating TCP data
    read_buf: Vec<u8>,
}

impl SshSession {
    fn new(stream: TcpStream, host_key_seed: [u8; 32]) -> Self {
        let host_key_public = crypto::ed25519_public_key(&host_key_seed);
        Self {
            state: SshState::ProtocolExchange,
            stream,
            client_version: [0u8; 256],
            client_version_len: 0,
            server_kexinit_payload: Vec::new(),
            client_kexinit_payload: Vec::new(),
            server_ephemeral_secret: [0u8; 32],
            server_ephemeral_public: [0u8; 32],
            session_id: [0u8; 32],
            session_id_set: false,
            host_key_seed,
            host_key_public,
            tx_transport: None,
            rx_transport: None,
            encrypted: false,
            remote_channel_id: 0,
            local_channel_id: 0,
            remote_window: 0,
            tx_seqno: 0,
            rx_seqno: 0,
            read_buf: Vec::new(),
        }
    }

    /// Run the SSH session to completion.
    fn run(&mut self) {
        // Send server version string
        let mut version_line = [0u8; 64];
        let vlen = ssh::SSH_VERSION.len();
        version_line[..vlen].copy_from_slice(ssh::SSH_VERSION);
        version_line[vlen] = b'\r';
        version_line[vlen + 1] = b'\n';
        if self.stream.write_all(&version_line[..vlen + 2]).is_err() {
            return;
        }

        // Main session loop
        loop {
            match self.state {
                SshState::ProtocolExchange => {
                    if !self.do_protocol_exchange() {
                        return;
                    }
                }
                SshState::AlgorithmNegotiation => {
                    if !self.do_algorithm_negotiation() {
                        return;
                    }
                }
                SshState::KeyExchange => {
                    if !self.do_key_exchange() {
                        return;
                    }
                }
                SshState::NewKeys => {
                    if !self.do_new_keys() {
                        return;
                    }
                }
                SshState::ServiceRequest => {
                    if !self.do_service_request() {
                        return;
                    }
                }
                SshState::Authentication => {
                    if !self.do_authentication() {
                        return;
                    }
                }
                SshState::ChannelOpen => {
                    if !self.do_channel_open() {
                        return;
                    }
                }
                SshState::ShellRunning => {
                    if !self.do_shell_running() {
                        return;
                    }
                }
                SshState::Closing => {
                    return;
                }
            }
        }
    }

    // =========================================================================
    // Read helpers
    // =========================================================================

    /// Read a line from the TCP stream (for version exchange).
    fn read_line(&mut self) -> Option<Vec<u8>> {
        let mut buf = [0u8; 1];
        let mut line = Vec::new();

        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => return None, // EOF
                Ok(1) => {
                    if buf[0] == b'\n' {
                        // Strip trailing \r if present
                        if line.last() == Some(&b'\r') {
                            line.pop();
                        }
                        return Some(line);
                    }
                    line.push(buf[0]);
                    if line.len() > 255 {
                        return None; // Line too long
                    }
                }
                _ => return None,
            }
        }
    }

    /// Read an unencrypted SSH packet.
    fn read_packet(&mut self) -> Option<Vec<u8>> {
        // Read 4-byte length
        let mut len_buf = [0u8; 4];
        if self.stream.read_exact(&mut len_buf).is_err() {
            return None;
        }
        let packet_length = u32::from_be_bytes(len_buf) as usize;
        if packet_length < 1 || packet_length > 35000 {
            return None;
        }

        // Read remaining data
        let mut data = alloc::vec![0u8; packet_length];
        if self.stream.read_exact(&mut data).is_err() {
            return None;
        }

        self.rx_seqno += 1;

        // Parse: padding_length(1) + payload + padding
        let padding_length = data[0] as usize;
        if padding_length + 1 > packet_length {
            return None;
        }
        let payload_length = packet_length - 1 - padding_length;
        Some(data[1..1 + payload_length].to_vec())
    }

    /// Read an encrypted SSH packet.
    fn read_encrypted_packet(&mut self) -> Option<Vec<u8>> {
        let rx = self.rx_transport.as_mut()?;

        // Step 1: Read 4 encrypted bytes (packet_length)
        let mut enc_len = [0u8; 4];
        if self.stream.read_exact(&mut enc_len).is_err() {
            return None;
        }

        // Step 2: Decrypt length to know how much more to read
        let packet_length = rx.decrypt_length(&enc_len) as usize;
        if packet_length < 1 || packet_length > 35000 {
            return None;
        }

        // Step 3: Read encrypted payload + 16-byte Poly1305 tag
        let mut data = alloc::vec![0u8; packet_length + 16];
        if self.stream.read_exact(&mut data).is_err() {
            return None;
        }

        // Step 4: Decrypt and verify MAC
        rx.decrypt_packet(&enc_len, &data)
    }

    /// Read a packet (encrypted or plaintext depending on state).
    fn recv_packet(&mut self) -> Option<Vec<u8>> {
        if self.encrypted {
            self.read_encrypted_packet()
        } else {
            self.read_packet()
        }
    }

    /// Send a packet (encrypted or plaintext depending on state).
    fn send_packet(&mut self, payload: &[u8]) -> bool {
        if self.encrypted {
            if let Some(tx) = &mut self.tx_transport {
                let mut out = [0u8; 8192];
                let n = tx.encrypt_packet(payload, &mut out);
                if n == 0 {
                    return false;
                }
                self.stream.write_all(&out[..n]).is_ok()
            } else {
                false
            }
        } else {
            let mut out = [0u8; 8192];
            let n = ssh::frame_packet(payload, &mut out);
            if n == 0 {
                return false;
            }
            self.tx_seqno += 1;
            self.stream.write_all(&out[..n]).is_ok()
        }
    }

    // =========================================================================
    // Protocol phases
    // =========================================================================

    fn do_protocol_exchange(&mut self) -> bool {
        // Read client version string
        let line = match self.read_line() {
            Some(l) => l,
            None => return false,
        };

        if !line.starts_with(b"SSH-2.0-") {
            return false;
        }

        let copy_len = line.len().min(self.client_version.len());
        self.client_version[..copy_len].copy_from_slice(&line[..copy_len]);
        self.client_version_len = copy_len;

        udebug!("sshd", "client_version_ok";);

        // Generate ephemeral key for this session
        // Use a simple counter-based seed (in production, use hardware RNG)
        let time = userlib::syscall::gettime();
        let mut seed_material = [0u8; 64];
        seed_material[..8].copy_from_slice(&time.to_le_bytes());
        seed_material[8..40].copy_from_slice(&self.host_key_seed);
        // Mix with SHA-256 to get ephemeral secret
        self.server_ephemeral_secret = crypto::sha256(&seed_material);
        self.server_ephemeral_public = crypto::x25519_public(&self.server_ephemeral_secret);

        // Send KEXINIT
        let cookie = crypto::sha256(b"sshd_cookie"); // deterministic for now
        let mut kexinit = [0u8; 512];
        let n = ssh::build_kexinit(&mut kexinit, &cookie[..16].try_into().unwrap());
        self.server_kexinit_payload = kexinit[..n].to_vec();

        if !self.send_packet(&kexinit[..n]) {
            return false;
        }

        self.state = SshState::AlgorithmNegotiation;
        true
    }

    fn do_algorithm_negotiation(&mut self) -> bool {
        // Read client KEXINIT
        let payload = match self.recv_packet() {
            Some(p) => p,
            None => return false,
        };

        if payload.is_empty() || payload[0] != ssh::msg::KEXINIT {
            return false;
        }

        self.client_kexinit_payload = payload;
        self.state = SshState::KeyExchange;

        udebug!("sshd", "kexinit_received";);
        true
    }

    fn do_key_exchange(&mut self) -> bool {
        // Read KEX_ECDH_INIT from client
        let payload = match self.recv_packet() {
            Some(p) => p,
            None => return false,
        };

        if payload.is_empty() || payload[0] != ssh::msg::KEX_ECDH_INIT {
            return false;
        }

        // Parse client's ephemeral public key Q_C
        let (q_c_data, _) = match ssh::read_string(&payload[1..]) {
            Some(r) => r,
            None => return false,
        };

        if q_c_data.len() != 32 {
            return false;
        }
        let mut q_c = [0u8; 32];
        q_c.copy_from_slice(q_c_data);

        // Compute shared secret K = X25519(server_secret, client_public)
        let shared_secret = crypto::x25519(&self.server_ephemeral_secret, &q_c);

        // Build host key blob
        let mut k_s = [0u8; 64];
        let k_s_len = ssh::build_host_key_blob(&self.host_key_public, &mut k_s);

        // Compute exchange hash H
        let h = ssh::compute_exchange_hash(
            &self.client_version[..self.client_version_len],
            ssh::SSH_VERSION,
            &self.client_kexinit_payload,
            &self.server_kexinit_payload,
            &k_s[..k_s_len],
            &q_c,
            &self.server_ephemeral_public,
            &shared_secret,
        );

        // Set session ID (first exchange hash)
        if !self.session_id_set {
            self.session_id = h;
            self.session_id_set = true;
        }

        // Sign the exchange hash with host key
        let signature = crypto::ed25519_sign(&self.host_key_seed, &h);

        // Send KEX_ECDH_REPLY
        let mut reply = [0u8; 512];
        let reply_len = ssh::build_kex_reply(
            &k_s[..k_s_len],
            &self.server_ephemeral_public,
            &signature,
            &mut reply,
        );
        if !self.send_packet(&reply[..reply_len]) {
            return false;
        }

        // Send NEWKEYS
        if !self.send_packet(&[ssh::msg::NEWKEYS]) {
            return false;
        }

        // Derive encryption keys (64 bytes each for chacha20-poly1305@openssh.com)
        // Key c2s = HASH(K||H||'C'||sid) || HASH(K||H||K1)  (64 bytes: main + header)
        // Key s2c = HASH(K||H||'D'||sid) || HASH(K||H||K1)  (64 bytes: main + header)
        let key_c2s = ssh::derive_key_64(&shared_secret, &h, b'C', &self.session_id);
        let key_s2c = ssh::derive_key_64(&shared_secret, &h, b'D', &self.session_id);

        // seqno tracks all binary packets sent/received.
        // TX: KEXINIT(0) + KEX_REPLY(1) + NEWKEYS(2) already sent → tx_seqno is 3
        // RX: KEXINIT(0) + KEX_INIT(1) received → rx_seqno is 2, client NEWKEYS
        //     will be received in do_new_keys (incrementing to 3)
        self.tx_transport = Some(ssh::EncryptedTransport::new(
            &key_s2c,
            self.tx_seqno, // NEWKEYS already sent and counted
        ));
        self.rx_transport = Some(ssh::EncryptedTransport::new(
            &key_c2s,
            self.rx_seqno + 1, // +1 for the client's NEWKEYS we'll receive next
        ));

        self.state = SshState::NewKeys;

        udebug!("sshd", "kex_complete";);
        true
    }

    fn do_new_keys(&mut self) -> bool {
        // Read client NEWKEYS
        let payload = match self.recv_packet() {
            Some(p) => p,
            None => return false,
        };

        if payload.is_empty() || payload[0] != ssh::msg::NEWKEYS {
            return false;
        }

        // Switch to encrypted transport
        self.encrypted = true;
        self.state = SshState::ServiceRequest;

        udebug!("sshd", "newkeys_done";);
        true
    }

    fn do_service_request(&mut self) -> bool {
        let payload = match self.recv_packet() {
            Some(p) => p,
            None => return false,
        };

        if payload.is_empty() || payload[0] != ssh::msg::SERVICE_REQUEST {
            return false;
        }

        let (service_name, _) = match ssh::read_string(&payload[1..]) {
            Some(r) => r,
            None => return false,
        };

        if service_name != b"ssh-userauth" {
            return false;
        }

        // Send SERVICE_ACCEPT
        let mut resp = [0u8; 64];
        resp[0] = ssh::msg::SERVICE_ACCEPT;
        let n = 1 + ssh::write_string(&mut resp[1..], b"ssh-userauth");
        if !self.send_packet(&resp[..n]) {
            return false;
        }

        self.state = SshState::Authentication;

        udebug!("sshd", "service_request_ok";);
        true
    }

    fn do_authentication(&mut self) -> bool {
        let payload = match self.recv_packet() {
            Some(p) => p,
            None => return false,
        };

        if payload.is_empty() || payload[0] != ssh::msg::USERAUTH_REQUEST {
            return false;
        }

        let rest = &payload[1..];

        // Parse: user(string) + service(string) + method(string) + ...
        let (username, rest) = match ssh::read_string(rest) {
            Some(r) => r,
            None => return false,
        };
        let (_service, rest) = match ssh::read_string(rest) {
            Some(r) => r,
            None => return false,
        };
        let (method, rest) = match ssh::read_string(rest) {
            Some(r) => r,
            None => return false,
        };

        if method == b"none" {
            // Reject "none" method, tell client what we support
            let mut resp = [0u8; 64];
            resp[0] = ssh::msg::USERAUTH_FAILURE;
            let n = 1 + ssh::write_string(&mut resp[1..], b"password");
            resp[n] = 0; // partial success = false
            if !self.send_packet(&resp[..n + 1]) {
                return false;
            }
            return true; // Stay in Authentication state
        }

        if method != b"password" {
            // Reject unsupported methods
            let mut resp = [0u8; 64];
            resp[0] = ssh::msg::USERAUTH_FAILURE;
            let n = 1 + ssh::write_string(&mut resp[1..], b"password");
            resp[n] = 0;
            if !self.send_packet(&resp[..n + 1]) {
                return false;
            }
            return true;
        }

        // Password method: boolean(change_password) + string(password)
        if rest.is_empty() {
            return false;
        }
        let _change = rest[0]; // false expected
        let (password, _) = match ssh::read_string(&rest[1..]) {
            Some(r) => r,
            None => return false,
        };

        if password == DEV_PASSWORD {
            uinfo!("sshd", "auth_ok"; user_len = username.len() as u32);
            if !self.send_packet(&[ssh::msg::USERAUTH_SUCCESS]) {
                return false;
            }
            self.state = SshState::ChannelOpen;
        } else {
            udebug!("sshd", "auth_failed";);
            let mut resp = [0u8; 64];
            resp[0] = ssh::msg::USERAUTH_FAILURE;
            let n = 1 + ssh::write_string(&mut resp[1..], b"password");
            resp[n] = 0;
            if !self.send_packet(&resp[..n + 1]) {
                return false;
            }
        }

        true
    }

    fn do_channel_open(&mut self) -> bool {
        let payload = match self.recv_packet() {
            Some(p) => p,
            None => return false,
        };

        if payload.is_empty() {
            return false;
        }

        match payload[0] {
            ssh::msg::CHANNEL_OPEN => {
                let rest = &payload[1..];
                let (channel_type, rest) = match ssh::read_string(rest) {
                    Some(r) => r,
                    None => return false,
                };

                if channel_type != b"session" {
                    return false;
                }

                let (sender_channel, rest) = match ssh::read_u32(rest) {
                    Some(r) => r,
                    None => return false,
                };
                let (initial_window, rest) = match ssh::read_u32(rest) {
                    Some(r) => r,
                    None => return false,
                };
                let (_max_packet, _) = match ssh::read_u32(rest) {
                    Some(r) => r,
                    None => return false,
                };

                self.remote_channel_id = sender_channel;
                self.local_channel_id = 0;
                self.remote_window = initial_window;

                // Send CHANNEL_OPEN_CONFIRMATION
                let mut resp = [0u8; 32];
                resp[0] = ssh::msg::CHANNEL_OPEN_CONFIRMATION;
                let mut pos = 1;
                pos += ssh::write_u32(&mut resp[pos..], sender_channel); // recipient channel
                pos += ssh::write_u32(&mut resp[pos..], self.local_channel_id); // sender channel
                pos += ssh::write_u32(&mut resp[pos..], 32768); // initial window
                pos += ssh::write_u32(&mut resp[pos..], 32768); // max packet size
                if !self.send_packet(&resp[..pos]) {
                    return false;
                }

                udebug!("sshd", "channel_opened";);
                // Stay in ChannelOpen to wait for shell/exec request
                true
            }
            ssh::msg::CHANNEL_REQUEST => {
                self.handle_channel_request(&payload)
            }
            ssh::msg::CHANNEL_WINDOW_ADJUST => {
                if payload.len() >= 9 {
                    let (_, rest) = ssh::read_u32(&payload[1..]).unwrap();
                    let (bytes, _) = ssh::read_u32(rest).unwrap();
                    self.remote_window = self.remote_window.saturating_add(bytes);
                }
                true
            }
            _ => true, // Ignore unknown messages
        }
    }

    fn handle_channel_request(&mut self, payload: &[u8]) -> bool {
        let rest = &payload[1..];
        let (_recipient, rest) = match ssh::read_u32(rest) {
            Some(r) => r,
            None => return false,
        };
        let (request_type, rest) = match ssh::read_string(rest) {
            Some(r) => r,
            None => return false,
        };
        let want_reply = if !rest.is_empty() { rest[0] != 0 } else { false };

        match request_type {
            b"pty-req" => {
                // Accept PTY request (we don't actually do anything with it)
                if want_reply {
                    let mut resp = [0u8; 8];
                    resp[0] = ssh::msg::CHANNEL_SUCCESS;
                    ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                    self.send_packet(&resp[..5]);
                }
                true
            }
            b"shell" => {
                if want_reply {
                    let mut resp = [0u8; 8];
                    resp[0] = ssh::msg::CHANNEL_SUCCESS;
                    ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                    self.send_packet(&resp[..5]);
                }
                self.state = SshState::ShellRunning;

                // Send banner and prompt
                self.send_channel_data(b"FilamentOS remote shell\r\nType 'help' for commands.\r\n$ ");

                uinfo!("sshd", "shell_started";);
                true
            }
            b"exec" => {
                // For SCP support: exec a command
                if want_reply {
                    let mut resp = [0u8; 8];
                    resp[0] = ssh::msg::CHANNEL_SUCCESS;
                    ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                    self.send_packet(&resp[..5]);
                }
                // TODO: implement exec for SCP
                true
            }
            _ => {
                // Unknown request — reject if reply wanted
                if want_reply {
                    // CHANNEL_FAILURE = 100
                    let mut resp = [0u8; 8];
                    resp[0] = 100; // CHANNEL_FAILURE
                    ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                    self.send_packet(&resp[..5]);
                }
                true
            }
        }
    }

    fn do_shell_running(&mut self) -> bool {
        // Bridge SSH channel data ↔ shell IPC
        let payload = match self.recv_packet() {
            Some(p) => p,
            None => return false,
        };

        if payload.is_empty() {
            return false;
        }

        match payload[0] {
            ssh::msg::CHANNEL_DATA => {
                // Data from SSH client → shell
                let rest = &payload[1..];
                let (_channel, rest) = match ssh::read_u32(rest) {
                    Some(r) => r,
                    None => return false,
                };
                let (data, _) = match ssh::read_string(rest) {
                    Some(r) => r,
                    None => return false,
                };

                self.process_shell_input(data)
            }
            ssh::msg::CHANNEL_WINDOW_ADJUST => {
                if payload.len() >= 9 {
                    let (_, rest) = ssh::read_u32(&payload[1..]).unwrap();
                    let (bytes, _) = ssh::read_u32(rest).unwrap();
                    self.remote_window = self.remote_window.saturating_add(bytes);
                }
                true
            }
            ssh::msg::CHANNEL_EOF | ssh::msg::CHANNEL_CLOSE => {
                // Client closed the channel
                let mut resp = [0u8; 8];
                resp[0] = ssh::msg::CHANNEL_CLOSE;
                ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                self.send_packet(&resp[..5]);
                self.state = SshState::Closing;
                true
            }
            _ => true,
        }
    }

    /// Process input data from SSH client, send to shell.
    fn process_shell_input(&mut self, data: &[u8]) -> bool {
        // Accumulate data until we get a line
        for &b in data {
            match b {
                b'\r' | b'\n' => {
                    if self.read_buf.is_empty() {
                        self.send_channel_data(b"\r\n$ ");
                        continue;
                    }

                    // Echo the newline
                    self.send_channel_data(b"\r\n");

                    let cmd = core::mem::take(&mut self.read_buf);
                    self.execute_command(&cmd);
                }
                0x7F | 0x08 => {
                    // Backspace
                    if !self.read_buf.is_empty() {
                        self.read_buf.pop();
                        self.send_channel_data(b"\x08 \x08");
                    }
                }
                0x03 => {
                    // Ctrl-C
                    self.read_buf.clear();
                    self.send_channel_data(b"^C\r\n$ ");
                }
                0x04 => {
                    // Ctrl-D → close
                    self.state = SshState::Closing;
                    return true;
                }
                _ if b >= 0x20 => {
                    self.read_buf.push(b);
                    // Echo
                    self.send_channel_data(&[b]);
                }
                _ => {} // Ignore other control chars
            }
        }
        true
    }

    /// Execute a command directly using syscalls.
    ///
    /// No IPC to shell — avoids blocking the shell's console and deadlocks.
    /// Implements basic commands for remote management.
    fn execute_command(&mut self, cmd: &[u8]) {
        let cmd = trim(cmd);
        if cmd.is_empty() {
            self.send_channel_data(b"$ ");
            return;
        }

        let (verb, args) = split_first_word(cmd);

        match verb {
            b"exit" | b"quit" => {
                self.send_channel_data(b"Goodbye.\r\n");
                self.state = SshState::Closing;
                return;
            }
            b"help" | b"?" => self.cmd_help(),
            b"ps" => self.cmd_ps(),
            b"uptime" => self.cmd_uptime(),
            b"ls" => self.cmd_ls(),
            b"kill" => self.cmd_kill(args),
            b"uname" => { self.send_channel_data(b"FilamentOS 0.1 aarch64 MT7988A\r\n"); }
            _ => {
                self.send_channel_data(b"Unknown command. Type 'help' for available commands.\r\n");
            }
        }

        self.send_channel_data(b"$ ");
    }

    fn cmd_help(&mut self) {
        self.send_channel_data(
            b"Commands:\r\n\
              \x20 help      Show this help\r\n\
              \x20 ps        Process list\r\n\
              \x20 uptime    System uptime\r\n\
              \x20 ls        List ramfs files\r\n\
              \x20 kill N    Kill process by PID\r\n\
              \x20 uname     System info\r\n\
              \x20 exit      Close session\r\n"
        );
    }

    fn cmd_ps(&mut self) {
        let mut procs = [syscall::ProcessInfo::empty(); 32];
        let count = syscall::ps_info(&mut procs);

        let mut out = [0u8; 2048];
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
                if pos < out.len() { out[pos] = b' '; pos += 1; }
            }
            let name = name_str(&p.name);
            pos += copy_bytes(&mut out[pos..], name);
            pos += copy_str(&mut out[pos..], "\r\n");
            if pos > out.len() - 64 { break; }
        }
        self.send_channel_data(&out[..pos]);
    }

    fn cmd_uptime(&mut self) {
        let ns = syscall::gettime();
        let secs = ns / 1_000_000_000;
        let mins = secs / 60;
        let hours = mins / 60;

        let mut out = [0u8; 64];
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
        self.send_channel_data(&out[..pos]);
    }

    fn cmd_ls(&mut self) {
        let mut entries = [syscall::RamfsListEntry::empty(); 32];
        let count = syscall::ramfs_list(&mut entries);
        if count < 0 {
            self.send_channel_data(b"ramfs_list failed\r\n");
            return;
        }

        let mut out = [0u8; 2048];
        let mut pos = 0;
        for i in 0..count as usize {
            let e = &entries[i];
            let name = e.name_str();
            pos += copy_bytes(&mut out[pos..], name);
            for _ in name.len()..20 {
                if pos < out.len() { out[pos] = b' '; pos += 1; }
            }
            pos += fmt_u64(&mut out[pos..], e.size);
            pos += copy_str(&mut out[pos..], "\r\n");
            if pos > out.len() - 128 { break; }
        }
        if count == 0 {
            pos += copy_str(&mut out[pos..], "(empty)\r\n");
        }
        self.send_channel_data(&out[..pos]);
    }

    fn cmd_kill(&mut self, args: &[u8]) {
        let args = trim(args);
        if args.is_empty() {
            self.send_channel_data(b"Usage: kill <pid>\r\n");
            return;
        }

        let pid = parse_u32(args);
        if pid == 0 {
            self.send_channel_data(b"Invalid PID\r\n");
            return;
        }

        let ret = syscall::kill(pid);
        let mut out = [0u8; 64];
        let mut pos = 0;
        if ret == 0 {
            pos += copy_str(&mut out[pos..], "Killed PID ");
            pos += fmt_u64(&mut out[pos..], pid as u64);
            pos += copy_str(&mut out[pos..], "\r\n");
        } else {
            pos += copy_str(&mut out[pos..], "kill failed\r\n");
        }
        self.send_channel_data(&out[..pos]);
    }


    /// Send data on the SSH channel.
    fn send_channel_data(&mut self, data: &[u8]) -> bool {
        if data.is_empty() {
            return true;
        }

        let mut pkt = [0u8; 4128];
        pkt[0] = ssh::msg::CHANNEL_DATA;
        let mut pos = 1;
        pos += ssh::write_u32(&mut pkt[pos..], self.remote_channel_id);
        pos += ssh::write_string(&mut pkt[pos..], data);
        self.send_packet(&pkt[..pos])
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

fn fmt_u32_pad(buf: &mut [u8], val: u32, width: usize) -> usize {
    let mut tmp = [0u8; 10];
    let digits = fmt_u64(&mut tmp, val as u64);
    let mut pos = 0;
    if digits < width {
        for _ in 0..(width - digits) {
            if pos < buf.len() { buf[pos] = b' '; pos += 1; }
        }
    }
    let copy_len = digits.min(buf.len() - pos);
    buf[pos..pos + copy_len].copy_from_slice(&tmp[..copy_len]);
    pos + copy_len
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

// =============================================================================
// Main
// =============================================================================

#[unsafe(no_mangle)]
fn main() {
    uinfo!("sshd", "starting";);

    // Retry binding until ipd is ready
    let mut listener = loop {
        let addr = SocketAddr::new(Ipv4Addr::UNSPECIFIED, SSH_PORT);
        match TcpListener::bind(addr) {
            Ok(l) => break l,
            Err(_) => {
                syscall::sleep_ns(1_000_000_000); // 1s retry
            }
        }
    };

    uinfo!("sshd", "listening"; port = SSH_PORT as u32);

    // Accept loop — re-bind after each connection since ipd's
    // listening socket is consumed on accept.
    loop {
        match listener.accept() {
            Ok((stream, remote)) => {
                uinfo!("sshd", "connection"; remote_port = remote.port as u32);

                let mut session = SshSession::new(stream, DEV_HOST_KEY_SEED);
                session.run();

                uinfo!("sshd", "session_ended";);
            }
            Err(_) => {
                uerror!("sshd", "accept_failed";);
            }
        }

        // Drop old listener and re-bind for the next connection
        drop(listener);
        syscall::sleep_ns(100_000_000); // 100ms grace for socket cleanup
        listener = loop {
            let addr = SocketAddr::new(Ipv4Addr::UNSPECIFIED, SSH_PORT);
            match TcpListener::bind(addr) {
                Ok(l) => break l,
                Err(_) => {
                    syscall::sleep_ns(500_000_000); // 500ms retry
                }
            }
        };
        uinfo!("sshd", "re_listening"; port = SSH_PORT as u32);
    }
}
