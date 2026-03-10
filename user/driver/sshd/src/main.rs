//! sshd — Minimal SSH-2 Server
//!
//! Bus framework driver that provides remote shell access over SSH.
//!
//! Architecture:
//!   1. Connects to ipd's "tcp:" socket service to listen on port 22
//!   2. Accepts TCP connections → gets DataPort for each
//!   3. Runs SSH-2 protocol (KEX, auth, channel) over the DataPort
//!   4. Spawns shell as child via exec_with_mailbox + SharedPipe
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
mod sftp;

use alloc::vec::Vec;

use libf::crypto;
use libf::io::{self, Read, Write};
use libf::time::Duration;
use libf::net::{TcpListener, TcpStream, SocketAddr, Ipv4Addr};

use libsys::syscall;
use libsys::syscall::Handle;
use libsys::ipc::{Mux, MuxFilter};
use libf::sync::SharedPipe;
use libos::supervision::SupervisionHandle;
use libos::{uinfo, udebug, uerror};

// =============================================================================
// Constants
// =============================================================================

const SSH_PORT: u16 = 22;
/// Hardcoded host key seed for development.
const DEV_HOST_KEY_SEED: [u8; 32] = [
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00,
];

/// Default password for development.
const DEV_PASSWORD: &[u8] = b"filament";

/// Window size we advertise to the client (how much it can send us).
const LOCAL_WINDOW_SIZE: u32 = 131072; // 128K

/// Send WINDOW_ADJUST when we've consumed this many bytes from the client.
/// Half the window — keeps the client flowing without excessive messages.
const WINDOW_ADJUST_THRESHOLD: u32 = LOCAL_WINDOW_SIZE / 2;

// =============================================================================
// SSH Session State Machine
// =============================================================================

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum SshState {
    ProtocolExchange,
    AlgorithmNegotiation,
    KeyExchange,
    NewKeys,
    ServiceRequest,
    Authentication,
    ChannelOpen,
    ShellRunning,
    SftpRunning,
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
    /// How many bytes the client allows us to send (decremented on send).
    remote_window: u32,
    /// How many bytes we've consumed from the client without telling it.
    /// When this crosses WINDOW_THRESHOLD, we send WINDOW_ADJUST.
    local_consumed: u32,

    // Sequence numbers
    tx_seqno: u64,
    rx_seqno: u64,

    // PTY dimensions from pty-req
    pty_cols: u16,
    pty_rows: u16,

    // Shell child process
    shell_ring: Option<SharedPipe>,
    shell_pid: Option<u32>,
    superq: Option<SupervisionHandle>,

    // Residual buffer: data pulled from pipe but not yet sent (window exhausted).
    tx_residual: [u8; 4096],
    tx_residual_off: usize,
    tx_residual_len: usize,

    // SSH-level receive buffer: raw TCP bytes awaiting SSH framing parse.
    // Decouples TCP reads from SSH packet parsing so recv never blocks.
    ssh_rx: [u8; 16384],
    ssh_rx_pos: usize,
    ssh_rx_len: usize,

    // SFTP subsystem handler (populated when subsystem="sftp" requested).
    sftp_handler: Option<sftp::SftpHandler>,

    // SFTP reassembly buffer: accumulates SSH channel data into SFTP packets.
    sftp_rx: Vec<u8>,

    // Diagnostic: count CHANNEL_DATA packets logged for hex dump.
    tx_diag_count: u32,
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
            local_consumed: 0,
            tx_seqno: 0,
            rx_seqno: 0,
            pty_cols: 80,
            pty_rows: 24,
            shell_ring: None,
            shell_pid: None,
            superq: None,
            tx_residual: [0u8; 4096],
            tx_residual_off: 0,
            tx_residual_len: 0,
            ssh_rx: [0u8; 16384],
            sftp_handler: None,
            sftp_rx: Vec::new(),
            tx_diag_count: 0,
            ssh_rx_pos: 0,
            ssh_rx_len: 0,
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
            uerror!("sshd", "version_write_failed";);
            libos::ulog::flush();
            return;
        }

        // Main session loop
        loop {
            let state_id = match self.state {
                SshState::ProtocolExchange => 0u32,
                SshState::AlgorithmNegotiation => 1,
                SshState::KeyExchange => 2,
                SshState::NewKeys => 3,
                SshState::ServiceRequest => 4,
                SshState::Authentication => 5,
                SshState::ChannelOpen => 6,
                SshState::ShellRunning => 7,
                SshState::SftpRunning => 8,
                SshState::Closing => 9,
            };

            let ok = match self.state {
                SshState::ProtocolExchange => self.do_protocol_exchange(),
                SshState::AlgorithmNegotiation => self.do_algorithm_negotiation(),
                SshState::KeyExchange => self.do_key_exchange(),
                SshState::NewKeys => self.do_new_keys(),
                SshState::ServiceRequest => self.do_service_request(),
                SshState::Authentication => self.do_authentication(),
                SshState::ChannelOpen => self.do_channel_open(),
                SshState::ShellRunning => self.do_shell_running(),
                SshState::SftpRunning => self.do_sftp_running(),
                SshState::Closing => {
                    uinfo!("sshd", "closing";);
                    libos::ulog::flush();
                    return;
                }
            };

            if !ok {
                uerror!("sshd", "state_failed"; state = state_id);
                libos::ulog::flush();
                return;
            }
        }
    }

    // =========================================================================
    // Read helpers
    // =========================================================================

    fn read_line(&mut self) -> Option<Vec<u8>> {
        let mut buf = [0u8; 1];
        let mut line = Vec::new();

        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => return None,
                Ok(1) => {
                    if buf[0] == b'\n' {
                        if line.last() == Some(&b'\r') {
                            line.pop();
                        }
                        return Some(line);
                    }
                    line.push(buf[0]);
                    if line.len() > 255 {
                        return None;
                    }
                }
                _ => return None,
            }
        }
    }

    fn read_packet(&mut self) -> Option<Vec<u8>> {
        let mut len_buf = [0u8; 4];
        if self.stream.read_exact(&mut len_buf).is_err() {
            return None;
        }
        let packet_length = u32::from_be_bytes(len_buf) as usize;
        if packet_length < 1 || packet_length > 35000 {
            return None;
        }

        let mut data = alloc::vec![0u8; packet_length];
        if self.stream.read_exact(&mut data).is_err() {
            return None;
        }

        self.rx_seqno += 1;

        let padding_length = data[0] as usize;
        if padding_length + 1 > packet_length {
            return None;
        }
        let payload_length = packet_length - 1 - padding_length;
        Some(data[1..1 + payload_length].to_vec())
    }

    fn read_encrypted_packet(&mut self) -> Option<Vec<u8>> {
        let rx = self.rx_transport.as_mut()?;

        let rx_seq = rx.seqno();

        // Read the 4-byte encrypted length. This is the ONLY read that may
        // timeout (WouldBlock) — no data consumed yet, framing stays intact.
        let mut enc_len = [0u8; 4];
        match self.stream.read_exact(&mut enc_len) {
            Ok(()) => {}
            Err(e) => {
                // WouldBlock = timeout, no data arrived. Not an error.
                if e.kind() != io::ErrorKind::WouldBlock {
                    uerror!("sshd", "recv_read_len_failed"; seq = rx_seq as u32);
                    libos::ulog::flush();
                }
                return None;
            }
        }

        let packet_length = rx.decrypt_length(&enc_len) as usize;
        if packet_length < 1 || packet_length > 35000 {
            uerror!("sshd", "recv_bad_pkt_len"; len = packet_length as u32, seq = rx_seq as u32);
            libos::ulog::flush();
            return None;
        }

        // Once we've read the header, the rest of the packet MUST arrive.
        // Clear any read timeout to avoid corrupting the framing.
        self.stream.set_read_timeout(0);

        let mut data = alloc::vec![0u8; packet_length + 16];
        if let Err(_) = self.stream.read_exact(&mut data) {
            uerror!("sshd", "recv_read_data_failed"; pkt_len = packet_length as u32);
            libos::ulog::flush();
            return None;
        }

        let result = rx.decrypt_packet(&enc_len, &data);
        if result.is_none() {
            uerror!("sshd", "recv_decrypt_failed"; pkt_len = packet_length as u32);
            libos::ulog::flush();
        }
        result
    }

    fn recv_packet(&mut self) -> Option<Vec<u8>> {
        if self.encrypted {
            self.read_encrypted_packet()
        } else {
            self.read_packet()
        }
    }

    // =========================================================================
    // Non-blocking SSH receive (buffered)
    // =========================================================================

    /// Drain any available TCP data into the SSH receive buffer.
    ///
    /// Non-blocking: only reads data that is already queued as CQEs.
    /// Call this before try_recv_packet() to feed the parser.
    fn buffer_available_tcp(&mut self) {
        // Compact: slide unconsumed bytes to front
        if self.ssh_rx_pos > 0 {
            let remaining = self.ssh_rx_len - self.ssh_rx_pos;
            self.ssh_rx.copy_within(self.ssh_rx_pos..self.ssh_rx_len, 0);
            self.ssh_rx_len = remaining;
            self.ssh_rx_pos = 0;
        }

        // Read whatever TCP data is immediately available
        while self.ssh_rx_len < self.ssh_rx.len() {
            if !self.stream.has_pending_data() {
                break;
            }
            match self.stream.read(&mut self.ssh_rx[self.ssh_rx_len..]) {
                Ok(0) => break,
                Ok(n) => self.ssh_rx_len += n,
                Err(_) => break,
            }
        }
    }

    /// Try to parse one complete SSH packet from the receive buffer.
    ///
    /// Returns None if insufficient data — never blocks.
    fn try_recv_packet(&mut self) -> Option<Vec<u8>> {
        if self.encrypted {
            self.try_recv_encrypted_packet()
        } else {
            self.try_recv_plain_packet()
        }
    }

    fn try_recv_encrypted_packet(&mut self) -> Option<Vec<u8>> {
        let avail = self.ssh_rx_len - self.ssh_rx_pos;
        if avail < 4 {
            return None; // Need at least the encrypted length header
        }

        let rx = self.rx_transport.as_ref()?;

        // Peek at the 4-byte encrypted length (side-effect-free)
        let enc_len: [u8; 4] = self.ssh_rx[self.ssh_rx_pos..self.ssh_rx_pos + 4]
            .try_into()
            .unwrap();
        let packet_length = rx.decrypt_length(&enc_len) as usize;

        if packet_length < 1 || packet_length > 35000 {
            uerror!("sshd", "recv_bad_pkt_len"; len = packet_length as u32);
            libos::ulog::flush();
            // Skip these 4 bytes to avoid getting stuck
            self.ssh_rx_pos += 4;
            return None;
        }

        // Total bytes needed: 4 (header) + packet_length + 16 (poly1305 tag)
        let total = 4 + packet_length + 16;
        if avail < total {
            return None; // Incomplete packet — wait for more TCP data
        }

        // We have a complete packet. Now decrypt (mutates seqno).
        let rx = self.rx_transport.as_mut()?;
        let body = &self.ssh_rx[self.ssh_rx_pos + 4..self.ssh_rx_pos + total];
        let result = rx.decrypt_packet(&enc_len, body);
        self.ssh_rx_pos += total;

        if result.is_none() {
            uerror!("sshd", "recv_decrypt_failed"; pkt_len = packet_length as u32);
            libos::ulog::flush();
        }
        result
    }

    fn try_recv_plain_packet(&mut self) -> Option<Vec<u8>> {
        let avail = self.ssh_rx_len - self.ssh_rx_pos;
        if avail < 4 {
            return None;
        }

        let buf = &self.ssh_rx[self.ssh_rx_pos..self.ssh_rx_len];
        let packet_length = u32::from_be_bytes(buf[..4].try_into().unwrap()) as usize;
        if packet_length < 2 || packet_length > 35000 {
            self.ssh_rx_pos += 4;
            return None;
        }

        let total = 4 + packet_length;
        if avail < total {
            return None; // Incomplete
        }

        let data = &buf[4..total];
        let padding_length = data[0] as usize;
        if padding_length + 1 > packet_length {
            self.ssh_rx_pos += total;
            return None;
        }
        let payload_length = packet_length - 1 - padding_length;
        let payload = data[1..1 + payload_length].to_vec();
        self.ssh_rx_pos += total;
        self.rx_seqno += 1;
        Some(payload)
    }

    fn send_packet(&mut self, payload: &[u8]) -> bool {
        if self.encrypted {
            if let Some(tx) = &mut self.tx_transport {
                let seq_before = tx.seqno();
                let mut out = [0u8; 8192];
                let n = tx.encrypt_packet(payload, &mut out);
                if n == 0 {
                    uerror!("sshd", "encrypt_failed"; payload_len = payload.len() as u32);
                    libos::ulog::flush();
                    return false;
                }

                // Log first encrypted CHANNEL_DATA packets for diagnosis.
                if self.tx_diag_count < 3 && payload.len() > 0 && payload[0] == ssh::msg::CHANNEL_DATA {
                    // Log first 16 bytes of encrypted output + seqno
                    let h0 = if n > 0 { out[0] } else { 0 };
                    let h1 = if n > 1 { out[1] } else { 0 };
                    let h2 = if n > 2 { out[2] } else { 0 };
                    let h3 = if n > 3 { out[3] } else { 0 };
                    udebug!("sshd", "tx_enc"; seq = seq_before as u32, plain_len = payload.len() as u32, enc_len = n as u32,
                        h0 = h0 as u32, h1 = h1 as u32, h2 = h2 as u32, h3 = h3 as u32);
                    self.tx_diag_count += 1;
                }

                match self.stream.write_all(&out[..n]) {
                    Ok(()) => true,
                    Err(e) => {
                        uerror!("sshd", "write_all_failed"; enc_len = n as u32, kind = e.kind() as u8 as u32);
                        libos::ulog::flush();
                        false
                    }
                }
            } else {
                false
            }
        } else {
            let mut out = [0u8; 8192];
            let n = ssh::frame_packet(payload, &mut out);
            if n == 0 { return false; }
            self.tx_seqno += 1;
            self.stream.write_all(&out[..n]).is_ok()
        }
    }

    // =========================================================================
    // Protocol phases
    // =========================================================================

    fn do_protocol_exchange(&mut self) -> bool {
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

        let time = syscall::gettime();
        let mut seed_material = [0u8; 64];
        seed_material[..8].copy_from_slice(&time.to_le_bytes());
        seed_material[8..40].copy_from_slice(&self.host_key_seed);
        self.server_ephemeral_secret = crypto::sha256(&seed_material);
        self.server_ephemeral_public = crypto::x25519_public(&self.server_ephemeral_secret);

        let cookie = crypto::sha256(b"sshd_cookie");
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
        let payload = match self.recv_packet() {
            Some(p) => p,
            None => return false,
        };

        if payload.is_empty() || payload[0] != ssh::msg::KEX_ECDH_INIT {
            return false;
        }

        let (q_c_data, _) = match ssh::read_string(&payload[1..]) {
            Some(r) => r,
            None => return false,
        };

        if q_c_data.len() != 32 {
            return false;
        }
        let mut q_c = [0u8; 32];
        q_c.copy_from_slice(q_c_data);

        let shared_secret = crypto::x25519(&self.server_ephemeral_secret, &q_c);

        let mut k_s = [0u8; 64];
        let k_s_len = ssh::build_host_key_blob(&self.host_key_public, &mut k_s);

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

        if !self.session_id_set {
            self.session_id = h;
            self.session_id_set = true;
        }

        let signature = crypto::ed25519_sign(&self.host_key_seed, &h);

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

        if !self.send_packet(&[ssh::msg::NEWKEYS]) {
            return false;
        }

        let key_c2s = ssh::derive_key_64(&shared_secret, &h, b'C', &self.session_id);
        let key_s2c = ssh::derive_key_64(&shared_secret, &h, b'D', &self.session_id);

        self.tx_transport = Some(ssh::EncryptedTransport::new(
            &key_s2c,
            self.tx_seqno,
        ));
        self.rx_transport = Some(ssh::EncryptedTransport::new(
            &key_c2s,
            self.rx_seqno + 1,
        ));

        self.state = SshState::NewKeys;

        udebug!("sshd", "kex_complete";);
        true
    }

    fn do_new_keys(&mut self) -> bool {
        let payload = match self.recv_packet() {
            Some(p) => p,
            None => return false,
        };

        if payload.is_empty() || payload[0] != ssh::msg::NEWKEYS {
            return false;
        }

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
            let mut resp = [0u8; 64];
            resp[0] = ssh::msg::USERAUTH_FAILURE;
            let n = 1 + ssh::write_string(&mut resp[1..], b"password");
            resp[n] = 0;
            if !self.send_packet(&resp[..n + 1]) {
                return false;
            }
            return true;
        }

        if method != b"password" {
            let mut resp = [0u8; 64];
            resp[0] = ssh::msg::USERAUTH_FAILURE;
            let n = 1 + ssh::write_string(&mut resp[1..], b"password");
            resp[n] = 0;
            if !self.send_packet(&resp[..n + 1]) {
                return false;
            }
            return true;
        }

        if rest.is_empty() {
            return false;
        }
        let _change = rest[0];
        let (password, _) = match ssh::read_string(&rest[1..]) {
            Some(r) => r,
            None => return false,
        };

        if password == DEV_PASSWORD {
            uinfo!("sshd", "auth_ok"; user_len = username.len() as u32);
            libos::ulog::flush();
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
            None => {
                uinfo!("sshd", "channel_open_recv_failed";);
                libos::ulog::flush();
                return false;
            }
        };

        if payload.is_empty() {
            uinfo!("sshd", "channel_open_empty";);
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

                let mut resp = [0u8; 32];
                resp[0] = ssh::msg::CHANNEL_OPEN_CONFIRMATION;
                let mut pos = 1;
                pos += ssh::write_u32(&mut resp[pos..], sender_channel);
                pos += ssh::write_u32(&mut resp[pos..], self.local_channel_id);
                pos += ssh::write_u32(&mut resp[pos..], LOCAL_WINDOW_SIZE);
                pos += ssh::write_u32(&mut resp[pos..], 32768); // max packet size
                if !self.send_packet(&resp[..pos]) {
                    return false;
                }

                udebug!("sshd", "channel_opened";);
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
            _ => true,
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
        let body = if rest.len() > 1 { &rest[1..] } else { &[] };

        match request_type {
            b"pty-req" => {
                // Parse: TERM(string) + cols(u32) + rows(u32) + ...
                if let Some((_term, rest2)) = ssh::read_string(body) {
                    if let Some((cols, rest3)) = ssh::read_u32(rest2) {
                        if let Some((rows, _)) = ssh::read_u32(rest3) {
                            if cols > 0 && cols <= 500 { self.pty_cols = cols as u16; }
                            if rows > 0 && rows <= 500 { self.pty_rows = rows as u16; }
                        }
                    }
                }

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

                // Spawn shell via exec_with_mailbox + SharedPipe
                if !self.spawn_shell() {
                    self.send_channel_data(b"Failed to spawn shell\r\n");
                    self.state = SshState::Closing;
                    return true;
                }

                self.state = SshState::ShellRunning;
                uinfo!("sshd", "shell_started";);
                libos::ulog::flush();
                true
            }
            b"window-change" => {
                // Parse: cols(u32) + rows(u32) + ...
                if let Some((cols, rest2)) = ssh::read_u32(body) {
                    if let Some((rows, _)) = ssh::read_u32(rest2) {
                        if cols > 0 && cols <= 500 { self.pty_cols = cols as u16; }
                        if rows > 0 && rows <= 500 { self.pty_rows = rows as u16; }

                        // Write SIZE message to ring RX
                        if let Some(ring) = &self.shell_ring {
                            let mut msg = [0u8; 32];
                            let mut i = 0;
                            for &b in b"SIZE " { msg[i] = b; i += 1; }
                            i += fmt_u16(&mut msg[i..], cols as u16);
                            msg[i] = b' '; i += 1;
                            i += fmt_u16(&mut msg[i..], rows as u16);
                            msg[i] = b'\n'; i += 1;
                            ring.push(&msg[..i]);
                            ring.notify();
                        }
                    }
                }
                // window-change never wants a reply
                true
            }
            b"subsystem" => {
                let (subsystem_name, _) = match ssh::read_string(body) {
                    Some(r) => r,
                    None => {
                        if want_reply {
                            let mut resp = [0u8; 8];
                            resp[0] = 100; // CHANNEL_FAILURE
                            ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                            self.send_packet(&resp[..5]);
                        }
                        return true;
                    }
                };

                if subsystem_name == b"sftp" {
                    uinfo!("sshd", "sftp_subsystem_requested";);
                    libos::ulog::flush();

                    match sftp::SftpHandler::new() {
                        Some(handler) => {
                            self.sftp_handler = Some(handler);
                            if want_reply {
                                let mut resp = [0u8; 8];
                                resp[0] = ssh::msg::CHANNEL_SUCCESS;
                                ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                                self.send_packet(&resp[..5]);
                            }
                            self.state = SshState::SftpRunning;
                            uinfo!("sshd", "sftp_started";);
                            libos::ulog::flush();
                        }
                        None => {
                            uerror!("sshd", "sftp_init_failed";);
                            if want_reply {
                                let mut resp = [0u8; 8];
                                resp[0] = 100; // CHANNEL_FAILURE
                                ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                                self.send_packet(&resp[..5]);
                            }
                        }
                    }
                } else {
                    if want_reply {
                        let mut resp = [0u8; 8];
                        resp[0] = 100; // CHANNEL_FAILURE
                        ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                        self.send_packet(&resp[..5]);
                    }
                }
                true
            }
            b"exec" => {
                if want_reply {
                    let mut resp = [0u8; 8];
                    resp[0] = ssh::msg::CHANNEL_SUCCESS;
                    ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                    self.send_packet(&resp[..5]);
                }
                true
            }
            _ => {
                if want_reply {
                    let mut resp = [0u8; 8];
                    resp[0] = 100; // CHANNEL_FAILURE
                    ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                    self.send_packet(&resp[..5]);
                }
                true
            }
        }
    }

    // =========================================================================
    // Shell spawn + bridge
    // =========================================================================

    /// Spawn shell as child process via exec_with_mailbox.
    fn spawn_shell(&mut self) -> bool {
        // Create shared pipe for shell I/O (64K output, 4K input)
        let ring = match SharedPipe::console() {
            Some(r) => r,
            None => {
                uerror!("sshd", "ring_create_failed";);
                return false;
            }
        };

        // Build mailbox
        let mut mailbox = [0u8; 73];
        mailbox[0..4].copy_from_slice(&0x4D424F58u32.to_le_bytes()); // "MBOX"
        mailbox[4..6].copy_from_slice(&1u16.to_le_bytes()); // version
        mailbox[64..68].copy_from_slice(&ring.shmem_id().to_le_bytes());
        mailbox[68..70].copy_from_slice(&self.pty_cols.to_le_bytes());
        mailbox[70..72].copy_from_slice(&self.pty_rows.to_le_bytes());
        mailbox[72] = 0x01; // flags: bit 0 = SSH transport (enable diagnostics)

        // IPC | MEM | SPAWN | KILL | SIGNAL
        let caps: u64 = 0x0C07;
        match syscall::exec_with_mailbox("shell", caps, &mailbox) {
            Ok((child_pid, _parent_mb_handle, parent_superq_handle)) => {
                ring.allow(child_pid);
                uinfo!("sshd", "shell_spawned"; pid = child_pid);
                libos::ulog::flush();
                self.superq = Some(SupervisionHandle::from_handle(parent_superq_handle));
                self.shell_pid = Some(child_pid);
                self.shell_ring = Some(ring);
                true
            }
            Err(e) => {
                uerror!("sshd", "shell_spawn_failed"; err = e as i32);
                libos::ulog::flush();
                false
            }
        }
    }

    /// Shell running — bridge SSH ↔ SharedPipe.
    ///
    /// Multiplexes pipe output and TCP input using a Mux. Pipe data and
    /// TCP data are both processed non-blockingly each iteration. recv_packet
    /// never blocks because TCP bytes are buffered incrementally and SSH
    /// packets are parsed only when complete.
    fn do_shell_running(&mut self) -> bool {
        uinfo!("sshd", "shell_loop_enter";);
        libos::ulog::flush();

        // Diagnostic: send a known test string to verify the data path.
        let test_sent = self.send_channel_data(b"[sshd: data path test]\r\n");
        udebug!("sshd", "test_probe"; sent = test_sent, window = self.remote_window, pool = self.stream.pool_remaining());
        libos::ulog::flush();

        // Create a Mux to watch pipe shmem, TCP DataPort, and SuperQ.
        let mux = match Mux::new() {
            Ok(m) => m,
            Err(_) => {
                uerror!("sshd", "mux_create_failed";);
                return false;
            }
        };

        let tcp_handle = self.stream.poll_handle();
        let pipe_handle = self.shell_ring.as_ref().map(|r| r.handle()).unwrap_or(Handle::INVALID);
        let superq_handle = self.superq.as_ref().map(|s| s.handle()).unwrap_or(Handle::INVALID);

        let _ = mux.add(tcp_handle, MuxFilter::Readable);
        if pipe_handle != Handle::INVALID {
            let _ = mux.add(pipe_handle, MuxFilter::Readable);
        }
        if superq_handle != Handle::INVALID {
            let _ = mux.add(superq_handle, MuxFilter::Readable);
        }

        udebug!("sshd", "mux_handles"; tcp = tcp_handle.0, pipe = pipe_handle.0, superq = superq_handle.0);
        libos::ulog::flush();

        let _ = mux.set_timeout(100);
        let mut loop_count = 0u32;
        let mut pipe_wake_count = 0u32;
        let mut tcp_wake_count = 0u32;
        let mut timeout_count = 0u32;

        loop {
            loop_count += 1;

            // Periodic status every ~5s (50 iterations at 100ms timeout)
            if loop_count % 50 == 0 {
                let pipe_readable = self.shell_ring.as_ref().map(|r| r.readable()).unwrap_or(0);
                let pipe_writable = self.shell_ring.as_ref().map(|r| r.writable()).unwrap_or(0);
                udebug!("sshd", "loop_status"; iter = loop_count,
                    pipe_rd = pipe_readable, pipe_wr = pipe_writable,
                    window = self.remote_window, ssh_buf = self.ssh_rx_len - self.ssh_rx_pos,
                    pipe_w = pipe_wake_count, tcp_w = tcp_wake_count, tmo = timeout_count);
                libos::ulog::flush();
                pipe_wake_count = 0;
                tcp_wake_count = 0;
                timeout_count = 0;
            }

            // 1. Drain pipe → SSH (non-blocking)
            if !self.drain_ring_tx() {
                uerror!("sshd", "drain_tx_failed";);
                libos::ulog::flush();
                return false;
            }

            // 2. Buffer any available TCP data (non-blocking)
            self.buffer_available_tcp();

            // 3. Parse and process complete SSH packets from buffer
            loop {
                match self.try_recv_packet() {
                    Some(payload) => {
                        if !self.handle_shell_packet(&payload) {
                            if self.state == SshState::Closing {
                                return true;
                            }
                            return false;
                        }
                    }
                    None => break,
                }
            }

            // 4. Check for EOF after buffering
            if self.stream.is_eof() && self.ssh_rx_pos >= self.ssh_rx_len {
                uinfo!("sshd", "shell_tcp_eof";);
                libos::ulog::flush();
                return false;
            }

            // 5. Check for state transition
            if self.state != SshState::ShellRunning {
                return true;
            }

            // 6. Check for child exit via SuperQ (non-blocking)
            if let Some(superq) = &self.superq {
                if let Ok(Some(_note)) = superq.try_recv() {
                    let _ = self.drain_ring_tx();
                    uinfo!("sshd", "shell_exited_superq";);
                    libos::ulog::flush();
                    self.send_channel_data(b"\r\n[Shell exited]\r\n");
                    let mut resp = [0u8; 8];
                    resp[0] = ssh::msg::CHANNEL_CLOSE;
                    ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                    self.send_packet(&resp[..5]);
                    self.state = SshState::Closing;
                    return true;
                }
            }

            // 7. Wait for activity on any watched handle (100ms timeout)
            match mux.wait() {
                Ok(event) => {
                    if event.handle == pipe_handle {
                        pipe_wake_count += 1;
                    } else if event.handle == tcp_handle {
                        tcp_wake_count += 1;
                    } else if event.is_signal() {
                        // signal, not counted
                    }
                }
                Err(_) => {
                    timeout_count += 1;
                }
            }
        }
    }

    // =========================================================================
    // SFTP subsystem bridge
    // =========================================================================

    /// SFTP running — bridge SSH channel ↔ SFTP protocol handler.
    ///
    /// SSH channel data carries SFTP packets (length-prefixed). We reassemble
    /// them from potentially fragmented SSH frames, dispatch to SftpHandler,
    /// and send responses back as SSH channel data.
    fn do_sftp_running(&mut self) -> bool {
        uinfo!("sshd", "sftp_loop_enter";);
        libos::ulog::flush();

        let mux = match Mux::new() {
            Ok(m) => m,
            Err(_) => return false,
        };

        let tcp_handle = self.stream.poll_handle();
        let _ = mux.add(tcp_handle, MuxFilter::Readable);
        let _ = mux.set_timeout(500);

        loop {
            // 1. Buffer available TCP data
            self.buffer_available_tcp();

            // 2. Parse and process SSH packets
            loop {
                match self.try_recv_packet() {
                    Some(payload) => {
                        if !self.handle_sftp_packet(&payload) {
                            if self.state == SshState::Closing {
                                return true;
                            }
                            return false;
                        }
                    }
                    None => break,
                }
            }

            // 3. Check for EOF
            if self.stream.is_eof() && self.ssh_rx_pos >= self.ssh_rx_len {
                uinfo!("sshd", "sftp_tcp_eof";);
                return false;
            }

            // 4. Check state transition
            if self.state != SshState::SftpRunning {
                return true;
            }

            // 5. Wait for activity
            let _ = mux.wait();
        }
    }

    /// Process a single SSH packet during SFTP-running state.
    fn handle_sftp_packet(&mut self, payload: &[u8]) -> bool {
        if payload.is_empty() {
            return false;
        }

        match payload[0] {
            ssh::msg::CHANNEL_DATA => {
                let rest = &payload[1..];
                let (_channel, rest) = match ssh::read_u32(rest) {
                    Some(r) => r,
                    None => return false,
                };
                let (data, _) = match ssh::read_string(rest) {
                    Some(r) => r,
                    None => return false,
                };

                // Track consumed bytes for window management
                self.local_consumed += data.len() as u32;
                self.send_window_adjust();

                // Append to SFTP reassembly buffer
                self.sftp_rx.extend_from_slice(data);

                // Process complete SFTP packets
                loop {
                    if self.sftp_rx.len() < 4 {
                        break;
                    }
                    let pkt_len = u32::from_be_bytes([
                        self.sftp_rx[0], self.sftp_rx[1],
                        self.sftp_rx[2], self.sftp_rx[3],
                    ]) as usize;

                    if pkt_len > 256 * 1024 {
                        uerror!("sshd", "sftp_pkt_too_large"; len = pkt_len as u32);
                        return false;
                    }

                    if self.sftp_rx.len() < 4 + pkt_len {
                        break; // Incomplete — wait for more data
                    }

                    // Extract the SFTP packet body (without length prefix)
                    let sftp_body: Vec<u8> = self.sftp_rx[4..4 + pkt_len].to_vec();

                    // Remove consumed bytes
                    let remaining = self.sftp_rx.len() - (4 + pkt_len);
                    if remaining > 0 {
                        self.sftp_rx.copy_within(4 + pkt_len.., 0);
                    }
                    self.sftp_rx.truncate(remaining);

                    // Dispatch to SFTP handler
                    if let Some(handler) = &mut self.sftp_handler {
                        let mut response = Vec::new();
                        handler.process(&sftp_body, &mut response);

                        if !response.is_empty() {
                            // Send response as SSH channel data
                            // SFTP response needs length prefix
                            let resp_len = response.len() as u32;
                            let mut framed = Vec::with_capacity(4 + response.len());
                            framed.extend_from_slice(&resp_len.to_be_bytes());
                            framed.extend_from_slice(&response);

                            // Send in chunks that fit in SSH channel data
                            let mut offset = 0;
                            while offset < framed.len() {
                                let chunk = (framed.len() - offset).min(4096);
                                let sent = self.send_channel_data(&framed[offset..offset + chunk]);
                                if sent == 0 {
                                    return false; // Connection error
                                }
                                offset += sent;
                            }
                        }
                    }
                }

                true
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
                let mut resp = [0u8; 8];
                resp[0] = ssh::msg::CHANNEL_CLOSE;
                ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                self.send_packet(&resp[..5]);
                self.state = SshState::Closing;
                false
            }
            ssh::msg::CHANNEL_REQUEST => {
                self.handle_channel_request(payload)
            }
            _ => true,
        }
    }

    /// Process a single SSH packet during shell-running state.
    /// Returns true to continue, false to stop (state changed or error).
    fn handle_shell_packet(&mut self, payload: &[u8]) -> bool {
        if payload.is_empty() {
            uerror!("sshd", "shell_empty_payload";);
            libos::ulog::flush();
            return false;
        }

        match payload[0] {
            ssh::msg::CHANNEL_DATA => {
                let rest = &payload[1..];
                let (_channel, rest) = match ssh::read_u32(rest) {
                    Some(r) => r,
                    None => return false,
                };
                let (data, _) = match ssh::read_string(rest) {
                    Some(r) => r,
                    None => return false,
                };

                // Signal relay
                if let Some(pid) = self.shell_pid {
                    for &b in data {
                        if b == 0x03 {
                            let _ = syscall::signal(pid, syscall::signal_event::INTERRUPT, 0);
                        }
                    }
                }

                // Forward client input to shell
                if let Some(ring) = &self.shell_ring {
                    ring.push(data);
                    ring.notify();
                }

                // Track bytes consumed so we can replenish the client's window.
                self.local_consumed += data.len() as u32;
                self.send_window_adjust();
                true
            }
            ssh::msg::CHANNEL_WINDOW_ADJUST => {
                if payload.len() >= 9 {
                    let (_, rest) = ssh::read_u32(&payload[1..]).unwrap();
                    let (bytes, _) = ssh::read_u32(rest).unwrap();
                    let old_window = self.remote_window;
                    self.remote_window = self.remote_window.saturating_add(bytes);
                    udebug!("sshd", "window_adjust"; added = bytes, old = old_window, new = self.remote_window);
                }
                true
            }
            ssh::msg::CHANNEL_REQUEST => {
                self.handle_channel_request(payload)
            }
            ssh::msg::CHANNEL_EOF | ssh::msg::CHANNEL_CLOSE => {
                if let Some(pid) = self.shell_pid {
                    let _ = syscall::signal(pid, syscall::signal_event::SHUTDOWN, 0);
                }
                let mut resp = [0u8; 8];
                resp[0] = ssh::msg::CHANNEL_CLOSE;
                ssh::write_u32(&mut resp[1..], self.remote_channel_id);
                self.send_packet(&resp[..5]);
                self.state = SshState::Closing;
                false
            }
            _ => true,
        }
    }

    /// Drain shell pipe → SSH channel data.
    ///
    /// Accumulates all available pipe data first, then sends in one shot.
    /// This coalesces many small shell writes (decoration, per-character
    /// output) into a single SSH packet instead of one packet per write.
    /// Returns false if the connection is broken.
    fn drain_ring_tx(&mut self) -> bool {
        // First, flush any residual data from a previous partial send.
        if self.tx_residual_len > self.tx_residual_off {
            if self.remote_window == 0 {
                return true; // Can't send yet — wait for WINDOW_ADJUST
            }
            let remaining = self.tx_residual_len - self.tx_residual_off;
            let mut tmp = [0u8; 4096];
            tmp[..remaining].copy_from_slice(
                &self.tx_residual[self.tx_residual_off..self.tx_residual_len]
            );
            let sent = self.send_channel_data(&tmp[..remaining]);
            self.tx_residual_off += sent;
            if self.tx_residual_off < self.tx_residual_len {
                return true; // Window or pool exhausted — retry after next mux wake
            }
            self.tx_residual_len = 0;
            self.tx_residual_off = 0;
        }

        // Accumulate all available pipe data before sending. This is the
        // key optimization: the shell writes many small fragments (ANSI
        // escapes, per-row cursor ops, individual characters). By draining
        // the entire pipe first, we coalesce them into one or two SSH
        // packets instead of dozens of 1-byte packets.
        let mut tx_buf = [0u8; 4096];
        let mut total = 0usize;

        if let Some(ring) = &self.shell_ring {
            loop {
                let n = ring.pull(&mut tx_buf[total..]);
                if n == 0 { break; }
                total += n;
                // Stop if buffer is nearly full — leave room for next pull
                if total >= tx_buf.len() - 256 { break; }
            }
            if total > 0 {
                ring.notify(); // Signal shell that pipe space is available
            }
        }

        if total == 0 {
            return true;
        }

        // If window is exhausted, stash everything for later.
        if self.remote_window == 0 {
            self.tx_residual[..total].copy_from_slice(&tx_buf[..total]);
            self.tx_residual_len = total;
            self.tx_residual_off = 0;
            udebug!("sshd", "drain_stall_window"; stashed = total as u32, window = 0u32);
            return true;
        }

        let sent = self.send_channel_data(&tx_buf[..total]);

        // Stash unsent bytes so we don't lose them.
        if sent < total {
            let leftover = total - sent;
            self.tx_residual[..leftover].copy_from_slice(&tx_buf[sent..total]);
            self.tx_residual_len = leftover;
            self.tx_residual_off = 0;
        }

        udebug!("sshd", "drain_ok"; pulled = total as u32, sent = sent as u32,
            window = self.remote_window, pool = self.stream.pool_remaining());

        true
    }

    /// Send data on the SSH channel, respecting the remote window.
    ///
    /// Chunks large payloads to fit within pool slots. Stops early if
    /// the remote window is exhausted or the TCP pool is full — caller
    /// should stash unsent bytes and retry after the next Mux wake.
    /// Returns the number of bytes actually sent (may be partial).
    /// Returns 0 only on genuine connection error (encrypted write failed).
    fn send_channel_data(&mut self, data: &[u8]) -> usize {
        if data.is_empty() {
            return 0;
        }

        // Max data per SSH packet (see pool slot size constraints).
        const MAX_CHUNK: usize = 1024;

        let mut offset = 0;
        let mut packets = 0u32;
        while offset < data.len() {
            // Respect the remote window — stop if exhausted.
            if self.remote_window == 0 {
                break;
            }

            // Check TCP pool capacity before building the packet.
            // Each encrypted SSH packet uses one 2048-byte pool slot.
            // If fewer than 2 slots remain, yield to let ipd post TX_DONEs.
            let pool_free = self.stream.pool_remaining();
            if pool_free < 2 {
                break;
            }

            let remaining = data.len() - offset;
            let chunk = remaining
                .min(MAX_CHUNK)
                .min(self.remote_window as usize);

            let mut pkt = [0u8; 1100];
            pkt[0] = ssh::msg::CHANNEL_DATA;
            let mut pos = 1;
            pos += ssh::write_u32(&mut pkt[pos..], self.remote_channel_id);
            pos += ssh::write_string(&mut pkt[pos..], &data[offset..offset + chunk]);
            if !self.send_packet(&pkt[..pos]) {
                return if offset > 0 { offset } else { 0 };
            }

            self.remote_window -= chunk as u32;
            offset += chunk;
            packets += 1;
        }

        if offset > 0 {
            udebug!("sshd", "chan_data_tx"; offered = data.len() as u32,
                sent = offset as u32, pkts = packets,
                window = self.remote_window, pool = self.stream.pool_remaining());
        }

        offset
    }

    /// Send WINDOW_ADJUST to let the client send more data.
    fn send_window_adjust(&mut self) {
        if self.local_consumed < WINDOW_ADJUST_THRESHOLD {
            return;
        }

        let mut pkt = [0u8; 16];
        pkt[0] = ssh::msg::CHANNEL_WINDOW_ADJUST;
        let mut pos = 1;
        pos += ssh::write_u32(&mut pkt[pos..], self.remote_channel_id);
        pos += ssh::write_u32(&mut pkt[pos..], self.local_consumed);
        self.send_packet(&pkt[..pos]);
        self.local_consumed = 0;
    }
}

// =============================================================================
// Helpers
// =============================================================================

fn fmt_u16(buf: &mut [u8], n: u16) -> usize {
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

#[unsafe(no_mangle)]
fn main() {
    uinfo!("sshd", "starting";);
    libos::ulog::flush();

    // Retry binding until ipd is ready
    let mut listener = loop {
        let addr = SocketAddr::new(Ipv4Addr::UNSPECIFIED, SSH_PORT);
        match TcpListener::bind(addr) {
            Ok(l) => break l,
            Err(_) => {
                libf::time::sleep(Duration::from_secs(1));
            }
        }
    };

    uinfo!("sshd", "listening"; port = SSH_PORT as u32);
    libos::ulog::flush();

    loop {
        match listener.accept() {
            Ok((stream, remote)) => {
                uinfo!("sshd", "connection"; remote_port = remote.port as u32);
                libos::ulog::flush();

                let mut session = SshSession::new(stream, DEV_HOST_KEY_SEED);
                session.run();

                // Kill shell if still alive
                if let Some(pid) = session.shell_pid {
                    let _ = syscall::kill(pid);
                }

                uinfo!("sshd", "session_ended";);
                libos::ulog::flush();
            }
            Err(_) => {
                uerror!("sshd", "accept_failed";);
                libos::ulog::flush();
            }
        }

        // Re-bind for next connection
        drop(listener);
        libf::time::sleep(Duration::from_millis(100));
        listener = loop {
            let addr = SocketAddr::new(Ipv4Addr::UNSPECIFIED, SSH_PORT);
            match TcpListener::bind(addr) {
                Ok(l) => break l,
                Err(_) => {
                    libf::time::sleep(Duration::from_millis(500));
                }
            }
        };
        uinfo!("sshd", "re_listening"; port = SSH_PORT as u32);
    }
}
