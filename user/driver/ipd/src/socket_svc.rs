//! Socket Service — exposes TCP sockets to external processes.
//!
//! External processes connect to ipd via the "tcp:" IPC port and send
//! LISTEN/CLOSE messages. Data flows through per-connection DataPorts
//! (one DataPort per TCP connection, zero-copy shared memory).
//!
//! Control plane: IPC channel (small, infrequent messages)
//! Data plane: DataPort per connection (zero-copy pool offsets)
//!
//! DataPort role assignment:
//!   ipd = Provider (creates the DataPort, allocates from first half of pool)
//!   client = Consumer (connects via shmem_id, allocates from second half)
//!
//! RX (ipd→client): ipd reads from TCP socket → writes to provider pool half →
//!   posts CQE with { flags: CQE_STREAM_DATA, result: offset, transferred: len }
//!
//! TX (client→ipd): client writes to consumer pool half → submits SQE with
//!   { opcode: STREAM_DATA, data_offset, data_len } → ipd reads SQE, sends
//!   data from pool to TCP socket, frees the consumer pool slot.

use smoltcp::iface::SocketSet;
use smoltcp::socket::tcp;

use userlib::bus::BlockPortConfig;
use userlib::data_port::DataPort;
use userlib::ipc::{Channel, Port};
use userlib::ring::{io_op, io_status, IoCqe};
use userlib::syscall::Handle;
use userlib::{udebug, uerror, uinfo};

// =============================================================================
// Constants
// =============================================================================

/// Maximum concurrent TCP connections via socket service.
const MAX_CONNECTIONS: usize = 8;

/// TCP buffer size per socket direction (RX and TX).
const SOCK_TCP_BUF_SIZE: usize = 4096;

/// DataPort pool size per connection (shared memory).
const CONN_POOL_SIZE: u32 = 65536; // 64KB

/// DataPort ring size per connection.
const CONN_RING_SIZE: u16 = 32;

// =============================================================================
// Control Protocol Message Types
// =============================================================================

pub mod msg_type {
    // Client → ipd
    pub const LISTEN: u8 = 0x01;
    pub const CLOSE: u8 = 0x03;

    // ipd → client
    pub const LISTEN_OK: u8 = 0x81;
    pub const ACCEPTED: u8 = 0x82;
    pub const CLOSE_OK: u8 = 0x83;
    pub const CLOSED: u8 = 0x84;
    pub const ERROR: u8 = 0x8F;
}

pub mod close_reason {
    pub const FIN: u8 = 0;
    pub const RST: u8 = 1;
}

// =============================================================================
// Connection State
// =============================================================================

#[derive(Clone, Copy, PartialEq, Eq)]
enum ConnState {
    /// Slot is free.
    Free,
    /// Client connected on IPC, waiting for LISTEN message or TCP accept.
    WaitingListen,
    /// TCP socket is listening, waiting for incoming connection.
    Listening,
    /// TCP connection established, DataPort bridging active.
    Established,
    /// TCP connection closing, waiting for smoltcp to finish.
    Closing,
}

/// Per-connection state.
struct TcpConn {
    state: ConnState,
    smol_handle: Option<smoltcp::iface::SocketHandle>,
    data_port: Option<DataPort>,
    /// Client IPC channel (for control messages back to client)
    client_channel: Option<Channel>,
    /// TCP port being listened on
    local_port: u16,
    /// Connection ID (for client reference)
    conn_id: u32,
    /// True if we already sent STREAM_EOF to the client
    eof_sent: bool,
}

impl TcpConn {
    const fn free() -> Self {
        Self {
            state: ConnState::Free,
            smol_handle: None,
            data_port: None,
            client_channel: None,
            local_port: 0,
            conn_id: 0,
            eof_sent: false,
        }
    }

    fn reset(&mut self) {
        self.state = ConnState::Free;
        self.smol_handle = None;
        self.data_port = None;
        self.client_channel = None;
        self.local_port = 0;
        self.conn_id = 0;
        self.eof_sent = false;
    }
}

// =============================================================================
// Static smoltcp TCP buffer storage for socket service connections
// =============================================================================

// Each connection gets its own RX and TX buffers. These must be separate
// static muts for the same reason as the main ipd buffers: SocketSet<'static>
// requires 'static borrows that can't come from struct fields.
//
// SAFETY: ipd is single-threaded. Each slot's buffers are borrowed once
// when the TCP socket is created and released when the socket is removed.

static mut SOCK_SVC_TCP_RX: [[u8; SOCK_TCP_BUF_SIZE]; MAX_CONNECTIONS] =
    [[0u8; SOCK_TCP_BUF_SIZE]; MAX_CONNECTIONS];
static mut SOCK_SVC_TCP_TX: [[u8; SOCK_TCP_BUF_SIZE]; MAX_CONNECTIONS] =
    [[0u8; SOCK_TCP_BUF_SIZE]; MAX_CONNECTIONS];

// =============================================================================
// Socket Service
// =============================================================================

/// Socket service state.
pub struct SocketService {
    /// IPC port for accepting control connections from clients.
    tcp_port: Option<Port>,
    /// Connection table.
    conns: [TcpConn; MAX_CONNECTIONS],
    /// Next connection ID (monotonic).
    next_conn_id: u32,
}

impl SocketService {
    pub const fn new() -> Self {
        Self {
            tcp_port: None,
            conns: [const { TcpConn::free() }; MAX_CONNECTIONS],
            next_conn_id: 1,
        }
    }

    /// Initialize the socket service: register "tcp:" port.
    pub fn init(&mut self) {
        match Port::register(b"tcp:") {
            Ok(port) => {
                uinfo!("ipd", "socket_svc_ready"; port = "tcp:");
                self.tcp_port = Some(port);
            }
            Err(e) => {
                uerror!("ipd", "tcp_port_register_failed";);
            }
        }
    }

    /// Get the port handle for Mux registration.
    pub fn port_handle(&self) -> Option<Handle> {
        self.tcp_port.as_ref().map(|p| p.handle())
    }

    /// Accept pending client connections on the "tcp:" port.
    /// Returns newly accepted (handle, slot_index) for Mux registration.
    pub fn accept_clients(&mut self) -> Option<(Handle, u32)> {
        let port = self.tcp_port.as_mut()?;

        match port.try_accept() {
            Some(channel) => {
                let handle = channel.handle();
                for (i, conn) in self.conns.iter_mut().enumerate() {
                    if conn.state == ConnState::Free {
                        conn.client_channel = Some(channel);
                        conn.state = ConnState::WaitingListen;
                        conn.conn_id = self.next_conn_id;
                        self.next_conn_id += 1;
                        udebug!("ipd", "socket_client_accepted"; slot = i as u32);
                        return Some((handle, i as u32));
                    }
                }
                // No free slots
                uerror!("ipd", "socket_slots_full";);
                None
            }
            None => None,
        }
    }

    /// Process control messages from a client channel.
    pub fn process_client_msg(&mut self, slot: usize, sockets: &mut SocketSet<'static>) {
        if slot >= MAX_CONNECTIONS {
            return;
        }
        let conn = &mut self.conns[slot];
        let channel = match &mut conn.client_channel {
            Some(ch) => ch,
            None => return,
        };

        let mut buf = [0u8; 64];
        let mut msg_copy = [0u8; 64];
        let mut msg_len = 0usize;
        let mut disconnected = false;

        match channel.try_recv(&mut buf) {
            Ok(Some(n)) if n >= 1 => {
                msg_copy[..n].copy_from_slice(&buf[..n]);
                msg_len = n;
            }
            Ok(Some(_)) | Ok(None) => {}
            Err(_) => {
                disconnected = true;
            }
        }

        if disconnected {
            udebug!("ipd", "socket_client_disconnected"; slot = slot as u32);
            self.close_connection(slot, sockets);
        } else if msg_len > 0 {
            self.handle_control_msg(slot, &msg_copy[..msg_len], sockets);
        }
    }

    /// Handle a control message from a client.
    fn handle_control_msg(
        &mut self,
        slot: usize,
        msg: &[u8],
        sockets: &mut SocketSet<'static>,
    ) {
        if msg.is_empty() {
            return;
        }
        match msg[0] {
            msg_type::LISTEN => {
                if msg.len() < 3 {
                    self.send_error(slot, b"bad listen msg");
                    return;
                }
                let port = u16::from_le_bytes([msg[1], msg[2]]);
                self.handle_listen(slot, port, sockets);
            }
            msg_type::CLOSE => {
                if msg.len() < 5 {
                    self.send_error(slot, b"bad close msg");
                    return;
                }
                let conn_id = u32::from_le_bytes([msg[1], msg[2], msg[3], msg[4]]);
                self.handle_close(slot, conn_id, sockets);
            }
            _ => {
                self.send_error(slot, b"unknown msg type");
            }
        }
    }

    /// Handle a LISTEN request: create a TCP listening socket.
    fn handle_listen(
        &mut self,
        slot: usize,
        port: u16,
        sockets: &mut SocketSet<'static>,
    ) {
        let handle = match self.create_tcp_socket(slot, sockets) {
            Some(h) => h,
            None => {
                self.send_error(slot, b"no socket buffers");
                return;
            }
        };

        let sock = sockets.get_mut::<tcp::Socket>(handle);
        if sock.listen(port).is_err() {
            uerror!("ipd", "socket_listen_failed"; port = port as u32, slot = slot as u32);
            self.send_error(slot, b"listen failed");
            sockets.remove(handle);
            return;
        }

        let conn = &mut self.conns[slot];
        conn.smol_handle = Some(handle);
        conn.local_port = port;
        conn.state = ConnState::Listening;

        // Send LISTEN_OK: type(1) + conn_id(4)
        let mut resp = [0u8; 8];
        resp[0] = msg_type::LISTEN_OK;
        resp[1..5].copy_from_slice(&conn.conn_id.to_le_bytes());
        self.send_to_client(slot, &resp[..5]);

        uinfo!("ipd", "socket_listening"; port = port as u32, slot = slot as u32);
    }

    /// Handle a CLOSE request.
    fn handle_close(
        &mut self,
        slot: usize,
        conn_id: u32,
        sockets: &mut SocketSet<'static>,
    ) {
        let conn = &mut self.conns[slot];
        if conn.conn_id != conn_id {
            self.send_error(slot, b"bad conn_id");
            return;
        }

        if let Some(handle) = conn.smol_handle {
            let sock = sockets.get_mut::<tcp::Socket>(handle);
            sock.close();
            conn.state = ConnState::Closing;
        }

        // Send CLOSE_OK: type(1) + conn_id(4)
        let mut resp = [0u8; 8];
        resp[0] = msg_type::CLOSE_OK;
        resp[1..5].copy_from_slice(&conn_id.to_le_bytes());
        self.send_to_client(slot, &resp[..5]);
    }

    /// Create a TCP socket for a connection slot.
    fn create_tcp_socket(
        &mut self,
        slot: usize,
        sockets: &mut SocketSet<'static>,
    ) -> Option<smoltcp::iface::SocketHandle> {
        if slot >= MAX_CONNECTIONS {
            return None;
        }

        // SAFETY: ipd is single-threaded. Each slot's buffers are borrowed once.
        let (rx_buf, tx_buf) = unsafe {
            (
                &mut *(&raw mut SOCK_SVC_TCP_RX[slot]),
                &mut *(&raw mut SOCK_SVC_TCP_TX[slot]),
            )
        };

        let rx = tcp::SocketBuffer::new(&mut rx_buf[..]);
        let tx = tcp::SocketBuffer::new(&mut tx_buf[..]);
        let socket = tcp::Socket::new(rx, tx);
        Some(sockets.add(socket))
    }

    /// Returns true if any connection is in the Established state.
    pub fn has_established(&self) -> bool {
        self.conns.iter().any(|c| c.state == ConnState::Established)
    }

    /// Poll all active connections: bridge smoltcp sockets ↔ DataPorts.
    pub fn poll(&mut self, sockets: &mut SocketSet<'static>) {
        for slot in 0..MAX_CONNECTIONS {
            match self.conns[slot].state {
                ConnState::Listening => self.poll_listening(slot, sockets),
                ConnState::Established => self.poll_established(slot, sockets),
                ConnState::Closing => self.poll_closing(slot, sockets),
                ConnState::Free | ConnState::WaitingListen => {}
            }
        }
    }

    /// Check if a listening socket has accepted a connection.
    fn poll_listening(&mut self, slot: usize, sockets: &mut SocketSet<'static>) {
        let conn = &mut self.conns[slot];
        let handle = match conn.smol_handle {
            Some(h) => h,
            None => return,
        };

        let sock = sockets.get_mut::<tcp::Socket>(handle);
        if !sock.is_active() {
            return;
        }

        // Socket transitioned from listening to connected
        if sock.may_send() && sock.may_recv() {
            let (remote_ip, remote_port) = match sock.remote_endpoint() {
                Some(ep) => {
                    let ip = match ep.addr {
                        smoltcp::wire::IpAddress::Ipv4(v4) => v4.octets(),
                    };
                    (ip, ep.port)
                }
                None => ([0u8; 4], 0u16),
            };

            // Create DataPort for this connection (ipd = provider)
            let config = BlockPortConfig {
                ring_size: CONN_RING_SIZE,
                side_size: 0,
                pool_size: CONN_POOL_SIZE,
            };
            let data_port = match DataPort::create(config) {
                Ok(dp) => dp,
                Err(_) => {
                    uerror!("ipd", "dataport_create_failed"; slot = slot as u32);
                    return;
                }
            };

            // Make the DataPort public so the client process can connect
            data_port.set_public();
            let shmem_id = data_port.shmem_id();

            let conn = &mut self.conns[slot];
            let conn_id = conn.conn_id;
            let local_port = conn.local_port;
            conn.data_port = Some(data_port);
            conn.state = ConnState::Established;
            conn.eof_sent = false;

            // Send ACCEPTED: type(1) + conn_id(4) + shmem_id(4) + remote_ip(4) + remote_port(2) = 15
            let mut resp = [0u8; 16];
            resp[0] = msg_type::ACCEPTED;
            resp[1..5].copy_from_slice(&conn_id.to_le_bytes());
            resp[5..9].copy_from_slice(&shmem_id.to_le_bytes());
            resp[9..13].copy_from_slice(&remote_ip);
            resp[13..15].copy_from_slice(&remote_port.to_le_bytes());
            // Inline send_to_client to avoid double borrow
            if let Some(ref mut ch) = self.conns[slot].client_channel {
                if ch.send(&resp[..15]).is_err() {
                    uerror!("ipd", "socket_ctrl_send_failed"; slot = slot as u32);
                }
            }

            uinfo!("ipd", "socket_accepted";
                slot = slot as u32,
                port = local_port as u32,
                remote_port = remote_port as u32);
        }
    }

    /// Bridge data between smoltcp TCP socket and DataPort.
    fn poll_established(&mut self, slot: usize, sockets: &mut SocketSet<'static>) {
        let conn = &mut self.conns[slot];
        let handle = match conn.smol_handle {
            Some(h) => h,
            None => return,
        };

        {
            let sock = sockets.get_mut::<tcp::Socket>(handle);

            // Check for connection close/reset
            if !sock.is_active() {
                drop(sock);
                self.notify_closed(slot, close_reason::RST, sockets);
                return;
            }
        }

        // RX: smoltcp → DataPort (ipd→client)
        self.drain_tcp_to_port(slot, sockets);

        // TX: DataPort → smoltcp (client→ipd)
        self.drain_port_to_tcp(slot, sockets);
    }

    /// Drain data from TCP socket into DataPort (RX direction: ipd→client).
    fn drain_tcp_to_port(&mut self, slot: usize, sockets: &mut SocketSet<'static>) {
        let conn = &mut self.conns[slot];
        let handle = match conn.smol_handle {
            Some(h) => h,
            None => return,
        };
        let data_port = match &mut conn.data_port {
            Some(dp) => dp,
            None => return,
        };

        let sock = sockets.get_mut::<tcp::Socket>(handle);

        if !sock.may_recv() {
            // Remote end closed — send EOF CQE (once)
            if !conn.eof_sent {
                let cqe = IoCqe {
                    status: io_status::OK,
                    flags: io_status::CQE_STREAM_EOF,
                    tag: 0,
                    transferred: 0,
                    result: 0,
                };
                data_port.complete(&cqe);
                data_port.notify();
                conn.eof_sent = true;
            }
            return;
        }

        let mut posted = 0u32;

        // Read up to 4 chunks per poll cycle
        for _ in 0..4 {
            if !sock.may_recv() {
                break;
            }

            let available = sock.recv_queue();
            if available == 0 {
                break;
            }

            let chunk_size = available.min(4096);

            // Allocate from provider half of pool
            let offset = match data_port.alloc(chunk_size as u32) {
                Some(o) => o,
                None => break, // Pool full
            };

            // Read from TCP socket directly into pool
            let actual = sock.recv(|data| {
                let to_copy = data.len().min(chunk_size);
                if let Some(buf) = data_port.pool_slice_mut(offset, to_copy as u32) {
                    buf.copy_from_slice(&data[..to_copy]);
                }
                (to_copy, to_copy)
            });

            match actual {
                Ok(n) if n > 0 => {
                    let cqe = IoCqe {
                        status: io_status::OK,
                        flags: io_status::CQE_STREAM_DATA,
                        tag: 0,
                        transferred: n as u32,
                        result: offset,
                    };
                    data_port.complete(&cqe);
                    posted += 1;
                }
                _ => {
                    data_port.free(offset);
                    break;
                }
            }
        }

        if posted > 0 {
            data_port.notify();
        }
    }

    /// Drain SQEs from DataPort into TCP socket (TX direction: client→ipd).
    fn drain_port_to_tcp(&mut self, slot: usize, sockets: &mut SocketSet<'static>) {
        let conn = &mut self.conns[slot];
        let handle = match conn.smol_handle {
            Some(h) => h,
            None => return,
        };
        let data_port = match &mut conn.data_port {
            Some(dp) => dp,
            None => return,
        };

        let sock = sockets.get_mut::<tcp::Socket>(handle);
        if !sock.may_send() {
            return;
        }

        for _ in 0..8 {
            let sqe = match data_port.recv() {
                Some(s) => s,
                None => break,
            };

            match sqe.opcode {
                io_op::STREAM_DATA => {
                    let offset = sqe.data_offset;
                    let len = sqe.data_len;

                    if let Some(data) = data_port.pool_slice(offset, len) {
                        if sock.send_slice(data).is_err() {
                            uerror!("ipd", "socket_tx_failed"; slot = slot as u32);
                        }
                    }

                    // Post TX completion CQE so the consumer can free its own pool slot.
                    // (Provider's PoolAlloc can't free consumer-half offsets.)
                    let cqe = IoCqe {
                        status: io_status::OK,
                        flags: io_status::CQE_STREAM_TX_DONE,
                        tag: 0,
                        transferred: 0,
                        result: offset,
                    };
                    data_port.complete(&cqe);
                    data_port.notify();
                }
                io_op::STREAM_RX_DONE => {
                    // Consumer is done with this RX buffer; free provider-half slot.
                    data_port.free(sqe.data_offset);
                }
                io_op::STREAM_EOF => {
                    sock.close();
                }
                _ => {}
            }
        }
    }

    /// Poll closing connections.
    fn poll_closing(&mut self, slot: usize, sockets: &mut SocketSet<'static>) {
        let conn = &mut self.conns[slot];
        let handle = match conn.smol_handle {
            Some(h) => h,
            None => {
                conn.reset();
                return;
            }
        };

        let sock = sockets.get_mut::<tcp::Socket>(handle);
        if sock.state() == tcp::State::TimeWait || !sock.is_active() {
            // Skip TIME_WAIT — abort immediately to free the socket slot.
            // We're a microkernel, not a public internet server.
            sock.abort();
            sockets.remove(handle);
            let port = conn.local_port;
            let had_client = conn.client_channel.is_some();
            conn.reset();

            // If the client (sshd) is still connected, re-listen on the same port
            // so the next TCP connection can be accepted.
            if had_client {
                // Client will need to send a new LISTEN to re-bind.
                // Nothing to do here — client handles reconnect.
            }
            udebug!("ipd", "socket_closed"; slot = slot as u32, port = port as u32);
        }
    }

    /// Notify client that a connection was closed.
    fn notify_closed(&mut self, slot: usize, reason: u8, sockets: &mut SocketSet<'static>) {
        if slot >= MAX_CONNECTIONS {
            return;
        }

        // Send EOF/RST CQE on the data port
        let flags = if reason == close_reason::RST {
            io_status::CQE_STREAM_RST
        } else {
            io_status::CQE_STREAM_EOF
        };
        if let Some(dp) = &mut self.conns[slot].data_port {
            let cqe = IoCqe {
                status: io_status::OK,
                flags,
                tag: 0,
                transferred: 0,
                result: 0,
            };
            dp.complete(&cqe);
            dp.notify();
        }

        // Send CLOSED message on control channel
        let conn_id = self.conns[slot].conn_id;
        let mut msg = [0u8; 8];
        msg[0] = msg_type::CLOSED;
        msg[1..5].copy_from_slice(&conn_id.to_le_bytes());
        msg[5] = reason;
        if let Some(ref mut ch) = self.conns[slot].client_channel {
            let _ = ch.send(&msg[..6]);
        }

        // Abort the socket immediately (skip TIME_WAIT) and free the slot
        if let Some(handle) = self.conns[slot].smol_handle {
            let sock = sockets.get_mut::<tcp::Socket>(handle);
            sock.abort();
            sockets.remove(handle);
        }
        let port = self.conns[slot].local_port;
        self.conns[slot].data_port = None;
        self.conns[slot].smol_handle = None;
        self.conns[slot].state = ConnState::Closing;

        udebug!("ipd", "socket_notify_closed"; slot = slot as u32, port = port as u32);
    }

    /// Close a connection and free resources.
    fn close_connection(&mut self, slot: usize, sockets: &mut SocketSet<'static>) {
        let conn = &mut self.conns[slot];
        if let Some(handle) = conn.smol_handle {
            let sock = sockets.get_mut::<tcp::Socket>(handle);
            sock.abort();
            sockets.remove(handle);
        }
        conn.reset();
    }

    /// Send a control message to a client.
    fn send_to_client(&mut self, slot: usize, msg: &[u8]) {
        let conn = &mut self.conns[slot];
        if let Some(ref mut ch) = conn.client_channel {
            if ch.send(msg).is_err() {
                uerror!("ipd", "socket_ctrl_send_failed"; slot = slot as u32);
            }
        }
    }

    /// Send an error message to a client.
    fn send_error(&mut self, slot: usize, reason: &[u8]) {
        let mut msg = [0u8; 64];
        msg[0] = msg_type::ERROR;
        let len = reason.len().min(63);
        msg[1..1 + len].copy_from_slice(&reason[..len]);
        self.send_to_client(slot, &msg[..1 + len]);
    }

    /// Get the number of additional smoltcp socket slots needed.
    pub const fn extra_socket_slots() -> usize {
        MAX_CONNECTIONS
    }

    /// Get a client channel handle for a specific slot (for Mux watching).
    pub fn client_handle(&self, slot: usize) -> Option<Handle> {
        if slot >= MAX_CONNECTIONS {
            return None;
        }
        self.conns[slot].client_channel.as_ref().map(|ch| ch.handle())
    }
}
