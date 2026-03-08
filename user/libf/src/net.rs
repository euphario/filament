//! Networking primitives — mirrors `std::net`.
//!
//! Provides [`TcpListener`], [`TcpStream`], [`Ipv4Addr`], and [`SocketAddr`]
//! with the same API shape as `std::net`. Backed by ipd's socket service:
//! control plane over IPC channel, data plane over DataPort shared memory.
//!
//! Also provides link-layer types ([`EthAddr`], [`eth`]) for Ethernet frame
//! processing shared across drivers (wifi2, ethd, switchd, ipd).
//!
//! # Example
//!
//! ```ignore
//! use libf::net::{TcpListener, SocketAddr, Ipv4Addr};
//! use libf::io::{Read, Write};
//!
//! let listener = TcpListener::bind(SocketAddr::new(Ipv4Addr::UNSPECIFIED, 8080))?;
//! let (mut stream, remote) = listener.accept()?;
//! let mut buf = [0u8; 1024];
//! let n = stream.read(&mut buf)?;
//! stream.write_all(&buf[..n])?;
//! ```

extern crate alloc;

use alloc::collections::VecDeque;
use crate::io;
use userlib::data_port::DataPort;
use userlib::ipc::Channel;
use userlib::ring::{io_op, io_status, IoCqe, IoSqe};

// =============================================================================
// Ethernet — link-layer types shared across drivers
// =============================================================================

/// Ethernet MAC address (6 bytes).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct EthAddr(pub [u8; 6]);

impl EthAddr {
    pub const BROADCAST: Self = Self([0xFF; 6]);
    pub const ZERO: Self = Self([0; 6]);

    pub fn is_broadcast(&self) -> bool { self.0 == [0xFF; 6] }
    pub fn is_multicast(&self) -> bool { self.0[0] & 0x01 != 0 }
}

/// Well-known EtherType constants.
pub mod ethertype {
    pub const IPV4: u16  = 0x0800;
    pub const ARP: u16   = 0x0806;
    pub const VLAN: u16  = 0x8100;
    pub const IPV6: u16  = 0x86DD;
    pub const EAPOL: u16 = 0x888E;
}

/// Ethernet II frame helpers.
///
/// ```ignore
/// use libf::net::eth;
/// if let Some(hdr) = eth::parse(&buf[offset..]) {
///     if hdr.ethertype == libf::net::ethertype::EAPOL {
///         let payload = &buf[offset + eth::HEADER_LEN..];
///     }
/// }
/// ```
pub mod eth {
    use super::EthAddr;

    /// Ethernet header length (DST + SRC + EtherType = 14 bytes).
    pub const HEADER_LEN: usize = 14;
    /// Standard Ethernet MTU.
    pub const MTU: usize = 1500;
    /// Maximum Ethernet frame (header + MTU, no FCS).
    pub const FRAME_MAX: usize = HEADER_LEN + MTU;

    /// Parsed Ethernet header.
    #[derive(Clone, Copy)]
    pub struct Header {
        pub dst: EthAddr,
        pub src: EthAddr,
        pub ethertype: u16,
    }

    /// Parse an Ethernet header from a byte slice.
    pub fn parse(frame: &[u8]) -> Option<Header> {
        if frame.len() < HEADER_LEN {
            return None;
        }
        let mut dst = [0u8; 6];
        let mut src = [0u8; 6];
        dst.copy_from_slice(&frame[..6]);
        src.copy_from_slice(&frame[6..12]);
        let ethertype = u16::from_be_bytes([frame[12], frame[13]]);
        Some(Header { dst: EthAddr(dst), src: EthAddr(src), ethertype })
    }

    /// Payload slice (everything after the 14-byte header).
    pub fn payload(frame: &[u8]) -> &[u8] {
        if frame.len() > HEADER_LEN { &frame[HEADER_LEN..] } else { &[] }
    }

    /// Write an Ethernet header into `buf`. Returns `HEADER_LEN` on success.
    pub fn write_header(buf: &mut [u8], dst: &EthAddr, src: &EthAddr, ethertype: u16) -> Option<usize> {
        if buf.len() < HEADER_LEN {
            return None;
        }
        buf[..6].copy_from_slice(&dst.0);
        buf[6..12].copy_from_slice(&src.0);
        buf[12..14].copy_from_slice(&ethertype.to_be_bytes());
        Some(HEADER_LEN)
    }
}

// =============================================================================
// Address types
// =============================================================================

/// An IPv4 address.
///
/// Mirrors `std::net::Ipv4Addr`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ipv4Addr(pub [u8; 4]);

impl Ipv4Addr {
    pub const UNSPECIFIED: Ipv4Addr = Ipv4Addr([0, 0, 0, 0]);
    pub const LOCALHOST: Ipv4Addr = Ipv4Addr([127, 0, 0, 1]);

    pub const fn new(a: u8, b: u8, c: u8, d: u8) -> Self {
        Ipv4Addr([a, b, c, d])
    }

    pub fn octets(&self) -> [u8; 4] {
        self.0
    }
}

/// An internet socket address (IPv4 + port).
///
/// Mirrors `std::net::SocketAddr` (IPv4 only).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SocketAddr {
    pub ip: Ipv4Addr,
    pub port: u16,
}

impl SocketAddr {
    pub const fn new(ip: Ipv4Addr, port: u16) -> Self {
        Self { ip, port }
    }
}

// =============================================================================
// Control protocol constants (must match ipd/src/socket_svc.rs)
// =============================================================================

mod msg_type {
    pub const LISTEN: u8 = 0x01;
    pub const CLOSE: u8 = 0x03;
    pub const LISTEN_OK: u8 = 0x81;
    pub const ACCEPTED: u8 = 0x82;
    pub const CLOSE_OK: u8 = 0x83;
    pub const CLOSED: u8 = 0x84;
    pub const ERROR: u8 = 0x8F;
}

// =============================================================================
// TcpListener
// =============================================================================

/// A TCP socket server, listening for connections.
///
/// Mirrors `std::net::TcpListener`.
pub struct TcpListener {
    /// IPC channel to ipd's socket service.
    control: Channel,
    /// Socket ID returned by ipd on LISTEN_OK.
    socket_id: u32,
    /// Local port being listened on.
    local_port: u16,
}

impl TcpListener {
    /// Creates a new `TcpListener` bound to the specified address.
    ///
    /// The listener will begin listening for incoming TCP connections.
    /// Only the port component of the address is used (ipd binds on
    /// all interfaces).
    pub fn bind(addr: SocketAddr) -> io::Result<TcpListener> {
        // Connect to ipd's "tcp:" port
        let mut control = Channel::connect(b"tcp:")
            .map_err(|_| io::Error::new(io::ErrorKind::ConnectionRefused))?;

        // Send LISTEN message: type(1) + port(2)
        let mut msg = [0u8; 3];
        msg[0] = msg_type::LISTEN;
        msg[1..3].copy_from_slice(&addr.port.to_le_bytes());
        control.send(&msg).map_err(io::Error::from)?;

        // Wait for LISTEN_OK response
        let mut resp = [0u8; 16];
        let n = control.recv(&mut resp).map_err(io::Error::from)?;
        if n < 5 || resp[0] != msg_type::LISTEN_OK {
            return Err(io::Error::new(io::ErrorKind::Other));
        }

        let socket_id = u32::from_le_bytes([resp[1], resp[2], resp[3], resp[4]]);

        Ok(TcpListener {
            control,
            socket_id,
            local_port: addr.port,
        })
    }

    /// Accept a new incoming connection.
    ///
    /// Blocks until a connection is established. Returns the stream and
    /// the remote address.
    pub fn accept(&mut self) -> io::Result<(TcpStream, SocketAddr)> {
        // Wait for ACCEPTED message from ipd
        let mut resp = [0u8; 16];
        let n = self.control.recv(&mut resp)
            .map_err(io::Error::from)?;

        if n < 15 || resp[0] != msg_type::ACCEPTED {
            if resp[0] == msg_type::ERROR {
                return Err(io::Error::new(io::ErrorKind::Other));
            }
            return Err(io::Error::new(io::ErrorKind::Other));
        }

        let conn_id = u32::from_le_bytes([resp[1], resp[2], resp[3], resp[4]]);
        let shmem_id = u32::from_le_bytes([resp[5], resp[6], resp[7], resp[8]]);
        let remote_ip = Ipv4Addr([resp[9], resp[10], resp[11], resp[12]]);
        let remote_port = u16::from_le_bytes([resp[13], resp[14]]);

        // Connect to the DataPort for this connection
        let data_port = DataPort::connect(shmem_id)
            .map_err(|_| io::Error::new(io::ErrorKind::Other))?;

        let remote_addr = SocketAddr::new(remote_ip, remote_port);

        Ok((
            TcpStream {
                data_port,
                conn_id,
                closed: false,
                // Inline RX buffer for partial reads
                rx_buf: [0u8; 4096],
                rx_pos: 0,
                rx_len: 0,
                eof: false,
                read_timeout_ms: 0,
                pending_cqes: VecDeque::new(),
            },
            remote_addr,
        ))
    }

    /// Non-blocking accept. Returns `Ok(None)` if no connection is pending.
    pub fn try_accept(&mut self) -> io::Result<Option<(TcpStream, SocketAddr)>> {
        let mut resp = [0u8; 16];
        match self.control.try_recv(&mut resp) {
            Ok(Some(n)) => {
                if n < 15 || resp[0] != msg_type::ACCEPTED {
                    if resp[0] == msg_type::ERROR {
                        return Err(io::Error::new(io::ErrorKind::Other));
                    }
                    return Err(io::Error::new(io::ErrorKind::Other));
                }

                let conn_id = u32::from_le_bytes([resp[1], resp[2], resp[3], resp[4]]);
                let shmem_id = u32::from_le_bytes([resp[5], resp[6], resp[7], resp[8]]);
                let remote_ip = Ipv4Addr([resp[9], resp[10], resp[11], resp[12]]);
                let remote_port = u16::from_le_bytes([resp[13], resp[14]]);

                let data_port = DataPort::connect(shmem_id)
                    .map_err(|_| io::Error::new(io::ErrorKind::Other))?;

                let remote_addr = SocketAddr::new(remote_ip, remote_port);

                Ok(Some((
                    TcpStream {
                        data_port,
                        conn_id,
                        closed: false,
                        rx_buf: [0u8; 4096],
                        rx_pos: 0,
                        rx_len: 0,
                        eof: false,
                        read_timeout_ms: 0,
                        pending_cqes: VecDeque::new(),
                    },
                    remote_addr,
                )))
            }
            Ok(None) => Ok(None), // WouldBlock
            Err(e) => Err(io::Error::from(e)),
        }
    }

    /// Returns the raw handle of the control channel (for Mux watching).
    pub fn control_handle(&self) -> userlib::syscall::Handle {
        self.control.handle()
    }

    /// Returns the local address that this listener is bound to.
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        Ok(SocketAddr::new(Ipv4Addr::UNSPECIFIED, self.local_port))
    }
}

impl Drop for TcpListener {
    fn drop(&mut self) {
        // Send CLOSE to ipd
        let mut msg = [0u8; 5];
        msg[0] = msg_type::CLOSE;
        msg[1..5].copy_from_slice(&self.socket_id.to_le_bytes());
        let _ = self.control.send(&msg);
    }
}

// =============================================================================
// TcpStream
// =============================================================================

/// A TCP stream between a local and a remote socket.
///
/// Mirrors `std::net::TcpStream`.
pub struct TcpStream {
    /// DataPort for zero-copy data transfer.
    data_port: DataPort,
    /// Connection ID (for control messages).
    conn_id: u32,
    /// Whether we've closed our end.
    closed: bool,
    /// Inline RX buffer for partial reads from CQEs.
    rx_buf: [u8; 4096],
    /// Current read position in rx_buf.
    rx_pos: usize,
    /// Valid data length in rx_buf.
    rx_len: usize,
    /// True when EOF has been received.
    eof: bool,
    /// Read timeout in milliseconds. 0 = infinite (default).
    read_timeout_ms: u32,
    /// CQEs consumed during drain_tx_completions but not yet processed.
    /// Prevents data loss when RX CQEs arrive while writing.
    pending_cqes: VecDeque<IoCqe>,
}

impl TcpStream {
    /// Set the read timeout in milliseconds.
    ///
    /// When set to a non-zero value, read operations will return
    /// `WouldBlock` if no data arrives within the timeout period.
    /// Set to 0 for infinite blocking (the default).
    ///
    /// Mirrors `std::net::TcpStream::set_read_timeout()`.
    pub fn set_read_timeout(&mut self, timeout_ms: u32) {
        self.read_timeout_ms = timeout_ms;
    }

    /// Handle suitable for registering with a Mux.
    ///
    /// Returns the DataPort's underlying shmem (or doorbell) handle.
    /// When ipd posts a CQE, this handle becomes readable.
    pub fn poll_handle(&self) -> userlib::syscall::Handle {
        self.data_port.mux_handle()
    }

    /// Free consumer pool slots (for backpressure-aware sends).
    ///
    /// Drains TX completions first to reclaim recently freed slots,
    /// then returns the number of available slots.
    pub fn pool_remaining(&mut self) -> u32 {
        self.drain_tx_completions();
        self.data_port.pool_remaining()
    }

    /// Non-blocking check for pending data.
    ///
    /// Returns true if buffered RX data or pending CQEs exist.
    /// Does NOT wait for new CQEs — only checks what's already queued.
    pub fn has_pending_data(&mut self) -> bool {
        if self.rx_pos < self.rx_len { return true; }
        if self.eof { return true; }
        self.drain_tx_completions();
        !self.pending_cqes.is_empty()
    }

    /// Returns true if the stream has reached end-of-file (peer closed).
    pub fn is_eof(&self) -> bool {
        self.eof
    }

    /// Wait until data is available to read, or timeout expires.
    ///
    /// Returns `true` if data is available (buffered or pending CQE),
    /// `false` if the timeout expired with no data.
    /// Also returns `true` if the stream is at EOF (so the caller
    /// discovers EOF via the subsequent read returning 0).
    pub fn wait_readable(&mut self, timeout_ms: u32) -> bool {
        // Already have buffered data
        if self.rx_pos < self.rx_len {
            return true;
        }
        if self.eof {
            return true;
        }
        // Drain TX completions so they don't cause false wakeups.
        // TX_DONE CQEs just free pool slots — they're not "readable data".
        // drain_tx_completions empties the entire CQ, sorting entries into
        // freed TX slots (TX_DONE) and pending_cqes (DATA/EOF/RST).
        self.drain_tx_completions();
        if !self.pending_cqes.is_empty() {
            return true;
        }
        // Wait for notification (from ipd: new data or TX_DONE)
        self.data_port.wait(timeout_ms);
        // Drain again — the wake may have been for TX_DONE only
        self.drain_tx_completions();
        !self.pending_cqes.is_empty()
    }

    /// Drain any TX completion CQEs, freeing consumer pool slots.
    ///
    /// When the provider (ipd) finishes sending data from our pool slots,
    /// it posts CQE_STREAM_TX_DONE so we can reclaim them. This must be
    /// called before writes to ensure pool slots are available.
    ///
    /// Also handles RX CQEs encountered during drain — buffers one
    /// STREAM_DATA in rx_buf if found.
    fn drain_tx_completions(&mut self) {
        loop {
            match self.data_port.poll_cq() {
                Some(cqe) => {
                    let stream_type = cqe.flags & 0x00FF;
                    if stream_type == io_status::CQE_STREAM_TX_DONE {
                        // Free our consumer pool slot
                        self.data_port.free(cqe.result);
                    } else {
                        // Non-TX CQE (STREAM_DATA, EOF, RST) — queue it for
                        // fill_rx_buf to process later.
                        self.pending_cqes.push_back(cqe);
                    }
                }
                None => break,
            }
        }
    }

    /// Fill the internal RX buffer from the DataPort's CQ.
    fn fill_rx_buf(&mut self) -> io::Result<()> {
        if self.rx_pos < self.rx_len {
            return Ok(()); // Still have buffered data
        }
        if self.eof {
            return Ok(());
        }

        // Reset buffer position
        self.rx_pos = 0;
        self.rx_len = 0;

        // Poll for a CQE from ipd (check pending first)
        let wait_ms = if self.read_timeout_ms > 0 { self.read_timeout_ms } else { 1000 };
        let mut waited = false;
        loop {
            // Check for CQEs saved by drain_tx_completions first
            let next_cqe = if let Some(cqe) = self.pending_cqes.pop_front() {
                Some(cqe)
            } else {
                self.data_port.poll_cq()
            };
            match next_cqe {
                Some(cqe) => {
                    let stream_type = cqe.flags & 0x00FF;
                    match stream_type {
                        io_status::CQE_STREAM_DATA => {
                            let offset = cqe.result;
                            let len = cqe.transferred as usize;
                            let to_copy = len.min(self.rx_buf.len());

                            if let Some(data) = self.data_port.pool_slice(offset, len as u32) {
                                self.rx_buf[..to_copy].copy_from_slice(&data[..to_copy]);
                                self.rx_len = to_copy;
                            }

                            // Tell ipd to free its provider-half allocation.
                            // (Consumer's PoolAlloc can't free provider-half offsets.)
                            let done = IoSqe {
                                opcode: io_op::STREAM_RX_DONE,
                                flags: 0,
                                priority: 0,
                                tag: 0,
                                lba: 0,
                                data_offset: offset,
                                data_len: 0,
                                param: 0,
                            };
                            self.data_port.submit(&done);
                            self.data_port.notify();
                            return Ok(());
                        }
                        io_status::CQE_STREAM_EOF => {
                            self.eof = true;
                            return Ok(());
                        }
                        io_status::CQE_STREAM_RST => {
                            self.eof = true;
                            return Err(io::Error::new(io::ErrorKind::ConnectionReset));
                        }
                        io_status::CQE_STREAM_TX_DONE => {
                            // Free our consumer pool slot (TX completion)
                            self.data_port.free(cqe.result);
                            continue;
                        }
                        _ => {
                            // Unknown CQE type — skip
                            continue;
                        }
                    }
                }
                None => {
                    // With a read timeout, return WouldBlock after waiting once
                    if waited && self.read_timeout_ms > 0 {
                        return Err(io::Error::new(io::ErrorKind::WouldBlock));
                    }
                    // No CQE available — wait for notification
                    self.data_port.wait(wait_ms);
                    waited = true;
                }
            }
        }
    }
}

impl io::Read for TcpStream {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.fill_rx_buf()?;

        if self.rx_pos >= self.rx_len {
            if self.eof {
                return Ok(0); // EOF
            }
            return Ok(0);
        }

        let available = self.rx_len - self.rx_pos;
        let to_copy = available.min(buf.len());
        buf[..to_copy].copy_from_slice(&self.rx_buf[self.rx_pos..self.rx_pos + to_copy]);
        self.rx_pos += to_copy;
        Ok(to_copy)
    }
}

impl io::Write for TcpStream {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.closed || self.eof {
            return Err(io::Error::new(io::ErrorKind::BrokenPipe));
        }

        if buf.is_empty() {
            return Ok(0);
        }

        // Drain TX completions to reclaim consumer pool slots
        self.drain_tx_completions();

        // Allocate from consumer half of pool
        let len = buf.len().min(4096) as u32;
        let offset = match self.data_port.alloc(len) {
            Some(o) => o,
            None => {
                // Pool full — wait for ipd to process our SQEs and post
                // TX_DONE CQEs so we can reclaim consumer pool slots.
                // ipd polls on a 10ms timer, so retry up to 50 times (500ms).
                let mut got = None;
                for _ in 0..50 {
                    self.data_port.wait(10);
                    self.drain_tx_completions();
                    if let Some(o) = self.data_port.alloc(len) {
                        got = Some(o);
                        break;
                    }
                }
                got.ok_or_else(|| io::Error::new(io::ErrorKind::WouldBlock))?
            }
        };

        // Write data to pool
        self.data_port.pool_write(offset, &buf[..len as usize]);

        // Submit SQE
        let sqe = IoSqe {
            opcode: io_op::STREAM_DATA,
            flags: 0,
            priority: 0,
            tag: 0,
            lba: 0,
            data_offset: offset,
            data_len: len,
            param: 0,
        };
        if !self.data_port.submit(&sqe) {
            // SQ ring full — data lost! Log this.
            userlib::uerror!("net", "sq_full"; data_len = len);
            userlib::ulog::flush();
            self.data_port.free(offset);
            return Err(io::Error::new(io::ErrorKind::WouldBlock));
        }
        self.data_port.notify();

        Ok(len as usize)
    }

    fn flush(&mut self) -> io::Result<()> {
        // TCP doesn't have an explicit flush — data is sent on next poll
        Ok(())
    }
}

impl Drop for TcpStream {
    fn drop(&mut self) {
        if !self.closed {
            // Send EOF SQE to signal we're done writing
            let sqe = IoSqe {
                opcode: io_op::STREAM_EOF,
                flags: 0,
                priority: 0,
                tag: 0,
                lba: 0,
                data_offset: 0,
                data_len: 0,
                param: 0,
            };
            let _ = self.data_port.submit(&sqe);
            let _ = self.data_port.notify();
            self.closed = true;
        }
    }
}
