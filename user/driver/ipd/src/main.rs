//! ipd — smoltcp-based IP Stack Driver
//!
//! Replaces netsvc with a real TCP/IP stack powered by smoltcp.
//! Connects as a DataPort consumer to netd (virtio-net) and provides:
//! - ARP (handled by smoltcp)
//! - ICMP echo (ping)
//! - UDP with built-in TFTP server (port 69)
//!
//! Architecture:
//!   ipd discovers net0 via devd port discovery, connects as DataPort consumer,
//!   and bridges smoltcp's Device trait to the DataPort ring. Received frames
//!   are buffered in an RxQueue, which smoltcp drains during iface.poll().
//!
//! smoltcp integration:
//!   Interface, SocketSet, and sockets are created ONCE in setup_smoltcp() and
//!   persist across all poll cycles. Buffer arrays live in separate static muts
//!   so the borrow checker is satisfied (SocketSet<'static> borrows from
//!   'static arrays). The SmolStack struct holds the Interface + SocketSet +
//!   socket handles together.

#![no_std]
#![no_main]

mod device;
mod rsh;
mod tftp;

use smoltcp::iface::{Config, Interface, SocketHandle, SocketSet, SocketStorage};
use smoltcp::socket::{dhcpv4, tcp, udp};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Address};

use userlib::bus::{BusMsg, BusError, BusCtx, Driver, Disposition, PortId, bus_msg};
use userlib::bus_runtime::driver_main;
use userlib::ipc::Timer;
use userlib::ring::{SideEntry, side_msg, side_status};
use userlib::syscall::Handle;
use userlib::{uinfo, uerror};

use device::{RxOffsetQueue, SmolDevice};
use rsh::RemoteShell;
use tftp::TftpServer;
use userlib::vfs_client::VfsClient;

// =============================================================================
// Constants
// =============================================================================

const TAG_DISCOVERY_TIMER: u32 = 1;
const TAG_POLL_TIMER: u32 = 2;
const TAG_DHCP_FALLBACK_TIMER: u32 = 3;
const DISCOVERY_INTERVAL_NS: u64 = 500_000_000;
const DHCP_FALLBACK_TIMEOUT_NS: u64 = 10_000_000_000; // 10 seconds
const NIC_PORT_NAME: &[u8] = b"net0";
const MAX_FRAME_SIZE: usize = 1514;
const STATIC_IP: Ipv4Address = Ipv4Address::new(10, 0, 2, 15);
const STATIC_GATEWAY: Ipv4Address = Ipv4Address::new(10, 0, 2, 2);
const STATIC_PREFIX_LEN: u8 = 24;
const TFTP_PORT: u16 = 69;
const HTTP_PORT: u16 = 80;
const RSH_PORT: u16 = 23;

// =============================================================================
// Static smoltcp buffer arrays
//
// These MUST be separate static muts (not fields in a struct) so that
// SocketSet<'static> can borrow them with 'static lifetime independently
// of the SmolStack that holds the SocketSet.
// =============================================================================

const SOCKET_SLOTS: usize = 5;
const UDP_META_SLOTS: usize = 8;
const UDP_BUF_SIZE: usize = 4096;
const TCP_BUF_SIZE: usize = 4096;

static mut SOCKET_STORAGE: [SocketStorage<'static>; SOCKET_SLOTS] =
    [const { SocketStorage::EMPTY }; SOCKET_SLOTS];

static mut UDP_RX_META: [udp::PacketMetadata; UDP_META_SLOTS] =
    [udp::PacketMetadata::EMPTY; UDP_META_SLOTS];
static mut UDP_RX_BUF: [u8; UDP_BUF_SIZE] = [0u8; UDP_BUF_SIZE];

static mut UDP_TX_META: [udp::PacketMetadata; UDP_META_SLOTS] =
    [udp::PacketMetadata::EMPTY; UDP_META_SLOTS];
static mut UDP_TX_BUF: [u8; UDP_BUF_SIZE] = [0u8; UDP_BUF_SIZE];

static mut TCP_RX_BUF: [u8; TCP_BUF_SIZE] = [0u8; TCP_BUF_SIZE];
static mut TCP_TX_BUF: [u8; TCP_BUF_SIZE] = [0u8; TCP_BUF_SIZE];

static mut TCP_RSH_RX_BUF: [u8; TCP_BUF_SIZE] = [0u8; TCP_BUF_SIZE];
static mut TCP_RSH_TX_BUF: [u8; TCP_BUF_SIZE] = [0u8; TCP_BUF_SIZE];

// =============================================================================
// SmolStack — persistent smoltcp state
// =============================================================================

/// Holds the smoltcp Interface, SocketSet, and socket handles.
/// Created once in setup_smoltcp(), polled on every data_ready/timer event.
struct SmolStack {
    iface: Interface,
    sockets: SocketSet<'static>,
    udp_handle: SocketHandle,
    tcp_handle: SocketHandle,
    tcp_rsh_handle: SocketHandle,
    dhcp_handle: SocketHandle,
}

/// Global smoltcp stack. Initialized once, never moved.
static mut SMOL_STACK: Option<SmolStack> = None;

// =============================================================================
// NIC State
// =============================================================================

#[derive(Clone, Copy, PartialEq, Eq)]
enum NicState {
    Empty,
    Probing,
    Up,
}

// =============================================================================
// IP Configuration State
// =============================================================================

#[derive(Clone, Copy, PartialEq, Eq)]
enum IpState {
    /// No IP address configured yet. DHCP in progress.
    Unconfigured,
    /// IP assigned by DHCP.
    DhcpConfigured,
    /// DHCP timed out, using static fallback.
    StaticFallback,
}

// =============================================================================
// ipd Driver State
// =============================================================================

struct IpdDriver {
    nic_state: NicState,
    nic_port: PortId,
    mac: [u8; 6],
    discovery_timer: Option<Timer>,
    poll_timer: Option<Timer>,
    dhcp_fallback_timer: Option<Timer>,
    discovering: bool,
    rx_queue: RxOffsetQueue,
    /// Set when NIC info arrives, cleared after setup_smoltcp runs.
    iface_ready: bool,
    ip_state: IpState,
    assigned_ip: [u8; 4],
    assigned_gateway: [u8; 4],
    assigned_prefix: u8,
    tftp: TftpServer,
    rsh: RemoteShell,
    vfs_client: Option<VfsClient>,
}

impl IpdDriver {
    const fn new() -> Self {
        Self {
            nic_state: NicState::Empty,
            nic_port: PortId(0),
            mac: [0u8; 6],
            discovery_timer: None,
            poll_timer: None,
            dhcp_fallback_timer: None,
            discovering: false,
            rx_queue: RxOffsetQueue::new(),
            iface_ready: false,
            ip_state: IpState::Unconfigured,
            assigned_ip: [0; 4],
            assigned_gateway: [0; 4],
            assigned_prefix: 0,
            tftp: TftpServer::new(),
            rsh: RemoteShell::new(),
            vfs_client: None,
        }
    }

    // =========================================================================
    // NIC Discovery
    // =========================================================================

    fn try_discover_nic(&mut self, ctx: &mut dyn BusCtx) -> bool {
        match ctx.discover_port(NIC_PORT_NAME) {
            Ok(shmem_id) => {
                uinfo!("ipd", "nic_found"; shmem_id = shmem_id);
                match ctx.connect_block_port(shmem_id) {
                    Ok(port_id) => {
                        self.nic_port = port_id;
                        self.nic_state = NicState::Probing;
                        uinfo!("ipd", "nic_connected";);
                        self.send_nic_query(ctx);
                        true
                    }
                    Err(_) => {
                        uerror!("ipd", "connect_failed";);
                        false
                    }
                }
            }
            Err(_) => false,
        }
    }

    fn send_nic_query(&mut self, ctx: &mut dyn BusCtx) {
        let request = SideEntry {
            msg_type: side_msg::QUERY_INFO,
            flags: 0,
            tag: 0,
            status: side_status::REQUEST,
            payload: [0; 24],
        };
        if let Some(port) = ctx.block_port(self.nic_port) {
            port.side_send(&request);
            port.notify();
        }
    }

    fn handle_side_response(&mut self, response: &SideEntry) {
        if response.msg_type != side_msg::QUERY_INFO || response.status != side_status::OK {
            return;
        }
        if self.nic_state != NicState::Probing {
            return;
        }

        self.mac.copy_from_slice(&response.payload[0..6]);
        let mtu = u16::from_le_bytes([response.payload[7], response.payload[8]]);
        uinfo!("ipd", "nic_info";
            mac0 = self.mac[0] as u32,
            mac1 = self.mac[1] as u32,
            mac2 = self.mac[2] as u32,
            mac3 = self.mac[3] as u32,
            mac4 = self.mac[4] as u32,
            mac5 = self.mac[5] as u32,
            mtu = mtu as u32);

        self.nic_state = NicState::Up;
        self.iface_ready = true;
        uinfo!("ipd", "nic_up"; dhcp = "starting");
    }

    // =========================================================================
    // smoltcp Setup (one-time initialization)
    // =========================================================================

    /// Create the smoltcp Interface, SocketSet, and UDP socket.
    /// Called exactly once after the NIC MAC address is known.
    ///
    /// SAFETY: All static mut buffer arrays are only accessed here (for
    /// initial borrow) and never touched again. ipd is single-threaded.
    fn setup_smoltcp(&mut self, ctx: &mut dyn BusCtx) {
        let hw_addr = HardwareAddress::Ethernet(EthernetAddress(self.mac));
        let config = Config::new(hw_addr);
        let now = Self::smoltcp_now();

        // Create Interface — start with NO IP address; DHCP will assign one.
        let iface = if let Some(port) = ctx.block_port(self.nic_port) {
            let mut device = SmolDevice::new(port, &mut self.rx_queue);
            Interface::new(config, &mut device, now)
        } else {
            uerror!("ipd", "setup_failed"; reason = "no block port");
            return;
        };

        // SAFETY: ipd is single-threaded. These statics are borrowed once here
        // and the resulting SocketSet<'static> is stored in SMOL_STACK. The
        // buffer arrays are never accessed directly again.
        let (socket_storage, rx_meta, rx_buf, tx_meta, tx_buf, tcp_rx, tcp_tx,
             tcp_rsh_rx, tcp_rsh_tx) = unsafe {
            (
                &mut *(&raw mut SOCKET_STORAGE),
                &mut *(&raw mut UDP_RX_META),
                &mut *(&raw mut UDP_RX_BUF),
                &mut *(&raw mut UDP_TX_META),
                &mut *(&raw mut UDP_TX_BUF),
                &mut *(&raw mut TCP_RX_BUF),
                &mut *(&raw mut TCP_TX_BUF),
                &mut *(&raw mut TCP_RSH_RX_BUF),
                &mut *(&raw mut TCP_RSH_TX_BUF),
            )
        };

        // Create SocketSet
        let mut sockets = SocketSet::new(&mut socket_storage[..]);

        // Create and bind UDP socket for TFTP
        let udp_rx = udp::PacketBuffer::new(&mut rx_meta[..], &mut rx_buf[..]);
        let udp_tx = udp::PacketBuffer::new(&mut tx_meta[..], &mut tx_buf[..]);
        let mut udp_socket = udp::Socket::new(udp_rx, udp_tx);
        let _ = udp_socket.bind(TFTP_PORT);
        let udp_handle = sockets.add(udp_socket);

        // Create TCP socket for HTTP server
        let tcp_rx_buf = tcp::SocketBuffer::new(&mut tcp_rx[..]);
        let tcp_tx_buf = tcp::SocketBuffer::new(&mut tcp_tx[..]);
        let mut tcp_socket = tcp::Socket::new(tcp_rx_buf, tcp_tx_buf);
        tcp_socket.listen(HTTP_PORT).ok();
        let tcp_handle = sockets.add(tcp_socket);

        // Create TCP socket for remote shell (port 23)
        let rsh_rx_buf = tcp::SocketBuffer::new(&mut tcp_rsh_rx[..]);
        let rsh_tx_buf = tcp::SocketBuffer::new(&mut tcp_rsh_tx[..]);
        let mut rsh_socket = tcp::Socket::new(rsh_rx_buf, rsh_tx_buf);
        rsh_socket.listen(RSH_PORT).ok();
        let tcp_rsh_handle = sockets.add(rsh_socket);

        // Create DHCP socket
        let dhcp_socket = dhcpv4::Socket::new();
        let dhcp_handle = sockets.add(dhcp_socket);

        // Store persistent stack
        // SAFETY: SMOL_STACK is only written here (once) and read in poll_smoltcp.
        unsafe {
            SMOL_STACK = Some(SmolStack {
                iface,
                sockets,
                udp_handle,
                tcp_handle,
                tcp_rsh_handle,
                dhcp_handle,
            });
        }

        // Arm the DHCP fallback timer — if no DHCP response in 10s, use static IP
        self.arm_dhcp_fallback_timer(ctx);

        uinfo!("ipd", "smoltcp_ready"; sockets = SOCKET_SLOTS as u32);
        uinfo!("ipd", "dhcp_start";);
    }

    // =========================================================================
    // Timers
    // =========================================================================

    fn smoltcp_now() -> Instant {
        let ns = userlib::syscall::gettime();
        Instant::from_millis((ns / 1_000_000) as i64)
    }

    fn arm_discovery_timer(&mut self, ctx: &mut dyn BusCtx) {
        if let Some(ref mut timer) = self.discovery_timer {
            let _ = timer.set(DISCOVERY_INTERVAL_NS);
        } else if let Ok(mut timer) = Timer::new() {
            let _ = timer.set(DISCOVERY_INTERVAL_NS);
            let handle = timer.handle();
            let _ = ctx.watch_handle(handle, TAG_DISCOVERY_TIMER);
            self.discovery_timer = Some(timer);
        }
    }

    fn arm_poll_timer(&mut self, delay_ms: Option<u64>, ctx: &mut dyn BusCtx) {
        let ns = match delay_ms {
            Some(ms) if ms > 0 => ms * 1_000_000,
            _ => 1_000_000_000, // Default 1s
        };

        if let Some(ref mut timer) = self.poll_timer {
            let _ = timer.set(ns);
        } else if let Ok(mut timer) = Timer::new() {
            let _ = timer.set(ns);
            let handle = timer.handle();
            let _ = ctx.watch_handle(handle, TAG_POLL_TIMER);
            self.poll_timer = Some(timer);
        }
    }

    fn arm_dhcp_fallback_timer(&mut self, ctx: &mut dyn BusCtx) {
        if let Ok(mut timer) = Timer::new() {
            let _ = timer.set(DHCP_FALLBACK_TIMEOUT_NS);
            let handle = timer.handle();
            let _ = ctx.watch_handle(handle, TAG_DHCP_FALLBACK_TIMER);
            self.dhcp_fallback_timer = Some(timer);
        }
    }

    /// Apply an IP configuration to the smoltcp interface and update driver state.
    fn apply_ip_config(&mut self, ip: Ipv4Address, prefix: u8, gateway: Ipv4Address, state: IpState) {
        let ip_bytes = ip.octets();
        let gw_bytes = gateway.octets();

        self.ip_state = state;
        self.assigned_ip = ip_bytes;
        self.assigned_gateway = gw_bytes;
        self.assigned_prefix = prefix;

        // Update remote shell with current IP info
        self.rsh.ip = ip_bytes;
        self.rsh.ip_source = match state {
            IpState::DhcpConfigured => 1,
            IpState::StaticFallback => 2,
            IpState::Unconfigured => 0,
        };

        // SAFETY: single-threaded, SMOL_STACK initialized before this is called.
        if let Some(stack) = unsafe { &mut *(&raw mut SMOL_STACK) } {
            stack.iface.update_ip_addrs(|addrs| {
                addrs.clear();
                let _ = addrs.push(IpCidr::new(IpAddress::Ipv4(ip), prefix));
            });
            stack.iface.routes_mut().add_default_ipv4_route(gateway).ok();
        }

        let state_str = match state {
            IpState::DhcpConfigured => "dhcp",
            IpState::StaticFallback => "static_fallback",
            IpState::Unconfigured => "none",
        };
        uinfo!("ipd", "ip_configured";
            ip0 = ip_bytes[0] as u32,
            ip1 = ip_bytes[1] as u32,
            ip2 = ip_bytes[2] as u32,
            ip3 = ip_bytes[3] as u32,
            prefix = prefix as u32,
            source = state_str);
    }

    /// Apply the static fallback IP when DHCP times out.
    fn apply_static_fallback(&mut self) {
        if self.ip_state != IpState::Unconfigured {
            return;
        }
        self.apply_ip_config(STATIC_IP, STATIC_PREFIX_LEN, STATIC_GATEWAY, IpState::StaticFallback);
    }

    // =========================================================================
    // DataPort I/O
    // =========================================================================

    /// Drain CQEs from the NIC DataPort into the RX offset queue.
    ///
    /// Zero-copy: only stores (offset, len) pairs — frame data stays in pool.
    fn drain_rx(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = self.nic_port;
        if let Some(port) = ctx.block_port(port_id) {
            for _ in 0..16 {
                if let Some(completion) = port.poll_completion() {
                    let offset = completion.result;
                    let len = completion.transferred;
                    if len > 0 && len <= MAX_FRAME_SIZE as u32 {
                        self.rx_queue.push(offset, len);
                    }
                } else {
                    break;
                }
            }
        }
    }

    /// Drain sidechannel responses from the NIC.
    fn drain_side(&mut self, ctx: &mut dyn BusCtx) {
        let port_id = self.nic_port;
        let mut responses: [Option<SideEntry>; 4] = [None; 4];
        let mut count = 0;

        if let Some(port) = ctx.block_port(port_id) {
            while count < 4 {
                if let Some(resp) = port.poll_side_response() {
                    responses[count] = Some(resp);
                    count += 1;
                } else {
                    break;
                }
            }
        }

        for i in 0..count {
            if let Some(ref resp) = responses[i] {
                self.handle_side_response(resp);
            }
        }
    }

    // =========================================================================
    // smoltcp Poll (called on every data_ready / timer event)
    // =========================================================================

    /// Run one smoltcp poll cycle using the persistent SmolStack.
    fn poll_smoltcp(&mut self, ctx: &mut dyn BusCtx) {
        // SAFETY: SMOL_STACK is initialized in setup_smoltcp() and only
        // accessed here. ipd is single-threaded.
        let stack = match unsafe { &mut *(&raw mut SMOL_STACK) } {
            Some(s) => s,
            None => return,
        };

        let now = Self::smoltcp_now();

        // Poll the interface (processes RX queue, handles ARP/ICMP)
        if let Some(port) = ctx.block_port(self.nic_port) {
            let mut device = SmolDevice::new(port, &mut self.rx_queue);
            stack.iface.poll(now, &mut device, &mut stack.sockets);
        }

        // Poll DHCP socket for configuration events
        {
            let event = stack.sockets.get_mut::<dhcpv4::Socket>(stack.dhcp_handle).poll();
            match event {
                Some(dhcpv4::Event::Configured(config)) => {
                    let ip = config.address.address();
                    let prefix = config.address.prefix_len();
                    let gateway = config.router.unwrap_or(STATIC_GATEWAY);
                    self.apply_ip_config(ip, prefix, gateway, IpState::DhcpConfigured);

                    // Cancel the fallback timer
                    if let Some(ref timer) = self.dhcp_fallback_timer {
                        let _ = ctx.unwatch_handle(timer.handle());
                    }
                    self.dhcp_fallback_timer = None;
                }
                Some(dhcpv4::Event::Deconfigured) => {
                    uinfo!("ipd", "dhcp_deconfigured";);
                    self.ip_state = IpState::Unconfigured;
                    self.assigned_ip = [0; 4];
                    self.assigned_gateway = [0; 4];
                    self.assigned_prefix = 0;

                    // Clear IP from interface
                    stack.iface.update_ip_addrs(|addrs| { addrs.clear(); });
                }
                None => {}
            }
        }

        // Process TFTP socket (UDP)
        {
            let sock = stack.sockets.get_mut::<udp::Socket>(stack.udp_handle);
            while let Ok((data, meta)) = sock.recv() {
                let remote = meta.endpoint;
                let mut resp_buf = [0u8; 516]; // 4 header + 512 data
                let resp_len = self.tftp.handle(data, remote, &mut resp_buf, &mut self.vfs_client);
                if resp_len > 0 {
                    let _ = sock.send_slice(&resp_buf[..resp_len], remote);
                }
            }
        }

        // Process HTTP socket (TCP)
        let ip_source = match self.ip_state {
            IpState::DhcpConfigured => "dhcp",
            IpState::StaticFallback => "static",
            IpState::Unconfigured => "none",
        };
        Self::process_http(&mut stack.sockets, stack.tcp_handle, self.assigned_ip, ip_source);

        // Process remote shell socket (TCP port 23)
        self.rsh.process(&mut stack.sockets, stack.tcp_rsh_handle);

        // Re-poll after socket processing (smoltcp may have TX to send)
        if let Some(port) = ctx.block_port(self.nic_port) {
            let mut device = SmolDevice::new(port, &mut self.rx_queue);
            stack.iface.poll(now, &mut device, &mut stack.sockets);
        }

        // Schedule next poll based on smoltcp's delay
        let delay = stack
            .iface
            .poll_delay(now, &stack.sockets)
            .map(|d| d.total_millis() as u64);
        self.arm_poll_timer(delay, ctx);
    }

    // =========================================================================
    // HTTP Server (TCP port 80)
    // =========================================================================

    /// Process the HTTP TCP socket: read request, send response, re-listen.
    fn process_http(sockets: &mut SocketSet<'static>, handle: SocketHandle, ip: [u8; 4], ip_source: &str) {
        let sock = sockets.get_mut::<tcp::Socket>(handle);

        if !sock.is_active() && !sock.is_listening() {
            // Connection closed — re-listen for next connection
            sock.listen(HTTP_PORT).ok();
            return;
        }

        if !sock.may_recv() {
            return;
        }

        // Read the request into a small buffer (we only need the first line)
        let mut req_buf = [0u8; 512];
        let mut req_len = 0;
        if let Ok(n) = sock.peek_slice(&mut req_buf) {
            req_len = n;
        }

        if req_len == 0 {
            return;
        }

        // Wait until we see the end of HTTP headers (\r\n\r\n)
        if !contains_header_end(&req_buf[..req_len]) {
            return;
        }

        // Consume the request data
        sock.recv(|data| (data.len(), ()));

        // Build response
        let mut resp = [0u8; 2048];
        let resp_len = build_http_response(&req_buf[..req_len], &mut resp, ip, ip_source);

        // Send response
        if resp_len > 0 {
            let _ = sock.send_slice(&resp[..resp_len]);
        }

        // Close our end after sending
        sock.close();
    }
}

// =============================================================================
// HTTP Response Builder
// =============================================================================

/// Check if the buffer contains the end-of-headers marker.
fn contains_header_end(data: &[u8]) -> bool {
    for i in 0..data.len().saturating_sub(3) {
        if data[i] == b'\r' && data[i + 1] == b'\n' && data[i + 2] == b'\r' && data[i + 3] == b'\n'
        {
            return true;
        }
    }
    false
}

/// Build an HTTP/1.0 response with system status page.
fn build_http_response(_request: &[u8], out: &mut [u8], ip: [u8; 4], ip_source: &str) -> usize {
    let uptime_ns = userlib::syscall::gettime();
    let uptime_s = uptime_ns / 1_000_000_000;

    // Build body
    let mut body = [0u8; 1024];
    let mut bpos = 0;

    bpos += copy_str(&mut body[bpos..], "BPI-R4 ipd status\n");
    bpos += copy_str(&mut body[bpos..], "==================\n");
    bpos += copy_str(&mut body[bpos..], "IP:     ");
    bpos += format_ip(&mut body[bpos..], ip);
    bpos += copy_str(&mut body[bpos..], " (");
    bpos += copy_str(&mut body[bpos..], ip_source);
    bpos += copy_str(&mut body[bpos..], ")\n");
    bpos += copy_str(&mut body[bpos..], "Uptime: ");
    bpos += format_u64(uptime_s, &mut body[bpos..]);
    bpos += copy_str(&mut body[bpos..], "s\n");
    bpos += copy_str(&mut body[bpos..], "Stack:  smoltcp 0.12\n");
    bpos += copy_str(&mut body[bpos..], "TCP:    port 80 (http)\n");
    bpos += copy_str(&mut body[bpos..], "UDP:    port 69 (tftp)\n");

    // Ramfs listing
    let mut entries = [userlib::syscall::RamfsListEntry::empty(); 32];
    let count = userlib::syscall::ramfs_list(&mut entries);
    if count > 0 {
        bpos += copy_str(&mut body[bpos..], "\nRamfs (/bin):\n");
        for i in 0..count as usize {
            let name = entries[i].name_str();
            let size = entries[i].size as u64;
            bpos += copy_str(&mut body[bpos..], "  ");
            bpos += copy_bytes(&mut body[bpos..], name);
            bpos += copy_str(&mut body[bpos..], "  ");
            bpos += format_u64(size, &mut body[bpos..]);
            bpos += copy_str(&mut body[bpos..], "\n");
            if bpos > 900 {
                break;
            }
        }
    }

    // Build HTTP header + body
    let mut pos = 0;
    pos += copy_str(&mut out[pos..], "HTTP/1.0 200 OK\r\n");
    pos += copy_str(&mut out[pos..], "Content-Type: text/plain\r\n");
    pos += copy_str(&mut out[pos..], "Connection: close\r\n");
    pos += copy_str(&mut out[pos..], "Content-Length: ");
    pos += format_u64(bpos as u64, &mut out[pos..]);
    pos += copy_str(&mut out[pos..], "\r\n\r\n");
    let remaining = out.len() - pos;
    let copy_len = bpos.min(remaining);
    out[pos..pos + copy_len].copy_from_slice(&body[..copy_len]);
    pos += copy_len;

    pos
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

fn format_ip(buf: &mut [u8], ip: [u8; 4]) -> usize {
    let mut pos = 0;
    for i in 0..4 {
        if i > 0 {
            pos += copy_str(&mut buf[pos..], ".");
        }
        pos += format_u64(ip[i] as u64, &mut buf[pos..]);
    }
    pos
}

fn format_u64(mut val: u64, buf: &mut [u8]) -> usize {
    if val == 0 {
        if !buf.is_empty() {
            buf[0] = b'0';
        }
        return 1;
    }
    let mut tmp = [0u8; 20];
    let mut i = 0;
    while val > 0 {
        tmp[i] = b'0' + (val % 10) as u8;
        val /= 10;
        i += 1;
    }
    let len = i.min(buf.len());
    for j in 0..len {
        buf[j] = tmp[i - 1 - j];
    }
    len
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for IpdDriver {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        uinfo!("ipd", "starting";);

        if self.try_discover_nic(ctx) {
            self.discovering = false;
            uinfo!("ipd", "nic_probing";);
        } else {
            self.discovering = true;
            self.arm_discovery_timer(ctx);
            uinfo!("ipd", "waiting_for_nic";);
        }

        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::QUERY_INFO => {
                let mut info = [0u8; 64];
                let prefix = b"ipd: ";
                info[..prefix.len()].copy_from_slice(prefix);
                let mut pos = prefix.len();

                let suffix = if self.nic_state == NicState::Up {
                    b"up, smoltcp" as &[u8]
                } else {
                    b"no NIC" as &[u8]
                };
                let slen = suffix.len().min(64 - pos);
                info[pos..pos + slen].copy_from_slice(&suffix[..slen]);
                pos += slen;

                let _ = ctx.respond_info(msg.seq_id, &info[..pos]);
                Disposition::Handled
            }
            _ => Disposition::Forward,
        }
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        if port != self.nic_port {
            return;
        }

        // Drain sidechannel first (may transition NIC to Up)
        self.drain_side(ctx);

        // If NIC just came up, initialize smoltcp (once)
        if self.iface_ready {
            let stack_exists = unsafe { (*(&raw const SMOL_STACK)).is_some() };
            if !stack_exists {
                self.setup_smoltcp(ctx);
            }
            self.iface_ready = false;
        }

        if self.nic_state != NicState::Up {
            return;
        }

        self.drain_rx(ctx);
        self.poll_smoltcp(ctx);
    }

    fn handle_event(&mut self, tag: u32, _handle: Handle, ctx: &mut dyn BusCtx) {
        match tag {
            TAG_DISCOVERY_TIMER if self.discovering => {
                if let Some(ref mut timer) = self.discovery_timer {
                    let _ = timer.wait();
                }

                if self.try_discover_nic(ctx) {
                    self.discovering = false;
                    if let Some(ref timer) = self.discovery_timer {
                        let _ = ctx.unwatch_handle(timer.handle());
                    }
                    self.discovery_timer = None;
                    uinfo!("ipd", "nic_probing";);
                } else {
                    self.arm_discovery_timer(ctx);
                }
            }
            TAG_POLL_TIMER => {
                if let Some(ref mut timer) = self.poll_timer {
                    let _ = timer.wait();
                }

                if self.nic_state == NicState::Up {
                    self.drain_rx(ctx);
                    self.poll_smoltcp(ctx);
                }
            }
            TAG_DHCP_FALLBACK_TIMER => {
                if let Some(ref mut timer) = self.dhcp_fallback_timer {
                    let _ = timer.wait();
                }
                self.apply_static_fallback();
                // Clean up the one-shot timer
                if let Some(ref timer) = self.dhcp_fallback_timer {
                    let _ = ctx.unwatch_handle(timer.handle());
                }
                self.dhcp_fallback_timer = None;
            }
            _ => {}
        }
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: IpdDriver = IpdDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"ipd", IpdDriverWrapper(driver));
}

struct IpdDriverWrapper(&'static mut IpdDriver);

impl Driver for IpdDriverWrapper {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.init(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        self.0.data_ready(port, ctx)
    }

    fn handle_event(&mut self, tag: u32, handle: Handle, ctx: &mut dyn BusCtx) {
        self.0.handle_event(tag, handle, ctx)
    }
}
