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

extern crate alloc;

mod device;
mod dhcp_server;
mod rsh;
mod socket_svc;
mod tftp;

use smoltcp::iface::{Config, Interface, SocketHandle, SocketSet, SocketStorage};
use smoltcp::socket::{dhcpv4, tcp, udp};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Address};

use userlib::bus::{BusMsg, BusError, BusCtx, Driver, Disposition, PortId, ConfigKey, bus_msg};
use userlib::bus_runtime::driver_main;
use userlib::ipc::Timer;
use userlib::ring::{SideEntry, side_msg, side_status};
use userlib::syscall::Handle;
use userlib::{uinfo, udebug, uerror};

use device::{RxOffsetQueue, SmolDevice, TxTracker, DataPathStats};
use rsh::RemoteShell;
use dhcp_server::DhcpServer;
use socket_svc::SocketService;
use tftp::TftpServer;
use userlib::vfs_client::VfsClient;

// =============================================================================
// Constants
// =============================================================================

const TAG_DISCOVERY_TIMER: u32 = 1;
const TAG_POLL_TIMER: u32 = 2;
const TAG_DHCP_FALLBACK_TIMER: u32 = 3;
const TAG_RSH_SHELL_RESPONSE: u32 = 4;
const TAG_SOCKET_SVC_PORT: u32 = 5;
/// Socket service client channels: TAG_SOCKET_CLIENT_BASE + slot_index
const TAG_SOCKET_CLIENT_BASE: u32 = 0x100;
/// Socket service DataPort wake: TAG_SOCKET_DATA_BASE + slot_index
const TAG_SOCKET_DATA_BASE: u32 = 0x200;
const DISCOVERY_INTERVAL_NS: u64 = 500_000_000;
const DHCP_FALLBACK_TIMEOUT_NS: u64 = 10_000_000_000; // 10 seconds
/// Coalescing window: drain all pending RX/TX CQEs, then poll smoltcp once.
/// 2ms balances latency (interactive SSH) vs. batching (throughput).
const COALESCE_MS: u64 = 2;
const NIC_PORT_NAMES: &[&[u8]] = &[b"wifi:0"];
use crate::device::MAX_FRAME_SIZE;
/// AP mode: ipd IS the gateway — use a private subnet.
const AP_IP: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);
const AP_PREFIX_LEN: u8 = 24;
/// Client/fallback IP when DHCP fails (QEMU virtio-net, etc.)
const STATIC_IP: Ipv4Address = Ipv4Address::new(10, 0, 2, 15);
const STATIC_GATEWAY: Ipv4Address = Ipv4Address::new(10, 0, 2, 2);
const STATIC_PREFIX_LEN: u8 = 24;
const TFTP_PORT: u16 = 69;
const HTTP_PORT: u16 = 80;
const RSH_PORT: u16 = 23;
const ECHO_PORT: u16 = 7;

// =============================================================================
// Static smoltcp buffer arrays
//
// These MUST be separate static muts (not fields in a struct) so that
// SocketSet<'static> can borrow them with 'static lifetime independently
// of the SmolStack that holds the SocketSet.
// =============================================================================

/// 7 base sockets + 8 socket service connections
const SOCKET_SLOTS: usize = 7 + SocketService::extra_socket_slots();
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

static mut DHCP_SRV_RX_META: [udp::PacketMetadata; UDP_META_SLOTS] =
    [udp::PacketMetadata::EMPTY; UDP_META_SLOTS];
static mut DHCP_SRV_RX_BUF: [u8; UDP_BUF_SIZE] = [0u8; UDP_BUF_SIZE];
static mut DHCP_SRV_TX_META: [udp::PacketMetadata; UDP_META_SLOTS] =
    [udp::PacketMetadata::EMPTY; UDP_META_SLOTS];
static mut DHCP_SRV_TX_BUF: [u8; UDP_BUF_SIZE] = [0u8; UDP_BUF_SIZE];

// TCP echo (port 7) — 16KB buffers to match socket_svc sizing
const TCP_ECHO_BUF_SIZE: usize = 16384;
static mut TCP_ECHO_RX_BUF: [u8; TCP_ECHO_BUF_SIZE] = [0u8; TCP_ECHO_BUF_SIZE];
static mut TCP_ECHO_TX_BUF: [u8; TCP_ECHO_BUF_SIZE] = [0u8; TCP_ECHO_BUF_SIZE];

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
    /// None in AP mode (no DHCP client needed).
    dhcp_handle: Option<SocketHandle>,
    /// DHCP server socket (AP mode only, port 67).
    dhcp_srv_handle: Option<SocketHandle>,
    /// TCP echo socket (port 7) — pure recv→send, no DataPort.
    tcp_echo_handle: SocketHandle,
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
    /// Manually configured via CONFIG SET.
    StaticConfigured,
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
    dhcp_server: DhcpServer,
    rsh: RemoteShell,
    vfs_client: Option<VfsClient>,
    /// Currently watched rsh shell response handle (for unwatch on completion).
    rsh_watched_handle: Option<Handle>,
    /// Bridge group ID for CQE demux (set from spawning port name).
    /// ethd tags each RX CQE with the source port's group_id in IoCqe.flags.
    /// This ipd instance only processes frames matching its group_id.
    group_id: u8,
    /// True when connected to a WiFi AP port (wifi:*). In AP mode ipd uses
    /// a static IP and does NOT run DHCP client (no upstream to discover from).
    ap_mode: bool,
    /// TX pool offset tracker — maps SQE tags to pool offsets for reclaim.
    tx_tracker: TxTracker,
    /// Data path statistics.
    stats: DataPathStats,
    /// CQEs peeked but not yet acked. Ack deferred until after smoltcp
    /// consumes RX frames, so the provider doesn't reclaim pool slots early.
    pending_acks: u32,
    /// Diagnostic: first 20 bytes of last RX frame (for `devc ipd get diag.rx`)
    last_rx_hdr: [u8; 20],
    last_rx_len: u32,
    last_rx_offset: u32,
    pool_base_diag: u64,
    /// Socket service: exposes TCP sockets to external processes via DataPorts.
    socket_svc: SocketService,
    /// Tracked DataPort handles for socket service connections.
    /// When a connection is established, its DataPort is watched in the Mux.
    /// When it closes, the watch is removed. This enables immediate wake on
    /// client TX instead of waiting for the 10ms poll timer.
    socket_data_watched: [bool; 8],
    /// True when the poll timer is armed at COALESCE_MS, awaiting a deferred
    /// poll_smoltcp(). Set by arm_coalesce(), cleared by TAG_POLL_TIMER handler.
    coalescing: bool,
    /// Rate-limiter for echo state logging.
    echo_log_counter: u32,
    /// Poll cycle counter for diagnostics.
    poll_count: u32,
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
            dhcp_server: DhcpServer::new(AP_IP, Ipv4Address::new(255, 255, 255, 0)),
            rsh: RemoteShell::new(),
            vfs_client: None,
            rsh_watched_handle: None,
            group_id: 0,
            ap_mode: false,
            tx_tracker: TxTracker::new(),
            stats: DataPathStats::new(),
            pending_acks: 0,
            last_rx_hdr: [0u8; 20],
            last_rx_len: 0,
            last_rx_offset: 0,
            pool_base_diag: 0,
            socket_svc: SocketService::new(),
            socket_data_watched: [false; 8],
            coalescing: false,
            echo_log_counter: 0,
            poll_count: 0,
        }
    }

    // =========================================================================
    // NIC Discovery
    // =========================================================================

    fn try_discover_nic(&mut self, ctx: &mut dyn BusCtx) -> bool {
        // Fast path: use trigger port shmem_id from spawn context (works in tree mode)
        if let Ok(shmem_id) = ctx.discover_port() {
            udebug!("ipd", "nic_found_ctx"; shmem_id = shmem_id);
            // In tree mode we're spawned by the wifi driver → AP mode
            self.ap_mode = true;
            return self.connect_nic(shmem_id, ctx);
        }
        // Fallback: query devd by name (works in root mode)
        for name in NIC_PORT_NAMES {
            if let Ok(shmem_id) = ctx.discover_port_by_name(name) {
                udebug!("ipd", "nic_found"; shmem_id = shmem_id);
                // wifi:* ports are AP mode, net:* ports would be client mode
                self.ap_mode = name.starts_with(b"wifi:");
                return self.connect_nic(shmem_id, ctx);
            }
        }
        false
    }

    fn connect_nic(&mut self, shmem_id: u32, ctx: &mut dyn BusCtx) -> bool {
        match ctx.connect_block_port(shmem_id) {
            Ok(port_id) => {
                self.nic_port = port_id;
                self.nic_state = NicState::Probing;
                udebug!("ipd", "nic_connected";);
                self.send_nic_query(ctx);
                true
            }
            Err(_) => {
                uerror!("ipd", "connect_failed";);
                false
            }
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
        udebug!("ipd", "nic_info";
            mac0 = self.mac[0] as u32,
            mac1 = self.mac[1] as u32,
            mac2 = self.mac[2] as u32,
            mac3 = self.mac[3] as u32,
            mac4 = self.mac[4] as u32,
            mac5 = self.mac[5] as u32,
            mtu = mtu as u32);

        self.nic_state = NicState::Up;
        self.iface_ready = true;
        udebug!("ipd", "nic_up"; dhcp = "starting");
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
            let mut device = SmolDevice::new(port, &mut self.rx_queue, &mut self.tx_tracker, &mut self.stats, self.group_id);
            Interface::new(config, &mut device, now)
        } else {
            uerror!("ipd", "setup_failed"; reason = "no block port");
            return;
        };

        // SAFETY: ipd is single-threaded. These statics are borrowed once here
        // and the resulting SocketSet<'static> is stored in SMOL_STACK. The
        // buffer arrays are never accessed directly again.
        let (socket_storage, rx_meta, rx_buf, tx_meta, tx_buf, tcp_rx, tcp_tx,
             tcp_rsh_rx, tcp_rsh_tx,
             dhcp_srv_rx_meta, dhcp_srv_rx_buf, dhcp_srv_tx_meta, dhcp_srv_tx_buf,
             tcp_echo_rx, tcp_echo_tx) = unsafe {
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
                &mut *(&raw mut DHCP_SRV_RX_META),
                &mut *(&raw mut DHCP_SRV_RX_BUF),
                &mut *(&raw mut DHCP_SRV_TX_META),
                &mut *(&raw mut DHCP_SRV_TX_BUF),
                &mut *(&raw mut TCP_ECHO_RX_BUF),
                &mut *(&raw mut TCP_ECHO_TX_BUF),
            )
        };

        // Create SocketSet
        let mut sockets = SocketSet::new(&mut socket_storage[..]);

        // Create and bind UDP socket for TFTP
        let udp_rx = udp::PacketBuffer::new(&mut rx_meta[..], &mut rx_buf[..]);
        let udp_tx = udp::PacketBuffer::new(&mut tx_meta[..], &mut tx_buf[..]);
        let mut udp_socket = udp::Socket::new(udp_rx, udp_tx);
        if udp_socket.bind(TFTP_PORT).is_err() {
            uerror!("ipd", "udp_bind_failed"; port = TFTP_PORT as u32);
        }
        let udp_handle = sockets.add(udp_socket);

        // Create TCP socket for HTTP server
        let tcp_rx_buf = tcp::SocketBuffer::new(&mut tcp_rx[..]);
        let tcp_tx_buf = tcp::SocketBuffer::new(&mut tcp_tx[..]);
        let mut tcp_socket = tcp::Socket::new(tcp_rx_buf, tcp_tx_buf);
        if tcp_socket.listen(HTTP_PORT).is_err() {
            uerror!("ipd", "tcp_listen_failed"; port = HTTP_PORT as u32);
        }
        let tcp_handle = sockets.add(tcp_socket);

        // Create TCP socket for remote shell (port 23)
        let rsh_rx_buf = tcp::SocketBuffer::new(&mut tcp_rsh_rx[..]);
        let rsh_tx_buf = tcp::SocketBuffer::new(&mut tcp_rsh_tx[..]);
        let mut rsh_socket = tcp::Socket::new(rsh_rx_buf, rsh_tx_buf);
        if rsh_socket.listen(RSH_PORT).is_err() {
            uerror!("ipd", "tcp_listen_failed"; port = RSH_PORT as u32);
        }
        let tcp_rsh_handle = sockets.add(rsh_socket);

        // Create DHCP socket (only in client mode — AP has no upstream)
        let dhcp_handle = if !self.ap_mode {
            let dhcp_socket = dhcpv4::Socket::new();
            Some(sockets.add(dhcp_socket))
        } else {
            None
        };

        // Create DHCP server socket (AP mode only — serve IPs to clients)
        let dhcp_srv_handle = if self.ap_mode {
            let srv_rx = udp::PacketBuffer::new(&mut dhcp_srv_rx_meta[..], &mut dhcp_srv_rx_buf[..]);
            let srv_tx = udp::PacketBuffer::new(&mut dhcp_srv_tx_meta[..], &mut dhcp_srv_tx_buf[..]);
            let mut srv_socket = udp::Socket::new(srv_rx, srv_tx);
            if srv_socket.bind(67).is_err() {
                uerror!("ipd", "dhcp_srv_bind_failed"; port = 67u32);
            }
            Some(sockets.add(srv_socket))
        } else {
            None
        };

        // Create TCP echo socket (port 7) — pure recv→send for network testing
        let echo_rx = tcp::SocketBuffer::new(&mut tcp_echo_rx[..]);
        let echo_tx = tcp::SocketBuffer::new(&mut tcp_echo_tx[..]);
        let mut echo_socket = tcp::Socket::new(echo_rx, echo_tx);
        echo_socket.set_nagle_enabled(false);
        if echo_socket.listen(ECHO_PORT).is_err() {
            uerror!("ipd", "tcp_listen_failed"; port = ECHO_PORT as u32);
        }
        let tcp_echo_handle = sockets.add(echo_socket);

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
                dhcp_srv_handle,
                tcp_echo_handle,
            });
        }

        if self.ap_mode {
            // AP mode: use static IP immediately, no DHCP
            self.apply_ip_config(AP_IP, AP_PREFIX_LEN, AP_IP, IpState::StaticConfigured);
            uinfo!("ipd", "ap_mode"; ip = "192.168.4.1/24");
        } else {
            // Client mode: arm DHCP fallback timer
            self.arm_dhcp_fallback_timer(ctx);
            udebug!("ipd", "dhcp_start";);
        }

        // Initialize socket service (register "tcp:" port)
        self.socket_svc.init();
        if let Some(handle) = self.socket_svc.port_handle() {
            if ctx.watch_handle(handle, TAG_SOCKET_SVC_PORT).is_err() {
                uerror!("ipd", "watch_failed"; tag = "socket_svc");
            }
        }

        udebug!("ipd", "smoltcp_ready"; sockets = SOCKET_SLOTS as u32);

        // Start the poll timer — drives smoltcp housekeeping (retransmissions,
        // keepalives, delayed ACKs). Re-armed only in TAG_POLL_TIMER handler.
        self.arm_poll_timer(None, ctx);
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
            if timer.set(DISCOVERY_INTERVAL_NS).is_err() {
                uerror!("ipd", "timer_set_failed"; tag = "discovery");
            }
        } else {
            match Timer::new() {
                Ok(mut timer) => {
                    if timer.set(DISCOVERY_INTERVAL_NS).is_err() {
                        uerror!("ipd", "timer_set_failed"; tag = "discovery_new");
                    }
                    let handle = timer.handle();
                    if ctx.watch_handle(handle, TAG_DISCOVERY_TIMER).is_err() {
                        uerror!("ipd", "watch_failed"; tag = "discovery");
                    }
                    self.discovery_timer = Some(timer);
                }
                Err(_) => uerror!("ipd", "timer_create_failed"; tag = "discovery"),
            }
        }
    }

    fn arm_poll_timer(&mut self, delay_ms: Option<u64>, ctx: &mut dyn BusCtx) {
        let ns = match delay_ms {
            Some(ms) => {
                if ms == 0 {
                    // smoltcp wants immediate re-poll (sub-millisecond timer
                    // truncated to 0). Use 1ms — the minimum useful interval.
                    1_000_000
                } else {
                    ms * 1_000_000
                }
            }
            None => 1_000_000_000, // 1s default housekeeping
        };

        if let Some(ref mut timer) = self.poll_timer {
            if timer.set(ns).is_err() {
                uerror!("ipd", "timer_set_failed"; tag = "poll");
            }
        } else {
            match Timer::new() {
                Ok(mut timer) => {
                    if timer.set(ns).is_err() {
                        uerror!("ipd", "timer_set_failed"; tag = "poll_new");
                    }
                    let handle = timer.handle();
                    if ctx.watch_handle(handle, TAG_POLL_TIMER).is_err() {
                        uerror!("ipd", "watch_failed"; tag = "poll");
                    }
                    self.poll_timer = Some(timer);
                }
                Err(_) => uerror!("ipd", "timer_create_failed"; tag = "poll"),
            }
        }
    }

    /// Re-arm the poll timer based on smoltcp's requested delay.
    /// Called only from TAG_POLL_TIMER handler after poll_smoltcp().
    fn rearm_poll_timer(&mut self, ctx: &mut dyn BusCtx) {
        let delay = unsafe { &mut *(&raw mut SMOL_STACK) }
            .as_mut()
            .and_then(|stack| {
                let now = Self::smoltcp_now();
                stack.iface.poll_delay(now, &stack.sockets)
                    .map(|d| d.total_millis() as u64)
            });
        self.arm_poll_timer(delay, ctx);
    }

    /// Arm the coalescing timer if not already armed.
    ///
    /// Called from data_ready() and socket event handlers instead of
    /// poll_smoltcp(). Defers the poll by COALESCE_MS so multiple events
    /// accumulate before a single iface.poll() processes them all.
    fn arm_coalesce(&mut self, ctx: &mut dyn BusCtx) {
        if !self.coalescing {
            self.arm_poll_timer(Some(COALESCE_MS), ctx);
            self.coalescing = true;
        }
    }

    fn arm_dhcp_fallback_timer(&mut self, ctx: &mut dyn BusCtx) {
        match Timer::new() {
            Ok(mut timer) => {
                if timer.set(DHCP_FALLBACK_TIMEOUT_NS).is_err() {
                    uerror!("ipd", "timer_set_failed"; tag = "dhcp_fb");
                }
                let handle = timer.handle();
                if ctx.watch_handle(handle, TAG_DHCP_FALLBACK_TIMER).is_err() {
                    uerror!("ipd", "watch_failed"; tag = "dhcp_fb");
                }
                self.dhcp_fallback_timer = Some(timer);
            }
            Err(_) => uerror!("ipd", "timer_create_failed"; tag = "dhcp_fb"),
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
            IpState::StaticFallback | IpState::StaticConfigured => 2,
            IpState::Unconfigured => 0,
        };

        // SAFETY: single-threaded, SMOL_STACK initialized before this is called.
        if let Some(stack) = unsafe { &mut *(&raw mut SMOL_STACK) } {
            stack.iface.update_ip_addrs(|addrs| {
                addrs.clear();
                if addrs.push(IpCidr::new(IpAddress::Ipv4(ip), prefix)).is_err() {
                    uerror!("ipd", "ip_push_failed"; ip0 = ip_bytes[0] as u32, ip1 = ip_bytes[1] as u32);
                }
            });
            if stack.iface.routes_mut().add_default_ipv4_route(gateway).is_err() {
                uerror!("ipd", "route_add_failed"; gw0 = gw_bytes[0] as u32, gw1 = gw_bytes[1] as u32);
            }
        }

        let state_str = match state {
            IpState::DhcpConfigured => "dhcp",
            IpState::StaticFallback => "static_fallback",
            IpState::StaticConfigured => "static",
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
    /// Uses deferred ack: peeks CQEs without advancing shared cq_head.
    /// Separates TX completions (CQE_FLAG_TX_DONE) from RX data CQEs.
    /// TX completions free the consumer pool slot. RX CQEs are buffered.
    /// Returns the number of RX CQEs peeked (for later ack).
    fn drain_rx(&mut self, ctx: &mut dyn BusCtx) -> u32 {
        use userlib::ring::io_status::CQE_FLAG_TX_DONE;

        let port_id = self.nic_port;
        let mut rx_count = 0u32;
        let mut total_peeked = 0u32;

        if let Some(port) = ctx.block_port(port_id) {
            let ring_mask = port.ring_mask();
            for _ in 0..32 {
                let cqe = match port.peek_completion() {
                    Some(c) => c,
                    None => break,
                };
                total_peeked += 1;

                if cqe.flags & CQE_FLAG_TX_DONE != 0 {
                    // TX completion — reclaim consumer pool slot
                    if let Some(offset) = self.tx_tracker.complete(cqe.tag, ring_mask) {
                        port.free(offset);
                        self.stats.tx_cqe_reclaimed += 1;
                    }
                    continue;
                }

                // RX data CQE — filter by group_id (stored in lower bits of flags)
                let group = cqe.flags & 0x00FF;
                if group != self.group_id as u16 {
                    continue;
                }

                let offset = cqe.result;
                let len = cqe.transferred;
                if len > 0 && len <= MAX_FRAME_SIZE as u32 {
                    self.rx_queue.push(offset, len);
                    rx_count += 1;
                    self.stats.rx_frames += 1;
                    self.stats.rx_bytes += len as u64;

                    // Capture diagnostic: first bytes of frame for debugging
                    self.last_rx_offset = offset;
                    self.last_rx_len = len;
                    let cap = 20.min(len as usize);
                    if let Some(slice) = port.pool_slice(offset, cap as u32) {
                        self.last_rx_hdr[..cap].copy_from_slice(&slice[..cap]);
                    }
                    if self.pool_base_diag == 0 {
                        self.pool_base_diag = port.pool_ptr() as u64;
                    }
                }
            }

            // DON'T ack here — defer until after smoltcp consumes the RX frames.
            // If we ack now, the provider reclaims pool slots that smoltcp
            // hasn't read yet (use-after-free on pool memory).
            self.pending_acks += total_peeked;
        }

        rx_count
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
            let mut device = SmolDevice::new(
                port,
                &mut self.rx_queue,
                &mut self.tx_tracker,
                &mut self.stats,
                self.group_id,
            );
            stack.iface.poll(now, &mut device, &mut stack.sockets);
        }

        // Ack peeked CQEs NOW — smoltcp has consumed the RX frames via
        // IpdRxToken, so the provider can safely reclaim pool slots.
        if self.pending_acks > 0 {
            if let Some(port) = ctx.block_port(self.nic_port) {
                port.ack_completions(self.pending_acks);
            }
            self.pending_acks = 0;
        }

        // Poll DHCP socket for configuration events (only when DHCP is enabled)
        if let Some(dhcp_handle) = stack.dhcp_handle {
        if self.ip_state == IpState::Unconfigured || self.ip_state == IpState::DhcpConfigured {
            let event = stack.sockets.get_mut::<dhcpv4::Socket>(dhcp_handle).poll();
            match event {
                Some(dhcpv4::Event::Configured(config)) => {
                    let ip = config.address.address();
                    let prefix = config.address.prefix_len();
                    let gateway = config.router.unwrap_or(STATIC_GATEWAY);
                    self.apply_ip_config(ip, prefix, gateway, IpState::DhcpConfigured);

                    // Cancel the fallback timer
                    if let Some(ref timer) = self.dhcp_fallback_timer {
                        if ctx.unwatch_handle(timer.handle()).is_err() {
                            uerror!("ipd", "unwatch_failed"; tag = "dhcp_fallback");
                        }
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
        } // if let Some(dhcp_handle)

        // Process TFTP socket (UDP)
        {
            let sock = stack.sockets.get_mut::<udp::Socket>(stack.udp_handle);
            while let Ok((data, meta)) = sock.recv() {
                let remote = meta.endpoint;
                let mut resp_buf = [0u8; 516]; // 4 header + 512 data
                let resp_len = self.tftp.handle(data, remote, &mut resp_buf, &mut self.vfs_client);
                if resp_len > 0 {
                    if sock.send_slice(&resp_buf[..resp_len], remote).is_err() {
                        uerror!("ipd", "udp_send_failed"; len = resp_len as u32);
                    }
                }
            }
        }

        // Process DHCP server socket (AP mode)
        if let Some(dhcp_srv_handle) = stack.dhcp_srv_handle {
            let sock = stack.sockets.get_mut::<udp::Socket>(dhcp_srv_handle);
            let mut dhcp_rx_count = 0u32;
            while let Ok((data, _meta)) = sock.recv() {
                dhcp_rx_count += 1;
                let mut resp_buf = [0u8; 576];
                if let Some((resp_len, dest)) = self.dhcp_server.handle(data, &mut resp_buf) {
                    if sock.send_slice(&resp_buf[..resp_len], dest).is_err() {
                        uerror!("ipd", "dhcp_srv_send_failed"; len = resp_len as u32);
                    }
                }
            }
            if dhcp_rx_count > 0 {
                uinfo!("ipd", "dhcp_srv_poll"; rx_count = dhcp_rx_count);
            }
        }

        // Process HTTP socket (TCP)
        let ip_source = match self.ip_state {
            IpState::DhcpConfigured => "dhcp",
            IpState::StaticFallback | IpState::StaticConfigured => "static",
            IpState::Unconfigured => "none",
        };
        Self::process_http(&mut stack.sockets, stack.tcp_handle, self.assigned_ip, ip_source);

        // Process remote shell socket (TCP port 23)
        self.rsh.process(&mut stack.sockets, stack.tcp_rsh_handle);

        // Process TCP echo socket (port 7)
        Self::process_echo(&mut stack.sockets, stack.tcp_echo_handle, &mut self.echo_log_counter,
                           &self.stats);

        // Watch/unwatch rsh pending IPC channel for responsive wake
        let new_handle = self.rsh.pending_handle();
        match (self.rsh_watched_handle, new_handle) {
            (None, Some(h)) => {
                // New pending command — watch the channel
                if ctx.watch_handle(h, TAG_RSH_SHELL_RESPONSE).is_err() {
                    uerror!("ipd", "watch_failed"; tag = "rsh");
                }
                self.rsh_watched_handle = Some(h);
            }
            (Some(old), None) => {
                // Command completed — unwatch
                if ctx.unwatch_handle(old).is_err() {
                    uerror!("ipd", "unwatch_failed"; tag = "rsh");
                }
                self.rsh_watched_handle = None;
            }
            (Some(old), Some(new)) if old.0 != new.0 => {
                // Handle changed (shouldn't happen, but be safe)
                if ctx.unwatch_handle(old).is_err() {
                    uerror!("ipd", "unwatch_failed"; tag = "rsh_old");
                }
                if ctx.watch_handle(new, TAG_RSH_SHELL_RESPONSE).is_err() {
                    uerror!("ipd", "watch_failed"; tag = "rsh_new");
                }
                self.rsh_watched_handle = Some(new);
            }
            _ => {} // No change
        }

        // Poll socket service: bridge TCP sockets ↔ DataPorts
        self.socket_svc.poll(&mut stack.sockets);

        // Watch/unwatch DataPort handles for socket service connections.
        // When a connection becomes Established, watch its DataPort so
        // client TX writes wake ipd immediately instead of waiting 10ms.
        for slot in 0..8 {
            let has_dp = self.socket_svc.data_port_handle(slot).is_some();
            if has_dp && !self.socket_data_watched[slot] {
                if let Some(h) = self.socket_svc.data_port_handle(slot) {
                    let tag = TAG_SOCKET_DATA_BASE + slot as u32;
                    if ctx.watch_handle(h, tag).is_err() {
                        uerror!("ipd", "watch_failed"; tag = tag);
                    } else {
                        self.socket_data_watched[slot] = true;
                        uinfo!("ipd", "dataport_watch_ok"; slot = slot as u32, tag = tag);
                    }
                }
            } else if !has_dp && self.socket_data_watched[slot] {
                self.socket_data_watched[slot] = false;
                // Handle already gone — no unwatch needed, Mux auto-cleans.
            }
        }

        // Re-poll after socket processing (smoltcp may have TX to send)
        let tx_before = self.stats.tx_frames;
        if let Some(port) = ctx.block_port(self.nic_port) {
            let mut device = SmolDevice::new(port, &mut self.rx_queue, &mut self.tx_tracker, &mut self.stats, self.group_id);
            stack.iface.poll(now, &mut device, &mut stack.sockets);
        }

        // Periodic TCP diagnostic: every 50 polls, dump socket state
        self.poll_count += 1;
        if self.poll_count % 50 == 0 {
            let tx_total = self.stats.tx_frames;
            let tx_this = tx_total - tx_before;
            // Log TCP socket state for active connections
            for slot in 0..8 {
                if let Some(h) = self.socket_svc.smol_handle(slot) {
                    let sock = stack.sockets.get::<tcp::Socket>(h);
                    udebug!("ipd", "tcp_diag";
                        poll = self.poll_count,
                        slot = slot as u32,
                        sq = sock.send_queue() as u32,
                        sc = sock.send_capacity() as u32,
                        rq = sock.recv_queue() as u32,
                        tx_t = tx_total as u32,
                        tx2 = tx_this as u32,
                        tx_drop = self.stats.tx_pool_drops,
                        tx_pf = self.stats.tx_pool_full
                    );
                }
            }
        }

    }

    // =========================================================================
    // Configuration Get/Set
    // =========================================================================

    fn ipd_config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        match key {
            b"" => self.format_summary(buf),
            b"ip" => format_ip(buf, self.assigned_ip),
            b"prefix" => format_u64(self.assigned_prefix as u64, buf),
            b"gateway" => format_ip(buf, self.assigned_gateway),
            b"mac" => self.format_mac(buf),
            b"dhcp" => {
                let s = match self.ip_state {
                    IpState::DhcpConfigured | IpState::Unconfigured => b"on" as &[u8],
                    IpState::StaticFallback | IpState::StaticConfigured => b"off",
                };
                let len = s.len().min(buf.len());
                buf[..len].copy_from_slice(&s[..len]);
                len
            }
            b"state" => {
                let s = match self.ip_state {
                    IpState::Unconfigured => b"unconfigured" as &[u8],
                    IpState::DhcpConfigured => b"dhcp",
                    IpState::StaticFallback => b"static_fallback",
                    IpState::StaticConfigured => b"static",
                };
                let len = s.len().min(buf.len());
                buf[..len].copy_from_slice(&s[..len]);
                len
            }
            b"stats" => self.format_stats(buf),
            b"diag.rx" => self.format_diag_rx(buf),
            _ => 0,
        }
    }

    fn format_summary(&self, buf: &mut [u8]) -> usize {
        use core::fmt::Write;
        struct BufWriter<'a> { buf: &'a mut [u8], pos: usize }
        impl Write for BufWriter<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                let bytes = s.as_bytes();
                let remaining = self.buf.len() - self.pos;
                let to_write = bytes.len().min(remaining);
                self.buf[self.pos..self.pos + to_write].copy_from_slice(&bytes[..to_write]);
                self.pos += to_write;
                Ok(())
            }
        }
        let mut w = BufWriter { buf, pos: 0 };

        let state = match self.ip_state {
            IpState::Unconfigured => "unconfigured",
            IpState::DhcpConfigured => "dhcp",
            IpState::StaticFallback => "static_fallback",
            IpState::StaticConfigured => "static",
        };
        let dhcp = match self.ip_state {
            IpState::DhcpConfigured | IpState::Unconfigured => "on",
            IpState::StaticFallback | IpState::StaticConfigured => "off",
        };

        let _ = core::write!(w,
            "ip={}.{}.{}.{}\n\
             prefix={}\n\
             gateway={}.{}.{}.{}\n\
             dhcp={}\n\
             state={}\n",
            self.assigned_ip[0], self.assigned_ip[1], self.assigned_ip[2], self.assigned_ip[3],
            self.assigned_prefix,
            self.assigned_gateway[0], self.assigned_gateway[1], self.assigned_gateway[2], self.assigned_gateway[3],
            dhcp,
            state
        );
        w.pos
    }

    fn format_stats(&self, buf: &mut [u8]) -> usize {
        use core::fmt::Write;
        struct BufWriter<'a> { buf: &'a mut [u8], pos: usize }
        impl Write for BufWriter<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                let bytes = s.as_bytes();
                let remaining = self.buf.len() - self.pos;
                let to_write = bytes.len().min(remaining);
                self.buf[self.pos..self.pos + to_write].copy_from_slice(&bytes[..to_write]);
                self.pos += to_write;
                Ok(())
            }
        }
        let mut w = BufWriter { buf, pos: 0 };
        let s = &self.stats;
        let _ = core::write!(w,
            "rx_frames={}\nrx_bytes={}\ntx_frames={}\ntx_bytes={}\n\
             tx_pool_drops={}\nrx_pool_full={}\ntx_cqe_reclaimed={}\ntx_pool_full={}\n",
            s.rx_frames, s.rx_bytes, s.tx_frames, s.tx_bytes,
            s.tx_pool_drops, s.rx_pool_full, s.tx_cqe_reclaimed, s.tx_pool_full);
        w.pos
    }

    fn format_diag_rx(&self, buf: &mut [u8]) -> usize {
        use core::fmt::Write;
        struct BufWriter<'a> { buf: &'a mut [u8], pos: usize }
        impl Write for BufWriter<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                let bytes = s.as_bytes();
                let remaining = self.buf.len() - self.pos;
                let to_write = bytes.len().min(remaining);
                self.buf[self.pos..self.pos + to_write].copy_from_slice(&bytes[..to_write]);
                self.pos += to_write;
                Ok(())
            }
        }
        let mut w = BufWriter { buf, pos: 0 };
        let _ = core::write!(w, "last_rx_len={}\nlast_rx_offset={}\npool_base=0x{:x}\n",
            self.last_rx_len, self.last_rx_offset, self.pool_base_diag);
        // Hex dump of first bytes
        let _ = core::write!(w, "hdr=");
        let cap = 20.min(self.last_rx_len as usize);
        for i in 0..cap {
            let _ = core::write!(w, "{:02x}", self.last_rx_hdr[i]);
            if i % 6 == 5 && i + 1 < cap { let _ = core::write!(w, " "); }
        }
        let _ = core::write!(w, "\n");
        // Parse Ethernet header if enough data
        if cap >= 14 {
            let _ = core::write!(w, "dst={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}\n",
                self.last_rx_hdr[0], self.last_rx_hdr[1], self.last_rx_hdr[2],
                self.last_rx_hdr[3], self.last_rx_hdr[4], self.last_rx_hdr[5]);
            let _ = core::write!(w, "src={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}\n",
                self.last_rx_hdr[6], self.last_rx_hdr[7], self.last_rx_hdr[8],
                self.last_rx_hdr[9], self.last_rx_hdr[10], self.last_rx_hdr[11]);
            let ethertype = ((self.last_rx_hdr[12] as u16) << 8) | self.last_rx_hdr[13] as u16;
            let _ = core::write!(w, "ethertype=0x{:04x}\n", ethertype);
        }
        let s = &self.stats;
        let _ = core::write!(w, "rx_frames={}\ntx_frames={}\ntx_pool_drops={}\npending_acks={}\n",
            s.rx_frames, s.tx_frames, s.tx_pool_drops, self.pending_acks);
        let _ = core::write!(w, "mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}\n",
            self.mac[0], self.mac[1], self.mac[2], self.mac[3], self.mac[4], self.mac[5]);
        let _ = core::write!(w, "ip={}.{}.{}.{}/{}\n",
            self.assigned_ip[0], self.assigned_ip[1], self.assigned_ip[2], self.assigned_ip[3],
            self.assigned_prefix);
        let _ = core::write!(w, "ap_mode={}\nnic_state={}\n",
            self.ap_mode, self.nic_state as u8);
        w.pos
    }

    fn ipd_config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], ctx: &mut dyn BusCtx) -> usize {
        match key {
            b"ip" => {
                if let Some(ip) = parse_ipv4(value) {
                    let gw = ipv4_from_bytes(&self.assigned_gateway);
                    let prefix = if self.assigned_prefix == 0 { STATIC_PREFIX_LEN } else { self.assigned_prefix };
                    self.apply_ip_config(ip, prefix, gw, IpState::StaticConfigured);
                    self.poll_smoltcp(ctx);
                    copy_str(buf, "OK\n")
                } else {
                    copy_str(buf, "ERR invalid ip\n")
                }
            }
            b"prefix" => {
                if let Some(prefix) = parse_u8(value) {
                    if prefix <= 32 {
                        let ip = ipv4_from_bytes(&self.assigned_ip);
                        let gw = ipv4_from_bytes(&self.assigned_gateway);
                        self.apply_ip_config(ip, prefix, gw, IpState::StaticConfigured);
                        self.poll_smoltcp(ctx);
                        copy_str(buf, "OK\n")
                    } else {
                        copy_str(buf, "ERR prefix must be 0-32\n")
                    }
                } else {
                    copy_str(buf, "ERR invalid prefix\n")
                }
            }
            b"gateway" => {
                if let Some(gw) = parse_ipv4(value) {
                    let ip = ipv4_from_bytes(&self.assigned_ip);
                    let prefix = if self.assigned_prefix == 0 { STATIC_PREFIX_LEN } else { self.assigned_prefix };
                    self.apply_ip_config(ip, prefix, gw, IpState::StaticConfigured);
                    self.poll_smoltcp(ctx);
                    copy_str(buf, "OK\n")
                } else {
                    copy_str(buf, "ERR invalid gateway\n")
                }
            }
            b"dhcp" => {
                match value {
                    b"on" => {
                        if let Some(stack) = unsafe { &mut *(&raw mut SMOL_STACK) } {
                            if let Some(dh) = stack.dhcp_handle {
                                self.ip_state = IpState::Unconfigured;
                                self.arm_dhcp_fallback_timer(ctx);
                                stack.sockets.get_mut::<dhcpv4::Socket>(dh).reset();
                                uinfo!("ipd", "dhcp_enabled";);
                                copy_str(buf, "OK\n")
                            } else {
                                copy_str(buf, "ERR no dhcp in ap mode\n")
                            }
                        } else {
                            copy_str(buf, "ERR no stack\n")
                        }
                    }
                    b"off" => {
                        if self.ip_state == IpState::DhcpConfigured || self.ip_state == IpState::Unconfigured {
                            self.ip_state = IpState::StaticConfigured;
                        }
                        if let Some(ref timer) = self.dhcp_fallback_timer {
                            if ctx.unwatch_handle(timer.handle()).is_err() {
                                uerror!("ipd", "unwatch_failed"; tag = "dhcp_off");
                            }
                        }
                        self.dhcp_fallback_timer = None;
                        // Reset DHCP socket to stop sending requests
                        if let Some(stack) = unsafe { &mut *(&raw mut SMOL_STACK) } {
                            if let Some(dh) = stack.dhcp_handle {
                                stack.sockets.get_mut::<dhcpv4::Socket>(dh).reset();
                            }
                        }
                        uinfo!("ipd", "dhcp_disabled";);
                        copy_str(buf, "OK\n")
                    }
                    _ => copy_str(buf, "ERR invalid value (use on/off)\n"),
                }
            }
            _ => 0,
        }
    }

    fn format_mac(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        for i in 0..6 {
            if i > 0 {
                pos += copy_str(&mut buf[pos..], ":");
            }
            let hi = self.mac[i] >> 4;
            let lo = self.mac[i] & 0x0F;
            if pos < buf.len() { buf[pos] = HEX_CHARS[hi as usize]; pos += 1; }
            if pos < buf.len() { buf[pos] = HEX_CHARS[lo as usize]; pos += 1; }
        }
        pos
    }

    // =========================================================================
    // HTTP Server (TCP port 80)
    // =========================================================================

    /// Process the HTTP TCP socket: read request, send response, re-listen.
    /// TCP echo (port 7): recv → send, re-listen on disconnect.
    /// Pure smoltcp — no DataPort, no IPC. For isolating WiFi TX issues.
    fn process_echo(sockets: &mut SocketSet<'static>, handle: SocketHandle, echo_log_counter: &mut u32,
                    stats: &DataPathStats) {
        let sock = sockets.get_mut::<tcp::Socket>(handle);

        if !sock.is_active() && !sock.is_listening() {
            if sock.listen(ECHO_PORT).is_err() {
                uerror!("ipd", "echo_relisten_failed";);
            }
            return;
        }

        // Log state once per ~64 polls when socket is active (not listening)
        if sock.is_active() {
            *echo_log_counter = echo_log_counter.wrapping_add(1);
            if *echo_log_counter & 63 == 1 {
                let recv_q = sock.recv_queue();
                let send_q = sock.send_queue();
                let state = sock.state() as u8;
                udebug!("ipd", "echo_state";
                    state = state as u32,
                    may_r = sock.may_recv() as u32,
                    may_s = sock.may_send() as u32,
                    recv_q = recv_q as u32,
                    send_q = send_q as u32,
                    dp_rx = stats.rx_frames,
                    dp_tx = stats.tx_frames);
            }
        }

        // Echo: drain RX → TX in a loop until blocked.
        let mut total_echoed = 0u32;
        while sock.may_recv() && sock.may_send() {
            let mut buf = [0u8; 2048];
            match sock.recv_slice(&mut buf) {
                Ok(n) if n > 0 => {
                    match sock.send_slice(&buf[..n]) {
                        Ok(sent) => {
                            total_echoed += sent as u32;
                            if sent < n {
                                udebug!("ipd", "echo_tx_full"; wanted = n as u32,
                                    sent = sent as u32);
                                break; // TX buffer full
                            }
                        }
                        Err(_) => {
                            uerror!("ipd", "echo_send_failed"; len = n as u32);
                            break;
                        }
                    }
                }
                _ => break,
            }
        }
        if total_echoed > 0 {
            uinfo!("ipd", "echo"; bytes = total_echoed,
                may_r = sock.may_recv() as u32, may_s = sock.may_send() as u32);
        }

        // Only close when peer has closed and we've drained all RX data.
        // Never close during SynReceived (may_recv/may_send are both false
        // during handshake, which previously triggered premature close).
        if sock.state() == tcp::State::CloseWait && sock.recv_queue() == 0 {
            sock.close();
        }
    }

    fn process_http(sockets: &mut SocketSet<'static>, handle: SocketHandle, ip: [u8; 4], ip_source: &str) {
        let sock = sockets.get_mut::<tcp::Socket>(handle);

        if !sock.is_active() && !sock.is_listening() {
            // Connection closed — re-listen for next connection
            if sock.listen(HTTP_PORT).is_err() {
                uerror!("ipd", "http_relisten_failed";);
            }
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
            if sock.send_slice(&resp[..resp_len]).is_err() {
                uerror!("ipd", "http_send_failed"; len = resp_len as u32);
            }
        }

        // Close our end after sending
        sock.close();
    }
}

// =============================================================================
// IP Parsing Helpers
// =============================================================================

const HEX_CHARS: &[u8; 16] = b"0123456789abcdef";

/// Parse an IPv4 address from ASCII bytes (e.g., b"10.0.2.15").
fn parse_ipv4(s: &[u8]) -> Option<Ipv4Address> {
    let s = core::str::from_utf8(s).ok()?;
    let mut octets = [0u8; 4];
    let mut idx = 0;
    for part in s.split('.') {
        if idx >= 4 { return None; }
        octets[idx] = part.parse::<u8>().ok()?;
        idx += 1;
    }
    if idx != 4 { return None; }
    Some(Ipv4Address::new(octets[0], octets[1], octets[2], octets[3]))
}

/// Construct an Ipv4Address from a 4-byte array.
fn ipv4_from_bytes(b: &[u8; 4]) -> Ipv4Address {
    Ipv4Address::new(b[0], b[1], b[2], b[3])
}

/// Parse a u8 from ASCII bytes.
fn parse_u8(s: &[u8]) -> Option<u8> {
    let s = core::str::from_utf8(s).ok()?;
    s.parse::<u8>().ok()
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
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        // group_id defaults to 0 (all ports in one bridge group).
        // When multiple bridge groups exist, switchd registers per-group ports
        // (bridge:0, bridge:1) and the group_id will be set via sidechannel query.
        udebug!("ipd", "starting"; group_id = self.group_id as u32);

        if self.try_discover_nic(ctx) {
            self.discovering = false;
            udebug!("ipd", "nic_probing";);
        } else {
            self.discovering = true;
            self.arm_discovery_timer(ctx);
            udebug!("ipd", "waiting_for_nic";);
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

                if ctx.respond_info(msg.seq_id, &info[..pos]).is_err() {
                    uerror!("ipd", "respond_info_failed";);
                }
                Disposition::Handled
            }
            // CONFIG_GET and CONFIG_SET handled by bus framework via
            // config_keys() / config_get() / config_set() trait methods.
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
        self.arm_coalesce(ctx);
    }

    fn handle_event(&mut self, tag: u32, _handle: Handle, ctx: &mut dyn BusCtx) {
        match tag {
            TAG_DISCOVERY_TIMER if self.discovering => {
                // Timer already consumed by Mux poll — no timer.wait() needed.
                if self.try_discover_nic(ctx) {
                    self.discovering = false;
                    if let Some(ref timer) = self.discovery_timer {
                        if ctx.unwatch_handle(timer.handle()).is_err() {
                            uerror!("ipd", "unwatch_failed"; tag = "discovery");
                        }
                    }
                    self.discovery_timer = None;
                    udebug!("ipd", "nic_probing";);
                } else {
                    self.arm_discovery_timer(ctx);
                }
            }
            TAG_POLL_TIMER => {
                if self.nic_state == NicState::Up {
                    self.drain_rx(ctx);
                    self.poll_smoltcp(ctx);
                }
                self.coalescing = false;
                self.rearm_poll_timer(ctx);
            }
            TAG_DHCP_FALLBACK_TIMER => {
                // Timer already consumed by Mux poll — no timer.wait() needed.
                self.apply_static_fallback();
                // Clean up the one-shot timer
                if let Some(ref timer) = self.dhcp_fallback_timer {
                    if ctx.unwatch_handle(timer.handle()).is_err() {
                        uerror!("ipd", "unwatch_failed"; tag = "dhcp_fb");
                    }
                }
                self.dhcp_fallback_timer = None;
            }
            TAG_RSH_SHELL_RESPONSE => {
                // Shell responded on IPC channel — process rsh to drain response
                // SAFETY: SMOL_STACK initialized before handle_event can fire.
                if let Some(stack) = unsafe { &mut *(&raw mut SMOL_STACK) } {
                    self.rsh.process(&mut stack.sockets, stack.tcp_rsh_handle);

                    // Update watch state
                    if self.rsh.pending_handle().is_none() {
                        if let Some(old) = self.rsh_watched_handle.take() {
                            if ctx.unwatch_handle(old).is_err() {
                                uerror!("ipd", "unwatch_failed"; tag = "rsh_evt");
                            }
                        }
                    }

                    // Defer poll to coalescing timer — flushes TCP data
                    self.arm_coalesce(ctx);
                }
            }
            TAG_SOCKET_SVC_PORT => {
                // New client connecting to "tcp:" port — accept and watch
                if let Some(stack) = unsafe { &mut *(&raw mut SMOL_STACK) } {
                    while let Some((handle, slot)) = self.socket_svc.accept_clients() {
                        let tag = TAG_SOCKET_CLIENT_BASE + slot;
                        if ctx.watch_handle(handle, tag).is_err() {
                            uerror!("ipd", "watch_failed"; tag = tag);
                        }
                    }
                    // Poll immediately to handle any pending LISTEN messages
                    self.socket_svc.poll(&mut stack.sockets);
                }
            }
            t if t >= TAG_SOCKET_DATA_BASE && t < TAG_SOCKET_DATA_BASE + 256 => {
                // DataPort wake from client TX — defer poll to coalescing timer
                let slot = (t - TAG_SOCKET_DATA_BASE) as usize;
                self.socket_svc.ack_data_port(slot);
                if self.nic_state == NicState::Up {
                    self.arm_coalesce(ctx);
                }
            }
            t if t >= TAG_SOCKET_CLIENT_BASE && t < TAG_SOCKET_CLIENT_BASE + 256 => {
                // Client channel readable — process control message
                let slot = (t - TAG_SOCKET_CLIENT_BASE) as usize;
                if let Some(stack) = unsafe { &mut *(&raw mut SMOL_STACK) } {
                    self.socket_svc.process_client_msg(slot, &mut stack.sockets);
                }
                // Defer poll to coalescing timer — flushes TCP data
                self.arm_coalesce(ctx);
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

const IPD_CONFIG_KEYS: &[ConfigKey] = &[
    ConfigKey::read_write(b"ip"),
    ConfigKey::read_write(b"prefix"),
    ConfigKey::read_write(b"gateway"),
    ConfigKey::read_write(b"dhcp"),
    ConfigKey::read_only(b"mac"),
    ConfigKey::read_only(b"state"),
    ConfigKey::read_only(b"stats"),
    ConfigKey::read_only(b"diag.rx"),
];

impl Driver for IpdDriverWrapper {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.reset(ctx)
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

    fn config_keys(&self) -> &[ConfigKey] {
        IPD_CONFIG_KEYS
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        self.0.ipd_config_get(key, buf)
    }

    fn config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], ctx: &mut dyn BusCtx) -> usize {
        self.0.ipd_config_set(key, value, buf, ctx)
    }
}
