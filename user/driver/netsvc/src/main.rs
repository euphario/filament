//! Network Service (netsvc)
//!
//! Single-process network stack: Ethernet → ARP → IP → ICMP (Phase 1).
//! NICs dynamically attach to the stack — netsvc discovers NIC ports
//! registered by device drivers (netd, ethd, wifid) and connects as
//! a DataPort consumer.
//!
//! Architecture:
//! - Static service, always running (like consoled/vfsd)
//! - Discovers NIC ports via `discover_port("net0")` with timer retry
//! - Connects as DataPort consumer to read RX packets, submit TX packets
//! - Queries NIC MAC/MTU via sidechannel
//! - Processes Ethernet/ARP/IP/ICMP inline (no IPC per packet)
//!
//! Phase 1: Static IP (10.0.2.15/24, gw 10.0.2.2), ARP replies, ICMP echo

#![no_std]
#![no_main]

mod types;
mod checksum;
mod eth;
mod arp;
mod ip;
mod icmp;
mod nic;

use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, PortId, bus_msg,
};
use userlib::bus_runtime::driver_main;
use userlib::ipc::Timer;
use userlib::ring::{IoSqe, SideEntry, io_op, side_msg, side_status};
use userlib::syscall::Handle;
use userlib::{uinfo, udebug, uerror};

use types::{EthAddr, Ipv4Addr, ethertype, ip_proto};
use nic::{NicTable, NicState};

// =============================================================================
// Constants
// =============================================================================

/// Tag for the NIC discovery timer in handle_event.
const TAG_DISCOVERY_TIMER: u32 = 1;

/// Discovery retry interval (500ms in nanoseconds).
const DISCOVERY_INTERVAL_NS: u64 = 500_000_000;

/// NIC port name to discover.
const NIC_PORT_NAME: &[u8] = b"net0";

/// Static IP configuration (QEMU user networking defaults).
const STATIC_IP: Ipv4Addr = Ipv4Addr::new(10, 0, 2, 15);
const STATIC_NETMASK: Ipv4Addr = Ipv4Addr::new(255, 255, 255, 0);
const STATIC_GATEWAY: Ipv4Addr = Ipv4Addr::new(10, 0, 2, 2);

/// Maximum packet buffer size (for building TX frames).
const MAX_FRAME_SIZE: usize = 1514;

// =============================================================================
// NetSvc Driver State
// =============================================================================

struct NetSvcDriver {
    nics: NicTable,
    /// Discovery timer handle (for retry).
    discovery_timer: Option<Timer>,
    /// Are we still looking for NICs?
    discovering: bool,
    /// IP identification counter (for outgoing IP packets).
    ip_id: u16,
}

impl NetSvcDriver {
    const fn new() -> Self {
        Self {
            nics: NicTable::new(),
            discovery_timer: None,
            discovering: false,
            ip_id: 0,
        }
    }

    fn next_ip_id(&mut self) -> u16 {
        let id = self.ip_id;
        self.ip_id = self.ip_id.wrapping_add(1);
        id
    }

    /// Try to discover and connect to a NIC port.
    fn try_discover_nic(&mut self, ctx: &mut dyn BusCtx) -> bool {
        match ctx.discover_port_by_name(NIC_PORT_NAME) {
            Ok(shmem_id) => {
                udebug!("netsvc", "nic_found"; shmem_id = shmem_id);
                match ctx.connect_block_port(shmem_id) {
                    Ok(port_id) => {
                        if let Some(idx) = self.nics.add(port_id, NIC_PORT_NAME) {
                            udebug!("netsvc", "nic_connected"; idx = idx as u32);

                            // Send NIC info query (response arrives in data_ready)
                            self.send_nic_query(idx, ctx);
                            return true;
                        }
                    }
                    Err(_) => {
                        uerror!("netsvc", "connect_failed";);
                    }
                }
            }
            Err(_) => {
                // Port not yet available
            }
        }
        false
    }

    /// Send a sidechannel query for NIC MAC/MTU (non-blocking).
    /// Response is processed in data_ready().
    fn send_nic_query(&mut self, nic_idx: usize, ctx: &mut dyn BusCtx) {
        let port_id = self.nics.nics[nic_idx].port_id;

        let request = SideEntry {
            msg_type: side_msg::QUERY_INFO,
            flags: 0,
            tag: nic_idx as u16,
            status: side_status::REQUEST,
            payload: [0; 24],
        };

        if let Some(port) = ctx.block_port(port_id) {
            port.side_send(&request);
            port.notify();
        }
    }

    /// Process a sidechannel response (NIC info query result).
    fn handle_side_response(&mut self, response: &SideEntry) {
        if response.msg_type != side_msg::QUERY_INFO || response.status != side_status::OK {
            return;
        }

        let idx = response.tag as usize;  // tag carries nic_idx
        if idx >= nic::MAX_NICS || self.nics.nics[idx].state != NicState::Probing {
            return;
        }

        // Parse response: [MAC(6), link_status(1), MTU(2)]
        let mut mac = [0u8; 6];
        mac.copy_from_slice(&response.payload[0..6]);
        self.nics.nics[idx].mac = EthAddr(mac);

        let mtu = u16::from_le_bytes([response.payload[7], response.payload[8]]);
        if mtu > 0 {
            self.nics.nics[idx].mtu = mtu;
        }

        let mut mac_buf = [0u8; 17];
        self.nics.nics[idx].mac.format(&mut mac_buf);
        udebug!("netsvc", "nic_info";
            mac = core::str::from_utf8(&mac_buf).unwrap_or("?"),
            mtu = mtu as u32);

        // Configure static IP and mark Up
        self.nics.nics[idx].ipv4 = STATIC_IP;
        self.nics.nics[idx].netmask = STATIC_NETMASK;
        self.nics.nics[idx].gateway = STATIC_GATEWAY;
        self.nics.nics[idx].state = NicState::Up;

        uinfo!("netsvc", "nic_up"; ip = "10.0.2.15");
    }

    /// Arm the discovery timer for retry.
    fn arm_discovery_timer(&mut self, ctx: &mut dyn BusCtx) {
        // Timer::set() takes a relative duration — the kernel adds current time
        if let Some(ref mut timer) = self.discovery_timer {
            let _ = timer.set(DISCOVERY_INTERVAL_NS);
        } else {
            if let Ok(mut timer) = Timer::new() {
                let _ = timer.set(DISCOVERY_INTERVAL_NS);
                let handle = timer.handle();
                let _ = ctx.watch_handle(handle, TAG_DISCOVERY_TIMER);
                self.discovery_timer = Some(timer);
            }
        }
    }

    /// Process a received Ethernet frame from a NIC.
    fn process_rx_frame(&mut self, nic_idx: usize, frame: &[u8], ctx: &mut dyn BusCtx) {
        let eth_hdr = match eth::parse(frame) {
            Some(h) => h,
            None => return,
        };

        // Check if frame is for us (our MAC, broadcast, or multicast)
        let our_mac = &self.nics.nics[nic_idx].mac;
        if eth_hdr.dst != *our_mac
            && !eth_hdr.dst.is_broadcast()
            && !eth_hdr.dst.is_multicast()
        {
            return;
        }

        let payload = &frame[eth::ETH_HEADER_LEN..];

        match eth_hdr.ethertype {
            ethertype::ARP => {
                self.handle_arp(nic_idx, &eth_hdr, payload, ctx);
            }
            ethertype::IPV4 => {
                self.handle_ipv4(nic_idx, &eth_hdr, payload, ctx);
            }
            _ => {
                // Unknown ethertype, drop
            }
        }
    }

    /// Handle an ARP packet.
    fn handle_arp(
        &mut self,
        nic_idx: usize,
        _eth_hdr: &eth::EthHeader,
        payload: &[u8],
        ctx: &mut dyn BusCtx,
    ) {
        let pkt = match arp::parse(payload) {
            Some(p) => p,
            None => return,
        };

        let our_ip = &self.nics.nics[nic_idx].ipv4;
        let our_mac = &self.nics.nics[nic_idx].mac;

        // Always learn sender's MAC/IP
        self.nics.nics[nic_idx]
            .arp
            .insert(pkt.sender_ip, pkt.sender_mac);

        if arp::is_request_for(&pkt, our_ip) {
            // Build ARP reply
            let mut frame_buf = [0u8; MAX_FRAME_SIZE];

            // Ethernet header
            let eth_len = match eth::write_header(
                &mut frame_buf,
                &pkt.sender_mac,
                our_mac,
                ethertype::ARP,
            ) {
                Some(n) => n,
                None => return,
            };

            // ARP reply
            let arp_len = match arp::build_reply(
                &mut frame_buf[eth_len..],
                our_mac,
                our_ip,
                &pkt.sender_mac,
                &pkt.sender_ip,
            ) {
                Some(n) => n,
                None => return,
            };

            let total_len = eth_len + arp_len;
            self.send_frame(nic_idx, &frame_buf[..total_len], ctx);
            udebug!("netsvc", "arp_reply";
                to_ip = pkt.sender_ip.as_u32());
        } else if arp::is_reply(&pkt) {
            // Already learned above, nothing else to do
        }
    }

    /// Handle an IPv4 packet.
    fn handle_ipv4(
        &mut self,
        nic_idx: usize,
        _eth_hdr: &eth::EthHeader,
        payload: &[u8],
        ctx: &mut dyn BusCtx,
    ) {
        let ip_hdr = match ip::parse(payload) {
            Some(h) => h,
            None => return,
        };

        let our_ip = &self.nics.nics[nic_idx].ipv4;

        // Only process packets for our IP or broadcast
        if ip_hdr.dst != *our_ip && !ip_hdr.dst.is_broadcast() {
            return;
        }

        let ip_payload = ip::payload(payload, &ip_hdr);

        match ip_hdr.protocol {
            ip_proto::ICMP => {
                self.handle_icmp(nic_idx, &ip_hdr, ip_payload, ctx);
            }
            _ => {
                // Drop unknown protocols (TCP/UDP come in Phase 2+)
            }
        }
    }

    /// Handle an ICMP packet.
    fn handle_icmp(
        &mut self,
        nic_idx: usize,
        ip_hdr: &ip::Ipv4Header,
        payload: &[u8],
        ctx: &mut dyn BusCtx,
    ) {
        let icmp_hdr = match icmp::parse(payload) {
            Some(h) => h,
            None => return,
        };

        if icmp::is_echo_request(&icmp_hdr) {
            // Copy values to avoid borrow conflicts
            let our_ip = self.nics.nics[nic_idx].ipv4;
            let our_mac = self.nics.nics[nic_idx].mac;
            let dst_ip = ip_hdr.src;

            // We need the destination MAC — look up sender's MAC from ARP
            let dst_mac = match self.nics.nics[nic_idx].arp.lookup(&dst_ip) {
                Some(mac) => mac,
                None => return, // Don't have MAC, can't reply
            };

            let mut frame_buf = [0u8; MAX_FRAME_SIZE];

            // Ethernet header
            let eth_len = match eth::write_header(
                &mut frame_buf,
                &dst_mac,
                &our_mac,
                ethertype::IPV4,
            ) {
                Some(n) => n,
                None => return,
            };

            // ICMP echo reply
            let mut icmp_buf = [0u8; 1480]; // max IP payload
            let icmp_len = match icmp::build_echo_reply(&mut icmp_buf, payload) {
                Some(n) => n,
                None => return,
            };

            // IP header
            let ip_id = self.next_ip_id();
            let ip_len = match ip::build_header(
                &mut frame_buf[eth_len..],
                &our_ip,
                &dst_ip,
                ip_proto::ICMP,
                icmp_len as u16,
                ip_id,
            ) {
                Some(n) => n,
                None => return,
            };

            // Copy ICMP payload after IP header
            let icmp_start = eth_len + ip_len;
            if icmp_start + icmp_len > MAX_FRAME_SIZE {
                return;
            }
            frame_buf[icmp_start..icmp_start + icmp_len].copy_from_slice(&icmp_buf[..icmp_len]);

            let total_len = icmp_start + icmp_len;
            self.send_frame(nic_idx, &frame_buf[..total_len], ctx);
            udebug!("netsvc", "icmp_reply";
                to_ip = dst_ip.as_u32(),
                id = icmp_hdr.id as u32,
                seq = icmp_hdr.seq as u32);
        }
    }

    /// Send an Ethernet frame out a NIC via the DataPort SQ.
    fn send_frame(&mut self, nic_idx: usize, frame: &[u8], ctx: &mut dyn BusCtx) {
        let port_id = self.nics.nics[nic_idx].port_id;

        if let Some(port) = ctx.block_port(port_id) {
            // Allocate pool space and write frame
            let len = frame.len() as u32;
            if let Some(offset) = port.alloc(len) {
                port.pool_write(offset, frame);

                let sqe = IoSqe {
                    opcode: io_op::NET_SEND,
                    flags: 0,
                    priority: 0,
                    tag: 0,
                    data_offset: offset,
                    data_len: len,
                    lba: 0,
                    param: 0,
                };
                port.submit(&sqe);
                port.notify();
            }
        }
    }
}

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for NetSvcDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        udebug!("netsvc", "starting";);

        // Try to discover NIC immediately (may fail if netd hasn't started yet)
        if self.try_discover_nic(ctx) {
            self.discovering = false;
            udebug!("netsvc", "nic_probing";);
        } else {
            // Start discovery timer for retry
            self.discovering = true;
            self.arm_discovery_timer(ctx);
            udebug!("netsvc", "waiting_for_nic";);
        }

        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::QUERY_INFO => {
                let mut info = [0u8; 64];
                let prefix = b"netsvc: ";
                info[..prefix.len()].copy_from_slice(prefix);
                let mut pos = prefix.len();

                if self.nics.count > 0 && self.nics.nics[0].state == NicState::Up {
                    let suffix = b"up, 1 NIC";
                    let slen = suffix.len().min(64 - pos);
                    info[pos..pos + slen].copy_from_slice(&suffix[..slen]);
                    pos += slen;
                } else {
                    let suffix = b"no NICs";
                    let slen = suffix.len().min(64 - pos);
                    info[pos..pos + slen].copy_from_slice(&suffix[..slen]);
                    pos += slen;
                }

                let _ = ctx.respond_info(msg.seq_id, &info[..pos]);
                Disposition::Handled
            }
            _ => Disposition::Forward,
        }
    }

    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) {
        // Find which NIC this port belongs to
        let nic_idx = match self.nics.find_by_port(port) {
            Some(idx) => idx,
            None => return,
        };

        if self.nics.nics[nic_idx].state != NicState::Up {
            return;
        }

        // Process received packets (CQE from NIC driver)
        // Read up to 16 completions per wake
        let mut cqes: [Option<(u32, u32)>; 16] = [None; 16]; // (pool_offset, length)
        let mut cqe_count = 0;

        if let Some(port) = ctx.block_port(port) {
            while cqe_count < 16 {
                if let Some(completion) = port.poll_completion() {
                    cqes[cqe_count] = Some((completion.result, completion.transferred));
                    cqe_count += 1;
                } else {
                    break;
                }
            }
        }

        // Process each received frame
        for i in 0..cqe_count {
            if let Some((pool_offset, frame_len)) = cqes[i] {
                if frame_len > 0 && frame_len <= MAX_FRAME_SIZE as u32 {
                    // Read frame from pool
                    let mut frame_buf = [0u8; MAX_FRAME_SIZE];
                    let mut got_frame = false;
                    if let Some(port_ref) = ctx.block_port(port) {
                        if let Some(data) = port_ref.pool_slice(pool_offset, frame_len) {
                            frame_buf[..frame_len as usize].copy_from_slice(data);
                            got_frame = true;
                        }
                    }

                    if got_frame {
                        self.process_rx_frame(nic_idx, &frame_buf[..frame_len as usize], ctx);
                    }
                }
            }
        }

        // Process sidechannel responses (NIC info queries)
        let mut side_responses: [Option<SideEntry>; 4] = [None; 4];
        let mut side_count = 0;
        if let Some(port_ref) = ctx.block_port(port) {
            while side_count < 4 {
                if let Some(resp) = port_ref.poll_side_response() {
                    side_responses[side_count] = Some(resp);
                    side_count += 1;
                } else {
                    break;
                }
            }
        }
        for i in 0..side_count {
            if let Some(ref resp) = side_responses[i] {
                self.handle_side_response(resp);
            }
        }
    }

    fn handle_event(&mut self, tag: u32, _handle: Handle, ctx: &mut dyn BusCtx) {
        if tag == TAG_DISCOVERY_TIMER && self.discovering {
            // Re-arm timer and retry discovery
            if let Some(ref mut timer) = self.discovery_timer {
                // Read the timer to clear the fired state
                let _ = timer.wait();
            }

            if self.try_discover_nic(ctx) {
                self.discovering = false;

                // Unwatch and drop the timer
                if let Some(ref timer) = self.discovery_timer {
                    let _ = ctx.unwatch_handle(timer.handle());
                }
                self.discovery_timer = None;

                udebug!("netsvc", "nic_probing";);
            } else {
                // Retry
                self.arm_discovery_timer(ctx);
            }
        }
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DRIVER: NetSvcDriver = NetSvcDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"netsvc", NetSvcDriverWrapper(driver));
}

struct NetSvcDriverWrapper(&'static mut NetSvcDriver);

impl Driver for NetSvcDriverWrapper {
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
}
