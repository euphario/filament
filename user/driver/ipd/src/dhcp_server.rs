//! Minimal DHCP Server for AP Mode
//!
//! Hands out IPs from 192.168.4.100–199 to WiFi clients.
//! Only supports DISCOVER→OFFER and REQUEST→ACK (the two exchanges
//! needed for basic DHCP assignment).

use smoltcp::wire::{
    DhcpMessageType, DhcpPacket, DhcpRepr, EthernetAddress, IpAddress, IpEndpoint, Ipv4Address,
};
use libos::{uinfo, udebug};

const LEASE_SECS: u32 = 3600;
const POOL_START: u8 = 100;
const POOL_END: u8 = 199;
const MAX_LEASES: usize = 16;

struct Lease {
    mac: [u8; 6],
    ip_last_octet: u8,
    expires_ns: u64,
}

pub struct DhcpServer {
    server_ip: Ipv4Address,
    subnet_mask: Ipv4Address,
    leases: [Option<Lease>; MAX_LEASES],
    /// Last processed transaction ID (for deduplication).
    last_xid: u32,
    /// Timestamp (ns) when last_xid was processed.
    last_xid_ns: u64,
}

impl DhcpServer {
    pub const fn new(server_ip: Ipv4Address, subnet_mask: Ipv4Address) -> Self {
        Self {
            server_ip,
            subnet_mask,
            leases: [const { None }; MAX_LEASES],
            last_xid: 0,
            last_xid_ns: 0,
        }
    }

    /// Process a received DHCP packet on port 67.
    /// Returns `Some((resp_len, dest_endpoint))` if a response should be sent.
    pub fn handle(&mut self, data: &[u8], out: &mut [u8]) -> Option<(usize, IpEndpoint)> {
        let packet = DhcpPacket::new_checked(data).ok()?;
        let repr = DhcpRepr::parse(&packet).ok()?;

        let now_ns = libsys::syscall::gettime();

        let msg_type_num = match repr.message_type {
            DhcpMessageType::Discover => 1u32,
            DhcpMessageType::Offer => 2,
            DhcpMessageType::Request => 3,
            DhcpMessageType::Ack => 5,
            _ => 99,
        };

        udebug!("dhcp", "rx"; xid = repr.transaction_id, msg = msg_type_num,
            last_xid = self.last_xid);

        // Dedup: suppress duplicate requests with the same XID within 1 second.
        // The RX path can deliver multiple copies of the same broadcast frame.
        if repr.transaction_id == self.last_xid
            && now_ns.wrapping_sub(self.last_xid_ns) < 1_000_000_000
        {
            udebug!("dhcp", "dedup_suppressed"; xid = repr.transaction_id);
            return None;
        }
        self.last_xid = repr.transaction_id;
        self.last_xid_ns = now_ns;

        match repr.message_type {
            DhcpMessageType::Discover => {
                let ip = self.alloc_ip(&repr.client_hardware_address, now_ns)?;
                let len = self.build_reply(
                    DhcpMessageType::Offer,
                    repr.transaction_id,
                    &repr.client_hardware_address,
                    ip,
                    out,
                )?;
                let dest = IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::BROADCAST), 68);
                Some((len, dest))
            }
            DhcpMessageType::Request => {
                // Validate the request is for us
                if let Some(sid) = repr.server_identifier {
                    if sid != self.server_ip {
                        return None; // Not for us
                    }
                }

                let ip = self.alloc_ip(&repr.client_hardware_address, now_ns)?;

                // Confirm: requested_ip must match what we allocated
                if let Some(req_ip) = repr.requested_ip {
                    if req_ip != ip {
                        return None; // Mismatch — ignore (could NAK but keep it simple)
                    }
                }

                let len = self.build_reply(
                    DhcpMessageType::Ack,
                    repr.transaction_id,
                    &repr.client_hardware_address,
                    ip,
                    out,
                )?;
                let dest = IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::BROADCAST), 68);
                Some((len, dest))
            }
            _ => None,
        }
    }

    /// Find existing lease for MAC or allocate a new IP from pool.
    fn alloc_ip(&mut self, mac: &EthernetAddress, now_ns: u64) -> Option<Ipv4Address> {
        let mac_bytes = mac.as_bytes();

        // Check for existing lease (even expired — reuse same IP for same MAC)
        for lease in self.leases.iter_mut().flatten() {
            if lease.mac == mac_bytes {
                lease.expires_ns = now_ns + (LEASE_SECS as u64) * 1_000_000_000;
                let octet = lease.ip_last_octet;
                return Some(self.ip_for_octet(octet));
            }
        }

        // Find unused octet: not in any active lease
        let mut used = [false; 256];
        for lease in self.leases.iter().flatten() {
            if lease.expires_ns > now_ns {
                used[lease.ip_last_octet as usize] = true;
            }
        }

        let octet = (POOL_START..=POOL_END).find(|&o| !used[o as usize])?;

        // Find a free slot (prefer empty, then expired)
        let slot = self
            .leases
            .iter()
            .position(|l| l.is_none())
            .or_else(|| {
                self.leases
                    .iter()
                    .position(|l| l.as_ref().is_some_and(|l| l.expires_ns <= now_ns))
            })?;

        self.leases[slot] = Some(Lease {
            mac: {
                let mut m = [0u8; 6];
                m.copy_from_slice(mac_bytes);
                m
            },
            ip_last_octet: octet,
            expires_ns: now_ns + (LEASE_SECS as u64) * 1_000_000_000,
        });

        Some(self.ip_for_octet(octet))
    }

    fn ip_for_octet(&self, last: u8) -> Ipv4Address {
        let base = self.server_ip.octets();
        Ipv4Address::new(base[0], base[1], base[2], last)
    }

    /// Build a DHCP reply (OFFER or ACK) into `out`. Returns the packet length.
    fn build_reply(
        &self,
        msg_type: DhcpMessageType,
        xid: u32,
        client_mac: &EthernetAddress,
        offered_ip: Ipv4Address,
        out: &mut [u8],
    ) -> Option<usize> {
        let repr = DhcpRepr {
            message_type: msg_type,
            transaction_id: xid,
            secs: 0,
            client_hardware_address: *client_mac,
            client_ip: Ipv4Address::UNSPECIFIED,
            your_ip: offered_ip,
            server_ip: self.server_ip,
            router: Some(self.server_ip),
            subnet_mask: Some(self.subnet_mask),
            relay_agent_ip: Ipv4Address::UNSPECIFIED,
            broadcast: false,
            requested_ip: None,
            client_identifier: None,
            server_identifier: Some(self.server_ip),
            parameter_request_list: None,
            dns_servers: None,
            max_size: None,
            lease_duration: Some(LEASE_SECS),
            renew_duration: None,
            rebind_duration: None,
            additional_options: &[],
        };

        let len = repr.buffer_len();
        if len > out.len() {
            return None;
        }

        let mut packet = DhcpPacket::new_checked(&mut out[..len]).ok()?;
        repr.emit(&mut packet).ok()?;

        Some(len)
    }
}
