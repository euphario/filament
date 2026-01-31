//! ARP Protocol (RFC 826)
//!
//! ARP table management and request/reply handling for IPv4 over Ethernet.
//!
//! ARP packet layout (28 bytes for IPv4/Ethernet):
//! ```text
//! [0..2]   Hardware type (0x0001 = Ethernet)
//! [2..4]   Protocol type (0x0800 = IPv4)
//! [4]      Hardware address length (6)
//! [5]      Protocol address length (4)
//! [6..8]   Operation (1 = request, 2 = reply)
//! [8..14]  Sender hardware address (MAC)
//! [14..18] Sender protocol address (IP)
//! [18..24] Target hardware address (MAC)
//! [24..28] Target protocol address (IP)
//! ```

use crate::types::{EthAddr, Ipv4Addr, read_be16, write_be16};

const ARP_HTYPE_ETHERNET: u16 = 0x0001;
const ARP_PTYPE_IPV4: u16 = 0x0800;
const ARP_OP_REQUEST: u16 = 1;
const ARP_OP_REPLY: u16 = 2;
const ARP_PACKET_LEN: usize = 28;

/// Maximum ARP table entries.
const MAX_ARP_ENTRIES: usize = 32;

/// ARP table entry.
#[derive(Clone, Copy)]
struct ArpEntry {
    ip: Ipv4Addr,
    mac: EthAddr,
    age: u32, // Incremented each time the table is swept; 0 = most recent
}

impl ArpEntry {
    const fn empty() -> Self {
        Self {
            ip: Ipv4Addr::ZERO,
            mac: EthAddr::ZERO,
            age: 0,
        }
    }
}

/// ARP table: maps IPv4 addresses to Ethernet MAC addresses.
pub struct ArpTable {
    entries: [ArpEntry; MAX_ARP_ENTRIES],
    count: usize,
}

impl ArpTable {
    pub const fn new() -> Self {
        Self {
            entries: [ArpEntry::empty(); MAX_ARP_ENTRIES],
            count: 0,
        }
    }

    /// Look up a MAC address for the given IP.
    pub fn lookup(&self, ip: &Ipv4Addr) -> Option<EthAddr> {
        for i in 0..self.count {
            if self.entries[i].ip == *ip {
                return Some(self.entries[i].mac);
            }
        }
        None
    }

    /// Insert or update an ARP entry.
    pub fn insert(&mut self, ip: Ipv4Addr, mac: EthAddr) {
        // Update existing
        for i in 0..self.count {
            if self.entries[i].ip == ip {
                self.entries[i].mac = mac;
                self.entries[i].age = 0;
                return;
            }
        }

        // Add new
        if self.count < MAX_ARP_ENTRIES {
            self.entries[self.count] = ArpEntry { ip, mac, age: 0 };
            self.count += 1;
        } else {
            // Replace oldest entry
            let mut oldest_idx = 0;
            let mut oldest_age = 0;
            for i in 0..self.count {
                if self.entries[i].age > oldest_age {
                    oldest_age = self.entries[i].age;
                    oldest_idx = i;
                }
            }
            self.entries[oldest_idx] = ArpEntry { ip, mac, age: 0 };
        }
    }

    /// Age all entries. Call periodically to enable eviction.
    pub fn age_entries(&mut self) {
        for i in 0..self.count {
            self.entries[i].age = self.entries[i].age.saturating_add(1);
        }
    }
}

/// Parsed ARP packet.
pub struct ArpPacket {
    pub op: u16,
    pub sender_mac: EthAddr,
    pub sender_ip: Ipv4Addr,
    pub target_mac: EthAddr,
    pub target_ip: Ipv4Addr,
}

/// Parse an ARP packet from raw bytes (the Ethernet payload, not including Eth header).
pub fn parse(data: &[u8]) -> Option<ArpPacket> {
    if data.len() < ARP_PACKET_LEN {
        return None;
    }

    let htype = read_be16(data, 0);
    let ptype = read_be16(data, 2);
    let hlen = data[4];
    let plen = data[5];

    // Only handle Ethernet + IPv4
    if htype != ARP_HTYPE_ETHERNET || ptype != ARP_PTYPE_IPV4 || hlen != 6 || plen != 4 {
        return None;
    }

    let op = read_be16(data, 6);
    let mut sender_mac = [0u8; 6];
    let mut sender_ip = [0u8; 4];
    let mut target_mac = [0u8; 6];
    let mut target_ip = [0u8; 4];

    sender_mac.copy_from_slice(&data[8..14]);
    sender_ip.copy_from_slice(&data[14..18]);
    target_mac.copy_from_slice(&data[18..24]);
    target_ip.copy_from_slice(&data[24..28]);

    Some(ArpPacket {
        op,
        sender_mac: EthAddr(sender_mac),
        sender_ip: Ipv4Addr(sender_ip),
        target_mac: EthAddr(target_mac),
        target_ip: Ipv4Addr(target_ip),
    })
}

/// Build an ARP reply packet into a buffer.
///
/// Returns the number of bytes written (28), or None if buffer too small.
pub fn build_reply(
    buf: &mut [u8],
    sender_mac: &EthAddr,
    sender_ip: &Ipv4Addr,
    target_mac: &EthAddr,
    target_ip: &Ipv4Addr,
) -> Option<usize> {
    if buf.len() < ARP_PACKET_LEN {
        return None;
    }

    write_be16(buf, 0, ARP_HTYPE_ETHERNET);
    write_be16(buf, 2, ARP_PTYPE_IPV4);
    buf[4] = 6; // hlen
    buf[5] = 4; // plen
    write_be16(buf, 6, ARP_OP_REPLY);
    buf[8..14].copy_from_slice(&sender_mac.0);
    buf[14..18].copy_from_slice(&sender_ip.0);
    buf[18..24].copy_from_slice(&target_mac.0);
    buf[24..28].copy_from_slice(&target_ip.0);

    Some(ARP_PACKET_LEN)
}

/// Build an ARP request packet into a buffer.
pub fn build_request(
    buf: &mut [u8],
    sender_mac: &EthAddr,
    sender_ip: &Ipv4Addr,
    target_ip: &Ipv4Addr,
) -> Option<usize> {
    if buf.len() < ARP_PACKET_LEN {
        return None;
    }

    write_be16(buf, 0, ARP_HTYPE_ETHERNET);
    write_be16(buf, 2, ARP_PTYPE_IPV4);
    buf[4] = 6; // hlen
    buf[5] = 4; // plen
    write_be16(buf, 6, ARP_OP_REQUEST);
    buf[8..14].copy_from_slice(&sender_mac.0);
    buf[14..18].copy_from_slice(&sender_ip.0);
    buf[18..24].copy_from_slice(&EthAddr::ZERO.0); // unknown
    buf[24..28].copy_from_slice(&target_ip.0);

    Some(ARP_PACKET_LEN)
}

/// Check if an ARP packet is a request for our IP.
pub fn is_request_for(pkt: &ArpPacket, our_ip: &Ipv4Addr) -> bool {
    pkt.op == ARP_OP_REQUEST && pkt.target_ip == *our_ip
}

/// Check if an ARP packet is a reply.
pub fn is_reply(pkt: &ArpPacket) -> bool {
    pkt.op == ARP_OP_REPLY
}
