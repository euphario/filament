//! IPv4 Processing (RFC 791)
//!
//! Parse and build IPv4 headers. Header checksum via RFC 1071.
//!
//! IPv4 header layout (20 bytes, no options):
//! ```text
//! [0]      Version (4) | IHL (5)
//! [1]      DSCP | ECN
//! [2..4]   Total length (big-endian)
//! [4..6]   Identification
//! [6..8]   Flags | Fragment offset
//! [8]      TTL
//! [9]      Protocol (1=ICMP, 6=TCP, 17=UDP)
//! [10..12] Header checksum
//! [12..16] Source IP
//! [16..20] Destination IP
//! ```

use crate::types::{Ipv4Addr, read_be16, write_be16};
use crate::checksum;

/// Minimum IPv4 header length (no options).
pub const IP_HEADER_LEN: usize = 20;

/// Parsed IPv4 header.
pub struct Ipv4Header {
    pub total_len: u16,
    pub ttl: u8,
    pub protocol: u8,
    pub src: Ipv4Addr,
    pub dst: Ipv4Addr,
    /// Header length in bytes (IHL * 4).
    pub header_len: usize,
}

/// Parse an IPv4 header from raw bytes.
///
/// Returns the parsed header, or None if invalid.
pub fn parse(data: &[u8]) -> Option<Ipv4Header> {
    if data.len() < IP_HEADER_LEN {
        return None;
    }

    let version = data[0] >> 4;
    if version != 4 {
        return None;
    }

    let ihl = (data[0] & 0x0F) as usize;
    let header_len = ihl * 4;
    if header_len < IP_HEADER_LEN || data.len() < header_len {
        return None;
    }

    let total_len = read_be16(data, 2);
    if (total_len as usize) < header_len || (total_len as usize) > data.len() {
        return None;
    }

    // Verify header checksum
    if !checksum::verify(&data[..header_len]) {
        return None;
    }

    let ttl = data[8];
    let protocol = data[9];

    let mut src = [0u8; 4];
    let mut dst = [0u8; 4];
    src.copy_from_slice(&data[12..16]);
    dst.copy_from_slice(&data[16..20]);

    Some(Ipv4Header {
        total_len,
        ttl,
        protocol,
        src: Ipv4Addr(src),
        dst: Ipv4Addr(dst),
        header_len,
    })
}

/// Build an IPv4 header into a buffer.
///
/// Writes a 20-byte header (no options) with computed checksum.
/// `payload_len` is the length of the payload after the IP header.
///
/// Returns 20 (header size), or None if buffer too small.
pub fn build_header(
    buf: &mut [u8],
    src: &Ipv4Addr,
    dst: &Ipv4Addr,
    protocol: u8,
    payload_len: u16,
    id: u16,
) -> Option<usize> {
    if buf.len() < IP_HEADER_LEN {
        return None;
    }

    let total_len = IP_HEADER_LEN as u16 + payload_len;

    buf[0] = 0x45; // Version 4, IHL 5
    buf[1] = 0x00; // DSCP 0, ECN 0
    write_be16(buf, 2, total_len);
    write_be16(buf, 4, id);
    write_be16(buf, 6, 0x4000); // Don't Fragment
    buf[8] = 64; // TTL
    buf[9] = protocol;
    write_be16(buf, 10, 0); // Checksum placeholder
    buf[12..16].copy_from_slice(&src.0);
    buf[16..20].copy_from_slice(&dst.0);

    // Compute header checksum
    let cksum = checksum::internet_checksum(&buf[..IP_HEADER_LEN]);
    write_be16(buf, 10, cksum);

    Some(IP_HEADER_LEN)
}

/// Get the payload slice from a parsed IP packet.
pub fn payload<'a>(data: &'a [u8], hdr: &Ipv4Header) -> &'a [u8] {
    &data[hdr.header_len..hdr.total_len as usize]
}
