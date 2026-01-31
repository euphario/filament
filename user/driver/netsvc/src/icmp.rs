//! ICMP Processing (RFC 792)
//!
//! Handles ICMP Echo Request/Reply (ping).
//!
//! ICMP header layout:
//! ```text
//! [0]      Type (8=echo request, 0=echo reply)
//! [1]      Code (0 for echo)
//! [2..4]   Checksum
//! [4..6]   Identifier
//! [6..8]   Sequence number
//! [8..]    Data
//! ```

use crate::types::{read_be16, write_be16};
use crate::checksum;

const ICMP_TYPE_ECHO_REPLY: u8 = 0;
const ICMP_TYPE_ECHO_REQUEST: u8 = 8;
const ICMP_HEADER_LEN: usize = 8;

/// Parsed ICMP header.
pub struct IcmpHeader {
    pub icmp_type: u8,
    pub code: u8,
    pub id: u16,
    pub seq: u16,
}

/// Parse an ICMP header from raw bytes (the IP payload).
pub fn parse(data: &[u8]) -> Option<IcmpHeader> {
    if data.len() < ICMP_HEADER_LEN {
        return None;
    }

    // Verify checksum over entire ICMP message
    if !checksum::verify(data) {
        return None;
    }

    Some(IcmpHeader {
        icmp_type: data[0],
        code: data[1],
        id: read_be16(data, 4),
        seq: read_be16(data, 6),
    })
}

/// Check if this is an echo request.
pub fn is_echo_request(hdr: &IcmpHeader) -> bool {
    hdr.icmp_type == ICMP_TYPE_ECHO_REQUEST && hdr.code == 0
}

/// Build an ICMP echo reply from an echo request.
///
/// Copies the request data and changes type to reply. Recomputes checksum.
/// `request_data` is the full ICMP payload (the IP payload from the request).
///
/// Returns the number of ICMP bytes written, or None if buffer too small.
pub fn build_echo_reply(buf: &mut [u8], request_data: &[u8]) -> Option<usize> {
    let len = request_data.len();
    if buf.len() < len || len < ICMP_HEADER_LEN {
        return None;
    }

    // Copy entire ICMP message
    buf[..len].copy_from_slice(request_data);

    // Change type to echo reply
    buf[0] = ICMP_TYPE_ECHO_REPLY;
    buf[1] = 0; // code

    // Zero checksum field before computing
    write_be16(buf, 2, 0);

    // Compute checksum
    let cksum = checksum::internet_checksum(&buf[..len]);
    write_be16(buf, 2, cksum);

    Some(len)
}
