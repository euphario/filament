//! Ethernet Frame Handling
//!
//! Parse and build Ethernet II frames (14-byte header).
//!
//! Frame layout:
//! ```text
//! [0..6]   Destination MAC (6 bytes)
//! [6..12]  Source MAC (6 bytes)
//! [12..14] EtherType (2 bytes, big-endian)
//! [14..]   Payload
//! ```

use crate::types::{EthAddr, read_be16, write_be16};

/// Minimum Ethernet frame size (header only, no FCS — NIC strips FCS).
pub const ETH_HEADER_LEN: usize = 14;
/// Maximum Ethernet payload (MTU 1500).
pub const ETH_MTU: usize = 1500;
/// Maximum Ethernet frame size (header + payload, no FCS).
pub const ETH_FRAME_MAX: usize = ETH_HEADER_LEN + ETH_MTU;

/// Parsed Ethernet header (references into a packet buffer).
pub struct EthHeader {
    pub dst: EthAddr,
    pub src: EthAddr,
    pub ethertype: u16,
}

/// Parse an Ethernet header from a raw frame.
///
/// Returns the header and the offset where the payload starts (14).
pub fn parse(frame: &[u8]) -> Option<EthHeader> {
    if frame.len() < ETH_HEADER_LEN {
        return None;
    }

    let mut dst = [0u8; 6];
    let mut src = [0u8; 6];
    dst.copy_from_slice(&frame[0..6]);
    src.copy_from_slice(&frame[6..12]);
    let ethertype = read_be16(frame, 12);

    Some(EthHeader {
        dst: EthAddr(dst),
        src: EthAddr(src),
        ethertype,
    })
}

/// Write an Ethernet header into a buffer.
///
/// Returns the number of bytes written (14), or None if buffer is too small.
pub fn write_header(buf: &mut [u8], dst: &EthAddr, src: &EthAddr, ethertype: u16) -> Option<usize> {
    if buf.len() < ETH_HEADER_LEN {
        return None;
    }
    buf[0..6].copy_from_slice(&dst.0);
    buf[6..12].copy_from_slice(&src.0);
    write_be16(buf, 12, ethertype);
    Some(ETH_HEADER_LEN)
}
