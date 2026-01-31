//! Network Type Definitions
//!
//! EthAddr, Ipv4Addr, and packet buffer helpers for no_std networking.

/// Ethernet MAC address (6 bytes).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct EthAddr(pub [u8; 6]);

impl EthAddr {
    pub const BROADCAST: Self = Self([0xFF; 6]);
    pub const ZERO: Self = Self([0; 6]);

    pub fn is_broadcast(&self) -> bool {
        self.0 == [0xFF; 6]
    }

    pub fn is_multicast(&self) -> bool {
        self.0[0] & 0x01 != 0
    }

    /// Format as "xx:xx:xx:xx:xx:xx" into buffer. Returns slice length (17).
    pub fn format(&self, buf: &mut [u8; 17]) {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        for i in 0..6 {
            buf[i * 3] = HEX[(self.0[i] >> 4) as usize];
            buf[i * 3 + 1] = HEX[(self.0[i] & 0xf) as usize];
            if i < 5 {
                buf[i * 3 + 2] = b':';
            }
        }
    }
}

/// IPv4 address (4 bytes, network byte order).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Ipv4Addr(pub [u8; 4]);

impl Ipv4Addr {
    pub const ZERO: Self = Self([0, 0, 0, 0]);
    pub const BROADCAST: Self = Self([255, 255, 255, 255]);

    pub const fn new(a: u8, b: u8, c: u8, d: u8) -> Self {
        Self([a, b, c, d])
    }

    /// Check if this address is in the same subnet as `other` given `mask`.
    pub fn same_subnet(&self, other: &Self, mask: &Self) -> bool {
        for i in 0..4 {
            if (self.0[i] & mask.0[i]) != (other.0[i] & mask.0[i]) {
                return false;
            }
        }
        true
    }

    pub fn is_broadcast(&self) -> bool {
        *self == Self::BROADCAST
    }

    /// Convert to u32 (big-endian/network order) for logging.
    pub fn as_u32(&self) -> u32 {
        ((self.0[0] as u32) << 24)
            | ((self.0[1] as u32) << 16)
            | ((self.0[2] as u32) << 8)
            | self.0[3] as u32
    }
}

/// EtherType constants (big-endian as they appear on the wire).
pub mod ethertype {
    pub const IPV4: u16 = 0x0800;
    pub const ARP: u16 = 0x0806;
    pub const IPV6: u16 = 0x86DD;
}

/// IP protocol numbers.
pub mod ip_proto {
    pub const ICMP: u8 = 1;
    pub const TCP: u8 = 6;
    pub const UDP: u8 = 17;
}

/// Read a big-endian u16 from a byte slice.
#[inline]
pub fn read_be16(data: &[u8], offset: usize) -> u16 {
    ((data[offset] as u16) << 8) | data[offset + 1] as u16
}

/// Write a big-endian u16 to a byte slice.
#[inline]
pub fn write_be16(data: &mut [u8], offset: usize, val: u16) {
    data[offset] = (val >> 8) as u8;
    data[offset + 1] = val as u8;
}

/// Read a big-endian u32 from a byte slice.
#[inline]
pub fn read_be32(data: &[u8], offset: usize) -> u32 {
    ((data[offset] as u32) << 24)
        | ((data[offset + 1] as u32) << 16)
        | ((data[offset + 2] as u32) << 8)
        | data[offset + 3] as u32
}

/// Write a big-endian u32 to a byte slice.
#[inline]
pub fn write_be32(data: &mut [u8], offset: usize, val: u32) {
    data[offset] = (val >> 24) as u8;
    data[offset + 1] = (val >> 16) as u8;
    data[offset + 2] = (val >> 8) as u8;
    data[offset + 3] = val as u8;
}
