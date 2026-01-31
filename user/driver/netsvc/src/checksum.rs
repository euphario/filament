//! Internet Checksum (RFC 1071)
//!
//! One's complement sum used by IP, ICMP, TCP, and UDP headers.

/// Compute the internet checksum over a byte slice.
///
/// Returns the 16-bit one's complement of the one's complement sum.
pub fn internet_checksum(data: &[u8]) -> u16 {
    let sum = checksum_partial(data, 0);
    fold(sum)
}

/// Accumulate a partial checksum (for building checksums over multiple regions).
///
/// `initial` is the running sum from a previous call (or 0 to start fresh).
pub fn checksum_partial(data: &[u8], initial: u32) -> u32 {
    let mut sum = initial;
    let mut i = 0;
    let len = data.len();

    // Process 16-bit words
    while i + 1 < len {
        sum += ((data[i] as u32) << 8) | data[i + 1] as u32;
        i += 2;
    }

    // Handle odd trailing byte
    if i < len {
        sum += (data[i] as u32) << 8;
    }

    sum
}

/// Fold a 32-bit partial sum into a 16-bit one's complement checksum.
pub fn fold(mut sum: u32) -> u16 {
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    !(sum as u16)
}

/// Verify a checksum: returns true if the data (including checksum field) sums to zero.
pub fn verify(data: &[u8]) -> bool {
    let sum = checksum_partial(data, 0);
    fold(sum) == 0
}
