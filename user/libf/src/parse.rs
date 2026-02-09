//! Number parsing from byte slices.

use crate::str::trim;

/// Parse a decimal u32 from a byte slice. Trims surrounding whitespace.
/// Returns `None` on empty input or non-digit characters.
pub fn parse_u32(input: &[u8]) -> Option<u32> {
    let s = trim(input);
    if s.is_empty() {
        return None;
    }
    let mut val: u32 = 0;
    for &ch in s {
        if ch >= b'0' && ch <= b'9' {
            val = val.checked_mul(10)?.checked_add((ch - b'0') as u32)?;
        } else {
            return None;
        }
    }
    Some(val)
}

/// Parse a decimal u64 from a byte slice. Trims surrounding whitespace.
pub fn parse_u64(input: &[u8]) -> Option<u64> {
    let s = trim(input);
    if s.is_empty() {
        return None;
    }
    let mut val: u64 = 0;
    for &ch in s {
        if ch >= b'0' && ch <= b'9' {
            val = val.checked_mul(10)?.checked_add((ch - b'0') as u64)?;
        } else {
            return None;
        }
    }
    Some(val)
}

/// Parse a hex u32 from a byte slice. Optional `0x`/`0X` prefix. Trims whitespace.
pub fn parse_hex_u32(input: &[u8]) -> Option<u32> {
    let s = trim(input);
    let s = strip_hex_prefix(s);
    if s.is_empty() {
        return None;
    }
    let mut val: u32 = 0;
    for &ch in s {
        let digit = hex_digit(ch)?;
        val = val.checked_mul(16)?.checked_add(digit as u32)?;
    }
    Some(val)
}

/// Parse a hex u64 from a byte slice. Optional `0x`/`0X` prefix. Trims whitespace.
pub fn parse_hex_u64(input: &[u8]) -> Option<u64> {
    let s = trim(input);
    let s = strip_hex_prefix(s);
    if s.is_empty() {
        return None;
    }
    let mut val: u64 = 0;
    for &ch in s {
        let digit = hex_digit(ch)?;
        val = val.checked_mul(16)?.checked_add(digit as u64)?;
    }
    Some(val)
}

#[inline]
fn strip_hex_prefix(s: &[u8]) -> &[u8] {
    if s.len() >= 2 && s[0] == b'0' && (s[1] == b'x' || s[1] == b'X') {
        &s[2..]
    } else {
        s
    }
}

#[inline]
fn hex_digit(ch: u8) -> Option<u8> {
    match ch {
        b'0'..=b'9' => Some(ch - b'0'),
        b'a'..=b'f' => Some(ch - b'a' + 10),
        b'A'..=b'F' => Some(ch - b'A' + 10),
        _ => None,
    }
}
