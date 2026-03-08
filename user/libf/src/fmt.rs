//! Formatting utilities — stack-allocated string buffers and number formatting.

use core::fmt;

/// A 64-byte stack buffer that implements `core::fmt::Write` and `Display`.
///
/// Use for lightweight number-to-string conversion without allocation:
/// ```ignore
/// let s = StackStr::from_u32(42);
/// console::write(s.as_bytes());
/// ```
pub struct StackStr {
    buf: [u8; 64],
    len: u8,
}

impl StackStr {
    /// Create an empty StackStr.
    pub const fn new() -> Self {
        Self { buf: [0u8; 64], len: 0 }
    }

    /// Format a u32 as decimal.
    pub fn from_u32(val: u32) -> Self {
        let mut s = Self::new();
        let n = format_u32_into(&mut s.buf, val);
        s.len = n as u8;
        s
    }

    /// Format a u64 as decimal.
    pub fn from_u64(val: u64) -> Self {
        let mut s = Self::new();
        let n = format_u64_into(&mut s.buf, val);
        s.len = n as u8;
        s
    }

    /// Format a u32 as hex with `0x` prefix.
    pub fn from_hex32(val: u32) -> Self {
        let mut s = Self::new();
        let n = format_hex32_into(&mut s.buf, val);
        s.len = n as u8;
        s
    }

    /// Format a u64 as hex with `0x` prefix.
    pub fn from_hex64(val: u64) -> Self {
        let mut s = Self::new();
        let n = format_hex64_into(&mut s.buf, val);
        s.len = n as u8;
        s
    }

    /// The formatted bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }

    /// The formatted string (always valid UTF-8 since we only produce ASCII).
    pub fn as_str(&self) -> &str {
        // Safety: we only write ASCII digits and 'x'/'a'-'f'
        unsafe { core::str::from_utf8_unchecked(&self.buf[..self.len as usize]) }
    }

    /// Length in bytes.
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// Whether empty.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl fmt::Display for StackStr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl fmt::Write for StackStr {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let avail = 64 - self.len as usize;
        let n = bytes.len().min(avail);
        self.buf[self.len as usize..self.len as usize + n].copy_from_slice(&bytes[..n]);
        self.len += n as u8;
        Ok(())
    }
}

/// Format a u32 as decimal into `buf`. Returns bytes written.
pub fn format_u32_into(buf: &mut [u8], val: u32) -> usize {
    if val == 0 {
        if !buf.is_empty() {
            buf[0] = b'0';
        }
        return 1;
    }
    let mut tmp = [0u8; 10];
    let mut pos = 0;
    let mut n = val;
    while n > 0 {
        tmp[pos] = b'0' + (n % 10) as u8;
        n /= 10;
        pos += 1;
    }
    let len = pos.min(buf.len());
    for i in 0..len {
        buf[i] = tmp[pos - 1 - i];
    }
    len
}

/// Format a u64 as decimal into `buf`. Returns bytes written.
pub fn format_u64_into(buf: &mut [u8], val: u64) -> usize {
    if val == 0 {
        if !buf.is_empty() {
            buf[0] = b'0';
        }
        return 1;
    }
    let mut tmp = [0u8; 20];
    let mut pos = 0;
    let mut n = val;
    while n > 0 {
        tmp[pos] = b'0' + (n % 10) as u8;
        n /= 10;
        pos += 1;
    }
    let len = pos.min(buf.len());
    for i in 0..len {
        buf[i] = tmp[pos - 1 - i];
    }
    len
}

/// Format a u32 as hex with `0x` prefix, minimal digits. Returns bytes written.
pub fn format_hex32_into(buf: &mut [u8], val: u32) -> usize {
    if buf.len() < 3 {
        return 0;
    }
    buf[0] = b'0';
    buf[1] = b'x';
    if val == 0 {
        buf[2] = b'0';
        return 3;
    }
    // Find first non-zero nibble
    let mut started = false;
    let mut pos = 2;
    for i in (0..8).rev() {
        let nibble = ((val >> (i * 4)) & 0xF) as u8;
        if nibble != 0 {
            started = true;
        }
        if started && pos < buf.len() {
            buf[pos] = if nibble < 10 { b'0' + nibble } else { b'a' + nibble - 10 };
            pos += 1;
        }
    }
    pos
}

/// A `core::fmt::Write` wrapper for `&mut [u8]` buffers.
///
/// Silently truncates on overflow — no panics, no errors. Use `finish()` to
/// get the number of bytes written.
///
/// ```ignore
/// let mut buf = [0u8; 128];
/// let mut w = BufWriter::new(&mut buf);
/// w.kv("channel", 6);
/// w.on_off(true);
/// let n = w.finish();
/// ```
pub struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> BufWriter<'a> {
    /// Create a new BufWriter over `buf`.
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Append raw bytes, silently truncating if full.
    pub fn put(&mut self, s: &[u8]) {
        let avail = self.buf.len() - self.pos;
        let n = s.len().min(avail);
        self.buf[self.pos..self.pos + n].copy_from_slice(&s[..n]);
        self.pos += n;
    }

    /// Write `key=val\n` where val is any `Display` type.
    pub fn kv(&mut self, key: &str, val: impl fmt::Display) {
        self.put(key.as_bytes());
        self.put(b"=");
        let _ = fmt::Write::write_fmt(self, format_args!("{}", val));
        self.put(b"\n");
    }

    /// Write `"on"` or `"off"`.
    pub fn on_off(&mut self, val: bool) {
        self.put(if val { b"on" } else { b"off" });
    }

    /// Write 6-byte MAC as 12 lowercase hex chars (no colons).
    pub fn mac(&mut self, addr: &[u8; 6]) {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        for &b in addr {
            self.put(&[HEX[(b >> 4) as usize], HEX[(b & 0xf) as usize]]);
        }
    }

    /// Bytes written so far.
    pub fn len(&self) -> usize {
        self.pos
    }

    /// Consume the writer, returning bytes written.
    pub fn finish(self) -> usize {
        self.pos
    }
}

impl fmt::Write for BufWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.put(s.as_bytes());
        Ok(())
    }
}

/// Format a u64 as hex with `0x` prefix, minimal digits. Returns bytes written.
pub fn format_hex64_into(buf: &mut [u8], val: u64) -> usize {
    if buf.len() < 3 {
        return 0;
    }
    buf[0] = b'0';
    buf[1] = b'x';
    if val == 0 {
        buf[2] = b'0';
        return 3;
    }
    let mut started = false;
    let mut pos = 2;
    for i in (0..16).rev() {
        let nibble = ((val >> (i * 4)) & 0xF) as u8;
        if nibble != 0 {
            started = true;
        }
        if started && pos < buf.len() {
            buf[pos] = if nibble < 10 { b'0' + nibble } else { b'a' + nibble - 10 };
            pos += 1;
        }
    }
    pos
}
