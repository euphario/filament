//! Position-tracking wire protocol reader/writer.
//!
//! Zero-copy, no-alloc cursor types for serializing and deserializing
//! query protocol messages. Every operation advances the position and
//! returns `None` on overflow — no silent truncation.

use crate::query::QueryHeader;

// =============================================================================
// WireReader
// =============================================================================

/// Zero-copy reader that tracks position and bounds-checks every read.
pub struct WireReader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> WireReader<'a> {
    /// Create a reader over the given buffer.
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Current read position.
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Bytes remaining after current position.
    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    /// Read a u8.
    pub fn u8(&mut self) -> Option<u8> {
        if self.pos >= self.buf.len() {
            return None;
        }
        let val = self.buf[self.pos];
        self.pos += 1;
        Some(val)
    }

    /// Read a little-endian u16.
    pub fn u16(&mut self) -> Option<u16> {
        if self.pos + 2 > self.buf.len() {
            return None;
        }
        let val = u16::from_le_bytes([self.buf[self.pos], self.buf[self.pos + 1]]);
        self.pos += 2;
        Some(val)
    }

    /// Read a little-endian u32.
    pub fn u32(&mut self) -> Option<u32> {
        let b = self.bytes(4)?;
        Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// Read a little-endian u64.
    pub fn u64(&mut self) -> Option<u64> {
        let b = self.bytes(8)?;
        Some(u64::from_le_bytes([
            b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
        ]))
    }

    /// Read a little-endian i32.
    pub fn i32(&mut self) -> Option<i32> {
        let b = self.bytes(4)?;
        Some(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// Borrow `len` bytes from the underlying buffer without copying.
    pub fn bytes(&mut self, len: usize) -> Option<&'a [u8]> {
        if self.pos + len > self.buf.len() {
            return None;
        }
        let slice = &self.buf[self.pos..self.pos + len];
        self.pos += len;
        Some(slice)
    }

    /// Read a u8 length prefix followed by that many bytes.
    pub fn len_u8_bytes(&mut self) -> Option<&'a [u8]> {
        let len = self.u8()? as usize;
        self.bytes(len)
    }

    /// Skip `n` bytes.
    pub fn skip(&mut self, n: usize) -> Option<()> {
        if self.pos + n > self.buf.len() {
            return None;
        }
        self.pos += n;
        Some(())
    }

    /// Create a sub-reader limited to the next `len` bytes.
    ///
    /// Advances this reader's position by `len`. The sub-reader can only
    /// access those `len` bytes — overflow in the sub-reader won't leak
    /// into the parent's remaining data.
    pub fn sub_reader(&mut self, len: usize) -> Option<WireReader<'a>> {
        let slice = self.bytes(len)?;
        Some(WireReader::new(slice))
    }

    // ── Query protocol helpers ──────────────────────────────────────────

    /// Read a QueryHeader (8 bytes).
    pub fn query_header(&mut self) -> Option<QueryHeader> {
        let msg_type = self.u16()?;
        let flags = self.u16()?;
        let seq_id = self.u32()?;
        Some(QueryHeader::with_flags(msg_type, flags, seq_id))
    }

    /// Read a KV list: count(u8) followed by (key_len(u8) + key + value_len(u8) + value) * count.
    ///
    /// Calls `visitor` for each (key, value) pair. Returns the number of pairs read.
    pub fn kv_list<F>(&mut self, mut visitor: F) -> Option<usize>
    where
        F: FnMut(&'a [u8], &'a [u8]),
    {
        let count = self.u8()? as usize;
        for _ in 0..count {
            let key = self.len_u8_bytes()?;
            let value = self.len_u8_bytes()?;
            visitor(key, value);
        }
        Some(count)
    }
}

// =============================================================================
// WireWriter
// =============================================================================

/// Position-tracking writer that bounds-checks every write.
pub struct WireWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> WireWriter<'a> {
    /// Create a writer over the given buffer.
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Current write position (= bytes written so far).
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Consume the writer, returning bytes written.
    pub fn finish(self) -> usize {
        self.pos
    }

    /// Write a u8.
    pub fn put_u8(&mut self, val: u8) -> Option<()> {
        if self.pos >= self.buf.len() {
            return None;
        }
        self.buf[self.pos] = val;
        self.pos += 1;
        Some(())
    }

    /// Write a little-endian u16.
    pub fn put_u16(&mut self, val: u16) -> Option<()> {
        let bytes = val.to_le_bytes();
        self.put_bytes(&bytes)
    }

    /// Write a little-endian u32.
    pub fn put_u32(&mut self, val: u32) -> Option<()> {
        let bytes = val.to_le_bytes();
        self.put_bytes(&bytes)
    }

    /// Write a little-endian u64.
    pub fn put_u64(&mut self, val: u64) -> Option<()> {
        let bytes = val.to_le_bytes();
        self.put_bytes(&bytes)
    }

    /// Write a little-endian i32.
    pub fn put_i32(&mut self, val: i32) -> Option<()> {
        let bytes = val.to_le_bytes();
        self.put_bytes(&bytes)
    }

    /// Write raw bytes.
    pub fn put_bytes(&mut self, data: &[u8]) -> Option<()> {
        if self.pos + data.len() > self.buf.len() {
            return None;
        }
        self.buf[self.pos..self.pos + data.len()].copy_from_slice(data);
        self.pos += data.len();
        Some(())
    }

    /// Write a u8 length prefix followed by the data bytes.
    ///
    /// Returns `None` if `data.len() > 255` or the buffer is too small.
    pub fn put_len_u8_bytes(&mut self, data: &[u8]) -> Option<()> {
        if data.len() > 255 {
            return None;
        }
        self.put_u8(data.len() as u8)?;
        self.put_bytes(data)
    }

    /// Write `data` padded with zeros to exactly `field_len` bytes.
    ///
    /// If `data` is longer than `field_len`, only the first `field_len` bytes are written.
    pub fn put_padded(&mut self, data: &[u8], field_len: usize) -> Option<()> {
        if self.pos + field_len > self.buf.len() {
            return None;
        }
        let copy_len = data.len().min(field_len);
        self.buf[self.pos..self.pos + copy_len].copy_from_slice(&data[..copy_len]);
        // Zero-fill remainder
        for i in copy_len..field_len {
            self.buf[self.pos + i] = 0;
        }
        self.pos += field_len;
        Some(())
    }

    /// Write `n` zero bytes.
    pub fn put_zeros(&mut self, n: usize) -> Option<()> {
        if self.pos + n > self.buf.len() {
            return None;
        }
        for i in 0..n {
            self.buf[self.pos + i] = 0;
        }
        self.pos += n;
        Some(())
    }

    // ── Query protocol helpers ──────────────────────────────────────────

    /// Write a QueryHeader (8 bytes).
    pub fn put_query_header(&mut self, h: &QueryHeader) -> Option<()> {
        self.put_u16(h.msg_type)?;
        self.put_u16(h.flags)?;
        self.put_u32(h.seq_id)
    }

    /// Write a KV list: count(u8) followed by (key_len(u8) + key + value_len(u8) + value) * count.
    ///
    /// Returns `None` if any key or value exceeds 255 bytes.
    pub fn put_kv_list(&mut self, kvs: &[(&[u8], &[u8])]) -> Option<()> {
        if kvs.len() > 255 {
            return None;
        }
        self.put_u8(kvs.len() as u8)?;
        for (k, v) in kvs {
            self.put_len_u8_bytes(k)?;
            self.put_len_u8_bytes(v)?;
        }
        Some(())
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_primitives() {
        let mut buf = [0u8; 64];
        let mut w = WireWriter::new(&mut buf);
        w.put_u8(0x42).unwrap();
        w.put_u16(0x1234).unwrap();
        w.put_u32(0xDEADBEEF).unwrap();
        w.put_u64(0x0102030405060708).unwrap();
        w.put_i32(-1).unwrap();
        let written = w.finish();
        assert_eq!(written, 1 + 2 + 4 + 8 + 4);

        let mut r = WireReader::new(&buf[..written]);
        assert_eq!(r.u8(), Some(0x42));
        assert_eq!(r.u16(), Some(0x1234));
        assert_eq!(r.u32(), Some(0xDEADBEEF));
        assert_eq!(r.u64(), Some(0x0102030405060708));
        assert_eq!(r.i32(), Some(-1));
        assert_eq!(r.remaining(), 0);
    }

    #[test]
    fn reader_overflow() {
        let buf = [0x01, 0x02];
        let mut r = WireReader::new(&buf);
        assert_eq!(r.u8(), Some(0x01));
        assert_eq!(r.u16(), None); // only 1 byte left, need 2
        assert_eq!(r.position(), 1); // position unchanged after failed read
    }

    #[test]
    fn writer_overflow() {
        let mut buf = [0u8; 3];
        let mut w = WireWriter::new(&mut buf);
        w.put_u16(0x1234).unwrap();
        assert_eq!(w.put_u16(0x5678), None); // only 1 byte left
        assert_eq!(w.position(), 2);
    }

    #[test]
    fn bytes_zero_copy() {
        let buf = [10, 20, 30, 40, 50];
        let mut r = WireReader::new(&buf);
        let slice = r.bytes(3).unwrap();
        assert_eq!(slice, &[10, 20, 30]);
        assert_eq!(r.remaining(), 2);
    }

    #[test]
    fn len_u8_bytes_round_trip() {
        let mut buf = [0u8; 32];
        let data = b"hello";
        let mut w = WireWriter::new(&mut buf);
        w.put_len_u8_bytes(data).unwrap();
        let written = w.finish();

        let mut r = WireReader::new(&buf[..written]);
        let read_back = r.len_u8_bytes().unwrap();
        assert_eq!(read_back, b"hello");
    }

    #[test]
    fn len_u8_bytes_rejects_oversized() {
        let mut buf = [0u8; 512];
        let data = [0u8; 256]; // 256 > 255
        let mut w = WireWriter::new(&mut buf);
        assert_eq!(w.put_len_u8_bytes(&data), None);
    }

    #[test]
    fn kv_list_round_trip_empty() {
        let mut buf = [0u8; 32];
        let mut w = WireWriter::new(&mut buf);
        w.put_kv_list(&[]).unwrap();
        let written = w.finish();

        let mut r = WireReader::new(&buf[..written]);
        let mut count = 0;
        r.kv_list(|_, _| count += 1).unwrap();
        assert_eq!(count, 0);
    }

    #[test]
    fn kv_list_round_trip() {
        let mut buf = [0u8; 128];
        let kvs: &[(&[u8], &[u8])] = &[
            (b"key1", b"val1"),
            (b"k2", b"value_two"),
            (b"abc", b""),
            (b"", b"x"),
        ];
        let mut w = WireWriter::new(&mut buf);
        w.put_kv_list(kvs).unwrap();
        let written = w.finish();

        let mut r = WireReader::new(&buf[..written]);
        let mut pairs: Vec<(&[u8], &[u8])> = Vec::new();
        let n = r.kv_list(|k, v| pairs.push((k, v))).unwrap();
        assert_eq!(n, 4);
        assert_eq!(pairs[0], (b"key1".as_slice(), b"val1".as_slice()));
        assert_eq!(pairs[1], (b"k2".as_slice(), b"value_two".as_slice()));
        assert_eq!(pairs[2], (b"abc".as_slice(), b"".as_slice()));
        assert_eq!(pairs[3], (b"".as_slice(), b"x".as_slice()));
    }

    #[test]
    fn sub_reader_bounds_isolation() {
        let buf = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut r = WireReader::new(&buf);
        r.u8().unwrap(); // consume 1 byte

        let mut sub = r.sub_reader(3).unwrap(); // bytes [2, 3, 4]
        assert_eq!(sub.u8(), Some(2));
        assert_eq!(sub.u8(), Some(3));
        assert_eq!(sub.u8(), Some(4));
        assert_eq!(sub.u8(), None); // sub-reader exhausted

        // Parent reader continues from byte 5
        assert_eq!(r.u8(), Some(5));
        assert_eq!(r.remaining(), 3);
    }

    #[test]
    fn query_header_round_trip() {
        let mut buf = [0u8; 16];
        let hdr = QueryHeader::with_flags(0x0107, 0x0004, 42);
        let mut w = WireWriter::new(&mut buf);
        w.put_query_header(&hdr).unwrap();
        let written = w.finish();
        assert_eq!(written, 8);

        let mut r = WireReader::new(&buf[..written]);
        let parsed = r.query_header().unwrap();
        assert_eq!(parsed.msg_type, 0x0107);
        assert_eq!(parsed.flags, 0x0004);
        assert_eq!(parsed.seq_id, 42);
    }

    #[test]
    fn put_padded() {
        let mut buf = [0xFF; 16];
        let mut w = WireWriter::new(&mut buf);
        w.put_padded(b"hi", 8).unwrap();
        let written = w.finish();
        assert_eq!(written, 8);
        assert_eq!(&buf[..8], &[b'h', b'i', 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn put_zeros() {
        let mut buf = [0xFF; 8];
        let mut w = WireWriter::new(&mut buf);
        w.put_u8(0x42).unwrap();
        w.put_zeros(3).unwrap();
        assert_eq!(&buf[..4], &[0x42, 0, 0, 0]);
    }

    #[test]
    fn skip() {
        let buf = [1, 2, 3, 4, 5];
        let mut r = WireReader::new(&buf);
        r.skip(3).unwrap();
        assert_eq!(r.u8(), Some(4));
        assert_eq!(r.skip(5), None); // only 1 byte left
    }
}
