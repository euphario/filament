//! Unified Logging Framework
//!
//! Production-ready, structured logging for kernel and userspace.
//!
//! # Log Format
//! ```text
//! <ms> <LVL> <subsys> [ctx] <event> key=val ...
//! ```
//!
//! # Example
//! ```ignore
//! kinfo!("pcie", "probe_ok"; dev = "0000:01:00.0", vendor = klog::hex32(0x14c3));
//! kerror!("fw", [dev = "0001:01:00.0"], "load_failed"; err = "timeout", next = "reset");
//! ```

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

// Re-export from shared serialize module (for klog::hex32(...) etc.)
#[allow(unused_imports)]
pub use crate::serialize::{ValueType, Serialize, Hex32, Hex64, hex32, hex64};

// ============================================================================
// Log Levels
// ============================================================================

/// Log levels from most to least severe
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum Level {
    /// Operation failed, degraded behavior likely
    Error = 0,
    /// Unexpected but recoverable
    Warn = 1,
    /// Major lifecycle events (boot, init, shutdown)
    Info = 2,
    /// Diagnostic - state transitions, operations
    Debug = 3,
    /// Very verbose, per-operation
    Trace = 4,
}

impl Level {
    pub const fn as_str(self) -> &'static str {
        match self {
            Level::Error => "ERROR",
            Level::Warn => "WARN ",
            Level::Info => "INFO ",
            Level::Debug => "DEBUG",
            Level::Trace => "TRACE",
        }
    }

    /// ANSI color code for this level
    pub const fn color(self) -> &'static [u8] {
        match self {
            Level::Error => b"\x1b[1;31m",  // Bold red
            Level::Warn => b"\x1b[33m",     // Yellow
            Level::Info => b"\x1b[32m",     // Green
            Level::Debug => b"\x1b[2;37m",  // Dim white
            Level::Trace => b"\x1b[2;36m",  // Dim cyan
        }
    }

    pub const fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(Level::Error),
            1 => Some(Level::Warn),
            2 => Some(Level::Info),
            3 => Some(Level::Debug),
            4 => Some(Level::Trace),
            _ => None,
        }
    }
}

// ============================================================================
// Binary Record Format
// ============================================================================

/// Record header (8 bytes, aligned)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct RecordHeader {
    /// Total record length including header (for skipping)
    pub total_len: u16,
    /// Milliseconds since boot
    pub ts_ms: u32,
    /// Log level
    pub level: u8,
    /// Reserved/flags
    pub flags: u8,
}

impl RecordHeader {
    pub const SIZE: usize = 8;
}

// ============================================================================
// Ring Buffer
// ============================================================================

/// Ring buffer size (64KB)
const RING_SIZE: usize = 65536;

/// Maximum record size (512 bytes)
pub const MAX_RECORD_SIZE: usize = 512;

/// Ring buffer for log records (binary format)
pub struct LogRing {
    buffer: [u8; RING_SIZE],
    /// Write position
    head: AtomicU32,
    /// Read position
    tail: AtomicU32,
    /// Number of dropped messages (monotonic)
    dropped: AtomicU64,
    /// Message sequence number
    sequence: AtomicU64,
}

impl LogRing {
    pub const fn new() -> Self {
        Self {
            buffer: [0; RING_SIZE],
            head: AtomicU32::new(0),
            tail: AtomicU32::new(0),
            dropped: AtomicU64::new(0),
            sequence: AtomicU64::new(0),
        }
    }

    /// Write a record to the ring buffer
    /// Returns false if record was dropped (too large)
    pub fn write(&mut self, data: &[u8]) -> bool {
        let len = data.len();
        if len > MAX_RECORD_SIZE || len == 0 {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }

        // Align to 8 bytes
        let aligned_len = (len + 7) & !7;

        let head = self.head.load(Ordering::Relaxed) as usize;
        let tail = self.tail.load(Ordering::Acquire) as usize;

        // Calculate available space
        let available = if head >= tail {
            RING_SIZE - head + tail
        } else {
            tail - head
        };

        // Need space for data + length marker to avoid head==tail ambiguity
        if available <= aligned_len {
            // Overwrite oldest records until we have space
            self.make_space(aligned_len);
        }

        // Write data (may wrap around)
        let mut pos = head;
        for &byte in data {
            self.buffer[pos] = byte;
            pos = (pos + 1) % RING_SIZE;
        }
        // Zero padding for alignment
        for _ in len..aligned_len {
            self.buffer[pos] = 0;
            pos = (pos + 1) % RING_SIZE;
        }

        self.head.store(pos as u32, Ordering::Release);
        self.sequence.fetch_add(1, Ordering::Relaxed);

        // Notify any subscribed processes (like logd) that logs are available
        // This is safe to call even during early boot - broadcast_event handles
        // the case where no tasks exist yet
        #[cfg(not(test))]
        {
            let event = crate::kernel::event::Event::klog_ready();
            crate::kernel::event::broadcast_event(event);
        }

        true
    }

    /// Make space by advancing tail (dropping oldest records)
    fn make_space(&mut self, needed: usize) {
        let mut freed = 0;
        while freed < needed {
            let tail = self.tail.load(Ordering::Relaxed) as usize;
            let head = self.head.load(Ordering::Relaxed) as usize;

            if tail == head {
                // Buffer empty, nothing to drop
                break;
            }

            // Read record length from header
            let len_lo = self.buffer[tail] as u16;
            let len_hi = self.buffer[(tail + 1) % RING_SIZE] as u16;
            let record_len = (len_hi << 8) | len_lo;

            if record_len == 0 || record_len as usize > MAX_RECORD_SIZE {
                // Corrupted record, reset buffer
                self.tail.store(head as u32, Ordering::Release);
                break;
            }

            let aligned_len = ((record_len as usize) + 7) & !7;
            let new_tail = (tail + aligned_len) % RING_SIZE;
            self.tail.store(new_tail as u32, Ordering::Release);
            self.dropped.fetch_add(1, Ordering::Relaxed);
            freed += aligned_len;
        }
    }

    /// Read next record from the ring buffer
    /// Returns the record data, or None if empty
    pub fn read(&mut self, out: &mut [u8; MAX_RECORD_SIZE]) -> Option<usize> {
        let tail = self.tail.load(Ordering::Relaxed) as usize;
        let head = self.head.load(Ordering::Acquire) as usize;

        if tail == head {
            return None;
        }

        // Read record length from header
        let len_lo = self.buffer[tail] as u16;
        let len_hi = self.buffer[(tail + 1) % RING_SIZE] as u16;
        let record_len = (len_hi << 8) | len_lo;

        if record_len == 0 || record_len as usize > MAX_RECORD_SIZE {
            // Corrupted, skip to head
            self.tail.store(head as u32, Ordering::Release);
            return None;
        }

        let len = record_len as usize;
        let aligned_len = (len + 7) & !7;

        // Copy record data
        let mut pos = tail;
        for i in 0..len {
            out[i] = self.buffer[pos];
            pos = (pos + 1) % RING_SIZE;
        }

        // Advance tail past padding
        let new_tail = (tail + aligned_len) % RING_SIZE;
        self.tail.store(new_tail as u32, Ordering::Release);

        Some(len)
    }

    /// Get number of dropped messages
    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Get sequence number
    pub fn sequence(&self) -> u64 {
        self.sequence.load(Ordering::Relaxed)
    }

    /// Check if buffer has data
    pub fn has_data(&self) -> bool {
        self.tail.load(Ordering::Acquire) != self.head.load(Ordering::Acquire)
    }
}

// ============================================================================
// Global State
// ============================================================================

/// Global log ring buffer
pub static mut LOG_RING: LogRing = LogRing::new();

/// Boot time in counter ticks
static BOOT_TIME: AtomicU64 = AtomicU64::new(0);

/// Counter frequency in Hz
static COUNTER_FREQ: AtomicU64 = AtomicU64::new(24_000_000);

/// Current log level
static LOG_LEVEL: AtomicU32 = AtomicU32::new(Level::Info as u32);

/// Initialize the logging system
pub fn init() {
    let boot_time: u64;
    let counter_freq: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) boot_time);
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) counter_freq);
    }
    BOOT_TIME.store(boot_time, Ordering::Release);
    COUNTER_FREQ.store(counter_freq, Ordering::Release);
}

/// Get milliseconds since boot
#[inline]
pub fn timestamp_ms() -> u32 {
    let now: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) now);
    }
    let boot = BOOT_TIME.load(Ordering::Relaxed);
    let freq = COUNTER_FREQ.load(Ordering::Relaxed);
    let elapsed = now.wrapping_sub(boot);
    let divisor = freq / 1000;
    if divisor > 0 {
        (elapsed / divisor) as u32
    } else {
        (elapsed / 24_000) as u32
    }
}

/// Set the current log level
pub fn set_level(level: Level) {
    LOG_LEVEL.store(level as u32, Ordering::Release);
}

/// Get the current log level
pub fn get_level() -> Level {
    Level::from_u8(LOG_LEVEL.load(Ordering::Acquire) as u8).unwrap_or(Level::Info)
}

/// Check if a log level is enabled
#[inline]
pub fn is_enabled(level: Level) -> bool {
    (level as u32) <= LOG_LEVEL.load(Ordering::Relaxed)
}

// ============================================================================
// Record Builder
// ============================================================================

/// Builder for constructing log records
pub struct RecordBuilder {
    buffer: [u8; MAX_RECORD_SIZE],
    pos: usize,
}

impl RecordBuilder {
    pub fn new() -> Self {
        Self {
            buffer: [0; MAX_RECORD_SIZE],
            pos: RecordHeader::SIZE, // Leave space for header
        }
    }

    /// Write the header
    pub fn header(&mut self, level: Level) {
        let ts = timestamp_ms();
        // total_len will be filled in at finish()
        self.buffer[2] = ts as u8;
        self.buffer[3] = (ts >> 8) as u8;
        self.buffer[4] = (ts >> 16) as u8;
        self.buffer[5] = (ts >> 24) as u8;
        self.buffer[6] = level as u8;
        self.buffer[7] = 0; // flags
    }

    /// Write subsystem name (length-prefixed)
    pub fn subsys(&mut self, name: &str) {
        let len = name.len().min(63);
        self.buffer[self.pos] = len as u8;
        self.pos += 1;
        self.buffer[self.pos..self.pos + len].copy_from_slice(&name.as_bytes()[..len]);
        self.pos += len;
    }

    /// Write event name (length-prefixed)
    pub fn event(&mut self, name: &str) {
        let len = name.len().min(63);
        self.buffer[self.pos] = len as u8;
        self.pos += 1;
        self.buffer[self.pos..self.pos + len].copy_from_slice(&name.as_bytes()[..len]);
        self.pos += len;
    }

    /// Write context count (placeholder for now)
    pub fn ctx_count(&mut self, count: u8) {
        self.buffer[self.pos] = count;
        self.pos += 1;
    }

    /// Write key-value count (placeholder for now)
    pub fn kv_count(&mut self, count: u8) {
        self.buffer[self.pos] = count;
        self.pos += 1;
    }

    /// Write a key-value pair using the Serialize trait
    /// This works with any lifetime - strings are copied immediately
    pub fn kv<T: Serialize>(&mut self, key: &str, value: T) {
        let key_len = key.len().min(31);

        // Check space
        let needed = 2 + key_len + value.serialized_size();
        if self.pos + needed > MAX_RECORD_SIZE {
            return; // Drop this KV pair
        }

        // Key length
        self.buffer[self.pos] = key_len as u8;
        self.pos += 1;

        // Value type
        self.buffer[self.pos] = value.type_marker();
        self.pos += 1;

        // Key bytes
        self.buffer[self.pos..self.pos + key_len].copy_from_slice(&key.as_bytes()[..key_len]);
        self.pos += key_len;

        // Value bytes (serialized directly)
        self.pos += value.serialize(&mut self.buffer[self.pos..]);
    }

    /// Finish building and write to ring buffer
    pub fn finish(mut self) {
        // Write total length in header
        let total_len = self.pos as u16;
        self.buffer[0] = total_len as u8;
        self.buffer[1] = (total_len >> 8) as u8;

        // Write to ring buffer
        // SAFETY: Single-threaded kernel context (for now)
        unsafe {
            let ring = &mut *core::ptr::addr_of_mut!(LOG_RING);
            ring.write(&self.buffer[..self.pos]);
        }
    }
}

// ============================================================================
// Formatting (Drain)
// ============================================================================

/// ANSI reset code
const RESET: &[u8] = b"\x1b[0m";

/// Format a record to text output
/// Returns the number of bytes written
pub fn format_record(record: &[u8], out: &mut [u8]) -> usize {
    if record.len() < RecordHeader::SIZE + 4 {
        return 0;
    }

    let mut out_pos = 0;

    // Parse header
    let ts_ms = u32::from_le_bytes([record[2], record[3], record[4], record[5]]);
    let level = Level::from_u8(record[6]).unwrap_or(Level::Info);

    // Timestamp (dim)
    out[out_pos..out_pos + 4].copy_from_slice(b"\x1b[2m");
    out_pos += 4;
    out_pos += write_u32_padded(&mut out[out_pos..], ts_ms, 8);
    out[out_pos..out_pos + RESET.len()].copy_from_slice(RESET);
    out_pos += RESET.len();
    out[out_pos] = b' ';
    out_pos += 1;

    // Level with color
    let color = level.color();
    out[out_pos..out_pos + color.len()].copy_from_slice(color);
    out_pos += color.len();

    let lvl_str = level.as_str();
    out[out_pos..out_pos + 5].copy_from_slice(lvl_str.as_bytes());
    out_pos += 5;

    out[out_pos..out_pos + RESET.len()].copy_from_slice(RESET);
    out_pos += RESET.len();
    out[out_pos] = b' ';
    out_pos += 1;

    // Parse variable parts
    let mut pos = RecordHeader::SIZE;

    // Subsystem (cyan)
    if pos >= record.len() {
        return 0;
    }
    let subsys_len = record[pos] as usize;
    pos += 1;
    if pos + subsys_len > record.len() {
        return 0;
    }
    // Right-pad subsystem to 6 chars
    let subsys = &record[pos..pos + subsys_len];
    for _ in 0..(6 - subsys_len.min(6)) {
        out[out_pos] = b' ';
        out_pos += 1;
    }
    // Cyan color for subsystem
    out[out_pos..out_pos + 5].copy_from_slice(b"\x1b[36m");
    out_pos += 5;
    out[out_pos..out_pos + subsys_len.min(6)].copy_from_slice(&subsys[..subsys_len.min(6)]);
    out_pos += subsys_len.min(6);
    out[out_pos..out_pos + RESET.len()].copy_from_slice(RESET);
    out_pos += RESET.len();
    out[out_pos] = b' ';
    out_pos += 1;
    pos += subsys_len;

    // Event (bold)
    if pos >= record.len() {
        return 0;
    }
    let event_len = record[pos] as usize;
    pos += 1;
    if pos + event_len > record.len() {
        return 0;
    }
    let event = &record[pos..pos + event_len];
    out[out_pos..out_pos + 4].copy_from_slice(b"\x1b[1m");
    out_pos += 4;
    out[out_pos..out_pos + event_len].copy_from_slice(event);
    out_pos += event_len;
    out[out_pos..out_pos + RESET.len()].copy_from_slice(RESET);
    out_pos += RESET.len();
    pos += event_len;

    // Context count
    if pos >= record.len() {
        return out_pos;
    }
    let ctx_count = record[pos] as usize;
    pos += 1;

    // KV count
    if pos >= record.len() {
        return out_pos;
    }
    let kv_count = record[pos] as usize;
    pos += 1;

    // Format context KV pairs (in brackets if any)
    if ctx_count > 0 {
        out[out_pos] = b' ';
        out_pos += 1;
        out[out_pos] = b'[';
        out_pos += 1;

        for i in 0..ctx_count {
            if pos + 2 > record.len() {
                break;
            }
            let key_len = record[pos] as usize;
            let vtype = record[pos + 1];
            pos += 2;

            if pos + key_len > record.len() {
                break;
            }
            let key = &record[pos..pos + key_len];
            pos += key_len;

            // Key in dim
            out[out_pos..out_pos + 4].copy_from_slice(b"\x1b[2m");
            out_pos += 4;
            out[out_pos..out_pos + key_len].copy_from_slice(key);
            out_pos += key_len;
            out[out_pos] = b'=';
            out_pos += 1;
            out[out_pos..out_pos + RESET.len()].copy_from_slice(RESET);
            out_pos += RESET.len();

            // Value
            out_pos += format_value(vtype, &record[pos..], &mut out[out_pos..], &mut pos);

            if i < ctx_count - 1 {
                out[out_pos] = b' ';
                out_pos += 1;
            }
        }
        out[out_pos] = b']';
        out_pos += 1;
    }

    // Format data KV pairs
    for _i in 0..kv_count {
        if pos + 2 > record.len() {
            break;
        }
        let key_len = record[pos] as usize;
        let vtype = record[pos + 1];
        pos += 2;

        if pos + key_len > record.len() {
            break;
        }
        let key = &record[pos..pos + key_len];
        pos += key_len;

        out[out_pos] = b' ';
        out_pos += 1;

        // Empty key = raw output (no "key=" prefix), used by userspace logs
        if key_len > 0 {
            // Key in dim
            out[out_pos..out_pos + 4].copy_from_slice(b"\x1b[2m");
            out_pos += 4;
            out[out_pos..out_pos + key_len].copy_from_slice(key);
            out_pos += key_len;
            out[out_pos] = b'=';
            out_pos += 1;
            out[out_pos..out_pos + RESET.len()].copy_from_slice(RESET);
            out_pos += RESET.len();
        }

        // Value
        out_pos += format_value(vtype, &record[pos..], &mut out[out_pos..], &mut pos);
    }

    // Newline
    out[out_pos] = b'\r';
    out_pos += 1;
    out[out_pos] = b'\n';
    out_pos += 1;

    out_pos
}

/// Format a value from binary to text
fn format_value(vtype: u8, data: &[u8], out: &mut [u8], pos: &mut usize) -> usize {
    let mut out_pos = 0;

    match vtype {
        0 => {
            // U64
            if data.len() >= 8 {
                let v = u64::from_le_bytes([
                    data[0], data[1], data[2], data[3],
                    data[4], data[5], data[6], data[7],
                ]);
                out_pos += write_u64(&mut out[out_pos..], v);
                *pos += 8;
            }
        }
        1 => {
            // I64
            if data.len() >= 8 {
                let v = i64::from_le_bytes([
                    data[0], data[1], data[2], data[3],
                    data[4], data[5], data[6], data[7],
                ]);
                out_pos += write_i64(&mut out[out_pos..], v);
                *pos += 8;
            }
        }
        2 => {
            // Hex32
            if data.len() >= 4 {
                let v = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                out[out_pos] = b'0';
                out[out_pos + 1] = b'x';
                out_pos += 2;
                out_pos += write_hex32(&mut out[out_pos..], v);
                *pos += 4;
            }
        }
        3 => {
            // Hex64
            if data.len() >= 8 {
                let v = u64::from_le_bytes([
                    data[0], data[1], data[2], data[3],
                    data[4], data[5], data[6], data[7],
                ]);
                out[out_pos] = b'0';
                out[out_pos + 1] = b'x';
                out_pos += 2;
                out_pos += write_hex64(&mut out[out_pos..], v);
                *pos += 8;
            }
        }
        4 => {
            // Bool
            if !data.is_empty() {
                if data[0] != 0 {
                    out[out_pos..out_pos + 4].copy_from_slice(b"true");
                    out_pos += 4;
                } else {
                    out[out_pos..out_pos + 5].copy_from_slice(b"false");
                    out_pos += 5;
                }
                *pos += 1;
            }
        }
        5 => {
            // Str
            if !data.is_empty() {
                let slen = data[0] as usize;
                *pos += 1;
                if data.len() > slen {
                    out[out_pos..out_pos + slen].copy_from_slice(&data[1..1 + slen]);
                    out_pos += slen;
                    *pos += slen;
                }
            }
        }
        _ => {}
    }

    out_pos
}

// ============================================================================
// Number Formatting Helpers
// ============================================================================

fn write_u32_padded(out: &mut [u8], val: u32, width: usize) -> usize {
    let mut buf = [b'0'; 10];
    let mut v = val;
    let mut i = 9;
    loop {
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if v == 0 {
            break;
        }
        if i == 0 {
            break;
        }
        i -= 1;
    }
    let num_digits = 10 - i;
    let padding = width.saturating_sub(num_digits);

    // Write padding zeros
    for j in 0..padding {
        out[j] = b'0';
    }
    // Write digits
    out[padding..padding + num_digits].copy_from_slice(&buf[i..]);
    padding + num_digits
}

fn write_u64(out: &mut [u8], val: u64) -> usize {
    let mut buf = [b'0'; 20];
    let mut v = val;
    let mut i = 19;
    loop {
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if v == 0 {
            break;
        }
        if i == 0 {
            break;
        }
        i -= 1;
    }
    let len = 20 - i;
    out[..len].copy_from_slice(&buf[i..]);
    len
}

fn write_i64(out: &mut [u8], val: i64) -> usize {
    if val < 0 {
        out[0] = b'-';
        1 + write_u64(&mut out[1..], (-(val as i128)) as u64)
    } else {
        write_u64(out, val as u64)
    }
}

const HEX_CHARS: &[u8; 16] = b"0123456789abcdef";

fn write_hex32(out: &mut [u8], val: u32) -> usize {
    for i in 0..8 {
        let nibble = ((val >> (28 - i * 4)) & 0xF) as usize;
        out[i] = HEX_CHARS[nibble];
    }
    8
}

fn write_hex64(out: &mut [u8], val: u64) -> usize {
    for i in 0..16 {
        let nibble = ((val >> (60 - i * 4)) & 0xF) as usize;
        out[i] = HEX_CHARS[nibble];
    }
    16
}

// ============================================================================
// Drain (called from timer interrupt or explicit flush)
// ============================================================================

/// Drain one record from the ring buffer and output directly to UART
/// Returns true if a record was drained
pub fn drain_one() -> bool {
    let mut record_buf = [0u8; MAX_RECORD_SIZE];
    let mut text_buf = [0u8; 1024];

    // SAFETY: Single-threaded or with IRQs disabled
    let record_len = unsafe {
        let ring = &mut *core::ptr::addr_of_mut!(LOG_RING);
        ring.read(&mut record_buf)
    };

    let Some(len) = record_len else {
        return false;
    };

    let text_len = format_record(&record_buf[..len], &mut text_buf);
    if text_len > 0 {
        // Write directly to UART
        crate::platform::current::uart::write_bytes(&text_buf[..text_len]);
    }

    true
}

/// Flush all pending records to UART
pub fn flush() {
    while drain_one() {}
}

/// Try to drain records (non-blocking, for timer interrupt)
pub fn try_drain(max_records: usize) -> usize {
    let mut count = 0;
    for _ in 0..max_records {
        if !drain_one() {
            break;
        }
        count += 1;
    }
    count
}

/// Check if there are pending records
pub fn has_pending() -> bool {
    unsafe {
        let ring = &*core::ptr::addr_of!(LOG_RING);
        ring.has_data()
    }
}

/// Get number of dropped records
pub fn dropped() -> u64 {
    unsafe {
        let ring = &*core::ptr::addr_of!(LOG_RING);
        ring.dropped()
    }
}

// ============================================================================
// User Message Logging (for syscall::klog)
// ============================================================================

/// Log a userspace message via syscall
/// Parses [subsys] prefix if present, otherwise uses "user" as subsystem
/// First word becomes event, rest becomes key=value pairs (output directly)
pub fn log_user_message(level: Level, msg: &[u8]) {
    if !is_enabled(level) {
        return;
    }

    // Trim trailing newline/CR
    let mut end = msg.len();
    while end > 0 && (msg[end - 1] == b'\n' || msg[end - 1] == b'\r') {
        end -= 1;
    }
    let msg = &msg[..end];

    if msg.is_empty() {
        return;
    }

    // Try to parse [subsys] prefix
    let (subsys, text) = if msg.first() == Some(&b'[') {
        if let Some(bracket_end) = msg.iter().position(|&b| b == b']') {
            let subsys = &msg[1..bracket_end];
            let rest_start = if msg.len() > bracket_end + 1 && msg[bracket_end + 1] == b' ' {
                bracket_end + 2
            } else {
                bracket_end + 1
            };
            (subsys, &msg[rest_start..])
        } else {
            (b"user".as_slice(), msg)
        }
    } else {
        (b"user".as_slice(), msg)
    };

    // Extract first word as event, rest as raw kv text
    let (event, rest) = if let Some(space_pos) = text.iter().position(|&b| b == b' ') {
        (&text[..space_pos], &text[space_pos + 1..])
    } else {
        (text, &[][..])
    };

    // Build the log record
    let mut builder = RecordBuilder::new();
    builder.header(level);

    // Subsystem (convert bytes to string, truncate if invalid UTF-8)
    let subsys_str = core::str::from_utf8(subsys).unwrap_or("user");
    builder.subsys(subsys_str);

    // Event = first word of message
    let event_str = core::str::from_utf8(event).unwrap_or("msg");
    builder.event(event_str);

    // No context, remaining text as raw (special type 6 = raw string, no key prefix)
    builder.ctx_count(0);
    if rest.is_empty() {
        builder.kv_count(0);
    } else {
        builder.kv_count(1);
        // Use empty key "" to signal raw output (formatter will skip "key=")
        let rest_str = core::str::from_utf8(rest).unwrap_or("");
        builder.kv("", rest_str);
    }

    builder.finish();
}

// ============================================================================
// Logging Macros
// ============================================================================

/// Helper to start building a log record
#[doc(hidden)]
#[inline]
pub fn _start_record(level: Level, subsys: &str, event: &str, ctx_count: u8, kv_count: u8) -> RecordBuilder {
    let mut builder = RecordBuilder::new();
    builder.header(level);
    builder.subsys(subsys);
    builder.event(event);
    builder.ctx_count(ctx_count);
    builder.kv_count(kv_count);
    builder
}

/// Count macro arguments
#[doc(hidden)]
#[macro_export]
macro_rules! _klog_count {
    () => { 0u8 };
    ($head:tt $($tail:tt)*) => { 1u8 + $crate::_klog_count!($($tail)*) };
}

/// Log a message
///
/// # Examples
/// ```ignore
/// klog!(INFO, "pcie", "probe_ok"; vendor = 0x14c3, device = 0x7990);
/// klog!(ERROR, "fw", [dev = bdf], "load_failed"; err = "timeout");
/// ```
#[macro_export]
macro_rules! klog {
    // With context: klog!(LEVEL, "subsys", [ctx...], "event"; kvs...)
    ($level:ident, $subsys:expr, [$($ctx_key:ident = $ctx_val:expr),* $(,)?], $event:expr; $($key:ident = $val:expr),* $(,)?) => {{
        if $crate::klog::is_enabled($crate::klog::Level::$level) {
            let ctx_count = $crate::_klog_count!($($ctx_key)*);
            let kv_count = $crate::_klog_count!($($key)*);
            let mut builder = $crate::klog::_start_record(
                $crate::klog::Level::$level,
                $subsys,
                $event,
                ctx_count,
                kv_count,
            );
            $(
                builder.kv(stringify!($ctx_key), $ctx_val);
            )*
            $(
                builder.kv(stringify!($key), $val);
            )*
            builder.finish();
        }
    }};
    // Without context: klog!(LEVEL, "subsys", "event"; kvs...)
    ($level:ident, $subsys:expr, $event:expr; $($key:ident = $val:expr),* $(,)?) => {{
        if $crate::klog::is_enabled($crate::klog::Level::$level) {
            let kv_count = $crate::_klog_count!($($key)*);
            let mut builder = $crate::klog::_start_record(
                $crate::klog::Level::$level,
                $subsys,
                $event,
                0,
                kv_count,
            );
            $(
                builder.kv(stringify!($key), $val);
            )*
            builder.finish();
        }
    }};
    // Event only: klog!(LEVEL, "subsys", "event")
    ($level:ident, $subsys:expr, $event:expr) => {{
        if $crate::klog::is_enabled($crate::klog::Level::$level) {
            let builder = $crate::klog::_start_record(
                $crate::klog::Level::$level,
                $subsys,
                $event,
                0,
                0,
            );
            builder.finish();
        }
    }};
}

/// Convenience macro for INFO level
#[macro_export]
macro_rules! kinfo {
    ($($tt:tt)*) => { $crate::klog!(Info, $($tt)*) };
}

/// Convenience macro for ERROR level
#[macro_export]
macro_rules! kerror {
    ($($tt:tt)*) => { $crate::klog!(Error, $($tt)*) };
}

/// Convenience macro for WARN level
#[macro_export]
macro_rules! kwarn {
    ($($tt:tt)*) => { $crate::klog!(Warn, $($tt)*) };
}

/// Convenience macro for DEBUG level
#[macro_export]
macro_rules! kdebug {
    ($($tt:tt)*) => { $crate::klog!(Debug, $($tt)*) };
}

/// Convenience macro for TRACE level
#[macro_export]
macro_rules! ktrace {
    ($($tt:tt)*) => { $crate::klog!(Trace, $($tt)*) };
}

