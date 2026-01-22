//! Unified Logging Framework (Userspace)
//!
//! Same format as kernel klog, drained via syscall.
//!
//! # Log Format
//! ```text
//! <ms> <LVL> <subsys> [ctx] <event> key=val ...
//! ```
//!
//! # Example
//! ```ignore
//! uinfo!("pcie", "probe_ok"; vendor = ulog::hex32(0x14c3));
//! uerror!("fw", [dev = "0001:01:00.0"], "load_failed"; err = "timeout");
//! ```

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

// Re-export from shared serialize module
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
    pub total_len: u16,
    pub ts_ms: u32,
    pub level: u8,
    pub flags: u8,
}

impl RecordHeader {
    pub const SIZE: usize = 8;
}

// ============================================================================
// Ring Buffer
// ============================================================================

/// Ring buffer size (32KB for userspace - smaller than kernel)
const RING_SIZE: usize = 32768;

/// Maximum record size (512 bytes)
pub const MAX_RECORD_SIZE: usize = 512;

/// Ring buffer for log records (binary format)
pub struct LogRing {
    buffer: [u8; RING_SIZE],
    head: AtomicU32,
    tail: AtomicU32,
    dropped: AtomicU64,
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

    pub fn write(&mut self, data: &[u8]) -> bool {
        let len = data.len();
        if len > MAX_RECORD_SIZE || len == 0 {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }

        let aligned_len = (len + 7) & !7;

        let head = self.head.load(Ordering::Relaxed) as usize;
        let tail = self.tail.load(Ordering::Acquire) as usize;

        let available = if head >= tail {
            RING_SIZE - head + tail
        } else {
            tail - head
        };

        if available <= aligned_len {
            self.make_space(aligned_len);
        }

        let mut pos = head;
        for &byte in data {
            self.buffer[pos] = byte;
            pos = (pos + 1) % RING_SIZE;
        }
        for _ in len..aligned_len {
            self.buffer[pos] = 0;
            pos = (pos + 1) % RING_SIZE;
        }

        self.head.store(pos as u32, Ordering::Release);
        self.sequence.fetch_add(1, Ordering::Relaxed);
        true
    }

    fn make_space(&mut self, needed: usize) {
        let mut freed = 0;
        while freed < needed {
            let tail = self.tail.load(Ordering::Relaxed) as usize;
            let head = self.head.load(Ordering::Relaxed) as usize;

            if tail == head {
                break;
            }

            let len_lo = self.buffer[tail] as u16;
            let len_hi = self.buffer[(tail + 1) % RING_SIZE] as u16;
            let record_len = (len_hi << 8) | len_lo;

            if record_len == 0 || record_len as usize > MAX_RECORD_SIZE {
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

    pub fn read(&mut self, out: &mut [u8; MAX_RECORD_SIZE]) -> Option<usize> {
        let tail = self.tail.load(Ordering::Relaxed) as usize;
        let head = self.head.load(Ordering::Acquire) as usize;

        if tail == head {
            return None;
        }

        let len_lo = self.buffer[tail] as u16;
        let len_hi = self.buffer[(tail + 1) % RING_SIZE] as u16;
        let record_len = (len_hi << 8) | len_lo;

        if record_len == 0 || record_len as usize > MAX_RECORD_SIZE {
            self.tail.store(head as u32, Ordering::Release);
            return None;
        }

        let len = record_len as usize;
        let aligned_len = (len + 7) & !7;

        let mut pos = tail;
        for i in 0..len {
            out[i] = self.buffer[pos];
            pos = (pos + 1) % RING_SIZE;
        }

        let new_tail = (tail + aligned_len) % RING_SIZE;
        self.tail.store(new_tail as u32, Ordering::Release);

        Some(len)
    }

    pub fn has_data(&self) -> bool {
        self.tail.load(Ordering::Acquire) != self.head.load(Ordering::Acquire)
    }

    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

// ============================================================================
// Global State
// ============================================================================

static mut LOG_RING: LogRing = LogRing::new();

/// Boot time (set from kernel via shared memory or estimate from first log)
static BOOT_TIME: AtomicU64 = AtomicU64::new(0);

/// Current log level
static LOG_LEVEL: AtomicU32 = AtomicU32::new(Level::Info as u32);

/// Stdout output enabled (can be disabled to prevent console corruption)
static STDOUT_ENABLED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(true);

/// Disable stdout output (logs still go to ring buffer)
pub fn disable_stdout() {
    STDOUT_ENABLED.store(false, Ordering::Release);
}

/// Enable stdout output
pub fn enable_stdout() {
    STDOUT_ENABLED.store(true, Ordering::Release);
}

/// Initialize userspace logging
/// Call this at program start
pub fn init() {
    // For now, just reset boot time. Could read from shared memory later.
    BOOT_TIME.store(0, Ordering::Release);
}

/// Get milliseconds since program start (approximation until we have shared boot time)
#[inline]
pub fn timestamp_ms() -> u32 {
    // Read ARM counter
    let now: u64;
    let freq: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) now);
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
    }
    let boot = BOOT_TIME.load(Ordering::Relaxed);

    // If boot time not set, set it now (first log call)
    if boot == 0 {
        BOOT_TIME.compare_exchange(0, now, Ordering::Release, Ordering::Relaxed).ok();
        return 0;
    }

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

pub struct RecordBuilder {
    buffer: [u8; MAX_RECORD_SIZE],
    pos: usize,
}

impl RecordBuilder {
    pub fn new() -> Self {
        Self {
            buffer: [0; MAX_RECORD_SIZE],
            pos: RecordHeader::SIZE,
        }
    }

    pub fn header(&mut self, level: Level) {
        let ts = timestamp_ms();
        self.buffer[2] = ts as u8;
        self.buffer[3] = (ts >> 8) as u8;
        self.buffer[4] = (ts >> 16) as u8;
        self.buffer[5] = (ts >> 24) as u8;
        self.buffer[6] = level as u8;
        self.buffer[7] = 0;
    }

    pub fn subsys(&mut self, name: &str) {
        let len = name.len().min(63);
        self.buffer[self.pos] = len as u8;
        self.pos += 1;
        self.buffer[self.pos..self.pos + len].copy_from_slice(&name.as_bytes()[..len]);
        self.pos += len;
    }

    pub fn event(&mut self, name: &str) {
        let len = name.len().min(63);
        self.buffer[self.pos] = len as u8;
        self.pos += 1;
        self.buffer[self.pos..self.pos + len].copy_from_slice(&name.as_bytes()[..len]);
        self.pos += len;
    }

    pub fn ctx_count(&mut self, count: u8) {
        self.buffer[self.pos] = count;
        self.pos += 1;
    }

    pub fn kv_count(&mut self, count: u8) {
        self.buffer[self.pos] = count;
        self.pos += 1;
    }

    /// Write a key-value pair using the Serialize trait
    /// This works with any lifetime - strings are copied immediately
    pub fn kv<T: Serialize>(&mut self, key: &str, value: T) {
        let key_len = key.len().min(31);
        let needed = 2 + key_len + value.serialized_size();
        if self.pos + needed > MAX_RECORD_SIZE {
            return;
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

    pub fn finish(mut self) {
        let total_len = self.pos as u16;
        self.buffer[0] = total_len as u8;
        self.buffer[1] = (total_len >> 8) as u8;

        unsafe {
            let ring = &mut *core::ptr::addr_of_mut!(LOG_RING);
            ring.write(&self.buffer[..self.pos]);
        }
    }
}

// ============================================================================
// Formatting (same as kernel)
// ============================================================================

/// ANSI reset code
const RESET: &[u8] = b"\x1b[0m";

pub fn format_record(record: &[u8], out: &mut [u8]) -> usize {
    if record.len() < RecordHeader::SIZE + 4 {
        return 0;
    }

    let mut out_pos = 0;

    let ts_ms = u32::from_le_bytes([record[2], record[3], record[4], record[5]]);
    let level = Level::from_u8(record[6]).unwrap_or(Level::Info);

    // Timestamp (dim)
    out[out_pos..out_pos + 4].copy_from_slice(b"\x1b[2m");
    out_pos += 4;
    out_pos += write_u32_padded(&mut out[out_pos..], ts_ms, 8);
    out[out_pos..out_pos + 4].copy_from_slice(RESET);
    out_pos += 4;
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

    let mut pos = RecordHeader::SIZE;

    // Subsystem (cyan)
    if pos >= record.len() { return 0; }
    let subsys_len = record[pos] as usize;
    pos += 1;
    if pos + subsys_len > record.len() { return 0; }
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
    if pos >= record.len() { return 0; }
    let event_len = record[pos] as usize;
    pos += 1;
    if pos + event_len > record.len() { return 0; }
    let event = &record[pos..pos + event_len];
    out[out_pos..out_pos + 4].copy_from_slice(b"\x1b[1m");
    out_pos += 4;
    out[out_pos..out_pos + event_len].copy_from_slice(event);
    out_pos += event_len;
    out[out_pos..out_pos + RESET.len()].copy_from_slice(RESET);
    out_pos += RESET.len();
    pos += event_len;

    // Context count
    if pos >= record.len() { return out_pos; }
    let ctx_count = record[pos] as usize;
    pos += 1;

    // KV count
    if pos >= record.len() { return out_pos; }
    let kv_count = record[pos] as usize;
    pos += 1;

    // Format context
    if ctx_count > 0 {
        out[out_pos] = b' ';
        out_pos += 1;
        out[out_pos] = b'[';
        out_pos += 1;

        for i in 0..ctx_count {
            if pos + 2 > record.len() { break; }
            let key_len = record[pos] as usize;
            let vtype = record[pos + 1];
            pos += 2;

            if pos + key_len > record.len() { break; }
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

            out_pos += format_value(vtype, &record[pos..], &mut out[out_pos..], &mut pos);

            if i < ctx_count - 1 {
                out[out_pos] = b' ';
                out_pos += 1;
            }
        }
        out[out_pos] = b']';
        out_pos += 1;
    }

    // Format data KVs
    for _i in 0..kv_count {
        if pos + 2 > record.len() { break; }
        let key_len = record[pos] as usize;
        let vtype = record[pos + 1];
        pos += 2;

        if pos + key_len > record.len() { break; }
        let key = &record[pos..pos + key_len];
        pos += key_len;

        out[out_pos] = b' ';
        out_pos += 1;

        // Key in dim
        out[out_pos..out_pos + 4].copy_from_slice(b"\x1b[2m");
        out_pos += 4;
        out[out_pos..out_pos + key_len].copy_from_slice(key);
        out_pos += key_len;
        out[out_pos] = b'=';
        out_pos += 1;
        out[out_pos..out_pos + RESET.len()].copy_from_slice(RESET);
        out_pos += RESET.len();

        out_pos += format_value(vtype, &record[pos..], &mut out[out_pos..], &mut pos);
    }

    out[out_pos] = b'\r';
    out_pos += 1;
    out[out_pos] = b'\n';
    out_pos += 1;

    out_pos
}

fn format_value(vtype: u8, data: &[u8], out: &mut [u8], pos: &mut usize) -> usize {
    let mut out_pos = 0;

    match vtype {
        0 => { // U64
            if data.len() >= 8 {
                let v = u64::from_le_bytes([
                    data[0], data[1], data[2], data[3],
                    data[4], data[5], data[6], data[7],
                ]);
                out_pos += write_u64(&mut out[out_pos..], v);
                *pos += 8;
            }
        }
        1 => { // I64
            if data.len() >= 8 {
                let v = i64::from_le_bytes([
                    data[0], data[1], data[2], data[3],
                    data[4], data[5], data[6], data[7],
                ]);
                out_pos += write_i64(&mut out[out_pos..], v);
                *pos += 8;
            }
        }
        2 => { // Hex32
            if data.len() >= 4 {
                let v = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                out[out_pos] = b'0';
                out[out_pos + 1] = b'x';
                out_pos += 2;
                out_pos += write_hex32(&mut out[out_pos..], v);
                *pos += 4;
            }
        }
        3 => { // Hex64
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
        4 => { // Bool
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
        5 => { // Str
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
// Number Formatting
// ============================================================================

fn write_u32_padded(out: &mut [u8], val: u32, width: usize) -> usize {
    let mut buf = [b'0'; 10];
    let mut v = val;
    let mut i = 9;
    loop {
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if v == 0 { break; }
        if i == 0 { break; }
        i -= 1;
    }
    let num_digits = 10 - i;
    let padding = width.saturating_sub(num_digits);
    for j in 0..padding {
        out[j] = b'0';
    }
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
        if v == 0 { break; }
        if i == 0 { break; }
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
// Drain
// ============================================================================

/// Drain one record and write to stdout (if enabled)
pub fn drain_one() -> bool {
    let mut record_buf = [0u8; MAX_RECORD_SIZE];
    let mut text_buf = [0u8; 1024];

    let record_len = unsafe {
        let ring = &mut *core::ptr::addr_of_mut!(LOG_RING);
        ring.read(&mut record_buf)
    };

    let Some(len) = record_len else {
        return false;
    };

    // Only write to stdout if enabled
    if STDOUT_ENABLED.load(Ordering::Acquire) {
        let text_len = format_record(&record_buf[..len], &mut text_buf);
        if text_len > 0 {
            let _ = crate::syscall::write(crate::syscall::Handle::STDOUT, &text_buf[..text_len]);
        }
    }

    true
}

/// Flush all pending records
pub fn flush() {
    while drain_one() {}
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
// Logging API
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
macro_rules! _ulog_count {
    () => { 0u8 };
    ($head:tt $($tail:tt)*) => { 1u8 + $crate::_ulog_count!($($tail)*) };
}

/// Log a message (userspace)
#[macro_export]
macro_rules! ulog {
    // With context
    ($level:ident, $subsys:expr, [$($ctx_key:ident = $ctx_val:expr),* $(,)?], $event:expr; $($key:ident = $val:expr),* $(,)?) => {{
        if $crate::ulog::is_enabled($crate::ulog::Level::$level) {
            let ctx_count = $crate::_ulog_count!($($ctx_key)*);
            let kv_count = $crate::_ulog_count!($($key)*);
            let mut builder = $crate::ulog::_start_record(
                $crate::ulog::Level::$level,
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
    // Without context
    ($level:ident, $subsys:expr, $event:expr; $($key:ident = $val:expr),* $(,)?) => {{
        if $crate::ulog::is_enabled($crate::ulog::Level::$level) {
            let kv_count = $crate::_ulog_count!($($key)*);
            let mut builder = $crate::ulog::_start_record(
                $crate::ulog::Level::$level,
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
    // Event only
    ($level:ident, $subsys:expr, $event:expr) => {{
        if $crate::ulog::is_enabled($crate::ulog::Level::$level) {
            let mut builder = $crate::ulog::_start_record(
                $crate::ulog::Level::$level,
                $subsys,
                $event,
                0,
                0,
            );
            builder.finish();
        }
    }};
}

#[macro_export]
macro_rules! uinfo {
    ($($tt:tt)*) => { $crate::ulog!(Info, $($tt)*) };
}

#[macro_export]
macro_rules! uerror {
    ($($tt:tt)*) => { $crate::ulog!(Error, $($tt)*) };
}

#[macro_export]
macro_rules! uwarn {
    ($($tt:tt)*) => { $crate::ulog!(Warn, $($tt)*) };
}

#[macro_export]
macro_rules! udebug {
    ($($tt:tt)*) => { $crate::ulog!(Debug, $($tt)*) };
}

#[macro_export]
macro_rules! utrace {
    ($($tt:tt)*) => { $crate::ulog!(Trace, $($tt)*) };
}

