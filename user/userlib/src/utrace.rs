//! Userspace Tracing Framework (utrace)
//!
//! Same API as kernel ktrace, drained via syscall.
//!
//! # Example
//! ```ignore
//! fn load_firmware(&mut self) -> Result<()> {
//!     let _span = uspan!("fw", "load_firmware"; dev = "0001:01:00.0");
//!     self.load_patch()?;
//!     Ok(())
//! }
//! ```

use core::sync::atomic::{AtomicU8, AtomicU32, AtomicU64, Ordering};

// Re-export from shared serialize module
pub use crate::serialize::{ValueType, Serialize, Hex32, Hex64, hex32, hex64};

// ============================================================================
// Trace Event Format
// ============================================================================

pub mod flags {
    pub const ENTER: u8 = 0;
    pub const EXIT: u8 = 1 << 6;
    pub const ERROR: u8 = 1 << 7;
}

pub const MAX_EVENT_SIZE: usize = 256;

// ============================================================================
// Ring Buffer
// ============================================================================

const RING_SIZE: usize = 32768;

struct TraceRing {
    buffer: [u8; RING_SIZE],
    head: AtomicU32,
    tail: AtomicU32,
    dropped: AtomicU64,
}

impl TraceRing {
    const fn new() -> Self {
        Self {
            buffer: [0; RING_SIZE],
            head: AtomicU32::new(0),
            tail: AtomicU32::new(0),
            dropped: AtomicU64::new(0),
        }
    }

    fn write(&mut self, data: &[u8]) -> bool {
        let len = data.len();
        if len > MAX_EVENT_SIZE || len == 0 {
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
        true
    }

    fn make_space(&mut self, needed: usize) {
        let mut freed = 0;
        while freed < needed {
            let tail = self.tail.load(Ordering::Relaxed) as usize;
            let head = self.head.load(Ordering::Relaxed) as usize;

            if tail == head { break; }

            let len_lo = self.buffer[tail] as u16;
            let len_hi = self.buffer[(tail + 1) % RING_SIZE] as u16;
            let event_len = (len_hi << 8) | len_lo;

            if event_len == 0 || event_len as usize > MAX_EVENT_SIZE {
                self.tail.store(head as u32, Ordering::Release);
                break;
            }

            let aligned_len = ((event_len as usize) + 7) & !7;
            let new_tail = (tail + aligned_len) % RING_SIZE;
            self.tail.store(new_tail as u32, Ordering::Release);
            self.dropped.fetch_add(1, Ordering::Relaxed);
            freed += aligned_len;
        }
    }

    fn read(&mut self, out: &mut [u8; MAX_EVENT_SIZE]) -> Option<usize> {
        let tail = self.tail.load(Ordering::Relaxed) as usize;
        let head = self.head.load(Ordering::Acquire) as usize;

        if tail == head { return None; }

        let len_lo = self.buffer[tail] as u16;
        let len_hi = self.buffer[(tail + 1) % RING_SIZE] as u16;
        let event_len = (len_hi << 8) | len_lo;

        if event_len == 0 || event_len as usize > MAX_EVENT_SIZE {
            self.tail.store(head as u32, Ordering::Release);
            return None;
        }

        let len = event_len as usize;
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

    fn has_data(&self) -> bool {
        self.tail.load(Ordering::Acquire) != self.head.load(Ordering::Acquire)
    }
}

// ============================================================================
// Global State
// ============================================================================

static mut TRACE_RING: TraceRing = TraceRing::new();
static TRACE_DEPTH: AtomicU8 = AtomicU8::new(0);
static BOOT_TIME: AtomicU64 = AtomicU64::new(0);
static TRACING_ENABLED: AtomicU8 = AtomicU8::new(1);

pub fn init() {
    BOOT_TIME.store(0, Ordering::Release);
}

#[inline]
pub fn timestamp_us() -> u32 {
    let now: u64;
    let freq: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) now);
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq);
    }
    let boot = BOOT_TIME.load(Ordering::Relaxed);

    if boot == 0 {
        BOOT_TIME.compare_exchange(0, now, Ordering::Release, Ordering::Relaxed).ok();
        return 0;
    }

    let elapsed = now.wrapping_sub(boot);
    let divisor = freq / 1_000_000;
    if divisor > 0 {
        (elapsed / divisor) as u32
    } else {
        (elapsed / 24) as u32
    }
}

pub fn set_enabled(enabled: bool) {
    TRACING_ENABLED.store(if enabled { 1 } else { 0 }, Ordering::Release);
}

#[inline]
pub fn is_enabled() -> bool {
    TRACING_ENABLED.load(Ordering::Relaxed) != 0
}

fn increment_depth() -> u8 {
    TRACE_DEPTH.fetch_add(1, Ordering::Relaxed)
}

fn decrement_depth() {
    TRACE_DEPTH.fetch_sub(1, Ordering::Relaxed);
}

fn current_depth() -> u8 {
    TRACE_DEPTH.load(Ordering::Relaxed)
}

// ============================================================================
// Event Builder
// ============================================================================

#[doc(hidden)]
pub struct EventBuilder {
    buffer: [u8; MAX_EVENT_SIZE],
    pos: usize,
}

impl EventBuilder {
    pub fn new() -> Self {
        Self {
            buffer: [0; MAX_EVENT_SIZE],
            pos: 8,
        }
    }

    pub fn header(&mut self, depth: u8, is_exit: bool, is_error: bool) {
        let ts = timestamp_us();
        self.buffer[2] = ts as u8;
        self.buffer[3] = (ts >> 8) as u8;
        self.buffer[4] = (ts >> 16) as u8;
        self.buffer[5] = (ts >> 24) as u8;

        let mut flag_byte = depth & 0x3F;
        if is_exit { flag_byte |= flags::EXIT; }
        if is_error { flag_byte |= flags::ERROR; }
        self.buffer[6] = flag_byte;
        self.buffer[7] = 0;
    }

    pub fn subsys(&mut self, name: &str) {
        let len = name.len().min(31);
        self.buffer[self.pos] = len as u8;
        self.pos += 1;
        self.buffer[self.pos..self.pos + len].copy_from_slice(&name.as_bytes()[..len]);
        self.pos += len;
    }

    pub fn name(&mut self, name: &str) {
        let len = name.len().min(63);
        self.buffer[self.pos] = len as u8;
        self.pos += 1;
        self.buffer[self.pos..self.pos + len].copy_from_slice(&name.as_bytes()[..len]);
        self.pos += len;
    }

    pub fn kv_count(&mut self, count: u8) {
        self.buffer[self.pos] = count;
        self.pos += 1;
    }

    pub fn kv<T: Serialize>(&mut self, key: &str, value: T) {
        let key_len = key.len().min(31);
        let needed = 2 + key_len + value.serialized_size();
        if self.pos + needed > MAX_EVENT_SIZE { return; }

        self.buffer[self.pos] = key_len as u8;
        self.pos += 1;
        self.buffer[self.pos] = value.type_marker();
        self.pos += 1;
        self.buffer[self.pos..self.pos + key_len].copy_from_slice(&key.as_bytes()[..key_len]);
        self.pos += key_len;
        self.pos += value.serialize(&mut self.buffer[self.pos..]);
    }

    pub fn finish(mut self) {
        let total_len = self.pos as u16;
        self.buffer[0] = total_len as u8;
        self.buffer[1] = (total_len >> 8) as u8;

        unsafe {
            let ring = &mut *core::ptr::addr_of_mut!(TRACE_RING);
            ring.write(&self.buffer[..self.pos]);
        }
    }
}

// ============================================================================
// Span Guard
// ============================================================================

pub struct Span {
    subsys: &'static str,
    name: &'static str,
    depth: u8,
    finished: bool,
}

impl Span {
    #[doc(hidden)]
    pub fn new(subsys: &'static str, name: &'static str, depth: u8) -> Self {
        Self { subsys, name, depth, finished: false }
    }

    pub fn exit_ok(mut self) {
        if is_enabled() {
            let mut builder = EventBuilder::new();
            builder.header(self.depth, true, false);
            builder.subsys(self.subsys);
            builder.name(self.name);
            builder.kv_count(0);
            builder.finish();
        }
        self.finished = true;
        decrement_depth();
    }

    pub fn exit_err_simple(mut self) {
        if is_enabled() {
            let mut builder = EventBuilder::new();
            builder.header(self.depth, true, true);
            builder.subsys(self.subsys);
            builder.name(self.name);
            builder.kv_count(0);
            builder.finish();
        }
        self.finished = true;
        decrement_depth();
    }
}

impl Drop for Span {
    fn drop(&mut self) {
        if !self.finished {
            if is_enabled() {
                let mut builder = EventBuilder::new();
                builder.header(self.depth, true, false);
                builder.subsys(self.subsys);
                builder.name(self.name);
                builder.kv_count(0);
                builder.finish();
            }
            decrement_depth();
        }
    }
}

// ============================================================================
// Public API
// ============================================================================

#[doc(hidden)]
#[inline]
pub fn _start_enter(subsys: &str, name: &str, kv_count: u8) -> EventBuilder {
    let depth = current_depth();
    let mut builder = EventBuilder::new();
    builder.header(depth, false, false);
    builder.subsys(subsys);
    builder.name(name);
    builder.kv_count(kv_count);
    builder
}

#[doc(hidden)]
#[inline]
pub fn _start_exit(subsys: &str, name: &str, is_error: bool, kv_count: u8) -> EventBuilder {
    let depth = current_depth();
    let mut builder = EventBuilder::new();
    builder.header(depth, true, is_error);
    builder.subsys(subsys);
    builder.name(name);
    builder.kv_count(kv_count);
    builder
}

#[doc(hidden)]
#[inline]
pub fn _enter_depth() -> u8 {
    increment_depth()
}

// ============================================================================
// Formatting (Drain)
// ============================================================================

const HEX_CHARS: &[u8; 16] = b"0123456789abcdef";

pub fn drain_one() -> bool {
    let mut event_buf = [0u8; MAX_EVENT_SIZE];
    let mut text_buf = [0u8; 512];

    let event_len = unsafe {
        let ring = &mut *core::ptr::addr_of_mut!(TRACE_RING);
        ring.read(&mut event_buf)
    };

    let Some(len) = event_len else { return false; };

    let text_len = format_event(&event_buf[..len], &mut text_buf);
    if text_len > 0 {
        crate::syscall::write(crate::syscall::STDOUT, &text_buf[..text_len]);
    }

    true
}

pub fn flush() {
    while drain_one() {}
}

pub fn has_pending() -> bool {
    unsafe {
        let ring = &*core::ptr::addr_of!(TRACE_RING);
        ring.has_data()
    }
}

fn format_event(event: &[u8], out: &mut [u8]) -> usize {
    if event.len() < 10 { return 0; }

    let mut out_pos = 0;

    let ts_us = u32::from_le_bytes([event[2], event[3], event[4], event[5]]);
    let flag_byte = event[6];
    let depth = flag_byte & 0x3F;
    let is_exit = (flag_byte & flags::EXIT) != 0;
    let is_error = (flag_byte & flags::ERROR) != 0;

    out_pos += write_u32_padded(&mut out[out_pos..], ts_us, 8);
    out[out_pos] = b' ';
    out_pos += 1;

    out[out_pos] = if is_exit { b'<' } else { b'>' };
    out_pos += 1;
    out[out_pos] = b' ';
    out_pos += 1;

    for _ in 0..depth {
        out[out_pos] = b' ';
        out[out_pos + 1] = b' ';
        out_pos += 2;
    }

    let mut pos = 8;

    if pos >= event.len() { return 0; }
    let subsys_len = event[pos] as usize;
    pos += 1;
    if pos + subsys_len > event.len() { return 0; }
    out[out_pos..out_pos + subsys_len].copy_from_slice(&event[pos..pos + subsys_len]);
    out_pos += subsys_len;
    out[out_pos] = b' ';
    out_pos += 1;
    pos += subsys_len;

    if pos >= event.len() { return 0; }
    let name_len = event[pos] as usize;
    pos += 1;
    if pos + name_len > event.len() { return 0; }
    out[out_pos..out_pos + name_len].copy_from_slice(&event[pos..pos + name_len]);
    out_pos += name_len;
    pos += name_len;

    if pos >= event.len() { return out_pos; }
    let kv_count = event[pos] as usize;
    pos += 1;

    for _i in 0..kv_count {
        if pos + 2 > event.len() { break; }
        let key_len = event[pos] as usize;
        let vtype = event[pos + 1];
        pos += 2;

        if pos + key_len > event.len() { break; }

        out[out_pos] = b' ';
        out_pos += 1;

        out[out_pos..out_pos + key_len].copy_from_slice(&event[pos..pos + key_len]);
        out_pos += key_len;
        out[out_pos] = b'=';
        out_pos += 1;
        pos += key_len;

        out_pos += format_value(vtype, &event[pos..], &mut out[out_pos..], &mut pos);
    }

    if is_exit && is_error {
        let err_str = b" [ERR]";
        out[out_pos..out_pos + err_str.len()].copy_from_slice(err_str);
        out_pos += err_str.len();
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
                let v = u64::from_le_bytes([data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]]);
                out_pos += write_u64(&mut out[out_pos..], v);
                *pos += 8;
            }
        }
        1 => { // I64
            if data.len() >= 8 {
                let v = i64::from_le_bytes([data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]]);
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
                let v = u64::from_le_bytes([data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]]);
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
    for j in 0..padding { out[j] = b'0'; }
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
// Macros
// ============================================================================

#[doc(hidden)]
#[macro_export]
macro_rules! _utrace_count {
    () => { 0u8 };
    ($head:tt $($tail:tt)*) => { 1u8 + $crate::_utrace_count!($($tail)*) };
}

/// Create a span (RAII guard) - userspace version
#[macro_export]
macro_rules! uspan {
    ($subsys:expr, $name:expr; $($key:ident = $val:expr),* $(,)?) => {{
        let depth = $crate::utrace::_enter_depth();
        if $crate::utrace::is_enabled() {
            let kv_count = $crate::_utrace_count!($($key)*);
            let mut builder = $crate::utrace::_start_enter($subsys, $name, kv_count);
            $(
                builder.kv(stringify!($key), $val);
            )*
            builder.finish();
        }
        $crate::utrace::Span::new($subsys, $name, depth)
    }};
    ($subsys:expr, $name:expr) => {{
        let depth = $crate::utrace::_enter_depth();
        if $crate::utrace::is_enabled() {
            let mut builder = $crate::utrace::_start_enter($subsys, $name, 0);
            builder.finish();
        }
        $crate::utrace::Span::new($subsys, $name, depth)
    }};
}

/// Manual enter (use uspan! instead when possible)
#[macro_export]
macro_rules! utrace_enter {
    ($subsys:expr, $name:expr; $($key:ident = $val:expr),* $(,)?) => {{
        if $crate::utrace::is_enabled() {
            let kv_count = $crate::_utrace_count!($($key)*);
            let mut builder = $crate::utrace::_start_enter($subsys, $name, kv_count);
            $(
                builder.kv(stringify!($key), $val);
            )*
            builder.finish();
        }
    }};
    ($subsys:expr, $name:expr) => {{
        if $crate::utrace::is_enabled() {
            let mut builder = $crate::utrace::_start_enter($subsys, $name, 0);
            builder.finish();
        }
    }};
}

/// Manual exit with success
#[macro_export]
macro_rules! utrace_exit {
    ($subsys:expr, $name:expr) => {{
        if $crate::utrace::is_enabled() {
            let mut builder = $crate::utrace::_start_exit($subsys, $name, false, 0);
            builder.finish();
        }
    }};
}

/// Manual exit with error
#[macro_export]
macro_rules! utrace_exit_err {
    ($subsys:expr, $name:expr; $($key:ident = $val:expr),* $(,)?) => {{
        if $crate::utrace::is_enabled() {
            let kv_count = $crate::_utrace_count!($($key)*);
            let mut builder = $crate::utrace::_start_exit($subsys, $name, true, kv_count);
            $(
                builder.kv(stringify!($key), $val);
            )*
            builder.finish();
        }
    }};
    ($subsys:expr, $name:expr) => {{
        if $crate::utrace::is_enabled() {
            let mut builder = $crate::utrace::_start_exit($subsys, $name, true, 0);
            builder.finish();
        }
    }};
}
