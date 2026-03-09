//! dmesg — dump kernel log ring (non-destructive)
//!
//! Usage:
//!   dmesg                  — dump all buffered records
//!   dmesg -n 20            — show last 20 records (tail)
//!   dmesg -t 5000          — show records from 5000ms onward
//!   dmesg -l notice        — only records at notice level or below
//!   dmesg -m pcie          — only records from "pcie" subsystem
//!   dmesg -l debug -m usb  — combined filter

use libsys::syscall;

/// Run the dmesg command
pub fn run(args: &[u8]) {
    let args = crate::trim(args);

    // Parse flags
    let mut max_level: u8 = 2;       // default: info and above
    let mut module_filter: &[u8] = &[];
    let mut tail_count: u32 = 0;     // 0 = show all
    let mut from_ms: u32 = 0;        // 0 = no filter

    let mut i = 0;
    let parts = split_args(args);
    while i < parts.count {
        let arg = parts.get(i);
        if arg == b"-l" && i + 1 < parts.count {
            i += 1;
            if let Some(lvl) = parse_level(parts.get(i)) {
                max_level = lvl;
            } else {
                crate::println!("Unknown level. Use: error, warn, info, notice, debug, trace");
                return;
            }
        } else if arg == b"-m" && i + 1 < parts.count {
            i += 1;
            module_filter = parts.get(i);
        } else if arg == b"-n" && i + 1 < parts.count {
            i += 1;
            if let Some(n) = parse_u32(parts.get(i)) {
                tail_count = n;
            } else {
                crate::println!("Invalid count for -n");
                return;
            }
        } else if arg == b"-t" && i + 1 < parts.count {
            i += 1;
            if let Some(ms) = parse_u32(parts.get(i)) {
                from_ms = ms;
            } else {
                crate::println!("Invalid timestamp for -t (milliseconds)");
                return;
            }
        } else if arg == b"-h" || arg == b"--help" {
            crate::println!("Usage: dmesg [-n <count>] [-t <ms>] [-l <level>] [-m <subsys>]");
            crate::println!("  -n <count>   Show last N records (tail)");
            crate::println!("  -t <ms>      Show records from timestamp (ms since boot)");
            crate::println!("  -l <level>   Filter by level (error/warn/info/notice/debug/trace)");
            crate::println!("  -m <subsys>  Filter by subsystem name");
            return;
        }
        i += 1;
    }

    // Flush any stale INTERRUPT signals before starting
    let pid = syscall::getpid();
    syscall::signal_flush(pid);

    // Read from kernel ring non-destructively with in-kernel filtering.
    // Level and timestamp filters are applied by the kernel before formatting.
    // Subsystem filter is applied in userspace (string match on formatted text).
    let mut buf = [0u8; 4096];

    // For -n (tail): first pass counts matching records to compute skip count
    let skip = if tail_count > 0 {
        let total = count_records(&mut buf, from_ms, max_level, module_filter);
        total.saturating_sub(tail_count)
    } else {
        0
    };

    // Main pass: output records (skipping first `skip` for tail mode)
    let mut out = [0u8; 2048];
    let mut out_len: usize = 0;
    let mut cursor: u32 = 0;
    let mut count: u32 = 0;
    let mut skipped: u32 = 0;

    loop {
        let n = syscall::klog_read_filtered(&mut buf, &mut cursor, from_ms, max_level);
        if n <= 0 {
            break;
        }

        // Parse length-prefixed records from buf[4..4+n]
        let data = &buf[4..4 + n as usize];
        let mut pos = 0;

        while pos + 2 <= data.len() {
            let rec_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
            pos += 2;
            if rec_len == 0 || pos + rec_len > data.len() {
                break;
            }
            let text = &data[pos..pos + rec_len];
            pos += rec_len;

            // Subsystem filter (userspace — requires string match on formatted text)
            if !module_filter.is_empty() && !contains_bytes_ci(text, module_filter) {
                continue;
            }

            // Skip records for tail mode
            if skipped < skip {
                skipped += 1;
                continue;
            }

            // Accumulate in output buffer; flush when full
            if out_len + text.len() > out.len() {
                crate::console::write(&out[..out_len]);
                out_len = 0;
            }
            if text.len() > out.len() {
                crate::console::write(text);
            } else {
                out[out_len..out_len + text.len()].copy_from_slice(text);
                out_len += text.len();
            }
            count += 1;
        }

        // Check for Ctrl+C between batches (flush output first)
        if out_len > 0 {
            crate::console::write(&out[..out_len]);
            out_len = 0;
        }
        if crate::interrupted() {
            crate::println!("\r\n^C");
            return;
        }
    }

    // Flush remaining buffered output
    if out_len > 0 {
        crate::console::write(&out[..out_len]);
    }

    if count == 0 {
        crate::println!("(no matching records)");
    }
}

/// Count matching records in the ring (for tail mode).
fn count_records(buf: &mut [u8; 4096], from_ms: u32, max_level: u8, module_filter: &[u8]) -> u32 {
    let mut cursor: u32 = 0;
    let mut total: u32 = 0;
    loop {
        let n = syscall::klog_read_filtered(buf, &mut cursor, from_ms, max_level);
        if n <= 0 {
            break;
        }
        let data = &buf[4..4 + n as usize];
        let mut pos = 0;
        while pos + 2 <= data.len() {
            let rec_len = u16::from_le_bytes([data[pos], data[pos + 1]]) as usize;
            pos += 2;
            if rec_len == 0 || pos + rec_len > data.len() {
                break;
            }
            let text = &data[pos..pos + rec_len];
            pos += rec_len;
            if !module_filter.is_empty() && !contains_bytes_ci(text, module_filter) {
                continue;
            }
            total += 1;
        }
    }
    total
}

/// Check if haystack contains needle (case-insensitive ASCII)
fn contains_bytes_ci(haystack: &[u8], needle: &[u8]) -> bool {
    if needle.len() > haystack.len() || needle.is_empty() { return false; }
    for i in 0..=(haystack.len() - needle.len()) {
        let mut matched = true;
        for j in 0..needle.len() {
            let a = haystack[i + j].to_ascii_lowercase();
            let b = needle[j].to_ascii_lowercase();
            if a != b {
                matched = false;
                break;
            }
        }
        if matched { return true; }
    }
    false
}

fn parse_level(name: &[u8]) -> Option<u8> {
    if crate::cmd_eq(name, b"error") { Some(0) }
    else if crate::cmd_eq(name, b"warn") { Some(1) }
    else if crate::cmd_eq(name, b"info") { Some(2) }
    else if crate::cmd_eq(name, b"notice") { Some(3) }
    else if crate::cmd_eq(name, b"debug") { Some(4) }
    else if crate::cmd_eq(name, b"trace") { Some(5) }
    else { None }
}

/// Parse a decimal u32 from bytes
fn parse_u32(s: &[u8]) -> Option<u32> {
    if s.is_empty() { return None; }
    let mut val: u32 = 0;
    for &b in s {
        if b < b'0' || b > b'9' { return None; }
        val = val.checked_mul(10)?.checked_add((b - b'0') as u32)?;
    }
    Some(val)
}

/// Simple arg splitter (max 8 args)
struct Args<'a> {
    parts: [&'a [u8]; 8],
    count: usize,
}

impl<'a> Args<'a> {
    fn get(&self, idx: usize) -> &'a [u8] {
        if idx < self.count { self.parts[idx] } else { &[] }
    }
}

fn split_args(input: &[u8]) -> Args<'_> {
    let mut args = Args { parts: [&[]; 8], count: 0 };
    let mut i = 0;
    while i < input.len() && args.count < 8 {
        // Skip whitespace
        while i < input.len() && input[i] == b' ' { i += 1; }
        if i >= input.len() { break; }
        let start = i;
        while i < input.len() && input[i] != b' ' { i += 1; }
        args.parts[args.count] = &input[start..i];
        args.count += 1;
    }
    args
}
