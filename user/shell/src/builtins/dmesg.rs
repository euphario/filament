//! dmesg — dump kernel log ring (non-destructive)
//!
//! Usage:
//!   dmesg                  — dump all buffered records
//!   dmesg -l notice        — only records at notice level or below
//!   dmesg -m pcie          — only records from "pcie" subsystem
//!   dmesg -l debug -m usb  — combined filter

use userlib::syscall;

/// Run the dmesg command
pub fn run(args: &[u8]) {
    let args = crate::trim(args);

    // Parse flags
    let mut level_filter: Option<u8> = None;
    let mut module_filter: &[u8] = &[];

    let mut i = 0;
    let parts = split_args(args);
    while i < parts.count {
        let arg = parts.get(i);
        if arg == b"-l" && i + 1 < parts.count {
            i += 1;
            level_filter = parse_level(parts.get(i));
            if level_filter.is_none() {
                crate::println!("Unknown level. Use: error, warn, info, notice, debug, trace");
                return;
            }
        } else if arg == b"-m" && i + 1 < parts.count {
            i += 1;
            module_filter = parts.get(i);
        } else if arg == b"-h" || arg == b"--help" {
            crate::println!("Usage: dmesg [-l <level>] [-m <subsys>]");
            crate::println!("  -l <level>   Filter by level (error/warn/info/notice/debug/trace)");
            crate::println!("  -m <subsys>  Filter by subsystem name");
            return;
        }
        i += 1;
    }

    // Read from kernel ring non-destructively
    let mut buf = [0u8; 1024];
    let mut cursor: u32 = 0;
    let mut count: u32 = 0;

    loop {
        let n = syscall::klog_read_at(&mut buf, &mut cursor);
        if n <= 0 {
            break;
        }

        let text = &buf[4..4 + n as usize];

        // Apply level filter (check ANSI-colored level field in formatted output)
        if let Some(max_level) = level_filter {
            if let Some(record_level) = extract_level_from_text(text) {
                if record_level > max_level {
                    continue;
                }
            }
        }

        // Apply module filter (check subsystem field in formatted output)
        if !module_filter.is_empty() {
            if !text_contains_subsys(text, module_filter) {
                continue;
            }
        }

        crate::console::write(text);
        count += 1;
    }

    if count == 0 {
        crate::println!("(kernel log ring empty)");
    }
}

/// Try to extract log level from formatted text.
/// The level field appears after timestamp, formatted as "ERROR", "WARN ", "INFO ", etc.
/// with ANSI color codes around it.
fn extract_level_from_text(text: &[u8]) -> Option<u8> {
    // Look for level keywords after the ANSI-colored timestamp
    // The format is: \e[2m<timestamp>\e[0m \e[color]LEVEL\e[0m ...
    // Find the level string by scanning for known patterns
    if contains_bytes(text, b"ERROR") { return Some(0); }
    if contains_bytes(text, b"WARN ") { return Some(1); }
    if contains_bytes(text, b"INFO ") { return Some(2); }
    if contains_bytes(text, b"NOTCE") { return Some(3); }
    if contains_bytes(text, b"DEBUG") { return Some(4); }
    if contains_bytes(text, b"TRACE") { return Some(5); }
    None
}

/// Check if text contains a subsystem name (case-insensitive byte search)
fn text_contains_subsys(text: &[u8], subsys: &[u8]) -> bool {
    // The subsystem appears as a cyan-colored field after the level
    // Simplified: just check if the subsystem name appears anywhere in the text
    contains_bytes_ci(text, subsys)
}

/// Check if haystack contains needle (exact bytes)
fn contains_bytes(haystack: &[u8], needle: &[u8]) -> bool {
    if needle.len() > haystack.len() { return false; }
    for i in 0..=(haystack.len() - needle.len()) {
        if &haystack[i..i + needle.len()] == needle {
            return true;
        }
    }
    false
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
