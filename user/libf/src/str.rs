//! Byte-slice string operations — no allocation needed.

/// Trim ASCII whitespace (space, tab, CR, LF) from both ends.
pub fn trim(input: &[u8]) -> &[u8] {
    let start = input.iter().position(|&c| !is_ws(c));
    let end = input.iter().rposition(|&c| !is_ws(c));
    match (start, end) {
        (Some(s), Some(e)) => &input[s..=e],
        _ => &[],
    }
}

/// Trim ASCII whitespace from the start.
pub fn trim_start(input: &[u8]) -> &[u8] {
    match input.iter().position(|&c| !is_ws(c)) {
        Some(i) => &input[i..],
        None => &[],
    }
}

/// Trim ASCII whitespace from the end.
pub fn trim_end(input: &[u8]) -> &[u8] {
    match input.iter().rposition(|&c| !is_ws(c)) {
        Some(i) => &input[..=i],
        None => &[],
    }
}

/// Case-insensitive equality for ASCII byte slices.
pub fn eq_ignore_ascii_case(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    for (&x, &y) in a.iter().zip(b.iter()) {
        if to_lower(x) != to_lower(y) {
            return false;
        }
    }
    true
}

/// Check if `haystack` starts with `prefix` (case-insensitive ASCII).
pub fn starts_with_ignore_case(haystack: &[u8], prefix: &[u8]) -> bool {
    if haystack.len() < prefix.len() {
        return false;
    }
    eq_ignore_ascii_case(&haystack[..prefix.len()], prefix)
}

/// Check if `haystack` contains `needle` (case-insensitive ASCII substring search).
pub fn contains_ignore_ascii_case(haystack: &[u8], needle: &[u8]) -> bool {
    if needle.len() > haystack.len() {
        return false;
    }
    if needle.is_empty() {
        return true;
    }
    for i in 0..=(haystack.len() - needle.len()) {
        let mut matched = true;
        for j in 0..needle.len() {
            if to_lower(haystack[i + j]) != to_lower(needle[j]) {
                matched = false;
                break;
            }
        }
        if matched {
            return true;
        }
    }
    false
}

/// Split on the first occurrence of `sep`. Returns `(before, after)`.
/// If `sep` is not found, returns `(input, &[])`.
pub fn split_once(input: &[u8], sep: u8) -> (&[u8], &[u8]) {
    match input.iter().position(|&c| c == sep) {
        Some(i) => (&input[..i], &input[i + 1..]),
        None => (input, &[]),
    }
}

/// Split on ASCII whitespace into up to `MAX` parts. Returns `([parts; MAX], count)`.
pub fn split_whitespace<const MAX: usize>(input: &[u8]) -> ([&[u8]; MAX], usize) {
    let mut parts: [&[u8]; MAX] = [&[]; MAX];
    let mut count = 0;
    let mut i = 0;
    while i < input.len() && count < MAX {
        // Skip whitespace
        while i < input.len() && is_ws(input[i]) {
            i += 1;
        }
        if i >= input.len() {
            break;
        }
        // Find end of token
        let start = i;
        while i < input.len() && !is_ws(input[i]) {
            i += 1;
        }
        parts[count] = &input[start..i];
        count += 1;
    }
    (parts, count)
}

#[inline]
fn is_ws(c: u8) -> bool {
    c == b' ' || c == b'\t' || c == b'\r' || c == b'\n'
}

#[inline]
fn to_lower(c: u8) -> u8 {
    if c >= b'A' && c <= b'Z' { c + 32 } else { c }
}
