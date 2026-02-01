//! ANSI Color Support
//!
//! Terminal colors for the shell.

use crate::console;

// ANSI escape codes
pub const RESET: &[u8] = b"\x1b[0m";
pub const BOLD: &[u8] = b"\x1b[1m";
pub const DIM: &[u8] = b"\x1b[2m";

// Foreground colors
pub const BLACK: &[u8] = b"\x1b[30m";
pub const RED: &[u8] = b"\x1b[31m";
pub const GREEN: &[u8] = b"\x1b[32m";
pub const YELLOW: &[u8] = b"\x1b[33m";
pub const BLUE: &[u8] = b"\x1b[34m";
pub const MAGENTA: &[u8] = b"\x1b[35m";
pub const CYAN: &[u8] = b"\x1b[36m";
pub const WHITE: &[u8] = b"\x1b[37m";

// Bright foreground colors
pub const BRIGHT_BLACK: &[u8] = b"\x1b[90m";
pub const BRIGHT_RED: &[u8] = b"\x1b[91m";
pub const BRIGHT_GREEN: &[u8] = b"\x1b[92m";
pub const BRIGHT_YELLOW: &[u8] = b"\x1b[93m";
pub const BRIGHT_BLUE: &[u8] = b"\x1b[94m";
pub const BRIGHT_MAGENTA: &[u8] = b"\x1b[95m";
pub const BRIGHT_CYAN: &[u8] = b"\x1b[96m";
pub const BRIGHT_WHITE: &[u8] = b"\x1b[97m";

/// Write a color code (skipped when output is being captured for IPC)
#[inline]
pub fn set(color: &[u8]) {
    if console::is_capturing() {
        return;
    }
    console::write(color);
}

/// Reset to default (skipped when output is being captured for IPC)
#[inline]
pub fn reset() {
    if console::is_capturing() {
        return;
    }
    console::write(RESET);
}

/// Print colored text
pub fn print(color: &[u8], text: &[u8]) {
    set(color);
    console::write(text);
    reset();
}

/// Print colored string
pub fn print_str(color: &[u8], text: &str) {
    print(color, text.as_bytes());
}
