//! dlog - Query and control driver logging via devd
//!
//! Usage:
//!   dlog           - Show last 20 log messages
//!   dlog -n 50     - Show last 50 log messages
//!   dlog on        - Enable live logging
//!   dlog off       - Disable live logging
//!   dlog status    - Show logging status

use userlib::devd::DevdClient;
use crate::output::CommandResult;
use crate::{print, println};

/// Parse a number from bytes
fn parse_num(s: &[u8]) -> Option<usize> {
    let mut n = 0usize;
    for &b in s {
        if b >= b'0' && b <= b'9' {
            n = n * 10 + (b - b'0') as usize;
        } else {
            return None;
        }
    }
    Some(n)
}

/// Check if bytes equal a string
fn cmd_eq(a: &[u8], b: &[u8]) -> bool {
    a == b
}

/// Trim whitespace
fn trim(s: &[u8]) -> &[u8] {
    let start = s.iter().position(|&b| b != b' ' && b != b'\t').unwrap_or(s.len());
    let end = s.iter().rposition(|&b| b != b' ' && b != b'\t').map(|p| p + 1).unwrap_or(start);
    &s[start..end]
}

pub fn run(args: &[u8]) -> CommandResult {
    let args = trim(args);

    // logs on
    if cmd_eq(args, b"on") {
        return cmd_enable(true);
    }

    // logs off
    if cmd_eq(args, b"off") {
        return cmd_enable(false);
    }

    // logs status
    if cmd_eq(args, b"status") {
        return cmd_status();
    }

    // logs -n <count>
    if args.starts_with(b"-n ") {
        let rest = trim(&args[3..]);
        if let Some(count) = parse_num(rest) {
            return cmd_show(count.min(100) as u8);
        } else {
            println!("Invalid count");
            return CommandResult::None;
        }
    }

    // logs (default: show last 20)
    if args.is_empty() {
        return cmd_show(20);
    }

    // dlog help
    if cmd_eq(args, b"help") {
        println!("Usage: dlog [options]");
        println!("  dlog          Show last 20 log messages from drivers");
        println!("  dlog -n N     Show last N log messages (max 100)");
        println!("  dlog on       Enable live logging to console");
        println!("  dlog off      Disable live logging");
        println!("  dlog status   Show logging status");
        return CommandResult::None;
    }

    println!("Unknown option. Try 'logs help'");
    CommandResult::None
}

fn cmd_show(count: u8) -> CommandResult {
    let mut client = match DevdClient::connect() {
        Ok(c) => c,
        Err(e) => {
            println!("Failed to connect to devd: {:?}", e);
            return CommandResult::None;
        }
    };

    match client.query_logs(count) {
        Ok((buf, len, live_enabled)) => {
            if len == 0 {
                println!("No log messages");
            } else {
                // Print the log text directly
                if let Ok(text) = core::str::from_utf8(&buf[..len]) {
                    print!("{}", text);
                }
            }

            // Show live status
            if live_enabled {
                println!("(live logging: ON)");
            } else {
                println!("(live logging: OFF - use 'logs on' to enable)");
            }
        }
        Err(e) => {
            println!("Failed to query logs: {:?}", e);
        }
    }

    CommandResult::None
}

fn cmd_enable(enable: bool) -> CommandResult {
    let mut client = match DevdClient::connect() {
        Ok(c) => c,
        Err(e) => {
            println!("Failed to connect to devd: {:?}", e);
            return CommandResult::None;
        }
    };

    match client.log_control(enable) {
        Ok(()) => {
            if enable {
                println!("Live logging enabled");
            } else {
                println!("Live logging disabled");
            }
        }
        Err(e) => {
            println!("Failed to control logging: {:?}", e);
        }
    }

    CommandResult::None
}

fn cmd_status() -> CommandResult {
    let mut client = match DevdClient::connect() {
        Ok(c) => c,
        Err(e) => {
            println!("Failed to connect to devd: {:?}", e);
            return CommandResult::None;
        }
    };

    // Query with 0 entries just to get status
    match client.query_logs(0) {
        Ok((_, _, live_enabled)) => {
            println!("Live logging: {}", if live_enabled { "ON" } else { "OFF" });
        }
        Err(e) => {
            println!("Failed to query status: {:?}", e);
        }
    }

    CommandResult::None
}
