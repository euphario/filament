//! devc Builtin - Driver Configuration
//!
//! Get and set configuration values on running drivers via devd.
//!
//! Usage:
//!   devc <service> get [key]        Get config (all if key omitted)
//!   devc <service> set <key> <value> Set config value

use crate::println;
use crate::output::{CommandResult, Table, Row};

pub fn run(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    if args.is_empty() || args == b"help" {
        show_help();
        return CommandResult::None;
    }

    // Parse: <service> <get|set> [key] [value]
    let args_str = match core::str::from_utf8(args) {
        Ok(s) => s,
        Err(_) => {
            println!("Invalid arguments");
            return CommandResult::None;
        }
    };

    let mut parts = args_str.splitn(4, ' ');
    let service = match parts.next() {
        Some(s) if !s.is_empty() => s,
        _ => {
            println!("Usage: devc <service> get [key] | set <key> <value>");
            return CommandResult::None;
        }
    };

    let op = match parts.next() {
        Some(s) => s,
        None => {
            println!("Usage: devc <service> get [key] | set <key> <value>");
            return CommandResult::None;
        }
    };

    match op {
        "get" => {
            let key = parts.next().unwrap_or("");
            let mut buf = [0u8; 1024];
            let n = userlib::config::get(service.as_bytes(), key.as_bytes(), &mut buf);
            if n > 0 {
                return format_response(&buf[..n]);
            } else {
                println!("Failed: no response from {}", service);
            }
        }
        "set" => {
            let key = match parts.next() {
                Some(k) => k,
                None => {
                    println!("Usage: devc <service> set <key> <value>");
                    return CommandResult::None;
                }
            };
            let value = match parts.next() {
                Some(v) => v,
                None => {
                    println!("Usage: devc <service> set <key> <value>");
                    return CommandResult::None;
                }
            };
            let mut buf = [0u8; 128];
            let n = userlib::config::set_raw(service.as_bytes(), key.as_bytes(), value.as_bytes(), &mut buf);
            if n > 0 {
                return format_response(&buf[..n]);
            } else {
                println!("Failed: no response from {}", service);
            }
        }
        _ => {
            println!("Unknown operation '{}'. Use 'get' or 'set'.", op);
        }
    }

    CommandResult::None
}

fn show_help() {
    println!("devc - Driver Configuration");
    println!();
    println!("Usage:");
    println!("  devc <service> get            Get all config (summary)");
    println!("  devc <service> get <key>      Get specific config value");
    println!("  devc <service> set <key> <val> Set config value");
    println!();
    println!("Examples:");
    println!("  devc ipd get                  Show IP config summary");
    println!("  devc ipd get net.ip           Show current IP address");
    println!("  devc ipd set net.ip 10.0.2.50");
    println!("  devc ipd set net.gateway 10.0.2.1");
    println!("  devc ipd set net.dhcp off");
}

/// Format driver response as a structured table.
/// Parses key=value pairs from the response and displays them in columns.
fn format_response(data: &[u8]) -> CommandResult {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    let text = match core::str::from_utf8(&data[..end]) {
        Ok(s) => s,
        Err(_) => return CommandResult::None,
    };

    // Try to parse as key=value table format
    let mut table = Table::new(&["KEY", "VALUE"]);
    let mut has_kv = false;

    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        // Check if line has key=value pairs (space or newline separated)
        for part in line.split_whitespace() {
            if let Some(eq_pos) = part.find('=') {
                let key = &part[..eq_pos];
                let val = &part[eq_pos + 1..];
                if !key.is_empty() {
                    table.add_row(Row::empty().string(key).string(val));
                    has_kv = true;
                }
            }
        }

        // If no key=value on this line, treat as header/message
        if !line.contains('=') && !line.is_empty() {
            // This is a header line - add as a single-column row
            table.add_row(Row::empty().string(line).str(""));
        }
    }

    if has_kv || !table.is_empty() {
        CommandResult::Table(table)
    } else {
        // Fallback: print raw text
        print_raw(text);
        CommandResult::None
    }
}

/// Print raw text with \n -> \r\n conversion
fn print_raw(text: &str) {
    for line in text.lines() {
        crate::print!("{}", line);
        crate::console::write(b"\r\n");
    }
}
