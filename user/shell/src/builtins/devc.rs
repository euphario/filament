//! devc Builtin - Driver Configuration
//!
//! Get and set configuration values on running drivers via devd.
//!
//! Usage:
//!   devc get <key>                  Broadcast: find key across all drivers
//!   devc set <key> <value>          Broadcast: set key on owning driver
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

    // Parse: [service] <get|set> [key] [value]
    // If first token is get/set, broadcast to all drivers (no service name).
    let args_str = match core::str::from_utf8(args) {
        Ok(s) => s,
        Err(_) => {
            println!("Invalid arguments");
            return CommandResult::None;
        }
    };

    let mut parts = args_str.splitn(4, ' ');
    let first = match parts.next() {
        Some(s) if !s.is_empty() => s,
        _ => {
            println!("Usage: devc [service] get [key] | set <key> <value>");
            return CommandResult::None;
        }
    };

    // Detect broadcast: first token is operation, not service name
    let (service, op) = if first == "get" || first == "set" {
        ("", first)
    } else {
        let op = match parts.next() {
            Some(s) => s,
            None => {
                println!("Usage: devc [service] get [key] | set <key> <value>");
                return CommandResult::None;
            }
        };
        (first, op)
    };

    match op {
        "get" => {
            let key = parts.next().unwrap_or("");
            let mut buf = [0u8; 1024];
            let n = userlib::config::get(service.as_bytes(), key.as_bytes(), &mut buf);
            if n > 0 {
                return format_response(&buf[..n]);
            } else if service.is_empty() {
                println!("No driver has that key");
            } else {
                println!("Failed: no response from {}", service);
            }
        }
        "set" => {
            let key = match parts.next() {
                Some(k) => k,
                None => {
                    println!("Usage: devc [service] set <key> <value>");
                    return CommandResult::None;
                }
            };
            let value = match parts.next() {
                Some(v) => v,
                None => {
                    println!("Usage: devc [service] set <key> <value>");
                    return CommandResult::None;
                }
            };
            let mut buf = [0u8; 128];
            let n = userlib::config::set_raw(service.as_bytes(), key.as_bytes(), value.as_bytes(), &mut buf);
            if n > 0 {
                return format_response(&buf[..n]);
            } else if service.is_empty() {
                println!("No driver has that key");
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
    println!("  devc get <key>                Broadcast: find key across all drivers");
    println!("  devc set <key> <val>          Broadcast: set key on owning driver");
    println!("  devc <service> get            Get all config from a specific driver");
    println!("  devc <service> get <key>      Get specific config value");
    println!("  devc <service> set <key> <val> Set config value");
    println!();
    println!("Examples:");
    println!("  devc get wifi.radio           Find wifi.radio across all drivers");
    println!("  devc set wifi.channel 6       Set channel on owning driver");
    println!("  devc wifid get                Show wifid config summary");
    println!("  devc wifid get wifi.radio     Get specific key from wifid");
}

/// Format driver response as a structured table.
/// Parses key=value pairs from the response and displays them in columns.
fn format_response(data: &[u8]) -> CommandResult {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    let text = match core::str::from_utf8(&data[..end]) {
        Ok(s) => s.trim(),
        Err(_) => return CommandResult::None,
    };

    if text.is_empty() {
        return CommandResult::None;
    }

    // Parse key=value pairs into table
    let mut table = Table::new(&["KEY", "VALUE"]);

    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        for part in line.split_whitespace() {
            if let Some(eq_pos) = part.find('=') {
                let key = &part[..eq_pos];
                let val = &part[eq_pos + 1..];
                if !key.is_empty() {
                    table.add_row(Row::empty().string(key).string(val));
                }
            }
        }
    }

    if !table.is_empty() {
        CommandResult::Table(table)
    } else {
        // No key=value pairs — print raw (e.g. "OK" from set)
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
