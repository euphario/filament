//! devc Builtin - Driver Configuration
//!
//! Get and set configuration values on running drivers via devd.
//!
//! Usage:
//!   devc <service> get [key]        Get config (all if key omitted)
//!   devc <service> set <key> <value> Set config value

use crate::println;
use crate::output::CommandResult;

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
            let mut buf = [0u8; 512];
            let n = userlib::config::get(service.as_bytes(), key.as_bytes(), &mut buf);
            if n > 0 {
                print_bytes(&buf[..n]);
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
                print_bytes(&buf[..n]);
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

/// Print a byte slice as text, converting \n to \r\n for proper terminal display.
fn print_bytes(data: &[u8]) {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    let data = &data[..end];

    // Print byte by byte, converting \n to \r\n
    for &b in data {
        if b == b'\n' {
            crate::console::write(b"\r\n");
        } else {
            crate::console::write(&[b]);
        }
    }

    // Ensure we end with a newline for clean prompt
    if !data.is_empty() && data[data.len() - 1] != b'\n' {
        crate::console::write(b"\r\n");
    }
}
