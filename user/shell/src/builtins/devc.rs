//! devc Builtin - Driver Configuration
//!
//! Get and set configuration values on running drivers via devd.
//!
//! Usage:
//!   devc get [key]                  Broadcast: find key across all drivers
//!   devc set <key> <value>          Broadcast: set key on owning driver
//!   devc <service> get [key]        Get config (all if key omitted)
//!   devc <service> set <key> <value> Set config value
//!   devc <address> get [key]        Address-routed get
//!   devc <address> set <key> <value> Address-routed set
//!   devc get @topology              Show port tree

use crate::println;
use crate::output::{CommandResult, Table, Row};

pub fn run(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    if args.is_empty() || args == b"help" {
        show_help();
        return CommandResult::None;
    }

    // Parse: [address|service] <get|set> [key] [value]
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
            println!("Usage: devc [address|service] get [key] | set <key> <value>");
            return CommandResult::None;
        }
    };

    // Detect broadcast: first token is operation, not service/address
    let (service, op) = if first == "get" || first == "set" {
        ("", first)
    } else if first.starts_with('/') || first.starts_with('@') {
        // Address mode: /path or @class — treat as "service" name for the CONFIG wire format
        let op = match parts.next() {
            Some(s) => s,
            None => {
                println!("Usage: devc <address> get [key] | set <key> <value>");
                return CommandResult::None;
            }
        };
        (first, op)
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
            let n = libos::config::get(service.as_bytes(), key.as_bytes(), &mut buf);
            if n > 0 {
                if key == "@topology" {
                    return format_topology(&buf[..n]);
                }
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
            let n = libos::config::set_raw(service.as_bytes(), key.as_bytes(), value.as_bytes(), &mut buf);
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
    println!("  devc get [key]                Broadcast: find key across all drivers");
    println!("  devc get @topology            Show port tree");
    println!("  devc set <key> <val>          Broadcast: set key on owning driver");
    println!("  devc <service> get            Get all config from a specific driver");
    println!("  devc <service> get <key>      Get specific config value");
    println!("  devc <service> set <key> <val> Set config value");
    println!("  devc @<class> get [key]       Query by class (wifi, nvme, block, etc.)");
    println!("  devc /<path> get [key]        Query by port path");
    println!();
    println!("Path format (port names contain ':', driver names don't):");
    println!("  /pcie:0/nvme:0/block:0/partd  Full path (port/driver pairs)");
    println!("  /pcie:0/nvme:0/block:0        Short path (port names only)");
    println!();
    println!("Examples:");
    println!("  devc get radio                Find 'radio' key across all drivers");
    println!("  devc wifid set channel 6      Set channel on wifid");
    println!("  devc wifid get                Show wifid config summary");
    println!("  devc @wifi get channel        Get wifi channel by class");
    println!("  devc /pcie:0/nvme:0 get       Query device at path");
    println!("  devc /pcie:0/block:0 get count  Targeted query");
    println!("  devc get @topology            Show device tree");
}

/// Format driver response as a structured table.
/// Parses key=value pairs and [source] headers from the response.
/// When source annotations are present, adds a SOURCE column.
fn format_response(data: &[u8]) -> CommandResult {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    let text = match core::str::from_utf8(&data[..end]) {
        Ok(s) => s.trim(),
        Err(_) => return CommandResult::None,
    };

    if text.is_empty() {
        return CommandResult::None;
    }

    // First pass: check if any [source] headers exist
    let has_sources = text.lines().any(|l| {
        let l = l.trim();
        l.starts_with('[') && l.ends_with(']') && l.len() > 2
    });

    if has_sources {
        let mut table = Table::new(&["SOURCE", "KEY", "VALUE"]);
        let mut current_source: &str = "";

        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }

            // [source] header — update current source context
            if line.starts_with('[') && line.ends_with(']') && line.len() > 2 {
                current_source = &line[1..line.len() - 1];
                continue;
            }

            if let Some(eq_pos) = line.find('=') {
                let key = &line[..eq_pos];
                let val = &line[eq_pos + 1..];
                if !key.is_empty() {
                    table.add_row(Row::empty()
                        .string(current_source)
                        .string(key)
                        .string(val));
                }
            }
        }

        if !table.is_empty() {
            return CommandResult::Table(table);
        }
    }

    // No sources — simple two-column table
    let mut table = Table::new(&["KEY", "VALUE"]);

    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        if let Some(eq_pos) = line.find('=') {
            let key = &line[..eq_pos];
            let val = &line[eq_pos + 1..];
            if !key.is_empty() {
                table.add_row(Row::empty().string(key).string(val));
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

/// Format @topology response as an indented tree.
///
/// Input: [source] headers with full paths + driver=<name> KVs.
/// Each [source] path encodes the tree position:
///   [pcie:0/pcied] driver=pcied
///   [pcie:0/pcied/nvme:0/nvmed] driver=nvmed
///   [pcie:0/pcied/nvme:0/nvmed/block:0/partd] driver=partd
///
/// Output: indented tree where depth = number of '/' in source path / 2.
/// Entries are sorted by source path so the tree displays correctly
/// regardless of IPC arrival order.
fn format_topology(data: &[u8]) -> CommandResult {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    let text = match core::str::from_utf8(&data[..end]) {
        Ok(s) => s.trim(),
        Err(_) => return CommandResult::None,
    };

    if text.is_empty() {
        return CommandResult::None;
    }

    // Collect (source_path, driver_name) pairs into a fixed array.
    // Entries arrive in arbitrary IPC order — we sort by source path
    // so that the tree displays parent before child consistently.
    const MAX_ENTRIES: usize = 32;
    let mut sources: [&str; MAX_ENTRIES] = [""; MAX_ENTRIES];
    let mut names: [&str; MAX_ENTRIES] = [""; MAX_ENTRIES];
    let mut count = 0;

    let mut current_source = "";
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() { continue; }

        if line.starts_with('[') && line.ends_with(']') && line.len() > 2 {
            current_source = &line[1..line.len() - 1];
            continue;
        }

        if count >= MAX_ENTRIES { break; }

        // Extract driver name from "driver=<name>" line
        let name = if let Some(eq) = line.find('=') {
            &line[eq + 1..]
        } else {
            line
        };

        sources[count] = current_source;
        names[count] = name;
        count += 1;
    }

    // Sort by source path (insertion sort — small array, no alloc).
    // Lexicographic order ensures parents appear before children
    // ("pcie:0/pcied" < "pcie:0/pcied/nvme:0/nvmed").
    for i in 1..count {
        let mut j = i;
        while j > 0 && sources[j] < sources[j - 1] {
            sources.swap(j, j - 1);
            names.swap(j, j - 1);
            j -= 1;
        }
    }

    // Display sorted entries
    for i in 0..count {
        let source = sources[i];
        let name = names[i];

        // Depth from source path: paths alternate port/driver segments,
        // so depth = slash_count / 2.
        //   "pcie:0/pcied"                         → 1 slash → depth 0
        //   "pcie:0/pcied/nvme:0/nvmed"             → 3 slashes → depth 1
        //   "pcie:0/pcied/nvme:0/nvmed/block:0/partd" → 5 slashes → depth 2
        let depth = if source.is_empty() {
            0
        } else {
            source.as_bytes().iter().filter(|&&b| b == b'/').count() / 2
        };

        // Extract the last port:name segment from source path (the trigger port)
        let port_label = if source.is_empty() {
            ""
        } else {
            let mut last_port = "";
            for seg in source.rsplit('/') {
                if seg.contains(':') {
                    last_port = seg;
                    break;
                }
            }
            last_port
        };

        // Print with indentation
        for _ in 0..depth {
            crate::print!("  ");
        }
        if port_label.is_empty() {
            crate::print!("{}", name);
        } else {
            crate::print!("{} [{}]", name, port_label);
        }
        crate::console::write(b"\r\n");
    }

    CommandResult::None
}

/// Print raw text with \n -> \r\n conversion
fn print_raw(text: &str) {
    for line in text.lines() {
        crate::print!("{}", line);
        crate::console::write(b"\r\n");
    }
}
