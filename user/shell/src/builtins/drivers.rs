//! Drivers Builtin - Device Driver Introspection
//!
//! Display driver and port hierarchy from devd.
//!
//! Usage:
//!   drivers              - Show all services and ports as tree
//!   drivers services     - Show services only (table)
//!   drivers ports        - Show ports only (table)
//!   drivers <name>       - Query detailed info from a specific service
//!   drivers help         - Show help

use core::fmt::Write;
use userlib::devd::DevdClient;
use userlib::query::{PortEntry, ServiceEntry, service_state, port_type, port_flags};
use crate::output::{CommandResult, Table, Row, Align};
use crate::{println, cmd_eq, trim};

/// Main entry point for drivers builtin
pub fn run(args: &[u8]) -> CommandResult {
    let args = trim(args);

    if args.is_empty() {
        cmd_tree()
    } else if cmd_eq(args, b"help") {
        show_help();
        CommandResult::None
    } else if cmd_eq(args, b"services") {
        cmd_services()
    } else if cmd_eq(args, b"ports") {
        cmd_ports()
    } else {
        // Query specific service by name
        cmd_info(args)
    }
}

fn show_help() {
    println!("drivers - Device Driver Introspection");
    println!();
    println!("Usage:");
    println!("  drivers              Show driver/port tree");
    println!("  drivers services     Show services table");
    println!("  drivers ports        Show ports table");
    println!("  drivers <name>       Query detailed info from a service");
    println!("  drivers help         Show this help");
    println!();
    println!("Examples:");
    println!("  drivers xhcid        Query USB controller info");
    println!("  drivers partd        Query partition driver info");
}

/// Query detailed info from a specific service
fn cmd_info(service_name: &[u8]) -> CommandResult {
    let mut client = match DevdClient::connect() {
        Ok(c) => c,
        Err(_) => return CommandResult::Error("failed to connect to devd"),
    };

    match client.query_service_info(service_name) {
        Ok((info, len)) => {
            // Print the info text
            if let Ok(text) = core::str::from_utf8(&info[..len]) {
                // Trim trailing nulls
                let text = text.trim_end_matches('\0');
                for line in text.lines() {
                    println!("{}", line);
                }
            } else {
                println!("(invalid UTF-8 response)");
            }
            CommandResult::None
        }
        Err(userlib::error::SysError::NotFound) => {
            if let Ok(name) = core::str::from_utf8(service_name) {
                println!("Service '{}' not found", name);
            } else {
                println!("Service not found");
            }
            CommandResult::None
        }
        Err(e) => {
            println!("Query failed: {:?}", e);
            CommandResult::None
        }
    }
}

/// Fixed-size string buffer for building output
struct StringBuf {
    buf: [u8; 4096],
    len: usize,
}

impl StringBuf {
    fn new() -> Self {
        Self { buf: [0; 4096], len: 0 }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl Write for StringBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.len;
        let to_copy = bytes.len().min(remaining);
        self.buf[self.len..self.len + to_copy].copy_from_slice(&bytes[..to_copy]);
        self.len += to_copy;
        Ok(())
    }
}

/// Display tree view of services and their ports
fn cmd_tree() -> CommandResult {
    let mut client = match DevdClient::connect() {
        Ok(c) => c,
        Err(_) => {
            return CommandResult::Error("failed to connect to devd");
        }
    };

    // Get services
    let (services, svc_count) = match client.list_services() {
        Ok(r) => r,
        Err(_) => {
            return CommandResult::Error("failed to list services");
        }
    };

    // Get ports
    let (ports, port_count) = match client.list_ports() {
        Ok(r) => r,
        Err(_) => {
            return CommandResult::Error("failed to list ports");
        }
    };

    // Build output in buffer
    let mut out = StringBuf::new();

    // Header
    let _ = writeln!(out, "\x1b[1m\x1b[36mDriver Tree\x1b[0m");
    let _ = writeln!(out);

    // Print services with their owned ports
    for i in 0..svc_count {
        let svc = &services[i];
        format_service(&mut out, svc);

        // Find ports owned by this service
        for j in 0..port_count {
            let port = &ports[j];
            if port.owner_idx == i as u8 {
                format_port(&mut out, port);
            }
        }
    }

    // Print root-level ports (owned by devd itself, owner_idx = 0xFF)
    let mut has_root = false;
    for j in 0..port_count {
        let port = &ports[j];
        if port.owner_idx == 0xFF {
            if !has_root {
                let _ = writeln!(out);
                let _ = writeln!(out, "\x1b[33mdevd (root)\x1b[0m");
                has_root = true;
            }
            format_port(&mut out, port);
        }
    }

    // Print entire buffer at once
    crate::print!("{}", out.as_str());
    CommandResult::None
}

/// Format a service with its state into buffer
fn format_service(out: &mut StringBuf, svc: &ServiceEntry) {
    let name = name_str(&svc.name);
    let state = svc.state_str();
    let state_color = match svc.state {
        service_state::READY => "\x1b[32m",     // green
        service_state::STARTING => "\x1b[33m", // yellow
        service_state::CRASHED | service_state::FAILED => "\x1b[31m", // red
        _ => "\x1b[2m", // dim
    };

    // Format: name pid=N [state]
    let _ = write!(out, "\x1b[1m\x1b[33m{}\x1b[0m", name);
    if svc.pid > 0 {
        let _ = write!(out, " \x1b[2mpid={}\x1b[0m", svc.pid);
    }
    let _ = write!(out, " {}[{}]\x1b[0m", state_color, state);
    if svc.total_restarts > 0 {
        let _ = write!(out, " \x1b[2mrestarts={}\x1b[0m", svc.total_restarts);
    }
    let _ = writeln!(out);
}

/// Format a port with indentation into buffer
fn format_port(out: &mut StringBuf, port: &PortEntry) {
    let name = port_name_str(&port.name);
    let type_str = port_type_str(port.port_type);
    let has_data = port.flags & port_flags::HAS_DATAPORT != 0;

    // Tree connector and port name
    let _ = write!(out, "  \x1b[2m└─\x1b[0m \x1b[36m{}\x1b[0m", name);

    // Type
    let _ = write!(out, " \x1b[2m({})\x1b[0m", type_str);

    // DataPort shmem_id if present
    if has_data && port.shmem_id > 0 {
        let _ = write!(out, " \x1b[35mshmem={}\x1b[0m", port.shmem_id);
    }

    let _ = writeln!(out);
}

/// Display services as a table
fn cmd_services() -> CommandResult {
    let mut client = match DevdClient::connect() {
        Ok(c) => c,
        Err(_) => {
            return CommandResult::Error("failed to connect to devd");
        }
    };

    let (services, count) = match client.list_services() {
        Ok(r) => r,
        Err(_) => {
            return CommandResult::Error("failed to list services");
        }
    };

    let mut table = Table::new(&["IDX", "NAME", "PID", "STATE", "RESTARTS"])
        .align(0, Align::Right)
        .align(2, Align::Right)
        .align(4, Align::Right);

    for i in 0..count {
        let svc = &services[i];
        let row = Row::empty()
            .uint(i as u64)
            .bytes(svc.name_str())
            .uint(svc.pid as u64)
            .str(svc.state_str())
            .uint(svc.total_restarts as u64);
        table.add_row(row);
    }

    CommandResult::Table(table)
}

/// Display ports as a table
fn cmd_ports() -> CommandResult {
    let mut client = match DevdClient::connect() {
        Ok(c) => c,
        Err(_) => {
            return CommandResult::Error("failed to connect to devd");
        }
    };

    let (ports, count) = match client.list_ports() {
        Ok(r) => r,
        Err(_) => {
            return CommandResult::Error("failed to list ports");
        }
    };

    let mut table = Table::new(&["NAME", "TYPE", "OWNER", "SHMEM", "PID"])
        .align(3, Align::Right)
        .align(4, Align::Right);

    for i in 0..count {
        let port = &ports[i];
        let has_data = port.flags & port_flags::HAS_DATAPORT != 0;

        let row = Row::empty()
            .bytes(port.name_str())
            .str(port_type_str(port.port_type))
            .uint(if port.owner_idx == 0xFF { 0 } else { port.owner_idx as u64 })
            .uint(if has_data { port.shmem_id as u64 } else { 0 })
            .uint(port.owner_pid as u64);
        table.add_row(row);
    }

    CommandResult::Table(table)
}

/// Convert port type to string
fn port_type_str(pt: u8) -> &'static str {
    match pt {
        port_type::BLOCK => "block",
        port_type::PARTITION => "partition",
        port_type::FILESYSTEM => "filesystem",
        port_type::USB => "usb",
        port_type::NETWORK => "network",
        port_type::CONSOLE => "console",
        port_type::SERVICE => "service",
        _ => "unknown",
    }
}

/// Convert service name bytes to str
fn name_str(name: &[u8; 16]) -> &str {
    let len = name.iter().position(|&b| b == 0).unwrap_or(16);
    core::str::from_utf8(&name[..len]).unwrap_or("???")
}

/// Convert port name bytes to str
fn port_name_str(name: &[u8; 20]) -> &str {
    let len = name.iter().position(|&b| b == 0).unwrap_or(20);
    core::str::from_utf8(&name[..len]).unwrap_or("???")
}
