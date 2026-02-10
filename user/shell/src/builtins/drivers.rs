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

/// Total display width for dot-leader alignment
const LINE_WIDTH: usize = 50;

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
    println!("  drivers usbd         Query USB controller info");
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
            if let Ok(text) = core::str::from_utf8(&info[..len]) {
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

// ───── Tree display ─────────────────────────────────────────

/// Display unified tree of ports and services.
///
/// Output format (port → service → port → service ...):
/// ```text
/// devd ─────────────────────────────────────────
/// ├─ ○ pcie0 ·····························     pcie
/// │ └─ ● pcied ··························· 2 ● ready
/// ├─ ○ uart0 ·····························     uart
/// │ └─ ● consoled ························ 3 ● ready
/// │   └─ ○ console: ······················     console
/// │     └─ ● shell ······················· 6 ● ready
/// ├─ ● vfsd ······························ 4 ● ready
/// │ └─ ○ vfs: ····························     filesystem shmem=1
/// └─ ● ipd ······························· 5 ● ready
///   └─ ○ ipd: ····························     service
/// ```
fn cmd_tree() -> CommandResult {
    let mut client = match DevdClient::connect() {
        Ok(c) => c,
        Err(_) => return CommandResult::Error("failed to connect to devd"),
    };

    let (services, svc_count) = match client.list_services() {
        Ok(r) => r,
        Err(_) => return CommandResult::Error("failed to list services"),
    };

    let (ports, port_count) = match client.list_ports() {
        Ok(r) => r,
        Err(_) => return CommandResult::Error("failed to list ports"),
    };

    let mut out = StringBuf::new();

    // Header: "devd ──────────────────────"
    let _ = write!(out, "\x1b[1mdevd\x1b[0m \x1b[2m");
    for _ in 0..LINE_WIDTH.saturating_sub(5) {
        let _ = write!(out, "─");
    }
    let _ = writeln!(out, "\x1b[0m");

    // Collect top-level items: kernel bus ports (owner=0xFF) and
    // top-level services (parent_idx=0xFF, no bus_path)
    //
    // A top-level item is either:
    //   - A kernel bus port (owner_idx == 0xFF)
    //   - A service with parent_idx == 0xFF that has no bus_path (not spawned from a port)
    //
    // Services spawned from kernel bus ports (has bus_path matching a port)
    // appear as children of that port, not as top-level items.

    const MAX_TOP: usize = 24;
    // Top-level items: (is_port, index)
    let mut top: [(bool, usize); MAX_TOP] = [(false, 0); MAX_TOP];
    let mut top_n = 0usize;

    // First: kernel bus ports (owner_idx == 0xFF, hardware port types)
    // Skip devd's own infrastructure ports (service class)
    for i in 0..port_count {
        if ports[i].owner_idx == 0xFF && is_hw_port_type(ports[i].port_type) && top_n < MAX_TOP {
            top[top_n] = (true, i);
            top_n += 1;
        }
    }

    // Then: top-level services not already shown as children of a port
    for i in 0..svc_count {
        let svc = &services[i];
        if svc.parent_idx != 0xFF { continue; }
        if name_str(&svc.name) == "devd" { continue; }

        // Skip if this service's bus_path matches ANY registered port —
        // it will be rendered as a child of that port's owner in the tree.
        let bp = svc_bus_path_str(svc);
        if !bp.is_empty() {
            let mut matched = false;
            for j in 0..port_count {
                if port_name_str(&ports[j].name) == bp {
                    matched = true;
                    break;
                }
            }
            if matched { continue; }
        }

        if top_n < MAX_TOP {
            top[top_n] = (false, i);
            top_n += 1;
        }
    }

    for ti in 0..top_n {
        let is_last = ti + 1 == top_n;
        let conn = if is_last { "\u{2514}\u{2500}" } else { "\u{251C}\u{2500}" };
        let cont = if is_last { "  " } else { "\u{2502} " };

        let (is_port, idx) = top[ti];
        if is_port {
            // Kernel bus port — show port, then service that claimed it as child
            emit_port_line(&mut out, "", conn, &ports[idx]);

            // Find the service that claimed this bus port (bus_path == port name)
            let pname = port_name_str(&ports[idx].name);
            let claimer = find_service_by_bus_path(&services, svc_count, pname);

            if let Some(si) = claimer {
                render_service_subtree(&mut out, cont, &services, svc_count, &ports, port_count, si);
            }
        } else {
            // Top-level service (no bus port parent)
            emit_svc_line(&mut out, "", conn, &services[idx]);
            render_service_children(&mut out, cont, &services, svc_count, &ports, port_count, idx);
        }
    }

    crate::print!("{}", out.as_str());
    CommandResult::None
}

/// Render a service as a single child under its parent, then recurse into its subtree.
fn render_service_subtree(
    out: &mut StringBuf, prefix: &str,
    services: &[ServiceEntry], svc_count: usize,
    ports: &[PortEntry], port_count: usize,
    si: usize,
) {
    // This service is the only child at this level
    let conn = "\u{2514}\u{2500}";
    emit_svc_line(out, prefix, conn, &services[si]);

    let mut child_prefix = PrefixBuf::new();
    child_prefix.push(prefix);
    child_prefix.push("  ");
    render_service_children(out, child_prefix.as_str(), services, svc_count, ports, port_count, si);
}

/// Render ports and children of a service.
fn render_service_children(
    out: &mut StringBuf, prefix: &str,
    services: &[ServiceEntry], svc_count: usize,
    ports: &[PortEntry], port_count: usize,
    si: usize,
) {
    let svc = &services[si];

    // Collect ports owned by this service
    let mut svc_ports = [0usize; 8];
    let mut sp_n = 0usize;
    for j in 0..port_count {
        if ports[j].owner_idx == svc.index && sp_n < 8 {
            svc_ports[sp_n] = j;
            sp_n += 1;
        }
    }

    for pi in 0..sp_n {
        let pj = svc_ports[pi];
        let port = &ports[pj];
        let pname = port_name_str(&port.name);
        let is_last = pi + 1 == sp_n;
        let port_conn = if is_last { "\u{2514}\u{2500}" } else { "\u{251C}\u{2500}" };
        emit_port_line(out, prefix, port_conn, port);

        // Find services that claimed this port (bus_path == port name)
        let claimer = find_service_by_bus_path(services, svc_count, pname);
        if let Some(ci) = claimer {
            let l1_cont = if is_last { "  " } else { "\u{2502} " };
            let mut child_prefix = PrefixBuf::new();
            child_prefix.push(prefix);
            child_prefix.push(l1_cont);
            render_service_subtree(out, child_prefix.as_str(), services, svc_count, ports, port_count, ci);
        }
    }
}

/// Find a service whose bus_path matches the given port name.
fn find_service_by_bus_path(services: &[ServiceEntry], count: usize, port_name: &str) -> Option<usize> {
    for i in 0..count {
        let bp = svc_bus_path_str(&services[i]);
        if !bp.is_empty() && bp == port_name {
            return Some(i);
        }
    }
    None
}

/// Emit a service line: prefix + conn + icon + label ···· pid ● state
fn emit_svc_line(out: &mut StringBuf, prefix: &str, conn: &str, svc: &ServiceEntry) {
    let name = name_str(&svc.name);
    let bus = extract_bus_short(svc);

    // Left content with ANSI colors
    let _ = write!(out, "\x1b[2m{}{}\x1b[0m ", prefix, conn);
    let left_dw;
    if let Some(b) = bus {
        let _ = write!(out, "\x1b[33m\u{25C6}\x1b[0m {} \u{2192} \x1b[1m{}\x1b[0m", b, name);
        // visible: prefix + conn(2) + sp(1) + ◆(1) + sp(1) + bus + sp(1) + →(1) + sp(1) + name
        left_dw = visible_len(prefix) + b.len() + name.len() + 8;
    } else {
        let _ = write!(out, "\x1b[1m\u{25CF}\x1b[0m \x1b[1m{}\x1b[0m", name);
        // visible: prefix + conn(2) + sp(1) + ●(1) + sp(1) + name
        left_dw = visible_len(prefix) + name.len() + 5;
    }

    // Right: " pid ● state"
    let state = svc.state_str();
    let sc = state_color(svc.state);
    let pid_dw = digit_count(svc.pid);
    // visible after dots: sp(1) + pid + sp(1) + ●(1) + sp(1) + state
    let right_dw = pid_dw + state.len() + 4;

    // Dots fill gap: left + sp + dots + right_visible = LINE_WIDTH
    let dots = LINE_WIDTH.saturating_sub(left_dw + right_dw + 1);
    let _ = write!(out, " \x1b[2m");
    for _ in 0..dots.max(1) { let _ = write!(out, "\u{00B7}"); }
    let _ = writeln!(out, "\x1b[0m {} {}\u{25CF}\x1b[0m {}", svc.pid, sc, state);
}

/// Emit a port line: prefix + conn + ○ name ···· type
fn emit_port_line(out: &mut StringBuf, prefix: &str, conn: &str, port: &PortEntry) {
    let pname_full = port_name_str(&port.name);
    // Strip /kernel/bus/ prefix for display
    let pname = pname_full.strip_prefix("/kernel/bus/").unwrap_or(pname_full);
    let type_s = port_type_str(port.port_type);
    let has_data = port.flags & port_flags::HAS_DATAPORT != 0;

    // Left content
    let _ = write!(out, "\x1b[2m{}{}\x1b[0m \x1b[36m\u{25CB}\x1b[0m \x1b[36m{}\x1b[0m",
                   prefix, conn, pname);
    // visible: prefix + conn(2) + sp(1) + ○(1) + sp(1) + pname
    let left_dw = visible_len(prefix) + pname.len() + 5;

    // Right: "     type [shmem=N]"
    // 5 spaces pad where PID column would be
    let mut right_dw = 6 + type_s.len(); // sp_after_dots(1) + 5_spaces + type
    if has_data && port.shmem_id > 0 {
        right_dw += 7 + digit_count(port.shmem_id); // " shmem=N"
    }

    // Dots
    let dots = LINE_WIDTH.saturating_sub(left_dw + right_dw + 1);
    let _ = write!(out, " \x1b[2m");
    for _ in 0..dots.max(1) { let _ = write!(out, "\u{00B7}"); }
    let _ = write!(out, "\x1b[0m      {}", type_s);
    if has_data && port.shmem_id > 0 {
        let _ = write!(out, " \x1b[35mshmem={}\x1b[0m", port.shmem_id);
    }
    let _ = writeln!(out);
}

// ───── Helper functions ─────────────────────────────────────

/// Compute visible (display) width of a string, ignoring ANSI escape sequences
fn visible_len(s: &str) -> usize {
    let b = s.as_bytes();
    let mut n = 0usize;
    let mut i = 0;
    while i < b.len() {
        if b[i] == 0x1b {
            // Skip ESC[...m
            i += 1;
            while i < b.len() && b[i] != b'm' { i += 1; }
            if i < b.len() { i += 1; }
        } else if b[i] & 0xC0 == 0x80 {
            // UTF-8 continuation byte - skip
            i += 1;
        } else {
            n += 1;
            i += 1;
        }
    }
    n
}

/// Extract short bus name from bus_path (e.g., "/kernel/bus/uart0" → "uart0")
fn extract_bus_short(svc: &ServiceEntry) -> Option<&str> {
    if !svc.has_bus_path() { return None; }
    let bp = core::str::from_utf8(svc.bus_path_str()).ok()?;
    bp.strip_prefix("/kernel/bus/").filter(|s| !s.is_empty())
}

/// Get bus_path as str for matching children to ports
fn svc_bus_path_str(svc: &ServiceEntry) -> &str {
    if !svc.has_bus_path() { return ""; }
    core::str::from_utf8(svc.bus_path_str()).unwrap_or("")
}

/// ANSI color code for service state
fn state_color(state: u8) -> &'static str {
    match state {
        service_state::READY => "\x1b[32m",
        service_state::STARTING => "\x1b[33m",
        service_state::CRASHED | service_state::FAILED => "\x1b[31m",
        _ => "\x1b[2m",
    }
}

/// Number of decimal digits in a u32
fn digit_count(n: u32) -> usize {
    if n == 0 { return 1; }
    let mut c = 0usize;
    let mut v = n;
    while v > 0 { c += 1; v /= 10; }
    c
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

/// Small buffer for building tree prefix strings
struct PrefixBuf {
    buf: [u8; 16],
    len: usize,
}

impl PrefixBuf {
    fn new() -> Self { Self { buf: [0; 16], len: 0 } }

    fn push(&mut self, s: &str) {
        let b = s.as_bytes();
        let n = b.len().min(self.buf.len() - self.len);
        self.buf[self.len..self.len + n].copy_from_slice(&b[..n]);
        self.len += n;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

// ───── Table displays ───────────────────────────────────────

/// Display services as a table
fn cmd_services() -> CommandResult {
    let mut client = match DevdClient::connect() {
        Ok(c) => c,
        Err(_) => return CommandResult::Error("failed to connect to devd"),
    };

    let (services, count) = match client.list_services() {
        Ok(r) => r,
        Err(_) => return CommandResult::Error("failed to list services"),
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
        Err(_) => return CommandResult::Error("failed to connect to devd"),
    };

    let (ports, count) = match client.list_ports() {
        Ok(r) => r,
        Err(_) => return CommandResult::Error("failed to list ports"),
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

// ───── Utility ──────────────────────────────────────────────

/// Check if a port type represents a hardware bus (kernel bus port)
fn is_hw_port_type(pt: u8) -> bool {
    matches!(pt, port_type::PCIE | port_type::UART | port_type::KLOG | port_type::ETHERNET | port_type::USB)
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
        port_type::STORAGE => "storage",
        port_type::PCIE => "pcie",
        port_type::UART => "uart",
        port_type::KLOG => "klog",
        port_type::ETHERNET => "ethernet",
        _ => "unknown",
    }
}

/// Convert service name bytes to str
fn name_str(name: &[u8; 16]) -> &str {
    let len = name.iter().position(|&b| b == 0).unwrap_or(16);
    core::str::from_utf8(&name[..len]).unwrap_or("???")
}

/// Convert port name bytes to str
fn port_name_str(name: &[u8; 18]) -> &str {
    let len = name.iter().position(|&b| b == 0).unwrap_or(18);
    core::str::from_utf8(&name[..len]).unwrap_or("???")
}
