//! Process List Builtin
//!
//! Display running processes with structured output.
//!
//! Usage:
//!   ps              - Show all processes (8 columns)
//!   ps -v           - + HNDL, CH, PORT, SHMEM, LIVE
//!   ps -vv          - + CSW, PF, PAGES, MAPS
//!   ps -vvv         - + KIDS, CAPS
//!   ps -r           - Reset all stats, then show ps

use userlib::syscall;
use crate::output::{Table, Row, Align, CommandResult};

fn prio_name(level: u8) -> &'static str {
    match level {
        0 => "RT",
        1 => "Crit",
        2 => "High",
        3 => "AbvN",
        4 => "Norm",
        5 => "BlwN",
        6 => "Low",
        7 => "Idle",
        _ => "?",
    }
}

/// Format activity age as human-readable idle time
fn idle_str(age_ms: u32) -> &'static str {
    if age_ms == 0 {
        "-"
    } else if age_ms < 1000 {
        "<1s"
    } else if age_ms < 3000 {
        "3s"
    } else if age_ms < 15000 {
        "15s"
    } else if age_ms < 60000 {
        "1m"
    } else if age_ms < 300000 {
        "5m"
    } else if age_ms < 3600000 {
        "1h"
    } else {
        ">1h"
    }
}

/// Liveness status as short string
fn liveness_str(status: u8) -> &'static str {
    match status {
        0 => "ok",
        1 => "ping",
        2 => "clos",
        _ => "?",
    }
}

/// Parse flags from args
fn parse_flags(args: &[u8]) -> (u8, bool) {
    let args = libf::str::trim(args);
    if args.is_empty() {
        return (0, false);
    }
    // Support: -v, -vv, -vvv, -r
    if args.first() == Some(&b'-') {
        let rest = &args[1..];
        if rest == b"r" {
            return (0, true);
        }
        let v_count = rest.iter().take_while(|&&c| c == b'v').count();
        if v_count > 0 && v_count == rest.len() {
            return ((v_count as u8).min(3), false);
        }
    }
    (0, false)
}

/// Main entry point for ps builtin
pub fn run(args: &[u8]) -> CommandResult {
    let (verbosity, reset) = parse_flags(args);

    if reset {
        syscall::reset_stats(0);
    }

    if verbosity == 0 {
        return CommandResult::Table(run_basic());
    }

    CommandResult::Table(run_extended(verbosity))
}

/// Basic ps with IDLE column
fn run_basic() -> Table {
    let mut buf: [syscall::ProcessInfo; 32] = [syscall::ProcessInfo::empty(); 32];
    let count = syscall::ps_info(&mut buf);

    let mut table = Table::new(&["PID", "PPID", "CPU", "PRIO", "STATE", "CPU_MS", "IDLE", "NAME"])
        .align(0, Align::Right)   // PID
        .align(1, Align::Right)   // PPID
        .align(2, Align::Right)   // CPU
        .align(5, Align::Right);  // CPU_MS

    for i in 0..count {
        let info = &buf[i];
        let cpu_ms = info.cpu_time_ns / 1_000_000;

        let mut row = Row::empty()
            .uint(info.pid as u64)
            .uint(info.ppid as u64);

        row = if info.cpu != 0xFF {
            row.uint(info.cpu as u64)
        } else {
            row.str("-")
        };

        row = if info.effective_priority != info.base_priority {
            row.str(prio_name(info.effective_priority))
        } else {
            row.str(prio_name(info.base_priority))
        };

        row = row
            .str(info.state_str())
            .uint(cpu_ms)
            .str(idle_str(info.activity_age_ms))
            .bytes(&info.name);

        table.add_row(row);
    }

    table
}

/// Extended ps with verbosity levels
fn run_extended(verbosity: u8) -> Table {
    let mut buf: [syscall::ProcessInfoEx; 32] = [syscall::ProcessInfoEx::empty(); 32];
    let count = syscall::ps_info_ex(&mut buf);

    // Build headers based on verbosity
    let headers: &[&'static str] = match verbosity {
        1 => &["PID", "PPID", "CPU", "PRIO", "STATE", "CPU_MS", "IDLE", "NAME", "HNDL", "CH", "PORT", "SHMEM", "SIG", "LIVE"],
        2 => &["PID", "PPID", "CPU", "PRIO", "STATE", "CPU_MS", "IDLE", "NAME", "HNDL", "CH", "PORT", "SHMEM", "SIG", "LIVE", "SENT", "RECV", "CSW", "PF", "PAGES", "MAPS"],
        _ => &["PID", "PPID", "CPU", "PRIO", "STATE", "CPU_MS", "IDLE", "NAME", "HNDL", "CH", "PORT", "SHMEM", "SIG", "LIVE", "SENT", "RECV", "CSW", "PF", "PAGES", "MAPS", "KIDS", "CAPS"],
    };

    let table = Table::new(headers)
        .align(0, Align::Right)   // PID
        .align(1, Align::Right)   // PPID
        .align(2, Align::Right)   // CPU
        .align(5, Align::Right)   // CPU_MS
        .align(8, Align::Right)   // HNDL
        .align(9, Align::Right)   // CH
        .align(10, Align::Right)  // PORT
        .align(11, Align::Right)  // SHMEM
        .align(12, Align::Right); // SIG

    let table = if verbosity >= 2 {
        table
            .align(14, Align::Right)  // SENT
            .align(15, Align::Right)  // RECV
            .align(16, Align::Right)  // CSW
            .align(17, Align::Right)  // PF
            .align(18, Align::Right)  // PAGES
            .align(19, Align::Right)  // MAPS
    } else {
        table
    };

    let mut table = if verbosity >= 3 {
        table.align(20, Align::Right) // KIDS
    } else {
        table
    };

    for i in 0..count {
        let info = &buf[i];
        let cpu_ms = info.cpu_time_ns / 1_000_000;

        let mut row = Row::empty()
            .uint(info.pid as u64)
            .uint(info.ppid as u64);

        row = if info.cpu != 0xFF {
            row.uint(info.cpu as u64)
        } else {
            row.str("-")
        };

        row = if info.effective_priority != info.base_priority {
            row.str(prio_name(info.effective_priority))
        } else {
            row.str(prio_name(info.base_priority))
        };

        row = row
            .str(info.state_str())
            .uint(cpu_ms)
            .str(idle_str(info.activity_age_ms))
            .bytes(&info.name);

        // -v: handles, channels, ports, shmem, signal pending, liveness
        row = row
            .uint(info.handle_count as u64)
            .uint(info.channel_count as u64)
            .uint(info.port_count as u64)
            .uint(info.shmem_count as u64)
            .uint(info.signal_pending as u64)
            .str(liveness_str(info.liveness_status));

        // -vv: IPC stats, context switches, page faults, pages, mappings
        if verbosity >= 2 {
            row = row
                .uint(info.ipc_sent as u64)
                .uint(info.ipc_recv as u64)
                .uint(info.context_switches as u64)
                .uint(info.page_faults as u64)
                .uint(info.heap_pages as u64)
                .uint(info.mapping_count as u64);
        }

        // -vvv: children, caps
        if verbosity >= 3 {
            row = row
                .uint(info.num_children as u64)
                .hex64(info.capabilities);
        }

        table.add_row(row);
    }

    table
}
