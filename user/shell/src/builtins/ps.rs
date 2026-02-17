//! Process List Builtin
//!
//! Display running processes with structured output.
//!
//! Usage:
//!   ps              - Show all processes (7 columns)
//!   ps -v           - + HNDL, CH, PORT, SHMEM
//!   ps -vv          - + PAGES, MAPS
//!   ps -vvv         - + KIDS, CAPS

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

/// Parse verbosity level from args: count 'v' characters after '-'
fn parse_verbosity(args: &[u8]) -> u8 {
    let args = libf::str::trim(args);
    if args.is_empty() {
        return 0;
    }
    // Support: -v, -vv, -vvv
    if args.first() == Some(&b'-') {
        let rest = &args[1..];
        let v_count = rest.iter().take_while(|&&c| c == b'v').count();
        if v_count > 0 && v_count == rest.len() {
            return (v_count as u8).min(3);
        }
    }
    0
}

/// Main entry point for ps builtin
pub fn run(args: &[u8]) -> CommandResult {
    let verbosity = parse_verbosity(args);

    if verbosity == 0 {
        return CommandResult::Table(run_basic());
    }

    CommandResult::Table(run_extended(verbosity))
}

/// Basic ps (unchanged 7-column output)
fn run_basic() -> Table {
    let mut buf: [syscall::ProcessInfo; 32] = [syscall::ProcessInfo::empty(); 32];
    let count = syscall::ps_info(&mut buf);

    let mut table = Table::new(&["PID", "PPID", "CPU", "PRIO", "STATE", "CPU_MS", "NAME"])
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
        1 => &["PID", "PPID", "CPU", "PRIO", "STATE", "CPU_MS", "NAME", "HNDL", "CH", "PORT", "SHMEM", "SIG"],
        2 => &["PID", "PPID", "CPU", "PRIO", "STATE", "CPU_MS", "NAME", "HNDL", "CH", "PORT", "SHMEM", "SIG", "SENT", "RECV", "PAGES", "MAPS"],
        _ => &["PID", "PPID", "CPU", "PRIO", "STATE", "CPU_MS", "NAME", "HNDL", "CH", "PORT", "SHMEM", "SIG", "SENT", "RECV", "PAGES", "MAPS", "KIDS", "CAPS"],
    };

    let mut table = Table::new(headers)
        .align(0, Align::Right)   // PID
        .align(1, Align::Right)   // PPID
        .align(2, Align::Right)   // CPU
        .align(5, Align::Right)   // CPU_MS
        .align(7, Align::Right)   // HNDL
        .align(8, Align::Right)   // CH
        .align(9, Align::Right)   // PORT
        .align(10, Align::Right)  // SHMEM
        .align(11, Align::Right); // SIG

    let table = if verbosity >= 2 {
        table
            .align(12, Align::Right)  // SENT
            .align(13, Align::Right)  // RECV
            .align(14, Align::Right)  // PAGES
            .align(15, Align::Right)  // MAPS
    } else {
        table
    };

    let mut table = if verbosity >= 3 {
        table.align(16, Align::Right) // KIDS
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
            .bytes(&info.name);

        // -v: handles, channels, ports, shmem, signal pending
        row = row
            .uint(info.handle_count as u64)
            .uint(info.channel_count as u64)
            .uint(info.port_count as u64)
            .uint(info.shmem_count as u64)
            .uint(info.signal_pending as u64);

        // -vv: IPC stats, pages, mappings
        if verbosity >= 2 {
            row = row
                .uint(info.ipc_sent as u64)
                .uint(info.ipc_recv as u64)
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
