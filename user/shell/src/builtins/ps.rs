//! Process List Builtin
//!
//! Display running processes with structured output.
//!
//! Usage:
//!   ps              - Show all processes

use userlib::syscall;
use crate::output::{Table, Row, Align};

/// Main entry point for ps builtin
pub fn run(_args: &[u8]) -> Table {
    let mut buf: [syscall::ProcessInfo; 32] = [syscall::ProcessInfo::empty(); 32];
    let count = syscall::ps_info(&mut buf);

    let mut table = Table::new(&["PID", "PPID", "CPU", "STATE", "LIVENESS", "AGE", "NAME"])
        .align(0, Align::Right)   // PID
        .align(1, Align::Right)   // PPID
        .align(2, Align::Right)   // CPU
        .align(5, Align::Right);  // AGE

    for i in 0..count {
        let info = &buf[i];

        // Liveness status string
        let liveness = match info.liveness_status {
            0 => "ok",
            1 => "ping",
            2 => "closing",
            _ => "?",
        };

        // Build the row
        let mut row = Row::empty()
            .uint(info.pid as u64)
            .uint(info.ppid as u64);

        row = if info.cpu != 0xFF {
            row.uint(info.cpu as u64)
        } else {
            row.str("-")
        };

        row = row
            .str(info.state_str())
            .str(liveness)
            .uint(info.activity_age_ms as u64)
            .bytes(&info.name);

        table.add_row(row);
    }

    table
}
