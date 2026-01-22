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
    let mut buf: [syscall::ProcessInfo; 16] = [syscall::ProcessInfo::empty(); 16];
    let count = syscall::ps_info(&mut buf);

    let mut table = Table::new(&["PID", "PPID", "STATE", "LIVENESS", "AGE", "NAME"])
        .align(0, Align::Right)   // PID
        .align(1, Align::Right)   // PPID
        .align(4, Align::Right);  // AGE

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
        let row = Row::empty()
            .uint(info.pid as u64)
            .uint(info.ppid as u64)
            .str(info.state_str())
            .str(liveness)
            .uint(info.activity_age_ms as u64)
            .bytes(&info.name);

        table.add_row(row);
    }

    table
}
