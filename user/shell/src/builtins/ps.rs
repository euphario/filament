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

    let mut table = Table::new(&["PID", "PPID", "STATE", "HB", "AGE", "NAME"])
        .align(0, Align::Right)   // PID
        .align(1, Align::Right);  // PPID

    for i in 0..count {
        let info = &buf[i];

        // Format heartbeat age
        let age_str: &'static str = if info.heartbeat_age_ms == 0 {
            "-"
        } else if info.heartbeat_age_ms < 1000 {
            "ms"
        } else if info.heartbeat_age_ms < 60000 {
            "s"
        } else {
            "m"
        };

        // Build the row
        let row = Row::empty()
            .uint(info.pid as u64)
            .uint(info.parent_pid as u64)
            .str(info.state_str())
            .str(info.heartbeat_str())
            .str(age_str)
            .bytes(&info.name);

        table.add_row(row);
    }

    table
}
