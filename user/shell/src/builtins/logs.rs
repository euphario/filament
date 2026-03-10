//! dlog - Query and control driver logging
//!
//! TODO: Reimplement via busd query protocol (devd v1 removed)

use crate::output::CommandResult;
use crate::println;

use libf::str::trim;

pub fn run(args: &[u8]) -> CommandResult {
    let args = trim(args);

    if args == b"help" {
        println!("Usage: dlog [options]");
        println!("  dlog          Show last 20 log messages from drivers");
        println!("  dlog -n N     Show last N log messages (max 100)");
        println!("  dlog on       Enable live logging to console");
        println!("  dlog off      Disable live logging");
        println!("  dlog status   Show logging status");
        return CommandResult::None;
    }

    println!("dlog: not yet available (busd migration pending)");
    CommandResult::None
}
