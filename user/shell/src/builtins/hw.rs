//! Hardware Query Builtin
//!
//! Query devd for device information.
//!
//! NOTE: This builtin requires the hw: scheme system which is not yet
//! implemented with the new unified API.
//!
//! Usage:
//!   hw              - Show all devices (hw list)
//!   hw list         - Show all devices
//!   hw bus          - List bus controllers
//!   hw tree         - Show device tree
//!   hw <path>       - Query specific device path

use crate::println;
use crate::output::CommandResult;

/// Main entry point for hw builtin
pub fn run(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    if args == b"help" || args == b"-h" || args == b"--help" {
        cmd_help();
        return CommandResult::None;
    }

    println!("Hardware query not available");
    println!("(hw: scheme system not yet implemented with new unified API)");
    println!();
    println!("Use 'ps' to see running processes.");

    CommandResult::None
}

fn cmd_help() {
    println!("Hardware query (via devd)");
    println!();
    println!("NOTE: This builtin is currently disabled.");
    println!("The hw: scheme system is not yet implemented with the new unified API.");
    println!();
    println!("Usage (when available):");
    println!("  hw              - Show all devices");
    println!("  hw list         - Show all devices");
    println!("  hw bus          - List bus controllers");
    println!("  hw tree         - Show device tree");
    println!("  hw <path>       - Query specific device path");
}
