//! Drivers Builtin - Device Driver Introspection
//!
//! TODO: Reimplement via busd query protocol (devd v1 removed)
//!
//! Usage:
//!   drivers              - Show all services and ports as tree
//!   drivers services     - Show services only (table)
//!   drivers ports        - Show ports only (table)
//!   drivers <name>       - Query detailed info from a specific service
//!   drivers help         - Show help

use crate::output::CommandResult;
use crate::{println, cmd_eq, trim};

/// Main entry point for drivers builtin
pub fn run(args: &[u8]) -> CommandResult {
    let args = trim(args);

    if cmd_eq(args, b"help") {
        show_help();
        return CommandResult::None;
    }

    println!("drivers: not yet available (busd migration pending)");
    CommandResult::None
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
}
