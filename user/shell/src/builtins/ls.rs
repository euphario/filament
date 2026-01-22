//! Directory Listing Builtin
//!
//! List directory contents. Currently only shows known binaries since vfsd
//! is not yet implemented with the new unified API.
//!
//! Usage:
//!   ls              - List /bin directory
//!   ls <path>       - List specified directory (limited)

use crate::println;
use crate::output::{Table, Row, Align, CommandResult};

/// Known binaries in /bin (embedded in initrd)
const KNOWN_BINARIES: &[&str] = &[
    "shell",
    "devd",
    "consoled",
    "logd",
    "pcied",
    "usbd",
    "fatfs",
    "wifid",
    "nvmed",
    "gpio",
    "pwm",
    "pciepoke",
];

/// Main entry point for ls builtin
pub fn run(args: &[u8]) -> CommandResult {
    let path = crate::trim(args);

    // Default to /bin if no path given
    let path = if path.is_empty() { b"/bin" as &[u8] } else { path };

    // Only support /bin for now
    if path != b"/bin" && path != b"bin" && path != b"." {
        println!("ls: vfsd not available, only /bin supported");
        return CommandResult::None;
    }

    // Build structured table
    let mut table = Table::new(&["TYPE", "NAME"])
        .align(0, Align::Left);

    for &name in KNOWN_BINARIES {
        let row = Row::empty()
            .str("file")
            .str(name);
        table.add_row(row);
    }

    CommandResult::Table(table)
}
