//! GPIO Expander Control (PCA9555)
//!
//! Shell builtin for quick GPIO operations on BPI-R4's PCA9555 expander.
//!
//! NOTE: This builtin requires the I2C scheme system which is not yet
//! implemented with the new unified API. Use the gpio daemon instead.
//!
//! Usage:
//!   gpio              - Show all pin states
//!   gpio read <pin>   - Read a specific pin (0-15)
//!   gpio set <pin> <0|1> - Set pin as output with value
//!   gpio usb <on|off> - Control USB VBUS power (pin 11)

use crate::println;

/// Main entry point for gpio builtin
pub fn run(args: &[u8]) {
    let args = crate::trim(args);

    if args == b"help" || args == b"-h" || args == b"--help" {
        cmd_help();
    } else {
        println!("GPIO builtin not available");
        println!("(I2C scheme system not yet implemented with new unified API)");
        println!();
        println!("Use the gpio daemon instead:");
        println!("  spawn bin/gpio");
    }
}

fn cmd_help() {
    println!("GPIO expander control (PCA9555)");
    println!();
    println!("NOTE: This builtin is currently disabled.");
    println!("The I2C scheme system is not yet implemented with the new unified API.");
    println!();
    println!("Usage (when available):");
    println!("  gpio              - Show all pin states");
    println!("  gpio read <pin>   - Read pin (0-15 or name)");
    println!("  gpio set <pin> <0|1> - Set pin output");
    println!("  gpio usb [on|off] - USB VBUS power control");
}
