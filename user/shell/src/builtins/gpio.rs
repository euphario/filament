//! GPIO Expander Control (PCA9555)
//!
//! Shell builtin for quick GPIO operations on BPI-R4's PCA9555 expander.
//! For continuous monitoring, use the full `gpio` daemon instead.
//!
//! Usage:
//!   gpio              - Show all pin states
//!   gpio read <pin>   - Read a specific pin (0-15)
//!   gpio set <pin> <0|1> - Set pin as output with value
//!   gpio usb <on|off> - Control USB VBUS power (pin 11)
//!
//! I2C Topology:
//!   I2C2 -> PCA9545@0x70 (channel 3) -> PCA9555@0x20

use userlib::{println, syscall};
use crate::{trim, parse_decimal};

// I2C addresses
const PCA9545_ADDR: &str = "i2c:2/70";  // I2C mux
const PCA9555_ADDR: &str = "i2c:2/20";  // GPIO expander
const MUX_CHANNEL_3: u8 = 0x08;

// PCA9555 registers
const INPUT_PORT0: u8 = 0x00;
const INPUT_PORT1: u8 = 0x01;
const OUTPUT_PORT0: u8 = 0x02;
const OUTPUT_PORT1: u8 = 0x03;
const CONFIG_PORT0: u8 = 0x06;
const CONFIG_PORT1: u8 = 0x07;

// Pin definitions
const GPIO_USB_VBUS: u8 = 11;

// Pin names
const PIN_NAMES: [&str; 16] = [
    "io0_0", "io0_1", "io0_2", "io0_3",
    "io0_4", "io0_5", "io0_6", "io0_7",
    "sfp_txfault", "sfp_los", "sfp_present",
    "usb_vbus",
    "sfp_txdis", "sfp_rate", "sfp_led",
    "io1_7",
];

/// Simple I2C GPIO context (opened/closed per command)
struct GpioCtx {
    mux_fd: u32,
    gpio_fd: u32,
}

impl GpioCtx {
    fn open() -> Option<Self> {
        let mux_fd = syscall::scheme_open(PCA9545_ADDR, 2);
        if mux_fd < 0 {
            println!("Failed to open I2C mux: {}", mux_fd);
            return None;
        }

        let gpio_fd = syscall::scheme_open(PCA9555_ADDR, 2);
        if gpio_fd < 0 {
            println!("Failed to open I2C GPIO: {}", gpio_fd);
            syscall::close(mux_fd as u32);
            return None;
        }

        Some(Self {
            mux_fd: mux_fd as u32,
            gpio_fd: gpio_fd as u32,
        })
    }

    fn select_mux(&self) -> bool {
        syscall::write(self.mux_fd, &[MUX_CHANNEL_3]) >= 0
    }

    fn read_reg(&self, reg: u8) -> Option<u8> {
        if !self.select_mux() {
            return None;
        }
        if syscall::write(self.gpio_fd, &[reg]) < 0 {
            return None;
        }
        let mut buf = [0u8];
        if syscall::read(self.gpio_fd, &mut buf) < 0 {
            return None;
        }
        Some(buf[0])
    }

    fn write_reg(&self, reg: u8, value: u8) -> bool {
        if !self.select_mux() {
            return false;
        }
        syscall::write(self.gpio_fd, &[reg, value]) >= 0
    }

    fn close(self) {
        syscall::close(self.gpio_fd);
        syscall::close(self.mux_fd);
    }
}

/// Main entry point for gpio builtin
pub fn run(args: &[u8]) {
    let args = trim(args);

    if args.is_empty() {
        cmd_status();
    } else if args.starts_with(b"read ") {
        cmd_read(&args[5..]);
    } else if args.starts_with(b"set ") {
        cmd_set(&args[4..]);
    } else if args.starts_with(b"usb ") {
        cmd_usb(&args[4..]);
    } else if args == b"usb" {
        cmd_usb_status();
    } else if args == b"help" || args == b"-h" || args == b"--help" {
        cmd_help();
    } else {
        println!("Unknown gpio command. Try 'gpio help'");
    }
}

fn cmd_help() {
    println!("GPIO expander control (PCA9555)");
    println!();
    println!("Usage:");
    println!("  gpio              - Show all pin states");
    println!("  gpio read <pin>   - Read pin (0-15 or name)");
    println!("  gpio set <pin> <0|1> - Set pin output");
    println!("  gpio usb [on|off] - USB VBUS power control");
    println!();
    println!("Pin names: usb_vbus(11), sfp_txfault(8), sfp_los(9),");
    println!("           sfp_present(10), sfp_txdis(12), sfp_rate(13), sfp_led(14)");
}

fn cmd_status() {
    let ctx = match GpioCtx::open() {
        Some(c) => c,
        None => return,
    };

    let in0 = ctx.read_reg(INPUT_PORT0).unwrap_or(0xFF);
    let in1 = ctx.read_reg(INPUT_PORT1).unwrap_or(0xFF);
    let cfg0 = ctx.read_reg(CONFIG_PORT0).unwrap_or(0xFF);
    let cfg1 = ctx.read_reg(CONFIG_PORT1).unwrap_or(0xFF);
    let out0 = ctx.read_reg(OUTPUT_PORT0).unwrap_or(0xFF);
    let out1 = ctx.read_reg(OUTPUT_PORT1).unwrap_or(0xFF);

    ctx.close();

    println!("PIN  NAME          DIR  VALUE");
    println!("---  ------------  ---  -----");

    for pin in 0u8..16 {
        let port = (pin / 8) as usize;
        let bit = pin % 8;

        let (cfg, inp, out) = if port == 0 {
            (cfg0, in0, out0)
        } else {
            (cfg1, in1, out1)
        };

        let is_input = (cfg >> bit) & 1 != 0;
        let value = if is_input {
            (inp >> bit) & 1
        } else {
            (out >> bit) & 1
        };

        let dir = if is_input { "in " } else { "out" };
        let name = PIN_NAMES[pin as usize];

        println!("{:2}   {:12}  {}  {}", pin, name, dir, value);
    }
}

fn cmd_read(args: &[u8]) {
    let pin = match parse_pin(args) {
        Some(p) => p,
        None => {
            println!("Invalid pin. Use 0-15 or pin name.");
            return;
        }
    };

    let ctx = match GpioCtx::open() {
        Some(c) => c,
        None => return,
    };

    let reg = if pin < 8 { INPUT_PORT0 } else { INPUT_PORT1 };
    let bit = pin % 8;

    let value = match ctx.read_reg(reg) {
        Some(v) => (v >> bit) & 1,
        None => {
            println!("Failed to read pin {}", pin);
            ctx.close();
            return;
        }
    };

    ctx.close();
    println!("{} (pin {}) = {}", PIN_NAMES[pin as usize], pin, value);
}

fn cmd_set(args: &[u8]) {
    let args = trim(args);

    // Parse: <pin> <value>
    let (pin_str, val_str) = match args.iter().position(|&c| c == b' ') {
        Some(pos) => (&args[..pos], trim(&args[pos + 1..])),
        None => {
            println!("Usage: gpio set <pin> <0|1>");
            return;
        }
    };

    let pin = match parse_pin(pin_str) {
        Some(p) => p,
        None => {
            println!("Invalid pin. Use 0-15 or pin name.");
            return;
        }
    };

    let value = if val_str == b"1" || val_str == b"on" || val_str == b"high" {
        true
    } else if val_str == b"0" || val_str == b"off" || val_str == b"low" {
        false
    } else {
        println!("Invalid value. Use 0/1, on/off, or high/low.");
        return;
    };

    let ctx = match GpioCtx::open() {
        Some(c) => c,
        None => return,
    };

    let port = (pin / 8) as usize;
    let bit = pin % 8;
    let mask = 1u8 << bit;

    // Read current config and output
    let cfg_reg = if port == 0 { CONFIG_PORT0 } else { CONFIG_PORT1 };
    let out_reg = if port == 0 { OUTPUT_PORT0 } else { OUTPUT_PORT1 };

    let mut cfg = ctx.read_reg(cfg_reg).unwrap_or(0xFF);
    let mut out = ctx.read_reg(out_reg).unwrap_or(0xFF);

    // Set as output (config bit = 0)
    cfg &= !mask;

    // Set value
    if value {
        out |= mask;
    } else {
        out &= !mask;
    }

    // Write config then output
    if !ctx.write_reg(cfg_reg, cfg) {
        println!("Failed to set pin direction");
        ctx.close();
        return;
    }

    if !ctx.write_reg(out_reg, out) {
        println!("Failed to set pin value");
        ctx.close();
        return;
    }

    ctx.close();
    println!("{} (pin {}) <- {}", PIN_NAMES[pin as usize], pin, if value { 1 } else { 0 });
}

fn cmd_usb(args: &[u8]) {
    let args = trim(args);

    let value = if args == b"on" || args == b"1" || args == b"enable" {
        true
    } else if args == b"off" || args == b"0" || args == b"disable" {
        false
    } else {
        println!("Usage: gpio usb <on|off>");
        return;
    };

    // Reuse cmd_set logic
    let pin_str = b"11";
    let val_str = if value { b"1" } else { b"0" };

    // Build args for cmd_set: "11 1" or "11 0"
    let mut buf = [0u8; 8];
    buf[0..2].copy_from_slice(pin_str);
    buf[2] = b' ';
    buf[3] = val_str[0];

    cmd_set(&buf[..4]);
}

fn cmd_usb_status() {
    let ctx = match GpioCtx::open() {
        Some(c) => c,
        None => return,
    };

    let in1 = ctx.read_reg(INPUT_PORT1).unwrap_or(0xFF);
    let cfg1 = ctx.read_reg(CONFIG_PORT1).unwrap_or(0xFF);
    let out1 = ctx.read_reg(OUTPUT_PORT1).unwrap_or(0xFF);

    ctx.close();

    let bit = GPIO_USB_VBUS % 8;
    let is_output = (cfg1 >> bit) & 1 == 0;
    let value = if is_output {
        (out1 >> bit) & 1
    } else {
        (in1 >> bit) & 1
    };

    let status = if value != 0 { "ON (5V enabled)" } else { "OFF (5V disabled)" };
    println!("USB VBUS: {}", status);
}

/// Parse pin number or name
fn parse_pin(s: &[u8]) -> Option<u8> {
    let s = trim(s);

    // Try numeric first
    if let Some(n) = parse_decimal(s) {
        if n < 16 {
            return Some(n as u8);
        }
    }

    // Try name lookup
    if let Ok(name) = core::str::from_utf8(s) {
        for (i, &pin_name) in PIN_NAMES.iter().enumerate() {
            if name.eq_ignore_ascii_case(pin_name) {
                return Some(i as u8);
            }
        }
    }

    None
}
