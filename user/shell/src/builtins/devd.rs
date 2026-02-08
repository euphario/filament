//! devd Builtin - Device Supervisor Control
//!
//! Control the device supervisor daemon via IPC.
//!
//! Usage:
//!   devd list                           - List all services
//!   devd help                           - Show help
//!
//! Note: The devd protocol is being rewritten. Some commands may not work.

use crate::println;
use userlib::ipc::{Channel, Timer, Mux, MuxFilter};
use userlib::syscall;
use crate::output::CommandResult;

/// Main entry point for devd builtin
pub fn run(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    if args.is_empty() || args == b"help" {
        show_help();
        return CommandResult::None;
    }

    // Parse subcommand
    if crate::cmd_starts_with(args, b"spawn ") {
        cmd_spawn(&args[6..])
    } else if crate::cmd_starts_with(args, b"start ") {
        cmd_start(&args[6..])
    } else if crate::cmd_starts_with(args, b"stop ") {
        cmd_stop(&args[5..])
    } else if crate::cmd_starts_with(args, b"restart ") {
        cmd_restart(&args[8..])
    } else if crate::cmd_eq(args, b"list") || crate::cmd_eq(args, b"ls") {
        cmd_list()
    } else {
        println!("Unknown devd command. Try 'devd help'");
        CommandResult::None
    }
}

fn show_help() {
    println!("devd - Device Supervisor Control");
    println!();
    println!("Service commands:");
    println!("  devd start <service>        Start a service");
    println!("  devd stop <service>         Stop a service");
    println!("  devd restart <service>      Restart a service");
    println!("  devd list                   List all services");
    println!();
    println!("Driver commands:");
    println!("  devd spawn <driver> <path>  Spawn driver for device");
    println!();
    println!("Examples:");
    println!("  devd start consoled");
    println!("  devd restart logd");
    println!("  devd spawn wifid /bus/pcie0/01:00.0");
}

fn cmd_spawn(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    // Parse: <driver> <device-path>
    let (driver, path) = match parse_two_args(args) {
        Some((d, p)) => (d, p),
        None => {
            println!("Usage: devd spawn <driver> <device-path>");
            return CommandResult::None;
        }
    };

    match send_command(&format_msg(b"SPAWN ", driver.as_bytes(), b" ", path.as_bytes())) {
        Ok(resp) => {
            println!("{}", resp);
        }
        Err(e) => {
            println!("Failed: {}", e);
        }
    }

    CommandResult::None
}

fn cmd_start(args: &[u8]) -> CommandResult {
    let name = match core::str::from_utf8(crate::trim(args)) {
        Ok(s) => s,
        Err(_) => {
            println!("Invalid service name");
            return CommandResult::None;
        }
    };

    if name.is_empty() {
        println!("Usage: devd start <service>");
        return CommandResult::None;
    }

    match send_simple_command(b"START ", name.as_bytes()) {
        Ok(resp) => println!("{}", resp),
        Err(e) => println!("Failed: {}", e),
    }

    CommandResult::None
}

fn cmd_stop(args: &[u8]) -> CommandResult {
    let name = match core::str::from_utf8(crate::trim(args)) {
        Ok(s) => s,
        Err(_) => {
            println!("Invalid service name");
            return CommandResult::None;
        }
    };

    if name.is_empty() {
        println!("Usage: devd stop <service>");
        return CommandResult::None;
    }

    match send_simple_command(b"STOP ", name.as_bytes()) {
        Ok(resp) => println!("{}", resp),
        Err(e) => println!("Failed: {}", e),
    }

    CommandResult::None
}

fn cmd_restart(args: &[u8]) -> CommandResult {
    let name = match core::str::from_utf8(crate::trim(args)) {
        Ok(s) => s,
        Err(_) => {
            println!("Invalid service name");
            return CommandResult::None;
        }
    };

    if name.is_empty() {
        println!("Usage: devd restart <service>");
        return CommandResult::None;
    }

    match send_simple_command(b"RESTART ", name.as_bytes()) {
        Ok(resp) => println!("{}", resp),
        Err(e) => println!("Failed: {}", e),
    }

    CommandResult::None
}

fn cmd_list() -> CommandResult {
    match send_command(b"LIST\n") {
        Ok(resp) => {
            println!("Services:");
            println!("{}", resp);
        }
        Err(e) => println!("Failed: {}", e),
    }

    CommandResult::None
}

/// Send a simple command with one argument: "CMD arg\n"
fn send_simple_command(cmd: &[u8], arg: &[u8]) -> Result<&'static str, &'static str> {
    let mut buf = [0u8; 128];
    let mut len = 0;

    // Copy command
    for &b in cmd {
        if len < buf.len() { buf[len] = b; len += 1; }
    }
    // Copy arg
    for &b in arg {
        if len < buf.len() { buf[len] = b; len += 1; }
    }
    // Newline
    if len < buf.len() { buf[len] = b'\n'; len += 1; }

    send_command(&buf[..len])
}

/// Format a message with multiple parts
fn format_msg(a: &[u8], b: &[u8], c: &[u8], d: &[u8]) -> [u8; 128] {
    let mut buf = [0u8; 128];
    let mut len = 0;
    for &byte in a { if len < 127 { buf[len] = byte; len += 1; } }
    for &byte in b { if len < 127 { buf[len] = byte; len += 1; } }
    for &byte in c { if len < 127 { buf[len] = byte; len += 1; } }
    for &byte in d { if len < 127 { buf[len] = byte; len += 1; } }
    if len < 128 { buf[len] = b'\n'; }
    buf
}

/// Send a command to devd and get response
fn send_command(cmd: &[u8]) -> Result<&'static str, &'static str> {
    // Connect to devd (admin commands via query port)
    let mut channel = match Channel::connect(b"devd-query:") {
        Ok(ch) => ch,
        Err(_) => return Err("failed to connect to devd"),
    };

    // Send command
    if channel.send(cmd).is_err() {
        return Err("send failed");
    }

    // Wait for response with timeout using Mux + Timer
    let mux = match Mux::new() {
        Ok(m) => m,
        Err(_) => return Err("failed to create mux"),
    };

    let mut timer = match Timer::new() {
        Ok(t) => t,
        Err(_) => return Err("failed to create timer"),
    };

    // 5 second timeout
    let now = syscall::gettime();
    let deadline = now + 5_000_000_000; // 5 seconds in ns
    if timer.set(deadline).is_err() {
        return Err("failed to set timer");
    }

    // Watch channel and timer
    let _ = mux.add(channel.handle(), MuxFilter::Readable);
    let _ = mux.add(timer.handle(), MuxFilter::Readable);

    // Wait for either response or timeout
    let mut resp = [0u8; 256];
    match mux.wait() {
        Ok(event) => {
            if event.handle == timer.handle() {
                return Err("no response (timeout)");
            }
            // Channel ready - read response
            match channel.recv(&mut resp) {
                Ok(n) if n > 0 => {
                    if resp[0] == b'O' && resp[1] == b'K' {
                        Ok("OK")
                    } else if resp[0] == b'E' && resp[1] == b'R' && resp[2] == b'R' {
                        Ok("Error from devd")
                    } else {
                        Ok("Response received")
                    }
                }
                _ => Err("recv failed"),
            }
        }
        Err(_) => Err("wait failed"),
    }
}

/// Parse two space-separated arguments
fn parse_two_args(args: &[u8]) -> Option<(&str, &str)> {
    let args_str = core::str::from_utf8(args).ok()?;
    let mut parts = args_str.split_whitespace();
    let first = parts.next()?;
    let second = parts.next()?;
    Some((first, second))
}
