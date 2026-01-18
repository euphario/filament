//! PCIe Register Poke Tool
//!
//! Interactive tool for reading/writing MT7996 registers via pcied.
//!
//! Usage:
//!   pciepoke r <offset>           - Read 32-bit register
//!   pciepoke w <offset> <value>   - Write 32-bit register
//!   pciepoke                      - Interactive mode (read commands from stdin)
//!
//! Examples:
//!   pciepoke r 0xd7044            - Read HIF_MISC
//!   pciepoke w 0xd7044 0x001c2000 - Write HIF_MISC
//!   pciepoke r d7044              - 0x prefix optional

#![no_std]
#![no_main]

use userlib::{println, print, syscall, Stdin, uinfo, uerror};
use userlib::ipc::protocols::PcieClient;

/// Default device to access (MT7996 HIF1)
const DEFAULT_PORT: u8 = 0;
const DEFAULT_BUS: u8 = 1;
const DEFAULT_DEVICE: u8 = 0;
const DEFAULT_FUNCTION: u8 = 0;

/// Parse a hex number with optional 0x prefix
fn parse_hex(s: &str) -> Option<u32> {
    let s = s.trim();
    let s = if s.starts_with("0x") || s.starts_with("0X") {
        &s[2..]
    } else {
        s
    };

    if s.is_empty() {
        return None;
    }

    let mut result: u32 = 0;
    for c in s.chars() {
        let digit = match c {
            '0'..='9' => c as u32 - '0' as u32,
            'a'..='f' => c as u32 - 'a' as u32 + 10,
            'A'..='F' => c as u32 - 'A' as u32 + 10,
            '_' => continue, // Allow underscores
            _ => return None,
        };
        result = result.checked_mul(16)?.checked_add(digit)?;
    }
    Some(result)
}

/// Read a register and print it
fn do_read(client: &mut PcieClient, offset: u32) {
    match client.read_register(DEFAULT_PORT, DEFAULT_BUS, DEFAULT_DEVICE, DEFAULT_FUNCTION, offset) {
        Ok(value) => {
            println!("0x{:05x} = 0x{:08x}", offset, value);
        }
        Err(e) => {
            println!("ERROR: read 0x{:05x} failed: {:?}", offset, e);
        }
    }
}

/// Write a register and print confirmation
fn do_write(client: &mut PcieClient, offset: u32, value: u32) {
    match client.write_register(DEFAULT_PORT, DEFAULT_BUS, DEFAULT_DEVICE, DEFAULT_FUNCTION, offset, value) {
        Ok(true) => {
            println!("0x{:05x} <- 0x{:08x} OK", offset, value);
        }
        Ok(false) => {
            println!("ERROR: write 0x{:05x} <- 0x{:08x} failed", offset, value);
        }
        Err(e) => {
            println!("ERROR: write 0x{:05x} failed: {:?}", offset, e);
        }
    }
}

/// Process a single command line
fn process_command(client: &mut PcieClient, line: &str) -> bool {
    let line = line.trim();

    // Skip empty lines and comments
    if line.is_empty() || line.starts_with('#') {
        return true;
    }

    // Split into words
    let mut words = line.split_whitespace();
    let cmd = match words.next() {
        Some(c) => c,
        None => return true,
    };

    match cmd {
        "r" | "read" => {
            if let Some(offset_str) = words.next() {
                if let Some(offset) = parse_hex(offset_str) {
                    do_read(client, offset);
                } else {
                    println!("Invalid offset: {}", offset_str);
                }
            } else {
                println!("Usage: r <offset>");
            }
        }
        "w" | "write" => {
            let offset_str = words.next();
            let value_str = words.next();

            match (offset_str, value_str) {
                (Some(o), Some(v)) => {
                    match (parse_hex(o), parse_hex(v)) {
                        (Some(offset), Some(value)) => do_write(client, offset, value),
                        (None, _) => println!("Invalid offset: {}", o),
                        (_, None) => println!("Invalid value: {}", v),
                    }
                }
                _ => println!("Usage: w <offset> <value>"),
            }
        }
        "rmw" => {
            // Read-modify-write: rmw <offset> <mask> <value>
            // Reads, ANDs with ~mask, ORs with value, writes back
            let offset_str = words.next();
            let mask_str = words.next();
            let value_str = words.next();

            match (offset_str, mask_str, value_str) {
                (Some(o), Some(m), Some(v)) => {
                    match (parse_hex(o), parse_hex(m), parse_hex(v)) {
                        (Some(offset), Some(mask), Some(value)) => {
                            match client.read_register(DEFAULT_PORT, DEFAULT_BUS, DEFAULT_DEVICE, DEFAULT_FUNCTION, offset) {
                                Ok(old) => {
                                    let new = (old & !mask) | (value & mask);
                                    println!("0x{:05x}: 0x{:08x} -> 0x{:08x}", offset, old, new);
                                    do_write(client, offset, new);
                                }
                                Err(e) => println!("ERROR: read failed: {:?}", e),
                            }
                        }
                        (None, _, _) => println!("Invalid offset: {}", o),
                        (_, None, _) => println!("Invalid mask: {}", m),
                        (_, _, None) => println!("Invalid value: {}", v),
                    }
                }
                _ => println!("Usage: rmw <offset> <mask> <value>"),
            }
        }
        "dump" => {
            // Dump a range: dump <start> <count>
            let start_str = words.next();
            let count_str = words.next().unwrap_or("16");

            if let Some(s) = start_str {
                if let (Some(start), Some(count)) = (parse_hex(s), parse_hex(count_str)) {
                    let count = count.min(256); // Limit to 256 registers
                    for i in 0..count {
                        let offset = start + i * 4;
                        match client.read_register(DEFAULT_PORT, DEFAULT_BUS, DEFAULT_DEVICE, DEFAULT_FUNCTION, offset) {
                            Ok(value) => {
                                print!("0x{:05x}=0x{:08x} ", offset, value);
                                if (i + 1) % 4 == 0 {
                                    println!();
                                }
                            }
                            Err(e) => {
                                println!("\nERROR at 0x{:05x}: {:?}", offset, e);
                                break;
                            }
                        }
                    }
                    if count % 4 != 0 {
                        println!();
                    }
                } else {
                    println!("Invalid parameter");
                }
            } else {
                println!("Usage: dump <start> [count]");
            }
        }
        "status" => {
            // Dump key DMA status registers
            println!("=== MT7996 DMA Status ===");
            let regs = [
                (0xd4100, "RST"),
                (0xd4208, "GLO_CFG"),
                (0xd42b0, "GLO_CFG_EXT0"),
                (0xd42b4, "GLO_CFG_EXT1"),
                (0xd413c, "BUSY_ENA"),
                (0xd4140, "BUSY_STAT"),
                (0xd7044, "HIF_MISC"),
                (0xd7030, "HOST_CONFIG"),
                (0xd7500, "AXI_R2A_CTRL"),
                (0x2108,  "MCU_INT_EVENT"),
            ];
            for (offset, name) in regs.iter() {
                match client.read_register(DEFAULT_PORT, DEFAULT_BUS, DEFAULT_DEVICE, DEFAULT_FUNCTION, *offset) {
                    Ok(value) => println!("  {:15} (0x{:05x}) = 0x{:08x}", name, offset, value),
                    Err(_) => println!("  {:15} (0x{:05x}) = ERROR", name, offset),
                }
            }
        }
        "fwdl" => {
            // Dump FWDL ring status
            println!("=== FWDL Ring (Q16) ===");
            let base = 0xd4400;
            let regs = [
                (base + 0x00, "DESC_BASE"),
                (base + 0x04, "RING_SIZE"),
                (base + 0x08, "CPU_IDX"),
                (base + 0x0c, "DMA_IDX"),
            ];
            for (offset, name) in regs.iter() {
                match client.read_register(DEFAULT_PORT, DEFAULT_BUS, DEFAULT_DEVICE, DEFAULT_FUNCTION, *offset) {
                    Ok(value) => println!("  {:15} (0x{:05x}) = 0x{:08x}", name, offset, value),
                    Err(_) => println!("  {:15} (0x{:05x}) = ERROR", name, offset),
                }
            }
        }
        "q" | "quit" | "exit" => {
            return false;
        }
        "help" | "?" => {
            println!("Commands:");
            println!("  r <offset>              - Read register");
            println!("  w <offset> <value>      - Write register");
            println!("  rmw <offset> <mask> <value> - Read-modify-write");
            println!("  dump <start> [count]    - Dump registers");
            println!("  status                  - Show key DMA status");
            println!("  fwdl                    - Show FWDL ring status");
            println!("  q                       - Quit");
        }
        _ => {
            println!("Unknown command: {}. Type 'help' for help.", cmd);
        }
    }

    true
}

/// Read a line with echo and basic line editing
fn read_line(stdin: &Stdin, buf: &mut [u8]) -> usize {
    let mut pos: usize = 0;

    while pos < buf.len() - 1 {
        if let Some(ch) = stdin.read_byte() {
            match ch {
                // Enter - end of line
                b'\r' | b'\n' => {
                    let _ = syscall::write(syscall::STDOUT, b"\r\n");
                    break;
                }
                // Backspace
                0x7F | 0x08 => {
                    if pos > 0 {
                        pos -= 1;
                        let _ = syscall::write(syscall::STDOUT, b"\x08 \x08");
                    }
                }
                // Ctrl+C - cancel line
                0x03 => {
                    let _ = syscall::write(syscall::STDOUT, b"^C\r\n");
                    return 0;
                }
                // Printable characters
                0x20..=0x7E => {
                    buf[pos] = ch;
                    pos += 1;
                    // Echo character
                    let _ = syscall::write(syscall::STDOUT, &[ch]);
                }
                // Ignore other control characters
                _ => {}
            }
        }
    }

    buf[pos] = 0;
    pos
}

#[unsafe(no_mangle)]
fn main() {
    uinfo!("pciepoke", "init");

    // Connect to pcied
    let mut client = match PcieClient::connect() {
        Ok(c) => c,
        Err(_) => {
            uerror!("pciepoke", "connect_failed");
            syscall::exit(1);
        }
    };

    // Interactive mode (command-line args not implemented yet for simplicity)
    println!("PCIe Register Poke Tool");
    println!("Device: port={} bus={} dev={} func={}",
             DEFAULT_PORT, DEFAULT_BUS, DEFAULT_DEVICE, DEFAULT_FUNCTION);
    println!("Type 'help' for commands, 'q' to quit");
    println!();

    let stdin = Stdin;
    let mut line_buf = [0u8; 128];

    loop {
        print!("> ");

        // Read a line
        let len = read_line(&stdin, &mut line_buf);
        if len == 0 {
            continue;
        }

        let line = match core::str::from_utf8(&line_buf[..len]) {
            Ok(s) => s,
            Err(_) => continue,
        };

        if !process_command(&mut client, line) {
            break;
        }
    }

    syscall::exit(0);
}
