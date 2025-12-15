//! Interactive shell for BPI-R4 kernel
//!
//! A minimal command-line shell that demonstrates syscall usage.

#![no_std]
#![no_main]

use userlib::{println, print, syscall, Stdin};

/// Maximum command line length
const MAX_LINE: usize = 128;

/// Static line buffer (in .bss) to avoid stack allocation issues
static mut LINE_BUF: [u8; MAX_LINE] = [0u8; MAX_LINE];

#[unsafe(no_mangle)]
fn main() {
    let _ = syscall::write(syscall::STDOUT, b"BPI-R4 Shell v0.1\r\n");
    let _ = syscall::write(syscall::STDOUT, b"Type 'help' for commands\r\n\r\n");

    let stdin = Stdin;

    loop {
        let _ = syscall::write(syscall::STDOUT, b"> ");

        // Read a line using static buffer
        let buf_slice = unsafe {
            core::slice::from_raw_parts_mut(
                core::ptr::addr_of_mut!(LINE_BUF) as *mut u8,
                MAX_LINE
            )
        };
        let len = read_line(&stdin, buf_slice);

        if len == 0 {
            continue;
        }

        let cmd = trim(&buf_slice[..len]);

        if cmd.is_empty() {
            continue;
        }

        execute_command(cmd);
    }
}

/// Print a decimal number
fn print_dec(val: usize) {
    if val == 0 {
        let _ = syscall::write(syscall::STDOUT, b"0");
        return;
    }
    let mut buf = [0u8; 10];
    let mut n = val;
    let mut i = 10;
    while n > 0 && i > 0 {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    let _ = syscall::write(syscall::STDOUT, &buf[i..]);
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
                // Ctrl+D - EOF (exit if empty line)
                0x04 => {
                    if pos == 0 {
                        let _ = syscall::write(syscall::STDOUT, b"\r\n");
                        syscall::exit(0);
                    }
                }
                // Printable characters
                0x20..=0x7E => {
                    buf[pos] = ch;
                    pos += 1;
                    // Echo character
                    let _ = syscall::write(syscall::STDOUT, &[ch]);
                }
                _ => {
                    // Ignore other control characters
                }
            }
        }
    }

    pos
}

/// Trim whitespace from a byte slice
fn trim(input: &[u8]) -> &[u8] {
    let start = input.iter().position(|&c| c != b' ' && c != b'\t' && c != b'\r' && c != b'\n');
    let end = input.iter().rposition(|&c| c != b' ' && c != b'\t' && c != b'\r' && c != b'\n');

    match (start, end) {
        (Some(s), Some(e)) => &input[s..=e],
        _ => &[],
    }
}

/// Check if command matches (case-insensitive for ASCII)
fn cmd_eq(cmd: &[u8], expected: &[u8]) -> bool {
    if cmd.len() != expected.len() {
        return false;
    }
    for (a, b) in cmd.iter().zip(expected.iter()) {
        let a_lower = if *a >= b'A' && *a <= b'Z' { a + 32 } else { *a };
        let b_lower = if *b >= b'A' && *b <= b'Z' { b + 32 } else { *b };
        if a_lower != b_lower {
            return false;
        }
    }
    true
}

/// Check if command starts with prefix
fn cmd_starts_with(cmd: &[u8], prefix: &[u8]) -> bool {
    if cmd.len() < prefix.len() {
        return false;
    }
    cmd_eq(&cmd[..prefix.len()], prefix)
}

/// Execute a command
fn execute_command(cmd: &[u8]) {
    // Built-in commands
    if cmd_eq(cmd, b"help") || cmd_eq(cmd, b"?") {
        cmd_help();
    } else if cmd_eq(cmd, b"exit") || cmd_eq(cmd, b"quit") || cmd_eq(cmd, b"q") {
        println!("Goodbye!");
        syscall::exit(0);
    } else if cmd_eq(cmd, b"pid") {
        cmd_pid();
    } else if cmd_eq(cmd, b"uptime") {
        cmd_uptime();
    } else if cmd_eq(cmd, b"mem") {
        cmd_mem();
    } else if cmd_starts_with(cmd, b"echo ") {
        cmd_echo(&cmd[5..]);
    } else if cmd_eq(cmd, b"echo") {
        println!();
    } else if cmd_starts_with(cmd, b"spawn ") {
        cmd_spawn(&cmd[6..]);
    } else if cmd_eq(cmd, b"yield") {
        syscall::yield_now();
        println!("Yielded CPU");
    } else if cmd_eq(cmd, b"panic") {
        panic!("User requested panic");
    } else if cmd_eq(cmd, b"usb") {
        cmd_run_program("bin/usbtest");
    } else {
        print!("Unknown command: ");
        print_bytes(cmd);
        println!();
        println!("Type 'help' for available commands");
    }
}

fn cmd_help() {
    println!("Available commands:");
    println!("  help, ?     - Show this help");
    println!("  exit, quit  - Exit the shell");
    println!("  pid         - Show current process ID");
    println!("  uptime      - Show system uptime (ticks)");
    println!("  mem         - Test memory allocation");
    println!("  echo <msg>  - Echo a message");
    println!("  spawn <id>  - Spawn process by ELF ID");
    println!("  usb         - Run USB userspace driver test");
    println!("  yield       - Yield CPU to other processes");
    println!("  panic       - Trigger a panic (test)");
}

fn cmd_pid() {
    let pid = syscall::getpid();
    let _ = syscall::write(syscall::STDOUT, b"PID: ");
    print_dec(pid as usize);
    let _ = syscall::write(syscall::STDOUT, b"\r\n");
}

fn cmd_uptime() {
    // Read cycle counter (available in EL0 if enabled)
    let cycles: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntvct_el0", out(reg) cycles);
    }
    // Assuming 24MHz timer (typical for ARM)
    let seconds = cycles / 24_000_000;
    let millis = (cycles % 24_000_000) / 24_000;

    let _ = syscall::write(syscall::STDOUT, b"Uptime: ");
    print_dec(seconds as usize);
    let _ = syscall::write(syscall::STDOUT, b".");
    // Print millis with leading zeros (3 digits)
    if millis < 100 { let _ = syscall::write(syscall::STDOUT, b"0"); }
    if millis < 10 { let _ = syscall::write(syscall::STDOUT, b"0"); }
    print_dec(millis as usize);
    let _ = syscall::write(syscall::STDOUT, b"s (");
    print_dec(cycles as usize);
    let _ = syscall::write(syscall::STDOUT, b" cycles)\r\n");
}

fn cmd_mem() {
    println!("Testing memory allocation...");

    // Try to allocate a page
    let addr = syscall::mmap(0, 4096, syscall::PROT_READ | syscall::PROT_WRITE);

    if addr < 0 {
        println!("mmap failed: {}", addr);
        return;
    }

    println!("Allocated page at 0x{:x}", addr as u64);

    // Write and read back
    let ptr = addr as *mut u8;
    unsafe {
        ptr.write_volatile(0x42);
        let val = ptr.read_volatile();
        if val == 0x42 {
            println!("Write/read test: OK");
        } else {
            println!("Write/read test: FAILED (got 0x{:x})", val);
        }
    }

    // Free the page
    let ret = syscall::munmap(addr as u64, 4096);
    if ret == 0 {
        println!("Page freed");
    } else {
        println!("munmap failed: {}", ret);
    }
}

fn cmd_echo(msg: &[u8]) {
    print_bytes(msg);
    println!();
}

fn cmd_spawn(arg: &[u8]) {
    // Parse ELF ID (simple decimal parser)
    let mut id: u32 = 0;
    for &ch in arg {
        if ch >= b'0' && ch <= b'9' {
            id = id * 10 + (ch - b'0') as u32;
        } else if ch == b' ' || ch == b'\t' {
            continue;
        } else {
            println!("Invalid ELF ID");
            return;
        }
    }

    println!("Spawning ELF ID {}...", id);
    let result = syscall::spawn(id);

    if result >= 0 {
        let pid = result as u32;
        println!("Spawned process with PID {}", pid);

        // Wait for it
        println!("Waiting for process to exit...");
        let wait_result = syscall::wait(pid as i32);

        if wait_result >= 0 {
            let exit_code = (wait_result & 0xFFFFFFFF) as i32;
            println!("Process exited with code {}", exit_code);
        } else {
            println!("wait failed: {}", wait_result);
        }
    } else {
        println!("spawn failed: {}", result);
    }
}

/// Print a byte slice to stdout
fn print_bytes(bytes: &[u8]) {
    let _ = syscall::write(syscall::STDOUT, bytes);
}

/// Run a program from ramfs by path
fn cmd_run_program(path: &str) {
    println!("Running {}...", path);
    let result = syscall::exec(path);

    if result >= 0 {
        let pid = result as u32;
        println!("Spawned process with PID {}", pid);

        // Wait for child to complete
        // Loop until child exits - kernel handles blocking
        loop {
            let wait_result = syscall::wait(pid as i32);

            if wait_result >= 0 {
                // Success - child exited
                let exit_code = (wait_result & 0xFF) as i32;
                println!("Process {} exited with code {}", pid, exit_code);
                break;
            } else if wait_result == -11 {
                // EAGAIN - child still running, just retry
                // Don't call yield_now() - it would undo the Blocked state
                continue;
            } else if wait_result == -10 {
                // ECHILD - no such child
                println!("Child {} no longer exists", pid);
                break;
            } else {
                println!("wait failed: {}", wait_result);
                break;
            }
        }
    } else {
        println!("exec failed: {}", result);
    }
}
