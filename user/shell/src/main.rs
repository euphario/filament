//! Interactive shell for BPI-R4 kernel
//!
//! A minimal command-line shell that demonstrates syscall usage.

#![no_std]
#![no_main]
#![allow(dead_code)]  // Shell commands reserved for future use

mod builtins;
mod color;
mod completion;
mod console;
mod cwd;
mod input_box;
mod output;
mod readline;

// Shell-local print macros that route through console (consoled channel when connected)
// This avoids the output interleaving that happens when using userlib::print! directly
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let _ = write!(&mut $crate::console::ConsoleWriter, $($arg)*);
    }};
}

#[macro_export]
macro_rules! println {
    () => {{
        $crate::console::write(b"\r\n");
    }};
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let _ = write!(&mut $crate::console::ConsoleWriter, $($arg)*);
        $crate::console::write(b"\r\n");
    }};
}

use userlib::ipc::Port;
use userlib::syscall;
use userlib::vfs_client::VfsClient;

/// Maximum background jobs to track
const MAX_BG_JOBS: usize = 16;

/// Static line buffer (in .bss) to avoid stack allocation issues
static mut LINE_BUF: [u8; readline::MAX_LINE] = [0u8; readline::MAX_LINE];

/// Command history
static mut HISTORY: readline::History = readline::History::new();

/// Background job PIDs (0 = empty slot)
static mut BG_PIDS: [u32; MAX_BG_JOBS] = [0; MAX_BG_JOBS];

/// Retry counter for console connection
static mut CONSOLE_RETRY: u8 = 0;

/// Current working directory
static mut CWD: cwd::WorkingDir = cwd::WorkingDir::new();

/// Cached VFS client (discovered once, reused)
static mut VFS_CLIENT: Option<VfsClient> = None;

/// Command server port for remote shell IPC
static mut CMD_PORT: Option<Port> = None;

#[unsafe(no_mangle)]
fn main() {
    // Initialize console connection (falls back to direct UART if consoled not available)
    console::init();

    // Register shell command server port for remote shell access
    if let Ok(port) = Port::register(b"shell-cmd:") {
        let handle = port.handle();
        unsafe { CMD_PORT = Some(port); }
        // Register IPC poll callback so read_byte() services remote commands
        // even while blocked waiting for console input
        console::console_mut().set_ipc_poll(handle, drain_ipc_commands);
    }

    // Colored welcome banner
    color::set(color::BOLD);
    color::set(color::CYAN);
    console::write(b"BPI-R4 Shell");
    color::reset();
    console::write(b" v0.3\r\n");
    color::set(color::DIM);
    if console::console().is_connected() {
        console::write(b"Connected to consoled\r\n");
    } else {
        console::write(b"Direct UART mode (consoled not ready)\r\n");
    }
    console::write(b"Type 'help' for commands\r\n\r\n");
    color::reset();

    loop {
        // Drain any pending IPC command requests from remote shell
        drain_ipc_commands();

        // Try to connect to consoled if not connected yet (lazy connection)
        if !console::console().is_connected() {
            let retry = unsafe { &mut *core::ptr::addr_of_mut!(CONSOLE_RETRY) };
            *retry = retry.wrapping_add(1);
            // Retry every 10 prompts
            if *retry % 10 == 0 {
                if console::console_mut().connect() {
                    console::write(b"\r\n[Connected to consoled]\r\n");
                }
            }
        }

        // Draw prompt with cwd
        color::set(color::BOLD);
        color::set(color::BLUE);
        let cwd_str = unsafe {
            let cwd = &*core::ptr::addr_of!(CWD);
            cwd.as_bytes()
        };
        console::write(cwd_str);
        color::set(color::GREEN);
        console::write(b" > ");
        color::reset();

        // Read a line using readline with history
        let (buf_slice, history) = unsafe {
            (
                core::slice::from_raw_parts_mut(
                    core::ptr::addr_of_mut!(LINE_BUF) as *mut u8,
                    readline::MAX_LINE
                ),
                &mut *core::ptr::addr_of_mut!(HISTORY)
            )
        };

        let mut editor = readline::LineEditor::new(buf_slice, history);
        let len = editor.read();

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

/// Drain pending IPC command requests from remote shell clients.
/// Accepts connections on the "shell-cmd:" port, executes commands with
/// output capture, and sends the response back over the channel.
fn drain_ipc_commands() {
    let port = match unsafe { &mut *core::ptr::addr_of_mut!(CMD_PORT) } {
        Some(p) => p,
        None => return,
    };

    // Process up to 4 pending requests per drain cycle
    for _ in 0..4 {
        let mut channel = match port.try_accept() {
            Some(ch) => ch,
            None => break,
        };

        let mut cmd_buf = [0u8; 256];
        let n = match channel.recv(&mut cmd_buf) {
            Ok(n) => n,
            Err(_) => continue,
        };

        if n == 0 {
            continue;
        }

        let cmd = trim(&cmd_buf[..n]);
        if cmd.is_empty() {
            continue;
        }

        // Execute command with output captured
        let mut capture = console::CaptureBuffer::new();
        console::begin_capture(&mut capture);
        execute_command(cmd);
        console::end_capture();

        // Send response in chunks (576 byte IPC limit, use 512 for data)
        let data = capture.as_slice();
        let mut offset = 0;
        while offset < data.len() {
            let chunk_end = (offset + 512).min(data.len());
            let _ = channel.send(&data[offset..chunk_end]);
            offset = chunk_end;
        }
        // Channel drops here -> closes -> signals EOF to client
    }
}

/// Print a decimal number
fn print_dec(val: usize) {
    if val == 0 {
        console::write(b"0");
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
    console::write(&buf[i..]);
}

/// Trim whitespace from a byte slice
pub fn trim(input: &[u8]) -> &[u8] {
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
    } else if cmd_eq(cmd, b"pwd") {
        cmd_pwd();
    } else if cmd_eq(cmd, b"cd") {
        cmd_cd(b"");
    } else if cmd_starts_with(cmd, b"cd ") {
        cmd_cd(&cmd[3..]);
    } else if cmd_eq(cmd, b"ls") {
        builtins::ls::cmd_ls(b"").print();
    } else if cmd_starts_with(cmd, b"ls ") {
        builtins::ls::cmd_ls(&cmd[3..]).print();
    } else if cmd_starts_with(cmd, b"cat ") {
        builtins::cat::cmd_cat(&cmd[4..]);
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
    } else if cmd_eq(cmd, b"panic") {
        panic!("User requested panic");
    } else if cmd_eq(cmd, b"usb") {
        cmd_run_program("bin/usbd");
    } else if cmd_eq(cmd, b"gpio") {
        builtins::gpio::run(b"");
    } else if cmd_starts_with(cmd, b"gpio ") {
        builtins::gpio::run(&cmd[5..]);
    } else if cmd_eq(cmd, b"fatfs") {
        cmd_run_program("bin/fatfs");
    } else if cmd_eq(cmd, b"pcied") {
        cmd_run_program("bin/pcied");
    } else if cmd_eq(cmd, b"wifi") {
        cmd_run_program("bin/wifid");
    } else if cmd_eq(cmd, b"fan") {
        // Start PWM driver in background
        cmd_bg(b"bin/pwm");
    } else if cmd_starts_with(cmd, b"fan ") {
        cmd_fan(&cmd[4..]);
    } else if cmd_eq(cmd, b"ps") {
        builtins::ps::run(b"").print();
    } else if cmd_starts_with(cmd, b"kill ") {
        cmd_kill(&cmd[5..]);
    } else if cmd_starts_with(cmd, b"bg ") {
        cmd_bg(&cmd[3..]);
    } else if cmd_eq(cmd, b"jobs") {
        cmd_jobs();
    } else if cmd_starts_with(cmd, b"log ") {
        cmd_log(&cmd[4..]);
    } else if cmd_eq(cmd, b"klog") {
        cmd_klog();
    } else if cmd_eq(cmd, b"logs") {
        cmd_logs(b"");
    } else if cmd_starts_with(cmd, b"logs ") {
        cmd_logs(&cmd[5..]);
    } else if cmd_eq(cmd, b"reset") || cmd_eq(cmd, b"reboot") {
        cmd_reset();
    } else if cmd_eq(cmd, b"hw") {
        builtins::hw::run(b"");
    } else if cmd_starts_with(cmd, b"hw ") {
        builtins::hw::run(&cmd[3..]);
    } else if cmd_eq(cmd, b"devd") {
        builtins::devd::run(b"");
    } else if cmd_starts_with(cmd, b"devd ") {
        builtins::devd::run(&cmd[5..]);
    } else if cmd_eq(cmd, b"drivers") {
        builtins::drivers::run(b"").print();
    } else if cmd_starts_with(cmd, b"drivers ") {
        builtins::drivers::run(&cmd[8..]).print();
    } else if cmd_eq(cmd, b"dlog") {
        builtins::logs::run(b"").print();
    } else if cmd_starts_with(cmd, b"dlog ") {
        builtins::logs::run(&cmd[5..]).print();
    } else if cmd_eq(cmd, b"handle") {
        builtins::handle::run(b"", &mut output::ShellOutput::new());
    } else if cmd_starts_with(cmd, b"handle ") {
        builtins::handle::run(&cmd[7..], &mut output::ShellOutput::new());
    } else if cmd_eq(cmd, b"resize") {
        cmd_resize();
    } else if cmd_eq(cmd, b"lsdev") {
        builtins::lsdev::cmd_lsdev(b"");
    } else if cmd_starts_with(cmd, b"lsdev ") {
        builtins::lsdev::cmd_lsdev(&cmd[6..]);
    } else if cmd_starts_with(cmd, b"devinfo ") {
        builtins::lsdev::cmd_devinfo(&cmd[8..]);
    } else if cmd_starts_with(cmd, b"devquery ") {
        builtins::lsdev::cmd_devquery(&cmd[9..]);
    } else {
        // Try to run as a binary from bin/ directory
        try_run_binary(cmd);
    }
}

/// Try to run an unknown command as a binary from bin/
/// Uses a two-phase approach:
/// 1. Fast path: Try kernel ramfs exec (embedded binaries)
/// 2. Fallback: Load from filesystem via exec_mem (external binaries)
fn try_run_binary(cmd: &[u8]) {
    // Extract command name (first word)
    let cmd_name = if let Some(pos) = cmd.iter().position(|&c| c == b' ') {
        &cmd[..pos]
    } else {
        cmd
    };

    // Convert to str
    let name = match core::str::from_utf8(trim(cmd_name)) {
        Ok(s) => s,
        Err(_) => {
            println!("Invalid command");
            return;
        }
    };

    if name.is_empty() {
        return;
    }

    // Build path: "bin/<name>"
    let mut path_buf = [0u8; 64];
    let prefix = b"bin/";
    let name_bytes = name.as_bytes();

    if prefix.len() + name_bytes.len() >= path_buf.len() {
        println!("Command name too long");
        return;
    }

    path_buf[..prefix.len()].copy_from_slice(prefix);
    path_buf[prefix.len()..prefix.len() + name_bytes.len()].copy_from_slice(name_bytes);
    let path_len = prefix.len() + name_bytes.len();

    let path = match core::str::from_utf8(&path_buf[..path_len]) {
        Ok(s) => s,
        Err(_) => {
            println!("Invalid path");
            return;
        }
    };

    // Phase 1: Try kernel ramfs exec (fast path for embedded binaries)
    let result = syscall::exec(path);

    if result >= 0 {
        wait_for_child(result as u32);
        return;
    }

    // Note: Future support for loading binaries from external filesystems (USB, network)
    // would go here via vfsd's ReadToShmem protocol

    // Both paths failed - command not found
    color::set(color::RED);
    print!("Unknown command: ");
    print_bytes(cmd);
    color::reset();
    println!();
    color::set(color::DIM);
    println!("Type 'help' for available commands");
    color::reset();
}

/// Wait for a child process to complete
fn wait_for_child(pid: u32) {
    loop {
        let wait_result = syscall::wait(pid as i32);
        if wait_result >= 0 {
            let exit_code = (wait_result & 0xFFFFFFFF) as i32;
            if exit_code != 0 {
                println!("Process {} exited with code {}", pid, exit_code);
            }
            break;
        } else if wait_result == -11 {
            // EAGAIN - child still running, retry
            continue;
        } else {
            println!("Wait failed: {}", wait_result);
            break;
        }
    }
}

// Note: Loading binaries from external filesystems (USB, network) will require
// vfsd's ReadToShmem protocol. The kernel exec() already handles ramfs directly.

fn cmd_help() {
    // Build help text in a single buffer to avoid syscall storm
    // Each line is: "  \e[33m{cmd:14}\e[0m\e[2m - \e[0m{desc}\n"
    let mut buf = [0u8; 2048];
    let mut pos = 0;

    // Helper to append bytes
    let mut append = |s: &[u8]| {
        let len = s.len().min(buf.len() - pos);
        buf[pos..pos + len].copy_from_slice(&s[..len]);
        pos += len;
    };

    // Header
    append(color::BOLD);
    append(color::CYAN);
    append(b"Available commands:\n");
    append(color::RESET);

    // Commands - inline help_line logic
    let commands: &[(&str, &str)] = &[
        ("help, ?", "Show this help"),
        ("exit, quit", "Exit the shell"),
        ("pwd", "Print working directory"),
        ("cd [path]", "Change directory"),
        ("pid", "Show current process ID"),
        ("uptime", "Show system uptime"),
        ("mem", "Test memory allocation"),
        ("echo <msg>", "Echo a message"),
        ("ls [path]", "List directory (default: cwd)"),
        ("cat <path>", "Display file contents"),
        ("mkdir <path>", "Create directory"),
        ("rmdir <path>", "Remove empty directory"),
        ("rm <path>", "Remove file"),
        ("cp <src> <dst>", "Copy file"),
        ("mv <src> <dst>", "Move/rename file"),
        ("spawn <id>", "Spawn process by ELF ID"),
        ("usb", "Run USB userspace driver"),
        ("gpio [cmd]", "GPIO control (try 'gpio help')"),
        ("fatfs", "Run FAT filesystem driver"),
        ("pcied", "Start PCIe daemon"),
        ("wifi", "Run WiFi driver (MT7996)"),
        ("fan [0-100]", "Fan control / set speed"),
        ("hw [path]", "Hardware info (list/bus/tree/<path>)"),
        ("handle", "Test handle API (timer/channel/poll)"),
        ("devd spawn", "Spawn driver via devd (with caps)"),
        ("drivers", "Show driver/port tree (services, ports, shmem)"),
        ("lsdev [class]", "List registered devices"),
        ("devinfo <id>", "Get device info by ID"),
        ("devquery <id>", "Query driver (blockinfo/partition)"),
        ("yield", "Yield CPU to other processes"),
        ("ps", "Show running processes"),
        ("kill <pid>", "Terminate a process"),
        ("bg <path>", "Run program in background"),
        ("jobs", "Show background jobs"),
        ("log <level>", "Set log level (error/warn/info/debug/trace)"),
        ("logs [on|off|n]", "Control console log region"),
        ("resize", "Detect/display terminal size"),
        ("reset", "Reset the system"),
    ];

    for (cmd, desc) in commands {
        append(b"  ");
        append(color::YELLOW);
        // Pad command to 14 chars
        append(cmd.as_bytes());
        for _ in cmd.len()..14 {
            append(b" ");
        }
        append(color::RESET);
        append(color::DIM);
        append(b" - ");
        append(color::RESET);
        append(desc.as_bytes());
        append(b"\n");
    }

    // Single write syscall
    console::write(&buf[..pos]);
}

fn cmd_pid() {
    let pid = syscall::getpid();
    console::write(b"PID: ");
    print_dec(pid as usize);
    console::write(b"\r\n");
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

    console::write(b"Uptime: ");
    print_dec(seconds as usize);
    console::write(b".");
    // Print millis with leading zeros (3 digits)
    if millis < 100 { console::write(b"0"); }
    if millis < 10 { console::write(b"0"); }
    print_dec(millis as usize);
    console::write(b"s (");
    print_dec(cycles as usize);
    console::write(b" cycles)\r\n");
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
    // Parse path from argument
    let path = match core::str::from_utf8(trim(arg)) {
        Ok(s) => s,
        Err(_) => {
            println!("Invalid path");
            return;
        }
    };

    if path.is_empty() {
        println!("Usage: spawn <path>");
        return;
    }

    println!("Spawning {}...", path);
    let result = syscall::exec(path);

    if result >= 0 {
        let pid = result as u32;
        println!("Spawned process with PID {}", pid);

        // Wait for it
        println!("Waiting for process to exit...");
        wait_for_child(pid);
    } else {
        println!("spawn failed: {}", result);
    }
}

/// Print a byte slice to stdout
fn print_bytes(bytes: &[u8]) {
    console::write(bytes);
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
                // Success - child exited, unpack (pid << 32 | exit_code)
                let exit_code = (wait_result & 0xFFFFFFFF) as i32;
                println!("Process {} exited with code {}", pid, exit_code);
                break;
            } else if wait_result == -11 {
                // EAGAIN - child still running, just retry
                // Don't call yield_now() - it would undo the Blocked state
                continue;
            } else if wait_result == -10 {
                // ECHILD - no such child (probably daemonized)
                println!("Process {} detached (daemonized)", pid);
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

/// Run a program in the background (don't wait for completion)
fn cmd_run_program_bg(path: &str) {
    println!("Starting {} in background...", path);
    let result = syscall::exec(path);

    if result >= 0 {
        let pid = result as u32;
        println!("[{}] Started background daemon PID {}", track_bg_job(pid), pid);
    } else {
        println!("exec failed: {}", result);
    }
}

/// Parse decimal number from byte slice
pub fn parse_decimal(input: &[u8]) -> Option<u32> {
    let trimmed = trim(input);
    if trimmed.is_empty() {
        return None;
    }

    let mut value: u32 = 0;
    for &ch in trimmed {
        if ch >= b'0' && ch <= b'9' {
            value = value.saturating_mul(10).saturating_add((ch - b'0') as u32);
        } else {
            return None;
        }
    }
    Some(value)
}


/// Kill a process by PID
fn cmd_kill(arg: &[u8]) {
    match parse_decimal(arg) {
        Some(pid) => {
            let result = syscall::kill(pid);
            if result == 0 {
                println!("Killed process {}", pid);
            } else {
                println!("Failed to kill {}: error {}", pid, result);
            }
        }
        None => {
            println!("Usage: kill <pid>");
        }
    }
}

/// Run a program in background
fn cmd_bg(arg: &[u8]) {
    // Convert bytes to str for exec
    let path = match core::str::from_utf8(trim(arg)) {
        Ok(s) => s,
        Err(_) => {
            println!("Invalid path");
            return;
        }
    };

    if path.is_empty() {
        println!("Usage: bg <path>");
        return;
    }

    let result = syscall::exec(path);

    if result >= 0 {
        let pid = result as u32;
        println!("[{}] Started background process PID {}", track_bg_job(pid), pid);
    } else {
        println!("exec failed: {}", result);
    }
}

/// Track a background job, returns job number (1-based)
fn track_bg_job(pid: u32) -> usize {
    unsafe {
        for i in 0..MAX_BG_JOBS {
            if BG_PIDS[i] == 0 {
                BG_PIDS[i] = pid;
                return i + 1;
            }
        }
    }
    0  // No slot available
}

/// Show background jobs
fn cmd_jobs() {
    let mut buf: [syscall::ProcessInfo; 16] = [syscall::ProcessInfo::empty(); 16];
    let count = syscall::ps_info(&mut buf);

    println!("JOB  PID   STATE      NAME");
    println!("---  ----  ---------  ---------------");

    let mut found = false;

    unsafe {
        for i in 0..MAX_BG_JOBS {
            let pid = BG_PIDS[i];
            if pid == 0 {
                continue;
            }

            // Look up process info
            let mut still_exists = false;
            for j in 0..count {
                if buf[j].pid == pid {
                    print!("{:3}  {:4}  {:9}  ", i + 1, pid, buf[j].state_str());
                    print_bytes(&buf[j].name);
                    println!();
                    still_exists = true;
                    found = true;

                    // If terminated (Exiting/Dying/Dead), clean up the slot
                    if buf[j].state >= 4 {  // 4=Exiting, 5=Dying, 6=Dead
                        BG_PIDS[i] = 0;
                    }
                    break;
                }
            }

            // Process no longer exists - clean up
            if !still_exists {
                BG_PIDS[i] = 0;
            }
        }
    }

    if !found {
        println!("No background jobs");
    }
}

/// Set fan speed via PWM driver IPC
fn cmd_fan(arg: &[u8]) {
    use userlib::ipc::Channel;

    let arg = trim(arg);

    // Parse percentage (0-100)
    let mut percent: u32 = 0;
    for &c in arg {
        if c >= b'0' && c <= b'9' {
            percent = percent * 10 + (c - b'0') as u32;
        } else {
            println!("Invalid fan speed. Use: fan <0-100>");
            return;
        }
    }

    if percent > 100 {
        println!("Fan speed must be 0-100");
        return;
    }

    // Connect to PWM driver
    let mut channel = match Channel::connect(b"pwm:") {
        Ok(ch) => ch,
        Err(_) => {
            println!("Failed to connect to PWM driver (not running?)");
            println!("Start it with: fan");
            return;
        }
    };

    // Send set fan speed command: [1, percent]
    let cmd = [1u8, percent as u8];
    if channel.send(&cmd).is_err() {
        println!("Failed to send command");
        return;
    }

    // Receive response
    let mut resp = [0u8; 1];
    match channel.recv(&mut resp) {
        Ok(n) if n > 0 => {
            println!("Fan speed set to {}%", resp[0]);
        }
        _ => {
            println!("No response from PWM driver");
        }
    }
    // Channel is dropped automatically
}

/// Set kernel log level
/// Dump all pending records from the kernel log ring
fn cmd_klog() {
    let mut buf = [0u8; 1024];
    let mut count = 0u32;
    loop {
        let n = syscall::klog_read(&mut buf);
        if n <= 0 {
            break;
        }
        print_bytes(&buf[..n as usize]);
        // Add newline if record doesn't end with one
        if n > 0 && buf[(n - 1) as usize] != b'\n' {
            println!();
        }
        count += 1;
    }
    if count == 0 {
        println!("(kernel log ring empty)");
    }
}

fn cmd_log(arg: &[u8]) {
    let level_str = trim(arg);

    let level = if cmd_eq(level_str, b"error") {
        syscall::log_level::ERROR
    } else if cmd_eq(level_str, b"warn") {
        syscall::log_level::WARN
    } else if cmd_eq(level_str, b"info") {
        syscall::log_level::INFO
    } else if cmd_eq(level_str, b"debug") {
        syscall::log_level::DEBUG
    } else if cmd_eq(level_str, b"trace") {
        syscall::log_level::TRACE
    } else {
        println!("Unknown log level. Use: error, warn, info, debug, trace");
        return;
    };

    let result = syscall::set_log_level(level);
    if result == 0 {
        print!("Log level set to ");
        print_bytes(level_str);
        println!();
    } else {
        println!("Failed to set log level: {}", result);
    }
}

/// Control console log region split
fn cmd_logs(arg: &[u8]) {
    let arg = trim(arg);

    // Check if connected to consoled
    if !console::console().is_connected() {
        println!("Not connected to consoled (using direct UART)");
        return;
    }

    if arg.is_empty() {
        // Toggle log split
        let current = console::console().log_split;
        console::console_mut().set_log_split(!current);
        if !current {
            println!("Log split enabled");
        } else {
            println!("Log split disabled (full screen for shell)");
        }
    } else if cmd_eq(arg, b"on") {
        console::console_mut().set_log_split(true);
        println!("Log split enabled");
    } else if cmd_eq(arg, b"off") {
        console::console_mut().set_log_split(false);
        println!("Log split disabled");
    } else if cmd_eq(arg, b"disconnect") {
        // Disconnect from logd completely (stops receiving logs)
        console::console().set_logd_connected(false);
        println!("Disconnected from logd (no new logs will appear)");
    } else if cmd_eq(arg, b"connect") || cmd_eq(arg, b"reconnect") {
        // Reconnect to logd
        console::console().set_logd_connected(true);
        println!("Reconnecting to logd...");
    } else if let Some(n) = parse_decimal(arg) {
        // Set number of log lines
        if n > 255 {
            println!("Log lines must be 0-255 (0 = auto)");
            return;
        }
        console::console_mut().set_log_lines(n as u8);
        if n == 0 {
            println!("Log lines set to auto");
        } else {
            println!("Log lines set to {}", n);
        }
    } else {
        println!("Usage: logs [on|off|disconnect|connect|<lines>]");
        println!("  logs            - Toggle log split");
        println!("  logs on         - Enable log region");
        println!("  logs off        - Disable log region (full screen for shell)");
        println!("  logs disconnect - Disconnect from logd (stop receiving logs)");
        println!("  logs connect    - Reconnect to logd");
        println!("  logs <n>        - Set log region to n lines (0 = auto)");
    }
}

/// Reset the system
fn cmd_reset() {
    println!("Resetting system...");
    let result = syscall::reset();
    if result < 0 {
        println!("Reset failed: error {}", result);
    }
}

/// Query and display terminal size
fn cmd_resize() {
    let con = console::console_mut();

    if !con.is_connected() {
        println!("Not connected to consoled (using direct UART)");
        println!("Current: {}x{}", con.cols, con.rows);
        return;
    }

    // Query consoled for current size (triggers re-detection)
    match con.query_size() {
        Some((cols, rows)) => {
            println!("Terminal size: {}x{}", cols, rows);
        }
        None => {
            println!("Failed to query terminal size");
            println!("Current: {}x{}", con.cols, con.rows);
        }
    }
}

/// Print working directory
fn cmd_pwd() {
    let path = unsafe {
        let cwd = &*core::ptr::addr_of!(CWD);
        cwd.as_str()
    };
    match path {
        Some(p) => println!("{}", p),
        None => println!("/"),
    }
}

/// Change directory
fn cmd_cd(path_arg: &[u8]) {
    use userlib::vfs_proto::open_flags;

    let path = trim(path_arg);

    // "cd" with no args goes to root — always valid
    if path.is_empty() {
        unsafe {
            let cwd = &mut *core::ptr::addr_of_mut!(CWD);
            cwd.cd(path);
        }
        return;
    }

    // Resolve the target path relative to cwd
    let mut resolved_buf = [0u8; cwd::MAX_PATH];
    let resolved = unsafe {
        let cwd = &*core::ptr::addr_of!(CWD);
        cwd.resolve(path, &mut resolved_buf)
    };
    let resolved = match resolved {
        Some(p) => p,
        None => {
            println!("cd: invalid path");
            return;
        }
    };

    // "/" is always valid (no mount needed)
    if resolved != b"/" {
        // Verify the directory exists via VFS
        if let Some(client) = get_vfs_client() {
            match client.open(resolved, open_flags::DIR | open_flags::RDONLY) {
                Ok(handle) => {
                    client.close(handle);
                }
                Err(e) => {
                    builtins::ls::print_vfs_error(b"cd", resolved, e);
                    return;
                }
            }
        }
        // If vfsd isn't available, allow cd anyway (path-only mode)
    }

    let success = unsafe {
        let cwd = &mut *core::ptr::addr_of_mut!(CWD);
        cwd.cd(path)
    };

    if !success {
        println!("cd: invalid path");
    }
}

/// Get current working directory (for use by builtins)
pub fn get_cwd() -> &'static cwd::WorkingDir {
    unsafe { &*core::ptr::addr_of!(CWD) }
}

/// Get cached VFS client (discovers lazily, retries on failure)
pub fn get_vfs_client() -> Option<&'static mut VfsClient> {
    unsafe {
        let client = &mut *core::ptr::addr_of_mut!(VFS_CLIENT);
        if client.is_some() {
            return client.as_mut();
        }
        // Always retry discovery — vfsd may not have started yet
        *client = VfsClient::discover().ok();
        client.as_mut()
    }
}

