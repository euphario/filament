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

use userlib::ipc::{Port, Process, Mux, MuxFilter};
use userlib::syscall;
use userlib::vfs_client::VfsClient;

// Re-export libf utilities so builtins can use crate::trim, crate::cmd_eq, etc.
pub use libf::str::trim;
pub use libf::str::eq_ignore_ascii_case as cmd_eq;
pub use libf::str::starts_with_ignore_case as cmd_starts_with;
pub use libf::parse::parse_u32 as parse_decimal;

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

        // Build prompt: bold-blue cwd + green " > " + reset
        let cwd_str = unsafe {
            let cwd = &*core::ptr::addr_of!(CWD);
            cwd.as_bytes()
        };

        let use_input_mode = console::console().is_connected();
        if !use_input_mode {
            // Legacy mode: write prompt directly
            color::set(color::BOLD);
            color::set(color::BLUE);
            console::write(cwd_str);
            color::set(color::GREEN);
            console::write(b" > ");
            color::reset();
        }

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

        // Build colored prompt for input mode
        if use_input_mode {
            let mut pbuf = [0u8; 48];
            let mut p = 0;
            // Bold blue
            pbuf[p..p+4].copy_from_slice(b"\x1b[1m");
            p += 4;
            pbuf[p..p+5].copy_from_slice(b"\x1b[34m");
            p += 5;
            // CWD (truncate if needed)
            let clen = cwd_str.len().min(48 - p - 12); // reserve for " > " + color codes
            pbuf[p..p+clen].copy_from_slice(&cwd_str[..clen]);
            p += clen;
            // Green
            pbuf[p..p+5].copy_from_slice(b"\x1b[32m");
            p += 5;
            // " > "
            pbuf[p..p+3].copy_from_slice(b" > ");
            p += 3;
            // Reset
            pbuf[p..p+4].copy_from_slice(b"\x1b[0m");
            p += 4;

            let visible_width = (clen + 3) as u8; // cwd + " > "
            editor.set_prompt(&pbuf[..p], visible_width);
        }

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
    let s = libf::fmt::StackStr::from_u64(val as u64);
    console::write(s.as_bytes());
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
    } else if cmd_eq(cmd, b"clear") || cmd_eq(cmd, b"cls") {
        cmd_clear();
    } else if cmd_eq(cmd, b"uname") {
        cmd_uname();
    } else if cmd_eq(cmd, b"mem") || cmd_eq(cmd, b"free") {
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
    } else if cmd_starts_with(cmd, b"ps ") {
        builtins::ps::run(&cmd[3..]).print();
    } else if cmd_starts_with(cmd, b"kill ") {
        cmd_kill(&cmd[5..]);
    } else if cmd_starts_with(cmd, b"signal ") {
        cmd_signal(&cmd[7..]);
    } else if cmd_starts_with(cmd, b"bg ") {
        cmd_bg(&cmd[3..]);
    } else if cmd_eq(cmd, b"jobs") {
        cmd_jobs();
    } else if cmd_starts_with(cmd, b"log ") {
        cmd_log(&cmd[4..]);
    } else if cmd_eq(cmd, b"dmesg") {
        builtins::dmesg::run(b"");
    } else if cmd_starts_with(cmd, b"dmesg ") {
        builtins::dmesg::run(&cmd[6..]);
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
    } else if cmd_eq(cmd, b"devc") {
        builtins::devc::run(b"").print();
    } else if cmd_starts_with(cmd, b"devc ") {
        builtins::devc::run(&cmd[5..]).print();
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
    } else if cmd_eq(cmd, b"mount") {
        builtins::mount::run(b"").print();
    } else if cmd_starts_with(cmd, b"mount ") {
        builtins::mount::run(&cmd[6..]).print();
    } else if cmd_eq(cmd, b"resize") {
        cmd_resize();
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

/// Wait for a child process to complete, interruptible by signals (Ctrl+C)
fn wait_for_child(pid: u32) {
    // Try Mux-based wait (supports signal delivery for Ctrl+C)
    let proc = match Process::watch(pid) {
        Ok(p) => p,
        Err(_) => {
            // Fallback: blocking wait (no signal support)
            loop {
                let r = syscall::wait(pid as i32);
                if r >= 0 {
                    let code = (r & 0xFFFFFFFF) as i32;
                    if code != 0 { println!("Process {} exited with code {}", pid, code); }
                    break;
                } else if r != -11 { break; }
            }
            return;
        }
    };

    let mux = match Mux::new() {
        Ok(m) => m,
        Err(_) => return,
    };
    let _ = mux.add(proc.handle(), MuxFilter::Readable);

    loop {
        match mux.wait() {
            Ok(event) => {
                if event.is_signal() {
                    if event.signal_event as u32 & syscall::signal_event::INTERRUPT != 0 {
                        // Ctrl+C: kill the foreground child
                        syscall::kill(pid);
                        continue; // Wait for actual exit
                    }
                }
                if event.is_readable() || event.is_closed() {
                    // Child exited
                    let r = syscall::wait(pid as i32);
                    if r >= 0 {
                        let code = (r & 0xFFFFFFFF) as i32;
                        if code != 0 {
                            println!("Process {} exited with code {}", pid, code);
                        }
                    }
                    break;
                }
            }
            Err(_) => break,
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
        ("clear", "Clear the screen"),
        ("uname", "Show system info"),
        ("free", "Show memory & system stats"),
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
        ("yield", "Yield CPU to other processes"),
        ("ps [-v|-vv|-vvv]", "Show processes (verbose: handles, mem, caps)"),
        ("kill [-9] <pid>", "Signal (SHUTDOWN) or force-kill a process"),
        ("signal <pid> <evt> [val]", "Send signal (evt: 1=exit 2=int 3=shut 4=port)"),
        ("bg <path>", "Run program in background"),
        ("jobs", "Show background jobs"),
        ("dmesg [-l] [-m]", "Dump kernel log ring (non-destructive)"),
        ("log <level>", "Set log level (error/warn/info/notice/debug/trace)"),
        ("logs [on|off|n]", "Control console log region"),
        ("resize", "Detect/display terminal size"),
        ("reset, reboot", "Graceful shutdown + reset"),
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
    let ns = syscall::gettime();
    let total_secs = ns / 1_000_000_000;
    let hours = total_secs / 3600;
    let mins = (total_secs % 3600) / 60;
    let secs = total_secs % 60;
    let millis = (ns % 1_000_000_000) / 1_000_000;

    print!("Uptime: {}h {}m {}.{:03}s", hours, mins, secs, millis);
    println!();
}

fn cmd_clear() {
    // ANSI: clear screen + cursor home
    console::write(b"\x1b[2J\x1b[H");
}

fn cmd_uname() {
    #[cfg(feature = "platform-mt7988a")]
    const PLATFORM: &str = "MT7988A";
    #[cfg(feature = "platform-qemu-virt")]
    const PLATFORM: &str = "qemu-virt";
    #[cfg(not(any(feature = "platform-mt7988a", feature = "platform-qemu-virt")))]
    const PLATFORM: &str = "unknown";

    println!("Filament 0.1.0 aarch64 {}", PLATFORM);
}

fn cmd_mem() {
    let mut info = syscall::SysInfo::empty();
    let ret = syscall::sysinfo(&mut info);
    if ret < 0 {
        println!("sysinfo failed: {}", ret);
        return;
    }

    let used = info.total_pages.saturating_sub(info.free_pages);
    let total_mb = (info.total_pages as u64 * 4096) / (1024 * 1024);
    let free_mb = (info.free_pages as u64 * 4096) / (1024 * 1024);
    let used_mb = (used as u64 * 4096) / (1024 * 1024);

    println!("Memory:");
    println!("  Total: {} pages ({} MB)", info.total_pages, total_mb);
    println!("  Free:  {} pages ({} MB)", info.free_pages, free_mb);
    println!("  Used:  {} pages ({} MB)", used, used_mb);

    let total_secs = info.uptime_ns / 1_000_000_000;
    let hours = total_secs / 3600;
    let mins = (total_secs % 3600) / 60;
    let secs = total_secs % 60;
    println!("Tasks: {} / CPUs: {}", info.num_tasks, info.num_cpus);
    println!("Uptime: {}h {}m {}s", hours, mins, secs);
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
        wait_for_child(pid);
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


/// Kill a process by PID
/// `kill <pid>` — send SHUTDOWN signal (graceful, like SIGTERM)
/// `kill -9 <pid>` — hard kill (like SIGKILL)
fn cmd_kill(arg: &[u8]) {
    let arg = trim(arg);
    let (force, pid_arg) = if arg.starts_with(b"-9 ") {
        (true, trim(&arg[3..]))
    } else if arg.starts_with(b"-9") && arg.len() > 2 {
        (true, trim(&arg[2..]))
    } else {
        (false, arg)
    };

    match parse_decimal(pid_arg) {
        Some(pid) => {
            if force {
                let result = syscall::kill(pid);
                if result == 0 {
                    println!("Killed process {}", pid);
                } else {
                    println!("Failed to kill {}: error {}", pid, result);
                }
            } else {
                // Graceful: send SHUTDOWN signal
                let sig_result = syscall::signal(pid, syscall::signal_event::SHUTDOWN, 0);
                if sig_result == 0 {
                    println!("Sent SHUTDOWN to {}", pid);
                } else {
                    println!("Signal failed (error {}), use kill -9 {}", sig_result, pid);
                }
            }
        }
        None => {
            println!("Usage: kill [-9] <pid>");
        }
    }
}

/// Signal event name for display (bitmask — may have multiple bits set)
fn signal_event_name(event: u32) -> &'static str {
    // Return first matching bit name (for display)
    if event & syscall::signal_event::CHILD_EXIT != 0 { return "CHILD_EXIT"; }
    if event & syscall::signal_event::SHUTDOWN != 0 { return "SHUTDOWN"; }
    if event & syscall::signal_event::INTERRUPT != 0 { return "INTERRUPT"; }
    if event & syscall::signal_event::READY != 0 { return "READY"; }
    if event & syscall::signal_event::RESIZE != 0 { return "RESIZE"; }
    if event & syscall::signal_event::PORT_CHANGED != 0 { return "PORT_CHANGED"; }
    if event & syscall::signal_event::HARDWARE != 0 { return "HARDWARE"; }
    if event & syscall::signal_event::HEARTBEAT != 0 { return "HEARTBEAT"; }
    if event & syscall::signal_event::EVICT != 0 { return "EVICT"; }
    ""
}

/// Signal command: send, peek, or flush
///   signal <pid> <event> [value]  — send
///   signal peek <pid>             — show pending signals
///   signal flush <pid>            — clear pending signals
fn cmd_signal(arg: &[u8]) {
    let arg = trim(arg);
    let (first, rest) = libf::str::split_once(arg, b' ');
    let rest = trim(rest);

    // Check for subcommands
    if libf::str::eq_ignore_ascii_case(first, b"peek") {
        let pid = match parse_decimal(rest) {
            Some(p) => p,
            None => {
                println!("Usage: signal peek <pid>");
                return;
            }
        };
        let mut buf = [syscall::PendingSignal { event: 0, value: 0 }; 8];
        let result = syscall::signal_peek(pid, &mut buf);
        if result < 0 {
            println!("Failed: error {}", result);
            return;
        }
        let count = result as usize;
        if count == 0 {
            println!("No pending signals for pid {}", pid);
        } else {
            println!("Pending signals for pid {}:", pid);
            for i in 0..count {
                let name = signal_event_name(buf[i].event);
                if name.is_empty() {
                    println!("  [{}] event={} value={}", i, buf[i].event, buf[i].value);
                } else {
                    println!("  [{}] {} value={}", i, name, buf[i].value);
                }
            }
            println!("({} pending)", count);
        }
        return;
    }

    if libf::str::eq_ignore_ascii_case(first, b"flush") {
        let pid = match parse_decimal(rest) {
            Some(p) => p,
            None => {
                println!("Usage: signal flush <pid>");
                return;
            }
        };
        let result = syscall::signal_flush(pid);
        if result < 0 {
            println!("Failed: error {}", result);
        } else {
            println!("Flushed {} signals from pid {}", result, pid);
        }
        return;
    }

    // Original send path: signal <pid> <event> [value]
    let pid_str = first;
    let (evt_str, val_str) = libf::str::split_once(rest, b' ');
    let val_str = trim(val_str);

    let pid = match parse_decimal(pid_str) {
        Some(p) => p,
        None => {
            println!("Usage: signal <pid> <event> [value]");
            println!("       signal peek <pid>");
            println!("       signal flush <pid>");
            println!("  Events: 1=CHILD_EXIT 2=SHUTDOWN 4=INTERRUPT 8=READY 64=PORT_CHANGED");
            return;
        }
    };
    let event = match parse_decimal(evt_str) {
        Some(e) => e,
        None => {
            println!("Usage: signal <pid> <event> [value]");
            return;
        }
    };
    let value = if val_str.is_empty() { 0u64 } else {
        match libf::parse::parse_u64(val_str) {
            Some(v) => v,
            None => {
                println!("Invalid value");
                return;
            }
        }
    };

    let result = syscall::signal(pid, event, value);
    if result == 0 {
        println!("Signal {} sent to pid {}", event, pid);
    } else {
        println!("Failed: error {}", result);
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
    let arg = trim(arg);

    // Parse subcommands: "log console <level>", "log module <subsys> <level>"
    let (first, rest) = libf::str::split_once(arg, b' ');
    let rest = trim(rest);

    if cmd_eq(first, b"console") && !rest.is_empty() {
        // Set console output level
        let level = match parse_log_level(rest) {
            Some(l) => l,
            None => {
                println!("Unknown level. Use: error, warn, info, notice, debug, trace");
                return;
            }
        };
        let result = syscall::set_console_level(level);
        if result == 0 {
            print!("Console level set to ");
            print_bytes(rest);
            println!();
        } else {
            println!("Failed: {}", result);
        }
        return;
    }

    if cmd_eq(first, b"module") && !rest.is_empty() {
        // Set per-module level: "log module pcie debug" or "log module pcie off"
        let (subsys, level_str) = libf::str::split_once(rest, b' ');
        let level_str = trim(level_str);
        if subsys.is_empty() || level_str.is_empty() {
            println!("Usage: log module <subsys> <level|off>");
            return;
        }
        let level = if cmd_eq(level_str, b"off") {
            0xFF // sentinel: remove override
        } else {
            match parse_log_level(level_str) {
                Some(l) => l,
                None => {
                    println!("Unknown level. Use: error, warn, info, notice, debug, trace, off");
                    return;
                }
            }
        };
        let result = syscall::set_module_level(subsys, level);
        if result == 0 {
            print!("Module ");
            print_bytes(subsys);
            print!(" level set to ");
            print_bytes(level_str);
            println!();
        } else {
            println!("Failed: {}", result);
        }
        return;
    }

    // Simple "log <level>" — sets both write and console levels
    let level = match parse_log_level(arg) {
        Some(l) => l,
        None => {
            println!("Usage: log <level>              Set write+console level");
            println!("       log console <level>      Set console output level only");
            println!("       log module <name> <level> Set per-module level");
            println!("Levels: error, warn, info, notice, debug, trace");
            return;
        }
    };

    let r1 = syscall::set_log_level(level);
    let r2 = syscall::set_console_level(level);
    if r1 == 0 && r2 == 0 {
        print!("Log level set to ");
        print_bytes(arg);
        println!();
    } else {
        println!("Failed: write={}, console={}", r1, r2);
    }
}

/// Parse a log level name to a u8 value
fn parse_log_level(name: &[u8]) -> Option<u8> {
    if cmd_eq(name, b"error") {
        Some(syscall::log_level::ERROR)
    } else if cmd_eq(name, b"warn") {
        Some(syscall::log_level::WARN)
    } else if cmd_eq(name, b"info") {
        Some(syscall::log_level::INFO)
    } else if cmd_eq(name, b"notice") {
        Some(syscall::log_level::NOTICE)
    } else if cmd_eq(name, b"debug") {
        Some(syscall::log_level::DEBUG)
    } else if cmd_eq(name, b"trace") {
        Some(syscall::log_level::TRACE)
    } else {
        None
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

/// Reset the system with graceful shutdown
fn cmd_reset() {
    println!("Shutting down...");

    // Send SHUTDOWN signal to all user processes (skip idle tasks and ourselves)
    let my_pid = syscall::getpid();
    let mut buf: [syscall::ProcessInfo; 32] = [syscall::ProcessInfo::empty(); 32];
    let count = syscall::ps_info(&mut buf);
    for i in 0..count {
        let pid = buf[i].pid;
        if pid <= 4 || pid == my_pid { continue; } // skip idle/kernel/devd/self
        let _ = syscall::signal(pid, syscall::signal_event::SHUTDOWN, 0);
    }

    // Brief delay to let drivers flush
    syscall::sleep_ms(200);

    println!("Resetting...");
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

