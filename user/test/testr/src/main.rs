//! Stress test runner (testr)
//!
//! Spawned by devd when stress-test feature is active.
//! Sequentially runs each test program, captures exit codes,
//! prints structured STRESS_RESULT lines, then calls shutdown().

#![no_std]
#![no_main]

use userlib::syscall;
use userlib::ipc::{Mux, Process, Timer, MuxFilter};

/// Test binary names (must exist in initrd)
static TESTS: &[&str] = &[
    "test_ipc_flood",
    "test_spawn_storm",
    "test_mux_stress",
];

/// Per-test timeout in milliseconds
const TEST_TIMEOUT_MS: u32 = 30_000;

fn log(msg: &[u8]) {
    syscall::debug_write(msg);
}

fn log_str(s: &str) {
    log(s.as_bytes());
}

/// Format a small number into a buffer. Returns the used slice.
fn fmt_u32(buf: &mut [u8], mut n: u32) -> usize {
    if n == 0 {
        buf[0] = b'0';
        return 1;
    }
    let mut tmp = [0u8; 10];
    let mut i = 0;
    while n > 0 {
        tmp[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    // Reverse into buf
    for j in 0..i {
        buf[j] = tmp[i - 1 - j];
    }
    i
}

fn fmt_i32(buf: &mut [u8], n: i32) -> usize {
    if n < 0 {
        buf[0] = b'-';
        1 + fmt_u32(&mut buf[1..], (-(n as i64)) as u32)
    } else {
        fmt_u32(buf, n as u32)
    }
}

/// Run a single test. Returns exit code (0 = pass, nonzero = fail, -1 = timeout).
fn run_test(name: &str) -> i32 {
    // Spawn the test binary
    let pid = syscall::exec(name);
    if pid < 0 {
        log_str("  ERROR: exec failed for ");
        log(name.as_bytes());
        log(b"\r\n");
        return 1;
    }
    let pid = pid as u32;

    // Watch the process and set a timeout
    let watcher = match Process::watch(pid) {
        Ok(w) => w,
        Err(_) => {
            log_str("  ERROR: Process::watch failed\r\n");
            return 1;
        }
    };

    let timer = match Timer::new() {
        Ok(t) => t,
        Err(_) => {
            log_str("  ERROR: Timer::new failed\r\n");
            return 1;
        }
    };

    let mux = match Mux::new() {
        Ok(m) => m,
        Err(_) => {
            log_str("  ERROR: Mux::new failed\r\n");
            return 1;
        }
    };

    // Add watcher and timer to mux
    if mux.add(watcher.handle(), MuxFilter::Readable).is_err() {
        log_str("  ERROR: mux.add watcher failed\r\n");
        return 1;
    }

    let mut timer = timer;
    let deadline = syscall::gettime() + (TEST_TIMEOUT_MS as u64) * 1_000_000;
    if timer.set(deadline).is_err() {
        log_str("  ERROR: timer.set failed\r\n");
        return 1;
    }

    if mux.add(timer.handle(), MuxFilter::Readable).is_err() {
        log_str("  ERROR: mux.add timer failed\r\n");
        return 1;
    }

    // Wait for either process exit or timeout
    let mut watcher = watcher;
    loop {
        match mux.wait() {
            Ok(event) => {
                if event.handle == watcher.handle() {
                    // Process exited — read exit code
                    let code = match watcher.wait() {
                        Ok(code) => code,
                        Err(_) => 1,
                    };
                    // Reap the child to free its task slot
                    let _ = syscall::wait(pid as i32);
                    return code;
                } else if event.handle == timer.handle() {
                    // Timeout — kill the test
                    log_str("  TIMEOUT\r\n");
                    let _ = syscall::kill(pid);
                    // Reap the child to free its task slot
                    let _ = watcher.wait();
                    let _ = syscall::wait(pid as i32);
                    return -1;
                }
            }
            Err(_) => {
                // Error waiting — try once more then bail
                return 1;
            }
        }
    }
}

#[unsafe(no_mangle)]
fn main() {
    // Wait briefly for console to be ready
    syscall::sleep_ms(100);

    log(b"\r\nSTRESS_START\r\n");

    let mut passed: u32 = 0;
    let mut total: u32 = 0;

    for &name in TESTS {
        total += 1;

        log(b"STRESS_RUN: ");
        log(name.as_bytes());
        log(b"\r\n");

        let exit_code = run_test(name);

        log(b"STRESS_RESULT: ");
        log(name.as_bytes());

        if exit_code == 0 {
            log(b" PASS\r\n");
            passed += 1;
        } else {
            log(b" FAIL exit_code=");
            let mut buf = [0u8; 12];
            let len = fmt_i32(&mut buf, exit_code);
            log(&buf[..len]);
            log(b"\r\n");
        }
    }

    // Summary
    log(b"STRESS_SUMMARY: ");
    let mut buf = [0u8; 12];
    let len = fmt_u32(&mut buf, passed);
    log(&buf[..len]);
    log(b"/");
    let len = fmt_u32(&mut buf, total);
    log(&buf[..len]);
    log(b" passed\r\n");

    // Done
    let final_code = if passed == total { 0u8 } else { 1u8 };
    log(b"STRESS_DONE exit_code=");
    let len = fmt_u32(&mut buf, final_code as u32);
    log(&buf[..len]);
    log(b"\r\n");

    // Shutdown the system (QEMU will exit)
    syscall::shutdown(final_code);

    // Fallback if shutdown not supported
    syscall::exit(final_code as i32);
}
