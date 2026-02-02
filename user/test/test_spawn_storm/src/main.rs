//! Process lifecycle stress test
//!
//! Phase 1: Sequential spawn/wait of test_exit_ok × 20 iterations × 3 rounds
//! Phase 2: Batch spawn 8 children, then batch wait all
//! Phase 3: Sequential spawn, reap, respawn (tests slot reuse)

#![no_std]
#![no_main]

use userlib::syscall;

fn log(msg: &[u8]) {
    syscall::debug_write(msg);
}

fn log_i64(n: i64) {
    if n < 0 {
        log(b"-");
        log_u64((-n) as u64);
    } else {
        log_u64(n as u64);
    }
}

fn log_u64(mut n: u64) {
    if n == 0 {
        log(b"0");
        return;
    }
    let mut buf = [0u8; 20];
    let mut i = 0;
    while n > 0 {
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut out = [0u8; 20];
    for j in 0..i {
        out[j] = buf[i - 1 - j];
    }
    log(&out[..i]);
}

/// Extract exit code from wait() return value.
/// wait() returns (pid << 32) | (exit_code & 0xFFFF_FFFF) on success,
/// or negative errno on error.
fn wait_exit_code(ret: i64) -> i32 {
    if ret < 0 {
        ret as i32 // Error
    } else {
        (ret & 0xFFFF_FFFF) as i32 // Exit code from lower 32 bits
    }
}

/// Phase 1: Sequential spawn+wait, 3 rounds of 20
fn phase1_sequential() -> bool {
    log(b"  phase1: sequential spawn/wait (3x20)...\r\n");

    for _round in 0..3u32 {
        for _ in 0..20u32 {
            let pid = syscall::exec("test_exit_ok");
            if pid < 0 {
                log(b"  phase1: FAIL - exec failed\r\n");
                return false;
            }
            let ret = syscall::wait(pid as i32);
            let code = wait_exit_code(ret);
            if code != 0 {
                log(b"  phase1: FAIL - wait ret=");
                log_i64(ret);
                log(b" code=");
                log_i64(code as i64);
                log(b"\r\n");
                return false;
            }
        }
    }

    log(b"  phase1: OK\r\n");
    true
}

/// Phase 2: Batch spawn 8, then batch wait
fn phase2_batch() -> bool {
    log(b"  phase2: batch spawn/wait (8 concurrent)...\r\n");

    const BATCH: usize = 8;
    let mut pids = [0i64; BATCH];

    for i in 0..BATCH {
        pids[i] = syscall::exec("test_exit_ok");
        if pids[i] < 0 {
            log(b"  phase2: FAIL - exec failed\r\n");
            // Wait for already spawned
            for j in 0..i {
                let _ = syscall::wait(pids[j] as i32);
            }
            return false;
        }
    }

    // Wait for all
    for i in 0..BATCH {
        let ret = syscall::wait(pids[i] as i32);
        let code = wait_exit_code(ret);
        if code != 0 {
            log(b"  phase2: FAIL - child exited with error\r\n");
            // Still wait for remaining
            for j in (i + 1)..BATCH {
                let _ = syscall::wait(pids[j] as i32);
            }
            return false;
        }
    }

    log(b"  phase2: OK\r\n");
    true
}

/// Phase 3: Spawn, reap, immediately respawn — tests slot reuse
fn phase3_slot_reuse() -> bool {
    log(b"  phase3: slot reuse (10 cycles)...\r\n");

    for _ in 0..10u32 {
        let pid = syscall::exec("test_exit_ok");
        if pid < 0 {
            log(b"  phase3: FAIL - exec failed\r\n");
            return false;
        }
        let ret = syscall::wait(pid as i32);
        let code = wait_exit_code(ret);
        if code != 0 {
            log(b"  phase3: FAIL - child exited with error\r\n");
            return false;
        }

        // Immediately respawn — the slot should have been freed
        let pid2 = syscall::exec("test_exit_ok");
        if pid2 < 0 {
            log(b"  phase3: FAIL - re-exec failed (slot reuse)\r\n");
            return false;
        }
        let ret2 = syscall::wait(pid2 as i32);
        let code2 = wait_exit_code(ret2);
        if code2 != 0 {
            log(b"  phase3: FAIL - re-spawned child failed\r\n");
            return false;
        }
    }

    log(b"  phase3: OK\r\n");
    true
}

#[unsafe(no_mangle)]
fn main() {
    let p1 = phase1_sequential();
    let p2 = phase2_batch();
    let p3 = phase3_slot_reuse();

    if p1 && p2 && p3 {
        syscall::exit(0);
    } else {
        syscall::exit(1);
    }
}
