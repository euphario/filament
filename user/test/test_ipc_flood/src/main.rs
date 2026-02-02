//! IPC channel stress test
//!
//! Phase 1: Create+drop channel pairs in batches (object alloc/free churn)
//! Phase 2: Single pair, send many messages, verify ordering via sequence numbers
//! Phase 3: 8 concurrent channel pairs, Mux-based interleaved recv

#![no_std]
#![no_main]

use userlib::syscall;
use userlib::ipc::{Channel, Mux, MuxFilter};

fn log(msg: &[u8]) {
    syscall::debug_write(msg);
}

fn log_i32(n: i32) {
    if n < 0 {
        log(b"-");
        log_u32((-(n as i64)) as u32);
    } else {
        log_u32(n as u32);
    }
}

fn log_u32(mut n: u32) {
    if n == 0 {
        log(b"0");
        return;
    }
    let mut buf = [0u8; 10];
    let mut i = 0;
    while n > 0 {
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    // Reverse
    let mut out = [0u8; 10];
    for j in 0..i {
        out[j] = buf[i - 1 - j];
    }
    log(&out[..i]);
}

/// Phase 1: Channel create/drop in series
/// MAX_HANDLES=64, each pair uses 2 handles, so we can do ~30 pairs at a time.
/// But since we drop immediately, handles get recycled.
fn phase1_alloc_churn() -> bool {
    log(b"  phase1: alloc churn (50 pairs)...\r\n");
    for _ in 0..50 {
        match Channel::pair() {
            Ok((_a, _b)) => {
                // Channels drop here — exercises close path
            }
            Err(e) => {
                log(b"  phase1: FAIL - channel_pair failed err=");
                log_i32(e.to_errno());
                log(b"\r\n");
                return false;
            }
        }
    }
    log(b"  phase1: OK\r\n");
    true
}

/// Phase 2: Sequential messaging with sequence number verification
/// Sends in batches (queue holds 8 msgs), then drains before next batch.
fn phase2_sequential_flood() -> bool {
    log(b"  phase2: sequential flood (1000 msgs)...\r\n");

    let (a, mut b) = match Channel::pair() {
        Ok(pair) => pair,
        Err(e) => {
            log(b"  phase2: FAIL - channel_pair failed err=");
            log_i32(e.to_errno());
            log(b"\r\n");
            return false;
        }
    };

    const COUNT: u32 = 1000;
    const BATCH: u32 = 8; // Channel queue size

    let mut next_send: u32 = 0;
    let mut next_recv: u32 = 0;

    while next_recv < COUNT {
        // Send a batch (up to BATCH messages, or until COUNT)
        while next_send < COUNT && next_send - next_recv < BATCH {
            let data = next_send.to_le_bytes();
            if a.send(&data).is_err() {
                log(b"  phase2: FAIL - send failed\r\n");
                return false;
            }
            next_send += 1;
        }

        // Drain all available messages
        loop {
            let mut buf = [0u8; 4];
            match b.try_recv(&mut buf) {
                Ok(Some(4)) => {
                    let got = u32::from_le_bytes(buf);
                    if got != next_recv {
                        log(b"  phase2: FAIL - sequence mismatch\r\n");
                        return false;
                    }
                    next_recv += 1;
                }
                Ok(Some(_)) => {
                    log(b"  phase2: FAIL - short read\r\n");
                    return false;
                }
                Ok(None) => break, // No more data available
                Err(_) => {
                    log(b"  phase2: FAIL - try_recv error\r\n");
                    return false;
                }
            }
        }
    }

    log(b"  phase2: OK\r\n");
    true
}

/// Phase 3: 8 concurrent channel pairs with Mux-based interleaved recv
/// Queue depth is 8, so we send in batches of 6, drain via Mux, repeat.
fn phase3_mux_interleave() -> bool {
    log(b"  phase3: mux interleave (8 pairs)...\r\n");

    const NUM_PAIRS: usize = 8;
    const MSGS_PER_PAIR: u32 = 50;
    const BATCH: u32 = 6; // Stay under queue depth (8)

    // Create pairs
    let mut senders: [Option<Channel>; NUM_PAIRS] = [const { None }; NUM_PAIRS];
    let mut receivers: [Option<Channel>; NUM_PAIRS] = [const { None }; NUM_PAIRS];

    for i in 0..NUM_PAIRS {
        match Channel::pair() {
            Ok((a, b)) => {
                senders[i] = Some(a);
                receivers[i] = Some(b);
            }
            Err(_) => {
                log(b"  phase3: FAIL - channel_pair failed\r\n");
                return false;
            }
        }
    }

    // Create Mux and add all receivers
    let mux = match Mux::new() {
        Ok(m) => m,
        Err(_) => {
            log(b"  phase3: FAIL - Mux::new failed\r\n");
            return false;
        }
    };

    for i in 0..NUM_PAIRS {
        if let Some(ref receiver) = receivers[i] {
            if mux.add(receiver.handle(), MuxFilter::Readable).is_err() {
                log(b"  phase3: FAIL - mux.add failed\r\n");
                return false;
            }
        }
    }

    // Track per-pair send/recv progress
    let mut sent = [0u32; NUM_PAIRS];
    let mut counts = [0u32; NUM_PAIRS];
    let total_expected = (NUM_PAIRS as u32) * MSGS_PER_PAIR;
    let mut total_received: u32 = 0;

    // Interleave: send a batch on all pairs, then drain via Mux, repeat
    while total_received < total_expected {
        // Send a batch on each pair that still has messages to send
        for i in 0..NUM_PAIRS {
            if sent[i] < MSGS_PER_PAIR {
                if let Some(ref sender) = senders[i] {
                    let end = core::cmp::min(sent[i] + BATCH, MSGS_PER_PAIR);
                    while sent[i] < end {
                        let val = (i as u32) << 16 | sent[i];
                        if sender.send(&val.to_le_bytes()).is_err() {
                            log(b"  phase3: FAIL - send failed\r\n");
                            return false;
                        }
                        sent[i] += 1;
                    }
                }
            }
        }

        // Drain all available messages via Mux (short timeout to avoid blocking)
        loop {
            match mux.wait_timeout(10) {
                Ok(event) => {
                    for i in 0..NUM_PAIRS {
                        if let Some(ref mut receiver) = receivers[i] {
                            if receiver.handle() == event.handle {
                                loop {
                                    let mut buf = [0u8; 4];
                                    match receiver.try_recv(&mut buf) {
                                        Ok(Some(4)) => {
                                            let val = u32::from_le_bytes(buf);
                                            let pair_idx = (val >> 16) as usize;
                                            if pair_idx != i {
                                                log(b"  phase3: FAIL - wrong pair index\r\n");
                                                return false;
                                            }
                                            counts[i] += 1;
                                            total_received += 1;
                                        }
                                        Ok(Some(_)) => {
                                            log(b"  phase3: FAIL - short read\r\n");
                                            return false;
                                        }
                                        Ok(None) => break,
                                        Err(_) => break,
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
                Err(userlib::SysError::Timeout) => break, // Batch drained, send more
                Err(_) => {
                    log(b"  phase3: FAIL - mux.wait error\r\n");
                    return false;
                }
            }
        }
    }

    // Verify all pairs got the right count
    for i in 0..NUM_PAIRS {
        if counts[i] != MSGS_PER_PAIR {
            log(b"  phase3: FAIL - wrong message count\r\n");
            return false;
        }
    }

    log(b"  phase3: OK\r\n");
    true
}

#[unsafe(no_mangle)]
fn main() {
    let p1 = phase1_alloc_churn();
    let p2 = phase2_sequential_flood();
    let p3 = phase3_mux_interleave();

    if p1 && p2 && p3 {
        syscall::exit(0);
    } else {
        syscall::exit(1);
    }
}
