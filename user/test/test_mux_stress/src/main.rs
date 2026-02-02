//! Event multiplexing stress test
//!
//! Phase 1: 8 timers with staggered deadlines in one Mux, verify all fire
//! Phase 2: 8 channel pairs in Mux, write to each, verify 8 readable events
//! Phase 3: Mixed handles (timers + channels) in one Mux

#![no_std]
#![no_main]

use userlib::syscall;
use userlib::ipc::{Channel, Mux, Timer, MuxFilter};

fn log(msg: &[u8]) {
    syscall::debug_write(msg);
}

/// Phase 1: 8 staggered timers
fn phase1_timers() -> bool {
    log(b"  phase1: 8 staggered timers...\r\n");

    const NUM_TIMERS: usize = 8;

    let mux = match Mux::new() {
        Ok(m) => m,
        Err(_) => {
            log(b"  phase1: FAIL - Mux::new\r\n");
            return false;
        }
    };

    let mut timers: [Option<Timer>; NUM_TIMERS] = [const { None }; NUM_TIMERS];
    let now = syscall::gettime();

    for i in 0..NUM_TIMERS {
        let mut timer = match Timer::new() {
            Ok(t) => t,
            Err(_) => {
                log(b"  phase1: FAIL - Timer::new\r\n");
                return false;
            }
        };

        // Stagger: 10ms, 20ms, 30ms, ...
        let deadline = now + ((i as u64 + 1) * 10_000_000);
        if timer.set(deadline).is_err() {
            log(b"  phase1: FAIL - timer.set\r\n");
            return false;
        }

        if mux.add(timer.handle(), MuxFilter::Readable).is_err() {
            log(b"  phase1: FAIL - mux.add\r\n");
            return false;
        }

        timers[i] = Some(timer);
    }

    // Set overall timeout
    if mux.set_timeout(5000).is_err() {
        log(b"  phase1: FAIL - set_timeout\r\n");
        return false;
    }

    // Collect all timer fires
    let mut fired = [false; NUM_TIMERS];
    let mut fire_count = 0u32;

    while fire_count < NUM_TIMERS as u32 {
        match mux.wait() {
            Ok(event) => {
                for i in 0..NUM_TIMERS {
                    if let Some(ref mut timer) = timers[i] {
                        if timer.handle() == event.handle && !fired[i] {
                            // Read the timer to acknowledge
                            let _ = timer.wait();
                            fired[i] = true;
                            fire_count += 1;
                            break;
                        }
                    }
                }
            }
            Err(userlib::SysError::Timeout) => {
                log(b"  phase1: FAIL - timeout before all timers fired\r\n");
                return false;
            }
            Err(_) => {
                log(b"  phase1: FAIL - mux.wait error\r\n");
                return false;
            }
        }
    }

    log(b"  phase1: OK\r\n");
    true
}

/// Phase 2: 8 channel pairs in Mux
fn phase2_channels() -> bool {
    log(b"  phase2: 8 channel pairs in mux...\r\n");

    const NUM_PAIRS: usize = 8;

    let mux = match Mux::new() {
        Ok(m) => m,
        Err(_) => {
            log(b"  phase2: FAIL - Mux::new\r\n");
            return false;
        }
    };

    let mut senders: [Option<Channel>; NUM_PAIRS] = [const { None }; NUM_PAIRS];
    let mut receivers: [Option<Channel>; NUM_PAIRS] = [const { None }; NUM_PAIRS];

    for i in 0..NUM_PAIRS {
        match Channel::pair() {
            Ok((a, b)) => {
                if mux.add(b.handle(), MuxFilter::Readable).is_err() {
                    log(b"  phase2: FAIL - mux.add\r\n");
                    return false;
                }
                senders[i] = Some(a);
                receivers[i] = Some(b);
            }
            Err(_) => {
                log(b"  phase2: FAIL - Channel::pair\r\n");
                return false;
            }
        }
    }

    // Write one message to each sender
    for i in 0..NUM_PAIRS {
        if let Some(ref sender) = senders[i] {
            let msg = (i as u32).to_le_bytes();
            if sender.send(&msg).is_err() {
                log(b"  phase2: FAIL - send\r\n");
                return false;
            }
        }
    }

    // Set timeout
    if mux.set_timeout(5000).is_err() {
        log(b"  phase2: FAIL - set_timeout\r\n");
        return false;
    }

    // Receive all via Mux
    let mut received = [false; NUM_PAIRS];
    let mut recv_count = 0u32;

    while recv_count < NUM_PAIRS as u32 {
        match mux.wait() {
            Ok(event) => {
                for i in 0..NUM_PAIRS {
                    if let Some(ref mut receiver) = receivers[i] {
                        if receiver.handle() == event.handle && !received[i] {
                            let mut buf = [0u8; 4];
                            match receiver.try_recv(&mut buf) {
                                Ok(Some(4)) => {
                                    let val = u32::from_le_bytes(buf);
                                    if val != i as u32 {
                                        log(b"  phase2: FAIL - wrong value\r\n");
                                        return false;
                                    }
                                    received[i] = true;
                                    recv_count += 1;
                                }
                                _ => {
                                    log(b"  phase2: FAIL - recv\r\n");
                                    return false;
                                }
                            }
                            break;
                        }
                    }
                }
            }
            Err(userlib::SysError::Timeout) => {
                log(b"  phase2: FAIL - timeout\r\n");
                return false;
            }
            Err(_) => {
                log(b"  phase2: FAIL - mux.wait error\r\n");
                return false;
            }
        }
    }

    log(b"  phase2: OK\r\n");
    true
}

/// Phase 3: Mixed handles (timers + channels) in one Mux
fn phase3_mixed() -> bool {
    log(b"  phase3: mixed handles (timers + channels)...\r\n");

    const NUM_TIMERS: usize = 4;
    const NUM_CHANNELS: usize = 4;

    let mux = match Mux::new() {
        Ok(m) => m,
        Err(_) => {
            log(b"  phase3: FAIL - Mux::new\r\n");
            return false;
        }
    };

    // Create timers
    let mut timers: [Option<Timer>; NUM_TIMERS] = [const { None }; NUM_TIMERS];
    let now = syscall::gettime();

    for i in 0..NUM_TIMERS {
        let mut timer = match Timer::new() {
            Ok(t) => t,
            Err(_) => {
                log(b"  phase3: FAIL - Timer::new\r\n");
                return false;
            }
        };

        let deadline = now + ((i as u64 + 1) * 10_000_000);
        if timer.set(deadline).is_err() {
            log(b"  phase3: FAIL - timer.set\r\n");
            return false;
        }
        if mux.add(timer.handle(), MuxFilter::Readable).is_err() {
            log(b"  phase3: FAIL - mux.add timer\r\n");
            return false;
        }
        timers[i] = Some(timer);
    }

    // Create channel pairs
    let mut senders: [Option<Channel>; NUM_CHANNELS] = [const { None }; NUM_CHANNELS];
    let mut receivers: [Option<Channel>; NUM_CHANNELS] = [const { None }; NUM_CHANNELS];

    for i in 0..NUM_CHANNELS {
        match Channel::pair() {
            Ok((a, b)) => {
                if mux.add(b.handle(), MuxFilter::Readable).is_err() {
                    log(b"  phase3: FAIL - mux.add channel\r\n");
                    return false;
                }
                senders[i] = Some(a);
                receivers[i] = Some(b);
            }
            Err(_) => {
                log(b"  phase3: FAIL - Channel::pair\r\n");
                return false;
            }
        }
    }

    // Write to all channels
    for i in 0..NUM_CHANNELS {
        if let Some(ref sender) = senders[i] {
            let msg = (i as u32 + 100).to_le_bytes();
            if sender.send(&msg).is_err() {
                log(b"  phase3: FAIL - send\r\n");
                return false;
            }
        }
    }

    // Set timeout
    if mux.set_timeout(5000).is_err() {
        log(b"  phase3: FAIL - set_timeout\r\n");
        return false;
    }

    // Wait for all events
    let total_events = (NUM_TIMERS + NUM_CHANNELS) as u32;
    let mut timer_fired = [false; NUM_TIMERS];
    let mut chan_received = [false; NUM_CHANNELS];
    let mut event_count = 0u32;

    while event_count < total_events {
        match mux.wait() {
            Ok(event) => {
                let mut matched = false;

                // Check timers
                for i in 0..NUM_TIMERS {
                    if let Some(ref mut timer) = timers[i] {
                        if timer.handle() == event.handle && !timer_fired[i] {
                            let _ = timer.wait();
                            timer_fired[i] = true;
                            event_count += 1;
                            matched = true;
                            break;
                        }
                    }
                }

                if !matched {
                    // Check channels
                    for i in 0..NUM_CHANNELS {
                        if let Some(ref mut receiver) = receivers[i] {
                            if receiver.handle() == event.handle && !chan_received[i] {
                                let mut buf = [0u8; 4];
                                match receiver.try_recv(&mut buf) {
                                    Ok(Some(4)) => {
                                        let val = u32::from_le_bytes(buf);
                                        if val != (i as u32 + 100) {
                                            log(b"  phase3: FAIL - wrong value\r\n");
                                            return false;
                                        }
                                        chan_received[i] = true;
                                        event_count += 1;
                                    }
                                    _ => {
                                        log(b"  phase3: FAIL - recv\r\n");
                                        return false;
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
            }
            Err(userlib::SysError::Timeout) => {
                log(b"  phase3: FAIL - timeout\r\n");
                return false;
            }
            Err(_) => {
                log(b"  phase3: FAIL - mux.wait error\r\n");
                return false;
            }
        }
    }

    log(b"  phase3: OK\r\n");
    true
}

#[unsafe(no_mangle)]
fn main() {
    let p1 = phase1_timers();
    let p2 = phase2_channels();
    let p3 = phase3_mixed();

    if p1 && p2 && p3 {
        syscall::exit(0);
    } else {
        syscall::exit(1);
    }
}
