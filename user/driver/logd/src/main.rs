//! Log Daemon (logd)
//!
//! Reads kernel logs and broadcasts to registered sinks.
//!
//! Design:
//! - Single reader of kernel log ring buffer
//! - Multiple sinks can register to receive logs
//! - Fire and forget: never blocks on slow sinks
//! - Sinks send periodic acks for health monitoring

#![no_std]
#![no_main]

use userlib::{println, syscall, EventFilter, kevent_subscribe, kevent_wait};
use userlib::syscall::{event_type, Event};
use userlib::ipc::Message;
use userlib::ipc::protocols::{LogdRequest, LogdEvent, MAX_LOG_RECORD};

/// Maximum number of registered sinks
const MAX_SINKS: usize = 8;

/// Sink health tracking
#[derive(Clone, Copy)]
struct Sink {
    /// Channel to this sink
    channel: u32,
    /// Is this slot active
    active: bool,
    /// Messages sent to this sink
    sent: u64,
    /// Send errors (channel full, etc.)
    errors: u64,
    /// Last ack received count
    ack_received: u64,
    /// Last ack processed count
    ack_processed: u64,
}

impl Sink {
    const fn empty() -> Self {
        Self {
            channel: 0,
            active: false,
            sent: 0,
            errors: 0,
            ack_received: 0,
            ack_processed: 0,
        }
    }
}

/// Log daemon state
struct Logd {
    /// Port for sink registration
    port: u32,
    /// Registered sinks
    sinks: [Sink; MAX_SINKS],
    /// Number of active sinks
    sink_count: usize,
    /// Total logs broadcast
    total_sent: u64,
    /// Total send errors
    total_errors: u64,
}

impl Logd {
    fn new(port: u32) -> Self {
        Self {
            port,
            sinks: [Sink::empty(); MAX_SINKS],
            sink_count: 0,
            total_sent: 0,
            total_errors: 0,
        }
    }

    /// Register a new sink
    fn register_sink(&mut self, channel: u32) -> bool {
        // Find empty slot
        for sink in &mut self.sinks {
            if !sink.active {
                sink.channel = channel;
                sink.active = true;
                sink.sent = 0;
                sink.errors = 0;
                sink.ack_received = 0;
                sink.ack_processed = 0;
                self.sink_count += 1;
                return true;
            }
        }
        false // No slots available
    }

    /// Remove a sink by channel
    fn remove_sink(&mut self, channel: u32) {
        for sink in &mut self.sinks {
            if sink.active && sink.channel == channel {
                sink.active = false;
                self.sink_count -= 1;
                syscall::channel_close(channel);
                break;
            }
        }
    }

    /// Update sink ack stats
    fn update_sink_ack(&mut self, channel: u32, received: u64, processed: u64) {
        for sink in &mut self.sinks {
            if sink.active && sink.channel == channel {
                sink.ack_received = received;
                sink.ack_processed = processed;
                break;
            }
        }
    }

    /// Broadcast log record to all sinks (fire and forget)
    fn broadcast(&mut self, record: &[u8]) {
        let event = LogdEvent::log(record);
        let mut buf = [0u8; MAX_LOG_RECORD + 8];
        let len = match event.serialize(&mut buf) {
            Ok(l) => l,
            Err(_) => return,
        };

        for sink in &mut self.sinks {
            if !sink.active {
                continue;
            }

            // Fire and forget send
            let result = syscall::send(sink.channel, &buf[..len]);
            if result >= 0 {
                sink.sent += 1;
                self.total_sent += 1;
            } else if result == -11 {
                // EAGAIN - channel full, count as error but don't disconnect
                sink.errors += 1;
                self.total_errors += 1;
            } else {
                // Channel dead, remove sink
                sink.active = false;
                self.sink_count -= 1;
                syscall::channel_close(sink.channel);
            }
        }
    }

    /// Handle incoming request from a sink
    fn handle_request(&mut self, channel: u32, data: &[u8]) {
        let Ok((request, _)) = LogdRequest::deserialize(data) else {
            return;
        };

        match request {
            LogdRequest::Register => {
                if self.register_sink(channel) {
                    // Send OK
                    let mut buf = [0u8; 4];
                    if let Ok(len) = LogdEvent::Ok.serialize(&mut buf) {
                        let _ = syscall::send(channel, &buf[..len]);
                    }
                } else {
                    // No slots, close channel
                    syscall::channel_close(channel);
                }
            }
            LogdRequest::Ack { received, processed } => {
                self.update_sink_ack(channel, received, processed);
            }
        }
    }

    /// Main event loop (kevent API)
    fn run(&mut self) {
        let mut events = [Event::empty(); 8];  // Batch receive
        let mut log_buf = [0u8; MAX_LOG_RECORD];
        let mut msg_buf = [0u8; 64];

        loop {
            // Wait for events (blocking, batch receive)
            let count = match kevent_wait(&mut events, u64::MAX) {
                Ok(n) => n,
                Err(_) => {
                    syscall::yield_now();
                    continue;
                }
            };

            for event in &events[..count] {
            match event.event_type {
                event_type::KLOG_READY => {
                    // Drain all available kernel logs
                    loop {
                        let n = syscall::klog_read(&mut log_buf);
                        if n <= 0 {
                            break;
                        }
                        self.broadcast(&log_buf[..n as usize]);
                    }
                }
                event_type::IPC_READY => {
                    let channel = event.data as u32;

                    // Check if this is a new connection on our port
                    let new_ch = syscall::port_accept(self.port);
                    if new_ch >= 0 {
                        // New sink connecting - wait for Register message
                        // The channel will send Register as first message
                    }

                    // Try to receive message from this channel
                    let n = syscall::receive_nonblock(channel, &mut msg_buf);
                    if n > 0 {
                        self.handle_request(channel, &msg_buf[..n as usize]);
                    } else if n < 0 && n != -11 {
                        // Channel error, remove sink
                        self.remove_sink(channel);
                    }
                }
                _ => {}
            }
            }
        }
    }
}

#[unsafe(no_mangle)]
fn main() {
    println!("logd: starting...");

    // Register port
    let port = syscall::port_register(b"logd");
    if port < 0 {
        println!("logd: failed to register port");
        syscall::exit(1);
    }

    let mut logd = Logd::new(port as u32);

    // Subscribe to KlogReady events (kevent API)
    if kevent_subscribe(EventFilter::KlogReady).is_err() {
        println!("logd: failed to subscribe to klog events");
        syscall::exit(1);
    }

    // Subscribe to IpcReady for our port (new connections) (kevent API)
    if kevent_subscribe(EventFilter::Ipc(port as u32)).is_err() {
        println!("logd: failed to subscribe to IPC events");
        syscall::exit(1);
    }

    println!("logd: ready");

    logd.run();
}
