//! Log Daemon (logd)
//!
//! Reads kernel logs and broadcasts to registered sinks.
//!
//! Design:
//! - Single reader of kernel log ring buffer
//! - Multiple sinks can register to receive logs
//! - Fire and forget: never blocks on slow sinks
//! - Sinks send periodic acks for health monitoring
//! - Uses handle API for unified waiting

#![no_std]
#![no_main]

use userlib::{println, syscall};
use userlib::syscall::{
    Handle, WaitFilter, WaitRequest, WaitResult,
    handle_klog_create, handle_wrap_channel, handle_wait, handle_close,
};
use userlib::ipc::{Message, notify_service_ready};
use userlib::ipc::protocols::{LogdRequest, LogdEvent, MAX_LOG_RECORD};

/// Maximum number of registered sinks
const MAX_SINKS: usize = 8;

/// Maximum handles to wait on (klog + port + sinks)
const MAX_WAIT_HANDLES: usize = 2 + MAX_SINKS;

/// Sink health tracking
#[derive(Clone, Copy)]
struct Sink {
    /// Channel to this sink
    channel: u32,
    /// Handle for this channel (for handle_wait)
    handle: Handle,
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
            handle: Handle::INVALID,
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
    /// Handle for the port (for handle_wait)
    port_handle: Handle,
    /// Handle for klog (kernel log availability)
    klog_handle: Handle,
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
    fn new(port: u32, port_handle: Handle, klog_handle: Handle) -> Self {
        Self {
            port,
            port_handle,
            klog_handle,
            sinks: [Sink::empty(); MAX_SINKS],
            sink_count: 0,
            total_sent: 0,
            total_errors: 0,
        }
    }

    /// Register a new sink
    fn register_sink(&mut self, channel: u32, handle: Handle) -> bool {
        // Find empty slot
        for sink in &mut self.sinks {
            if !sink.active {
                sink.channel = channel;
                sink.handle = handle;
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
                let _ = handle_close(sink.handle);
                syscall::channel_close(channel);
                break;
            }
        }
    }

    /// Remove a sink by handle
    fn remove_sink_by_handle(&mut self, handle: Handle) {
        for sink in &mut self.sinks {
            if sink.active && sink.handle.raw() == handle.raw() {
                sink.active = false;
                self.sink_count -= 1;
                let _ = handle_close(sink.handle);
                syscall::channel_close(sink.channel);
                break;
            }
        }
    }

    /// Find channel by handle
    fn find_channel_by_handle(&self, handle: Handle) -> Option<u32> {
        for sink in &self.sinks {
            if sink.active && sink.handle.raw() == handle.raw() {
                return Some(sink.channel);
            }
        }
        None
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
                let _ = handle_close(sink.handle);
                syscall::channel_close(sink.channel);
            }
        }
    }

    /// Handle incoming request from a sink
    fn handle_request(&mut self, channel: u32, handle: Handle, data: &[u8]) {
        let Ok((request, _)) = LogdRequest::deserialize(data) else {
            return;
        };

        match request {
            LogdRequest::Register => {
                if self.register_sink(channel, handle) {
                    // Send OK
                    let _ = LogdEvent::Ok.send_to(channel);
                } else {
                    // No slots, close channel and handle
                    let _ = handle_close(handle);
                    syscall::channel_close(channel);
                }
            }
            LogdRequest::Ack { received, processed } => {
                self.update_sink_ack(channel, received, processed);
            }
        }
    }

    /// Build wait request array from current handles
    fn build_wait_requests(&self) -> ([WaitRequest; MAX_WAIT_HANDLES], usize) {
        let mut requests = [WaitRequest::new(Handle::INVALID, WaitFilter::Readable); MAX_WAIT_HANDLES];
        let mut count = 0;

        // Always wait on klog handle (for kernel log availability)
        requests[count] = WaitRequest::new(self.klog_handle, WaitFilter::Readable);
        count += 1;

        // Always wait on port handle (for new connections)
        requests[count] = WaitRequest::new(self.port_handle, WaitFilter::Readable);
        count += 1;

        // Wait on all active sink handles
        for sink in &self.sinks {
            if sink.active {
                requests[count] = WaitRequest::new(sink.handle, WaitFilter::Readable);
                count += 1;
            }
        }

        (requests, count)
    }

    /// Main event loop (handle API)
    fn run(&mut self) {
        let mut results = [WaitResult::empty(); MAX_WAIT_HANDLES];
        let mut log_buf = [0u8; MAX_LOG_RECORD];
        let mut msg_buf = [0u8; 64];

        loop {
            // Build wait request array
            let (requests, req_count) = self.build_wait_requests();

            // Wait for events (blocking)
            let count = match handle_wait(&requests[..req_count], &mut results, u64::MAX) {
                Ok(n) => n,
                Err(_) => {
                    syscall::yield_now();
                    continue;
                }
            };

            for result in &results[..count] {
                // Check if this is the klog handle
                if result.handle.raw() == self.klog_handle.raw() {
                    // Drain all available kernel logs
                    loop {
                        let n = syscall::klog_read(&mut log_buf);
                        if n <= 0 {
                            break;
                        }
                        self.broadcast(&log_buf[..n as usize]);
                    }
                }
                // Check if this is the port handle (new connection)
                else if result.handle.raw() == self.port_handle.raw() {
                    let new_ch = syscall::port_accept(self.port);
                    if new_ch >= 0 {
                        let new_ch = new_ch as u32;
                        // Wrap the new channel as a handle
                        match handle_wrap_channel(new_ch) {
                            Ok(handle) => {
                                // Check for pending Register message
                                let n = syscall::receive_nonblock(new_ch, &mut msg_buf);
                                if n > 0 {
                                    self.handle_request(new_ch, handle, &msg_buf[..n as usize]);
                                } else {
                                    // No message yet, register with empty handle
                                    // (will receive Register later)
                                    if !self.register_sink(new_ch, handle) {
                                        // No slots, close
                                        let _ = handle_close(handle);
                                        syscall::channel_close(new_ch);
                                    }
                                }
                            }
                            Err(_) => {
                                // Failed to wrap channel, close it
                                syscall::channel_close(new_ch);
                            }
                        }
                    }
                }
                // Otherwise it's a sink handle
                else {
                    if let Some(channel) = self.find_channel_by_handle(result.handle) {
                        // Message on existing channel
                        let n = syscall::receive_nonblock(channel, &mut msg_buf);
                        if n > 0 {
                            self.handle_request(channel, result.handle, &msg_buf[..n as usize]);
                        } else if n < 0 && n != -11 {
                            // Channel error, remove sink
                            self.remove_sink_by_handle(result.handle);
                        }
                    }
                }
            }
        }
    }
}

#[unsafe(no_mangle)]
fn main() {
    // Register port
    let port = syscall::port_register(b"logd");
    if port < 0 {
        println!("logd: failed to register port");
        syscall::exit(1);
    }
    let port = port as u32;

    // Create klog handle for kernel log availability
    let klog_handle = match handle_klog_create() {
        Ok(h) => h,
        Err(_) => {
            println!("logd: failed to create klog handle");
            syscall::exit(1);
        }
    };

    // Wrap the port as a handle for new connections
    let port_handle = match handle_wrap_channel(port) {
        Ok(h) => h,
        Err(_) => {
            println!("logd: failed to wrap port as handle");
            syscall::exit(1);
        }
    };

    let mut logd = Logd::new(port, port_handle, klog_handle);

    // Notify devd that logd is ready (port registered)
    notify_service_ready("logd");

    logd.run();
}
