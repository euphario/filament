//! Console Daemon (consoled)
//!
//! Manages the terminal display with split regions for logs and shell.
//!
//! Architecture:
//! ```text
//! ┌─────────────────────────────┐
//! │  Log Region (scrolling)     │ ← klog, ktrace, ulog, utrace
//! ├─────────────────────────────┤
//! │  Shell Region (dynamic)     │ ← shell I/O via console protocol
//! │  > command_                 │
//! └─────────────────────────────┘
//! ```
//!
//! The shell communicates via the "console" port and sends InputState
//! updates so consoled can dynamically adjust regions.

#![no_std]
#![no_main]

use userlib::{println, syscall, ByteRing, EventFilter, kevent_subscribe, kevent_timer, kevent_wait};
use userlib::syscall::{event_type, Event};
use userlib::ipc::Message;
use userlib::ipc::protocols::{
    ConsoleRequest, ConsoleResponse, InputState,
    LogdRequest, LogdEvent, MAX_LOG_RECORD,
};
use userlib::ulog::format_record;

/// Default terminal size (will try to detect)
const DEFAULT_COLS: u16 = 80;
const DEFAULT_ROWS: u16 = 24;

/// Minimum shell region size
const MIN_SHELL_LINES: u8 = 2;

/// ANSI escape sequences
mod ansi {
    use userlib::syscall;

    /// Query terminal size using cursor position report (CPR)
    /// Returns (cols, rows) or None if detection fails
    pub fn query_screen_size() -> Option<(u16, u16)> {
        // Save cursor, move to bottom-right corner, query position
        // ESC[s = save cursor
        // ESC[9999;9999H = move to large coords (clamps to screen edge)
        // ESC[6n = query cursor position (DSR - Device Status Report)
        write(b"\x1b[s\x1b[9999;9999H\x1b[6n");

        // Read response: ESC [ rows ; cols R
        // Give the terminal some time to respond
        let mut buf = [0u8; 32];
        let mut len = 0;

        // Simple polling loop with timeout
        // Read bytes until we get 'R' or timeout
        for _ in 0..100 {  // ~100ms timeout at 1ms per iteration
            let mut byte = [0u8; 1];
            let n = syscall::read(syscall::STDIN, &mut byte);
            if n > 0 {
                if len < buf.len() {
                    buf[len] = byte[0];
                    len += 1;
                }
                if byte[0] == b'R' {
                    break;
                }
            } else {
                // No data yet, small delay
                for _ in 0..10000 {
                    core::hint::spin_loop();
                }
            }
        }

        // Restore cursor position
        write(b"\x1b[u");

        // Parse response: ESC [ rows ; cols R
        // Example: "\x1b[24;80R" means 24 rows, 80 cols
        if len < 6 {
            return None;  // Too short
        }

        // Find ESC [
        let mut start = 0;
        while start + 2 < len && !(buf[start] == 0x1b && buf[start + 1] == b'[') {
            start += 1;
        }

        if start + 2 >= len {
            return None;
        }

        // Parse rows (after '[')
        let mut i = start + 2;
        let mut rows: u16 = 0;
        while i < len && buf[i] >= b'0' && buf[i] <= b'9' {
            rows = rows.saturating_mul(10).saturating_add((buf[i] - b'0') as u16);
            i += 1;
        }

        // Expect ';'
        if i >= len || buf[i] != b';' {
            return None;
        }
        i += 1;

        // Parse cols
        let mut cols: u16 = 0;
        while i < len && buf[i] >= b'0' && buf[i] <= b'9' {
            cols = cols.saturating_mul(10).saturating_add((buf[i] - b'0') as u16);
            i += 1;
        }

        // Expect 'R'
        if i >= len || buf[i] != b'R' {
            return None;
        }

        // Validate reasonable dimensions
        if rows >= 10 && rows <= 500 && cols >= 40 && cols <= 500 {
            Some((cols, rows))
        } else {
            None
        }
    }

    /// Clear screen
    pub fn clear() {
        write(b"\x1b[2J");
    }

    /// Move cursor to position (1-indexed)
    pub fn goto(row: u16, col: u16) {
        let mut buf = [0u8; 16];
        let len = format_goto(&mut buf, row, col);
        write(&buf[..len]);
    }

    /// Set scrolling region (1-indexed, inclusive)
    pub fn set_scroll_region(top: u16, bottom: u16) {
        let mut buf = [0u8; 16];
        let len = format_scroll_region(&mut buf, top, bottom);
        write(&buf[..len]);
    }

    /// Reset scrolling region to full screen
    pub fn reset_scroll_region() {
        write(b"\x1b[r");
    }

    /// Save cursor position
    pub fn save_cursor() {
        write(b"\x1b[s");
    }

    /// Restore cursor position
    pub fn restore_cursor() {
        write(b"\x1b[u");
    }

    /// Clear line from cursor to end
    pub fn clear_to_eol() {
        write(b"\x1b[K");
    }

    /// Clear entire line
    pub fn clear_line() {
        write(b"\x1b[2K");
    }

    /// Scroll up one line (in current scroll region)
    pub fn scroll_up() {
        write(b"\x1b[S");
    }

    /// Colors
    pub fn set_dim() {
        write(b"\x1b[2m");
    }

    pub fn set_bold() {
        write(b"\x1b[1m");
    }

    pub fn set_cyan() {
        write(b"\x1b[36m");
    }

    pub fn set_yellow() {
        write(b"\x1b[33m");
    }

    pub fn set_red() {
        write(b"\x1b[31m");
    }

    pub fn reset_style() {
        write(b"\x1b[0m");
    }

    fn write(data: &[u8]) {
        let _ = syscall::write(syscall::STDOUT, data);
    }

    fn format_goto(buf: &mut [u8], row: u16, col: u16) -> usize {
        // ESC [ row ; col H
        buf[0] = 0x1b;
        buf[1] = b'[';
        let mut i = 2;
        i += format_u16(&mut buf[i..], row);
        buf[i] = b';';
        i += 1;
        i += format_u16(&mut buf[i..], col);
        buf[i] = b'H';
        i + 1
    }

    fn format_scroll_region(buf: &mut [u8], top: u16, bottom: u16) -> usize {
        // ESC [ top ; bottom r
        buf[0] = 0x1b;
        buf[1] = b'[';
        let mut i = 2;
        i += format_u16(&mut buf[i..], top);
        buf[i] = b';';
        i += 1;
        i += format_u16(&mut buf[i..], bottom);
        buf[i] = b'r';
        i + 1
    }

    fn format_u16(buf: &mut [u8], n: u16) -> usize {
        if n == 0 {
            buf[0] = b'0';
            return 1;
        }
        let mut tmp = [0u8; 5];
        let mut val = n;
        let mut len = 0;
        while val > 0 {
            tmp[len] = b'0' + (val % 10) as u8;
            val /= 10;
            len += 1;
        }
        for i in 0..len {
            buf[i] = tmp[len - 1 - i];
        }
        len
    }
}

/// Size of input ring buffer (1KB for keyboard input)
const INPUT_RING_SIZE: usize = 1024;

/// Console state
struct Console {
    /// Terminal dimensions
    cols: u16,
    rows: u16,

    /// Log split enabled
    log_split: bool,

    /// Number of lines for logs (0 = auto)
    log_lines: u8,

    /// Current shell input state
    shell_state: InputState,

    /// Shell channel (if connected)
    shell_channel: Option<u32>,

    /// UART file descriptor
    uart_fd: u32,

    /// Cursor position in shell region (for restoring after log output)
    shell_cursor_row: u16,
    shell_cursor_col: u16,

    /// Shell output ring buffer (shell writes, consoled reads)
    shell_output_ring: Option<ByteRing>,

    /// Shell input ring buffer (consoled writes, shell reads)
    shell_input_ring: Option<ByteRing>,
}

impl Console {
    fn new(uart_fd: u32) -> Self {
        Self {
            cols: DEFAULT_COLS,
            rows: DEFAULT_ROWS,
            log_split: false,  // TODO: re-enable when layout bugs fixed
            log_lines: 0,  // Auto
            shell_state: InputState::SingleLine,
            shell_channel: None,
            uart_fd,
            shell_cursor_row: DEFAULT_ROWS,
            shell_cursor_col: 1,
            shell_output_ring: None,
            shell_input_ring: None,
        }
    }

    /// Calculate log region size
    fn log_region_lines(&self) -> u16 {
        if !self.log_split {
            return 0;
        }

        let shell_lines = self.shell_region_lines();
        let available = self.rows.saturating_sub(shell_lines).saturating_sub(1);  // -1 for separator

        if self.log_lines > 0 {
            (self.log_lines as u16).min(available)
        } else {
            // Auto: give 2/3 to logs
            (self.rows * 2 / 3).min(available)
        }
    }

    /// Calculate shell region size based on input state
    fn shell_region_lines(&self) -> u16 {
        let lines = match self.shell_state {
            InputState::SingleLine => 1,
            InputState::MultiLine { lines } => lines as u16,
            InputState::Completion { lines } => 1 + lines as u16,
            InputState::Pager => self.rows,  // Full screen
            InputState::Busy => 0,
        };
        lines.max(MIN_SHELL_LINES as u16)
    }

    /// Shell region start row (1-indexed)
    fn shell_region_start(&self) -> u16 {
        if !self.log_split {
            return 1;
        }
        self.log_region_lines() + 2  // +1 for separator, +1 for 1-indexed
    }

    /// Setup screen regions
    fn setup_regions(&self) {
        if !self.log_split || matches!(self.shell_state, InputState::Pager) {
            // Full screen for shell
            ansi::reset_scroll_region();
            return;
        }

        let log_end = self.log_region_lines();
        let shell_start = self.shell_region_start();

        // Set log region scroll area
        ansi::set_scroll_region(1, log_end);

        // Draw separator
        ansi::goto(log_end + 1, 1);
        ansi::set_dim();
        for _ in 0..self.cols {
            let _ = syscall::write(syscall::STDOUT, b"-");
        }
        ansi::reset_style();

        // Move to shell region
        ansi::goto(shell_start, 1);
    }

    /// Write a log line to the log region
    fn write_log(&mut self, line: &[u8]) {
        if !self.log_split {
            return;  // Logs disabled
        }

        // Save shell cursor
        ansi::save_cursor();

        // Switch to log region
        let log_end = self.log_region_lines();
        ansi::set_scroll_region(1, log_end);

        // Go to bottom of log region and scroll if needed
        ansi::goto(log_end, 1);
        ansi::clear_line();

        // Write the log line
        let _ = syscall::write(syscall::STDOUT, line);
        let _ = syscall::write(syscall::STDOUT, b"\r\n");

        // Restore shell region
        let shell_start = self.shell_region_start();
        ansi::set_scroll_region(shell_start, self.rows);

        // Restore cursor
        ansi::restore_cursor();
    }

    /// Write to shell region
    fn write_shell(&mut self, data: &[u8]) {
        // Just write directly - shell manages its own cursor
        let _ = syscall::write(syscall::STDOUT, data);
    }

    /// Handle input state change from shell
    fn set_shell_state(&mut self, state: InputState) {
        if self.shell_state == state {
            return;
        }

        self.shell_state = state;
        self.setup_regions();
    }

    /// Handle log split toggle
    fn set_log_split(&mut self, enabled: bool) {
        if self.log_split == enabled {
            return;
        }

        self.log_split = enabled;

        // Clear screen and redraw
        ansi::clear();
        ansi::goto(1, 1);

        if enabled {
            self.setup_regions();
        } else {
            ansi::reset_scroll_region();
        }
    }
}

/// Console server
struct ConsoleServer {
    console: Console,
    port: u32,
    /// Channel to logd (if connected)
    logd_channel: Option<u32>,
    /// Additional shell channels (for doorbell messages)
    shell_extra_channels: [Option<u32>; 4],
}

impl ConsoleServer {
    fn new(uart_fd: u32) -> Option<Self> {
        // Register console port
        let port = syscall::port_register(b"console");
        if port < 0 {
            println!("consoled: failed to register port");
            return None;
        }

        Some(Self {
            console: Console::new(uart_fd),
            port: port as u32,
            logd_channel: None,
            shell_extra_channels: [None; 4],
        })
    }

    /// Connect to logd and register as a sink
    fn connect_logd(&mut self) -> bool {
        let ch = syscall::port_connect(b"logd");
        if ch < 0 {
            return false;
        }
        let channel = ch as u32;

        // Send register request
        let request = LogdRequest::Register;
        let mut buf = [0u8; 8];
        if let Ok(len) = request.serialize(&mut buf) {
            if syscall::send(channel, &buf[..len]) < 0 {
                syscall::channel_close(channel);
                return false;
            }
        }

        // Wait for OK response
        let mut resp_buf = [0u8; 8];
        let n = syscall::receive(channel, &mut resp_buf);
        if n > 0 {
            if let Ok((LogdEvent::Ok, _)) = LogdEvent::deserialize(&resp_buf[..n as usize]) {
                // Subscribe to IpcReady events for this channel (kevent API)
                let _ = kevent_subscribe(EventFilter::Ipc(channel));
                self.logd_channel = Some(channel);
                return true;
            }
        }

        syscall::channel_close(channel);
        false
    }

    fn run(&mut self) {
        // Reset terminal state first - ensures clean state on remote terminal
        // ESC c = Full Reset (RIS), ESC[!p = Soft Reset (DECSTR)
        let _ = syscall::write(syscall::STDOUT, b"\x1bc");  // Full terminal reset
        let _ = syscall::write(syscall::STDOUT, b"\x1b[!p"); // Soft reset (DECSTR)
        let _ = syscall::write(syscall::STDOUT, b"\x1b[?25h"); // Show cursor

        // Query terminal size before setting up regions
        if let Some((cols, rows)) = ansi::query_screen_size() {
            self.console.cols = cols;
            self.console.rows = rows;
        }

        // Initial setup
        ansi::clear();
        self.console.setup_regions();

        // Show startup message in log region with detected dimensions
        let mut msg = [0u8; 64];
        let len = format_startup_msg(&mut msg, self.console.cols, self.console.rows);
        self.console.write_log(&msg[..len]);

        // Subscribe to IPC events for our port (new connections) - kevent API
        let _ = kevent_subscribe(EventFilter::Ipc(self.port));

        // Subscribe to FdReadable for UART input (fd 0 = STDIN) - kevent API
        let _ = kevent_subscribe(EventFilter::Read(syscall::STDIN));

        // Subscribe to Timer events (for recurring logd reconnect)
        let _ = kevent_subscribe(EventFilter::Timer(0));

        // Try to connect to logd
        if self.connect_logd() {
            self.console.write_log(b"\x1b[2mconsoled: connected to logd\x1b[0m");
        } else {
            self.console.write_log(b"\x1b[33mconsoled: logd not available\x1b[0m");
        }

        // Set up recurring timer for logd reconnect attempts (every 5 seconds)
        // kevent_timer(id, interval_ns, initial_ns) - id=1, recurring every 5s
        let _ = kevent_timer(1, 5_000_000_000, 5_000_000_000);

        let mut events = [Event::empty(); 8];  // Batch receive up to 8 events
        let mut buf = [0u8; 256];
        let mut log_buf = [0u8; MAX_LOG_RECORD + 8];

        loop {
            // Wait for events (blocking, batch receive) - kevent API
            let count = match kevent_wait(&mut events, u64::MAX) {
                Ok(n) => n,
                Err(_) => {
                    syscall::yield_now();
                    continue;
                }
            };

            // Process all received events
            for event in &events[..count] {
                match event.event_type {
                    event_type::FD_READABLE => {
                        // UART has data available - read and forward to shell
                        self.handle_uart_input(&mut buf);
                    }
                    event_type::IPC_READY => {
                        let channel = event.data as u32;

                        // Check if this is a new connection on our port
                        if channel == self.port {
                            let ch = syscall::port_accept(self.port);
                            if ch >= 0 {
                                let new_ch = ch as u32;

                                // Is this the first shell connection or an extra one?
                                if self.console.shell_channel.is_none() {
                                    // First connection - this is the main shell channel
                                    self.console.shell_channel = Some(new_ch);
                                    self.console.write_log(b"\x1b[2mconsoled: shell connected\x1b[0m");

                                    // Subscribe to events for shell channel (kevent API)
                                    let _ = kevent_subscribe(EventFilter::Ipc(new_ch));

                                    // Create input ring for keyboard input → shell
                                    if let Ok(ring) = ByteRing::create(INPUT_RING_SIZE) {
                                        // Allow shell to map the ring
                                        let shell_pid = syscall::channel_get_peer(new_ch);
                                        if shell_pid > 0 {
                                            let _ = ring.allow(shell_pid as u32);
                                        }

                                        // Send SetupInputRing to shell
                                        let setup = ConsoleResponse::SetupInputRing {
                                            shmem_id: ring.shmem_id(),
                                        };
                                        let mut ring_buf = [0u8; 16];
                                        if let Ok(len) = setup.serialize(&mut ring_buf) {
                                            let _ = syscall::send(new_ch, &ring_buf[..len]);
                                        }

                                        self.console.shell_input_ring = Some(ring);
                                        self.console.write_log(b"\x1b[2mconsoled: input ring created\x1b[0m");
                                    }

                                    // Send ready response
                                    let ready = ConsoleResponse::Ready {
                                        cols: self.console.cols,
                                        rows: self.console.rows,
                                        log_split: self.console.log_split,
                                    };
                                    let mut resp_buf = [0u8; 16];
                                    if let Ok(len) = ready.serialize(&mut resp_buf) {
                                        let _ = syscall::send(new_ch, &resp_buf[..len]);
                                    }
                                } else {
                                    // Extra connection (likely for doorbell) - track it
                                    for slot in &mut self.shell_extra_channels {
                                        if slot.is_none() {
                                            *slot = Some(new_ch);
                                            let _ = kevent_subscribe(EventFilter::Ipc(new_ch));
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        // Check if this is from logd
                        else if Some(channel) == self.logd_channel {
                            self.handle_logd_message(&mut log_buf);
                        }
                        // Check if this is from shell (main or extra channel)
                        else if Some(channel) == self.console.shell_channel
                            || self.shell_extra_channels.contains(&Some(channel))
                        {
                            let n = syscall::receive_nonblock(channel, &mut buf);
                            if n > 0 {
                                if let Ok((request, _)) = ConsoleRequest::deserialize(&buf[..n as usize]) {
                                    self.handle_request(channel, request);
                                }
                            } else if n < 0 && n != -11 {
                                // Channel closed
                                if Some(channel) == self.console.shell_channel {
                                    self.console.shell_channel = None;
                                    self.console.write_log(b"\x1b[2mconsoled: shell disconnected\x1b[0m");
                                } else {
                                    // Remove from extra channels
                                    for slot in &mut self.shell_extra_channels {
                                        if *slot == Some(channel) {
                                            *slot = None;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    event_type::TIMER => {
                        // Recurring timer fired - retry logd connection if needed
                        // (timer auto-rearms due to kevent_timer with interval > 0)
                        if self.logd_channel.is_none() {
                            if self.connect_logd() {
                                self.console.write_log(b"\x1b[2mconsoled: connected to logd\x1b[0m");
                            }
                        }

                        // Check for terminal resize
                        self.check_terminal_resize();
                    }
                    _ => {}
                }
            }
        }
    }

    /// Handle UART input - read and forward to shell
    fn handle_uart_input(&mut self, buf: &mut [u8]) {
        // Read available data from UART
        let n = syscall::read(self.console.uart_fd, buf);
        if n > 0 {
            let data = &buf[..n as usize];

            // Try to use input ring if available
            if let Some(ring) = &self.console.shell_input_ring {
                // Write to ring
                let mut offset = 0;
                while offset < data.len() {
                    let written = ring.write(&data[offset..]);
                    if written == 0 {
                        // Ring full - send doorbell and break (don't block)
                        self.send_input_doorbell();
                        break;
                    }
                    offset += written;
                }
                // Send doorbell to notify shell
                self.send_input_doorbell();
            } else if let Some(shell_ch) = self.console.shell_channel {
                // Fallback to legacy IPC
                let input = ConsoleResponse::input(data);
                let mut resp_buf = [0u8; 128];
                if let Ok(len) = input.serialize(&mut resp_buf) {
                    let _ = syscall::send(shell_ch, &resp_buf[..len]);
                }
            }
        }
    }

    /// Send input doorbell to wake shell
    fn send_input_doorbell(&self) {
        if let Some(shell_ch) = self.console.shell_channel {
            let doorbell = ConsoleResponse::Doorbell;
            let mut buf = [0u8; 4];
            if let Ok(len) = doorbell.serialize(&mut buf) {
                let _ = syscall::send(shell_ch, &buf[..len]);
            }
        }
    }

    /// Handle log messages from logd
    fn handle_logd_message(&mut self, buf: &mut [u8]) {
        // Drain all pending messages
        let Some(channel) = self.logd_channel else { return };

        loop {
            let n = syscall::receive_nonblock(channel, buf);
            if n <= 0 {
                if n < 0 && n != -11 {
                    // Channel error, disconnect
                    self.logd_channel = None;
                    self.console.write_log(b"\x1b[33mconsoled: logd disconnected\x1b[0m");
                }
                break;
            }

            if let Ok((event, _)) = LogdEvent::deserialize(&buf[..n as usize]) {
                match event {
                    LogdEvent::Log { data, len } => {
                        // Format and display the log record
                        let record = &data[..len as usize];
                        let mut text_buf = [0u8; 1024];
                        let text_len = format_record(record, &mut text_buf);
                        if text_len > 0 {
                            // Strip trailing newline
                            let mut display_len = text_len;
                            while display_len > 0 && (text_buf[display_len - 1] == b'\n' || text_buf[display_len - 1] == b'\r') {
                                display_len -= 1;
                            }
                            if display_len > 0 {
                                self.console.write_log(&text_buf[..display_len]);
                            }
                        }
                    }
                    LogdEvent::Ok => {}
                }
            }
        }
    }

    fn handle_request(&mut self, channel: u32, request: ConsoleRequest) {
        match request {
            ConsoleRequest::Write { data, len } => {
                self.console.write_shell(&data[..len as usize]);
                self.send_ok(channel);
            }
            ConsoleRequest::SetInputState(state) => {
                self.console.set_shell_state(state);
                self.send_ok(channel);
            }
            ConsoleRequest::SetLogSplit(enabled) => {
                self.console.set_log_split(enabled);
                self.send_ok(channel);
            }
            ConsoleRequest::SetLogLines(lines) => {
                self.console.log_lines = lines;
                self.console.setup_regions();
                self.send_ok(channel);
            }
            ConsoleRequest::Ready => {
                // Shell is ready - send current config
                let ready = ConsoleResponse::Ready {
                    cols: self.console.cols,
                    rows: self.console.rows,
                    log_split: self.console.log_split,
                };
                let mut buf = [0u8; 16];
                if let Ok(len) = ready.serialize(&mut buf) {
                    let _ = syscall::send(channel, &buf[..len]);
                }
            }
            ConsoleRequest::SetLogdConnected(connected) => {
                if connected {
                    // Reconnect to logd if not already connected
                    if self.logd_channel.is_none() {
                        if self.connect_logd() {
                            self.console.write_log(b"\x1b[2mconsoled: reconnected to logd\x1b[0m");
                        } else {
                            self.console.write_log(b"\x1b[33mconsoled: logd not available\x1b[0m");
                        }
                    }
                } else {
                    // Disconnect from logd
                    if let Some(ch) = self.logd_channel.take() {
                        // Unsubscribe from IPC events for this channel (kevent API)
                        let _ = userlib::kevent_unsubscribe(EventFilter::Ipc(ch));
                        // Close the channel
                        syscall::close(ch);
                        self.console.write_log(b"\x1b[2mconsoled: disconnected from logd\x1b[0m");
                    }
                }
                self.send_ok(channel);
            }
            ConsoleRequest::SetupOutputRing { shmem_id } => {
                // Shell created an output ring, map it for reading
                match ByteRing::map(shmem_id) {
                    Ok(ring) => {
                        self.console.shell_output_ring = Some(ring);
                        self.console.write_log(b"\x1b[2mconsoled: shell output ring mapped\x1b[0m");
                        self.send_ok(channel);
                    }
                    Err(_) => {
                        self.console.write_log(b"\x1b[31mconsoled: failed to map shell output ring\x1b[0m");
                        // Send error response
                        let err = ConsoleResponse::Error(-1);
                        let mut buf = [0u8; 4];
                        if let Ok(len) = err.serialize(&mut buf) {
                            let _ = syscall::send(channel, &buf[..len]);
                        }
                    }
                }
            }
            ConsoleRequest::Doorbell => {
                // Shell notified us that data is available in the output ring
                self.drain_shell_output_ring();
                self.send_ok(channel);
            }
            ConsoleRequest::QuerySize => {
                // Shell requested terminal size re-detection
                if let Some((cols, rows)) = ansi::query_screen_size() {
                    if cols != self.console.cols || rows != self.console.rows {
                        self.console.cols = cols;
                        self.console.rows = rows;
                        self.console.setup_regions();
                    }
                }
                // Send current size (whether changed or not)
                let resize = ConsoleResponse::Resize {
                    cols: self.console.cols,
                    rows: self.console.rows,
                };
                let mut buf = [0u8; 8];
                if let Ok(len) = resize.serialize(&mut buf) {
                    let _ = syscall::send(channel, &buf[..len]);
                }
            }
        }
    }

    /// Check for terminal resize and notify shell if changed
    fn check_terminal_resize(&mut self) {
        if let Some((new_cols, new_rows)) = ansi::query_screen_size() {
            if new_cols != self.console.cols || new_rows != self.console.rows {
                let old_cols = self.console.cols;
                let old_rows = self.console.rows;

                self.console.cols = new_cols;
                self.console.rows = new_rows;

                // Reconfigure regions for new size
                self.console.setup_regions();

                // Notify shell of resize
                if let Some(shell_ch) = self.console.shell_channel {
                    let resize = ConsoleResponse::Resize {
                        cols: new_cols,
                        rows: new_rows,
                    };
                    let mut buf = [0u8; 8];
                    if let Ok(len) = resize.serialize(&mut buf) {
                        let _ = syscall::send(shell_ch, &buf[..len]);
                    }
                }

                // Log the resize
                let mut msg = [0u8; 48];
                let len = format_resize_msg(&mut msg, old_cols, old_rows, new_cols, new_rows);
                self.console.write_log(&msg[..len]);
            }
        }
    }

    /// Drain data from shell output ring and write to display
    fn drain_shell_output_ring(&mut self) {
        let mut buf = [0u8; 512];

        // Read all available data from the ring
        loop {
            // Separate the borrow of ring from the borrow of console
            let n = {
                let Some(ring) = &self.console.shell_output_ring else { return };
                ring.read(&mut buf)
            };
            if n == 0 {
                break;
            }
            self.console.write_shell(&buf[..n]);
        }
    }

    fn send_ok(&self, channel: u32) {
        let ok = ConsoleResponse::Ok;
        let mut buf = [0u8; 4];
        if let Ok(len) = ok.serialize(&mut buf) {
            let _ = syscall::send(channel, &buf[..len]);
        }
    }
}

/// Format startup message with terminal dimensions
fn format_startup_msg(buf: &mut [u8], cols: u16, rows: u16) -> usize {
    // "\x1b[1;36mconsoled\x1b[0m: started (COLSxROWS)"
    let prefix = b"\x1b[1;36mconsoled\x1b[0m: started (";
    let suffix = b")";

    let mut i = 0;
    for &b in prefix {
        if i < buf.len() {
            buf[i] = b;
            i += 1;
        }
    }

    // Format cols
    i += format_u16_into(&mut buf[i..], cols);

    if i < buf.len() {
        buf[i] = b'x';
        i += 1;
    }

    // Format rows
    i += format_u16_into(&mut buf[i..], rows);

    for &b in suffix {
        if i < buf.len() {
            buf[i] = b;
            i += 1;
        }
    }

    i
}

/// Format resize message
fn format_resize_msg(buf: &mut [u8], old_cols: u16, old_rows: u16, new_cols: u16, new_rows: u16) -> usize {
    // "\x1b[2mconsoled: resize OLDxOLD -> NEWxNEW\x1b[0m"
    let prefix = b"\x1b[2mconsoled: resize ";
    let arrow = b" -> ";
    let suffix = b"\x1b[0m";

    let mut i = 0;
    for &b in prefix {
        if i < buf.len() { buf[i] = b; i += 1; }
    }
    i += format_u16_into(&mut buf[i..], old_cols);
    if i < buf.len() { buf[i] = b'x'; i += 1; }
    i += format_u16_into(&mut buf[i..], old_rows);
    for &b in arrow {
        if i < buf.len() { buf[i] = b; i += 1; }
    }
    i += format_u16_into(&mut buf[i..], new_cols);
    if i < buf.len() { buf[i] = b'x'; i += 1; }
    i += format_u16_into(&mut buf[i..], new_rows);
    for &b in suffix {
        if i < buf.len() { buf[i] = b; i += 1; }
    }
    i
}

/// Format u16 into buffer, returns bytes written
fn format_u16_into(buf: &mut [u8], n: u16) -> usize {
    if n == 0 {
        if !buf.is_empty() {
            buf[0] = b'0';
        }
        return 1;
    }

    let mut tmp = [0u8; 5];
    let mut val = n;
    let mut len = 0;
    while val > 0 {
        tmp[len] = b'0' + (val % 10) as u8;
        val /= 10;
        len += 1;
    }

    for i in 0..len {
        if i < buf.len() {
            buf[i] = tmp[len - 1 - i];
        }
    }
    len
}

#[unsafe(no_mangle)]
fn main() {
    // For now, use stdout/stdin directly
    // TODO: Open UART device via scheme once devd provides it
    let uart_fd = syscall::STDOUT;

    println!("consoled: starting...");

    let mut server = match ConsoleServer::new(uart_fd) {
        Some(s) => s,
        None => {
            println!("consoled: failed to initialize");
            syscall::exit(1);
        }
    };

    server.run();
}
