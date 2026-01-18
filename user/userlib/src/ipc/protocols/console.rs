//! Console Protocol
//!
//! Type-safe IPC protocol for console I/O with region management.
//!
//! The console service (consoled) owns the display device (UART, SSH, etc.)
//! and manages screen regions for logs and shell interaction.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────┐
//! │  Log Region (scrolling)     │ ← klog, ktrace, ulog, utrace
//! ├─────────────────────────────┤
//! │  Shell Region (dynamic)     │ ← shell I/O via this protocol
//! │  > command_                 │
//! └─────────────────────────────┘
//! ```
//!
//! Shell sends InputState updates so consoled can adjust regions dynamically.

use super::super::error::{IpcError, IpcResult};
use super::super::protocol::{Protocol, Message};

/// Console service protocol
pub struct ConsoleProtocol;

impl Protocol for ConsoleProtocol {
    type Request = ConsoleRequest;
    type Response = ConsoleResponse;
    const PORT_NAME: &'static [u8] = b"console";
}

/// Maximum data in a single write request (shell → console)
pub const MAX_WRITE_SIZE: usize = 256;

/// Maximum data in a single input event (console → shell)
pub const MAX_INPUT_SIZE: usize = 256;

/// Command codes
const CMD_WRITE: u8 = 1;
const CMD_SET_INPUT_STATE: u8 = 2;
const CMD_SET_LOG_SPLIT: u8 = 3;
const CMD_READY: u8 = 4;
const CMD_SET_LOG_LINES: u8 = 5;
const CMD_SET_LOGD_CONNECTED: u8 = 6;
const CMD_SETUP_OUTPUT_RING: u8 = 7;  // Shell sends output ring shmem_id
const CMD_DOORBELL: u8 = 8;           // Notification: data available in ring
const CMD_QUERY_SIZE: u8 = 9;         // Request terminal size re-detection

/// Event codes (console → shell)
const EVT_INPUT: u8 = 1;
const EVT_RESIZE: u8 = 2;
const EVT_READY: u8 = 3;
const EVT_SETUP_INPUT_RING: u8 = 4;   // Consoled sends input ring shmem_id
const EVT_DOORBELL: u8 = 5;           // Notification: input available in ring

/// Shell's input state - tells console how much space is needed
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputState {
    /// Normal single-line prompt
    SingleLine,
    /// Multi-line editing (here-doc, paste)
    MultiLine { lines: u8 },
    /// Showing completions
    Completion { lines: u8 },
    /// Paging through output (like less)
    Pager,
    /// Shell is busy running a command
    Busy,
}

impl InputState {
    fn to_byte(self) -> u8 {
        match self {
            InputState::SingleLine => 0,
            InputState::MultiLine { .. } => 1,
            InputState::Completion { .. } => 2,
            InputState::Pager => 3,
            InputState::Busy => 4,
        }
    }

    fn lines(self) -> u8 {
        match self {
            InputState::SingleLine => 1,
            InputState::MultiLine { lines } => lines,
            InputState::Completion { lines } => lines,
            InputState::Pager => 0,  // Full screen
            InputState::Busy => 0,
        }
    }

    fn from_bytes(state: u8, lines: u8) -> Self {
        match state {
            0 => InputState::SingleLine,
            1 => InputState::MultiLine { lines },
            2 => InputState::Completion { lines },
            3 => InputState::Pager,
            4 => InputState::Busy,
            _ => InputState::SingleLine,
        }
    }
}

/// Console request messages (shell → console)
#[derive(Debug, Clone)]
pub enum ConsoleRequest {
    /// Write data to shell region (legacy - use ring instead)
    Write { data: [u8; MAX_WRITE_SIZE], len: u8 },
    /// Update input state (console adjusts regions)
    SetInputState(InputState),
    /// Enable/disable log split
    SetLogSplit(bool),
    /// Set specific number of log lines (0 = auto)
    SetLogLines(u8),
    /// Shell is ready after init
    Ready,
    /// Connect/disconnect from logd
    /// true = connect (or reconnect), false = disconnect
    SetLogdConnected(bool),
    /// Setup output ring buffer (shell → consoled)
    /// Shell creates ByteRing, sends shmem_id for consoled to map
    SetupOutputRing { shmem_id: u32 },
    /// Doorbell: notify that data is available in output ring
    Doorbell,
    /// Request terminal size re-detection (returns Resize response)
    QuerySize,
}

impl ConsoleRequest {
    /// Create a write request from a byte slice
    pub fn write(data: &[u8]) -> Self {
        let mut buf = [0u8; MAX_WRITE_SIZE];
        let len = data.len().min(MAX_WRITE_SIZE);
        buf[..len].copy_from_slice(&data[..len]);
        ConsoleRequest::Write { data: buf, len: len as u8 }
    }
}

impl Message for ConsoleRequest {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        match self {
            ConsoleRequest::Write { data, len } => {
                let total = 2 + *len as usize;
                if buf.len() < total {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_WRITE;
                buf[1] = *len;
                buf[2..2 + *len as usize].copy_from_slice(&data[..*len as usize]);
                Ok(total)
            }
            ConsoleRequest::SetInputState(state) => {
                if buf.len() < 3 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_SET_INPUT_STATE;
                buf[1] = state.to_byte();
                buf[2] = state.lines();
                Ok(3)
            }
            ConsoleRequest::SetLogSplit(enabled) => {
                if buf.len() < 2 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_SET_LOG_SPLIT;
                buf[1] = if *enabled { 1 } else { 0 };
                Ok(2)
            }
            ConsoleRequest::SetLogLines(lines) => {
                if buf.len() < 2 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_SET_LOG_LINES;
                buf[1] = *lines;
                Ok(2)
            }
            ConsoleRequest::Ready => {
                if buf.is_empty() {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_READY;
                Ok(1)
            }
            ConsoleRequest::SetLogdConnected(connected) => {
                if buf.len() < 2 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_SET_LOGD_CONNECTED;
                buf[1] = if *connected { 1 } else { 0 };
                Ok(2)
            }
            ConsoleRequest::SetupOutputRing { shmem_id } => {
                if buf.len() < 5 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_SETUP_OUTPUT_RING;
                buf[1..5].copy_from_slice(&shmem_id.to_le_bytes());
                Ok(5)
            }
            ConsoleRequest::Doorbell => {
                if buf.is_empty() {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_DOORBELL;
                Ok(1)
            }
            ConsoleRequest::QuerySize => {
                if buf.is_empty() {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_QUERY_SIZE;
                Ok(1)
            }
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }

        match buf[0] {
            CMD_WRITE => {
                if buf.len() < 2 {
                    return Err(IpcError::Truncated);
                }
                let len = buf[1] as usize;
                if buf.len() < 2 + len {
                    return Err(IpcError::Truncated);
                }
                let mut data = [0u8; MAX_WRITE_SIZE];
                data[..len].copy_from_slice(&buf[2..2 + len]);
                Ok((ConsoleRequest::Write { data, len: len as u8 }, 2 + len))
            }
            CMD_SET_INPUT_STATE => {
                if buf.len() < 3 {
                    return Err(IpcError::Truncated);
                }
                let state = InputState::from_bytes(buf[1], buf[2]);
                Ok((ConsoleRequest::SetInputState(state), 3))
            }
            CMD_SET_LOG_SPLIT => {
                if buf.len() < 2 {
                    return Err(IpcError::Truncated);
                }
                Ok((ConsoleRequest::SetLogSplit(buf[1] != 0), 2))
            }
            CMD_SET_LOG_LINES => {
                if buf.len() < 2 {
                    return Err(IpcError::Truncated);
                }
                Ok((ConsoleRequest::SetLogLines(buf[1]), 2))
            }
            CMD_READY => {
                Ok((ConsoleRequest::Ready, 1))
            }
            CMD_SET_LOGD_CONNECTED => {
                if buf.len() < 2 {
                    return Err(IpcError::Truncated);
                }
                Ok((ConsoleRequest::SetLogdConnected(buf[1] != 0), 2))
            }
            CMD_SETUP_OUTPUT_RING => {
                if buf.len() < 5 {
                    return Err(IpcError::Truncated);
                }
                let shmem_id = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
                Ok((ConsoleRequest::SetupOutputRing { shmem_id }, 5))
            }
            CMD_DOORBELL => {
                Ok((ConsoleRequest::Doorbell, 1))
            }
            CMD_QUERY_SIZE => {
                Ok((ConsoleRequest::QuerySize, 1))
            }
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    fn serialized_size(&self) -> usize {
        match self {
            ConsoleRequest::Write { len, .. } => 2 + *len as usize,
            ConsoleRequest::SetInputState(_) => 3,
            ConsoleRequest::SetLogSplit(_) => 2,
            ConsoleRequest::SetLogLines(_) => 2,
            ConsoleRequest::Ready => 1,
            ConsoleRequest::SetLogdConnected(_) => 2,
            ConsoleRequest::SetupOutputRing { .. } => 5,
            ConsoleRequest::Doorbell => 1,
            ConsoleRequest::QuerySize => 1,
        }
    }
}

/// Console response/event messages (console → shell)
#[derive(Debug, Clone)]
pub enum ConsoleResponse {
    /// Acknowledgment
    Ok,
    /// User input from keyboard (can be large for paste) - legacy, use input ring
    Input { data: [u8; MAX_INPUT_SIZE], len: u16 },
    /// Terminal size changed
    Resize { cols: u16, rows: u16 },
    /// Console ready, initial configuration
    Ready { cols: u16, rows: u16, log_split: bool },
    /// Error
    Error(i8),
    /// Setup input ring buffer (consoled → shell)
    /// Consoled creates ByteRing for keyboard input, sends shmem_id for shell to map
    SetupInputRing { shmem_id: u32 },
    /// Doorbell: notify that input is available in input ring
    Doorbell,
}

impl ConsoleResponse {
    /// Create an input response from a byte slice
    pub fn input(data: &[u8]) -> Self {
        let mut buf = [0u8; MAX_INPUT_SIZE];
        let len = data.len().min(MAX_INPUT_SIZE);
        buf[..len].copy_from_slice(&data[..len]);
        ConsoleResponse::Input { data: buf, len: len as u16 }
    }
}

impl Message for ConsoleResponse {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        match self {
            ConsoleResponse::Ok => {
                if buf.is_empty() {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = 0;
                Ok(1)
            }
            ConsoleResponse::Input { data, len } => {
                // Format: cmd(1) + len_lo(1) + len_hi(1) + data(len)
                let total = 3 + *len as usize;
                if buf.len() < total {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = EVT_INPUT;
                buf[1] = (*len & 0xFF) as u8;
                buf[2] = (*len >> 8) as u8;
                buf[3..total].copy_from_slice(&data[..*len as usize]);
                Ok(total)
            }
            ConsoleResponse::Resize { cols, rows } => {
                if buf.len() < 5 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = EVT_RESIZE;
                buf[1] = (*cols & 0xFF) as u8;
                buf[2] = (*cols >> 8) as u8;
                buf[3] = (*rows & 0xFF) as u8;
                buf[4] = (*rows >> 8) as u8;
                Ok(5)
            }
            ConsoleResponse::Ready { cols, rows, log_split } => {
                if buf.len() < 6 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = EVT_READY;
                buf[1] = (*cols & 0xFF) as u8;
                buf[2] = (*cols >> 8) as u8;
                buf[3] = (*rows & 0xFF) as u8;
                buf[4] = (*rows >> 8) as u8;
                buf[5] = if *log_split { 1 } else { 0 };
                Ok(6)
            }
            ConsoleResponse::Error(code) => {
                if buf.is_empty() {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = 0x80 | (*code as u8 & 0x7F);
                Ok(1)
            }
            ConsoleResponse::SetupInputRing { shmem_id } => {
                if buf.len() < 5 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = EVT_SETUP_INPUT_RING;
                buf[1..5].copy_from_slice(&shmem_id.to_le_bytes());
                Ok(5)
            }
            ConsoleResponse::Doorbell => {
                if buf.is_empty() {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = EVT_DOORBELL;
                Ok(1)
            }
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }

        match buf[0] {
            0 => Ok((ConsoleResponse::Ok, 1)),
            EVT_INPUT => {
                if buf.len() < 3 {
                    return Err(IpcError::Truncated);
                }
                let len = (buf[1] as u16) | ((buf[2] as u16) << 8);
                let len_usize = len as usize;
                if buf.len() < 3 + len_usize {
                    return Err(IpcError::Truncated);
                }
                let mut data = [0u8; MAX_INPUT_SIZE];
                let copy_len = len_usize.min(MAX_INPUT_SIZE);
                data[..copy_len].copy_from_slice(&buf[3..3 + copy_len]);
                Ok((ConsoleResponse::Input { data, len }, 3 + len_usize))
            }
            EVT_RESIZE => {
                if buf.len() < 5 {
                    return Err(IpcError::Truncated);
                }
                let cols = buf[1] as u16 | ((buf[2] as u16) << 8);
                let rows = buf[3] as u16 | ((buf[4] as u16) << 8);
                Ok((ConsoleResponse::Resize { cols, rows }, 5))
            }
            EVT_READY => {
                if buf.len() < 6 {
                    return Err(IpcError::Truncated);
                }
                let cols = buf[1] as u16 | ((buf[2] as u16) << 8);
                let rows = buf[3] as u16 | ((buf[4] as u16) << 8);
                let log_split = buf[5] != 0;
                Ok((ConsoleResponse::Ready { cols, rows, log_split }, 6))
            }
            EVT_SETUP_INPUT_RING => {
                if buf.len() < 5 {
                    return Err(IpcError::Truncated);
                }
                let shmem_id = u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
                Ok((ConsoleResponse::SetupInputRing { shmem_id }, 5))
            }
            EVT_DOORBELL => {
                Ok((ConsoleResponse::Doorbell, 1))
            }
            x if x & 0x80 != 0 => {
                let code = (x & 0x7F) as i8;
                Ok((ConsoleResponse::Error(code), 1))
            }
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    fn serialized_size(&self) -> usize {
        match self {
            ConsoleResponse::Ok => 1,
            ConsoleResponse::Input { len, .. } => 3 + *len as usize,
            ConsoleResponse::Resize { .. } => 5,
            ConsoleResponse::Ready { .. } => 6,
            ConsoleResponse::Error(_) => 1,
            ConsoleResponse::SetupInputRing { .. } => 5,
            ConsoleResponse::Doorbell => 1,
        }
    }
}

/// Convenience client for console operations (used by shell)
pub struct ConsoleClient {
    channel: u32,
}

impl ConsoleClient {
    /// Connect to console service
    pub fn connect() -> IpcResult<Self> {
        let ch = crate::syscall::port_connect(ConsoleProtocol::PORT_NAME);
        if ch < 0 {
            return Err(IpcError::from_errno(ch as i64));
        }
        Ok(Self { channel: ch as u32 })
    }

    /// Send ready signal
    pub fn ready(&self) -> IpcResult<ConsoleResponse> {
        self.send(&ConsoleRequest::Ready)
    }

    /// Write data to console
    pub fn write(&self, data: &[u8]) -> IpcResult<()> {
        // May need to split into multiple writes
        let mut offset = 0;
        while offset < data.len() {
            let chunk_len = (data.len() - offset).min(MAX_WRITE_SIZE);
            let request = ConsoleRequest::write(&data[offset..offset + chunk_len]);
            self.send(&request)?;
            offset += chunk_len;
        }
        Ok(())
    }

    /// Update input state
    pub fn set_input_state(&self, state: InputState) -> IpcResult<()> {
        self.send(&ConsoleRequest::SetInputState(state))?;
        Ok(())
    }

    /// Enable/disable log split
    pub fn set_log_split(&self, enabled: bool) -> IpcResult<()> {
        self.send(&ConsoleRequest::SetLogSplit(enabled))?;
        Ok(())
    }

    /// Set log region size
    pub fn set_log_lines(&self, lines: u8) -> IpcResult<()> {
        self.send(&ConsoleRequest::SetLogLines(lines))?;
        Ok(())
    }

    /// Connect/disconnect from logd
    /// When disconnected, no log messages will be received (reduces CPU/memory usage)
    /// When reconnected, only new messages will appear (missed messages are lost)
    pub fn set_logd_connected(&self, connected: bool) -> IpcResult<()> {
        self.send(&ConsoleRequest::SetLogdConnected(connected))?;
        Ok(())
    }

    /// Query terminal size (triggers re-detection)
    /// Returns (cols, rows) on success
    pub fn query_size(&self) -> IpcResult<(u16, u16)> {
        let response = self.send(&ConsoleRequest::QuerySize)?;
        match response {
            ConsoleResponse::Resize { cols, rows } => Ok((cols, rows)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Receive an event (blocking)
    pub fn receive(&self) -> IpcResult<ConsoleResponse> {
        let mut buf = [0u8; 128];
        let n = crate::syscall::receive(self.channel, &mut buf);
        if n <= 0 {
            return Err(IpcError::from_errno(n as i64));
        }
        let (response, _) = ConsoleResponse::deserialize(&buf[..n as usize])?;
        Ok(response)
    }

    /// Try to receive an event (non-blocking)
    pub fn try_receive(&self) -> IpcResult<Option<ConsoleResponse>> {
        let mut buf = [0u8; 128];
        let n = crate::syscall::receive_nonblock(self.channel, &mut buf);
        if n == -11 {  // EAGAIN
            return Ok(None);
        }
        if n <= 0 {
            return Err(IpcError::from_errno(n as i64));
        }
        let (response, _) = ConsoleResponse::deserialize(&buf[..n as usize])?;
        Ok(Some(response))
    }

    fn send(&self, request: &ConsoleRequest) -> IpcResult<ConsoleResponse> {
        let mut buf = [0u8; 256];
        let len = request.serialize(&mut buf)?;
        let sent = crate::syscall::send(self.channel, &buf[..len]);
        if sent < 0 {
            return Err(IpcError::from_errno(sent as i64));
        }

        // Wait for response
        let n = crate::syscall::receive(self.channel, &mut buf);
        if n <= 0 {
            return Err(IpcError::from_errno(n as i64));
        }
        let (response, _) = ConsoleResponse::deserialize(&buf[..n as usize])?;
        Ok(response)
    }
}

impl Drop for ConsoleClient {
    fn drop(&mut self) {
        crate::syscall::channel_close(self.channel);
    }
}
