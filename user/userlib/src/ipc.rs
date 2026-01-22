//! IPC - Unified IPC using 5-syscall interface
//!
//! Everything is an object. Everything uses: open, read, write, map, close.

use crate::error::{SysError, SysResult};
use crate::syscall::{Handle, ObjectType, open, read, write, close, channel_pair};

// Re-export handle type
pub type ObjHandle = Handle;

// Re-export Mux types from abi crate
pub use abi::{MuxFilter, MuxEvent};

// ============================================================================
// Channel - Bidirectional IPC
// ============================================================================

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChannelState {
    Open,
    HalfClosed,
    Closed,
}

pub struct Channel {
    handle: ObjHandle,
    state: ChannelState,
}

impl Channel {
    /// Create a channel pair
    pub fn pair() -> SysResult<(Self, Self)> {
        let (handle_a, handle_b) = channel_pair()?;
        Ok((
            Self { handle: handle_a, state: ChannelState::Open },
            Self { handle: handle_b, state: ChannelState::Open },
        ))
    }

    /// Connect to a named port
    pub fn connect(port_name: &[u8]) -> SysResult<Self> {
        let handle = open(ObjectType::Channel, port_name)?;
        Ok(Self { handle, state: ChannelState::Open })
    }

    pub fn handle(&self) -> ObjHandle { self.handle }
    pub fn state(&self) -> ChannelState { self.state }

    pub fn send(&self, data: &[u8]) -> SysResult<()> {
        // Don't send on closed or half-closed channels
        if self.state == ChannelState::Closed || self.state == ChannelState::HalfClosed {
            return Err(SysError::ConnectionReset);
        }
        write(self.handle, data)?;
        Ok(())
    }

    pub fn recv(&mut self, buf: &mut [u8]) -> SysResult<usize> {
        if self.state == ChannelState::Closed {
            return Err(SysError::ConnectionReset);
        }
        match read(self.handle, buf) {
            Ok(n) => Ok(n),
            Err(SysError::ConnectionReset) => {
                self.state = ChannelState::HalfClosed;
                Err(SysError::ConnectionReset)
            }
            Err(e) => Err(e),
        }
    }
}

impl Drop for Channel {
    fn drop(&mut self) {
        if self.state != ChannelState::Closed {
            let _ = close(self.handle);
            self.state = ChannelState::Closed;
        }
    }
}

// ============================================================================
// Port - Named service endpoint
// ============================================================================

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortState {
    Listening,
    Closed,
}

pub struct Port {
    handle: ObjHandle,
    state: PortState,
}

impl Port {
    /// Register a named port
    pub fn register(name: &[u8]) -> SysResult<Self> {
        let handle = open(ObjectType::Port, name)?;
        Ok(Self { handle, state: PortState::Listening })
    }

    pub fn handle(&self) -> ObjHandle { self.handle }
    pub fn state(&self) -> PortState { self.state }

    /// Accept a connection (blocking)
    pub fn accept(&self) -> SysResult<Channel> {
        if self.state == PortState::Closed {
            return Err(SysError::BadFd);
        }
        // Kernel returns [handle: u32] (4 bytes) or [handle: u32, client_pid: u32] (8 bytes)
        let mut buf = [0u8; 8];
        let n = read(self.handle, &mut buf)?;
        if n >= 4 {
            let handle = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
            // buf[4..8] contains client_pid if n >= 8
            Ok(Channel {
                handle: Handle(handle),
                state: ChannelState::Open,
            })
        } else {
            Err(SysError::InvalidArgument)
        }
    }
}

impl Drop for Port {
    fn drop(&mut self) {
        if self.state != PortState::Closed {
            let _ = close(self.handle);
            self.state = PortState::Closed;
        }
    }
}

// ============================================================================
// Timer - Deadline-based timer
// ============================================================================

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerState {
    Disarmed,
    Armed,
    Fired,
    Closed,
}

pub struct Timer {
    handle: ObjHandle,
    state: TimerState,
}

impl Timer {
    pub fn new() -> SysResult<Self> {
        let handle = open(ObjectType::Timer, &[])?;
        Ok(Self { handle, state: TimerState::Disarmed })
    }

    pub fn handle(&self) -> ObjHandle { self.handle }
    pub fn state(&self) -> TimerState { self.state }

    pub fn set(&mut self, deadline_ns: u64) -> SysResult<()> {
        if self.state == TimerState::Closed {
            return Err(SysError::BadFd);
        }
        write(self.handle, &deadline_ns.to_le_bytes())?;
        self.state = TimerState::Armed;
        Ok(())
    }

    pub fn wait(&mut self) -> SysResult<u64> {
        if self.state == TimerState::Closed {
            return Err(SysError::BadFd);
        }
        let mut buf = [0u8; 8];
        read(self.handle, &mut buf)?;
        self.state = TimerState::Fired;
        Ok(u64::from_le_bytes(buf))
    }
}

impl Drop for Timer {
    fn drop(&mut self) {
        if self.state != TimerState::Closed {
            let _ = close(self.handle);
            self.state = TimerState::Closed;
        }
    }
}

// ============================================================================
// Mux - Event multiplexer
// ============================================================================

pub struct Mux {
    handle: ObjHandle,
}

impl Mux {
    pub fn new() -> SysResult<Self> {
        let handle = open(ObjectType::Mux, &[])?;
        Ok(Self { handle })
    }

    pub fn handle(&self) -> ObjHandle { self.handle }

    pub fn add(&self, target: ObjHandle, filter: MuxFilter) -> SysResult<()> {
        let mut buf = [0u8; 8];
        buf[0] = 0; // Add
        buf[1] = filter as u8;
        buf[4..8].copy_from_slice(&target.0.to_le_bytes());
        write(self.handle, &buf)?;
        Ok(())
    }

    pub fn remove(&self, target: ObjHandle) -> SysResult<()> {
        let mut buf = [0u8; 8];
        buf[0] = 1; // Remove
        buf[4..8].copy_from_slice(&target.0.to_le_bytes());
        write(self.handle, &buf)?;
        Ok(())
    }

    pub fn wait(&self) -> SysResult<MuxEvent> {
        let mut event = MuxEvent::empty();
        let buf = unsafe {
            core::slice::from_raw_parts_mut(
                &mut event as *mut MuxEvent as *mut u8,
                core::mem::size_of::<MuxEvent>(),
            )
        };
        read(self.handle, buf)?;
        Ok(event)
    }
}

impl Drop for Mux {
    fn drop(&mut self) {
        let _ = close(self.handle);
    }
}

// ============================================================================
// Process - Child process watch
// ============================================================================

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProcessState {
    Running,
    Exited(i32),
    Closed,
}

pub struct Process {
    handle: ObjHandle,
    state: ProcessState,
}

impl Process {
    pub fn watch(pid: u32) -> SysResult<Self> {
        let handle = open(ObjectType::Process, &pid.to_le_bytes())?;
        Ok(Self { handle, state: ProcessState::Running })
    }

    pub fn handle(&self) -> ObjHandle { self.handle }
    pub fn state(&self) -> ProcessState { self.state }

    pub fn wait(&mut self) -> SysResult<i32> {
        if let ProcessState::Exited(code) = self.state {
            return Ok(code);
        }
        if self.state == ProcessState::Closed {
            return Err(SysError::BadFd);
        }
        let mut buf = [0u8; 4];
        read(self.handle, &mut buf)?;
        let code = i32::from_le_bytes(buf);
        self.state = ProcessState::Exited(code);
        Ok(code)
    }
}

impl Drop for Process {
    fn drop(&mut self) {
        if self.state != ProcessState::Closed {
            let _ = close(self.handle);
            self.state = ProcessState::Closed;
        }
    }
}

// ============================================================================
// Message buffer helper
// ============================================================================

pub const MAX_MESSAGE_SIZE: usize = 4096;

pub struct Message {
    data: [u8; MAX_MESSAGE_SIZE],
    len: usize,
}

impl Message {
    pub const fn new() -> Self {
        Self { data: [0u8; MAX_MESSAGE_SIZE], len: 0 }
    }

    pub fn from_slice(data: &[u8]) -> Self {
        let mut msg = Self::new();
        let n = data.len().min(MAX_MESSAGE_SIZE);
        msg.data[..n].copy_from_slice(&data[..n]);
        msg.len = n;
        msg
    }

    pub fn data(&self) -> &[u8] { &self.data[..self.len] }
    pub fn buffer_mut(&mut self) -> &mut [u8] { &mut self.data }
    pub fn set_len(&mut self, len: usize) { self.len = len.min(MAX_MESSAGE_SIZE); }
    pub fn len(&self) -> usize { self.len }
    pub fn is_empty(&self) -> bool { self.len == 0 }
}

impl Default for Message {
    fn default() -> Self { Self::new() }
}

// ============================================================================
// Convenience functions - avoid Mux boilerplate
// ============================================================================

/// Wait for any of the given handles to become readable
///
/// Returns the index of the first ready handle.
/// Creates a temporary Mux internally.
pub fn wait_any(handles: &[ObjHandle]) -> SysResult<usize> {
    let mux = Mux::new()?;
    for &h in handles {
        mux.add(h, MuxFilter::Readable)?;
    }
    let event = mux.wait()?;
    // Find which index matched
    for (i, &h) in handles.iter().enumerate() {
        if h == event.handle {
            return Ok(i);
        }
    }
    Ok(0) // Fallback
}

/// Wait for a single handle to become readable
///
/// Simpler than creating a Mux for one handle.
pub fn wait_one(handle: ObjHandle) -> SysResult<()> {
    let mux = Mux::new()?;
    mux.add(handle, MuxFilter::Readable)?;
    mux.wait()?;
    Ok(())
}

/// Event loop builder - fluent API for setting up event handling
pub struct EventLoop {
    mux: Mux,
    handles: [Option<ObjHandle>; 16],
    count: usize,
}

impl EventLoop {
    /// Create a new event loop
    pub fn new() -> SysResult<Self> {
        Ok(Self {
            mux: Mux::new()?,
            handles: [None; 16],
            count: 0,
        })
    }

    /// Watch a handle for readability
    pub fn watch(&mut self, handle: ObjHandle) -> SysResult<&mut Self> {
        if self.count < 16 {
            self.mux.add(handle, MuxFilter::Readable)?;
            self.handles[self.count] = Some(handle);
            self.count += 1;
        }
        Ok(self)
    }

    /// Stop watching a handle
    pub fn unwatch(&mut self, handle: ObjHandle) -> SysResult<&mut Self> {
        self.mux.remove(handle)?;
        for slot in &mut self.handles {
            if *slot == Some(handle) {
                *slot = None;
                break;
            }
        }
        Ok(self)
    }

    /// Wait for next event, returns the ready handle
    pub fn wait(&self) -> SysResult<ObjHandle> {
        let event = self.mux.wait()?;
        Ok(event.handle)
    }

    /// Get underlying mux handle (for advanced use)
    pub fn mux_handle(&self) -> ObjHandle {
        self.mux.handle()
    }
}
