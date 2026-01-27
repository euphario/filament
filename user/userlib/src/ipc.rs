//! IPC - Unified IPC using 5-syscall interface
//!
//! Everything is an object. Everything uses: open, read, write, map, close.

use crate::error::{SysError, SysResult};
use crate::syscall::{Handle, ObjectType, open, read, write, map, close, channel_pair};

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
    /// Maximum active connections (0 = unlimited)
    max_connections: u8,
    /// Current active connection count
    active_connections: u8,
}

impl Port {
    /// Register a named port with unlimited connections
    pub fn register(name: &[u8]) -> SysResult<Self> {
        let handle = open(ObjectType::Port, name)?;
        Ok(Self {
            handle,
            state: PortState::Listening,
            max_connections: 0,
            active_connections: 0,
        })
    }

    /// Register a named port with a connection limit
    ///
    /// When active connections reach the limit, `accept()` returns WouldBlock.
    /// Caller must call `connection_closed()` when a channel closes to decrement
    /// the counter and allow new connections.
    pub fn with_limit(name: &[u8], max_connections: u8) -> SysResult<Self> {
        let handle = open(ObjectType::Port, name)?;
        Ok(Self {
            handle,
            state: PortState::Listening,
            max_connections,
            active_connections: 0,
        })
    }

    pub fn handle(&self) -> ObjHandle { self.handle }
    pub fn state(&self) -> PortState { self.state }
    pub fn active_connections(&self) -> u8 { self.active_connections }

    /// Notify that a connection was closed
    ///
    /// Must be called when a channel accepted from this port is closed,
    /// to allow new connections when a limit is set.
    pub fn connection_closed(&mut self) {
        self.active_connections = self.active_connections.saturating_sub(1);
    }

    /// Accept a connection (blocking)
    ///
    /// Returns WouldBlock if connection limit is reached.
    pub fn accept(&mut self) -> SysResult<Channel> {
        let (channel, _pid) = self.accept_with_pid()?;
        Ok(channel)
    }

    /// Accept a connection and return the client's PID
    ///
    /// Returns (Channel, client_pid) on success.
    /// Returns WouldBlock if connection limit is reached.
    pub fn accept_with_pid(&mut self) -> SysResult<(Channel, u32)> {
        if self.state == PortState::Closed {
            return Err(SysError::BadFd);
        }

        // Check connection limit
        if self.max_connections > 0 && self.active_connections >= self.max_connections {
            return Err(SysError::WouldBlock);
        }

        // Kernel returns [handle: u32, client_pid: u32] (8 bytes)
        let mut buf = [0u8; 8];
        let n = read(self.handle, &mut buf)?;
        if n >= 4 {
            let handle = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
            let client_pid = if n >= 8 {
                u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]])
            } else {
                0 // Unknown PID
            };
            self.active_connections = self.active_connections.saturating_add(1);
            Ok((Channel {
                handle: Handle(handle),
                state: ChannelState::Open,
            }, client_pid))
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
        let n = read(self.handle, buf)?;
        if n > 0 {
            return Ok(event);
        }
        // Kernel returned 0 events - return error instead of busy-looping
        Err(SysError::WouldBlock)
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

// ============================================================================
// Shmem - Shared memory
// ============================================================================

/// Shared memory region with DMA support
pub struct Shmem {
    handle: ObjHandle,
    shmem_id: u32,
    vaddr: u64,
    paddr: u64,
    size: usize,
}

impl Shmem {
    /// Create a new shared memory region of the given size
    pub fn create(size: usize) -> SysResult<Self> {
        // Use special shmem open that returns both handle and shmem_id
        let (handle, shmem_id) = crate::syscall::open_shmem_create(size)?;
        // After open, use map() to get the address
        let vaddr = map(handle, 0)?;
        // Query physical address via read (16-byte buffer returns paddr+size)
        let mut info = [0u8; 16];
        read(handle, &mut info)?;
        let paddr = u64::from_le_bytes([
            info[0], info[1], info[2], info[3],
            info[4], info[5], info[6], info[7],
        ]);
        Ok(Self {
            handle,
            shmem_id,
            vaddr,
            paddr,
            size,
        })
    }

    /// Open an existing shared memory region by ID
    pub fn open_existing(shmem_id: u32) -> SysResult<Self> {
        let handle = open(ObjectType::Shmem, &shmem_id.to_le_bytes())?;
        let vaddr = map(handle, 0)?;
        // Query physical address and size via read (16-byte buffer)
        let mut info = [0u8; 16];
        read(handle, &mut info)?;
        let paddr = u64::from_le_bytes([
            info[0], info[1], info[2], info[3],
            info[4], info[5], info[6], info[7],
        ]);
        let size = u64::from_le_bytes([
            info[8], info[9], info[10], info[11],
            info[12], info[13], info[14], info[15],
        ]) as usize;
        Ok(Self {
            handle,
            shmem_id,
            vaddr,
            paddr,
            size,
        })
    }

    pub fn handle(&self) -> ObjHandle { self.handle }
    pub fn shmem_id(&self) -> u32 { self.shmem_id }
    pub fn vaddr(&self) -> u64 { self.vaddr }
    pub fn paddr(&self) -> u64 { self.paddr }
    pub fn size(&self) -> usize { self.size }

    /// Get a pointer to the shared memory
    pub fn as_ptr(&self) -> *mut u8 {
        self.vaddr as *mut u8
    }

    /// Grant access to another process
    pub fn allow(&self, peer_pid: u32) -> SysResult<()> {
        let mut buf = [0u8; 5];
        buf[0] = 0; // ALLOW command
        buf[1..5].copy_from_slice(&peer_pid.to_le_bytes());
        write(self.handle, &buf)?;
        Ok(())
    }

    /// Make region public (accessible by any process)
    pub fn set_public(&self) -> SysResult<()> {
        let mut buf = [0u8; 1];
        buf[0] = 2; // SET_PUBLIC command
        write(self.handle, &buf)?;
        Ok(())
    }

    /// Wait for notification (blocking)
    pub fn wait(&self, timeout_ms: u32) -> SysResult<()> {
        let mut buf = [0u8; 4];
        buf.copy_from_slice(&timeout_ms.to_le_bytes());
        read(self.handle, &mut buf)?;
        Ok(())
    }

    /// Notify waiters
    pub fn notify(&self) -> SysResult<u32> {
        let mut buf = [0u8; 1];
        buf[0] = 1; // NOTIFY command
        let woken = write(self.handle, &buf)?;
        Ok(woken as u32)
    }
}

impl Drop for Shmem {
    fn drop(&mut self) {
        let _ = close(self.handle);
    }
}

// ============================================================================
// PciDevice - PCI configuration access
// ============================================================================

/// PCI device handle for configuration space access
pub struct PciDevice {
    handle: ObjHandle,
    bdf: u32,
}

impl PciDevice {
    /// Open a PCI device by BDF (bus/device/function)
    pub fn open(bdf: u32) -> SysResult<Self> {
        let handle = open(ObjectType::PciDevice, &bdf.to_le_bytes())?;
        Ok(Self { handle, bdf })
    }

    pub fn handle(&self) -> ObjHandle { self.handle }
    pub fn bdf(&self) -> u32 { self.bdf }

    /// Read from PCI configuration space
    pub fn config_read(&self, offset: u16, size: u8) -> SysResult<u32> {
        let mut params = [0u8; 3];
        params[0..2].copy_from_slice(&offset.to_le_bytes());
        params[2] = size;
        let value = read(self.handle, &mut params)?;
        Ok(value as u32)
    }

    /// Write to PCI configuration space
    pub fn config_write(&self, offset: u16, size: u8, value: u32) -> SysResult<()> {
        let mut params = [0u8; 7];
        params[0..2].copy_from_slice(&offset.to_le_bytes());
        params[2] = size;
        params[3..7].copy_from_slice(&value.to_le_bytes());
        write(self.handle, &params)?;
        Ok(())
    }
}

impl Drop for PciDevice {
    fn drop(&mut self) {
        let _ = close(self.handle);
    }
}

// ============================================================================
// Msi - MSI vector allocation
// ============================================================================

/// MSI info returned from kernel
pub struct MsiInfo {
    pub first_irq: u32,
    pub count: u8,
    pub bdf: u32,
}

/// MSI vector allocation
pub struct Msi {
    handle: ObjHandle,
    info: MsiInfo,
}

impl Msi {
    /// Allocate MSI vectors for a device
    pub fn allocate(bdf: u32, count: u8) -> SysResult<Self> {
        let mut params = [0u8; 5];
        params[0..4].copy_from_slice(&bdf.to_le_bytes());
        params[4] = count;
        let handle = open(ObjectType::Msi, &params)?;

        // Read MSI info
        let mut buf = [0u8; 9];
        read(handle, &mut buf)?;
        let first_irq = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let count = buf[4];
        let bdf = u32::from_le_bytes([buf[5], buf[6], buf[7], buf[8]]);

        Ok(Self {
            handle,
            info: MsiInfo { first_irq, count, bdf },
        })
    }

    pub fn handle(&self) -> ObjHandle { self.handle }
    pub fn info(&self) -> &MsiInfo { &self.info }
    pub fn first_irq(&self) -> u32 { self.info.first_irq }
    pub fn count(&self) -> u8 { self.info.count }
}

impl Drop for Msi {
    fn drop(&mut self) {
        let _ = close(self.handle);
    }
}

// ============================================================================
// PipeRing - High-performance IPC with typed messages
// ============================================================================

/// High-performance ring buffer IPC
///
/// Memory layout:
/// ```text
/// [RingHeader (64 bytes)][TX Ring][RX Ring]
/// ```
///
/// # Usage
///
/// ```ignore
/// // Create a ring with preset configuration
/// let ring = PipeRing::create(abi::ring_presets::CONSOLE_TX, abi::ring_presets::CONSOLE_RX)?;
///
/// // Map to access shared memory
/// let addr = ring.map()?;
///
/// // Get header for atomic operations
/// let header = ring.header();
/// ```
pub struct PipeRing {
    handle: ObjHandle,
    vaddr: u64,
    tx_config: u16,
    rx_config: u16,
}

impl PipeRing {
    /// Create a new pipe ring
    ///
    /// # Arguments
    /// * `tx_config` - TX ring configuration (use `abi::ring_presets::*`)
    /// * `rx_config` - RX ring configuration
    pub fn create(tx_config: u16, rx_config: u16) -> SysResult<Self> {
        let params = abi::RingParams { tx: tx_config, rx: rx_config };
        let params_bytes = unsafe {
            core::slice::from_raw_parts(
                &params as *const abi::RingParams as *const u8,
                core::mem::size_of::<abi::RingParams>()
            )
        };
        let handle = open(ObjectType::Ring, params_bytes)?;
        Ok(Self {
            handle,
            vaddr: 0,
            tx_config,
            rx_config,
        })
    }

    /// Create with preset for console output (large TX, small RX)
    pub fn console_out() -> SysResult<Self> {
        Self::create(abi::ring_presets::CONSOLE_TX, abi::ring_presets::CONSOLE_RX)
    }

    /// Create with balanced configuration
    pub fn balanced() -> SysResult<Self> {
        Self::create(abi::ring_presets::BALANCED, abi::ring_presets::BALANCED)
    }

    /// Create with bulk configuration (large buffers)
    pub fn bulk() -> SysResult<Self> {
        Self::create(abi::ring_presets::BULK, abi::ring_presets::BULK)
    }

    pub fn handle(&self) -> ObjHandle { self.handle }
    pub fn is_mapped(&self) -> bool { self.vaddr != 0 }
    pub fn vaddr(&self) -> u64 { self.vaddr }

    /// Map the ring into address space
    pub fn map_ring(&mut self) -> SysResult<u64> {
        if self.vaddr != 0 {
            return Ok(self.vaddr);
        }
        let addr = map(self.handle, 0)?;  // flags = 0 for default mapping
        self.vaddr = addr;
        Ok(addr)
    }

    /// Get the ring header (must be mapped first)
    ///
    /// # Safety
    /// Caller must ensure the ring is mapped before calling this.
    pub fn header(&self) -> Option<&abi::RingHeader> {
        if self.vaddr == 0 {
            return None;
        }
        Some(unsafe { &*(self.vaddr as *const abi::RingHeader) })
    }

    /// Get mutable reference to ring header
    ///
    /// # Safety
    /// Caller must ensure the ring is mapped before calling this.
    pub fn header_mut(&mut self) -> Option<&mut abi::RingHeader> {
        if self.vaddr == 0 {
            return None;
        }
        Some(unsafe { &mut *(self.vaddr as *mut abi::RingHeader) })
    }

    /// Get TX ring size in bytes
    pub fn tx_ring_size(&self) -> usize {
        abi::ring_config::ring_size(self.tx_config)
    }

    /// Get RX ring size in bytes
    pub fn rx_ring_size(&self) -> usize {
        abi::ring_config::ring_size(self.rx_config)
    }

    /// Get TX slot size in bytes
    pub fn tx_slot_size(&self) -> usize {
        abi::ring_config::slot_size(self.tx_config)
    }

    /// Get RX slot size in bytes
    pub fn rx_slot_size(&self) -> usize {
        abi::ring_config::slot_size(self.rx_config)
    }

    /// Get pointer to TX ring buffer (must be mapped first)
    pub fn tx_ring_ptr(&self) -> Option<*mut u8> {
        if self.vaddr == 0 {
            return None;
        }
        Some(unsafe { (self.vaddr as *mut u8).add(64) })
    }

    /// Get pointer to RX ring buffer (must be mapped first)
    pub fn rx_ring_ptr(&self) -> Option<*mut u8> {
        if self.vaddr == 0 {
            return None;
        }
        Some(unsafe { (self.vaddr as *mut u8).add(64 + self.tx_ring_size()) })
    }

    /// Notify peer that data is available
    ///
    /// Call this after writing to the TX ring to wake the peer.
    pub fn notify(&self) -> SysResult<()> {
        write(self.handle, &[])?;
        Ok(())
    }

    /// Wait for data to be available
    ///
    /// Blocks until peer notifies or ring is closed.
    pub fn wait(&self) -> SysResult<()> {
        let mut buf = [0u8; 0];
        read(self.handle, &mut buf)?;
        Ok(())
    }
}

impl Drop for PipeRing {
    fn drop(&mut self) {
        let _ = close(self.handle);
    }
}
