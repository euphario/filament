//! Bus Framework - Traits, Types, and Message Definitions
//!
//! **THIS IS THE ONLY WAY TO WRITE DRIVERS.**
//!
//! All userspace drivers MUST use this framework. No driver may directly use
//! Mux, Channel, Port, DevdClient, CommandRing, or raw syscalls for its event
//! loop or IPC. If a driver needs something the framework doesn't provide
//! (a new data transport, a new callback, a new BusCtx method), the framework
//! is what gets updated — not bypassed.
//!
//! A driver is a struct that implements `Driver`, and its `main()` calls
//! `driver_main()`. That's it. Everything else is the runtime's job.
//!
//! ## Architecture
//!
//! ```text
//! ┌──────────────────────────────────────────────┐
//! │  Driver impl (domain logic only)             │
//! │  fn init()  fn command()  fn data_ready()    │
//! └──────────────┬───────────────────────────────┘
//!                │ calls methods on
//! ┌──────────────▼───────────────────────────────┐
//! │  BusCtx (trait)                               │
//! │  reply()  send_down()  spawn_child()  ...    │
//! └──────────────┬───────────────────────────────┘
//!                │ implemented by
//! ┌──────────────▼───────────────────────────────┐
//! │  DriverRuntime (concrete)                     │
//! │  Mux + DevdClient + DataPorts + children     │
//! └──────────────────────────────────────────────┘
//! ```

// ============================================================================
// Constants
// ============================================================================

/// Maximum payload size in a single bus message.
/// Envelope is 16 bytes + 240 payload = 256 bytes total (cache-friendly).
pub const BUS_MSG_PAYLOAD: usize = 240;

/// Maximum number of chunks for reassembly.
pub const MAX_CHUNKS: usize = 255;

/// Maximum children per driver.
pub const MAX_CHILDREN: usize = 8;

/// Maximum ports per type per driver.
pub const MAX_PORTS: usize = 8;

/// Maximum pending requests (for deadline tracking).
pub const MAX_PENDING: usize = 16;

/// Maximum kernel buses a driver can claim.
pub const MAX_KERNEL_BUSES: usize = 4;

// ============================================================================
// Opaque Handles
// ============================================================================

/// Opaque child identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChildId(pub u8);

/// Opaque port identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PortId(pub u8);

/// Opaque kernel bus identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KernelBusId(pub u8);

// ============================================================================
// Spawn Context
// ============================================================================

/// Context provided by devd for rule-spawned drivers.
///
/// When devd matches a rule and tells a parent to spawn a child driver,
/// the child can query this context to discover which port triggered
/// the spawn. The port name is the key — it identifies the parent's
/// device port that the child should connect to.
#[derive(Clone, Copy)]
pub struct SpawnContext {
    /// Port name that triggered the spawn (e.g., "pci/00:02.0:nvme")
    name: [u8; 64],
    /// Actual length of port name
    name_len: u8,
    /// Type of the port that triggered the spawn
    pub port_type: crate::devd::PortType,
    /// Opaque metadata from the port registration (e.g., BAR0 info)
    metadata_buf: [u8; 64],
    /// Actual length of metadata
    metadata_len: u8,
}

impl SpawnContext {
    /// Create a new spawn context from raw devd response.
    pub(crate) fn new(
        name_buf: &[u8; 64],
        name_len: usize,
        port_type: crate::devd::PortType,
        metadata: &[u8],
    ) -> Self {
        let len = name_len.min(64) as u8;
        let mut name = [0u8; 64];
        name[..len as usize].copy_from_slice(&name_buf[..len as usize]);
        let meta_len = metadata.len().min(64) as u8;
        let mut metadata_buf = [0u8; 64];
        metadata_buf[..meta_len as usize].copy_from_slice(&metadata[..meta_len as usize]);
        Self {
            name, name_len: len, port_type,
            metadata_buf, metadata_len: meta_len,
        }
    }

    /// Get the trigger port name as a byte slice.
    pub fn port_name(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }

    /// Get the metadata bytes attached to the port registration.
    pub fn metadata(&self) -> &[u8] {
        &self.metadata_buf[..self.metadata_len as usize]
    }
}

// ============================================================================
// Error Types
// ============================================================================

/// Bus framework error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BusError {
    /// Transport link is down.
    LinkDown,
    /// Transport buffer full.
    BufferFull,
    /// Invalid message format.
    InvalidMessage,
    /// No space for new child/port.
    NoSpace,
    /// Child/port not found.
    NotFound,
    /// Operation timed out.
    Timeout,
    /// Spawn failed.
    SpawnFailed,
    /// Shared memory error.
    ShmemError,
    /// Internal error.
    Internal,
}

// ============================================================================
// Kernel Bus State
// ============================================================================

/// Kernel bus state (mirrors kernel BusState enum).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum KernelBusState {
    /// Bus is idle, hardware safe, ready for a driver.
    Safe = 0,
    /// Bus is claimed by a driver.
    Claimed = 1,
    /// Bus is being reset (hardware quiescing).
    Resetting = 2,
}

impl KernelBusState {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Safe,
            1 => Self::Claimed,
            2 => Self::Resetting,
            _ => Self::Safe,
        }
    }
}

/// Reason for a kernel bus state change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum KernelBusChangeReason {
    Connected = 0,
    Disconnected = 1,
    OwnerCrashed = 2,
    ResetRequested = 3,
    ResetComplete = 4,
    Handoff = 5,
    DriverClaimed = 6,
    DriverExited = 7,
}

impl KernelBusChangeReason {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Connected,
            1 => Self::Disconnected,
            2 => Self::OwnerCrashed,
            3 => Self::ResetRequested,
            4 => Self::ResetComplete,
            5 => Self::Handoff,
            6 => Self::DriverClaimed,
            7 => Self::DriverExited,
            _ => Self::Connected,
        }
    }
}

/// Information about a kernel bus obtained via `claim_kernel_bus()`.
///
/// Parsed from the kernel's StateSnapshot message. Updated automatically
/// by the runtime when StateChanged notifications arrive.
#[derive(Clone, Copy, Debug)]
pub struct KernelBusInfo {
    /// Bus type (0=PCIe, 1=USB, 2=Platform).
    pub bus_type: u8,
    /// Bus index (e.g., 0 for pcie0).
    pub bus_index: u8,
    /// Current state.
    pub state: KernelBusState,
    /// Number of devices on the bus.
    pub device_count: u8,
    /// Hardware capability flags.
    pub capabilities: u8,
}

impl KernelBusInfo {
    pub const fn empty() -> Self {
        Self {
            bus_type: 0,
            bus_index: 0,
            state: KernelBusState::Safe,
            device_count: 0,
            capabilities: 0,
        }
    }
}

// ============================================================================
// Bus Message Flags
// ============================================================================

/// Bus message flags (bitfield).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BusMsgFlags(pub u8);

impl BusMsgFlags {
    /// More chunks follow (multi-chunk message).
    pub const MORE: Self = Self(1 << 0);
    /// This is an unsolicited event (flows up).
    pub const EVENT: Self = Self(1 << 1);
    /// Broadcast to all children.
    pub const BROADCAST: Self = Self(1 << 2);

    pub const fn empty() -> Self {
        Self(0)
    }

    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

// ============================================================================
// Bus Message Envelope
// ============================================================================

/// Bus message envelope — the unit of communication on the control plane.
///
/// 256 bytes total, cache-line friendly. The runtime handles
/// serialization — drivers work with typed payloads via read/write helpers.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct BusMsg {
    /// What kind of message (see `bus_msg` module).
    pub msg_type: u32,
    /// Sequence ID: 0 = unsolicited event, >0 = request/response pair.
    pub seq_id: u32,
    /// Flags (MORE, EVENT, BROADCAST).
    pub flags: BusMsgFlags,
    /// Reserved padding.
    pub _pad0: u8,
    /// Hop counter (incremented at each forward).
    pub origin: u16,
    /// Chunk index for multi-chunk messages.
    pub chunk_index: u8,
    /// Reserved padding.
    pub _pad1: u8,
    /// How much of payload[] is valid.
    pub payload_len: u16,
    /// Fixed-size payload.
    pub payload: [u8; BUS_MSG_PAYLOAD],
}

// 4 + 4 + 1 + 1 + 2 + 1 + 1 + 2 + 240 = 256 bytes
const _: () = assert!(core::mem::size_of::<BusMsg>() == 256);

impl BusMsg {
    /// Create an empty message with given type.
    pub const fn new(msg_type: u32) -> Self {
        Self {
            msg_type,
            seq_id: 0,
            flags: BusMsgFlags::empty(),
            _pad0: 0,
            origin: 0,
            chunk_index: 0,
            _pad1: 0,
            payload_len: 0,
            payload: [0; BUS_MSG_PAYLOAD],
        }
    }

    /// Create a message with payload bytes.
    pub fn with_payload(msg_type: u32, data: &[u8]) -> Self {
        let mut msg = Self::new(msg_type);
        let len = data.len().min(BUS_MSG_PAYLOAD);
        msg.payload[..len].copy_from_slice(&data[..len]);
        msg.payload_len = len as u16;
        msg
    }

    /// Create an event message (flows up to parent).
    pub fn event(msg_type: u32, data: &[u8]) -> Self {
        let mut msg = Self::with_payload(msg_type, data);
        msg.flags = BusMsgFlags::EVENT;
        msg
    }

    // === Payload read helpers ===

    pub fn read_u8(&self, offset: usize) -> u8 {
        if offset < BUS_MSG_PAYLOAD {
            self.payload[offset]
        } else {
            0
        }
    }

    pub fn read_u16(&self, offset: usize) -> u16 {
        if offset + 2 <= BUS_MSG_PAYLOAD {
            u16::from_le_bytes([self.payload[offset], self.payload[offset + 1]])
        } else {
            0
        }
    }

    pub fn read_u32(&self, offset: usize) -> u32 {
        if offset + 4 <= BUS_MSG_PAYLOAD {
            u32::from_le_bytes([
                self.payload[offset],
                self.payload[offset + 1],
                self.payload[offset + 2],
                self.payload[offset + 3],
            ])
        } else {
            0
        }
    }

    pub fn read_u64(&self, offset: usize) -> u64 {
        if offset + 8 <= BUS_MSG_PAYLOAD {
            u64::from_le_bytes([
                self.payload[offset],
                self.payload[offset + 1],
                self.payload[offset + 2],
                self.payload[offset + 3],
                self.payload[offset + 4],
                self.payload[offset + 5],
                self.payload[offset + 6],
                self.payload[offset + 7],
            ])
        } else {
            0
        }
    }

    pub fn read_bytes(&self, offset: usize, len: usize) -> &[u8] {
        let end = (offset + len).min(self.payload_len as usize).min(BUS_MSG_PAYLOAD);
        if offset < end {
            &self.payload[offset..end]
        } else {
            &[]
        }
    }

    // === Payload write helpers ===

    pub fn write_u8(&mut self, offset: usize, val: u8) {
        if offset < BUS_MSG_PAYLOAD {
            self.payload[offset] = val;
            self.payload_len = self.payload_len.max((offset + 1) as u16);
        }
    }

    pub fn write_u16(&mut self, offset: usize, val: u16) {
        if offset + 2 <= BUS_MSG_PAYLOAD {
            self.payload[offset..offset + 2].copy_from_slice(&val.to_le_bytes());
            self.payload_len = self.payload_len.max((offset + 2) as u16);
        }
    }

    pub fn write_u32(&mut self, offset: usize, val: u32) {
        if offset + 4 <= BUS_MSG_PAYLOAD {
            self.payload[offset..offset + 4].copy_from_slice(&val.to_le_bytes());
            self.payload_len = self.payload_len.max((offset + 4) as u16);
        }
    }

    pub fn write_u64(&mut self, offset: usize, val: u64) {
        if offset + 8 <= BUS_MSG_PAYLOAD {
            self.payload[offset..offset + 8].copy_from_slice(&val.to_le_bytes());
            self.payload_len = self.payload_len.max((offset + 8) as u16);
        }
    }

    pub fn write_bytes(&mut self, offset: usize, data: &[u8]) {
        let end = (offset + data.len()).min(BUS_MSG_PAYLOAD);
        if offset < end {
            let len = end - offset;
            self.payload[offset..end].copy_from_slice(&data[..len]);
            self.payload_len = self.payload_len.max(end as u16);
        }
    }

    /// Read a null-terminated byte string starting at `offset`.
    /// Returns the bytes before the null terminator (or to end of payload).
    pub fn read_null_terminated(&self, offset: usize) -> &[u8] {
        let end = self.payload_len as usize;
        if offset >= end {
            return &[];
        }
        for i in offset..end {
            if self.payload[i] == 0 {
                return &self.payload[offset..i];
            }
        }
        &self.payload[offset..end]
    }

    /// Parse a CONFIG_SET payload: `key\0value\0`.
    /// Returns (key, value) byte slices.
    pub fn parse_config_kv(&self) -> (&[u8], &[u8]) {
        let key = self.read_null_terminated(0);
        let value_start = key.len() + 1; // skip null separator
        let value = self.read_null_terminated(value_start);
        (key, value)
    }

    /// Get valid payload slice.
    pub fn payload_data(&self) -> &[u8] {
        &self.payload[..self.payload_len as usize]
    }

    /// Serialize to bytes for transport.
    pub fn to_bytes(&self) -> [u8; 256] {
        unsafe { core::mem::transmute_copy(self) }
    }

    /// Deserialize from bytes.
    pub fn from_bytes(data: &[u8; 256]) -> Self {
        // Use read_unaligned — data may not satisfy BusMsg alignment (4 bytes)
        unsafe { core::ptr::read_unaligned(data.as_ptr() as *const Self) }
    }
}

impl Default for BusMsg {
    fn default() -> Self {
        Self::new(0)
    }
}

// ============================================================================
// Message Type Constants
// ============================================================================

/// Bus message type constants.
pub mod bus_msg {
    // Lifecycle (0x01xx)
    pub const PING: u32 = 0x0100;
    pub const SHUTDOWN: u32 = 0x0101;
    pub const RESET: u32 = 0x0102;

    // Discovery (0x02xx)
    pub const QUERY_INFO: u32 = 0x0200;
    pub const QUERY_TREE: u32 = 0x0201;
    pub const QUERY_CAPS: u32 = 0x0202;

    // Device orchestration (0x03xx)
    pub const ATTACH_DISK: u32 = 0x0300;
    pub const REPORT_PARTITIONS: u32 = 0x0301;
    pub const REGISTER_PARTITION: u32 = 0x0302;
    pub const PARTITION_READY: u32 = 0x0303;
    pub const MOUNT_PARTITION: u32 = 0x0304;
    // Events (0x04xx) — unsolicited, flow up
    pub const LINK_DOWN: u32 = 0x0400;
    pub const LINK_UP: u32 = 0x0401;
    pub const DEVICE_ERROR: u32 = 0x0402;
    pub const DEVICE_REMOVED: u32 = 0x0403;
    pub const CHILD_READY: u32 = 0x0404;

    // SpawnChild (internal to runtime, not exposed to Driver)
    pub const SPAWN_CHILD: u32 = 0x0500;
    pub const STOP_CHILD: u32 = 0x0501;

    // Configuration (0x06xx)
    pub const CONFIG_GET: u32 = 0x0600;
    pub const CONFIG_SET: u32 = 0x0601;
    pub const CONFIG_RESPONSE: u32 = 0x0602;
}

/// Well-known configuration key strings.
///
/// Payload layout for CONFIG_GET: `key\0`
/// Payload layout for CONFIG_SET: `key\0value\0`
/// Payload layout for CONFIG_RESPONSE: value bytes (no null terminator needed)
pub mod config_key {
    pub const NET_IP: &[u8] = b"net.ip";
    pub const NET_PREFIX: &[u8] = b"net.prefix";
    pub const NET_GATEWAY: &[u8] = b"net.gateway";
    pub const NET_DHCP: &[u8] = b"net.dhcp";
    pub const NET_MAC: &[u8] = b"net.mac";
    pub const NET_STATE: &[u8] = b"net.state";
    pub const NET_SUMMARY: &[u8] = b"net.summary";
}

// ============================================================================
// Config Key Registration
// ============================================================================

/// A configuration key advertised by a driver.
///
/// Drivers return a static slice of these from `config_keys()`. The framework
/// uses them to auto-generate the summary (empty-key GET) and the `keys=` line
/// listing writable keys.
pub struct ConfigKey {
    /// Key name (e.g. b"net.ip").
    pub name: &'static [u8],
    /// Whether SET is supported for this key.
    pub writable: bool,
}

impl ConfigKey {
    pub const fn read_only(name: &'static [u8]) -> Self {
        Self { name, writable: false }
    }

    pub const fn read_write(name: &'static [u8]) -> Self {
        Self { name, writable: true }
    }
}

// ============================================================================
// Disposition
// ============================================================================

/// What to do after peeking at a command.
pub enum Disposition {
    /// I consumed it. Don't forward to children.
    Handled,
    /// Not for me. Forward to all children.
    Forward,
    /// I handled it AND children should also see it.
    HandledAndForward,
}

// ============================================================================
// Driver Trait
// ============================================================================

/// The trait every driver implements. All domain logic, no IPC plumbing.
///
/// The runtime calls these callbacks in response to events. The driver
/// uses `BusCtx` to interact with the system.
pub trait Driver {
    /// Called once when the bus link to parent is established.
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError>;

    /// A command arrived from above (parent or further up the tree).
    /// Peek at it: return Handled, Forward, or HandledAndForward.
    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition;

    /// An unsolicited event arrived from a child.
    fn event(&mut self, _child: ChildId, msg: &BusMsg, ctx: &mut dyn BusCtx) {
        // Default: forward up to parent
        let _ = ctx.send_up(msg);
    }

    /// A response arrived for a request this driver sent via send_down().
    fn response(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) {}

    /// A child process died.
    fn child_exited(&mut self, _child: ChildId, _exit_code: i32, _ctx: &mut dyn BusCtx) {}

    /// Data is ready on a data port (SQ entry arrived, CQ completion, etc).
    fn data_ready(&mut self, _port: PortId, _ctx: &mut dyn BusCtx) {}

    /// A deadline expired for a pending request.
    fn deadline(&mut self, _seq_id: u32, _ctx: &mut dyn BusCtx) {}

    /// A driver-registered handle became readable.
    ///
    /// Called when a handle registered via `ctx.watch_handle()` fires.
    /// The `tag` is the value passed when the handle was registered.
    fn handle_event(&mut self, _tag: u32, _handle: crate::syscall::Handle, _ctx: &mut dyn BusCtx) {}

    // === Configuration ===

    /// Declare supported config keys. Return an empty slice if no config.
    ///
    /// The framework uses this to:
    /// - Auto-generate the summary response for empty-key GET
    /// - Build the `keys=` line listing writable keys
    /// - Validate keys before calling `config_get()`/`config_set()`
    fn config_keys(&self) -> &[ConfigKey] { &[] }

    /// Get a config value. Called by the framework for CONFIG_GET.
    /// Write the value into `buf` and return the number of bytes written.
    fn config_get(&self, _key: &[u8], _buf: &mut [u8]) -> usize { 0 }

    /// Set a config value. Called by the framework for CONFIG_SET.
    /// Write the response (e.g. "OK\n" or "ERR ...\n") into `buf`.
    /// Returns the number of bytes written.
    fn config_set(&mut self, _key: &[u8], _value: &[u8], _buf: &mut [u8], _ctx: &mut dyn BusCtx) -> usize { 0 }

    // === Bus state ===

    /// A kernel bus state changed (Safe, Claimed, Resetting).
    ///
    /// Called when a bus claimed via `ctx.claim_kernel_bus()` receives a
    /// StateChanged notification from the kernel. The runtime automatically
    /// forwards these to devd — this callback is for driver-specific reactions.
    fn bus_state_changed(
        &mut self,
        _bus: KernelBusId,
        _old_state: KernelBusState,
        _new_state: KernelBusState,
        _reason: KernelBusChangeReason,
        _ctx: &mut dyn BusCtx,
    ) {}
}

// ============================================================================
// BusCtx Trait
// ============================================================================

/// The "do things" interface provided by the runtime to the driver.
///
/// Object-safe: no generic methods, no Self-returning methods.
pub trait BusCtx {
    // === Tree routing ===

    /// Reply to a command (sends response back up toward originator).
    fn reply(&mut self, original: &BusMsg, payload: &[u8]) -> Result<(), BusError>;

    /// Forward a command to all children (flood down).
    fn forward_down(&mut self, msg: &BusMsg) -> Result<(), BusError>;

    /// Send a new command down to all children. Returns seq_id for tracking.
    fn send_down(
        &mut self,
        msg_type: u32,
        payload: &[u8],
        deadline_ms: u32,
    ) -> Result<u32, BusError>;

    /// Send an unsolicited event up to parent.
    fn send_up(&mut self, msg: &BusMsg) -> Result<(), BusError>;

    /// Send an unsolicited event up with fresh payload.
    fn emit_event(&mut self, msg_type: u32, payload: &[u8]) -> Result<(), BusError>;

    // === Child management ===

    /// Spawn a child process. Returns ChildId for tracking.
    fn spawn_child(&mut self, binary_name: &[u8]) -> Result<ChildId, BusError>;

    /// Number of connected children.
    fn child_count(&self) -> usize;

    // === Data ports ===

    /// Create a block data port (this driver is provider). Returns PortId.
    fn create_block_port(&mut self, config: BlockPortConfig) -> Result<PortId, BusError>;

    /// Connect to a block data port (this driver is consumer). Returns PortId.
    fn connect_block_port(&mut self, shmem_id: u32) -> Result<PortId, BusError>;

    /// Access a block port by ID.
    fn block_port(&mut self, id: PortId) -> Option<&mut dyn BlockTransport>;

    // === Event loop integration ===

    /// Register a handle with the runtime's Mux for event-driven wakeup.
    ///
    /// When the handle becomes readable, `Driver::handle_event()` is called
    /// with the given tag. Use this for kernel bus channels, device ports, etc.
    fn watch_handle(&mut self, handle: crate::syscall::Handle, tag: u32) -> Result<(), BusError>;

    /// Remove a handle from the runtime's Mux.
    fn unwatch_handle(&mut self, handle: crate::syscall::Handle) -> Result<(), BusError>;

    // === Kernel bus ===

    /// Connect to a kernel bus and claim it.
    ///
    /// Encapsulates the full kernel bus protocol:
    /// 1. Connect to the bus port (e.g., `/kernel/bus/pcie0`)
    /// 2. Receive StateSnapshot
    /// 3. Wait for Safe state if bus is Resetting
    /// 4. Send SetDriver with this process's PID
    /// 5. Register the channel with Mux for ongoing state notifications
    ///
    /// State changes are automatically forwarded to devd and dispatched
    /// to `Driver::bus_state_changed()`.
    ///
    /// Returns the initial bus info on success.
    fn claim_kernel_bus(&mut self, path: &[u8]) -> Result<(KernelBusId, KernelBusInfo), BusError>;

    /// Get cached info for a claimed kernel bus.
    fn kernel_bus_info(&self, id: KernelBusId) -> Option<&KernelBusInfo>;

    /// Enable bus mastering for a PCI device via kernel bus control.
    ///
    /// `bus_id`: The kernel bus returned by `claim_kernel_bus()`.
    /// `device_bdf`: Packed BDF (bus<<8 | dev<<3 | func).
    fn enable_bus_mastering(&mut self, bus_id: KernelBusId, device_bdf: u16) -> Result<(), BusError>;

    // === Identity ===

    /// This driver's name.
    fn name(&self) -> &[u8];

    // === Devd integration (v1) ===

    /// Report partitions to devd (orchestration shortcut).
    fn report_partitions(
        &mut self,
        disk_shmem_id: u32,
        scheme: u8,
        parts: &[crate::query::PartitionInfoMsg],
    ) -> Result<(), BusError>;

    /// Notify devd that a partition DataPort is ready.
    fn partition_ready(&mut self, name: &[u8], shmem_id: u32) -> Result<(), BusError>;

    /// Respond to a QueryInfo request with text.
    fn respond_info(&mut self, seq_id: u32, info: &[u8]) -> Result<(), BusError>;

    /// Acknowledge a spawn request.
    fn ack_spawn(&mut self, seq_id: u32, result: i32, pid: u32) -> Result<(), BusError>;

    /// Register a port with devd.
    ///
    /// `shmem_id` is the DataPort shared memory ID (0 = no DataPort).
    /// For Block ports, a non-zero shmem_id triggers block orchestration
    /// in devd (spawns partd, sends ATTACH_DISK).
    fn register_port(
        &mut self,
        name: &[u8],
        port_type: crate::devd::PortType,
        shmem_id: u32,
        parent: Option<&[u8]>,
    ) -> Result<(), BusError> {
        self.register_port_with_metadata(name, port_type, shmem_id, parent, &[])
    }

    /// Register a port with devd including metadata.
    ///
    /// Metadata is opaque bytes stored alongside the port and delivered
    /// to child drivers via their spawn context. Used e.g. to pass BAR0
    /// info from pcied to nvmed without requiring a separate IPC channel.
    fn register_port_with_metadata(
        &mut self,
        name: &[u8],
        port_type: crate::devd::PortType,
        shmem_id: u32,
        parent: Option<&[u8]>,
        metadata: &[u8],
    ) -> Result<(), BusError>;

    /// Report driver state to devd.
    fn report_state(&mut self, state: crate::devd::DriverState) -> Result<(), BusError>;

    /// Get spawn context from devd (cached after first call).
    ///
    /// Returns the port name and type that triggered this driver's spawn.
    /// Returns `NotFound` if this driver was not rule-spawned.
    /// Returns `LinkDown` or `Internal` on transport failure.
    fn spawn_context(&mut self) -> Result<&SpawnContext, BusError>;

    /// Discover a port's DataPort shmem_id by name via devd-query.
    ///
    /// Queries devd for the named port (e.g., `b"vfs:"`) and returns
    /// the shmem_id of its DataPort. Returns `NotFound` if the port
    /// doesn't exist or has no DataPort.
    fn discover_port(&mut self, name: &[u8]) -> Result<u32, BusError>;
}

// ============================================================================
// Data Transport Traits
// ============================================================================

/// Block port configuration.
#[derive(Clone, Copy)]
pub struct BlockPortConfig {
    /// Number of SQ/CQ entries (power of 2).
    pub ring_size: u16,
    /// Number of sidechannel entries (power of 2, 0 to disable).
    pub side_size: u16,
    /// Size of data buffer pool in bytes.
    pub pool_size: u32,
}

impl Default for BlockPortConfig {
    fn default() -> Self {
        Self {
            ring_size: 64,
            side_size: 8,
            pool_size: 256 * 1024,
        }
    }
}

/// Block device geometry.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct BlockGeometry {
    pub block_size: u32,
    pub block_count: u64,
    pub max_transfer: u32,
}

/// Block operation completion.
#[derive(Clone, Copy)]
pub struct BlockCompletion {
    pub tag: u32,
    pub status: u16,
    pub transferred: u32,
    pub result: u32,
}

/// Packet descriptor for packet transport.
#[derive(Clone, Copy)]
pub struct PacketDescriptor {
    pub buf_offset: u32,
    pub len: u32,
}

/// Packet port configuration.
#[derive(Clone, Copy)]
pub struct PacketPortConfig {
    pub ring_size: u16,
    pub pool_size: u32,
}

/// Device event for event transport.
#[derive(Clone, Copy)]
pub struct DeviceEvent {
    pub event_type: u16,
    pub data: [u8; 14],
}

/// Port error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortError {
    /// Queue full.
    Full,
    /// No buffer space.
    NoSpace,
    /// Invalid argument.
    Invalid,
    /// Port not connected.
    NotConnected,
}

/// Block device data transport (SQ/CQ pattern).
///
/// Wraps the DataPort/LayeredRing infrastructure with a clean API.
/// Both provider and consumer sides implement this trait.
pub trait BlockTransport {
    /// Submit a read request. Returns tag for matching completion.
    fn submit_read(&mut self, lba: u64, buf_offset: u32, len: u32) -> Result<u32, PortError>;

    /// Submit a write request. Returns tag.
    fn submit_write(&mut self, lba: u64, buf_offset: u32, len: u32) -> Result<u32, PortError>;

    /// Submit an arbitrary request (consumer side).
    fn submit(&self, sqe: &crate::ring::IoSqe) -> bool;

    /// Receive next submission (provider side).
    fn recv_request(&self) -> Option<crate::ring::IoSqe>;

    /// Complete a request with success (provider side).
    fn complete_ok(&self, tag: u32, transferred: u32) -> bool;

    /// Complete a request with error (provider side).
    fn complete_error(&self, tag: u32, status: u16) -> bool;

    /// Complete a request with result and transferred (provider side).
    fn complete_with_result(&self, tag: u32, result: u32, transferred: u32) -> bool;

    /// Complete a request with error status and result code (provider side).
    fn complete_error_with_result(&self, tag: u32, status: u16, result: u32) -> bool;

    /// Complete a request with a raw CQE (provider side).
    fn complete(&self, cqe: &crate::ring::IoCqe) -> bool;

    /// Poll for next completion (consumer side, non-blocking).
    fn poll_completion(&self) -> Option<BlockCompletion>;

    /// Allocate buffer in the DMA pool. Returns offset.
    fn alloc(&mut self, size: u32) -> Option<u32>;

    /// Reset pool allocator (free all allocations).
    fn reset_pool(&mut self);

    /// Get slice into DMA pool at offset.
    fn pool_slice(&self, offset: u32, len: u32) -> Option<&[u8]>;

    /// Get mutable slice into DMA pool.
    fn pool_slice_mut(&self, offset: u32, len: u32) -> Option<&mut [u8]>;

    /// Write data to pool at offset.
    fn pool_write(&self, offset: u32, data: &[u8]) -> bool;

    /// Get raw pool base pointer (for zero-copy token access).
    fn pool_ptr(&self) -> *const u8;

    /// Get pool physical address (for DMA programming).
    fn pool_phys(&self) -> u64;

    /// Get pool size.
    fn pool_size(&self) -> u32;

    /// Query geometry via sidechannel.
    fn query_geometry(&mut self) -> Option<BlockGeometry>;

    /// Respond to geometry query (provider side).
    fn respond_geometry(&self, entry: &crate::ring::SideEntry, info: &BlockGeometry) -> bool;

    /// Poll for sidechannel request (provider side).
    fn poll_side_request(&self) -> Option<crate::ring::SideEntry>;

    /// Poll for sidechannel response (consumer side).
    fn poll_side_response(&self) -> Option<crate::ring::SideEntry>;

    /// Send a sidechannel entry (consumer side).
    fn side_send(&self, entry: &crate::ring::SideEntry) -> bool;

    /// Get shared memory ID.
    fn shmem_id(&self) -> u32;

    /// Make port publicly accessible.
    fn set_public(&self) -> bool;

    /// Notify peer that work is available.
    fn notify(&self);

    /// Block until peer notifies (completion or request available).
    ///
    /// Uses shmem wait — no polling, no busy loops. Returns true if
    /// notified, false on timeout.
    fn wait(&self, timeout_ms: u32) -> bool;

    /// Get the underlying shmem handle for Mux registration.
    fn mux_handle(&self) -> Option<crate::syscall::Handle>;
}

/// Stream data transport (byte ring pattern).
pub trait StreamTransport {
    fn write_data(&mut self, data: &[u8]) -> Result<usize, PortError>;
    fn read_data(&mut self, buf: &mut [u8]) -> Result<usize, PortError>;
    fn mux_handle(&self) -> Option<crate::syscall::Handle>;
}

/// Packet data transport (TX/RX descriptor rings).
pub trait PacketTransport {
    fn submit_tx(&mut self, buf_offset: u32, len: u32) -> Result<u32, PortError>;
    fn poll_tx_completion(&mut self) -> Option<u32>;
    fn poll_rx(&mut self) -> Option<PacketDescriptor>;
    fn alloc(&mut self, size: u32) -> Option<u32>;
    fn pool_slice(&self, offset: u32, len: u32) -> Option<&[u8]>;
    fn pool_slice_mut(&self, offset: u32, len: u32) -> Option<&mut [u8]>;
    fn pool_phys(&self) -> u64;
    fn mux_handle(&self) -> Option<crate::syscall::Handle>;
    fn notify(&self);
}

/// Event data transport (small typed events).
pub trait EventTransport {
    fn poll_event(&mut self) -> Option<DeviceEvent>;
    fn mux_handle(&self) -> Option<crate::syscall::Handle>;
}

// ============================================================================
// Control Transport Trait (internal to runtime)
// ============================================================================

/// How bus messages actually move between parent and child.
/// The Driver never sees this. The runtime uses it internally.
///
/// For v1, implemented via Channel (IPC). Can be replaced with
/// shmem-backed ring for higher performance.
pub trait ControlTransport {
    /// Send a message to the peer.
    fn send(&mut self, msg: &BusMsg) -> Result<(), BusError>;

    /// Try to receive a message (non-blocking).
    fn try_recv(&mut self) -> Option<BusMsg>;

    /// Get a handle for Mux registration (event-driven wakeup).
    fn mux_handle(&self) -> crate::syscall::Handle;

    /// Check if the link is alive.
    fn is_alive(&self) -> bool;
}
