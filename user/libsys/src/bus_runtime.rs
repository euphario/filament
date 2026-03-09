//! Bus Runtime - DriverRuntime and driver_main()
//!
//! The runtime owns the event loop and dispatches to Driver callbacks.
//! Uses a Mux-based event loop — blocks until something happens, no polling.
//!
//! Internally uses DevdClient for communication with devd and translates
//! DevdCommands into BusMsg for the driver's `command()` callback.
//!
//! ## Event sources
//!
//! The Mux watches:
//! - **devd channel** — commands from devd (SpawnChild, QueryInfo, etc.)
//! - **block port shmem handles** — data port notifications (SQ/CQ activity)
//! - **driver-registered handles** — custom handles via `ctx.watch_handle()`

use crate::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, SpawnContext,
    PortId, KernelBusId, KernelBusInfo, IrqPolicy,
    BlockTransport, BlockPortConfig,
    MAX_PORTS, MAX_KERNEL_BUSES, MAX_PENDING, bus_msg,
};
use crate::bus_block::ShmemBlockPort;
use crate::devd::{DevdClient, DevdCommand};
use crate::ipc::{Channel, Irq, Mux, MuxFilter};
use crate::mailbox::Mailbox;
use crate::query::{QueryHeader, ServiceInfoResult, SpawnChildContext, SpawnContextResponse, msg as query_msg, query_flags, port_type as qport_type, error as query_error};
use crate::syscall::{self, Handle, LogLevel};
use crate::hash_map::HashMap;

// ============================================================================
// Constants
// ============================================================================

/// Maximum block ports (provider + consumer combined).
const MAX_BLOCK_PORTS: usize = MAX_PORTS;

/// Maximum children this driver can relay for.
const MAX_CHILDREN: usize = 8;

/// Maximum forwarded requests awaiting relay response.
const MAX_FORWARDED: usize = 16;

/// Tag for devd channel in handle registry.
const TAG_DEVD: u32 = 0xFFFF_FF00;

/// Tag base for block port shmem handles.
const TAG_BLOCK_PORT_BASE: u32 = 0xFFFF_FE00;

/// Tag base for kernel bus channels.
const TAG_KERNEL_BUS_BASE: u32 = 0xFFFF_FD00;

/// Tag base for child supervision queue handles.
const TAG_CHILD_SUPERQ_BASE: u32 = 0xFFFF_FC00;

/// Tag for parent supervision queue (tree mode — receiving commands from parent).
const TAG_PARENT_SUPERQ: u32 = 0xFFFF_FB00;

/// Maximum managed IRQs per driver.
const MAX_MANAGED_IRQS: usize = 4;

/// Maximum managed timers per driver (tracked for stop/interval change).
const MAX_MANAGED_TIMERS: usize = 4;

// ============================================================================
// Managed IRQ / Timer
// ============================================================================

/// An IRQ managed by the runtime.
/// AutoAck: acked after handle_event (default).
/// DriverManaged: driver calls ack_irq(tag) when ready.
struct ManagedIrq {
    irq: Irq,
    tag: u32,
    policy: IrqPolicy,
}

/// A recurring timer managed by the runtime — kernel auto-rearms via inline timer.
struct ManagedTimer {
    tag: u32,
    interval_ns: u64,
}

// ============================================================================
// Handle Registry
// ============================================================================

/// Maps Mux handles to tags for dispatch.
struct HandleRegistry {
    map: HashMap<Handle, u32>,
}

impl HandleRegistry {
    fn new() -> Self {
        Self { map: HashMap::new() }
    }

    fn add(&mut self, handle: Handle, tag: u32) -> bool {
        self.map.insert(handle, tag);
        true
    }

    fn remove(&mut self, handle: Handle) -> bool {
        self.map.remove(&handle).is_some()
    }

    fn find_tag(&self, handle: Handle) -> Option<u32> {
        self.map.get(&handle).copied()
    }
}

// ============================================================================
// Child Entry (for parent-child relay)
// ============================================================================

/// Spawn context cached for a child — used to answer GET_SPAWN_CONTEXT locally.
struct ChildSpawnCtx {
    port_type: u8,
    port_id: u8,
    port_name: [u8; 64],
    port_name_len: u8,
    metadata: [u8; 64],
    metadata_len: u8,
    shmem_id: u32,
    context_kvs: [([u8; 32], u8, [u8; 64], u8); 4],
    context_kv_count: u8,
}

/// A child spawned by this driver via exec_with_mailbox.
///
/// Communication (both directions via SuperQ):
/// - Child→parent: FORWARD notes via SuperQ up ring
/// - Parent→child: FORWARD notes via SuperQ down ring
/// - Child death: EXIT note via SuperQ + CHILD_EXIT signal
struct ChildEntry {
    /// Shared mailbox page (keeps shmem handle alive; birth context already read).
    mailbox: Mailbox,
    /// Supervision queue for bidirectional parent↔child messaging.
    superq: Option<crate::supervision::SupervisionHandle>,
    /// Child PID.
    pid: u32,
    /// Cached spawn context for path annotation and config forwarding.
    ctx: ChildSpawnCtx,
    /// Binary name of the child (e.g., "pcied", "nvmed").
    binary_name: [u8; 16],
    binary_name_len: u8,
    /// True once the child has sent a bus protocol message.
    bus_active: bool,
}

/// A request forwarded from child to devd, awaiting response relay.
struct ForwardedRequest {
    /// Sequence ID used toward devd (high-bit range).
    devd_seq_id: u32,
    /// Index into children array.
    child_idx: u8,
    /// Original sequence ID from the child.
    child_seq_id: u32,
}

// ============================================================================
// Pending Config Query (async forwarding)
// ============================================================================

/// Maximum concurrent config queries being forwarded to children.
const MAX_PENDING_CONFIG: usize = 4;

/// Who originated the config query (for routing the response back).
#[derive(Clone, Copy)]
enum ConfigQueryOrigin {
    /// Query came from devd (respond via respond_info).
    Devd { seq_id: u32 },
    /// Query came from parent via supervision channel (respond via child channel).
    /// (This variant is used when bus_runtime itself is a child and received
    /// a forwarded config query from its parent.)
    Parent { seq_id: u32 },
}

/// A config query being forwarded to children, awaiting convergence.
///
/// The runtime sends the query to all children simultaneously and waits
/// for each child to respond (either with a real response or EOL).
/// When all children have responded, the aggregated result is sent back
/// to the originator.
struct PendingConfigQuery {
    /// Who asked and where to send the response.
    origin: ConfigQueryOrigin,
    /// CONFIG_GET or CONFIG_SET.
    msg_type: u16,
    /// Bitmask of children the query was sent to (bit N = child index N).
    sent_mask: u8,
    /// Bitmask of children that have responded with EOL (or channel close).
    eol_mask: u8,
    /// Sequence ID used when sending to children.
    child_seq_id: u32,
    /// Accumulated response buffer (for GET: concatenated, for SET: first match).
    response_buf: [u8; 1024],
    /// Bytes written to response_buf.
    response_len: u16,
    /// Deadline sequence for timeout (safety net — should not fire normally).
    deadline_ns: u64,
    /// Bytes of local prefix in response_buf.
    local_prefix_len: u16,
    /// Relay mode: annotate and forward each child response immediately
    /// instead of aggregating into response_buf. Used by @topology to avoid
    /// the 576-byte IPC message limit on deep trees.
    relay: bool,
}

impl PendingConfigQuery {
    fn is_converged(&self) -> bool {
        (self.eol_mask & self.sent_mask) == self.sent_mask
    }
}

// ============================================================================
// Route Action (message routing through supervision tree)
// ============================================================================

/// Routing decision for an inbound ADDRESSED message.
enum RouteAction<'a> {
    /// No route or unaddressed — handle locally (includes child forwarding for CONFIG).
    Local,
    /// Addressed to self — handle locally only, NO child forwarding.
    LocalOnly,
    /// Route targets a specific child — forward only.
    ForwardTo(usize, &'a [u8]),
    /// Route matches this node AND children — handle locally + forward to all.
    LocalAndForward(&'a [u8]),
    /// Route doesn't match this node — forward to all children.
    ForwardAll(&'a [u8]),
}

// ============================================================================
// Kernel Bus Entry
// ============================================================================

/// Kernel bus protocol message types (wire format, first byte of payload).
mod kbus_proto {
    pub const STATE_SNAPSHOT: u8 = 0;
    pub const DEVICE_LIST: u8 = 3;
    // STATE_CHANGED uses supervision protocol (abi::supervision::STATE_CHANGED = 3)
}

/// Maximum enumerated devices per bus
const MAX_BUS_DEVICES: usize = 32;

/// A claimed kernel bus connection.
struct KernelBusEntry {
    channel: Channel,
    info: KernelBusInfo,
    devices: [abi::BusDevice; MAX_BUS_DEVICES],
    device_count: usize,
}

/// Parse a StateSnapshot message (16 bytes from kernel).
fn parse_state_snapshot(data: &[u8]) -> Option<KernelBusInfo> {
    if data.len() < 8 || data[0] != kbus_proto::STATE_SNAPSHOT {
        return None;
    }
    Some(KernelBusInfo {
        bus_type: data[1],
        bus_index: data[2],
        device_count: data[4],
        capabilities: data[5],
    })
}

// ============================================================================
// Pending Request Tracking
// ============================================================================

/// A pending request sent via `send_down()` with a deadline.
struct PendingRequest {
    /// Sequence ID returned to the caller.
    seq: u32,
    /// Absolute deadline in nanoseconds (from `gettime()`).
    /// 0 means no deadline (infinite timeout).
    deadline_ns: u64,
}

// ============================================================================
// Runtime Context
// ============================================================================

/// Spawn context cache state machine.
///
/// ```text
///   NotQueried ──[spawn_context() called]──► Cached(SpawnContext) | NotSpawned
///   Cached     ──[spawn_context() called]──► Cached (returns cached)
///   NotSpawned ──[spawn_context() called]──► NotSpawned (returns NotFound)
/// ```
enum SpawnCtxCache {
    /// Not yet queried from devd.
    NotQueried,
    /// Queried, driver was rule-spawned. Context is immutable.
    Cached(SpawnContext),
    /// Queried, driver was NOT rule-spawned (devd returned nothing).
    NotSpawned,
}

/// Mutable context passed to Driver callbacks via &mut dyn BusCtx.
///
/// This is separated from DriverRuntime so we can borrow it mutably
/// while the driver is also borrowed mutably by the runtime.
struct RuntimeCtx {
    /// Connection to devd (supervision channel).
    devd: DevdClient,
    /// Event multiplexer — single Mux for all event sources.
    mux: Mux,
    /// Handle-to-tag registry for dispatch.
    handles: HandleRegistry,
    /// Block data ports.
    block_ports: [Option<ShmemBlockPort>; MAX_BLOCK_PORTS],
    /// Block port count.
    block_port_count: usize,
    /// Kernel bus connections.
    kernel_buses: [Option<KernelBusEntry>; MAX_KERNEL_BUSES],
    /// Kernel bus count.
    kernel_bus_count: usize,
    /// Next sequence ID for outgoing messages.
    next_seq: u32,
    /// Current command being processed (for reply routing).
    current_cmd_seq: u32,
    /// Current command type (for reply routing).
    current_cmd_type: u32,
    /// Driver name.
    name: [u8; 32],
    /// Driver name length.
    name_len: usize,
    /// Cached spawn context (queried once from devd).
    spawn_ctx: SpawnCtxCache,
    /// Pending requests with deadlines (from `send_down()`).
    pending: [Option<PendingRequest>; MAX_PENDING],
    /// Number of active pending requests.
    pending_count: usize,
    /// Children spawned by this driver (parent-child relay).
    children: [Option<ChildEntry>; MAX_CHILDREN],
    /// Number of active children.
    child_count: usize,
    /// Forwarded requests from children → devd awaiting response relay.
    forwarded: [Option<ForwardedRequest>; MAX_FORWARDED],
    /// Next sequence ID for forwarded requests (high-bit range).
    forwarded_next_seq: u32,
    /// Config queries being forwarded to children (async, non-blocking).
    pending_config: [Option<PendingConfigQuery>; MAX_PENDING_CONFIG],
    /// Next sequence ID for config queries to children (0xC000_xxxx range).
    config_query_next_seq: u32,
    /// Set on first respond_info failure — suppresses subsequent klog spam.
    devd_channel_broken: bool,
    /// Managed IRQs (auto-ack after handle_event).
    managed_irqs: [Option<ManagedIrq>; MAX_MANAGED_IRQS],
    /// Managed timers (auto-rearm after handle_event).
    managed_timers: [Option<ManagedTimer>; MAX_MANAGED_TIMERS],
    /// Last timeout set on Mux (for sticky timeout optimization).
    /// u32::MAX means "no timeout set yet".
    last_timeout_ms: u32,
}

impl RuntimeCtx {
    fn new(devd: DevdClient, mux: Mux, name: &[u8]) -> Self {
        let mut name_buf = [0u8; 32];
        let len = name.len().min(32);
        name_buf[..len].copy_from_slice(&name[..len]);

        Self {
            devd,
            mux,
            handles: HandleRegistry::new(),
            block_ports: [const { None }; MAX_BLOCK_PORTS],
            block_port_count: 0,
            kernel_buses: [const { None }; MAX_KERNEL_BUSES],
            kernel_bus_count: 0,
            next_seq: 1,
            current_cmd_seq: 0,
            current_cmd_type: 0,
            name: name_buf,
            name_len: len,
            spawn_ctx: SpawnCtxCache::NotQueried,
            pending: [const { None }; MAX_PENDING],
            pending_count: 0,
            children: [const { None }; MAX_CHILDREN],
            child_count: 0,
            forwarded: [const { None }; MAX_FORWARDED],
            forwarded_next_seq: 0x4000_0001,
            pending_config: [const { None }; MAX_PENDING_CONFIG],
            config_query_next_seq: 0xC000_0001,
            devd_channel_broken: false,
            managed_irqs: [const { None }; MAX_MANAGED_IRQS],
            managed_timers: [const { None }; MAX_MANAGED_TIMERS],
            last_timeout_ms: u32::MAX,
        }
    }

    fn alloc_seq(&mut self) -> u32 {
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        if self.next_seq == 0 {
            self.next_seq = 1;
        }
        seq
    }

    /// Send an end-of-line response (this node has no answer and no children).
    fn respond_info_eol(&mut self, seq_id: u32) -> Result<(), BusError> {
        let result = self.devd
            .respond_info_eol(seq_id)
            .map_err(|_| BusError::Internal);
        if result.is_err() && !self.devd_channel_broken {
            self.devd_channel_broken = true;
            crate::uerror!("bus", "devd_channel_broken");
        }
        result
    }

    /// Return the nearest deadline in ms from now, or 0 if no pending deadlines.
    ///
    /// Used to set the Mux timeout so the event loop wakes up to fire
    /// expired deadline callbacks even if no I/O event arrives.
    fn nearest_deadline_ms(&self) -> u32 {
        let now = syscall::gettime();
        let mut nearest: u64 = u64::MAX;
        for slot in &self.pending {
            if let Some(req) = slot {
                if req.deadline_ns > 0 && req.deadline_ns < nearest {
                    nearest = req.deadline_ns;
                }
            }
        }
        for slot in &self.pending_config {
            if let Some(pcq) = slot {
                if pcq.deadline_ns > 0 && pcq.deadline_ns < nearest {
                    nearest = pcq.deadline_ns;
                }
            }
        }
        if nearest == u64::MAX {
            return 0;
        }
        if nearest <= now {
            return 1; // Already expired, wake immediately
        }
        // Convert delta to ms, round up
        let delta_ns = nearest - now;
        let ms = (delta_ns + 999_999) / 1_000_000;
        ms.min(u32::MAX as u64) as u32
    }

    /// Drain all expired pending requests. Returns seq IDs of expired ones.
    ///
    /// Caller should invoke `Driver::deadline()` for each returned seq ID.
    fn drain_expired(&mut self, expired: &mut [u32; MAX_PENDING]) -> usize {
        if self.pending_count == 0 {
            return 0;
        }
        let now = syscall::gettime();
        let mut count = 0;
        for slot in &mut self.pending {
            if let Some(req) = slot {
                if req.deadline_ns > 0 && req.deadline_ns <= now {
                    if count < MAX_PENDING {
                        expired[count] = req.seq;
                        count += 1;
                    }
                    *slot = None;
                    self.pending_count -= 1;
                }
            }
        }
        count
    }

    /// Remove a pending request by seq ID (called when response arrives).
    #[allow(dead_code)]
    fn cancel_pending(&mut self, seq: u32) {
        for slot in &mut self.pending {
            if let Some(req) = slot {
                if req.seq == seq {
                    *slot = None;
                    self.pending_count -= 1;
                    return;
                }
            }
        }
    }

    /// Allocate a config query sequence ID (0xC000_xxxx range).
    fn alloc_config_seq(&mut self) -> u32 {
        let seq = self.config_query_next_seq;
        self.config_query_next_seq = self.config_query_next_seq.wrapping_add(1);
        if self.config_query_next_seq < 0xC000_0001 {
            self.config_query_next_seq = 0xC000_0001;
        }
        seq
    }

    /// Find a pending config query by the seq_id used toward children.
    fn find_pending_config(&mut self, child_seq: u32) -> Option<usize> {
        for (i, slot) in self.pending_config.iter().enumerate() {
            if let Some(pcq) = slot {
                if pcq.child_seq_id == child_seq {
                    return Some(i);
                }
            }
        }
        None
    }

    /// Find a free pending config query slot.
    fn alloc_pending_config_slot(&self) -> Option<usize> {
        self.pending_config.iter().position(|s| s.is_none())
    }

    /// Set relay mode on the most recently allocated pending config slot.
    fn set_last_pending_relay(&mut self) {
        // Find the most recently filled slot (last non-None)
        for slot in self.pending_config.iter_mut().rev() {
            if let Some(pcq) = slot {
                pcq.relay = true;
                return;
            }
        }
    }

    /// Check and fire any expired config query deadlines.
    fn drain_expired_config(&mut self) -> [Option<usize>; MAX_PENDING_CONFIG] {
        let mut expired = [None; MAX_PENDING_CONFIG];
        let mut count = 0;
        let now = syscall::gettime();
        for (i, slot) in self.pending_config.iter().enumerate() {
            if let Some(pcq) = slot {
                if pcq.deadline_ns > 0 && pcq.deadline_ns <= now {
                    if count < MAX_PENDING_CONFIG {
                        expired[count] = Some(i);
                        count += 1;
                    }
                }
            }
        }
        expired
    }

    /// Register the devd channel with the Mux.
    fn register_devd_handle(&mut self) {
        if let Some(h) = self.devd.handle() {
            if let Err(e) = self.mux.add(h, MuxFilter::Readable) {
                crate::uerror!("bus", "devd_mux_add_failed"; err = e.as_str());
            }
            self.handles.add(h, TAG_DEVD);
        }
    }

    /// Register a block port's handle with the Mux.
    ///
    /// Uses the doorbell handle if set, otherwise falls back to shmem handle.
    fn register_block_port_handle(&mut self, port_idx: usize) {
        if let Some(ref port) = self.block_ports[port_idx] {
            if let Some(h) = port.mux_handle() {
                let tag = TAG_BLOCK_PORT_BASE + port_idx as u32;
                if let Err(e) = self.mux.add(h, MuxFilter::Readable) {
                    crate::uerror!("bus", "port_mux_add_failed"; err = e.as_str());
                }
                self.handles.add(h, tag);
            }
        }
    }


    /// Remove all children whose trigger port matches `port_name`.
    fn remove_children_for_port(&mut self, port_name: &[u8]) {
        for i in 0..MAX_CHILDREN {
            let matches = self.children[i].as_ref().map(|e| {
                let n = &e.ctx.port_name[..e.ctx.port_name_len as usize];
                n == port_name
            }).unwrap_or(false);
            if matches {
                self.drop_child_entry(i);
            }
        }
    }

    /// Clean up a child entry: unwatch SuperQ from Mux and drop.
    fn drop_child_entry(&mut self, child_idx: usize) {
        if let Some(entry) = &self.children[child_idx] {
            if let Some(ref superq) = entry.superq {
                let h = superq.handle();
                let _ = self.mux.remove(h);
                self.handles.remove(h);
            }
        }
        self.children[child_idx] = None;
        if self.child_count > 0 { self.child_count -= 1; }
    }
}

impl BusCtx for RuntimeCtx {
    // NOTE: reply(), forward_down(), send_up(), and emit_event() are no-ops.
    // Replies use dedicated BusCtx methods (respond_info, ack_spawn, etc.).
    // Forwarding and events are not yet wired — the bus tree currently has at
    // most 2 levels, so there is no upstream to forward to. When multi-level
    // bus trees are needed, these must be implemented.

    fn reply(&mut self, _original: &BusMsg, _payload: &[u8]) -> Result<(), BusError> {
        Ok(())
    }

    fn forward_down(&mut self, _msg: &BusMsg) -> Result<(), BusError> {
        Ok(())
    }

    fn send_down(
        &mut self,
        _msg_type: u32,
        _payload: &[u8],
        deadline_ms: u32,
    ) -> Result<u32, BusError> {
        let seq = self.alloc_seq();

        // Track deadline if non-zero
        if deadline_ms > 0 {
            let deadline_ns = syscall::gettime() + (deadline_ms as u64) * 1_000_000;
            for slot in &mut self.pending {
                if slot.is_none() {
                    *slot = Some(PendingRequest { seq, deadline_ns });
                    self.pending_count += 1;
                    return Ok(seq);
                }
            }
        }

        Ok(seq)
    }

    fn send_up(&mut self, _msg: &BusMsg) -> Result<(), BusError> {
        Ok(())
    }

    fn emit_event(&mut self, _msg_type: u32, _payload: &[u8]) -> Result<(), BusError> {
        Ok(())
    }

    fn create_block_port(&mut self, config: BlockPortConfig) -> Result<PortId, BusError> {
        let port = ShmemBlockPort::create(&config).map_err(|_| BusError::ShmemError)?;

        for (i, slot) in self.block_ports.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(port);
                self.block_port_count += 1;
                // Register with Mux for event-driven wakeup
                self.register_block_port_handle(i);
                return Ok(PortId(i as u8));
            }
        }
        Err(BusError::NoSpace)
    }

    fn connect_block_port(&mut self, shmem_id: u32) -> Result<PortId, BusError> {
        let port = ShmemBlockPort::connect(shmem_id).map_err(|_| BusError::ShmemError)?;

        for (i, slot) in self.block_ports.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(port);
                self.block_port_count += 1;
                // Register with Mux for event-driven wakeup
                self.register_block_port_handle(i);
                return Ok(PortId(i as u8));
            }
        }
        Err(BusError::NoSpace)
    }

    fn block_port(&mut self, id: PortId) -> Option<&mut dyn BlockTransport> {
        let idx = id.0 as usize;
        if idx < MAX_BLOCK_PORTS {
            self.block_ports[idx]
                .as_mut()
                .map(|p| p as &mut dyn BlockTransport)
        } else {
            None
        }
    }

    fn claim_kernel_bus(&mut self, path: &[u8]) -> Result<(KernelBusId, KernelBusInfo), BusError> {
        if self.kernel_bus_count >= MAX_KERNEL_BUSES {
            return Err(BusError::NoSpace);
        }

        // Connect to the kernel bus port (as owner — supervisor already connected)
        let mut ch = Channel::connect(path).map_err(|_| BusError::LinkDown)?;

        // Receive initial StateSnapshot (kernel sends on connect)
        // Use large buffer to also handle DeviceList if it arrives in same recv
        let mut buf = [0u8; 576];
        let n = ch.recv(&mut buf).map_err(|_| BusError::LinkDown)?;
        let info = parse_state_snapshot(&buf[..n]).ok_or(BusError::InvalidMessage)?;

        // Receive DeviceList message(s) if devices are present
        let mut devices = [abi::BusDevice::empty(); MAX_BUS_DEVICES];
        let mut device_count: usize = 0;
        let total_expected = info.device_count as usize;

        while device_count < total_expected {
            let n = ch.recv(&mut buf).map_err(|_| BusError::LinkDown)?;
            if n < 8 || buf[0] != kbus_proto::DEVICE_LIST {
                break; // Unexpected message, stop
            }
            let _total = buf[1] as usize;
            let offset = buf[2] as usize;
            let count = buf[3] as usize;
            let dev_start = 8;
            let dev_end = dev_start + count * 32;
            if n < dev_end {
                break; // Message too short
            }
            for i in 0..count {
                let idx = offset + i;
                if idx >= MAX_BUS_DEVICES {
                    break;
                }
                let src = dev_start + i * 32;
                let dev_bytes = &buf[src..src + 32];
                devices[idx] = unsafe { *(dev_bytes.as_ptr() as *const abi::BusDevice) };
                if idx + 1 > device_count {
                    device_count = idx + 1;
                }
            }
        }

        // Register the channel with Mux for ongoing messages
        let bus_idx = self.kernel_bus_count;
        let tag = TAG_KERNEL_BUS_BASE + bus_idx as u32;
        let ch_handle = ch.handle();
        self.mux.add(ch_handle, MuxFilter::Readable).map_err(|_| BusError::Internal)?;
        if !self.handles.add(ch_handle, tag) {
            let _ = self.mux.remove(ch_handle);
            return Err(BusError::NoSpace);
        }

        // Store the entry
        self.kernel_buses[bus_idx] = Some(KernelBusEntry {
            channel: ch,
            info,
            devices,
            device_count,
        });
        self.kernel_bus_count += 1;

        Ok((KernelBusId(bus_idx as u8), info))
    }

    fn kernel_bus_info(&self, id: KernelBusId) -> Option<&KernelBusInfo> {
        let idx = id.0 as usize;
        self.kernel_buses.get(idx)?.as_ref().map(|e| &e.info)
    }

    fn bus_devices(&self, id: KernelBusId) -> Option<&[abi::BusDevice]> {
        let idx = id.0 as usize;
        let entry = self.kernel_buses.get(idx)?.as_ref()?;
        Some(&entry.devices[..entry.device_count])
    }

    fn enable_bus_mastering(&mut self, bus_id: KernelBusId, device_bdf: u16) -> Result<(), BusError> {
        let idx = bus_id.0 as usize;
        let entry = self.kernel_buses.get_mut(idx)
            .and_then(|e| e.as_mut())
            .ok_or(BusError::InvalidMessage)?;

        // Build EnableBusMastering message: [type(1), device_id(2)]
        let mut msg = [0u8; 3];
        msg[0] = 16; // EnableBusMastering
        msg[1..3].copy_from_slice(&device_bdf.to_le_bytes());
        entry.channel.send(&msg).map_err(|_| BusError::LinkDown)?;
        Ok(())
    }

    fn bus_send(&mut self, bus_id: KernelBusId, msg: &[u8]) -> Result<(), BusError> {
        let idx = bus_id.0 as usize;
        let entry = self.kernel_buses.get_mut(idx)
            .and_then(|e| e.as_mut())
            .ok_or(BusError::InvalidMessage)?;
        entry.channel.send(msg).map_err(|_| BusError::LinkDown)
    }

    fn watch_handle(&mut self, handle: Handle, tag: u32) -> Result<(), BusError> {
        self.mux.add(handle, MuxFilter::Readable).map_err(|_| BusError::Internal)?;
        if !self.handles.add(handle, tag) {
            let _ = self.mux.remove(handle);
            return Err(BusError::NoSpace);
        }
        Ok(())
    }

    fn unwatch_handle(&mut self, handle: Handle) -> Result<(), BusError> {
        let _ = self.mux.remove(handle);
        self.handles.remove(handle);
        Ok(())
    }

    fn watch_irq(&mut self, irq_num: u32, tag: u32) -> Result<(), BusError> {
        self.watch_irq_with_policy(irq_num, tag, IrqPolicy::AutoAck)
    }

    fn watch_irq_with_policy(&mut self, irq_num: u32, tag: u32, policy: IrqPolicy) -> Result<(), BusError> {
        let irq = Irq::new(irq_num).map_err(|_| BusError::NotFound)?;
        let handle = irq.handle();
        self.mux.add(handle, MuxFilter::Readable).map_err(|_| BusError::Internal)?;
        if !self.handles.add(handle, tag) {
            let _ = self.mux.remove(handle);
            return Err(BusError::NoSpace);
        }
        for slot in self.managed_irqs.iter_mut() {
            if slot.is_none() {
                *slot = Some(ManagedIrq { irq, tag, policy });
                return Ok(());
            }
        }
        // No managed slot — undo mux/handle registration
        let _ = self.mux.remove(handle);
        self.handles.remove(handle);
        Err(BusError::NoSpace)
    }

    fn ack_irq(&mut self, tag: u32) {
        for slot in self.managed_irqs.iter_mut() {
            if let Some(mirq) = slot {
                if mirq.tag == tag {
                    let _ = mirq.irq.ack();
                    return;
                }
            }
        }
    }

    fn set_irq_affinity(&mut self, tag: u32, cpu: u32) -> Result<(), BusError> {
        for slot in self.managed_irqs.iter_mut() {
            if let Some(mirq) = slot {
                if mirq.tag == tag {
                    mirq.irq.set_affinity(cpu).map_err(|_| BusError::Internal)?;
                    return Ok(());
                }
            }
        }
        Err(BusError::NotFound)
    }

    fn start_timer(&mut self, tag: u32, interval_ns: u64) -> Result<(), BusError> {
        self.mux.add_recurring_timer(tag, interval_ns).map_err(|_| BusError::Internal)?;
        for slot in self.managed_timers.iter_mut() {
            if slot.is_none() {
                *slot = Some(ManagedTimer { tag, interval_ns });
                return Ok(());
            }
        }
        let _ = self.mux.remove_timer(tag);
        Err(BusError::NoSpace)
    }

    fn stop_timer(&mut self, tag: u32) -> Result<(), BusError> {
        for slot in self.managed_timers.iter_mut() {
            if let Some(mt) = slot {
                if mt.tag == tag {
                    let _ = self.mux.remove_timer(tag);
                    *slot = None;
                    return Ok(());
                }
            }
        }
        Err(BusError::NotFound)
    }

    fn set_timer_interval(&mut self, tag: u32, interval_ns: u64) -> Result<(), BusError> {
        self.mux.set_timer_interval(tag, interval_ns).map_err(|_| BusError::NotFound)?;
        for slot in self.managed_timers.iter_mut() {
            if let Some(mt) = slot {
                if mt.tag == tag {
                    mt.interval_ns = interval_ns;
                    return Ok(());
                }
            }
        }
        Err(BusError::NotFound)
    }

    fn name(&self) -> &[u8] {
        &self.name[..self.name_len]
    }

    fn respond_info(&mut self, seq_id: u32, info: &[u8]) -> Result<(), BusError> {
        let result = self.devd
            .respond_info(seq_id, info)
            .map_err(|_| BusError::Internal);
        if result.is_err() && !self.devd_channel_broken {
            self.devd_channel_broken = true;
            crate::uerror!("bus", "devd_channel_broken");
        }
        result
    }

    fn register_port_with_info(
        &mut self,
        info: &abi::PortInfo,
        shmem_id: u32,
    ) -> Result<(), BusError> {
        // Auto-set parent_port_id from spawn context if the driver didn't set it.
        // This builds the correct port hierarchy in devd's registry.
        let mut info_copy;
        let info_to_send = if info.parent_port_id == 0xFF {
            if let SpawnCtxCache::Cached(ref ctx) = self.spawn_ctx {
                if ctx.port_id != 0xFF {
                    info_copy = *info;
                    info_copy.parent_port_id = ctx.port_id;
                    &info_copy
                } else {
                    info
                }
            } else {
                info
            }
        } else {
            info
        };

        // Register with devd via supervision channel
        // Port starts in Safe state — supervisor will fire rules
        self.devd
            .register_port_info(info_to_send, shmem_id)
            .map_err(|_| BusError::Internal)?;

        // Log port registration from the framework — drivers don't need to
        if let Ok(name_str) = core::str::from_utf8(info_to_send.name_bytes()) {
            crate::uinfo!("bus", "port_registered";
                driver = core::str::from_utf8(&self.name[..self.name_len]).unwrap_or("?"),
                port = name_str,
                class = info_to_send.port_class.as_str()
            );
        }

        Ok(())
    }

    fn set_port_state(
        &mut self,
        name: &[u8],
        state: abi::PortState,
    ) -> Result<(), BusError> {
        // Port going Safe: remove children spawned for this port.
        // The EXIT note hasn't arrived yet (microtask delay), so the stale
        // child entry would cause the next ADDRESSED SpawnChild to be
        // forwarded to the dead child's SuperQ instead of handled locally.
        if state == abi::PortState::Safe {
            self.remove_children_for_port(name);
        }

        // Log state transition from the framework — consistent across all drivers
        crate::uinfo!("bus", "port_transition";
            driver = core::str::from_utf8(&self.name[..self.name_len]).unwrap_or("?"),
            port = core::str::from_utf8(name).unwrap_or("?"),
            state = state.as_str()
        );

        self.devd
            .set_port_state(name, state)
            .map_err(|_| BusError::Internal)
    }

    fn spawn_context(&mut self) -> Result<&SpawnContext, BusError> {
        // Query devd (or parent relay) on first call, cache the result.
        //
        // The response may include an extra 4 bytes of shmem_id LE appended
        // after the standard SpawnContextResponse (sent by parent relay).
        // Root-mode devd responses omit this — shmem_id defaults to 0.
        if matches!(self.spawn_ctx, SpawnCtxCache::NotQueried) {
            match self.devd.get_spawn_context() {
                Ok(Some((name_buf, name_len, port_class, meta_buf, meta_len, port_id, kvs, kv_count, shmem_id))) => {
                    let mut ctx = SpawnContext::new(&name_buf, name_len, port_class, &meta_buf[..meta_len], port_id);
                    if kv_count > 0 {
                        ctx.set_context_kvs(&kvs, kv_count);
                    }
                    ctx.shmem_id = shmem_id;
                    self.spawn_ctx = SpawnCtxCache::Cached(ctx);
                }
                Ok(None) => {
                    self.spawn_ctx = SpawnCtxCache::NotSpawned;
                }
                Err(_) => {
                    // Transport error — don't cache, let caller retry or fail
                    return Err(BusError::LinkDown);
                }
            }
        }

        match &self.spawn_ctx {
            SpawnCtxCache::Cached(ctx) => Ok(ctx),
            SpawnCtxCache::NotSpawned => Err(BusError::NotFound),
            SpawnCtxCache::NotQueried => unreachable!(),
        }
    }

    fn discover_port(&mut self) -> Result<u32, BusError> {
        // Check cached spawn context first — parent relay provides shmem_id
        // directly, avoiding a round-trip query to devd.
        if let SpawnCtxCache::Cached(ctx) = &self.spawn_ctx {
            if ctx.shmem_id != 0 {
                return Ok(ctx.shmem_id);
            }
        }

        // Use port_id from spawn context — unambiguous tree-based identity
        let port_id = match &self.spawn_ctx {
            SpawnCtxCache::Cached(ctx) if ctx.port_id != 0xFF => ctx.port_id,
            SpawnCtxCache::Cached(_) => return Err(BusError::NotFound),
            SpawnCtxCache::NotSpawned => return Err(BusError::NotFound),
            SpawnCtxCache::NotQueried => {
                // Query spawn context first
                if let Err(e) = BusCtx::spawn_context(self).map(|_| ()) {
                    return Err(e);
                }
                // Re-check shmem_id from freshly cached context
                if let SpawnCtxCache::Cached(ctx) = &self.spawn_ctx {
                    if ctx.shmem_id != 0 {
                        return Ok(ctx.shmem_id);
                    }
                    if ctx.port_id != 0xFF { ctx.port_id } else { return Err(BusError::NotFound); }
                } else {
                    return Err(BusError::NotFound);
                }
            }
        };

        match self.devd.query_port_shmem_id_by_id(port_id) {
            Ok(Some(shmem_id)) => Ok(shmem_id),
            Ok(None) => Err(BusError::NotFound),
            Err(_) => Err(BusError::LinkDown),
        }
    }

    fn discover_port_by_name(&mut self, name: &[u8]) -> Result<u32, BusError> {
        match self.devd.query_port_shmem_id(name) {
            Ok(Some(shmem_id)) => Ok(shmem_id),
            Ok(None) => Err(BusError::NotFound),
            Err(_) => Err(BusError::LinkDown),
        }
    }

    fn register_mount(&mut self, prefix: &[u8], shmem_id: u32) -> Result<(), BusError> {
        self.devd.register_mount(prefix, shmem_id)
            .map_err(|_| BusError::Internal)
    }

}

// ============================================================================
// DevdCommand → BusMsg Translation
// ============================================================================

/// Translate a DevdCommand into a BusMsg for the driver's command() callback.
fn devd_command_to_bus_msg(cmd: &DevdCommand) -> Option<BusMsg> {
    match cmd {
        DevdCommand::QueryInfo { seq_id } => {
            let mut msg = BusMsg::new(bus_msg::QUERY_INFO);
            msg.seq_id = *seq_id;
            Some(msg)
        }
        DevdCommand::ConfigGet { seq_id, key, key_len } => {
            let mut msg = BusMsg::new(bus_msg::CONFIG_GET);
            msg.seq_id = *seq_id;
            msg.write_bytes(0, &key[..*key_len]);
            msg.write_u8(*key_len, 0); // null terminator
            msg.payload_len = (*key_len + 1) as u16;
            Some(msg)
        }
        DevdCommand::ConfigSet { seq_id, key, key_len, value, value_len } => {
            let mut msg = BusMsg::new(bus_msg::CONFIG_SET);
            msg.seq_id = *seq_id;
            msg.write_bytes(0, &key[..*key_len]);
            msg.write_u8(*key_len, 0); // null separator
            msg.write_bytes(*key_len + 1, &value[..*value_len]);
            msg.write_u8(*key_len + 1 + *value_len, 0); // null terminator
            msg.payload_len = (*key_len + 1 + *value_len + 1) as u16;
            Some(msg)
        }
        // SpawnChild/StopChild handled by framework (supervision protocol),
        // not dispatched to driver
        DevdCommand::SpawnChild { .. } | DevdCommand::StopChild { .. } => None,
    }
}

// ============================================================================
// DriverRuntime
// ============================================================================

/// Routing decision for the current dispatch cycle.
///
/// Set before `dispatch_raw_command()`, consumed during dispatch, reset after.
/// Single source of truth — replaces the old routed_child/skip_children/pending_child_route fields.
enum RoutingMode {
    /// Normal dispatch: local handling + forward to all children.
    Broadcast,
    /// Addressed to self: local handling only, no children.
    LocalOnly,
    /// Route to specific child (terminal — no remaining route).
    ForwardTo { child: usize },
    /// Route to specific child with remaining sub-route.
    ForwardWithRoute { child: usize, route: [u8; 128], route_len: usize },
}

impl RoutingMode {
    /// Target child index for routed modes.
    fn target_child(&self) -> Option<usize> {
        match self {
            RoutingMode::ForwardTo { child } | RoutingMode::ForwardWithRoute { child, .. } => Some(*child),
            _ => None,
        }
    }

    /// True if this is a routed mode (ForwardTo or ForwardWithRoute).
    fn is_routed(&self) -> bool {
        matches!(self, RoutingMode::ForwardTo { .. } | RoutingMode::ForwardWithRoute { .. })
    }

    /// True if local handling should be skipped (ForwardWithRoute has sub-route = skip self).
    fn skip_local(&self) -> bool {
        matches!(self, RoutingMode::ForwardTo { .. } | RoutingMode::ForwardWithRoute { .. })
    }

    /// True if we should forward to children (everything except LocalOnly).
    fn forward_to_children(&self) -> bool {
        !matches!(self, RoutingMode::LocalOnly)
    }
}

/// The main runtime that manages the event loop and dispatches to Driver.
///
/// Uses a Mux-based event loop: blocks until an event source fires,
/// dispatches to the appropriate Driver callback, then blocks again.
/// No polling, no sleep loops.
pub struct DriverRuntime<D: Driver> {
    driver: D,
    ctx: RuntimeCtx,
    /// Current routing decision (set before dispatch, reset after).
    routing: RoutingMode,
}

impl<D: Driver> DriverRuntime<D> {
    /// Create a new runtime with the given driver and devd connection.
    fn new(driver: D, devd: DevdClient, mux: Mux, name: &[u8]) -> Self {
        Self {
            driver,
            ctx: RuntimeCtx::new(devd, mux, name),
            routing: RoutingMode::Broadcast,
        }
    }

    /// Run the event loop. Never returns.
    fn run(&mut self) -> ! {
        // Register devd channel with Mux
        self.ctx.register_devd_handle();

        // In tree mode, register our SuperQ handle so we get woken when
        // parent sends us commands via the kernel supervision queue.
        if let Some(h) = self.ctx.devd.superq_obj_handle() {
            if let Err(e) = self.ctx.mux.add(h, MuxFilter::Readable) {
                crate::uerror!("bus", "superq_mux_add_failed"; err = e.as_str());
            }
            self.ctx.handles.add(h, TAG_PARENT_SUPERQ);
        }

        // Initialize the driver (hardware is Safe — bring it up)
        if let Err(e) = self.driver.reset(&mut self.ctx) {
            let err_name = match e {
                BusError::LinkDown => "LinkDown",
                BusError::BufferFull => "BufferFull",
                BusError::InvalidMessage => "InvalidMessage",
                BusError::NoSpace => "NoSpace",
                BusError::NotFound => "NotFound",
                BusError::Timeout => "Timeout",
                BusError::SpawnFailed => "SpawnFailed",
                BusError::ShmemError => "ShmemError",
                BusError::Internal => "Internal",
                BusError::NotPresent => "NotPresent",
            };
            crate::ulog::flush();
            crate::uerror!("bus", "reset_failed"; err = err_name);
            // NotPresent = hardware absent, exit cleanly (devd won't respawn)
            // Other errors = crash, exit with code 1 (devd will respawn)
            let code = if matches!(e, BusError::NotPresent) { 0 } else { 1 };
            syscall::exit(code);
        }

        // Report Ready to devd — transitions service from Starting to Ready
        let _ = self.ctx.devd.report_state(crate::devd::DriverState::Ready);

        // Flush structured logs from reset before entering event loop
        crate::ulog::flush();

        // Event loop: block on Mux, dispatch deadlines, repeat
        loop {
            // Fire any expired deadlines before blocking
            let mut expired = [0u32; MAX_PENDING];
            let n_expired = self.ctx.drain_expired(&mut expired);
            for i in 0..n_expired {
                self.driver.deadline(expired[i], &mut self.ctx);
            }

            // Complete any timed-out config queries
            let expired_configs = self.ctx.drain_expired_config();
            for idx_opt in &expired_configs {
                if let Some(slot) = idx_opt {
                    // Timeout is a safety net — should not fire in normal operation.
                    // Log ERROR so we can investigate stuck queries.
                    crate::uerror!("bus", "config_query_timeout");
                    self.complete_config_query(*slot);
                }
            }

            // Block until next event or nearest deadline.
            // Sticky timeout: only call set_timeout when the value changes,
            // reducing 3 syscalls (set+wait+clear) to 1 (wait) per iteration.
            let timeout_ms = self.ctx.nearest_deadline_ms();
            if timeout_ms != self.ctx.last_timeout_ms {
                let _ = self.ctx.mux.set_timeout(timeout_ms);
                self.ctx.last_timeout_ms = timeout_ms;
            }
            let event = match self.ctx.mux.wait() {
                Ok(ev) => ev,
                Err(_) => continue,
            };

            // Signal events have Handle::INVALID — dispatch before tag lookup
            if event.is_signal() {
                let sig = event.signal_event as u32;
                let val = event.signal_value;

                // CHILD_EXIT: child died — clean up
                if sig & abi::signal_event::CHILD_EXIT != 0 {
                    let child_pid = (val >> 32) as u32;
                    self.handle_child_exit(child_pid);
                }

                // Pass all signals to the driver (including CHILD_EXIT —
                // bus-managed children are already handled above, but drivers
                // that spawn children directly need to see CHILD_EXIT too).
                self.driver.signal(event.signal_event, event.signal_value, &mut self.ctx);
                continue;
            }

            let handle = event.handle;

            // Inline timer events: handle field carries the tag directly
            if event.event == abi::mux_filter::TIMER {
                let tag = handle.0;
                self.driver.handle_event(tag, Handle(0), &mut self.ctx);
                crate::ulog::flush();
                continue;
            }

            // Look up what this handle maps to
            let tag = match self.ctx.handles.find_tag(handle) {
                Some(t) => t,
                None => continue, // Unknown handle, skip
            };

            if tag == TAG_DEVD {
                self.handle_devd_event(handle);
            } else if tag == TAG_PARENT_SUPERQ {
                // Parent sent us a command via SuperQ (tree mode)
                self.handle_parent_superq_command();
            } else if tag >= TAG_CHILD_SUPERQ_BASE && tag < TAG_KERNEL_BUS_BASE {
                // Child SuperQ readable — child sent a message via supervision queue
                let child_idx = (tag - TAG_CHILD_SUPERQ_BASE) as usize;
                self.handle_child_superq_event(child_idx);
            } else if tag >= TAG_KERNEL_BUS_BASE && tag < TAG_BLOCK_PORT_BASE {
                // Kernel bus state notification
                let bus_idx = (tag - TAG_KERNEL_BUS_BASE) as usize;
                self.handle_kernel_bus_event(bus_idx);
            } else if tag >= TAG_BLOCK_PORT_BASE && tag < TAG_DEVD {
                // Block port notification (doorbell or shmem)
                let port_idx = (tag - TAG_BLOCK_PORT_BASE) as usize;
                if port_idx < MAX_BLOCK_PORTS {
                    // Ack the doorbell to prevent spurious re-fires
                    if let Some(ref port) = self.ctx.block_ports[port_idx] {
                        port.ack();
                    }
                    self.driver.data_ready(PortId(port_idx as u8), &mut self.ctx);
                }
            } else {
                // Driver-registered handle
                self.driver.handle_event(tag, handle, &mut self.ctx);

                // Auto-ack managed IRQs (only AutoAck policy)
                for slot in self.ctx.managed_irqs.iter_mut() {
                    if let Some(mirq) = slot {
                        if mirq.tag == tag {
                            if mirq.policy == IrqPolicy::AutoAck {
                                let _ = mirq.irq.ack();
                            }
                            break;
                        }
                    }
                }

            }

            // Flush any structured logs emitted during dispatch
            crate::ulog::flush();
        }
    }

    /// Handle a kernel bus channel event (state snapshot or supervision message).
    fn handle_kernel_bus_event(&mut self, bus_idx: usize) {
        let mut buf = [0u8; 32];

        // Drain all pending messages from the bus channel
        loop {
            let n = match &mut self.ctx.kernel_buses[bus_idx] {
                Some(entry) => match entry.channel.try_recv(&mut buf) {
                    Ok(Some(n)) => n,
                    _ => break,
                },
                None => break,
            };

            if n == 0 { continue; }

            match buf[0] {
                kbus_proto::STATE_SNAPSHOT => {
                    if let Some(new_info) = parse_state_snapshot(&buf[..n]) {
                        if let Some(entry) = &mut self.ctx.kernel_buses[bus_idx] {
                            entry.info = new_info;
                        }
                    }
                }
                abi::supervision::STATE_CHANGED if n >= 4 => {
                    // Supervision protocol: [STATE_CHANGED, old, new, reason]
                    // We're the owner — kernel is telling us about state changes
                    // (normally we wouldn't see these since we ARE the owner,
                    //  but they can arrive on Reset/Safe transitions)
                }
                other => {
                    // Forward unknown message types to driver
                    let bus_id = KernelBusId(bus_idx as u8);
                    self.driver.bus_event(bus_id, other, &buf[1..n], &mut self.ctx);
                }
            }
        }
    }

    /// Strip any ADDRESSED route from raw bytes and dispatch as a command.
    ///
    /// This is the entry point for locally-handled messages after routing.
    fn dispatch_raw_command(&mut self, raw: &[u8]) {
        // Try path-based forwarding on the RAW message (route intact).
        // If the message has an ADDRESSED route and the first segment
        // matches a child's port name, forward the message with the
        // consumed segment removed.
        if self.try_forward_spawn_to_child(raw) {
            return;
        }

        // Strip route for local dispatch
        let (stripped, slen) = Self::strip_route(raw);

        // Diagnostic: log raw command details for SpawnChild
        if slen >= QueryHeader::SIZE {
            let msg_type = u16::from_le_bytes([stripped[0], stripped[1]]);
            if msg_type == crate::query::msg::SPAWN_CHILD && slen >= 20 {
                let ctx_len = stripped[10];
                crate::udebug!("bus", "strip_spawn"; raw_len = raw.len() as u32, stripped_len = slen as u32, ctx_len = ctx_len as u32);
            }
        }

        match DevdClient::parse_command_buf(&stripped[..slen]) {
            Ok(Some(cmd)) => {
                self.dispatch_devd_command(cmd);
            }
            Ok(None) => {}
            Err(e) => {
                crate::uwarn!("bus", "devd_cmd_parse_failed"; err = e.as_str());
            }
        }
    }

    /// Try to forward a SpawnChild to the correct child using path-based routing.
    ///
    /// Operates on the RAW message (with ADDRESSED route intact). Walks the
    /// route segments using the same `:` disambiguation as `route_inbound`:
    /// - Segments with `:` are port names → match children or skip self's port
    /// - Segments without `:` are driver names → verify matches self, skip
    ///
    /// Returns true if forwarded (caller should skip normal dispatch).
    fn try_forward_spawn_to_child(&mut self, raw: &[u8]) -> bool {
        if raw.len() < QueryHeader::SIZE {
            return false;
        }

        // Quick check: is this a SPAWN_CHILD message?
        let msg_type = u16::from_le_bytes([raw[0], raw[1]]);
        if msg_type != query_msg::SPAWN_CHILD {
            return false;
        }

        // Check ADDRESSED flag — if not set, no path routing
        let flags = u16::from_le_bytes([raw[2], raw[3]]);
        if flags & query_flags::ADDRESSED == 0 {
            return false;
        }

        // Extract route
        if raw.len() <= QueryHeader::SIZE {
            return false;
        }
        let route_len = raw[QueryHeader::SIZE] as usize;
        let route_start = QueryHeader::SIZE + 1;
        let route_end = route_start + route_len;
        if route_end > raw.len() || route_len == 0 {
            return false;
        }
        let route = &raw[route_start..route_end];

        // Walk segments — skip self's trigger port and driver name
        let mut path = if !route.is_empty() && route[0] == b'/' {
            &route[1..]
        } else {
            route
        };

        loop {
            if path.is_empty() {
                return false;
            }

            let (segment, rest) = match path.iter().position(|&b| b == b'/') {
                Some(pos) => (&path[..pos], &path[pos + 1..]),
                None => (path, &[] as &[u8]),
            };

            if segment.is_empty() {
                return false;
            }

            if segment.iter().any(|&b| b == b':') {
                // Port segment — try child match first
                if let Some(child_idx) = self.find_child_by_port_name(segment) {
                    return self.forward_spawn_message(child_idx, rest, raw, flags, route_end);
                } else if self.matches_own_trigger_port(segment) {
                    path = rest;
                    continue;
                } else {
                    return false;
                }
            } else {
                // Driver name — if matches self, skip
                if self.matches_own_driver_name(segment) {
                    path = rest;
                    continue;
                } else {
                    return false;
                }
            }
        }
    }

    /// Forward a SpawnChild message to a child with a rewritten route.
    fn forward_spawn_message(
        &mut self,
        child_idx: usize,
        remaining: &[u8],
        raw: &[u8],
        flags: u16,
        route_end: usize,
    ) -> bool {
        let payload = &raw[route_end..];
        let mut fwd = [0u8; 576];

        let total = if remaining.is_empty() {
            // Last segment consumed — send without ADDRESSED flag
            fwd[..QueryHeader::SIZE].copy_from_slice(&raw[..QueryHeader::SIZE]);
            let new_flags = flags & !query_flags::ADDRESSED;
            fwd[2..4].copy_from_slice(&new_flags.to_le_bytes());
            let plen = payload.len().min(576 - QueryHeader::SIZE);
            fwd[QueryHeader::SIZE..QueryHeader::SIZE + plen].copy_from_slice(&payload[..plen]);
            QueryHeader::SIZE + plen
        } else {
            // More segments remain — rewrite route to remaining path
            let new_route_len = remaining.len();
            if new_route_len > 255 {
                return false;
            }
            fwd[..QueryHeader::SIZE].copy_from_slice(&raw[..QueryHeader::SIZE]);
            // ADDRESSED stays set
            fwd[QueryHeader::SIZE] = new_route_len as u8;
            let new_route_start = QueryHeader::SIZE + 1;
            fwd[new_route_start..new_route_start + new_route_len].copy_from_slice(remaining);
            let out_payload_start = new_route_start + new_route_len;
            let plen = payload.len().min(576 - out_payload_start);
            fwd[out_payload_start..out_payload_start + plen].copy_from_slice(&payload[..plen]);
            out_payload_start + plen
        };

        // Send via mailbox
        self.send_to_child_superq(child_idx, &fwd[..total]);
        true
    }

    /// Dispatch a DevdCommand to the driver.
    ///
    /// All commands go through BusMsg — single conversion path.
    /// The framework intercepts known types (spawn, stop, config) before
    /// the driver sees them. Everything else goes to `driver.command()`.
    fn dispatch_devd_command(&mut self, cmd: DevdCommand) {
        // SpawnChild: framework spawns the child with a shared mailbox page.
        //
        // The mailbox contains the spawn context (MailboxHeader + KVs) so the
        // child reads it at boot without a GET_SPAWN_CONTEXT round-trip.
        // Post-spawn communication uses mailbox writes + signals:
        // - Parent writes commands at offset 64, signals child with CONFIG
        // - Child writes responses at offset 2048, signals parent with MAILBOX
        if let DevdCommand::SpawnChild { seq_id, binary, binary_len, caps, filter, priority, context } = &cmd {
            let name = core::str::from_utf8(&binary[..*binary_len]).unwrap_or("???");

            // Diagnostic: log context presence and metadata length
            let ctx_meta_len = context.as_ref().map(|c| c.metadata_len).unwrap_or(0);
            crate::udebug!("bus", "spawn_child"; child = name, has_ctx = context.is_some() as u32, ctx_meta = ctx_meta_len as u32);

            // Build mailbox content: MailboxHeader + spawn context KVs
            let mut mb_buf = [0u8; 4096];
            build_child_mailbox(&mut mb_buf, filter, context.as_ref(), *priority);

            // Spawn with mailbox — parent gets shmem handle + superq handle back
            let spawn_result = match syscall::exec_with_mailbox(name, *caps, &mb_buf) {
                Ok((pid, parent_mb_handle, parent_superq_handle)) => {
                    match Mailbox::from_handle(parent_mb_handle) {
                        Ok(mb) => {
                            let superq = crate::supervision::SupervisionHandle::from_handle(parent_superq_handle);
                            Some((mb, superq, pid))
                        }
                        Err(e) => {
                            crate::uwarn!("bus", "mailbox_map_failed"; err = e.as_str(), handle = parent_mb_handle.raw());
                            None
                        }
                    }
                }
                Err(e) => {
                    crate::uwarn!("bus", "exec_with_mb_failed"; errno = e);
                    None
                }
            };

            let (result, child_pid) = if let Some((mailbox, superq, pid)) = spawn_result {
                let stored = self.store_child_entry_with_superq(mailbox, superq, pid, filter, context.as_ref(), &binary[..*binary_len]);
                if !stored {
                    crate::uwarn!("bus", "no_child_slot");
                }
                (0i32, pid)
            } else {
                (-1i32, 0u32)
            };

            if self.ctx.devd.ack_spawn(*seq_id, result, child_pid).is_err() {
                crate::uwarn!("bus", "spawn_ack_failed");
            }
            return;
        }

        let bus_msg = match devd_command_to_bus_msg(&cmd) {
            Some(msg) => msg,
            None => return,
        };

        self.ctx.current_cmd_seq = bus_msg.seq_id;
        self.ctx.current_cmd_type = bus_msg.msg_type;

        match bus_msg.msg_type {
            // --- Framework-handled: configuration ---

            bus_msg::CONFIG_GET => {
                let key = bus_msg.read_null_terminated(0);

                // Special: @topology forces every driver to respond with driver=<name>.
                // Forwarded to children via normal config path — source annotation
                // builds the full path through the supervision tree.
                if key == b"@topology" {
                    self.handle_topology_query(bus_msg.seq_id);
                    return;
                }

                let mut buf = [0u8; 512];
                let skip_local = self.routing.skip_local();
                let forward_to_children = self.ctx.child_count > 0
                    && self.routing.forward_to_children();

                // Note: no self-annotation here. The PARENT always annotates
                // our response with [trigger_port/driver_name] via append_with_source.

                if key.is_empty() {
                    // Summary: send local config immediately, relay children's
                    // responses individually. Relay mode avoids accumulating a
                    // response that might exceed the 576-byte IPC message limit.
                    let len = if skip_local { 0 } else {
                        build_config_summary(&self.driver, &mut buf)
                    };

                    // Send own local config immediately (before children)
                    if len > 0 {
                        let _ = self.ctx.respond_info(bus_msg.seq_id, &buf[..len]);
                    }

                    if forward_to_children {
                        let origin = ConfigQueryOrigin::Devd { seq_id: bus_msg.seq_id };
                        // Empty local_prefix — we already sent local data above.
                        // Relay mode: each child's response is annotated and
                        // forwarded immediately instead of accumulated.
                        if self.start_config_forward(
                            origin, query_msg::CONFIG_GET, key, &[], &[],
                        ) {
                            self.ctx.set_last_pending_relay();
                            return; // EOL sent when children converge
                        }
                        // Fallback: no children sent — send EOL now
                        let _ = self.ctx.respond_info_eol(bus_msg.seq_id);
                    } else {
                        // No children (leaf) or routed-to-child with no children
                        let _ = self.ctx.respond_info_eol(bus_msg.seq_id);
                    }
                } else {
                    if !skip_local {
                        // Specific key: try exact match first
                        let mut val_tmp = [0u8; 256];
                        let vlen = self.driver.config_get(key, &mut val_tmp);

                        if vlen > 0 {
                            // Exact match — respond immediately + EOL
                            let len = format_kv(key, &val_tmp[..vlen], &mut buf);
                            let _ = self.ctx.respond_info(bus_msg.seq_id, &buf[..len]);
                            let _ = self.ctx.respond_info_eol(bus_msg.seq_id);
                        } else {
                            // No exact match — try prefix, then forward to children
                            let plen = build_prefix_matches(&self.driver, key, &mut buf);
                            // Send local prefix matches immediately
                            if plen > 0 {
                                let _ = self.ctx.respond_info(bus_msg.seq_id, &buf[..plen]);
                            }
                            if forward_to_children {
                                let origin = ConfigQueryOrigin::Devd { seq_id: bus_msg.seq_id };
                                // Relay mode: forward each child's response individually
                                if self.start_config_forward(
                                    origin, query_msg::CONFIG_GET, key, &[], &[],
                                ) {
                                    self.ctx.set_last_pending_relay();
                                    return; // EOL sent when children converge
                                }
                            }
                            let _ = self.ctx.respond_info_eol(bus_msg.seq_id);
                        }
                    } else {
                        // Routed to child — forward query to targeted child only
                        if forward_to_children {
                            let origin = ConfigQueryOrigin::Devd { seq_id: bus_msg.seq_id };
                            if !self.start_config_forward(
                                origin, query_msg::CONFIG_GET, key, &[], &[],
                            ) {
                                let _ = self.ctx.respond_info_eol(bus_msg.seq_id);
                            }
                        } else {
                            let _ = self.ctx.respond_info_eol(bus_msg.seq_id);
                        }
                    }
                }
            }

            bus_msg::CONFIG_SET => {
                let (key, value) = bus_msg.parse_config_kv();
                let mut buf = [0u8; 512];
                let skip_local = self.routing.skip_local();
                let forward_to_children = self.ctx.child_count > 0
                    && self.routing.forward_to_children();

                let keys = self.driver.config_keys();
                let valid = !skip_local && keys.iter().any(|k| k.name == key && k.writable);

                if valid {
                    let len = self.driver.config_set(key, value, &mut buf, &mut self.ctx);
                    let _ = self.ctx.respond_info(bus_msg.seq_id, &buf[..len]);
                    let _ = self.ctx.respond_info_eol(bus_msg.seq_id);
                } else if !skip_local && keys.iter().any(|k| k.name == key) {
                    let len = copy_static(&mut buf, b"ERR read-only key\n");
                    let _ = self.ctx.respond_info(bus_msg.seq_id, &buf[..len]);
                    let _ = self.ctx.respond_info_eol(bus_msg.seq_id);
                } else if forward_to_children {
                    // Unknown key or routed to child — forward asynchronously
                    let origin = ConfigQueryOrigin::Devd { seq_id: bus_msg.seq_id };
                    if self.start_config_forward(
                        origin, query_msg::CONFIG_SET, key, value, &[],
                    ) {
                        self.ctx.set_last_pending_relay();
                        return; // EOL sent when children converge
                    }
                    let _ = self.ctx.respond_info_eol(bus_msg.seq_id);
                } else {
                    // Leaf node or local-only, no match — send EOL
                    let _ = self.ctx.respond_info_eol(bus_msg.seq_id);
                }
            }

            // --- Driver-handled: everything else ---

            _ => {
                let disposition = self.driver.command(&bus_msg, &mut self.ctx);
                match disposition {
                    Disposition::Handled => {}
                    Disposition::Forward | Disposition::HandledAndForward => {
                        // Forward to children (no-op in v1 devd-backed runtime)
                    }
                }
            }
        }
    }

    /// Start an async config forward to all children.
    ///
    /// Sends the query to all children simultaneously and stores a
    /// PendingConfigQuery. When all children respond (or timeout), the
    /// accumulated result is sent to the originator.
    ///
    /// `local_prefix` is pre-filled into the response buffer (e.g. local
    /// config_get results that should precede child results).
    ///
    /// Returns true if the async forward was started, false on failure
    /// (caller should respond immediately with fallback).
    fn start_config_forward(
        &mut self,
        origin: ConfigQueryOrigin,
        msg_type: u16,
        key: &[u8],
        value: &[u8],
        local_prefix: &[u8],
    ) -> bool {
        // Allocate pending slot
        let slot = match self.ctx.alloc_pending_config_slot() {
            Some(s) => s,
            None => return false,
        };

        // Allocate seq_id for children
        let child_seq = self.ctx.alloc_config_seq();

        // Build the query message (same wire format as devd → driver)
        let mut qbuf = [0u8; 384];
        let mut header = QueryHeader::new(msg_type, child_seq);

        // If there's a pending child route, set ADDRESSED flag and include route
        let (route_data, route_len) = match &self.routing {
            RoutingMode::ForwardWithRoute { route, route_len, .. } => (&route[..*route_len], *route_len),
            _ => (&[] as &[u8], 0),
        };
        if route_len > 0 {
            header.flags |= query_flags::ADDRESSED;
        }

        let hdr_bytes = header.to_bytes();
        qbuf[..QueryHeader::SIZE].copy_from_slice(&hdr_bytes);
        let mut offset = QueryHeader::SIZE;

        // Insert route bytes (route_len + route_data) if ADDRESSED
        if route_len > 0 {
            qbuf[offset] = route_len as u8;
            offset += 1;
            qbuf[offset..offset + route_len].copy_from_slice(route_data);
            offset += route_len;
        }

        // Write key (null-terminated for CONFIG_GET, null-separated for CONFIG_SET)
        let klen = key.len().min(qbuf.len() - offset - 2);
        qbuf[offset..offset + klen].copy_from_slice(&key[..klen]);
        offset += klen;

        if msg_type == query_msg::CONFIG_SET {
            qbuf[offset] = 0;
            offset += 1;
            let vlen = value.len().min(qbuf.len() - offset - 1);
            qbuf[offset..offset + vlen].copy_from_slice(&value[..vlen]);
            offset += vlen;
        }

        // Send to children via mailbox — either one targeted child or all
        let mut sent_mask: u8 = 0;
        if let Some(target) = self.routing.target_child() {
            // Routed: send only to the targeted child
            if self.ctx.children[target].is_some() {
                self.send_to_child_superq(target, &qbuf[..offset]);
                sent_mask |= 1 << target;
            }
        } else {
            for i in 0..MAX_CHILDREN {
                if let Some(entry) = &self.ctx.children[i] {
                    // Skip children that haven't spoken the bus protocol
                    // (e.g., shell) — they won't respond with EOL.
                    if !entry.bus_active { continue; }
                    self.send_to_child_superq(i, &qbuf[..offset]);
                    sent_mask |= 1 << i;
                }
            }
        }

        if sent_mask == 0 {
            return false;
        }

        // Pre-fill response buffer with local data
        let mut response_buf = [0u8; 1024];
        let local_len = local_prefix.len().min(1024);
        response_buf[..local_len].copy_from_slice(&local_prefix[..local_len]);

        // 5s timeout — safety net only, EOL convergence is the normal path
        let deadline_ns = syscall::gettime() + 5_000_000_000;

        self.ctx.pending_config[slot] = Some(PendingConfigQuery {
            origin,
            msg_type,
            sent_mask,
            eol_mask: 0,
            child_seq_id: child_seq,
            response_buf,
            response_len: local_len as u16,
            deadline_ns,
            local_prefix_len: local_len as u16,
            relay: false,
        });

        true
    }

    /// Complete a converged or timed-out config query — send response to originator.
    fn complete_config_query(&mut self, slot: usize) {
        let mut pcq = match self.ctx.pending_config[slot].take() {
            Some(q) => q,
            None => return,
        };

        let seq_id = match pcq.origin {
            ConfigQueryOrigin::Devd { seq_id } => seq_id,
            ConfigQueryOrigin::Parent { seq_id } => seq_id,
        };

        if pcq.response_len > 0 {
            let _ = self.ctx.respond_info(seq_id, &pcq.response_buf[..pcq.response_len as usize]);
        }
        // Always send EOL after data (or as sole response if no data).
        // This signals the parent that this subtree is complete.
        let _ = self.ctx.respond_info_eol(seq_id);
    }

    /// Handle a SERVICE_INFO_RESULT from a child that matches a pending config query.
    ///
    /// Returns true if the message was consumed (caller should not forward it).
    fn handle_config_response(&mut self, child_idx: usize, seq_id: u32, info_bytes: &[u8], resp_flags: u16) -> bool {
        let slot = match self.ctx.find_pending_config(seq_id) {
            Some(s) => s,
            None => return false,
        };

        // Extract fields from pcq without holding mutable borrow across respond_info
        let (is_eol, is_relay, origin_seq) = match self.ctx.pending_config[slot].as_ref() {
            Some(pcq) => {
                let eol = (resp_flags & query_flags::EOL != 0)
                    || (pcq.msg_type == query_msg::CONFIG_SET && info_bytes == b"ERR unknown key\n");
                let oseq = match pcq.origin {
                    ConfigQueryOrigin::Devd { seq_id } => seq_id,
                    ConfigQueryOrigin::Parent { seq_id } => seq_id,
                };
                (eol, pcq.relay, oseq)
            }
            None => return false,
        };

        if !is_eol && !info_bytes.is_empty() {
            // Build prefix: "port_name/binary_name" for [source] headers
            let mut child_prefix_buf = [0u8; 96];
            let child_prefix_len = match &self.ctx.children[child_idx] {
                Some(c) => {
                    let plen = c.ctx.port_name_len as usize;
                    let blen = c.binary_name_len as usize;
                    let mut pos2 = 0;
                    child_prefix_buf[..plen].copy_from_slice(&c.ctx.port_name[..plen]);
                    pos2 += plen;
                    if blen > 0 {
                        child_prefix_buf[pos2] = b'/';
                        pos2 += 1;
                        child_prefix_buf[pos2..pos2 + blen].copy_from_slice(&c.binary_name[..blen]);
                        pos2 += blen;
                    }
                    pos2
                }
                None => 0,
            };
            let child_prefix = &child_prefix_buf[..child_prefix_len];

            if is_relay {
                // Relay mode: annotate and send immediately, don't accumulate.
                let mut tmp = [0u8; 512];
                let written = append_with_source(&mut tmp, 0, child_prefix, info_bytes);
                if written > 0 {
                    let _ = self.ctx.respond_info(origin_seq, &tmp[..written]);
                }
            } else if let Some(pcq) = self.ctx.pending_config[slot].as_mut() {
                let pos = pcq.response_len as usize;
                let written = append_with_source(
                    &mut pcq.response_buf, pos, child_prefix, info_bytes,
                );
                pcq.response_len += written as u16;
            }
        }

        // Set EOL bit for this child when EOL received
        if let Some(pcq) = self.ctx.pending_config[slot].as_mut() {
            if is_eol {
                pcq.eol_mask |= 1 << child_idx;
            }
        }

        let converged = self.ctx.pending_config[slot]
            .as_ref()
            .map_or(false, |pcq| pcq.is_converged());
        if converged {
            self.complete_config_query(slot);
        }

        true
    }

    /// Handle @topology query: every driver responds with driver=<name>.
    ///
    /// Each node sends its own response immediately (small, fits IPC limit),
    /// then forwards @topology to children. Children's responses are relayed
    /// individually with source annotation — NOT aggregated. This avoids the
    /// 576-byte IPC message limit that would truncate deep trees.
    ///
    /// EOL is sent only after all children have EOL'd.
    fn handle_topology_query(&mut self, seq_id: u32) {
        let name = &self.ctx.name[..self.ctx.name_len];
        let mut buf = [0u8; 64];
        let len = format_kv(b"driver", name, &mut buf);

        // Send own identity immediately
        let _ = self.ctx.respond_info(seq_id, &buf[..len]);

        let has_bus_children = self.ctx.children.iter()
            .any(|c| c.as_ref().map_or(false, |e| e.bus_active));

        if has_bus_children {
            // Forward @topology to children, track EOL convergence only (no aggregation).
            // Pass empty local_prefix — we already sent our own data above.
            let origin = ConfigQueryOrigin::Devd { seq_id };
            if self.start_config_forward(
                origin, query_msg::CONFIG_GET, b"@topology", &[], &[],
            ) {
                // Enable relay mode on the just-allocated pending slot.
                // Relay mode annotates and forwards each child response immediately
                // instead of aggregating (avoids 576B IPC limit on deep trees).
                self.ctx.set_last_pending_relay();
                return; // EOL sent when children converge
            }
        }

        // Leaf or no children — send EOL now
        let _ = self.ctx.respond_info_eol(seq_id);
    }

    // ========================================================================
    // Routing Layer (ADDRESSED messages through supervision tree)
    // ========================================================================

    /// Make a routing decision for an inbound message.
    ///
    /// If the message has the ADDRESSED flag, extracts the route and decides
    /// whether to handle locally, forward to a specific child, or broadcast.
    /// Returns `Local` for non-ADDRESSED messages (legacy/broadcast).
    /// Route an inbound ADDRESSED message using `:` disambiguation.
    ///
    /// Segments containing `:` are port names → match against children.
    /// Segments without `:` are driver names → match against self.
    ///
    /// Both full paths (`/pcie:0/pcied/nvme:0/nvmed`) and short paths
    /// (`/pcie:0/nvme:0`) work: port segments match children, driver name
    /// segments match self and get skipped.
    fn route_inbound<'a>(&self, raw: &'a [u8]) -> RouteAction<'a> {
        if raw.len() < QueryHeader::SIZE {
            return RouteAction::Local;
        }

        let header = match QueryHeader::from_bytes(raw) {
            Some(h) => h,
            None => return RouteAction::Local,
        };

        // Not addressed — handle as broadcast/legacy
        if header.flags & query_flags::ADDRESSED == 0 {
            return RouteAction::Local;
        }

        // Extract route: route_len(1) + route(N) after QueryHeader
        let route_start = QueryHeader::SIZE;
        if raw.len() <= route_start {
            return RouteAction::Local;
        }

        let route_len = raw[route_start] as usize;
        let route_data_start = route_start + 1;
        if route_len == 0 || raw.len() < route_data_start + route_len {
            return RouteAction::Local;
        }

        let route = &raw[route_data_start..route_data_start + route_len];

        if route[0] == b'@' {
            // Class routing: @name — match against all nodes
            return RouteAction::LocalAndForward(route);
        }

        // Path routing — walk segments using : disambiguation
        let mut path = if route[0] == b'/' { &route[1..] } else { route };

        loop {
            if path.is_empty() {
                // Path exhausted = broadcast to subtree
                return RouteAction::Local;
            }

            // Split at next '/'
            let (segment, rest) = match path.iter().position(|&b| b == b'/') {
                Some(pos) => (&path[..pos], &path[pos + 1..]),
                None => (path, &[] as &[u8]),
            };

            if segment.iter().any(|&b| b == b':') {
                // PORT segment — find child on that port
                if let Some(child_idx) = self.find_child_by_port_name(segment) {
                    return if rest.is_empty() {
                        RouteAction::ForwardTo(child_idx, &[])
                    } else {
                        RouteAction::ForwardTo(child_idx, rest)
                    };
                } else if self.matches_own_trigger_port(segment) {
                    // Matches our own trigger port — skip, continue routing
                    path = rest;
                    continue;
                } else {
                    // Unknown port
                    return RouteAction::Local;
                }
            } else {
                // DRIVER NAME segment — verify matches self, then skip
                if self.matches_own_driver_name(segment) {
                    if rest.is_empty() {
                        return RouteAction::LocalOnly;
                    } else {
                        path = rest;
                        continue;
                    }
                } else {
                    // Name mismatch
                    return RouteAction::Local;
                }
            }
        }
    }

    /// Find a child by port name segment.
    fn find_child_by_port_name(&self, segment: &[u8]) -> Option<usize> {
        for i in 0..MAX_CHILDREN {
            if let Some(entry) = &self.ctx.children[i] {
                let name = &entry.ctx.port_name[..entry.ctx.port_name_len as usize];
                if name == segment {
                    return Some(i);
                }
            }
        }
        None
    }

    /// Check if a segment matches this driver's own binary name.
    fn matches_own_driver_name(&self, segment: &[u8]) -> bool {
        let name = &self.ctx.name[..self.ctx.name_len];
        name == segment
    }

    /// Check if a segment matches this driver's own trigger port name.
    fn matches_own_trigger_port(&self, segment: &[u8]) -> bool {
        match &self.ctx.spawn_ctx {
            SpawnCtxCache::Cached(ctx) => {
                let pn = ctx.port_name();
                // Try exact match
                if pn == segment { return true; }
                // Try stripped "/" prefix (kernel bus ports are "/pcie:0" etc.)
                if pn.starts_with(b"/") && &pn[1..] == segment {
                    return true;
                }
                false
            }
            _ => false,
        }
    }

    /// Build a message with ADDRESSED flag and route prepended for forwarding.
    ///
    /// Takes the original raw message, strips the existing route, and prepends
    /// the new child route. If `child_route` is empty, removes ADDRESSED flag.
    fn build_routed_message(
        raw: &[u8],
        child_route: &[u8],
        out: &mut [u8; 576],
    ) -> usize {
        if raw.len() < QueryHeader::SIZE {
            return 0;
        }

        // Find payload start (skip past header + existing route)
        let has_addressed = {
            let flags = u16::from_le_bytes([raw[2], raw[3]]);
            flags & query_flags::ADDRESSED != 0
        };

        let payload_start = if has_addressed && raw.len() > QueryHeader::SIZE {
            let route_len = raw[QueryHeader::SIZE] as usize;
            QueryHeader::SIZE + 1 + route_len
        } else {
            QueryHeader::SIZE
        };

        let payload = &raw[payload_start..];

        if child_route.is_empty() {
            // No route for child — send without ADDRESSED flag
            out[..QueryHeader::SIZE].copy_from_slice(&raw[..QueryHeader::SIZE]);
            // Clear ADDRESSED flag
            let flags = u16::from_le_bytes([out[2], out[3]]) & !query_flags::ADDRESSED;
            out[2..4].copy_from_slice(&flags.to_le_bytes());
            let plen = payload.len().min(out.len() - QueryHeader::SIZE);
            out[QueryHeader::SIZE..QueryHeader::SIZE + plen].copy_from_slice(&payload[..plen]);
            QueryHeader::SIZE + plen
        } else {
            // Build: header(ADDRESSED) + route_len + route + payload
            out[..QueryHeader::SIZE].copy_from_slice(&raw[..QueryHeader::SIZE]);
            // Ensure ADDRESSED flag is set
            let flags = u16::from_le_bytes([out[2], out[3]]) | query_flags::ADDRESSED;
            out[2..4].copy_from_slice(&flags.to_le_bytes());

            let mut pos = QueryHeader::SIZE;
            let route_len = child_route.len().min(255);
            out[pos] = route_len as u8;
            pos += 1;
            out[pos..pos + route_len].copy_from_slice(&child_route[..route_len]);
            pos += route_len;

            let plen = payload.len().min(out.len() - pos);
            out[pos..pos + plen].copy_from_slice(&payload[..plen]);
            pos + plen
        }
    }

    /// Send a command to a child via its SuperQ (parent→child direction).
    ///
    /// Writes a FORWARD note containing the raw payload into the child's
    /// supervision queue down ring. The kernel wakes the child's subscriber.
    fn send_to_child_superq(&mut self, child_idx: usize, payload: &[u8]) {
        if let Some(entry) = &self.ctx.children[child_idx] {
            if let Some(ref superq) = entry.superq {
                let _ = superq.send_forward(payload, 0);
            }
        }
    }

    /// Forward a routed message to a specific child via SuperQ.
    fn forward_to_child_routed(&mut self, child_idx: usize, raw: &[u8], child_route: &[u8]) {
        let mut buf = [0u8; 576];
        let len = Self::build_routed_message(raw, child_route, &mut buf);
        if len > 0 {
            self.send_to_child_superq(child_idx, &buf[..len]);
        }
    }

    /// Forward a routed message to all children via SuperQ.
    fn forward_to_all_children(&mut self, raw: &[u8], child_route: &[u8]) {
        let mut buf = [0u8; 576];
        let len = Self::build_routed_message(raw, child_route, &mut buf);
        if len > 0 {
            for i in 0..MAX_CHILDREN {
                if self.ctx.children[i].is_some() {
                    self.send_to_child_superq(i, &buf[..len]);
                }
            }
        }
    }

    /// Strip ADDRESSED route from raw bytes for local handling.
    ///
    /// Returns (stripped_buf, stripped_len). The stripped message has the
    /// ADDRESSED flag cleared and route bytes removed, so parse_command_buf
    /// sees a standard message.
    fn strip_route(raw: &[u8]) -> ([u8; 576], usize) {
        let mut out = [0u8; 576];
        if raw.len() < QueryHeader::SIZE {
            let len = raw.len().min(576);
            out[..len].copy_from_slice(&raw[..len]);
            return (out, len);
        }

        let flags = u16::from_le_bytes([raw[2], raw[3]]);
        if flags & query_flags::ADDRESSED == 0 {
            let len = raw.len().min(576);
            out[..len].copy_from_slice(&raw[..len]);
            return (out, len);
        }

        // Skip route
        let route_len = if raw.len() > QueryHeader::SIZE {
            raw[QueryHeader::SIZE] as usize
        } else {
            0
        };
        let payload_start = QueryHeader::SIZE + 1 + route_len;

        // Copy header with ADDRESSED cleared
        out[..QueryHeader::SIZE].copy_from_slice(&raw[..QueryHeader::SIZE]);
        let new_flags = flags & !query_flags::ADDRESSED;
        out[2..4].copy_from_slice(&new_flags.to_le_bytes());

        // Copy payload
        if payload_start < raw.len() {
            let payload = &raw[payload_start..];
            let plen = payload.len().min(576 - QueryHeader::SIZE);
            out[QueryHeader::SIZE..QueryHeader::SIZE + plen].copy_from_slice(&payload[..plen]);
            (out, QueryHeader::SIZE + plen)
        } else {
            (out, QueryHeader::SIZE)
        }
    }

    /// Store a ChildEntry for a newly spawned child.
    ///
    /// No Mux registration needed — children communicate via signals,
    /// not polled channels.
    fn store_child_entry(
        &mut self,
        mailbox: Mailbox,
        pid: u32,
        filter: &crate::devd::SpawnFilter,
        context: Option<&SpawnChildContext>,
        binary_name: &[u8],
    ) -> bool {
        // Find free slot
        let slot = match self.ctx.children.iter().position(|c| c.is_none()) {
            Some(s) => s,
            None => return false,
        };

        // Build ChildSpawnCtx from filter pattern (port name) and context
        let mut spawn_ctx = ChildSpawnCtx {
            port_type: 0,
            port_id: 0xFF,
            port_name: [0u8; 64],
            port_name_len: 0,
            metadata: [0u8; 64],
            metadata_len: 0,
            shmem_id: 0,
            context_kvs: [([0u8; 32], 0u8, [0u8; 64], 0u8); 4],
            context_kv_count: 0,
        };

        // Port name from filter pattern (exact match)
        let pname = filter.pattern_bytes();
        let plen = pname.len().min(64);
        spawn_ctx.port_name[..plen].copy_from_slice(&pname[..plen]);
        spawn_ctx.port_name_len = plen as u8;

        // Fill from context if available
        if let Some(ctx) = context {
            spawn_ctx.port_type = ctx.port_type;
            spawn_ctx.port_id = ctx.port_id;
            spawn_ctx.shmem_id = ctx.shmem_id;
            let mlen = ctx.metadata_len as usize;
            spawn_ctx.metadata[..mlen].copy_from_slice(&ctx.metadata[..mlen]);
            spawn_ctx.metadata_len = ctx.metadata_len;
            let kv_n = (ctx.kv_count as usize).min(4);
            for i in 0..kv_n {
                let klen = ctx.kv_keys_len[i] as usize;
                let vlen = ctx.kv_values_len[i] as usize;
                spawn_ctx.context_kvs[i].0[..klen].copy_from_slice(&ctx.kv_keys[i][..klen]);
                spawn_ctx.context_kvs[i].1 = klen as u8;
                spawn_ctx.context_kvs[i].2[..vlen].copy_from_slice(&ctx.kv_values[i][..vlen]);
                spawn_ctx.context_kvs[i].3 = vlen as u8;
            }
            spawn_ctx.context_kv_count = kv_n as u8;
        }

        // Store binary name for path annotation
        let mut bname = [0u8; 16];
        let blen = binary_name.len().min(16);
        bname[..blen].copy_from_slice(&binary_name[..blen]);

        self.ctx.children[slot] = Some(ChildEntry {
            mailbox,
            superq: None,
            pid,
            ctx: spawn_ctx,
            binary_name: bname,
            binary_name_len: blen as u8,
            bus_active: false,
        });
        self.ctx.child_count += 1;

        true
    }

    /// Store a child entry with a SuperQ handle for reliable child→parent messaging.
    /// Registers the SuperQ handle in the Mux with TAG_CHILD_SUPERQ_BASE + slot.
    fn store_child_entry_with_superq(
        &mut self,
        mailbox: Mailbox,
        superq: crate::supervision::SupervisionHandle,
        pid: u32,
        filter: &crate::devd::SpawnFilter,
        context: Option<&SpawnChildContext>,
        binary_name: &[u8],
    ) -> bool {
        // Reuse existing logic — store_child_entry sets superq=None
        if !self.store_child_entry(mailbox, pid, filter, context, binary_name) {
            // store failed — need to drop superq cleanly
            core::mem::forget(superq); // don't close, slot not stored
            return false;
        }
        // Find the slot we just stored (last child with this pid)
        let slot = match self.find_child_by_pid(pid) {
            Some(s) => s,
            None => return true, // shouldn't happen
        };
        // Register SuperQ handle in Mux
        let sq_handle = superq.handle();
        let tag = TAG_CHILD_SUPERQ_BASE + slot as u32;
        if self.ctx.mux.add(sq_handle, MuxFilter::Readable).is_ok() {
            self.ctx.handles.add(sq_handle, tag);
        }
        // Store superq in entry
        if let Some(entry) = &mut self.ctx.children[slot] {
            entry.superq = Some(superq);
        }
        true
    }

    /// Handle CHILD_EXIT signal — child process died.
    ///
    /// Cleans up the ChildEntry and sets EOL on pending config queries.
    fn handle_child_exit(&mut self, child_pid: u32) {
        let child_idx = match self.find_child_by_pid(child_pid) {
            Some(i) => i,
            None => return, // Not one of our children
        };

        crate::uinfo!("bus", "child_exited"; pid = child_pid as u64);

        self.ctx.drop_child_entry(child_idx);

        // Implicit EOL: set eol_mask bit for this child on all pending queries
        let child_bit = 1u8 << child_idx;
        for slot in 0..MAX_PENDING_CONFIG {
            let converged = if let Some(pcq) = self.ctx.pending_config[slot].as_mut() {
                if pcq.sent_mask & child_bit != 0 {
                    pcq.eol_mask |= child_bit;
                }
                pcq.is_converged()
            } else {
                false
            };
            if converged {
                self.complete_config_query(slot);
            }
        }
    }

    /// Handle child SuperQ readable event — child sent a FORWARD note.
    /// Handle devd channel readable event.
    ///
    /// Reads a raw message, checks for child relay, applies routing, and dispatches.
    fn handle_devd_event(&mut self, handle: Handle) {
        let mut raw_buf = [0u8; 512];
        match self.ctx.devd.raw_recv(&mut raw_buf) {
            Ok(Some(n)) if n >= QueryHeader::SIZE => {
                if !self.try_relay_to_child(&raw_buf, n) {
                    let action = self.route_inbound(&raw_buf[..n]);
                    match action {
                        RouteAction::Local => {
                            self.dispatch_raw_command(&raw_buf[..n]);
                        }
                        RouteAction::LocalOnly => {
                            self.routing = RoutingMode::LocalOnly;
                            self.dispatch_raw_command(&raw_buf[..n]);
                            self.routing = RoutingMode::Broadcast;
                        }
                        RouteAction::ForwardTo(child_idx, child_route) => {
                            if child_route.is_empty() {
                                self.routing = RoutingMode::ForwardTo { child: child_idx };
                            } else {
                                let mut route = [0u8; 128];
                                let rlen = child_route.len().min(128);
                                route[..rlen].copy_from_slice(&child_route[..rlen]);
                                self.routing = RoutingMode::ForwardWithRoute {
                                    child: child_idx, route, route_len: rlen,
                                };
                            }
                            self.dispatch_raw_command(&raw_buf[..n]);
                            self.routing = RoutingMode::Broadcast;
                        }
                        RouteAction::LocalAndForward(child_route) => {
                            self.dispatch_raw_command(&raw_buf[..n]);
                            self.forward_to_all_children(&raw_buf[..n], child_route);
                        }
                        RouteAction::ForwardAll(child_route) => {
                            self.forward_to_all_children(&raw_buf[..n], child_route);
                        }
                    }
                }
            }
            Ok(_) => {}
            Err(e) => {
                // Devd channel broken — remove from Mux to prevent hot-loop.
                crate::uwarn!("bus", "devd_recv_failed"; err = e.as_str());
                let _ = self.ctx.mux.remove(handle);
                self.ctx.handles.remove(handle);
            }
        }
    }

    /// Handle parent command received via SuperQ (tree mode).
    ///
    /// Reads FORWARD notes from our SupervisionChild handle and dispatches
    /// as raw commands (SpawnChild, ConfigGet, etc).
    fn handle_parent_superq_command(&mut self) {
        // Drain pending EXIT notes from all children BEFORE processing parent
        // commands. This prevents try_forward_spawn_to_child from forwarding
        // a SpawnChild to a dead child whose EXIT note hasn't been processed yet.
        self.drain_child_superq_exits();

        let mut raw_buf = [0u8; 512];
        // Drain all available commands from parent
        loop {
            let n = match self.ctx.devd.recv_superq(&mut raw_buf) {
                Ok(Some(n)) => n,
                _ => break,
            };
            if n >= QueryHeader::SIZE {
                if !self.try_relay_to_child(&raw_buf, n) {
                    self.dispatch_raw_command(&raw_buf[..n]);
                }
            }
        }
    }

    /// Drain pending EXIT notes from all child SuperQs.
    ///
    /// Only peeks for EXIT notes — if a FORWARD note is found, it's left
    /// for the normal handle_child_superq_event path by re-processing it here.
    fn drain_child_superq_exits(&mut self) {
        for child_idx in 0..MAX_CHILDREN {
            // Only check children that exist and have a SuperQ
            let has_superq = self.ctx.children[child_idx]
                .as_ref()
                .map(|e| e.superq.is_some())
                .unwrap_or(false);
            if !has_superq { continue; }

            // Delegate to the full handler which now detects EXIT notes
            self.handle_child_superq_event(child_idx);
        }
    }

    ///
    /// Reads all pending notes from the child's SuperQ and dispatches
    /// FORWARD notes based on the query protocol msg_type in the payload.
    /// EXIT notes trigger child cleanup via handle_child_exit().
    fn handle_child_superq_event(&mut self, child_idx: usize) {
        // Drain all available notes from this child's SuperQ.
        // Use try_recv() instead of recv_forward() so we see EXIT notes
        // (recv_forward silently drops non-FORWARD notes).
        loop {
            let superq = match &self.ctx.children[child_idx] {
                Some(entry) => match &entry.superq {
                    Some(sq) => sq,
                    None => return,
                },
                None => return,
            };

            let note = match superq.try_recv() {
                Ok(Some(n)) => n,
                Ok(None) => break,
                Err(_) => {
                    // PeerClosed — child exited, SuperQ is HalfClosed with empty ring.
                    // Treat like EXIT note: clean up child entry.
                    let pid = self.ctx.children[child_idx]
                        .as_ref().map(|e| e.pid).unwrap_or(0);
                    self.handle_child_exit(pid);
                    return;
                }
            };

            // EXIT note: child died — clean up entry and stop draining
            if note.note_type == abi::supervision_note::EXIT {
                let pid = self.ctx.children[child_idx]
                    .as_ref().map(|e| e.pid).unwrap_or(0);
                self.handle_child_exit(pid);
                return;
            }

            // Only process FORWARD notes (skip unknown types)
            if note.note_type != abi::supervision_note::FORWARD {
                continue;
            }

            // Reassemble FORWARD payload
            let mut payload_buf = [0u8; 576];
            let chunk_len = (note.len as usize).min(120).min(payload_buf.len());
            payload_buf[..chunk_len].copy_from_slice(&note.payload[..chunk_len]);
            let mut pos = chunk_len;

            // Read continuation fragments (HAS_MORE flag)
            if note.flags & 1 != 0 {
                loop {
                    let cont = match superq.recv() {
                        Ok(n) => n,
                        Err(e) => {
                            crate::uwarn!("bus", "superq_frag_recv_err"; err = e.as_str());
                            break;
                        }
                    };
                    let clen = (cont.len as usize).min(120);
                    let n = clen.min(payload_buf.len().saturating_sub(pos));
                    if n > 0 {
                        payload_buf[pos..pos + n].copy_from_slice(&cont.payload[..n]);
                        pos += n;
                    }
                    if cont.flags & 1 == 0 { break; }
                }
            }
            let len = pos;

            if len < QueryHeader::SIZE {
                continue;
            }

            // Mark child as active
            if let Some(entry) = &mut self.ctx.children[child_idx] {
                entry.bus_active = true;
            }

            // Dispatch based on query protocol msg_type
            let header = match QueryHeader::from_bytes(&payload_buf[..len]) {
                Some(h) => h,
                None => continue,
            };

            match header.msg_type {
                // Fire-and-forget messages: annotate path and send to devd
                query_msg::STATE_CHANGE | query_msg::SPAWN_ACK => {
                    let (annotated, alen) = self.annotate_path_upward(child_idx, &payload_buf[..len]);
                    let _ = self.ctx.devd.raw_send(&annotated[..alen]);
                }
                // Config/info result from child
                query_msg::SERVICE_INFO_RESULT => {
                    // Check EOL flag — if set, this is a CONFIG_EOL
                    if header.flags & query_flags::EOL != 0 {
                        // CONFIG_EOL: mark this child as done in pending config queries
                        let child_bit = 1u8 << child_idx;
                        let mut converged_slot = None;
                        for slot in 0..MAX_PENDING_CONFIG {
                            if let Some(pcq) = self.ctx.pending_config[slot].as_mut() {
                                if pcq.sent_mask & child_bit != 0 {
                                    pcq.eol_mask |= child_bit;
                                    if pcq.is_converged() {
                                        converged_slot = Some(slot);
                                    }
                                }
                            }
                        }
                        if let Some(slot) = converged_slot {
                            self.complete_config_query(slot);
                        }
                    } else {
                        // CONFIG_RESULT: try to handle as config response
                        if let Some((result, info_bytes)) = ServiceInfoResult::from_bytes(&payload_buf[..len]) {
                            if !self.handle_config_response(child_idx, header.seq_id, info_bytes, result.header.flags) {
                                self.forward_to_devd(child_idx, header.seq_id, &payload_buf[..len]);
                            }
                        }
                    }
                }
                // Port registration: forward to devd with seq rewriting
                query_msg::REGISTER_PORT_INFO => {
                    self.forward_to_devd(child_idx, header.seq_id, &payload_buf[..len]);
                }
                // Everything else: forward to devd
                _ => {
                    self.forward_to_devd(child_idx, header.seq_id, &payload_buf[..len]);
                }
            }
        }
    }


    /// Find a child entry index by PID.
    fn find_child_by_pid(&self, pid: u32) -> Option<usize> {
        for i in 0..MAX_CHILDREN {
            if let Some(entry) = &self.ctx.children[i] {
                if entry.pid == pid {
                    return Some(i);
                }
            }
        }
        None
    }

    /// Annotate a child message with path information for upward routing.
    ///
    /// Prepends the child's trigger port name as a path segment. If the
    /// message already has an ADDRESSED route (from a deeper relay node),
    /// the segment is prepended to the existing route: `segment/existing`.
    /// Otherwise creates a new route from just the segment.
    ///
    /// Returns (annotated_buf, length). If the child has no port name or
    /// the message is invalid, returns a copy of the original message.
    fn annotate_path_upward(&self, child_idx: usize, msg: &[u8]) -> ([u8; 576], usize) {
        let mut out = [0u8; 576];

        // Build segment: "port_name/binary_name" (e.g., "nvme:0/nvmed")
        let mut segment_buf = [0u8; 96];
        let seg_len = match &self.ctx.children[child_idx] {
            Some(entry) => {
                let plen = entry.ctx.port_name_len as usize;
                if plen == 0 {
                    let len = msg.len().min(576);
                    out[..len].copy_from_slice(&msg[..len]);
                    return (out, len);
                }
                let mut pos = 0;
                segment_buf[pos..pos + plen].copy_from_slice(&entry.ctx.port_name[..plen]);
                pos += plen;
                let blen = entry.binary_name_len as usize;
                if blen > 0 {
                    segment_buf[pos] = b'/';
                    pos += 1;
                    segment_buf[pos..pos + blen].copy_from_slice(&entry.binary_name[..blen]);
                    pos += blen;
                }
                pos
            }
            None => {
                let len = msg.len().min(576);
                out[..len].copy_from_slice(&msg[..len]);
                return (out, len);
            }
        };
        let segment = &segment_buf[..seg_len];

        if msg.len() < QueryHeader::SIZE {
            let len = msg.len().min(576);
            out[..len].copy_from_slice(&msg[..len]);
            return (out, len);
        }

        let flags = u16::from_le_bytes([msg[2], msg[3]]);

        if flags & query_flags::ADDRESSED != 0 {
            // Message already has a route — prepend our segment
            let existing_route_len = if msg.len() > QueryHeader::SIZE {
                msg[QueryHeader::SIZE] as usize
            } else {
                0
            };
            let payload_start = QueryHeader::SIZE + 1 + existing_route_len;
            let existing_route = &msg[QueryHeader::SIZE + 1..payload_start.min(msg.len())];

            // New route: segment + "/" + existing_route
            let new_route_len = seg_len + 1 + existing_route.len();
            if new_route_len > 255 {
                // Route too long — return message unchanged
                let len = msg.len().min(576);
                out[..len].copy_from_slice(&msg[..len]);
                return (out, len);
            }

            // Copy header with ADDRESSED flag (already set)
            out[..QueryHeader::SIZE].copy_from_slice(&msg[..QueryHeader::SIZE]);
            // Route length
            out[QueryHeader::SIZE] = new_route_len as u8;
            // New route: segment/existing
            let route_start = QueryHeader::SIZE + 1;
            out[route_start..route_start + seg_len].copy_from_slice(segment);
            out[route_start + seg_len] = b'/';
            out[route_start + seg_len + 1..route_start + new_route_len]
                .copy_from_slice(existing_route);
            // Copy payload
            let out_payload_start = route_start + new_route_len;
            if payload_start < msg.len() {
                let payload = &msg[payload_start..];
                let plen = payload.len().min(576 - out_payload_start);
                out[out_payload_start..out_payload_start + plen].copy_from_slice(&payload[..plen]);
                (out, out_payload_start + plen)
            } else {
                (out, out_payload_start)
            }
        } else {
            // No existing route — create one from just the segment
            let new_route_len = seg_len;
            let new_flags = flags | query_flags::ADDRESSED;

            // Copy header with ADDRESSED flag set
            out[..QueryHeader::SIZE].copy_from_slice(&msg[..QueryHeader::SIZE]);
            out[2..4].copy_from_slice(&new_flags.to_le_bytes());
            // Route length
            out[QueryHeader::SIZE] = new_route_len as u8;
            // Route: just the segment
            let route_start = QueryHeader::SIZE + 1;
            out[route_start..route_start + seg_len].copy_from_slice(segment);
            // Copy payload (everything after original header)
            let out_payload_start = route_start + seg_len;
            if QueryHeader::SIZE < msg.len() {
                let payload = &msg[QueryHeader::SIZE..];
                let plen = payload.len().min(576 - out_payload_start);
                out[out_payload_start..out_payload_start + plen].copy_from_slice(&payload[..plen]);
                (out, out_payload_start + plen)
            } else {
                (out, out_payload_start)
            }
        }
    }

    /// Forward a child message to devd with seq_id rewriting for response relay.
    ///
    /// Annotates the message with path information before forwarding.
    fn forward_to_devd(&mut self, child_idx: usize, child_seq_id: u32, msg: &[u8]) {
        let (annotated, alen) = self.annotate_path_upward(child_idx, msg);

        // Allocate forwarded seq_id
        let devd_seq = self.ctx.forwarded_next_seq;
        self.ctx.forwarded_next_seq = self.ctx.forwarded_next_seq.wrapping_add(1);
        if self.ctx.forwarded_next_seq < 0x4000_0001 {
            self.ctx.forwarded_next_seq = 0x4000_0001;
        }

        // Store forwarding record
        let stored = self.ctx.forwarded.iter_mut()
            .find(|f| f.is_none())
            .map(|slot| {
                *slot = Some(ForwardedRequest {
                    devd_seq_id: devd_seq,
                    child_idx: child_idx as u8,
                    child_seq_id,
                });
                true
            })
            .unwrap_or(false);

        if !stored {
            crate::uerror!("bus", "fwd_table_full");
            return;
        }

        // Rewrite seq_id in annotated message and send to devd
        let mut fwd = [0u8; 576];
        fwd[..alen].copy_from_slice(&annotated[..alen]);
        // seq_id is at offset 4..8 in QueryHeader
        fwd[4..8].copy_from_slice(&devd_seq.to_le_bytes());
        let _ = self.ctx.devd.raw_send(&fwd[..alen]);
    }

    /// Check if a devd response matches a forwarded request and relay it to the child.
    ///
    /// Returns true if the message was relayed (caller should NOT dispatch it locally).
    fn try_relay_to_child(&mut self, buf: &[u8], len: usize) -> bool {
        if len < QueryHeader::SIZE { return false; }
        let header = match QueryHeader::from_bytes(&buf[..len]) {
            Some(h) => h,
            None => return false,
        };

        // Check if this seq_id matches any forwarded request
        let seq = header.seq_id;
        let mut found: Option<(u8, u32)> = None;
        for slot in &mut self.ctx.forwarded {
            if let Some(fwd) = slot {
                if fwd.devd_seq_id == seq {
                    found = Some((fwd.child_idx, fwd.child_seq_id));
                    *slot = None;
                    break;
                }
            }
        }

        let (child_idx, child_seq) = match found {
            Some(f) => f,
            None => return false,
        };

        // Rewrite seq_id back to child's original and send via mailbox
        let mut relay = [0u8; 512];
        let rlen = len.min(512);
        relay[..rlen].copy_from_slice(&buf[..rlen]);
        relay[4..8].copy_from_slice(&child_seq.to_le_bytes());

        // Forward devd response to child via SuperQ
        self.send_to_child_superq(child_idx as usize, &relay[..rlen]);

        true
    }
}

// ============================================================================
// Entry Point
// ============================================================================

/// Entry point for all drivers using the bus framework. Never returns.
///
/// If spawned with exec_with_mailbox (tree mode), reads spawn context from
/// the mailbox and routes all devd protocol through the parent via mailbox
/// writes + signals. The parent annotates paths and relays to devd.
///
/// If no mailbox (root mode), connects to devd directly.
///
/// # Arguments
/// * `name` - Driver name (for logging and identification)
/// * `driver` - The Driver implementation
pub fn driver_main<D: Driver>(name: &[u8], driver: D) -> ! {
    // Check for mailbox (Handle::MAILBOX = slot 5).
    // If present, this child was spawned via exec_with_mailbox and its
    // spawn context is in the mailbox header — no GET_SPAWN_CONTEXT needed.
    let mailbox = Mailbox::from_handle(abi::Handle::MAILBOX).ok();
    let has_mailbox = mailbox.as_ref().map_or(false, |mb| mb.is_valid());

    // Diagnostic: log mailbox state
    let mb_mapped = mailbox.is_some();
    let driver_name_str = core::str::from_utf8(name).unwrap_or("?");
    crate::unotice!("bus", "driver_init"; driver = driver_name_str, mb_mapped = mb_mapped as u32, mb_valid = has_mailbox as u32);
    if has_mailbox {
        let hdr = mailbox.as_ref().unwrap().header();
        crate::unotice!("bus", "mailbox_hdr"; kv_count = hdr.kv_count as u32, dev_count = hdr.device_count as u32, bus_type = hdr.bus_type as u32);
    } else if mb_mapped {
        // Mailbox shmem exists but header is invalid — dump raw magic for debugging
        let hdr = mailbox.as_ref().unwrap().header();
        crate::unotice!("bus", "mailbox_invalid"; magic = crate::ulog::hex32(hdr.magic), version = hdr.version as u32);
    }

    // Pre-populate spawn context from mailbox if available
    let pre_spawn_ctx = if has_mailbox {
        parse_spawn_context_from_mailbox(mailbox.as_ref().unwrap())
    } else {
        None
    };

    // Two modes:
    // 1. Tree mode: spawned by exec_with_mailbox, SuperQ at slot 4 for parent↔child
    // 2. Root mode: no mailbox, connect to devd-query: port (e.g. devd itself)
    let devd = if has_mailbox {
        let superq = crate::supervision::SupervisionHandle::from_handle(abi::Handle::SUPERVISION);
        DevdClient::from_supervision(superq)
    } else {
        match DevdClient::connect() {
            Ok(c) => c,
            Err(e) => {
                crate::uerror!("bus", "devd_connect_failed"; err = e.as_str());
                syscall::exit(1);
            }
        }
    };

    // Create the event loop Mux
    let mux = match Mux::new() {
        Ok(m) => m,
        Err(e) => {
            crate::uerror!("bus", "mux_create_failed"; err = e.as_str());
            syscall::exit(1);
        }
    };

    let mut runtime = DriverRuntime::new(driver, devd, mux, name);

    // If we have a pre-populated spawn context from mailbox, cache it
    // so the child never needs GET_SPAWN_CONTEXT.
    if let Some(ctx) = pre_spawn_ctx {
        crate::unotice!("bus", "spawn_ctx_from_mb"; meta_len = ctx.metadata().len() as u32);
        runtime.ctx.spawn_ctx = SpawnCtxCache::Cached(ctx);
    } else if has_mailbox {
        crate::unotice!("bus", "mb_parse_failed";);
    }

    // Flush diagnostic logs before entering reset — guarantees visibility
    // even if the driver crashes during init
    crate::ulog::flush();

    runtime.run()
}

/// Parse SpawnContext from a mailbox header + KVs.
///
/// Called during driver_main startup to pre-populate the spawn context cache
/// so the child never needs to query GET_SPAWN_CONTEXT via IPC.
fn parse_spawn_context_from_mailbox(mb: &Mailbox) -> Option<SpawnContext> {
    use crate::bus::SpawnContext;

    let hdr = mb.header();
    if !hdr.is_valid() {
        return None;
    }

    // Read KVs from mailbox
    let mut keys = [[0u8; 32]; 4];
    let mut key_lens = [0u8; 4];
    let mut values = [[0u8; 64]; 4];
    let mut value_lens = [0u8; 4];
    let kv_count = mb.read_kvs(&mut keys, &mut key_lens, &mut values, &mut value_lens);

    // Parse well-known KVs and collect pass-through KVs
    let mut port_name = [0u8; 64];
    let mut port_name_len = 0usize;
    let mut metadata = [0u8; 64];
    let mut metadata_len = 0usize;
    let mut shmem_id: u32 = 0;
    // Pass-through context KVs packed for set_context_kvs
    let mut kv_pack: [([u8; 32], u8, [u8; 64], u8); 4] = [([0; 32], 0, [0; 64], 0); 4];
    let mut ctx_kv_count = 0usize;

    for i in 0..kv_count {
        let klen = key_lens[i] as usize;
        let vlen = value_lens[i] as usize;
        let key = &keys[i][..klen];
        let val = &values[i][..vlen];

        if key == b"port.name" {
            let n = val.len().min(64);
            port_name[..n].copy_from_slice(&val[..n]);
            port_name_len = n;
        } else if key == b"port.metadata" {
            let n = val.len().min(64);
            metadata[..n].copy_from_slice(&val[..n]);
            metadata_len = n;
        } else if key == b"port.shmem_id" && vlen >= 4 {
            shmem_id = u32::from_le_bytes([val[0], val[1], val[2], val[3]]);
        } else if ctx_kv_count < 4 {
            // Pass-through context KV (e.g., "mount.path")
            kv_pack[ctx_kv_count].0[..klen].copy_from_slice(key);
            kv_pack[ctx_kv_count].1 = klen as u8;
            kv_pack[ctx_kv_count].2[..vlen].copy_from_slice(val);
            kv_pack[ctx_kv_count].3 = vlen as u8;
            ctx_kv_count += 1;
        }
    }

    let port_class = abi::PortClass::from_u16(hdr.bus_type as u16)
        .unwrap_or(abi::PortClass::Unknown);

    let mut ctx = SpawnContext::new(
        &port_name,
        port_name_len,
        port_class,
        &metadata[..metadata_len],
        hdr.bus_index,
    );
    ctx.shmem_id = shmem_id;

    if ctx_kv_count > 0 {
        ctx.set_context_kvs(&kv_pack, ctx_kv_count);
    }

    Some(ctx)
}

// ============================================================================
// Source-Annotated Config Response Appender
// ============================================================================

/// Append config response data to buf at `pos`, annotating with source path.
///
/// Builds full paths through the supervision tree:
/// - Existing `[path]` headers in `data` are rewritten to `[prefix/path]`
/// - Raw key=value lines before any header get a new `[prefix]` header prepended
/// - If `prefix` is empty, data is copied as-is
///
/// Returns number of bytes written.
fn append_with_source(buf: &mut [u8], pos: usize, prefix: &[u8], data: &[u8]) -> usize {
    let cap = buf.len();
    if pos >= cap || data.is_empty() {
        return 0;
    }
    if prefix.is_empty() {
        let n = data.len().min(cap - pos);
        buf[pos..pos + n].copy_from_slice(&data[..n]);
        return n;
    }

    let mut out = pos;
    let mut prepended = false; // wrote [prefix]\n for raw lines?
    let mut i = 0;

    while i < data.len() && out < cap {
        // Find end of current line
        let line_end = data[i..].iter().position(|&b| b == b'\n')
            .map(|p| i + p + 1)
            .unwrap_or(data.len());
        let line = &data[i..line_end];
        let trimmed_end = if line.ends_with(b"\n") { line.len() - 1 } else { line.len() };
        let trimmed = &line[..trimmed_end];

        if trimmed.starts_with(b"[") && trimmed.ends_with(b"]") && trimmed.len() > 2 {
            // [path] header — rewrite to [prefix/path]
            let old_path = &trimmed[1..trimmed.len() - 1];
            // Need: [ + prefix + / + old_path + ] + \n
            let needed = 1 + prefix.len() + 1 + old_path.len() + 2;
            if out + needed <= cap {
                buf[out] = b'['; out += 1;
                buf[out..out + prefix.len()].copy_from_slice(prefix); out += prefix.len();
                buf[out] = b'/'; out += 1;
                buf[out..out + old_path.len()].copy_from_slice(old_path); out += old_path.len();
                buf[out] = b']'; out += 1;
                buf[out] = b'\n'; out += 1;
            }
            // Header covers following raw lines — don't add another [prefix]
            prepended = true;
        } else if !trimmed.is_empty() {
            // Raw key=value line — prepend [prefix]\n before the first one
            if !prepended {
                let hdr = 1 + prefix.len() + 2; // [prefix]\n
                if out + hdr <= cap {
                    buf[out] = b'['; out += 1;
                    buf[out..out + prefix.len()].copy_from_slice(prefix); out += prefix.len();
                    buf[out] = b']'; out += 1;
                    buf[out] = b'\n'; out += 1;
                }
                prepended = true;
            }
            let n = line.len().min(cap - out);
            if n > 0 {
                buf[out..out + n].copy_from_slice(&line[..n]);
                out += n;
            }
        }

        i = line_end;
    }

    out - pos
}

// ============================================================================
// Config Summary Builder
// ============================================================================

/// Auto-build a summary response from the driver's registered config keys.
///
/// For each key, calls `config_get()` and emits `key=value\n`.
/// Appends a `keys=` line listing all writable keys.
fn build_config_summary<D: Driver>(driver: &D, buf: &mut [u8]) -> usize {
    let keys = driver.config_keys();
    let mut pos = 0;

    // Emit key=value lines
    for key in keys {
        // key name
        let klen = key.name.len().min(buf.len() - pos);
        buf[pos..pos + klen].copy_from_slice(&key.name[..klen]);
        pos += klen;

        // '='
        if pos < buf.len() { buf[pos] = b'='; pos += 1; }

        // value from driver
        let n = driver.config_get(key.name, &mut buf[pos..]);
        pos += n;

        // newline
        if pos < buf.len() { buf[pos] = b'\n'; pos += 1; }
    }

    // Append keys= line (writable keys only)
    let writable: [&[u8]; 16] = {
        let mut arr: [&[u8]; 16] = [b""; 16];
        let mut count = 0;
        for key in keys {
            if key.writable && count < 16 {
                arr[count] = key.name;
                count += 1;
            }
        }
        arr
    };

    let writable_count = keys.iter().filter(|k| k.writable).count().min(16);
    if writable_count > 0 {
        pos += copy_static(&mut buf[pos..], b"keys=");
        for i in 0..writable_count {
            if i > 0 {
                if pos < buf.len() { buf[pos] = b','; pos += 1; }
            }
            let name = writable[i];
            let nlen = name.len().min(buf.len() - pos);
            buf[pos..pos + nlen].copy_from_slice(&name[..nlen]);
            pos += nlen;
        }
        if pos < buf.len() { buf[pos] = b'\n'; pos += 1; }
    }

    pos
}

fn copy_static(dst: &mut [u8], src: &[u8]) -> usize {
    let len = src.len().min(dst.len());
    dst[..len].copy_from_slice(&src[..len]);
    len
}

/// Format key=value\n into buf (uses a temp buffer to avoid overlap with value).
/// Returns bytes written (including trailing newline).
fn format_kv(key: &[u8], value: &[u8], buf: &mut [u8]) -> usize {
    let mut tmp = [0u8; 512];
    let mut pos = 0;
    let klen = key.len().min(tmp.len());
    tmp[..klen].copy_from_slice(&key[..klen]);
    pos += klen;
    if pos < tmp.len() { tmp[pos] = b'='; pos += 1; }
    let vlen = value.len().min(tmp.len() - pos);
    tmp[pos..pos + vlen].copy_from_slice(&value[..vlen]);
    pos += vlen;
    if pos < tmp.len() { tmp[pos] = b'\n'; pos += 1; }
    let out_len = pos.min(buf.len());
    buf[..out_len].copy_from_slice(&tmp[..out_len]);
    out_len
}

/// Return all keys whose name starts with `prefix`, formatted as key=value\n lines.
fn build_prefix_matches<D: Driver>(driver: &D, prefix: &[u8], buf: &mut [u8]) -> usize {
    let keys = driver.config_keys();
    let mut pos = 0;

    for key in keys {
        if key.name.len() >= prefix.len() && &key.name[..prefix.len()] == prefix {
            // key name
            let klen = key.name.len().min(buf.len() - pos);
            buf[pos..pos + klen].copy_from_slice(&key.name[..klen]);
            pos += klen;
            // '='
            if pos < buf.len() { buf[pos] = b'='; pos += 1; }
            // value
            let n = driver.config_get(key.name, &mut buf[pos..]);
            pos += n;
            // newline
            if pos < buf.len() { buf[pos] = b'\n'; pos += 1; }
        }
    }

    pos
}

// ============================================================================
// Mailbox Builder
// ============================================================================

/// Build a child mailbox buffer with MailboxHeader + spawn context KVs.
///
/// The MailboxHeader fields encode the spawn context that the child
/// reads at startup instead of querying GET_SPAWN_CONTEXT via IPC.
///
/// Layout: [0..64) MailboxHeader, [64..) KV pairs in wire format:
///   key_len(u8) + key + value_len(u8) + value, repeated kv_count times.
fn build_child_mailbox(
    buf: &mut [u8; 4096],
    filter: &crate::devd::SpawnFilter,
    context: Option<&SpawnChildContext>,
    priority: u8,
) {
    buf.fill(0);

    // Build MailboxHeader at offset 0
    // Note: parent_pid is stamped by kernel during exec_with_mailbox (offset 36)
    let mut hdr = abi::MailboxHeader::empty();
    hdr.priority = priority;
    hdr.flags = 0;

    if let Some(ctx) = context {
        hdr.bus_type = ctx.port_type; // Reuse bus_type for port_type
        hdr.bus_index = ctx.port_id;
        hdr.kv_count = 0; // Will be set below
    }

    // Write port name as a KV: "port.name" = filter.pattern
    let pname = filter.pattern_bytes();
    let mut kv_count: u8 = 0;
    let mut pos: usize = 64; // KVs start right after header

    // KV: port.name
    if !pname.is_empty() {
        let key = b"port.name";
        let klen = key.len();
        let vlen = pname.len().min(64);
        if pos + 1 + klen + 1 + vlen < 2048 {
            buf[pos] = klen as u8; pos += 1;
            buf[pos..pos + klen].copy_from_slice(key); pos += klen;
            buf[pos] = vlen as u8; pos += 1;
            buf[pos..pos + vlen].copy_from_slice(&pname[..vlen]); pos += vlen;
            kv_count += 1;
        }
    }

    // KV: metadata (if present)
    if let Some(ctx) = context {
        let mlen = ctx.metadata_len as usize;
        if mlen > 0 {
            let key = b"port.metadata";
            let klen = key.len();
            if pos + 1 + klen + 1 + mlen < 2048 {
                buf[pos] = klen as u8; pos += 1;
                buf[pos..pos + klen].copy_from_slice(key); pos += klen;
                buf[pos] = mlen as u8; pos += 1;
                buf[pos..pos + mlen].copy_from_slice(&ctx.metadata[..mlen]); pos += mlen;
                kv_count += 1;
            }
        }

        // KV: shmem_id (if non-zero)
        if ctx.shmem_id != 0 {
            let key = b"port.shmem_id";
            let klen = key.len();
            let val = ctx.shmem_id.to_le_bytes();
            let vlen = 4;
            if pos + 1 + klen + 1 + vlen < 2048 {
                buf[pos] = klen as u8; pos += 1;
                buf[pos..pos + klen].copy_from_slice(key); pos += klen;
                buf[pos] = vlen as u8; pos += 1;
                buf[pos..pos + vlen].copy_from_slice(&val); pos += vlen;
                kv_count += 1;
            }
        }

        // Context template KVs from SpawnChildContext
        let ctx_kv_n = (ctx.kv_count as usize).min(4);
        for i in 0..ctx_kv_n {
            let klen = ctx.kv_keys_len[i] as usize;
            let vlen = ctx.kv_values_len[i] as usize;
            if klen == 0 { continue; }
            if pos + 1 + klen + 1 + vlen < 2048 {
                buf[pos] = klen as u8; pos += 1;
                buf[pos..pos + klen].copy_from_slice(&ctx.kv_keys[i][..klen]); pos += klen;
                buf[pos] = vlen as u8; pos += 1;
                buf[pos..pos + vlen].copy_from_slice(&ctx.kv_values[i][..vlen]); pos += vlen;
                kv_count += 1;
            }
        }
    }

    hdr.kv_count = kv_count;

    // Write header to buf
    let hdr_bytes = unsafe {
        core::slice::from_raw_parts(&hdr as *const abi::MailboxHeader as *const u8, 64)
    };
    buf[..64].copy_from_slice(hdr_bytes);
}
