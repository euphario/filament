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
    PortId, KernelBusId, KernelBusInfo,
    BlockTransport, BlockPortConfig,
    MAX_PORTS, MAX_KERNEL_BUSES, MAX_PENDING, bus_msg,
};
use crate::bus_block::ShmemBlockPort;
use crate::devd::{DevdClient, DevdCommand};
use crate::ipc::{Channel, Mux, MuxFilter};
use crate::query::{QueryHeader, SpawnChildContext, SpawnContextResponse, msg as query_msg, port_type as qport_type, error as query_error};
use crate::syscall::{self, Handle, LogLevel};

// ============================================================================
// Constants
// ============================================================================

/// Maximum block ports (provider + consumer combined).
const MAX_BLOCK_PORTS: usize = MAX_PORTS;

/// Maximum driver-registered handles.
const MAX_DRIVER_HANDLES: usize = 32;

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

/// Tag base for child supervision channels.
const TAG_CHILD_BASE: u32 = 0xFFFF_FC00;

// ============================================================================
// Handle Registry
// ============================================================================

/// Maps Mux handles to tags for dispatch.
struct HandleRegistry {
    entries: [(Handle, u32); MAX_DRIVER_HANDLES],
    count: usize,
}

impl HandleRegistry {
    const fn new() -> Self {
        Self {
            entries: [(Handle(0), 0); MAX_DRIVER_HANDLES],
            count: 0,
        }
    }

    fn add(&mut self, handle: Handle, tag: u32) -> bool {
        if self.count >= MAX_DRIVER_HANDLES {
            return false;
        }
        self.entries[self.count] = (handle, tag);
        self.count += 1;
        true
    }

    fn remove(&mut self, handle: Handle) -> bool {
        for i in 0..self.count {
            if self.entries[i].0 == handle {
                // Swap with last
                self.count -= 1;
                if i < self.count {
                    self.entries[i] = self.entries[self.count];
                }
                return true;
            }
        }
        false
    }

    fn find_tag(&self, handle: Handle) -> Option<u32> {
        for i in 0..self.count {
            if self.entries[i].0 == handle {
                return Some(self.entries[i].1);
            }
        }
        None
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

/// A child spawned by this driver via EXEC_WITH_CHANNEL.
struct ChildEntry {
    /// Supervision channel to the child.
    channel: Channel,
    /// Child PID.
    pid: u32,
    /// Cached spawn context for answering GET_SPAWN_CONTEXT.
    ctx: ChildSpawnCtx,
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

    /// Return the nearest deadline in ms from now, or 0 if no pending deadlines.
    ///
    /// Used to set the Mux timeout so the event loop wakes up to fire
    /// expired deadline callbacks even if no I/O event arrives.
    fn nearest_deadline_ms(&self) -> u32 {
        if self.pending_count == 0 {
            return 0;
        }
        let now = syscall::gettime();
        let mut nearest: u64 = u64::MAX;
        for slot in &self.pending {
            if let Some(req) = slot {
                if req.deadline_ns > 0 && req.deadline_ns < nearest {
                    nearest = req.deadline_ns;
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

    /// Register the devd channel with the Mux.
    fn register_devd_handle(&mut self) {
        if let Some(h) = self.devd.handle() {
            if self.mux.add(h, MuxFilter::Readable).is_err() {
                syscall::klog(LogLevel::Error, b"[bus] devd mux_add failed");
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
                if self.mux.add(h, MuxFilter::Readable).is_err() {
                    syscall::klog(LogLevel::Error, b"[bus] port mux_add failed");
                }
                self.handles.add(h, tag);
            }
        }
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

    fn name(&self) -> &[u8] {
        &self.name[..self.name_len]
    }

    fn respond_info(&mut self, seq_id: u32, info: &[u8]) -> Result<(), BusError> {
        self.devd
            .respond_info(seq_id, info)
            .map_err(|_| BusError::Internal)
    }

    fn register_port_with_info(
        &mut self,
        info: &abi::PortInfo,
        shmem_id: u32,
    ) -> Result<(), BusError> {
        // Register with devd via supervision channel
        // Port starts in Safe state — supervisor will fire rules
        self.devd
            .register_port_info(info, shmem_id)
            .map_err(|_| BusError::Internal)?;
        Ok(())
    }

    fn set_port_state(
        &mut self,
        name: &[u8],
        state: abi::PortState,
    ) -> Result<(), BusError> {
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

/// The main runtime that manages the event loop and dispatches to Driver.
///
/// Uses a Mux-based event loop: blocks until an event source fires,
/// dispatches to the appropriate Driver callback, then blocks again.
/// No polling, no sleep loops.
pub struct DriverRuntime<D: Driver> {
    driver: D,
    ctx: RuntimeCtx,
}

impl<D: Driver> DriverRuntime<D> {
    /// Create a new runtime with the given driver and devd connection.
    fn new(driver: D, devd: DevdClient, mux: Mux, name: &[u8]) -> Self {
        Self {
            driver,
            ctx: RuntimeCtx::new(devd, mux, name),
        }
    }

    /// Run the event loop. Never returns.
    fn run(&mut self) -> ! {
        // Register devd channel with Mux
        self.ctx.register_devd_handle();

        // Initialize the driver (hardware is Safe — bring it up)
        if let Err(e) = self.driver.reset(&mut self.ctx) {
            let mut buf = [0u8; 64];
            buf[..20].copy_from_slice(b"[bus] reset failed: ");
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
            let elen = err_name.len().min(40);
            buf[20..20 + elen].copy_from_slice(&err_name.as_bytes()[..elen]);
            crate::ulog::flush();
            syscall::klog(LogLevel::Error, &buf[..20 + elen]);
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

            // Block until next event or nearest deadline
            let timeout_ms = self.ctx.nearest_deadline_ms();
            let event = if timeout_ms > 0 {
                match self.ctx.mux.wait_timeout(timeout_ms) {
                    Ok(ev) => ev,
                    Err(_) => continue, // Timeout or error — loop to check deadlines
                }
            } else {
                match self.ctx.mux.wait() {
                    Ok(ev) => ev,
                    Err(_) => continue,
                }
            };

            let handle = event.handle;

            // Look up what this handle maps to
            let tag = match self.ctx.handles.find_tag(handle) {
                Some(t) => t,
                None => continue, // Unknown handle, skip
            };

            if tag == TAG_DEVD {
                // Devd channel is readable — read raw bytes first.
                // Check if the message is a response to a forwarded child request.
                // If so, relay it to the child and skip normal dispatch.
                let mut raw_buf = [0u8; 512];
                match self.ctx.devd.raw_recv(&mut raw_buf) {
                    Ok(Some(n)) if n >= QueryHeader::SIZE => {
                        if !self.try_relay_to_child(&raw_buf, n) {
                            // Not a forwarded response — parse as normal command
                            match DevdClient::parse_command_buf(&raw_buf[..n]) {
                                Ok(Some(cmd)) => {
                                    self.dispatch_devd_command(cmd);
                                }
                                Ok(None) => {}
                                Err(_) => {}
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(_) => {
                        // Devd channel broken — remove from Mux to prevent hot-loop.
                        // Driver continues running with existing state but cannot
                        // receive new commands from devd.
                        let _ = self.ctx.mux.remove(handle);
                        self.ctx.handles.remove(handle);
                    }
                }
            } else if tag >= TAG_CHILD_BASE && tag < TAG_KERNEL_BUS_BASE {
                // Child supervision channel message — relay or answer locally
                let child_idx = (tag - TAG_CHILD_BASE) as usize;
                self.handle_child_message(child_idx);
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
            }

            // Flush any structured logs emitted during dispatch
            crate::ulog::flush();
        }
    }

    /// Handle a kernel bus channel event (state snapshot or supervision message).
    fn handle_kernel_bus_event(&mut self, bus_idx: usize) {
        let mut buf = [0u8; 16];

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
                _ => {
                    // Unknown message type, ignore
                }
            }
        }
    }

    /// Dispatch a DevdCommand to the driver.
    ///
    /// All commands go through BusMsg — single conversion path.
    /// The framework intercepts known types (spawn, stop, config) before
    /// the driver sees them. Everything else goes to `driver.command()`.
    fn dispatch_devd_command(&mut self, cmd: DevdCommand) {
        // SpawnChild: framework spawns the child with a supervision channel.
        // Creates a channel pair, passes one end to child via exec_with_channel,
        // keeps the other end for parent-child relay.
        if let DevdCommand::SpawnChild { seq_id, binary, binary_len, caps, filter, context } = &cmd {
            let name = core::str::from_utf8(&binary[..*binary_len]).unwrap_or("???");

            // Create channel pair for parent-child supervision
            let spawn_result = match Channel::pair() {
                Ok((parent_end, child_end)) => {
                    let child_handle = child_end.into_raw_handle();
                    let p = syscall::exec_with_channel(name, *caps, child_handle);
                    if p > 0 {
                        Some((parent_end, p as u32))
                    } else {
                        drop(parent_end);
                        None
                    }
                }
                Err(_) => None,
            };

            // Fallback: spawn without channel
            let (result, child_pid) = if let Some((parent_end, pid)) = spawn_result {
                // Store ChildEntry for relay
                let stored = self.store_child_entry(parent_end, pid, filter, context.as_ref());
                if !stored {
                    syscall::klog(LogLevel::Warn, b"[bus] no child slot");
                }
                (0i32, pid)
            } else {
                // Channel pair failed — fall back to plain spawn (no relay)
                let pid = if *caps != 0 {
                    syscall::exec_with_caps(name, *caps)
                } else {
                    syscall::exec(name)
                };
                if pid > 0 { (0i32, pid as u32) } else { (-1i32, 0u32) }
            };

            if self.ctx.devd.ack_spawn(*seq_id, result, child_pid).is_err() {
                syscall::klog(LogLevel::Warn, b"[bus] spawn_ack failed");
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
                let mut buf = [0u8; 512];

                let len = if key.is_empty() {
                    build_config_summary(&self.driver, &mut buf)
                } else {
                    self.driver.config_get(key, &mut buf)
                };

                let _ = self.ctx.respond_info(bus_msg.seq_id, &buf[..len]);
            }

            bus_msg::CONFIG_SET => {
                let (key, value) = bus_msg.parse_config_kv();
                let mut buf = [0u8; 128];

                let keys = self.driver.config_keys();
                let valid = keys.iter().any(|k| k.name == key && k.writable);

                let len = if valid {
                    self.driver.config_set(key, value, &mut buf, &mut self.ctx)
                } else if keys.iter().any(|k| k.name == key) {
                    copy_static(&mut buf, b"ERR read-only key\n")
                } else {
                    copy_static(&mut buf, b"ERR unknown key\n")
                };

                let _ = self.ctx.respond_info(bus_msg.seq_id, &buf[..len]);
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

    /// Store a ChildEntry for a newly spawned child and register its channel
    /// with the Mux.
    fn store_child_entry(
        &mut self,
        parent_end: Channel,
        pid: u32,
        filter: &crate::devd::SpawnFilter,
        context: Option<&SpawnChildContext>,
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

        // Register channel with Mux
        let ch_handle = parent_end.handle();
        let tag = TAG_CHILD_BASE + slot as u32;
        if self.ctx.mux.add(ch_handle, MuxFilter::Readable).is_err() {
            syscall::klog(LogLevel::Error, b"[bus] child mux_add failed");
            return false;
        }
        self.ctx.handles.add(ch_handle, tag);

        self.ctx.children[slot] = Some(ChildEntry {
            channel: parent_end,
            pid,
            ctx: spawn_ctx,
        });
        self.ctx.child_count += 1;

        true
    }

    /// Handle a message from a child's supervision channel.
    ///
    /// Dispatches locally for GET_SPAWN_CONTEXT, forwards everything else
    /// to devd with response relay tracking.
    fn handle_child_message(&mut self, child_idx: usize) {
        let mut buf = [0u8; 512];

        // Read message from child
        let n = match &mut self.ctx.children[child_idx] {
            Some(entry) => match entry.channel.try_recv(&mut buf) {
                Ok(Some(n)) if n >= QueryHeader::SIZE => n,
                Ok(Some(_)) => return, // Too small, ignore
                Ok(None) => return,    // No data yet
                Err(_) => {
                    // Channel error (PeerClosed, etc.) — child died.
                    // Remove from Mux and free the slot to prevent hot-loop starvation.
                    syscall::klog(LogLevel::Info, b"[bus] child channel closed");
                    let handle = entry.channel.handle();
                    let _ = self.ctx.mux.remove(handle);
                    self.ctx.handles.remove(handle);
                    self.ctx.children[child_idx] = None;
                    if self.ctx.child_count > 0 { self.ctx.child_count -= 1; }
                    return;
                }
            },
            None => return,
        };

        let header = match QueryHeader::from_bytes(&buf[..n]) {
            Some(h) => h,
            None => return,
        };

        match header.msg_type {
            // GET_SPAWN_CONTEXT: answer locally from stored ChildSpawnCtx
            query_msg::GET_SPAWN_CONTEXT => {
                self.reply_spawn_context(child_idx, header.seq_id);
            }

            // REGISTER_PORT_INFO / SET_PORT_STATE / QUERY_PORT / UPDATE_PORT_SHMEM_ID:
            // Forward to devd with response relay tracking
            query_msg::REGISTER_PORT_INFO
            | query_msg::SET_PORT_STATE
            | query_msg::QUERY_PORT
            | query_msg::UPDATE_PORT_SHMEM_ID => {
                self.forward_to_devd(child_idx, header.seq_id, &buf[..n]);
            }

            // STATE_CHANGE: fire-and-forget forward to devd
            query_msg::STATE_CHANGE => {
                let _ = self.ctx.devd.raw_send(&buf[..n]);
            }

            // LOG_MESSAGE: fire-and-forget forward to devd
            query_msg::LOG_MESSAGE => {
                let _ = self.ctx.devd.raw_send(&buf[..n]);
            }

            // Unknown: forward to devd
            _ => {
                self.forward_to_devd(child_idx, header.seq_id, &buf[..n]);
            }
        }
    }

    /// Reply to a child's GET_SPAWN_CONTEXT with locally cached data.
    fn reply_spawn_context(&mut self, child_idx: usize, seq_id: u32) {
        let entry = match &self.ctx.children[child_idx] {
            Some(e) => e,
            None => return,
        };
        let c = &entry.ctx;

        // Convert port_type to legacy port_type constant for wire format
        let resp = SpawnContextResponse {
            header: QueryHeader::new(query_msg::SPAWN_CONTEXT, seq_id),
            result: query_error::OK,
            port_type: c.port_type,
            port_name_len: c.port_name_len,
            metadata_len: c.metadata_len,
            port_id: c.port_id,
        };

        // Build KV refs for write_to_full
        let kv_n = c.context_kv_count as usize;
        let mut kv_refs: [(&[u8], &[u8]); 4] = [(&[], &[]); 4];
        for i in 0..kv_n {
            let klen = c.context_kvs[i].1 as usize;
            let vlen = c.context_kvs[i].3 as usize;
            kv_refs[i] = (&c.context_kvs[i].0[..klen], &c.context_kvs[i].2[..vlen]);
        }

        let port_name = &c.port_name[..c.port_name_len as usize];
        let metadata = &c.metadata[..c.metadata_len as usize];

        let mut reply_buf = [0u8; 512];
        if let Some(len) = resp.write_to_full(&mut reply_buf, port_name, metadata, &kv_refs[..kv_n]) {
            // Also append shmem_id after the standard spawn context response.
            // The child's bus_runtime will pick this up to populate SpawnContext.shmem_id.
            // Wire: after KV section, append shmem_id(4 bytes LE).
            if len + 4 <= reply_buf.len() {
                reply_buf[len..len + 4].copy_from_slice(&c.shmem_id.to_le_bytes());
                let total = len + 4;
                if let Some(entry) = &mut self.ctx.children[child_idx] {
                    let _ = entry.channel.send(&reply_buf[..total]);
                }
            }
        }
    }

    /// Forward a child message to devd with seq_id rewriting for response relay.
    fn forward_to_devd(&mut self, child_idx: usize, child_seq_id: u32, msg: &[u8]) {
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
            syscall::klog(LogLevel::Error, b"[bus] fwd table full, dropping child request");
            return;
        }

        // Rewrite seq_id in message and send to devd
        let mut fwd = [0u8; 512];
        let len = msg.len().min(512);
        fwd[..len].copy_from_slice(&msg[..len]);
        // seq_id is at offset 4..8 in QueryHeader
        fwd[4..8].copy_from_slice(&devd_seq.to_le_bytes());
        let _ = self.ctx.devd.raw_send(&fwd[..len]);
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

        // Rewrite seq_id back to child's original and send
        let mut relay = [0u8; 512];
        let rlen = len.min(512);
        relay[..rlen].copy_from_slice(&buf[..rlen]);
        relay[4..8].copy_from_slice(&child_seq.to_le_bytes());

        if let Some(entry) = &mut self.ctx.children[child_idx as usize] {
            let _ = entry.channel.send(&relay[..rlen]);
        }

        true
    }
}

// ============================================================================
// Entry Point
// ============================================================================

/// Entry point for all drivers using the bus framework. Never returns.
///
/// Checks Handle::SUPERVISION (slot 4) first for tree mode — if valid,
/// uses it as the supervision channel to the parent driver. Otherwise
/// connects to devd directly as a root-level driver.
///
/// # Arguments
/// * `name` - Driver name (for logging and identification)
/// * `driver` - The Driver implementation
pub fn driver_main<D: Driver>(name: &[u8], driver: D) -> ! {
    // Try tree mode first: check if handle 4 (SUPERVISION) is a valid, connected channel.
    // Probe with a zero-length read:
    //   BadFd → handle doesn't exist (root mode)
    //   ConnectionReset/PeerClosed → handle exists but peer dropped (fall back to root mode)
    //   WouldBlock/Ok → handle exists and peer is alive (tree mode)
    let devd = {
        let mut probe = [0u8; 0];
        let supervision_alive = match crate::syscall::read(abi::Handle::SUPERVISION, &mut probe) {
            Err(crate::error::SysError::BadFd) => false,
            _ => true, // WouldBlock or Ok means handle exists and peer is alive
        };

        if supervision_alive {
            let ch = Channel::from_raw_handle(abi::Handle::SUPERVISION);
            DevdClient::from_channel(ch)
        } else {
            match DevdClient::connect() {
                Ok(c) => c,
                Err(_) => {
                    syscall::klog(LogLevel::Error, b"[bus] devd connect failed");
                    syscall::exit(1);
                }
            }
        }
    };

    // Create the event loop Mux
    let mux = match Mux::new() {
        Ok(m) => m,
        Err(_) => {
            syscall::klog(LogLevel::Error, b"[bus] mux create failed");
            syscall::exit(1);
        }
    };

    let mut runtime = DriverRuntime::new(driver, devd, mux, name);
    runtime.run()
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
