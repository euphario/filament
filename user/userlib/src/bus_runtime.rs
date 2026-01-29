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
//! - **devd channel** — commands from devd (SpawnChild, AttachDisk, etc.)
//! - **block port shmem handles** — data port notifications (SQ/CQ activity)
//! - **driver-registered handles** — custom handles via `ctx.watch_handle()`

use crate::bus::{
    BusMsg, BusMsgFlags, BusError, BusCtx, Driver, Disposition, SpawnContext,
    ChildId, PortId, KernelBusId, KernelBusInfo, KernelBusState, KernelBusChangeReason,
    BlockTransport, BlockPortConfig,
    MAX_CHILDREN, MAX_PORTS, MAX_KERNEL_BUSES, bus_msg,
};
use crate::bus_block::ShmemBlockPort;
use crate::devd::{DevdClient, DevdCommand, DriverState};
use crate::ipc::{Channel, Mux, MuxFilter, ObjHandle};
use crate::syscall::{self, Handle, LogLevel};

// ============================================================================
// Constants
// ============================================================================

/// Maximum block ports (provider + consumer combined).
const MAX_BLOCK_PORTS: usize = MAX_PORTS;

/// Maximum driver-registered handles.
const MAX_DRIVER_HANDLES: usize = 32;

/// Tag for devd channel in handle registry.
const TAG_DEVD: u32 = 0xFFFF_FF00;

/// Tag base for block port shmem handles.
const TAG_BLOCK_PORT_BASE: u32 = 0xFFFF_FE00;

/// Tag base for kernel bus channels.
const TAG_KERNEL_BUS_BASE: u32 = 0xFFFF_FD00;

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
// Child Tracking
// ============================================================================

struct ChildEntry {
    pid: u32,
    alive: bool,
}

// ============================================================================
// Kernel Bus Entry
// ============================================================================

/// Kernel bus protocol message types (wire format, first byte of payload).
mod kbus_proto {
    pub const STATE_SNAPSHOT: u8 = 0;
    pub const STATE_CHANGED: u8 = 1;
    pub const SET_DRIVER: u8 = 22;
}

/// A claimed kernel bus connection.
struct KernelBusEntry {
    channel: Channel,
    info: KernelBusInfo,
}

/// Parse a StateSnapshot message (16 bytes from kernel).
fn parse_state_snapshot(data: &[u8]) -> Option<KernelBusInfo> {
    if data.len() < 8 || data[0] != kbus_proto::STATE_SNAPSHOT {
        return None;
    }
    Some(KernelBusInfo {
        bus_type: data[1],
        bus_index: data[2],
        state: KernelBusState::from_u8(data[3]),
        device_count: data[4],
        capabilities: data[5],
    })
}

/// Parse a StateChanged message (4 bytes from kernel).
fn parse_state_changed(data: &[u8]) -> Option<(KernelBusState, KernelBusState, KernelBusChangeReason)> {
    if data.len() < 4 || data[0] != kbus_proto::STATE_CHANGED {
        return None;
    }
    Some((
        KernelBusState::from_u8(data[1]),
        KernelBusState::from_u8(data[2]),
        KernelBusChangeReason::from_u8(data[3]),
    ))
}

/// Build a SetDriver message.
fn build_set_driver_msg(driver_pid: u32) -> [u8; 5] {
    let mut msg = [0u8; 5];
    msg[0] = kbus_proto::SET_DRIVER;
    msg[1..5].copy_from_slice(&driver_pid.to_le_bytes());
    msg
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
    /// Connection to devd.
    devd: DevdClient,
    /// Event multiplexer — single Mux for all event sources.
    mux: Mux,
    /// Handle-to-tag registry for dispatch.
    handles: HandleRegistry,
    /// Block data ports.
    block_ports: [Option<ShmemBlockPort>; MAX_BLOCK_PORTS],
    /// Block port count.
    block_port_count: usize,
    /// Children.
    children: [Option<ChildEntry>; MAX_CHILDREN],
    /// Child count.
    child_count: usize,
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
            children: [const { None }; MAX_CHILDREN],
            child_count: 0,
            kernel_buses: [const { None }; MAX_KERNEL_BUSES],
            kernel_bus_count: 0,
            next_seq: 1,
            current_cmd_seq: 0,
            current_cmd_type: 0,
            name: name_buf,
            name_len: len,
            spawn_ctx: SpawnCtxCache::NotQueried,
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

    /// Register the devd channel with the Mux.
    fn register_devd_handle(&mut self) {
        if let Some(h) = self.devd.handle() {
            let _ = self.mux.add(h, MuxFilter::Readable);
            self.handles.add(h, TAG_DEVD);
        }
    }

    /// Register a block port's shmem handle with the Mux.
    fn register_block_port_handle(&mut self, port_idx: usize) {
        if let Some(ref port) = self.block_ports[port_idx] {
            if let Some(h) = port.mux_handle() {
                let tag = TAG_BLOCK_PORT_BASE + port_idx as u32;
                let _ = self.mux.add(h, MuxFilter::Readable);
                self.handles.add(h, tag);
            }
        }
    }
}

impl BusCtx for RuntimeCtx {
    fn reply(&mut self, original: &BusMsg, payload: &[u8]) -> Result<(), BusError> {
        let _ = original;
        let _ = payload;
        // Most replies are handled via dedicated BusCtx methods
        // (report_partitions, partition_ready, respond_info, ack_spawn).
        Ok(())
    }

    fn forward_down(&mut self, msg: &BusMsg) -> Result<(), BusError> {
        let _ = msg;
        Ok(())
    }

    fn send_down(
        &mut self,
        msg_type: u32,
        payload: &[u8],
        _deadline_ms: u32,
    ) -> Result<u32, BusError> {
        let seq = self.alloc_seq();
        let _ = msg_type;
        let _ = payload;
        Ok(seq)
    }

    fn send_up(&mut self, msg: &BusMsg) -> Result<(), BusError> {
        let _ = msg;
        Ok(())
    }

    fn emit_event(&mut self, msg_type: u32, payload: &[u8]) -> Result<(), BusError> {
        let _ = msg_type;
        let _ = payload;
        Ok(())
    }

    fn spawn_child(&mut self, binary_name: &[u8]) -> Result<ChildId, BusError> {
        let binary_str = core::str::from_utf8(binary_name).map_err(|_| BusError::SpawnFailed)?;
        let pid = syscall::exec(binary_str);
        if pid <= 0 {
            return Err(BusError::SpawnFailed);
        }

        for (i, slot) in self.children.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(ChildEntry {
                    pid: pid as u32,
                    alive: true,
                });
                self.child_count += 1;
                return Ok(ChildId(i as u8));
            }
        }
        Err(BusError::NoSpace)
    }

    fn child_count(&self) -> usize {
        self.child_count
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

        // Connect to the kernel bus port
        let mut ch = Channel::connect(path).map_err(|_| BusError::LinkDown)?;

        // Receive initial StateSnapshot
        let mut buf = [0u8; 16];
        let n = ch.recv(&mut buf).map_err(|_| BusError::LinkDown)?;
        let mut info = parse_state_snapshot(&buf[..n]).ok_or(BusError::InvalidMessage)?;

        // If bus is Resetting, wait for Safe
        if info.state == KernelBusState::Resetting {
            loop {
                let n = ch.recv(&mut buf).map_err(|_| BusError::LinkDown)?;
                if n == 0 { continue; }
                if buf[0] == kbus_proto::STATE_SNAPSHOT {
                    if let Some(new_info) = parse_state_snapshot(&buf[..n]) {
                        info = new_info;
                        if info.state == KernelBusState::Safe { break; }
                    }
                } else if buf[0] == kbus_proto::STATE_CHANGED && n >= 4 {
                    if KernelBusState::from_u8(buf[2]) == KernelBusState::Safe {
                        info.state = KernelBusState::Safe;
                        break;
                    }
                }
            }
        }

        // Send SetDriver with our PID
        let my_pid = syscall::getpid() as u32;
        let msg = build_set_driver_msg(my_pid);
        ch.send(&msg).map_err(|_| BusError::LinkDown)?;

        // Register the channel with Mux for ongoing state notifications
        let bus_idx = self.kernel_bus_count;
        let tag = TAG_KERNEL_BUS_BASE + bus_idx as u32;
        let ch_handle = ch.handle();
        self.mux.add(ch_handle, MuxFilter::Readable).map_err(|_| BusError::Internal)?;
        if !self.handles.add(ch_handle, tag) {
            let _ = self.mux.remove(ch_handle);
            return Err(BusError::NoSpace);
        }

        // Store the entry
        self.kernel_buses[bus_idx] = Some(KernelBusEntry { channel: ch, info });
        self.kernel_bus_count += 1;

        Ok((KernelBusId(bus_idx as u8), info))
    }

    fn kernel_bus_info(&self, id: KernelBusId) -> Option<&KernelBusInfo> {
        let idx = id.0 as usize;
        self.kernel_buses.get(idx)?.as_ref().map(|e| &e.info)
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

    fn report_partitions(
        &mut self,
        disk_shmem_id: u32,
        scheme: u8,
        parts: &[crate::query::PartitionInfoMsg],
    ) -> Result<(), BusError> {
        self.devd
            .report_partitions(disk_shmem_id, scheme, parts)
            .map_err(|_| BusError::Internal)
    }

    fn partition_ready(&mut self, name: &[u8], shmem_id: u32) -> Result<(), BusError> {
        self.devd
            .partition_ready(name, shmem_id)
            .map_err(|_| BusError::Internal)
    }

    fn respond_info(&mut self, seq_id: u32, info: &[u8]) -> Result<(), BusError> {
        self.devd
            .respond_info(seq_id, info)
            .map_err(|_| BusError::Internal)
    }

    fn ack_spawn(&mut self, seq_id: u32, result: i32, pid: u32) -> Result<(), BusError> {
        self.devd
            .ack_spawn(seq_id, result, pid)
            .map_err(|_| BusError::Internal)
    }

    fn register_port(
        &mut self,
        name: &[u8],
        port_type: crate::devd::PortType,
        parent: Option<&[u8]>,
    ) -> Result<(), BusError> {
        self.devd
            .register_port(name, port_type, parent)
            .map_err(|_| BusError::Internal)
    }

    fn report_state(&mut self, state: DriverState) -> Result<(), BusError> {
        self.devd
            .report_state(state)
            .map_err(|_| BusError::Internal)
    }

    fn spawn_context(&mut self) -> Result<&SpawnContext, BusError> {
        // Query devd on first call, cache the result
        if matches!(self.spawn_ctx, SpawnCtxCache::NotQueried) {
            match self.devd.get_spawn_context() {
                Ok(Some((name_buf, name_len, port_type))) => {
                    self.spawn_ctx = SpawnCtxCache::Cached(
                        SpawnContext::new(&name_buf, name_len, port_type),
                    );
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
}

// ============================================================================
// DevdCommand → BusMsg Translation
// ============================================================================

/// Translate a DevdCommand into a BusMsg for the driver's command() callback.
fn devd_command_to_bus_msg(cmd: &DevdCommand) -> Option<BusMsg> {
    match cmd {
        DevdCommand::AttachDisk {
            seq_id,
            shmem_id,
            source,
            source_len,
            block_size,
            block_count,
            ..
        } => {
            let mut msg = BusMsg::new(bus_msg::ATTACH_DISK);
            msg.seq_id = *seq_id;
            msg.write_u32(0, *shmem_id);
            msg.write_u32(4, *block_size);
            msg.write_u64(8, *block_count);
            msg.write_bytes(16, &source[..*source_len]);
            msg.write_u8(16 + *source_len, 0); // null terminator
            Some(msg)
        }
        DevdCommand::RegisterPartition {
            seq_id,
            index,
            name,
            name_len,
        } => {
            let mut msg = BusMsg::new(bus_msg::REGISTER_PARTITION);
            msg.seq_id = *seq_id;
            msg.write_u8(0, *index);
            msg.write_bytes(1, &name[..*name_len]);
            Some(msg)
        }
        DevdCommand::QueryInfo { seq_id } => {
            let mut msg = BusMsg::new(bus_msg::QUERY_INFO);
            msg.seq_id = *seq_id;
            Some(msg)
        }
        DevdCommand::MountPartition {
            seq_id,
            shmem_id,
            source,
            source_len,
            mount_point,
            mount_len,
            fs_hint,
            block_size,
            block_count,
        } => {
            let mut msg = BusMsg::new(bus_msg::MOUNT_PARTITION);
            msg.seq_id = *seq_id;
            msg.write_u32(0, *shmem_id);
            msg.write_u8(4, *fs_hint);
            msg.write_u32(5, *block_size);
            msg.write_u64(9, *block_count);
            msg.write_u8(17, *source_len as u8);
            msg.write_bytes(18, &source[..*source_len]);
            let mount_offset = 18 + *source_len;
            msg.write_u8(mount_offset, *mount_len as u8);
            msg.write_bytes(mount_offset + 1, &mount_point[..*mount_len]);
            Some(msg)
        }
        DevdCommand::SpawnChild { .. } => None,
        DevdCommand::StopChild { .. } => None,
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

        // Initialize the driver
        if let Err(e) = self.driver.init(&mut self.ctx) {
            let mut buf = [0u8; 64];
            buf[..19].copy_from_slice(b"[bus] init failed: ");
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
            };
            let elen = err_name.len().min(40);
            buf[19..19 + elen].copy_from_slice(&err_name.as_bytes()[..elen]);
            syscall::klog(LogLevel::Error, &buf[..19 + elen]);
            syscall::exit(1);
        }

        // Report ready to devd
        let _ = self.ctx.devd.report_state(DriverState::Ready);

        // Event loop: block on Mux, dispatch, repeat
        loop {
            let event = match self.ctx.mux.wait() {
                Ok(ev) => ev,
                Err(_) => continue,
            };

            let handle = event.handle;

            // Look up what this handle maps to
            let tag = match self.ctx.handles.find_tag(handle) {
                Some(t) => t,
                None => continue, // Unknown handle, skip
            };

            if tag == TAG_DEVD {
                // Devd channel is readable — process one command per wake
                // (Mux fires again if more data is pending on the channel)
                match self.ctx.devd.poll_command() {
                    Ok(Some(cmd)) => self.dispatch_devd_command(cmd),
                    Ok(None) | Err(_) => {}
                }
            } else if tag >= TAG_KERNEL_BUS_BASE && tag < TAG_BLOCK_PORT_BASE {
                // Kernel bus state notification
                let bus_idx = (tag - TAG_KERNEL_BUS_BASE) as usize;
                self.handle_kernel_bus_event(bus_idx);
            } else if tag >= TAG_BLOCK_PORT_BASE && tag < TAG_DEVD {
                // Block port shmem notification
                let port_idx = (tag - TAG_BLOCK_PORT_BASE) as usize;
                if port_idx < MAX_BLOCK_PORTS {
                    // Check for actual activity before dispatching
                    let has_activity = if let Some(ref port) = self.ctx.block_ports[port_idx] {
                        let dp = port.data_port();
                        dp.sq_pending() > 0 || dp.cq_pending() > 0 || dp.side_pending() > 0
                    } else {
                        false
                    };

                    if has_activity {
                        self.driver.data_ready(PortId(port_idx as u8), &mut self.ctx);
                    }
                }
            } else {
                // Driver-registered handle
                self.driver.handle_event(tag, handle, &mut self.ctx);
            }
        }
    }

    /// Handle a kernel bus state notification.
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
                        let old_state = self.ctx.kernel_buses[bus_idx]
                            .as_ref()
                            .map(|e| e.info.state)
                            .unwrap_or(KernelBusState::Safe);
                        if let Some(entry) = &mut self.ctx.kernel_buses[bus_idx] {
                            entry.info = new_info;
                        }
                        // Auto-forward to devd
                        let _ = self.ctx.devd.report_state(DriverState::Ready);
                        // Notify driver
                        self.driver.bus_state_changed(
                            KernelBusId(bus_idx as u8),
                            old_state,
                            new_info.state,
                            KernelBusChangeReason::Connected,
                            &mut self.ctx,
                        );
                    }
                }
                kbus_proto::STATE_CHANGED => {
                    if let Some((from, to, reason)) = parse_state_changed(&buf[..n]) {
                        if let Some(entry) = &mut self.ctx.kernel_buses[bus_idx] {
                            entry.info.state = to;
                        }
                        // Auto-forward to devd
                        let _ = self.ctx.devd.report_state(DriverState::Ready);
                        // Notify driver
                        self.driver.bus_state_changed(
                            KernelBusId(bus_idx as u8),
                            from,
                            to,
                            reason,
                            &mut self.ctx,
                        );
                    }
                }
                _ => {
                    // Unknown message type, ignore
                }
            }
        }
    }

    /// Dispatch a DevdCommand to the driver.
    fn dispatch_devd_command(&mut self, cmd: DevdCommand) {
        match &cmd {
            // SpawnChild is handled by the runtime, not the driver
            DevdCommand::SpawnChild {
                seq_id,
                binary,
                binary_len,
                filter,
                caps,
            } => {
                let binary_str = match core::str::from_utf8(&binary[..*binary_len]) {
                    Ok(s) => s,
                    Err(_) => {
                        let _ = self.ctx.devd.ack_spawn(*seq_id, -1, 0);
                        return;
                    }
                };
                let pid = if *caps != 0 {
                    syscall::exec_with_caps(binary_str, *caps)
                } else {
                    syscall::exec(binary_str)
                };
                if pid > 0 {
                    let _ = self.ctx.devd.ack_spawn(*seq_id, 0, pid as u32);
                    for slot in &mut self.ctx.children {
                        if slot.is_none() {
                            *slot = Some(ChildEntry {
                                pid: pid as u32,
                                alive: true,
                            });
                            self.ctx.child_count += 1;
                            break;
                        }
                    }
                } else {
                    let _ = self.ctx.devd.ack_spawn(*seq_id, pid as i32, 0);
                }
            }

            DevdCommand::StopChild { child_pid, .. } => {
                let _ = syscall::kill(*child_pid);
                for slot in &mut self.ctx.children {
                    if let Some(entry) = slot {
                        if entry.pid == *child_pid {
                            entry.alive = false;
                            break;
                        }
                    }
                }
            }

            _ => {
                if let Some(bus_msg) = devd_command_to_bus_msg(&cmd) {
                    self.ctx.current_cmd_seq = bus_msg.seq_id;
                    self.ctx.current_cmd_type = bus_msg.msg_type;

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
    }
}

// ============================================================================
// Entry Point
// ============================================================================

/// Entry point for all drivers using the bus framework. Never returns.
///
/// Connects to devd, creates a Mux, and enters the event-driven loop.
/// The runtime blocks on `Mux::wait()` — no polling, no sleep loops.
///
/// # Arguments
/// * `name` - Driver name (for logging and identification)
/// * `driver` - The Driver implementation
pub fn driver_main<D: Driver>(name: &[u8], driver: D) -> ! {
    // Connect to devd
    let devd = match DevdClient::connect() {
        Ok(c) => c,
        Err(_) => {
            syscall::klog(LogLevel::Error, b"[bus] devd connect failed");
            syscall::exit(1);
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
