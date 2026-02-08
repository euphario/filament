//! Device Supervisor Daemon (devd)
//!
//! Service registry and supervisor for the microkernel. Acts as PID 1 (init).
//! Uses only the unified 5-syscall interface (open, read, write, map, close).
//!
//! ## Responsibilities
//! - Track all services and their registered ports
//! - Route client queries to correct service
//! - Auto-spawn drivers via PORT_RULES when ports transition to Ready
//! - Supervise with exponential backoff and restart on crash
//!
//! ## Architecture
//!
//! devd uses a modular, trait-based design:
//! - `service.rs` - Service state machine (ServiceManager trait)
//! - `ports.rs` - Port registry (PortRegistry trait)
//! - `process.rs` - Process management (ProcessManager trait)
//! - `rules.rs` - Unified port rules engine (PortClass → driver binary)
//!
//! Each module is testable independently via trait mocking.

#![no_std]
#![no_main]

extern crate abi;

mod service;
mod ports;
mod process;
mod devices;
mod query;
mod rules;

use userlib::syscall;
use userlib::{uinfo, uerror};
use userlib::ipc::{Port, Timer, EventLoop, ObjHandle, Channel};
use userlib::error::{SysError, SysResult};
use userlib::query::{
    AttachDisk, ReportPartitions, RegisterPartitionMsg, PartitionReadyMsg,
    PartitionInfoMsg,
    QueryHeader, msg, fs_hint, part_scheme,
};

use service::{
    ServiceState, ServiceManager, ServiceRegistry,
    MAX_SERVICES, MAX_RESTARTS,
    INITIAL_BACKOFF_MS, MAX_BACKOFF_MS, FAILED_RETRY_MS,
};
use ports::{PortRegistry, Ports};
use process::{ProcessManager, SyscallProcessManager};
use devices::{DeviceStore, DeviceRegistry};
use query::{QueryHandler, MSG_BUFFER_SIZE, MAX_QUERY_CLIENTS};

// =============================================================================
// Admin Client Handling (shell text commands)
// =============================================================================

/// Maximum number of admin clients (shell connections)
const MAX_ADMIN_CLIENTS: usize = 4;

/// Admin client connection (for text commands from shell)
struct AdminClient {
    channel: Channel,
}

/// Admin client registry
struct AdminClients {
    clients: [Option<AdminClient>; MAX_ADMIN_CLIENTS],
}

/// Copy a static byte slice into a buffer, returning the number of bytes copied.
fn copy_static(dst: &mut [u8], src: &[u8]) -> usize {
    let len = src.len().min(dst.len());
    dst[..len].copy_from_slice(&src[..len]);
    len
}

/// Trim whitespace and newlines from a byte slice
fn trim_bytes(b: &[u8]) -> &[u8] {
    let mut start = 0;
    let mut end = b.len();
    while start < end && (b[start] == b' ' || b[start] == b'\n' || b[start] == b'\r' || b[start] == b'\t') {
        start += 1;
    }
    while end > start && (b[end - 1] == b' ' || b[end - 1] == b'\n' || b[end - 1] == b'\r' || b[end - 1] == b'\t') {
        end -= 1;
    }
    &b[start..end]
}

impl AdminClients {
    const fn new() -> Self {
        Self {
            clients: [const { None }; MAX_ADMIN_CLIENTS],
        }
    }

    fn add(&mut self, channel: Channel) -> Option<usize> {
        let slot = (0..MAX_ADMIN_CLIENTS).find(|&i| self.clients[i].is_none())?;
        self.clients[slot] = Some(AdminClient { channel });
        Some(slot)
    }

    fn remove(&mut self, slot: usize) -> Option<Channel> {
        if slot >= MAX_ADMIN_CLIENTS {
            return None;
        }
        self.clients[slot].take().map(|c| c.channel)
    }

    fn find_by_handle(&self, handle: ObjHandle) -> Option<usize> {
        (0..MAX_ADMIN_CLIENTS).find(|&i| {
            self.clients[i]
                .as_ref()
                .map(|c| c.channel.handle() == handle)
                .unwrap_or(false)
        })
    }

    fn get_mut(&mut self, slot: usize) -> Option<&mut AdminClient> {
        self.clients.get_mut(slot).and_then(|c| c.as_mut())
    }
}

// =============================================================================
// Logging
// =============================================================================

// All logging uses structured ulog macros from userlib:
// - uinfo!("devd", "event_name"; key = val, ...)
// - uerror!("devd", "error_name"; key = val, ...)

// =============================================================================
// Devd - Service Supervisor
// =============================================================================

/// Maximum number of recently spawned dynamic PIDs to track
const MAX_RECENT_DYNAMIC_PIDS: usize = 8;

/// Maximum spawn contexts to track (PID -> trigger port)
const MAX_SPAWN_CONTEXTS: usize = 32;

/// Maximum in-flight spawn commands to track
const MAX_INFLIGHT_SPAWNS: usize = 8;

/// Spawn context entry - maps child PID to trigger port
#[derive(Clone, Copy)]
struct SpawnContext {
    /// Child process PID (0 = empty slot)
    pid: u32,
    /// Port type that triggered spawn
    port_type: u8,
    /// Trigger port name
    port_name: [u8; 32],
    /// Length of port name
    port_name_len: u8,
    /// Opaque metadata from the port registration (e.g., BAR0 info)
    metadata: [u8; 64],
    /// Length of metadata
    metadata_len: u8,
}

impl SpawnContext {
    const fn empty() -> Self {
        Self {
            pid: 0,
            port_type: 0,
            port_name: [0u8; 32],
            port_name_len: 0,
            metadata: [0u8; 64],
            metadata_len: 0,
        }
    }
}

/// In-flight spawn tracking - maps seq_id to port info for SPAWN_ACK
#[derive(Clone, Copy)]
struct InflightSpawn {
    /// Sequence ID (0 = empty slot)
    seq_id: u32,
    /// Port type that triggered spawn
    port_type: u8,
    /// Trigger port name
    port_name: [u8; 32],
    /// Length of port name
    port_name_len: u8,
    /// Binary name that was spawned
    binary_name: [u8; 16],
    /// Length of binary name
    binary_name_len: u8,
    /// Service index for parent-delegated spawns (0xFF = not a service spawn)
    service_idx: u8,
    /// Capability bits for the spawned process
    caps: u64,
    /// Link ID tying spawn to port (carried through to service slot)
    link_id: u32,
}

impl InflightSpawn {
    const fn empty() -> Self {
        Self {
            seq_id: 0,
            port_type: 0,
            port_name: [0u8; 32],
            port_name_len: 0,
            binary_name: [0u8; 16],
            binary_name_len: 0,
            service_idx: 0xFF,
            caps: 0,
            link_id: 0,
        }
    }
}

/// Maximum pending info queries
const MAX_PENDING_INFO_QUERIES: usize = 8;

// =============================================================================
// Orchestration State Machines (for block device stack)
// =============================================================================

/// Maximum number of tracked disks
const MAX_TRACKED_DISKS: usize = 8;

/// Maximum number of tracked partitions per disk
const MAX_PARTITIONS_PER_DISK: usize = 8;

/// Disk lifecycle state machine
///
/// State transitions:
///   Empty -> Discovered: Block port registered with shmem_id
///   Discovered -> AttachSent: AttachDisk sent to partd
///   AttachSent -> PartitionsReported: ReportPartitions received
///   PartitionsReported -> Ready: All partitions processed
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DiskState {
    /// Slot is empty
    Empty,
    /// Disk discovered, tracking started
    Discovered,
    /// AttachDisk sent to partd, waiting for response
    AttachSent { seq_id: u32 },
    /// ReportPartitions received, processing partitions
    PartitionsReported,
    /// All partitions processed
    Ready,
}

/// Partition lifecycle state machine
///
/// State transitions:
///   Empty -> Discovered: Reported by partd in ReportPartitions
///   Discovered -> Registering: RegisterPartition sent to partd
///   Registering -> Ready: PartitionReady received
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PartitionState {
    /// Slot is empty
    Empty,
    /// Partition discovered (from ReportPartitions)
    Discovered,
    /// RegisterPartition sent, waiting for PartitionReady
    Registering { seq_id: u32 },
    /// DataPort ready
    Ready { shmem_id: u32 },
}

/// Information about a tracked disk (from Block port registration)
#[derive(Clone, Copy)]
struct TrackedDisk {
    /// Current state in lifecycle
    state: DiskState,
    /// shmem_id of the DataPort
    shmem_id: u32,
    /// Block size in bytes
    block_size: u32,
    /// Total block count
    block_count: u64,
    /// Port name (e.g., "disk0:")
    port_name: [u8; 32],
    /// Length of port name
    port_name_len: u8,
    /// Owner service index
    owner_idx: u8,
    /// Number of partitions discovered
    partition_count: u8,
    /// Partition scheme (0=MBR, 1=GPT)
    scheme: u8,
    /// Partition info (filled by REPORT_PARTITIONS)
    partitions: [TrackedPartition; MAX_PARTITIONS_PER_DISK],
}

impl TrackedDisk {
    const fn empty() -> Self {
        Self {
            state: DiskState::Empty,
            shmem_id: 0,
            block_size: 0,
            block_count: 0,
            port_name: [0u8; 32],
            port_name_len: 0,
            owner_idx: 0,
            partition_count: 0,
            scheme: 0,
            partitions: [const { TrackedPartition::empty() }; MAX_PARTITIONS_PER_DISK],
        }
    }

    /// Transition to a new state (validates transition is legal)
    fn transition(&mut self, new_state: DiskState) -> bool {
        let valid = match (&self.state, &new_state) {
            (DiskState::Empty, DiskState::Discovered) => true,
            (DiskState::Discovered, DiskState::AttachSent { .. }) => true,
            (DiskState::AttachSent { .. }, DiskState::PartitionsReported) => true,
            (DiskState::PartitionsReported, DiskState::Ready) => true,
            // Backward: partd crashed, re-discover
            (DiskState::AttachSent { .. }, DiskState::Discovered) => true,
            (DiskState::PartitionsReported, DiskState::Discovered) => true,
            (DiskState::Ready, DiskState::Discovered) => true,
            _ => false,
        };
        if valid {
            self.state = new_state;
        }
        valid
    }
}

/// Information about a partition (from REPORT_PARTITIONS)
#[derive(Clone, Copy)]
struct TrackedPartition {
    /// Current state in lifecycle
    state: PartitionState,
    /// Partition index (from disk's partition table)
    index: u8,
    /// MBR partition type
    part_type: u8,
    /// Is bootable?
    bootable: bool,
    /// Start LBA
    start_lba: u64,
    /// Size in sectors
    size_sectors: u64,
    /// Assigned port name (e.g., "part0:")
    assigned_name: [u8; 32],
    /// Length of assigned name
    assigned_name_len: u8,
}

impl TrackedPartition {
    const fn empty() -> Self {
        Self {
            state: PartitionState::Empty,
            index: 0,
            part_type: 0,
            bootable: false,
            start_lba: 0,
            size_sectors: 0,
            assigned_name: [0u8; 32],
            assigned_name_len: 0,
        }
    }

    /// Transition to a new state (validates transition is legal)
    fn transition(&mut self, new_state: PartitionState) -> bool {
        let valid = match (&self.state, &new_state) {
            (PartitionState::Empty, PartitionState::Discovered) => true,
            (PartitionState::Discovered, PartitionState::Registering { .. }) => true,
            (PartitionState::Registering { .. }, PartitionState::Ready { .. }) => true,
            _ => false,
        };
        if valid {
            self.state = new_state;
        }
        valid
    }

    /// Get shmem_id if partition is Ready or later
    fn shmem_id(&self) -> Option<u32> {
        match self.state {
            PartitionState::Ready { shmem_id } => Some(shmem_id),
            _ => None,
        }
    }
}

/// A pending service info query waiting for driver response
#[derive(Clone, Copy)]
struct PendingInfoQuery {
    /// Sequence ID we used when forwarding (0 = empty slot)
    forward_seq_id: u32,
    /// Original client's sequence ID
    client_seq_id: u32,
    /// Client slot to relay response to
    client_slot: u8,
    /// Driver slot we forwarded to
    driver_slot: u8,
}

impl PendingInfoQuery {
    const fn empty() -> Self {
        Self {
            forward_seq_id: 0,
            client_seq_id: 0,
            client_slot: 0,
            driver_slot: 0,
        }
    }
}

// =============================================================================
// Log Buffer
// =============================================================================

/// Maximum log entries to buffer
const MAX_LOG_ENTRIES: usize = 64;
/// Maximum length of a single log message
const MAX_LOG_MSG_LEN: usize = 128;

/// A single log entry
#[derive(Clone, Copy)]
struct LogEntry {
    /// Service index that sent the log (0xFF = devd itself)
    service_idx: u8,
    /// Log level
    level: u8,
    /// Message length
    len: u8,
    /// Message text
    msg: [u8; MAX_LOG_MSG_LEN],
}

impl LogEntry {
    const fn empty() -> Self {
        Self {
            service_idx: 0xFF,
            level: 0,
            len: 0,
            msg: [0u8; MAX_LOG_MSG_LEN],
        }
    }
}

/// Ring buffer for log entries
struct LogBuffer {
    entries: [LogEntry; MAX_LOG_ENTRIES],
    /// Write position (next entry to write)
    write_pos: usize,
    /// Number of entries (max = MAX_LOG_ENTRIES)
    count: usize,
}

impl LogBuffer {
    const fn new() -> Self {
        Self {
            entries: [const { LogEntry::empty() }; MAX_LOG_ENTRIES],
            write_pos: 0,
            count: 0,
        }
    }

    /// Add a log entry
    fn push(&mut self, service_idx: u8, level: u8, msg: &[u8]) {
        let len = msg.len().min(MAX_LOG_MSG_LEN);
        let mut entry = LogEntry::empty();
        entry.service_idx = service_idx;
        entry.level = level;
        entry.len = len as u8;
        entry.msg[..len].copy_from_slice(&msg[..len]);

        self.entries[self.write_pos] = entry;
        self.write_pos = (self.write_pos + 1) % MAX_LOG_ENTRIES;
        if self.count < MAX_LOG_ENTRIES {
            self.count += 1;
        }
    }

    /// Format recent entries into a buffer (oldest first for display)
    fn format_recent(&self, max_count: usize, buf: &mut [u8]) -> usize {
        let count = self.count.min(max_count);
        if count == 0 {
            return 0;
        }

        // Calculate starting position (oldest entry to show)
        let start = if self.write_pos >= count {
            self.write_pos - count
        } else {
            MAX_LOG_ENTRIES - (count - self.write_pos)
        };

        let mut pos = 0;
        for i in 0..count {
            let idx = (start + i) % MAX_LOG_ENTRIES;
            let entry = &self.entries[idx];

            if pos >= buf.len() {
                break;
            }

            // Format: "[LEVEL] message\n"
            let level_str = match entry.level {
                0 => b"ERR ",
                1 => b"WARN",
                2 => b"INFO",
                _ => b"DBG ",
            };
            let msg_len = entry.len as usize;

            // Check space
            let needed = 1 + level_str.len() + 2 + msg_len + 1; // [LEVEL] msg\n
            if pos + needed > buf.len() {
                break;
            }

            buf[pos] = b'[';
            pos += 1;
            buf[pos..pos + level_str.len()].copy_from_slice(level_str);
            pos += level_str.len();
            buf[pos] = b']';
            pos += 1;
            buf[pos] = b' ';
            pos += 1;
            buf[pos..pos + msg_len].copy_from_slice(&entry.msg[..msg_len]);
            pos += msg_len;
            buf[pos] = b'\n';
            pos += 1;
        }

        pos
    }
}

pub struct Devd {
    /// Query port for driver supervision and admin commands
    query_port: Option<Port>,
    /// Event loop
    events: Option<EventLoop>,
    /// Service registry
    services: ServiceRegistry,
    /// Port registry
    ports: Ports,
    /// Process manager
    process_mgr: SyscallProcessManager,
    /// Device registry (for hierarchical queries)
    devices: DeviceRegistry,
    /// Query handler for client connections
    query_handler: QueryHandler,
    /// Admin clients (shell text commands)
    admin_clients: AdminClients,
    /// Recently spawned dynamic PIDs (workaround for spawn-before-slot-setup race)
    /// Maps PID -> service_idx for dynamic drivers that haven't connected yet
    recent_dynamic_pids: [(u32, u8); MAX_RECENT_DYNAMIC_PIDS],
    /// Spawn context: maps child PID to trigger port (for GET_SPAWN_CONTEXT)
    spawn_contexts: [SpawnContext; MAX_SPAWN_CONTEXTS],
    /// In-flight spawn commands: maps seq_id to port info
    inflight_spawns: [InflightSpawn; MAX_INFLIGHT_SPAWNS],
    /// Pending info queries: maps forwarded seq_id to client info
    pending_info_queries: [PendingInfoQuery; MAX_PENDING_INFO_QUERIES],
    /// Next sequence ID for forwarded queries
    next_forward_seq_id: u32,
    /// Log ring buffer
    log_buffer: LogBuffer,
    /// Whether live logging is enabled (print to console as received)
    live_logging: bool,
    // =========================================================================
    // Orchestration State (state machines track pending operations)
    // =========================================================================
    /// Tracked disks - each disk has its own DiskState state machine
    tracked_disks: [TrackedDisk; MAX_TRACKED_DISKS],
    /// Next partition name counter (part0, part1, ...)
    next_partition_num: u32,
    /// Service index for partd (0xFF = not known yet)
    partd_service_idx: u8,
    /// Orchestration sequence ID counter
    orch_seq_id: u32,
    /// Next link ID for port↔service pairing (0 = sentinel, starts at 1)
    next_link_id: u32,
}

impl Devd {
    pub const fn new() -> Self {
        Self {
            query_port: None,
            events: None,
            services: ServiceRegistry::new(),
            ports: Ports::new(),
            process_mgr: SyscallProcessManager::new(),
            devices: DeviceRegistry::new(),
            query_handler: QueryHandler::new(),
            admin_clients: AdminClients::new(),
            recent_dynamic_pids: [(0, 0); MAX_RECENT_DYNAMIC_PIDS],
            spawn_contexts: [const { SpawnContext::empty() }; MAX_SPAWN_CONTEXTS],
            inflight_spawns: [const { InflightSpawn::empty() }; MAX_INFLIGHT_SPAWNS],
            pending_info_queries: [const { PendingInfoQuery::empty() }; MAX_PENDING_INFO_QUERIES],
            next_forward_seq_id: 1,
            log_buffer: LogBuffer::new(),
            live_logging: false, // Off during boot, enable after all services ready
            // Orchestration state (disk/partition state machines)
            tracked_disks: [const { TrackedDisk::empty() }; MAX_TRACKED_DISKS],
            next_partition_num: 0,
            partd_service_idx: 0xFF,
            orch_seq_id: 1,
            next_link_id: 1,
        }
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    fn now_ms() -> u64 {
        syscall::gettime() / 1_000_000
    }

    /// Allocate a unique link ID for port↔service pairing.
    fn alloc_link_id(&mut self) -> u32 {
        let id = self.next_link_id;
        self.next_link_id = self.next_link_id.wrapping_add(1).max(1);
        id
    }

    /// Get service name by index, for log messages
    fn svc_name(&self, idx: u8) -> &str {
        self.services.get(idx as usize).map(|s| s.name()).unwrap_or("?")
    }

    // =========================================================================
    // Initialization
    // =========================================================================

    pub fn init(&mut self) -> SysResult<()> {
        let mut events = EventLoop::new()?;

        let query_port = Port::register(b"devd-query:")?;
        events.watch(query_port.handle())?;
        uinfo!("devd", "query_port_init"; handle = query_port.handle().raw());

        self.query_port = Some(query_port);
        self.events = Some(events);

        // Register devd's service port in the port registry
        let devd_info = abi::PortInfo::new(b"devd-query:", abi::PortClass::Service);
        let _ = self.ports.register_with_port_info(&devd_info, 0xFF, 0);

        // Discover kernel buses — rules fire on Ready, spawning bus drivers
        self.discover_kernel_buses();

        // Spawn boot services that don't have a port trigger
        self.create_and_spawn("vfsd", userlib::devd::caps::DRIVER, &[], 0);
        #[cfg(feature = "stress-test")]
        self.create_and_spawn("testr", u64::MAX, &[], 0);

        Ok(())
    }

    // =========================================================================
    // Bus Discovery
    // =========================================================================

    /// Query kernel for registered buses, register as ports, and let rules spawn drivers.
    ///
    /// Each kernel bus is mirrored as a port in devd's port registry.
    /// The port transitions to Ready, which triggers the unified PORT_RULES
    /// to spawn the appropriate bus driver.
    fn discover_kernel_buses(&mut self) {
        let mut buses = [abi::BusInfo::empty(); 16];
        let count = match userlib::ipc::bus_list(&mut buses) {
            Ok(n) => n,
            Err(e) => {
                uerror!("devd", "bus_list_failed"; err = e.to_errno());
                return;
            }
        };

        uinfo!("devd", "bus_discovery"; count = count as u32);

        for i in 0..count {
            let bus = &buses[i];
            let path = &bus.path[..bus.path_len as usize];

            // Map bus_type → PortClass
            let (port_class, port_subclass) = match bus.bus_type {
                abi::bus_type::PCIE     => (abi::PortClass::Pcie, 0u16),
                abi::bus_type::USB      => (abi::PortClass::Usb, abi::port_subclass::USB_XHCI),
                abi::bus_type::ETHERNET => (abi::PortClass::Ethernet, 0),
                abi::bus_type::UART     => (abi::PortClass::Uart, 0),
                abi::bus_type::KLOG     => (abi::PortClass::Klog, 0),
                _ => continue,  // No PortClass for this bus type (e.g., Platform)
            };

            // Build PortInfo from BusInfo
            let mut port_info = abi::PortInfo::new(path, port_class);
            port_info.port_subclass = port_subclass;

            // Register in devd's port registry (owner = 0xFF = devd/kernel)
            let _ = self.ports.register_with_port_info(&port_info, 0xFF, 0);

            uinfo!("devd", "bus_port_registered";
                name = core::str::from_utf8(path).unwrap_or("?"),
                class = port_class as u16
            );

            // Set port to Claimed — kernel bus ports are managed by the kernel,
            // so they're immediately available. Fire rules on the transition.
            let _ = self.ports.set_state(path, abi::PortState::Claimed);

            // Fire rules on the Claimed transition
            self.check_class_rules(&port_info, 0xFF);
        }
    }

    /// Spawn a bus driver directly (for kernel bus ports where owner=0xFF).
    ///
    /// Unlike driver-owned ports where SpawnChild goes via the owner's channel,
    /// kernel bus ports are owned by devd itself, so we spawn directly.
    /// Spawn (or respawn) a service from its service slot.
    ///
    /// The slot must already exist with name, caps, and trigger_port set.
    /// This is the single spawn path — used by rule matching, restart timers,
    /// and admin commands alike.
    fn spawn_service(&mut self, idx: usize) {
        if idx >= MAX_SERVICES {
            return;
        }

        let now = Self::now_ms();

        // Extract spawn info from service slot
        let (name_buf, name_len, caps, trigger_port_buf, trigger_port_len) = {
            let service = match self.services.get(idx) {
                Some(s) => s,
                None => return,
            };
            let name = service.name();
            let mut nb = [0u8; 16];
            let nl = name.len().min(16);
            nb[..nl].copy_from_slice(&name.as_bytes()[..nl]);

            let mut tp = [0u8; 32];
            let tpl = service.trigger_port_len as usize;
            tp[..tpl].copy_from_slice(&service.trigger_port[..tpl]);

            (nb, nl, service.caps, tp, tpl)
        };

        let binary = core::str::from_utf8(&name_buf[..name_len]).unwrap_or("???");

        // Spawn the process
        let (pid, watcher) = match self.process_mgr.spawn_with_caps(binary, caps) {
            Ok(result) => result,
            Err(_) => {
                uerror!("devd", "svc_spawn_failed"; binary = binary);
                self.services.transition(idx, ServiceState::Failed { code: -1 }, now);
                return;
            }
        };

        // Store spawn context — always, even for services with no trigger port.
        // Bus-framework drivers query GET_SPAWN_CONTEXT on startup; without an
        // entry they crash.
        let trigger_port = &trigger_port_buf[..trigger_port_len];
        let (port_type, metadata_buf, metadata_len) = if trigger_port_len > 0 {
            let pt = self.ports.get_port_type(trigger_port).unwrap_or(0);
            let md: ([u8; 64], usize) = self.ports.get(trigger_port)
                .map(|p| {
                    let m = p.metadata();
                    let mut buf = [0u8; 64];
                    let len = m.len().min(64);
                    buf[..len].copy_from_slice(&m[..len]);
                    (buf, len)
                })
                .unwrap_or(([0u8; 64], 0));
            (pt, md.0, md.1)
        } else {
            (0u8, [0u8; 64], 0usize)
        };
        self.store_spawn_context(pid, port_type, trigger_port, &metadata_buf[..metadata_len]);

        // Watch the process
        if let Some(events) = &mut self.events {
            let _ = events.watch(watcher.handle());
        }

        // Update service state
        if let Some(service) = self.services.get_mut(idx) {
            service.pid = pid;
            service.watcher = Some(watcher);
            service.state = ServiceState::Starting;
            service.last_change = now;
        }

        self.add_recent_dynamic_pid(pid, idx as u8);

        uinfo!("devd", "svc_spawned"; binary = binary, pid = pid);
    }

    /// Create a service slot and spawn.  Used by check_class_rules for port-triggered
    /// drivers and by init for boot services.
    fn create_and_spawn(&mut self, binary: &str, caps: u64, trigger_port: &[u8], link_id: u32) {
        let now = Self::now_ms();
        let idx = match self.services.create_dynamic_service_with_state(
            0, binary.as_bytes(), now, ServiceState::Pending,
        ) {
            Some(i) => i,
            None => {
                uerror!("devd", "no_service_slot"; binary = binary);
                return;
            }
        };

        if let Some(service) = self.services.get_mut(idx) {
            service.caps = caps;
            service.link_id = link_id;
            if !trigger_port.is_empty() {
                service.set_trigger_port(trigger_port);
            }
        }

        self.spawn_service(idx);
    }

    // =========================================================================
    // Service State Changes
    // =========================================================================

    fn handle_service_exit(&mut self, idx: usize, code: i32) {
        if idx >= MAX_SERVICES {
            return;
        }

        let now = Self::now_ms();

        #[derive(Clone, Copy)]
        enum ExitAction {
            Stopped,
            Crashed { backoff_ms: u32 },
            Failed,
        }

        // Update service state and collect info
        let (action, old_pid, svc_idx_u8) = {
            let service = match self.services.get_mut(idx) {
                Some(s) => s,
                None => return,
            };

            let mut name_buf = [0u8; 16];
            let name_str = service.name();
            let name_len = name_str.len().min(16);
            name_buf[..name_len].copy_from_slice(&name_str.as_bytes()[..name_len]);

            let old_pid = service.pid;

            // Clear PID - the process has exited
            service.pid = 0;

            // Remove watcher from event loop
            if let Some(watcher) = service.watcher.take() {
                if let Some(events) = &mut self.events {
                    let _ = events.unwatch(watcher.handle());
                }
            }

            // Remove service channel from event loop
            if let Some(channel) = service.channel.take() {
                if let Some(events) = &mut self.events {
                    let _ = events.unwatch(channel.handle());
                }
            }

            if code == 0 {
                service.state = ServiceState::Stopped { code: 0 };
                service.last_change = now;
                (ExitAction::Stopped, old_pid, idx as u8)
            } else {
                service.total_restarts = service.total_restarts.saturating_add(1);

                let restarts = match service.state {
                    ServiceState::Crashed { restarts, .. } => restarts + 1,
                    _ => 1,
                };

                if restarts >= MAX_RESTARTS {
                    service.state = ServiceState::Failed { code };
                    service.last_change = now;
                    let name = core::str::from_utf8(&name_buf[..name_len]).unwrap_or("?");
                    uerror!("devd", "svc_failed"; name = name, code = code as i32, restarts = service.total_restarts as u32);
                    (ExitAction::Failed, old_pid, idx as u8)
                } else {
                    service.state = ServiceState::Crashed { code, restarts };
                    service.last_change = now;
                    service.backoff_ms = (service.backoff_ms * 2).min(MAX_BACKOFF_MS);
                    (ExitAction::Crashed { backoff_ms: service.backoff_ms }, old_pid, idx as u8)
                }
            }
        };

        // Clean up recent_dynamic_pids tracking for this PID
        if old_pid != 0 {
            self.remove_recent_dynamic_pid(old_pid);
            self.remove_spawn_context(old_pid);
        }

        // Unregister ports owned by this service (scan port registry)
        // Collect child_link_ids before unregistering (for orphan killing)
        let mut ports_to_remove: [Option<[u8; 32]>; 8] = [None; 8];
        let mut removed_link_ids: [u32; 8] = [0; 8];
        let mut remove_count = 0;
        self.ports.for_each(|port| {
            if port.owner() == svc_idx_u8 && remove_count < 8 {
                let mut name = [0u8; 32];
                let n = port.name();
                let len = n.len().min(32);
                name[..len].copy_from_slice(&n[..len]);
                ports_to_remove[remove_count] = Some(name);
                removed_link_ids[remove_count] = port.child_link_id();
                remove_count += 1;
            }
        });
        for name in ports_to_remove.iter().flatten() {
            let len = name.iter().position(|&b| b == 0).unwrap_or(32);
            self.ports.unregister(&name[..len]);
        }

        // If partd exited, reset orchestration state so new instance re-attaches
        if svc_idx_u8 == self.partd_service_idx {
            self.partd_service_idx = 0xFF;
            for disk in self.tracked_disks.iter_mut() {
                if disk.state != DiskState::Empty && disk.state != DiskState::Discovered {
                    disk.state = DiskState::Discovered;
                    disk.partition_count = 0;
                    for p in disk.partitions.iter_mut() {
                        *p = TrackedPartition::empty();
                    }
                }
            }
            uinfo!("devd", "partd_reset"; disks_rediscovered = true);
        }

        // Kill child services whose link_id matches a removed port's child_link_id.
        // Example: consoled owns "console:" port with child_link_id=5 → shell has link_id=5.
        // When consoled dies, shell must be killed so a new one can be spawned.
        if remove_count > 0 {
            let mut children_to_kill: [(u32, usize); 8] = [(0, 0); 8];
            let mut kill_count = 0;
            self.services.for_each(|child_idx, child_svc| {
                if child_svc.link_id == 0 || child_idx == idx || !child_svc.state.is_running() || child_svc.pid == 0 {
                    return;
                }
                for i in 0..remove_count {
                    if child_svc.link_id == removed_link_ids[i] && removed_link_ids[i] != 0 && kill_count < 8 {
                        children_to_kill[kill_count] = (child_svc.pid, child_idx);
                        kill_count += 1;
                        break;
                    }
                }
            });
            for i in 0..kill_count {
                let (pid, child_idx) = children_to_kill[i];
                if let Some(child_svc) = self.services.get(child_idx) {
                    uinfo!("devd", "kill_orphaned_child";
                        name = child_svc.name(), pid = pid);
                }
                let _ = self.process_mgr.kill(pid);
                // Exit watcher will fire and handle cleanup via handle_service_exit
            }
        }

        // Create restart timer if needed
        match action {
            ExitAction::Crashed { backoff_ms } => {
                if let Ok(mut timer) = Timer::new() {
                    let deadline_ns = (backoff_ms as u64) * 1_000_000;
                    if timer.set(deadline_ns).is_ok() {
                        if let Some(events) = &mut self.events {
                            let _ = events.watch(timer.handle());
                        }
                        if let Some(service) = self.services.get_mut(idx) {
                            service.restart_timer = Some(timer);
                        }
                    }
                }
            }
            ExitAction::Failed => {
                if let Ok(mut timer) = Timer::new() {
                    let deadline_ns = FAILED_RETRY_MS * 1_000_000;
                    if timer.set(deadline_ns).is_ok() {
                        if let Some(events) = &mut self.events {
                            let _ = events.watch(timer.handle());
                        }
                        if let Some(service) = self.services.get_mut(idx) {
                            service.restart_timer = Some(timer);
                        }
                    }
                }
            }
            ExitAction::Stopped => {}
        }
    }

    // =========================================================================
    // Admin Client Handling (shell text commands)
    // =========================================================================

    fn handle_admin_client_event(&mut self, handle: ObjHandle) {
        let slot = match self.admin_clients.find_by_handle(handle) {
            Some(s) => s,
            None => return,
        };

        let mut buf = [0u8; 128];
        let len = {
            let client = match self.admin_clients.get_mut(slot) {
                Some(c) => c,
                None => return,
            };
            match client.channel.recv(&mut buf) {
                Ok(n) if n > 0 => n,
                Ok(_) => {
                    // EOF - client disconnected
                    self.remove_admin_client(slot);
                    return;
                }
                Err(SysError::WouldBlock) => return,
                Err(_) => {
                    self.remove_admin_client(slot);
                    return;
                }
            }
        };

        // Parse and handle text command
        let mut resp_buf = [0u8; 512];
        let resp_len = self.handle_admin_command(&buf[..len], &mut resp_buf);

        // Send response
        if resp_len > 0 {
            if let Some(client) = self.admin_clients.get_mut(slot) {
                let _ = client.channel.send(&resp_buf[..resp_len]);
            }
        }
    }

    fn handle_admin_command(&mut self, cmd: &[u8], resp: &mut [u8]) -> usize {
        // Trim whitespace and newline
        let cmd = trim_bytes(cmd);

        let static_resp = if cmd.starts_with(b"START ") {
            let name = trim_bytes(&cmd[6..]);
            self.admin_start_service(name)
        } else if cmd.starts_with(b"STOP ") {
            let name = trim_bytes(&cmd[5..]);
            self.admin_stop_service(name)
        } else if cmd.starts_with(b"RESTART ") {
            let name = trim_bytes(&cmd[8..]);
            self.admin_restart_service(name)
        } else if cmd == b"LIST" {
            self.admin_list_services()
        } else if cmd.starts_with(b"SPAWN ") {
            let args = trim_bytes(&cmd[6..]);
            self.admin_spawn_driver(args)
        } else if cmd.starts_with(b"CONFIG ") {
            let args = trim_bytes(&cmd[7..]);
            return self.admin_config_command(args, resp);
        } else {
            b"ERR unknown command\n"
        };

        let len = static_resp.len().min(resp.len());
        resp[..len].copy_from_slice(&static_resp[..len]);
        len
    }

    fn admin_start_service(&mut self, name: &[u8]) -> &'static [u8] {
        let name_str = match core::str::from_utf8(name) {
            Ok(s) => s,
            Err(_) => return b"ERR invalid name\n",
        };

        // Find service by name
        let idx = match self.services.find_by_name(name_str) {
            Some(i) => i,
            None => return b"ERR service not found\n",
        };

        // Check if already running
        if let Some(service) = self.services.get(idx) {
            if service.state.is_running() {
                return b"ERR already running\n";
            }
        }

        self.spawn_service(idx);
        b"OK\n"
    }

    fn admin_stop_service(&mut self, name: &[u8]) -> &'static [u8] {
        let name_str = match core::str::from_utf8(name) {
            Ok(s) => s,
            Err(_) => return b"ERR invalid name\n",
        };

        let idx = match self.services.find_by_name(name_str) {
            Some(i) => i,
            None => return b"ERR service not found\n",
        };

        // Check if running
        let pid = self.services.get(idx).map(|s| s.pid).unwrap_or(0);
        if pid == 0 {
            return b"ERR not running\n";
        }

        // Kill the process — kernel cascading kill handles children
        let _ = self.process_mgr.kill(pid);
        // The process exit watcher will fire and handle cleanup
        b"OK\n"
    }

    fn admin_restart_service(&mut self, name: &[u8]) -> &'static [u8] {
        let name_str = match core::str::from_utf8(name) {
            Ok(s) => s,
            Err(_) => return b"ERR invalid name\n",
        };

        let idx = match self.services.find_by_name(name_str) {
            Some(i) => i,
            None => return b"ERR service not found\n",
        };

        // Kill if running — exit handler will fire, then we respawn
        let pid = self.services.get(idx).map(|s| s.pid).unwrap_or(0);
        if pid != 0 {
            let _ = self.process_mgr.kill(pid);
            // Exit handler will transition state. For immediate restart, set a short timer.
        }

        // Reset state and respawn
        let now = Self::now_ms();
        if let Some(service) = self.services.get_mut(idx) {
            service.state = ServiceState::Pending;
            service.last_change = now;
        }

        self.spawn_service(idx);
        b"OK\n"
    }

    fn admin_list_services(&self) -> &'static [u8] {
        // For now, just return OK
        // A proper implementation would build a list string, but that requires allocation
        // The shell can use devd-query for detailed listing
        b"OK\n"
    }

    fn admin_spawn_driver(&mut self, args: &[u8]) -> &'static [u8] {
        // Parse: <binary> <port>
        let args_str = match core::str::from_utf8(args) {
            Ok(s) => s,
            Err(_) => return b"ERR invalid args\n",
        };

        let mut parts = args_str.split_whitespace();
        let binary = match parts.next() {
            Some(b) => b,
            None => return b"ERR missing binary\n",
        };
        let port = parts.next().map(|p| p.as_bytes());

        // Spawn the driver dynamically
        self.spawn_dynamic_driver(binary, port.unwrap_or(b""));
        b"OK\n"
    }

    /// Handle CONFIG admin command: forward to driver, wait for response.
    ///
    /// Format: `<service> GET [key]` or `<service> SET <key> <value>`
    /// Returns number of bytes written to resp buffer.
    fn admin_config_command(&mut self, args: &[u8], resp: &mut [u8]) -> usize {
        let args_str = match core::str::from_utf8(args) {
            Ok(s) => s,
            Err(_) => return copy_static(resp, b"ERR invalid args\n"),
        };

        let mut parts = args_str.splitn(4, ' ');
        let service_name = match parts.next() {
            Some(s) if !s.is_empty() => s,
            _ => return copy_static(resp, b"ERR missing service name\n"),
        };
        let operation = match parts.next() {
            Some(s) => s,
            None => return copy_static(resp, b"ERR missing operation (GET/SET)\n"),
        };

        // Find service by name
        let service_idx = match self.services.find_by_name(service_name) {
            Some(i) => i,
            None => return copy_static(resp, b"ERR service not found\n"),
        };

        // Check service is running
        if let Some(service) = self.services.get(service_idx) {
            if !service.state.is_running() {
                return copy_static(resp, b"ERR service not running\n");
            }
        }

        // Find the driver's query client slot
        let driver_slot = match self.query_handler.find_by_service_idx(service_idx as u8) {
            Some(slot) => slot,
            None => return copy_static(resp, b"ERR driver not connected\n"),
        };

        // Build and send the query message
        let seq_id = self.next_forward_seq_id;
        self.next_forward_seq_id = self.next_forward_seq_id.wrapping_add(1);
        if self.next_forward_seq_id == 0 {
            self.next_forward_seq_id = 1;
        }

        let sent = match operation {
            "GET" | "get" => {
                let key = parts.next().unwrap_or("");
                self.send_config_query(driver_slot, seq_id, msg::CONFIG_GET, key.as_bytes(), &[])
            }
            "SET" | "set" => {
                let key = match parts.next() {
                    Some(k) => k,
                    None => return copy_static(resp, b"ERR missing key\n"),
                };
                let value = match parts.next() {
                    Some(v) => v,
                    None => return copy_static(resp, b"ERR missing value\n"),
                };
                self.send_config_query(driver_slot, seq_id, msg::CONFIG_SET, key.as_bytes(), value.as_bytes())
            }
            _ => return copy_static(resp, b"ERR unknown operation (use GET or SET)\n"),
        };

        if !sent {
            return copy_static(resp, b"ERR send failed\n");
        }

        // Wait for response (blocking recv on driver channel)
        let mut recv_buf = [0u8; 1100];
        let recv_len = match self.query_handler.get_mut(driver_slot) {
            Some(client) => {
                // Use blocking recv — driver responds synchronously
                match client.channel.recv(&mut recv_buf) {
                    Ok(n) => n,
                    Err(_) => return copy_static(resp, b"ERR recv failed\n"),
                }
            }
            None => return copy_static(resp, b"ERR driver disconnected\n"),
        };

        // Parse ServiceInfoResult response
        use userlib::query::{ServiceInfoResult, error};
        if let Some((result, info_bytes)) = ServiceInfoResult::from_bytes(&recv_buf[..recv_len]) {
            if result.result == error::OK && !info_bytes.is_empty() {
                let len = info_bytes.len().min(resp.len());
                resp[..len].copy_from_slice(&info_bytes[..len]);
                len
            } else {
                copy_static(resp, b"ERR driver returned error\n")
            }
        } else {
            copy_static(resp, b"ERR invalid response\n")
        }
    }

    /// Send a CONFIG_GET or CONFIG_SET query to a driver via its query channel.
    fn send_config_query(&mut self, driver_slot: usize, seq_id: u32, msg_type: u16, key: &[u8], value: &[u8]) -> bool {
        let mut buf = [0u8; 256];

        // Build QueryHeader
        let header = QueryHeader::new(msg_type, seq_id);
        let header_bytes = header.to_bytes();
        buf[..QueryHeader::SIZE].copy_from_slice(&header_bytes);

        let mut offset = QueryHeader::SIZE;

        // Write key
        let key_len = key.len().min(buf.len() - offset - 2);
        buf[offset..offset + key_len].copy_from_slice(&key[..key_len]);
        offset += key_len;

        if msg_type == msg::CONFIG_SET {
            // Null separator between key and value
            buf[offset] = 0;
            offset += 1;
            let value_len = value.len().min(buf.len() - offset - 1);
            buf[offset..offset + value_len].copy_from_slice(&value[..value_len]);
            offset += value_len;
        }

        if let Some(client) = self.query_handler.get_mut(driver_slot) {
            client.channel.send(&buf[..offset]).is_ok()
        } else {
            false
        }
    }

    fn remove_admin_client(&mut self, slot: usize) {
        if let Some(channel) = self.admin_clients.remove(slot) {
            if let Some(events) = &mut self.events {
                let _ = events.unwatch(channel.handle());
            }
        }
    }

    fn handle_process_exit(&mut self, handle: ObjHandle) {
        let found_idx = self.services.find_by_watcher(handle);

        if let Some(idx) = found_idx {
            let code = self.services.get_mut(idx)
                .and_then(|s| s.watcher.as_mut())
                .map(|w| w.wait().unwrap_or(-1))
                .unwrap_or(-1);
            self.handle_service_exit(idx, code);
        }
    }

    fn handle_restart_timer(&mut self, idx: usize) {
        if idx >= MAX_SERVICES {
            return;
        }

        let now = Self::now_ms();
        // Clear timer and determine action
        let should_spawn = {
            let service = match self.services.get_mut(idx) {
                Some(s) => s,
                None => return,
            };

            // Clear the timer
            if let Some(timer) = service.restart_timer.take() {
                if let Some(events) = &mut self.events {
                    let _ = events.unwatch(timer.handle());
                }
            }

            match service.state {
                ServiceState::Crashed { .. } => true,
                ServiceState::Failed { .. } => {
                    // Reset for retry
                    service.backoff_ms = INITIAL_BACKOFF_MS;
                    service.last_change = now;
                    true
                }
                _ => false,
            }
        };

        if should_spawn {
            self.spawn_service(idx);
        }
    }

    /// Poll restart timers using wall-clock comparison.
    ///
    /// Fallback for timers that couldn't be added to the Mux (16-handle limit).
    /// Called on every event loop iteration (100ms timeout). Timers that ARE
    /// in the Mux fire immediately via Mux events; this catches the rest.
    fn poll_restart_timers(&mut self) {
        let now = Self::now_ms();
        let mut ready = [0u16; MAX_SERVICES];
        let mut count = 0;
        self.services.for_each(|idx, svc| {
            if svc.restart_timer.is_some() && count < MAX_SERVICES {
                let deadline = svc.last_change + svc.backoff_ms as u64;
                if now >= deadline {
                    ready[count] = idx as u16;
                    count += 1;
                }
            }
        });
        for i in 0..count {
            self.handle_restart_timer(ready[i] as usize);
        }
    }

    // =========================================================================
    // Query Event Handling
    // =========================================================================

    fn handle_query_port_event(&mut self) {
        let query_port = match &mut self.query_port {
            Some(p) => p,
            None => return,
        };

        match query_port.accept_with_pid() {
            Ok((channel, client_pid)) => {
                uinfo!("devd", "query_accept"; pid = client_pid);
                // Check if this is a known service (driver)
                // First try the normal service registry lookup for Ready services
                let mut service_idx = self.services.find_by_pid(client_pid)
                    .filter(|&i| {
                        self.services.get(i)
                            .map(|s| s.state == ServiceState::Ready)
                            .unwrap_or(false)
                    })
                    .map(|i| i as u8);

                // Also check for Starting services - they can connect to register
                // ports before reporting Ready. Don't auto-transition; wait for
                // explicit STATE_CHANGE(Ready) from the driver.
                if service_idx.is_none() {
                    if let Some(idx) = self.services.find_by_pid(client_pid) {
                        let is_starting = self.services.get(idx)
                            .map(|s| s.state == ServiceState::Starting)
                            .unwrap_or(false);
                        if is_starting {
                            // Recognize as driver but keep in Starting state
                            // Driver must send STATE_CHANGE(Ready) when actually ready
                            service_idx = Some(idx as u8);
                        }
                    }
                }

                // If not found, check recent_dynamic_pids for race condition workaround
                // This handles the case where the child connects before the parent
                // finishes setting up the service slot
                if service_idx.is_none() {
                    if let Some(idx) = self.find_recent_dynamic_pid(client_pid) {
                        // Trust the recent_dynamic_pids entry - it was added immediately
                        // after spawn, before the child could have connected
                        service_idx = Some(idx);
                        // Don't remove yet - keep tracking until slot is fully set up
                        // The entry will be cleaned up on next lookup or service exit
                    }
                }

                // Unknown PID — treat as admin client (shell)
                if service_idx.is_none() {
                    uinfo!("devd", "admin_accept"; pid = client_pid);
                    if let Some(events) = &mut self.events {
                        let _ = events.watch(channel.handle());
                    }
                    let _ = self.admin_clients.add(channel);
                    return;
                }

                // Save handle before moving channel into query handler
                let ch_handle = channel.handle();

                // Log the service_idx that will be used
                if let Some(idx) = service_idx {
                    if let Some(svc) = self.services.get(idx as usize) {
                        uinfo!("devd", "query_driver"; pid = client_pid, idx = idx as u32, name = svc.name());
                    }
                }

                // Add to query handler
                match self.query_handler.add_client(channel, service_idx, client_pid) {
                    Some(slot) => {
                        // Check if this is partd for orchestration
                        if let Some(idx) = service_idx {
                            self.check_orchestration_driver_connected(idx);
                        }

                        // Try immediate non-blocking read — clients often send
                        // their request before we accept, so the message is
                        // already buffered in the channel.  Process it now so
                        // we don't depend on fitting the handle into the Mux.
                        self.try_immediate_query_read(slot);

                        // Add to event loop for future messages (best-effort —
                        // may fail if Mux is full, but clients use non-blocking
                        // recv with polling to avoid deadlock in that case)
                        if let Some(events) = &mut self.events {
                            let _ = events.watch(ch_handle);
                        }
                    }
                    None => {
                        uerror!("devd", "query_full"; pid = client_pid);
                    }
                }
            }
            Err(SysError::WouldBlock) => {}
            Err(_e) => {
                uerror!("devd", "query_accept_failed";);
            }
        }
    }

    /// Try a non-blocking read from a newly accepted query client.
    ///
    /// Clients typically send their request before we accept, so the
    /// message is already buffered.  Processing it here avoids depending
    /// on fitting the channel handle into the Mux (which has limited slots).
    fn try_immediate_query_read(&mut self, slot: usize) {
        let client = match self.query_handler.get_mut(slot) {
            Some(c) => c,
            None => return,
        };

        let mut recv_buf = [0u8; MSG_BUFFER_SIZE];
        match client.channel.try_recv(&mut recv_buf) {
            Ok(Some(len)) if len > 0 => {
                self.dispatch_query_message(slot, &recv_buf[..len]);
            }
            _ => {} // No data yet — EventLoop will handle it later
        }
    }

    fn handle_query_client_event(&mut self, handle: ObjHandle) {
        let slot = match self.query_handler.find_by_handle(handle) {
            Some(s) => s,
            None => return,
        };
        self.try_read_query_slot(slot);
    }

    /// Non-blocking read from a single query slot.  Shared by Mux-driven
    /// handle_query_client_event and the periodic poll_query_clients sweep.
    fn try_read_query_slot(&mut self, slot: usize) {
        let client = match self.query_handler.get_mut(slot) {
            Some(c) => c,
            None => return,
        };

        let mut recv_buf = [0u8; MSG_BUFFER_SIZE];
        match client.channel.try_recv(&mut recv_buf) {
            Ok(Some(len)) if len > 0 => {
                self.dispatch_query_message(slot, &recv_buf[..len]);
            }
            Ok(Some(_)) | Ok(None) => {
                // No data yet — will be picked up next poll or Mux event
            }
            Err(SysError::PeerClosed) | Err(SysError::ConnectionReset) => {
                self.remove_query_client(slot);
            }
            Err(_e) => {
                uerror!("devd", "query_recv_failed"; slot = slot as u32);
                self.remove_query_client(slot);
            }
        }
    }

    /// Poll all active query clients for pending messages.
    ///
    /// This handles channels that could not be watched in the Mux due to
    /// the 16-handle limit.  Called on every event loop iteration (including
    /// timeouts) so messages are never stuck for long.
    fn poll_query_clients(&mut self) {
        if self.query_handler.active_count() == 0 {
            return;
        }
        for slot in 0..MAX_QUERY_CLIENTS {
            if self.query_handler.get(slot).is_some() {
                self.try_read_query_slot(slot);
            }
        }
    }

    /// Dispatch a query message by type.  Shared by handle_query_client_event
    /// (Mux-driven) and try_immediate_query_read (accept-driven).
    fn dispatch_query_message(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{msg, QueryHeader};

        let msg_type = QueryHeader::from_bytes(buf).map(|h| h.msg_type);

        match msg_type {
            Some(msg::REGISTER_PORT_INFO) => {
                self.handle_port_register_info_msg(slot, buf);
            }
            Some(msg::STATE_CHANGE) => {
                self.handle_state_change_msg(slot, buf);
            }
            Some(msg::SPAWN_ACK) => {
                self.handle_spawn_ack_msg(slot, buf);
            }
            Some(msg::QUERY_DRIVER) => {
                self.handle_query_driver_forward(slot, buf);
            }
            Some(msg::GET_SPAWN_CONTEXT) => {
                self.handle_get_spawn_context(slot, buf);
            }
            Some(msg::QUERY_PORT) => {
                self.handle_query_port(slot, buf);
            }
            Some(msg::UPDATE_PORT_SHMEM_ID) => {
                self.handle_update_port_shmem_id(slot, buf);
            }
            Some(msg::SET_PORT_STATE) => {
                self.handle_set_port_state(slot, buf);
            }
            Some(msg::LIST_PORTS) => {
                self.handle_list_ports(slot, buf);
            }
            Some(msg::LIST_SERVICES) => {
                self.handle_list_services(slot, buf);
            }
            Some(msg::QUERY_SERVICE_INFO) => {
                self.handle_query_service_info(slot, buf);
            }
            Some(msg::SERVICE_INFO_RESULT) => {
                self.handle_service_info_result(slot, buf);
            }
            Some(msg::LOG_MESSAGE) => {
                self.handle_log_message(slot, buf);
            }
            Some(msg::LOG_QUERY) => {
                self.handle_log_query(slot, buf);
            }
            Some(msg::LOG_CONTROL) => {
                self.handle_log_control(slot, buf);
            }
            Some(msg::REPORT_PARTITIONS) => {
                self.handle_report_partitions(slot, buf);
            }
            Some(msg::PARTITION_READY) => {
                self.handle_partition_ready(slot, buf);
            }
            _ => {
                let mut response_buf = [0u8; MSG_BUFFER_SIZE];
                let response_len = self.query_handler.handle_message(
                    slot,
                    buf,
                    &mut self.devices,
                    &self.services,
                    &mut response_buf,
                );

                if let Some(resp_len) = response_len {
                    if let Some(client) = self.query_handler.get_mut(slot) {
                        let _ = client.channel.send(&response_buf[..resp_len]);
                    }
                }
            }
        }
    }

    /// Handle REGISTER_PORT_INFO message (unified PortInfo registration)
    fn handle_port_register_info_msg(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::error;

        // Parse the registration message
        let info = match self.query_handler.parse_port_register_info(slot, buf) {
            Some(i) => i,
            None => {
                // Permission denied or invalid format
                if let Some(header) = userlib::query::QueryHeader::from_bytes(buf) {
                    self.query_handler.send_port_register_response(
                        slot, header.seq_id, error::PERMISSION_DENIED
                    );
                }
                return;
            }
        };

        // Register the port with unified PortInfo
        let result = self.handle_port_registration(
            &info.port_info,
            info.owner_idx,
            info.shmem_id,
        );

        // Send response
        let result_code = match result {
            Ok(()) => {
                // Port registered successfully — set to Claimed and fire rules.
                // Port registered but not yet Ready. Rules fire when the
                // driver sends STATE_CHANGE(Ready) — see handle_state_change_msg.
                error::OK
            }
            Err(_e) => {
                if let Ok(name_str) = core::str::from_utf8(info.port_info.name_bytes()) {
                    uerror!("devd", "port_register_failed"; name = name_str, owner = info.owner_idx as u32);
                }
                error::INVALID_REQUEST
            }
        };
        self.query_handler.send_port_register_response(slot, info.seq_id, result_code);
    }

    fn handle_state_change_msg(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{StateChange, driver_state};

        // Parse the state change message
        let state_msg = match StateChange::from_bytes(buf) {
            Some(s) => s,
            None => {
                uerror!("devd", "invalid_state_change"; slot = slot as u32);
                return;
            }
        };

        // Get the driver's service index from the query client
        let driver_idx = match self.query_handler.get_service_idx(slot) {
            Some(idx) => idx,
            None => {
                return;
            }
        };

        uinfo!("devd", "svc_state_change"; name = self.svc_name(driver_idx as u8), state = state_msg.new_state as u32);

        // When driver reports Ready, transition service state and activate ports
        if state_msg.new_state == driver_state::READY {
            let now = Self::now_ms();
            if let Some(service) = self.services.get_mut(driver_idx as usize) {
                service.state = ServiceState::Ready;
                service.last_change = now;
                service.backoff_ms = INITIAL_BACKOFF_MS;
            }

            // Transition all ports owned by this driver to Claimed and fire rules.
            // Ports were registered during reset() but held in Initialize state
            // until the driver reports Ready.
            let owner_idx = driver_idx as u8;
            let mut ready_ports: [([u8; 32], usize); 8] = [([0u8; 32], 0); 8];
            let mut ready_count = 0usize;
            self.ports.for_each(|port| {
                if port.owner() == owner_idx && port.state() != abi::PortState::Claimed && ready_count < 8 {
                    let mut name = [0u8; 32];
                    let n = port.name();
                    let len = n.len().min(32);
                    name[..len].copy_from_slice(&n[..len]);
                    ready_ports[ready_count] = (name, len);
                    ready_count += 1;
                }
            });
            for i in 0..ready_count {
                let (name, len) = &ready_ports[i];
                let port_name = &name[..*len];
                let _ = self.ports.set_state(port_name, abi::PortState::Claimed);
                if let Some(port) = self.ports.get(port_name) {
                    let info = *port.port_info();
                    self.check_class_rules(&info, owner_idx);
                }
            }
        }
    }

    /// Handle GET_SPAWN_CONTEXT message from a child driver
    ///
    /// Returns the port name that triggered this driver's spawn.
    fn handle_get_spawn_context(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{QueryHeader, SpawnContextResponse, error};

        let header = match QueryHeader::from_bytes(buf) {
            Some(h) => h,
            None => {
                uerror!("devd", "invalid_spawn_ctx_msg"; slot = slot as u32);
                return;
            }
        };

        // Get the client's PID
        let client_pid = match self.query_handler.get(slot) {
            Some(client) => client.pid,
            None => {
                uerror!("devd", "unknown_spawn_ctx_slot"; slot = slot as u32);
                return;
            }
        };

        // Look up spawn context for this PID
        let mut response_buf = [0u8; 256];
        let resp_len = if let Some((port_type, port_name, metadata)) = self.get_spawn_context(client_pid) {
            let resp = SpawnContextResponse::new(header.seq_id, error::OK, port_type);
            resp.write_to_with_metadata(&mut response_buf, port_name, metadata)
                .unwrap_or(SpawnContextResponse::HEADER_SIZE)
        } else {
            // No context available - return error
            let resp = SpawnContextResponse::new(header.seq_id, error::NOT_FOUND, 0);
            resp.write_to(&mut response_buf, &[])
                .unwrap_or(SpawnContextResponse::HEADER_SIZE)
        };

        // Send response
        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.channel.send(&response_buf[..resp_len]);
        }
    }

    /// Handle QUERY_PORT message - returns port info including shmem_id
    fn handle_query_port(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{QueryPort, PortInfoResponse, port_flags, error};

        let (query, port_name) = match QueryPort::from_bytes(buf) {
            Some(q) => q,
            None => {
                uerror!("devd", "invalid_query_port"; slot = slot as u32);
                return;
            }
        };

        let seq_id = query.header.seq_id;

        // Look up the port
        let resp = if let Some(port) = self.ports.get(port_name) {
            // Get owner PID from service index
            let owner_pid = self.services.get(port.owner() as usize)
                .map(|s| s.pid)
                .unwrap_or(0);

            // Build flags
            let mut flags = 0u8;
            if port.has_dataport() {
                flags |= port_flags::HAS_DATAPORT;
            }
            if port.is_available() {
                flags |= port_flags::AVAILABLE;
            }

            PortInfoResponse::success(
                seq_id,
                port.port_type(),
                flags,
                port.shmem_id(),
                owner_pid,
            )
        } else {
            PortInfoResponse::new(seq_id, error::NOT_FOUND)
        };

        // Send response
        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.channel.send(&resp.to_bytes());
        }
    }

    /// Handle UPDATE_PORT_SHMEM_ID message - updates shmem_id for existing port
    fn handle_update_port_shmem_id(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{UpdatePortShmemId, PortRegisterResponse, error};

        let (update, port_name) = match UpdatePortShmemId::from_bytes(buf) {
            Some(u) => u,
            None => {
                uerror!("devd", "invalid_update_shmem"; slot = slot as u32);
                return;
            }
        };

        let seq_id = update.header.seq_id;

        // Get port info before updating (for orchestration check)
        let port_class = self.ports.get(port_name).map(|p| p.port_class());
        let owner_idx = self.ports.find_owner(port_name);

        // Update the port's shmem_id
        let result = self.ports.set_shmem_id(port_name, update.shmem_id);

        let result_code = match result {
            Ok(()) => {
                uinfo!("devd", "port_shmem_update"; port = core::str::from_utf8(port_name).unwrap_or("?"), shmem_id = update.shmem_id);
                error::OK
            }
            Err(_) => {
                uerror!("devd", "shmem_update_failed"; port = core::str::from_utf8(port_name).unwrap_or("?"));
                error::NOT_FOUND
            }
        };

        // Send response (reuse PortRegisterResponse format)
        let resp = PortRegisterResponse::new(seq_id, result_code);
        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.channel.send(&resp.to_bytes());
        }

        // Track block device for ATTACH_DISK orchestration
        if result_code == error::OK
            && update.shmem_id != 0
            && port_class == Some(abi::PortClass::Block)
        {
            if let Some(owner) = owner_idx {
                self.track_and_attach_disk(port_name, update.shmem_id, owner);
            }
        }
    }

    /// Handle SET_PORT_STATE message - transitions a port to a new state
    fn handle_set_port_state(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{SetPortState, error};

        // Validate slot index
        if slot >= MAX_QUERY_CLIENTS {
            uerror!("devd", "set_port_state_invalid_slot"; slot = slot as u32);
            return;
        }

        // Validate minimum buffer size
        if buf.len() < SetPortState::FIXED_SIZE {
            uerror!("devd", "set_port_state_buf_too_small"; len = buf.len() as u32);
            return;
        }

        // Parse message
        let (req, port_name) = match SetPortState::from_bytes(buf) {
            Some(m) => m,
            None => {
                uerror!("devd", "invalid_set_port_state"; slot = slot as u32);
                return;
            }
        };

        let seq_id = req.header.seq_id;

        // Validate port name
        if port_name.is_empty() || port_name.len() > 32 {
            uerror!("devd", "set_port_state_invalid_name"; len = port_name.len() as u32);
            self.send_set_port_state_response(slot, seq_id, error::INVALID_REQUEST);
            return;
        }

        // Validate state value
        let new_state = match abi::PortState::from_u8(req.state) {
            Some(s) => s,
            None => {
                uerror!("devd", "invalid_port_state"; state = req.state as u32);
                self.send_set_port_state_response(slot, seq_id, error::INVALID_REQUEST);
                return;
            }
        };

        // Get port info before state change (for rule checking)
        let port_info = self.ports.get(port_name).map(|p| *p.port_info());
        let owner_idx = self.ports.find_owner(port_name);

        // Update the port's state
        let old_state = self.ports.set_state(port_name, new_state);

        let result_code = match old_state {
            Some(old) => {
                uinfo!("devd", "port_state_change";
                    port = core::str::from_utf8(port_name).unwrap_or("?"),
                    from = old as u8,
                    to = new_state as u8
                );
                error::OK
            }
            None => {
                uerror!("devd", "port_not_found"; port = core::str::from_utf8(port_name).unwrap_or("?"));
                error::NOT_FOUND
            }
        };

        // Send response with correct msg_type
        self.send_set_port_state_response(slot, seq_id, result_code);

        // Port went Safe — child service is gone, mark matching service Stopped.
        // If owning driver is still Ready, transition Safe → Claimed and re-fire rules.
        if result_code == error::OK && new_state == abi::PortState::Safe {
            // Mark services bound to this port (by link_id) as Stopped
            let port_link_id = self.ports.get(port_name)
                .map(|p| p.child_link_id())
                .unwrap_or(0);
            if port_link_id != 0 {
                self.services.for_each_mut(|_, svc| {
                    if svc.link_id == port_link_id && svc.state.is_running() {
                        svc.state = service::ServiceState::Stopped { code: 0 };
                    }
                });
            }

            // Driver still alive — re-claim the port and fire spawn rules
            let driver_ready = owner_idx
                .and_then(|oi| self.services.get(oi as usize))
                .map(|s| s.state == service::ServiceState::Ready)
                .unwrap_or(false);
            if driver_ready {
                let _ = self.ports.set_state(port_name, abi::PortState::Claimed);
                if let (Some(info), Some(owner)) = (port_info, owner_idx) {
                    self.check_class_rules(&info, owner);
                }
            }
        }

        // Check rules on transition to Claimed (from external SET_PORT_STATE)
        if result_code == error::OK
            && new_state == abi::PortState::Claimed
            && old_state != Some(abi::PortState::Claimed)
        {
            if let (Some(info), Some(owner)) = (port_info, owner_idx) {
                self.check_class_rules(&info, owner);
            }
        }
    }

    /// Send SET_PORT_STATE response with correct msg_type
    fn send_set_port_state_response(&mut self, slot: usize, seq_id: u32, result: i32) {
        use userlib::query::msg;

        // Build response: header (8 bytes) + result (4 bytes)
        let mut resp = [0u8; 12];
        resp[0..2].copy_from_slice(&msg::SET_PORT_STATE.to_le_bytes());
        resp[2..4].copy_from_slice(&0u16.to_le_bytes()); // flags
        resp[4..8].copy_from_slice(&seq_id.to_le_bytes());
        resp[8..12].copy_from_slice(&result.to_le_bytes());

        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.channel.send(&resp);
        }
    }

    /// Handle LIST_PORTS message - returns all registered ports
    fn handle_list_ports(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{QueryHeader, PortsListResponse, PortEntry, port_flags};

        let header = match QueryHeader::from_bytes(buf) {
            Some(h) => h,
            None => {
                uerror!("devd", "invalid_list_ports"; slot = slot as u32);
                return;
            }
        };

        let seq_id = header.seq_id;

        // Count ports and build entries
        let mut entries = [[0u8; PortEntry::SIZE]; 32];
        let mut count = 0usize;

        self.ports.for_each(|port| {
            if count >= 32 {
                return;
            }

            // Get owner PID
            let owner_pid = if port.owner() == 0xFF {
                0 // devd itself
            } else {
                self.services.get(port.owner() as usize)
                    .map(|s| s.pid)
                    .unwrap_or(0)
            };

            // Build flags
            let mut flags = 0u8;
            if port.has_dataport() {
                flags |= port_flags::HAS_DATAPORT;
            }
            if port.is_available() {
                flags |= port_flags::AVAILABLE;
            }

            // Build entry
            let mut name = [0u8; 20];
            let name_bytes = port.name();
            let name_len = name_bytes.len().min(20);
            name[..name_len].copy_from_slice(&name_bytes[..name_len]);

            let entry = PortEntry {
                name,
                port_type: port.port_type(),
                flags,
                owner_idx: port.owner(),
                parent_idx: port.parent_idx().unwrap_or(0xFF),
                shmem_id: port.shmem_id(),
                owner_pid,
            };

            entries[count] = entry.to_bytes();
            count += 1;
        });

        // Build response
        let resp_header = PortsListResponse::new(seq_id, count as u16);

        // Send response: header + entries
        let mut resp_buf = [0u8; 12 + 32 * 32]; // header + 32 entries max
        let header_bytes = resp_header.header_to_bytes();
        resp_buf[..PortsListResponse::HEADER_SIZE].copy_from_slice(&header_bytes);

        let mut offset = PortsListResponse::HEADER_SIZE;
        for i in 0..count {
            resp_buf[offset..offset + PortEntry::SIZE].copy_from_slice(&entries[i]);
            offset += PortEntry::SIZE;
        }

        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.channel.send(&resp_buf[..offset]);
        }
    }

    /// Handle LIST_SERVICES message - returns all services
    fn handle_list_services(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{QueryHeader, ServicesListResponse, ServiceEntry, service_state};

        let header = match QueryHeader::from_bytes(buf) {
            Some(h) => h,
            None => {
                uerror!("devd", "invalid_list_services"; slot = slot as u32);
                return;
            }
        };

        let seq_id = header.seq_id;

        // First pass: collect service data (to avoid borrow issues with spawn_contexts lookup)
        struct ServiceData {
            name: [u8; 16],
            pid: u32,
            state: u8,
            index: u8,
            parent_idx: u8,
            child_count: u8,
            total_restarts: u32,
            last_change: u32,
        }
        let mut service_data: [Option<ServiceData>; 16] = [const { None }; 16];
        let mut count = 0usize;

        self.services.for_each(|idx, service| {
            if count >= 16 {
                return;
            }

            let mut name = [0u8; 16];
            let name_str = service.name().as_bytes();
            let name_len = name_str.len().min(16);
            name[..name_len].copy_from_slice(&name_str[..name_len]);

            let state = match service.state {
                ServiceState::Pending => service_state::PENDING,
                ServiceState::Starting => service_state::STARTING,
                ServiceState::Ready => service_state::READY,
                ServiceState::Stopped { .. } => service_state::STOPPED,
                ServiceState::Crashed { .. } => service_state::CRASHED,
                ServiceState::Failed { .. } => service_state::FAILED,
            };

            service_data[count] = Some(ServiceData {
                name,
                pid: service.pid,
                state,
                index: idx as u8,
                parent_idx: 0xFF,
                child_count: 0,
                total_restarts: service.total_restarts,
                last_change: service.last_change as u32,
            });
            count += 1;
        });

        // Second pass: build entries with spawn context lookup
        let mut entries = [[0u8; ServiceEntry::SIZE]; 16];
        for i in 0..count {
            if let Some(data) = &service_data[i] {
                // Look up spawn context for this service's PID
                let mut bus_path = [0u8; 32];
                if data.pid != 0 {
                    if let Some((_port_type, port_name, _metadata)) = self.get_spawn_context(data.pid) {
                        let len = port_name.len().min(32);
                        bus_path[..len].copy_from_slice(&port_name[..len]);
                    }
                }

                let entry = ServiceEntry {
                    name: data.name,
                    pid: data.pid,
                    state: data.state,
                    index: data.index,
                    parent_idx: data.parent_idx,
                    child_count: data.child_count,
                    total_restarts: data.total_restarts,
                    last_change: data.last_change,
                    bus_path,
                };
                entries[i] = entry.to_bytes();
            }
        }

        // Build response
        let resp_header = ServicesListResponse::new(seq_id, count as u16);

        // Send response: header + entries
        let mut resp_buf = [0u8; 12 + 16 * ServiceEntry::SIZE]; // header + 16 entries max
        let header_bytes = resp_header.header_to_bytes();
        resp_buf[..ServicesListResponse::HEADER_SIZE].copy_from_slice(&header_bytes);

        let mut offset = ServicesListResponse::HEADER_SIZE;
        for i in 0..count {
            resp_buf[offset..offset + ServiceEntry::SIZE].copy_from_slice(&entries[i]);
            offset += ServiceEntry::SIZE;
        }

        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.channel.send(&resp_buf[..offset]);
        }
    }

    /// Handle QUERY_SERVICE_INFO - forward to the named service's driver
    fn handle_query_service_info(&mut self, client_slot: usize, buf: &[u8]) {
        use userlib::query::{QueryHeader, QueryServiceInfo, ErrorResponse, error};

        // Parse the request
        let (req, name_bytes) = match QueryServiceInfo::from_bytes(buf) {
            Some(r) => r,
            None => {
                uerror!("devd", "invalid_query_svc_info"; slot = client_slot as u32);
                return;
            }
        };

        let client_seq_id = req.header.seq_id;

        // Get service name as string
        let name_str = match core::str::from_utf8(name_bytes) {
            Ok(s) => s,
            Err(_) => {
                if let Some(client) = self.query_handler.get_mut(client_slot) {
                    let resp = ErrorResponse::new(client_seq_id, error::INVALID_REQUEST);
                    let _ = client.channel.send(&resp.to_bytes());
                }
                return;
            }
        };

        // Find the service by name
        let service_idx = match self.services.find_by_name(name_str) {
            Some(idx) => idx,
            None => {
                if let Some(client) = self.query_handler.get_mut(client_slot) {
                    let resp = ErrorResponse::new(client_seq_id, error::NOT_FOUND);
                    let _ = client.channel.send(&resp.to_bytes());
                }
                return;
            }
        };

        // Find the driver's query client slot
        let driver_slot = match self.query_handler.find_by_service_idx(service_idx as u8) {
            Some(slot) => slot,
            None => {
                // Driver not connected to query port
                if let Some(client) = self.query_handler.get_mut(client_slot) {
                    let resp = ErrorResponse::new(client_seq_id, error::NO_DRIVER);
                    let _ = client.channel.send(&resp.to_bytes());
                }
                return;
            }
        };

        // Allocate a forward sequence ID
        let forward_seq_id = self.next_forward_seq_id;
        self.next_forward_seq_id = self.next_forward_seq_id.wrapping_add(1);
        if self.next_forward_seq_id == 0 {
            self.next_forward_seq_id = 1;
        }

        // Store pending query for response routing
        let mut stored = false;
        for pending in &mut self.pending_info_queries {
            if pending.forward_seq_id == 0 {
                *pending = PendingInfoQuery {
                    forward_seq_id,
                    client_seq_id,
                    client_slot: client_slot as u8,
                    driver_slot: driver_slot as u8,
                };
                stored = true;
                break;
            }
        }

        if !stored {
            uerror!("devd", "pending_query_full";);
            if let Some(client) = self.query_handler.get_mut(client_slot) {
                let resp = ErrorResponse::new(client_seq_id, error::DEVICE_ERROR);
                let _ = client.channel.send(&resp.to_bytes());
            }
            return;
        }

        // Forward the query to the driver with our forwarded seq_id
        let forward_req = QueryServiceInfo::new(forward_seq_id);
        let mut forward_buf = [0u8; 128];
        if let Some(len) = forward_req.write_to(&mut forward_buf, name_bytes) {
            if let Some(driver_client) = self.query_handler.get_mut(driver_slot) {
                if let Err(_e) = driver_client.channel.send(&forward_buf[..len]) {
                    uerror!("devd", "query_forward_failed";);
                    // Clear the pending entry
                    for pending in &mut self.pending_info_queries {
                        if pending.forward_seq_id == forward_seq_id {
                            *pending = PendingInfoQuery::empty();
                            break;
                        }
                    }
                    if let Some(client) = self.query_handler.get_mut(client_slot) {
                        let resp = ErrorResponse::new(client_seq_id, error::NO_DRIVER);
                        let _ = client.channel.send(&resp.to_bytes());
                    }
                }
            }
        }
    }

    /// Handle SERVICE_INFO_RESULT - relay response to original client
    fn handle_service_info_result(&mut self, driver_slot: usize, buf: &[u8]) {
        use userlib::query::{QueryHeader, ServiceInfoResult, ErrorResponse, error};

        // Parse the response
        let header = match QueryHeader::from_bytes(buf) {
            Some(h) => h,
            None => {
                uerror!("devd", "invalid_svc_info_result"; slot = driver_slot as u32);
                return;
            }
        };

        let forward_seq_id = header.seq_id;

        // Find the pending query by forward_seq_id
        let mut pending_info: Option<(u32, u8)> = None;
        for pending in &mut self.pending_info_queries {
            if pending.forward_seq_id == forward_seq_id && pending.driver_slot == driver_slot as u8 {
                pending_info = Some((pending.client_seq_id, pending.client_slot));
                *pending = PendingInfoQuery::empty();
                break;
            }
        }

        let (client_seq_id, client_slot) = match pending_info {
            Some(info) => info,
            None => {
                uerror!("devd", "no_pending_query"; seq = forward_seq_id);
                return;
            }
        };

        // Parse the full response to rewrite the seq_id
        if let Some((resp, info_bytes)) = ServiceInfoResult::from_bytes(buf) {
            // Build response with original client's seq_id
            let client_resp = if resp.result == error::OK {
                ServiceInfoResult::success(client_seq_id, info_bytes.len() as u16)
            } else {
                ServiceInfoResult::new(client_seq_id, resp.result)
            };

            let mut resp_buf = [0u8; 1100];
            if let Some(len) = client_resp.write_to(&mut resp_buf, info_bytes) {
                if let Some(client) = self.query_handler.get_mut(client_slot as usize) {
                    let _ = client.channel.send(&resp_buf[..len]);
                }
            }
        } else {
            // Couldn't parse - send error
            if let Some(client) = self.query_handler.get_mut(client_slot as usize) {
                let resp = ErrorResponse::new(client_seq_id, error::DEVICE_ERROR);
                let _ = client.channel.send(&resp.to_bytes());
            }
        }
    }

    // =========================================================================
    // Log Handling
    // =========================================================================

    /// Handle LOG_MESSAGE from a driver
    fn handle_log_message(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::LogMessage;

        let (msg, text) = match LogMessage::from_bytes(buf) {
            Some(m) => m,
            None => return,
        };

        // Get service index for this slot (-1 means not a driver)
        let service_idx = self.query_handler.get(slot)
            .map(|c| c.service_idx)
            .unwrap_or(-1);

        let service_idx_u8 = if service_idx >= 0 { service_idx as u8 } else { 0xFF };

        // Buffer the message
        self.log_buffer.push(service_idx_u8, msg.level, text);

    }

    /// Handle LOG_QUERY - return buffered logs
    fn handle_log_query(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{LogQuery, LogHistory};

        let req = match LogQuery::from_bytes(buf) {
            Some(r) => r,
            None => return,
        };

        let max_count = if req.max_count == 0 { 20 } else { req.max_count as usize };

        let mut text_buf = [0u8; 4000];
        let text_len = self.log_buffer.format_recent(max_count, &mut text_buf);

        let resp = LogHistory::new(
            req.header.seq_id,
            self.log_buffer.count.min(max_count) as u8,
            self.live_logging,
        );

        let mut resp_buf = [0u8; 4100];
        if let Some(len) = resp.write_to(&mut resp_buf, &text_buf[..text_len]) {
            if let Some(client) = self.query_handler.get_mut(slot) {
                let _ = client.channel.send(&resp_buf[..len]);
            }
        }
    }

    /// Handle LOG_CONTROL - enable/disable live logging
    fn handle_log_control(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{LogControl, log_cmd, ErrorResponse, error};

        let req = match LogControl::from_bytes(buf) {
            Some(r) => r,
            None => return,
        };

        match req.command {
            log_cmd::ENABLE => {
                self.live_logging = true;
            }
            log_cmd::DISABLE => {
                self.live_logging = false;
            }
            _ => {}
        }

        // Send ack
        let resp = ErrorResponse::new(req.header.seq_id, error::OK);
        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.channel.send(&resp.to_bytes());
        }
    }

    /// Handle SPAWN_ACK message from a driver
    ///
    /// When a driver spawns children and sends SPAWN_ACK, we need to:
    /// 1. Create service slots for the children so they can be recognized
    /// 2. Add them to recent_dynamic_pids for the lookup race window
    fn handle_spawn_ack_msg(&mut self, slot: usize, buf: &[u8]) {
        use crate::query::QueryHandler;

        // Parse the SPAWN_ACK message
        let (seq_id, result, match_count, spawn_count, pids) = match QueryHandler::parse_spawn_ack(buf) {
            Some(p) => p,
            None => {
                uerror!("devd", "invalid_spawn_ack"; slot = slot as u32);
                return;
            }
        };

        // Get the parent driver's service index
        let parent_idx = match self.query_handler.get_service_idx(slot) {
            Some(idx) => idx,
            None => {
                return;
            }
        };

        uinfo!("devd", "spawn_ack"; parent = self.svc_name(parent_idx as u8), seq = seq_id, result = result as i32, spawn = spawn_count as u32);

        // Consume the inflight spawn to get port context and binary name
        let spawn_ctx = self.consume_inflight_spawn(seq_id);

        if spawn_count == 0 || result < 0 {
            if let Some((_, _, _, _, _, service_idx, _, _)) = spawn_ctx {
                if service_idx != 0xFF {
                    let now = Self::now_ms();
                    self.services.transition(service_idx as usize, ServiceState::Failed { code: result as i32 }, now);
                }
            }
            return;
        }

        // Check if this is a service spawn (parent-delegated)
        if let Some((port_type, port_name, port_name_len, _, _, service_idx, spawn_caps, link_id)) = spawn_ctx {
            if service_idx != 0xFF && spawn_count >= 1 {
                let child_pid = pids[0];
                let now = Self::now_ms();
                let pname = &port_name[..port_name_len as usize];
                if let Some(service) = self.services.get_mut(service_idx as usize) {
                    service.pid = child_pid;
                    service.set_trigger_port(pname);
                    service.caps = spawn_caps;
                    service.link_id = link_id;
                    uinfo!("devd", "service_spawn_ack"; service = service.name(), pid = child_pid);
                }
                if !pname.is_empty() {
                    let metadata: ([u8; 64], usize) = self.ports.get(pname)
                        .map(|p| {
                            let m = p.metadata();
                            let mut buf = [0u8; 64];
                            let len = m.len().min(64);
                            buf[..len].copy_from_slice(&m[..len]);
                            (buf, len)
                        })
                        .unwrap_or(([0u8; 64], 0));
                    self.store_spawn_context(child_pid, port_type, pname, &metadata.0[..metadata.1]);
                }
                self.query_handler.upgrade_client_to_driver(child_pid, service_idx);
                return;
            }
        }

        // Extract binary name, caps, and link_id from spawn context (for dynamic spawns)
        let binary_name: &[u8] = spawn_ctx.as_ref()
            .map(|(_, _, _, bin, bin_len, _, _, _)| &bin[..*bin_len as usize])
            .unwrap_or(b"???");
        let spawn_caps: u64 = spawn_ctx.as_ref()
            .map(|(_, _, _, _, _, _, caps, _)| *caps)
            .unwrap_or(0);
        let spawn_link_id: u32 = spawn_ctx.as_ref()
            .map(|(_, _, _, _, _, _, _, lid)| *lid)
            .unwrap_or(0);

        // Create or reuse service slots for each spawned child
        let now = Self::now_ms();
        for i in 0..spawn_count as usize {
            let child_pid = pids[i];
            if child_pid == 0 {
                continue;
            }

            // Reuse existing service slot by link_id if available,
            // otherwise create a new one. This prevents slot accumulation on respawn.
            let mut reused_idx: Option<usize> = None;
            if spawn_link_id != 0 {
                self.services.for_each(|idx, svc| {
                    if svc.link_id == spawn_link_id {
                        reused_idx = Some(idx);
                    }
                });
            }

            let slot_idx = if let Some(idx) = reused_idx {
                // Reuse existing slot — update PID and state
                if let Some(svc) = self.services.get_mut(idx) {
                    svc.pid = child_pid;
                    svc.state = ServiceState::Starting;
                    svc.caps = spawn_caps;
                }
                idx
            } else {
                // Create a new service slot.
                // Use Starting state — the child hasn't reported Ready yet.
                match self.services.create_dynamic_service_with_state(
                    child_pid, binary_name, now, ServiceState::Starting,
                ) {
                    Some(idx) => idx,
                    None => {
                        uerror!("devd", "no_child_slot"; pid = child_pid);
                        continue;
                    }
                }
            };

            // Store trigger_port, caps, and link_id on the service slot (for restart)
            if let Some((_, port_name, port_name_len, _, _, _, _, link_id)) = spawn_ctx {
                let pname = &port_name[..port_name_len as usize];
                if let Some(service) = self.services.get_mut(slot_idx) {
                    service.set_trigger_port(pname);
                    service.caps = spawn_caps;
                    service.link_id = link_id;
                }
            }

            // Add to recent_dynamic_pids (for race condition handling)
            self.add_recent_dynamic_pid(child_pid, slot_idx as u8);

            // Store spawn context (port that triggered spawn) for GET_SPAWN_CONTEXT
            // Include metadata from the port registration (e.g., BAR0 info)
            if let Some((port_type, port_name, port_name_len, _, _, _, _, _)) = spawn_ctx {
                let pname = &port_name[..port_name_len as usize];
                // Look up metadata from the registered port
                let metadata: ([u8; 64], usize) = self.ports.get(pname)
                    .map(|p| {
                        let m = p.metadata();
                        let mut buf = [0u8; 64];
                        let len = m.len().min(64);
                        buf[..len].copy_from_slice(&m[..len]);
                        (buf, len)
                    })
                    .unwrap_or(([0u8; 64], 0));
                self.store_spawn_context(child_pid, port_type, pname, &metadata.0[..metadata.1]);
            }

            // Upgrade any already-connected client with this PID to driver status
            // This handles the race where child connects before SPAWN_ACK arrives
            self.query_handler.upgrade_client_to_driver(child_pid, slot_idx as u8);

        }
    }

    fn handle_query_driver_forward(&mut self, _client_slot: usize, buf: &[u8]) {
        use userlib::query::{ErrorResponse, QueryHeader, error};

        // Parse the driver query
        let parsed = self.query_handler.parse_driver_query(buf, &self.devices);

        let (seq_id, driver_idx, _query_type, _payload) = match parsed {
            Some(p) => p,
            None => {
                // Send error response
                if let Some(header) = QueryHeader::from_bytes(buf) {
                    let resp = ErrorResponse::new(header.seq_id, error::NOT_FOUND);
                    if let Some(client) = self.query_handler.get_mut(_client_slot) {
                        let _ = client.channel.send(&resp.to_bytes());
                    }
                }
                return;
            }
        };

        // Find driver's channel
        let driver_channel = self.services.get(driver_idx as usize)
            .and_then(|s| s.channel.as_ref());

        match driver_channel {
            Some(_ch) => {
                // TODO: Forward query to driver and wait for response
                // For now, return NOT_SUPPORTED since we haven't integrated driver-side handling
                let resp = ErrorResponse::new(seq_id, error::NOT_SUPPORTED);
                if let Some(client) = self.query_handler.get_mut(_client_slot) {
                    let _ = client.channel.send(&resp.to_bytes());
                }
            }
            None => {
                let resp = ErrorResponse::new(seq_id, error::NO_DRIVER);
                if let Some(client) = self.query_handler.get_mut(_client_slot) {
                    let _ = client.channel.send(&resp.to_bytes());
                }
            }
        }
    }

    fn remove_query_client(&mut self, slot: usize) {
        if let Some(channel) = self.query_handler.remove_client(slot) {
            if let Some(events) = &mut self.events {
                let _ = events.unwatch(channel.handle());
            }
        }
    }

    // =========================================================================
    // Dynamic Port Registration
    // =========================================================================

    /// Handle port registration with unified PortInfo
    ///
    /// Called when a driver sends REGISTER_PORT_INFO to devd-query:.
    /// Uses the unified PortInfo struct for type-safe, structured metadata.
    pub fn handle_port_registration(
        &mut self,
        port_info: &abi::PortInfo,
        owner_idx: u8,
        shmem_id: u32,
    ) -> Result<(), SysError> {
        use crate::ports::PortRegistry;

        // Register the port with unified PortInfo
        self.ports.register_with_port_info(port_info, owner_idx, shmem_id)?;

        let port_name = port_info.name_bytes();

        // Log registration with class info
        if let Ok(name_str) = core::str::from_utf8(port_name) {
            if shmem_id != 0 {
                uinfo!("devd", "port_info_registered";
                    name = name_str,
                    class = port_info.port_class as u16,
                    subclass = port_info.port_subclass,
                    shmem_id = shmem_id
                );
            } else {
                uinfo!("devd", "port_info_registered";
                    name = name_str,
                    class = port_info.port_class as u16,
                    subclass = port_info.port_subclass
                );
            }
        }

        // Track block devices for ATTACH_DISK orchestration
        if port_info.port_class == abi::PortClass::Block && shmem_id != 0 {
            self.track_and_attach_disk(port_name, shmem_id, owner_idx);
        }

        Ok(())
    }

    /// Fire spawn rules for a port that transitioned to Claimed.
    ///
    /// For kernel bus ports (owner=0xFF): devd spawns directly.
    /// For driver-owned ports: devd sends SPAWN_CHILD to the owning driver,
    /// which spawns the child as its own child process.
    fn check_class_rules(&mut self, port_info: &abi::PortInfo, owner_idx: u8) {
        let rule = match rules::find_port_rule(port_info) {
            Some(r) => r,
            None => return,
        };

        // Guard: don't spawn if a running service is already bound to this port.
        // Match by link_id from the port, not by trigger_port string.
        let port_name = port_info.name_bytes();
        let port_link_id = self.ports.get(port_name)
            .map(|p| p.child_link_id())
            .unwrap_or(0);
        let mut existing_idx: Option<usize> = None;
        if port_link_id != 0 {
            self.services.for_each(|idx, svc| {
                if svc.link_id == port_link_id {
                    existing_idx = Some(idx);
                }
            });
        }
        if let Some(idx) = existing_idx {
            if !self.services.get(idx).map(|s| s.state.is_exited()).unwrap_or(false) {
                return;
            }
            // Cancel old instance's restart timer to prevent duplicate spawn
            let old_timer = self.services.get_mut(idx).and_then(|s| s.restart_timer.take());
            if let Some(timer) = old_timer {
                if let Some(events) = &mut self.events {
                    let _ = events.unwatch(timer.handle());
                }
            }
            if let Some(service) = self.services.get_mut(idx) {
                service.state = ServiceState::Stopped { code: 0 };
            }
        }

        // Allocate a link_id for this port↔service pairing
        let link_id = self.alloc_link_id();
        if let Some(port) = self.ports.get_mut(port_name) {
            port.set_child_link_id(link_id);
        }

        uinfo!("devd", "port_rule_matched";
            class = port_info.port_class as u16,
            subclass = port_info.port_subclass,
            driver = rule.driver
        );

        if owner_idx != 0xFF {
            // Driver-owned port: delegate spawn to the owning driver.
            // The child becomes the driver's child, not devd's.
            let port_type = self.ports.get_port_type(port_name).unwrap_or(0);
            if let Some(seq_id) = self.query_handler.send_spawn_child_with_caps(
                owner_idx, rule.driver.as_bytes(), port_name, rule.caps,
            ) {
                self.track_inflight_spawn(seq_id, port_type, port_name, rule.driver, rule.caps, link_id);
            } else {
                uerror!("devd", "spawn_child_send_failed"; driver = rule.driver, owner = owner_idx as u32);
            }
        } else {
            // Kernel bus port: devd spawns directly (no driver to delegate to)
            if let Some(idx) = existing_idx {
                // Update existing service's link_id to the new one
                if let Some(service) = self.services.get_mut(idx) {
                    service.link_id = link_id;
                }
                self.spawn_service(idx);
            } else {
                self.create_and_spawn(rule.driver, rule.caps, port_name, link_id);
            }
        }
    }

    // =========================================================================
    // Block Device Orchestration
    // =========================================================================

    /// Track a block device, spawn partd if needed, and send ATTACH_DISK.
    ///
    /// partd is a singleton — spawned on first disk discovery, reused for all
    /// subsequent disks.  When partd connects it receives ATTACH_DISK for every
    /// pending disk via partd_connected().
    fn track_and_attach_disk(&mut self, port_name: &[u8], shmem_id: u32, owner_idx: u8) {
        // Track this disk
        let disk_slot = self.track_disk(port_name, shmem_id, owner_idx);
        if disk_slot.is_none() {
            uerror!("devd", "disk_track_full"; shmem_id = shmem_id);
            return;
        }
        let disk_slot = disk_slot.unwrap();

        uinfo!("devd", "disk_tracked"; slot = disk_slot as u32, shmem_id = shmem_id);

        if self.partd_service_idx != 0xFF {
            // partd already connected — send ATTACH_DISK immediately
            self.send_attach_disk(disk_slot);
        } else {
            // Spawn partd if not already spawned (check by name)
            let already_spawned = {
                let mut found = false;
                self.services.for_each(|_idx, svc| {
                    if svc.name() == "partd" { found = true; }
                });
                found
            };
            if !already_spawned {
                self.create_and_spawn("partd", userlib::devd::caps::DRIVER, &[], 0);
            }
            // partd_connected() will send ATTACH_DISK for all Discovered disks
        }
    }

    /// Track a disk for orchestration
    /// State transition: Empty -> Discovered
    fn track_disk(&mut self, port_name: &[u8], shmem_id: u32, owner_idx: u8) -> Option<usize> {
        // Find empty slot (state == Empty)
        let slot = (0..MAX_TRACKED_DISKS).find(|&i| {
            self.tracked_disks[i].state == DiskState::Empty
        })?;

        let disk = &mut self.tracked_disks[slot];

        // Transition Empty -> Discovered
        if !disk.transition(DiskState::Discovered) {
            uerror!("devd", "disk_transition_failed"; target = "Discovered");
            return None;
        }

        disk.shmem_id = shmem_id;
        disk.owner_idx = owner_idx;
        let len = port_name.len().min(32);
        disk.port_name[..len].copy_from_slice(&port_name[..len]);
        disk.port_name_len = len as u8;
        // block_size and block_count will be filled when we query the DataPort

        Some(slot)
    }

    /// Find tracked disk by shmem_id (in any non-Empty state)
    fn find_disk_by_shmem_id(&self, shmem_id: u32) -> Option<usize> {
        (0..MAX_TRACKED_DISKS).find(|&i| {
            self.tracked_disks[i].state != DiskState::Empty
                && self.tracked_disks[i].shmem_id == shmem_id
        })
    }

    /// Get next orchestration sequence ID
    fn next_orch_seq_id(&mut self) -> u32 {
        let id = self.orch_seq_id;
        self.orch_seq_id = self.orch_seq_id.wrapping_add(1);
        if self.orch_seq_id == 0 {
            self.orch_seq_id = 1;
        }
        id
    }

    /// Send ATTACH_DISK to partd
    /// State transition: Discovered -> AttachSent
    fn send_attach_disk(&mut self, disk_slot: usize) {
        let partd_idx = self.partd_service_idx;
        if partd_idx == 0xFF {
            uerror!("devd", "attach_no_partd";);
            return;
        }

        // Verify disk is in Discovered state
        if self.tracked_disks[disk_slot].state != DiskState::Discovered {
            uerror!("devd", "attach_wrong_state";);
            return;
        }

        // Get disk info - copy to local before mutable borrow
        let (shmem_id, port_name_buf, port_name_len) = {
            let disk = &self.tracked_disks[disk_slot];
            let mut name_buf = [0u8; 32];
            let len = disk.port_name_len as usize;
            name_buf[..len].copy_from_slice(&disk.port_name[..len]);
            (disk.shmem_id, name_buf, len)
        };
        let port_name = &port_name_buf[..port_name_len];

        // Build ATTACH_DISK message
        // Note: block_size and block_count are 0 for now - partd will query DataPort
        let seq_id = self.next_orch_seq_id();
        let attach = AttachDisk::new(seq_id, shmem_id, 0, 0);
        let mut buf = [0u8; 128];
        let len = match attach.write_to(&mut buf, port_name) {
            Some(l) => l,
            None => {
                uerror!("devd", "attach_serialize_failed";);
                return;
            }
        };

        // Transition Discovered -> AttachSent (tracks seq_id in state)
        if !self.tracked_disks[disk_slot].transition(DiskState::AttachSent { seq_id }) {
            uerror!("devd", "disk_transition_failed"; target = "AttachSent");
            return;
        }

        // Send to partd
        let slot = self.query_handler.find_by_service_idx(partd_idx);
        if let Some(slot) = slot {
            if let Some(client) = self.query_handler.get_mut(slot) {
                match client.channel.send(&buf[..len]) {
                    Ok(_) => {
                        if let Ok(name_str) = core::str::from_utf8(port_name) {
                            uinfo!("devd", "attach_disk_sent"; seq = seq_id, port = name_str, shmem_id = shmem_id);
                        }
                    }
                    Err(_e) => {
                        uerror!("devd", "attach_send_failed";);
                    }
                }
            }
        } else {
            uerror!("devd", "partd_slot_missing"; idx = partd_idx as u32);
        }
    }

    /// Handle REPORT_PARTITIONS from partd
    /// State transitions:
    ///   Disk: AttachSent -> PartitionsReported
    ///   Partitions: Empty -> Discovered
    fn handle_report_partitions(&mut self, slot: usize, buf: &[u8]) {
        uinfo!("devd", "report_partitions"; slot = slot as u32, len = buf.len() as u32);
        let report = match ReportPartitions::from_bytes(buf) {
            Some(r) => r,
            None => {
                uerror!("devd", "invalid_report_partitions";);
                return;
            }
        };

        // Find the disk this is for
        let disk_slot = match self.find_disk_by_shmem_id(report.source_shmem_id) {
            Some(s) => s,
            None => {
                uerror!("devd", "unknown_disk"; shmem_id = report.source_shmem_id);
                return;
            }
        };

        // Verify disk is in AttachSent state
        if !matches!(self.tracked_disks[disk_slot].state, DiskState::AttachSent { .. }) {
            uerror!("devd", "report_wrong_state";);
            return;
        }

        // Transition disk: AttachSent -> PartitionsReported
        if !self.tracked_disks[disk_slot].transition(DiskState::PartitionsReported) {
            uerror!("devd", "disk_transition_failed"; target = "PartitionsReported");
            return;
        }

        let scheme_str = if report.scheme == part_scheme::GPT { "GPT" } else { "MBR" };
        uinfo!("devd", "partitions_found"; count = report.count as u32, scheme = scheme_str);

        // Parse partition info from buffer (follows fixed header)
        let partition_count = (report.count as usize).min(MAX_PARTITIONS_PER_DISK);

        let part_data = &buf[ReportPartitions::FIXED_SIZE..];
        for i in 0..partition_count {
            let offset = i * PartitionInfoMsg::SIZE;
            if offset + PartitionInfoMsg::SIZE > part_data.len() {
                break;
            }
            let pdata = &part_data[offset..offset + PartitionInfoMsg::SIZE];
            if let Some(pinfo) = PartitionInfoMsg::from_bytes(pdata) {
                let part = &mut self.tracked_disks[disk_slot].partitions[i];

                // Transition partition: Empty -> Discovered
                if !part.transition(PartitionState::Discovered) {
                    uerror!("devd", "part_transition_failed"; idx = i as u32, target = "Discovered");
                    continue;
                }

                part.index = pinfo.index;
                part.part_type = pinfo.part_type;
                part.bootable = pinfo.bootable != 0;
                part.start_lba = pinfo.start_lba;
                part.size_sectors = pinfo.size_sectors;

                // Log each partition
                let fs_str = match fs_hint::from_mbr_type(pinfo.part_type) {
                    fs_hint::FAT12 => "FAT12",
                    fs_hint::FAT16 => "FAT16",
                    fs_hint::FAT32 => "FAT32",
                    fs_hint::EXT4 => "ext4",
                    fs_hint::NTFS => "NTFS",
                    fs_hint::SWAP => "swap",
                    _ => "unknown",
                };
                uinfo!("devd", "partition_info"; idx = pinfo.index as u32, fs = fs_str, start = pinfo.start_lba as u64, size = pinfo.size_sectors as u64);
            }
        }

        // Store partition count and scheme
        self.tracked_disks[disk_slot].partition_count = partition_count as u8;
        self.tracked_disks[disk_slot].scheme = report.scheme;

        // Collect partitions to register (avoid borrow issues)
        let mut to_register = [false; MAX_PARTITIONS_PER_DISK];
        for i in 0..partition_count {
            let part = &self.tracked_disks[disk_slot].partitions[i];
            if part.state == PartitionState::Discovered {
                let fs_type = fs_hint::from_mbr_type(part.part_type);
                // Only register partitions with known filesystem types
                if fs_type != fs_hint::UNKNOWN && fs_type != fs_hint::SWAP {
                    to_register[i] = true;
                }
            }
        }

        // Now send REGISTER_PARTITION for each mountable partition
        for i in 0..partition_count {
            if to_register[i] {
                self.send_register_partition(disk_slot, i, slot);
            }
        }
    }

    /// Send REGISTER_PARTITION to partd
    /// State transition: Discovered -> Registering
    fn send_register_partition(&mut self, disk_slot: usize, part_idx: usize, partd_slot: usize) {
        // Verify partition is in Discovered state
        if self.tracked_disks[disk_slot].partitions[part_idx].state != PartitionState::Discovered {
            uerror!("devd", "part_register_wrong_state";);
            return;
        }

        // Get partition info before mutable operations
        let part_table_index = self.tracked_disks[disk_slot].partitions[part_idx].index;
        let part_type = self.tracked_disks[disk_slot].partitions[part_idx].part_type;

        // Assign partition name with filesystem type tag: "N:fat", "N:ext2", etc.
        let part_num = self.next_partition_num;
        self.next_partition_num += 1;

        let mut name_buf = [0u8; 32];
        let name_len = {
            let mut pos = 0;
            // Write partition number
            if part_num == 0 {
                name_buf[pos] = b'0';
                pos += 1;
            } else {
                let mut n = part_num;
                let mut digits = [0u8; 10];
                let mut d = 0;
                while n > 0 {
                    digits[d] = b'0' + (n % 10) as u8;
                    n /= 10;
                    d += 1;
                }
                for i in (0..d).rev() {
                    name_buf[pos] = digits[i];
                    pos += 1;
                }
            }
            // Write type tag based on filesystem hint
            name_buf[pos] = b':';
            pos += 1;
            let fs_type = fs_hint::from_mbr_type(part_type);
            let tag: &[u8] = match fs_type {
                fs_hint::FAT12 | fs_hint::FAT16 | fs_hint::FAT32 => b"fat",
                fs_hint::EXFAT => b"exfat",
                fs_hint::EXT2 | fs_hint::EXT4 => b"ext2",
                fs_hint::NTFS => b"ntfs",
                _ => b"blk",
            };
            let end = (pos + tag.len()).min(name_buf.len());
            name_buf[pos..end].copy_from_slice(&tag[..end - pos]);
            pos = end;
            pos
        };

        // Build REGISTER_PARTITION message
        let seq_id = self.next_orch_seq_id();
        let reg = RegisterPartitionMsg::new(seq_id, part_table_index);
        let mut buf = [0u8; 64];
        let len = match reg.write_to(&mut buf, &name_buf[..name_len]) {
            Some(l) => l,
            None => {
                uerror!("devd", "part_serialize_failed";);
                return;
            }
        };

        // Store assigned name and transition to Registering state
        {
            let part = &mut self.tracked_disks[disk_slot].partitions[part_idx];
            part.assigned_name[..name_len].copy_from_slice(&name_buf[..name_len]);
            part.assigned_name_len = name_len as u8;

            // Transition Discovered -> Registering (tracks seq_id in state)
            if !part.transition(PartitionState::Registering { seq_id }) {
                uerror!("devd", "part_transition_failed"; target = "Registering");
                return;
            }
        }

        // Send to partd
        if let Some(client) = self.query_handler.get_mut(partd_slot) {
            match client.channel.send(&buf[..len]) {
                Ok(_) => {
                    if let Ok(name_str) = core::str::from_utf8(&name_buf[..name_len]) {
                        uinfo!("devd", "register_partition"; idx = part_table_index as u32, name = name_str);
                    }
                }
                Err(_e) => {
                    uerror!("devd", "part_register_send_failed";);
                }
            }
        }
    }

    /// Handle PARTITION_READY from partd
    /// State transition: Registering -> Ready
    fn handle_partition_ready(&mut self, _slot: usize, buf: &[u8]) {
        let (ready, name) = match PartitionReadyMsg::from_bytes(buf) {
            Some(r) => r,
            None => {
                uerror!("devd", "invalid_partition_ready";);
                return;
            }
        };

        // Find the partition by name (must be in Registering state)
        let mut found = None;
        'outer: for disk_idx in 0..MAX_TRACKED_DISKS {
            let disk = &self.tracked_disks[disk_idx];
            if disk.state == DiskState::Empty {
                continue;
            }
            for part_idx in 0..(disk.partition_count as usize).min(MAX_PARTITIONS_PER_DISK) {
                let part = &disk.partitions[part_idx];
                // Must be in Registering state to receive Ready
                if !matches!(part.state, PartitionState::Registering { .. }) {
                    continue;
                }
                let assigned_name = &part.assigned_name[..part.assigned_name_len as usize];
                if assigned_name == name {
                    found = Some((disk_idx, part_idx));
                    break 'outer;
                }
            }
        }

        let (disk_idx, part_idx) = match found {
            Some(f) => f,
            None => {
                if let Ok(name_str) = core::str::from_utf8(name) {
                    uerror!("devd", "unknown_partition"; name = name_str);
                }
                return;
            }
        };

        // Transition partition: Registering -> Ready
        {
            let part = &mut self.tracked_disks[disk_idx].partitions[part_idx];
            if !part.transition(PartitionState::Ready { shmem_id: ready.shmem_id }) {
                uerror!("devd", "part_transition_failed"; target = "Ready");
                return;
            }
        }

        if let Ok(name_str) = core::str::from_utf8(name) {
            uinfo!("devd", "partition_ready"; name = name_str, shmem_id = ready.shmem_id);
        }

        // Get partition type for class-based rule matching and registration
        let part_type = self.tracked_disks[disk_idx].partitions[part_idx].part_type;
        let disk = &self.tracked_disks[disk_idx];
        let disk_name = &disk.port_name[..disk.port_name_len as usize];

        // Create PortInfo for the partition with MBR type as subclass
        let mut port_info = abi::PortInfo::new(name, abi::PortClass::Block);
        port_info.port_subclass = part_type as u16;  // MBR type codes map to port_subclass
        port_info.set_parent(disk_name);

        // Register the partition port with unified PortInfo
        let _ = self.ports.register_with_port_info(&port_info, self.partd_service_idx, ready.shmem_id);

        // Check class-based rules for auto-spawning filesystem drivers
        self.check_class_rules(&port_info, self.partd_service_idx);
    }

    /// Check if a newly connected driver is partd
    fn check_orchestration_driver_connected(&mut self, service_idx: u8) {
        if let Some(service) = self.services.get(service_idx as usize) {
            if service.name() == "partd" {
                self.partd_connected(service_idx);
            }
        }
    }

    /// Called when partd connects to devd-query
    /// Send AttachDisk for any disks still in Discovered state
    fn partd_connected(&mut self, service_idx: u8) {
        if self.partd_service_idx != 0xFF {
            return; // Already connected
        }
        self.partd_service_idx = service_idx;
        uinfo!("devd", "partd_connected"; svc = service_idx as u32);

        // Send AttachDisk for any disks in Discovered state
        // (they haven't had AttachDisk sent yet)
        for disk_slot in 0..MAX_TRACKED_DISKS {
            if self.tracked_disks[disk_slot].state == DiskState::Discovered {
                self.send_attach_disk(disk_slot);
            }
        }
    }

    /// Spawn a driver dynamically
    fn spawn_dynamic_driver(&mut self, binary: &str, trigger_port: &[u8]) {
        self.spawn_dynamic_driver_with_caps(binary, trigger_port, 0);
    }

    /// Spawn a driver dynamically with explicit capabilities
    fn spawn_dynamic_driver_with_caps(&mut self, binary: &str, trigger_port: &[u8], caps: u64) {
        let now = Self::now_ms();

        // Spawn the process first (with caps if specified)
        let (pid, watcher) = match self.process_mgr.spawn_with_caps(binary, caps) {
            Ok(result) => result,
            Err(e) => {
                uerror!("devd", "dynamic_spawn_failed"; binary = binary);
                return;
            }
        };

        // Store spawn context for GET_SPAWN_CONTEXT
        // Include metadata from port registration (e.g., BAR0 info)
        if !trigger_port.is_empty() {
            let port_type = self.ports.get_port_type(trigger_port).unwrap_or(0);
            let metadata: ([u8; 64], usize) = self.ports.get(trigger_port)
                .map(|p| {
                    let m = p.metadata();
                    let mut buf = [0u8; 64];
                    let len = m.len().min(64);
                    buf[..len].copy_from_slice(&m[..len]);
                    (buf, len)
                })
                .unwrap_or(([0u8; 64], 0));
            self.store_spawn_context(pid, port_type, trigger_port, &metadata.0[..metadata.1]);
        }

        // Create service slot using create_dynamic_service (which actually creates the entry)
        let slot_idx = self.services.create_dynamic_service(pid, binary.as_bytes(), now);

        // Set up the service slot with watcher, trigger_port, and caps
        if let Some(idx) = slot_idx {
            if let Some(service) = self.services.get_mut(idx) {
                service.watcher = Some(watcher);
                service.set_trigger_port(trigger_port);
                service.caps = caps;
            }

            // Add to recent_dynamic_pids for race condition handling
            self.add_recent_dynamic_pid(pid, idx as u8);

            // Add watcher to event loop
            if let Some(events) = &mut self.events {
                if let Some(service) = self.services.get(idx) {
                    if let Some(w) = &service.watcher {
                        let _ = events.watch(w.handle());
                    }
                }
            }
        } else {
            // No slot available
            if let Some(events) = &mut self.events {
                let _ = events.watch(watcher.handle());
            }
            uerror!("devd", "no_dynamic_slot"; pid = pid);
        }
    }

    /// Add a recently spawned dynamic PID to the tracking array
    fn add_recent_dynamic_pid(&mut self, pid: u32, service_idx: u8) {
        // Find an empty slot or the oldest entry
        for entry in &mut self.recent_dynamic_pids {
            if entry.0 == 0 {
                *entry = (pid, service_idx);
                return;
            }
        }
        // All slots full - overwrite the first one (oldest)
        self.recent_dynamic_pids[0] = (pid, service_idx);
    }

    /// Look up a recently spawned dynamic PID
    fn find_recent_dynamic_pid(&self, pid: u32) -> Option<u8> {
        for &(p, idx) in &self.recent_dynamic_pids {
            if p == pid {
                return Some(idx);
            }
        }
        None
    }

    /// Remove a PID from the recent dynamic PIDs tracking
    fn remove_recent_dynamic_pid(&mut self, pid: u32) {
        for entry in &mut self.recent_dynamic_pids {
            if entry.0 == pid {
                *entry = (0, 0);
                return;
            }
        }
    }

    // =========================================================================
    // Spawn Context Tracking
    // =========================================================================

    /// Track an in-flight spawn command (seq_id -> port info + binary name + caps)
    fn track_inflight_spawn(&mut self, seq_id: u32, port_type: u8, port_name: &[u8], binary: &str, caps: u64, link_id: u32) {
        // Find empty slot
        for entry in &mut self.inflight_spawns {
            if entry.seq_id == 0 {
                entry.seq_id = seq_id;
                entry.port_type = port_type;
                let len = port_name.len().min(32);
                entry.port_name[..len].copy_from_slice(&port_name[..len]);
                entry.port_name_len = len as u8;
                let bin_len = binary.len().min(16);
                entry.binary_name[..bin_len].copy_from_slice(&binary.as_bytes()[..bin_len]);
                entry.binary_name_len = bin_len as u8;
                entry.service_idx = 0xFF;  // Not a service spawn
                entry.caps = caps;
                entry.link_id = link_id;
                return;
            }
        }
    }

    /// Consume an in-flight spawn by seq_id
    /// Returns (port_type, port_name, port_len, binary_name, binary_len, service_idx, caps, link_id)
    fn consume_inflight_spawn(&mut self, seq_id: u32) -> Option<(u8, [u8; 32], u8, [u8; 16], u8, u8, u64, u32)> {
        for entry in &mut self.inflight_spawns {
            if entry.seq_id == seq_id {
                let result = (entry.port_type, entry.port_name, entry.port_name_len,
                              entry.binary_name, entry.binary_name_len, entry.service_idx,
                              entry.caps, entry.link_id);
                *entry = InflightSpawn::empty();
                return Some(result);
            }
        }
        None
    }

    /// Store spawn context for a child PID (with optional metadata from port registration)
    fn store_spawn_context(&mut self, pid: u32, port_type: u8, port_name: &[u8], metadata: &[u8]) {
        // Find empty slot or oldest entry
        let mut empty_slot = None;
        for (i, entry) in self.spawn_contexts.iter_mut().enumerate() {
            if entry.pid == 0 {
                empty_slot = Some(i);
                break;
            }
        }

        let slot = empty_slot.unwrap_or(0); // Overwrite first slot if full
        self.spawn_contexts[slot].pid = pid;
        self.spawn_contexts[slot].port_type = port_type;
        let len = port_name.len().min(32);
        self.spawn_contexts[slot].port_name[..len].copy_from_slice(&port_name[..len]);
        self.spawn_contexts[slot].port_name_len = len as u8;
        let meta_len = metadata.len().min(64);
        self.spawn_contexts[slot].metadata[..meta_len].copy_from_slice(&metadata[..meta_len]);
        self.spawn_contexts[slot].metadata_len = meta_len as u8;
    }

    /// Get spawn context for a PID
    fn get_spawn_context(&self, pid: u32) -> Option<(u8, &[u8], &[u8])> {
        for entry in &self.spawn_contexts {
            if entry.pid == pid {
                return Some((
                    entry.port_type,
                    &entry.port_name[..entry.port_name_len as usize],
                    &entry.metadata[..entry.metadata_len as usize],
                ));
            }
        }
        None
    }

    /// Remove spawn context for a PID (when process exits)
    fn remove_spawn_context(&mut self, pid: u32) {
        for entry in &mut self.spawn_contexts {
            if entry.pid == pid {
                *entry = SpawnContext::empty();
                return;
            }
        }
    }

    // =========================================================================
    // Main Loop
    // =========================================================================

    pub fn run(&mut self) -> ! {
        // Set a persistent timeout so the Mux wakes periodically even if no
        // watched handles fire.  This lets us poll query clients whose channel
        // handles couldn't fit into the Mux (16-handle limit).
        if let Some(events) = &self.events {
            if events.set_timeout(100).is_err() {
                uerror!("devd", "timeout_set_failed";);
            }
        } else {
            uerror!("devd", "events_not_init";);
        }

        loop {
            let events = self.events.as_ref().expect("devd: events not initialized");
            let wait_result = events.wait();

            match wait_result {
                Ok(handle) => {
                    // Query port event?
                    if let Some(query_port) = &self.query_port {
                        if handle == query_port.handle() {
                            self.handle_query_port_event();
                            self.poll_query_clients();
                            continue;
                        }
                    }

                    // Query client message?
                    if self.query_handler.find_by_handle(handle).is_some() {
                        self.handle_query_client_event(handle);
                        self.poll_query_clients();
                        continue;
                    }

                    // Admin client (shell) message?
                    if self.admin_clients.find_by_handle(handle).is_some() {
                        self.handle_admin_client_event(handle);
                        self.poll_query_clients();
                        continue;
                    }

                    // Service restart timer?
                    if let Some(idx) = self.services.find_by_timer(handle) {
                        self.handle_restart_timer(idx);
                        self.poll_query_clients();
                        continue;
                    }

                    // Process exit (watcher)?
                    self.handle_process_exit(handle);
                }
                Err(SysError::Timeout) | Err(SysError::WouldBlock) => {
                    // Timeout fired or spurious wake — poll query clients
                }
                Err(_e) => {
                    uerror!("devd", "wait_failed";);
                }
            }

            // Poll all query clients for pending messages.  Handles channels
            // that could not be added to the Mux due to the 16-slot limit.
            self.poll_query_clients();

            // Poll restart timers via wall-clock comparison.  Catches timers
            // that couldn't be added to the Mux (16-handle limit).
            self.poll_restart_timers();

            // Flush structured logs so they appear on the console
            userlib::ulog::flush();
        }
    }
}

// =============================================================================
// Main
// =============================================================================

static mut DEVD: Devd = Devd::new();

#[unsafe(no_mangle)]
#[allow(static_mut_refs)]
fn main() -> ! {
    userlib::io::disable_stdout();

    let devd = unsafe { &mut DEVD };

    if let Err(e) = devd.init() {
        uerror!("devd", "init_failed";);
        syscall::exit(1);
    }

    uinfo!("devd", "started"; services = devd.services.count() as u32);
    userlib::ulog::flush();
    devd.run()
}
