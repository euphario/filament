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

mod mounts;
mod service;
mod ports;
mod process;
mod query;
mod rules;

use userlib::syscall;
use userlib::{uinfo, unotice, uwarn, uerror, udebug};
use userlib::ipc::{Port, Channel, EventLoop, ObjHandle, Mux, MuxFilter};
use userlib::syscall::Handle;
use userlib::error::{SysError, SysResult};
use userlib::query::{
    QueryHeader, SpawnChildContext, msg, query_flags,
};

use service::{
    ServiceState, ServiceManager, ServiceRegistry,
    MAX_SERVICES, MAX_RESTARTS,
    INITIAL_BACKOFF_MS, MAX_BACKOFF_MS, FAILED_RETRY_MS,
};
use ports::{PortRegistry, Ports};
use process::{ProcessManager, SyscallProcessManager};
use query::{QueryHandler, MSG_BUFFER_SIZE, MAX_QUERY_CLIENTS};

/// Copy a static byte slice into a buffer, returning the number of bytes copied.
fn copy_static(dst: &mut [u8], src: &[u8]) -> usize {
    let len = src.len().min(dst.len());
    dst[..len].copy_from_slice(&src[..len]);
    len
}


/// Append config response data to buf at `pos`, annotating with source path.
///
/// Builds full paths through the supervision tree:
/// - Existing `[path]` headers in `data` are rewritten to `[prefix/path]`
/// - Raw key=value lines before any header get a new `[prefix]` header prepended
/// - If `prefix` is empty, data is copied as-is
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
    let mut prepended = false;
    let mut i = 0;

    while i < data.len() && out < cap {
        let line_end = data[i..].iter().position(|&b| b == b'\n')
            .map(|p| i + p + 1)
            .unwrap_or(data.len());
        let line = &data[i..line_end];
        let trimmed_end = if line.ends_with(b"\n") { line.len() - 1 } else { line.len() };
        let trimmed = &line[..trimmed_end];

        if trimmed.starts_with(b"[") && trimmed.ends_with(b"]") && trimmed.len() > 2 {
            let old_path = &trimmed[1..trimmed.len() - 1];
            if old_path == prefix {
                // Driver's own [name] header matches service prefix — keep as-is
                let n = line.len().min(cap - out);
                if n > 0 {
                    buf[out..out + n].copy_from_slice(&line[..n]);
                    out += n;
                }
            } else {
                let needed = 1 + prefix.len() + 1 + old_path.len() + 2;
                if out + needed <= cap {
                    buf[out] = b'['; out += 1;
                    buf[out..out + prefix.len()].copy_from_slice(prefix); out += prefix.len();
                    buf[out] = b'/'; out += 1;
                    buf[out..out + old_path.len()].copy_from_slice(old_path); out += old_path.len();
                    buf[out] = b']'; out += 1;
                    buf[out] = b'\n'; out += 1;
                }
            }
            prepended = true;
        } else if !trimmed.is_empty() {
            if !prepended {
                let hdr = 1 + prefix.len() + 2;
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

/// Check if a class name matches a registered port.
fn class_name_matches(name: &[u8], port: &ports::RegisteredPort) -> bool {
    match name {
        b"wifi" => port.port_class() == abi::PortClass::Network
            && port.port_subclass() == abi::port_subclass::NET_WIFI,
        b"nvme" => port.port_subclass() == abi::port_subclass::STORAGE_NVME,
        b"block" => port.port_class() == abi::PortClass::Block,
        b"net" | b"network" => port.port_class() == abi::PortClass::Network,
        b"usb" => port.port_class() == abi::PortClass::Usb,
        b"pcie" => port.port_class() == abi::PortClass::Pcie,
        b"fs" | b"filesystem" => port.port_class() == abi::PortClass::Filesystem,
        b"console" => port.port_class() == abi::PortClass::Console,
        b"storage" => port.port_class() == abi::PortClass::StorageController,
        b"ethernet" => port.port_class() == abi::PortClass::Ethernet,
        _ => false,
    }
}

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

/// Max context key-value pairs per spawn
const MAX_CONTEXT_KV: usize = 4;
/// Max key length for context KV
const MAX_CONTEXT_KEY: usize = 32;
/// Max value length for context KV
const MAX_CONTEXT_VALUE: usize = 64;

/// A single context key-value pair stored with spawn context
#[derive(Clone, Copy)]
struct ContextKv {
    key: [u8; MAX_CONTEXT_KEY],
    key_len: u8,
    value: [u8; MAX_CONTEXT_VALUE],
    value_len: u8,
}

impl ContextKv {
    const fn empty() -> Self {
        Self {
            key: [0u8; MAX_CONTEXT_KEY],
            key_len: 0,
            value: [0u8; MAX_CONTEXT_VALUE],
            value_len: 0,
        }
    }

    fn set(&mut self, key: &[u8], value: &[u8]) {
        let klen = key.len().min(MAX_CONTEXT_KEY);
        self.key[..klen].copy_from_slice(&key[..klen]);
        self.key_len = klen as u8;
        let vlen = value.len().min(MAX_CONTEXT_VALUE);
        self.value[..vlen].copy_from_slice(&value[..vlen]);
        self.value_len = vlen as u8;
    }

    fn key(&self) -> &[u8] {
        &self.key[..self.key_len as usize]
    }

    fn value(&self) -> &[u8] {
        &self.value[..self.value_len as usize]
    }
}

struct SpawnContext {
    /// Child process PID (0 = empty slot)
    pid: u32,
    /// Port type that triggered spawn
    port_type: u8,
    /// Port ID of the trigger port (monotonic, from port registry)
    trigger_port_id: u8,
    /// Trigger port name
    port_name: [u8; 32],
    /// Length of port name
    port_name_len: u8,
    /// Opaque metadata from the port registration (e.g., BAR0 info)
    metadata: [u8; 64],
    /// Length of metadata
    metadata_len: u8,
    /// Context key-value pairs from rule template expansion
    context_kv: [ContextKv; MAX_CONTEXT_KV],
    /// Number of context KV pairs
    context_kv_count: u8,
}

impl SpawnContext {
    const fn empty() -> Self {
        Self {
            pid: 0,
            port_type: 0,
            trigger_port_id: 0xFF,
            port_name: [0u8; 32],
            port_name_len: 0,
            metadata: [0u8; 64],
            metadata_len: 0,
            context_kv: [const { ContextKv::empty() }; MAX_CONTEXT_KV],
            context_kv_count: 0,
        }
    }
}

/// Lookup table for template expansion: up to 4 key→value pairs.
struct TemplateLookup {
    keys: [[u8; 16]; 4],
    key_lens: [u8; 4],
    values: [[u8; 32]; 4],
    value_lens: [u8; 4],
    count: usize,
}

impl TemplateLookup {
    fn new() -> Self {
        Self {
            keys: [[0u8; 16]; 4],
            key_lens: [0; 4],
            values: [[0u8; 32]; 4],
            value_lens: [0; 4],
            count: 0,
        }
    }

    fn add(&mut self, key: &[u8], value: &[u8]) {
        if self.count >= 4 { return; }
        let kl = key.len().min(16);
        self.keys[self.count][..kl].copy_from_slice(&key[..kl]);
        self.key_lens[self.count] = kl as u8;
        let vl = value.len().min(32);
        self.values[self.count][..vl].copy_from_slice(&value[..vl]);
        self.value_lens[self.count] = vl as u8;
        self.count += 1;
    }

    fn get(&self, key: &[u8]) -> Option<&[u8]> {
        for i in 0..self.count {
            if &self.keys[i][..self.key_lens[i] as usize] == key {
                return Some(&self.values[i][..self.value_lens[i] as usize]);
            }
        }
        None
    }
}

/// Expand a template string containing `{key}` placeholders.
///
/// Looks up keys against the provided lookup table.
/// Returns the number of bytes written to `out`.
fn expand_template(template: &[u8], out: &mut [u8], lookup: &TemplateLookup) -> usize {
    let mut wi = 0; // write index into out
    let mut ri = 0; // read index into template

    while ri < template.len() && wi < out.len() {
        if template[ri] == b'{' {
            // Find matching '}'
            if let Some(close) = template[ri + 1..].iter().position(|&b| b == b'}') {
                let key = &template[ri + 1..ri + 1 + close];
                if let Some(val) = lookup.get(key) {
                    let copy_len = val.len().min(out.len() - wi);
                    out[wi..wi + copy_len].copy_from_slice(&val[..copy_len]);
                    wi += copy_len;
                }
                // Skip past the closing '}'
                ri += 1 + close + 1;
            } else {
                // No matching '}' — copy the '{' literally
                out[wi] = template[ri];
                wi += 1;
                ri += 1;
            }
        } else {
            out[wi] = template[ri];
            wi += 1;
            ri += 1;
        }
    }

    wi
}

/// In-flight spawn tracking - maps seq_id to port info for SPAWN_ACK
#[derive(Clone, Copy)]
struct InflightSpawn {
    /// Sequence ID (0 = empty slot)
    seq_id: u32,
    /// Port type that triggered spawn
    port_type: u8,
    /// Port ID of the trigger port (monotonic, from port registry)
    trigger_port_id: u8,
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
    /// Service index of the PARENT driver that received the SpawnChild command
    /// (0xFF = direct spawn by devd, not delegated)
    parent_service_idx: u8,
    /// Capability bits for the spawned process
    caps: u64,
    /// Link ID tying spawn to port (carried through to service slot)
    link_id: u32,
    /// Context KV pairs from rule expansion (carried through SPAWN_ACK)
    context_kv: [ContextKv; MAX_CONTEXT_KV],
    context_kv_count: u8,
}

impl InflightSpawn {
    const fn empty() -> Self {
        Self {
            seq_id: 0,
            port_type: 0,
            trigger_port_id: 0xFF,
            port_name: [0u8; 32],
            port_name_len: 0,
            binary_name: [0u8; 16],
            binary_name_len: 0,
            service_idx: 0xFF,
            parent_service_idx: 0xFF,
            caps: 0,
            link_id: 0,
            context_kv: [const { ContextKv::empty() }; MAX_CONTEXT_KV],
            context_kv_count: 0,
        }
    }
}

// =============================================================================
// Deferred Rule Fire (SuperQ command serialization)
// =============================================================================

/// Maximum deferred rule fires (ports waiting for SuperQ command slot)
const MAX_DEFERRED_RULES: usize = 8;

/// A rule fire deferred because the parent driver's SuperQ was busy.
#[derive(Clone, Copy)]
struct DeferredRuleFire {
    /// Port ID whose rule needs firing
    port_id: u8,
    /// Owner service index of the port
    owner_idx: u8,
    /// Active flag (false = empty slot)
    active: bool,
}

impl DeferredRuleFire {
    const fn empty() -> Self {
        Self { port_id: 0, owner_idx: 0xFF, active: false }
    }
}

// =============================================================================
// Pending Admin Request (unified async tracking for config + info queries)
// =============================================================================

/// Maximum concurrent admin requests (config broadcast, config single, info query).
const MAX_PENDING_REQUESTS: usize = 8;

/// Maximum targets per admin request.
const MAX_REQUEST_TARGETS: usize = 16;

/// A pending admin request forwarded to one or more drivers.
///
/// Unifies PendingConfigBroadcast, blocking config single-service,
/// and PendingInfoQuery into one tracking struct with deadlines on everything.
///
/// - `deadline_ns == 0` → empty slot
/// - EOL convergence: `(eol_mask & sent_mask) == sent_mask` → all targets responded
/// - `client_seq_id != 0` → info query (response gets seq_id rewriting)
struct PendingAdminRequest {
    /// Admin client slot to send response to.
    client_slot: u8,
    /// Original client's sequence ID (nonzero for info queries needing seq_id rewrite).
    client_seq_id: u32,
    /// Message type: CONFIG_GET, CONFIG_SET, or QUERY_SERVICE_INFO.
    msg_type: u16,
    /// Bitmask of targets sent to (bit N = forward_seq_ids[N]).
    sent_mask: u16,
    /// Bitmask of targets that have sent EOL (or channel close).
    eol_mask: u16,
    /// Forward seq_ids used toward each target (0 = unused).
    forward_seq_ids: [u32; MAX_REQUEST_TARGETS],
    /// Accumulated response buffer (unused in relay mode).
    response_buf: [u8; 1024],
    /// Bytes written to response_buf (unused in relay mode).
    response_len: u16,
    /// Deadline in nanoseconds (0 = inactive slot).
    deadline_ns: u64,
    /// Relay mode: send each driver response immediately instead of accumulating.
    relay: bool,
}

/// A target for an admin request: query handler slot + optional address.
#[derive(Clone, Copy)]
struct AdminTarget {
    slot: usize,
    address: [u8; 64],
    address_len: usize,
}

impl AdminTarget {
    const fn empty() -> Self {
        Self { slot: 0, address: [0u8; 64], address_len: 0 }
    }

    fn with_address(slot: usize, address: &[u8]) -> Self {
        let mut t = Self { slot, address: [0u8; 64], address_len: 0 };
        let len = address.len().min(64);
        let dst = &mut t.address[..len];
        dst.copy_from_slice(&address[..len]);
        t.address_len = len;
        t
    }

    /// Build a path address: "/name" for routing through bus_runtime.
    fn with_path_address(slot: usize, name: &[u8]) -> Self {
        let mut t = Self { slot, address: [0u8; 64], address_len: 0 };
        let nlen = name.len().min(63);
        t.address[0] = b'/';
        t.address[1..1 + nlen].copy_from_slice(&name[..nlen]);
        t.address_len = 1 + nlen;
        t
    }
}

impl PendingAdminRequest {
    const fn empty() -> Self {
        Self {
            client_slot: 0,
            client_seq_id: 0,
            msg_type: 0,
            sent_mask: 0,
            eol_mask: 0,
            forward_seq_ids: [0; MAX_REQUEST_TARGETS],
            response_buf: [0u8; 1024],
            response_len: 0,
            deadline_ns: 0,
            relay: false,
        }
    }

    fn is_active(&self) -> bool {
        self.deadline_ns != 0
    }

    fn is_converged(&self) -> bool {
        (self.eol_mask & self.sent_mask) == self.sent_mask
    }

    fn matches_seq(&self, seq_id: u32) -> bool {
        seq_id != 0 && self.forward_seq_ids.iter().any(|&s| s == seq_id)
    }

    /// Find the bit index for a given seq_id in forward_seq_ids.
    fn seq_bit_index(&self, seq_id: u32) -> Option<usize> {
        self.forward_seq_ids.iter().position(|&s| s == seq_id)
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
    /// Mount table: path prefix → transport
    mounts: mounts::MountTable,
    /// Process manager
    process_mgr: SyscallProcessManager,
    /// Query handler for client connections
    query_handler: QueryHandler,
    /// Recently spawned dynamic PIDs (workaround for spawn-before-slot-setup race)
    /// Maps PID -> service_idx for dynamic drivers that haven't connected yet
    recent_dynamic_pids: [(u32, u8); MAX_RECENT_DYNAMIC_PIDS],
    /// Spawn context: maps child PID to trigger port (for GET_SPAWN_CONTEXT)
    spawn_contexts: [SpawnContext; MAX_SPAWN_CONTEXTS],
    /// In-flight spawn commands: maps seq_id to port info
    inflight_spawns: [InflightSpawn; MAX_INFLIGHT_SPAWNS],
    /// Pending admin requests: unified tracking for config + info queries
    pending_requests: [PendingAdminRequest; MAX_PENDING_REQUESTS],
    /// Next sequence ID for forwarded queries
    next_forward_seq_id: u32,
    /// Log ring buffer
    log_buffer: LogBuffer,
    /// Whether live logging is enabled (print to console as received)
    live_logging: bool,
    /// Next link ID for port↔service pairing (0 = sentinel, starts at 1)
    next_link_id: u32,
    /// Pending context KV pairs for next spawn (set by check_class_rules, consumed by spawn_service)
    pending_context_kvs: [ContextKv; MAX_CONTEXT_KV],
    pending_context_kv_count: u8,
    /// Deferred rule fires: ports whose SpawnChild couldn't be sent because
    /// the parent driver's SuperQ already had a command in-flight.
    deferred_rules: [DeferredRuleFire; MAX_DEFERRED_RULES],
    /// Bitmask of query client slots that failed to be added to the Mux
    /// (bit N = slot N needs polling because watch() failed)
    overflow_query_mask: u16,
}

impl Devd {
    pub const fn new() -> Self {
        Self {
            query_port: None,
            events: None,
            services: ServiceRegistry::new(),
            ports: Ports::new(),
            mounts: mounts::MountTable::new(),
            process_mgr: SyscallProcessManager::new(),
            query_handler: QueryHandler::new(),
            recent_dynamic_pids: [(0, 0); MAX_RECENT_DYNAMIC_PIDS],
            spawn_contexts: [const { SpawnContext::empty() }; MAX_SPAWN_CONTEXTS],
            inflight_spawns: [const { InflightSpawn::empty() }; MAX_INFLIGHT_SPAWNS],
            pending_requests: [const { PendingAdminRequest::empty() }; MAX_PENDING_REQUESTS],
            next_forward_seq_id: 1,
            log_buffer: LogBuffer::new(),
            live_logging: false, // Off during boot, enable after all services ready
            next_link_id: 1,
            pending_context_kvs: [const { ContextKv::empty() }; MAX_CONTEXT_KV],
            pending_context_kv_count: 0,
            deferred_rules: [const { DeferredRuleFire::empty() }; MAX_DEFERRED_RULES],
            overflow_query_mask: 0,
        }
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    fn now_ms() -> u64 {
        syscall::gettime() / 1_000_000
    }

    /// Whether any state requires periodic polling (overflow clients only).
    /// Restart timers and admin request timeouts are handled by Mux inline timers.
    fn needs_polling(&self) -> bool {
        self.overflow_query_mask != 0
    }

    /// Recalculate the Mux timeout based on whether polling is needed.
    /// 100ms when polling is needed, 0 (block forever) otherwise.
    fn update_timeout(&mut self) {
        if let Some(events) = &self.events {
            let timeout = if self.needs_polling() { 100 } else { 0 };
            let _ = events.set_timeout(timeout);
        }
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
        udebug!("devd", "query_port_init"; handle = query_port.handle().raw());

        self.query_port = Some(query_port);
        self.events = Some(events);

        // Register devd's service port in the port registry
        let devd_info = abi::PortInfo::new(b"devd-query:", abi::PortClass::Service);
        if let Err(e) = self.ports.register_with_port_info(&devd_info, 0xFF, 0) {
            uerror!("devd", "port_reg_failed"; name = "devd-query:", err = e.to_errno());
        }

        // Discover kernel buses — rules fire on Ready, spawning bus drivers
        self.discover_kernel_buses();
        #[cfg(feature = "stress-test")]
        self.create_and_spawn("testr", u64::MAX, abi::priority::INHERIT, &[], 0);

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
        let mut buses = [abi::PortInfo::empty(); 16];
        let count = match userlib::ipc::bus_list(&mut buses) {
            Ok(n) => n,
            Err(e) => {
                uerror!("devd", "bus_list_failed"; err = e.to_errno());
                return;
            }
        };

        unotice!("devd", "bus_discovery"; count = count as u32);

        for i in 0..count {
            let port_info = &buses[i];
            let path = &port_info.name[..port_info.name_len as usize];

            // Skip unknown/service ports (e.g., Platform bus type)
            if port_info.port_class == abi::PortClass::Unknown || port_info.port_class == abi::PortClass::Service {
                continue;
            }

            // Register in devd's port registry (owner = 0xFF = devd/kernel)
            if let Err(e) = self.ports.register_with_port_info(port_info, 0xFF, 0) {
                uerror!("devd", "bus_port_reg_failed";
                    name = core::str::from_utf8(path).unwrap_or("?"), err = e.to_errno());
                continue;
            }

            udebug!("devd", "bus_port_registered";
                name = core::str::from_utf8(path).unwrap_or("?"),
                class = port_info.port_class as u16
            );

            // Set port to Claimed — kernel bus ports are managed by the kernel,
            // so they're immediately available. Fire rules on the transition.
            let pid = self.ports.get_port_id(path).unwrap_or(0xFF);
            let old_state = self.ports.set_state(path, abi::PortState::Claimed);
            match old_state {
                Some(old) => {
                    uinfo!("devd", "port_transition";
                        port = core::str::from_utf8(path).unwrap_or("?"),
                        from = old.as_str(),
                        to = abi::PortState::Claimed.as_str()
                    );
                }
                None => {
                    uwarn!("devd", "bus_port_state_failed";
                        name = core::str::from_utf8(path).unwrap_or("?"));
                }
            }

            // Fire rules on the Claimed transition
            self.check_class_rules(port_info, 0xFF, pid);
        }
    }

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
        let (name_buf, name_len, caps, priority, trigger_port_buf, trigger_port_len) = {
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

            (nb, nl, service.caps, service.priority, tp, tpl)
        };

        let binary = core::str::from_utf8(&name_buf[..name_len]).unwrap_or("???");

        // Gather spawn context for the birth mailbox
        let trigger_port = &trigger_port_buf[..trigger_port_len];
        let (port_type, metadata_buf, metadata_len, trig_port_id) = if trigger_port_len > 0 {
            let pt = self.ports.get_port_type(trigger_port).unwrap_or(0);
            let pid_val = self.ports.get_port_id(trigger_port).unwrap_or(0xFF);
            let md: ([u8; 64], usize) = self.ports.get(trigger_port)
                .map(|p| {
                    let m = p.metadata();
                    let mut buf = [0u8; 64];
                    let len = m.len().min(64);
                    buf[..len].copy_from_slice(&m[..len]);
                    (buf, len)
                })
                .unwrap_or(([0u8; 64], 0));
            (pt, md.0, md.1, pid_val)
        } else {
            (0u8, [0u8; 64], 0usize, 0xFF)
        };

        // Collect pending context KV pairs from rule expansion
        let kv_count = self.pending_context_kv_count as usize;
        let pending_kvs = self.pending_context_kvs;
        self.pending_context_kv_count = 0;

        // Build mailbox page with spawn context
        let mut mb_buf = [0u8; 4096];
        Self::build_spawn_mailbox(
            &mut mb_buf, port_type, trig_port_id, trigger_port,
            &metadata_buf[..metadata_len], &pending_kvs[..kv_count], priority,
        );

        // Spawn with exec_with_mailbox — parent gets shmem handle + superq handle back
        let (pid, parent_mb_handle, parent_superq_handle) = match self.process_mgr.spawn_with_caps(binary, caps, &mb_buf) {
            Ok(result) => result,
            Err(_) => {
                uerror!("devd", "svc_spawn_failed"; binary = binary);
                self.services.transition(idx, ServiceState::Failed { code: -1 }, now);
                return;
            }
        };

        // Map the mailbox shmem for parent-side access
        let mb_addr = match userlib::syscall::map(parent_mb_handle, 0) {
            Ok(addr) if addr != 0 => addr,
            _ => {
                uwarn!("devd", "svc_mailbox_map_failed"; pid = pid);
                0
            }
        };

        // Add supervision-based client (SuperQ for bidirectional communication).
        // Watch the SuperQ handle in our EventLoop's Mux so we get woken
        // when the child sends a message — no more scanning/polling.
        if mb_addr != 0 {
            let superq = userlib::SupervisionHandle::from_handle(parent_superq_handle);
            let superq_handle = superq.handle();
            match self.query_handler.add_supervision_client(
                superq, idx as u8, pid,
            ) {
                Some(_slot) => {
                    // Register SuperQ handle in EventLoop Mux for event-driven dispatch
                    if let Some(events) = &mut self.events {
                        let _ = events.watch(superq_handle);
                    }
                }
                None => {
                    uwarn!("devd", "no_query_slot"; pid = pid);
                }
            }
        }

        // Store spawn context for GET_SPAWN_CONTEXT queries from the child
        // (kept for compatibility — child can also read from mailbox directly)
        let mut kv_refs: [(&[u8], &[u8]); MAX_CONTEXT_KV] = [(&[], &[]); MAX_CONTEXT_KV];
        for i in 0..kv_count {
            kv_refs[i] = (pending_kvs[i].key(), pending_kvs[i].value());
        }
        self.store_spawn_context_with_kv(
            pid, port_type, trigger_port, &metadata_buf[..metadata_len], trig_port_id,
            &kv_refs[..kv_count],
        );

        // Set up exception channel so child faults produce a diagnostic log
        // instead of silent death. The kernel freezes the child and sends
        // ExceptionInfo on the channel; devd reads it, logs, then kills.
        let mut exc_handle_raw = 0u32;
        let mut exc_child_raw = 0u32;
        if let Ok((exc_a, exc_b)) = Channel::pair() {
            let child_handle = exc_b.into_raw_handle();
            if syscall::set_exception_channel(pid, child_handle).is_ok() {
                let h = exc_a.handle();
                if let Some(events) = &mut self.events {
                    let _ = events.watch(h);
                }
                exc_handle_raw = h.raw();
                exc_child_raw = child_handle.raw();
            }
            // Keep BOTH channel ends alive:
            // - exc_a: watched in Mux for fault notifications
            // - exc_b (child_handle): kept so the channel stays Open;
            //   closing it would HalfClose the channel, firing a spurious
            //   CLOSED event on exc_a and preventing the kernel from
            //   delivering ExceptionInfo via send(channel_id).
            core::mem::forget(exc_a);
        }

        // Set per-child resource limits based on driver presets
        let preset = rules::resource_preset(binary);
        let _ = syscall::set_resource_limits(
            pid, preset.max_channels, preset.max_ports,
            preset.max_shmem, preset.max_children,
        );

        // Update service state
        if let Some(service) = self.services.get_mut(idx) {
            service.pid = pid;
            service.state = ServiceState::Starting;
            service.last_change = now;
            service.exc_channel = exc_handle_raw;
            service.exc_channel_child = exc_child_raw;
        }

        self.add_recent_dynamic_pid(pid, idx as u8);

        unotice!("devd", "svc_spawned"; binary = binary, pid = pid);
    }

    /// Build a mailbox page with spawn context for a child service.
    fn build_spawn_mailbox(
        buf: &mut [u8; 4096],
        port_type: u8,
        port_id: u8,
        trigger_port: &[u8],
        metadata: &[u8],
        kvs: &[ContextKv],
        priority: u8,
    ) {
        buf.fill(0);

        let mut hdr = abi::MailboxHeader::empty();
        hdr.bus_type = port_type;
        hdr.bus_index = port_id;
        hdr.priority = priority;

        // Write KVs starting at offset 64 (after header)
        let mut kv_count: u8 = 0;
        let mut pos: usize = 64;

        // KV: port.name = trigger_port
        if !trigger_port.is_empty() {
            let key = b"port.name";
            let klen = key.len();
            let vlen = trigger_port.len().min(64);
            if pos + 1 + klen + 1 + vlen < 2048 {
                buf[pos] = klen as u8; pos += 1;
                buf[pos..pos + klen].copy_from_slice(key); pos += klen;
                buf[pos] = vlen as u8; pos += 1;
                buf[pos..pos + vlen].copy_from_slice(&trigger_port[..vlen]); pos += vlen;
                kv_count += 1;
            }
        }

        // KV: port.metadata
        if !metadata.is_empty() {
            let key = b"port.metadata";
            let klen = key.len();
            let mlen = metadata.len().min(64);
            if pos + 1 + klen + 1 + mlen < 2048 {
                buf[pos] = klen as u8; pos += 1;
                buf[pos..pos + klen].copy_from_slice(key); pos += klen;
                buf[pos] = mlen as u8; pos += 1;
                buf[pos..pos + mlen].copy_from_slice(&metadata[..mlen]); pos += mlen;
                kv_count += 1;
            }
        }

        // Context template KVs from rule expansion
        for kv in kvs {
            let klen = kv.key_len as usize;
            let vlen = kv.value_len as usize;
            if klen == 0 { continue; }
            if pos + 1 + klen + 1 + vlen < 2048 {
                buf[pos] = klen as u8; pos += 1;
                buf[pos..pos + klen].copy_from_slice(&kv.key[..klen]); pos += klen;
                buf[pos] = vlen as u8; pos += 1;
                buf[pos..pos + vlen].copy_from_slice(&kv.value[..vlen]); pos += vlen;
                kv_count += 1;
            }
        }

        hdr.kv_count = kv_count;

        // Write header to buf
        let hdr_bytes = unsafe {
            core::slice::from_raw_parts(&hdr as *const abi::MailboxHeader as *const u8, 64)
        };
        buf[..64].copy_from_slice(hdr_bytes);
    }

    /// Create a service slot and spawn.  Used by check_class_rules for port-triggered
    /// drivers and by init for boot services.
    fn create_and_spawn(&mut self, binary: &str, caps: u64, priority: u8, trigger_port: &[u8], link_id: u32) {
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
            service.priority = priority;
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

            // Remove query client for this service (SuperQ or channel)
            if let Some(qslot) = self.query_handler.find_by_service_idx(idx as u8) {
                // Unwatch from event loop if channel-based
                if let Some(handle) = self.query_handler.get(qslot).and_then(|c| c.handle()) {
                    if let Some(events) = &mut self.events {
                        let _ = events.unwatch(handle);
                    }
                }
                self.query_handler.remove_client(qslot);
            }

            // Close exception channel for this service (both ends)
            if service.exc_channel != 0 {
                let h = Handle(service.exc_channel);
                if let Some(events) = &mut self.events {
                    let _ = events.unwatch(h);
                }
                let _ = syscall::close(h);
                service.exc_channel = 0;
            }
            if service.exc_channel_child != 0 {
                let _ = syscall::close(Handle(service.exc_channel_child));
                service.exc_channel_child = 0;
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
            self.mounts.remove_by_owner(old_pid);
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

        // Kill child services whose link_id matches a removed port's child_link_id.
        // Example: consoled owns "console:0" port with child_link_id=5 → shell has link_id=5.
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
                // Supervision channel close will trigger cleanup via handle_service_exit
            }
        }

        // Create restart timer if needed (inline timer in Mux — tag = service index)
        match action {
            ExitAction::Crashed { backoff_ms } => {
                let deadline_ns = (backoff_ms as u64) * 1_000_000;
                if let Some(events) = &self.events {
                    if events.add_timer(idx as u32, deadline_ns).is_err() {
                        uwarn!("devd", "restart_timer_add_failed"; idx = idx as u32);
                    }
                }
                if let Some(service) = self.services.get_mut(idx) {
                    service.has_restart_timer = true;
                }
            }
            ExitAction::Failed => {
                let deadline_ns = FAILED_RETRY_MS * 1_000_000;
                if let Some(events) = &self.events {
                    if events.add_timer(idx as u32, deadline_ns).is_err() {
                        uwarn!("devd", "restart_timer_add_failed"; idx = idx as u32);
                    }
                }
                if let Some(service) = self.services.get_mut(idx) {
                    service.has_restart_timer = true;
                }
            }
            ExitAction::Stopped => {}
        }
    }

    // =========================================================================
    // Admin Client Handling (shell text commands)
    // =========================================================================

    fn handle_admin_command(&mut self, cmd: &[u8], resp: &mut [u8], from_slot: usize) -> usize {
        if cmd.is_empty() || resp.is_empty() {
            return 0;
        }
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
            return self.admin_config_command(args, resp, from_slot);
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
        // Supervision channel close will trigger cleanup via handle_service_exit
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

        // Spawn via create_and_spawn (creates service slot + spawns)
        self.create_and_spawn(binary, 0, abi::priority::INHERIT, port.unwrap_or(b""), 0);
        b"OK\n"
    }

    /// Handle CONFIG admin command: forward to driver, wait for response.
    ///
    /// Format: `<service> GET [key]` or `<service> SET <key> <value>`
    /// Broadcast: `GET [key]` or `SET <key> <value>` (no service name)
    /// Address: `<address> GET [key]` or `<address> SET <key> <value>`
    ///   - Path:  `/pcie:0/nvme:0 GET channel`
    ///   - Class: `@wifi GET`
    /// Returns number of bytes written to resp buffer.
    fn admin_config_command(&mut self, args: &[u8], resp: &mut [u8], from_slot: usize) -> usize {
        let args_str = match core::str::from_utf8(args) {
            Ok(s) => s,
            Err(_) => return copy_static(resp, b"ERR invalid args\n"),
        };

        let mut parts = args_str.splitn(4, ' ');
        let first = match parts.next() {
            Some(s) if !s.is_empty() => s,
            _ => return copy_static(resp, b"ERR missing args\n"),
        };

        // Detect broadcast: first token is operation, not service name
        if matches!(first, "GET" | "get" | "SET" | "set") {
            if matches!(first, "GET" | "get") {
                let key = parts.next().unwrap_or("");
                // Key already consumed — broadcast directly
                return self.admin_config_broadcast_inner(msg::CONFIG_GET, key.as_bytes(), &[], resp, from_slot);
            }
            return self.admin_config_broadcast(first, parts, resp, from_slot);
        }

        // Address mode: first token starts with '/' (path) or '@' (class)
        if first.starts_with('/') || first.starts_with('@') {
            return self.admin_config_addressed(first, parts, resp, from_slot);
        }

        // Tree path mode: service/child/path (e.g. pcied/00:01.0:xhci)
        // The sub-route is forwarded through the supervision tree.
        if let Some(slash_pos) = first.find('/') {
            let service_name = &first[..slash_pos];
            let sub_route = &first[slash_pos..]; // includes leading '/'
            let operation = match parts.next() {
                Some(s) => s,
                None => return copy_static(resp, b"ERR missing operation (GET/SET)\n"),
            };
            return self.admin_config_tree_path(service_name, sub_route, operation, parts, resp, from_slot);
        }

        // Service name mode (existing behavior)
        let service_name = first;
        let operation = match parts.next() {
            Some(s) => s,
            None => return copy_static(resp, b"ERR missing operation (GET/SET)\n"),
        };

        self.admin_config_single_service(service_name, operation, parts, resp, from_slot)
    }

    /// Handle CONFIG for a named service (async via PendingAdminRequest).
    ///
    /// Finds ALL services matching the name. Direct-channel services get a
    /// self-addressed query (local-only, no child forwarding). Tree children
    /// get routed through their parent with `/trigger_port` addressing.
    fn admin_config_single_service(
        &mut self,
        service_name: &str,
        operation: &str,
        mut parts: core::str::SplitN<'_, char>,
        resp: &mut [u8],
        from_slot: usize,
    ) -> usize {
        // Collect all matching services
        let mut admin_targets = [AdminTarget::empty(); MAX_REQUEST_TARGETS];
        let mut target_count = 0usize;

        for idx in 0..self.services.count() {
            let svc = match self.services.get(idx) {
                Some(s) if s.name() == service_name && s.state.is_running() => s,
                _ => continue,
            };

            // Determine driver slot and address for this service
            if let Some(slot) = self.query_handler.find_by_service_idx(idx as u8) {
                // Direct channel: self-address for local-only (no child forwarding)
                let name = self.ctx_name(idx);
                if target_count < MAX_REQUEST_TARGETS {
                    admin_targets[target_count] = AdminTarget::with_path_address(slot, &name.0[..name.1]);
                    target_count += 1;
                }
            } else {
                // Tree child: route through parent with /trigger_port address
                let tp = svc.trigger_port();
                if tp.is_empty() { continue; }
                let owner = match self.ports.get(tp).map(|p| p.owner()) {
                    Some(o) => o,
                    None => continue,
                };
                let parent_slot = match self.query_handler.find_by_service_idx(owner) {
                    Some(s) => s,
                    None => continue,
                };
                if target_count < MAX_REQUEST_TARGETS {
                    admin_targets[target_count] = AdminTarget::with_path_address(parent_slot, tp);
                    target_count += 1;
                }
            }
        }

        if target_count == 0 {
            return copy_static(resp, b"ERR service not found\n");
        }

        let (msg_type, key, value) = match operation {
            "GET" | "get" => {
                let key = parts.next().unwrap_or("");
                (msg::CONFIG_GET, key, "")
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
                (msg::CONFIG_SET, key, value)
            }
            _ => return copy_static(resp, b"ERR unknown operation (use GET or SET)\n"),
        };

        if !self.start_admin_request_multi(from_slot as u8, 0, msg_type, &admin_targets[..target_count], key.as_bytes(), value.as_bytes()) {
            return copy_static(resp, b"ERR send failed\n");
        }

        // Return 0 — response sent asynchronously when driver responds or timeout
        0
    }

    /// Handle tree-path CONFIG command (e.g. "pcied/00:01.0:xhci get count").
    ///
    /// Splits into service name + sub-route. Finds the service's direct query
    /// channel and sends with the sub-route as an ADDRESSED path, which
    /// bus_runtime forwards through the supervision tree.
    fn admin_config_tree_path(
        &mut self,
        service_name: &str,
        sub_route: &str,
        operation: &str,
        mut parts: core::str::SplitN<'_, char>,
        resp: &mut [u8],
        from_slot: usize,
    ) -> usize {
        // Find direct-channel service matching the name
        let mut admin_targets = [AdminTarget::empty(); MAX_REQUEST_TARGETS];
        let mut target_count = 0usize;

        for idx in 0..self.services.count() {
            let svc = match self.services.get(idx) {
                Some(s) if s.name() == service_name && s.state.is_running() => s,
                _ => continue,
            };

            // Only target direct-channel services (root of the tree)
            if let Some(slot) = self.query_handler.find_by_service_idx(idx as u8) {
                if target_count < MAX_REQUEST_TARGETS {
                    admin_targets[target_count] = AdminTarget::with_address(slot, sub_route.as_bytes());
                    target_count += 1;
                }
            }
        }

        if target_count == 0 {
            return copy_static(resp, b"ERR service not found\n");
        }

        let (msg_type, key, value) = match operation {
            "GET" | "get" => {
                let key = parts.next().unwrap_or("");
                (msg::CONFIG_GET, key, "")
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
                (msg::CONFIG_SET, key, value)
            }
            _ => return copy_static(resp, b"ERR unknown operation (use GET or SET)\n"),
        };

        if !self.start_admin_request_multi(from_slot as u8, 0, msg_type, &admin_targets[..target_count], key.as_bytes(), value.as_bytes()) {
            return copy_static(resp, b"ERR send failed\n");
        }

        0
    }

    /// Get the bus_runtime name for a service (used as self-address).
    fn ctx_name(&self, service_idx: usize) -> ([u8; 32], usize) {
        // The bus_runtime name is the service's dynamic_name (e.g. "pcied", "consoled")
        if let Some(svc) = self.services.get(service_idx) {
            let name = svc.name();
            let mut buf = [0u8; 32];
            let len = name.len().min(32);
            buf[..len].copy_from_slice(&name.as_bytes()[..len]);
            (buf, len)
        } else {
            ([0u8; 32], 0)
        }
    }

    /// Handle addressed CONFIG command (/path or @class routing).
    fn admin_config_addressed(
        &mut self,
        address: &str,
        mut parts: core::str::SplitN<'_, char>,
        resp: &mut [u8],
        from_slot: usize,
    ) -> usize {
        let operation = match parts.next() {
            Some(s) => s,
            None => return copy_static(resp, b"ERR missing operation (GET/SET)\n"),
        };

        if matches!(operation, "GET" | "get") {
            let key = parts.next().unwrap_or("");
            // Find target services by address, then broadcast to them
            return self.send_addressed_query(address, operation, key, "", resp, from_slot);
        }

        if matches!(operation, "SET" | "set") {
            let key = match parts.next() {
                Some(k) => k,
                None => return copy_static(resp, b"ERR missing key\n"),
            };
            let value = match parts.next() {
                Some(v) => v,
                None => return copy_static(resp, b"ERR missing value\n"),
            };
            return self.send_addressed_query(address, operation, key, value, resp, from_slot);
        }

        copy_static(resp, b"ERR unknown operation (use GET or SET)\n")
    }

    /// Find service indices matching an address and send config queries (async).
    ///
    /// Path mode (`/pcie:0/nvme:0/block:0/partd`): parse first segment as root
    /// port, find owning service, forward remaining path as ADDRESSED route.
    /// Class mode (`@wifi`): find all matching services, broadcast.
    fn send_addressed_query(
        &mut self,
        address: &str,
        operation: &str,
        key: &str,
        value: &str,
        resp: &mut [u8],
        from_slot: usize,
    ) -> usize {
        let msg_type = if matches!(operation, "GET" | "get") {
            msg::CONFIG_GET
        } else {
            msg::CONFIG_SET
        };

        if address.starts_with('/') {
            // Path mode: split first segment (root port) from remaining route
            let path = &address[1..]; // strip leading '/'
            let (first_seg, remaining) = match path.find('/') {
                Some(pos) => (&path[..pos], &path[pos + 1..]),
                None => (path, ""),
            };

            if first_seg.is_empty() {
                return copy_static(resp, b"ERR empty path segment\n");
            }

            // First segment should contain ':' (port name) — find owning service
            let mut target_slot = None;
            for idx in 0..self.services.count() {
                let svc = match self.services.get(idx) {
                    Some(s) if s.state.is_running() => s,
                    _ => continue,
                };
                let tp = svc.trigger_port();
                let seg_bytes = first_seg.as_bytes();
                // Match: trigger_port ends with the first segment
                // e.g., "/pcie:0" ends with "pcie:0"
                if tp.len() >= seg_bytes.len() {
                    let suffix = &tp[tp.len() - seg_bytes.len()..];
                    if suffix == seg_bytes {
                        // Verify it's preceded by '/' or is the full path
                        let preceded_by_slash = tp.len() > seg_bytes.len()
                            && tp[tp.len() - seg_bytes.len() - 1] == b'/';
                        if tp.len() == seg_bytes.len() || preceded_by_slash {
                            if let Some(slot) = self.query_handler.find_by_service_idx(idx as u8) {
                                target_slot = Some(slot);
                                break;
                            }
                        }
                    }
                }
            }

            let slot = match target_slot {
                Some(s) => s,
                None => return copy_static(resp, b"ERR no matching service\n"),
            };

            // Build AdminTarget with remaining route as address
            let mut admin_targets = [AdminTarget::empty(); 1];
            admin_targets[0] = AdminTarget::with_address(slot, remaining.as_bytes());

            if !self.start_admin_request_multi(from_slot as u8, 0, msg_type, &admin_targets[..1], key.as_bytes(), value.as_bytes()) {
                return copy_static(resp, b"ERR send failed\n");
            }
            return 0;
        }

        if address.starts_with('@') {
            // Class mode: find all matching services
            let (targets, count) = self.find_services_by_class(address.as_bytes());
            if count == 0 {
                return copy_static(resp, b"ERR no matching service\n");
            }
            if !self.start_admin_request(from_slot as u8, 0, msg_type, &targets[..count], key.as_bytes(), value.as_bytes(), &[]) {
                return copy_static(resp, b"ERR send failed\n");
            }
            return 0;
        }

        copy_static(resp, b"ERR invalid address format\n")
    }

    /// Find query client slots matching a class address (@wifi, @storage, etc.)
    fn find_services_by_class(&self, address: &[u8]) -> ([usize; 16], usize) {
        let mut slots = [0usize; 16];
        let mut count = 0;

        if address.starts_with(b"@") {
            let class_name = &address[1..];
            self.ports.for_each(|port| {
                if count >= 16 { return; }
                if class_name_matches(class_name, port) {
                    let owner = port.owner();
                    if let Some(slot) = self.query_handler.find_by_service_idx(owner) {
                        // Deduplicate
                        if !slots[..count].contains(&slot) {
                            slots[count] = slot;
                            count += 1;
                        }
                    }
                }
            });
        }

        (slots, count)
    }

    /// Broadcast a CONFIG query to all managed drivers (async via PendingAdminRequest).
    fn admin_config_broadcast_inner(
        &mut self,
        msg_type: u16,
        key: &[u8],
        value: &[u8],
        resp: &mut [u8],
        from_slot: usize,
    ) -> usize {
        // Collect managed driver slots
        let mut targets = [0usize; MAX_REQUEST_TARGETS];
        let mut count = 0;
        for slot in 0..MAX_QUERY_CLIENTS {
            if slot == from_slot { continue; }
            let is_managed = match self.query_handler.get(slot) {
                Some(c) if c.is_managed => true,
                _ => continue,
            };
            if !is_managed { continue; }
            if count < MAX_REQUEST_TARGETS {
                targets[count] = slot;
                count += 1;
            }
        }

        if count == 0 {
            return copy_static(resp, b"ERR no drivers connected\n");
        }

        if !self.start_admin_request(from_slot as u8, 0, msg_type, &targets[..count], key, value, &[]) {
            return copy_static(resp, b"ERR send failed\n");
        }

        0
    }

    /// Send a CONFIG_GET or CONFIG_SET query to a driver via its query channel.
    ///
    /// When `address` is non-empty, sets ADDRESSED flag and prepends
    /// route_len + route before the key payload.
    fn send_config_query(
        &mut self,
        driver_slot: usize,
        seq_id: u32,
        msg_type: u16,
        key: &[u8],
        value: &[u8],
        address: &[u8],
    ) -> bool {
        let mut buf = [0u8; 256];

        // Build QueryHeader
        let mut header = QueryHeader::new(msg_type, seq_id);
        if !address.is_empty() {
            header.flags |= query_flags::ADDRESSED;
        }
        let header_bytes = header.to_bytes();
        buf[..QueryHeader::SIZE].copy_from_slice(&header_bytes);

        let mut offset = QueryHeader::SIZE;

        // Write address route if present
        if !address.is_empty() {
            let alen = address.len().min(255);
            buf[offset] = alen as u8;
            offset += 1;
            buf[offset..offset + alen].copy_from_slice(&address[..alen]);
            offset += alen;
        }

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
            client.send(&buf[..offset]).is_ok()
        } else {
            false
        }
    }

    /// Broadcast a CONFIG command to all connected managed drivers (async).
    ///
    /// Parses operation/key/value from remaining args and delegates to
    /// admin_config_broadcast_inner via PendingAdminRequest.
    fn admin_config_broadcast(
        &mut self,
        operation: &str,
        mut parts: core::str::SplitN<'_, char>,
        resp: &mut [u8],
        from_slot: usize,
    ) -> usize {
        let msg_type;
        let key;
        let value;

        match operation {
            "GET" | "get" => {
                msg_type = msg::CONFIG_GET;
                key = parts.next().unwrap_or("");
                value = "";
            }
            "SET" | "set" => {
                msg_type = msg::CONFIG_SET;
                key = match parts.next() {
                    Some(k) => k,
                    None => return copy_static(resp, b"ERR missing key\n"),
                };
                value = match parts.next() {
                    Some(v) => v,
                    None => return copy_static(resp, b"ERR missing value\n"),
                };
            }
            _ => return copy_static(resp, b"ERR unknown operation\n"),
        }

        self.admin_config_broadcast_inner(msg_type, key.as_bytes(), value.as_bytes(), resp, from_slot)
    }

    /// Allocate a pending admin request slot, send queries to targets, set deadline.
    ///
    /// `client_seq_id` is nonzero for info queries (response gets seq_id rewriting).
    /// `address` is the routing address for ADDRESSED queries (empty = broadcast).
    /// Returns true if the request was started, false if no slots or no sends succeeded.
    /// Start an admin request with per-target addresses.
    ///
    /// Each AdminTarget specifies a query handler slot and an optional address.
    /// This supports multi-match queries where different targets need different routes.
    fn start_admin_request_multi(
        &mut self,
        client_slot: u8,
        client_seq_id: u32,
        msg_type: u16,
        targets: &[AdminTarget],
        key: &[u8],
        value: &[u8],
    ) -> bool {
        // Find empty slot
        let req_slot = match self.pending_requests.iter().position(|r| !r.is_active()) {
            Some(s) => s,
            None => return false,
        };

        let mut req = PendingAdminRequest::empty();
        req.client_slot = client_slot;
        req.client_seq_id = client_seq_id;
        req.msg_type = msg_type;

        let mut target_idx: usize = 0;
        for t in targets {
            if target_idx >= MAX_REQUEST_TARGETS { break; }

            let seq_id = self.next_forward_seq_id;
            self.next_forward_seq_id = self.next_forward_seq_id.wrapping_add(1);
            if self.next_forward_seq_id == 0 { self.next_forward_seq_id = 1; }

            let sent = if msg_type == msg::QUERY_SERVICE_INFO {
                self.send_info_query_to_driver(t.slot, seq_id, key)
            } else {
                self.send_config_query(t.slot, seq_id, msg_type, key, value, &t.address[..t.address_len])
            };

            if sent {
                req.forward_seq_ids[target_idx] = seq_id;
                req.sent_mask |= 1 << target_idx;
                target_idx += 1;
            }
        }

        if req.sent_mask == 0 {
            return false;
        }

        // 5s timeout via inline timer (tag = 0x100 | req_slot)
        req.deadline_ns = userlib::syscall::gettime() + 5_000_000_000;
        // Relay mode for CONFIG queries: send each response immediately to avoid
        // accumulating into a 1024-byte buffer that can silently truncate.
        req.relay = client_seq_id == 0;
        self.pending_requests[req_slot] = req;
        if let Some(events) = &self.events {
            let _ = events.add_timer(0x100 | req_slot as u32, 5_000_000_000);
        }
        true
    }

    /// Convenience: start an admin request with the same address for all targets.
    fn start_admin_request(
        &mut self,
        client_slot: u8,
        client_seq_id: u32,
        msg_type: u16,
        targets: &[usize],
        key: &[u8],
        value: &[u8],
        address: &[u8],
    ) -> bool {
        let mut admin_targets = [AdminTarget::empty(); MAX_REQUEST_TARGETS];
        let count = targets.len().min(MAX_REQUEST_TARGETS);
        for i in 0..count {
            admin_targets[i] = AdminTarget::with_address(targets[i], address);
        }
        self.start_admin_request_multi(client_slot, client_seq_id, msg_type, &admin_targets[..count], key, value)
    }

    /// Check if a SERVICE_INFO_RESULT matches a pending admin request.
    ///
    /// Returns true if the message was consumed (caller should not process further).
    fn try_match_admin_response(&mut self, driver_slot: usize, seq_id: u32, info_bytes: &[u8], resp_flags: u16) -> bool {
        // Find matching request
        let req_slot = match self.pending_requests.iter().position(|r| {
            r.is_active() && r.matches_seq(seq_id)
        }) {
            Some(s) => s,
            None => return false,
        };

        // Extract fields we need before taking &mut borrows on other self fields
        let msg_type = self.pending_requests[req_slot].msg_type;
        let is_relay = self.pending_requests[req_slot].relay;
        let client_slot = self.pending_requests[req_slot].client_slot as usize;

        // EOL detection: flag-based or string-based (backward compat for CONFIG_SET)
        let is_eol = (resp_flags & query_flags::EOL != 0)
            || (msg_type == msg::CONFIG_SET && info_bytes == b"ERR unknown key\n");

        if !is_eol && !info_bytes.is_empty() {
            // Annotate with trigger_port/service_name source headers
            // e.g., "pcie:0/pcied" for a kernel bus driver
            let mut prefix_buf = [0u8; 64];
            let prefix_len = self.query_handler.get_service_idx(driver_slot)
                .and_then(|idx| self.services.get(idx as usize))
                .map(|s| {
                    let mut pos2 = 0usize;
                    let cap = prefix_buf.len();
                    // Strip "/" prefix from trigger port (e.g., "/pcie:0" → "pcie:0")
                    let tp = s.trigger_port();
                    let short = if tp.starts_with(b"/") {
                        &tp[1..]
                    } else {
                        tp
                    };
                    if !short.is_empty() {
                        let tlen = short.len().min(cap.saturating_sub(1));
                        prefix_buf[..tlen].copy_from_slice(&short[..tlen]);
                        pos2 += tlen;
                        if pos2 < cap {
                            prefix_buf[pos2] = b'/';
                            pos2 += 1;
                        }
                    }
                    let n = s.name().as_bytes();
                    let nlen = n.len().min(cap.saturating_sub(pos2));
                    prefix_buf[pos2..pos2 + nlen].copy_from_slice(&n[..nlen]);
                    pos2 + nlen
                })
                .unwrap_or(0);

            if is_relay {
                // Relay mode: annotate and send immediately instead of accumulating.
                let mut tmp = [0u8; 576];
                let written = append_with_source(
                    &mut tmp, 0,
                    &prefix_buf[..prefix_len], info_bytes,
                );
                if written > 0 {
                    self.send_chunked_response(client_slot, &tmp[..written]);
                    // Track that we relayed data (response_buf unused, but response_len
                    // marks "data was sent" for complete_admin_request error detection).
                    self.pending_requests[req_slot].response_len = 1;
                }
            } else {
                let req = &mut self.pending_requests[req_slot];
                let pos = req.response_len as usize;
                let written = append_with_source(
                    &mut req.response_buf, pos,
                    &prefix_buf[..prefix_len], info_bytes,
                );
                req.response_len += written as u16;
            }
        }

        // Set EOL bit for this target
        let req = &mut self.pending_requests[req_slot];
        if is_eol {
            if let Some(bit_idx) = req.seq_bit_index(seq_id) {
                req.eol_mask |= 1 << bit_idx;
            }
        }

        if req.is_converged() {
            self.complete_admin_request(req_slot);
        }

        true
    }

    /// Complete a converged or timed-out admin request — send response to client.
    fn complete_admin_request(&mut self, req_slot: usize) {
        let req = &self.pending_requests[req_slot];
        let client_slot = req.client_slot as usize;
        let client_seq_id = req.client_seq_id;
        let response_len = req.response_len as usize;
        let is_relay = req.relay;

        if client_seq_id != 0 {
            // Info query: build ServiceInfoResult with client's original seq_id.
            // Uses respond_info chunking (DevdClient handles IPC limit).
            use userlib::query::{ServiceInfoResult, ErrorResponse, error};

            if response_len > 0 {
                let client_resp = ServiceInfoResult::success(client_seq_id, response_len as u16);
                let mut resp_buf = [0u8; 1100];
                if let Some(len) = client_resp.write_to(&mut resp_buf, &req.response_buf[..response_len]) {
                    if let Some(client) = self.query_handler.get_mut(client_slot) {
                        let _ = client.send(&resp_buf[..len]);
                    }
                }
            } else {
                if let Some(client) = self.query_handler.get_mut(client_slot) {
                    let resp = ErrorResponse::new(client_seq_id, error::NOT_FOUND);
                    let _ = client.send(&resp.to_bytes());
                }
            }
        } else if is_relay {
            // Relay mode: data was already sent immediately. Nothing to accumulate.
            // If no data was relayed at all, send error.
            if response_len == 0 {
                if let Some(client) = self.query_handler.get_mut(client_slot) {
                    let _ = client.send(b"ERR key not found\n");
                }
            }
        } else {
            // Config query: send raw response or error text.
            // IPC max payload is 576 bytes — chunk on line boundaries if needed.
            if response_len > 0 {
                // Copy response out of pending_requests to satisfy borrow checker
                let mut resp_copy = [0u8; 1024];
                resp_copy[..response_len].copy_from_slice(&req.response_buf[..response_len]);
                self.send_chunked_response(client_slot, &resp_copy[..response_len]);
            } else {
                if let Some(client) = self.query_handler.get_mut(client_slot) {
                    let _ = client.send(b"ERR key not found\n");
                }
            }
        }

        self.pending_requests[req_slot] = PendingAdminRequest::empty();
        // Cancel the inline timer (may already be auto-removed if it fired)
        if let Some(events) = &self.events {
            let _ = events.remove_timer(0x100 | req_slot as u32);
        }
    }

    /// Send a raw text response to a query client, chunked on line boundaries
    /// to stay within the 576-byte IPC message limit.
    fn send_chunked_response(&mut self, client_slot: usize, response: &[u8]) {
        const MAX_CHUNK: usize = 576;
        let mut offset = 0;
        while offset < response.len() {
            let remaining = response.len() - offset;
            let chunk_end = if remaining <= MAX_CHUNK {
                response.len()
            } else {
                // Find last newline within MAX_CHUNK bytes
                response[offset..offset + MAX_CHUNK].iter().rposition(|&b| b == b'\n')
                    .map(|p| offset + p + 1)
                    .unwrap_or(offset + MAX_CHUNK)
            };
            if let Some(client) = self.query_handler.get_mut(client_slot) {
                let _ = client.send(&response[offset..chunk_end]);
            }
            offset = chunk_end;
        }
    }

    /// Handle an admin request timeout (inline timer fired).
    fn handle_admin_request_timeout(&mut self, req_slot: usize) {
        if req_slot < MAX_PENDING_REQUESTS && self.pending_requests[req_slot].is_active() {
            uerror!("devd", "admin_request_timeout"; slot = req_slot);
            self.complete_admin_request(req_slot);
        }
    }

    /// Send a QUERY_SERVICE_INFO to a driver with a forwarded seq_id.
    fn send_info_query_to_driver(&mut self, driver_slot: usize, seq_id: u32, name: &[u8]) -> bool {
        use userlib::query::QueryServiceInfo;
        let forward_req = QueryServiceInfo::new(seq_id);
        let mut forward_buf = [0u8; 128];
        if let Some(len) = forward_req.write_to(&mut forward_buf, name) {
            if let Some(client) = self.query_handler.get_mut(driver_slot) {
                return client.send(&forward_buf[..len]).is_ok();
            }
        }
        false
    }

    fn handle_restart_timer(&mut self, idx: usize) {
        if idx >= MAX_SERVICES {
            return;
        }

        let now = Self::now_ms();
        // Inline timer auto-removed by kernel on fire — just clear the flag
        if let Some(service) = self.services.get_mut(idx) {
            service.has_restart_timer = false;
        }

        // Determine action
        let should_spawn = {
            let service = match self.services.get_mut(idx) {
                Some(s) => s,
                None => return,
            };
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
            // For driver-owned services, re-fire rules so the parent driver
            // spawns the child (preserving hierarchy).  Kernel bus services
            // (owner=0xFF) spawn directly.
            let tp = self.services.get(idx).map(|s| {
                let tpl = s.trigger_port_len as usize;
                let mut buf = [0u8; 32];
                buf[..tpl].copy_from_slice(&s.trigger_port[..tpl]);
                (buf, tpl)
            });
            if let Some((tp_buf, tp_len)) = tp {
                let tp_name = &tp_buf[..tp_len];
                if let Some(owner) = self.ports.find_owner(tp_name) {
                    if owner != 0xFF {
                        if let Some(port) = self.ports.get(tp_name) {
                            let info = *port.port_info();
                            let pid = port.port_id();
                            self.check_class_rules(&info, owner, pid);
                            return;
                        }
                    }
                }
            }
            self.spawn_service(idx);
        }
    }


    // =========================================================================
    // Exception Channel Handling
    // =========================================================================

    /// Handle an exception channel event — a child has faulted.
    /// Reads ExceptionInfo, logs the crash, and kills the frozen child.
    fn handle_exception_event(&mut self, handle: ObjHandle) {
        let mut buf = [0u8; 32];
        let n = match syscall::read(handle, &mut buf) {
            Ok(n) => n,
            Err(_) => return,
        };
        if n < 32 { return; }

        // Parse ExceptionInfo: [pid:4][fault_type:1][_pad:3][esr:8][elr:8][far:8]
        let pid = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let fault_type = buf[4];
        let esr = u64::from_le_bytes(buf[8..16].try_into().unwrap_or([0; 8]));
        let elr = u64::from_le_bytes(buf[16..24].try_into().unwrap_or([0; 8]));
        let far = u64::from_le_bytes(buf[24..32].try_into().unwrap_or([0; 8]));

        let fault_name = match fault_type {
            0 => "DataAbort",
            1 => "InstrAbort",
            2 => "SError",
            _ => "Other",
        };

        // Find service name for the faulted child
        let svc_name = self.services.find_by_pid(pid)
            .and_then(|idx| self.services.get(idx))
            .map(|s| s.name())
            .unwrap_or("unknown");

        uwarn!("devd", "child_fault";
            pid = pid,
            name = svc_name,
            fault = fault_name,
            elr = userlib::ulog::hex64(elr),
            far = userlib::ulog::hex64(far),
            esr = userlib::ulog::hex64(esr));

        // Kill the frozen child — normal restart flow will handle respawn
        let _ = syscall::exception_resume(pid, abi::exception_action::KILL);
    }

    // =========================================================================
    // Signal Event Handling
    // =========================================================================

    fn handle_signal_event(&mut self, event: &abi::MuxEvent) {
        let sig = event.signal_event as u32;
        if sig & abi::signal_event::PORT_CHANGED != 0 {
            let port_idx = (event.signal_value >> 32) as u16;
            let new_state = event.signal_value as u32;
            udebug!("devd", "port_state_signal"; port = port_idx, state = new_state);
        }
        if sig & abi::signal_event::CHILD_EXIT != 0 {
            let child_pid = (event.signal_value >> 32) as u32;
            let exit_code = event.signal_value as i32;
            udebug!("devd", "child_exit_signal"; pid = child_pid, code = exit_code);
            if let Some(idx) = self.services.find_by_pid(child_pid) {
                self.handle_service_exit(idx, exit_code);
            }
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
                udebug!("devd", "query_accept"; pid = client_pid);
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

                // Save handle before moving channel into query handler
                let ch_handle = channel.handle();

                // Log the service_idx that will be used
                if let Some(idx) = service_idx {
                    if let Some(svc) = self.services.get(idx as usize) {
                        udebug!("devd", "query_driver"; pid = client_pid, idx = idx as u32, name = svc.name());
                    }
                }

                // Add to query handler
                match self.query_handler.add_client(channel, service_idx, client_pid) {
                    Some(slot) => {
                        // Try immediate non-blocking read — clients often send
                        // their request before we accept, so the message is
                        // already buffered in the channel.  Process it now so
                        // we don't depend on fitting the handle into the Mux.
                        self.try_immediate_query_read(slot);

                        // Add to event loop for future messages
                        if let Some(events) = &mut self.events {
                            if events.watch(ch_handle).is_err() {
                                // Mux full — mark this slot for polling
                                self.overflow_query_mask |= 1 << slot;
                                self.update_timeout();
                                uwarn!("devd", "query_watch_failed"; pid = client_pid);
                            }
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
        match client.try_recv(&mut recv_buf) {
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
        match client.try_recv(&mut recv_buf) {
            Ok(Some(len)) if len > 0 => {
                self.dispatch_query_message(slot, &recv_buf[..len]);
            }
            Ok(Some(_)) | Ok(None) => {
                // No data yet — will be picked up next poll or Mux event
            }
            Err(SysError::PeerClosed) | Err(SysError::ConnectionReset) => {
                // If managed service, channel close = child died
                let managed_svc_idx = self.query_handler.get(slot)
                    .filter(|c| c.is_managed && c.service_idx >= 0)
                    .map(|c| c.service_idx as usize);
                self.remove_query_client(slot);
                if let Some(svc_idx) = managed_svc_idx {
                    self.handle_service_exit(svc_idx, -1);
                }
                return;
            }
            Err(_e) => {
                uerror!("devd", "query_recv_failed"; slot = slot as u32);
                self.remove_query_client(slot);
            }
        }
    }

    /// Poll overflow query clients for pending messages.
    ///
    /// Only polls clients whose channel handles could not be added to the Mux
    /// (tracked in overflow_query_mask).  Skips entirely when all clients are
    /// Mux-watched.
    fn poll_query_clients(&mut self) {
        if self.overflow_query_mask == 0 {
            return;
        }
        let mut mask = self.overflow_query_mask;
        while mask != 0 {
            let slot = mask.trailing_zeros() as usize;
            mask &= mask - 1; // clear lowest set bit
            self.try_read_query_slot(slot);
        }
    }

    /// Check if a msg_type value belongs to a known binary protocol message.
    /// Binary msg_types live in ranges 0x01xx-0x05xx. Text admin commands
    /// start with ASCII letters (e.g., 'C'=0x43) giving msg_type >= 0x4100.
    fn is_known_binary_msg(msg_type: u16) -> bool {
        let high = msg_type >> 8;
        high >= 1 && high <= 5
    }

    /// Extract the ADDRESSED source path from a raw message.
    ///
    /// If the ADDRESSED flag is set, extracts the route bytes into `path`
    /// and produces a stripped copy (header with ADDRESSED cleared + payload)
    /// in `stripped`. If not ADDRESSED, path is empty and stripped = original.
    ///
    /// Returns (path_len, stripped_len).
    /// Strip ADDRESSED route bytes from a binary message.
    /// Returns (route_len, stripped_message_len).
    fn strip_addressed_route(
        buf: &[u8],
        stripped: &mut [u8; 512],
    ) -> (usize, usize) {
        if buf.len() < QueryHeader::SIZE {
            let len = buf.len().min(512);
            stripped[..len].copy_from_slice(&buf[..len]);
            return (0, len);
        }

        let flags = u16::from_le_bytes([buf[2], buf[3]]);
        if flags & query_flags::ADDRESSED == 0 {
            let len = buf.len().min(512);
            stripped[..len].copy_from_slice(&buf[..len]);
            return (0, len);
        }

        let route_len = if buf.len() > QueryHeader::SIZE {
            buf[QueryHeader::SIZE] as usize
        } else {
            0
        };

        // Build stripped: header (ADDRESSED cleared) + payload (skip route)
        let payload_start = QueryHeader::SIZE + 1 + route_len;
        stripped[..QueryHeader::SIZE].copy_from_slice(&buf[..QueryHeader::SIZE]);
        let new_flags = flags & !query_flags::ADDRESSED;
        stripped[2..4].copy_from_slice(&new_flags.to_le_bytes());

        if payload_start < buf.len() {
            let payload = &buf[payload_start..];
            let plen = payload.len().min(512 - QueryHeader::SIZE);
            stripped[QueryHeader::SIZE..QueryHeader::SIZE + plen].copy_from_slice(&payload[..plen]);
            (route_len, QueryHeader::SIZE + plen)
        } else {
            (route_len, QueryHeader::SIZE)
        }
    }

    /// Dispatch a query message by type.  Shared by handle_query_client_event
    /// (Mux-driven) and try_immediate_query_read (accept-driven).
    fn dispatch_query_message(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{msg, QueryHeader};

        // Strip ADDRESSED route from binary messages. Text admin commands
        // (e.g., "CONFIG GET key\n") parse as QueryHeader with garbage
        // msg_type and may coincidentally have the ADDRESSED bit set.
        let raw_msg_type = if buf.len() >= 2 {
            u16::from_le_bytes([buf[0], buf[1]])
        } else {
            0
        };
        let is_binary = Self::is_known_binary_msg(raw_msg_type);

        let mut stripped = [0u8; 512];
        let dispatch_len;
        let mut route_buf = [0u8; 96];
        let mut route_len = 0usize;
        if is_binary {
            // Save the route before stripping it (needed for STATE_CHANGE attribution)
            let flags = if buf.len() >= 4 { u16::from_le_bytes([buf[2], buf[3]]) } else { 0 };
            if flags & query_flags::ADDRESSED != 0 && buf.len() > QueryHeader::SIZE {
                route_len = (buf[QueryHeader::SIZE] as usize).min(route_buf.len());
                let route_start = QueryHeader::SIZE + 1;
                if route_start + route_len <= buf.len() {
                    route_buf[..route_len].copy_from_slice(&buf[route_start..route_start + route_len]);
                }
            }
            let (_, slen) = Self::strip_addressed_route(buf, &mut stripped);
            dispatch_len = slen;
        } else {
            let len = buf.len().min(512);
            stripped[..len].copy_from_slice(&buf[..len]);
            dispatch_len = len;
        }

        let dispatch_buf = &stripped[..dispatch_len];
        let msg_type = QueryHeader::from_bytes(dispatch_buf).map(|h| h.msg_type);

        match msg_type {
            Some(msg::REGISTER_PORT_INFO) => {
                // For routed PORT_REGISTER messages (from grandchildren via parent relay),
                // resolve the actual owning service from the route so the port is
                // attributed to the grandchild, not the relay parent.
                let mut owner_override = if route_len > 0 {
                    self.resolve_service_idx_from_route(&route_buf[..route_len])
                        .map(|i| i as u8)
                } else {
                    None
                };
                // If routed but can't resolve (SPAWN_ACK lost), create from inflight
                if owner_override.is_none() && route_len > 0 {
                    owner_override = self.create_service_from_inflight_route(&route_buf[..route_len])
                        .map(|i| i as u8);
                }
                self.handle_port_register_info_msg(slot, dispatch_buf, owner_override);
            }
            Some(msg::STATE_CHANGE) => {
                // Resolve the service index: routed messages identify the
                // service by binary name in the route (works for grandchildren
                // that don't have a query client slot in devd). Non-routed
                // messages fall back to the query client's service index.
                let mut svc_idx = if route_len > 0 {
                    self.resolve_service_idx_from_route(&route_buf[..route_len])
                } else {
                    self.query_handler.get_service_idx(slot).map(|i| i as usize)
                };
                // If routed STATE_CHANGE can't resolve (e.g. SPAWN_ACK was
                // lost in a race), create the service entry on the fly
                // from the inflight spawn matching this binary name.
                if svc_idx.is_none() && route_len > 0 {
                    svc_idx = self.create_service_from_inflight_route(&route_buf[..route_len]);
                }
                if let Some(idx) = svc_idx {
                    self.handle_state_change_for_service(idx, dispatch_buf);
                }
            }
            Some(msg::SPAWN_ACK) => {
                self.handle_spawn_ack_msg(slot, dispatch_buf);
            }
            Some(msg::GET_SPAWN_CONTEXT) => {
                self.handle_get_spawn_context(slot, dispatch_buf);
            }
            Some(msg::QUERY_PORT) => {
                self.handle_query_port(slot, dispatch_buf);
            }
            Some(msg::UPDATE_PORT_SHMEM_ID) => {
                self.handle_update_port_shmem_id(slot, dispatch_buf);
            }
            Some(msg::SET_PORT_STATE) => {
                self.handle_set_port_state(slot, dispatch_buf);
            }
            Some(msg::LIST_PORTS) => {
                self.handle_list_ports(slot, dispatch_buf);
            }
            Some(msg::LIST_SERVICES) => {
                self.handle_list_services(slot, dispatch_buf);
            }
            Some(msg::QUERY_SERVICE_INFO) => {
                self.handle_query_service_info(slot, dispatch_buf);
            }
            Some(msg::SERVICE_INFO_RESULT) => {
                self.handle_service_info_result(slot, dispatch_buf);
            }
            Some(msg::LOG_MESSAGE) => {
                self.handle_log_message(slot, dispatch_buf);
            }
            Some(msg::LOG_QUERY) => {
                self.handle_log_query(slot, dispatch_buf);
            }
            Some(msg::LOG_CONTROL) => {
                self.handle_log_control(slot, dispatch_buf);
            }
            Some(msg::REGISTER_MOUNT) => {
                self.handle_register_mount(slot, dispatch_buf);
            }
            Some(msg::RESOLVE_PATH) => {
                self.handle_resolve_path(slot, dispatch_buf);
            }
            Some(msg::LIST_MOUNTS) => {
                self.handle_list_mounts(slot, dispatch_buf);
            }
            Some(msg::CONFIG_GET) | Some(msg::CONFIG_SET) => {
                // CONFIG_GET/SET should never arrive at devd — they are sent
                // BY devd TO managed drivers. Ignore to prevent binary payloads
                // from being misinterpreted as text admin commands.
            }
            Some(_) => {
                // Unknown binary message type — try as text admin command.
                // (QueryHeader::from_bytes returns Some for ANY >=8 byte buffer,
                // so text commands like "CONFIG GET key\n" land here too)
                let mut response_buf = [0u8; MSG_BUFFER_SIZE];
                let resp_len = self.handle_admin_command(dispatch_buf, &mut response_buf, slot);
                if resp_len > 0 {
                    if let Some(client) = self.query_handler.get_mut(slot) {
                        let _ = client.send(&response_buf[..resp_len]);
                    }
                }
            }
            None => {
                // Buffer too short for QueryHeader — try as text admin command
                // (e.g. "LIST\n" which is < 8 bytes)
                let mut resp_buf = [0u8; MSG_BUFFER_SIZE];
                let resp_len = self.handle_admin_command(dispatch_buf, &mut resp_buf, slot);
                if resp_len > 0 {
                    if let Some(client) = self.query_handler.get_mut(slot) {
                        let _ = client.send(&resp_buf[..resp_len]);
                    }
                }
            }
        }
    }

    /// Resolve a service's query slot from an ADDRESSED route.
    ///
    /// The route format is "port_name/binary_name[/port_name/binary_name]*".
    /// The last segment after the final "/" is the binary name of the actual
    /// service that sent the message. We look up its query slot by finding the
    /// service with that binary name and returning its query handler slot.
    /// Resolve a route to a query client slot (for messages that need a client slot).
    fn resolve_service_slot_from_route(&self, route: &[u8]) -> Option<usize> {
        let svc_idx = self.resolve_service_idx_from_route(route)?;
        self.query_handler.find_by_service_idx(svc_idx as u8)
    }

    /// Resolve a route to a service registry index.
    ///
    /// Works for both direct children (with query client) and grandchildren
    /// (no query client — they communicate via their parent's SuperQ relay).
    fn resolve_service_idx_from_route(&self, route: &[u8]) -> Option<usize> {
        // Find the last segment: route is "a/b/c/d", we want "d"
        // Segments alternate: port_name/binary_name/port_name/binary_name
        // The last segment is the binary name of the service
        let mut last_slash = None;
        for i in (0..route.len()).rev() {
            if route[i] == b'/' {
                last_slash = Some(i);
                break;
            }
        }
        let binary_name = match last_slash {
            Some(pos) => &route[pos + 1..],
            None => route, // Single segment = binary name itself
        };
        if binary_name.is_empty() {
            return None;
        }
        let name_str = core::str::from_utf8(binary_name).ok()?;
        self.services.find_by_name(name_str)
    }

    /// Create a service entry from an inflight spawn when SPAWN_ACK was lost.
    ///
    /// Extracts the binary name from the route's last segment, matches it
    /// against inflight_spawns, consumes the inflight entry, and creates
    /// a service slot. Returns the service index if successful.
    fn create_service_from_inflight_route(&mut self, route: &[u8]) -> Option<usize> {
        // Extract binary name from route (last segment)
        let mut last_slash = None;
        for i in (0..route.len()).rev() {
            if route[i] == b'/' {
                last_slash = Some(i);
                break;
            }
        }
        let binary_name = match last_slash {
            Some(pos) => &route[pos + 1..],
            None => route,
        };
        if binary_name.is_empty() {
            return None;
        }

        // Find matching inflight spawn by binary name, capture parent before consuming
        let mut found_seq = 0u32;
        let mut parent_svc_idx = 0u8;
        for entry in &self.inflight_spawns {
            if entry.seq_id != 0 {
                let ename = &entry.binary_name[..entry.binary_name_len as usize];
                if ename == binary_name {
                    found_seq = entry.seq_id;
                    parent_svc_idx = entry.parent_service_idx;
                    break;
                }
            }
        }
        if found_seq == 0 {
            return None;
        }

        // Consume the inflight spawn and create a service entry
        let spawn_ctx = self.consume_inflight_spawn(found_seq)?;
        let (port_type, port_name, port_name_len, _, _, _, spawn_caps, link_id, trig_port_id) = spawn_ctx;
        let pname = &port_name[..port_name_len as usize];
        let now = Self::now_ms();

        // Reuse existing slot by link_id, or create new
        let mut reused_idx: Option<usize> = None;
        if link_id != 0 {
            self.services.for_each(|idx, svc| {
                if svc.link_id == link_id {
                    reused_idx = Some(idx);
                }
            });
        }

        let slot_idx = if let Some(idx) = reused_idx {
            if let Some(svc) = self.services.get_mut(idx) {
                svc.pid = 0;
                svc.state = ServiceState::Starting;
                svc.caps = spawn_caps;
                svc.link_id = link_id;
            }
            idx
        } else {
            self.services.create_dynamic_service_with_state(
                0, binary_name, now, ServiceState::Starting,
            )?
        };

        if let Some(svc) = self.services.get_mut(slot_idx) {
            svc.set_trigger_port(pname);
        }

        udebug!("devd", "svc_created_from_route"; name = core::str::from_utf8(binary_name).unwrap_or("?"), slot = slot_idx as u32);

        // Drain deferred rules for this parent (inflight consumed = slot freed)
        self.drain_deferred_rules(parent_svc_idx);

        Some(slot_idx)
    }

    /// Handle REGISTER_PORT_INFO message (unified PortInfo registration)
    ///
    /// `owner_override`: if Some, use this service index as owner instead of the
    /// query client's service index. Used for routed PORT_REGISTER messages from
    /// grandchildren (e.g., nvmed registering block:0 via pcied relay).
    fn handle_port_register_info_msg(&mut self, slot: usize, buf: &[u8], owner_override: Option<u8>) {
        use userlib::query::error;

        // Parse the registration message
        let mut info = match self.query_handler.parse_port_register_info(slot, buf) {
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

        // For routed messages (grandchild ports), the actual owner is the
        // grandchild but the relay parent (with query client) handles SpawnChild routing.
        let relay_owner = info.owner_idx; // Always the query client's service (relay parent)
        if let Some(owner) = owner_override {
            info.owner_idx = owner; // Actual owner for rule matching
        }

        // Register the port with unified PortInfo
        let result = self.handle_port_registration(
            &info.port_info,
            info.owner_idx,
            relay_owner,
            info.shmem_id,
        );

        // Send response
        let result_code = match result {
            Ok(port_id) => {
                // Port registered successfully. Store the source path
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
        let driver_idx = match self.query_handler.get_service_idx(slot) {
            Some(idx) => idx as usize,
            None => return,
        };
        self.handle_state_change_for_service(driver_idx, buf);
    }

    /// Handle STATE_CHANGE for a service identified by service index.
    fn handle_state_change_for_service(&mut self, driver_idx: usize, buf: &[u8]) {
        use userlib::query::{StateChange, driver_state};

        // Parse the state change message
        let state_msg = match StateChange::from_bytes(buf) {
            Some(s) => s,
            None => {
                uerror!("devd", "invalid_state_change"; svc_idx = driver_idx as u32);
                return;
            }
        };

        let driver_idx = driver_idx;

        udebug!("devd", "svc_state_change"; name = self.svc_name(driver_idx as u8), state = state_msg.new_state as u32);

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
            let mut ready_port_ids: [u8; 8] = [0xFF; 8];
            let mut ready_count = 0usize;
            self.ports.for_each(|port| {
                if port.owner() == owner_idx && port.state() != abi::PortState::Claimed && ready_count < 8 {
                    ready_port_ids[ready_count] = port.port_id();
                    ready_count += 1;
                }
            });
            for i in 0..ready_count {
                let pid = ready_port_ids[i];
                let old_state = self.ports.set_state_by_id(pid, abi::PortState::Claimed);
                match old_state {
                    Some(old) => {
                        let port_name = self.ports.get_by_id(pid)
                            .map(|p| core::str::from_utf8(p.name()).unwrap_or("?"))
                            .unwrap_or("?");
                        uinfo!("devd", "port_transition";
                            port = port_name,
                            from = old.as_str(),
                            to = abi::PortState::Claimed.as_str()
                        );
                    }
                    None => {
                        uwarn!("devd", "port_claim_failed"; port_id = pid as u32);
                    }
                }
                if let Some(port) = self.ports.get_by_id(pid) {
                    let info = *port.port_info();
                    self.check_class_rules(&info, owner_idx, pid);
                }
            }
        }
    }

    /// Handle GET_SPAWN_CONTEXT message from a child driver
    ///
    /// Returns the port name that triggered this driver's spawn,
    /// plus any context key-value pairs from rule template expansion.
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

        // Look up spawn context for this PID (including context KV pairs)
        let mut response_buf = [0u8; 512];
        let resp_len = if let Some(spawn_ctx) = self.get_spawn_context_full(client_pid) {
            let mut resp = SpawnContextResponse::new(header.seq_id, error::OK, spawn_ctx.port_type);
            resp.port_id = spawn_ctx.trigger_port_id;
            let port_name = &spawn_ctx.port_name[..spawn_ctx.port_name_len as usize];
            let metadata = &spawn_ctx.metadata[..spawn_ctx.metadata_len as usize];

            // Build context KV slice for serialization
            let kv_count = spawn_ctx.context_kv_count as usize;
            let mut kv_refs: [(&[u8], &[u8]); MAX_CONTEXT_KV] = [(&[], &[]); MAX_CONTEXT_KV];
            for i in 0..kv_count {
                kv_refs[i] = (spawn_ctx.context_kv[i].key(), spawn_ctx.context_kv[i].value());
            }

            resp.write_to_full(&mut response_buf, port_name, metadata, &kv_refs[..kv_count])
                .unwrap_or(SpawnContextResponse::HEADER_SIZE)
        } else {
            // No context available - return error
            let resp = SpawnContextResponse::new(header.seq_id, error::NOT_FOUND, 0);
            resp.write_to(&mut response_buf, &[])
                .unwrap_or(SpawnContextResponse::HEADER_SIZE)
        };

        // Send response
        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.send(&response_buf[..resp_len]);
        }
    }

    /// Handle QUERY_PORT message - returns port info including shmem_id
    ///
    /// Supports two lookup modes:
    /// - port_id != 0xFF: lookup by port_id (unambiguous, for same-name ports)
    /// - port_id == 0xFF: lookup by name (legacy/default)
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

        // Look up the port — by port_id if specified, else by name
        let port = if query.port_id != 0xFF {
            self.ports.get_by_id(query.port_id)
        } else {
            self.ports.get(port_name)
        };

        let resp = if let Some(port) = port {
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
            let _ = client.send(&resp.to_bytes());
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

        // Update the port's shmem_id
        let result = self.ports.set_shmem_id(port_name, update.shmem_id);

        let result_code = match result {
            Ok(()) => {
                udebug!("devd", "port_shmem_update"; port = core::str::from_utf8(port_name).unwrap_or("?"), shmem_id = update.shmem_id);
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
            let _ = client.send(&resp.to_bytes());
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

        // Get port info + port_id before state change (for rule checking)
        let port_info = self.ports.get(port_name).map(|p| *p.port_info());
        let port_id = self.ports.get_port_id(port_name).unwrap_or(0xFF);
        let owner_idx = self.ports.find_owner(port_name);
        // Update the port's state
        let old_state = self.ports.set_state(port_name, new_state);

        let result_code = match old_state {
            Some(old) => {
                uinfo!("devd", "port_transition";
                    port = core::str::from_utf8(port_name).unwrap_or("?"),
                    from = old.as_str(),
                    to = new_state.as_str()
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
            let port_link_id = self.ports.get_by_id(port_id)
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
                if self.ports.set_state_by_id(port_id, abi::PortState::Claimed).is_none() {
                    uwarn!("devd", "port_reclaim_failed"; port_id = port_id as u32);
                }
                if let (Some(info), Some(owner)) = (port_info, owner_idx) {
                    self.check_class_rules(&info, owner, port_id);
                }
            }
        }

        // Check rules on transition to Claimed (from external SET_PORT_STATE)
        if result_code == error::OK
            && new_state == abi::PortState::Claimed
            && old_state != Some(abi::PortState::Claimed)
        {
            if let (Some(info), Some(owner)) = (port_info, owner_idx) {
                self.check_class_rules(&info, owner, port_id);
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
            let _ = client.send(&resp);
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
            let mut name = [0u8; 18];
            let name_bytes = port.name();
            let name_len = name_bytes.len().min(18);
            name[..name_len].copy_from_slice(&name_bytes[..name_len]);

            let entry = PortEntry {
                name,
                port_id: port.port_id(),
                port_type: port.port_type(),
                flags,
                owner_idx: port.owner(),
                parent_idx: port.parent_port_id(),
                _pad: 0,
                shmem_id: port.shmem_id(),
                owner_pid,
            };

            entries[count] = entry.to_bytes();
            count += 1;
        });

        // Send response in chunks that fit within IPC payload limit (576 bytes).
        // Each chunk: 12-byte header + up to 17 × 32-byte entries = 556 bytes.
        let total = count as u16;
        let mut sent = 0usize;
        while sent < count {
            let chunk = (count - sent).min(PortsListResponse::MAX_PER_MSG);
            let resp_header = PortsListResponse::new(seq_id, chunk as u16, total);

            let mut resp_buf = [0u8; PortsListResponse::HEADER_SIZE + PortsListResponse::MAX_PER_MSG * PortEntry::SIZE];
            resp_buf[..PortsListResponse::HEADER_SIZE].copy_from_slice(&resp_header.header_to_bytes());

            let mut offset = PortsListResponse::HEADER_SIZE;
            for i in 0..chunk {
                resp_buf[offset..offset + PortEntry::SIZE].copy_from_slice(&entries[sent + i]);
                offset += PortEntry::SIZE;
            }

            if let Some(client) = self.query_handler.get_mut(slot) {
                if client.send(&resp_buf[..offset]).is_err() {
                    uerror!("devd", "list_ports_send_fail"; slot = slot as u32, chunk = chunk as u32);
                    break;
                }
            } else {
                break;
            }
            sent += chunk;
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

        // Collect service data into entries.
        // trigger_port is on the Service struct so bus_path is available even
        // for non-running services (pid=0), unlike the old spawn_context lookup.
        let mut entries = [[0u8; ServiceEntry::SIZE]; 16];
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

            // Use trigger_port directly — persists across crashes/restarts
            let mut bus_path = [0u8; 32];
            let tp = service.trigger_port();
            let tp_len = tp.len().min(32);
            bus_path[..tp_len].copy_from_slice(&tp[..tp_len]);

            let entry = ServiceEntry {
                name,
                pid: service.pid,
                state,
                index: idx as u8,
                parent_idx: 0xFF,
                child_count: 0,
                total_restarts: service.total_restarts,
                last_change: service.last_change as u32,
                bus_path,
            };
            entries[count] = entry.to_bytes();
            count += 1;
        });

        // Send response in chunks that fit within IPC payload limit (576 bytes).
        // Each chunk: 12-byte header + up to 8 × 64-byte entries = 524 bytes.
        let total = count as u16;
        let mut sent = 0usize;
        while sent < count {
            let chunk = (count - sent).min(ServicesListResponse::MAX_PER_MSG);
            let resp_header = ServicesListResponse::new(seq_id, chunk as u16, total);

            let mut resp_buf = [0u8; ServicesListResponse::HEADER_SIZE + ServicesListResponse::MAX_PER_MSG * ServiceEntry::SIZE];
            resp_buf[..ServicesListResponse::HEADER_SIZE].copy_from_slice(&resp_header.header_to_bytes());

            let mut offset = ServicesListResponse::HEADER_SIZE;
            for i in 0..chunk {
                resp_buf[offset..offset + ServiceEntry::SIZE].copy_from_slice(&entries[sent + i]);
                offset += ServiceEntry::SIZE;
            }

            if let Some(client) = self.query_handler.get_mut(slot) {
                if client.send(&resp_buf[..offset]).is_err() {
                    uerror!("devd", "list_svc_send_fail"; slot = slot as u32, chunk = chunk as u32);
                    break;
                }
            } else {
                break;
            }
            sent += chunk;
        }
    }

    /// Handle QUERY_SERVICE_INFO - forward to the named service's driver (async)
    fn handle_query_service_info(&mut self, client_slot: usize, buf: &[u8]) {
        use userlib::query::{QueryServiceInfo, ErrorResponse, error};

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
                    let _ = client.send(&resp.to_bytes());
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
                    let _ = client.send(&resp.to_bytes());
                }
                return;
            }
        };

        // Find the driver's query client slot
        let driver_slot = match self.query_handler.find_by_service_idx(service_idx as u8) {
            Some(slot) => slot,
            None => {
                if let Some(client) = self.query_handler.get_mut(client_slot) {
                    let resp = ErrorResponse::new(client_seq_id, error::NO_DRIVER);
                    let _ = client.send(&resp.to_bytes());
                }
                return;
            }
        };

        // Use unified admin request with client_seq_id for info query response rewriting
        let targets = [driver_slot];
        if !self.start_admin_request(client_slot as u8, client_seq_id, msg::QUERY_SERVICE_INFO, &targets, name_bytes, &[], &[]) {
            if let Some(client) = self.query_handler.get_mut(client_slot) {
                let resp = ErrorResponse::new(client_seq_id, error::DEVICE_ERROR);
                let _ = client.send(&resp.to_bytes());
            }
        }
    }

    /// Handle SERVICE_INFO_RESULT - unified matching via PendingAdminRequest
    fn handle_service_info_result(&mut self, driver_slot: usize, buf: &[u8]) {
        use userlib::query::ServiceInfoResult;

        if let Some((resp, info_bytes)) = ServiceInfoResult::from_bytes(buf) {
            self.try_match_admin_response(driver_slot, resp.header.seq_id, info_bytes, resp.header.flags);
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
                let _ = client.send(&resp_buf[..len]);
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
            let _ = client.send(&resp.to_bytes());
        }
    }

    // =========================================================================
    // Mount Table Handlers
    // =========================================================================

    /// Handle REGISTER_MOUNT message — driver registers a mount prefix.
    fn handle_register_mount(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{RegisterMount, PortRegisterResponse, mount_transport, error};

        let (transport_type, prefix, port_name, shmem_id) = match RegisterMount::from_bytes(buf) {
            Some(parsed) => parsed,
            None => {
                uerror!("devd", "invalid_register_mount"; slot = slot as u32);
                return;
            }
        };

        let seq_id = QueryHeader::from_bytes(buf).map(|h| h.seq_id).unwrap_or(0);
        let caller_pid = self.query_handler.get(slot)
            .map(|c| c.pid)
            .unwrap_or(0);

        let transport = match transport_type {
            mount_transport::PORT => {
                let mut name = [0u8; 32];
                let len = port_name.len().min(32);
                name[..len].copy_from_slice(&port_name[..len]);
                mounts::MountTransport::Port { name, name_len: len as u8 }
            }
            mount_transport::DATAPORT => {
                mounts::MountTransport::DataPort { shmem_id }
            }
            _ => {
                uerror!("devd", "unknown_mount_transport"; transport = transport_type as u32);
                return;
            }
        };

        let result = match self.mounts.mount(prefix, transport, caller_pid) {
            Ok(()) => {
                uinfo!("devd", "mount_registered";
                    prefix = core::str::from_utf8(prefix).unwrap_or("?"),
                    pid = caller_pid
                );
                error::OK
            }
            Err(mounts::MountError::Duplicate) => error::DUPLICATE,
            Err(mounts::MountError::TableFull) => error::TABLE_FULL,
            Err(_) => error::INVALID_REQUEST,
        };

        // Send ack using PortRegisterResponse (12 bytes: header + result)
        let resp = PortRegisterResponse::new(seq_id, result);
        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.send(&resp.to_bytes());
        }
    }

    /// Handle RESOLVE_PATH message — resolve path to mount transport.
    fn handle_resolve_path(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{ResolvePath, ResolvePathResponse, mount_transport};

        let path = match ResolvePath::from_bytes(buf) {
            Some(p) => p,
            None => {
                uerror!("devd", "invalid_resolve_path"; slot = slot as u32);
                return;
            }
        };

        let seq_id = QueryHeader::from_bytes(buf).map(|h| h.seq_id).unwrap_or(0);

        let mut resp_buf = [0u8; ResolvePathResponse::SIZE];
        let resp_len = match self.mounts.resolve(path) {
            Some((transport, remaining)) => {
                match transport {
                    mounts::MountTransport::DataPort { shmem_id } => {
                        ResolvePathResponse::write_dataport(&mut resp_buf, seq_id, *shmem_id, remaining)
                    }
                    mounts::MountTransport::Port { name, name_len } => {
                        ResolvePathResponse::write_port(&mut resp_buf, seq_id, &name[..*name_len as usize], remaining)
                    }
                }
            }
            None => {
                ResolvePathResponse::write_not_found(&mut resp_buf, seq_id)
            }
        };

        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.send(&resp_buf[..resp_len]);
        }
    }

    /// Handle LIST_MOUNTS message — return all registered mounts.
    fn handle_list_mounts(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{MountsListResponse, MountListEntry, mount_transport};

        let seq_id = QueryHeader::from_bytes(buf).map(|h| h.seq_id).unwrap_or(0);

        // Collect entries
        let mut entries = [[0u8; MountListEntry::SIZE]; mounts::MAX_MOUNTS];
        let mut count = 0usize;

        self.mounts.for_each(|entry| {
            if count >= mounts::MAX_MOUNTS {
                return;
            }
            let (transport_type, port_name, shmem_id) = match &entry.transport {
                mounts::MountTransport::Port { name, name_len } => {
                    (mount_transport::PORT, &name[..*name_len as usize], 0u32)
                }
                mounts::MountTransport::DataPort { shmem_id } => {
                    (mount_transport::DATAPORT, &[][..], *shmem_id)
                }
            };
            MountListEntry::write_to(&mut entries[count], transport_type, entry.prefix_bytes(), port_name, shmem_id);
            count += 1;
        });

        // Send in chunks
        let total = count as u16;
        let mut sent = 0usize;
        while sent < count {
            let chunk = (count - sent).min(MountListEntry::MAX_PER_MSG);
            let mut resp_buf = [0u8; MountsListResponse::HEADER_SIZE + MountListEntry::MAX_PER_MSG * MountListEntry::SIZE];
            MountsListResponse::write_header(&mut resp_buf, seq_id, chunk as u16, total);

            let mut offset = MountsListResponse::HEADER_SIZE;
            for i in 0..chunk {
                resp_buf[offset..offset + MountListEntry::SIZE].copy_from_slice(&entries[sent + i]);
                offset += MountListEntry::SIZE;
            }

            if let Some(client) = self.query_handler.get_mut(slot) {
                if client.send(&resp_buf[..offset]).is_err() {
                    break;
                }
            } else {
                break;
            }
            sent += chunk;
        }

        // If no mounts, send empty response
        if count == 0 {
            let mut resp_buf = [0u8; MountsListResponse::HEADER_SIZE];
            MountsListResponse::write_header(&mut resp_buf, seq_id, 0, 0);
            if let Some(client) = self.query_handler.get_mut(slot) {
                let _ = client.send(&resp_buf);
            }
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

        // Consume inflight spawn FIRST, then drain deferred rules for this parent.
        // This must happen regardless of success/failure so deferred rules don't starve.
        let _parent_for_drain = parent_idx;

        udebug!("devd", "spawn_ack"; parent = self.svc_name(parent_idx as u8), seq = seq_id, result = result as i32, spawn = spawn_count as u32);

        // Consume the inflight spawn to get port context and binary name
        let spawn_ctx = self.consume_inflight_spawn(seq_id);

        if spawn_count == 0 || result < 0 {
            if let Some((_, _, _, _, _, service_idx, _, _, _)) = spawn_ctx {
                if service_idx != 0xFF {
                    let now = Self::now_ms();
                    self.services.transition(service_idx as usize, ServiceState::Failed { code: result as i32 }, now);
                }
            }
            self.drain_deferred_rules(_parent_for_drain);
            return;
        }

        // Check if this is a service spawn (parent-delegated)
        if let Some((port_type, port_name, port_name_len, _, _, service_idx, spawn_caps, link_id, trig_port_id)) = spawn_ctx {
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
                    // Use pending context KVs (moved from inflight spawn by consume_inflight_spawn)
                    // Copy out of self to avoid borrow conflict
                    let kv_count = self.pending_context_kv_count as usize;
                    let pending_kvs = self.pending_context_kvs;
                    self.pending_context_kv_count = 0;
                    let mut kv_refs: [(&[u8], &[u8]); MAX_CONTEXT_KV] = [(&[], &[]); MAX_CONTEXT_KV];
                    for i in 0..kv_count {
                        kv_refs[i] = (pending_kvs[i].key(), pending_kvs[i].value());
                    }
                    self.store_spawn_context_with_kv(
                        child_pid, port_type, pname, &metadata.0[..metadata.1], trig_port_id,
                        &kv_refs[..kv_count],
                    );
                }
                self.query_handler.upgrade_to_managed(child_pid, service_idx);
                self.drain_deferred_rules(_parent_for_drain);
                return;
            }
        }

        // Extract binary name, caps, and link_id from spawn context (for dynamic spawns)
        let binary_name: &[u8] = spawn_ctx.as_ref()
            .map(|(_, _, _, bin, bin_len, _, _, _, _)| &bin[..*bin_len as usize])
            .unwrap_or(b"???");
        let spawn_caps: u64 = spawn_ctx.as_ref()
            .map(|(_, _, _, _, _, _, caps, _, _)| *caps)
            .unwrap_or(0);
        let spawn_link_id: u32 = spawn_ctx.as_ref()
            .map(|(_, _, _, _, _, _, _, lid, _)| *lid)
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
            if let Some((_, port_name, port_name_len, _, _, _, _, link_id, _)) = spawn_ctx {
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
            if let Some((port_type, port_name, port_name_len, _, _, _, _, _, trig_port_id)) = spawn_ctx {
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
                self.store_spawn_context(child_pid, port_type, pname, &metadata.0[..metadata.1], trig_port_id);
            }

            // Upgrade any already-connected client with this PID to driver status
            // This handles the race where child connects before SPAWN_ACK arrives
            self.query_handler.upgrade_to_managed(child_pid, slot_idx as u8);

        }

        // Drain deferred rules now that this parent's SuperQ has capacity
        self.drain_deferred_rules(_parent_for_drain);
    }



    fn remove_query_client(&mut self, slot: usize) {
        let was_overflow = self.overflow_query_mask & (1 << slot) != 0;
        self.overflow_query_mask &= !(1 << slot);
        // Get the handle before removing (for unwatch)
        let handle = self.query_handler.get(slot).and_then(|c| c.handle());
        if self.query_handler.remove_client(slot) {
            if !was_overflow {
                if let Some(h) = handle {
                    if let Some(events) = &mut self.events {
                        let _ = events.unwatch(h);
                    }
                }
            }
        }
        self.update_timeout();
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
        relay_owner_idx: u8,
        shmem_id: u32,
    ) -> Result<u8, SysError> {
        use crate::ports::PortRegistry;

        // Register the port with unified PortInfo
        let port_id = self.ports.register_with_port_info(port_info, owner_idx, shmem_id)?;

        // Set relay owner (differs from owner for grandchild ports)
        if relay_owner_idx != owner_idx {
            if let Some(port) = self.ports.get_mut_by_id(port_id) {
                port.set_relay_owner(relay_owner_idx);
            }
        }

        let port_name = port_info.name_bytes();

        // Log registration with class info
        if let Ok(name_str) = core::str::from_utf8(port_name) {
            if shmem_id != 0 {
                udebug!("devd", "port_registered";
                    name = name_str,
                    class = port_info.port_class.as_str(),
                    shmem_id = shmem_id
                );
            } else {
                udebug!("devd", "port_registered";
                    name = name_str,
                    class = port_info.port_class.as_str()
                );
            }
        }

        Ok(port_id)
    }

    /// Fire spawn rules for a port that transitioned to Claimed.
    ///
    /// For kernel bus ports (owner=0xFF): devd spawns directly.
    /// For driver-owned ports: devd sends SPAWN_CHILD to the owning driver,
    /// which spawns the child as its own child process.
    fn check_class_rules(&mut self, port_info: &abi::PortInfo, owner_idx: u8, port_id: u8) {
        let rule = match rules::find_port_rule(port_info) {
            Some(r) => r,
            None => return,
        };

        // Auto-mount: if rule has mount_path, register a Port mount
        if let Some(mount_path) = rule.mount_path {
            let port_name = port_info.name_bytes();
            let transport = mounts::MountTransport::Port {
                name: {
                    let mut n = [0u8; 32];
                    let len = port_name.len().min(32);
                    n[..len].copy_from_slice(&port_name[..len]);
                    n
                },
                name_len: port_name.len().min(32) as u8,
            };
            let _ = self.mounts.mount(mount_path.as_bytes(), transport, 0);
        }

        // Guard: don't spawn if a running service is already bound to this port.
        // Use port_id for unambiguous lookup (two ports can share a name).
        let port_link_id = self.ports.get_by_id(port_id)
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
            // Only respawn if Crashed or Failed — NOT Stopped (code 0 = hardware absent).
            let can_respawn = self.services.get(idx)
                .map(|s| matches!(s.state, ServiceState::Crashed { .. } | ServiceState::Failed { .. }))
                .unwrap_or(false);
            if !can_respawn {
                return;
            }
            // Cancel old instance's inline restart timer to prevent duplicate spawn
            if let Some(service) = self.services.get_mut(idx) {
                if service.has_restart_timer {
                    if let Some(events) = &self.events {
                        let _ = events.remove_timer(idx as u32);
                    }
                    service.has_restart_timer = false;
                }
            }
            if let Some(service) = self.services.get_mut(idx) {
                service.state = ServiceState::Stopped { code: 0 };
            }
        }

        // Allocate a link_id for this port↔service pairing
        let port_name = port_info.name_bytes();
        let link_id = self.alloc_link_id();
        if let Some(port) = self.ports.get_mut_by_id(port_id) {
            port.set_child_link_id(link_id);
        }

        uinfo!("devd", "spawn_driver";
            driver = rule.driver,
            port = core::str::from_utf8(port_info.name_bytes()).unwrap_or("?"),
            class = port_info.port_class.as_str()
        );

        // Expand rule context templates (if any)
        self.pending_context_kv_count = 0;
        if !rule.context.is_empty() {
            // Build lookup table for template expansion
            let mut lookup = TemplateLookup::new();
            lookup.add(b"trigger.name", port_info.name_bytes());

            let parent_port_id = port_info.parent_port_id;
            if parent_port_id != 0xFF {
                if let Some(parent) = self.ports.get_by_id(parent_port_id) {
                    lookup.add(b"block.name", parent.name());
                }
            }

            let count = rule.context.len().min(MAX_CONTEXT_KV);
            for i in 0..count {
                let (key, template) = rule.context[i];
                let mut value_buf = [0u8; MAX_CONTEXT_VALUE];
                let value_len = expand_template(template.as_bytes(), &mut value_buf, &lookup);
                self.pending_context_kvs[i].set(key.as_bytes(), &value_buf[..value_len]);
            }
            self.pending_context_kv_count = count as u8;
        }

        if owner_idx != 0xFF {
            // Driver-owned port: delegate spawn to the owning driver.
            // The child becomes the driver's child, not devd's.
            //
            // For grandchild ports, SpawnChild routes through the relay parent
            // (the driver with a direct query client connection to devd).
            let relay_idx = self.ports.get_by_id(port_id)
                .map(|p| p.relay_owner())
                .unwrap_or(owner_idx);
            //
            // SuperQ serialization: only one command in-flight per driver.
            // If the relay driver already has a pending SpawnChild, defer this rule.
            if self.has_inflight_superq_spawn(relay_idx) {
                uwarn!("devd", "spawn_deferred_inflight"; relay = relay_idx as u32, port_id = port_id as u32);
                self.defer_rule_fire(port_id, owner_idx);
                return;
            }
            // Build context so parent can answer GET_SPAWN_CONTEXT locally.
            let port_type = self.ports.get_by_id(port_id)
                .map(|p| p.port_type())
                .unwrap_or(0);
            let shmem_id = self.ports.get_by_id(port_id)
                .map(|p| p.shmem_id())
                .unwrap_or(0);
            let mut spawn_ctx = SpawnChildContext::empty();
            spawn_ctx.port_type = port_type;
            spawn_ctx.port_id = port_id;
            spawn_ctx.shmem_id = shmem_id;
            // Copy metadata from port
            if let Some(p) = self.ports.get_by_id(port_id) {
                let m = p.metadata();
                let mlen = m.len().min(64);
                spawn_ctx.metadata[..mlen].copy_from_slice(&m[..mlen]);
                spawn_ctx.metadata_len = mlen as u8;
            }
            // Copy pending context KVs
            let kv_n = (self.pending_context_kv_count as usize).min(4);
            for i in 0..kv_n {
                let k = self.pending_context_kvs[i].key();
                let v = self.pending_context_kvs[i].value();
                let klen = k.len().min(32);
                spawn_ctx.kv_keys[i][..klen].copy_from_slice(&k[..klen]);
                spawn_ctx.kv_keys_len[i] = klen as u8;
                let vlen = v.len().min(64);
                spawn_ctx.kv_values[i][..vlen].copy_from_slice(&v[..vlen]);
                spawn_ctx.kv_values_len[i] = vlen as u8;
            }
            spawn_ctx.kv_count = kv_n as u8;

            // Build full spawn path from port's parent chain.
            // resolve_path walks parent_port_id links to build e.g. "pcie:0/nvme:0"
            let mut spawn_path = [0u8; 96];
            let spawn_path_len = self.ports.resolve_path(port_id, &mut spawn_path);

            if let Some(seq_id) = self.query_handler.send_spawn_child_with_path(
                relay_idx, rule.driver.as_bytes(), port_name, rule.caps, rule.priority,
                Some(&spawn_ctx), &spawn_path[..spawn_path_len],
            ) {
                self.track_inflight_spawn(seq_id, port_type, port_name, rule.driver, rule.caps, link_id, port_id, relay_idx);
            } else {
                uerror!("devd", "spawn_child_send_failed"; driver = rule.driver, owner = relay_idx as u32);
            }
        } else {
            // Kernel bus port: devd spawns directly (no driver to delegate to)
            if let Some(idx) = existing_idx {
                // Update existing service's link_id to the new one
                if let Some(service) = self.services.get_mut(idx) {
                    service.link_id = link_id;
                    service.priority = rule.priority;
                }
                self.spawn_service(idx);
            } else {
                self.create_and_spawn(rule.driver, rule.caps, rule.priority, port_name, link_id);
            }
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
    fn track_inflight_spawn(&mut self, seq_id: u32, port_type: u8, port_name: &[u8], binary: &str, caps: u64, link_id: u32, trigger_port_id: u8, parent_svc_idx: u8) {
        // Find empty slot
        for entry in &mut self.inflight_spawns {
            if entry.seq_id == 0 {
                entry.seq_id = seq_id;
                entry.port_type = port_type;
                entry.trigger_port_id = trigger_port_id;
                let len = port_name.len().min(32);
                entry.port_name[..len].copy_from_slice(&port_name[..len]);
                entry.port_name_len = len as u8;
                let bin_len = binary.len().min(16);
                entry.binary_name[..bin_len].copy_from_slice(&binary.as_bytes()[..bin_len]);
                entry.binary_name_len = bin_len as u8;
                entry.service_idx = 0xFF;  // Not a service spawn
                entry.parent_service_idx = parent_svc_idx;
                entry.caps = caps;
                entry.link_id = link_id;
                // Copy pending context KVs
                let kv_count = self.pending_context_kv_count as usize;
                for i in 0..kv_count {
                    entry.context_kv[i] = self.pending_context_kvs[i];
                }
                entry.context_kv_count = self.pending_context_kv_count;
                self.pending_context_kv_count = 0;
                return;
            }
        }
    }

    /// Check if a parent driver's SuperQ is at capacity for inflight spawns.
    ///
    /// The SuperQ down-ring has 8 entries. Each SpawnChild message with ADDRESSED
    /// routing takes ~3 FORWARD notes. Allow 2 concurrent inflight spawns per
    /// parent (6 notes), leaving headroom in the 8-entry ring.
    fn has_inflight_superq_spawn(&self, parent_svc_idx: u8) -> bool {
        const MAX_INFLIGHT_PER_PARENT: usize = 2;
        let count = self.inflight_spawns.iter()
            .filter(|e| e.seq_id != 0 && e.parent_service_idx == parent_svc_idx)
            .count();
        count >= MAX_INFLIGHT_PER_PARENT
    }

    /// Queue a deferred rule fire for a port whose parent SuperQ is busy.
    fn defer_rule_fire(&mut self, port_id: u8, owner_idx: u8) {
        for entry in &mut self.deferred_rules {
            if !entry.active {
                *entry = DeferredRuleFire { port_id, owner_idx, active: true };
                return;
            }
        }
        uwarn!("devd", "deferred_rules_full"; port_id = port_id as u32, owner = owner_idx as u32);
    }

    /// Drain deferred rules for a parent driver whose SuperQ has capacity.
    fn drain_deferred_rules(&mut self, parent_svc_idx: u8) {
        // Fire ONE deferred rule for this parent. If there are more, they'll be
        // drained on the next spawn_ack.
        for i in 0..MAX_DEFERRED_RULES {
            if self.deferred_rules[i].active && self.deferred_rules[i].owner_idx == parent_svc_idx {
                let port_id = self.deferred_rules[i].port_id;
                let owner_idx = self.deferred_rules[i].owner_idx;
                self.deferred_rules[i] = DeferredRuleFire::empty();
                if let Some(port) = self.ports.get_by_id(port_id) {
                    let info = *port.port_info();
                    self.check_class_rules(&info, owner_idx, port_id);
                }
                return;
            }
        }
    }

    /// Consume an in-flight spawn by seq_id.
    ///
    /// Returns (port_type, port_name, port_len, binary_name, binary_len, service_idx, caps, link_id, trigger_port_id).
    /// Any context KV pairs from the inflight spawn are moved to `pending_context_kvs`.
    fn consume_inflight_spawn(&mut self, seq_id: u32) -> Option<(u8, [u8; 32], u8, [u8; 16], u8, u8, u64, u32, u8)> {
        for entry in &mut self.inflight_spawns {
            if entry.seq_id == seq_id {
                let result = (entry.port_type, entry.port_name, entry.port_name_len,
                              entry.binary_name, entry.binary_name_len, entry.service_idx,
                              entry.caps, entry.link_id, entry.trigger_port_id);
                // Move context KVs to pending
                let kv_count = entry.context_kv_count as usize;
                for i in 0..kv_count {
                    self.pending_context_kvs[i] = entry.context_kv[i];
                }
                self.pending_context_kv_count = entry.context_kv_count;
                *entry = InflightSpawn::empty();
                return Some(result);
            }
        }
        None
    }

    /// Store spawn context for a child PID (with optional metadata from port registration)
    fn store_spawn_context(&mut self, pid: u32, port_type: u8, port_name: &[u8], metadata: &[u8], trigger_port_id: u8) {
        self.store_spawn_context_with_kv(pid, port_type, port_name, metadata, trigger_port_id, &[]);
    }

    /// Store spawn context with context key-value pairs from rule template expansion
    fn store_spawn_context_with_kv(
        &mut self,
        pid: u32,
        port_type: u8,
        port_name: &[u8],
        metadata: &[u8],
        trigger_port_id: u8,
        context_kvs: &[(&[u8], &[u8])],
    ) {
        // Find empty slot or oldest entry
        let mut empty_slot = None;
        for (i, entry) in self.spawn_contexts.iter_mut().enumerate() {
            if entry.pid == 0 {
                empty_slot = Some(i);
                break;
            }
        }

        let slot = empty_slot.unwrap_or(0); // Overwrite first slot if full
        self.spawn_contexts[slot] = SpawnContext::empty();
        self.spawn_contexts[slot].pid = pid;
        self.spawn_contexts[slot].port_type = port_type;
        self.spawn_contexts[slot].trigger_port_id = trigger_port_id;
        let len = port_name.len().min(32);
        self.spawn_contexts[slot].port_name[..len].copy_from_slice(&port_name[..len]);
        self.spawn_contexts[slot].port_name_len = len as u8;
        let meta_len = metadata.len().min(64);
        self.spawn_contexts[slot].metadata[..meta_len].copy_from_slice(&metadata[..meta_len]);
        self.spawn_contexts[slot].metadata_len = meta_len as u8;

        // Store context KV pairs
        let kv_count = context_kvs.len().min(MAX_CONTEXT_KV);
        for (i, (key, value)) in context_kvs[..kv_count].iter().enumerate() {
            self.spawn_contexts[slot].context_kv[i].set(key, value);
        }
        self.spawn_contexts[slot].context_kv_count = kv_count as u8;
    }

    /// Get spawn context for a PID
    /// Returns (port_type, port_name, metadata, trigger_port_id)
    fn get_spawn_context(&self, pid: u32) -> Option<(u8, &[u8], &[u8], u8)> {
        for entry in &self.spawn_contexts {
            if entry.pid == pid {
                return Some((
                    entry.port_type,
                    &entry.port_name[..entry.port_name_len as usize],
                    &entry.metadata[..entry.metadata_len as usize],
                    entry.trigger_port_id,
                ));
            }
        }
        None
    }

    /// Get full spawn context for a PID, including context KV pairs
    fn get_spawn_context_full(&self, pid: u32) -> Option<&SpawnContext> {
        self.spawn_contexts.iter().find(|e| e.pid == pid)
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
        // Start with no timeout — the Mux blocks until an event fires.
        // update_timeout() will enable 100ms polling if overflow clients appear.
        self.update_timeout();

        loop {
            let events = self.events.as_ref().expect("devd: events not initialized");
            let wait_result = events.wait_event();

            match wait_result {
                Ok(event) => {
                    // Inline timer event (restart timers + admin request timeouts)
                    if (event.event & abi::mux_filter::TIMER) != 0 {
                        let tag = event.handle.raw();
                        if tag < MAX_SERVICES as u32 {
                            self.handle_restart_timer(tag as usize);
                        } else if tag >= 0x100 && tag < 0x100 + MAX_PENDING_REQUESTS as u32 {
                            self.handle_admin_request_timeout((tag - 0x100) as usize);
                        }
                    }
                    // Signal event (handle is INVALID)
                    else if event.is_signal() {
                        self.handle_signal_event(&event);
                        // Fall through to poll/flush below
                    } else {
                        let handle = event.handle;
                        // Exception channel event? (child faulted)
                        if self.services.find_by_exc_channel(handle.raw()).is_some() {
                            self.handle_exception_event(handle);
                        }
                        // Query port event?
                        else if let Some(query_port) = &self.query_port {
                            if handle == query_port.handle() {
                                self.handle_query_port_event();
                            } else if self.query_handler.find_by_handle(handle).is_some() {
                                // Query client message (or supervision channel close)
                                self.handle_query_client_event(handle);
                            }
                        } else if self.query_handler.find_by_handle(handle).is_some() {
                            self.handle_query_client_event(handle);
                        }
                    }
                }
                Err(SysError::Timeout) | Err(SysError::WouldBlock) => {
                    // Timeout fired or spurious wake — poll query clients
                }
                Err(_e) => {
                    uerror!("devd", "wait_failed";);
                }
            }

            // Poll overflow query clients (skipped when all are Mux-watched)
            self.poll_query_clients();

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

    unotice!("devd", "started"; services = devd.services.count() as u32);
    userlib::ulog::flush();
    devd.run()
}
