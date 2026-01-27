//! Device Supervisor Daemon (devd)
//!
//! Service registry and supervisor for the microkernel. Acts as PID 1 (init).
//! Uses only the unified 5-syscall interface (open, read, write, map, close).
//!
//! ## Responsibilities
//! - Track all services and their registered ports
//! - Route client queries to correct service
//! - Enforce startup dependencies (requires/depends_on)
//! - Prune service branch on crash, then restart
//! - Supervise with exponential backoff
//!
//! ## Architecture
//!
//! devd uses a modular, trait-based design:
//! - `service.rs` - Service state machine (ServiceManager trait)
//! - `ports.rs` - Port registry (PortRegistry trait)
//! - `process.rs` - Process management (ProcessManager trait)
//! - `deps.rs` - Dependency resolution (DependencyResolver trait)
//!
//! Each module is testable independently via trait mocking.

#![no_std]
#![no_main]

mod service;
mod ports;
mod process;
mod deps;
mod devices;
mod query;
mod rules;

use userlib::syscall::{self, LogLevel};
use userlib::ipc::{Port, Timer, EventLoop, ObjHandle, Channel};
use userlib::error::{SysError, SysResult};

use service::{
    ServiceState, ServiceManager, ServiceRegistry, PortDef,
    MAX_SERVICES, MAX_PORTS_PER_SERVICE, MAX_RESTARTS,
    INITIAL_BACKOFF_MS, MAX_BACKOFF_MS, FAILED_RETRY_MS,
};
use ports::{PortRegistry, Ports, PortType};
use process::{ProcessManager, SyscallProcessManager};
use deps::{DependencyResolver, Dependencies};
use devices::{DeviceStore, DeviceRegistry};
use query::{QueryHandler, MSG_BUFFER_SIZE};
use rules::{RulesEngine, StaticRules};

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

macro_rules! dlog {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        const PREFIX: &[u8] = b"[devd] ";
        let mut buf = [0u8; 128];
        buf[..PREFIX.len()].copy_from_slice(PREFIX);
        let mut pos = PREFIX.len();
        struct W<'a> { b: &'a mut [u8], p: &'a mut usize }
        impl core::fmt::Write for W<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                for &b in s.as_bytes() {
                    if *self.p < self.b.len() { self.b[*self.p] = b; *self.p += 1; }
                }
                Ok(())
            }
        }
        let _ = write!(W { b: &mut buf, p: &mut pos }, $($arg)*);
        syscall::klog(LogLevel::Info, &buf[..pos]);
    }};
}

macro_rules! derror {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        const PREFIX: &[u8] = b"[devd] ";
        let mut buf = [0u8; 128];
        buf[..PREFIX.len()].copy_from_slice(PREFIX);
        let mut pos = PREFIX.len();
        struct W<'a> { b: &'a mut [u8], p: &'a mut usize }
        impl core::fmt::Write for W<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                for &b in s.as_bytes() {
                    if *self.p < self.b.len() { self.b[*self.p] = b; *self.p += 1; }
                }
                Ok(())
            }
        }
        let _ = write!(W { b: &mut buf, p: &mut pos }, $($arg)*);
        syscall::klog(LogLevel::Error, &buf[..pos]);
    }};
}

// =============================================================================
// Devd - Service Supervisor
// =============================================================================

/// Maximum number of recently spawned dynamic PIDs to track
const MAX_RECENT_DYNAMIC_PIDS: usize = 8;

/// Maximum number of pending spawn commands per driver
const MAX_PENDING_SPAWNS_PER_DRIVER: usize = 4;

/// Maximum number of drivers that can have pending spawns
const MAX_DRIVERS_WITH_PENDING: usize = 8;

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
}

impl SpawnContext {
    const fn empty() -> Self {
        Self {
            pid: 0,
            port_type: 0,
            port_name: [0u8; 32],
            port_name_len: 0,
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
        }
    }
}

/// A pending spawn command waiting for driver to report Ready
#[derive(Clone, Copy)]
struct PendingSpawn {
    /// Service index of the driver that should spawn
    driver_idx: u8,
    /// Binary name to spawn (index into static strings)
    binary: &'static str,
    /// Trigger port name
    port_name: [u8; 32],
    /// Length of port name
    port_name_len: u8,
}

impl PendingSpawn {
    const fn empty() -> Self {
        Self {
            driver_idx: 0xFF,
            binary: "",
            port_name: [0u8; 32],
            port_name_len: 0,
        }
    }
}

pub struct Devd {
    /// Our port for services to announce ready
    port: Option<Port>,
    /// Query port for device queries
    query_port: Option<Port>,
    /// Event loop
    events: Option<EventLoop>,
    /// Service registry
    services: ServiceRegistry,
    /// Port registry
    ports: Ports,
    /// Process manager
    process_mgr: SyscallProcessManager,
    /// Dependency resolver
    deps: Dependencies,
    /// Device registry (for hierarchical queries)
    devices: DeviceRegistry,
    /// Query handler for client connections
    query_handler: QueryHandler,
    /// Rules engine for auto-spawning drivers
    rules: StaticRules,
    /// Admin clients (shell text commands)
    admin_clients: AdminClients,
    /// Recently spawned dynamic PIDs (workaround for spawn-before-slot-setup race)
    /// Maps PID -> service_idx for dynamic drivers that haven't connected yet
    recent_dynamic_pids: [(u32, u8); MAX_RECENT_DYNAMIC_PIDS],
    /// Pending spawn commands waiting for drivers to report Ready
    /// Keyed by driver service index
    pending_spawns: [PendingSpawn; MAX_DRIVERS_WITH_PENDING * MAX_PENDING_SPAWNS_PER_DRIVER],
    /// Spawn context: maps child PID to trigger port (for GET_SPAWN_CONTEXT)
    spawn_contexts: [SpawnContext; MAX_SPAWN_CONTEXTS],
    /// In-flight spawn commands: maps seq_id to port info
    inflight_spawns: [InflightSpawn; MAX_INFLIGHT_SPAWNS],
}

impl Devd {
    pub const fn new() -> Self {
        Self {
            port: None,
            query_port: None,
            events: None,
            services: ServiceRegistry::new(),
            ports: Ports::new(),
            process_mgr: SyscallProcessManager::new(),
            deps: Dependencies::new(),
            devices: DeviceRegistry::new(),
            query_handler: QueryHandler::new(),
            rules: StaticRules::new(),
            admin_clients: AdminClients::new(),
            recent_dynamic_pids: [(0, 0); MAX_RECENT_DYNAMIC_PIDS],
            pending_spawns: [const { PendingSpawn::empty() }; MAX_DRIVERS_WITH_PENDING * MAX_PENDING_SPAWNS_PER_DRIVER],
            spawn_contexts: [const { SpawnContext::empty() }; MAX_SPAWN_CONTEXTS],
            inflight_spawns: [const { InflightSpawn::empty() }; MAX_INFLIGHT_SPAWNS],
        }
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    fn now_ms() -> u64 {
        syscall::gettime() / 1_000_000
    }

    // =========================================================================
    // Initialization
    // =========================================================================

    pub fn init(&mut self) -> SysResult<()> {
        syscall::klog(LogLevel::Info, b"[devd] init: creating EventLoop");
        let mut events = EventLoop::new()?;

        syscall::klog(LogLevel::Info, b"[devd] init: registering port devd:");
        let port = Port::register(b"devd:")?;
        syscall::klog(LogLevel::Info, b"[devd] init: watching port");
        events.watch(port.handle())?;

        syscall::klog(LogLevel::Info, b"[devd] init: registering query port devd-query:");
        let query_port = Port::register(b"devd-query:")?;
        syscall::klog(LogLevel::Info, b"[devd] init: watching query port");
        events.watch(query_port.handle())?;

        self.port = Some(port);
        self.query_port = Some(query_port);
        self.events = Some(events);

        syscall::klog(LogLevel::Info, b"[devd] init: registering in internal registry");
        self.ports.register_typed(b"devd:", 0xFF, PortType::Service)?; // 0xFF = devd itself
        self.ports.register_typed(b"devd-query:", 0xFF, PortType::Service)?;

        syscall::klog(LogLevel::Info, b"[devd] init: initializing services");
        self.services.init_from_defs();

        Ok(())
    }

    // =========================================================================
    // Dependency Resolution
    // =========================================================================

    fn check_pending_services(&mut self) {
        syscall::klog(LogLevel::Info, b"[devd] check_pending_services");
        let now = Self::now_ms();

        // Collect indices of services to spawn
        let mut to_spawn = [None; MAX_SERVICES];
        let mut spawn_count = 0;

        self.services.for_each(|i, service| {
            if service.state == ServiceState::Pending
                && self.deps.satisfied(service, &self.ports)
            {
                if spawn_count < MAX_SERVICES {
                    to_spawn[spawn_count] = Some(i);
                    spawn_count += 1;
                }
            }
        });

        // Spawn collected services
        for idx in to_spawn.iter().flatten().copied() {
            dlog!("spawning service idx={}", idx);
            self.spawn_service(idx, now);
        }
    }

    // =========================================================================
    // Service Spawning
    // =========================================================================

    fn spawn_service(&mut self, idx: usize, now: u64) {
        if idx >= MAX_SERVICES {
            return;
        }

        // Get binary name
        let binary = match self.services.get(idx).and_then(|s| s.def()) {
            Some(d) => d.binary,
            None => return,
        };

        // Spawn the process
        let (pid, watcher) = match self.process_mgr.spawn(binary) {
            Ok(result) => result,
            Err(e) => {
                derror!("spawn {} failed err={:?}", binary, e);
                self.services.transition(idx, ServiceState::Failed { code: -1 }, now);
                return;
            }
        };

        // Add watcher to event loop
        if let Some(events) = &mut self.events {
            let _ = events.watch(watcher.handle());
        }

        // Collect port definitions before mutable borrow
        let port_defs: [Option<PortDef>; MAX_PORTS_PER_SERVICE] = {
            match self.services.get(idx).and_then(|s| s.def()) {
                Some(def) => {
                    let mut arr = [None; MAX_PORTS_PER_SERVICE];
                    for (i, pd) in def.registers.iter().enumerate() {
                        if i < MAX_PORTS_PER_SERVICE {
                            arr[i] = Some(*pd);
                        }
                    }
                    arr
                }
                None => return,
            }
        };

        // Update service state
        if let Some(service) = self.services.get_mut(idx) {
            service.pid = pid;
            service.watcher = Some(watcher);
            service.state = ServiceState::Starting;
            service.last_change = now;
        }

        dlog!("spawned {} pid={}", binary, pid);

        // Pre-register ports this service will provide (not available yet)
        for pd in port_defs.iter().flatten() {
            let _ = self.ports.register_typed(pd.name, idx as u8, PortType::from_u8(pd.port_type));
            self.ports.set_availability(pd.name, false);
        }

        // For services that don't register ports, mark Ready immediately
        self.mark_portless_services_ready();
    }

    // =========================================================================
    // Service State Changes
    // =========================================================================

    fn handle_service_exit(&mut self, idx: usize, code: i32) {
        if idx >= MAX_SERVICES {
            return;
        }

        let now = Self::now_ms();

        // Collect info about the exit
        #[derive(Clone, Copy)]
        enum ExitAction {
            Stopped,
            Crashed { backoff_ms: u32 },
            Failed,
        }

        // First pass: collect info and update service state
        let (port_names, should_prune, action, old_pid) = {
            let service = match self.services.get_mut(idx) {
                Some(s) => s,
                None => return,
            };

            // Copy name to local buffer (name() borrows dynamic_name)
            let mut name_buf = [0u8; 16];
            let name_str = service.name();
            let name_len = name_str.len().min(16);
            name_buf[..name_len].copy_from_slice(&name_str.as_bytes()[..name_len]);

            let auto_restart = service.def().map(|d| d.auto_restart).unwrap_or(false);
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

            // Collect port names
            let mut port_arr = [None; MAX_PORTS_PER_SERVICE];
            if let Some(def) = service.def() {
                for (i, pd) in def.registers.iter().enumerate() {
                    if i < MAX_PORTS_PER_SERVICE {
                        port_arr[i] = Some(pd.name);
                    }
                }
            }

            if code == 0 {
                service.state = ServiceState::Stopped { code: 0 };
                service.last_change = now;
                (port_arr, false, ExitAction::Stopped, old_pid)
            } else {
                service.total_restarts = service.total_restarts.saturating_add(1);

                let restarts = match service.state {
                    ServiceState::Crashed { restarts, .. } => restarts + 1,
                    _ => 1,
                };

                if restarts >= MAX_RESTARTS || !auto_restart {
                    service.state = ServiceState::Failed { code };
                    service.last_change = now;
                    let name = core::str::from_utf8(&name_buf[..name_len]).unwrap_or("?");
                    derror!("{} failed code={} restarts={}", name, code, service.total_restarts);
                    (port_arr, false, if auto_restart { ExitAction::Failed } else { ExitAction::Stopped }, old_pid)
                } else {
                    service.state = ServiceState::Crashed { code, restarts };
                    service.last_change = now;
                    service.backoff_ms = (service.backoff_ms * 2).min(MAX_BACKOFF_MS);
                    (port_arr, true, ExitAction::Crashed { backoff_ms: service.backoff_ms }, old_pid)
                }
            }
        };

        // Clean up recent_dynamic_pids tracking for this PID
        if old_pid != 0 {
            self.remove_recent_dynamic_pid(old_pid);
        }

        // Unregister ports
        for port_name in port_names.iter().flatten() {
            self.ports.unregister(port_name);
        }

        // Prune children if needed
        if should_prune {
            self.prune_dependents(idx);
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

    fn handle_service_ready(&mut self, idx: usize) {
        if idx >= MAX_SERVICES {
            return;
        }

        let now = Self::now_ms();

        // Update service state and collect port names
        let port_names: [Option<&'static [u8]>; MAX_PORTS_PER_SERVICE] = {
            let service = match self.services.get_mut(idx) {
                Some(s) => s,
                None => return,
            };

            let mut arr = [None; MAX_PORTS_PER_SERVICE];
            if let Some(def) = service.def() {
                for (i, pd) in def.registers.iter().enumerate() {
                    if i < MAX_PORTS_PER_SERVICE {
                        arr[i] = Some(pd.name);
                    }
                }
            }

            dlog!("service {} state: {:?} -> Ready", service.name(), service.state);
            service.state = ServiceState::Ready;
            service.last_change = now;
            service.backoff_ms = INITIAL_BACKOFF_MS;
            arr
        };

        // Mark ports available
        for port_name in port_names.iter().flatten() {
            self.ports.set_availability(port_name, true);
        }

        // Check pending services
        self.check_pending_services();
    }

    // =========================================================================
    // Branch Pruning
    // =========================================================================

    fn prune_dependents(&mut self, provider_idx: usize) {
        if provider_idx >= MAX_SERVICES {
            return;
        }

        let dependents = self.deps.find_dependents(provider_idx, &self.services);

        // Kill dependents recursively (depth-first)
        for dep_idx in dependents.iter() {
            self.prune_dependents(dep_idx);
            self.kill_service(dep_idx);
        }
    }

    fn kill_service(&mut self, idx: usize) {
        if idx >= MAX_SERVICES {
            return;
        }

        let now = Self::now_ms();

        // Get service info
        let (pid, port_names) = {
            let service = match self.services.get_mut(idx) {
                Some(s) => s,
                None => return,
            };

            if !service.state.is_running() {
                return;
            }

            let name = service.name();
            let pid = service.pid;

            dlog!("killing {} pid={} (branch prune)", name, pid);

            // Clean up watcher
            if let Some(watcher) = service.watcher.take() {
                if let Some(events) = &mut self.events {
                    let _ = events.unwatch(watcher.handle());
                }
            }

            // Collect port names
            let mut arr = [None; MAX_PORTS_PER_SERVICE];
            if let Some(def) = service.def() {
                for (i, pd) in def.registers.iter().enumerate() {
                    if i < MAX_PORTS_PER_SERVICE {
                        arr[i] = Some(pd.name);
                    }
                }
            }

            service.pid = 0;
            service.state = ServiceState::Pending;
            service.last_change = now;
            (pid, arr)
        };

        // Kill the process
        if pid != 0 {
            let _ = self.process_mgr.kill(pid);
            // Clean up recent_dynamic_pids tracking for this PID
            self.remove_recent_dynamic_pid(pid);
        }

        // Unregister ports
        for port_name in port_names.iter().flatten() {
            self.ports.unregister(port_name);
        }

        // Remove devices registered by this service
        let removed = self.devices.remove_by_owner(idx as u8);
        if removed > 0 {
            dlog!("removed {} devices from service idx={}", removed, idx);
        }
    }

    // =========================================================================
    // Event Handling
    // =========================================================================

    fn handle_port_event(&mut self) {
        let port = match &mut self.port {
            Some(p) => p,
            None => return,
        };

        match port.accept_with_pid() {
            Ok((channel, client_pid)) => {
                // Find which service connected by PID
                let service_idx = self.services.find_by_pid(client_pid)
                    .filter(|&i| {
                        self.services.get(i)
                            .map(|s| s.state == ServiceState::Starting)
                            .unwrap_or(false)
                    });

                if let Some(idx) = service_idx {
                    // Store channel and add to event loop
                    if let Some(events) = &mut self.events {
                        let _ = events.watch(channel.handle());
                    }
                    if let Some(service) = self.services.get_mut(idx) {
                        service.channel = Some(channel);
                    }
                    // Mark service as Ready
                    self.handle_service_ready(idx);
                } else {
                    // Unknown connection - treat as admin client (shell)
                    if let Some(events) = &mut self.events {
                        let _ = events.watch(channel.handle());
                    }
                    match self.admin_clients.add(channel) {
                        Some(slot) => {
                            dlog!("admin client connected slot={} pid={}", slot, client_pid);
                        }
                        None => {
                            dlog!("admin client rejected (full) pid={}", client_pid);
                        }
                    }
                }
            }
            Err(SysError::WouldBlock) => {}
            Err(e) => {
                derror!("devd: accept failed err={:?}", e);
            }
        }
    }

    fn mark_portless_services_ready(&mut self) {
        let now = Self::now_ms();

        self.services.for_each_mut(|_i, service| {
            let registers_empty = service.def().map(|d| d.registers.is_empty()).unwrap_or(true);
            if service.state == ServiceState::Starting
                && registers_empty
                && service.pid != 0
            {
                dlog!("service {} state: Starting -> Ready (portless)", service.name());
                service.state = ServiceState::Ready;
                service.last_change = now;
                service.backoff_ms = INITIAL_BACKOFF_MS;
            }
        });
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
        let response = self.handle_admin_command(&buf[..len]);

        // Send response
        if let Some(client) = self.admin_clients.get_mut(slot) {
            let _ = client.channel.send(response);
        }
    }

    fn handle_admin_command(&mut self, cmd: &[u8]) -> &'static [u8] {
        // Trim whitespace and newline
        let cmd = trim_bytes(cmd);

        if cmd.starts_with(b"START ") {
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
        } else {
            b"ERR unknown command\n"
        }
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

        // Spawn it
        let now = Self::now_ms();
        self.spawn_service(idx, now);

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
        if let Some(service) = self.services.get(idx) {
            if !service.state.is_running() {
                return b"ERR not running\n";
            }
        }

        self.kill_service(idx);
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

        // Kill if running
        if let Some(service) = self.services.get(idx) {
            if service.state.is_running() {
                self.kill_service(idx);
            }
        }

        // Reset state to Pending so it can be spawned
        let now = Self::now_ms();
        if let Some(service) = self.services.get_mut(idx) {
            service.state = ServiceState::Pending;
            service.last_change = now;
        }

        // Spawn it
        self.spawn_service(idx, now);
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

    fn remove_admin_client(&mut self, slot: usize) {
        if let Some(channel) = self.admin_clients.remove(slot) {
            if let Some(events) = &mut self.events {
                let _ = events.unwatch(channel.handle());
            }
            dlog!("admin client disconnected slot={}", slot);
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
                    // Copy name to local buffer before modifying service
                    let mut name_buf = [0u8; 16];
                    let name_str = service.name();
                    let name_len = name_str.len().min(16);
                    name_buf[..name_len].copy_from_slice(&name_str.as_bytes()[..name_len]);

                    service.backoff_ms = INITIAL_BACKOFF_MS;
                    service.state = ServiceState::Pending;
                    service.last_change = now;

                    let name = core::str::from_utf8(&name_buf[..name_len]).unwrap_or("?");
                    dlog!("{} retry after failed (total_restarts={})", name, service.total_restarts);
                    false
                }
                _ => false,
            }
        };

        if should_spawn {
            self.spawn_service(idx, now);
        } else {
            self.check_pending_services();
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
                // Check if this is a known service (driver)
                // First try the normal service registry lookup for Ready services
                let mut service_idx = self.services.find_by_pid(client_pid)
                    .filter(|&i| {
                        self.services.get(i)
                            .map(|s| s.state == ServiceState::Ready)
                            .unwrap_or(false)
                    })
                    .map(|i| i as u8);

                // Also check for Starting services - connecting to query port means
                // the service is ready to communicate, so transition to Ready
                if service_idx.is_none() {
                    if let Some(idx) = self.services.find_by_pid(client_pid) {
                        let is_starting = self.services.get(idx)
                            .map(|s| s.state == ServiceState::Starting)
                            .unwrap_or(false);
                        if is_starting {
                            // Transition to Ready and recognize as a driver
                            self.handle_service_ready(idx);
                            service_idx = Some(idx as u8);
                            dlog!("query: service {} transitioned Starting -> Ready via query port",
                                self.services.get(idx).map(|s| s.name()).unwrap_or("?"));
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
                        dlog!("query: found pid={} in recent_dynamic_pids idx={}", client_pid, idx);
                    }
                }

                // Add to event loop
                if let Some(events) = &mut self.events {
                    let _ = events.watch(channel.handle());
                }

                // Add to query handler
                match self.query_handler.add_client(channel, service_idx, client_pid) {
                    Some(slot) => {
                        let client_type = if service_idx.is_some() { "driver" } else { "client" };
                        dlog!("query: {} connected (slot={}, pid={}, svc_idx={:?})",
                            client_type, slot, client_pid, service_idx);
                    }
                    None => {
                        derror!("query: too many clients, rejecting pid={}", client_pid);
                    }
                }
            }
            Err(SysError::WouldBlock) => {}
            Err(e) => {
                derror!("query: accept failed err={:?}", e);
            }
        }
    }

    fn handle_query_client_event(&mut self, handle: ObjHandle) {
        use userlib::query::{msg, QueryHeader};

        let slot = match self.query_handler.find_by_handle(handle) {
            Some(s) => s,
            None => return,
        };

        // Read message from client
        let mut recv_buf = [0u8; MSG_BUFFER_SIZE];
        let mut response_buf = [0u8; MSG_BUFFER_SIZE];

        let client = match self.query_handler.get_mut(slot) {
            Some(c) => c,
            None => return,
        };

        match client.channel.recv(&mut recv_buf) {
            Ok(len) if len > 0 => {
                // Check message type for special handling
                let msg_type = QueryHeader::from_bytes(&recv_buf[..len])
                    .map(|h| h.msg_type);

                match msg_type {
                    Some(msg::REGISTER_PORT) => {
                        // Handle port registration specially
                        self.handle_port_register_msg(slot, &recv_buf[..len]);
                    }
                    Some(msg::STATE_CHANGE) => {
                        // Handle driver state change
                        self.handle_state_change_msg(slot, &recv_buf[..len]);
                    }
                    Some(msg::SPAWN_ACK) => {
                        // Handle spawn acknowledgement from driver
                        self.handle_spawn_ack_msg(slot, &recv_buf[..len]);
                    }
                    Some(msg::QUERY_DRIVER) => {
                        // Forward to driver
                        self.handle_query_driver_forward(slot, &recv_buf[..len]);
                    }
                    Some(msg::GET_SPAWN_CONTEXT) => {
                        // Handle spawn context query from child
                        self.handle_get_spawn_context(slot, &recv_buf[..len]);
                    }
                    Some(msg::QUERY_PORT) => {
                        // Handle port info query
                        self.handle_query_port(slot, &recv_buf[..len]);
                    }
                    Some(msg::UPDATE_PORT_SHMEM_ID) => {
                        // Handle update shmem_id for existing port
                        self.handle_update_port_shmem_id(slot, &recv_buf[..len]);
                    }
                    Some(msg::LIST_PORTS) => {
                        // Handle port list query
                        self.handle_list_ports(slot, &recv_buf[..len]);
                    }
                    Some(msg::LIST_SERVICES) => {
                        // Handle service list query
                        self.handle_list_services(slot, &recv_buf[..len]);
                    }
                    _ => {
                        // Process normally
                        let response_len = self.query_handler.handle_message(
                            slot,
                            &recv_buf[..len],
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
            Ok(_) => {
                // Empty read or EOF - client disconnected
                self.remove_query_client(slot);
            }
            Err(SysError::WouldBlock) => {}
            Err(SysError::PeerClosed) => {
                // Expected - client disconnected after query completed
                self.remove_query_client(slot);
            }
            Err(e) => {
                derror!("query: recv failed slot={} err={:?}", slot, e);
                self.remove_query_client(slot);
            }
        }
    }

    fn handle_port_register_msg(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{error, port_type};

        // Parse the registration message
        let info = match self.query_handler.parse_port_register(slot, buf) {
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

        // Convert port_type u8 to PortType enum
        let port_type_enum = PortType::from_u8(info.port_type);

        // Register the port
        let result = self.handle_port_registration(
            info.name,
            port_type_enum,
            info.parent,
            info.owner_idx,
            info.shmem_id,
        );

        // Send response
        let result_code = match result {
            Ok(()) => error::OK,
            Err(e) => {
                if let Ok(name_str) = core::str::from_utf8(info.name) {
                    derror!("port registration failed: {} err={:?} owner={}", name_str, e, info.owner_idx);
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
                derror!("invalid STATE_CHANGE message from slot={}", slot);
                return;
            }
        };

        // Get the driver's service index from the query client
        let driver_idx = match self.query_handler.get_service_idx(slot) {
            Some(idx) => idx,
            None => {
                dlog!("STATE_CHANGE from non-driver slot={}", slot);
                return;
            }
        };

        dlog!("driver {} state_change: {}", driver_idx, state_msg.new_state);

        // When driver reports Ready, send any pending spawn commands
        if state_msg.new_state == driver_state::READY {
            self.flush_pending_spawns(driver_idx);
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
                derror!("invalid GET_SPAWN_CONTEXT message from slot={}", slot);
                return;
            }
        };

        // Get the client's PID
        let client_pid = match self.query_handler.get(slot) {
            Some(client) => client.pid,
            None => {
                derror!("GET_SPAWN_CONTEXT from unknown slot={}", slot);
                return;
            }
        };

        // Look up spawn context for this PID
        let mut response_buf = [0u8; 128];
        let resp_len = if let Some((port_type, port_name)) = self.get_spawn_context(client_pid) {
            dlog!("GET_SPAWN_CONTEXT for pid={}: port={}", client_pid,
                core::str::from_utf8(port_name).unwrap_or("?"));

            let resp = SpawnContextResponse::new(header.seq_id, error::OK, port_type);
            resp.write_to(&mut response_buf, port_name)
                .unwrap_or(SpawnContextResponse::HEADER_SIZE)
        } else {
            dlog!("GET_SPAWN_CONTEXT for pid={}: no context", client_pid);
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
                derror!("invalid QUERY_PORT message from slot={}", slot);
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
                port.port_type() as u8,
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
                derror!("invalid UPDATE_PORT_SHMEM_ID message from slot={}", slot);
                return;
            }
        };

        let seq_id = update.header.seq_id;

        // Update the port's shmem_id
        let result = self.ports.set_shmem_id(port_name, update.shmem_id);

        let result_code = match result {
            Ok(()) => {
                dlog!("port {:?} shmem_id updated to {}",
                    core::str::from_utf8(port_name).unwrap_or("?"),
                    update.shmem_id);
                error::OK
            }
            Err(_) => {
                derror!("failed to update shmem_id for port {:?}",
                    core::str::from_utf8(port_name).unwrap_or("?"));
                error::NOT_FOUND
            }
        };

        // Send response (reuse PortRegisterResponse format)
        let resp = PortRegisterResponse::new(seq_id, result_code);
        if let Some(client) = self.query_handler.get_mut(slot) {
            let _ = client.channel.send(&resp.to_bytes());
        }
    }

    /// Handle LIST_PORTS message - returns all registered ports
    fn handle_list_ports(&mut self, slot: usize, buf: &[u8]) {
        use userlib::query::{QueryHeader, PortsListResponse, PortEntry, port_flags};

        let header = match QueryHeader::from_bytes(buf) {
            Some(h) => h,
            None => {
                derror!("invalid LIST_PORTS message from slot={}", slot);
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
                port_type: port.port_type() as u8,
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
                derror!("invalid LIST_SERVICES message from slot={}", slot);
                return;
            }
        };

        let seq_id = header.seq_id;

        // Build service entries
        let mut entries = [[0u8; ServiceEntry::SIZE]; 16];
        let mut count = 0usize;

        self.services.for_each(|idx, service| {
            if count >= 16 {
                return;
            }

            // Build entry
            let mut name = [0u8; 16];
            let name_str = service.name().as_bytes();
            let name_len = name_str.len().min(16);
            name[..name_len].copy_from_slice(&name_str[..name_len]);

            // Map state to constant
            let state = match service.state {
                ServiceState::Pending => service_state::PENDING,
                ServiceState::Starting => service_state::STARTING,
                ServiceState::Ready => service_state::READY,
                ServiceState::Stopped { .. } => service_state::STOPPED,
                ServiceState::Crashed { .. } => service_state::CRASHED,
                ServiceState::Failed { .. } => service_state::FAILED,
            };

            let entry = ServiceEntry {
                name,
                pid: service.pid,
                state,
                index: idx as u8,
                parent_idx: service.parent.unwrap_or(0xFF),
                child_count: service.child_count,
                total_restarts: service.total_restarts,
                last_change: service.last_change as u32,
            };

            entries[count] = entry.to_bytes();
            count += 1;
        });

        // Build response
        let resp_header = ServicesListResponse::new(seq_id, count as u16);

        // Send response: header + entries
        let mut resp_buf = [0u8; 12 + 16 * 32]; // header + 16 entries max
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

    /// Queue a spawn command for later delivery when driver reports Ready
    fn queue_pending_spawn(&mut self, driver_idx: u8, binary: &'static str, port_name: &[u8]) {
        // Find an empty slot
        for spawn in &mut self.pending_spawns {
            if spawn.driver_idx == 0xFF {
                spawn.driver_idx = driver_idx;
                spawn.binary = binary;
                let len = port_name.len().min(32);
                spawn.port_name[..len].copy_from_slice(&port_name[..len]);
                spawn.port_name_len = len as u8;
                dlog!("queued spawn {} for driver {} (port trigger)", binary, driver_idx);
                return;
            }
        }
        derror!("pending spawn queue full for driver {}", driver_idx);
    }

    /// Send all pending spawn commands for a driver
    fn flush_pending_spawns(&mut self, driver_idx: u8) {
        // Collect pending spawns for this driver
        let mut to_send: [(Option<&'static str>, [u8; 32], u8); MAX_PENDING_SPAWNS_PER_DRIVER] =
            [(None, [0u8; 32], 0); MAX_PENDING_SPAWNS_PER_DRIVER];
        let mut count = 0;

        for spawn in &mut self.pending_spawns {
            if spawn.driver_idx == driver_idx {
                if count < MAX_PENDING_SPAWNS_PER_DRIVER {
                    to_send[count] = (Some(spawn.binary), spawn.port_name, spawn.port_name_len);
                    count += 1;
                }
                // Clear the slot
                *spawn = PendingSpawn::empty();
            }
        }

        if count == 0 {
            return;
        }

        dlog!("flushing {} pending spawns for driver {}", count, driver_idx);

        // Send the commands
        for i in 0..count {
            if let (Some(binary), port_name, port_len) = to_send[i] {
                let port = &port_name[..port_len as usize];
                match self.query_handler.send_spawn_child(driver_idx, binary.as_bytes(), port) {
                    Some(seq_id) => {
                        dlog!("sent queued SPAWN_CHILD seq={} binary={}", seq_id, binary);
                        // Track inflight spawn so we can store context when ACK arrives
                        let port_type = self.ports.get_port_type(port).unwrap_or(0);
                        self.track_inflight_spawn(seq_id, port_type, port, binary);
                    }
                    None => {
                        // Driver still not connected - fall back to direct spawn
                        dlog!("driver {} still not connected, spawning {} directly", driver_idx, binary);
                        self.spawn_dynamic_driver(binary, port);
                    }
                }
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
                derror!("invalid SPAWN_ACK message from slot={}", slot);
                return;
            }
        };

        // Get the parent driver's service index
        let parent_idx = match self.query_handler.get_service_idx(slot) {
            Some(idx) => idx,
            None => {
                dlog!("SPAWN_ACK from non-driver slot={}", slot);
                return;
            }
        };

        dlog!("SPAWN_ACK from driver {}: seq={} result={} match={} spawn={}",
            parent_idx, seq_id, result, match_count, spawn_count);

        // Consume the inflight spawn to get port context and binary name
        let spawn_ctx = self.consume_inflight_spawn(seq_id);

        if spawn_count == 0 || result < 0 {
            return;
        }

        // Extract binary name from spawn context
        let binary_name: &[u8] = spawn_ctx.as_ref()
            .map(|(_, _, _, bin, bin_len)| &bin[..*bin_len as usize])
            .unwrap_or(b"???");

        // Create service slots for each spawned child
        let now = Self::now_ms();
        for i in 0..spawn_count as usize {
            let child_pid = pids[i];
            if child_pid == 0 {
                continue;
            }

            // Create a new service slot for this child with its binary name
            let slot_idx = match self.services.create_dynamic_service(child_pid, binary_name, now) {
                Some(idx) => idx,
                None => {
                    derror!("no service slot for child pid={}", child_pid);
                    continue;
                }
            };

            // Add to recent_dynamic_pids (for race condition handling)
            self.add_recent_dynamic_pid(child_pid, slot_idx as u8);

            // Store spawn context (port that triggered spawn) for GET_SPAWN_CONTEXT
            if let Some((port_type, port_name, port_name_len, _, _)) = spawn_ctx {
                self.store_spawn_context(child_pid, port_type, &port_name[..port_name_len as usize]);
                dlog!("stored spawn context for pid={}: port={}", child_pid,
                    core::str::from_utf8(&port_name[..port_name_len as usize]).unwrap_or("?"));
            }

            // Upgrade any already-connected client with this PID to driver status
            // This handles the race where child connects before SPAWN_ACK arrives
            self.query_handler.upgrade_client_to_driver(child_pid, slot_idx as u8);

            dlog!("child pid={} registered in slot {} (parent={})", child_pid, slot_idx, parent_idx);
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
            dlog!("query: client disconnected slot={}", slot);
        }
    }

    // =========================================================================
    // Dynamic Port Registration
    // =========================================================================

    /// Handle a port registration request from a driver
    ///
    /// Called when a driver sends REGISTER_PORT to devd-query:.
    /// Registers the port with hierarchy info and checks rules for auto-spawning.
    pub fn handle_port_registration(
        &mut self,
        port_name: &[u8],
        port_type: PortType,
        parent_name: Option<&[u8]>,
        owner_idx: u8,
        shmem_id: u32,
    ) -> Result<(), SysError> {
        // Register the port with DataPort info
        self.ports.register_with_dataport(port_name, owner_idx, port_type, shmem_id, parent_name)?;

        // Get parent type for rule matching
        let parent_type = parent_name.and_then(|p| self.ports.port_type(p));

        // Log registration
        if let Ok(name_str) = core::str::from_utf8(port_name) {
            if shmem_id != 0 {
                dlog!("port registered: {} type={:?} shmem_id={} parent={:?}",
                    name_str, port_type, shmem_id, parent_type);
            } else {
                dlog!("port registered: {} type={:?} parent={:?}",
                    name_str, port_type, parent_type);
            }
        }

        // Check rules for auto-spawning
        self.check_rules_for_port(port_name, port_type, parent_type, owner_idx);

        Ok(())
    }

    /// Check if any rules match a newly registered port
    fn check_rules_for_port(
        &mut self,
        port_name: &[u8],
        port_type: PortType,
        parent_type: Option<PortType>,
        owner_idx: u8,
    ) {
        let rule = match self.rules.find_matching_rule(port_type, parent_type) {
            Some(r) => r,
            None => return,
        };

        dlog!("rule matched: {} should spawn {} for port", owner_idx, rule.driver_binary);

        // Queue the spawn command - it will be sent when the driver reports Ready
        // This avoids the race condition where SPAWN_CHILD arrives while the driver
        // is still in register_port() waiting for the REGISTER_PORT response
        self.queue_pending_spawn(owner_idx, rule.driver_binary, port_name);
    }

    /// Spawn a driver dynamically (not from SERVICE_DEFS)
    fn spawn_dynamic_driver(&mut self, binary: &str, trigger_port: &[u8]) {
        let now = Self::now_ms();

        // Find empty service slot BEFORE spawning so we can track immediately
        let slot_idx = self.services.find_empty_slot();
        let idx = slot_idx.unwrap_or(0xFF); // 0xFF = invalid/no slot

        // Spawn the process
        let (pid, watcher) = match self.process_mgr.spawn(binary) {
            Ok(result) => result,
            Err(e) => {
                derror!("dynamic spawn {} failed err={:?}", binary, e);
                return;
            }
        };

        // IMMEDIATELY add to recent_dynamic_pids - this MUST happen before anything
        // else because the spawned child may already be running and connecting!
        // The child process starts immediately when spawn returns.
        if idx != 0xFF {
            self.add_recent_dynamic_pid(pid, idx as u8);
        }

        // Store spawn context for GET_SPAWN_CONTEXT
        if !trigger_port.is_empty() {
            let port_type = self.ports.get_port_type(trigger_port).unwrap_or(0);
            self.store_spawn_context(pid, port_type, trigger_port);
        }

        // Now set up the service slot
        if let Some(idx) = slot_idx {
            if let Some(service) = self.services.get_mut(idx) {
                service.pid = pid;
                service.watcher = Some(watcher);
                service.state = service::ServiceState::Ready;
                service.last_change = now;
            }

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
            derror!("no service slot for dynamic driver pid={}", pid);
        }

        if let Ok(port_str) = core::str::from_utf8(trigger_port) {
            dlog!("spawned {} pid={} slot={:?} for trigger={}", binary, pid, slot_idx, port_str);
        } else {
            dlog!("spawned {} pid={} slot={:?}", binary, pid, slot_idx);
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

    /// Track an in-flight spawn command (seq_id -> port info + binary name)
    fn track_inflight_spawn(&mut self, seq_id: u32, port_type: u8, port_name: &[u8], binary: &str) {
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
                return;
            }
        }
        // No slot available - oldest will be overwritten implicitly when it's used
    }

    /// Consume an in-flight spawn by seq_id, returning (port_type, port_name, port_len, binary_name, binary_len)
    fn consume_inflight_spawn(&mut self, seq_id: u32) -> Option<(u8, [u8; 32], u8, [u8; 16], u8)> {
        for entry in &mut self.inflight_spawns {
            if entry.seq_id == seq_id {
                let result = (entry.port_type, entry.port_name, entry.port_name_len,
                              entry.binary_name, entry.binary_name_len);
                *entry = InflightSpawn::empty();
                return Some(result);
            }
        }
        None
    }

    /// Store spawn context for a child PID
    fn store_spawn_context(&mut self, pid: u32, port_type: u8, port_name: &[u8]) {
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
    }

    /// Get spawn context for a PID
    fn get_spawn_context(&self, pid: u32) -> Option<(u8, &[u8])> {
        for entry in &self.spawn_contexts {
            if entry.pid == pid {
                return Some((entry.port_type, &entry.port_name[..entry.port_name_len as usize]));
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
        // Start services that have no dependencies
        self.check_pending_services();

        loop {
            let events = self.events.as_ref().expect("devd: events not initialized");
            let wait_result = events.wait();

            match wait_result {
                Ok(handle) => {
                    // Service port event?
                    if let Some(port) = &self.port {
                        if handle == port.handle() {
                            self.handle_port_event();
                            continue;
                        }
                    }

                    // Query port event?
                    if let Some(query_port) = &self.query_port {
                        if handle == query_port.handle() {
                            self.handle_query_port_event();
                            continue;
                        }
                    }

                    // Query client message?
                    if self.query_handler.find_by_handle(handle).is_some() {
                        self.handle_query_client_event(handle);
                        continue;
                    }

                    // Admin client (shell) message?
                    if self.admin_clients.find_by_handle(handle).is_some() {
                        self.handle_admin_client_event(handle);
                        continue;
                    }

                    // Service restart timer?
                    if let Some(idx) = self.services.find_by_timer(handle) {
                        self.handle_restart_timer(idx);
                        continue;
                    }

                    // Process exit (watcher)?
                    self.handle_process_exit(handle);
                }
                Err(e) => {
                    derror!("wait failed err={:?}", e);
                }
            }
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

    syscall::klog(LogLevel::Info, b"[devd] main() entry");

    let devd = unsafe { &mut DEVD };

    syscall::klog(LogLevel::Info, b"[devd] calling init()");
    if let Err(e) = devd.init() {
        derror!("devd: init failed err={:?}", e);
        syscall::exit(1);
    }
    syscall::klog(LogLevel::Info, b"[devd] init() complete");

    dlog!("devd: started services={}", devd.services.count());

    syscall::klog(LogLevel::Info, b"[devd] entering run()");
    devd.run()
}
