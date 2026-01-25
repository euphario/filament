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
//! ## Service Tree Example
//! ```text
//! devd (PID 1, registers "devd:")
//!  ├─ consoled (requires: devd:, registers: console:)
//!  │   └─ shell (requires: console:)
//!  ├─ pcied (requires: devd:, registers: pcie:)
//!  │   └─ wifid (requires: pcie:, depends_on: console:)
//!  └─ usbd (requires: devd:, registers: usb:)
//!      └─ fatfs (requires: usb:, registers: fat:)
//! ```

#![no_std]
#![no_main]

use userlib::syscall::{self, LogLevel};
use userlib::ipc::{Port, Timer, Process, Channel, ObjHandle, EventLoop};
use userlib::error::{SysError, SysResult};

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
// Constants
// =============================================================================

const MAX_SERVICES: usize = 16;
const MAX_PORTS_PER_SERVICE: usize = 4;
const MAX_CHILDREN_PER_SERVICE: usize = 8;
const MAX_PORT_NAME: usize = 32;
const MAX_RESTARTS: u8 = 3;
const INITIAL_BACKOFF_MS: u32 = 1000;
const MAX_BACKOFF_MS: u32 = 30000;
/// Time before Failed services are retried (5 minutes)
const FAILED_RETRY_MS: u64 = 5 * 60 * 1000;
/// Time Ready before restart count resets (1 minute)
const STABLE_READY_MS: u64 = 60 * 1000;

// =============================================================================
// Service State Machine
// =============================================================================

/// Service lifecycle state
///
/// State transitions:
///   Pending ──[deps satisfied]──► Starting
///   Starting ──[grace period elapsed]──► Ready
///   Ready ──[exit(0)]──► Stopped
///   Ready ──[exit(!=0)]──► Crashed
///   Crashed ──[backoff elapsed, restarts < MAX]──► Starting
///   Crashed ──[restarts >= MAX]──► Failed
///   Failed ──[FAILED_RETRY_MS elapsed, auto_restart]──► Pending (recovery)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServiceState {
    /// Waiting for dependencies to be satisfied
    Pending,
    /// Spawned, waiting for it to register/connect
    Starting { spawn_time: u64 },
    /// Service is ready (connected, ports registered)
    Ready,
    /// Service exited cleanly
    Stopped { code: i32 },
    /// Service crashed, pending restart
    Crashed { code: i32, restarts: u8 },
    /// Service failed permanently (max restarts)
    Failed { code: i32 },
}

impl ServiceState {
    pub fn is_running(&self) -> bool {
        matches!(self, ServiceState::Starting { .. } | ServiceState::Ready)
    }

    pub fn is_terminal(&self) -> bool {
        matches!(self, ServiceState::Stopped { .. } | ServiceState::Failed { .. })
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            ServiceState::Pending => "pending",
            ServiceState::Starting { .. } => "starting",
            ServiceState::Ready => "ready",
            ServiceState::Stopped { .. } => "stopped",
            ServiceState::Crashed { .. } => "crashed",
            ServiceState::Failed { .. } => "failed",
        }
    }
}

// =============================================================================
// Dependency Types
// =============================================================================

/// Dependency specification
#[derive(Clone, Copy, Debug)]
pub enum Dependency {
    /// Hard dependency - service won't start until this port exists
    Requires(&'static [u8]),
    /// Soft dependency - service starts anyway, handles missing gracefully
    DependsOn(&'static [u8]),
}

impl Dependency {
    pub fn port_name(&self) -> &'static [u8] {
        match self {
            Dependency::Requires(name) | Dependency::DependsOn(name) => name,
        }
    }

    pub fn is_required(&self) -> bool {
        matches!(self, Dependency::Requires(_))
    }
}

// =============================================================================
// Service Definition (static config)
// =============================================================================

/// Static service definition (compiled-in)
pub struct ServiceDef {
    /// Binary name in initrd
    pub binary: &'static str,
    /// Ports this service will register
    pub registers: &'static [&'static [u8]],
    /// Dependencies
    pub dependencies: &'static [Dependency],
    /// Auto-restart on crash?
    pub auto_restart: bool,
    /// Parent service (None = direct child of devd)
    pub parent: Option<&'static str>,
}

/// Service definitions - the "service tree"
static SERVICE_DEFS: &[ServiceDef] = &[
    ServiceDef {
        binary: "consoled",
        registers: &[b"console:"],
        dependencies: &[],  // devd: is implicit
        auto_restart: true,
        parent: None,
    },
    ServiceDef {
        binary: "shell",
        registers: &[],  // shell doesn't register any ports
        dependencies: &[Dependency::Requires(b"console:")],  // Wait for consoled
        auto_restart: true,
        parent: None,
    },
    // Future services would be added here:
    // ServiceDef {
    //     binary: "pcied",
    //     registers: &[b"pcie:"],
    //     dependencies: &[],
    //     auto_restart: true,
    //     parent: None,
    // },
    // ServiceDef {
    //     binary: "wifid",
    //     registers: &[b"wifi:"],
    //     dependencies: &[Dependency::Requires(b"pcie:")],
    //     auto_restart: true,
    //     parent: Some("pcied"),
    // },
];

// =============================================================================
// Port Registration
// =============================================================================

/// A registered port
#[derive(Clone, Copy)]
pub struct RegisteredPort {
    /// Port name (e.g., "console:")
    name: [u8; MAX_PORT_NAME],
    name_len: u8,
    /// Service index that owns this port
    owner: u8,
    /// Is the port currently available?
    available: bool,
}

impl RegisteredPort {
    pub const fn empty() -> Self {
        Self {
            name: [0; MAX_PORT_NAME],
            name_len: 0,
            owner: 0,
            available: false,
        }
    }

    pub fn name(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }

    pub fn matches(&self, name: &[u8]) -> bool {
        self.name_len as usize == name.len() &&
            &self.name[..self.name_len as usize] == name
    }
}

// =============================================================================
// Service Instance (runtime state)
// =============================================================================

/// A running or tracked service instance
pub struct Service {
    /// Index into SERVICE_DEFS
    def_index: u8,
    /// Current state
    state: ServiceState,
    /// Process ID (0 if not running)
    pid: u32,
    /// Process watcher handle
    watcher: Option<Process>,
    /// Channel to this service (for IPC)
    _channel: Option<Channel>,  // For future IPC with service
    /// Indices of child services
    children: [Option<u8>; MAX_CHILDREN_PER_SERVICE],
    child_count: u8,
    /// Parent service index (None = devd)
    parent: Option<u8>,
    /// Restart backoff in ms
    backoff_ms: u32,
    /// Last state change timestamp
    last_change: u64,
    /// Total restarts (for monitoring, not reset)
    total_restarts: u32,
}

impl Service {
    pub const fn empty() -> Self {
        Self {
            def_index: 0,
            state: ServiceState::Pending,
            pid: 0,
            watcher: None,
            _channel: None,
            children: [None; MAX_CHILDREN_PER_SERVICE],
            child_count: 0,
            parent: None,
            backoff_ms: INITIAL_BACKOFF_MS,
            last_change: 0,
            total_restarts: 0,
        }
    }

    pub fn def(&self) -> &'static ServiceDef {
        &SERVICE_DEFS[self.def_index as usize]
    }

    pub fn name(&self) -> &'static str {
        self.def().binary
    }

    /// Add a child service index
    pub fn add_child(&mut self, child_idx: u8) -> bool {
        if self.child_count as usize >= MAX_CHILDREN_PER_SERVICE {
            return false;
        }
        self.children[self.child_count as usize] = Some(child_idx);
        self.child_count += 1;
        true
    }

    /// Remove a child service index
    pub fn remove_child(&mut self, child_idx: u8) {
        for i in 0..self.child_count as usize {
            if self.children[i] == Some(child_idx) {
                // Shift remaining
                for j in i..self.child_count as usize - 1 {
                    self.children[j] = self.children[j + 1];
                }
                self.children[self.child_count as usize - 1] = None;
                self.child_count -= 1;
                return;
            }
        }
    }

    /// Transition to a new state
    pub fn transition(&mut self, new_state: ServiceState, now: u64) {
        let old = self.state;
        self.state = new_state;
        self.last_change = now;
        dlog!("service {} state: {:?} -> {:?}", self.name(), old, new_state);
    }
}

// =============================================================================
// Service Registry (devd core)
// =============================================================================

pub struct Devd {
    /// Our port for clients to connect
    port: Option<Port>,
    /// Event loop
    events: Option<EventLoop>,
    /// Maintenance timer
    timer: Option<Timer>,
    /// All tracked services
    services: [Option<Service>; MAX_SERVICES],
    service_count: usize,
    /// All registered ports (for routing)
    ports: [Option<RegisteredPort>; MAX_SERVICES * MAX_PORTS_PER_SERVICE],
    port_count: usize,
    /// Current time in ms
    now_ms: u64,
}

impl Devd {
    pub const fn new() -> Self {
        Self {
            port: None,
            events: None,
            timer: None,
            services: [const { None }; MAX_SERVICES],
            service_count: 0,
            ports: [const { None }; MAX_SERVICES * MAX_PORTS_PER_SERVICE],
            port_count: 0,
            now_ms: 0,
        }
    }

    // =========================================================================
    // Initialization
    // =========================================================================

    pub fn init(&mut self) -> SysResult<()> {
        // Create event loop
        let mut events = EventLoop::new()?;

        // Register our port
        let port = Port::register(b"devd:")?;
        events.watch(port.handle())?;

        // Create maintenance timer (1 second interval)
        let mut timer = Timer::new()?;
        timer.set(1_000_000_000)?;
        events.watch(timer.handle())?;

        self.port = Some(port);
        self.timer = Some(timer);
        self.events = Some(events);

        // Register devd's own port in the registry
        self.register_port(b"devd:", 0xFF)?; // 0xFF = devd itself

        // Initialize services from definitions
        self.init_services();

        Ok(())
    }

    fn init_services(&mut self) {
        // Create service entries for all defined services
        for (i, def) in SERVICE_DEFS.iter().enumerate() {
            if self.service_count >= MAX_SERVICES {
                derror!("too many services limit={}", MAX_SERVICES);
                break;
            }

            let mut service = Service::empty();
            service.def_index = i as u8;
            service.state = ServiceState::Pending;

            // Find parent index
            if let Some(parent_name) = def.parent {
                for (j, s) in self.services.iter().enumerate() {
                    if let Some(s) = s {
                        if s.name() == parent_name {
                            service.parent = Some(j as u8);
                            break;
                        }
                    }
                }
            }

            self.services[self.service_count] = Some(service);
            self.service_count += 1;
        }

        // Link children to parents
        for i in 0..self.service_count {
            if let Some(service) = &self.services[i] {
                if let Some(parent_idx) = service.parent {
                    if let Some(parent) = &mut self.services[parent_idx as usize] {
                        parent.add_child(i as u8);
                    }
                }
            }
        }
    }

    // =========================================================================
    // Port Registry
    // =========================================================================

    fn register_port(&mut self, name: &[u8], owner: u8) -> SysResult<()> {
        if name.len() > MAX_PORT_NAME {
            return Err(SysError::InvalidArgument);
        }

        // Check for duplicate
        for p in self.ports.iter().flatten() {
            if p.matches(name) {
                derror!("port {:?} already registered", core::str::from_utf8(name));
                return Err(SysError::AlreadyExists);
            }
        }

        // Find empty slot
        for slot in &mut self.ports {
            if slot.is_none() {
                let mut port = RegisteredPort::empty();
                port.name[..name.len()].copy_from_slice(name);
                port.name_len = name.len() as u8;
                port.owner = owner;
                port.available = true;
                *slot = Some(port);
                self.port_count += 1;
                dlog!("port {:?} registered owner={}", core::str::from_utf8(name), owner);
                return Ok(());
            }
        }

        Err(SysError::NoSpace)
    }

    #[allow(dead_code)]  // Infrastructure for future use
    fn unregister_port(&mut self, name: &[u8]) {
        for slot in &mut self.ports {
            if let Some(p) = slot {
                if p.matches(name) {
                    dlog!("port {:?} unregistered", core::str::from_utf8(name));
                    *slot = None;
                    self.port_count -= 1;
                    return;
                }
            }
        }
    }

    fn port_available(&self, name: &[u8]) -> bool {
        self.ports.iter()
            .flatten()
            .any(|p| p.matches(name) && p.available)
    }

    fn set_port_availability(&mut self, name: &[u8], available: bool) {
        for slot in &mut self.ports {
            if let Some(p) = slot {
                if p.matches(name) {
                    p.available = available;
                    return;
                }
            }
        }
    }

    // =========================================================================
    // Dependency Resolution
    // =========================================================================

    fn deps_satisfied(&self, service: &Service) -> bool {
        for dep in service.def().dependencies {
            if dep.is_required() && !self.port_available(dep.port_name()) {
                return false;
            }
        }
        true
    }

    fn check_pending_services(&mut self) {
        let now = self.now_ms;
        for i in 0..self.service_count {
            if let Some(service) = &self.services[i] {
                if service.state == ServiceState::Pending && self.deps_satisfied(service) {
                    // Borrow ends here, spawn needs &mut self
                    self.spawn_service(i, now);
                }
            }
        }
    }

    // =========================================================================
    // Service Spawning
    // =========================================================================

    fn spawn_service(&mut self, idx: usize, now: u64) {
        // Bounds check
        if idx >= MAX_SERVICES {
            return;
        }

        // Get binary name and spawn
        let binary = {
            let service = match &self.services[idx] {
                Some(s) => s,
                None => return,
            };
            service.def().binary
        };

        // Spawn the process
        let pid = syscall::exec(binary);
        if pid < 0 {
            derror!("spawn {} failed err={}", binary, pid);
            if let Some(s) = &mut self.services[idx] {
                s.transition(ServiceState::Failed { code: pid as i32 }, now);
            }
            return;
        }
        let pid = pid as u32;

        // Watch the process
        let watcher = match Process::watch(pid) {
            Ok(w) => w,
            Err(e) => {
                derror!("watch {} failed err={:?}", binary, e);
                if let Some(s) = &mut self.services[idx] {
                    s.transition(ServiceState::Failed { code: -1 }, now);
                }
                return;
            }
        };

        // Add to event loop
        if let Some(events) = &mut self.events {
            let _ = events.watch(watcher.handle());
        }

        // Collect port names before taking mutable borrow
        let port_names: [Option<&'static [u8]>; MAX_PORTS_PER_SERVICE] = {
            let service = match &self.services[idx] {
                Some(s) => s,
                None => return,
            };
            let mut arr = [None; MAX_PORTS_PER_SERVICE];
            for (i, &pn) in service.def().registers.iter().enumerate() {
                if i < MAX_PORTS_PER_SERVICE {
                    arr[i] = Some(pn);
                }
            }
            arr
        };

        // Update service state
        if let Some(service) = &mut self.services[idx] {
            service.pid = pid;
            service.watcher = Some(watcher);
            service.transition(ServiceState::Starting { spawn_time: now }, now);
        }

        dlog!("spawned {} pid={}", binary, pid);

        // Pre-register ports this service will provide
        for port_name in port_names.iter().flatten() {
            let _ = self.register_port(port_name, idx as u8);
            self.set_port_availability(port_name, false);
        }
    }

    // =========================================================================
    // Service State Changes
    // =========================================================================

    fn handle_service_exit(&mut self, idx: usize, code: i32) {
        // Bounds check
        if idx >= MAX_SERVICES {
            return;
        }

        let now = self.now_ms;

        // First pass: collect info and update service state
        let (_name, port_names, should_prune, _auto_restart) = {
            let service = match &mut self.services[idx] {
                Some(s) => s,
                None => return,
            };

            let name = service.name();
            let auto_restart = service.def().auto_restart;

            // Remove from event loop
            if let Some(watcher) = service.watcher.take() {
                if let Some(events) = &mut self.events {
                    let _ = events.unwatch(watcher.handle());
                }
            }

            // Collect port names to mark unavailable
            let port_names: [Option<&'static [u8]>; MAX_PORTS_PER_SERVICE] = {
                let mut arr = [None; MAX_PORTS_PER_SERVICE];
                for (i, &pn) in service.def().registers.iter().enumerate() {
                    if i < MAX_PORTS_PER_SERVICE {
                        arr[i] = Some(pn);
                    }
                }
                arr
            };

            if code == 0 {
                service.transition(ServiceState::Stopped { code: 0 }, now);
                (name, port_names, false, auto_restart)
            } else {
                // Track total restarts for monitoring
                service.total_restarts = service.total_restarts.saturating_add(1);

                let restarts = match service.state {
                    ServiceState::Crashed { restarts, .. } => restarts + 1,
                    _ => 1,
                };

                if restarts >= MAX_RESTARTS || !auto_restart {
                    service.transition(ServiceState::Failed { code }, now);
                    derror!("{} failed code={} restarts={}", name, code, service.total_restarts);
                    (name, port_names, false, auto_restart)
                } else {
                    service.transition(ServiceState::Crashed { code, restarts }, now);
                    service.backoff_ms = (service.backoff_ms * 2).min(MAX_BACKOFF_MS);
                    (name, port_names, true, auto_restart)
                }
            }
        };

        // Second pass: mark ports unavailable (service borrow is dropped)
        for port_name in port_names.iter().flatten() {
            self.set_port_availability(port_name, false);
        }

        // Third pass: prune children if needed
        if should_prune {
            self.prune_children(idx);
        }
    }

    fn handle_service_ready(&mut self, idx: usize) {
        // Bounds check
        if idx >= MAX_SERVICES {
            return;
        }

        let now = self.now_ms;

        // First pass: update service state and collect port names
        let port_names: [Option<&'static [u8]>; MAX_PORTS_PER_SERVICE] = {
            let service = match &mut self.services[idx] {
                Some(s) => s,
                None => return,
            };

            let mut arr = [None; MAX_PORTS_PER_SERVICE];
            for (i, &pn) in service.def().registers.iter().enumerate() {
                if i < MAX_PORTS_PER_SERVICE {
                    arr[i] = Some(pn);
                }
            }

            service.transition(ServiceState::Ready, now);
            service.backoff_ms = INITIAL_BACKOFF_MS;
            arr
        };

        // Second pass: mark ports available
        for port_name in port_names.iter().flatten() {
            self.set_port_availability(port_name, true);
        }

        // Third pass: check pending services
        self.check_pending_services();
    }

    // =========================================================================
    // Branch Pruning (kill children on parent crash)
    // =========================================================================

    fn prune_children(&mut self, parent_idx: usize) {
        // Bounds check
        if parent_idx >= MAX_SERVICES {
            return;
        }

        // Collect children to kill (can't borrow mutably while iterating)
        let mut to_kill = [None; MAX_CHILDREN_PER_SERVICE];
        let mut kill_count = 0;

        if let Some(parent) = &self.services[parent_idx] {
            for i in 0..parent.child_count as usize {
                if let Some(child_idx) = parent.children[i] {
                    to_kill[kill_count] = Some(child_idx);
                    kill_count += 1;
                }
            }
        }

        // Kill children recursively (depth-first, leaves first)
        for i in 0..kill_count {
            if let Some(child_idx) = to_kill[i] {
                // Recurse first (kill grandchildren)
                self.prune_children(child_idx as usize);

                // Then kill this child
                self.kill_service(child_idx as usize);
            }
        }
    }

    fn kill_service(&mut self, idx: usize) {
        // Bounds check
        if idx >= MAX_SERVICES {
            return;
        }

        let now = self.now_ms;

        // First pass: check state, collect info, kill process, cleanup watcher
        let port_names: [Option<&'static [u8]>; MAX_PORTS_PER_SERVICE] = {
            let service = match &mut self.services[idx] {
                Some(s) => s,
                None => return,
            };

            if !service.state.is_running() {
                return;
            }

            let name = service.name();
            let pid = service.pid;

            dlog!("killing {} pid={} (branch prune)", name, pid);

            // Kill the process
            if pid != 0 {
                let _ = syscall::kill(pid);
            }

            // Clean up watcher
            if let Some(watcher) = service.watcher.take() {
                if let Some(events) = &mut self.events {
                    let _ = events.unwatch(watcher.handle());
                }
            }

            // Collect port names
            let mut arr = [None; MAX_PORTS_PER_SERVICE];
            for (i, &pn) in service.def().registers.iter().enumerate() {
                if i < MAX_PORTS_PER_SERVICE {
                    arr[i] = Some(pn);
                }
            }

            // Set to pending
            service.pid = 0;
            service.transition(ServiceState::Pending, now);
            arr
        };

        // Second pass: mark ports unavailable
        for port_name in port_names.iter().flatten() {
            self.set_port_availability(port_name, false);
        }
    }

    // =========================================================================
    // Ready Detection
    // =========================================================================

    /// Time (ms) after spawn to assume service is ready
    const STARTUP_GRACE_MS: u64 = 100;

    fn check_starting_services(&mut self) {
        let now = self.now_ms;

        // Collect indices of services that should be promoted to Ready
        let mut to_ready = [None; MAX_SERVICES];
        let mut ready_count = 0;

        for i in 0..self.service_count {
            if let Some(service) = &self.services[i] {
                if let ServiceState::Starting { spawn_time } = service.state {
                    let elapsed = now.saturating_sub(spawn_time);
                    if elapsed >= Self::STARTUP_GRACE_MS {
                        to_ready[ready_count] = Some(i);
                        ready_count += 1;
                    }
                }
            }
        }

        // Promote to Ready (borrows are released)
        for idx in to_ready.iter().flatten() {
            self.handle_service_ready(*idx);
        }
    }

    // =========================================================================
    // Restart Logic
    // =========================================================================

    fn check_restarts(&mut self) {
        let now = self.now_ms;

        // Collect indices of services that need restart
        let mut to_restart = [None; MAX_SERVICES];
        let mut restart_count = 0;

        for i in 0..self.service_count {
            if let Some(service) = &self.services[i] {
                if let ServiceState::Crashed { .. } = service.state {
                    let elapsed = now.saturating_sub(service.last_change);
                    if elapsed >= service.backoff_ms as u64 {
                        to_restart[restart_count] = Some(i);
                        restart_count += 1;
                    }
                }
            }
        }

        // Now spawn (borrows are released)
        for idx in to_restart.iter().flatten() {
            self.spawn_service(*idx, now);
        }
    }

    /// Check Failed services for retry after FAILED_RETRY_MS
    fn check_failed_services(&mut self) {
        let now = self.now_ms;

        // Collect indices of services that should be retried
        let mut to_retry = [None; MAX_SERVICES];
        let mut retry_count = 0;

        for i in 0..self.service_count {
            if let Some(service) = &self.services[i] {
                if let ServiceState::Failed { .. } = service.state {
                    if service.def().auto_restart {
                        let elapsed = now.saturating_sub(service.last_change);
                        if elapsed >= FAILED_RETRY_MS {
                            to_retry[retry_count] = Some(i);
                            retry_count += 1;
                        }
                    }
                }
            }
        }

        // Transition to Pending and reset backoff
        for idx in to_retry.iter().flatten() {
            if let Some(service) = &mut self.services[*idx] {
                let name = service.name();
                service.backoff_ms = INITIAL_BACKOFF_MS;
                service.transition(ServiceState::Pending, now);
                dlog!("{} retry after failed (total_restarts={})", name, service.total_restarts);
            }
        }
    }

    /// Reset restart count for services that have been Ready for STABLE_READY_MS
    fn check_stable_services(&mut self) {
        let now = self.now_ms;

        for i in 0..self.service_count {
            if let Some(service) = &mut self.services[i] {
                if service.state == ServiceState::Ready {
                    let elapsed = now.saturating_sub(service.last_change);
                    if elapsed >= STABLE_READY_MS && service.backoff_ms != INITIAL_BACKOFF_MS {
                        // Service has been stable - reset backoff
                        service.backoff_ms = INITIAL_BACKOFF_MS;
                    }
                }
            }
        }
    }

    // =========================================================================
    // Event Handling
    // =========================================================================

    fn handle_timer(&mut self) {
        // Update time
        self.now_ms = syscall::gettime() / 1_000_000; // ns to ms

        // Re-arm timer
        if let Some(timer) = &mut self.timer {
            let _ = timer.wait();
            let _ = timer.set(1_000_000_000);
        }

        // Check for Starting services that should be Ready
        self.check_starting_services();

        // Check for services needing restart (Crashed -> Starting)
        self.check_restarts();

        // Check for Failed services that should be retried (Failed -> Pending)
        self.check_failed_services();

        // Reset backoff for stable Ready services
        self.check_stable_services();

        // Check for pending services (Pending -> Starting)
        self.check_pending_services();
    }

    fn handle_port_event(&mut self) {
        let port = match &mut self.port {
            Some(p) => p,
            None => return,
        };

        match port.accept() {
            Ok(channel) => {
                // TODO: Handle client messages (service queries, etc.)
                // For now, check if it's a service announcing ready
                drop(channel);
            }
            Err(SysError::WouldBlock) => {
                // Spurious wake, ignore
            }
            Err(e) => {
                derror!("devd: accept failed err={:?}", e);
            }
        }
    }

    fn handle_process_exit(&mut self, handle: ObjHandle) {
        // Find which service this watcher belongs to
        let mut found_idx = None;
        for i in 0..self.service_count {
            if let Some(service) = &self.services[i] {
                if let Some(watcher) = &service.watcher {
                    if watcher.handle() == handle {
                        found_idx = Some(i);
                        break;
                    }
                }
            }
        }

        // Get exit code and handle exit (separate borrow)
        if let Some(idx) = found_idx {
            let code = self.services[idx]
                .as_mut()
                .and_then(|s| s.watcher.as_mut())
                .map(|w| w.wait().unwrap_or(-1))
                .unwrap_or(-1);
            self.handle_service_exit(idx, code);
        }
    }

    // =========================================================================
    // Main Loop
    // =========================================================================

    pub fn run(&mut self) -> ! {
        // Initial time
        self.now_ms = syscall::gettime() / 1_000_000;

        // Start services that have no dependencies
        self.check_pending_services();

        loop {
            let events = self.events.as_ref().expect("devd: events not initialized");
            let wait_result = events.wait();

            match wait_result {
                Ok(handle) => {
                    // Timer?
                    if let Some(timer) = &self.timer {
                        if handle == timer.handle() {
                            self.handle_timer();
                            continue;
                        }
                    }

                    // Port?
                    if let Some(port) = &self.port {
                        if handle == port.handle() {
                            self.handle_port_event();
                            continue;
                        }
                    }

                    // Must be a process exit
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
#[allow(static_mut_refs)]  // Required for no_std global state
fn main() -> ! {
    userlib::io::disable_stdout();

    let devd = unsafe { &mut DEVD };

    // Initialize
    if let Err(e) = devd.init() {
        derror!("devd: init failed err={:?}", e);
        syscall::exit(1);
    }

    dlog!("devd: started services={}", devd.service_count);

    // Run main loop
    devd.run()
}
