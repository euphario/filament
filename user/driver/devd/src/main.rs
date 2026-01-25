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

// =============================================================================
// Service State Machine (Event-Driven)
// =============================================================================

/// Service lifecycle state
///
/// All transitions are event-driven, no polling:
///
///   Pending ──[deps satisfied]──► Starting
///       Event: Another service becomes Ready
///
///   Starting ──[service connects to devd:]──► Ready
///       Event: Port accept on devd: port
///
///   Ready ──[exit(0)]──► Stopped
///       Event: Process watcher fires
///
///   Ready ──[exit(!=0)]──► Crashed (sets restart timer)
///       Event: Process watcher fires
///
///   Crashed ──[restart timer fires, restarts < MAX]──► Starting
///       Event: Per-service Timer fires
///
///   Crashed ──[restarts >= MAX]──► Failed (sets recovery timer)
///       Immediate transition, no event needed
///
///   Failed ──[recovery timer fires]──► Pending
///       Event: Per-service Timer fires (5 minute delay)
///
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServiceState {
    /// Waiting for dependencies to be satisfied
    Pending,
    /// Spawned, waiting for service to connect to devd:
    Starting,
    /// Service is ready (connected to devd:)
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
    /// Channel from this service (for ready announcement)
    channel: Option<Channel>,
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
    /// Restart timer (set when Crashed or Failed, fires when ready to retry)
    restart_timer: Option<Timer>,
}

impl Service {
    pub const fn empty() -> Self {
        Self {
            def_index: 0,
            state: ServiceState::Pending,
            pid: 0,
            watcher: None,
            channel: None,
            children: [None; MAX_CHILDREN_PER_SERVICE],
            child_count: 0,
            parent: None,
            backoff_ms: INITIAL_BACKOFF_MS,
            last_change: 0,
            total_restarts: 0,
            restart_timer: None,
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
    /// Our port for services to announce ready
    port: Option<Port>,
    /// Event loop
    events: Option<EventLoop>,
    /// All tracked services
    services: [Option<Service>; MAX_SERVICES],
    service_count: usize,
    /// All registered ports (for routing)
    ports: [Option<RegisteredPort>; MAX_SERVICES * MAX_PORTS_PER_SERVICE],
    port_count: usize,
}

impl Devd {
    pub const fn new() -> Self {
        Self {
            port: None,
            events: None,
            services: [const { None }; MAX_SERVICES],
            service_count: 0,
            ports: [const { None }; MAX_SERVICES * MAX_PORTS_PER_SERVICE],
            port_count: 0,
        }
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    /// Get current time in milliseconds
    fn now_ms() -> u64 {
        syscall::gettime() / 1_000_000
    }

    // =========================================================================
    // Initialization
    // =========================================================================

    pub fn init(&mut self) -> SysResult<()> {
        // Create event loop
        let mut events = EventLoop::new()?;

        // Register our port (services connect here to announce ready)
        let port = Port::register(b"devd:")?;
        events.watch(port.handle())?;

        self.port = Some(port);
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
        let now = Self::now_ms();
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
            service.transition(ServiceState::Starting, now);
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

        let now = Self::now_ms();

        // Collect info about the exit
        #[derive(Clone, Copy)]
        enum ExitAction {
            Stopped,
            Crashed { backoff_ms: u32 },
            Failed,
        }

        // First pass: collect info and update service state
        let (port_names, should_prune, action) = {
            let service = match &mut self.services[idx] {
                Some(s) => s,
                None => return,
            };

            let name = service.name();
            let auto_restart = service.def().auto_restart;

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
                (port_names, false, ExitAction::Stopped)
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
                    (port_names, false, if auto_restart { ExitAction::Failed } else { ExitAction::Stopped })
                } else {
                    service.transition(ServiceState::Crashed { code, restarts }, now);
                    service.backoff_ms = (service.backoff_ms * 2).min(MAX_BACKOFF_MS);
                    (port_names, true, ExitAction::Crashed { backoff_ms: service.backoff_ms })
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

        // Fourth pass: create restart timer if needed
        match action {
            ExitAction::Crashed { backoff_ms } => {
                // Create timer for restart after backoff
                if let Ok(mut timer) = Timer::new() {
                    let deadline_ns = (backoff_ms as u64) * 1_000_000;
                    if timer.set(deadline_ns).is_ok() {
                        if let Some(events) = &mut self.events {
                            let _ = events.watch(timer.handle());
                        }
                        if let Some(service) = &mut self.services[idx] {
                            service.restart_timer = Some(timer);
                        }
                    }
                }
            }
            ExitAction::Failed => {
                // Create timer for recovery retry after FAILED_RETRY_MS
                if let Ok(mut timer) = Timer::new() {
                    let deadline_ns = FAILED_RETRY_MS * 1_000_000;
                    if timer.set(deadline_ns).is_ok() {
                        if let Some(events) = &mut self.events {
                            let _ = events.watch(timer.handle());
                        }
                        if let Some(service) = &mut self.services[idx] {
                            service.restart_timer = Some(timer);
                        }
                    }
                }
            }
            ExitAction::Stopped => {
                // Clean exit, no restart timer
            }
        }
    }

    fn handle_service_ready(&mut self, idx: usize) {
        // Bounds check
        if idx >= MAX_SERVICES {
            return;
        }

        let now = Self::now_ms();

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

        let now = Self::now_ms();

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
                let mut service_idx = None;
                for i in 0..self.service_count {
                    if let Some(service) = &self.services[i] {
                        if service.pid == client_pid && service.state == ServiceState::Starting {
                            service_idx = Some(i);
                            break;
                        }
                    }
                }

                if let Some(idx) = service_idx {
                    // Store channel and add to event loop for future messages
                    if let Some(events) = &mut self.events {
                        let _ = events.watch(channel.handle());
                    }
                    if let Some(service) = &mut self.services[idx] {
                        service.channel = Some(channel);
                    }
                    // Mark service as Ready
                    self.handle_service_ready(idx);
                } else {
                    // Unknown connection - just keep the channel for now
                    // Could be a client query, future enhancement
                    drop(channel);
                }
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

    /// Handle a service's restart timer firing
    fn handle_restart_timer(&mut self, idx: usize) {
        if idx >= MAX_SERVICES {
            return;
        }

        let now = Self::now_ms();

        // Collect info and clear timer
        let should_spawn = {
            let service = match &mut self.services[idx] {
                Some(s) => s,
                None => return,
            };

            // Clear the timer
            if let Some(timer) = service.restart_timer.take() {
                if let Some(events) = &mut self.events {
                    let _ = events.unwatch(timer.handle());
                }
                // timer is dropped here
            }

            match service.state {
                ServiceState::Crashed { .. } => {
                    // Restart the service
                    true
                }
                ServiceState::Failed { .. } => {
                    // Recovery: transition to Pending
                    let name = service.name();
                    service.backoff_ms = INITIAL_BACKOFF_MS;
                    service.transition(ServiceState::Pending, now);
                    dlog!("{} retry after failed (total_restarts={})", name, service.total_restarts);
                    false // check_pending_services will spawn if deps satisfied
                }
                _ => false,
            }
        };

        if should_spawn {
            self.spawn_service(idx, now);
        } else {
            // Check if pending services can now start
            self.check_pending_services();
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
                    // Port event (service connecting)?
                    if let Some(port) = &self.port {
                        if handle == port.handle() {
                            self.handle_port_event();
                            continue;
                        }
                    }

                    // Service restart timer?
                    let mut timer_idx = None;
                    for i in 0..self.service_count {
                        if let Some(service) = &self.services[i] {
                            if let Some(timer) = &service.restart_timer {
                                if timer.handle() == handle {
                                    timer_idx = Some(i);
                                    break;
                                }
                            }
                        }
                    }
                    if let Some(idx) = timer_idx {
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
