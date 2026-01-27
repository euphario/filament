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
use userlib::ipc::{Port, Timer, EventLoop, ObjHandle};
use userlib::error::{SysError, SysResult};

use service::{
    ServiceState, ServiceManager, ServiceRegistry,
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
        self.ports.register(b"devd:", 0xFF)?; // 0xFF = devd itself
        self.ports.register(b"devd-query:", 0xFF)?;

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
        let binary = match self.services.get(idx) {
            Some(s) => s.def().binary,
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

        // Collect port names before mutable borrow
        let port_names: [Option<&'static [u8]>; MAX_PORTS_PER_SERVICE] = {
            match self.services.get(idx) {
                Some(s) => {
                    let mut arr = [None; MAX_PORTS_PER_SERVICE];
                    for (i, &pn) in s.def().registers.iter().enumerate() {
                        if i < MAX_PORTS_PER_SERVICE {
                            arr[i] = Some(pn);
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
        for port_name in port_names.iter().flatten() {
            let _ = self.ports.register(port_name, idx as u8);
            self.ports.set_availability(port_name, false);
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
        let (port_names, should_prune, action) = {
            let service = match self.services.get_mut(idx) {
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

            // Collect port names
            let mut port_arr = [None; MAX_PORTS_PER_SERVICE];
            for (i, &pn) in service.def().registers.iter().enumerate() {
                if i < MAX_PORTS_PER_SERVICE {
                    port_arr[i] = Some(pn);
                }
            }

            if code == 0 {
                service.state = ServiceState::Stopped { code: 0 };
                service.last_change = now;
                (port_arr, false, ExitAction::Stopped)
            } else {
                service.total_restarts = service.total_restarts.saturating_add(1);

                let restarts = match service.state {
                    ServiceState::Crashed { restarts, .. } => restarts + 1,
                    _ => 1,
                };

                if restarts >= MAX_RESTARTS || !auto_restart {
                    service.state = ServiceState::Failed { code };
                    service.last_change = now;
                    derror!("{} failed code={} restarts={}", name, code, service.total_restarts);
                    (port_arr, false, if auto_restart { ExitAction::Failed } else { ExitAction::Stopped })
                } else {
                    service.state = ServiceState::Crashed { code, restarts };
                    service.last_change = now;
                    service.backoff_ms = (service.backoff_ms * 2).min(MAX_BACKOFF_MS);
                    (port_arr, true, ExitAction::Crashed { backoff_ms: service.backoff_ms })
                }
            }
        };

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
            for (i, &pn) in service.def().registers.iter().enumerate() {
                if i < MAX_PORTS_PER_SERVICE {
                    arr[i] = Some(pn);
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
            for (i, &pn) in service.def().registers.iter().enumerate() {
                if i < MAX_PORTS_PER_SERVICE {
                    arr[i] = Some(pn);
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
                    // Unknown connection - drop for now
                    drop(channel);
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
            if service.state == ServiceState::Starting
                && service.def().registers.is_empty()
                && service.pid != 0
            {
                dlog!("service {} state: Starting -> Ready (portless)", service.name());
                service.state = ServiceState::Ready;
                service.last_change = now;
                service.backoff_ms = INITIAL_BACKOFF_MS;
            }
        });
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
                    let name = service.name();
                    service.backoff_ms = INITIAL_BACKOFF_MS;
                    service.state = ServiceState::Pending;
                    service.last_change = now;
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
                let service_idx = self.services.find_by_pid(client_pid)
                    .filter(|&i| {
                        self.services.get(i)
                            .map(|s| s.state == ServiceState::Ready)
                            .unwrap_or(false)
                    })
                    .map(|i| i as u8);

                // Add to event loop
                if let Some(events) = &mut self.events {
                    let _ = events.watch(channel.handle());
                }

                // Add to query handler
                match self.query_handler.add_client(channel, service_idx) {
                    Some(slot) => {
                        let client_type = if service_idx.is_some() { "driver" } else { "client" };
                        dlog!("query: {} connected (slot={}, pid={})", client_type, slot, client_pid);
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
                    Some(msg::QUERY_DRIVER) => {
                        // Forward to driver
                        self.handle_query_driver_forward(slot, &recv_buf[..len]);
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
            Err(_) => error::INVALID_REQUEST,
        };
        self.query_handler.send_port_register_response(slot, info.seq_id, result_code);
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
        _shmem_id: u32,
    ) -> Result<(), SysError> {
        // Register the port
        if let Some(parent) = parent_name {
            self.ports.register_child(port_name, owner_idx, parent, port_type)?;
        } else {
            self.ports.register_typed(port_name, owner_idx, port_type)?;
        }

        // Get parent type for rule matching
        let parent_type = parent_name.and_then(|p| self.ports.port_type(p));

        // Log registration
        if let Ok(name_str) = core::str::from_utf8(port_name) {
            dlog!("port registered: {} type={:?} parent={:?}",
                name_str, port_type, parent_type);
        }

        // Check rules for auto-spawning
        self.check_rules_for_port(port_name, port_type, parent_type);

        Ok(())
    }

    /// Check if any rules match a newly registered port
    fn check_rules_for_port(
        &mut self,
        port_name: &[u8],
        port_type: PortType,
        parent_type: Option<PortType>,
    ) {
        let rule = match self.rules.find_matching_rule(port_type, parent_type) {
            Some(r) => r,
            None => return,
        };

        dlog!("rule matched: spawning {} for port", rule.driver_binary);

        // Spawn the driver
        // For now, we use a dynamic service slot
        // In the future, this could use a more sophisticated approach
        self.spawn_dynamic_driver(rule.driver_binary, port_name);
    }

    /// Spawn a driver dynamically (not from SERVICE_DEFS)
    fn spawn_dynamic_driver(&mut self, binary: &str, trigger_port: &[u8]) {
        let now = Self::now_ms();

        // Spawn the process
        let (pid, watcher) = match self.process_mgr.spawn(binary) {
            Ok(result) => result,
            Err(e) => {
                derror!("dynamic spawn {} failed err={:?}", binary, e);
                return;
            }
        };

        // Add watcher to event loop
        if let Some(events) = &mut self.events {
            let _ = events.watch(watcher.handle());
        }

        // Find empty service slot for dynamic driver
        let slot = self.services.find_empty_slot();
        if let Some(idx) = slot {
            // Use the slot for tracking this dynamic driver
            if let Some(service) = self.services.get_mut(idx) {
                service.pid = pid;
                service.watcher = Some(watcher);
                service.state = service::ServiceState::Starting;
                service.last_change = now;
            }
        }

        if let Ok(port_str) = core::str::from_utf8(trigger_port) {
            dlog!("spawned {} pid={} for trigger={}", binary, pid, port_str);
        } else {
            dlog!("spawned {} pid={}", binary, pid);
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
