//! Service State Machine
//!
//! Defines service lifecycle states and runtime instances.
//! All services are dynamically spawned via PORT_RULES or boot services.
//! Uses trait-based design for testability.

use userlib::ipc::{Channel, Timer, Process, ObjHandle};

// =============================================================================
// Constants
// =============================================================================

pub const MAX_SERVICES: usize = 16;
pub const MAX_RESTARTS: u8 = 3;
pub const INITIAL_BACKOFF_MS: u32 = 1000;
pub const MAX_BACKOFF_MS: u32 = 30000;
/// Time before Failed services are retried (5 minutes)
pub const FAILED_RETRY_MS: u64 = 5 * 60 * 1000;

// =============================================================================
// Service State Machine
// =============================================================================

/// Service lifecycle state
///
/// All transitions are event-driven, no polling:
///
///   Starting ──[service reports Ready via STATE_CHANGE]──► Ready
///       Event: Driver sends STATE_CHANGE(Ready) on query channel
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
///   Failed ──[recovery timer fires]──► Starting (retry)
///       Event: Per-service Timer fires (5 minute delay)
///
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServiceState {
    /// Waiting (unused — kept for compatibility with handle_service_ready callers)
    Pending,
    /// Spawned, waiting for service to report Ready
    Starting,
    /// Service is ready
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
        matches!(self, ServiceState::Starting | ServiceState::Ready)
    }

    pub fn is_terminal(&self) -> bool {
        matches!(self, ServiceState::Stopped { .. } | ServiceState::Failed { .. })
    }

    /// Service exited and can be respawned (Stopped, Crashed, or Failed)
    pub fn is_exited(&self) -> bool {
        matches!(self, ServiceState::Stopped { .. } | ServiceState::Crashed { .. } | ServiceState::Failed { .. })
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            ServiceState::Pending => "pending",
            ServiceState::Starting => "starting",
            ServiceState::Ready => "ready",
            ServiceState::Stopped { .. } => "stopped",
            ServiceState::Crashed { .. } => "crashed",
            ServiceState::Failed { .. } => "failed",
        }
    }
}

// =============================================================================
// Service Instance (runtime state)
// =============================================================================

/// A running or tracked service instance.
///
/// All services are created dynamically — there are no static definitions.
/// Services are spawned either by PORT_RULES (when a port transitions to Ready)
/// or as boot services (spawned directly during init).
pub struct Service {
    /// Current state
    pub state: ServiceState,
    /// Process ID (0 if not running)
    pub pid: u32,
    /// Process watcher handle
    pub watcher: Option<Process>,
    /// Channel from this service (for ready announcement)
    pub channel: Option<Channel>,
    /// Restart backoff in ms
    pub backoff_ms: u32,
    /// Last state change timestamp
    pub last_change: u64,
    /// Total restarts (for monitoring, not reset)
    pub total_restarts: u32,
    /// Restart timer (set when Crashed or Failed)
    pub restart_timer: Option<Timer>,
    /// Service binary name
    pub dynamic_name: [u8; 16],
    /// Port name that triggered this service's spawn (for restart)
    pub trigger_port: [u8; 32],
    /// Length of trigger_port name
    pub trigger_port_len: u8,
    /// Capability bits (stored at spawn time, used for restart)
    pub caps: u64,
    /// Unique link ID tying this service to its trigger port (0 = unlinked)
    pub link_id: u32,
}

impl Service {
    pub const fn empty() -> Self {
        Self {
            state: ServiceState::Pending,
            pid: 0,
            watcher: None,
            channel: None,
            backoff_ms: INITIAL_BACKOFF_MS,
            last_change: 0,
            total_restarts: 0,
            restart_timer: None,
            dynamic_name: [0; 16],
            trigger_port: [0; 32],
            trigger_port_len: 0,
            caps: 0,
            link_id: 0,
        }
    }

    /// Get service name
    pub fn name(&self) -> &str {
        let len = self.dynamic_name.iter().position(|&b| b == 0).unwrap_or(16);
        core::str::from_utf8(&self.dynamic_name[..len]).unwrap_or("???")
    }

    /// Set service name
    pub fn set_dynamic_name(&mut self, name: &[u8]) {
        let len = name.len().min(16);
        self.dynamic_name[..len].copy_from_slice(&name[..len]);
        if len < 16 {
            self.dynamic_name[len..].fill(0);
        }
    }

    /// Set trigger port (the port that caused this service to be spawned)
    pub fn set_trigger_port(&mut self, port_name: &[u8]) {
        let len = port_name.len().min(32);
        self.trigger_port[..len].copy_from_slice(&port_name[..len]);
        if len < 32 {
            self.trigger_port[len..].fill(0);
        }
        self.trigger_port_len = len as u8;
    }

    /// Get trigger port name (empty slice if none)
    pub fn trigger_port(&self) -> &[u8] {
        &self.trigger_port[..self.trigger_port_len as usize]
    }
}

// =============================================================================
// ServiceManager Trait
// =============================================================================

/// Service lifecycle management trait
pub trait ServiceManager {
    /// Get service by index (read-only)
    fn get(&self, idx: usize) -> Option<&Service>;

    /// Get service by index (mutable)
    fn get_mut(&mut self, idx: usize) -> Option<&mut Service>;

    /// Transition a service to a new state
    fn transition(&mut self, idx: usize, state: ServiceState, now: u64);

    /// Find service by PID
    fn find_by_pid(&self, pid: u32) -> Option<usize>;

    /// Find service by binary name
    fn find_by_name(&self, name: &str) -> Option<usize>;

    /// Get total service count
    fn count(&self) -> usize;

    /// Find service by watcher handle
    fn find_by_watcher(&self, handle: ObjHandle) -> Option<usize>;

    /// Find service by restart timer handle
    fn find_by_timer(&self, handle: ObjHandle) -> Option<usize>;

    /// Iterate over all services
    fn for_each<F: FnMut(usize, &Service)>(&self, f: F);

    /// Iterate over all services (mutable)
    fn for_each_mut<F: FnMut(usize, &mut Service)>(&mut self, f: F);
}

// =============================================================================
// ServiceRegistry Implementation
// =============================================================================

/// Concrete implementation of ServiceManager
pub struct ServiceRegistry {
    services: [Option<Service>; MAX_SERVICES],
    count: usize,
}

impl ServiceRegistry {
    pub const fn new() -> Self {
        Self {
            services: [const { None }; MAX_SERVICES],
            count: 0,
        }
    }

    /// Find an empty service slot
    pub fn find_empty_slot(&self) -> Option<usize> {
        (0..MAX_SERVICES).find(|&i| self.services[i].is_none())
    }

    /// Find a service by binary name
    pub fn find_by_binary(&self, binary: &str) -> Option<usize> {
        (0..self.count).find(|&i| {
            self.services[i]
                .as_ref()
                .map(|s| s.name() == binary)
                .unwrap_or(false)
        })
    }

    /// Ensure count includes a specific slot index
    pub fn ensure_count_includes(&mut self, slot_idx: usize) {
        if slot_idx >= self.count {
            self.count = slot_idx + 1;
        }
    }

    /// Create a service slot for a spawned process.
    ///
    /// Returns the slot index if successful, None if no slots available.
    pub fn create_dynamic_service(&mut self, pid: u32, name: &[u8], now: u64) -> Option<usize> {
        self.create_dynamic_service_with_state(pid, name, now, ServiceState::Ready)
    }

    pub fn create_dynamic_service_with_state(
        &mut self,
        pid: u32,
        name: &[u8],
        now: u64,
        initial_state: ServiceState,
    ) -> Option<usize> {
        let slot_idx = self.find_empty_slot()?;

        let mut service = Service::empty();
        service.pid = pid;
        service.state = initial_state;
        service.last_change = now;
        service.set_dynamic_name(name);

        self.services[slot_idx] = Some(service);
        self.ensure_count_includes(slot_idx);

        Some(slot_idx)
    }
}

impl ServiceManager for ServiceRegistry {
    fn get(&self, idx: usize) -> Option<&Service> {
        self.services.get(idx).and_then(|s| s.as_ref())
    }

    fn get_mut(&mut self, idx: usize) -> Option<&mut Service> {
        self.services.get_mut(idx).and_then(|s| s.as_mut())
    }

    fn transition(&mut self, idx: usize, state: ServiceState, now: u64) {
        if let Some(service) = self.get_mut(idx) {
            service.state = state;
            service.last_change = now;
        }
    }

    fn find_by_pid(&self, pid: u32) -> Option<usize> {
        (0..self.count).find(|&i| {
            self.services[i]
                .as_ref()
                .map(|s| s.pid == pid)
                .unwrap_or(false)
        })
    }

    fn find_by_name(&self, name: &str) -> Option<usize> {
        (0..self.count).find(|&i| {
            self.services[i]
                .as_ref()
                .map(|s| s.name() == name)
                .unwrap_or(false)
        })
    }

    fn count(&self) -> usize {
        self.count
    }

    fn find_by_watcher(&self, handle: ObjHandle) -> Option<usize> {
        (0..self.count).find(|&i| {
            self.services[i]
                .as_ref()
                .and_then(|s| s.watcher.as_ref())
                .map(|w| w.handle() == handle)
                .unwrap_or(false)
        })
    }

    fn find_by_timer(&self, handle: ObjHandle) -> Option<usize> {
        (0..self.count).find(|&i| {
            self.services[i]
                .as_ref()
                .and_then(|s| s.restart_timer.as_ref())
                .map(|t| t.handle() == handle)
                .unwrap_or(false)
        })
    }

    fn for_each<F: FnMut(usize, &Service)>(&self, mut f: F) {
        for i in 0..self.count {
            if let Some(service) = &self.services[i] {
                f(i, service);
            }
        }
    }

    fn for_each_mut<F: FnMut(usize, &mut Service)>(&mut self, mut f: F) {
        for i in 0..self.count {
            if let Some(service) = &mut self.services[i] {
                f(i, service);
            }
        }
    }
}
