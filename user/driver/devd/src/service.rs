//! Service State Machine
//!
//! Defines service lifecycle states, definitions, and runtime instances.
//! Uses trait-based design for testability.

use userlib::ipc::{Channel, Timer, Process, ObjHandle};

// =============================================================================
// Constants
// =============================================================================

pub const MAX_SERVICES: usize = 16;
pub const MAX_CHILDREN_PER_SERVICE: usize = 8;
pub const MAX_PORTS_PER_SERVICE: usize = 4;
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
        matches!(self, ServiceState::Starting | ServiceState::Ready)
    }

    pub fn is_terminal(&self) -> bool {
        matches!(self, ServiceState::Stopped { .. } | ServiceState::Failed { .. })
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
pub static SERVICE_DEFS: &[ServiceDef] = &[
    ServiceDef {
        binary: "consoled",
        registers: &[b"console:"],
        dependencies: &[],  // devd: is implicit
        auto_restart: true,
        parent: None,
    },
    ServiceDef {
        binary: "vfsd",
        registers: &[b"vfs:"],
        dependencies: &[Dependency::Requires(b"console:")],  // Wait for console so we see output
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
    // PCIe bus driver - enumerates PCI devices and registers them with devd
    // TODO: Make this conditional on platform detection (QEMU has ECAM, MT7988 has different)
    ServiceDef {
        binary: "pcied",
        registers: &[b"pcie:"],  // PCI bus port
        dependencies: &[Dependency::Requires(b"console:")],  // Wait for console so we see output
        auto_restart: false,
        parent: None,
    },
    // QEMU USB driver - only works on QEMU virt platform
    // TODO: Make this conditional on platform detection
    ServiceDef {
        binary: "qemu-usbd",
        registers: &[],  // USB driver - doesn't register ports yet
        dependencies: &[Dependency::Requires(b"console:")],  // Wait for console so we see output
        auto_restart: false,  // Don't restart on failure during testing
        parent: None,
    },
];

// =============================================================================
// Service Instance (runtime state)
// =============================================================================

/// A running or tracked service instance
pub struct Service {
    /// Index into SERVICE_DEFS
    pub def_index: u8,
    /// Current state
    pub state: ServiceState,
    /// Process ID (0 if not running)
    pub pid: u32,
    /// Process watcher handle
    pub watcher: Option<Process>,
    /// Channel from this service (for ready announcement)
    pub channel: Option<Channel>,
    /// Indices of child services
    pub children: [Option<u8>; MAX_CHILDREN_PER_SERVICE],
    /// Number of children
    pub child_count: u8,
    /// Parent service index (None = devd)
    pub parent: Option<u8>,
    /// Restart backoff in ms
    pub backoff_ms: u32,
    /// Last state change timestamp
    pub last_change: u64,
    /// Total restarts (for monitoring, not reset)
    pub total_restarts: u32,
    /// Restart timer (set when Crashed or Failed)
    pub restart_timer: Option<Timer>,
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

    /// Initialize services from static definitions
    pub fn init_from_defs(&mut self) {
        for (i, _def) in SERVICE_DEFS.iter().enumerate() {
            if self.count >= MAX_SERVICES {
                break;
            }

            let mut service = Service::empty();
            service.def_index = i as u8;
            service.state = ServiceState::Pending;

            self.services[self.count] = Some(service);
            self.count += 1;
        }

        // Link parents and children
        self.link_parent_child();
    }

    fn link_parent_child(&mut self) {
        // First pass: find parent indices
        for i in 0..self.count {
            if let Some(service) = &self.services[i] {
                if let Some(parent_name) = service.def().parent {
                    // Find parent index
                    let parent_idx = (0..self.count).find(|&j| {
                        self.services[j]
                            .as_ref()
                            .map(|s| s.name() == parent_name)
                            .unwrap_or(false)
                    });

                    if let Some(pidx) = parent_idx {
                        if let Some(s) = &mut self.services[i] {
                            s.parent = Some(pidx as u8);
                        }
                    }
                }
            }
        }

        // Second pass: add children to parents
        for i in 0..self.count {
            let parent_idx = self.services[i]
                .as_ref()
                .and_then(|s| s.parent)
                .map(|p| p as usize);

            if let Some(pidx) = parent_idx {
                if let Some(parent) = &mut self.services[pidx] {
                    parent.add_child(i as u8);
                }
            }
        }
    }

    /// Add a new service (returns index)
    pub fn add(&mut self, service: Service) -> Option<usize> {
        if self.count >= MAX_SERVICES {
            return None;
        }
        let idx = self.count;
        self.services[idx] = Some(service);
        self.count += 1;
        Some(idx)
    }

    /// Find an empty service slot (for dynamic drivers)
    ///
    /// Unlike `add()` which only uses count, this searches for any None slot.
    /// Used for dynamically spawned drivers that don't have static definitions.
    pub fn find_empty_slot(&self) -> Option<usize> {
        (0..MAX_SERVICES).find(|&i| self.services[i].is_none())
    }

    /// Ensure count includes a specific slot index (for dynamic services)
    pub fn ensure_count_includes(&mut self, slot_idx: usize) {
        if slot_idx >= self.count {
            self.count = slot_idx + 1;
        }
    }

    /// Create a dynamic service slot for a spawned child
    ///
    /// Returns the slot index if successful, None if no slots available.
    pub fn create_dynamic_service(&mut self, pid: u32, now: u64) -> Option<usize> {
        let slot_idx = self.find_empty_slot()?;

        // Create a new service in the slot
        let mut service = Service::empty();
        service.pid = pid;
        service.state = ServiceState::Ready;  // Children are ready immediately
        service.last_change = now;
        service.def_index = 0xFF;  // No static definition

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
