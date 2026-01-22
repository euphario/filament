//! Unified Process Supervisor
//!
//! Single state machine for all supervised processes.
//!
//! # Model
//!
//! Everything is ports. Kernel exports platform ports, bus drivers export
//! device ports, drivers export service ports.
//!
//! ```text
//! kernel → uart:   → consoled → console: → shell
//! kernel → gpio:   → gpio
//! kernel → pcie0:  → pcied → mt7996e: → wifid → wifi:
//! kernel → usb0:   → usbd  → msc:     → fatfs
//! ```

pub mod process;

pub use process::{Process, ProcessConfig, ProcessState};

use crate::state::{Effect, Effects};
use crate::effects::{SyscallExecutor, SpawnResult, EffectExecutor};
use userlib::syscall::{self, Handle, handle_timer_set};

/// Maximum supervised processes
pub const MAX_PROCESSES: usize = 24;

/// Maximum ports tracked
pub const MAX_PORTS: usize = 32;

/// Supervisor - manages all process lifecycles
pub struct Supervisor {
    processes: [Option<Process>; MAX_PROCESSES],
    count: usize,

    /// Known available ports: (name, providing process index)
    /// Index usize::MAX means kernel-provided port
    ports: [Option<(&'static str, usize)>; MAX_PORTS],
    port_count: usize,

    executor: SyscallExecutor,

    /// Timer handle for deadline scheduling (handle API)
    timer_handle: Handle,
}

impl Supervisor {
    pub const fn new() -> Self {
        const NONE_PROC: Option<Process> = None;
        const NONE_PORT: Option<(&'static str, usize)> = None;
        Self {
            processes: [NONE_PROC; MAX_PROCESSES],
            count: 0,
            ports: [NONE_PORT; MAX_PORTS],
            port_count: 0,
            executor: SyscallExecutor::new(),
            timer_handle: Handle::INVALID,
        }
    }

    /// Set the timer handle for deadline scheduling (call after handle_timer_create)
    pub fn set_timer_handle(&mut self, handle: Handle) {
        self.timer_handle = handle;
        self.executor.set_timer_handle(handle);
    }

    /// Register a kernel-provided port (uart:, gpio:, pcie0:, etc.)
    pub fn register_kernel_port(&mut self, name: &'static str) {
        if self.port_count < MAX_PORTS {
            self.ports[self.port_count] = Some((name, usize::MAX));
            self.port_count += 1;
        }
    }

    /// Register a process
    pub fn register(&mut self, config: ProcessConfig) -> Option<usize> {
        if self.count >= MAX_PROCESSES {
            return None;
        }

        let idx = self.count;
        self.processes[idx] = Some(Process::new(config));
        self.count += 1;
        Some(idx)
    }

    /// Adopt an already-running process
    pub fn adopt(&mut self, config: ProcessConfig, pid: u32) -> Option<usize> {
        if self.count >= MAX_PROCESSES {
            return None;
        }

        let idx = self.count;
        self.processes[idx] = Some(Process::new_adopted(config, pid));
        self.count += 1;
        Some(idx)
    }

    /// Start a process by index
    pub fn start(&mut self, idx: usize) -> Option<u32> {
        let now = syscall::gettime() as u64;

        // Check dependency and get config - scoped to release borrow
        let (dep_satisfied, has_dep, binary) = {
            let process = self.processes.get(idx)?.as_ref()?;
            let satisfied = self.is_port_available(process.config.depends_on);
            (satisfied, process.config.depends_on.is_some(), process.config.binary)
        };

        // Handle Start event
        {
            let process = self.processes.get_mut(idx)?.as_mut()?;
            if let Some(trans) = process.handle(process::Event::Start, now) {
                process.apply(trans);
            }
        }

        if has_dep && !dep_satisfied {
            // Waiting for dependency
            let process = self.processes.get_mut(idx)?.as_mut()?;
            if let Some(trans) = process.handle(process::Event::DependencyUnavailable, now) {
                process.apply(trans);
            }
            return None;
        }

        // Dependency satisfied or none - get effects to spawn
        let effects = {
            let process = self.processes.get_mut(idx)?.as_mut()?;
            if let Some(trans) = process.handle(process::Event::DependencyAvailable, now) {
                process.apply(trans)
            } else {
                return None;
            }
        };

        // Execute spawn effects
        if let Some(SpawnResult::Success { pid }) = self.execute_effects(effects, binary) {
            // Handle spawned event - scoped borrow
            let effects = {
                let process = self.processes.get_mut(idx)?.as_mut()?;
                if let Some(trans) = process.handle(process::Event::Spawned { pid }, now) {
                    Some(process.apply(trans))
                } else {
                    None
                }
            };
            if let Some(eff) = effects {
                self.execute_effects(eff, binary);
            }
            return Some(pid);
        }

        None
    }

    /// Start all auto-start processes whose dependencies are satisfied
    pub fn start_ready(&mut self) -> usize {
        let mut started = 0;

        for idx in 0..self.count {
            let should_start = self.processes[idx]
                .as_ref()
                .map(|p| matches!(p.state, ProcessState::Stopped) && p.config.auto_start)
                .unwrap_or(false);

            if should_start {
                if self.start(idx).is_some() {
                    started += 1;
                }
            }
        }

        started
    }

    /// Handle process exit
    pub fn handle_exit(&mut self, pid: u32, code: i32) -> bool {
        let now = syscall::gettime() as u64;

        let idx = match self.find_by_pid(pid) {
            Some(i) => i,
            None => return false,
        };

        // Get binary name first
        let binary = match self.processes[idx].as_ref() {
            Some(p) => p.config.binary,
            None => return false,
        };

        if code != 0 {
            syscall::debug(b"[devd] CRASH ");
            syscall::debug(binary.as_bytes());
            syscall::debug(b"\n");
        }

        // Remove ports this process provided
        self.remove_ports_for(idx);

        // Handle exit event - get effects
        let effects = {
            let process = match self.processes[idx].as_mut() {
                Some(p) => p,
                None => return false,
            };
            if let Some(trans) = process.handle(process::Event::ProcessExit { pid, code }, now) {
                Some(process.apply(trans))
            } else {
                None
            }
        };

        // Execute effects outside of borrow
        if let Some(eff) = effects {
            self.execute_effects(eff, binary);
        }

        // Check if crashed and evaluate recovery
        let is_crashed = self.processes[idx]
            .as_ref()
            .map(|p| matches!(p.state, ProcessState::Crashed { .. }))
            .unwrap_or(false);

        if is_crashed {
            // Get recovery transition
            let (effects, log_info) = {
                let process = self.processes[idx].as_mut().unwrap();
                let trans = process.evaluate_crash(now);

                let log_info = match &trans.state {
                    ProcessState::PendingRestart { restart_at } => {
                        let delay_ms = restart_at.saturating_sub(now) / 1_000_000;
                        Some((true, delay_ms))
                    }
                    ProcessState::Failed => Some((false, 0)),
                    _ => None,
                };

                (process.apply(trans), log_info)
            };

            // Log failures
            if let Some((pending, _delay_ms)) = log_info {
                if !pending {
                    syscall::debug(b"[devd] FAILED ");
                    syscall::debug(binary.as_bytes());
                    syscall::debug(b"\n");
                }
            }

            self.execute_effects(effects, binary);
        }

        // Notify dependents that ports disappeared
        self.handle_dependency_lost(idx);

        true
    }

    /// Handle port registration
    pub fn handle_port_ready(&mut self, port: &'static str, pid: u32) {
        let now = syscall::gettime() as u64;

        let idx = match self.find_by_pid(pid) {
            Some(i) => i,
            None => {
                // External port registration - just record it
                self.add_port(port, usize::MAX);
                self.wake_waiting_for_port(port);
                return;
            }
        };

        // Record the port
        self.add_port(port, idx);

        // Notify the process - scoped borrow to get effects
        let effects_and_binary = {
            if let Some(process) = self.processes[idx].as_mut() {
                if let Some(trans) = process.handle(process::Event::PortRegistered, now) {
                    let binary = process.config.binary;
                    let effects = process.apply(trans);
                    Some((effects, binary))
                } else {
                    None
                }
            } else {
                None
            }
        };

        // Execute effects outside of borrow
        if let Some((effects, binary)) = effects_and_binary {
            self.execute_effects(effects, binary);
        }

        // Wake processes waiting for this port
        self.wake_waiting_for_port(port);
    }

    /// Handle timer expiry
    pub fn handle_timer(&mut self) {
        let now = syscall::gettime() as u64;

        for idx in 0..self.count {
            // Extract state info in scoped borrow
            let (binary, depends_on, state_kind) = {
                let process = match self.processes[idx].as_ref() {
                    Some(p) => p,
                    None => continue,
                };

                // Check deadline is expired
                match process.state.deadline() {
                    Some(d) if now >= d => {},
                    _ => continue,
                }

                let state_kind = match &process.state {
                    ProcessState::PendingRestart { .. } => 1,
                    ProcessState::Starting { .. } | ProcessState::Binding { .. } => 2,
                    _ => 0,
                };

                (process.config.binary, process.config.depends_on, state_kind)
            };

            if state_kind == 0 {
                continue;
            }

            if state_kind == 1 {
                // PendingRestart - check dependency
                let port_available = self.is_port_available(depends_on);

                if port_available {
                    // Get effects from transition
                    let effects = {
                        let process = self.processes[idx].as_mut().unwrap();
                        if let Some(trans) = process.handle(process::Event::DeadlineExpired, now) {
                            Some(process.apply(trans))
                        } else {
                            None
                        }
                    };

                    // Execute spawn and handle result
                    if let Some(eff) = effects {
                        if let Some(SpawnResult::Success { pid }) = self.execute_effects(eff, binary) {
                            // Handle spawned event
                            let effects = {
                                let process = self.processes[idx].as_mut().unwrap();
                                if let Some(trans) = process.handle(process::Event::Spawned { pid }, now) {
                                    Some(process.apply(trans))
                                } else {
                                    None
                                }
                            };
                            if let Some(eff) = effects {
                                self.execute_effects(eff, binary);
                            }
                        }
                    }
                } else {
                    // Dependency not ready - go back to waiting
                    let process = self.processes[idx].as_mut().unwrap();
                    if let Some(trans) = process.handle(process::Event::DependencyUnavailable, now) {
                        process.apply(trans);
                    }
                }
            } else if state_kind == 2 {
                // Starting or Binding timeout
                let effects = {
                    let process = self.processes[idx].as_mut().unwrap();
                    if let Some(trans) = process.handle(process::Event::DeadlineExpired, now) {
                        Some(process.apply(trans))
                    } else {
                        None
                    }
                };
                if let Some(eff) = effects {
                    self.execute_effects(eff, binary);
                }
            }
        }

        self.schedule_next_deadline();
    }

    // === Helpers ===

    fn find_by_pid(&self, pid: u32) -> Option<usize> {
        (0..self.count).find(|&i| {
            self.processes[i]
                .as_ref()
                .map(|p| p.state.pid() == Some(pid))
                .unwrap_or(false)
        })
    }

    fn is_port_available(&self, port: Option<&'static str>) -> bool {
        match port {
            None => true,
            Some(name) => self.ports[..self.port_count]
                .iter()
                .any(|p| p.map(|(n, _)| n == name).unwrap_or(false)),
        }
    }

    fn add_port(&mut self, name: &'static str, process_idx: usize) {
        // Check if exists
        for p in &self.ports[..self.port_count] {
            if let Some((n, _)) = p {
                if *n == name {
                    return;
                }
            }
        }

        if self.port_count < MAX_PORTS {
            self.ports[self.port_count] = Some((name, process_idx));
            self.port_count += 1;
        }
    }

    fn remove_ports_for(&mut self, process_idx: usize) {
        let mut i = 0;
        while i < self.port_count {
            if let Some((_, idx)) = self.ports[i] {
                if idx == process_idx {
                    self.ports[i] = self.ports[self.port_count - 1];
                    self.ports[self.port_count - 1] = None;
                    self.port_count -= 1;
                    continue;
                }
            }
            i += 1;
        }
    }

    fn wake_waiting_for_port(&mut self, port: &'static str) {
        let now = syscall::gettime() as u64;

        for idx in 0..self.count {
            // Check if should wake and get binary - scoped borrow
            let wake_info = {
                let process = match self.processes[idx].as_ref() {
                    Some(p) => p,
                    None => continue,
                };
                if matches!(process.state, ProcessState::WaitingDependency)
                    && process.config.depends_on == Some(port)
                {
                    Some(process.config.binary)
                } else {
                    None
                }
            };

            let binary = match wake_info {
                Some(b) => b,
                None => continue,
            };

            // Get effects from DependencyAvailable transition
            let effects = {
                let process = self.processes[idx].as_mut().unwrap();
                if let Some(trans) = process.handle(process::Event::DependencyAvailable, now) {
                    Some(process.apply(trans))
                } else {
                    None
                }
            };

            // Execute spawn effects
            if let Some(eff) = effects {
                if let Some(SpawnResult::Success { pid }) = self.execute_effects(eff, binary) {
                    // Handle spawned event
                    let effects = {
                        let process = self.processes[idx].as_mut().unwrap();
                        if let Some(trans) = process.handle(process::Event::Spawned { pid }, now) {
                            Some(process.apply(trans))
                        } else {
                            None
                        }
                    };
                    if let Some(eff) = effects {
                        self.execute_effects(eff, binary);
                    }
                }
            }
        }
    }

    fn handle_dependency_lost(&mut self, _crashed_idx: usize) {
        // TODO: Notify/stop processes that depended on crashed process's ports
        // For now, they'll fail when they try to use the port
    }

    fn execute_effects(&mut self, effects: Effects, context: &str) -> Option<SpawnResult> {
        let mut spawn_result = None;

        for effect in effects.iter() {
            match effect {
                Effect::ScheduleDeadline(deadline) => {
                    self.executor.schedule_deadline(*deadline);
                }
                Effect::CancelTimer => {
                    self.executor.cancel_timer();
                }
                Effect::SpawnProcess { binary } => {
                    spawn_result = Some(self.executor.spawn_process(binary));
                }
                Effect::KillProcess { pid } => {
                    self.executor.kill_process(*pid);
                }
                Effect::Log { level, message } => {
                    self.executor.log(*level, context, message);
                }
                Effect::RequestBusReset { bus_type, bus_index, level } => {
                    self.executor.request_bus_reset(*bus_type, *bus_index, *level);
                }
            }
        }

        spawn_result
    }

    fn schedule_next_deadline(&mut self) {
        let mut earliest: u64 = 0;

        for i in 0..self.count {
            if let Some(ref process) = self.processes[i] {
                if let Some(deadline) = process.state.deadline() {
                    if earliest == 0 || deadline < earliest {
                        earliest = deadline;
                    }
                }
            }
        }

        if earliest > 0 {
            let now = syscall::gettime() as u64;
            let delay = earliest.saturating_sub(now);
            // Timer handle must be set before use
            if delay > 0 && self.timer_handle.is_valid() {
                let _ = handle_timer_set(self.timer_handle, delay, 0);
            }
        }
    }

    /// Get process count
    pub fn count(&self) -> usize {
        self.count
    }

    /// Dump state (for debugging)
    #[allow(dead_code)]
    pub fn dump_state(&self) {
        for i in 0..self.count {
            if let Some(ref p) = self.processes[i] {
                syscall::debug(b"[devd] ");
                syscall::debug(p.config.binary.as_bytes());
                syscall::debug(b": ");
                syscall::debug(p.state.name().as_bytes());
                syscall::debug(b"\n");
            }
        }
    }
}
