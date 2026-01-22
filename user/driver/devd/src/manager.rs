//! Driver Manager
//!
//! Orchestrates driver state machines and executes effects.
//! This replaces the scattered driver management in DeviceManager.

use crate::state::driver::{Driver, DriverConfig, DriverState};
use crate::state::{Effect, Effects, Event};
use crate::effects::{SyscallExecutor, SpawnResult};
use userlib::syscall;

/// Maximum number of managed drivers
pub const MAX_DRIVERS: usize = 16;

/// Driver manager - orchestrates driver state machines
pub struct DriverManager {
    drivers: [Option<Driver>; MAX_DRIVERS],
    count: usize,
    executor: SyscallExecutor,
}

impl DriverManager {
    pub const fn new() -> Self {
        const NONE: Option<Driver> = None;
        Self {
            drivers: [NONE; MAX_DRIVERS],
            count: 0,
            executor: SyscallExecutor::new(),
        }
    }

    /// Register a driver with configuration
    pub fn register(&mut self, binary: &'static str, auto_restart: bool, has_device: bool) -> Option<usize> {
        if self.count >= MAX_DRIVERS {
            return None;
        }

        let config = DriverConfig {
            binary,
            has_device,
            bus_type: None,
            bus_index: None,
            auto_restart,
            start_timeout_ns: DriverConfig::DEFAULT_START_TIMEOUT,
            bind_timeout_ns: DriverConfig::DEFAULT_BIND_TIMEOUT,
            restart_delay_ns: DriverConfig::DEFAULT_RESTART_DELAY,
        };

        let idx = self.count;
        self.drivers[idx] = Some(Driver::new(config));
        self.count += 1;
        Some(idx)
    }

    /// Register a driver for a device (with bus info)
    pub fn register_device_driver(
        &mut self,
        binary: &'static str,
        bus_type: u8,
        bus_index: u8,
    ) -> Option<usize> {
        if self.count >= MAX_DRIVERS {
            return None;
        }

        let config = DriverConfig {
            binary,
            has_device: true,
            bus_type: Some(bus_type),
            bus_index: Some(bus_index),
            auto_restart: true,
            start_timeout_ns: DriverConfig::DEFAULT_START_TIMEOUT,
            bind_timeout_ns: DriverConfig::DEFAULT_BIND_TIMEOUT,
            restart_delay_ns: DriverConfig::DEFAULT_RESTART_DELAY,
        };

        let idx = self.count;
        self.drivers[idx] = Some(Driver::new(config));
        self.count += 1;
        Some(idx)
    }

    /// Adopt an already-running process as a managed driver
    /// Used when process was spawned externally (e.g., by ServiceManager)
    pub fn adopt(&mut self, binary: &'static str, pid: u32, auto_restart: bool) -> Option<usize> {
        if self.count >= MAX_DRIVERS {
            return None;
        }

        let config = DriverConfig {
            binary,
            has_device: false,
            bus_type: None,
            bus_index: None,
            auto_restart,
            start_timeout_ns: DriverConfig::DEFAULT_START_TIMEOUT,
            bind_timeout_ns: DriverConfig::DEFAULT_BIND_TIMEOUT,
            restart_delay_ns: DriverConfig::DEFAULT_RESTART_DELAY,
        };

        let idx = self.count;
        self.drivers[idx] = Some(Driver::new_adopted(config, pid));
        self.count += 1;
        crate::dlog!("driver_adopted binary={} pid={} idx={}", binary, pid, idx);
        Some(idx)
    }

    /// Get driver by index
    pub fn get(&self, idx: usize) -> Option<&Driver> {
        self.drivers.get(idx).and_then(|d| d.as_ref())
    }

    /// Find driver by pid
    pub fn find_by_pid(&self, pid: u32) -> Option<usize> {
        for i in 0..self.count {
            if let Some(ref driver) = self.drivers[i] {
                if driver.state.pid() == Some(pid) {
                    return Some(i);
                }
            }
        }
        None
    }

    /// Find driver by binary name
    pub fn find_by_binary(&self, binary: &str) -> Option<usize> {
        for i in 0..self.count {
            if let Some(ref driver) = self.drivers[i] {
                if driver.config.binary == binary {
                    return Some(i);
                }
            }
        }
        None
    }

    /// Start a driver by index
    pub fn start(&mut self, idx: usize) -> Option<u32> {
        let now = syscall::gettime() as u64;

        let driver = self.drivers.get_mut(idx)?.as_mut()?;
        let binary = driver.config.binary;

        // Handle Start event
        let trans = driver.handle(Event::Start, now)?;
        let effects = driver.apply(trans);

        // Execute effects
        let spawn_result = self.execute_effects(effects, binary);

        // If spawn succeeded, handle Spawned event
        if let Some(SpawnResult::Success { pid }) = spawn_result {
            if let Some(trans) = driver.handle(Event::Spawned { pid }, now) {
                let effects = driver.apply(trans);
                self.execute_effects(effects, binary);
            }
            return Some(pid);
        }

        None
    }

    /// Start a driver by binary name
    pub fn start_by_name(&mut self, binary: &str) -> Option<u32> {
        let idx = self.find_by_binary(binary)?;
        self.start(idx)
    }

    /// Handle process exit event
    pub fn handle_exit(&mut self, pid: u32, code: i32) -> bool {
        let now = syscall::gettime() as u64;

        let idx = match self.find_by_pid(pid) {
            Some(i) => i,
            None => return false,
        };

        let driver = match self.drivers[idx].as_mut() {
            Some(d) => d,
            None => return false,
        };

        let binary = driver.config.binary;

        // Log the exit
        if code == 0 {
            crate::dlog!("driver_exit_ok binary={} pid={}", binary, pid);
        } else {
            crate::derror!("driver_crashed binary={} pid={} code={}", binary, pid, code);
        }

        // Handle ProcessExit event
        let event = Event::ProcessExit { pid, code };
        if let Some(trans) = driver.handle(event, now) {
            let effects = driver.apply(trans);
            self.execute_effects(effects, binary);
        }

        // If now in Crashed state, evaluate recovery
        if matches!(driver.state, DriverState::Crashed { .. }) {
            let trans = driver.evaluate_crash(now);

            // Log recovery decision
            match &trans.state {
                DriverState::PendingRestart { restart_at } => {
                    let delay_ms = restart_at.saturating_sub(now) / 1_000_000;
                    crate::dlog!("driver_restart_pending binary={} delay_ms={}", binary, delay_ms);
                }
                DriverState::Failed => {
                    crate::derror!("driver_failed binary={}", binary);
                }
                DriverState::Stopped => {
                    crate::dlog!("driver_stopped_no_restart binary={}", binary);
                }
                _ => {}
            }

            let effects = driver.apply(trans);
            self.execute_effects(effects, binary);
        }

        true
    }

    /// Handle bind complete notification
    pub fn handle_bind_complete(&mut self, pid: u32) -> bool {
        let now = syscall::gettime() as u64;

        let idx = match self.find_by_pid(pid) {
            Some(i) => i,
            None => return false,
        };

        let driver = match self.drivers[idx].as_mut() {
            Some(d) => d,
            None => return false,
        };

        let binary = driver.config.binary;

        if let Some(trans) = driver.handle(Event::BindComplete { pid }, now) {
            crate::dlog!("driver_bound binary={} pid={}", binary, pid);
            let effects = driver.apply(trans);
            self.execute_effects(effects, binary);
            return true;
        }

        false
    }

    /// Handle timer expiry - check all drivers for expired deadlines
    pub fn handle_timer(&mut self) {
        let now = syscall::gettime() as u64;

        for i in 0..self.count {
            let driver = match self.drivers[i].as_mut() {
                Some(d) => d,
                None => continue,
            };

            // Check if this driver has an expired deadline
            if let Some(deadline) = driver.state.deadline() {
                if now >= deadline {
                    let binary = driver.config.binary;

                    // Handle deadline expiry
                    if let Some(trans) = driver.handle(Event::DeadlineExpired, now) {
                        let effects = driver.apply(trans);

                        // Execute effects, handle spawn if needed
                        if let Some(SpawnResult::Success { pid }) =
                            self.execute_effects(effects, binary)
                        {
                            // Process spawned event
                            if let Some(trans) = driver.handle(Event::Spawned { pid }, now) {
                                let effects = driver.apply(trans);
                                self.execute_effects(effects, binary);
                            }
                        }
                    }
                }
            }
        }

        // Recalculate and schedule next deadline
        self.schedule_next_deadline();
    }

    /// Handle bus becoming safe
    pub fn handle_bus_safe(&mut self, bus_type: u8, bus_index: u8) {
        let now = syscall::gettime() as u64;

        for i in 0..self.count {
            let driver = match self.drivers[i].as_mut() {
                Some(d) => d,
                None => continue,
            };

            // Check if waiting for this bus
            if let DriverState::WaitingBusSafe { bus_type: bt, bus_index: bi } = driver.state {
                if bt == bus_type && bi == bus_index {
                    let binary = driver.config.binary;
                    let event = Event::BusSafe { bus_type, bus_index };

                    if let Some(trans) = driver.handle(event, now) {
                        let effects = driver.apply(trans);

                        if let Some(SpawnResult::Success { pid }) =
                            self.execute_effects(effects, binary)
                        {
                            if let Some(trans) = driver.handle(Event::Spawned { pid }, now) {
                                let effects = driver.apply(trans);
                                self.execute_effects(effects, binary);
                            }
                        }
                    }
                }
            }
        }

        self.schedule_next_deadline();
    }

    /// Execute effects and return spawn result if any
    fn execute_effects(&mut self, effects: Effects, context: &str) -> Option<SpawnResult> {
        use crate::effects::EffectExecutor;

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

    /// Schedule timer for next deadline across all drivers
    fn schedule_next_deadline(&mut self) {
        let mut earliest: u64 = 0;

        for i in 0..self.count {
            if let Some(ref driver) = self.drivers[i] {
                if let Some(deadline) = driver.state.deadline() {
                    if earliest == 0 || deadline < earliest {
                        earliest = deadline;
                    }
                }
            }
        }

        if earliest > 0 {
            let now = syscall::gettime() as u64;
            let delay = earliest.saturating_sub(now);
            if delay > 0 {
                syscall::timer_set(delay);
            }
        }
    }

    /// Get number of registered drivers
    pub fn count(&self) -> usize {
        self.count
    }

    /// Get driver state for a specific driver
    pub fn state(&self, idx: usize) -> Option<&DriverState> {
        self.drivers.get(idx)?.as_ref().map(|d| &d.state)
    }

    /// Get driver binary name
    pub fn binary(&self, idx: usize) -> Option<&'static str> {
        self.drivers.get(idx)?.as_ref().map(|d| d.config.binary)
    }

    /// Dump state for debugging
    pub fn dump_state(&self) {
        for i in 0..self.count {
            if let Some(ref driver) = self.drivers[i] {
                crate::dlog!("driver_state idx={} binary={} state={} restarts={}",
                    i, driver.config.binary, driver.state.name(), driver.restart_count);
            }
        }
    }
}
