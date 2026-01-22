//! Process State Machine
//!
//! Pure state machine for process lifecycle. No syscalls, fully testable.
//!
//! # States
//!
//! ```text
//! Stopped ─────> WaitingDependency ─────> Starting ─────> Binding ─────> Bound
//!    ^                   ^                    │              │             │
//!    │                   │                    v              v             v
//!    │                   │                 Crashed <─────────────────────────
//!    │                   │                    │
//!    │                   │                    v
//!    │                   └─────────── PendingRestart
//!    │                                        │
//!    └────────────────────────────────────────┴────────> Failed
//! ```

use crate::state::{Effect, Effects};

/// Process lifecycle state
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProcessState {
    /// Not running, not scheduled
    Stopped,

    /// Waiting for dependency port to be available
    WaitingDependency,

    /// Spawning process, waiting for spawn result
    Starting { deadline: u64 },

    /// Process running, waiting for it to register its ports (if any)
    Binding { pid: u32, deadline: u64 },

    /// Fully operational
    Bound { pid: u32 },

    /// Process exited with error
    Crashed { code: i32 },

    /// Waiting for restart timer
    PendingRestart { restart_at: u64 },

    /// Permanently failed
    Failed,
}

impl ProcessState {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Stopped => "Stopped",
            Self::WaitingDependency => "WaitingDependency",
            Self::Starting { .. } => "Starting",
            Self::Binding { .. } => "Binding",
            Self::Bound { .. } => "Bound",
            Self::Crashed { .. } => "Crashed",
            Self::PendingRestart { .. } => "PendingRestart",
            Self::Failed => "Failed",
        }
    }

    pub fn deadline(&self) -> Option<u64> {
        match self {
            Self::Starting { deadline } |
            Self::Binding { deadline, .. } => Some(*deadline),
            Self::PendingRestart { restart_at } => Some(*restart_at),
            _ => None,
        }
    }

    pub fn pid(&self) -> Option<u32> {
        match self {
            Self::Binding { pid, .. } | Self::Bound { pid } => Some(*pid),
            _ => None,
        }
    }
}

/// Configuration for a supervised process
#[derive(Debug, Clone)]
pub struct ProcessConfig {
    /// Binary name to execute
    pub binary: &'static str,

    /// Port this process depends on (None = no dependency)
    pub depends_on: Option<&'static str>,

    /// Ports this process provides (empty strings = end of list)
    pub provides_ports: [&'static str; 4],

    /// Auto-start when dependency is satisfied
    pub auto_start: bool,

    /// Auto-restart on crash
    pub auto_restart: bool,

    /// Timeout for spawn to succeed (ns)
    pub start_timeout_ns: u64,

    /// Timeout for process to register its ports (ns)
    pub ready_timeout_ns: u64,

    /// Delay before restart after crash (ns)
    pub restart_delay_ns: u64,
}

impl ProcessConfig {
    pub const DEFAULT_START_TIMEOUT: u64 = 5_000_000_000;   // 5s
    pub const DEFAULT_READY_TIMEOUT: u64 = 30_000_000_000;  // 30s
    pub const DEFAULT_RESTART_DELAY: u64 = 1_000_000_000;   // 1s

    /// Check if this process provides any ports
    pub fn has_ports(&self) -> bool {
        !self.provides_ports[0].is_empty()
    }
}

/// Events that trigger state transitions
#[derive(Debug, Clone, Copy)]
pub enum Event {
    /// Start requested
    Start,

    /// Dependency port is not available
    DependencyUnavailable,

    /// Dependency port became available
    DependencyAvailable,

    /// Process was spawned successfully
    Spawned { pid: u32 },

    /// Process registered its port(s)
    PortRegistered,

    /// Process exited
    ProcessExit { pid: u32, code: i32 },

    /// Timer/deadline expired
    DeadlineExpired,

    /// Stop requested
    Stop,
}

/// Result of a state transition
#[derive(Debug)]
pub struct Transition {
    pub state: ProcessState,
    pub effects: Effects,
}

impl Transition {
    pub fn to(state: ProcessState) -> Self {
        Self { state, effects: Effects::none() }
    }

    pub fn to_with(state: ProcessState, effects: Effects) -> Self {
        Self { state, effects }
    }
}

/// Process state machine
#[derive(Debug)]
pub struct Process {
    pub config: ProcessConfig,
    pub state: ProcessState,
    pub restart_count: u32,
}

impl Process {
    pub fn new(config: ProcessConfig) -> Self {
        Self {
            config,
            state: ProcessState::Stopped,
            restart_count: 0,
        }
    }

    /// Create a process that adopts an already-running pid
    pub fn new_adopted(config: ProcessConfig, pid: u32) -> Self {
        Self {
            config,
            state: ProcessState::Bound { pid },
            restart_count: 0,
        }
    }

    /// Handle an event, returning the transition (pure function)
    pub fn handle(&self, event: Event, now: u64) -> Option<Transition> {
        match (&self.state, event) {
            // === Start ===
            (ProcessState::Stopped, Event::Start) => {
                if self.config.depends_on.is_some() {
                    // Has dependency - caller should check if satisfied
                    // For now, assume we'll get DependencyAvailable or DependencyUnavailable
                    Some(Transition::to(ProcessState::WaitingDependency))
                } else {
                    // No dependency - spawn immediately
                    let deadline = now + self.config.start_timeout_ns;
                    Some(Transition::to_with(
                        ProcessState::Starting { deadline },
                        Effects::one(Effect::SpawnProcess { binary: self.config.binary }),
                    ))
                }
            }

            // === Dependency handling ===
            (ProcessState::Stopped, Event::DependencyAvailable) |
            (ProcessState::WaitingDependency, Event::DependencyAvailable) => {
                let deadline = now + self.config.start_timeout_ns;
                Some(Transition::to_with(
                    ProcessState::Starting { deadline },
                    Effects::one(Effect::SpawnProcess { binary: self.config.binary }),
                ))
            }

            (ProcessState::Stopped, Event::DependencyUnavailable) |
            (ProcessState::WaitingDependency, Event::DependencyUnavailable) => {
                Some(Transition::to(ProcessState::WaitingDependency))
            }

            // === Spawn success ===
            (ProcessState::Starting { .. }, Event::Spawned { pid }) => {
                if self.config.has_ports() {
                    // Wait for port registration
                    let deadline = now + self.config.ready_timeout_ns;
                    let mut effects = Effects::none();
                    effects.push(Effect::ScheduleDeadline(deadline));
                    Some(Transition::to_with(
                        ProcessState::Binding { pid, deadline },
                        effects,
                    ))
                } else {
                    // No ports to wait for - immediately bound
                    Some(Transition::to(ProcessState::Bound { pid }))
                }
            }

            // === Port registered (binding complete) ===
            (ProcessState::Binding { pid, .. }, Event::PortRegistered) => {
                let mut effects = Effects::none();
                effects.push(Effect::CancelTimer);
                Some(Transition::to_with(ProcessState::Bound { pid: *pid }, effects))
            }

            // === Clean exit ===
            (ProcessState::Bound { pid }, Event::ProcessExit { pid: exit_pid, code: 0 })
                if *pid == exit_pid =>
            {
                Some(Transition::to(ProcessState::Stopped))
            }

            // === Crash from running states ===
            (ProcessState::Starting { .. }, Event::DeadlineExpired) |
            (ProcessState::Starting { .. }, Event::ProcessExit { .. }) => {
                Some(Transition::to(ProcessState::Crashed { code: -1 }))
            }

            (ProcessState::Binding { pid, .. }, Event::DeadlineExpired) => {
                let mut effects = Effects::none();
                effects.push(Effect::KillProcess { pid: *pid });
                Some(Transition::to_with(ProcessState::Crashed { code: -2 }, effects))
            }

            (ProcessState::Binding { pid, .. }, Event::ProcessExit { pid: exit_pid, code })
                if *pid == exit_pid =>
            {
                Some(Transition::to(ProcessState::Crashed { code }))
            }

            (ProcessState::Bound { pid }, Event::ProcessExit { pid: exit_pid, code })
                if *pid == exit_pid && code != 0 =>
            {
                Some(Transition::to(ProcessState::Crashed { code }))
            }

            // === Restart flow ===
            (ProcessState::PendingRestart { restart_at }, Event::DeadlineExpired)
                if now >= *restart_at =>
            {
                // Caller should check dependency before spawning
                let deadline = now + self.config.start_timeout_ns;
                Some(Transition::to_with(
                    ProcessState::Starting { deadline },
                    Effects::one(Effect::SpawnProcess { binary: self.config.binary }),
                ))
            }

            (ProcessState::PendingRestart { .. }, Event::DependencyUnavailable) => {
                Some(Transition::to(ProcessState::WaitingDependency))
            }

            // === Manual stop ===
            (state, Event::Stop) if state.pid().is_some() => {
                let pid = state.pid().unwrap();
                Some(Transition::to_with(
                    ProcessState::Stopped,
                    Effects::one(Effect::KillProcess { pid }),
                ))
            }

            (_, Event::Stop) => Some(Transition::to(ProcessState::Stopped)),

            // No transition for this event in this state
            _ => None,
        }
    }

    /// Evaluate crash recovery - returns transition to recovery state
    pub fn evaluate_crash(&self, now: u64) -> Transition {
        if !self.config.auto_restart {
            return Transition::to(ProcessState::Stopped);
        }

        // Simple policy: restart after delay, up to 5 times
        if self.restart_count >= 5 {
            return Transition::to(ProcessState::Failed);
        }

        let restart_at = now + self.config.restart_delay_ns;
        Transition::to_with(
            ProcessState::PendingRestart { restart_at },
            Effects::one(Effect::ScheduleDeadline(restart_at)),
        )
    }

    /// Apply a transition
    pub fn apply(&mut self, transition: Transition) -> Effects {
        // Track restarts
        if matches!(transition.state, ProcessState::Starting { .. })
            && matches!(
                self.state,
                ProcessState::PendingRestart { .. } | ProcessState::WaitingDependency
            )
        {
            self.restart_count += 1;
        }

        // Reset counter on successful bind
        if matches!(transition.state, ProcessState::Bound { .. }) {
            self.restart_count = 0;
        }

        self.state = transition.state;
        transition.effects
    }
}
