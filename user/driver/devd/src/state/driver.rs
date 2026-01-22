//! Driver State Machine
//!
//! Pure state machine for driver lifecycle. No syscalls, fully testable.
//!
//! # States
//!
//! ```text
//! Stopped ─────> Starting ─────> Binding ─────> Bound
//!    ^              │               │             │
//!    │              v               v             v
//!    │           Crashed <─────────────────────────
//!    │              │
//!    │              v
//!    │        PendingRestart ───> WaitingBusSafe ───> Resetting
//!    │              │                                     │
//!    └──────────────┴─────────────────────────────────────┘
//!                                    │
//!                                    v
//!                                 Failed
//! ```

use super::{Effect, Effects, Event, LogLevel, Transition};

/// Driver state with embedded temporal/contextual data
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DriverState {
    /// Not running, not scheduled
    Stopped,

    /// Spawn requested, waiting for process
    Starting { deadline: u64 },

    /// Process running, waiting for bind_complete
    Binding { pid: u32, deadline: u64 },

    /// Fully operational
    Bound { pid: u32 },

    /// Process exited with error
    Crashed { code: i32 },

    /// Waiting for restart timer
    PendingRestart { restart_at: u64 },

    /// Waiting for bus to become safe
    WaitingBusSafe { bus_type: u8, bus_index: u8 },

    /// Bus reset in progress
    Resetting { level: u8, deadline: u64 },

    /// Permanently failed
    Failed,
}

impl DriverState {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Stopped => "Stopped",
            Self::Starting { .. } => "Starting",
            Self::Binding { .. } => "Binding",
            Self::Bound { .. } => "Bound",
            Self::Crashed { .. } => "Crashed",
            Self::PendingRestart { .. } => "PendingRestart",
            Self::WaitingBusSafe { .. } => "WaitingBusSafe",
            Self::Resetting { .. } => "Resetting",
            Self::Failed => "Failed",
        }
    }

    pub fn deadline(&self) -> Option<u64> {
        match self {
            Self::Starting { deadline }
            | Self::Binding { deadline, .. }
            | Self::Resetting { deadline, .. } => Some(*deadline),
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

/// Configuration for a driver
#[derive(Debug, Clone)]
pub struct DriverConfig {
    pub binary: &'static str,
    pub has_device: bool,
    pub bus_type: Option<u8>,
    pub bus_index: Option<u8>,
    pub auto_restart: bool,
    pub start_timeout_ns: u64,
    pub bind_timeout_ns: u64,
    pub restart_delay_ns: u64,
}

impl DriverConfig {
    pub const DEFAULT_START_TIMEOUT: u64 = 5_000_000_000;  // 5s
    pub const DEFAULT_BIND_TIMEOUT: u64 = 10_000_000_000;  // 10s
    pub const DEFAULT_RESTART_DELAY: u64 = 1_000_000_000;  // 1s
}

/// Driver state machine
#[derive(Debug)]
pub struct Driver {
    pub config: DriverConfig,
    pub state: DriverState,
    pub restart_count: u32,
}

impl Driver {
    pub fn new(config: DriverConfig) -> Self {
        Self {
            config,
            state: DriverState::Stopped,
            restart_count: 0,
        }
    }

    /// Create a driver that adopts an already-running process
    /// Used when a process was spawned externally (e.g., by ServiceManager)
    pub fn new_adopted(config: DriverConfig, pid: u32) -> Self {
        Self {
            config,
            state: DriverState::Bound { pid },
            restart_count: 0,
        }
    }

    /// Handle an event, returning the transition (pure function)
    pub fn handle(&self, event: Event, now: u64) -> Option<Transition<DriverState>> {
        match (&self.state, event) {
            // === Start ===
            (DriverState::Stopped, Event::Start) => {
                let deadline = now + self.config.start_timeout_ns;
                Some(Transition::to_with(
                    DriverState::Starting { deadline },
                    Effects::one(Effect::SpawnProcess { binary: self.config.binary }),
                ))
            }

            // === Spawn success ===
            (DriverState::Starting { .. }, Event::Spawned { pid }) => {
                let deadline = now + self.config.bind_timeout_ns;
                let mut effects = Effects::none();
                effects.push(Effect::ScheduleDeadline(deadline));
                Some(Transition::to_with(
                    DriverState::Binding { pid, deadline },
                    effects,
                ))
            }

            // === Bind complete ===
            (DriverState::Binding { pid, .. }, Event::BindComplete { pid: event_pid })
                if *pid == event_pid =>
            {
                let mut effects = Effects::none();
                effects.push(Effect::CancelTimer);
                effects.push(Effect::Log {
                    level: LogLevel::Info,
                    message: "driver_bound",
                });
                Some(Transition::to_with(DriverState::Bound { pid: *pid }, effects))
            }

            // === Clean exit ===
            (DriverState::Bound { pid }, Event::ProcessExit { pid: exit_pid, code: 0 })
                if *pid == exit_pid =>
            {
                Some(Transition::to(DriverState::Stopped))
            }

            // === Crash from any running state ===
            (DriverState::Starting { .. }, Event::DeadlineExpired) |
            (DriverState::Starting { .. }, Event::ProcessExit { .. }) => {
                Some(Transition::to(DriverState::Crashed { code: -1 }))
            }

            (DriverState::Binding { pid, .. }, Event::DeadlineExpired) => {
                let mut effects = Effects::none();
                effects.push(Effect::KillProcess { pid: *pid });
                Some(Transition::to_with(DriverState::Crashed { code: -2 }, effects))
            }

            (DriverState::Binding { pid, .. }, Event::ProcessExit { pid: exit_pid, code })
                if *pid == exit_pid =>
            {
                Some(Transition::to(DriverState::Crashed { code }))
            }

            (DriverState::Bound { pid }, Event::ProcessExit { pid: exit_pid, code })
                if *pid == exit_pid && code != 0 =>
            {
                Some(Transition::to(DriverState::Crashed { code }))
            }

            // === Recovery evaluation (called after entering Crashed) ===
            // This needs external input about restart policy

            // === Manual stop ===
            (state, Event::Stop) if state.pid().is_some() => {
                let pid = state.pid().unwrap();
                Some(Transition::to_with(
                    DriverState::Stopped,
                    Effects::one(Effect::KillProcess { pid }),
                ))
            }

            (_, Event::Stop) => Some(Transition::to(DriverState::Stopped)),

            // === Restart flow ===
            (DriverState::PendingRestart { restart_at }, Event::DeadlineExpired)
                if now >= *restart_at =>
            {
                if self.config.has_device {
                    // Need to wait for bus
                    Some(Transition::to(DriverState::WaitingBusSafe {
                        bus_type: self.config.bus_type.unwrap_or(0),
                        bus_index: self.config.bus_index.unwrap_or(0),
                    }))
                } else {
                    // Service - restart immediately
                    let deadline = now + self.config.start_timeout_ns;
                    Some(Transition::to_with(
                        DriverState::Starting { deadline },
                        Effects::one(Effect::SpawnProcess { binary: self.config.binary }),
                    ))
                }
            }

            (DriverState::WaitingBusSafe { bus_type, bus_index }, Event::BusSafe { bus_type: bt, bus_index: bi })
                if *bus_type == bt && *bus_index == bi =>
            {
                let deadline = now + self.config.start_timeout_ns;
                Some(Transition::to_with(
                    DriverState::Starting { deadline },
                    Effects::one(Effect::SpawnProcess { binary: self.config.binary }),
                ))
            }

            (DriverState::Resetting { .. }, Event::ResetComplete { success: true }) => {
                let deadline = now + self.config.start_timeout_ns;
                Some(Transition::to_with(
                    DriverState::Starting { deadline },
                    Effects::one(Effect::SpawnProcess { binary: self.config.binary }),
                ))
            }

            (DriverState::Resetting { .. }, Event::ResetComplete { success: false }) => {
                Some(Transition::to(DriverState::Failed))
            }

            (DriverState::Resetting { .. }, Event::DeadlineExpired) => {
                Some(Transition::to(DriverState::Failed))
            }

            // No transition for this event in this state
            _ => None,
        }
    }

    /// Evaluate crash recovery - returns transition to recovery state
    pub fn evaluate_crash(&self, now: u64) -> Transition<DriverState> {
        if !self.config.auto_restart {
            return Transition::to(DriverState::Stopped);
        }

        // Simple policy: restart after delay, up to 5 times
        if self.restart_count >= 5 {
            return Transition::to(DriverState::Failed);
        }

        let restart_at = now + self.config.restart_delay_ns;
        Transition::to_with(
            DriverState::PendingRestart { restart_at },
            Effects::one(Effect::ScheduleDeadline(restart_at)),
        )
    }

    /// Apply a transition
    pub fn apply(&mut self, transition: Transition<DriverState>) -> Effects {
        // Track restarts
        if matches!(transition.state, DriverState::Starting { .. })
            && matches!(
                self.state,
                DriverState::PendingRestart { .. }
                    | DriverState::WaitingBusSafe { .. }
                    | DriverState::Resetting { .. }
            )
        {
            self.restart_count += 1;
        }

        // Reset counter on successful bind
        if matches!(transition.state, DriverState::Bound { .. }) {
            self.restart_count = 0;
        }

        self.state = transition.state;
        transition.effects
    }
}

// Tests would use std for Vec - run with `cargo test` in std mode
// The state machine logic is pure and testable without syscalls
