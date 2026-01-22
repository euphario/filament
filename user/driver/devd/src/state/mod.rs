//! State Machine Infrastructure
//!
//! Traits and types for building testable, pure state machines.
//!
//! # Design Principles
//!
//! 1. **Pure transitions**: State machines return new state + effects, don't execute them
//! 2. **Explicit effects**: Timer scheduling, process spawning are returned, not performed
//! 3. **Testable**: No syscalls in state logic, all dependencies injected
//! 4. **Small states**: Each state contains only what's needed to know "what's next"

pub mod driver;

/// A side effect that should be executed after a state transition
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Effect {
    /// Schedule a timer to fire at absolute time (nanoseconds)
    ScheduleDeadline(u64),

    /// Cancel any pending timer
    CancelTimer,

    /// Spawn a process with given binary name
    /// Returns: will need to capture the pid in the next event
    SpawnProcess { binary: &'static str },

    /// Kill a process
    KillProcess { pid: u32 },

    /// Log a message (for debugging/monitoring)
    Log { level: LogLevel, message: &'static str },

    /// Request bus reset at given level
    RequestBusReset { bus_type: u8, bus_index: u8, level: u8 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogLevel {
    Info,
    Warn,
    Error,
}

/// Result of a state transition: new state + any effects to execute
#[derive(Debug)]
pub struct Transition<S> {
    /// The new state
    pub state: S,
    /// Effects to execute (timers, spawns, etc.)
    pub effects: Effects,
}

impl<S> Transition<S> {
    pub fn to(state: S) -> Self {
        Self {
            state,
            effects: Effects::none(),
        }
    }

    pub fn to_with(state: S, effects: Effects) -> Self {
        Self { state, effects }
    }
}

/// Collection of effects to execute
#[derive(Debug, Default)]
pub struct Effects {
    items: [Option<Effect>; 4], // Small fixed array, no alloc
    count: usize,
}

impl Effects {
    pub const fn none() -> Self {
        Self {
            items: [None, None, None, None],
            count: 0,
        }
    }

    pub fn one(effect: Effect) -> Self {
        Self {
            items: [Some(effect), None, None, None],
            count: 1,
        }
    }

    pub fn push(&mut self, effect: Effect) {
        if self.count < 4 {
            self.items[self.count] = Some(effect);
            self.count += 1;
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &Effect> {
        self.items[..self.count].iter().filter_map(|e| e.as_ref())
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

/// Events that can trigger state transitions
#[derive(Debug, Clone, Copy)]
pub enum Event {
    /// Start the driver/service
    Start,

    /// Process was spawned successfully
    Spawned { pid: u32 },

    /// Process reported bind complete
    BindComplete { pid: u32 },

    /// Process exited
    ProcessExit { pid: u32, code: i32 },

    /// Timer/deadline expired
    DeadlineExpired,

    /// Bus became safe
    BusSafe { bus_type: u8, bus_index: u8 },

    /// Bus reset completed
    ResetComplete { success: bool },

    /// Manual stop requested
    Stop,
}

/// Trait for state machines - defines pure transition logic
pub trait StateMachine: Sized {
    /// The state type
    type State: Clone;

    /// Get current state
    fn state(&self) -> &Self::State;

    /// Handle an event, returning new state and effects
    /// This is a PURE function - no side effects
    fn handle(&self, event: Event) -> Option<Transition<Self::State>>;

    /// Apply a transition (mutates self)
    fn apply(&mut self, transition: Transition<Self::State>);
}

/// Time provider trait - injectable for testing
pub trait Clock {
    fn now_ns(&self) -> u64;
}

/// System clock implementation
pub struct SystemClock;

impl Clock for SystemClock {
    fn now_ns(&self) -> u64 {
        userlib::syscall::gettime() as u64
    }
}

/// Mock clock for testing
#[cfg(test)]
pub struct MockClock {
    pub time: core::cell::Cell<u64>,
}

#[cfg(test)]
impl MockClock {
    pub fn new(time: u64) -> Self {
        Self { time: core::cell::Cell::new(time) }
    }

    pub fn advance(&self, ns: u64) {
        self.time.set(self.time.get() + ns);
    }
}

#[cfg(test)]
impl Clock for MockClock {
    fn now_ns(&self) -> u64 {
        self.time.get()
    }
}
