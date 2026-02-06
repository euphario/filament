//! Task State Machine
//!
//! This module defines the task lifecycle states and enforces valid transitions.
//!
//! # State Diagram
//!
//! ```text
//!                          spawn()
//!                             │
//!                             ▼
//!     ┌─────────────────── Ready ◄──────────────────────┐
//!     │                      │                          │
//!     │                      │ schedule()               │
//!     │                      ▼                          │
//!     │                   Running ──────────────────────┤
//!     │                    │ │ │                        │
//!     │        yield/      │ │ │  exit()               │
//!     │        preempt     │ │ └──────┐                │
//!     │           │        │ │        │                │
//!     │           │  block │ │        ▼                │
//!     │           │        │ │    Exiting              │
//!     │           │        ▼ │        │                │
//!     │           │   ┌────────┐      │ grace timeout  │
//!     │           │   │Sleeping│      ▼                │
//!     │           │   │   or   │   Dying               │
//!     │           │   │Waiting │      │                │
//!     │           │   └────────┘      │ cleanup done   │
//!     │           │        │          ▼                │
//!     │           │        │       Dead ───────────────┘
//!     │           │        │       (slot reused)
//!     │           │        │
//!     │           │  wake  │
//!     └───────────┴────────┘
//! ```
//!
//! # Key Invariants
//!
//! 1. Only Running tasks can transition to blocked states (Sleeping/Waiting)
//! 2. Only blocked tasks can be woken (→ Ready)
//! 3. Terminated tasks (Exiting/Dying/Dead) cannot become runnable again
//! 4. Sleeping has no deadline; Waiting always has a deadline
//! 5. State transitions only happen through validated methods

/// Why a task is sleeping (event-loop style, no deadline)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepReason {
    /// Waiting in event loop (kevent_wait, mux read)
    EventLoop,
    /// Waiting for hardware IRQ (no timeout)
    Irq,
    /// Waiting for channel message (blocking recv)
    ChannelRecv,
    /// Waiting for port accept (blocking accept)
    PortAccept,
}

/// Why a task is waiting (request-response style, has deadline)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitReason {
    /// Waiting for IPC call reply
    IpcCall,
    /// Waiting for child process to exit
    Child,
    /// Waiting for shared memory notification
    ShmemNotify { shmem_id: u32 },
    /// Waiting for timed event
    TimedEvent,
    /// Waiting for timer expiry
    Timer,
    /// Throttled due to syscall storm - blocked until deadline
    Throttled,
}

/// Why a task is being forcibly evicted
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvictionReason {
    /// Invalid state transition attempted (state machine violation)
    InvalidStateTransition,
    /// Internal state corruption detected
    StateCorruption,
    /// Syscall handler panicked or detected corruption
    SyscallPanic,
    /// Task failed liveness check (unresponsive too long)
    LivenessTimeout,
    /// Task exhausted a kernel resource (handles, memory, etc.)
    ResourceExhaustion,
}

/// Task lifecycle state
///
/// This is the SINGLE SOURCE OF TRUTH for task state.
/// All state-dependent data (exit code, deadline, etc.) is embedded in the state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskState {
    /// Task is ready to run (in run queue)
    Ready,

    /// Task is currently executing on a specific CPU
    /// The cpu field identifies which CPU owns this task.
    Running { cpu: u32 },

    /// Task is sleeping - waiting for any event, no deadline
    /// Used for event-loop style processes (devd, servers)
    Sleeping { reason: SleepReason },

    /// Task is waiting for specific response, has deadline
    /// If deadline passes, something is stuck (liveness concern)
    Waiting { reason: WaitReason, deadline: u64 },

    /// Task has exited, pending Phase 1 cleanup (notifications)
    Exiting { code: i32 },

    /// Task in grace period for Phase 2 cleanup (resource release)
    Dying { code: i32, until: u64 },

    /// Task being forcibly evicted due to state corruption or violation
    /// Kernel-initiated termination - goes directly to Dead (no grace period)
    Evicting { reason: EvictionReason },

    /// Task fully cleaned up, slot can be reused
    Dead,
}

/// Error returned when an invalid state transition is attempted
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidTransition {
    pub from: &'static str,
    pub to: &'static str,
}

impl TaskState {
    // ========================================================================
    // State queries
    // ========================================================================

    /// Task can be scheduled (Ready or Running on current CPU)
    /// Note: For SMP safety, selection should also check kernel_stack_owner.
    pub fn is_runnable(&self) -> bool {
        matches!(self, TaskState::Ready | TaskState::Running { .. })
    }

    /// Check if task is currently running
    pub fn is_running(&self) -> bool {
        matches!(self, TaskState::Running { .. })
    }

    /// Get the CPU this task is running on, if any
    pub fn running_cpu(&self) -> Option<u32> {
        match self {
            TaskState::Running { cpu } => Some(*cpu),
            _ => None,
        }
    }

    /// Task is blocked waiting for something
    pub fn is_blocked(&self) -> bool {
        matches!(self, TaskState::Sleeping { .. } | TaskState::Waiting { .. })
    }

    /// Task is sleeping (event loop, healthy idle)
    pub fn is_sleeping(&self) -> bool {
        matches!(self, TaskState::Sleeping { .. })
    }

    /// Task is waiting (request pending, has deadline)
    pub fn is_waiting(&self) -> bool {
        matches!(self, TaskState::Waiting { .. })
    }

    /// Task has terminated (cannot run again)
    pub fn is_terminated(&self) -> bool {
        matches!(
            self,
            TaskState::Exiting { .. } | TaskState::Dying { .. } | TaskState::Evicting { .. } | TaskState::Dead
        )
    }

    /// Task is being evicted (kernel-initiated forced termination)
    pub fn is_evicting(&self) -> bool {
        matches!(self, TaskState::Evicting { .. })
    }

    /// Get deadline if this state has one
    pub fn deadline(&self) -> Option<u64> {
        match self {
            TaskState::Waiting { deadline, .. } => Some(*deadline),
            TaskState::Dying { until, .. } => Some(*until),
            _ => None,
        }
    }

    /// Get exit code if task has exited
    pub fn exit_code(&self) -> Option<i32> {
        match self {
            TaskState::Exiting { code } | TaskState::Dying { code, .. } => Some(*code),
            _ => None,
        }
    }

    /// Get sleep reason if sleeping
    pub fn sleep_reason(&self) -> Option<SleepReason> {
        match self {
            TaskState::Sleeping { reason } => Some(*reason),
            _ => None,
        }
    }

    /// Get wait reason if waiting
    pub fn wait_reason(&self) -> Option<WaitReason> {
        match self {
            TaskState::Waiting { reason, .. } => Some(*reason),
            _ => None,
        }
    }

    /// Get eviction reason if evicting
    pub fn eviction_reason(&self) -> Option<EvictionReason> {
        match self {
            TaskState::Evicting { reason } => Some(*reason),
            _ => None,
        }
    }

    /// State name for debugging/logging
    pub fn name(&self) -> &'static str {
        match self {
            TaskState::Ready => "Ready",
            TaskState::Running { .. } => "Running",
            TaskState::Sleeping { .. } => "Sleeping",
            TaskState::Waiting { .. } => "Waiting",
            TaskState::Exiting { .. } => "Exiting",
            TaskState::Dying { .. } => "Dying",
            TaskState::Evicting { .. } => "Evicting",
            TaskState::Dead => "Dead",
        }
    }

    /// Numeric code for ProcessInfo
    pub fn state_code(&self) -> u8 {
        match self {
            TaskState::Ready => 0,
            TaskState::Running { .. } => 1,
            TaskState::Sleeping { .. } => 2,
            TaskState::Waiting { .. } => 3,
            TaskState::Exiting { .. } => 4,
            TaskState::Dying { .. } => 5,
            TaskState::Dead => 6,
            TaskState::Evicting { .. } => 7,
        }
    }

    // ========================================================================
    // Validated transitions
    // ========================================================================

    /// Check if transition to new state is valid
    pub fn can_transition_to(&self, to: &TaskState) -> bool {
        use TaskState::*;

        match (self, to) {
            // Ready can only go to Running (via scheduler)
            (Ready, Running { .. }) => true,

            // Running can go to:
            // - Ready (yield/preempt)
            // - Sleeping (block without deadline)
            // - Waiting (block with deadline)
            // - Exiting (exit/kill)
            (Running { .. }, Ready) => true,
            (Running { .. }, Sleeping { .. }) => true,
            (Running { .. }, Waiting { .. }) => true,
            (Running { .. }, Exiting { .. }) => true,

            // Sleeping/Waiting can go to:
            // - Ready (wake)
            // - Exiting (killed while blocked)
            (Sleeping { .. }, Ready) => true,
            (Sleeping { .. }, Exiting { .. }) => true,
            (Waiting { .. }, Ready) => true,
            (Waiting { .. }, Exiting { .. }) => true,

            // Exiting goes to Dying (grace period starts)
            (Exiting { .. }, Dying { .. }) => true,
            // Or directly to Dead (no grace needed)
            (Exiting { .. }, Dead) => true,

            // Dying goes to Dead (cleanup complete)
            (Dying { .. }, Dead) => true,

            // Dead goes to Ready (slot reused for new task)
            (Dead, Ready) => true,

            // Evicting: kernel-initiated forced termination
            // Any non-terminal state can be evicted
            (Ready, Evicting { .. }) => true,
            (Running { .. }, Evicting { .. }) => true,
            (Sleeping { .. }, Evicting { .. }) => true,
            (Waiting { .. }, Evicting { .. }) => true,
            // Evicting goes directly to Dead (no grace period)
            (Evicting { .. }, Dead) => true,

            // All other transitions are invalid
            _ => false,
        }
    }

    /// Attempt state transition, returns error if invalid
    ///
    /// This is the ONLY way states should be changed (enforces invariants)
    pub fn transition(&mut self, to: TaskState) -> Result<TaskState, InvalidTransition> {
        if self.can_transition_to(&to) {
            let old = *self;
            *self = to;
            Ok(old)
        } else {
            Err(InvalidTransition {
                from: self.name(),
                to: to.name(),
            })
        }
    }

    // ========================================================================
    // Convenience transition methods
    // ========================================================================

    /// Running → Ready (yield or preemption)
    pub fn yield_cpu(&mut self) -> Result<(), InvalidTransition> {
        self.transition(TaskState::Ready).map(|_| ())
    }

    /// Ready → Running (scheduled on specific CPU)
    pub fn schedule(&mut self, cpu: u32) -> Result<(), InvalidTransition> {
        self.transition(TaskState::Running { cpu }).map(|_| ())
    }

    /// Running → Sleeping (block for event, no deadline)
    pub fn sleep(&mut self, reason: SleepReason) -> Result<(), InvalidTransition> {
        self.transition(TaskState::Sleeping { reason }).map(|_| ())
    }

    /// Running → Waiting (block for response, with deadline)
    pub fn wait(&mut self, reason: WaitReason, deadline: u64) -> Result<(), InvalidTransition> {
        self.transition(TaskState::Waiting { reason, deadline })
            .map(|_| ())
    }

    /// Sleeping/Waiting → Ready (event arrived)
    pub fn wake(&mut self) -> Result<(), InvalidTransition> {
        self.transition(TaskState::Ready).map(|_| ())
    }

    /// Any runnable/blocked → Exiting (voluntary exit or killed)
    pub fn exit(&mut self, code: i32) -> Result<(), InvalidTransition> {
        // Can exit from Running, Sleeping, or Waiting
        if matches!(
            self,
            TaskState::Running { .. } | TaskState::Sleeping { .. } | TaskState::Waiting { .. }
        ) {
            *self = TaskState::Exiting { code };
            Ok(())
        } else if matches!(self, TaskState::Ready) {
            // Ready task being killed before it got to run
            *self = TaskState::Exiting { code };
            Ok(())
        } else {
            Err(InvalidTransition {
                from: self.name(),
                to: "Exiting",
            })
        }
    }

    /// Exiting → Dying (start grace period)
    pub fn start_dying(&mut self, until: u64) -> Result<i32, InvalidTransition> {
        if let TaskState::Exiting { code } = *self {
            *self = TaskState::Dying { code, until };
            Ok(code)
        } else {
            Err(InvalidTransition {
                from: self.name(),
                to: "Dying",
            })
        }
    }

    /// Exiting/Dying/Evicting → Dead (cleanup complete)
    pub fn finalize(&mut self) -> Result<i32, InvalidTransition> {
        match *self {
            TaskState::Exiting { code } | TaskState::Dying { code, .. } => {
                *self = TaskState::Dead;
                Ok(code)
            }
            TaskState::Evicting { .. } => {
                // Evicted tasks get exit code -1 (abnormal termination)
                *self = TaskState::Dead;
                Ok(-1)
            }
            _ => Err(InvalidTransition {
                from: self.name(),
                to: "Dead",
            }),
        }
    }

    /// Any non-terminal → Evicting (kernel-initiated forced termination)
    pub fn evict(&mut self, reason: EvictionReason) -> Result<(), InvalidTransition> {
        self.transition(TaskState::Evicting { reason }).map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_lifecycle() {
        let mut state = TaskState::Ready;

        // Ready → Running
        assert!(state.schedule(0).is_ok());
        assert_eq!(state, TaskState::Running { cpu: 0 });

        // Running → Sleeping
        assert!(state.sleep(SleepReason::EventLoop).is_ok());
        assert!(state.is_sleeping());

        // Sleeping → Ready (wake)
        assert!(state.wake().is_ok());
        assert_eq!(state, TaskState::Ready);

        // Ready → Running → Exiting
        assert!(state.schedule(0).is_ok());
        assert!(state.exit(0).is_ok());
        assert!(state.is_terminated());

        // Exiting → Dying
        assert!(state.start_dying(1000).is_ok());

        // Dying → Dead
        assert!(state.finalize().is_ok());
        assert_eq!(state, TaskState::Dead);
    }

    #[test]
    fn test_invalid_transitions() {
        let mut state = TaskState::Ready;

        // Ready cannot go directly to Sleeping
        assert!(state.sleep(SleepReason::EventLoop).is_err());

        // Ready cannot wake (not blocked)
        assert!(state.wake().is_err());

        // Dead cannot exit
        state = TaskState::Dead;
        assert!(state.exit(1).is_err());
    }

    #[test]
    fn test_kill_while_blocked() {
        let mut state = TaskState::Running { cpu: 0 };
        state.sleep(SleepReason::Irq).unwrap();

        // Can kill a sleeping task
        assert!(state.exit(-9).is_ok());
        assert_eq!(state.exit_code(), Some(-9));
    }

    #[test]
    fn test_wait_with_deadline() {
        let mut state = TaskState::Running { cpu: 0 };
        assert!(state.wait(WaitReason::Timer, 12345).is_ok());

        if let TaskState::Waiting { deadline, reason } = state {
            assert_eq!(deadline, 12345);
            assert_eq!(reason, WaitReason::Timer);
        } else {
            panic!("Expected Waiting state");
        }
    }

    #[test]
    fn test_wait_reasons() {
        let mut state = TaskState::Running { cpu: 0 };

        // IpcCall
        assert!(state.wait(WaitReason::IpcCall, 100).is_ok());
        assert!(state.wake().is_ok());

        // Child
        assert!(state.schedule(0).is_ok());
        assert!(state.wait(WaitReason::Child, 200).is_ok());
        assert!(state.wake().is_ok());

        // ShmemNotify
        assert!(state.schedule(0).is_ok());
        assert!(state.wait(WaitReason::ShmemNotify, 300).is_ok());
        assert!(state.wake().is_ok());
    }

    #[test]
    fn test_sleep_reasons() {
        let mut state = TaskState::Running { cpu: 0 };

        // EventLoop
        assert!(state.sleep(SleepReason::EventLoop).is_ok());
        assert!(state.wake().is_ok());

        // Ipc
        assert!(state.schedule(0).is_ok());
        assert!(state.sleep(SleepReason::Irq).is_ok());
        assert!(state.wake().is_ok());
    }

    #[test]
    fn test_is_runnable() {
        let state = TaskState::Ready;
        assert!(state.is_runnable());

        let state = TaskState::Running { cpu: 0 };
        assert!(state.is_runnable());

        let state = TaskState::Sleeping { reason: SleepReason::EventLoop };
        assert!(!state.is_runnable());

        let state = TaskState::Dead;
        assert!(!state.is_runnable());
    }

    #[test]
    fn test_is_blocked() {
        let state = TaskState::Sleeping { reason: SleepReason::Irq };
        assert!(state.is_blocked());

        let state = TaskState::Waiting { reason: WaitReason::Timer, deadline: 100 };
        assert!(state.is_blocked());

        let state = TaskState::Running { cpu: 0 };
        assert!(!state.is_blocked());
    }

    #[test]
    fn test_is_terminated() {
        let state = TaskState::Exiting { code: 0 };
        assert!(state.is_terminated());

        let state = TaskState::Dying { code: -1, until: 0 };
        assert!(state.is_terminated());

        let state = TaskState::Dead;
        assert!(state.is_terminated());

        let state = TaskState::Running { cpu: 0 };
        assert!(!state.is_terminated());
    }

    #[test]
    fn test_exit_code_extraction() {
        let state = TaskState::Exiting { code: 42 };
        assert_eq!(state.exit_code(), Some(42));

        let state = TaskState::Dying { code: -15, until: 0 };
        assert_eq!(state.exit_code(), Some(-15));

        let state = TaskState::Dead;
        assert_eq!(state.exit_code(), None);

        let state = TaskState::Running { cpu: 0 };
        assert_eq!(state.exit_code(), None);
    }

    #[test]
    fn test_yield_cpu() {
        let mut state = TaskState::Running { cpu: 0 };
        assert!(state.yield_cpu().is_ok());
        assert_eq!(state, TaskState::Ready);
    }

    #[test]
    fn test_cannot_yield_from_blocked() {
        let mut state = TaskState::Sleeping { reason: SleepReason::EventLoop };
        assert!(state.yield_cpu().is_err());
    }

    #[test]
    fn test_finalize_returns_code() {
        let mut state = TaskState::Dying { code: 99, until: 1000 };
        let result = state.finalize();
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 99);
        assert_eq!(state, TaskState::Dead);
    }

    #[test]
    fn test_double_exit_fails() {
        let mut state = TaskState::Exiting { code: 1 };
        assert!(state.exit(2).is_err());
    }

    #[test]
    fn test_wake_from_waiting() {
        let mut state = TaskState::Waiting { reason: WaitReason::Timer, deadline: 100 };
        assert!(state.wake().is_ok());
        assert_eq!(state, TaskState::Ready);
    }

    #[test]
    fn test_evict_from_running() {
        let mut state = TaskState::Running { cpu: 0 };
        assert!(state.evict(EvictionReason::InvalidStateTransition).is_ok());
        assert!(state.is_evicting());
        assert!(state.is_terminated());
        assert_eq!(state.eviction_reason(), Some(EvictionReason::InvalidStateTransition));
    }

    #[test]
    fn test_evict_from_sleeping() {
        let mut state = TaskState::Sleeping { reason: SleepReason::EventLoop };
        assert!(state.evict(EvictionReason::LivenessTimeout).is_ok());
        assert!(state.is_evicting());
    }

    #[test]
    fn test_evict_from_waiting() {
        let mut state = TaskState::Waiting { reason: WaitReason::Timer, deadline: 100 };
        assert!(state.evict(EvictionReason::ResourceExhaustion).is_ok());
        assert!(state.is_evicting());
    }

    #[test]
    fn test_evict_from_ready() {
        let mut state = TaskState::Ready;
        assert!(state.evict(EvictionReason::StateCorruption).is_ok());
        assert!(state.is_evicting());
    }

    #[test]
    fn test_cannot_evict_dead() {
        let mut state = TaskState::Dead;
        assert!(state.evict(EvictionReason::InvalidStateTransition).is_err());
    }

    #[test]
    fn test_evicting_to_dead() {
        let mut state = TaskState::Evicting { reason: EvictionReason::SyscallPanic };
        let result = state.finalize();
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), -1);
        assert_eq!(state, TaskState::Dead);
    }

    #[test]
    fn test_evicting_is_terminated() {
        let state = TaskState::Evicting { reason: EvictionReason::LivenessTimeout };
        assert!(state.is_terminated());
        assert!(!state.is_runnable());
        assert!(!state.is_blocked());
    }
}
