//! Unified Object System
//!
//! Everything is a handle. Five syscalls:
//! - open(type, params) → handle
//! - read(handle, buf) → bytes
//! - write(handle, buf) → bytes
//! - map(handle) → vaddr
//! - close(handle)
//!
//! # Design Principles
//!
//! 1. **Trait-based contracts**: All pollable objects implement `Pollable`
//! 2. **State machines**: Objects have explicit states with valid transitions
//! 3. **Functional style**: Minimize mutation, prefer pure functions
//! 4. **Testability**: Traits allow mocking and unit testing
//!
//! # Object Types
//!
//! | Type | open() | read() | write() | map() |
//! |------|--------|--------|---------|-------|
//! | Channel | create pair | recv msg | send msg | ✗ |
//! | Timer | create | wait for tick | set deadline | ✗ |
//! | Process | spawn binary | wait for exit → code | kill? | ✗ |
//! | Port | register name | accept → new handle | ✗ | ✗ |
//! | Shmem | create region | notify signal | signal peer | ✓ |
//! | DmaPool | allocate | ✗ | ✗ | ✓ |
//! | Mmio | request region | ✗ | ✗ | ✓ |
//! | Console | get stdin/out | read input | write output | ✗ |
//! | Klog | create | read log lines | ✗ | ✗ |
//! | Mux | create | wait → (handle, event) | add/remove watch | ✗ |

use crate::kernel::ipc::waker::SubscriberSet;
use crate::kernel::ipc::traits::Subscriber;
use crate::kernel::task::TaskId;

// ============================================================================
// Pollable Trait - The contract for waitable objects
// ============================================================================

/// Poll filter flags - imported from ABI (single source of truth)
pub use abi::mux_filter as poll;

/// Result of polling an object
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PollResult {
    /// Which events are ready (bitmask of poll::*)
    pub ready: u8,
    /// Additional data (type-specific)
    pub data: u64,
}

impl PollResult {
    pub const NONE: Self = Self { ready: 0, data: 0 };

    pub fn readable() -> Self {
        Self { ready: poll::READABLE, data: 0 }
    }

    pub fn writable() -> Self {
        Self { ready: poll::WRITABLE, data: 0 }
    }

    pub fn closed() -> Self {
        Self { ready: poll::CLOSED, data: 0 }
    }

    pub fn is_ready(&self) -> bool {
        self.ready != 0
    }
}

/// Trait for objects that can be polled for readiness
///
/// Implementors should:
/// 1. Return current readiness state from `poll()`
/// 2. Store subscriber in `subscribe()` for later wakeup
/// 3. Clear subscriber in `unsubscribe()`
pub trait Pollable {
    /// Check current readiness state (non-blocking)
    fn poll(&self, filter: u8) -> PollResult;

    /// Register subscriber to be woken when events occur
    fn subscribe(&mut self, subscriber: Subscriber);

    /// Unregister subscriber
    fn unsubscribe(&mut self);
}

pub mod types;
pub mod handle;
pub mod syscall;

// Re-exports (ObjectType is defined below, not in types)
pub use handle::{Handle, HandleTable};

// ============================================================================
// Object Type - imported from abi crate (single source of truth)
// ============================================================================

pub use abi::ObjectType;

// ============================================================================
// Object (the kernel-side state)
// ============================================================================

/// The kernel object behind a handle
pub enum Object {
    /// IPC channel (peer is another handle or closed)
    Channel(ChannelObject),

    /// Timer
    Timer(TimerObject),

    /// Child process
    Process(ProcessObject),

    /// Named port (listening)
    Port(PortObject),

    /// Shared memory
    Shmem(ShmemObject),

    /// DMA pool
    DmaPool(DmaPoolObject),

    /// MMIO region
    Mmio(MmioObject),

    /// Console I/O
    Console(ConsoleObject),

    /// Kernel log
    Klog(KlogObject),

    /// Multiplexer
    Mux(MuxObject),

    /// PCIe bus
    PciBus(PciBusObject),
}

// ============================================================================
// Channel Object
// ============================================================================

/// Channel state - explicit state machine
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChannelState {
    /// Channel is open and connected
    Open,
    /// Peer has closed, but messages may remain in queue
    HalfClosed,
    /// Fully closed
    Closed,
}

impl ChannelState {
    /// Valid state transitions
    pub fn can_transition_to(&self, new: ChannelState) -> bool {
        match (*self, new) {
            (ChannelState::Open, ChannelState::HalfClosed) => true,
            (ChannelState::Open, ChannelState::Closed) => true,
            (ChannelState::HalfClosed, ChannelState::Closed) => true,
            _ => false,
        }
    }
}

/// IPC channel - bidirectional message passing
pub struct ChannelObject {
    /// Channel ID in ipc backend (our end)
    pub channel_id: u32,
    /// Current state
    pub state: ChannelState,
    /// Subscriber for wake
    pub subscriber: Option<Subscriber>,
}

impl ChannelObject {
    pub fn new(channel_id: u32) -> Self {
        Self {
            channel_id,
            state: ChannelState::Open,
            subscriber: None,
        }
    }

    /// Transition to new state (validates transition)
    pub fn transition(&mut self, new_state: ChannelState) -> bool {
        if self.state.can_transition_to(new_state) {
            self.state = new_state;
            true
        } else {
            false
        }
    }
}

impl Pollable for ChannelObject {
    fn poll(&self, filter: u8) -> PollResult {
        use crate::kernel::ipc;

        match self.state {
            ChannelState::Closed => PollResult::closed(),
            ChannelState::HalfClosed => {
                // Can still read remaining messages
                if (filter & poll::READABLE) != 0 && ipc::channel_has_messages(self.channel_id) {
                    PollResult::readable()
                } else {
                    PollResult::closed()
                }
            }
            ChannelState::Open => {
                let mut ready = 0u8;
                if (filter & poll::READABLE) != 0 && ipc::channel_has_messages(self.channel_id) {
                    ready |= poll::READABLE;
                }
                if (filter & poll::WRITABLE) != 0 && !ipc::channel_queue_full(self.channel_id) {
                    ready |= poll::WRITABLE;
                }
                PollResult { ready, data: 0 }
            }
        }
    }

    fn subscribe(&mut self, subscriber: Subscriber) {
        self.subscriber = Some(subscriber);
        // Also register with ipc backend
        let _ = crate::kernel::ipc::subscribe(self.channel_id, subscriber.task_id, subscriber, crate::kernel::ipc::WakeReason::Readable);
    }

    fn unsubscribe(&mut self) {
        self.subscriber = None;
    }
}

// ============================================================================
// Timer Object
// ============================================================================

/// Timer state
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerState {
    /// Timer not set
    Disarmed,
    /// Timer set, waiting for deadline
    Armed,
    /// Timer fired (deadline passed)
    Fired,
}

/// Timer - write to set, read to wait
pub struct TimerObject {
    /// Current state
    pub state: TimerState,
    /// Deadline tick (0 = disarmed)
    pub deadline: u64,
    /// Interval for recurring (0 = one-shot)
    pub interval: u64,
    /// Subscriber to wake
    pub subscriber: Option<Subscriber>,
}

impl TimerObject {
    pub fn new() -> Self {
        Self {
            state: TimerState::Disarmed,
            deadline: 0,
            interval: 0,
            subscriber: None,
        }
    }

    /// Arm the timer with deadline and optional interval
    pub fn arm(&mut self, deadline: u64, interval: u64) {
        self.deadline = deadline;
        self.interval = interval;
        self.state = TimerState::Armed;
    }

    /// Disarm the timer
    pub fn disarm(&mut self) {
        self.deadline = 0;
        self.interval = 0;
        self.state = TimerState::Disarmed;
    }

    /// Check and update state based on current tick
    pub fn check(&mut self, current_tick: u64) -> bool {
        if self.state == TimerState::Armed && current_tick >= self.deadline {
            self.state = TimerState::Fired;
            true
        } else {
            false
        }
    }

    /// Consume fired state (for one-shot) or rearm (for recurring)
    pub fn consume(&mut self, current_tick: u64) {
        if self.state == TimerState::Fired {
            if self.interval > 0 {
                // Recurring - rearm
                self.deadline = current_tick + self.interval;
                self.state = TimerState::Armed;
            } else {
                // One-shot - disarm
                self.disarm();
            }
        }
    }
}

impl Pollable for TimerObject {
    fn poll(&self, filter: u8) -> PollResult {
        if (filter & poll::READABLE) == 0 {
            return PollResult::NONE;
        }

        let current_tick = crate::platform::mt7988::timer::ticks();
        if self.state == TimerState::Armed && current_tick >= self.deadline {
            PollResult { ready: poll::READABLE, data: current_tick }
        } else if self.state == TimerState::Fired {
            PollResult { ready: poll::READABLE, data: self.deadline }
        } else {
            PollResult::NONE
        }
    }

    fn subscribe(&mut self, subscriber: Subscriber) {
        self.subscriber = Some(subscriber);
    }

    fn unsubscribe(&mut self) {
        self.subscriber = None;
    }
}

// ============================================================================
// Process Object
// ============================================================================

/// Child process handle
pub struct ProcessObject {
    /// Child PID
    pub pid: TaskId,
    /// Exit code (Some when exited)
    pub exit_code: Option<i32>,
    /// Subscriber to wake on exit
    pub subscriber: Option<Subscriber>,
}

impl ProcessObject {
    /// Lazy check if target process has exited
    /// Updates exit_code cache if target has exited
    pub fn check_exit(&mut self) {
        if self.exit_code.is_some() {
            return; // Already cached
        }

        use crate::kernel::task;
        let code = unsafe {
            let sched = task::scheduler();
            if let Some(target_slot) = sched.slot_by_pid(self.pid) {
                if let Some(ref target) = sched.tasks[target_slot] {
                    target.state.exit_code()
                } else {
                    Some(-1)
                }
            } else {
                // Task not found - was reaped
                Some(-1)
            }
        };
        self.exit_code = code;
    }
}

impl Pollable for ProcessObject {
    fn poll(&self, filter: u8) -> PollResult {
        if (filter & poll::READABLE) == 0 {
            return PollResult::NONE;
        }

        // Check cached exit code first
        if self.exit_code.is_some() {
            return PollResult::readable();
        }

        // Lazy check - see if target has exited
        // Note: We can't mutate self here, so do a non-caching check
        use crate::kernel::task;
        let has_exited = unsafe {
            let sched = task::scheduler();
            if let Some(target_slot) = sched.slot_by_pid(self.pid) {
                if let Some(ref target) = sched.tasks[target_slot] {
                    target.state.exit_code().is_some()
                } else {
                    true // No task in slot
                }
            } else {
                true // Slot not found - task was reaped
            }
        };

        if has_exited {
            PollResult::readable()
        } else {
            PollResult::NONE
        }
    }

    fn subscribe(&mut self, subscriber: Subscriber) {
        self.subscriber = Some(subscriber);
    }

    fn unsubscribe(&mut self) {
        self.subscriber = None;
    }
}

// ============================================================================
// Port Object
// ============================================================================

/// Port state
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortState {
    /// Port is listening for connections
    Listening,
    /// Port is closed
    Closed,
}

/// Named port for accepting connections
pub struct PortObject {
    /// Port ID in ipc backend
    pub port_id: u32,
    /// Port state
    pub state: PortState,
    /// Port name
    pub name: [u8; 32],
    pub name_len: u8,
    /// Subscriber to wake on connection
    pub subscriber: Option<Subscriber>,
}

impl PortObject {
    pub fn new(port_id: u32, name: &[u8]) -> Self {
        let mut obj = Self {
            port_id,
            state: PortState::Listening,
            name: [0u8; 32],
            name_len: name.len().min(32) as u8,
            subscriber: None,
        };
        let len = obj.name_len as usize;
        obj.name[..len].copy_from_slice(&name[..len]);
        obj
    }
}

impl Pollable for PortObject {
    fn poll(&self, filter: u8) -> PollResult {
        match self.state {
            PortState::Closed => PollResult::closed(),
            PortState::Listening => {
                if (filter & poll::READABLE) != 0 {
                    // Check ipc for pending connections
                    if crate::kernel::ipc::port_has_pending(self.port_id) {
                        PollResult::readable()
                    } else {
                        PollResult::NONE
                    }
                } else {
                    PollResult::NONE
                }
            }
        }
    }

    fn subscribe(&mut self, subscriber: Subscriber) {
        self.subscriber = Some(subscriber);
    }

    fn unsubscribe(&mut self) {
        self.subscriber = None;
    }
}

// ============================================================================
// Memory Objects (mappable)
// ============================================================================

/// Shared memory region
pub struct ShmemObject {
    /// Physical address
    pub paddr: u64,
    /// Size in bytes
    pub size: usize,
    /// Mapped virtual address (per-task, 0 = not mapped)
    pub vaddr: u64,
    /// Peer notification (for signaling)
    pub peer_subscriber: Option<Subscriber>,
}

/// DMA pool
pub struct DmaPoolObject {
    /// Physical address
    pub paddr: u64,
    /// Size in bytes
    pub size: usize,
    /// Mapped virtual address
    pub vaddr: u64,
}

/// MMIO region
pub struct MmioObject {
    /// Physical address
    pub paddr: u64,
    /// Size in bytes
    pub size: usize,
    /// Mapped virtual address
    pub vaddr: u64,
}

// ============================================================================
// Console Object
// ============================================================================

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConsoleType {
    Stdin,
    Stdout,
    Stderr,
}

pub struct ConsoleObject {
    pub console_type: ConsoleType,
    pub subscriber: Option<Subscriber>,
}

impl ConsoleObject {
    pub fn new(console_type: ConsoleType) -> Self {
        Self {
            console_type,
            subscriber: None,
        }
    }
}

impl Pollable for ConsoleObject {
    fn poll(&self, filter: u8) -> PollResult {
        use crate::platform::mt7988::uart;

        match self.console_type {
            ConsoleType::Stdin => {
                if (filter & poll::READABLE) != 0 && uart::rx_buffer_has_data() {
                    PollResult::readable()
                } else {
                    PollResult::NONE
                }
            }
            ConsoleType::Stdout | ConsoleType::Stderr => {
                // Always writable (buffered)
                if (filter & poll::WRITABLE) != 0 {
                    PollResult::writable()
                } else {
                    PollResult::NONE
                }
            }
        }
    }

    fn subscribe(&mut self, subscriber: Subscriber) {
        self.subscriber = Some(subscriber);
        // For stdin, register with UART for input notification
        if self.console_type == ConsoleType::Stdin {
            crate::platform::mt7988::uart::block_for_input(subscriber.task_id);
        }
    }

    fn unsubscribe(&mut self) {
        self.subscriber = None;
        if self.console_type == ConsoleType::Stdin {
            crate::platform::mt7988::uart::clear_blocked();
        }
    }
}

// ============================================================================
// Klog Object
// ============================================================================

pub struct KlogObject {
    /// Read position in kernel log buffer
    pub read_pos: usize,
    pub subscriber: Option<Subscriber>,
}

// ============================================================================
// Mux Object (multiplexer)
// ============================================================================

/// Watch entry for multiplexer
#[derive(Clone, Copy)]
pub struct MuxWatch {
    pub handle: u32,
    pub filter: u8,  // READABLE, WRITABLE, CLOSED, etc.
}

// Re-export MuxEvent and MuxFilter from abi crate - single source of truth
pub use abi::{MuxEvent, MuxFilter};

pub struct MuxObject {
    /// Watched handles
    pub watches: [Option<MuxWatch>; 16],
    /// Subscriber to wake
    pub subscriber: Option<Subscriber>,
}

// ============================================================================
// PCIe Bus Object
// ============================================================================

pub struct PciBusObject {
    /// BDF filter (0 = all devices)
    pub bdf_filter: u32,
}

// ============================================================================
// Message Queue (for channels)
// ============================================================================

// Keep these small to avoid stack overflow when creating HandleTable
// (HandleTable = 64 handles * sizeof(Object), Object sized by largest variant)
pub const MAX_MSG_SIZE: usize = 32;
pub const MAX_QUEUE_DEPTH: usize = 2;

pub struct Message {
    pub data: [u8; MAX_MSG_SIZE],
    pub len: u16,
}

pub struct MessageQueue {
    pub messages: [Option<Message>; MAX_QUEUE_DEPTH],
    pub head: u8,
    pub tail: u8,
    pub count: u8,
}

impl MessageQueue {
    pub const fn new() -> Self {
        Self {
            messages: [const { None }; MAX_QUEUE_DEPTH],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    pub fn push(&mut self, msg: Message) -> bool {
        if self.count as usize >= MAX_QUEUE_DEPTH {
            return false;
        }
        self.messages[self.tail as usize] = Some(msg);
        self.tail = (self.tail + 1) % MAX_QUEUE_DEPTH as u8;
        self.count += 1;
        true
    }

    pub fn pop(&mut self) -> Option<Message> {
        if self.count == 0 {
            return None;
        }
        let msg = self.messages[self.head as usize].take();
        self.head = (self.head + 1) % MAX_QUEUE_DEPTH as u8;
        self.count -= 1;
        msg
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn is_full(&self) -> bool {
        self.count as usize >= MAX_QUEUE_DEPTH
    }
}

// ============================================================================
// Default implementations
// ============================================================================

impl Default for ChannelObject {
    fn default() -> Self {
        Self {
            channel_id: 0,
            state: ChannelState::Closed,
            subscriber: None,
        }
    }
}

impl Default for TimerObject {
    fn default() -> Self {
        TimerObject::new()
    }
}

impl Default for MuxObject {
    fn default() -> Self {
        Self {
            watches: [const { None }; 16],
            subscriber: None,
        }
    }
}

impl Default for PortObject {
    fn default() -> Self {
        Self {
            port_id: 0,
            state: PortState::Closed,
            name: [0u8; 32],
            name_len: 0,
            subscriber: None,
        }
    }
}

impl Default for ConsoleObject {
    fn default() -> Self {
        Self {
            console_type: ConsoleType::Stdin,
            subscriber: None,
        }
    }
}
