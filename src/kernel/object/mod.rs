//! Unified Object System
//!
//! The kernel's unified interface for all resources. Everything is accessed
//! through handles using just 5 syscalls.
//!
//! # Syscall Interface
//!
//! | Syscall | Number | Purpose |
//! |---------|--------|---------|
//! | `open`  | 100 | Create or open an object → handle |
//! | `read`  | 101 | Read from object (recv, accept, poll) |
//! | `write` | 102 | Write to object (send, arm timer, add watch) |
//! | `map`   | 103 | Map object to memory address |
//! | `close` | 104 | Close handle, release resources |
//!
//! # Architecture
//!
//! ```text
//! ┌────────────────────────────────────────────────────────────────┐
//! │                    Userspace                                    │
//! │    open(Channel, ...)  read(h)  write(h, msg)  close(h)        │
//! └────────────────────────────────┬───────────────────────────────┘
//!                                  │ syscall
//! ┌────────────────────────────────▼───────────────────────────────┐
//! │                    object/syscall.rs                           │
//! │    sys_open()  sys_read()  sys_write()  sys_map()  sys_close() │
//! └────────────────────────────────┬───────────────────────────────┘
//!                                  │
//! ┌────────────────────────────────▼───────────────────────────────┐
//! │                    Object enum + HandleTable                    │
//! │  Channel │ Timer │ Port │ Process │ Shmem │ Mux │ Console │... │
//! └────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Object Types
//!
//! | Type | open() | read() | write() | map() |
//! |------|--------|--------|---------|-------|
//! | **Channel** | create pair | recv msg | send msg | - |
//! | **Timer** | create | wait for tick | set deadline | - |
//! | **Process** | spawn binary | wait for exit | kill | - |
//! | **Port** | register name | accept connection | - | - |
//! | **Shmem** | create region | - | - | ✓ |
//! | **DmaPool** | allocate | - | - | ✓ |
//! | **Mmio** | request region | - | - | ✓ |
//! | **Console** | get stdin/out | read input | write output | - |
//! | **Klog** | create | read log lines | - | - |
//! | **Mux** | create | wait for events | add/remove watch | - |
//!
//! # Public API
//!
//! ## Types
//! - [`Object`] - Enum of all kernel object types
//! - [`ObjectType`] - Type discriminator (from abi crate)
//! - [`Handle`] / [`HandleTable`] - Per-task handle allocation
//! - [`Pollable`] - Trait for objects that support polling
//! - [`PollResult`] - Result of polling (ready flags + data)
//!
//! ## Object State Machines
//! - [`ChannelState`] - Open → HalfClosed → Closed
//! - [`PortState`] - Listening → Closed
//! - [`TimerState`] - Disarmed → Armed → Fired
//!
//! ## Syscall Handlers (in [`syscall`] submodule)
//! - `sys_open()` - Create objects
//! - `sys_read()` - Read/receive/accept/poll
//! - `sys_write()` - Write/send/arm/configure
//! - `sys_map()` - Map to memory
//! - `sys_close()` - Close and cleanup
//!
//! # Design Principles
//!
//! 1. **Trait-based contracts** - All pollable objects implement [`Pollable`]
//! 2. **State machines** - Objects have explicit states with validated transitions
//! 3. **Encapsulation** - Object fields are private, accessed via methods
//! 4. **Testability** - Traits allow mocking for unit tests
//!
//! # Example
//!
//! ```ignore
//! // Userspace creates a timer
//! let h = syscall::open(ObjectType::Timer, &[])?;
//!
//! // Arm the timer (1 second from now)
//! let deadline = syscall::gettime() + 1_000_000_000;
//! syscall::write(h, &deadline.to_le_bytes())?;
//!
//! // Wait for timer
//! let mut buf = [0u8; 8];
//! syscall::read(h, &mut buf)?;  // Blocks until timer fires
//!
//! // Cleanup
//! syscall::close(h)?;
//! ```

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
    channel_id: u32,
    /// Current state
    state: ChannelState,
    /// Subscriber for wake
    subscriber: Option<Subscriber>,
}

impl ChannelObject {
    /// Get the channel ID
    pub fn channel_id(&self) -> u32 { self.channel_id }
    /// Get the current state
    pub fn state(&self) -> ChannelState { self.state }
    /// Check if channel has a subscriber
    pub fn has_subscriber(&self) -> bool { self.subscriber.is_some() }
    /// Get subscriber (for waking)
    pub fn subscriber(&self) -> Option<Subscriber> { self.subscriber }
}

impl ChannelObject {
    /// Create a new channel object
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

    /// Set subscriber (internal use)
    pub(crate) fn set_subscriber(&mut self, sub: Option<Subscriber>) {
        self.subscriber = sub;
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
        self.set_subscriber(Some(subscriber));
        // Also register with ipc backend
        let _ = crate::kernel::ipc::subscribe(self.channel_id, subscriber.task_id, subscriber, crate::kernel::ipc::WakeReason::Readable);
    }

    fn unsubscribe(&mut self) {
        self.set_subscriber(None);
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
    state: TimerState,
    /// Deadline tick (0 = disarmed)
    deadline: u64,
    /// Interval for recurring (0 = one-shot)
    interval: u64,
    /// Subscriber to wake
    subscriber: Option<Subscriber>,
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

    /// Get the current state
    pub fn state(&self) -> TimerState { self.state }
    /// Get the deadline tick
    pub fn deadline(&self) -> u64 { self.deadline }
    /// Get the interval
    pub fn interval(&self) -> u64 { self.interval }
    /// Get subscriber (for waking)
    pub fn subscriber(&self) -> Option<Subscriber> { self.subscriber }

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

    /// Set deadline directly (for syscall handler)
    pub(crate) fn set_deadline(&mut self, deadline: u64) {
        self.deadline = deadline;
    }

    /// Set interval directly (for syscall handler)
    pub(crate) fn set_interval(&mut self, interval: u64) {
        self.interval = interval;
    }

    /// Set state directly (for syscall handler)
    pub(crate) fn set_state(&mut self, state: TimerState) {
        self.state = state;
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

    /// Set subscriber (internal use)
    pub(crate) fn set_subscriber(&mut self, sub: Option<Subscriber>) {
        self.subscriber = sub;
    }
}

impl Pollable for TimerObject {
    fn poll(&self, filter: u8) -> PollResult {
        if (filter & poll::READABLE) == 0 {
            return PollResult::NONE;
        }

        let current_tick = crate::platform::mt7988::timer::ticks();
        if self.state() == TimerState::Armed && current_tick >= self.deadline() {
            PollResult { ready: poll::READABLE, data: current_tick }
        } else if self.state() == TimerState::Fired {
            PollResult { ready: poll::READABLE, data: self.deadline() }
        } else {
            PollResult::NONE
        }
    }

    fn subscribe(&mut self, subscriber: Subscriber) {
        self.set_subscriber(Some(subscriber));
    }

    fn unsubscribe(&mut self) {
        self.set_subscriber(None);
    }
}

// ============================================================================
// Process Object
// ============================================================================

/// Child process handle
pub struct ProcessObject {
    /// Child PID
    pid: TaskId,
    /// Exit code (Some when exited)
    exit_code: Option<i32>,
    /// Subscriber to wake on exit
    subscriber: Option<Subscriber>,
}

impl ProcessObject {
    /// Create a new process object
    pub fn new(pid: TaskId) -> Self {
        Self {
            pid,
            exit_code: None,
            subscriber: None,
        }
    }

    /// Get the process ID
    pub fn pid(&self) -> TaskId { self.pid }
    /// Get the exit code (if exited)
    pub fn exit_code(&self) -> Option<i32> { self.exit_code }
    /// Get subscriber (for waking)
    pub fn subscriber(&self) -> Option<Subscriber> { self.subscriber }

    /// Set exit code (internal use)
    pub(crate) fn set_exit_code(&mut self, code: Option<i32>) {
        self.exit_code = code;
    }

    /// Set subscriber (internal use)
    pub(crate) fn set_subscriber(&mut self, sub: Option<Subscriber>) {
        self.subscriber = sub;
    }

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
                if let Some(target) = sched.task(target_slot) {
                    target.state().exit_code()
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
        if self.exit_code().is_some() {
            return PollResult::readable();
        }

        // Lazy check - see if target has exited
        // Note: We can't mutate self here, so do a non-caching check
        use crate::kernel::task;
        let has_exited = unsafe {
            let sched = task::scheduler();
            if let Some(target_slot) = sched.slot_by_pid(self.pid()) {
                if let Some(target) = sched.task(target_slot) {
                    target.state().exit_code().is_some()
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
        self.set_subscriber(Some(subscriber));
    }

    fn unsubscribe(&mut self) {
        self.set_subscriber(None);
    }
}

// ============================================================================
// Child Exit Notification
// ============================================================================

/// Notify all ProcessObject handles in a task's object_table about a child exit.
///
/// Called by task lifecycle code when a child process exits.
/// Iterates the parent's object_table looking for ProcessObject handles
/// watching the exited child and updates their exit code.
///
/// Returns a list of subscribers to wake.
pub fn notify_child_exit(
    object_table: &mut HandleTable,
    child_pid: TaskId,
    exit_code: i32,
) -> crate::kernel::ipc::waker::WakeList {
    let mut wake_list = crate::kernel::ipc::waker::WakeList::new();

    for entry in object_table.entries_mut() {
        if let Object::Process(ref mut proc_obj) = entry.object {
            // Check if this ProcessObject is watching the exited child
            if proc_obj.pid() == child_pid {
                // Update the cached exit code
                proc_obj.set_exit_code(Some(exit_code));

                // Add subscriber to wake list (if any)
                if let Some(sub) = proc_obj.subscriber() {
                    wake_list.push(sub);
                }
            }
        }
    }

    wake_list
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
    port_id: u32,
    /// Port state
    state: PortState,
    /// Port name
    name: [u8; 32],
    name_len: u8,
    /// Subscriber to wake on connection
    subscriber: Option<Subscriber>,
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

    /// Get the port ID
    pub fn port_id(&self) -> u32 { self.port_id }
    /// Get the current state
    pub fn state(&self) -> PortState { self.state }
    /// Get the port name
    pub fn name(&self) -> &[u8] { &self.name[..self.name_len as usize] }
    /// Get subscriber (for waking)
    pub fn subscriber(&self) -> Option<Subscriber> { self.subscriber }

    /// Set state (internal use)
    pub(crate) fn set_state(&mut self, state: PortState) {
        self.state = state;
    }

    /// Set subscriber (internal use)
    pub(crate) fn set_subscriber(&mut self, sub: Option<Subscriber>) {
        self.subscriber = sub;
    }
}

impl Pollable for PortObject {
    fn poll(&self, filter: u8) -> PollResult {
        match self.state() {
            PortState::Closed => PollResult::closed(),
            PortState::Listening => {
                if (filter & poll::READABLE) != 0 {
                    // Check ipc for pending connections
                    if crate::kernel::ipc::port_has_pending(self.port_id()) {
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
        self.set_subscriber(Some(subscriber));
    }

    fn unsubscribe(&mut self) {
        self.set_subscriber(None);
    }
}

// ============================================================================
// Memory Objects (mappable)
// ============================================================================

/// Shared memory region
pub struct ShmemObject {
    /// Region ID in shmem table (for cleanup and notifications)
    shmem_id: u32,
    /// Physical address
    paddr: u64,
    /// Size in bytes
    size: usize,
    /// Mapped virtual address (per-task, 0 = not mapped)
    vaddr: u64,
    /// Peer notification (for signaling)
    peer_subscriber: Option<Subscriber>,
}

impl ShmemObject {
    pub fn new(shmem_id: u32, paddr: u64, size: usize, vaddr: u64) -> Self {
        Self { shmem_id, paddr, size, vaddr, peer_subscriber: None }
    }
    pub fn shmem_id(&self) -> u32 { self.shmem_id }
    pub fn paddr(&self) -> u64 { self.paddr }
    pub fn size(&self) -> usize { self.size }
    pub fn vaddr(&self) -> u64 { self.vaddr }
    pub fn peer_subscriber(&self) -> Option<Subscriber> { self.peer_subscriber }
    pub(crate) fn set_vaddr(&mut self, vaddr: u64) { self.vaddr = vaddr; }
    pub(crate) fn set_peer_subscriber(&mut self, sub: Option<Subscriber>) { self.peer_subscriber = sub; }
}

/// DMA pool
pub struct DmaPoolObject {
    /// Physical address
    paddr: u64,
    /// Size in bytes
    size: usize,
    /// Mapped virtual address
    vaddr: u64,
}

impl DmaPoolObject {
    pub fn new(paddr: u64, size: usize, vaddr: u64) -> Self {
        Self { paddr, size, vaddr }
    }
    pub fn paddr(&self) -> u64 { self.paddr }
    pub fn size(&self) -> usize { self.size }
    pub fn vaddr(&self) -> u64 { self.vaddr }
    pub(crate) fn set_vaddr(&mut self, vaddr: u64) { self.vaddr = vaddr; }
}

/// MMIO region
pub struct MmioObject {
    /// Physical address
    paddr: u64,
    /// Size in bytes
    size: usize,
    /// Mapped virtual address
    vaddr: u64,
}

impl MmioObject {
    pub fn new(paddr: u64, size: usize, vaddr: u64) -> Self {
        Self { paddr, size, vaddr }
    }
    pub fn paddr(&self) -> u64 { self.paddr }
    pub fn size(&self) -> usize { self.size }
    pub fn vaddr(&self) -> u64 { self.vaddr }
    pub(crate) fn set_vaddr(&mut self, vaddr: u64) { self.vaddr = vaddr; }
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
    console_type: ConsoleType,
    subscriber: Option<Subscriber>,
}

impl ConsoleObject {
    pub fn new(console_type: ConsoleType) -> Self {
        Self {
            console_type,
            subscriber: None,
        }
    }
    pub fn console_type(&self) -> ConsoleType { self.console_type }
    pub fn subscriber(&self) -> Option<Subscriber> { self.subscriber }
    pub(crate) fn set_subscriber(&mut self, sub: Option<Subscriber>) { self.subscriber = sub; }
}

impl Pollable for ConsoleObject {
    fn poll(&self, filter: u8) -> PollResult {
        use crate::platform::mt7988::uart;

        match self.console_type() {
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
        let is_stdin = self.console_type() == ConsoleType::Stdin;
        self.set_subscriber(Some(subscriber));
        // For stdin, register with UART for input notification
        if is_stdin {
            crate::platform::mt7988::uart::block_for_input(subscriber.task_id);
        }
    }

    fn unsubscribe(&mut self) {
        let is_stdin = self.console_type() == ConsoleType::Stdin;
        self.set_subscriber(None);
        if is_stdin {
            crate::platform::mt7988::uart::clear_blocked();
        }
    }
}

// ============================================================================
// Klog Object
// ============================================================================

pub struct KlogObject {
    /// Read position in kernel log buffer
    read_pos: usize,
    subscriber: Option<Subscriber>,
}

impl KlogObject {
    pub fn new() -> Self {
        Self { read_pos: 0, subscriber: None }
    }
    pub fn read_pos(&self) -> usize { self.read_pos }
    pub fn subscriber(&self) -> Option<Subscriber> { self.subscriber }
    pub(crate) fn set_read_pos(&mut self, pos: usize) { self.read_pos = pos; }
    pub(crate) fn set_subscriber(&mut self, sub: Option<Subscriber>) { self.subscriber = sub; }
}

// ============================================================================
// Mux Object (multiplexer)
// ============================================================================

/// Watch entry for multiplexer
#[derive(Clone, Copy)]
pub struct MuxWatch {
    handle: u32,
    filter: u8,  // READABLE, WRITABLE, CLOSED, etc.
}

impl MuxWatch {
    pub fn new(handle: u32, filter: u8) -> Self {
        Self { handle, filter }
    }
    pub fn handle(&self) -> u32 { self.handle }
    pub fn filter(&self) -> u8 { self.filter }
    pub(crate) fn set_filter(&mut self, filter: u8) { self.filter = filter; }
}

// Re-export MuxEvent from abi crate - single source of truth
pub use abi::MuxEvent;

pub struct MuxObject {
    /// Watched handles
    watches: [Option<MuxWatch>; 16],
    /// Subscriber to wake
    subscriber: Option<Subscriber>,
}

impl MuxObject {
    pub fn new() -> Self {
        Self { watches: [None; 16], subscriber: None }
    }
    pub fn subscriber(&self) -> Option<Subscriber> { self.subscriber }
    pub fn watches(&self) -> &[Option<MuxWatch>; 16] { &self.watches }
    pub fn watches_mut(&mut self) -> &mut [Option<MuxWatch>; 16] { &mut self.watches }
    pub(crate) fn set_subscriber(&mut self, sub: Option<Subscriber>) { self.subscriber = sub; }
}

// ============================================================================
// PCIe Bus Object
// ============================================================================

pub struct PciBusObject {
    /// BDF filter (0 = all devices)
    bdf_filter: u32,
}

impl PciBusObject {
    pub fn new(bdf_filter: u32) -> Self {
        Self { bdf_filter }
    }
    pub fn bdf_filter(&self) -> u32 { self.bdf_filter }
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

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Timer tests
    #[test]
    fn test_timer_new() {
        let timer = TimerObject::new();
        assert_eq!(timer.state(), &TimerState::Disarmed);
        assert_eq!(timer.deadline(), 0);
        assert_eq!(timer.interval(), 0);
    }

    #[test]
    fn test_timer_arm() {
        let mut timer = TimerObject::new();
        timer.arm(1000, 0);
        assert_eq!(timer.state(), &TimerState::Armed);
        assert_eq!(timer.deadline(), 1000);
    }

    #[test]
    fn test_timer_arm_recurring() {
        let mut timer = TimerObject::new();
        timer.arm(1000, 100);
        assert_eq!(timer.interval(), 100);
    }

    #[test]
    fn test_timer_disarm() {
        let mut timer = TimerObject::new();
        timer.arm(1000, 0);
        timer.disarm();
        assert_eq!(timer.state(), &TimerState::Disarmed);
    }

    #[test]
    fn test_timer_fire() {
        let mut timer = TimerObject::new();
        timer.arm(1000, 0);
        timer.fire();
        assert_eq!(timer.state(), &TimerState::Fired);
    }

    #[test]
    fn test_timer_is_armed() {
        let timer = TimerObject::new();
        assert!(!timer.is_armed());

        let mut timer = TimerObject::new();
        timer.arm(1000, 0);
        assert!(timer.is_armed());
    }

    // Channel state tests
    #[test]
    fn test_channel_state_open() {
        assert!(ChannelState::Open.is_open());
        assert!(!ChannelState::Closed.is_open());
    }

    #[test]
    fn test_channel_state_closed() {
        assert!(ChannelState::Closed.is_closed());
        assert!(!ChannelState::Open.is_closed());
    }

    // Port state tests
    #[test]
    fn test_port_state_listening() {
        assert!(PortState::Listening.is_listening());
        assert!(!PortState::Closed.is_listening());
    }

    // ProcessObject tests
    #[test]
    fn test_process_object_new() {
        let proc = ProcessObject::new(42);
        assert_eq!(proc.pid(), 42);
        assert!(proc.exit_code().is_none());
    }

    #[test]
    fn test_process_object_set_exit_code() {
        let mut proc = ProcessObject::new(42);
        proc.set_exit_code(Some(0));
        assert_eq!(proc.exit_code(), Some(0));
    }

    // ShmemObject tests
    #[test]
    fn test_shmem_object_new() {
        let shmem = ShmemObject::new(1, 0x8000_0000, 4096, 0x1000_0000);
        assert_eq!(shmem.shmem_id(), 1);
        assert_eq!(shmem.paddr(), 0x8000_0000);
        assert_eq!(shmem.size(), 4096);
        assert_eq!(shmem.vaddr(), 0x1000_0000);
    }

    // DmaPoolObject tests
    #[test]
    fn test_dma_pool_object_new() {
        let dma = DmaPoolObject::new(0x4000_0000, 4096, 0x2000_0000);
        assert_eq!(dma.paddr(), 0x4000_0000);
        assert_eq!(dma.size(), 4096);
        assert_eq!(dma.vaddr(), 0x2000_0000);
    }

    // MmioObject tests
    #[test]
    fn test_mmio_object_new() {
        let mmio = MmioObject::new(0x1000_0000, 0x1000, 0x3000_0000);
        assert_eq!(mmio.paddr(), 0x1000_0000);
        assert_eq!(mmio.size(), 0x1000);
        assert_eq!(mmio.vaddr(), 0x3000_0000);
    }

    // KlogObject tests
    #[test]
    fn test_klog_object_new() {
        let klog = KlogObject::new();
        assert_eq!(klog.read_pos(), 0);
    }

    #[test]
    fn test_klog_object_set_read_pos() {
        let mut klog = KlogObject::new();
        klog.set_read_pos(100);
        assert_eq!(klog.read_pos(), 100);
    }

    // PciBusObject tests
    #[test]
    fn test_pci_bus_object_new() {
        let pci = PciBusObject::new(0);
        assert_eq!(pci.bdf_filter(), 0);

        let pci = PciBusObject::new(0x0100); // Bus 1
        assert_eq!(pci.bdf_filter(), 0x0100);
    }

    // ObjectType tests
    #[test]
    fn test_object_type_from_u32() {
        assert_eq!(ObjectType::from_u32(1), Some(ObjectType::Channel));
        assert_eq!(ObjectType::from_u32(2), Some(ObjectType::Timer));
        assert_eq!(ObjectType::from_u32(99), None);
    }

    // MuxObject tests
    #[test]
    fn test_mux_object_default() {
        let mux = MuxObject::default();
        assert!(mux.subscriber().is_none());
    }

    // Subscriber tests
    #[test]
    fn test_subscriber_new() {
        let sub = Subscriber::new(123, 5);
        assert_eq!(sub.task_id, 123);
        assert_eq!(sub.generation, 5);
    }

    // ConsoleType tests
    #[test]
    fn test_console_type() {
        let console = ConsoleObject { console_type: ConsoleType::Stdout, subscriber: None };
        assert_eq!(console.console_type, ConsoleType::Stdout);
    }
}
