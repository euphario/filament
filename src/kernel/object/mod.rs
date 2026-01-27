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
//! | **DmaPool** | allocate | get paddr/size | - | ✓ |
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
//! - Channel state is queried from `ipc::channel` backend
//! - Port state is queried from `ipc::port` backend
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

// ============================================================================
// HasSubscriber Trait - Reduces boilerplate for subscriber management
// ============================================================================

/// Internal trait for objects that have a subscriber field.
///
/// This trait reduces boilerplate across the 7 object types that use
/// the subscriber pattern (Channel, Timer, Process, Port, Console, Klog, Mux).
///
/// # Design Note
/// This trait is pub(crate) because subscriber management is an internal
/// concern - external code uses the Pollable trait's subscribe/unsubscribe.
pub(crate) trait HasSubscriber {
    /// Get mutable reference to the subscriber field
    fn subscriber_mut(&mut self) -> &mut Option<Subscriber>;

    /// Get the current subscriber (provided)
    fn get_subscriber(&self) -> Option<Subscriber>;

    /// Check if a subscriber is registered (provided)
    fn has_subscriber(&self) -> bool {
        self.get_subscriber().is_some()
    }

    /// Set or clear the subscriber (provided)
    fn set_subscriber(&mut self, sub: Option<Subscriber>) {
        *self.subscriber_mut() = sub;
    }
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

    /// PCIe bus (device enumeration)
    PciBus(PciBusObject),

    /// PCI device (config access)
    PciDevice(PciDeviceObject),

    /// MSI vectors
    Msi(MsiObject),

    /// Bus list
    BusList(BusListObject),

    /// Ring buffer IPC (high-performance)
    Ring(RingObject),
}

// ============================================================================
// Channel Object
// ============================================================================

// ChannelState is defined in ipc/channel.rs - we query ipc backend for state
// instead of caching it locally. This ensures single source of truth.

/// IPC channel - bidirectional message passing
///
/// This is a thin wrapper around an ipc channel_id. State is not cached
/// locally - we query the ipc backend when needed. This ensures the
/// ipc::channel::ChannelState is the single source of truth.
pub struct ChannelObject {
    /// Channel ID in ipc backend (our end)
    channel_id: u32,
    /// Subscriber for wake
    subscriber: Option<Subscriber>,
}

impl ChannelObject {
    /// Create a new channel object
    pub fn new(channel_id: u32) -> Self {
        Self {
            channel_id,
            subscriber: None,
        }
    }

    /// Get the channel ID
    pub fn channel_id(&self) -> u32 { self.channel_id }

    /// Check if channel is open (queries ipc backend)
    pub fn is_open(&self) -> bool {
        crate::kernel::ipc::channel_exists(self.channel_id)
            && !crate::kernel::ipc::channel_is_closed(self.channel_id)
    }

    /// Check if channel is closed (queries ipc backend)
    pub fn is_closed(&self) -> bool {
        !crate::kernel::ipc::channel_exists(self.channel_id)
            || crate::kernel::ipc::channel_is_closed(self.channel_id)
    }

    /// Get subscriber (for waking) - delegates to trait
    pub fn subscriber(&self) -> Option<Subscriber> { self.get_subscriber() }
}

impl HasSubscriber for ChannelObject {
    fn subscriber_mut(&mut self) -> &mut Option<Subscriber> {
        &mut self.subscriber
    }

    fn get_subscriber(&self) -> Option<Subscriber> {
        self.subscriber
    }
}

impl Pollable for ChannelObject {
    fn poll(&self, filter: u8) -> PollResult {
        use crate::kernel::ipc;

        // Query ipc backend for state (single source of truth)
        if !ipc::channel_exists(self.channel_id) {
            return PollResult::closed();
        }

        let is_closed = ipc::channel_is_closed(self.channel_id);
        let has_messages = ipc::channel_has_messages(self.channel_id);

        if is_closed {
            // Peer closed - can still read remaining messages
            if (filter & poll::READABLE) != 0 && has_messages {
                PollResult::readable()
            } else {
                PollResult::closed()
            }
        } else {
            // Channel open
            let mut ready = 0u8;
            if (filter & poll::READABLE) != 0 && has_messages {
                ready |= poll::READABLE;
            }
            if (filter & poll::WRITABLE) != 0 && !ipc::channel_queue_full(self.channel_id) {
                ready |= poll::WRITABLE;
            }
            PollResult { ready, data: 0 }
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

impl TimerState {
    /// Valid state transitions for timer
    pub fn can_transition_to(&self, new: TimerState) -> bool {
        match (*self, new) {
            // Can always disarm
            (_, TimerState::Disarmed) => true,
            // Can arm from disarmed or fired (re-arm)
            (TimerState::Disarmed, TimerState::Armed) => true,
            (TimerState::Fired, TimerState::Armed) => true,
            // Can fire only from armed
            (TimerState::Armed, TimerState::Fired) => true,
            _ => false,
        }
    }

    /// Check if timer is armed
    pub fn is_armed(&self) -> bool {
        matches!(self, TimerState::Armed)
    }

    /// Get human-readable state name
    pub fn name(&self) -> &'static str {
        match self {
            TimerState::Disarmed => "disarmed",
            TimerState::Armed => "armed",
            TimerState::Fired => "fired",
        }
    }
}

impl crate::kernel::ipc::traits::StateMachine for TimerState {
    fn can_transition_to(&self, new: &Self) -> bool {
        TimerState::can_transition_to(self, *new)
    }

    fn name(&self) -> &'static str {
        TimerState::name(self)
    }
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
    /// Get subscriber (for waking) - delegates to trait
    pub fn subscriber(&self) -> Option<Subscriber> { self.get_subscriber() }

    /// Check if timer is armed
    pub fn is_armed(&self) -> bool {
        self.state.is_armed()
    }

    /// Arm the timer with deadline and optional interval
    /// Always succeeds (can arm from any state)
    pub fn arm(&mut self, deadline: u64, interval: u64) {
        self.deadline = deadline;
        self.interval = interval;
        self.state = TimerState::Armed;
    }

    /// Fire the timer (only valid from Armed state)
    /// Returns true if transition succeeded
    pub fn fire(&mut self) -> bool {
        if self.state == TimerState::Armed {
            self.state = TimerState::Fired;
            true
        } else {
            false
        }
    }

    /// Disarm the timer (always succeeds)
    pub fn disarm(&mut self) {
        self.deadline = 0;
        self.interval = 0;
        self.state = TimerState::Disarmed;
    }

    /// Set deadline directly (for syscall handler - used with arm())
    pub(crate) fn set_deadline(&mut self, deadline: u64) {
        self.deadline = deadline;
    }

    /// Set interval directly (for syscall handler - used with arm())
    pub(crate) fn set_interval(&mut self, interval: u64) {
        self.interval = interval;
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

impl HasSubscriber for TimerObject {
    fn subscriber_mut(&mut self) -> &mut Option<Subscriber> {
        &mut self.subscriber
    }

    fn get_subscriber(&self) -> Option<Subscriber> {
        self.subscriber
    }
}

impl Pollable for TimerObject {
    fn poll(&self, filter: u8) -> PollResult {
        use crate::platform::current::timer;

        if (filter & poll::READABLE) == 0 {
            return PollResult::NONE;
        }

        if self.state() == TimerState::Armed && timer::is_expired(self.deadline()) {
            PollResult { ready: poll::READABLE, data: timer::counter() }
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
    /// Get subscriber (for waking) - delegates to trait
    pub fn subscriber(&self) -> Option<Subscriber> { self.get_subscriber() }

    /// Set exit code (internal use)
    pub(crate) fn set_exit_code(&mut self, code: Option<i32>) {
        self.exit_code = code;
    }

    /// Lazy check if target process has exited
    /// Updates exit_code cache if target has exited
    pub fn check_exit(&mut self) {
        if self.exit_code.is_some() {
            return; // Already cached
        }

        use crate::kernel::task;
        let code = task::with_scheduler(|sched| {
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
        });
        self.exit_code = code;
    }
}

impl HasSubscriber for ProcessObject {
    fn subscriber_mut(&mut self) -> &mut Option<Subscriber> {
        &mut self.subscriber
    }

    fn get_subscriber(&self) -> Option<Subscriber> {
        self.subscriber
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
        let has_exited = task::with_scheduler(|sched| {
            if let Some(target_slot) = sched.slot_by_pid(self.pid()) {
                if let Some(target) = sched.task(target_slot) {
                    target.state().exit_code().is_some()
                } else {
                    true // No task in slot
                }
            } else {
                true // Slot not found - task was reaped
            }
        });

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
// Port Object
// ============================================================================

// PortState is defined in ipc/port.rs - we query ipc backend for state
// instead of caching it locally. This ensures single source of truth.

/// Named port for accepting connections
///
/// This is a thin wrapper around an ipc port_id. State is not cached
/// locally - we query the ipc backend when needed. This ensures the
/// ipc::port::PortState is the single source of truth.
pub struct PortObject {
    /// Port ID in ipc backend
    port_id: u32,
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

    /// Check if port is listening (queries ipc backend)
    pub fn is_listening(&self) -> bool {
        crate::kernel::ipc::port_is_listening(self.port_id)
    }

    /// Check if port is closed (queries ipc backend)
    pub fn is_closed(&self) -> bool {
        !crate::kernel::ipc::port_exists(self.port_id)
            || !crate::kernel::ipc::port_is_listening(self.port_id)
    }

    /// Get the port name
    pub fn name(&self) -> &[u8] { &self.name[..self.name_len as usize] }

    /// Check if port name matches
    pub fn name_matches(&self, other: &[u8]) -> bool {
        self.name() == other
    }

    /// Get subscriber (for waking) - delegates to trait
    pub fn subscriber(&self) -> Option<Subscriber> { self.get_subscriber() }
}

impl HasSubscriber for PortObject {
    fn subscriber_mut(&mut self) -> &mut Option<Subscriber> {
        &mut self.subscriber
    }

    fn get_subscriber(&self) -> Option<Subscriber> {
        self.subscriber
    }
}

impl Pollable for PortObject {
    fn poll(&self, filter: u8) -> PollResult {
        // Query ipc backend for state (single source of truth)
        if self.is_closed() {
            return PollResult::closed();
        }

        // Port is listening
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
    /// Subscriber for mux integration (woken when notified)
    mux_subscriber: Option<Subscriber>,
    /// Notified flag - set by notify(), cleared by read/poll
    notified: bool,
}

impl ShmemObject {
    pub fn new(shmem_id: u32, paddr: u64, size: usize, vaddr: u64) -> Self {
        Self { shmem_id, paddr, size, vaddr, peer_subscriber: None, mux_subscriber: None, notified: false }
    }
    pub fn shmem_id(&self) -> u32 { self.shmem_id }
    pub fn paddr(&self) -> u64 { self.paddr }
    pub fn size(&self) -> usize { self.size }
    pub fn vaddr(&self) -> u64 { self.vaddr }
    pub fn peer_subscriber(&self) -> Option<Subscriber> { self.peer_subscriber }
    pub(crate) fn set_vaddr(&mut self, vaddr: u64) { self.vaddr = vaddr; }
    pub(crate) fn set_peer_subscriber(&mut self, sub: Option<Subscriber>) { self.peer_subscriber = sub; }

    // Mux integration
    pub fn mux_subscriber(&self) -> Option<Subscriber> { self.mux_subscriber }
    pub fn set_mux_subscriber(&mut self, sub: Option<Subscriber>) { self.mux_subscriber = sub; }
    pub fn is_notified(&self) -> bool { self.notified }
    pub fn set_notified(&mut self, notified: bool) { self.notified = notified; }
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
    /// Get subscriber (for waking) - delegates to trait
    pub fn subscriber(&self) -> Option<Subscriber> { self.get_subscriber() }
}

impl HasSubscriber for ConsoleObject {
    fn subscriber_mut(&mut self) -> &mut Option<Subscriber> {
        &mut self.subscriber
    }

    fn get_subscriber(&self) -> Option<Subscriber> {
        self.subscriber
    }
}

impl Pollable for ConsoleObject {
    fn poll(&self, filter: u8) -> PollResult {
        use crate::platform::current::uart;

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
            crate::platform::current::uart::block_for_input(subscriber.task_id);
        }
    }

    fn unsubscribe(&mut self) {
        let is_stdin = self.console_type() == ConsoleType::Stdin;
        self.set_subscriber(None);
        if is_stdin {
            crate::platform::current::uart::clear_blocked();
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
    /// Get subscriber (for waking) - delegates to trait
    pub fn subscriber(&self) -> Option<Subscriber> { self.get_subscriber() }
    pub(crate) fn set_read_pos(&mut self, pos: usize) { self.read_pos = pos; }
}

impl HasSubscriber for KlogObject {
    fn subscriber_mut(&mut self) -> &mut Option<Subscriber> {
        &mut self.subscriber
    }

    fn get_subscriber(&self) -> Option<Subscriber> {
        self.subscriber
    }
}

impl Pollable for KlogObject {
    fn poll(&self, filter: u8) -> PollResult {
        if (filter & poll::READABLE) == 0 {
            return PollResult::NONE;
        }
        // Check if there's unread log data
        // For now, always report readable (log buffer always has data)
        PollResult::readable()
    }

    fn subscribe(&mut self, subscriber: Subscriber) {
        self.set_subscriber(Some(subscriber));
    }

    fn unsubscribe(&mut self) {
        self.set_subscriber(None);
    }
}

// ============================================================================
// Mux Object (multiplexer)
// ============================================================================

/// Maximum watches per Mux object
pub const MAX_MUX_WATCHES: usize = 16;

/// Maximum events returned per mux poll
pub const MAX_MUX_EVENTS: usize = 16;

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
    watches: [Option<MuxWatch>; MAX_MUX_WATCHES],
    /// Subscriber to wake
    subscriber: Option<Subscriber>,
}

impl MuxObject {
    pub fn new() -> Self {
        Self { watches: [None; MAX_MUX_WATCHES], subscriber: None }
    }
    /// Get subscriber (for waking) - delegates to trait
    pub fn subscriber(&self) -> Option<Subscriber> { self.get_subscriber() }
    pub fn watches(&self) -> &[Option<MuxWatch>; MAX_MUX_WATCHES] { &self.watches }
    pub fn watches_mut(&mut self) -> &mut [Option<MuxWatch>; MAX_MUX_WATCHES] { &mut self.watches }
}

impl HasSubscriber for MuxObject {
    fn subscriber_mut(&mut self) -> &mut Option<Subscriber> {
        &mut self.subscriber
    }

    fn get_subscriber(&self) -> Option<Subscriber> {
        self.subscriber
    }
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
// PCI Device Object (for config read/write)
// ============================================================================

/// PCI device handle for configuration space access
pub struct PciDeviceObject {
    /// Device BDF (bus/device/function)
    bdf: u32,
}

impl PciDeviceObject {
    pub fn new(bdf: u32) -> Self {
        Self { bdf }
    }
    pub fn bdf(&self) -> u32 { self.bdf }
}

// ============================================================================
// MSI Object
// ============================================================================

/// MSI vector allocation result
pub struct MsiObject {
    /// Device BDF this MSI belongs to
    bdf: u32,
    /// First IRQ number
    first_irq: u32,
    /// Number of vectors allocated
    count: u8,
}

impl MsiObject {
    pub fn new(bdf: u32, first_irq: u32, count: u8) -> Self {
        Self { bdf, first_irq, count }
    }
    pub fn bdf(&self) -> u32 { self.bdf }
    pub fn first_irq(&self) -> u32 { self.first_irq }
    pub fn count(&self) -> u8 { self.count }
}

// ============================================================================
// Bus List Object
// ============================================================================

/// Bus enumeration iterator
pub struct BusListObject {
    /// Current cursor position
    cursor: u32,
}

impl BusListObject {
    pub fn new() -> Self {
        Self { cursor: 0 }
    }
    pub fn cursor(&self) -> u32 { self.cursor }
    pub fn advance(&mut self) { self.cursor += 1; }
}

// ============================================================================
// Ring Object (high-performance IPC)
// ============================================================================

/// Ring buffer IPC - high-performance typed message passing
///
/// Memory layout:
/// ```text
/// ┌───────────────────────────────────────────────────────────────┐
/// │ RingHeader (64 bytes, cache-line aligned)                     │
/// │   tx_head, tx_tail, rx_head, rx_tail, configs, flags          │
/// ├───────────────────────────────────────────────────────────────┤
/// │ TX Ring Buffer (configurable size, e.g., 64KB)                │
/// │   Producer writes here, consumer reads                        │
/// ├───────────────────────────────────────────────────────────────┤
/// │ RX Ring Buffer (configurable size, e.g., 4KB)                 │
/// │   Consumer writes here, producer reads (for acks/flow ctrl)   │
/// └───────────────────────────────────────────────────────────────┘
/// ```
///
/// # Design
///
/// The ring operates in shared memory with lock-free head/tail pointers.
/// - Producer atomically updates tx_tail after writing
/// - Consumer atomically updates tx_head after reading
/// - Notifications are batched (wake only on empty→non-empty transition)
///
/// # Syscall behavior
///
/// - `open(Ring, flags, arg)` - Create ring, arg points to RingParams
/// - `map(handle)` - Map shared memory into task's address space
/// - `read(handle, buf)` - Wait for data available (returns immediately if data present)
/// - `write(handle, buf)` - Notify peer that data was written (for wakeup)
/// - `close(handle)` - Unmap and cleanup
pub struct RingObject {
    /// Physical address of shared memory region
    paddr: u64,
    /// Total size (header + tx ring + rx ring)
    size: usize,
    /// Mapped virtual address (0 = not mapped)
    vaddr: u64,
    /// TX ring configuration
    tx_config: u16,
    /// RX ring configuration
    rx_config: u16,
    /// Peer task ID (for waking)
    peer_pid: Option<u32>,
    /// Subscriber for wake notifications
    subscriber: Option<Subscriber>,
    /// Is this the "creator" side (owns the physical memory)?
    is_creator: bool,
}

impl RingObject {
    /// Create a new ring object (creator side - owns memory)
    pub fn new_creator(paddr: u64, size: usize, tx_config: u16, rx_config: u16) -> Self {
        Self {
            paddr,
            size,
            vaddr: 0,
            tx_config,
            rx_config,
            peer_pid: None,
            subscriber: None,
            is_creator: true,
        }
    }

    /// Create a ring object from existing shared memory (peer side)
    pub fn new_peer(paddr: u64, size: usize, tx_config: u16, rx_config: u16) -> Self {
        Self {
            paddr,
            size,
            vaddr: 0,
            tx_config,
            rx_config,
            peer_pid: None,
            subscriber: None,
            is_creator: false,
        }
    }

    pub fn paddr(&self) -> u64 { self.paddr }
    pub fn size(&self) -> usize { self.size }
    pub fn vaddr(&self) -> u64 { self.vaddr }
    pub fn tx_config(&self) -> u16 { self.tx_config }
    pub fn rx_config(&self) -> u16 { self.rx_config }
    pub fn peer_pid(&self) -> Option<u32> { self.peer_pid }
    pub fn is_creator(&self) -> bool { self.is_creator }
    pub fn subscriber(&self) -> Option<Subscriber> { self.get_subscriber() }

    pub(crate) fn set_vaddr(&mut self, vaddr: u64) { self.vaddr = vaddr; }
    pub(crate) fn set_peer_pid(&mut self, pid: Option<u32>) { self.peer_pid = pid; }

    /// Calculate TX ring offset from base
    pub fn tx_ring_offset(&self) -> usize {
        64 // After RingHeader
    }

    /// Calculate RX ring offset from base
    pub fn rx_ring_offset(&self) -> usize {
        64 + abi::ring_config::ring_size(self.tx_config)
    }

    /// Calculate TX ring size
    pub fn tx_ring_size(&self) -> usize {
        abi::ring_config::ring_size(self.tx_config)
    }

    /// Calculate RX ring size
    pub fn rx_ring_size(&self) -> usize {
        abi::ring_config::ring_size(self.rx_config)
    }
}

impl HasSubscriber for RingObject {
    fn subscriber_mut(&mut self) -> &mut Option<Subscriber> {
        &mut self.subscriber
    }

    fn get_subscriber(&self) -> Option<Subscriber> {
        self.subscriber
    }
}

impl Pollable for RingObject {
    fn poll(&self, filter: u8) -> PollResult {
        // Ring is always readable/writable if mapped (userspace manages head/tail)
        // We just check if mapped and return ready
        if self.vaddr == 0 {
            return PollResult::NONE;
        }

        let mut ready = 0u8;

        // Check CLOSED first - if peer died, report closed
        if (filter & poll::CLOSED) != 0 {
            if let Some(peer) = self.peer_pid {
                // Check if peer task is still alive
                // Use try_scheduler to avoid deadlock if called from context holding scheduler lock
                let peer_alive = if let Some(sched) = crate::kernel::task::try_scheduler() {
                    sched.slot_by_pid(peer).is_some()
                } else {
                    true // Assume alive if we can't check (lock held)
                };
                if !peer_alive {
                    ready |= poll::CLOSED;
                }
            }
        }

        if (filter & poll::READABLE) != 0 {
            ready |= poll::READABLE;
        }
        if (filter & poll::WRITABLE) != 0 {
            ready |= poll::WRITABLE;
        }
        PollResult { ready, data: 0 }
    }

    fn subscribe(&mut self, subscriber: Subscriber) {
        self.set_subscriber(Some(subscriber));
    }

    fn unsubscribe(&mut self) {
        self.set_subscriber(None);
    }
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
    messages: [Option<Message>; MAX_QUEUE_DEPTH],
    head: u8,
    tail: u8,
    count: u8,
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

    pub fn len(&self) -> usize {
        self.count as usize
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
        assert_eq!(timer.state(), TimerState::Disarmed);
        assert_eq!(timer.deadline(), 0);
        assert_eq!(timer.interval(), 0);
    }

    #[test]
    fn test_timer_arm() {
        let mut timer = TimerObject::new();
        timer.arm(1000, 0);
        assert_eq!(timer.state(), TimerState::Armed);
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
        assert_eq!(timer.state(), TimerState::Disarmed);
    }

    #[test]
    fn test_timer_fire() {
        let mut timer = TimerObject::new();
        timer.arm(1000, 0);
        assert!(timer.fire());
        assert_eq!(timer.state(), TimerState::Fired);
    }

    #[test]
    fn test_timer_fire_invalid() {
        let mut timer = TimerObject::new();
        // Can't fire from Disarmed
        assert!(!timer.fire());
        assert_eq!(timer.state(), TimerState::Disarmed);
    }

    #[test]
    fn test_timer_is_armed() {
        let timer = TimerObject::new();
        assert!(!timer.is_armed());

        let mut timer = TimerObject::new();
        timer.arm(1000, 0);
        assert!(timer.is_armed());
    }

    #[test]
    fn test_timer_state_transitions() {
        // Test valid transitions
        assert!(TimerState::Disarmed.can_transition_to(TimerState::Armed));
        assert!(TimerState::Armed.can_transition_to(TimerState::Fired));
        assert!(TimerState::Fired.can_transition_to(TimerState::Armed)); // re-arm
        assert!(TimerState::Armed.can_transition_to(TimerState::Disarmed));
        assert!(TimerState::Fired.can_transition_to(TimerState::Disarmed));

        // Test invalid transitions
        assert!(!TimerState::Disarmed.can_transition_to(TimerState::Fired));
        assert!(!TimerState::Fired.can_transition_to(TimerState::Fired));
    }

    // Channel state is now queried from ipc backend - no local enum to test
    // ChannelObject.is_open() and is_closed() query ipc::channel_exists/is_closed

    // Port state is now queried from ipc backend - no local enum to test
    // PortObject.is_listening() and is_closed() query ipc::port_exists/is_listening

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
