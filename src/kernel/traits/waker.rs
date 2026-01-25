//! Waker Trait Definitions
//!
//! Decouples task waking from scheduler implementation.
//! IPC subsystem uses Waker trait, doesn't know about scheduler internals.
//!
//! # Key Fix: Serialized Wake Operations
//!
//! The scheduler lock MUST be held during wake to prevent races:
//!
//! ```ignore
//! // OLD (racy):
//! fn wake_one(sub: &Subscriber) {
//!     let _guard = IrqGuard::new();  // Only disables interrupts!
//!     unsafe {
//!         let mut sched = task::scheduler();
//!         sched.wake_by_pid(sub.task_id);  // Race: another CPU can modify!
//!     }
//! }
//!
//! // NEW (safe):
//! impl Waker for SchedulerWaker {
//!     fn wake(&self, task_id: TaskId) {
//!         // Takes scheduler lock internally, serializing all wakes
//!         with_scheduler(|sched| sched.wake_by_pid(task_id));
//!     }
//! }
//! ```

/// Task identifier
pub type TaskId = u32;

// ============================================================================
// Wake Reason
// ============================================================================

/// Reason for waking a task
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WakeReason {
    /// Data available to read
    Readable,
    /// Space available to write
    Writable,
    /// Object closed
    Closed,
    /// Connection accepted (ports)
    Accepted,
    /// Timer expired
    Timeout,
    /// Signal received
    Signal,
    /// Child process exited
    ChildExit,
}

impl WakeReason {
    /// Convert to bitmask bit
    pub fn to_bit(self) -> u8 {
        match self {
            WakeReason::Readable => 1 << 0,
            WakeReason::Writable => 1 << 1,
            WakeReason::Closed => 1 << 2,
            WakeReason::Accepted => 1 << 3,
            WakeReason::Timeout => 1 << 4,
            WakeReason::Signal => 1 << 5,
            WakeReason::ChildExit => 1 << 6,
        }
    }

    /// Create from bitmask (returns first matching)
    pub fn from_bits(bits: u8) -> Option<Self> {
        if bits & (1 << 0) != 0 { return Some(WakeReason::Readable); }
        if bits & (1 << 1) != 0 { return Some(WakeReason::Writable); }
        if bits & (1 << 2) != 0 { return Some(WakeReason::Closed); }
        if bits & (1 << 3) != 0 { return Some(WakeReason::Accepted); }
        if bits & (1 << 4) != 0 { return Some(WakeReason::Timeout); }
        if bits & (1 << 5) != 0 { return Some(WakeReason::Signal); }
        if bits & (1 << 6) != 0 { return Some(WakeReason::ChildExit); }
        None
    }
}

// ============================================================================
// Subscriber
// ============================================================================

/// A task waiting for events
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Subscriber {
    /// Task to wake
    pub task_id: TaskId,
    /// Generation counter (detects stale subscribers after task slot reuse)
    pub generation: u32,
}

impl Subscriber {
    /// Create new subscriber
    pub const fn new(task_id: TaskId, generation: u32) -> Self {
        Self { task_id, generation }
    }

    /// Create simple subscriber (generation 0 bypasses stale PID detection)
    ///
    /// Prefer using Subscriber::new() with proper generation when possible.
    pub const fn simple(task_id: TaskId) -> Self {
        Self { task_id, generation: 0 }
    }
}

// ============================================================================
// Wake List
// ============================================================================

/// Maximum subscribers per wake list
pub const MAX_WAKE_LIST: usize = 16;

/// List of subscribers to wake (returned by IPC operations)
///
/// Operations return this list. Caller wakes OUTSIDE any locks.
#[derive(Clone)]
pub struct WakeList {
    entries: [Option<Subscriber>; MAX_WAKE_LIST],
    count: usize,
}

impl WakeList {
    /// Create empty list
    pub const fn new() -> Self {
        Self {
            entries: [None; MAX_WAKE_LIST],
            count: 0,
        }
    }

    /// Add subscriber to list
    ///
    /// Silently drops if list is full (wake lists are best-effort).
    pub fn push(&mut self, sub: Subscriber) {
        if self.count < MAX_WAKE_LIST {
            self.entries[self.count] = Some(sub);
            self.count += 1;
        }
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Get count
    pub fn len(&self) -> usize {
        self.count
    }

    /// Iterate over subscribers
    pub fn iter(&self) -> impl Iterator<Item = Subscriber> + '_ {
        self.entries[..self.count].iter().filter_map(|s| *s)
    }

    /// Merge another list into this one
    pub fn merge(&mut self, other: &WakeList) {
        for sub in other.iter() {
            self.push(sub);
        }
    }
}

impl Default for WakeList {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Waker Trait
// ============================================================================

/// Trait for waking tasks
///
/// # Contract
///
/// 1. `wake()` is thread-safe (internally serialized)
/// 2. `wake()` checks task generation to prevent stale wakes
/// 3. `wake_all()` wakes all tasks in list
///
/// # Implementation Note
///
/// The implementation MUST hold the scheduler lock during wake
/// to prevent race conditions with concurrent state modifications.
pub trait Waker: Send + Sync {
    /// Wake a single task
    ///
    /// Thread-safe. Internally serializes with scheduler.
    fn wake(&self, sub: &Subscriber, reason: WakeReason);

    /// Wake all tasks in list
    fn wake_all(&self, list: &WakeList, reason: WakeReason) {
        for sub in list.iter() {
            self.wake(&sub, reason);
        }
    }

    /// Wake by task ID (no generation check)
    fn wake_pid(&self, task_id: TaskId, reason: WakeReason) {
        self.wake(&Subscriber::simple(task_id), reason);
    }
}

// ============================================================================
// Subscriber Set (for IPC objects)
// ============================================================================

/// Maximum subscribers per object
pub const MAX_SUBSCRIBERS: usize = 8;

/// Set of subscribers for an IPC object
///
/// # Key Fix: Returns Error on Overflow
///
/// Old code silently dropped subscriptions when full.
/// New code returns error so caller can handle it.
pub struct SubscriberSet {
    entries: [(Subscriber, WakeReason); MAX_SUBSCRIBERS],
    count: u8,
}

impl SubscriberSet {
    /// Create empty set
    pub const fn new() -> Self {
        const EMPTY: (Subscriber, WakeReason) = (
            Subscriber { task_id: 0, generation: 0 },
            WakeReason::Readable,
        );
        Self {
            entries: [EMPTY; MAX_SUBSCRIBERS],
            count: 0,
        }
    }

    /// Add subscriber for event type
    ///
    /// # Returns
    /// - `Ok(())` on success or if subscriber already exists (updates filter)
    /// - `Err(())` if set is full (NEVER silently drops)
    pub fn add(&mut self, sub: Subscriber, filter: WakeReason) -> Result<(), ()> {
        // Check for existing entry
        for i in 0..self.count as usize {
            if self.entries[i].0.task_id == sub.task_id {
                self.entries[i] = (sub, filter);
                return Ok(());
            }
        }

        // Add new entry
        if (self.count as usize) < MAX_SUBSCRIBERS {
            self.entries[self.count as usize] = (sub, filter);
            self.count += 1;
            Ok(())
        } else {
            Err(())  // Full - caller MUST handle this
        }
    }

    /// Remove subscriber
    pub fn remove(&mut self, sub: Subscriber) {
        for i in 0..self.count as usize {
            if self.entries[i].0.task_id == sub.task_id {
                if i < (self.count as usize - 1) {
                    self.entries[i] = self.entries[self.count as usize - 1];
                }
                self.count -= 1;
                return;
            }
        }
    }

    /// Remove by task ID only
    pub fn remove_by_task(&mut self, task_id: TaskId) {
        for i in 0..self.count as usize {
            if self.entries[i].0.task_id == task_id {
                if i < (self.count as usize - 1) {
                    self.entries[i] = self.entries[self.count as usize - 1];
                }
                self.count -= 1;
                return;
            }
        }
    }

    /// Get subscribers for specific event
    pub fn get_for(&self, filter: WakeReason) -> WakeList {
        let mut list = WakeList::new();
        for i in 0..self.count as usize {
            if self.entries[i].1 == filter {
                list.push(self.entries[i].0);
            }
        }
        list
    }

    /// Get all subscribers
    pub fn get_all(&self) -> WakeList {
        let mut list = WakeList::new();
        for i in 0..self.count as usize {
            list.push(self.entries[i].0);
        }
        list
    }

    /// Drain all subscribers
    pub fn drain(&mut self) -> WakeList {
        let list = self.get_all();
        self.count = 0;
        list
    }

    /// Check if task is subscribed
    pub fn contains(&self, task_id: TaskId) -> bool {
        for i in 0..self.count as usize {
            if self.entries[i].0.task_id == task_id {
                return true;
            }
        }
        false
    }

    /// Get count
    pub fn len(&self) -> usize {
        self.count as usize
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Check if full
    pub fn is_full(&self) -> bool {
        self.count as usize >= MAX_SUBSCRIBERS
    }
}

impl Default for SubscriberSet {
    fn default() -> Self {
        Self::new()
    }
}
