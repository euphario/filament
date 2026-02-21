//! Supervision Queue — kernel-backed note queue for parent↔child supervision
//!
//! Replaces the single-slot shared memory mailbox transport with a reliable
//! queued message delivery system. Each SupervisionQueue is a bidirectional
//! link between a parent and child process with two ring buffers:
//!
//! - `up` ring: child → parent (PORT_REGISTER, SPAWN_ACK, CONFIG_RESULT, etc.)
//! - `down` ring: parent → child (SPAWN_CHILD, CONFIG_GET, SHUTDOWN, etc.)
//!
//! The kernel auto-injects EXIT notes into the `up` ring when a child dies.

use crate::kernel::ipc::traits::Subscriber;
use crate::kernel::ipc::waker::WakeList;
use crate::kernel::lock::{SpinLock, lock_class};
use crate::kernel::object::{PollResult, poll, WaitQueue, Waiter};
use crate::kernel::task::TaskId;
use abi::SupervisionNote;

/// Number of notes per direction (8 × 128 bytes = 1KB per direction)
pub const SUPERQ_CAPACITY: usize = 8;

/// Maximum concurrent supervision links (enough for 20+ drivers)
pub const MAX_SUPERVISIONS: usize = 32;

// ============================================================================
// SuperQ State Machine
// ============================================================================

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SuperQState {
    /// Both ends alive
    Open,
    /// Peer died, drain remaining notes
    HalfClosed,
    /// Fully closed
    Closed,
}

// ============================================================================
// SuperQRing — one-direction note ring buffer
// ============================================================================

pub struct SuperQRing {
    notes: [SupervisionNote; SUPERQ_CAPACITY],
    head: u8,
    tail: u8,
    count: u8,
}

impl SuperQRing {
    pub const fn new() -> Self {
        Self {
            notes: [SupervisionNote::empty(); SUPERQ_CAPACITY],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    /// Enqueue a note. Returns false if full.
    pub fn push(&mut self, note: SupervisionNote) -> bool {
        if self.count as usize >= SUPERQ_CAPACITY {
            return false;
        }
        self.notes[self.tail as usize] = note;
        self.tail = (self.tail + 1) % SUPERQ_CAPACITY as u8;
        self.count += 1;
        true
    }

    /// Dequeue a note. Returns None if empty.
    pub fn pop(&mut self) -> Option<SupervisionNote> {
        if self.count == 0 {
            return None;
        }
        let note = self.notes[self.head as usize];
        self.head = (self.head + 1) % SUPERQ_CAPACITY as u8;
        self.count -= 1;
        Some(note)
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn is_full(&self) -> bool {
        self.count as usize >= SUPERQ_CAPACITY
    }

    pub fn len(&self) -> usize {
        self.count as usize
    }
}

// ============================================================================
// SupervisionQueue — bidirectional parent↔child link
// ============================================================================

pub struct SupervisionQueue {
    /// Child → Parent notes
    pub up: SuperQRing,
    /// Parent → Child notes
    pub down: SuperQRing,
    /// State machine
    pub state: SuperQState,
    /// Subscriber waiting for up-direction notes (parent's Mux)
    pub up_wait: WaitQueue,
    /// Subscriber waiting for down-direction notes (child's Mux)
    pub down_wait: WaitQueue,
    /// Child task ID
    pub child_id: TaskId,
    /// Parent task ID
    pub parent_id: TaskId,
}

impl SupervisionQueue {
    pub fn new(parent_id: TaskId, child_id: TaskId) -> Self {
        Self {
            up: SuperQRing::new(),
            down: SuperQRing::new(),
            state: SuperQState::Open,
            up_wait: WaitQueue::new(),
            down_wait: WaitQueue::new(),
            child_id,
            parent_id,
        }
    }
}

// ============================================================================
// SupervisionTable — global table of all supervision links
// ============================================================================

pub struct SupervisionTable {
    queues: [Option<SupervisionQueue>; MAX_SUPERVISIONS],
}

impl SupervisionTable {
    pub const fn new() -> Self {
        Self {
            queues: [const { None }; MAX_SUPERVISIONS],
        }
    }

    /// Create a new supervision link. Returns the supervision_id.
    pub fn create(&mut self, parent_id: TaskId, child_id: TaskId) -> Option<u32> {
        for (i, slot) in self.queues.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(SupervisionQueue::new(parent_id, child_id));
                return Some(i as u32);
            }
        }
        None
    }

    /// Get a reference to a supervision queue by ID.
    pub fn get(&self, id: u32) -> Option<&SupervisionQueue> {
        self.queues.get(id as usize)?.as_ref()
    }

    /// Get a mutable reference to a supervision queue by ID.
    pub fn get_mut(&mut self, id: u32) -> Option<&mut SupervisionQueue> {
        self.queues.get_mut(id as usize)?.as_mut()
    }

    /// Find a supervision queue by child_id.
    pub fn find_by_child(&self, child_id: TaskId) -> Option<u32> {
        for (i, slot) in self.queues.iter().enumerate() {
            if let Some(q) = slot {
                if q.child_id == child_id && q.state != SuperQState::Closed {
                    return Some(i as u32);
                }
            }
        }
        None
    }

    /// Destroy a supervision link and return the wake lists.
    pub fn destroy(&mut self, id: u32) -> Option<(WakeList, WakeList)> {
        let slot = self.queues.get_mut(id as usize)?;
        let q = slot.as_mut()?;
        let up_wakes = q.up_wait.drain();
        let down_wakes = q.down_wait.drain();
        *slot = None;
        Some((up_wakes, down_wakes))
    }

    /// Inject an EXIT note into the `up` ring for a dying child.
    /// Called from microtask/child exit path.
    /// Returns the WakeList for the parent (to be woken outside locks).
    pub fn inject_exit(&mut self, child_id: TaskId, exit_code: i32) -> WakeList {
        let Some(id) = self.find_by_child(child_id) else {
            return WakeList::new();
        };
        let Some(q) = self.get_mut(id) else {
            return WakeList::new();
        };

        // Build EXIT note
        let mut note = SupervisionNote::empty();
        note.note_type = abi::supervision_note::EXIT;
        // payload: pid(u32 LE) + exit_code(i32 LE)
        note.payload[0..4].copy_from_slice(&child_id.to_le_bytes());
        note.payload[4..8].copy_from_slice(&exit_code.to_le_bytes());
        note.len = 8;

        let _ = q.up.push(note);
        q.state = SuperQState::HalfClosed;

        // Wake parent's subscriber
        q.up_wait.wake(poll::READABLE | poll::CLOSED)
    }
}

// ============================================================================
// Global supervision table
// ============================================================================

static SUPERVISION_TABLE: SpinLock<SupervisionTable> = SpinLock::new(
    lock_class::SUBSYSTEM,
    SupervisionTable::new(),
);

/// Access the global supervision table under lock.
pub fn with_supervision_table<F, R>(f: F) -> R
where
    F: FnOnce(&mut SupervisionTable) -> R,
{
    let mut table = SUPERVISION_TABLE.lock();
    f(&mut table)
}

// ============================================================================
// Public API (called from syscall handlers and elf.rs)
// ============================================================================

/// Create a new supervision link. Returns supervision_id.
pub fn create(parent_id: TaskId, child_id: TaskId) -> Option<u32> {
    with_supervision_table(|t| t.create(parent_id, child_id))
}

/// Send a note from child to parent (up direction).
/// Returns WakeList for deferred parent waking.
pub fn send_up(supervision_id: u32, note: SupervisionNote) -> Result<WakeList, ()> {
    with_supervision_table(|t| {
        let q = t.get_mut(supervision_id).ok_or(())?;
        if q.state == SuperQState::Closed {
            return Err(());
        }
        if !q.up.push(note) {
            return Err(()); // Queue full
        }
        Ok(q.up_wait.wake(poll::READABLE))
    })
}

/// Send a note from parent to child (down direction).
/// Returns WakeList for deferred child waking.
pub fn send_down(supervision_id: u32, note: SupervisionNote) -> Result<WakeList, ()> {
    with_supervision_table(|t| {
        let q = t.get_mut(supervision_id).ok_or(())?;
        if q.state == SuperQState::Closed {
            return Err(());
        }
        if !q.down.push(note) {
            return Err(()); // Queue full
        }
        Ok(q.down_wait.wake(poll::READABLE))
    })
}

/// Receive a note in parent from child (up direction).
///
/// Returns Ok(note) if a note was available, Err(true) if ring is empty
/// and child has exited (HalfClosed/Closed), Err(false) if ring is just empty.
pub fn recv_up(supervision_id: u32) -> Result<SupervisionNote, bool> {
    with_supervision_table(|t| {
        let Some(q) = t.get_mut(supervision_id) else {
            return Err(true);
        };
        match q.up.pop() {
            Some(note) => Ok(note),
            None => Err(q.state != SuperQState::Open),
        }
    })
}

/// Receive a note in child from parent (down direction).
///
/// Same semantics as recv_up: Err(true) = peer gone, Err(false) = just empty.
pub fn recv_down(supervision_id: u32) -> Result<SupervisionNote, bool> {
    with_supervision_table(|t| {
        let Some(q) = t.get_mut(supervision_id) else {
            return Err(true);
        };
        match q.down.pop() {
            Some(note) => Ok(note),
            None => Err(q.state != SuperQState::Open),
        }
    })
}

/// Poll the up ring (parent side): is there a note from the child?
pub fn poll_up(supervision_id: u32, filter: u8) -> PollResult {
    with_supervision_table(|t| {
        let Some(q) = t.get(supervision_id) else {
            return PollResult::closed();
        };
        match q.state {
            SuperQState::Closed => PollResult::closed(),
            SuperQState::HalfClosed => {
                if !q.up.is_empty() && (filter & poll::READABLE) != 0 {
                    PollResult::readable()
                } else {
                    PollResult::closed()
                }
            }
            SuperQState::Open => {
                let mut ready = 0u8;
                if (filter & poll::READABLE) != 0 && !q.up.is_empty() {
                    ready |= poll::READABLE;
                }
                if (filter & poll::WRITABLE) != 0 && !q.down.is_full() {
                    ready |= poll::WRITABLE;
                }
                PollResult { ready, data: 0 }
            }
        }
    })
}

/// Poll the down ring (child side): is there a note from the parent?
pub fn poll_down(supervision_id: u32, filter: u8) -> PollResult {
    with_supervision_table(|t| {
        let Some(q) = t.get(supervision_id) else {
            return PollResult::closed();
        };
        match q.state {
            SuperQState::Closed => PollResult::closed(),
            SuperQState::HalfClosed => {
                if !q.down.is_empty() && (filter & poll::READABLE) != 0 {
                    PollResult::readable()
                } else {
                    PollResult::closed()
                }
            }
            SuperQState::Open => {
                let mut ready = 0u8;
                if (filter & poll::READABLE) != 0 && !q.down.is_empty() {
                    ready |= poll::READABLE;
                }
                if (filter & poll::WRITABLE) != 0 && !q.up.is_full() {
                    ready |= poll::WRITABLE;
                }
                PollResult { ready, data: 0 }
            }
        }
    })
}

/// Subscribe for up-ring events (parent side).
pub fn subscribe_up(supervision_id: u32, subscriber: Subscriber) {
    with_supervision_table(|t| {
        if let Some(q) = t.get_mut(supervision_id) {
            q.up_wait.subscribe(Waiter::from_subscriber(subscriber, poll::READABLE | poll::CLOSED));
        }
    });
}

/// Subscribe for down-ring events (child side).
pub fn subscribe_down(supervision_id: u32, subscriber: Subscriber) {
    with_supervision_table(|t| {
        if let Some(q) = t.get_mut(supervision_id) {
            q.down_wait.subscribe(Waiter::from_subscriber(subscriber, poll::READABLE | poll::CLOSED));
        }
    });
}

/// Unsubscribe from up-ring events (parent side).
pub fn unsubscribe_up(supervision_id: u32, task_id: TaskId) {
    with_supervision_table(|t| {
        if let Some(q) = t.get_mut(supervision_id) {
            q.up_wait.unsubscribe(task_id);
        }
    });
}

/// Unsubscribe from down-ring events (child side).
pub fn unsubscribe_down(supervision_id: u32, task_id: TaskId) {
    with_supervision_table(|t| {
        if let Some(q) = t.get_mut(supervision_id) {
            q.down_wait.unsubscribe(task_id);
        }
    });
}

/// Close the parent end. Transitions to HalfClosed/Closed.
/// Returns WakeList for the child.
pub fn close_parent(supervision_id: u32) -> WakeList {
    with_supervision_table(|t| {
        let Some(q) = t.get_mut(supervision_id) else {
            return WakeList::new();
        };
        match q.state {
            SuperQState::Open => {
                q.state = SuperQState::HalfClosed;
                q.down_wait.wake(poll::CLOSED)
            }
            SuperQState::HalfClosed => {
                // Child already closed, now fully closed
                let wakes = q.down_wait.drain();
                q.state = SuperQState::Closed;
                wakes
            }
            SuperQState::Closed => WakeList::new(),
        }
    })
}

/// Close the child end. Transitions to HalfClosed/Closed.
/// Returns WakeList for the parent.
pub fn close_child(supervision_id: u32) -> WakeList {
    with_supervision_table(|t| {
        let Some(q) = t.get_mut(supervision_id) else {
            return WakeList::new();
        };
        match q.state {
            SuperQState::Open => {
                q.state = SuperQState::HalfClosed;
                q.up_wait.wake(poll::CLOSED)
            }
            SuperQState::HalfClosed => {
                q.state = SuperQState::Closed;
                q.up_wait.drain()
            }
            SuperQState::Closed => WakeList::new(),
        }
    })
}

/// Inject EXIT note on child death. Called from microtask cleanup.
pub fn inject_exit(child_id: TaskId, exit_code: i32) -> WakeList {
    with_supervision_table(|t| t.inject_exit(child_id, exit_code))
}
