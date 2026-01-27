//! Unified Wake Mechanism
//!
//! This module provides the SINGLE point of waking tasks in the IPC system.
//! All wake operations go through here - no other wake paths allowed.
//!
//! # Design
//!
//! The key insight is that waking must happen OUTSIDE locks:
//! 1. Operation on object (e.g., send) returns list of subscribers to wake
//! 2. Caller releases object lock
//! 3. Caller calls `wake()` with the subscriber list
//!
//! This prevents deadlocks and priority inversion.
//!
//! # Usage
//!
//! ```ignore
//! // Inside object operation
//! fn send(&mut self, msg: Message) -> Result<WakeList, IpcError> {
//!     // ... do the send ...
//!     Ok(self.subscribers.get_for(WakeReason::Readable))
//! }
//!
//! // Caller
//! let wake_list = with_channel_table(|t| t.send(ch, msg, pid))?;
//! wake(&wake_list, WakeReason::Readable);
//! ```

use super::traits::{Subscriber, WakeReason};
use crate::kernel::task;

/// Maximum subscribers per object
pub const MAX_SUBSCRIBERS: usize = 8;

/// List of subscribers to wake (returned by operations)
///
/// This is a small fixed-size array to avoid allocation.
/// Operations return this, and caller wakes outside lock.
#[derive(Clone)]
pub struct WakeList {
    entries: [Option<Subscriber>; MAX_SUBSCRIBERS],
    count: usize,
}

impl WakeList {
    /// Create empty wake list
    pub const fn new() -> Self {
        Self {
            entries: [None; MAX_SUBSCRIBERS],
            count: 0,
        }
    }

    /// Add a subscriber to the wake list
    pub fn push(&mut self, sub: Subscriber) {
        if self.count < MAX_SUBSCRIBERS {
            self.entries[self.count] = Some(sub);
            self.count += 1;
        }
    }

    /// Add multiple subscribers
    pub fn extend(&mut self, subs: &[Subscriber]) {
        for sub in subs {
            self.push(*sub);
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

    /// Merge another wake list into this one
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
// SubscriberSet
// ============================================================================

/// Set of subscribers for an object
///
/// Each entry tracks (subscriber, filter) so we can wake only
/// those interested in a specific event type.
pub struct SubscriberSet {
    /// (Subscriber, WakeReason filter) pairs
    entries: [(Subscriber, WakeReason); MAX_SUBSCRIBERS],
    /// Number of active entries
    count: u8,
}

impl SubscriberSet {
    /// Create empty subscriber set
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

    /// Add a subscriber for a specific event type
    ///
    /// If the subscriber already exists (same task_id), updates the filter.
    /// Returns Ok(()) on success, Err(()) if subscriber set is full.
    pub fn add(&mut self, sub: Subscriber, filter: WakeReason) -> Result<(), ()> {
        // Check for existing entry with same task_id
        for i in 0..self.count as usize {
            if self.entries[i].0.task_id == sub.task_id {
                // Update existing entry
                self.entries[i] = (sub, filter);
                return Ok(());
            }
        }

        // Add new entry if space available
        if (self.count as usize) < MAX_SUBSCRIBERS {
            self.entries[self.count as usize] = (sub, filter);
            self.count += 1;
            Ok(())
        } else {
            Err(())  // Subscriber set is full
        }
    }

    /// Remove a subscriber
    pub fn remove(&mut self, sub: Subscriber) {
        for i in 0..self.count as usize {
            if self.entries[i].0.task_id == sub.task_id {
                // Swap with last entry and decrement count
                if i < (self.count as usize - 1) {
                    self.entries[i] = self.entries[self.count as usize - 1];
                }
                self.count -= 1;
                return;
            }
        }
    }

    /// Remove subscriber by task ID only
    pub fn remove_by_task(&mut self, task_id: u32) {
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

    /// Get subscribers for a specific event type
    pub fn get_for(&self, filter: WakeReason) -> WakeList {
        let mut list = WakeList::new();
        for i in 0..self.count as usize {
            if self.entries[i].1 == filter {
                list.push(self.entries[i].0);
            }
        }
        list
    }

    /// Get ALL subscribers (for close events that wake everyone)
    pub fn get_all(&self) -> WakeList {
        let mut list = WakeList::new();
        for i in 0..self.count as usize {
            list.push(self.entries[i].0);
        }
        list
    }

    /// Drain all subscribers, returning them
    pub fn drain(&mut self) -> WakeList {
        let list = self.get_all();
        self.count = 0;
        list
    }

    /// Check if a task is subscribed
    pub fn contains(&self, task_id: u32) -> bool {
        for i in 0..self.count as usize {
            if self.entries[i].0.task_id == task_id {
                return true;
            }
        }
        false
    }

    /// Get number of subscribers
    pub fn len(&self) -> usize {
        self.count as usize
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

impl Default for SubscriberSet {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Wake Functions
// ============================================================================

/// Wake all subscribers in a wake list
///
/// This is THE function that wakes tasks. No other wake paths!
/// Must be called OUTSIDE any channel/port locks.
pub fn wake(list: &WakeList, _reason: WakeReason) {
    for sub in list.iter() {
        wake_one(&sub);
    }
}

/// Wake a single subscriber
///
/// SERIALIZATION: Uses try_scheduler() to avoid deadlock when called from
/// contexts that already hold the scheduler lock (e.g., reap_terminated).
/// If lock is held, defers wake via request_wake.
fn wake_one(sub: &Subscriber) {
    // Use try_scheduler to avoid deadlock if called from reap_terminated
    if let Some(mut sched) = task::try_scheduler() {
        // Use unified wake function that handles IPC return values,
        // liveness state reset, and state validation
        sched.wake_by_pid(sub.task_id);
    } else {
        // Lock held - defer the wake
        crate::arch::aarch64::sync::cpu_flags().request_wake(sub.task_id);
    }
}

/// Wake a task by PID directly (for compatibility)
///
/// Prefer using WakeList when possible.
/// SERIALIZATION: Uses try_scheduler internally for SMP safety.
/// If scheduler lock is held, defers wake.
pub fn wake_pid(pid: u32) {
    wake_one(&Subscriber::simple(pid));
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wake_list_new() {
        let list = WakeList::new();
        assert!(list.is_empty());
        assert_eq!(list.len(), 0);
    }

    #[test]
    fn test_wake_list_push() {
        let mut list = WakeList::new();
        list.push(Subscriber::new(1, 1));
        list.push(Subscriber::new(2, 1));

        assert!(!list.is_empty());
        assert_eq!(list.len(), 2);
    }

    #[test]
    fn test_wake_list_iter() {
        let mut list = WakeList::new();
        list.push(Subscriber::new(1, 1));
        list.push(Subscriber::new(2, 2));
        list.push(Subscriber::new(3, 3));

        let ids: Vec<u32> = list.iter().map(|s| s.task_id).collect();
        assert_eq!(ids, vec![1, 2, 3]);
    }

    #[test]
    fn test_wake_list_merge() {
        let mut list1 = WakeList::new();
        list1.push(Subscriber::new(1, 1));

        let mut list2 = WakeList::new();
        list2.push(Subscriber::new(2, 2));

        list1.merge(&list2);
        assert_eq!(list1.len(), 2);
    }

    #[test]
    fn test_subscriber_set_add_remove() {
        let mut set = SubscriberSet::new();
        assert!(set.is_empty());

        assert!(set.add(Subscriber::new(1, 1), WakeReason::Readable).is_ok());
        assert_eq!(set.len(), 1);
        assert!(set.contains(1));

        assert!(set.add(Subscriber::new(2, 1), WakeReason::Writable).is_ok());
        assert_eq!(set.len(), 2);

        set.remove(Subscriber::new(1, 1));
        assert_eq!(set.len(), 1);
        assert!(!set.contains(1));
        assert!(set.contains(2));
    }

    #[test]
    fn test_subscriber_set_update_existing() {
        let mut set = SubscriberSet::new();

        assert!(set.add(Subscriber::new(1, 1), WakeReason::Readable).is_ok());
        assert!(set.add(Subscriber::new(1, 2), WakeReason::Writable).is_ok()); // Same task, different gen/filter

        // Should have only one entry (updated)
        assert_eq!(set.len(), 1);
    }

    #[test]
    fn test_subscriber_set_get_for_filter() {
        let mut set = SubscriberSet::new();

        assert!(set.add(Subscriber::new(1, 1), WakeReason::Readable).is_ok());
        assert!(set.add(Subscriber::new(2, 1), WakeReason::Writable).is_ok());
        assert!(set.add(Subscriber::new(3, 1), WakeReason::Readable).is_ok());

        let readable = set.get_for(WakeReason::Readable);
        assert_eq!(readable.len(), 2);

        let writable = set.get_for(WakeReason::Writable);
        assert_eq!(writable.len(), 1);

        let closed = set.get_for(WakeReason::Closed);
        assert_eq!(closed.len(), 0);
    }

    #[test]
    fn test_subscriber_set_drain() {
        let mut set = SubscriberSet::new();

        assert!(set.add(Subscriber::new(1, 1), WakeReason::Readable).is_ok());
        assert!(set.add(Subscriber::new(2, 1), WakeReason::Writable).is_ok());

        let drained = set.drain();
        assert_eq!(drained.len(), 2);
        assert!(set.is_empty());
    }

    #[test]
    fn test_subscriber_set_remove_by_task() {
        let mut set = SubscriberSet::new();

        assert!(set.add(Subscriber::new(1, 5), WakeReason::Readable).is_ok());
        assert!(set.add(Subscriber::new(2, 10), WakeReason::Readable).is_ok());

        set.remove_by_task(1);
        assert!(!set.contains(1));
        assert!(set.contains(2));
    }

    #[test]
    fn test_subscriber_set_overflow() {
        let mut set = SubscriberSet::new();

        // Fill up to MAX_SUBSCRIBERS
        for i in 0..MAX_SUBSCRIBERS {
            assert!(set.add(Subscriber::new(i as u32, 1), WakeReason::Readable).is_ok());
        }
        assert_eq!(set.len(), MAX_SUBSCRIBERS);

        // Next add should fail
        assert!(set.add(Subscriber::new(100, 1), WakeReason::Readable).is_err());
        assert_eq!(set.len(), MAX_SUBSCRIBERS);  // Still at max
    }
}
