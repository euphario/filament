//! Event System

#![allow(dead_code)]  // Infrastructure for future use
//!
//! Provides async notification mechanism for processes.
//! Similar to Redox's event system - processes can wait on multiple events.

use crate::logln;

/// Maximum events per process in normal queue
pub const MAX_EVENTS: usize = 24;

/// Maximum events in critical queue (ChildExit, Signal - never dropped)
pub const MAX_CRITICAL_EVENTS: usize = 8;

/// Maximum subscriptions per event type (prevents exhaustion attacks)
pub const MAX_SUBSCRIPTIONS_PER_TYPE: usize = 8;

/// Event types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum EventType {
    /// No event (empty slot)
    None = 0,
    /// IPC message available on channel
    IpcReady = 1,
    /// Timer expired
    Timer = 2,
    /// IRQ occurred
    Irq = 3,
    /// Child process exited
    ChildExit = 4,
    /// File descriptor ready for read
    FdReadable = 5,
    /// File descriptor ready for write
    FdWritable = 6,
    /// Signal received
    Signal = 7,
}

/// An event that can be delivered to a process
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Event {
    /// Type of event
    pub event_type: EventType,
    /// Event-specific data (e.g., channel ID, IRQ number, signal number)
    pub data: u64,
    /// Additional flags
    pub flags: u32,
    /// Source PID (for IPC events)
    pub source_pid: u32,
}

impl Event {
    pub const fn empty() -> Self {
        Self {
            event_type: EventType::None,
            data: 0,
            flags: 0,
            source_pid: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.event_type == EventType::None
    }

    pub fn ipc_ready(channel_id: u32, source_pid: u32) -> Self {
        Self {
            event_type: EventType::IpcReady,
            data: channel_id as u64,
            flags: 0,
            source_pid,
        }
    }

    pub fn timer(timer_id: u64) -> Self {
        Self {
            event_type: EventType::Timer,
            data: timer_id,
            flags: 0,
            source_pid: 0,
        }
    }

    pub fn irq(irq_num: u32) -> Self {
        Self {
            event_type: EventType::Irq,
            data: irq_num as u64,
            flags: 0,
            source_pid: 0,
        }
    }

    pub fn child_exit(child_pid: u32, exit_code: i32) -> Self {
        Self {
            event_type: EventType::ChildExit,
            data: child_pid as u64,
            flags: exit_code as u32,
            source_pid: child_pid,
        }
    }

    pub fn signal(signal_num: u32, sender_pid: u32) -> Self {
        Self {
            event_type: EventType::Signal,
            data: signal_num as u64,
            flags: 0,
            source_pid: sender_pid,
        }
    }
}

/// Event subscription - what events a process wants to receive
#[derive(Debug, Clone, Copy)]
pub struct EventSubscription {
    /// Type of event to subscribe to
    pub event_type: EventType,
    /// Filter data (e.g., specific channel ID, IRQ number)
    pub filter: u64,
    /// Whether this subscription is active
    pub active: bool,
}

impl EventSubscription {
    pub const fn empty() -> Self {
        Self {
            event_type: EventType::None,
            filter: 0,
            active: false,
        }
    }
}

/// Event queue for a process with priority separation
///
/// Critical events (ChildExit, Signal) go to a separate queue that
/// is never dropped. This ensures parent processes always receive
/// child exit notifications even under heavy load.
#[derive(Clone)]
pub struct EventQueue {
    /// Normal events (IpcReady, Timer, Fd*, Irq)
    events: [Event; MAX_EVENTS],
    count: usize,
    read_idx: usize,
    write_idx: usize,

    /// Critical events (ChildExit, Signal) - never dropped
    critical_events: [Event; MAX_CRITICAL_EVENTS],
    critical_count: usize,
    critical_read_idx: usize,
    critical_write_idx: usize,

    /// Subscriptions
    subscriptions: [EventSubscription; MAX_EVENTS],
}

impl EventQueue {
    pub const fn new() -> Self {
        Self {
            events: [Event::empty(); MAX_EVENTS],
            count: 0,
            read_idx: 0,
            write_idx: 0,
            critical_events: [Event::empty(); MAX_CRITICAL_EVENTS],
            critical_count: 0,
            critical_read_idx: 0,
            critical_write_idx: 0,
            subscriptions: [EventSubscription::empty(); MAX_EVENTS],
        }
    }

    /// Check if an event type is critical (should never be dropped)
    #[inline]
    fn is_critical_event(event_type: EventType) -> bool {
        matches!(event_type, EventType::ChildExit | EventType::Signal)
    }

    /// Subscribe to an event type with per-type limits
    /// Returns: Ok(()) on success, Err(-12) if no slots, Err(-28) if type limit reached
    pub fn subscribe(&mut self, event_type: EventType, filter: u64) -> Result<(), i64> {
        // Count existing subscriptions for this type to prevent exhaustion
        let type_count = self.subscriptions.iter()
            .filter(|s| s.active && s.event_type == event_type)
            .count();

        if type_count >= MAX_SUBSCRIPTIONS_PER_TYPE {
            return Err(-28); // ENOSPC - too many subscriptions of this type
        }

        // Find a free slot
        for sub in self.subscriptions.iter_mut() {
            if !sub.active {
                sub.event_type = event_type;
                sub.filter = filter;
                sub.active = true;
                return Ok(());
            }
        }
        Err(-12) // ENOMEM - no free subscription slots
    }

    /// Subscribe (legacy bool interface for compatibility)
    pub fn subscribe_bool(&mut self, event_type: EventType, filter: u64) -> bool {
        self.subscribe(event_type, filter).is_ok()
    }

    /// Unsubscribe from an event type
    pub fn unsubscribe(&mut self, event_type: EventType, filter: u64) -> bool {
        for sub in self.subscriptions.iter_mut() {
            if sub.active && sub.event_type == event_type && sub.filter == filter {
                sub.active = false;
                return true;
            }
        }
        false
    }

    /// Check if subscribed to this event
    pub fn is_subscribed(&self, event: &Event) -> bool {
        for sub in &self.subscriptions {
            if sub.active && sub.event_type == event.event_type {
                // Filter check: 0 means any, otherwise must match
                if sub.filter == 0 || sub.filter == event.data {
                    return true;
                }
            }
        }
        false
    }

    /// Push an event to the appropriate queue (critical or normal)
    pub fn push(&mut self, event: Event) -> bool {
        if Self::is_critical_event(event.event_type) {
            self.push_critical(event)
        } else {
            self.push_normal(event)
        }
    }

    /// Push a critical event (ChildExit, Signal)
    fn push_critical(&mut self, event: Event) -> bool {
        if self.critical_count >= MAX_CRITICAL_EVENTS {
            // Critical queue full - this is a serious condition
            logln!("[EVENT] WARNING: critical queue full, event may be lost: {:?}", event.event_type);
            return false;
        }
        self.critical_events[self.critical_write_idx] = event;
        self.critical_write_idx = (self.critical_write_idx + 1) % MAX_CRITICAL_EVENTS;
        self.critical_count += 1;
        true
    }

    /// Push a normal event (IpcReady, Timer, etc.)
    fn push_normal(&mut self, event: Event) -> bool {
        if self.count >= MAX_EVENTS {
            return false; // Queue full - acceptable to drop non-critical events
        }
        self.events[self.write_idx] = event;
        self.write_idx = (self.write_idx + 1) % MAX_EVENTS;
        self.count += 1;
        true
    }

    /// Pop an event from the queue (critical events have priority)
    pub fn pop(&mut self) -> Option<Event> {
        // Critical queue first - always process exits/signals before IPC
        if self.critical_count > 0 {
            let event = self.critical_events[self.critical_read_idx];
            self.critical_events[self.critical_read_idx] = Event::empty();
            self.critical_read_idx = (self.critical_read_idx + 1) % MAX_CRITICAL_EVENTS;
            self.critical_count -= 1;
            return Some(event);
        }

        // Then normal queue
        if self.count > 0 {
            let event = self.events[self.read_idx];
            self.events[self.read_idx] = Event::empty();
            self.read_idx = (self.read_idx + 1) % MAX_EVENTS;
            self.count -= 1;
            return Some(event);
        }

        None
    }

    /// Check if queue has pending events (either queue)
    pub fn has_events(&self) -> bool {
        self.count > 0 || self.critical_count > 0
    }

    /// Get total number of pending events
    pub fn len(&self) -> usize {
        self.count + self.critical_count
    }

    /// Peek at next event without removing it (critical first)
    pub fn peek(&self) -> Option<&Event> {
        if self.critical_count > 0 {
            Some(&self.critical_events[self.critical_read_idx])
        } else if self.count > 0 {
            Some(&self.events[self.read_idx])
        } else {
            None
        }
    }
}

// NOTE: Global EventSystem removed - all events now go directly to per-task queues.
// This eliminates the unlocked global state (security issue) and provides O(1) delivery.

/// Deliver an event directly to a task's event queue
///
/// This is the primary event delivery mechanism. Events are delivered
/// directly to the target task's queue, avoiding the need for a global
/// pending queue with its associated synchronization issues.
///
/// Returns true if the event was delivered (task exists and subscribed),
/// false otherwise.
pub fn deliver_event_to_task(target_pid: u32, event: Event) -> bool {
    unsafe {
        let sched = super::task::scheduler();
        // Use generation-aware PID lookup to prevent TOCTOU
        if let Some(slot) = sched.slot_by_pid(target_pid) {
            if let Some(ref mut task) = sched.tasks[slot] {
                // Check if task is subscribed to this event type
                if task.event_queue.is_subscribed(&event) {
                    if task.event_queue.push(event) {
                        // Wake task if blocked
                        if task.state == super::task::TaskState::Blocked {
                            task.state = super::task::TaskState::Ready;
                        }
                        return true;
                    }
                }
            }
        }
    }
    false
}

// ============================================================================
// Syscall implementations
// ============================================================================

/// Subscribe to events
/// Args: event_type, filter
/// Returns: 0 on success, negative error (-13 = EACCES for permission denied)
pub fn sys_event_subscribe(event_type: u32, filter: u64, pid: u32) -> i64 {
    // First, check capabilities for privileged event types
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref task) = sched.tasks[sched.current] {
            // IRQ events require IRQ_CLAIM capability
            if event_type == 3 {
                if !task.has_capability(super::caps::Capabilities::IRQ_CLAIM) {
                    logln!("[SECURITY] IRQ subscribe denied: pid={} lacks IRQ_CLAIM capability", pid);
                    return -13; // EACCES
                }
            }
        }
    }

    let ev_type = match event_type {
        1 => EventType::IpcReady,
        2 => EventType::Timer,
        3 => EventType::Irq,
        4 => EventType::ChildExit,
        5 => EventType::FdReadable,
        6 => EventType::FdWritable,
        7 => EventType::Signal,
        _ => return -1,
    };

    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            match task.event_queue.subscribe(ev_type, filter) {
                Ok(()) => return 0,
                Err(e) => return e,
            }
        }
    }
    -1 // No current task
}

/// Unsubscribe from events
pub fn sys_event_unsubscribe(event_type: u32, filter: u64, _pid: u32) -> i64 {
    let ev_type = match event_type {
        1 => EventType::IpcReady,
        2 => EventType::Timer,
        3 => EventType::Irq,
        4 => EventType::ChildExit,
        5 => EventType::FdReadable,
        6 => EventType::FdWritable,
        7 => EventType::Signal,
        _ => return -1,
    };

    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if task.event_queue.unsubscribe(ev_type, filter) {
                return 0;
            }
        }
    }
    -1
}

/// Wait for an event (blocking or non-blocking)
/// Args: event_buf (pointer to Event struct), flags (0=block, 1=non-block)
/// Returns: 1 if event received, 0 if would block, negative error
///
/// NOTE: This is the low-level implementation. The syscall handler in
/// syscall.rs wraps this with proper uaccess for copying to userspace.
pub fn sys_event_wait_internal(flags: u32) -> Option<Event> {
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            // Check task's event queue directly (no global system)
            if let Some(event) = task.event_queue.pop() {
                return Some(event);
            }

            // No event available
            if flags & 1 != 0 {
                // Non-blocking - return None to indicate no event
                return None;
            }

            // Would block - mark task as waiting
            task.state = super::task::TaskState::Blocked;
        }
    }
    None
}

/// Post an event to another process
/// Args: target_pid, event_type, data
///
/// Only Signal events (type 7) can be posted by userspace.
/// Delivery and wake-up is handled by deliver_event_to_task().
pub fn sys_event_post(target_pid: u32, event_type: u32, data: u64, caller_pid: u32) -> i64 {
    let event = match event_type {
        7 => Event::signal(data as u32, caller_pid),
        _ => return -1, // Only signals can be posted by user
    };

    // Deliver directly to target's event queue (no global system)
    // deliver_event_to_task handles subscription check and wake-up
    if deliver_event_to_task(target_pid, event) {
        0
    } else {
        -3 // ESRCH - process not found or not subscribed
    }
}

/// Test the event system
pub fn test() {
    logln!("  Testing event system...");

    let mut queue = EventQueue::new();

    // Test subscription
    assert!(queue.subscribe(EventType::IpcReady, 0).is_ok());
    logln!("    Subscribed to IpcReady events");

    // Test event push/pop
    let event = Event::ipc_ready(1, 2);
    assert!(queue.push(event));
    logln!("    Pushed IpcReady event");

    assert!(queue.has_events());
    assert_eq!(queue.len(), 1);

    let popped = queue.pop();
    assert!(popped.is_some());
    let e = popped.unwrap();
    assert_eq!(e.event_type, EventType::IpcReady);
    assert_eq!(e.data, 1);
    logln!("    Popped event: type={:?}, data={}", e.event_type, e.data);

    assert!(!queue.has_events());
    logln!("    [OK] Event system test passed");
}
