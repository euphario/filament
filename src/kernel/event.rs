//! Event System

#![allow(dead_code)]  // Infrastructure for future use
//!
//! Provides async notification mechanism for processes.
//! Similar to Redox's event system - processes can wait on multiple events.

use crate::logln;

/// Maximum events per process
pub const MAX_EVENTS: usize = 32;

/// Maximum pending events in global queue
pub const MAX_PENDING: usize = 64;

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

/// Event queue for a process
#[derive(Clone)]
pub struct EventQueue {
    /// Pending events
    events: [Event; MAX_EVENTS],
    /// Number of pending events
    count: usize,
    /// Read index (circular buffer)
    read_idx: usize,
    /// Write index
    write_idx: usize,
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
            subscriptions: [EventSubscription::empty(); MAX_EVENTS],
        }
    }

    /// Subscribe to an event type
    pub fn subscribe(&mut self, event_type: EventType, filter: u64) -> bool {
        for sub in self.subscriptions.iter_mut() {
            if !sub.active {
                sub.event_type = event_type;
                sub.filter = filter;
                sub.active = true;
                return true;
            }
        }
        false
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

    /// Push an event to the queue
    pub fn push(&mut self, event: Event) -> bool {
        if self.count >= MAX_EVENTS {
            return false; // Queue full
        }

        self.events[self.write_idx] = event;
        self.write_idx = (self.write_idx + 1) % MAX_EVENTS;
        self.count += 1;
        true
    }

    /// Pop an event from the queue
    pub fn pop(&mut self) -> Option<Event> {
        if self.count == 0 {
            return None;
        }

        let event = self.events[self.read_idx];
        self.events[self.read_idx] = Event::empty();
        self.read_idx = (self.read_idx + 1) % MAX_EVENTS;
        self.count -= 1;
        Some(event)
    }

    /// Check if queue has pending events
    pub fn has_events(&self) -> bool {
        self.count > 0
    }

    /// Get number of pending events
    pub fn len(&self) -> usize {
        self.count
    }

    /// Peek at next event without removing it
    pub fn peek(&self) -> Option<&Event> {
        if self.count == 0 {
            None
        } else {
            Some(&self.events[self.read_idx])
        }
    }
}

/// Global event delivery system
pub struct EventSystem {
    /// Pending events to be delivered (temporary holding)
    pending: [Event; MAX_PENDING],
    /// Target PIDs for pending events
    targets: [u32; MAX_PENDING],
    /// Number of pending deliveries
    pending_count: usize,
}

impl EventSystem {
    pub const fn new() -> Self {
        Self {
            pending: [Event::empty(); MAX_PENDING],
            targets: [0; MAX_PENDING],
            pending_count: 0,
        }
    }

    /// Queue an event for delivery to a specific process
    pub fn send_to(&mut self, pid: u32, event: Event) -> bool {
        if self.pending_count >= MAX_PENDING {
            return false;
        }

        self.pending[self.pending_count] = event;
        self.targets[self.pending_count] = pid;
        self.pending_count += 1;
        true
    }

    /// Deliver pending events to process queues
    /// Called from scheduler or syscall context
    pub fn deliver_pending(&mut self) {
        // This would iterate through pending events and deliver them
        // to the appropriate process event queues
        // For now, we'll handle this in the syscall layer
        self.pending_count = 0;
    }

    /// Get pending events for a specific PID
    pub fn get_pending_for(&mut self, pid: u32) -> Option<Event> {
        for i in 0..self.pending_count {
            if self.targets[i] == pid {
                let event = self.pending[i];
                // Remove from pending (shift remaining)
                for j in i..self.pending_count - 1 {
                    self.pending[j] = self.pending[j + 1];
                    self.targets[j] = self.targets[j + 1];
                }
                self.pending_count -= 1;
                return Some(event);
            }
        }
        None
    }
}

/// Global event system instance
static mut EVENT_SYSTEM: EventSystem = EventSystem::new();

/// Get the global event system
pub unsafe fn event_system() -> &'static mut EventSystem {
    &mut *core::ptr::addr_of_mut!(EVENT_SYSTEM)
}

/// Send an event to a process
pub fn send_event(pid: u32, event: Event) -> bool {
    unsafe { event_system().send_to(pid, event) }
}

// ============================================================================
// Syscall implementations
// ============================================================================

/// Subscribe to events
/// Args: event_type, filter
/// Returns: 0 on success, negative error
pub fn sys_event_subscribe(event_type: u32, filter: u64, _pid: u32) -> i64 {
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
            if task.event_queue.subscribe(ev_type, filter) {
                return 0;
            }
        }
    }
    -1
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
pub fn sys_event_wait(event_buf: u64, flags: u32) -> i64 {
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            // First check for pending events in global system
            let pid = task.id;
            if let Some(event) = event_system().get_pending_for(pid) {
                // Copy event to user buffer
                let buf = event_buf as *mut Event;
                core::ptr::write_volatile(buf, event);
                return 1;
            }

            // Check task's event queue
            if let Some(event) = task.event_queue.pop() {
                let buf = event_buf as *mut Event;
                core::ptr::write_volatile(buf, event);
                return 1;
            }

            // No event available
            if flags & 1 != 0 {
                // Non-blocking
                return 0;
            }

            // Would block - mark task as waiting
            task.state = super::task::TaskState::Blocked;
            // Scheduler will handle waking us up
            return -11; // EAGAIN - caller should yield
        }
    }
    -1
}

/// Post an event to another process
/// Args: target_pid, event_type, data
pub fn sys_event_post(target_pid: u32, event_type: u32, data: u64, caller_pid: u32) -> i64 {
    let event = match event_type {
        7 => Event::signal(data as u32, caller_pid),
        _ => return -1, // Only signals can be posted by user
    };

    if send_event(target_pid, event) {
        // Try to wake target if blocked
        unsafe {
            let sched = super::task::scheduler();
            for task_opt in sched.tasks.iter_mut() {
                if let Some(ref mut task) = task_opt {
                    if task.id == target_pid && task.state == super::task::TaskState::Blocked {
                        task.state = super::task::TaskState::Ready;
                        break;
                    }
                }
            }
        }
        0
    } else {
        -1
    }
}

/// Test the event system
pub fn test() {
    logln!("  Testing event system...");

    let mut queue = EventQueue::new();

    // Test subscription
    assert!(queue.subscribe(EventType::IpcReady, 0));
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
