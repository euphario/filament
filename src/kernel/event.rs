//! Event System

#![allow(dead_code)]  // Infrastructure for future use
//!
//! Provides async notification mechanism for processes.
//! Similar to Redox's event system - processes can wait on multiple events.

use crate::{kwarn, print_direct};

// ============================================================================
// Debug Assertions
// ============================================================================

/// Assert that IRQs are disabled (for SMP safety)
/// This ensures that queue operations are not interrupted.
#[cfg(debug_assertions)]
#[inline]
fn assert_irqs_disabled() {
    let daif: u64;
    unsafe { core::arch::asm!("mrs {}, daif", out(reg) daif); }
    debug_assert!(
        (daif & (1 << 7)) != 0,
        "IRQs must be disabled during event queue operations"
    );
}

/// No-op in release builds
#[cfg(not(debug_assertions))]
#[inline]
fn assert_irqs_disabled() {}

/// Maximum events per process in normal queue
pub const MAX_EVENTS: usize = 24;

/// Maximum events in critical queue (ChildExit, Signal - never dropped)
pub const MAX_CRITICAL_EVENTS: usize = 8;

/// Maximum subscriptions per event type (prevents exhaustion attacks)
/// Scales with MAX_BUSES: devd needs (buses + 4) IpcReady subscriptions
/// Formula: MAX_BUSES * 2 + 8 provides headroom for multiple subscribers
pub const MAX_SUBSCRIPTIONS_PER_TYPE: usize = super::bus::MAX_BUSES * 2 + 8;

/// Event types (legacy, used internally)
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
    /// Kernel log available (for logd)
    KlogReady = 8,
    /// Shared memory region is being destroyed (owner died)
    ShmemInvalid = 9,
}

/// Unified event filter - what to wait for (BSD kqueue-inspired)
///
/// Used for kevent_subscribe() and kevent_wait() APIs.
/// The u32 parameter is a filter-specific identifier (channel_id, timer_id, etc.)
/// A value of 0 typically means "any" (any timer, any child, any signal).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventFilter {
    /// IPC channel has message (channel_id)
    Ipc(u32),
    /// Timer expired (timer_id, 0 = any timer)
    Timer(u32),
    /// IRQ occurred (irq_num)
    Irq(u32),
    /// Child process exited (pid, 0 = any child)
    ChildExit(u32),
    /// File descriptor readable (fd)
    Read(u32),
    /// File descriptor writable (fd)
    Write(u32),
    /// Signal received (signal_num, 0 = any)
    Signal(u32),
    /// Kernel log available
    KlogReady,
    /// Shared memory region is being destroyed (shmem_id)
    ShmemInvalid(u32),
}

impl EventFilter {
    /// Convert to EventType for internal matching
    pub fn to_event_type(&self) -> EventType {
        match self {
            EventFilter::Ipc(_) => EventType::IpcReady,
            EventFilter::Timer(_) => EventType::Timer,
            EventFilter::Irq(_) => EventType::Irq,
            EventFilter::ChildExit(_) => EventType::ChildExit,
            EventFilter::Read(_) => EventType::FdReadable,
            EventFilter::Write(_) => EventType::FdWritable,
            EventFilter::Signal(_) => EventType::Signal,
            EventFilter::KlogReady => EventType::KlogReady,
            EventFilter::ShmemInvalid(_) => EventType::ShmemInvalid,
        }
    }

    /// Get the filter value (id/fd/etc.)
    pub fn filter_value(&self) -> u64 {
        match self {
            EventFilter::Ipc(id) => *id as u64,
            EventFilter::Timer(id) => *id as u64,
            EventFilter::Irq(num) => *num as u64,
            EventFilter::ChildExit(pid) => *pid as u64,
            EventFilter::Read(fd) => *fd as u64,
            EventFilter::Write(fd) => *fd as u64,
            EventFilter::Signal(num) => *num as u64,
            EventFilter::KlogReady => 0,
            EventFilter::ShmemInvalid(id) => *id as u64,
        }
    }

    /// Create from type and value (for syscall deserialization)
    /// type: 1=Ipc, 2=Timer, 3=Irq, 4=ChildExit, 5=Read, 6=Write, 7=Signal, 8=KlogReady
    pub fn from_type_and_value(filter_type: u32, filter_value: u32) -> Option<Self> {
        match filter_type {
            1 => Some(EventFilter::Ipc(filter_value)),
            2 => Some(EventFilter::Timer(filter_value)),
            3 => Some(EventFilter::Irq(filter_value)),
            4 => Some(EventFilter::ChildExit(filter_value)),
            5 => Some(EventFilter::Read(filter_value)),
            6 => Some(EventFilter::Write(filter_value)),
            7 => Some(EventFilter::Signal(filter_value)),
            8 => Some(EventFilter::KlogReady),
            9 => Some(EventFilter::ShmemInvalid(filter_value)),
            _ => None,
        }
    }
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

    /// Create a timer event (legacy: data = deadline)
    pub fn timer(deadline: u64) -> Self {
        Self {
            event_type: EventType::Timer,
            data: deadline,
            flags: 0,
            source_pid: 0,
        }
    }

    /// Create a timer event with explicit ID (kevent style)
    /// data = timer_id (1-8), flags stores low 32 bits of deadline
    pub fn timer_with_id(timer_id: u32, deadline: u64) -> Self {
        Self {
            event_type: EventType::Timer,
            data: timer_id as u64,
            flags: deadline as u32,  // Low 32 bits of deadline for debugging
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

    pub fn klog_ready() -> Self {
        Self {
            event_type: EventType::KlogReady,
            data: 0,
            flags: 0,
            source_pid: 0,
        }
    }

    /// Shared memory region is being destroyed (owner died)
    /// data = shmem_id, source_pid = owner_pid who is dying
    pub fn shmem_invalid(shmem_id: u32, owner_pid: u32) -> Self {
        Self {
            event_type: EventType::ShmemInvalid,
            data: shmem_id as u64,
            flags: 0,
            source_pid: owner_pid,
        }
    }

    pub fn fd_readable(fd: u64) -> Self {
        Self {
            event_type: EventType::FdReadable,
            data: fd,
            flags: 0,
            source_pid: 0,
        }
    }
}

/// Subscription state - explicit enum instead of bool
///
/// This makes the state machine clearer than a simple `active: bool`:
/// - Empty: Slot available for new subscription
/// - Active: Listening for events
///
/// Future extensions could add:
/// - OneShot: Auto-unsubscribe after first event
/// - Paused: Temporarily disabled
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubscriptionState {
    /// Slot not in use
    Empty,
    /// Subscription is active, listening for events
    Active,
}

impl SubscriptionState {
    /// Valid state transitions
    pub fn can_transition_to(&self, new: SubscriptionState) -> bool {
        match (*self, new) {
            (SubscriptionState::Empty, SubscriptionState::Active) => true,
            (SubscriptionState::Active, SubscriptionState::Empty) => true,
            _ => false,
        }
    }

    /// Check if this is an empty slot
    pub fn is_empty(&self) -> bool {
        matches!(self, SubscriptionState::Empty)
    }

    /// Check if this is an active subscription
    pub fn is_active(&self) -> bool {
        matches!(self, SubscriptionState::Active)
    }

    /// Get human-readable state name
    pub fn name(&self) -> &'static str {
        match self {
            SubscriptionState::Empty => "empty",
            SubscriptionState::Active => "active",
        }
    }
}

impl super::ipc::traits::StateMachine for SubscriptionState {
    fn can_transition_to(&self, new: &Self) -> bool {
        SubscriptionState::can_transition_to(self, *new)
    }

    fn name(&self) -> &'static str {
        SubscriptionState::name(self)
    }
}

impl Default for SubscriptionState {
    fn default() -> Self {
        SubscriptionState::Empty
    }
}

/// Event subscription - what events a process wants to receive
#[derive(Debug, Clone, Copy)]
pub struct EventSubscription {
    /// Type of event to subscribe to
    pub event_type: EventType,
    /// Filter data (e.g., specific channel ID, IRQ number)
    pub filter: u64,
    /// Subscription state - private, use state() and transition methods
    state: SubscriptionState,
}

impl EventSubscription {
    /// Get current state
    pub fn state(&self) -> SubscriptionState {
        self.state
    }
}

impl EventSubscription {
    pub const fn empty() -> Self {
        Self {
            event_type: EventType::None,
            filter: 0,
            state: SubscriptionState::Empty,
        }
    }

    /// Check if this subscription is active
    #[inline]
    pub fn is_active(&self) -> bool {
        self.state.is_active()
    }

    /// Activate this subscription (Empty -> Active)
    /// Returns true if transition succeeded
    pub fn activate(&mut self, event_type: EventType, filter: u64) -> bool {
        if self.state.can_transition_to(SubscriptionState::Active) {
            self.event_type = event_type;
            self.filter = filter;
            self.state = SubscriptionState::Active;
            true
        } else {
            false
        }
    }

    /// Deactivate this subscription (Active -> Empty)
    /// Returns true if transition succeeded
    pub fn deactivate(&mut self) -> bool {
        if self.state.can_transition_to(SubscriptionState::Empty) {
            self.state = SubscriptionState::Empty;
            true
        } else {
            false
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

    /// Subscriptions (public for liveness checker)
    pub subscriptions: [EventSubscription; MAX_EVENTS],
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
            .filter(|s| s.is_active() && s.event_type == event_type)
            .count();

        if type_count >= MAX_SUBSCRIPTIONS_PER_TYPE {
            return Err(-28); // ENOSPC - too many subscriptions of this type
        }

        // Find a free slot and activate using transition method
        for sub in self.subscriptions.iter_mut() {
            if sub.state.is_empty() {
                if sub.activate(event_type, filter) {
                    return Ok(());
                }
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
            if sub.is_active() && sub.event_type == event_type && sub.filter == filter {
                // Use transition method to deactivate
                return sub.deactivate();
            }
        }
        false
    }

    /// Check if subscribed to this event
    pub fn is_subscribed(&self, event: &Event) -> bool {
        for sub in &self.subscriptions {
            if sub.is_active() && sub.event_type == event.event_type {
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
            kwarn!("event", "critical_queue_full"; event_type = event.event_type as u64);
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

    /// Check for a pending ChildExit event matching a specific child PID
    ///
    /// child_pid: PID to look for (0 = any child)
    /// Returns Some((child_pid, exit_code)) if found, None otherwise.
    /// Does NOT consume the event - caller must call pop_child_exit() to consume.
    pub fn peek_child_exit(&self, child_pid: u32) -> Option<(u32, i32)> {
        if self.critical_count == 0 {
            return None;
        }

        // Scan the critical queue for matching ChildExit events
        let mut idx = self.critical_read_idx;
        for _ in 0..self.critical_count {
            let event = &self.critical_events[idx];
            if event.event_type == EventType::ChildExit {
                let exit_pid = event.data as u32;
                let exit_code = event.flags as i32;

                // Match if watching this specific child or any child
                if child_pid == 0 || child_pid == exit_pid {
                    return Some((exit_pid, exit_code));
                }
            }
            idx = (idx + 1) % MAX_CRITICAL_EVENTS;
        }
        None
    }

    /// Pop (consume) a pending ChildExit event matching a specific child PID
    ///
    /// child_pid: PID to look for (0 = any child)
    /// Returns Some((child_pid, exit_code)) if found and consumed, None otherwise.
    pub fn pop_child_exit(&mut self, child_pid: u32) -> Option<(u32, i32)> {
        if self.critical_count == 0 {
            return None;
        }

        // Scan the critical queue for matching ChildExit events
        let mut idx = self.critical_read_idx;
        for i in 0..self.critical_count {
            let event = &self.critical_events[idx];
            if event.event_type == EventType::ChildExit {
                let exit_pid = event.data as u32;
                let exit_code = event.flags as i32;

                // Match if watching this specific child or any child
                if child_pid == 0 || child_pid == exit_pid {
                    // Remove this event by shifting subsequent events
                    // This is O(n) but critical queue is small (4 elements)
                    let mut src = idx;
                    for _ in i..self.critical_count - 1 {
                        let next = (src + 1) % MAX_CRITICAL_EVENTS;
                        self.critical_events[src] = self.critical_events[next];
                        src = next;
                    }
                    self.critical_count -= 1;
                    // Adjust write index
                    if self.critical_count == 0 {
                        self.critical_write_idx = self.critical_read_idx;
                    } else {
                        self.critical_write_idx = (self.critical_read_idx + self.critical_count) % MAX_CRITICAL_EVENTS;
                    }
                    return Some((exit_pid, exit_code));
                }
            }
            idx = (idx + 1) % MAX_CRITICAL_EVENTS;
        }
        None
    }

    /// Check for a pending ShmemInvalid event matching a specific shmem ID
    ///
    /// shmem_id: ID to look for (0 = any shmem)
    /// Returns Some((shmem_id, owner_pid)) if found, None otherwise.
    /// Does NOT consume the event - caller must call pop_next() to consume.
    pub fn peek_shmem_invalid(&self, shmem_id: u32) -> Option<(u32, u32)> {
        // ShmemInvalid events go to the normal queue, not critical queue
        if self.count == 0 {
            return None;
        }

        // Scan the normal queue for matching ShmemInvalid events
        let mut idx = self.read_idx;
        for _ in 0..self.count {
            let event = &self.events[idx];
            if event.event_type == EventType::ShmemInvalid {
                let found_id = event.data as u32;
                let owner_pid = event.source_pid;

                // Match if watching this specific shmem or any shmem
                if shmem_id == 0 || shmem_id == found_id {
                    return Some((found_id, owner_pid));
                }
            }
            idx = (idx + 1) % MAX_EVENTS;
        }
        None
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
///
/// # Deadlock Prevention
///
/// Uses try_scheduler() to prevent deadlock if called from IRQ context
/// while the scheduler lock is already held. If the lock can't be acquired,
/// queues a deferred wake request instead.
pub fn deliver_event_to_task(target_pid: u32, event: Event) -> bool {
    // Ensure IRQs are disabled for scheduler access
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    // Use try_scheduler to prevent deadlock if called from IRQ context
    let Some(mut sched) = super::task::try_scheduler() else {
        // Lock held - queue deferred wake and return.
        // The event won't be delivered this time, but task will wake
        // and can poll for events.
        crate::arch::aarch64::sync::cpu_flags().request_wake(target_pid);
        return false;
    };

    // Use generation-aware PID lookup to prevent TOCTOU
    if let Some(slot) = sched.slot_by_pid(target_pid) {
        if let Some(task) = sched.task_mut(slot) {
            // Check if task is subscribed to this event type
            if task.event_queue.is_subscribed(&event) {
                if task.event_queue.push(event) {
                    // Wake task if blocked (using unified wake function)
                    sched.wake_task(slot);
                    return true;
                }
            }
        }
    }
    false
}

/// Broadcast an event to all subscribed tasks
///
/// Used for system-wide events like KlogReady where multiple processes
/// may want to receive the notification.
///
/// Returns the number of tasks that received the event.
///
/// # Deadlock Prevention
///
/// Uses try_lock() instead of lock() because this function is called from
/// LOG_RING.write() during logging. If the scheduler lock is already held
/// (e.g., during spawn or other scheduler operations that log), we skip
/// the broadcast rather than deadlock. The log is still written; tasks
/// just won't be immediately woken (they'll see it on their next poll).
pub fn broadcast_event(event: Event) -> usize {
    // Use try_lock to prevent deadlock when logging inside scheduler lock
    let Some(mut sched) = super::task::try_scheduler() else {
        // Scheduler lock held - skip broadcast to prevent deadlock
        // Event is still logged, just not broadcast to waiting tasks
        return 0;
    };

    let mut delivered = 0;
    for (_slot, task_opt) in sched.iter_tasks_mut() {
        if let Some(task) = task_opt {
            // Check if task is subscribed to this event type
            if task.event_queue.is_subscribed(&event) {
                if task.event_queue.push(event) {
                    // Wake task if blocked
                    if task.is_blocked() {
                        crate::transition_or_log!(task, wake);
                        // Reset liveness - task received event
                        task.liveness_state = super::liveness::LivenessState::Normal;
                    }
                    delivered += 1;
                }
            }
        }
    }
    delivered
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
        if let Some(task) = sched.current_task() {
            // IRQ events require IRQ_CLAIM capability
            if event_type == 3 {
                if !task.has_capability(super::caps::Capabilities::IRQ_CLAIM) {
                    super::security_log::log_capability_denied(pid, "IRQ_CLAIM", "event_subscribe");
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
        8 => EventType::KlogReady,
        _ => return -1,
    };

    unsafe {
        let mut sched = super::task::scheduler();
        if let Some(task) = sched.current_task_mut() {
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
        8 => EventType::KlogReady,
        _ => return -1,
    };

    unsafe {
        let mut sched = super::task::scheduler();
        if let Some(task) = sched.current_task_mut() {
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
        let mut sched = super::task::scheduler();
        if let Some(task) = sched.current_task_mut() {
            // Check task's event queue directly (no global system)
            if let Some(event) = task.event_queue.pop() {
                return Some(event);
            }

            // No event available
            if flags & 1 != 0 {
                // Non-blocking - return None to indicate no event
                return None;
            }

            // Would block - mark task as sleeping on event loop
            let _ = task.set_sleeping(super::task::SleepReason::EventLoop);
        }
    }
    None
}

/// Post an event to another process
/// Args: target_pid, event_type, data
///
/// Only Signal events (type 7) can be posted by userspace.
/// Signal delivery is subject to the receiver's allowlist (if set).
/// Delivery and wake-up is handled by deliver_event_to_task().
pub fn sys_event_post(target_pid: u32, event_type: u32, data: u64, caller_pid: u32) -> i64 {
    // Only signals can be posted by userspace
    if event_type != 7 {
        return -1;
    }

    // Disable IRQs for entire operation (allowlist check + delivery)
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();

    // Check signal allowlist before delivering
    unsafe {
        let sched = super::task::scheduler();
        if let Some(slot) = sched.slot_by_pid(target_pid) {
            if let Some(task) = sched.task(slot) {
                if !task.can_receive_signal_from(caller_pid) {
                    super::security_log::log_signal_blocked(target_pid, caller_pid);
                    return -13; // EACCES - sender not in allowlist
                }
            }
        } else {
            return -3; // ESRCH - process not found
        }
    }

    let event = Event::signal(data as u32, caller_pid);

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
    print_direct!("  Testing event system...\n");

    let mut queue = EventQueue::new();

    // Test subscription
    assert!(queue.subscribe(EventType::IpcReady, 0).is_ok());
    print_direct!("    Subscribed to IpcReady events\n");

    // Test event push/pop
    let event = Event::ipc_ready(1, 2);
    assert!(queue.push(event));
    print_direct!("    Pushed IpcReady event\n");

    assert!(queue.has_events());
    assert_eq!(queue.len(), 1);

    let popped = queue.pop();
    assert!(popped.is_some());
    let e = popped.unwrap();
    assert_eq!(e.event_type, EventType::IpcReady);
    assert_eq!(e.data, 1);
    print_direct!("    Popped event: type={:?}, data={}\n", e.event_type, e.data);

    assert!(!queue.has_events());
    print_direct!("    [OK] Event system test passed\n");
}
