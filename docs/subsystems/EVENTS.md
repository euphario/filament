# Unified Event System

## Overview

The kernel provides a BSD kqueue-inspired unified event system. Tasks can subscribe to various event types and wait for any of them in a single blocking call.

## Design Intent

The unified event system replaces fragmented APIs with a single, consistent interface:

- **One wait call** - `kevent_wait()` returns any subscribed event
- **Multiple timers** - Each task can have up to 8 independent timers
- **Batch processing** - Receive multiple events in one syscall
- **Type-safe subscriptions** - EventFilter enum prevents errors

## Event Types

| Type | Description | Filter Parameter |
|------|-------------|------------------|
| IpcReady | Message available on channel | Channel ID |
| Timer | Timer expired | Timer ID (0-7) |
| Irq | Hardware interrupt occurred | IRQ number |
| ChildExit | Child process exited | Child PID (0 = any) |
| FdReadable | File descriptor readable | FD number |
| FdWritable | File descriptor writable | FD number |
| Signal | Signal received | Signal number (0 = any) |
| KlogReady | Kernel log available | N/A |

## Syscalls

### kevent_subscribe (syscall 70)

Subscribe to an event type:

```rust
pub fn kevent_subscribe(filter: EventFilter) -> Result<(), i32>;

// Examples
kevent_subscribe(EventFilter::Ipc(channel_id))?;
kevent_subscribe(EventFilter::ChildExit(0))?;  // Any child
kevent_subscribe(EventFilter::Timer(1))?;       // Timer slot 1
```

### kevent_unsubscribe (syscall 71)

Remove a subscription:

```rust
pub fn kevent_unsubscribe(filter: EventFilter) -> Result<(), i32>;
```

### kevent_timer (syscall 72)

Set or cancel a timer:

```rust
pub fn kevent_timer(id: u32, interval_ns: u64, initial_ns: u64) -> i32;

// One-shot timer in slot 0, fires after 1 second
kevent_timer(0, 0, 1_000_000_000);

// Recurring timer in slot 1, fires every 500ms
kevent_timer(1, 500_000_000, 500_000_000);

// Cancel timer in slot 1
kevent_timer(1, 0, 0);
```

Parameters:
- `id`: Timer slot (0-7)
- `interval_ns`: Recurring interval (0 = one-shot)
- `initial_ns`: Time until first fire (0 = use interval)

### kevent_wait (syscall 73)

Wait for events:

```rust
pub fn kevent_wait(events: &mut [Event], timeout_ns: u64) -> Result<usize, i32>;

// Block forever until event
let count = kevent_wait(&mut events, u64::MAX)?;

// Poll (non-blocking)
let count = kevent_wait(&mut events, 0)?;

// Wait up to 100ms
let count = kevent_wait(&mut events, 100_000_000)?;
```

Returns the number of events received.

## Event Structure

```rust
#[repr(C)]
pub struct Event {
    pub event_type: u32,    // event_type::* constant
    pub _pad: u32,
    pub data: u64,          // Channel ID, timer ID, child PID, etc.
    pub flags: u32,         // Exit code for ChildExit, etc.
    pub source_pid: u32,    // Source PID for IPC/signals
}
```

## EventFilter Enum

```rust
pub enum EventFilter {
    Ipc(u32),       // IPC channel ID
    Timer(u32),     // Timer slot 0-7
    Irq(u32),       // IRQ number
    ChildExit(u32), // Child PID (0 = any)
    Read(u32),      // File descriptor
    Write(u32),     // File descriptor
    Signal(u32),    // Signal number (0 = any)
    KlogReady,      // Kernel log ready
}
```

## Usage Pattern

```rust
// Initialize
kevent_subscribe(EventFilter::Ipc(my_channel))?;
kevent_subscribe(EventFilter::ChildExit(0))?;
kevent_timer(0, 0, 5_000_000_000)?;  // 5 second timeout

// Main loop
let mut events = [Event::empty(); 8];
loop {
    let count = kevent_wait(&mut events, u64::MAX)?;

    for event in &events[..count] {
        match event.event_type {
            event_type::IPC_READY => {
                let channel = event.data as u32;
                handle_message(channel);
            }
            event_type::CHILD_EXIT => {
                let pid = event.data as u32;
                let exit_code = event.flags as i32;
                handle_child_exit(pid, exit_code);
            }
            event_type::TIMER => {
                let timer_id = event.data as u32;
                handle_timeout(timer_id);
            }
            _ => {}
        }
    }
}
```

## Critical Events

Some events are never dropped, even under load:

- **ChildExit** - Always delivered (uses priority queue)
- **Signal** - Always delivered

These use a separate 8-slot critical event queue that takes priority over normal events.

## Timer Precision

- ARM generic timer runs at 24MHz (~41ns resolution)
- Practical precision: microseconds
- Recurring timers auto-reset after each fire

## Comparison with Legacy API

| Old API | New API | Notes |
|---------|---------|-------|
| `event_subscribe(type, filter)` | `kevent_subscribe(EventFilter::*)` | Type-safe |
| `timer_set(duration)` | `kevent_timer(0, 0, duration)` | Multiple timers |
| `event_wait(&event, flags)` | `kevent_wait(&events, timeout)` | Batch receive |

The old APIs remain for backwards compatibility but are deprecated.

## Design Rationale

### Why Unified?

1. **Single wait point** - No need to poll multiple sources
2. **Atomicity** - All events handled in one wakeup
3. **Efficiency** - Fewer context switches

### Why Multiple Timers?

Drivers often need multiple independent timeouts:
- Connection timeout
- Periodic heartbeat
- Hardware operation timeout

With one timer per task, these had to be multiplexed manually.

### Why Batch Receive?

Under load, multiple events may be pending. Batch receive:
- Reduces syscall overhead
- Allows prioritization in userspace
- Handles bursts efficiently
