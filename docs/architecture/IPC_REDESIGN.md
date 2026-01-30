# IPC Redesign: Zero-Copy Command Rings

## Current Problem

Every IPC message requires:
```
serialize → syscall(write) → kernel copy → syscall(read) → deserialize
```

This is slow and error-prone. WouldBlock confusion, complex state machines.

## New Design: Shared Memory Command Rings

**Core insight:** Channels are for *waiting*, shared memory is for *data*.

```
┌─────────────────────────────────────────────────────────────────┐
│                        Shared Memory                            │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  Command Ring (devd → driver)                           │   │
│  │  [Cmd][Cmd][Cmd][Cmd][Cmd][Cmd][Cmd][Cmd]              │   │
│  │       ↑                   ↑                             │   │
│  │      tail               head                            │   │
│  └─────────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  Response Ring (driver → devd)                          │   │
│  │  [Rsp][Rsp][Rsp][Rsp][Rsp][Rsp][Rsp][Rsp]              │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
        ↑                                           ↑
        │ mmap                                      │ mmap
   ┌────┴────┐                                ┌────┴────┐
   │  devd   │ ←── wake channel (empty) ────→ │ driver  │
   └─────────┘                                └─────────┘
```

## Data Structures

```rust
// Fixed-size command - NO SERIALIZATION
// This struct IS the wire format
#[repr(C, align(64))]  // Cache-line aligned
pub struct Command {
    pub cmd_type: u32,      // ATTACH_DISK, REGISTER_PARTITION, etc.
    pub seq_id: u32,        // For matching responses
    pub flags: u32,
    pub _pad: u32,
    pub payload: [u8; 48],  // Command-specific data, fixed size
}

// Fixed-size response
#[repr(C, align(64))]
pub struct Response {
    pub seq_id: u32,        // Matches command
    pub result: i32,        // 0 = success, negative = error
    pub payload: [u8; 56],  // Response data
}

// The shared ring
#[repr(C)]
pub struct CommandRing {
    pub head: AtomicU32,        // Producer (devd) writes here
    pub tail: AtomicU32,        // Consumer (driver) reads here
    pub wake_handle: u32,       // Channel handle for signaling
    pub _pad: u32,
    pub commands: [Command; RING_SIZE],
}

pub const RING_SIZE: usize = 16;  // Power of 2
```

## API

### Producer (devd) side:

```rust
impl CommandRing {
    /// Get next slot to write (None if ring full)
    pub fn try_reserve(&self) -> Option<&mut Command> {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);

        if head - tail >= RING_SIZE as u32 {
            return None;  // Ring full
        }

        let slot = &mut self.commands[(head as usize) % RING_SIZE];
        Some(slot)
    }

    /// Publish the command (makes it visible to consumer)
    pub fn publish(&self) {
        // Memory barrier: ensure command data is visible before head update
        self.head.fetch_add(1, Ordering::Release);
    }

    /// Wake the consumer (only if it's sleeping)
    pub fn notify(&self) {
        syscall::write(self.wake_handle, &[]);  // Empty write = wake
    }

    /// Reserve, blocking if full (with timeout for safety)
    pub fn reserve(&self, timeout_ms: u32) -> Result<&mut Command, Error> {
        loop {
            if let Some(slot) = self.try_reserve() {
                return Ok(slot);
            }
            // Ring full - wait for consumer (with timeout!)
            syscall::wait_with_timeout(self.wake_handle, timeout_ms)?;
        }
    }
}
```

### Consumer (driver) side:

```rust
impl CommandRing {
    /// Poll for next command (non-blocking)
    pub fn try_recv(&self) -> Option<&Command> {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);

        if tail == head {
            return None;  // Ring empty
        }

        Some(&self.commands[(tail as usize) % RING_SIZE])
    }

    /// Mark command as consumed
    pub fn consume(&self) {
        self.tail.fetch_add(1, Ordering::Release);
    }

    /// Wait for command (blocking)
    pub fn recv(&self) -> &Command {
        loop {
            if let Some(cmd) = self.try_recv() {
                return cmd;
            }
            // Ring empty - sleep until producer notifies
            syscall::wait(self.wake_handle);
        }
    }
}
```

## Usage Example: devd → partd

### devd sends AttachDisk:

```rust
// Get a slot (blocks if ring full, with timeout)
let cmd = ring.reserve(5000)?;  // 5 second timeout

// Fill in command - NO SERIALIZATION, just write fields
cmd.cmd_type = CMD_ATTACH_DISK;
cmd.seq_id = self.next_seq();
cmd.payload[0..4].copy_from_slice(&shmem_id.to_le_bytes());
cmd.payload[4..8].copy_from_slice(&block_size.to_le_bytes());
// ... etc

// Make it visible and wake driver
ring.publish();
ring.notify();
```

### partd receives:

```rust
loop {
    // Wait for command (blocks if ring empty)
    let cmd = ring.recv();

    match cmd.cmd_type {
        CMD_ATTACH_DISK => {
            let shmem_id = u32::from_le_bytes(cmd.payload[0..4].try_into().unwrap());
            let block_size = u32::from_le_bytes(cmd.payload[4..8].try_into().unwrap());
            handle_attach_disk(shmem_id, block_size);
        }
        CMD_REGISTER_PARTITION => { ... }
        _ => { /* unknown */ }
    }

    // Mark as consumed (frees slot for producer)
    ring.consume();
}
```

## Comparison

### Old way (current):
```rust
// devd side
let msg = AttachDiskMsg { shmem_id, block_size, ... };
let bytes = msg.to_bytes();           // Serialize
syscall::write(channel, &bytes)?;     // Syscall + kernel copy

// partd side
let mut buf = [0u8; 256];
let n = channel.recv(&mut buf)?;      // Syscall + kernel copy
let msg = AttachDiskMsg::from_bytes(&buf[..n])?;  // Deserialize
```

**Costs:** 2 syscalls, 2 kernel copies, serialize, deserialize

### New way (ring):
```rust
// devd side
let cmd = ring.reserve(5000)?;        // Just pointer math
cmd.cmd_type = CMD_ATTACH_DISK;       // Direct write to shared memory
cmd.payload[0..4].copy_from_slice(&shmem_id.to_le_bytes());
ring.publish();                       // Just atomic increment
ring.notify();                        // 1 syscall (only to wake)

// partd side
let cmd = ring.recv();                // Pointer math + maybe 1 syscall to sleep
let shmem_id = u32::from_le_bytes(...);  // Direct read
ring.consume();                       // Atomic increment
```

**Costs:** 0-2 syscalls (only for wake/sleep), 0 kernel copies, no serialize/deserialize

## When the Ring is Hot (No Syscalls!)

If producer and consumer are both active:
- Producer writes, publishes, notifies → notify is a no-op if consumer isn't sleeping
- Consumer polls, processes, consumes → no syscall needed

The syscall only happens when:
1. Ring is empty and consumer needs to sleep
2. Ring was empty and producer needs to wake consumer

## Required Kernel Support

Minimal changes needed:

```rust
// New: Lightweight wake channel
// - No message buffer, just a wake flag
// - write() = set wake flag + wake sleepers
// - read() = clear wake flag, sleep if not set

pub struct WakeChannel {
    woken: AtomicBool,
    subscriber: Option<Subscriber>,
}

impl WakeChannel {
    fn write(&self) {
        self.woken.store(true, Ordering::Release);
        if let Some(sub) = self.subscriber {
            wake(sub);
        }
    }

    fn read_blocking(&self) {
        loop {
            if self.woken.swap(false, Ordering::Acquire) {
                return;  // Was woken, return immediately
            }
            // Sleep until write() is called
            sleep_until_woken();
        }
    }
}
```

## Migration Path

1. Add `WakeChannel` kernel object type
2. Add `CommandRing` to userlib
3. Update devd ↔ partd to use rings (test case)
4. Update devd ↔ fatfsd
5. Update devd ↔ xhcid
6. Deprecate old query protocol

## Benefits

| Aspect | Old | New |
|--------|-----|-----|
| Syscalls per message | 2 | 0-2 |
| Kernel copies | 2 | 0 |
| Serialization | Yes | No |
| Cache behavior | Poor (kernel buffers) | Good (shared memory) |
| Latency | High | Low |
| Complexity | High (WouldBlock, etc) | Low |

## Open Questions

1. **Ring size:** 16 slots enough? Make it configurable?
2. **Backpressure:** What if producer is faster than consumer? Timeout + error?
3. **Multiple producers:** Need separate rings or locking?
4. **Priorities:** Different rings for high/low priority commands?
