# Signal-Based Supervision

> Replaces channel-based supervision with lightweight signals, shared
> mailboxes, and kernel-delivered port state notifications.

## Motivation

The current supervision model uses IPC channels for everything:
lifecycle, config delivery, state reporting, death detection. This
works but creates scaling problems:

- **devd holds a channel per child** — Mux grows linearly with services
- **bus_runtime is mandatory** — 400+ lines of framework to parse
  supervision protocol, manage Mux, dispatch callbacks
- **kernel bus ports use channel messages** — `notify_supervisor()`
  encodes state changes into IPC messages, devd decodes them
- **config delivery requires protocol** — StateSnapshot, DeviceList,
  SpawnContext all encoded into IPC payloads

Signals collapse the control plane into something cheaper:

| Before (channels) | After (signals) |
|---|---|
| Supervision channel per child | `post_signal(child, SHUTDOWN, 0)` |
| Channel message for state change | `post_signal(devd, PORT_CHANGED, port\|state)` |
| Protocol parsing in bus_runtime | Signal handler in 3 lines |
| QueryHandler in devd (500+ lines) | Metrics object read |
| Mux with O(children) handles | Mux with O(active-queries) handles |

## Signal Mechanism

### Data Structure

```rust
/// A pending signal in a task's signal queue.
/// 12 bytes — fits 8 signals in 96 bytes of TCB space.
#[derive(Clone, Copy)]
pub struct PendingSignal {
    pub event: u32,    // event type bitmask
    pub value: u64,    // inline payload or mailbox pointer
}
```

### Per-Task Signal Queue

```rust
// In Task (TCB):
pub signal_queue: [PendingSignal; 8],
pub signal_head: u8,
pub signal_tail: u8,
pub signal_mask: u32,   // which events are enabled (all by default)
```

8 entries is enough because signals are consumed on the next Mux
poll. If the queue overflows, the kernel coalesces: ORs the event
bits into the newest entry. This matches QNX pulse semantics — you
may miss individual signals, but never miss that an event type
occurred.

### Delivery via Microtask Queue

Signal delivery reuses the existing microtask infrastructure:

```rust
pub enum MicroTask {
    // ... existing cleanup variants unchanged ...

    /// Deliver a signal to a task's signal queue and wake it.
    Signal { target: TaskId, event: u32, value: u64 },
}
```

The execution path:

```
sys_signal(target, event, value)
  or kernel: bus port state change
  or kernel: child exit
      │
      ▼
microtask::enqueue(MicroTask::Signal { target, event, value })
      │
      ▼ (drain runs outside scheduler lock)
      │
exec_signal():
  1. Lock target's signal queue (TCB field, IRQ-disable only)
  2. Write PendingSignal { event, value } to queue
  3. If queue full: coalesce (OR event bits into tail entry)
  4. Wake target task (if blocked)
```

Same lock ordering as today. Same drain budget. Same wake path.

### Existing Microtasks That Become Signals

| Current MicroTask | Signal equivalent |
|---|---|
| `NotifyParentExit { parent_id, child_pid, code }` | `Signal { target: parent, EVENT_CHILD_EXIT, (pid << 32) \| code }` |
| `KillChildren { pid }` | Loop: `Signal { target: child, EVENT_SHUTDOWN, 0 }` per child |
| `Wake { pid }` | Stays as-is (internal scheduler mechanism, not user-visible) |
| `Evict { pid, reason }` | `Signal { target: pid, EVENT_EVICT, reason }` |

Wake and WakeSweep remain internal — they're scheduler primitives,
not user-visible events.

### Mux Integration

When a task polls its Mux, signals are checked **first**, before any
handle events:

```rust
// In read_mux (kernel/object/syscall.rs):
fn poll_mux(task_id: TaskId, ...) -> ReadResult {
    // Phase 0: Check signal queue (highest priority)
    if let Some(sig) = dequeue_signal(task_id) {
        return ReadResult::Signal(sig.event, sig.value);
    }

    // Phase 1-3: Normal handle polling (unchanged)
    ...
}
```

Userspace sees signals as a Mux event type:

```rust
match mux.wait() {
    Event::Signal(event, value) => { ... }
    Event::Readable(handle) => { ... }
    Event::Writable(handle) => { ... }
    Event::Closed(handle) => { ... }
}
```

### Syscall

```
sys_signal(target_pid: u32, event: u32, value: u64) -> i64

Returns: 0 on success, negative errno on failure
```

Fits the unified model as `write(ObjectType::Signal, ...)` but a
dedicated syscall number is simpler since it doesn't operate on a
handle. Alternatively: `write(process_handle, signal_payload)`.

### Permission Model

- **Parent → child**: always allowed (parent spawned it)
- **Child → parent**: always allowed (for READY, status reporting)
- **Kernel → any task**: always allowed (port state, eviction)
- **Cross-tree**: requires `CAP_SIGNAL` capability
- **signal_allowlist**: per-task allowlist (already in TCB) for
  explicit grants

## Event Types

```rust
pub mod signal_event {
    // Lifecycle
    pub const CHILD_EXIT: u32    = 1 << 0;  // value: (pid << 32) | exit_code
    pub const SHUTDOWN: u32      = 1 << 1;  // value: reason (REBOOT/POWEROFF/RESTART)
    pub const INIT: u32          = 1 << 2;  // value: 0 (mailbox has config)
    pub const READY: u32         = 1 << 3;  // value: 0 (child is operational)

    // Interactive
    pub const INTERRUPT: u32     = 1 << 4;  // value: 0 (Ctrl+C)
    pub const RESIZE: u32        = 1 << 5;  // value: (width << 16) | height

    // Kernel-delivered
    pub const PORT_CHANGED: u32  = 1 << 6;  // value: (port_idx << 8) | new_state
    pub const HARDWARE: u32      = 1 << 7;  // value: irq number

    // Supervision
    pub const HEARTBEAT: u32     = 1 << 8;  // value: 0 (ping from supervisor)
    pub const CONFIG: u32        = 1 << 9;  // value: mailbox offset
    pub const EVICT: u32         = 1 << 10; // value: reason code
}
```

Most signals carry scalar payloads inline in the 64-bit value field.
For structured data (device lists, config blobs), the value points
into a shared mailbox page.

## Mailbox

A shared memory page allocated at spawn time, mapped in both parent
and child. Replaces: StateSnapshot, DeviceList, SpawnContext protocol
messages.

### Setup

```
exec_with_mailbox(elf_path, caps, mailbox_content)
  → kernel allocates 4KB shared page
  → copies mailbox_content into page
  → maps page in parent at known address
  → maps page in child at known address
  → child gets handle slot 4 = MAILBOX (shmem handle)
  → parent retains its mapping
```

Or simpler: parent creates Shmem, writes config, passes shmem handle
to child via existing exec_with_channel (handle slot 4 could be
shmem instead of channel, or slot 5).

### Layout

The mailbox page has a simple header + typed payload:

```rust
/// Mailbox header (first 64 bytes of shared page)
#[repr(C)]
pub struct MailboxHeader {
    pub magic: u32,         // 0x4D424F58 ("MBOX")
    pub version: u16,       // Protocol version
    pub flags: u16,         // PARENT_WRITTEN | CHILD_WRITTEN
    pub parent_seq: u32,    // Incremented when parent writes
    pub child_seq: u32,     // Incremented when child writes

    // Device/bus info (written by parent before INIT signal)
    pub bus_type: u8,
    pub bus_index: u8,
    pub device_count: u8,
    pub capabilities: u8,
    pub base_addr: u64,
    pub size: u64,
    pub irq: u32,
    pub _reserved: [u8; 20],
}
// Offset 64: device list, config KVs, etc.
```

### Usage Pattern

```
Parent (devd)                         Child (driver)
  │                                      │
  ├─ write MailboxHeader to shmem        │
  ├─ write device list at offset 64      │
  ├─ exec_with_mailbox(elf, shmem) ──────┤
  │                                      │ handle 4 = mailbox shmem
  ├─ post_signal(child, INIT, 0) ────────┤
  │                                      ├─ map mailbox
  │                                      ├─ read header + device list
  │                                      ├─ init hardware
  │              READY  ◄────────────────┤  post_signal(parent, READY, 0)
  │                                      │
  │  ... runtime ...                     │
  │                                      │
  ├─ update config at offset 256         │
  ├─ bump parent_seq                     │
  ├─ post_signal(child, CONFIG, 256) ────┤
  │                                      ├─ read config from offset 256
  │                                      ├─ apply config
  │                                      │
  ├─ post_signal(child, SHUTDOWN, 0) ────┤
  │                                      ├─ flush hardware
  │                                      ├─ exit(0)
  │              CHILD_EXIT  ◄───────────┤  (kernel delivers)
```

Zero channel messages. Zero protocol encoding. Zero bus_runtime.

### Status Reporting (Child → Parent)

The child writes status to a known offset in the mailbox. Parent
reads it on demand (or after receiving READY signal). No query/response
protocol needed.

```rust
// Child writes periodically or on state change:
mailbox.write_at(STATUS_OFFSET, &DriverStatus {
    state: DriverState::Running,
    uptime_ms: now(),
    requests_served: self.stats.total,
    errors: self.stats.errors,
});
```

Parent reads it whenever it wants — no IPC, no syscall, just a
memory read from a mapped page.

## Kernel Port Signaling

The kernel already owns bus ports and tracks devd's PID. Today,
`BusController::notify_supervisor()` sends IPC channel messages.
With signals, it posts directly:

### Current (channel-based)

```rust
// In bus/controller.rs notify_supervisor():
let payload = [STATE_CHANGED, old_state, new_state, reason_code];
let mut msg = Message::new();
msg.payload[..4].copy_from_slice(&payload);
ipc::send_unchecked(supervisor_ch, msg)?;  // needs channel
```

### New (signal-based)

```rust
// In bus/controller.rs notify_supervisor():
let value = (self.bus_index as u64) << 8 | new_state as u64;
microtask::enqueue(MicroTask::Signal {
    target: devd_pid(),
    event: signal_event::PORT_CHANGED,
    value,
});
```

No channel. No message encoding. No `supervisor_ch` field on
BusController. The kernel signals devd directly.

### What devd does with PORT_CHANGED

```rust
// devd event loop:
Event::Signal(PORT_CHANGED, value) => {
    let port_idx = (value >> 8) as u8;
    let new_state = (value & 0xFF) as u8;

    match new_state {
        STATE_SAFE => {
            // Driver exited, port is safe — check restart policy
            maybe_restart(port_idx);
        }
        STATE_CLAIMED => {
            // Driver connected — record it
            mark_active(port_idx);
        }
        STATE_RESETTING => {
            // Hardware reset in progress — wait
        }
    }
}
```

### What this removes from BusController

- `supervisor_ch: Option<ChannelId>` — gone
- `supervisor_pid: Option<Pid>` — replaced by `devd_pid()` global
- `notify_supervisor()` method — replaced by 3-line signal post
- `handle_supervisor_disconnect()` — not needed (no channel to disconnect)
- The entire supervision channel lifecycle in `handle_connect()`

## Metrics Object (System Observability)

A read-only kernel object for querying system health. Opens the door
for `top`-like monitoring, devd health checks, and shell diagnostics.

### Interface

```
handle = open(ObjectType::Metrics, 0)
read(handle, buf, METRIC_PROCTABLE)  → process table with PID/PPID/state/CPU
read(handle, buf, METRIC_SYSTEM)     → global stats (uptime, memory, IPC counts)
read(handle, buf, METRIC_PORTS)      → port state table
```

The `flags` argument to `read()` selects which metric set to return.

### Process Table (METRIC_PROCTABLE)

Returns the same data as `list_processes_ex()` but adds signal
stats and is available through a handle (not a special syscall):

```rust
pub struct ProcessMetric {
    pub pid: u32,
    pub ppid: u32,
    pub state: u8,
    pub cpu: u8,
    pub priority: u8,
    pub num_children: u8,
    pub cpu_time_ns: u64,
    pub handle_count: u16,
    pub channel_count: u16,
    pub signal_pending: u32,    // pending signal bitmask
    pub signals_received: u32,  // lifetime signal count
    pub name: [u8; 16],
}
```

devd can read this to reconstruct the full process tree from
PID/PPID relationships. No channels needed — just open the metrics
object and read. Combined with CHILD_EXIT signals for real-time
notification, devd has full visibility without holding any
supervision channels.

### System Metrics (METRIC_SYSTEM)

```rust
pub struct SystemMetrics {
    pub uptime_ns: u64,
    pub total_pages: u32,
    pub free_pages: u32,
    pub task_count: u16,
    pub cpu_count: u8,
    pub _pad: u8,
    pub context_switches: u64,   // global counter
    pub signals_delivered: u64,  // global counter
    pub ipc_messages: u64,       // global counter
    pub microtasks_drained: u64, // global counter
    pub per_cpu_idle_ns: [u64; 4], // idle time per CPU
}
```

Shell command: `top` or `sysinfo` reads this and formats it.

### Port Table (METRIC_PORTS)

```rust
pub struct PortMetric {
    pub name: [u8; 32],
    pub name_len: u8,
    pub bus_type: u8,
    pub state: u8,       // Safe/Claimed/Resetting
    pub owner_pid: u32,  // who claimed it (0 = nobody)
    pub base_addr: u64,
    pub capabilities: u8,
}
```

devd reads this at startup to discover existing ports (replaces
`discover_kernel_buses()` with a metrics read). Combined with
PORT_CHANGED signals for real-time updates.

## What a Driver Becomes

### Before (bus_runtime, ~400 lines of framework)

```rust
struct MyDriver { ... }

impl Driver for MyDriver {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        // read StateSnapshot from bus channel
        // parse DeviceList
        // init hardware
    }
    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        // parse message type, dispatch
    }
    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) { ... }
}

fn main() { driver_main(b"mydriver", MyDriver::new()); }
```

### After (signal + mailbox, driver owns its loop)

```rust
fn main() {
    let mailbox = Shmem::from_raw(Handle::MAILBOX);
    let header: &MailboxHeader = mailbox.read_at(0);
    let mmio = Mmio::map(header.base_addr, header.size);

    init_hardware(&mmio, header);

    signal(ppid(), signal_event::READY, 0);

    let mux = Mux::new();
    // add data handles if this driver does I/O

    loop {
        match mux.wait() {
            Signal(SHUTDOWN, _) => break,
            Signal(CONFIG, offset) => {
                let cfg = mailbox.read_at(offset as usize);
                apply_config(cfg, &mmio);
            }
            Signal(INTERRUPT, _) => abort_current_op(),
            Readable(h) => handle_data(h),
        }
    }

    shutdown_hardware(&mmio);
}
```

No Driver trait. No driver_main. No BusCtx. No bus_runtime. No
supervision channel. No protocol encoding. 30 lines of clear,
sequential code.

bus_runtime can still exist as a convenience library for drivers
that want the callback pattern, but it's no longer mandatory.

## What This Replaces

| Component | Status |
|---|---|
| `bus_runtime.rs` supervision protocol | Replaced by signals |
| `bus_runtime.rs` StateSnapshot parsing | Replaced by mailbox read |
| `bus_runtime.rs` SpawnContext delivery | Replaced by mailbox KVs |
| `BusController.supervisor_ch` | Replaced by `devd_pid()` + signals |
| `BusController.notify_supervisor()` | Replaced by signal post |
| devd QueryHandler (supervision part) | Replaced by signals + metrics |
| devd per-child channels (lifecycle) | Replaced by CHILD_EXIT signal |
| `MicroTask::NotifyParentExit` | Replaced by `MicroTask::Signal` |
| `MicroTask::Evict` | Replaced by `MicroTask::Signal` |
| `Process::watch()` object type | Optional (signals cover exit detection) |

## What Stays

| Component | Why |
|---|---|
| Channels | Request/response IPC (shell commands, queries) |
| Ports | Named service discovery (console:, devd-query:) |
| DataPort rings | Bulk I/O (block devices, network) |
| Mux | Event multiplexing (signals are a new event type) |
| bus_runtime (optional) | Convenience library for callback-style drivers |

## The Three Planes

```
┌──────────────────────────────────────────────┐
│  Signals              (control plane)        │
│  Lifecycle, interrupts, port state changes   │
│  ~10 cycles, no allocation, no locks         │
├──────────────────────────────────────────────┤
│  Mailbox              (config/status plane)  │
│  Shared page: device info, config, status    │
│  Zero-copy, no syscall to read/write data    │
├──────────────────────────────────────────────┤
│  Channels + DataPort  (data plane)           │
│  IPC messages, ring buffers, DMA pools       │
│  For interactive queries and bulk I/O        │
└──────────────────────────────────────────────┘
```

## Migration Path

1. **Add signal infrastructure** — `MicroTask::Signal`, per-task
   signal queue in TCB, Mux signal polling
2. **Add `sys_signal()` syscall** — or `write(process_handle, ...)`
3. **Add Metrics object** — `open(Metrics)`, read process/system/port
   tables
4. **Convert kernel port notifications** — `notify_supervisor()` →
   signal post to `devd_pid()`
5. **Convert `NotifyParentExit`** — existing microtask → generic
   Signal variant
6. **Add mailbox to spawn** — `exec_with_mailbox()` or pass shmem
   handle via slot 5
7. **Write signal-based driver examples** — cpud, consoled as proof
   of concept
8. **Make bus_runtime optional** — drivers can use signals+mailbox
   directly
9. **Simplify devd** — remove per-child channels for lifecycle,
   use signals + metrics reads

Steps 1-3 are additive (no breaking changes). Steps 4-5 convert
existing internal mechanisms. Steps 6-9 are the gradual migration.

## Open Questions

- **Signal coalescing policy**: OR event bits (lose individual
  signals) vs. drop oldest (lose history)? OR is simpler and
  matches the "event happened" semantics.
- **Mailbox size**: 4KB fits most configs. Larger drivers (PCIe
  with 32 devices) may need 8KB. Fixed at spawn or growable?
- **Metrics polling rate**: Should devd poll metrics on a timer,
  or only read after receiving a signal? Signal-driven is more
  efficient.
- **Backward compatibility**: Keep bus_runtime working during
  migration? Yes — make it optional, not deleted.
