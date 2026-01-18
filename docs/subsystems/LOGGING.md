# Unified Logging Framework

## Design Decisions (Locked In)

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Timestamp | boot_ms always | Wall clock is syslog daemon's concern |
| K/U format | Identical | Subsystem name is the differentiator |
| Subsystem names | Compile-time validated | const fn check, no runtime overhead |
| Ring buffer | Binary records | Compact, fast write, format on drain |
| KV storage | Typed values + string keys | Compact, proper numeric formatting |
| Max message | 512 bytes | Room for detailed errors |
| Ring size | 64KB | ~128 messages |
| Overflow | Overwrite oldest | Keep recent context for debugging |
| INFO level | Major milestones only | State transitions are DEBUG |
| Control plane | devd only | Unified control, no kernel sysctl |

## Design Goals

1. **One format everywhere** - Kernel, userspace, all drivers use identical format
2. **Zero allocation** - Fixed buffers only, no heap
3. **Non-blocking** - Never wait for UART, never deadlock
4. **Structured** - `key=value` for all facts, stable event tokens
5. **Filterable** - By subsystem, level, device, at compile-time and runtime
6. **Grep-friendly** - Single lines, consistent schema, machine-parseable
7. **Survives panic** - Direct output path for fatal errors

## Log Line Format

```
<ms> <LVL> <subsys> [ctx] <event> key=val ...
```

| Field | Width | Description | Example |
|-------|-------|-------------|---------|
| ms | 8 digits | Milliseconds since boot, zero-padded | `00012483` |
| LVL | 5 chars | Level (ERROR/WARN/INFO/DEBUG/TRACE) | `INFO ` |
| subsys | 6 chars | Subsystem name, left-padded | ` pcie` |
| ctx | variable | Context in brackets, optional | `[dev=0000:01:00.0]` |
| event | variable | Stable event token (snake_case) | `probe_ok` |
| key=val | variable | Structured data | `vendor=0x14c3 device=0x7990` |

### Examples

```
00012483 INFO   pcie [dev=0000:01:00.0] probe_ok vendor=0x14c3 device=0x7990
00012490 INFO  mt7996 [dev=0001:01:00.0 hif=1] bar_map_ok phys=0x30200000 size=2097152
00012501 WARN    dma [dev=0001:01:00.0] rst_stuck val=0x00000030 expected=0x00000000
00012510 ERROR    fw [dev=0001:01:00.0] load_failed component=WM err=timeout next=reset
00012520 DEBUG  sched [pid=4] task_switch from=3 to=4 reason=yield
00000042 INFO  kernel [] boot_start version="0.1.0" cpu=cortex-a73
```

## Subsystems (Registered Names)

| Subsystem | Description |
|-----------|-------------|
| `kernel` | Core kernel (boot, memory, panic) |
| `sched` | Scheduler, task management |
| `irq` | Interrupt handling |
| `mmu` | Memory management, page tables |
| `ipc` | Inter-process communication |
| `shmem` | Shared memory |
| `pcie` | PCIe bus driver |
| `usb` | USB host controller |
| `mt7996` | MT7996 WiFi driver |
| `dma` | DMA engine operations |
| `fw` | Firmware loading |
| `mcu` | MCU communication |
| `devd` | Device supervisor |
| `shell` | Shell/console |
| `fatfs` | FAT filesystem |

New subsystems: just use them. No registration required. Keep names short (<=6 chars preferred).

## API

### Kernel (src/klog.rs)

```rust
// Basic logging - subsystem from module path or explicit
log!(INFO, "pcie", "probe_ok"; dev = bdf, vendor = vid, device = did);
log!(ERROR, "fw", "load_failed"; component = "WM", err = "timeout", next = "reset");

// With context (common pattern)
log!(INFO, "mt7996", [dev = bdf, hif = 1], "bar_map_ok"; phys = addr, size = sz);

// Convenience macros (subsystem from caller module)
info!("probe_ok"; vendor = 0x14c3, device = 0x7990);
warn!("rst_stuck"; val = rst, expected = 0);
error!("load_failed"; err = "timeout", next = "reset");
debug!("task_switch"; from = old_pid, to = new_pid);
trace!("reg_write"; addr = 0xd4208, val = 0x1430b875);

// For dumps (multi-line, bracketed)
dump_begin!("dma", "reg_dump"; dev = bdf, count = 10);
// ... individual dump lines ...
dump_end!("dma", "reg_dump"; dev = bdf);
```

### Userspace (userlib/src/ulog.rs)

Identical API, different backend (syscall to drain buffer).

```rust
use userlib::log::{log, info, warn, error, debug, trace};

info!("mt7996", "init_start"; version = "1.0");
error!("mt7996", [dev = bdf], "fw_timeout"; queue = "WM", cpu_idx = 5, dma_idx = 0);
```

## Implementation

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         CALLSITE                                 │
│  log!(INFO, "pcie", "probe_ok"; vendor=0x14c3)                  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      FORMAT (inline)                             │
│  - Check level vs compile-time/runtime threshold                │
│  - Format into stack buffer (256 bytes max)                     │
│  - No allocation, no locks                                       │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    RING BUFFER (klog)                            │
│  - 32KB lock-free ring (single producer OK for now)             │
│  - Atomic head/tail pointers                                     │
│  - Overwrite oldest on full (track dropped count)               │
│  - IRQ-safe: no locks in write path                             │
└─────────────────────────────────────────────────────────────────┘
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
┌─────────────────────────┐     ┌─────────────────────────────────┐
│    NORMAL DRAIN         │     │      PANIC DRAIN                 │
│  - Timer tick (10ms)    │     │  - Direct UART (polled)          │
│  - Non-blocking flush   │     │  - Flush entire buffer           │
│  - 64 bytes per tick    │     │  - Then print panic info         │
└─────────────────────────┘     └─────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         UART                                     │
│  - Buffered output (existing 4KB buffer)                        │
│  - Timer-driven drain to hardware                               │
└─────────────────────────────────────────────────────────────────┘
```

### Binary Record Format

Records are stored in binary for compact storage and fast writes. Formatting to text
happens only when draining to UART.

```rust
/// Fixed header (8 bytes)
struct RecordHeader {
    total_len: u16,       // Total record length including header
    ts_ms: u32,           // Milliseconds since boot
    level: u8,            // Level enum (0-4)
    flags: u8,            // Reserved for future use
}

/// Value types for key-value pairs
#[repr(u8)]
enum ValueType {
    U64 = 0,              // 8 bytes, formatted as decimal
    I64 = 1,              // 8 bytes, formatted as signed decimal
    Hex32 = 2,            // 4 bytes, formatted as 0x%08x
    Hex64 = 3,            // 8 bytes, formatted as 0x%016x
    Bool = 4,             // 1 byte, formatted as true/false
    Str = 5,              // 1-byte len + bytes, no quotes
}

/// Key-value pair (variable length)
struct KeyValue {
    key_len: u8,          // Length of key string
    value_type: u8,       // ValueType enum
    // key_bytes[key_len]
    // value_bytes[...] (size depends on value_type)
}

/// Full record layout:
/// [RecordHeader: 8 bytes]
/// [subsys_len: 1 byte]
/// [subsys: subsys_len bytes]
/// [event_len: 1 byte]
/// [event: event_len bytes]
/// [ctx_count: 1 byte]
/// [ctx KeyValue pairs...]
/// [kv_count: 1 byte]
/// [data KeyValue pairs...]
```

### Ring Buffer Design

```rust
struct LogRing {
    buffer: [u8; 65536],      // 64KB
    head: AtomicU32,          // Write position
    tail: AtomicU32,          // Read position
    dropped: AtomicU64,       // Dropped message count (monotonic)
    sequence: AtomicU64,      // Message sequence number
}
```

**Write path** (must be fast, IRQ-safe):
1. Serialize record into 512-byte stack buffer
2. Calculate total length
3. Atomic reserve space in ring (CAS on head)
4. If not enough space: advance tail (overwrite oldest), increment dropped
5. Copy record bytes to ring
6. Memory barrier (ensure visibility)

**Read path** (timer context):
1. Check if data available (head != tail)
2. Read RecordHeader to get total_len
3. Format record to text (stack buffer)
4. Write text to UART buffer
5. Advance tail

### Levels

```rust
#[repr(u8)]
pub enum Level {
    Error = 0,  // Operation failed, degraded behavior
    Warn  = 1,  // Unexpected but recoverable
    Info  = 2,  // Major lifecycle events, state transitions
    Debug = 3,  // Diagnostic, not per-packet
    Trace = 4,  // Very verbose, per-operation
}
```

**Compile-time filtering**: `#[cfg(feature = "log-level-debug")]`
**Runtime filtering**: Per-subsystem level threshold (future: via devd)

### Formatting Rules

1. **Timestamps**: 8-digit milliseconds, zero-padded (`00012483`)
2. **Hex values**: Always `0x` prefix, lowercase, appropriate width
   - Registers: 8 digits (`0x1430b875`)
   - Addresses: 8+ digits (`0x30200000`)
   - Small values: minimal (`0x14c3`)
3. **Strings**: No quotes unless contains spaces
4. **Booleans**: `true`/`false` (not `1`/`0`)
5. **Sizes**: Raw bytes (let consumer convert to KB/MB)
6. **Events**: snake_case, stable tokens (`probe_ok`, `load_failed`)

### Error Triad

All ERROR logs must include:
- **op**: What operation failed
- **err**: Error code/reason
- **next**: What happens next (retry, reset, panic, ignore)

```rust
error!("cfg_read_failed"; op = "pcie_config", err = "timeout", next = "retry");
```

### Context Keys (Standardized)

| Key | Format | Description |
|-----|--------|-------------|
| `dev` | `SSSS:BB:DD.F` | PCI device (segment:bus:device.func) |
| `pid` | decimal | Process ID |
| `tid` | decimal | Thread/task ID |
| `irq` | decimal | IRQ number |
| `cpu` | decimal | CPU number |
| `hif` | `1` or `2` | MT7996 HIF interface |
| `q` | name | Queue name (WM, FWDL, etc.) |
| `addr` | `0x...` | Memory address |
| `reg` | `0x...` | Register offset |

## Migration Path

### Phase 1: Core Framework
1. Implement `src/klog.rs` with ring buffer and macros
2. Implement `user/userlib/src/ulog.rs` (same macros, syscall drain)
3. Keep old `println!` working (maps to `info!("legacy", "print"; msg = ...)`)

### Phase 2: Adopt in Key Paths
1. Convert `devd` to use new logging
2. Convert `pcied` to use new logging
3. Convert kernel boot sequence

### Phase 3: Full Migration
1. Convert all drivers
2. Remove old print infrastructure
3. Add runtime level control via devd

### Phase 4: Advanced Features
1. Per-subsystem runtime filtering
2. Snapshot-on-error (dump last N messages on ERROR)
3. Remote streaming (when netd exists)

## What Gets Removed

After migration:
- `src/log.rs` (old log levels) → replaced by `klog.rs`
- `log_info!`, `log_warn!`, etc. → replaced by `info!`, `warn!`
- `logln!` in userspace → replaced by `info!`, etc.
- Security logging → integrated into main framework with subsys=`security`

## Don't Do

1. **Don't log steps** - Log state transitions and results
2. **Don't use prose** - Use `key=value`
3. **Don't invent new events** - Reuse existing tokens
4. **Don't log in hot paths** - Use TRACE level, compile out
5. **Don't block** - Drop messages if buffer full
6. **Don't allocate** - Stack buffers only
7. **Don't mix shell UI with logs** - Shell prompts are separate

## File Layout

```
src/
  klog.rs              # Kernel logging (ring buffer, macros, drain)

user/userlib/src/
  ulog.rs              # Userspace logging (same API, syscall drain)

docs/
  LOGGING.md           # This document
```
