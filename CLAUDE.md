# BPI-R4 Kernel Project

A reliability-focused microkernel for the Banana Pi BPI-R4 (MT7988A SoC).

## Quick Links

| Document | Description |
|----------|-------------|
| [docs/PRINCIPLES.md](docs/PRINCIPLES.md) | **MUST READ** - Core design philosophy |
| [docs/README.md](docs/README.md) | Documentation index |
| [docs/architecture/](docs/architecture/) | System architecture |
| [docs/decisions/ADR.md](docs/decisions/ADR.md) | Architecture decisions |

## The Laws

These are inviolable. No exceptions. No "just this once."

### 1. State Machine is the Law of the Land

Every component with lifecycle has an **explicit state enum**:
```rust
pub enum ChannelState { Open, HalfClosed, Closed }
pub enum PortState { Listening, Closed }
pub enum TimerState { Armed, Disarmed }
```

- **Single source of truth** - ONE state field, not `is_closed + peer_gone + blocked`
- **Explicit transitions** - State changes only via defined methods with validation
- **No implicit state** - No deriving state from multiple booleans

### 2. Traits and Modules for Testability

```rust
pub trait Pollable {
    fn poll(&self, filter: u8) -> PollResult;
    fn subscribe(&mut self, sub: Subscriber);
    fn unsubscribe(&mut self);
}
```

- **Trait-based contracts** - Define behavior, not implementation
- **Pure functions where possible** - Inputs in, outputs out, no hidden state
- **Separate syscall glue from core logic** - Core logic is testable without scheduler

### 3. Unified 5-Syscall Interface

Everything is an object. Everything uses the same 5 syscalls:

| Syscall | Number | Purpose |
|---------|--------|---------|
| `open` | 100 | Open/create any object (channel, port, timer, shmem, etc.) |
| `read` | 101 | Read from object (recv message, accept connection, poll mux) |
| `write` | 102 | Write to object (send message, arm timer, add to mux) |
| `map` | 103 | Map object to memory (shmem, DMA pool, MMIO) |
| `close` | 104 | Close object |

- **IPC is a channel** - Same semantics as file open/read/write/close
- **No special syscalls** - Everything goes through the unified interface
- **Object types via open flags** - `open(ObjectType::Channel, flags, arg)`

### 4. No Tech Debt

- **Complete migrations only** - No half-finished rewrites
- **Deprecate completely** - Old code returns ENOSYS, then gets deleted
- **No backward compatibility shims** - Clean breaks, not cruft accumulation

### 5. Subscriber-Based Waking

```rust
pub struct Subscriber { task_id: u32, generation: u32 }
pub enum WakeReason { Readable, Writable, Closed, Accepted, Timeout }
```

- **Single wake mechanism** - Tasks subscribe to events, get woken
- **Generation prevents stale wakes** - Detect task slot reuse
- **Wake outside locks** - Collect subscribers under lock, wake after release

---

## Key Principles (Summary)

See [docs/PRINCIPLES.md](docs/PRINCIPLES.md) for complete design philosophy.

- **SMP safe always** - Multi-core safe code required
- **State machine design** - Every boundary has explicit states
- **No busy-wait loops** - Use events, never poll
- **Minimal kernel** - Device management in userspace (devd)
- **DMA memory non-cacheable** - No cache flushes in driver code
- **Capability-based security** - Check permissions for privileged ops

---

## Build & Run

### Build Commands

```bash
# Full build (kernel + userspace + initrd)
./build.sh

# Build with MT7996 firmware embedded
./build.sh --with-firmware

# Build with self-tests enabled
./build.sh --test
```

### Load via U-Boot

```
1. loady 0x46000000
2. (send kernel.bin via Xmodem)
3. go 0x46000000
```

### Hardware

- **Board**: Banana Pi BPI-R4
- **SoC**: MediaTek MT7988A
- **CPU**: ARM Cortex-A73 (AArch64)
- **USB**: SSUSB0 (IRQ 205, M.2), SSUSB1 (IRQ 204, USB-A via VL822)

---

## Project Structure

```
bpi-r4-kernel/
├── src/                    # Kernel code
│   ├── main.rs             # Entry point
│   ├── kernel/             # Core kernel modules
│   ├── arch/aarch64/       # ARM64-specific code
│   └── platform/mt7988/    # MT7988A SoC code
├── user/
│   ├── userlib/            # Userspace library
│   ├── shell/              # Shell program
│   └── driver/
│       ├── devd/           # Device supervisor (PID 1)
│       ├── pcied/          # PCIe bus driver
│       ├── usbd/           # USB daemon
│       ├── wifid/          # MT7996 WiFi driver
│       └── fatfs/          # FAT filesystem
├── docs/                   # Documentation
│   ├── PRINCIPLES.md       # Design philosophy
│   ├── architecture/       # System design
│   ├── subsystems/         # Kernel subsystems
│   │   ├── LOGGING.md      # Logging framework
│   │   └── TRACING.md      # Tracing framework
│   ├── drivers/            # Driver docs
│   │   ├── USB.md          # USB/xHCI
│   │   └── mt7996-linux/   # MT7996 reference
│   └── decisions/          # ADRs
└── build.sh                # Build script
```

### Linux Reference Code

| Directory | Purpose |
|-----------|---------|
| `mt76/` | Latest mt76 driver (use this) |
| `linux-upstream/` | Latest upstream kernel |
| `linux/` | Local 6.12 with debug printks |

**Always use `mt76/` or `linux-upstream/`** for WiFi driver reference.

---

## Architecture Overview

```
┌─────────────────────────────────────────┐
│  Userspace (apps, drivers)              │
├─────────────────────────────────────────┤
│  Kernel services (IPC, events, shmem)   │
├─────────────────────────────────────────┤
│  Platform HAL (src/platform/mt7988/)    │
├─────────────────────────────────────────┤
│  Arch (src/arch/aarch64/)               │
└─────────────────────────────────────────┘
```

### Key Concepts

- **Minimal kernel** - Provides IPC, memory, scheduling, bus safety only
- **devd is single source of truth** - All device state tracked in userspace
- **Event-driven** - Use kernel event system, not polling
- **Non-cacheable DMA** - All DMA buffers mapped without caching for coherency

See [docs/architecture/OVERVIEW.md](docs/architecture/OVERVIEW.md) for details.

---

## Logging & Tracing

### Logging (see [docs/subsystems/LOGGING.md](docs/subsystems/LOGGING.md))

```
<ms> <LVL> <subsys> [ctx] <event> key=val ...
```

```rust
// Kernel
kinfo!("pcie", "probe_ok"; vendor = 0x14c3, device = 0x7990);
kerror!("fw", [dev = bdf], "load_failed"; err = "timeout");

// Userspace
uinfo!("devd", "init_start"; version = "1.0");
```

### Tracing (see [docs/subsystems/TRACING.md](docs/subsystems/TRACING.md))

```rust
let _span = span!("fw", "load_firmware"; dev = dev.bdf());
load_patch()?;  // Nested spans show call hierarchy
```

---

## MT7996 WiFi Driver

Documentation in [docs/drivers/mt7996-linux/](docs/drivers/mt7996-linux/).

### Translation Rules (MANDATORY)

The wifid driver MUST follow Linux mt76/mt7996 driver code exactly:

1. **EXACT BIT MANIPULATION** - Identical bit operations
2. **EXACT REGISTER ACCESS ORDER** - Same sequence as Linux
3. **EXACT RMW PATTERN** - `mt76_set` = read | bits, `mt76_clear` = read & ~bits
4. **EXACT TIMING** - Same delays
5. **NO IMPROVEMENTS** - Use C way even if Rust is "better"
6. **DOCUMENT EVERY LINE** - Comment with Linux source file:line

### Key Registers

```
MT_WFDMA0_RST           = 0xd4100
MT_WFDMA0_GLO_CFG       = 0xd4208
MT_WFDMA0_RST_DTX_PTR   = 0xd420c
MT_WFDMA_EXT_CSR_HIF_MISC = 0xd7044
```

---

## Common Issues

### USB Known Issues (Regressions)

| Issue | Status | Workaround |
|-------|--------|------------|
| Hub interrupt endpoint not firing | Regression | Polling with GET_PORT_STATUS |
| MSC data transfer flaky | Regression | BOT recovery, retries |

### MT7996

- **RST=0x30 is NORMAL** - Confirmed on working OpenWRT
- **MCU commands use MCU_WM queue (hw_idx=17)**, NOT FWDL
- **FWDL queue (hw_idx=16)** only for raw firmware chunks

---

## Current Work

### Unified Syscall Migration (IN PROGRESS)
- **Kernel object system complete** (`src/kernel/object/`)
  - Pollable trait with explicit state machines
  - open/read/write/map/close implemented
  - Channel, Port, Timer, Mux objects working
- **Userlib ipc2 module complete** (`user/userlib/src/ipc2.rs`)
  - High-level types: Channel, Port, Timer, Mux, Console, Process
  - Old ipc module marked deprecated
- **Legacy syscalls kept working for boot**
  - Old IPC, Handle, Scheme syscalls work but marked LEGACY
  - System boots and runs with existing userspace

### Next Steps
1. Migrate Service framework (`userlib/src/service.rs`) to ipc2
2. Update devd to use ipc2 directly or updated Service
3. Update consoled to use ipc2
4. Update shell to use ipc2
5. Remove legacy syscalls after all userspace migrated

### MT7996 WiFi Driver
- Prefetch configuration fixed to match Linux exactly
- Cache coherency fix applied (kernel DMA pool flush)
- Awaiting hardware test with new serial adapter

---

## Changelog (Recent)

### 2026-01-20 (Session 1)
- **Unified 5-syscall interface** - Complete rewrite of kernel API
  - `src/kernel/object/mod.rs` - Object system with Pollable trait
  - `src/kernel/object/syscall.rs` - open/read/write/map/close handlers
  - Explicit state machines: ChannelState, PortState, TimerState
  - Connected to ipc2 backend for channel/port operations
  - MuxObject for multiplexed waiting (like epoll/kqueue)
- **Documented "The Laws"** in CLAUDE.md
  - State machine is law
  - Traits for testability
  - Unified syscall interface
  - No tech debt policy
  - Subscriber-based waking
- **New userlib ipc2 module** (`user/userlib/src/ipc2.rs`)
  - `Channel` - Bidirectional IPC with explicit state machine
  - `Port` - Named service endpoint with Listening/Closed states
  - `Timer` - Deadline-based timer with Armed/Disarmed/Fired states
  - `Mux` - Event multiplexer (like epoll/kqueue)
  - `Console` - Standard I/O wrapper
  - `Process` - Child process handle for exit watching
  - `Message` - Message buffer helper
- **Legacy syscalls marked but kept working** - For boot compatibility
  - Old IPC syscalls (6-17) marked LEGACY, still work
  - Handle syscalls (80+) marked LEGACY, still work
  - Scheme syscalls (28-30) marked LEGACY, still work
  - Migration path: Service framework -> devd/consoled/shell -> remove legacy

### 2026-01-17 (Session 3)
- **Capability-based security system**
  - `exec_with_caps` syscall for spawning with explicit capabilities
  - Capability presets: BUS_DRIVER, DEVICE_DRIVER, FS_DRIVER, SERVICE_DRIVER, GPIO_DRIVER, USER_DEFAULT
  - devd grants appropriate capabilities to each driver type
  - `verify_binary_signature()` stub for future crypto verification
  - Child capabilities limited to parent's capabilities (delegation model)
- **Bus capabilities reporting**
  - Added `bus_caps` module with hardware capability flags
  - PCIe: BUS_MASTER, MSI, MSIX, LINK_UP (read from hardware)
  - USB: USB_2_0, USB_3_0, RUNNING (read from xHCI registers)
  - `StateSnapshot.capabilities` now reports actual hardware state to devd
- **devd crash recovery**
  - `reset_all_buses()` function resets all hardware to Safe state
  - When devd (PID 1) crashes: kill all tasks, reset buses, respawn devd
  - System continues instead of halting
- **Driver restart improvements**
  - Restart waits for bus Safe signal, not just timer
  - `waiting_for_bus_safe` flag ensures proper sequencing
  - `auto_started` flag: only auto-restart drivers spawned by devd
  - Manually started drivers are not auto-restarted on crash
  - Refactored restart logic into `do_driver_restart()` for consistency

### 2026-01-17 (Session 2)
- **Unified event system** (BSD kqueue-inspired)
  - Multiple timers per task (8 slots) with recurring support
  - EventFilter enum for type-safe subscriptions
  - Batch kevent_wait() for efficiency
  - Migrated consoled, devd, gpio, logd to kevent API
- **Terminal size detection**
  - Query via cursor position report (DSR/CPR)
  - Periodic re-check every 5 seconds
  - Shell `resize` command for manual detection
- **Userspace fault handling**
  - User faults now terminate task gracefully (not hang)
  - Children killed, parent notified via ChildExit event
  - Exit code reflects signal (-11=SIGSEGV, -7=SIGBUS, -6=SIGABRT)
- **Shell fixes**
  - Fixed echo not working (insert_char bug in readline.rs)
- **pcied auto-start re-enabled**

### 2026-01-17 (Session 1)
- **Documentation restructured** into cohesive hierarchy
  - Created `docs/PRINCIPLES.md`, `docs/architecture/`, `docs/subsystems/`
  - Moved ADR.md, USB.md, etc. to proper locations
  - Slimmed CLAUDE.md to essential reference
- **Code cleanup**
  - Removed dead `user/driver/msc/` (usbd handles MSC directly)
  - Removed deprecated syscalls: `pci_enumerate`, `pci_bar_map`, `pci_claim`
  - Documented partial implementations and logging migration status
- **Logging migration** - Migrated all drivers (except wifid) to structured ulog
  - usbd, fatfs, gpio, pwm, nvmed, pciepoke now use uinfo!/uerror!/uwarn!
  - Removed verbose debug output, kept only state changes and errors
- Added kernel unit tests for lock.rs, dma_pool.rs, shmem.rs
- Fixed MT7996 prefetch configuration

### 2026-01-16
- Added logd daemon for log distribution
- Added KlogReady event type
- Consolidated WiFi drivers (wifid3 → wifid)
- Added capability checking infrastructure

### 2026-01-15
- Fixed MT7996 queue selection bugs
- Fixed McuTxd::SIZE (60 → 64 bytes)
- Fixed McuTxd.len calculation

---

## Technical Debt

See [docs/decisions/ADR.md](docs/decisions/ADR.md) for architecture decisions.

### Known Limitations

- **Hub interrupt polling** - USB hub interrupt endpoint regression
- **No kernel VFS** - Filesystem policy in userspace (intentional)
- **Signature verification stub** - `verify_binary_signature()` always passes (infrastructure ready for crypto)

### Partial Implementations

| File | Issue | Notes |
|------|-------|-------|
| `src/platform/mt7988/eth.rs:279` | PHY via MDIO not configured | ETH driver placeholder |
| `src/platform/mt7988/sd.rs:307` | CSD capacity hardcoded 512MB | Parse from register |
| `src/kernel/elf.rs:167` | Signature verification stub | Always passes, ready for crypto |
| `user/driver/usbd/src/main.rs:4739` | SCSI WRITE(10) not impl | Read works |
| `user/driver/wifid/src/main.rs:2221` | RX response uses delay | Replace with event |
| `user/userlib/src/byte_ring.rs:45` | No futex notification | Optimization |

### Logging Migration Status

| Driver | Status |
|--------|--------|
| usbd | ✓ Migrated |
| fatfs | ✓ Migrated |
| gpio | ✓ Migrated |
| pwm | ✓ Migrated |
| nvmed | ✓ Migrated |
| pciepoke | ✓ Migrated |
| devd | ✓ Migrated |
| pcied | ✓ Migrated |
| wifid | Partial (pending) |

