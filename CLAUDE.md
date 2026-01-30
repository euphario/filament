# BPI-R4 Kernel Project

A reliability-focused microkernel for the Banana Pi BPI-R4 (MT7988A SoC).

## Quick Links

| Document | Description |
|----------|-------------|
| [docs/PRINCIPLES.md](docs/PRINCIPLES.md) | **MUST READ** - Core design philosophy |
| [docs/README.md](docs/README.md) | Documentation index |
| [docs/architecture/](docs/architecture/) | System architecture |
| [docs/architecture/DRIVER_STACK.md](docs/architecture/DRIVER_STACK.md) | Zero-copy ring-based driver I/O |
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

### 4. ObjectService Owns All Tables

**Critical architecture**: Object tables are NOT in Task struct. ObjectService owns them.

```rust
pub struct ObjectService {
    tables: SpinLock<[Option<ObjectTable>; MAX_TASKS]>,
}

impl ObjectService {
    pub fn with_table<F, R>(&self, task_id: TaskId, f: F) -> Result<R>
    pub fn with_table_mut<F, R>(&self, task_id: TaskId, f: F) -> Result<R>
}
```

**Lock ordering** (always this order):
1. Scheduler lock (via `task::with_scheduler()`)
2. ObjectService tables lock (via `object_service().tables.lock()`)

**Pattern for syscall dispatch**:
```rust
// Get task ID from scheduler
let task_id = task::with_scheduler(|sched| sched.task(slot).map(|t| t.id));

// Access object table via ObjectService
object_service().with_table_mut(task_id, |table| {
    // Operate on table...
})?;
```

**Key files**:
- `src/kernel/object_service.rs` - ObjectService with tables
- `src/kernel/object/syscall.rs` - Syscall dispatch using ObjectService

### 5. No Tech Debt

- **Complete migrations only** - No half-finished rewrites
- **Deprecate completely** - Old code returns ENOSYS, then gets deleted
- **No backward compatibility shims** - Clean breaks, not cruft accumulation

### 6. Subscriber-Based Waking

```rust
pub struct Subscriber { task_id: u32, generation: u32 }
pub enum WakeReason { Readable, Writable, Closed, Accepted, Timeout }
```

- **Single wake mechanism** - Tasks subscribe to events, get woken
- **Generation prevents stale wakes** - Detect task slot reuse
- **Wake outside locks** - Collect subscribers under lock, wake after release

### 7. Bus Framework Is The Only Way To Write Drivers

All userspace drivers use the bus framework (`userlib/src/bus.rs`). No exceptions.

```rust
struct MyDriver { /* domain state */ }

impl Driver for MyDriver {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> { ... }
    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition { ... }
    fn data_ready(&mut self, port: PortId, ctx: &mut dyn BusCtx) { ... }
}

fn main() { driver_main(b"mydriver", MyDriver::new()); }
```

- **No hand-rolled event loops** - `driver_main()` owns the Mux and blocks on events
- **No direct IPC** - Drivers never import Mux, Channel, Port, DevdClient, or CommandRing
- **No raw syscalls for I/O** - All I/O goes through `BusCtx` and transport traits
- **Extend the framework, don't bypass it** - If a driver needs something missing (new data transport, new callback, new BusCtx method), update `bus.rs`/`bus_runtime.rs` — never work around it with ad-hoc IPC
- **Domain logic only** - A driver struct contains device state and hardware interaction, nothing else

---

## Key Principles (Summary)

See [docs/PRINCIPLES.md](docs/PRINCIPLES.md) for complete design philosophy.

- **SMP safe always** - Multi-core safe code required
- **State machine design** - Every boundary has explicit states
- **No busy-wait loops** - Use events, never poll
- **Minimal kernel** - Device management in userspace (devd)
- **DMA memory non-cacheable** - No cache flushes in driver code
- **Capability-based security** - Check permissions for privileged ops
- **Bus framework for all drivers** - `driver_main()` + `Driver` trait, no exceptions

---

## Build & Run

### Build Commands (cargo xtask)

```bash
# Full build (kernel + userspace + initrd)
./x build

# Build for QEMU virt target
./x build --platform qemu

# Build with self-tests enabled
./x build --test

# Build with MT7996 firmware embedded
./x build --firmware

# Build only specific user programs
./x build --only devd shell

# Run in QEMU
./x qemu

# Clean all build artifacts
./x clean
```

### Legacy Build Script

The `./build.sh` script is deprecated but still works for compatibility.

### Running on QEMU

```bash
# Build and run (recommended)
./x qemu

# Manual run
./x build --platform qemu
qemu-system-aarch64 -M virt,gic-version=3 -cpu cortex-a72 -m 512M -nographic -kernel kernel.bin
```

QEMU differences from real hardware:
- Loads kernel at 0x40080000 (vs 0x46000000 on MT7988A)
- Uses PL011 UART at 0x09000000 (vs 16550 at 0x11000000)
- Uses GICv3 (vs custom interrupt controller)
- No PCIe or USB enumeration (stubs only)

### Load via U-Boot

```
1. loady 0x46000000
2. (send kernel.bin via Xmodem)
3. go 0x46000000
```

### Hardware

**Physical hardware:**
- **Board**: Banana Pi BPI-R4
- **SoC**: MediaTek MT7988A
- **CPU**: ARM Cortex-A73 (AArch64)
- **USB**: SSUSB0 (IRQ 205, M.2), SSUSB1 (IRQ 204, USB-A via VL822)

**Emulation (QEMU virt):**
- **Machine**: virt with GICv3
- **CPU**: Cortex-A72 or max
- **RAM**: 512M+ recommended
- Useful for testing kernel/userspace without hardware

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

### Driver Stack (Zero-Copy I/O)

Block device drivers use **shared memory rings** for bulk data, not IPC:

```
┌─────────┐     ┌───────────┐     ┌─────────┐     ┌──────┐
│  fatfs  │ ←─→ │ partition │ ←─→ │  usbd   │ ←─→ │ xHCI │
└─────────┘     └───────────┘     └─────────┘     └──────┘
     ↑               ↑                 ↑
     └───────────────┴─────────────────┘
              Shared Memory Pool
         (DMA writes, apps read - zero copy)
```

- **DataPort API**: Ring + Sidechannel + Pool in shared memory
- **SQ/CQ carry offsets** into pool, not actual data
- **Sidechannel**: Control plane for queries (devd → driver chain)
- **IPC only for control**: Never for bulk data (576 byte limit)

See [docs/architecture/DRIVER_STACK.md](docs/architecture/DRIVER_STACK.md) for details.

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

### ObjectService Tables Migration (COMPLETE)
- **Object tables moved to ObjectService** - Not in Task struct anymore
  - `src/kernel/object_service.rs` owns all per-task object tables
  - Scheduler lock and tables lock are separate (proper lock ordering)
  - Task struct no longer has `object_table` field
- **All syscall dispatch uses ObjectService**
  - `object/syscall.rs` - read/write/map/close use `object_service().with_table[_mut]()`
  - Child exit notifications via `object_service().notify_child_exit()`
  - Timer checking via `object_service().check_timers_for_task()`

### Trait-Based Syscall Layer (COMPLETE)
- **Syscalls now go through trait boundaries** - Pure microkernel design
  - All scheduler access via `task::with_scheduler(|sched| { ... })`
  - No more `unsafe { task::scheduler() }` in syscall handlers
  - Traits define behavior, implementations wrap kernel internals
- **Trait hierarchy**
  - `SyscallContext` - Entry point, provides access to subsystems
  - `ObjectOps` - Unified 5-syscall interface (open/read/write/map/close)
  - `MemoryOps` - Memory management (mmap/munmap)
  - `ProcessOps` - Process lifecycle (exit/kill/spawn)
  - `UserAccess` - Safe user memory access

### Driver Stack Migration (IN PROGRESS)
See [docs/architecture/DRIVER_STACK.md](docs/architecture/DRIVER_STACK.md) for architecture.

Infrastructure complete (Phases 1-4):
- `userlib/src/ring.rs` - Ring protocol with IoSqe/IoCqe/SideEntry
- `userlib/src/data_port.rs` - DataPort API with Layer trait
- `devd/src/ports.rs` - Port hierarchy with parent/child
- `devd/src/rules.rs` - Auto-spawn rules for port events

Driver conversion TODO (Phases 5-6):
- [ ] Convert qemu-usbd to use DataPort
- [ ] Convert partition to use DataPort
- [ ] Convert fatfs to use DataPort
- [ ] Remove IPC-based block protocol

### Next Steps
1. Convert USB stack to DataPort ring protocol
2. Migrate Service framework (`userlib/src/service.rs`) to ipc2
3. Update devd to use ipc2 directly or updated Service
4. Update consoled to use ipc2
5. Update shell to use ipc2
6. Remove legacy syscalls after all userspace migrated

### MT7996 WiFi Driver
- Prefetch configuration fixed to match Linux exactly
- Cache coherency fix applied (kernel DMA pool flush)
- Awaiting hardware test with new serial adapter

---

## Changelog (Recent)

### 2026-01-27
- **Driver Stack Architecture Documentation**
  - Created `docs/architecture/DRIVER_STACK.md` documenting zero-copy ring-based I/O
  - DataPort = Ring + Sidechannel + Pool in shared memory
  - SQ/CQ carry offsets, not data (avoids 576-byte IPC limit)
  - Sidechannel for control plane queries (devd → driver chain)
- **QEMU USB Disk Support**
  - Added `./x qemu-usb` command to create FAT16 USB disk images
  - Pure Rust MBR + FAT16 creation (no external tools on macOS)
  - FAT16 partition at LBA 2048, auto-detected by partition driver
- **Temporary IPC workaround** (to be replaced by DataPort)
  - Limited block reads to 1 sector at a time to fit IPC payload
  - Affected: fatfs, partition, qemu-usbd

### 2026-01-23
- **Scheduler State Machine Fixes** - Critical context switching bugs fixed
  - **Fixed `context_saved` flag bug** (`src/kernel/sched.rs`)
    - Previously only set when `from_blocked`, causing yielding tasks to not have flag set
    - Now always set when entering context_switch branch, regardless of blocking state
    - Prevents bug where task's context was saved but flag wasn't set, causing next switch to skip restoration
  - **Fixed crash recovery** (`src/main.rs`)
    - Kill loop now starts from slot 1, preserving idle task in slot 0
    - Changed init detection from `pid == 1` (matches idle) to `is_init` flag (matches devd)
  - **Fixed exception handler idle handling** (`src/main.rs`)
    - When next task after termination is idle (slot 0), enter `idle_entry()` directly
    - Previously tried to eret to idle which failed (idle has no user trap frame)
    - Prevents cascade failure where devd crash caused idle to fault at PC=0x0
- **Scheduler HAL Redesign** - Proper idle task and context switching
  - Created HAL traits for CPU operations (`src/hal/cpu.rs`, `src/hal/context.rs`)
    - `Cpu` trait: idle (WFI), IRQ enable/disable, memory barriers, CPU ID
    - `Context` trait: kernel context for context switching
    - `ContextSwitch` trait: architecture-specific context switch
  - AArch64 HAL implementation (`src/arch/aarch64/hal.rs`)
    - `Aarch64Cpu`: WFI, DAIF manipulation for IRQs
    - `Aarch64Context`: callee-saved registers (x19-x30, sp)
    - `aarch64_context_switch`: assembly implementation
  - **Real idle task** (`src/kernel/idle.rs`)
    - Idle task is now a proper kernel task in slot 0
    - Runs WFI loop, has valid kernel context for context switching
    - Created before devd spawns (devd now in slot 1)
  - **Fixed blocking syscall context switching**
    - When task blocks, scheduler context switches to idle (slot 0)
    - Idle task runs WFI with IRQs enabled
    - Timer interrupt sets NEED_RESCHED flag
    - `do_resched_if_needed()` switches from idle to ready task
    - Blocked task's kernel context properly saved/restored
  - **Fixes busy loop issue** - Tasks no longer wake ~63 times/second
  - Design doc: `docs/architecture/SCHEDULER_REDESIGN.md`
- **QEMU virt platform now working** - Full boot to userspace on QEMU
  - Fixed linker script: QEMU loads kernel at 0x40080000, not 0x40000000
  - `src/linker-qemu.ld` updated with correct KERNEL_PHYS_BASE
  - Added L2 page tables (2MB blocks) for TTBR1 - more compatible than 1GB blocks
  - Boot sequence: `B1SPEJMK` then kernel initialization
  - Run with: `./x qemu` or manual: `qemu-system-aarch64 -M virt,gic-version=3 -cpu cortex-a72 -m 512M -nographic -kernel kernel.bin`
- **Boot code improvements** (`src/arch/aarch64/boot.S`)
  - Platform-conditional UART debug macros (MT7988A 16550 vs QEMU PL011)
  - Better exception handler with detailed ESR/ELR dump
  - Cleaner debug checkpoint sequence
- **Fixed Mux subscribe-poll race condition** - Critical IPC fix
  - Merged Phase 2 (subscribe) and Phase 3 (poll) into single atomic lock section
  - Previously: subscribe released lock, poll acquired separately - race window
  - Now: subscribe and poll happen in one `with_table_mut()` call
  - Fixes issue where port connection wake was lost if it arrived between subscribe and poll
  - File: `src/kernel/object/syscall.rs` - `read_mux_via_service()`
- **Unified Clock HAL** - Consistent time source across platforms
  - Added `now_ns()`, `deadline_ns()`, `is_expired()` to HAL Timer trait
  - All timer operations now use hardware counter in nanoseconds
  - Platforms: `src/platform/qemu_virt/timer.rs`, `src/platform/mt7988/timer.rs`
- **ObjectService tables migration complete** - Phase 4-6 of syscall rebuild
  - Object tables moved from Task struct to ObjectService
  - `src/kernel/object_service.rs` owns `BTreeMap<TaskId, ObjectTable>` with its own lock
  - All syscall dispatch (read/write/map/close) now uses `object_service().with_table[_mut]()`
  - Child exit notifications migrated to `object_service().notify_child_exit()`
  - Timer checking migrated to `object_service().check_timers_for_task()`
  - `object_table` field removed from Task struct
- **Lock ordering established**: scheduler lock first, then ObjectService tables lock
- **Cleanup**: Removed dead `read_mux_impl`, unused IPC backend re-exports

### 2026-01-22
- **Trait-based syscall layer refactoring** - Pure microkernel design
  - All syscall handlers now use trait boundaries
  - Replaced `unsafe { task::scheduler() }` with `task::with_scheduler(|sched| { ... })`
  - Syscalls are thin dispatchers: validate args, call trait method, return result
- **New trait files** (`src/kernel/traits/`)
  - `syscall_ctx.rs` - SyscallContext entry point trait
  - `object_ops.rs` - ObjectOps for unified 5-syscall interface
  - `memory_ops.rs` - MemoryOps for mmap/munmap
  - `process_ops.rs` - ProcessOps for exit/kill/spawn
  - `user_access.rs` - UserAccess for safe user memory access
- **New implementation files**
  - `syscall_ctx_impl.rs` - KernelSyscallContext
  - `object_ops_impl.rs` - KernelObjectOps
  - `memory_ops_impl.rs` - KernelMemoryOps
  - `process_ops_impl.rs` - KernelProcessOps
  - `user_access_impl.rs` - KernelUserAccess
- **Unsafe block reduction**
  - `object/syscall.rs`: 30 unsafe blocks -> 3 (inherent: slice conversions)
  - `syscall/memory.rs`: 4 unsafe blocks -> 0
  - `syscall/misc.rs`: Inherent unsafe only (asm, raw statics)
  - `syscall/process.rs`: Inherent unsafe only (update_current_task_globals, asm)

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

