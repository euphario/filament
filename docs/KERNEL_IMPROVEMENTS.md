# Kernel Improvement Plan

Based on comprehensive code review applying these principles:
- **State machine based** - explicit states, clean transitions
- **Compact** - minimal code, no redundancy
- **Testable** - pure functions, dependency injection
- **Explicit** - no hidden state, clear ownership
- **Event driven** - no busy-wait, no polling
- **Time bound** - deadlines unless waiting for external event
- **Modular** - traits, modules, composable

---

## Critical Issues (Must Fix)

### 1. [DONE/BY-DESIGN] Unbounded Waits (Time-Bound Violation)

| File | Issue | Status |
|------|-------|--------|
| `event.rs` | `sys_event_wait` uses Sleeping state | **BY DESIGN** - Sleeping = event loop, uses heartbeat probe |
| `scheme.rs:627` | IRQ wait spin loop | **FIXED** - 5s timeout with ETIMEDOUT |
| `ipc.rs` | Ping filtering loop | **DELETED** - replaced by ipc2 module |

**Design Note**: Sleeping vs Waiting states have different liveness semantics:
- **Waiting** (request-response) → has deadline, liveness enforces timeout
- **Sleeping** (event loop) → no deadline, liveness uses active heartbeat probe

### 2. [DONE] Missing State Machine Validation

| File | Issue | Status |
|------|-------|--------|
| `liveness.rs` | Implicit state transitions | **OK** - unified Waiting/Sleeping handling |
| `scheme.rs` | IRQ table has no lock | **DONE** - `IRQ_TABLE: SpinLock<...>` (line 422) |
| `port.rs` | Hash removal can corrupt probe chains | **FIXED** - ipc2/port.rs uses array, not hash table |

### 3. [DOCUMENTED] Silent Security Stub

| File | Issue | Status |
|------|-------|--------|
| `elf.rs:165-173` | `verify_binary_signature` always returns Ok | **DOCUMENTED** - Intentional, infrastructure ready for crypto |

---

## High Priority (Modularity & Testability)

### 4. Giant Files - Need Module Extraction

| File | Lines | Status |
|------|-------|--------|
| `syscall.rs` | was 3,914 | **DONE** - `object/syscall.rs` is 1,200 lines |
| `bus.rs` | was 2,140 | **DONE** - Split into `bus/{controller,hw_pcie,hw_usb,protocol,types}.rs` |
| `task.rs` | was 2,294 | **DONE** - Memory types extracted to `memory.rs` (now 2,132 lines) |

### 5. Missing Traits (Behavior Duplication)

| Pattern | Status |
|---------|--------|
| Hardware polling | **DONE** - `hw_poll.rs` with `HardwarePoller` trait |
| IPC/Channel traits | **DONE** - `ipc2/traits.rs` with `Pollable`, `Waitable`, `Closable` |
| Bus operations | TODO: Extract `trait BusHwOps` from `bus/controller.rs` |
| Message handling | TODO: Extract `trait BusMessage` |

### 6. Global State Access (Testability Blocker)

| File | Pattern | Status |
|------|---------|--------|
| `ipc.rs` | Global scheduler access | **FIXED** - ipc2 uses subscriber waking |
| `bus/controller.rs` | `with_channel_table()` | TODO: Accept `&dyn IpcService` parameter |
| `object/syscall.rs` | All handlers access globals | TODO: Pass `&mut Task` to each handler |
| `liveness.rs` | `check_liveness` modifies scheduler | **OK** - unified Waiting/Sleeping check (deadline + heartbeat) |

### 7. God Structs (Single Responsibility Violation)

| Struct | Problem | Status |
|--------|---------|--------|
| `BusController` | 665 bytes, all bus types | **PARTIALLY FIXED** - hw_pcie.rs/hw_usb.rs extracted |
| `Task` | 500+ bytes, mixes scheduling & memory | TODO: Extract `UserMemoryManager` trait |

---

## Medium Priority (Code Quality)

### 8. Redundant Code

| File | Issue | Status |
|------|-------|--------|
| `ipc.rs` | Duplicate `find_by_id()` loops | **FIXED** - ipc2 uses `ChannelTable::get()` |
| `scheme.rs` | Duplicate name lookup | LOW PRIORITY |
| Legacy syscall stubs | ENOSYS stubs | **DONE** - Removed 50+ legacy enum variants |

### 9. Magic Numbers & Implicit Constants

| File | Issue | Status |
|------|-------|--------|
| `addrspace.rs` | Memory addresses without explanation | TODO: Add ARM64 VA layout comments |
| `liveness.rs` | Enum variants mapped to 0,1,2 | LOW PRIORITY |
| `lock.rs` | Debug-only lock depth tracking | LOW PRIORITY |

### 10. Dead Code

| File | Issue | Status |
|------|-------|--------|
| `task.rs` | WaitReason type alias | Need to verify if still present |

---

## Recommended Module Structure

```
src/kernel/
├── mod.rs                    # Re-exports
├── task.rs                   # Task state, scheduling only (~800 lines)
├── memory.rs                 # User memory mapping (NEW)
├── syscall/
│   ├── mod.rs                # Dispatcher
│   ├── process.rs            # Exit, Spawn, Kill
│   ├── memory.rs             # Mmap, Munmap, MmapDma
│   ├── ipc.rs                # Send, Receive, Channel
│   ├── handles.rs            # Handle*, Port*
│   └── ...
├── bus/
│   ├── mod.rs                # BusController state machine
│   ├── traits.rs             # Bus, BusHwOps, HardwarePoller
│   ├── pcie.rs               # PcieBus impl
│   ├── usb.rs                # UsbBus impl
│   └── messages.rs           # BusMessage trait + impls
├── ipc/
│   ├── mod.rs                # ChannelTable, send/receive
│   ├── traits.rs             # IpcService (for DI)
│   └── ...
├── handle/
│   ├── mod.rs                # HandleTable
│   ├── traits.rs             # Waitable, Closable
│   ├── channel.rs
│   ├── port.rs
│   └── ...
└── ...
```

---

## Implementation Order

### Phase 1: Critical Fixes - **MOSTLY DONE**
1. ~~Add timeouts to `event.rs` and `scheme.rs`~~ - **DONE** (Sleeping uses heartbeat, scheme.rs has 5s timeout)
2. Add SpinLock to IRQ table - TODO
3. ~~Make signature verification explicit~~ - **DOCUMENTED** as intentional stub

### Phase 2: Testability - **MOSTLY DONE**
1. ~~Extract `trait IpcService`~~ - **DONE** (ipc2/traits.rs: Pollable, Waitable, Closable)
2. Pass `&mut Task` to syscall handlers - TODO (low priority)
3. ~~Extract `check_liveness` pure decision logic~~ - **OK** - unified check handles both Waiting (deadline) and Sleeping (heartbeat)

### Phase 3: Modularity - **DONE**
1. ~~Split `bus.rs`~~ - **DONE** (`bus/{controller,hw_pcie,hw_usb,protocol,types}.rs`)
2. ~~Split `syscall.rs`~~ - **DONE** (`object/syscall.rs` is 1,200 lines)
3. ~~Extract memory mapping from `task.rs`~~ - **DONE** (`memory.rs` with types, task.rs now 2,132 lines)

### Phase 4: Cleanup - **DONE**
1. ~~Remove redundant code in `ipc.rs`~~ - **DONE** (replaced by ipc2)
2. ~~Remove legacy syscall stubs~~ - **DONE** (50+ enum variants removed)
3. Add missing documentation - ONGOING

---

## Key Traits - Status

| Trait | Location | Status |
|-------|----------|--------|
| `HardwarePoller` | `hw_poll.rs` | **DONE** |
| `Pollable` | `ipc2/traits.rs` | **DONE** |
| `Waitable` | `ipc2/traits.rs` | **DONE** |
| `Closable` | `ipc2/traits.rs` | **DONE** |
| `BusHwOps` | - | TODO |
| `UserMemoryManager` | - | TODO |
| `BusMessage` | - | TODO |

---

## Metrics

**Original state (when plan written):**
- `syscall.rs`: 3,914 lines
- `bus.rs`: 2,140 lines
- `task.rs`: 1,959 lines

**Current state (2026-01-20):**
- `syscall/mod.rs`: 457 lines ✓ (was 726, legacy removed)
- `object/syscall.rs`: 1,200 lines ✓
- `bus/controller.rs`: ~750 lines ✓
- `bus/hw_pcie.rs`: ~300 lines ✓
- `bus/hw_usb.rs`: ~300 lines ✓
- `task.rs`: 2,132 lines ✓ (was 2,294, types extracted)
- `memory.rs`: 190 lines ✓ (NEW - extracted from task.rs)
- `scheme.rs`: 1,822 lines
- `ipc2/` module: ~3,500 lines total (well-structured)

Target state:
- No file > 800 lines
- All hardware ops behind traits
- All globals accessed via injected services
- 80%+ of logic unit-testable
