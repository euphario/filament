# Master Plan: Kernel Technical Debt & Migration

## Overview

This plan consolidates all identified technical debt and migration work into a prioritized, actionable roadmap. Work is organized into phases that can be completed incrementally.

---

## Current State Summary

### Module Sizes (Lines of Code)

| Module | LOC | Status |
|--------|-----|--------|
| task/mod.rs | ~2,100 | **Too large** - needs splitting |
| ipc/ (all files) | ~2,614 | Internal IPC |
| ~~handle/ (all files)~~ | ~~1,681~~ | **REMOVED** (Jan 22, 2026) |
| object/ (all files) | ~2,430 | Unified syscalls (now primary) |
| syscall/ (all files) | ~2,178 | Mixed concerns |
| ~~scheme.rs~~ | ~~1,817~~ | **REMOVED** (Jan 22, 2026) |
| ~~fd.rs~~ | ~~200~~ | **REMOVED** (Jan 22, 2026) |
| irq.rs | ~200 | NEW - IRQ handling extracted from scheme.rs |

### Completed Work

- [x] Made Task/Scheduler fields private with accessor methods
- [x] Made Object struct fields private with accessors
- [x] Made IPC internal table access functions private
- [x] Added comprehensive API documentation to key modules
- [x] Cleaned up unused imports/re-exports (14 warnings)
- [x] **Removed legacy handle/ module** (~1,681 LOC removed)
- [x] **Removed handle_table from Task struct**
- [x] **Migrated child exit notification to object_table**
- [x] **Removed fd_table from Task struct** (Jan 22, 2026)
- [x] **Removed fd.rs module** (~200 LOC removed, Jan 22, 2026)
- [x] **Removed scheme.rs module** (~1,817 LOC removed, Jan 22, 2026)
- [x] **Created irq.rs module** (~200 LOC, extracted IRQ handling from scheme.rs)
- [x] **Split task module** (Jan 22, 2026) - Created tcb.rs (842 LOC) from mod.rs
- [x] **Phase 4 verified complete** (Jan 22, 2026) - Syscalls already thin wrappers
- [x] **Phase 5 verified complete** (Jan 22, 2026) - Bus module already well-organized

---

## Phase 1: Driver Migration to ipc2 (HIGH PRIORITY)

**Goal**: Migrate disabled drivers to the unified 5-syscall interface (ipc2).

**Why First**: Drivers are the main consumers of the IPC system. Migrating them:
- Validates the ipc2 design with real use cases
- Enables removal of legacy IPC code
- Unblocks Phase 2 (IPC consolidation)

### Drivers to Migrate

| Driver | Status | Dependencies | Effort |
|--------|--------|--------------|--------|
| logd | Disabled | None | Small |
| gpio | Disabled | devd | Small |
| pwm | Disabled | devd | Small |
| nvmed | Disabled | pcied | Medium |
| pciepoke | Disabled | pcied | Small |
| pcied | Disabled | devd | Medium |
| fatfs | Disabled | usbd | Medium |
| usbd | Disabled | devd | Large |
| vfsd | Disabled | fatfs | Medium |
| wifid | Disabled | devd, pcied | Large |

### Migration Pattern

Each driver migration follows this pattern:

```rust
// Before (old IPC)
use userlib::ipc::{Channel, Message};
let chan = Channel::connect("devd")?;
chan.send(&msg)?;
let reply = chan.recv()?;

// After (ipc2)
use userlib::ipc2::{Channel, Port, Message};
let port = Port::connect("devd")?;  // Named service lookup
let chan = port.accept()?;          // Get channel from port
chan.send(&msg)?;
let reply = chan.recv()?;
```

### Suggested Order

1. **logd** - Simplest, no dependencies, good test case
2. **gpio, pwm** - Simple drivers, validate pattern
3. **pciepoke** - Debug tool, quick win
4. **pcied** - Required by nvmed, wifid
5. **nvmed** - Disk driver, tests data path
6. **fatfs** - Filesystem, depends on usbd/nvmed
7. **usbd** - Complex USB stack, thorough test
8. **vfsd** - VFS layer, depends on fatfs
9. **wifid** - Most complex, last

---

## Phase 2: IPC Consolidation (COMPLETE)

**Goal**: Merge three IPC systems into one unified implementation.

**Status**: COMPLETE - All legacy IPC code removed. Only object/ system remains.

### Current Architecture (After Consolidation)

```
┌─────────────────────────────────────────────────────┐
│ object/syscall.rs                                   │
│ (unified 5-syscall interface)                       │
├─────────────────────────────────────────────────────┤
│ object/mod.rs                                       │
│ (Object types + Pollable trait)                     │
├─────────────────────────────────────────────────────┤
│ ipc/                                                │
│ (internal: channels, ports, queues)                 │
├─────────────────────────────────────────────────────┤
│ irq.rs                                              │
│ (IRQ registration for userspace drivers)            │
└─────────────────────────────────────────────────────┘
```

### Completed Steps

1. ~~Remove handle/ syscalls~~ - **DONE** (module deleted)
2. ~~Remove handle_table from Task~~ - **DONE** (now only object_table)
3. ~~Migrate child exit notification~~ - **DONE** (uses object::notify_child_exit)
4. ~~Remove fd_table from Task~~ - **DONE** (Jan 22, 2026)
5. ~~Remove fd.rs module~~ - **DONE** (Jan 22, 2026)
6. ~~Remove scheme.rs module~~ - **DONE** (Jan 22, 2026, IRQ handling moved to irq.rs)

### Files Removed

| File | LOC |
|------|-----|
| handle/channel.rs | 116 |
| handle/port.rs | 330 |
| handle/timer.rs | 131 |
| handle/console.rs | 121 |
| handle/klog.rs | 69 |
| handle/child_exit.rs | 175 |
| handle/shmem.rs | 94 |
| handle/mod.rs | 457 |
| handle/traits.rs | 186 |
| fd.rs | ~200 |
| scheme.rs | ~1,817 |
| **Total removed** | **~1,681** |

---

## Phase 3: Task Module Splitting (COMPLETE)

**Goal**: Split 2,145 LOC task/mod.rs into focused submodules.

**Status**: COMPLETE - Task module split into focused submodules (Jan 22, 2026).

### Current Structure

```
task/
├── mod.rs          # Scheduler + global functions (1,193 LOC)
├── tcb.rs          # Task struct, contexts, assembly (842 LOC)
├── state.rs        # TaskState enum, state machine (396 LOC)
├── lifecycle.rs    # Task lifecycle operations (396 LOC)
└── policy.rs       # Scheduling policy trait (189 LOC)
```

### Actual LOC Distribution

| File | LOC | Contents |
|------|-----|----------|
| mod.rs | 1,193 | Scheduler struct, global statics, re-exports |
| tcb.rs | 842 | Task struct, TrapFrame, CpuContext, TimerDesc, assembly |
| state.rs | 396 | TaskState, SleepReason, WaitReason |
| lifecycle.rs | 396 | exit(), kill(), wait_child(), wake() |
| policy.rs | 189 | SchedulingPolicy trait, PriorityRoundRobin |

### Completed Steps

1. ~~Extract state.rs~~ - **DONE** (already existed)
2. ~~Extract tcb.rs~~ - **DONE** (Jan 22, 2026)
3. ~~Extract lifecycle.rs~~ - **DONE** (already existed)
4. ~~Update mod.rs~~ - **DONE** (Jan 22, 2026)

---

## Phase 4: Syscall Layer Cleanup (COMPLETE)

**Goal**: Make syscall handlers thin wrappers over domain modules.

**Status**: COMPLETE - Syscalls already follow the target pattern (Jan 22, 2026).

### Current Architecture (Already Correct)

Syscall handlers are thin wrappers that:
1. Validate user pointers via `uaccess` module
2. Check capabilities via `require_capability()`
3. Call domain functions (lifecycle, shmem, pci, bus)
4. Handle scheduling and return results

### Syscall File Structure

| File | LOC | Purpose |
|------|-----|---------|
| syscall/process.rs | 535 | Process lifecycle (delegates to task/lifecycle) |
| syscall/misc.rs | 509 | Utility syscalls (yield, time, logging) |
| syscall/memory.rs | 262 | Memory mapping (delegates to task.mmap_*) |
| syscall/pci.rs | 237 | PCI operations (delegates to pci/bus modules) |
| syscall/shmem.rs | 167 | Shared memory (delegates to shmem module) |

---

## Phase 5: Bus Module Organization (COMPLETE)

**Goal**: Separate bus abstraction from hardware-specific code.

**Status**: COMPLETE - Clean separation already exists (Jan 22, 2026).

### Current Structure (Already Correct)

```
kernel/bus/
├── mod.rs         # 529 LOC - BusState, re-exports
├── types.rs       # 178 LOC - BusType, BusInfo, DeviceInfo
├── protocol.rs    # 131 LOC - StateChangeReason, protocol types
├── controller.rs  # 740 LOC - BusController state machine
├── hw_pcie.rs     # 293 LOC - MT7988A PCIe hardware ops
└── hw_usb.rs      # 320 LOC - MT7988A USB/xHCI hardware ops
```

The bus module cleanly separates:
- **Abstract layer**: types.rs, protocol.rs, controller.rs
- **Hardware layer**: hw_pcie.rs, hw_usb.rs (MT7988A-specific)

---

## Implementation Timeline

### Immediate (This Week)

- [ ] Start logd migration to ipc2
- [ ] Start gpio, pwm migrations

### Short Term (2-4 Weeks)

- [ ] Complete all simple driver migrations (logd, gpio, pwm, pciepoke)
- [ ] Migrate pcied to ipc2
- [ ] Start nvmed, fatfs migrations

### Medium Term (1-2 Months)

- [ ] Complete all driver migrations
- [ ] Begin IPC consolidation (Phase 2)
- [ ] Split task module (Phase 3)

### Long Term (As Needed)

- [ ] Complete IPC consolidation
- [ ] Syscall layer cleanup (Phase 4)
- [ ] Bus module organization (Phase 5)

---

## Verification Criteria

### For Each Phase

1. **Build passes** - `./build.sh` succeeds
2. **Boot works** - Kernel boots, devd/consoled/shell work
3. **No regressions** - Existing functionality preserved
4. **Tests pass** - Unit tests and integration tests pass

### For Driver Migrations

- Driver builds and loads
- Basic functionality works (e.g., gpio can toggle pins)
- IPC communication with devd works
- No use of deprecated ipc module

### For IPC Consolidation

- All handle/ syscalls return ENOSYS
- No code references handle/ module
- Unified syscalls (100-104) work for all object types

---

## Risk Mitigation

### Risk: Breaking Boot

**Mitigation**: Keep legacy syscalls working until all userspace migrated. Migration is incremental - one driver at a time.

### Risk: IPC Performance Regression

**Mitigation**: Benchmark before/after. ipc2 uses same internal ipc/ implementation, so performance should be identical.

### Risk: Complex Driver Breaks (usbd, wifid)

**Mitigation**: Migrate simple drivers first to validate pattern. Complex drivers last, with careful testing.

---

## Quick Reference: What to Do Next

1. **Pick a disabled driver** from Phase 1 list (suggest: logd)
2. **Update its IPC imports** to use `userlib::ipc2`
3. **Update its code** to use new Channel/Port/Timer APIs
4. **Enable it in build.sh** - Move from disabled to enabled
5. **Test** - Ensure it builds, loads, and works
6. **Repeat** for next driver

---

## Appendix: File Reference

### Kernel IPC Files (current structure)

```
src/kernel/
├── ipc/
│   ├── mod.rs       # ~499 LOC - Main IPC API
│   ├── table.rs     # ~433 LOC - Channel table
│   ├── port.rs      # ~476 LOC - Port registry
│   ├── types.rs     # ~240 LOC - Message types
│   ├── queue.rs     # ~247 LOC - Message queue
│   ├── waker.rs     # ~398 LOC - Wake mechanism
│   ├── error.rs     # ~52 LOC - Error types
│   └── traits.rs    # ~269 LOC - Pollable trait
│
├── object/
│   ├── mod.rs       # ~965 LOC - Object types + notify_child_exit
│   ├── syscall.rs   # ~1,206 LOC - Syscall handlers
│   ├── handle.rs    # ~178 LOC - Handle table
│   └── types.rs     # ~38 LOC - Type exports
│
├── irq.rs           # ~200 LOC - IRQ registration for userspace drivers
│
│   # REMOVED (Jan 22, 2026):
│   # handle/         - Legacy handle system (~1,681 LOC)
│   # fd.rs           - Legacy file descriptors (~200 LOC)
│   # scheme.rs       - Legacy kernel schemes (~1,817 LOC)
```

### Userspace IPC Files (for migration reference)

```
user/userlib/src/
├── ipc.rs           # Old IPC (deprecated)
├── ipc2.rs          # New unified IPC (use this)
└── service.rs       # Service framework (needs migration)
```
