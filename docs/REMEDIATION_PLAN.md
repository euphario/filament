# Kernel Remediation Plan

## Goal

Bring the kernel to production-ready state by systematically addressing all violations of stated design principles.

**Exit Criteria**: All items GREEN except Stress Test and Security Audit.

---

## Current State (Baseline)

| Metric | Value | Target |
|--------|-------|--------|
| Public API surface | 806 items | Minimize |
| Test coverage | ~13% (90 tests / 666 pub fn) | >50% |
| Panicking unwraps | 95 | 0 |
| Explicit panics | 12 | 0 (or justified) |
| TODOs | 10 | 0 |
| Direct state assignments | ~20 | 0 |
| Unsafe global accessors | 4 (`scheduler()`, `bus_registry()`, etc.) | 0 |
| Unsafe blocks | 193 | Minimize + audit |
| Kernel LOC | 32,563 | Reduce |

---

## Phase 0: Dead Code Removal

**Goal**: Remove all unused code before investing effort in fixing it.

### 0.1 Compiler Warnings
```bash
cargo build --release 2>&1 | grep -E "warning.*unused|warning.*dead_code|warning.*never used"
```

- [ ] Enable `#![warn(dead_code)]` in lib.rs
- [ ] Remove all unused functions
- [ ] Remove all unused structs/enums
- [ ] Remove all unused imports
- [ ] Remove `#[allow(dead_code)]` after cleanup

### 0.2 Unreachable Code Paths
- [ ] Audit all `#[allow(dead_code)]` annotations
- [ ] Remove code paths that can never execute
- [ ] Remove feature-gated code that will never be enabled

### 0.3 Duplicate Functionality
- [ ] Find functions that do the same thing
- [ ] Consolidate into single implementation
- [ ] Remove duplicates

**Verification**:
```bash
cargo build --release 2>&1 | grep -c "warning"
# Target: 0 warnings
```

---

## Phase 1: State Machine Enforcement

**Goal**: All state changes go through validated transition methods. No direct assignment.

### 1.1 Audit Direct State Assignments

Current violations:
```
src/kernel/shmem.rs:462     task.state = ...
src/kernel/shmem.rs:651     region.state = RegionState::Dying
src/kernel/liveness.rs:258  task.state = TaskState::Ready
src/kernel/bus/controller.rs:66,147,326,705,730  self.state = ...
src/kernel/bus/mod.rs:270,281,290,353  bus.state = ...
src/kernel/event.rs:371,387,636,742  task.state = ..., sub.state = ...
src/kernel/object/mod.rs:266,367,374,389  self.state = ...
```

### 1.2 For Each State Enum, Create Transition Methods

**Pattern**:
```rust
impl TaskState {
    /// Transition to Ready state
    /// Returns Err if transition is invalid from current state
    pub fn to_ready(&mut self) -> Result<(), InvalidTransition> {
        match self {
            TaskState::Sleeping { .. } | TaskState::Waiting { .. } => {
                *self = TaskState::Ready;
                Ok(())
            }
            _ => Err(InvalidTransition { from: *self, to: "Ready" })
        }
    }
}
```

### 1.3 State Enums to Fix

| Enum | File | Transition Methods Needed |
|------|------|---------------------------|
| `TaskState` | task/state.rs | sleep(), wait(), wake(), exit(), schedule() |
| `ChannelState` | object/mod.rs | close(), half_close() |
| `PortState` | object/mod.rs | close() |
| `TimerState` | object/mod.rs | arm(), disarm(), fire() |
| `BusState` | bus/mod.rs | reset(), safe(), activate() |
| `BusMasteringState` | bus/controller.rs | enable(), disable() |
| `RegionState` | shmem.rs | map(), unmap(), die() |
| `SubscriptionState` | event.rs | activate(), deactivate() |

### 1.4 Replace All Direct Assignments

For each violation:
1. Replace `x.state = NewState` with `x.state.to_new_state()?`
2. Handle the Result (propagate or log)
3. Add `#[deny(direct_state_assignment)]` lint if possible

**Verification**:
```bash
grep -rn "\.state = " src/kernel/ | grep -v "fn \|impl \|// " | wc -l
# Target: 0
```

---

## Phase 2: Encapsulation (No Leaky APIs)

**Goal**: No raw access to internal state. All access through controlled APIs.

### 2.1 Remove Unsafe Global Accessors

| Accessor | File | Replacement |
|----------|------|-------------|
| `scheduler()` | task/mod.rs | `with_scheduler(f)` closure |
| `bus_registry()` | bus/mod.rs | `with_bus_registry(f)` closure |
| `subsystem_mut()` | pci/mod.rs | `with_pci(f)` closure |

**Pattern**:
```rust
// REMOVE
pub unsafe fn scheduler() -> &'static mut Scheduler { ... }

// REPLACE WITH
pub fn with_scheduler<R, F: FnOnce(&mut Scheduler) -> R>(f: F) -> R {
    let _guard = IrqGuard::new();
    unsafe { f(&mut SCHEDULER) }
}
```

### 2.2 Make Struct Fields Private

For each major struct, make fields private and add accessor methods:

| Struct | File | Current | Target |
|--------|------|---------|--------|
| `Task` | task/tcb.rs | All pub | id, parent_id pub; rest private |
| `Scheduler` | task/mod.rs | tasks pub | All private |
| `Object` | object/mod.rs | Variant fields pub | All private |
| `Channel` | ipc/channel.rs | Fields pub | All private |
| `ChannelTable` | ipc/table.rs | Exposed via closure | Private, typed API |

### 2.3 Remove `entries_mut()` Style Iterators

These expose internal state:
```rust
// REMOVE patterns like:
pub fn entries_mut(&mut self) -> impl Iterator<Item = &mut Entry>

// REPLACE with specific operations:
pub fn for_each_matching<F>(&mut self, filter: impl Fn(&Entry) -> bool, action: F)
```

### 2.4 Visibility Audit

```bash
grep -rn "^pub " src/kernel/ | wc -l
```

For each `pub` item:
- Should it be `pub(crate)`?
- Should it be `pub(super)`?
- Should it be private?

**Verification**:
```bash
# No raw scheduler/bus_registry access
grep -rn "scheduler()\|bus_registry()\|subsystem_mut()" src/kernel/ | grep -v "^.*:.*fn " | wc -l
# Target: 0
```

---

## Phase 3: Complete Unified Syscall Interface

**Goal**: All resource types accessible through open/read/write/map/close.

### 3.1 Implement Missing `open_*` Functions

| Object Type | File | Status | Action |
|-------------|------|--------|--------|
| Channel | object/syscall.rs | Done | - |
| Port | object/syscall.rs | Done | - |
| Timer | object/syscall.rs | Done | - |
| Mux | object/syscall.rs | Done | - |
| Console | object/syscall.rs | Done | - |
| Process | object/syscall.rs | Done | - |
| **Shmem** | object/syscall.rs | TODO | Implement |
| **DmaPool** | object/syscall.rs | TODO | Implement |
| **Mmio** | object/syscall.rs | TODO | Implement |
| **Klog** | object/syscall.rs | TODO | Implement |
| **PciBus** | object/syscall.rs | TODO | Implement |

### 3.2 Implement Missing `close` Cleanup

```
src/kernel/object/syscall.rs:1195:    // TODO: Unmap, release (Shmem)
src/kernel/object/syscall.rs:1200:    // TODO: Unmap, release (DmaPool)
src/kernel/object/syscall.rs:1205:    // TODO: Unmap, release (Mmio)
```

### 3.3 Wire to Existing Subsystems

- `open_shmem` -> `shmem::create_region()`
- `open_dma_pool` -> `dma_pool::allocate()`
- `open_mmio` -> `task.mmap_device()`
- `open_klog` -> New klog handle type
- `open_pci_bus` -> `pci::claim_bus()`

**Verification**:
```bash
grep -rn "TODO" src/kernel/object/syscall.rs | wc -l
# Target: 0
```

---

## Phase 4: Eliminate Panics

**Goal**: Kernel never panics on recoverable errors.

### 4.1 Audit All `unwrap()` Calls

```bash
grep -rn "\.unwrap()" src/kernel/ > /tmp/unwraps.txt
wc -l /tmp/unwraps.txt
# Current: 95
```

For each unwrap:
1. Can this ever fail? If no, add comment explaining why
2. If yes, replace with proper error handling

**Replacement patterns**:
```rust
// Option that should never be None (invariant)
let x = opt.expect("invariant: X is always set after init");

// Option that can fail -> propagate
let x = opt.ok_or(Error::NotFound)?;

// Option in infallible context -> handle gracefully
let x = match opt {
    Some(v) => v,
    None => {
        kerror!("unexpected None");
        return default;
    }
};
```

### 4.2 Audit All `expect()` Calls

Same process. Ensure message explains the invariant.

### 4.3 Audit Explicit `panic!()` Calls

```
src/kernel/task/mod.rs:754     panic!("run_user_task: invalid task")
src/kernel/ipc/error.rs:285    panic!("wrong error type")
src/kernel/ipc/error.rs:298    panic!("wrong error type")
src/kernel/ipc/table.rs:458    panic!("wrong error type")
src/kernel/ipc/traits.rs:355   panic!("wrong action")
src/kernel/lock.rs:53,67,141   panic!(deadlock/underflow)
src/kernel/lock.rs:328         panic!(...)
src/kernel/percpu.rs:208       panic!("CPU ID exceeds MAX_CPUS")
```

For each:
- Is this a true invariant violation (bug)? Keep but document
- Is this recoverable? Replace with Result

### 4.4 Audit `unreachable!()` Calls

These should be truly unreachable. If not, replace with error handling.

**Verification**:
```bash
grep -rn "unwrap()\|\.expect(\|panic!\|unreachable!" src/kernel/ | wc -l
# Target: <20 (only true invariants)
```

---

## Phase 5: Test Coverage

**Goal**: >50% test coverage on public API.

### 5.1 Prioritize by Risk

| Module | LOC | Tests | Priority |
|--------|-----|-------|----------|
| object/syscall.rs | 1,206 | 0 | HIGH |
| task/mod.rs | 1,193 | 0 | HIGH |
| object/mod.rs | 1,001 | 0 | HIGH |
| task/tcb.rs | 842 | 0 | HIGH |
| event.rs | 818 | 0 | HIGH |
| shmem.rs | 812 | 0 | MEDIUM |
| ipc/port.rs | 787 | ? | MEDIUM |
| bus/controller.rs | 740 | 0 | MEDIUM |
| ipc/table.rs | 569 | Yes | LOW (has tests) |

### 5.2 Test Categories

1. **Unit tests** (in-module `#[cfg(test)]`)
   - State machine transitions
   - Error conditions
   - Edge cases

2. **Integration tests** (separate test crate)
   - Multi-module interactions
   - Syscall sequences
   - Resource lifecycle

### 5.3 Test Template

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_transition_valid() {
        let mut state = TaskState::Ready;
        assert!(state.to_running().is_ok());
        assert!(matches!(state, TaskState::Running));
    }

    #[test]
    fn test_state_transition_invalid() {
        let mut state = TaskState::Dead;
        assert!(state.to_running().is_err());
    }
}
```

### 5.4 Coverage Target Per Module

| Module | Target Tests |
|--------|--------------|
| task/state.rs | 10 (all transitions) |
| task/tcb.rs | 15 (Task methods) |
| task/mod.rs | 20 (Scheduler methods) |
| object/mod.rs | 20 (Object state machines) |
| object/syscall.rs | 30 (each syscall path) |
| event.rs | 15 (subscribe/wake/timeout) |
| shmem.rs | 10 (create/map/unmap/cleanup) |
| bus/*.rs | 15 (state machine, controller) |
| ipc/*.rs | 20 (already has some) |

**Verification**:
```bash
cargo test 2>&1 | grep -E "^test.*ok|^test.*FAILED" | wc -l
# Target: >150 tests
```

---

## Phase 6: Unsafe Audit

**Goal**: Minimize unsafe, document all remaining.

### 6.1 Categorize Unsafe Blocks

```bash
grep -rn "unsafe {" src/kernel/ > /tmp/unsafe.txt
```

Categories:
1. **FFI/Assembly** - Required, document
2. **Raw pointer access** - Can it use safe abstraction?
3. **Global mutable state** - Can it use locking?
4. **Transmute/casting** - Is it necessary?

### 6.2 Reduce Unsafe Surface

For each unsafe block:
1. Can the unsafe be pushed into a smaller function?
2. Can a safe wrapper be created?
3. Is the invariant documented?

**Pattern**:
```rust
// Before: large unsafe block
unsafe {
    let ptr = get_ptr();
    // ... 20 lines of code ...
    *ptr = value;
}

// After: minimal unsafe
fn write_ptr(ptr: *mut T, value: T) {
    // SAFETY: ptr is valid because [invariant]
    unsafe { *ptr = value; }
}
// ... safe code ...
write_ptr(ptr, value);
```

### 6.3 Document All Remaining Unsafe

Every `unsafe` block must have a `// SAFETY:` comment explaining:
1. What invariant makes this safe
2. Who is responsible for maintaining the invariant

**Verification**:
```bash
grep -rn "unsafe {" src/kernel/ | wc -l
# Target: <100 (down from 193)

# All unsafe has SAFETY comment
grep -B1 "unsafe {" src/kernel/**/*.rs | grep -c "SAFETY"
# Target: matches unsafe count
```

---

## Phase 7: Separation of Concerns

**Goal**: Each module has single responsibility. No god objects.

### 7.1 Module Responsibility Audit

| Module | Current Responsibility | Issues |
|--------|----------------------|--------|
| task/mod.rs | Scheduler + Task table + Context switch | Too much |
| object/syscall.rs | All object syscalls | OK (thin wrappers) |
| object/mod.rs | All object types | Could split by type |
| shmem.rs | Shmem + waiting logic | Waiting should be in sched |
| event.rs | Event dispatch + task waking | Task waking crosses boundary |

### 7.2 Refactoring Plan

**task/ module**:
```
task/
├── mod.rs          # Re-exports only
├── table.rs        # Task table (allocation, lookup)
├── scheduler.rs    # Scheduling logic (pick next, timers)
├── tcb.rs          # Task struct (already extracted)
├── state.rs        # TaskState enum (already extracted)
├── lifecycle.rs    # exit/kill/wait (already extracted)
└── context.rs      # Context switch (extract from tcb.rs)
```

**object/ module**:
```
object/
├── mod.rs          # Re-exports only
├── types.rs        # Object enum, ObjectType
├── handle.rs       # HandleTable (already exists)
├── syscall.rs      # Syscall dispatch (already exists)
├── channel.rs      # ChannelObject impl
├── port.rs         # PortObject impl
├── timer.rs        # TimerObject impl
├── mux.rs          # MuxObject impl
└── process.rs      # ProcessObject impl
```

### 7.3 Extract Cross-Cutting Concerns

- **Waking logic**: Currently in event.rs, shmem.rs, object/mod.rs
  → Extract to `waker.rs` or use existing `ipc/waker.rs`

- **Timeout handling**: Currently in sched.rs, event.rs, shmem.rs
  → Centralize in scheduler

**Verification**:
```bash
# No file > 800 LOC (except generated)
wc -l src/kernel/**/*.rs src/kernel/*.rs | awk '$1 > 800 {print}'
# Target: 0 files
```

---

## Phase 8: Trait-Based Abstractions

**Goal**: Behavior defined by traits, not concrete types.

### 8.1 Existing Traits (Audit)

| Trait | File | Usage |
|-------|------|-------|
| `Pollable` | ipc/traits.rs | Objects that can be polled |
| `Waitable` | ipc/traits.rs | Objects that support wait |
| `Closable` | ipc/traits.rs | Objects that can be closed |
| `SchedulingPolicy` | task/policy.rs | Scheduler algorithm |
| `PciHostOps` | pci/host.rs | PCI host bridge ops |

### 8.2 Missing Traits

| Need | Purpose |
|------|---------|
| `StateMachine<S>` | Generic state transition |
| `Resource` | Common resource lifecycle (open/close/cleanup) |
| `Mappable` | Objects that can be memory-mapped |
| `BusOps` | Abstract bus operations |

### 8.3 Implement StateMachine Trait

```rust
pub trait StateMachine {
    type State: Copy + Eq;
    type Transition;
    type Error;

    fn state(&self) -> Self::State;
    fn transition(&mut self, t: Self::Transition) -> Result<(), Self::Error>;
}

// Usage
impl StateMachine for ChannelObject {
    type State = ChannelState;
    type Transition = ChannelTransition;
    type Error = InvalidTransition;

    fn state(&self) -> ChannelState { self.state }
    fn transition(&mut self, t: ChannelTransition) -> Result<(), InvalidTransition> {
        // Validated transition
    }
}
```

**Verification**:
- All state-bearing types implement `StateMachine` or equivalent
- No direct field access for state

---

## Iteration Checklist

After each phase, verify:

```bash
# Build passes
./build.sh

# No warnings
cargo build --release 2>&1 | grep -c "warning"

# Tests pass
cargo test

# Boot test (on hardware or QEMU)
# - Kernel boots
# - devd/consoled/shell work
# - Basic IPC works
```

---

## Progress Tracking

| Phase | Status | Blocking Issues |
|-------|--------|-----------------|
| Phase 0: Dead Code | NOT STARTED | |
| Phase 1: State Machines | NOT STARTED | |
| Phase 2: Encapsulation | NOT STARTED | |
| Phase 3: Unified Syscalls | NOT STARTED | |
| Phase 4: Eliminate Panics | NOT STARTED | |
| Phase 5: Test Coverage | NOT STARTED | |
| Phase 6: Unsafe Audit | NOT STARTED | |
| Phase 7: Separation of Concerns | NOT STARTED | |
| Phase 8: Trait Abstractions | NOT STARTED | |

---

## Success Criteria (GREEN)

| Metric | Target | Verification |
|--------|--------|--------------|
| Dead code warnings | 0 | `cargo build 2>&1 \| grep dead_code` |
| Direct state assignments | 0 | `grep "\.state = " \| wc -l` |
| Unsafe global accessors | 0 | `grep "scheduler()\|bus_registry()"` |
| TODO in kernel | 0 | `grep TODO src/kernel/` |
| Test count | >150 | `cargo test \| grep "^test"` |
| Unwrap/panic | <20 | `grep "unwrap\|panic" \| wc -l` |
| File size | <800 LOC | `wc -l` |
| Build | Clean | `./build.sh` |
| Boot | Works | Manual test |

---

## Appendix: Command Reference

```bash
# Find all TODOs
grep -rn "TODO\|FIXME\|HACK\|XXX" src/kernel/

# Find direct state assignments
grep -rn "\.state = " src/kernel/ | grep -v "fn \|// "

# Find unsafe blocks
grep -rn "unsafe {" src/kernel/

# Find panicking code
grep -rn "unwrap()\|expect(\|panic!\|unreachable!" src/kernel/

# Count public API
grep -rn "^pub fn\|^pub struct\|^pub enum" src/kernel/ | wc -l

# Count tests
grep -rn "#\[test\]" src/ | wc -l

# Module sizes
wc -l src/kernel/**/*.rs src/kernel/*.rs | sort -n

# Build with all warnings
RUSTFLAGS="-D warnings" cargo build --release
```
