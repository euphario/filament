
# Filament Microtasks Architecture

This document describes the architectural changes needed to evolve Filament’s microtask mechanism into a first-class kernel subsystem. The goal is to keep the scheduler compact, improve correctness, and support SMP and multi-architecture targets (AArch64 and x86_64), while remaining bounded and auditable.

---

## Goals

### Primary goals

- **Correctness-first:** microtasks must not introduce new races, deadlocks, or unbounded work.
- **Keep the scheduler lean:** the scheduler decides *who runs next*; microtasks handle deferred side effects and cleanup.
- **Bounded behavior:** no unbounded allocation; strict execution budgets.
- **SMP-friendly:** per-CPU queues with safe remote enqueue.
- **Portable:** arch-neutral design; arch layer only provides IPI/poke and safe-point hooks.

### Non-goals

- Not a replacement for kernel threads.
- Not a policy engine.
- Not for heavy or blocking work.

---

## Concept

Microtasks are **deferred, bounded kernel jobs** that complete state-machine transitions or cleanup outside of hot paths and lock-heavy regions.

They run only at **safe points**:
- syscall exit
- interrupt return
- idle loop

They never block and run under a strict budget.

---

## Microtask Model

Microtasks represent deferred completion steps.

Good examples:

- Wake a task
- Reap a dead process
- Finalize handle/object close
- Kick IPC endpoint waiters
- Advance bus/controller initialization state machine

Bad examples:

- Scheduler policy decisions
- Full system scans
- Heavy parsing or enumeration
- Blocking or allocation-heavy operations

---

## Representation

Use a fixed enum — no closures, no trait objects.

```rust
enum MicroTask {
    WakePid(Pid),
    ReapPid(Pid),
    IpcKick(ObjectId),
    FinalizeClose { pid: Pid, handle: Handle },
    BusTick(BusId),
}
```

Why enum:

- Fixed size
- Auditable
- Traceable
- Allocation-free
- Easy to fuzz and test

---

## Queue Design

### Per-CPU bounded ring buffer

- One queue per CPU
- Fixed capacity (16–64 entries typical)
- No dynamic allocation
- Fast local enqueue
- Remote enqueue allowed with atomic/lock discipline

```rust
PerCpu<MicroTaskQueue<N>>
```

### Remote enqueue

Remote CPU enqueue:
- push into target CPU queue
- send arch poke/IPI if needed (idle or latency-sensitive case)

---

## Safe Points

Microtasks run only at safe points where it is valid to:

- take scheduler locks
- mutate task state
- wake tasks

Call sites:

- syscall exit path
- interrupt return path
- idle loop

---

## Budgeted Execution

Runner executes with a strict budget:

```rust
microtask::run(budget: usize)
```

Typical budgets:

- syscall exit: 8–16
- interrupt exit: 4–8
- idle loop: 32–128

This prevents starvation and keeps latency bounded.

---

## Overflow Strategy

Queues are bounded — overflow must not break correctness.

### Sticky flag + bounded sweep model

If enqueue fails:

- set sticky flag (e.g. NEEDS_IPC_SWEEP)
- runner performs bounded corrective sweep later

Never silently drop required work.

---

## Correctness Rules

### Rule 1 — Never block

Microtasks must not sleep or wait.

If progress is not possible:
- re-enqueue
- rely on later events
- escalate to userspace

### Rule 2 — No lock held across side effects

Pattern:

1. collect under lock
2. drop lock
3. perform actions (wake, notify, etc.)

### Rule 3 — Idempotent handlers

Microtasks may run twice — handlers must re-check state and safely no-op.

### Rule 4 — Only run at safe points

Enforced by design and optional debug assertions.

---

## Integration: What Moves Out of the Scheduler

Scheduler keeps:

- pick next task
- context switch
- run queue invariants

Move to microtasks:

- deferred wakes
- process reaping
- handle/object finalization
- IPC waiter kicks
- bus/controller init ticks
- liveness sweeps

---

## Object & IPC Lifetime Integration

Use microtasks to finalize destruction:

- mark dead under lock
- enqueue finalize microtask
- perform heavy cleanup later

This avoids:

- lock inversion
- teardown while referenced
- long critical sections

---

## Bus / Discovery Integration

Replace busy loops with microtask ticks:

```text
BusTick(bus_id)
```

Each run advances the controller state machine one step under budget.

---

## Architecture Portability

Arch layer provides only:

- CPU poke / IPI
- safe-point hooks
- interrupt/syscall exit hooks

Microtask subsystem stays arch-neutral.

---

## Migration Plan

### Step 1 — Wrap existing deferred wake as MicroTask::WakePid

### Step 2 — Add ReapPid microtask

### Step 3 — Add IpcKick microtask

### Step 4 — Convert bus init loops to BusTick

### Step 5 — Add overflow flags + sweeps

Each step reduces scheduler complexity.

---

## Debug Invariants

Assert in debug builds:

- dead tasks are never woken
- reaped tasks have no handles/waiters/mappings
- closed endpoints leave no waiters
- runner never exceeds budget
- runner executes only at safe points

---

## Testing Strategy

- Double enqueue races
- Wake vs exit races
- Endpoint close with waiters
- Queue overflow recovery
- SMP remote enqueue correctness

---

## Summary

Scheduler = **who runs next**  
Microtasks = **bounded deferred completion and cleanup**  
Subsystems = **state machines that enqueue microtasks**  
Arch = **safe points + IPIs only**

This keeps Filament compact, portable, and correctness-driven while scaling beyond single-board targets.
