
# Filament Microtasks Implementation Plan

This companion document is implementation-oriented. It maps the Microtask architecture onto concrete modules, hook points, and a migration plan from your current “first-gen pending wake queue” to a general, bounded, per-CPU microtask system.

This is written to preserve Filament’s priorities:
- correctness-first
- state-machine driven
- bounded, auditable behavior
- SMP- and arch-portable (AArch64 + x86_64)

---

## 1) Where the code should live

### Proposed module layout

```
src/kernel/
  microtask.rs           # core types + queue + runner
  microtask_queue.rs     # optional: ring buffer implementation (if separated)
  safe_points.rs         # optional: centralize safe-point hooks (debug asserts)
  ...
src/arch/
  aarch64/
    ipi.rs               # poke CPU / resched IPI
    traps.rs             # syscall/irq return hook calls
  x86_64/
    apic.rs              # poke CPU
    idt.rs / syscall.rs  # syscall/irq return hook calls
```

**Rule:** `kernel/microtask.rs` is arch-neutral. `arch/*` only provides CPU poke and safe-point call sites.

---

## 2) Core API surface (small and strict)

### Types

```rust
#[derive(Copy, Clone, Debug)]
pub enum MicroTask {
    WakePid(Pid),
    ReapPid(Pid),
    IpcKick(ObjectId),
    FinalizeClose { pid: Pid, handle: Handle },
    BusTick(BusId),
}

bitflags::bitflags! {
    pub struct MicroTaskSweepFlags: u32 {
        const NEEDS_WAKE_SWEEP = 1 << 0;
        const NEEDS_IPC_SWEEP  = 1 << 1;
        const NEEDS_REAP_SWEEP = 1 << 2;
        const NEEDS_BUS_TICK   = 1 << 3;
    }
}
```

### Enqueue

```rust
pub fn enqueue_local(task: MicroTask) -> bool;
pub fn enqueue_remote(cpu: CpuId, task: MicroTask) -> bool;
```

### Runner

```rust
pub fn run(budget: usize);
```

### Optional helper

```rust
pub fn request_wake(pid: Pid);   // wraps enqueue_local/remote logic
```

**Design constraints:**
- `enqueue_*` is allocation-free
- `run()` is bounded by `budget`
- handlers are idempotent and state-checked

---

## 3) Queue implementation options

Start with **simple and correct**, then optimize.

### Option A: Per-CPU ring buffer + spinlock (recommended first)
- `MicroTaskQueue<N>` has a small ring buffer and a lock.
- Local enqueue: lock + push (fast, bounded).
- Remote enqueue: lock + push; optionally poke CPU.

Pros: simplest, hardest to get wrong, good enough for early SMP.
Cons: lock contention possible if remote enqueues are frequent (usually they aren’t).

### Option B: Lock-free MPSC per-CPU (later)
- producers on many CPUs, single consumer on target CPU.
- more complex; do only when measured need exists.

**Recommendation:** ship Option A first.

---

## 4) Safe points: exact hook locations

Microtasks should run only at safe points. You already have “safe points” conceptually (deferred wake processing). Consolidate those calls.

### Hook point 1: Syscall exit
Right before returning to user mode:

- `microtask::run(SYSCALL_BUDGET)`
- then return to user

### Hook point 2: Interrupt return
At the end of interrupt handling, before returning to user:

- `microtask::run(IRQ_BUDGET)`

### Hook point 3: Idle loop
Before executing `WFI/HLT`, drain more:

- `microtask::run(IDLE_BUDGET)`
- if still work remains, skip `WFI` (optional)
- else `WFI`

### Debug-only guard (optional but recommended)
Add a `debug_assert!(in_safe_point_context())` within `microtask::run()` to prevent accidental use elsewhere.

---

## 5) Handler contracts (correctness rules as code)

For each `MicroTask` variant implement:

```rust
fn handle(task: MicroTask) {
    match task {
      MicroTask::WakePid(pid) => handle_wake_pid(pid),
      MicroTask::ReapPid(pid) => handle_reap_pid(pid),
      ...
    }
}
```

**Hard rules:**
- handlers must not block
- handlers must not allocate unbounded memory
- handlers must not hold locks across wakeups or external calls
- handlers must be idempotent (safe to run twice)

### Example: WakePid handler
Pseudo:

1. lookup pid in task table
2. if state is already runnable or dead → return
3. call scheduler wake path (safe point means lock is OK)

### Example: IpcKick handler
1. lock endpoint
2. compute list of waiters to wake + clean dead ones
3. drop endpoint lock
4. wake tasks (or enqueue WakePid microtasks)

This matches your existing IPC waker style.

---

## 6) Integrations: what to migrate first

You’ll get big wins by migrating these in order:

### Stage 1: Replace current pending wake queue
Current: `request_wake(pid)` stores into an arch-local pending array.

Migration:
- implement `MicroTask::WakePid`
- change `request_wake(pid)` to enqueue `WakePid(pid)`
- call `microtask::run()` at the same safe points you already use

This should be nearly mechanical.

### Stage 2: Process teardown → `ReapPid`
Today, exit paths often become “do everything now”. That grows the scheduler/lifecycle code.

New approach:
- on exit: mark state dead under lock, detach from run queues, enqueue `ReapPid(pid)`
- `ReapPid` handler performs bounded cleanup:
  - release handle table
  - detach from IPC wait lists (if not already)
  - free address space mappings
  - release kernel stack to pool

**Key invariant:** after `ReapPid`, the PID slot can be reused safely.

### Stage 3: Handle/object close finalization
Close syscall should be cheap:
- mark handle dead, remove mapping under lock
- enqueue `FinalizeClose { pid, handle }` (or `FinalizeObject(obj)`)
- handler drops refs, runs destructor, notifies IPC endpoints if needed

### Stage 4: Bus/controller init ticks
Replace any `while continue_init() {}` patterns with:
- enqueue `BusTick(bus_id)`
- handler advances one state-machine step per call
- re-enqueue if more work remains
- if it needs to wait for timeouts, just re-enqueue later via timer events

This keeps boot/bring-up bounded and debuggable.

---

## 7) Budget and priority tuning

### Suggested budgets
- `SYSCALL_BUDGET = 8..16`
- `IRQ_BUDGET     = 4..8`
- `IDLE_BUDGET    = 32..128`

### Optional priority split (later)
If microtasks grow, consider two queues:
- high priority (wakes)
- low priority (cleanup)

But keep it single-queue until you need it.

---

## 8) Overflow: make it correct under pressure

Because the queue is bounded, failure to enqueue must not silently break correctness.

### Strategy: sticky sweep flags
On enqueue failure:
- set a sticky flag (per-CPU)
- runner checks flags and runs a bounded “sweep” microtask

Example:
- queue full while enqueuing `WakePid(pid)`
- set `NEEDS_WAKE_SWEEP`
- later: runner executes a bounded scan over a small structure (like a pending PID set) to re-issue wakes

**Keep sweeps bounded** (max K items per run) and re-set flag if more remains.

---

## 9) SMP details: remote enqueue + CPU poke

### When to poke
- If enqueued to a remote CPU and it is idle, poke immediately.
- Otherwise you can rely on periodic timer tick / next safe point, or poke if latency-sensitive.

### Minimal arch interface
```rust
pub trait ArchMicrotask {
    fn poke_cpu(cpu: CpuId);
}
```

AArch64: SGI
x86_64: APIC IPI

Keep the kernel logic arch-neutral.

---

## 10) Lifetimes: microtasks pair naturally with “deferred free”

Microtasks are ideal for correctness of object lifetimes:

- avoid freeing while locks are held
- avoid freeing while references might still exist
- centralize destruction paths

Two common patterns:

### Pattern A: refcount + deferred drop
- close decrements refcount to zero under lock
- enqueue a finalize microtask to run destructor outside locks

### Pattern B: generation-based reclamation
- mark slot dead (generation++) under lock
- finalize in microtask later
- stale handles fail generation check

Either works; choose one consistent approach across objects and IPC endpoints.

---

## 11) Debugging and correctness instrumentation

### Debug invariants (recommended)
Add `debug_assert!` checks and counters:

- microtask runner budget never exceeded
- dead tasks never re-enter runnable state
- reaped tasks have no handle entries / waitlist membership
- endpoint close does not leave waiters stranded
- queue overflow increments a counter and sets a sweep flag

### Trace hooks
Microtasks are perfect trace points:
- `trace_microtask_enqueue(kind, cpu, success)`
- `trace_microtask_run(kind, cpu)`
- `trace_microtask_overflow(kind, cpu)`

This will make “why did I not wake?” bugs tractable.

---

## 12) Minimal “diff plan” for your current codebase

1. Add `src/kernel/microtask.rs` with:
   - enum `MicroTask`
   - per-CPU queue
   - `enqueue_local`, `enqueue_remote`, `run`
2. Replace existing deferred wake queue enqueues with `microtask::enqueue_* (WakePid)`
3. Add `microtask::run(budget)` calls at safe points (syscall exit, irq return, idle loop)
4. Move process teardown into `ReapPid`
5. Move IPC “kicks” and close finalization into microtasks
6. Replace bus init loops with `BusTick` microtasks

Each step is small, testable, and reduces scheduler bloat.

---

## Summary

- Make microtasks a kernel subsystem, not an arch quirk.
- Keep it bounded, budgeted, and safe-point-only.
- Move cleanup and “completion work” out of scheduler/IPC hot paths.
- Use microtasks to enforce lifetimes and state-machine completion.
- This makes Filament smaller, more correct, and portable across AArch64 and x86_64.

