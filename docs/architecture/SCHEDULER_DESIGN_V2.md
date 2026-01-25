# Scheduler Design v2: Production-Ready

## Executive Summary

This document proposes targeted improvements to the scheduler that:
1. **Fix identified bugs** without complete rewrite
2. **Enable SMP** with proper locking strategy
3. **Improve error handling** with Result types
4. **Maintain testability** through traits
5. **Preserve what works** (state machine, two-phase cleanup)

**Philosophy**: Evolutionary improvement, not revolutionary rewrite.

---

## Assessment Summary

### What's Working Well

| Component | Status | Notes |
|-----------|--------|-------|
| **State Machine** | Excellent | Comprehensive, well-tested, enforces invariants |
| **Two-Phase Cleanup** | Good | Exiting→Dying→Dead allows graceful shutdown |
| **PID Generation** | Good | Generation counter prevents stale PID issues |
| **Idle Task** | Good | Real task in slot 0, proper context switching |
| **HAL Traits** | Good | Cpu, Context, Timer traits exist and work |

### Issues to Fix

| Issue | Severity | Section |
|-------|----------|---------|
| SMP not supported (lock held across context switch) | High | §2 |
| Race in `do_resched_if_needed()` | Medium | §3.1 |
| CURRENT_TRAP_FRAME use-after-free risk | Medium | §3.2 |
| Boolean returns hide errors | Medium | §4 |
| `context_saved` is implicit state | Low | §5 |
| Large functions (reap_terminated) | Low | §6 |
| ObjectService coupling | Low | §7 |

---

## 1. Core Principle: Lock Discipline

### The Problem

Current code holds scheduler "lock" (IrqGuard) across context_switch:
```
Task A: IrqGuard → context_switch(A, B) → [A frozen with guard on stack]
Task B: resumes → needs scheduler → deadlock (if SpinLock) or race (if just IrqGuard)
```

### The Solution: Three-Phase Scheduling

```rust
pub fn reschedule() -> bool {
    // PHASE 1: Under lock - make decision, prepare switch
    let switch_decision = {
        let mut guard = SCHEDULER.lock();
        guard.prepare_switch()
    }; // Lock released

    // PHASE 2: No lock - context switch (pure register manipulation)
    if let Some(decision) = switch_decision {
        unsafe {
            set_current_slot(decision.to_slot);
            update_globals_for_task(decision.to_slot);
            context_switch(decision.from_ctx, decision.to_ctx);
        }
        // === Resumed here ===

        // PHASE 3: Reacquire lock - finalize
        let mut guard = SCHEDULER.lock();
        guard.finalize_resume(decision.from_slot);
    }

    true
}
```

### Key Insight

The context_switch itself is **just register save/restore** - it doesn't need scheduler data. We:
1. Prepare everything under lock (get pointers, mark target Running)
2. Release lock
3. Do context switch (safe: we're the only one touching these specific registers)
4. Reacquire lock
5. Handle any races (task killed while switched out)

### SwitchDecision Structure

```rust
/// Information needed to perform a context switch
/// Extracted while holding lock, used after releasing it
struct SwitchDecision {
    from_slot: usize,
    to_slot: usize,
    from_ctx: *mut CpuContext,
    to_ctx: *const CpuContext,
}
```

**Safety**: The pointers are valid because:
- `from_slot` is us - we're not going anywhere
- `to_slot` is marked Running before lock release - won't be reaped
- Task structs don't move (fixed array)

---

## 2. Scheduler Module Structure

### Proposed Layout

```
src/kernel/sched/
├── mod.rs              # Public API, re-exports
├── core.rs             # SchedulerCore - task table, state
├── switch.rs           # Context switching logic
├── policy.rs           # Scheduling policy trait + implementations
├── reaper.rs           # Task cleanup (split from current monolith)
└── error.rs            # Error types

src/kernel/task/
├── mod.rs              # Task struct, Schedulable trait
├── state.rs            # TaskState enum (keep as-is, it's good)
├── tcb.rs              # TrapFrame, CpuContext (keep as-is)
└── lifecycle.rs        # Spawn, exit, eviction
```

### SchedulerCore

```rust
/// Core scheduler data protected by SpinLock
pub struct SchedulerCore {
    /// Task table - fixed array, slots are stable
    tasks: [Option<Task>; MAX_TASKS],

    /// Generation counter per slot for PID stability
    generations: [u32; MAX_TASKS],

    /// Next deadline for tickless optimization
    next_deadline: u64,

    /// Scheduling policy
    policy: PriorityRoundRobin,
}

/// Global scheduler with SMP-safe locking
static SCHEDULER: SpinLock<SchedulerCore> = SpinLock::new(SchedulerCore::new());
```

### Public API

```rust
/// Execute closure with scheduler lock held
/// This is the PRIMARY way to access the scheduler
pub fn with_scheduler<R>(f: impl FnOnce(&mut SchedulerCore) -> R) -> R {
    let mut guard = SCHEDULER.lock();
    f(&mut *guard)
}

/// Reschedule - may context switch
/// Returns true if we switched to a different task
pub fn reschedule() -> bool {
    // Three-phase implementation as above
}

/// Wake a task by PID
pub fn wake(pid: u32) -> Result<(), WakeError> {
    with_scheduler(|sched| sched.wake_by_pid(pid))
}

/// Block current task (no deadline)
pub fn sleep_current(reason: SleepReason) -> Result<(), BlockError> {
    with_scheduler(|sched| sched.sleep_current_impl(reason))?;
    reschedule();
    Ok(())
}

/// Block current task with deadline
pub fn wait_current(reason: WaitReason, deadline: u64) -> Result<(), BlockError> {
    with_scheduler(|sched| sched.wait_current_impl(reason, deadline))?;
    reschedule();
    Ok(())
}
```

---

## 3. Bug Fixes

### 3.1 Race in do_resched_if_needed()

**Current code** (buggy):
```rust
pub unsafe extern "C" fn do_resched_if_needed() {
    let need_resched = cpu_flags().check_and_clear_resched();

    if !need_resched {
        let sched = scheduler();  // NO LOCK!
        let current_blocked = sched.tasks[my_slot]...
        // RACE: another CPU could modify task state
    }
}
```

**Fixed**:
```rust
pub unsafe extern "C" fn do_resched_if_needed() {
    let need_resched = cpu_flags().check_and_clear_resched();

    let should_resched = with_scheduler(|sched| {
        if need_resched {
            return true;
        }
        // Check if current task is blocked (needs lock)
        let my_slot = current_slot();
        sched.task(my_slot)
            .map(|t| t.is_blocked())
            .unwrap_or(false)
    });

    if should_resched {
        reschedule();
    }
}
```

### 3.2 CURRENT_TRAP_FRAME Safety

**Problem**: When task dies, CURRENT_TRAP_FRAME may still point to freed memory.

**Solution**: Validate on every access + clear on reap.

```rust
/// Safely get current trap frame
/// Returns None if current slot is invalid or has no task
pub fn current_trap_frame() -> Option<&'static mut TrapFrame> {
    let ptr = CURRENT_TRAP_FRAME.load(Ordering::Acquire);
    if ptr.is_null() {
        return None;
    }

    // Validate pointer is in kernel space
    let addr = ptr as u64;
    if addr < KERNEL_VIRT_BASE {
        return None;
    }

    // Validate current task exists
    with_scheduler(|sched| {
        let slot = current_slot();
        if sched.task(slot).is_some() {
            Some(unsafe { &mut *ptr })
        } else {
            None
        }
    })
}

/// Clear trap frame pointer when task is reaped
fn on_task_reaped(slot: usize) {
    if current_slot() == slot {
        CURRENT_TRAP_FRAME.store(core::ptr::null_mut(), Ordering::Release);
    }
}
```

---

## 4. Error Handling

### Error Types

```rust
// src/kernel/sched/error.rs

/// Errors from wake operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeError {
    /// Task not found (invalid PID or slot)
    NotFound,
    /// Task exists but not blocked (already Ready/Running)
    NotBlocked,
    /// Task is terminated (Exiting/Dying/Dead)
    Terminated,
}

/// Errors from blocking operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockError {
    /// Current task not found (should never happen)
    NoCurrentTask,
    /// Invalid state transition
    InvalidTransition { from: &'static str, to: &'static str },
    /// Cannot block idle task
    CannotBlockIdle,
}

/// Errors from spawn operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpawnError {
    /// No free task slots
    NoFreeSlots,
    /// Out of memory (for stack, page tables, etc.)
    OutOfMemory,
    /// Invalid ELF
    InvalidElf,
}

impl WakeError {
    pub fn to_errno(self) -> i64 {
        match self {
            WakeError::NotFound => -3,      // ESRCH
            WakeError::NotBlocked => -114,  // EALREADY
            WakeError::Terminated => -3,    // ESRCH
        }
    }
}
```

### Updated Method Signatures

```rust
impl SchedulerCore {
    /// Wake task by slot - returns detailed error
    pub fn wake_task(&mut self, slot: usize) -> Result<(), WakeError> {
        let task = self.tasks.get_mut(slot)
            .and_then(|t| t.as_mut())
            .ok_or(WakeError::NotFound)?;

        if task.is_terminated() {
            return Err(WakeError::Terminated);
        }

        if !task.is_blocked() {
            return Err(WakeError::NotBlocked);
        }

        task.wake().map_err(|e| {
            // Log transition error
            kerror!("sched", "wake_failed";
                slot = slot as u64,
                from = e.from,
                to = e.to
            );
            WakeError::NotBlocked
        })
    }

    /// Wake task by PID
    pub fn wake_by_pid(&mut self, pid: u32) -> Result<(), WakeError> {
        let slot = self.slot_by_pid(pid).ok_or(WakeError::NotFound)?;
        self.wake_task(slot)
    }
}
```

---

## 5. Context State Clarification

### The `context_saved` Problem

Currently `context_saved` is a boolean that means:
- `true`: Task was context-switched out, needs `context_switch()` to resume
- `false`: Task can resume via trap frame (eret)

This is **implicit state** separate from TaskState, which violates "state machine is law".

### Option A: Make it Part of Task State (Recommended)

```rust
pub enum TaskState {
    Ready,
    ReadyWithContext,  // NEW: Ready but needs context_switch to resume
    Running,
    Sleeping { reason: SleepReason },
    SleepingWithContext { reason: SleepReason },  // NEW
    Waiting { reason: WaitReason, deadline: u64 },
    WaitingWithContext { reason: WaitReason, deadline: u64 },  // NEW
    Exiting { code: i32 },
    Dying { code: i32, until: u64 },
    Evicting { reason: EvictionReason },
    Dead,
}

impl TaskState {
    pub fn needs_context_switch(&self) -> bool {
        matches!(self,
            TaskState::ReadyWithContext |
            TaskState::SleepingWithContext { .. } |
            TaskState::WaitingWithContext { .. }
        )
    }
}
```

### Option B: Document as Implementation Detail (Simpler)

Keep `context_saved` but:
1. Add strong documentation
2. Add debug assertions
3. Only access via accessor methods

```rust
impl Task {
    /// Whether this task needs context_switch() to resume
    ///
    /// # Invariants
    /// - `context_saved` is true IFF the task's kernel context was saved
    ///   by context_switch() and hasn't been restored yet
    /// - A task that enters via syscall (eret) has context_saved=false
    /// - A task that blocked has context_saved=true
    pub fn needs_context_restore(&self) -> bool {
        debug_assert!(
            !self.context_saved || self.is_blocked(),
            "context_saved should only be true for blocked tasks"
        );
        self.context_saved
    }

    /// Mark that context was saved (called before context_switch)
    pub(crate) fn mark_context_saved(&mut self) {
        debug_assert!(
            self.state() == TaskState::Running || self.is_blocked(),
            "can only save context of running/blocked task"
        );
        self.context_saved = true;
    }

    /// Clear context_saved (called after context_switch restores)
    pub(crate) fn clear_context_saved(&mut self) {
        self.context_saved = false;
    }
}
```

**Recommendation**: Option B for now (less churn), with clear path to Option A later.

---

## 6. Reaper Refactoring

### Current Problem

`reap_terminated()` is 173 lines handling three different flows:
1. Exiting → Dying (Phase 1: notifications)
2. Dying → Dead (Phase 2: forced cleanup)
3. Evicting → Dead (immediate cleanup)

### Proposed Split

```rust
// src/kernel/sched/reaper.rs

impl SchedulerCore {
    /// Main reaper entry point - called from timer tick
    pub fn reap_terminated(&mut self, current_tick: u64) {
        let current = current_slot();

        for slot in 1..MAX_TASKS {  // Skip idle
            if slot == current { continue; }

            let Some(task) = self.tasks[slot].as_mut() else { continue };

            match *task.state() {
                TaskState::Exiting { code } => {
                    self.handle_exiting(slot, code, current_tick);
                }
                TaskState::Dying { code, until } if current_tick >= until => {
                    self.handle_dying_expired(slot, code);
                }
                TaskState::Evicting { reason } => {
                    self.handle_evicting(slot, reason);
                }
                _ => {}
            }
        }
    }

    /// Phase 1: Task called exit(), send notifications
    fn handle_exiting(&mut self, slot: usize, code: i32, current_tick: u64) {
        let task = self.tasks[slot].as_mut().unwrap();
        let pid = task.id;
        let parent_id = task.parent_id;

        // 1. Notify IPC peers
        ipc::process_cleanup(pid);

        // 2. Mark shmem as dying
        shmem::begin_cleanup(pid);

        // 3. Reparent children to init
        self.reparent_children(slot);

        // 4. Transition to Dying with grace period
        let grace_period = 100; // ticks
        task.set_dying(code, current_tick + grace_period);

        // 5. Wake parent if sleeping on ChildWait
        if let Some(parent_slot) = self.slot_by_pid(parent_id) {
            if let Some(parent) = self.tasks[parent_slot].as_mut() {
                if parent.is_waiting_for_child() {
                    let _ = parent.wake();
                }
            }
        }
    }

    /// Phase 2: Grace period expired, force cleanup
    fn handle_dying_expired(&mut self, slot: usize, code: i32) {
        let task = self.tasks[slot].as_mut().unwrap();
        let pid = task.id;

        // Force cleanup
        shmem::finalize_cleanup(pid);
        irq::process_cleanup(pid);
        pci::release_all_devices(pid);
        bus::process_cleanup(pid);
        object_service().remove_task_table(pid);

        // Clear trap frame if this was current
        on_task_reaped(slot);

        // Transition to Dead
        task.set_dead();

        // Free task slot
        self.tasks[slot] = None;

        kinfo!("task", "reaped"; pid = pid as u64, code = code as i64);
    }

    /// Eviction: Immediate cleanup (no grace period)
    fn handle_evicting(&mut self, slot: usize, reason: EvictionReason) {
        let task = self.tasks[slot].as_ref().unwrap();
        let pid = task.id;

        kerror!("task", "evicted";
            pid = pid as u64,
            reason = reason.as_str()
        );

        // Same cleanup as dying, but immediate
        shmem::finalize_cleanup(pid);
        irq::process_cleanup(pid);
        pci::release_all_devices(pid);
        bus::process_cleanup(pid);
        object_service().remove_task_table(pid);

        on_task_reaped(slot);

        // Direct to Dead (skip Dying)
        self.tasks[slot] = None;
    }

    /// Reparent children of dying task to init (PID 2)
    fn reparent_children(&mut self, dying_slot: usize) {
        let dying_pid = self.tasks[dying_slot].as_ref().unwrap().id;

        for slot in 1..MAX_TASKS {
            if let Some(ref mut task) = self.tasks[slot] {
                if task.parent_id == dying_pid {
                    task.parent_id = 2; // init's PID
                }
            }
        }
    }
}
```

---

## 7. ObjectService Integration

### Current Problem

Scheduler calls ObjectService directly with no error handling:
```rust
object_service().create_task_table(id, false);  // No error check!
object_service().remove_task_table(pid);        // No error check!
```

### Solution: Explicit Interface

```rust
/// Trait for task lifecycle hooks
/// Implemented by ObjectService, testable with mocks
pub trait TaskLifecycleHooks: Send + Sync {
    /// Called when task is created
    fn on_task_created(&self, task_id: u32, is_user: bool) -> Result<(), &'static str>;

    /// Called when task is about to be reaped
    fn on_task_dying(&self, task_id: u32);

    /// Called when task slot is freed
    fn on_task_reaped(&self, task_id: u32) -> Result<(), &'static str>;
}

// In scheduler initialization
static LIFECYCLE_HOOKS: AtomicPtr<dyn TaskLifecycleHooks> = AtomicPtr::new(null_mut());

pub fn set_lifecycle_hooks(hooks: &'static dyn TaskLifecycleHooks) {
    LIFECYCLE_HOOKS.store(hooks as *const _ as *mut _, Ordering::Release);
}

fn lifecycle_hooks() -> Option<&'static dyn TaskLifecycleHooks> {
    let ptr = LIFECYCLE_HOOKS.load(Ordering::Acquire);
    if ptr.is_null() {
        None
    } else {
        Some(unsafe { &*ptr })
    }
}

// Usage in scheduler
impl SchedulerCore {
    pub fn add_user_task(&mut self, name: &str) -> Result<(u32, usize), SpawnError> {
        let slot = self.find_free_slot().ok_or(SpawnError::NoFreeSlots)?;
        let id = self.make_pid(slot);

        // Notify hooks
        if let Some(hooks) = lifecycle_hooks() {
            hooks.on_task_created(id, true)
                .map_err(|_| SpawnError::OutOfMemory)?;
        }

        // Create task...
        Ok((id, slot))
    }
}
```

---

## 8. Testing Strategy

### Unit Tests (Already Good)

State machine tests in `state.rs` are comprehensive. Keep them.

### New Integration Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    /// Mock lifecycle hooks for testing
    struct MockHooks {
        created: RefCell<Vec<u32>>,
        reaped: RefCell<Vec<u32>>,
    }

    impl TaskLifecycleHooks for MockHooks {
        fn on_task_created(&self, id: u32, _is_user: bool) -> Result<(), &'static str> {
            self.created.borrow_mut().push(id);
            Ok(())
        }
        fn on_task_dying(&self, _id: u32) {}
        fn on_task_reaped(&self, id: u32) -> Result<(), &'static str> {
            self.reaped.borrow_mut().push(id);
            Ok(())
        }
    }

    #[test]
    fn test_task_lifecycle_hooks_called() {
        let hooks = MockHooks::default();
        // ... test that hooks are called at right times
    }

    #[test]
    fn test_wake_returns_not_blocked_for_ready_task() {
        let mut sched = SchedulerCore::new();
        sched.add_task(/* ready task */);

        let result = sched.wake_task(1);
        assert_eq!(result, Err(WakeError::NotBlocked));
    }

    #[test]
    fn test_reaper_transitions_exiting_to_dying() {
        let mut sched = SchedulerCore::new();
        // Add task, set to Exiting
        sched.reap_terminated(0);
        // Assert task is now Dying
    }
}
```

---

## 9. Migration Plan

### Phase 1: Bug Fixes (No API Changes)
1. Fix `do_resched_if_needed()` race condition
2. Add CURRENT_TRAP_FRAME safety checks
3. Add Result types to internal methods (keep bool wrappers for now)

### Phase 2: Structure Cleanup
1. Split `reap_terminated()` into smaller functions
2. Add `context_saved` accessor methods with assertions
3. Add TaskLifecycleHooks trait

### Phase 3: SMP Preparation
1. Change `static mut SCHEDULER` to `static SCHEDULER: SpinLock<SchedulerCore>`
2. Implement three-phase reschedule
3. Add per-CPU current_slot (already exists via percpu)

### Phase 4: Error Handling
1. Change public API to return Result
2. Update all callers
3. Add comprehensive logging on errors

---

## 10. Non-Goals (Deferred)

These are out of scope for this iteration:

1. **Per-CPU run queues** - Needed for true SMP scalability, but complex
2. **Priority inheritance** - For mutex priority inversion, not needed yet
3. **Real-time scheduling** - Deadline-based guarantees, future work
4. **NUMA awareness** - Not relevant for BPI-R4 (single socket)
5. **Task migration** - Moving tasks between CPUs, needs per-CPU queues first

---

## Summary

This design:
- **Fixes real bugs** (race condition, use-after-free risk)
- **Enables SMP** via three-phase scheduling with lock release
- **Improves errors** with Result types instead of booleans
- **Maintains compatibility** with existing code
- **Is testable** through traits and hooks
- **Respects the laws** (state machine, SMP safe, no busy-wait)

The changes are incremental and can be done in phases without a complete rewrite.
