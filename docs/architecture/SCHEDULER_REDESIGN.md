# Scheduler Redesign

## Overview

This document describes a clean redesign of the scheduler, task, and HAL layers with proper separation of concerns.

## Current Problems

1. **No idle task** - "idle" is a loop inside scheduler code, not a real task
2. **Tight coupling** - Scheduler directly manipulates Task internals
3. **HAL leakage** - Scheduler contains `asm!("wfi")` directly
4. **Context switch confusion** - User tasks don't have valid kernel contexts for blocking
5. **Testability** - Can't unit test scheduler without real tasks

## Design Principles

1. **Traits define contracts** - Scheduler works with traits, not concrete types
2. **HAL abstracts hardware** - CPU operations behind traits, mockable for tests
3. **Idle is a real task** - Slot 0 always has a runnable idle task
4. **State machine is law** - All state changes go through validated transitions
5. **Context is always valid** - Every task has a valid context for switching

---

## Layer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Syscall Layer                          │
│   (read_mux, spawn, exit - thin dispatchers)                │
├─────────────────────────────────────────────────────────────┤
│                     Scheduler Layer                         │
│   (policy, task selection, blocking/waking)                 │
├─────────────────────────────────────────────────────────────┤
│                       Task Layer                            │
│   (state machine, context, per-task resources)              │
├─────────────────────────────────────────────────────────────┤
│                       HAL Layer                             │
│   (context switch, idle, IRQ control, timer)                │
└─────────────────────────────────────────────────────────────┘
```

---

## HAL Layer (`src/hal/`)

### CPU Trait

```rust
// src/hal/cpu.rs

/// CPU operations - architecture provides implementation
pub trait Cpu: Send + Sync {
    /// Enter low-power idle state until interrupt
    /// ARM: WFI, x86: HLT, RISC-V: WFI
    fn idle(&self);

    /// Enable interrupts
    fn enable_irq(&self);

    /// Disable interrupts, return previous state
    fn disable_irq(&self) -> bool;

    /// Restore interrupt state
    fn restore_irq(&self, was_enabled: bool);

    /// Memory barrier (full)
    fn memory_barrier(&self);

    /// Get current CPU ID (for SMP)
    fn cpu_id(&self) -> usize;
}

/// RAII guard for disabling IRQs
pub struct IrqGuard<'a, C: Cpu> {
    cpu: &'a C,
    was_enabled: bool,
}

impl<'a, C: Cpu> IrqGuard<'a, C> {
    pub fn new(cpu: &'a C) -> Self {
        let was_enabled = cpu.disable_irq();
        Self { cpu, was_enabled }
    }
}

impl<C: Cpu> Drop for IrqGuard<'_, C> {
    fn drop(&mut self) {
        self.cpu.restore_irq(self.was_enabled);
    }
}
```

### Context Trait

```rust
// src/hal/context.rs

/// CPU context for kernel-to-kernel switching
/// Architecture-specific layout
pub trait Context: Default + Clone {
    /// Create context that will start executing at `entry` with given `stack`
    fn new_at(entry: fn() -> !, stack: *mut u8) -> Self;

    /// Get stack pointer
    fn stack_pointer(&self) -> *mut u8;

    /// Set stack pointer
    fn set_stack_pointer(&mut self, sp: *mut u8);
}

/// Context switching operations
pub trait ContextSwitch {
    type Ctx: Context;

    /// Switch from current context to next
    ///
    /// # Safety
    /// - `current` must point to valid, aligned context storage
    /// - `next` must point to valid context that was previously saved or initialized
    /// - Must be called with IRQs disabled
    unsafe fn switch(current: &mut Self::Ctx, next: &Self::Ctx);
}
```

### Timer Trait

```rust
// src/hal/timer.rs

/// Hardware timer operations
pub trait Timer: Send + Sync {
    /// Current counter value (monotonic, never wraps in practice)
    fn counter(&self) -> u64;

    /// Timer frequency in Hz
    fn frequency(&self) -> u64;

    /// Convert nanoseconds to ticks
    fn ns_to_ticks(&self, ns: u64) -> u64 {
        (ns * self.frequency()) / 1_000_000_000
    }

    /// Convert ticks to nanoseconds
    fn ticks_to_ns(&self, ticks: u64) -> u64 {
        (ticks * 1_000_000_000) / self.frequency()
    }

    /// Check if deadline has passed
    fn is_expired(&self, deadline: u64) -> bool {
        self.counter() >= deadline
    }

    /// Set one-shot timer to fire at deadline
    fn set_deadline(&self, deadline: u64);

    /// Disable timer interrupt
    fn disable(&self);
}
```

### AArch64 Implementation

```rust
// src/arch/aarch64/hal.rs

pub struct Aarch64Cpu;

impl Cpu for Aarch64Cpu {
    fn idle(&self) {
        unsafe { core::arch::asm!("wfi") }
    }

    fn enable_irq(&self) {
        unsafe { core::arch::asm!("msr daifclr, #2") }
    }

    fn disable_irq(&self) -> bool {
        let daif: u64;
        unsafe {
            core::arch::asm!("mrs {}, daif", out(reg) daif);
            core::arch::asm!("msr daifset, #2");
        }
        (daif & 0x80) == 0  // Was IRQ enabled?
    }

    fn restore_irq(&self, was_enabled: bool) {
        if was_enabled {
            self.enable_irq();
        }
    }

    fn memory_barrier(&self) {
        unsafe { core::arch::asm!("dsb sy; isb") }
    }

    fn cpu_id(&self) -> usize {
        let mpidr: u64;
        unsafe { core::arch::asm!("mrs {}, mpidr_el1", out(reg) mpidr) }
        (mpidr & 0xFF) as usize
    }
}

/// AArch64 kernel context (callee-saved registers)
#[repr(C)]
#[derive(Clone, Default)]
pub struct Aarch64Context {
    pub x19: u64,
    pub x20: u64,
    pub x21: u64,
    pub x22: u64,
    pub x23: u64,
    pub x24: u64,
    pub x25: u64,
    pub x26: u64,
    pub x27: u64,
    pub x28: u64,
    pub x29: u64,  // Frame pointer
    pub x30: u64,  // Link register (return address)
    pub sp: u64,   // Stack pointer
}

impl Context for Aarch64Context {
    fn new_at(entry: fn() -> !, stack: *mut u8) -> Self {
        Self {
            x30: entry as u64,  // Return address = entry point
            sp: stack as u64,
            ..Default::default()
        }
    }

    fn stack_pointer(&self) -> *mut u8 {
        self.sp as *mut u8
    }

    fn set_stack_pointer(&mut self, sp: *mut u8) {
        self.sp = sp as u64;
    }
}

pub struct Aarch64ContextSwitch;

impl ContextSwitch for Aarch64ContextSwitch {
    type Ctx = Aarch64Context;

    unsafe fn switch(current: &mut Self::Ctx, next: &Self::Ctx) {
        context_switch_asm(current as *mut _, next as *const _);
    }
}

extern "C" {
    fn context_switch_asm(current: *mut Aarch64Context, next: *const Aarch64Context);
}
```

---

## Task Layer (`src/kernel/task/`)

### Task State

```rust
// src/kernel/task/state.rs

/// Task lifecycle states
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TaskState {
    /// Can be scheduled
    Ready,

    /// Currently executing
    Running,

    /// Blocked waiting for event (no deadline)
    Sleeping { reason: SleepReason },

    /// Blocked with timeout
    Waiting { reason: WaitReason, deadline: u64 },

    /// Exiting, cleaning up
    Exiting { code: i32 },

    /// Terminated, slot can be reused
    Dead,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SleepReason {
    EventLoop,      // Waiting in mux.wait()
    ChannelRecv,    // Waiting for IPC message
    Mutex,          // Waiting for lock
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaitReason {
    Timer,          // Explicit sleep/timeout
    TimedRecv,      // IPC with timeout
    ChildExit,      // Waiting for child process
}

impl TaskState {
    /// Valid state transitions (enforced)
    pub fn can_transition_to(&self, to: &TaskState) -> bool {
        use TaskState::*;
        matches!(
            (self, to),
            // Ready can only become Running
            (Ready, Running) |
            // Running can go anywhere except Dead directly
            (Running, Ready) |
            (Running, Sleeping { .. }) |
            (Running, Waiting { .. }) |
            (Running, Exiting { .. }) |
            // Blocked states can wake or be killed
            (Sleeping { .. }, Ready) |
            (Sleeping { .. }, Exiting { .. }) |
            (Waiting { .. }, Ready) |
            (Waiting { .. }, Exiting { .. }) |
            // Exiting goes to Dead
            (Exiting { .. }, Dead) |
            // Dead can become Ready (slot reuse)
            (Dead, Ready)
        )
    }

    pub fn is_runnable(&self) -> bool {
        matches!(self, TaskState::Ready | TaskState::Running)
    }

    pub fn is_blocked(&self) -> bool {
        matches!(self, TaskState::Sleeping { .. } | TaskState::Waiting { .. })
    }

    pub fn deadline(&self) -> Option<u64> {
        match self {
            TaskState::Waiting { deadline, .. } => Some(*deadline),
            _ => None,
        }
    }
}
```

### Schedulable Trait

```rust
// src/kernel/task/traits.rs

use super::state::TaskState;
use crate::hal::context::Context;

/// What the scheduler needs to know about a task
pub trait Schedulable {
    type Context: Context;

    /// Unique task identifier
    fn id(&self) -> u32;

    /// Current state
    fn state(&self) -> TaskState;

    /// Scheduling priority (lower = higher priority)
    fn priority(&self) -> u8;

    /// Mutable access to kernel context for switching
    fn context_mut(&mut self) -> &mut Self::Context;

    /// Immutable access to kernel context
    fn context(&self) -> &Self::Context;

    // State transitions (return Err if invalid)

    fn set_running(&mut self) -> Result<(), InvalidTransition>;
    fn set_ready(&mut self) -> Result<(), InvalidTransition>;
    fn set_sleeping(&mut self, reason: SleepReason) -> Result<(), InvalidTransition>;
    fn set_waiting(&mut self, reason: WaitReason, deadline: u64) -> Result<(), InvalidTransition>;
    fn wake(&mut self) -> Result<(), InvalidTransition>;
    fn exit(&mut self, code: i32) -> Result<(), InvalidTransition>;
}

#[derive(Debug)]
pub struct InvalidTransition {
    pub from: &'static str,
    pub to: &'static str,
}
```

### Task Implementation

```rust
// src/kernel/task/task.rs

use super::{state::*, traits::*};
use crate::hal::context::Context;

/// Priority levels
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Priority {
    High = 0,
    Normal = 1,
    Low = 2,
    Idle = 3,  // Only for idle task
}

/// A task (kernel or user)
pub struct Task<C: Context> {
    id: u32,
    state: TaskState,
    priority: Priority,
    context: C,

    // User task specific
    pub trap_frame: Option<TrapFrame>,
    pub address_space: Option<AddressSpace>,

    // Resources
    pub name: [u8; 16],
    pub parent_id: u32,
    // ... other fields
}

impl<C: Context> Task<C> {
    /// Create a new kernel task
    pub fn new_kernel(id: u32, entry: fn() -> !, stack: *mut u8, name: &str) -> Self {
        let mut task = Self {
            id,
            state: TaskState::Ready,
            priority: Priority::Normal,
            context: C::new_at(entry, stack),
            trap_frame: None,
            address_space: None,
            name: [0; 16],
            parent_id: 0,
        };
        // Copy name
        let bytes = name.as_bytes();
        let len = bytes.len().min(15);
        task.name[..len].copy_from_slice(&bytes[..len]);
        task
    }

    /// Create idle task (special priority)
    pub fn new_idle(id: u32, entry: fn() -> !, stack: *mut u8) -> Self {
        let mut task = Self::new_kernel(id, entry, stack, "idle");
        task.priority = Priority::Idle;
        task
    }

    /// Create user task (context set up later when entering usermode)
    pub fn new_user(id: u32, name: &str, address_space: AddressSpace) -> Self {
        Self {
            id,
            state: TaskState::Ready,
            priority: Priority::Normal,
            context: C::default(),
            trap_frame: Some(TrapFrame::new()),
            address_space: Some(address_space),
            name: {
                let mut n = [0; 16];
                let bytes = name.as_bytes();
                let len = bytes.len().min(15);
                n[..len].copy_from_slice(&bytes[..len]);
                n
            },
            parent_id: 0,
        }
    }
}

impl<C: Context> Schedulable for Task<C> {
    type Context = C;

    fn id(&self) -> u32 { self.id }
    fn state(&self) -> TaskState { self.state }
    fn priority(&self) -> u8 { self.priority as u8 }
    fn context_mut(&mut self) -> &mut C { &mut self.context }
    fn context(&self) -> &C { &self.context }

    fn set_running(&mut self) -> Result<(), InvalidTransition> {
        if self.state.can_transition_to(&TaskState::Running) {
            self.state = TaskState::Running;
            Ok(())
        } else {
            Err(InvalidTransition {
                from: self.state.name(),
                to: "Running"
            })
        }
    }

    fn set_ready(&mut self) -> Result<(), InvalidTransition> {
        if self.state.can_transition_to(&TaskState::Ready) {
            self.state = TaskState::Ready;
            Ok(())
        } else {
            Err(InvalidTransition {
                from: self.state.name(),
                to: "Ready"
            })
        }
    }

    fn set_sleeping(&mut self, reason: SleepReason) -> Result<(), InvalidTransition> {
        let new_state = TaskState::Sleeping { reason };
        if self.state.can_transition_to(&new_state) {
            self.state = new_state;
            Ok(())
        } else {
            Err(InvalidTransition {
                from: self.state.name(),
                to: "Sleeping"
            })
        }
    }

    fn set_waiting(&mut self, reason: WaitReason, deadline: u64) -> Result<(), InvalidTransition> {
        let new_state = TaskState::Waiting { reason, deadline };
        if self.state.can_transition_to(&new_state) {
            self.state = new_state;
            Ok(())
        } else {
            Err(InvalidTransition {
                from: self.state.name(),
                to: "Waiting"
            })
        }
    }

    fn wake(&mut self) -> Result<(), InvalidTransition> {
        if self.state.can_transition_to(&TaskState::Ready) {
            self.state = TaskState::Ready;
            Ok(())
        } else {
            Err(InvalidTransition {
                from: self.state.name(),
                to: "Ready"
            })
        }
    }

    fn exit(&mut self, code: i32) -> Result<(), InvalidTransition> {
        let new_state = TaskState::Exiting { code };
        if self.state.can_transition_to(&new_state) {
            self.state = new_state;
            Ok(())
        } else {
            Err(InvalidTransition {
                from: self.state.name(),
                to: "Exiting"
            })
        }
    }
}
```

---

## Scheduler Layer (`src/kernel/sched/`)

### Scheduling Policy Trait

```rust
// src/kernel/sched/policy.rs

use crate::kernel::task::traits::Schedulable;

/// Scheduling algorithm
pub trait SchedulingPolicy<T: Schedulable> {
    /// Select next task to run
    ///
    /// # Arguments
    /// - `current_slot` - Currently running task's slot
    /// - `tasks` - All task slots
    ///
    /// # Returns
    /// - Slot of task to run (always returns something - at minimum, idle task)
    fn select_next(&mut self, current_slot: usize, tasks: &[Option<T>]) -> usize;

    /// Called when task becomes ready
    fn on_task_ready(&mut self, _slot: usize, _priority: u8) {}

    /// Called when task blocks
    fn on_task_blocked(&mut self, _slot: usize) {}

    /// Called when task exits
    fn on_task_exit(&mut self, _slot: usize) {}

    /// Policy name for debugging
    fn name(&self) -> &'static str;
}
```

### Priority Round-Robin Policy

```rust
// src/kernel/sched/priority_rr.rs

use super::policy::SchedulingPolicy;
use crate::kernel::task::{traits::Schedulable, state::TaskState};

pub struct PriorityRoundRobin {
    /// Slot 0 is always idle task
    idle_slot: usize,
}

impl PriorityRoundRobin {
    pub const fn new() -> Self {
        Self { idle_slot: 0 }
    }
}

impl<T: Schedulable> SchedulingPolicy<T> for PriorityRoundRobin {
    fn select_next(&mut self, current_slot: usize, tasks: &[Option<T>]) -> usize {
        let mut best_slot = self.idle_slot;  // Default to idle
        let mut best_priority = u8::MAX;

        // Scan from current+1 for round-robin fairness
        let n = tasks.len();
        let start = (current_slot + 1) % n;

        for i in 0..n {
            let slot = (start + i) % n;

            // Skip idle in first pass (it's the fallback)
            if slot == self.idle_slot {
                continue;
            }

            if let Some(ref task) = tasks[slot] {
                if task.state() == TaskState::Ready {
                    let priority = task.priority();
                    if priority < best_priority {
                        best_slot = slot;
                        best_priority = priority;

                        // Early exit for highest priority (0)
                        if priority == 0 {
                            break;
                        }
                    }
                }
            }
        }

        // If no ready task found, return idle
        // Idle task is ALWAYS ready (it never blocks)
        best_slot
    }

    fn name(&self) -> &'static str {
        "PriorityRoundRobin"
    }
}
```

### Scheduler

```rust
// src/kernel/sched/scheduler.rs

use crate::hal::{cpu::Cpu, context::ContextSwitch, timer::Timer};
use crate::kernel::task::{traits::Schedulable, state::*};
use super::policy::SchedulingPolicy;

pub const MAX_TASKS: usize = 16;
pub const IDLE_SLOT: usize = 0;

/// The scheduler
pub struct Scheduler<T, C, P, Tm>
where
    T: Schedulable,
    C: ContextSwitch<Ctx = T::Context>,
    P: SchedulingPolicy<T>,
    Tm: Timer,
{
    tasks: [Option<T>; MAX_TASKS],
    current_slot: usize,
    policy: P,

    // HAL references
    context_switch: C,
    timer: Tm,

    // Deadline tracking for tickless
    next_deadline: u64,
}

impl<T, C, P, Tm> Scheduler<T, C, P, Tm>
where
    T: Schedulable,
    C: ContextSwitch<Ctx = T::Context>,
    P: SchedulingPolicy<T>,
    Tm: Timer,
{
    /// Create scheduler with idle task
    pub fn new(idle_task: T, policy: P, context_switch: C, timer: Tm) -> Self {
        let mut tasks: [Option<T>; MAX_TASKS] = Default::default();
        tasks[IDLE_SLOT] = Some(idle_task);

        Self {
            tasks,
            current_slot: IDLE_SLOT,
            policy,
            context_switch,
            timer,
            next_deadline: u64::MAX,
        }
    }

    /// Get current task
    pub fn current(&self) -> Option<&T> {
        self.tasks[self.current_slot].as_ref()
    }

    /// Get current task mutably
    pub fn current_mut(&mut self) -> Option<&mut T> {
        self.tasks[self.current_slot].as_mut()
    }

    /// Get current slot
    pub fn current_slot(&self) -> usize {
        self.current_slot
    }

    /// Add a task, returns slot
    pub fn add_task(&mut self, task: T) -> Option<usize> {
        // Find empty slot (skip idle slot)
        for slot in 1..MAX_TASKS {
            if self.tasks[slot].is_none() {
                self.tasks[slot] = Some(task);
                return Some(slot);
            }
        }
        None
    }

    /// Get task by slot
    pub fn task(&self, slot: usize) -> Option<&T> {
        self.tasks.get(slot)?.as_ref()
    }

    /// Get task mutably by slot
    pub fn task_mut(&mut self, slot: usize) -> Option<&mut T> {
        self.tasks.get_mut(slot)?.as_mut()
    }

    /// Record a deadline for tickless scheduling
    pub fn note_deadline(&mut self, deadline: u64) {
        if deadline < self.next_deadline {
            self.next_deadline = deadline;
        }
    }

    /// Reschedule - pick next task and switch to it
    ///
    /// This is the main scheduling entry point. It:
    /// 1. Marks current task as Ready (if it was Running)
    /// 2. Picks the next task to run via policy
    /// 3. Context switches to that task
    ///
    /// When this function returns, we've been switched back.
    pub fn reschedule(&mut self) {
        let from_slot = self.current_slot;

        // Mark current as Ready if it was Running
        // (Blocked tasks stay blocked)
        if let Some(ref mut current) = self.tasks[from_slot] {
            if current.state() == TaskState::Running {
                let _ = current.set_ready();
            }
        }

        // Select next task
        let to_slot = self.policy.select_next(from_slot, &self.tasks);

        // If same task, just ensure it's Running
        if to_slot == from_slot {
            if let Some(ref mut task) = self.tasks[from_slot] {
                let _ = task.set_running();
            }
            return;
        }

        // Switch to different task
        self.switch_to(from_slot, to_slot);
    }

    /// Switch from one task to another
    fn switch_to(&mut self, from_slot: usize, to_slot: usize) {
        // Get context pointers
        let from_ctx = match self.tasks[from_slot].as_mut() {
            Some(t) => t.context_mut() as *mut T::Context,
            None => return,
        };
        let to_ctx = match self.tasks[to_slot].as_ref() {
            Some(t) => t.context() as *const T::Context,
            None => return,
        };

        // Update current slot BEFORE switch
        self.current_slot = to_slot;

        // Mark target as Running
        if let Some(ref mut task) = self.tasks[to_slot] {
            let _ = task.set_running();
        }

        // Do the context switch
        // When this returns, we've been switched BACK to from_slot
        unsafe {
            C::switch(&mut *from_ctx, &*to_ctx);
        }

        // === Resumed here after being switched back ===

        // Restore our slot
        self.current_slot = from_slot;

        // Mark ourselves as Running
        if let Some(ref mut task) = self.tasks[from_slot] {
            let _ = task.set_running();
        }
    }

    /// Wake a task by slot
    pub fn wake(&mut self, slot: usize) -> bool {
        if let Some(ref mut task) = self.tasks[slot] {
            if task.state().is_blocked() {
                task.wake().is_ok()
            } else {
                false
            }
        } else {
            false
        }
    }

    /// Check all deadlines and wake expired tasks
    pub fn check_deadlines(&mut self) -> usize {
        let now = self.timer.counter();

        // Early return if no deadlines due
        if now < self.next_deadline {
            return 0;
        }

        let mut woken = 0;
        let mut earliest = u64::MAX;

        for slot in 0..MAX_TASKS {
            if let Some(ref mut task) = self.tasks[slot] {
                if let Some(deadline) = task.state().deadline() {
                    if now >= deadline {
                        // Deadline passed - wake
                        if task.wake().is_ok() {
                            woken += 1;
                        }
                    } else if deadline < earliest {
                        earliest = deadline;
                    }
                }
            }
        }

        self.next_deadline = earliest;
        woken
    }

    /// Compute next deadline for tickless idle
    pub fn compute_next_deadline(&self) -> Option<u64> {
        if self.next_deadline == u64::MAX {
            None
        } else {
            Some(self.next_deadline)
        }
    }
}
```

---

## Idle Task

```rust
// src/kernel/idle.rs

use crate::hal::cpu::Cpu;

/// Idle task entry point
///
/// This is a kernel task that runs when no other task is ready.
/// It simply loops, calling WFI to save power.
///
/// The idle task NEVER blocks - it's always Ready or Running.
pub fn idle_entry<C: Cpu>(cpu: &C) -> ! {
    loop {
        // Wait for interrupt
        // When IRQ fires, we'll return here, check_deadlines runs,
        // and if a task is ready, we get context-switched away
        cpu.idle();

        // Note: We don't call reschedule() here.
        // The timer IRQ handler does that.
        // When we're switched back, we just loop and idle again.
    }
}

/// Stack for idle task (small, it does almost nothing)
#[repr(align(16))]
pub struct IdleStack([u8; 4096]);

impl IdleStack {
    pub const fn new() -> Self {
        Self([0; 4096])
    }

    pub fn top(&self) -> *mut u8 {
        unsafe { self.0.as_ptr().add(4096) as *mut u8 }
    }
}

pub static mut IDLE_STACK: IdleStack = IdleStack::new();
```

---

## Initialization

```rust
// src/kernel/init.rs

use crate::hal::{cpu::Cpu, context::ContextSwitch, timer::Timer};
use crate::kernel::{
    task::Task,
    sched::{Scheduler, PriorityRoundRobin},
    idle::{idle_entry, IDLE_STACK},
};

/// Initialize the scheduler with idle task
pub fn init_scheduler<C, CS, T>(cpu: C, context_switch: CS, timer: T) -> Scheduler<Task<CS::Ctx>, CS, PriorityRoundRobin, T>
where
    C: Cpu + 'static,
    CS: ContextSwitch,
    T: Timer,
{
    // Create idle task
    let idle_stack_top = unsafe { IDLE_STACK.top() };

    // The idle entry needs the CPU reference - we use a wrapper
    fn idle_wrapper() -> ! {
        // Access global CPU (or pass through TLS)
        let cpu = get_cpu();
        idle_entry(cpu)
    }

    let idle_task = Task::new_idle(
        0,  // ID 0 for idle
        idle_wrapper,
        idle_stack_top,
    );

    let policy = PriorityRoundRobin::new();

    Scheduler::new(idle_task, policy, context_switch, timer)
}
```

---

## IRQ Handler Integration

```rust
// src/kernel/irq.rs

/// Timer IRQ handler (called from assembly)
pub fn handle_timer_irq<S>(sched: &mut S)
where
    S: /* scheduler trait bounds */
{
    // 1. Check deadlines, wake any expired tasks
    sched.check_deadlines();

    // 2. Reschedule (will switch if higher-priority task is ready)
    sched.reschedule();

    // Note: If we were in idle task and a user task is now ready,
    // reschedule() will context_switch to it.
    // When that task blocks, it will context_switch back to idle.
}
```

---

## Blocking Syscall Flow

With this design, a blocking syscall like `mux.wait()` works cleanly:

```
1. User task calls mux.wait() syscall
2. Kernel: No events ready, set task state to Sleeping
3. Kernel: Call sched.reschedule()
4. Scheduler: Current task is Sleeping, select next task (maybe idle)
5. Scheduler: context_switch(current, idle)
   - Saves current task's kernel context
   - Loads idle task's context
   - Idle task resumes in its loop

6. [Time passes, idle does WFI]

7. Timer IRQ fires
8. IRQ handler: check_deadlines() wakes user task (Sleeping -> Ready)
9. IRQ handler: reschedule()
10. Scheduler: Idle is Running, user task is Ready (higher priority)
11. Scheduler: context_switch(idle, user_task)
    - Saves idle's context
    - Loads user task's context
    - User task resumes EXACTLY where it was in the syscall

12. User task: Continues after reschedule() call
13. User task: Polls mux, finds events, returns to userspace
```

---

## Testing

With traits, we can easily test the scheduler:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    /// Mock task for testing
    struct MockTask {
        id: u32,
        state: TaskState,
        priority: u8,
        context: MockContext,
    }

    #[derive(Default, Clone)]
    struct MockContext {
        switch_count: u32,
    }

    impl Context for MockContext {
        fn new_at(_entry: fn() -> !, _stack: *mut u8) -> Self {
            Self::default()
        }
        fn stack_pointer(&self) -> *mut u8 { core::ptr::null_mut() }
        fn set_stack_pointer(&mut self, _: *mut u8) {}
    }

    struct MockContextSwitch;

    impl ContextSwitch for MockContextSwitch {
        type Ctx = MockContext;

        unsafe fn switch(current: &mut Self::Ctx, _next: &Self::Ctx) {
            current.switch_count += 1;
            // In tests, we don't actually switch - just record it happened
        }
    }

    #[test]
    fn test_scheduler_selects_highest_priority() {
        // Create scheduler with mock components
        // Add tasks with different priorities
        // Verify policy selects correctly
    }

    #[test]
    fn test_blocked_task_not_selected() {
        // Create scheduler
        // Block a task
        // Verify it's not selected
    }

    #[test]
    fn test_wake_transitions_to_ready() {
        // Create blocked task
        // Wake it
        // Verify state is Ready
    }
}
```

---

## Migration Notes

This is a clean redesign. Key differences from current code:

1. **Idle task is real** - Created at boot in slot 0, always has valid context
2. **Traits everywhere** - Scheduler uses `Schedulable`, HAL uses `Cpu`/`ContextSwitch`
3. **No enter_idle()** - Idle is just another task, scheduler switches to it normally
4. **No SYSCALL_SWITCHED_TASK hack** - Context switch handles everything
5. **Testable** - Mock implementations for unit tests

Files to create/modify:
- `src/hal/cpu.rs` - CPU trait
- `src/hal/context.rs` - Context/ContextSwitch traits
- `src/hal/timer.rs` - Timer trait
- `src/arch/aarch64/hal.rs` - AArch64 implementations
- `src/kernel/task/state.rs` - State enum (exists, clean up)
- `src/kernel/task/traits.rs` - Schedulable trait (new)
- `src/kernel/task/task.rs` - Task implementation
- `src/kernel/sched/policy.rs` - Policy trait (exists, update)
- `src/kernel/sched/scheduler.rs` - New scheduler implementation
- `src/kernel/idle.rs` - Idle task (new)
- Delete: Most of `src/kernel/sched.rs` (old scheduler)
