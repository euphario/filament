# Design Principles

This document defines the core design philosophy. These are **hard rules** that must be followed in all code.

## Philosophy

> **Every boundary is a state machine. No exceptions.**

This microkernel is built on one fundamental principle: all communication across trust boundaries, process boundaries, or hardware boundaries MUST be governed by explicit, documented state machines.

### Why?

1. **Determinism** - Same inputs produce same outputs. Always.
2. **Observability** - You can always answer "what state is it in?"
3. **Recoverability** - Fault handling is a state transition, not ad-hoc cleanup.
4. **Live Update** - State machines are serializable. Functions mid-execution are not.
5. **Verifiability** - Explicit states can be formally verified.

---

## Architecture & Portability

### SMP Compatibility Always
All code must be safe for multi-core execution. Use atomic operations, proper locking, per-CPU data structures where needed. Never assume single-core.

### Clear Separation of Concerns
Each module has a single responsibility. No god objects. One file should do one thing.

### Hardware Abstraction
No chip-specific (MT7988A) or board-specific (BPI-R4) code outside `src/platform/` and `src/arch/`. Upper layers use traits for hardware access.

### Use Traits for Abstraction
Define behavior through traits, implement for specific hardware. This enables testing and portability.

---

## Code Quality & Security

### Production Hardening Mindset
Write code as if it's going to production. No "good enough for now". Every line matters.

### Security First
Capability checks for privileged operations. Validate all user input. No trust across privilege boundaries.

### No TODOs Without Agreement
Don't leave TODO comments without explicit user approval. Either implement fully or document as a known limitation.

### Document Design Alongside Code
Design rationale at top of files or in adjacent markdown. Code should be self-documenting but complex decisions need explanation.

---

## State & Control Flow

### State Machine Design
Model complex behavior as explicit state machines. States, transitions, and events should be clear and documented. Draw the state machine first, code second.

### No Busy-Wait Loops
Blocked tasks MUST WFI (Wait For Interrupt) or yield. Use kernel timer events for scheduling, not polling loops. Never spin waiting for hardware.

### Event-Driven Where Possible
Prefer event queues over polling. Wake on events, not timeouts. The kernel has a complete event subscription system - use it.

---

## Scheduling & Concurrency

### Blocked Tasks Never Spin
A blocked task must either:
- WFI (in kernel context)
- Yield to Ready tasks (in user context)
- Never consume CPU cycles waiting

### Timer Events Use kernel tick_count
Not raw hardware counter. This provides a consistent time base across all components.

### IRQ Handlers Set Flags Only
Actual work (preemption, task switch) happens at safe points. IRQ handlers must be minimal - set a flag and return.

---

## Event System

### Critical Events Never Dropped
ChildExit and Signal events use a priority queue separate from normal events. These are never lost even under load.

### Timer Events Direct to Task
No subscription filtering needed for timer events. They go directly to the requesting task.

### Receiver Allowlist for Signals
Opt-in model for inter-process signals. A process must explicitly allow signals from specific senders.

### Subscribe Before Waiting
Tasks must subscribe to event types they care about before calling event_wait().

---

## Driver Supervision (devd)

### Exponential Backoff
Crash recovery uses increasing delays:
- First failures: fast retry
- Repeated failures: longer delays
- Maximum reached: mark device dead

### Bus Reset Before Restart
Clean hardware state before driver restart. Never restart a driver on dirty hardware.

**Sequence:**
1. Driver crashes → kernel resets bus hardware → bus enters Safe state
2. Kernel sends StateSnapshot(Safe) to devd
3. devd receives Safe signal → checks if escalation delay passed
4. If delay passed: restart immediately. If not: set timer for remaining delay.

### Wait for Bus Safe
Driver restart is gated on bus state, not just a timer. The kernel is authoritative on when hardware is ready.

### Auto-Start vs Manual Start
Track how drivers were started:
- **Auto-started** (by devd via match rules): eligible for auto-restart on crash
- **Manually started** (by shell command): NOT auto-restarted on crash

This prevents surprise restarts for debugging sessions.

### Kernel Timer Events
No userspace polling for scheduling. Use kernel timer events for all timeouts and delays.

---

## Fault Handling

### Userspace Faults Don't Halt System
When a userspace task faults (SIGSEGV, SIGBUS, etc.):
1. Task is terminated with signal-style exit code
2. Task's children are killed
3. Task's resources are cleaned up (IPC, shmem, bus ownership)
4. Parent is notified via ChildExit event
5. System continues running

### devd Crash Recovery
devd (PID 1) is special. If devd crashes:
1. Kill all other tasks
2. Reset all buses to Safe state
3. Respawn devd from initrd with full capabilities
4. System continues (not halt)

### Exit Codes Reflect Signal
Exit codes for faulted tasks use negative signal numbers:
- `-11` = SIGSEGV (memory fault)
- `-7` = SIGBUS (alignment, bus error)
- `-6` = SIGABRT (other faults)

---

## Memory Rules

### DMA Memory is Non-Cacheable
All DMA-capable memory (shmem, dma_pool, ring buffers) is mapped as **Normal Non-Cacheable** in userspace. This is by design:
- No cache coherency protocol on MT7988A PCIe
- No explicit cache flushes needed in driver code
- CPU writes immediately visible to device

### No Cache Flush in Driver Code
The kernel flushes cache when allocating DMA memory. Drivers never need to call `dc cvac` or similar.

### Memory Barriers Before I/O
Use `dsb sy` before kicking device queues. This ensures write ordering.

---

## Layer Boundaries

```
┌─────────────────────────────────────────┐
│  Userspace (shell, drivers, apps)       │  ← No HW access except via syscalls
├─────────────────────────────────────────┤
│  Kernel services (IPC, events, shmem)   │  ← Platform-agnostic
├─────────────────────────────────────────┤
│  Platform HAL (src/platform/mt7988/)    │  ← SoC-specific, trait implementations
├─────────────────────────────────────────┤
│  Arch (src/arch/aarch64/)               │  ← CPU architecture specific
└─────────────────────────────────────────┘
```

### What Goes Where

| Layer | Responsibility | Examples |
|-------|----------------|----------|
| **Userspace** | Application logic, device drivers, IPC | shell, devd, pcied, wifid, usbd |
| **Kernel Services** | Task scheduling, IPC, memory management | syscalls, events, shmem |
| **Platform HAL** | SoC-specific initialization and traits | MT7988A clocks, UART init |
| **Architecture** | CPU-specific code | ARM64 MMU, exception handlers |

---

## Microkernel Philosophy

The kernel is **minimal** - provides raw capabilities only:
- `bus_list()` - Expose buses from DTB
- `mmap_device(phys_addr, size)` - Map device MMIO (with capability check)
- `shmem_create()` - DMA-capable shared memory
- `irq_claim()` - Claim interrupt

**NOT in kernel**:
- No device registry
- No PCI/USB/etc specific logic
- No driver management
- No filesystem
- No network stack

Device management lives in **userspace** with **devd as the single source of truth**.

---

## Quick Reference: Hard Constraints

| Rule | Violation Result | Where Defined |
|------|------------------|---------------|
| SMP safe always | Data corruption, race conditions | Architecture & Portability |
| No TODOs without approval | Code review rejection | Code Quality |
| No polling loops | Power waste, missed events | State & Control Flow |
| DMA memory non-cacheable | Coherency failures | Memory Rules |
| Capability checks always | Security vulnerability | Code Quality |
| Hardware abstraction enforced | Portability loss | Architecture & Portability |
| Event-driven design | Missed events, busy-wait | State & Control Flow |
| State machine explicit | Race conditions | State & Control Flow |

---

## Anti-Patterns to Avoid

### Don't Do

1. **Don't log steps** - Log state transitions and results, not "starting X", "doing Y"
2. **Don't use prose in logs** - Use `key=value` structured format
3. **Don't log in hot paths** - Use TRACE level, compile out in release
4. **Don't block in log path** - Drop messages if buffer full
5. **Don't allocate in log path** - Stack buffers only
6. **Don't busy-wait** - Use events or WFI
7. **Don't poll hardware** - Use interrupts
8. **Don't skip capability checks** - Every privileged operation needs a check
9. **Don't put device logic in kernel** - Drivers belong in userspace
10. **Don't invent new patterns** - Follow existing conventions
