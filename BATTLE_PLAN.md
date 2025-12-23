# Kernel Battle Plan

This is a **merged, prioritized action plan** combining the existing kernel.md roadmap with critical bugs found during code review. Organized by urgency: fix correctness bugs first, then harden, then optimize.

---

## Phase 0: Critical Correctness Bugs (FIX IMMEDIATELY)

These are **showstopper bugs** that can cause crashes, security holes, or data corruption right now.

### 0.1 sys_exit Use-After-Free
**Location**: `src/kernel/syscall.rs:362-438`

**Bug**: `sys_exit()` marks the current task as terminated, then returns. The return path continues executing on the terminated task's stack, which can be freed/reused by another CPU or the reaper.

**Fix**:
```rust
// sys_exit must NEVER return
fn sys_exit(code: i32) -> ! {
    mark_terminated();
    // Switch to a safe stack (idle/scheduler stack) BEFORE freeing anything
    switch_to_idle_and_reap();
    unreachable!();
}
```

**Success**: `sys_exit` has return type `!` and never returns to user stack.

---

### 0.2 sys_kill Missing Authorization
**Location**: `src/kernel/syscall.rs:1557-1632`

**Bug**: Any process can kill any other process. No permission check.

**Fix**:
- Only allow killing children, or same-UID processes
- Or implement capability-based kill permission
- At minimum: prevent killing PID 1 (init)

**Success**: Unprivileged process cannot kill arbitrary PIDs.

---

### 0.3 Shmem Reference Count Ignored
**Location**: `src/kernel/shmem.rs:197` (refcount field), `src/kernel/shmem.rs:344-369` (destroy)

**Bug**: `ShmemRegion` has a `ref_count` field but it's never checked. `shmem_destroy` frees memory even if other processes have it mapped → UAF.

**Fix**:
```rust
fn shmem_destroy(id: u32) -> Result<(), ShmemError> {
    let region = get_region(id)?;
    if region.ref_count > 1 {
        return Err(ShmemError::StillMapped);
    }
    // Actually free
}
```

**Success**: Shmem destroy fails if ref_count > 1.

---

### 0.4 IPC Channel Creation Race
**Location**: `src/kernel/ipc.rs:288-313`

**Bug**: Channel pair creation is not atomic. If interrupted between creating endpoints, state is inconsistent.

**Fix**: Wrap channel creation in a lock or make it a single atomic operation.

---

### 0.5 uaccess Physical Address Validation Gap
**Location**: `src/kernel/uaccess.rs:250-316` (`verify_page_mapped`)

**Bug**: Validates virtual mapping exists but doesn't verify the physical address is actually user-owned memory (could be MMIO, kernel memory).

**Fix**: Check that resolved physical address is in user-allocatable range, not MMIO or kernel.

---

## Phase 1: Locking Infrastructure (Required for Everything Else)

Before fixing PMM races or scheduler issues, we need proper locks.

### 1.1 Implement SpinLock<T> with IrqSave
**From**: kernel.md 4.1, 7.3

**Actions**:
- Implement `SpinLock<T>` that disables IRQs on lock
- `lock()` returns a guard that re-enables IRQs on drop
- Debug mode: track lock owner, detect deadlocks

```rust
pub struct SpinLock<T> {
    locked: AtomicBool,
    data: UnsafeCell<T>,
}

impl<T> SpinLock<T> {
    pub fn lock(&self) -> SpinLockGuard<T> {
        let irq_state = disable_irqs();
        while self.locked.swap(true, Ordering::Acquire) {
            core::hint::spin_loop();
        }
        SpinLockGuard { lock: self, irq_state }
    }
}
```

**Success**: All shared mutable state accessed through locks.

---

### 1.2 Per-CPU Infrastructure
**From**: kernel.md 7.2

**Actions**:
- Add `PerCpu<T>` abstraction
- Set `TPIDR_EL1` to per-CPU base on each CPU
- Implement `cpu_id()`, `current_task()`, `this_cpu()`

**Success**: No global `static mut CURRENT_TASK`.

---

## Phase 2: PMM Hardening

### 2.1 Add Spinlock to PMM
**From**: My review (lines 256-273)

**Bug**: `alloc_page()`, `free_page()` are not thread-safe. Multiple CPUs can corrupt the bitmap.

**Fix**: Wrap PMM state in `SpinLock<PmmState>`.

---

### 2.2 Dynamic Kernel Size from Linker Symbols
**From**: kernel.md 2.1

**Bug**: Hardcoded `KERNEL_SIZE` will eventually overlap live kernel memory.

**Fix**:
- Export `__kernel_start`, `__kernel_end` from linker script
- Compute reserved ranges at boot
- Mark reserved pages in bitmap

---

### 2.3 Ownership Semantics
**From**: kernel.md 2.2

**Actions**:
- Document ownership rules
- Add `PageOwner` enum: `Kernel`, `User(pid)`, `Device`, `Shared(shmem_id)`
- Assert no double-free in debug builds

---

## Phase 3: Scheduler Correctness

### 3.1 Fix Scheduler Current Index Race
**Location**: `src/kernel/task.rs:1176-1241`

**Bug**: `current_idx` is modified without locking. On SMP, this is a data race.

**Fix**: Use atomic or per-CPU current task pointer.

---

### 3.2 Per-CPU Runqueues
**From**: kernel.md 7.4

**Actions**:
- Move runqueue to per-CPU structures
- Simple work stealing for idle CPUs
- Global task list for introspection only

---

### 3.3 Task Entry Trampoline
**From**: kernel.md 4.2

**Bug**: Using `x30` for entry point complicates lifecycle.

**Fix**: Trampoline that calls entry, then `task_exit()` on return.

---

## Phase 4: Address Space & TLB

### 4.1 ASID Allocator
**From**: kernel.md 1.1

**Bug**: Global `tlbi vmalle1` on every context switch is catastrophic.

**Actions**:
- 8-16 bit ASID allocator
- Store ASID in `AddressSpace`
- Program TTBR0 with `{asid, root}`

---

### 4.2 Centralized TLB API
**From**: kernel.md 1.2

**Actions**:
- Create `arch::tlb` module
- `tlbi_user_asid(asid)`, `tlbi_user_va(asid, va)`
- Ban ad-hoc inline asm outside this module

---

### 4.3 TLB Shootdowns
**From**: kernel.md 7.6

**Actions** (after ASID + IPI):
- Track which CPUs have each ASID active
- On unmap: invalidate locally, IPI others, wait for ack

---

## Phase 5: IPC Hardening

### 5.1 Fix TOCTOU in IPC
**From**: My review

**Bug**: User pointer validation and access are not atomic.

**Fix**: Combine validation + copy in a single critical section with PAN control.

---

### 5.2 Explicit IPC Backpressure
**From**: kernel.md 6.2

**Actions**:
- Make queue full/empty explicitly observable
- Add blocking send/recv with timeout

---

### 5.3 Zero-Copy Large IPC
**From**: kernel.md 6.1

**Actions** (lower priority):
- Page-grant/lend for large payloads
- Reuse shmem infrastructure

---

## Phase 6: Platform Abstraction (RPi Support)

### 6.1 HAL Trait Definitions
**From**: User requirement (RPi 3/4/5 support)

**Actions**:
- Define `trait InterruptController { fn enable_irq(), fn ack_irq(), ... }`
- Define `trait Timer { fn init(), fn set_interval(), ... }`
- Define `trait Uart { fn putc(), fn getc(), ... }`

---

### 6.2 GICv2 vs GICv3 Abstraction

**Current**: Only GIC-600 (GICv3) for MT7988A
**Needed**: GICv2 for RPi 3, GICv3 for RPi 4/5

**Actions**:
- `arch::gic` module with runtime detection or compile-time selection
- Abstract redistributor vs CPU interface differences

---

### 6.3 Platform-Specific Init
**Actions**:
- `platform::init()` called from generic kernel
- Platform provides: UART, GIC, Timer, GPIO base addresses
- Board detection via DTB or compile-time feature

---

## Phase 7: VM Consolidation

### 7.1 Single Mapping Helper
**From**: kernel.md 5.1

**Bug**: `mmap`, `mmap_dma`, `mmap_phys`, `mmap_device` duplicate logic.

**Fix**: Single `map_region()` with flags for allocation/fixed/device.

---

### 7.2 phys↔virt Helpers
**From**: kernel.md 5.2

**Bug**: Ad-hoc `KERNEL_VIRT_BASE | phys` is fragile.

**Fix**: `phys_to_kva()`, `kva_to_phys()` helpers.

---

## Phase 8: Debug & Safety

### 8.1 PAN Enforcement
**From**: kernel.md 3.1

**Actions**:
- Enable PAN permanently
- Only clear inside uaccess helpers
- Crash on accidental user pointer dereference

---

### 8.2 Lock Debug Assertions
**From**: kernel.md 8.1

**Actions**:
- Track lock owner in debug builds
- Assert "no sleeping while holding spinlock"
- Per-CPU ring buffer logs (avoid global log lock)

---

## Execution Order (Recommended)

```
Week 1: Phase 0 (critical bugs) + Phase 1.1 (SpinLock)
         └─ sys_exit UAF, kill auth, shmem refcount
         └─ SpinLock<T> implementation

Week 2: Phase 1.2 (per-CPU) + Phase 2 (PMM)
         └─ TPIDR_EL1, cpu_id(), current_task()
         └─ PMM spinlock, linker symbols

Week 3: Phase 3 (scheduler)
         └─ current_idx race, per-CPU runqueues
         └─ Task entry trampoline

Week 4: Phase 4 (ASID + TLB)
         └─ ASID allocator
         └─ Centralized TLB API

Week 5+: Phase 5-8 (IPC, abstraction, debug)
         └─ Can be done incrementally
```

---

## Success Milestones

1. **M1**: No UAF bugs, kill requires permission → security baseline
2. **M2**: PMM thread-safe, per-CPU infrastructure → SMP ready
3. **M3**: Scheduler uses per-CPU queues → 2-core boot works
4. **M4**: ASID + TLB shootdowns → context switch is fast
5. **M5**: HAL traits defined → RPi 3 compiles
6. **M6**: All platforms boot → RPi 3/4/5 + BPI-R4 supported

---

## Files to Modify (Priority Order)

| File | Changes |
|------|---------|
| `src/kernel/syscall.rs` | sys_exit UAF, kill auth |
| `src/kernel/shmem.rs` | refcount enforcement |
| `src/kernel/lock.rs` | NEW: SpinLock<T> |
| `src/kernel/percpu.rs` | NEW: PerCpu<T> |
| `src/kernel/pmm.rs` | spinlock, linker symbols |
| `src/kernel/task.rs` | per-CPU runqueue, trampoline |
| `src/kernel/mmu.rs` | ASID allocator |
| `src/arch/tlb.rs` | NEW: centralized TLB |
| `src/platform/mod.rs` | HAL traits |

---

## What NOT to Do

- Don't add features until correctness bugs are fixed
- Don't optimize before locking is in place
- Don't add more platforms before abstractions exist
- Don't add zero-copy IPC before basic IPC is hardened

---

This plan is aggressive but achievable. Phase 0 + 1 are the foundation. Everything else builds on them.
