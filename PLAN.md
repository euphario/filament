# Plan: Demand Paging, ASLR, Per-CPU Idle States

Three features, ordered by implementation dependency.

---

## Feature 1: Demand Paging (Lazy Allocation)

### Problem
Every page is committed at mmap/spawn time. This wastes physical memory and blocks
MAX_HANDLES growth (ObjectService tables consume all available .data).

### Design

**Core idea**: mmap() records a VMA but doesn't allocate physical pages. On first access,
the page fault handler allocates one page, zeros it, maps it, and resumes the faulting
instruction.

**What becomes lazy**:
- `mmap()` anonymous pages (OwnedAnon) — biggest win
- BSS segments in ELF loading — zero-fill on demand
- Stack growth — start with 4 pages (16KB), grow on fault up to 64 pages (256KB)

**What stays eager** (no change):
- ELF code/data segments — need file content at load time
- DMA mappings — hardware needs physical addresses immediately
- Shared memory / MMIO — borrowed pages, not ours to lazy-allocate

### Implementation

#### 1a. Extend HeapMapping to track lazy regions

`src/kernel/memory.rs`:
```rust
pub struct HeapMapping {
    pub virt_addr: u64,
    pub phys_addr: u64,      // 0 for lazy (pages allocated individually)
    pub num_pages: usize,
    pub kind: MappingKind,
    pub lazy: bool,           // NEW: pages allocated on demand
    pub writable: bool,       // NEW: needed for fault handler to set permissions
    pub executable: bool,     // NEW: needed for fault handler
}
```

For lazy mappings, `phys_addr` is unused — each page gets its own physical frame on fault.
On cleanup (munmap/exit), walk the page table to find committed pages and free them.

#### 1b. Page fault recovery path in exception handler

`src/main.rs` — `exception_from_user_rust()`:
```rust
pub extern "C" fn exception_from_user_rust(esr: u64, elr: u64, far: u64) {
    let ec = (esr >> 26) & 0x3F;
    let dfsc = esr & 0x3F;

    // Data Abort from lower EL (ec=0x24), Translation Fault (dfsc=0x04..0x07)
    if ec == 0b100100 && (dfsc >= 0x04 && dfsc <= 0x07) {
        if try_demand_page(far) {
            return; // Assembly reloads same trap frame, ERETsback to retry
        }
    }

    // ... existing crash handling (unchanged) ...
}
```

**Key insight**: The assembly after `bl exception_from_user_rust` reloads trap_frame_ptr
and TTBR0 from per-CPU data. If we don't modify those (no scheduler run, no task switch),
it ERETsback to the same faulting instruction. The instruction retries with the page now
mapped. No assembly changes needed.

#### 1c. `try_demand_page()` function

New function in `src/kernel/demand_page.rs`:
```
fn try_demand_page(far: u64) -> bool:
    1. Get current task's slot from scheduler
    2. Look up HeapMapping containing FAR (linear scan of mappings array)
    3. If found and mapping.lazy:
        a. Allocate one physical page from PMM
        b. Zero the page (via TTBR1 kernel mapping)
        c. Map into task's address space (map_page with mapping's permissions)
        d. TLB invalidate for that VA: tlbi vale1is + dsb ish + isb
        e. Return true (handled)
    4. Also check stack region: if FAR is between guard page and current stack bottom,
       allocate and map (stack growth)
    5. Return false (real fault, kill task)
```

#### 1d. Lazy mmap()

`src/kernel/syscall/memory.rs` — change `sys_mmap()`:
- For OwnedAnon: record HeapMapping with `lazy: true`, don't call pmm::alloc_pages
- For OwnedDma/BorrowedShmem/DeviceMmio: keep eager (unchanged)
- Return virtual address immediately

#### 1e. Lazy BSS in ELF loader

`src/kernel/elf.rs` — in `load_elf()`:
- For each PT_LOAD segment where filesz < memsz (BSS region):
  - Only allocate/map pages for the file content (0..filesz)
  - Record BSS pages (filesz..memsz) as a lazy HeapMapping
  - They'll be zero-filled on first access

#### 1f. Lazy stack growth

`src/kernel/elf.rs` — in `spawn_from_elf_internal()`:
- Allocate only 4 pages (16KB) initially instead of 64 (256KB)
- Record full stack region as lazy HeapMapping
- Map the initial 4 pages eagerly (task needs some stack immediately)
- Guard page stays unmapped (hard fault on overflow, not lazy)

#### 1g. Cleanup for lazy mappings

`src/kernel/syscall/memory.rs` — `do_munmap()` and task exit:
- For lazy mappings: walk page table entries in the VA range
- For each valid PTE: extract physical address, free the page
- For eager mappings: unchanged (free contiguous phys_addr block)

### Files Changed
| File | Change |
|------|--------|
| `src/kernel/memory.rs` | Add lazy/writable/executable to HeapMapping |
| `src/kernel/demand_page.rs` | **NEW** — try_demand_page(), VMA lookup |
| `src/kernel/mod.rs` | Add `pub mod demand_page` |
| `src/main.rs` | Add fault recovery check at top of exception_from_user_rust |
| `src/kernel/syscall/memory.rs` | Lazy mmap for OwnedAnon, lazy-aware munmap |
| `src/kernel/elf.rs` | Lazy BSS, lazy stack (4 pages initial) |

---

## Feature 2: ASLR

### Problem
All user binaries load at fixed 0x40010000, stack at 0x80000000, heap at 0x50000000.
Predictable layout makes exploit development trivial.

### Design

**Three independent randomizations per spawn**:
- Code base: +[0..255] × 64KB = 16MB range (8 bits entropy)
- Stack top: −[0..255] × 4KB = 1MB range (8 bits entropy)
- Heap start: +[0..15] × 1MB = 16MB range (4 bits entropy)

**PIE compilation**: User binaries compiled as position-independent (ET_DYN).
Kernel ELF loader applies random base offset and processes R_AARCH64_RELATIVE relocations.

### Implementation

#### 2a. Kernel RNG

New `src/kernel/rng.rs`:
- xorshift64 seeded from CNTPCT_EL0 at boot
- Atomic CAS loop for SMP safety (lock-free)
- `next_u64()` and `next_bounded(n)` functions
- Not cryptographic — sufficient for ASLR entropy

#### 2b. PIE compilation

`user/userlib/user.ld`:
- Remove fixed `USER_BASE = 0x40010000`
- Link at base 0 (standard PIE)
- Add `.dynamic`, `.rela.dyn`, `.got` output sections

`xtask/src/main.rs` + `user/.cargo/config.toml`:
- Add `-Crelocation-model=pie` and `-Clink-arg=-pie` to rustflags

#### 2c. ELF loader PIE support

`src/kernel/elf.rs`:
- For ET_DYN binaries: compute random load_base, add to all p_vaddr
- Entry point = load_base + e_entry
- Parse PT_DYNAMIC → find DT_RELA/DT_RELASZ
- Apply R_AARCH64_RELATIVE: `*(load_base + r_offset) = load_base + r_addend`
- Write relocated values via TTBR1 kernel mapping (same pattern as segment copying)

#### 2d. Stack and heap randomization

In `spawn_from_elf_internal()`:
- Compute AslrOffsets (code, stack, heap)
- Stack top = 0x80000000 - stack_offset
- Heap start = 0x50000000 + heap_offset (stored in task via set_heap_start)
- Pass code offset to load_elf()

### Files Changed
| File | Change |
|------|--------|
| `src/kernel/rng.rs` | **NEW** — xorshift64 PRNG |
| `src/kernel/mod.rs` | Add `pub mod rng` |
| `src/main.rs` | Call rng::init() at boot |
| `src/kernel/elf.rs` | PIE loading, relocation processing, ASLR offsets |
| `src/kernel/task/tcb.rs` | Add set_heap_start() |
| `user/userlib/user.ld` | PIE linker script |
| `xtask/src/main.rs` | PIE rustflags |
| `user/.cargo/config.toml` | PIE rustflags |

---

## Feature 3: Per-CPU Idle States (PSCI CPU_SUSPEND)

### Problem
Idle CPUs only use WFI (lightest possible idle). PSCI CPU_SUSPEND can enter
deeper retention states with more clock gating / power savings.

### Design

**Standby mode only** (StateType=0): CPU state preserved, wakes on any interrupt,
returns to caller. Essentially a "deeper WFI" — no context save/restore needed.

The 10ms periodic timer means we never idle longer than 10ms. Standby is still
beneficial on real hardware (MT7988A) where the ATF gates more clocks.
Tickless idle (skip timer ticks when idle) is a separate future optimization.

### Implementation

#### 3a. Add cpu_suspend_standby() to SMP module

`src/arch/aarch64/smp.rs`:
```rust
/// Enter CPU standby via PSCI CPU_SUSPEND.
/// State preserved, returns on any interrupt (like deep WFI).
pub fn cpu_suspend_standby() {
    // power_state=0: StateType=Standby, Level=Core, StateID=0
    unsafe { psci_call(psci::CPU_SUSPEND, 0, 0, 0); }
}
```

#### 3b. Update idle loop

`src/kernel/idle.rs`:
```rust
loop {
    cpu_impl.enable_irq();
    cpu_local().set_idle();

    // Use PSCI standby for deeper power savings on real hardware.
    // On QEMU this is equivalent to WFI.
    smp::cpu_suspend_standby();

    cpu_local().clear_idle();
}
```

Simple replacement: always use CPU_SUSPEND standby instead of WFI. The PSCI
implementation falls back to WFI if standby isn't supported, so this is safe.

**Alternative**: Keep WFI as fast path, use CPU_SUSPEND only when predicted idle
duration > threshold. But without tickless, the threshold is always ≤10ms, so
the distinction is minimal. Keep it simple for now.

### Files Changed
| File | Change |
|------|--------|
| `src/arch/aarch64/smp.rs` | Add `cpu_suspend_standby()` public function |
| `src/kernel/idle.rs` | Replace `cpu_impl.idle()` with `smp::cpu_suspend_standby()` |

---

## Implementation Order

1. **Feature 3 (Idle states)** — 2 files, smallest change, independent
2. **Feature 1 (Demand paging)** — 6 files, kernel-only, no build system changes
3. **Feature 2 (ASLR)** — 8 files, needs PIE rebuild of all user binaries

Feature 2 last because PIE is a big build system change that touches all binaries.
Feature 1 and 3 are independent and could be done in parallel.
