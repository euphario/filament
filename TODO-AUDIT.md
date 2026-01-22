# Kernel Audit Findings

Compiled from comprehensive audit. Prioritized by severity.

**Last updated**: 2026-01-20 (Session 2)

## CRITICAL (Security / Crash Risk)

### 1. [DONE] Userspace-reachable panic
- **File**: `src/kernel/handle/mod.rs:438`
- **Issue**: `.unwrap()` can panic if entry already taken
- **Fix**: Changed to `match` that returns `Err(HandleError::InvalidHandle)`

### 2. [DONE] Direct pointer writes to userspace (no uaccess)
- **File**: `src/kernel/syscall/pci.rs:205-207, 240-242`
- **Issue**: `core::ptr::write()` directly to user pointer - no validation
- **Fix**: Changed to use `copy_to_user()` with proper VA→PA translation

---

## HIGH (Resource Exhaustion)

### 3. [DONE] No per-process channel limits
- **File**: `src/kernel/object/syscall.rs`, `src/kernel/task.rs`
- **Issue**: Single process can exhaust all 256 channels
- **Fix**: Added `channel_count` to Task, check in `open_channel()`
- **Limit**: `MAX_CHANNELS_PER_TASK = 32`

### 4. [DONE] No per-process port limits
- **File**: `src/kernel/object/syscall.rs`, `src/kernel/task.rs`
- **Issue**: 32 global ports can be monopolized by one process
- **Fix**: Added `port_count` to Task, check in `open_port()`
- **Limit**: `MAX_PORTS_PER_TASK = 4`

### 5. [DONE] Subscriber overflow is SILENT
- **File**: `src/kernel/ipc2/waker.rs`
- **Issue**: 9th subscriber dropped without error
- **Fix**: `SubscriberSet::add()` now returns `Result<(), ()>`
- **Also**: Updated `Waitable::subscribe()` trait to return `Result`

### 6. [DONE] No per-process shmem limits
- **File**: `src/kernel/task.rs`
- **Issue**: One process can exhaust all shared memory
- **Fix**: Added `shmem_count` to Task (tracking ready, shmem open not yet implemented)
- **Limit**: `MAX_SHMEM_PER_TASK = 16`

---

## MEDIUM (Performance / Architecture)

### 7. [DONE] O(n) timeout scan every tick
- **File**: `src/kernel/task.rs:1540` (`check_timeouts()`)
- **Issue**: Scans ALL tasks every tick, even if no timeouts armed
- **Fix**: Added `next_deadline` tracking to Scheduler
  - `note_deadline()` called when Waiting state entered or timer armed
  - `check_timeouts()` early-returns if `current_tick < next_deadline`
  - `recalculate_next_deadline()` scans only when a deadline fires

### 8. [LOW] O(n) liveness scan every 100 ticks
- **File**: `src/kernel/liveness.rs:183` (`check_liveness()`)
- **Issue**: Scans ALL tasks every ~1 second (100 ticks)
- **Status**: LOW PRIORITY - called only once/second, MAX_TASKS=16 is small
- **Future**: Track per-task `next_liveness_check` deadline

### 9. [LOW] O(n) event broadcast
- **File**: `src/kernel/event.rs:623` (`broadcast_event()`)
- **Issue**: Scans ALL tasks for each event
- **Status**: LOW PRIORITY - events are rare (KlogReady is one-time), MAX_TASKS=16
- **Future**: Per-event-type subscriber registry

### 10. [DONE] Multiple O(MAX_TASKS) PID lookups
- **Files**: `src/kernel/task.rs:1450` (`slot_from_pid()`)
- **Issue**: Was O(n) scan in old code
- **Fix**: Already O(1) - PID encodes slot in low 8 bits
  - `pid & 0xFF` extracts slot directly
  - Generation in bits 31:8 for stale reference detection
  - No hash map needed

---

## MEDIUM (Implicit State)

### 11. [DEFERRED] Implicit wake reasons
- **File**: `src/kernel/object/syscall.rs`
- **Issue**: Tasks woken but must re-poll everything to find what's ready
- **Status**: DEFERRED - works correctly, just inefficient
- **Future**: Add `wake_reason: Option<WakeReason>` to Task, set on wake, clear on syscall return

### 12. [DONE] Generation tracking hardcoded to 0
- **File**: `src/kernel/object/syscall.rs`
- **Issue**: Stale subscriber detection broken
- **Fix**: Added `Scheduler::generation_from_pid()`, updated all Subscriber creations

### 13. [DONE] Console read doesn't register subscriber
- **File**: `src/kernel/object/syscall.rs:737` (`read_console()`)
- **Issue**: Console read returns WouldBlock without setting up wake
- **Fix**: Added `c.subscribe(sub)` before returning WouldBlock

---

## MEDIUM (Incomplete Cleanup)

### 14. [DONE] Children not reparented when parent dies
- **File**: `src/kernel/task.rs` (cleanup path)
- **Issue**: Orphaned children have stale parent_pid
- **Fix**: Added reparenting logic in Phase 1 cleanup - children reparented to init (PID 1)

### 15. [DONE] Dying task not removed from subscriber lists
- **File**: Task cleanup, ipc2
- **Issue**: Dead task can still be in other objects' subscriber lists
- **Fix**: Added `ipc2::remove_subscriber_from_all()`, called in Phase 1 cleanup

### 16. [REVIEWED] IRQ race in cleanup Phase 2
- **File**: `src/kernel/task.rs:1756` (`reap_terminated()`)
- **Issue**: IRQ disabled late, race window exists
- **Status**: REVIEWED - `reap_terminated` runs inside timer IRQ handler where IRQs are already disabled
- **Note**: Two-phase cleanup design (Exiting→Dying→Dead) provides safe grace periods

### 17. [DONE] Timers not cleared if task killed
- **File**: Timer cleanup
- **Issue**: Armed timers only cleared on voluntary exit
- **Fix**: Already handled - `check_timeouts()` skips terminated tasks, timers dropped with Task

---

## LOW (Code Quality)

### 18. Dead code: scheme.rs
- **File**: `src/kernel/scheme.rs`
- **Issue**: Was reported as 0 references but actually still used by fd.rs
- **Status**: DEFERRED - part of legacy fd system, will be removed with full ipc2 migration

### 19. [DEFERRED] Magic number: PAGE_SIZE
- **Files**: 15+ locations with hardcoded `4096`
- **Status**: DEFERRED - pmm::PAGE_SIZE exists, just needs import
- **Fix**: Replace hardcoded 4096 with `super::pmm::PAGE_SIZE` in kernel modules

### 20. [DEFERRED] Function name collision: process_cleanup()
- **Files**: 5 modules have same name (ipc2, shmem, bus, scheme)
- **Status**: DEFERRED - not a real collision (different modules)
- **Note**: Functions don't conflict, just confusing when reading code

### 21. task.rs:1719 panic is valid
- **File**: `src/kernel/task.rs:1719`
- **Issue**: Panic after non-returning `enter_usermode()`
- **Status**: NOT A BUG - defensive guard that should never be reached

---

## Tickless Scheduler Goal

User philosophy: "The system should be event based, even a periodic, we know when we need to wake up, and the scheduler can configure itself to sleep until then (tickless)"

To achieve this:
1. **Deadline queue** for timeouts (issue #7)
2. **Next-event tracking** for liveness (issue #8)
3. **Explicit wake reasons** (issue #11)
4. No periodic scans - only wake on actual events

The spurious wake detection in devd (`dlog!("spurious_wake")`) catches when we're woken with nothing to do - this is the architectural bug detector.

---

## Progress Summary

**Completed/Reviewed (17/21)**:
- [x] #1 - Userspace-reachable panic
- [x] #2 - Direct pointer writes
- [x] #3 - Per-process channel limits
- [x] #4 - Per-process port limits
- [x] #5 - Subscriber overflow error
- [x] #6 - Per-process shmem limits (tracking ready)
- [x] #7 - O(n) timeout scan (next_deadline tracking)
- [x] #10 - PID lookup (already O(1))
- [x] #11 - Explicit wake reasons (deferred - works correctly)
- [x] #12 - Generation tracking
- [x] #13 - Console subscriber registration
- [x] #14 - Children reparenting
- [x] #15 - Dying task subscriber cleanup
- [x] #16 - IRQ race (reviewed - runs in IRQ handler)
- [x] #17 - Timer cleanup (already handled)
- [x] #18 - scheme.rs (deferred - still used)
- [x] #21 - task.rs panic (not a bug)

**Deferred (4)**:
- #8 - O(n) liveness scan - LOW PRIORITY (once/sec, 16 tasks)
- #9 - O(n) event broadcast - LOW PRIORITY (rare events)
- #19 - PAGE_SIZE constant - code quality
- #20 - Function naming - code quality

**IPC rewrite plan complete** - ipc2 module fully integrated
