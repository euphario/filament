//! Locking Primitives
//!
//! Provides SpinLock with IRQ-safe semantics for kernel data structures.
//!
//! ## Design
//!
//! All locks in this module:
//! - Disable IRQs on lock acquisition (irqsave pattern)
//! - Restore IRQ state on drop
//! - Are SMP-safe (use atomic spinlock)
//!
//! ## Usage
//!
//! ```rust
//! static DATA: SpinLock<MyData> = SpinLock::new(lock_class::RESOURCE, MyData::new());
//!
//! fn access_data() {
//!     let mut guard = DATA.lock();
//!     guard.field = value;
//! } // IRQs restored, lock released on drop
//! ```

use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicU32, Ordering};

// ============================================================================
// Lock Ordering Classes
// ============================================================================

/// Lock ordering classes for deadlock prevention.
///
/// Locks must be acquired in strictly increasing class order.
/// Gaps between values allow inserting new classes without renumbering.
///
/// ## Ordering (outermost → innermost)
///
/// ```text
/// SCHEDULER(10) → OBJ_SERVICE(20) → SUBSYSTEM(30) → RESOURCE(40)
/// ```
pub mod lock_class {
    /// Skip ordering checks (test locks, one-off locks)
    pub const UNORDERED: u8 = 0;
    /// Scheduler lock — acquired first (outermost)
    pub const SCHEDULER: u8 = 10;
    /// Microtask queue — between scheduler and ObjectService
    pub const MICROTASK: u8 = 15;
    /// ObjectService per-task table locks
    pub const OBJ_SERVICE: u8 = 20;
    /// Bus registry — between OBJ_SERVICE and SUBSYSTEM (acquires IPC locks)
    pub const BUS: u8 = 25;
    /// Subsystem locks: CHANNEL_TABLE, PORT_REGISTRY, IRQ_TABLE, LOG_RING
    pub const SUBSYSTEM: u8 = 30;
    /// Resource locks: DMA_POOL, SHMEM, PMM, ASID_ALLOCATOR, VM_OBJECTS
    pub const RESOURCE: u8 = 40;
}

// ============================================================================
// Debug Statistics (for diagnosing lock contention + ordering enforcement)
// ============================================================================

#[cfg(debug_assertions)]
mod stats {
    use super::*;

    /// Maximum allowed lock nesting depth
    pub const MAX_LOCK_DEPTH: usize = 8;

    /// Per-CPU lock nesting depth
    static LOCK_DEPTH: [AtomicU32; 4] = [
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
    ];

    /// Per-CPU stack of held lock classes (for ordering enforcement)
    /// Each CPU tracks which lock classes it currently holds.
    static HELD_CLASSES: [[AtomicU32; MAX_LOCK_DEPTH]; 4] = [
        [const { AtomicU32::new(0) }; MAX_LOCK_DEPTH],
        [const { AtomicU32::new(0) }; MAX_LOCK_DEPTH],
        [const { AtomicU32::new(0) }; MAX_LOCK_DEPTH],
        [const { AtomicU32::new(0) }; MAX_LOCK_DEPTH],
    ];

    /// Push lock class onto per-CPU stack, enforce ordering.
    ///
    /// Panics if:
    /// - Nesting depth exceeds MAX_LOCK_DEPTH
    /// - Class is <= the top of the stack (ordering violation), unless class == 0
    pub fn increment_depth(cpu: u32, class: u8) {
        let cpu_idx = cpu as usize;
        if cpu_idx >= LOCK_DEPTH.len() {
            return;
        }
        let depth = LOCK_DEPTH[cpu_idx].fetch_add(1, Ordering::Relaxed) as usize;
        if depth >= MAX_LOCK_DEPTH {
            panic!(
                "Lock nesting too deep ({} >= {}) on CPU {} - possible deadlock",
                depth + 1, MAX_LOCK_DEPTH, cpu
            );
        }

        // Enforce ordering: new class must be > top of stack (unless UNORDERED)
        if class != lock_class::UNORDERED && depth > 0 {
            let top = HELD_CLASSES[cpu_idx][depth - 1].load(Ordering::Relaxed) as u8;
            if top != lock_class::UNORDERED && class <= top {
                panic!(
                    "Lock ordering violation on CPU {}: acquiring class {} while holding class {} (depth {})",
                    cpu, class, top, depth
                );
            }
        }

        // Push class onto stack
        HELD_CLASSES[cpu_idx][depth].store(class as u32, Ordering::Relaxed);
    }

    /// Pop lock class from per-CPU stack, verify it matches.
    pub fn decrement_depth(cpu: u32, class: u8) {
        let cpu_idx = cpu as usize;
        if cpu_idx >= LOCK_DEPTH.len() {
            return;
        }
        let old_depth = LOCK_DEPTH[cpu_idx].fetch_sub(1, Ordering::Relaxed);
        if old_depth == 0 {
            panic!("Lock depth underflow on CPU {} - mismatched lock/unlock", cpu);
        }
        let depth = (old_depth - 1) as usize;

        // Verify LIFO: the class being released must match the top of stack
        let top = HELD_CLASSES[cpu_idx][depth].load(Ordering::Relaxed) as u8;
        if top != class {
            panic!(
                "Lock release order violation on CPU {}: releasing class {} but top is class {} (depth {})",
                cpu, class, top, depth
            );
        }

        // Clear the stack slot
        HELD_CLASSES[cpu_idx][depth].store(0, Ordering::Relaxed);
    }

    /// Get current lock depth for a CPU
    pub fn get_depth(cpu: u32) -> u32 {
        if (cpu as usize) < LOCK_DEPTH.len() {
            LOCK_DEPTH[cpu as usize].load(Ordering::Relaxed)
        } else {
            0
        }
    }
}

/// A ticket spinlock that protects data with IRQ-safe semantics.
///
/// Uses ticket algorithm for FIFO fairness under SMP contention:
/// each acquirer takes a ticket and waits until their number is served.
///
/// When locked:
/// 1. IRQs are disabled (prevents deadlock from interrupt handler)
/// 2. Spinlock is acquired (for SMP safety)
///
/// When guard is dropped:
/// 1. Spinlock is released (now_serving incremented)
/// 2. IRQ state is restored
///
/// # Safety
///
/// This lock is safe to use from both process and interrupt context,
/// but code holding the lock MUST NOT block or sleep.
pub struct SpinLock<T> {
    next_ticket: AtomicU32,
    now_serving: AtomicU32,
    data: UnsafeCell<T>,
    /// Lock ordering class (always present, zero-cost in release — only checked in debug)
    lock_class: u8,
    #[cfg(debug_assertions)]
    owner_cpu: AtomicU32,
}

// SAFETY: SpinLock<T> can be shared between threads/CPUs if T is Send
unsafe impl<T: Send> Sync for SpinLock<T> {}
unsafe impl<T: Send> Send for SpinLock<T> {}

impl<T> SpinLock<T> {
    /// Create a new spinlock protecting the given data.
    ///
    /// `class` is the lock ordering class (see `lock_class` module).
    /// In debug builds, acquiring locks out of order panics.
    /// Use `lock_class::UNORDERED` (0) to skip ordering checks.
    pub const fn new(class: u8, data: T) -> Self {
        Self {
            next_ticket: AtomicU32::new(0),
            now_serving: AtomicU32::new(0),
            data: UnsafeCell::new(data),
            lock_class: class,
            #[cfg(debug_assertions)]
            owner_cpu: AtomicU32::new(u32::MAX),
        }
    }

    /// Acquire the lock, disabling IRQs first.
    ///
    /// Returns a guard that will release the lock and restore IRQs on drop.
    /// Uses ticket algorithm for FIFO fairness.
    ///
    /// # Panics
    ///
    /// In debug builds, panics if the same CPU tries to acquire a lock it already holds.
    #[inline]
    pub fn lock(&self) -> SpinLockGuard<'_, T> {
        // Step 1: Disable IRQs and save state
        let irqs_were_enabled = disable_irqs_save();

        // Step 2: Take a ticket and wait for our turn
        #[cfg(debug_assertions)]
        let cpu = cpu_id();

        let ticket = self.next_ticket.fetch_add(1, Ordering::Relaxed);
        while self.now_serving.load(Ordering::Acquire) != ticket {
            #[cfg(debug_assertions)]
            {
                // Check for deadlock (same CPU already holds lock)
                if self.owner_cpu.load(Ordering::Relaxed) == cpu {
                    panic!("SpinLock deadlock: CPU {} already holds this lock", cpu);
                }
            }
            core::hint::spin_loop();
        }

        #[cfg(debug_assertions)]
        {
            self.owner_cpu.store(cpu, Ordering::Relaxed);
            stats::increment_depth(cpu, self.lock_class);
        }

        SpinLockGuard {
            lock: self,
            irqs_were_enabled,
            #[cfg(debug_assertions)]
            owner_cpu: cpu,
            #[cfg(debug_assertions)]
            lock_class: self.lock_class,
        }
    }

    /// Try to acquire the lock without spinning.
    ///
    /// Returns `Some(guard)` if lock was acquired, `None` if already held.
    #[inline]
    pub fn try_lock(&self) -> Option<SpinLockGuard<'_, T>> {
        // Step 1: Disable IRQs and save state
        let irqs_were_enabled = disable_irqs_save();

        // Step 2: Try to acquire lock (only succeeds if no one is waiting)
        let current = self.now_serving.load(Ordering::Relaxed);
        if self.next_ticket.compare_exchange(
            current, current.wrapping_add(1),
            Ordering::Acquire, Ordering::Relaxed
        ).is_err() {
            // Lock is held or someone is waiting - restore IRQs and return None
            restore_irqs(irqs_were_enabled);
            return None;
        }

        #[cfg(debug_assertions)]
        let cpu = cpu_id();
        #[cfg(debug_assertions)]
        {
            self.owner_cpu.store(cpu, Ordering::Relaxed);
            stats::increment_depth(cpu, self.lock_class);
        }

        Some(SpinLockGuard {
            lock: self,
            irqs_were_enabled,
            #[cfg(debug_assertions)]
            owner_cpu: cpu,
            #[cfg(debug_assertions)]
            lock_class: self.lock_class,
        })
    }

    /// Check if the lock is currently held.
    ///
    /// Note: This is racy - the lock may be acquired/released between the check
    /// and any action you take based on it.
    #[inline]
    pub fn is_locked(&self) -> bool {
        let next = self.next_ticket.load(Ordering::Relaxed);
        let serving = self.now_serving.load(Ordering::Relaxed);
        next != serving
    }

    /// Get a reference to the underlying data without locking.
    ///
    /// # Safety
    ///
    /// The caller must ensure exclusive access to the data.
    /// This is only safe during initialization before the lock is used.
    ///
    /// # Visibility
    ///
    /// `pub(crate)` to limit unsafe access to kernel internals only.
    #[inline]
    pub(crate) unsafe fn get_unchecked(&self) -> &T {
        &*self.data.get()
    }

    /// Get a mutable reference to the underlying data without locking.
    ///
    /// # Safety
    ///
    /// The caller must ensure exclusive access to the data.
    /// This is only safe during initialization before the lock is used.
    ///
    /// # Visibility
    ///
    /// `pub(crate)` to limit unsafe access to kernel internals only.
    #[inline]
    pub(crate) unsafe fn get_unchecked_mut(&self) -> &mut T {
        &mut *self.data.get()
    }
}

/// Guard that releases a SpinLock when dropped.
///
/// Provides access to the protected data via Deref/DerefMut.
pub struct SpinLockGuard<'a, T> {
    lock: &'a SpinLock<T>,
    irqs_were_enabled: bool,
    #[cfg(debug_assertions)]
    owner_cpu: u32,
    #[cfg(debug_assertions)]
    lock_class: u8,
}

impl<'a, T> Deref for SpinLockGuard<'a, T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &T {
        // SAFETY: Guard existence proves we hold the lock
        unsafe { &*self.lock.data.get() }
    }
}

impl<'a, T> DerefMut for SpinLockGuard<'a, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: Guard existence proves we hold the lock exclusively
        unsafe { &mut *self.lock.data.get() }
    }
}

impl<'a, T> Drop for SpinLockGuard<'a, T> {
    #[inline]
    fn drop(&mut self) {
        // Step 1: Update debug stats and clear owner
        #[cfg(debug_assertions)]
        {
            stats::decrement_depth(self.owner_cpu, self.lock_class);
            self.lock.owner_cpu.store(u32::MAX, Ordering::Relaxed);
        }

        // Step 2: Release spinlock — advance now_serving to next ticket
        self.lock.now_serving.fetch_add(1, Ordering::Release);

        // Step 3: Restore IRQ state
        restore_irqs(self.irqs_were_enabled);
    }
}

/// Disable IRQs and return whether they were enabled.
#[inline]
fn disable_irqs_save() -> bool {
    let daif: u64;
    unsafe {
        core::arch::asm!("mrs {}, daif", out(reg) daif);
        core::arch::asm!("msr daifset, #2"); // Set I bit (disable IRQs)
    }
    // I bit is bit 7 - if clear, IRQs were enabled
    (daif & (1 << 7)) == 0
}

/// Restore IRQ state.
#[inline]
fn restore_irqs(were_enabled: bool) {
    if were_enabled {
        unsafe {
            core::arch::asm!("msr daifclr, #2"); // Clear I bit (enable IRQs)
        }
    }
}

/// Get current CPU ID (for debug deadlock detection).
#[inline]
#[cfg(debug_assertions)]
fn cpu_id() -> u32 {
    let mpidr: u64;
    unsafe {
        core::arch::asm!("mrs {}, mpidr_el1", out(reg) mpidr);
    }
    // Extract Aff0 (CPU ID within cluster)
    (mpidr & 0xFF) as u32
}

/// Get the current lock nesting depth for this CPU (debug builds only).
///
/// Returns 0 if no locks are held, or the number of nested locks.
/// This is useful for debugging and ensuring locks are properly released.
#[cfg(debug_assertions)]
pub fn current_lock_depth() -> u32 {
    stats::get_depth(cpu_id())
}

/// Get the current lock nesting depth for this CPU.
/// Always returns 0 in release builds.
#[cfg(not(debug_assertions))]
pub fn current_lock_depth() -> u32 {
    0
}

/// Assert that no locks are currently held on this CPU.
///
/// Use this before blocking operations or at context switch points.
/// Panics in debug builds if any locks are held.
#[cfg(debug_assertions)]
pub fn assert_no_locks_held() {
    let depth = current_lock_depth();
    if depth > 0 {
        panic!(
            "assert_no_locks_held failed: {} lock(s) still held on CPU {}",
            depth, cpu_id()
        );
    }
}

/// Assert that no locks are currently held (no-op in release builds).
#[cfg(not(debug_assertions))]
#[inline]
pub fn assert_no_locks_held() {}

// ============================================================================
// Read-Write Lock (for future use)
// ============================================================================

/// A read-write lock for data that is read often but written rarely.
///
/// Multiple readers can hold the lock simultaneously, but writers get
/// exclusive access. Uses IRQ-save semantics like SpinLock.
///
/// # Note
///
/// This is a simple implementation that may have writer starvation.
/// For high-contention scenarios, consider a more sophisticated approach.
pub struct RwLock<T> {
    /// Number of readers (0 = unlocked, -1 = write-locked, >0 = read count)
    state: AtomicI32,
    data: UnsafeCell<T>,
}

use core::sync::atomic::AtomicI32;

unsafe impl<T: Send> Sync for RwLock<T> {}
unsafe impl<T: Send> Send for RwLock<T> {}

impl<T> RwLock<T> {
    /// Create a new read-write lock.
    pub const fn new(data: T) -> Self {
        Self {
            state: AtomicI32::new(0),
            data: UnsafeCell::new(data),
        }
    }

    /// Acquire a read lock.
    #[inline]
    pub fn read(&self) -> RwLockReadGuard<'_, T> {
        let irqs_were_enabled = disable_irqs_save();

        loop {
            let state = self.state.load(Ordering::Acquire);
            if state >= 0 {
                // Not write-locked, try to increment reader count
                if self.state.compare_exchange_weak(
                    state, state + 1,
                    Ordering::AcqRel, Ordering::Relaxed
                ).is_ok() {
                    return RwLockReadGuard { lock: self, irqs_were_enabled };
                }
            }
            core::hint::spin_loop();
        }
    }

    /// Acquire a write lock.
    #[inline]
    pub fn write(&self) -> RwLockWriteGuard<'_, T> {
        let irqs_were_enabled = disable_irqs_save();

        // Try to set state to -1 (exclusive)
        while self.state.compare_exchange_weak(
            0, -1,
            Ordering::AcqRel, Ordering::Relaxed
        ).is_err() {
            core::hint::spin_loop();
        }

        RwLockWriteGuard { lock: self, irqs_were_enabled }
    }
}

/// Guard for read access to RwLock data.
pub struct RwLockReadGuard<'a, T> {
    lock: &'a RwLock<T>,
    irqs_were_enabled: bool,
}

impl<'a, T> Deref for RwLockReadGuard<'a, T> {
    type Target = T;
    fn deref(&self) -> &T {
        unsafe { &*self.lock.data.get() }
    }
}

impl<'a, T> Drop for RwLockReadGuard<'a, T> {
    fn drop(&mut self) {
        self.lock.state.fetch_sub(1, Ordering::Release);
        restore_irqs(self.irqs_were_enabled);
    }
}

/// Guard for write access to RwLock data.
pub struct RwLockWriteGuard<'a, T> {
    lock: &'a RwLock<T>,
    irqs_were_enabled: bool,
}

impl<'a, T> Deref for RwLockWriteGuard<'a, T> {
    type Target = T;
    fn deref(&self) -> &T {
        unsafe { &*self.lock.data.get() }
    }
}

impl<'a, T> DerefMut for RwLockWriteGuard<'a, T> {
    fn deref_mut(&mut self) -> &mut T {
        unsafe { &mut *self.lock.data.get() }
    }
}

impl<'a, T> Drop for RwLockWriteGuard<'a, T> {
    fn drop(&mut self) {
        self.lock.state.store(0, Ordering::Release);
        restore_irqs(self.irqs_were_enabled);
    }
}

// ============================================================================
// Self-tests
// ============================================================================

#[cfg(feature = "selftest")]
pub fn test() {
    use crate::{kdebug, kinfo};

    kdebug!("lock", "test_start");

    // Test 1: Basic SpinLock acquire/release
    {
        static TEST_LOCK: SpinLock<u32> = SpinLock::new(0, 42);

        let guard = TEST_LOCK.lock();
        assert_eq!(*guard, 42, "SpinLock initial value wrong");
        assert!(TEST_LOCK.is_locked(), "Lock should be held");
        drop(guard);
        assert!(!TEST_LOCK.is_locked(), "Lock should be released");
        kdebug!("lock", "spinlock_basic_ok");
    }

    // Test 2: SpinLock mutation
    {
        static COUNTER: SpinLock<u32> = SpinLock::new(0, 0);

        {
            let mut guard = COUNTER.lock();
            *guard += 1;
        }
        {
            let mut guard = COUNTER.lock();
            *guard += 1;
        }
        {
            let guard = COUNTER.lock();
            assert_eq!(*guard, 2, "Counter should be 2");
        }
        kdebug!("lock", "spinlock_mutation_ok");
    }

    // Test 3: try_lock succeeds when unlocked
    {
        static TRY_LOCK: SpinLock<u32> = SpinLock::new(0, 100);

        let result = TRY_LOCK.try_lock();
        assert!(result.is_some(), "try_lock should succeed when unlocked");
        drop(result);
        kdebug!("lock", "try_lock_success_ok");
    }

    // Test 4: IRQ state preservation
    // This tests that IRQs are properly saved/restored
    {
        static IRQ_TEST: SpinLock<u32> = SpinLock::new(0, 0);

        // Get IRQ state before
        let daif_before: u64;
        unsafe { core::arch::asm!("mrs {}, daif", out(reg) daif_before); }
        let irqs_enabled_before = (daif_before & (1 << 7)) == 0;

        {
            let _guard = IRQ_TEST.lock();
            // IRQs should be disabled while holding lock
            let daif_during: u64;
            unsafe { core::arch::asm!("mrs {}, daif", out(reg) daif_during); }
            assert!((daif_during & (1 << 7)) != 0, "IRQs should be disabled during lock");
        }

        // IRQ state should be restored
        let daif_after: u64;
        unsafe { core::arch::asm!("mrs {}, daif", out(reg) daif_after); }
        let irqs_enabled_after = (daif_after & (1 << 7)) == 0;
        assert_eq!(irqs_enabled_before, irqs_enabled_after, "IRQ state not restored");
        kdebug!("lock", "irq_preserve_ok");
    }

    // Test 5: RwLock read access
    {
        static RW_TEST: RwLock<u32> = RwLock::new(123);

        let guard = RW_TEST.read();
        assert_eq!(*guard, 123, "RwLock read value wrong");
        drop(guard);
        kdebug!("lock", "rwlock_read_ok");
    }

    // Test 6: RwLock write access
    {
        static RW_WRITE: RwLock<u32> = RwLock::new(0);

        {
            let mut guard = RW_WRITE.write();
            *guard = 456;
        }
        {
            let guard = RW_WRITE.read();
            assert_eq!(*guard, 456, "RwLock write value not persisted");
        }
        kdebug!("lock", "rwlock_write_ok");
    }

    // Test 7: Multiple readers allowed
    {
        static MULTI_READ: RwLock<u32> = RwLock::new(789);

        let r1 = MULTI_READ.read();
        // Note: We can't easily test simultaneous readers on single-core,
        // but we verify the state tracking works
        assert_eq!(MULTI_READ.state.load(Ordering::Relaxed), 1, "Should have 1 reader");
        drop(r1);
        assert_eq!(MULTI_READ.state.load(Ordering::Relaxed), 0, "Should have 0 readers");
        kdebug!("lock", "rwlock_multi_ok");
    }

    // Test 8: Lock depth tracking (debug builds only)
    #[cfg(debug_assertions)]
    {
        static DEPTH_TEST: SpinLock<u32> = SpinLock::new(0, 0);

        let depth_before = current_lock_depth();
        {
            let _g = DEPTH_TEST.lock();
            let depth_during = current_lock_depth();
            assert_eq!(depth_during, depth_before + 1, "Lock depth should increment");
        }
        let depth_after = current_lock_depth();
        assert_eq!(depth_after, depth_before, "Lock depth should return to original");
        kdebug!("lock", "depth_tracking_ok");
    }

    kdebug!("lock", "test_ok");
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spinlock_new() {
        let lock: SpinLock<u32> = SpinLock::new(0, 42);
        assert_eq!(*lock.lock(), 42);
    }

    #[test]
    fn test_spinlock_lock_unlock() {
        let lock = SpinLock::new(0, 0u32);
        {
            let mut guard = lock.lock();
            *guard = 100;
        }
        assert_eq!(*lock.lock(), 100);
    }

    #[test]
    fn test_spinlock_try_lock() {
        let lock = SpinLock::new(0, 0u32);

        let guard = lock.try_lock();
        assert!(guard.is_some());
        drop(guard);
    }

    #[test]
    fn test_spinlock_multiple_access() {
        let lock = SpinLock::new(0, 0u32);

        {
            let mut g = lock.lock();
            *g += 1;
        }
        {
            let mut g = lock.lock();
            *g += 1;
        }
        assert_eq!(*lock.lock(), 2);
    }

    #[test]
    fn test_rwlock_new() {
        let lock: RwLock<u32> = RwLock::new(123);
        assert_eq!(*lock.read(), 123);
    }

    #[test]
    fn test_rwlock_read() {
        let lock = RwLock::new(456u32);
        let guard = lock.read();
        assert_eq!(*guard, 456);
    }

    #[test]
    fn test_rwlock_write() {
        let lock = RwLock::new(0u32);
        {
            let mut guard = lock.write();
            *guard = 789;
        }
        assert_eq!(*lock.read(), 789);
    }

    #[test]
    fn test_rwlock_try_read() {
        let lock = RwLock::new(100u32);
        let guard = lock.try_read();
        assert!(guard.is_some());
    }

    #[test]
    fn test_rwlock_try_write() {
        let lock = RwLock::new(200u32);
        let guard = lock.try_write();
        assert!(guard.is_some());
    }

    #[test]
    fn test_spinlock_const_new() {
        static LOCK: SpinLock<u32> = SpinLock::new(0, 999);
        assert_eq!(*LOCK.lock(), 999);
    }

    #[test]
    fn test_rwlock_const_new() {
        static LOCK: RwLock<u32> = RwLock::new(888);
        assert_eq!(*LOCK.read(), 888);
    }
}
