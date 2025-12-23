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
//! static DATA: SpinLock<MyData> = SpinLock::new(MyData::new());
//!
//! fn access_data() {
//!     let mut guard = DATA.lock();
//!     guard.field = value;
//! } // IRQs restored, lock released on drop
//! ```

use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

/// A spinlock that protects data with IRQ-safe semantics.
///
/// When locked:
/// 1. IRQs are disabled (prevents deadlock from interrupt handler)
/// 2. Spinlock is acquired (for SMP safety)
///
/// When guard is dropped:
/// 1. Spinlock is released
/// 2. IRQ state is restored
///
/// # Safety
///
/// This lock is safe to use from both process and interrupt context,
/// but code holding the lock MUST NOT block or sleep.
pub struct SpinLock<T> {
    locked: AtomicBool,
    data: UnsafeCell<T>,
    #[cfg(debug_assertions)]
    owner_cpu: AtomicU32,
}

// SAFETY: SpinLock<T> can be shared between threads/CPUs if T is Send
unsafe impl<T: Send> Sync for SpinLock<T> {}
unsafe impl<T: Send> Send for SpinLock<T> {}

impl<T> SpinLock<T> {
    /// Create a new spinlock protecting the given data.
    pub const fn new(data: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            data: UnsafeCell::new(data),
            #[cfg(debug_assertions)]
            owner_cpu: AtomicU32::new(u32::MAX),
        }
    }

    /// Acquire the lock, disabling IRQs first.
    ///
    /// Returns a guard that will release the lock and restore IRQs on drop.
    ///
    /// # Panics
    ///
    /// In debug builds, panics if the same CPU tries to acquire a lock it already holds.
    #[inline]
    pub fn lock(&self) -> SpinLockGuard<'_, T> {
        // Step 1: Disable IRQs and save state
        let irqs_were_enabled = disable_irqs_save();

        // Step 2: Acquire spinlock
        #[cfg(debug_assertions)]
        let cpu = cpu_id();

        while self.locked.swap(true, Ordering::Acquire) {
            // Spin until we acquire the lock
            // On single-core, this should never spin (if lock is held, we have a bug)
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
        self.owner_cpu.store(cpu, Ordering::Relaxed);

        SpinLockGuard {
            lock: self,
            irqs_were_enabled,
        }
    }

    /// Try to acquire the lock without spinning.
    ///
    /// Returns `Some(guard)` if lock was acquired, `None` if already held.
    #[inline]
    pub fn try_lock(&self) -> Option<SpinLockGuard<'_, T>> {
        // Step 1: Disable IRQs and save state
        let irqs_were_enabled = disable_irqs_save();

        // Step 2: Try to acquire lock
        if self.locked.swap(true, Ordering::Acquire) {
            // Lock was already held - restore IRQs and return None
            restore_irqs(irqs_were_enabled);
            return None;
        }

        #[cfg(debug_assertions)]
        self.owner_cpu.store(cpu_id(), Ordering::Relaxed);

        Some(SpinLockGuard {
            lock: self,
            irqs_were_enabled,
        })
    }

    /// Check if the lock is currently held.
    ///
    /// Note: This is racy - the lock may be acquired/released between the check
    /// and any action you take based on it.
    #[inline]
    pub fn is_locked(&self) -> bool {
        self.locked.load(Ordering::Relaxed)
    }

    /// Get a reference to the underlying data without locking.
    ///
    /// # Safety
    ///
    /// The caller must ensure exclusive access to the data.
    /// This is only safe during initialization before the lock is used.
    #[inline]
    pub unsafe fn get_unchecked(&self) -> &T {
        &*self.data.get()
    }

    /// Get a mutable reference to the underlying data without locking.
    ///
    /// # Safety
    ///
    /// The caller must ensure exclusive access to the data.
    /// This is only safe during initialization before the lock is used.
    #[inline]
    pub unsafe fn get_unchecked_mut(&self) -> &mut T {
        &mut *self.data.get()
    }
}

/// Guard that releases a SpinLock when dropped.
///
/// Provides access to the protected data via Deref/DerefMut.
pub struct SpinLockGuard<'a, T> {
    lock: &'a SpinLock<T>,
    irqs_were_enabled: bool,
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
        // Step 1: Clear owner (debug only)
        #[cfg(debug_assertions)]
        self.lock.owner_cpu.store(u32::MAX, Ordering::Relaxed);

        // Step 2: Release spinlock
        self.lock.locked.store(false, Ordering::Release);

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
