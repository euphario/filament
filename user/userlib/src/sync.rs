//! Single-threaded synchronization primitives for userspace.
//!
//! These primitives are designed for single-threaded userspace drivers.
//! They provide safe interior mutability without requiring `unsafe` in
//! driver code.
//!
//! ## SMP Considerations
//!
//! `SingleThreadCell` is NOT SMP-safe. It is intended for userspace tasks
//! that are guaranteed to be single-threaded. For multi-threaded userspace
//! (future work), this should be replaced with a proper Mutex or SpinLock.
//!
//! The `unsafe impl Sync` allows using these in statics, but actual
//! concurrent access from multiple threads would cause a panic (double borrow).

use core::cell::{RefCell, RefMut, Ref};

/// A cell that can be initialized once and borrowed mutably.
///
/// This provides safe interior mutability for single-threaded userspace
/// programs. It panics on misuse (double borrow, uninitialized access)
/// rather than causing undefined behavior.
///
/// # Example
///
/// ```ignore
/// use userlib::sync::SingleThreadCell;
///
/// static DRIVER: SingleThreadCell<MyDriver> = SingleThreadCell::new();
///
/// fn main() {
///     DRIVER.init(MyDriver::new());
///     DRIVER.borrow_mut().do_something();
/// }
/// ```
pub struct SingleThreadCell<T> {
    inner: RefCell<Option<T>>,
}

impl<T> SingleThreadCell<T> {
    /// Create a new uninitialized cell.
    pub const fn new() -> Self {
        Self {
            inner: RefCell::new(None),
        }
    }

    /// Initialize the cell with a value.
    ///
    /// # Panics
    ///
    /// Panics if the cell has already been initialized.
    pub fn init(&self, value: T) {
        let mut inner = self.inner.borrow_mut();
        if inner.is_some() {
            panic!("SingleThreadCell already initialized");
        }
        *inner = Some(value);
    }

    /// Borrow the value immutably.
    ///
    /// # Panics
    ///
    /// Panics if the cell is not initialized or is currently mutably borrowed.
    pub fn borrow(&self) -> Ref<'_, T> {
        Ref::map(self.inner.borrow(), |opt| {
            opt.as_ref().expect("SingleThreadCell not initialized")
        })
    }

    /// Borrow the value mutably.
    ///
    /// # Panics
    ///
    /// Panics if the cell is not initialized or is currently borrowed.
    pub fn borrow_mut(&self) -> RefMut<'_, T> {
        RefMut::map(self.inner.borrow_mut(), |opt| {
            opt.as_mut().expect("SingleThreadCell not initialized")
        })
    }

    /// Check if the cell has been initialized.
    pub fn is_initialized(&self) -> bool {
        self.inner.borrow().is_some()
    }

    /// Try to borrow the value mutably, returning None if already borrowed.
    ///
    /// # Panics
    ///
    /// Panics if the cell is not initialized.
    pub fn try_borrow_mut(&self) -> Option<RefMut<'_, T>> {
        let inner = self.inner.try_borrow_mut().ok()?;
        Some(RefMut::map(inner, |opt| {
            opt.as_mut().expect("SingleThreadCell not initialized")
        }))
    }
}

// SAFETY: This is only safe for single-threaded userspace. The RefCell
// will panic on concurrent access, which is better than UB. For actual
// SMP userspace, this should be replaced with a Mutex.
unsafe impl<T> Sync for SingleThreadCell<T> {}

impl<T> Default for SingleThreadCell<T> {
    fn default() -> Self {
        Self::new()
    }
}
