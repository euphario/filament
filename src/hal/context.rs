//! Context Switch HAL
//!
//! Platform-agnostic context switching for kernel-to-kernel transitions.

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
