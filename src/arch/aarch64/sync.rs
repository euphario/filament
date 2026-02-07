//! Synchronization Primitives
//!
//! Provides interrupt guards and other synchronization primitives
//! for safe access to kernel data structures.

#![allow(dead_code)]  // Some methods are infrastructure for future use

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

/// Guard that disables IRQs on creation and restores on drop.
///
/// Only saves and restores the I (IRQ) bit - leaves D, A, F bits untouched.
/// This prevents accidentally clobbering other exception mask bits.
///
/// Use this when modifying scheduler state, task lists, or any
/// shared data that could be accessed from interrupt context.
///
/// # Example
/// ```
/// {
///     let _guard = IrqGuard::new();
///     // IRQs disabled here
///     scheduler.current = next_slot;
/// } // IRQs restored here
/// ```
pub struct IrqGuard {
    /// True if IRQs were enabled before we disabled them
    irqs_were_enabled: bool,
}

impl IrqGuard {
    /// Create a new IrqGuard, disabling IRQs.
    /// Only the I bit state is saved - D/A/F bits are not touched.
    #[inline]
    pub fn new() -> Self {
        let daif: u64;
        unsafe {
            // Read current DAIF to check I bit state
            core::arch::asm!("mrs {}, daif", out(reg) daif);
            // Disable IRQs (set I bit) - uses DAIFSET which only sets bits
            core::arch::asm!("msr daifset, #2");
        }
        // I bit is bit 7 - if clear, IRQs were enabled
        Self { irqs_were_enabled: (daif & (1 << 7)) == 0 }
    }

    /// Check if IRQs were enabled before this guard was created
    #[inline]
    pub fn was_enabled(&self) -> bool {
        self.irqs_were_enabled
    }
}

impl Drop for IrqGuard {
    #[inline]
    fn drop(&mut self) {
        if self.irqs_were_enabled {
            unsafe {
                // Clear I bit to re-enable IRQs - uses DAIFCLR which only clears bits
                core::arch::asm!("msr daifclr, #2");
            }
        }
        // If IRQs were already disabled, leave them disabled
    }
}

/// Per-CPU flags for deferred work
pub struct CpuFlags {
    /// Set by timer IRQ, cleared after reschedule
    need_resched: AtomicBool,
    /// Timer tick count (for debugging/stats)
    timer_ticks: AtomicU32,
    /// Unhandled IRQ count (for debugging)
    unhandled_irqs: AtomicU32,
    /// Last unhandled IRQ number
    last_unhandled_irq: AtomicU32,
}

impl CpuFlags {
    pub const fn new() -> Self {
        Self {
            need_resched: AtomicBool::new(false),
            timer_ticks: AtomicU32::new(0),
            unhandled_irqs: AtomicU32::new(0),
            last_unhandled_irq: AtomicU32::new(0),
        }
    }

    /// Mark that a reschedule is needed (called from timer IRQ)
    #[inline]
    pub fn set_need_resched(&self) {
        self.need_resched.store(true, Ordering::Release);
    }

    /// Check and clear need_resched flag (called from safe point)
    #[inline]
    pub fn check_and_clear_resched(&self) -> bool {
        self.need_resched.swap(false, Ordering::AcqRel)
    }

    /// Check need_resched without clearing
    #[inline]
    pub fn need_resched(&self) -> bool {
        self.need_resched.load(Ordering::Acquire)
    }

    /// Increment timer tick count
    #[inline]
    pub fn tick(&self) {
        self.timer_ticks.fetch_add(1, Ordering::Relaxed);
    }

    /// Get timer tick count
    #[inline]
    pub fn get_ticks(&self) -> u32 {
        self.timer_ticks.load(Ordering::Relaxed)
    }

    /// Record an unhandled IRQ (instead of printing in IRQ context)
    #[inline]
    pub fn record_unhandled_irq(&self, irq: u32) {
        self.unhandled_irqs.fetch_add(1, Ordering::Relaxed);
        self.last_unhandled_irq.store(irq, Ordering::Relaxed);
    }

    /// Get unhandled IRQ stats
    #[inline]
    pub fn get_unhandled_stats(&self) -> (u32, u32) {
        (
            self.unhandled_irqs.load(Ordering::Relaxed),
            self.last_unhandled_irq.load(Ordering::Relaxed),
        )
    }

    /// Clear unhandled IRQ counter (after logging)
    #[inline]
    pub fn clear_unhandled_stats(&self) {
        self.unhandled_irqs.store(0, Ordering::Relaxed);
    }
}

/// Global CPU flags (single core for now)
static CPU_FLAGS: CpuFlags = CpuFlags::new();

/// Get the current CPU's flags
#[inline]
pub fn cpu_flags() -> &'static CpuFlags {
    &CPU_FLAGS
}

/// Check if we're currently in interrupt context
/// (IRQs are disabled by hardware when in IRQ handler)
#[inline]
pub fn in_interrupt_context() -> bool {
    let daif: u64;
    unsafe {
        core::arch::asm!("mrs {}, daif", out(reg) daif);
    }
    // I bit set means IRQs disabled (likely in IRQ handler)
    (daif & (1 << 7)) != 0
}

// ============================================================================
// PAN (Privileged Access Never) Support
// ============================================================================

/// Check if PAN (Privileged Access Never) is supported by the CPU.
///
/// PAN is an ARMv8.1 feature. We check ID_AA64MMFR1_EL1.PAN field (bits [23:20]).
/// Value 0 = not supported, 1+ = supported.
#[inline]
pub fn is_pan_supported() -> bool {
    let mmfr1: u64;
    unsafe {
        core::arch::asm!("mrs {}, ID_AA64MMFR1_EL1", out(reg) mmfr1);
    }
    // PAN field is bits [23:20]
    ((mmfr1 >> 20) & 0xF) != 0
}

/// Check if PAN (Privileged Access Never) is currently enabled.
///
/// PAN prevents the kernel from accidentally accessing user memory directly.
/// When PAN is enabled, any load/store to user addresses from EL1 causes a fault.
///
/// This is a security feature - the kernel should always access user memory
/// through explicit uaccess helpers that translate VA→PA and access via TTBR1.
///
/// Returns false if PAN is not supported by the CPU.
#[inline]
pub fn is_pan_enabled() -> bool {
    if !is_pan_supported() {
        return false;
    }
    let pstate: u64;
    unsafe {
        // Read PSTATE.PAN via the PAN system register
        core::arch::asm!("mrs {}, S3_0_C4_C2_3", out(reg) pstate);
    }
    // PAN bit is bit 22 in PSTATE
    (pstate & (1 << 22)) != 0
}

/// Verify that PAN is enabled and working correctly.
///
/// This should be called early in kernel initialization to ensure
/// the security feature is active. If PAN is not supported by the CPU,
/// logs a warning but continues (the kernel still works safely via
/// explicit VA→PA translation in uaccess).
pub fn verify_pan_enabled() {
    if !is_pan_supported() {
        // PAN not supported on this CPU - that's OK, uaccess still works
        // via explicit translation. Just skip the check.
        return;
    }
    if !is_pan_enabled() {
        panic!("PAN (Privileged Access Never) is not enabled! Security risk.");
    }
}

/// Debug: Get SCTLR_EL1 for checking SPAN bit
/// SPAN=0 means PAN is set on exception entry
#[inline]
pub fn get_sctlr_span() -> bool {
    let sctlr: u64;
    unsafe {
        core::arch::asm!("mrs {}, sctlr_el1", out(reg) sctlr);
    }
    // SPAN is bit 23
    (sctlr & (1 << 23)) != 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{kdebug, kinfo};

    pub fn test() {
        kdebug!("sync", "test_start");

        // Test IrqGuard - only I bit should be affected
        let before: u64;
        unsafe { core::arch::asm!("mrs {}, daif", out(reg) before); }
        let irqs_were_enabled = (before & (1 << 7)) == 0;

        {
            let guard = IrqGuard::new();
            let during: u64;
            unsafe { core::arch::asm!("mrs {}, daif", out(reg) during); }
            // IRQs should be disabled (I bit set)
            assert!((during & (1 << 7)) != 0, "IRQs not disabled in guard");
            // D, A, F bits should be unchanged
            assert_eq!(before & 0x2C0, during & 0x2C0, "D/A/F bits were modified");
            assert_eq!(guard.was_enabled(), irqs_were_enabled);
            let _ = guard; // Keep guard alive
        }

        let after: u64;
        unsafe { core::arch::asm!("mrs {}, daif", out(reg) after); }
        // Only I bit should be restored, D/A/F should be unchanged from before
        assert_eq!(before & (1 << 7), after & (1 << 7), "I bit not restored after guard drop");

        // Test CpuFlags
        let flags = cpu_flags();
        assert!(!flags.need_resched());
        flags.set_need_resched();
        assert!(flags.need_resched());
        assert!(flags.check_and_clear_resched());
        assert!(!flags.need_resched());

        kinfo!("sync", "test_ok");
    }
}
