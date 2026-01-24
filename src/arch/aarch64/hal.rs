//! AArch64 HAL Implementation
//!
//! Implements the HAL traits for AArch64 (ARM64) architecture.

use crate::hal::{Cpu, Context, ContextSwitch};

// ============================================================================
// Cpu Implementation
// ============================================================================

/// AArch64 CPU operations
pub struct Aarch64Cpu;

impl Aarch64Cpu {
    pub const fn new() -> Self {
        Self
    }
}

impl Cpu for Aarch64Cpu {
    #[inline]
    fn idle(&self) {
        unsafe { core::arch::asm!("wfi") }
    }

    #[inline]
    fn enable_irq(&self) {
        unsafe { core::arch::asm!("msr daifclr, #2") }
    }

    #[inline]
    fn disable_irq(&self) -> bool {
        let daif: u64;
        unsafe {
            core::arch::asm!("mrs {}, daif", out(reg) daif);
            core::arch::asm!("msr daifset, #2");
        }
        // I bit is bit 7 - if clear, IRQs were enabled
        (daif & 0x80) == 0
    }

    #[inline]
    fn restore_irq(&self, was_enabled: bool) {
        if was_enabled {
            self.enable_irq();
        }
    }

    #[inline]
    fn memory_barrier(&self) {
        unsafe { core::arch::asm!("dsb sy; isb") }
    }

    #[inline]
    fn cpu_id(&self) -> usize {
        let mpidr: u64;
        unsafe { core::arch::asm!("mrs {}, mpidr_el1", out(reg) mpidr) }
        (mpidr & 0xFF) as usize
    }
}

/// Global AArch64 CPU instance
pub static AARCH64_CPU: Aarch64Cpu = Aarch64Cpu::new();

// ============================================================================
// Context Implementation
// ============================================================================

/// AArch64 kernel context (callee-saved registers)
///
/// This matches the layout expected by context_switch_asm in task/tcb.rs.
#[repr(C)]
#[derive(Clone)]
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

impl Default for Aarch64Context {
    fn default() -> Self {
        Self {
            x19: 0, x20: 0, x21: 0, x22: 0,
            x23: 0, x24: 0, x25: 0, x26: 0,
            x27: 0, x28: 0, x29: 0, x30: 0,
            sp: 0,
        }
    }
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

// ============================================================================
// ContextSwitch Implementation
// ============================================================================

/// AArch64 context switch implementation
pub struct Aarch64ContextSwitch;

impl ContextSwitch for Aarch64ContextSwitch {
    type Ctx = Aarch64Context;

    unsafe fn switch(current: &mut Self::Ctx, next: &Self::Ctx) {
        aarch64_context_switch(current as *mut _, next as *const _);
    }
}

// Assembly implementation for context switch
// This is a simplified version that matches the existing context_switch_asm layout
extern "C" {
    fn aarch64_context_switch(current: *mut Aarch64Context, next: *const Aarch64Context);
}

// Provide the assembly implementation
core::arch::global_asm!(r#"
.global aarch64_context_switch
.type aarch64_context_switch, @function
aarch64_context_switch:
    // MEMORY BARRIER: Ensure all stores are visible before switch
    dsb     sy

    // Save callee-saved registers for current task
    stp     x19, x20, [x0, #0]
    stp     x21, x22, [x0, #16]
    stp     x23, x24, [x0, #32]
    stp     x25, x26, [x0, #48]
    stp     x27, x28, [x0, #64]
    stp     x29, x30, [x0, #80]
    mov     x9, sp
    str     x9, [x0, #96]

    // Restore callee-saved registers for next task
    ldp     x19, x20, [x1, #0]
    ldp     x21, x22, [x1, #16]
    ldp     x23, x24, [x1, #32]
    ldp     x25, x26, [x1, #48]
    ldp     x27, x28, [x1, #64]
    ldp     x29, x30, [x1, #80]
    ldr     x9, [x1, #96]
    mov     sp, x9

    // MEMORY BARRIER: Ensure all loads complete before returning
    dsb     sy
    isb

    ret
"#);

// ============================================================================
// Global Accessor
// ============================================================================

/// Get the AArch64 CPU implementation
#[inline]
pub fn cpu() -> &'static Aarch64Cpu {
    &AARCH64_CPU
}
