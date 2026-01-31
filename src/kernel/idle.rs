//! Idle Task
//!
//! Each CPU has its own idle task in a reserved scheduler slot (slot N = CPU N).
//!
//! # Design
//!
//! - **Always Ready**: The idle task never blocks - it's always Ready or Running
//! - **Lowest Priority**: Uses Priority::Low (or conceptual Priority::Idle)
//! - **WFI Loop**: Enters low-power wait state until next interrupt
//! - **Real Task**: Has proper kernel context for context switching
//! - **Per-CPU**: Each CPU has its own idle task and stack
//!
//! # Why a Real Idle Task?
//!
//! Previously, "idle" was a code path inside the scheduler. This caused problems:
//! 1. Blocked tasks had nowhere to save their kernel context
//! 2. No proper WFI with IRQs enabled between work
//! 3. Context switching was inconsistent
//!
//! With a real idle task:
//! - Every context switch is task→task (consistent)
//! - Blocked task saves context, idle task resumes
//! - When woken, blocked task's context is restored properly

use crate::hal::Cpu;
use crate::arch::aarch64::hal::cpu;

/// Idle task entry point
///
/// This is a kernel task that runs when no other task is ready.
/// It loops forever, calling WFI to save power between interrupts.
///
/// The idle task NEVER blocks - it's always Ready or Running.
pub fn idle_entry() -> ! {
    // Get the CPU for WFI
    let cpu_impl = cpu();

    loop {
        // Enable interrupts before WFI.
        // We may enter this loop with IRQs disabled (after context_switch from
        // reschedule which holds IrqGuard). Timer IRQs need to fire to wake tasks.
        cpu_impl.enable_irq();

        // Wait for interrupt
        // When IRQ fires, we return here, timer_tick runs check_deadlines,
        // and if a task is ready, we get context-switched away via do_resched_if_needed
        // in irq_from_kernel.
        cpu_impl.idle();

        // Note: We don't call reschedule() here.
        // The timer IRQ handler sets NEED_RESCHED flag.
        // do_resched_if_needed() runs at safe points (syscall return, IRQ return)
        // and will switch us away if another task became ready.
        //
        // When that task blocks, it will context_switch back to us,
        // and we just loop and idle again.
    }
}

/// Stack for idle task (small, it does almost nothing)
/// 4KB is enough - idle task only saves callee-saved registers
#[repr(C, align(16))]
pub struct IdleStack {
    data: [u8; 4096],
}

impl IdleStack {
    pub const fn new() -> Self {
        Self { data: [0; 4096] }
    }

    /// Get the stack top (stacks grow downward on ARM)
    pub fn top(&self) -> *mut u8 {
        unsafe { (self.data.as_ptr() as *mut u8).add(4096) }
    }
}

use super::percpu::MAX_CPUS;

/// Per-CPU idle stacks - statically allocated
static mut IDLE_STACKS: [IdleStack; MAX_CPUS] = [
    IdleStack::new(),
    IdleStack::new(),
    IdleStack::new(),
    IdleStack::new(),
];

/// Get the idle stack top for a given CPU
/// # Safety
/// Must only be called during scheduler initialization
pub unsafe fn idle_stack_top_for_cpu(cpu: u32) -> *mut u8 {
    let stacks_ptr = core::ptr::addr_of_mut!(IDLE_STACKS);
    (*stacks_ptr)[cpu as usize].data.as_mut_ptr().add(4096)
}

/// Get the idle stack top for CPU 0 (backward compat)
/// # Safety
/// Must only be called during scheduler initialization
pub unsafe fn idle_stack_top() -> *mut u8 {
    idle_stack_top_for_cpu(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_idle_stack_alignment() {
        unsafe {
            let top = idle_stack_top_for_cpu(0);
            assert_eq!(top as usize % 16, 0, "Stack must be 16-byte aligned");
        }
    }
}
