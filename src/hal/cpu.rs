//! CPU Operations HAL
//!
//! Platform-agnostic CPU operations for idle, IRQ control, and barriers.

/// CPU operations - architecture provides implementation
pub trait Cpu: Send + Sync {
    /// Enter low-power idle state until interrupt
    /// ARM: WFI, x86: HLT, RISC-V: WFI
    fn idle(&self);

    /// Enable interrupts
    fn enable_irq(&self);

    /// Disable interrupts, return previous state
    fn disable_irq(&self) -> bool;

    /// Restore interrupt state
    fn restore_irq(&self, was_enabled: bool);

    /// Memory barrier (full)
    fn memory_barrier(&self);

    /// Get current CPU ID (for SMP)
    fn cpu_id(&self) -> usize;
}

/// RAII guard for disabling IRQs
pub struct IrqGuard<'a, C: Cpu> {
    cpu: &'a C,
    was_enabled: bool,
}

impl<'a, C: Cpu> IrqGuard<'a, C> {
    pub fn new(cpu: &'a C) -> Self {
        let was_enabled = cpu.disable_irq();
        Self { cpu, was_enabled }
    }
}

impl<C: Cpu> Drop for IrqGuard<'_, C> {
    fn drop(&mut self) {
        self.cpu.restore_irq(self.was_enabled);
    }
}
