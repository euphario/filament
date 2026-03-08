//! IRQ Registration and Handling
//!
//! Allows userspace drivers to register for hardware interrupts.
//! When an IRQ fires, the owning process is woken.

/// Error type for IRQ registration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrqError {
    /// IRQ number is already registered by another process
    AlreadyRegistered,
    /// No free slots in the IRQ registration table
    TableFull,
}

/// IRQ delivery state machine.
///
/// Transitions:
///   Idle → WaitingForIrq (driver calls wait/blocks)
///   Idle → Pending       (IRQ fires while driver is busy)
///   WaitingForIrq → Idle (IRQ fires → wake blocked driver, consume)
///   Pending → Idle       (driver calls check_pending → consume)
#[derive(Clone, Copy)]
pub enum IrqState {
    /// No IRQ pending, no one waiting
    Idle,
    /// Driver is blocked waiting for the next IRQ
    WaitingForIrq { pid: u32 },
    /// One or more IRQs arrived while driver was not waiting
    Pending { count: u32 },
}

/// IRQ registration entry
#[derive(Clone, Copy)]
pub struct IrqRegistration {
    /// IRQ number (0xFFFFFFFF = empty slot)
    pub irq_num: u32,
    /// Process that registered for this IRQ
    pub owner_pid: u32,
    /// Delivery state
    pub state: IrqState,
}

impl IrqRegistration {
    pub const fn empty() -> Self {
        Self {
            irq_num: 0xFFFFFFFF,
            owner_pid: 0,
            state: IrqState::Idle,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.irq_num == 0xFFFFFFFF
    }
}

/// Maximum IRQ registrations
pub const MAX_IRQ_REGISTRATIONS: usize = 32;

/// Global IRQ registration table (protected by SpinLock)
static IRQ_TABLE: super::lock::SpinLock<[IrqRegistration; MAX_IRQ_REGISTRATIONS]> =
    super::lock::SpinLock::new(super::lock::lock_class::SUBSYSTEM, [IrqRegistration::empty(); MAX_IRQ_REGISTRATIONS]);

/// Execute a closure with exclusive access to the IRQ table
#[inline]
fn with_irq_table<R, F: FnOnce(&mut [IrqRegistration; MAX_IRQ_REGISTRATIONS]) -> R>(f: F) -> R {
    let mut table = IRQ_TABLE.lock();
    f(&mut *table)
}

/// Register a process for an IRQ.
///
/// For virtual MSI IRQs (>= 1024), skips GIC enable — the GIC SPI is
/// managed by the MSI dispatch layer, not per-IRQ registration.
pub fn register(irq_num: u32, pid: u32) -> Result<(), IrqError> {
    with_irq_table(|table| {
        // Check if already registered
        for reg in table.iter() {
            if !reg.is_empty() && reg.irq_num == irq_num {
                return Err(IrqError::AlreadyRegistered);
            }
        }

        // Find empty slot and register
        for reg in table.iter_mut() {
            if reg.is_empty() {
                reg.irq_num = irq_num;
                reg.owner_pid = pid;
                reg.state = IrqState::Idle;

                // Enable the IRQ in the GIC (skip for virtual MSI IRQs)
                if !crate::kernel::pci::MsiAllocator::is_msi_irq(irq_num) {
                    crate::platform::current::gic::enable_irq(irq_num);
                    crate::platform::current::gic::debug_irq(irq_num);
                }

                return Ok(());
            }
        }
        Err(IrqError::TableFull)
    })
}

/// Unregister a process from an IRQ
pub fn unregister(irq_num: u32, pid: u32) -> bool {
    with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid {
                if !crate::kernel::pci::MsiAllocator::is_msi_irq(irq_num) {
                    crate::platform::current::gic::disable_irq(irq_num);
                }
                *reg = IrqRegistration::empty();
                return true;
            }
        }
        false
    })
}

/// Set IRQ affinity — route an SPI to a specific CPU core.
/// Only the owning process can change affinity.
pub fn set_affinity(irq_num: u32, pid: u32, cpu: u32) -> bool {
    let is_owner = with_irq_table(|table| {
        table.iter().any(|reg| !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid)
    });
    if is_owner {
        crate::platform::current::gic::set_affinity(irq_num, cpu);
        true
    } else {
        false
    }
}

/// Clean up all IRQ registrations for a process (called on process exit)
pub fn process_cleanup(pid: u32) {
    with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.owner_pid == pid {
                if !crate::kernel::pci::MsiAllocator::is_msi_irq(reg.irq_num) {
                    crate::platform::current::gic::disable_irq(reg.irq_num);
                }
                *reg = IrqRegistration::empty();
            }
        }
    })
}

/// Called from IRQ handler when an interrupt fires.
/// Returns the PID to wake up, if any.
pub fn notify(irq_num: u32) -> Option<u32> {
    // Mask the IRQ immediately to prevent interrupt storm.
    // For virtual MSI IRQs, masking is done at the MAC level by the dispatch
    // code (W1C of the status register), so skip GIC disable.
    if !crate::kernel::pci::MsiAllocator::is_msi_irq(irq_num) {
        crate::platform::current::gic::disable_irq(irq_num);
    }

    // Update state and collect wake info
    let (wake_pid, owner_pid) = with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num {
                match reg.state {
                    IrqState::WaitingForIrq { pid } => {
                        // Driver was blocked — wake it, transition to Idle
                        reg.state = IrqState::Idle;
                        return (Some(pid), reg.owner_pid);
                    }
                    IrqState::Pending { count } => {
                        // Already pending — coalesce
                        reg.state = IrqState::Pending { count: count.saturating_add(1) };
                        return (None, reg.owner_pid);
                    }
                    IrqState::Idle => {
                        // Driver not waiting — mark pending
                        reg.state = IrqState::Pending { count: 1 };
                        return (None, reg.owner_pid);
                    }
                }
            }
        }
        (None, 0)
    });

    // Wake process outside the IRQ table lock
    if let Some(pid) = wake_pid {
        super::task::with_scheduler(|sched| {
            sched.wake_by_pid(pid);
        });
        Some(pid)
    } else if owner_pid != 0 {
        // Owner exists but wasn't blocked — wake via scheduler in case
        // they're sleeping on a Mux that watches this IRQ
        super::task::with_scheduler(|sched| {
            sched.wake_by_pid(owner_pid);
        });
        Some(owner_pid)
    } else {
        None
    }
}

/// Check if an IRQ is pending for a process.
/// If pending, consumes the event and re-enables the IRQ.
pub fn check_pending(irq_num: u32, pid: u32) -> Option<u32> {
    with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid {
                if let IrqState::Pending { count } = reg.state {
                    reg.state = IrqState::Idle;
                    // Re-enable at GIC (skip for virtual MSI IRQs — MAC handles it)
                    if !crate::kernel::pci::MsiAllocator::is_msi_irq(irq_num) {
                        crate::platform::current::gic::enable_irq(irq_num);
                    }
                    return Some(count);
                }
                return None;
            }
        }
        None
    })
}

/// Non-consuming check: returns true if an IRQ is pending for this process.
/// Unlike check_pending(), does NOT clear the flag or re-enable the IRQ.
/// Used by Mux polling to report readiness without consuming the event.
pub fn check_pending_peek(irq_num: u32, pid: u32) -> bool {
    with_irq_table(|table| {
        for reg in table.iter() {
            if !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid {
                return matches!(reg.state, IrqState::Pending { .. });
            }
        }
        false
    })
}

/// Block waiting for an IRQ (called from syscall context)
pub fn wait(irq_num: u32, pid: u32) -> Result<u32, i32> {
    // Check if IRQ is already pending
    if let Some(count) = check_pending(irq_num, pid) {
        return Ok(count);
    }

    // Transition to WaitingForIrq
    let found = with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid {
                reg.state = IrqState::WaitingForIrq { pid };
                return true;
            }
        }
        false
    });

    if !found {
        return Err(-22); // EINVAL - not registered for this IRQ
    }

    // Atomically block and reschedule
    if !super::sched::sleep_and_reschedule(super::task::SleepReason::Irq) {
        return Err(-11); // EAGAIN
    }

    // Woken up - check for pending IRQ
    if let Some(count) = check_pending(irq_num, pid) {
        Ok(count)
    } else {
        Err(-4) // EINTR - interrupted
    }
}
