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

/// IRQ registration entry
#[derive(Clone, Copy)]
pub struct IrqRegistration {
    /// IRQ number
    pub irq_num: u32,
    /// Process that registered for this IRQ
    pub owner_pid: u32,
    /// Whether an IRQ is pending (occurred but not yet read)
    pub pending: bool,
    /// Count of pending IRQs (allows coalescing)
    pub pending_count: u32,
    /// PID of process blocked waiting for this IRQ (0 = none)
    pub blocked_pid: u32,
}

impl IrqRegistration {
    pub const fn empty() -> Self {
        Self {
            irq_num: 0xFFFFFFFF,
            owner_pid: 0,
            pending: false,
            pending_count: 0,
            blocked_pid: 0,
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
    super::lock::SpinLock::new([IrqRegistration::empty(); MAX_IRQ_REGISTRATIONS]);

/// Execute a closure with exclusive access to the IRQ table
#[inline]
fn with_irq_table<R, F: FnOnce(&mut [IrqRegistration; MAX_IRQ_REGISTRATIONS]) -> R>(f: F) -> R {
    let mut table = IRQ_TABLE.lock();
    f(&mut *table)
}

/// Register a process for an IRQ
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
                reg.pending = false;
                reg.pending_count = 0;

                // Enable the IRQ in the GIC
                crate::platform::current::gic::enable_irq(irq_num);
                crate::platform::current::gic::debug_irq(irq_num);

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
                crate::platform::current::gic::disable_irq(irq_num);
                *reg = IrqRegistration::empty();
                return true;
            }
        }
        false
    })
}

/// Clean up all IRQ registrations for a process (called on process exit)
pub fn process_cleanup(pid: u32) {
    with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.owner_pid == pid {
                crate::platform::current::gic::disable_irq(reg.irq_num);
                *reg = IrqRegistration::empty();
            }
        }
    })
}

/// Called from IRQ handler when an interrupt fires
/// Returns the PID to wake up, if any
pub fn notify(irq_num: u32) -> Option<u32> {
    // Mask the IRQ immediately to prevent interrupt storm
    crate::platform::current::gic::disable_irq(irq_num);

    // Update IRQ table state and collect wake info
    let (wake_blocked, owner_pid) = with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num {
                reg.pending = true;
                reg.pending_count = reg.pending_count.saturating_add(1);

                if reg.blocked_pid != 0 {
                    let pid = reg.blocked_pid;
                    reg.blocked_pid = 0;
                    return (Some(pid), reg.owner_pid);
                } else {
                    return (None, reg.owner_pid);
                }
            }
        }
        (None, 0)
    });

    // Wake process outside the IRQ table lock
    if let Some(pid) = wake_blocked {
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

/// Check if an IRQ is pending for a process
/// If pending, clears the flag and re-enables the IRQ
pub fn check_pending(irq_num: u32, pid: u32) -> Option<u32> {
    with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid {
                if reg.pending {
                    let count = reg.pending_count;
                    reg.pending = false;
                    reg.pending_count = 0;
                    crate::platform::current::gic::enable_irq(irq_num);
                    return Some(count);
                }
                return None;
            }
        }
        None
    })
}

/// Block waiting for an IRQ (called from syscall context)
pub fn wait(irq_num: u32, pid: u32) -> Result<u32, i32> {
    // Check if IRQ is already pending
    if let Some(count) = check_pending(irq_num, pid) {
        return Ok(count);
    }

    // Set up for blocking
    let found = with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid {
                reg.blocked_pid = pid;
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
