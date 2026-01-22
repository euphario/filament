//! Scheme System

#![allow(dead_code)]  // Infrastructure for future use
//!
//! Redox-style scheme handling for `scheme:path` URLs.
//! Schemes are like virtual filesystems that handle specific resource types.
//!
//! Built-in kernel schemes:
//! - `memory:` - memory information and allocation
//! - `time:` - system time and timers
//! - `irq:` - IRQ registration and handling
//! - `null:` - null device (discard writes, EOF on read)
//! - `zero:` - zero device (infinite zeros on read)
//! - `console:` - serial console I/O

use super::fd::{FdEntry, FdFlags, FdType};
use crate::arch::aarch64::tlb;
use crate::{kinfo, print_direct};

/// Maximum number of registered schemes
pub const MAX_SCHEMES: usize = 32;

/// Maximum path length
pub const MAX_PATH: usize = 256;

/// Scheme types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchemeType {
    /// Empty slot
    None,
    /// Kernel-provided scheme
    Kernel,
    /// User-space scheme (handled by a daemon)
    User,
}

/// Scheme handle - returned when opening a scheme resource
#[derive(Debug, Clone, Copy)]
pub struct SchemeHandle {
    /// Scheme ID
    pub scheme_id: u16,
    /// Resource-specific handle
    pub handle: u64,
    /// Flags
    pub flags: u32,
}

/// Kernel scheme operations
pub trait KernelScheme {
    /// Open a resource within this scheme
    fn open(&self, path: &[u8], flags: u32) -> Result<SchemeHandle, i32>;

    /// Read from a handle
    fn read(&self, handle: &SchemeHandle, buf: &mut [u8]) -> Result<usize, i32>;

    /// Write to a handle
    fn write(&self, handle: &SchemeHandle, buf: &[u8]) -> Result<usize, i32>;

    /// Close a handle
    fn close(&self, handle: &SchemeHandle) -> Result<(), i32>;

    /// Scheme name
    fn name(&self) -> &'static str;
}

/// Scheme registry entry
#[derive(Clone, Copy)]
pub struct SchemeEntry {
    /// Scheme name (e.g., "memory", "time")
    pub name: [u8; 32],
    /// Name length
    pub name_len: usize,
    /// Type of scheme
    pub scheme_type: SchemeType,
    /// For user schemes: owner PID
    pub owner_pid: u32,
    /// For user schemes: channel to communicate with daemon
    pub channel_id: u32,
    /// Scheme ID (index in registry)
    pub id: u16,
}

impl SchemeEntry {
    pub const fn empty() -> Self {
        Self {
            name: [0; 32],
            name_len: 0,
            scheme_type: SchemeType::None,
            owner_pid: 0,
            channel_id: 0,
            id: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.scheme_type == SchemeType::None
    }

    pub fn name_str(&self) -> &str {
        core::str::from_utf8(&self.name[..self.name_len]).unwrap_or("")
    }
}

/// Scheme registry
pub struct SchemeRegistry {
    schemes: [SchemeEntry; MAX_SCHEMES],
    count: usize,
}

impl SchemeRegistry {
    pub const fn new() -> Self {
        Self {
            schemes: [SchemeEntry::empty(); MAX_SCHEMES],
            count: 0,
        }
    }

    /// Register a kernel scheme
    pub fn register_kernel(&mut self, name: &str) -> Option<u16> {
        if name.len() > 31 {
            return None;
        }

        // Check if already registered
        for entry in &self.schemes {
            if !entry.is_empty() && entry.name_str() == name {
                return None; // Already exists
            }
        }

        // Find empty slot
        for (i, entry) in self.schemes.iter_mut().enumerate() {
            if entry.is_empty() {
                entry.name[..name.len()].copy_from_slice(name.as_bytes());
                entry.name_len = name.len();
                entry.scheme_type = SchemeType::Kernel;
                entry.owner_pid = 0;
                entry.channel_id = 0;
                entry.id = i as u16;
                self.count += 1;
                return Some(i as u16);
            }
        }
        None
    }

    /// Register a user scheme
    pub fn register_user(&mut self, name: &str, owner_pid: u32, channel_id: u32) -> Option<u16> {
        if name.len() > 31 {
            return None;
        }

        // Check if already registered
        for entry in &self.schemes {
            if !entry.is_empty() && entry.name_str() == name {
                return None;
            }
        }

        for (i, entry) in self.schemes.iter_mut().enumerate() {
            if entry.is_empty() {
                entry.name[..name.len()].copy_from_slice(name.as_bytes());
                entry.name_len = name.len();
                entry.scheme_type = SchemeType::User;
                entry.owner_pid = owner_pid;
                entry.channel_id = channel_id;
                entry.id = i as u16;
                self.count += 1;
                return Some(i as u16);
            }
        }
        None
    }

    /// Unregister a scheme
    pub fn unregister(&mut self, name: &str) -> bool {
        for entry in self.schemes.iter_mut() {
            if !entry.is_empty() && entry.name_str() == name {
                *entry = SchemeEntry::empty();
                self.count -= 1;
                return true;
            }
        }
        false
    }

    /// Find a scheme by name
    pub fn find(&self, name: &str) -> Option<&SchemeEntry> {
        for entry in &self.schemes {
            if !entry.is_empty() && entry.name_str() == name {
                return Some(entry);
            }
        }
        None
    }

    /// List all registered schemes
    pub fn list(&self) -> impl Iterator<Item = &SchemeEntry> {
        self.schemes.iter().filter(|e| !e.is_empty())
    }
}

/// Global scheme registry
static mut SCHEME_REGISTRY: SchemeRegistry = SchemeRegistry::new();

/// Get the global scheme registry
pub unsafe fn registry() -> &'static mut SchemeRegistry {
    &mut *core::ptr::addr_of_mut!(SCHEME_REGISTRY)
}

// ============================================================================
// Kernel Scheme Implementations
// ============================================================================

/// Memory scheme - provides memory info and operations
pub struct MemoryScheme;

impl MemoryScheme {
    /// Handle types for memory scheme
    const HANDLE_INFO: u64 = 1;      // Read memory info
    const HANDLE_PHYSICAL: u64 = 2;  // Physical memory operations
}

impl KernelScheme for MemoryScheme {
    fn name(&self) -> &'static str {
        "memory"
    }

    fn open(&self, path: &[u8], _flags: u32) -> Result<SchemeHandle, i32> {
        let path_str = core::str::from_utf8(path).map_err(|_| -22)?; // EINVAL

        let handle = match path_str {
            "" | "info" => Self::HANDLE_INFO,
            "physical" => Self::HANDLE_PHYSICAL,
            _ => return Err(-2), // ENOENT
        };

        Ok(SchemeHandle {
            scheme_id: 0, // Will be set by caller
            handle,
            flags: 0,
        })
    }

    fn read(&self, handle: &SchemeHandle, buf: &mut [u8]) -> Result<usize, i32> {
        match handle.handle {
            Self::HANDLE_INFO => {
                // Return memory info as text
                let total = super::pmm::total_count();
                let free = super::pmm::free_count();
                let s = format_memory_info(total, free);
                let bytes = s.as_bytes();
                let len = core::cmp::min(bytes.len(), buf.len());
                buf[..len].copy_from_slice(&bytes[..len]);
                Ok(len)
            }
            _ => Err(-22), // EINVAL
        }
    }

    fn write(&self, _handle: &SchemeHandle, _buf: &[u8]) -> Result<usize, i32> {
        Err(-1) // EPERM - memory scheme is read-only
    }

    fn close(&self, _handle: &SchemeHandle) -> Result<(), i32> {
        Ok(())
    }
}

fn format_memory_info(total: usize, free: usize) -> &'static str {
    // Simple static response for now
    // In a real implementation, we'd format this dynamically
    unsafe {
        static mut BUF: [u8; 128] = [0; 128];
        let buf = &mut *core::ptr::addr_of_mut!(BUF);
        let mut pos = 0;

        // Very basic number formatting
        pos += write_str(&mut buf[pos..], "total_pages:");
        pos += write_num(&mut buf[pos..], total);
        pos += write_str(&mut buf[pos..], "\nfree_pages:");
        pos += write_num(&mut buf[pos..], free);
        pos += write_str(&mut buf[pos..], "\npage_size:4096\n");

        core::str::from_utf8(&buf[..pos]).unwrap_or("")
    }
}

fn write_str(buf: &mut [u8], s: &str) -> usize {
    let bytes = s.as_bytes();
    let len = core::cmp::min(bytes.len(), buf.len());
    buf[..len].copy_from_slice(&bytes[..len]);
    len
}

fn write_num(buf: &mut [u8], mut n: usize) -> usize {
    if n == 0 {
        if !buf.is_empty() {
            buf[0] = b'0';
            return 1;
        }
        return 0;
    }

    let mut digits = [0u8; 20];
    let mut i = 0;
    while n > 0 && i < 20 {
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }

    let len = core::cmp::min(i, buf.len());
    for j in 0..len {
        buf[j] = digits[i - 1 - j];
    }
    len
}

/// Time scheme - provides time info and timer operations
pub struct TimeScheme;

impl TimeScheme {
    const HANDLE_NOW: u64 = 1;      // Read current time
    const HANDLE_UPTIME: u64 = 2;   // Read uptime
    const HANDLE_FREQ: u64 = 3;     // Read timer frequency
}

impl KernelScheme for TimeScheme {
    fn name(&self) -> &'static str {
        "time"
    }

    fn open(&self, path: &[u8], _flags: u32) -> Result<SchemeHandle, i32> {
        let path_str = core::str::from_utf8(path).map_err(|_| -22)?;

        let handle = match path_str {
            "" | "now" => Self::HANDLE_NOW,
            "uptime" => Self::HANDLE_UPTIME,
            "freq" => Self::HANDLE_FREQ,
            _ => return Err(-2),
        };

        Ok(SchemeHandle {
            scheme_id: 0,
            handle,
            flags: 0,
        })
    }

    fn read(&self, handle: &SchemeHandle, buf: &mut [u8]) -> Result<usize, i32> {
        if buf.len() < 8 {
            return Err(-22);
        }

        let value: u64 = match handle.handle {
            Self::HANDLE_NOW | Self::HANDLE_UPTIME => {
                let counter = crate::platform::mt7988::timer::counter();
                let freq = crate::platform::mt7988::timer::frequency();
                if freq > 0 {
                    (counter as u128 * 1_000_000_000 / freq as u128) as u64
                } else {
                    0
                }
            }
            Self::HANDLE_FREQ => crate::platform::mt7988::timer::frequency(),
            _ => return Err(-22),
        };

        // Return as little-endian u64
        buf[..8].copy_from_slice(&value.to_le_bytes());
        Ok(8)
    }

    fn write(&self, _handle: &SchemeHandle, _buf: &[u8]) -> Result<usize, i32> {
        Err(-1) // Time scheme is read-only for now
    }

    fn close(&self, _handle: &SchemeHandle) -> Result<(), i32> {
        Ok(())
    }
}

/// IRQ scheme - allows processes to register for IRQ events
pub struct IrqScheme;

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
/// Automatically disables interrupts for the duration.
#[inline]
pub fn with_irq_table<R, F: FnOnce(&mut [IrqRegistration; MAX_IRQ_REGISTRATIONS]) -> R>(f: F) -> R {
    let mut table = IRQ_TABLE.lock();
    f(&mut *table)
}

/// Register a process for an IRQ
pub fn irq_register(irq_num: u32, pid: u32) -> bool {
    with_irq_table(|table| {
        // Check if already registered (atomic with registration)
        for reg in table.iter() {
            if !reg.is_empty() && reg.irq_num == irq_num {
                return false; // IRQ already taken
            }
        }

        // Find empty slot and register atomically
        for reg in table.iter_mut() {
            if reg.is_empty() {
                reg.irq_num = irq_num;
                reg.owner_pid = pid;
                reg.pending = false;
                reg.pending_count = 0;

                // Enable the IRQ in the GIC
                crate::platform::mt7988::gic::enable_irq(irq_num);

                // Debug: show GIC state for this IRQ
                crate::platform::mt7988::gic::debug_irq(irq_num);

                return true;
            }
        }
        false
    })
}

/// Unregister a process from an IRQ
pub fn irq_unregister(irq_num: u32, pid: u32) -> bool {
    with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid {
                // Disable the IRQ in the GIC
                crate::platform::mt7988::gic::disable_irq(irq_num);

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
                // Disable the IRQ in the GIC
                crate::platform::mt7988::gic::disable_irq(reg.irq_num);
                *reg = IrqRegistration::empty();
            }
        }
    })
}

/// Called from IRQ handler when an interrupt fires
/// Returns the PID to wake up, if any
pub fn irq_notify(irq_num: u32) -> Option<u32> {
    // Mask the IRQ immediately to prevent interrupt storm (level-triggered)
    // Userspace will unmask by reading from the irq: scheme
    crate::platform::mt7988::gic::disable_irq(irq_num);

    // First, update IRQ table state and collect wake info
    // Keep lock scope minimal to avoid deadlock with scheduler
    let (wake_blocked, owner_pid) = with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num {
                reg.pending = true;
                reg.pending_count += 1;

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

    // Now handle waking outside the IRQ table lock
    if let Some(pid) = wake_blocked {
        // Wake blocked process using unified wake function
        unsafe {
            let sched = super::task::scheduler();
            sched.wake_by_pid(pid);
        }
        Some(pid)
    } else if owner_pid != 0 {
        // Send event to the process (fallback for non-blocking mode)
        let event = super::event::Event::irq(irq_num);
        super::event::deliver_event_to_task(owner_pid, event);
        Some(owner_pid)
    } else {
        None
    }
}

/// Check if an IRQ is pending for a process
/// If pending, clears the flag and re-enables the IRQ at the GIC
pub fn irq_check_pending(irq_num: u32, pid: u32) -> Option<u32> {
    with_irq_table(|table| {
        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid {
                if reg.pending {
                    let count = reg.pending_count;
                    reg.pending = false;
                    reg.pending_count = 0;

                    // Re-enable the IRQ at the GIC (it was masked when it fired)
                    // This allows the next interrupt to be delivered
                    crate::platform::mt7988::gic::enable_irq(irq_num);

                    return Some(count);
                }
                return None;
            }
        }
        None
    })
}

/// Block waiting for an IRQ (called from syscall context)
/// Returns pending count when IRQ fires, or error
pub fn irq_wait(irq_num: u32, pid: u32) -> Result<u32, i32> {
    // Check if IRQ is already pending
    if let Some(count) = irq_check_pending(irq_num, pid) {
        return Ok(count);
    }

    // Not pending - set up for blocking (minimal lock scope)
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

    // Now handle blocking outside the IRQ table lock
    unsafe {

        // Mark current task as Sleeping and switch to another task
        let sched = super::task::scheduler();
        let caller_slot = super::task::current_slot();

        // Mark current as Sleeping to exclude it from scheduling (waiting for IRQ)
        if let Some(ref mut task) = sched.tasks[caller_slot] {
            task.state = super::task::TaskState::Sleeping {
                reason: super::task::SleepReason::EventLoop,
            };
        }

        // Find next ready task
        if let Some(next_slot) = sched.schedule() {
            if next_slot != caller_slot {
                // Adjust ELR to restart the syscall when this task resumes
                if let Some(ref mut task) = sched.tasks[caller_slot] {
                    task.trap_frame.elr_el1 = task.trap_frame.elr_el1.wrapping_sub(4);
                }

                super::task::set_current_slot(next_slot);
                sched.current = next_slot;
                if let Some(ref mut task) = sched.tasks[next_slot] {
                    task.state = super::task::TaskState::Running;
                }
                super::task::update_current_task_globals();
                super::task::SYSCALL_SWITCHED_TASK.store(1, core::sync::atomic::Ordering::Release);

                // Return value is ignored since we switched tasks
                return Err(-11); // EAGAIN
            }
        }

        // No other task to switch to - spin-wait in kernel with interrupts enabled
        if let Some(ref mut task) = sched.tasks[caller_slot] {
            task.state = super::task::TaskState::Running;
        }

        // Spin-wait with interrupts enabled until IRQ fires
        // IMPORTANT: Add timeout to prevent infinite hang if IRQ never arrives
        const IRQ_WAIT_TIMEOUT_MS: u64 = 5000; // 5 second timeout
        let start = crate::platform::mt7988::timer::counter();
        let freq = crate::platform::mt7988::timer::frequency();
        let deadline = start + (IRQ_WAIT_TIMEOUT_MS * freq / 1000);

        loop {
            // Check timeout first
            let now = crate::platform::mt7988::timer::counter();
            if now >= deadline {
                crate::kwarn!("scheme", "irq_wait_timeout"; irq = irq_num as u64, pid = pid as u64);
                return Err(-110); // ETIMEDOUT
            }

            core::arch::asm!("msr daifclr, #2"); // Unmask IRQs
            core::arch::asm!("wfi");

            if let Some(count) = irq_check_pending(irq_num, pid) {
                return Ok(count);
            }
        }
    }
}

impl IrqScheme {
    // Handle encodes the IRQ number
}

impl KernelScheme for IrqScheme {
    fn name(&self) -> &'static str {
        "irq"
    }

    fn open(&self, path: &[u8], flags: u32) -> Result<SchemeHandle, i32> {
        let path_str = core::str::from_utf8(path).map_err(|_| -22)?;

        // Parse IRQ number from path
        let irq_num: u32 = path_str.parse().map_err(|_| -22)?;

        // Validate IRQ number (0-447 for GIC-600)
        if irq_num >= 448 {
            return Err(-22);
        }

        // Get current process PID and check IRQ_CLAIM capability
        let pid = unsafe {
            let sched = super::task::scheduler();
            let task = sched.tasks[sched.current]
                .as_ref()
                .ok_or(-1)?;

            // Require IRQ_CLAIM capability
            if !task.has_capability(super::caps::Capabilities::IRQ_CLAIM) {
                return Err(-1); // EPERM
            }

            task.id
        };

        // Register for this IRQ
        if !irq_register(irq_num, pid) {
            return Err(-16); // EBUSY - IRQ already registered
        }

        // Store O_NONBLOCK in high bit of handle (bit 31)
        // irq_num uses bits 0-8, plenty of room
        const O_NONBLOCK: u32 = 0x800;
        let nonblock_bit = if (flags & O_NONBLOCK) != 0 { 1u64 << 31 } else { 0 };

        Ok(SchemeHandle {
            scheme_id: 0,
            handle: (irq_num as u64) | nonblock_bit,
            flags: pid,  // Store PID in flags for close()
        })
    }

    fn read(&self, handle: &SchemeHandle, buf: &mut [u8]) -> Result<usize, i32> {
        if buf.len() < 4 {
            return Err(-22);
        }

        // Extract nonblock flag from high bit of handle
        let is_nonblock = (handle.handle & (1u64 << 31)) != 0;
        let irq_num = (handle.handle & 0x1FF) as u32;  // Low 9 bits
        let pid = handle.flags;

        if is_nonblock {
            // Non-blocking mode: check if IRQ is pending, return EAGAIN if not
            if let Some(count) = irq_check_pending(irq_num, pid) {
                buf[..4].copy_from_slice(&count.to_le_bytes());
                Ok(4)
            } else {
                Err(-11) // EAGAIN - no IRQ pending
            }
        } else {
            // Blocking wait for IRQ - blocks until IRQ fires
            let count = irq_wait(irq_num, pid)?;
            buf[..4].copy_from_slice(&count.to_le_bytes());
            Ok(4)
        }
    }

    fn write(&self, _handle: &SchemeHandle, _buf: &[u8]) -> Result<usize, i32> {
        // Writing could be used to acknowledge/clear the IRQ
        // For now, reading clears it automatically
        Err(-1) // EPERM
    }

    fn close(&self, handle: &SchemeHandle) -> Result<(), i32> {
        // Mask out nonblock bit to get IRQ number
        let irq_num = (handle.handle & 0x1FF) as u32;
        let pid = handle.flags;

        // Unregister IRQ handler
        irq_unregister(irq_num, pid);
        Ok(())
    }
}

/// Console scheme - provides UART serial console I/O
/// Path format: empty or "0" for UART0
pub struct ConsoleScheme;

impl ConsoleScheme {
    /// Handle types
    const HANDLE_IN: u64 = 0;   // Read from console (stdin)
    const HANDLE_OUT: u64 = 1;  // Write to console (stdout)
    const HANDLE_ERR: u64 = 2;  // Write to console (stderr)
    const HANDLE_RW: u64 = 3;   // Bidirectional
}

impl KernelScheme for ConsoleScheme {
    fn name(&self) -> &'static str {
        "console"
    }

    fn open(&self, path: &[u8], flags: u32) -> Result<SchemeHandle, i32> {
        let path_str = core::str::from_utf8(path).map_err(|_| -22)?;

        // Determine handle type based on path and flags
        let handle = match path_str {
            "" | "0" => {
                // Check open flags
                let access = flags & 0x3;
                match access {
                    0 => Self::HANDLE_IN,  // O_RDONLY
                    1 => Self::HANDLE_OUT, // O_WRONLY
                    2 => Self::HANDLE_RW,  // O_RDWR
                    _ => Self::HANDLE_RW,
                }
            }
            "in" | "stdin" => Self::HANDLE_IN,
            "out" | "stdout" => Self::HANDLE_OUT,
            "err" | "stderr" => Self::HANDLE_ERR,
            _ => return Err(-2), // ENOENT
        };

        Ok(SchemeHandle {
            scheme_id: 0,
            handle,
            flags: 0,
        })
    }

    fn read(&self, handle: &SchemeHandle, buf: &mut [u8]) -> Result<usize, i32> {
        // Can only read from HANDLE_IN or HANDLE_RW
        if handle.handle != Self::HANDLE_IN && handle.handle != Self::HANDLE_RW {
            return Err(-9); // EBADF
        }

        if buf.is_empty() {
            return Ok(0);
        }

        // Non-blocking read from UART
        // Read as many characters as available (up to buf.len())
        let mut count = 0;
        while count < buf.len() {
            match crate::platform::mt7988::uart::try_getc() {
                Some(c) => {
                    buf[count] = c as u8;
                    count += 1;
                    // Echo the character back for interactive use
                    crate::platform::mt7988::uart::putc(c);
                    if c == '\n' || c == '\r' {
                        break; // Line-based input
                    }
                }
                None => break, // No more characters available
            }
        }

        if count > 0 {
            Ok(count)
        } else {
            Err(-11) // EAGAIN - would block
        }
    }

    fn write(&self, handle: &SchemeHandle, buf: &[u8]) -> Result<usize, i32> {
        // Can only write to HANDLE_OUT, HANDLE_ERR, or HANDLE_RW
        if handle.handle == Self::HANDLE_IN {
            return Err(-9); // EBADF
        }

        // Write all bytes to UART
        for &byte in buf {
            crate::platform::mt7988::uart::putc(byte as char);
        }

        Ok(buf.len())
    }

    fn close(&self, _handle: &SchemeHandle) -> Result<(), i32> {
        Ok(()) // Nothing to clean up
    }
}

/// Null scheme - discards writes, returns EOF on read
pub struct NullScheme;

impl KernelScheme for NullScheme {
    fn name(&self) -> &'static str {
        "null"
    }

    fn open(&self, _path: &[u8], _flags: u32) -> Result<SchemeHandle, i32> {
        Ok(SchemeHandle {
            scheme_id: 0,
            handle: 0,
            flags: 0,
        })
    }

    fn read(&self, _handle: &SchemeHandle, _buf: &mut [u8]) -> Result<usize, i32> {
        Ok(0) // EOF
    }

    fn write(&self, _handle: &SchemeHandle, buf: &[u8]) -> Result<usize, i32> {
        Ok(buf.len()) // Discard
    }

    fn close(&self, _handle: &SchemeHandle) -> Result<(), i32> {
        Ok(())
    }
}

/// Zero scheme - returns infinite zeros on read
pub struct ZeroScheme;

impl KernelScheme for ZeroScheme {
    fn name(&self) -> &'static str {
        "zero"
    }

    fn open(&self, _path: &[u8], _flags: u32) -> Result<SchemeHandle, i32> {
        Ok(SchemeHandle {
            scheme_id: 0,
            handle: 0,
            flags: 0,
        })
    }

    fn read(&self, _handle: &SchemeHandle, buf: &mut [u8]) -> Result<usize, i32> {
        for byte in buf.iter_mut() {
            *byte = 0;
        }
        Ok(buf.len())
    }

    fn write(&self, _handle: &SchemeHandle, buf: &[u8]) -> Result<usize, i32> {
        Ok(buf.len()) // Discard like null
    }

    fn close(&self, _handle: &SchemeHandle) -> Result<(), i32> {
        Ok(())
    }
}

/// MMIO scheme - maps device memory into userspace
/// Path format: <phys_addr_hex>/<size_hex>
/// Example: mmio:10000000/1000 maps 0x1000 bytes at physical 0x10000000
///
/// Security: This scheme should only be accessible to privileged driver processes
pub struct MmioScheme;

impl MmioScheme {
    /// Parse hex address from string
    fn parse_hex(s: &str) -> Option<u64> {
        let s = s.trim_start_matches("0x").trim_start_matches("0X");
        u64::from_str_radix(s, 16).ok()
    }

    /// Allocate virtual address in device region
    /// Uses a simple bump allocator in the device MMIO region
    fn alloc_device_virt(num_pages: usize) -> Option<u64> {
        const DEVICE_VIRT_START: u64 = 0x2000_0000; // 512MB mark for device mappings
        const DEVICE_VIRT_END: u64 = 0x3000_0000;   // 768MB mark

        unsafe {
            static mut NEXT_VIRT: u64 = DEVICE_VIRT_START;
            let virt = *core::ptr::addr_of!(NEXT_VIRT);
            let new_virt = virt + (num_pages * 4096) as u64;

            if new_virt > DEVICE_VIRT_END {
                return None;
            }

            *core::ptr::addr_of_mut!(NEXT_VIRT) = new_virt;
            Some(virt)
        }
    }
}

impl KernelScheme for MmioScheme {
    fn name(&self) -> &'static str {
        "mmio"
    }

    fn open(&self, path: &[u8], _flags: u32) -> Result<SchemeHandle, i32> {
        let path_str = core::str::from_utf8(path).map_err(|_| -22)?;

        // Parse phys_addr/size (no Vec in no_std, manual split)
        let slash_pos = path_str.find('/').ok_or(-22)?;
        let phys_str = &path_str[..slash_pos];
        let size_str = &path_str[slash_pos + 1..];

        if phys_str.is_empty() || size_str.is_empty() {
            return Err(-22); // EINVAL
        }

        let phys_addr = Self::parse_hex(phys_str).ok_or(-22)?;
        let size = Self::parse_hex(size_str).ok_or(-22)? as usize;

        if size == 0 {
            return Err(-22);
        }

        // Check alignment (must be page-aligned)
        if phys_addr & 0xFFF != 0 || size & 0xFFF != 0 {
            return Err(-22);
        }

        let num_pages = size / 4096;

        // Allocate virtual address space
        let virt_addr = Self::alloc_device_virt(num_pages).ok_or(-12)?; // ENOMEM

        // Get current task and map device pages
        unsafe {
            let sched = super::task::scheduler();
            if let Some(ref mut task) = sched.tasks[sched.current] {
                if let Some(ref mut addr_space) = task.address_space {
                    for i in 0..num_pages {
                        let page_virt = virt_addr + (i * 4096) as u64;
                        let page_phys = phys_addr + (i * 4096) as u64;
                        if !addr_space.map_device_page(page_virt, page_phys, true) {
                            // Cleanup on failure (would need unmap logic)
                            return Err(-12);
                        }
                    }

                    // Invalidate TLB for the new mappings using centralized API
                    let asid = addr_space.get_asid();
                    tlb::invalidate_va_range(asid, virt_addr, num_pages);
                } else {
                    return Err(-1); // Not a user task
                }
            } else {
                return Err(-1);
            }
        }

        // Encode virt_addr, phys_addr, and num_pages in handle
        // For simplicity, store virt_addr in handle, phys/size available via path
        Ok(SchemeHandle {
            scheme_id: 0,
            handle: virt_addr,
            flags: ((num_pages as u32) << 16) | ((phys_addr >> 12) as u32 & 0xFFFF),
        })
    }

    fn read(&self, handle: &SchemeHandle, buf: &mut [u8]) -> Result<usize, i32> {
        // Return the virtual address as a u64
        if buf.len() >= 8 {
            buf[..8].copy_from_slice(&handle.handle.to_le_bytes());
            Ok(8)
        } else {
            Err(-22)
        }
    }

    fn write(&self, _handle: &SchemeHandle, _buf: &[u8]) -> Result<usize, i32> {
        // MMIO scheme doesn't support write - access memory directly
        Err(-22)
    }

    fn close(&self, handle: &SchemeHandle) -> Result<(), i32> {
        let virt_addr = handle.handle;
        let num_pages = (handle.flags >> 16) as usize;

        if num_pages == 0 {
            return Ok(());
        }

        // Unmap pages from current task's address space
        unsafe {
            let sched = super::task::scheduler();
            if let Some(ref mut task) = sched.tasks[sched.current] {
                if let Some(ref mut addr_space) = task.address_space {
                    // Unmap all pages first
                    for i in 0..num_pages {
                        let page_virt = virt_addr + (i * 4096) as u64;
                        addr_space.unmap_page(page_virt);
                    }

                    // Invalidate TLB using centralized API
                    let asid = addr_space.get_asid();
                    tlb::invalidate_va_range(asid, virt_addr, num_pages);
                }
            }
        }

        // Note: We don't track physical pages for MMIO - they're device memory, not RAM
        Ok(())
    }
}

// ============================================================================
// I2C Scheme - provides I2C bus access to userspace
// ============================================================================

/// I2C scheme - provides I2C bus access
/// Path format: <bus>/<addr_hex>
/// Example: i2c:0/20 opens I2C bus 0, device address 0x20
///
/// Operations:
/// - read(buf): Read bytes from device (first byte is register address)
/// - write(buf): Write bytes to device (first byte is register address)
pub struct I2cScheme;

impl I2cScheme {
    fn parse_hex(s: &str) -> Option<u32> {
        let s = s.trim_start_matches("0x").trim_start_matches("0X");
        u32::from_str_radix(s, 16).ok()
    }
}

impl KernelScheme for I2cScheme {
    fn name(&self) -> &'static str {
        "i2c"
    }

    fn open(&self, path: &[u8], _flags: u32) -> Result<SchemeHandle, i32> {
        let path_str = core::str::from_utf8(path).map_err(|_| -22)?;

        // Parse bus/addr
        let slash_pos = path_str.find('/').ok_or(-22)?;
        let bus_str = &path_str[..slash_pos];
        let addr_str = &path_str[slash_pos + 1..];

        if bus_str.is_empty() || addr_str.is_empty() {
            return Err(-22); // EINVAL
        }

        let bus: u32 = bus_str.parse().map_err(|_| -22)?;
        let addr = Self::parse_hex(addr_str).ok_or(-22)?;

        if bus > 2 {
            return Err(-19); // ENODEV - only I2C0-2 supported
        }

        if addr > 0x7F {
            return Err(-22); // EINVAL - 7-bit address max
        }

        // Verify we can access the I2C controller (initializes on first use)
        let valid = crate::platform::mt7988::i2c::with_controller(bus, |_| true)
            .unwrap_or(false);
        if !valid {
            return Err(-19); // ENODEV
        }

        // Encode bus and addr in handle
        // handle: upper 32 bits = bus, lower 32 bits = addr
        let handle = ((bus as u64) << 32) | (addr as u64);

        Ok(SchemeHandle {
            scheme_id: 0,
            handle,
            flags: 0,
        })
    }

    fn read(&self, handle: &SchemeHandle, buf: &mut [u8]) -> Result<usize, i32> {
        let bus = (handle.handle >> 32) as u32;
        let addr = (handle.handle & 0xFF) as u8;

        if buf.is_empty() {
            return Ok(0);
        }

        // Read from device under lock
        crate::platform::mt7988::i2c::with_controller(bus, |ctrl| {
            ctrl.read(addr, buf)
        }).ok_or(-19)??;

        Ok(buf.len())
    }

    fn write(&self, handle: &SchemeHandle, buf: &[u8]) -> Result<usize, i32> {
        let bus = (handle.handle >> 32) as u32;
        let addr = (handle.handle & 0xFF) as u8;

        if buf.is_empty() {
            return Ok(0);
        }

        // Write to device under lock
        crate::platform::mt7988::i2c::with_controller(bus, |ctrl| {
            ctrl.write(addr, buf)
        }).ok_or(-19)??;

        Ok(buf.len())
    }

    fn close(&self, _handle: &SchemeHandle) -> Result<(), i32> {
        // Nothing to clean up for I2C
        Ok(())
    }
}

// ============================================================================
// PCIe Scheme
// ============================================================================

/// PCIe scheme - provides access to PCI devices
///
/// Path formats:
/// - `pcie:list` - list all devices (read returns JSON-like device info)
/// - `pcie:PPBB:DD.F/config` - config space (read/write at handle offset)
/// - `pcie:PPBB:DD.F/bar0` through `bar5` - map BAR (returns mapped vaddr)
/// - `pcie:PPBB:DD.F/msi` - allocate MSI (write count, read returns IRQ)
/// - `pcie:PPBB:DD.F/claim` - claim device ownership
///
/// Where PP=port, BB=bus, DD=device, F=function (all hex)
/// Example: pcie:0001:00.0/config for port 0, bus 1, device 0, function 0
pub struct PcieScheme;

/// PcieScheme handle types
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum PcieHandleType {
    List = 0,
    Config = 1,
    Bar = 2,
    Msi = 3,
    Claim = 4,
}

impl PcieScheme {
    /// Parse BDF from path segment "PPBB:DD.F"
    fn parse_bdf(s: &str) -> Option<super::pci::PciBdf> {
        // Format: PPBB:DD.F where PP=port, BB=bus, DD=device, F=function
        let colon_pos = s.find(':')?;
        let dot_pos = s.find('.')?;

        if colon_pos < 2 || dot_pos <= colon_pos + 1 {
            return None;
        }

        let port_bus = &s[..colon_pos];
        let device = &s[colon_pos + 1..dot_pos];
        let function = &s[dot_pos + 1..];

        // Parse port and bus from PPBB
        if port_bus.len() != 4 {
            return None;
        }
        let port = u8::from_str_radix(&port_bus[0..2], 16).ok()?;
        let bus = u8::from_str_radix(&port_bus[2..4], 16).ok()?;
        let dev = u8::from_str_radix(device, 16).ok()?;
        let func = u8::from_str_radix(function, 16).ok()?;

        if dev > 31 || func > 7 {
            return None;
        }

        Some(super::pci::PciBdf::with_port(port, bus, dev, func))
    }

    /// Encode handle: type(8) | bar(8) | bdf(32) | offset(16)
    fn encode_handle(handle_type: PcieHandleType, bar: u8, bdf: super::pci::PciBdf, offset: u16) -> u64 {
        ((handle_type as u64) << 56)
            | ((bar as u64) << 48)
            | ((bdf.to_u32() as u64) << 16)
            | (offset as u64)
    }

    /// Decode handle
    fn decode_handle(handle: u64) -> (PcieHandleType, u8, super::pci::PciBdf, u16) {
        let handle_type = match (handle >> 56) as u8 {
            0 => PcieHandleType::List,
            1 => PcieHandleType::Config,
            2 => PcieHandleType::Bar,
            3 => PcieHandleType::Msi,
            4 => PcieHandleType::Claim,
            _ => PcieHandleType::List,
        };
        let bar = ((handle >> 48) & 0xFF) as u8;
        let bdf = super::pci::PciBdf::from_u32(((handle >> 16) & 0xFFFFFFFF) as u32);
        let offset = (handle & 0xFFFF) as u16;
        (handle_type, bar, bdf, offset)
    }

    /// Format device list
    fn format_device_list(buf: &mut [u8]) -> usize {
        use core::fmt::Write;

        struct BufWriter<'a> {
            buf: &'a mut [u8],
            pos: usize,
        }

        impl<'a> Write for BufWriter<'a> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                let bytes = s.as_bytes();
                let remaining = self.buf.len() - self.pos;
                let to_write = bytes.len().min(remaining);
                self.buf[self.pos..self.pos + to_write].copy_from_slice(&bytes[..to_write]);
                self.pos += to_write;
                Ok(())
            }
        }

        let mut writer = BufWriter { buf, pos: 0 };

        for dev in super::pci::devices() {
            let _ = writeln!(
                writer,
                "{:02x}{:02x}:{:02x}.{} {:04x}:{:04x} {}",
                dev.bdf.port, dev.bdf.bus, dev.bdf.device, dev.bdf.function,
                dev.vendor_id, dev.device_id,
                dev.class_name()
            );
        }

        writer.pos
    }
}

impl KernelScheme for PcieScheme {
    fn name(&self) -> &'static str {
        "pcie"
    }

    fn open(&self, path: &[u8], _flags: u32) -> Result<SchemeHandle, i32> {
        let path_str = core::str::from_utf8(path).map_err(|_| -22)?;

        // Handle "list" path
        if path_str == "list" {
            return Ok(SchemeHandle {
                scheme_id: scheme_ids::PCIE,
                handle: Self::encode_handle(PcieHandleType::List, 0, super::pci::PciBdf::new(0, 0, 0), 0),
                flags: 0,
            });
        }

        // Parse BDF and resource type from path
        let slash_pos = path_str.find('/').ok_or(-22)?;
        let bdf_str = &path_str[..slash_pos];
        let resource = &path_str[slash_pos + 1..];

        let bdf = Self::parse_bdf(bdf_str).ok_or(-22)?;

        // Check device exists
        if super::pci::find_by_bdf(bdf).is_none() {
            return Err(-2); // ENOENT
        }

        let (handle_type, bar) = match resource {
            "config" => (PcieHandleType::Config, 0),
            "bar0" => (PcieHandleType::Bar, 0),
            "bar1" => (PcieHandleType::Bar, 1),
            "bar2" => (PcieHandleType::Bar, 2),
            "bar3" => (PcieHandleType::Bar, 3),
            "bar4" => (PcieHandleType::Bar, 4),
            "bar5" => (PcieHandleType::Bar, 5),
            "msi" => (PcieHandleType::Msi, 0),
            "claim" => (PcieHandleType::Claim, 0),
            _ => return Err(-22), // EINVAL
        };

        Ok(SchemeHandle {
            scheme_id: scheme_ids::PCIE,
            handle: Self::encode_handle(handle_type, bar, bdf, 0),
            flags: 0,
        })
    }

    fn read(&self, handle: &SchemeHandle, buf: &mut [u8]) -> Result<usize, i32> {
        let (handle_type, bar, bdf, offset) = Self::decode_handle(handle.handle);

        match handle_type {
            PcieHandleType::List => {
                // Return device list
                let len = Self::format_device_list(buf);
                Ok(len)
            }

            PcieHandleType::Config => {
                // Read config space at current offset
                if buf.len() < 4 {
                    return Err(-22);
                }

                match super::pci::config_read32(bdf, offset) {
                    Ok(val) => {
                        buf[..4].copy_from_slice(&val.to_le_bytes());
                        Ok(4)
                    }
                    Err(_) => Err(-5), // EIO
                }
            }

            PcieHandleType::Bar => {
                // Read returns BAR info (phys_addr:8, size:8)
                if buf.len() < 16 {
                    return Err(-22);
                }

                match super::pci::bar_info(bdf, bar) {
                    Ok((addr, size)) => {
                        buf[..8].copy_from_slice(&addr.to_le_bytes());
                        buf[8..16].copy_from_slice(&size.to_le_bytes());
                        Ok(16)
                    }
                    Err(_) => Err(-22),
                }
            }

            PcieHandleType::Msi => {
                // Read returns allocated IRQ number
                // For now, just return device's MSI capability info
                if let Some(dev) = super::pci::find_by_bdf(bdf) {
                    if buf.len() >= 2 {
                        buf[0] = dev.msi_cap;
                        buf[1] = dev.msix_cap;
                        Ok(2)
                    } else {
                        Err(-22)
                    }
                } else {
                    Err(-2) // ENOENT
                }
            }

            PcieHandleType::Claim => {
                // Read returns ownership status
                if let Some(dev) = super::pci::find_by_bdf(bdf) {
                    if buf.len() >= 4 {
                        buf[..4].copy_from_slice(&dev.owner_pid.to_le_bytes());
                        Ok(4)
                    } else {
                        Err(-22)
                    }
                } else {
                    Err(-2)
                }
            }
        }
    }

    fn write(&self, handle: &SchemeHandle, buf: &[u8]) -> Result<usize, i32> {
        let (handle_type, bar, bdf, offset) = Self::decode_handle(handle.handle);
        let pid = unsafe { super::task::scheduler().current_task_id().unwrap_or(0) };

        match handle_type {
            PcieHandleType::List => {
                // Can't write to list
                Err(-1) // EPERM
            }

            PcieHandleType::Config => {
                // Write config space - requires ownership
                if let Some(dev) = super::pci::find_by_bdf(bdf) {
                    if dev.owner_pid != pid {
                        return Err(-1); // EPERM
                    }
                } else {
                    return Err(-2);
                }

                if buf.len() < 4 {
                    return Err(-22);
                }

                let val = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                match super::pci::config_write32(bdf, offset, val) {
                    Ok(()) => Ok(4),
                    Err(_) => Err(-5), // EIO
                }
            }

            PcieHandleType::Bar => {
                // Write to BAR handle maps it and returns vaddr
                // Requires ownership
                if let Some(dev) = super::pci::find_by_bdf(bdf) {
                    if dev.owner_pid != pid {
                        return Err(-1); // EPERM
                    }
                } else {
                    return Err(-2);
                }

                // Get BAR info
                let (phys_addr, size) = match super::pci::bar_info(bdf, bar) {
                    Ok(info) => info,
                    Err(_) => return Err(-22),
                };

                if size == 0 {
                    return Err(-22);
                }

                // Map into process address space
                unsafe {
                    let sched = super::task::scheduler();
                    for task_opt in sched.tasks.iter_mut() {
                        if let Some(ref mut task) = task_opt {
                            if task.id == pid {
                                match task.mmap_phys(phys_addr, size as usize) {
                                    Some(_vaddr) => return Ok(buf.len()),
                                    None => return Err(-12), // ENOMEM
                                }
                            }
                        }
                    }
                }
                Err(-3) // ESRCH
            }

            PcieHandleType::Msi => {
                // Write count to allocate MSI vectors
                if let Some(dev) = super::pci::find_by_bdf(bdf) {
                    if dev.owner_pid != pid {
                        return Err(-1);
                    }
                } else {
                    return Err(-2);
                }

                if buf.is_empty() {
                    return Err(-22);
                }

                let count = buf[0];
                match super::pci::msi_alloc(bdf, count) {
                    Ok(_irq) => Ok(1),
                    Err(super::pci::PciError::NoMsiVectors) => Err(-12),
                    Err(_) => Err(-5),
                }
            }

            PcieHandleType::Claim => {
                // Write to claim device
                match super::pci::claim_device(bdf, pid) {
                    Ok(()) => Ok(buf.len()),
                    Err(super::pci::PciError::NotFound) => Err(-2),
                    Err(super::pci::PciError::PermissionDenied) => Err(-1),
                    Err(_) => Err(-5),
                }
            }
        }
    }

    fn close(&self, handle: &SchemeHandle) -> Result<(), i32> {
        let (handle_type, _bar, _bdf, _offset) = Self::decode_handle(handle.handle);

        // If closing a claimed device, we could optionally release ownership
        // For now, ownership persists until process exit
        if handle_type == PcieHandleType::Claim {
            // Could call pci::release_device here if desired
        }

        Ok(())
    }
}

// ============================================================================
// Global scheme instances
// ============================================================================

static MEMORY_SCHEME: MemoryScheme = MemoryScheme;
static TIME_SCHEME: TimeScheme = TimeScheme;
static IRQ_SCHEME: IrqScheme = IrqScheme;
static NULL_SCHEME: NullScheme = NullScheme;
static ZERO_SCHEME: ZeroScheme = ZeroScheme;
static CONSOLE_SCHEME: ConsoleScheme = ConsoleScheme;
static MMIO_SCHEME: MmioScheme = MmioScheme;
static I2C_SCHEME: I2cScheme = I2cScheme;
static PCIE_SCHEME: PcieScheme = PcieScheme;

/// Kernel scheme IDs (must match registration order in init())
pub mod scheme_ids {
    pub const MEMORY: u16 = 0;
    pub const TIME: u16 = 1;
    pub const IRQ: u16 = 2;
    pub const NULL: u16 = 3;
    pub const ZERO: u16 = 4;
    pub const CONSOLE: u16 = 5;
    pub const MMIO: u16 = 6;
    pub const I2C: u16 = 7;
    pub const PCIE: u16 = 8;
}

/// Get kernel scheme by name
pub fn get_kernel_scheme(name: &str) -> Option<&'static dyn KernelScheme> {
    match name {
        "memory" => Some(&MEMORY_SCHEME),
        "time" => Some(&TIME_SCHEME),
        "irq" => Some(&IRQ_SCHEME),
        "null" => Some(&NULL_SCHEME),
        "zero" => Some(&ZERO_SCHEME),
        "console" => Some(&CONSOLE_SCHEME),
        "mmio" => Some(&MMIO_SCHEME),
        "i2c" => Some(&I2C_SCHEME),
        "pcie" => Some(&PCIE_SCHEME),
        _ => None,
    }
}

/// Get kernel scheme by ID
pub fn get_kernel_scheme_by_id(id: u16) -> Option<&'static dyn KernelScheme> {
    match id {
        scheme_ids::MEMORY => Some(&MEMORY_SCHEME),
        scheme_ids::TIME => Some(&TIME_SCHEME),
        scheme_ids::IRQ => Some(&IRQ_SCHEME),
        scheme_ids::NULL => Some(&NULL_SCHEME),
        scheme_ids::ZERO => Some(&ZERO_SCHEME),
        scheme_ids::CONSOLE => Some(&CONSOLE_SCHEME),
        scheme_ids::MMIO => Some(&MMIO_SCHEME),
        scheme_ids::I2C => Some(&I2C_SCHEME),
        scheme_ids::PCIE => Some(&PCIE_SCHEME),
        _ => None,
    }
}

/// Close a kernel scheme handle by ID
/// Called when a file descriptor is closed
pub fn close_kernel_scheme(scheme_id: u16, handle: u64, flags: u32) {
    if let Some(scheme) = get_kernel_scheme_by_id(scheme_id) {
        let scheme_handle = SchemeHandle {
            scheme_id,
            handle,
            flags,
        };
        // Ignore errors on close - process is cleaning up anyway
        let _ = scheme.close(&scheme_handle);
    }
}

// ============================================================================
// Path parsing and scheme routing
// ============================================================================

/// Parse a scheme:path URL
/// Returns (scheme_name, path) or None if invalid
pub fn parse_url(url: &[u8]) -> Option<(&str, &str)> {
    let url_str = core::str::from_utf8(url).ok()?;

    // Find the colon separator
    let colon_pos = url_str.find(':')?;

    if colon_pos == 0 {
        return None; // Empty scheme name
    }

    let scheme = &url_str[..colon_pos];
    let path = if colon_pos + 1 < url_str.len() {
        &url_str[colon_pos + 1..]
    } else {
        ""
    };

    Some((scheme, path))
}

/// Open a scheme URL
/// Returns FdEntry on success
pub fn open_url(url: &[u8], flags: u32) -> Result<(FdEntry, SchemeHandle), i32> {
    let (scheme_name, path) = parse_url(url).ok_or(-22)?; // EINVAL

    // Check if it's a kernel scheme
    if let Some(scheme) = get_kernel_scheme(scheme_name) {
        let mut handle = scheme.open(path.as_bytes(), flags)?;

        // Find scheme ID
        unsafe {
            if let Some(entry) = registry().find(scheme_name) {
                handle.scheme_id = entry.id;
            }
        }

        // Create FD entry with scheme type
        let fd_entry = FdEntry {
            fd_type: FdType::Scheme {
                scheme_id: handle.scheme_id,
                handle: handle.handle,
                scheme_flags: handle.flags,
            },
            flags: FdFlags {
                readable: true,
                writable: (flags & 0x3) != 0, // O_WRONLY or O_RDWR
                nonblocking: (flags & 0x800) != 0,
            },
            offset: 0,
        };

        return Ok((fd_entry, handle));
    }

    // Check user schemes
    unsafe {
        if let Some(entry) = registry().find(scheme_name) {
            if entry.scheme_type == SchemeType::User {
                // Forward to user daemon via IPC
                return open_user_scheme(entry, path, flags);
            }
        }
    }

    Err(-2) // ENOENT - scheme not found
}

/// Open a user scheme - sends request to daemon via IPC
fn open_user_scheme(
    entry: &SchemeEntry,
    path: &str,
    flags: u32,
) -> Result<(FdEntry, SchemeHandle), i32> {
    use super::ipc::{self, Message, MessageHeader, MessageType, MAX_INLINE_PAYLOAD, waker, WakeReason};

    let daemon_channel = entry.channel_id;
    let daemon_pid = entry.owner_pid;

    // Get caller PID
    let caller_pid = unsafe {
        let sched = super::task::scheduler();
        sched.tasks[sched.current]
            .as_ref()
            .map(|t| t.id)
            .ok_or(-1)?
    };

    // Create a channel pair for this connection
    let (client_ch, server_ch) = ipc::create_channel_pair(caller_pid, daemon_pid)
        .map_err(|_| -12i32)?; // ENOMEM

    // Build open request message
    // Format: server_ch (4 bytes) + flags (4 bytes) + path (remaining bytes)
    // The server_ch is included so the daemon knows where to send responses
    let mut payload = [0u8; MAX_INLINE_PAYLOAD];
    payload[0..4].copy_from_slice(&server_ch.to_le_bytes());
    payload[4..8].copy_from_slice(&flags.to_le_bytes());
    let path_bytes = path.as_bytes();
    let path_len = core::cmp::min(path_bytes.len(), MAX_INLINE_PAYLOAD - 8);
    payload[8..8 + path_len].copy_from_slice(&path_bytes[..path_len]);

    // Create connect message
    let msg = Message {
        header: MessageHeader {
            msg_type: MessageType::Connect,
            sender: caller_pid,
            msg_id: server_ch, // Also in payload for easier access
            payload_len: (8 + path_len) as u32,
            flags: 0,
        },
        payload,
    };

    // Get peer info before sending
    let peer_owner = ipc::get_peer_owner_unchecked(daemon_channel);
    let peer_channel = ipc::get_peer_id(daemon_channel);

    // Send to daemon's main channel (use unchecked send since caller may not own channel)
    let wake_list = ipc::send_unchecked(daemon_channel, msg)
        .map_err(|e| e.to_errno() as i32)?;

    // Wake subscribers from the send
    waker::wake(&wake_list, WakeReason::Readable);

    // Push IpcReady event to daemon's event queue (for event-driven processes)
    if let (Some(peer_pid), Some(peer_ch)) = (peer_owner, peer_channel) {
        if peer_pid != 0 {
            let _guard = crate::arch::aarch64::sync::IrqGuard::new();
            unsafe {
                let sched = super::task::scheduler();
                if let Some(slot) = sched.slot_by_pid(peer_pid) {
                    if let Some(ref mut task) = sched.tasks[slot] {
                        let event = super::event::Event::ipc_ready(peer_ch, caller_pid);
                        if task.event_queue.is_subscribed(&event) {
                            task.event_queue.push(event);
                            // Wake task using unified wake function
                            sched.wake_task(slot);
                        }
                    }
                }
            }
        }
    }

    // For non-blocking operation, we return immediately with the client channel
    // The daemon will process the request and send a response
    // For full blocking, we'd wait for a response here

    // Create FD entry that wraps the channel
    let fd_entry = FdEntry {
        fd_type: FdType::Channel(client_ch),
        flags: FdFlags {
            readable: true,
            writable: (flags & 0x3) != 0,
            nonblocking: (flags & 0x800) != 0,
        },
        offset: 0,
    };

    let handle = SchemeHandle {
        scheme_id: entry.id,
        handle: client_ch as u64,
        flags,
    };

    Ok((fd_entry, handle))
}

/// Initialize kernel schemes
pub fn init() {
    unsafe {
        let reg = registry();
        reg.register_kernel("memory");   // ID 0
        reg.register_kernel("time");     // ID 1
        reg.register_kernel("irq");      // ID 2
        reg.register_kernel("null");     // ID 3
        reg.register_kernel("zero");     // ID 4
        reg.register_kernel("console");  // ID 5
        reg.register_kernel("mmio");     // ID 6
        reg.register_kernel("i2c");      // ID 7
        reg.register_kernel("pcie");     // ID 8
    }
    kinfo!("scheme", "init_ok"; schemes = "memory,time,irq,null,zero,console,mmio,i2c,pcie");
}

/// Test scheme system
pub fn test() {
    print_direct!("  Testing scheme system...\n");

    // Test URL parsing
    let url = b"memory:info";
    if let Some((scheme, path)) = parse_url(url) {
        print_direct!("    Parsed 'memory:info' -> scheme='{}', path='{}'\n", scheme, path);
    }

    // Test memory scheme
    if let Some(scheme) = get_kernel_scheme("memory") {
        print_direct!("    Found memory scheme\n");
        if let Ok(handle) = scheme.open(b"info", 0) {
            print_direct!("    Opened memory:info, handle={}\n", handle.handle);
            let mut buf = [0u8; 64];
            if let Ok(n) = scheme.read(&handle, &mut buf) {
                let s = core::str::from_utf8(&buf[..n]).unwrap_or("");
                print_direct!("    Read {} bytes: {}\n", n, s.trim());
            }
        }
    }

    // Test time scheme
    if let Some(scheme) = get_kernel_scheme("time") {
        if let Ok(handle) = scheme.open(b"now", 0) {
            let mut buf = [0u8; 8];
            if let Ok(_) = scheme.read(&handle, &mut buf) {
                let ns = u64::from_le_bytes(buf);
                print_direct!("    time:now = {} ns\n", ns);
            }
        }
    }

    // List registered schemes
    print_direct!("    Registered schemes:\n");
    unsafe {
        for entry in registry().list() {
            print_direct!("      {}: ({:?})\n", entry.name_str(), entry.scheme_type);
        }
    }

    print_direct!("    [OK] Scheme system test passed\n");
}
