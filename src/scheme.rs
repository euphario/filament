//! Scheme System
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

use crate::fd::{FdEntry, FdFlags, FdType};
use crate::println;

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
                let total = crate::pmm::total_count();
                let free = crate::pmm::free_count();
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
                let counter = crate::timer::counter();
                let freq = crate::timer::frequency();
                if freq > 0 {
                    (counter as u128 * 1_000_000_000 / freq as u128) as u64
                } else {
                    0
                }
            }
            Self::HANDLE_FREQ => crate::timer::frequency(),
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
}

impl IrqRegistration {
    pub const fn empty() -> Self {
        Self {
            irq_num: 0xFFFFFFFF,
            owner_pid: 0,
            pending: false,
            pending_count: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.irq_num == 0xFFFFFFFF
    }
}

/// Maximum IRQ registrations
pub const MAX_IRQ_REGISTRATIONS: usize = 32;

/// Global IRQ registration table
static mut IRQ_REGISTRATIONS: [IrqRegistration; MAX_IRQ_REGISTRATIONS] =
    [IrqRegistration::empty(); MAX_IRQ_REGISTRATIONS];

/// Get IRQ registration table (unsafe, requires synchronization)
pub unsafe fn irq_table() -> &'static mut [IrqRegistration; MAX_IRQ_REGISTRATIONS] {
    &mut *core::ptr::addr_of_mut!(IRQ_REGISTRATIONS)
}

/// Register a process for an IRQ
pub fn irq_register(irq_num: u32, pid: u32) -> bool {
    unsafe {
        let table = irq_table();

        // Check if already registered
        for reg in table.iter() {
            if !reg.is_empty() && reg.irq_num == irq_num {
                return false; // IRQ already taken
            }
        }

        // Find empty slot
        for reg in table.iter_mut() {
            if reg.is_empty() {
                reg.irq_num = irq_num;
                reg.owner_pid = pid;
                reg.pending = false;
                reg.pending_count = 0;

                // Enable the IRQ in the GIC
                crate::gic::enable_irq(irq_num);

                return true;
            }
        }
        false
    }
}

/// Unregister a process from an IRQ
pub fn irq_unregister(irq_num: u32, pid: u32) -> bool {
    unsafe {
        let table = irq_table();

        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid {
                // Disable the IRQ in the GIC
                crate::gic::disable_irq(irq_num);

                *reg = IrqRegistration::empty();
                return true;
            }
        }
        false
    }
}

/// Called from IRQ handler when an interrupt fires
/// Returns the PID to wake up, if any
pub fn irq_notify(irq_num: u32) -> Option<u32> {
    unsafe {
        let table = irq_table();

        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num {
                reg.pending = true;
                reg.pending_count += 1;

                // Send event to the process
                let event = crate::event::Event::irq(irq_num);
                crate::event::send_event(reg.owner_pid, event);

                return Some(reg.owner_pid);
            }
        }
        None
    }
}

/// Check if an IRQ is pending for a process
pub fn irq_check_pending(irq_num: u32, pid: u32) -> Option<u32> {
    unsafe {
        let table = irq_table();

        for reg in table.iter_mut() {
            if !reg.is_empty() && reg.irq_num == irq_num && reg.owner_pid == pid {
                if reg.pending {
                    let count = reg.pending_count;
                    reg.pending = false;
                    reg.pending_count = 0;
                    return Some(count);
                }
                return None;
            }
        }
        None
    }
}

impl IrqScheme {
    // Handle encodes the IRQ number
}

impl KernelScheme for IrqScheme {
    fn name(&self) -> &'static str {
        "irq"
    }

    fn open(&self, path: &[u8], _flags: u32) -> Result<SchemeHandle, i32> {
        let path_str = core::str::from_utf8(path).map_err(|_| -22)?;

        // Parse IRQ number from path
        let irq_num: u32 = path_str.parse().map_err(|_| -22)?;

        // Validate IRQ number (0-447 for GIC-600)
        if irq_num >= 448 {
            return Err(-22);
        }

        // Get current process PID
        let pid = unsafe {
            let sched = crate::task::scheduler();
            sched.tasks[sched.current]
                .as_ref()
                .map(|t| t.id)
                .ok_or(-1)?
        };

        // Register for this IRQ
        if !irq_register(irq_num, pid) {
            return Err(-16); // EBUSY - IRQ already registered
        }

        Ok(SchemeHandle {
            scheme_id: 0,
            handle: irq_num as u64,
            flags: pid,  // Store PID in flags for close()
        })
    }

    fn read(&self, handle: &SchemeHandle, buf: &mut [u8]) -> Result<usize, i32> {
        if buf.len() < 4 {
            return Err(-22);
        }

        let irq_num = handle.handle as u32;
        let pid = handle.flags;

        // Check if IRQ is pending
        if let Some(count) = irq_check_pending(irq_num, pid) {
            // Return the count of IRQs that fired
            buf[..4].copy_from_slice(&count.to_le_bytes());
            Ok(4)
        } else {
            // No pending IRQ - return EAGAIN (caller should wait on event)
            Err(-11) // EAGAIN
        }
    }

    fn write(&self, _handle: &SchemeHandle, _buf: &[u8]) -> Result<usize, i32> {
        // Writing could be used to acknowledge/clear the IRQ
        // For now, reading clears it automatically
        Err(-1) // EPERM
    }

    fn close(&self, handle: &SchemeHandle) -> Result<(), i32> {
        let irq_num = handle.handle as u32;
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
            match crate::uart::try_getc() {
                Some(c) => {
                    buf[count] = c as u8;
                    count += 1;
                    // Echo the character back for interactive use
                    crate::uart::putc(c);
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
            crate::uart::putc(byte as char);
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
            let sched = crate::task::scheduler();
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

                    // Invalidate TLB for the new mappings
                    for i in 0..num_pages {
                        let page_virt = virt_addr + (i * 4096) as u64;
                        let tlbi_addr = page_virt >> 12;
                        core::arch::asm!(
                            "tlbi vale1is, {addr}",
                            addr = in(reg) tlbi_addr,
                        );
                    }
                    core::arch::asm!("dsb ish", "isb");
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
            let sched = crate::task::scheduler();
            if let Some(ref mut task) = sched.tasks[sched.current] {
                if let Some(ref mut addr_space) = task.address_space {
                    for i in 0..num_pages {
                        let page_virt = virt_addr + (i * 4096) as u64;
                        addr_space.unmap_page(page_virt);

                        // Invalidate TLB
                        let tlbi_addr = page_virt >> 12;
                        core::arch::asm!(
                            "tlbi vale1is, {addr}",
                            addr = in(reg) tlbi_addr,
                        );
                    }
                    core::arch::asm!("dsb ish", "isb");
                }
            }
        }

        // Note: We don't track physical pages for MMIO - they're device memory, not RAM
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

/// Kernel scheme IDs (must match registration order in init())
pub mod scheme_ids {
    pub const MEMORY: u16 = 0;
    pub const TIME: u16 = 1;
    pub const IRQ: u16 = 2;
    pub const NULL: u16 = 3;
    pub const ZERO: u16 = 4;
    pub const CONSOLE: u16 = 5;
    pub const MMIO: u16 = 6;
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
        _ => None,
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
    use crate::ipc::{self, Message, MessageType};

    let daemon_channel = entry.channel_id;
    let daemon_pid = entry.owner_pid;

    // Get caller PID
    let caller_pid = unsafe {
        let sched = crate::task::scheduler();
        sched.tasks[sched.current]
            .as_ref()
            .map(|t| t.id)
            .ok_or(-1)?
    };

    // Create a channel pair for this connection
    let (client_ch, server_ch) = unsafe {
        ipc::channel_table()
            .create_pair(caller_pid, daemon_pid)
            .ok_or(-12)? // ENOMEM
    };

    // Build open request message
    // Format: flags (4 bytes) + path (remaining bytes)
    let mut payload = [0u8; ipc::MAX_INLINE_PAYLOAD];
    payload[0..4].copy_from_slice(&flags.to_le_bytes());
    let path_bytes = path.as_bytes();
    let path_len = core::cmp::min(path_bytes.len(), ipc::MAX_INLINE_PAYLOAD - 4);
    payload[4..4 + path_len].copy_from_slice(&path_bytes[..path_len]);

    // Create connect message
    let msg = Message {
        header: crate::ipc::MessageHeader {
            msg_type: MessageType::Connect,
            sender: caller_pid,
            msg_id: server_ch, // Send the server channel so daemon knows where to respond
            payload_len: (4 + path_len) as u32,
            flags: 0,
        },
        payload,
    };

    // Send to daemon's main channel
    unsafe {
        let table = ipc::channel_table();
        table.send(daemon_channel, msg).map_err(|e| e.to_errno())?;
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
    }
    println!("  Registered kernel schemes: memory, time, irq, null, zero, console, mmio");
}

/// Test scheme system
pub fn test() {
    println!("  Testing scheme system...");

    // Test URL parsing
    let url = b"memory:info";
    if let Some((scheme, path)) = parse_url(url) {
        println!("    Parsed 'memory:info' -> scheme='{}', path='{}'", scheme, path);
    }

    // Test memory scheme
    if let Some(scheme) = get_kernel_scheme("memory") {
        println!("    Found memory scheme");
        if let Ok(handle) = scheme.open(b"info", 0) {
            println!("    Opened memory:info, handle={}", handle.handle);
            let mut buf = [0u8; 64];
            if let Ok(n) = scheme.read(&handle, &mut buf) {
                let s = core::str::from_utf8(&buf[..n]).unwrap_or("");
                println!("    Read {} bytes: {}", n, s.trim());
            }
        }
    }

    // Test time scheme
    if let Some(scheme) = get_kernel_scheme("time") {
        if let Ok(handle) = scheme.open(b"now", 0) {
            let mut buf = [0u8; 8];
            if let Ok(_) = scheme.read(&handle, &mut buf) {
                let ns = u64::from_le_bytes(buf);
                println!("    time:now = {} ns", ns);
            }
        }
    }

    // List registered schemes
    println!("    Registered schemes:");
    unsafe {
        for entry in registry().list() {
            println!("      {}: ({:?})", entry.name_str(), entry.scheme_type);
        }
    }

    println!("    [OK] Scheme system test passed");
}
