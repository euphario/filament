//! System call wrappers - Unified 5-syscall interface
//!
//! Only the minimal syscalls needed for the unified object model.
//! Types and constants are imported from the shared `abi` crate.

use core::arch::asm;
use crate::error::{SysError, SysResult};

// Re-export types from abi crate for convenience
pub use abi::{ProcessInfo, ProcessInfoEx, SysInfo, ObjectType, Handle, LogLevel, PciEnumEntry, PendingSignal};
pub use abi::{syscall as syscall_num, log_level, liveness_status, prot, errno, signal_event};

// Local aliases for syscall numbers (shorter names for internal use)
use abi::syscall as sys;

// ============================================================================
// Raw Syscall Functions
// ============================================================================

#[inline(always)]
fn syscall0(num: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            lateout("x0") ret,
            lateout("x9") _,
            lateout("x10") _,
            lateout("x11") _,
            lateout("x12") _,
            lateout("x13") _,
            lateout("x14") _,
            lateout("x15") _,
            options(nostack)
        );
    }
    ret
}

#[inline(always)]
fn syscall1(num: u64, a0: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            inlateout("x0") a0 => ret,
            lateout("x9") _,
            lateout("x10") _,
            lateout("x11") _,
            lateout("x12") _,
            lateout("x13") _,
            lateout("x14") _,
            lateout("x15") _,
            options(nostack)
        );
    }
    ret
}

#[inline(always)]
fn syscall2(num: u64, a0: u64, a1: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            inlateout("x0") a0 => ret,
            in("x1") a1,
            lateout("x9") _,
            lateout("x10") _,
            lateout("x11") _,
            lateout("x12") _,
            lateout("x13") _,
            lateout("x14") _,
            lateout("x15") _,
            options(nostack)
        );
    }
    ret
}

#[inline(always)]
pub(crate) fn syscall3(num: u64, a0: u64, a1: u64, a2: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            inlateout("x0") a0 => ret,
            in("x1") a1,
            in("x2") a2,
            lateout("x9") _,
            lateout("x10") _,
            lateout("x11") _,
            lateout("x12") _,
            lateout("x13") _,
            lateout("x14") _,
            lateout("x15") _,
            options(nostack)
        );
    }
    ret
}

#[inline(always)]
fn syscall5(num: u64, a0: u64, a1: u64, a2: u64, a3: u64, a4: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            inlateout("x0") a0 => ret,
            in("x1") a1,
            in("x2") a2,
            in("x3") a3,
            in("x4") a4,
            lateout("x9") _,
            lateout("x10") _,
            lateout("x11") _,
            lateout("x12") _,
            lateout("x13") _,
            lateout("x14") _,
            lateout("x15") _,
            options(nostack)
        );
    }
    ret
}

// ============================================================================
// Process Lifecycle
// ============================================================================

/// Exit the current process
pub fn exit(code: i32) -> ! {
    syscall1(sys::EXIT, code as u64);
    loop { unsafe { asm!("wfe") }; }
}

/// Execute a program from initrd
pub fn exec(path: &str) -> i64 {
    syscall2(sys::EXEC, path.as_ptr() as u64, path.len() as u64)
}

/// Spawn a child with a shared mailbox page.
///
/// `mailbox_data` is copied into a 4KB shmem page mapped at Handle::MAILBOX in the child.
/// Returns `Ok((child_pid, parent_mailbox_handle, parent_superq_handle))` on success.
/// The mailbox handle points to the same 4KB shmem page, allowing post-spawn writes.
/// The supervision handle is the parent end of the SupervisionQueue for reliable note delivery.
pub fn exec_with_mailbox(path: &str, caps: u64, mailbox: &[u8]) -> Result<(u32, Handle, Handle), i64> {
    let result = syscall5(sys::EXEC_WITH_MAILBOX, path.as_ptr() as u64, path.len() as u64, caps, mailbox.as_ptr() as u64, mailbox.len() as u64);
    if result < 0 {
        Err(result)
    } else {
        // Unpack: bits 0-15 = child_pid, bits 16-31 = shmem packed, bits 32-47 = superq packed
        // Packed handle format: (gen:8 << 8) | (index:8) → expand to Handle raw: (gen << 24) | index
        let child_pid = (result & 0xFFFF) as u32;
        let shmem_packed = ((result >> 16) & 0xFFFF) as u32;
        let superq_packed = ((result >> 32) & 0xFFFF) as u32;
        let shmem_raw = ((shmem_packed & 0xFF00) << 16) | (shmem_packed & 0xFF);
        let superq_raw = ((superq_packed & 0xFF00) << 16) | (superq_packed & 0xFF);
        Ok((child_pid, Handle::from_raw(shmem_raw), Handle::from_raw(superq_raw)))
    }
}

/// Get current process ID
pub fn getpid() -> u32 {
    syscall0(sys::GETPID) as u32
}

/// Wait for a child process to exit
pub fn wait(pid: i32) -> i64 {
    syscall3(sys::WAIT, pid as u64, 0, 0)
}

/// Kill a process by PID
pub fn kill(pid: u32) -> i64 {
    syscall1(sys::KILL, pid as u64)
}

/// Send an async signal to another task
pub fn signal(target_pid: u32, event: u32, value: u64) -> i64 {
    syscall3(sys::SIGNAL, target_pid as u64, event as u64, value)
}

/// Peek at pending signals for a target process (non-consuming)
pub fn signal_peek(pid: u32, buf: &mut [abi::PendingSignal]) -> i64 {
    syscall3(sys::SIGNAL_PEEK, pid as u64, buf.as_mut_ptr() as u64, buf.len() as u64)
}

/// Flush (clear) all pending signals for a target process
pub fn signal_flush(pid: u32) -> i64 {
    syscall1(sys::SIGNAL_FLUSH, pid as u64)
}

/// Reset per-task statistics counters (0 = all tasks)
pub fn reset_stats(pid: u32) -> i64 {
    syscall1(sys::RESET_STATS, pid as u64)
}

// ============================================================================
// Memory
// ============================================================================

/// Allocate memory pages
pub fn mmap(addr: u64, size: usize, prot: u32) -> i64 {
    syscall3(sys::MMAP, addr, size as u64, prot as u64)
}

/// Free memory pages
pub fn munmap(addr: u64, size: usize) -> i64 {
    syscall2(sys::MUNMAP, addr, size as u64)
}

pub const PROT_READ: u32 = abi::prot::READ;
pub const PROT_WRITE: u32 = abi::prot::WRITE;
pub const PROT_EXEC: u32 = abi::prot::EXEC;

// ============================================================================
// Time
// ============================================================================

/// Get current time in nanoseconds since boot
pub fn gettime() -> u64 {
    syscall0(sys::GETTIME) as u64
}

/// Sleep for duration in nanoseconds
///
/// Light oneshot timer - task sleeps for the specified duration.
/// More efficient than Timer object for simple delays.
///
/// Note: The kernel's SLEEP syscall expects a duration, not an absolute deadline.
/// It internally computes deadline = now + duration.
pub fn sleep_ns(ns: u64) {
    syscall1(sys::SLEEP, ns);
}

/// Sleep for duration in microseconds
pub fn sleep_us(us: u32) {
    sleep_ns(us as u64 * 1_000);
}

/// Get caller's base and effective priority
/// Returns: (base_priority, effective_priority)
pub fn get_priority() -> (u8, u8) {
    let raw = syscall0(sys::GET_PRIORITY);
    ((raw & 0xFF) as u8, ((raw >> 8) & 0xFF) as u8)
}

/// Sleep for duration in milliseconds
pub fn sleep_ms(ms: u32) {
    sleep_ns(ms as u64 * 1_000_000);
}

// ============================================================================
// Logging (kernel buffer)
// ============================================================================

/// Write formatted text log message to kernel.
///
/// **Do not call directly** — use the structured logging macros instead:
/// `uinfo!`, `uerror!`, `uwarn!`, `udebug!`, `utrace!`
///
/// Those macros produce binary records via `klog_write`, preserving
/// structured key-value data through the entire pipeline.
pub fn klog(level: LogLevel, msg: &[u8]) {
    let _ = syscall3(sys::KLOG, level as u64, msg.as_ptr() as u64, msg.len() as u64);
}

/// Write a binary log record into the kernel log ring.
///
/// The record must be in the standard binary format produced by
/// `ulog::RecordBuilder`. Structure is preserved end-to-end.
pub fn klog_write(record: &[u8]) -> i64 {
    syscall2(sys::KLOG_WRITE, record.as_ptr() as u64, record.len() as u64)
}

/// Read one formatted log record from the kernel log ring.
///
/// Returns the number of bytes written to `buf`, or 0 if no logs available.
pub fn klog_read(buf: &mut [u8]) -> i64 {
    syscall2(sys::KLOG_READ, buf.as_mut_ptr() as u64, buf.len() as u64)
}

/// Set console output log level
pub fn set_console_level(level: u8) -> i64 {
    syscall1(sys::SET_CONSOLE_LEVEL, level as u64)
}

/// Set per-module log level override (level=0xFF removes override)
pub fn set_module_level(subsys: &[u8], level: u8) -> i64 {
    syscall3(sys::SET_MODULE_LEVEL, subsys.as_ptr() as u64, subsys.len() as u64, level as u64)
}

/// Register exception channel on a child task.
/// When child faults, kernel sends ExceptionInfo on this channel instead of killing.
pub fn set_exception_channel(child_pid: u32, channel_handle: Handle) -> SysResult<()> {
    let ret = syscall2(sys::SET_EXCEPTION_CHANNEL, child_pid as u64, channel_handle.0 as u64);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(())
    }
}

/// Resume or kill a frozen (faulted) child task.
/// action: 0=resume, 1=kill
pub fn exception_resume(child_pid: u32, action: u32) -> SysResult<()> {
    let ret = syscall2(sys::EXCEPTION_RESUME, child_pid as u64, action as u64);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(())
    }
}

/// Set per-task resource limits on a child process.
/// Caller must be parent of child_pid.
pub fn set_resource_limits(child_pid: u32, max_channels: u16, max_ports: u16, max_shmem: u16, max_children: u16) -> SysResult<()> {
    let ret = syscall5(sys::SET_RESOURCE_LIMITS, child_pid as u64, max_channels as u64, max_ports as u64, max_shmem as u64, max_children as u64);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(())
    }
}

/// Non-destructive read from kernel log ring at a cursor position with filters.
///
/// The `cursor` is read and updated: pass 0 to start from oldest record.
/// `from_ms`: only return records with ts_ms >= this value (0 = no filter).
/// `max_level`: only return records with level <= this value (0xFF = no filter).
/// Returns the number of formatted text bytes, or 0 if caught up.
/// The formatted text is written to `buf[4..]`, cursor to `buf[0..4]`.
pub fn klog_read_at(buf: &mut [u8], cursor: &mut u32) -> i64 {
    klog_read_filtered(buf, cursor, 0, 0xFF)
}

/// Non-destructive read with timestamp and level filters applied in-kernel.
pub fn klog_read_filtered(buf: &mut [u8], cursor: &mut u32, from_ms: u32, max_level: u8) -> i64 {
    if buf.len() < 13 {
        return -22; // EINVAL: need 9-byte header + at least len(2) + 1 byte
    }
    // Write 9-byte header: cursor(4) + from_ms(4) + max_level(1)
    buf[0..4].copy_from_slice(&cursor.to_le_bytes());
    buf[4..8].copy_from_slice(&from_ms.to_le_bytes());
    buf[8] = max_level;

    // Signal cursor mode with bit 31 set on length
    let len_with_flag = buf.len() | 0x80000000;
    let ret = syscall2(sys::KLOG_READ, buf.as_mut_ptr() as u64, len_with_flag as u64);

    if ret > 0 {
        // Read back updated cursor from first 4 bytes
        *cursor = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    }
    ret
}

/// Raw debug write to UART (bypasses klog buffer)
pub fn debug_write(msg: &[u8]) {
    let _ = syscall2(sys::DEBUG_WRITE, msg.as_ptr() as u64, msg.len() as u64);
}

// ============================================================================
// Debug / Admin (consider moving to services)
// ============================================================================

/// Get process list info (debug)
pub fn ps_info(buf: &mut [ProcessInfo]) -> usize {
    let ret = syscall2(sys::PS_INFO, buf.as_mut_ptr() as u64, buf.len() as u64);
    if ret < 0 { 0 } else { ret as usize }
}

/// Get extended process list info with resource accounting
pub fn ps_info_ex(buf: &mut [ProcessInfoEx]) -> usize {
    let ret = syscall2(sys::PS_INFO_EX, buf.as_mut_ptr() as u64, buf.len() as u64);
    if ret < 0 { 0 } else { ret as usize }
}

/// Set kernel log level
pub fn set_log_level(level: u8) -> i64 {
    syscall1(sys::SET_LOG_LEVEL, level as u64)
}

/// Get system-wide information (memory, tasks, uptime)
pub fn sysinfo(info: &mut SysInfo) -> i64 {
    syscall1(sys::SYSINFO, info as *mut SysInfo as u64)
}

/// Reset the system
pub fn reset() -> i64 {
    syscall0(sys::RESET)
}

/// Shutdown the system (PSCI SYSTEM_OFF on QEMU)
///
/// On QEMU, this causes the emulator to exit. On real hardware,
/// falls back to reset. Requires ALL capabilities.
pub fn shutdown(exit_code: u8) -> i64 {
    syscall1(sys::SHUTDOWN, exit_code as u64)
}

/// Trigger a kernel panic for debugging exception dump output.
/// Requires KILL capability.
pub fn kernel_panic() -> i64 {
    syscall0(sys::KERNEL_PANIC)
}


// ============================================================================
// Unified Object Interface (the 5 syscalls)
// ============================================================================

/// Alias for Handle (backward compatibility)
pub type ObjHandle = Handle;

/// Open a new object handle
pub fn open(obj_type: ObjectType, params: &[u8]) -> SysResult<Handle> {
    let ret = syscall3(
        sys::OPEN,
        obj_type as u64,
        params.as_ptr() as u64,
        params.len() as u64,
    );
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(Handle(ret as u32))
    }
}

/// Open a new shmem region and return (handle, shmem_id)
/// The kernel returns shmem_id in upper 32 bits, handle in lower 32 bits
pub fn open_shmem_create(size: usize) -> SysResult<(Handle, u32)> {
    let ret = syscall3(
        sys::OPEN,
        ObjectType::Shmem as u64,
        (&(size as u64).to_le_bytes()).as_ptr() as u64,
        8, // size is u64 = 8 bytes
    );
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        let handle = Handle((ret & 0xFFFF_FFFF) as u32);
        let shmem_id = ((ret >> 32) & 0xFFFF_FFFF) as u32;
        Ok((handle, shmem_id))
    }
}

/// Read from a handle
pub fn read(handle: Handle, buf: &mut [u8]) -> SysResult<usize> {
    let ret = syscall3(
        sys::READ,
        handle.0 as u64,
        buf.as_mut_ptr() as u64,
        buf.len() as u64,
    );
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(ret as usize)
    }
}

/// Try to read from a handle without blocking.
/// Returns Ok(Some(n)) if data was read, Ok(None) if nothing available (WouldBlock).
pub fn try_read(handle: Handle, buf: &mut [u8]) -> SysResult<Option<usize>> {
    match read(handle, buf) {
        Ok(n) => Ok(Some(n)),
        Err(SysError::WouldBlock) => Ok(None),
        Err(e) => Err(e),
    }
}

/// Write to a handle
pub fn write(handle: Handle, buf: &[u8]) -> SysResult<usize> {
    let ret = syscall3(
        sys::WRITE,
        handle.0 as u64,
        buf.as_ptr() as u64,
        buf.len() as u64,
    );
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(ret as usize)
    }
}

/// Map a handle into address space (for Shmem, DmaPool, Mmio)
///
/// Returns the virtual address on success. Address 0 is never valid
/// (user heap starts at 0x50000000) and is treated as an error.
pub fn map(handle: Handle, flags: u32) -> SysResult<u64> {
    let ret = syscall2(sys::MAP, handle.0 as u64, flags as u64);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else if ret == 0 {
        // Address 0 is never a valid mapping — treat as failure
        Err(SysError::from_errno(-12)) // ENOMEM
    } else {
        Ok(ret as u64)
    }
}

/// Close a handle
pub fn close(handle: Handle) -> SysResult<()> {
    let ret = syscall1(sys::CLOSE, handle.0 as u64);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(())
    }
}

// ============================================================================
// PCI Enumeration
// ============================================================================

/// Query PCI devices from kernel registry.
///
/// Opens a PciBus handle, reads the device list, and closes the handle.
/// Returns the number of devices written to `buf`.
pub fn pci_enumerate(buf: &mut [abi::PciEnumEntry]) -> SysResult<usize> {
    let handle = open(ObjectType::PciBus, &[])?;

    // Read entries into the buffer (reinterpret as bytes)
    let entry_size = core::mem::size_of::<abi::PciEnumEntry>();
    let byte_buf = unsafe {
        core::slice::from_raw_parts_mut(
            buf.as_mut_ptr() as *mut u8,
            buf.len() * entry_size,
        )
    };
    let result = read(handle, byte_buf);
    let _ = close(handle);
    // Kernel returns byte count — convert to entry count
    result.map(|bytes| bytes / entry_size)
}

// ============================================================================
// Ramfs
// ============================================================================

pub use abi::RamfsListEntry;

/// List ramfs entries. Returns number of entries on success, negative on error.
pub fn ramfs_list(buf: &mut [RamfsListEntry]) -> i64 {
    syscall2(
        sys::RAMFS_LIST,
        buf.as_mut_ptr() as u64,
        (buf.len() * RamfsListEntry::SIZE) as u64,
    )
}

// ============================================================================
// Channels
// ============================================================================

/// Create a channel pair (returns two handles)
pub fn channel_pair() -> SysResult<(Handle, Handle)> {
    let ret = syscall3(sys::OPEN, ObjectType::Channel as u64, 0, 0);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        let handle_a = Handle((ret >> 32) as u32);
        let handle_b = Handle((ret & 0xFFFFFFFF) as u32);
        Ok((handle_a, handle_b))
    }
}
