//! System call wrappers - Unified 5-syscall interface
//!
//! Only the minimal syscalls needed for the unified object model.
//! Types and constants are imported from the shared `abi` crate.

use core::arch::asm;
use crate::error::{SysError, SysResult};

// Re-export types from abi crate for convenience
pub use abi::{ProcessInfo, ObjectType, Handle, LogLevel};
pub use abi::{syscall as syscall_num, log_level, liveness_status, prot, errno};

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
fn syscall3(num: u64, a0: u64, a1: u64, a2: u64) -> i64 {
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
fn syscall4(num: u64, a0: u64, a1: u64, a2: u64, a3: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            inlateout("x0") a0 => ret,
            in("x1") a1,
            in("x2") a2,
            in("x3") a3,
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

/// Execute a program from initrd with explicit capabilities
pub fn exec_with_caps(path: &str, caps: u64) -> i64 {
    syscall3(sys::EXEC_WITH_CAPS, path.as_ptr() as u64, path.len() as u64, caps)
}

/// Execute ELF from memory
pub fn exec_mem(elf_data: &[u8], name: Option<&str>) -> i64 {
    let (name_ptr, name_len) = match name {
        Some(n) => (n.as_ptr() as u64, n.len() as u64),
        None => (0, 0),
    };
    syscall4(sys::EXEC_MEM, elf_data.as_ptr() as u64, elf_data.len() as u64, name_ptr, name_len)
}

/// Get current process ID
pub fn getpid() -> u32 {
    syscall0(sys::GETPID) as u32
}

/// Wait for a child process to exit
pub fn wait(pid: i32) -> i64 {
    syscall1(sys::WAIT, pid as u64)
}

/// Kill a process by PID
pub fn kill(pid: u32) -> i64 {
    syscall1(sys::KILL, pid as u64)
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

/// Sleep for duration in milliseconds
pub fn sleep_ms(ms: u32) {
    sleep_ns(ms as u64 * 1_000_000);
}

// ============================================================================
// Logging (kernel buffer)
// ============================================================================

/// Write structured log message to kernel
pub fn klog(level: LogLevel, msg: &[u8]) {
    let _ = syscall3(sys::KLOG, level as u64, msg.as_ptr() as u64, msg.len() as u64);
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

/// Set kernel log level
pub fn set_log_level(level: u8) -> i64 {
    syscall1(sys::SET_LOG_LEVEL, level as u64)
}

/// Reset the system
pub fn reset() -> i64 {
    syscall0(sys::RESET)
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
pub fn map(handle: Handle, flags: u32) -> SysResult<u64> {
    let ret = syscall2(sys::MAP, handle.0 as u64, flags as u64);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
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
