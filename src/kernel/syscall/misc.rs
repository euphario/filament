//! Miscellaneous Syscall Handlers
//!
//! This module contains utility syscalls that don't fit into other categories:
//!
//! - Process info: `sys_getpid`
//! - Logging: `sys_debug_write`, `sys_klog_read`, `sys_set_log_level`
//! - Time: `sys_gettime`, `sys_sleep`
//! - System: `sys_reset`, `sys_ramfs_list`
//!
//! ## Removed Syscalls (Microkernel Design)
//!
//! The following syscalls were removed in favor of microkernel principles:
//!
//! - `sys_yield` - IPC blocking makes yield redundant
//! - `sys_klog` - Logging goes through logd service via IPC
//! - `sys_cpu_stats` - Statistics belong in userspace services
//! - `sys_signal_allow` - Permission model uses capabilities

use crate::print_direct;
use super::super::uaccess;
use super::super::caps::Capabilities;
use super::super::syscall_ctx_impl::create_syscall_context;
use super::super::traits::syscall_ctx::SyscallContext;

/// Syscall error codes (re-exported from parent for convenience)
use super::SyscallError;

// Use shared helpers from parent module
use super::uaccess_to_errno;

// sys_yield REMOVED - Microkernel design: IPC blocking makes yield redundant.
// A task with nothing to do blocks on its IPC endpoint waiting for work.

/// Write to debug console - DIRECT UART OUTPUT
/// SECURITY: Copies user buffer via page table translation
/// NOTE: Writes DIRECTLY to UART, bypassing all buffering!
/// Use for debugging timing/ordering issues.
pub(super) fn sys_debug_write(buf_ptr: u64, len: usize) -> i64 {
    // Validate length
    if len == 0 {
        return 0;
    }
    if len > 256 {
        // Limit to 256 bytes for direct UART (unbuffered = slow)
        return SyscallError::InvalidArgument as i64;
    }

    // Copy from user space using proper VA-to-PA translation
    let mut kernel_buf = [0u8; 256];
    match uaccess::copy_from_user(&mut kernel_buf[..len], buf_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Write DIRECTLY to UART, bypassing all buffering
    for &byte in &kernel_buf[..len] {
        crate::platform::mt7988::uart::putc(byte as char);
    }

    len as i64
}

// sys_klog REMOVED - Microkernel design: logging goes through logd via IPC.
// Userspace processes send log messages to logd service, not to kernel directly.

/// Get current process ID
pub(super) fn sys_getpid() -> i64 {
    let ctx = create_syscall_context();
    ctx.current_task_id().unwrap_or(0) as i64
}

/// Get system time in nanoseconds
pub(super) fn sys_gettime() -> i64 {
    let counter = crate::platform::mt7988::timer::counter();
    let freq = crate::platform::mt7988::timer::frequency();
    // Convert counter to nanoseconds
    if freq > 0 {
        ((counter as u128 * 1_000_000_000) / freq as u128) as i64
    } else {
        0
    }
}

/// Sleep until deadline (nanoseconds since boot)
///
/// Light oneshot timer - task enters Waiting state with deadline.
/// No object overhead, integrates with tickless scheduler.
/// Uses state machine for validated transitions.
pub(super) fn sys_sleep(deadline_ns: u64) -> i64 {
    use crate::kernel::{sched, task::WaitReason};

    // Convert ns deadline to timer ticks
    let freq = crate::platform::mt7988::timer::frequency();
    let deadline_ticks = if freq > 0 {
        ((deadline_ns as u128 * freq as u128) / 1_000_000_000) as u64
    } else {
        return SyscallError::InvalidArgument as i64;
    };

    // Check if deadline already passed
    let now = crate::platform::mt7988::timer::counter();
    if now >= deadline_ticks {
        return 0; // Already past deadline, return immediately
    }

    // Set task to Waiting state via state machine
    if !sched::wait_current(WaitReason::Timer, deadline_ticks) {
        return SyscallError::InvalidArgument as i64;
    }

    // Reschedule to another task
    sched::reschedule();

    0
}

/// Set kernel log level
/// Args: level (0=Error, 1=Warn, 2=Info, 3=Debug, 4=Trace)
pub(super) fn sys_set_log_level(level: u8) -> i64 {
    match crate::klog::Level::from_u8(level) {
        Some(lvl) => {
            crate::klog::set_level(lvl);
            0
        }
        None => SyscallError::InvalidArgument as i64,
    }
}

/// Read one formatted kernel log record into userspace buffer
/// Args: buf (pointer to buffer), len (buffer size)
/// Returns: number of bytes written, 0 if no logs available, negative on error
pub(super) fn sys_klog_read(buf_ptr: u64, buf_len: usize) -> i64 {
    // Validate buffer
    if buf_len == 0 || buf_len > 1024 {
        return SyscallError::InvalidArgument as i64;
    }

    // Allocate temporary buffer for formatted output
    let mut text_buf = [0u8; 1024];
    let mut record_buf = [0u8; crate::klog::MAX_RECORD_SIZE];

    // Read one record from kernel log ring
    let record_len = unsafe {
        let ring = &mut *core::ptr::addr_of_mut!(crate::klog::LOG_RING);
        ring.read(&mut record_buf)
    };

    let Some(len) = record_len else {
        // No logs available
        return 0;
    };

    // Format the record
    let text_len = crate::klog::format_record(&record_buf[..len], &mut text_buf);
    if text_len == 0 {
        return 0;
    }

    // Copy to userspace (truncate if buffer too small)
    let copy_len = text_len.min(buf_len);
    match uaccess::copy_to_user(buf_ptr, &text_buf[..copy_len]) {
        Ok(n) => n as i64,
        Err(_) => SyscallError::BadAddress as i64,
    }
}

/// Perform hardware reset using MT7988A watchdog
/// This is public so it can be called from devd watchdog recovery
pub fn hardware_reset() -> ! {
    use crate::arch::aarch64::mmio::MmioRegion;
    use crate::arch::aarch64::mmu;
    use crate::platform::mt7988 as platform;

    let wdt = MmioRegion::new(mmu::phys_to_virt(platform::TOPRGU_BASE as u64) as usize);

    const WDT_MODE: usize = 0x00;
    const WDT_SWRST: usize = 0x14;
    const WDT_MODE_KEY: u32 = 0x2200_0000;
    const WDT_MODE_EXTEN: u32 = 1 << 2;
    const WDT_MODE_EN: u32 = 1 << 0;
    const WDT_SWRST_KEY: u32 = 0x1209;

    wdt.write32(WDT_MODE, WDT_MODE_KEY | WDT_MODE_EXTEN | WDT_MODE_EN);
    crate::arch::aarch64::mmio::dsb();
    wdt.write32(WDT_SWRST, WDT_SWRST_KEY);
    crate::arch::aarch64::mmio::dsb();

    loop {
        unsafe { core::arch::asm!("wfi"); }
    }
}

/// Reset/reboot the system syscall
pub(super) fn sys_reset() -> i64 {
    // Only processes with ALL capabilities can reset the system
    let ctx = create_syscall_context();
    if let Err(_) = ctx.require_capability(Capabilities::ALL.bits()) {
        return SyscallError::PermissionDenied as i64;
    }

    print_direct!("\n========================================\n");
    print_direct!("  System Reset Requested\n");
    print_direct!("========================================\n");

    hardware_reset();
}

// sys_signal_allow REMOVED - Microkernel design: permission model uses capabilities.
// Signal permissions are controlled via the capability system, not explicit allowlists.

// CpuStatsEntry and sys_cpu_stats REMOVED - Microkernel design: statistics belong in userspace.
// A userspace service can track CPU usage via scheduler callbacks or performance counters.

/// Ramfs list entry for userspace
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RamfsListEntry {
    /// Filename (null-terminated)
    pub name: [u8; 100],
    /// File size in bytes
    pub size: u64,
    /// File type: 0 = regular, 1 = directory
    pub file_type: u8,
    /// Padding for alignment
    pub _pad: [u8; 7],
}

impl RamfsListEntry {
    const SIZE: usize = 116; // 100 + 8 + 1 + 7 = 116 bytes
}

/// List ramfs entries
/// Args: buf_ptr (output buffer), buf_len (buffer size in bytes)
/// Returns: number of entries written on success, negative error on failure
///
/// Each entry is 116 bytes (RamfsListEntry):
/// - name[100]: null-terminated filename
/// - size[8]: u64 file size
/// - file_type[1]: 0=regular, 1=directory
/// - _pad[7]: padding
pub(super) fn sys_ramfs_list(buf_ptr: u64, buf_len: usize) -> i64 {
    // Calculate how many entries we can fit
    let max_entries = buf_len / RamfsListEntry::SIZE;
    if max_entries == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate user pointer for writing
    if let Err(e) = uaccess::validate_user_write(buf_ptr, buf_len) {
        return uaccess_to_errno(e);
    }

    // Get ramfs entries
    let ramfs = crate::ramfs::ramfs();
    let count = ramfs.len().min(max_entries);

    // Write entries to user buffer
    for i in 0..count {
        if let Some(entry) = ramfs.get(i) {
            let mut list_entry = RamfsListEntry {
                name: [0u8; 100],
                size: entry.size as u64,
                file_type: if entry.is_dir() { 1 } else { 0 },
                _pad: [0u8; 7],
            };

            // Copy filename
            let name_len = entry.name.iter().position(|&c| c == 0).unwrap_or(100).min(100);
            list_entry.name[..name_len].copy_from_slice(&entry.name[..name_len]);

            // Convert to bytes and copy to userspace
            let entry_bytes = unsafe {
                core::slice::from_raw_parts(
                    &list_entry as *const RamfsListEntry as *const u8,
                    RamfsListEntry::SIZE,
                )
            };

            let offset = i * RamfsListEntry::SIZE;
            if let Err(e) = uaccess::copy_to_user(buf_ptr + offset as u64, entry_bytes) {
                return uaccess_to_errno(e);
            }
        }
    }

    count as i64
}
