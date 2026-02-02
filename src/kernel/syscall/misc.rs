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
use crate::kernel::error::KernelError;

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
        return KernelError::InvalidArg.to_errno();
    }

    // Copy from user space using proper VA-to-PA translation
    let mut kernel_buf = [0u8; 256];
    match uaccess::copy_from_user(&mut kernel_buf[..len], buf_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Write DIRECTLY to UART, bypassing all buffering
    for &byte in &kernel_buf[..len] {
        crate::platform::current::uart::putc(byte as char);
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
    crate::platform::current::timer::now_ns() as i64
}

/// Sleep until deadline (nanoseconds since boot)
///
/// Light oneshot timer - task enters Waiting state with deadline.
/// No object overhead, integrates with tickless scheduler.
/// Delegates through MiscOps trait for testability.
pub(super) fn sys_sleep(deadline_ns: u64) -> i64 {
    use crate::platform::current::timer;

    // Create deadline using unified clock API
    // Note: deadline_ns is duration from now, not absolute time
    let deadline = timer::deadline_ns(deadline_ns);

    // Check if deadline already passed (shouldn't happen but be safe)
    if timer::is_expired(deadline) {
        return 0;
    }

    let ctx = create_syscall_context();
    match ctx.misc().sleep_until(deadline) {
        Ok(()) => 0,
        Err(e) => e.to_errno(),
    }
}

/// Set kernel log level
/// Args: level (0=Error, 1=Warn, 2=Info, 3=Debug, 4=Trace)
pub(super) fn sys_set_log_level(level: u8) -> i64 {
    match crate::klog::Level::from_u8(level) {
        Some(lvl) => {
            crate::klog::set_level(lvl);
            0
        }
        None => KernelError::InvalidArg.to_errno(),
    }
}

/// Read one formatted kernel log record into userspace buffer
/// Args: buf (pointer to buffer), len (buffer size)
/// Returns: number of bytes written, 0 if no logs available, negative on error
pub(super) fn sys_klog_read(buf_ptr: u64, buf_len: usize) -> i64 {
    if buf_len == 0 || buf_len > 1024 {
        return KernelError::InvalidArg.to_errno();
    }

    let ctx = create_syscall_context();
    let mut text_buf = [0u8; 1024];
    let text_len = ctx.misc().read_log_record(&mut text_buf);
    if text_len <= 0 {
        return text_len;
    }

    let copy_len = (text_len as usize).min(buf_len);
    match uaccess::copy_to_user(buf_ptr, &text_buf[..copy_len]) {
        Ok(n) => n as i64,
        Err(_) => KernelError::BadAddress.to_errno(),
    }
}

/// Write a log message to kernel log buffer
/// Args: level (0=Error, 1=Warn, 2=Info, 3=Debug, 4=Trace), buf (message), len (length)
/// This is a compatibility shim - userspace should migrate to Klog object
pub(super) fn sys_klog(level: u8, buf_ptr: u64, buf_len: usize) -> i64 {
    if buf_len == 0 || buf_len > 256 {
        return KernelError::InvalidArg.to_errno();
    }

    let mut msg_buf = [0u8; 256];
    match uaccess::copy_from_user(&mut msg_buf[..buf_len], buf_ptr) {
        Ok(_) => {}
        Err(_) => return KernelError::BadAddress.to_errno(),
    }

    let ctx = create_syscall_context();
    let caller_pid = ctx.current_task_id().unwrap_or(0);
    ctx.misc().write_user_log(caller_pid, level, &msg_buf[..buf_len])
}

/// Write a pre-built binary log record into the kernel log ring
/// Args: buf_ptr (record bytes), buf_len (record length)
///
/// The record must be in the standard binary format (RecordHeader + fields).
/// The kernel validates the header and writes it directly — no text conversion.
/// This preserves structured key-value data from userspace ulog macros.
pub(super) fn sys_klog_write(buf_ptr: u64, buf_len: usize) -> i64 {
    if buf_len < crate::klog::RecordHeader::SIZE || buf_len > crate::klog::MAX_RECORD_SIZE {
        return KernelError::InvalidArg.to_errno();
    }

    let mut record = [0u8; crate::klog::MAX_RECORD_SIZE];
    match uaccess::copy_from_user(&mut record[..buf_len], buf_ptr) {
        Ok(_) => {}
        Err(_) => return KernelError::BadAddress.to_errno(),
    }

    let ctx = create_syscall_context();
    ctx.misc().write_raw_record(&record[..buf_len])
}

/// Perform hardware reset
/// This is public so it can be called from devd watchdog recovery
pub fn hardware_reset() -> ! {
    #[cfg(feature = "platform-mt7988a")]
    {
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
    }

    #[cfg(feature = "platform-qemu-virt")]
    {
        // QEMU virt: Use PSCI SYSTEM_RESET (0x84000009)
        const PSCI_SYSTEM_RESET: u64 = 0x84000009;
        unsafe {
            core::arch::asm!(
                "hvc #0",
                in("x0") PSCI_SYSTEM_RESET,
                options(noreturn)
            );
        }
    }

    // Fallback: halt (unreachable on platforms with reset implemented)
    #[allow(unreachable_code)]
    loop {
        unsafe { core::arch::asm!("wfi"); }
    }
}

/// Reset/reboot the system syscall
pub(super) fn sys_reset() -> i64 {
    // Only processes with ALL capabilities can reset the system
    let ctx = create_syscall_context();
    if let Err(_) = ctx.require_capability(Capabilities::ALL.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    print_direct!("\n========================================\n");
    print_direct!("  System Reset Requested\n");
    print_direct!("========================================\n");

    hardware_reset();
}

/// Shutdown the system (PSCI SYSTEM_OFF on QEMU, reset on MT7988A)
///
/// Unlike reset, this powers off the machine. On QEMU this causes
/// the emulator to exit cleanly, which is needed for automated testing.
pub(super) fn sys_shutdown(exit_code: u8) -> i64 {
    // Only processes with ALL capabilities can shutdown
    let ctx = create_syscall_context();
    if let Err(_) = ctx.require_capability(Capabilities::ALL.bits()) {
        return KernelError::PermDenied.to_errno();
    }

    print_direct!("\n========================================\n");
    print_direct!("  System Shutdown (exit_code={})\n", exit_code);
    print_direct!("========================================\n");

    #[cfg(feature = "platform-qemu-virt")]
    {
        // QEMU virt: Use PSCI SYSTEM_OFF (0x84000008)
        const PSCI_SYSTEM_OFF: u64 = 0x84000008;
        unsafe {
            core::arch::asm!(
                "hvc #0",
                in("x0") PSCI_SYSTEM_OFF,
                options(noreturn)
            );
        }
    }

    #[cfg(not(feature = "platform-qemu-virt"))]
    {
        // Non-QEMU: fall back to hardware reset
        hardware_reset();
    }
}

// sys_signal_allow REMOVED - Microkernel design: permission model uses capabilities.
// Signal permissions are controlled via the capability system, not explicit allowlists.

// CpuStatsEntry and sys_cpu_stats REMOVED - Microkernel design: statistics belong in userspace.
// A userspace service can track CPU usage via scheduler callbacks or performance counters.

/// Ramfs list entry for userspace
///
/// Layout (120 bytes with #[repr(C)] alignment):
///   [0..100]   name (null-terminated)
///   [100..104] implicit padding (u64 alignment)
///   [104..112] size (u64)
///   [112]      file_type (0=regular, 1=directory)
///   [113..120] _pad
use abi::RamfsListEntry;

/// List ramfs entries
/// Args: buf_ptr (output buffer), buf_len (buffer size in bytes)
/// Returns: number of entries written on success, negative error on failure
pub(super) fn sys_ramfs_list(buf_ptr: u64, buf_len: usize) -> i64 {
    let max_entries = buf_len / RamfsListEntry::SIZE;
    if max_entries == 0 {
        return KernelError::InvalidArg.to_errno();
    }

    if let Err(e) = uaccess::validate_user_write(buf_ptr, buf_len) {
        return uaccess_to_errno(e);
    }

    let ctx = create_syscall_context();

    // Use stack buffer (limit to 32 entries to avoid excessive stack use)
    let actual_max = max_entries.min(32);
    let mut entries = [RamfsListEntry {
        name: [0u8; 100],
        size: 0,
        file_type: 0,
        _pad: [0u8; 7],
    }; 32];

    let count = ctx.misc().list_ramfs(&mut entries[..actual_max], actual_max);

    for i in 0..count {
        let entry_bytes = unsafe {
            core::slice::from_raw_parts(
                &entries[i] as *const RamfsListEntry as *const u8,
                RamfsListEntry::SIZE,
            )
        };
        let offset = i * RamfsListEntry::SIZE;
        if let Err(e) = uaccess::copy_to_user(buf_ptr + offset as u64, entry_bytes) {
            return uaccess_to_errno(e);
        }
    }

    count as i64
}
