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
/// Uses state machine for validated transitions.
pub(super) fn sys_sleep(deadline_ns: u64) -> i64 {
    use crate::kernel::{sched, task::WaitReason};
    use crate::platform::current::timer;

    // Create deadline using unified clock API
    // Note: deadline_ns is duration from now, not absolute time
    let deadline = timer::deadline_ns(deadline_ns);

    // Check if deadline already passed (shouldn't happen but be safe)
    if timer::is_expired(deadline) {
        return 0;
    }

    // Set task to Waiting state via state machine
    if !sched::wait_current(WaitReason::Timer, deadline) {
        return KernelError::InvalidArg.to_errno();
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
        None => KernelError::InvalidArg.to_errno(),
    }
}

/// Read one formatted kernel log record into userspace buffer
/// Args: buf (pointer to buffer), len (buffer size)
/// Returns: number of bytes written, 0 if no logs available, negative on error
pub(super) fn sys_klog_read(buf_ptr: u64, buf_len: usize) -> i64 {
    // Validate buffer
    if buf_len == 0 || buf_len > 1024 {
        return KernelError::InvalidArg.to_errno();
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
        Err(_) => KernelError::BadAddress.to_errno(),
    }
}

/// Write a log message to kernel log buffer
/// Args: level (0=Error, 1=Warn, 2=Info, 3=Debug, 4=Trace), buf (message), len (length)
/// This is a compatibility shim - userspace should migrate to Klog object
pub(super) fn sys_klog(level: u8, buf_ptr: u64, buf_len: usize) -> i64 {
    // Validate level
    let level = match crate::klog::Level::from_u8(level) {
        Some(l) => l,
        None => return KernelError::InvalidArg.to_errno(),
    };

    // Validate buffer size
    if buf_len == 0 || buf_len > 256 {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy message from userspace
    let mut msg_buf = [0u8; 256];
    match uaccess::copy_from_user(&mut msg_buf[..buf_len], buf_ptr) {
        Ok(_) => {}
        Err(_) => return KernelError::BadAddress.to_errno(),
    }

    // Get caller PID for log message
    let caller_pid = crate::kernel::task::with_scheduler(|sched| {
        sched.current_task_id().unwrap_or(0)
    });

    // Build a log record
    // Format: subsys="user", event=<message>
    let msg = &msg_buf[..buf_len];

    // Use RecordBuilder to write to log ring
    let mut builder = crate::klog::RecordBuilder::new();
    builder.header(level);
    builder.subsys("user");

    // Parse the message to extract event name if possible
    // Format expected: "[name] message..." or just "message"
    let (event, _rest) = if msg.starts_with(b"[") {
        // Try to extract name from [name] prefix
        if let Some(end) = msg.iter().position(|&c| c == b']') {
            let name = core::str::from_utf8(&msg[1..end]).unwrap_or("msg");
            (name, &msg[end + 1..])
        } else {
            ("msg", msg)
        }
    } else {
        ("msg", msg)
    };

    builder.event(event);
    builder.ctx_count(0);

    // Add the full message as a key-value pair, plus PID for debugging
    if let Ok(text) = core::str::from_utf8(msg) {
        // Check if message is effectively empty (only whitespace)
        let trimmed = text.trim();
        if trimmed.is_empty() && level == crate::klog::Level::Error {
            // Log a warning about empty error message for debugging
            crate::kwarn!("syscall", "empty_klog"; pid = caller_pid as u64, len = buf_len as u64);
        }
        builder.kv_count(2);
        builder.kv("pid", caller_pid as u64);
        builder.kv("text", text);
    } else {
        builder.kv_count(1);
        builder.kv("pid", caller_pid as u64);
    }

    builder.finish();

    buf_len as i64
}

/// Write a pre-built binary log record into the kernel log ring
/// Args: buf_ptr (record bytes), buf_len (record length)
///
/// The record must be in the standard binary format (RecordHeader + fields).
/// The kernel validates the header and writes it directly — no text conversion.
/// This preserves structured key-value data from userspace ulog macros.
pub(super) fn sys_klog_write(buf_ptr: u64, buf_len: usize) -> i64 {
    // Validate size
    if buf_len < crate::klog::RecordHeader::SIZE || buf_len > crate::klog::MAX_RECORD_SIZE {
        return KernelError::InvalidArg.to_errno();
    }

    // Copy record from userspace
    let mut record = [0u8; crate::klog::MAX_RECORD_SIZE];
    match uaccess::copy_from_user(&mut record[..buf_len], buf_ptr) {
        Ok(_) => {}
        Err(_) => return KernelError::BadAddress.to_errno(),
    }

    // Validate: total_len in header must match buf_len
    let total_len = u16::from_le_bytes([record[0], record[1]]) as usize;
    if total_len != buf_len {
        return KernelError::InvalidArg.to_errno();
    }

    // Validate: level must be known
    let level = record[6];
    if crate::klog::Level::from_u8(level).is_none() {
        return KernelError::InvalidArg.to_errno();
    }

    // Re-stamp the timestamp from kernel's clock so all records use the
    // same time base (userspace counter may drift or start from different epoch)
    let ts = crate::klog::timestamp_ms();
    record[2] = ts as u8;
    record[3] = (ts >> 8) as u8;
    record[4] = (ts >> 16) as u8;
    record[5] = (ts >> 24) as u8;

    // Write directly into kernel log ring — structure preserved
    unsafe {
        let ring = &mut *core::ptr::addr_of_mut!(crate::klog::LOG_RING);
        ring.write(&record[..buf_len]);
    }

    buf_len as i64
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
///
/// Each entry is 120 bytes (#[repr(C)] RamfsListEntry):
/// - name[100]: null-terminated filename
/// - _align[4]: implicit padding for u64 alignment
/// - size[8]: u64 file size
/// - file_type[1]: 0=regular, 1=directory
/// - _pad[7]: padding
pub(super) fn sys_ramfs_list(buf_ptr: u64, buf_len: usize) -> i64 {
    // Calculate how many entries we can fit
    let max_entries = buf_len / RamfsListEntry::SIZE;
    if max_entries == 0 {
        return KernelError::InvalidArg.to_errno();
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
