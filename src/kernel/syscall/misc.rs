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

/// Read one formatted kernel log record into userspace buffer.
///
/// Two modes:
/// - Legacy (buf_len only): destructive drain from ring. Used by logd.
/// - Cursor-based (buf_len has bit 31 set): non-destructive peek.
///   arg0 = buf_ptr, arg1 = buf_len | 0x80000000
///   First 4 bytes of buf are a u32 cursor (read + written back).
///   Remaining buf_len-4 bytes receive the formatted record.
///
/// Returns: number of bytes written, 0 if no logs available, negative on error
pub(super) fn sys_klog_read(buf_ptr: u64, buf_len: usize) -> i64 {
    // Check for cursor mode (bit 31 set)
    let cursor_mode = (buf_len & 0x80000000) != 0;
    let actual_len = buf_len & 0x7FFFFFFF;

    if cursor_mode {
        // Cursor-based non-destructive read
        if actual_len < 5 {
            return KernelError::InvalidArg.to_errno();
        }

        // Read cursor from first 4 bytes of user buffer
        let mut cursor_bytes = [0u8; 4];
        match uaccess::copy_from_user(&mut cursor_bytes, buf_ptr) {
            Ok(_) => {}
            Err(_) => return KernelError::BadAddress.to_errno(),
        }
        let cursor = u32::from_le_bytes(cursor_bytes);

        let mut record_buf = [0u8; crate::klog::MAX_RECORD_SIZE];
        let result = {
            let ring = crate::klog::LOG_RING.lock();
            ring.peek_at(cursor, &mut record_buf)
        };

        let Some((len, new_cursor)) = result else {
            return 0;
        };

        // Format to text
        let text_space = actual_len - 4;
        let mut text_buf = [0u8; 1024];
        let text_len = crate::klog::format_record(&record_buf[..len], &mut text_buf);
        if text_len == 0 {
            return 0;
        }

        let copy_len = text_len.min(text_space);

        // Write back new cursor
        let new_cursor_bytes = new_cursor.to_le_bytes();
        if uaccess::copy_to_user(buf_ptr, &new_cursor_bytes).is_err() {
            return KernelError::BadAddress.to_errno();
        }

        // Write formatted text after cursor
        match uaccess::copy_to_user(buf_ptr + 4, &text_buf[..copy_len]) {
            Ok(n) => n as i64,
            Err(_) => KernelError::BadAddress.to_errno(),
        }
    } else {
        // Legacy destructive drain
        if actual_len == 0 || actual_len > 1024 {
            return KernelError::InvalidArg.to_errno();
        }

        let ctx = create_syscall_context();
        let mut text_buf = [0u8; 1024];
        let text_len = ctx.misc().read_log_record(&mut text_buf);
        if text_len <= 0 {
            return text_len;
        }

        let copy_len = (text_len as usize).min(actual_len);
        match uaccess::copy_to_user(buf_ptr, &text_buf[..copy_len]) {
            Ok(n) => n as i64,
            Err(_) => KernelError::BadAddress.to_errno(),
        }
    }
}

/// Set console output log level
/// Args: level (0=Error, 1=Warn, 2=Info, 3=Notice, 4=Debug, 5=Trace)
pub(super) fn sys_set_console_level(level: u8) -> i64 {
    match crate::klog::Level::from_u8(level) {
        Some(lvl) => {
            crate::klog::set_console_level(lvl);
            0
        }
        None => KernelError::InvalidArg.to_errno(),
    }
}

/// Set per-module log level override
/// Args: subsys_ptr (subsystem name), subsys_len (name length), level (0xFF = remove)
pub(super) fn sys_set_module_level(subsys_ptr: u64, subsys_len: usize, level: u8) -> i64 {
    if subsys_len == 0 || subsys_len > 32 {
        return KernelError::InvalidArg.to_errno();
    }

    // Validate level (0xFF = remove override, otherwise must be valid level)
    if level != 0xFF && crate::klog::Level::from_u8(level).is_none() {
        return KernelError::InvalidArg.to_errno();
    }

    let mut name_buf = [0u8; 32];
    match uaccess::copy_from_user(&mut name_buf[..subsys_len], subsys_ptr) {
        Ok(_) => {}
        Err(_) => return KernelError::BadAddress.to_errno(),
    }

    crate::klog::set_module_level(&name_buf[..subsys_len], level);
    0
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
        use crate::kernel::arch::mmio::MmioRegion;
        use crate::kernel::arch::mmu;
        use crate::platform::mt7988 as platform;

        let wdt = MmioRegion::new(mmu::phys_to_virt(platform::TOPRGU_BASE as u64) as usize);

        const WDT_MODE: usize = 0x00;
        const WDT_SWRST: usize = 0x14;
        const WDT_MODE_KEY: u32 = 0x2200_0000;
        const WDT_MODE_EXTEN: u32 = 1 << 2;
        const WDT_MODE_EN: u32 = 1 << 0;
        const WDT_SWRST_KEY: u32 = 0x1209;

        wdt.write32(WDT_MODE, WDT_MODE_KEY | WDT_MODE_EXTEN | WDT_MODE_EN);
        crate::kernel::arch::mmio::dsb();
        wdt.write32(WDT_SWRST, WDT_SWRST_KEY);
        crate::kernel::arch::mmio::dsb();
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
    // Requires KILL capability (system-level privilege)
    let ctx = create_syscall_context();
    if let Err(_) = ctx.require_capability(Capabilities::KILL.bits()) {
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
    // Requires KILL capability (system-level privilege)
    let ctx = create_syscall_context();
    if let Err(_) = ctx.require_capability(Capabilities::KILL.bits()) {
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

/// Get system-wide information (memory, tasks, uptime)
/// Args: buf_ptr (pointer to SysInfo struct, 24 bytes)
pub(super) fn sys_sysinfo(buf_ptr: u64) -> i64 {
    use crate::kernel::pmm;
    use crate::kernel::task;
    use crate::kernel::sched;
    use crate::kernel::percpu::MAX_CPUS;

    if let Err(e) = uaccess::validate_user_write(buf_ptr, core::mem::size_of::<abi::SysInfo>()) {
        return uaccess_to_errno(e);
    }

    let num_tasks = task::with_scheduler(|sched_ref| {
        let mut count: u16 = 0;
        for (slot, task_opt) in sched_ref.iter_tasks() {
            if sched::is_idle_slot(slot) { continue; }
            if task_opt.is_some() { count += 1; }
        }
        count
    });

    let info = abi::SysInfo {
        uptime_ns: crate::platform::current::timer::now_ns(),
        total_pages: pmm::total_count() as u32,
        free_pages: pmm::free_count() as u32,
        num_tasks,
        num_cpus: MAX_CPUS as u16,
        _pad: [0; 4],
    };

    let info_bytes = unsafe {
        core::slice::from_raw_parts(
            &info as *const abi::SysInfo as *const u8,
            core::mem::size_of::<abi::SysInfo>(),
        )
    };

    match uaccess::copy_to_user(buf_ptr, info_bytes) {
        Ok(_) => 0,
        Err(e) => uaccess_to_errno(e),
    }
}

/// Get caller's base and effective priority
/// Returns: bits [7:0] = base_priority, bits [15:8] = effective_priority
pub(super) fn sys_get_priority() -> i64 {
    let slot = crate::kernel::task::current_slot();
    crate::kernel::task::with_scheduler(|sched| {
        if let Some(task) = sched.task(slot) {
            let base = task.base_priority() as u8;
            let effective = task.effective_priority() as u8;
            ((effective as i64) << 8) | (base as i64)
        } else {
            -1
        }
    })
}

/// Register exception channel on a child task.
///
/// When the child faults, instead of being killed, it is frozen and fault info
/// is sent on the specified channel. The parent can then resume or kill the child.
///
/// Args: child_pid, channel_handle (raw u32 handle value from caller's table)
pub(super) fn sys_set_exception_channel(child_pid: u32, channel_handle: u32) -> i64 {
    use crate::kernel::error::KernelError;
    use crate::kernel::object::{Handle, Object};
    use crate::kernel::object_service::object_service;
    use crate::kernel::task;

    let caller_slot = task::current_slot();
    let caller_pid = task::with_scheduler(|sched| {
        sched.task(caller_slot).map(|t| t.id).unwrap_or(0)
    });
    if caller_pid == 0 {
        return KernelError::NoProcess.to_errno();
    }

    // Verify channel handle exists and extract channel_id
    let handle = Handle::from_raw(channel_handle);
    let channel_id = object_service().with_table(caller_pid, |table| {
        match table.get(handle) {
            Some(entry) => match &entry.object {
                Object::Channel(ch) => Ok(ch.channel_id()),
                _ => Err(KernelError::BadHandle),
            },
            None => Err(KernelError::BadHandle),
        }
    });

    let channel_id = match channel_id {
        Ok(Ok(id)) => id,
        Ok(Err(e)) => return e.to_errno(),
        Err(e) => return e.to_errno(),
    };

    // Verify child_pid is a child of caller and set exception_channel
    task::with_scheduler(|sched| {
        // Find child and verify parent relationship
        let child_slot = match sched.slot_by_pid(child_pid) {
            Some(s) => s,
            None => return KernelError::NoProcess.to_errno(),
        };
        let child = match sched.task_mut(child_slot) {
            Some(t) => t,
            None => return KernelError::NoProcess.to_errno(),
        };
        if child.parent_id != caller_pid {
            return KernelError::PermDenied.to_errno();
        }
        child.exception_channel = Some((caller_pid, channel_id));
        0
    })
}

/// Resume or kill a frozen (faulted) child task.
///
/// Args: child_pid, action (0=resume, 1=kill)
pub(super) fn sys_exception_resume(child_pid: u32, action: u32) -> i64 {
    use crate::kernel::error::KernelError;
    use crate::kernel::task;

    let caller_slot = task::current_slot();
    let caller_pid = task::with_scheduler(|sched| {
        sched.task(caller_slot).map(|t| t.id).unwrap_or(0)
    });
    if caller_pid == 0 {
        return KernelError::NoProcess.to_errno();
    }

    task::with_scheduler(|sched| {
        let child_slot = match sched.slot_by_pid(child_pid) {
            Some(s) => s,
            None => return KernelError::NoProcess.to_errno(),
        };
        let child = match sched.task_mut(child_slot) {
            Some(t) => t,
            None => return KernelError::NoProcess.to_errno(),
        };

        // Verify parent relationship
        if child.parent_id != caller_pid {
            return KernelError::PermDenied.to_errno();
        }

        // Must be frozen
        if !child.is_frozen() {
            return KernelError::InvalidArg.to_errno();
        }

        match action {
            0 => {
                // Resume: Frozen → Ready
                match child.resume_from_freeze() {
                    Ok(()) => 0,
                    Err(_) => KernelError::InvalidArg.to_errno(),
                }
            }
            1 => {
                // Kill: Frozen → Exiting
                match child.set_exiting(-11) { // SIGSEGV
                    Ok(()) => 0,
                    Err(_) => KernelError::InvalidArg.to_errno(),
                }
            }
            _ => KernelError::InvalidArg.to_errno(),
        }
    })
}
