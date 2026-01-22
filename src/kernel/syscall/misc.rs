//! Miscellaneous Syscall Handlers
//!
//! This module contains various utility syscalls that don't fit into
//! other categories:
//!
//! - Process control: `sys_yield`, `sys_getpid`, `sys_signal_allow`
//! - Logging: `sys_debug_write`, `sys_klog`, `sys_klog_read`, `sys_set_log_level`
//! - Time: `sys_gettime`
//! - System: `sys_reset`, `sys_cpu_stats`, `sys_ramfs_list`

use crate::print_direct;
use super::super::uaccess::{self, UAccessError};
use super::super::caps::Capabilities;

/// Syscall error codes (re-exported from parent for convenience)
use super::SyscallError;

/// Convert UAccessError to syscall error code
fn uaccess_to_errno(e: UAccessError) -> i64 {
    e.to_errno() as i64
}

/// Get current process ID from scheduler
fn current_pid() -> u32 {
    unsafe {
        super::super::task::scheduler().current_task_id().unwrap_or(1)
    }
}

/// Check if current task has the required capability
/// Returns Ok(()) if the capability is present, or an error code if not
fn require_capability(cap: Capabilities) -> Result<(), i64> {
    use crate::kwarn;

    unsafe {
        let sched = super::super::task::scheduler();
        if let Some(task) = sched.current_task() {
            if task.has_capability(cap) {
                return Ok(());
            }
            // Log via security infrastructure
            super::super::security_log::log_security_event(
                super::super::security_log::SecurityEvent::CapabilityDenied,
                task.id,
                "", // cap name logged separately
            );
            kwarn!("security", "cap_denied"; pid = task.id as u64, cap = core::any::type_name_of_val(&cap));
        }
    }
    Err(SyscallError::PermissionDenied as i64)
}

/// Yield CPU to another process.
///
/// SYSCALL ENTRY POINT - wraps internal sched::yield_current().
///
/// Key behavior:
/// - If current task is Running, it becomes Ready and we reschedule
/// - If current task is Blocked (waiting for event), it STAYS Blocked!
///   This is critical - a blocked task calling yield should not become Ready.
/// - If no other task is ready AND current is Blocked, we WFI until woken
///
/// This ensures proper event-driven behavior where blocked tasks only run
/// when their event arrives.
pub(super) fn sys_yield() -> i64 {
    let caller_slot = super::super::task::current_slot();

    // Pre-store return value in caller's trap frame
    unsafe {
        let sched = super::super::task::scheduler();
        if let Some(task) = sched.task_mut(caller_slot) {
            task.trap_frame.x0 = 0;
        }
    }

    // Check if current task is blocked BEFORE yielding
    let is_blocked = super::super::sched::is_current_blocked();

    // Try to yield (this respects Blocked state - won't change it to Ready)
    let switched = super::super::sched::yield_current();

    if switched {
        // We switched to another task
        return 0;
    }

    // We're still the current task (no other ready task found)
    // If we're Blocked, we MUST wait for an interrupt to wake us
    // If we're Ready, just return (busy-wait caller will retry)

    if is_blocked {
        // Blocked task with no other ready tasks - must WFI until woken
        // WFI loop until we're woken OR another task becomes ready
        loop {
            // Enable IRQs, WFI, disable IRQs
            unsafe {
                core::arch::asm!("msr daifclr, #2");  // Enable IRQs
                core::arch::asm!("wfi");
                core::arch::asm!("msr daifset, #2");  // Disable IRQs
            }

            // CRITICAL: Try to reschedule after WFI!
            // The timer might have woken a DIFFERENT task (e.g., devd's timer fired).
            // If another task became Ready and we switched to it, return immediately
            // so svc_handler can perform the actual context switch.
            if super::super::sched::reschedule() {
                // We've set up a switch to another task - return now to execute it
                return 0;
            }

            // Check if the ORIGINAL caller has been woken (Blocked -> Ready)
            let still_blocked = unsafe {
                let sched = super::super::task::scheduler();
                sched.task(caller_slot)
                    .map(|t| t.is_blocked())
                    .unwrap_or(false)
            };
            if !still_blocked {
                break;
            }
        }
    } else {
        // Not blocked, just no other tasks - do one WFI for power saving
        unsafe {
            core::arch::asm!("msr daifclr, #2");  // Enable IRQs
            core::arch::asm!("wfi");
            core::arch::asm!("msr daifset, #2");  // Disable IRQs
        }
    }

    // Ensure we're marked Running before returning to userspace
    unsafe {
        let sched = super::super::task::scheduler();
        if let Some(task) = sched.task_mut(caller_slot) {
            if *task.state() == super::super::task::TaskState::Ready {
                let _ = task.set_running();
            }
        }
    }

    0
}

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

/// Write structured log message
/// level: 0=Error, 1=Warn, 2=Info, 3=Debug, 4=Trace
/// Parses [subsys] prefix from message for structured output
pub(super) fn sys_klog(level: u8, buf_ptr: u64, len: usize) -> i64 {
    // Validate level
    let log_level = match crate::klog::Level::from_u8(level) {
        Some(l) => l,
        None => return SyscallError::InvalidArgument as i64,
    };

    // Validate length
    if len == 0 {
        return 0;
    }
    if len > 4096 {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy from user space
    let mut kernel_buf = [0u8; 4096];
    match uaccess::copy_from_user(&mut kernel_buf[..len], buf_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Log via structured klog system
    crate::klog::log_user_message(log_level, &kernel_buf[..len]);

    len as i64
}

/// Get current process ID
pub(super) fn sys_getpid() -> i64 {
    current_pid() as i64
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
    if let Err(e) = require_capability(Capabilities::ALL) {
        return e;
    }

    print_direct!("\n========================================\n");
    print_direct!("  System Reset Requested\n");
    print_direct!("========================================\n");

    hardware_reset();
}

/// Allow a specific PID to send signals to this process.
///
/// By default (empty allowlist), all processes can send signals. Once at least
/// one PID is added to the allowlist, only those PIDs (plus the parent) can send.
///
/// Args: sender_pid - PID to allow signals from
/// Returns: 0 on success, negative error
pub(super) fn sys_signal_allow(sender_pid: u32) -> i64 {
    unsafe {
        let sched = super::super::task::scheduler();
        if let Some(task) = sched.current_task_mut() {
            if task.allow_signals_from(sender_pid) {
                return 0;
            } else {
                return SyscallError::OutOfMemory as i64; // Allowlist full (-12)
            }
        }
    }
    SyscallError::NoProcess as i64
}

/// CPU statistics entry for userspace
#[repr(C)]
#[derive(Clone, Copy)]
pub struct CpuStatsEntry {
    /// CPU ID (0-based)
    pub cpu_id: u32,
    /// Reserved/padding
    pub _pad: u32,
    /// Total tick count since boot
    pub tick_count: u64,
    /// Ticks spent in idle (WFI)
    pub idle_ticks: u64,
}

impl CpuStatsEntry {
    const SIZE: usize = 24; // 4 + 4 + 8 + 8 = 24 bytes
}

/// Get CPU statistics
/// Args: buf_ptr (output buffer for CpuStatsEntry array), buf_len (buffer size in bytes)
/// Returns: number of CPUs on success, negative error on failure
///
/// Each entry is 24 bytes (CpuStatsEntry):
/// - cpu_id[4]: u32 CPU ID
/// - _pad[4]: padding
/// - tick_count[8]: u64 total ticks
/// - idle_ticks[8]: u64 idle ticks
///
/// Usage calculation: busy% = 100 * (1 - idle_ticks / tick_count)
pub(super) fn sys_cpu_stats(buf_ptr: u64, buf_len: usize) -> i64 {
    use super::super::percpu::{MAX_CPUS, cpu_local};

    // Calculate how many entries we can fit
    let max_entries = buf_len / CpuStatsEntry::SIZE;
    if max_entries == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate user pointer for writing
    if let Err(e) = uaccess::validate_user_write(buf_ptr, buf_len) {
        return uaccess_to_errno(e);
    }

    // For now, only report one CPU (single-core implementation)
    // In SMP, we'd iterate over all online CPUs
    let num_cpus = 1.min(max_entries).min(MAX_CPUS);

    // Get current CPU's stats
    let cpu_data = cpu_local();
    let entry = CpuStatsEntry {
        cpu_id: cpu_data.cpu_id,
        _pad: 0,
        tick_count: cpu_data.ticks(),
        idle_ticks: cpu_data.get_idle_ticks(),
    };

    // Convert to bytes and copy to userspace
    let entry_bytes = unsafe {
        core::slice::from_raw_parts(
            &entry as *const CpuStatsEntry as *const u8,
            CpuStatsEntry::SIZE,
        )
    };

    if let Err(e) = uaccess::copy_to_user(buf_ptr, entry_bytes) {
        return uaccess_to_errno(e);
    }

    num_cpus as i64
}

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
