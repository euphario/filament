//! System Call Handler
//!
//! Handles SVC (Supervisor Call) exceptions from user space.
//! Syscall convention (similar to Linux/Redox):
//!   - x8: syscall number
//!   - x0-x5: arguments
//!   - x0: return value
//!
//! For a microkernel, we keep syscalls minimal:
//!   - Process management
//!   - Memory management
//!   - IPC (message passing)
//!   - Scheduler control
//!
//! SECURITY: All user pointers are validated before access using the uaccess module.

// Submodules organized by functionality
// All IPC goes through the unified 5-syscall interface (100-104)
// Legacy shmem.rs and pci.rs removed - use unified interface
mod memory;
mod misc;

// Re-export hardware_reset for use from main.rs
pub use misc::hardware_reset;
mod process;

use crate::{kwarn, kdebug};
use crate::span;
#[allow(unused_imports)]  // Used for phys_to_dma method on dyn Platform
use crate::hal::Platform;
use super::uaccess::UAccessError;
use super::caps::Capabilities;

/// Syscall numbers
///
/// The unified 5-syscall interface (100-104) replaces legacy IPC, handle, and event syscalls.
/// Legacy numbers are kept in From<u64> for backward error reporting but removed from enum.
#[repr(u64)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyscallNumber {
    // Core process/memory (0-10)
    Exit = 0,
    DebugWrite = 1,
    Yield = 2,
    GetPid = 3,
    Mmap = 4,
    Munmap = 5,
    // 6-7: legacy IPC (removed)
    Spawn = 8,
    Wait = 9,
    GetTime = 10,
    Sleep = 11,
    // 12-30: legacy IPC/FD/scheme (removed)
    Exec = 31,
    Daemonize = 32,
    Kill = 33,
    PsInfo = 34,
    SetLogLevel = 35,
    MmapDma = 36,
    Reset = 37,
    // 38-47: legacy shmem (removed) - use unified open/read/write/close
    // 51-54: legacy PCI config (removed) - use unified open/read/write
    SignalAllow = 56,
    // 57-59: legacy timer/heartbeat/bus_list (removed)
    MmapDevice = 60,
    DmaPoolCreate = 61,
    DmaPoolCreateHigh = 62,
    RamfsList = 63,
    ExecMem = 64,
    KlogRead = 65,
    GetCapabilities = 66,
    // 67: legacy ChannelGetPeer (removed)
    CpuStats = 68,
    // 70-75: legacy kevent/device_list (removed) - use unified open/read
    ExecWithCaps = 74,
    Klog = 76,

    // 80-96: legacy handle system (removed) - use 100-104

    // Unified interface (100-104) - THE 5 SYSCALLS
    Open = 100,
    Read = 101,
    Write = 102,
    Map = 103,
    Close = 104,

    /// Invalid/unknown syscall
    Invalid = 0xFFFF,
}

impl From<u64> for SyscallNumber {
    fn from(n: u64) -> Self {
        match n {
            // Core process/memory
            0 => SyscallNumber::Exit,
            1 => SyscallNumber::DebugWrite,
            2 => SyscallNumber::Yield,
            3 => SyscallNumber::GetPid,
            4 => SyscallNumber::Mmap,
            5 => SyscallNumber::Munmap,
            8 => SyscallNumber::Spawn,
            9 => SyscallNumber::Wait,
            10 => SyscallNumber::GetTime,
            11 => SyscallNumber::Sleep,
            31 => SyscallNumber::Exec,
            32 => SyscallNumber::Daemonize,
            33 => SyscallNumber::Kill,
            34 => SyscallNumber::PsInfo,
            35 => SyscallNumber::SetLogLevel,
            36 => SyscallNumber::MmapDma,
            37 => SyscallNumber::Reset,
            // 39-47: legacy shmem -> Invalid (use unified interface)
            // 51-54: legacy PCI -> Invalid (use unified interface)
            56 => SyscallNumber::SignalAllow,
            // 59: legacy BusList -> Invalid (use unified interface)
            60 => SyscallNumber::MmapDevice,
            61 => SyscallNumber::DmaPoolCreate,
            62 => SyscallNumber::DmaPoolCreateHigh,
            63 => SyscallNumber::RamfsList,
            64 => SyscallNumber::ExecMem,
            65 => SyscallNumber::KlogRead,
            66 => SyscallNumber::GetCapabilities,
            68 => SyscallNumber::CpuStats,
            74 => SyscallNumber::ExecWithCaps,
            // 75: legacy DeviceList -> Invalid (use unified interface)
            76 => SyscallNumber::Klog,
            // Unified interface (100-104)
            100 => SyscallNumber::Open,
            101 => SyscallNumber::Read,
            102 => SyscallNumber::Write,
            103 => SyscallNumber::Map,
            104 => SyscallNumber::Close,
            // All other numbers (legacy IPC, FD, handle, event) -> Invalid
            _ => SyscallNumber::Invalid,
        }
    }
}

/// Syscall error codes (POSIX-like)
#[repr(i64)]
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // Infrastructure: some variants reserved for future use
pub enum SyscallError {
    Success = 0,
    /// EPERM - Operation not permitted
    PermissionDenied = -1,
    /// ENOENT - No such file or directory
    NotFound = -2,
    /// ESRCH - No such process
    NoProcess = -3,
    /// EINTR - Interrupted system call
    Interrupted = -4,
    /// EIO - I/O error
    IoError = -5,
    /// EBADF - Bad file descriptor
    BadFd = -9,
    /// ECHILD - No child processes
    NoChild = -10,
    /// EAGAIN - Resource temporarily unavailable
    WouldBlock = -11,
    /// ENOMEM - Out of memory
    OutOfMemory = -12,
    /// EACCES - Permission denied
    AccessDenied = -13,
    /// EFAULT - Bad address
    BadAddress = -14,
    /// EBUSY - Device or resource busy
    Busy = -16,
    /// EEXIST - File/resource already exists
    AlreadyExists = -17,
    /// EINVAL - Invalid argument
    InvalidArgument = -22,
    /// ENOSYS - Function not implemented
    NotImplemented = -38,
}

impl SyscallError {
    /// Convert to errno-style error code
    pub fn to_errno(self) -> i32 {
        self as i32
    }
}

/// Syscall arguments structure
#[repr(C)]
pub struct SyscallArgs {
    pub num: u64,   // x8
    pub arg0: u64,  // x0
    pub arg1: u64,  // x1
    pub arg2: u64,  // x2
    pub arg3: u64,  // x3
    pub arg4: u64,  // x4
    pub arg5: u64,  // x5
}

// ============================================================================
// Helper functions shared by submodules
// ============================================================================

/// Convert UAccessError to syscall error code
fn uaccess_to_errno(e: UAccessError) -> i64 {
    e.to_errno() as i64
}

/// Get current process ID from scheduler
fn current_pid() -> u32 {
    unsafe {
        super::task::scheduler().current_task_id().unwrap_or(1)
    }
}

/// Check if current task has the required capability
/// Returns Ok(()) if the capability is present, or an error code if not
pub(crate) fn require_capability(cap: Capabilities) -> Result<(), i64> {
    unsafe {
        let sched = super::task::scheduler();
        if let Some(task) = sched.current_task() {
            if task.has_capability(cap) {
                return Ok(());
            }
            // Log via security infrastructure
            super::security_log::log_security_event(
                super::security_log::SecurityEvent::CapabilityDenied,
                task.id,
                "", // cap name logged separately
            );
            kwarn!("security", "cap_denied"; pid = task.id as u64, cap = core::any::type_name_of_val(&cap));
        }
    }
    Err(SyscallError::PermissionDenied as i64)
}

// ============================================================================
// Syscall dispatcher
// ============================================================================

/// Handle a syscall
/// Returns the result to be placed in x0
pub fn handle(args: &SyscallArgs) -> i64 {
    let syscall = SyscallNumber::from(args.num);
    let pid = current_pid();

    // Get syscall name for tracing (only for non-trivial syscalls)
    let syscall_name = syscall_name(syscall);

    // Skip tracing for very high-frequency syscalls to reduce overhead
    let should_trace = !matches!(syscall,
        SyscallNumber::Yield |
        SyscallNumber::GetTime |
        SyscallNumber::DebugWrite |
        SyscallNumber::Klog
    );

    // Create span for traceable syscalls
    let _span = if should_trace {
        Some(span!("syscall", syscall_name; pid = pid as u64, num = args.num))
    } else {
        None
    };

    let result = match syscall {
        // Process management (process.rs)
        SyscallNumber::Exit => process::sys_exit(args.arg0 as i32),
        SyscallNumber::Spawn => process::sys_spawn(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::Wait => process::sys_wait(args.arg0 as i32, args.arg1, args.arg2 as u32),
        SyscallNumber::Exec => process::sys_exec(args.arg0, args.arg1 as usize),
        SyscallNumber::ExecWithCaps => process::sys_exec_with_caps(args.arg0, args.arg1 as usize, args.arg2),
        SyscallNumber::ExecMem => process::sys_exec_mem(args.arg0, args.arg1 as usize, args.arg2, args.arg3 as usize),
        SyscallNumber::Daemonize => process::sys_daemonize(),
        SyscallNumber::Kill => process::sys_kill(args.arg0 as u32),
        SyscallNumber::PsInfo => process::sys_ps_info(args.arg0, args.arg1 as usize),
        SyscallNumber::GetCapabilities => process::sys_get_capabilities(args.arg0 as u32),

        // Misc syscalls (misc.rs)
        SyscallNumber::DebugWrite => misc::sys_debug_write(args.arg0, args.arg1 as usize),
        SyscallNumber::Yield => misc::sys_yield(),
        SyscallNumber::GetPid => misc::sys_getpid(),
        SyscallNumber::GetTime => misc::sys_gettime(),
        SyscallNumber::Sleep => misc::sys_sleep(args.arg0),
        SyscallNumber::SetLogLevel => misc::sys_set_log_level(args.arg0 as u8),
        SyscallNumber::KlogRead => misc::sys_klog_read(args.arg0, args.arg1 as usize),
        SyscallNumber::Reset => misc::sys_reset(),
        SyscallNumber::SignalAllow => misc::sys_signal_allow(args.arg0 as u32),
        SyscallNumber::CpuStats => misc::sys_cpu_stats(args.arg0, args.arg1 as usize),
        SyscallNumber::RamfsList => misc::sys_ramfs_list(args.arg0, args.arg1 as usize),
        SyscallNumber::Klog => misc::sys_klog(args.arg0 as u8, args.arg1, args.arg2 as usize),

        // Memory management (memory.rs)
        SyscallNumber::Mmap => memory::sys_mmap(args.arg0, args.arg1 as usize, args.arg2 as u32),
        SyscallNumber::Munmap => memory::sys_munmap(args.arg0, args.arg1 as usize),
        SyscallNumber::MmapDma => memory::sys_mmap_dma(args.arg0 as usize, args.arg1),
        SyscallNumber::MmapDevice => memory::sys_mmap_device(args.arg0, args.arg1),
        SyscallNumber::DmaPoolCreate => memory::sys_dma_pool_create(args.arg0 as usize, args.arg1, args.arg2),
        SyscallNumber::DmaPoolCreateHigh => memory::sys_dma_pool_create_high(args.arg0 as usize, args.arg1, args.arg2),

        // Legacy shmem (39-47) and PCI (51-54, 59, 75) removed - use unified interface (100-104)

        // Unified interface (100-104) - THE 5 SYSCALLS
        SyscallNumber::Open => super::object::syscall::open(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::Read => super::object::syscall::read(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::Write => super::object::syscall::write(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::Map => super::object::syscall::map(args.arg0 as u32, args.arg1 as u32),
        SyscallNumber::Close => super::object::syscall::close(args.arg0 as u32),

        // Invalid or legacy syscall numbers -> ENOSYS
        SyscallNumber::Invalid => {
            kwarn!("syscall", "invalid"; num = args.num, pid = pid as u64);
            SyscallError::NotImplemented as i64
        }
    };

    // Log failed syscalls at debug level (helps with debugging)
    if should_trace && result < 0 {
        kdebug!("syscall", "failed"; name = syscall_name, result = result, pid = pid as u64);
    }

    result
}

/// Get syscall name as static str for tracing
fn syscall_name(syscall: SyscallNumber) -> &'static str {
    match syscall {
        // Core process/memory
        SyscallNumber::Exit => "exit",
        SyscallNumber::DebugWrite => "debug_write",
        SyscallNumber::Yield => "yield",
        SyscallNumber::GetPid => "getpid",
        SyscallNumber::Mmap => "mmap",
        SyscallNumber::Munmap => "munmap",
        SyscallNumber::Spawn => "spawn",
        SyscallNumber::Wait => "wait",
        SyscallNumber::GetTime => "gettime",
        SyscallNumber::Sleep => "sleep",
        SyscallNumber::Exec => "exec",
        SyscallNumber::Daemonize => "daemonize",
        SyscallNumber::Kill => "kill",
        SyscallNumber::PsInfo => "ps_info",
        SyscallNumber::SetLogLevel => "set_log_level",
        SyscallNumber::MmapDma => "mmap_dma",
        SyscallNumber::Reset => "reset",
        // Misc
        SyscallNumber::SignalAllow => "signal_allow",
        SyscallNumber::MmapDevice => "mmap_device",
        SyscallNumber::DmaPoolCreate => "dma_pool_create",
        SyscallNumber::DmaPoolCreateHigh => "dma_pool_create_high",
        SyscallNumber::RamfsList => "ramfs_list",
        SyscallNumber::ExecMem => "exec_mem",
        SyscallNumber::KlogRead => "klog_read",
        SyscallNumber::GetCapabilities => "get_capabilities",
        SyscallNumber::CpuStats => "cpu_stats",
        SyscallNumber::ExecWithCaps => "exec_with_caps",
        SyscallNumber::Klog => "klog",
        // Unified interface (100-104)
        SyscallNumber::Open => "open",
        SyscallNumber::Read => "read",
        SyscallNumber::Write => "write",
        SyscallNumber::Map => "map",
        SyscallNumber::Close => "close",
        SyscallNumber::Invalid => "invalid",
    }
}

// ============================================================================
// Assembly entry point
// ============================================================================

/// Entry point called from assembly SVC handler
///
/// This function is called from boot.S with raw syscall arguments.
/// It constructs a SyscallArgs struct and delegates to handle().
#[no_mangle]
pub extern "C" fn syscall_handler_rust(
    arg0: u64, arg1: u64, arg2: u64, arg3: u64, arg4: u64, arg5: u64, _unused: u64, num: u64
) -> i64 {
    // Update activity tick for liveness tracking (all syscalls count as activity)
    unsafe {
        let slot = super::task::current_slot();
        let sched = super::task::scheduler();
        if let Some(task) = sched.task_mut(slot) {
            task.last_activity_tick = crate::platform::mt7988::timer::ticks();

            // Reset liveness state if we're in PingSent with channel=0 (implicit pong)
            // The syscall itself IS the pong - proves the task is alive and responsive
            if let super::liveness::LivenessState::PingSent { channel: 0, .. } = task.liveness_state {
                task.liveness_state = super::liveness::LivenessState::Normal;
            }

            // Debug: log if devd (slot 0) enters a syscall in unexpected state
            if slot == 0 && !matches!(*task.state(), super::task::TaskState::Running) {
                crate::print_direct!("[SDBG] devd syscall entry state={:?} num={}\n", task.state(), num);
            }
        }
    }

    let args = SyscallArgs {
        num,
        arg0,
        arg1,
        arg2,
        arg3,
        arg4,
        arg5,
    };

    let result = handle(&args);

    // NOTE: do_resched_if_needed is called by assembly svc_handler after we return
    // Don't call it here - would cause infinite WFI loop when task blocks

    // Safe point: flush deferred log buffer before returning to user
    super::log::flush();

    result
}
