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

use crate::{kwarn, kdebug, kinfo, print_direct};
use crate::span;
#[allow(unused_imports)]  // Used for phys_to_dma method on dyn Platform
use crate::hal::Platform;
use super::uaccess::{self, UAccessError};
use super::caps::Capabilities;

/// Syscall numbers
#[repr(u64)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyscallNumber {
    /// Exit current process
    Exit = 0,
    /// Write to debug console (for testing)
    DebugWrite = 1,
    /// Yield CPU to scheduler
    Yield = 2,
    /// Get current process ID
    GetPid = 3,
    /// Allocate memory pages
    Mmap = 4,
    /// Free memory pages
    Munmap = 5,
    /// Send IPC message on channel
    Send = 6,
    /// Receive IPC message from channel
    Receive = 7,
    /// Spawn new process
    Spawn = 8,
    /// Wait for child process
    Wait = 9,
    /// Get system time
    GetTime = 10,
    /// Create a channel pair
    ChannelCreate = 11,
    /// Close a channel
    ChannelClose = 12,
    /// Transfer channel to another process
    ChannelTransfer = 13,
    /// Register a named port
    PortRegister = 14,
    /// Unregister a named port
    PortUnregister = 15,
    /// Connect to a named port
    PortConnect = 16,
    /// Accept a connection on a port
    PortAccept = 17,
    /// Open a file/resource
    Open = 18,
    /// Close a file descriptor
    Close = 19,
    /// Read from a file descriptor
    Read = 20,
    /// Write to a file descriptor
    Write = 21,
    /// Duplicate a file descriptor
    Dup = 22,
    /// Duplicate to specific FD number
    Dup2 = 23,
    /// Subscribe to events
    EventSubscribe = 24,
    /// Unsubscribe from events
    EventUnsubscribe = 25,
    /// Wait for an event
    EventWait = 26,
    /// Post an event to another process
    EventPost = 27,
    /// Open a scheme URL (scheme:path)
    SchemeOpen = 28,
    /// Register a user scheme
    SchemeRegister = 29,
    /// Unregister a user scheme
    SchemeUnregister = 30,
    /// Execute a program from path (ramfs)
    Exec = 31,
    /// Detach from parent and become a daemon
    Daemonize = 32,
    /// Kill (terminate) a process by PID
    Kill = 33,
    /// Get process list info
    PsInfo = 34,
    /// Set kernel log level
    SetLogLevel = 35,
    /// Allocate DMA-capable memory (returns VA, writes PA to pointer)
    MmapDma = 36,
    /// Reset/reboot the system
    Reset = 37,
    /// Seek to position in file descriptor
    Lseek = 38,
    /// Create shared memory region
    ShmemCreate = 39,
    /// Map existing shared memory region
    ShmemMap = 40,
    /// Allow another process to map shared memory
    ShmemAllow = 41,
    /// Wait for shared memory notification
    ShmemWait = 42,
    /// Notify shared memory waiters
    ShmemNotify = 43,
    /// Destroy shared memory region
    ShmemDestroy = 44,
    /// Send with direct switch to receiver (fast-path IPC)
    SendDirect = 45,
    /// Receive IPC message with timeout
    ReceiveTimeout = 46,
    /// Unmap shared memory region from this process
    ShmemUnmap = 47,
    /// Read PCI config space
    PciConfigRead = 51,
    /// Write PCI config space
    PciConfigWrite = 52,
    /// Allocate MSI vector(s) for a device
    PciMsiAlloc = 54,
    /// Allow a PID to send signals to this process
    SignalAllow = 56,
    /// Set a timer - delivers Timer event after duration_ns nanoseconds
    TimerSet = 57,
    /// Send heartbeat - updates last_heartbeat for monitoring
    Heartbeat = 58,
    /// List available buses (returns BusInfo array)
    BusList = 59,
    /// Map device MMIO into process address space (generic, no device tracking)
    MmapDevice = 60,
    /// Allocate from DMA pool (low memory for PCIe devices, < 4GB)
    DmaPoolCreate = 61,
    /// Allocate from high DMA pool (36-bit addresses, > 4GB)
    DmaPoolCreateHigh = 62,
    /// List ramfs entries (for vfsd directory listing)
    RamfsList = 63,
    /// Execute ELF from memory buffer (for loading from vfsd/fatfs)
    ExecMem = 64,
    /// Read formatted kernel log record (for consoled)
    KlogRead = 65,
    /// Get capabilities of a process (for security checks in protocol handlers)
    GetCapabilities = 66,
    /// Get peer PID of a channel (for identifying caller in protocol handlers)
    ChannelGetPeer = 67,
    /// Get CPU statistics (tick counts, idle ticks, CPU count)
    CpuStats = 68,
    /// Subscribe to events (kevent-style: unified filter)
    KeventSubscribe = 70,
    /// Unsubscribe from events (kevent-style)
    KeventUnsubscribe = 71,
    /// Set/modify timer (kevent-style: multiple timers, recurring support)
    KeventTimer = 72,
    /// Wait for events (kevent-style: batch receive)
    KeventWait = 73,
    /// Execute with explicit capability grant (for privilege separation)
    ExecWithCaps = 74,
    /// Invalid syscall
    Invalid = 0xFFFF,
}

impl From<u64> for SyscallNumber {
    fn from(n: u64) -> Self {
        match n {
            0 => SyscallNumber::Exit,
            1 => SyscallNumber::DebugWrite,
            2 => SyscallNumber::Yield,
            3 => SyscallNumber::GetPid,
            4 => SyscallNumber::Mmap,
            5 => SyscallNumber::Munmap,
            6 => SyscallNumber::Send,
            7 => SyscallNumber::Receive,
            8 => SyscallNumber::Spawn,
            9 => SyscallNumber::Wait,
            10 => SyscallNumber::GetTime,
            11 => SyscallNumber::ChannelCreate,
            12 => SyscallNumber::ChannelClose,
            13 => SyscallNumber::ChannelTransfer,
            14 => SyscallNumber::PortRegister,
            15 => SyscallNumber::PortUnregister,
            16 => SyscallNumber::PortConnect,
            17 => SyscallNumber::PortAccept,
            18 => SyscallNumber::Open,
            19 => SyscallNumber::Close,
            20 => SyscallNumber::Read,
            21 => SyscallNumber::Write,
            22 => SyscallNumber::Dup,
            23 => SyscallNumber::Dup2,
            24 => SyscallNumber::EventSubscribe,
            25 => SyscallNumber::EventUnsubscribe,
            26 => SyscallNumber::EventWait,
            27 => SyscallNumber::EventPost,
            28 => SyscallNumber::SchemeOpen,
            29 => SyscallNumber::SchemeRegister,
            30 => SyscallNumber::SchemeUnregister,
            31 => SyscallNumber::Exec,
            32 => SyscallNumber::Daemonize,
            33 => SyscallNumber::Kill,
            34 => SyscallNumber::PsInfo,
            35 => SyscallNumber::SetLogLevel,
            36 => SyscallNumber::MmapDma,
            37 => SyscallNumber::Reset,
            38 => SyscallNumber::Lseek,
            39 => SyscallNumber::ShmemCreate,
            40 => SyscallNumber::ShmemMap,
            41 => SyscallNumber::ShmemAllow,
            42 => SyscallNumber::ShmemWait,
            43 => SyscallNumber::ShmemNotify,
            44 => SyscallNumber::ShmemDestroy,
            45 => SyscallNumber::SendDirect,
            46 => SyscallNumber::ReceiveTimeout,
            47 => SyscallNumber::ShmemUnmap,
            51 => SyscallNumber::PciConfigRead,
            52 => SyscallNumber::PciConfigWrite,
            54 => SyscallNumber::PciMsiAlloc,
            56 => SyscallNumber::SignalAllow,
            57 => SyscallNumber::TimerSet,
            58 => SyscallNumber::Heartbeat,
            59 => SyscallNumber::BusList,
            60 => SyscallNumber::MmapDevice,
            61 => SyscallNumber::DmaPoolCreate,
            62 => SyscallNumber::DmaPoolCreateHigh,
            63 => SyscallNumber::RamfsList,
            64 => SyscallNumber::ExecMem,
            65 => SyscallNumber::KlogRead,
            66 => SyscallNumber::GetCapabilities,
            67 => SyscallNumber::ChannelGetPeer,
            68 => SyscallNumber::CpuStats,
            70 => SyscallNumber::KeventSubscribe,
            71 => SyscallNumber::KeventUnsubscribe,
            72 => SyscallNumber::KeventTimer,
            73 => SyscallNumber::KeventWait,
            74 => SyscallNumber::ExecWithCaps,
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

/// Convert UAccessError to syscall error code
fn uaccess_to_errno(e: UAccessError) -> i64 {
    e.to_errno() as i64
}

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
        SyscallNumber::DebugWrite
    );

    // Create span for traceable syscalls
    let _span = if should_trace {
        Some(span!("syscall", syscall_name; pid = pid as u64, num = args.num))
    } else {
        None
    };

    let result = match syscall {
        SyscallNumber::Exit => sys_exit(args.arg0 as i32),
        SyscallNumber::DebugWrite => sys_debug_write(args.arg0, args.arg1 as usize),
        SyscallNumber::Yield => sys_yield(),
        SyscallNumber::GetPid => sys_getpid(),
        SyscallNumber::GetTime => sys_gettime(),
        SyscallNumber::Mmap => sys_mmap(args.arg0, args.arg1 as usize, args.arg2 as u32),
        SyscallNumber::Munmap => sys_munmap(args.arg0, args.arg1 as usize),
        SyscallNumber::Send => sys_send(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::SendDirect => sys_send_direct(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::Receive => sys_receive(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::ReceiveTimeout => sys_receive_timeout(args.arg0 as u32, args.arg1, args.arg2 as usize, args.arg3 as u32),
        SyscallNumber::ChannelCreate => sys_channel_create(),
        SyscallNumber::ChannelClose => sys_channel_close(args.arg0 as u32),
        SyscallNumber::ChannelTransfer => sys_channel_transfer(args.arg0 as u32, args.arg1 as u32),
        SyscallNumber::PortRegister => sys_port_register(args.arg0, args.arg1 as usize),
        SyscallNumber::PortUnregister => sys_port_unregister(args.arg0, args.arg1 as usize),
        SyscallNumber::PortConnect => sys_port_connect(args.arg0, args.arg1 as usize),
        SyscallNumber::PortAccept => sys_port_accept(args.arg0 as u32),
        SyscallNumber::Open => sys_open(args.arg0, args.arg1 as usize, args.arg2 as u32),
        SyscallNumber::Close => sys_close(args.arg0 as u32),
        SyscallNumber::Read => sys_read(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::Write => sys_write(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::Dup => sys_dup(args.arg0 as u32),
        SyscallNumber::Dup2 => sys_dup2(args.arg0 as u32, args.arg1 as u32),
        SyscallNumber::EventSubscribe => {
            super::event::sys_event_subscribe(args.arg0 as u32, args.arg1, current_pid())
        }
        SyscallNumber::EventUnsubscribe => {
            super::event::sys_event_unsubscribe(args.arg0 as u32, args.arg1, current_pid())
        }
        SyscallNumber::EventWait => {
            sys_event_wait(args.arg0, args.arg1 as u32)
        }
        SyscallNumber::EventPost => {
            super::event::sys_event_post(args.arg0 as u32, args.arg1 as u32, args.arg2, current_pid())
        }
        SyscallNumber::SchemeOpen => {
            sys_scheme_open(args.arg0, args.arg1 as usize, args.arg2 as u32)
        }
        SyscallNumber::SchemeRegister => {
            sys_scheme_register(args.arg0, args.arg1 as usize, args.arg2 as u32)
        }
        SyscallNumber::SchemeUnregister => {
            sys_scheme_unregister(args.arg0, args.arg1 as usize)
        }
        SyscallNumber::Spawn => sys_spawn(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::Wait => sys_wait(args.arg0 as i32, args.arg1, args.arg2 as u32),
        SyscallNumber::Exec => sys_exec(args.arg0, args.arg1 as usize),
        SyscallNumber::Daemonize => sys_daemonize(),
        SyscallNumber::Kill => sys_kill(args.arg0 as u32),
        SyscallNumber::PsInfo => sys_ps_info(args.arg0, args.arg1 as usize),
        SyscallNumber::SetLogLevel => sys_set_log_level(args.arg0 as u8),
        SyscallNumber::MmapDma => sys_mmap_dma(args.arg0 as usize, args.arg1),
        SyscallNumber::Reset => sys_reset(),
        SyscallNumber::Lseek => sys_lseek(args.arg0 as u32, args.arg1 as i64, args.arg2 as u32),
        SyscallNumber::ShmemCreate => sys_shmem_create(args.arg0 as usize, args.arg1, args.arg2),
        SyscallNumber::ShmemMap => sys_shmem_map(args.arg0 as u32, args.arg1, args.arg2),
        SyscallNumber::ShmemAllow => sys_shmem_allow(args.arg0 as u32, args.arg1 as u32),
        SyscallNumber::ShmemWait => sys_shmem_wait(args.arg0 as u32, args.arg1 as u32),
        SyscallNumber::ShmemNotify => sys_shmem_notify(args.arg0 as u32),
        SyscallNumber::ShmemDestroy => sys_shmem_destroy(args.arg0 as u32),
        SyscallNumber::ShmemUnmap => sys_shmem_unmap(args.arg0 as u32),
        SyscallNumber::PciConfigRead => {
            sys_pci_config_read(args.arg0 as u32, args.arg1 as u16, args.arg2 as u8)
        }
        SyscallNumber::PciConfigWrite => {
            sys_pci_config_write(args.arg0 as u32, args.arg1 as u16, args.arg2 as u8, args.arg3 as u32)
        }
        SyscallNumber::PciMsiAlloc => {
            sys_pci_msi_alloc(args.arg0 as u32, args.arg1 as u8)
        }
        SyscallNumber::SignalAllow => sys_signal_allow(args.arg0 as u32),
        SyscallNumber::TimerSet => sys_timer_set(args.arg0),
        SyscallNumber::Heartbeat => sys_heartbeat(),
        SyscallNumber::BusList => sys_bus_list(args.arg0, args.arg1),
        SyscallNumber::MmapDevice => sys_mmap_device(args.arg0, args.arg1),
        SyscallNumber::DmaPoolCreate => sys_dma_pool_create(args.arg0 as usize, args.arg1, args.arg2),
        SyscallNumber::DmaPoolCreateHigh => sys_dma_pool_create_high(args.arg0 as usize, args.arg1, args.arg2),
        SyscallNumber::RamfsList => sys_ramfs_list(args.arg0, args.arg1 as usize),
        SyscallNumber::ExecMem => sys_exec_mem(args.arg0, args.arg1 as usize, args.arg2, args.arg3 as usize),
        SyscallNumber::KlogRead => sys_klog_read(args.arg0, args.arg1 as usize),
        SyscallNumber::GetCapabilities => sys_get_capabilities(args.arg0 as u32),
        SyscallNumber::ChannelGetPeer => sys_channel_get_peer(args.arg0 as u32),
        SyscallNumber::CpuStats => sys_cpu_stats(args.arg0, args.arg1 as usize),
        SyscallNumber::KeventSubscribe => sys_kevent_subscribe(args.arg0 as u32, args.arg1 as u32),
        SyscallNumber::KeventUnsubscribe => sys_kevent_unsubscribe(args.arg0 as u32, args.arg1 as u32),
        SyscallNumber::KeventTimer => sys_kevent_timer(args.arg0 as u32, args.arg1, args.arg2),
        SyscallNumber::KeventWait => sys_kevent_wait(args.arg0, args.arg1 as u32, args.arg2),
        SyscallNumber::ExecWithCaps => sys_exec_with_caps(args.arg0, args.arg1 as usize, args.arg2),
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
        SyscallNumber::Exit => "exit",
        SyscallNumber::DebugWrite => "debug_write",
        SyscallNumber::Yield => "yield",
        SyscallNumber::GetPid => "getpid",
        SyscallNumber::Mmap => "mmap",
        SyscallNumber::Munmap => "munmap",
        SyscallNumber::Send => "send",
        SyscallNumber::Receive => "receive",
        SyscallNumber::Spawn => "spawn",
        SyscallNumber::Wait => "wait",
        SyscallNumber::GetTime => "gettime",
        SyscallNumber::ChannelCreate => "channel_create",
        SyscallNumber::ChannelClose => "channel_close",
        SyscallNumber::ChannelTransfer => "channel_transfer",
        SyscallNumber::PortRegister => "port_register",
        SyscallNumber::PortUnregister => "port_unregister",
        SyscallNumber::PortConnect => "port_connect",
        SyscallNumber::PortAccept => "port_accept",
        SyscallNumber::Open => "open",
        SyscallNumber::Close => "close",
        SyscallNumber::Read => "read",
        SyscallNumber::Write => "write",
        SyscallNumber::Dup => "dup",
        SyscallNumber::Dup2 => "dup2",
        SyscallNumber::EventSubscribe => "event_subscribe",
        SyscallNumber::EventUnsubscribe => "event_unsubscribe",
        SyscallNumber::EventWait => "event_wait",
        SyscallNumber::EventPost => "event_post",
        SyscallNumber::SchemeOpen => "scheme_open",
        SyscallNumber::SchemeRegister => "scheme_register",
        SyscallNumber::SchemeUnregister => "scheme_unregister",
        SyscallNumber::Exec => "exec",
        SyscallNumber::Daemonize => "daemonize",
        SyscallNumber::Kill => "kill",
        SyscallNumber::PsInfo => "ps_info",
        SyscallNumber::SetLogLevel => "set_log_level",
        SyscallNumber::MmapDma => "mmap_dma",
        SyscallNumber::Reset => "reset",
        SyscallNumber::Lseek => "lseek",
        SyscallNumber::ShmemCreate => "shmem_create",
        SyscallNumber::ShmemMap => "shmem_map",
        SyscallNumber::ShmemAllow => "shmem_allow",
        SyscallNumber::ShmemWait => "shmem_wait",
        SyscallNumber::ShmemNotify => "shmem_notify",
        SyscallNumber::ShmemDestroy => "shmem_destroy",
        SyscallNumber::SendDirect => "send_direct",
        SyscallNumber::ReceiveTimeout => "receive_timeout",
        SyscallNumber::ShmemUnmap => "shmem_unmap",
        SyscallNumber::PciConfigRead => "pci_config_read",
        SyscallNumber::PciConfigWrite => "pci_config_write",
        SyscallNumber::PciMsiAlloc => "pci_msi_alloc",
        SyscallNumber::SignalAllow => "signal_allow",
        SyscallNumber::TimerSet => "timer_set",
        SyscallNumber::Heartbeat => "heartbeat",
        SyscallNumber::BusList => "bus_list",
        SyscallNumber::MmapDevice => "mmap_device",
        SyscallNumber::DmaPoolCreate => "dma_pool_create",
        SyscallNumber::DmaPoolCreateHigh => "dma_pool_create_high",
        SyscallNumber::RamfsList => "ramfs_list",
        SyscallNumber::ExecMem => "exec_mem",
        SyscallNumber::KlogRead => "klog_read",
        SyscallNumber::GetCapabilities => "get_capabilities",
        SyscallNumber::ChannelGetPeer => "channel_get_peer",
        SyscallNumber::CpuStats => "cpu_stats",
        SyscallNumber::KeventSubscribe => "kevent_subscribe",
        SyscallNumber::KeventUnsubscribe => "kevent_unsubscribe",
        SyscallNumber::KeventTimer => "kevent_timer",
        SyscallNumber::KeventWait => "kevent_wait",
        SyscallNumber::ExecWithCaps => "exec_with_caps",
        SyscallNumber::Invalid => "invalid",
    }
}

/// Get current process ID from scheduler
fn current_pid() -> u32 {
    unsafe {
        super::task::scheduler().current_task_id().unwrap_or(1)
    }
}

/// Check if current task has the required capability
/// Returns Ok(()) if the capability is present, or an error code if not
fn require_capability(cap: Capabilities) -> Result<(), i64> {
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref task) = sched.tasks[sched.current] {
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

/// Exit current process
fn sys_exit(code: i32) -> i64 {
    // Flush UART buffer so pending output from this process appears first
    while crate::platform::mt7988::uart::has_buffered_output() {
        crate::platform::mt7988::uart::flush_buffer();
    }

    print_direct!("\n========================================\n");
    print_direct!("  Process exited with code: {}\n", code);
    print_direct!("========================================\n");

    unsafe {
        let sched = super::task::scheduler();
        let current_slot = super::task::current_slot();  // Use per-CPU slot for consistency

        // Get current task's info before terminating
        let (pid, parent_id) = if let Some(ref task) = sched.tasks[current_slot] {
            (task.id, task.parent_id)
        } else {
            (0, 0)
        };

        // Clean up process resources
        // CRITICAL: bus cleanup MUST come FIRST to stop DMA before freeing buffers!
        super::bus::process_cleanup(pid);
        super::shmem::process_cleanup(pid);
        super::scheme::process_cleanup(pid);
        super::pci::release_all_devices(pid);
        super::port::process_cleanup(pid);
        super::ipc::process_cleanup(pid);

        // Close all file descriptors (releases scheme handles, channels, etc.)
        if let Some(ref mut task) = sched.tasks[current_slot] {
            task.fd_table.close_all(pid);
        }

        // Set exit code and mark as terminated
        if let Some(ref mut task) = sched.tasks[current_slot] {
            task.exit_code = code;
            task.state = super::task::TaskState::Terminated;
        }

        // Send ChildExit event to parent (always, regardless of parent state)
        if parent_id != 0 {
            for task_opt in sched.tasks.iter_mut() {
                if let Some(ref mut task) = task_opt {
                    if task.id == parent_id {
                        // Queue ChildExit event
                        let event = super::event::Event::child_exit(pid, code);
                        task.event_queue.push(event);
                        // Wake parent if blocked
                        if task.state == super::task::TaskState::Blocked {
                            task.state = super::task::TaskState::Ready;
                        }
                        break;
                    }
                }
            }
        } else {
            // Orphan process (no parent) - mark for later cleanup
            // We can't free the slot now because we're still running on this task's kernel stack!
            // The scheduler's reap_terminated() will clean it up after context switch.
            // Task stays as Terminated until then.
        }

        // Debug: show task states before scheduling
        print_direct!("  Task states at exit (current_slot={}):\n", super::task::current_slot());
        for (i, task_opt) in sched.tasks.iter().enumerate() {
            if let Some(ref task) = task_opt {
                let state_str = match task.state {
                    super::task::TaskState::Ready => "Ready",
                    super::task::TaskState::Running => "Running",
                    super::task::TaskState::Blocked => "Waiting",
                    super::task::TaskState::Terminated => "Terminated",
                };
                print_direct!("    [{}] {} - {}\n", i, task.name_str(), state_str);
            }
        }

        // Try to schedule next task
        if let Some(next_slot) = sched.schedule() {
            // Mark next task as running and update globals
            super::task::set_current_slot(next_slot);
            sched.current = next_slot;
            if let Some(ref mut task) = sched.tasks[next_slot] {
                task.state = super::task::TaskState::Running;
            }
            super::task::update_current_task_globals();
            // CRITICAL: Tell assembly we switched tasks so it doesn't
            // overwrite the new task's x0 with our return value
            super::task::SYSCALL_SWITCHED_TASK.store(1, core::sync::atomic::Ordering::Release);

            // Return - svc_handler will load new task's state and eret
            0
        } else {
            // No more tasks - halt
            kinfo!("sys_exit", "halt_no_tasks");
            loop {
                core::arch::asm!("wfi");
            }
        }
    }
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
fn sys_yield() -> i64 {
    let caller_slot = super::task::current_slot();

    // Pre-store return value in caller's trap frame
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[caller_slot] {
            task.trap_frame.x0 = 0;
        }
    }

    // Check if current task is blocked BEFORE yielding
    let is_blocked = super::sched::is_current_blocked();

    // Try to yield (this respects Blocked state - won't change it to Ready)
    let switched = super::sched::yield_current();

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
            if super::sched::reschedule() {
                // We've set up a switch to another task - return now to execute it
                return 0;
            }

            // Check if the ORIGINAL caller has been woken (Blocked → Ready)
            let still_blocked = unsafe {
                let sched = super::task::scheduler();
                sched.tasks[caller_slot]
                    .as_ref()
                    .map(|t| t.state == super::task::TaskState::Blocked)
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
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[caller_slot] {
            if task.state == super::task::TaskState::Ready {
                task.state = super::task::TaskState::Running;
            }
        }
    }

    0
}

/// Write to debug console
/// SECURITY: Copies user buffer via page table translation
fn sys_debug_write(buf_ptr: u64, len: usize) -> i64 {
    // Validate length
    if len == 0 {
        return 0;
    }
    if len > 4096 {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy from user space using proper VA-to-PA translation
    let mut kernel_buf = [0u8; 4096];
    match uaccess::copy_from_user(&mut kernel_buf[..len], buf_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Write to debug console
    for &byte in &kernel_buf[..len] {
        if byte == 0 {
            break;
        }
        crate::platform::mt7988::uart::putc(byte as char);
    }

    len as i64
}

/// Get current process ID
fn sys_getpid() -> i64 {
    current_pid() as i64
}

/// Get system time in nanoseconds
fn sys_gettime() -> i64 {
    let counter = crate::platform::mt7988::timer::counter();
    let freq = crate::platform::mt7988::timer::frequency();
    // Convert counter to nanoseconds
    if freq > 0 {
        ((counter as u128 * 1_000_000_000) / freq as u128) as i64
    } else {
        0
    }
}

/// Map memory pages into user address space
/// Args: addr (hint, ignored for now), size, prot
/// Returns: virtual address on success, negative error on failure
fn sys_mmap(_addr: u64, size: usize, prot: u32) -> i64 {
    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Parse protection flags (simplified: bit 0 = write, bit 1 = exec)
    let writable = (prot & 0x2) != 0;  // PROT_WRITE
    let executable = (prot & 0x4) != 0; // PROT_EXEC

    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            match task.mmap(size, writable, executable) {
                Some(virt_addr) => virt_addr as i64,
                None => SyscallError::OutOfMemory as i64,
            }
        } else {
            SyscallError::InvalidArgument as i64
        }
    }
}

/// Allocate DMA-capable memory
/// Args: size, dma_ptr (pointer to u64 for DMA address)
/// Returns: virtual address on success, writes DMA address to dma_ptr
///
/// The DMA address is what should be programmed into device descriptors for
/// bus-mastering DMA operations. On platforms with identity-mapped PCIe inbound
/// windows, this equals the CPU physical address. On platforms with an offset,
/// the appropriate translation is applied.
///
/// SECURITY: Writes to user pointer via page table translation
fn sys_mmap_dma(size: usize, dma_ptr: u64) -> i64 {
    // Check DMA capability
    if let Err(e) = require_capability(Capabilities::DMA) {
        return e;
    }

    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate dma_ptr
    if dma_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(dma_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }

    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            match task.mmap_dma(size) {
                Some((virt_addr, phys_addr)) => {
                    // Convert physical address to DMA address using platform HAL
                    let platform = crate::platform::mt7988::platform::platform();
                    let dma_addr = platform.phys_to_dma(phys_addr);

                    // Write DMA address to user pointer
                    if dma_ptr != 0 {
                        if let Err(e) = uaccess::put_user::<u64>(dma_ptr, dma_addr) {
                            // Unmap the allocation since we couldn't return the dma addr
                            task.munmap(virt_addr, size);
                            return uaccess_to_errno(e);
                        }
                    }
                    virt_addr as i64
                }
                None => SyscallError::OutOfMemory as i64,
            }
        } else {
            SyscallError::InvalidArgument as i64
        }
    }
}

/// Unmap memory pages from user address space
/// Args: addr, size
/// Returns: 0 on success, negative error on failure
fn sys_munmap(addr: u64, size: usize) -> i64 {
    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate that the address is in user space (not kernel)
    if !uaccess::is_user_address(addr) {
        return SyscallError::BadAddress as i64;
    }

    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if task.munmap(addr, size) {
                SyscallError::Success as i64
            } else {
                SyscallError::InvalidArgument as i64
            }
        } else {
            SyscallError::InvalidArgument as i64
        }
    }
}

/// Send IPC message on a channel
/// Args: channel_id, data_ptr, data_len
/// SECURITY: Copies user buffer via page table translation
fn sys_send(channel_id: u32, data_ptr: u64, data_len: usize) -> i64 {
    // Validate length
    if data_len > super::ipc::MAX_INLINE_PAYLOAD {
        return SyscallError::InvalidArgument as i64;
    }

    let caller_pid = current_pid();

    // Copy from user space if we have data
    let mut kernel_buf = [0u8; super::ipc::MAX_INLINE_PAYLOAD];
    if data_len > 0 {
        match uaccess::copy_from_user(&mut kernel_buf[..data_len], data_ptr) {
            Ok(_) => {}
            Err(e) => return uaccess_to_errno(e),
        }
    }

    // Check if this is a message to a kernel bus controller
    // If so, process it synchronously instead of queuing
    let is_kernel_bus = super::ipc::with_channel_table(|table| {
        // Find the peer channel
        if let Some(slot) = table.endpoints.iter().position(|e| e.id == channel_id) {
            let peer_id = table.endpoints[slot].peer;
            if let Some(peer_slot) = table.endpoints.iter().position(|e| e.id == peer_id) {
                // Kernel channels have owner PID 0
                return table.endpoints[peer_slot].owner == 0;
            }
        }
        false
    });

    if is_kernel_bus && data_len > 0 {
        // Process kernel bus message synchronously
        super::bus::process_bus_message(channel_id, &kernel_buf[..data_len]);
        return SyscallError::Success as i64;
    }

    // Send via IPC
    let data = if data_len > 0 { &kernel_buf[..data_len] } else { &[] };
    match super::ipc::sys_send(channel_id, caller_pid, data) {
        Ok(()) => SyscallError::Success as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Send IPC message with direct switch to receiver (fast-path)
/// Args: channel_id, data_ptr, data_len
/// SECURITY: Copies user buffer via page table translation
fn sys_send_direct(channel_id: u32, data_ptr: u64, data_len: usize) -> i64 {
    // Validate length
    if data_len > super::ipc::MAX_INLINE_PAYLOAD {
        return SyscallError::InvalidArgument as i64;
    }

    let caller_pid = current_pid();

    // Copy from user space if we have data
    let mut kernel_buf = [0u8; super::ipc::MAX_INLINE_PAYLOAD];
    if data_len > 0 {
        match uaccess::copy_from_user(&mut kernel_buf[..data_len], data_ptr) {
            Ok(_) => {}
            Err(e) => return uaccess_to_errno(e),
        }
    }

    // Send via IPC with direct switch
    let data = if data_len > 0 { &kernel_buf[..data_len] } else { &[] };
    match super::ipc::sys_send_direct(channel_id, caller_pid, data) {
        Ok(()) => SyscallError::Success as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Receive IPC message from a channel
/// Args: channel_id, buf_ptr, buf_len
/// Returns: message length on success, negative error on failure
/// SECURITY: Copies to user buffer via page table translation
fn sys_receive(channel_id: u32, buf_ptr: u64, buf_len: usize) -> i64 {
    // Validate user pointer if we have a buffer
    if buf_len > 0 {
        if let Err(e) = uaccess::validate_user_write(buf_ptr, buf_len) {
            return uaccess_to_errno(e);
        }
    }

    let caller_pid = current_pid();

    // Use sys_receive_blocking which atomically checks queue and registers
    // as blocked receiver while holding the lock. This prevents a race where
    // a sender/closer could queue a message after we check but before we
    // register as blocked (leaving us blocked forever with a message waiting).
    match super::ipc::sys_receive_blocking(channel_id, caller_pid) {
        Ok(msg) => {
            // Copy message to user buffer
            let copy_len = core::cmp::min(msg.header.payload_len as usize, buf_len);
            if copy_len > 0 {
                // Use copy_to_user for proper VA-to-PA translation
                if let Err(e) = uaccess::copy_to_user(buf_ptr, &msg.payload[..copy_len]) {
                    return uaccess_to_errno(e);
                }
            }
            // Return actual message length (so caller knows if truncated)
            msg.header.payload_len as i64
        }
        Err(super::ipc::ChannelError::WouldBlock) => {
            // Task is already registered as blocked receiver by sys_receive_blocking
            // Just mark task as blocked and trigger reschedule
            unsafe {
                let sched = super::task::scheduler();
                let current_slot = sched.current;

                if let Some(ref mut task) = sched.tasks[current_slot] {
                    task.state = super::task::TaskState::Blocked;
                    // Pre-store return value before switching
                    task.trap_frame.x0 = SyscallError::WouldBlock as u64;
                }
                // Set resched flag so svc_handler triggers context switch
                crate::arch::aarch64::sync::cpu_flags().set_need_resched();
            }
            SyscallError::WouldBlock as i64
        }
        Err(e) => e.to_errno() as i64,
    }
}

/// Receive IPC message from a channel with timeout
/// Args: channel_id, buf_ptr, buf_len, timeout_ms (0 = block forever)
/// Returns: message length on success, -ETIMEDOUT on timeout, negative error on failure
/// SECURITY: Copies to user buffer via page table translation
fn sys_receive_timeout(channel_id: u32, buf_ptr: u64, buf_len: usize, timeout_ms: u32) -> i64 {
    // Validate user pointer if we have a buffer
    if buf_len > 0 {
        if let Err(e) = uaccess::validate_user_write(buf_ptr, buf_len) {
            return uaccess_to_errno(e);
        }
    }

    let caller_pid = current_pid();

    // For non-blocking (timeout=0), use simple non-blocking receive
    if timeout_ms == 0 {
        match super::ipc::sys_receive(channel_id, caller_pid) {
            Ok(msg) => {
                let copy_len = core::cmp::min(msg.header.payload_len as usize, buf_len);
                if copy_len > 0 {
                    if let Err(e) = uaccess::copy_to_user(buf_ptr, &msg.payload[..copy_len]) {
                        return uaccess_to_errno(e);
                    }
                }
                return msg.header.payload_len as i64;
            }
            Err(e) => return e.to_errno() as i64,
        }
    }

    // Use sys_receive_blocking which atomically checks queue and registers
    // as blocked receiver while holding the lock. This prevents a race where
    // a sender/closer could queue a message after we check but before we
    // register as blocked (leaving us blocked forever with a message waiting).
    match super::ipc::sys_receive_blocking(channel_id, caller_pid) {
        Ok(msg) => {
            // Copy message to user buffer
            let copy_len = core::cmp::min(msg.header.payload_len as usize, buf_len);
            if copy_len > 0 {
                // Use copy_to_user for proper VA-to-PA translation
                if let Err(e) = uaccess::copy_to_user(buf_ptr, &msg.payload[..copy_len]) {
                    return uaccess_to_errno(e);
                }
            }
            // Return actual message length (so caller knows if truncated)
            msg.header.payload_len as i64
        }
        Err(super::ipc::ChannelError::WouldBlock) => {
            // Task is already registered as blocked receiver by sys_receive_blocking
            // Now set up the timeout and block
            unsafe {
                let sched = super::task::scheduler();
                let current_slot = super::task::current_slot();  // Use per-CPU slot for consistency

                if let Some(ref mut task) = sched.tasks[current_slot] {
                    task.state = super::task::TaskState::Blocked;
                    task.wait_reason = Some(super::task::WaitReason::Ipc);

                    // Set wake timeout
                    // Timer runs at 100 Hz (10ms per tick), so timeout_ms / 10 = ticks
                    let timeout_ticks = (timeout_ms as u64 + 9) / 10;
                    let current_tick = crate::platform::mt7988::timer::ticks();
                    // Use saturating_add to prevent overflow (would cause immediate wake)
                    task.wake_at = current_tick.saturating_add(timeout_ticks);

                    // Pre-store timeout error in trap frame - this is what task will see
                    // if it wakes from timeout rather than receiving a message
                    task.trap_frame.x0 = (-110i64) as u64; // ETIMEDOUT
                }
                // Set resched flag so svc_handler triggers context switch
                crate::arch::aarch64::sync::cpu_flags().set_need_resched();
            }
            SyscallError::WouldBlock as i64
        }
        Err(e) => e.to_errno() as i64,
    }
}

/// Create a channel pair
/// Returns: channel_id_a in low 32 bits, channel_id_b in high 32 bits
fn sys_channel_create() -> i64 {
    let caller_pid = current_pid();

    match super::ipc::sys_channel_create(caller_pid) {
        Ok((ch_a, ch_b)) => {
            // Pack both channel IDs into return value
            ((ch_b as i64) << 32) | (ch_a as i64)
        }
        Err(e) => e.to_errno() as i64,
    }
}

/// Close a channel
fn sys_channel_close(channel_id: u32) -> i64 {
    let caller_pid = current_pid();

    match super::ipc::sys_channel_close(channel_id, caller_pid) {
        Ok(()) => SyscallError::Success as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Transfer a channel to another process
fn sys_channel_transfer(channel_id: u32, to_pid: u32) -> i64 {
    let caller_pid = current_pid();

    match super::ipc::sys_channel_transfer(channel_id, caller_pid, to_pid) {
        Ok(()) => SyscallError::Success as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Register a named port
/// SECURITY: Copies user name buffer via page table translation
fn sys_port_register(name_ptr: u64, name_len: usize) -> i64 {
    // Validate length
    if name_len == 0 || name_len > super::port::MAX_PORT_NAME {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy name from user space
    let mut name_buf = [0u8; 64]; // MAX_PORT_NAME
    match uaccess::copy_from_user(&mut name_buf[..name_len], name_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    let caller_pid = current_pid();

    // Register port using kernel buffer
    match super::port::sys_port_register_buf(&name_buf[..name_len], caller_pid) {
        Ok(channel_id) => channel_id as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Unregister a named port
/// SECURITY: Copies user name buffer via page table translation
fn sys_port_unregister(name_ptr: u64, name_len: usize) -> i64 {
    // Validate length
    if name_len == 0 || name_len > super::port::MAX_PORT_NAME {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy name from user space
    let mut name_buf = [0u8; 64];
    match uaccess::copy_from_user(&mut name_buf[..name_len], name_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    let caller_pid = current_pid();

    match super::port::sys_port_unregister_buf(&name_buf[..name_len], caller_pid) {
        Ok(()) => SyscallError::Success as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Connect to a named port
/// SECURITY: Copies user name buffer via page table translation
fn sys_port_connect(name_ptr: u64, name_len: usize) -> i64 {
    // Validate length
    if name_len == 0 || name_len > super::port::MAX_PORT_NAME {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy name from user space
    let mut name_buf = [0u8; 64];
    match uaccess::copy_from_user(&mut name_buf[..name_len], name_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    let caller_pid = current_pid();

    match super::port::sys_port_connect_buf(&name_buf[..name_len], caller_pid) {
        Ok(channel_id) => channel_id as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Accept a connection on a port
/// Returns channel ID on success, -EAGAIN if no connection pending
fn sys_port_accept(listen_channel: u32) -> i64 {
    let caller_pid = current_pid();

    match super::port::sys_port_accept(listen_channel, caller_pid) {
        Ok(channel_id) => channel_id as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Open a file by path
/// Args: path_ptr, path_len, flags
/// Returns: file descriptor on success, negative error on failure
/// SECURITY: Copies user path buffer via page table translation
fn sys_open(path_ptr: u64, path_len: usize, flags: u32) -> i64 {
    // Validate length
    if path_len == 0 || path_len > 256 {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy path from user space
    let mut path_buf = [0u8; 256];
    match uaccess::copy_from_user(&mut path_buf[..path_len], path_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Try to open the path
    let entry = match super::fd::open_path(&path_buf[..path_len], flags) {
        Some(e) => e,
        None => return SyscallError::NotFound as i64,
    };

    // Allocate an FD in the current process
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if let Some(fd) = task.fd_table.alloc() {
                task.fd_table.set(fd, entry);
                return fd as i64;
            } else {
                return SyscallError::OutOfMemory as i64;
            }
        }
    }
    SyscallError::InvalidArgument as i64
}

/// Close a file descriptor
/// Args: fd
/// Returns: 0 on success, negative error on failure
fn sys_close(fd: u32) -> i64 {
    let caller_pid = current_pid();
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if task.fd_table.close_with_pid(fd, caller_pid) {
                return SyscallError::Success as i64;
            }
        }
    }
    SyscallError::BadFd as i64
}

/// Read from a file descriptor
/// Args: fd, buf_ptr, buf_len
/// Returns: bytes read on success, negative error on failure
/// SECURITY: Copies to user buffer via page table translation
fn sys_read(fd: u32, buf_ptr: u64, buf_len: usize) -> i64 {
    if buf_len == 0 {
        return 0;
    }

    // Limit buffer size to prevent stack overflow
    if buf_len > 4096 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate user pointer for writing (before we do the read)
    if let Err(e) = uaccess::validate_user_write(buf_ptr, buf_len) {
        return uaccess_to_errno(e);
    }

    // Read into kernel buffer first
    let mut kernel_buf = [0u8; 4096];

    // For blocking channel reads, use the blocking receive path
    let (is_channel, channel_id, is_blocking) = unsafe {
        let sched = super::task::scheduler();
        if let Some(ref task) = sched.tasks[sched.current] {
            if let Some(entry) = task.fd_table.get(fd) {
                if let super::fd::FdType::Channel(ch) = entry.fd_type {
                    (true, ch, !entry.flags.nonblocking)
                } else {
                    (false, 0, false)
                }
            } else {
                return SyscallError::BadFd as i64;
            }
        } else {
            return SyscallError::BadFd as i64;
        }
    };

    // For blocking channel reads, use blocking receive
    if is_channel && is_blocking {
        let caller_pid = current_pid();
        match super::ipc::sys_receive_blocking(channel_id, caller_pid) {
            Ok(msg) => {
                let payload = msg.payload_slice();
                let copy_len = core::cmp::min(payload.len(), buf_len);
                kernel_buf[..copy_len].copy_from_slice(&payload[..copy_len]);
                match uaccess::copy_to_user(buf_ptr, &kernel_buf[..copy_len]) {
                    Ok(_) => copy_len as i64,
                    Err(e) => uaccess_to_errno(e),
                }
            }
            Err(super::ipc::ChannelError::WouldBlock) => {
                // Block the task
                unsafe {
                    let sched = super::task::scheduler();
                    let current_slot = sched.current;
                    if let Some(ref mut task) = sched.tasks[current_slot] {
                        task.state = super::task::TaskState::Blocked;
                        task.wait_reason = Some(super::task::WaitReason::Ipc);
                    }
                    crate::arch::aarch64::sync::cpu_flags().set_need_resched();
                }
                SyscallError::WouldBlock as i64
            }
            Err(super::ipc::ChannelError::PeerClosed) => 0, // EOF
            Err(_) => -5, // EIO
        }
    } else {
        // Non-channel or non-blocking read
        let bytes_read: isize;

        unsafe {
            let sched = super::task::scheduler();
            if let Some(ref mut task) = sched.tasks[sched.current] {
                let caller_pid = task.id;
                if let Some(entry) = task.fd_table.get(fd) {
                    bytes_read = super::fd::fd_read(entry, &mut kernel_buf[..buf_len], caller_pid);
                } else {
                    return SyscallError::BadFd as i64;
                }

                // Update file offset for seekable file types (like Ramfs)
                if bytes_read > 0 {
                    if let Some(entry) = task.fd_table.get_mut(fd) {
                        entry.offset += bytes_read as u64;
                    }
                }
            } else {
                return SyscallError::BadFd as i64;
            }
        }

        // If read succeeded, copy to user space
        if bytes_read > 0 {
            match uaccess::copy_to_user(buf_ptr, &kernel_buf[..bytes_read as usize]) {
                Ok(_) => bytes_read as i64,
                Err(e) => uaccess_to_errno(e),
            }
        } else {
            bytes_read as i64
        }
    }
}

/// Write to a file descriptor
/// Args: fd, buf_ptr, buf_len
/// Returns: bytes written on success, negative error on failure
/// SECURITY: Copies user buffer via page table translation
fn sys_write(fd: u32, buf_ptr: u64, buf_len: usize) -> i64 {
    if buf_len == 0 {
        return 0;
    }

    // Limit buffer size to prevent stack overflow
    if buf_len > 4096 {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy user data to kernel buffer using proper VA-to-PA translation
    let mut kernel_buf = [0u8; 4096];
    match uaccess::copy_from_user(&mut kernel_buf[..buf_len], buf_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref task) = sched.tasks[sched.current] {
            let caller_pid = task.id;
            if let Some(entry) = task.fd_table.get(fd) {
                return super::fd::fd_write(entry, &kernel_buf[..buf_len], caller_pid) as i64;
            }
        }
    }
    SyscallError::BadFd as i64
}

/// Duplicate a file descriptor
/// Args: old_fd
/// Returns: new fd on success, negative error on failure
fn sys_dup(old_fd: u32) -> i64 {
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if let Some(new_fd) = task.fd_table.dup(old_fd) {
                return new_fd as i64;
            }
        }
    }
    SyscallError::BadFd as i64
}

/// Duplicate a file descriptor to a specific number
/// Args: old_fd, new_fd
/// Returns: new_fd on success, negative error on failure
fn sys_dup2(old_fd: u32, new_fd: u32) -> i64 {
    let caller_pid = current_pid();
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if task.fd_table.dup2(old_fd, new_fd, caller_pid) {
                return new_fd as i64;
            }
        }
    }
    SyscallError::BadFd as i64
}

/// Seek to a position in a file descriptor
/// Args: fd, offset, whence (SEEK_SET=0, SEEK_CUR=1, SEEK_END=2)
/// Returns: new position on success, negative error on failure
fn sys_lseek(fd: u32, offset: i64, whence: u32) -> i64 {
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if let Some(entry) = task.fd_table.get_mut(fd) {
                let new_pos = match whence {
                    0 => {
                        // SEEK_SET: absolute position
                        if offset < 0 {
                            return SyscallError::InvalidArgument as i64;
                        }
                        offset as u64
                    }
                    1 => {
                        // SEEK_CUR: relative to current position
                        let current = entry.offset as i64;
                        let new = current + offset;
                        if new < 0 {
                            return SyscallError::InvalidArgument as i64;
                        }
                        new as u64
                    }
                    2 => {
                        // SEEK_END: relative to end of file
                        // Get file size based on fd type
                        let file_size = match entry.fd_type {
                            super::fd::FdType::Ramfs { size, .. } => size as i64,
                            _ => return SyscallError::NotImplemented as i64,
                        };
                        let new = file_size + offset;
                        if new < 0 {
                            return SyscallError::InvalidArgument as i64;
                        }
                        new as u64
                    }
                    _ => return SyscallError::InvalidArgument as i64,
                };
                entry.offset = new_pos;
                return new_pos as i64;
            }
        }
    }
    SyscallError::BadFd as i64
}

/// Open a scheme URL (scheme:path)
/// Args: url_ptr, url_len, flags
/// Returns: file descriptor on success, negative error on failure
/// SECURITY: Copies user URL buffer via page table translation
fn sys_scheme_open(url_ptr: u64, url_len: usize, flags: u32) -> i64 {
    // Validate length
    if url_len == 0 || url_len > 256 {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy URL from user space
    let mut url_buf = [0u8; 256];
    match uaccess::copy_from_user(&mut url_buf[..url_len], url_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Try to open via scheme system
    match super::scheme::open_url(&url_buf[..url_len], flags) {
        Ok((fd_entry, _handle)) => {
            // Allocate FD and store entry
            unsafe {
                let sched = super::task::scheduler();
                if let Some(ref mut task) = sched.tasks[sched.current] {
                    if let Some(fd) = task.fd_table.alloc() {
                        task.fd_table.set(fd, fd_entry);
                        return fd as i64;
                    }
                }
            }
            SyscallError::OutOfMemory as i64
        }
        Err(e) => e as i64,
    }
}

/// Register a user scheme
/// Args: name_ptr, name_len, channel_id
/// Returns: 0 on success, negative error on failure
/// SECURITY: Copies user name buffer via page table translation
fn sys_scheme_register(name_ptr: u64, name_len: usize, channel_id: u32) -> i64 {
    // Check SCHEME_CREATE capability
    if let Err(e) = require_capability(Capabilities::SCHEME_CREATE) {
        return e;
    }

    // Validate name length
    if name_len == 0 || name_len > 31 {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy name from user space
    let mut name_buf = [0u8; 32];
    match uaccess::copy_from_user(&mut name_buf[..name_len], name_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Get current PID
    let owner_pid = current_pid();

    // Parse scheme name from kernel buffer
    let name = match core::str::from_utf8(&name_buf[..name_len]) {
        Ok(s) => s,
        Err(_) => return SyscallError::InvalidArgument as i64,
    };

    // Check if name conflicts with kernel schemes
    if super::scheme::get_kernel_scheme(name).is_some() {
        return SyscallError::AlreadyExists as i64;
    }

    // Register the user scheme
    unsafe {
        let reg = super::scheme::registry();
        if reg.register_user(name, owner_pid, channel_id).is_some() {
            0
        } else {
            SyscallError::OutOfMemory as i64
        }
    }
}

/// Unregister a user scheme
/// Args: name_ptr, name_len
/// Returns: 0 on success, negative error on failure
/// SECURITY: Copies user name buffer via page table translation
fn sys_scheme_unregister(name_ptr: u64, name_len: usize) -> i64 {
    // Validate name length
    if name_len == 0 || name_len > 31 {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy name from user space
    let mut name_buf = [0u8; 32];
    match uaccess::copy_from_user(&mut name_buf[..name_len], name_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Get current PID
    let owner_pid = current_pid();

    // Parse scheme name from kernel buffer
    let name = match core::str::from_utf8(&name_buf[..name_len]) {
        Ok(s) => s,
        Err(_) => return SyscallError::InvalidArgument as i64,
    };

    // Check ownership and unregister
    unsafe {
        let reg = super::scheme::registry();
        if let Some(entry) = reg.find(name) {
            // Verify ownership
            if entry.scheme_type != super::scheme::SchemeType::User {
                return SyscallError::PermissionDenied as i64;
            }
            if entry.owner_pid != owner_pid {
                return SyscallError::PermissionDenied as i64;
            }
        } else {
            return SyscallError::NotFound as i64;
        }

        if reg.unregister(name) {
            0
        } else {
            SyscallError::NotFound as i64
        }
    }
}

/// Spawn a new process from a built-in ELF binary
/// Args: elf_id (built-in ELF identifier), name_ptr, name_len
/// Returns: child PID on success, negative error on failure
/// SECURITY: Copies user name buffer via page table translation
fn sys_spawn(elf_id: u32, name_ptr: u64, name_len: usize) -> i64 {
    // Check SPAWN capability
    if let Err(e) = require_capability(Capabilities::SPAWN) {
        return e;
    }

    // Validate name length
    if name_len == 0 || name_len > 15 {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy name from user space
    let mut name_buf = [0u8; 16];
    match uaccess::copy_from_user(&mut name_buf[..name_len], name_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Get the ELF binary by ID
    let elf_data = match super::elf::get_elf_by_id(elf_id) {
        Some(data) => data,
        None => return SyscallError::NotFound as i64,
    };

    // Parse name from kernel buffer
    let name = core::str::from_utf8(&name_buf[..name_len]).unwrap_or("child");

    // Get current PID as parent
    let parent_id = current_pid();

    // Spawn the child process
    match super::elf::spawn_from_elf_with_parent(elf_data, name, parent_id) {
        Ok((child_id, _slot)) => child_id as i64,
        Err(_) => SyscallError::OutOfMemory as i64,
    }
}

/// Execute a program from a path (searches ramfs)
/// Args: path_ptr, path_len
/// Returns: child PID on success, negative error on failure
/// SECURITY: Copies user path buffer via page table translation
fn sys_exec(path_ptr: u64, path_len: usize) -> i64 {
    // Validate path length
    if path_len == 0 || path_len > 127 {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy path from user space
    let mut path_buf = [0u8; 128];
    match uaccess::copy_from_user(&mut path_buf[..path_len], path_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Parse path from kernel buffer
    let path = match core::str::from_utf8(&path_buf[..path_len]) {
        Ok(s) => s,
        Err(_) => return SyscallError::InvalidArgument as i64,
    };

    // Get current PID as parent
    let parent_id = current_pid();

    // Try to spawn from ramfs path
    match super::elf::spawn_from_path_with_parent(path, parent_id) {
        Ok((child_id, _slot)) => child_id as i64,
        Err(super::elf::ElfError::NotExecutable) => SyscallError::NotFound as i64,
        Err(_) => SyscallError::OutOfMemory as i64,
    }
}

/// Execute a program with explicit capability grant (privilege separation)
/// Args: path_ptr, path_len, capabilities (bitmask)
/// Returns: child PID on success, negative error on failure
/// SECURITY: Child capabilities are intersected with parent's - can't escalate
fn sys_exec_with_caps(path_ptr: u64, path_len: usize, capabilities: u64) -> i64 {
    // Require SPAWN capability
    if let Err(e) = require_capability(super::caps::Capabilities::SPAWN) {
        return e;
    }

    // Require GRANT capability to delegate capabilities to children
    if let Err(e) = require_capability(super::caps::Capabilities::GRANT) {
        return e;
    }

    // Validate path length
    if path_len == 0 || path_len > 127 {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy path from user space
    let mut path_buf = [0u8; 128];
    match uaccess::copy_from_user(&mut path_buf[..path_len], path_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    // Parse path from kernel buffer
    let path = match core::str::from_utf8(&path_buf[..path_len]) {
        Ok(s) => s,
        Err(_) => return SyscallError::InvalidArgument as i64,
    };

    // Get current PID as parent
    let parent_id = current_pid();

    // Convert capabilities bitmask
    let requested_caps = super::caps::Capabilities::from_bits(capabilities);

    // Try to spawn from ramfs path with explicit capabilities
    match super::elf::spawn_from_path_with_caps_find(path, parent_id, requested_caps) {
        Ok((child_id, _slot)) => child_id as i64,
        Err(super::elf::ElfError::NotExecutable) => SyscallError::NotFound as i64,
        Err(_) => SyscallError::OutOfMemory as i64,
    }
}

/// Execute ELF binary from a memory buffer
/// Args:
///   elf_ptr: Pointer to ELF data in userspace
///   elf_len: Size of ELF data in bytes
///   name_ptr: Pointer to process name (or 0 for default)
///   name_len: Length of process name
/// Returns: PID of new process on success, negative error on failure
/// SECURITY: Reads ELF data via page table translation
fn sys_exec_mem(elf_ptr: u64, elf_len: usize, name_ptr: u64, name_len: usize) -> i64 {
    // Validate ELF size (reasonable bounds: at least an ELF header, at most 16MB)
    if elf_len < 64 || elf_len > 16 * 1024 * 1024 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate and read ELF data from userspace
    // We need to copy to kernel memory since the ELF loader works with slices
    if let Err(e) = uaccess::validate_user_read(elf_ptr, elf_len) {
        return uaccess_to_errno(e);
    }

    // Allocate kernel buffer for ELF data
    // Use the PMM for larger allocations
    let num_pages = (elf_len + 4095) / 4096;
    let elf_phys = match super::pmm::alloc_pages(num_pages) {
        Some(addr) => addr,
        None => return SyscallError::OutOfMemory as i64,
    };

    // Get kernel virtual address for the allocation
    let elf_virt = crate::arch::aarch64::mmu::phys_to_virt(elf_phys as u64) as *mut u8;

    // Copy ELF data from userspace to kernel buffer
    let elf_slice = unsafe { core::slice::from_raw_parts_mut(elf_virt, elf_len) };
    if let Err(e) = uaccess::copy_from_user(elf_slice, elf_ptr) {
        super::pmm::free_pages(elf_phys, num_pages);
        return uaccess_to_errno(e);
    }

    // Parse process name
    let mut name_buf = [0u8; 32];
    let name = if name_ptr != 0 && name_len > 0 {
        let actual_len = name_len.min(31);
        if let Err(e) = uaccess::copy_from_user(&mut name_buf[..actual_len], name_ptr) {
            super::pmm::free_pages(elf_phys, num_pages);
            return uaccess_to_errno(e);
        }
        core::str::from_utf8(&name_buf[..actual_len]).unwrap_or("exec_mem")
    } else {
        "exec_mem"
    };

    // Get current PID as parent
    let parent_id = current_pid();

    // Spawn from the ELF data
    let result = match super::elf::spawn_from_elf_with_parent(elf_slice, name, parent_id) {
        Ok((child_id, _slot)) => child_id as i64,
        Err(super::elf::ElfError::BadMagic) => SyscallError::InvalidArgument as i64,
        Err(super::elf::ElfError::NotExecutable) => SyscallError::InvalidArgument as i64,
        Err(super::elf::ElfError::WrongArch) => SyscallError::InvalidArgument as i64,
        Err(_) => SyscallError::OutOfMemory as i64,
    };

    // Free the kernel buffer (ELF data has been copied to process address space)
    super::pmm::free_pages(elf_phys, num_pages);

    result
}

/// Wait flags
pub const WNOHANG: u32 = 1;  // Don't block if no child has exited

/// Wait for a child process to exit
/// Args: pid (-1 = any child, >0 = specific child), status_ptr (pointer to i32 for exit status), flags
/// flags: 0 = block until child exits, WNOHANG (1) = return immediately if no child exited
/// Returns: PID of exited child on success, 0 if WNOHANG and no child exited, negative error
/// SECURITY: Writes to user status pointer via page table translation
fn sys_wait(pid: i32, status_ptr: u64, flags: u32) -> i64 {
    let caller_pid = current_pid();

    // Validate status pointer if provided
    if status_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(status_ptr, core::mem::size_of::<i32>()) {
            return uaccess_to_errno(e);
        }
    }

    unsafe {
        let sched = super::task::scheduler();

        // Find the calling task
        let caller_slot = super::task::current_slot();  // Use per-CPU slot for consistency
        let caller_task = match &sched.tasks[caller_slot] {
            Some(t) if t.id == caller_pid => t,
            _ => return SyscallError::InvalidArgument as i64,
        };

        // Check if caller has any children
        if !caller_task.has_children() {
            return SyscallError::NoChild as i64;
        }

        // Look for terminated children - first pass: find terminated child info
        let mut found_child: Option<(usize, u32, i32)> = None; // (slot, pid, exit_code)

        for (slot, task_opt) in sched.tasks.iter().enumerate() {
            if let Some(ref task) = task_opt {
                // Check if this is a child of the caller
                if task.parent_id != caller_pid {
                    continue;
                }

                // Check if waiting for specific PID or any child
                if pid > 0 && task.id != pid as u32 {
                    continue;
                }

                // Check if terminated
                if task.state == super::task::TaskState::Terminated {
                    found_child = Some((slot, task.id, task.exit_code));
                    break;
                }
            }
        }

        // Second pass: reap the child if found (outside of iterator)
        if let Some((child_slot, child_pid, exit_code)) = found_child {
            // Write exit status to user space if pointer provided
            if status_ptr != 0 {
                if let Err(e) = uaccess::put_user::<i32>(status_ptr, exit_code) {
                    return uaccess_to_errno(e);
                }
            }

            // Remove child from parent's list
            if let Some(ref mut parent) = sched.tasks[caller_slot] {
                parent.remove_child(child_pid);
            }

            // Free the child task's resources
            sched.bump_generation(child_slot);  // Ensure next task in this slot gets new PID
            sched.tasks[child_slot] = None;

            // Return (pid << 32 | exit_code) as documented
            return ((child_pid as i64) << 32) | ((exit_code as i64) & 0xFFFFFFFF);
        }

        // No terminated children found
        // If WNOHANG is set, return 0 immediately without blocking
        if (flags & WNOHANG) != 0 {
            return 0;  // No child exited, but don't block
        }

        // Block and switch to another task
        if let Some(ref mut parent) = sched.tasks[caller_slot] {
            parent.state = super::task::TaskState::Blocked;
            // Pre-store return value in caller's trap frame
            parent.trap_frame.x0 = SyscallError::WouldBlock as i64 as u64;
        }

        // Find another task to run
        if let Some(next_slot) = sched.schedule() {
            if next_slot != caller_slot {
                super::task::set_current_slot(next_slot);
                sched.current = next_slot;
                if let Some(ref mut next) = sched.tasks[next_slot] {
                    next.state = super::task::TaskState::Running;
                }
                super::task::update_current_task_globals();
                super::task::SYSCALL_SWITCHED_TASK.store(1, core::sync::atomic::Ordering::Release);
            }
        }

        SyscallError::WouldBlock as i64
    }
}

/// Wait for an event (blocking or non-blocking)
/// Args: event_buf (pointer to Event struct), flags (0=block, 1=non-block)
/// Returns: 1 if event received, 0 if would block, negative error
/// SECURITY: Writes Event struct via page table translation
fn sys_event_wait(event_buf: u64, flags: u32) -> i64 {
    // Validate user pointer for writing an Event struct
    let event_size = core::mem::size_of::<super::event::Event>();
    if let Err(e) = uaccess::validate_user_write(event_buf, event_size) {
        return uaccess_to_errno(e);
    }

    unsafe {
        let sched = super::task::scheduler();
        let slot = sched.current;
        if let Some(ref mut task) = sched.tasks[slot] {
            // Check task's event queue
            if let Some(event) = task.event_queue.pop() {
                let event_bytes = core::slice::from_raw_parts(
                    &event as *const super::event::Event as *const u8,
                    event_size
                );
                if let Err(e) = uaccess::copy_to_user(event_buf, event_bytes) {
                    return uaccess_to_errno(e);
                }
                return 1;
            }

            // No event available
            if flags & 1 != 0 {
                // Non-blocking - return immediately
                return 0;
            }

            // Would block - mark task as waiting (use sched module for proper state)
            task.state = super::task::TaskState::Blocked;
            task.wait_reason = Some(super::task::WaitReason::Event);
            // Trigger context switch so another task can run
            crate::arch::aarch64::sync::cpu_flags().set_need_resched();
            return SyscallError::WouldBlock as i64;
        }
    }
    SyscallError::InvalidArgument as i64
}

/// Process info structure for ps_info syscall
/// Must match userlib definition
#[repr(C)]
#[derive(Clone, Copy)]
pub struct ProcessInfo {
    pub pid: u32,
    pub parent_pid: u32,
    pub state: u8,           // 0=Ready, 1=Running, 2=Blocked, 3=Terminated
    pub heartbeat_status: u8, // 0=never, 1=alive (<1s), 2=stale (1-5s), 3=dead (>5s)
    pub _pad: [u8; 2],       // Alignment padding
    pub heartbeat_age_ms: u32, // Milliseconds since last heartbeat (0 if never)
    pub name: [u8; 16],
}

/// Heartbeat status values
pub mod heartbeat_status {
    pub const NEVER: u8 = 0;   // Never sent heartbeat
    pub const ALIVE: u8 = 1;   // Last heartbeat < 1 second ago
    pub const STALE: u8 = 2;   // Last heartbeat 1-5 seconds ago
    pub const DEAD: u8 = 3;    // Last heartbeat > 5 seconds ago
}

impl ProcessInfo {
    #[allow(dead_code)] // Infrastructure for future use
    pub const fn empty() -> Self {
        Self {
            pid: 0,
            parent_pid: 0,
            state: 0,
            heartbeat_status: 0,
            _pad: [0; 2],
            heartbeat_age_ms: 0,
            name: [0; 16],
        }
    }
}

/// Daemonize - detach from parent and become a daemon
fn sys_daemonize() -> i64 {
    let caller_pid = current_pid();

    unsafe {
        let sched = super::task::scheduler();

        // Find the calling task
        let mut parent_id = 0u32;
        for task_opt in sched.tasks.iter_mut() {
            if let Some(ref mut task) = task_opt {
                if task.id == caller_pid {
                    // Save parent ID and clear it
                    parent_id = task.parent_id;
                    task.parent_id = 0;
                    break;
                }
            }
        }

        // Remove from parent's children list and wake parent if blocked
        if parent_id != 0 {
            for task_opt in sched.tasks.iter_mut() {
                if let Some(ref mut task) = task_opt {
                    if task.id == parent_id {
                        task.remove_child(caller_pid);
                        // Wake up parent if it's blocked (likely waiting on wait())
                        if task.state == super::task::TaskState::Blocked {
                            task.state = super::task::TaskState::Ready;
                        }
                        break;
                    }
                }
            }
        }
    }

    0  // Success
}

/// Kill a process by PID
///
/// SECURITY: Only allows killing:
/// - The caller itself (suicide)
/// - Children of the caller
/// - Processes with CAP_KILL capability can kill any process except init (PID 1)
fn sys_kill(pid: u32) -> i64 {
    if pid == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // SECURITY: Never allow killing init (PID 1) - system would become unstable
    if pid == 1 {
        kwarn!("security", "kill_init_denied"; caller = current_pid() as u64);
        return SyscallError::PermissionDenied as i64;
    }

    let caller_pid = current_pid();

    unsafe {
        let sched = super::task::scheduler();

        // Find the target task and get its info
        let mut target_slot = None;
        let mut parent_id = 0u32;
        let mut target_parent_id = 0u32;

        for (i, task_opt) in sched.tasks.iter().enumerate() {
            if let Some(ref task) = task_opt {
                if task.id == pid {
                    target_slot = Some(i);
                    parent_id = task.parent_id;
                    target_parent_id = task.parent_id;
                    break;
                }
            }
        }

        let slot = match target_slot {
            Some(s) => s,
            None => return SyscallError::NoProcess as i64,
        };

        // SECURITY: Authorization check
        // Allow if:
        // 1. Killing self (suicide)
        // 2. Target is a child of caller
        // 3. Caller has CAP_KILL capability
        let is_suicide = pid == caller_pid;
        let is_child = target_parent_id == caller_pid;
        let has_cap_kill = require_capability(Capabilities::KILL).is_ok();

        if !is_suicide && !is_child && !has_cap_kill {
            kwarn!("security", "kill_denied"; caller = caller_pid as u64, target = pid as u64);
            return SyscallError::PermissionDenied as i64;
        }

        // Clean up process resources (same as sys_exit)
        // CRITICAL: bus cleanup MUST come FIRST to stop DMA before freeing buffers!
        super::bus::process_cleanup(pid);
        super::shmem::process_cleanup(pid);
        super::scheme::process_cleanup(pid);
        super::pci::release_all_devices(pid);
        super::port::process_cleanup(pid);
        super::ipc::process_cleanup(pid);

        // Close all file descriptors and mark as terminated
        if let Some(ref mut task) = sched.tasks[slot] {
            task.fd_table.close_all(pid);
            task.exit_code = -9;  // SIGKILL
            task.state = super::task::TaskState::Terminated;
        }

        // Send ChildExit event to parent and remove from child list
        if parent_id != 0 {
            for task_opt in sched.tasks.iter_mut() {
                if let Some(ref mut task) = task_opt {
                    if task.id == parent_id {
                        // Remove from parent's child list
                        task.remove_child(pid);
                        // Queue ChildExit event
                        let event = super::event::Event::child_exit(pid, -9);
                        task.event_queue.push(event);
                        // Wake parent if blocked
                        if task.state == super::task::TaskState::Blocked {
                            task.state = super::task::TaskState::Ready;
                        }
                        break;
                    }
                }
            }
            // Task has parent but will be reaped by reap_terminated
            // (parent receives event but doesn't need to call wait())
        } else {
            // Orphan process (no parent)
            // If killing a DIFFERENT task, we can free it immediately
            // If killing OURSELVES, defer to reap_terminated (we're still on our stack!)
            if pid != caller_pid {
                sched.bump_generation(slot);
                sched.tasks[slot] = None;
            }
            // Otherwise, task stays as Terminated until timer reaps it
        }

        // If killing current task, schedule next
        if pid == caller_pid {
            if let Some(next_slot) = sched.schedule() {
                super::task::set_current_slot(next_slot);
                sched.current = next_slot;
                if let Some(ref mut next) = sched.tasks[next_slot] {
                    next.state = super::task::TaskState::Running;
                }
                super::task::update_current_task_globals();
                // Signal to assembly not to store return value into the new task
                super::task::SYSCALL_SWITCHED_TASK.store(1, core::sync::atomic::Ordering::Release);
            }
        }
    }

    0  // Success
}

/// Get process info list
/// Args: buf_ptr (pointer to ProcessInfo array), max_entries
/// Returns: number of entries written
fn sys_ps_info(buf_ptr: u64, max_entries: usize) -> i64 {
    if max_entries == 0 {
        return 0;
    }

    // Validate user pointer
    let entry_size = core::mem::size_of::<ProcessInfo>();
    if let Err(e) = uaccess::validate_user_write(buf_ptr, entry_size * max_entries) {
        return uaccess_to_errno(e);
    }

    let mut count = 0usize;

    unsafe {
        let sched = super::task::scheduler();

        for task_opt in sched.tasks.iter() {
            if count >= max_entries {
                break;
            }

            if let Some(ref task) = task_opt {
                // Calculate heartbeat age and status
                let current_tick = crate::platform::mt7988::timer::ticks();
                let (hb_status, hb_age_ms) = if task.last_heartbeat == 0 {
                    (heartbeat_status::NEVER, 0u32)
                } else {
                    // Timer runs at 100 Hz (10ms per tick)
                    let age_ticks = current_tick.saturating_sub(task.last_heartbeat);
                    let age_ms = (age_ticks * 10) as u32;  // 10ms per tick

                    let status = if age_ms < 1000 {
                        heartbeat_status::ALIVE  // < 1 second
                    } else if age_ms < 5000 {
                        heartbeat_status::STALE  // 1-5 seconds
                    } else {
                        heartbeat_status::DEAD   // > 5 seconds
                    };
                    (status, age_ms)
                };

                let info = ProcessInfo {
                    pid: task.id,
                    parent_pid: task.parent_id,
                    state: task.state as u8,
                    heartbeat_status: hb_status,
                    _pad: [0; 2],
                    heartbeat_age_ms: hb_age_ms,
                    name: task.name,
                };

                // Write to user buffer
                let offset = (count * entry_size) as u64;
                let info_bytes = core::slice::from_raw_parts(
                    &info as *const ProcessInfo as *const u8,
                    entry_size
                );

                if let Err(_) = uaccess::copy_to_user(buf_ptr + offset, info_bytes) {
                    break;
                }

                count += 1;
            }
        }
    }

    count as i64
}

/// Set kernel log level
/// Args: level (0=Error, 1=Warn, 2=Info, 3=Debug, 4=Trace)
fn sys_set_log_level(level: u8) -> i64 {
    match crate::log::LogLevel::from_u8(level) {
        Some(lvl) => {
            crate::log::set_level(lvl);
            0
        }
        None => SyscallError::InvalidArgument as i64,
    }
}

/// Read one formatted kernel log record into userspace buffer
/// Args: buf (pointer to buffer), len (buffer size)
/// Returns: number of bytes written, 0 if no logs available, negative on error
fn sys_klog_read(buf_ptr: u64, buf_len: usize) -> i64 {
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

/// Get capabilities of a process
/// Args: pid (0 = current process)
/// Returns: capability bits as i64 on success, negative error on failure
///
/// This allows protocol handlers to verify caller permissions before
/// executing privileged operations. Any process can query any other
/// process's capabilities (they are not secret).
fn sys_get_capabilities(pid: u32) -> i64 {
    let target_pid = if pid == 0 { current_pid() } else { pid };

    unsafe {
        let sched = super::task::scheduler();

        // Find the task by PID
        for slot in 0..super::task::MAX_TASKS {
            if let Some(ref task) = sched.tasks[slot] {
                if task.id == target_pid {
                    return task.capabilities.bits() as i64;
                }
            }
        }
    }

    // Process not found
    SyscallError::NoProcess as i64
}

/// Get peer PID of a channel
/// Args: channel_id
/// Returns: peer PID on success, negative error on failure
///
/// This allows protocol handlers to identify who is sending requests.
/// Combined with get_capabilities(), servers can verify caller permissions.
fn sys_channel_get_peer(channel_id: u32) -> i64 {
    let caller = current_pid();

    super::ipc::with_channel_table(|table| {
        // Verify caller owns this channel
        if table.get_owner(channel_id) != Some(caller) {
            return SyscallError::PermissionDenied as i64;
        }

        // Get peer owner
        match table.get_peer_owner(channel_id) {
            Some(peer_pid) => peer_pid as i64,
            None => SyscallError::NotFound as i64,
        }
    })
}

/// Reset/reboot the system using MT7988A watchdog
fn sys_reset() -> i64 {
    // Only processes with ALL capabilities can reset the system
    if let Err(e) = require_capability(Capabilities::ALL) {
        return e;
    }

    print_direct!("\n========================================\n");
    print_direct!("  System Reset Requested\n");
    print_direct!("========================================\n");

    // MT7988A TOPRGU (Top Reset Generation Unit) registers
    use crate::arch::aarch64::mmio::MmioRegion;
    use crate::arch::aarch64::mmu;
    use crate::platform::mt7988 as platform;

    let wdt = MmioRegion::new(mmu::phys_to_virt(platform::TOPRGU_BASE as u64) as usize);

    const WDT_MODE: usize = 0x00;
    const WDT_SWRST: usize = 0x14;

    // WDT_MODE bits
    const WDT_MODE_KEY: u32 = 0x2200_0000;  // Magic key for writes
    const WDT_MODE_EXTEN: u32 = 1 << 2;      // Enable external reset
    const WDT_MODE_EN: u32 = 1 << 0;         // Enable watchdog

    // WDT_SWRST key
    const WDT_SWRST_KEY: u32 = 0x1209;

    // Enable watchdog with external reset
    wdt.write32(WDT_MODE, WDT_MODE_KEY | WDT_MODE_EXTEN | WDT_MODE_EN);

    // Ensure the write completes
    crate::arch::aarch64::mmio::dsb();

    // Trigger software reset
    wdt.write32(WDT_SWRST, WDT_SWRST_KEY);

    // Ensure the write completes
    crate::arch::aarch64::mmio::dsb();

    // Should not reach here, but loop just in case
    kinfo!("sys_reset", "triggered");
    loop {
        unsafe { core::arch::asm!("wfi"); }
    }
}

// ============================================================================
// PCI Syscalls
// ============================================================================

/// Read PCI config space
/// bdf: device address (port:8 | bus:8 | device:5 | function:3)
/// offset: register offset (must be aligned)
/// size: 1, 2, or 4 bytes
/// Returns: value or -errno
fn sys_pci_config_read(bdf: u32, offset: u16, size: u8) -> i64 {
    use super::pci::{self, PciBdf};

    // Require RAW_DEVICE capability for PCI config access
    if let Err(e) = require_capability(Capabilities::RAW_DEVICE) {
        return e;
    }

    // Validate offset and size
    // PCIe extended config space is 4096 bytes (0x000-0xFFF)
    // Standard PCI config is 256 bytes (0x00-0xFF)
    const PCI_CONFIG_MAX: u16 = 4096;
    if offset >= PCI_CONFIG_MAX || (size != 1 && size != 2 && size != 4) {
        return SyscallError::InvalidArgument as i64;
    }
    if (offset as u32) + (size as u32) > PCI_CONFIG_MAX as u32 {
        return SyscallError::InvalidArgument as i64;
    }
    // Check alignment (2-byte access must be 2-aligned, 4-byte must be 4-aligned)
    if (size == 2 && (offset & 1) != 0) || (size == 4 && (offset & 3) != 0) {
        return SyscallError::InvalidArgument as i64;
    }

    let bdf = PciBdf::from_u32(bdf);

    // Check device exists and caller has access
    let dev = match pci::find_by_bdf(bdf) {
        Some(d) => d,
        None => return SyscallError::NotFound as i64,
    };

    // Check ownership (0 = unclaimed, anyone can read)
    let pid = current_pid();
    if dev.owner_pid != 0 && dev.owner_pid != pid {
        return SyscallError::PermissionDenied as i64;
    }

    match pci::config_read32(bdf, offset & !0x3) {
        Ok(val32) => {
            // Extract the requested portion
            match size {
                1 => {
                    let shift = (offset & 3) * 8;
                    ((val32 >> shift) & 0xFF) as i64
                }
                2 => {
                    let shift = (offset & 2) * 8;
                    ((val32 >> shift) & 0xFFFF) as i64
                }
                4 => val32 as i64,
                _ => SyscallError::InvalidArgument as i64,
            }
        }
        Err(_) => SyscallError::IoError as i64,
    }
}

/// Write PCI config space
/// bdf: device address
/// offset: register offset
/// size: 1, 2, or 4 bytes
/// value: value to write
/// Returns: 0 or -errno
fn sys_pci_config_write(bdf: u32, offset: u16, size: u8, value: u32) -> i64 {
    use super::pci::{self, PciBdf};

    // Require RAW_DEVICE capability for PCI config access
    if let Err(e) = require_capability(Capabilities::RAW_DEVICE) {
        return e;
    }

    // Validate offset and size
    // PCIe extended config space is 4096 bytes (0x000-0xFFF)
    const PCI_CONFIG_MAX: u16 = 4096;
    if offset >= PCI_CONFIG_MAX || (size != 1 && size != 2 && size != 4) {
        return SyscallError::InvalidArgument as i64;
    }
    if (offset as u32) + (size as u32) > PCI_CONFIG_MAX as u32 {
        return SyscallError::InvalidArgument as i64;
    }
    // Check alignment (2-byte access must be 2-aligned, 4-byte must be 4-aligned)
    if (size == 2 && (offset & 1) != 0) || (size == 4 && (offset & 3) != 0) {
        return SyscallError::InvalidArgument as i64;
    }

    let bdf = PciBdf::from_u32(bdf);

    // Check device exists and caller owns it
    let dev = match pci::find_by_bdf(bdf) {
        Some(d) => d,
        None => return SyscallError::NotFound as i64,
    };

    let pid = current_pid();
    if dev.owner_pid != pid {
        return SyscallError::PermissionDenied as i64;
    }

    // For partial writes, do read-modify-write
    let aligned_offset = offset & !0x3;

    let result = if size == 4 {
        pci::config_write32(bdf, aligned_offset, value)
    } else {
        // Read current value
        let current = match pci::config_read32(bdf, aligned_offset) {
            Ok(v) => v,
            Err(_) => return SyscallError::IoError as i64,
        };

        let (mask, shift) = match size {
            1 => (0xFFu32, ((offset & 3) * 8) as u32),
            2 => (0xFFFFu32, ((offset & 2) * 8) as u32),
            _ => return SyscallError::InvalidArgument as i64,
        };

        let new_val = (current & !(mask << shift)) | ((value & mask) << shift);
        pci::config_write32(bdf, aligned_offset, new_val)
    };

    match result {
        Ok(()) => 0,
        Err(_) => SyscallError::IoError as i64,
    }
}

/// Allocate MSI vector(s) for a device
/// bdf: device address
/// count: number of vectors requested (power of 2)
/// Returns: first IRQ number or -errno
fn sys_pci_msi_alloc(bdf: u32, count: u8) -> i64 {
    use super::pci::{self, PciBdf};

    // Require IRQ_CLAIM capability for MSI allocation
    if let Err(e) = require_capability(Capabilities::IRQ_CLAIM) {
        return e;
    }

    let bdf = PciBdf::from_u32(bdf);
    let pid = current_pid();

    // Check device exists and caller owns it
    let dev = match pci::find_by_bdf(bdf) {
        Some(d) => d,
        None => return SyscallError::NotFound as i64,
    };

    if dev.owner_pid != pid {
        return SyscallError::PermissionDenied as i64;
    }

    // Check device supports MSI
    if !dev.has_msi() && !dev.has_msix() {
        return SyscallError::NotImplemented as i64;
    }

    // Allocate vectors
    match pci::msi_alloc(bdf, count) {
        Ok(irq) => irq as i64,
        Err(pci::PciError::NoMsiVectors) => SyscallError::OutOfMemory as i64,
        Err(_) => SyscallError::IoError as i64,
    }
}

/// Map device MMIO into process address space
/// Generic syscall - kernel doesn't track devices, just maps physical memory
/// phys_addr: physical address of device MMIO region
/// size: size in bytes to map
/// Returns: virtual address on success, negative error on failure
fn sys_mmap_device(phys_addr: u64, size: u64) -> i64 {
    // Require MMIO capability for device memory mapping
    if let Err(e) = require_capability(Capabilities::MMIO) {
        return e;
    }

    // Validate size
    if size == 0 || size > 16 * 1024 * 1024 {
        // Max 16MB per mapping
        return SyscallError::InvalidArgument as i64;
    }

    // Map into calling process's address space
    let pid = current_pid();
    unsafe {
        let sched = super::task::scheduler();
        for task_opt in sched.tasks.iter_mut() {
            if let Some(ref mut task) = task_opt {
                if task.id == pid {
                    match task.mmap_device(phys_addr, size as usize) {
                        Some(vaddr) => return vaddr as i64,
                        None => return SyscallError::OutOfMemory as i64,
                    }
                }
            }
        }
    }

    SyscallError::NoProcess as i64
}

/// Allocate from DMA pool (low memory for PCIe devices)
/// Args: size, vaddr_ptr, paddr_ptr
/// Returns: 0 on success, negative error on failure
///
/// This allocates from a pre-reserved pool of low memory (0x40100000-0x40300000)
/// that some PCIe devices require for DMA operations. Use this instead of
/// shmem_create when the device can't access higher memory addresses.
fn sys_dma_pool_create(size: usize, vaddr_ptr: u64, paddr_ptr: u64) -> i64 {
    // Require DMA capability
    if let Err(e) = require_capability(Capabilities::DMA) {
        return e;
    }

    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate output pointers
    if vaddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(vaddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }
    if paddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(paddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }

    let caller_pid = current_pid();

    // Allocate from DMA pool
    let phys_addr = match super::dma_pool::alloc(size) {
        Ok(addr) => addr,
        Err(_) => return SyscallError::OutOfMemory as i64,
    };

    // Map into process address space
    let virt_addr = match super::dma_pool::map_into_process(caller_pid, phys_addr, size) {
        Ok(addr) => addr,
        Err(_) => return SyscallError::OutOfMemory as i64,
    };

    // Write output values
    if vaddr_ptr != 0 {
        if let Err(e) = uaccess::put_user::<u64>(vaddr_ptr, virt_addr) {
            return uaccess_to_errno(e);
        }
    }
    if paddr_ptr != 0 {
        if let Err(e) = uaccess::put_user::<u64>(paddr_ptr, phys_addr) {
            return uaccess_to_errno(e);
        }
    }

    0 // Success
}

/// Allocate from high DMA pool (36-bit addresses, > 4GB)
///
/// Same interface as sys_dma_pool_create but allocates from high memory
/// for devices that use 36-bit DMA addressing (like MT7996 TX buffers).
fn sys_dma_pool_create_high(size: usize, vaddr_ptr: u64, paddr_ptr: u64) -> i64 {
    if let Err(e) = require_capability(Capabilities::DMA) {
        return e;
    }

    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    if vaddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(vaddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }
    if paddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(paddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }

    let caller_pid = current_pid();

    // Allocate from HIGH DMA pool (36-bit addresses)
    let phys_addr = match super::dma_pool::alloc_high(size) {
        Ok(addr) => addr,
        Err(_) => return SyscallError::OutOfMemory as i64,
    };

    // Map into process address space
    let virt_addr = match super::dma_pool::map_into_process_high(caller_pid, phys_addr, size) {
        Ok(addr) => addr,
        Err(_) => return SyscallError::OutOfMemory as i64,
    };

    if vaddr_ptr != 0 {
        if let Err(e) = uaccess::put_user::<u64>(vaddr_ptr, virt_addr) {
            return uaccess_to_errno(e);
        }
    }
    if paddr_ptr != 0 {
        if let Err(e) = uaccess::put_user::<u64>(paddr_ptr, phys_addr) {
            return uaccess_to_errno(e);
        }
    }

    0
}

/// Allow a specific PID to send signals to this process.
///
/// By default (empty allowlist), all processes can send signals. Once at least
/// one PID is added to the allowlist, only those PIDs (plus the parent) can send.
///
/// Args: sender_pid - PID to allow signals from
/// Returns: 0 on success, negative error
fn sys_signal_allow(sender_pid: u32) -> i64 {
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if task.allow_signals_from(sender_pid) {
                return 0;
            } else {
                return SyscallError::OutOfMemory as i64; // Allowlist full (-12)
            }
        }
    }
    SyscallError::NoProcess as i64
}

/// Set a timer - delivers Timer event after duration_ns nanoseconds
/// Args: duration_ns (0 to cancel)
/// Returns: 0 on success, negative error
///
/// LEGACY API - uses timer slot 0 for backwards compatibility.
/// New code should use kevent_timer() for multiple timers and recurring support.
fn sys_timer_set(duration_ns: u64) -> i64 {
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if duration_ns == 0 {
                // Cancel timer 0
                task.timers[0].deadline = 0;
                task.timers[0].interval = 0;
                return 0;
            }

            // Timer runs at 100 Hz (10ms per tick)
            // Convert nanoseconds to ticks: duration_ns / 10_000_000
            let duration_ticks = duration_ns / 10_000_000;

            // Use the timer's tick_count (same counter that check_timeouts receives)
            let current_tick = crate::platform::mt7988::timer::ticks();

            // Set timer 0 as one-shot (backwards compatible)
            task.timers[0].id = 0;
            task.timers[0].interval = 0;  // One-shot
            task.timers[0].deadline = current_tick.saturating_add(duration_ticks.max(1));
            return 0;
        }
    }
    SyscallError::NoProcess as i64
}

/// Set or modify a timer with kevent-style API
/// Args: id (1-7 for user timers, 0 reserved), interval_ns, initial_ns
/// Returns: 0 on success, negative error
///
/// - id=0 with interval_ns=0 and initial_ns=0: cancel all timers
/// - id=1-7: set timer with given parameters
/// - interval_ns=0: one-shot timer (fires once, then deactivates)
/// - interval_ns>0: recurring timer (re-arms after each fire)
/// - initial_ns: time until first fire (0 = use interval for periodic timers)
fn sys_kevent_timer(id: u32, interval_ns: u64, initial_ns: u64) -> i64 {
    // Validate timer ID (1-7 for user, 0 special)
    if id >= super::task::MAX_TIMERS_PER_TASK as u32 {
        return SyscallError::InvalidArgument as i64;
    }

    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            // Special case: id=0 with both values 0 = cancel all
            if id == 0 && interval_ns == 0 && initial_ns == 0 {
                for timer in task.timers.iter_mut() {
                    timer.deadline = 0;
                    timer.interval = 0;
                }
                return 0;
            }

            // For id=0, treat it as legacy timer (one-shot only)
            // New API uses id 1-7

            // Convert nanoseconds to ticks (100 Hz = 10ms per tick)
            let interval_ticks = interval_ns / 10_000_000;
            let initial_ticks = if initial_ns > 0 {
                initial_ns / 10_000_000
            } else if interval_ticks > 0 {
                interval_ticks  // Periodic with no initial = use interval
            } else {
                return SyscallError::InvalidArgument as i64; // One-shot needs initial_ns
            };

            let current_tick = crate::platform::mt7988::timer::ticks();

            task.timers[id as usize].id = id;
            task.timers[id as usize].interval = interval_ticks;
            task.timers[id as usize].deadline = current_tick.saturating_add(initial_ticks.max(1));
            return 0;
        }
    }
    SyscallError::NoProcess as i64
}

/// Subscribe to events using kevent-style unified filter
/// Args: filter_type (1=Ipc, 2=Timer, 3=Irq, etc.), filter_value (channel_id, timer_id, etc.)
/// Returns: 0 on success, negative error
///
/// This wraps the existing event subscription system with a cleaner API.
/// Internally converts EventFilter to EventType for the subscription.
fn sys_kevent_subscribe(filter_type: u32, filter_value: u32) -> i64 {
    use super::event::EventFilter;

    // Validate and convert filter
    let filter = match EventFilter::from_type_and_value(filter_type, filter_value) {
        Some(f) => f,
        None => return SyscallError::InvalidArgument as i64,
    };

    // Check capabilities for privileged event types
    let pid = current_pid();
    if filter_type == 3 {  // IRQ
        if let Err(e) = require_capability(Capabilities::IRQ_CLAIM) {
            super::security_log::log_capability_denied(pid, "IRQ_CLAIM", "kevent_subscribe");
            return e;
        }
    }

    // Subscribe using the event type and filter value
    let ev_type = filter.to_event_type();
    let filter_val = filter.filter_value();

    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            match task.event_queue.subscribe(ev_type, filter_val) {
                Ok(()) => return 0,
                Err(e) => return e,
            }
        }
    }
    SyscallError::NoProcess as i64
}

/// Unsubscribe from events using kevent-style unified filter
/// Args: filter_type, filter_value
/// Returns: 0 on success, negative error
fn sys_kevent_unsubscribe(filter_type: u32, filter_value: u32) -> i64 {
    use super::event::EventFilter;

    let filter = match EventFilter::from_type_and_value(filter_type, filter_value) {
        Some(f) => f,
        None => return SyscallError::InvalidArgument as i64,
    };

    let ev_type = filter.to_event_type();
    let filter_val = filter.filter_value();

    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if task.event_queue.unsubscribe(ev_type, filter_val) {
                return 0;
            }
        }
    }
    SyscallError::NotFound as i64
}

/// Wait for events with batch receive (kevent-style)
/// Args: events_ptr (pointer to Event array), max_events, timeout_ns
/// Returns: number of events received, or negative error
///
/// timeout_ns semantics:
/// - 0: poll (return immediately with whatever events are available)
/// - u64::MAX: block forever until at least one event
/// - other: block up to timeout_ns nanoseconds
///
/// This drains up to max_events from the queue in one call, reducing
/// context switch overhead compared to single-event sys_event_wait.
fn sys_kevent_wait(events_ptr: u64, max_events: u32, timeout_ns: u64) -> i64 {
    if max_events == 0 {
        return 0;  // Nothing to do
    }

    // Limit max_events to prevent excessive copying
    let max = core::cmp::min(max_events as usize, 32);

    // Validate user buffer for writing
    let event_size = core::mem::size_of::<super::event::Event>();
    let buf_size = max * event_size;
    if let Err(e) = uaccess::validate_user_write(events_ptr, buf_size) {
        return uaccess_to_errno(e);
    }

    let caller_slot = super::task::current_slot();

    // First try: drain available events
    let mut count = 0usize;
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[caller_slot] {
            while count < max {
                if let Some(event) = task.event_queue.pop() {
                    // Write event to user buffer
                    let offset = count * event_size;
                    let event_bytes = core::slice::from_raw_parts(
                        &event as *const super::event::Event as *const u8,
                        event_size
                    );
                    if let Err(_) = uaccess::copy_to_user(events_ptr + offset as u64, event_bytes) {
                        if count > 0 {
                            return count as i64;  // Return what we have
                        }
                        return SyscallError::BadAddress as i64;
                    }
                    count += 1;
                } else {
                    break;  // No more events
                }
            }
        } else {
            return SyscallError::NoProcess as i64;
        }
    }

    // If we got events, return immediately
    if count > 0 {
        return count as i64;
    }

    // No events available - check timeout
    if timeout_ns == 0 {
        // Poll mode - return 0 (no events)
        return 0;
    }

    // Block waiting for events
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[caller_slot] {
            task.state = super::task::TaskState::Blocked;
            task.wait_reason = Some(super::task::WaitReason::Event);

            // Set wake_at if timeout is not "forever"
            if timeout_ns != u64::MAX {
                // Convert timeout_ns to ticks (100 Hz = 10ms per tick)
                let timeout_ticks = timeout_ns / 10_000_000;
                let current_tick = crate::platform::mt7988::timer::ticks();
                task.wake_at = current_tick.saturating_add(timeout_ticks.max(1));
            }

            // Pre-store return value (0 = timeout with no events)
            // If woken by event, we'll re-drain and update x0
            task.trap_frame.x0 = 0;
        }

        // Switch to another task
        if let Some(next_slot) = sched.schedule() {
            if next_slot != caller_slot {
                super::task::set_current_slot(next_slot);
                sched.current = next_slot;
                if let Some(ref mut next) = sched.tasks[next_slot] {
                    next.state = super::task::TaskState::Running;
                }
                super::task::update_current_task_globals();
                super::task::SYSCALL_SWITCHED_TASK.store(1, core::sync::atomic::Ordering::Release);
            }
        }
    }

    // When we return here after being woken, svc_handler will eret with x0 from trap_frame
    // The wake path should have set x0 appropriately
    0
}

/// Send heartbeat - updates last_heartbeat timestamp for monitoring
/// Returns: 0 on success, negative error
fn sys_heartbeat() -> i64 {
    unsafe {
        let sched = super::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            // Update last_heartbeat to current tick
            let current_tick = crate::platform::mt7988::timer::ticks();
            task.last_heartbeat = current_tick;
            return 0;
        }
    }
    SyscallError::NoProcess as i64
}

/// List available buses
/// Args: buf_ptr (pointer to BusInfo array), max_count
/// Returns: number of buses written, or negative error
fn sys_bus_list(buf_ptr: u64, max_count: u64) -> i64 {
    use super::bus::{BusInfo, get_bus_list};

    if buf_ptr == 0 {
        // Just return count
        return super::bus::get_bus_count() as i64;
    }

    let max = max_count as usize;
    if max == 0 {
        return 0;
    }

    // Validate user buffer for writing
    let buf_size = max * core::mem::size_of::<BusInfo>();
    if super::uaccess::validate_user_write(buf_ptr, buf_size).is_err() {
        return SyscallError::BadAddress as i64;
    }

    // Create temporary buffer on stack (max 16 buses)
    let mut temp = [BusInfo::empty(); 16];
    let count = get_bus_list(&mut temp[..max.min(16)]);

    // Copy to userspace
    unsafe {
        let user_buf = buf_ptr as *mut BusInfo;
        for i in 0..count {
            core::ptr::write(user_buf.add(i), temp[i]);
        }
    }

    count as i64
}

/// Syscall handler called from exception vector (assembly)
/// This is the entry point from the SVC handler in boot.S
#[no_mangle]
pub extern "C" fn syscall_handler_rust(
    arg0: u64, arg1: u64, arg2: u64, arg3: u64, arg4: u64, arg5: u64, _unused: u64, num: u64
) -> i64 {
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

/// Test syscall handling
#[allow(dead_code)] // Test infrastructure
pub fn test() {
    print_direct!("  Testing syscall infrastructure...\n");

    // Test debug write - uses kernel addresses during boot, so skip validation test
    let msg = "Hello from syscall!\n";
    // Note: This works during kernel init because we're using kernel pointers
    // In real user mode, user pointers would be validated
    let result = sys_debug_write(msg.as_ptr() as u64, msg.len());
    print_direct!("    debug_write returned: {}\n", result);

    // Test getpid
    let pid = sys_getpid();
    print_direct!("    getpid returned: {}\n", pid);

    // Test gettime
    let time = sys_gettime();
    print_direct!("    gettime returned: {} ns\n", time);

    // Test user address validation
    print_direct!("    Testing address validation...\n");
    assert!(uaccess::is_user_address(0x4000_0000));
    assert!(!uaccess::is_user_address(0xFFFF_0000_0000_0000));
    assert!(!uaccess::is_user_address(0)); // Null pointer
    print_direct!("    Address validation: OK\n");

    print_direct!("    [OK] Syscall infrastructure ready\n");
}

// =============================================================================
// Shared Memory Syscalls
// =============================================================================

/// Create a new shared memory region
/// Args: size, vaddr_ptr, paddr_ptr
/// Returns: shmem_id on success, negative error on failure
fn sys_shmem_create(size: usize, vaddr_ptr: u64, paddr_ptr: u64) -> i64 {
    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate output pointers
    if vaddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(vaddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }
    if paddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(paddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }

    let caller_pid = current_pid();

    match super::shmem::create(caller_pid, size) {
        Ok((shmem_id, vaddr, paddr)) => {
            // Write output values
            if vaddr_ptr != 0 {
                if let Err(e) = uaccess::put_user::<u64>(vaddr_ptr, vaddr) {
                    return uaccess_to_errno(e);
                }
            }
            if paddr_ptr != 0 {
                if let Err(e) = uaccess::put_user::<u64>(paddr_ptr, paddr) {
                    return uaccess_to_errno(e);
                }
            }
            shmem_id as i64
        }
        Err(e) => e,
    }
}

/// Map an existing shared memory region
/// Args: shmem_id, vaddr_ptr, paddr_ptr
/// Returns: 0 on success, negative error on failure
fn sys_shmem_map(shmem_id: u32, vaddr_ptr: u64, paddr_ptr: u64) -> i64 {
    // Validate output pointers
    if vaddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(vaddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }
    if paddr_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(paddr_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }

    let caller_pid = current_pid();

    match super::shmem::map(caller_pid, shmem_id) {
        Ok((vaddr, paddr)) => {
            // Write output values
            if vaddr_ptr != 0 {
                if let Err(e) = uaccess::put_user::<u64>(vaddr_ptr, vaddr) {
                    return uaccess_to_errno(e);
                }
            }
            if paddr_ptr != 0 {
                if let Err(e) = uaccess::put_user::<u64>(paddr_ptr, paddr) {
                    return uaccess_to_errno(e);
                }
            }
            0
        }
        Err(e) => e,
    }
}

/// Allow another process to map shared memory
/// Args: shmem_id, peer_pid
/// Returns: 0 on success, negative error on failure
fn sys_shmem_allow(shmem_id: u32, peer_pid: u32) -> i64 {
    let caller_pid = current_pid();

    match super::shmem::allow(caller_pid, shmem_id, peer_pid) {
        Ok(()) => 0,
        Err(e) => e,
    }
}

/// Wait for shared memory notification
/// Args: shmem_id, timeout_ms
/// Returns: 0 on notify, negative error on failure/timeout
fn sys_shmem_wait(shmem_id: u32, timeout_ms: u32) -> i64 {
    let caller_pid = current_pid();

    match super::shmem::wait(caller_pid, shmem_id, timeout_ms) {
        Ok(()) => 0,
        Err(-11) => {
            // EAGAIN = blocked, pre-store success return value
            // When woken by notify, task will resume and see 0
            unsafe {
                let sched = super::task::scheduler();
                if let Some(ref mut task) = sched.tasks[sched.current] {
                    task.trap_frame.x0 = 0;
                }
            }
            SyscallError::WouldBlock as i64
        }
        Err(e) => e,
    }
}

/// Notify shared memory waiters
/// Args: shmem_id
/// Returns: number of waiters woken, or negative error
fn sys_shmem_notify(shmem_id: u32) -> i64 {
    let caller_pid = current_pid();

    match super::shmem::notify(caller_pid, shmem_id) {
        Ok(woken) => woken as i64,
        Err(e) => e,
    }
}

/// Destroy a shared memory region
/// Args: shmem_id
/// Returns: 0 on success, negative error on failure
fn sys_shmem_destroy(shmem_id: u32) -> i64 {
    let caller_pid = current_pid();

    match super::shmem::destroy(caller_pid, shmem_id) {
        Ok(()) => 0,
        Err(e) => e,
    }
}

/// Unmap a shared memory region from this process
/// Args: shmem_id
/// Returns: 0 on success, negative error on failure
fn sys_shmem_unmap(shmem_id: u32) -> i64 {
    let caller_pid = current_pid();

    match super::shmem::unmap(caller_pid, shmem_id) {
        Ok(()) => 0,
        Err(e) => e,
    }
}

/// Ramfs directory entry for userspace (fixed size for easy iteration)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RamfsListEntry {
    /// Filename (null-terminated, max 100 bytes)
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
fn sys_ramfs_list(buf_ptr: u64, buf_len: usize) -> i64 {
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
fn sys_cpu_stats(buf_ptr: u64, buf_len: usize) -> i64 {
    use super::percpu::{MAX_CPUS, cpu_local};

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
