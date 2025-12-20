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

use crate::println;
use crate::uaccess::{self, UAccessError};

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
            _ => SyscallNumber::Invalid,
        }
    }
}

/// Syscall error codes (POSIX-like)
#[repr(i64)]
#[derive(Debug, Clone, Copy)]
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
    e.to_errno()
}

/// Handle a syscall
/// Returns the result to be placed in x0
pub fn handle(args: &SyscallArgs) -> i64 {
    let syscall = SyscallNumber::from(args.num);

    match syscall {
        SyscallNumber::Exit => sys_exit(args.arg0 as i32),
        SyscallNumber::DebugWrite => sys_debug_write(args.arg0, args.arg1 as usize),
        SyscallNumber::Yield => sys_yield(),
        SyscallNumber::GetPid => sys_getpid(),
        SyscallNumber::GetTime => sys_gettime(),
        SyscallNumber::Mmap => sys_mmap(args.arg0, args.arg1 as usize, args.arg2 as u32),
        SyscallNumber::Munmap => sys_munmap(args.arg0, args.arg1 as usize),
        SyscallNumber::Send => sys_send(args.arg0 as u32, args.arg1, args.arg2 as usize),
        SyscallNumber::Receive => sys_receive(args.arg0 as u32, args.arg1, args.arg2 as usize),
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
            crate::event::sys_event_subscribe(args.arg0 as u32, args.arg1, current_pid())
        }
        SyscallNumber::EventUnsubscribe => {
            crate::event::sys_event_unsubscribe(args.arg0 as u32, args.arg1, current_pid())
        }
        SyscallNumber::EventWait => {
            sys_event_wait(args.arg0, args.arg1 as u32)
        }
        SyscallNumber::EventPost => {
            crate::event::sys_event_post(args.arg0 as u32, args.arg1 as u32, args.arg2, current_pid())
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
        SyscallNumber::Wait => sys_wait(args.arg0 as i32, args.arg1),
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
        SyscallNumber::Invalid => {
            println!("[SYSCALL] Invalid syscall number: {}", args.num);
            SyscallError::NotImplemented as i64
        }
    }
}

/// Get current process ID from scheduler
fn current_pid() -> u32 {
    unsafe {
        crate::task::scheduler().current_task_id().unwrap_or(1)
    }
}

/// Exit current process
fn sys_exit(code: i32) -> i64 {
    println!();
    println!("========================================");
    println!("  Process exited with code: {}", code);
    println!("========================================");

    unsafe {
        let sched = crate::task::scheduler();
        let current_slot = sched.current;

        // Get current task's info before terminating
        let (pid, parent_id) = if let Some(ref task) = sched.tasks[current_slot] {
            (task.id, task.parent_id)
        } else {
            (0, 0)
        };

        // Set exit code and mark as terminated
        if let Some(ref mut task) = sched.tasks[current_slot] {
            task.exit_code = code;
            task.state = crate::task::TaskState::Terminated;
        }

        // Wake up parent if it's blocked (waiting for children)
        if parent_id != 0 {
            for task_opt in sched.tasks.iter_mut() {
                if let Some(ref mut task) = task_opt {
                    if task.id == parent_id && task.state == crate::task::TaskState::Blocked {
                        // Wake up the parent
                        task.state = crate::task::TaskState::Ready;
                        // Send ChildExit event
                        let event = crate::event::Event::child_exit(pid, code);
                        task.event_queue.push(event);
                        break;
                    }
                }
            }
        } else {
            // Orphan process (no parent) - auto-reap immediately
            // No one will wait() on it, so free the slot now
            sched.tasks[current_slot] = None;
        }

        // Try to schedule next task
        if let Some(next_slot) = sched.schedule() {
            // Mark next task as running and update globals
            sched.current = next_slot;
            if let Some(ref mut task) = sched.tasks[next_slot] {
                task.state = crate::task::TaskState::Running;
            }
            crate::task::update_current_task_globals();
            println!("  Switching to task {}", next_slot);
            // Return - svc_handler will load new task's state and eret
            0
        } else {
            // No more tasks - halt
            println!("  No more processes - halting.");
            loop {
                core::arch::asm!("wfi");
            }
        }
    }
}

/// Yield CPU to another process
/// If no other task is ready, waits for an interrupt (WFI) before returning.
/// This ensures yield() actually pauses when used for polling delays.
fn sys_yield() -> i64 {
    unsafe {
        let sched = crate::task::scheduler();
        let caller_slot = sched.current;

        // Mark current as ready (not running)
        if let Some(ref mut task) = sched.tasks[caller_slot] {
            task.state = crate::task::TaskState::Ready;
            // Pre-store return value in caller's trap frame
            task.trap_frame.x0 = 0;
        }

        // Find next task
        if let Some(next_slot) = sched.schedule() {
            if next_slot != caller_slot {
                sched.current = next_slot;
                if let Some(ref mut task) = sched.tasks[next_slot] {
                    task.state = crate::task::TaskState::Running;
                }
                crate::task::update_current_task_globals();
                // Signal to assembly not to store return value
                crate::task::SYSCALL_SWITCHED_TASK = 1;
            } else {
                // Same task - no other task ready to run
                // Use WFI to actually wait for an interrupt before returning
                // This makes yield() useful for polling loops that need real delays
                core::arch::asm!("wfi");

                // Mark as running again after waking from WFI
                if let Some(ref mut task) = sched.tasks[caller_slot] {
                    task.state = crate::task::TaskState::Running;
                }
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
        crate::uart::putc(byte as char);
    }

    len as i64
}

/// Get current process ID
fn sys_getpid() -> i64 {
    current_pid() as i64
}

/// Get system time in nanoseconds
fn sys_gettime() -> i64 {
    let counter = crate::timer::counter();
    let freq = crate::timer::frequency();
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
        let sched = crate::task::scheduler();
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
/// Args: size, phys_ptr (pointer to u64 for physical address)
/// Returns: virtual address on success, writes physical address to phys_ptr
/// SECURITY: Writes to user pointer via page table translation
fn sys_mmap_dma(size: usize, phys_ptr: u64) -> i64 {
    if size == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    // Validate phys_ptr
    if phys_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(phys_ptr, core::mem::size_of::<u64>()) {
            return uaccess_to_errno(e);
        }
    }

    unsafe {
        let sched = crate::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            match task.mmap_dma(size) {
                Some((virt_addr, phys_addr)) => {
                    // Write physical address to user pointer
                    if phys_ptr != 0 {
                        if let Err(e) = uaccess::put_user::<u64>(phys_ptr, phys_addr) {
                            // Unmap the allocation since we couldn't return the phys addr
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
        let sched = crate::task::scheduler();
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
    if data_len > crate::ipc::MAX_INLINE_PAYLOAD {
        return SyscallError::InvalidArgument as i64;
    }

    let caller_pid = current_pid();

    // Copy from user space if we have data
    let mut kernel_buf = [0u8; crate::ipc::MAX_INLINE_PAYLOAD];
    if data_len > 0 {
        match uaccess::copy_from_user(&mut kernel_buf[..data_len], data_ptr) {
            Ok(_) => {}
            Err(e) => return uaccess_to_errno(e),
        }
    }

    // Send via IPC
    let data = if data_len > 0 { &kernel_buf[..data_len] } else { &[] };
    match crate::ipc::sys_send(channel_id, caller_pid, data) {
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

    match crate::ipc::sys_receive(channel_id, caller_pid) {
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
        Err(e) => e.to_errno() as i64,
    }
}

/// Create a channel pair
/// Returns: channel_id_a in low 32 bits, channel_id_b in high 32 bits
fn sys_channel_create() -> i64 {
    let caller_pid = current_pid();

    match crate::ipc::sys_channel_create(caller_pid) {
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

    match crate::ipc::sys_channel_close(channel_id, caller_pid) {
        Ok(()) => SyscallError::Success as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Transfer a channel to another process
fn sys_channel_transfer(channel_id: u32, to_pid: u32) -> i64 {
    let caller_pid = current_pid();

    match crate::ipc::sys_channel_transfer(channel_id, caller_pid, to_pid) {
        Ok(()) => SyscallError::Success as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Register a named port
/// SECURITY: Copies user name buffer via page table translation
fn sys_port_register(name_ptr: u64, name_len: usize) -> i64 {
    // Validate length
    if name_len == 0 || name_len > crate::port::MAX_PORT_NAME {
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
    match crate::port::sys_port_register_buf(&name_buf[..name_len], caller_pid) {
        Ok(channel_id) => channel_id as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Unregister a named port
/// SECURITY: Copies user name buffer via page table translation
fn sys_port_unregister(name_ptr: u64, name_len: usize) -> i64 {
    // Validate length
    if name_len == 0 || name_len > crate::port::MAX_PORT_NAME {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy name from user space
    let mut name_buf = [0u8; 64];
    match uaccess::copy_from_user(&mut name_buf[..name_len], name_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    let caller_pid = current_pid();

    match crate::port::sys_port_unregister_buf(&name_buf[..name_len], caller_pid) {
        Ok(()) => SyscallError::Success as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Connect to a named port
/// SECURITY: Copies user name buffer via page table translation
fn sys_port_connect(name_ptr: u64, name_len: usize) -> i64 {
    // Validate length
    if name_len == 0 || name_len > crate::port::MAX_PORT_NAME {
        return SyscallError::InvalidArgument as i64;
    }

    // Copy name from user space
    let mut name_buf = [0u8; 64];
    match uaccess::copy_from_user(&mut name_buf[..name_len], name_ptr) {
        Ok(_) => {}
        Err(e) => return uaccess_to_errno(e),
    }

    let caller_pid = current_pid();

    match crate::port::sys_port_connect_buf(&name_buf[..name_len], caller_pid) {
        Ok(channel_id) => channel_id as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Accept a connection on a port
fn sys_port_accept(listen_channel: u32) -> i64 {
    let caller_pid = current_pid();

    match crate::port::sys_port_accept(listen_channel, caller_pid) {
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
    let entry = match crate::fd::open_path(&path_buf[..path_len], flags) {
        Some(e) => e,
        None => return SyscallError::NotFound as i64,
    };

    // Allocate an FD in the current process
    unsafe {
        let sched = crate::task::scheduler();
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
    unsafe {
        let sched = crate::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if task.fd_table.close(fd) {
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
    let bytes_read: isize;

    unsafe {
        let sched = crate::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            let caller_pid = task.id;
            if let Some(entry) = task.fd_table.get(fd) {
                bytes_read = crate::fd::fd_read(entry, &mut kernel_buf[..buf_len], caller_pid);
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
        let sched = crate::task::scheduler();
        if let Some(ref task) = sched.tasks[sched.current] {
            let caller_pid = task.id;
            if let Some(entry) = task.fd_table.get(fd) {
                return crate::fd::fd_write(entry, &kernel_buf[..buf_len], caller_pid) as i64;
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
        let sched = crate::task::scheduler();
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
    unsafe {
        let sched = crate::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            if task.fd_table.dup2(old_fd, new_fd) {
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
        let sched = crate::task::scheduler();
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
                            crate::fd::FdType::Ramfs { size, .. } => size as i64,
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
    match crate::scheme::open_url(&url_buf[..url_len], flags) {
        Ok((fd_entry, _handle)) => {
            // Allocate FD and store entry
            unsafe {
                let sched = crate::task::scheduler();
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
    if crate::scheme::get_kernel_scheme(name).is_some() {
        return SyscallError::AlreadyExists as i64;
    }

    // Register the user scheme
    unsafe {
        let reg = crate::scheme::registry();
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
        let reg = crate::scheme::registry();
        if let Some(entry) = reg.find(name) {
            // Verify ownership
            if entry.scheme_type != crate::scheme::SchemeType::User {
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
    let elf_data = match crate::elf::get_elf_by_id(elf_id) {
        Some(data) => data,
        None => return SyscallError::NotFound as i64,
    };

    // Parse name from kernel buffer
    let name = core::str::from_utf8(&name_buf[..name_len]).unwrap_or("child");

    // Get current PID as parent
    let parent_id = current_pid();

    // Spawn the child process
    match crate::elf::spawn_from_elf_with_parent(elf_data, name, parent_id) {
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
    match crate::elf::spawn_from_path_with_parent(path, parent_id) {
        Ok((child_id, _slot)) => child_id as i64,
        Err(crate::elf::ElfError::NotExecutable) => SyscallError::NotFound as i64,
        Err(_) => SyscallError::OutOfMemory as i64,
    }
}

/// Wait for a child process to exit
/// Args: pid (-1 = any child, >0 = specific child), status_ptr (pointer to i32 for exit status)
/// Returns: PID of exited child on success, 0 if no children/would block, negative error
/// SECURITY: Writes to user status pointer via page table translation
fn sys_wait(pid: i32, status_ptr: u64) -> i64 {
    let caller_pid = current_pid();

    // Validate status pointer if provided
    if status_ptr != 0 {
        if let Err(e) = uaccess::validate_user_write(status_ptr, core::mem::size_of::<i32>()) {
            return uaccess_to_errno(e);
        }
    }

    unsafe {
        let sched = crate::task::scheduler();

        // Find the calling task
        let caller_slot = sched.current;
        let caller_task = match &sched.tasks[caller_slot] {
            Some(t) if t.id == caller_pid => t,
            _ => return SyscallError::InvalidArgument as i64,
        };

        // Check if caller has any children
        if !caller_task.has_children() {
            return SyscallError::NoChild as i64;
        }

        // Look for terminated children
        for task_opt in sched.tasks.iter() {
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
                if task.state == crate::task::TaskState::Terminated {
                    let child_pid = task.id;
                    let exit_code = task.exit_code;

                    // Write exit status to user space if pointer provided
                    if status_ptr != 0 {
                        if let Err(e) = uaccess::put_user::<i32>(status_ptr, exit_code) {
                            return uaccess_to_errno(e);
                        }
                    }

                    // Remove child from parent's list and reap the task
                    // We need mutable access, so we'll do this in a second pass
                    let child_slot = sched.tasks.iter().position(|t| {
                        t.as_ref().map(|t| t.id == child_pid).unwrap_or(false)
                    });

                    if let Some(slot) = child_slot {
                        // Get mutable reference to parent and remove child
                        if let Some(ref mut parent) = sched.tasks[caller_slot] {
                            parent.remove_child(child_pid);
                        }

                        // Free the child task's resources
                        sched.tasks[slot] = None;
                    }

                    return child_pid as i64;
                }
            }
        }

        // No terminated children found - need to block and switch
        if let Some(ref mut parent) = sched.tasks[caller_slot] {
            parent.state = crate::task::TaskState::Blocked;
            // Pre-store return value in caller's trap frame
            parent.trap_frame.x0 = SyscallError::WouldBlock as i64 as u64;
        }

        // Find another task to run
        if let Some(next_slot) = sched.schedule() {
            if next_slot != caller_slot {
                sched.current = next_slot;
                if let Some(ref mut next) = sched.tasks[next_slot] {
                    next.state = crate::task::TaskState::Running;
                }
                crate::task::update_current_task_globals();
                crate::task::SYSCALL_SWITCHED_TASK = 1;
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
    let event_size = core::mem::size_of::<crate::event::Event>();
    if let Err(e) = uaccess::validate_user_write(event_buf, event_size) {
        return uaccess_to_errno(e);
    }

    unsafe {
        let sched = crate::task::scheduler();
        if let Some(ref mut task) = sched.tasks[sched.current] {
            // First check for pending events in global system
            let pid = task.id;
            if let Some(event) = crate::event::event_system().get_pending_for(pid) {
                // Copy event to user buffer via page table translation
                let event_bytes = core::slice::from_raw_parts(
                    &event as *const crate::event::Event as *const u8,
                    event_size
                );
                if let Err(e) = uaccess::copy_to_user(event_buf, event_bytes) {
                    return uaccess_to_errno(e);
                }
                return 1;
            }

            // Check task's event queue
            if let Some(event) = task.event_queue.pop() {
                let event_bytes = core::slice::from_raw_parts(
                    &event as *const crate::event::Event as *const u8,
                    event_size
                );
                if let Err(e) = uaccess::copy_to_user(event_buf, event_bytes) {
                    return uaccess_to_errno(e);
                }
                return 1;
            }

            // No event available
            if flags & 1 != 0 {
                // Non-blocking
                return 0;
            }

            // Would block - mark task as waiting
            task.state = crate::task::TaskState::Blocked;
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
    pub state: u8,      // 0=Ready, 1=Running, 2=Blocked, 3=Terminated
    pub _pad: [u8; 3],  // Alignment padding
    pub name: [u8; 16],
}

impl ProcessInfo {
    pub const fn empty() -> Self {
        Self {
            pid: 0,
            parent_pid: 0,
            state: 0,
            _pad: [0; 3],
            name: [0; 16],
        }
    }
}

/// Daemonize - detach from parent and become a daemon
fn sys_daemonize() -> i64 {
    let caller_pid = current_pid();

    unsafe {
        let sched = crate::task::scheduler();

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
                        if task.state == crate::task::TaskState::Blocked {
                            task.state = crate::task::TaskState::Ready;
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
fn sys_kill(pid: u32) -> i64 {
    if pid == 0 {
        return SyscallError::InvalidArgument as i64;
    }

    let caller_pid = current_pid();

    unsafe {
        let sched = crate::task::scheduler();

        // Find the target task
        let mut target_slot = None;
        let mut parent_id = 0u32;

        for (i, task_opt) in sched.tasks.iter().enumerate() {
            if let Some(ref task) = task_opt {
                if task.id == pid {
                    target_slot = Some(i);
                    parent_id = task.parent_id;
                    break;
                }
            }
        }

        let slot = match target_slot {
            Some(s) => s,
            None => return SyscallError::NoProcess as i64,
        };

        // Mark as terminated with SIGKILL-style exit code
        if let Some(ref mut task) = sched.tasks[slot] {
            task.exit_code = -9;  // SIGKILL
            task.state = crate::task::TaskState::Terminated;
        }

        // Wake parent if blocked (waiting on wait())
        if parent_id != 0 {
            for task_opt in sched.tasks.iter_mut() {
                if let Some(ref mut task) = task_opt {
                    if task.id == parent_id && task.state == crate::task::TaskState::Blocked {
                        task.state = crate::task::TaskState::Ready;
                        // Send ChildExit event
                        let event = crate::event::Event::child_exit(pid, -9);
                        task.event_queue.push(event);
                        break;
                    }
                }
            }
        } else {
            // Orphan process (no parent) - auto-reap immediately
            // No one will wait() on it, so free the slot now
            sched.tasks[slot] = None;
        }

        // If killing current task, schedule next
        if pid == caller_pid {
            if let Some(next_slot) = sched.schedule() {
                sched.current = next_slot;
                if let Some(ref mut next) = sched.tasks[next_slot] {
                    next.state = crate::task::TaskState::Running;
                }
                crate::task::update_current_task_globals();
                // Signal to assembly not to store return value into the new task
                crate::task::SYSCALL_SWITCHED_TASK = 1;
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
        let sched = crate::task::scheduler();

        for task_opt in sched.tasks.iter() {
            if count >= max_entries {
                break;
            }

            if let Some(ref task) = task_opt {
                let info = ProcessInfo {
                    pid: task.id,
                    parent_pid: task.parent_id,
                    state: task.state as u8,
                    _pad: [0; 3],
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

/// Reset/reboot the system using MT7988A watchdog
fn sys_reset() -> ! {
    println!();
    println!("========================================");
    println!("  System Reset Requested");
    println!("========================================");

    // MT7988A TOPRGU (Top Reset Generation Unit) registers
    // Physical address, accessed via kernel virtual mapping
    const TOPRGU_BASE: u64 = crate::mmu::KERNEL_VIRT_BASE | 0x1001_C000;
    const WDT_MODE: usize = 0x00;
    const WDT_SWRST: usize = 0x14;

    // WDT_MODE bits
    const WDT_MODE_KEY: u32 = 0x2200_0000;  // Magic key for writes
    const WDT_MODE_EXTEN: u32 = 1 << 2;      // Enable external reset
    const WDT_MODE_EN: u32 = 1 << 0;         // Enable watchdog

    // WDT_SWRST key
    const WDT_SWRST_KEY: u32 = 0x1209;

    unsafe {
        let wdt_mode = (TOPRGU_BASE + WDT_MODE as u64) as *mut u32;
        let wdt_swrst = (TOPRGU_BASE + WDT_SWRST as u64) as *mut u32;

        // Enable watchdog with external reset
        core::ptr::write_volatile(wdt_mode, WDT_MODE_KEY | WDT_MODE_EXTEN | WDT_MODE_EN);

        // Ensure the write completes
        core::arch::asm!("dsb sy");

        // Trigger software reset
        core::ptr::write_volatile(wdt_swrst, WDT_SWRST_KEY);

        // Ensure the write completes
        core::arch::asm!("dsb sy");
    }

    // Should not reach here, but loop just in case
    println!("Reset triggered, waiting...");
    loop {
        unsafe { core::arch::asm!("wfi"); }
    }
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

    handle(&args)
}

/// Test syscall handling
pub fn test() {
    println!("  Testing syscall infrastructure...");

    // Test debug write - uses kernel addresses during boot, so skip validation test
    let msg = "Hello from syscall!\n";
    // Note: This works during kernel init because we're using kernel pointers
    // In real user mode, user pointers would be validated
    let result = sys_debug_write(msg.as_ptr() as u64, msg.len());
    println!("    debug_write returned: {}", result);

    // Test getpid
    let pid = sys_getpid();
    println!("    getpid returned: {}", pid);

    // Test gettime
    let time = sys_gettime();
    println!("    gettime returned: {} ns", time);

    // Test user address validation
    println!("    Testing address validation...");
    assert!(uaccess::is_user_address(0x4000_0000));
    assert!(!uaccess::is_user_address(0xFFFF_0000_0000_0000));
    assert!(!uaccess::is_user_address(0)); // Null pointer
    println!("    Address validation: OK");

    println!("    [OK] Syscall infrastructure ready");
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

    match crate::shmem::create(caller_pid, size) {
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

    match crate::shmem::map(caller_pid, shmem_id) {
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

    match crate::shmem::allow(caller_pid, shmem_id, peer_pid) {
        Ok(()) => 0,
        Err(e) => e,
    }
}

/// Wait for shared memory notification
/// Args: shmem_id, timeout_ms
/// Returns: 0 on notify, negative error on failure/timeout
fn sys_shmem_wait(shmem_id: u32, timeout_ms: u32) -> i64 {
    let caller_pid = current_pid();

    match crate::shmem::wait(caller_pid, shmem_id, timeout_ms) {
        Ok(()) => 0,
        Err(e) => e,
    }
}

/// Notify shared memory waiters
/// Args: shmem_id
/// Returns: number of waiters woken, or negative error
fn sys_shmem_notify(shmem_id: u32) -> i64 {
    let caller_pid = current_pid();

    match crate::shmem::notify(caller_pid, shmem_id) {
        Ok(woken) => woken as i64,
        Err(e) => e,
    }
}

/// Destroy a shared memory region
/// Args: shmem_id
/// Returns: 0 on success, negative error on failure
fn sys_shmem_destroy(shmem_id: u32) -> i64 {
    let caller_pid = current_pid();

    match crate::shmem::destroy(caller_pid, shmem_id) {
        Ok(()) => 0,
        Err(e) => e,
    }
}
