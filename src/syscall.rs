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
        }

        // Try to schedule next task (don't reap yet - parent needs to wait())
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
fn sys_yield() -> i64 {
    unsafe {
        let sched = crate::task::scheduler();

        // Mark current as ready (not running)
        if let Some(ref mut task) = sched.tasks[sched.current] {
            task.state = crate::task::TaskState::Ready;
        }

        // Find next task
        if let Some(next_slot) = sched.schedule() {
            if next_slot != sched.current {
                sched.current = next_slot;
                if let Some(ref mut task) = sched.tasks[next_slot] {
                    task.state = crate::task::TaskState::Running;
                }
                crate::task::update_current_task_globals();
            } else {
                // Same task, mark as running again
                if let Some(ref mut task) = sched.tasks[sched.current] {
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
    let mut kernel_buf = [0u8; 256]; // MAX_INLINE_PAYLOAD
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
        if let Some(ref task) = sched.tasks[sched.current] {
            let caller_pid = task.id;
            if let Some(entry) = task.fd_table.get(fd) {
                bytes_read = crate::fd::fd_read(entry, &mut kernel_buf[..buf_len], caller_pid);
            } else {
                return SyscallError::BadFd as i64;
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

        // No terminated children found - need to block
        // Mark parent as blocked and switch to another task
        if let Some(ref mut parent) = sched.tasks[caller_slot] {
            parent.state = crate::task::TaskState::Blocked;
        }

        // Force a context switch to let child run
        // Find next runnable task
        if let Some(next_slot) = sched.schedule() {
            if next_slot != caller_slot {
                sched.current = next_slot;
                if let Some(ref mut next) = sched.tasks[next_slot] {
                    next.state = crate::task::TaskState::Running;
                }
                crate::task::update_current_task_globals();
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
