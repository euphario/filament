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

use crate::println;

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
            _ => SyscallNumber::Invalid,
        }
    }
}

/// Syscall error codes
#[repr(i64)]
#[derive(Debug, Clone, Copy)]
pub enum SyscallError {
    Success = 0,
    InvalidSyscall = -1,
    InvalidArgument = -2,
    OutOfMemory = -3,
    PermissionDenied = -4,
    NotFound = -5,
    WouldBlock = -6,
    BadAddress = -7,
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
        SyscallNumber::Spawn | SyscallNumber::Wait => {
            // Not implemented yet
            SyscallError::InvalidSyscall as i64
        }
        SyscallNumber::Invalid => {
            println!("[SYSCALL] Invalid syscall number: {}", args.num);
            SyscallError::InvalidSyscall as i64
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
        sched.terminate_current(code);
        sched.reap_terminated();

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
fn sys_debug_write(buf_ptr: u64, len: usize) -> i64 {
    // Safety: In a real implementation, we'd validate the pointer
    // is in user space and accessible
    if len == 0 || len > 1024 {
        return SyscallError::InvalidArgument as i64;
    }

    unsafe {
        let slice = core::slice::from_raw_parts(buf_ptr as *const u8, len);
        for &byte in slice {
            if byte == 0 {
                break;
            }
            crate::uart::putc(byte as char);
        }
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
fn sys_send(channel_id: u32, data_ptr: u64, data_len: usize) -> i64 {
    // Validate length
    if data_len > crate::ipc::MAX_INLINE_PAYLOAD {
        return SyscallError::InvalidArgument as i64;
    }

    let caller_pid = current_pid();

    // Read data from user space
    let data = if data_len > 0 && data_ptr != 0 {
        unsafe {
            core::slice::from_raw_parts(data_ptr as *const u8, data_len)
        }
    } else {
        &[]
    };

    // Send via IPC
    match crate::ipc::sys_send(channel_id, caller_pid, data) {
        Ok(()) => SyscallError::Success as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Receive IPC message from a channel
/// Args: channel_id, buf_ptr, buf_len
/// Returns: message length on success, negative error on failure
fn sys_receive(channel_id: u32, buf_ptr: u64, buf_len: usize) -> i64 {
    let caller_pid = current_pid();

    match crate::ipc::sys_receive(channel_id, caller_pid) {
        Ok(msg) => {
            // Copy message to user buffer
            let copy_len = core::cmp::min(msg.header.payload_len as usize, buf_len);
            if copy_len > 0 && buf_ptr != 0 {
                unsafe {
                    let dest = buf_ptr as *mut u8;
                    for i in 0..copy_len {
                        core::ptr::write_volatile(dest.add(i), msg.payload[i]);
                    }
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
fn sys_port_register(name_ptr: u64, name_len: usize) -> i64 {
    let caller_pid = current_pid();

    match crate::port::sys_port_register(name_ptr, name_len, caller_pid) {
        Ok(channel_id) => channel_id as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Unregister a named port
fn sys_port_unregister(name_ptr: u64, name_len: usize) -> i64 {
    let caller_pid = current_pid();

    match crate::port::sys_port_unregister(name_ptr, name_len, caller_pid) {
        Ok(()) => SyscallError::Success as i64,
        Err(e) => e.to_errno() as i64,
    }
}

/// Connect to a named port
fn sys_port_connect(name_ptr: u64, name_len: usize) -> i64 {
    let caller_pid = current_pid();

    match crate::port::sys_port_connect(name_ptr, name_len, caller_pid) {
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
fn sys_open(path_ptr: u64, path_len: usize, flags: u32) -> i64 {
    if path_len == 0 || path_len > 256 {
        return SyscallError::InvalidArgument as i64;
    }

    // Read path from user space
    let path = unsafe {
        core::slice::from_raw_parts(path_ptr as *const u8, path_len)
    };

    // Try to open the path
    let entry = match crate::fd::open_path(path, flags) {
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
                return SyscallError::OutOfMemory as i64; // No free FDs
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
    SyscallError::InvalidArgument as i64
}

/// Read from a file descriptor
/// Args: fd, buf_ptr, buf_len
/// Returns: bytes read on success, negative error on failure
fn sys_read(fd: u32, buf_ptr: u64, buf_len: usize) -> i64 {
    if buf_len == 0 {
        return 0;
    }

    unsafe {
        let sched = crate::task::scheduler();
        if let Some(ref task) = sched.tasks[sched.current] {
            if let Some(entry) = task.fd_table.get(fd) {
                // Create a mutable buffer slice in user space
                let buf = core::slice::from_raw_parts_mut(buf_ptr as *mut u8, buf_len);
                return crate::fd::fd_read(entry, buf) as i64;
            }
        }
    }
    SyscallError::InvalidArgument as i64
}

/// Write to a file descriptor
/// Args: fd, buf_ptr, buf_len
/// Returns: bytes written on success, negative error on failure
fn sys_write(fd: u32, buf_ptr: u64, buf_len: usize) -> i64 {
    if buf_len == 0 {
        return 0;
    }

    unsafe {
        let sched = crate::task::scheduler();
        if let Some(ref task) = sched.tasks[sched.current] {
            if let Some(entry) = task.fd_table.get(fd) {
                // Create a buffer slice from user space
                let buf = core::slice::from_raw_parts(buf_ptr as *const u8, buf_len);
                return crate::fd::fd_write(entry, buf) as i64;
            }
        }
    }
    SyscallError::InvalidArgument as i64
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
    SyscallError::InvalidArgument as i64
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

    // Test debug write
    let msg = "Hello from syscall!\n";
    let result = sys_debug_write(msg.as_ptr() as u64, msg.len());
    println!("    debug_write returned: {}", result);

    // Test getpid
    let pid = sys_getpid();
    println!("    getpid returned: {}", pid);

    // Test gettime
    let time = sys_gettime();
    println!("    gettime returned: {} ns", time);

    println!("    [OK] Syscall infrastructure ready");
}
