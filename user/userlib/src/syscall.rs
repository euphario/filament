//! System call wrappers
//!
//! These match the syscall numbers in the kernel's syscall.rs

use core::arch::asm;

// Syscall numbers (must match kernel)
pub const SYS_EXIT: u64 = 0;
pub const SYS_DEBUG_WRITE: u64 = 1;
pub const SYS_YIELD: u64 = 2;
pub const SYS_GETPID: u64 = 3;
pub const SYS_MMAP: u64 = 4;
pub const SYS_MUNMAP: u64 = 5;
pub const SYS_SEND: u64 = 6;
pub const SYS_RECEIVE: u64 = 7;
pub const SYS_SPAWN: u64 = 8;
pub const SYS_WAIT: u64 = 9;
pub const SYS_GETTIME: u64 = 10;
pub const SYS_CHANNEL_CREATE: u64 = 11;
pub const SYS_CHANNEL_CLOSE: u64 = 12;
pub const SYS_CHANNEL_TRANSFER: u64 = 13;
pub const SYS_PORT_REGISTER: u64 = 14;
pub const SYS_PORT_UNREGISTER: u64 = 15;
pub const SYS_PORT_CONNECT: u64 = 16;
pub const SYS_PORT_ACCEPT: u64 = 17;
pub const SYS_OPEN: u64 = 18;
pub const SYS_CLOSE: u64 = 19;
pub const SYS_READ: u64 = 20;
pub const SYS_WRITE: u64 = 21;
pub const SYS_DUP: u64 = 22;
pub const SYS_DUP2: u64 = 23;
pub const SYS_EVENT_SUBSCRIBE: u64 = 24;
pub const SYS_EVENT_UNSUBSCRIBE: u64 = 25;
pub const SYS_EVENT_WAIT: u64 = 26;
pub const SYS_EVENT_POST: u64 = 27;
pub const SYS_SCHEME_OPEN: u64 = 28;
pub const SYS_SCHEME_REGISTER: u64 = 29;
pub const SYS_SCHEME_UNREGISTER: u64 = 30;
pub const SYS_EXEC: u64 = 31;

// Raw syscall functions

#[inline(always)]
pub fn syscall0(num: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            lateout("x0") ret,
            // Clobber caller-saved registers that kernel might use
            lateout("x9") _,
            lateout("x10") _,
            lateout("x11") _,
            lateout("x12") _,
            lateout("x13") _,
            lateout("x14") _,
            lateout("x15") _,
            options(nostack)
        );
    }
    ret
}

#[inline(always)]
pub fn syscall1(num: u64, a0: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            inlateout("x0") a0 => ret,
            // Clobber caller-saved registers that kernel might use
            lateout("x9") _,
            lateout("x10") _,
            lateout("x11") _,
            lateout("x12") _,
            lateout("x13") _,
            lateout("x14") _,
            lateout("x15") _,
            options(nostack)
        );
    }
    ret
}

#[inline(always)]
pub fn syscall2(num: u64, a0: u64, a1: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            inlateout("x0") a0 => ret,
            in("x1") a1,
            // Clobber caller-saved registers that kernel might use
            lateout("x9") _,
            lateout("x10") _,
            lateout("x11") _,
            lateout("x12") _,
            lateout("x13") _,
            lateout("x14") _,
            lateout("x15") _,
            options(nostack)
        );
    }
    ret
}

#[inline(always)]
pub fn syscall3(num: u64, a0: u64, a1: u64, a2: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            inlateout("x0") a0 => ret,
            in("x1") a1,
            in("x2") a2,
            // Clobber caller-saved registers that kernel might use
            lateout("x9") _,
            lateout("x10") _,
            lateout("x11") _,
            lateout("x12") _,
            lateout("x13") _,
            lateout("x14") _,
            lateout("x15") _,
            options(nostack)
        );
    }
    ret
}

#[inline(always)]
pub fn syscall4(num: u64, a0: u64, a1: u64, a2: u64, a3: u64) -> i64 {
    let ret: i64;
    unsafe {
        asm!(
            "svc #0",
            in("x8") num,
            inlateout("x0") a0 => ret,
            in("x1") a1,
            in("x2") a2,
            in("x3") a3,
            // Clobber caller-saved registers that kernel might use
            lateout("x9") _,
            lateout("x10") _,
            lateout("x11") _,
            lateout("x12") _,
            lateout("x13") _,
            lateout("x14") _,
            lateout("x15") _,
            options(nostack)
        );
    }
    ret
}

// High-level syscall wrappers

/// Exit the current process
pub fn exit(code: i32) -> ! {
    syscall1(SYS_EXIT, code as u64);
    // Should never return, but just in case
    loop {
        unsafe { asm!("wfe") };
    }
}

/// Yield the CPU to other processes
pub fn yield_now() {
    syscall0(SYS_YIELD);
}

/// Get the current process ID
pub fn getpid() -> u32 {
    syscall0(SYS_GETPID) as u32
}

/// Read from a file descriptor
/// Returns number of bytes read, or negative error code
pub fn read(fd: u32, buf: &mut [u8]) -> isize {
    syscall3(SYS_READ, fd as u64, buf.as_mut_ptr() as u64, buf.len() as u64) as isize
}

/// Write to a file descriptor
/// Returns number of bytes written, or negative error code
pub fn write(fd: u32, buf: &[u8]) -> isize {
    syscall3(SYS_WRITE, fd as u64, buf.as_ptr() as u64, buf.len() as u64) as isize
}

/// Open a file/resource by path
/// Returns file descriptor, or negative error code
pub fn open(path: &[u8], flags: u32) -> i32 {
    syscall2(SYS_OPEN, path.as_ptr() as u64, flags as u64) as i32
}

/// Close a file descriptor
pub fn close(fd: u32) -> i32 {
    syscall1(SYS_CLOSE, fd as u64) as i32
}

/// Duplicate a file descriptor
pub fn dup(fd: u32) -> i32 {
    syscall1(SYS_DUP, fd as u64) as i32
}

/// Duplicate a file descriptor to a specific number
pub fn dup2(old_fd: u32, new_fd: u32) -> i32 {
    syscall2(SYS_DUP2, old_fd as u64, new_fd as u64) as i32
}

/// Map memory
/// Returns virtual address, or negative error code
pub fn mmap(addr: u64, size: usize, prot: u32) -> i64 {
    syscall3(SYS_MMAP, addr, size as u64, prot as u64)
}

/// Unmap memory
pub fn munmap(addr: u64, size: usize) -> i32 {
    syscall2(SYS_MUNMAP, addr, size as u64) as i32
}

/// Spawn a new process from built-in ELF ID (legacy)
/// Returns pid on success, or negative error code
pub fn spawn(elf_id: u32) -> i64 {
    syscall1(SYS_SPAWN, elf_id as u64)
}

/// Execute a program from a path (searches ramfs)
/// Returns pid on success, or negative error code
pub fn exec(path: &str) -> i64 {
    syscall2(SYS_EXEC, path.as_ptr() as u64, path.len() as u64)
}

/// Wait for a child process to exit
/// Returns (pid << 32 | exit_code) on success, or negative error code
pub fn wait(pid: i32) -> i64 {
    syscall1(SYS_WAIT, pid as u64)
}

// IPC syscalls

/// Create a channel pair
/// Returns (channel_a, channel_b) packed as i64, or negative error
pub fn channel_create() -> i64 {
    syscall0(SYS_CHANNEL_CREATE)
}

/// Close a channel
pub fn channel_close(channel_id: u32) -> i32 {
    syscall1(SYS_CHANNEL_CLOSE, channel_id as u64) as i32
}

/// Send data on a channel
pub fn send(channel_id: u32, data: &[u8]) -> i32 {
    syscall3(SYS_SEND, channel_id as u64, data.as_ptr() as u64, data.len() as u64) as i32
}

/// Receive data from a channel
pub fn receive(channel_id: u32, buf: &mut [u8]) -> isize {
    syscall3(SYS_RECEIVE, channel_id as u64, buf.as_mut_ptr() as u64, buf.len() as u64) as isize
}

// Port syscalls

/// Register a named port
pub fn port_register(name: &[u8]) -> i64 {
    syscall2(SYS_PORT_REGISTER, name.as_ptr() as u64, name.len() as u64)
}

/// Unregister a named port
pub fn port_unregister(name: &[u8]) -> i32 {
    syscall2(SYS_PORT_UNREGISTER, name.as_ptr() as u64, name.len() as u64) as i32
}

/// Connect to a named port
pub fn port_connect(name: &[u8]) -> i64 {
    syscall2(SYS_PORT_CONNECT, name.as_ptr() as u64, name.len() as u64)
}

/// Accept a connection on a port's listen channel
pub fn port_accept(listen_channel: u32) -> i64 {
    syscall1(SYS_PORT_ACCEPT, listen_channel as u64)
}

// Standard file descriptors
pub const STDIN: u32 = 0;
pub const STDOUT: u32 = 1;
pub const STDERR: u32 = 2;

// Open flags
pub const O_RDONLY: u32 = 0;
pub const O_WRONLY: u32 = 1;
pub const O_RDWR: u32 = 2;
pub const O_CREAT: u32 = 0x40;
pub const O_TRUNC: u32 = 0x200;
pub const O_APPEND: u32 = 0x400;
pub const O_NONBLOCK: u32 = 0x800;

// Memory protection flags
pub const PROT_READ: u32 = 1;
pub const PROT_WRITE: u32 = 2;
pub const PROT_EXEC: u32 = 4;
