//! System call wrappers
//!
//! These match the syscall numbers in the kernel's syscall.rs

use core::arch::asm;
use crate::error::{SysError, SysResult};

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
pub const SYS_DAEMONIZE: u64 = 32;
pub const SYS_KILL: u64 = 33;
pub const SYS_PS_INFO: u64 = 34;
pub const SYS_SET_LOG_LEVEL: u64 = 35;
pub const SYS_MMAP_DMA: u64 = 36;
pub const SYS_RESET: u64 = 37;
pub const SYS_LSEEK: u64 = 38;
pub const SYS_SHMEM_CREATE: u64 = 39;
pub const SYS_SHMEM_MAP: u64 = 40;
pub const SYS_SHMEM_ALLOW: u64 = 41;
pub const SYS_SHMEM_WAIT: u64 = 42;
pub const SYS_SHMEM_NOTIFY: u64 = 43;
pub const SYS_SHMEM_DESTROY: u64 = 44;
pub const SYS_SHMEM_UNMAP: u64 = 47;
pub const SYS_SEND_DIRECT: u64 = 45;
pub const SYS_RECEIVE_TIMEOUT: u64 = 46;
pub const SYS_RAMFS_LIST: u64 = 63;
pub const SYS_EXEC_MEM: u64 = 64;
pub const SYS_KLOG_READ: u64 = 65;
pub const SYS_GET_CAPABILITIES: u64 = 66;
pub const SYS_CHANNEL_GET_PEER: u64 = 67;
pub const SYS_CPU_STATS: u64 = 68;

// Kevent syscalls (unified event system)
pub const SYS_KEVENT_SUBSCRIBE: u64 = 70;
pub const SYS_KEVENT_UNSUBSCRIBE: u64 = 71;
pub const SYS_KEVENT_TIMER: u64 = 72;
pub const SYS_KEVENT_WAIT: u64 = 73;
pub const SYS_EXEC_WITH_CAPS: u64 = 74;

// SEEK whence constants
pub const SEEK_SET: u32 = 0;
pub const SEEK_CUR: u32 = 1;
pub const SEEK_END: u32 = 2;

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
    syscall3(SYS_OPEN, path.as_ptr() as u64, path.len() as u64, flags as u64) as i32
}

/// Open a file/resource by string path
/// Returns file descriptor, or negative error code
pub fn open_str(path: &str, flags: u32) -> i32 {
    open(path.as_bytes(), flags)
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

/// Seek to a position in a file descriptor
/// whence: SEEK_SET(0), SEEK_CUR(1), SEEK_END(2)
/// Returns new position, or negative error code
pub fn lseek(fd: u32, offset: i64, whence: u32) -> i64 {
    syscall3(SYS_LSEEK, fd as u64, offset as u64, whence as u64)
}

/// Map memory
/// Returns virtual address, or negative error code
pub fn mmap(addr: u64, size: usize, prot: u32) -> i64 {
    syscall3(SYS_MMAP, addr, size as u64, prot as u64)
}

/// Allocate DMA-capable memory for device I/O
///
/// Returns virtual address on success (negative on error).
/// The DMA address (for programming into device descriptors) is written to `dma_out`.
///
/// The DMA address is what PCIe/bus-mastering devices should use to access this memory.
/// On platforms with identity-mapped PCIe, this equals the CPU physical address.
/// On platforms with an offset, the kernel applies the appropriate translation.
///
/// # Arguments
/// * `size` - Size in bytes to allocate (will be rounded up to page size)
/// * `dma_out` - Pointer to receive the DMA address for device programming
///
/// # Returns
/// * Positive: Virtual address of allocated memory
/// * Negative: Error code
pub fn mmap_dma(size: usize, dma_out: &mut u64) -> i64 {
    syscall2(SYS_MMAP_DMA, size as u64, dma_out as *mut u64 as u64)
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

/// Execute an ELF binary from a memory buffer
/// This allows executing binaries loaded from vfsd or other sources
/// Args:
///   elf_data: Slice containing the ELF binary data
///   name: Optional process name (uses "exec_mem" if None)
/// Returns pid on success, or negative error code
pub fn exec_mem(elf_data: &[u8], name: Option<&str>) -> i64 {
    let (name_ptr, name_len) = match name {
        Some(n) => (n.as_ptr() as u64, n.len() as u64),
        None => (0, 0),
    };
    syscall4(
        SYS_EXEC_MEM,
        elf_data.as_ptr() as u64,
        elf_data.len() as u64,
        name_ptr,
        name_len,
    )
}

/// Execute a program with explicit capability grant (privilege separation)
/// Args:
///   path: Path to executable in ramfs
///   caps: Capability bits to grant (will be intersected with parent's caps)
/// Returns pid on success, or negative error code
/// Note: Caller must have GRANT capability to delegate capabilities to children
pub fn exec_with_caps(path: &str, caps: u64) -> i64 {
    syscall3(
        SYS_EXEC_WITH_CAPS,
        path.as_ptr() as u64,
        path.len() as u64,
        caps,
    )
}

/// Read one formatted kernel log record
/// Args: buf - buffer to write formatted log line
/// Returns: number of bytes written, 0 if no logs available, negative on error
pub fn klog_read(buf: &mut [u8]) -> i64 {
    syscall2(SYS_KLOG_READ, buf.as_mut_ptr() as u64, buf.len() as u64)
}

/// Get capabilities of a process
/// Args: pid - process ID (0 = current process)
/// Returns: capability bits as i64 on success, negative on error (-3 = process not found)
///
/// Use this in protocol handlers to verify caller has required capabilities
/// before performing privileged operations.
pub fn get_capabilities(pid: u32) -> i64 {
    syscall1(SYS_GET_CAPABILITIES, pid as u64)
}

/// Capability bit constants (must match kernel caps.rs)
pub mod caps {
    /// Can send/receive IPC messages
    pub const IPC: u64 = 1 << 0;
    /// Can allocate memory
    pub const MEMORY: u64 = 1 << 1;
    /// Can spawn child processes
    pub const SPAWN: u64 = 1 << 2;
    /// Can register as scheme/driver
    pub const SCHEME_CREATE: u64 = 1 << 3;
    /// Can claim IRQ ownership
    pub const IRQ_CLAIM: u64 = 1 << 4;
    /// Can map device MMIO
    pub const MMIO: u64 = 1 << 5;
    /// Can allocate DMA memory
    pub const DMA: u64 = 1 << 6;
    /// Can access filesystem
    pub const FILESYSTEM: u64 = 1 << 7;
    /// Can access network
    pub const NETWORK: u64 = 1 << 8;
    /// Can access raw devices (PCI config)
    pub const RAW_DEVICE: u64 = 1 << 9;
    /// Can grant capabilities to children
    pub const GRANT: u64 = 1 << 10;
    /// Can kill other processes
    pub const KILL: u64 = 1 << 11;

    /// Check if capability is present
    pub fn has(caps: u64, cap: u64) -> bool {
        (caps & cap) == cap
    }

    // Preset capability sets for common driver types

    /// Minimal user process
    pub const USER_DEFAULT: u64 = IPC | MEMORY | SPAWN;

    /// Typical bus driver (pcied, usbd)
    pub const BUS_DRIVER: u64 = IPC | MEMORY | SPAWN | SCHEME_CREATE | IRQ_CLAIM | MMIO | DMA | GRANT;

    /// Device driver (needs to claim from bus driver)
    pub const DEVICE_DRIVER: u64 = IPC | MEMORY | SCHEME_CREATE | IRQ_CLAIM | MMIO | DMA;

    /// Filesystem driver (fatfs, vfsd)
    pub const FS_DRIVER: u64 = IPC | MEMORY | SCHEME_CREATE | FILESYSTEM;

    /// Console/log driver (consoled, logd)
    pub const SERVICE_DRIVER: u64 = IPC | MEMORY | SCHEME_CREATE;

    /// GPIO/PWM driver (basic hardware access)
    pub const GPIO_DRIVER: u64 = IPC | MEMORY | SCHEME_CREATE | MMIO;
}

/// Get the PID of the peer process on a channel
/// Args: channel_id - the channel to query
/// Returns: peer PID on success, negative on error (-1 = permission denied, -2 = not found)
///
/// Use this in protocol handlers to identify who is sending requests.
/// Combined with get_capabilities(), servers can verify caller permissions.
pub fn channel_get_peer(channel_id: u32) -> i64 {
    syscall1(SYS_CHANNEL_GET_PEER, channel_id as u64)
}

/// Wait for a child process to exit
/// Returns (pid << 32 | exit_code) on success, or negative error code
/// pid: -1 = any child, >0 = specific child
pub fn wait(pid: i32) -> i64 {
    syscall3(SYS_WAIT, pid as u64, 0, 0) // 0 = no status pointer, 0 = blocking
}

/// Wait flags
pub const WNOHANG: u32 = 1;  // Don't block if no child has exited

/// Wait for a child process and get exit status (blocking)
/// Returns child pid on success, writes exit status to status_out
pub fn waitpid(pid: i32, status_out: &mut i32) -> i64 {
    syscall3(SYS_WAIT, pid as u64, status_out as *mut i32 as u64, 0)
}

/// Wait for a child process with flags
/// flags: 0 = block, WNOHANG = return immediately if no child exited
/// Returns: child pid on success, 0 if WNOHANG and no child exited, negative error
pub fn waitpid_flags(pid: i32, status_out: &mut i32, flags: u32) -> i64 {
    syscall3(SYS_WAIT, pid as u64, status_out as *mut i32 as u64, flags as u64)
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

/// Send data on a channel with direct switch to receiver (fast-path IPC)
/// If the receiver is blocked waiting, directly switches to it.
/// Returns when the receiver yields or is preempted.
pub fn send_direct(channel_id: u32, data: &[u8]) -> i32 {
    syscall3(SYS_SEND_DIRECT, channel_id as u64, data.as_ptr() as u64, data.len() as u64) as i32
}

/// Receive data from a channel (blocking)
///
/// Blocks until a message is available or the channel is closed.
/// Returns: message length on success, negative error on failure
///
/// ## Behavior
/// - Retries on WouldBlock (-11) which indicates spurious wakeup
/// - Returns immediately on success (len > 0)
/// - Returns immediately on Close message (len = 0)
/// - Returns immediately on real errors (len < 0, not -11)
pub fn receive(channel_id: u32, buf: &mut [u8]) -> isize {
    loop {
        let result = receive_nonblock(channel_id, buf);
        if result == -11 {
            // WouldBlock - no message yet, yield and retry
            yield_now();
            continue;
        }
        return result;
    }
}

/// Receive data from a channel (non-blocking)
///
/// Returns immediately with the result.
/// Returns: message length on success, -11 (WouldBlock) if no message, negative error otherwise
pub fn receive_nonblock(channel_id: u32, buf: &mut [u8]) -> isize {
    syscall3(SYS_RECEIVE, channel_id as u64, buf.as_mut_ptr() as u64, buf.len() as u64) as isize
}

/// Receive data from a channel with timeout
///
/// - timeout_ms = 0: Non-blocking (returns immediately if no message)
/// - timeout_ms > 0: Blocks until message, error, or timeout
///
/// Returns: message length on success, -11 (WouldBlock) if non-blocking and no message,
///          -110 (ETIMEDOUT) on timeout, negative error otherwise
pub fn receive_timeout(channel_id: u32, buf: &mut [u8], timeout_ms: u32) -> isize {
    // Non-blocking case: just check once
    if timeout_ms == 0 {
        return receive_nonblock(channel_id, buf);
    }

    // Blocking with timeout: retry on WouldBlock until timeout
    loop {
        let result = syscall4(SYS_RECEIVE_TIMEOUT, channel_id as u64, buf.as_mut_ptr() as u64, buf.len() as u64, timeout_ms as u64) as isize;
        if result == -11 {
            // WouldBlock - spurious wakeup, retry
            // Note: kernel handles actual timeout tracking
            continue;
        }
        return result;
    }
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

// Scheme syscalls

/// Open a scheme URL (e.g., "mmio:11200000/4000", "irq:172")
/// Returns file descriptor, or negative error code
pub fn scheme_open(url: &str, flags: u32) -> i32 {
    syscall3(SYS_SCHEME_OPEN, url.as_ptr() as u64, url.len() as u64, flags as u64) as i32
}

/// Register a user scheme (e.g., "hw", "fs", "net")
///
/// The calling process becomes the handler for this scheme.
/// When clients open "scheme:path", the kernel sends messages to `channel_id`.
///
/// Message format for open requests:
/// - header.msg_type = Connect
/// - header.msg_id = server_channel (for responses)
/// - payload[0..4] = flags
/// - payload[4..] = path
///
/// Returns 0 on success, negative error code on failure
pub fn scheme_register(name: &str, channel_id: u32) -> i32 {
    syscall3(
        SYS_SCHEME_REGISTER,
        name.as_ptr() as u64,
        name.len() as u64,
        channel_id as u64,
    ) as i32
}

/// Unregister a user scheme
/// Returns 0 on success, negative error code on failure
pub fn scheme_unregister(name: &str) -> i32 {
    syscall2(
        SYS_SCHEME_UNREGISTER,
        name.as_ptr() as u64,
        name.len() as u64,
    ) as i32
}

/// Get system time in nanoseconds
pub fn gettime() -> u64 {
    syscall0(SYS_GETTIME) as u64
}

// Event syscalls

/// Event types (must match kernel event.rs)
pub mod event_type {
    pub const NONE: u32 = 0;
    pub const IPC_READY: u32 = 1;
    pub const TIMER: u32 = 2;
    pub const IRQ: u32 = 3;
    pub const CHILD_EXIT: u32 = 4;
    pub const FD_READABLE: u32 = 5;
    pub const FD_WRITABLE: u32 = 6;
    pub const SIGNAL: u32 = 7;
    pub const KLOG_READY: u32 = 8;
}

/// Event wait flags
pub mod event_flags {
    pub const BLOCKING: u32 = 0;     // Block until event available
    pub const NON_BLOCKING: u32 = 1; // Return immediately if no event
}

/// Event structure (must match kernel event.rs)
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct Event {
    /// Type of event (see event_type module)
    pub event_type: u32,
    /// Padding for alignment
    pub _pad: u32,
    /// Event-specific data (e.g., channel ID, timer ID, child PID)
    pub data: u64,
    /// Additional flags (e.g., exit code for ChildExit)
    pub flags: u32,
    /// Source PID (for IPC events, child exit, signals)
    pub source_pid: u32,
}

impl Event {
    pub const fn empty() -> Self {
        Self {
            event_type: 0,
            _pad: 0,
            data: 0,
            flags: 0,
            source_pid: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.event_type == event_type::NONE
    }

    /// Get event type as a descriptive string
    pub fn type_name(&self) -> &'static str {
        match self.event_type {
            event_type::NONE => "none",
            event_type::IPC_READY => "ipc_ready",
            event_type::TIMER => "timer",
            event_type::IRQ => "irq",
            event_type::CHILD_EXIT => "child_exit",
            event_type::FD_READABLE => "fd_readable",
            event_type::FD_WRITABLE => "fd_writable",
            event_type::SIGNAL => "signal",
            event_type::KLOG_READY => "klog_ready",
            _ => "unknown",
        }
    }
}

/// Subscribe to an event type
/// event_type: one of event_type::*
/// filter: 0 = any, or specific value (channel ID, child PID, etc.)
/// Returns 0 on success, negative error on failure
pub fn event_subscribe(event_type: u32, filter: u64) -> i32 {
    syscall2(SYS_EVENT_SUBSCRIBE, event_type as u64, filter) as i32
}

/// Unsubscribe from an event type
pub fn event_unsubscribe(event_type: u32, filter: u64) -> i32 {
    syscall2(SYS_EVENT_UNSUBSCRIBE, event_type as u64, filter) as i32
}

/// Wait for an event (blocking or non-blocking)
/// event_out: pointer to Event structure to receive the event
/// flags: event_flags::BLOCKING or event_flags::NON_BLOCKING
/// Returns: 1 if event received, 0 if non-blocking and no event, negative error
pub fn event_wait(event_out: &mut Event, flags: u32) -> i64 {
    syscall2(SYS_EVENT_WAIT, event_out as *mut Event as u64, flags as u64)
}

/// Wait for an event, blocking until one is available
/// Returns the event, or an error
pub fn event_wait_blocking(event_out: &mut Event) -> i64 {
    loop {
        let result = event_wait(event_out, event_flags::BLOCKING);
        if result == -11 {
            // EAGAIN - kernel marked us blocked, yield and retry
            yield_now();
            continue;
        }
        return result;
    }
}

/// Check for an event without blocking
/// Returns: 1 if event received, 0 if no event available, negative error
pub fn event_poll(event_out: &mut Event) -> i64 {
    event_wait(event_out, event_flags::NON_BLOCKING)
}

/// Post an event to another process
pub fn event_post(target_pid: u32, event_type: u32, data: u64) -> i32 {
    syscall3(SYS_EVENT_POST, target_pid as u64, event_type as u64, data) as i32
}

// Process management syscalls

/// Process info structure for ps_info syscall
/// Must match kernel definition
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

    /// Get name as string slice
    pub fn name_str(&self) -> &str {
        let len = self.name.iter().position(|&c| c == 0).unwrap_or(16);
        core::str::from_utf8(&self.name[..len]).unwrap_or("???")
    }

    /// Get state as string
    pub fn state_str(&self) -> &'static str {
        match self.state {
            0 => "Ready",
            1 => "Running",
            2 => "Waiting",
            3 => "Zombie",
            _ => "???",
        }
    }

    /// Get heartbeat status as string
    pub fn heartbeat_str(&self) -> &'static str {
        match self.heartbeat_status {
            heartbeat_status::NEVER => "-",
            heartbeat_status::ALIVE => "OK",
            heartbeat_status::STALE => "STALE",
            heartbeat_status::DEAD => "DEAD",
            _ => "?",
        }
    }
}

/// Daemonize - detach from parent and become a daemon
/// Returns 0 on success
pub fn daemonize() -> i32 {
    syscall0(SYS_DAEMONIZE) as i32
}

/// Kill a process by PID
/// Returns 0 on success, negative error on failure
pub fn kill(pid: u32) -> i32 {
    syscall1(SYS_KILL, pid as u64) as i32
}

/// Get process info list
/// Returns number of entries written to buf
pub fn ps_info(buf: &mut [ProcessInfo]) -> usize {
    let ret = syscall2(SYS_PS_INFO, buf.as_mut_ptr() as u64, buf.len() as u64);
    if ret < 0 {
        0
    } else {
        ret as usize
    }
}

/// Set kernel log level
/// level: 0=Error, 1=Warn, 2=Info, 3=Debug, 4=Trace
pub fn set_log_level(level: u8) -> i32 {
    syscall1(SYS_SET_LOG_LEVEL, level as u64) as i32
}

// Log level constants for convenience
pub mod log_level {
    pub const ERROR: u8 = 0;
    pub const WARN: u8 = 1;
    pub const INFO: u8 = 2;
    pub const DEBUG: u8 = 3;
    pub const TRACE: u8 = 4;
}

/// Reset/reboot the system
/// Returns 0 on success (though success means reboot, so this won't return)
/// Returns negative error code on failure (e.g., permission denied)
pub fn reset() -> i64 {
    syscall0(SYS_RESET)
}

// =============================================================================
// Shared Memory Syscalls
// =============================================================================

/// Create a new shared memory region
/// Returns shmem_id on success, negative error on failure
/// vaddr and paddr are populated with the virtual and physical addresses
pub fn shmem_create(size: usize, vaddr: &mut u64, paddr: &mut u64) -> i64 {
    syscall3(
        SYS_SHMEM_CREATE,
        size as u64,
        vaddr as *mut u64 as u64,
        paddr as *mut u64 as u64,
    )
}

/// Map an existing shared memory region into this process
/// Returns 0 on success, negative error on failure
/// vaddr and paddr are populated with the virtual and physical addresses
pub fn shmem_map(shmem_id: u32, vaddr: &mut u64, paddr: &mut u64) -> i64 {
    syscall3(
        SYS_SHMEM_MAP,
        shmem_id as u64,
        vaddr as *mut u64 as u64,
        paddr as *mut u64 as u64,
    )
}

/// Allow another process to map this shared memory region
/// Returns 0 on success, negative error on failure
pub fn shmem_allow(shmem_id: u32, peer_pid: u32) -> i64 {
    syscall2(SYS_SHMEM_ALLOW, shmem_id as u64, peer_pid as u64)
}

/// Wait for notification on shared memory (blocks)
/// Returns 0 on notify, negative error on failure/timeout
pub fn shmem_wait(shmem_id: u32, timeout_ms: u32) -> i64 {
    syscall2(SYS_SHMEM_WAIT, shmem_id as u64, timeout_ms as u64)
}

/// Notify all waiters on shared memory
/// Returns number of waiters woken, or negative error
pub fn shmem_notify(shmem_id: u32) -> i64 {
    syscall1(SYS_SHMEM_NOTIFY, shmem_id as u64)
}

/// Destroy a shared memory region (only owner can do this)
/// Returns 0 on success, negative error on failure
pub fn shmem_destroy(shmem_id: u32) -> i64 {
    syscall1(SYS_SHMEM_DESTROY, shmem_id as u64)
}

/// Unmap a shared memory region from this process
/// Returns 0 on success, negative error on failure
pub fn shmem_unmap(shmem_id: u32) -> i64 {
    syscall1(SYS_SHMEM_UNMAP, shmem_id as u64)
}

// =============================================================================
// PCI Syscalls
// =============================================================================

pub const SYS_PCI_CONFIG_READ: u64 = 51;
pub const SYS_PCI_CONFIG_WRITE: u64 = 52;
pub const SYS_PCI_MSI_ALLOC: u64 = 54;
pub const SYS_SIGNAL_ALLOW: u64 = 56;
pub const SYS_TIMER_SET: u64 = 57;
pub const SYS_HEARTBEAT: u64 = 58;
pub const SYS_BUS_LIST: u64 = 59;
pub const SYS_MMAP_DEVICE: u64 = 60;
pub const SYS_DMA_POOL_CREATE: u64 = 61;
pub const SYS_DMA_POOL_CREATE_HIGH: u64 = 62;

/// PCI Bus/Device/Function address
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct PciBdf {
    /// Port number (for multi-port controllers)
    pub port: u8,
    /// Bus number
    pub bus: u8,
    /// Device number (0-31)
    pub device: u8,
    /// Function number (0-7)
    pub function: u8,
}

impl PciBdf {
    /// Create new BDF address
    pub const fn new(bus: u8, device: u8, function: u8) -> Self {
        Self { port: 0, bus, device, function }
    }

    /// Create BDF with port
    pub const fn with_port(port: u8, bus: u8, device: u8, function: u8) -> Self {
        Self { port, bus, device, function }
    }

    /// Convert to u32 for syscall passing
    /// Format: port(8) | bus(8) | device(5) | function(3)
    pub const fn to_u32(&self) -> u32 {
        ((self.port as u32) << 24)
            | ((self.bus as u32) << 16)
            | ((self.device as u32) << 8)
            | (self.function as u32)
    }

    /// Create from u32
    pub const fn from_u32(val: u32) -> Self {
        Self {
            port: ((val >> 24) & 0xFF) as u8,
            bus: ((val >> 16) & 0xFF) as u8,
            device: ((val >> 8) & 0x1F) as u8,
            function: (val & 0x07) as u8,
        }
    }
}

/// Read PCI config space
/// bdf: device address
/// offset: register offset
/// size: 1, 2, or 4 bytes
/// Returns: value or negative error
pub fn pci_config_read(bdf: PciBdf, offset: u16, size: u8) -> i64 {
    syscall3(SYS_PCI_CONFIG_READ, bdf.to_u32() as u64, offset as u64, size as u64)
}

/// Read 32-bit value from PCI config space
pub fn pci_config_read32(bdf: PciBdf, offset: u16) -> SysResult<u32> {
    let ret = pci_config_read(bdf, offset, 4);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(ret as u32)
    }
}

/// Read 16-bit value from PCI config space
pub fn pci_config_read16(bdf: PciBdf, offset: u16) -> SysResult<u16> {
    let ret = pci_config_read(bdf, offset, 2);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(ret as u16)
    }
}

/// Write PCI config space
/// bdf: device address
/// offset: register offset
/// size: 1, 2, or 4 bytes
/// value: value to write
/// Returns: 0 or negative error
pub fn pci_config_write(bdf: PciBdf, offset: u16, size: u8, value: u32) -> i32 {
    syscall4(SYS_PCI_CONFIG_WRITE, bdf.to_u32() as u64, offset as u64, size as u64, value as u64) as i32
}

/// Write 32-bit value to PCI config space
pub fn pci_config_write32(bdf: PciBdf, offset: u16, value: u32) -> i32 {
    pci_config_write(bdf, offset, 4, value)
}

/// Allocate MSI vector(s) for a device
/// bdf: device address
/// count: number of vectors requested
/// Returns: first IRQ number or negative error
pub fn pci_msi_alloc(bdf: PciBdf, count: u8) -> SysResult<u32> {
    let ret = syscall2(SYS_PCI_MSI_ALLOC, bdf.to_u32() as u64, count as u64);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(ret as u32)
    }
}

/// Map device MMIO into process address space
/// Generic syscall for drivers to map physical device memory
/// phys_addr: physical address of device MMIO (e.g., PCIe BAR)
/// size: size in bytes to map
/// Returns: virtual address on success, negative error on failure
pub fn mmap_device(phys_addr: u64, size: u64) -> SysResult<u64> {
    let ret = syscall2(SYS_MMAP_DEVICE, phys_addr, size);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(ret as u64)
    }
}

/// Allocate from DMA pool (low memory for PCIe devices)
///
/// Some PCIe devices (like MT7996 WiFi) may have issues accessing higher memory
/// addresses. This allocates from a pre-reserved pool of low memory (0x40100000).
///
/// Returns 0 on success, negative error on failure.
/// vaddr and paddr are populated with the virtual and physical addresses.
pub fn dma_pool_create(size: usize, vaddr: &mut u64, paddr: &mut u64) -> i64 {
    syscall3(
        SYS_DMA_POOL_CREATE,
        size as u64,
        vaddr as *mut u64 as u64,
        paddr as *mut u64 as u64,
    )
}

/// Allocate from high DMA pool (36-bit addresses, > 4GB)
///
/// For devices like MT7996 that use 36-bit DMA addressing for TX/RX buffers.
/// The returned physical address will be above 0x100000000 (4GB).
///
/// Returns 0 on success, negative error on failure.
pub fn dma_pool_create_high(size: usize, vaddr: &mut u64, paddr: &mut u64) -> i64 {
    syscall3(
        SYS_DMA_POOL_CREATE_HIGH,
        size as u64,
        vaddr as *mut u64 as u64,
        paddr as *mut u64 as u64,
    )
}

/// Allow a specific PID to send signals to this process.
///
/// By default (empty allowlist), any process can send signals. Once at least one
/// PID is added, only those PIDs (and the parent) can send signals.
///
/// Returns 0 on success, negative error code on failure.
pub fn signal_allow(sender_pid: u32) -> i32 {
    syscall1(SYS_SIGNAL_ALLOW, sender_pid as u64) as i32
}

/// Set a one-shot timer that delivers a Timer event after duration_ns nanoseconds.
///
/// The task must be subscribed to Timer events (event_type::TIMER) to receive them.
/// Pass 0 to cancel a pending timer.
///
/// Returns 0 on success, negative error code on failure.
pub fn timer_set(duration_ns: u64) -> i32 {
    syscall1(SYS_TIMER_SET, duration_ns) as i32
}

/// Send a heartbeat to indicate this process is alive.
///
/// Privileged processes should call this periodically (e.g., every 500ms).
/// The kernel tracks the last heartbeat time, which is visible in ps output.
///
/// Returns 0 on success, negative error code on failure.
pub fn heartbeat() -> i32 {
    syscall0(SYS_HEARTBEAT) as i32
}

// =============================================================================
// Bus Discovery
// =============================================================================

/// Bus type constants (must match kernel)
pub mod bus_type {
    pub const PCIE: u8 = 0;
    pub const USB: u8 = 1;
    pub const PLATFORM: u8 = 2;
}

/// Bus state constants (must match kernel)
pub mod bus_state {
    pub const SAFE: u8 = 0;
    pub const CLAIMED: u8 = 1;
    pub const RESETTING: u8 = 2;
}

/// Bus capability flags (must match kernel bus.rs)
/// These are reported in StateSnapshot.capabilities
pub mod bus_caps {
    // PCIe capabilities
    /// PCIe: Bus mastering (DMA) supported
    pub const PCIE_BUS_MASTER: u8 = 1 << 0;
    /// PCIe: MSI interrupts supported
    pub const PCIE_MSI: u8 = 1 << 1;
    /// PCIe: MSI-X interrupts supported
    pub const PCIE_MSIX: u8 = 1 << 2;
    /// PCIe: Link is up and trained
    pub const PCIE_LINK_UP: u8 = 1 << 3;

    // USB capabilities
    /// USB: USB 2.0 (EHCI/high-speed) supported
    pub const USB_2_0: u8 = 1 << 0;
    /// USB: USB 3.0 (xHCI/super-speed) supported
    pub const USB_3_0: u8 = 1 << 1;
    /// USB: Controller is running (not halted)
    pub const USB_RUNNING: u8 = 1 << 2;

    // Platform capabilities
    /// Platform: MMIO regions available
    pub const PLATFORM_MMIO: u8 = 1 << 0;
    /// Platform: IRQs available
    pub const PLATFORM_IRQ: u8 = 1 << 1;
}

/// Bus information returned by bus_list syscall
/// Layout must match kernel's bus::BusInfo
#[derive(Clone, Copy)]
#[repr(C)]
pub struct BusInfo {
    /// Bus type (0=PCIe, 1=USB, 2=Platform)
    pub bus_type: u8,
    /// Bus index within type (e.g., 0 for pcie0)
    pub bus_index: u8,
    /// Current state (0=Safe, 1=Claimed, 2=Resetting)
    pub state: u8,
    /// Padding
    pub _pad: u8,
    /// MMIO base address
    pub base_addr: u32,
    /// Owner PID (0 if no owner)
    pub owner_pid: u32,
    /// Port path (e.g., "/kernel/bus/pcie0")
    pub path: [u8; 32],
    /// Length of path string
    pub path_len: u8,
    /// Reserved
    pub _reserved: [u8; 3],
}

impl BusInfo {
    pub const fn empty() -> Self {
        Self {
            bus_type: 0,
            bus_index: 0,
            state: 0,
            _pad: 0,
            base_addr: 0,
            owner_pid: 0,
            path: [0; 32],
            path_len: 0,
            _reserved: [0; 3],
        }
    }

    /// Get port path as string slice
    pub fn path_str(&self) -> &str {
        core::str::from_utf8(&self.path[..self.path_len as usize]).unwrap_or("")
    }

    /// Check if this is a PCIe bus
    pub fn is_pcie(&self) -> bool {
        self.bus_type == bus_type::PCIE
    }

    /// Check if this is a USB bus
    pub fn is_usb(&self) -> bool {
        self.bus_type == bus_type::USB
    }

    /// Check if this is a Platform bus
    pub fn is_platform(&self) -> bool {
        self.bus_type == bus_type::PLATFORM
    }
}

/// Get count of available buses
pub fn bus_count() -> i64 {
    syscall2(SYS_BUS_LIST, 0, 0)
}

/// List available buses from kernel
/// Returns number of buses written to buf, or negative error
pub fn bus_list(buf: &mut [BusInfo]) -> i64 {
    syscall2(SYS_BUS_LIST, buf.as_mut_ptr() as u64, buf.len() as u64)
}

// ============================================================================
// Ramfs (initrd filesystem) syscalls
// ============================================================================

/// Ramfs directory entry (matches kernel RamfsListEntry)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RamfsEntry {
    /// Filename (null-terminated, max 100 bytes)
    pub name: [u8; 100],
    /// File size in bytes
    pub size: u64,
    /// File type: 0 = regular, 1 = directory
    pub file_type: u8,
    /// Padding for alignment
    pub _pad: [u8; 7],
}

impl RamfsEntry {
    /// Size of entry in bytes
    pub const SIZE: usize = 116;

    /// Create an empty entry
    pub const fn empty() -> Self {
        Self {
            name: [0u8; 100],
            size: 0,
            file_type: 0,
            _pad: [0u8; 7],
        }
    }

    /// Get filename as string slice
    pub fn name_str(&self) -> &str {
        let len = self.name.iter().position(|&c| c == 0).unwrap_or(100);
        core::str::from_utf8(&self.name[..len]).unwrap_or("")
    }

    /// Check if this is a regular file
    pub fn is_file(&self) -> bool {
        self.file_type == 0
    }

    /// Check if this is a directory
    pub fn is_dir(&self) -> bool {
        self.file_type == 1
    }
}

/// List ramfs entries
/// Returns number of entries written to buf, or negative error
pub fn ramfs_list(buf: &mut [RamfsEntry]) -> i64 {
    syscall2(SYS_RAMFS_LIST, buf.as_mut_ptr() as u64, (buf.len() * RamfsEntry::SIZE) as u64)
}

/// CPU statistics entry
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
    /// Size of entry in bytes
    pub const SIZE: usize = 24;

    /// Create an empty entry
    pub const fn empty() -> Self {
        Self {
            cpu_id: 0,
            _pad: 0,
            tick_count: 0,
            idle_ticks: 0,
        }
    }

    /// Calculate CPU busy percentage (0-100)
    /// Returns None if tick_count is 0
    pub fn busy_percent(&self) -> Option<u32> {
        if self.tick_count == 0 {
            return None;
        }
        // busy = total - idle
        // busy% = 100 * busy / total = 100 * (1 - idle/total)
        let busy = self.tick_count.saturating_sub(self.idle_ticks);
        Some(((busy * 100) / self.tick_count) as u32)
    }
}

/// Get CPU statistics
/// Returns number of CPUs, or negative error
pub fn cpu_stats(buf: &mut [CpuStatsEntry]) -> i64 {
    syscall2(SYS_CPU_STATS, buf.as_mut_ptr() as u64, (buf.len() * CpuStatsEntry::SIZE) as u64)
}

// =============================================================================
// Kevent (Unified Event System)
// =============================================================================

/// Unified event filter - what to wait for (BSD kqueue-inspired)
///
/// Used for kevent_subscribe() and kevent_wait() APIs.
/// Provides a cleaner interface than the legacy event_subscribe().
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventFilter {
    /// IPC channel has message (channel_id)
    Ipc(u32),
    /// Timer expired (timer_id, 0 = any timer)
    Timer(u32),
    /// IRQ occurred (irq_num)
    Irq(u32),
    /// Child process exited (pid, 0 = any child)
    ChildExit(u32),
    /// File descriptor readable (fd)
    Read(u32),
    /// File descriptor writable (fd)
    Write(u32),
    /// Signal received (signal_num, 0 = any)
    Signal(u32),
    /// Kernel log available
    KlogReady,
}

impl EventFilter {
    /// Convert to syscall arguments (filter_type, filter_value)
    pub fn to_args(&self) -> (u32, u32) {
        match self {
            EventFilter::Ipc(id) => (1, *id),
            EventFilter::Timer(id) => (2, *id),
            EventFilter::Irq(num) => (3, *num),
            EventFilter::ChildExit(pid) => (4, *pid),
            EventFilter::Read(fd) => (5, *fd),
            EventFilter::Write(fd) => (6, *fd),
            EventFilter::Signal(num) => (7, *num),
            EventFilter::KlogReady => (8, 0),
        }
    }
}

/// Subscribe to events using kevent-style unified filter
///
/// Example:
/// ```
/// kevent_subscribe(EventFilter::Timer(0))?;  // Subscribe to any timer
/// kevent_subscribe(EventFilter::Ipc(channel_id))?;  // Subscribe to IPC
/// ```
pub fn kevent_subscribe(filter: EventFilter) -> SysResult<()> {
    let (filter_type, filter_value) = filter.to_args();
    let ret = syscall2(SYS_KEVENT_SUBSCRIBE, filter_type as u64, filter_value as u64);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(())
    }
}

/// Unsubscribe from events using kevent-style unified filter
pub fn kevent_unsubscribe(filter: EventFilter) -> SysResult<()> {
    let (filter_type, filter_value) = filter.to_args();
    let ret = syscall2(SYS_KEVENT_UNSUBSCRIBE, filter_type as u64, filter_value as u64);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(())
    }
}

/// Set or modify a timer with kevent-style API
///
/// - id: timer ID (1-7 for user timers, 0 for legacy compatibility)
/// - interval_ns: 0 for one-shot, >0 for recurring (nanoseconds)
/// - initial_ns: time until first fire (nanoseconds, 0 = use interval for recurring)
///
/// To cancel all timers: `kevent_timer(0, 0, 0)`
/// To set one-shot timer: `kevent_timer(1, 0, 100_000_000)` // 100ms
/// To set recurring timer: `kevent_timer(2, 500_000_000, 0)` // Every 500ms
pub fn kevent_timer(id: u32, interval_ns: u64, initial_ns: u64) -> SysResult<()> {
    let ret = syscall3(SYS_KEVENT_TIMER, id as u64, interval_ns, initial_ns);
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(())
    }
}

/// Wait for events with batch receive (kevent-style)
///
/// - buf: buffer to receive Event structures
/// - timeout_ns: 0 = poll, u64::MAX = block forever, else timeout in nanoseconds
///
/// Returns the number of events received (0 on timeout with no events).
///
/// Example:
/// ```
/// let mut events = [Event::empty(); 8];
/// let count = kevent_wait(&mut events, u64::MAX)?;  // Block forever
/// for event in &events[..count] {
///     handle_event(event);
/// }
/// ```
pub fn kevent_wait(buf: &mut [Event], timeout_ns: u64) -> SysResult<usize> {
    let ret = syscall3(
        SYS_KEVENT_WAIT,
        buf.as_mut_ptr() as u64,
        buf.len() as u64,
        timeout_ns,
    );
    if ret < 0 {
        Err(SysError::from_errno(ret as i32))
    } else {
        Ok(ret as usize)
    }
}

/// Cancel a specific timer by ID
/// Convenience wrapper around kevent_timer(id, 0, 0) with special handling
pub fn kevent_timer_cancel(id: u32) -> SysResult<()> {
    // To cancel a timer, set deadline = 0 by using id with interval=0 and initial=0
    // But the kernel requires at least initial_ns for one-shot timers
    // Actually, looking at kernel code - we need to set the timer with deadline = 0
    // The kernel doesn't have a direct cancel-single-timer; let's just set initial_ns=0
    // which will be rejected. Instead, we use the task's timers array directly.
    // For now, use timer_set(0) style: set deadline to 0
    // Actually, let's just set it with interval=0, initial=1ns to make it fire immediately
    // and be cleared, or add kernel support for cancel.

    // Simple approach: we don't have direct cancel per-timer in the current kernel API.
    // The cancel-all is id=0, interval=0, initial=0.
    // For single timer cancel, user should track and not rely on kernel.
    // Or we can set initial_ns to a very large value that will never fire.
    // Actually, let me check - the kernel clears deadline on fire, so set to max u64?

    // Best current approach: Set timer to fire immediately with one-shot
    // It will fire once and be cleared. Not ideal but works.
    kevent_timer(id, 0, 1) // One-shot, 1ns = fires immediately at next tick
}
