//! File Descriptor Management
//!
//! Provides file descriptor abstraction for user processes.
//! In Redox style, everything is a file - console, devices, IPC, etc.

use crate::ipc::{self, MessageType};
use crate::process::Pid;
use crate::uart;

/// Maximum file descriptors per process
pub const MAX_FDS: usize = 32;

/// File descriptor number type
pub type Fd = u32;

/// Standard file descriptors
pub const STDIN: Fd = 0;
pub const STDOUT: Fd = 1;
pub const STDERR: Fd = 2;

/// File descriptor flags
#[derive(Clone, Copy, Debug)]
pub struct FdFlags {
    pub readable: bool,
    pub writable: bool,
    pub nonblocking: bool,
}

impl FdFlags {
    pub const fn read_only() -> Self {
        Self { readable: true, writable: false, nonblocking: false }
    }

    pub const fn write_only() -> Self {
        Self { readable: false, writable: true, nonblocking: false }
    }

    pub const fn read_write() -> Self {
        Self { readable: true, writable: true, nonblocking: false }
    }
}

/// Types of file descriptors
#[derive(Clone, Copy, Debug)]
pub enum FdType {
    /// Not in use
    None,
    /// Console input (keyboard)
    ConsoleIn,
    /// Console output (UART)
    ConsoleOut,
    /// Null device (/dev/null equivalent)
    Null,
    /// IPC channel
    Channel(u32),  // Channel ID
    /// Scheme-based file (kernel or user scheme)
    Scheme {
        scheme_id: u16,
        handle: u64,
        scheme_flags: u32,  // Extra data for scheme (e.g., owner pid for IRQ scheme)
    },
    /// MMIO mapping (device memory mapped to userspace)
    Mmio {
        virt_addr: u64,    // Virtual address in user space
        phys_addr: u64,    // Physical address of device
        num_pages: usize,  // Number of pages mapped
    },
}

/// A file descriptor entry
#[derive(Clone, Copy, Debug)]
pub struct FdEntry {
    pub fd_type: FdType,
    pub flags: FdFlags,
    pub offset: u64,  // Current file offset (for seekable files)
}

impl FdEntry {
    pub const fn empty() -> Self {
        Self {
            fd_type: FdType::None,
            flags: FdFlags { readable: false, writable: false, nonblocking: false },
            offset: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        matches!(self.fd_type, FdType::None)
    }
}

/// File descriptor table for a process
#[derive(Clone)]
pub struct FdTable {
    entries: [FdEntry; MAX_FDS],
}

impl FdTable {
    /// Create a new FD table with standard streams
    pub fn new() -> Self {
        let mut table = Self {
            entries: [FdEntry::empty(); MAX_FDS],
        };

        // Set up standard file descriptors
        table.entries[STDIN as usize] = FdEntry {
            fd_type: FdType::ConsoleIn,
            flags: FdFlags::read_only(),
            offset: 0,
        };
        table.entries[STDOUT as usize] = FdEntry {
            fd_type: FdType::ConsoleOut,
            flags: FdFlags::write_only(),
            offset: 0,
        };
        table.entries[STDERR as usize] = FdEntry {
            fd_type: FdType::ConsoleOut,
            flags: FdFlags::write_only(),
            offset: 0,
        };

        table
    }

    /// Allocate a new file descriptor
    pub fn alloc(&mut self) -> Option<Fd> {
        for (i, entry) in self.entries.iter().enumerate() {
            if entry.is_empty() {
                return Some(i as Fd);
            }
        }
        None
    }

    /// Get an FD entry
    pub fn get(&self, fd: Fd) -> Option<&FdEntry> {
        let idx = fd as usize;
        if idx < MAX_FDS && !self.entries[idx].is_empty() {
            Some(&self.entries[idx])
        } else {
            None
        }
    }

    /// Get an FD entry mutably
    pub fn get_mut(&mut self, fd: Fd) -> Option<&mut FdEntry> {
        let idx = fd as usize;
        if idx < MAX_FDS && !self.entries[idx].is_empty() {
            Some(&mut self.entries[idx])
        } else {
            None
        }
    }

    /// Set an FD entry
    pub fn set(&mut self, fd: Fd, entry: FdEntry) -> bool {
        let idx = fd as usize;
        if idx < MAX_FDS {
            self.entries[idx] = entry;
            true
        } else {
            false
        }
    }

    /// Close a file descriptor
    pub fn close(&mut self, fd: Fd) -> bool {
        let idx = fd as usize;
        if idx < MAX_FDS && !self.entries[idx].is_empty() {
            self.entries[idx] = FdEntry::empty();
            true
        } else {
            false
        }
    }

    /// Duplicate a file descriptor
    pub fn dup(&mut self, old_fd: Fd) -> Option<Fd> {
        let old_entry = *self.get(old_fd)?;
        let new_fd = self.alloc()?;
        self.entries[new_fd as usize] = old_entry;
        Some(new_fd)
    }

    /// Duplicate a file descriptor to a specific number
    pub fn dup2(&mut self, old_fd: Fd, new_fd: Fd) -> bool {
        if old_fd == new_fd {
            return self.get(old_fd).is_some();
        }

        let old_entry = match self.get(old_fd) {
            Some(e) => *e,
            None => return false,
        };

        let new_idx = new_fd as usize;
        if new_idx >= MAX_FDS {
            return false;
        }

        // Close new_fd if open
        self.entries[new_idx] = old_entry;
        true
    }
}

/// Read from a file descriptor
/// Returns number of bytes read, or negative error
/// caller_pid: Required for channel operations (permission check)
pub fn fd_read(entry: &FdEntry, buf: &mut [u8], caller_pid: Pid) -> isize {
    if !entry.flags.readable {
        return -9; // EBADF
    }

    match entry.fd_type {
        FdType::None => -9, // EBADF
        FdType::ConsoleIn => {
            // Read from UART
            if buf.is_empty() {
                return 0;
            }

            if entry.flags.nonblocking {
                // Non-blocking: return immediately
                match uart::try_getc() {
                    Some(c) => {
                        buf[0] = c as u8;
                        1
                    }
                    None => -11, // EAGAIN
                }
            } else {
                // "Blocking" read: if no data, switch tasks and return EAGAIN
                // Caller should retry. This allows other tasks to run.
                match uart::try_getc() {
                    Some(c) => {
                        buf[0] = c as u8;
                        let mut count = 1;
                        // Read more if available (non-blocking for subsequent chars)
                        while count < buf.len() {
                            match uart::try_getc() {
                                Some(c) => {
                                    buf[count] = c as u8;
                                    count += 1;
                                }
                                None => break,
                            }
                        }
                        count as isize
                    }
                    None => {
                        // No data available - switch to another task if possible
                        unsafe {
                            let sched = crate::task::scheduler();
                            if let Some(next_slot) = sched.schedule() {
                                if next_slot != sched.current {
                                    // Mark current as ready and switch
                                    if let Some(ref mut current) = sched.tasks[sched.current] {
                                        current.state = crate::task::TaskState::Ready;
                                    }
                                    sched.current = next_slot;
                                    if let Some(ref mut next) = sched.tasks[next_slot] {
                                        next.state = crate::task::TaskState::Running;
                                    }
                                    crate::task::update_current_task_globals();
                                    crate::task::SYSCALL_SWITCHED_TASK = 1;
                                }
                            }
                        }
                        -11 // EAGAIN - caller should retry
                    }
                }
            }
        }
        FdType::ConsoleOut => -9, // EBADF - Can't read from output
        FdType::Null => 0,        // EOF
        FdType::Channel(channel_id) => {
            // Read from IPC channel
            match ipc::sys_receive(channel_id, caller_pid) {
                Ok(msg) => {
                    // Only process data-like messages for read()
                    match msg.header.msg_type {
                        MessageType::Data | MessageType::Reply => {
                            let payload = msg.payload_slice();
                            let copy_len = core::cmp::min(payload.len(), buf.len());
                            buf[..copy_len].copy_from_slice(&payload[..copy_len]);
                            copy_len as isize
                        }
                        MessageType::Close => {
                            // Peer closed - return EOF
                            0
                        }
                        MessageType::Error => {
                            // Extract error code from payload
                            if msg.header.payload_len >= 4 {
                                let err_bytes: [u8; 4] = msg.payload[0..4].try_into().unwrap();
                                i32::from_le_bytes(err_bytes) as isize
                            } else {
                                -5 // EIO
                            }
                        }
                        _ => {
                            // Other message types (Request, Connect, Accept)
                            // For simple read(), treat as data
                            let payload = msg.payload_slice();
                            let copy_len = core::cmp::min(payload.len(), buf.len());
                            buf[..copy_len].copy_from_slice(&payload[..copy_len]);
                            copy_len as isize
                        }
                    }
                }
                Err(ipc::IpcError::WouldBlock) => {
                    if entry.flags.nonblocking {
                        -11 // EAGAIN
                    } else {
                        // Register for blocking - syscall layer handles actual blocking
                        unsafe {
                            let _ = ipc::channel_table().block_receiver(channel_id, caller_pid);
                        }
                        -11 // EAGAIN - caller should block and retry
                    }
                }
                Err(ipc::IpcError::PeerClosed) => 0, // EOF
                Err(ipc::IpcError::InvalidChannel) => -9, // EBADF
                Err(_) => -5, // EIO
            }
        }
        FdType::Scheme { scheme_id, handle, scheme_flags } => {
            // Read from kernel scheme
            if let Some(scheme) = crate::scheme::get_kernel_scheme_by_id(scheme_id) {
                let scheme_handle = crate::scheme::SchemeHandle {
                    scheme_id,
                    handle,
                    flags: scheme_flags,
                };
                match scheme.read(&scheme_handle, buf) {
                    Ok(n) => n as isize,
                    Err(e) => e as isize,
                }
            } else {
                -9 // EBADF
            }
        }
        FdType::Mmio { virt_addr, .. } => {
            // For MMIO FDs, read returns the mapped virtual address
            // (User can then access memory directly via mmap-style access)
            if buf.len() >= 8 {
                buf[..8].copy_from_slice(&virt_addr.to_le_bytes());
                8
            } else {
                -22 // EINVAL
            }
        }
    }
}

/// Write to a file descriptor
/// Returns number of bytes written, or negative error
/// caller_pid: Required for channel operations (permission check)
pub fn fd_write(entry: &FdEntry, buf: &[u8], caller_pid: Pid) -> isize {
    if !entry.flags.writable {
        return -9; // EBADF
    }

    match entry.fd_type {
        FdType::None => -9, // EBADF
        FdType::ConsoleIn => -9, // EBADF - Can't write to input
        FdType::ConsoleOut => {
            // Write to UART
            for &byte in buf {
                uart::putc(byte as char);
            }
            buf.len() as isize
        }
        FdType::Null => buf.len() as isize, // Discard
        FdType::Channel(channel_id) => {
            // Write to IPC channel
            // Split data into chunks if needed (max payload is MAX_INLINE_PAYLOAD)
            let max_payload = ipc::MAX_INLINE_PAYLOAD;
            let chunk = if buf.len() > max_payload {
                &buf[..max_payload]
            } else {
                buf
            };

            match ipc::sys_send(channel_id, caller_pid, chunk) {
                Ok(()) => chunk.len() as isize,
                Err(ipc::IpcError::QueueFull) => {
                    if entry.flags.nonblocking {
                        -11 // EAGAIN
                    } else {
                        // Would block - for now return EAGAIN
                        // Full blocking would require blocking the sender
                        -11
                    }
                }
                Err(ipc::IpcError::PeerClosed) => -32, // EPIPE
                Err(ipc::IpcError::InvalidChannel) => -9, // EBADF
                Err(ipc::IpcError::PermissionDenied) => -13, // EACCES
                Err(_) => -5, // EIO
            }
        }
        FdType::Scheme { scheme_id, handle, scheme_flags } => {
            // Write to kernel scheme
            if let Some(scheme) = crate::scheme::get_kernel_scheme_by_id(scheme_id) {
                let scheme_handle = crate::scheme::SchemeHandle {
                    scheme_id,
                    handle,
                    flags: scheme_flags,
                };
                match scheme.write(&scheme_handle, buf) {
                    Ok(n) => n as isize,
                    Err(e) => e as isize,
                }
            } else {
                -9 // EBADF
            }
        }
        FdType::Mmio { .. } => {
            // MMIO FDs don't support write() - access memory directly
            -22 // EINVAL
        }
    }
}

/// Open flags (simplified)
pub mod open_flags {
    pub const O_RDONLY: u32 = 0;
    pub const O_WRONLY: u32 = 1;
    pub const O_RDWR: u32 = 2;
    pub const O_CREAT: u32 = 0x40;
    pub const O_TRUNC: u32 = 0x200;
    pub const O_APPEND: u32 = 0x400;
    pub const O_NONBLOCK: u32 = 0x800;
}

/// Open a file by path
/// For now, only supports special paths: "null:", "console:"
pub fn open_path(path: &[u8], flags: u32) -> Option<FdEntry> {
    // Parse access mode
    let access = flags & 0x3;
    let readable = access == open_flags::O_RDONLY || access == open_flags::O_RDWR;
    let writable = access == open_flags::O_WRONLY || access == open_flags::O_RDWR;
    let nonblocking = (flags & open_flags::O_NONBLOCK) != 0;

    let fd_flags = FdFlags { readable, writable, nonblocking };

    // Match known paths
    if path == b"null:" || path == b"/dev/null" {
        Some(FdEntry {
            fd_type: FdType::Null,
            flags: fd_flags,
            offset: 0,
        })
    } else if path == b"console:" || path == b"/dev/console" {
        // Console is both input and output based on flags
        let fd_type = if readable && !writable {
            FdType::ConsoleIn
        } else {
            FdType::ConsoleOut
        };
        Some(FdEntry {
            fd_type,
            flags: fd_flags,
            offset: 0,
        })
    } else {
        // Unknown path - will be handled by scheme system later
        None
    }
}
