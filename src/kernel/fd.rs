//! File Descriptor Management

#![allow(dead_code)]  // Infrastructure for future use
//!
//! Provides file descriptor abstraction for user processes.
//! In Redox style, everything is a file - console, devices, IPC, etc.

use super::ipc::{self, MessageType};
use super::process::Pid;
use crate::platform::mt7988::uart;

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
    /// Ramfs file (from initrd)
    Ramfs {
        entry_idx: usize,  // Index into ramfs entries
        size: usize,       // File size (cached for convenience)
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

    /// Close a file descriptor with proper resource cleanup
    /// caller_pid: Required for channel close permission check
    pub fn close_with_pid(&mut self, fd: Fd, caller_pid: Pid) -> bool {
        let idx = fd as usize;
        if idx < MAX_FDS && !self.entries[idx].is_empty() {
            // Clean up scheme resources before clearing the entry
            self.cleanup_fd_resources(&self.entries[idx], caller_pid);
            self.entries[idx] = FdEntry::empty();
            true
        } else {
            false
        }
    }

    /// Close a file descriptor (simple version for backward compatibility)
    pub fn close(&mut self, fd: Fd) -> bool {
        let idx = fd as usize;
        if idx < MAX_FDS && !self.entries[idx].is_empty() {
            // For simple close without PID, just close schemes (channels need PID)
            if let FdType::Scheme { scheme_id, handle, scheme_flags } = self.entries[idx].fd_type {
                super::scheme::close_kernel_scheme(scheme_id, handle, scheme_flags);
            }
            self.entries[idx] = FdEntry::empty();
            true
        } else {
            false
        }
    }

    /// Close all file descriptors (called on process exit)
    /// caller_pid: The PID of the exiting process
    pub fn close_all(&mut self, caller_pid: Pid) {
        for idx in 0..MAX_FDS {
            if !self.entries[idx].is_empty() {
                self.cleanup_fd_resources(&self.entries[idx], caller_pid);
                self.entries[idx] = FdEntry::empty();
            }
        }
    }

    /// Clean up resources associated with an FD entry
    fn cleanup_fd_resources(&self, entry: &FdEntry, caller_pid: Pid) {
        match entry.fd_type {
            FdType::Scheme { scheme_id, handle, scheme_flags } => {
                // Call scheme's close handler
                super::scheme::close_kernel_scheme(scheme_id, handle, scheme_flags);
            }
            FdType::Channel(channel_id) => {
                // Close IPC channel
                let _ = super::ipc::sys_channel_close(channel_id, caller_pid);
            }
            // Other FD types don't need special cleanup
            FdType::None | FdType::ConsoleIn | FdType::ConsoleOut |
            FdType::Null | FdType::Ramfs { .. } | FdType::Mmio { .. } => {}
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
    /// caller_pid: Required for channel close permission check if new_fd was open
    pub fn dup2(&mut self, old_fd: Fd, new_fd: Fd, caller_pid: Pid) -> bool {
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

        // Close new_fd if open (with proper cleanup)
        if !self.entries[new_idx].is_empty() {
            self.cleanup_fd_resources(&self.entries[new_idx], caller_pid);
        }
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
            // Read from UART RX ring buffer
            // Data is collected by IRQ handler into the buffer
            if buf.is_empty() {
                return 0;
            }

            if entry.flags.nonblocking {
                // Non-blocking: return immediately from buffer
                match uart::rx_buffer_read() {
                    Some(c) => {
                        buf[0] = c;
                        1
                    }
                    None => -11, // EAGAIN
                }
            } else {
                // Blocking read: read from buffer, block if empty
                // Read as many bytes as available in buffer (up to buf size)
                let count = uart::rx_buffer_read_bytes(buf);
                if count > 0 {
                    count as isize
                } else {
                    // No data in buffer - block waiting for UART RX interrupt
                    unsafe {
                        let sched = super::task::scheduler();
                        let my_slot = super::task::current_slot();
                        let current_pid = sched.tasks[my_slot]
                            .as_ref().map(|t| t.id).unwrap_or(0);

                        // Check if there's another task to switch to FIRST
                        let has_other_task = sched.tasks.iter().enumerate().any(|(slot, t)| {
                            slot != my_slot &&
                            t.as_ref().map(|task| task.state == super::task::TaskState::Ready).unwrap_or(false)
                        });

                        if has_other_task {
                            // There's another task - we can safely block
                            if uart::get_blocked_pid() == current_pid {
                                uart::clear_blocked();
                            }

                            if uart::block_for_input(current_pid) {
                                // Block the task (sleeping, waiting for UART input event)
                                if let Some(ref mut current) = sched.tasks[my_slot] {
                                    current.state = super::task::TaskState::Sleeping {
                                        reason: super::task::SleepReason::EventLoop,
                                    };
                                    current.trap_frame.x0 = (-11i64) as u64;
                                }

                                // Switch to another task
                                if let Some(next_slot) = sched.schedule() {
                                    super::task::set_current_slot(next_slot);
                                    sched.current = next_slot;
                                    if let Some(ref mut next) = sched.tasks[next_slot] {
                                        next.state = super::task::TaskState::Running;
                                    }
                                    super::task::update_current_task_globals();
                                    super::task::SYSCALL_SWITCHED_TASK.store(1, core::sync::atomic::Ordering::Release);
                                }
                            }
                        } else {
                            // No other task - use WFI to wait for IRQ
                            while uart::has_buffered_output() {
                                uart::flush_buffer();
                            }
                            uart::block_for_input(current_pid);
                            core::arch::asm!("msr daifclr, #2");  // Enable IRQs
                            core::arch::asm!("wfi");
                            core::arch::asm!("msr daifset, #2");  // Disable IRQs
                            uart::clear_blocked();
                        }
                    }
                    -11 // EAGAIN - caller retries after being woken
                }
            }
        }
        FdType::ConsoleOut => -9, // EBADF - Can't read from output
        FdType::Null => 0,        // EOF
        FdType::Channel(channel_id) => {
            // Read from IPC channel using ipc
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
                            if let Some(err_bytes) = msg.payload.get(0..4)
                                .and_then(|s| <[u8; 4]>::try_from(s).ok()) {
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
                        // Subscribe for wake notification via ipc subscriber system
                        ipc::channel_register_waker(channel_id, caller_pid);
                        -11 // EAGAIN - caller should block and retry
                    }
                }
                Err(ipc::IpcError::PeerClosed) => 0, // EOF
                Err(ipc::IpcError::Closed) => 0, // EOF
                Err(ipc::IpcError::InvalidChannel { .. }) => -9, // EBADF
                Err(_) => -5, // EIO
            }
        }
        FdType::Scheme { scheme_id, handle, scheme_flags } => {
            // Read from kernel scheme
            if let Some(scheme) = super::scheme::get_kernel_scheme_by_id(scheme_id) {
                let scheme_handle = super::scheme::SchemeHandle {
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
        FdType::Ramfs { entry_idx, size } => {
            // Read from ramfs file
            let offset = entry.offset as usize;
            if offset >= size {
                return 0; // EOF
            }

            // Get the ramfs entry
            if let Some(ramfs_entry) = crate::ramfs::ramfs().get(entry_idx) {
                let data = ramfs_entry.data_slice();
                let remaining = size - offset;
                let to_read = core::cmp::min(buf.len(), remaining);

                if to_read > 0 && offset < data.len() {
                    let actual_read = core::cmp::min(to_read, data.len() - offset);
                    buf[..actual_read].copy_from_slice(&data[offset..offset + actual_read]);
                    actual_read as isize
                } else {
                    0 // EOF
                }
            } else {
                -5 // EIO - entry not found
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
            // Write to UART and flush immediately for debugging
            uart::write_buffered(buf);
            uart::flush_buffer();
            buf.len() as isize
        }
        FdType::Null => buf.len() as isize, // Discard
        FdType::Channel(channel_id) => {
            // Write to IPC channel using ipc
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
                Err(ipc::IpcError::Closed) => -32, // EPIPE
                Err(ipc::IpcError::InvalidChannel { .. }) => -9, // EBADF
                Err(ipc::IpcError::NotOwner { .. }) => -13, // EACCES
                Err(_) => -5, // EIO
            }
        }
        FdType::Scheme { scheme_id, handle, scheme_flags } => {
            // Write to kernel scheme
            if let Some(scheme) = super::scheme::get_kernel_scheme_by_id(scheme_id) {
                let scheme_handle = super::scheme::SchemeHandle {
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
        FdType::Ramfs { .. } => {
            // Ramfs is read-only
            -30 // EROFS (read-only file system)
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
/// Supports special paths ("null:", "console:") and ramfs files
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
        // Try to find file in ramfs
        let path_str = core::str::from_utf8(path).ok()?;

        // Strip leading "/" for ramfs lookup
        let lookup_path = path_str.trim_start_matches('/');

        // Search through ramfs entries to find matching file
        let ramfs = crate::ramfs::ramfs();
        for idx in 0..ramfs.len() {
            if let Some(entry) = ramfs.get(idx) {
                let entry_name = entry.name_str();
                // Strip leading "./" from tar entries
                let entry_clean = entry_name.trim_start_matches("./");

                if entry_clean == lookup_path && entry.is_file() {
                    return Some(FdEntry {
                        fd_type: FdType::Ramfs {
                            entry_idx: idx,
                            size: entry.size,
                        },
                        flags: FdFlags::read_only(), // ramfs is read-only
                        offset: 0,
                    });
                }
            }
        }

        // Not found
        None
    }
}
