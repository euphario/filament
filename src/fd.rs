//! File Descriptor Management
//!
//! Provides file descriptor abstraction for user processes.
//! In Redox style, everything is a file - console, devices, IPC, etc.

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
    // Future: scheme-based file
    // Scheme { scheme_id: u32, handle: u64 },
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
pub fn fd_read(entry: &FdEntry, buf: &mut [u8]) -> isize {
    if !entry.flags.readable {
        return -1; // EBADF
    }

    match entry.fd_type {
        FdType::None => -1,
        FdType::ConsoleIn => {
            // Read from UART (blocking for now)
            // For simplicity, just read one character if available
            if buf.is_empty() {
                return 0;
            }
            // Non-blocking read attempt
            match uart::try_getc() {
                Some(c) => {
                    buf[0] = c as u8;
                    1
                }
                None => {
                    if entry.flags.nonblocking {
                        -11 // EAGAIN
                    } else {
                        // Would block - for now return 0
                        0
                    }
                }
            }
        }
        FdType::ConsoleOut => -1, // Can't read from output
        FdType::Null => 0,        // EOF
        FdType::Channel(_id) => {
            // TODO: Read from IPC channel
            -1
        }
    }
}

/// Write to a file descriptor
/// Returns number of bytes written, or negative error
pub fn fd_write(entry: &FdEntry, buf: &[u8]) -> isize {
    if !entry.flags.writable {
        return -1; // EBADF
    }

    match entry.fd_type {
        FdType::None => -1,
        FdType::ConsoleIn => -1, // Can't write to input
        FdType::ConsoleOut => {
            // Write to UART
            for &byte in buf {
                uart::putc(byte as char);
            }
            buf.len() as isize
        }
        FdType::Null => buf.len() as isize, // Discard
        FdType::Channel(_id) => {
            // TODO: Write to IPC channel
            -1
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
