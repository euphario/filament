//! RAM Filesystem (initrd) support
//!
//! Provides access to files from a TAR archive loaded into memory at boot.
//! The initrd can be loaded by U-Boot alongside the kernel.

use crate::println;

/// TAR block size
const BLOCK_SIZE: usize = 512;

/// Maximum number of files in ramfs
const MAX_FILES: usize = 64;

/// Maximum filename length
const MAX_NAME_LEN: usize = 100;

/// TAR file type flags
mod typeflag {
    pub const REGULAR: u8 = b'0';
    pub const REGULAR_ALT: u8 = 0; // Old tar compatibility
    pub const DIRECTORY: u8 = b'5';
}

/// A file entry in the ramfs
#[derive(Clone, Copy)]
pub struct RamfsEntry {
    /// Filename (null-terminated)
    pub name: [u8; MAX_NAME_LEN],
    /// File size in bytes
    pub size: usize,
    /// Pointer to file data
    pub data: *const u8,
    /// File type (regular, directory, etc.)
    pub file_type: u8,
}

impl RamfsEntry {
    const fn empty() -> Self {
        Self {
            name: [0; MAX_NAME_LEN],
            size: 0,
            data: core::ptr::null(),
            file_type: 0,
        }
    }

    /// Get filename as string slice
    pub fn name_str(&self) -> &str {
        let len = self.name.iter().position(|&c| c == 0).unwrap_or(MAX_NAME_LEN);
        core::str::from_utf8(&self.name[..len]).unwrap_or("")
    }

    /// Get file data as slice
    pub fn data_slice(&self) -> &[u8] {
        if self.data.is_null() || self.size == 0 {
            &[]
        } else {
            unsafe { core::slice::from_raw_parts(self.data, self.size) }
        }
    }

    /// Check if this is a regular file
    pub fn is_file(&self) -> bool {
        self.file_type == typeflag::REGULAR || self.file_type == typeflag::REGULAR_ALT
    }

    /// Check if this is a directory
    pub fn is_dir(&self) -> bool {
        self.file_type == typeflag::DIRECTORY
    }
}

/// The RAM filesystem
pub struct Ramfs {
    entries: [RamfsEntry; MAX_FILES],
    count: usize,
    base_addr: usize,
    total_size: usize,
}

impl Ramfs {
    /// Create an empty ramfs
    const fn new() -> Self {
        Self {
            entries: [RamfsEntry::empty(); MAX_FILES],
            count: 0,
            base_addr: 0,
            total_size: 0,
        }
    }

    /// Initialize ramfs from a TAR archive in memory
    /// Returns number of files found
    pub fn init(&mut self, base: usize, size: usize) -> usize {
        self.base_addr = base;
        self.total_size = size;
        self.count = 0;

        let data = unsafe { core::slice::from_raw_parts(base as *const u8, size) };

        // Parse TAR archive
        let mut offset = 0;
        while offset + BLOCK_SIZE <= size && self.count < MAX_FILES {
            let header = &data[offset..offset + BLOCK_SIZE];

            // Check for end of archive (two zero blocks)
            if header.iter().all(|&b| b == 0) {
                break;
            }

            // Parse TAR header
            let name = &header[0..100];
            let size_octal = &header[124..136];
            let typeflag = header[156];

            // Parse size (octal ASCII)
            let file_size = parse_octal(size_octal);

            // Skip if name is empty
            if name[0] == 0 {
                break;
            }

            // Create entry
            let mut entry = RamfsEntry::empty();
            let name_len = name.iter().position(|&c| c == 0).unwrap_or(100);
            entry.name[..name_len].copy_from_slice(&name[..name_len]);
            entry.size = file_size;
            entry.file_type = typeflag;

            // Data starts in next block
            let data_offset = offset + BLOCK_SIZE;
            if data_offset < size {
                entry.data = unsafe { (base as *const u8).add(data_offset) };
            }

            self.entries[self.count] = entry;
            self.count += 1;

            // Move to next file (header + data blocks)
            let data_blocks = (file_size + BLOCK_SIZE - 1) / BLOCK_SIZE;
            offset += BLOCK_SIZE + data_blocks * BLOCK_SIZE;
        }

        self.count
    }

    /// Find a file by name
    pub fn find(&self, name: &str) -> Option<&RamfsEntry> {
        let name_bytes = name.as_bytes();
        for i in 0..self.count {
            let entry = &self.entries[i];
            let entry_name = entry.name_str();

            // Exact match
            if entry_name == name {
                return Some(entry);
            }

            // Match without leading "./"
            if entry_name.starts_with("./") && &entry_name[2..] == name {
                return Some(entry);
            }

            // Match with trailing "/" stripped (for directories)
            let stripped = entry_name.trim_end_matches('/');
            if stripped == name {
                return Some(entry);
            }
        }
        None
    }

    /// Get file by index
    pub fn get(&self, index: usize) -> Option<&RamfsEntry> {
        if index < self.count {
            Some(&self.entries[index])
        } else {
            None
        }
    }

    /// Get number of files
    pub fn len(&self) -> usize {
        self.count
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// List all files (for debugging)
    pub fn list(&self) {
        println!("  Ramfs contents ({} files):", self.count);
        for i in 0..self.count {
            let entry = &self.entries[i];
            let type_char = if entry.is_dir() { 'd' } else { '-' };
            println!("    {} {:>8} {}", type_char, entry.size, entry.name_str());
        }
    }
}

/// Parse octal ASCII string to usize
fn parse_octal(s: &[u8]) -> usize {
    let mut result = 0usize;
    for &c in s {
        if c == 0 || c == b' ' {
            break;
        }
        if c >= b'0' && c <= b'7' {
            result = result * 8 + (c - b'0') as usize;
        }
    }
    result
}

// Global ramfs instance
static mut RAMFS: Ramfs = Ramfs::new();

/// Initialize the global ramfs from a TAR archive
/// Call this at boot with the address and size of the initrd
pub fn init(base: usize, size: usize) -> usize {
    unsafe { RAMFS.init(base, size) }
}

/// Get a reference to the global ramfs
pub fn ramfs() -> &'static Ramfs {
    unsafe { &*core::ptr::addr_of!(RAMFS) }
}

/// Find a file by name
pub fn find(name: &str) -> Option<&'static RamfsEntry> {
    ramfs().find(name)
}

/// Find and return file data
pub fn read_file(name: &str) -> Option<&'static [u8]> {
    find(name).map(|e| e.data_slice())
}

/// List all files
pub fn list() {
    ramfs().list();
}

/// Test ramfs functionality
pub fn test() {
    println!("  Testing ramfs...");

    let fs = ramfs();
    if fs.is_empty() {
        println!("    No initrd loaded");
        return;
    }

    list();
    println!("    [OK] Ramfs initialized");
}
