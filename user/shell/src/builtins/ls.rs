//! Directory Listing Builtin
//!
//! List directory contents.
//!
//! Usage:
//!   ls              - List current directory
//!   ls /            - List root (shows /bin, /mnt)
//!   ls /bin         - List embedded binaries
//!   ls /mnt         - List mounted filesystems
//!   ls /mnt/fat0    - List FAT filesystem root

use crate::println;
use crate::output::{Table, Row, Align, CommandResult};
use crate::cwd::MAX_PATH;
use userlib::ipc::Channel;
use userlib::vfs::{ListDir, DirEntries, DirEntry, VfsHeader, msg, file_type};

/// Known binaries in /bin (embedded in initrd)
const KNOWN_BINARIES: &[&str] = &[
    "shell",
    "devd",
    "consoled",
    "vfsd",
    "partition",
    "fatfs",
    "qemu-usbd",
];

/// Main entry point for ls builtin
pub fn run(args: &[u8]) -> CommandResult {
    let path = crate::trim(args);

    // Use cwd if no path given
    let resolved_path: &[u8] = if path.is_empty() {
        crate::get_cwd().as_bytes()
    } else if path[0] != b'/' {
        // Relative path - resolve against cwd
        // For now, just prepend cwd (proper resolution in cwd.rs)
        static mut RESOLVED: [u8; MAX_PATH] = [0u8; MAX_PATH];
        let resolved = unsafe { &mut *core::ptr::addr_of_mut!(RESOLVED) };
        match crate::get_cwd().resolve(path, resolved) {
            Some(p) => p,
            None => {
                println!("ls: invalid path");
                return CommandResult::None;
            }
        }
    } else {
        path
    };

    // Route based on path
    if resolved_path == b"/" {
        return ls_root();
    }

    if resolved_path == b"/bin" || resolved_path.starts_with(b"/bin/") {
        return ls_bin();
    }

    if resolved_path == b"/mnt" || resolved_path.starts_with(b"/mnt") {
        return ls_vfs(resolved_path);
    }

    println!("ls: path not found");
    CommandResult::None
}

/// List root directory (virtual)
fn ls_root() -> CommandResult {
    let mut table = Table::new(&["TYPE", "NAME"])
        .align(0, Align::Left);

    // Root contains /bin and /mnt
    table.add_row(Row::empty().str("dir").str("bin"));
    table.add_row(Row::empty().str("dir").str("mnt"));

    CommandResult::Table(table)
}

/// List /bin directory (hardcoded)
fn ls_bin() -> CommandResult {
    let mut table = Table::new(&["TYPE", "NAME"])
        .align(0, Align::Left);

    for &name in KNOWN_BINARIES {
        let row = Row::empty()
            .str("file")
            .str(name);
        table.add_row(row);
    }

    CommandResult::Table(table)
}

/// List via VFS daemon
fn ls_vfs(path: &[u8]) -> CommandResult {
    // Connect to vfsd
    let mut channel = match Channel::connect(b"vfs:") {
        Ok(ch) => ch,
        Err(e) => {
            println!("ls: cannot connect to vfsd: {:?}", e);
            return CommandResult::None;
        }
    };

    // Send LIST_DIR request
    let req = ListDir::new(1); // seq_id = 1
    let mut buf = [0u8; 256];
    let len = match req.write_to(&mut buf, path) {
        Some(n) => n,
        None => {
            println!("ls: path too long");
            return CommandResult::None;
        }
    };

    if let Err(e) = channel.send(&buf[..len]) {
        println!("ls: send failed: {:?}", e);
        return CommandResult::None;
    }

    // Wait for response
    if let Err(e) = userlib::ipc::wait_one(channel.handle()) {
        println!("ls: wait failed: {:?}", e);
        return CommandResult::None;
    }

    // Receive response
    let mut resp_buf = [0u8; 512];
    let resp_len = match channel.recv(&mut resp_buf) {
        Ok(n) => n,
        Err(e) => {
            println!("ls: recv failed: {:?}", e);
            return CommandResult::None;
        }
    };

    // Parse response header
    let header = match VfsHeader::from_bytes(&resp_buf[..resp_len]) {
        Some(h) => h,
        None => {
            println!("ls: invalid response");
            return CommandResult::None;
        }
    };

    // Check for error response
    if header.msg_type == msg::RESULT {
        // Need VfsHeader::SIZE + 4 bytes to read the i32 result code
        let min_result_size = VfsHeader::SIZE + 4;
        let code = if resp_len >= min_result_size {
            i32::from_le_bytes([
                resp_buf[VfsHeader::SIZE],
                resp_buf[VfsHeader::SIZE + 1],
                resp_buf[VfsHeader::SIZE + 2],
                resp_buf[VfsHeader::SIZE + 3],
            ])
        } else {
            -1
        };
        println!("ls: error {}", code);
        return CommandResult::None;
    }

    // Parse DIR_ENTRIES
    if header.msg_type != msg::DIR_ENTRIES {
        println!("ls: unexpected response type {}", header.msg_type);
        return CommandResult::None;
    }

    let entries = match DirEntries::from_bytes(&resp_buf[..resp_len]) {
        Some(e) => e,
        None => {
            println!("ls: invalid dir entries");
            return CommandResult::None;
        }
    };

    if entries.count == 0 {
        println!("(empty directory)");
        return CommandResult::None;
    }

    // Build table
    let mut table = Table::new(&["TYPE", "SIZE", "NAME"])
        .align(1, Align::Right);

    // Parse each entry
    let mut offset = DirEntries::HEADER_SIZE;
    for _ in 0..entries.count {
        if offset >= resp_len {
            break;
        }

        let (entry, name) = match DirEntry::from_bytes(&resp_buf[offset..resp_len]) {
            Some(e) => e,
            None => break,
        };

        let type_str = match entry.file_type {
            t if t == file_type::DIRECTORY => "dir",
            t if t == file_type::FILE => "file",
            t if t == file_type::SYMLINK => "link",
            _ => "?",
        };

        // Copy name to avoid lifetime issues
        let name_str = core::str::from_utf8(name).unwrap_or("?");

        let row = Row::empty()
            .str(type_str)
            .uint(entry.size)
            .string(name_str);  // Use string() instead of str() for non-static
        table.add_row(row);

        offset += entry.total_size();
    }

    CommandResult::Table(table)
}
