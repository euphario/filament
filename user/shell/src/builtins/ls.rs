//! ls - List directory contents via VFS

use crate::println;
use crate::output::{CommandResult, Table, Row, Value, Align};
use userlib::vfs_proto::{open_flags, file_type, VfsDirEntry};

pub fn cmd_ls(args: &[u8]) -> CommandResult {
    let path = crate::trim(args);

    // Resolve path relative to cwd
    let mut resolved_buf = [0u8; crate::cwd::MAX_PATH];
    let path = match crate::get_cwd().resolve(path, &mut resolved_buf) {
        Some(p) => p,
        None => return CommandResult::Error("invalid path"),
    };

    let client = match crate::get_vfs_client() {
        Some(c) => c,
        None => return CommandResult::Error("vfsd not available"),
    };

    let handle = match client.open(path, open_flags::DIR | open_flags::RDONLY) {
        Ok(h) => h,
        Err(e) => {
            print_vfs_error(b"ls", path, e);
            return CommandResult::None;
        }
    };

    let mut entries = [VfsDirEntry::empty(); 32];
    let result = match client.readdir(handle, &mut entries) {
        Ok(count) => {
            let mut table = Table::new(&["NAME", "TYPE", "SIZE"])
                .align(2, Align::Right);

            for i in 0..count {
                let e = &entries[i];
                let name = e.name_bytes();

                let type_str = if e.file_type == file_type::DIR {
                    "dir"
                } else {
                    "file"
                };

                let size_val = format_size(e.size);

                table.add_row(
                    Row::empty()
                        .bytes(name)
                        .str(type_str)
                        .bytes(&size_val.0[..size_val.1])
                );
            }

            CommandResult::Table(table)
        }
        Err(e) => {
            print_vfs_error(b"ls", path, e);
            CommandResult::None
        }
    };

    let _ = client.close(handle);
    result
}

/// Format size into a human-readable string, returns (buffer, length)
fn format_size(size: u32) -> ([u8; 16], usize) {
    let mut buf = [0u8; 16];
    let len;
    if size >= 1024 * 1024 {
        let mb = size / (1024 * 1024);
        let frac = (size % (1024 * 1024)) / (1024 * 100);
        len = write_dec(&mut buf, mb);
        buf[len] = b'.';
        let len2 = write_dec(&mut buf[len + 1..], frac);
        buf[len + 1 + len2] = b'M';
        (buf, len + 1 + len2 + 1)
    } else if size >= 1024 {
        let kb = size / 1024;
        let frac = (size % 1024) / 100;
        len = write_dec(&mut buf, kb);
        buf[len] = b'.';
        let len2 = write_dec(&mut buf[len + 1..], frac);
        buf[len + 1 + len2] = b'K';
        (buf, len + 1 + len2 + 1)
    } else {
        len = write_dec(&mut buf, size);
        buf[len] = b'B';
        (buf, len + 1)
    }
}

fn write_dec(buf: &mut [u8], val: u32) -> usize {
    if val == 0 {
        buf[0] = b'0';
        return 1;
    }
    let mut n = val;
    let mut digits = 0;
    let mut tmp = [0u8; 10];
    while n > 0 {
        tmp[digits] = b'0' + (n % 10) as u8;
        n /= 10;
        digits += 1;
    }
    for i in 0..digits {
        buf[i] = tmp[digits - 1 - i];
    }
    digits
}

pub fn print_vfs_error(cmd: &[u8], path: &[u8], e: userlib::vfs_client::VfsError) {
    use userlib::vfs_client::VfsError;
    crate::console::write(cmd);
    crate::console::write(b": ");
    crate::console::write(path);
    crate::console::write(b": ");
    let msg = match e {
        VfsError::NotFound => b"not found" as &[u8],
        VfsError::NotDir => b"not a directory",
        VfsError::IsDir => b"is a directory",
        VfsError::Permission => b"permission denied",
        VfsError::NoMount => b"no mount point",
        VfsError::ReadOnly => b"read-only filesystem",
        VfsError::Timeout => b"timeout",
        VfsError::NotAvailable => b"vfsd not available",
        _ => b"I/O error",
    };
    crate::console::write(msg);
    println!();
}
