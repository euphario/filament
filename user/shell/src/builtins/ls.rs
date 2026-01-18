//! Directory Listing Builtin
//!
//! List directory contents via vfsd with structured output.
//!
//! Usage:
//!   ls              - List /bin directory
//!   ls <path>       - List specified directory

use userlib::{println, syscall};
use userlib::ipc::Message;
use userlib::ipc::protocols::{
    FsRequest, FsResponse, fs_error, FileType,
    filesystem::MAX_DIR_ENTRIES,
};
use crate::output::{Table, Row, Align, CommandResult};

/// Main entry point for ls builtin
pub fn run(args: &[u8]) -> CommandResult {
    let path = crate::trim(args);

    // Default to /bin if no path given
    let path = if path.is_empty() { b"/bin" as &[u8] } else { path };

    // Connect to vfsd
    let ch = syscall::port_connect(b"vfs");
    if ch < 0 {
        return CommandResult::Error("vfsd not running");
    }
    let ch = ch as u32;

    // Build ReadDir request
    let request = FsRequest::read_dir(path, 0);

    // Serialize and send
    let mut req_buf = [0u8; 512];
    let req_len = match request.serialize(&mut req_buf) {
        Ok(len) => len,
        Err(_) => {
            syscall::channel_close(ch);
            return CommandResult::Error("serialize failed");
        }
    };

    let sent = syscall::send(ch, &req_buf[..req_len]);
    if sent < 0 {
        syscall::channel_close(ch);
        return CommandResult::Error("send failed");
    }

    // Receive response
    let mut resp_buf = [0u8; 2048];
    let received = syscall::receive(ch, &mut resp_buf);
    syscall::channel_close(ch);

    if received <= 0 {
        return CommandResult::Error("no response");
    }

    // Deserialize response
    let response = match FsResponse::deserialize(&resp_buf[..received as usize]) {
        Ok((resp, _)) => resp,
        Err(_) => {
            return CommandResult::Error("parse failed");
        }
    };

    // Handle response
    match response {
        FsResponse::DirEntries { entries, count, more: _ } => {
            if count == 0 {
                return CommandResult::Ok("(empty directory)");
            }

            // Build structured table
            let mut table = Table::new(&["TYPE", "SIZE", "NAME"])
                .align(1, Align::Right);  // Right-align SIZE

            for i in 0..(count as usize).min(MAX_DIR_ENTRIES) {
                let entry = &entries[i];

                // Type indicator
                let type_str: &'static str = match entry.file_type {
                    FileType::Directory => "dir",
                    FileType::Regular => "file",
                    FileType::Unknown => "????",
                };

                // Build row with name from bytes
                let row = Row::empty()
                    .str(type_str)
                    .uint(entry.size)
                    .bytes(&entry.name[..entry.name_len as usize]);

                table.add_row(row);
            }

            CommandResult::Table(table)
        }
        FsResponse::Error(code) => {
            let msg: &'static str = match code {
                c if c == fs_error::NOT_FOUND => "not found",
                c if c == fs_error::PERMISSION => "permission denied",
                c if c == fs_error::INVALID => "invalid path",
                _ => "unknown error",
            };
            println!("ls: {}", msg);
            CommandResult::None
        }
        _ => {
            CommandResult::Error("unexpected response")
        }
    }
}
