//! Cat Builtin - Display File Contents
//!
//! Usage:
//!   cat <path>      - Display file contents

use crate::println;
use crate::output::{ByteBuffer, CommandResult};
use userlib::vfs_client::VfsClient;
use userlib::vfs::VfsError;

/// Main entry point for cat builtin
pub fn run(path: &[u8]) -> CommandResult {
    let path = crate::trim(path);

    if path.is_empty() {
        return CommandResult::Error("usage: cat <path>");
    }

    // Must be under /mnt for VFS access
    if !path.starts_with(b"/mnt") {
        return CommandResult::Error("cat: only /mnt paths supported");
    }

    // Connect to vfsd
    let mut client = match VfsClient::connect() {
        Ok(c) => c,
        Err(_) => return CommandResult::Error("cat: cannot connect to vfsd"),
    };

    // Read file
    let mut resp_buf = [0u8; 4096 + 64];
    match client.read_file(path, 0, 4096, &mut resp_buf) {
        Ok(content) => {
            let mut output = ByteBuffer::new();
            output.push(content.data);

            // Note truncation if file is larger
            if !content.eof && content.data.len() >= 4096 {
                println!("... (truncated, file larger than 4KB)");
            }

            CommandResult::Bytes(output)
        }
        Err(VfsError::NotFound) => CommandResult::Error("cat: file not found"),
        Err(VfsError::PermissionDenied) => CommandResult::Error("cat: permission denied"),
        Err(VfsError::NotDirectory) => CommandResult::Error("cat: is a directory"),
        Err(_) => CommandResult::Error("cat: read failed"),
    }
}
