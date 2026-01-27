//! Cat Builtin - Display File Contents
//!
//! Usage:
//!   cat <path>      - Display file contents

use crate::println;
use crate::output::{ByteBuffer, CommandResult};
use userlib::vfs_client::VfsClient;
use userlib::vfs::VfsError;

/// Max bytes to read per chunk (IPC limit constraint)
const CHUNK_SIZE: u32 = 512;

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

    // Read file in chunks until eof
    let mut output = ByteBuffer::new();
    let mut offset: u64 = 0;
    let mut resp_buf = [0u8; 1024]; // 512 data + headers

    loop {
        match client.read_file(path, offset, CHUNK_SIZE, &mut resp_buf) {
            Ok(content) => {
                let added = output.push(content.data);
                offset += added as u64;

                // Stop if we hit eof or filled buffer
                if content.eof || added < content.data.len() {
                    break;
                }
            }
            Err(VfsError::NotFound) => return CommandResult::Error("cat: file not found"),
            Err(VfsError::PermissionDenied) => return CommandResult::Error("cat: permission denied"),
            Err(VfsError::NotDirectory) => return CommandResult::Error("cat: is a directory"),
            Err(_) => return CommandResult::Error("cat: read failed"),
        }
    }

    // Note if output buffer filled up (file larger than ByteBuffer can hold)
    if output.is_full() {
        println!("... (truncated, file larger than {} bytes)", output.capacity());
    }

    CommandResult::Bytes(output)
}
