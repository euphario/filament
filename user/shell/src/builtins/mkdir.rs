//! Mkdir Builtin - Create Directory
//!
//! Usage:
//!   mkdir <path>    - Create a directory

use crate::output::CommandResult;
use userlib::vfs_client::VfsClient;
use userlib::vfs::VfsError;

/// Main entry point for mkdir builtin
pub fn run(path: &[u8]) -> CommandResult {
    let path = crate::trim(path);

    if path.is_empty() {
        return CommandResult::Error("usage: mkdir <path>");
    }

    // Must be under /mnt for VFS access
    if !path.starts_with(b"/mnt") {
        return CommandResult::Error("mkdir: only /mnt paths supported");
    }

    // Connect to vfsd
    let mut client = match VfsClient::connect() {
        Ok(c) => c,
        Err(_) => return CommandResult::Error("mkdir: cannot connect to vfsd"),
    };

    // Create directory
    match client.mkdir(path) {
        Ok(()) => CommandResult::Ok("directory created"),
        Err(VfsError::AlreadyExists) => CommandResult::Error("mkdir: already exists"),
        Err(VfsError::NotFound) => CommandResult::Error("mkdir: parent directory not found"),
        Err(VfsError::PermissionDenied) => CommandResult::Error("mkdir: permission denied"),
        Err(_) => CommandResult::Error("mkdir: failed"),
    }
}
