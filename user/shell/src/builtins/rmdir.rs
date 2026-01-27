//! Rmdir Builtin - Remove Empty Directory
//!
//! Usage:
//!   rmdir <path>    - Remove an empty directory

use crate::output::CommandResult;
use userlib::vfs_client::VfsClient;
use userlib::vfs::VfsError;

/// Main entry point for rmdir builtin
pub fn run(path: &[u8]) -> CommandResult {
    let path = crate::trim(path);

    if path.is_empty() {
        return CommandResult::Error("usage: rmdir <path>");
    }

    // Must be under /mnt for VFS access
    if !path.starts_with(b"/mnt") {
        return CommandResult::Error("rmdir: only /mnt paths supported");
    }

    // Connect to vfsd
    let mut client = match VfsClient::connect() {
        Ok(c) => c,
        Err(_) => return CommandResult::Error("rmdir: cannot connect to vfsd"),
    };

    // Remove directory
    match client.remove_dir(path) {
        Ok(()) => CommandResult::Ok("directory removed"),
        Err(VfsError::NotFound) => CommandResult::Error("rmdir: not found"),
        Err(VfsError::NotEmpty) => CommandResult::Error("rmdir: directory not empty"),
        Err(VfsError::NotDirectory) => CommandResult::Error("rmdir: not a directory"),
        Err(VfsError::PermissionDenied) => CommandResult::Error("rmdir: permission denied"),
        Err(_) => CommandResult::Error("rmdir: failed"),
    }
}
