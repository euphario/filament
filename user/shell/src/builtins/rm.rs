//! Rm Builtin - Remove File
//!
//! Usage:
//!   rm <path>       - Remove a file

use crate::output::CommandResult;
use userlib::vfs_client::VfsClient;
use userlib::vfs::VfsError;

/// Main entry point for rm builtin
pub fn run(path: &[u8]) -> CommandResult {
    let path = crate::trim(path);

    if path.is_empty() {
        return CommandResult::Error("usage: rm <path>");
    }

    // Must be under /mnt for VFS access
    if !path.starts_with(b"/mnt") {
        return CommandResult::Error("rm: only /mnt paths supported");
    }

    // Connect to vfsd
    let mut client = match VfsClient::connect() {
        Ok(c) => c,
        Err(_) => return CommandResult::Error("rm: cannot connect to vfsd"),
    };

    // Remove file
    match client.remove_file(path) {
        Ok(()) => CommandResult::Ok("removed"),
        Err(VfsError::NotFound) => CommandResult::Error("rm: not found"),
        Err(VfsError::NotFile) => CommandResult::Error("rm: is a directory (use rmdir)"),
        Err(VfsError::PermissionDenied) => CommandResult::Error("rm: permission denied"),
        Err(_) => CommandResult::Error("rm: failed"),
    }
}
