//! Cp Builtin - Copy File
//!
//! Usage:
//!   cp <src> <dst>  - Copy a file

use crate::println;
use crate::output::CommandResult;
use userlib::vfs_client::VfsClient;
use userlib::vfs::VfsError;

/// Main entry point for cp builtin
pub fn run(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    // Parse "src dst" from args
    let (src, dst) = match parse_two_paths(args) {
        Some(p) => p,
        None => return CommandResult::Error("usage: cp <src> <dst>"),
    };

    // Both must be under /mnt for VFS access
    if !src.starts_with(b"/mnt") || !dst.starts_with(b"/mnt") {
        return CommandResult::Error("cp: only /mnt paths supported");
    }

    // Connect to vfsd
    let mut client = match VfsClient::connect() {
        Ok(c) => c,
        Err(_) => return CommandResult::Error("cp: cannot connect to vfsd"),
    };

    // Copy file
    match client.copy_file(src, dst) {
        Ok(()) => CommandResult::Ok("copied"),
        Err(VfsError::NotFound) => CommandResult::Error("cp: source not found"),
        Err(VfsError::NotFile) => CommandResult::Error("cp: source is a directory"),
        Err(VfsError::PermissionDenied) => CommandResult::Error("cp: permission denied"),
        Err(VfsError::NoSpace) => CommandResult::Error("cp: no space left"),
        Err(_) => CommandResult::Error("cp: failed"),
    }
}

/// Parse two space-separated paths from args
fn parse_two_paths(args: &[u8]) -> Option<(&[u8], &[u8])> {
    // Find first space
    let space_pos = args.iter().position(|&c| c == b' ')?;

    if space_pos == 0 || space_pos >= args.len() - 1 {
        return None;
    }

    let src = crate::trim(&args[..space_pos]);
    let dst = crate::trim(&args[space_pos + 1..]);

    if src.is_empty() || dst.is_empty() {
        return None;
    }

    Some((src, dst))
}
