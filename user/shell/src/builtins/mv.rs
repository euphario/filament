//! Mv Builtin - Move/Rename File
//!
//! Usage:
//!   mv <src> <dst>  - Move or rename a file

use crate::output::CommandResult;
use userlib::vfs_client::VfsClient;
use userlib::vfs::VfsError;

/// Main entry point for mv builtin
pub fn run(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    // Parse "src dst" from args
    let (src, dst) = match parse_two_paths(args) {
        Some(p) => p,
        None => return CommandResult::Error("usage: mv <src> <dst>"),
    };

    // Both must be under /mnt for VFS access
    if !src.starts_with(b"/mnt") || !dst.starts_with(b"/mnt") {
        return CommandResult::Error("mv: only /mnt paths supported");
    }

    // Connect to vfsd
    let mut client = match VfsClient::connect() {
        Ok(c) => c,
        Err(_) => return CommandResult::Error("mv: cannot connect to vfsd"),
    };

    // Move = copy + delete (no atomic rename yet)
    // First copy
    if let Err(e) = client.copy_file(src, dst) {
        return match e {
            VfsError::NotFound => CommandResult::Error("mv: source not found"),
            VfsError::NotFile => CommandResult::Error("mv: source is a directory"),
            VfsError::PermissionDenied => CommandResult::Error("mv: permission denied"),
            VfsError::NoSpace => CommandResult::Error("mv: no space left"),
            _ => CommandResult::Error("mv: copy failed"),
        };
    }

    // Then delete source
    match client.remove_file(src) {
        Ok(()) => CommandResult::Ok("moved"),
        Err(_) => {
            // Copy succeeded but delete failed - file exists in both places
            CommandResult::Error("mv: copied but failed to remove source")
        }
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
