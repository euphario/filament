//! cat - Display file contents via VFS

use crate::println;
use crate::output::CommandResult;
use userlib::vfs_proto::open_flags;

pub fn cmd_cat(args: &[u8]) -> CommandResult {
    let path = crate::trim(args);

    if path.is_empty() {
        println!("Usage: cat <path>");
        return CommandResult::None;
    }

    // Resolve path relative to cwd
    let mut resolved_buf = [0u8; crate::cwd::MAX_PATH];
    let path = match crate::get_cwd().resolve(path, &mut resolved_buf) {
        Some(p) => p,
        None => {
            println!("cat: invalid path");
            return CommandResult::None;
        }
    };

    let client = match crate::get_vfs_client() {
        Some(c) => c,
        None => {
            println!("cat: vfsd not available");
            return CommandResult::None;
        }
    };

    let handle = match client.open(path, open_flags::RDONLY) {
        Ok(h) => h,
        Err(e) => {
            crate::builtins::ls::print_vfs_error(b"cat", path, e);
            return CommandResult::None;
        }
    };

    let mut offset: u64 = 0;
    let mut buf = [0u8; 4096];

    loop {
        match client.read(handle, offset, &mut buf) {
            Ok(0) => break, // EOF
            Ok(n) => {
                crate::console::write(&buf[..n]);
                offset += n as u64;
            }
            Err(e) => {
                crate::builtins::ls::print_vfs_error(b"cat", path, e);
                break;
            }
        }
    }

    let _ = client.close(handle);
    CommandResult::None
}
