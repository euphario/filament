//! Hardware Query Builtin
//!
//! Query devd's hw: scheme for device information.
//!
//! Usage:
//!   hw              - Show all devices (hw list)
//!   hw list         - Show all devices
//!   hw bus          - List bus controllers
//!   hw tree         - Show device tree
//!   hw <path>       - Query specific device path

use userlib::{println, syscall};
use crate::output::CommandResult;

/// Main entry point for hw builtin
pub fn run(args: &[u8]) -> CommandResult {
    let args = crate::trim(args);

    // Default to "list" if no args
    let path = if args.is_empty() {
        "list"
    } else {
        match core::str::from_utf8(args) {
            Ok(s) => s.trim(),
            Err(_) => {
                return CommandResult::Error("invalid path");
            }
        }
    };

    // Build the scheme URL: "hw:<path>"
    let mut url_buf = [0u8; 64];
    let prefix = b"hw:";
    url_buf[..prefix.len()].copy_from_slice(prefix);
    let path_bytes = path.as_bytes();
    let url_len = prefix.len() + path_bytes.len().min(url_buf.len() - prefix.len());
    url_buf[prefix.len()..url_len].copy_from_slice(&path_bytes[..url_len - prefix.len()]);

    let url = match core::str::from_utf8(&url_buf[..url_len]) {
        Ok(s) => s,
        Err(_) => return CommandResult::Error("invalid url"),
    };

    // Open the scheme
    let fd = syscall::scheme_open(url, 0);
    if fd < 0 {
        println!("Failed to open {}: error {}", url, fd);
        return CommandResult::None;
    }

    // Read the response (retry on EAGAIN/-11 as we may need to wait for devd)
    let mut buf = [0u8; 1024];
    let mut len: isize = -11;
    let mut retries = 0;
    while len == -11 && retries < 100 {
        len = syscall::read(fd as u32, &mut buf);
        if len == -11 {
            syscall::yield_now();
            retries += 1;
        }
    }
    syscall::close(fd as u32);

    if len < 0 {
        println!("Failed to read from {}: error {}", url, len);
        return CommandResult::None;
    }

    // Print the response directly (devd formats it)
    if len > 0 {
        if let Ok(s) = core::str::from_utf8(&buf[..len as usize]) {
            userlib::print!("{}", s);
        }
    }

    CommandResult::None
}
