//! Tab Completion
//!
//! Provides path and command completion using vfsd and builtin commands.

use userlib::syscall;
use userlib::ipc::Message;
use userlib::ipc::protocols::{
    FsRequest, FsResponse,
    filesystem::MAX_DIR_ENTRIES,
};

/// Maximum completions to show
const MAX_COMPLETIONS: usize = 16;

/// Builtin commands for completion
const BUILTINS: &[&[u8]] = &[
    b"help", b"exit", b"quit", b"pid", b"uptime", b"mem", b"echo",
    b"spawn", b"yield", b"panic", b"usb", b"gpio", b"fatfs", b"pcied",
    b"wifi", b"fan", b"ps", b"kill", b"bg", b"jobs", b"log", b"reset",
    b"reboot", b"hw", b"ls", b"cat",
];

/// Completion result
pub struct Completions {
    /// Common prefix that can be completed
    pub common: [u8; 64],
    pub common_len: usize,
    /// Number of matches
    pub count: usize,
    /// Individual matches (for display)
    matches: [[u8; 32]; MAX_COMPLETIONS],
    match_lens: [usize; MAX_COMPLETIONS],
}

impl Completions {
    pub const fn empty() -> Self {
        Self {
            common: [0u8; 64],
            common_len: 0,
            count: 0,
            matches: [[0u8; 32]; MAX_COMPLETIONS],
            match_lens: [0; MAX_COMPLETIONS],
        }
    }

    /// Get a match by index
    pub fn get_match(&self, idx: usize) -> Option<&[u8]> {
        if idx < self.count {
            Some(&self.matches[idx][..self.match_lens[idx]])
        } else {
            None
        }
    }

    fn add_match(&mut self, name: &[u8]) {
        if self.count >= MAX_COMPLETIONS {
            return;
        }
        let len = name.len().min(32);
        self.matches[self.count][..len].copy_from_slice(&name[..len]);
        self.match_lens[self.count] = len;
        self.count += 1;
    }

    fn compute_common(&mut self) {
        if self.count == 0 {
            self.common_len = 0;
            return;
        }

        if self.count == 1 {
            let len = self.match_lens[0].min(64);
            self.common[..len].copy_from_slice(&self.matches[0][..len]);
            self.common_len = len;
            return;
        }

        // Find common prefix among all matches
        let first = &self.matches[0][..self.match_lens[0]];
        let mut common_len = first.len().min(64);

        for i in 1..self.count {
            let m = &self.matches[i][..self.match_lens[i]];
            let mut j = 0;
            while j < common_len && j < m.len() && first[j] == m[j] {
                j += 1;
            }
            common_len = j;
        }

        self.common[..common_len].copy_from_slice(&first[..common_len]);
        self.common_len = common_len;
    }
}

/// Complete a path prefix
/// Returns completions for the given partial path
pub fn complete_path(partial: &[u8]) -> Completions {
    let mut result = Completions::empty();

    // Find the last '/' to split into directory and prefix
    let (dir_path, prefix) = if let Some(slash_pos) = partial.iter().rposition(|&c| c == b'/') {
        (&partial[..slash_pos + 1], &partial[slash_pos + 1..])
    } else {
        // No slash - assume current directory or bin/
        (b"/bin/" as &[u8], partial)
    };

    // Connect to vfsd
    let ch = syscall::port_connect(b"vfs");
    if ch < 0 {
        return result;
    }
    let ch = ch as u32;

    // Request directory listing
    let request = FsRequest::read_dir(dir_path, 0);

    let mut req_buf = [0u8; 512];
    let req_len = match request.serialize(&mut req_buf) {
        Ok(len) => len,
        Err(_) => {
            syscall::channel_close(ch);
            return result;
        }
    };

    if syscall::send(ch, &req_buf[..req_len]) < 0 {
        syscall::channel_close(ch);
        return result;
    }

    let mut resp_buf = [0u8; 2048];
    let received = syscall::receive(ch, &mut resp_buf);
    syscall::channel_close(ch);

    if received <= 0 {
        return result;
    }

    let response = match FsResponse::deserialize(&resp_buf[..received as usize]) {
        Ok((resp, _)) => resp,
        Err(_) => return result,
    };

    // Find matching entries
    if let FsResponse::DirEntries { entries, count, .. } = response {
        for i in 0..(count as usize).min(MAX_DIR_ENTRIES) {
            let entry = &entries[i];
            let name = &entry.name[..entry.name_len as usize];

            // Check if name starts with prefix
            if prefix.is_empty() || (name.len() >= prefix.len() && &name[..prefix.len()] == prefix) {
                result.add_match(name);
            }
        }
    }

    result.compute_common();
    result
}

/// Complete a command (searches builtins + /bin)
pub fn complete_command(partial: &[u8]) -> Completions {
    let mut result = Completions::empty();

    // First, check builtins
    for &builtin in BUILTINS {
        if partial.is_empty() || (builtin.len() >= partial.len() && &builtin[..partial.len()] == partial) {
            result.add_match(builtin);
        }
    }

    // Then check /bin/ via vfsd
    let bin_completions = complete_path_in_dir(b"/bin/", partial);
    for i in 0..bin_completions.count {
        if let Some(m) = bin_completions.get_match(i) {
            // Avoid duplicates (builtin might have same name as binary)
            let mut is_dup = false;
            for j in 0..result.count {
                if let Some(existing) = result.get_match(j) {
                    if existing == m {
                        is_dup = true;
                        break;
                    }
                }
            }
            if !is_dup {
                result.add_match(m);
            }
        }
    }

    result.compute_common();
    result
}

/// Complete a path within a specific directory
fn complete_path_in_dir(dir: &[u8], prefix: &[u8]) -> Completions {
    let mut result = Completions::empty();

    // Connect to vfsd
    let ch = syscall::port_connect(b"vfs");
    if ch < 0 {
        return result;
    }
    let ch = ch as u32;

    // Request directory listing
    let request = FsRequest::read_dir(dir, 0);

    let mut req_buf = [0u8; 512];
    let req_len = match request.serialize(&mut req_buf) {
        Ok(len) => len,
        Err(_) => {
            syscall::channel_close(ch);
            return result;
        }
    };

    if syscall::send(ch, &req_buf[..req_len]) < 0 {
        syscall::channel_close(ch);
        return result;
    }

    let mut resp_buf = [0u8; 2048];
    let received = syscall::receive(ch, &mut resp_buf);
    syscall::channel_close(ch);

    if received <= 0 {
        return result;
    }

    let response = match FsResponse::deserialize(&resp_buf[..received as usize]) {
        Ok((resp, _)) => resp,
        Err(_) => return result,
    };

    // Find matching entries
    if let FsResponse::DirEntries { entries, count, .. } = response {
        for i in 0..(count as usize).min(MAX_DIR_ENTRIES) {
            let entry = &entries[i];
            let name = &entry.name[..entry.name_len as usize];

            // Check if name starts with prefix
            if prefix.is_empty() || (name.len() >= prefix.len() && &name[..prefix.len()] == prefix) {
                result.add_match(name);
            }
        }
    }

    result
}
