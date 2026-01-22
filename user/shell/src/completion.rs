//! Tab Completion
//!
//! Provides command completion using builtin commands.
//! Path completion is disabled until vfsd is available.

/// Maximum completions to show
const MAX_COMPLETIONS: usize = 16;

/// Builtin commands for completion
const BUILTINS: &[&[u8]] = &[
    b"help", b"exit", b"quit", b"pid", b"uptime", b"mem", b"echo",
    b"spawn", b"yield", b"panic", b"usb", b"gpio", b"fatfs", b"pcied",
    b"wifi", b"fan", b"ps", b"kill", b"bg", b"jobs", b"log", b"reset",
    b"reboot", b"hw", b"ls", b"cat", b"devd", b"handle", b"resize",
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
/// Returns empty completions (vfsd not available)
pub fn complete_path(_partial: &[u8]) -> Completions {
    // Path completion disabled - vfsd not available
    Completions::empty()
}

/// Complete a command (searches builtins only)
pub fn complete_command(partial: &[u8]) -> Completions {
    let mut result = Completions::empty();

    // Check builtins
    for &builtin in BUILTINS {
        if partial.is_empty() || (builtin.len() >= partial.len() && &builtin[..partial.len()] == partial) {
            result.add_match(builtin);
        }
    }

    result.compute_common();
    result
}
