//! Current Working Directory
//!
//! Tracks the shell's current working directory for path resolution.
//! Uses `libf::path::PathBuf` for normalization and joining.

use libf::path::{Path, PathBuf};

/// Current working directory state
pub struct WorkingDir {
    /// None means root "/", avoids allocation in const context.
    path: Option<PathBuf>,
}

impl WorkingDir {
    /// Create new working directory at root
    pub const fn new() -> Self {
        Self { path: None }
    }

    fn path_ref(&self) -> &Path {
        match &self.path {
            Some(p) => p.as_path(),
            None => Path::new(b"/"),
        }
    }

    /// Get current path as bytes
    pub fn as_bytes(&self) -> &[u8] {
        self.path_ref().as_bytes()
    }

    /// Get current path as str (if valid UTF-8)
    pub fn as_str(&self) -> Option<&str> {
        core::str::from_utf8(self.as_bytes()).ok()
    }

    /// Change directory. Returns true on success.
    pub fn cd(&mut self, path: &[u8]) -> bool {
        if path.is_empty() {
            self.path = None; // root
            return true;
        }

        let new = if path[0] == b'/' {
            Path::new(path).normalize()
        } else {
            self.path_ref().join(path).normalize()
        };
        self.path = Some(new);
        true
    }

    /// Resolve a path relative to cwd. Returns the normalized path.
    pub fn resolve(&self, path: &[u8]) -> PathBuf {
        if path.is_empty() {
            return self.path_ref().to_path_buf();
        }

        if path[0] == b'/' {
            Path::new(path).normalize()
        } else {
            self.path_ref().join(path).normalize()
        }
    }
}
