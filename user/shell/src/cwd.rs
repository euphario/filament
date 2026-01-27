//! Current Working Directory
//!
//! Tracks the shell's current working directory for path resolution.

/// Maximum path length
pub const MAX_PATH: usize = 256;

/// Current working directory state
pub struct WorkingDir {
    path: [u8; MAX_PATH],
    len: usize,
}

impl WorkingDir {
    /// Create new working directory at root
    pub const fn new() -> Self {
        let mut path = [0u8; MAX_PATH];
        path[0] = b'/';
        Self { path, len: 1 }
    }

    /// Get current path as bytes
    pub fn as_bytes(&self) -> &[u8] {
        &self.path[..self.len]
    }

    /// Get current path as str (if valid UTF-8)
    pub fn as_str(&self) -> Option<&str> {
        core::str::from_utf8(&self.path[..self.len]).ok()
    }

    /// Change directory
    /// Returns true on success, false if path too long
    pub fn cd(&mut self, path: &[u8]) -> bool {
        if path.is_empty() {
            // cd with no args = go to root
            self.path[0] = b'/';
            self.len = 1;
            return true;
        }

        if path[0] == b'/' {
            // Absolute path
            self.set_absolute(path)
        } else {
            // Relative path
            self.set_relative(path)
        }
    }

    /// Set to absolute path
    fn set_absolute(&mut self, path: &[u8]) -> bool {
        // Normalize the path
        let mut new_path = [0u8; MAX_PATH];
        let new_len = normalize_path(path, &mut new_path);

        if new_len == 0 || new_len > MAX_PATH {
            return false;
        }

        self.path[..new_len].copy_from_slice(&new_path[..new_len]);
        self.len = new_len;
        true
    }

    /// Set relative to current directory
    fn set_relative(&mut self, path: &[u8]) -> bool {
        // Build combined path
        let mut combined = [0u8; MAX_PATH * 2];

        // Copy current path
        combined[..self.len].copy_from_slice(&self.path[..self.len]);
        let mut pos = self.len;

        // Add separator if needed
        if pos > 0 && combined[pos - 1] != b'/' {
            if pos >= combined.len() {
                return false;
            }
            combined[pos] = b'/';
            pos += 1;
        }

        // Add relative path
        if pos + path.len() > combined.len() {
            return false;
        }
        combined[pos..pos + path.len()].copy_from_slice(path);
        pos += path.len();

        // Normalize
        let mut new_path = [0u8; MAX_PATH];
        let new_len = normalize_path(&combined[..pos], &mut new_path);

        if new_len == 0 || new_len > MAX_PATH {
            return false;
        }

        self.path[..new_len].copy_from_slice(&new_path[..new_len]);
        self.len = new_len;
        true
    }

    /// Resolve a path relative to cwd
    /// Returns the resolved path in the output buffer
    pub fn resolve<'a>(&self, path: &[u8], out: &'a mut [u8; MAX_PATH]) -> Option<&'a [u8]> {
        if path.is_empty() {
            // Empty path = cwd
            out[..self.len].copy_from_slice(&self.path[..self.len]);
            return Some(&out[..self.len]);
        }

        if path[0] == b'/' {
            // Absolute path - normalize and return
            let len = normalize_path(path, out);
            if len > 0 {
                Some(&out[..len])
            } else {
                None
            }
        } else {
            // Relative path - combine with cwd
            let mut combined = [0u8; MAX_PATH * 2];

            // Copy cwd
            combined[..self.len].copy_from_slice(&self.path[..self.len]);
            let mut pos = self.len;

            // Add separator
            if pos > 0 && combined[pos - 1] != b'/' {
                combined[pos] = b'/';
                pos += 1;
            }

            // Add path
            if pos + path.len() > combined.len() {
                return None;
            }
            combined[pos..pos + path.len()].copy_from_slice(path);
            pos += path.len();

            let len = normalize_path(&combined[..pos], out);
            if len > 0 {
                Some(&out[..len])
            } else {
                None
            }
        }
    }

    /// Check if cwd is under a given prefix
    pub fn starts_with(&self, prefix: &[u8]) -> bool {
        if prefix.len() > self.len {
            return false;
        }
        &self.path[..prefix.len()] == prefix
    }
}

/// Normalize a path (handle . and ..)
/// Returns length of normalized path, or 0 on error
fn normalize_path(input: &[u8], output: &mut [u8; MAX_PATH]) -> usize {
    let mut components: [&[u8]; 32] = [&[]; 32];
    let mut count = 0;

    // Split by /
    let mut start = 0;
    for i in 0..input.len() {
        if input[i] == b'/' {
            if i > start {
                let comp = &input[start..i];
                if comp == b"." {
                    // Skip
                } else if comp == b".." {
                    if count > 0 {
                        count -= 1;
                    }
                } else if count < components.len() {
                    components[count] = comp;
                    count += 1;
                }
            }
            start = i + 1;
        }
    }

    // Handle last component
    if start < input.len() {
        let comp = &input[start..];
        if comp == b"." {
            // Skip
        } else if comp == b".." {
            if count > 0 {
                count -= 1;
            }
        } else if count < components.len() {
            components[count] = comp;
            count += 1;
        }
    }

    // Build output
    let mut pos = 0;
    output[pos] = b'/';
    pos += 1;

    for i in 0..count {
        if i > 0 {
            if pos >= output.len() {
                return 0;
            }
            output[pos] = b'/';
            pos += 1;
        }

        let comp = components[i];
        if pos + comp.len() > output.len() {
            return 0;
        }
        output[pos..pos + comp.len()].copy_from_slice(comp);
        pos += comp.len();
    }

    pos
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize() {
        let mut out = [0u8; MAX_PATH];

        assert_eq!(normalize_path(b"/", &mut out), 1);
        assert_eq!(&out[..1], b"/");

        assert_eq!(normalize_path(b"/foo/bar", &mut out), 8);
        assert_eq!(&out[..8], b"/foo/bar");

        assert_eq!(normalize_path(b"/foo/../bar", &mut out), 4);
        assert_eq!(&out[..4], b"/bar");

        assert_eq!(normalize_path(b"/foo/./bar", &mut out), 8);
        assert_eq!(&out[..8], b"/foo/bar");
    }
}
