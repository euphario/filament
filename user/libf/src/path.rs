//! Filesystem paths — mirrors `std::path`.
//!
//! Byte-based (not `OsStr`) since Filament paths are always UTF-8 bytes.
//! Single separator `/`, no Windows support.

use alloc::borrow::ToOwned;
use alloc::vec::Vec;
use core::borrow::Borrow;
use core::fmt;
use core::ops::Deref;

const SEP: u8 = b'/';

// ============================================================================
// PathBuf (owned)
// ============================================================================

/// An owned, mutable path — mirrors `std::path::PathBuf`.
#[derive(Clone, PartialEq, Eq, Hash)]
pub struct PathBuf {
    inner: Vec<u8>,
}

impl PathBuf {
    /// Create an empty path.
    pub fn new() -> Self {
        Self { inner: Vec::new() }
    }

    /// Create from a byte slice.
    pub fn from(s: &[u8]) -> Self {
        Self { inner: s.into() }
    }

    /// Append a component, inserting a separator if needed.
    ///
    /// If `component` is absolute (starts with `/`), it replaces the path.
    pub fn push(&mut self, component: &[u8]) {
        if component.first() == Some(&SEP) {
            // Absolute path replaces everything
            self.inner.clear();
            self.inner.extend_from_slice(component);
            return;
        }
        if !self.inner.is_empty() && self.inner.last() != Some(&SEP) {
            self.inner.push(SEP);
        }
        self.inner.extend_from_slice(component);
    }

    /// Remove the last component. Returns false if the path is root or empty.
    pub fn pop(&mut self) -> bool {
        match self.as_path().parent() {
            Some(parent) => {
                let len = parent.inner.len();
                self.inner.truncate(len);
                true
            }
            None => false,
        }
    }

    /// Borrow as a `Path`.
    pub fn as_path(&self) -> &Path {
        Path::new(&self.inner)
    }

    /// Set the file extension. Returns false if there is no file name.
    pub fn set_extension(&mut self, ext: &[u8]) -> bool {
        let file_name = match self.as_path().file_name() {
            Some(f) => f,
            None => return false,
        };

        // Find current extension start
        let stem_end = match file_name.iter().rposition(|&b| b == b'.') {
            Some(dot) if dot > 0 => {
                // Calculate absolute position: path_len - file_name_len + dot
                self.inner.len() - file_name.len() + dot
            }
            _ => self.inner.len(),
        };

        self.inner.truncate(stem_end);
        if !ext.is_empty() {
            self.inner.push(b'.');
            self.inner.extend_from_slice(ext);
        }
        true
    }
}

impl Default for PathBuf {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for PathBuf {
    type Target = Path;
    fn deref(&self) -> &Path {
        self.as_path()
    }
}

impl fmt::Debug for PathBuf {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PathBuf(\"{}\")", self.as_path())
    }
}

impl fmt::Display for PathBuf {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self.as_path(), f)
    }
}

impl From<&[u8]> for PathBuf {
    fn from(s: &[u8]) -> Self {
        PathBuf::from(s)
    }
}

impl From<&Path> for PathBuf {
    fn from(p: &Path) -> Self {
        p.to_path_buf()
    }
}

// ============================================================================
// Path (borrowed, unsized)
// ============================================================================

/// A borrowed path slice — mirrors `std::path::Path`.
///
/// This is an unsized type, always used behind a reference.
/// Zero-cost wrapper around `[u8]`.
#[repr(transparent)]
pub struct Path {
    inner: [u8],
}

impl Path {
    /// Wrap a byte slice as a `Path`. Zero-cost.
    pub fn new(s: &[u8]) -> &Path {
        // SAFETY: Path is #[repr(transparent)] over [u8]
        unsafe { &*(s as *const [u8] as *const Path) }
    }

    /// The raw bytes of this path.
    pub fn as_bytes(&self) -> &[u8] {
        &self.inner
    }

    /// Returns true if the path starts with `/`.
    pub fn is_absolute(&self) -> bool {
        self.inner.first() == Some(&SEP)
    }

    /// Returns true if the path does not start with `/`.
    pub fn is_relative(&self) -> bool {
        !self.is_absolute()
    }

    /// Get the parent directory, or `None` if this is root or empty.
    pub fn parent(&self) -> Option<&Path> {
        if self.inner.is_empty() {
            return None;
        }

        // Strip trailing slashes for parent calculation
        let bytes = strip_trailing_sep(&self.inner);
        if bytes.is_empty() {
            // Was all slashes (root) — no parent
            return None;
        }

        match bytes.iter().rposition(|&b| b == SEP) {
            Some(0) => {
                // Parent is root "/"
                Some(Path::new(b"/"))
            }
            Some(pos) => Some(Path::new(&self.inner[..pos])),
            None => {
                // No slash — relative path, parent is empty
                if bytes.len() == self.inner.len() {
                    Some(Path::new(b""))
                } else {
                    None
                }
            }
        }
    }

    /// Get the final component (file name), or `None` if the path ends
    /// at root or is empty.
    pub fn file_name(&self) -> Option<&[u8]> {
        let bytes = strip_trailing_sep(&self.inner);
        if bytes.is_empty() {
            return None;
        }
        match bytes.iter().rposition(|&b| b == SEP) {
            Some(pos) => {
                let name = &bytes[pos + 1..];
                if name.is_empty() { None } else { Some(name) }
            }
            None => Some(bytes),
        }
    }

    /// Get the file stem (name without extension).
    pub fn file_stem(&self) -> Option<&[u8]> {
        let name = self.file_name()?;
        match name.iter().rposition(|&b| b == b'.') {
            Some(dot) if dot > 0 => Some(&name[..dot]),
            _ => Some(name),
        }
    }

    /// Get the file extension (without the dot).
    pub fn extension(&self) -> Option<&[u8]> {
        let name = self.file_name()?;
        match name.iter().rposition(|&b| b == b'.') {
            Some(dot) if dot > 0 && dot + 1 < name.len() => Some(&name[dot + 1..]),
            _ => None,
        }
    }

    /// Iterate over path components.
    pub fn components(&self) -> Components<'_> {
        Components {
            path: &self.inner,
            pos: 0,
            front_root: self.is_absolute(),
        }
    }

    /// Join this path with another component, returning a new `PathBuf`.
    pub fn join(&self, other: &[u8]) -> PathBuf {
        let mut buf = self.to_path_buf();
        buf.push(other);
        buf
    }

    /// Returns true if this path starts with the given prefix.
    pub fn starts_with(&self, base: &Path) -> bool {
        let mut self_comps = self.components();
        let mut base_comps = base.components();
        loop {
            match (base_comps.next(), self_comps.next()) {
                (None, _) => return true,
                (Some(_), None) => return false,
                (Some(a), Some(b)) => {
                    if a != b {
                        return false;
                    }
                }
            }
        }
    }

    /// Returns true if this path ends with the given suffix.
    pub fn ends_with(&self, child: &Path) -> bool {
        // Compare components from the end
        let self_comps: alloc::vec::Vec<_> = self.components().collect();
        let child_comps: alloc::vec::Vec<_> = child.components().collect();
        if child_comps.len() > self_comps.len() {
            return false;
        }
        let offset = self_comps.len() - child_comps.len();
        self_comps[offset..] == child_comps[..]
    }

    /// Strip a prefix from this path.
    pub fn strip_prefix(&self, base: &Path) -> Result<&Path, StripPrefixError> {
        let mut self_comps = self.components();
        let mut base_comps = base.components();

        loop {
            match base_comps.next() {
                None => {
                    // Consumed all base components — remaining is the result
                    let remaining = &self.inner[self_comps.pos..];
                    // Skip leading separator if present
                    let remaining = if remaining.first() == Some(&SEP) {
                        &remaining[1..]
                    } else {
                        remaining
                    };
                    return Ok(Path::new(remaining));
                }
                Some(base_c) => match self_comps.next() {
                    Some(self_c) if self_c == base_c => continue,
                    _ => return Err(StripPrefixError(())),
                },
            }
        }
    }

    /// Create an owned `PathBuf` from this path.
    pub fn to_path_buf(&self) -> PathBuf {
        PathBuf { inner: self.inner.into() }
    }

    /// Produce a normalized `PathBuf` — resolves `.`, `..`, and double slashes.
    pub fn normalize(&self) -> PathBuf {
        let mut out = Vec::with_capacity(self.inner.len());
        let absolute = self.is_absolute();

        if absolute {
            out.push(SEP);
        }

        for comp in self.components() {
            match comp {
                Component::RootDir | Component::CurDir => {}
                Component::ParentDir => {
                    // Pop last component if there is one
                    if let Some(last_sep) = out.iter().rposition(|&b| b == SEP) {
                        if absolute || last_sep > 0 {
                            out.truncate(last_sep);
                        }
                    } else if !absolute && !out.is_empty() {
                        out.clear();
                    } else if !absolute {
                        if !out.is_empty() {
                            out.push(SEP);
                        }
                        out.extend_from_slice(b"..");
                    }
                }
                Component::Normal(s) => {
                    if !out.is_empty() && out.last() != Some(&SEP) {
                        out.push(SEP);
                    }
                    out.extend_from_slice(s);
                }
            }
        }

        // Ensure absolute paths are at least "/"
        if absolute && out.is_empty() {
            out.push(SEP);
        }

        PathBuf { inner: out }
    }
}

impl PartialEq for Path {
    fn eq(&self, other: &Path) -> bool {
        self.inner == other.inner
    }
}

impl Eq for Path {}

impl fmt::Debug for Path {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Path(\"{}\")", self)
    }
}

impl fmt::Display for Path {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Best effort UTF-8 display
        match core::str::from_utf8(&self.inner) {
            Ok(s) => f.write_str(s),
            Err(_) => write!(f, "<non-utf8 path>"),
        }
    }
}

impl ToOwned for Path {
    type Owned = PathBuf;
    fn to_owned(&self) -> PathBuf {
        self.to_path_buf()
    }
}

impl AsRef<Path> for [u8] {
    fn as_ref(&self) -> &Path {
        Path::new(self)
    }
}

impl AsRef<Path> for PathBuf {
    fn as_ref(&self) -> &Path {
        self.as_path()
    }
}

impl Borrow<Path> for PathBuf {
    fn borrow(&self) -> &Path {
        self.as_path()
    }
}

// ============================================================================
// Components
// ============================================================================

/// A single component of a path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Component<'a> {
    /// The root directory `/`.
    RootDir,
    /// Current directory `.`.
    CurDir,
    /// Parent directory `..`.
    ParentDir,
    /// A normal path segment.
    Normal(&'a [u8]),
}

/// Iterator over path components.
pub struct Components<'a> {
    path: &'a [u8],
    pos: usize,
    front_root: bool,
}

impl<'a> Iterator for Components<'a> {
    type Item = Component<'a>;

    fn next(&mut self) -> Option<Component<'a>> {
        // Emit root first if absolute
        if self.front_root {
            self.front_root = false;
            // Skip the leading '/'
            self.pos = 1;
            return Some(Component::RootDir);
        }

        // Skip separators
        while self.pos < self.path.len() && self.path[self.pos] == SEP {
            self.pos += 1;
        }

        if self.pos >= self.path.len() {
            return None;
        }

        // Find end of segment
        let start = self.pos;
        while self.pos < self.path.len() && self.path[self.pos] != SEP {
            self.pos += 1;
        }

        let segment = &self.path[start..self.pos];
        Some(match segment {
            b"." => Component::CurDir,
            b".." => Component::ParentDir,
            s => Component::Normal(s),
        })
    }
}

// ============================================================================
// StripPrefixError
// ============================================================================

/// Error returned by `Path::strip_prefix` when the path doesn't start
/// with the given prefix.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StripPrefixError(());

impl fmt::Display for StripPrefixError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("prefix not found")
    }
}

// ============================================================================
// Helpers
// ============================================================================

/// Strip trailing separators, preserving root "/".
fn strip_trailing_sep(bytes: &[u8]) -> &[u8] {
    let mut end = bytes.len();
    while end > 1 && bytes[end - 1] == SEP {
        end -= 1;
    }
    &bytes[..end]
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn path_is_absolute() {
        assert!(Path::new(b"/foo").is_absolute());
        assert!(!Path::new(b"foo").is_absolute());
        assert!(Path::new(b"/").is_absolute());
    }

    #[test]
    fn path_parent() {
        assert_eq!(Path::new(b"/foo/bar").parent().unwrap().as_bytes(), b"/foo");
        assert_eq!(Path::new(b"/foo").parent().unwrap().as_bytes(), b"/");
        assert!(Path::new(b"/").parent().is_none());
        assert_eq!(Path::new(b"foo/bar").parent().unwrap().as_bytes(), b"foo");
        assert_eq!(Path::new(b"foo").parent().unwrap().as_bytes(), b"");
    }

    #[test]
    fn path_file_name() {
        assert_eq!(Path::new(b"/foo/bar.txt").file_name(), Some(b"bar.txt".as_ref()));
        assert_eq!(Path::new(b"/foo/").file_name(), Some(b"foo".as_ref()));
        assert_eq!(Path::new(b"/").file_name(), None);
        assert_eq!(Path::new(b"hello").file_name(), Some(b"hello".as_ref()));
    }

    #[test]
    fn path_extension() {
        assert_eq!(Path::new(b"/foo/bar.txt").extension(), Some(b"txt".as_ref()));
        assert_eq!(Path::new(b"/foo/bar").extension(), None);
        assert_eq!(Path::new(b".hidden").extension(), None);
        assert_eq!(Path::new(b"file.tar.gz").extension(), Some(b"gz".as_ref()));
    }

    #[test]
    fn path_file_stem() {
        assert_eq!(Path::new(b"foo.txt").file_stem(), Some(b"foo".as_ref()));
        assert_eq!(Path::new(b".hidden").file_stem(), Some(b".hidden".as_ref()));
        assert_eq!(Path::new(b"archive.tar.gz").file_stem(), Some(b"archive.tar".as_ref()));
    }

    #[test]
    fn path_components() {
        let comps: alloc::vec::Vec<_> = Path::new(b"/foo/bar/baz").components().collect();
        assert_eq!(comps, alloc::vec![
            Component::RootDir,
            Component::Normal(b"foo"),
            Component::Normal(b"bar"),
            Component::Normal(b"baz"),
        ]);
    }

    #[test]
    fn path_components_relative() {
        let comps: alloc::vec::Vec<_> = Path::new(b"foo/bar").components().collect();
        assert_eq!(comps, alloc::vec![
            Component::Normal(b"foo"),
            Component::Normal(b"bar"),
        ]);
    }

    #[test]
    fn path_components_dots() {
        let comps: alloc::vec::Vec<_> = Path::new(b"/foo/../bar/./baz").components().collect();
        assert_eq!(comps, alloc::vec![
            Component::RootDir,
            Component::Normal(b"foo"),
            Component::ParentDir,
            Component::Normal(b"bar"),
            Component::CurDir,
            Component::Normal(b"baz"),
        ]);
    }

    #[test]
    fn path_components_double_slash() {
        let comps: alloc::vec::Vec<_> = Path::new(b"/foo//bar").components().collect();
        assert_eq!(comps, alloc::vec![
            Component::RootDir,
            Component::Normal(b"foo"),
            Component::Normal(b"bar"),
        ]);
    }

    #[test]
    fn pathbuf_push() {
        let mut p = PathBuf::from(b"/foo");
        p.push(b"bar");
        assert_eq!(p.as_bytes(), b"/foo/bar");

        p.push(b"baz.txt");
        assert_eq!(p.as_bytes(), b"/foo/bar/baz.txt");

        // Absolute replaces
        p.push(b"/new/root");
        assert_eq!(p.as_bytes(), b"/new/root");
    }

    #[test]
    fn pathbuf_push_trailing_slash() {
        let mut p = PathBuf::from(b"/foo/");
        p.push(b"bar");
        assert_eq!(p.as_bytes(), b"/foo/bar");
    }

    #[test]
    fn pathbuf_pop() {
        let mut p = PathBuf::from(b"/foo/bar/baz");
        assert!(p.pop());
        assert_eq!(p.as_bytes(), b"/foo/bar");
        assert!(p.pop());
        assert_eq!(p.as_bytes(), b"/foo");
        assert!(p.pop());
        assert_eq!(p.as_bytes(), b"/");
        assert!(!p.pop());
    }

    #[test]
    fn path_join() {
        let p = Path::new(b"/foo");
        let joined = p.join(b"bar/baz");
        assert_eq!(joined.as_bytes(), b"/foo/bar/baz");
    }

    #[test]
    fn path_starts_with() {
        let p = Path::new(b"/foo/bar/baz");
        assert!(p.starts_with(Path::new(b"/foo")));
        assert!(p.starts_with(Path::new(b"/foo/bar")));
        assert!(p.starts_with(Path::new(b"/foo/bar/baz")));
        assert!(!p.starts_with(Path::new(b"/foo/ba")));
        assert!(!p.starts_with(Path::new(b"/bar")));
    }

    #[test]
    fn path_strip_prefix() {
        let p = Path::new(b"/foo/bar/baz");
        let rest = p.strip_prefix(Path::new(b"/foo")).unwrap();
        assert_eq!(rest.as_bytes(), b"bar/baz");

        let rest = p.strip_prefix(Path::new(b"/foo/bar")).unwrap();
        assert_eq!(rest.as_bytes(), b"baz");

        assert!(p.strip_prefix(Path::new(b"/other")).is_err());
    }

    #[test]
    fn path_normalize() {
        assert_eq!(Path::new(b"/foo/../bar").normalize().as_bytes(), b"/bar");
        assert_eq!(Path::new(b"/foo/./bar").normalize().as_bytes(), b"/foo/bar");
        assert_eq!(Path::new(b"/foo//bar").normalize().as_bytes(), b"/foo/bar");
        assert_eq!(Path::new(b"/foo/bar/..").normalize().as_bytes(), b"/foo");
        assert_eq!(Path::new(b"/..").normalize().as_bytes(), b"/");
        assert_eq!(Path::new(b"foo/../bar").normalize().as_bytes(), b"bar");
    }

    #[test]
    fn path_normalize_root() {
        assert_eq!(Path::new(b"/").normalize().as_bytes(), b"/");
        assert_eq!(Path::new(b"///").normalize().as_bytes(), b"/");
    }

    #[test]
    fn pathbuf_set_extension() {
        let mut p = PathBuf::from(b"/foo/bar.txt");
        assert!(p.set_extension(b"md"));
        assert_eq!(p.as_bytes(), b"/foo/bar.md");

        assert!(p.set_extension(b""));
        assert_eq!(p.as_bytes(), b"/foo/bar");

        assert!(p.set_extension(b"rs"));
        assert_eq!(p.as_bytes(), b"/foo/bar.rs");
    }

    #[test]
    fn path_ends_with() {
        let p = Path::new(b"/foo/bar/baz");
        assert!(p.ends_with(Path::new(b"baz")));
        assert!(p.ends_with(Path::new(b"bar/baz")));
        assert!(!p.ends_with(Path::new(b"az")));
    }

    #[test]
    fn path_display() {
        use alloc::format;
        assert_eq!(format!("{}", Path::new(b"/foo/bar")), "/foo/bar");
    }
}
