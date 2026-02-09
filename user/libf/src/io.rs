//! I/O traits and error types — mirrors `std::io`.
//!
//! Provides [`Read`], [`Write`], and [`BufRead`] traits with the same
//! signatures as `std::io`. The [`Error`] type wraps [`SysError`] into
//! std-compatible [`ErrorKind`] variants.
//!
//! [`SysError`]: userlib::error::SysError

extern crate alloc;

use alloc::vec::Vec;
use core::fmt;
use userlib::error::SysError;

// ============================================================================
// Error types
// ============================================================================

/// A list specifying general categories of I/O error.
///
/// Mirrors `std::io::ErrorKind`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorKind {
    NotFound,
    PermissionDenied,
    ConnectionRefused,
    ConnectionReset,
    BrokenPipe,
    AlreadyExists,
    WouldBlock,
    InvalidInput,
    TimedOut,
    Interrupted,
    OutOfMemory,
    UnexpectedEof,
    Unsupported,
    Other,
}

impl ErrorKind {
    fn as_str(&self) -> &'static str {
        match self {
            ErrorKind::NotFound => "entity not found",
            ErrorKind::PermissionDenied => "permission denied",
            ErrorKind::ConnectionRefused => "connection refused",
            ErrorKind::ConnectionReset => "connection reset",
            ErrorKind::BrokenPipe => "broken pipe",
            ErrorKind::AlreadyExists => "entity already exists",
            ErrorKind::WouldBlock => "operation would block",
            ErrorKind::InvalidInput => "invalid input parameter",
            ErrorKind::TimedOut => "timed out",
            ErrorKind::Interrupted => "operation interrupted",
            ErrorKind::OutOfMemory => "out of memory",
            ErrorKind::UnexpectedEof => "unexpected end of file",
            ErrorKind::Unsupported => "unsupported",
            ErrorKind::Other => "other error",
        }
    }
}

impl fmt::Display for ErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// The error type for I/O operations.
///
/// Mirrors `std::io::Error`. Wraps an [`ErrorKind`].
pub struct Error {
    kind: ErrorKind,
}

impl Error {
    /// Creates a new I/O error from an [`ErrorKind`].
    pub fn new(kind: ErrorKind) -> Self {
        Self { kind }
    }

    /// Returns the corresponding [`ErrorKind`] for this error.
    pub fn kind(&self) -> ErrorKind {
        self.kind
    }
}

impl From<ErrorKind> for Error {
    fn from(kind: ErrorKind) -> Self {
        Self { kind }
    }
}

impl From<SysError> for Error {
    fn from(e: SysError) -> Self {
        let kind = match e {
            SysError::NotFound => ErrorKind::NotFound,
            SysError::PermissionDenied | SysError::AccessDenied => ErrorKind::PermissionDenied,
            SysError::ConnectionRefused => ErrorKind::ConnectionRefused,
            SysError::ConnectionReset => ErrorKind::ConnectionReset,
            SysError::PeerClosed => ErrorKind::BrokenPipe,
            SysError::AlreadyExists => ErrorKind::AlreadyExists,
            SysError::WouldBlock => ErrorKind::WouldBlock,
            SysError::InvalidArgument => ErrorKind::InvalidInput,
            SysError::Timeout => ErrorKind::TimedOut,
            SysError::Interrupted => ErrorKind::Interrupted,
            SysError::OutOfMemory => ErrorKind::OutOfMemory,
            SysError::NotImplemented | SysError::OperationNotSupported => ErrorKind::Unsupported,
            _ => ErrorKind::Other,
        };
        Self { kind }
    }
}

impl fmt::Debug for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Error")
            .field("kind", &self.kind)
            .finish()
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.kind.as_str())
    }
}

// ============================================================================
// Result type alias
// ============================================================================

/// A specialized [`Result`] type for I/O operations.
///
/// Mirrors `std::io::Result`.
pub type Result<T> = core::result::Result<T, Error>;

// ============================================================================
// Read trait
// ============================================================================

/// The `Read` trait allows for reading bytes from a source.
///
/// Mirrors `std::io::Read`.
pub trait Read {
    /// Pull some bytes from this source into the specified buffer, returning
    /// how many bytes were read.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize>;

    /// Read the exact number of bytes required to fill `buf`.
    ///
    /// Returns `Err(UnexpectedEof)` if EOF is hit before `buf` is full.
    fn read_exact(&mut self, mut buf: &mut [u8]) -> Result<()> {
        while !buf.is_empty() {
            match self.read(buf) {
                Ok(0) => return Err(Error::new(ErrorKind::UnexpectedEof)),
                Ok(n) => buf = &mut buf[n..],
                Err(ref e) if e.kind() == ErrorKind::Interrupted => {}
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// Read all bytes until EOF, appending to `buf`.
    ///
    /// Returns the number of bytes read.
    fn read_to_end(&mut self, buf: &mut Vec<u8>) -> Result<usize> {
        let mut total = 0;
        let mut tmp = [0u8; 512];
        loop {
            match self.read(&mut tmp) {
                Ok(0) => return Ok(total),
                Ok(n) => {
                    buf.extend_from_slice(&tmp[..n]);
                    total += n;
                }
                Err(ref e) if e.kind() == ErrorKind::Interrupted => {}
                Err(e) => return Err(e),
            }
        }
    }
}

// ============================================================================
// Write trait
// ============================================================================

/// A trait for objects which are byte-oriented sinks.
///
/// Mirrors `std::io::Write`.
pub trait Write {
    /// Write a buffer into this writer, returning how many bytes were written.
    fn write(&mut self, buf: &[u8]) -> Result<usize>;

    /// Flush this output stream, ensuring all buffered contents reach their
    /// destination.
    fn flush(&mut self) -> Result<()>;

    /// Attempts to write an entire buffer into this writer.
    fn write_all(&mut self, mut buf: &[u8]) -> Result<()> {
        while !buf.is_empty() {
            match self.write(buf) {
                Ok(0) => return Err(Error::new(ErrorKind::Other)),
                Ok(n) => buf = &buf[n..],
                Err(ref e) if e.kind() == ErrorKind::Interrupted => {}
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }
}

// ============================================================================
// BufRead trait
// ============================================================================

/// A `Read`er with an internal buffer, allowing certain kinds of
/// reading to be done more efficiently.
///
/// Mirrors `std::io::BufRead`.
pub trait BufRead: Read {
    /// Returns the contents of the internal buffer, filling it with more data
    /// from the inner reader if it is empty.
    fn fill_buf(&mut self) -> Result<&[u8]>;

    /// Tells this buffer that `amt` bytes have been consumed from the buffer,
    /// so they should no longer be returned in calls to `fill_buf`.
    fn consume(&mut self, amt: usize);
}

// ============================================================================
// Impls on userlib types
// ============================================================================

impl Read for userlib::ipc::Channel {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.recv(buf).map_err(Error::from)
    }
}

impl Write for userlib::ipc::Channel {
    fn write(&mut self, buf: &[u8]) -> Result<usize> {
        self.send(buf).map(|()| buf.len()).map_err(Error::from)
    }

    fn flush(&mut self) -> Result<()> {
        Ok(())
    }
}

// ============================================================================
// Cursor
// ============================================================================

/// A `Cursor` wraps an in-memory buffer and provides `Read` and/or `Write`
/// implementations for it.
///
/// Mirrors `std::io::Cursor`.
pub struct Cursor<T> {
    inner: T,
    pos: u64,
}

impl<T> Cursor<T> {
    /// Creates a new cursor wrapping the provided underlying value.
    pub fn new(inner: T) -> Self {
        Self { inner, pos: 0 }
    }

    /// Returns the current position of this cursor.
    pub fn position(&self) -> u64 {
        self.pos
    }

    /// Sets the position of this cursor.
    pub fn set_position(&mut self, pos: u64) {
        self.pos = pos;
    }

    /// Consumes this cursor, returning the underlying value.
    pub fn into_inner(self) -> T {
        self.inner
    }

    /// Gets a reference to the underlying value.
    pub fn get_ref(&self) -> &T {
        &self.inner
    }

    /// Gets a mutable reference to the underlying value.
    pub fn get_mut(&mut self) -> &mut T {
        &mut self.inner
    }
}

impl Read for Cursor<&[u8]> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        let pos = self.pos as usize;
        if pos >= self.inner.len() {
            return Ok(0);
        }
        let remaining = &self.inner[pos..];
        let n = remaining.len().min(buf.len());
        buf[..n].copy_from_slice(&remaining[..n]);
        self.pos += n as u64;
        Ok(n)
    }
}

impl Read for Cursor<Vec<u8>> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        let pos = self.pos as usize;
        if pos >= self.inner.len() {
            return Ok(0);
        }
        let remaining = &self.inner[pos..];
        let n = remaining.len().min(buf.len());
        buf[..n].copy_from_slice(&remaining[..n]);
        self.pos += n as u64;
        Ok(n)
    }
}

impl Write for Cursor<Vec<u8>> {
    fn write(&mut self, buf: &[u8]) -> Result<usize> {
        let pos = self.pos as usize;
        // Extend if writing past end
        if pos + buf.len() > self.inner.len() {
            self.inner.resize(pos + buf.len(), 0);
        }
        self.inner[pos..pos + buf.len()].copy_from_slice(buf);
        self.pos += buf.len() as u64;
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<()> {
        Ok(())
    }
}

impl BufRead for Cursor<&[u8]> {
    fn fill_buf(&mut self) -> Result<&[u8]> {
        let pos = self.pos as usize;
        if pos >= self.inner.len() {
            Ok(&[])
        } else {
            Ok(&self.inner[pos..])
        }
    }

    fn consume(&mut self, amt: usize) {
        self.pos += amt as u64;
    }
}

impl BufRead for Cursor<Vec<u8>> {
    fn fill_buf(&mut self) -> Result<&[u8]> {
        let pos = self.pos as usize;
        if pos >= self.inner.len() {
            Ok(&[])
        } else {
            Ok(&self.inner[pos..])
        }
    }

    fn consume(&mut self, amt: usize) {
        self.pos += amt as u64;
    }
}

// ============================================================================
// Helpers
// ============================================================================

/// Copies the entire contents of a reader into a writer.
///
/// Returns the number of bytes copied. Mirrors `std::io::copy`.
pub fn copy(reader: &mut dyn Read, writer: &mut dyn Write) -> Result<u64> {
    let mut buf = [0u8; 512];
    let mut total: u64 = 0;
    loop {
        let n = match reader.read(&mut buf) {
            Ok(0) => return Ok(total),
            Ok(n) => n,
            Err(ref e) if e.kind() == ErrorKind::Interrupted => continue,
            Err(e) => return Err(e),
        };
        writer.write_all(&buf[..n])?;
        total += n as u64;
    }
}
