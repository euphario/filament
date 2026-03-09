//! libf — Filament Standard Library
//!
//! The programming interface for Filament OS. Wraps libsys's raw syscalls
//! into higher-level abstractions: formatting, string operations, number
//! parsing, and alloc re-exports.
//!
//! Relationship to libsys: just as libc wraps kernel syscalls into printf/malloc/FILE*,
//! libf wraps libsys's raw syscalls into StackStr/trim/parse_u32.

#![no_std]

extern crate alloc;

pub mod collections;
pub mod crypto;
pub mod fmt;
pub mod io;
pub mod net;
pub mod parse;
pub mod path;
pub mod str;
pub mod sync;
pub mod time;

/// Prelude — import everything a typical program needs.
///
/// ```ignore
/// use libf::prelude::*;
/// ```
pub mod prelude {
    // Alloc types
    pub extern crate alloc;
    pub use alloc::string::String;
    pub use alloc::vec::Vec;
    pub use alloc::vec;
    pub use alloc::boxed::Box;
    pub use alloc::format;

    // Collections
    pub use alloc::collections::VecDeque;

    // libf utilities
    pub use crate::fmt::{StackStr, BufWriter};
    pub use crate::str::{trim, eq_ignore_ascii_case};
    pub use crate::parse::parse_u32;
    pub use crate::time::{Duration, Instant};
    pub use crate::path::{Path, PathBuf};

    // libsys essentials
    pub use libsys::error::{SysError, SysResult};
}
