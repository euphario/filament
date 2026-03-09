//! libf — Filament Standard Library
//!
//! The programming interface for Filament OS. Wraps libsys's raw syscalls
//! into higher-level abstractions: formatting, string operations, number
//! parsing, and alloc re-exports.
//!
//! Relationship to libsys: just as libc wraps kernel syscalls into printf/malloc/FILE*,
//! libf wraps libsys's raw syscalls into StackStr/trim/parse_u32.
//!
//! # Host tests
//!
//! Pure-logic modules (collections, fmt, parse, path, str, time) can be tested
//! on the host: `cargo test --no-default-features`
//!
//! Modules that depend on libsys or crypto crates are gated behind the `target`
//! feature (enabled by default for production builds).

#![cfg_attr(not(test), no_std)]

extern crate alloc;

pub mod collections;
pub mod fmt;
pub mod parse;
pub mod path;
pub mod str;
pub mod time;

// Modules that depend on libsys/crypto (target builds only)
#[cfg(feature = "target")]
pub mod crypto;
#[cfg(feature = "target")]
pub mod io;
#[cfg(feature = "target")]
pub mod net;
#[cfg(feature = "target")]
pub mod sync;

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
    pub use crate::time::Duration;
    #[cfg(feature = "target")]
    pub use crate::time::Instant;
    pub use crate::path::{Path, PathBuf};

    // libsys essentials (target builds only)
    #[cfg(feature = "target")]
    pub use libsys::error::{SysError, SysResult};
}
