//! libf — Filament Standard Library
//!
//! The programming interface for Filament OS. Wraps userlib's raw syscalls
//! into higher-level abstractions: formatting, string operations, number
//! parsing, and alloc re-exports.
//!
//! Relationship to userlib: just as libc wraps kernel syscalls into printf/malloc/FILE*,
//! libf wraps userlib's raw syscalls into StackStr/trim/parse_u32.

#![no_std]

extern crate alloc;

pub mod fmt;
pub mod parse;
pub mod str;

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

    // libf utilities
    pub use crate::fmt::StackStr;
    pub use crate::str::{trim, eq_ignore_ascii_case};
    pub use crate::parse::parse_u32;

    // userlib essentials
    pub use userlib::error::{SysError, SysResult};
}
