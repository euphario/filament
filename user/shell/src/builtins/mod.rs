//! Shell Built-in Commands
//!
//! Modular built-in commands that can be easily detached if needed.
//! Each module provides a `run(args: &[u8])` function.
//!
//! To disable a builtin, simply comment out its `pub mod` line and the
//! corresponding match arm in main.rs.

pub mod gpio;
pub mod hw;
pub mod ls;
pub mod ps;

// Future builtins:
// pub mod fan;
// pub mod jobs;
