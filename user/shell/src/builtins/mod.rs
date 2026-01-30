//! Shell Built-in Commands
//!
//! Modular built-in commands that can be easily detached if needed.
//! Each module provides a `run(args: &[u8])` function.
//!
//! To disable a builtin, simply comment out its `pub mod` line and the
//! corresponding match arm in main.rs.

pub mod cat;
pub mod devd;
pub mod drivers;
pub mod gpio;
pub mod handle;
pub mod hw;
pub mod logs;
pub mod ls;
pub mod lsdev;
pub mod ps;
