//! Minimal test helper — just exits successfully.
//!
//! Used by test_spawn_storm to exercise process spawn/wait/reap.

#![no_std]
#![no_main]

#[unsafe(no_mangle)]
fn main() {
    libsys::syscall::exit(0);
}
