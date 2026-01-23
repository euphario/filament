//! Platform abstraction layer
//!
//! This module provides conditional compilation for different hardware platforms.
//! The `current` submodule re-exports the active platform based on Cargo features.

#[cfg(feature = "platform-mt7988a")]
pub mod mt7988;

#[cfg(feature = "platform-qemu-virt")]
pub mod qemu_virt;

/// Current platform re-export
///
/// Use `platform::current::*` for platform-agnostic access to platform modules.
/// This allows kernel code to work across different platforms without
/// hardcoding platform names.
pub mod current {
    #[cfg(feature = "platform-mt7988a")]
    pub use super::mt7988::*;

    #[cfg(feature = "platform-qemu-virt")]
    pub use super::qemu_virt::*;
}
