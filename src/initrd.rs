//! Embedded initrd support

#![allow(dead_code)]  // Infrastructure for future use
//!
//! This module includes the initrd TAR archive at compile time.
//! The data is compiled directly into the kernel binary via include_bytes!.

/// Embedded initrd data
/// Built by: cd user && ./mkinitrd.sh
#[cfg(feature = "embed-initrd")]
static INITRD_DATA: &[u8] = include_bytes!("../user/initrd.tar");

/// Placeholder when initrd is not embedded
#[cfg(not(feature = "embed-initrd"))]
static INITRD_DATA: &[u8] = &[];

/// Get the embedded initrd data
/// Returns (address, size) or None if no initrd embedded
pub fn get_embedded_initrd() -> Option<(usize, usize)> {
    if INITRD_DATA.is_empty() {
        None
    } else {
        Some((INITRD_DATA.as_ptr() as usize, INITRD_DATA.len()))
    }
}

/// Check if initrd is embedded
pub fn has_embedded_initrd() -> bool {
    !INITRD_DATA.is_empty()
}
