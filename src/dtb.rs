//! Embedded Device Tree Blob (DTB) support
//!
//! This module includes the compiled DTB at compile time.
//! The DTB is compiled from bpi-r4.dts by build.sh using dtc.

/// Embedded DTB data
/// Built by: dtc -I dts -O dtb -o bpi-r4.dtb bpi-r4.dts
#[cfg(feature = "embed-dtb")]
static DTB_DATA: &[u8] = include_bytes!("../bpi-r4.dtb");

/// Placeholder when DTB is not embedded
#[cfg(not(feature = "embed-dtb"))]
static DTB_DATA: &[u8] = &[];

/// Get the embedded DTB data
/// Returns the DTB slice or None if not embedded
pub fn get_embedded_dtb() -> Option<&'static [u8]> {
    if DTB_DATA.is_empty() {
        None
    } else {
        Some(DTB_DATA)
    }
}

/// Get physical address of embedded DTB
/// Note: This is a virtual address in kernel space, needs conversion
pub fn get_embedded_dtb_addr() -> Option<u64> {
    if DTB_DATA.is_empty() {
        None
    } else {
        Some(DTB_DATA.as_ptr() as u64)
    }
}

/// Check if DTB is embedded
pub fn has_embedded_dtb() -> bool {
    !DTB_DATA.is_empty()
}
