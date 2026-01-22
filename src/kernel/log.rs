//! Kernel Logging - DEPRECATED
//!
//! This module is kept for backwards compatibility only.
//! All logging should use klog (kinfo!, kerror!, etc.) which writes to LOG_RING.
//!
//! LOG_BUFFER has been removed. Use klog::flush() instead of log::flush().

/// Deprecated - use klog::flush() instead
#[inline]
pub fn flush() {
    crate::klog::flush();
}

/// Deprecated - use klog::flush() instead
#[inline]
pub fn try_flush() -> bool {
    crate::klog::try_drain(8);
    true
}
