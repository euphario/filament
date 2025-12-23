//! Kernel Logging Infrastructure

#![allow(dead_code)]  // Infrastructure for future use
//!
//! Provides structured logging with timestamps, log levels, and module names.

use core::sync::atomic::{AtomicU8, AtomicU64, Ordering};
use crate::logln;

/// Log levels from most to least severe
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3,
    Trace = 4,
}

impl LogLevel {
    pub fn as_str(self) -> &'static str {
        match self {
            LogLevel::Error => "ERROR",
            LogLevel::Warn => "WARN ",
            LogLevel::Info => "INFO ",
            LogLevel::Debug => "DEBUG",
            LogLevel::Trace => "TRACE",
        }
    }

    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(LogLevel::Error),
            1 => Some(LogLevel::Warn),
            2 => Some(LogLevel::Info),
            3 => Some(LogLevel::Debug),
            4 => Some(LogLevel::Trace),
            _ => None,
        }
    }
}

/// Current log level - messages at or below this level are printed (atomic for SMP safety)
static LOG_LEVEL: AtomicU8 = AtomicU8::new(LogLevel::Info as u8);

/// Boot time in counter ticks (atomic for SMP safety - written once at init)
static BOOT_TIME: AtomicU64 = AtomicU64::new(0);

/// Counter frequency in Hz (atomic for SMP safety - written once at init)
static COUNTER_FREQ: AtomicU64 = AtomicU64::new(24_000_000); // Default 24MHz

/// Initialize the logging system
pub fn init() {
    let boot_time: u64;
    let counter_freq: u64;
    unsafe {
        // Read current counter as boot time
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) boot_time);

        // Read counter frequency
        core::arch::asm!("mrs {}, cntfrq_el0", out(reg) counter_freq);
    }
    BOOT_TIME.store(boot_time, Ordering::Release);
    COUNTER_FREQ.store(counter_freq, Ordering::Release);
}

/// Set the current log level
pub fn set_level(level: LogLevel) {
    LOG_LEVEL.store(level as u8, Ordering::Release);
}

/// Get the current log level
pub fn get_level() -> LogLevel {
    LogLevel::from_u8(LOG_LEVEL.load(Ordering::Acquire)).unwrap_or(LogLevel::Info)
}

/// Check if a log level is enabled
#[inline]
pub fn is_enabled(level: LogLevel) -> bool {
    level as u8 <= LOG_LEVEL.load(Ordering::Relaxed)
}

/// Get milliseconds since boot
pub fn timestamp_ms() -> u64 {
    let now: u64;
    unsafe {
        core::arch::asm!("mrs {}, cntpct_el0", out(reg) now);
    }
    let boot = BOOT_TIME.load(Ordering::Acquire);
    let freq = COUNTER_FREQ.load(Ordering::Acquire);
    let elapsed = now.wrapping_sub(boot);
    // Convert to milliseconds: elapsed * 1000 / freq
    // To avoid overflow, do: elapsed / (freq / 1000)
    let divisor = freq / 1000;
    if divisor > 0 {
        elapsed / divisor
    } else {
        elapsed / 24_000 // Fallback for 24MHz
    }
}

/// Internal log function - called by macros
#[doc(hidden)]
pub fn _log(level: LogLevel, module: &str, args: core::fmt::Arguments) {
    if !is_enabled(level) {
        return;
    }

    let ts = timestamp_ms();
    let secs = ts / 1000;
    let millis = ts % 1000;

    // Shorten module path for readability
    // "bpi_r4_kernel::gic" -> "gic"
    let short_module = module
        .rsplit("::")
        .next()
        .unwrap_or(module);

    logln!(
        "[{:8}.{:03}] {} [{}] {}",
        secs, millis, level.as_str(), short_module, args
    );
}

/// Log macro - use the convenience macros below instead
#[macro_export]
macro_rules! klog {
    ($level:expr, $($arg:tt)*) => {{
        if $crate::log::is_enabled($level) {
            $crate::log::_log($level, module_path!(), format_args!($($arg)*));
        }
    }};
}

/// Log an error message
#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => {
        $crate::klog!($crate::log::LogLevel::Error, $($arg)*)
    };
}

/// Log a warning message
#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => {
        $crate::klog!($crate::log::LogLevel::Warn, $($arg)*)
    };
}

/// Log an info message
#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => {
        $crate::klog!($crate::log::LogLevel::Info, $($arg)*)
    };
}

/// Log a debug message
#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => {
        $crate::klog!($crate::log::LogLevel::Debug, $($arg)*)
    };
}

/// Log a trace message
#[macro_export]
macro_rules! log_trace {
    ($($arg:tt)*) => {
        $crate::klog!($crate::log::LogLevel::Trace, $($arg)*)
    };
}
