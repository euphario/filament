//! Structured tracing for userspace drivers
//!
//! Provides subsystem-based logging with filtering support.
//!
//! # Usage
//!
//! ```rust
//! use userlib::{trace, trace_enabled};
//!
//! // Basic tracing
//! trace!(DMA, "ring {} initialized", ring_id);
//! trace!(MCU, "cmd=0x{:02x} seq={}", cmd, seq);
//!
//! // Conditional tracing (for expensive operations)
//! if trace_enabled!(FW) {
//!     trace!(FW, "firmware region dump: {:?}", region);
//! }
//! ```

use core::sync::atomic::{AtomicU32, Ordering};

/// Trace subsystems - each bit corresponds to a subsystem
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Subsystem {
    /// General/default subsystem
    GEN = 0,
    /// DMA engine operations
    DMA = 1,
    /// MCU/firmware communication
    MCU = 2,
    /// Firmware loading
    FW = 3,
    /// PCIe operations
    PCIE = 4,
    /// USB operations
    USB = 5,
    /// WiFi/wireless
    WIFI = 6,
    /// Initialization sequences
    INIT = 7,
    /// Interrupt handling
    IRQ = 8,
    /// Register access (very verbose)
    REG = 9,
    /// Error conditions
    ERR = 10,
}

impl Subsystem {
    /// Get the bit mask for this subsystem
    #[inline]
    pub const fn mask(self) -> u32 {
        1 << (self as u32)
    }

    /// Get the name as a static string
    pub const fn name(self) -> &'static str {
        match self {
            Subsystem::GEN => "GEN",
            Subsystem::DMA => "DMA",
            Subsystem::MCU => "MCU",
            Subsystem::FW => "FW",
            Subsystem::PCIE => "PCIE",
            Subsystem::USB => "USB",
            Subsystem::WIFI => "WIFI",
            Subsystem::INIT => "INIT",
            Subsystem::IRQ => "IRQ",
            Subsystem::REG => "REG",
            Subsystem::ERR => "ERR",
        }
    }
}

/// Global trace mask - bits set = subsystems enabled
/// Default: all enabled except REG (too verbose)
static TRACE_MASK: AtomicU32 = AtomicU32::new(
    (1 << Subsystem::GEN as u32)
    | (1 << Subsystem::DMA as u32)
    | (1 << Subsystem::MCU as u32)
    | (1 << Subsystem::FW as u32)
    | (1 << Subsystem::PCIE as u32)
    | (1 << Subsystem::USB as u32)
    | (1 << Subsystem::WIFI as u32)
    | (1 << Subsystem::INIT as u32)
    | (1 << Subsystem::IRQ as u32)
    // REG intentionally disabled by default
    | (1 << Subsystem::ERR as u32)
);

/// Check if a subsystem is enabled for tracing
#[inline]
pub fn is_enabled(subsys: Subsystem) -> bool {
    (TRACE_MASK.load(Ordering::Relaxed) & subsys.mask()) != 0
}

/// Enable tracing for a subsystem
pub fn enable(subsys: Subsystem) {
    TRACE_MASK.fetch_or(subsys.mask(), Ordering::Relaxed);
}

/// Disable tracing for a subsystem
pub fn disable(subsys: Subsystem) {
    TRACE_MASK.fetch_and(!subsys.mask(), Ordering::Relaxed);
}

/// Enable all subsystems
pub fn enable_all() {
    TRACE_MASK.store(0xFFFF_FFFF, Ordering::Relaxed);
}

/// Disable all subsystems
pub fn disable_all() {
    TRACE_MASK.store(0, Ordering::Relaxed);
}

/// Set the trace mask directly
pub fn set_mask(mask: u32) {
    TRACE_MASK.store(mask, Ordering::Relaxed);
}

/// Get the current trace mask
pub fn get_mask() -> u32 {
    TRACE_MASK.load(Ordering::Relaxed)
}

/// Trace macro - prints with subsystem prefix if enabled
///
/// # Examples
///
/// ```rust
/// trace!(DMA, "ring {} initialized", 0);
/// trace!(MCU, "command sent: 0x{:02x}", cmd);
/// trace!(ERR, "failed to allocate buffer");
/// ```
#[macro_export]
macro_rules! trace {
    ($subsys:ident, $($arg:tt)*) => {{
        if $crate::trace::is_enabled($crate::trace::Subsystem::$subsys) {
            $crate::println!("[{}] {}", $crate::trace::Subsystem::$subsys.name(), format_args!($($arg)*));
        }
    }};
}

/// Check if a subsystem is enabled (for conditional expensive operations)
///
/// # Example
///
/// ```rust
/// if trace_enabled!(FW) {
///     let dump = expensive_firmware_dump();
///     trace!(FW, "{}", dump);
/// }
/// ```
#[macro_export]
macro_rules! trace_enabled {
    ($subsys:ident) => {
        $crate::trace::is_enabled($crate::trace::Subsystem::$subsys)
    };
}

/// Trace a register read (REG subsystem, very verbose)
#[macro_export]
macro_rules! trace_reg_read {
    ($offset:expr, $val:expr) => {{
        if $crate::trace::is_enabled($crate::trace::Subsystem::REG) {
            $crate::println!("[REG] R 0x{:08x} = 0x{:08x}", $offset, $val);
        }
    }};
}

/// Trace a register write (REG subsystem, very verbose)
#[macro_export]
macro_rules! trace_reg_write {
    ($offset:expr, $val:expr) => {{
        if $crate::trace::is_enabled($crate::trace::Subsystem::REG) {
            $crate::println!("[REG] W 0x{:08x} <- 0x{:08x}", $offset, $val);
        }
    }};
}

/// Trace an error condition (ERR subsystem, always important)
#[macro_export]
macro_rules! trace_err {
    ($($arg:tt)*) => {{
        // ERR always prints, regardless of mask
        $crate::println!("[ERR] {}", format_args!($($arg)*));
    }};
}
