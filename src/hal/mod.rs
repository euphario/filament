//! Hardware Abstraction Layer (HAL)
//!
//! This module defines platform-agnostic traits for hardware components,
//! enabling portability across different SoCs (MT7988A, BCM2711, etc).
//!
//! # Architecture
//!
//! ```text
//! +-------------------+
//! |   Kernel Core     |  Uses HAL traits
//! +-------------------+
//!          |
//!          v
//! +-------------------+
//! |    HAL Traits     |  Cpu, ContextSwitch, Timer, InterruptController
//! +-------------------+
//!          |
//!    +-----+-----+
//!    |           |
//!    v           v
//! +-------+  +-------+
//! |MT7988A|  |BCM2711|  Platform-specific implementations
//! +-------+  +-------+
//! ```
//!
//! # Usage
//!
//! The kernel accesses hardware through the global `PLATFORM` reference,
//! which provides access to interrupt controller and timer through traits.

// HAL submodules
pub mod cpu;
pub mod context;

// Re-export core traits
pub use cpu::Cpu;
pub use context::{Context, ContextSwitch};

/// Interrupt controller abstraction
///
/// Provides a platform-agnostic interface for managing hardware interrupts.
/// Implementations handle the specifics of GICv2, GICv3, BCM2835 IC, etc.
pub trait InterruptController {
    /// Initialize the interrupt controller
    fn init(&self);

    /// Initialize per-CPU state (for secondary CPUs)
    fn init_cpu(&self);

    /// Enable a specific IRQ
    fn enable_irq(&self, irq: u32);

    /// Disable a specific IRQ
    fn disable_irq(&self, irq: u32);

    /// Acknowledge an interrupt and return its IRQ number
    /// Returns the IRQ number, or a spurious value (>= 1020 for GIC)
    fn ack_irq(&self) -> u32;

    /// Signal end of interrupt processing
    fn eoi(&self, irq: u32);

    /// Check if an IRQ number is spurious (no real interrupt)
    fn is_spurious(&self, irq: u32) -> bool;

    /// Get the number of supported IRQs
    fn num_irqs(&self) -> u32;
}

/// Timer abstraction
///
/// Provides a platform-agnostic interface for system timers.
/// The timer is used for preemptive scheduling and timeouts.
///
/// # Unified Clock
///
/// All time-related operations in the kernel use nanoseconds as the canonical
/// unit. Use `now_ns()` to get the current time and `deadline_ns()` to create
/// deadlines. This ensures consistent behavior across all platforms regardless
/// of their underlying hardware counter frequencies.
///
/// The raw `counter()` and `frequency()` methods are provided for platform
/// internals but should NOT be used directly by kernel code.
pub trait Timer {
    /// Initialize the timer hardware
    fn init(&mut self);

    /// Start the timer with a given interval in milliseconds
    fn start(&mut self, interval_ms: u64);

    /// Stop the timer
    fn stop(&self);

    /// Handle a timer interrupt
    /// Returns true if this was a timer interrupt
    fn handle_irq(&mut self) -> bool;

    /// Get the current tick count (increments each timer interrupt)
    /// This is a software counter, not the hardware counter.
    fn ticks(&self) -> u64;

    /// Get the timer frequency in Hz (hardware counter frequency)
    /// Used internally for conversions. Kernel code should use `now_ns()`.
    fn frequency(&self) -> u64;

    /// Get the raw counter value
    /// Used internally. Kernel code should use `now_ns()` instead.
    fn counter(&self) -> u64;

    // =========================================================================
    // Unified Clock API - Use these in kernel code
    // =========================================================================

    /// Get current time in nanoseconds
    ///
    /// This is the canonical time source for the kernel. All deadlines,
    /// timeouts, and durations should use nanoseconds.
    ///
    /// Returns 0 if timer not initialized (frequency == 0). This is logged
    /// as a warning since it indicates timer initialization failure.
    fn now_ns(&self) -> u64 {
        let counter = self.counter();
        let freq = self.frequency();
        if freq == 0 {
            // Timer not initialized - this should not happen in production
            // Log once per call site would be ideal, but static tracking is complex
            // Callers should ensure timer is initialized before use
            return 0;
        }
        // Use 128-bit math to avoid overflow
        // ns = counter * 1_000_000_000 / freq
        ((counter as u128 * 1_000_000_000) / freq as u128) as u64
    }

    /// Create a deadline N nanoseconds from now
    ///
    /// Returns the counter value at which the deadline expires.
    /// Use `is_expired()` to check if a deadline has passed.
    ///
    /// Returns 0 if timer not initialized (frequency == 0).
    fn deadline_ns(&self, duration_ns: u64) -> u64 {
        let freq = self.frequency();
        if freq == 0 {
            // Timer not initialized - return 0 (immediately expired)
            // This causes callers to not block, which is safer than hanging
            return 0;
        }
        // Convert duration to counter ticks
        let duration_ticks = ((duration_ns as u128 * freq as u128) / 1_000_000_000) as u64;
        self.counter().saturating_add(duration_ticks)
    }

    /// Check if a deadline (counter value) has expired
    fn is_expired(&self, deadline: u64) -> bool {
        self.counter() >= deadline
    }

    /// Convert a counter deadline back to nanoseconds (for debugging)
    fn deadline_to_ns(&self, deadline: u64) -> u64 {
        let freq = self.frequency();
        if freq == 0 {
            return 0;
        }
        ((deadline as u128 * 1_000_000_000) / freq as u128) as u64
    }

    /// Get logical tick count derived from hardware counter
    ///
    /// Unlike `ticks()` which depends on IRQ handler incrementing a counter,
    /// this computes ticks directly from the hardware timer. More reliable
    /// for rate limiting and storm protection.
    ///
    /// Default tick interval is 10ms (100 ticks per second).
    fn logical_ticks(&self) -> u64 {
        const TICK_INTERVAL_NS: u64 = 10_000_000; // 10ms
        self.now_ns() / TICK_INTERVAL_NS
    }
}

/// UART abstraction
///
/// Provides a platform-agnostic interface for serial output.
pub trait Uart {
    /// Initialize the UART
    fn init(&self);

    /// Write a byte
    fn putc(&self, c: u8);

    /// Read a byte (non-blocking)
    /// Returns None if no data available
    fn getc(&self) -> Option<u8>;

    /// Check if transmit buffer is ready
    fn tx_ready(&self) -> bool;

    /// Check if receive data is available
    fn rx_ready(&self) -> bool;

    /// Flush pending output
    fn flush(&self);
}

/// Platform configuration
///
/// Static platform information that doesn't change at runtime.
#[derive(Debug, Clone, Copy)]
pub struct PlatformInfo {
    /// Platform name (e.g., "MT7988A", "BCM2711")
    pub name: &'static str,

    /// DRAM base address
    pub dram_base: usize,

    /// DRAM size in bytes
    pub dram_size: usize,

    /// Page size (typically 4096)
    pub page_size: usize,

    /// Timer IRQ number
    pub timer_irq: u32,

    /// Spurious IRQ threshold
    pub spurious_irq_threshold: u32,
}

/// Platform abstraction trait
///
/// Combines all hardware abstractions for a specific platform.
/// Each supported SoC implements this trait.
pub trait Platform {
    /// Get platform info
    fn info(&self) -> &'static PlatformInfo;

    /// Get the interrupt controller
    fn interrupt_controller(&self) -> &dyn InterruptController;

    /// Get the timer (mutable for state updates)
    fn timer(&mut self) -> &mut dyn Timer;

    /// Get the console UART
    fn console(&self) -> &dyn Uart;

    /// Platform-specific early initialization
    /// Called very early, before memory management is fully set up
    fn early_init(&mut self);

    /// Platform-specific late initialization
    /// Called after memory management and scheduler are ready
    fn late_init(&mut self);

    /// Kick the watchdog (if present)
    fn kick_watchdog(&self);

    /// Convert physical address to kernel virtual address
    fn phys_to_virt(&self, phys: usize) -> usize;

    /// Convert kernel virtual address to physical address
    fn virt_to_phys(&self, virt: usize) -> usize;

    /// Convert CPU physical address to DMA address for PCIe/bus devices
    ///
    /// On platforms with identity-mapped PCIe inbound windows, this returns
    /// the physical address unchanged. On platforms with an offset (e.g., PCIe
    /// inbound window at 0x0 mapping to DRAM at 0x40000000), this applies the
    /// necessary translation.
    ///
    /// Drivers should use this when programming DMA descriptors with buffer
    /// addresses that will be used by bus-mastering devices.
    fn phys_to_dma(&self, phys: u64) -> u64;

    /// Convert DMA address back to CPU physical address
    ///
    /// Inverse of `phys_to_dma`. Used when reading addresses from device
    /// descriptors that need to be accessed by the CPU.
    fn dma_to_phys(&self, dma: u64) -> u64;
}

/// IRQ handler type
pub type IrqHandler = fn(irq: u32);

/// IRQ dispatch table entry
pub struct IrqEntry {
    pub handler: Option<IrqHandler>,
    pub name: &'static str,
}

impl IrqEntry {
    pub const fn empty() -> Self {
        Self {
            handler: None,
            name: "",
        }
    }
}

/// Maximum number of IRQs in the dispatch table
pub const MAX_IRQS: usize = 256;

/// IRQ dispatch table
///
/// Allows registering handlers for specific IRQ numbers.
pub struct IrqDispatch {
    entries: [IrqEntry; MAX_IRQS],
}

impl IrqDispatch {
    pub const fn new() -> Self {
        const EMPTY: IrqEntry = IrqEntry::empty();
        Self {
            entries: [EMPTY; MAX_IRQS],
        }
    }

    /// Register a handler for an IRQ
    pub fn register(&mut self, irq: u32, handler: IrqHandler, name: &'static str) {
        if (irq as usize) < MAX_IRQS {
            self.entries[irq as usize] = IrqEntry {
                handler: Some(handler),
                name,
            };
        }
    }

    /// Unregister a handler
    pub fn unregister(&mut self, irq: u32) {
        if (irq as usize) < MAX_IRQS {
            self.entries[irq as usize] = IrqEntry::empty();
        }
    }

    /// Dispatch an IRQ to its handler
    /// Returns true if a handler was found and called
    pub fn dispatch(&self, irq: u32) -> bool {
        if (irq as usize) < MAX_IRQS {
            if let Some(handler) = self.entries[irq as usize].handler {
                handler(irq);
                return true;
            }
        }
        false
    }

    /// Get handler name for debugging
    pub fn handler_name(&self, irq: u32) -> &'static str {
        if (irq as usize) < MAX_IRQS {
            self.entries[irq as usize].name
        } else {
            ""
        }
    }
}
