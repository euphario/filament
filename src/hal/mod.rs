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
//! |    HAL Traits     |  InterruptController, Timer, Platform
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
    fn ticks(&self) -> u64;

    /// Get the timer frequency in Hz
    fn frequency(&self) -> u64;

    /// Get the raw counter value (for high-resolution timing)
    fn counter(&self) -> u64;
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
