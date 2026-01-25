//! Platform Support for QEMU virt (AArch64)
//!
//! This module provides support for running in QEMU's generic ARM virtual machine.
//! The virt machine provides:
//! - PL011 UART at 0x09000000
//! - GICv3 interrupt controller
//! - ARM generic timer
//! - virtio devices (optional)
//!
//! This allows testing the kernel without real hardware.

#![allow(dead_code)]

pub mod uart;
pub mod gic;
pub mod timer;
pub mod platform;
pub mod bus;

// =============================================================================
// Memory Map (QEMU virt machine)
// =============================================================================

/// DRAM base address (QEMU virt default)
pub const DRAM_BASE: usize = 0x4000_0000;

/// DRAM size (512MB for testing - must match QEMU -m parameter)
pub const DRAM_SIZE: usize = 0x2000_0000;

/// End of usable DRAM
pub const DRAM_END: usize = DRAM_BASE + DRAM_SIZE;

/// Kernel physical load address (where QEMU places the kernel - 512KB after DRAM base)
/// Must match linker-qemu.ld KERNEL_PHYS_BASE
pub const KERNEL_PHYS_BASE: usize = 0x4008_0000;

// =============================================================================
// DMA Pool (must not overlap with kernel)
// =============================================================================

/// DMA pool base address (16MB into DRAM, well after kernel image ends)
/// Kernel loads at 0x40080000 and is ~1MB, so this gives plenty of margin
pub const DMA_POOL_BASE: u64 = 0x4100_0000;

/// DMA pool size (4MB for descriptor rings and TX buffers)
pub const DMA_POOL_SIZE: usize = 4 * 1024 * 1024;

/// High DMA pool base address (not used on QEMU, but define for compatibility)
pub const DMA_POOL_HIGH_BASE: u64 = 0x5000_0000;

/// High DMA pool size (16MB)
pub const DMA_POOL_HIGH_SIZE: usize = 16 * 1024 * 1024;

/// Initrd load address
pub const INITRD_ADDR: usize = 0x4800_0000;

/// Maximum initrd size
pub const INITRD_MAX_SIZE: usize = 16 * 1024 * 1024;

// =============================================================================
// GICv3 (QEMU virt machine)
// =============================================================================

/// GIC Distributor base address
pub const GICD_BASE: usize = 0x0800_0000;

/// GIC Redistributor base address (CPU 0)
pub const GICR_BASE: usize = 0x080a_0000;

/// GIC Redistributor stride (per CPU)
pub const GICR_STRIDE: usize = 0x2_0000;

// =============================================================================
// PL011 UART
// =============================================================================

/// PL011 UART base address
pub const UART0_BASE: usize = 0x0900_0000;

// =============================================================================
// Flash / ROM
// =============================================================================

/// Flash base (not used in our setup)
pub const FLASH_BASE: usize = 0x0000_0000;

// =============================================================================
// IRQ Numbers
// =============================================================================

pub mod irq {
    //! IRQ numbers for QEMU virt
    //!
    //! GIC interrupt IDs:
    //! - SGI: 0-15 (Software Generated Interrupts)
    //! - PPI: 16-31 (Private Peripheral Interrupts)
    //! - SPI: 32+ (Shared Peripheral Interrupts)

    /// Timer PPI (EL1 Physical Timer) - same on all ARM
    pub const TIMER_PPI: u32 = 30;

    /// Spurious interrupt threshold
    pub const SPURIOUS_THRESHOLD: u32 = 1020;

    /// UART0 interrupt (SPI 1 -> IRQ 33)
    pub const UART0: u32 = 33;

    /// Convert SPI number to GIC IRQ number
    #[inline]
    pub const fn spi_to_irq(spi: u32) -> u32 {
        spi + 32
    }
}

// =============================================================================
// Clock Frequencies
// =============================================================================

pub mod clk {
    /// QEMU uses a fixed timer frequency
    pub const SYS_CLK_HZ: u64 = 62_500_000;

    /// UART clock (not actually used for PL011 in QEMU)
    pub const UART_CLK_HZ: u64 = 24_000_000;
}

// =============================================================================
// Page/Memory Constants
// =============================================================================

/// Page size (4KB)
pub const PAGE_SIZE: usize = 4096;

/// Page shift (12 bits)
pub const PAGE_SHIFT: usize = 12;

// =============================================================================
// Helper Functions
// =============================================================================

/// Check if an address is in the user virtual range (TTBR0)
#[inline]
pub const fn is_user_addr(addr: u64) -> bool {
    addr < 0x0001_0000_0000_0000
}
