//! Platform Support for MediaTek MT7988A SoC
//!
//! This module contains all SoC-specific code for the MT7988A:
//! - Hardware addresses and IRQ numbers (constants below)
//! - GIC interrupt controller driver
//! - UART driver
//! - Timer driver
//! - Peripheral drivers (Ethernet, SD, I2C)
//!
//! All SoC-specific addresses, IRQ numbers, and hardware constants live here.
//! This is the ONE place to look for "what is the magic number for X?"

#![allow(dead_code)]  // Many constants are defined for completeness/documentation

pub mod gic;
pub mod uart;
pub mod timer;
pub mod eth;
pub mod sd;
pub mod i2c;
pub mod wdt;
pub mod platform;
pub mod bus;

// =============================================================================
// Platform Constants
// =============================================================================

// Re-export KERNEL_VIRT_BASE from arch layer (single source of truth)
pub use crate::arch::aarch64::mmu::KERNEL_VIRT_BASE;

// =============================================================================
// Memory Map
// =============================================================================

/// DRAM base address
pub const DRAM_BASE: usize = 0x4000_0000;

/// DRAM size (4GB for BPI-R4)
/// Note: Adjust this for different board variants
pub const DRAM_SIZE: usize = 0x1_0000_0000;  // 4GB

/// End of usable DRAM
pub const DRAM_END: usize = DRAM_BASE + DRAM_SIZE;

/// Default initrd load address (set by U-Boot)
pub const INITRD_ADDR: usize = 0x4800_0000;

/// Maximum initrd size
pub const INITRD_MAX_SIZE: usize = 16 * 1024 * 1024;

// =============================================================================
// GIC-600 (ARM CoreLink GIC-600)
// =============================================================================

/// GIC Distributor base address
pub const GICD_BASE: usize = 0x0c00_0000;

/// GIC Redistributor base address (CPU 0)
pub const GICR_BASE: usize = 0x0c08_0000;

/// GIC Redistributor stride (per CPU)
pub const GICR_STRIDE: usize = 0x2_0000;

// =============================================================================
// UART
// =============================================================================

/// UART0 (debug console) base address
pub const UART0_BASE: usize = 0x1100_0000;

/// UART1 base address
pub const UART1_BASE: usize = 0x1100_1000;

/// UART2 base address
pub const UART2_BASE: usize = 0x1100_2000;

// =============================================================================
// I2C Controllers
// =============================================================================

/// I2C0 base address
pub const I2C0_BASE: usize = 0x1100_3000;

/// I2C1 base address
pub const I2C1_BASE: usize = 0x1100_4000;

/// I2C2 base address
pub const I2C2_BASE: usize = 0x1100_5000;

// =============================================================================
// SD/eMMC (MSDC)
// =============================================================================

/// MSDC0 (eMMC) base address
pub const MSDC0_BASE: usize = 0x1123_0000;

/// MSDC1 (SD card) base address
pub const MSDC1_BASE: usize = 0x1124_0000;

// =============================================================================
// Ethernet (Frame Engine)
// =============================================================================

/// Frame Engine base address
pub const FE_BASE: usize = 0x1510_0000;

/// GMAC base address
pub const GMAC_BASE: usize = FE_BASE + 0x1_0000;

/// PDMA base address
pub const PDMA_BASE: usize = FE_BASE + 0x6000;

/// QDMA base address
pub const QDMA_BASE: usize = FE_BASE + 0x4400;

/// PPE base address
pub const PPE_BASE: usize = FE_BASE + 0x2000;

// =============================================================================
// USB (SSUSB/xHCI)
// =============================================================================

/// SSUSB0 base address (M.2 slot)
pub const SSUSB0_BASE: usize = 0x1119_0000;

/// SSUSB1 base address (USB-A ports via VL822 hub)
pub const SSUSB1_BASE: usize = 0x1120_0000;

/// SSUSB IP configuration base
pub const SSUSB0_IPPC_BASE: usize = 0x1119_3e00;
pub const SSUSB1_IPPC_BASE: usize = 0x1120_3e00;

// =============================================================================
// PCIe
// =============================================================================

/// PCIe0 controller base
pub const PCIE0_BASE: usize = 0x1128_0000;

/// PCIe1 controller base
pub const PCIE1_BASE: usize = 0x1129_0000;

/// PCIe2 controller base
pub const PCIE2_BASE: usize = 0x112a_0000;

// =============================================================================
// Watchdog / Reset
// =============================================================================

/// Top Reset Generation Unit (TOPRGU) base address
pub const TOPRGU_BASE: usize = 0x1001_C000;

// =============================================================================
// WiFi (MT7996)
// =============================================================================

/// WiFi base address
pub const WIFI_BASE: usize = 0x18000000;

// =============================================================================
// IRQ Numbers
// =============================================================================

pub mod irq {
    //! IRQ numbers for MT7988A
    //!
    //! GIC interrupt IDs:
    //! - SGI: 0-15 (Software Generated Interrupts)
    //! - PPI: 16-31 (Private Peripheral Interrupts)
    //! - SPI: 32+ (Shared Peripheral Interrupts)
    //!
    //! For SPIs, the GIC IRQ number = SPI number + 32

    /// Timer PPI (EL1 Physical Timer)
    pub const TIMER_PPI: u32 = 30;

    /// Spurious interrupt threshold (GIC returns >= 1020 for spurious)
    pub const SPURIOUS_THRESHOLD: u32 = 1020;

    /// SSUSB0 interrupt (SPI 173 -> IRQ 205)
    pub const SSUSB0: u32 = 205;

    /// SSUSB1 interrupt (SPI 172 -> IRQ 204)
    pub const SSUSB1: u32 = 204;

    /// MSDC0 (eMMC) interrupt
    pub const MSDC0: u32 = 129;

    /// MSDC1 (SD) interrupt
    pub const MSDC1: u32 = 135;

    /// UART0 interrupt (SPI 123 -> IRQ 155)
    pub const UART0: u32 = 155;

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
    /// System clock frequency (26 MHz reference)
    pub const SYS_CLK_HZ: u64 = 26_000_000;

    /// UART clock frequency
    pub const UART_CLK_HZ: u64 = 40_000_000;

    /// I2C source clock
    pub const I2C_CLK_HZ: u64 = 124_800_000;
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
// NOTE: phys_to_virt/virt_to_phys removed - use arch::aarch64::mmu instead

/// Check if an address is in the user virtual range (TTBR0)
#[inline]
pub const fn is_user_addr(addr: u64) -> bool {
    addr < 0x0001_0000_0000_0000
}
