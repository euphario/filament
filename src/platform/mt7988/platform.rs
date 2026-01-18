//! MT7988A Platform Implementation
//!
//! Implements the HAL Platform trait for the MediaTek MT7988A SoC
//! used in the Banana Pi BPI-R4 router board.

use crate::hal::{InterruptController, Timer, Uart, Platform, PlatformInfo};
use crate::arch::aarch64::mmu;
use super::{gic, timer, uart, wdt, irq, DRAM_BASE, DRAM_SIZE, PAGE_SIZE};

/// Static platform info for MT7988A
static PLATFORM_INFO: PlatformInfo = PlatformInfo {
    name: "MT7988A",
    dram_base: DRAM_BASE,
    dram_size: DRAM_SIZE,
    page_size: PAGE_SIZE,
    timer_irq: irq::TIMER_PPI,
    spurious_irq_threshold: irq::SPURIOUS_THRESHOLD,
};

/// MT7988A Platform implementation
pub struct Mt7988Platform {
    initialized: bool,
}

impl Mt7988Platform {
    pub const fn new() -> Self {
        Self { initialized: false }
    }
}

impl Platform for Mt7988Platform {
    fn info(&self) -> &'static PlatformInfo {
        &PLATFORM_INFO
    }

    fn interrupt_controller(&self) -> &dyn InterruptController {
        gic::as_interrupt_controller()
    }

    fn timer(&mut self) -> &mut dyn Timer {
        timer::as_timer()
    }

    fn console(&self) -> &dyn Uart {
        uart::as_uart()
    }

    fn early_init(&mut self) {
        if self.initialized {
            return;
        }

        // Initialize console first (already done before Platform::early_init is called)
        // uart::init();

        // Initialize watchdog (disable or configure)
        wdt::init();

        // Initialize GIC
        gic::init();

        self.initialized = true;
    }

    fn late_init(&mut self) {
        // Timer is initialized and started by the caller after scheduler is ready
        timer::init();
    }

    fn kick_watchdog(&self) {
        wdt::kick();
    }

    fn phys_to_virt(&self, phys: usize) -> usize {
        mmu::phys_to_virt(phys as u64) as usize
    }

    fn virt_to_phys(&self, virt: usize) -> usize {
        mmu::virt_to_phys(virt as u64) as usize
    }

    fn phys_to_dma(&self, phys: u64) -> u64 {
        // MT7988A PCIe uses identity mapping for inbound DMA
        // (no dma-ranges offset in device tree, confirmed by Linux driver)
        //
        // DRAM at CPU physical 0x40000000 is accessed at PCIe bus address 0x40000000
        //
        // If this changes (e.g., for a different MT7988 board configuration),
        // update this translation accordingly:
        //   phys - DRAM_BASE as u64 + PCIE_INBOUND_BASE
        phys
    }

    fn dma_to_phys(&self, dma: u64) -> u64 {
        // Identity mapping: DMA address == CPU physical address
        dma
    }
}

/// Global platform instance
static mut PLATFORM: Mt7988Platform = Mt7988Platform::new();

/// Get the global platform instance
pub fn platform() -> &'static mut dyn Platform {
    unsafe { &mut *core::ptr::addr_of_mut!(PLATFORM) }
}

/// Get the platform as the concrete type (for platform-specific operations)
pub fn mt7988_platform() -> &'static mut Mt7988Platform {
    unsafe { &mut *core::ptr::addr_of_mut!(PLATFORM) }
}
