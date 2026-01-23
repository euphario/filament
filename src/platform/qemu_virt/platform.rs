//! QEMU virt Platform Implementation
//!
//! Implements the HAL Platform trait for the QEMU virt machine (AArch64).
//! This enables testing the kernel without real hardware.

use crate::hal::{InterruptController, Timer, Uart, Platform, PlatformInfo};
use crate::arch::aarch64::mmu;
use super::{gic, timer, uart, irq, DRAM_BASE, DRAM_SIZE, PAGE_SIZE};

/// Static platform info for QEMU virt
static PLATFORM_INFO: PlatformInfo = PlatformInfo {
    name: "QEMU virt",
    dram_base: DRAM_BASE,
    dram_size: DRAM_SIZE,
    page_size: PAGE_SIZE,
    timer_irq: irq::TIMER_PPI,
    spurious_irq_threshold: irq::SPURIOUS_THRESHOLD,
};

/// QEMU virt Platform implementation
pub struct QemuVirtPlatform {
    initialized: bool,
}

impl QemuVirtPlatform {
    pub const fn new() -> Self {
        Self { initialized: false }
    }
}

impl Platform for QemuVirtPlatform {
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

        // Initialize GIC
        gic::init();

        self.initialized = true;
    }

    fn late_init(&mut self) {
        // Timer is initialized and started by the caller after scheduler is ready
        timer::init();
    }

    fn kick_watchdog(&self) {
        // QEMU virt doesn't have a watchdog
    }

    fn phys_to_virt(&self, phys: usize) -> usize {
        mmu::phys_to_virt(phys as u64) as usize
    }

    fn virt_to_phys(&self, virt: usize) -> usize {
        mmu::virt_to_phys(virt as u64) as usize
    }

    fn phys_to_dma(&self, phys: u64) -> u64 {
        // QEMU virt uses identity mapping for DMA
        phys
    }

    fn dma_to_phys(&self, dma: u64) -> u64 {
        // Identity mapping: DMA address == CPU physical address
        dma
    }
}

/// Global platform instance
static mut PLATFORM: QemuVirtPlatform = QemuVirtPlatform::new();

/// Get the global platform instance
pub fn platform() -> &'static mut dyn Platform {
    unsafe { &mut *core::ptr::addr_of_mut!(PLATFORM) }
}

/// Get the platform as the concrete type (for platform-specific operations)
pub fn qemu_virt_platform() -> &'static mut QemuVirtPlatform {
    unsafe { &mut *core::ptr::addr_of_mut!(PLATFORM) }
}
