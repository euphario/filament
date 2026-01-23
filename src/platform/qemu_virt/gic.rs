//! ARM GICv3 driver for QEMU virt
//!
//! QEMU's virt machine uses GICv3 at 0x08000000 (distributor)
//! and 0x080a0000 (redistributor).

use crate::arch::aarch64::mmio::MmioRegion;
use crate::hal::InterruptController;
use super::{GICD_BASE, GICR_BASE, irq};

/// GICD Register offsets
mod gicd {
    pub const CTLR: usize = 0x0000;
    pub const TYPER: usize = 0x0004;
    pub const IIDR: usize = 0x0008;
    pub const IGROUPR: usize = 0x0080;
    pub const ISENABLER: usize = 0x0100;
    pub const ICENABLER: usize = 0x0180;
    pub const ISPENDR: usize = 0x0200;
    pub const ICPENDR: usize = 0x0280;
    pub const IPRIORITYR: usize = 0x0400;
    pub const ITARGETSR: usize = 0x0800;
    pub const ICFGR: usize = 0x0c00;
    pub const IROUTER: usize = 0x6000;
}

/// GICR Register offsets
mod gicr {
    pub const CTLR: usize = 0x0000;
    pub const IIDR: usize = 0x0004;
    pub const TYPER: usize = 0x0008;
    pub const WAKER: usize = 0x0014;

    pub const SGI_BASE: usize = 0x10000;
    pub const IGROUPR0: usize = SGI_BASE + 0x0080;
    pub const ISENABLER0: usize = SGI_BASE + 0x0100;
    pub const ICENABLER0: usize = SGI_BASE + 0x0180;
    pub const IPRIORITYR: usize = SGI_BASE + 0x0400;
    pub const ICFGR0: usize = SGI_BASE + 0x0c00;
    pub const ICFGR1: usize = SGI_BASE + 0x0c04;
}

mod gicd_ctlr {
    pub const ENABLE_GRP0: u32 = 1 << 0;
    pub const ENABLE_GRP1_NS: u32 = 1 << 1;
    pub const ENABLE_GRP1_S: u32 = 1 << 2;
    pub const ARE_S: u32 = 1 << 4;
    pub const ARE_NS: u32 = 1 << 5;
    pub const DS: u32 = 1 << 6;
}

mod gicr_waker {
    pub const PROCESSOR_SLEEP: u32 = 1 << 1;
    pub const CHILDREN_ASLEEP: u32 = 1 << 2;
}

pub struct Gic {
    gicd: MmioRegion,
    gicr: MmioRegion,
}

impl Gic {
    pub const fn new() -> Self {
        Self {
            gicd: MmioRegion::new(GICD_BASE),
            gicr: MmioRegion::new(GICR_BASE),
        }
    }

    pub fn init(&self) {
        self.init_cpu_interface();
        self.gicr_wake();
        self.gicd_init();
        self.gicr_init();
    }

    fn init_cpu_interface(&self) {
        unsafe {
            // Enable system register access
            let mut sre: u64;
            core::arch::asm!("mrs {}, S3_0_C12_C12_5", out(reg) sre);
            sre |= 0x7;
            core::arch::asm!("msr S3_0_C12_C12_5, {}", in(reg) sre);
            core::arch::asm!("isb");

            // Set priority mask
            core::arch::asm!("msr S3_0_C4_C6_0, {}", in(reg) 0xffu64);

            // Enable Group 1 interrupts
            core::arch::asm!("msr S3_0_C12_C12_7, {}", in(reg) 1u64);
            core::arch::asm!("isb");
        }
    }

    fn gicr_wake(&self) {
        let waker = self.gicr_read(gicr::WAKER);
        self.gicr_write(gicr::WAKER, waker & !gicr_waker::PROCESSOR_SLEEP);

        let mut retries = 0;
        while (self.gicr_read(gicr::WAKER) & gicr_waker::CHILDREN_ASLEEP) != 0 {
            retries += 1;
            if retries >= 100_000 {
                // QEMU may not implement this properly, continue anyway
                break;
            }
            core::hint::spin_loop();
        }
    }

    fn gicd_init(&self) {
        // Disable distributor
        self.gicd_write(gicd::CTLR, 0);

        // Wait for RWP
        let mut retries = 0;
        while (self.gicd_read(gicd::CTLR) & (1 << 31)) != 0 {
            retries += 1;
            if retries >= 100_000 {
                break;
            }
            core::hint::spin_loop();
        }

        // Configure SPIs
        let typer = self.gicd_read(gicd::TYPER);
        let it_lines = ((typer & 0x1f) + 1) * 32;

        for i in (32..it_lines).step_by(32) {
            let reg_idx = (i / 32) as usize;
            self.gicd_write(gicd::IGROUPR + reg_idx * 4, 0xffffffff);
        }

        for i in (32..it_lines).step_by(4) {
            let reg_idx = (i / 4) as usize;
            self.gicd_write(gicd::IPRIORITYR + reg_idx * 4, 0xa0a0a0a0);
        }

        // Enable distributor
        let ctlr = gicd_ctlr::ENABLE_GRP1_NS
            | gicd_ctlr::ENABLE_GRP1_S
            | gicd_ctlr::ARE_S
            | gicd_ctlr::ARE_NS
            | gicd_ctlr::DS;
        self.gicd_write(gicd::CTLR, ctlr);
    }

    fn gicr_init(&self) {
        self.gicr_write(gicr::IGROUPR0, 0xffffffff);

        for i in 0..8 {
            self.gicr_write(gicr::IPRIORITYR + i * 4, 0xa0a0a0a0);
        }

        self.gicr_write(gicr::ISENABLER0, 0x0000ffff);
    }

    pub fn enable_irq(&self, irq: u32) {
        if irq < 32 {
            let bit = 1u32 << irq;
            self.gicr_write(gicr::ISENABLER0, bit);
        } else {
            let irouter_offset = gicd::IROUTER + ((irq - 32) as usize) * 8;
            self.gicd_write64(irouter_offset, 0);
            let reg_idx = (irq / 32) as usize;
            let bit = 1u32 << (irq % 32);
            self.gicd_write(gicd::ISENABLER + reg_idx * 4, bit);
        }
    }

    pub fn disable_irq(&self, irq: u32) {
        if irq < 32 {
            let bit = 1u32 << irq;
            self.gicr_write(gicr::ICENABLER0, bit);
        } else {
            let reg_idx = (irq / 32) as usize;
            let bit = 1u32 << (irq % 32);
            self.gicd_write(gicd::ICENABLER + reg_idx * 4, bit);
        }
    }

    pub fn ack_irq(&self) -> u32 {
        let irq: u64;
        unsafe {
            core::arch::asm!("mrs {}, S3_0_C12_C12_0", out(reg) irq);
        }
        irq as u32
    }

    pub fn eoi(&self, irq: u32) {
        unsafe {
            core::arch::asm!("msr S3_0_C12_C12_1, {}", in(reg) irq as u64);
        }
    }

    pub fn num_irqs(&self) -> u32 {
        let typer = self.gicd_read(gicd::TYPER);
        ((typer & 0x1f) + 1) * 32
    }

    #[inline]
    fn gicd_read(&self, offset: usize) -> u32 {
        self.gicd.read32(offset)
    }

    #[inline]
    fn gicd_write(&self, offset: usize, value: u32) {
        self.gicd.write32(offset, value);
    }

    #[inline]
    fn gicd_write64(&self, offset: usize, value: u64) {
        self.gicd.write64(offset, value);
    }

    #[inline]
    fn gicr_read(&self, offset: usize) -> u32 {
        self.gicr.read32(offset)
    }

    #[inline]
    fn gicr_write(&self, offset: usize, value: u32) {
        self.gicr.write32(offset, value);
    }
}

impl InterruptController for Gic {
    fn init(&self) {}

    fn init_cpu(&self) {
        unsafe {
            let mut sre: u64;
            core::arch::asm!("mrs {}, S3_0_C12_C12_5", out(reg) sre);
            sre |= 0x7;
            core::arch::asm!("msr S3_0_C12_C12_5, {}", in(reg) sre);
            core::arch::asm!("isb");

            core::arch::asm!("msr S3_0_C4_C6_0, {}", in(reg) 0xFFu64);
            core::arch::asm!("msr S3_0_C12_C12_7, {}", in(reg) 1u64);
            core::arch::asm!("isb");
        }
    }

    fn enable_irq(&self, irq: u32) {
        Gic::enable_irq(self, irq);
    }

    fn disable_irq(&self, irq: u32) {
        Gic::disable_irq(self, irq);
    }

    fn ack_irq(&self) -> u32 {
        Gic::ack_irq(self)
    }

    fn eoi(&self, irq: u32) {
        Gic::eoi(self, irq);
    }

    fn is_spurious(&self, irq: u32) -> bool {
        irq >= irq::SPURIOUS_THRESHOLD
    }

    fn num_irqs(&self) -> u32 {
        Gic::num_irqs(self)
    }
}

// ============================================================================
// Global Instance and Public API
// ============================================================================

static mut GIC: Gic = Gic::new();

pub fn init() {
    unsafe {
        (*core::ptr::addr_of_mut!(GIC)).init();
    }
}

pub fn info() -> (u32, u32, u32) {
    unsafe {
        let gic = &*core::ptr::addr_of!(GIC);
        let typer = gic.gicd_read(gicd::TYPER);
        let iidr = gic.gicd_read(gicd::IIDR);
        let product = (iidr >> 24) & 0xff;
        let variant = (iidr >> 16) & 0xf;
        let num_irqs = ((typer & 0x1f) + 1) * 32;
        (product, variant, num_irqs)
    }
}

pub fn enable_irq(irq: u32) {
    unsafe { (*core::ptr::addr_of_mut!(GIC)).enable_irq(irq); }
}

pub fn disable_irq(irq: u32) {
    unsafe { (*core::ptr::addr_of_mut!(GIC)).disable_irq(irq); }
}

pub fn ack_irq() -> u32 {
    unsafe { (*core::ptr::addr_of!(GIC)).ack_irq() }
}

pub fn eoi(irq: u32) {
    unsafe { (*core::ptr::addr_of!(GIC)).eoi(irq); }
}

pub fn init_cpu() {
    unsafe {
        let mut sre: u64;
        core::arch::asm!("mrs {}, S3_0_C12_C12_5", out(reg) sre);
        sre |= 0x7;
        core::arch::asm!("msr S3_0_C12_C12_5, {}", in(reg) sre);
        core::arch::asm!("isb");

        core::arch::asm!("msr S3_0_C4_C6_0, {}", in(reg) 0xFFu64);
        core::arch::asm!("msr S3_0_C12_C12_7, {}", in(reg) 1u64);
        core::arch::asm!("isb");
    }
}

pub fn as_interrupt_controller() -> &'static dyn InterruptController {
    unsafe { &*core::ptr::addr_of!(GIC) }
}

/// Debug helper to print IRQ state (no-op for QEMU)
pub fn debug_irq(_irq: u32) {
    // No-op for QEMU - minimal debug output
}
