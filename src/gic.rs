//! ARM GICv3 (Generic Interrupt Controller) driver for MT7988A
//!
//! GIC base addresses for MT7988:
//! - GICD (Distributor):    0x0c000000
//! - GICR (Redistributor):  0x0c080000

use core::ptr::{read_volatile, write_volatile};

/// GIC Distributor base address
const GICD_BASE: usize = 0x0c000000;

/// GIC Redistributor base address (CPU 0)
const GICR_BASE: usize = 0x0c080000;

/// GICD Register offsets
mod gicd {
    pub const CTLR: usize = 0x0000;      // Distributor Control
    pub const TYPER: usize = 0x0004;     // Interrupt Controller Type
    pub const IIDR: usize = 0x0008;      // Implementer Identification
    pub const IGROUPR: usize = 0x0080;   // Interrupt Group (banked)
    pub const ISENABLER: usize = 0x0100; // Interrupt Set-Enable
    pub const ICENABLER: usize = 0x0180; // Interrupt Clear-Enable
    pub const ISPENDR: usize = 0x0200;   // Interrupt Set-Pending
    pub const ICPENDR: usize = 0x0280;   // Interrupt Clear-Pending
    pub const IPRIORITYR: usize = 0x0400; // Interrupt Priority
    pub const ITARGETSR: usize = 0x0800; // Interrupt Targets (GICv2 compat)
    pub const ICFGR: usize = 0x0c00;     // Interrupt Configuration
    pub const IROUTER: usize = 0x6000;   // Interrupt Routing (64-bit per SPI)
}

/// GICR Register offsets (Redistributor)
mod gicr {
    // RD_base (first 64KB)
    pub const CTLR: usize = 0x0000;
    pub const IIDR: usize = 0x0004;
    pub const TYPER: usize = 0x0008;
    pub const WAKER: usize = 0x0014;

    // SGI_base (second 64KB, offset 0x10000)
    pub const SGI_BASE: usize = 0x10000;
    pub const IGROUPR0: usize = SGI_BASE + 0x0080;
    pub const ISENABLER0: usize = SGI_BASE + 0x0100;
    pub const ICENABLER0: usize = SGI_BASE + 0x0180;
    pub const IPRIORITYR: usize = SGI_BASE + 0x0400;
    pub const ICFGR0: usize = SGI_BASE + 0x0c00;
    pub const ICFGR1: usize = SGI_BASE + 0x0c04;
}

/// GICD_CTLR bits
mod gicd_ctlr {
    pub const ENABLE_GRP0: u32 = 1 << 0;
    pub const ENABLE_GRP1_NS: u32 = 1 << 1;
    pub const ENABLE_GRP1_S: u32 = 1 << 2;
    pub const ARE_S: u32 = 1 << 4;
    pub const ARE_NS: u32 = 1 << 5;
    pub const DS: u32 = 1 << 6;
}

/// GICR_WAKER bits
mod gicr_waker {
    pub const PROCESSOR_SLEEP: u32 = 1 << 1;
    pub const CHILDREN_ASLEEP: u32 = 1 << 2;
}

pub struct Gic {
    gicd_base: usize,
    gicr_base: usize,
}

impl Gic {
    pub const fn new() -> Self {
        Self {
            gicd_base: GICD_BASE,
            gicr_base: GICR_BASE,
        }
    }

    /// Initialize the GIC
    pub fn init(&self) {
        // Initialize CPU interface (system registers)
        self.init_cpu_interface();

        // Wake up redistributor
        self.gicr_wake();

        // Initialize distributor
        self.gicd_init();

        // Configure redistributor for SGIs/PPIs
        self.gicr_init();
    }

    /// Initialize GIC CPU interface via system registers
    fn init_cpu_interface(&self) {
        unsafe {
            // Enable system register access (ICC_SRE_EL1)
            let mut sre: u64;
            core::arch::asm!("mrs {}, S3_0_C12_C12_5", out(reg) sre); // ICC_SRE_EL1
            sre |= 0x7; // Enable SRE, DFB, DIB
            core::arch::asm!("msr S3_0_C12_C12_5, {}", in(reg) sre);
            core::arch::asm!("isb");

            // Set priority mask to allow all priorities (ICC_PMR_EL1)
            core::arch::asm!("msr S3_0_C4_C6_0, {}", in(reg) 0xffu64);

            // Enable Group 1 interrupts (ICC_IGRPEN1_EL1)
            core::arch::asm!("msr S3_0_C12_C12_7, {}", in(reg) 1u64);

            core::arch::asm!("isb");
        }
    }

    /// Wake up the redistributor
    fn gicr_wake(&self) {
        unsafe {
            let waker = self.gicr_read(gicr::WAKER);

            // Clear ProcessorSleep bit
            self.gicr_write(gicr::WAKER, waker & !gicr_waker::PROCESSOR_SLEEP);

            // Wait for ChildrenAsleep to clear
            while (self.gicr_read(gicr::WAKER) & gicr_waker::CHILDREN_ASLEEP) != 0 {
                core::hint::spin_loop();
            }
        }
    }

    /// Initialize the distributor
    fn gicd_init(&self) {
        unsafe {
            // Disable distributor while configuring
            self.gicd_write(gicd::CTLR, 0);

            // Wait for RWP (Register Write Pending) to clear
            while (self.gicd_read(gicd::CTLR) & (1 << 31)) != 0 {
                core::hint::spin_loop();
            }

            // Read number of interrupt lines
            let typer = self.gicd_read(gicd::TYPER);
            let it_lines = ((typer & 0x1f) + 1) * 32;
            // Configure all SPIs (interrupts 32+) as Group 1, priority 0xa0
            for i in (32..it_lines).step_by(32) {
                let reg_idx = (i / 32) as usize;
                // Set all to Group 1 (non-secure)
                self.gicd_write(gicd::IGROUPR + reg_idx * 4, 0xffffffff);
            }

            // Set default priority for SPIs
            for i in (32..it_lines).step_by(4) {
                let reg_idx = (i / 4) as usize;
                self.gicd_write(gicd::IPRIORITYR + reg_idx * 4, 0xa0a0a0a0);
            }

            // Enable distributor with affinity routing
            // DS (bit 6) = Disable Security - makes all interrupts non-secure
            let ctlr = gicd_ctlr::ENABLE_GRP1_NS
                | gicd_ctlr::ENABLE_GRP1_S
                | gicd_ctlr::ARE_S
                | gicd_ctlr::ARE_NS
                | gicd_ctlr::DS;  // Disable security to allow Group configuration
            self.gicd_write(gicd::CTLR, ctlr);
        }
    }

    /// Initialize the redistributor for SGIs/PPIs
    fn gicr_init(&self) {
        unsafe {
            // Configure SGIs (0-15) and PPIs (16-31) as Group 1
            self.gicr_write(gicr::IGROUPR0, 0xffffffff);

            // Set default priority
            for i in 0..8 {
                self.gicr_write(gicr::IPRIORITYR + i * 4, 0xa0a0a0a0);
            }

            // Enable all SGIs (for software interrupts)
            self.gicr_write(gicr::ISENABLER0, 0x0000ffff);
        }
    }

    /// Enable a specific SPI (interrupt number 32+)
    pub fn enable_irq(&self, irq: u32) {
        if irq < 32 {
            // SGI/PPI - use redistributor
            unsafe {
                let bit = 1u32 << irq;
                self.gicr_write(gicr::ISENABLER0, bit);
            }
        } else {
            // SPI - use distributor
            unsafe {
                // Configure routing to CPU 0 (IROUTER with affinity = 0)
                let irouter_offset = gicd::IROUTER + ((irq - 32) as usize) * 8;
                self.gicd_write64(irouter_offset, 0);

                // Enable the interrupt
                let reg_idx = (irq / 32) as usize;
                let bit = 1u32 << (irq % 32);
                self.gicd_write(gicd::ISENABLER + reg_idx * 4, bit);
            }
        }
    }

    /// Disable a specific interrupt
    pub fn disable_irq(&self, irq: u32) {
        if irq < 32 {
            unsafe {
                let bit = 1u32 << irq;
                self.gicr_write(gicr::ICENABLER0, bit);
            }
        } else {
            unsafe {
                let reg_idx = (irq / 32) as usize;
                let bit = 1u32 << (irq % 32);
                self.gicd_write(gicd::ICENABLER + reg_idx * 4, bit);
            }
        }
    }

    /// Acknowledge an interrupt (read IAR)
    pub fn ack_irq(&self) -> u32 {
        let irq: u64;
        unsafe {
            core::arch::asm!("mrs {}, S3_0_C12_C12_0", out(reg) irq); // ICC_IAR1_EL1
        }
        irq as u32
    }

    /// Signal end of interrupt
    pub fn eoi(&self, irq: u32) {
        unsafe {
            core::arch::asm!("msr S3_0_C12_C12_1, {}", in(reg) irq as u64); // ICC_EOIR1_EL1
        }
    }

    /// Get GIC version info
    pub fn version(&self) -> (u32, u32) {
        unsafe {
            let iidr = self.gicd_read(gicd::IIDR);
            let product = (iidr >> 24) & 0xff;
            let variant = (iidr >> 16) & 0xf;
            (product, variant)
        }
    }

    /// Get number of supported IRQ lines
    pub fn num_irqs(&self) -> u32 {
        unsafe {
            let typer = self.gicd_read(gicd::TYPER);
            ((typer & 0x1f) + 1) * 32
        }
    }

    #[inline]
    unsafe fn gicd_read(&self, offset: usize) -> u32 {
        read_volatile((self.gicd_base + offset) as *const u32)
    }

    #[inline]
    unsafe fn gicd_write(&self, offset: usize, value: u32) {
        write_volatile((self.gicd_base + offset) as *mut u32, value);
    }

    #[inline]
    unsafe fn gicd_write64(&self, offset: usize, value: u64) {
        write_volatile((self.gicd_base + offset) as *mut u64, value);
    }

    #[inline]
    unsafe fn gicr_read(&self, offset: usize) -> u32 {
        read_volatile((self.gicr_base + offset) as *const u32)
    }

    #[inline]
    unsafe fn gicr_write(&self, offset: usize, value: u32) {
        write_volatile((self.gicr_base + offset) as *mut u32, value);
    }
}

/// Global GIC instance
static mut GIC: Gic = Gic::new();

/// Initialize the global GIC
pub fn init() {
    unsafe {
        (*core::ptr::addr_of_mut!(GIC)).init();
    }
}

/// Get GIC info
pub fn info() -> (u32, u32, u32) {
    unsafe {
        let gic = &*core::ptr::addr_of!(GIC);
        let (product, variant) = gic.version();
        let num_irqs = gic.num_irqs();
        (product, variant, num_irqs)
    }
}

/// Enable an IRQ
pub fn enable_irq(irq: u32) {
    unsafe {
        (*core::ptr::addr_of_mut!(GIC)).enable_irq(irq);
    }
}

/// Disable an IRQ
pub fn disable_irq(irq: u32) {
    unsafe {
        (*core::ptr::addr_of_mut!(GIC)).disable_irq(irq);
    }
}

/// Acknowledge IRQ
pub fn ack_irq() -> u32 {
    unsafe { (*core::ptr::addr_of!(GIC)).ack_irq() }
}

/// End of interrupt
pub fn eoi(irq: u32) {
    unsafe {
        (*core::ptr::addr_of!(GIC)).eoi(irq);
    }
}

/// Debug: dump GIC state for an IRQ
pub fn debug_irq(irq: u32) {
    unsafe {
        let gic = &*core::ptr::addr_of!(GIC);

        if irq >= 32 {
            let reg_idx = (irq / 32) as usize;
            let bit_pos = irq % 32;

            // Read ISENABLER (is enabled?)
            let isenabler = gic.gicd_read(gicd::ISENABLER + reg_idx * 4);
            let enabled = (isenabler >> bit_pos) & 1;

            // Read ISPENDR (is pending at GIC?)
            let ispendr = gic.gicd_read(gicd::ISPENDR + reg_idx * 4);
            let pending = (ispendr >> bit_pos) & 1;

            // Read IGROUPR
            let igroupr = gic.gicd_read(gicd::IGROUPR + reg_idx * 4);
            let group = (igroupr >> bit_pos) & 1;

            // Read IROUTER
            let irouter_offset = gicd::IROUTER + ((irq - 32) as usize) * 8;
            let irouter_lo = gic.gicd_read(irouter_offset);
            let irouter_hi = gic.gicd_read(irouter_offset + 4);

            crate::println!("[GIC] IRQ {} state:", irq);
            crate::println!("  Enabled: {}", enabled);
            crate::println!("  Pending: {}", pending);
            crate::println!("  Group:   {} (1=NS)", group);
            crate::println!("  IROUTER: 0x{:08x}_{:08x}", irouter_hi, irouter_lo);

            // Read GICD_CTLR
            let ctlr = gic.gicd_read(gicd::CTLR);
            crate::println!("  GICD_CTLR: 0x{:08x}", ctlr);
        }
    }
}

/// Initialize GIC CPU interface for secondary CPUs
/// Called from secondary CPU entry after coming online
pub fn init_cpu() {
    unsafe {
        // Enable system register access (ICC_SRE_EL1)
        let mut sre: u64;
        core::arch::asm!("mrs {}, S3_0_C12_C12_5", out(reg) sre); // ICC_SRE_EL1
        sre |= 0x7; // Enable SRE, DFB, DIB
        core::arch::asm!("msr S3_0_C12_C12_5, {}", in(reg) sre);
        core::arch::asm!("isb");

        // Set priority mask to allow all priorities (ICC_PMR_EL1)
        let pmr: u64 = 0xFF;
        core::arch::asm!("msr S3_0_C4_C6_0, {}", in(reg) pmr);

        // Enable Group 1 interrupts (ICC_IGRPEN1_EL1)
        let igrpen: u64 = 1;
        core::arch::asm!("msr S3_0_C12_C12_7, {}", in(reg) igrpen);

        core::arch::asm!("isb");
    }
}
