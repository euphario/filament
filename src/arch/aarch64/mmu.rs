//! AArch64 MMU (Memory Management Unit) driver
//!
//! Implements 4-level page tables with 4KB pages for kernel/user separation.
//!
//! Address Space Layout:
//! - TTBR0_EL1: User space (0x0000_0000_0000_0000 - 0x0000_FFFF_FFFF_FFFF)
//! - TTBR1_EL1: Kernel space (0xFFFF_0000_0000_0000 - 0xFFFF_FFFF_FFFF_FFFF)
//!
//! Virtual Address Layout (48-bit):
//! [63:48] - Sign extension (must match bit 47)
//! [47:39] - L0 index (9 bits, 512 entries)
//! [38:30] - L1 index (9 bits, 512 entries)
//! [29:21] - L2 index (9 bits, 512 entries)
//! [20:12] - L3 index (9 bits, 512 entries)
//! [11:0]  - Page offset (12 bits, 4KB)

#![allow(dead_code)]  // Many items are infrastructure for future use

use crate::logln;

/// Page size (4KB)
pub const PAGE_SIZE: usize = 4096;

/// Number of entries per table (512 for 4KB pages)
const ENTRIES_PER_TABLE: usize = 512;

/// Kernel virtual base address (upper half)
/// Maps physical 0x0 to virtual 0xFFFF_0000_0000_0000
pub const KERNEL_VIRT_BASE: u64 = 0xFFFF_0000_0000_0000;

/// Physical to kernel virtual address conversion
#[inline]
pub fn phys_to_virt(phys: u64) -> u64 {
    phys | KERNEL_VIRT_BASE
}

/// Kernel virtual to physical address conversion
#[inline]
pub fn virt_to_phys(virt: u64) -> u64 {
    virt & !KERNEL_VIRT_BASE
}

/// Page table entry flags
pub mod flags {
    pub const VALID: u64 = 1 << 0;
    pub const TABLE: u64 = 1 << 1;  // For L0-L2: points to next level table
    pub const PAGE: u64 = 1 << 1;   // For L3: this is a page entry
    pub const AF: u64 = 1 << 10;    // Access flag (must be 1)
    pub const SH_NONE: u64 = 0 << 8;   // Non-shareable
    pub const SH_OUTER: u64 = 2 << 8;  // Outer shareable (for DMA with bus masters)
    pub const SH_INNER: u64 = 3 << 8;  // Inner shareable
    pub const AP_RW_EL1: u64 = 0 << 6;  // Read-write at EL1 only
    pub const AP_RW_ALL: u64 = 1 << 6;  // Read-write at all ELs
    pub const AP_RO_EL1: u64 = 2 << 6;  // Read-only at EL1 only
    pub const AP_RO_ALL: u64 = 3 << 6;  // Read-only at all ELs
    pub const UXN: u64 = 1 << 54;   // Unprivileged execute never
    pub const PXN: u64 = 1 << 53;   // Privileged execute never
    pub const NS: u64 = 1 << 5;     // Non-secure (for block/page)
}

/// Memory attribute indices (for MAIR_EL1)
pub mod attr {
    pub const DEVICE: u64 = 0 << 2;     // Attr index 0: Device-nGnRnE
    pub const NORMAL: u64 = 1 << 2;     // Attr index 1: Normal memory (cacheable)
    pub const NORMAL_NC: u64 = 2 << 2;  // Attr index 2: Normal Non-Cacheable (for DMA buffers)
}

/// MAIR_EL1 value
/// Attr0: Device-nGnRnE (0x00) - for MMIO registers
/// Attr1: Normal, Write-Back (0xFF) - for regular memory
/// Attr2: Normal, Non-Cacheable (0x44) - for DMA buffers
const MAIR_VALUE: u64 = 0x00_00_00_00_00_44_FF_00;

/// TCR_EL1 configuration
/// - T0SZ = 16 (48-bit VA for TTBR0)
/// - T1SZ = 16 (48-bit VA for TTBR1)
/// - TG0 = 0 (4KB granule for TTBR0)
/// - TG1 = 2 (4KB granule for TTBR1)
/// - IPS = 5 (48-bit PA)
/// - IRGN0/ORGN0 = Write-Back (for TTBR0 page table walks)
/// - SH0 = Inner Shareable (for TTBR0 page table walks)
const TCR_VALUE: u64 = (16 << 0)       // T0SZ = 16 (48-bit VA)
                     | (16 << 16)      // T1SZ = 16 (48-bit VA)
                     | (0b00 << 14)    // TG0 = 4KB
                     | (0b10 << 30)    // TG1 = 4KB
                     | (0b101 << 32)   // IPS = 48-bit PA
                     | (0b01 << 8)     // IRGN0 = Write-Back Write-Allocate
                     | (0b01 << 10)    // ORGN0 = Write-Back Write-Allocate
                     | (0b11 << 12);   // SH0 = Inner Shareable

/// Page table (512 entries, 4KB aligned)
#[repr(C, align(4096))]
pub struct PageTable {
    pub entries: [u64; ENTRIES_PER_TABLE],
}

impl PageTable {
    pub const fn new() -> Self {
        Self {
            entries: [0; ENTRIES_PER_TABLE],
        }
    }

    /// Clear all entries
    pub fn clear(&mut self) {
        for entry in self.entries.iter_mut() {
            *entry = 0;
        }
    }

    /// Set an entry pointing to next level table
    pub fn set_table(&mut self, index: usize, table_addr: u64) {
        self.entries[index] = table_addr | flags::VALID | flags::TABLE;
    }

    /// Set a block entry (1GB at L1, 2MB at L2)
    pub fn set_block(&mut self, index: usize, phys_addr: u64, is_device: bool) {
        let attr = if is_device { attr::DEVICE } else { attr::NORMAL };
        self.entries[index] = phys_addr
            | flags::VALID
            | flags::AF
            | flags::SH_INNER
            | attr;
        // Note: no TABLE flag = block descriptor
    }

    /// Set a block entry with full control over flags
    pub fn set_block_with_flags(&mut self, index: usize, phys_addr: u64, extra_flags: u64, attr_idx: u64) {
        self.entries[index] = phys_addr
            | flags::VALID
            | flags::AF
            | flags::SH_INNER
            | attr_idx
            | extra_flags;
    }

    /// Set a page entry (4KB at L3)
    pub fn set_page(&mut self, index: usize, phys_addr: u64, is_device: bool) {
        let attr = if is_device { attr::DEVICE } else { attr::NORMAL };
        self.entries[index] = phys_addr
            | flags::VALID
            | flags::PAGE
            | flags::AF
            | flags::SH_INNER
            | attr;
    }

    /// Set a page entry with user access
    pub fn set_user_page(&mut self, index: usize, phys_addr: u64, writable: bool, executable: bool) {
        let ap = if writable { flags::AP_RW_ALL } else { flags::AP_RO_ALL };
        let xn = if executable { 0 } else { flags::UXN };
        self.entries[index] = phys_addr
            | flags::VALID
            | flags::PAGE
            | flags::AF
            | flags::SH_INNER
            | attr::NORMAL
            | ap
            | xn
            | flags::PXN;  // Kernel can't execute user code
    }

    /// Get physical address of this table
    pub fn phys_addr(&self) -> u64 {
        self as *const _ as u64
    }
}

/// Static page tables for kernel (TTBR1)
#[repr(C, align(4096))]
struct KernelPageTables {
    l0: PageTable,
    l1: PageTable,
}

static mut KERNEL_TABLES: KernelPageTables = KernelPageTables {
    l0: PageTable::new(),
    l1: PageTable::new(),
};

/// Static page tables for identity mapping (TTBR0) - used during boot and as template
#[repr(C, align(4096))]
struct IdentityPageTables {
    l0: PageTable,
    l1: PageTable,
}

static mut IDENTITY_TABLES: IdentityPageTables = IdentityPageTables {
    l0: PageTable::new(),
    l1: PageTable::new(),
};

/// Initialize MMU with kernel/user separation
pub fn init() {
    unsafe {
        // Set up identity mapping (TTBR0) for user space / boot
        let id_tables = &mut *core::ptr::addr_of_mut!(IDENTITY_TABLES);
        let id_l1_addr = &id_tables.l1 as *const _ as u64;
        id_tables.l0.set_table(0, id_l1_addr);

        // Identity map: VA = PA for lower 4GB
        id_tables.l1.set_block(0, 0x00000000, true);  // Device (peripherals)
        id_tables.l1.set_block(1, 0x40000000, false); // Normal (DRAM)
        id_tables.l1.set_block(2, 0x80000000, false); // More DRAM
        id_tables.l1.set_block(3, 0xC0000000, false); // More DRAM

        // Set up kernel mapping (TTBR1) for upper half
        // 0xFFFF_0000_0000_0000 maps to 0x0000_0000_0000_0000
        let kern_tables = &mut *core::ptr::addr_of_mut!(KERNEL_TABLES);
        let kern_l1_addr = &kern_tables.l1 as *const _ as u64;
        kern_tables.l0.set_table(0, kern_l1_addr);

        // Kernel space mapping with UXN (unprivileged execute never)
        // Block 0: Device memory (UART, GIC)
        kern_tables.l1.set_block_with_flags(
            0,
            0x00000000,
            flags::UXN | flags::PXN,  // No execute from device memory
            attr::DEVICE
        );
        // Block 1: Kernel code/data (DRAM) - allow kernel execute
        kern_tables.l1.set_block_with_flags(
            1,
            0x40000000,
            flags::UXN,  // User can't execute, kernel can
            attr::NORMAL
        );
        // Block 2-3: More DRAM
        kern_tables.l1.set_block_with_flags(2, 0x80000000, flags::UXN, attr::NORMAL);
        kern_tables.l1.set_block_with_flags(3, 0xC0000000, flags::UXN, attr::NORMAL);
    }
}

/// Enable the MMU with separate kernel and user address spaces
pub fn enable() {
    unsafe {
        let id_tables = &*core::ptr::addr_of!(IDENTITY_TABLES);
        let kern_tables = &*core::ptr::addr_of!(KERNEL_TABLES);
        let ttbr0 = &id_tables.l0 as *const _ as u64;
        let ttbr1 = &kern_tables.l0 as *const _ as u64;

        // Set MAIR_EL1
        core::arch::asm!("msr mair_el1, {}", in(reg) MAIR_VALUE);

        // Set TCR_EL1
        core::arch::asm!("msr tcr_el1, {}", in(reg) TCR_VALUE);

        // Set TTBR0_EL1 (user space / identity mapping)
        core::arch::asm!("msr ttbr0_el1, {}", in(reg) ttbr0);

        // Save kernel TTBR0 for syscall handler to restore when entering kernel
        *core::ptr::addr_of_mut!(KERNEL_TTBR0) = ttbr0;

        // Set TTBR1_EL1 (kernel space)
        core::arch::asm!("msr ttbr1_el1, {}", in(reg) ttbr1);

        // Ensure all writes complete
        core::arch::asm!("isb");

        // Invalidate TLBs
        core::arch::asm!("tlbi vmalle1is");
        core::arch::asm!("dsb ish");
        core::arch::asm!("isb");

        // Enable MMU via SCTLR_EL1
        let mut sctlr: u64;
        core::arch::asm!("mrs {}, sctlr_el1", out(reg) sctlr);

        sctlr |= 1 << 0;   // M: Enable MMU
        sctlr |= 1 << 2;   // C: Enable data cache
        sctlr |= 1 << 12;  // I: Enable instruction cache
        sctlr |= 1 << 26;  // UCI: Allow cache maintenance (DC CVAC, DC CIVAC) from EL0

        core::arch::asm!("msr sctlr_el1, {}", in(reg) sctlr);
        core::arch::asm!("isb");
    }
}

/// Switch to a new user address space (change TTBR0)
/// # Safety
/// The page table must be valid and properly set up
pub unsafe fn switch_user_space(ttbr0_phys: u64) {
    core::arch::asm!(
        "msr ttbr0_el1, {}",
        "isb",
        "tlbi vmalle1is",  // Invalidate all TLB entries
        "dsb ish",
        "isb",
        in(reg) ttbr0_phys
    );
}

/// Kernel TTBR0 value (identity mapping) - accessible from assembly
/// This is set during MMU init and used by syscall handler to switch back to kernel space
#[no_mangle]
pub static mut KERNEL_TTBR0: u64 = 0;

/// Get current TTBR0 value
pub fn ttbr0() -> u64 {
    let val: u64;
    unsafe {
        core::arch::asm!("mrs {}, ttbr0_el1", out(reg) val);
    }
    val
}

/// Get current TTBR1 value
pub fn ttbr1() -> u64 {
    let val: u64;
    unsafe {
        core::arch::asm!("mrs {}, ttbr1_el1", out(reg) val);
    }
    val
}

/// Check if MMU is enabled
pub fn is_enabled() -> bool {
    let sctlr: u64;
    unsafe {
        core::arch::asm!("mrs {}, sctlr_el1", out(reg) sctlr);
    }
    (sctlr & 1) != 0
}

/// Print MMU info
pub fn print_info() {
    logln!("  TTBR0_EL1: 0x{:016x} (user/identity)", ttbr0());
    logln!("  TTBR1_EL1: 0x{:016x} (kernel)", ttbr1());
    logln!("  MMU:       {}", if is_enabled() { "enabled" } else { "disabled" });
}
