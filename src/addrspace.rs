//! Address Space Manager
//!
//! Manages per-process virtual address spaces using the PMM for page table allocation.
//! Each process gets its own page tables loaded into TTBR0 during context switch.

use crate::mmu::{self, PageTable, PAGE_SIZE, flags, attr};
use crate::pmm;
use crate::println;

/// Kernel virtual address base for accessing physical memory via TTBR1
const KERNEL_VIRT_BASE: u64 = 0xFFFF_0000_0000_0000;

/// Convert a physical address to a kernel virtual address for access via TTBR1
#[inline(always)]
fn phys_to_virt(phys: u64) -> *mut u64 {
    (KERNEL_VIRT_BASE | phys) as *mut u64
}

/// Maximum number of L1 entries we support for user space
/// Entry 0 (0x00000000-0x3FFFFFFF) is typically reserved/device
/// Entry 1+ (0x40000000+) can be used for user programs
const MAX_USER_L1_ENTRIES: usize = 4;

/// User space start address (after device memory region)
pub const USER_SPACE_START: u64 = 0x0000_0000_4000_0000;

/// User space end address
pub const USER_SPACE_END: u64 = 0x0000_0001_0000_0000;

/// Represents a process's virtual address space
pub struct AddressSpace {
    /// Physical address of L0 page table (loaded into TTBR0)
    pub ttbr0: u64,
    /// Physical addresses of allocated page tables (for cleanup)
    page_tables: [u64; 16],  // L0 + L1 + some L2/L3
    num_tables: usize,
}

impl AddressSpace {
    /// Create a new empty address space
    pub fn new() -> Option<Self> {
        // Allocate L0 table
        let l0_phys = pmm::alloc_page()?;

        // Allocate L1 table
        let l1_phys = pmm::alloc_page()?;

        // Zero out the tables (must use TTBR1 mapping since MMU is enabled)
        unsafe {
            let l0_ptr = phys_to_virt(l0_phys as u64);
            let l1_ptr = phys_to_virt(l1_phys as u64);
            for i in 0..512 {
                core::ptr::write_volatile(l0_ptr.add(i), 0);
                core::ptr::write_volatile(l1_ptr.add(i), 0);
            }

            // Set up L0 -> L1 table entry
            // Entry 0: points to L1 table
            core::ptr::write_volatile(
                l0_ptr,
                l1_phys as u64 | flags::VALID | flags::TABLE
            );
        }

        // Note: We don't map device memory in user address spaces.
        // User processes access devices through syscalls (microkernel design).
        // L1 entries start empty, allowing map_page() to create L2/L3 tables.

        Some(Self {
            ttbr0: l0_phys as u64,
            page_tables: {
                let mut tables = [0u64; 16];
                tables[0] = l0_phys as u64;
                tables[1] = l1_phys as u64;
                tables
            },
            num_tables: 2,
        })
    }

    /// Map a region of physical memory into user space
    /// Returns true on success
    pub fn map_region(
        &mut self,
        virt_start: u64,
        phys_start: u64,
        size: usize,
        writable: bool,
        executable: bool,
    ) -> bool {
        // For now, only support 1GB block mappings at L1 level
        // This is a simplification - full implementation would use L2/L3 for fine-grained mapping

        if size != 0x40000000 {
            // Only 1GB blocks supported for now
            return false;
        }

        // Calculate L1 index
        let l1_index = ((virt_start >> 30) & 0x1FF) as usize;
        if l1_index >= MAX_USER_L1_ENTRIES {
            return false;
        }

        // Get L1 table address
        let l1_phys = self.page_tables[1];

        // Set up the block entry
        let ap = if writable { flags::AP_RW_ALL } else { flags::AP_RO_ALL };
        let xn = if executable { 0 } else { flags::UXN };

        unsafe {
            let l1_ptr = phys_to_virt(l1_phys);
            core::ptr::write_volatile(
                l1_ptr.add(l1_index),
                phys_start
                    | flags::VALID
                    | flags::AF
                    | flags::SH_INNER
                    | attr::NORMAL
                    | ap
                    | xn
                    | flags::PXN  // Kernel cannot execute user code
            );
        }

        true
    }

    /// Map a 4KB page at a specific virtual address
    /// This requires allocating L2 and L3 tables as needed
    pub fn map_page(
        &mut self,
        virt_addr: u64,
        phys_addr: u64,
        writable: bool,
        executable: bool,
    ) -> bool {
        // Extract table indices from virtual address
        let l1_index = ((virt_addr >> 30) & 0x1FF) as usize;
        let l2_index = ((virt_addr >> 21) & 0x1FF) as usize;
        let l3_index = ((virt_addr >> 12) & 0x1FF) as usize;

        // For simplicity, this implementation only supports a single L2/L3 chain
        // A full implementation would track multiple L2/L3 tables

        // Get or allocate L2 table
        let l1_phys = self.page_tables[1];
        let l2_phys = unsafe {
            let l1_ptr = phys_to_virt(l1_phys);
            let entry = core::ptr::read_volatile(l1_ptr.add(l1_index));

            if entry == 0 {
                // Need to allocate L2 table
                if self.num_tables >= 16 {
                    return false;  // Out of tracking space
                }
                let new_l2 = match pmm::alloc_page() {
                    Some(addr) => addr,
                    None => return false,
                };
                // Zero the new table
                let new_ptr = phys_to_virt(new_l2 as u64);
                for i in 0..512 {
                    core::ptr::write_volatile(new_ptr.add(i), 0);
                }
                // Set L1 entry to point to L2
                core::ptr::write_volatile(
                    l1_ptr.add(l1_index),
                    new_l2 as u64 | flags::VALID | flags::TABLE
                );
                self.page_tables[self.num_tables] = new_l2 as u64;
                self.num_tables += 1;
                new_l2 as u64
            } else if (entry & flags::TABLE) != 0 {
                // Already a table entry
                entry & 0x0000_FFFF_FFFF_F000
            } else {
                // It's a block entry, can't map pages here
                return false;
            }
        };

        // Get or allocate L3 table
        let l3_phys = unsafe {
            let l2_ptr = phys_to_virt(l2_phys);
            let entry = core::ptr::read_volatile(l2_ptr.add(l2_index));

            if entry == 0 {
                // Need to allocate L3 table
                if self.num_tables >= 16 {
                    return false;
                }
                let new_l3 = match pmm::alloc_page() {
                    Some(addr) => addr,
                    None => return false,
                };
                // Zero the new table
                let new_ptr = phys_to_virt(new_l3 as u64);
                for i in 0..512 {
                    core::ptr::write_volatile(new_ptr.add(i), 0);
                }
                // Set L2 entry to point to L3
                core::ptr::write_volatile(
                    l2_ptr.add(l2_index),
                    new_l3 as u64 | flags::VALID | flags::TABLE
                );
                self.page_tables[self.num_tables] = new_l3 as u64;
                self.num_tables += 1;
                new_l3 as u64
            } else if (entry & flags::TABLE) != 0 {
                entry & 0x0000_FFFF_FFFF_F000
            } else {
                return false;
            }
        };

        // Set the L3 page entry
        let ap = if writable { flags::AP_RW_ALL } else { flags::AP_RO_ALL };
        let xn = if executable { 0 } else { flags::UXN };

        unsafe {
            let l3_ptr = phys_to_virt(l3_phys);
            core::ptr::write_volatile(
                l3_ptr.add(l3_index),
                phys_addr
                    | flags::VALID
                    | flags::PAGE  // L3 entries use PAGE bit
                    | flags::AF
                    | flags::SH_INNER
                    | attr::NORMAL
                    | ap
                    | xn
                    | flags::PXN
            );
        }

        true
    }

    /// Map a 4KB device page at a specific virtual address
    /// Similar to map_page but uses device memory attributes (non-cacheable)
    pub fn map_device_page(
        &mut self,
        virt_addr: u64,
        phys_addr: u64,
        writable: bool,
    ) -> bool {
        // Extract table indices from virtual address
        let l1_index = ((virt_addr >> 30) & 0x1FF) as usize;
        let l2_index = ((virt_addr >> 21) & 0x1FF) as usize;
        let l3_index = ((virt_addr >> 12) & 0x1FF) as usize;

        // Get or allocate L2 table
        let l1_phys = self.page_tables[1];
        let l2_phys = unsafe {
            let l1_ptr = phys_to_virt(l1_phys);
            let entry = core::ptr::read_volatile(l1_ptr.add(l1_index));

            if entry == 0 {
                // Need to allocate L2 table
                if self.num_tables >= 16 {
                    return false;
                }
                let new_l2 = match pmm::alloc_page() {
                    Some(addr) => addr,
                    None => return false,
                };
                let new_ptr = phys_to_virt(new_l2 as u64);
                for i in 0..512 {
                    core::ptr::write_volatile(new_ptr.add(i), 0);
                }
                core::ptr::write_volatile(
                    l1_ptr.add(l1_index),
                    new_l2 as u64 | flags::VALID | flags::TABLE
                );
                self.page_tables[self.num_tables] = new_l2 as u64;
                self.num_tables += 1;
                new_l2 as u64
            } else if (entry & flags::TABLE) != 0 {
                entry & 0x0000_FFFF_FFFF_F000
            } else {
                return false;
            }
        };

        // Get or allocate L3 table
        let l3_phys = unsafe {
            let l2_ptr = phys_to_virt(l2_phys);
            let entry = core::ptr::read_volatile(l2_ptr.add(l2_index));

            if entry == 0 {
                if self.num_tables >= 16 {
                    return false;
                }
                let new_l3 = match pmm::alloc_page() {
                    Some(addr) => addr,
                    None => return false,
                };
                let new_ptr = phys_to_virt(new_l3 as u64);
                for i in 0..512 {
                    core::ptr::write_volatile(new_ptr.add(i), 0);
                }
                core::ptr::write_volatile(
                    l2_ptr.add(l2_index),
                    new_l3 as u64 | flags::VALID | flags::TABLE
                );
                self.page_tables[self.num_tables] = new_l3 as u64;
                self.num_tables += 1;
                new_l3 as u64
            } else if (entry & flags::TABLE) != 0 {
                entry & 0x0000_FFFF_FFFF_F000
            } else {
                return false;
            }
        };

        // Set the L3 page entry with device memory attributes
        let ap = if writable { flags::AP_RW_ALL } else { flags::AP_RO_ALL };

        unsafe {
            let l3_ptr = phys_to_virt(l3_phys);
            core::ptr::write_volatile(
                l3_ptr.add(l3_index),
                phys_addr
                    | flags::VALID
                    | flags::PAGE  // L3 entries use PAGE bit
                    | flags::AF
                    | flags::SH_INNER
                    | attr::DEVICE  // Device memory - non-cacheable
                    | ap
                    | flags::UXN   // No user execute
                    | flags::PXN   // No kernel execute
            );
        }

        true
    }

    /// Map a 4KB DMA buffer page at a specific virtual address
    /// Uses Normal Non-Cacheable memory for proper DMA coherency
    pub fn map_dma_page(
        &mut self,
        virt_addr: u64,
        phys_addr: u64,
        writable: bool,
    ) -> bool {
        // Extract table indices from virtual address
        let l1_index = ((virt_addr >> 30) & 0x1FF) as usize;
        let l2_index = ((virt_addr >> 21) & 0x1FF) as usize;
        let l3_index = ((virt_addr >> 12) & 0x1FF) as usize;

        // Get or allocate L2 table
        let l1_phys = self.page_tables[1];
        let l2_phys = unsafe {
            let l1_ptr = phys_to_virt(l1_phys);
            let entry = core::ptr::read_volatile(l1_ptr.add(l1_index));

            if entry == 0 {
                if self.num_tables >= 16 {
                    return false;
                }
                let new_l2 = match pmm::alloc_page() {
                    Some(addr) => addr,
                    None => return false,
                };
                let new_ptr = phys_to_virt(new_l2 as u64);
                for i in 0..512 {
                    core::ptr::write_volatile(new_ptr.add(i), 0);
                }
                core::ptr::write_volatile(
                    l1_ptr.add(l1_index),
                    new_l2 as u64 | flags::VALID | flags::TABLE
                );
                self.page_tables[self.num_tables] = new_l2 as u64;
                self.num_tables += 1;
                new_l2 as u64
            } else if (entry & flags::TABLE) != 0 {
                entry & 0x0000_FFFF_FFFF_F000
            } else {
                return false;
            }
        };

        // Get or allocate L3 table
        let l3_phys = unsafe {
            let l2_ptr = phys_to_virt(l2_phys);
            let entry = core::ptr::read_volatile(l2_ptr.add(l2_index));

            if entry == 0 {
                if self.num_tables >= 16 {
                    return false;
                }
                let new_l3 = match pmm::alloc_page() {
                    Some(addr) => addr,
                    None => return false,
                };
                let new_ptr = phys_to_virt(new_l3 as u64);
                for i in 0..512 {
                    core::ptr::write_volatile(new_ptr.add(i), 0);
                }
                core::ptr::write_volatile(
                    l2_ptr.add(l2_index),
                    new_l3 as u64 | flags::VALID | flags::TABLE
                );
                self.page_tables[self.num_tables] = new_l3 as u64;
                self.num_tables += 1;
                new_l3 as u64
            } else if (entry & flags::TABLE) != 0 {
                entry & 0x0000_FFFF_FFFF_F000
            } else {
                return false;
            }
        };

        // Set the L3 page entry with Normal Cacheable memory for DMA
        // MT7988A requires cacheable memory + explicit cache operations.
        // Userspace must flush before DMA reads and invalidate after DMA writes.
        let ap = if writable { flags::AP_RW_ALL } else { flags::AP_RO_ALL };

        unsafe {
            let l3_ptr = phys_to_virt(l3_phys);
            core::ptr::write_volatile(
                l3_ptr.add(l3_index),
                phys_addr
                    | flags::VALID
                    | flags::PAGE
                    | flags::AF
                    | flags::SH_INNER   // Inner shareable for multi-core
                    | attr::NORMAL      // Normal cacheable - requires explicit cache ops
                    | ap
                    | flags::UXN
                    | flags::PXN
            );
        }

        true
    }

    /// Unmap a 4KB page at a specific virtual address
    /// Returns the physical address that was mapped, or None if not mapped
    pub fn unmap_page(&mut self, virt_addr: u64) -> Option<u64> {
        // Extract table indices from virtual address
        let l1_index = ((virt_addr >> 30) & 0x1FF) as usize;
        let l2_index = ((virt_addr >> 21) & 0x1FF) as usize;
        let l3_index = ((virt_addr >> 12) & 0x1FF) as usize;

        let l1_phys = self.page_tables[1];

        unsafe {
            // Check L1 entry
            let l1_ptr = phys_to_virt(l1_phys);
            let l1_entry = core::ptr::read_volatile(l1_ptr.add(l1_index));

            if (l1_entry & flags::VALID) == 0 {
                return None; // Not mapped
            }

            if (l1_entry & flags::TABLE) == 0 {
                // It's a 1GB block entry - we don't support unmapping these
                return None;
            }

            // Get L2 table
            let l2_phys = l1_entry & 0x0000_FFFF_FFFF_F000;
            let l2_ptr = phys_to_virt(l2_phys);
            let l2_entry = core::ptr::read_volatile(l2_ptr.add(l2_index));

            if (l2_entry & flags::VALID) == 0 {
                return None; // Not mapped
            }

            if (l2_entry & flags::TABLE) == 0 {
                // It's a 2MB block entry - we don't support unmapping these
                return None;
            }

            // Get L3 table
            let l3_phys = l2_entry & 0x0000_FFFF_FFFF_F000;
            let l3_ptr = phys_to_virt(l3_phys);
            let l3_entry = core::ptr::read_volatile(l3_ptr.add(l3_index));

            if (l3_entry & flags::VALID) == 0 {
                return None; // Not mapped
            }

            // Get the physical address before clearing
            let phys_addr = l3_entry & 0x0000_FFFF_FFFF_F000;

            // Clear the L3 entry (mark as invalid)
            core::ptr::write_volatile(l3_ptr.add(l3_index), 0);

            // Ensure the write is visible
            core::arch::asm!("dsb ishst");

            Some(phys_addr)
        }
    }

    /// Activate this address space (switch TTBR0)
    pub unsafe fn activate(&self) {
        mmu::switch_user_space(self.ttbr0);
    }

    /// Get the TTBR0 value for this address space
    pub fn get_ttbr0(&self) -> u64 {
        self.ttbr0
    }
}

impl Drop for AddressSpace {
    fn drop(&mut self) {
        // Free all allocated page tables
        for i in 0..self.num_tables {
            if self.page_tables[i] != 0 {
                pmm::free_page(self.page_tables[i] as usize);
            }
        }
    }
}

/// Test the address space manager
pub fn test() {
    println!("  Testing address space creation...");

    if let Some(mut addr_space) = AddressSpace::new() {
        println!("    Created address space, TTBR0: 0x{:016x}", addr_space.ttbr0);

        // Allocate a physical page for user code
        if let Some(user_page) = pmm::alloc_page() {
            println!("    Allocated user page at 0x{:08x}", user_page);

            // Map it into the address space at a user address
            let user_virt = 0x4000_0000u64;  // 1GB mark
            if addr_space.map_page(user_virt, user_page as u64, true, true) {
                println!("    Mapped 0x{:08x} -> 0x{:016x}", user_page, user_virt);
                println!("    [OK] Address space test passed");
            } else {
                println!("    [!!] Failed to map page");
            }

            pmm::free_page(user_page);
        } else {
            println!("    [!!] Failed to allocate user page");
        }

        // addr_space will be dropped here, freeing page tables
        println!("    Address space cleaned up");
    } else {
        println!("    [!!] Failed to create address space");
    }
}
