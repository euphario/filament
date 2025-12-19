//! xHCI Ring structures (Command, Transfer, Event rings)

use crate::trb::{Trb, trb_type};
use crate::transfer::{flush_cache_line, invalidate_cache_line, dsb, isb};

/// Ring size (number of TRBs) - must be power of 2 for easy wrap
pub const RING_SIZE: usize = 64;

/// Command/Transfer Ring structure
pub struct Ring {
    pub trbs: *mut Trb,         // Pointer to TRB array
    pub enqueue: usize,         // Enqueue index
    pub dequeue: usize,         // Dequeue index
    pub cycle: bool,            // Producer cycle state
    pub phys_addr: u64,         // Physical address for hardware
}

impl Ring {
    /// Initialize a ring at the given virtual and physical addresses
    pub fn init(virt: *mut Trb, phys: u64) -> Self {
        unsafe {
            // Zero out the ring
            for i in 0..RING_SIZE {
                *virt.add(i) = Trb::new();
            }

            // Set up link TRB at the end to wrap around
            let link = &mut *virt.add(RING_SIZE - 1);
            link.param = phys;  // Points back to start
            link.set_type(trb_type::LINK);
            link.control |= 1 << 1;  // Toggle Cycle bit (TC) - bit 1, NOT bit 5 (IOC)

            // Flush entire ring to memory after CPU writes (for DMA)
            for i in 0..RING_SIZE {
                flush_cache_line(virt.add(i) as u64);
            }
            dsb();
        }

        Self {
            trbs: virt,
            enqueue: 0,
            dequeue: 0,
            cycle: true,
            phys_addr: phys,
        }
    }

    /// Enqueue a TRB and return its physical address
    pub fn enqueue(&mut self, trb: &Trb) -> u64 {
        let idx = self.enqueue;
        unsafe {
            let dest = &mut *self.trbs.add(idx);

            dest.param = trb.param;
            dest.status = trb.status;
            // Set control with our cycle bit
            dest.control = (trb.control & !1) | (self.cycle as u32);

            // Flush TRB to memory after CPU writes (before hardware reads)
            flush_cache_line(dest as *const _ as u64);
            dsb();
        }

        let trb_phys = self.phys_addr + (idx * 16) as u64;

        // Advance enqueue pointer
        self.enqueue += 1;
        if self.enqueue >= RING_SIZE - 1 {
            // Hit the link TRB - toggle cycle and wrap
            unsafe {
                let link = &mut *self.trbs.add(RING_SIZE - 1);
                link.set_cycle(self.cycle);

                // Flush the link TRB too
                flush_cache_line(link as *const _ as u64);
                dsb();
            }
            self.cycle = !self.cycle;
            self.enqueue = 0;
        }

        trb_phys
    }
}

/// Event Ring Segment Table Entry
#[repr(C, align(64))]
#[derive(Clone, Copy, Default)]
pub struct ErstEntry {
    pub ring_base: u64,     // Physical address of event ring segment
    pub ring_size: u32,     // Number of TRBs in segment
    pub _reserved: u32,
}

/// Event Ring structure
pub struct EventRing {
    pub trbs: *mut Trb,         // Pointer to TRB array
    pub erst: *mut ErstEntry,   // Event Ring Segment Table
    pub dequeue: usize,         // Dequeue index
    pub cycle: bool,            // Consumer cycle state (starts as 1)
    pub trbs_phys: u64,         // Physical address of TRBs
    pub erst_phys: u64,         // Physical address of ERST
    pub usbsts_addr: u64,       // Virtual address of USBSTS for PCIe sync (non-coherent ARM fix)
}

impl EventRing {
    /// Initialize an event ring
    pub fn init(trbs_virt: *mut Trb, trbs_phys: u64,
            erst_virt: *mut ErstEntry, erst_phys: u64,
            usbsts_addr: u64) -> Self {
        unsafe {
            // Zero out the ring
            for i in 0..RING_SIZE {
                *trbs_virt.add(i) = Trb::new();
            }

            // Flush entire event ring to memory (for DMA)
            for i in 0..RING_SIZE {
                flush_cache_line(trbs_virt.add(i) as u64);
            }
            dsb();

            // Set up ERST with single segment
            let erst = &mut *erst_virt;
            erst.ring_base = trbs_phys;
            erst.ring_size = RING_SIZE as u32;
            erst._reserved = 0;

            // Flush ERST to memory
            flush_cache_line(erst_virt as u64);
            dsb();
        }

        Self {
            trbs: trbs_virt,
            erst: erst_virt,
            dequeue: 0,
            cycle: true,
            trbs_phys,
            erst_phys,
            usbsts_addr,
        }
    }

    /// Dequeue an event TRB (returns None if no event pending)
    /// Invalidate cache before CPU reads from DMA buffer
    pub fn dequeue(&mut self) -> Option<Trb> {
        unsafe {
            let target = self.dequeue;
            let ptr = self.trbs.add(target);

            // Invalidate cache before reading DMA data
            invalidate_cache_line(ptr as u64);
            dsb();
            isb();

            // Now read the TRB as a single struct
            let trb = core::ptr::read_volatile(ptr);

            let trb_cycle = (trb.control & 1) != 0;
            if trb_cycle == self.cycle {
                // Valid event found - advance dequeue pointer
                self.dequeue += 1;
                if self.dequeue >= RING_SIZE {
                    self.dequeue = 0;
                    self.cycle = !self.cycle;
                }

                return Some(trb);
            }

            None
        }
    }

    /// Debug: check what we see at current dequeue position
    pub fn debug_check(&self, print: &dyn Fn(&str), print_hex64: &dyn Fn(u64)) {
        unsafe {
            let target = self.dequeue;
            let ptr = self.trbs.add(target);
            let trb_addr = ptr as u64;
            let trb_phys = self.trbs_phys + (target * 16) as u64;

            // Step 1: volatile read without any invalidation
            let trb1 = core::ptr::read_volatile(ptr);
            print("        [");
            // Note: caller needs to handle number printing
            print_hex64(target as u64);
            print("] @v");
            print_hex64(trb_addr);
            print("/p");
            print_hex64(trb_phys);
            print(": raw=");
            print_hex64(trb1.get_type() as u64);

            // Step 2: Invalidate then read
            invalidate_cache_line(trb_addr);
            dsb();
            isb();
            let trb2 = core::ptr::read_volatile(ptr);
            print(", civac=");
            print_hex64(trb2.get_type() as u64);
            print("\n");
        }
    }

    /// Get current dequeue pointer for ERDP register
    pub fn erdp(&self) -> u64 {
        self.trbs_phys + (self.dequeue * 16) as u64
    }

    /// Debug: return current state (dequeue_idx, expected_cycle, trb_control_at_dequeue)
    pub fn debug_state(&self) -> (usize, bool, u32) {
        unsafe {
            let ptr = self.trbs.add(self.dequeue);
            invalidate_cache_line(ptr as u64);
            dsb();
            isb();
            let trb = core::ptr::read_volatile(ptr);
            (self.dequeue, self.cycle, trb.control)
        }
    }
}

/// Device Context Base Address Array
/// Slot 0 = scratchpad buffer array pointer
/// Slots 1-255 = device context pointers
#[repr(C, align(64))]
pub struct Dcbaa {
    pub entries: [u64; 256],
}

impl Dcbaa {
    pub const fn new() -> Self {
        Self { entries: [0; 256] }
    }
}
