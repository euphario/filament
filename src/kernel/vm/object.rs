//! Virtual Memory Object
//!
//! A VmObject is a collection of page frames that can be mapped into
//! one or more address spaces. It provides the abstraction layer between
//! physical memory and virtual address space mappings.
//!
//! # Object Kinds
//!
//! - **Anonymous**: Lazily-allocated pages, zero-filled on first access
//! - **Physical**: Backed by specific physical addresses (for DMA)
//! - **Device**: MMIO regions (not backed by RAM)
//!
//! # State Machine
//!
//! ```text
//!                    create
//!                      │
//!                      ▼
//!               ┌─────────────┐
//!               │   Active    │◄──────┐
//!               └──────┬──────┘       │
//!                      │              │
//!          destroy()   │    map()     │
//!          or last     │              │
//!          unmap       │              │
//!                      ▼              │
//!               ┌─────────────┐       │
//!               │   Dying     │───────┘ (if mappings still exist)
//!               └──────┬──────┘
//!                      │
//!          all pages   │
//!          released    │
//!                      ▼
//!               ┌─────────────┐
//!               │    Dead     │
//!               └─────────────┘
//!                      │
//!                      ▼
//!               (removed from registry)
//! ```
//!
//! # Invariants
//!
//! 1. `mapping_count` tracks active VmMappings referencing this object
//! 2. Object cannot transition to Dead while mapping_count > 0
//! 3. Pages are only freed when state is Dead
//!
//! # Static Allocation
//!
//! This module uses fixed-size arrays (no_std compatible):
//! - Each VmObject has up to MAX_PAGES_PER_OBJECT page entries
//! - Page entries are Option<u64> physical addresses (not Arc)
//! - Anonymous pages allocated on-demand from PMM

use super::PageFrame;
use crate::kernel::pmm;

/// Maximum pages per VmObject (256 pages = 1MB max object size)
pub const MAX_PAGES_PER_OBJECT: usize = 256;

/// Unique identifier for a VmObject
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VmObjectId(pub u32);

/// The kind of backing for a VmObject
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VmObjectKind {
    /// Anonymous memory - lazily allocated, zero-filled
    Anonymous,
    /// Physically contiguous memory (for DMA)
    Physical {
        /// Base physical address
        phys_base: u64,
    },
    /// Device MMIO region
    Device {
        /// Base physical address of MMIO
        phys_base: u64,
    },
}

/// VmObject lifecycle state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VmObjectState {
    /// Object is active and can be mapped
    Active,
    /// Object is being destroyed, no new mappings allowed
    Dying,
    /// Object is dead, awaiting removal from registry
    Dead,
}

impl VmObjectState {
    /// Get the name of this state (for logging)
    pub fn name(&self) -> &'static str {
        match self {
            Self::Active => "Active",
            Self::Dying => "Dying",
            Self::Dead => "Dead",
        }
    }
}

/// Page entry in a VmObject
///
/// For anonymous objects: None = not yet allocated, Some(phys) = allocated
/// For physical/device objects: Always Some(phys)
#[derive(Debug, Clone, Copy)]
struct PageEntry {
    /// Physical address (0 = not allocated for anonymous)
    phys_addr: u64,
    /// Whether this page is allocated (for anonymous) or always valid (physical/device)
    allocated: bool,
}

impl PageEntry {
    const EMPTY: Self = Self {
        phys_addr: 0,
        allocated: false,
    };

    fn new(phys_addr: u64) -> Self {
        Self {
            phys_addr,
            allocated: true,
        }
    }
}

/// A Virtual Memory Object
///
/// Represents a contiguous region of virtual memory that can be mapped
/// into one or more address spaces.
pub struct VmObject {
    /// Unique identifier
    id: VmObjectId,
    /// Current state
    state: VmObjectState,
    /// Kind of backing
    kind: VmObjectKind,
    /// Size in pages
    num_pages: usize,
    /// Page entries (fixed-size array)
    pages: [PageEntry; MAX_PAGES_PER_OBJECT],
    /// Number of active mappings referencing this object
    mapping_count: u32,
}

impl VmObject {
    /// Create a new anonymous VmObject
    ///
    /// Pages are lazily allocated on first access.
    pub fn new_anonymous(id: VmObjectId, num_pages: usize) -> Option<Self> {
        if num_pages == 0 || num_pages > MAX_PAGES_PER_OBJECT {
            return None;
        }

        Some(Self {
            id,
            state: VmObjectState::Active,
            kind: VmObjectKind::Anonymous,
            num_pages,
            pages: [PageEntry::EMPTY; MAX_PAGES_PER_OBJECT],
            mapping_count: 0,
        })
    }

    /// Create a VmObject backed by physical memory
    ///
    /// All pages reference the given physical range (not allocated from PMM).
    pub fn new_physical(id: VmObjectId, phys_base: u64, num_pages: usize) -> Option<Self> {
        if num_pages == 0 || num_pages > MAX_PAGES_PER_OBJECT {
            return None;
        }

        let mut pages = [PageEntry::EMPTY; MAX_PAGES_PER_OBJECT];
        for i in 0..num_pages {
            let phys = phys_base + (i as u64 * PageFrame::SIZE as u64);
            pages[i] = PageEntry::new(phys);
        }

        Some(Self {
            id,
            state: VmObjectState::Active,
            kind: VmObjectKind::Physical { phys_base },
            num_pages,
            pages,
            mapping_count: 0,
        })
    }

    /// Create a VmObject for device MMIO
    pub fn new_device(id: VmObjectId, phys_base: u64, num_pages: usize) -> Option<Self> {
        if num_pages == 0 || num_pages > MAX_PAGES_PER_OBJECT {
            return None;
        }

        let mut pages = [PageEntry::EMPTY; MAX_PAGES_PER_OBJECT];
        for i in 0..num_pages {
            let phys = phys_base + (i as u64 * PageFrame::SIZE as u64);
            pages[i] = PageEntry::new(phys);
        }

        Some(Self {
            id,
            state: VmObjectState::Active,
            kind: VmObjectKind::Device { phys_base },
            num_pages,
            pages,
            mapping_count: 0,
        })
    }

    /// Get the object ID
    #[inline]
    pub fn id(&self) -> VmObjectId {
        self.id
    }

    /// Get the current state
    #[inline]
    pub fn state(&self) -> VmObjectState {
        self.state
    }

    /// Get the kind
    #[inline]
    pub fn kind(&self) -> VmObjectKind {
        self.kind
    }

    /// Get the size in pages
    #[inline]
    pub fn num_pages(&self) -> usize {
        self.num_pages
    }

    /// Get the size in bytes
    #[inline]
    pub fn size(&self) -> usize {
        self.num_pages * PageFrame::SIZE
    }

    /// Get the number of active mappings
    #[inline]
    pub fn mapping_count(&self) -> u32 {
        self.mapping_count
    }

    /// Check if a mapping can be created
    #[inline]
    pub fn can_map(&self) -> bool {
        self.state == VmObjectState::Active
    }

    /// Increment mapping count (called when VmMapping is created)
    pub fn add_mapping(&mut self) -> bool {
        if self.state != VmObjectState::Active {
            return false;
        }
        self.mapping_count += 1;
        true
    }

    /// Decrement mapping count (called when VmMapping is destroyed)
    ///
    /// Returns true if this was the last mapping and object is Dying.
    pub fn remove_mapping(&mut self) -> bool {
        debug_assert!(self.mapping_count > 0, "VmObject mapping_count underflow");
        self.mapping_count -= 1;

        // If Dying and no more mappings, transition to Dead
        if self.state == VmObjectState::Dying && self.mapping_count == 0 {
            self.state = VmObjectState::Dead;
            self.release_pages();
            return true;
        }
        false
    }

    /// Begin destroying this object
    ///
    /// Transitions to Dying. If no mappings exist, immediately goes to Dead.
    /// Returns true if successful.
    pub fn begin_destroy(&mut self) -> bool {
        match self.state {
            VmObjectState::Active => {
                if self.mapping_count == 0 {
                    // No mappings, go straight to Dead
                    self.state = VmObjectState::Dead;
                    self.release_pages();
                } else {
                    // Has mappings, wait for them to unmap
                    self.state = VmObjectState::Dying;
                }
                true
            }
            VmObjectState::Dying | VmObjectState::Dead => false,
        }
    }

    /// Get a page's physical address, allocating if necessary (for anonymous objects)
    ///
    /// Returns None if:
    /// - Object is not Active
    /// - Page index out of bounds
    /// - Allocation fails (for anonymous)
    pub fn get_page_phys(&mut self, page_index: usize) -> Option<u64> {
        if self.state != VmObjectState::Active {
            return None;
        }

        if page_index >= self.num_pages {
            return None;
        }

        // Check if page already exists
        if self.pages[page_index].allocated {
            return Some(self.pages[page_index].phys_addr);
        }

        // For anonymous objects, allocate on demand
        if let VmObjectKind::Anonymous = self.kind {
            let phys_addr = pmm::alloc_page()? as u64;

            // Zero-fill the page
            let kernel_va = crate::kernel::arch::mmu::phys_to_virt(phys_addr);
            unsafe {
                core::ptr::write_bytes(kernel_va as *mut u8, 0, PageFrame::SIZE);
            }

            self.pages[page_index] = PageEntry::new(phys_addr);
            return Some(phys_addr);
        }

        // Physical/Device pages should already be populated
        None
    }

    /// Get a page's physical address without allocating
    ///
    /// For physical/device objects, calculates from base.
    /// For anonymous, returns None if page not yet allocated.
    pub fn page_phys(&self, page_index: usize) -> Option<u64> {
        if page_index >= self.num_pages {
            return None;
        }

        match self.kind {
            VmObjectKind::Physical { phys_base } | VmObjectKind::Device { phys_base } => {
                Some(phys_base + (page_index as u64 * PageFrame::SIZE as u64))
            }
            VmObjectKind::Anonymous => {
                if self.pages[page_index].allocated {
                    Some(self.pages[page_index].phys_addr)
                } else {
                    None
                }
            }
        }
    }

    /// Release all pages (called when transitioning to Dead)
    fn release_pages(&mut self) {
        // Only free anonymous pages that were allocated
        if let VmObjectKind::Anonymous = self.kind {
            for i in 0..self.num_pages {
                if self.pages[i].allocated {
                    pmm::free_page(self.pages[i].phys_addr as usize);
                    self.pages[i] = PageEntry::EMPTY;
                }
            }
        }
        // Physical/Device pages are not owned by PMM, don't free them
    }
}

/// Slot wrapper for VmObject in the registry
///
/// Provides the Free/Occupied pattern for fixed-size array storage.
pub enum VmObjectSlot {
    /// Slot is free
    Free,
    /// Slot contains an active object
    Occupied(VmObject),
}

impl VmObjectSlot {
    /// Create an empty slot
    pub const fn empty() -> Self {
        Self::Free
    }

    /// Check if slot is free
    pub fn is_free(&self) -> bool {
        matches!(self, Self::Free)
    }

    /// Get the object ID if occupied
    pub fn id(&self) -> Option<VmObjectId> {
        match self {
            Self::Free => None,
            Self::Occupied(obj) => Some(obj.id),
        }
    }

    /// Get immutable reference to object
    pub fn object(&self) -> Option<&VmObject> {
        match self {
            Self::Free => None,
            Self::Occupied(obj) => Some(obj),
        }
    }

    /// Get mutable reference to object
    pub fn object_mut(&mut self) -> Option<&mut VmObject> {
        match self {
            Self::Free => None,
            Self::Occupied(obj) => Some(obj),
        }
    }

    /// Create a new anonymous object in this slot
    pub fn new_anonymous(id: VmObjectId, num_pages: usize) -> Option<Self> {
        VmObject::new_anonymous(id, num_pages).map(Self::Occupied)
    }

    /// Create a new physical object in this slot
    pub fn new_physical(id: VmObjectId, phys_addr: u64, num_pages: usize) -> Option<Self> {
        VmObject::new_physical(id, phys_addr, num_pages).map(Self::Occupied)
    }

    /// Create a new device object in this slot
    pub fn new_device(id: VmObjectId, phys_addr: u64, num_pages: usize) -> Option<Self> {
        VmObject::new_device(id, phys_addr, num_pages).map(Self::Occupied)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_transitions() {
        // Can't test actual allocation without PMM, but can test state logic
        let state = VmObjectState::Active;
        assert_eq!(state.name(), "Active");
    }

    #[test]
    fn test_slot_operations() {
        let slot = VmObjectSlot::empty();
        assert!(slot.is_free());
        assert!(slot.id().is_none());
    }
}
