//! Virtual Memory Object System
//!
//! This module provides the foundational VM abstractions:
//!
//! - [`PageFrame`]: Physical page reference (no refcounting, ownership via VmObject)
//! - [`VmObject`]: Collection of page frames (anonymous, physical, device)
//! - [`VmMapping`]: Mapping of a VmObject slice into an address space
//!
//! # Design Principles
//!
//! 1. **Explicit state machines** - Every object has a lifecycle with defined states
//! 2. **Static allocation** - All structures use fixed-size arrays (no_std compatible)
//! 3. **Single ownership** - VmObjects own their pages, mappings reference objects by ID
//!
//! # Memory Layout
//!
//! - Global `VmObject` registry: fixed array of MAX_VM_OBJECTS
//! - Each `VmObject`: fixed array of MAX_PAGES_PER_OBJECT page entries
//! - Each `AddressSpace`: uses existing HeapMapping slots for VmMappings
//!
//! # State Machines
//!
//! ## VmObject States
//! - `Free` → `Active` (create)
//! - `Active` → `Dying` (destroy or last unmap)
//! - `Dying` → `Free` (after cleanup)
//!
//! ## VmMapping States
//! - `Mapped` → `Unmapped` (explicit unmap or process exit)

mod page_frame;
mod object;
mod mapping;

pub use page_frame::PageFrame;
pub use object::{VmObjectId, VmObjectKind, VmObjectState};

use crate::kernel::lock::SpinLock;
use object::VmObjectSlot;

/// Maximum number of VmObjects in the system
pub const MAX_VM_OBJECTS: usize = 256;

/// Global VmObject registry
static VM_OBJECTS: SpinLock<VmObjectRegistry> = SpinLock::new(crate::kernel::lock::lock_class::RESOURCE, VmObjectRegistry::new());

/// Registry for all VmObjects in the system
pub struct VmObjectRegistry {
    objects: [VmObjectSlot; MAX_VM_OBJECTS],
    next_id: u32,
}

impl VmObjectRegistry {
    const fn new() -> Self {
        const EMPTY: VmObjectSlot = VmObjectSlot::empty();
        Self {
            objects: [EMPTY; MAX_VM_OBJECTS],
            next_id: 1,
        }
    }

    /// Find a free slot
    fn find_free_slot(&self) -> Option<usize> {
        self.objects.iter().position(|s| s.is_free())
    }

    /// Get object by ID
    fn get(&self, id: VmObjectId) -> Option<&VmObjectSlot> {
        self.objects.iter().find(|s| s.id() == Some(id))
    }

    /// Get mutable object by ID
    fn get_mut(&mut self, id: VmObjectId) -> Option<&mut VmObjectSlot> {
        self.objects.iter_mut().find(|s| s.id() == Some(id))
    }
}

/// Access the global VmObject registry
pub fn with_vm_objects<F, R>(f: F) -> R
where
    F: FnOnce(&mut VmObjectRegistry) -> R,
{
    let mut registry = VM_OBJECTS.lock();
    f(&mut registry)
}

/// Create a new anonymous VmObject
///
/// Returns the object ID, or None if:
/// - No free slots in registry
/// - num_pages > MAX_PAGES_PER_OBJECT
/// - num_pages is 0
pub fn create_anonymous(num_pages: usize) -> Option<VmObjectId> {
    with_vm_objects(|registry| {
        let slot_idx = registry.find_free_slot()?;
        let id = VmObjectId(registry.next_id);
        registry.next_id = registry.next_id.wrapping_add(1);
        if registry.next_id == 0 {
            registry.next_id = 1; // Skip 0
        }

        registry.objects[slot_idx] = VmObjectSlot::new_anonymous(id, num_pages)?;
        Some(id)
    })
}

/// Create a VmObject backed by specific physical pages (for DMA)
pub fn create_physical(phys_addr: u64, num_pages: usize) -> Option<VmObjectId> {
    with_vm_objects(|registry| {
        let slot_idx = registry.find_free_slot()?;
        let id = VmObjectId(registry.next_id);
        registry.next_id = registry.next_id.wrapping_add(1);
        if registry.next_id == 0 {
            registry.next_id = 1;
        }

        registry.objects[slot_idx] = VmObjectSlot::new_physical(id, phys_addr, num_pages)?;
        Some(id)
    })
}

/// Create a VmObject for device MMIO
pub fn create_device(phys_addr: u64, num_pages: usize) -> Option<VmObjectId> {
    with_vm_objects(|registry| {
        let slot_idx = registry.find_free_slot()?;
        let id = VmObjectId(registry.next_id);
        registry.next_id = registry.next_id.wrapping_add(1);
        if registry.next_id == 0 {
            registry.next_id = 1;
        }

        registry.objects[slot_idx] = VmObjectSlot::new_device(id, phys_addr, num_pages)?;
        Some(id)
    })
}

/// Get information about a VmObject
pub fn object_info(id: VmObjectId) -> Option<(VmObjectKind, VmObjectState, usize, u32)> {
    with_vm_objects(|registry| {
        let slot = registry.get(id)?;
        let obj = slot.object()?;
        Some((obj.kind(), obj.state(), obj.num_pages(), obj.mapping_count()))
    })
}

/// Add a mapping reference to a VmObject
///
/// Returns true if successful, false if object is not Active.
pub fn add_object_mapping(id: VmObjectId) -> bool {
    with_vm_objects(|registry| {
        if let Some(slot) = registry.get_mut(id) {
            if let Some(obj) = slot.object_mut() {
                return obj.add_mapping();
            }
        }
        false
    })
}

/// Remove a mapping reference from a VmObject
///
/// Returns true if object transitioned to Dead (caller should clean up).
pub fn remove_object_mapping(id: VmObjectId) -> bool {
    with_vm_objects(|registry| {
        if let Some(slot) = registry.get_mut(id) {
            if let Some(obj) = slot.object_mut() {
                let became_dead = obj.remove_mapping();
                if obj.state() == VmObjectState::Dead {
                    // Clean up the slot
                    *slot = VmObjectSlot::empty();
                }
                return became_dead;
            }
        }
        false
    })
}

/// Destroy a VmObject
///
/// Marks as Dying if mappings exist, otherwise immediately frees.
/// Returns true if successful.
pub fn destroy_object(id: VmObjectId) -> bool {
    with_vm_objects(|registry| {
        if let Some(slot) = registry.get_mut(id) {
            if let Some(obj) = slot.object_mut() {
                let result = obj.begin_destroy();
                if obj.state() == VmObjectState::Dead {
                    *slot = VmObjectSlot::empty();
                }
                return result;
            }
        }
        false
    })
}

/// Get a page's physical address from a VmObject
///
/// For anonymous objects, allocates the page on demand.
/// Returns None if object not found, page index out of bounds, or allocation fails.
pub fn get_object_page_phys(id: VmObjectId, page_index: usize) -> Option<u64> {
    with_vm_objects(|registry| {
        if let Some(slot) = registry.get_mut(id) {
            if let Some(obj) = slot.object_mut() {
                return obj.get_page_phys(page_index);
            }
        }
        None
    })
}

/// Get a page's physical address without allocating
pub fn object_page_phys(id: VmObjectId, page_index: usize) -> Option<u64> {
    with_vm_objects(|registry| {
        if let Some(slot) = registry.get(id) {
            if let Some(obj) = slot.object() {
                return obj.page_phys(page_index);
            }
        }
        None
    })
}
