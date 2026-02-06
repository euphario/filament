//! Virtual Memory Mappings
//!
//! A VmMapping represents a slice of a VmObject mapped into an address space
//! at a specific virtual address with specific permissions.
//!
//! # State Machine
//!
//! ```text
//!         create
//!           │
//!           ▼
//!    ┌─────────────┐
//!    │   Mapped    │
//!    └──────┬──────┘
//!           │
//!    unmap()│
//!           │
//!           ▼
//!    ┌─────────────┐
//!    │  Unmapped   │
//!    └─────────────┘
//!           │
//!           ▼
//!       (dropped)
//! ```
//!
//! # Invariants
//!
//! 1. A mapping holds a reference to its VmObject (via mapping_count)
//! 2. Page table entries are only valid while mapping is Mapped
//! 3. Unmapping clears page table entries and releases VmObject reference
//!
//! # Design (no_std)
//!
//! - References VmObject by ID, not Arc
//! - Access object via global registry (with_vm_objects)
//! - Fixed-size - fits in HeapMapping slots

use super::{VmObjectId, PageFrame};

/// Unique identifier for a VmMapping within an address space
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VmMappingId(pub u32);

/// Memory protection flags
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Protection(u8);

impl Protection {
    /// No access
    pub const NONE: Self = Self(0);
    /// Read permission
    pub const READ: Self = Self(1 << 0);
    /// Write permission
    pub const WRITE: Self = Self(1 << 1);
    /// Execute permission
    pub const EXEC: Self = Self(1 << 2);

    /// Read + Write
    pub const RW: Self = Self(Self::READ.0 | Self::WRITE.0);
    /// Read + Execute
    pub const RX: Self = Self(Self::READ.0 | Self::EXEC.0);
    /// Read + Write + Execute
    pub const RWX: Self = Self(Self::READ.0 | Self::WRITE.0 | Self::EXEC.0);

    /// Check if readable
    #[inline]
    pub fn is_readable(self) -> bool {
        (self.0 & Self::READ.0) != 0
    }

    /// Check if writable
    #[inline]
    pub fn is_writable(self) -> bool {
        (self.0 & Self::WRITE.0) != 0
    }

    /// Check if executable
    #[inline]
    pub fn is_executable(self) -> bool {
        (self.0 & Self::EXEC.0) != 0
    }

    /// Combine protections
    #[inline]
    pub fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Convert to page table flags for ARM64
    pub fn to_pte_flags(self) -> u64 {
        use crate::arch::aarch64::mmu::flags;

        let mut pte = flags::AF | flags::VALID | flags::PAGE;

        // AP[2:1] encoding for EL0 access:
        // 01 = RW at all ELs
        // 11 = RO at all ELs
        if self.is_writable() {
            pte |= flags::AP_RW_ALL;
        } else if self.is_readable() {
            pte |= flags::AP_RO_ALL;
        }
        // If neither readable nor writable, leave AP as 0 (no EL0 access)

        // UXN/PXN for execute permission
        if !self.is_executable() {
            pte |= flags::UXN | flags::PXN;
        }

        pte
    }
}

/// Mapping state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VmMappingState {
    /// Mapping is active, page table entries are valid
    Mapped,
    /// Mapping has been unmapped, page table entries cleared
    Unmapped,
}

impl VmMappingState {
    /// Get state name for logging
    pub fn name(&self) -> &'static str {
        match self {
            Self::Mapped => "Mapped",
            Self::Unmapped => "Unmapped",
        }
    }
}

/// A mapping of a VmObject into an address space
///
/// References VmObject by ID (not Arc). Access object via registry.
pub struct VmMapping {
    /// Unique ID within the address space
    id: VmMappingId,
    /// Current state
    state: VmMappingState,
    /// The VmObject being mapped (by ID)
    object_id: VmObjectId,
    /// Offset into the VmObject (in pages)
    offset_pages: usize,
    /// Number of pages mapped
    num_pages: usize,
    /// Virtual address where mapping starts
    virt_addr: u64,
    /// Protection flags
    prot: Protection,
}

impl VmMapping {
    /// Create a new mapping
    ///
    /// This does NOT install page table entries - caller must do that.
    /// Caller must have already called add_object_mapping() on the VmObject.
    pub fn new(
        id: VmMappingId,
        object_id: VmObjectId,
        offset_pages: usize,
        num_pages: usize,
        virt_addr: u64,
        prot: Protection,
    ) -> Self {
        Self {
            id,
            state: VmMappingState::Mapped,
            object_id,
            offset_pages,
            num_pages,
            virt_addr,
            prot,
        }
    }

    /// Get the mapping ID
    #[inline]
    pub fn id(&self) -> VmMappingId {
        self.id
    }

    /// Get the current state
    #[inline]
    pub fn state(&self) -> VmMappingState {
        self.state
    }

    /// Get the VmObject ID
    #[inline]
    pub fn object_id(&self) -> VmObjectId {
        self.object_id
    }

    /// Get the virtual address
    #[inline]
    pub fn virt_addr(&self) -> u64 {
        self.virt_addr
    }

    /// Get the offset in pages
    #[inline]
    pub fn offset_pages(&self) -> usize {
        self.offset_pages
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

    /// Get the protection flags
    #[inline]
    pub fn protection(&self) -> Protection {
        self.prot
    }

    /// Check if this mapping contains a virtual address
    #[inline]
    pub fn contains(&self, va: u64) -> bool {
        va >= self.virt_addr && va < self.virt_addr + self.size() as u64
    }

    /// Get the page index in the VmObject for a given virtual address
    ///
    /// Returns None if address not in this mapping or mapping is unmapped.
    pub fn object_page_index(&self, va: u64) -> Option<usize> {
        if self.state != VmMappingState::Mapped {
            return None;
        }

        if !self.contains(va) {
            return None;
        }

        let page_offset = ((va - self.virt_addr) / PageFrame::SIZE as u64) as usize;
        Some(self.offset_pages + page_offset)
    }

    /// Mark this mapping as unmapped
    ///
    /// This does NOT clear page table entries - caller must do that first.
    /// Caller must also call remove_object_mapping() on the VmObject.
    pub fn mark_unmapped(&mut self) {
        self.state = VmMappingState::Unmapped;
    }

    /// Check if mapping is currently mapped
    #[inline]
    pub fn is_mapped(&self) -> bool {
        self.state == VmMappingState::Mapped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_protection() {
        let prot = Protection::RW;
        assert!(prot.is_readable());
        assert!(prot.is_writable());
        assert!(!prot.is_executable());

        let prot2 = Protection::READ.union(Protection::EXEC);
        assert!(prot2.is_readable());
        assert!(!prot2.is_writable());
        assert!(prot2.is_executable());
    }

    #[test]
    fn test_state_names() {
        assert_eq!(VmMappingState::Mapped.name(), "Mapped");
        assert_eq!(VmMappingState::Unmapped.name(), "Unmapped");
    }

    #[test]
    fn test_mapping_contains() {
        let mapping = VmMapping::new(
            VmMappingId(1),
            VmObjectId(1),
            0,
            4,  // 4 pages = 16KB
            0x1000_0000,
            Protection::RW,
        );

        assert!(mapping.contains(0x1000_0000));
        assert!(mapping.contains(0x1000_3FFF));
        assert!(!mapping.contains(0x1000_4000));
        assert!(!mapping.contains(0x0FFF_FFFF));
    }

    #[test]
    fn test_object_page_index() {
        let mapping = VmMapping::new(
            VmMappingId(1),
            VmObjectId(1),
            2,  // offset 2 pages into object
            4,  // 4 pages mapped
            0x1000_0000,
            Protection::RW,
        );

        // First page of mapping = page 2 of object
        assert_eq!(mapping.object_page_index(0x1000_0000), Some(2));
        // Second page of mapping = page 3 of object
        assert_eq!(mapping.object_page_index(0x1000_1000), Some(3));
        // Last page of mapping = page 5 of object
        assert_eq!(mapping.object_page_index(0x1000_3000), Some(5));
        // Outside mapping
        assert_eq!(mapping.object_page_index(0x1000_4000), None);
    }
}
