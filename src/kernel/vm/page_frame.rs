//! Physical Page Frame Management
//!
//! A PageFrame represents a single 4KB physical page. This is the atomic
//! unit of physical memory in the VM system.
//!
//! # Design (no_std)
//!
//! This module is intentionally simple - it provides constants and utility
//! types for page management. The actual page ownership and lifecycle is
//! managed by VmObject, not by PageFrame structs with reference counting.
//!
//! For the no_std kernel:
//! - VmObject owns pages directly via PageEntry (phys_addr + allocated flag)
//! - PMM handles physical allocation/deallocation
//! - No Arc/refcounting needed - ownership is explicit via VmObject

/// Page size (4KB)
pub const PAGE_SIZE: usize = 4096;

/// Flags for page attributes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PageFrameFlags(u32);

impl PageFrameFlags {
    /// No special flags
    pub const NONE: Self = Self(0);
    /// Page is dirty (modified)
    pub const DIRTY: Self = Self(1 << 0);
    /// Page has been accessed
    pub const ACCESSED: Self = Self(1 << 1);
    /// Page is pinned (cannot be reclaimed)
    pub const PINNED: Self = Self(1 << 2);
    /// Page is for DMA (physically contiguous requirement)
    pub const DMA: Self = Self(1 << 3);
    /// Page is device MMIO (not backed by RAM)
    pub const DEVICE: Self = Self(1 << 4);

    /// Check if a flag is set
    #[inline]
    pub fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    /// Combine flags
    #[inline]
    pub fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// PageFrame provides constants and utilities for page management.
///
/// Note: This is not a reference-counted handle. Actual page ownership
/// is managed by VmObject's PageEntry array. This struct exists to
/// provide SIZE constant and page arithmetic helpers.
pub struct PageFrame;

impl PageFrame {
    /// Page size (4KB)
    pub const SIZE: usize = PAGE_SIZE;

    /// Align address down to page boundary
    #[inline]
    pub const fn align_down(addr: u64) -> u64 {
        addr & !(Self::SIZE as u64 - 1)
    }

    /// Align address up to page boundary
    #[inline]
    pub const fn align_up(addr: u64) -> u64 {
        (addr + Self::SIZE as u64 - 1) & !(Self::SIZE as u64 - 1)
    }

    /// Check if address is page-aligned
    #[inline]
    pub const fn is_aligned(addr: u64) -> bool {
        (addr & (Self::SIZE as u64 - 1)) == 0
    }

    /// Calculate number of pages needed for a byte size
    #[inline]
    pub const fn pages_for_bytes(bytes: usize) -> usize {
        (bytes + Self::SIZE - 1) / Self::SIZE
    }

    /// Calculate page index for an address within a range
    #[inline]
    pub const fn page_index(base: u64, addr: u64) -> usize {
        ((addr - base) / Self::SIZE as u64) as usize
    }

    /// Calculate offset within a page
    #[inline]
    pub const fn page_offset(addr: u64) -> usize {
        (addr as usize) & (Self::SIZE - 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flags() {
        let flags = PageFrameFlags::DIRTY.union(PageFrameFlags::PINNED);
        assert!(flags.contains(PageFrameFlags::DIRTY));
        assert!(flags.contains(PageFrameFlags::PINNED));
        assert!(!flags.contains(PageFrameFlags::DMA));
    }

    #[test]
    fn test_alignment() {
        assert_eq!(PageFrame::align_down(0x1234), 0x1000);
        assert_eq!(PageFrame::align_up(0x1234), 0x2000);
        assert!(PageFrame::is_aligned(0x4000));
        assert!(!PageFrame::is_aligned(0x4001));
    }

    #[test]
    fn test_page_arithmetic() {
        assert_eq!(PageFrame::pages_for_bytes(4096), 1);
        assert_eq!(PageFrame::pages_for_bytes(4097), 2);
        assert_eq!(PageFrame::pages_for_bytes(0), 0);
        assert_eq!(PageFrame::page_index(0x10000, 0x12000), 2);
        assert_eq!(PageFrame::page_offset(0x12345), 0x345);
    }
}
