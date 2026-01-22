//! Capability-based Security
//!
//! Implements a simple capability system for the microkernel.
//! Each task has a set of capabilities that control what it can do.
//!
//! Capabilities are granted at spawn time and can be delegated (with restrictions).

#![allow(dead_code)]

/// Capability flags (bitfield)
/// Each bit represents a specific capability
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(transparent)]
pub struct Capabilities(u64);

impl Capabilities {
    /// Empty capability set
    pub const NONE: Self = Self(0);

    /// All capabilities (superuser)
    pub const ALL: Self = Self(u64::MAX);

    /// Default capabilities for user processes
    pub const USER_DEFAULT: Self = Self(
        Self::IPC.0 | Self::MEMORY.0 | Self::SPAWN.0
    );

    /// Default capabilities for drivers
    pub const DRIVER_DEFAULT: Self = Self(
        Self::IPC.0 | Self::MEMORY.0 | Self::SPAWN.0 |
        Self::SCHEME_CREATE.0 | Self::IRQ_CLAIM.0 | Self::MMIO.0 |
        Self::DMA.0 | Self::RAW_DEVICE.0
    );

    // Individual capability bits

    /// Can use IPC (send/receive messages)
    pub const IPC: Self = Self(1 << 0);

    /// Can allocate memory (mmap)
    pub const MEMORY: Self = Self(1 << 1);

    /// Can spawn child processes
    pub const SPAWN: Self = Self(1 << 2);

    /// Can create schemes (register as driver)
    pub const SCHEME_CREATE: Self = Self(1 << 3);

    /// Can claim IRQ ownership
    pub const IRQ_CLAIM: Self = Self(1 << 4);

    /// Can map MMIO regions
    pub const MMIO: Self = Self(1 << 5);

    /// Can allocate DMA memory
    pub const DMA: Self = Self(1 << 6);

    /// Can access filesystem
    pub const FILESYSTEM: Self = Self(1 << 7);

    /// Can access network
    pub const NETWORK: Self = Self(1 << 8);

    /// Can access raw devices
    pub const RAW_DEVICE: Self = Self(1 << 9);

    /// Can grant capabilities to children (delegation)
    pub const GRANT: Self = Self(1 << 10);

    /// Can kill any process (except init)
    pub const KILL: Self = Self(1 << 11);
    pub const RESERVED_12: Self = Self(1 << 12);
    pub const RESERVED_13: Self = Self(1 << 13);
    pub const RESERVED_14: Self = Self(1 << 14);
    pub const RESERVED_15: Self = Self(1 << 15);

    /// Create capabilities from raw bits
    pub const fn from_bits(bits: u64) -> Self {
        Self(bits)
    }

    /// Get raw bits
    pub const fn bits(&self) -> u64 {
        self.0
    }

    /// Check if this set contains a capability
    #[inline]
    pub const fn has(&self, cap: Capabilities) -> bool {
        (self.0 & cap.0) == cap.0
    }

    /// Check if this set contains all capabilities in another set
    #[inline]
    pub const fn contains(&self, other: Capabilities) -> bool {
        (self.0 & other.0) == other.0
    }

    /// Add capabilities
    #[inline]
    pub const fn with(&self, other: Capabilities) -> Self {
        Self(self.0 | other.0)
    }

    /// Remove capabilities
    #[inline]
    pub const fn without(&self, other: Capabilities) -> Self {
        Self(self.0 & !other.0)
    }

    /// Intersection of capabilities
    #[inline]
    pub const fn intersect(&self, other: Capabilities) -> Self {
        Self(self.0 & other.0)
    }

    /// Check if empty
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.0 == 0
    }
}

impl Default for Capabilities {
    fn default() -> Self {
        Self::NONE
    }
}

impl core::ops::BitOr for Capabilities {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitAnd for Capabilities {
    type Output = Self;
    fn bitand(self, rhs: Self) -> Self {
        Self(self.0 & rhs.0)
    }
}

impl core::ops::Not for Capabilities {
    type Output = Self;
    fn not(self) -> Self {
        Self(!self.0)
    }
}

/// Error when capability check fails
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapabilityError {
    /// The capability that was required
    pub required: Capabilities,
    /// The capabilities the process had
    pub had: Capabilities,
}

impl CapabilityError {
    pub fn new(required: Capabilities, had: Capabilities) -> Self {
        Self { required, had }
    }
}

/// Check if a task has the required capability
/// Returns Ok(()) if capability is present, Err otherwise
#[inline]
pub fn require(caps: Capabilities, required: Capabilities) -> Result<(), CapabilityError> {
    if caps.has(required) {
        Ok(())
    } else {
        Err(CapabilityError::new(required, caps))
    }
}

/// Determine capabilities for a child process based on parent's caps and requested caps
/// Children can only have capabilities the parent has and grants
pub fn child_capabilities(parent_caps: Capabilities, requested: Capabilities) -> Capabilities {
    // Child can't have more caps than parent
    let allowed = parent_caps.intersect(requested);

    // Parent must have GRANT capability to delegate any caps
    if !parent_caps.has(Capabilities::GRANT) {
        // Without GRANT, child gets minimal capabilities
        return Capabilities::USER_DEFAULT.intersect(allowed);
    }

    allowed
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_capabilities_none() {
        assert_eq!(Capabilities::NONE.bits(), 0);
    }

    #[test]
    fn test_capabilities_all() {
        assert_eq!(Capabilities::ALL.bits(), u64::MAX);
    }

    #[test]
    fn test_capabilities_has() {
        let caps = Capabilities::IPC.union(Capabilities::MEMORY);
        assert!(caps.has(Capabilities::IPC));
        assert!(caps.has(Capabilities::MEMORY));
        assert!(!caps.has(Capabilities::SPAWN));
    }

    #[test]
    fn test_capabilities_union() {
        let a = Capabilities::IPC;
        let b = Capabilities::MEMORY;
        let c = a.union(b);
        assert!(c.has(Capabilities::IPC));
        assert!(c.has(Capabilities::MEMORY));
    }

    #[test]
    fn test_capabilities_intersect() {
        let a = Capabilities::IPC.union(Capabilities::MEMORY);
        let b = Capabilities::MEMORY.union(Capabilities::SPAWN);
        let c = a.intersect(b);
        assert!(!c.has(Capabilities::IPC));
        assert!(c.has(Capabilities::MEMORY));
        assert!(!c.has(Capabilities::SPAWN));
    }

    #[test]
    fn test_capabilities_user_default() {
        let caps = Capabilities::USER_DEFAULT;
        assert!(caps.has(Capabilities::IPC));
        assert!(caps.has(Capabilities::MEMORY));
        assert!(caps.has(Capabilities::SPAWN));
        assert!(!caps.has(Capabilities::MMIO));
    }

    #[test]
    fn test_capabilities_driver_default() {
        let caps = Capabilities::DRIVER_DEFAULT;
        assert!(caps.has(Capabilities::MMIO));
        assert!(caps.has(Capabilities::DMA));
        assert!(caps.has(Capabilities::IRQ_CLAIM));
    }

    #[test]
    fn test_child_capabilities_with_grant() {
        let parent = Capabilities::ALL;
        let requested = Capabilities::IPC.union(Capabilities::MEMORY);
        let child = child_capabilities(parent, requested);
        assert!(child.has(Capabilities::IPC));
        assert!(child.has(Capabilities::MEMORY));
        assert!(!child.has(Capabilities::MMIO)); // Not requested
    }

    #[test]
    fn test_child_capabilities_without_grant() {
        let parent = Capabilities::IPC.union(Capabilities::MEMORY).union(Capabilities::SPAWN);
        let requested = Capabilities::ALL;
        let child = child_capabilities(parent, requested);
        // Without GRANT, child gets USER_DEFAULT intersection
        assert!(child.has(Capabilities::IPC));
        assert!(child.has(Capabilities::MEMORY));
    }

    #[test]
    fn test_check_capability_success() {
        let caps = Capabilities::IPC.union(Capabilities::MEMORY);
        assert!(check_capability(caps, Capabilities::IPC).is_ok());
    }

    #[test]
    fn test_check_capability_failure() {
        let caps = Capabilities::IPC;
        assert!(check_capability(caps, Capabilities::MMIO).is_err());
    }
}
