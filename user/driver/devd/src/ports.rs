//! Port Registry
//!
//! Tracks registered service ports for routing and dependency resolution.
//! Supports hierarchical port trees (e.g., disk0: → part0:, part1:).
//! Uses trait-based design for testability.

use userlib::error::SysError;

// =============================================================================
// Constants
// =============================================================================

pub const MAX_PORT_NAME: usize = 32;
pub const MAX_PORTS: usize = 64;

// =============================================================================
// Port Type
// =============================================================================

/// Type of port - used for matching rules and understanding hierarchy
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PortType {
    /// Unknown or unspecified type
    Unknown = 0,
    /// Raw block device (scsi0:, nvme0:, disk0:)
    Block = 1,
    /// Partition on a block device (part0:, part1:)
    Partition = 2,
    /// Mounted filesystem (fat0:, ext0:)
    Filesystem = 3,
    /// USB endpoint or device
    Usb = 4,
    /// Network endpoint
    Network = 5,
    /// Console/TTY
    Console = 6,
    /// Service port (devd:, logd:)
    Service = 7,
}

impl PortType {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => PortType::Block,
            2 => PortType::Partition,
            3 => PortType::Filesystem,
            4 => PortType::Usb,
            5 => PortType::Network,
            6 => PortType::Console,
            7 => PortType::Service,
            _ => PortType::Unknown,
        }
    }
}

// =============================================================================
// RegisteredPort
// =============================================================================

/// A registered port with optional parent for hierarchy
#[derive(Clone, Copy)]
pub struct RegisteredPort {
    /// Port name (e.g., "console:")
    name: [u8; MAX_PORT_NAME],
    name_len: u8,
    /// Service index that owns this port (0xFF = devd itself)
    owner: u8,
    /// Is the port currently available?
    available: bool,
    /// Port type for matching and hierarchy
    port_type: PortType,
    /// Parent port index (0xFF = no parent / root)
    parent_idx: u8,
}

/// No parent sentinel value
pub const NO_PARENT: u8 = 0xFF;

impl RegisteredPort {
    pub const fn empty() -> Self {
        Self {
            name: [0; MAX_PORT_NAME],
            name_len: 0,
            owner: 0,
            available: false,
            port_type: PortType::Unknown,
            parent_idx: NO_PARENT,
        }
    }

    pub fn name(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }

    pub fn matches(&self, name: &[u8]) -> bool {
        self.name_len as usize == name.len() &&
            &self.name[..self.name_len as usize] == name
    }

    pub fn owner(&self) -> u8 {
        self.owner
    }

    pub fn is_available(&self) -> bool {
        self.available
    }

    pub fn port_type(&self) -> PortType {
        self.port_type
    }

    pub fn parent_idx(&self) -> Option<u8> {
        if self.parent_idx == NO_PARENT {
            None
        } else {
            Some(self.parent_idx)
        }
    }

    pub fn has_parent(&self) -> bool {
        self.parent_idx != NO_PARENT
    }
}

// =============================================================================
// PortRegistry Trait
// =============================================================================

/// Port registration and lookup trait
pub trait PortRegistry {
    /// Register a new port (root level, no parent)
    fn register(&mut self, name: &[u8], owner: u8) -> Result<(), SysError>;

    /// Register a child port with parent and type
    fn register_child(
        &mut self,
        name: &[u8],
        owner: u8,
        parent: &[u8],
        port_type: PortType,
    ) -> Result<(), SysError>;

    /// Register a port with type (root level)
    fn register_typed(
        &mut self,
        name: &[u8],
        owner: u8,
        port_type: PortType,
    ) -> Result<(), SysError>;

    /// Unregister a port by name
    fn unregister(&mut self, name: &[u8]);

    /// Check if a port is available (registered and ready)
    fn available(&self, name: &[u8]) -> bool;

    /// Set port availability status
    fn set_availability(&mut self, name: &[u8], available: bool);

    /// Find owner of a port
    fn find_owner(&self, name: &[u8]) -> Option<u8>;

    /// Get port by name
    fn get(&self, name: &[u8]) -> Option<&RegisteredPort>;

    /// Get port type
    fn port_type(&self, name: &[u8]) -> Option<PortType>;

    /// Get parent port name
    fn parent(&self, name: &[u8]) -> Option<&[u8]>;

    /// Get children of a port (ports that have this as parent)
    fn children(&self, name: &[u8]) -> ChildIterator<'_>;

    /// Get port count
    fn count(&self) -> usize;

    /// Iterate over all registered ports
    fn for_each<F: FnMut(&RegisteredPort)>(&self, f: F);

    /// Walk the port tree depth-first, calling f with (port, depth)
    fn walk_tree<F: FnMut(&RegisteredPort, usize)>(&self, f: F);

    /// Iterate over root ports (no parent)
    fn roots(&self) -> RootIterator<'_>;
}

/// Iterator over child ports
pub struct ChildIterator<'a> {
    ports: &'a [Option<RegisteredPort>; MAX_PORTS],
    parent_idx: Option<usize>,
    current: usize,
}

impl<'a> Iterator for ChildIterator<'a> {
    type Item = &'a RegisteredPort;

    fn next(&mut self) -> Option<Self::Item> {
        let parent_idx = self.parent_idx? as u8;

        while self.current < MAX_PORTS {
            let idx = self.current;
            self.current += 1;

            if let Some(port) = &self.ports[idx] {
                if port.parent_idx == parent_idx {
                    return Some(port);
                }
            }
        }
        None
    }
}

/// Iterator over root ports (no parent)
pub struct RootIterator<'a> {
    ports: &'a [Option<RegisteredPort>; MAX_PORTS],
    current: usize,
}

impl<'a> Iterator for RootIterator<'a> {
    type Item = &'a RegisteredPort;

    fn next(&mut self) -> Option<Self::Item> {
        while self.current < MAX_PORTS {
            let idx = self.current;
            self.current += 1;

            if let Some(port) = &self.ports[idx] {
                if port.parent_idx == NO_PARENT {
                    return Some(port);
                }
            }
        }
        None
    }
}

// =============================================================================
// Ports Implementation
// =============================================================================

/// Concrete implementation of PortRegistry
pub struct Ports {
    ports: [Option<RegisteredPort>; MAX_PORTS],
    count: usize,
}

impl Ports {
    pub const fn new() -> Self {
        Self {
            ports: [const { None }; MAX_PORTS],
            count: 0,
        }
    }

    fn find_slot(&self, name: &[u8]) -> Option<usize> {
        (0..MAX_PORTS).find(|&i| {
            self.ports[i]
                .as_ref()
                .map(|p| p.matches(name))
                .unwrap_or(false)
        })
    }

    fn find_empty_slot(&self) -> Option<usize> {
        (0..MAX_PORTS).find(|&i| self.ports[i].is_none())
    }
}

impl PortRegistry for Ports {
    fn register(&mut self, name: &[u8], owner: u8) -> Result<(), SysError> {
        self.register_typed(name, owner, PortType::Unknown)
    }

    fn register_child(
        &mut self,
        name: &[u8],
        owner: u8,
        parent: &[u8],
        port_type: PortType,
    ) -> Result<(), SysError> {
        if name.len() > MAX_PORT_NAME {
            return Err(SysError::InvalidArgument);
        }

        // Find parent
        let parent_idx = self.find_slot(parent).ok_or(SysError::NotFound)? as u8;

        // Check for duplicate
        if self.find_slot(name).is_some() {
            return Err(SysError::AlreadyExists);
        }

        // Find empty slot
        let slot_idx = self.find_empty_slot().ok_or(SysError::NoSpace)?;

        let mut port = RegisteredPort::empty();
        port.name[..name.len()].copy_from_slice(name);
        port.name_len = name.len() as u8;
        port.owner = owner;
        port.available = true;
        port.port_type = port_type;
        port.parent_idx = parent_idx;

        self.ports[slot_idx] = Some(port);
        self.count += 1;

        Ok(())
    }

    fn register_typed(
        &mut self,
        name: &[u8],
        owner: u8,
        port_type: PortType,
    ) -> Result<(), SysError> {
        if name.len() > MAX_PORT_NAME {
            return Err(SysError::InvalidArgument);
        }

        // Check for duplicate
        if self.find_slot(name).is_some() {
            return Err(SysError::AlreadyExists);
        }

        // Find empty slot
        let slot_idx = self.find_empty_slot().ok_or(SysError::NoSpace)?;

        let mut port = RegisteredPort::empty();
        port.name[..name.len()].copy_from_slice(name);
        port.name_len = name.len() as u8;
        port.owner = owner;
        port.available = true;
        port.port_type = port_type;
        port.parent_idx = NO_PARENT;

        self.ports[slot_idx] = Some(port);
        self.count += 1;

        Ok(())
    }

    fn unregister(&mut self, name: &[u8]) {
        if let Some(idx) = self.find_slot(name) {
            self.ports[idx] = None;
            self.count = self.count.saturating_sub(1);
        }
    }

    fn available(&self, name: &[u8]) -> bool {
        self.find_slot(name)
            .and_then(|i| self.ports[i].as_ref())
            .map(|p| p.available)
            .unwrap_or(false)
    }

    fn set_availability(&mut self, name: &[u8], available: bool) {
        if let Some(idx) = self.find_slot(name) {
            if let Some(port) = &mut self.ports[idx] {
                port.available = available;
            }
        }
    }

    fn find_owner(&self, name: &[u8]) -> Option<u8> {
        self.find_slot(name)
            .and_then(|i| self.ports[i].as_ref())
            .map(|p| p.owner)
    }

    fn get(&self, name: &[u8]) -> Option<&RegisteredPort> {
        self.find_slot(name).and_then(|i| self.ports[i].as_ref())
    }

    fn port_type(&self, name: &[u8]) -> Option<PortType> {
        self.get(name).map(|p| p.port_type)
    }

    fn parent(&self, name: &[u8]) -> Option<&[u8]> {
        let port = self.get(name)?;
        let parent_idx = port.parent_idx;
        if parent_idx == NO_PARENT {
            return None;
        }
        self.ports[parent_idx as usize].as_ref().map(|p| p.name())
    }

    fn children(&self, name: &[u8]) -> ChildIterator<'_> {
        let parent_idx = self.find_slot(name);
        ChildIterator {
            ports: &self.ports,
            parent_idx,
            current: 0,
        }
    }

    fn count(&self) -> usize {
        self.count
    }

    fn for_each<F: FnMut(&RegisteredPort)>(&self, mut f: F) {
        for port in self.ports.iter().flatten() {
            f(port);
        }
    }

    fn walk_tree<F: FnMut(&RegisteredPort, usize)>(&self, mut f: F) {
        // Walk depth-first from roots
        fn walk_recursive<F: FnMut(&RegisteredPort, usize)>(
            ports: &[Option<RegisteredPort>; MAX_PORTS],
            idx: usize,
            depth: usize,
            f: &mut F,
        ) {
            if let Some(port) = &ports[idx] {
                f(port, depth);
                // Find children of this port
                for i in 0..MAX_PORTS {
                    if let Some(child) = &ports[i] {
                        if child.parent_idx == idx as u8 {
                            walk_recursive(ports, i, depth + 1, f);
                        }
                    }
                }
            }
        }

        // Start from roots
        for i in 0..MAX_PORTS {
            if let Some(port) = &self.ports[i] {
                if port.parent_idx == NO_PARENT {
                    walk_recursive(&self.ports, i, 0, &mut f);
                }
            }
        }
    }

    fn roots(&self) -> RootIterator<'_> {
        RootIterator {
            ports: &self.ports,
            current: 0,
        }
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_unregister() {
        let mut ports = Ports::new();

        assert!(ports.register(b"test:", 1).is_ok());
        assert_eq!(ports.count(), 1);
        assert!(ports.available(b"test:"));

        ports.unregister(b"test:");
        assert_eq!(ports.count(), 0);
        assert!(!ports.available(b"test:"));
    }

    #[test]
    fn test_duplicate_register() {
        let mut ports = Ports::new();

        assert!(ports.register(b"test:", 1).is_ok());
        assert!(ports.register(b"test:", 2).is_err());
    }

    #[test]
    fn test_availability() {
        let mut ports = Ports::new();

        assert!(ports.register(b"test:", 1).is_ok());
        assert!(ports.available(b"test:"));

        ports.set_availability(b"test:", false);
        assert!(!ports.available(b"test:"));

        ports.set_availability(b"test:", true);
        assert!(ports.available(b"test:"));
    }

    #[test]
    fn test_find_owner() {
        let mut ports = Ports::new();

        assert!(ports.register(b"test:", 42).is_ok());
        assert_eq!(ports.find_owner(b"test:"), Some(42));
        assert_eq!(ports.find_owner(b"nonexistent:"), None);
    }

    #[test]
    fn test_port_type() {
        let mut ports = Ports::new();

        assert!(ports.register_typed(b"disk0:", 1, PortType::Block).is_ok());
        assert_eq!(ports.port_type(b"disk0:"), Some(PortType::Block));

        // Default register uses Unknown type
        assert!(ports.register(b"other:", 2).is_ok());
        assert_eq!(ports.port_type(b"other:"), Some(PortType::Unknown));
    }

    #[test]
    fn test_parent_child() {
        let mut ports = Ports::new();

        // Register parent
        assert!(ports.register_typed(b"disk0:", 1, PortType::Block).is_ok());

        // Register children
        assert!(ports.register_child(b"part0:", 1, b"disk0:", PortType::Partition).is_ok());
        assert!(ports.register_child(b"part1:", 1, b"disk0:", PortType::Partition).is_ok());

        // Check parent
        assert_eq!(ports.parent(b"part0:"), Some(b"disk0:".as_slice()));
        assert_eq!(ports.parent(b"part1:"), Some(b"disk0:".as_slice()));
        assert_eq!(ports.parent(b"disk0:"), None); // Root has no parent

        // Check children (count them manually)
        let mut child_count = 0;
        for _ in ports.children(b"disk0:") {
            child_count += 1;
        }
        assert_eq!(child_count, 2);

        // Check types
        assert_eq!(ports.port_type(b"part0:"), Some(PortType::Partition));
        assert_eq!(ports.port_type(b"part1:"), Some(PortType::Partition));
    }

    #[test]
    fn test_register_child_missing_parent() {
        let mut ports = Ports::new();

        // Should fail - parent doesn't exist
        let result = ports.register_child(b"part0:", 1, b"disk0:", PortType::Partition);
        assert!(result.is_err());
    }

    #[test]
    fn test_roots() {
        let mut ports = Ports::new();

        // Register two root ports
        assert!(ports.register_typed(b"disk0:", 1, PortType::Block).is_ok());
        assert!(ports.register_typed(b"console:", 2, PortType::Console).is_ok());

        // Register a child
        assert!(ports.register_child(b"part0:", 1, b"disk0:", PortType::Partition).is_ok());

        // Only roots should be returned (count manually)
        let mut root_count = 0;
        for _ in ports.roots() {
            root_count += 1;
        }
        assert_eq!(root_count, 2);
    }

    #[test]
    fn test_walk_tree() {
        let mut ports = Ports::new();

        // Build tree: disk0: -> part0:, part1: -> fat0:
        assert!(ports.register_typed(b"disk0:", 1, PortType::Block).is_ok());
        assert!(ports.register_child(b"part0:", 1, b"disk0:", PortType::Partition).is_ok());
        assert!(ports.register_child(b"part1:", 1, b"disk0:", PortType::Partition).is_ok());
        assert!(ports.register_child(b"fat0:", 2, b"part1:", PortType::Filesystem).is_ok());

        // Walk and track what we see
        let mut count = 0;
        let mut found_disk0_depth0 = false;
        let mut found_part0_depth1 = false;
        let mut found_part1_depth1 = false;
        let mut found_fat0_depth2 = false;

        ports.walk_tree(|port, depth| {
            count += 1;
            if port.matches(b"disk0:") && depth == 0 {
                found_disk0_depth0 = true;
            }
            if port.matches(b"part0:") && depth == 1 {
                found_part0_depth1 = true;
            }
            if port.matches(b"part1:") && depth == 1 {
                found_part1_depth1 = true;
            }
            if port.matches(b"fat0:") && depth == 2 {
                found_fat0_depth2 = true;
            }
        });

        // Should have 4 entries
        assert_eq!(count, 4);

        // Check each was found at correct depth
        assert!(found_disk0_depth0);
        assert!(found_part0_depth1);
        assert!(found_part1_depth1);
        assert!(found_fat0_depth2);
    }
}
