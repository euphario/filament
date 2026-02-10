//! Port Registry
//!
//! Tracks registered service ports for routing and dependency resolution.
//! Flat store keyed by port name. Uses trait-based design for testability.

use abi::{PortInfo, PortClass, PortState};
use userlib::error::SysError;

// =============================================================================
// Constants
// =============================================================================

pub const MAX_PORT_NAME: usize = 32;
pub const MAX_PORTS: usize = 64;

// =============================================================================
// RegisteredPort
// =============================================================================

/// A registered port with unified PortInfo
#[derive(Clone, Copy)]
pub struct RegisteredPort {
    /// Port name (e.g., "console:")
    name: [u8; MAX_PORT_NAME],
    name_len: u8,
    /// Monotonic port ID assigned at registration
    port_id: u8,
    /// Parent port ID (0xFF = root / no parent)
    parent_port_id: u8,
    /// Service index that owns this port (0xFF = devd itself)
    owner: u8,
    /// Port lifecycle state
    state: PortState,
    /// DataPort shared memory ID (0 = no DataPort)
    shmem_id: u32,
    /// Unified port info
    port_info: PortInfo,
    /// Link ID of the child service spawned for this port (0 = none)
    child_link_id: u32,
}

impl RegisteredPort {
    pub const fn empty() -> Self {
        Self {
            name: [0; MAX_PORT_NAME],
            name_len: 0,
            port_id: 0xFF,
            parent_port_id: 0xFF,
            owner: 0,
            state: PortState::Safe,
            shmem_id: 0,
            port_info: PortInfo::empty(),
            child_link_id: 0,
        }
    }

    pub fn name(&self) -> &[u8] {
        &self.name[..self.name_len as usize]
    }

    pub fn matches(&self, name: &[u8]) -> bool {
        self.name_len as usize == name.len() &&
            &self.name[..self.name_len as usize] == name
    }

    pub fn port_id(&self) -> u8 {
        self.port_id
    }

    pub fn parent_port_id(&self) -> u8 {
        self.parent_port_id
    }

    pub fn owner(&self) -> u8 {
        self.owner
    }

    pub fn state(&self) -> PortState {
        self.state
    }

    pub fn is_ready(&self) -> bool {
        self.state == PortState::Claimed
    }

    /// Legacy compatibility - returns true if Claimed
    pub fn is_available(&self) -> bool {
        self.state == PortState::Claimed
    }

    /// Get wire protocol port type (u8) derived from PortClass
    /// Values: 0=Unknown, 1=Block, 2=Partition (Block with subclass), 3=Filesystem,
    ///         4=Usb, 5=Network, 6=Console, 7=Service, 8=Storage,
    ///         9=Pcie, 10=Uart, 11=Klog, 12=Ethernet
    pub fn port_type(&self) -> u8 {
        match self.port_info.port_class {
            PortClass::Block => {
                // Distinguish block vs partition by subclass
                if self.port_info.port_subclass != 0 { 2 } else { 1 }
            }
            PortClass::Filesystem => 3,
            PortClass::Usb => 4,
            PortClass::Network => 5,
            PortClass::Console => 6,
            PortClass::Service => 7,
            PortClass::StorageController => 8,
            PortClass::Pcie => 9,
            PortClass::Uart => 10,
            PortClass::Klog => 11,
            PortClass::Ethernet => 12,
            _ => 0,
        }
    }

    pub fn shmem_id(&self) -> u32 {
        self.shmem_id
    }

    pub fn has_dataport(&self) -> bool {
        self.shmem_id != 0
    }

    pub fn set_shmem_id(&mut self, shmem_id: u32) {
        self.shmem_id = shmem_id;
    }

    pub fn child_link_id(&self) -> u32 {
        self.child_link_id
    }

    pub fn set_child_link_id(&mut self, id: u32) {
        self.child_link_id = id;
    }

    /// Get raw metadata bytes from PortInfo
    pub fn metadata(&self) -> &[u8] {
        // Safety: accessing raw union field is always safe for reads
        unsafe { &self.port_info.metadata.raw }
    }

    /// Get unified PortInfo
    pub fn port_info(&self) -> &PortInfo {
        &self.port_info
    }

    /// Get port class
    pub fn port_class(&self) -> PortClass {
        self.port_info.port_class
    }

    /// Get port subclass
    pub fn port_subclass(&self) -> u16 {
        self.port_info.port_subclass
    }

    /// Get vendor ID
    pub fn vendor_id(&self) -> u16 {
        self.port_info.vendor_id
    }

    /// Get device ID
    pub fn device_id(&self) -> u16 {
        self.port_info.device_id
    }

    /// Get capability flags
    pub fn caps(&self) -> u16 {
        self.port_info.caps
    }
}

// =============================================================================
// PortRegistry Trait
// =============================================================================

/// Port registration and lookup trait
pub trait PortRegistry {
    /// Update shmem_id for an existing port
    fn set_shmem_id(&mut self, name: &[u8], shmem_id: u32) -> Result<(), SysError>;

    /// Register a port with unified PortInfo
    fn register_with_port_info(
        &mut self,
        info: &PortInfo,
        owner: u8,
        shmem_id: u32,
    ) -> Result<(), SysError>;

    /// Unregister a port by name
    fn unregister(&mut self, name: &[u8]);

    /// Check if a port is ready
    fn is_ready(&self, name: &[u8]) -> bool;

    /// Get port state
    fn get_state(&self, name: &[u8]) -> Option<PortState>;

    /// Set port state. Returns previous state if port exists.
    fn set_state(&mut self, name: &[u8], state: PortState) -> Option<PortState>;

    /// Set port state by port_id. Returns previous state if port exists.
    fn set_state_by_id(&mut self, port_id: u8, state: PortState) -> Option<PortState>;

    /// Legacy: Check if a port is available (alias for is_ready)
    fn available(&self, name: &[u8]) -> bool { self.is_ready(name) }

    /// Find owner of a port
    fn find_owner(&self, name: &[u8]) -> Option<u8>;

    /// Get port by name
    fn get(&self, name: &[u8]) -> Option<&RegisteredPort>;

    /// Get port count
    fn count(&self) -> usize;

    /// Iterate over all registered ports
    fn for_each<F: FnMut(&RegisteredPort)>(&self, f: F);
}

// =============================================================================
// Ports Implementation
// =============================================================================

/// Concrete implementation of PortRegistry
pub struct Ports {
    ports: [Option<RegisteredPort>; MAX_PORTS],
    count: usize,
    /// Monotonic counter for port IDs
    next_port_id: u8,
}

impl Ports {
    pub const fn new() -> Self {
        Self {
            ports: [const { None }; MAX_PORTS],
            count: 0,
            next_port_id: 0,
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

    /// Find slot by (parent_port_id, name) pair — the tree-unique key
    fn find_slot_with_parent(&self, name: &[u8], parent_id: u8) -> Option<usize> {
        (0..MAX_PORTS).find(|&i| {
            self.ports[i]
                .as_ref()
                .map(|p| p.matches(name) && p.parent_port_id == parent_id)
                .unwrap_or(false)
        })
    }

    fn find_empty_slot(&self) -> Option<usize> {
        (0..MAX_PORTS).find(|&i| self.ports[i].is_none())
    }

    /// Find a port by its monotonic port_id
    pub fn find_by_id(&self, port_id: u8) -> Option<usize> {
        (0..MAX_PORTS).find(|&i| {
            self.ports[i]
                .as_ref()
                .map(|p| p.port_id == port_id)
                .unwrap_or(false)
        })
    }

    /// Walk parent chain and build colon-separated path.
    /// Returns the number of bytes written to buf.
    pub fn resolve_path(&self, port_id: u8, buf: &mut [u8]) -> usize {
        // Collect names bottom-up
        let mut chain: [([u8; MAX_PORT_NAME], u8); 16] = [([0u8; MAX_PORT_NAME], 0); 16];
        let mut depth = 0;
        let mut current_id = port_id;

        while current_id != 0xFF && depth < 16 {
            let slot = match self.find_by_id(current_id) {
                Some(s) => s,
                None => break,
            };
            let port = match &self.ports[slot] {
                Some(p) => p,
                None => break,
            };
            chain[depth].0[..port.name_len as usize].copy_from_slice(&port.name[..port.name_len as usize]);
            chain[depth].1 = port.name_len;
            current_id = port.parent_port_id;
            depth += 1;
        }

        if depth == 0 {
            return 0;
        }

        // Write top-down (reverse the chain)
        let mut pos = 0;
        for i in (0..depth).rev() {
            let name_len = chain[i].1 as usize;
            if pos + name_len > buf.len() {
                break;
            }
            buf[pos..pos + name_len].copy_from_slice(&chain[i].0[..name_len]);
            pos += name_len;
            // Add colon separator (except after the last segment)
            if i > 0 && pos < buf.len() {
                buf[pos] = b':';
                pos += 1;
            }
        }
        pos
    }

    /// Get a mutable reference to a registered port by name
    pub fn get_mut(&mut self, name: &[u8]) -> Option<&mut RegisteredPort> {
        self.find_slot(name).and_then(|i| self.ports[i].as_mut())
    }

    /// Get a mutable reference to a registered port by port_id
    pub fn get_mut_by_id(&mut self, port_id: u8) -> Option<&mut RegisteredPort> {
        self.find_by_id(port_id).and_then(|i| self.ports[i].as_mut())
    }

    /// Get a port by port_id
    pub fn get_by_id(&self, port_id: u8) -> Option<&RegisteredPort> {
        self.find_by_id(port_id).and_then(|i| self.ports[i].as_ref())
    }

    /// Get the type of a registered port (wire protocol value)
    pub fn get_port_type(&self, name: &[u8]) -> Option<u8> {
        let slot = self.find_slot(name)?;
        self.ports[slot].as_ref().map(|p| p.port_type())
    }

    /// Get port_id for a port by name (first match)
    pub fn get_port_id(&self, name: &[u8]) -> Option<u8> {
        let slot = self.find_slot(name)?;
        self.ports[slot].as_ref().map(|p| p.port_id)
    }
}

impl PortRegistry for Ports {
    fn set_shmem_id(&mut self, name: &[u8], shmem_id: u32) -> Result<(), SysError> {
        if let Some(idx) = self.find_slot(name) {
            if let Some(port) = &mut self.ports[idx] {
                port.shmem_id = shmem_id;
                return Ok(());
            }
        }
        Err(SysError::NotFound)
    }

    fn register_with_port_info(
        &mut self,
        info: &PortInfo,
        owner: u8,
        shmem_id: u32,
    ) -> Result<(), SysError> {
        let name = info.name_bytes();
        if name.len() > MAX_PORT_NAME {
            return Err(SysError::InvalidArgument);
        }

        // Use parent_port_id directly from PortInfo
        let parent_id = info.parent_port_id;

        // Tree-based uniqueness: check (parent_id, name) pair
        if let Some(slot_idx) = self.find_slot_with_parent(name, parent_id) {
            if let Some(port) = &mut self.ports[slot_idx] {
                if port.owner != owner {
                    return Err(SysError::AlreadyExists);
                }
                // Same owner re-registration — update in place
                port.shmem_id = shmem_id;
                port.port_info = *info;
                return Ok(());
            }
        }

        // Find empty slot for new port
        let slot_idx = self.find_empty_slot().ok_or(SysError::NoSpace)?;

        // Assign monotonic port_id
        let port_id = self.next_port_id;
        self.next_port_id = self.next_port_id.wrapping_add(1);
        // Skip 0xFF (reserved for "no parent")
        if self.next_port_id == 0xFF {
            self.next_port_id = 0;
        }

        let mut port = RegisteredPort::empty();
        port.name[..name.len()].copy_from_slice(name);
        port.name_len = name.len() as u8;
        port.port_id = port_id;
        port.parent_port_id = parent_id;
        port.owner = owner;
        port.state = PortState::Safe;  // Starts Safe, transitions to Claimed when owner connects
        port.shmem_id = shmem_id;
        port.port_info = *info;

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

    fn is_ready(&self, name: &[u8]) -> bool {
        self.find_slot(name)
            .and_then(|i| self.ports[i].as_ref())
            .map(|p| p.state == PortState::Claimed)
            .unwrap_or(false)
    }

    fn get_state(&self, name: &[u8]) -> Option<PortState> {
        self.find_slot(name)
            .and_then(|i| self.ports[i].as_ref())
            .map(|p| p.state)
    }

    fn set_state(&mut self, name: &[u8], state: PortState) -> Option<PortState> {
        if let Some(idx) = self.find_slot(name) {
            if let Some(port) = &mut self.ports[idx] {
                let old = port.state;
                port.state = state;
                return Some(old);
            }
        }
        None
    }

    fn set_state_by_id(&mut self, port_id: u8, state: PortState) -> Option<PortState> {
        if let Some(idx) = self.find_by_id(port_id) {
            if let Some(port) = &mut self.ports[idx] {
                let old = port.state;
                port.state = state;
                return Some(old);
            }
        }
        None
    }

    fn find_owner(&self, name: &[u8]) -> Option<u8> {
        self.find_slot(name)
            .and_then(|i| self.ports[i].as_ref())
            .map(|p| p.owner)
    }

    fn get(&self, name: &[u8]) -> Option<&RegisteredPort> {
        self.find_slot(name).and_then(|i| self.ports[i].as_ref())
    }

    fn count(&self) -> usize {
        self.count
    }

    fn for_each<F: FnMut(&RegisteredPort)>(&self, mut f: F) {
        for port in self.ports.iter().flatten() {
            f(port);
        }
    }

}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper to register a port with PortInfo (root level, no parent)
    fn register_port(ports: &mut Ports, name: &[u8], owner: u8, class: PortClass) -> Result<(), SysError> {
        let info = PortInfo::new(name, class);
        ports.register_with_port_info(&info, owner, 0)
    }

    /// Helper to register a port with a parent
    fn register_port_with_parent(ports: &mut Ports, name: &[u8], parent: &[u8], owner: u8, class: PortClass) -> Result<(), SysError> {
        let mut info = PortInfo::new(name, class);
        info.parent_port_id = ports.get(parent).map(|p| p.port_id()).unwrap_or(0xFF);
        ports.register_with_port_info(&info, owner, 0)
    }

    #[test]
    fn test_register_unregister() {
        let mut ports = Ports::new();

        assert!(register_port(&mut ports, b"test:", 1, PortClass::Service).is_ok());
        assert_eq!(ports.count(), 1);
        // Port starts Safe (not yet claimed by owner)
        assert!(!ports.available(b"test:"));
        assert_eq!(ports.get_state(b"test:"), Some(PortState::Safe));

        ports.unregister(b"test:");
        assert_eq!(ports.count(), 0);
        assert!(!ports.available(b"test:"));
    }

    #[test]
    fn test_duplicate_register_same_parent() {
        let mut ports = Ports::new();

        // Same name, same parent (root), different owner → collision
        assert!(register_port(&mut ports, b"test:", 1, PortClass::Service).is_ok());
        assert!(register_port(&mut ports, b"test:", 2, PortClass::Service).is_err());
    }

    #[test]
    fn test_same_name_different_parent_no_collision() {
        let mut ports = Ports::new();

        // Register parent ports
        assert!(register_port(&mut ports, b"nvme0", 1, PortClass::StorageController).is_ok());
        assert!(register_port(&mut ports, b"usb0:msc", 2, PortClass::Usb).is_ok());

        // Register "0:fat" under nvme0 → ok
        assert!(register_port_with_parent(&mut ports, b"0:fat", b"nvme0", 3, PortClass::Block).is_ok());
        // Register "0:fat" under usb0:msc → ok (different parent = no collision)
        assert!(register_port_with_parent(&mut ports, b"0:fat", b"usb0:msc", 4, PortClass::Block).is_ok());
        assert_eq!(ports.count(), 4);
    }

    #[test]
    fn test_port_id_assigned() {
        let mut ports = Ports::new();

        assert!(register_port(&mut ports, b"pcie0", 1, PortClass::Pcie).is_ok());
        assert!(register_port(&mut ports, b"uart0", 2, PortClass::Uart).is_ok());

        let p0 = ports.get(b"pcie0").unwrap();
        let p1 = ports.get(b"uart0").unwrap();
        assert_eq!(p0.port_id(), 0);
        assert_eq!(p1.port_id(), 1);
        assert_eq!(p0.parent_port_id(), 0xFF); // root
    }

    #[test]
    fn test_resolve_path() {
        let mut ports = Ports::new();

        // Build a tree: pcie0 → 00:02.0 → nvme0 → 0:fat
        assert!(register_port(&mut ports, b"pcie0", 1, PortClass::Pcie).is_ok());
        assert!(register_port_with_parent(&mut ports, b"00:02.0", b"pcie0", 2, PortClass::Pcie).is_ok());
        assert!(register_port_with_parent(&mut ports, b"nvme0", b"00:02.0", 3, PortClass::StorageController).is_ok());
        assert!(register_port_with_parent(&mut ports, b"0:fat", b"nvme0", 4, PortClass::Block).is_ok());

        // Resolve path for leaf port (0:fat)
        let fat_id = ports.get(b"0:fat").unwrap().port_id();
        let mut buf = [0u8; 128];
        let len = ports.resolve_path(fat_id, &mut buf);
        assert_eq!(&buf[..len], b"pcie0:00:02.0:nvme0:0:fat");
    }

    #[test]
    fn test_find_by_id() {
        let mut ports = Ports::new();

        assert!(register_port(&mut ports, b"test:", 1, PortClass::Service).is_ok());
        let port_id = ports.get(b"test:").unwrap().port_id();
        assert!(ports.find_by_id(port_id).is_some());
        assert!(ports.find_by_id(0xFE).is_none());
    }

    #[test]
    fn test_port_state() {
        let mut ports = Ports::new();

        assert!(register_port(&mut ports, b"test:", 1, PortClass::Service).is_ok());
        assert_eq!(ports.get_state(b"test:"), Some(PortState::Safe));
        assert!(!ports.is_ready(b"test:"));

        // Transition to Claimed
        let old = ports.set_state(b"test:", PortState::Claimed);
        assert_eq!(old, Some(PortState::Safe));
        assert!(ports.is_ready(b"test:"));

        // Transition to Resetting
        let old = ports.set_state(b"test:", PortState::Resetting);
        assert_eq!(old, Some(PortState::Claimed));
        assert!(!ports.is_ready(b"test:"));
        assert_eq!(ports.get_state(b"test:"), Some(PortState::Resetting));

        // Transition back to Safe
        let old = ports.set_state(b"test:", PortState::Safe);
        assert_eq!(old, Some(PortState::Resetting));
        assert!(!ports.is_ready(b"test:"));
    }

    #[test]
    fn test_find_owner() {
        let mut ports = Ports::new();

        assert!(register_port(&mut ports, b"test:", 42, PortClass::Service).is_ok());
        assert_eq!(ports.find_owner(b"test:"), Some(42));
        assert_eq!(ports.find_owner(b"nonexistent:"), None);
    }

    #[test]
    fn test_port_class() {
        let mut ports = Ports::new();

        assert!(register_port(&mut ports, b"disk0:", 1, PortClass::Block).is_ok());
        assert_eq!(ports.get(b"disk0:").map(|p| p.port_class()), Some(PortClass::Block));

        assert!(register_port(&mut ports, b"other:", 2, PortClass::Unknown).is_ok());
        assert_eq!(ports.get(b"other:").map(|p| p.port_class()), Some(PortClass::Unknown));
    }
}
