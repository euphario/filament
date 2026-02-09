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

    /// Get a mutable reference to a registered port by name
    pub fn get_mut(&mut self, name: &[u8]) -> Option<&mut RegisteredPort> {
        self.find_slot(name).and_then(|i| self.ports[i].as_mut())
    }

    /// Get the type of a registered port (wire protocol value)
    pub fn get_port_type(&self, name: &[u8]) -> Option<u8> {
        let slot = self.find_slot(name)?;
        self.ports[slot].as_ref().map(|p| p.port_type())
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

        // Check for existing port
        if let Some(slot_idx) = self.find_slot(name) {
            if let Some(port) = &mut self.ports[slot_idx] {
                if port.owner != owner {
                    return Err(SysError::AlreadyExists);
                }
                // Update with new info
                port.shmem_id = shmem_id;
                // Keep existing state on re-registration
                port.port_info = *info;
                return Ok(());
            }
        }

        // Find empty slot for new port
        let slot_idx = self.find_empty_slot().ok_or(SysError::NoSpace)?;

        let mut port = RegisteredPort::empty();
        port.name[..name.len()].copy_from_slice(name);
        port.name_len = name.len() as u8;
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

    /// Helper to register a port with PortInfo
    fn register_port(ports: &mut Ports, name: &[u8], owner: u8, class: PortClass) -> Result<(), SysError> {
        let info = PortInfo::new(name, class);
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
    fn test_duplicate_register() {
        let mut ports = Ports::new();

        assert!(register_port(&mut ports, b"test:", 1, PortClass::Service).is_ok());
        assert!(register_port(&mut ports, b"test:", 2, PortClass::Service).is_err());
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
