//! Port Registry
//!
//! Tracks registered service ports for routing and dependency resolution.
//! Uses trait-based design for testability.

use userlib::error::SysError;

// =============================================================================
// Constants
// =============================================================================

pub const MAX_PORT_NAME: usize = 32;
pub const MAX_PORTS: usize = 64;

// =============================================================================
// RegisteredPort
// =============================================================================

/// A registered port
#[derive(Clone, Copy)]
pub struct RegisteredPort {
    /// Port name (e.g., "console:")
    name: [u8; MAX_PORT_NAME],
    name_len: u8,
    /// Service index that owns this port (0xFF = devd itself)
    owner: u8,
    /// Is the port currently available?
    available: bool,
}

impl RegisteredPort {
    pub const fn empty() -> Self {
        Self {
            name: [0; MAX_PORT_NAME],
            name_len: 0,
            owner: 0,
            available: false,
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
}

// =============================================================================
// PortRegistry Trait
// =============================================================================

/// Port registration and lookup trait
pub trait PortRegistry {
    /// Register a new port
    fn register(&mut self, name: &[u8], owner: u8) -> Result<(), SysError>;

    /// Unregister a port by name
    fn unregister(&mut self, name: &[u8]);

    /// Check if a port is available (registered and ready)
    fn available(&self, name: &[u8]) -> bool;

    /// Set port availability status
    fn set_availability(&mut self, name: &[u8], available: bool);

    /// Find owner of a port
    fn find_owner(&self, name: &[u8]) -> Option<u8>;

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
}

impl PortRegistry for Ports {
    fn register(&mut self, name: &[u8], owner: u8) -> Result<(), SysError> {
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
}
