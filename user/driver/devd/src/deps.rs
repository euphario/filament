//! Dependency Resolution
//!
//! Checks service dependencies and finds dependent services.
//! Uses trait-based design for testability.

use crate::service::{Service, ServiceManager, ServiceRegistry, MAX_SERVICES};
use crate::ports::{PortRegistry, Ports};

// =============================================================================
// DependencyResolver Trait
// =============================================================================

/// Dependency checking trait
///
/// Note: Uses concrete types for PortRegistry and ServiceManager because
/// the for_each methods have generic parameters which makes them not
/// object-safe. In practice, we only have one implementation anyway.
pub trait DependencyResolver {
    /// Check if a service's dependencies are satisfied
    fn satisfied(&self, service: &Service, ports: &Ports) -> bool;

    /// Find all services that depend on ports provided by a service
    /// Returns indices of dependent services
    fn find_dependents(
        &self,
        provider_idx: usize,
        services: &ServiceRegistry,
    ) -> DependentList;
}

// =============================================================================
// DependentList - No-alloc list of dependent indices
// =============================================================================

/// List of dependent service indices (no heap allocation)
pub struct DependentList {
    indices: [Option<usize>; MAX_SERVICES],
    count: usize,
}

impl DependentList {
    pub const fn new() -> Self {
        Self {
            indices: [None; MAX_SERVICES],
            count: 0,
        }
    }

    pub fn push(&mut self, idx: usize) -> bool {
        if self.count >= MAX_SERVICES {
            return false;
        }
        self.indices[self.count] = Some(idx);
        self.count += 1;
        true
    }

    pub fn contains(&self, idx: usize) -> bool {
        self.indices[..self.count].iter().any(|&i| i == Some(idx))
    }

    pub fn iter(&self) -> impl Iterator<Item = usize> + '_ {
        self.indices[..self.count].iter().filter_map(|&i| i)
    }

    pub fn len(&self) -> usize {
        self.count
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

// =============================================================================
// Dependencies Implementation
// =============================================================================

/// Concrete implementation of DependencyResolver
pub struct Dependencies;

impl Dependencies {
    pub const fn new() -> Self {
        Self
    }
}

impl DependencyResolver for Dependencies {
    fn satisfied(&self, service: &Service, ports: &Ports) -> bool {
        if let Some(def) = service.def() {
            for dep in def.dependencies {
                if dep.is_required() && !ports.available(dep.port_name()) {
                    return false;
                }
            }
        }
        true
    }

    fn find_dependents(
        &self,
        provider_idx: usize,
        services: &ServiceRegistry,
    ) -> DependentList {
        let mut dependents = DependentList::new();

        // Get ports that the provider registers
        let provider_ports: [Option<&'static [u8]>; 4] = {
            let mut ports = [None; 4];
            if let Some(def) = services.get(provider_idx).and_then(|s| s.def()) {
                for (i, pd) in def.registers.iter().enumerate() {
                    if i < 4 {
                        ports[i] = Some(pd.name);
                    }
                }
            }
            ports
        };

        // Also collect hierarchical children
        let children: [Option<u8>; 8] = {
            let mut ch = [None; 8];
            if let Some(provider) = services.get(provider_idx) {
                for i in 0..provider.child_count as usize {
                    if i < 8 {
                        ch[i] = provider.children[i];
                    }
                }
            }
            ch
        };

        // Find all services that depend on provider's ports
        services.for_each(|i, service| {
            if i == provider_idx {
                return; // Don't include provider itself
            }

            if !service.state.is_running() {
                return; // Skip non-running services
            }

            // Check if depends on provider's ports
            let depends_on_ports = service.def()
                .map(|d| d.dependencies.iter().any(|dep| {
                    let dep_port = dep.port_name();
                    provider_ports.iter().flatten().any(|&p| p == dep_port)
                }))
                .unwrap_or(false);

            // Check if is a child of provider
            let is_child = children.iter().flatten().any(|&c| c as usize == i);

            if (depends_on_ports || is_child) && !dependents.contains(i) {
                dependents.push(i);
            }
        });

        dependents
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ports::Ports;

    // Note: Full testing would require mock ServiceManager
    // For now, test DependentList

    #[test]
    fn test_dependent_list() {
        let mut list = DependentList::new();

        assert!(list.push(1));
        assert!(list.push(3));
        assert!(list.push(5));

        assert!(list.contains(1));
        assert!(list.contains(3));
        assert!(!list.contains(2));

        let items: Vec<_> = list.iter().collect();
        assert_eq!(items, vec![1, 3, 5]);
    }

    #[test]
    fn test_deps_with_ports() {
        use crate::service::Service;

        let deps = Dependencies::new();
        let mut ports = Ports::new();

        // Create a minimal service for testing
        let mut service = Service::empty();
        service.def_index = 1; // shell - depends on console:

        // Without console: port, deps not satisfied
        assert!(!deps.satisfied(&service, &ports));

        // Register console: port
        ports.register(b"console:", 0).unwrap();

        // Now deps should be satisfied
        assert!(deps.satisfied(&service, &ports));
    }
}
