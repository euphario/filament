//! Rules Engine for Dynamic Driver Spawning
//!
//! When a port is registered with a specific type, check rules to determine
//! if a driver should be auto-spawned. This enables composable driver stacks
//! where each layer spawns the next.
//!
//! ## Example Flow
//!
//! 1. usbd registers `disk0:` with type Block
//! 2. Rules engine matches Block → partd driver
//! 3. devd spawns partd driver with `disk0:` as trigger
//! 4. partd registers `part0:`, `part1:` with type Partition, parent=disk0:
//! 5. Rules engine matches Partition → fatfsd driver
//! 6. devd spawns fatfsd for each partd

use crate::ports::PortType;

// =============================================================================
// Rule Definition
// =============================================================================

/// A rule for auto-spawning drivers
#[derive(Clone, Copy)]
pub struct Rule {
    /// Match ports with this parent type (None = any parent)
    pub match_parent_type: Option<PortType>,
    /// Match ports with this type (None = any type)
    pub match_port_type: Option<PortType>,
    /// Driver binary to spawn
    pub driver_binary: &'static str,
    /// Should the driver receive the port name as an argument?
    pub pass_port_name: bool,
    /// Capability bits for spawned child (0 = inherit parent's caps)
    pub caps: u64,
}

impl Rule {
    /// Check if this rule matches a port
    pub fn matches(&self, port_type: PortType, parent_type: Option<PortType>) -> bool {
        // Check port type if specified
        if let Some(required_type) = self.match_port_type {
            if port_type != required_type {
                return false;
            }
        }

        // Check parent type if specified
        if let Some(required_parent) = self.match_parent_type {
            match parent_type {
                Some(pt) if pt == required_parent => {}
                _ => return false,
            }
        }

        true
    }
}

// =============================================================================
// Built-in Rules
// =============================================================================

/// Static rules for driver auto-spawning
///
/// Order matters - first matching rule wins.
pub static RULES: &[Rule] = &[
    // xhcid branch disabled — testing pcied → nvmed → partd → fatfsd path only
    // Rule {
    //     match_parent_type: None,
    //     match_port_type: Some(PortType::Usb),
    //     driver_binary: "xhcid",
    //     pass_port_name: true,
    //     caps: userlib::devd::caps::DRIVER,
    // },
    // When a Storage controller appears (NVMe, AHCI), spawn nvmed
    Rule {
        match_parent_type: None,
        match_port_type: Some(PortType::Storage),
        driver_binary: "nvmed",
        pass_port_name: true,
        caps: userlib::devd::caps::DRIVER,
    },
    // When a Network controller appears (virtio-net, etc.), spawn netd
    Rule {
        match_parent_type: None,
        match_port_type: Some(PortType::Network),
        driver_binary: "netd",
        pass_port_name: true,
        caps: userlib::devd::caps::DRIVER,
    },
    // When a Block device appears, spawn partd scanner
    Rule {
        match_parent_type: None,
        match_port_type: Some(PortType::Block),
        driver_binary: "partd",
        pass_port_name: true,
        caps: userlib::devd::caps::DRIVER,
    },
    // When a Partition appears with a Block parent, spawn fatfsd
    // (fatfsd will probe and only attach if it recognizes the filesystem)
    Rule {
        match_parent_type: Some(PortType::Block),
        match_port_type: Some(PortType::Partition),
        driver_binary: "fatfsd",
        pass_port_name: true,
        caps: userlib::devd::caps::DRIVER,
    },
];

// =============================================================================
// Rules Engine Trait
// =============================================================================

/// Trait for rules-based driver spawning
pub trait RulesEngine {
    /// Find matching rule for a port
    fn find_matching_rule(
        &self,
        port_type: PortType,
        parent_type: Option<PortType>,
    ) -> Option<&'static Rule>;

    /// Get all rules
    fn rules(&self) -> &'static [Rule];
}

// =============================================================================
// Static Rules Implementation
// =============================================================================

/// Rules engine using static RULES array
pub struct StaticRules;

impl StaticRules {
    pub const fn new() -> Self {
        Self
    }
}

impl RulesEngine for StaticRules {
    fn find_matching_rule(
        &self,
        port_type: PortType,
        parent_type: Option<PortType>,
    ) -> Option<&'static Rule> {
        for rule in RULES {
            if rule.matches(port_type, parent_type) {
                return Some(rule);
            }
        }
        None
    }

    fn rules(&self) -> &'static [Rule] {
        RULES
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rule_matches_port_type() {
        let rule = Rule {
            match_parent_type: None,
            match_port_type: Some(PortType::Block),
            driver_binary: "partd",
            pass_port_name: true,
            caps: 0,
        };

        assert!(rule.matches(PortType::Block, None));
        assert!(rule.matches(PortType::Block, Some(PortType::Usb)));
        assert!(!rule.matches(PortType::Partition, None));
    }

    #[test]
    fn test_rule_matches_parent_type() {
        let rule = Rule {
            match_parent_type: Some(PortType::Block),
            match_port_type: Some(PortType::Partition),
            driver_binary: "fatfsd",
            pass_port_name: true,
            caps: 0,
        };

        assert!(rule.matches(PortType::Partition, Some(PortType::Block)));
        assert!(!rule.matches(PortType::Partition, Some(PortType::Usb)));
        assert!(!rule.matches(PortType::Partition, None));
        assert!(!rule.matches(PortType::Block, Some(PortType::Block)));
    }

    #[test]
    fn test_static_rules_find() {
        let engine = StaticRules::new();

        // Block device should match partd rule
        let rule = engine.find_matching_rule(PortType::Block, None);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver_binary, "partd");

        // Partition with Block parent should match fatfsd rule
        let rule = engine.find_matching_rule(PortType::Partition, Some(PortType::Block));
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver_binary, "fatfsd");

        // Console should not match any rule
        let rule = engine.find_matching_rule(PortType::Console, None);
        assert!(rule.is_none());
    }
}
