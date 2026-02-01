//! Rules Engine for Dynamic Driver Spawning
//!
//! When a port is registered, its path suffix determines which driver to spawn.
//! Paths are built automatically by walking the port parent chain:
//!
//!   pcie0/00:02.0:xhci → spawn usbd
//!   pcie0/00:02.0/usb0:msc → spawn partd
//!   pcie0/00:02.0/usb0:msc/0:fat → spawn fatfsd
//!
//! The last segment's suffix (after ':') carries the type tag.
//! Longest matching suffix wins.

// =============================================================================
// Path-Based Rule
// =============================================================================

/// A rule matching path suffixes to driver binaries.
///
/// The suffix is matched against the last segment of the port's full path.
/// For example, suffix ":xhci" matches segment "00:02.0:xhci".
#[derive(Clone, Copy)]
pub struct PathRule {
    /// Suffix to match against the last path segment (e.g., ":fat", ":xhci")
    pub suffix: &'static str,
    /// Driver binary to spawn
    pub driver: &'static str,
    /// Capability bits for spawned child (0 = inherit parent's caps)
    pub caps: u64,
}

impl PathRule {
    /// Check if a path segment's suffix matches this rule.
    pub fn matches_segment(&self, segment: &[u8]) -> bool {
        let suffix = self.suffix.as_bytes();
        if segment.len() < suffix.len() {
            return false;
        }
        &segment[segment.len() - suffix.len()..] == suffix
    }
}

// =============================================================================
// Built-in Path Rules
// =============================================================================

/// Path-based rules for driver auto-spawning.
///
/// Matching is done against the last segment of the port's full path.
/// All rules are checked; the longest matching suffix wins.
pub static PATH_RULES: &[PathRule] = &[
    // PCI device type matches
    PathRule { suffix: ":xhci",  driver: "usbd",   caps: userlib::devd::caps::DRIVER },
    PathRule { suffix: ":nvme",  driver: "nvmed",  caps: userlib::devd::caps::DRIVER },
    PathRule { suffix: ":network", driver: "netd", caps: userlib::devd::caps::DRIVER },
    // Block device type → partition scanner
    PathRule { suffix: ":msc",   driver: "partd",  caps: userlib::devd::caps::DRIVER },
    // Filesystem type matches
    PathRule { suffix: ":fat",   driver: "fatfsd", caps: userlib::devd::caps::DRIVER },
    PathRule { suffix: ":ext2",  driver: "ext2fsd", caps: userlib::devd::caps::DRIVER },
];

/// Find the best matching path rule for a segment (longest suffix wins).
pub fn find_path_rule(segment: &[u8]) -> Option<&'static PathRule> {
    let mut best: Option<&PathRule> = None;
    let mut best_len = 0;
    for rule in PATH_RULES {
        if rule.matches_segment(segment) && rule.suffix.len() > best_len {
            best = Some(rule);
            best_len = rule.suffix.len();
        }
    }
    best
}

// =============================================================================
// Legacy Rule (kept for PortType references in ports.rs)
// =============================================================================

use crate::ports::PortType;

/// Legacy rule struct — kept temporarily while PortType is still used
/// in port registration. Will be removed once all drivers use path-based names.
#[derive(Clone, Copy)]
pub struct Rule {
    pub match_parent_type: Option<PortType>,
    pub match_port_type: Option<PortType>,
    pub driver_binary: &'static str,
    pub pass_port_name: bool,
    pub caps: u64,
}

impl Rule {
    pub fn matches(&self, port_type: PortType, parent_type: Option<PortType>) -> bool {
        if let Some(required_type) = self.match_port_type {
            if port_type != required_type {
                return false;
            }
        }
        if let Some(required_parent) = self.match_parent_type {
            match parent_type {
                Some(pt) if pt == required_parent => {}
                _ => return false,
            }
        }
        true
    }
}

/// Legacy static rules — kept temporarily for backward compat.
pub static RULES: &[Rule] = &[
    Rule {
        match_parent_type: None,
        match_port_type: Some(PortType::Usb),
        driver_binary: "usbd",
        pass_port_name: true,
        caps: userlib::devd::caps::DRIVER,
    },
    Rule {
        match_parent_type: None,
        match_port_type: Some(PortType::Storage),
        driver_binary: "nvmed",
        pass_port_name: true,
        caps: userlib::devd::caps::DRIVER,
    },
    Rule {
        match_parent_type: None,
        match_port_type: Some(PortType::Network),
        driver_binary: "netd",
        pass_port_name: true,
        caps: userlib::devd::caps::DRIVER,
    },
    Rule {
        match_parent_type: None,
        match_port_type: Some(PortType::Block),
        driver_binary: "partd",
        pass_port_name: true,
        caps: userlib::devd::caps::DRIVER,
    },
    Rule {
        match_parent_type: Some(PortType::Block),
        match_port_type: Some(PortType::Partition),
        driver_binary: "fatfsd",
        pass_port_name: true,
        caps: userlib::devd::caps::DRIVER,
    },
];

pub trait RulesEngine {
    fn find_matching_rule(
        &self,
        port_type: PortType,
        parent_type: Option<PortType>,
    ) -> Option<&'static Rule>;

    fn rules(&self) -> &'static [Rule];
}

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
    fn test_path_rule_matches_suffix() {
        let rule = PathRule {
            suffix: ":xhci",
            driver: "usbd",
            caps: 0,
        };

        assert!(rule.matches_segment(b"00:02.0:xhci"));
        assert!(rule.matches_segment(b":xhci"));
        assert!(!rule.matches_segment(b"xhci")); // no colon prefix
        assert!(!rule.matches_segment(b"00:02.0:nvme"));
    }

    #[test]
    fn test_path_rule_matches_fat() {
        let rule = PathRule {
            suffix: ":fat",
            driver: "fatfsd",
            caps: 0,
        };

        assert!(rule.matches_segment(b"0:fat"));
        assert!(rule.matches_segment(b"1:fat"));
        assert!(!rule.matches_segment(b"0:ext2"));
    }

    #[test]
    fn test_find_path_rule_longest_match() {
        // ":msc" should match for "usb0:msc"
        let rule = find_path_rule(b"usb0:msc");
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "partd");

        // ":fat" should match
        let rule = find_path_rule(b"0:fat");
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "fatfsd");

        // ":network" should match netd
        let rule = find_path_rule(b"00:01.0:network");
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "netd");

        // No match for console
        let rule = find_path_rule(b"console");
        assert!(rule.is_none());
    }

    #[test]
    fn test_legacy_rule_matches_port_type() {
        let rule = Rule {
            match_parent_type: None,
            match_port_type: Some(PortType::Block),
            driver_binary: "partd",
            pass_port_name: true,
            caps: 0,
        };

        assert!(rule.matches(PortType::Block, None));
        assert!(!rule.matches(PortType::Partition, None));
    }
}
