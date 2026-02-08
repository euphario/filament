//! Unified Port Rules Engine
//!
//! Single rule table for all driver auto-spawning. Rules fire when a port
//! transitions to Ready — never at registration time.
//!
//! Hardware-layer rules (kernel bus ports → bus drivers):
//!   PortClass::Pcie → pcied
//!   PortClass::Uart → consoled
//!   PortClass::Klog → logd
//!   PortClass::Ethernet → ethd
//!
//! Driver-layer rules (driver ports → children):
//!   PortClass::Console → shell
//!   PortClass::Usb + USB_XHCI → usbd
//!   PortClass::Block + BLOCK_FAT* → fatfsd
//!
//! Singletons (spawned by orchestration, not rules):
//!   partd — spawned on first block device discovery

use abi::{PortClass, PortInfo, port_subclass};

// =============================================================================
// Port Rule
// =============================================================================

/// Subclass matching strategy
#[derive(Clone, Copy)]
pub enum SubclassMatch {
    /// Match any subclass value
    Any,
    /// Match exact subclass value
    Exact(u16),
    /// Match one of several subclass values
    OneOf(&'static [u16]),
}

impl SubclassMatch {
    /// Check if a subclass value matches this matcher
    pub fn matches(&self, subclass: u16) -> bool {
        match self {
            SubclassMatch::Any => true,
            SubclassMatch::Exact(v) => subclass == *v,
            SubclassMatch::OneOf(values) => values.contains(&subclass),
        }
    }
}

/// A rule matching PortClass + subclass to driver binaries.
#[derive(Clone, Copy)]
pub struct PortRule {
    /// Port class to match
    pub class: PortClass,
    /// Subclass matcher
    pub subclass: SubclassMatch,
    /// Driver binary to spawn
    pub driver: &'static str,
    /// Capability bits for spawned child
    pub caps: u64,
}

impl PortRule {
    /// Check if a PortInfo matches this rule
    pub fn matches(&self, info: &PortInfo) -> bool {
        if info.port_class != self.class {
            return false;
        }
        self.subclass.matches(info.port_subclass)
    }
}

// =============================================================================
// Unified Port Rules Table
// =============================================================================

/// FAT filesystem subclass values (MBR partition type codes)
static FAT_SUBCLASSES: &[u16] = &[
    port_subclass::BLOCK_FAT12,
    port_subclass::BLOCK_FAT16,
    port_subclass::BLOCK_FAT32,
    port_subclass::BLOCK_FAT32_LBA,
];

/// Unified port rules for all driver auto-spawning.
///
/// Rules are checked in order; first match wins. This table replaces
/// the former BUS_DRIVER_RULES (hardware layer) and CLASS_RULES (driver layer).
pub static PORT_RULES: &[PortRule] = &[
    // =========================================================================
    // Hardware layer (kernel bus ports → bus drivers)
    // =========================================================================
    PortRule {
        class: PortClass::Pcie,
        subclass: SubclassMatch::Any,
        driver: "pcied",
        caps: userlib::devd::caps::DRIVER,
    },
    PortRule {
        class: PortClass::Usb,
        subclass: SubclassMatch::Exact(port_subclass::USB_XHCI),
        driver: "usbd",
        caps: userlib::devd::caps::DRIVER,
    },
    PortRule {
        class: PortClass::Ethernet,
        subclass: SubclassMatch::Any,
        driver: "ethd",
        caps: userlib::devd::caps::DRIVER,
    },
    PortRule {
        class: PortClass::Uart,
        subclass: SubclassMatch::Any,
        driver: "consoled",
        caps: userlib::devd::caps::DRIVER,
    },
    PortRule {
        class: PortClass::Klog,
        subclass: SubclassMatch::Any,
        driver: "logd",
        caps: userlib::devd::caps::DRIVER,
    },

    // =========================================================================
    // Driver layer (driver ports → children)
    // =========================================================================
    PortRule {
        class: PortClass::Console,
        subclass: SubclassMatch::Any,
        driver: "shell",
        caps: userlib::devd::caps::USER,
    },
    PortRule {
        class: PortClass::StorageController,
        subclass: SubclassMatch::Exact(port_subclass::STORAGE_NVME),
        driver: "nvmed",
        caps: userlib::devd::caps::DRIVER,
    },
    PortRule {
        class: PortClass::Network,
        subclass: SubclassMatch::Exact(port_subclass::NET_SWITCH),
        driver: "switchd",
        caps: userlib::devd::caps::DRIVER,
    },
    PortRule {
        class: PortClass::Network,
        subclass: SubclassMatch::Exact(port_subclass::NET_SWITCH_PORT),
        driver: "ipd",
        caps: userlib::devd::caps::DRIVER,
    },
    PortRule {
        class: PortClass::Network,
        subclass: SubclassMatch::Exact(port_subclass::NET_BRIDGE_GROUP),
        driver: "ipd",
        caps: userlib::devd::caps::DRIVER,
    },
    PortRule {
        class: PortClass::Network,
        subclass: SubclassMatch::Exact(port_subclass::NET_ETHERNET),
        driver: "ipd",
        caps: userlib::devd::caps::DRIVER,
    },
    PortRule {
        class: PortClass::Network,
        subclass: SubclassMatch::Exact(port_subclass::NET_WIFI),
        driver: "wifid",
        caps: userlib::devd::caps::DRIVER,
    },
    // Block(BLOCK_RAW) → partd is NOT here.  partd is a singleton spawned by
    // disk orchestration on first block device discovery (track_and_attach_disk).
    PortRule {
        class: PortClass::Block,
        subclass: SubclassMatch::OneOf(FAT_SUBCLASSES),
        driver: "fatfsd",
        caps: userlib::devd::caps::DRIVER,
    },
    PortRule {
        class: PortClass::Block,
        subclass: SubclassMatch::Exact(port_subclass::BLOCK_LINUX),
        driver: "ext2fsd",
        caps: userlib::devd::caps::DRIVER,
    },
];

/// Find the first matching port rule for a PortInfo.
pub fn find_port_rule(info: &PortInfo) -> Option<&'static PortRule> {
    PORT_RULES.iter().find(|rule| rule.matches(info))
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_port_info(class: PortClass, subclass: u16) -> PortInfo {
        let mut info = PortInfo::empty();
        info.port_class = class;
        info.port_subclass = subclass;
        info
    }

    #[test]
    fn test_subclass_match_any() {
        let matcher = SubclassMatch::Any;
        assert!(matcher.matches(0));
        assert!(matcher.matches(0x30));
        assert!(matcher.matches(0xFFFF));
    }

    #[test]
    fn test_subclass_match_exact() {
        let matcher = SubclassMatch::Exact(0x30);
        assert!(matcher.matches(0x30));
        assert!(!matcher.matches(0x00));
        assert!(!matcher.matches(0x08));
    }

    #[test]
    fn test_subclass_match_one_of() {
        let values: &[u16] = &[0x01, 0x06, 0x0b, 0x0c];
        let matcher = SubclassMatch::OneOf(values);
        assert!(matcher.matches(0x01));
        assert!(matcher.matches(0x06));
        assert!(matcher.matches(0x0b));
        assert!(matcher.matches(0x0c));
        assert!(!matcher.matches(0x00));
        assert!(!matcher.matches(0x83));
    }

    // Hardware-layer rules
    #[test]
    fn test_rule_pcie() {
        let info = make_port_info(PortClass::Pcie, 0);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "pcied");
    }

    #[test]
    fn test_rule_uart() {
        let info = make_port_info(PortClass::Uart, 0);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "consoled");
    }

    #[test]
    fn test_rule_klog() {
        let info = make_port_info(PortClass::Klog, 0);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "logd");
    }

    #[test]
    fn test_rule_ethernet() {
        let info = make_port_info(PortClass::Ethernet, 0);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "ethd");
    }

    // Driver-layer rules
    #[test]
    fn test_rule_console() {
        let info = make_port_info(PortClass::Console, 0);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "shell");
    }

    #[test]
    fn test_rule_usb_xhci() {
        let info = make_port_info(PortClass::Usb, port_subclass::USB_XHCI);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "usbd");
    }

    #[test]
    fn test_rule_nvme() {
        let info = make_port_info(PortClass::StorageController, port_subclass::STORAGE_NVME);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "nvmed");
    }

    #[test]
    fn test_rule_network_ethernet() {
        let info = make_port_info(PortClass::Network, port_subclass::NET_ETHERNET);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "ipd");
    }

    #[test]
    fn test_rule_network_wifi() {
        let info = make_port_info(PortClass::Network, port_subclass::NET_WIFI);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "wifid");
    }

    #[test]
    fn test_rule_network_switch() {
        let info = make_port_info(PortClass::Network, port_subclass::NET_SWITCH);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "switchd");
    }

    #[test]
    fn test_rule_network_switch_port() {
        let info = make_port_info(PortClass::Network, port_subclass::NET_SWITCH_PORT);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "ipd");
    }

    #[test]
    fn test_rule_network_bridge_group() {
        let info = make_port_info(PortClass::Network, port_subclass::NET_BRIDGE_GROUP);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "ipd");
    }

    #[test]
    fn test_rule_block_raw_no_match() {
        // BLOCK_RAW doesn't match any rule — partd is a singleton
        // spawned by disk orchestration, not by port rules.
        let info = make_port_info(PortClass::Block, port_subclass::BLOCK_RAW);
        let rule = find_port_rule(&info);
        assert!(rule.is_none());
    }

    #[test]
    fn test_rule_fat_partitions() {
        for &sub in FAT_SUBCLASSES {
            let info = make_port_info(PortClass::Block, sub);
            let rule = find_port_rule(&info);
            assert!(rule.is_some());
            assert_eq!(rule.unwrap().driver, "fatfsd");
        }
    }

    #[test]
    fn test_rule_linux_partition() {
        let info = make_port_info(PortClass::Block, port_subclass::BLOCK_LINUX);
        let rule = find_port_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "ext2fsd");
    }

    #[test]
    fn test_rule_no_match() {
        // Service should not match any rule
        let info = make_port_info(PortClass::Service, 0);
        let rule = find_port_rule(&info);
        assert!(rule.is_none());
    }
}
