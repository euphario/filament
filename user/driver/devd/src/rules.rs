//! Rules Engine for Dynamic Driver Spawning
//!
//! When a port is registered with PortInfo, its class and subclass determine
//! which driver to spawn. This replaces the legacy string-based suffix matching.
//!
//! Class-based matching:
//!   PortClass::Usb + USB_XHCI → spawn usbd
//!   PortClass::Block + BLOCK_RAW → spawn partd
//!   PortClass::Block + BLOCK_FAT* → spawn fatfsd

use abi::{PortClass, PortInfo, port_subclass};

// =============================================================================
// Class-Based Rule
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
pub struct ClassRule {
    /// Port class to match
    pub class: PortClass,
    /// Subclass matcher
    pub subclass: SubclassMatch,
    /// Driver binary to spawn
    pub driver: &'static str,
    /// Capability bits for spawned child (0 = inherit parent's caps)
    pub caps: u64,
}

impl ClassRule {
    /// Check if a PortInfo matches this rule
    pub fn matches(&self, info: &PortInfo) -> bool {
        if info.port_class != self.class {
            return false;
        }
        self.subclass.matches(info.port_subclass)
    }
}

// =============================================================================
// Built-in Class Rules
// =============================================================================

/// FAT filesystem subclass values (MBR partition type codes)
static FAT_SUBCLASSES: &[u16] = &[
    port_subclass::BLOCK_FAT12,
    port_subclass::BLOCK_FAT16,
    port_subclass::BLOCK_FAT32,
    port_subclass::BLOCK_FAT32_LBA,
];

/// Class-based rules for driver auto-spawning.
///
/// Rules are checked in order; first match wins.
pub static CLASS_RULES: &[ClassRule] = &[
    // Console port → spawn shell
    ClassRule {
        class: PortClass::Console,
        subclass: SubclassMatch::Any,
        driver: "shell",
        caps: userlib::devd::caps::USER,
    },
    // USB xHCI controller → spawn USB daemon
    ClassRule {
        class: PortClass::Usb,
        subclass: SubclassMatch::Exact(port_subclass::USB_XHCI),
        driver: "usbd",
        caps: userlib::devd::caps::DRIVER,
    },
    // NVMe storage controller → spawn NVMe daemon
    ClassRule {
        class: PortClass::StorageController,
        subclass: SubclassMatch::Exact(port_subclass::STORAGE_NVME),
        driver: "nvmed",
        caps: userlib::devd::caps::DRIVER,
    },
    // L2 switch → spawn switch manager
    ClassRule {
        class: PortClass::Network,
        subclass: SubclassMatch::Exact(port_subclass::NET_SWITCH),
        driver: "switchd",
        caps: userlib::devd::caps::DRIVER,
    },
    // Switch port (NIC) → spawn IP stack
    ClassRule {
        class: PortClass::Network,
        subclass: SubclassMatch::Exact(port_subclass::NET_SWITCH_PORT),
        driver: "ipd",
        caps: userlib::devd::caps::DRIVER,
    },
    // Ethernet NIC → spawn IP stack
    ClassRule {
        class: PortClass::Network,
        subclass: SubclassMatch::Exact(port_subclass::NET_ETHERNET),
        driver: "ipd",
        caps: userlib::devd::caps::DRIVER,
    },
    // WiFi adapter → spawn WiFi daemon
    ClassRule {
        class: PortClass::Network,
        subclass: SubclassMatch::Exact(port_subclass::NET_WIFI),
        driver: "wifid",
        caps: userlib::devd::caps::DRIVER,
    },
    // Raw block device (whole disk) → spawn partition scanner
    ClassRule {
        class: PortClass::Block,
        subclass: SubclassMatch::Exact(port_subclass::BLOCK_RAW),
        driver: "partd",
        caps: userlib::devd::caps::DRIVER,
    },
    // FAT partition → spawn FAT filesystem driver
    ClassRule {
        class: PortClass::Block,
        subclass: SubclassMatch::OneOf(FAT_SUBCLASSES),
        driver: "fatfsd",
        caps: userlib::devd::caps::DRIVER,
    },
    // Linux partition → spawn ext2 filesystem driver
    ClassRule {
        class: PortClass::Block,
        subclass: SubclassMatch::Exact(port_subclass::BLOCK_LINUX),
        driver: "ext2fsd",
        caps: userlib::devd::caps::DRIVER,
    },
];

/// Find the first matching class rule for a PortInfo.
pub fn find_class_rule(info: &PortInfo) -> Option<&'static ClassRule> {
    CLASS_RULES.iter().find(|rule| rule.matches(info))
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

    #[test]
    fn test_class_rule_usb_xhci() {
        let info = make_port_info(PortClass::Usb, port_subclass::USB_XHCI);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "usbd");
    }

    #[test]
    fn test_class_rule_nvme() {
        let info = make_port_info(PortClass::StorageController, port_subclass::STORAGE_NVME);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "nvmed");
    }

    #[test]
    fn test_class_rule_network_ethernet() {
        // NET_ETHERNET (0x00) should match ipd
        let info = make_port_info(PortClass::Network, port_subclass::NET_ETHERNET);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "ipd");
    }

    #[test]
    fn test_class_rule_network_wifi() {
        // NET_WIFI (0x01) should match wifid
        let info = make_port_info(PortClass::Network, port_subclass::NET_WIFI);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "wifid");
    }

    #[test]
    fn test_class_rule_network_switch() {
        // NET_SWITCH (0x10) should match switchd
        let info = make_port_info(PortClass::Network, port_subclass::NET_SWITCH);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "switchd");
    }

    #[test]
    fn test_class_rule_network_switch_port() {
        // NET_SWITCH_PORT (0x02) should match ipd
        let info = make_port_info(PortClass::Network, port_subclass::NET_SWITCH_PORT);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "ipd");
    }

    #[test]
    fn test_class_rule_block_raw() {
        let info = make_port_info(PortClass::Block, port_subclass::BLOCK_RAW);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "partd");
    }

    #[test]
    fn test_class_rule_fat_partitions() {
        // FAT12
        let info = make_port_info(PortClass::Block, port_subclass::BLOCK_FAT12);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "fatfsd");

        // FAT16
        let info = make_port_info(PortClass::Block, port_subclass::BLOCK_FAT16);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "fatfsd");

        // FAT32
        let info = make_port_info(PortClass::Block, port_subclass::BLOCK_FAT32);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "fatfsd");

        // FAT32 LBA
        let info = make_port_info(PortClass::Block, port_subclass::BLOCK_FAT32_LBA);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "fatfsd");
    }

    #[test]
    fn test_class_rule_linux_partition() {
        let info = make_port_info(PortClass::Block, port_subclass::BLOCK_LINUX);
        let rule = find_class_rule(&info);
        assert!(rule.is_some());
        assert_eq!(rule.unwrap().driver, "ext2fsd");
    }

    #[test]
    fn test_class_rule_no_match() {
        // Console should not match any rule
        let info = make_port_info(PortClass::Console, 0);
        let rule = find_class_rule(&info);
        assert!(rule.is_none());

        // Service should not match any rule
        let info = make_port_info(PortClass::Service, 0);
        let rule = find_class_rule(&info);
        assert!(rule.is_none());
    }
}
