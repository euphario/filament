//! MT7996 Register Definitions
//!
//! Based on Linux mt76/mt7996 driver.

/// Hardware revision register
pub const MT_HW_REV: u32 = 0x70010204;

/// Hardware chip ID register
pub const MT_HW_CHIPID: u32 = 0x70010200;

/// PAD GPIO register (used for variant detection)
pub const MT_PAD_GPIO: u32 = 0x700056f0;

/// Top configuration register
pub const MT_TOP_CFG: u32 = 0x70020000;

/// WFDMA0 base
pub const MT_WFDMA0_BASE: u32 = 0xd4000;

/// HIF2 offset when accessed via HIF1's BAR
/// Linux uses: MT_WFDMA0_PCIE1(0) - MT_WFDMA0(0) = 0xd8000 - 0xd4000 = 0x4000
/// This allows accessing HIF2 registers as HIF1_BAR + HIF1_OFS + register_offset
pub const MT_HIF1_OFS: u32 = 0x4000;

/// WFDMA0 busy status
pub const MT_WFDMA0_BUSY_ENA: u32 = MT_WFDMA0_BASE + 0x13c;

/// PCIe MAC interrupt enable
pub const MT_PCIE_MAC_INT_ENABLE: u32 = 0x74030188;

/// Layer 1 remap base register
pub const MT_INFRA_REMAP_L1_BASE: u32 = 0x700d02f8;

/// Layer 2 remap base register
pub const MT_INFRA_REMAP_L2_BASE: u32 = 0x700d0300;

/// CBTOP remap registers
pub const MT_INFRA_REMAP_CBTOP1_BASE: u32 = 0x0228;
pub const MT_INFRA_REMAP_CBTOP2_BASE: u32 = 0x022c;

/// Remap mask for layer 1
pub const MT_REMAP_L1_MASK: u32 = 0x7fff_ffff;
pub const MT_REMAP_L1_OFFSET: u32 = 18;
pub const MT_REMAP_L1_BASE_MASK: u32 = 0x3ffff;

/// Device IDs
pub mod device_id {
    /// MT7996 primary device
    pub const MT7996: u16 = 0x7990;
    /// MT7996 secondary (HIF2)
    pub const MT7996_2: u16 = 0x7991;
    /// MT7992 primary device
    pub const MT7992: u16 = 0x7992;
    /// MT7992 secondary
    pub const MT7992_2: u16 = 0x799a;
    /// MT7990 primary
    pub const MT7990: u16 = 0x7993;
    /// MT7990 secondary
    pub const MT7990_2: u16 = 0x799b;
}

/// Address ranges for layer 1 remapping
pub mod l1_range {
    /// Infra range start
    pub const INFRA_START: u32 = 0x18000000;
    pub const INFRA_END: u32 = 0x18ffffff;
    /// WFSYS range start
    pub const WFSYS_START: u32 = 0x7c000000;
    pub const WFSYS_END: u32 = 0x7fffffff;
}

/// CBTOP range for PCIe-specific remapping
pub mod cbtop {
    pub const CBTOP1_PHY_START: u32 = 0x70000000;
    pub const CBTOP1_PHY_END: u32 = 0x77ffffff;
    pub const CBTOP2_PHY_START: u32 = 0x78000000;
    pub const CBTOP2_PHY_END: u32 = 0x7fffffff;
}

/// Chip variant types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipVariant {
    /// MT7996 with 4+4+4 antenna configuration
    Mt7996_444,
    /// MT7996 with 2+3+3 antenna configuration
    Mt7996_233,
    /// MT7992 variant
    Mt7992,
    /// MT7990 variant
    Mt7990,
    /// Unknown variant
    Unknown,
}

impl ChipVariant {
    /// Detect variant from device ID
    pub fn from_device_id(device_id: u16) -> Self {
        match device_id {
            device_id::MT7996 | device_id::MT7996_2 => ChipVariant::Mt7996_444,
            device_id::MT7992 | device_id::MT7992_2 => ChipVariant::Mt7992,
            device_id::MT7990 | device_id::MT7990_2 => ChipVariant::Mt7990,
            _ => ChipVariant::Unknown,
        }
    }
}
