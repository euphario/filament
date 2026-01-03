//! PCIe MAC Register Definitions
//!
//! Based on MediaTek pcie-mediatek-gen3.c driver.

/// Reset control register offset (from pcie-mediatek-gen3.c)
pub const PCIE_RST_CTRL_REG: usize = 0x148;

/// Reset control bits (active high - set to assert reset)
pub mod rst_ctrl {
    /// MAC reset
    pub const PCIE_MAC_RSTB: u32 = 1 << 0;
    /// PHY reset
    pub const PCIE_PHY_RSTB: u32 = 1 << 1;
    /// Bridge reset
    pub const PCIE_BRG_RSTB: u32 = 1 << 2;
    /// PERST# (Fundamental Reset to endpoint)
    pub const PCIE_PE_RSTB: u32 = 1 << 3;

    /// All reset bits combined
    pub const ALL: u32 = PCIE_MAC_RSTB | PCIE_PHY_RSTB | PCIE_BRG_RSTB | PCIE_PE_RSTB;

    /// Resets to de-assert in first stage (everything except PERST#)
    pub const STAGE1: u32 = PCIE_MAC_RSTB | PCIE_PHY_RSTB | PCIE_BRG_RSTB;
}

/// LTSSM status register offset
pub const PCIE_LTSSM_STATUS_REG: usize = 0x150;

/// LTSSM status bits
pub mod ltssm {
    /// LTSSM state mask (bits 28:24)
    pub const STATE_MASK: u32 = 0x1F << 24;
    pub const STATE_SHIFT: u32 = 24;

    // LTSSM states
    pub const DETECT_QUIET: u32 = 0x00;
    pub const DETECT_ACTIVE: u32 = 0x01;
    pub const POLLING_ACTIVE: u32 = 0x02;
    pub const L0: u32 = 0x10;  // Link up and operational
}

/// Link status register offset
pub const PCIE_LINK_STATUS_REG: usize = 0x154;

/// Link status bits
pub mod link_status {
    /// Link training complete, link is up (bit 8)
    pub const PCIE_PORT_LINKUP: u32 = 1 << 8;

    /// Link speed mask (bits 19:16)
    pub const SPEED_MASK: u32 = 0xF << 16;
    pub const SPEED_SHIFT: u32 = 16;

    /// Link width mask (bits 25:20)
    pub const WIDTH_MASK: u32 = 0x3F << 20;
    pub const WIDTH_SHIFT: u32 = 20;
}

/// MISC control register (from pcie-mediatek-gen3.c)
pub const PCIE_MISC_CTRL_REG: usize = 0x348;

/// MISC control bits
pub mod misc_ctrl {
    /// Disable DVFSRC voltage request
    pub const DISABLE_DVFSRC_VLT_REQ: u32 = 1 << 1;
}

/// Configuration space access registers (MediaTek specific)
/// Config space is accessed via TLP mechanism, not standard ECAM
pub const PCIE_CFGNUM_REG: usize = 0x140;
pub const PCIE_CFG_OFFSET_ADDR: usize = 0x1000;

/// TLP header configuration bits (CFGNUM_REG at 0x140)
pub mod cfgnum {
    /// Device/Function field (bits 7:0) - devfn = device << 3 | function
    pub const DEVFN_MASK: u32 = 0xFF;
    /// Bus number field (bits 15:8)
    pub const BUS_SHIFT: u32 = 8;
    pub const BUS_MASK: u32 = 0xFF << 8;
    /// Byte enable field (bits 19:16)
    pub const BYTE_EN_SHIFT: u32 = 16;
    pub const BYTE_EN_MASK: u32 = 0xF << 16;
    /// Force byte enable bit (bit 20)
    pub const FORCE_BYTE_EN: u32 = 1 << 20;
}

/// PCIe configuration space register offsets (standard PCI)
pub mod cfg {
    /// Vendor ID (16-bit)
    pub const VENDOR_ID: u16 = 0x00;
    /// Device ID (16-bit)
    pub const DEVICE_ID: u16 = 0x02;
    /// Command register (16-bit)
    pub const COMMAND: u16 = 0x04;
    /// Status register (16-bit)
    pub const STATUS: u16 = 0x06;
    /// Revision ID (8-bit)
    pub const REVISION_ID: u16 = 0x08;
    /// Class code (24-bit, at offset 0x09)
    pub const CLASS_CODE: u16 = 0x09;
    /// Cache line size (8-bit)
    pub const CACHE_LINE_SIZE: u16 = 0x0C;
    /// Latency timer (8-bit)
    pub const LATENCY_TIMER: u16 = 0x0D;
    /// Header type (8-bit)
    pub const HEADER_TYPE: u16 = 0x0E;
    /// BIST (8-bit)
    pub const BIST: u16 = 0x0F;

    /// Base Address Registers
    pub const BAR0: u16 = 0x10;
    pub const BAR1: u16 = 0x14;
    pub const BAR2: u16 = 0x18;
    pub const BAR3: u16 = 0x1C;
    pub const BAR4: u16 = 0x20;
    pub const BAR5: u16 = 0x24;

    /// Subsystem Vendor ID (16-bit)
    pub const SUBSYS_VENDOR_ID: u16 = 0x2C;
    /// Subsystem ID (16-bit)
    pub const SUBSYS_ID: u16 = 0x2E;

    /// Expansion ROM base
    pub const ROM_BASE: u16 = 0x30;

    /// Capabilities pointer (8-bit)
    pub const CAP_PTR: u16 = 0x34;

    /// Interrupt line (8-bit)
    pub const INT_LINE: u16 = 0x3C;
    /// Interrupt pin (8-bit)
    pub const INT_PIN: u16 = 0x3D;

    // Type 1 (Bridge) specific registers
    /// Primary bus number
    pub const PRIMARY_BUS: u16 = 0x18;
    /// Secondary bus number
    pub const SECONDARY_BUS: u16 = 0x19;
    /// Subordinate bus number
    pub const SUBORDINATE_BUS: u16 = 0x1A;
    /// Memory base (bits 15:4 = address bits 31:20)
    pub const MEMORY_BASE: u16 = 0x20;
    /// Memory limit (bits 15:4 = address bits 31:20)
    pub const MEMORY_LIMIT: u16 = 0x22;
    /// Prefetchable memory base
    pub const PREF_MEMORY_BASE: u16 = 0x24;
    /// Prefetchable memory limit
    pub const PREF_MEMORY_LIMIT: u16 = 0x26;
}

/// Command register bits
pub mod command {
    /// I/O space enable
    pub const IO_SPACE: u16 = 1 << 0;
    /// Memory space enable
    pub const MEM_SPACE: u16 = 1 << 1;
    /// Bus master enable
    pub const BUS_MASTER: u16 = 1 << 2;
    /// Interrupt disable
    pub const INT_DISABLE: u16 = 1 << 10;
}

/// PCI Capability IDs
pub mod cap_id {
    /// Power Management
    pub const PM: u8 = 0x01;
    /// AGP
    pub const AGP: u8 = 0x02;
    /// MSI
    pub const MSI: u8 = 0x05;
    /// PCI Express
    pub const PCIE: u8 = 0x10;
    /// MSI-X
    pub const MSIX: u8 = 0x11;
}

/// PCIe Express Capability offsets (from capability base)
pub mod pcie_cap {
    /// Capability ID and Next pointer (8-bit each)
    pub const CAP_ID: u16 = 0x00;
    pub const CAP_NEXT: u16 = 0x01;
    /// PCIe Capabilities Register (16-bit)
    pub const PCIE_CAP: u16 = 0x02;
    /// Device Capabilities (32-bit)
    pub const DEV_CAP: u16 = 0x04;
    /// Device Control (16-bit)
    pub const DEV_CTL: u16 = 0x08;
    /// Device Status (16-bit)
    pub const DEV_STA: u16 = 0x0A;
    /// Link Capabilities (32-bit)
    pub const LINK_CAP: u16 = 0x0C;
    /// Link Control (16-bit)
    pub const LINK_CTL: u16 = 0x10;
    /// Link Status (16-bit)
    pub const LINK_STA: u16 = 0x12;
}

/// Link Control register bits
pub mod link_ctl {
    /// ASPM L0s Enable
    pub const ASPM_L0S: u16 = 1 << 0;
    /// ASPM L1 Enable
    pub const ASPM_L1: u16 = 1 << 1;
    /// Both ASPM states
    pub const ASPM_MASK: u16 = ASPM_L0S | ASPM_L1;
}

/// Header type values
pub mod header_type {
    /// Multi-function device bit
    pub const MULTI_FUNC: u8 = 0x80;
    /// Header type mask
    pub const TYPE_MASK: u8 = 0x7F;
    /// Type 0: Endpoint
    pub const TYPE_ENDPOINT: u8 = 0x00;
    /// Type 1: PCI-to-PCI bridge
    pub const TYPE_BRIDGE: u8 = 0x01;
    /// Type 2: CardBus bridge
    pub const TYPE_CARDBUS: u8 = 0x02;
}

/// Address Translation Unit (ATU) registers
/// Programs outbound CPU-to-PCIe address translation
pub const PCIE_TRANS_TABLE_BASE_REG: usize = 0x800;

/// ATU register offsets within each translation table entry
pub mod atu {
    /// Source address MSB offset (CPU address upper 32 bits)
    pub const SRC_ADDR_MSB_OFFSET: usize = 0x4;
    /// Translation address LSB offset (PCIe address lower 32 bits)
    pub const TRSL_ADDR_LSB_OFFSET: usize = 0x8;
    /// Translation address MSB offset (PCIe address upper 32 bits)
    pub const TRSL_ADDR_MSB_OFFSET: usize = 0xc;
    /// Translation parameters offset (size encoding)
    pub const TRSL_PARAM_OFFSET: usize = 0x10;
    /// Spacing between translation table entries
    pub const TLB_SET_OFFSET: usize = 0x20;

    /// Maximum number of translation tables
    pub const MAX_TRANS_TABLES: usize = 8;

    /// ATU type bits (in source address LSB register)
    pub const TYPE_MEM: u32 = 0x0;  // Memory type
    pub const TYPE_IO: u32 = 0x1;   // I/O type

    /// Minimum ATU window size (4KB)
    pub const MIN_SIZE: u64 = 0x1000;
}
