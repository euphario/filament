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

    // LTSSM states (from pcie-mediatek-gen3.c)
    pub const DETECT_QUIET: u32 = 0x00;
    pub const DETECT_ACTIVE: u32 = 0x01;
    pub const POLLING_ACTIVE: u32 = 0x02;
    pub const POLLING_COMPLIANCE: u32 = 0x03;
    pub const POLLING_CONFIG: u32 = 0x04;
    pub const CONFIG_LINKWIDTH_START: u32 = 0x05;
    pub const CONFIG_LINKWIDTH_ACCEPT: u32 = 0x06;
    pub const CONFIG_LANENUM_WAIT: u32 = 0x07;
    pub const CONFIG_LANENUM_ACCEPT: u32 = 0x08;
    pub const CONFIG_COMPLETE: u32 = 0x09;
    pub const CONFIG_IDLE: u32 = 0x0A;
    pub const RECOVERY_RCVRLOCK: u32 = 0x0B;
    pub const RECOVERY_EQUALIZATION: u32 = 0x0C;
    pub const RECOVERY_SPEED: u32 = 0x0D;
    pub const RECOVERY_RCVRCONFIG: u32 = 0x0E;
    pub const RECOVERY_IDLE: u32 = 0x0F;
    pub const L0: u32 = 0x10;  // Link up and operational
    pub const L0S: u32 = 0x11;
    pub const L1_ENTRY: u32 = 0x12;
    pub const L1_IDLE: u32 = 0x13;
    pub const L2_IDLE: u32 = 0x14;
    pub const L2_TRANSMIT_WAKE: u32 = 0x15;
    pub const DISABLED: u32 = 0x16;

    /// Get LTSSM state from register value
    #[inline]
    pub const fn get_state(val: u32) -> u32 {
        (val & STATE_MASK) >> STATE_SHIFT
    }

    /// Get state name
    pub fn state_name(state: u32) -> &'static str {
        match state {
            0x00 => "detect.quiet",
            0x01 => "detect.active",
            0x02 => "polling.active",
            0x03 => "polling.compliance",
            0x04 => "polling.config",
            0x05 => "config.linkwidth.start",
            0x06 => "config.linkwidth.accept",
            0x07 => "config.lanenum.wait",
            0x08 => "config.lanenum.accept",
            0x09 => "config.complete",
            0x0A => "config.idle",
            0x0B => "recovery.rcvrlock",
            0x0C => "recovery.equalization",
            0x0D => "recovery.speed",
            0x0E => "recovery.rcvrconfig",
            0x0F => "recovery.idle",
            0x10 => "L0",
            0x11 => "L0s",
            0x12 => "L1.entry",
            0x13 => "L1.idle",
            0x14 => "L2.idle",
            0x15 => "L2.transmit.wake",
            0x16 => "disabled",
            _ => "unknown",
        }
    }

    /// Decode full LTSSM register (for debugging)
    /// Returns (state_name, other_bits_description)
    pub fn decode(val: u32) -> (&'static str, u32) {
        let state = get_state(val);
        let other_bits = val & !STATE_MASK;
        (state_name(state), other_bits)
    }
}

/// Link status register offset
pub const PCIE_LINK_STATUS_REG: usize = 0x154;

/// Link status bits (MAC register 0x154)
///
/// Note: This register only has LINKUP bit. Speed/width are in the
/// PCIe capability Link Status register at config space offset 0x92
/// (accessed via MAC + 0x1092).
pub mod link_status {
    /// Link training complete, link is up (bit 8)
    pub const PCIE_PORT_LINKUP: u32 = 1 << 8;
}

/// PCIE_SETTING_REG - Mode and capability settings (from pcie-mediatek-gen3.c)
pub const PCIE_SETTING_REG: usize = 0x80;

/// PCIE_SETTING_REG bits
pub mod pcie_setting {
    /// Root Complex mode enable (CRITICAL for inbound DMA to work!)
    pub const RC_MODE: u32 = 1 << 0;
    /// Link width setting mask
    pub const LINK_WIDTH_MASK: u32 = 0xF << 8;
    /// Gen support mask (bits 14:12)
    /// For Gen3: set bits 0-1 = 0x3, for Gen2: set bit 0 = 0x1
    pub const GEN_SUPPORT_MASK: u32 = 0x7 << 12;
    /// Gen2 support (bit 12)
    pub const GEN2_SUPPORT: u32 = 0x1 << 12;
    /// Gen3 support (bits 12-13)
    pub const GEN3_SUPPORT: u32 = 0x3 << 12;
}

/// Link Control 2 / Status 2 register (in config space at 0x1000 + 0xb0)
pub const PCIE_CONF_LINK2_CTL_STS: usize = 0x10b0;

/// Link Control 2 bits
pub mod link_ctl2 {
    /// Target link speed mask (bits 3:0)
    pub const TARGET_SPEED_MASK: u32 = 0xF;
    /// Gen1 (2.5 GT/s)
    pub const GEN1: u32 = 1;
    /// Gen2 (5 GT/s)
    pub const GEN2: u32 = 2;
    /// Gen3 (8 GT/s)
    pub const GEN3: u32 = 3;
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
    /// Link Control 2 (16-bit)
    pub const LINK_CTL2: u16 = 0x30;
    /// Link Status 2 (16-bit)
    pub const LINK_STA2: u16 = 0x32;
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

/// Device Capabilities register bits (offset 0x04 from PCIe cap)
pub mod dev_cap {
    /// Max Payload Size Supported (bits 2:0)
    /// 000=128B, 001=256B, 010=512B, 011=1KB, 100=2KB, 101=4KB
    pub const MPS_MASK: u32 = 0x7;
}

/// Device Status register bits (offset 0x0A from PCIe cap)
pub mod dev_sta {
    /// Correctable Error Detected (bit 0)
    pub const CED: u16 = 1 << 0;
    /// Non-Fatal Error Detected (bit 1)
    pub const NFED: u16 = 1 << 1;
    /// Fatal Error Detected (bit 2)
    pub const FED: u16 = 1 << 2;
    /// Unsupported Request Detected (bit 3) - indicates bad DMA address or unsupported TLP
    pub const URD: u16 = 1 << 3;
    /// AUX Power Detected (bit 4)
    pub const APD: u16 = 1 << 4;
    /// Transactions Pending (bit 5)
    pub const TP: u16 = 1 << 5;

    /// All error bits (W1C - Write 1 to Clear)
    pub const ALL_ERRORS: u16 = CED | NFED | FED | URD;
}

/// Device Control register bits (offset 0x08 from PCIe cap)
pub mod dev_ctl {
    /// Relaxed Ordering Enable (bit 4)
    pub const RELAX_ORDER_EN: u16 = 1 << 4;
    /// Max Payload Size (bits 7:5)
    pub const MPS_SHIFT: u16 = 5;
    pub const MPS_MASK: u16 = 0x7 << 5;
    /// Extended Tag Enable (bit 8)
    pub const EXT_TAG_EN: u16 = 1 << 8;
    /// No Snoop Enable (bit 11)
    pub const NO_SNOOP_EN: u16 = 1 << 11;
    /// Max Read Request Size (bits 14:12)
    /// 000=128B, 001=256B, 010=512B, 011=1KB, 100=2KB, 101=4KB
    pub const MRRS_SHIFT: u16 = 12;
    pub const MRRS_MASK: u16 = 0x7 << 12;

    /// MRRS value for 512 bytes (default per spec)
    pub const MRRS_512: u16 = 2 << 12;
    /// MRRS value for 4KB (maximum)
    pub const MRRS_4K: u16 = 5 << 12;
    /// MPS value for 128 bytes (safe default)
    pub const MPS_128: u16 = 0 << 5;
    /// MPS value for 256 bytes
    pub const MPS_256: u16 = 1 << 5;
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
///
/// The MediaTek PCIe Gen3 controller uses ATU tables for address translation:
///
/// **Outbound (CPU → PCIe):** Programs how CPU addresses map to PCIe bus addresses
/// when the CPU accesses device BARs. Configured at offset 0x800.
///
/// **Inbound (PCIe → CPU):** On MT7988A, inbound DMA uses identity mapping by default.
/// The AXI interconnect passes PCIe memory transactions directly to DRAM without
/// explicit translation table configuration. This means:
/// - PCIe bus address 0x40000000 → CPU physical address 0x40000000 (DRAM base)
/// - No explicit inbound ATU configuration required
///
/// For platforms that require explicit inbound windows, additional registers would
/// be needed (typically separate inbound ATU tables or iATU registers).
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

    /// Maximum number of translation tables (outbound)
    pub const MAX_TRANS_TABLES: usize = 8;

    /// ATU type bits (in source address LSB register)
    pub const TYPE_MEM: u32 = 0x0;  // Memory type
    pub const TYPE_IO: u32 = 0x1;   // I/O type

    /// ATU enable bit
    pub const ATR_EN: u32 = 1 << 0;

    /// Minimum ATU window size (4KB)
    pub const MIN_SIZE: u64 = 0x1000;

    /// Calculate ATU size encoding for a given window size
    /// Formula from Linux: (((size - 1) << 1) & 0x7E) | ATR_EN
    #[inline]
    pub const fn size_encoding(size: u64) -> u32 {
        if size == 0 {
            return 0;
        }
        let leading = size.leading_zeros();
        if leading >= 63 {
            return ATR_EN; // Minimum size
        }
        let size_log2 = 63 - leading; // fls(size) - 1
        if size_log2 >= 1 {
            (((size_log2 - 1) << 1) & 0x7E) as u32 | ATR_EN
        } else {
            ATR_EN
        }
    }
}

// ============================================================================
// MSI (Message Signaled Interrupts) Registers
// From Linux pcie-mediatek-gen3.c
// ============================================================================

/// Interrupt enable register offset
pub const PCIE_INT_ENABLE_REG: usize = 0x180;

/// Interrupt status register offset
pub const PCIE_INT_STATUS_REG: usize = 0x184;

/// MSI set enable register offset
pub const PCIE_MSI_SET_ENABLE_REG: usize = 0x190;

/// MSI set base register (set 0)
pub const PCIE_MSI_SET_BASE_REG: usize = 0xc00;

/// MSI set high address base
pub const PCIE_MSI_SET_ADDR_HI_BASE: usize = 0xc80;

/// MSI register constants
pub mod msi {
    /// Number of MSI sets (from Linux PCIE_MSI_SET_NUM)
    pub const SET_NUM: usize = 8;

    /// Offset between MSI sets (16 bytes per set)
    pub const SET_OFFSET: usize = 0x10;

    /// Status offset within MSI set
    pub const SET_STATUS_OFFSET: usize = 0x04;

    /// Enable offset within MSI set
    pub const SET_ENABLE_OFFSET: usize = 0x08;

    /// High address offset between sets
    pub const ADDR_HI_OFFSET: usize = 0x04;

    /// MSI enable bits in INT_ENABLE register (bits 15:8)
    /// Linux: PCIE_MSI_ENABLE = GENMASK(PCIE_MSI_SET_NUM + 8 - 1, 8)
    pub const INT_ENABLE_BITS: u32 = 0xFF << 8;

    /// MSI set enable bits in MSI_SET_ENABLE register (bits 7:0)
    /// Linux: PCIE_MSI_SET_ENABLE = GENMASK(PCIE_MSI_SET_NUM - 1, 0)
    pub const SET_ENABLE_BITS: u32 = 0xFF;
}
