//! MT7988A PCIe SoC Wrapper
//!
//! MediaTek MT7988A has 4 PCIe Gen3 controllers with:
//! - INFRACFG_AO for clock gates (inverted polarity: SET=gate, CLR=ungate)
//! - PEXTP PHY reset in reset controller
//! - TLP-based config space access through MAC registers

use super::{SocPcie, SocError, PciePortConfig};
use crate::MmioRegion;

/// INFRACFG_AO base address (clock/reset controller)
const INFRACFG_AO_BASE: u64 = 0x1000_1000;
const INFRACFG_AO_SIZE: u64 = 0x1000;

/// MAC register region size
const PCIE_MAC_SIZE: u64 = 0x2000;

/// Reset controller registers (set/clear model)
mod reset {
    /// RST0 SET register (write 1 to assert reset)
    pub const RST0_SET: usize = 0x70;
    /// RST0 CLR register (write 1 to deassert reset)
    pub const RST0_CLR: usize = 0x74;
    /// RST0 STA register (read status)
    pub const RST0_STA: usize = 0x78;

    /// PEXTP MAC software reset (bit 6)
    pub const PEXTP_MAC_SWRST: u32 = 1 << 6;
}

/// INFRA0 clock gate registers
/// Note: MediaTek uses inverted polarity - SET = gate (disable), CLR = ungate (enable)
mod infra0 {
    #[allow(dead_code)]
    pub const SET: usize = 0x10;  // Used to gate (disable) clocks
    pub const CLR: usize = 0x14;  // Used to ungate (enable) clocks
    pub const STA: usize = 0x18;

    pub const PCIE_PERI_26M_P0: u32 = 1 << 7;
    pub const PCIE_PERI_26M_P1: u32 = 1 << 8;
    pub const PCIE_PERI_26M_P2: u32 = 1 << 9;
    pub const PCIE_PERI_26M_P3: u32 = 1 << 10;

    pub const ALL: u32 = PCIE_PERI_26M_P0 | PCIE_PERI_26M_P1 |
                         PCIE_PERI_26M_P2 | PCIE_PERI_26M_P3;
}

/// INFRA3 clock gate registers
mod infra3 {
    #[allow(dead_code)]
    pub const SET: usize = 0x60;  // Used to gate (disable) clocks
    pub const CLR: usize = 0x64;  // Used to ungate (enable) clocks
    pub const STA: usize = 0x68;

    pub const PCIE_GFMUX_TL_P0: u32 = 1 << 20;
    pub const PCIE_GFMUX_TL_P1: u32 = 1 << 21;
    pub const PCIE_GFMUX_TL_P2: u32 = 1 << 22;
    pub const PCIE_GFMUX_TL_P3: u32 = 1 << 23;

    pub const PCIE_PIPE_P0: u32 = 1 << 24;
    pub const PCIE_PIPE_P1: u32 = 1 << 25;
    pub const PCIE_PIPE_P2: u32 = 1 << 26;
    pub const PCIE_PIPE_P3: u32 = 1 << 27;

    pub const PCIE_133M_P0: u32 = 1 << 28;
    pub const PCIE_133M_P1: u32 = 1 << 29;
    pub const PCIE_133M_P2: u32 = 1 << 30;
    pub const PCIE_133M_P3: u32 = 1 << 31;

    pub const ALL: u32 = PCIE_GFMUX_TL_P0 | PCIE_GFMUX_TL_P1 |
                         PCIE_GFMUX_TL_P2 | PCIE_GFMUX_TL_P3 |
                         PCIE_PIPE_P0 | PCIE_PIPE_P1 |
                         PCIE_PIPE_P2 | PCIE_PIPE_P3 |
                         PCIE_133M_P0 | PCIE_133M_P1 |
                         PCIE_133M_P2 | PCIE_133M_P3;
}

/// Port clock bits indexed by port number
const PORT_INFRA0_BITS: [u32; 4] = [
    infra0::PCIE_PERI_26M_P0,
    infra0::PCIE_PERI_26M_P1,
    infra0::PCIE_PERI_26M_P2,
    infra0::PCIE_PERI_26M_P3,
];

const PORT_INFRA3_BITS: [u32; 4] = [
    infra3::PCIE_GFMUX_TL_P0 | infra3::PCIE_PIPE_P0 | infra3::PCIE_133M_P0,
    infra3::PCIE_GFMUX_TL_P1 | infra3::PCIE_PIPE_P1 | infra3::PCIE_133M_P1,
    infra3::PCIE_GFMUX_TL_P2 | infra3::PCIE_PIPE_P2 | infra3::PCIE_133M_P2,
    infra3::PCIE_GFMUX_TL_P3 | infra3::PCIE_PIPE_P3 | infra3::PCIE_133M_P3,
];

/// Memory window size per port (from DTS: 0x7e00000 = ~126MB)
/// First 2MB (0x200000) is I/O space, rest is memory space
const PCIE_MEM_SIZE: u64 = 0x07e0_0000;

/// Static port configuration for MT7988A
/// Note: Port descriptions are generic - board layer can override with slot names
/// Memory bases are at offset 0x200000 from window start (I/O space is first 2MB)
const MT7988A_PORTS: [PciePortConfig; 4] = [
    PciePortConfig {
        index: 0,
        mac_base: 0x11300000,
        mac_size: PCIE_MAC_SIZE,
        mem_base: 0x30200000,  // Window 0x30000000 + 0x200000 offset
        mem_size: PCIE_MEM_SIZE,
        irq: 168 + 32,  // SPI 168 -> GIC 200
        desc: "PCIe0",
    },
    PciePortConfig {
        index: 1,
        mac_base: 0x11310000,
        mac_size: PCIE_MAC_SIZE,
        mem_base: 0x38200000,  // Window 0x38000000 + 0x200000 offset
        mem_size: PCIE_MEM_SIZE,
        irq: 169 + 32,  // SPI 169 -> GIC 201
        desc: "PCIe1",
    },
    PciePortConfig {
        index: 2,
        mac_base: 0x11280000,
        mac_size: PCIE_MAC_SIZE,
        mem_base: 0x20200000,  // Window 0x20000000 + 0x200000 offset
        mem_size: PCIE_MEM_SIZE,
        irq: 170 + 32,  // SPI 170 -> GIC 202
        desc: "PCIe2",
    },
    PciePortConfig {
        index: 3,
        mac_base: 0x11290000,
        mac_size: PCIE_MAC_SIZE,
        mem_base: 0x28200000,  // Window 0x28000000 + 0x200000 offset
        mem_size: PCIE_MEM_SIZE,
        irq: 171 + 32,  // SPI 171 -> GIC 203
        desc: "PCIe3",
    },
];

/// MT7988A PCIe SoC wrapper
pub struct Mt7988aSoc {
    infracfg: MmioRegion,
}

impl Mt7988aSoc {
    /// Create a new MT7988A SoC wrapper
    ///
    /// Maps the INFRACFG_AO region for clock/reset control.
    pub fn new() -> Result<Self, SocError> {
        let infracfg = MmioRegion::open(INFRACFG_AO_BASE, INFRACFG_AO_SIZE)
            .ok_or(SocError::MmioFailed)?;
        Ok(Self { infracfg })
    }
}

impl SocPcie for Mt7988aSoc {
    fn port_count(&self) -> u8 {
        4
    }

    fn port_config(&self, index: u8) -> Option<PciePortConfig> {
        MT7988A_PORTS.get(index as usize).copied()
    }

    fn enable_clocks(&self) -> Result<(), SocError> {
        // Enable by writing to CLR registers (clear gate = enable clock)
        self.infracfg.write32(infra0::CLR, infra0::ALL);
        self.infracfg.write32(infra3::CLR, infra3::ALL);
        Ok(())
    }

    fn enable_port_clocks(&self, index: u8) -> Result<(), SocError> {
        if index >= 4 {
            return Err(SocError::InvalidPort);
        }

        // Enable by writing to CLR registers
        self.infracfg.write32(infra0::CLR, PORT_INFRA0_BITS[index as usize]);
        self.infracfg.write32(infra3::CLR, PORT_INFRA3_BITS[index as usize]);
        Ok(())
    }

    fn deassert_phy_reset(&self) -> Result<(), SocError> {
        self.infracfg.write32(reset::RST0_CLR, reset::PEXTP_MAC_SWRST);
        Ok(())
    }

    fn assert_phy_reset(&self) -> Result<(), SocError> {
        self.infracfg.write32(reset::RST0_SET, reset::PEXTP_MAC_SWRST);
        Ok(())
    }

    fn is_phy_reset_asserted(&self) -> bool {
        (self.infracfg.read32(reset::RST0_STA) & reset::PEXTP_MAC_SWRST) != 0
    }

    fn clock_status(&self) -> (u32, u32) {
        (
            self.infracfg.read32(infra0::STA),
            self.infracfg.read32(infra3::STA),
        )
    }

    fn reset_status(&self) -> u32 {
        self.infracfg.read32(reset::RST0_STA)
    }
}
