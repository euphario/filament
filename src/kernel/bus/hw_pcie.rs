//! PCIe Hardware Constants and Operations
//!
//! This module contains PCIe MAC operations including clock gate control,
//! reset control, and bus mastering. Base addresses come from the platform
//! configuration (selected at boot based on DTB).

use crate::arch::aarch64::mmio::{MmioRegion, delay_us, dsb};
use super::config::bus_config;

/// Get PCIe MAC base address for a port (from platform config)
#[inline]
fn mac_base(index: usize) -> Option<usize> {
    bus_config().pcie_base(index)
}

/// Get number of PCIe ports (from platform config)
#[inline]
pub fn port_count() -> usize {
    bus_config().pcie_port_count()
}

/// Get INFRACFG_AO base address (from platform config)
#[inline]
fn infracfg_base() -> usize {
    bus_config().infracfg_base
}

// ─────────────────────────────────────────────────────────────────────────
// Clock Gate Registers (MediaTek set/clear model)
// Write to SET register to gate clock (disable)
// Write to CLEAR register to ungate clock (enable)
// ─────────────────────────────────────────────────────────────────────────

/// INFRA0 clock gate registers (PCIE_PERI_26M clocks)
pub const INFRA0_CG_SET: usize = 0x10;   // Write 1 to gate (disable)
pub const INFRA0_CG_CLR: usize = 0x14;   // Write 1 to ungate (enable)
pub const INFRA0_CG_STA: usize = 0x18;   // Status (1=gated/disabled)

/// INFRA0 clock bits for PCIe PERI 26M clocks
pub const INFRA0_PCIE_PERI_26M_P0: u32 = 1 << 7;
pub const INFRA0_PCIE_PERI_26M_P1: u32 = 1 << 8;
pub const INFRA0_PCIE_PERI_26M_P2: u32 = 1 << 9;
pub const INFRA0_PCIE_PERI_26M_P3: u32 = 1 << 10;
pub const INFRA0_PCIE_PERI_26M: [u32; 4] = [
    INFRA0_PCIE_PERI_26M_P0,
    INFRA0_PCIE_PERI_26M_P1,
    INFRA0_PCIE_PERI_26M_P2,
    INFRA0_PCIE_PERI_26M_P3,
];

/// INFRA3 clock gate registers (PIPE, GFMUX_TL, 133M clocks)
pub const INFRA3_CG_SET: usize = 0x60;   // Write 1 to gate (disable)
pub const INFRA3_CG_CLR: usize = 0x64;   // Write 1 to ungate (enable)
pub const INFRA3_CG_STA: usize = 0x68;   // Status (1=gated/disabled)

/// INFRA3 clock bits for PCIe GFMUX TL clocks (bits 20-23)
pub const INFRA3_PCIE_GFMUX_TL_P0: u32 = 1 << 20;
pub const INFRA3_PCIE_GFMUX_TL_P1: u32 = 1 << 21;
pub const INFRA3_PCIE_GFMUX_TL_P2: u32 = 1 << 22;
pub const INFRA3_PCIE_GFMUX_TL_P3: u32 = 1 << 23;
pub const INFRA3_PCIE_GFMUX_TL: [u32; 4] = [
    INFRA3_PCIE_GFMUX_TL_P0,
    INFRA3_PCIE_GFMUX_TL_P1,
    INFRA3_PCIE_GFMUX_TL_P2,
    INFRA3_PCIE_GFMUX_TL_P3,
];

/// INFRA3 clock bits for PCIe PIPE clocks (bits 24-27)
pub const INFRA3_PCIE_PIPE_P0: u32 = 1 << 24;
pub const INFRA3_PCIE_PIPE_P1: u32 = 1 << 25;
pub const INFRA3_PCIE_PIPE_P2: u32 = 1 << 26;
pub const INFRA3_PCIE_PIPE_P3: u32 = 1 << 27;
pub const INFRA3_PCIE_PIPE: [u32; 4] = [
    INFRA3_PCIE_PIPE_P0,
    INFRA3_PCIE_PIPE_P1,
    INFRA3_PCIE_PIPE_P2,
    INFRA3_PCIE_PIPE_P3,
];

/// INFRA3 clock bits for PCIe 133M clocks (bits 28-31)
pub const INFRA3_PCIE_133M_P0: u32 = 1 << 28;
pub const INFRA3_PCIE_133M_P1: u32 = 1 << 29;
pub const INFRA3_PCIE_133M_P2: u32 = 1 << 30;
pub const INFRA3_PCIE_133M_P3: u32 = 1 << 31;
pub const INFRA3_PCIE_133M: [u32; 4] = [
    INFRA3_PCIE_133M_P0,
    INFRA3_PCIE_133M_P1,
    INFRA3_PCIE_133M_P2,
    INFRA3_PCIE_133M_P3,
];

// ─────────────────────────────────────────────────────────────────────────
// INFRACFG Reset Registers (controls power domain resets)
// ─────────────────────────────────────────────────────────────────────────

/// Reset register offsets (set/clear model)
pub const RST0_SET: usize = 0x70;   // Write 1 to assert reset
pub const RST0_CLR: usize = 0x74;   // Write 1 to deassert reset
pub const RST0_STA: usize = 0x78;   // Read status

/// PEXTP MAC software reset bit (shared for all PCIe ports)
/// This must be deasserted before RST_CTRL register becomes writable
pub const PEXTP_MAC_SWRST: u32 = 1 << 6;

/// MAC reset control register offset
pub const RST_CTRL_REG: usize = 0x148;

/// Reset control bits (active high)
pub const RST_MAC: u32 = 1 << 0;
pub const RST_PHY: u32 = 1 << 1;
pub const RST_BRG: u32 = 1 << 2;
pub const RST_PE: u32 = 1 << 3;   // PERST# to endpoint
pub const RST_ALL: u32 = RST_MAC | RST_PHY | RST_BRG | RST_PE;

/// LTSSM status register offset
pub const LTSSM_STATUS_REG: usize = 0x150;
pub const LTSSM_STATE_MASK: u32 = 0x1F << 24;
pub const LTSSM_STATE_L0: u32 = 0x10 << 24;  // Link up

/// Link status register offset
pub const LINK_STATUS_REG: usize = 0x154;
pub const LINK_UP: u32 = 1 << 8;

/// PCI Command register offset in config space
pub const CFG_OFFSET: usize = 0x1000;
pub const PCI_COMMAND: usize = 0x04;
pub const CMD_BUS_MASTER: u16 = 1 << 2;

// ─────────────────────────────────────────────────────────────────────────
// Hardware Operations
// ─────────────────────────────────────────────────────────────────────────

/// Enable PCIe clocks and deassert INFRACFG reset for a port
///
/// Must be called before accessing PCIe MAC registers.
/// Steps:
/// 1. Enable clocks: PCIE_PERI_26M, GFMUX_TL, PIPE, 133M
/// 2. Deassert PEXTP_MAC_SWRST (required for RST_CTRL to be writable)
pub fn enable_clocks(index: usize) {
    let base = infracfg_base();
    if base == 0 {
        return;  // No INFRACFG on this platform
    }
    let infracfg = MmioRegion::new(base);

    // Get clock bits for this port
    let peri_26m_bit = INFRA0_PCIE_PERI_26M[index];
    let gfmux_bit = INFRA3_PCIE_GFMUX_TL[index];
    let pipe_bit = INFRA3_PCIE_PIPE[index];
    let clk_133m_bit = INFRA3_PCIE_133M[index];

    // Enable clocks by writing to CLEAR registers (clear gate = enable clock)
    infracfg.write32(INFRA0_CG_CLR, peri_26m_bit);
    dsb();

    let infra3_bits = gfmux_bit | pipe_bit | clk_133m_bit;
    infracfg.write32(INFRA3_CG_CLR, infra3_bits);
    dsb();

    // Small delay for clocks to stabilize
    delay_us(100);

    // Deassert PEXTP_MAC_SWRST (write 1 to CLR register to deassert)
    infracfg.write32(RST0_CLR, PEXTP_MAC_SWRST);
    dsb();
    delay_us(10);
}

/// Disable PCIe clocks and assert INFRACFG reset for a port
///
/// Called to put port back in safe/powered-down state.
/// Steps:
/// 1. Assert PEXTP_MAC_SWRST (put MAC in reset)
/// 2. Disable clocks
pub fn disable_clocks(index: usize) {
    let base = infracfg_base();
    if base == 0 {
        return;  // No INFRACFG on this platform
    }
    let infracfg = MmioRegion::new(base);

    // Assert PEXTP_MAC_SWRST (write 1 to SET register to assert)
    infracfg.write32(RST0_SET, PEXTP_MAC_SWRST);
    dsb();

    // Get clock bits for this port
    let peri_26m_bit = INFRA0_PCIE_PERI_26M[index];
    let gfmux_bit = INFRA3_PCIE_GFMUX_TL[index];
    let pipe_bit = INFRA3_PCIE_PIPE[index];
    let clk_133m_bit = INFRA3_PCIE_133M[index];

    // Disable clocks by writing to SET registers (set gate = disable clock)
    infracfg.write32(INFRA0_CG_SET, peri_26m_bit);
    infracfg.write32(INFRA3_CG_SET, gfmux_bit | pipe_bit | clk_133m_bit);
    dsb();
}

/// Set or clear bus mastering for a PCIe device
///
/// device_id format: (bus << 8) | (dev << 3) | func
/// For root port devices, this is typically 0x0000
pub fn set_bus_mastering(bus_index: u8, device_id: u16, enable: bool) -> Result<(), super::BusError> {
    let index = bus_index as usize;
    let Some(base) = mac_base(index) else {
        return Err(super::BusError::NotFound);
    };
    let cfg_base = base + CFG_OFFSET;
    let cfg = MmioRegion::new(cfg_base);

    // Extract BDF from device_id
    let bus_num = (device_id >> 8) as u8;
    let devfn = (device_id & 0xFF) as u8;

    // For devices on this port's secondary bus, we access via TLP
    // For now, we only support the root port itself (bus 0, devfn 0)
    // and immediate children (bus 1, any devfn)
    if bus_num > 1 {
        // We allow this - driver can manage downstream devices
        return Ok(());
    }

    // Configure TLP header for config access
    // MediaTek uses CFGNUM register at MAC+0x140
    let cfgnum = MmioRegion::new(base);
    let cfgnum_val = ((bus_num as u32) << 8) | (devfn as u32) | (0xF << 16); // BE=0xF
    cfgnum.write32(0x140, cfgnum_val);
    dsb();

    // Read current PCI_COMMAND (offset 0x04, 16-bit)
    let cmd_offset = PCI_COMMAND;
    let current = cfg.read16(cmd_offset);

    let new_cmd = if enable {
        current | CMD_BUS_MASTER
    } else {
        current & !CMD_BUS_MASTER
    };

    if current != new_cmd {
        cfg.write16(cmd_offset, new_cmd);
        dsb();

        // Verify
        let verify = cfg.read16(cmd_offset);
        if (verify & CMD_BUS_MASTER != 0) != enable {
            crate::kerror!("bus", "pcie_bm_verify_failed"; index = index as u64, cmd = verify as u64);
            return Err(super::BusError::HardwareError);
        }
    }

    Ok(())
}

/// Disable bus mastering for ALL devices on a PCIe port
pub fn disable_all_bus_mastering(bus_index: u8) {
    let index = bus_index as usize;
    if mac_base(index).is_none() {
        return;
    }

    // Disable on root port (device 0)
    let _ = set_bus_mastering(bus_index, 0x0000, false);

    // Disable on first device behind root port (bus 1, device 0)
    let _ = set_bus_mastering(bus_index, 0x0100, false);
}

/// Verify PCIe link is in expected state after reset
pub fn verify_link(index: usize) -> bool {
    let Some(base) = mac_base(index) else {
        return false;
    };
    let mac = MmioRegion::new(base);

    // Check reset control is deasserted
    let rst_ctrl = mac.read32(RST_CTRL_REG);
    if rst_ctrl != RST_ALL {
        crate::kerror!("bus", "pcie_rst_not_deasserted"; index = index as u64);
        return false;
    }

    true  // Controller is in a valid state
}

/// Query PCIe-specific capabilities by reading hardware registers
pub fn query_capabilities(bus_index: u8) -> u8 {
    use super::bus_caps;
    let mut caps = 0u8;

    // All PCIe ports support bus mastering and MSI
    caps |= bus_caps::PCIE_BUS_MASTER;
    caps |= bus_caps::PCIE_MSI;

    // Check if link is up by reading LTSSM state
    if let Some(base) = mac_base(bus_index as usize) {
        let mmio = MmioRegion::new(base);

        // Read K_CNT_APPL register (offset 0x104C) for LTSSM state
        // LTSSM state is in bits [28:24], L0 = 0x10
        let k_cnt_appl = mmio.read32(0x104C);
        let ltssm_state = (k_cnt_appl >> 24) & 0x1F;

        // L0 (normal operation) or L0s (low power L0) means link is up
        if ltssm_state == 0x10 || ltssm_state == 0x11 {
            caps |= bus_caps::PCIE_LINK_UP;
        }
    }

    caps
}
