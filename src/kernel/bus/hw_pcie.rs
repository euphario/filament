//! PCIe Hardware Constants and Operations
//!
//! This module contains PCIe MAC operations including clock gate control,
//! reset control, and bus mastering. Base addresses are stored per-bus
//! in BusController (set by probed via open(Bus) syscall).

use crate::kernel::arch::mmio::{MmioRegion, delay_us, dsb};

/// Get INFRACFG_AO base address (platform-specific constant)
#[inline]
fn infracfg_base() -> usize {
    #[cfg(feature = "platform-mt7988a")]
    { 0x1000_1000 }
    #[cfg(not(feature = "platform-mt7988a"))]
    { 0 } // No INFRACFG on QEMU
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

/// Reset control bits — per Linux pcie-mediatek-gen3.c mtk_pcie_startup_port():
/// Setting a bit ASSERTS that reset signal (device held in reset).
/// Clearing a bit DE-ASSERTS that reset signal (device running).
/// Sequence: assert all (set) → wait 100ms → deassert all (clear) → poll link.
/// Upper bits in the register contain other MAC control — always use RMW.
pub const RSTB_MAC: u32 = 1 << 0;
pub const RSTB_PHY: u32 = 1 << 1;
pub const RSTB_BRG: u32 = 1 << 2;
pub const RSTB_PE: u32 = 1 << 3;   // PERST# to endpoint (1=asserted=device in reset)
pub const RSTB_ALL: u32 = RSTB_MAC | RSTB_PHY | RSTB_BRG | RSTB_PE;

/// LTSSM status register offset
pub const LTSSM_STATUS_REG: usize = 0x150;
pub const LTSSM_STATE_MASK: u32 = 0x1F << 24;
pub const LTSSM_STATE_L0: u32 = 0x10 << 24;  // Link up

/// Link status register offset
pub const LINK_STATUS_REG: usize = 0x154;
pub const LINK_UP: u32 = 1 << 8;

/// MAC configuration registers (per Linux pcie-mediatek-gen3.c)
pub const PCIE_SETTING_REG: usize = 0x80;
pub const PCIE_RC_MODE: u32 = 1 << 0;
pub const PCIE_PCI_IDS_1: usize = 0x9C;
pub const PCIE_INT_ENABLE_REG: usize = 0x180;
pub const PCIE_INTX_ENABLE: u32 = 0xF << 16;  // GENMASK(19,16)
pub const PCIE_MISC_CTRL_REG: usize = 0x348;
pub const PCIE_DISABLE_DVFSRC_VLT_REQ: u32 = 1 << 1;

/// CFGNUM register for downstream config access
pub const CFGNUM_REG: usize = 0x140;
pub const CFGNUM_ENABLE: u32 = 1 << 20;
pub const CFGNUM_BE_MASK: u32 = 0xF << 16;

/// PCI Command register offset in config space
pub const CFG_OFFSET: usize = 0x1000;
pub const PCI_COMMAND: usize = 0x04;
pub const CMD_BUS_MASTER: u16 = 1 << 2;

// ─────────────────────────────────────────────────────────────────────────
// Address Translation Registers (ATR)
// Per Linux pcie-mediatek-gen3.c — maps CPU physical addresses to PCIe bus addresses.
// Each entry is 0x20 bytes. Up to 8 entries per port. Base at MAC+0x800.
// ─────────────────────────────────────────────────────────────────────────

pub const ATR_TABLE_BASE: usize = 0x800;
pub const ATR_ENTRY_SIZE: usize = 0x20;
pub const ATR_SRC_ADDR_MSB: usize = 0x04;
pub const ATR_TRSL_ADDR_LSB: usize = 0x08;
pub const ATR_TRSL_ADDR_MSB: usize = 0x0C;
pub const ATR_TRSL_PARAM: usize = 0x10;
pub const ATR_MAX_ENTRIES: usize = 8;

/// ATR size field: enable bit + log2(size) encoding
/// Per Linux: PCIE_ATR_SIZE(size) = (((size - 1) << 1) & 0x7E) | 1
fn atr_size_field(log2_size: u32) -> u32 {
    (((log2_size - 1) << 1) & 0x7E) | 1
}

/// Find-last-set (1-based position of highest set bit, like C fls)
fn fls(v: u64) -> u32 {
    if v == 0 { return 0; }
    64 - v.leading_zeros()
}

/// Find-first-set (1-based position of lowest set bit, like C ffs)
fn ffs(v: u64) -> u32 {
    if v == 0 { return 0; }
    v.trailing_zeros() + 1
}

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

/// Disable per-port PCIe clocks
///
/// Called to gate clocks for a port that has no link (save power).
/// Only gates per-port clock domains. Does NOT touch PEXTP_MAC_SWRST
/// because it is a shared reset for ALL PCIe ports — asserting it
/// would kill other ports' MAC logic and disrupt link training.
pub fn disable_clocks(index: usize) {
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

    // Disable per-port clocks by writing to SET registers (set gate = disable clock)
    infracfg.write32(INFRA0_CG_SET, peri_26m_bit);
    infracfg.write32(INFRA3_CG_SET, gfmux_bit | pipe_bit | clk_133m_bit);
    dsb();
}

/// PCIe startup: MAC config + root port setup + two-phase reset + link training.
///
/// Combines Linux pcie-mediatek-gen3.c and U-Boot mtk_pcie_startup_port():
/// 1. Configure MAC: RC mode, class code, mask INTx, disable DVFSRC
/// 2. Configure root port via direct MAC offsets (BAR0, bus numbers, command)
/// 3. Assert all reset signals (set bits) — RMW to preserve upper control bits
/// 4. Wait 100ms (PCIe CEM spec TPVPERL: power/clock stabilization)
/// 5. Two-phase deassert: MAC/PHY/BRG first → 100ms → PE (PERST#)
/// 6. Poll for link training (1000ms timeout)
///
/// Returns true if link came up (device connected), false if timeout (empty port).
pub fn startup_port(mac_base: usize) -> bool {
    let mac = MmioRegion::new(mac_base);

    // ── MAC configuration (before RST_CTRL, per Linux) ──

    // Set as Root Complex mode
    let mut setting = mac.read32(PCIE_SETTING_REG);
    setting |= PCIE_RC_MODE;
    mac.write32(PCIE_SETTING_REG, setting);

    // Set class code to PCI-to-PCI bridge (0x0604)
    let mut ids1 = mac.read32(PCIE_PCI_IDS_1);
    ids1 &= 0xFF; // Preserve revision ID (bits 7:0), clear class (bits 31:8)
    ids1 |= 0x0604_00; // Bridge class << 8
    mac.write32(PCIE_PCI_IDS_1, ids1);

    // Mask all INTx interrupts
    let mut int_en = mac.read32(PCIE_INT_ENABLE_REG);
    int_en &= !PCIE_INTX_ENABLE;
    mac.write32(PCIE_INT_ENABLE_REG, int_en);

    // Disable DVFSRC voltage request
    let mut misc = mac.read32(PCIE_MISC_CTRL_REG);
    misc |= PCIE_DISABLE_DVFSRC_VLT_REQ;
    mac.write32(PCIE_MISC_CTRL_REG, misc);

    // ── RST_CTRL: assert → delay → two-phase deassert ──

    // Assert all reset signals (RMW preserves upper control bits)
    let mut val = mac.read32(RST_CTRL_REG);
    val |= RSTB_ALL;
    mac.write32(RST_CTRL_REG, val);
    dsb();

    // PCIe CEM spec TPVPERL — 100ms for power and clock stabilization
    delay_us(100_000);

    // Phase 1: Deassert MAC, PHY, BRG — keep PE (PERST#) asserted
    // MAC needs time to initialize before endpoint comes out of reset
    val &= !(RSTB_MAC | RSTB_PHY | RSTB_BRG);
    mac.write32(RST_CTRL_REG, val);
    dsb();
    delay_us(100_000);

    // Phase 2: Deassert PE (PERST# to endpoint)
    val &= !RSTB_PE;
    mac.write32(RST_CTRL_REG, val);
    dsb();

    // Poll for link training to complete (1000ms timeout, 1ms poll)
    // Linux uses ~3s, U-Boot uses ~1s — 1s should be plenty for NVMe
    for _ in 0..1000 {
        delay_us(1000);
        let link_status = mac.read32(LINK_STATUS_REG);
        if (link_status & LINK_UP) != 0 {
            return true;
        }
    }

    // Log final state for diagnostics
    let rst_readback = mac.read32(RST_CTRL_REG);
    let ltssm = mac.read32(LTSSM_STATUS_REG);
    crate::kdebug!("bus", "pcie_link_timeout";
        rst = crate::klog::hex32(rst_readback),
        ltssm = crate::klog::hex32(ltssm));

    false
}

/// Configure the root port via config window (MAC+0x1000) after link up.
///
/// The config window at MAC+0x1000..0x10FF exposes the root port's PCI config
/// registers. After link training completes, we must configure:
/// - Bus numbers: primary=0, secondary=1, subordinate=0xFF
/// - Command: MEM + MASTER enabled (so downstream config/MMIO accesses work)
/// - Memory window: base and limit for downstream BAR forwarding
///
/// Without this, CFGNUM-based downstream config reads (bus 1+) return 0xFFFF
/// because the root port won't forward Type 1 config cycles.
pub fn configure_root_port(mac_base: usize) {
    let mac = MmioRegion::new(mac_base);

    // Set CFGNUM to target root port (bus=0, devfn=0) before config writes.
    // Per Linux: always set CFGNUM before accessing MAC+0x1000.
    let cfgnum = CFGNUM_ENABLE | CFGNUM_BE_MASK; // bus=0, devfn=0
    mac.write32(CFGNUM_REG, cfgnum);

    // Bus numbers: primary=0, secondary=1, subordinate=0xFF
    let busno = mac.read32(CFG_OFFSET + 0x18);
    mac.write32(CFG_OFFSET + 0x18, (busno & !0x00FF_FFFF) | 0x00FF_0100);

    // Command: enable I/O(0x01) + MEM(0x02) + MASTER(0x04) + SERR(0x100)
    let cmd = mac.read32(CFG_OFFSET + PCI_COMMAND);
    mac.write32(CFG_OFFSET + PCI_COMMAND, (cmd & 0xFFFF_0000) | 0x0107);

    // Memory window: set a wide range so root port forwards all memory accesses
    // Offset 0x20: Memory Base / Memory Limit (upper 12 bits of 32-bit address, 1MB granularity)
    // Set base=0x20000000 (0x2000 << 16), limit=0x2FFF0000 (0x2FFF << 16)
    mac.write32(CFG_OFFSET + 0x20, 0x2FFF_2000);

    // Prefetchable memory: set base > limit to disable (no prefetchable window)
    mac.write32(CFG_OFFSET + 0x24, 0x0000_FFF0);

    dsb();
}

/// Set up ATR (Address Translation Register) tables for outbound memory access.
///
/// Per Linux pcie-mediatek-gen3.c mtk_pcie_set_trans_table():
/// Programs MAC+0x800 region to translate CPU physical addresses to PCIe bus addresses.
/// This is required for downstream device BAR MMIO access.
///
/// Uses 1:1 mapping (CPU addr = PCIe bus addr) matching the DTS ranges.
/// Splits non-power-of-2 ranges into multiple entries (up to 8).
pub fn setup_atr(mac_base: usize, port_index: usize) {
    // Memory aperture per port (from MT7988A DTS)
    let (cpu_addr, size): (u64, u64) = match port_index {
        0 => (0x2020_0000, 0x07E0_0000), // 126MB
        1 => (0x2420_0000, 0x03E0_0000), // 62MB
        2 => (0x2620_0000, 0x01E0_0000), // 30MB
        3 => (0x2820_0000, 0x07E0_0000), // 126MB
        _ => return,
    };

    let mac = MmioRegion::new(mac_base);
    let mut remaining = size;
    let mut addr = cpu_addr;
    let mut entry = 0usize;

    while remaining > 0 && entry < ATR_MAX_ENTRIES {
        // Table size must be power-of-2 and aligned to current address
        let mut table_size = 1u64 << (fls(remaining) - 1);
        if addr > 0 {
            let addr_align = 1u64 << (ffs(addr) - 1);
            if addr_align < table_size {
                table_size = addr_align;
            }
        }

        if table_size < 0x1000 {
            break; // Minimum 4KB
        }

        let table_base = ATR_TABLE_BASE + entry * ATR_ENTRY_SIZE;
        let log2 = fls(table_size) - 1;

        // SRC_ADDR_LSB: cpu_addr[31:0] | ATR_SIZE(log2) with enable bit
        mac.write32(table_base, (addr as u32) | atr_size_field(log2));
        // SRC_ADDR_MSB: cpu_addr[63:32]
        mac.write32(table_base + ATR_SRC_ADDR_MSB, (addr >> 32) as u32);
        // TRSL_ADDR_LSB: pci_addr[31:0] (1:1 mapping)
        mac.write32(table_base + ATR_TRSL_ADDR_LSB, addr as u32);
        // TRSL_ADDR_MSB: pci_addr[63:32]
        mac.write32(table_base + ATR_TRSL_ADDR_MSB, (addr >> 32) as u32);
        // TRSL_PARAM: MEM type = 0 (ATR_TYPE_MEM | ATR_TLP_TYPE_MEM)
        mac.write32(table_base + ATR_TRSL_PARAM, 0);

        addr += table_size;
        remaining -= table_size;
        entry += 1;
    }

    dsb();

    crate::kdebug!("bus", "pcie_atr_setup"; port = port_index as u64, entries = entry as u64);
}

/// Set or clear bus mastering for a PCIe device
///
/// device_id format: (bus << 8) | (dev << 3) | func
/// For root port devices, this is typically 0x0000
pub fn set_bus_mastering(bus_index: u8, device_id: u16, enable: bool, ecam_based: bool, mac_base_addr: u64) -> Result<(), super::BusError> {
    // ECAM-based platforms (QEMU): set bus mastering via kernel PCI subsystem
    if ecam_based {
        use crate::kernel::pci::{self, PciBdf};
        let bdf = PciBdf::from_u32(device_id as u32);
        let cmd_status = pci::config_read32(bdf, 0x04).map_err(|_| super::BusError::HardwareError)?;
        let cmd16 = cmd_status as u16;
        let new_cmd = if enable { cmd16 | 0x4 } else { cmd16 & !0x4 };
        if cmd16 != new_cmd {
            // Preserve Status register (upper 16 bits) — Status bits are W1C,
            // so writing 0 to them avoids accidentally clearing status.
            let new_val = (cmd_status & 0xFFFF_0000) | (new_cmd as u32);
            pci::config_write32(bdf, 0x04, new_val).map_err(|_| super::BusError::HardwareError)?;
        }
        // Verify and log
        let verify = pci::config_read32(bdf, 0x04).unwrap_or(0xFFFFFFFF);
        crate::kinfo!("bus", "ecam_bm_set";
            bdf = device_id as u64,
            bus = bdf.bus as u64,
            dev = bdf.device as u64,
            before = cmd16 as u64,
            after = (verify & 0xFFFF) as u64);
        return Ok(());
    }

    let base = mac_base_addr as usize;
    if base == 0 {
        return Err(super::BusError::NotFound);
    }
    let cfg = MmioRegion::new(base);

    // Extract BDF from device_id — strip port encoding from upper nibble.
    // Port encoding: upper nibble of bus byte = port index (e.g., 0x31 = port 3, bus 1).
    // mac_base_addr already selects the correct port, so we only need the actual bus number.
    let bus_num = (device_id >> 8) as u8 & 0x0F;
    let devfn = (device_id & 0xFF) as u8;

    // When enabling bus mastering on a downstream device (bus > 0), also ensure
    // the root port bridge has Bus Master enabled. Without it, the root port won't
    // forward DMA TLPs from downstream devices upstream to system memory.
    // This mirrors Linux PCI core behavior (pci_set_master walks up bridge hierarchy).
    if enable && bus_num != 0 {
        // Enable Bus Master on root port (bus 0) via direct config window.
        // CFGNUM must be set to target root port before accessing MAC+0x1000.
        cfg.write32(CFGNUM_REG, CFGNUM_ENABLE | CFGNUM_BE_MASK); // bus=0, devfn=0
        dsb();
        let rp_cmd_status = cfg.read32(CFG_OFFSET + PCI_COMMAND);
        let rp_current = (rp_cmd_status & 0xFFFF) as u16;
        if rp_cmd_status != 0xFFFF_FFFF && (rp_current & CMD_BUS_MASTER) == 0 {
            let rp_new = (rp_cmd_status & 0xFFFF_0000) | ((rp_current | CMD_BUS_MASTER) as u32);
            cfg.write32(CFG_OFFSET + PCI_COMMAND, rp_new);
            dsb();
            let rp_verify = (cfg.read32(CFG_OFFSET + PCI_COMMAND) & 0xFFFF) as u16;
            crate::kinfo!("bus", "mac_bm_set"; port = bus_index as u64, bus = 0u64, devfn = 0u64, before = rp_current as u64, after = rp_verify as u64);
        }
    }

    // Set CFGNUM to target the correct bus/devfn before accessing MAC+0x1000.
    // Per Linux: always set CFGNUM before config space access, even for bus 0.
    let cfgnum_val = CFGNUM_ENABLE | CFGNUM_BE_MASK
        | ((bus_num as u32) << 8) | (devfn as u32);
    cfg.write32(CFGNUM_REG, cfgnum_val);
    dsb();

    // Read PCI Command/Status as 32-bit (some MAC config windows don't support 16-bit access)
    let cmd_status_offset = CFG_OFFSET + PCI_COMMAND;
    let cmd_status = cfg.read32(cmd_status_offset);
    let current = (cmd_status & 0xFFFF) as u16;

    // All-1s means config space not accessible — skip silently
    if cmd_status == 0xFFFF_FFFF {
        return Ok(());
    }

    let new_cmd = if enable {
        current | CMD_BUS_MASTER
    } else {
        current & !CMD_BUS_MASTER
    };

    if current != new_cmd {
        // Write back command register, preserve status (upper 16 bits as 0 — W1C safe)
        let new_val = (cmd_status & 0xFFFF_0000) | (new_cmd as u32);
        cfg.write32(cmd_status_offset, new_val);
        dsb();

        // Verify
        let verify = cfg.read32(cmd_status_offset);
        let verify_cmd = (verify & 0xFFFF) as u16;
        if (verify_cmd & CMD_BUS_MASTER != 0) != enable {
            crate::kerror!("bus", "pcie_bm_verify_failed"; index = bus_index as u64, cmd = verify_cmd as u64);
            return Err(super::BusError::HardwareError);
        }
        crate::kinfo!("bus", "mac_bm_set"; port = bus_index as u64, bus = bus_num as u64, devfn = devfn as u64, before = current as u64, after = verify_cmd as u64);
    }

    Ok(())
}

/// Disable bus mastering for ALL devices on a PCIe port
pub fn disable_all_bus_mastering(bus_index: u8, ecam_based: bool, mac_base_addr: u64) {
    // ECAM-based platforms (QEMU): disable via kernel PCI subsystem for all known devices
    if ecam_based {
        use crate::kernel::pci;
        pci::with_devices(|registry| {
            for i in 0..registry.len() {
                if let Some(dev) = registry.get(i) {
                    let device_id = ((dev.bdf.bus as u16) << 8)
                        | ((dev.bdf.device as u16) << 3)
                        | (dev.bdf.function as u16);
                    let _ = set_bus_mastering(bus_index, device_id, false, ecam_based, mac_base_addr);
                }
            }
        });
        return;
    }

    if mac_base_addr == 0 {
        return;
    }

    // Disable on root port (device 0)
    let _ = set_bus_mastering(bus_index, 0x0000, false, ecam_based, mac_base_addr);

    // Disable on first device behind root port (bus 1, device 0)
    let _ = set_bus_mastering(bus_index, 0x0100, false, ecam_based, mac_base_addr);
}

/// Verify PCIe link is in expected state after reset
pub fn verify_link(index: usize, ecam_based: bool, mac_base_addr: u64) -> bool {
    // ECAM-based platforms (QEMU): always assume valid
    if ecam_based {
        return true;
    }

    let base = mac_base_addr as usize;
    if base == 0 {
        return false;
    }
    let mac = MmioRegion::new(base);

    // Check reset bits are cleared (0 = deasserted = running)
    let rst_ctrl = mac.read32(RST_CTRL_REG);
    if (rst_ctrl & RSTB_ALL) != 0 {
        crate::kerror!("bus", "pcie_rst_not_deasserted"; index = index as u64);
        return false;
    }

    true  // Controller is in a valid state
}

/// Query PCIe-specific capabilities by reading hardware registers
pub fn query_capabilities(_bus_index: u8, ecam_based: bool, mac_base_addr: u64) -> u8 {
    use super::bus_caps;
    let mut caps = 0u8;

    // All PCIe ports support bus mastering and MSI
    caps |= bus_caps::PCIE_BUS_MASTER;
    caps |= bus_caps::PCIE_MSI;

    // ECAM-based platforms (QEMU): can't read MAC registers, assume link up
    if ecam_based {
        caps |= bus_caps::PCIE_LINK_UP;
        return caps;
    }

    // MAC-based platforms (MT7988): check if link is up by reading LTSSM state
    let base = mac_base_addr as usize;
    if base != 0 {
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
