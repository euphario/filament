//! Mt76Device — MMIO register access, address remapping, WFSYS reset
//!
//! Provides all register access methods for the MT7996 via BAR0.
//! Other modules take `&Mt76Device` to read/write registers.

use userlib::{uinfo, uerror};
use crate::regs::*;

/// Error type for device operations.
#[derive(Debug)]
pub enum DeviceError {
    /// Poll timed out waiting for expected value
    PollTimeout { reg: u32, mask: u32, expected: u32, got: u32 },
    /// Driver ownership request failed
    DriverOwnTimeout { band: u32 },
}

impl DeviceError {
    pub fn name(&self) -> &'static str {
        match self {
            Self::PollTimeout { .. } => "poll_timeout",
            Self::DriverOwnTimeout { .. } => "driver_own_timeout",
        }
    }
}

impl core::fmt::Display for DeviceError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::PollTimeout { reg, mask, expected, got } =>
                write!(f, "poll timeout reg={:#x} mask={:#x} expected={:#x} got={:#x}", reg, mask, expected, got),
            Self::DriverOwnTimeout { band } =>
                write!(f, "driver_own timeout band={}", band),
        }
    }
}

/// MT7996 fixed register map — direct BAR offset for high addresses.
/// Source: mmio.c:144-194 mt7996_reg_map[] (MT7996 variant)
/// Format: (phys_addr, mapped_offset, size)
const FIXED_MAP: &[(u32, u32, u32)] = &[
    (0x54000000, 0x02000, 0x1000),  // WFDMA_0 (PCIE0 MCU DMA0)
    (0x55000000, 0x03000, 0x1000),  // WFDMA_1 (PCIE0 MCU DMA1)
    (0x56000000, 0x04000, 0x1000),  // WFDMA reserved
    (0x57000000, 0x05000, 0x1000),  // WFDMA MCU wrap CR
    (0x58000000, 0x06000, 0x1000),  // WFDMA PCIE1 MCU DMA0 (MEM_DMA)
    (0x59000000, 0x07000, 0x1000),  // WFDMA PCIE1 MCU DMA1
    (0x820c0000, 0x08000, 0x4000),  // WF_UMAC_TOP (PLE)
    (0x820c8000, 0x0c000, 0x2000),  // WF_UMAC_TOP (PSE)
    (0x820cc000, 0x0e000, 0x1000),  // WF_UMAC_TOP (PP/MDP)
    (0x74030000, 0x10000, 0x1000),  // PCIe MAC
    (0x820e0000, 0x20000, 0x0400),  // WF_LMAC_TOP BN0 (WF_CFG)
    (0x820e1000, 0x20400, 0x0200),  // WF_LMAC_TOP BN0 (WF_TRB)
    (0x820e2000, 0x20800, 0x0400),  // WF_LMAC_TOP BN0 (WF_AGG)
    (0x820e3000, 0x20c00, 0x0400),  // WF_LMAC_TOP BN0 (WF_ARB)
    (0x820e4000, 0x21000, 0x0400),  // WF_LMAC_TOP BN0 (WF_TMAC)
    (0x820e5000, 0x21400, 0x0800),  // WF_LMAC_TOP BN0 (WF_RMAC)
    (0x820ce000, 0x21c00, 0x0200),  // WF_LMAC_TOP (WF_SEC)
    (0x820e7000, 0x21e00, 0x0200),  // WF_LMAC_TOP BN0 (WF_DMA)
    (0x820cf000, 0x22000, 0x1000),  // WF_LMAC_TOP (WF_PF)
    (0x820e9000, 0x23400, 0x0200),  // WF_LMAC_TOP BN0 (WF_WTBLOFF)
    (0x820ea000, 0x24000, 0x0200),  // WF_LMAC_TOP BN0 (WF_ETBF)
    (0x820eb000, 0x24200, 0x0400),  // WF_LMAC_TOP BN0 (WF_LPON)
    (0x820ec000, 0x24600, 0x0200),  // WF_LMAC_TOP BN0 (WF_INT)
    (0x820ed000, 0x24800, 0x0800),  // WF_LMAC_TOP BN0 (WF_MIB)
    (0x820ca000, 0x26000, 0x2000),  // WF_LMAC_TOP BN0 (WF_MUCOP)
    (0x820d0000, 0x30000, 0x10000), // WF_LMAC_TOP (WF_WTBLON)
    (0x40000000, 0x70000, 0x10000), // WF_UMAC_SYSRAM
    (0x00400000, 0x80000, 0x10000), // WF_MCU_SYSRAM
    (0x00410000, 0x90000, 0x10000), // WF_MCU_SYSRAM (configure register)
    (0x820f0000, 0xa0000, 0x0400),  // WF_LMAC_TOP BN1 (WF_CFG)
    (0x820f1000, 0xa0600, 0x0200),  // WF_LMAC_TOP BN1 (WF_TRB)
    (0x820f2000, 0xa0800, 0x0400),  // WF_LMAC_TOP BN1 (WF_AGG)
    (0x820f3000, 0xa0c00, 0x0400),  // WF_LMAC_TOP BN1 (WF_ARB)
    (0x820f4000, 0xa1000, 0x0400),  // WF_LMAC_TOP BN1 (WF_TMAC)
    (0x820f5000, 0xa1400, 0x0800),  // WF_LMAC_TOP BN1 (WF_RMAC)
    (0x820f7000, 0xa1e00, 0x0200),  // WF_LMAC_TOP BN1 (WF_DMA)
    (0x820f9000, 0xa3400, 0x0200),  // WF_LMAC_TOP BN1 (WF_WTBLOFF)
    (0x820fa000, 0xa4000, 0x0200),  // WF_LMAC_TOP BN1 (WF_ETBF)
    (0x820fb000, 0xa4200, 0x0400),  // WF_LMAC_TOP BN1 (WF_LPON)
    (0x820fc000, 0xa4600, 0x0200),  // WF_LMAC_TOP BN1 (WF_INT)
    (0x820fd000, 0xa4800, 0x0800),  // WF_LMAC_TOP BN1 (WF_MIB)
    (0x820cc000, 0xa5000, 0x2000),  // WF_LMAC_TOP BN1 (WF_MUCOP)
    (0x820c4000, 0xa8000, 0x4000),  // WF_LMAC_TOP BN1 (WF_MUCOP)
    (0x820b0000, 0xae000, 0x1000),  // [APB2] WFSYS_ON
    (0x80020000, 0xb0000, 0x10000), // WF_TOP_MISC_OFF
    (0x81020000, 0xc0000, 0x10000), // WF_TOP_MISC_ON
    (0x7c020000, 0xd0000, 0x10000), // CONN_INFRA, wfdma
    (0x7c060000, 0xe0000, 0x10000), // CONN_INFRA, conn_host_csr_top
    (0x7c000000, 0xf0000, 0x10000), // CONN_INFRA
];

/// MT7996 device — BAR0 register access with address translation.
pub struct Mt76Device {
    bar0_base: u64,
    bar0_size: u64,
    pub has_hif2: bool,
}

impl Mt76Device {
    pub fn new(bar0_base: u64, bar0_size: u64, has_hif2: bool) -> Self {
        Self { bar0_base, bar0_size, has_hif2 }
    }

    // ========================================================================
    // Raw register access — direct BAR0 offsets
    // ========================================================================

    /// Write 32-bit value at BAR0 offset.
    #[inline]
    pub fn wr(&self, reg: u32, val: u32) {
        let offset = reg as usize;
        if offset < self.bar0_size as usize {
            unsafe {
                let ptr = (self.bar0_base as *mut u32).add(offset / 4);
                core::ptr::write_volatile(ptr, val);
            }
        }
    }

    /// Read 32-bit value at BAR0 offset.
    #[inline]
    pub fn rr(&self, reg: u32) -> u32 {
        let offset = reg as usize;
        if offset < self.bar0_size as usize {
            unsafe {
                let ptr = (self.bar0_base as *const u32).add(offset / 4);
                core::ptr::read_volatile(ptr)
            }
        } else {
            0xFFFF_FFFF
        }
    }

    /// Read-modify-write: set bits (read | bits).
    #[inline]
    pub fn set(&self, reg: u32, bits: u32) {
        let val = self.rr(reg);
        self.wr(reg, val | bits);
    }

    /// Read-modify-write: clear bits (read & !bits).
    #[inline]
    pub fn clear(&self, reg: u32, bits: u32) {
        let val = self.rr(reg);
        self.wr(reg, val & !bits);
    }

    /// Read-modify-write: masked replacement.
    #[inline]
    pub fn rmw(&self, reg: u32, mask: u32, val: u32) {
        let old = self.rr(reg);
        self.wr(reg, (old & !mask) | val);
    }

    /// Poll register until (read & mask) == expected, or timeout.
    /// Returns Ok(value) on success, Err(DeviceError) on timeout.
    ///
    /// Uses 1ms sleep per iteration to avoid syscall storms from
    /// `delay_us(10)` spinning on `gettime()`. Hardware registers
    /// typically clear BUSY in < 1μs, so the first read usually succeeds.
    pub fn poll(&self, reg: u32, mask: u32, expected: u32, timeout_us: u32) -> Result<u32, DeviceError> {
        // First try without sleeping — hardware is usually ready immediately
        let val = self.rr(reg);
        if (val & mask) == expected {
            return Ok(val);
        }

        // Fall back to 1ms sleep loop
        let timeout_ms = (timeout_us / 1000).max(1);
        for _ in 0..timeout_ms {
            let val = self.rr(reg);
            if (val & mask) == expected {
                return Ok(val);
            }
            userlib::delay_ms(1);
        }
        let got = self.rr(reg) & mask;
        Err(DeviceError::PollTimeout { reg, mask, expected, got })
    }

    /// Poll register with millisecond granularity.
    pub fn poll_msec(&self, reg: u32, mask: u32, expected: u32, timeout_ms: u32) -> Result<u32, DeviceError> {
        for _ in 0..timeout_ms.max(1) {
            let val = self.rr(reg);
            if (val & mask) == expected {
                return Ok(val);
            }
            userlib::delay_ms(1);
        }
        let got = self.rr(reg) & mask;
        Err(DeviceError::PollTimeout { reg, mask, expected, got })
    }

    // ========================================================================
    // Translated register access — FIXED_MAP + L1/L2 remap
    // ========================================================================

    /// Translate a register address through the fixed map table.
    /// Addresses < 0x100000 are BAR offsets, used as-is.
    /// Source: mmio.c:317-362
    fn translate(&self, addr: u32) -> u32 {
        if addr < 0x100000 {
            return addr;
        }
        for &(phys, mapped, size) in FIXED_MAP {
            if addr >= phys && (addr - phys) < size {
                return mapped + (addr - phys);
            }
        }
        self.remap(addr)
    }

    /// Route unmapped addresses to L1 or L2 remap.
    /// Source: mmio.c:340-362
    fn remap(&self, addr: u32) -> u32 {
        // L1: INFRA/WFSYS 0x18000000..=0x18bfffff
        if addr >= MT_INFRA_BASE && addr <= MT_WFSYS1_PHY_END {
            return self.remap_l1(addr);
        }
        // L1: CONN_INFRA MCU
        if addr >= MT_INFRA_MCU_START && addr <= MT_INFRA_MCU_END {
            return self.remap_l1(addr - MT_INFRA_MCU_START + MT_INFRA_BASE);
        }
        // L1: CBTOP 0x70000000..=0x77ffffff
        if addr >= 0x70000000 && addr <= MT_CBTOP1_PHY_END {
            return self.remap_l1(addr);
        }
        // L2: everything else (PHYRX 0x83xxxxxx, etc)
        self.remap_l2(addr)
    }

    /// L1 remap: 16-bit base field, 64KB window at HIF_REMAP_BASE_L1.
    /// Source: mmio.c mt7996_reg_map_l1()
    fn remap_l1(&self, addr: u32) -> u32 {
        let offset = addr & 0xFFFF;
        let base = (addr >> 16) & 0xFFFF;

        let current = self.rr(MT_HIF_REMAP_L1);
        self.wr(MT_HIF_REMAP_L1, (current & 0x0000FFFF) | (base << 16));
        let _ = self.rr(MT_HIF_REMAP_L1); // push write

        HIF_REMAP_BASE_L1 + offset
    }

    /// L2 remap: 4KB window at HIF_REMAP_BASE_L2.
    /// Source: mmio.c:280-301 mt7996_reg_map_l2()
    fn remap_l2(&self, addr: u32) -> u32 {
        let offset = addr & 0xFFF;
        let base = addr >> 12;
        let l2_mask: u32 = 0x000F_FFFF;

        let current = self.rr(MT_HIF_REMAP_L2);
        self.wr(MT_HIF_REMAP_L2, (current & !l2_mask) | (base & l2_mask));
        let _ = self.rr(MT_HIF_REMAP_L2); // push write

        HIF_REMAP_BASE_L2 + offset
    }

    /// Write to a translated (high) register address.
    pub fn reg_wr(&self, addr: u32, val: u32) {
        self.wr(self.translate(addr), val);
    }

    /// Read from a translated (high) register address.
    pub fn reg_rr(&self, addr: u32) -> u32 {
        self.rr(self.translate(addr))
    }

    /// Set bits at a translated address.
    pub fn reg_set(&self, addr: u32, bits: u32) {
        let mapped = self.translate(addr);
        self.set(mapped, bits);
    }

    /// Clear bits at a translated address.
    pub fn reg_clear(&self, addr: u32, bits: u32) {
        let mapped = self.translate(addr);
        self.clear(mapped, bits);
    }

    /// Read-modify-write at a translated address.
    pub fn reg_rmw(&self, addr: u32, mask: u32, val: u32) {
        let mapped = self.translate(addr);
        self.rmw(mapped, mask, val);
    }

    /// Poll a translated register.
    pub fn reg_poll(&self, addr: u32, mask: u32, val: u32, timeout_us: u32) -> Result<u32, DeviceError> {
        let mapped = self.translate(addr);
        self.poll(mapped, mask, val, timeout_us)
    }

    // ========================================================================
    // L1 remap convenience (for addresses known to need L1)
    // ========================================================================

    /// Write via L1 remap.
    pub fn wr_remap(&self, addr: u32, val: u32) {
        self.wr(self.remap_l1(addr), val);
    }

    /// Read via L1 remap.
    pub fn rr_remap(&self, addr: u32) -> u32 {
        self.rr(self.remap_l1(addr))
    }

    /// Set bits via L1 remap.
    pub fn set_remap(&self, addr: u32, bits: u32) {
        let mapped = self.remap_l1(addr);
        self.set(mapped, bits);
    }

    /// Clear bits via L1 remap.
    pub fn clear_remap(&self, addr: u32, bits: u32) {
        let mapped = self.remap_l1(addr);
        self.clear(mapped, bits);
    }

    // ========================================================================
    // Hardware operations
    // ========================================================================

    /// WFSYS reset — set + clear WF_SUBSYS_RST with 20ms delays.
    /// Source: init.c
    pub fn wfsys_reset(&self) {
        self.set_remap(MT_WF_SUBSYS_RST, 0x1);
        userlib::delay_ms(20);
        self.clear_remap(MT_WF_SUBSYS_RST, 0x1);
        userlib::delay_ms(20);
    }

    /// Claim driver ownership from firmware for a band.
    /// Source: mcu.c:3559-3572
    pub fn driver_own(&self, band: u32) -> Result<(), DeviceError> {
        self.wr(mt_top_lpcr_host_band(band), MT_TOP_LPCR_HOST_DRV_OWN);

        self.poll_msec(
            mt_top_lpcr_host_band(band),
            MT_TOP_LPCR_HOST_FW_OWN_STAT,
            0,
            500,
        ).map_err(|_| {
            uerror!("device", "driver_own_timeout"; band = band);
            DeviceError::DriverOwnTimeout { band }
        })?;

        self.wr(
            mt_top_lpcr_host_band_irq_stat(band),
            MT_TOP_LPCR_HOST_BAND_STAT,
        );

        Ok(())
    }

    /// Read and clear all MIB counters for a band.
    /// All MIB registers are read-to-clear — reading prevents counter overflow.
    /// Source: mac.c:2743-2882
    pub fn update_mib_stats(&self, band: usize) {
        let mib = WF_MIB_BASE[band];

        // Core counters
        let _ = self.reg_rr(mib + MT_MIB_RSCR1_OFS);
        let _ = self.reg_rr(mib + MT_MIB_RSCR33_OFS);
        let _ = self.reg_rr(mib + MT_MIB_RSCR31_OFS);
        let _ = self.reg_rr(mib + MT_MIB_SDR6_OFS);
        let _ = self.reg_rr(mib + MT_MIB_RVSR0_OFS);
        let _ = self.reg_rr(mib + MT_MIB_RSCR35_OFS);
        let _ = self.reg_rr(mib + MT_MIB_RSCR36_OFS);

        // TX counters
        let _ = self.reg_rr(mib + MT_MIB_TSCR0_OFS);
        let _ = self.reg_rr(mib + MT_MIB_TSCR2_OFS);

        // RX AMPDU
        let _ = self.reg_rr(mib + MT_MIB_RSCR27_OFS);
        let _ = self.reg_rr(mib + MT_MIB_RSCR28_OFS);
        let _ = self.reg_rr(mib + MT_MIB_RSCR29_OFS);
        let _ = self.reg_rr(mib + MT_MIB_RSCR30_OFS);

        // TX RWP
        let _ = self.reg_rr(mib + MT_MIB_SDR27_OFS);
        let _ = self.reg_rr(mib + MT_MIB_SDR28_OFS);

        // RX pf drop (UMIB)
        let _ = self.reg_rr(mt_umib_rpdcr(band as u32));

        // RX vec queue overflow
        let _ = self.reg_rr(mib + MT_MIB_RVSR1_OFS);

        // RX BA
        let _ = self.reg_rr(mib + MT_MIB_TSCR1_OFS);

        // Beamforming
        let _ = self.reg_rr(mib + MT_MIB_BSCR0_OFS);
        let _ = self.reg_rr(mib + MT_MIB_BSCR1_OFS);
        let _ = self.reg_rr(mib + MT_MIB_BSCR2_OFS);
        let _ = self.reg_rr(mib + MT_MIB_TSCR5_OFS);
        let _ = self.reg_rr(mib + MT_MIB_TSCR6_OFS);
        let _ = self.reg_rr(mib + MT_MIB_TSCR7_OFS);
        let _ = self.reg_rr(mib + MT_MIB_BSCR3_OFS);
        let _ = self.reg_rr(mib + MT_MIB_BSCR4_OFS);
        let _ = self.reg_rr(mib + MT_MIB_BSCR5_OFS);
        let _ = self.reg_rr(mib + MT_MIB_BSCR6_OFS);

        // ETBF feedback
        let _ = self.reg_rr(mt_etbf_rx_fb_cont(band));

        // BF trigger
        let _ = self.reg_rr(mib + MT_MIB_BSCR7_OFS);
        let _ = self.reg_rr(mib + MT_MIB_BSCR17_OFS);

        // TX AGG counts (16 entries)
        for i in 0..16u32 {
            let _ = self.reg_rr(mib + MT_MIB_TRDR1_OFS + (i << 2));
        }

        // ACK fail
        let _ = self.reg_rr(mib + MT_MIB_BFTFCR_OFS);
    }
}
