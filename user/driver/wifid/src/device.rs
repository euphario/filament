//! MT7996 Device — MMIO register access, hw_info, WFSYS reset
//!
//! Mt7996Dev holds the BAR0 base pointer and provides all register access
//! methods. Other modules (dma, mcu, firmware) take `&Mt7996Dev` to read/write
//! registers — matching the Linux pattern where `dev` is threaded through
//! everything.

use userlib::{uinfo, uwarn, uerror, udebug};
use crate::regs::*;
use crate::dma::DmaConfig;

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

/// MT7996 device state
pub struct Mt7996Dev {
    pub(crate) bar0_base: u64,
    pub(crate) bar0_size: u64,
    pub(crate) has_hif2: bool,
    pub(crate) config: DmaConfig,
}

impl Mt7996Dev {
    pub fn new(bar0_base: u64, bar0_size: u64, has_hif2: bool) -> Self {
        // MT7996 has TWO PCIe devices but HIF2 registers (0xd8xxx) are accessible
        // through HIF1's BAR at offset 0xd8xxx. Linux mmio.c __mt7996_reg_addr():
        //   if (addr < 0x100000) return addr;
        // So 0xd8xxx is accessed via HIF1_BAR + 0xd8xxx, not through HIF2's separate BAR.
        Self {
            bar0_base,
            bar0_size,
            has_hif2,
            config: DmaConfig::new(has_hif2),
        }
    }

    // ========================================================================
    // Register access — EXACT mt76 equivalents
    // ========================================================================

    #[inline]
    pub fn mt76_wr(&self, reg: u32, val: u32) {
        let offset = reg as usize;
        if offset < self.bar0_size as usize {
            unsafe {
                let ptr = (self.bar0_base as *mut u32).add(offset / 4);
                core::ptr::write_volatile(ptr, val);
            }
        }
    }

    #[inline]
    pub fn mt76_rr(&self, reg: u32) -> u32 {
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

    #[inline]
    pub fn mt76_set(&self, reg: u32, bits: u32) {
        let val = self.mt76_rr(reg);
        self.mt76_wr(reg, val | bits);
    }

    #[inline]
    pub fn mt76_clear(&self, reg: u32, bits: u32) {
        let val = self.mt76_rr(reg);
        self.mt76_wr(reg, val & !bits);
    }

    #[inline]
    pub fn mt76_rmw(&self, reg: u32, mask: u32, val: u32) {
        let old = self.mt76_rr(reg);
        self.mt76_wr(reg, (old & !mask) | val);
    }

    pub fn mt76_poll(&self, reg: u32, mask: u32, val: u32, timeout_us: u32) -> bool {
        let iterations = timeout_us / 10;
        for _ in 0..iterations.max(1) {
            if (self.mt76_rr(reg) & mask) == val {
                return true;
            }
            userlib::delay_us(10);
        }
        false
    }

    pub fn mt76_poll_msec(&self, reg: u32, mask: u32, val: u32, timeout_ms: u32) -> bool {
        for _ in 0..timeout_ms.max(1) {
            if (self.mt76_rr(reg) & mask) == val {
                return true;
            }
            userlib::delay_ms(1);
        }
        false
    }

    // ========================================================================
    // Fixed Register Map + Translated Access — EXACT Linux translation
    // Source: mmio.c:mt7996_reg_map[] (lines 144-194), __mt7996_reg_addr()
    //
    // Band registers (0x820eXXXX for band 0, 0x820fXXXX for band 1,
    // 0x830eXXXX for band 2) need address translation. The fixed map table
    // provides direct BAR offsets for mapped regions. Unmapped addresses
    // fall back to L1 remap.
    //
    // DMA code continues using mt76_wr/mt76_rr (raw BAR offsets < 0x100000).
    // MAC init code uses reg_wr/reg_rr/reg_set/reg_clear/reg_rmw (translated).
    // ========================================================================

    /// Translate a register address through the fixed map table.
    /// If addr < 0x100000, returns as-is (already a BAR offset).
    /// Otherwise searches FIXED_MAP, then routes to L1 or L2 remap.
    /// Source: mmio.c:317-338 __mt7996_reg_addr(), 340-362 __mt7996_reg_remap_addr()
    fn reg_addr(&self, addr: u32) -> u32 {
        if addr < 0x100000 {
            return addr;
        }
        for &(phys, mapped, size) in FIXED_MAP {
            if addr >= phys && (addr - phys) < size {
                return mapped + (addr - phys);
            }
        }
        self.reg_remap_addr(addr)
    }

    /// Route unmapped addresses to L1 or L2 remap based on address ranges.
    /// Source: mmio.c:340-362 __mt7996_reg_remap_addr()
    fn reg_remap_addr(&self, addr: u32) -> u32 {
        // L1: INFRA/WFSYS range 0x18000000..=0x18bfffff
        if addr >= MT_INFRA_BASE && addr <= MT_WFSYS1_PHY_END {
            return self.mt7996_reg_map_l1(addr);
        }
        // L1: CONN_INFRA MCU → convert to physical addr
        if addr >= MT_INFRA_MCU_START && addr <= MT_INFRA_MCU_END {
            return self.mt7996_reg_map_l1(addr - MT_INFRA_MCU_START + MT_INFRA_BASE);
        }
        // L1: CBTOP 0x70000000..=0x77ffffff (PCIe, MT7996)
        if addr >= 0x70000000 && addr <= MT_CBTOP1_PHY_END {
            return self.mt7996_reg_map_l1(addr);
        }
        // L2: everything else (including PHYRX 0x83xxxxxx)
        self.mt7996_reg_map_l2(addr)
    }

    /// Write to a translated register address
    pub fn reg_wr(&self, addr: u32, val: u32) {
        self.mt76_wr(self.reg_addr(addr), val);
    }

    /// Read from a translated register address
    pub fn reg_rr(&self, addr: u32) -> u32 {
        self.mt76_rr(self.reg_addr(addr))
    }

    /// Set bits at a translated register address (read | bits)
    pub fn reg_set(&self, addr: u32, bits: u32) {
        let mapped = self.reg_addr(addr);
        let val = self.mt76_rr(mapped);
        self.mt76_wr(mapped, val | bits);
    }

    /// Clear bits at a translated register address (read & ~bits)
    pub fn reg_clear(&self, addr: u32, bits: u32) {
        let mapped = self.reg_addr(addr);
        let val = self.mt76_rr(mapped);
        self.mt76_wr(mapped, val & !bits);
    }

    /// Read-modify-write at a translated register address
    pub fn reg_rmw(&self, addr: u32, mask: u32, val: u32) {
        let mapped = self.reg_addr(addr);
        let old = self.mt76_rr(mapped);
        self.mt76_wr(mapped, (old & !mask) | val);
    }

    /// Poll a translated register for expected value
    pub fn reg_poll(&self, addr: u32, mask: u32, val: u32, timeout_us: u32) -> bool {
        let mapped = self.reg_addr(addr);
        self.mt76_poll(mapped, mask, val, timeout_us)
    }

    // ========================================================================
    // L1 Address Remapping — EXACT Linux translation
    // Source: mmio.c:mt7996_reg_map_l1()
    // ========================================================================

    /// Remap a high address via L1 remap mechanism
    pub fn mt7996_reg_map_l1(&self, addr: u32) -> u32 {
        let offset = addr & 0xFFFF;
        let base = (addr >> 16) & 0xFFFF;
        let l1_mask: u32 = 0xFFFF0000;
        let val = base << 16;

        let current = self.mt76_rr(MT_HIF_REMAP_L1);
        let new_val = (current & !l1_mask) | val;
        self.mt76_wr(MT_HIF_REMAP_L1, new_val);

        // Read back to push write (memory barrier)
        let _ = self.mt76_rr(MT_HIF_REMAP_L1);

        HIF_REMAP_BASE_L1 + offset
    }

    // ========================================================================
    // L2 Address Remapping — EXACT Linux translation
    // Source: mmio.c:280-301 mt7996_reg_map_l2()
    //
    // L2 has a 4KB window at BAR offset 0x1000 (HIF_REMAP_BASE_L2).
    // Addr bits[31:12] are written to the remap register at 0x1b4.
    // Addr bits[11:0] become the offset within the window.
    // ========================================================================

    /// Remap a high address via L2 remap mechanism
    pub fn mt7996_reg_map_l2(&self, addr: u32) -> u32 {
        // MT7996 (not mt7990):
        // offset = FIELD_GET(GENMASK(11,0), addr)
        // base = FIELD_GET(GENMASK(31,12), addr)
        // l2_mask = GENMASK(19,0)
        // val = FIELD_PREP(GENMASK(19,0), base)
        let offset = addr & 0xFFF;
        let base = addr >> 12;
        let l2_mask: u32 = 0x000F_FFFF; // GENMASK(19,0)
        let val = base & 0x000F_FFFF;

        let current = self.mt76_rr(MT_HIF_REMAP_L2);
        self.mt76_wr(MT_HIF_REMAP_L2, (current & !l2_mask) | val);
        // Read back to push write (memory barrier)
        let _ = self.mt76_rr(MT_HIF_REMAP_L2);

        HIF_REMAP_BASE_L2 + offset
    }

    /// Write to a high address using L1 remap
    pub fn mt76_wr_remap(&self, addr: u32, val: u32) {
        let remapped = self.mt7996_reg_map_l1(addr);
        self.mt76_wr(remapped, val);
    }

    /// Read from a high address using L1 remap
    pub fn mt76_rr_remap(&self, addr: u32) -> u32 {
        let remapped = self.mt7996_reg_map_l1(addr);
        self.mt76_rr(remapped)
    }

    /// Set bits in a high address using L1 remap
    pub fn mt76_set_remap(&self, addr: u32, bits: u32) {
        let remapped = self.mt7996_reg_map_l1(addr);
        let val = self.mt76_rr(remapped);
        self.mt76_wr(remapped, val | bits);
    }

    /// Clear bits in a high address using L1 remap
    pub fn mt76_clear_remap(&self, addr: u32, bits: u32) {
        let remapped = self.mt7996_reg_map_l1(addr);
        let val = self.mt76_rr(remapped);
        self.mt76_wr(remapped, val & !bits);
    }

    // ========================================================================
    // mt7996_wfsys_reset() — EXACT Linux translation
    // Source: init.c
    // ========================================================================

    pub fn mt7996_wfsys_reset(&self) {
        self.mt76_set_remap(MT_WF_SUBSYS_RST, 0x1);
        userlib::delay_ms(20);
        self.mt76_clear_remap(MT_WF_SUBSYS_RST, 0x1);
        userlib::delay_ms(20);
    }

    // ========================================================================
    // mt7996_driver_own() — EXACT Linux translation
    // Source: mcu.c:3559-3572
    // ========================================================================

    pub fn mt7996_driver_own(&self, band: u32) -> Result<(), i32> {
        self.mt76_wr(mt_top_lpcr_host_band(band), MT_TOP_LPCR_HOST_DRV_OWN);

        if !self.mt76_poll_msec(
            mt_top_lpcr_host_band(band),
            MT_TOP_LPCR_HOST_FW_OWN_STAT,
            0,
            500,
        ) {
            uerror!("init", "driver_own_timeout"; band = band);
            return Err(-1);
        }

        self.mt76_wr(
            mt_top_lpcr_host_band_irq_stat(band),
            MT_TOP_LPCR_HOST_BAND_STAT,
        );

        Ok(())
    }
}
