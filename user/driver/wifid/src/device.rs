//! MT7996 Device — MMIO register access, hw_info, WFSYS reset
//!
//! Mt7996Dev holds the BAR0 base pointer and provides all register access
//! methods. Other modules (dma, mcu, firmware) take `&Mt7996Dev` to read/write
//! registers — matching the Linux pattern where `dev` is threaded through
//! everything.

use userlib::{uinfo, uwarn, uerror, udebug};
use crate::regs::*;
use crate::dma::DmaConfig;

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
