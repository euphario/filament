//! MAC Setup — WTBL clearing, per-band register init, basic rate tables, MIB clearing
//!
//! Free functions operating on Mt76Device, ported from wifid/mac.rs.
//! Source: Linux mt7996/init.c:541-638, mac.c:97-104, mac.c:2062-2132, mac.c:2743-2882
//!
//! IMPORTANT: All registers in this module are high addresses (0x820xxxxx)
//! requiring FIXED_MAP translation. Use reg_* methods, NOT raw rr/wr.

use crate::device::Mt76Device;
use crate::dma::TxRing;
use crate::mcu::{self, McuError, FwIrq};
use crate::regs::*;

// ============================================================================
// Small MAC helpers
// ============================================================================

/// Enable noise floor measurement for a band.
/// Source: mac.c:2124-2132 mt7996_mac_enable_nf()
pub fn enable_nf(dev: &Mt76Device, band: u32) {
    dev.reg_set(
        mt_wf_phyrx_band(band, MT_WF_PHYRX_CSD_BAND_RXTD12_OFS),
        MT_WF_PHYRX_CSD_BAND_RXTD12_IRPI_SW_CLR_ONLY
            | MT_WF_PHYRX_CSD_BAND_RXTD12_IRPI_SW_CLR,
    );
    dev.reg_set(
        mt_wf_phyrx_band(band, MT_WF_PHYRX_BAND_RX_CTRL1_OFS),
        0x5, // IPI_EN
    );
}

/// Reset CCA stats for a band.
/// Source: mac.c:2062-2069 mt7996_mac_cca_stats_reset()
pub fn cca_stats_reset(dev: &Mt76Device, band: u32) {
    let reg = mt_wf_phyrx_band(band, MT_WF_PHYRX_BAND_RX_CTRL1_OFS);
    dev.reg_clear(reg, MT_WF_PHYRX_BAND_RX_CTRL1_STSCNT_EN);
    dev.reg_set(reg, (1 << 11) | (1 << 9));
}

/// Update a WTBL entry's admin count.
/// Source: mac.c:97-104 mt7996_mac_wtbl_update()
pub fn wtbl_update(dev: &Mt76Device, idx: u32, mask: u32) -> bool {
    dev.reg_rmw(MT_WTBL_UPDATE, MT_WTBL_UPDATE_WLAN_IDX,
            (idx & MT_WTBL_UPDATE_WLAN_IDX) | mask);
    dev.reg_poll(MT_WTBL_UPDATE, MT_WTBL_UPDATE_BUSY, 0, 5000).is_ok()
}

/// Read-to-clear all hardware MIB counters for a band.
/// Source: mac.c:2743-2882 mt7996_mac_update_stats()
///
/// Without periodic clearing, hardware counters wrap and skew firmware stats.
pub fn update_stats(dev: &Mt76Device, band: usize) {
    let mib = WF_MIB_BASE[band];

    // Core counters
    let _ = dev.reg_rr(mib + MT_MIB_RSCR1_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_RSCR33_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_RSCR31_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_SDR6_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_RVSR0_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_RSCR35_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_RSCR36_OFS);

    // TX counters
    let _ = dev.reg_rr(mib + MT_MIB_TSCR0_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_TSCR2_OFS);

    // RX AMPDU
    let _ = dev.reg_rr(mib + MT_MIB_RSCR27_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_RSCR28_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_RSCR29_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_RSCR30_OFS);

    // TX RWP
    let _ = dev.reg_rr(mib + MT_MIB_SDR27_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_SDR28_OFS);

    // RX pf drop (UMIB register, different base)
    let _ = dev.reg_rr(mt_umib_rpdcr(band as u32));

    // RX vec queue overflow
    let _ = dev.reg_rr(mib + MT_MIB_RVSR1_OFS);

    // RX BA
    let _ = dev.reg_rr(mib + MT_MIB_TSCR1_OFS);

    // Beamforming
    let _ = dev.reg_rr(mib + MT_MIB_BSCR0_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_BSCR1_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_BSCR2_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_TSCR5_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_TSCR6_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_TSCR7_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_BSCR3_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_BSCR4_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_BSCR5_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_BSCR6_OFS);

    // ETBF feedback
    let _ = dev.reg_rr(mt_etbf_rx_fb_cont(band));

    // BF trigger/completion
    let _ = dev.reg_rr(mib + MT_MIB_BSCR7_OFS);
    let _ = dev.reg_rr(mib + MT_MIB_BSCR17_OFS);

    // TX AGG counts (16 entries)
    for i in 0..16u32 {
        let _ = dev.reg_rr(mib + MT_MIB_TRDR1_OFS + (i << 2));
    }

    // RTS/ACK fail
    let _ = dev.reg_rr(mib + MT_MIB_BFTFCR_OFS);
}

// ============================================================================
// Per-band register init
// ============================================================================

/// Per-band register initialization.
/// Source: init.c:541-576 mt7996_mac_init_band()
fn init_band(dev: &Mt76Device, band: usize) {
    // Clear estimated EIFS value for Rx duration & OBSS time
    dev.reg_wr(mt_wf_rmac(band, MT_WF_RMAC_RSVD0_OFS), MT_WF_RMAC_RSVD0_EIFS_CLR);

    // Clear backoff time for Rx duration
    dev.reg_clear(mt_wf_rmac(band, MT_WF_RMAC_MIB_AIRTIME1_OFS), MT_WF_RMAC_MIB_NONQOSD_BACKOFF);
    dev.reg_clear(mt_wf_rmac(band, MT_WF_RMAC_MIB_AIRTIME3_OFS), MT_WF_RMAC_MIB_QOS01_BACKOFF);
    dev.reg_clear(mt_wf_rmac(band, MT_WF_RMAC_MIB_AIRTIME4_OFS), MT_WF_RMAC_MIB_QOS23_BACKOFF);

    // Clear backoff time for Tx duration
    dev.reg_clear(mt_wtbloff(band, MT_WTBLOFF_ACR_OFS), MT_WTBLOFF_ADM_BACKOFFTIME);

    // Clear backoff time and set software compensation for OBSS time
    let mask = MT_WF_RMAC_MIB_OBSS_BACKOFF | MT_WF_RMAC_MIB_ED_OFFSET;
    let set = 4 << 16; // OBSS_BACKOFF=0, ED_OFFSET=4 at bits[20:16]
    dev.reg_rmw(mt_wf_rmac(band, MT_WF_RMAC_MIB_AIRTIME0_OFS), mask, set);

    // Filter out non-resp frames, instantaneous signal reporting
    let mask = MT_WTBLOFF_RSCR_RCPI_MODE | MT_WTBLOFF_RSCR_RCPI_PARAM;
    let set = 0x3 << 24; // RCPI_MODE=0, RCPI_PARAM=3 at bits[25:24]
    dev.reg_rmw(mt_wtbloff(band, MT_WTBLOFF_RSCR_OFS), mask, set);

    // PPDU TXS to host
    dev.reg_set(mt_wf_agg(band, MT_AGG_ACR4_OFS), MT_AGG_ACR_PPDU_TXS2H);
}

// ============================================================================
// Basic rate programming
// ============================================================================

/// Program 12 basic rates (4 CCK + 8 OFDM) into firmware rate table.
/// Source: init.c:578-591 mt7996_mac_init_basic_rates()
fn init_basic_rates(
    dev: &Mt76Device, ring: &mut TxRing,
    seq: &mut u8, mut irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    const HW_VALUES: [u16; 12] = [
        0x0000, 0x0001, 0x0002, 0x0003, // CCK
        0x010b, 0x010f, 0x010a, 0x010e, // OFDM
        0x0109, 0x010d, 0x0108, 0x010c,
    ];

    for (i, &hw_val) in HW_VALUES.iter().enumerate() {
        let idx = (MT7996_BASIC_RATES_TBL as u16) + 2 * (i as u16);
        let mode = (hw_val >> 8) as u16;
        let rate_bits = (hw_val & 0xFF) as u16;
        let rate = (mode << 6) | (rate_bits & 0x3F);

        mcu::set_fixed_rate_table(dev, ring, idx as u8, rate, false, 0, *seq, irq.as_deref_mut())?;
        *seq = seq.wrapping_add(1);
    }
    Ok(())
}

// ============================================================================
// Full MAC init entry point
// ============================================================================

/// Full MAC initialization.
/// Source: init.c:593-638 mt7996_mac_init()
///
/// 1. Clear MDP_DCR2.RX_TRANS_SHORT
/// 2. Clear WTBL admin count for all entries
/// 3. RRO module init
/// 4. WA HIF TXD version
/// 5. Per-band register init (bands 0, 1, 2)
/// 6. Basic rate programming
pub fn init(
    dev: &Mt76Device, ring: &mut TxRing,
    seq: &mut u8, mut irq: Option<&mut FwIrq>,
) -> Result<(), McuError> {
    dev.reg_clear(MT_MDP_DCR2, MT_MDP_DCR2_RX_TRANS_SHORT);

    // WTBL clear
    let wtbl_size = (MT7996_WTBL_SIZE_GROUP << 8) + MT7996_WTBL_BMC_SIZE;
    for i in 0..wtbl_size {
        wtbl_update(dev, i, MT_WTBL_UPDATE_ADM_COUNT_CLEAR);
    }

    // RRO module init
    mcu::set_rro(dev, ring, UNI_RRO_SET_PLATFORM_TYPE, 2, *seq, irq.as_deref_mut())?;
    *seq = seq.wrapping_add(1);
    mcu::set_rro(dev, ring, UNI_RRO_SET_BYPASS_MODE, 3, *seq, irq.as_deref_mut())?;
    *seq = seq.wrapping_add(1);
    mcu::set_rro(dev, ring, UNI_RRO_SET_TXFREE_PATH, 1, *seq, irq.as_deref_mut())?;
    *seq = seq.wrapping_add(1);

    // WA HIF TXD version
    const HIF_TXD_V2_1: u32 = 0x21;
    mcu::wa_cmd(dev, ring, MCU_WA_PARAM_HW_PATH_HIF_VER, HIF_TXD_V2_1, 0, *seq)?;
    *seq = seq.wrapping_add(1);

    // Per-band init
    for band in 0..3 {
        init_band(dev, band);
    }

    // Basic rates
    init_basic_rates(dev, ring, seq, irq)?;

    Ok(())
}
