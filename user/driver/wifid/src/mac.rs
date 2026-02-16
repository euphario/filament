//! MT7996 MAC Initialization — register init + MCU commands
//!
//! EXACT translation of Linux mt7996/init.c:541-638 and mac.c:97-104.
//! Per-band register writes, WTBL clearing, RRO module setup, HIF TXD
//! version, and basic rate table programming.

use userlib::{uinfo, udebug};
use crate::regs::*;
use crate::device::Mt7996Dev;
use crate::dma::TxRing;
use crate::mcu;

impl Mt7996Dev {
    /// Enable noise floor measurement for a band.
    /// Source: mac.c:2124-2132 mt7996_mac_enable_nf()
    ///
    /// Enables IPI (Interference Power Indicator) measurement for CCA
    /// (Clear Channel Assessment) and signal detection. Without this,
    /// the radio cannot determine if the channel is clear for TX.
    pub fn mac_enable_nf(&self, band: u32) {
        // mt76_set(dev, MT_WF_PHYRX_CSD_BAND_RXTD12(band),
        //          MT_WF_PHYRX_CSD_BAND_RXTD12_IRPI_SW_CLR_ONLY |
        //          MT_WF_PHYRX_CSD_BAND_RXTD12_IRPI_SW_CLR);
        self.reg_set(
            mt_wf_phyrx_band(band, MT_WF_PHYRX_CSD_BAND_RXTD12_OFS),
            MT_WF_PHYRX_CSD_BAND_RXTD12_IRPI_SW_CLR_ONLY
                | MT_WF_PHYRX_CSD_BAND_RXTD12_IRPI_SW_CLR,
        );

        // mt76_set(dev, MT_WF_PHYRX_BAND_RX_CTRL1(band),
        //          FIELD_PREP(MT_WF_PHYRX_BAND_RX_CTRL1_IPI_EN, 0x5));
        self.reg_set(
            mt_wf_phyrx_band(band, MT_WF_PHYRX_BAND_RX_CTRL1_OFS),
            0x5, // FIELD_PREP(GENMASK(2,0), 0x5) = 0x5
        );
    }

    /// Reset CCA stats for a band.
    /// Source: mac.c:2062-2069 mt7996_mac_cca_stats_reset()
    pub fn mac_cca_stats_reset(&self, band: u32) {
        let reg = mt_wf_phyrx_band(band, MT_WF_PHYRX_BAND_RX_CTRL1_OFS);
        // mt76_clear(dev, reg, MT_WF_PHYRX_BAND_RX_CTRL1_STSCNT_EN);
        self.reg_clear(reg, MT_WF_PHYRX_BAND_RX_CTRL1_STSCNT_EN);
        // mt76_set(dev, reg, BIT(11) | BIT(9));
        self.reg_set(reg, (1 << 11) | (1 << 9));
    }

    /// Update a WTBL entry's admin count (or other mask bits).
    /// Source: mac.c:97-104 mt7996_mac_wtbl_update()
    fn mac_wtbl_update(&self, idx: u32, mask: u32) -> bool {
        // mt76_rmw(dev, MT_WTBL_UPDATE, MT_WTBL_UPDATE_WLAN_IDX,
        //          FIELD_PREP(MT_WTBL_UPDATE_WLAN_IDX, idx) | mask);
        self.reg_rmw(
            MT_WTBL_UPDATE,
            MT_WTBL_UPDATE_WLAN_IDX,
            (idx & MT_WTBL_UPDATE_WLAN_IDX) | mask,
        );

        // return mt76_poll(dev, MT_WTBL_UPDATE, MT_WTBL_UPDATE_BUSY, 0, 5000);
        self.reg_poll(MT_WTBL_UPDATE, MT_WTBL_UPDATE_BUSY, 0, 5000)
    }

    /// Per-band register initialization.
    /// Source: init.c:541-576 mt7996_mac_init_band()
    fn mac_init_band(&self, band: usize) {
        // clear estimated value of EIFS for Rx duration & OBSS time
        // mt76_wr(dev, MT_WF_RMAC_RSVD0(band), MT_WF_RMAC_RSVD0_EIFS_CLR);
        self.reg_wr(
            mt_wf_rmac(band, MT_WF_RMAC_RSVD0_OFS),
            MT_WF_RMAC_RSVD0_EIFS_CLR,
        );

        // clear backoff time for Rx duration
        // mt76_clear(dev, MT_WF_RMAC_MIB_AIRTIME1(band), MT_WF_RMAC_MIB_NONQOSD_BACKOFF);
        self.reg_clear(
            mt_wf_rmac(band, MT_WF_RMAC_MIB_AIRTIME1_OFS),
            MT_WF_RMAC_MIB_NONQOSD_BACKOFF,
        );

        // mt76_clear(dev, MT_WF_RMAC_MIB_AIRTIME3(band), MT_WF_RMAC_MIB_QOS01_BACKOFF);
        self.reg_clear(
            mt_wf_rmac(band, MT_WF_RMAC_MIB_AIRTIME3_OFS),
            MT_WF_RMAC_MIB_QOS01_BACKOFF,
        );

        // mt76_clear(dev, MT_WF_RMAC_MIB_AIRTIME4(band), MT_WF_RMAC_MIB_QOS23_BACKOFF);
        self.reg_clear(
            mt_wf_rmac(band, MT_WF_RMAC_MIB_AIRTIME4_OFS),
            MT_WF_RMAC_MIB_QOS23_BACKOFF,
        );

        // clear backoff time for Tx duration
        // mt76_clear(dev, MT_WTBLOFF_ACR(band), MT_WTBLOFF_ADM_BACKOFFTIME);
        self.reg_clear(
            mt_wtbloff(band, MT_WTBLOFF_ACR_OFS),
            MT_WTBLOFF_ADM_BACKOFFTIME,
        );

        // clear backoff time and set software compensation for OBSS time
        // mask = MT_WF_RMAC_MIB_OBSS_BACKOFF | MT_WF_RMAC_MIB_ED_OFFSET;
        // set = FIELD_PREP(MT_WF_RMAC_MIB_OBSS_BACKOFF, 0) |
        //       FIELD_PREP(MT_WF_RMAC_MIB_ED_OFFSET, 4);
        // mt76_rmw(dev, MT_WF_RMAC_MIB_AIRTIME0(band), mask, set);
        let mask = MT_WF_RMAC_MIB_OBSS_BACKOFF | MT_WF_RMAC_MIB_ED_OFFSET;
        let set = 0 | (4 << 16); // OBSS_BACKOFF=0, ED_OFFSET=4 at bits[20:16]
        self.reg_rmw(
            mt_wf_rmac(band, MT_WF_RMAC_MIB_AIRTIME0_OFS),
            mask,
            set,
        );

        // filter out non-resp frames and get instantaneous signal reporting
        // mask = MT_WTBLOFF_RSCR_RCPI_MODE | MT_WTBLOFF_RSCR_RCPI_PARAM;
        // set = FIELD_PREP(MT_WTBLOFF_RSCR_RCPI_MODE, 0) |
        //       FIELD_PREP(MT_WTBLOFF_RSCR_RCPI_PARAM, 0x3);
        // mt76_rmw(dev, MT_WTBLOFF_RSCR(band), mask, set);
        let mask = MT_WTBLOFF_RSCR_RCPI_MODE | MT_WTBLOFF_RSCR_RCPI_PARAM;
        let set = 0 | (0x3 << 24); // RCPI_MODE=0, RCPI_PARAM=3 at bits[25:24]
        self.reg_rmw(
            mt_wtbloff(band, MT_WTBLOFF_RSCR_OFS),
            mask,
            set,
        );

        // MT_TXD5_TX_STATUS_HOST (MPDU format) has higher priority than
        // MT_AGG_ACR_PPDU_TXS2H (PPDU format) even though ACR bit is set.
        // mt76_set(dev, MT_AGG_ACR4(band), MT_AGG_ACR_PPDU_TXS2H);
        self.reg_set(
            mt_wf_agg(band, MT_AGG_ACR4_OFS),
            MT_AGG_ACR_PPDU_TXS2H,
        );
    }

    /// Program 12 basic rates (4 CCK + 8 OFDM) into firmware rate table.
    /// Source: init.c:578-591 mt7996_mac_init_basic_rates()
    ///
    /// Rate hw_values from mac80211.c mt76_rates[]:
    ///   CCK:  0x0000, 0x0001, 0x0002, 0x0003
    ///   OFDM: 0x010b, 0x010f, 0x010a, 0x010e, 0x0109, 0x010d, 0x0108, 0x010c
    fn mac_init_basic_rates(&self, ring: &mut TxRing, seq: &mut u8) -> Result<(), i32> {
        // mt76_rates[] hw_value: (phy_type << 8) | rate_idx
        // MT_PHY_TYPE_CCK = 0, MT_PHY_TYPE_OFDM = 1
        const HW_VALUES: [u16; 12] = [
            0x0000, 0x0001, 0x0002, 0x0003, // CCK idx 0-3
            0x010b, 0x010f, 0x010a, 0x010e, // OFDM idx 11,15,10,14
            0x0109, 0x010d, 0x0108, 0x010c, // OFDM idx 9,13,8,12
        ];

        for (i, &hw_val) in HW_VALUES.iter().enumerate() {
            // odd index for driver, even index for firmware
            // u16 idx = MT7996_BASIC_RATES_TBL + 2 * i;
            let idx = (MT7996_BASIC_RATES_TBL as u16) + 2 * (i as u16);

            // rate = FIELD_PREP(MT_TX_RATE_MODE, hw_val >> 8) |
            //        FIELD_PREP(MT_TX_RATE_IDX, hw_val & 0xFF);
            // MT_TX_RATE_MODE = GENMASK(9,6), MT_TX_RATE_IDX = GENMASK(5,0)
            let mode = (hw_val >> 8) as u16;
            let rate_bits = (hw_val & 0xFF) as u16;
            let rate = (mode << 6) | (rate_bits & 0x3F);

            self.mcu_set_fixed_rate_table(ring, idx as u8, rate, false, 0, *seq)?;
            *seq = seq.wrapping_add(1);
        }
        Ok(())
    }

    /// Full MAC initialization — entry point.
    /// Source: init.c:593-638 mt7996_mac_init()
    ///
    /// 1. Clear MDP_DCR2.RX_TRANS_SHORT
    /// 2. Clear WTBL admin count for all entries
    /// 3. RRO module init (WM-only → wm_ring)
    /// 4. WA HIF TXD version (WA → wa_ring)
    /// 5. Per-band register init (bands 0, 1, 2)
    /// 6. Basic rate programming (WM-only → wm_ring)
    pub fn mac_init(&self, wm_ring: &mut TxRing, wa_ring: &mut TxRing, seq: &mut u8) -> Result<(), i32> {
        // mt76_clear(dev, MT_MDP_DCR2, MT_MDP_DCR2_RX_TRANS_SHORT);
        self.reg_clear(MT_MDP_DCR2, MT_MDP_DCR2_RX_TRANS_SHORT);

        // WTBL size: (wtbl_size_group << 8) + BMC_SIZE
        // MT7996 defaults: group=4, BMC=64 → 1088 entries
        // Hardcoded until EEPROM response parsing is implemented.
        let wtbl_size = (MT7996_WTBL_SIZE_GROUP << 8) + MT7996_WTBL_BMC_SIZE; // 1088

        // for (i = 0; i < mt7996_wtbl_size(dev); i++)
        //     mt7996_mac_wtbl_update(dev, i, MT_WTBL_UPDATE_ADM_COUNT_CLEAR);
        uinfo!("mac", "wtbl_clear"; count = wtbl_size);
        for i in 0..wtbl_size {
            self.mac_wtbl_update(i, MT_WTBL_UPDATE_ADM_COUNT_CLEAR);
        }

        // RRO module init — init.c:609-626
        // MCU_WM_UNI_CMD(RRO) — WM-only, uses wm_ring
        // BPI-R4 MT7996 has HIF2, is_mt7996=true.
        // has_hwrro is effectively false (WED not yet initialized).
        // if (dev->hif2)
        //     mt7996_mcu_set_rro(dev, UNI_RRO_SET_PLATFORM_TYPE, 2);  // is_mt7996
        self.mcu_set_rro(wm_ring, mcu::UNI_RRO_SET_PLATFORM_TYPE, 2, *seq)?;
        *seq = seq.wrapping_add(1);

        // No HW RRO path:
        // mt7996_mcu_set_rro(dev, UNI_RRO_SET_BYPASS_MODE, 3);
        self.mcu_set_rro(wm_ring, mcu::UNI_RRO_SET_BYPASS_MODE, 3, *seq)?;
        *seq = seq.wrapping_add(1);

        // mt7996_mcu_set_rro(dev, UNI_RRO_SET_TXFREE_PATH, 1);
        self.mcu_set_rro(wm_ring, mcu::UNI_RRO_SET_TXFREE_PATH, 1, *seq)?;
        *seq = seq.wrapping_add(1);

        // WA HIF TXD version — init.c:628-630
        // MCU_WA_PARAM_CMD(SET) — WA command, uses wa_ring
        // mt7996_mcu_wa_cmd(dev, MCU_WA_PARAM_CMD(SET),
        //                   MCU_WA_PARAM_HW_PATH_HIF_VER, HIF_TXD_V2_1, 0);
        const HIF_TXD_V2_1: u32 = 0x21;
        self.mcu_wa_cmd(wa_ring, mcu::MCU_WA_PARAM_HW_PATH_HIF_VER, HIF_TXD_V2_1, 0, *seq)?;
        *seq = seq.wrapping_add(1);

        // Per-band init — init.c:632-633
        // for (i = MT_BAND0; i <= MT_BAND2; i++)
        //     mt7996_mac_init_band(dev, i);
        for band in 0..3 {
            self.mac_init_band(band);
        }

        // Basic rates — init.c:635
        // MCU_WM_UNI_CMD(FIXED_RATE_TABLE) — WM-only, uses wm_ring
        // mt7996_mac_init_basic_rates(dev);
        self.mac_init_basic_rates(wm_ring, seq)?;

        uinfo!("mac", "mac_init_done");
        Ok(())
    }
}
