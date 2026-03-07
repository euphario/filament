//! RX Event Processing — RXD parsing, frame classification
//!
//! Parses the 16-byte RXD header prepended to every RX buffer by the MT7996
//! WFDMA engine. Classifies frames into management, data, EAPOL, TX free,
//! or other for the main event loop.
//!
//! Reference: Linux mt76/mt7996/mac.c, mt76/dma.c

use crate::regs::*;

// ============================================================================
// Frame classification
// ============================================================================

/// Classified frame type from RXD inspection.
#[derive(Clone, Copy, PartialEq)]
pub enum RxFrameClass {
    /// 802.11 data frame (header-translated or raw)
    Data,
    /// 802.11 management frame (auth, assoc, probe, deauth, etc.)
    Mgmt,
    /// TX free notification from WA (token reclaim)
    TxFree,
    /// Everything else (TXS, TXRXV, firmware events, etc.)
    Other,
}

/// Parsed RXD metadata from first 16 bytes of RX buffer.
pub struct RxdInfo {
    pub class: RxFrameClass,
    pub wlan_idx: u16,
    pub hdr_trans: bool,
    pub fcs_err: bool,
    /// Byte offset from buffer start to 802.11/Ethernet frame header
    pub frame_ofs: usize,
    /// RSSI in dBm (from Group 3 RCPI chain 0). -128 = unknown.
    pub rssi: i8,
    /// RX byte count from RXD0
    pub byte_cnt: u16,
    /// Raw RXD words (for PHY info extraction)
    pub rxd1: u32,
}

/// EAPOL frame extracted from non-header-translated 802.11 data.
pub struct RxEapolFrame {
    pub src_mac: [u8; 6],
    pub body: [u8; 256],
    pub body_len: u16,
}

impl RxEapolFrame {
    pub const EMPTY: Self = Self {
        src_mac: [0; 6],
        body: [0; 256],
        body_len: 0,
    };
}

/// Classify an RX buffer by parsing RXD0-RXD3.
///
/// Returns RxdInfo with frame class, WLAN index, header translation flag,
/// FCS error flag, and byte offset to the actual frame data.
///
/// Source: Linux mt76/mt7996/mac.c mt7996_mac_fill_rx(), dma.c rx_process_seg()
pub fn classify_rxd(buf: &[u8]) -> RxdInfo {
    if buf.len() < 16 {
        return RxdInfo {
            class: RxFrameClass::Other,
            wlan_idx: 0,
            hdr_trans: false,
            fcs_err: false,
            frame_ofs: 0,
            rssi: -128,
            byte_cnt: 0,
            rxd1: 0,
        };
    }

    let rxd0 = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    let rxd1 = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    let rxd2 = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
    let rxd3 = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);

    let byte_cnt = (rxd0 & MT_RXD0_LENGTH) as u16;
    let pkt_type = (rxd0 & MT_RXD0_PKT_TYPE_MASK) >> MT_RXD0_PKT_TYPE_SHIFT;
    let wlan_idx = (rxd1 & MT_RXD1_NORMAL_WLAN_IDX) as u16;
    let hdr_trans = rxd2 & MT_RXD2_NORMAL_HDR_TRANS != 0;
    let ndata = rxd2 & MT_RXD2_NORMAL_NDATA != 0;
    let fcs_err = rxd3 & MT_RXD3_NORMAL_FCS_ERR != 0;

    // Classify by pkt_type
    // Source: mt76_connac3_mac.h PKT_TYPE definitions
    let class = match pkt_type {
        PKT_TYPE_NORMAL if ndata => RxFrameClass::Mgmt,
        PKT_TYPE_NORMAL => RxFrameClass::Data,
        PKT_TYPE_TXRX_NOTIFY => RxFrameClass::TxFree,
        _ => RxFrameClass::Other,
    };

    // Calculate frame offset: skip RXD groups to reach 802.11/Ethernet header
    // Base RXD = 32 bytes (8 DWORDs, though we only read first 4 here)
    // Group order: G4(16), G1(16), G2(16), G3(16), G5(96)
    // Source: Linux mt76/mt7996/mac.c mt7996_mac_fill_rx()
    let mut frame_ofs: usize = 32;
    if rxd1 & MT_RXD1_NORMAL_GROUP_4 != 0 { frame_ofs += 16; }
    if rxd1 & MT_RXD1_NORMAL_GROUP_1 != 0 { frame_ofs += 16; }
    if rxd1 & MT_RXD1_NORMAL_GROUP_2 != 0 { frame_ofs += 16; }
    if rxd1 & MT_RXD1_NORMAL_GROUP_3 != 0 { frame_ofs += 16; }
    if rxd1 & MT_RXD1_NORMAL_GROUP_5 != 0 { frame_ofs += 96; }

    // Header padding removal (firmware pads for alignment)
    let remove_pad = ((rxd2 >> 13) & 0x7) as usize;
    frame_ofs += 2 * remove_pad;

    // RSSI from Group 3 (P-RXV) DW3 byte 0 = RCPI chain 0
    // Source: Linux mt76/mac.c mt76_connac3_mac_fill_rx()
    let rssi = if rxd1 & MT_RXD1_NORMAL_GROUP_3 != 0 {
        let mut g3_ofs: usize = 32;
        if rxd1 & MT_RXD1_NORMAL_GROUP_4 != 0 { g3_ofs += 16; }
        if rxd1 & MT_RXD1_NORMAL_GROUP_1 != 0 { g3_ofs += 16; }
        if rxd1 & MT_RXD1_NORMAL_GROUP_2 != 0 { g3_ofs += 16; }
        if g3_ofs + 16 <= buf.len() {
            let g3_dw3_ofs = g3_ofs + 12;
            let rcpi = buf[g3_dw3_ofs]; // chain 0 RCPI
            (((rcpi as i32) - 220) / 2).clamp(-128, 0) as i8
        } else {
            -128i8
        }
    } else {
        -128i8
    };

    RxdInfo {
        class,
        wlan_idx,
        hdr_trans,
        fcs_err,
        frame_ofs,
        rssi,
        byte_cnt,
        rxd1,
    }
}

/// Reclassify a frame by reading the FC byte (for non-header-translated frames).
///
/// The NDATA bit in RXD2 is unreliable — unicast management frames can arrive
/// with NDATA=0, misclassified as Data. Always check FC type field for
/// non-HDR_TRANS PKT_TYPE_NORMAL frames.
///
/// Returns updated RxFrameClass.
pub fn reclassify_by_fc(fc0: u8) -> RxFrameClass {
    let fc_type = (fc0 >> 2) & 0x3;
    match fc_type {
        0 => RxFrameClass::Mgmt, // Management
        2 => RxFrameClass::Data, // Data
        _ => RxFrameClass::Other,
    }
}

/// Extract management subtype from FC byte 0.
/// Returns bits [7:4] = subtype field.
pub fn mgmt_subtype(fc0: u8) -> u8 {
    (fc0 >> 4) & 0xF
}

/// Check if a non-header-translated data frame contains EAPOL.
///
/// Looks for LLC/SNAP header (AA AA 03 00 00 00) + EtherType 0x888E
/// after the 802.11 data header.
///
/// Returns (eapol_body_offset, eapol_body_len) if EAPOL detected.
pub fn detect_eapol(buf: &[u8], frame_ofs: usize, fc0: u8, byte_cnt: usize) -> Option<(usize, usize)> {
    let subtype = (fc0 >> 4) & 0xF;
    let is_qos = (subtype & 0x8) != 0;
    let hdr_len = if is_qos { 26usize } else { 24usize };
    let llc_ofs = frame_ofs + hdr_len;

    if llc_ofs + 8 > byte_cnt || byte_cnt > buf.len() {
        return None;
    }

    let llc = &buf[llc_ofs..llc_ofs + 8];
    if llc[0] == 0xAA && llc[1] == 0xAA && llc[2] == 0x03
        && llc[3] == 0x00 && llc[4] == 0x00 && llc[5] == 0x00
        && llc[6] == 0x88 && llc[7] == 0x8E
    {
        let eapol_start = llc_ofs + 8;
        let eapol_len = byte_cnt - eapol_start;
        if eapol_len <= 256 {
            return Some((eapol_start, eapol_len));
        }
    }
    None
}
