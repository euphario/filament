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

/// Compute byte offset from base RXD (32B) through RXD groups.
///
/// Group order: G4(16), G1(16), G2(16), G3(16), G5(96).
/// Pass `include_g3 = false` to stop before Group 3 (for RSSI extraction),
/// or `true` to include all groups (for frame data offset).
pub fn group_offset(rxd1: u32, include_g3: bool) -> usize {
    let mut ofs: usize = 32;
    if rxd1 & MT_RXD1_NORMAL_GROUP_4 != 0 { ofs += 16; }
    if rxd1 & MT_RXD1_NORMAL_GROUP_1 != 0 { ofs += 16; }
    if rxd1 & MT_RXD1_NORMAL_GROUP_2 != 0 { ofs += 16; }
    if include_g3 {
        if rxd1 & MT_RXD1_NORMAL_GROUP_3 != 0 { ofs += 16; }
        if rxd1 & MT_RXD1_NORMAL_GROUP_5 != 0 { ofs += 96; }
    }
    ofs
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

    // Frame data offset: skip all groups + padding
    let mut frame_ofs = group_offset(rxd1, true);
    let remove_pad = ((rxd2 >> 13) & 0x7) as usize;
    frame_ofs += 2 * remove_pad;

    // RSSI from Group 3 DW3 byte 0 = RCPI chain 0
    // Source: Linux mt76/mac.c mt76_connac3_mac_fill_rx()
    let rssi = if rxd1 & MT_RXD1_NORMAL_GROUP_3 != 0 {
        let g3_ofs = group_offset(rxd1, false);
        if g3_ofs + 16 <= buf.len() {
            let rcpi = buf[g3_ofs + 12];
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

// 802.11 frame header sizes
const DOT11_HDR_LEN: usize = 24;
const DOT11_QOS_HDR_LEN: usize = 26;

// LLC/SNAP + EAPOL EtherType (AA:AA:03:00:00:00:88:8E)
const LLC_SNAP_EAPOL: [u8; 8] = [0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8E];

/// Check if a non-header-translated data frame contains EAPOL.
///
/// Looks for LLC/SNAP header + EtherType 0x888E after the 802.11 header.
/// Returns (eapol_body_offset, eapol_body_len) if EAPOL detected.
pub fn detect_eapol(buf: &[u8], frame_ofs: usize, fc0: u8, byte_cnt: usize) -> Option<(usize, usize)> {
    let subtype = (fc0 >> 4) & 0xF;
    let is_qos = (subtype & 0x8) != 0;
    let hdr_len = if is_qos { DOT11_QOS_HDR_LEN } else { DOT11_HDR_LEN };
    let llc_ofs = frame_ofs + hdr_len;

    if llc_ofs + LLC_SNAP_EAPOL.len() > byte_cnt || byte_cnt > buf.len() {
        return None;
    }

    if buf[llc_ofs..llc_ofs + 8] == LLC_SNAP_EAPOL {
        let eapol_start = llc_ofs + LLC_SNAP_EAPOL.len();
        let eapol_len = byte_cnt - eapol_start;
        if eapol_len <= 256 {
            return Some((eapol_start, eapol_len));
        }
    }
    None
}
