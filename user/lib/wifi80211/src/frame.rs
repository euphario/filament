//! 802.11 frame building and parsing — driver-agnostic, no TXD
//!
//! All frame builders write raw 802.11 frames (MAC header + body).
//! The hardware driver prepends its own TXD before TX.
//!
//! Source: IEEE 802.11-2020, Linux net/mac80211/

use crate::types::BssConfig;

// ============================================================================
// IE builders (shared between beacon, probe response, assoc response)
// ============================================================================

/// Write SSID IE. Returns bytes written.
fn write_ssid_ie(buf: &mut [u8], ssid: &[u8], ssid_len: u8) -> usize {
    let len = ssid_len as usize;
    buf[0] = 0x00; // Element ID: SSID
    buf[1] = ssid_len;
    buf[2..2 + len].copy_from_slice(&ssid[..len]);
    2 + len
}

/// Write Supported Rates IE (802.11b/g, first 8 rates). Returns bytes written.
/// 802.11g basic rate set: 6, 12, 24 Mbps (marked with bit 7).
/// 802.11b basic rates: 1, 2, 5.5, 11 Mbps.
/// Source: IEEE 802.11-2020 Table 15-6, hostapd default config
fn write_rates_ie(buf: &mut [u8]) -> usize {
    buf[0] = 0x01; // Element ID: Supported Rates
    buf[1] = 8;    // length (max 8 in this IE)
    buf[2] = 0x82; // 1 Mbps (basic)
    buf[3] = 0x84; // 2 Mbps (basic)
    buf[4] = 0x8b; // 5.5 Mbps (basic)
    buf[5] = 0x96; // 11 Mbps (basic)
    buf[6] = 0x8c; // 6 Mbps (basic) — was 0x0c, must be basic for 802.11g
    buf[7] = 0x12; // 9 Mbps
    buf[8] = 0x98; // 12 Mbps (basic) — was 0x18
    buf[9] = 0x24; // 18 Mbps
    10
}

/// Write Extended Supported Rates IE (802.11g rates beyond first 8). Returns bytes written.
/// Source: IEEE 802.11-2020 §9.4.2.13
fn write_ext_rates_ie(buf: &mut [u8]) -> usize {
    buf[0] = 50;   // Element ID: Extended Supported Rates
    buf[1] = 4;    // length
    buf[2] = 0xb0; // 24 Mbps (basic) — 0x30 | 0x80
    buf[3] = 0x48; // 36 Mbps
    buf[4] = 0x60; // 48 Mbps
    buf[5] = 0x6c; // 54 Mbps
    6
}

/// Write Country IE (ID 7). Returns bytes written.
/// Source: IEEE 802.11-2020 §9.4.2.9
/// macOS refuses to connect to APs without a Country IE.
fn write_country_ie(buf: &mut [u8], channel: u8) -> usize {
    buf[0] = 7;    // Element ID: Country
    buf[1] = 6;    // length: country_string(3) + one subband_triplet(3)
    // Country string: "US" + space padding (environment: any)
    buf[2] = b'U';
    buf[3] = b'S';
    buf[4] = b' ';
    // Subband triplet: channels 1-11, max 30 dBm
    buf[5] = 1;    // first_channel
    buf[6] = 11;   // num_channels
    buf[7] = 30;   // max_tx_power_dBm
    8
}

/// Write ERP Information IE. Returns bytes written.
/// Source: IEEE 802.11-2020 §9.4.2.13
/// Byte: bit0=NonERP_Present, bit1=Use_Protection, bit2=Barker_Preamble_Mode
/// All zeros = no protection needed (pure 802.11g/n environment)
fn write_erp_ie(buf: &mut [u8]) -> usize {
    buf[0] = 42;   // Element ID: ERP Information
    buf[1] = 1;    // length
    buf[2] = 0x00; // No protection needed
    3
}

/// Write HT Capabilities IE (ID 45). Returns bytes written (28 = 2 + 26).
/// Source: IEEE 802.11-2020 §9.4.2.55, Linux ieee80211_ie_build_ht_cap()
fn write_ht_cap_ie(buf: &mut [u8]) -> usize {
    buf[0] = 45;   // Element ID: HT Capabilities
    buf[1] = 26;   // length

    // HT Capability Info (2 bytes) — IEEE 802.11-2020 §9.4.2.55.2
    //   bit 1: HT 40MHz supported = 0 (20MHz only for now)
    //   bit 5: Short GI for 20MHz = 1
    //   bit 6: Short GI for 40MHz = 0
    //   bit 10-11: SM Power Save = 0b11 (disabled)
    //   bit 12: HT-Delayed Block Ack = 0
    //   bit 13: Max A-MSDU Length = 0 (3839 bytes)
    // = 0x0020 | (0x3 << 10) = 0x0C20
    let ht_cap: u16 = 0x0020 | (0x3 << 10);
    buf[2..4].copy_from_slice(&ht_cap.to_le_bytes());

    // A-MPDU Parameters (1 byte) — §9.4.2.55.3
    //   bits 1:0 = Max A-MPDU Length Exponent = 3 (65535 bytes)
    //   bits 4:2 = Min MPDU Start Spacing = 5 (4μs)
    buf[4] = 0x03 | (5 << 2);

    // Supported MCS Set (16 bytes) — §9.4.2.55.4
    // 2 spatial streams: MCS 0-15 supported
    buf[5] = 0xFF;  // MCS 0-7 (1 stream)
    buf[6] = 0xFF;  // MCS 8-15 (2 streams)
    // bytes 7-20: zeros (MCS 16+ not supported, etc.)

    // HT Extended Capabilities (2 bytes) — zeros
    // TX Beamforming (4 bytes) — zeros
    // ASEL Capabilities (1 byte) — zeros

    28
}

/// Write HT Operation IE (ID 61). Returns bytes written (24 = 2 + 22).
/// Source: IEEE 802.11-2020 §9.4.2.56, Linux ieee80211_ie_build_ht_oper()
fn write_ht_oper_ie(buf: &mut [u8], channel: u8) -> usize {
    buf[0] = 61;   // Element ID: HT Operation
    buf[1] = 22;   // length

    // Primary Channel
    buf[2] = channel;

    // HT Operation Information (5 bytes)
    //   byte 0 bits 1:0 = secondary channel offset = 0 (no secondary, 20MHz)
    //   byte 0 bit 2 = STA channel width = 0 (20MHz)
    //   byte 0 bit 3 = RIFS mode = 0
    buf[3] = 0x00;
    //   byte 1 bits 1:0 = HT protection = 0 (no protection)
    //   byte 1 bit 2 = non-greenfield present = 0
    //   byte 1 bit 4 = OBSS non-HT present = 0
    buf[4] = 0x00;
    // bytes 2-4: zeros

    // Basic MCS Set (16 bytes) at offset 7..23
    // Basic MCS = MCS 0-7 (1 stream minimum)
    buf[9] = 0xFF; // offset 2+7 = 9: basic MCS 0-7

    24
}

/// Write WMM/WME Parameter Element (vendor-specific IE 221). Returns bytes written (26 = 2 + 24).
/// Source: Wi-Fi Alliance WMM Specification v1.2, Linux ieee80211_ie_build_wmm()
fn write_wmm_ie(buf: &mut [u8]) -> usize {
    buf[0] = 221;  // Element ID: Vendor Specific
    buf[1] = 24;   // length

    // OUI: Microsoft 00:50:f2, type 2 (WMM), subtype 1 (param element)
    buf[2] = 0x00;
    buf[3] = 0x50;
    buf[4] = 0xf2;
    buf[5] = 0x02; // WMM type
    buf[6] = 0x01; // WMM Parameter Element subtype
    buf[7] = 0x01; // Version 1

    // QoS Info (1 byte): EDCA Parameter Set Update Count=0, U-APSD=0
    buf[8] = 0x00;

    // Reserved (1 byte)
    buf[9] = 0x00;

    // AC_BE (Best Effort) — 4 bytes
    //   ACI/AIFSN: ACI=0(BE), ACM=0, AIFSN=3
    buf[10] = 0x03;
    //   ECWmin/ECWmax: ECWmin=4(CWmin=15), ECWmax=10(CWmax=1023)
    buf[11] = 0xa4;
    //   TXOP Limit: 0
    buf[12] = 0x00;
    buf[13] = 0x00;

    // AC_BK (Background) — 4 bytes
    //   ACI/AIFSN: ACI=1(BK), ACM=0, AIFSN=7
    buf[14] = 0x27;
    //   ECWmin/ECWmax: ECWmin=4, ECWmax=10
    buf[15] = 0xa4;
    //   TXOP Limit: 0
    buf[16] = 0x00;
    buf[17] = 0x00;

    // AC_VI (Video) — 4 bytes
    //   ACI/AIFSN: ACI=2(VI), ACM=0, AIFSN=2
    buf[18] = 0x42;
    //   ECWmin/ECWmax: ECWmin=3(CWmin=7), ECWmax=4(CWmax=15)
    buf[19] = 0x43;
    //   TXOP Limit: 94 (3.008ms)
    buf[20] = 94;
    buf[21] = 0x00;

    // AC_VO (Voice) — 4 bytes
    //   ACI/AIFSN: ACI=3(VO), ACM=0, AIFSN=2
    buf[22] = 0x62;
    //   ECWmin/ECWmax: ECWmin=2(CWmin=3), ECWmax=3(CWmax=7)
    buf[23] = 0x32;
    //   TXOP Limit: 47 (1.504ms)
    buf[24] = 47;
    buf[25] = 0x00;

    26
}

/// Write DS Parameter Set IE. Returns bytes written.
fn write_ds_ie(buf: &mut [u8], channel: u8) -> usize {
    buf[0] = 0x03; // Element ID: DS Parameter Set
    buf[1] = 1;
    buf[2] = channel;
    3
}

/// Write TIM IE (minimal). Returns bytes written.
fn write_tim_ie(buf: &mut [u8]) -> usize {
    buf[0] = 0x05; // Element ID: TIM
    buf[1] = 4;    // length
    buf[2] = 0;    // DTIM count
    buf[3] = 1;    // DTIM period
    buf[4] = 0;    // bitmap control
    buf[5] = 0;    // partial virtual bitmap
    6
}

// ============================================================================
// Beacon
// ============================================================================

/// Build a beacon frame (raw 802.11, no TXD).
/// Returns bytes written into `buf`.
///
/// Layout: MAC header (24) + timestamp (8) + interval (2) + capability (2) + IEs
pub fn build_beacon(buf: &mut [u8], bss: &BssConfig, seq: u16) -> usize {
    // SSID(2+n) + Rates(10) + DS(3) + TIM(6) + Country(8) + ERP(3) + HT_Cap(28) +
    // ExtRates(6) + HT_Oper(24) + WMM(26)
    let ie_len = (2 + bss.ssid_len as usize) + 10 + 3 + 6 + 8 + 3 + 28 + 6 + 24 + 26;
    let total = 24 + 8 + 2 + 2 + ie_len;
    if buf.len() < total {
        return 0;
    }
    for b in buf[..total].iter_mut() { *b = 0; }

    // MAC Header (24 bytes)
    // FC: type=0 mgmt, subtype=8 beacon → 0x0080
    buf[0..2].copy_from_slice(&0x0080u16.to_le_bytes());
    // duration = 0
    // addr1 (DA): broadcast
    buf[4..10].copy_from_slice(&[0xFF; 6]);
    // addr2 (SA): our MAC
    buf[10..16].copy_from_slice(&bss.bssid);
    // addr3 (BSSID): our MAC
    buf[16..22].copy_from_slice(&bss.bssid);
    // seq_ctrl
    buf[22..24].copy_from_slice(&(seq << 4).to_le_bytes());

    // Fixed fields
    let body = 24;
    // timestamp: u64 = 0 (hardware fills)
    // beacon_interval: 100 TU
    buf[body + 8..body + 10].copy_from_slice(&100u16.to_le_bytes());
    // capability: ESS | Short Preamble | Short Slot Time = 0x0421
    buf[body + 10..body + 12].copy_from_slice(&0x0421u16.to_le_bytes());

    // IEs — order per IEEE 802.11-2020 §9.3.3.3 Table 9-27
    let mut p = body + 12;
    p += write_ssid_ie(&mut buf[p..], &bss.ssid, bss.ssid_len);
    p += write_rates_ie(&mut buf[p..]);
    p += write_ds_ie(&mut buf[p..], bss.channel);
    p += write_tim_ie(&mut buf[p..]);
    p += write_country_ie(&mut buf[p..], bss.channel);  // ID 7
    p += write_erp_ie(&mut buf[p..]);                    // ID 42
    p += write_ht_cap_ie(&mut buf[p..]);                 // ID 45
    p += write_ext_rates_ie(&mut buf[p..]);              // ID 50
    p += write_ht_oper_ie(&mut buf[p..], bss.channel);   // ID 61
    p += write_wmm_ie(&mut buf[p..]);                    // ID 221 (vendor)

    p
}

// ============================================================================
// Probe Response
// ============================================================================

/// Build a probe response frame (raw 802.11, no TXD).
/// Returns bytes written into `buf`.
pub fn build_probe_response(buf: &mut [u8], bss: &BssConfig, dest: &[u8; 6], seq: u16) -> usize {
    // SSID(2+n) + Rates(10) + DS(3) + Country(8) + ERP(3) + HT_Cap(28) +
    // ExtRates(6) + HT_Oper(24) + WMM(26)
    let ie_len = (2 + bss.ssid_len as usize) + 10 + 3 + 8 + 3 + 28 + 6 + 24 + 26;
    let total = 24 + 8 + 2 + 2 + ie_len;
    if buf.len() < total {
        return 0;
    }
    for b in buf[..total].iter_mut() { *b = 0; }

    // FC: type=0 mgmt, subtype=5 probe response → 0x0050
    buf[0..2].copy_from_slice(&0x0050u16.to_le_bytes());
    // addr1 (DA): requesting station
    buf[4..10].copy_from_slice(dest);
    // addr2 (SA): our MAC
    buf[10..16].copy_from_slice(&bss.bssid);
    // addr3 (BSSID)
    buf[16..22].copy_from_slice(&bss.bssid);
    buf[22..24].copy_from_slice(&(seq << 4).to_le_bytes());

    let body = 24;
    buf[body + 8..body + 10].copy_from_slice(&100u16.to_le_bytes());
    buf[body + 10..body + 12].copy_from_slice(&0x0421u16.to_le_bytes());

    // IEs — probe response omits TIM per spec
    let mut p = body + 12;
    p += write_ssid_ie(&mut buf[p..], &bss.ssid, bss.ssid_len);
    p += write_rates_ie(&mut buf[p..]);
    p += write_ds_ie(&mut buf[p..], bss.channel);
    p += write_country_ie(&mut buf[p..], bss.channel);
    p += write_erp_ie(&mut buf[p..]);
    p += write_ht_cap_ie(&mut buf[p..]);
    p += write_ext_rates_ie(&mut buf[p..]);
    p += write_ht_oper_ie(&mut buf[p..], bss.channel);
    p += write_wmm_ie(&mut buf[p..]);

    p
}

// ============================================================================
// Authentication Response (Open System)
// ============================================================================

/// Build an authentication response frame (Open System, status=success).
/// Returns bytes written into `buf`.
///
/// Auth body: algorithm(2) + seq_num(2) + status(2) = 6 bytes
/// Source: IEEE 802.11-2020 §12.3.3.2
pub fn build_auth_response(buf: &mut [u8], bssid: &[u8; 6], dest: &[u8; 6], seq: u16) -> usize {
    const TOTAL: usize = 24 + 6; // MAC header + auth body
    if buf.len() < TOTAL {
        return 0;
    }
    for b in buf[..TOTAL].iter_mut() { *b = 0; }

    // FC: type=0 mgmt, subtype=0xB auth → 0x00B0
    buf[0..2].copy_from_slice(&0x00B0u16.to_le_bytes());
    buf[4..10].copy_from_slice(dest);
    buf[10..16].copy_from_slice(bssid);
    buf[16..22].copy_from_slice(bssid);
    buf[22..24].copy_from_slice(&(seq << 4).to_le_bytes());

    // Auth body
    let body = 24;
    // Algorithm: Open System = 0
    buf[body..body + 2].copy_from_slice(&0u16.to_le_bytes());
    // Auth transaction sequence number: 2 (response)
    buf[body + 2..body + 4].copy_from_slice(&2u16.to_le_bytes());
    // Status: Success = 0
    buf[body + 4..body + 6].copy_from_slice(&0u16.to_le_bytes());

    TOTAL
}

// ============================================================================
// Association Response
// ============================================================================

/// Build an association response frame (status=success).
/// Returns bytes written into `buf`.
///
/// Assoc resp body: capability(2) + status(2) + AID(2) + Supported Rates IE
/// Source: IEEE 802.11-2020 §9.3.3.7
pub fn build_assoc_response(buf: &mut [u8], bss: &BssConfig, dest: &[u8; 6], aid: u16, seq: u16) -> usize {
    const FIXED: usize = 24 + 2 + 2 + 2; // header + cap + status + AID
    let total = FIXED + 10 + 3 + 28 + 6 + 24 + 26; // Rates+ERP+HT_Cap+ExtRates+HT_Oper+WMM
    if buf.len() < total {
        return 0;
    }
    for b in buf[..total].iter_mut() { *b = 0; }

    // FC: type=0 mgmt, subtype=1 assoc response → 0x0010
    buf[0..2].copy_from_slice(&0x0010u16.to_le_bytes());
    buf[4..10].copy_from_slice(dest);
    buf[10..16].copy_from_slice(&bss.bssid);
    buf[16..22].copy_from_slice(&bss.bssid);
    buf[22..24].copy_from_slice(&(seq << 4).to_le_bytes());

    let body = 24;
    // Capability: ESS | Short Preamble | Short Slot Time = 0x0421
    buf[body..body + 2].copy_from_slice(&0x0421u16.to_le_bytes());
    // Status: Success = 0
    buf[body + 2..body + 4].copy_from_slice(&0u16.to_le_bytes());
    // AID: bits [15:14] must be 0b11 per spec (§9.4.1.8)
    let aid_field = aid | 0xC000;
    buf[body + 4..body + 6].copy_from_slice(&aid_field.to_le_bytes());

    // IEs
    let mut p = body + 6;
    p += write_rates_ie(&mut buf[p..]);
    p += write_erp_ie(&mut buf[p..]);
    p += write_ht_cap_ie(&mut buf[p..]);
    p += write_ext_rates_ie(&mut buf[p..]);
    p += write_ht_oper_ie(&mut buf[p..], bss.channel);
    p += write_wmm_ie(&mut buf[p..]);

    p
}

// ============================================================================
// Frame Parsing
// ============================================================================

/// Extract (addr1/DA, addr2/SA, addr3/BSSID) from an 802.11 frame.
/// Requires at least 22 bytes (FC + Duration + 3×addr).
pub fn parse_frame_addrs(frame: &[u8]) -> Option<([u8; 6], [u8; 6], [u8; 6])> {
    if frame.len() < 22 {
        return None;
    }
    let mut da = [0u8; 6];
    let mut sa = [0u8; 6];
    let mut bssid = [0u8; 6];
    da.copy_from_slice(&frame[4..10]);
    sa.copy_from_slice(&frame[10..16]);
    bssid.copy_from_slice(&frame[16..22]);
    Some((da, sa, bssid))
}
