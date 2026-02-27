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
// Deauthentication
// ============================================================================

/// Build a deauthentication frame.
/// Reason codes: 2 = PREV_AUTH_NOT_VALID, 6 = CLASS2_FRAME_FROM_NONAUTH_STA,
///               7 = CLASS3_FRAME_FROM_NONASSOC_STA
/// Source: IEEE 802.11-2020 §9.3.3.1, Table 9-49
pub fn build_deauth(buf: &mut [u8], bssid: &[u8; 6], dest: &[u8; 6], reason: u16, seq: u16) -> usize {
    const TOTAL: usize = 24 + 2; // MAC header + reason code
    if buf.len() < TOTAL {
        return 0;
    }
    for b in buf[..TOTAL].iter_mut() { *b = 0; }

    // FC: type=0 mgmt, subtype=0xC deauth → 0x00C0
    buf[0..2].copy_from_slice(&0x00C0u16.to_le_bytes());
    buf[4..10].copy_from_slice(dest);
    buf[10..16].copy_from_slice(bssid);
    buf[16..22].copy_from_slice(bssid);
    buf[22..24].copy_from_slice(&(seq << 4).to_le_bytes());

    // Reason code
    buf[24..26].copy_from_slice(&reason.to_le_bytes());

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
    build_assoc_or_reassoc_response(buf, bss, dest, aid, seq, false)
}

/// Build a Reassociation Response frame.
/// Identical to Association Response but with FC subtype=3 (0x0030).
/// Source: IEEE 802.11-2020 §9.3.3.8
pub fn build_reassoc_response(buf: &mut [u8], bss: &BssConfig, dest: &[u8; 6], aid: u16, seq: u16) -> usize {
    build_assoc_or_reassoc_response(buf, bss, dest, aid, seq, true)
}

fn build_assoc_or_reassoc_response(buf: &mut [u8], bss: &BssConfig, dest: &[u8; 6], aid: u16, seq: u16, reassoc: bool) -> usize {
    const FIXED: usize = 24 + 2 + 2 + 2; // header + cap + status + AID
    let total = FIXED + 10 + 3 + 28 + 6 + 24 + 26; // Rates+ERP+HT_Cap+ExtRates+HT_Oper+WMM
    if buf.len() < total {
        return 0;
    }
    for b in buf[..total].iter_mut() { *b = 0; }

    // FC: type=0 mgmt, subtype=1 assoc response (0x0010) or subtype=3 reassoc response (0x0030)
    let fc: u16 = if reassoc { 0x0030 } else { 0x0010 };
    buf[0..2].copy_from_slice(&fc.to_le_bytes());
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

// ============================================================================
// ADDBA Response (Block Ack)
// ============================================================================

/// Build an ADDBA Response frame.
/// Source: IEEE 802.11-2020 §9.6.5.2, FreeBSD ieee80211_send_addba_resp(),
///         OpenBSD ieee80211_recv_addba_req()
///
/// Layout: MAC header (24) + Category(1) + Action(1) + Token(1) +
///         Status(2) + BA Params(2) + Timeout(2) = 33 bytes
pub fn build_addba_response(
    buf: &mut [u8],
    bssid: &[u8; 6],
    dest: &[u8; 6],
    token: u8,
    tid: u8,
    win_size: u16,
    timeout: u16,
    seq: u16,
) -> usize {
    const TOTAL: usize = 24 + 9; // MAC header + action body
    if buf.len() < TOTAL {
        return 0;
    }
    for b in buf[..TOTAL].iter_mut() { *b = 0; }

    // FC: type=0 mgmt, subtype=0xD action → 0x00D0
    buf[0..2].copy_from_slice(&0x00D0u16.to_le_bytes());
    buf[4..10].copy_from_slice(dest);
    buf[10..16].copy_from_slice(bssid);
    buf[16..22].copy_from_slice(bssid);
    buf[22..24].copy_from_slice(&(seq << 4).to_le_bytes());

    let body = 24;
    // Category: Block Ack = 3 — IEEE 802.11-2020 Table 9-51
    buf[body] = 3;
    // Action: ADDBA Response = 1
    buf[body + 1] = 1;
    // Dialog Token
    buf[body + 2] = token;
    // Status Code: Success = 0
    buf[body + 3..body + 5].copy_from_slice(&0u16.to_le_bytes());
    // BA Parameters: A-MSDU(0) | BA policy(1=immediate) | TID | buffer size
    // Bits [0]: A-MSDU supported = 0
    // Bits [1]: BA policy = 1 (immediate)
    // Bits [5:2]: TID
    // Bits [15:6]: buffer size
    let ba_params: u16 = 0x0002 | ((tid as u16 & 0xF) << 2) | ((win_size & 0x3FF) << 6);
    buf[body + 5..body + 7].copy_from_slice(&ba_params.to_le_bytes());
    // BA Timeout Value
    buf[body + 7..body + 9].copy_from_slice(&timeout.to_le_bytes());

    TOTAL
}

/// Build a DELBA frame (Delete Block Ack).
/// Source: IEEE 802.11-2020 §9.6.5.4
///
/// Layout: MAC header (24) + Category(1) + Action(1) + DELBA params(2) +
///         Reason(2) = 30 bytes
pub fn build_delba(
    buf: &mut [u8],
    bssid: &[u8; 6],
    dest: &[u8; 6],
    tid: u8,
    initiator: bool,
    reason: u16,
    seq: u16,
) -> usize {
    const TOTAL: usize = 24 + 6;
    if buf.len() < TOTAL {
        return 0;
    }
    for b in buf[..TOTAL].iter_mut() { *b = 0; }

    // FC: type=0 mgmt, subtype=0xD action → 0x00D0
    buf[0..2].copy_from_slice(&0x00D0u16.to_le_bytes());
    buf[4..10].copy_from_slice(dest);
    buf[10..16].copy_from_slice(bssid);
    buf[16..22].copy_from_slice(bssid);
    buf[22..24].copy_from_slice(&(seq << 4).to_le_bytes());

    let body = 24;
    // Category: Block Ack = 3
    buf[body] = 3;
    // Action: DELBA = 2
    buf[body + 1] = 2;
    // DELBA Parameters: bits [11]: initiator, bits [15:12]: TID
    let delba_params: u16 = ((initiator as u16) << 11) | ((tid as u16 & 0xF) << 12);
    buf[body + 2..body + 4].copy_from_slice(&delba_params.to_le_bytes());
    // Reason Code
    buf[body + 4..body + 6].copy_from_slice(&reason.to_le_bytes());

    TOTAL
}

// ============================================================================
// Association Request IE Parsing
// ============================================================================

/// Parsed IEs from an association/reassociation request body.
pub struct AssocIes {
    /// Capability info from fixed fields
    pub cap_info: u16,
    /// HT Capabilities info word from IE 45 (0 if not present)
    pub ht_cap: u16,
    /// A-MPDU params byte from IE 45 (0 if not present)
    pub ht_ampdu_param: u8,
    /// Whether HT Capabilities IE was present
    pub has_ht: bool,
    /// SSID from IE 0 (length, then bytes). ssid_len=0 if not found.
    pub ssid: [u8; 32],
    pub ssid_len: u8,
}

/// Parse IEs from an assoc/reassoc request frame body (after MAC header).
/// For AssocReq: body starts at cap_info(2) + listen_interval(2) + IEs
/// For ReassocReq: body starts at cap_info(2) + listen_interval(2) + current_ap(6) + IEs
///
/// Source: IEEE 802.11-2020 §9.3.3.6 (Assoc), §9.3.3.8 (Reassoc)
/// FreeBSD hostap_recv_mgmt() step 7, OpenBSD ieee80211_recv_assoc_req()
pub fn parse_assoc_ies(body: &[u8], is_reassoc: bool) -> Option<AssocIes> {
    let fixed_len = if is_reassoc { 10 } else { 4 }; // cap(2)+listen(2) [+current_ap(6)]
    if body.len() < fixed_len {
        return None;
    }

    let cap_info = u16::from_le_bytes([body[0], body[1]]);

    let mut result = AssocIes {
        cap_info,
        ht_cap: 0,
        ht_ampdu_param: 0,
        has_ht: false,
        ssid: [0; 32],
        ssid_len: 0,
    };

    // Walk IEs
    let mut pos = fixed_len;
    while pos + 2 <= body.len() {
        let ie_id = body[pos];
        let ie_len = body[pos + 1] as usize;
        if pos + 2 + ie_len > body.len() {
            break; // truncated IE
        }
        let ie_data = &body[pos + 2..pos + 2 + ie_len];

        match ie_id {
            // SSID (IE 0)
            0 => {
                let len = ie_len.min(32);
                result.ssid[..len].copy_from_slice(&ie_data[..len]);
                result.ssid_len = len as u8;
            }
            // HT Capabilities (IE 45) — IEEE 802.11-2020 §9.4.2.55
            45 => {
                if ie_len >= 2 {
                    result.ht_cap = u16::from_le_bytes([ie_data[0], ie_data[1]]);
                    result.has_ht = true;
                }
                if ie_len >= 3 {
                    result.ht_ampdu_param = ie_data[2];
                }
            }
            // RSN (IE 48) — stub, no validation yet
            // TODO: parse RSN IE for security negotiation
            48 => {}
            _ => {}
        }

        pos += 2 + ie_len;
    }

    Some(result)
}
