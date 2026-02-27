//! Common 802.11 types — BSS config, STA table entries, frame classification

pub const MAX_SSID_LEN: usize = 32;
pub const MAX_STAS: usize = 64;

/// BSS configuration for an AP
pub struct BssConfig {
    pub bssid: [u8; 6],
    pub ssid: [u8; MAX_SSID_LEN],
    pub ssid_len: u8,
    pub channel: u8,
}

/// STA lifecycle state machine
#[derive(Clone, Copy, PartialEq)]
pub enum StaState {
    /// Slot is free
    Free,
    /// STA has completed Open System auth (awaiting assoc)
    Authenticated,
    /// STA is fully associated
    Associated,
}

/// RX PHY info extracted from RXD Group 3 (P-RXV).
/// Updated on every received management frame.
#[derive(Clone, Copy)]
pub struct RxPhyInfo {
    /// RSSI in dBm (from RCPI chain 0). -128 = unknown.
    pub rssi: i8,
    /// Per-chain RCPI values (0-255, 0=unused chain). Up to 4 chains.
    pub rcpi: [u8; 4],
    /// RX rate index from P-RXV DW0 bits [6:0] — MCS or legacy rate.
    pub rate: u8,
    /// NSS (number of spatial streams) from P-RXV DW0 bits [9:7]. 0-based.
    pub nss: u8,
    /// Bandwidth: 0=20MHz, 1=40MHz, 2=80MHz, 3=160MHz.
    pub bw: u8,
    /// PHY mode from P-RXV DW0 bits [14:12]: 0=CCK, 1=OFDM, 2=HT, 4=VHT, 8=HE, 9=EHT
    pub mode: u8,
    /// Short GI / HE GI type
    pub gi: u8,
    /// STBC flag
    pub stbc: bool,
}

impl RxPhyInfo {
    pub const UNKNOWN: Self = Self {
        rssi: -128,
        rcpi: [0; 4],
        rate: 0,
        nss: 0,
        bw: 0,
        mode: 0,
        gi: 0,
        stbc: false,
    };
}

/// Per-STA table entry
#[derive(Clone, Copy)]
pub struct StaEntry {
    pub mac: [u8; 6],
    /// 1-based Association ID (0 = unassigned)
    pub aid: u16,
    /// Firmware WCID (WLAN index) for TX routing
    pub wlan_idx: u16,
    pub state: StaState,
    /// Tick when this STA was last seen (auth, assoc, or data frame)
    pub last_seen: u32,
    /// Last observed RSSI in dBm. -128 = unknown.
    pub rssi: i8,
    /// Last RX PHY info
    pub phy: RxPhyInfo,
}

impl StaEntry {
    pub const FREE: Self = Self {
        mac: [0; 6],
        aid: 0,
        wlan_idx: 0,
        state: StaState::Free,
        last_seen: 0,
        rssi: -128,
        phy: RxPhyInfo::UNKNOWN,
    };
}

/// Parsed incoming management frame (driver-extracted from RXD)
pub struct RxMgmtFrame {
    pub subtype: MgmtSubtype,
    /// Source MAC (addr2 / transmitter)
    pub addr2: [u8; 6],
    /// RSSI in dBm (from RCPI). -128 = unknown.
    pub rssi: i8,
    /// Full RX PHY info
    pub phy: RxPhyInfo,
}

/// 802.11 management frame subtypes we handle
#[derive(Clone, Copy, PartialEq)]
pub enum MgmtSubtype {
    ProbeReq,
    Auth,
    AssocReq,
    Deauth,
    Disassoc,
    Other(u8),
}

/// Actions the AP wants the driver to perform after processing an RX frame
pub enum ApAction<'a> {
    /// Transmit a raw 802.11 frame (no TXD — driver prepends its own)
    TxFrame(&'a [u8]),
    /// Tell firmware about a new associated STA
    RegisterSta { mac: [u8; 6], aid: u16 },
    /// Remove STA from firmware
    RemoveSta { aid: u16, wlan_idx: u16 },
}

/// Parse FC byte 0 into MgmtSubtype.
/// FC byte 0 bits [3:2] = type (0=mgmt), bits [7:4] = subtype.
///
/// Source: IEEE 802.11-2020 Table 9-1
pub fn parse_mgmt_subtype(fc0: u8) -> MgmtSubtype {
    let frame_type = (fc0 >> 2) & 0x3;
    if frame_type != 0 {
        return MgmtSubtype::Other(fc0);
    }
    let subtype = (fc0 >> 4) & 0xF;
    match subtype {
        0x0 => MgmtSubtype::AssocReq,   // Association Request
        0x4 => MgmtSubtype::ProbeReq,   // Probe Request
        0xB => MgmtSubtype::Auth,        // Authentication
        0xA => MgmtSubtype::Disassoc,    // Disassociation
        0xC => MgmtSubtype::Deauth,      // Deauthentication
        _ => MgmtSubtype::Other(subtype),
    }
}
