//! Common 802.11 types — BSS config, STA table entries, frame classification

pub const MAX_SSID_LEN: usize = 32;
pub const MAX_STAS: usize = 64;

/// BSS configuration for an AP
pub struct BssConfig {
    pub bssid: [u8; 6],
    pub ssid: [u8; MAX_SSID_LEN],
    pub ssid_len: u8,
    pub channel: u8,
    /// Channel width: 0=20MHz, 1=40MHz
    pub bandwidth: u8,
    /// Secondary channel offset for HT40: -1=below, 0=none, +1=above
    pub secondary_channel_offset: i8,
    /// ERP protection needed (non-ERP STA present)
    pub erp_protection: bool,
    /// HT protection mode (0=none, 1=nonmember, 2=20MHz, 3=non-HT mixed)
    pub ht_protection: u8,
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

/// STA lifecycle events for validated state transitions.
/// Source: FreeBSD ieee80211_node.c, OpenBSD ieee80211_node.c
#[derive(Clone, Copy, PartialEq)]
pub enum StaEvent {
    Auth,
    Assoc,
    Deauth,
    Disassoc,
}

impl StaState {
    /// Validate and execute state transition. Returns Err with 802.11 reason
    /// code if the transition is invalid (caller should send deauth/disassoc).
    /// Source: FreeBSD hostap_recv_mgmt(), OpenBSD ieee80211_recv_auth()
    pub fn transition(self, event: StaEvent) -> Result<StaState, u16> {
        match (self, event) {
            (StaState::Free, StaEvent::Auth) => Ok(StaState::Authenticated),
            (StaState::Authenticated, StaEvent::Auth) => Ok(StaState::Authenticated), // re-auth
            (StaState::Authenticated, StaEvent::Assoc) => Ok(StaState::Associated),
            (StaState::Associated, StaEvent::Auth) => Ok(StaState::Authenticated),    // re-auth from assoc
            (StaState::Associated, StaEvent::Assoc) => Ok(StaState::Associated),      // re-assoc
            (_, StaEvent::Deauth) => Ok(StaState::Free),
            (StaState::Associated, StaEvent::Disassoc) => Ok(StaState::Authenticated),
            // Invalid transitions
            (StaState::Free, StaEvent::Assoc) => Err(6),     // Class 2 from non-auth
            (StaState::Free, StaEvent::Disassoc) => Err(6),
            (StaState::Authenticated, StaEvent::Disassoc) => Err(7), // Class 3 from non-assoc
        }
    }
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

/// Per-TID RX Block Ack session state.
/// Source: FreeBSD ieee80211_ht.c, OpenBSD ieee80211_recv_addba_req()
#[derive(Clone, Copy, PartialEq)]
pub enum BaState {
    /// No BA session for this TID
    Init,
    /// BA session negotiated and active
    Agreed,
}

/// Per-TID RX Block Ack session.
/// Source: Linux mt76/mt7996/mac.c, FreeBSD ieee80211_ampdu_rx_start()
#[derive(Clone, Copy)]
pub struct RxBaSession {
    pub state: BaState,
    pub tid: u8,
    /// Negotiated window size (max 64)
    pub win_size: u16,
    /// Starting sequence number from ADDBA Request
    pub ssn: u16,
    /// Dialog token from request (echoed in response)
    pub token: u8,
}

impl RxBaSession {
    pub const INIT: Self = Self {
        state: BaState::Init,
        tid: 0,
        win_size: 0,
        ssn: 0,
        token: 0,
    };
}

/// STA capability flags (derived from assoc request IEs)
pub const STA_FLAG_HT: u16    = 0x0001;
pub const STA_FLAG_QOS: u16   = 0x0002;
pub const STA_FLAG_SGI20: u16 = 0x0004;
pub const STA_FLAG_ERP: u16   = 0x0008;
pub const STA_FLAG_NON_ERP: u16 = 0x0010;
pub const STA_FLAG_HT40: u16  = 0x0020;
pub const STA_FLAG_SGI40: u16 = 0x0040;

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
    /// Per-TID RX Block Ack sessions (TID 0-7)
    pub rx_ba: [RxBaSession; 8],
    /// Capability info from assoc request fixed fields
    pub cap_info: u16,
    /// HT capability flags from IE 45 (0 if not HT)
    pub ht_cap: u16,
    /// A-MPDU params from IE 45 byte 3
    pub ht_param: u8,
    /// STA capability flags (STA_FLAG_HT, STA_FLAG_QOS, etc.)
    pub flags: u16,
    /// Power save mode. Placeholder, always false for now.
    pub ps_mode: bool,
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
        rx_ba: [RxBaSession::INIT; 8],
        cap_info: 0,
        ht_cap: 0,
        ht_param: 0,
        flags: 0,
        ps_mode: false,
    };
}

/// Maximum size of frame body we copy from DMA buffer for mgmt frame parsing.
/// Must accommodate the largest management frame body we care about.
/// Assoc request can be large with many IEs; 256 bytes covers common cases.
pub const MAX_MGMT_BODY_LEN: usize = 256;

/// Parsed incoming management frame (driver-extracted from RXD).
/// `body` contains the frame payload after the 24-byte MAC header,
/// copied from the DMA buffer before descriptor reset.
pub struct RxMgmtFrame {
    pub subtype: MgmtSubtype,
    /// Source MAC (addr2 / transmitter)
    pub addr2: [u8; 6],
    /// BSSID (addr3) for validation
    pub addr3: [u8; 6],
    /// RSSI in dBm (from RCPI). -128 = unknown.
    pub rssi: i8,
    /// Full RX PHY info
    pub phy: RxPhyInfo,
    /// Frame body after MAC header (24 bytes). Length in `body_len`.
    pub body: [u8; MAX_MGMT_BODY_LEN],
    /// Actual length of valid data in `body`
    pub body_len: u16,
}

/// 802.11 management frame subtypes we handle
#[derive(Clone, Copy, PartialEq)]
pub enum MgmtSubtype {
    ProbeReq,
    Auth,
    AssocReq,
    ReassocReq,
    Deauth,
    Disassoc,
    Action,
    Other(u8),
}

/// Actions the AP wants the driver to perform after processing an RX frame
pub enum ApAction<'a> {
    /// Transmit a raw 802.11 frame (no TXD — driver prepends its own)
    TxFrame(&'a [u8]),
    /// Tell firmware about a new associated STA (with HT capabilities)
    RegisterSta {
        mac: [u8; 6],
        aid: u16,
        /// HT capability info from IE 45 (0 if not HT)
        ht_cap: u16,
        /// A-MPDU parameters from IE 45 byte 3
        ht_param: u8,
        /// STA_FLAG_HT, STA_FLAG_QOS, STA_FLAG_SGI20
        flags: u16,
    },
    /// Remove STA from firmware (mac needed for DISCONNECT MCU command)
    RemoveSta { mac: [u8; 6], aid: u16, wlan_idx: u16 },
    /// Notify firmware about a BA session start/stop (STA_REC_BA MCU command).
    /// Source: Linux mt7996/mcu.c mt7996_mcu_sta_ba()
    NotifyBaSession { mac: [u8; 6], tid: u8, ssn: u16, win_size: u16, start: bool },
    /// STA power save state changed (for TIM bitmap updates)
    StaPsChanged { mac: [u8; 6], aid: u16, ps_mode: bool },
}

/// Result of processing an RX management frame.
/// Up to 12 actions to accommodate BA teardowns (8 TIDs) + RemoveSta + TxFrame.
pub struct ApResult<'a> {
    pub actions: [Option<ApAction<'a>>; 12],
    pub count: usize,
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
        0x2 => MgmtSubtype::ReassocReq, // Reassociation Request
        0x4 => MgmtSubtype::ProbeReq,   // Probe Request
        0xB => MgmtSubtype::Auth,        // Authentication
        0xA => MgmtSubtype::Disassoc,    // Disassociation
        0xC => MgmtSubtype::Deauth,      // Deauthentication
        0xD => MgmtSubtype::Action,      // Action
        _ => MgmtSubtype::Other(subtype),
    }
}
