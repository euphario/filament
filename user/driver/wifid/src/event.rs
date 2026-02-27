//! MT7996 MCU Event Processing & RX Frame Classification
//!
//! Handles unsolicited MCU events (firmware logs, thermal, watchdog) and
//! classifies received frames by type for diagnostic counters.
//!
//! Source: Linux mt76/mt7996/mac.c, mt76/mt7996/mcu.c

use userlib::udebug;
use crate::regs::*;

// ============================================================================
// RX Frame Classification
// ============================================================================

/// Classification result for a received frame.
/// Beacon/ProbeReq/ProbeResp are kept for MIB counting; Auth/AssocReq/Deauth/Disassoc
/// are folded into MgmtAction (handled by wifi80211's ApManager).
#[derive(Clone, Copy, Debug)]
pub enum RxFrameClass {
    Data,
    Mgmt,
    Beacon,
    ProbeReq,
    ProbeResp,
    Auth,
    AssocReq,
    Deauth,
    Disassoc,
    TxStatus,
    TxrxNotify,
    FwMonitor,
    Unknown(u8),
}

/// Address type from RXD3 bits 17:16
#[derive(Clone, Copy, Debug)]
pub enum AddrType {
    U2M,    // 0 — unicast to me
    Mcast,  // 1 — multicast
    Bcast,  // 2 — broadcast
    Other,  // 3 — other
}

impl AddrType {
    pub fn from_rxd3(rxd3: u32) -> Self {
        match (rxd3 & MT_RXD3_NORMAL_ADDR_TYPE_MASK) >> MT_RXD3_NORMAL_ADDR_TYPE_SHIFT {
            0 => AddrType::U2M,
            1 => AddrType::Mcast,
            2 => AddrType::Bcast,
            _ => AddrType::Other,
        }
    }
}

/// Classification result with metadata
pub struct RxClassResult {
    pub class: RxFrameClass,
    pub band_idx: u8,
    pub addr_type: AddrType,
    pub fcs_err: bool,
}

/// Classify a received frame from its RXD words.
///
/// For NORMAL frames (pkt_type=2), distinguishes data vs management via
/// RXD2 NDATA bit. For management frames, the caller should read the 802.11
/// FC bytes to further distinguish beacon/probe subtypes.
///
/// Source: Linux mt76/mac.c mt76_connac3_mac_fill_rx()
pub fn classify_rx_frame(rxd0: u32, rxd1: u32, rxd2: u32, rxd3: u32) -> RxClassResult {
    let pkt_type = (rxd0 >> MT_RXD0_PKT_TYPE_SHIFT) & 0x1F;
    let band_idx = ((rxd1 & MT_RXD1_NORMAL_BAND_IDX_MASK) >> MT_RXD1_NORMAL_BAND_IDX_SHIFT) as u8;
    let addr_type = AddrType::from_rxd3(rxd3);
    let fcs_err = rxd3 & MT_RXD3_NORMAL_FCS_ERR != 0;

    let class = match pkt_type {
        PKT_TYPE_TXS => RxFrameClass::TxStatus,
        PKT_TYPE_TXRX_NOTIFY => RxFrameClass::TxrxNotify,
        PKT_TYPE_RX_FW_MONITOR => RxFrameClass::FwMonitor,
        PKT_TYPE_NORMAL => {
            if rxd2 & MT_RXD2_NORMAL_NDATA != 0 {
                RxFrameClass::Mgmt // Will be refined by caller reading FC
            } else {
                RxFrameClass::Data
            }
        }
        PKT_TYPE_RX_EVENT => RxFrameClass::Mgmt, // MCU event on data ring (rare)
        other => RxFrameClass::Unknown(other as u8),
    };

    RxClassResult { class, band_idx, addr_type, fcs_err }
}

/// Refine Mgmt classification using 802.11 Frame Control subtype.
/// FC byte 0 bits [7:4] = subtype, bits [3:2] = type (0=mgmt)
///
/// Source: IEEE 802.11-2020 Table 9-1
pub fn refine_mgmt_subtype(fc0: u8) -> RxFrameClass {
    let frame_type = (fc0 >> 2) & 0x3;
    if frame_type != 0 {
        return RxFrameClass::Mgmt; // Not a management frame
    }
    let subtype = (fc0 >> 4) & 0xF;
    match subtype {
        0x0 => RxFrameClass::AssocReq,   // Association Request
        0x4 => RxFrameClass::ProbeReq,   // Probe Request
        0x5 => RxFrameClass::ProbeResp,  // Probe Response
        0x8 => RxFrameClass::Beacon,     // Beacon
        0xA => RxFrameClass::Disassoc,   // Disassociation
        0xB => RxFrameClass::Auth,        // Authentication
        0xC => RxFrameClass::Deauth,      // Deauthentication
        _ => RxFrameClass::Mgmt,
    }
}

// ============================================================================
// Software MIB Counters
// ============================================================================

/// Software RX MIB counters — one per band + one global for MCU events.
/// These track what we actually see in the DMA rings, unlike hardware MIB
/// registers which count at the MAC level.
#[derive(Clone, Copy, Default)]
pub struct RxMibCounters {
    pub total: u32,
    pub data: u32,
    pub mgmt: u32,
    pub beacon: u32,
    pub probe_req: u32,
    pub probe_resp: u32,
    pub auth: u32,
    pub assoc_req: u32,
    pub deauth: u32,
    pub fcs_err: u32,
    pub txs: u32,
    pub txrx_notify: u32,
    pub fw_monitor: u32,
    pub unknown: u32,
    // Per addr_type
    pub unicast: u32,
    pub multicast: u32,
    pub broadcast: u32,
    // MCU events
    pub mcu_events: u32,
    pub mcu_thermal: u32,
    pub mcu_wdt: u32,
}

impl RxMibCounters {
    pub const fn new() -> Self {
        Self {
            total: 0, data: 0, mgmt: 0, beacon: 0, probe_req: 0, probe_resp: 0,
            auth: 0, assoc_req: 0, deauth: 0,
            fcs_err: 0, txs: 0, txrx_notify: 0, fw_monitor: 0, unknown: 0,
            unicast: 0, multicast: 0, broadcast: 0,
            mcu_events: 0, mcu_thermal: 0, mcu_wdt: 0,
        }
    }

    /// Update counters from a classification result
    pub fn record(&mut self, class: RxFrameClass, addr_type: AddrType, fcs_err: bool) {
        self.total = self.total.wrapping_add(1);

        if fcs_err {
            self.fcs_err = self.fcs_err.wrapping_add(1);
        }

        match addr_type {
            AddrType::U2M => self.unicast = self.unicast.wrapping_add(1),
            AddrType::Mcast => self.multicast = self.multicast.wrapping_add(1),
            AddrType::Bcast => self.broadcast = self.broadcast.wrapping_add(1),
            AddrType::Other => {}
        }

        match class {
            RxFrameClass::Data => self.data = self.data.wrapping_add(1),
            RxFrameClass::Mgmt => self.mgmt = self.mgmt.wrapping_add(1),
            RxFrameClass::Beacon => {
                self.mgmt = self.mgmt.wrapping_add(1);
                self.beacon = self.beacon.wrapping_add(1);
            }
            RxFrameClass::ProbeReq => {
                self.mgmt = self.mgmt.wrapping_add(1);
                self.probe_req = self.probe_req.wrapping_add(1);
            }
            RxFrameClass::ProbeResp => {
                self.mgmt = self.mgmt.wrapping_add(1);
                self.probe_resp = self.probe_resp.wrapping_add(1);
            }
            RxFrameClass::Auth => {
                self.mgmt = self.mgmt.wrapping_add(1);
                self.auth = self.auth.wrapping_add(1);
            }
            RxFrameClass::AssocReq => {
                self.mgmt = self.mgmt.wrapping_add(1);
                self.assoc_req = self.assoc_req.wrapping_add(1);
            }
            RxFrameClass::Deauth | RxFrameClass::Disassoc => {
                self.mgmt = self.mgmt.wrapping_add(1);
                self.deauth = self.deauth.wrapping_add(1);
            }
            RxFrameClass::TxStatus => self.txs = self.txs.wrapping_add(1),
            RxFrameClass::TxrxNotify => self.txrx_notify = self.txrx_notify.wrapping_add(1),
            RxFrameClass::FwMonitor => self.fw_monitor = self.fw_monitor.wrapping_add(1),
            RxFrameClass::Unknown(_) => self.unknown = self.unknown.wrapping_add(1),
        }
    }
}

// ============================================================================
// MCU Event Processing
// ============================================================================

/// Process an MCU event from the WM or WA RX ring buffer.
///
/// The buffer contains RXD (32 bytes) + MCU UNI event header.
/// RXD0 bits[31:27] = pkt_type.
///
/// mt7996_mcu_rxd layout (after 32-byte RXD):
///   byte 36: eid (event ID)
///   byte 37: seq
///   byte 38: option (bit 2 = MCU_UNI_CMD_UNSOLICITED_EVENT)
///
/// Source: Linux mt76/mt7996/mcu.c mt7996_mcu_rx_event()
pub fn process_mcu_event(buf: &[u8], counters: &mut RxMibCounters) {
    if buf.len() < 39 {
        return; // Need RXD(32) + uni header through option byte
    }

    // Read RXD0 for pkt_type
    let rxd0 = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    let pkt_type = (rxd0 >> MT_RXD0_PKT_TYPE_SHIFT) & 0x1F;

    if pkt_type != PKT_TYPE_RX_EVENT {
        // Not an MCU event — could be TXRXV or other MCU-ring traffic
        counters.total = counters.total.wrapping_add(1);
        return;
    }

    counters.mcu_events = counters.mcu_events.wrapping_add(1);
    counters.total = counters.total.wrapping_add(1);

    let eid = buf[36];
    let _seq = buf[37];
    let option = buf[38];

    // MCU_UNI_CMD_UNSOLICITED_EVENT = BIT(2) = 0x04
    // Source: Linux mt76/mcu.h
    const MCU_UNI_CMD_UNSOLICITED_EVENT: u8 = 0x04;

    // Not an unsolicited event → command response, handled by wait_rx_response
    if option & MCU_UNI_CMD_UNSOLICITED_EVENT == 0 {
        return;
    }

    // Unsolicited event — log and update counters
    match eid {
        MCU_UNI_EVENT_THERMAL => {
            counters.mcu_thermal = counters.mcu_thermal.wrapping_add(1);
            udebug!("fw_event", "thermal"; eid = eid as u32);
        }
        MCU_UNI_EVENT_FW_LOG => {
            // Firmware log message — just count, don't spam
            udebug!("fw_event", "fw_log");
        }
        MCU_UNI_EVENT_IE_COUNTDOWN => {
            udebug!("fw_event", "ie_countdown");
        }
        MCU_UNI_EVENT_RDD_REPORT => {
            udebug!("fw_event", "rdd_report");
        }
        MCU_UNI_EVENT_ALL_STA_INFO => {
            // Periodic stats response — silenced (too noisy)
        }
        0x0D => {
            // MCU_UNI_CMD_REG_ACCESS response — solicited command response to
            // mcu_get_chan_mib_info/mcu_get_all_sta_info. Silenced.
        }
        _ => {
            udebug!("fw_event", "unknown"; eid = eid as u32);
        }
    }
}
