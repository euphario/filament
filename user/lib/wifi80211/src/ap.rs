//! AP state machine — STA table, auth/assoc handling
//!
//! Processes incoming management frames and returns actions for the
//! hardware driver to execute (TX frames, register/remove STAs).

use crate::frame;
use crate::types::*;

/// STA inactivity timeout in ticks. STAs not seen for this many ticks
/// are evicted. At 2 ticks/sec (500ms timer), 60 = 30 seconds.
pub const STA_AGING_TICKS: u32 = 60;

/// AP management state. Owns the STA table and BSS config.
pub struct ApManager {
    pub bss: BssConfig,
    stas: [StaEntry; MAX_STAS],
    mgmt_seq: u16,
}

/// Result of processing an RX management frame.
/// Up to 2 actions: typically a TX frame + optional STA registration.
pub struct ApResult<'a> {
    pub action1: Option<ApAction<'a>>,
    pub action2: Option<ApAction<'a>>,
}

impl ApManager {
    pub fn new(bss: BssConfig) -> Self {
        Self {
            bss,
            stas: [StaEntry::FREE; MAX_STAS],
            mgmt_seq: 0,
        }
    }

    /// Process an incoming management frame. Caller provides `tx_buf` as
    /// scratch space for any response frame to be transmitted.
    /// `now` is the current tick counter for STA aging.
    ///
    /// Returns up to 2 actions for the driver to execute.
    pub fn handle_rx_mgmt<'a>(
        &mut self,
        frame: &RxMgmtFrame,
        tx_buf: &'a mut [u8],
        now: u32,
    ) -> ApResult<'a> {
        match frame.subtype {
            MgmtSubtype::ProbeReq => {
                let seq = self.next_seq();
                let len = frame::build_probe_response(tx_buf, &self.bss, &frame.addr2, seq);
                ApResult {
                    action1: if len > 0 { Some(ApAction::TxFrame(&tx_buf[..len])) } else { None },
                    action2: None,
                }
            }
            MgmtSubtype::Auth => {
                // Open System auth: allocate/find STA, move to Authenticated
                self.find_or_alloc_sta(&frame.addr2, now);
                let seq = self.next_seq();
                let len = frame::build_auth_response(tx_buf, &self.bss.bssid, &frame.addr2, seq);
                ApResult {
                    action1: if len > 0 { Some(ApAction::TxFrame(&tx_buf[..len])) } else { None },
                    action2: None,
                }
            }
            MgmtSubtype::AssocReq => {
                // Must be Authenticated first
                let (aid, is_new) = match self.associate_sta(&frame.addr2, now) {
                    Some(v) => v,
                    None => {
                        // Not authenticated — ignore (could send deauth, but keep it simple)
                        return ApResult { action1: None, action2: None };
                    }
                };
                let seq = self.next_seq();
                let len = frame::build_assoc_response(tx_buf, &self.bss, &frame.addr2, aid, seq);
                ApResult {
                    action1: if len > 0 { Some(ApAction::TxFrame(&tx_buf[..len])) } else { None },
                    action2: if is_new {
                        Some(ApAction::RegisterSta { mac: frame.addr2, aid })
                    } else {
                        None
                    },
                }
            }
            MgmtSubtype::Deauth | MgmtSubtype::Disassoc => {
                let aid = self.remove_sta(&frame.addr2);
                ApResult {
                    action1: aid.map(|a| ApAction::RemoveSta { aid: a }),
                    action2: None,
                }
            }
            MgmtSubtype::Other(_) => {
                ApResult { action1: None, action2: None }
            }
        }
    }

    /// Generate a beacon frame. Called periodically by the driver.
    pub fn beacon(&mut self, buf: &mut [u8]) -> usize {
        let seq = self.next_seq();
        frame::build_beacon(buf, &self.bss, seq)
    }

    pub fn sta_count(&self) -> usize {
        self.stas.iter().filter(|s| s.state != StaState::Free).count()
    }

    pub fn find_sta(&self, mac: &[u8; 6]) -> Option<&StaEntry> {
        self.stas.iter().find(|s| s.state != StaState::Free && s.mac == *mac)
    }

    /// Get management sequence number and advance. Used by the driver when it
    /// needs to wrap wifi80211 frames in TXD (the driver manages the shared
    /// sequence counter for frames it builds directly too).
    pub fn next_seq(&mut self) -> u16 {
        let s = self.mgmt_seq;
        self.mgmt_seq = (s + 1) & 0xFFF;
        s
    }

    // -- internal --

    /// Find existing STA or allocate a new slot. Sets state to Authenticated.
    fn find_or_alloc_sta(&mut self, mac: &[u8; 6], now: u32) -> Option<usize> {
        // Check if already exists
        for (i, sta) in self.stas.iter_mut().enumerate() {
            if sta.state != StaState::Free && sta.mac == *mac {
                sta.state = StaState::Authenticated;
                sta.last_seen = now;
                return Some(i);
            }
        }
        // Allocate new slot
        for (i, sta) in self.stas.iter_mut().enumerate() {
            if sta.state == StaState::Free {
                sta.mac = *mac;
                sta.aid = (i as u16) + 1; // 1-based AID
                sta.state = StaState::Authenticated;
                sta.last_seen = now;
                return Some(i);
            }
        }
        None // Table full
    }

    /// Move an Authenticated STA to Associated. Returns (aid, is_newly_associated).
    fn associate_sta(&mut self, mac: &[u8; 6], now: u32) -> Option<(u16, bool)> {
        for sta in self.stas.iter_mut() {
            if sta.mac == *mac && sta.state != StaState::Free {
                let is_new = sta.state != StaState::Associated;
                sta.state = StaState::Associated;
                sta.last_seen = now;
                return Some((sta.aid, is_new));
            }
        }
        None // Not found / not authenticated
    }

    /// Remove STA, returns AID if it was found.
    fn remove_sta(&mut self, mac: &[u8; 6]) -> Option<u16> {
        for sta in self.stas.iter_mut() {
            if sta.state != StaState::Free && sta.mac == *mac {
                let aid = sta.aid;
                *sta = StaEntry::FREE;
                return Some(aid);
            }
        }
        None
    }

    /// Evict STAs that haven't been seen for STA_AGING_TICKS.
    /// Returns the number of STAs evicted. The caller should handle
    /// any RemoveSta firmware notifications for Associated STAs.
    /// `evicted` buffer receives AIDs of evicted Associated STAs.
    pub fn age_stas(&mut self, now: u32, evicted: &mut [u16]) -> usize {
        let mut count = 0;
        for sta in self.stas.iter_mut() {
            if sta.state == StaState::Free {
                continue;
            }
            if now.wrapping_sub(sta.last_seen) >= STA_AGING_TICKS {
                if sta.state == StaState::Associated && count < evicted.len() {
                    evicted[count] = sta.aid;
                    count += 1;
                }
                *sta = StaEntry::FREE;
            }
        }
        count
    }
}
