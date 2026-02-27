//! AP state machine — STA table, auth/assoc handling
//!
//! Processes incoming management frames and returns actions for the
//! hardware driver to execute (TX frames, register/remove STAs).

use crate::frame;
use crate::types::*;

/// Associated STA inactivity timeout in ticks. STAs not seen for this many
/// ticks are evicted. At 2 ticks/sec (500ms timer), 600 = 300 seconds.
/// Matches Linux ieee80211_sta_expire default (300s).
pub const STA_AGING_TICKS: u32 = 600;

/// Auth-only STA timeout in ticks. STAs that authenticated but never
/// associated are evicted much faster to avoid table exhaustion from
/// drive-by auth frames. At 2 ticks/sec, 10 = 5 seconds.
pub const STA_AUTH_TIMEOUT_TICKS: u32 = 10;

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
                // Open System auth: allocate/find STA, move to Authenticated.
                // If the table is full, evict the weakest-signal STA to make room.
                let slot = match self.find_or_alloc_sta(&frame.addr2, now) {
                    Some(s) => s,
                    None => {
                        // Table full — evict weakest to make room
                        match self.evict_weakest(frame.rssi) {
                            Some((evicted_aid, evicted_wlan_idx)) => {
                                match self.find_or_alloc_sta(&frame.addr2, now) {
                                    Some(s2) => s2,
                                    None => {
                                        let seq = self.next_seq();
                                        let len = frame::build_deauth(tx_buf, &self.bss.bssid, &frame.addr2, 17, seq);
                                        return ApResult {
                                            action1: if len > 0 { Some(ApAction::TxFrame(&tx_buf[..len])) } else { None },
                                            action2: Some(ApAction::RemoveSta { aid: evicted_aid, wlan_idx: evicted_wlan_idx }),
                                        };
                                    }
                                }
                            }
                            None => {
                                // New STA is weaker than everyone — reject
                                let seq = self.next_seq();
                                let len = frame::build_deauth(tx_buf, &self.bss.bssid, &frame.addr2, 17, seq);
                                return ApResult {
                                    action1: if len > 0 { Some(ApAction::TxFrame(&tx_buf[..len])) } else { None },
                                    action2: None,
                                };
                            }
                        }
                    }
                };
                // Update RSSI + PHY on the STA
                self.stas[slot].rssi = frame.rssi;
                self.stas[slot].phy = frame.phy;
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
                        // Not authenticated — send deauth so client restarts from Auth.
                        // Reason 6 = Class 2 frame from non-authenticated STA.
                        let seq = self.next_seq();
                        let len = frame::build_deauth(tx_buf, &self.bss.bssid, &frame.addr2, 6, seq);
                        return ApResult {
                            action1: if len > 0 { Some(ApAction::TxFrame(&tx_buf[..len])) } else { None },
                            action2: None,
                        };
                    }
                };
                // Update RSSI + PHY on assoc too
                if let Some(sta) = self.stas.iter_mut().find(|s| s.mac == frame.addr2 && s.state != StaState::Free) {
                    sta.rssi = frame.rssi;
                    sta.phy = frame.phy;
                }
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
                let removed = self.remove_sta(&frame.addr2);
                ApResult {
                    action1: removed.map(|(aid, wlan_idx)| ApAction::RemoveSta { aid, wlan_idx }),
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

    /// Iterate over all active (non-Free) STAs for diagnostics.
    pub fn iter_stas(&self) -> impl Iterator<Item = &StaEntry> {
        self.stas.iter().filter(|s| s.state != StaState::Free)
    }

    /// Update last_seen for a STA based on data-path activity (e.g. RX data
    /// frame with known source MAC). Prevents aging out active clients that
    /// only send data, not management frames.
    pub fn touch_sta(&mut self, mac: &[u8; 6], now: u32) {
        for sta in self.stas.iter_mut() {
            if sta.state != StaState::Free && sta.mac == *mac {
                sta.last_seen = now;
                return;
            }
        }
    }

    /// Set the firmware WCID for a STA identified by MAC address.
    pub fn set_sta_wlan_idx(&mut self, mac: &[u8; 6], idx: u16) {
        for sta in self.stas.iter_mut() {
            if sta.state != StaState::Free && sta.mac == *mac {
                sta.wlan_idx = idx;
                return;
            }
        }
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

    /// Evict the weakest-signal STA to make room for a stronger one.
    /// Prefers evicting Authenticated (not yet associated) STAs over Associated.
    /// Returns the AID of the evicted STA (if Associated, caller should notify FW).
    /// Returns None if the new STA's RSSI is weaker than all existing STAs.
    fn evict_weakest(&mut self, new_rssi: i8) -> Option<(u16, u16)> {
        // First pass: find weakest Authenticated-only STA
        let mut weakest_idx: Option<usize> = None;
        let mut weakest_rssi: i8 = 127;
        for (i, sta) in self.stas.iter().enumerate() {
            if sta.state == StaState::Authenticated && sta.rssi < weakest_rssi {
                weakest_rssi = sta.rssi;
                weakest_idx = Some(i);
            }
        }
        // Second pass: if no auth-only, find weakest Associated STA
        if weakest_idx.is_none() {
            for (i, sta) in self.stas.iter().enumerate() {
                if sta.state == StaState::Associated && sta.rssi < weakest_rssi {
                    weakest_rssi = sta.rssi;
                    weakest_idx = Some(i);
                }
            }
        }
        // Only evict if new STA has stronger signal
        if let Some(idx) = weakest_idx {
            if new_rssi > weakest_rssi {
                let aid = self.stas[idx].aid;
                let wlan_idx = self.stas[idx].wlan_idx;
                self.stas[idx] = StaEntry::FREE;
                return Some((aid, wlan_idx));
            }
        }
        None
    }

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

    /// Remove STA, returns (AID, wlan_idx) if it was found.
    fn remove_sta(&mut self, mac: &[u8; 6]) -> Option<(u16, u16)> {
        for sta in self.stas.iter_mut() {
            if sta.state != StaState::Free && sta.mac == *mac {
                let aid = sta.aid;
                let wlan_idx = sta.wlan_idx;
                *sta = StaEntry::FREE;
                return Some((aid, wlan_idx));
            }
        }
        None
    }

    /// Evict STAs based on inactivity. Uses different timeouts:
    /// - Authenticated (not yet associated): STA_AUTH_TIMEOUT_TICKS (5s)
    /// - Associated: STA_AGING_TICKS (30s)
    ///
    /// Returns the number of Associated STAs evicted. The caller should
    /// handle RemoveSta firmware notifications for those.
    /// `evicted` buffer receives (AID, wlan_idx) of evicted Associated STAs.
    pub fn age_stas(&mut self, now: u32, evicted: &mut [(u16, u16)]) -> usize {
        let mut count = 0;
        for sta in self.stas.iter_mut() {
            if sta.state == StaState::Free {
                continue;
            }
            let timeout = if sta.state == StaState::Associated {
                STA_AGING_TICKS
            } else {
                STA_AUTH_TIMEOUT_TICKS
            };
            if now.wrapping_sub(sta.last_seen) >= timeout {
                if sta.state == StaState::Associated && count < evicted.len() {
                    evicted[count] = (sta.aid, sta.wlan_idx);
                    count += 1;
                }
                *sta = StaEntry::FREE;
            }
        }
        count
    }
}
