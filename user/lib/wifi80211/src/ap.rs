//! AP state machine — STA table, auth/assoc handling
//!
//! Processes incoming management frames and returns actions for the
//! hardware driver to execute (TX frames, register/remove STAs).
//!
//! Protocol logic derived from FreeBSD ieee80211_hostap.c and
//! OpenBSD ieee80211_input.c. Uses our flat [StaEntry; 64] array
//! and ApAction return pattern.

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

/// Maximum BA window size we negotiate (64 = standard HT maximum).
/// Source: IEEE 802.11-2020 §9.4.1.14, FreeBSD IEEE80211_AGGR_BAWMAX
const BA_MAX_WIN_SIZE: u16 = 64;

/// AP management state. Owns the STA table and BSS config.
pub struct ApManager {
    pub bss: BssConfig,
    stas: [StaEntry; MAX_STAS],
    mgmt_seq: u16,
}

/// Result of processing an RX management frame.
/// Up to 3 actions: e.g. TxFrame + RegisterSta + NotifyBaSession.
pub struct ApResult<'a> {
    pub actions: [Option<ApAction<'a>>; 3],
}

impl<'a> ApResult<'a> {
    fn none() -> Self {
        Self { actions: [None, None, None] }
    }

    fn one(a: ApAction<'a>) -> Self {
        Self { actions: [Some(a), None, None] }
    }

    fn two(a: ApAction<'a>, b: ApAction<'a>) -> Self {
        Self { actions: [Some(a), Some(b), None] }
    }

    fn three(a: ApAction<'a>, b: ApAction<'a>, c: ApAction<'a>) -> Self {
        Self { actions: [Some(a), Some(b), Some(c)] }
    }
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
    /// Returns up to 3 actions for the driver to execute.
    pub fn handle_rx_mgmt<'a>(
        &mut self,
        frame: &RxMgmtFrame,
        tx_buf: &'a mut [u8],
        now: u32,
    ) -> ApResult<'a> {
        // BSSID validation — reject frames not addressed to our BSS.
        // ProbeReq uses broadcast/wildcard BSSID, so skip validation.
        // Source: FreeBSD hostap_recv_mgmt(), OpenBSD ieee80211_recv_auth()
        if !matches!(frame.subtype, MgmtSubtype::ProbeReq) {
            if frame.addr3 != self.bss.bssid && frame.addr3 != [0xFF; 6] {
                return ApResult::none();
            }
        }

        match frame.subtype {
            MgmtSubtype::ProbeReq => {
                let seq = self.next_seq();
                let len = frame::build_probe_response(tx_buf, &self.bss, &frame.addr2, seq);
                if len > 0 {
                    ApResult::one(ApAction::TxFrame(&tx_buf[..len]))
                } else {
                    ApResult::none()
                }
            }
            MgmtSubtype::Auth => {
                self.handle_auth(frame, tx_buf, now)
            }
            MgmtSubtype::AssocReq | MgmtSubtype::ReassocReq => {
                self.handle_assoc(frame, tx_buf, now)
            }
            MgmtSubtype::Deauth | MgmtSubtype::Disassoc => {
                self.handle_deauth_disassoc(frame)
            }
            MgmtSubtype::Action => {
                self.handle_action(frame, tx_buf, now)
            }
            MgmtSubtype::Other(_) => {
                ApResult::none()
            }
        }
    }

    /// Handle Authentication frame.
    /// Source: FreeBSD hostap_recv_mgmt() AUTH case, OpenBSD ieee80211_recv_auth()
    fn handle_auth<'a>(
        &mut self,
        frame: &RxMgmtFrame,
        tx_buf: &'a mut [u8],
        now: u32,
    ) -> ApResult<'a> {
        // If this STA was already Associated with a firmware WTBL entry,
        // we must remove the old entry before re-registration. Otherwise
        // the firmware ends up with stale WTBL entries that break HDR_TRANS.
        let old_wtbl = self.stas.iter().find(|s| {
            s.state == StaState::Associated && s.mac == frame.addr2 && s.wlan_idx != 0
        }).map(|s| (s.aid, s.wlan_idx));

        let slot = match self.find_or_alloc_sta(&frame.addr2, now) {
            Some(s) => s,
            None => {
                // Table full — evict weakest to make room
                match self.evict_weakest(frame.rssi) {
                    Some((evicted_mac, evicted_aid, evicted_wlan_idx)) => {
                        match self.find_or_alloc_sta(&frame.addr2, now) {
                            Some(s2) => s2,
                            None => {
                                let seq = self.next_seq();
                                let len = frame::build_deauth(tx_buf, &self.bss.bssid, &frame.addr2, 17, seq);
                                return if len > 0 {
                                    ApResult::two(
                                        ApAction::TxFrame(&tx_buf[..len]),
                                        ApAction::RemoveSta { mac: evicted_mac, aid: evicted_aid, wlan_idx: evicted_wlan_idx },
                                    )
                                } else {
                                    ApResult::one(ApAction::RemoveSta { mac: evicted_mac, aid: evicted_aid, wlan_idx: evicted_wlan_idx })
                                };
                            }
                        }
                    }
                    None => {
                        // New STA is weaker than everyone — reject
                        let seq = self.next_seq();
                        let len = frame::build_deauth(tx_buf, &self.bss.bssid, &frame.addr2, 17, seq);
                        return if len > 0 {
                            ApResult::one(ApAction::TxFrame(&tx_buf[..len]))
                        } else {
                            ApResult::none()
                        };
                    }
                }
            }
        };

        // Use validated state transition
        let sta = &mut self.stas[slot];
        match sta.state.transition(StaEvent::Auth) {
            Ok(new_state) => sta.state = new_state,
            Err(_) => {} // Auth from Free is always valid, shouldn't fail
        }

        // Clear old wlan_idx so re-association gets a fresh WTBL entry
        if old_wtbl.is_some() {
            self.stas[slot].wlan_idx = 0;
        }
        // Clear BA sessions on re-auth (STA is starting fresh)
        self.stas[slot].rx_ba = [RxBaSession::INIT; 8];
        self.stas[slot].flags = 0;
        // Update RSSI + PHY on the STA
        self.stas[slot].rssi = frame.rssi;
        self.stas[slot].phy = frame.phy;
        let seq = self.next_seq();
        let len = frame::build_auth_response(tx_buf, &self.bss.bssid, &frame.addr2, seq);
        match (len > 0, old_wtbl) {
            (true, Some((aid, wlan_idx))) => {
                ApResult::two(
                    ApAction::TxFrame(&tx_buf[..len]),
                    ApAction::RemoveSta { mac: frame.addr2, aid, wlan_idx },
                )
            }
            (true, None) => {
                ApResult::one(ApAction::TxFrame(&tx_buf[..len]))
            }
            (false, Some((aid, wlan_idx))) => {
                ApResult::one(ApAction::RemoveSta { mac: frame.addr2, aid, wlan_idx })
            }
            (false, None) => ApResult::none(),
        }
    }

    /// Handle AssocReq / ReassocReq frame.
    /// Source: FreeBSD hostap_recv_mgmt() ASSOC/REASSOC case,
    ///         OpenBSD ieee80211_recv_assoc_req()
    fn handle_assoc<'a>(
        &mut self,
        frame: &RxMgmtFrame,
        tx_buf: &'a mut [u8],
        now: u32,
    ) -> ApResult<'a> {
        let is_reassoc = frame.subtype == MgmtSubtype::ReassocReq;

        // Parse assoc request IEs from frame body
        let body = &frame.body[..frame.body_len as usize];
        let ies = frame::parse_assoc_ies(body, is_reassoc);

        // SSID validation — reject if SSID doesn't match our BSS.
        // Source: FreeBSD hostap_recv_mgmt() step 7, OpenBSD ieee80211_recv_assoc_req()
        if let Some(ref ies) = ies {
            if ies.ssid_len > 0 {
                let our_ssid = &self.bss.ssid[..self.bss.ssid_len as usize];
                let their_ssid = &ies.ssid[..ies.ssid_len as usize];
                if our_ssid != their_ssid {
                    // SSID mismatch — silently ignore (don't send deauth, client
                    // may be scanning). Source: OpenBSD ieee80211_recv_assoc_req()
                    return ApResult::none();
                }
            }
        }

        // Find STA — must be Authenticated first. Use validated transition.
        let slot = match self.find_sta_slot(&frame.addr2) {
            Some(s) => s,
            None => {
                // Not in table — send deauth, Class 2 from non-auth STA
                let seq = self.next_seq();
                let len = frame::build_deauth(tx_buf, &self.bss.bssid, &frame.addr2, 6, seq);
                return if len > 0 {
                    ApResult::one(ApAction::TxFrame(&tx_buf[..len]))
                } else {
                    ApResult::none()
                };
            }
        };

        let sta = &mut self.stas[slot];
        let was_associated = sta.state == StaState::Associated;
        match sta.state.transition(StaEvent::Assoc) {
            Ok(new_state) => sta.state = new_state,
            Err(reason) => {
                // Invalid transition (e.g. Class 2 from non-auth STA)
                let seq = self.next_seq();
                let len = frame::build_deauth(tx_buf, &self.bss.bssid, &frame.addr2, reason, seq);
                return if len > 0 {
                    ApResult::one(ApAction::TxFrame(&tx_buf[..len]))
                } else {
                    ApResult::none()
                };
            }
        }

        // Update STA fields from parsed IEs
        sta.last_seen = now;
        sta.rssi = frame.rssi;
        sta.phy = frame.phy;
        if let Some(ref ies) = ies {
            sta.cap_info = ies.cap_info;
            if ies.has_ht {
                sta.ht_cap = ies.ht_cap;
                sta.ht_param = ies.ht_ampdu_param;
                sta.flags |= STA_FLAG_HT;
                // Check Short GI 20MHz (bit 5 of HT cap info)
                if ies.ht_cap & 0x0020 != 0 {
                    sta.flags |= STA_FLAG_SGI20;
                }
            }
            // QoS: set if capability info bit 9 (QoS) is set, or if HT
            // (HT STAs are always QoS-capable per spec)
            if ies.cap_info & (1 << 9) != 0 || ies.has_ht {
                sta.flags |= STA_FLAG_QOS;
            }
            // TODO: track non-ERP STAs, update ERP IE in beacon
            // TODO: track non-HT STAs, update HT Oper IE protection mode
            // TODO: validate client supports all basic rates
        }

        let aid = sta.aid;
        let is_new = !was_associated;

        let seq = self.next_seq();
        let len = if is_reassoc {
            frame::build_reassoc_response(tx_buf, &self.bss, &frame.addr2, aid, seq)
        } else {
            frame::build_assoc_response(tx_buf, &self.bss, &frame.addr2, aid, seq)
        };
        match (len > 0, is_new) {
            (true, true) => {
                ApResult::two(
                    ApAction::TxFrame(&tx_buf[..len]),
                    ApAction::RegisterSta { mac: frame.addr2, aid },
                )
            }
            (true, false) => {
                ApResult::one(ApAction::TxFrame(&tx_buf[..len]))
            }
            (false, true) => {
                ApResult::one(ApAction::RegisterSta { mac: frame.addr2, aid })
            }
            (false, false) => ApResult::none(),
        }
    }

    /// Handle Deauth / Disassoc frames.
    fn handle_deauth_disassoc<'a>(&mut self, frame: &RxMgmtFrame) -> ApResult<'a> {
        // Tear down all BA sessions before removing STA
        // (firmware cleanup happens via RemoveSta, but track state)
        if let Some(slot) = self.find_sta_slot(&frame.addr2) {
            self.stas[slot].rx_ba = [RxBaSession::INIT; 8];
        }

        let removed = self.remove_sta(&frame.addr2);
        match removed {
            Some((mac, aid, wlan_idx)) => {
                ApResult::one(ApAction::RemoveSta { mac, aid, wlan_idx })
            }
            None => ApResult::none(),
        }
    }

    /// Handle Action frames — BA negotiation, etc.
    /// Source: FreeBSD ieee80211_recv_action(), OpenBSD ieee80211_recv_addba_req()
    fn handle_action<'a>(
        &mut self,
        frame: &RxMgmtFrame,
        tx_buf: &'a mut [u8],
        now: u32,
    ) -> ApResult<'a> {
        // Touch STA last_seen to prevent aging
        self.touch_sta(&frame.addr2, now);

        let body = &frame.body[..frame.body_len as usize];
        if body.len() < 2 {
            return ApResult::none();
        }

        let category = body[0];
        let action = body[1];

        match category {
            // Category 3: Block Ack — IEEE 802.11-2020 §9.6.5
            3 => self.handle_ba_action(frame, tx_buf, body, action),
            // Category 8: SA Query — stub
            // TODO: SA Query for PMF (Protected Management Frames)
            8 => ApResult::none(),
            _ => ApResult::none(),
        }
    }

    /// Handle Block Ack action frames (ADDBA Request, DELBA).
    /// Source: FreeBSD ieee80211_recv_action_ba(), OpenBSD ieee80211_recv_addba_req()
    fn handle_ba_action<'a>(
        &mut self,
        frame: &RxMgmtFrame,
        tx_buf: &'a mut [u8],
        body: &[u8],
        action: u8,
    ) -> ApResult<'a> {
        match action {
            // ADDBA Request (action=0)
            // Body: category(1) + action(1) + token(1) + ba_params(2) + timeout(2) + ssn(2) = 9 bytes
            0 => {
                if body.len() < 9 {
                    return ApResult::none();
                }
                let token = body[2];
                let ba_params = u16::from_le_bytes([body[3], body[4]]);
                let timeout = u16::from_le_bytes([body[5], body[6]]);
                let ssn = u16::from_le_bytes([body[7], body[8]]);

                // Extract TID from ba_params bits [5:2]
                let tid = ((ba_params >> 2) & 0xF) as u8;
                // Extract requested buffer size from ba_params bits [15:6]
                let req_win_size = (ba_params >> 6) & 0x3FF;
                // Negotiate: min of requested and our max
                let win_size = req_win_size.min(BA_MAX_WIN_SIZE);

                if tid > 7 {
                    return ApResult::none();
                }

                // Must be from an associated STA
                let slot = match self.find_sta_slot(&frame.addr2) {
                    Some(s) if self.stas[s].state == StaState::Associated => s,
                    _ => return ApResult::none(),
                };

                // Record the BA session — Source: FreeBSD ieee80211_ampdu_rx_start()
                let sta = &mut self.stas[slot];
                sta.rx_ba[tid as usize] = RxBaSession {
                    state: BaState::Agreed,
                    tid,
                    win_size,
                    ssn,
                    token,
                };

                let mac = sta.mac;
                let seq = self.next_seq();
                let len = frame::build_addba_response(
                    tx_buf, &self.bss.bssid, &frame.addr2,
                    token, tid, win_size, timeout, seq,
                );

                // Return TxFrame + NotifyBaSession so driver tells firmware
                if len > 0 {
                    ApResult::two(
                        ApAction::TxFrame(&tx_buf[..len]),
                        ApAction::NotifyBaSession { mac, tid, ssn, win_size, start: true },
                    )
                } else {
                    ApResult::one(
                        ApAction::NotifyBaSession { mac, tid, ssn, win_size, start: true },
                    )
                }
            }
            // DELBA (action=2)
            // Body: category(1) + action(1) + delba_params(2) + reason(2) = 6 bytes
            2 => {
                if body.len() < 6 {
                    return ApResult::none();
                }
                let delba_params = u16::from_le_bytes([body[2], body[3]]);
                // TID from bits [15:12]
                let tid = ((delba_params >> 12) & 0xF) as u8;
                // Initiator bit [11] — 1 means the sender initiated the DELBA
                // (we are the recipient being notified)

                if tid > 7 {
                    return ApResult::none();
                }

                let slot = match self.find_sta_slot(&frame.addr2) {
                    Some(s) => s,
                    None => return ApResult::none(),
                };

                let sta = &mut self.stas[slot];
                let was_agreed = sta.rx_ba[tid as usize].state == BaState::Agreed;
                sta.rx_ba[tid as usize] = RxBaSession::INIT;

                if was_agreed {
                    let mac = sta.mac;
                    ApResult::one(
                        ApAction::NotifyBaSession { mac, tid, ssn: 0, win_size: 0, start: false },
                    )
                } else {
                    ApResult::none()
                }
            }
            // ADDBA Response (action=1) — we're AP, shouldn't receive this
            // unless we sent ADDBA Request (TX aggregation, not implemented yet)
            _ => ApResult::none(),
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

    /// Find slot index for a STA by MAC.
    fn find_sta_slot(&self, mac: &[u8; 6]) -> Option<usize> {
        self.stas.iter().position(|s| s.state != StaState::Free && s.mac == *mac)
    }

    /// Evict the weakest-signal STA to make room for a stronger one.
    /// Prefers evicting Authenticated (not yet associated) STAs over Associated.
    /// Returns (mac, aid, wlan_idx) of the evicted STA.
    /// Returns None if the new STA's RSSI is weaker than all existing STAs.
    fn evict_weakest(&mut self, new_rssi: i8) -> Option<([u8; 6], u16, u16)> {
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
                let mac = self.stas[idx].mac;
                let aid = self.stas[idx].aid;
                let wlan_idx = self.stas[idx].wlan_idx;
                self.stas[idx] = StaEntry::FREE;
                return Some((mac, aid, wlan_idx));
            }
        }
        None
    }

    /// Find existing STA or allocate a new slot. Sets state to Authenticated.
    fn find_or_alloc_sta(&mut self, mac: &[u8; 6], now: u32) -> Option<usize> {
        // Check if already exists
        for (i, sta) in self.stas.iter_mut().enumerate() {
            if sta.state != StaState::Free && sta.mac == *mac {
                // Use validated state transition
                if let Ok(new_state) = sta.state.transition(StaEvent::Auth) {
                    sta.state = new_state;
                }
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

    /// Remove STA, returns (mac, AID, wlan_idx) if it was found.
    fn remove_sta(&mut self, mac: &[u8; 6]) -> Option<([u8; 6], u16, u16)> {
        for sta in self.stas.iter_mut() {
            if sta.state != StaState::Free && sta.mac == *mac {
                let sta_mac = sta.mac;
                let aid = sta.aid;
                let wlan_idx = sta.wlan_idx;
                *sta = StaEntry::FREE;
                return Some((sta_mac, aid, wlan_idx));
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
    /// `evicted` buffer receives (mac, AID, wlan_idx) of evicted Associated STAs.
    pub fn age_stas(&mut self, now: u32, evicted: &mut [([u8; 6], u16, u16)]) -> usize {
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
                    evicted[count] = (sta.mac, sta.aid, sta.wlan_idx);
                    count += 1;
                }
                *sta = StaEntry::FREE;
            }
        }
        count
    }
}
