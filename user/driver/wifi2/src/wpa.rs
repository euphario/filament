//! WPA2-PSK — key derivation, EAPOL 4-way handshake, AES Key Wrap
//!
//! The MT7996 firmware handles per-frame AES-CCMP encrypt/decrypt after keys
//! are installed. This module handles the software side:
//!   - PMK derivation: PBKDF2-HMAC-SHA1(passphrase, SSID, 4096, 32)
//!   - PTK derivation: PRF-384(PMK, nonces, MACs)
//!   - EAPOL-Key frame building/parsing (M1-M4)
//!   - MIC computation: HMAC-SHA1-128 (truncated to 16 bytes)
//!   - GTK wrapping: AES Key Wrap (RFC 3394)
//!
//! Source: IEEE 802.11-2020 §12.7, RFC 3394, RFC 6070

use libf::crypto::{HmacSha1, aes128_encrypt_block};
use wifi80211::types::HandshakeState;

// =============================================================================
// Key Derivation
// =============================================================================

/// PBKDF2-HMAC-SHA1 with 4096 iterations, 32-byte output.
/// IEEE 802.11-2020 §12.7.1.3 — PSK = PBKDF2(passphrase, SSID, 4096, 256)
pub fn pbkdf2_sha1(passphrase: &[u8], ssid: &[u8]) -> [u8; 32] {
    let mut result = [0u8; 32];
    // Two blocks needed for 32-byte output (SHA-1 = 20 bytes per block)
    for block_idx in 0u32..2 {
        let mut u = [0u8; 20];
        let mut accum = [0u8; 20];

        // U1 = HMAC-SHA1(passphrase, ssid || INT(block_idx + 1))
        let mut hmac = HmacSha1::new(passphrase);
        hmac.update(ssid);
        hmac.update(&(block_idx + 1).to_be_bytes());
        u = hmac.finalize();
        accum = u;

        // U2..U4096 — XOR chain
        for _ in 1..4096 {
            u = HmacSha1::mac(passphrase, &u);
            for j in 0..20 {
                accum[j] ^= u[j];
            }
        }

        // Copy to result (block 0 → bytes 0..20, block 1 → bytes 20..32)
        let start = block_idx as usize * 20;
        let end = (start + 20).min(32);
        result[start..end].copy_from_slice(&accum[..end - start]);
    }
    result
}

/// PRF-384 — IEEE 802.11-2020 §12.7.1.2
/// Output: 48 bytes (KCK=16 + KEK=16 + TK=16)
///
/// PRF-X(K, A, B) = HMAC-SHA1(K, A || 0x00 || B || i) for i = 0,1,2
fn prf_384(key: &[u8; 32], label: &[u8], data: &[u8]) -> [u8; 48] {
    let mut result = [0u8; 48];
    // 48 bytes / 20 bytes per HMAC = 3 iterations (ceil)
    for i in 0u8..3 {
        let mut hmac = HmacSha1::new(key);
        hmac.update(label);
        hmac.update(&[0u8]); // zero separator
        hmac.update(data);
        hmac.update(&[i]);
        let block = hmac.finalize();

        let start = i as usize * 20;
        let end = (start + 20).min(48);
        result[start..end].copy_from_slice(&block[..end - start]);
    }
    result
}

/// Derive PTK from PMK, AP address, STA address, and nonces.
/// IEEE 802.11-2020 §12.7.1.3
///
/// PTK = PRF-384(PMK, "Pairwise key expansion",
///               min(AA,SPA) || max(AA,SPA) || min(ANonce,SNonce) || max(ANonce,SNonce))
pub fn derive_ptk(
    pmk: &[u8; 32],
    aa: &[u8; 6],    // Authenticator Address (AP MAC)
    spa: &[u8; 6],   // Supplicant Address (STA MAC)
    anonce: &[u8; 32],
    snonce: &[u8; 32],
) -> [u8; 48] {
    let mut data = [0u8; 76]; // 6+6+32+32

    // Addresses: min first, then max (lexicographic)
    if aa < spa {
        data[0..6].copy_from_slice(aa);
        data[6..12].copy_from_slice(spa);
    } else {
        data[0..6].copy_from_slice(spa);
        data[6..12].copy_from_slice(aa);
    }

    // Nonces: min first, then max (lexicographic)
    if anonce < snonce {
        data[12..44].copy_from_slice(anonce);
        data[44..76].copy_from_slice(snonce);
    } else {
        data[12..44].copy_from_slice(snonce);
        data[44..76].copy_from_slice(anonce);
    }

    prf_384(pmk, b"Pairwise key expansion", &data)
}

/// Compute EAPOL MIC: HMAC-SHA1 truncated to 16 bytes.
/// IEEE 802.11-2020 §12.7.2 (Key descriptor version 2 = HMAC-SHA1-128)
pub fn compute_mic(kck: &[u8], eapol_frame: &[u8]) -> [u8; 16] {
    let full = HmacSha1::mac(kck, eapol_frame);
    let mut mic = [0u8; 16];
    mic.copy_from_slice(&full[..16]);
    mic
}

/// Verify EAPOL MIC: zero the MIC field in the frame, compute, compare.
/// `mic_offset` is the offset of the 16-byte MIC within `eapol_frame`.
pub fn verify_mic(kck: &[u8], eapol_frame: &[u8], mic_offset: usize) -> bool {
    if mic_offset + 16 > eapol_frame.len() {
        return false;
    }
    // Copy frame and zero the MIC field
    let mut buf = [0u8; 256];
    let len = eapol_frame.len().min(buf.len());
    buf[..len].copy_from_slice(&eapol_frame[..len]);
    for b in &mut buf[mic_offset..mic_offset + 16] {
        *b = 0;
    }
    let expected = compute_mic(kck, &buf[..len]);
    // Constant-time comparison
    let mut diff = 0u8;
    for i in 0..16 {
        diff |= eapol_frame[mic_offset + i] ^ expected[i];
    }
    diff == 0
}

// =============================================================================
// AES Key Wrap (RFC 3394)
// =============================================================================

/// AES Key Wrap — RFC 3394 §2.2.1
/// Wraps `plaintext` (must be multiple of 8 bytes) with `kek` (16 bytes).
/// Returns wrapped data: 8 bytes IV + plaintext.len() bytes.
/// `out` must be at least `plaintext.len() + 8` bytes.
/// Returns number of bytes written to `out`.
pub fn aes_key_wrap(kek: &[u8; 16], plaintext: &[u8], out: &mut [u8]) -> usize {
    let n = plaintext.len() / 8; // number of 64-bit blocks
    if n == 0 || plaintext.len() % 8 != 0 || out.len() < plaintext.len() + 8 {
        return 0;
    }

    // A = IV (0xA6A6A6A6A6A6A6A6)
    let mut a = [0xA6u8; 8];
    // R[1..n] = plaintext blocks
    out[8..8 + plaintext.len()].copy_from_slice(plaintext);

    for j in 0u32..6 {
        for i in 1..=n {
            // B = AES(K, A || R[i])
            let mut block = [0u8; 16];
            block[..8].copy_from_slice(&a);
            let r_start = i * 8;
            block[8..16].copy_from_slice(&out[r_start..r_start + 8]);

            let b = aes128_encrypt_block(kek, &block);

            // A = MSB(64, B) ^ t  where t = (n*j)+i
            let t = (n as u64) * (j as u64) + (i as u64);
            a.copy_from_slice(&b[..8]);
            let t_bytes = t.to_be_bytes();
            for k in 0..8 {
                a[k] ^= t_bytes[k];
            }
            // R[i] = LSB(64, B)
            out[r_start..r_start + 8].copy_from_slice(&b[8..16]);
        }
    }

    out[..8].copy_from_slice(&a);
    plaintext.len() + 8
}

// =============================================================================
// EAPOL-Key Frame Building
// =============================================================================

/// EAPOL EtherType
pub const EAPOL_ETHERTYPE: u16 = 0x888E;

// EAPOL header offsets (after Ethernet header)
const EAPOL_VERSION: u8 = 2;       // 802.1X-2004
const EAPOL_TYPE_KEY: u8 = 3;      // EAPOL-Key

// Key descriptor type
const KEY_DESC_TYPE_RSN: u8 = 2;   // IEEE 802.11 RSNA

// Key Info bit fields (u16)
const KEY_INFO_TYPE_HMAC_SHA1: u16 = 2;   // Key descriptor version 2
const KEY_INFO_PAIRWISE: u16 = 1 << 3;
const KEY_INFO_INSTALL: u16 = 1 << 6;
const KEY_INFO_ACK: u16 = 1 << 7;
const KEY_INFO_MIC: u16 = 1 << 8;
const KEY_INFO_SECURE: u16 = 1 << 9;
const KEY_INFO_ENCRYPTED_DATA: u16 = 1 << 12;

/// Build EAPOL-Key M1: AP → STA
/// ANonce, ACK set, no MIC, no encrypted data.
///
/// `buf` layout: Ethernet header (14) + EAPOL header (4) + Key frame (95+)
/// Returns total frame length.
pub fn build_eapol_m1(
    buf: &mut [u8],
    dst_mac: &[u8; 6],
    src_mac: &[u8; 6],
    anonce: &[u8; 32],
    replay_counter: u64,
) -> usize {
    let key_data_len = 0u16;
    let body_len = 95 + key_data_len as usize; // EAPOL-Key body length
    let total = 14 + 4 + body_len;
    if buf.len() < total {
        return 0;
    }
    for b in buf[..total].iter_mut() { *b = 0; }

    // Ethernet header
    buf[0..6].copy_from_slice(dst_mac);
    buf[6..12].copy_from_slice(src_mac);
    buf[12..14].copy_from_slice(&EAPOL_ETHERTYPE.to_be_bytes());

    // EAPOL header
    let eapol = &mut buf[14..];
    eapol[0] = EAPOL_VERSION;
    eapol[1] = EAPOL_TYPE_KEY;
    eapol[2..4].copy_from_slice(&(body_len as u16).to_be_bytes());

    // EAPOL-Key frame
    let key = &mut eapol[4..];
    key[0] = KEY_DESC_TYPE_RSN;
    // Key Info: descriptor version 2 + pairwise + ACK
    let key_info: u16 = KEY_INFO_TYPE_HMAC_SHA1 | KEY_INFO_PAIRWISE | KEY_INFO_ACK;
    key[1..3].copy_from_slice(&key_info.to_be_bytes());
    // Key Length: 16 (CCMP)
    key[3..5].copy_from_slice(&16u16.to_be_bytes());
    // Replay Counter
    key[5..13].copy_from_slice(&replay_counter.to_be_bytes());
    // Key Nonce (ANonce)
    key[13..45].copy_from_slice(anonce);
    // Key IV: zeros (13 bytes at offset 45..61 — not used for WPA2)
    // RSC: zeros (8 bytes at offset 61..69)
    // Reserved: zeros (8 bytes at offset 69..77)
    // Key MIC: zeros (16 bytes at offset 77..93, no MIC in M1)
    // Key Data Length: 0
    key[93..95].copy_from_slice(&key_data_len.to_be_bytes());

    total
}

/// Build EAPOL-Key M3: AP → STA
/// ANonce, MIC, Install, ACK, Secure, Encrypted key data (wrapped GTK + RSN IE).
///
/// Returns total frame length.
pub fn build_eapol_m3(
    buf: &mut [u8],
    dst_mac: &[u8; 6],
    src_mac: &[u8; 6],
    anonce: &[u8; 32],
    replay_counter: u64,
    kck: &[u8],
    kek: &[u8; 16],
    gtk: &[u8; 16],
    gtk_idx: u8,
    rsn_ie: &[u8],
    rsn_ie_len: usize,
) -> usize {
    // Build key data: GTK KDE + RSN IE
    // GTK KDE: type(1) + len(1) + OUI(3) + data_type(1) + key_id(1) + tx(1) + GTK(16) = 24
    // Then RSN IE
    let mut key_data_plain = [0u8; 64];
    let mut kd_len = 0usize;

    // GTK KDE — IEEE 802.11-2020 §12.7.2 Figure 12-34
    key_data_plain[0] = 0xDD; // Type: vendor specific
    key_data_plain[1] = 22;   // Length: OUI(3) + data_type(1) + keyid(1) + tx(1) + gtk(16)
    key_data_plain[2] = 0x00; // OUI: 00:0F:AC
    key_data_plain[3] = 0x0F;
    key_data_plain[4] = 0xAC;
    key_data_plain[5] = 0x01; // Data type: GTK
    key_data_plain[6] = gtk_idx & 0x03; // Key ID (bits 0-1)
    key_data_plain[7] = 0;    // TX bit = 0 for group key from AP
    key_data_plain[8..24].copy_from_slice(gtk);
    kd_len = 24;

    // Append RSN IE
    let rsn_copy_len = rsn_ie_len.min(key_data_plain.len() - kd_len);
    key_data_plain[kd_len..kd_len + rsn_copy_len].copy_from_slice(&rsn_ie[..rsn_copy_len]);
    kd_len += rsn_copy_len;

    // Pad to multiple of 8 for AES Key Wrap
    while kd_len % 8 != 0 {
        key_data_plain[kd_len] = 0xDD; // padding per spec
        kd_len += 1;
    }

    // AES Key Wrap the key data
    let mut key_data_wrapped = [0u8; 80]; // max: 64 + 8
    let wrapped_len = aes_key_wrap(kek, &key_data_plain[..kd_len], &mut key_data_wrapped);
    if wrapped_len == 0 {
        return 0;
    }

    let body_len = 95 + wrapped_len;
    let total = 14 + 4 + body_len;
    if buf.len() < total {
        return 0;
    }
    for b in buf[..total].iter_mut() { *b = 0; }

    // Ethernet header
    buf[0..6].copy_from_slice(dst_mac);
    buf[6..12].copy_from_slice(src_mac);
    buf[12..14].copy_from_slice(&EAPOL_ETHERTYPE.to_be_bytes());

    // EAPOL header
    let eapol = &mut buf[14..];
    eapol[0] = EAPOL_VERSION;
    eapol[1] = EAPOL_TYPE_KEY;
    eapol[2..4].copy_from_slice(&(body_len as u16).to_be_bytes());

    // EAPOL-Key frame
    let key = &mut eapol[4..];
    key[0] = KEY_DESC_TYPE_RSN;
    let key_info: u16 = KEY_INFO_TYPE_HMAC_SHA1 | KEY_INFO_PAIRWISE | KEY_INFO_INSTALL
        | KEY_INFO_ACK | KEY_INFO_MIC | KEY_INFO_SECURE | KEY_INFO_ENCRYPTED_DATA;
    key[1..3].copy_from_slice(&key_info.to_be_bytes());
    key[3..5].copy_from_slice(&16u16.to_be_bytes()); // Key Length: 16 (CCMP)
    key[5..13].copy_from_slice(&replay_counter.to_be_bytes());
    key[13..45].copy_from_slice(anonce);
    // Key IV, RSC, Reserved: zeros
    // Key Data Length
    key[93..95].copy_from_slice(&(wrapped_len as u16).to_be_bytes());
    // Key Data (wrapped)
    key[95..95 + wrapped_len].copy_from_slice(&key_data_wrapped[..wrapped_len]);

    // Compute and insert MIC over the EAPOL frame (from EAPOL header, not Ethernet)
    let mic = compute_mic(kck, &buf[14..total]);
    buf[14 + 4 + 77..14 + 4 + 93].copy_from_slice(&mic);

    total
}

// =============================================================================
// EAPOL-Key Frame Parsing
// =============================================================================

/// Parsed EAPOL M2 from STA
pub struct EapolM2 {
    pub snonce: [u8; 32],
    pub mic: [u8; 16],
}

/// Parsed EAPOL M4 from STA
pub struct EapolM4 {
    pub mic: [u8; 16],
}

/// Parse EAPOL M2 (STA → AP).
/// `frame` starts at the EAPOL header (after Ethernet header, i.e., buf[14..]).
/// Returns SNonce and MIC.
pub fn parse_eapol_m2(frame: &[u8]) -> Option<EapolM2> {
    // Minimum: EAPOL header(4) + key descriptor type(1) + key info(2) + key len(2) +
    //          replay(8) + nonce(32) + key_iv(16) + rsc(8) + reserved(8) + mic(16) + data_len(2) = 99
    if frame.len() < 99 {
        return None;
    }
    if frame[0] != EAPOL_VERSION || frame[1] != EAPOL_TYPE_KEY {
        return None;
    }
    let key = &frame[4..];
    if key[0] != KEY_DESC_TYPE_RSN {
        return None;
    }
    let key_info = u16::from_be_bytes([key[1], key[2]]);
    // M2: Pairwise + MIC set, ACK not set
    if key_info & KEY_INFO_PAIRWISE == 0 || key_info & KEY_INFO_MIC == 0 {
        return None;
    }
    if key_info & KEY_INFO_ACK != 0 {
        return None; // ACK is set in AP frames (M1, M3), not STA frames
    }

    let mut snonce = [0u8; 32];
    snonce.copy_from_slice(&key[13..45]);

    let mut mic = [0u8; 16];
    mic.copy_from_slice(&key[77..93]);

    Some(EapolM2 { snonce, mic })
}

/// Parse EAPOL M4 (STA → AP).
/// `frame` starts at the EAPOL header (after Ethernet header).
pub fn parse_eapol_m4(frame: &[u8]) -> Option<EapolM4> {
    if frame.len() < 99 {
        return None;
    }
    if frame[0] != EAPOL_VERSION || frame[1] != EAPOL_TYPE_KEY {
        return None;
    }
    let key = &frame[4..];
    if key[0] != KEY_DESC_TYPE_RSN {
        return None;
    }
    let key_info = u16::from_be_bytes([key[1], key[2]]);
    // M4: Pairwise + MIC + Secure, no ACK, no encrypted data
    if key_info & KEY_INFO_PAIRWISE == 0 || key_info & KEY_INFO_MIC == 0 {
        return None;
    }
    if key_info & KEY_INFO_ACK != 0 {
        return None;
    }

    let mut mic = [0u8; 16];
    mic.copy_from_slice(&key[77..93]);

    Some(EapolM4 { mic })
}

// =============================================================================
// Handshake Context
// =============================================================================

/// AP-side WPA2-PSK handshake context.
/// Created once with the passphrase and SSID; drives handshakes for all STAs.
pub struct HandshakeCtx {
    /// Pre-computed PMK from passphrase + SSID
    pmk: [u8; 32],
    /// Group Temporal Key (shared among all STAs)
    gtk: [u8; 16],
    /// GTK key index (0-3, typically 1)
    gtk_idx: u8,
    /// Monotonic counter for ANonce generation
    nonce_counter: u32,
    /// BSSID (AP MAC address)
    bssid: [u8; 6],
    /// Pre-built RSN IE for inclusion in M3
    rsn_ie: [u8; 22],
    rsn_ie_len: usize,
}

impl HandshakeCtx {
    /// Create handshake context. Derives PMK from passphrase + SSID.
    /// Generates initial GTK from SHA-256(time || bssid || "GTK").
    pub fn new(passphrase: &[u8], ssid: &[u8], bssid: &[u8; 6], rsn_ie: &[u8], rsn_ie_len: usize) -> Self {
        let pmk = pbkdf2_sha1(passphrase, ssid);

        // Generate GTK: SHA-256(time || bssid || "GTK"), take first 16 bytes
        let time_ns = libsys::syscall::gettime();
        let mut gtk_seed = [0u8; 46]; // 8 + 6 + 32 max label
        gtk_seed[..8].copy_from_slice(&time_ns.to_le_bytes());
        gtk_seed[8..14].copy_from_slice(bssid);
        gtk_seed[14..17].copy_from_slice(b"GTK");
        let gtk_hash = libf::crypto::sha256(&gtk_seed[..17]);
        let mut gtk = [0u8; 16];
        gtk.copy_from_slice(&gtk_hash[..16]);

        let mut rsn_buf = [0u8; 22];
        let copy_len = rsn_ie_len.min(22);
        rsn_buf[..copy_len].copy_from_slice(&rsn_ie[..copy_len]);

        Self {
            pmk,
            gtk,
            gtk_idx: 1,
            nonce_counter: 0,
            bssid: *bssid,
            rsn_ie: rsn_buf,
            rsn_ie_len: copy_len,
        }
    }

    /// Generate a fresh ANonce for a new handshake.
    /// Uses SHA-256(time || bssid || counter) — not cryptographically random,
    /// but sufficient for nonce uniqueness (no getrandom syscall available).
    pub fn generate_anonce(&mut self) -> [u8; 32] {
        self.nonce_counter = self.nonce_counter.wrapping_add(1);
        let time_ns = libsys::syscall::gettime();
        let mut seed = [0u8; 18]; // 8 + 6 + 4
        seed[..8].copy_from_slice(&time_ns.to_le_bytes());
        seed[8..14].copy_from_slice(&self.bssid);
        seed[14..18].copy_from_slice(&self.nonce_counter.to_le_bytes());
        libf::crypto::sha256(&seed)
    }

    /// Build EAPOL M1 for a STA. Sets sta's anonce and replay_counter.
    /// Returns frame length written to `buf`.
    pub fn build_m1(
        &mut self,
        sta_mac: &[u8; 6],
        sta_anonce: &mut [u8; 32],
        sta_replay: &mut u64,
        buf: &mut [u8],
    ) -> usize {
        let anonce = self.generate_anonce();
        *sta_anonce = anonce;
        *sta_replay += 1;
        build_eapol_m1(buf, sta_mac, &self.bssid, &anonce, *sta_replay)
    }

    /// Process M2 from STA: derive PTK, verify MIC.
    /// On success, stores PTK in `ptk_out` and SNonce in `snonce_out`.
    /// `eapol_frame` starts at the EAPOL header (buf[14..]).
    pub fn process_m2(
        &self,
        sta_mac: &[u8; 6],
        anonce: &[u8; 32],
        replay_counter: u64,
        eapol_frame: &[u8],
        ptk_out: &mut [u8; 48],
        snonce_out: &mut [u8; 32],
    ) -> bool {
        let m2 = match parse_eapol_m2(eapol_frame) {
            Some(m) => m,
            None => return false,
        };

        // Check replay counter: M2 must echo our M1 replay counter
        if eapol_frame.len() < 17 {
            return false;
        }
        let rx_replay = u64::from_be_bytes([
            eapol_frame[9], eapol_frame[10], eapol_frame[11], eapol_frame[12],
            eapol_frame[13], eapol_frame[14], eapol_frame[15], eapol_frame[16],
        ]);
        if rx_replay != replay_counter {
            return false;
        }

        // Derive PTK
        let ptk = derive_ptk(&self.pmk, &self.bssid, sta_mac, anonce, &m2.snonce);

        // Verify MIC using KCK (first 16 bytes of PTK)
        // MIC offset in EAPOL frame: 4 (EAPOL hdr) + 77 (key frame offset) = 81
        if !verify_mic(&ptk[..16], eapol_frame, 81) {
            return false;
        }

        *ptk_out = ptk;
        *snonce_out = m2.snonce;
        true
    }

    /// Build EAPOL M3 for a STA.
    /// Returns frame length written to `buf`.
    pub fn build_m3(
        &self,
        sta_mac: &[u8; 6],
        anonce: &[u8; 32],
        replay_counter: &mut u64,
        kck: &[u8],
        kek: &[u8; 16],
        buf: &mut [u8],
    ) -> usize {
        *replay_counter += 1;
        build_eapol_m3(
            buf, sta_mac, &self.bssid, anonce, *replay_counter,
            kck, kek, &self.gtk, self.gtk_idx, &self.rsn_ie, self.rsn_ie_len,
        )
    }

    /// Process M4 from STA: verify MIC.
    /// `eapol_frame` starts at the EAPOL header (buf[14..]).
    pub fn process_m4(&self, kck: &[u8], eapol_frame: &[u8]) -> bool {
        let _m4 = match parse_eapol_m4(eapol_frame) {
            Some(m) => m,
            None => return false,
        };
        // Verify MIC at offset 81
        verify_mic(kck, eapol_frame, 81)
    }

    /// Get the GTK for key installation.
    pub fn gtk(&self) -> &[u8; 16] {
        &self.gtk
    }

    /// Get the GTK key index.
    pub fn gtk_idx(&self) -> u8 {
        self.gtk_idx
    }
}
