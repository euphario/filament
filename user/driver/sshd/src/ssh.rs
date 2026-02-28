//! SSH-2 Protocol Implementation
//!
//! Minimal SSH-2 server supporting:
//! - Key exchange: curve25519-sha256
//! - Host key: ssh-ed25519
//! - Encryption: chacha20-poly1305@openssh.com
//! - Authentication: password
//!
//! Reference: RFC 4253 (SSH Transport Layer Protocol)

extern crate alloc;
use alloc::vec::Vec;

use libf::crypto;

// =============================================================================
// SSH Message Types (RFC 4253 § 12)
// =============================================================================

pub mod msg {
    pub const DISCONNECT: u8 = 1;
    pub const IGNORE: u8 = 2;
    pub const UNIMPLEMENTED: u8 = 3;
    pub const SERVICE_REQUEST: u8 = 5;
    pub const SERVICE_ACCEPT: u8 = 6;
    pub const KEXINIT: u8 = 20;
    pub const NEWKEYS: u8 = 21;
    pub const KEX_ECDH_INIT: u8 = 30;  // SSH_MSG_KEX_ECDH_INIT (RFC 5656)
    pub const KEX_ECDH_REPLY: u8 = 31; // SSH_MSG_KEX_ECDH_REPLY
    pub const USERAUTH_REQUEST: u8 = 50;
    pub const USERAUTH_FAILURE: u8 = 51;
    pub const USERAUTH_SUCCESS: u8 = 52;
    pub const CHANNEL_OPEN: u8 = 90;
    pub const CHANNEL_OPEN_CONFIRMATION: u8 = 91;
    pub const CHANNEL_WINDOW_ADJUST: u8 = 93;
    pub const CHANNEL_DATA: u8 = 94;
    pub const CHANNEL_EOF: u8 = 96;
    pub const CHANNEL_CLOSE: u8 = 97;
    pub const CHANNEL_REQUEST: u8 = 98;
    pub const CHANNEL_SUCCESS: u8 = 99;
}

// =============================================================================
// SSH String/Packet Helpers
// =============================================================================

/// Write an SSH string (4-byte BE length + data) to `out`.
/// Returns bytes written.
pub fn write_string(out: &mut [u8], data: &[u8]) -> usize {
    if out.len() < 4 + data.len() {
        return 0;
    }
    let len = data.len() as u32;
    out[0..4].copy_from_slice(&len.to_be_bytes());
    out[4..4 + data.len()].copy_from_slice(data);
    4 + data.len()
}

/// Read an SSH string from `input`. Returns (data_slice, remaining).
pub fn read_string(input: &[u8]) -> Option<(&[u8], &[u8])> {
    if input.len() < 4 {
        return None;
    }
    let len = u32::from_be_bytes([input[0], input[1], input[2], input[3]]) as usize;
    if input.len() < 4 + len {
        return None;
    }
    Some((&input[4..4 + len], &input[4 + len..]))
}

/// Write a u32 in big-endian.
pub fn write_u32(out: &mut [u8], val: u32) -> usize {
    if out.len() < 4 {
        return 0;
    }
    out[0..4].copy_from_slice(&val.to_be_bytes());
    4
}

/// Read a u32 in big-endian.
pub fn read_u32(input: &[u8]) -> Option<(u32, &[u8])> {
    if input.len() < 4 {
        return None;
    }
    let val = u32::from_be_bytes([input[0], input[1], input[2], input[3]]);
    Some((val, &input[4..]))
}

/// Write an SSH boolean.
pub fn write_bool(out: &mut [u8], val: bool) -> usize {
    if out.is_empty() {
        return 0;
    }
    out[0] = if val { 1 } else { 0 };
    1
}

/// Write an SSH mpint (multi-precision integer, RFC 4251 § 5).
/// For curve25519, this is a 32-byte big-endian value with leading zero if high bit set.
pub fn write_mpint(out: &mut [u8], val: &[u8; 32]) -> usize {
    let needs_pad = val[0] & 0x80 != 0;
    let total = if needs_pad { 4 + 1 + 32 } else { 4 + 32 };
    if out.len() < total {
        return 0;
    }
    if needs_pad {
        out[0..4].copy_from_slice(&33u32.to_be_bytes());
        out[4] = 0;
        out[5..37].copy_from_slice(val);
    } else {
        out[0..4].copy_from_slice(&32u32.to_be_bytes());
        out[4..36].copy_from_slice(val);
    }
    total
}

// =============================================================================
// Algorithm Negotiation Strings
// =============================================================================

const KEX_ALGORITHMS: &[u8] = b"curve25519-sha256";
const HOST_KEY_ALGORITHMS: &[u8] = b"ssh-ed25519";
const CIPHERS: &[u8] = b"chacha20-poly1305@openssh.com";
const MACS: &[u8] = b"none"; // implicit with AEAD
const COMPRESSION: &[u8] = b"none";

/// SSH version string (no trailing CRLF).
pub const SSH_VERSION: &[u8] = b"SSH-2.0-FilamentOS_1.0";

// =============================================================================
// KEXINIT Builder
// =============================================================================

/// Build a KEXINIT message (RFC 4253 § 7.1).
///
/// Returns the number of bytes written to `out` (payload, no framing).
pub fn build_kexinit(out: &mut [u8], cookie: &[u8; 16]) -> usize {
    let mut pos = 0;

    // msg type
    out[pos] = msg::KEXINIT;
    pos += 1;

    // 16-byte cookie
    out[pos..pos + 16].copy_from_slice(cookie);
    pos += 16;

    // 10 name-lists: kex, host key, cipher c2s, cipher s2c, mac c2s, mac s2c,
    //                compression c2s, compression s2c, language c2s, language s2c
    let lists: &[&[u8]] = &[
        KEX_ALGORITHMS,
        HOST_KEY_ALGORITHMS,
        CIPHERS, CIPHERS,       // cipher c2s, s2c
        MACS, MACS,             // mac c2s, s2c (none for AEAD)
        COMPRESSION, COMPRESSION, // compression c2s, s2c
        b"", b"",               // language c2s, s2c
    ];

    for list in lists {
        pos += write_string(&mut out[pos..], list);
    }

    // first_kex_packet_follows (boolean, false)
    pos += write_bool(&mut out[pos..], false);

    // reserved (uint32, 0)
    pos += write_u32(&mut out[pos..], 0);

    pos
}

// =============================================================================
// Key Exchange (curve25519-sha256)
// =============================================================================

/// Build the exchange hash H for curve25519-sha256 (RFC 8731).
///
/// H = SHA-256(V_C || V_S || I_C || I_S || K_S || Q_C || Q_S || K)
///
/// Where:
///   V_C = client version string
///   V_S = server version string
///   I_C = client KEXINIT payload
///   I_S = server KEXINIT payload
///   K_S = host key blob (ssh-ed25519 format)
///   Q_C = client ephemeral public key (32 bytes)
///   Q_S = server ephemeral public key (32 bytes)
///   K   = shared secret (mpint)
pub fn compute_exchange_hash(
    v_c: &[u8],       // client version string
    v_s: &[u8],       // server version string
    i_c: &[u8],       // client KEXINIT payload
    i_s: &[u8],       // server KEXINIT payload
    k_s: &[u8],       // host key blob
    q_c: &[u8; 32],   // client ephemeral public
    q_s: &[u8; 32],   // server ephemeral public
    k: &[u8; 32],     // shared secret
) -> [u8; 32] {
    let mut hasher = crypto::Sha256::new();

    // Helper: hash an SSH string (4-byte BE length + data) without a temp buffer.
    // This avoids buffer size issues with large KEXINIT payloads (~1KB).
    let hash_string = |h: &mut crypto::Sha256, data: &[u8]| {
        h.update(&(data.len() as u32).to_be_bytes());
        h.update(data);
    };

    hash_string(&mut hasher, v_c);
    hash_string(&mut hasher, v_s);
    hash_string(&mut hasher, i_c);
    hash_string(&mut hasher, i_s);
    hash_string(&mut hasher, k_s);
    hash_string(&mut hasher, q_c);
    hash_string(&mut hasher, q_s);

    // K is encoded as mpint
    let mut tmp = [0u8; 64];
    let n = write_mpint(&mut tmp, k);
    hasher.update(&tmp[..n]);

    hasher.finalize()
}

/// Build the host key blob for ssh-ed25519.
///
/// Format: string "ssh-ed25519" || string <32-byte public key>
pub fn build_host_key_blob(public_key: &[u8; 32], out: &mut [u8]) -> usize {
    let mut pos = 0;
    pos += write_string(&mut out[pos..], b"ssh-ed25519");
    pos += write_string(&mut out[pos..], public_key);
    pos
}

/// Build KEX_ECDH_REPLY message.
///
/// Content: K_S (host key blob) || Q_S (server ephemeral pub) || signature
pub fn build_kex_reply(
    host_key_blob: &[u8],
    server_ephemeral_pub: &[u8; 32],
    signature: &[u8; 64],
    out: &mut [u8],
) -> usize {
    let mut pos = 0;

    out[pos] = msg::KEX_ECDH_REPLY;
    pos += 1;

    // K_S (host key blob as string)
    pos += write_string(&mut out[pos..], host_key_blob);

    // Q_S (server ephemeral public key as string)
    pos += write_string(&mut out[pos..], server_ephemeral_pub);

    // Signature: string "ssh-ed25519" || string <64-byte sig>
    let mut sig_blob = [0u8; 128];
    let mut spos = 0;
    spos += write_string(&mut sig_blob[spos..], b"ssh-ed25519");
    spos += write_string(&mut sig_blob[spos..], signature);
    pos += write_string(&mut out[pos..], &sig_blob[..spos]);

    pos
}

// =============================================================================
// Key Derivation (RFC 4253 § 7.2)
// =============================================================================

/// Derive a key from the shared secret K, exchange hash H, and session ID.
///
/// HASH(K || H || X || session_id)
/// where X is a single character identifying the key type.
pub fn derive_key(
    k: &[u8; 32],       // shared secret
    h: &[u8; 32],       // exchange hash
    letter: u8,          // key identifier ('A'-'F')
    session_id: &[u8; 32], // session ID (= H for first KEX)
) -> [u8; 32] {
    let mut hasher = crypto::Sha256::new();

    // K as mpint
    let mut tmp = [0u8; 64];
    let n = write_mpint(&mut tmp, k);
    hasher.update(&tmp[..n]);

    // H
    hasher.update(h);

    // single letter
    hasher.update(&[letter]);

    // session_id
    hasher.update(session_id);

    hasher.finalize()
}

/// Derive an extended key (64 bytes) for ciphers that need more than hash_len bytes.
///
/// K1 = HASH(K || H || X || session_id)
/// K2 = HASH(K || H || K1)
/// key = K1 || K2
///
/// Used by chacha20-poly1305@openssh.com which needs 64 bytes per direction
/// (32 for main cipher + 32 for header cipher).
pub fn derive_key_64(
    k: &[u8; 32],
    h: &[u8; 32],
    letter: u8,
    session_id: &[u8; 32],
) -> [u8; 64] {
    let k1 = derive_key(k, h, letter, session_id);

    // K2 = HASH(K || H || K1)
    let mut hasher = crypto::Sha256::new();
    let mut tmp = [0u8; 64];
    let n = write_mpint(&mut tmp, k);
    hasher.update(&tmp[..n]);
    hasher.update(h);
    hasher.update(&k1);
    let k2 = hasher.finalize();

    let mut out = [0u8; 64];
    out[..32].copy_from_slice(&k1);
    out[32..].copy_from_slice(&k2);
    out
}

// =============================================================================
// Packet Framing (RFC 4253 § 6)
// =============================================================================

/// Frame an SSH payload into a binary packet (unencrypted).
///
/// Format: packet_length(4) || padding_length(1) || payload || padding
///
/// Returns total bytes written.
pub fn frame_packet(payload: &[u8], out: &mut [u8]) -> usize {
    let payload_len = payload.len();
    // RFC 4253 § 6: (packet_length(4) + padding_length(1) + payload + padding)
    // must be a multiple of the cipher block size or 8, whichever is larger.
    // Padding must be at least 4 bytes.
    let block_size = 8;
    let remainder = (5 + payload_len) % block_size;
    let padding_len = if remainder == 0 { 0 } else { block_size - remainder };
    let padding_len = if padding_len < 4 { padding_len + block_size } else { padding_len };

    let packet_length = (1 + payload_len + padding_len) as u32;
    let total = 4 + packet_length as usize;

    if out.len() < total {
        return 0;
    }

    out[0..4].copy_from_slice(&packet_length.to_be_bytes());
    out[4] = padding_len as u8;
    out[5..5 + payload_len].copy_from_slice(payload);
    // Zero padding
    for i in 0..padding_len {
        out[5 + payload_len + i] = 0;
    }

    total
}

/// Parse an SSH binary packet (unencrypted).
///
/// Returns the payload slice.
pub fn parse_packet(data: &[u8]) -> Option<&[u8]> {
    if data.len() < 5 {
        return None;
    }
    let packet_length = u32::from_be_bytes([data[0], data[1], data[2], data[3]]) as usize;
    if data.len() < 4 + packet_length {
        return None;
    }
    let padding_length = data[4] as usize;
    let payload_length = packet_length - 1 - padding_length;
    if payload_length > packet_length {
        return None;
    }
    Some(&data[5..5 + payload_length])
}

// =============================================================================
// Encrypted Transport (chacha20-poly1305@openssh.com)
// =============================================================================

/// Encrypted packet state for one direction.
///
/// Implements the chacha20-poly1305@openssh.com wire format:
///
/// 1. Encrypt 4-byte packet_length with ChaCha20(header_key, nonce=seqno, counter=0)
/// 2. Derive Poly1305 OTK from ChaCha20(main_key, nonce=seqno, counter=0) block
/// 3. Encrypt payload with ChaCha20(main_key, nonce=seqno, counter=1)
/// 4. Poly1305 MAC over (encrypted_header || encrypted_payload)
///
/// Wire: encrypted_length(4) || encrypted_payload(N) || poly1305_tag(16)
pub struct EncryptedTransport {
    /// Main cipher key (32 bytes) — payload encryption + Poly1305 OTK.
    main_key: [u8; 32],
    /// Header cipher key (32 bytes) — packet_length encryption only.
    header_key: [u8; 32],
    /// Sequence number (used as nonce).
    seqno: u64,
}

impl EncryptedTransport {
    /// Create from a 64-byte key: bytes 0-31 = main, bytes 32-63 = header.
    /// `initial_seqno` is the sequence number of the first encrypted packet
    /// (counts all prior unencrypted binary packets — version exchange excluded).
    pub fn new(key: &[u8; 64], initial_seqno: u64) -> Self {
        let mut main_key = [0u8; 32];
        let mut header_key = [0u8; 32];
        main_key.copy_from_slice(&key[..32]);
        header_key.copy_from_slice(&key[32..]);
        Self {
            main_key,
            header_key,
            seqno: initial_seqno,
        }
    }

    /// Build the 12-byte nonce from the sequence number.
    /// OpenSSH uses big-endian seqno in the last 8 bytes.
    fn nonce(&self) -> [u8; 12] {
        let mut n = [0u8; 12];
        n[4..12].copy_from_slice(&self.seqno.to_be_bytes());
        n
    }

    /// Encrypt and frame a payload.
    ///
    /// Returns total bytes written to `out`.
    pub fn encrypt_packet(&mut self, payload: &[u8], out: &mut [u8]) -> usize {
        // AEAD framing: for chacha20-poly1305@openssh.com, the 4-byte length is
        // encrypted separately (AAD), so packet_length must be a multiple of
        // block_size (8), NOT (4 + packet_length).
        let payload_len = payload.len();
        let block_size = 8;
        let inner = 1 + payload_len; // padding_length byte + payload
        let remainder = inner % block_size;
        let padding_len = if remainder == 0 { 0 } else { block_size - remainder };
        let padding_len = if padding_len < 4 { padding_len + block_size } else { padding_len };
        let packet_length = (1 + payload_len + padding_len) as u32;
        let plain_len = 4 + packet_length as usize;

        if plain_len > 8192 || out.len() < plain_len + 16 {
            return 0;
        }

        let mut plain = [0u8; 8192];
        plain[0..4].copy_from_slice(&packet_length.to_be_bytes());
        plain[4] = padding_len as u8;
        plain[5..5 + payload_len].copy_from_slice(payload);
        // Zero padding (already zero from array init)

        let nonce = self.nonce();

        // Step 1: Encrypt packet_length (4 bytes) with header key, counter=0
        out[..4].copy_from_slice(&plain[..4]);
        crypto::chacha20_encrypt(&self.header_key, &nonce, 0, &mut out[..4]);

        // Step 2: Encrypt payload (padding_len + payload + padding) with main key, counter=1
        let payload_len = plain_len - 4;
        out[4..4 + payload_len].copy_from_slice(&plain[4..plain_len]);
        crypto::chacha20_encrypt(&self.main_key, &nonce, 1, &mut out[4..4 + payload_len]);

        // Step 3: Derive Poly1305 one-time key from ChaCha20(main_key, nonce, counter=0)
        let mut poly_key = [0u8; 32];
        crypto::chacha20_encrypt(&self.main_key, &nonce, 0, &mut poly_key);

        // Step 4: Compute Poly1305 MAC over encrypted_header + encrypted_payload
        let mac = crypto::poly1305_mac(&poly_key, &out[..4 + payload_len]);

        // Append MAC
        out[4 + payload_len..4 + payload_len + 16].copy_from_slice(&mac);

        self.seqno += 1;
        4 + payload_len + 16
    }

    /// Decrypt a received packet.
    ///
    /// `encrypted_length` is the 4 encrypted bytes already read from the wire.
    /// `data` is the encrypted payload + 16-byte Poly1305 tag.
    /// Returns the decrypted SSH payload (without padding), or None on MAC failure.
    pub fn decrypt_packet(
        &mut self,
        encrypted_length: &[u8; 4],
        data: &[u8],
    ) -> Option<Vec<u8>> {
        if data.len() < 1 + 16 {
            return None;
        }

        let nonce = self.nonce();
        let tag_offset = data.len() - 16;
        let ciphertext = &data[..tag_offset];
        let received_tag = &data[tag_offset..];

        // Step 1: Derive Poly1305 OTK from ChaCha20(main_key, nonce, counter=0)
        let mut poly_key = [0u8; 32];
        crypto::chacha20_encrypt(&self.main_key, &nonce, 0, &mut poly_key);

        // Step 2: Verify MAC over encrypted_length + ciphertext
        let mut mac_input = Vec::with_capacity(4 + ciphertext.len());
        mac_input.extend_from_slice(encrypted_length);
        mac_input.extend_from_slice(ciphertext);
        let expected_tag = crypto::poly1305_mac(&poly_key, &mac_input);

        // Constant-time comparison
        let mut diff = 0u8;
        for i in 0..16 {
            diff |= received_tag[i] ^ expected_tag[i];
        }
        if diff != 0 {
            return None; // MAC verification failed
        }

        // Step 3: Decrypt packet_length with header key, counter=0
        let mut length_bytes = *encrypted_length;
        crypto::chacha20_encrypt(&self.header_key, &nonce, 0, &mut length_bytes);
        let _packet_length = u32::from_be_bytes(length_bytes) as usize;

        // Step 4: Decrypt payload with main key, counter=1
        let mut plaintext = ciphertext.to_vec();
        crypto::chacha20_encrypt(&self.main_key, &nonce, 1, &mut plaintext);

        self.seqno += 1;

        // Parse: padding_length(1) + payload + padding
        if plaintext.is_empty() {
            return None;
        }
        let padding_len = plaintext[0] as usize;
        if padding_len + 1 > plaintext.len() {
            return None;
        }
        let payload_len = plaintext.len() - 1 - padding_len;
        Some(plaintext[1..1 + payload_len].to_vec())
    }

    /// Decrypt just the 4-byte packet length (for reading the rest of the packet).
    pub fn decrypt_length(&self, encrypted: &[u8; 4]) -> u32 {
        let nonce = self.nonce();
        let mut buf = *encrypted;
        crypto::chacha20_encrypt(&self.header_key, &nonce, 0, &mut buf);
        u32::from_be_bytes(buf)
    }

    pub fn seqno(&self) -> u64 {
        self.seqno
    }
}
