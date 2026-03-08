//! Cryptographic primitives — thin wrappers over vendored RustCrypto crates.
//!
//! Pure-Rust software implementations. Future hardware offload (EIP-97
//! crypto engine on MT7988A) can slot in underneath without changing callers.
//!
//! # Algorithms
//!
//! | Algorithm | Crate | Use |
//! |-----------|-------|-----|
//! | SHA-256 | `sha2` | SSH KEX hash, HMAC |
//! | HMAC-SHA-256 | `hmac` | SSH packet integrity |
//! | SHA-1 | `sha1` | WPA2 PBKDF2, EAPOL MIC |
//! | HMAC-SHA-1 | `hmac` | WPA2 key derivation |
//! | AES-128 ECB | `aes` | WPA2 AES Key Wrap (RFC 3394) |
//! | ChaCha20-Poly1305 | `chacha20poly1305` | SSH packet encryption |
//! | X25519 ECDH | `curve25519-dalek` | SSH key exchange |
//! | Ed25519 | `ed25519-dalek` | SSH host key signing/verification |

extern crate alloc;

use sha2::Digest;

// =============================================================================
// SHA-1
// =============================================================================

/// Compute SHA-1 hash of `data` in one shot.
pub fn sha1(data: &[u8]) -> [u8; 20] {
    let mut hasher = sha1::Sha1::new();
    hasher.update(data);
    let result = hasher.finalize();
    let mut out = [0u8; 20];
    out.copy_from_slice(&result);
    out
}

/// Incremental SHA-1 hasher.
pub struct Sha1 {
    inner: sha1::Sha1,
}

impl Sha1 {
    pub fn new() -> Self {
        Self { inner: sha1::Sha1::new() }
    }

    pub fn update(&mut self, data: &[u8]) {
        self.inner.update(data);
    }

    pub fn finalize(self) -> [u8; 20] {
        let result = self.inner.finalize();
        let mut out = [0u8; 20];
        out.copy_from_slice(&result);
        out
    }
}

// =============================================================================
// HMAC-SHA-1
// =============================================================================

/// HMAC-SHA-1 for WPA2 key derivation and EAPOL MIC.
pub struct HmacSha1 {
    inner: hmac::Hmac<sha1::Sha1>,
}

impl HmacSha1 {
    /// Create a new HMAC-SHA-1 instance with the given key.
    pub fn new(key: &[u8]) -> Self {
        use hmac::Mac;
        Self {
            inner: hmac::Hmac::<sha1::Sha1>::new_from_slice(key)
                .expect("HMAC key length is always valid"),
        }
    }

    /// Feed data into the HMAC.
    pub fn update(&mut self, data: &[u8]) {
        use hmac::Mac;
        self.inner.update(data);
    }

    /// Finalize and return the 20-byte MAC.
    pub fn finalize(self) -> [u8; 20] {
        use hmac::Mac;
        let result = self.inner.finalize();
        let mut out = [0u8; 20];
        out.copy_from_slice(&result.into_bytes());
        out
    }

    /// Compute HMAC-SHA-1 in one shot.
    pub fn mac(key: &[u8], data: &[u8]) -> [u8; 20] {
        let mut hmac = Self::new(key);
        hmac.update(data);
        hmac.finalize()
    }

    /// Verify a MAC against expected value (constant-time).
    pub fn verify(key: &[u8], data: &[u8], expected: &[u8; 20]) -> bool {
        let computed = Self::mac(key, data);
        let mut diff = 0u8;
        for i in 0..20 {
            diff |= computed[i] ^ expected[i];
        }
        diff == 0
    }
}

// =============================================================================
// AES-128 ECB (single block)
// =============================================================================

/// Encrypt a single 16-byte block with AES-128 in ECB mode.
///
/// Used for AES Key Wrap (RFC 3394) — wrapping GTK for EAPOL M3.
pub fn aes128_encrypt_block(key: &[u8; 16], block: &[u8; 16]) -> [u8; 16] {
    use aes::cipher::{BlockEncrypt, KeyInit};
    use aes::cipher::generic_array::GenericArray;

    let cipher = aes::Aes128::new(GenericArray::from_slice(key));
    let mut out = GenericArray::clone_from_slice(block);
    cipher.encrypt_block(&mut out);
    let mut result = [0u8; 16];
    result.copy_from_slice(&out);
    result
}

// =============================================================================
// SHA-256
// =============================================================================

/// Compute SHA-256 hash of `data` in one shot.
pub fn sha256(data: &[u8]) -> [u8; 32] {
    let mut hasher = sha2::Sha256::new();
    hasher.update(data);
    let result = hasher.finalize();
    let mut out = [0u8; 32];
    out.copy_from_slice(&result);
    out
}

/// Incremental SHA-256 hasher.
pub struct Sha256 {
    inner: sha2::Sha256,
}

impl Sha256 {
    pub fn new() -> Self {
        Self { inner: sha2::Sha256::new() }
    }

    pub fn update(&mut self, data: &[u8]) {
        self.inner.update(data);
    }

    pub fn finalize(self) -> [u8; 32] {
        let result = self.inner.finalize();
        let mut out = [0u8; 32];
        out.copy_from_slice(&result);
        out
    }
}

// =============================================================================
// HMAC-SHA-256
// =============================================================================

/// HMAC-SHA-256 for packet integrity.
pub struct HmacSha256 {
    inner: hmac::Hmac<sha2::Sha256>,
}

impl HmacSha256 {
    /// Create a new HMAC-SHA-256 instance with the given key.
    pub fn new(key: &[u8]) -> Self {
        use hmac::Mac;
        Self {
            inner: hmac::Hmac::<sha2::Sha256>::new_from_slice(key)
                .expect("HMAC key length is always valid"),
        }
    }

    /// Feed data into the HMAC.
    pub fn update(&mut self, data: &[u8]) {
        use hmac::Mac;
        self.inner.update(data);
    }

    /// Finalize and return the 32-byte MAC.
    pub fn finalize(self) -> [u8; 32] {
        use hmac::Mac;
        let result = self.inner.finalize();
        let mut out = [0u8; 32];
        out.copy_from_slice(&result.into_bytes());
        out
    }

    /// Compute HMAC-SHA-256 in one shot.
    pub fn mac(key: &[u8], data: &[u8]) -> [u8; 32] {
        let mut hmac = Self::new(key);
        hmac.update(data);
        hmac.finalize()
    }

    /// Verify a MAC against expected value.
    pub fn verify(key: &[u8], data: &[u8], expected: &[u8; 32]) -> bool {
        let computed = Self::mac(key, data);
        // Constant-time comparison
        let mut diff = 0u8;
        for i in 0..32 {
            diff |= computed[i] ^ expected[i];
        }
        diff == 0
    }
}

// =============================================================================
// ChaCha20-Poly1305 (AEAD)
// =============================================================================

/// ChaCha20-Poly1305 authenticated encryption.
pub struct ChaCha20Poly1305 {
    inner: chacha20poly1305::ChaCha20Poly1305,
}

impl ChaCha20Poly1305 {
    /// Create a new ChaCha20-Poly1305 cipher with the given 32-byte key.
    pub fn new(key: &[u8; 32]) -> Self {
        use chacha20poly1305::KeyInit;
        Self {
            inner: chacha20poly1305::ChaCha20Poly1305::new(key.into()),
        }
    }

    /// Encrypt plaintext with associated data.
    ///
    /// Returns the ciphertext + 16-byte authentication tag appended.
    /// `out` must be at least `plaintext.len() + 16` bytes.
    /// Returns the number of bytes written to `out`, or None on error.
    pub fn encrypt(
        &self,
        nonce: &[u8; 12],
        aad: &[u8],
        plaintext: &[u8],
        out: &mut [u8],
    ) -> Option<usize> {
        use chacha20poly1305::AeadInPlace;
        use alloc::vec::Vec;

        if out.len() < plaintext.len() + 16 {
            return None;
        }

        let mut buffer = Vec::from(plaintext);
        match self.inner.encrypt_in_place(
            nonce.into(),
            aad,
            &mut buffer,
        ) {
            Ok(()) => {
                let len = buffer.len();
                out[..len].copy_from_slice(&buffer);
                Some(len)
            }
            Err(_) => None,
        }
    }

    /// Decrypt ciphertext with associated data.
    ///
    /// `ciphertext` includes the 16-byte authentication tag at the end.
    /// `out` must be at least `ciphertext.len() - 16` bytes.
    /// Returns the number of plaintext bytes, or None if authentication fails.
    pub fn decrypt(
        &self,
        nonce: &[u8; 12],
        aad: &[u8],
        ciphertext: &[u8],
        out: &mut [u8],
    ) -> Option<usize> {
        use chacha20poly1305::AeadInPlace;
        use alloc::vec::Vec;

        if ciphertext.len() < 16 {
            return None;
        }
        let plaintext_len = ciphertext.len() - 16;
        if out.len() < plaintext_len {
            return None;
        }

        let mut buffer = Vec::from(ciphertext);
        match self.inner.decrypt_in_place(
            nonce.into(),
            aad,
            &mut buffer,
        ) {
            Ok(()) => {
                let len = buffer.len();
                out[..len].copy_from_slice(&buffer);
                Some(len)
            }
            Err(_) => None,
        }
    }
}

// =============================================================================
// Raw ChaCha20 (for OpenSSH chacha20-poly1305@openssh.com)
// =============================================================================

/// Apply ChaCha20 stream cipher in-place.
///
/// Used by OpenSSH's chacha20-poly1305@openssh.com which needs raw ChaCha20
/// (separate from the AEAD construction) for header encryption and payload
/// encryption with explicit counter control.
///
/// `counter` sets the initial block counter (0 for Poly1305 OTK, 1 for payload).
pub fn chacha20_encrypt(key: &[u8; 32], nonce: &[u8; 12], counter: u32, data: &mut [u8]) {
    use chacha20::cipher::{KeyIvInit, StreamCipher, StreamCipherSeek};
    let mut cipher = chacha20::ChaCha20::new(key.into(), nonce.into());
    // Seek to the desired counter position (each block = 64 bytes)
    StreamCipherSeek::seek(&mut cipher, counter as u64 * 64);
    cipher.apply_keystream(data);
}

// =============================================================================
// Raw Poly1305 MAC
// =============================================================================

/// Compute a Poly1305 MAC with the given 32-byte one-time key.
///
/// Used by OpenSSH's chacha20-poly1305@openssh.com to compute the MAC
/// over encrypted data using a key derived from ChaCha20 block 0.
pub fn poly1305_mac(key: &[u8; 32], data: &[u8]) -> [u8; 16] {
    use poly1305::universal_hash::KeyInit;
    use poly1305::Poly1305;

    let mac = Poly1305::new(key.into());
    let result = mac.compute_unpadded(data);
    let mut out = [0u8; 16];
    out.copy_from_slice(result.as_ref());
    out
}

// =============================================================================
// X25519 ECDH Key Exchange
// =============================================================================

/// Perform X25519 Diffie-Hellman key exchange.
///
/// Given a 32-byte secret key and a 32-byte public key, compute the
/// shared secret.
pub fn x25519(secret: &[u8; 32], public: &[u8; 32]) -> [u8; 32] {
    use curve25519_dalek::montgomery::MontgomeryPoint;
    use curve25519_dalek::scalar::Scalar;

    let scalar = Scalar::from_bytes_mod_order(*secret);
    let point = MontgomeryPoint(*public);
    let shared = scalar * point;
    shared.to_bytes()
}

/// Generate an X25519 public key from a secret key.
pub fn x25519_public(secret: &[u8; 32]) -> [u8; 32] {
    use curve25519_dalek::constants::X25519_BASEPOINT;
    use curve25519_dalek::scalar::Scalar;

    let scalar = Scalar::from_bytes_mod_order(*secret);
    let public = scalar * X25519_BASEPOINT;
    public.to_bytes()
}

// =============================================================================
// Ed25519 Signatures
// =============================================================================

/// Sign a message with an Ed25519 secret key.
///
/// The secret key is the 32-byte seed (not the expanded 64-byte key).
pub fn ed25519_sign(secret_key: &[u8; 32], msg: &[u8]) -> [u8; 64] {
    use ed25519_dalek::{SigningKey, Signer};

    let signing_key = SigningKey::from_bytes(secret_key);
    let sig = signing_key.sign(msg);
    sig.to_bytes()
}

/// Verify an Ed25519 signature.
pub fn ed25519_verify(public_key: &[u8; 32], msg: &[u8], signature: &[u8; 64]) -> bool {
    use ed25519_dalek::{VerifyingKey, Signature, Verifier};

    let verifying_key = match VerifyingKey::from_bytes(public_key) {
        Ok(k) => k,
        Err(_) => return false,
    };
    let sig = Signature::from_bytes(signature);
    verifying_key.verify(msg, &sig).is_ok()
}

/// Derive the Ed25519 public key from a secret key (seed).
pub fn ed25519_public_key(secret_key: &[u8; 32]) -> [u8; 32] {
    use ed25519_dalek::SigningKey;

    let signing_key = SigningKey::from_bytes(secret_key);
    signing_key.verifying_key().to_bytes()
}
