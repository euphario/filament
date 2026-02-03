//! Config API - Trait-based driver configuration
//!
//! `ConfigTransport` is the trait boundary: get/set config on a named service.
//! `DevdConfigTransport` is the concrete implementation that talks to devd.
//! Free functions `get()`, `set()`, `set_raw()`, `list()` use the default
//! transport for convenience — callers never touch the transport directly.
//!
//! If the transport changes (binary protocol, direct channel, etc.), only
//! the `ConfigTransport` impl changes. Callers are unaffected.
//!
//! Usage:
//! ```
//! let mut buf = [0u8; 256];
//! let n = config::get(b"ipd", b"net.ip", &mut buf);
//! let ok = config::set(b"ipd", b"net.ip", b"10.0.2.50");
//! let n = config::list(b"ipd", &mut buf);
//! ```

use crate::ipc::{Channel, Mux, MuxFilter};

/// Response timeout in milliseconds.
const TIMEOUT_MS: u32 = 5_000;

// ============================================================================
// Trait
// ============================================================================

/// Transport-agnostic config operations.
///
/// Implementors handle the wire format, connection management, and timeout.
/// Callers just pass service name, key, value — no protocol knowledge needed.
pub trait ConfigTransport {
    /// Get a config value. Empty key returns full summary (self-documenting).
    /// Returns bytes written to `out`, or 0 on error/timeout.
    fn get(&mut self, service: &[u8], key: &[u8], out: &mut [u8]) -> usize;

    /// Set a config value. Returns bytes written to `out` (the driver's
    /// response, e.g. "OK\n" or "ERR ...\n"), or 0 on error/timeout.
    fn set(&mut self, service: &[u8], key: &[u8], value: &[u8], out: &mut [u8]) -> usize;
}

// ============================================================================
// Devd implementation
// ============================================================================

/// Config transport via devd admin channel (text protocol).
///
/// Stateless: each call opens a channel, sends the command, waits for
/// the response with a timeout, and drops the channel.
pub struct DevdConfigTransport;

impl ConfigTransport for DevdConfigTransport {
    fn get(&mut self, service: &[u8], key: &[u8], out: &mut [u8]) -> usize {
        send_devd_cmd(service, b"GET", key, &[], out)
    }

    fn set(&mut self, service: &[u8], key: &[u8], value: &[u8], out: &mut [u8]) -> usize {
        send_devd_cmd(service, b"SET", key, value, out)
    }
}

// ============================================================================
// Convenience free functions (use default transport)
// ============================================================================

/// Get a config value from a driver. Empty key returns a full summary
/// (self-documenting list of all keys and current values).
/// Returns bytes written to `out`, or 0 on error.
pub fn get(service: &[u8], key: &[u8], out: &mut [u8]) -> usize {
    DevdConfigTransport.get(service, key, out)
}

/// Set a config value on a driver. Returns true on success (response
/// starts with "OK").
pub fn set(service: &[u8], key: &[u8], value: &[u8]) -> bool {
    let mut buf = [0u8; 64];
    let n = DevdConfigTransport.set(service, key, value, &mut buf);
    n >= 2 && buf[0] == b'O' && buf[1] == b'K'
}

/// Set a config value, returning the raw response (including error messages).
/// Returns bytes written to `out`, or 0 on error.
pub fn set_raw(service: &[u8], key: &[u8], value: &[u8], out: &mut [u8]) -> usize {
    DevdConfigTransport.set(service, key, value, out)
}

/// List available config keys for a service (alias for get with empty key).
/// Returns bytes written to `out`, or 0 on error.
pub fn list(service: &[u8], out: &mut [u8]) -> usize {
    get(service, b"", out)
}

// ============================================================================
// Devd text protocol internals
// ============================================================================

/// Format and send a CONFIG command to devd, return response bytes written.
///
/// Wire format: `CONFIG <service> <op> [<key>] [<value>]\n`
fn send_devd_cmd(service: &[u8], op: &[u8], key: &[u8], value: &[u8], out: &mut [u8]) -> usize {
    let mut cmd = [0u8; 192];
    let mut pos = 0;

    pos += copy(&mut cmd[pos..], b"CONFIG ");
    pos += copy(&mut cmd[pos..], service);
    pos += copy(&mut cmd[pos..], b" ");
    pos += copy(&mut cmd[pos..], op);

    if !key.is_empty() {
        pos += copy(&mut cmd[pos..], b" ");
        pos += copy(&mut cmd[pos..], key);
    }

    if !value.is_empty() {
        pos += copy(&mut cmd[pos..], b" ");
        pos += copy(&mut cmd[pos..], value);
    }

    if pos < cmd.len() {
        cmd[pos] = b'\n';
        pos += 1;
    }

    send_and_recv(&cmd[..pos], out)
}

/// Open a channel to devd, send a message, wait for response with timeout.
fn send_and_recv(msg: &[u8], out: &mut [u8]) -> usize {
    let mut ch = match Channel::connect(b"devd:") {
        Ok(ch) => ch,
        Err(_) => return 0,
    };

    if ch.send(msg).is_err() {
        return 0;
    }

    let mux = match Mux::new() {
        Ok(m) => m,
        Err(_) => return 0,
    };
    let _ = mux.add(ch.handle(), MuxFilter::Readable);

    match mux.wait_timeout(TIMEOUT_MS) {
        Ok(_) => match ch.try_recv(out) {
            Ok(Some(n)) => n,
            _ => 0,
        },
        Err(_) => 0,
    }
}

fn copy(dst: &mut [u8], src: &[u8]) -> usize {
    let len = src.len().min(dst.len());
    dst[..len].copy_from_slice(&src[..len]);
    len
}
