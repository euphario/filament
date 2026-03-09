//! devc config get/set handlers
//!
//! Separated from main.rs for clarity. Uses BufWriter throughout.

use core::fmt::Write;
use libf::fmt::BufWriter;
use libsys::bus::ConfigKey;

use crate::DriverState;

/// Config keys exposed to `devc get wifi2 <key>`.
pub static CONFIG_KEYS: &[ConfigKey] = &[
    ConfigKey { name: b"status", writable: false },
    ConfigKey { name: b"radio", writable: false },
    ConfigKey { name: b"channel", writable: false },
];

/// Handle `devc get wifi2 <key>`.
pub fn config_get(state: &DriverState, channel: u8, key: &[u8], buf: &mut [u8]) -> usize {
    let mut w = BufWriter::new(buf);

    match key {
        b"status" => {
            w.kv("state", state.name());
            w.kv("channel", channel);
        }
        b"radio" => {
            let on = matches!(state, DriverState::RadioOn | DriverState::ApActive);
            w.on_off(on);
        }
        b"channel" => {
            let _ = write!(w, "{}", channel);
        }
        _ => return 0,
    }

    w.finish()
}
