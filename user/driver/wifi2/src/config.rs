//! devc config get/set handlers
//!
//! Separated from main.rs for clarity. Uses BufWriter throughout.

use core::fmt::Write;
use libf::fmt::BufWriter;
use libos::bus::ConfigKey;

use crate::DriverState;

/// Config keys exposed to `devc get wifi2 <key>`.
pub static CONFIG_KEYS: &[ConfigKey] = &[
    ConfigKey::read_only(b"state"),
    ConfigKey::read_only(b"radio.on"),
    ConfigKey::read_only(b"radio.channel"),
];

/// Handle `devc get wifi2 <key>`.
pub fn config_get(state: &DriverState, channel: u8, key: &[u8], buf: &mut [u8]) -> usize {
    let mut w = BufWriter::new(buf);

    match key {
        b"state" => {
            let _ = write!(w, "{}", state.name());
        }
        b"radio.on" => {
            let on = matches!(state, DriverState::RadioOn | DriverState::ApActive);
            w.on_off(on);
        }
        b"radio.channel" => {
            let _ = write!(w, "{}", channel);
        }
        _ => return 0,
    }

    w.finish()
}
