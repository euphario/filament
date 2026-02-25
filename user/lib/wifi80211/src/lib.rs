//! wifi80211 — Driver-agnostic 802.11 AP stack
//!
//! Provides STA table management, auth/assoc state machine, and
//! 802.11 frame building/parsing. Hardware drivers implement the
//! small `ApAction` dispatch loop; this crate handles protocol logic.

#![no_std]

pub mod ap;
pub mod frame;
pub mod types;
