//! CPU Frequency Scaling
//!
//! Shared types for DVFS. Platform-specific modules implement the actual
//! PLL programming (MT7988A ARMPLL, QEMU stub, future x86 P-states).

/// Operating Performance Point
#[derive(Clone, Copy)]
pub struct Opp {
    pub freq_khz: u32,
    pub voltage_uv: u32,
}
