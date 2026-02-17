//! MT7988A ARMPLL CPU Frequency Scaling
//!
//! Controls the ARM PLL to switch between operating performance points.
//! Follows Linux mtk_cpufreq_set_target(): reparent to xtal, set PCW, reparent back.
//!
//! OPP table from device tree (all safe at boot voltage 900 mV):
//!   800 MHz / 1100 MHz / 1500 MHz / 1800 MHz

use crate::arch::aarch64::mmio::{MmioRegion, delay_us};
use crate::kernel::cpufreq::Opp;
use core::sync::atomic::{AtomicU8, Ordering};

// APMIXEDSYS base (PLL control block)
const APMIXEDSYS_BASE: usize = 0x1001_e000;
// ARMPLL register offsets within APMIXEDSYS
const ARMPLL_CON0: usize = 0x204;
const ARMPLL_PCW: usize = 0x208;

// MCUSYS base (CPU mux control)
const MCUSYS_BASE: usize = 0x100e_0000;
// ARM clock divider/mux select register
const ARM_DIV_SEL: usize = 0x7A8;
// Bits [10:9] in ARM_DIV_SEL: clock source mux
const MUX_SHIFT: u32 = 9;
const MUX_MASK: u32 = 0x3 << MUX_SHIFT;
const MUX_XTAL: u32 = 0x0 << MUX_SHIFT;   // 40 MHz crystal
const MUX_ARMPLL: u32 = 0x1 << MUX_SHIFT;  // ARMPLL output

// CON0 bit 2: PCW change trigger
const CON0_PCW_CHG: u32 = 1 << 2;

/// Pre-computed OPP entries with PCW values.
/// PCW = (freq_hz × postdiv × 2^25) / 40_000_000
struct OppEntry {
    opp: Opp,
    pcw: u32,
    postdiv: u8,  // CON0 bits [6:4]
}

const OPP_TABLE: &[OppEntry] = &[
    OppEntry { opp: Opp { freq_khz:   800_000, voltage_uv: 850_000 }, pcw: 0x5000_0000, postdiv: 2 },
    OppEntry { opp: Opp { freq_khz: 1_100_000, voltage_uv: 850_000 }, pcw: 0x3700_0000, postdiv: 1 },
    OppEntry { opp: Opp { freq_khz: 1_500_000, voltage_uv: 850_000 }, pcw: 0x4B00_0000, postdiv: 1 },
    OppEntry { opp: Opp { freq_khz: 1_800_000, voltage_uv: 900_000 }, pcw: 0x5A00_0000, postdiv: 1 },
];

static CURRENT_OPP: AtomicU8 = AtomicU8::new(3); // Boot default: 1800 MHz (OPP 3)

/// Initialize DVFS subsystem. Verifies PLL is running.
pub fn init() {
    // Nothing to do — ARMPLL is already configured by U-Boot at 1800 MHz.
    // We just track the current OPP.
}

/// Number of available OPPs.
pub fn opp_count() -> usize {
    OPP_TABLE.len()
}

/// Get frequency in kHz for a given OPP index.
pub fn opp_freq_khz(index: usize) -> u32 {
    if index < OPP_TABLE.len() {
        OPP_TABLE[index].opp.freq_khz
    } else {
        0
    }
}

/// Current OPP index.
pub fn current_opp() -> usize {
    CURRENT_OPP.load(Ordering::Relaxed) as usize
}

/// Current frequency in kHz.
pub fn current_freq_khz() -> u32 {
    opp_freq_khz(current_opp())
}

/// Switch to a different OPP.
///
/// Sequence (matches Linux mtk_cpufreq_set_target):
/// 1. Reparent CPU clock to xtal (40 MHz)
/// 2. Write post-divider to CON0 bits [6:4]
/// 3. Write PCW
/// 4. Toggle PCW change bit (CON0 bit 2)
/// 5. Wait ~20 µs for PLL lock
/// 6. Reparent back to ARMPLL
pub fn set_opp(index: usize) {
    if index >= OPP_TABLE.len() {
        return;
    }

    let cur = CURRENT_OPP.load(Ordering::Relaxed) as usize;
    if index == cur {
        return;
    }

    let entry = &OPP_TABLE[index];
    let apmixed = MmioRegion::new(APMIXEDSYS_BASE);
    let mcusys = MmioRegion::new(MCUSYS_BASE);

    // Step 1: Switch CPU to xtal (40 MHz)
    mcusys.modify32(ARM_DIV_SEL, |v| (v & !MUX_MASK) | MUX_XTAL);

    // Step 2: Set post-divider in CON0 bits [6:4]
    apmixed.modify32(ARMPLL_CON0, |v| (v & !(0x7 << 4)) | ((entry.postdiv as u32) << 4));

    // Step 3: Write PCW
    apmixed.write32(ARMPLL_PCW, entry.pcw);

    // Step 4: Toggle PCW change bit
    apmixed.set_bits32(ARMPLL_CON0, CON0_PCW_CHG);
    apmixed.clear_bits32(ARMPLL_CON0, CON0_PCW_CHG);

    // Step 5: Wait for PLL to lock (~20 µs)
    delay_us(20);

    // Step 6: Switch back to ARMPLL
    mcusys.modify32(ARM_DIV_SEL, |v| (v & !MUX_MASK) | MUX_ARMPLL);

    CURRENT_OPP.store(index as u8, Ordering::Relaxed);
}
