//! MT7988A INFRACFG Clock Gate and Reset Helpers
//!
//! Thin wrappers around INFRACFG_AO registers for kernel-internal use
//! (thermal, RNG). Userspace drivers (pcied, usbd) have their own MMIO
//! mappings and don't use these.
//!
//! MediaTek clock gate registers use inverted polarity:
//!   SET register = gate (disable) the clock
//!   CLR register = ungate (enable) the clock
//!   STA register = read current state (1 = gated/disabled)

use super::KERNEL_VIRT_BASE;

/// INFRACFG_AO base address
const INFRACFG_AO_BASE: usize = 0x1000_1000;

// Gate register sets (SET=gate/disable, CLR=ungate/enable, STA=status)
mod infra0 {
    pub const CLR: usize = 0x14;

    /// CLK_INFRA_66M_TRNG (bit 26) — TRNG peripheral clock
    pub const TRNG: u32 = 1 << 26;
}

mod infra2 {
    pub const CLR: usize = 0x54;

    /// CLK_INFRA_26M_THERM_SYSTEM (bit 0)
    pub const THERM_SYSTEM: u32 = 1 << 0;
}

// Reset register sets (SET=assert, CLR=deassert)
mod rst1 {
    pub const CLR: usize = 0x84;

    /// THERM_CTRL_SWRST (bit 9)
    pub const THERM_CTRL_SWRST: u32 = 1 << 9;
}

/// Write to an INFRACFG_AO register
#[inline]
fn write_reg(offset: usize, value: u32) {
    let addr = (KERNEL_VIRT_BASE as usize | INFRACFG_AO_BASE) + offset;
    unsafe {
        core::ptr::write_volatile(addr as *mut u32, value);
    }
}

/// Enable the TRNG peripheral clock (66MHz).
///
/// Writes to INFRA0 CLR register (clear gate = enable clock).
/// Must be called before accessing TRNG registers at 0x1020F000.
pub fn enable_trng_clock() {
    write_reg(infra0::CLR, infra0::TRNG);
}

/// Enable the 26MHz thermal system clock.
///
/// Writes to INFRA2 CLR register (clear gate = enable clock).
pub fn enable_thermal_clock() {
    write_reg(infra2::CLR, infra2::THERM_SYSTEM);
}

/// Deassert thermal controller reset.
///
/// Writes to RST1 CLR register (clear reset = deassert).
pub fn deassert_thermal_reset() {
    write_reg(rst1::CLR, rst1::THERM_CTRL_SWRST);
}
