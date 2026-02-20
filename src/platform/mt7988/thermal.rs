//! MT7988A LVTS (Low Voltage Thermal Sensor)
//!
//! Reads CPU temperature from LVTS controller 0, sensor 0.
//!
//! LVTS base: 0x1100a000, controller stride 0x100, 2 controllers, 4 sensors each.
//! Uses golden_temp=50 default (no efuse read in v1).
//!
//! Reference: Linux drivers/thermal/mediatek/lvts_thermal.c

use super::KERNEL_VIRT_BASE;
use super::clocks;

/// LVTS base address
const LVTS_BASE: usize = 0x1100_A000;

/// Register offsets (relative to controller base)
mod reg {
    /// Monitor control 0 — enable sensing
    pub const MONCTL0: usize = 0x00;
    /// MSR control 0 — measurement period
    pub const MSRCTL0: usize = 0x38;
    /// Sensor connection config
    pub const TSSEL: usize = 0x40;
    /// Calibration scale
    pub const CALSCALE: usize = 0x48;
    /// Config register (for init command protocol)
    pub const CONFIG: usize = 0x50;
    /// Clock enable
    pub const CLKEN: usize = 0xE4;
    /// MSR data 0 — temperature reading for sensor 0
    pub const MSR0: usize = 0x90;
}

/// Golden temperature default (degrees C) when efuse is not read
const GOLDEN_TEMP: i32 = 50;

/// Calibration constant (from Linux lvts_thermal.c)
const COEFF_A: i64 = -204650;

/// Whether LVTS has been initialized
static mut INITIALIZED: bool = false;

#[inline]
fn read_reg(offset: usize) -> u32 {
    let addr = (KERNEL_VIRT_BASE as usize | LVTS_BASE) + offset;
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline]
fn write_reg(offset: usize, value: u32) {
    let addr = (KERNEL_VIRT_BASE as usize | LVTS_BASE) + offset;
    unsafe { core::ptr::write_volatile(addr as *mut u32, value); }
}

/// Small delay (~1us)
#[inline]
fn delay_us(us: u32) {
    for _ in 0..us {
        for _ in 0..100 {
            core::hint::spin_loop();
        }
    }
}

/// Initialize the LVTS thermal sensor.
///
/// Must be called after clocks module is available.
/// This enables the thermal clock, deasserts reset, and starts continuous monitoring.
pub fn init() {
    // Enable clock and deassert reset
    clocks::enable_thermal_clock();
    clocks::deassert_thermal_reset();
    delay_us(100);

    // Enable LVTS clock
    write_reg(reg::CLKEN, 1);
    delay_us(10);

    // Configure sensor connection (TSSEL)
    // Default: all sensors connected, filter enabled
    write_reg(reg::TSSEL, 0x0);

    // Calibration scale — use default (no efuse)
    write_reg(reg::CALSCALE, 0x0);

    // Measurement control — single filter, continuous mode
    write_reg(reg::MSRCTL0, 0x0);

    // Enable monitoring for sensor 0
    // MONCTL0 bit 0 = enable sensor 0
    write_reg(reg::MONCTL0, 0x1);

    // Wait for first measurement
    delay_us(500);

    unsafe { INITIALIZED = true; }
}

/// Read CPU temperature in millidegrees Celsius.
///
/// Returns 0 if LVTS is not initialized or reading is invalid.
///
/// Conversion formula (from Linux lvts_thermal.c):
///   temp_mC = ((raw * coeff_a) >> 14) + golden_offset
/// where golden_offset = (golden_temp * (-coeff_a)) >> 14
pub fn read_cpu_temp_mc() -> i32 {
    if unsafe { !INITIALIZED } {
        return 0;
    }

    let msr = read_reg(reg::MSR0);

    // Bit 16 = valid flag
    if (msr & (1 << 16)) == 0 {
        return 0;
    }

    let raw = (msr & 0xFFFF) as i64;
    if raw == 0 {
        return 0;
    }

    // Linux formula: temp = ((raw * coeff_a) >> 14) + golden_offset
    let golden_offset = (GOLDEN_TEMP as i64 * (-COEFF_A)) >> 14;
    let temp_mc = ((raw * COEFF_A) >> 14) + golden_offset;

    temp_mc as i32
}
