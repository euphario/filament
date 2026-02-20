//! MT7988A LVTS (Low Voltage Thermal Sensor)
//!
//! Reads CPU temperature from LVTS controller 0, sensor 0 (CPU_0).
//!
//! LVTS base: 0x1100a000, controller stride 0x100, 2 controllers, 4 sensors each.
//! Calibration data from efuse at 0x11f50918 (40 bytes).
//!
//! MT7988 uses IMMEDIATE mode (reads from LVTS_IMMD0, not MSR0).
//!
//! Reference: Linux drivers/thermal/mediatek/lvts_thermal.c

use super::KERNEL_VIRT_BASE;
use super::clocks;
use crate::kinfo;

/// LVTS base address
const LVTS_BASE: usize = 0x1100_A000;

/// Efuse base address and calibration offset
const EFUSE_BASE: usize = 0x11F5_0000;
const EFUSE_CAL_OFFSET: usize = 0x918;
const EFUSE_CAL_SIZE: usize = 40;

/// Register offsets (relative to controller base)
mod reg {
    pub const MONCTL1: usize = 0x04;
    pub const MONCTL2: usize = 0x08;
    pub const MONINT: usize = 0x0C;
    pub const MSRCTL0: usize = 0x38;
    pub const MSRCTL1: usize = 0x3C;
    pub const TSSEL: usize = 0x40;
    pub const CALSCALE: usize = 0x48;
    pub const ID: usize = 0x4C;
    pub const CONFIG: usize = 0x50;
    pub const EDATA00: usize = 0x54;
    pub const IMMD0: usize = 0xA0;
    pub const CLKEN: usize = 0xE4;
}

/// Calibration constant (from Linux lvts_thermal.c, MT7988-specific)
const COEFF_A: i64 = -204650; // temp_factor
const COEFF_B: i64 = 204650;  // temp_offset

/// Maximum valid golden temp value
const GOLDEN_TEMP_MAX: u32 = 62;

/// Default golden temp when efuse is invalid
const GOLDEN_TEMP_DEFAULT: u32 = 50;

/// MT7988 connection commands (written to CONFIG register)
const CONN_CMDS: &[u32] = &[0xC103FFFF, 0xC502FC55];

/// MT7988 initialization commands (written to CONFIG register)
const INIT_CMDS: &[u32] = &[
    0xC1030300, 0xC1030420, 0xC1030500, 0xC10307A6, 0xC1030CFC,
    0xC1030A8C, 0xC103098D, 0xC10308F1, 0xC1030B04, 0xC1030E01,
    0xC10306B8,
];

/// Sensor 0 calibration byte offsets within efuse data
/// (from Linux mt7988_lvts_ap_data_ctrl sensor cal_offsets)
const SENSOR0_CAL_OFFSETS: [usize; 3] = [0x00, 0x01, 0x02];

/// Whether LVTS has been initialized
static mut INITIALIZED: bool = false;

/// Precomputed golden_temp_offset for temperature conversion
/// golden_temp_offset = golden_temp * 500 + temp_offset
static mut GOLDEN_TEMP_OFFSET: i64 = 0;

#[inline]
fn read_lvts(offset: usize) -> u32 {
    let addr = (KERNEL_VIRT_BASE as usize | LVTS_BASE) + offset;
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline]
fn write_lvts(offset: usize, value: u32) {
    let addr = (KERNEL_VIRT_BASE as usize | LVTS_BASE) + offset;
    unsafe { core::ptr::write_volatile(addr as *mut u32, value); }
}

#[inline]
fn read_efuse(offset: usize) -> u8 {
    let addr = (KERNEL_VIRT_BASE as usize | EFUSE_BASE) + EFUSE_CAL_OFFSET + offset;
    unsafe { core::ptr::read_volatile(addr as *const u8) }
}

#[inline]
fn read_efuse_u32(offset: usize) -> u32 {
    let addr = (KERNEL_VIRT_BASE as usize | EFUSE_BASE) + EFUSE_CAL_OFFSET + offset;
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Small delay (~1us per unit at ~1GHz)
#[inline]
fn delay_us(us: u32) {
    for _ in 0..us {
        for _ in 0..100 {
            core::hint::spin_loop();
        }
    }
}

/// Write a command to the CONFIG register with required 2µs delay.
fn write_config(cmd: u32) {
    write_lvts(reg::CONFIG, cmd);
    delay_us(2);
}

/// Read efuse calibration data and extract golden temp + sensor calibration.
///
/// Returns (golden_temp, calibration_value_for_sensor0).
fn read_calibration() -> (u32, u32) {
    // Golden temp is in bits [31:24] of the first efuse word
    let efuse_word0 = read_efuse_u32(0);
    let gt = (efuse_word0 >> 24) & 0xFF;

    let golden_temp = if gt > 0 && gt < GOLDEN_TEMP_MAX {
        gt
    } else {
        GOLDEN_TEMP_DEFAULT
    };

    // Sensor 0 calibration: 3 bytes packed into u32
    let cal = if gt > 0 {
        (read_efuse(SENSOR0_CAL_OFFSETS[0]) as u32)
            | ((read_efuse(SENSOR0_CAL_OFFSETS[1]) as u32) << 8)
            | ((read_efuse(SENSOR0_CAL_OFFSETS[2]) as u32) << 16)
    } else {
        // No valid efuse data and MT7988 has no def_calibration
        0
    };

    (golden_temp, cal)
}

/// Initialize the LVTS thermal sensor (controller 0 only).
///
/// Follows the Linux lvts_thermal.c init sequence:
/// 1. Enable clock, deassert reset
/// 2. Read efuse calibration
/// 3. Enable LVTS clock
/// 4. Send connection commands
/// 5. Send init commands
/// 6. Write calibration data
/// 7. Configure sensing parameters
/// 8. Start immediate mode measurement
pub fn init() {
    // Enable clock and deassert reset via INFRACFG
    clocks::enable_thermal_clock();
    clocks::deassert_thermal_reset();
    delay_us(100);

    // Read calibration from efuse
    let (golden_temp, cal0) = read_calibration();

    // Precompute golden_temp_offset for temperature conversion
    // Linux: golden_temp_offset = golden_temp * 500 + temp_offset
    unsafe {
        GOLDEN_TEMP_OFFSET = (golden_temp as i64) * 500 + COEFF_B;
    }

    if cal0 == 0 {
        kinfo!("thermal", "no_calibration"; golden_temp = golden_temp as u64);
        // No valid calibration — sensor readings will be invalid
        // but we continue init so we can at least report the raw value
    }

    // Enable LVTS clock
    write_lvts(reg::CLKEN, 1);
    delay_us(10);

    // --- Connection phase ---
    for &cmd in CONN_CMDS {
        write_config(cmd);
    }

    // Verify connection: LVTS_ID bit 7 should be set
    let id = read_lvts(reg::ID);
    if (id & (1 << 7)) == 0 {
        kinfo!("thermal", "not_connected"; id = id as u64);
        return;
    }

    // --- Initialization phase ---
    for &cmd in INIT_CMDS {
        write_config(cmd);
    }

    // --- Calibration phase ---
    // Write calibration for sensor 0, zeros for others
    write_lvts(reg::EDATA00, cal0);
    write_lvts(reg::EDATA00 + 4, 0);
    write_lvts(reg::EDATA00 + 8, 0);
    write_lvts(reg::EDATA00 + 12, 0);

    // --- Configuration phase ---
    // Sensing point index numbering
    write_lvts(reg::TSSEL, 0x1312_1110);

    // ADC voltage rounding
    write_lvts(reg::CALSCALE, 0x300);

    // Filter strategy: all filters = 0 (one sample)
    write_lvts(reg::MSRCTL0, 0x0);

    // Period/interval: 0
    write_lvts(reg::MONCTL1, 0x0);
    write_lvts(reg::MONCTL2, 0x0);

    // Disable all interrupts
    write_lvts(reg::MONINT, 0x0);

    // --- Start immediate mode ---
    // MSRCTL1 bit layout (from Linux):
    //   bit 4: immediate measurement on sensor 0
    //   bit 5: immediate measurement on sensor 1
    //   bit 6: immediate measurement on sensor 2
    //   bit 9: immediate measurement on sensor 3
    write_lvts(reg::MSRCTL1, 1 << 4); // sensor 0 only

    // Wait for first measurement
    delay_us(500);

    unsafe { INITIALIZED = true; }

    // Try an immediate read for diagnostics
    let immd = read_lvts(reg::IMMD0);
    let valid = (immd & (1 << 16)) != 0;
    let raw = immd & 0xFFFF;
    kinfo!("thermal", "ready"; golden_temp = golden_temp as u64, cal = cal0 as u64,
           raw = raw as u64, valid = valid as u64);
}

/// Read CPU temperature in millidegrees Celsius.
///
/// Returns 0 if LVTS is not initialized or reading is invalid.
///
/// Uses IMMEDIATE mode (LVTS_IMMD0).
///
/// Conversion formula (from Linux lvts_thermal.c):
///   temp = ((raw * temp_factor) >> 14) + golden_temp_offset
/// where golden_temp_offset = golden_temp * 500 + temp_offset
pub fn read_cpu_temp_mc() -> i32 {
    if unsafe { !INITIALIZED } {
        return 0;
    }

    let immd = read_lvts(reg::IMMD0);

    // Bit 16 = valid flag — poll briefly if not ready
    if (immd & (1 << 16)) == 0 {
        // Try a few more times with small delays
        for _ in 0..10 {
            delay_us(50);
            let immd2 = read_lvts(reg::IMMD0);
            if (immd2 & (1 << 16)) != 0 {
                let raw = (immd2 & 0xFFFF) as i64;
                if raw == 0 { return 0; }
                let temp = ((raw * COEFF_A) >> 14) + unsafe { GOLDEN_TEMP_OFFSET };
                return temp as i32;
            }
        }
        return 0;
    }

    let raw = (immd & 0xFFFF) as i64;
    if raw == 0 {
        return 0;
    }

    let temp = ((raw * COEFF_A) >> 14) + unsafe { GOLDEN_TEMP_OFFSET };
    temp as i32
}
