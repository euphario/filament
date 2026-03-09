//! PWM Fan Driver for MT7988A
//!
//! Controls the CPU fan via PWM0 on GPIO57.
//!
//! Hardware (from Linux DTS: fan { pwms = <&pwm 0 50000>; }):
//!   - PWM controller: 0x10048000
//!   - Channel 0 (offset 0x80)
//!   - GPIO57, pinmux mode 1
//!   - Fan connector: 3-pin (GND, 5V, PWM)

#![no_std]
#![no_main]

extern crate libsys;

use libsys::bus::{BusCtx, BusError, BusMsg, ConfigKey, Disposition, Driver, KernelBusId};
use libsys::bus_runtime::driver_main;
use libsys::MmioRegion;
use libsys::unotice;

// =============================================================================
// MT7988 PWM Hardware Constants
// =============================================================================

/// PWM channel 0 base offset within PWM controller MMIO
const PWM_CH0_BASE: usize = 0x80;

/// PWM register offsets (relative to channel base)
const PWMCON: usize = 0x00;
const PWMDWIDTH: usize = 0x2C;
const PWMTHRES: usize = 0x30;

/// PWMCON bits
const PWMCON_OLD_MODE: u32 = 1 << 15;

/// Global PWM enable register (offset 0 from PWM base)
const PWM_EN: usize = 0x00;

/// Pinctrl GPIO mode register base (within pinctrl MMIO)
const GPIO_MODE_BASE: usize = 0x300;

/// GPIO57 pinmux: register offset = 0x300 + (57/8)*4 = 0x31C
///                 bit shift = (57%8)*4 = 4
///                 mode value = 1 (PWM0 function)
const GPIO57_MODE_OFFSET: usize = GPIO_MODE_BASE + (57 / 8) * 4; // 0x31C
const GPIO57_MODE_SHIFT: usize = (57 % 8) * 4;                    // 4
const GPIO57_MODE_MASK: u32 = 0xF << GPIO57_MODE_SHIFT;           // 0xF0
const GPIO57_PWM_MODE: u32 = 1;

/// PWM period (13-bit max = 8191). Use 4096 for easy percentage math.
const PWM_PERIOD: u32 = 4096;

/// Minimum duty cycle the fan motor can sustain (below this it stalls).
/// 0 = off, anything 1-19 is clamped up to 20.
const MIN_DUTY_PCT: u32 = 20;

/// Pinctrl base address and size (for MMIO open)
const PINCTRL_BASE: u64 = 0x1001_F000;
const PINCTRL_SIZE: u64 = 0x1000;

// =============================================================================
// Fan Driver
// =============================================================================

struct FanDriver {
    bus_id: Option<KernelBusId>,
    pwm: Option<MmioRegion>,
    duty_pct: u32,
}

impl FanDriver {
    fn new() -> Self {
        Self {
            bus_id: None,
            pwm: None,
            duty_pct: 0,
        }
    }

    /// Configure GPIO57 pinmux to PWM0 function (mode 1)
    fn configure_pinmux(&self, pinctrl: &MmioRegion) {
        let val = pinctrl.read32(GPIO57_MODE_OFFSET);
        let new_val = (val & !GPIO57_MODE_MASK) | (GPIO57_PWM_MODE << GPIO57_MODE_SHIFT);
        pinctrl.write32(GPIO57_MODE_OFFSET, new_val);
    }

    /// Initialize PWM channel 0 and set fan to 100%
    fn init_pwm(&mut self) {
        let pwm = match &self.pwm {
            Some(p) => p,
            None => return,
        };

        // Enable PWM channel 0 in global enable register
        let en = pwm.read32(PWM_EN);
        pwm.write32(PWM_EN, en | (1 << 0));

        // Configure PWMCON: old mode, no clock division
        pwm.write32(PWM_CH0_BASE + PWMCON, PWMCON_OLD_MODE);

        // Set period
        pwm.write32(PWM_CH0_BASE + PWMDWIDTH, PWM_PERIOD);

        // Set 50% duty (quiet default, thermal feedback will adjust if needed)
        pwm.write32(PWM_CH0_BASE + PWMTHRES, PWM_PERIOD / 2);
        self.duty_pct = 50;
    }

    /// Set fan speed (0-100%). 0 = off, 1-19 clamped to MIN_DUTY_PCT.
    fn set_fan_speed(&mut self, percent: u32) {
        let percent = percent.min(100);
        let effective = if percent == 0 { 0 } else { percent.max(MIN_DUTY_PCT) };
        self.duty_pct = percent;

        if let Some(pwm) = &self.pwm {
            let threshold = (PWM_PERIOD * effective) / 100;
            pwm.write32(PWM_CH0_BASE + PWMTHRES, threshold);
        }
    }
}

fn copy_to(buf: &mut [u8], pos: usize, src: &[u8]) -> usize {
    let len = src.len().min(buf.len().saturating_sub(pos));
    buf[pos..pos + len].copy_from_slice(&src[..len]);
    pos + len
}

fn fmt_u32(buf: &mut [u8], pos: usize, val: u32) -> usize {
    let mut tmp = [0u8; 12];
    let len = format_u32(&mut tmp, val);
    copy_to(buf, pos, &tmp[..len])
}

fn format_u32(buf: &mut [u8], val: u32) -> usize {
    if val == 0 {
        if !buf.is_empty() { buf[0] = b'0'; }
        return 1;
    }
    let mut n = val;
    let mut len = 0;
    while n > 0 { len += 1; n /= 10; }
    n = val;
    let w = len.min(buf.len());
    for i in (0..w).rev() {
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    w
}

fn parse_u32(s: &[u8]) -> Option<u32> {
    if s.is_empty() { return None; }
    let mut val: u32 = 0;
    for &b in s {
        if b < b'0' || b > b'9' { return None; }
        val = val.checked_mul(10)?.checked_add((b - b'0') as u32)?;
    }
    Some(val)
}

// =============================================================================
// Config Keys
// =============================================================================

const FAN_CONFIG_KEYS: &[ConfigKey] = &[
    ConfigKey::read_write(b"fan"),
];

// =============================================================================
// Driver Trait Implementation
// =============================================================================

impl Driver for FanDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        let (bus_id, _info) = ctx.claim_kernel_bus(b"/pwm:0")?;
        self.bus_id = Some(bus_id);

        // Map PWM MMIO (base comes from bus info, but we also need pinctrl)
        let pwm_mmio = match MmioRegion::open(0x1004_8000, 0x1000) {
            Some(m) => m,
            None => return Err(BusError::NotPresent),
        };

        // Map pinctrl MMIO for GPIO57 mux configuration
        let pinctrl_mmio = match MmioRegion::open(PINCTRL_BASE, PINCTRL_SIZE) {
            Some(m) => m,
            None => return Err(BusError::NotPresent),
        };

        // Configure GPIO57 → PWM0
        self.configure_pinmux(&pinctrl_mmio);

        self.pwm = Some(pwm_mmio);

        // Initialize PWM channel 0 at 100% (thermal safety default)
        self.init_pwm();

        unotice!("pwmd", "ready"; fan = self.duty_pct as u64);

        Ok(())
    }

    fn command(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) -> Disposition {
        Disposition::Forward
    }

    fn config_keys(&self) -> &[ConfigKey] {
        FAN_CONFIG_KEYS
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        match key {
            b"fan" => {
                let mut p = fmt_u32(buf, 0, self.duty_pct);
                p = copy_to(buf, p, b"%");
                p
            }
            _ => 0,
        }
    }

    fn config_set(&mut self, key: &[u8], value: &[u8], buf: &mut [u8], _ctx: &mut dyn BusCtx) -> usize {
        match key {
            b"fan" => {
                if let Some(pct) = parse_u32(value) {
                    if pct > 100 {
                        return copy_to(buf, 0, b"ERR range 0-100\n");
                    }
                    self.set_fan_speed(pct);
                    let mut p = fmt_u32(buf, 0, self.duty_pct);
                    p = copy_to(buf, p, b"%");
                    p
                } else {
                    copy_to(buf, 0, b"ERR invalid number\n")
                }
            }
            _ => 0,
        }
    }
}

// =============================================================================
// Entry Point
// =============================================================================

#[unsafe(no_mangle)]
fn main() {
    driver_main(b"pwmd", FanDriver::new());
}
