//! PWM Driver for MT7988 Fan Control
//!
//! Controls the CPU fan via PWM6 on GPIO62.
//!
//! Hardware:
//!   - PWM controller: 0x10048000
//!   - Pinctrl: 0x1001f000
//!   - Fan connector: 3-pin (GND, 5V, PWM)
//!   - PWM6_0 group uses GPIO62

#![no_std]
#![no_main]
#![allow(dead_code)]  // PWM constants for different channels

use userlib::{syscall, MmioRegion, uinfo, uwarn, uerror};

// =============================================================================
// MT7988 PWM Controller
// =============================================================================

/// PWM controller base address
const PWM_BASE: u64 = 0x10048000;
const PWM_SIZE: u64 = 0x1000;

/// MT7988 PWM channel configuration
/// Channel base offset: 0x80, channel spacing: 0x40
const PWM_CHANNEL_BASE: usize = 0x80;
const PWM_CHANNEL_STRIDE: usize = 0x40;

/// PWM6 is used for the fan
const FAN_PWM_CHANNEL: usize = 6;

/// PWM register offsets (relative to channel base)
mod pwm_reg {
    pub const PWMCON: usize = 0x00;      // Control register
    pub const PWMDWIDTH: usize = 0x2C;   // Period width (bits 12:0)
    pub const PWMTHRES: usize = 0x30;    // Duty threshold (bits 12:0)
}

/// PWMCON register bits
mod pwmcon {
    pub const OLD_MODE: u32 = 1 << 15;   // Old PWM mode (ignores CLKSEL)
}

/// Global PWM enable register
const PWM_EN: usize = 0x0000;

// =============================================================================
// MT7988 Pinctrl (GPIO62 → PWM6)
// =============================================================================

/// Pinctrl base address
const PINCTRL_BASE: u64 = 0x1001f000;
const PINCTRL_SIZE: u64 = 0x1000;

/// GPIO mode registers
/// Each register controls 8 pins, 4 bits per pin
/// GPIO62 is in register 62/8 = 7, bits (62%8)*4 = 24-27
const GPIO_MODE_BASE: usize = 0x300;
const GPIO62_MODE_REG: usize = GPIO_MODE_BASE + (62 / 8) * 4;  // 0x31C
const GPIO62_MODE_SHIFT: usize = (62 % 8) * 4;  // 24
const GPIO62_MODE_MASK: u32 = 0xF << 24;

/// PWM function mode value for GPIO62
/// Mode 0 = GPIO, Mode 1-7 = alternate functions
/// PWM6_0 is typically mode 1
const GPIO62_PWM_MODE: u32 = 1;

// =============================================================================
// PWM Driver
// =============================================================================

struct PwmDriver {
    pwm: MmioRegion,
    pinctrl: MmioRegion,
    period: u32,      // PWM period value
    duty: u32,        // Current duty cycle (0-100)
}

impl PwmDriver {
    fn new(pwm: MmioRegion, pinctrl: MmioRegion) -> Self {
        Self {
            pwm,
            pinctrl,
            period: 0,
            duty: 0,
        }
    }

    /// Get channel register offset
    fn channel_offset(&self, channel: usize) -> usize {
        PWM_CHANNEL_BASE + channel * PWM_CHANNEL_STRIDE
    }

    /// Configure GPIO62 for PWM6 function
    ///
    /// MT7988 has a complex pinctrl system with multiple register regions.
    /// The GPIO MODE registers are in the gpio region (0x1001f000).
    /// Pin 62 is in the 7th 32-bit register (62/8 = 7), bits 24-27.
    ///
    /// However, the bootloader (U-Boot) may have already configured this,
    /// and the pinctrl might need additional setup in other iocfg regions.
    fn configure_pinmux(&mut self) -> bool {
        // Standard 8-pins-per-register layout: offset = 0x300 + (pin/8)*4
        // Pin 62: register 7 at offset 0x31C, bits [27:24]
        let mode_offset_v1 = GPIO_MODE_BASE + (62 / 8) * 4;  // 0x31C

        // Read current value
        let mode_v1 = self.pinctrl.read32(mode_offset_v1);

        // Try writing to the standard offset
        let bit_shift = (62 % 8) * 4;  // 24
        let mode_mask = 0xFu32 << bit_shift;
        let new_mode = (mode_v1 & !mode_mask) | (GPIO62_PWM_MODE << bit_shift);

        self.pinctrl.write32(mode_offset_v1, new_mode);

        // Verify
        let verify = self.pinctrl.read32(mode_offset_v1);

        if verify == new_mode && verify != mode_v1 {
            uinfo!("pwm", "pinmux_configured"; gpio = 62u64);
            true
        } else {
            uwarn!("pwm", "pinmux_unchanged"; gpio = 62u64);
            // Continue anyway - PWM might still work (bootloader may have pre-configured)
            true
        }
    }

    /// Initialize PWM6 for fan control
    fn init_pwm(&mut self) -> bool {
        let ch_off = self.channel_offset(FAN_PWM_CHANNEL);

        // Enable PWM6 in global enable register
        let en = self.pwm.read32(PWM_EN);
        self.pwm.write32(PWM_EN, en | (1 << FAN_PWM_CHANNEL));

        // Configure PWMCON: old mode, clock divider = 0 (no division)
        // Old mode uses PWMDWIDTH/PWMTHRES for period/duty
        let pwmcon = pwmcon::OLD_MODE | 0;  // divider = 0
        self.pwm.write32(ch_off + pwm_reg::PWMCON, pwmcon);

        // Set period (max 13 bits = 8191)
        // Use 4096 for easy percentage calculation
        self.period = 4096;
        self.pwm.write32(ch_off + pwm_reg::PWMDWIDTH, self.period);

        // Set initial duty to 0 (fan off)
        self.duty = 0;
        self.pwm.write32(ch_off + pwm_reg::PWMTHRES, 0);

        uinfo!("pwm", "channel_ready"; channel = FAN_PWM_CHANNEL as u64);
        true
    }

    /// Set fan speed (0-100%)
    fn set_fan_speed(&mut self, percent: u32) -> bool {
        let percent = percent.min(100);
        self.duty = percent;

        // Calculate threshold from percentage
        // threshold = period * percent / 100
        let threshold = (self.period * percent) / 100;

        let ch_off = self.channel_offset(FAN_PWM_CHANNEL);
        self.pwm.write32(ch_off + pwm_reg::PWMTHRES, threshold);

        true
    }

    /// Get current fan speed (0-100%)
    fn get_fan_speed(&self) -> u32 {
        self.duty
    }
}

// =============================================================================
// IPC Message Protocol
// =============================================================================

/// PWM IPC commands
const PWM_CMD_SET_FAN: u8 = 1;    // Set fan speed: [1, percent]
const PWM_CMD_GET_FAN: u8 = 2;    // Get fan speed: [2] -> [percent]

// =============================================================================
// Main
// =============================================================================

#[unsafe(no_mangle)]
fn main() {
    uinfo!("pwm", "init_start");

    // Register the "pwm" port
    let port_result = syscall::port_register(b"pwm");
    if port_result < 0 {
        if port_result == -17 {  // EEXIST
            uerror!("pwm", "port_exists");
        } else {
            uerror!("pwm", "port_register_failed"; err = port_result as i64);
        }
        syscall::exit(1);
    }
    let listen_channel = port_result as u32;

    // Open MMIO regions
    let pwm_mmio = match MmioRegion::open(PWM_BASE, PWM_SIZE) {
        Some(m) => m,
        None => {
            uerror!("pwm", "mmio_map_failed"; region = "pwm");
            syscall::exit(1);
        }
    };

    let pinctrl_mmio = match MmioRegion::open(PINCTRL_BASE, PINCTRL_SIZE) {
        Some(m) => m,
        None => {
            uerror!("pwm", "mmio_map_failed"; region = "pinctrl");
            syscall::exit(1);
        }
    };

    // Create driver
    let mut pwm = PwmDriver::new(pwm_mmio, pinctrl_mmio);

    // Configure pinmux
    if !pwm.configure_pinmux() {
        uwarn!("pwm", "pinmux_failed");
    }

    // Initialize PWM
    if !pwm.init_pwm() {
        uerror!("pwm", "init_failed");
        syscall::exit(1);
    }

    // Set initial fan speed to 100%
    pwm.set_fan_speed(100);

    uinfo!("pwm", "init_complete"; channel = listen_channel as u64);

    // Main loop - handle IPC requests (devd supervises us, no need to daemonize)
    let mut msg_buf = [0u8; 16];
    loop {
        // Accept connection
        let client = syscall::port_accept(listen_channel);
        if client < 0 {
            syscall::yield_now();
            continue;
        }
        let client_ch = client as u32;

        // Read request
        let len = syscall::receive(client_ch, &mut msg_buf);
        if len > 0 {
            let cmd = msg_buf[0];
            let response = match cmd {
                PWM_CMD_SET_FAN if len >= 2 => {
                    let percent = msg_buf[1] as u32;
                    pwm.set_fan_speed(percent);
                    percent as i8
                }
                PWM_CMD_GET_FAN => {
                    pwm.get_fan_speed() as i8
                }
                _ => -22i8,  // EINVAL
            };

            // Send response
            let resp = [response as u8];
            let _ = syscall::send(client_ch, &resp);
        }

        syscall::channel_close(client_ch);
    }
}
