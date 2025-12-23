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

use userlib::{println, syscall, MmioRegion};

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
        println!("  Configuring GPIO62 for PWM6...");

        // Try multiple GPIO mode register offsets based on different interpretations
        // of the PIN_FIELD macro parameters

        // Standard 8-pins-per-register layout: offset = 0x300 + (pin/8)*4
        // Pin 62: register 7 at offset 0x31C, bits [27:24]
        let mode_offset_v1 = GPIO_MODE_BASE + (62 / 8) * 4;  // 0x31C

        // Alternative with 0x10 stride per register group
        // This would be 0x300 + (pin/8)*0x10 for some interpretations
        let mode_offset_v2 = GPIO_MODE_BASE + (62 / 8) * 0x10;  // 0x370

        // Read both to see which one has meaningful data
        let mode_v1 = self.pinctrl.read32(mode_offset_v1);
        let mode_v2 = self.pinctrl.read32(mode_offset_v2);

        println!("    GPIO mode reg 0x{:03x}: 0x{:08x}", mode_offset_v1, mode_v1);
        println!("    GPIO mode reg 0x{:03x}: 0x{:08x}", mode_offset_v2, mode_v2);

        // Also check IOCFG regions which might control higher GPIOs
        // iocfg_rb at 0x11d20000 might control GPIO62
        // (Can't map this without another MmioRegion, so just note it)
        println!("    Note: GPIO62 might be controlled by iocfg_rb at 0x11d20000");

        // Try writing to the standard offset
        let bit_shift = (62 % 8) * 4;  // 24
        let mode_mask = 0xFu32 << bit_shift;
        let new_mode = (mode_v1 & !mode_mask) | (GPIO62_PWM_MODE << bit_shift);

        self.pinctrl.write32(mode_offset_v1, new_mode);

        // Verify
        let verify = self.pinctrl.read32(mode_offset_v1);
        println!("    GPIO mode reg after: 0x{:08x}", verify);

        if verify == new_mode && verify != mode_v1 {
            println!("    GPIO62 configured for PWM6");
            true
        } else {
            println!("    WARNING: GPIO mode register unchanged");
            println!("    Bootloader may have pre-configured, or different region needed");
            // Continue anyway - PWM might still work
            true
        }
    }

    /// Initialize PWM6 for fan control
    fn init_pwm(&mut self) -> bool {
        println!("  Initializing PWM6...");

        let ch_off = self.channel_offset(FAN_PWM_CHANNEL);

        // Enable PWM6 in global enable register
        let en = self.pwm.read32(PWM_EN);
        self.pwm.write32(PWM_EN, en | (1 << FAN_PWM_CHANNEL));
        println!("    PWM enable: 0x{:08x} -> 0x{:08x}", en, self.pwm.read32(PWM_EN));

        // Configure PWMCON: old mode, clock divider = 0 (no division)
        // Old mode uses PWMDWIDTH/PWMTHRES for period/duty
        let pwmcon = pwmcon::OLD_MODE | 0;  // divider = 0
        self.pwm.write32(ch_off + pwm_reg::PWMCON, pwmcon);
        println!("    PWMCON: 0x{:08x}", self.pwm.read32(ch_off + pwm_reg::PWMCON));

        // Set period (max 13 bits = 8191)
        // Use 4096 for easy percentage calculation
        self.period = 4096;
        self.pwm.write32(ch_off + pwm_reg::PWMDWIDTH, self.period);
        println!("    PWMDWIDTH (period): {}", self.pwm.read32(ch_off + pwm_reg::PWMDWIDTH));

        // Set initial duty to 0 (fan off)
        self.duty = 0;
        self.pwm.write32(ch_off + pwm_reg::PWMTHRES, 0);
        println!("    PWMTHRES (duty): {}", self.pwm.read32(ch_off + pwm_reg::PWMTHRES));

        println!("    PWM6 initialized");
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
    println!("========================================");
    println!("  PWM Fan Driver");
    println!("  MT7988 PWM6 on GPIO62");
    println!("========================================");
    println!();

    // Register the "pwm" port
    let port_result = syscall::port_register(b"pwm");
    if port_result < 0 {
        if port_result == -17 {  // EEXIST
            println!("ERROR: PWM driver already running");
        } else {
            println!("ERROR: Failed to register PWM port: {}", port_result);
        }
        syscall::exit(1);
    }
    let listen_channel = port_result as u32;
    println!("  Registered 'pwm' port (channel {})", listen_channel);

    // Open MMIO regions
    println!();
    println!("=== Opening MMIO Regions ===");

    let pwm_mmio = match MmioRegion::open(PWM_BASE, PWM_SIZE) {
        Some(m) => {
            println!("  PWM controller: 0x{:08x}", m.virt_base());
            m
        }
        None => {
            println!("ERROR: Failed to map PWM controller");
            syscall::exit(1);
        }
    };

    let pinctrl_mmio = match MmioRegion::open(PINCTRL_BASE, PINCTRL_SIZE) {
        Some(m) => {
            println!("  Pinctrl: 0x{:08x}", m.virt_base());
            m
        }
        None => {
            println!("ERROR: Failed to map pinctrl");
            syscall::exit(1);
        }
    };

    // Create driver
    let mut pwm = PwmDriver::new(pwm_mmio, pinctrl_mmio);

    // Configure pinmux
    println!();
    println!("=== Configuring Pinmux ===");
    if !pwm.configure_pinmux() {
        println!("WARNING: Pinmux configuration failed, continuing anyway");
    }

    // Initialize PWM
    println!();
    println!("=== Initializing PWM ===");
    if !pwm.init_pwm() {
        println!("ERROR: PWM initialization failed");
        syscall::exit(1);
    }

    // Set initial fan speed to 100% to test
    println!();
    println!("=== Setting Initial Fan Speed ===");
    pwm.set_fan_speed(100);
    println!("  Fan speed set to 100% (for testing)");

    // Read back PWM registers to verify
    let ch_off = pwm.channel_offset(FAN_PWM_CHANNEL);
    println!("  PWM6 PWMCON: 0x{:08x}", pwm.pwm.read32(ch_off + pwm_reg::PWMCON));
    println!("  PWM6 PWMDWIDTH: {}", pwm.pwm.read32(ch_off + pwm_reg::PWMDWIDTH));
    println!("  PWM6 PWMTHRES: {}", pwm.pwm.read32(ch_off + pwm_reg::PWMTHRES));

    // Daemonize
    println!();
    println!("========================================");
    println!("  PWM Fan driver initialized!");
    println!("========================================");

    let result = syscall::daemonize();
    if result == 0 {
        println!("  Daemonized - running as background PWM driver");

        // Main loop - handle IPC requests
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
    } else {
        println!("  Daemonize failed: {}", result);
    }

    syscall::exit(0);
}
