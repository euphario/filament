//! PCA9555 GPIO Expander Driver
//!
//! Userspace driver for the BPI-R4's PCA9555 GPIO expander.
//! Accessed via PCA9545 I2C multiplexer on I2C bus 0.
//!
//! I2C Topology:
//!   I2C0 -> PCA9545@0x70 (channel 3) -> PCA9555@0x20
//!
//! GPIO Assignments (BPI-R4):
//!   Pin 9:  USB VBUS enable (active high)
//!   Pin 11: SFP LED
//!   Pins 12-14: SFP controls

#![no_std]
#![no_main]

use userlib::{println, print, logln, flush_log, syscall};

// =============================================================================
// I2C Addresses
// =============================================================================

const PCA9545_ADDR: u8 = 0x70;  // I2C multiplexer
const PCA9555_ADDR: u8 = 0x20;  // GPIO expander

// PCA9545 channel selection
const MUX_CHANNEL_3: u8 = 0x08;  // Enable channel 3 only

// PCA9555 registers
mod pca9555 {
    pub const INPUT_PORT0: u8 = 0x00;
    pub const INPUT_PORT1: u8 = 0x01;
    pub const OUTPUT_PORT0: u8 = 0x02;
    pub const OUTPUT_PORT1: u8 = 0x03;
    pub const POLARITY_PORT0: u8 = 0x04;
    pub const POLARITY_PORT1: u8 = 0x05;
    pub const CONFIG_PORT0: u8 = 0x06;
    pub const CONFIG_PORT1: u8 = 0x07;
}

// GPIO pin definitions for BPI-R4
// PCA_A_IO1_3/VBUS_AB_PDN = Port 1, bit 3 = pin 11
// Active LOW for power-down, so HIGH enables VBUS
pub const GPIO_USB_VBUS: u8 = 11;  // USB 5V power enable (IO1_3)

// =============================================================================
// GPIO Expander Driver
// =============================================================================

struct GpioExpander {
    i2c_mux_fd: i32,   // FD for PCA9545 multiplexer
    i2c_gpio_fd: i32,  // FD for PCA9555 expander
    output_state: [u8; 2],  // Current output state (port 0 and 1)
    config_state: [u8; 2],  // Current config state (0=output, 1=input)
}

impl GpioExpander {
    fn new() -> Option<Self> {
        // Open I2C devices via i2c scheme
        // Format: i2c:bus/addr
        // Use O_RDWR (2) for read/write access

        // Open PCA9545 multiplexer on I2C2 @ 0x70
        // (BPI-R4 has PCA9545 on i2c2, not i2c0)
        let mux_fd = syscall::scheme_open("i2c:2/70", 2);  // O_RDWR
        if mux_fd < 0 {
            println!("  Failed to open I2C mux: {}", mux_fd);
            return None;
        }
        println!("  PCA9545 mux opened (fd={})", mux_fd);

        // Open PCA9555 expander on I2C2 @ 0x20
        // Note: The kernel I2C scheme doesn't know about the mux,
        // so we access via the same bus but different address
        let gpio_fd = syscall::scheme_open("i2c:2/20", 2);  // O_RDWR
        if gpio_fd < 0 {
            println!("  Failed to open I2C GPIO: {}", gpio_fd);
            syscall::close(mux_fd as u32);
            return None;
        }
        println!("  PCA9555 GPIO opened (fd={})", gpio_fd);

        Some(Self {
            i2c_mux_fd: mux_fd,
            i2c_gpio_fd: gpio_fd,
            output_state: [0xFF, 0xFF],  // Default all high
            config_state: [0xFF, 0xFF],  // Default all inputs
        })
    }

    /// Select the multiplexer channel for PCA9555 access
    fn select_mux_channel(&self) -> bool {
        // Write channel selection to PCA9545
        let data = [MUX_CHANNEL_3];
        let result = syscall::write(self.i2c_mux_fd as u32, &data);
        if result < 0 {
            println!("  Mux select failed: {}", result);
            return false;
        }
        true
    }

    /// Write to PCA9555 register
    fn write_reg(&self, reg: u8, value: u8) -> bool {
        // Must select mux channel first
        if !self.select_mux_channel() {
            return false;
        }

        // Write register address and value
        let data = [reg, value];
        let result = syscall::write(self.i2c_gpio_fd as u32, &data);
        result >= 0
    }

    /// Read from PCA9555 register
    fn read_reg(&self, reg: u8) -> Option<u8> {
        // Must select mux channel first
        if !self.select_mux_channel() {
            return None;
        }

        // Write register address
        let addr = [reg];
        if syscall::write(self.i2c_gpio_fd as u32, &addr) < 0 {
            return None;
        }

        // Read value
        let mut buf = [0u8];
        if syscall::read(self.i2c_gpio_fd as u32, &mut buf) < 0 {
            return None;
        }

        Some(buf[0])
    }

    /// Initialize the GPIO expander
    fn init(&mut self) -> bool {
        println!("  Initializing PCA9555...");

        // Select mux channel
        if !self.select_mux_channel() {
            println!("    Failed to select mux channel");
            return false;
        }
        println!("    Mux channel 3 selected");

        // Read current state
        if let Some(out0) = self.read_reg(pca9555::OUTPUT_PORT0) {
            self.output_state[0] = out0;
            println!("    Output port 0: 0x{:02x}", out0);
        } else {
            println!("    Failed to read output port 0");
            return false;
        }

        if let Some(out1) = self.read_reg(pca9555::OUTPUT_PORT1) {
            self.output_state[1] = out1;
            println!("    Output port 1: 0x{:02x}", out1);
        }

        if let Some(cfg0) = self.read_reg(pca9555::CONFIG_PORT0) {
            self.config_state[0] = cfg0;
            println!("    Config port 0: 0x{:02x}", cfg0);
        }

        if let Some(cfg1) = self.read_reg(pca9555::CONFIG_PORT1) {
            self.config_state[1] = cfg1;
            println!("    Config port 1: 0x{:02x}", cfg1);
        }

        println!("    PCA9555 initialized");
        true
    }

    /// Set a GPIO pin as output with given value
    fn set_output(&mut self, pin: u8, high: bool) -> bool {
        if pin >= 16 {
            return false;
        }

        let port = (pin / 8) as usize;
        let bit = pin % 8;
        let mask = 1u8 << bit;

        // Update config to output (0 = output)
        self.config_state[port] &= !mask;

        // Update output state
        if high {
            self.output_state[port] |= mask;
        } else {
            self.output_state[port] &= !mask;
        }

        // Write config register
        let config_reg = if port == 0 { pca9555::CONFIG_PORT0 } else { pca9555::CONFIG_PORT1 };
        if !self.write_reg(config_reg, self.config_state[port]) {
            return false;
        }

        // Write output register
        let output_reg = if port == 0 { pca9555::OUTPUT_PORT0 } else { pca9555::OUTPUT_PORT1 };
        self.write_reg(output_reg, self.output_state[port])
    }

    /// Read a GPIO pin value
    fn read_pin(&self, pin: u8) -> Option<bool> {
        if pin >= 16 {
            return None;
        }

        let port = pin / 8;
        let bit = pin % 8;
        let input_reg = if port == 0 { pca9555::INPUT_PORT0 } else { pca9555::INPUT_PORT1 };

        self.read_reg(input_reg).map(|v| (v >> bit) & 1 != 0)
    }

    /// Enable USB VBUS power
    fn enable_usb_vbus(&mut self) -> bool {
        println!("  Enabling USB VBUS (pin {})...", GPIO_USB_VBUS);
        if self.set_output(GPIO_USB_VBUS, true) {
            println!("    USB VBUS enabled");
            true
        } else {
            println!("    Failed to enable USB VBUS");
            false
        }
    }

    /// Disable USB VBUS power
    fn disable_usb_vbus(&mut self) -> bool {
        println!("  Disabling USB VBUS (pin {})...", GPIO_USB_VBUS);
        self.set_output(GPIO_USB_VBUS, false)
    }
}

// =============================================================================
// IPC Message Protocol
// =============================================================================

// GPIO IPC commands
const GPIO_CMD_SET_PIN: u8 = 1;    // Set pin high/low: [1, pin, value]
const GPIO_CMD_GET_PIN: u8 = 2;    // Get pin value: [2, pin] -> [value]
const GPIO_CMD_USB_VBUS: u8 = 3;   // Control USB VBUS: [3, enable]

// =============================================================================
// Main
// =============================================================================

#[unsafe(no_mangle)]
fn main() {
    println!("========================================");
    println!("  GPIO Expander Driver (PCA9555)");
    println!("  BPI-R4 I2C GPIO Expander");
    println!("========================================");
    println!();

    // Register the "gpio" port
    let port_result = syscall::port_register(b"gpio");
    if port_result < 0 {
        if port_result == -17 {  // EEXIST
            println!("ERROR: GPIO driver already running");
        } else {
            println!("ERROR: Failed to register GPIO port: {}", port_result);
        }
        syscall::exit(1);
    }
    let listen_channel = port_result as u32;
    println!("  Registered 'gpio' port (channel {})", listen_channel);

    // Create and initialize GPIO expander
    println!();
    println!("=== I2C Initialization ===");
    let mut gpio = match GpioExpander::new() {
        Some(g) => g,
        None => {
            println!("ERROR: Failed to open I2C devices");
            syscall::exit(1);
        }
    };

    println!();
    println!("=== PCA9555 Initialization ===");
    if !gpio.init() {
        println!("ERROR: Failed to initialize PCA9555");
        syscall::exit(1);
    }

    // Enable USB VBUS by default
    println!();
    println!("=== Enabling USB Power ===");
    if !gpio.enable_usb_vbus() {
        println!("WARNING: Failed to enable USB VBUS");
    }

    // Daemonize
    println!();
    println!("========================================");
    println!("  GPIO driver initialized!");
    println!("========================================");

    let result = syscall::daemonize();
    if result == 0 {
        println!("  Daemonized - running as background GPIO driver");

        // Main loop - handle IPC requests
        let mut msg_buf = [0u8; 16];
        loop {
            // Try to accept connections on the listen channel
            let client = syscall::port_accept(listen_channel);
            if client >= 0 {
                let client_ch = client as u32;

                // Read request
                let len = syscall::receive(client_ch, &mut msg_buf);
                if len > 0 {
                    let cmd = msg_buf[0];
                    let response = match cmd {
                        GPIO_CMD_SET_PIN if len >= 3 => {
                            let pin = msg_buf[1];
                            let value = msg_buf[2] != 0;
                            if gpio.set_output(pin, value) { 0i8 } else { -1i8 }
                        }
                        GPIO_CMD_GET_PIN if len >= 2 => {
                            let pin = msg_buf[1];
                            match gpio.read_pin(pin) {
                                Some(true) => 1i8,
                                Some(false) => 0i8,
                                None => -1i8,
                            }
                        }
                        GPIO_CMD_USB_VBUS if len >= 2 => {
                            let enable = msg_buf[1] != 0;
                            let ok = if enable {
                                gpio.enable_usb_vbus()
                            } else {
                                gpio.disable_usb_vbus()
                            };
                            if ok { 0i8 } else { -1i8 }
                        }
                        _ => -22i8,  // EINVAL
                    };

                    // Send response
                    let resp = [response as u8];
                    let _ = syscall::send(client_ch, &resp);
                }

                syscall::channel_close(client_ch);
            }

            flush_log();
            syscall::yield_now();
        }
    } else {
        println!("  Daemonize failed: {}", result);
    }

    syscall::exit(0);
}
