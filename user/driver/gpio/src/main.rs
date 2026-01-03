//! PCA9555 GPIO Expander Driver
//!
//! Userspace driver for the BPI-R4's PCA9555 GPIO expander.
//! Accessed via PCA9545 I2C multiplexer on I2C bus 2.
//!
//! I2C Topology:
//!   I2C2 -> PCA9545@0x70 (channel 3) -> PCA9555@0x20
//!
//! GPIO Assignments (BPI-R4):
//!   Pin 11: USB VBUS enable (active high)
//!   Pin 11: SFP LED
//!   Pins 12-14: SFP controls

#![no_std]
#![no_main]
#![allow(dead_code)]  // Constants/registers for future use

use userlib::{println, syscall};
use userlib::syscall::{Event, event_type, event_flags};
use userlib::ipc::{Server, Connection, IpcError, DevdClient};
use userlib::ipc::protocols::{GpioProtocol, GpioRequest, GpioResponse};
use userlib::ipc::protocols::devd::PropertyList;

/// Polling interval for input change detection (100ms in nanoseconds)
const POLL_INTERVAL_NS: u64 = 100_000_000;

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
pub const GPIO_USB_VBUS: u8 = 11;  // USB 5V power enable (IO1_3)

// Pin name mappings (index = pin number)
const PIN_NAMES: [&str; 16] = [
    "io0_0", "io0_1", "io0_2", "io0_3",       // Port 0: pins 0-3
    "io0_4", "io0_5", "io0_6", "io0_7",       // Port 0: pins 4-7
    "sfp_txfault", "sfp_los", "sfp_present",  // Port 1: pins 8-10 (SFP status)
    "usb_vbus",                                // Port 1: pin 11 (USB power)
    "sfp_txdis", "sfp_rate", "sfp_led",       // Port 1: pins 12-14 (SFP control)
    "io1_7",                                   // Port 1: pin 15 (spare)
];

// Full paths for device tree (index = pin number)
const PIN_PATHS: [&str; 16] = [
    "/bus/i2c2/pca9555@20/io0_0",
    "/bus/i2c2/pca9555@20/io0_1",
    "/bus/i2c2/pca9555@20/io0_2",
    "/bus/i2c2/pca9555@20/io0_3",
    "/bus/i2c2/pca9555@20/io0_4",
    "/bus/i2c2/pca9555@20/io0_5",
    "/bus/i2c2/pca9555@20/io0_6",
    "/bus/i2c2/pca9555@20/io0_7",
    "/bus/i2c2/pca9555@20/sfp_txfault",
    "/bus/i2c2/pca9555@20/sfp_los",
    "/bus/i2c2/pca9555@20/sfp_present",
    "/bus/i2c2/pca9555@20/usb_vbus",
    "/bus/i2c2/pca9555@20/sfp_txdis",
    "/bus/i2c2/pca9555@20/sfp_rate",
    "/bus/i2c2/pca9555@20/sfp_led",
    "/bus/i2c2/pca9555@20/io1_7",
];

// =============================================================================
// GPIO Expander Driver
// =============================================================================

struct GpioExpander {
    i2c_mux_fd: i32,
    i2c_gpio_fd: i32,
    output_state: [u8; 2],
    config_state: [u8; 2],
    /// Cached input state for change detection
    input_state: [u8; 2],
}

impl GpioExpander {
    fn new() -> Option<Self> {
        // Open PCA9545 multiplexer on I2C2 @ 0x70
        let mux_fd = syscall::scheme_open("i2c:2/70", 2);
        if mux_fd < 0 {
            println!("  Failed to open I2C mux: {}", mux_fd);
            return None;
        }
        println!("  PCA9545 mux opened (fd={})", mux_fd);

        // Open PCA9555 expander on I2C2 @ 0x20
        let gpio_fd = syscall::scheme_open("i2c:2/20", 2);
        if gpio_fd < 0 {
            println!("  Failed to open I2C GPIO: {}", gpio_fd);
            syscall::close(mux_fd as u32);
            return None;
        }
        println!("  PCA9555 GPIO opened (fd={})", gpio_fd);

        Some(Self {
            i2c_mux_fd: mux_fd,
            i2c_gpio_fd: gpio_fd,
            output_state: [0xFF, 0xFF],
            config_state: [0xFF, 0xFF],
            input_state: [0xFF, 0xFF],
        })
    }

    fn select_mux_channel(&self) -> bool {
        let data = [MUX_CHANNEL_3];
        let result = syscall::write(self.i2c_mux_fd as u32, &data);
        if result < 0 {
            println!("  Mux select failed: {}", result);
            return false;
        }
        true
    }

    fn write_reg(&self, reg: u8, value: u8) -> bool {
        if !self.select_mux_channel() {
            return false;
        }
        let data = [reg, value];
        syscall::write(self.i2c_gpio_fd as u32, &data) >= 0
    }

    fn read_reg(&self, reg: u8) -> Option<u8> {
        if !self.select_mux_channel() {
            return None;
        }
        let addr = [reg];
        if syscall::write(self.i2c_gpio_fd as u32, &addr) < 0 {
            return None;
        }
        let mut buf = [0u8];
        if syscall::read(self.i2c_gpio_fd as u32, &mut buf) < 0 {
            return None;
        }
        Some(buf[0])
    }

    fn init(&mut self) -> bool {
        println!("  Initializing PCA9555...");

        if !self.select_mux_channel() {
            println!("    Failed to select mux channel");
            return false;
        }
        println!("    Mux channel 3 selected");

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

        // Read initial input state
        if let Some(in0) = self.read_reg(pca9555::INPUT_PORT0) {
            self.input_state[0] = in0;
            println!("    Input port 0: 0x{:02x}", in0);
        }
        if let Some(in1) = self.read_reg(pca9555::INPUT_PORT1) {
            self.input_state[1] = in1;
            println!("    Input port 1: 0x{:02x}", in1);
        }

        println!("    PCA9555 initialized");
        true
    }

    /// Read current input state (both ports)
    fn read_inputs(&self) -> Option<[u8; 2]> {
        let in0 = self.read_reg(pca9555::INPUT_PORT0)?;
        let in1 = self.read_reg(pca9555::INPUT_PORT1)?;
        Some([in0, in1])
    }

    /// Poll for input changes and notify devd of any changes
    /// Returns the number of pins that changed
    fn poll_changes(&mut self, devd: &mut Option<DevdClient>) -> usize {
        let current = match self.read_inputs() {
            Some(v) => v,
            None => return 0,
        };

        let mut changed_count = 0;

        // Check each port for changes
        for port in 0..2 {
            let old = self.input_state[port];
            let new = current[port];
            let changed = old ^ new;

            if changed != 0 {
                // Find which bits changed
                for bit in 0..8 {
                    if (changed >> bit) & 1 != 0 {
                        let pin = (port * 8 + bit) as u8;
                        // Only notify for input pins (config bit = 1)
                        if (self.config_state[port] >> bit) & 1 != 0 {
                            changed_count += 1;
                            // Update devd if connected
                            if let Some(client) = devd {
                                self.update_pin_in_devd(client, pin);
                            }
                        }
                    }
                }
            }
        }

        // Update cached state
        self.input_state = current;
        changed_count
    }

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

    fn read_pin(&self, pin: u8) -> Option<bool> {
        if pin >= 16 {
            return None;
        }

        let port = pin / 8;
        let bit = pin % 8;
        let input_reg = if port == 0 { pca9555::INPUT_PORT0 } else { pca9555::INPUT_PORT1 };

        self.read_reg(input_reg).map(|v| (v >> bit) & 1 != 0)
    }

    /// Handle a GPIO request
    fn handle_request(&mut self, request: &GpioRequest, devd: &mut Option<DevdClient>) -> GpioResponse {
        match request {
            GpioRequest::SetPin { pin, high } => {
                if self.set_output(*pin, *high) {
                    // Update devd with new state
                    if let Some(client) = devd {
                        self.update_pin_in_devd(client, *pin);
                    }
                    GpioResponse::Ok
                } else {
                    GpioResponse::Error(-1)
                }
            }
            GpioRequest::GetPin { pin } => {
                match self.read_pin(*pin) {
                    Some(value) => GpioResponse::PinValue(value),
                    None => GpioResponse::Error(-1),
                }
            }
        }
    }

    /// Register all GPIO pins with devd
    fn register_with_devd(&self, devd: &mut DevdClient) {
        for pin in 0u8..16 {
            let port = pin / 8;
            let bit = pin % 8;
            let is_output = (self.config_state[port as usize] >> bit) & 1 == 0;
            let value = if is_output {
                (self.output_state[port as usize] >> bit) & 1 != 0
            } else {
                self.read_pin(pin).unwrap_or(false)
            };

            let path = PIN_PATHS[pin as usize];
            let props = PropertyList::new()
                .add("type", "gpio")
                .add("dir", if is_output { "out" } else { "in" })
                .add("val", if value { "1" } else { "0" })
                .add("name", PIN_NAMES[pin as usize]);

            if let Err(e) = devd.register(path, props) {
                println!("  Failed to register {} with devd: {:?}", path, e);
            }
        }
    }

    /// Update a single pin's state in devd
    fn update_pin_in_devd(&self, devd: &mut DevdClient, pin: u8) {
        if pin >= 16 {
            return;
        }

        let port = pin / 8;
        let bit = pin % 8;
        let is_output = (self.config_state[port as usize] >> bit) & 1 == 0;
        let value = if is_output {
            (self.output_state[port as usize] >> bit) & 1 != 0
        } else {
            self.read_pin(pin).unwrap_or(false)
        };

        let path = PIN_PATHS[pin as usize];
        let props = PropertyList::new()
            .add("type", "gpio")
            .add("dir", if is_output { "out" } else { "in" })
            .add("val", if value { "1" } else { "0" })
            .add("name", PIN_NAMES[pin as usize]);

        let _ = devd.update(path, props);
    }
}

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

    // Create and initialize GPIO expander
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

    // Enable USB VBUS by default (pin 11)
    println!();
    println!("=== Enabling USB Power ===");
    if gpio.set_output(GPIO_USB_VBUS, true) {
        println!("  USB VBUS enabled (pin {})", GPIO_USB_VBUS);
    } else {
        println!("  WARNING: Failed to enable USB VBUS");
    }

    // Register GPIO server
    println!();
    let server = match Server::<GpioProtocol>::register() {
        Ok(s) => {
            println!("  Registered 'gpio' port");
            s
        }
        Err(IpcError::ResourceBusy) => {
            println!("ERROR: GPIO driver already running");
            syscall::exit(1);
        }
        Err(e) => {
            println!("ERROR: Failed to register GPIO port: {:?}", e);
            syscall::exit(1);
        }
    };

    // Connect to devd and register pins
    println!();
    println!("=== Registering with devd ===");
    let mut devd_client: Option<DevdClient> = match DevdClient::connect() {
        Ok(mut client) => {
            println!("  Connected to devd");
            gpio.register_with_devd(&mut client);
            println!("  Registered {} GPIO pins", 16);
            Some(client)
        }
        Err(e) => {
            println!("  WARNING: Failed to connect to devd: {:?}", e);
            println!("  GPIO pins will not be visible in device tree");
            None
        }
    };

    // Subscribe to events
    println!();
    println!("=== Setting up event handling ===");

    // Subscribe to IPC events for GPIO port
    let gpio_channel = server.listen_channel();
    let result = syscall::event_subscribe(event_type::IPC_READY, gpio_channel as u64);
    if result != 0 {
        println!("ERROR: Failed to subscribe to IPC events: {}", result);
        syscall::exit(1);
    }
    println!("  IpcReady(gpio:{}): OK", gpio_channel);

    // Subscribe to timer events for polling
    let result = syscall::event_subscribe(event_type::TIMER, 0);
    if result != 0 {
        println!("ERROR: Failed to subscribe to Timer events: {}", result);
        syscall::exit(1);
    }
    println!("  Timer: OK");

    // Set initial polling timer
    syscall::timer_set(POLL_INTERVAL_NS);

    println!();
    println!("========================================");
    println!("  GPIO driver ready (polling every 100ms)");
    println!("========================================");

    // Event-driven main loop
    let mut event = Event::empty();

    loop {
        let result = syscall::event_wait(&mut event, event_flags::BLOCKING);

        if result == -11 {
            // EAGAIN - yield and retry
            syscall::yield_now();
            continue;
        }

        if result <= 0 {
            syscall::yield_now();
            continue;
        }

        match event.event_type {
            et if et == event_type::IPC_READY => {
                let channel = event.data as u32;
                if channel == gpio_channel {
                    // Accept connection on GPIO port
                    let mut conn: Connection<GpioProtocol> = match server.accept() {
                        Ok(c) => c,
                        Err(_) => continue,
                    };

                    // Handle request
                    match conn.receive() {
                        Ok(request) => {
                            let response = gpio.handle_request(&request, &mut devd_client);
                            let _ = conn.send(&response);
                        }
                        Err(_) => {}
                    }
                    // Connection dropped here
                }
            }

            et if et == event_type::TIMER => {
                // Poll for input changes
                let changed = gpio.poll_changes(&mut devd_client);
                if changed > 0 {
                    println!("[gpio] {} input pin(s) changed", changed);
                }

                // Re-arm timer for next poll
                syscall::timer_set(POLL_INTERVAL_NS);
            }

            _ => {}
        }
    }
}
