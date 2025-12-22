//! PCIe Host Controller Daemon
//!
//! Initializes PCIe ports, enumerates connected devices, and provides
//! a query service for other drivers to find PCIe devices.
//!
//! ## IPC Protocol
//!
//! Clients connect to port "pcie" and send queries:
//! - FIND_DEVICE (0x01): vendor(2) + device(2) → count + device list
//!
//! Device info format: port(1) + bus(1) + dev(1) + func(1) + vendor(2) +
//!                     device(2) + class(3) + bar0_addr(8) + bar0_size(4)
//!                     = 23 bytes per device

#![no_std]
#![no_main]

use userlib::{println, print, syscall};
use pcie::{
    Board, BpiR4, PcieInit, SocPcie,
    PcieController, PcieDevice,
    consts,
};

/// Maximum devices we can track
const MAX_DEVICES: usize = 32;

/// Command codes
const CMD_FIND_DEVICE: u8 = 0x01;

/// Stored device info for queries
#[derive(Clone, Copy, Default)]
struct StoredDevice {
    port: u8,
    bus: u8,
    device: u8,
    function: u8,
    vendor_id: u16,
    device_id: u16,
    class_code: u32,
    bar0_addr: u64,
    bar0_size: u32,
}

/// Device registry
struct DeviceRegistry {
    devices: [StoredDevice; MAX_DEVICES],
    count: usize,
}

impl DeviceRegistry {
    fn new() -> Self {
        Self {
            devices: [StoredDevice::default(); MAX_DEVICES],
            count: 0,
        }
    }

    fn add(&mut self, dev: StoredDevice) {
        if self.count < MAX_DEVICES {
            self.devices[self.count] = dev;
            self.count += 1;
        }
    }

    fn find(&self, vendor: u16, device: u16) -> impl Iterator<Item = &StoredDevice> {
        self.devices[..self.count].iter().filter(move |d| {
            (vendor == 0xFFFF || d.vendor_id == vendor) &&
            (device == 0xFFFF || d.device_id == device)
        })
    }
}

fn debug_print(s: &str) {
    print!("{} ", s);
}

fn debug_status(val: u32) {
    print!("[{:08x}] ", val);
}

#[unsafe(no_mangle)]
fn main() {
    println!("=== PCIe Host Controller Driver ===");
    println!();

    let mut registry = DeviceRegistry::new();

    // Step 1: Initialize board
    print!("Initializing board... ");
    let board = match BpiR4::new() {
        Ok(b) => b,
        Err(e) => {
            println!("FAILED: {:?}", e);
            syscall::exit(1);
            return;
        }
    };
    println!("OK ({})", board.name());

    // Step 2: Initialize PCIe subsystem via PcieInit helper
    print!("Initializing PCIe... ");
    let mut pcie_init = PcieInit::new(board);
    if let Err(e) = pcie_init.init() {
        println!("FAILED: {:?}", e);
        syscall::exit(1);
        return;
    }

    // Show clock/reset status
    let board = pcie_init.board();
    let (infra0, infra3) = board.soc().clock_status();
    let rst = board.soc().reset_status();
    println!("OK");
    println!("  INFRA0={:08x} INFRA3={:08x} RST={:08x}", infra0, infra3, rst);
    println!();

    // Step 3: Initialize each port and enumerate devices
    let mut any_device_found = false;

    for port in 0..board.soc().port_count() {
        let slot = match board.slot_info(port) {
            Some(s) => s,
            None => continue,
        };

        let port_config = match board.soc().port_config(port) {
            Some(c) => c,
            None => continue,
        };

        print!("[pcie{}] {} ", port, slot.slot_name);

        match PcieController::init(port_config, debug_print, debug_status) {
            Ok(mut controller) => {
                if controller.is_link_up() {
                    let speed = controller.link_speed();
                    let width = controller.link_width();
                    println!("Link up! (Gen{} x{})", speed, width);

                    // Enumerate devices on this port
                    let devices = controller.enumerate();

                    if devices.is_empty() {
                        println!("[pcie{}] No devices found on bus", port);
                    } else {
                        println!("[pcie{}] Found {} device(s):", port, devices.len());
                        for dev in devices.iter() {
                            any_device_found = true;
                            print_device(dev);

                            // Add to registry
                            registry.add(StoredDevice {
                                port,
                                bus: dev.bdf.bus,
                                device: dev.bdf.device,
                                function: dev.bdf.function,
                                vendor_id: dev.id.vendor_id,
                                device_id: dev.id.device_id,
                                class_code: dev.id.class_code,
                                bar0_addr: dev.bar0_addr,
                                bar0_size: dev.bar0_size,
                            });
                        }
                    }
                } else {
                    println!("No link (empty slot?)");
                }
            }
            Err(e) => {
                println!("FAILED: {:?}", e);
            }
        }
        println!();
    }

    if any_device_found {
        println!("=== PCIe enumeration complete ===");
        println!("Registered {} device(s)", registry.count);
    } else {
        println!("No PCIe devices detected");
    }

    // Step 4: Register port and serve queries
    println!();
    print!("Registering pcie port... ");
    let listen_channel = syscall::port_register(b"pcie");
    if listen_channel < 0 {
        if listen_channel == -17 {
            println!("FAILED: PCIe daemon already running");
        } else {
            println!("FAILED: {}", listen_channel);
        }
        syscall::exit(1);
        return;
    }
    println!("OK");

    // Daemonize - detach from parent shell
    let daemon_result = syscall::daemonize();
    if daemon_result == 0 {
        println!("PCIe daemon running");
    }

    // Main loop: handle queries
    let mut buf = [0u8; 128];
    loop {
        // Accept connection
        let channel = syscall::port_accept(listen_channel as u32);
        if channel < 0 {
            syscall::yield_now();
            continue;
        }

        // Handle messages on this channel
        loop {
            let len = syscall::receive(channel as u32, &mut buf);
            if len <= 0 {
                break;  // Client disconnected or error, go back to accept
            }

            let cmd = buf[0];
            match cmd {
                CMD_FIND_DEVICE => {
                    if len >= 5 {
                        let vendor = u16::from_le_bytes([buf[1], buf[2]]);
                        let device = u16::from_le_bytes([buf[3], buf[4]]);

                        // Build response
                        let mut resp = [0u8; 128];
                        let mut offset = 1usize;
                        let mut count = 0u8;

                        for dev in registry.find(vendor, device) {
                            if offset + 23 > resp.len() {
                                break;
                            }

                            resp[offset] = dev.port;
                            resp[offset + 1] = dev.bus;
                            resp[offset + 2] = dev.device;
                            resp[offset + 3] = dev.function;
                            resp[offset + 4..offset + 6].copy_from_slice(&dev.vendor_id.to_le_bytes());
                            resp[offset + 6..offset + 8].copy_from_slice(&dev.device_id.to_le_bytes());
                            resp[offset + 8..offset + 11].copy_from_slice(&dev.class_code.to_le_bytes()[..3]);
                            resp[offset + 11..offset + 19].copy_from_slice(&dev.bar0_addr.to_le_bytes());
                            resp[offset + 19..offset + 23].copy_from_slice(&dev.bar0_size.to_le_bytes());

                            offset += 23;
                            count += 1;
                        }

                        resp[0] = count;
                        syscall::send(channel as u32, &resp[..offset]);
                    }
                }
                _ => {
                    // Unknown command, send empty response
                    syscall::send(channel as u32, &[0u8]);
                }
            }
        }
    }
}

/// Print device information
fn print_device(dev: &PcieDevice) {
    println!("  {:02x}:{:02x}.{} [{:04x}:{:04x}] class {:06x} rev {:02x}",
             dev.bdf.bus, dev.bdf.device, dev.bdf.function,
             dev.id.vendor_id, dev.id.device_id,
             dev.id.class_code, dev.id.revision);

    // Print BAR0 info for endpoints
    if !dev.is_bridge && dev.bar0_size > 0 {
        println!("         BAR0: 0x{:08x} size {}KB",
                 dev.bar0_addr, dev.bar0_size / 1024);
    }

    // Print class description
    println!("         {}", dev.id.class_name());

    // Print vendor name if known
    let vendor_name = match dev.id.vendor_id {
        consts::vendor::MEDIATEK => "MediaTek",
        consts::vendor::INTEL => "Intel",
        consts::vendor::REALTEK => "Realtek",
        consts::vendor::QUALCOMM => "Qualcomm",
        consts::vendor::MICRON => "Micron",
        consts::vendor::SAMSUNG => "Samsung",
        consts::vendor::NVIDIA => "NVIDIA",
        consts::vendor::AMD => "AMD",
        consts::vendor::BROADCOM => "Broadcom",
        consts::vendor::MARVELL => "Marvell",
        _ => return,
    };

    // Print device name if known
    let device_name = match (dev.id.vendor_id, dev.id.device_id) {
        (consts::vendor::MEDIATEK, consts::device::MT7996) => "MT7996 WiFi 7",
        (consts::vendor::MEDIATEK, consts::device::MT7990) => "MT7996 WiFi 7 (Band A)",
        (consts::vendor::MEDIATEK, consts::device::MT7991) => "MT7996 WiFi 7 (Band B)",
        (consts::vendor::MEDIATEK, consts::device::MT7915) => "MT7915 WiFi 6",
        (consts::vendor::MEDIATEK, consts::device::MT7988_RC) => "MT7988 Root Complex",
        _ => {
            println!("         Vendor: {}", vendor_name);
            return;
        }
    };

    println!("         {} {}", vendor_name, device_name);
}
