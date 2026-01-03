//! WiFi Daemon
//!
//! Generic WiFi device manager that discovers PCIe WiFi devices and
//! initializes them using the appropriate driver.
//!
//! ## Supported Devices
//!
//! - MediaTek MT7996/MT7992/MT7990 family
//!
//! ## Architecture
//!
//! Drivers implement the WifiDriver trait. Currently drivers are linked
//! directly; in the future they can be separate processes accessed via IPC.

#![no_std]
#![no_main]
#![allow(dead_code)]  // WiFi driver state machine reserved for future use

mod driver;
mod drivers;

use userlib::{println, print, syscall};
use pcie::{PcieClient, consts};
use driver::AnyWifiDriver;

#[unsafe(no_mangle)]
fn main() {
    println!("=== WiFi Daemon ===");
    println!();

    // Connect to pcied
    print!("Connecting to pcied... ");
    let client = match PcieClient::connect() {
        Some(c) => c,
        None => {
            println!("FAILED (is pcied running?)");
            syscall::exit(1);
        }
    };
    println!("OK");

    // Query for WiFi devices (check common WiFi vendors)
    let vendors = [
        (consts::vendor::MEDIATEK, "MediaTek"),
        // Future: Qualcomm, Intel, Realtek, etc.
    ];

    let mut wifi_count = 0;
    let mut used_ports: [Option<u8>; 8] = [None; 8];  // Track ports we touched
    let mut used_port_count = 0usize;

    for (vendor_id, vendor_name) in vendors {
        print!("Scanning {} devices... ", vendor_name);
        let devices = client.find_devices(vendor_id, 0xFFFF);
        println!("found {}", devices.len());

        // Collect all device infos into a fixed-size array for probe_with_peers
        let mut all_devices: [pcie::PcieDeviceInfo; 16] = [Default::default(); 16];
        let mut device_count = 0;
        for info in devices.iter() {
            if device_count < 16 {
                all_devices[device_count] = *info;
                device_count += 1;
            }
        }

        for info in devices.iter() {
            // Debug: print device info BEFORE probing
            println!("  [{:02x}:{:02x}.{}] {:04x}:{:04x} BAR0=0x{:x}/{}KB cmd=0x{:04x}",
                info.bus, info.device, info.function,
                info.vendor_id, info.device_id,
                info.bar0_addr, info.bar0_size / 1024,
                info.command);

            // Try to find a driver for this device (passing all devices so it can find peers like HIF2)
            if let Some(mut driver) = AnyWifiDriver::probe_with_peers(info, &all_devices[..device_count]) {
                wifi_count += 1;
                // Track this port for cleanup
                if used_port_count < 8 {
                    used_ports[used_port_count] = Some(info.port);
                    used_port_count += 1;
                }
                let dev_info = driver.device_info();

                println!();
                println!("=== {} WiFi Device ===", dev_info.chip_name);
                println!("  Driver: {}", driver.driver_name());
                println!("  Device: {:04x}:{:04x}", dev_info.vendor_id, dev_info.device_id);
                println!("  State: {:?}", driver.state());

                // Check firmware availability
                print!("  Firmware: ");
                if driver.firmware_available() {
                    println!("available");

                    // Load firmware
                    print!("  Loading firmware... ");
                    match driver.load_firmware() {
                        Ok(()) => {
                            println!("OK");
                            println!("  State: {:?}", driver.state());

                            // Initialize device
                            print!("  Initializing... ");
                            match driver.init() {
                                Ok(()) => {
                                    println!("OK");
                                    println!("  State: {:?}", driver.state());
                                }
                                Err(e) => {
                                    println!("FAILED: {:?}", e);
                                }
                            }
                        }
                        Err(e) => {
                            println!("FAILED: {:?}", e);
                        }
                    }
                } else {
                    println!("NOT FOUND");
                    println!("  Hint: Place firmware on USB drive or in initrd");
                }

                // Print driver-specific info if available
                print_driver_debug(&driver);
            }
        }
    }

    println!();
    if wifi_count > 0 {
        println!("=== {} WiFi device(s) processed ===", wifi_count);
    } else {
        println!("No supported WiFi devices found");
    }

    syscall::exit(0);
}

/// Print driver-specific debug info
fn print_driver_debug(driver: &AnyWifiDriver) {
    // For MT7996, we can downcast and print extra info
    match driver {
        AnyWifiDriver::Mt7996(_) => {
            // In the future, drivers could expose a debug_info() method
            println!("  (use mt7996-specific tools for detailed info)");
        }
    }
}
