//! WiFi Daemon
//!
//! Connects to pcied to find MT7996 WiFi devices, loads firmware, and initializes them.

#![no_std]
#![no_main]

use userlib::{println, print, syscall};
use pcie::{PcieClient, consts};
use mt7996::{Mt7996Device, mcu::Mcu, firmware};

#[unsafe(no_mangle)]
fn main() {
    println!("=== MT7996 WiFi Driver ===");
    println!();

    // Check for firmware files first
    print!("Checking firmware files... ");
    if firmware::check_firmware_files() {
        println!("OK");
    } else {
        println!("MISSING");
        println!("  Required: mt7996_rom_patch.bin, mt7996_wm.bin, mt7996_wa.bin");
        println!("  Place in: /lib/firmware/mediatek/mt7996/");
        syscall::exit(1);
        return;
    }

    // Connect to pcied
    print!("Connecting to pcied... ");
    let client = match PcieClient::connect() {
        Some(c) => c,
        None => {
            println!("FAILED (is pcied running?)");
            syscall::exit(1);
            return;
        }
    };
    println!("OK");

    // Query for MediaTek WiFi devices
    print!("Querying for MT7996 devices... ");
    let devices = client.find_devices(consts::vendor::MEDIATEK, 0xFFFF);
    println!("found {} MediaTek device(s)", devices.len());

    let mut wifi_found = false;

    for info in devices.iter() {
        // Check if this is an MT7996 family device
        if !is_mt7996_device(info.device_id) {
            continue;
        }

        println!();
        println!("Found MT7996 on pcie{}", info.port);
        println!("  Device: {:04x}:{:04x}", info.vendor_id, info.device_id);
        println!("  BDF: {:02x}:{:02x}.{}", info.bus, info.device, info.function);
        println!("  BAR0: 0x{:016x} ({}KB)", info.bar0_addr, info.bar0_size / 1024);

        // Initialize the MT7996 device
        match Mt7996Device::init_from_info(info) {
            Ok(wifi) => {
                wifi_found = true;
                print_device_info(&wifi);

                // Try to load firmware
                let mut mcu = Mcu::new(&wifi);
                match mcu.load_firmware() {
                    Ok(()) => {
                        println!("MCU state: {:?}", mcu.state());
                    }
                    Err(e) => {
                        println!("Firmware load FAILED: {:?}", e);
                    }
                }
            }
            Err(e) => {
                println!("  Init FAILED: {:?}", e);
            }
        }
    }

    println!();
    if wifi_found {
        println!("=== WiFi device initialized ===");
    } else {
        println!("No MT7996 WiFi device found");
    }

    syscall::exit(0);
}

/// Check if device ID is MT7996 family
fn is_mt7996_device(device_id: u16) -> bool {
    matches!(device_id,
        mt7996::regs::device_id::MT7996 |
        mt7996::regs::device_id::MT7996_2 |
        mt7996::regs::device_id::MT7992 |
        mt7996::regs::device_id::MT7992_2 |
        mt7996::regs::device_id::MT7990 |
        mt7996::regs::device_id::MT7990_2
    )
}

/// Print device information
fn print_device_info(wifi: &Mt7996Device) {
    println!("  BAR0 size: {}KB", wifi.bar0_size() / 1024);
    println!("  BAR0 virt: 0x{:016x}", wifi.bar0_virt());

    // Try raw reads at various offsets
    println!("  Raw[0x000]: 0x{:08x}", wifi.read32_raw(0x000));
    println!("  Raw[0x004]: 0x{:08x}", wifi.read32_raw(0x004));

    // Try some different offsets that might be more interesting
    println!("  Raw[0x100]: 0x{:08x}", wifi.read32_raw(0x100));
    println!("  Raw[0x1000]: 0x{:08x}", wifi.read32_raw(0x1000));
    println!("  Raw[0x10000]: 0x{:08x}", wifi.read32_raw(0x10000));

    // The HIF (Host Interface) base for MT7996 is typically at 0x4000
    println!("  Raw[0x4000]: 0x{:08x}", wifi.read32_raw(0x4000));
    println!("  Raw[0x4004]: 0x{:08x}", wifi.read32_raw(0x4004));

    let variant = wifi.variant();
    println!("  Variant: {:?}", variant);
}
