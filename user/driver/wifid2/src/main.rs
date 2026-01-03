//! WiFi Daemon v2 - Uses mt7996_v2 (EXACT Linux Translation)
//!
//! This daemon implements the EXACT initialization sequence
//! from Linux mt76/mt7996 driver for firmware loading.
//!
//! ## Key Difference from wifid
//!
//! Uses mt7996_v2 library which is a line-by-line translation of Linux,
//! rather than a cleaned-up interpretation.

#![no_std]
#![no_main]

use userlib::{println, print, syscall};
use pcie::{PcieClient, consts};

/// MT7996 device IDs
const DEVICE_MT7996_HIF1: u16 = 0x7990;
const DEVICE_MT7996_HIF2: u16 = 0x7991;

#[unsafe(no_mangle)]
fn main() {
    println!("=== WiFi Daemon v2 ===");
    println!("Using mt7996_v2 (exact Linux translation)");
    println!();

    // Step 1: Connect to pcied
    print!("Connecting to pcied... ");
    let client = match PcieClient::connect() {
        Some(c) => c,
        None => {
            println!("FAILED (is pcied running?)");
            syscall::exit(1);
        }
    };
    println!("OK");

    // Step 2: Find MT7996 devices
    print!("Scanning for MediaTek devices... ");
    let devices = client.find_devices(consts::vendor::MEDIATEK, 0xFFFF);
    println!("found {}", devices.len());

    // Find HIF1 (main device) and HIF2 (secondary)
    let mut hif1_info: Option<pcie::PcieDeviceInfo> = None;
    let mut hif2_info: Option<pcie::PcieDeviceInfo> = None;

    for info in devices.iter() {
        println!("  [{:02x}:{:02x}.{}] {:04x}:{:04x} BAR0=0x{:x}/{}KB cmd=0x{:04x}",
            info.bus, info.device, info.function,
            info.vendor_id, info.device_id,
            info.bar0_addr, info.bar0_size / 1024,
            info.command);

        match info.device_id {
            DEVICE_MT7996_HIF1 => hif1_info = Some(*info),
            DEVICE_MT7996_HIF2 => hif2_info = Some(*info),
            _ => {}
        }
    }

    // Must have HIF1 (main interface)
    let hif1 = match hif1_info {
        Some(info) => info,
        None => {
            println!("\nNo MT7996 HIF1 device found!");
            syscall::exit(1);
        }
    };

    println!();
    println!("=== MT7996 WiFi Device ===");
    println!("  HIF1: {:04x}:{:04x} @ 0x{:x}", hif1.vendor_id, hif1.device_id, hif1.bar0_addr);
    if let Some(ref hif2) = hif2_info {
        println!("  HIF2: {:04x}:{:04x} @ 0x{:x}", hif2.vendor_id, hif2.device_id, hif2.bar0_addr);
    }

    println!();
    println!("=== Device Discovery Complete ===");
    println!("(Full initialization not yet implemented in v2)");

    // Clean up: Reset port before exit
    print!("\nResetting PCIe port {}... ", hif1.port);
    if client.reset_port(hif1.port) {
        println!("OK");
    } else {
        println!("FAILED");
    }

    syscall::exit(0);
}
