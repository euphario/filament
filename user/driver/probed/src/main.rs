//! probed — Boot-time bus discovery
//!
//! Ephemeral userspace process that creates bus controllers via open(Bus).
//! After probed exits, the kernel permanently locks bus creation.
//!
//! Platform bus tables are compiled in via feature flags. Future enhancement:
//! parse FDT for dynamic discovery.

#![no_std]
#![no_main]

use abi::{bus_type, bus_create_flags, BusCreateInfo, ObjectType};
use userlib::syscall;

#[unsafe(no_mangle)]
fn main() {
    syscall::debug_write(b"probed: starting bus discovery\r\n");

    #[cfg(feature = "platform-qemu-virt")]
    register_qemu_buses();

    #[cfg(feature = "platform-mt7988a")]
    register_mt7988_buses();

    syscall::debug_write(b"probed: bus discovery complete\r\n");
    // exit(0) is called automatically by _start after main returns
}

/// Create a bus and return its handle.
/// Caller can write(handle, BusDevice) to register devices, then close.
fn bus_create(info: &BusCreateInfo) -> Option<userlib::Handle> {
    let params = unsafe {
        core::slice::from_raw_parts(
            info as *const BusCreateInfo as *const u8,
            core::mem::size_of::<BusCreateInfo>(),
        )
    };
    match syscall::open(ObjectType::Bus, params) {
        Ok(handle) => Some(handle),
        Err(_) => {
            syscall::debug_write(b"probed: bus_create failed\r\n");
            None
        }
    }
}

// ============================================================================
// QEMU virt platform
// ============================================================================

#[cfg(feature = "platform-qemu-virt")]
fn register_qemu_buses() {
    // PCIe (ECAM at 0x4010000000)
    bus_create(&BusCreateInfo {
        bus_type: bus_type::PCIE,
        bus_index: 0,
        flags: bus_create_flags::ECAM,
        _pad: 0,
        base_addr: 0x40_1000_0000,
        size: 0x1000_0000,
        irq: 0,
        _reserved: [0; 4],
    });

    // UART (PL011 at 0x09000000)
    bus_create(&BusCreateInfo {
        bus_type: bus_type::UART,
        bus_index: 0,
        flags: 0,
        _pad: 0,
        base_addr: 0x0900_0000,
        size: 0x1000,
        irq: 33, // SPI 1 + 32
        _reserved: [0; 4],
    });

    // Klog
    bus_create(&BusCreateInfo::new(bus_type::KLOG, 0));

    // Platform
    bus_create(&BusCreateInfo::new(bus_type::PLATFORM, 0));
}

// ============================================================================
// MT7988A platform (BPI-R4)
// ============================================================================

#[cfg(feature = "platform-mt7988a")]
fn register_mt7988_buses() {
    // PCIe ports (MAC-based)
    let pcie_bases: [u64; 4] = [
        0x1130_0000, // PCIe0
        0x1131_0000, // PCIe1
        0x1128_0000, // PCIe2
        0x1129_0000, // PCIe3
    ];
    for (i, &base) in pcie_bases.iter().enumerate() {
        bus_create(&BusCreateInfo {
            bus_type: bus_type::PCIE,
            bus_index: i as u8,
            flags: 0,
            _pad: 0,
            base_addr: base,
            size: 0x1_0000,
            irq: 0,
            _reserved: [0; 4],
        });
    }

    // USB controllers
    bus_create(&BusCreateInfo {
        bus_type: bus_type::USB,
        bus_index: 0,
        flags: 0,
        _pad: 0,
        base_addr: 0x1119_0000,
        size: 0x1_0000,
        irq: 205, // SSUSB0
        _reserved: [0; 4],
    });
    bus_create(&BusCreateInfo {
        bus_type: bus_type::USB,
        bus_index: 1,
        flags: 0,
        _pad: 0,
        base_addr: 0x1120_0000,
        size: 0x1_0000,
        irq: 204, // SSUSB1
        _reserved: [0; 4],
    });

    // Ethernet (GMAC)
    bus_create(&BusCreateInfo {
        bus_type: bus_type::ETHERNET,
        bus_index: 0,
        flags: 0,
        _pad: 0,
        base_addr: 0x1510_0000,
        size: 0x1_0000,
        irq: 0,
        _reserved: [0; 4],
    });

    // Platform
    bus_create(&BusCreateInfo::new(bus_type::PLATFORM, 0));

    // UART
    bus_create(&BusCreateInfo {
        bus_type: bus_type::UART,
        bus_index: 0,
        flags: 0,
        _pad: 0,
        base_addr: 0x1100_0000,
        size: 0x1000,
        irq: 155, // SPI 123 + 32
        _reserved: [0; 4],
    });

    // Klog
    bus_create(&BusCreateInfo::new(bus_type::KLOG, 0));
}
