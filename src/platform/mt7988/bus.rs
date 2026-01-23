//! MT7988A Bus Configuration
//!
//! Registers the buses available on the MT7988A SoC:
//! - 4 PCIe ports (pcie0-pcie3)
//! - 2 USB/xHCI controllers (usb0-usb1)
//! - 1 Platform pseudo-bus

use crate::kernel::bus::{BusType, BusState, with_bus_registry};
use crate::kernel::process::Pid;
use crate::kinfo;

/// Register all buses for MT7988A platform
pub fn register_buses(kernel_pid: Pid) {
    with_bus_registry(|registry| {
        // PCIe ports - MT7988A has 4 PCIe ports
        for i in 0..4u8 {
            if let Some(bus) = registry.add(BusType::PCIe, i) {
                // Start in Resetting - hardware reset happens in complete_init()
                bus.set_initial_state(BusState::Resetting);
                let _ = bus.register_port(kernel_pid);
                kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Resetting");
            }
        }

        // USB controllers - MT7988A has 2 SSUSB controllers
        for i in 0..2u8 {
            if let Some(bus) = registry.add(BusType::Usb, i) {
                // Start in Resetting - hardware reset happens in complete_init()
                bus.set_initial_state(BusState::Resetting);
                let _ = bus.register_port(kernel_pid);
                kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Resetting");
            }
        }

        // Platform pseudo-bus (no hardware reset needed, starts Safe)
        if let Some(bus) = registry.add(BusType::Platform, 0) {
            bus.set_initial_state(BusState::Safe);
            let _ = bus.register_port(kernel_pid);
            kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Safe");
        }
    });
}
