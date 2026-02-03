//! MT7988A Bus Configuration
//!
//! Registers the buses available on the MT7988A SoC.
//! Bus counts come from the platform config (selected at boot from DTB).

use crate::kernel::bus::{BusType, BusState, with_bus_registry, bus_config};
use crate::kernel::process::Pid;
use crate::kinfo;

/// Register all buses for this platform
pub fn register_buses(kernel_pid: Pid) {
    let config = bus_config();

    with_bus_registry(|registry| {
        // PCIe ports - count from platform config
        let pcie_count = config.pcie_port_count();
        for i in 0..pcie_count {
            if let Some(bus) = registry.add(BusType::PCIe, i as u8) {
                // Start in Resetting - hardware reset happens in complete_init()
                bus.set_initial_state(BusState::Resetting);
                let _ = bus.register_port(kernel_pid);
                kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Resetting");
            }
        }

        // USB controllers - count from platform config
        let usb_count = config.usb_controller_count();
        for i in 0..usb_count {
            if let Some(bus) = registry.add(BusType::Usb, i as u8) {
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

        // Ethernet (GMAC1 + internal switch, no reset needed, starts Safe)
        if let Some(bus) = registry.add(BusType::Ethernet, 0) {
            bus.set_initial_state(BusState::Safe);
            let _ = bus.register_port(kernel_pid);
            kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Safe");
        }
    });
}
