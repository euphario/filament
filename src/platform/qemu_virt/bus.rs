//! QEMU virt Bus Configuration
//!
//! Registers buses based on platform config. QEMU virt typically has
//! no PCIe or USB hardware (uses virtio instead).

use crate::kernel::bus::{BusType, BusState, with_bus_registry, bus_config};
use crate::kernel::process::Pid;
use crate::kinfo;

/// Register buses for this platform
pub fn register_buses(kernel_pid: Pid) {
    let config = bus_config();

    with_bus_registry(|registry| {
        // PCIe ports - count from platform config (typically 0 for QEMU)
        let pcie_count = config.pcie_port_count();
        for i in 0..pcie_count {
            if let Some(bus) = registry.add(BusType::PCIe, i as u8) {
                bus.set_initial_state(BusState::Resetting);
                let _ = bus.register_port(kernel_pid);
                kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Resetting");
            }
        }

        // USB controllers - count from platform config (typically 0 for QEMU)
        let usb_count = config.usb_controller_count();
        for i in 0..usb_count {
            if let Some(bus) = registry.add(BusType::Usb, i as u8) {
                bus.set_initial_state(BusState::Resetting);
                let _ = bus.register_port(kernel_pid);
                kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Resetting");
            }
        }

        // Register ECAM PCI host controller if this platform uses ECAM
        if config.is_pcie_ecam_based() {
            if let Some(base) = config.pcie_base(0) {
                crate::kernel::pci::register_ecam_host(base);
            }
        }

        // UART bus (serial console)
        if let Some(bus) = registry.add(BusType::Uart, 0) {
            bus.set_initial_state(BusState::Safe);
            let _ = bus.register_port(kernel_pid);
            kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Safe");
        }

        // Klog bus (kernel log)
        if let Some(bus) = registry.add(BusType::Klog, 0) {
            bus.set_initial_state(BusState::Safe);
            let _ = bus.register_port(kernel_pid);
            kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Safe");
        }

        // Platform pseudo-bus (always present)
        if let Some(bus) = registry.add(BusType::Platform, 0) {
            bus.set_initial_state(BusState::Safe);
            let _ = bus.register_port(kernel_pid);
            kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Safe");
        }
    });
}
