//! QEMU virt Bus Configuration
//!
//! QEMU virt machine has no PCIe or USB hardware.
//! Only registers the Platform pseudo-bus for software devices.

use crate::kernel::bus::{BusType, BusState, with_bus_registry};
use crate::kernel::process::Pid;
use crate::kinfo;

/// Register buses for QEMU virt platform
pub fn register_buses(kernel_pid: Pid) {
    with_bus_registry(|registry| {
        // Platform pseudo-bus only - no hardware buses on QEMU virt
        if let Some(bus) = registry.add(BusType::Platform, 0) {
            bus.set_initial_state(BusState::Safe);
            let _ = bus.register_port(kernel_pid);
            kinfo!("bus", "registered"; name = bus.port_name_str(), state = "Safe");
        }
    });
}
