//! PCIe Host Controller Daemon
//!
//! Initializes PCIe ports, enumerates connected devices, and provides
//! a query service for other drivers to find PCIe devices.
//!
//! Uses type-safe IPC via `Server<PcieProtocol>`.

#![no_std]
#![no_main]
#![allow(dead_code)]  // IPC commands reserved for future use
#![allow(unreachable_code)]  // Exit points for clarity

use userlib::syscall;
use userlib::{uinfo, uerror};
use userlib::ipc::{Server, Connection, IpcError};
use userlib::ipc::protocols::{
    PcieProtocol, PcieRequest, PcieResponse,
    PcieDeviceInfo, DeviceList,
    DevdClient, PropertyList,
};
use pcie::{
    Board, BpiR4, PcieInit, SocPcie,
    PcieController,
};

/// Maximum devices we can track (internal registry)
const MAX_DEVICES: usize = 32;

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
    command: u16,
}

impl StoredDevice {
    /// Convert to protocol device info
    fn to_info(&self) -> PcieDeviceInfo {
        PcieDeviceInfo {
            port: self.port,
            bus: self.bus,
            device: self.device,
            function: self.function,
            vendor_id: self.vendor_id,
            device_id: self.device_id,
            class_code: self.class_code,
            bar0_addr: self.bar0_addr,
            bar0_size: self.bar0_size,
            command: self.command,
        }
    }
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

    /// Find a device by port/bus/device/function
    fn find_by_bdf(&self, port: u8, bus: u8, device: u8, function: u8) -> Option<&StoredDevice> {
        self.devices[..self.count].iter().find(|d| {
            d.port == port && d.bus == bus && d.device == device && d.function == function
        })
    }

    /// Handle a FindDevice request, returning device list and ports queried
    fn find_devices(&self, vendor: u16, device: u16) -> (DeviceList, [Option<u8>; 4]) {
        let mut list = DeviceList::new();
        let mut ports: [Option<u8>; 4] = [None; 4];
        let mut port_count = 0usize;

        for dev in self.find(vendor, device) {
            list.push(dev.to_info());

            // Track port for auto-reset on client disconnect
            let port_already_tracked = ports.iter()
                .take(port_count)
                .any(|p| *p == Some(dev.port));
            if !port_already_tracked && port_count < 4 {
                ports[port_count] = Some(dev.port);
                port_count += 1;
            }
        }

        (list, ports)
    }
}

fn debug_print(_s: &str) {}
fn debug_status(_val: u32) {}

/// Format a u16 as hex string
fn hex_u16(val: u16, buf: &mut [u8; 4]) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    buf[0] = HEX[((val >> 12) & 0xf) as usize];
    buf[1] = HEX[((val >> 8) & 0xf) as usize];
    buf[2] = HEX[((val >> 4) & 0xf) as usize];
    buf[3] = HEX[(val & 0xf) as usize];
}

/// Map PCI class code to device class name (matches devd's DeviceClass)
fn class_name(class_code: u32) -> &'static str {
    // Use upper 16 bits (class + subclass)
    let class = (class_code >> 8) & 0xFFFF;
    match class {
        0x0200 => "network",       // Network controller
        0x0280 => "network",       // Network controller (other)
        0x0100..=0x010F => "storage", // Mass storage
        0x0C03 => "usb",           // USB controller
        0x0300..=0x030F => "display", // Display controller
        0x0400..=0x040F => "multimedia", // Multimedia
        0x0600..=0x060F => "bridge", // Bridge device
        0x0700 => "serial",        // Serial controller
        0x0900 => "input",         // Input device
        _ => "other",
    }
}

/// Register all devices with devd and claim the buses we manage
fn register_with_devd(registry: &DeviceRegistry, _active_ports: u8) {
    let mut devd = match DevdClient::connect() {
        Ok(d) => d,
        Err(_) => {
            uerror!("pcied", "devd_connect_failed");
            return;
        }
    };

    // Claim all PCIe buses
    let mut buses_claimed = 0usize;
    for port in 0..4u8 {
        let mut bus_path = [0u8; 12];
        bus_path[..9].copy_from_slice(b"/bus/pcie");
        bus_path[9] = b'0' + port;
        let path = core::str::from_utf8(&bus_path[..10]).unwrap_or("");

        if devd.claim_bus(path).is_ok() {
            buses_claimed += 1;
        }
    }

    // Register discovered devices
    let mut registered = 0usize;

    for i in 0..registry.count {
        let dev = &registry.devices[i];

        // Build path: /bus/pcie{port}/{bdf}
        let mut path_buf = [0u8; 32];
        let path = {
            let port = dev.port;
            let bus = dev.bus;
            let device = dev.device;
            let function = dev.function;

            const HEX: &[u8; 16] = b"0123456789abcdef";
            let mut i = 0;

            path_buf[i..i+9].copy_from_slice(b"/bus/pcie");
            i += 9;
            path_buf[i] = b'0' + port;
            i += 1;
            path_buf[i] = b'/';
            i += 1;
            path_buf[i] = HEX[(bus >> 4) as usize];
            i += 1;
            path_buf[i] = HEX[(bus & 0xf) as usize];
            i += 1;
            path_buf[i] = b':';
            i += 1;
            path_buf[i] = HEX[(device >> 4) as usize];
            i += 1;
            path_buf[i] = HEX[(device & 0xf) as usize];
            i += 1;
            path_buf[i] = b'.';
            i += 1;
            path_buf[i] = HEX[(function & 0xf) as usize];
            i += 1;

            core::str::from_utf8(&path_buf[..i]).unwrap_or("")
        };

        let mut vendor_buf = [0u8; 4];
        let mut device_buf = [0u8; 4];

        hex_u16(dev.vendor_id, &mut vendor_buf);
        hex_u16(dev.device_id, &mut device_buf);

        let vendor_str = core::str::from_utf8(&vendor_buf).unwrap_or("0000");
        let device_str = core::str::from_utf8(&device_buf).unwrap_or("0000");

        let props = PropertyList::new()
            .add("bus", "pcie")
            .add("vendor", vendor_str)
            .add("device", device_str)
            .add("class", class_name(dev.class_code))
            .add("state", "discovered");

        if devd.register(path, props).is_ok() {
            registered += 1;
        }
    }

    uinfo!("pcied", "devd_registered"; buses = buses_claimed, devices = registered);
}

#[unsafe(no_mangle)]
fn main() {
    // Disable all stdout to not corrupt consoled's display
    userlib::io::disable_stdout();
    userlib::ulog::disable_stdout();

    userlib::ulog::init();
    uinfo!("pcied", "init_start");

    let mut registry = DeviceRegistry::new();

    // Initialize board
    let board = match BpiR4::new() {
        Ok(b) => b,
        Err(_e) => {
            uerror!("pcied", "board_init_failed");
            syscall::exit(1);
            return;
        }
    };

    // Initialize PCIe subsystem
    let mut pcie_init = PcieInit::new(board);
    if let Err(_e) = pcie_init.init() {
        uerror!("pcied", "pcie_init_failed");
        syscall::exit(1);
        return;
    }
    let board = pcie_init.board();

    // Discover PCIe buses from kernel
    let mut buses = [syscall::BusInfo::empty(); 16];
    let bus_count = syscall::bus_list(&mut buses);
    if bus_count < 0 {
        uerror!("pcied", "bus_list_failed"; err = bus_count);
        syscall::exit(1);
        return;
    }

    // Find which PCIe ports are available
    let mut available_ports = 0u8;
    for i in 0..(bus_count as usize) {
        let info = &buses[i];
        if info.bus_type == syscall::bus_type::PCIE {
            let port = info.bus_index;
            if (port as usize) < 4 {
                available_ports |= 1 << port;
            }
        }
    }

    if available_ports == 0 {
        uerror!("pcied", "no_pcie_ports");
        syscall::exit(1);
        return;
    }

    // Step 4: Initialize each available port and enumerate devices
    // devd has authorized us via SetDriver - we trust our spawn context
    let mut any_device_found = false;
    let mut active_ports: u8 = 0;  // Bitmask of ports we successfully enumerated

    for port in 0..board.soc().port_count() {
        // Skip ports not in kernel's bus list
        if (available_ports & (1 << port)) == 0 {
            continue;
        }

        let slot = match board.slot_info(port) {
            Some(s) => s,
            None => continue,
        };

        let port_config = match board.soc().port_config(port) {
            Some(c) => c,
            None => continue,
        };

        match PcieController::init(port_config, debug_print, debug_status) {
            Ok(mut controller) => {
                if controller.is_link_up() {
                    let speed = controller.link_speed();
                    let width = controller.link_width();
                    uinfo!("pcied", "link_up"; port = port, speed = speed, width = width);

                    active_ports |= 1 << port;
                    let devices = controller.enumerate();

                    for dev in devices.iter() {
                        any_device_found = true;
                        uinfo!("pcied", "device_found";
                            port = port,
                            vendor = dev.id.vendor_id,
                            device = dev.id.device_id
                        );

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
                            command: dev.command,
                        });
                    }
                }
            }
            Err(_) => {}
        }
    }

    // Register buses and devices with devd
    if active_ports != 0 {
        register_with_devd(&registry, active_ports);
    }

    // Register IPC port
    let server = match Server::<PcieProtocol>::register() {
        Ok(s) => s,
        Err(IpcError::ResourceBusy) => {
            uerror!("pcied", "already_running");
            syscall::exit(1);
        }
        Err(_) => {
            uerror!("pcied", "port_register_failed");
            syscall::exit(1);
        }
    };

    uinfo!("pcied", "ready"; devices = registry.count);

    // Main loop: handle queries (devd supervises us, no need to daemonize)
    loop {
        // Accept connection
        let mut conn: Connection<PcieProtocol> = match server.accept() {
            Ok(c) => c,
            Err(_) => {
                syscall::yield_now();
                continue;
            }
        };

        // Track ports this client queried (for auto-reset on disconnect)
        let mut client_ports: [Option<u8>; 4] = [None; 4];
        let mut client_port_count = 0usize;

        // Handle messages on this connection until disconnect
        loop {
            match conn.receive() {
                Ok(request) => {
                    let response = match request {
                        PcieRequest::FindDevice { vendor_id, device_id } => {
                            let (list, ports) = registry.find_devices(vendor_id, device_id);
                            // Merge new ports into client's tracked ports
                            for port_opt in ports.iter() {
                                if let Some(port) = *port_opt {
                                    let already_tracked = client_ports.iter()
                                        .take(client_port_count)
                                        .any(|p| *p == Some(port));
                                    if !already_tracked && client_port_count < 4 {
                                        client_ports[client_port_count] = Some(port);
                                        client_port_count += 1;
                                    }
                                }
                            }
                            PcieResponse::Devices(list)
                        }
                        PcieRequest::ResetPort { port } => {
                            // Security: ResetPort requires MMIO capability
                            if !conn.client_has_capability(syscall::caps::MMIO) {
                                uerror!("pcied", "reset_denied"; port = port,
                                    client_pid = conn.client_pid().unwrap_or(0));
                                PcieResponse::Error(-1) // Permission denied
                            } else {
                                let board = pcie_init.board();
                                if let Some(port_config) = board.soc().port_config(port) {
                                    match PcieController::init(port_config, |_| {}, |_| {}) {
                                        Ok(mut controller) => {
                                            let link_up = controller.reset_link();
                                            uinfo!("pcied", "port_reset"; port = port, link_up = link_up);
                                            PcieResponse::ResetResult(true)
                                        }
                                        Err(_) => PcieResponse::ResetResult(false)
                                    }
                                } else {
                                    PcieResponse::ResetResult(false)
                                }
                            }
                        }
                        PcieRequest::EnableBusMaster { port, bus, device, function } => {
                            // Security: EnableBusMaster requires DMA capability
                            if !conn.client_has_capability(syscall::caps::DMA) {
                                uerror!("pcied", "bme_denied"; port = port, bus = bus,
                                    client_pid = conn.client_pid().unwrap_or(0));
                                PcieResponse::Error(-1) // Permission denied
                            } else {
                                let board = pcie_init.board();
                                if let Some(port_config) = board.soc().port_config(port) {
                                    match PcieController::open_existing(port_config) {
                                        Ok(controller) => {
                                            let success = controller.enable_bus_master(bus, device, function);
                                            if success && bus > 0 {
                                                // Also enable BME on parent bridge
                                                controller.enable_bus_master(0, 0, 0);
                                            }
                                            PcieResponse::ResetResult(success)
                                        }
                                        Err(_) => PcieResponse::ResetResult(false)
                                    }
                                } else {
                                    PcieResponse::ResetResult(false)
                                }
                            }
                        }
                        PcieRequest::ReadDeviceStatus { port, bus, device, function } => {
                            // Use open_existing() - do NOT re-init/reset the port!
                            let board = pcie_init.board();
                            if let Some(port_config) = board.soc().port_config(port) {
                                match PcieController::open_existing(port_config) {
                                    Ok(controller) => {
                                        if let Some(status) = controller.read_device_status(bus, device, function) {
                                            PcieResponse::DeviceStatus(status)
                                        } else {
                                            PcieResponse::Error(-1)
                                        }
                                    }
                                    Err(_) => PcieResponse::Error(-1),
                                }
                            } else {
                                PcieResponse::Error(-1)
                            }
                        }
                        PcieRequest::ClearDeviceStatus { port, bus, device, function } => {
                            // Security: ClearDeviceStatus requires MMIO capability
                            if !conn.client_has_capability(syscall::caps::MMIO) {
                                uerror!("pcied", "clear_status_denied"; port = port,
                                    client_pid = conn.client_pid().unwrap_or(0));
                                PcieResponse::Error(-1) // Permission denied
                            } else {
                                // Use open_existing() - do NOT re-init/reset the port!
                                let board = pcie_init.board();
                                if let Some(port_config) = board.soc().port_config(port) {
                                    match PcieController::open_existing(port_config) {
                                        Ok(controller) => {
                                            let success = controller.clear_device_status(bus, device, function);
                                            PcieResponse::ResetResult(success)
                                        }
                                        Err(_) => PcieResponse::ResetResult(false),
                                    }
                                } else {
                                    PcieResponse::ResetResult(false)
                                }
                            }
                        }
                        PcieRequest::ReadRegister { port, bus, device, function, offset } => {
                            if let Some(dev) = registry.find_by_bdf(port, bus, device, function) {
                                if dev.bar0_size == 0 {
                                    PcieResponse::Error(-2)
                                } else if offset as u64 + 4 > dev.bar0_size as u64 {
                                    PcieResponse::Error(-3)
                                } else {
                                    match syscall::mmap_device(dev.bar0_addr, dev.bar0_size as u64) {
                                        Ok(vaddr) => {
                                            let reg_ptr = (vaddr + offset as u64) as *const u32;
                                            let value = unsafe { core::ptr::read_volatile(reg_ptr) };
                                            PcieResponse::RegisterValue(value)
                                        }
                                        Err(_) => PcieResponse::Error(-4)
                                    }
                                }
                            } else {
                                PcieResponse::Error(-1)
                            }
                        }
                        PcieRequest::WriteRegister { port, bus, device, function, offset, value } => {
                            if let Some(dev) = registry.find_by_bdf(port, bus, device, function) {
                                if dev.bar0_size == 0 || offset as u64 + 4 > dev.bar0_size as u64 {
                                    PcieResponse::ResetResult(false)
                                } else {
                                    match syscall::mmap_device(dev.bar0_addr, dev.bar0_size as u64) {
                                        Ok(vaddr) => {
                                            let reg_ptr = (vaddr + offset as u64) as *mut u32;
                                            unsafe { core::ptr::write_volatile(reg_ptr, value) };
                                            PcieResponse::ResetResult(true)
                                        }
                                        Err(_) => PcieResponse::ResetResult(false)
                                    }
                                }
                            } else {
                                PcieResponse::ResetResult(false)
                            }
                        }
                    };
                    let _ = conn.send(&response);
                }
                Err(_) => {
                    // Disconnect - auto-reset ports this client queried
                    for port_opt in client_ports.iter().take(client_port_count) {
                        if let Some(port) = *port_opt {
                            let board = pcie_init.board();
                            if let Some(port_config) = board.soc().port_config(port) {
                                if let Ok(mut controller) = PcieController::init(port_config, |_| {}, |_| {}) {
                                    let link_up = controller.reset_link();
                                    uinfo!("pcied", "auto_reset"; port = port, link_up = link_up);
                                }
                            }
                        }
                    }
                    break;
                }
            }
        }
        // Connection dropped here, channel closed automatically
    }
}

