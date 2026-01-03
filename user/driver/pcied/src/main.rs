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

use userlib::{println, print, syscall};
use userlib::ipc::{Server, Connection, IpcError};
use userlib::ipc::protocols::{
    PcieProtocol, PcieRequest, PcieResponse,
    PcieDeviceInfo, DeviceList,
    DevdClient, PropertyList,
};
use pcie::{
    Board, BpiR4, PcieInit, SocPcie,
    PcieController, PcieDevice,
    consts,
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

fn debug_print(s: &str) {
    print!("{} ", s);
}

fn debug_status(val: u32) {
    print!("[{:08x}] ", val);
}

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

/// Register all devices with devd
fn register_with_devd(registry: &DeviceRegistry) {
    print!("Registering with devd... ");

    let mut devd = match DevdClient::connect() {
        Ok(d) => d,
        Err(e) => {
            println!("FAILED: {:?}", e);
            return;
        }
    };

    let mut registered = 0usize;

    for i in 0..registry.count {
        let dev = &registry.devices[i];

        // Build path: /bus/pcie{port}/{bdf}
        // e.g. /bus/pcie0/01:00.0
        let mut path_buf = [0u8; 32];
        let path = {
            let port = dev.port;
            let bus = dev.bus;
            let device = dev.device;
            let function = dev.function;

            // Format: /bus/pcie{port}/{bus:02x}:{dev:02x}.{func:x}
            const HEX: &[u8; 16] = b"0123456789abcdef";
            let mut i = 0;

            // /bus/pcie
            path_buf[i..i+9].copy_from_slice(b"/bus/pcie");
            i += 9;
            path_buf[i] = b'0' + port;
            i += 1;
            path_buf[i] = b'/';
            i += 1;
            // bus:02x
            path_buf[i] = HEX[(bus >> 4) as usize];
            i += 1;
            path_buf[i] = HEX[(bus & 0xf) as usize];
            i += 1;
            path_buf[i] = b':';
            i += 1;
            // dev:02x
            path_buf[i] = HEX[(device >> 4) as usize];
            i += 1;
            path_buf[i] = HEX[(device & 0xf) as usize];
            i += 1;
            path_buf[i] = b'.';
            i += 1;
            // func:x
            path_buf[i] = HEX[(function & 0xf) as usize];
            i += 1;

            core::str::from_utf8(&path_buf[..i]).unwrap_or("")
        };

        // Format hex values as strings
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
            .add("state", "bound");

        if devd.register(path, props).is_ok() {
            registered += 1;
        }
    }

    println!("OK ({} devices)", registered);
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

    // Step 3: Discover PCIe buses from kernel (single source of truth from DTB)
    // Note: We don't connect to bus ports - devd authorizes us via SetDriver
    let mut buses = [syscall::BusInfo::empty(); 16];
    let bus_count = syscall::bus_list(&mut buses);
    if bus_count < 0 {
        println!("Failed to query bus list: {}", bus_count);
        syscall::exit(1);
        return;
    }

    // Find which PCIe ports are available
    let mut available_ports = 0u8;
    println!("Discovering PCIe buses (from kernel)...");
    for i in 0..(bus_count as usize) {
        let info = &buses[i];
        if info.bus_type == syscall::bus_type::PCIE {
            let port = info.bus_index;
            if (port as usize) < 4 {
                available_ports |= 1 << port;
                println!("  {}: available", info.path_str());
            }
        }
    }

    if available_ports == 0 {
        println!("No PCIe ports found in kernel bus list");
        syscall::exit(1);
        return;
    }
    println!();

    // Step 4: Initialize each available port and enumerate devices
    // devd has authorized us via SetDriver - we trust our spawn context
    let mut any_device_found = false;

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
                                command: dev.command,
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

        // Register all devices with devd
        register_with_devd(&registry);
    } else {
        println!("No PCIe devices detected");
    }

    // Step 4: Register port and serve queries
    println!();
    let server = match Server::<PcieProtocol>::register() {
        Ok(s) => {
            println!("Registered 'pcie' port");
            s
        }
        Err(IpcError::ResourceBusy) => {
            println!("ERROR: PCIe daemon already running");
            syscall::exit(1);
        }
        Err(e) => {
            println!("ERROR: Failed to register pcie port: {:?}", e);
            syscall::exit(1);
        }
    };
    println!("PCIe daemon running");

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
                            let board = pcie_init.board();
                            if let Some(port_config) = board.soc().port_config(port) {
                                println!("[pcied] Resetting port {}...", port);
                                match PcieController::init(port_config, |_| {}, |_| {}) {
                                    Ok(mut controller) => {
                                        if controller.reset_link() {
                                            println!("[pcied] Port {} reset OK, link up", port);
                                        } else {
                                            println!("[pcied] Port {} reset, no link", port);
                                        }
                                        PcieResponse::ResetResult(true)
                                    }
                                    Err(e) => {
                                        println!("[pcied] Port {} reset failed: {:?}", port, e);
                                        PcieResponse::ResetResult(false)
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
                    if client_port_count > 0 {
                        println!("[pcied] Client disconnected, resetting {} port(s)", client_port_count);
                        for port_opt in client_ports.iter().take(client_port_count) {
                            if let Some(port) = *port_opt {
                                let board = pcie_init.board();
                                if let Some(port_config) = board.soc().port_config(port) {
                                    print!("[pcied] Auto-resetting port {}... ", port);
                                    match PcieController::init(port_config, |_| {}, |_| {}) {
                                        Ok(mut controller) => {
                                            if controller.reset_link() {
                                                println!("OK (link up)");
                                            } else {
                                                println!("OK (no link)");
                                            }
                                        }
                                        Err(_) => println!("failed"),
                                    }
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
        (consts::vendor::MEDIATEK, consts::device::MT7996) => "MT7996 WiFi 7 (primary)",
        (consts::vendor::MEDIATEK, consts::device::MT7996_HIF2) => "MT7996 WiFi 7 (HIF2)",
        (consts::vendor::MEDIATEK, consts::device::MT7992) => "MT7992 WiFi 7 (primary)",
        (consts::vendor::MEDIATEK, consts::device::MT7992_HIF2) => "MT7992 WiFi 7 (HIF2)",
        (consts::vendor::MEDIATEK, consts::device::MT7990) => "MT7990 WiFi 7 (primary)",
        (consts::vendor::MEDIATEK, consts::device::MT7990_HIF2) => "MT7990 WiFi 7 (HIF2)",
        (consts::vendor::MEDIATEK, consts::device::MT7915) => "MT7915 WiFi 6",
        (consts::vendor::MEDIATEK, consts::device::MT7988_RC) => "MT7988 Root Complex",
        _ => {
            println!("         Vendor: {}", vendor_name);
            return;
        }
    };

    println!("         {} {}", vendor_name, device_name);
}
