//! Device Supervisor Daemon (devd)
//!
//! The unified device manager and service supervisor for the microkernel.
//! Acts as PID 1 (init) - detects devices, spawns drivers, supervises all.
//!
//! ## Responsibilities
//! - Scan buses (Platform, PCIe, USB reports)
//! - Match devices to drivers via compiled rules
//! - Spawn and supervise driver daemons
//! - Restart crashed drivers (with backoff)
//! - Provide device query/subscribe API for clients
//!
//! ## Architecture
//! ```text
//! devd (PID 1)
//!  ├─ detects UART → spawns shell
//!  ├─ detects xHCI → spawns usbd
//!  │   └─ usbd reports USB devices back
//!  └─ supervises all, restarts on crash
//! ```

#![no_std]
#![no_main]

use userlib::println;
use userlib::syscall::{self, PciDeviceInfo};

// =============================================================================
// Constants
// =============================================================================

const MAX_DRIVERS: usize = 16;
const MAX_DEVICES: usize = 32;
const MAX_RESTARTS: u32 = 3;
const RESTART_DELAY_MS: u64 = 1000;

// PCI class codes
const PCI_CLASS_USB: u32 = 0x0C0300;  // USB controller (class 0C, subclass 03)

// =============================================================================
// Data Structures
// =============================================================================

/// Bus types for device identification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusType {
    /// Platform devices (UART, timers, etc. - always present)
    Platform,
    /// PCI Express devices
    PCIe,
    /// USB devices (reported by usbd)
    USB,
}

/// Device class for matching
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceClass {
    /// Serial console (UART)
    Serial,
    /// USB host controller
    UsbController,
    /// Mass storage device
    Storage,
    /// Network interface
    Network,
    /// Human interface device (keyboard, mouse)
    Hid,
    /// Unknown/other
    Other,
}

/// A discovered device
#[derive(Clone)]
pub struct Device {
    /// Unique device path (e.g., "/platform/uart", "/pcie/0/usb")
    pub path: [u8; 64],
    pub path_len: usize,
    /// Bus type
    pub bus: BusType,
    /// Device class
    pub class: DeviceClass,
    /// Vendor ID (if applicable)
    pub vendor_id: u16,
    /// Device ID (if applicable)
    pub device_id: u16,
    /// PCI BDF (if PCIe device)
    pub pci_bdf: u32,
    /// Whether a driver is assigned
    pub driver_assigned: bool,
}

impl Device {
    pub const fn empty() -> Self {
        Self {
            path: [0; 64],
            path_len: 0,
            bus: BusType::Platform,
            class: DeviceClass::Other,
            vendor_id: 0,
            device_id: 0,
            pci_bdf: 0,
            driver_assigned: false,
        }
    }

    pub fn set_path(&mut self, s: &str) {
        let bytes = s.as_bytes();
        let len = bytes.len().min(self.path.len());
        self.path[..len].copy_from_slice(&bytes[..len]);
        self.path_len = len;
    }

    pub fn path_str(&self) -> &str {
        core::str::from_utf8(&self.path[..self.path_len]).unwrap_or("")
    }
}

/// Driver state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverState {
    /// Not started
    Stopped,
    /// Running normally
    Running,
    /// Crashed, pending restart
    Crashed,
    /// Failed permanently (max restarts exceeded)
    Failed,
}

/// A supervised driver
pub struct DriverEntry {
    /// Binary name (e.g., "shell", "usbd")
    pub binary: &'static str,
    /// Process ID (0 if not running)
    pub pid: u32,
    /// Current state
    pub state: DriverState,
    /// Number of restarts
    pub restart_count: u32,
    /// Device path this driver manages
    pub device_path: [u8; 64],
    pub device_path_len: usize,
}

impl DriverEntry {
    pub const fn empty() -> Self {
        Self {
            binary: "",
            pid: 0,
            state: DriverState::Stopped,
            restart_count: 0,
            device_path: [0; 64],
            device_path_len: 0,
        }
    }

    pub fn is_active(&self) -> bool {
        self.pid != 0
    }
}

/// Match rule: which driver handles which device
pub struct MatchRule {
    /// Bus type to match (None = any)
    pub bus: Option<BusType>,
    /// Device class to match (None = any)
    pub class: Option<DeviceClass>,
    /// PCI class code to match (None = any)
    pub pci_class: Option<u32>,
    /// Driver binary to spawn
    pub driver: &'static str,
}

// =============================================================================
// Match Rules (compiled-in for v1)
// =============================================================================

static MATCH_RULES: &[MatchRule] = &[
    // Platform UART → shell
    MatchRule {
        bus: Some(BusType::Platform),
        class: Some(DeviceClass::Serial),
        pci_class: None,
        driver: "shell",
    },
    // Platform USB controller (MT7988A SSUSB) → usbd
    MatchRule {
        bus: Some(BusType::Platform),
        class: Some(DeviceClass::UsbController),
        pci_class: None,
        driver: "usbd",
    },
    // PCIe USB controller → usbd (for future/other boards)
    MatchRule {
        bus: Some(BusType::PCIe),
        class: None,
        pci_class: Some(PCI_CLASS_USB),
        driver: "usbd",
    },
];

// =============================================================================
// Device Manager
// =============================================================================

struct DeviceManager {
    devices: [Device; MAX_DEVICES],
    device_count: usize,
    drivers: [DriverEntry; MAX_DRIVERS],
    driver_count: usize,
}

impl DeviceManager {
    const fn new() -> Self {
        const EMPTY_DEV: Device = Device::empty();
        const EMPTY_DRV: DriverEntry = DriverEntry::empty();
        Self {
            devices: [EMPTY_DEV; MAX_DEVICES],
            device_count: 0,
            drivers: [EMPTY_DRV; MAX_DRIVERS],
            driver_count: 0,
        }
    }

    /// Add a discovered device
    fn add_device(&mut self, device: Device) -> bool {
        if self.device_count >= MAX_DEVICES {
            println!("[devd] Device table full!");
            return false;
        }
        self.devices[self.device_count] = device;
        self.device_count += 1;
        true
    }

    /// Spawn a driver for a device
    fn spawn_driver(&mut self, binary: &'static str, device_path: &str) -> Option<u32> {
        // Find empty driver slot
        let slot = self.drivers[..self.driver_count]
            .iter()
            .position(|d| !d.is_active())
            .unwrap_or(self.driver_count);

        if slot >= MAX_DRIVERS {
            println!("[devd] Driver table full!");
            return None;
        }

        // Spawn the process
        let pid = syscall::exec(binary);
        if pid < 0 {
            println!("[devd] Failed to spawn {}: error {}", binary, pid);
            return None;
        }

        let pid = pid as u32;
        println!("[devd] Spawned {} (PID {}) for {}", binary, pid, device_path);

        // Record driver entry
        let entry = &mut self.drivers[slot];
        entry.binary = binary;
        entry.pid = pid;
        entry.state = DriverState::Running;
        entry.restart_count = 0;

        let path_bytes = device_path.as_bytes();
        let len = path_bytes.len().min(entry.device_path.len());
        entry.device_path[..len].copy_from_slice(&path_bytes[..len]);
        entry.device_path_len = len;

        if slot >= self.driver_count {
            self.driver_count = slot + 1;
        }

        Some(pid)
    }

    /// Handle a child process exit
    fn handle_exit(&mut self, pid: u32, exit_code: i32) {
        // Find which driver this was
        for i in 0..self.driver_count {
            if self.drivers[i].pid == pid {
                let entry = &mut self.drivers[i];
                let binary = entry.binary;

                if exit_code == 0 {
                    println!("[devd] {} (PID {}) exited normally", binary, pid);
                    entry.state = DriverState::Stopped;
                    entry.pid = 0;
                } else {
                    println!("[devd] {} (PID {}) crashed with code {}", binary, pid, exit_code);
                    entry.restart_count += 1;

                    if entry.restart_count > MAX_RESTARTS {
                        println!("[devd] {} exceeded max restarts, marking failed", binary);
                        entry.state = DriverState::Failed;
                        entry.pid = 0;
                    } else {
                        println!("[devd] Restarting {} (attempt {}/{})",
                                 binary, entry.restart_count, MAX_RESTARTS);
                        entry.state = DriverState::Crashed;
                        entry.pid = 0;

                        // Delay before restart (spin-wait using gettime)
                        let start = syscall::gettime();
                        while syscall::gettime() < start + RESTART_DELAY_MS {
                            syscall::yield_now();
                        }

                        // Respawn
                        let new_pid = syscall::exec(binary);
                        if new_pid > 0 {
                            entry.pid = new_pid as u32;
                            entry.state = DriverState::Running;
                            println!("[devd] {} restarted as PID {}", binary, new_pid);
                        } else {
                            println!("[devd] Failed to restart {}", binary);
                            entry.state = DriverState::Failed;
                        }
                    }
                }
                return;
            }
        }

        println!("[devd] Unknown child PID {} exited", pid);
    }
}

// =============================================================================
// Bus Scanning
// =============================================================================

/// Scan platform devices (UART, USB controllers are always present on MT7988A)
fn scan_platform(mgr: &mut DeviceManager) {
    println!("[devd] Scanning platform devices...");

    // UART0 is always present
    let mut uart = Device::empty();
    uart.set_path("/platform/uart0");
    uart.bus = BusType::Platform;
    uart.class = DeviceClass::Serial;

    if mgr.add_device(uart) {
        println!("[devd]   Found: /platform/uart0 (Serial)");
    }

    // SSUSB1 - USB-A ports (xHCI at 0x11200000)
    // This is the one with the external USB ports on BPI-R4
    let mut usb1 = Device::empty();
    usb1.set_path("/platform/ssusb1");
    usb1.bus = BusType::Platform;
    usb1.class = DeviceClass::UsbController;

    if mgr.add_device(usb1) {
        println!("[devd]   Found: /platform/ssusb1 (USB Controller)");
    }
}

/// Scan PCIe bus for devices
fn scan_pcie(mgr: &mut DeviceManager) {
    println!("[devd] Scanning PCIe bus...");

    let mut devices = [PciDeviceInfo::default(); 16];
    let count = syscall::pci_enumerate(&mut devices);

    if count < 0 {
        println!("[devd]   PCIe enumeration failed: {}", count);
        return;
    }

    let count = count as usize;
    println!("[devd]   Found {} PCIe device(s)", count);

    for i in 0..count {
        let pci = &devices[i];
        let class = pci.class_code >> 8;  // Base class + subclass

        // Determine device class
        let dev_class = match class {
            0x0C03 => DeviceClass::UsbController,
            0x0200 => DeviceClass::Network,
            0x0100 | 0x0106 => DeviceClass::Storage,
            _ => DeviceClass::Other,
        };

        let bdf = pci.bdf;
        let bus = (bdf >> 8) & 0xFF;
        let dev = (bdf >> 3) & 0x1F;
        let func = bdf & 0x7;

        // Create device path
        let mut device = Device::empty();
        // Format path manually since we don't have format!
        let path = match dev_class {
            DeviceClass::UsbController => "/pcie/usb",
            DeviceClass::Network => "/pcie/net",
            DeviceClass::Storage => "/pcie/storage",
            _ => "/pcie/unknown",
        };
        device.set_path(path);
        device.bus = BusType::PCIe;
        device.class = dev_class;
        device.vendor_id = pci.vendor_id;
        device.device_id = pci.device_id;
        device.pci_bdf = bdf;

        println!("[devd]   {:04x}:{:02x}:{:02x}.{} - {:04x}:{:04x} class {:06x} ({:?})",
                 (bdf >> 16) & 0xFF, bus, dev, func,
                 pci.vendor_id, pci.device_id, pci.class_code, dev_class);

        mgr.add_device(device);
    }
}

// =============================================================================
// Main Loop
// =============================================================================

fn spawn_drivers_for_devices(mgr: &mut DeviceManager) {
    println!("[devd] Matching devices to drivers...");

    for i in 0..mgr.device_count {
        // Copy out what we need to avoid borrow issues
        let bus = mgr.devices[i].bus;
        let class = mgr.devices[i].class;
        let assigned = mgr.devices[i].driver_assigned;

        if assigned {
            continue;
        }

        // Find matching driver
        let mut driver_to_spawn: Option<&'static str> = None;

        for rule in MATCH_RULES {
            let mut matched = true;

            if let Some(rule_bus) = rule.bus {
                if rule_bus != bus {
                    matched = false;
                }
            }

            if let Some(rule_class) = rule.class {
                if rule_class != class {
                    matched = false;
                }
            }

            if let Some(pci_class) = rule.pci_class {
                if bus != BusType::PCIe {
                    matched = false;
                } else if pci_class == PCI_CLASS_USB && class != DeviceClass::UsbController {
                    matched = false;
                }
            }

            if matched {
                driver_to_spawn = Some(rule.driver);
                break;
            }
        }

        // Spawn if we found a match
        if let Some(driver) = driver_to_spawn {
            let path = mgr.devices[i].path_str();
            // Need to copy path since spawn_driver borrows mgr mutably
            let mut path_buf = [0u8; 64];
            let path_bytes = path.as_bytes();
            let len = path_bytes.len().min(64);
            path_buf[..len].copy_from_slice(&path_bytes[..len]);

            if let Ok(path_str) = core::str::from_utf8(&path_buf[..len]) {
                if mgr.spawn_driver(driver, path_str).is_some() {
                    mgr.devices[i].driver_assigned = true;
                }
            }
        }
    }
}

#[unsafe(no_mangle)]
fn main() -> ! {
    println!();
    println!("========================================");
    println!("  devd - Device Supervisor");
    println!("  BPI-R4 / MT7988A");
    println!("========================================");
    println!();

    let mut mgr = DeviceManager::new();

    // Phase 1: Scan buses
    scan_platform(&mut mgr);
    scan_pcie(&mut mgr);

    // Phase 2: Match and spawn drivers
    spawn_drivers_for_devices(&mut mgr);

    println!();
    println!("[devd] Supervision started ({} devices, {} drivers)",
             mgr.device_count, mgr.driver_count);
    println!();

    // Phase 3: Supervision loop - wait for children, restart on crash
    loop {
        let mut status: i32 = 0;
        let pid = syscall::waitpid(-1, &mut status);

        if pid > 0 {
            mgr.handle_exit(pid as u32, status);
        } else if pid == -10 {
            // ECHILD - no children, shouldn't happen if drivers are running
            // Wait a bit and continue
            let start = syscall::gettime();
            while syscall::gettime() < start + 1000 {
                syscall::yield_now();
            }
        } else {
            // Other error or would-block, just yield
            syscall::yield_now();
        }
    }
}
