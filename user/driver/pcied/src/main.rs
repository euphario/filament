//! PCIe Bus Driver
//!
//! Enumerates PCIe devices via ECAM, enables bus mastering, maps BARs,
//! and registers devices with devd for driver spawning.
//!
//! Supports:
//! - QEMU virt (ECAM at 0x4010000000)
//! - MT7988A (via platform-specific MAC access)

#![no_std]
#![no_main]

use userlib::syscall;
use userlib::syscall::LogLevel;
use userlib::devd::{DevdClient, PortType};
use userlib::error::SysError;
use userlib::mmio::MmioRegion;

// ============================================================================
// Logging
// ============================================================================

macro_rules! plog {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        const PREFIX: &[u8] = b"[pcied] ";
        let mut buf = [0u8; 256];
        buf[..PREFIX.len()].copy_from_slice(PREFIX);
        let mut pos = PREFIX.len();
        struct W<'a> { b: &'a mut [u8], p: &'a mut usize }
        impl core::fmt::Write for W<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                for &byte in s.as_bytes() {
                    if *self.p < self.b.len() { self.b[*self.p] = byte; *self.p += 1; }
                }
                Ok(())
            }
        }
        let _ = write!(W { b: &mut buf, p: &mut pos }, $($arg)*);
        syscall::klog(LogLevel::Info, &buf[..pos]);
    }};
}

// ============================================================================
// Platform Detection
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Platform {
    QemuVirt,
    Mt7988a,
    Unknown,
}

impl Platform {
    fn detect() -> Self {
        // For now, assume QEMU virt platform
        // The full ECAM mapping will validate if the platform is correct
        // by checking vendor IDs during enumeration
        //
        // TODO: Use FDT or other method to detect platform without
        // requiring an extra MMIO mapping
        Platform::QemuVirt
    }

    fn ecam_base(&self) -> Option<u64> {
        match self {
            Platform::QemuVirt => Some(QEMU_ECAM_BASE),
            Platform::Mt7988a => None, // Uses MAC-based access
            Platform::Unknown => None,
        }
    }
}

// ============================================================================
// ECAM Constants
// ============================================================================

/// QEMU virt ECAM base address
const QEMU_ECAM_BASE: u64 = 0x40_1000_0000;

/// QEMU virt ECAM size (4MB - 4 buses * 1MB each)
const QEMU_ECAM_SIZE: u64 = 0x40_0000;

/// Maximum buses to scan
const MAX_BUSES: u8 = 4;

/// Maximum devices per bus
const MAX_DEVICES: u8 = 32;

/// Maximum functions per device
const MAX_FUNCTIONS: u8 = 8;

// ============================================================================
// PCI Configuration Space
// ============================================================================

/// PCI config space register offsets
mod pci_cfg {
    pub const VENDOR_DEVICE: usize = 0x00;
    pub const COMMAND_STATUS: usize = 0x04;
    pub const CLASS_REV: usize = 0x08;
    pub const HEADER_TYPE: usize = 0x0C;
    pub const BAR0: usize = 0x10;
}

/// PCI command register bits
mod pci_cmd {
    pub const MEMORY_SPACE: u16 = 1 << 1;
    pub const BUS_MASTER: u16 = 1 << 2;
}

/// PCI class codes
mod pci_class {
    pub const NETWORK: u8 = 0x02;
    pub const SERIAL_BUS: u8 = 0x0C;
}

/// PCI subclass codes for serial bus
mod pci_subclass {
    pub const USB: u8 = 0x03;
}

/// PCI prog-if for USB
mod pci_progif {
    pub const XHCI: u8 = 0x30;
}

// ============================================================================
// ECAM Access
// ============================================================================

/// ECAM-based PCI config space access
struct EcamAccess {
    mmio: MmioRegion,
}

impl EcamAccess {
    /// Create ECAM accessor by mapping the config space region
    fn new(ecam_phys: u64) -> Option<Self> {
        let mmio = MmioRegion::open(ecam_phys, QEMU_ECAM_SIZE)?;
        Some(Self { mmio })
    }

    /// Calculate config space offset for BDF
    fn offset(&self, bus: u8, device: u8, function: u8, reg: usize) -> usize {
        let bdf = ((bus as usize) << 20) | ((device as usize) << 15) | ((function as usize) << 12);
        bdf + reg
    }

    /// Read 32-bit config register
    fn read32(&self, bus: u8, device: u8, function: u8, reg: usize) -> u32 {
        let offset = self.offset(bus, device, function, reg);
        self.mmio.read32(offset)
    }

    /// Write 32-bit config register
    fn write32(&self, bus: u8, device: u8, function: u8, reg: usize, val: u32) {
        let offset = self.offset(bus, device, function, reg);
        self.mmio.write32(offset, val);
    }

    /// Read 16-bit config register
    fn read16(&self, bus: u8, device: u8, function: u8, reg: usize) -> u16 {
        let offset = self.offset(bus, device, function, reg);
        self.mmio.read16(offset)
    }

    /// Write 16-bit config register
    fn write16(&self, bus: u8, device: u8, function: u8, reg: usize, val: u16) {
        let offset = self.offset(bus, device, function, reg);
        self.mmio.write16(offset, val);
    }
}

// ============================================================================
// Device Discovery
// ============================================================================

/// Discovered PCI device
#[derive(Clone, Copy, Default)]
struct PciDevice {
    bus: u8,
    device: u8,
    function: u8,
    vendor_id: u16,
    device_id: u16,
    class: u8,
    subclass: u8,
    prog_if: u8,
    bar0_phys: u64,
    bar0_size: u64,
}

impl PciDevice {
    /// Check if this is an xHCI controller
    fn is_xhci(&self) -> bool {
        self.class == pci_class::SERIAL_BUS
            && self.subclass == pci_subclass::USB
            && self.prog_if == pci_progif::XHCI
    }

    /// Check if this is a network controller
    fn is_network(&self) -> bool {
        self.class == pci_class::NETWORK
    }

    /// Get class name for devd registration
    fn class_name(&self) -> &'static str {
        match (self.class, self.subclass, self.prog_if) {
            (0x0C, 0x03, 0x30) => "xhci",
            (0x0C, 0x03, 0x20) => "ehci",
            (0x0C, 0x03, _) => "usb",
            (0x02, _, _) => "network",
            (0x01, _, _) => "storage",
            (0x03, _, _) => "display",
            (0x06, _, _) => "bridge",
            _ => "other",
        }
    }

    /// Get port type for devd registration
    fn port_type(&self) -> PortType {
        match (self.class, self.subclass) {
            (0x0C, 0x03) => PortType::Usb,      // USB controller
            (0x02, _) => PortType::Network,     // Network controller
            (0x01, _) => PortType::Block,       // Storage controller
            _ => PortType::Service,             // Default
        }
    }
}

/// Device registry
struct DeviceRegistry {
    devices: [PciDevice; 32],
    count: usize,
}

impl DeviceRegistry {
    fn new() -> Self {
        Self {
            devices: [PciDevice::default(); 32],
            count: 0,
        }
    }

    fn add(&mut self, dev: PciDevice) -> bool {
        if self.count < 32 {
            self.devices[self.count] = dev;
            self.count += 1;
            true
        } else {
            false
        }
    }

    fn iter(&self) -> impl Iterator<Item = &PciDevice> {
        self.devices[..self.count].iter()
    }

    fn format_info(&self) -> [u8; 1024] {
        use core::fmt::Write;
        let mut buf = [0u8; 1024];
        let mut pos = 0;

        struct W<'a> { b: &'a mut [u8], p: &'a mut usize }
        impl core::fmt::Write for W<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                for &byte in s.as_bytes() {
                    if *self.p < self.b.len() { self.b[*self.p] = byte; *self.p += 1; }
                }
                Ok(())
            }
        }

        let mut w = W { b: &mut buf, p: &mut pos };

        let _ = writeln!(w, "PCIe Bus Driver (ECAM)");
        let _ = writeln!(w, "  Platform: QEMU virt");
        let _ = writeln!(w, "  ECAM Base: {:#x}", QEMU_ECAM_BASE);
        let _ = writeln!(w, "");
        let _ = writeln!(w, "Devices: {}", self.count);

        for dev in self.iter() {
            let _ = writeln!(w, "  {:02x}:{:02x}.{}: {} [{:04x}:{:04x}]",
                dev.bus, dev.device, dev.function,
                dev.class_name(), dev.vendor_id, dev.device_id);
            if dev.bar0_phys != 0 {
                let size_kb = dev.bar0_size / 1024;
                if size_kb >= 1024 {
                    let _ = writeln!(w, "    BAR0: {:#x} ({} MB)", dev.bar0_phys, size_kb / 1024);
                } else {
                    let _ = writeln!(w, "    BAR0: {:#x} ({} KB)", dev.bar0_phys, size_kb);
                }
            }
        }

        buf
    }
}

// ============================================================================
// Bus Enumeration
// ============================================================================

/// Enumerate PCI bus using ECAM
fn enumerate_ecam(ecam: &EcamAccess, registry: &mut DeviceRegistry) {
    for bus in 0..MAX_BUSES {
        for device in 0..MAX_DEVICES {
            // Check function 0 first
            let vendor_device = ecam.read32(bus, device, 0, pci_cfg::VENDOR_DEVICE);
            let vendor_id = (vendor_device & 0xFFFF) as u16;

            if vendor_id == 0xFFFF || vendor_id == 0 {
                continue; // No device
            }

            // Check header type for multi-function
            let header = ecam.read32(bus, device, 0, pci_cfg::HEADER_TYPE);
            let header_type = ((header >> 16) & 0xFF) as u8;
            let multi_function = (header_type & 0x80) != 0;

            let max_func = if multi_function { MAX_FUNCTIONS } else { 1 };

            for function in 0..max_func {
                if function > 0 {
                    let vd = ecam.read32(bus, device, function, pci_cfg::VENDOR_DEVICE);
                    if (vd & 0xFFFF) as u16 == 0xFFFF {
                        continue;
                    }
                }

                if let Some(dev) = probe_device(ecam, bus, device, function) {
                    registry.add(dev);
                }
            }
        }
    }
}

/// Probe a single device
fn probe_device(ecam: &EcamAccess, bus: u8, device: u8, function: u8) -> Option<PciDevice> {
    let vendor_device = ecam.read32(bus, device, function, pci_cfg::VENDOR_DEVICE);
    let vendor_id = (vendor_device & 0xFFFF) as u16;
    let device_id = ((vendor_device >> 16) & 0xFFFF) as u16;

    if vendor_id == 0xFFFF || vendor_id == 0 {
        return None;
    }

    let class_rev = ecam.read32(bus, device, function, pci_cfg::CLASS_REV);
    let prog_if = ((class_rev >> 8) & 0xFF) as u8;
    let subclass = ((class_rev >> 16) & 0xFF) as u8;
    let class = ((class_rev >> 24) & 0xFF) as u8;

    let header = ecam.read32(bus, device, function, pci_cfg::HEADER_TYPE);
    let header_type = ((header >> 16) & 0x7F) as u8;

    // Skip bridges for now (header type 1)
    if header_type != 0 {
        return None;
    }

    // Probe BAR0
    let (bar0_phys, bar0_size) = probe_bar(ecam, bus, device, function, pci_cfg::BAR0);

    Some(PciDevice {
        bus,
        device,
        function,
        vendor_id,
        device_id,
        class,
        subclass,
        prog_if,
        bar0_phys,
        bar0_size,
    })
}

/// Probe a BAR to get its address and size
fn probe_bar(ecam: &EcamAccess, bus: u8, device: u8, function: u8, bar_reg: usize) -> (u64, u64) {
    let original = ecam.read32(bus, device, function, bar_reg);

    // Check if it's an I/O BAR (bit 0 set)
    if (original & 1) != 0 {
        return (0, 0); // Skip I/O BARs
    }

    // Check if it's a 64-bit BAR
    let bar_type = (original >> 1) & 0x3;
    let is_64bit = bar_type == 2;

    // Write all 1s to get size
    ecam.write32(bus, device, function, bar_reg, 0xFFFFFFFF);
    let size_mask_low = ecam.read32(bus, device, function, bar_reg);

    let size_mask_high = if is_64bit {
        let orig_high = ecam.read32(bus, device, function, bar_reg + 4);
        ecam.write32(bus, device, function, bar_reg + 4, 0xFFFFFFFF);
        let mask_high = ecam.read32(bus, device, function, bar_reg + 4);
        ecam.write32(bus, device, function, bar_reg + 4, orig_high);
        mask_high
    } else {
        0xFFFFFFFF // For 32-bit BARs, treat high bits as all 1s
    };

    ecam.write32(bus, device, function, bar_reg, original);

    if size_mask_low == 0 || size_mask_low == 0xFFFFFFFF {
        return (0, 0);
    }

    // Calculate size:
    // 1. Combine low and high masks
    // 2. Mask out type bits (low 4 bits)
    // 3. Invert and add 1 to get size
    let full_mask = ((size_mask_high as u64) << 32) | (size_mask_low as u64);
    let size = !(full_mask & !0xF) + 1;

    // Get address
    let mut addr = (original & !0xF) as u64;

    if is_64bit {
        let high = ecam.read32(bus, device, function, bar_reg + 4);
        addr |= (high as u64) << 32;
    }

    (addr, size)
}

// ============================================================================
// Device Setup
// ============================================================================

/// Enable bus mastering for a device
fn enable_bus_master(ecam: &EcamAccess, dev: &PciDevice) {
    let cmd = ecam.read16(dev.bus, dev.device, dev.function, pci_cfg::COMMAND_STATUS);
    if (cmd & pci_cmd::BUS_MASTER) == 0 {
        ecam.write16(dev.bus, dev.device, dev.function, pci_cfg::COMMAND_STATUS,
            cmd | pci_cmd::BUS_MASTER | pci_cmd::MEMORY_SPACE);
    }
}

// ============================================================================
// devd Registration
// ============================================================================

/// Register device with devd
fn register_device(client: &mut DevdClient, dev: &PciDevice) -> Result<(), SysError> {
    // Format port name: pci/{bb:dd.f}:{class}
    // e.g., "pci/00:01.0:xhci"
    let mut name_buf = [0u8; 32];
    let class_name = dev.class_name();

    // Build name manually
    let mut i = 0;
    for b in b"pci/" {
        name_buf[i] = *b;
        i += 1;
    }

    // BDF: bb:dd.f
    const HEX: &[u8; 16] = b"0123456789abcdef";
    name_buf[i] = HEX[(dev.bus >> 4) as usize]; i += 1;
    name_buf[i] = HEX[(dev.bus & 0xf) as usize]; i += 1;
    name_buf[i] = b':'; i += 1;
    name_buf[i] = HEX[(dev.device >> 4) as usize]; i += 1;
    name_buf[i] = HEX[(dev.device & 0xf) as usize]; i += 1;
    name_buf[i] = b'.'; i += 1;
    name_buf[i] = HEX[(dev.function & 0xf) as usize]; i += 1;
    name_buf[i] = b':'; i += 1;

    for b in class_name.as_bytes() {
        name_buf[i] = *b;
        i += 1;
    }

    let name = &name_buf[..i];

    // Register with devd
    client.register_port(name, dev.port_type(), None)?;

    Ok(())
}

// ============================================================================
// Main
// ============================================================================

static mut REGISTRY: DeviceRegistry = DeviceRegistry {
    devices: [PciDevice {
        bus: 0, device: 0, function: 0,
        vendor_id: 0, device_id: 0,
        class: 0, subclass: 0, prog_if: 0,
        bar0_phys: 0, bar0_size: 0,
    }; 32],
    count: 0,
};

#[unsafe(no_mangle)]
fn main() {
    plog!("starting");

    // Detect platform
    let platform = Platform::detect();
    match platform {
        Platform::QemuVirt => plog!("platform: QEMU virt"),
        Platform::Mt7988a => plog!("platform: MT7988A"),
        Platform::Unknown => {
            plog!("unknown platform, exiting");
            syscall::exit(1);
        }
    }

    // Get ECAM base
    let ecam_base = match platform.ecam_base() {
        Some(base) => base,
        None => {
            plog!("no ECAM support for this platform");
            syscall::exit(1);
        }
    };

    plog!("ECAM base: {:#x}", ecam_base);

    // Map ECAM config space
    let ecam = match EcamAccess::new(ecam_base) {
        Some(e) => e,
        None => {
            plog!("failed to map ECAM");
            syscall::exit(1);
        }
    };

    // Enumerate devices
    unsafe {
        enumerate_ecam(&ecam, &mut REGISTRY);
        plog!("found {} devices", REGISTRY.count);
    }

    // Connect to devd
    let mut devd_client = match DevdClient::connect() {
        Ok(c) => c,
        Err(_) => {
            plog!("failed to connect to devd");
            syscall::exit(1);
        }
    };

    plog!("connected to devd");

    // Register our service port
    if let Err(_) = devd_client.register_port(b"pcie:", PortType::Service, None) {
        plog!("failed to register pcie: port");
    }

    // Enable bus mastering and register devices with devd
    unsafe {
        for dev in REGISTRY.iter() {
            // Enable bus master for devices that need DMA
            if dev.is_xhci() || dev.is_network() {
                enable_bus_master(&ecam, dev);
                plog!("enabled bus master for {:02x}:{:02x}.{}",
                    dev.bus, dev.device, dev.function);
            }

            // Register with devd
            if let Err(_) = register_device(&mut devd_client, dev) {
                plog!("failed to register device with devd");
            }
        }
    }

    // Report ready
    if let Err(_) = devd_client.report_state(userlib::devd::DriverState::Ready) {
        plog!("failed to report ready state");
    }

    plog!("ready, entering service loop");

    // Service loop - handle spawn commands from devd
    loop {
        // Poll for devd commands
        match devd_client.poll_command() {
            Ok(Some(cmd)) => {
                match cmd {
                    userlib::devd::DevdCommand::SpawnChild { seq_id, binary, binary_len, .. } => {
                        if let Ok(s) = core::str::from_utf8(&binary[..binary_len]) {
                            plog!("received spawn command for: {}", s);
                        }

                        // TODO: Spawn child driver
                        // For now, just ack with failure
                        let _ = devd_client.ack_spawn(seq_id, -1, 0);
                    }
                    userlib::devd::DevdCommand::StopChild { child_pid, .. } => {
                        plog!("stop child: {}", child_pid);
                    }
                    userlib::devd::DevdCommand::QueryInfo { seq_id } => {
                        let info = unsafe { REGISTRY.format_info() };
                        let info_len = info.iter().rposition(|&b| b != 0).map(|p| p + 1).unwrap_or(0);
                        let _ = devd_client.respond_info(seq_id, &info[..info_len]);
                    }
                }
            }
            Ok(None) => {
                // No command, sleep briefly
                syscall::sleep_ms(100);
            }
            Err(_) => {
                // Error polling, sleep and retry
                syscall::sleep_ms(100);
            }
        }
    }
}
