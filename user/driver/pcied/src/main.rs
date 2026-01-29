//! PCIe Bus Driver
//!
//! Enumerates PCIe devices via ECAM, creates per-device ports, and spawns
//! child drivers when requested by devd.
//!
//! Architecture:
//! 1. Connect to kernel's /kernel/bus/pcie0 to receive bus state
//! 2. Wait for Safe state before proceeding
//! 3. Enumerate PCI devices via ECAM
//! 4. Create a kernel Port for each device (e.g., pci/00:02.0:xhci)
//! 5. Register devices with devd
//! 6. When devd sends SpawnChild, spawn driver and send device info to child
//!
//! Supports:
//! - QEMU virt (ECAM at 0x4010000000)
//! - MT7988A (via platform-specific MAC access)

#![no_std]
#![no_main]

use userlib::syscall::{self, Handle, LogLevel};
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, KernelBusId, KernelBusInfo,
    KernelBusState, KernelBusChangeReason, bus_msg,
};
use userlib::bus_runtime::driver_main;
use userlib::devd::PortType;
use userlib::mmio::MmioRegion;
use userlib::ipc::{Port, ObjHandle};

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
        Platform::QemuVirt
    }

    fn ecam_base(&self) -> Option<u64> {
        match self {
            Platform::QemuVirt => Some(QEMU_ECAM_BASE),
            Platform::Mt7988a => None,
            Platform::Unknown => None,
        }
    }
}

// ============================================================================
// ECAM Constants
// ============================================================================

const QEMU_ECAM_BASE: u64 = 0x40_1000_0000;
const QEMU_ECAM_SIZE: u64 = 0x40_0000;

const QEMU_PCI_MMIO_BASE: u64 = 0x1000_0000;
const QEMU_PCI_MMIO_END: u64 = 0x3EFF_FFFF;

static mut BAR_ALLOC_NEXT: u64 = QEMU_PCI_MMIO_BASE;

const MAX_BUSES: u8 = 4;
const MAX_DEVICES: u8 = 32;
const MAX_FUNCTIONS: u8 = 8;

// ============================================================================
// PCI Configuration Space
// ============================================================================

mod pci_cfg {
    pub const VENDOR_DEVICE: usize = 0x00;
    pub const COMMAND_STATUS: usize = 0x04;
    pub const CLASS_REV: usize = 0x08;
    pub const HEADER_TYPE: usize = 0x0C;
    pub const BAR0: usize = 0x10;
}

mod pci_cmd {
    pub const MEMORY_SPACE: u16 = 0x0002;
    pub const BUS_MASTER: u16 = 0x0004;
}

mod pci_class {
    pub const BRIDGE: u8 = 0x06;
    pub const NETWORK: u8 = 0x02;
    pub const SERIAL_BUS: u8 = 0x0C;
    pub const MASS_STORAGE: u8 = 0x01;
}

mod pci_subclass {
    pub const USB: u8 = 0x03;
    pub const NVME: u8 = 0x08;
}

mod pci_prog_if {
    pub const XHCI: u8 = 0x30;
}

// ============================================================================
// ECAM Access
// ============================================================================

struct EcamAccess {
    mmio: MmioRegion,
}

impl EcamAccess {
    fn new(ecam_phys: u64) -> Option<Self> {
        let mmio = MmioRegion::open(ecam_phys, QEMU_ECAM_SIZE)?;
        Some(Self { mmio })
    }

    fn config_offset(bus: u8, device: u8, function: u8, reg: usize) -> usize {
        ((bus as usize) << 20) | ((device as usize) << 15) | ((function as usize) << 12) | reg
    }

    fn read32(&self, bus: u8, device: u8, function: u8, reg: usize) -> u32 {
        self.mmio.read32(Self::config_offset(bus, device, function, reg))
    }

    fn write32(&self, bus: u8, device: u8, function: u8, reg: usize, val: u32) {
        self.mmio.write32(Self::config_offset(bus, device, function, reg), val);
    }

    fn read16(&self, bus: u8, device: u8, function: u8, reg: usize) -> u16 {
        self.mmio.read16(Self::config_offset(bus, device, function, reg))
    }

    fn write16(&self, bus: u8, device: u8, function: u8, reg: usize, val: u16) {
        self.mmio.write16(Self::config_offset(bus, device, function, reg), val);
    }
}

// ============================================================================
// PCI Device
// ============================================================================

#[derive(Clone, Copy)]
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
    fn is_xhci(&self) -> bool {
        self.class == pci_class::SERIAL_BUS &&
        self.subclass == pci_subclass::USB &&
        self.prog_if == pci_prog_if::XHCI
    }

    fn is_network(&self) -> bool {
        self.class == pci_class::NETWORK
    }

    fn class_name(&self) -> &'static str {
        match (self.class, self.subclass, self.prog_if) {
            (pci_class::SERIAL_BUS, pci_subclass::USB, pci_prog_if::XHCI) => "xhci",
            (pci_class::NETWORK, _, _) => "network",
            (pci_class::MASS_STORAGE, pci_subclass::NVME, _) => "nvme",
            (pci_class::BRIDGE, _, _) => "bridge",
            _ => "unknown",
        }
    }

    fn is_nvme(&self) -> bool {
        self.class == pci_class::MASS_STORAGE && self.subclass == pci_subclass::NVME
    }

    fn port_type(&self) -> PortType {
        if self.is_xhci() {
            PortType::Usb
        } else if self.is_nvme() {
            PortType::Storage
        } else if self.is_network() {
            PortType::Network
        } else {
            PortType::Service
        }
    }

    fn format_port_name(&self, buf: &mut [u8; 32]) -> usize {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let class_name = self.class_name();

        let mut i = 0;
        for b in b"pci/" { buf[i] = *b; i += 1; }
        buf[i] = HEX[(self.bus >> 4) as usize]; i += 1;
        buf[i] = HEX[(self.bus & 0xf) as usize]; i += 1;
        buf[i] = b':'; i += 1;
        buf[i] = HEX[(self.device >> 4) as usize]; i += 1;
        buf[i] = HEX[(self.device & 0xf) as usize]; i += 1;
        buf[i] = b'.'; i += 1;
        buf[i] = HEX[(self.function & 0xf) as usize]; i += 1;
        buf[i] = b':'; i += 1;
        for b in class_name.as_bytes() {
            if i >= buf.len() { break; }
            buf[i] = *b;
            i += 1;
        }
        i
    }
}

// ============================================================================
// Device Registry
// ============================================================================

const MAX_PCI_DEVICES: usize = 32;

struct DeviceRegistry {
    devices: [PciDevice; MAX_PCI_DEVICES],
    count: usize,
}

impl DeviceRegistry {
    fn add(&mut self, dev: PciDevice) {
        if self.count < MAX_PCI_DEVICES {
            self.devices[self.count] = dev;
            self.count += 1;
        }
    }

    fn iter(&self) -> impl Iterator<Item = &PciDevice> {
        self.devices[..self.count].iter()
    }

    fn get(&self, idx: usize) -> Option<&PciDevice> {
        if idx < self.count { Some(&self.devices[idx]) } else { None }
    }
}

// ============================================================================
// Device Ports
// ============================================================================

/// Handle tags for device port events.
/// Port accept: TAG_PORT_BASE + port_index
/// Child channel: TAG_CHILD_BASE + port_index
const TAG_PORT_BASE: u32 = 0x1000;
const TAG_CHILD_BASE: u32 = 0x2000;

struct DevicePort {
    port: Port,
    dev_idx: usize,
    child: Option<ObjHandle>,
    info_sent: bool,
}

const MAX_DEVICE_PORTS: usize = 16;

struct DevicePorts {
    ports: [Option<DevicePort>; MAX_DEVICE_PORTS],
}

impl DevicePorts {
    const fn new() -> Self {
        Self {
            ports: [const { None }; MAX_DEVICE_PORTS],
        }
    }

    fn add(&mut self, port: Port, dev_idx: usize) -> Option<usize> {
        for (i, slot) in self.ports.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(DevicePort {
                    port,
                    dev_idx,
                    child: None,
                    info_sent: false,
                });
                return Some(i);
            }
        }
        None
    }
}

// ============================================================================
// Enumeration
// ============================================================================

fn enumerate_ecam(ecam: &EcamAccess, registry: &mut DeviceRegistry) {
    for bus in 0..MAX_BUSES {
        for device in 0..MAX_DEVICES {
            let vendor_device = ecam.read32(bus, device, 0, pci_cfg::VENDOR_DEVICE);
            let vendor_id = (vendor_device & 0xFFFF) as u16;

            if vendor_id == 0xFFFF || vendor_id == 0 { continue; }

            let header = ecam.read32(bus, device, 0, pci_cfg::HEADER_TYPE);
            let header_type = ((header >> 16) & 0xFF) as u8;
            let multi_function = (header_type & 0x80) != 0;
            let max_func = if multi_function { MAX_FUNCTIONS } else { 1 };

            for function in 0..max_func {
                if function > 0 {
                    let vd = ecam.read32(bus, device, function, pci_cfg::VENDOR_DEVICE);
                    if (vd & 0xFFFF) as u16 == 0xFFFF { continue; }
                }
                if let Some(dev) = probe_device(ecam, bus, device, function) {
                    registry.add(dev);
                }
            }
        }
    }
}

fn probe_device(ecam: &EcamAccess, bus: u8, device: u8, function: u8) -> Option<PciDevice> {
    let vendor_device = ecam.read32(bus, device, function, pci_cfg::VENDOR_DEVICE);
    let vendor_id = (vendor_device & 0xFFFF) as u16;
    let device_id = ((vendor_device >> 16) & 0xFFFF) as u16;

    if vendor_id == 0xFFFF || vendor_id == 0 { return None; }

    let class_rev = ecam.read32(bus, device, function, pci_cfg::CLASS_REV);
    let prog_if = ((class_rev >> 8) & 0xFF) as u8;
    let subclass = ((class_rev >> 16) & 0xFF) as u8;
    let class = ((class_rev >> 24) & 0xFF) as u8;

    let header = ecam.read32(bus, device, function, pci_cfg::HEADER_TYPE);
    let header_type = ((header >> 16) & 0x7F) as u8;
    if header_type != 0 { return None; }

    let (bar0_phys, bar0_size) = probe_bar(ecam, bus, device, function, pci_cfg::BAR0);

    Some(PciDevice {
        bus, device, function, vendor_id, device_id,
        class, subclass, prog_if, bar0_phys, bar0_size,
    })
}

fn probe_bar(ecam: &EcamAccess, bus: u8, device: u8, function: u8, bar_reg: usize) -> (u64, u64) {
    let original = ecam.read32(bus, device, function, bar_reg);
    if (original & 1) != 0 { return (0, 0); }

    let bar_type = (original >> 1) & 0x3;
    let is_64bit = bar_type == 2;

    ecam.write32(bus, device, function, bar_reg, 0xFFFFFFFF);
    let size_mask_low = ecam.read32(bus, device, function, bar_reg);

    let size_mask_high = if is_64bit {
        let orig_high = ecam.read32(bus, device, function, bar_reg + 4);
        ecam.write32(bus, device, function, bar_reg + 4, 0xFFFFFFFF);
        let mask_high = ecam.read32(bus, device, function, bar_reg + 4);
        ecam.write32(bus, device, function, bar_reg + 4, orig_high);
        mask_high
    } else {
        0xFFFFFFFF
    };

    ecam.write32(bus, device, function, bar_reg, original);

    if size_mask_low == 0 || size_mask_low == 0xFFFFFFFF { return (0, 0); }

    let full_mask = ((size_mask_high as u64) << 32) | (size_mask_low as u64);
    let size = !(full_mask & !0xF) + 1;

    let mut addr = (original & !0xF) as u64;
    if is_64bit {
        let high = ecam.read32(bus, device, function, bar_reg + 4);
        addr |= (high as u64) << 32;
    }

    if addr == 0 && size > 0 {
        unsafe {
            let align_mask = size - 1;
            BAR_ALLOC_NEXT = (BAR_ALLOC_NEXT + align_mask) & !align_mask;
            if BAR_ALLOC_NEXT + size <= QEMU_PCI_MMIO_END {
                addr = BAR_ALLOC_NEXT;
                BAR_ALLOC_NEXT += size;
                let bar_value = (addr as u32) | (original & 0xF);
                ecam.write32(bus, device, function, bar_reg, bar_value);
                if is_64bit {
                    ecam.write32(bus, device, function, bar_reg + 4, 0);
                }
                let cmd = ecam.read16(bus, device, function, pci_cfg::COMMAND_STATUS);
                if (cmd & pci_cmd::MEMORY_SPACE) == 0 {
                    ecam.write16(bus, device, function, pci_cfg::COMMAND_STATUS,
                                 cmd | pci_cmd::MEMORY_SPACE);
                }
            }
        }
    }

    (addr, size)
}

fn enable_bus_master(ecam: &EcamAccess, dev: &PciDevice) {
    let cmd = ecam.read16(dev.bus, dev.device, dev.function, pci_cfg::COMMAND_STATUS);
    if (cmd & pci_cmd::BUS_MASTER) == 0 {
        ecam.write16(dev.bus, dev.device, dev.function, pci_cfg::COMMAND_STATUS,
                     cmd | pci_cmd::BUS_MASTER);
    }
}

// ============================================================================
// Device Info Protocol
// ============================================================================

fn send_device_info(handle: ObjHandle, dev: &PciDevice) -> bool {
    let mut buf = [0u8; 12];
    buf[0..8].copy_from_slice(&dev.bar0_phys.to_le_bytes());
    buf[8..12].copy_from_slice(&(dev.bar0_size as u32).to_le_bytes());
    syscall::write(handle, &buf).is_ok()
}

// ============================================================================
// PCIe Driver (Bus Framework)
// ============================================================================

struct PcieDriver {
    platform: Platform,
    registry: DeviceRegistry,
    device_ports: DevicePorts,
    kernel_bus: Option<KernelBusId>,
    initialized: bool,
}

impl PcieDriver {
    const fn new() -> Self {
        Self {
            platform: Platform::Unknown,
            registry: DeviceRegistry {
                devices: [PciDevice {
                    bus: 0, device: 0, function: 0,
                    vendor_id: 0, device_id: 0,
                    class: 0, subclass: 0, prog_if: 0,
                    bar0_phys: 0, bar0_size: 0,
                }; MAX_PCI_DEVICES],
                count: 0,
            },
            device_ports: DevicePorts::new(),
            kernel_bus: None,
            initialized: false,
        }
    }

    fn handle_port_accept(&mut self, port_idx: usize) {
        let dp = match &mut self.device_ports.ports[port_idx] {
            Some(dp) => dp,
            None => return,
        };

        match dp.port.try_accept() {
            Some(channel) => {
                let handle = channel.handle();
                dp.child = Some(handle);
                dp.info_sent = false;

                if let Some(dev) = self.registry.get(dp.dev_idx) {
                    if send_device_info(handle, dev) {
                        dp.info_sent = true;
                    }
                }

                // Keep the channel handle alive — don't drop it
                core::mem::forget(channel);
            }
            None => {}
        }
    }

    fn handle_child_message(&mut self, port_idx: usize) {
        let dp = match &mut self.device_ports.ports[port_idx] {
            Some(dp) => dp,
            None => return,
        };

        if let Some(handle) = dp.child {
            let mut buf = [0u8; 64];
            match syscall::try_read(handle, &mut buf) {
                Ok(Some(0)) | Err(_) => {
                    // Child disconnected
                    let _ = syscall::close(handle);
                    dp.child = None;
                    dp.info_sent = false;
                }
                _ => {}
            }
        }
    }

    fn format_info(&self) -> [u8; 256] {
        use core::fmt::Write;

        let mut buf = [0u8; 256];
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
        let _ = writeln!(w, "PCIe Bus Driver (bus framework)");
        let _ = writeln!(w, "  ECAM Base: {:#x}", QEMU_ECAM_BASE);
        let _ = writeln!(w, "  Devices: {}", self.registry.count);
        for dev in self.registry.iter() {
            let _ = writeln!(w, "    {:02x}:{:02x}.{} {:04x}:{:04x} {} bar0={:#x}",
                dev.bus, dev.device, dev.function,
                dev.vendor_id, dev.device_id,
                dev.class_name(),
                dev.bar0_phys);
        }

        buf
    }
}

// ============================================================================
// Driver Trait Implementation
// ============================================================================

impl Driver for PcieDriver {
    fn bus_state_changed(
        &mut self,
        _bus: KernelBusId,
        _old_state: KernelBusState,
        new_state: KernelBusState,
        reason: KernelBusChangeReason,
        _ctx: &mut dyn BusCtx,
    ) {
        plog!("bus state → {:?} (reason: {:?})", new_state, reason);
    }

    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        plog!("starting");

        // Get bus path from spawn context, falling back to default
        let mut bus_path_buf = [0u8; 64];
        let bus_path_len;
        match ctx.spawn_context() {
            Ok(spawn_ctx) => {
                let name = spawn_ctx.port_name();
                bus_path_len = name.len().min(64);
                bus_path_buf[..bus_path_len].copy_from_slice(&name[..bus_path_len]);
            }
            Err(_) => {
                let default = b"/kernel/bus/pcie0";
                bus_path_len = default.len();
                bus_path_buf[..bus_path_len].copy_from_slice(default);
            }
        }
        let bus_path = &bus_path_buf[..bus_path_len];

        // Claim the kernel bus — handles the full protocol:
        // connect → StateSnapshot → wait Safe → SetDriver → Mux registration
        match ctx.claim_kernel_bus(bus_path) {
            Ok((bus_id, info)) => {
                plog!("kernel bus claimed (type={}, caps={:#x})", info.bus_type, info.capabilities);
                self.kernel_bus = Some(bus_id);
            }
            Err(_) => {
                plog!("kernel bus not available (QEMU?)");
            }
        }

        // Detect platform
        self.platform = Platform::detect();
        let ecam_base = match self.platform.ecam_base() {
            Some(base) => base,
            None => {
                plog!("no ECAM support");
                return Err(BusError::Internal);
            }
        };

        // Map ECAM
        let ecam = match EcamAccess::new(ecam_base) {
            Some(e) => e,
            None => {
                plog!("failed to map ECAM");
                return Err(BusError::ShmemError);
            }
        };

        // Enumerate PCI devices
        enumerate_ecam(&ecam, &mut self.registry);

        // Enable bus mastering
        for dev in self.registry.iter() {
            if dev.is_xhci() || dev.is_network() || dev.is_nvme() {
                enable_bus_master(&ecam, dev);
            }
        }

        // Create per-device ports and register with devd + Mux
        for idx in 0..self.registry.count {
            let dev = &self.registry.devices[idx];
            let mut name_buf = [0u8; 32];
            let name_len = dev.format_port_name(&mut name_buf);
            let name = &name_buf[..name_len];

            match Port::register(name) {
                Ok(port) => {
                    let port_handle = port.handle();
                    if let Some(port_idx) = self.device_ports.add(port, idx) {
                        // Watch port for incoming connections
                        let _ = ctx.watch_handle(port_handle, TAG_PORT_BASE + port_idx as u32);
                    }
                    let _ = ctx.register_port(name, dev.port_type(), None);
                }
                Err(_) => {
                    plog!("failed to create port for device {}", idx);
                }
            }
        }

        let platform_name = match self.platform {
            Platform::QemuVirt => "QEMU",
            Platform::Mt7988a => "MT7988A",
            Platform::Unknown => "?",
        };
        plog!("ready ({}, {} devices)", platform_name, self.registry.count);

        self.initialized = true;
        Ok(())
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        match msg.msg_type {
            bus_msg::QUERY_INFO => {
                let info = self.format_info();
                let len = info.iter().rposition(|&b| b != 0).map(|p| p + 1).unwrap_or(0);
                let _ = ctx.respond_info(msg.seq_id, &info[..len]);
                Disposition::Handled
            }
            _ => Disposition::Forward,
        }
    }

    fn handle_event(&mut self, tag: u32, handle: Handle, ctx: &mut dyn BusCtx) {
        if !self.initialized { return; }

        if tag >= TAG_CHILD_BASE {
            // Child channel message
            let port_idx = (tag - TAG_CHILD_BASE) as usize;
            self.handle_child_message(port_idx);
        } else if tag >= TAG_PORT_BASE {
            // Port accept event
            let port_idx = (tag - TAG_PORT_BASE) as usize;
            self.handle_port_accept(port_idx);

            // If a child was accepted, watch its channel too
            if let Some(ref dp) = self.device_ports.ports[port_idx] {
                if let Some(child_handle) = dp.child {
                    let _ = ctx.watch_handle(child_handle, TAG_CHILD_BASE + port_idx as u32);
                }
            }
        }
    }
}

// ============================================================================
// Main
// ============================================================================

static mut DRIVER: PcieDriver = PcieDriver::new();

#[unsafe(no_mangle)]
fn main() {
    let driver = unsafe { &mut *(&raw mut DRIVER) };
    driver_main(b"pcied", PcieDriverWrapper(driver));
}

struct PcieDriverWrapper(&'static mut PcieDriver);

impl Driver for PcieDriverWrapper {
    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.init(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn bus_state_changed(
        &mut self,
        bus: KernelBusId,
        old_state: KernelBusState,
        new_state: KernelBusState,
        reason: KernelBusChangeReason,
        ctx: &mut dyn BusCtx,
    ) {
        self.0.bus_state_changed(bus, old_state, new_state, reason, ctx)
    }

    fn handle_event(&mut self, tag: u32, handle: Handle, ctx: &mut dyn BusCtx) {
        self.0.handle_event(tag, handle, ctx)
    }
}
