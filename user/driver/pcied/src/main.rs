//! PCIe Bus Driver
//!
//! Reads PCI device list from the kernel bus protocol and registers
//! per-device ports with devd. The kernel handles all ECAM/MAC access,
//! BAR probing, and BAR allocation. pcied is a thin policy layer.
//!
//! Architecture:
//! 1. Connect to kernel's /kernel/bus/pcie0 to receive bus state + device list
//! 2. Read device list from BusCtx::bus_devices() (delivered via bus protocol)
//! 3. Enable bus mastering for DMA-capable devices via kernel bus control
//! 4. Register each device as a devd port with BAR0 metadata
//! 5. devd rules match port types and spawn child drivers (nvmed, usbd, etc.)
//! 6. Child drivers read BAR0 info from spawn context metadata

#![no_std]
#![no_main]

use userlib::{uinfo, uerror};
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, KernelBusId,
    bus_msg, PortInfo, PortClass, PortMetadata, port_subclass,
};
use userlib::bus_runtime::driver_main;

// ============================================================================
// PCI Class Constants
// ============================================================================

mod pci_class {
    pub const NETWORK: u8 = 0x02;
    pub const SERIAL_BUS: u8 = 0x0C;
    pub const MASS_STORAGE: u8 = 0x01;
}

mod pci_subclass {
    pub const USB: u8 = 0x03;
    pub const NVME: u8 = 0x08;
    pub const WIFI: u8 = 0x80;
}

mod pci_prog_if {
    pub const XHCI: u8 = 0x30;
}

// ============================================================================
// Device Classification (from BusDevice)
// ============================================================================

const MAX_PCI_DEVICES: usize = 32;

fn class_name(base_class: u8, subclass: u8, prog_if: u8) -> &'static str {
    match (base_class, subclass, prog_if) {
        (pci_class::SERIAL_BUS, pci_subclass::USB, pci_prog_if::XHCI) => "xhci",
        (pci_class::NETWORK, pci_subclass::WIFI, _) => "wifi",
        (pci_class::NETWORK, _, _) => "network",
        (pci_class::MASS_STORAGE, pci_subclass::NVME, _) => "nvme",
        (0x06, _, _) => "bridge",
        _ => "unknown",
    }
}

fn port_class_subclass(base_class: u8, subclass: u8, prog_if: u8) -> (PortClass, u16) {
    match (base_class, subclass, prog_if) {
        (pci_class::SERIAL_BUS, pci_subclass::USB, pci_prog_if::XHCI) => {
            (PortClass::Usb, port_subclass::USB_XHCI)
        }
        (pci_class::MASS_STORAGE, pci_subclass::NVME, _) => {
            (PortClass::StorageController, port_subclass::STORAGE_NVME)
        }
        (pci_class::NETWORK, pci_subclass::WIFI, _) => {
            (PortClass::Network, port_subclass::NET_WIFI)
        }
        (pci_class::NETWORK, _, _) => {
            (PortClass::Network, port_subclass::NET_ETHERNET)
        }
        _ => (PortClass::Service, 0),
    }
}

fn needs_bus_mastering(base_class: u8, subclass: u8, prog_if: u8) -> bool {
    matches!(
        (base_class, subclass, prog_if),
        (pci_class::SERIAL_BUS, pci_subclass::USB, pci_prog_if::XHCI)
        | (pci_class::MASS_STORAGE, pci_subclass::NVME, _)
        | (pci_class::NETWORK, _, _)
    )
}

fn format_port_name(dev: &userlib::BusDevice, buf: &mut [u8; 32]) -> usize {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let bus = dev.pci_bus();
    let device = dev.pci_device();
    let function = dev.pci_function();
    let cname = class_name(dev.base_class(), dev.subclass(), dev.prog_if());

    let mut i = 0;
    buf[i] = HEX[(bus >> 4) as usize]; i += 1;
    buf[i] = HEX[(bus & 0xf) as usize]; i += 1;
    buf[i] = b':'; i += 1;
    buf[i] = HEX[(device >> 4) as usize]; i += 1;
    buf[i] = HEX[(device & 0xf) as usize]; i += 1;
    buf[i] = b'.'; i += 1;
    buf[i] = HEX[(function & 0xf) as usize]; i += 1;
    buf[i] = b':'; i += 1;
    for b in cname.as_bytes() {
        if i >= buf.len() { break; }
        buf[i] = *b;
        i += 1;
    }
    i
}

// ============================================================================
// PCIe Driver (Bus Framework)
// ============================================================================

struct PcieDriver {
    devices: [userlib::BusDevice; MAX_PCI_DEVICES],
    count: usize,
    kernel_bus: Option<KernelBusId>,
}

impl PcieDriver {
    const fn new() -> Self {
        Self {
            devices: [userlib::BusDevice::empty(); MAX_PCI_DEVICES],
            count: 0,
            kernel_bus: None,
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
        let _ = writeln!(w, "PCIe Bus Driver (kernel-enumerated)");
        let _ = writeln!(w, "  Devices: {}", self.count);
        for dev in &self.devices[..self.count] {
            let cname = class_name(dev.base_class(), dev.subclass(), dev.prog_if());
            let _ = writeln!(w, "    {:02x}:{:02x}.{} {:04x}:{:04x} {} bar0={:#x}",
                dev.pci_bus(), dev.pci_device(), dev.pci_function(),
                dev.vendor_id, dev.device_id,
                cname,
                dev.resource0);
        }

        buf
    }
}

// ============================================================================
// Driver Trait Implementation
// ============================================================================

impl Driver for PcieDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        uinfo!("pcied", "starting";);

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

        // Claim the kernel bus — receives StateSnapshot + DeviceList
        match ctx.claim_kernel_bus(bus_path) {
            Ok((bus_id, info)) => {
                uinfo!("pcied", "bus_claimed"; bus_type = info.bus_type as u32, caps = userlib::ulog::hex32(info.capabilities as u32));
                self.kernel_bus = Some(bus_id);
            }
            Err(_) => {
                uerror!("pcied", "bus_unavailable";);
                return Err(BusError::Internal);
            }
        }

        // Read device list from bus protocol (delivered on connect)
        if let Some(bus_id) = self.kernel_bus {
            if let Some(bus_devs) = ctx.bus_devices(bus_id) {
                let count = bus_devs.len().min(MAX_PCI_DEVICES);
                self.devices[..count].copy_from_slice(&bus_devs[..count]);
                self.count = count;
                uinfo!("pcied", "pci_devices"; count = count as u32);
            } else {
                uinfo!("pcied", "no_devices";);
            }
        }

        // Enable bus mastering for DMA-capable devices via kernel bus control
        if let Some(bus_id) = self.kernel_bus {
            for idx in 0..self.count {
                let dev = &self.devices[idx];
                if needs_bus_mastering(dev.base_class(), dev.subclass(), dev.prog_if()) {
                    let device_bdf = (dev.id & 0xFFFF) as u16;
                    if let Err(_e) = ctx.enable_bus_mastering(bus_id, device_bdf) {
                        uerror!("pcied", "bus_master_failed"; bdf = dev.id);
                    }
                }
            }
        }

        // Register per-device ports with devd using unified PortInfo
        for idx in 0..self.count {
            let dev = &self.devices[idx];
            let mut name_buf = [0u8; 32];
            let name_len = format_port_name(dev, &mut name_buf);
            let name = &name_buf[..name_len];

            // Build PortInfo with class/subclass and vendor/device IDs
            let (class, subclass) = port_class_subclass(
                dev.base_class(), dev.subclass(), dev.prog_if()
            );
            let mut info = PortInfo::new(name, class);
            info.port_subclass = subclass;
            info.vendor_id = dev.vendor_id;
            info.device_id = dev.device_id;

            // Encode BAR0 info into metadata: [bar0_addr: u64 LE, bar0_size: u32 LE]
            // Child drivers (usbd, nvmed) read this to map MMIO registers
            let mut raw = [0u8; 24];
            raw[0..8].copy_from_slice(&dev.resource0.to_le_bytes());
            raw[8..12].copy_from_slice(&dev.resource1.to_le_bytes());
            info.metadata = PortMetadata { raw };

            let _ = ctx.register_port_with_info(&info, 0);

            uinfo!("pcied", "port_registered";
                name = core::str::from_utf8(name).unwrap_or("?"),
                bar0 = userlib::ulog::hex64(dev.resource0),
                size = userlib::ulog::hex32(dev.resource1));
        }

        uinfo!("pcied", "ready"; devices = self.count as u32);
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
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.reset(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }
}
