//! PCIe Bus Driver
//!
//! Reads PCI device list from the kernel bus protocol and registers
//! per-device ports with devd. The kernel handles all ECAM/MAC access,
//! BAR probing, and BAR allocation. pcied is a thin policy layer.
//!
//! Architecture:
//! 1. Connect to kernel's /pcie:0 to receive bus state + device list
//! 2. Read device list from BusCtx::bus_devices() (delivered via bus protocol)
//! 3. Enable bus mastering for DMA-capable devices via kernel bus control
//! 4. Register each device as a devd port with BAR0 metadata
//! 5. devd rules match port types and spawn child drivers (nvmed, usbd, etc.)
//! 6. Child drivers read BAR0 info from spawn context metadata

#![no_std]
#![no_main]

use userlib::{uinfo, uerror};
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, KernelBusId, ConfigKey,
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

/// MediaTek WiFi HIF2 companion device IDs.
/// These are secondary PCIe functions managed through the primary HIF1 BAR.
/// The main driver (wifid) accesses HIF2 registers at 0xd8xxx via HIF1's MMIO space.
/// Registering them as separate ports would spawn duplicate driver instances that
/// race on shared WFSYS hardware (both do reset + DMA init on same registers).
mod mt_hif2 {
    pub const MT7996: u16 = 0x7991;
    pub const MT7992: u16 = 0x7993;
    pub const MT7990: u16 = 0x799b;
}

fn is_hif2_companion(vendor_id: u16, device_id: u16) -> bool {
    vendor_id == 0x14c3 && matches!(device_id, mt_hif2::MT7996 | mt_hif2::MT7992 | mt_hif2::MT7990)
}

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

/// Format port name as "class:index" (e.g., "nvme:0", "xhci:0")
fn format_port_name(cname: &str, index: u8, buf: &mut [u8; 32]) -> usize {
    let mut i = 0;
    for &b in cname.as_bytes() {
        if i >= buf.len() { break; }
        buf[i] = b;
        i += 1;
    }
    if i < buf.len() { buf[i] = b':'; i += 1; }
    if i < buf.len() { buf[i] = b'0' + index; i += 1; }
    i
}

// ============================================================================
// PCIe Driver (Bus Framework)
// ============================================================================

struct PcieDriver {
    devices: [userlib::BusDevice; MAX_PCI_DEVICES],
    count: usize,
    kernel_bus: Option<KernelBusId>,
    /// Per-class port index counters for class:index naming
    class_counters: [u8; 8], // indexed by ClassCounter enum
}

#[derive(Clone, Copy)]
#[repr(usize)]
enum ClassCounter {
    Nvme = 0,
    Xhci = 1,
    Wifi = 2,
    Network = 3,
    Bridge = 4,
    Unknown = 5,
}

fn class_counter_for(base_class: u8, subclass: u8, prog_if: u8) -> ClassCounter {
    match (base_class, subclass, prog_if) {
        (pci_class::MASS_STORAGE, pci_subclass::NVME, _) => ClassCounter::Nvme,
        (pci_class::SERIAL_BUS, pci_subclass::USB, pci_prog_if::XHCI) => ClassCounter::Xhci,
        (pci_class::NETWORK, pci_subclass::WIFI, _) => ClassCounter::Wifi,
        (pci_class::NETWORK, _, _) => ClassCounter::Network,
        (0x06, _, _) => ClassCounter::Bridge,
        _ => ClassCounter::Unknown,
    }
}

impl PcieDriver {
    const fn new() -> Self {
        Self {
            devices: [userlib::BusDevice::empty(); MAX_PCI_DEVICES],
            count: 0,
            kernel_bus: None,
            class_counters: [0; 8],
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
                let default = b"/pcie:0";
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
        // Reset class counters for deterministic naming
        self.class_counters = [0; 8];
        for idx in 0..self.count {
            let dev = &self.devices[idx];

            // Skip HIF2 companion devices — managed through primary driver's BAR
            if is_hif2_companion(dev.vendor_id, dev.device_id) {
                uinfo!("pcied", "skip_hif2"; device_id = dev.device_id);
                continue;
            }

            // Compute class:index port name
            let cname = class_name(dev.base_class(), dev.subclass(), dev.prog_if());
            let counter = class_counter_for(dev.base_class(), dev.subclass(), dev.prog_if());
            let cidx = self.class_counters[counter as usize];
            self.class_counters[counter as usize] = cidx + 1;

            let mut name_buf = [0u8; 32];
            let name_len = format_port_name(cname, cidx, &mut name_buf);
            let name = &name_buf[..name_len];

            // Build PortInfo with class/subclass and vendor/device IDs
            let (class, subclass) = port_class_subclass(
                dev.base_class(), dev.subclass(), dev.prog_if()
            );
            let mut info = PortInfo::new(name, class);
            info.port_subclass = subclass;
            info.vendor_id = dev.vendor_id;
            info.device_id = dev.device_id;

            // Encode BAR0 info into metadata:
            //   [0..8]  bar0_addr: u64 LE
            //   [8..12] bar0_size: u32 LE
            //   [12..16] bdf: u32 LE (bus<<8 | dev<<3 | fn)
            // Child drivers (usbd, nvmed, netd) read this to map MMIO and access PCI config
            let mut raw = [0u8; 24];
            raw[0..8].copy_from_slice(&dev.resource0.to_le_bytes());
            raw[8..12].copy_from_slice(&dev.resource1.to_le_bytes());
            let device_bdf = (dev.id & 0xFFFF) as u32;
            raw[12..16].copy_from_slice(&device_bdf.to_le_bytes());
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

const PCIE_CONFIG_KEYS: &[ConfigKey] = &[
    ConfigKey::read_only(b"devices"),
];

impl PcieDriverWrapper {
    fn fmt_hex16(val: u16, buf: &mut [u8]) -> usize {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        if buf.len() < 6 { return 0; }
        buf[0] = b'0'; buf[1] = b'x';
        buf[2] = HEX[((val >> 12) & 0xf) as usize];
        buf[3] = HEX[((val >> 8) & 0xf) as usize];
        buf[4] = HEX[((val >> 4) & 0xf) as usize];
        buf[5] = HEX[(val & 0xf) as usize];
        6
    }
}

impl Driver for PcieDriverWrapper {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        self.0.reset(ctx)
    }

    fn command(&mut self, msg: &BusMsg, ctx: &mut dyn BusCtx) -> Disposition {
        self.0.command(msg, ctx)
    }

    fn config_keys(&self) -> &[ConfigKey] {
        PCIE_CONFIG_KEYS
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        match key {
            b"devices" => {
                // Compact list: "class:N(vid:did) class:N(...) ..."
                let mut pos = 0;
                let mut counters = [0u8; 8];
                for dev in &self.0.devices[..self.0.count] {
                    if pos > 0 && pos < buf.len() { buf[pos] = b' '; pos += 1; }
                    // class:index
                    let cname = class_name(dev.base_class(), dev.subclass(), dev.prog_if());
                    let counter = class_counter_for(dev.base_class(), dev.subclass(), dev.prog_if());
                    let cidx = counters[counter as usize];
                    counters[counter as usize] = cidx + 1;
                    let mut name_buf = [0u8; 32];
                    let nlen = format_port_name(cname, cidx, &mut name_buf);
                    let copy = nlen.min(buf.len() - pos);
                    buf[pos..pos + copy].copy_from_slice(&name_buf[..copy]);
                    pos += copy;
                    // (class,vid:did)
                    if pos < buf.len() { buf[pos] = b'('; pos += 1; }
                    let cname = class_name(dev.base_class(), dev.subclass(), dev.prog_if());
                    let clen = cname.len().min(buf.len() - pos);
                    buf[pos..pos + clen].copy_from_slice(&cname.as_bytes()[..clen]);
                    pos += clen;
                    if pos < buf.len() { buf[pos] = b','; pos += 1; }
                    let n = Self::fmt_hex16(dev.vendor_id, &mut buf[pos..]);
                    pos += n;
                    if pos < buf.len() { buf[pos] = b':'; pos += 1; }
                    let n = Self::fmt_hex16(dev.device_id, &mut buf[pos..]);
                    pos += n;
                    if pos < buf.len() { buf[pos] = b')'; pos += 1; }
                }
                pos
            }
            _ => 0,
        }
    }
}
