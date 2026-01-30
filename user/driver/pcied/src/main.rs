//! PCIe Bus Driver
//!
//! Reads PCI device list from the kernel's PCI registry and registers
//! per-device ports with devd. The kernel handles all ECAM/MAC access,
//! BAR probing, and BAR allocation. pcied is a thin policy layer.
//!
//! Architecture:
//! 1. Connect to kernel's /kernel/bus/pcie0 to receive bus state
//! 2. Wait for Safe state (kernel has already enumerated devices)
//! 3. Read PCI device list from kernel via pci_enumerate()
//! 4. Enable bus mastering for DMA-capable devices via kernel bus control
//! 5. Register each device as a devd port with BAR0 metadata
//! 6. devd rules match port types and spawn child drivers (nvmed, xhcid, etc.)
//! 7. Child drivers read BAR0 info from spawn context metadata

#![no_std]
#![no_main]

use userlib::syscall;
use userlib::{uinfo, uerror};
use userlib::bus::{
    BusMsg, BusError, BusCtx, Driver, Disposition, KernelBusId,
    KernelBusState, KernelBusChangeReason, bus_msg,
};
use userlib::bus_runtime::driver_main;
use userlib::devd::PortType;

// ============================================================================
// Logging — uses structured ulog macros from userlib
// ============================================================================

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
}

mod pci_prog_if {
    pub const XHCI: u8 = 0x30;
}

// ============================================================================
// Device Classification (from PciEnumEntry)
// ============================================================================

const MAX_PCI_DEVICES: usize = 32;

fn class_name(base_class: u8, subclass: u8, prog_if: u8) -> &'static str {
    match (base_class, subclass, prog_if) {
        (pci_class::SERIAL_BUS, pci_subclass::USB, pci_prog_if::XHCI) => "xhci",
        (pci_class::NETWORK, _, _) => "network",
        (pci_class::MASS_STORAGE, pci_subclass::NVME, _) => "nvme",
        (0x06, _, _) => "bridge",
        _ => "unknown",
    }
}

fn port_type(base_class: u8, subclass: u8, prog_if: u8) -> PortType {
    match (base_class, subclass, prog_if) {
        (pci_class::SERIAL_BUS, pci_subclass::USB, pci_prog_if::XHCI) => PortType::Usb,
        (pci_class::MASS_STORAGE, pci_subclass::NVME, _) => PortType::Storage,
        (pci_class::NETWORK, _, _) => PortType::Network,
        _ => PortType::Service,
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

fn format_port_name(entry: &syscall::PciEnumEntry, buf: &mut [u8; 32]) -> usize {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let bus = entry.bus();
    let device = entry.device();
    let function = entry.function();
    let cname = class_name(entry.base_class(), entry.subclass(), entry.prog_if());

    let mut i = 0;
    for b in b"pci/" { buf[i] = *b; i += 1; }
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

/// Build BAR0 metadata: [bar0_phys: u64 LE, bar0_size: u32 LE] = 12 bytes
fn bar0_metadata(entry: &syscall::PciEnumEntry) -> [u8; 12] {
    let mut meta = [0u8; 12];
    meta[0..8].copy_from_slice(&entry.bar0_addr.to_le_bytes());
    meta[8..12].copy_from_slice(&entry.bar0_size.to_le_bytes());
    meta
}

// ============================================================================
// PCIe Driver (Bus Framework)
// ============================================================================

struct PcieDriver {
    entries: [syscall::PciEnumEntry; MAX_PCI_DEVICES],
    count: usize,
    kernel_bus: Option<KernelBusId>,
}

impl PcieDriver {
    const fn new() -> Self {
        Self {
            entries: [syscall::PciEnumEntry::empty(); MAX_PCI_DEVICES],
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
        for entry in &self.entries[..self.count] {
            let cname = class_name(entry.base_class(), entry.subclass(), entry.prog_if());
            let _ = writeln!(w, "    {:02x}:{:02x}.{} {:04x}:{:04x} {} bar0={:#x}",
                entry.bus(), entry.device(), entry.function(),
                entry.vendor_id, entry.device_id,
                cname,
                entry.bar0_addr);
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
        _new_state: KernelBusState,
        _reason: KernelBusChangeReason,
        _ctx: &mut dyn BusCtx,
    ) {
        uinfo!("pcied", "bus_state_change";);
    }

    fn init(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
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

        // Claim the kernel bus — handles the full protocol:
        // connect -> StateSnapshot -> wait Safe -> SetDriver -> Mux registration
        match ctx.claim_kernel_bus(bus_path) {
            Ok((bus_id, info)) => {
                uinfo!("pcied", "bus_claimed"; bus_type = info.bus_type as u32, caps = userlib::ulog::hex32(info.capabilities as u32));
                self.kernel_bus = Some(bus_id);
            }
            Err(_) => {
                uerror!("pcied", "bus_unavailable";);
            }
        }

        // Read PCI device list from kernel registry
        match syscall::pci_enumerate(&mut self.entries) {
            Ok(count) => {
                self.count = count;
                uinfo!("pcied", "pci_devices"; count = count as u32);
            }
            Err(e) => {
                uerror!("pcied", "pci_enumerate_failed";);
                return Err(BusError::Internal);
            }
        }

        // Enable bus mastering for DMA-capable devices via kernel bus control
        if let Some(bus_id) = self.kernel_bus {
            for idx in 0..self.count {
                let entry = &self.entries[idx];
                if needs_bus_mastering(entry.base_class(), entry.subclass(), entry.prog_if()) {
                    let device_bdf = (entry.bdf & 0xFFFF) as u16;
                    if let Err(e) = ctx.enable_bus_mastering(bus_id, device_bdf) {
                        uerror!("pcied", "bus_master_failed"; bdf = entry.bdf);
                    }
                }
            }
        }

        // Register per-device ports with devd including BAR0 metadata
        for idx in 0..self.count {
            let entry = &self.entries[idx];
            let mut name_buf = [0u8; 32];
            let name_len = format_port_name(entry, &mut name_buf);
            let name = &name_buf[..name_len];

            // BAR0 metadata: [bar0_phys: u64 LE, bar0_size: u32 LE]
            let metadata = bar0_metadata(entry);
            let pt = port_type(entry.base_class(), entry.subclass(), entry.prog_if());
            let _ = ctx.register_port_with_metadata(name, pt, 0, None, &metadata);

            uinfo!("pcied", "port_registered";
                name = core::str::from_utf8(name).unwrap_or("?"),
                bar0 = userlib::ulog::hex64(entry.bar0_addr),
                size = userlib::ulog::hex32(entry.bar0_size));
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
}
