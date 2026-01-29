//! QEMU USB Driver
//!
//! USB driver for QEMU virt machine with standard xHCI.
//! Uses GenericSoc (no vendor-specific init) and the shared xhci library.
//!
//! This driver demonstrates the pattern for non-MT7988A USB controllers:
//! - No IPPC initialization (handled by firmware/QEMU)
//! - No PHY configuration (handled by firmware/QEMU)
//! - Pure xHCI operation
//!
//! ## Note on QEMU xHCI
//!
//! QEMU's qemu-xhci device is a PCI device. Run QEMU with:
//! ```
//! qemu-system-aarch64 ... -device qemu-xhci,id=xhci -device usb-storage,drive=mydrive
//! ```
//!
//! The xHCI controller appears at a PCI BAR address assigned by QEMU firmware.

#![no_std]
#![no_main]

mod transport;
mod bot;
mod xhci_bulk;
mod block;
mod partition;
mod ring_proto;
// Block info for DataPort geometry responses + USB hardware parameters
#[derive(Clone, Copy)]
struct BlockInfo {
    block_size: u32,
    block_count: u64,
    // USB hardware parameters for xHCI communication
    slot_id: u8,
    bulk_in_dci: u8,
    bulk_out_dci: u8,
}

impl BlockInfo {
    fn empty() -> Self {
        Self {
            block_size: 512,
            block_count: 0,
            slot_id: 0,
            bulk_in_dci: 0,
            bulk_out_dci: 0,
        }
    }
    fn is_valid(&self) -> bool {
        self.block_count > 0 && self.slot_id > 0
    }
}

use userlib::syscall;
use userlib::Channel;
use userlib::devd::{
    DevdClient, PortType, DeviceClass, DeviceInfo, DriverState,
    SpawnHandler, SpawnFilter, run_driver_loop,
};
use userlib::data_port::{DataPort, DataPortConfig};
use userlib::ring::{IoSqe, IoCqe, io_op, io_status};
use usb::{MmioRegion, DmaPool, XhciController, XhciCaps};
use usb::soc::{GenericSoc, SocUsb};
use usb::ring::{Ring, EventRing, ErstEntry};
use usb::trb::{Trb, trb_type};
use usb::xhci::op as xhci_op;
use usb::transfer::dsb;
use usb::enumeration::{
    device_speed, build_enable_slot_trb, build_address_device_trb,
    build_configure_endpoint_trb, init_slot_context, init_ep0_context,
    init_input_control_for_address, default_max_packet_size, endpoint_address_to_dci,
};
use usb::usb::{SlotContext, EndpointContext, InputControlContext, usb_req, DeviceDescriptor};
use usb::transfer::{SetupPacket, Direction, build_setup_trb, build_data_trb, build_status_trb};
use usb::msc::msc;

// =============================================================================
// PCI Device Discovery (via pcied spawn context)
// =============================================================================

/// xHCI device info from pcied
struct XhciPciDevice {
    pub bar0: u64,
    pub bar_size: usize,
}

/// Get xHCI device info via spawn context
///
/// Flow:
/// 1. Get spawn context from devd (port name like "pci/00:02.0:xhci")
/// 2. Connect to that port (pcied's device port)
/// 3. Receive device info: bar0_phys(8) + bar0_size(4) = 12 bytes
fn get_device_from_spawn_context(devd: &mut DevdClient) -> Option<XhciPciDevice> {
    // Step 1: Get spawn context from devd
    let (port_name, port_len) = match devd.get_spawn_context() {
        Ok(Some((name, len, _ptype))) => (name, len),
        Ok(None) => {
            log("[xhcid] No spawn context");
            return None;
        }
        Err(_) => {
            log("[xhcid] Spawn context error");
            return None;
        }
    };

    // Step 2: Connect to pcied's device port
    let mut pcie_channel = match Channel::connect(&port_name[..port_len]) {
        Ok(ch) => ch,
        Err(_) => {
            log("[xhcid] Failed to connect to device port");
            return None;
        }
    };

    // Step 3: Receive device info from pcied
    // recv() blocks until data arrives or peer closes
    // Protocol: bar0_phys(8) + bar0_size(4) = 12 bytes
    let mut buf = [0u8; 12];
    match pcie_channel.recv(&mut buf) {
        Ok(len) if len >= 12 => {
            let bar0 = u64::from_le_bytes([
                buf[0], buf[1], buf[2], buf[3],
                buf[4], buf[5], buf[6], buf[7],
            ]);
            let bar_size = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]) as usize;

            if bar0 == 0 || bar_size == 0 {
                log("[xhcid] Invalid BAR (zero)");
                return None;
            }

            Some(XhciPciDevice { bar0, bar_size })
        }
        Ok(_) => {
            log("[xhcid] Short device info response");
            None
        }
        Err(_) => {
            log("[xhcid] Device info recv failed");
            None
        }
    }
}

// =============================================================================
// Memory Layout
// =============================================================================

/// Memory layout for xHCI structures in DMA pool
///
/// Layout (64KB total):
///   0x0000-0x07FF: DCBAA (256 entries * 8 bytes = 2KB)
///   0x0800-0x080F: ERST (1 entry * 16 bytes)
///   0x1000-0x1FFF: Command Ring (64 TRBs * 16 = 1KB)
///   0x2000-0x2FFF: Event Ring (64 TRBs * 16 = 1KB)
///   0x3000-0x3FFF: Scratchpad (for control transfers)
///   0x4000+: Per-device contexts (4KB each, up to 8 devices)
///
#[allow(dead_code)]
mod layout {
    /// DCBAA: Device Context Base Address Array
    /// 256 entries * 8 bytes = 2KB
    pub const DCBAA_OFFSET: usize = 0x0000;
    pub const DCBAA_SIZE: usize = 256 * 8;

    /// ERST: Event Ring Segment Table
    /// 1 entry * 16 bytes = 16 bytes (at offset 0x800)
    pub const ERST_OFFSET: usize = 0x0800;
    pub const ERST_SIZE: usize = 16;

    /// Command Ring: 64 TRBs * 16 bytes = 1KB
    pub const CMD_RING_OFFSET: usize = 0x1000;
    pub const CMD_RING_SIZE: usize = 64 * 16;

    /// Event Ring: 64 TRBs * 16 bytes = 1KB
    pub const EVT_RING_OFFSET: usize = 0x2000;
    pub const EVT_RING_SIZE: usize = 64 * 16;

    /// Scratchpad for control transfer data (descriptors, etc.)
    pub const SCRATCH_OFFSET: usize = 0x3000;
    pub const SCRATCH_SIZE: usize = 0x1000;  // 4KB

    /// Per-device context base (4KB per device)
    pub const DEVICE_CTX_BASE: usize = 0x4000;
    pub const DEVICE_CTX_SIZE: usize = 0x1000;  // 4KB per device
    pub const MAX_DEVICES: usize = 8;

    /// Total DMA pool size needed
    pub const TOTAL_SIZE: usize = DEVICE_CTX_BASE + (DEVICE_CTX_SIZE * MAX_DEVICES);  // 64KB

    /// Per-device context offsets (within each 4KB device page)
    pub const DEV_INPUT_CTX: usize = 0x000;    // Input context (1056 bytes)
    pub const DEV_OUTPUT_CTX: usize = 0x500;   // Device context (1024 bytes)
    pub const DEV_EP0_RING: usize = 0x900;     // EP0 transfer ring (1KB)
}

// =============================================================================
// Device Tracking
// =============================================================================

/// Information about an enumerated USB device
#[derive(Clone, Copy, Default)]
struct UsbDevice {
    /// Port number (1-indexed)
    port: u8,
    /// Slot ID assigned by xHCI
    slot_id: u8,
    /// Device speed
    speed: u8,
    /// Is this device an MSC device?
    is_msc: bool,
    /// Bulk IN endpoint DCI (0 if none)
    bulk_in_dci: u8,
    /// Bulk OUT endpoint DCI (0 if none)
    bulk_out_dci: u8,
    /// Max packet size for bulk endpoints
    bulk_max_packet: u16,
    /// Vendor ID
    vendor_id: u16,
    /// Product ID
    product_id: u16,
    /// EP0 transfer ring (per-device, needs tracking for control transfers)
    ep0_ring_cycle: bool,
    ep0_ring_enqueue: u8,
}

/// xHCI command completion result
#[derive(Clone, Copy, Debug)]
struct CommandResult {
    slot_id: u8,
    completion_code: u32,
    param: u64,
}

// =============================================================================
// Driver State
// =============================================================================

/// QEMU USB driver instance
struct QemuUsbDriver {
    /// SoC wrapper (generic, no vendor init)
    soc: GenericSoc,
    /// xHCI controller (owns MMIO region)
    xhci: Option<XhciController>,
    /// xHCI capabilities
    caps: Option<XhciCaps>,
    /// DMA memory pool
    dma: Option<DmaPool>,
    /// Command ring
    cmd_ring: Option<Ring>,
    /// Event ring
    evt_ring: Option<EventRing>,
    /// xHCI base address (physical)
    xhci_base: u64,
    /// xHCI region size
    xhci_size: usize,
    /// Tracked devices (indexed by slot_id - 1)
    devices: [Option<UsbDevice>; layout::MAX_DEVICES],
    /// Command tag counter (for MSC operations)
    cmd_tag: u32,
    /// First FAT partition info (for block service)
    partition_info: BlockInfo,
    /// Bulk OUT ring state (persists between calls)
    bulk_out_enqueue: usize,
    bulk_out_cycle: bool,
    /// Bulk IN ring state (persists between calls)
    bulk_in_enqueue: usize,
    bulk_in_cycle: bool,
}

impl QemuUsbDriver {
    /// Create a new driver instance from discovered PCI device
    fn new(xhci_base: u64, xhci_size: usize) -> Self {
        let soc = GenericSoc::new(xhci_base, xhci_size, 0);
        Self {
            soc,
            xhci: None,
            caps: None,
            dma: None,
            cmd_ring: None,
            evt_ring: None,
            xhci_base,
            xhci_size,
            devices: [None; layout::MAX_DEVICES],
            cmd_tag: 1,
            partition_info: BlockInfo::empty(),
            bulk_out_enqueue: 0,
            bulk_out_cycle: true,
            bulk_in_enqueue: 0,
            bulk_in_cycle: true,
        }
    }

    /// Format info text for introspection queries
    fn format_info(&self) -> [u8; 1024] {
        let mut buf = [0u8; 1024];
        let mut pos = 0;

        let append = |buf: &mut [u8], pos: &mut usize, s: &[u8]| {
            let len = s.len().min(buf.len() - *pos);
            buf[*pos..*pos + len].copy_from_slice(&s[..len]);
            *pos += len;
        };

        append(&mut buf, &mut pos, b"USB Host Controller (xHCI)\n");

        // Controller info
        if let Some(caps) = &self.caps {
            append(&mut buf, &mut pos, b"  Max Slots: ");
            let mut num_buf = [0u8; 16];
            let num_len = format_u32(&mut num_buf, caps.max_slots as u32);
            append(&mut buf, &mut pos, &num_buf[..num_len]);
            append(&mut buf, &mut pos, b"\n  Max Ports: ");
            let num_len = format_u32(&mut num_buf, caps.max_ports as u32);
            append(&mut buf, &mut pos, &num_buf[..num_len]);
            append(&mut buf, &mut pos, b"\n");
        }

        // Count active devices
        let mut device_count = 0;
        for dev in &self.devices {
            if dev.is_some() {
                device_count += 1;
            }
        }

        append(&mut buf, &mut pos, b"\nDevices: ");
        let mut num_buf = [0u8; 16];
        let num_len = format_u32(&mut num_buf, device_count);
        append(&mut buf, &mut pos, &num_buf[..num_len]);
        append(&mut buf, &mut pos, b"\n");

        // Device details
        for (i, dev_opt) in self.devices.iter().enumerate() {
            if let Some(dev) = dev_opt {
                append(&mut buf, &mut pos, b"  Slot ");
                let num_len = format_u32(&mut num_buf, dev.slot_id as u32);
                append(&mut buf, &mut pos, &num_buf[..num_len]);
                append(&mut buf, &mut pos, b": ");

                if dev.is_msc {
                    append(&mut buf, &mut pos, b"USB Mass Storage");
                } else {
                    append(&mut buf, &mut pos, b"USB Device");
                }

                append(&mut buf, &mut pos, b"\n    Port: ");
                let num_len = format_u32(&mut num_buf, dev.port as u32);
                append(&mut buf, &mut pos, &num_buf[..num_len]);

                append(&mut buf, &mut pos, b", Speed: ");
                let speed_name = match dev.speed {
                    1 => b"Full-Speed" as &[u8],
                    2 => b"Low-Speed",
                    3 => b"High-Speed",
                    4 => b"SuperSpeed",
                    _ => b"Unknown",
                };
                append(&mut buf, &mut pos, speed_name);

                append(&mut buf, &mut pos, b"\n    VID:PID: 0x");
                let num_len = format_hex_16(&mut num_buf, dev.vendor_id);
                append(&mut buf, &mut pos, &num_buf[..num_len]);
                append(&mut buf, &mut pos, b":0x");
                let num_len = format_hex_16(&mut num_buf, dev.product_id);
                append(&mut buf, &mut pos, &num_buf[..num_len]);

                if dev.is_msc {
                    append(&mut buf, &mut pos, b"\n    Bulk IN DCI: ");
                    let num_len = format_u32(&mut num_buf, dev.bulk_in_dci as u32);
                    append(&mut buf, &mut pos, &num_buf[..num_len]);
                    append(&mut buf, &mut pos, b", Bulk OUT DCI: ");
                    let num_len = format_u32(&mut num_buf, dev.bulk_out_dci as u32);
                    append(&mut buf, &mut pos, &num_buf[..num_len]);
                }

                append(&mut buf, &mut pos, b"\n");
            }
        }

        buf
    }

    /// Initialize the USB controller
    fn init(&mut self) -> Result<(), &'static str> {
        log("[qemu-usbd] Starting initialization");

        // Step 1: SoC pre-init (no-op for GenericSoc)
        self.soc.pre_init().map_err(|_| "SoC pre-init failed")?;

        // Step 2: Map xHCI MMIO region
        log("[qemu-usbd] Mapping xHCI MMIO");
        let mmio = MmioRegion::open(self.xhci_base, self.xhci_size as u64)
            .ok_or("Failed to map xHCI MMIO")?;

        // Step 3: Create controller (takes ownership of mmio) and read capabilities
        log("[qemu-usbd] Reading xHCI capabilities");
        let mut xhci = XhciController::new(mmio);
        let caps = xhci.read_capabilities().ok_or("Failed to read capabilities")?;

        log_val("[qemu-usbd] Max slots: ", caps.max_slots as u64);
        log_val("[qemu-usbd] Max ports: ", caps.max_ports as u64);
        log_hex("[qemu-usbd] Version: ", caps.version as u64);

        self.caps = Some(caps);

        // Step 4: Halt and reset controller
        log("[qemu-usbd] Halting controller");
        let (halt_ok, reset_ok) = xhci.halt_and_reset();
        if !halt_ok {
            return Err("Failed to halt controller");
        }
        if !reset_ok {
            return Err("Failed to reset controller");
        }
        usb::delay_ms(50);  // Let controller stabilize

        self.xhci = Some(xhci);

        // Step 5: Allocate DMA memory
        log("[qemu-usbd] Allocating DMA memory");
        let dma = DmaPool::alloc(layout::TOTAL_SIZE)
            .ok_or("Failed to allocate DMA memory")?;

        log_hex("[qemu-usbd] DMA vaddr: ", dma.vaddr());
        log_hex("[qemu-usbd] DMA paddr: ", dma.paddr());

        self.dma = Some(dma);

        // Step 6: Initialize data structures
        self.init_data_structures()?;

        // Step 7: Program xHCI registers
        self.program_registers()?;

        // Step 8: Start controller
        self.start_controller()?;

        log("[qemu-usbd] Initialization complete");
        Ok(())
    }

    /// Initialize DMA data structures (DCBAA, rings)
    fn init_data_structures(&mut self) -> Result<(), &'static str> {
        let dma = self.dma.as_mut().ok_or("No DMA memory")?;
        let vbase = dma.vaddr();
        let pbase = dma.paddr();

        // Zero all memory first
        dma.zero();

        // Initialize DCBAA (already zeroed)
        log("[qemu-usbd] Initializing DCBAA");

        // Initialize Command Ring
        log("[qemu-usbd] Initializing Command Ring");
        let cmd_ring_virt = (vbase + layout::CMD_RING_OFFSET as u64) as *mut Trb;
        let cmd_ring_phys = pbase + layout::CMD_RING_OFFSET as u64;
        let cmd_ring = Ring::init(cmd_ring_virt, cmd_ring_phys);
        self.cmd_ring = Some(cmd_ring);

        // Initialize Event Ring and ERST
        log("[qemu-usbd] Initializing Event Ring");
        let evt_ring_virt = (vbase + layout::EVT_RING_OFFSET as u64) as *mut Trb;
        let evt_ring_phys = pbase + layout::EVT_RING_OFFSET as u64;
        let erst_virt = (vbase + layout::ERST_OFFSET as u64) as *mut ErstEntry;
        let erst_phys = pbase + layout::ERST_OFFSET as u64;

        // Get USBSTS address for event ring (needed for EHB clearing)
        let xhci = self.xhci.as_ref().ok_or("No xHCI controller")?;
        let usbsts_addr = xhci.mmio_base() + xhci.op_offset() as u64 + xhci_op::USBSTS as u64;

        let evt_ring = EventRing::init(evt_ring_virt, evt_ring_phys, erst_virt, erst_phys, usbsts_addr);
        self.evt_ring = Some(evt_ring);

        // Flush all structures to memory
        dsb();

        Ok(())
    }

    /// Program xHCI registers
    fn program_registers(&mut self) -> Result<(), &'static str> {
        let xhci = self.xhci.as_mut().ok_or("No xHCI controller")?;
        let caps = self.caps.as_ref().ok_or("No capabilities")?;
        let dma = self.dma.as_ref().ok_or("No DMA memory")?;

        let pbase = dma.paddr();
        let dcbaa_phys = pbase + layout::DCBAA_OFFSET as u64;
        let erst_phys = pbase + layout::ERST_OFFSET as u64;
        let cmd_ring_phys = pbase + layout::CMD_RING_OFFSET as u64;
        let evt_ring_phys = pbase + layout::EVT_RING_OFFSET as u64;

        // Set max slots
        log_val("[qemu-usbd] Setting max slots: ", caps.max_slots as u64);
        xhci.set_max_slots(caps.max_slots);

        // Set DCBAAP
        log_hex("[qemu-usbd] Setting DCBAAP: ", dcbaa_phys);
        xhci.set_dcbaap(dcbaa_phys);

        // Set CRCR (Command Ring Control Register)
        log_hex("[qemu-usbd] Setting CRCR: ", cmd_ring_phys);
        xhci.set_crcr(cmd_ring_phys, true);  // cycle = true

        // Program interrupter 0
        log("[qemu-usbd] Programming interrupter 0");
        xhci.program_interrupter(0, erst_phys, 1, evt_ring_phys);

        Ok(())
    }

    /// Start the xHCI controller
    fn start_controller(&mut self) -> Result<(), &'static str> {
        let xhci = self.xhci.as_mut().ok_or("No xHCI controller")?;
        let caps = self.caps.as_ref().ok_or("No capabilities")?;

        // Start controller
        log("[qemu-usbd] Starting controller");
        xhci.start();
        usb::delay_ms(10);

        if !xhci.is_running() {
            return Err("Controller failed to start");
        }
        log("[qemu-usbd] Controller running");

        // Power on all ports
        log_val("[qemu-usbd] Powering on ports: ", caps.max_ports as u64);
        xhci.power_on_all_ports();
        usb::delay_ms(100);  // Let ports stabilize

        // Check port status
        for port in 0..caps.max_ports {
            let connected = xhci.port_connected(port);
            let enabled = xhci.port_enabled(port);
            if connected {
                log_val("[qemu-usbd] Port connected: ", port as u64);
                if enabled {
                    log_val("[qemu-usbd] Port enabled: ", port as u64);
                }
            }
        }

        Ok(())
    }

    /// Poll for events and handle device connections
    fn poll_events(&mut self) {
        let xhci = match self.xhci.as_mut() {
            Some(x) => x,
            None => return,
        };
        let caps = match self.caps.as_ref() {
            Some(c) => c,
            None => return,
        };
        let evt_ring = match self.evt_ring.as_mut() {
            Some(e) => e,
            None => return,
        };

        // Check for port status change events
        for port in 0..caps.max_ports {
            if xhci.port_connected(port) {
                let speed = xhci.port_speed(port);
                log_val("[qemu-usbd] Device on port ", port as u64);
                log_val("[qemu-usbd]   Speed: ", speed as u64);
            }
        }

        // Process event ring
        while let Some(trb) = evt_ring.dequeue() {
            let trb_type_val = trb.get_type();
            let cc = trb.event_completion_code();

            match trb_type_val {
                // Port Status Change Event
                trb_type::PORT_STATUS_CHANGE => {
                    let port_id = ((trb.control >> 24) & 0xFF) as u32;
                    log_val("[qemu-usbd] Port status change: ", port_id as u64);
                }
                // Command Completion Event
                trb_type::COMMAND_COMPLETION => {
                    log_val("[qemu-usbd] Command completion, cc=", cc as u64);
                }
                // Transfer Event
                trb_type::TRANSFER_EVENT => {
                    log_val("[qemu-usbd] Transfer event, cc=", cc as u64);
                }
                _ => {
                    log_val("[qemu-usbd] Event type: ", trb_type_val as u64);
                }
            }
        }

        // Update ERDP
        xhci.update_erdp(0, evt_ring.erdp());
    }

    // =========================================================================
    // Device Enumeration
    // =========================================================================

    /// Send a command and wait for completion
    fn send_command(&mut self, trb: Trb) -> Option<CommandResult> {
        let xhci = self.xhci.as_mut()?;
        let cmd_ring = self.cmd_ring.as_mut()?;
        let evt_ring = self.evt_ring.as_mut()?;

        // Enqueue command
        cmd_ring.enqueue(&trb);
        dsb();

        // Ring doorbell 0 (host controller)
        xhci.ring_doorbell(0, 0);

        // Wait for completion (poll event ring)
        for _ in 0..1000 {
            usb::delay_ms(1);

            while let Some(evt) = evt_ring.dequeue() {
                if evt.get_type() == trb_type::COMMAND_COMPLETION {
                    let cc = evt.event_completion_code();
                    let slot_id = ((evt.control >> 24) & 0xFF) as u8;

                    xhci.update_erdp(0, evt_ring.erdp());

                    return Some(CommandResult {
                        slot_id,
                        completion_code: cc,
                        param: evt.param,
                    });
                }
            }
            xhci.update_erdp(0, evt_ring.erdp());
        }

        log("[qemu-usbd] Command timeout");
        None
    }

    /// Stop an endpoint (required before Set TR Dequeue Pointer)
    fn stop_endpoint(&mut self, slot_id: u8, dci: u8) -> bool {
        // Stop Endpoint command: type 15
        let trb = Trb {
            param: 0,
            status: 0,
            control: ((slot_id as u32) << 24) | ((dci as u32) << 16) | (trb_type::STOP_ENDPOINT << 10) | 1,
        };

        match self.send_command(trb) {
            Some(result) if result.completion_code == 1 => true,  // Success
            Some(result) if result.completion_code == 15 => true, // Context State Error (already stopped)
            _ => false,
        }
    }

    /// Set the TR Dequeue Pointer for an endpoint
    fn set_tr_dequeue_pointer(&mut self, slot_id: u8, dci: u8, ring_phys: u64, dcs: bool) -> bool {
        // Set TR Dequeue Pointer command: type 16
        // param = New TR Dequeue Pointer | DCS (bit 0)
        let param = ring_phys | if dcs { 1 } else { 0 };
        let trb = Trb {
            param,
            status: 0,
            control: ((slot_id as u32) << 24) | ((dci as u32) << 16) | (trb_type::SET_TR_DEQUEUE << 10) | 1,
        };

        match self.send_command(trb) {
            Some(result) if result.completion_code == 1 => true,  // Success
            _ => false,
        }
    }

    /// Enumerate a device on a port
    fn enumerate_device(&mut self, port: u8) -> Option<u8> {
        let speed = {
            let xhci = self.xhci.as_ref()?;
            if !xhci.port_connected(port) {
                return None;
            }
            xhci.port_speed(port)
        };

        log_val("[qemu-usbd] Enumerating port ", port as u64);
        log_val("[qemu-usbd]   Speed: ", speed as u64);

        // Step 1: Enable Slot
        let enable_slot = build_enable_slot_trb();
        let result = self.send_command(enable_slot)?;

        if result.completion_code != 1 {  // 1 = Success
            log_val("[qemu-usbd]   EnableSlot failed, cc=", result.completion_code as u64);
            return None;
        }

        let slot_id = result.slot_id;
        log_val("[qemu-usbd]   Slot ID: ", slot_id as u64);

        // Check slot is valid
        if slot_id == 0 || slot_id as usize > layout::MAX_DEVICES {
            log("[qemu-usbd]   Invalid slot ID");
            return None;
        }

        // Step 2: Set up device context in DMA memory
        let dma = self.dma.as_ref()?;
        let vbase = dma.vaddr();
        let pbase = dma.paddr();

        let dev_offset = layout::DEVICE_CTX_BASE + ((slot_id as usize - 1) * layout::DEVICE_CTX_SIZE);
        let dev_vaddr = vbase + dev_offset as u64;
        let dev_paddr = pbase + dev_offset as u64;

        // Clear device context area
        unsafe {
            core::ptr::write_bytes(dev_vaddr as *mut u8, 0, layout::DEVICE_CTX_SIZE);
        }

        // Initialize input control context
        let icc = unsafe { &mut *((dev_vaddr + layout::DEV_INPUT_CTX as u64) as *mut InputControlContext) };
        init_input_control_for_address(icc);

        // Initialize slot context
        let slot_ctx = unsafe { &mut *((dev_vaddr + layout::DEV_INPUT_CTX as u64 + 32) as *mut SlotContext) };
        let xhci_speed = match speed {
            1 => device_speed::LOW,
            2 => device_speed::FULL,
            3 => device_speed::HIGH,
            4 => device_speed::SUPER,
            5 => device_speed::SUPER_PLUS,
            _ => device_speed::FULL,
        };
        init_slot_context(slot_ctx, xhci_speed, 0, (port + 1) as u32, 1, 0, 0);

        // Initialize EP0 context
        let ep0_ctx = unsafe { &mut *((dev_vaddr + layout::DEV_INPUT_CTX as u64 + 64) as *mut EndpointContext) };
        let ep0_ring_phys = dev_paddr + layout::DEV_EP0_RING as u64;
        let max_packet = default_max_packet_size(xhci_speed);
        init_ep0_context(ep0_ctx, max_packet, ep0_ring_phys | 1);  // | 1 for cycle bit

        // Initialize EP0 transfer ring (link TRB at end)
        let ep0_ring_vaddr = dev_vaddr + layout::DEV_EP0_RING as u64;
        unsafe {
            // Create link TRB at position 63
            let link_trb = (ep0_ring_vaddr as *mut Trb).add(63);
            (*link_trb).param = ep0_ring_phys;
            (*link_trb).status = 0;
            (*link_trb).control = (trb_type::LINK << 10) | 1;  // Toggle cycle
        }

        // Set device context pointer in DCBAA
        let dcbaa_ptr = (vbase + layout::DCBAA_OFFSET as u64) as *mut u64;
        let output_ctx_phys = dev_paddr + layout::DEV_OUTPUT_CTX as u64;
        unsafe {
            dcbaa_ptr.add(slot_id as usize).write_volatile(output_ctx_phys);
        }
        dsb();

        // Step 3: Address Device command
        let input_ctx_phys = dev_paddr + layout::DEV_INPUT_CTX as u64;
        let addr_cmd = build_address_device_trb(input_ctx_phys, slot_id as u32, false);
        let result = self.send_command(addr_cmd)?;

        if result.completion_code != 1 {
            log_val("[qemu-usbd]   AddressDevice failed, cc=", result.completion_code as u64);
            return None;
        }

        log("[qemu-usbd]   Device addressed");

        // Create device tracking entry
        let device = UsbDevice {
            port,
            slot_id,
            speed,
            is_msc: false,
            bulk_in_dci: 0,
            bulk_out_dci: 0,
            bulk_max_packet: 0,
            vendor_id: 0,
            product_id: 0,
            ep0_ring_cycle: true,
            ep0_ring_enqueue: 0,
        };
        self.devices[(slot_id as usize) - 1] = Some(device);

        Some(slot_id)
    }

    /// Enumerate all connected devices
    fn enumerate_all_devices(&mut self) {
        let max_ports = match self.caps.as_ref() {
            Some(c) => c.max_ports,
            None => return,
        };

        log("[qemu-usbd] Enumerating devices...");

        for port in 0..max_ports {
            let connected = match self.xhci.as_ref() {
                Some(x) => x.port_connected(port),
                None => false,
            };

            if connected {
                if let Some(slot_id) = self.enumerate_device(port) {
                    log_val("[qemu-usbd]   Port ", port as u64);
                    log_val("[qemu-usbd]   -> Slot ", slot_id as u64);
                }
            }
        }
    }

    // =========================================================================
    // Control Transfers
    // =========================================================================

    /// Execute a control transfer on endpoint 0
    ///
    /// Returns the data for IN transfers, or empty slice for OUT/no-data.
    fn control_transfer(&mut self, slot_id: u8, setup: &SetupPacket, data_buf: Option<&mut [u8]>) -> Option<usize> {
        let dma = self.dma.as_ref()?;
        let xhci = self.xhci.as_mut()?;
        let evt_ring = self.evt_ring.as_mut()?;

        let vbase = dma.vaddr();
        let pbase = dma.paddr();

        // Get device info
        let dev_idx = (slot_id as usize).checked_sub(1)?;
        let device = self.devices[dev_idx].as_mut()?;

        // Get EP0 ring location
        let dev_offset = layout::DEVICE_CTX_BASE + dev_idx * layout::DEVICE_CTX_SIZE;
        let ep0_ring_vaddr = vbase + dev_offset as u64 + layout::DEV_EP0_RING as u64;
        let ep0_ring_phys = pbase + dev_offset as u64 + layout::DEV_EP0_RING as u64;

        // Use scratchpad for data buffer
        let scratch_vaddr = vbase + layout::SCRATCH_OFFSET as u64;
        let scratch_paddr = pbase + layout::SCRATCH_OFFSET as u64;

        let has_data = setup.has_data();
        let is_in = setup.is_in();
        let data_len = setup.length as usize;

        // Build and enqueue TRBs
        let mut enqueue = device.ep0_ring_enqueue as usize;
        let mut cycle = if device.ep0_ring_cycle { 1u32 } else { 0 };

        // Setup TRB
        let setup_trb = build_setup_trb(setup, cycle);
        unsafe {
            let trb_ptr = (ep0_ring_vaddr as *mut Trb).add(enqueue);
            core::ptr::write_volatile(trb_ptr, setup_trb);
        }
        enqueue += 1;
        if enqueue >= 63 { enqueue = 0; cycle ^= 1; }

        // Data TRB (if needed)
        if has_data {
            // For OUT transfers, copy data to scratch buffer first
            if !is_in {
                if let Some(buf) = data_buf.as_ref() {
                    unsafe {
                        core::ptr::copy_nonoverlapping(buf.as_ptr(), scratch_vaddr as *mut u8, buf.len().min(data_len));
                    }
                }
            }

            let dir = if is_in { Direction::In } else { Direction::Out };
            let data_trb = build_data_trb(scratch_paddr, setup.length, dir, cycle);
            unsafe {
                let trb_ptr = (ep0_ring_vaddr as *mut Trb).add(enqueue);
                core::ptr::write_volatile(trb_ptr, data_trb);
            }
            enqueue += 1;
            if enqueue >= 63 { enqueue = 0; cycle ^= 1; }
        }

        // Status TRB
        let data_dir = if has_data { Some(if is_in { Direction::In } else { Direction::Out }) } else { None };
        let status_trb = build_status_trb(data_dir, cycle);
        unsafe {
            let trb_ptr = (ep0_ring_vaddr as *mut Trb).add(enqueue);
            core::ptr::write_volatile(trb_ptr, status_trb);
        }
        enqueue += 1;
        if enqueue >= 63 { enqueue = 0; cycle ^= 1; }

        // Update device's EP0 ring state
        device.ep0_ring_enqueue = enqueue as u8;
        device.ep0_ring_cycle = cycle != 0;

        dsb();

        // Ring doorbell for slot (EP0 = doorbell target 1)
        xhci.ring_doorbell(slot_id, 1);

        // Wait for completion (poll without sleeping for faster response)
        for iter in 0..100_000 {
            // Short delay every 100 iterations to avoid overwhelming CPU
            if iter % 100 == 0 {
                core::hint::spin_loop();
            }

            while let Some(evt) = evt_ring.dequeue() {
                let evt_type = evt.get_type();
                let cc = evt.event_completion_code();
                let evt_slot = ((evt.control >> 24) & 0xFF) as u8;

                xhci.update_erdp(0, evt_ring.erdp());

                if evt_type == trb_type::TRANSFER_EVENT && evt_slot == slot_id {
                    if cc == 1 || cc == 13 {  // Success or Short Packet
                        // For IN transfers, copy data to caller's buffer
                        if has_data && is_in {
                            if let Some(buf) = data_buf {
                                let copy_len = buf.len().min(data_len);
                                unsafe {
                                    core::ptr::copy_nonoverlapping(scratch_vaddr as *const u8, buf.as_mut_ptr(), copy_len);
                                }
                                return Some(copy_len);
                            }
                        }
                        return Some(0);
                    } else {
                        log_val("[qemu-usbd] Control transfer failed, cc=", cc as u64);
                        return None;
                    }
                }
            }
            xhci.update_erdp(0, evt_ring.erdp());
        }

        log("[qemu-usbd] Control transfer timeout");
        None
    }

    /// Get device descriptor
    fn get_device_descriptor(&mut self, slot_id: u8) -> Option<DeviceDescriptor> {
        let mut buf = [0u8; 18];
        let setup = SetupPacket::new(
            usb_req::DIR_IN | usb_req::TYPE_STANDARD | usb_req::RECIP_DEVICE,
            usb_req::GET_DESCRIPTOR,
            (usb_req::DESC_DEVICE << 8) as u16,  // Descriptor type in high byte
            0,
            18,
        );

        let len = self.control_transfer(slot_id, &setup, Some(&mut buf))?;
        if len < 18 {
            return None;
        }

        Some(unsafe { core::ptr::read_unaligned(buf.as_ptr() as *const DeviceDescriptor) })
    }

    /// Get configuration descriptor (and all interface/endpoint descriptors)
    fn get_config_descriptor(&mut self, slot_id: u8, config_idx: u8, buf: &mut [u8]) -> Option<usize> {
        // First get just the config descriptor header to find total length
        let setup = SetupPacket::new(
            usb_req::DIR_IN | usb_req::TYPE_STANDARD | usb_req::RECIP_DEVICE,
            usb_req::GET_DESCRIPTOR,
            ((usb_req::DESC_CONFIGURATION << 8) | config_idx as u16) as u16,
            0,
            9,
        );

        let mut header = [0u8; 9];
        self.control_transfer(slot_id, &setup, Some(&mut header))?;

        // Get total length from config descriptor
        let total_len = u16::from_le_bytes([header[2], header[3]]) as usize;
        let fetch_len = total_len.min(buf.len());

        // Now fetch the full descriptor
        let setup2 = SetupPacket::new(
            usb_req::DIR_IN | usb_req::TYPE_STANDARD | usb_req::RECIP_DEVICE,
            usb_req::GET_DESCRIPTOR,
            ((usb_req::DESC_CONFIGURATION << 8) | config_idx as u16) as u16,
            0,
            fetch_len as u16,
        );

        self.control_transfer(slot_id, &setup2, Some(&mut buf[..fetch_len]))
    }

    /// Set configuration
    fn set_configuration(&mut self, slot_id: u8, config_value: u8) -> bool {
        let setup = SetupPacket::new(
            usb_req::DIR_OUT | usb_req::TYPE_STANDARD | usb_req::RECIP_DEVICE,
            usb_req::SET_CONFIGURATION,
            config_value as u16,
            0,
            0,
        );

        self.control_transfer(slot_id, &setup, None).is_some()
    }

    // =========================================================================
    // MSC Detection
    // =========================================================================

    /// Detect if device is MSC and configure bulk endpoints
    fn detect_msc(&mut self, slot_id: u8) -> bool {
        log_val("[qemu-usbd] Detecting MSC for slot ", slot_id as u64);

        // Get device descriptor for vendor/product info
        let dev_desc = match self.get_device_descriptor(slot_id) {
            Some(d) => d,
            None => {
                log("[qemu-usbd]   Failed to get device descriptor");
                return false;
            }
        };

        log_hex("[qemu-usbd]   VID:PID = ", ((dev_desc.vendor_id as u64) << 16) | dev_desc.product_id as u64);

        // Update device info
        if let Some(dev) = self.devices[(slot_id as usize) - 1].as_mut() {
            dev.vendor_id = dev_desc.vendor_id;
            dev.product_id = dev_desc.product_id;
        }

        // Get configuration descriptor
        let mut config_buf = [0u8; 256];
        let config_len = match self.get_config_descriptor(slot_id, 0, &mut config_buf) {
            Some(len) => len,
            None => {
                log("[qemu-usbd]   Failed to get config descriptor");
                return false;
            }
        };

        log_val("[qemu-usbd]   Config descriptor length: ", config_len as u64);

        // Parse descriptors to find MSC interface
        let mut offset = 0;
        let mut msc_interface: Option<u8> = None;
        let mut bulk_in_ep: Option<(u8, u16)> = None;  // (address, max_packet)
        let mut bulk_out_ep: Option<(u8, u16)> = None;

        while offset + 2 <= config_len {
            let desc_len = config_buf[offset] as usize;
            let desc_type = config_buf[offset + 1];

            if desc_len == 0 || offset + desc_len > config_len {
                break;
            }

            match desc_type {
                4 => {  // Interface descriptor
                    if desc_len >= 9 {
                        let iface_num = config_buf[offset + 2];
                        let iface_class = config_buf[offset + 5];
                        let iface_subclass = config_buf[offset + 6];
                        let iface_protocol = config_buf[offset + 7];

                        log_val("[qemu-usbd]   Interface ", iface_num as u64);
                        log_hex("[qemu-usbd]     Class: ", iface_class as u64);

                        if iface_class == msc::CLASS &&
                           iface_subclass == msc::SUBCLASS_SCSI &&
                           iface_protocol == msc::PROTOCOL_BOT {
                            log("[qemu-usbd]     -> MSC BOT device!");
                            msc_interface = Some(iface_num);
                        }
                    }
                }
                5 => {  // Endpoint descriptor
                    if desc_len >= 7 && msc_interface.is_some() {
                        let ep_addr = config_buf[offset + 2];
                        let ep_attr = config_buf[offset + 3];
                        let max_packet = u16::from_le_bytes([config_buf[offset + 4], config_buf[offset + 5]]);

                        let ep_type = ep_attr & 0x03;
                        let is_in = (ep_addr & 0x80) != 0;

                        log_hex("[qemu-usbd]     EP: ", ep_addr as u64);
                        log_val("[qemu-usbd]       MaxPacket: ", max_packet as u64);

                        if ep_type == 2 {  // Bulk
                            if is_in && bulk_in_ep.is_none() {
                                bulk_in_ep = Some((ep_addr, max_packet));
                            } else if !is_in && bulk_out_ep.is_none() {
                                bulk_out_ep = Some((ep_addr, max_packet));
                            }
                        }
                    }
                }
                _ => {}
            }

            offset += desc_len;
        }

        // Check if we found MSC with both endpoints
        if msc_interface.is_none() || bulk_in_ep.is_none() || bulk_out_ep.is_none() {
            log("[qemu-usbd]   Not an MSC device");
            return false;
        }

        let (bulk_in_addr, bulk_in_max) = bulk_in_ep.unwrap();
        let (bulk_out_addr, bulk_out_max) = bulk_out_ep.unwrap();

        log("[qemu-usbd]   MSC device detected!");
        log_hex("[qemu-usbd]   Bulk IN:  ", bulk_in_addr as u64);
        log_hex("[qemu-usbd]   Bulk OUT: ", bulk_out_addr as u64);

        // Update device tracking
        if let Some(dev) = self.devices[(slot_id as usize) - 1].as_mut() {
            dev.is_msc = true;
            dev.bulk_in_dci = endpoint_address_to_dci(bulk_in_addr) as u8;
            dev.bulk_out_dci = endpoint_address_to_dci(bulk_out_addr) as u8;
            dev.bulk_max_packet = bulk_in_max.max(bulk_out_max);
        }

        // Set configuration
        let config_value = config_buf[5];  // bConfigurationValue from config descriptor
        if !self.set_configuration(slot_id, config_value) {
            log("[qemu-usbd]   Failed to set configuration");
            return false;
        }
        log_val("[qemu-usbd]   Configuration set: ", config_value as u64);

        true
    }

    /// Enumerate and detect MSC devices
    fn detect_all_msc(&mut self) {
        log("[qemu-usbd] Detecting MSC devices...");

        // Iterate through enumerated devices
        for slot_idx in 0..layout::MAX_DEVICES {
            if let Some(dev) = &self.devices[slot_idx] {
                let slot_id = dev.slot_id;
                if slot_id > 0 {
                    self.detect_msc(slot_id);
                }
            }
        }

        // Summary
        for slot_idx in 0..layout::MAX_DEVICES {
            if let Some(dev) = &self.devices[slot_idx] {
                if dev.is_msc {
                    log_val("[qemu-usbd] MSC device at slot ", dev.slot_id as u64);
                    log_hex("[qemu-usbd]   VID:PID = ", ((dev.vendor_id as u64) << 16) | dev.product_id as u64);
                    log_val("[qemu-usbd]   Bulk IN DCI:  ", dev.bulk_in_dci as u64);
                    log_val("[qemu-usbd]   Bulk OUT DCI: ", dev.bulk_out_dci as u64);
                }
            }
        }
    }

    // =========================================================================
    // Bulk Transfers - This is where you see the difference from network
    // =========================================================================
    //
    // In network: DMA gives you a packet, filters decode layers
    // In USB: We BUILD the packet, DMA sends it, we KNOW what response means
    //
    // There's no "query the CBW filter" - we construct the CBW ourselves
    // and we know the response is data because we asked for data.

    /// Configure bulk endpoints for MSC device
    /// This tells the xHCI controller about the bulk endpoints
    fn configure_msc_endpoints(&mut self, slot_id: u8) -> bool {
        let dev_idx = match (slot_id as usize).checked_sub(1) {
            Some(idx) if idx < layout::MAX_DEVICES => idx,
            _ => return false,
        };

        let (bulk_in_dci, bulk_out_dci, bulk_max_packet, speed) = {
            let dev = match &self.devices[dev_idx] {
                Some(d) if d.is_msc => d,
                _ => return false,
            };
            (dev.bulk_in_dci, dev.bulk_out_dci, dev.bulk_max_packet, dev.speed)
        };

        let dma = match self.dma.as_ref() {
            Some(d) => d,
            None => return false,
        };

        let vbase = dma.vaddr();
        let pbase = dma.paddr();

        // Device context area
        let dev_offset = layout::DEVICE_CTX_BASE + dev_idx * layout::DEVICE_CTX_SIZE;
        let input_ctx_vaddr = vbase + dev_offset as u64 + layout::DEV_INPUT_CTX as u64;
        let input_ctx_paddr = pbase + dev_offset as u64 + layout::DEV_INPUT_CTX as u64;

        // We need to set up bulk endpoint rings
        // For simplicity, carve out space after EP0 ring
        let bulk_out_ring_offset = layout::DEV_EP0_RING + 0x400;  // EP0 ring + 1KB
        let bulk_in_ring_offset = bulk_out_ring_offset + 0x400;   // + another 1KB

        let bulk_out_ring_phys = pbase + dev_offset as u64 + bulk_out_ring_offset as u64;
        let bulk_in_ring_phys = pbase + dev_offset as u64 + bulk_in_ring_offset as u64;
        let bulk_out_ring_vaddr = vbase + dev_offset as u64 + bulk_out_ring_offset as u64;
        let bulk_in_ring_vaddr = vbase + dev_offset as u64 + bulk_in_ring_offset as u64;

        // Initialize bulk rings with link TRBs
        unsafe {
            // Bulk OUT ring - link TRB at position 63
            let link_out = (bulk_out_ring_vaddr as *mut Trb).add(63);
            (*link_out).param = bulk_out_ring_phys;
            (*link_out).status = 0;
            (*link_out).control = (trb_type::LINK << 10) | (1 << 1) | 1; // TC=1, C=1

            // Bulk IN ring - link TRB at position 63
            let link_in = (bulk_in_ring_vaddr as *mut Trb).add(63);
            (*link_in).param = bulk_in_ring_phys;
            (*link_in).status = 0;
            (*link_in).control = (trb_type::LINK << 10) | (1 << 1) | 1; // TC=1, C=1
        }

        // Set up Input Control Context
        let icc = unsafe { &mut *(input_ctx_vaddr as *mut InputControlContext) };
        // Add flags: slot (bit 0) + EP OUT (bit N) + EP IN (bit M)
        // DCI 4 = bulk out -> bit 4, DCI 3 = bulk in -> bit 3
        icc.drop_flags = 0;
        icc.add_flags = 1 | (1 << bulk_out_dci) | (1 << bulk_in_dci);

        // Update slot context (context entries)
        let slot_ctx = unsafe { &mut *((input_ctx_vaddr + 32) as *mut SlotContext) };
        // Set context entries to max endpoint DCI
        let max_dci = bulk_in_dci.max(bulk_out_dci);
        slot_ctx.set_context_entries(max_dci as u32);

        // Configure Bulk OUT endpoint (DCI 4 = endpoints[3])
        let ep_out_idx = (bulk_out_dci - 1) as usize;
        let ep_out = unsafe { &mut *((input_ctx_vaddr + 32 + 32 * (ep_out_idx + 1) as u64) as *mut EndpointContext) };
        ep_out.dw0 = 0;
        ep_out.dw1 = 0;
        ep_out.set_ep_type(2);  // Bulk OUT
        ep_out.set_max_packet_size(bulk_max_packet as u32);
        ep_out.set_max_burst_size(0);
        ep_out.set_cerr(3);
        ep_out.set_tr_dequeue_ptr(bulk_out_ring_phys, true);  // DCS=1
        ep_out.set_average_trb_length(1024);

        // Configure Bulk IN endpoint (DCI 3 = endpoints[2])
        let ep_in_idx = (bulk_in_dci - 1) as usize;
        let ep_in = unsafe { &mut *((input_ctx_vaddr + 32 + 32 * (ep_in_idx + 1) as u64) as *mut EndpointContext) };
        ep_in.dw0 = 0;
        ep_in.dw1 = 0;
        ep_in.set_ep_type(6);  // Bulk IN
        ep_in.set_max_packet_size(bulk_max_packet as u32);
        ep_in.set_max_burst_size(0);
        ep_in.set_cerr(3);
        ep_in.set_tr_dequeue_ptr(bulk_in_ring_phys, true);  // DCS=1
        ep_in.set_average_trb_length(1024);

        dsb();

        // Send Configure Endpoint command
        let cmd = build_configure_endpoint_trb(input_ctx_paddr, slot_id as u32, false);
        let result = match self.send_command(cmd) {
            Some(r) => r,
            None => {
                log("[qemu-usbd] Configure Endpoint command timeout");
                return false;
            }
        };

        if result.completion_code != 1 {
            log_val("[qemu-usbd] Configure Endpoint failed, cc=", result.completion_code as u64);
            return false;
        }

        log("[qemu-usbd] Bulk endpoints configured");
        true
    }

    /// Send a bulk OUT transfer (host -> device)
    /// This is "dumb" - we just send bytes, no protocol awareness
    fn bulk_out(&mut self, slot_id: u8, data: &[u8]) -> bool {
        let dev_idx = match (slot_id as usize).checked_sub(1) {
            Some(idx) if idx < layout::MAX_DEVICES => idx,
            _ => return false,
        };

        let bulk_out_dci = match &self.devices[dev_idx] {
            Some(d) => d.bulk_out_dci,
            None => return false,
        };

        let dma = match self.dma.as_ref() {
            Some(d) => d,
            None => return false,
        };
        let xhci = match self.xhci.as_mut() {
            Some(x) => x,
            None => return false,
        };
        let evt_ring = match self.evt_ring.as_mut() {
            Some(e) => e,
            None => return false,
        };

        let vbase = dma.vaddr();
        let pbase = dma.paddr();

        // Use scratch area for data
        let scratch_vaddr = vbase + layout::SCRATCH_OFFSET as u64;
        let scratch_paddr = pbase + layout::SCRATCH_OFFSET as u64;

        // Copy data to DMA buffer
        let len = data.len().min(layout::SCRATCH_SIZE);
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), scratch_vaddr as *mut u8, len);
        }

        // Get bulk OUT ring
        let dev_offset = layout::DEVICE_CTX_BASE + dev_idx * layout::DEVICE_CTX_SIZE;
        let bulk_out_ring_offset = layout::DEV_EP0_RING + 0x400;
        let bulk_out_ring_vaddr = vbase + dev_offset as u64 + bulk_out_ring_offset as u64;

        // Build Normal TRB for bulk transfer
        // This is the key difference from network: we manually build the TRB
        // There's no "filter" that understands the payload - it's just bytes
        let trb = Trb {
            param: scratch_paddr,
            status: len as u32,
            control: (trb_type::NORMAL << 10) | (1 << 5) | 1,  // IOC=1, C=1
        };

        unsafe {
            let trb_ptr = bulk_out_ring_vaddr as *mut Trb;
            core::ptr::write_volatile(trb_ptr, trb);
        }
        dsb();

        // Ring doorbell for bulk OUT endpoint
        xhci.ring_doorbell(slot_id, bulk_out_dci);

        // Wait for completion
        for _ in 0..100_000 {
            if let Some(evt) = evt_ring.dequeue() {
                if evt.get_type() == trb_type::TRANSFER_EVENT {
                    let cc = evt.event_completion_code();
                    xhci.update_erdp(0, evt_ring.erdp());
                    return cc == 1 || cc == 13;
                }
            }
            xhci.update_erdp(0, evt_ring.erdp());
        }
        false
    }

    /// Receive a bulk IN transfer (device -> host)
    /// Again, "dumb" - we just receive bytes, interpretation is up to caller
    fn bulk_in(&mut self, slot_id: u8, buf: &mut [u8]) -> Option<usize> {
        let dev_idx = match (slot_id as usize).checked_sub(1) {
            Some(idx) if idx < layout::MAX_DEVICES => idx,
            _ => return None,
        };

        let bulk_in_dci = match &self.devices[dev_idx] {
            Some(d) => d.bulk_in_dci,
            None => return None,
        };

        let dma = match self.dma.as_ref() {
            Some(d) => d,
            None => return None,
        };
        let xhci = match self.xhci.as_mut() {
            Some(x) => x,
            None => return None,
        };
        let evt_ring = match self.evt_ring.as_mut() {
            Some(e) => e,
            None => return None,
        };

        let vbase = dma.vaddr();
        let pbase = dma.paddr();

        // Use scratch area for receive
        let scratch_vaddr = vbase + layout::SCRATCH_OFFSET as u64;
        let scratch_paddr = pbase + layout::SCRATCH_OFFSET as u64;

        // Get bulk IN ring
        let dev_offset = layout::DEVICE_CTX_BASE + dev_idx * layout::DEVICE_CTX_SIZE;
        let bulk_in_ring_offset = layout::DEV_EP0_RING + 0x800;
        let bulk_in_ring_vaddr = vbase + dev_offset as u64 + bulk_in_ring_offset as u64;

        let max_len = buf.len().min(layout::SCRATCH_SIZE);

        // Build Normal TRB for bulk IN
        let trb = Trb {
            param: scratch_paddr,
            status: max_len as u32,
            control: (trb_type::NORMAL << 10) | (1 << 5) | 1,  // IOC=1, C=1
        };

        unsafe {
            let trb_ptr = bulk_in_ring_vaddr as *mut Trb;
            core::ptr::write_volatile(trb_ptr, trb);
        }
        dsb();

        // Ring doorbell for bulk IN endpoint
        xhci.ring_doorbell(slot_id, bulk_in_dci);

        // Wait for completion
        for _ in 0..100_000 {
            if let Some(evt) = evt_ring.dequeue() {
                if evt.get_type() == trb_type::TRANSFER_EVENT {
                    let cc = evt.event_completion_code();
                    // TRB transfer length tells us residual
                    let residual = (evt.status & 0xFFFFFF) as usize;
                    let transferred = max_len.saturating_sub(residual);

                    xhci.update_erdp(0, evt_ring.erdp());

                    if cc == 1 || cc == 13 {
                        // Copy received data
                        unsafe {
                            core::ptr::copy_nonoverlapping(
                                scratch_vaddr as *const u8,
                                buf.as_mut_ptr(),
                                transferred
                            );
                        }
                        return Some(transferred);
                    }
                    return None;
                }
            }
            xhci.update_erdp(0, evt_ring.erdp());
        }
        None
    }

    // =========================================================================
    // SCSI/BOT Layer - WE build the protocol, hardware just moves bytes
    // =========================================================================
    //
    // This is the key insight: unlike network where filters decode protocol,
    // here WE construct the CBW, WE know the response format, WE parse CSW.
    // The hardware has NO understanding of SCSI or BOT.

    /// Execute a SCSI command via BOT (Bulk-Only Transport)
    ///
    /// Protocol:
    /// 1. Send CBW (31 bytes) on bulk OUT
    /// 2. Optional data phase (IN or OUT depending on command)
    /// 3. Receive CSW (13 bytes) on bulk IN
    ///
    /// The hardware doesn't know any of this - it just moves bytes.
    /// WE construct CBW, WE interpret CSW.
    fn scsi_command(
        &mut self,
        slot_id: u8,
        cdb: &[u8],
        data_in: Option<&mut [u8]>,
        data_out: Option<&[u8]>,
    ) -> Option<u8> {
        use usb::msc::{Cbw, Csw, msc};

        let tag = self.cmd_tag;
        self.cmd_tag += 1;

        // Determine data direction and length
        let (data_len, direction_in) = match (&data_in, &data_out) {
            (Some(buf), None) => (buf.len() as u32, true),
            (None, Some(buf)) => (buf.len() as u32, false),
            _ => (0, true),
        };

        // === Step 1: Build and send CBW ===
        // This is US building the protocol packet - no filter involved
        let cbw = Cbw::new(tag, data_len, direction_in, 0, cdb);
        let cbw_bytes = unsafe {
            core::slice::from_raw_parts(&cbw as *const Cbw as *const u8, 31)
        };

        if !self.bulk_out(slot_id, cbw_bytes) {
            log("[qemu-usbd] CBW send failed");
            return None;
        }

        // === Step 2: Data phase (if any) ===
        if let Some(buf) = data_in {
            if self.bulk_in(slot_id, buf).is_none() {
                log("[qemu-usbd] Data IN failed");
                return None;
            }
        } else if let Some(buf) = data_out {
            if !self.bulk_out(slot_id, buf) {
                log("[qemu-usbd] Data OUT failed");
                return None;
            }
        }

        // === Step 3: Receive CSW ===
        // Again, WE know the response is 13 bytes CSW - hardware doesn't
        let mut csw_bytes = [0u8; 13];
        if self.bulk_in(slot_id, &mut csw_bytes).is_none() {
            log("[qemu-usbd] CSW receive failed");
            return None;
        }

        // === Step 4: Parse CSW - WE do this, not a filter ===
        let csw = unsafe { &*(csw_bytes.as_ptr() as *const Csw) };

        // Validate CSW
        if csw.signature != msc::CSW_SIGNATURE {
            log("[qemu-usbd] Invalid CSW signature");
            return None;
        }
        if csw.tag != tag {
            log("[qemu-usbd] CSW tag mismatch");
            return None;
        }

        Some(csw.status)
    }

    /// SCSI INQUIRY - asks device "who are you?"
    fn scsi_inquiry(&mut self, slot_id: u8) -> Option<[u8; 36]> {
        let cdb = [0x12, 0, 0, 0, 36, 0, 0, 0, 0, 0];  // INQUIRY, 36 bytes
        let mut response = [0u8; 36];

        let status = self.scsi_command(slot_id, &cdb, Some(&mut response), None)?;
        if status != 0 {
            return None;
        }
        Some(response)
    }

    /// SCSI READ CAPACITY - asks device "how big are you?"
    fn scsi_read_capacity(&mut self, slot_id: u8) -> Option<(u32, u32)> {
        let cdb = [0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0];  // READ CAPACITY(10)
        let mut response = [0u8; 8];

        let status = self.scsi_command(slot_id, &cdb, Some(&mut response), None)?;
        if status != 0 {
            return None;
        }

        // Parse response (big-endian)
        let last_lba = u32::from_be_bytes([response[0], response[1], response[2], response[3]]);
        let block_size = u32::from_be_bytes([response[4], response[5], response[6], response[7]]);

        Some((last_lba, block_size))
    }

    /// Test MSC device with SCSI commands
    fn test_msc(&mut self, slot_id: u8) {
        log_val("[qemu-usbd] Testing MSC device at slot ", slot_id as u64);

        // Configure endpoints first
        if !self.configure_msc_endpoints(slot_id) {
            log("[qemu-usbd] Failed to configure endpoints");
            return;
        }

        // INQUIRY
        log("[qemu-usbd] Sending INQUIRY...");
        if let Some(inquiry) = self.scsi_inquiry(slot_id) {
            // Bytes 8-15: vendor, 16-31: product
            log("[qemu-usbd] INQUIRY response received");
            log_val("[qemu-usbd]   Device type: ", (inquiry[0] & 0x1F) as u64);
        } else {
            log("[qemu-usbd] INQUIRY failed");
        }

        // READ CAPACITY
        log("[qemu-usbd] Sending READ CAPACITY...");
        if let Some((last_lba, block_size)) = self.scsi_read_capacity(slot_id) {
            let total_blocks = last_lba + 1;
            let size_mb = (total_blocks as u64 * block_size as u64) / (1024 * 1024);
            log_val("[qemu-usbd]   Last LBA: ", last_lba as u64);
            log_val("[qemu-usbd]   Block size: ", block_size as u64);
            log_val("[qemu-usbd]   Total size MB: ", size_mb);
        } else {
            log("[qemu-usbd] READ CAPACITY failed");
        }
    }

    // =========================================================================
    // Composable Block Stack Demo
    // =========================================================================
    //
    // This shows how the layered architecture works:
    //   XhciBulk (thin hardware layer - TRBs, rings, doorbells)
    //   ↓ implements ByteTransport
    //   BotScsi (protocol layer - CBW/CSW, hidden from above)
    //   ↓ implements ScsiTransport
    //   ScsiBlockDevice (block layer - read_blocks/write_blocks)
    //   ↓ implements BlockDevice
    //   Application (just sees read/write, knows nothing about USB/SCSI/BOT)

    /// Test the composable block stack
    ///
    /// Creates the full layer hierarchy and reads sector 0.
    /// Each layer hides its internals from the layer above.
    fn test_block_stack(&mut self, slot_id: u8) {
        use crate::block::{BlockDevice, ScsiBlockDevice, BotScsi};
        use crate::xhci_bulk::XhciBulk;

        log("[qemu-usbd] === Testing composable block stack ===");

        // Configure bulk endpoints first (required before any bulk transfers)
        if !self.configure_msc_endpoints(slot_id) {
            log("[qemu-usbd] Failed to configure endpoints");
            return;
        }
        log("[qemu-usbd] Bulk endpoints configured");

        let dev_idx = match (slot_id as usize).checked_sub(1) {
            Some(idx) if idx < layout::MAX_DEVICES => idx,
            _ => return,
        };

        let (bulk_in_dci, bulk_out_dci) = match &self.devices[dev_idx] {
            Some(d) if d.is_msc => (d.bulk_in_dci, d.bulk_out_dci),
            _ => return,
        };

        let dma = match self.dma.as_ref() {
            Some(d) => d,
            None => return,
        };
        let xhci = match self.xhci.as_mut() {
            Some(x) => x,
            None => return,
        };
        let evt_ring = match self.evt_ring.as_mut() {
            Some(e) => e,
            None => return,
        };

        let vbase = dma.vaddr();
        let pbase = dma.paddr();

        // Device context and ring locations
        let dev_offset = layout::DEVICE_CTX_BASE + dev_idx * layout::DEVICE_CTX_SIZE;
        let bulk_out_ring_offset = layout::DEV_EP0_RING + 0x400;
        let bulk_in_ring_offset = bulk_out_ring_offset + 0x400;
        let data_buf_offset = bulk_in_ring_offset + 0x400;  // After bulk rings

        let out_ring_vaddr = vbase + dev_offset as u64 + bulk_out_ring_offset as u64;
        let out_ring_paddr = pbase + dev_offset as u64 + bulk_out_ring_offset as u64;
        let in_ring_vaddr = vbase + dev_offset as u64 + bulk_in_ring_offset as u64;
        let in_ring_paddr = pbase + dev_offset as u64 + bulk_in_ring_offset as u64;
        let data_buf_vaddr = vbase + dev_offset as u64 + data_buf_offset as u64;
        let data_buf_paddr = pbase + dev_offset as u64 + data_buf_offset as u64;
        let data_buf_size = 4096;

        // Layer 1: Create XhciBulk - the thin SoC layer
        // Only THIS layer knows about TRBs, rings, cycle bits, doorbells
        let bulk = XhciBulk::new(
            xhci,
            evt_ring,
            slot_id,
            bulk_in_dci,
            bulk_out_dci,
            out_ring_vaddr as *mut Trb,
            out_ring_paddr,
            in_ring_vaddr as *mut Trb,
            in_ring_paddr,
            data_buf_vaddr as *mut u8,
            data_buf_paddr,
            data_buf_size,
        );

        // Layer 2: Wrap in BotScsi - the protocol layer
        // Only THIS layer knows about CBW/CSW, tags, protocol state
        // XhciBulk doesn't know about BOT, BotScsi doesn't know about TRBs
        let scsi = BotScsi::new(bulk);

        // Layer 3: Wrap in ScsiBlockDevice - the block layer
        // Only THIS layer knows about SCSI CDBs, LBAs, block counts
        // BotScsi doesn't know about SCSI commands, ScsiBlockDevice doesn't know about BOT
        let block_dev = match ScsiBlockDevice::new(scsi) {
            Some(dev) => dev,
            None => {
                log("[qemu-usbd] Failed to create block device");
                return;
            }
        };

        // Now we have a BlockDevice!
        // The application only sees: read_blocks, write_blocks, block_size, block_count
        // It has NO IDEA there's USB, xHCI, TRBs, BOT, SCSI underneath
        log_val("[qemu-usbd] Block size: ", block_dev.block_size() as u64);
        log_val("[qemu-usbd] Block count: ", block_dev.block_count());

        // Save RAW disk info for block service (disk0: exposes the full disk)
        let raw_disk_block_count = block_dev.block_count();
        let raw_disk_block_size = block_dev.block_size();

        // Layer 4: Parse partition table
        // The partition layer reads the MBR and exposes partitions as BlockDevices
        use crate::partition::{read_partition_table, partition_type_name, PartitionDevice, Mbr};

        let mut block_dev = block_dev;

        log("[qemu-usbd] Reading partition table...");
        let mbr = match read_partition_table(&mut block_dev) {
            Some(m) => m,
            None => {
                log("[qemu-usbd] No valid MBR found");
                return;
            }
        };

        if mbr.is_gpt {
            log("[qemu-usbd] GPT disk detected (not supported yet)");
            return;
        }

        log("[qemu-usbd] MBR partition table:");
        for (i, entry) in mbr.partitions() {
            log_val("[qemu-usbd]   Partition ", i as u64);
            log_hex("[qemu-usbd]     Type: ", entry.partition_type as u64);
            log("[qemu-usbd]     Name: ");
            log(partition_type_name(entry.partition_type));
            log_val("[qemu-usbd]     Start LBA: ", entry.start_lba as u64);
            log_val("[qemu-usbd]     Sectors: ", entry.sector_count as u64);
            let size_mb = (entry.sector_count as u64 * 512) / (1024 * 1024);
            log_val("[qemu-usbd]     Size MB: ", size_mb);
            if entry.bootable == 0x80 {
                log("[qemu-usbd]     (bootable)");
            }
        }

        // Layer 5: Create a PartitionDevice for the first FAT partition
        // This wraps the raw disk and offsets all LBAs automatically
        // Filesystem sees LBA 0 = partition start, knows nothing about MBR
        let mut partition_found = false;

        if let Some(fat_entry) = mbr.find_fat_partition() {
            log("[qemu-usbd] Found FAT partition, creating PartitionDevice...");

            let part_dev = match PartitionDevice::from_mbr_entry(block_dev, fat_entry) {
                Some(p) => p,
                None => {
                    log("[qemu-usbd] Failed to create partition device");
                    return;
                }
            };

            log_val("[qemu-usbd] Partition start LBA: ", part_dev.start_lba());
            log_val("[qemu-usbd] Partition blocks: ", part_dev.block_count());

            // Now the filesystem would use this PartitionDevice
            // It sees a BlockDevice starting at LBA 0, completely unaware that:
            // - It's actually at LBA 2048 (or wherever) on the disk
            // - The disk is accessed via SCSI commands
            // - SCSI goes through BOT protocol
            // - BOT runs on xHCI bulk transfers
            // - xHCI uses TRB rings and doorbells
            //
            // Complete stack:
            //   PartitionDevice<ScsiBlockDevice<BotScsi<XhciBulk>>>
            //     └─ BlockDevice trait at each boundary

            // Read first sector of partition (FAT boot sector)
            let mut boot_sector = [0u8; 512];
            let mut part_dev = part_dev;
            if part_dev.read_blocks(0, &mut boot_sector).is_ok() {
                log("[qemu-usbd] Read partition boot sector!");

                // Check for FAT signature
                if boot_sector[510] == 0x55 && boot_sector[511] == 0xAA {
                    log("[qemu-usbd]   FAT boot signature found");
                    partition_found = true;

                    // Parse some FAT fields
                    let bytes_per_sector = u16::from_le_bytes([boot_sector[11], boot_sector[12]]);
                    let sectors_per_cluster = boot_sector[13];
                    let reserved_sectors = u16::from_le_bytes([boot_sector[14], boot_sector[15]]);
                    let num_fats = boot_sector[16];

                    log_val("[qemu-usbd]   Bytes/sector: ", bytes_per_sector as u64);
                    log_val("[qemu-usbd]   Sectors/cluster: ", sectors_per_cluster as u64);
                    log_val("[qemu-usbd]   Reserved sectors: ", reserved_sectors as u64);
                    log_val("[qemu-usbd]   Number of FATs: ", num_fats as u64);

                    // OEM name (bytes 3-10)
                    log("[qemu-usbd]   OEM: ");
                    let oem = &boot_sector[3..11];
                    // Just show as hex for now
                    for b in oem {
                        if *b >= 0x20 && *b < 0x7F {
                            // Printable ASCII - would need proper string handling
                        }
                    }
                }
            } else {
                log("[qemu-usbd] Failed to read partition boot sector");
            }
        } else {
            log("[qemu-usbd] No FAT partition found");
        }

        // Save RAW disk info for DataPort geometry responses
        // disk0: exposes the full raw disk (LBA 0 = MBR), not a partition
        // partition driver will read MBR and create part0:, part1: ports
        if partition_found {
            self.partition_info = BlockInfo {
                block_count: raw_disk_block_count,
                block_size: raw_disk_block_size,
                slot_id,
                bulk_in_dci,
                bulk_out_dci,
            };
            log("[qemu-usbd] Raw disk info saved for DataPort");
        }

        log("[qemu-usbd] === Block stack test complete ===");

        // Reset ring state so service loop starts fresh
        self.reset_bulk_rings(slot_id, bulk_in_dci, bulk_out_dci);
    }

    /// Reset bulk transfer rings to initial state
    /// Uses xHCI commands to reset both software and hardware state
    fn reset_bulk_rings(&mut self, slot_id: u8, bulk_in_dci: u8, bulk_out_dci: u8) {
        use usb::trb::{Trb, trb_type};
        use usb::transfer::dsb;

        let dma = match self.dma.as_ref() {
            Some(d) => d,
            None => return,
        };

        let dev_idx = match (slot_id as usize).checked_sub(1) {
            Some(idx) if idx < layout::MAX_DEVICES => idx,
            _ => return,
        };

        let vbase = dma.vaddr();
        let pbase = dma.paddr();

        // Ring locations (same as test_block_stack)
        let dev_offset = layout::DEVICE_CTX_BASE + dev_idx * layout::DEVICE_CTX_SIZE;
        let bulk_out_ring_offset = layout::DEV_EP0_RING + 0x400;
        let bulk_in_ring_offset = bulk_out_ring_offset + 0x400;

        let out_ring_vaddr = (vbase + dev_offset as u64 + bulk_out_ring_offset as u64) as *mut Trb;
        let out_ring_paddr = pbase + dev_offset as u64 + bulk_out_ring_offset as u64;
        let in_ring_vaddr = (vbase + dev_offset as u64 + bulk_in_ring_offset as u64) as *mut Trb;
        let in_ring_paddr = pbase + dev_offset as u64 + bulk_in_ring_offset as u64;

        const RING_SIZE: usize = 64;

        // Step 1: Stop both endpoints (required before Set TR Dequeue Pointer)
        self.stop_endpoint(slot_id, bulk_out_dci);
        self.stop_endpoint(slot_id, bulk_in_dci);

        // Step 2: Clear OUT ring and set up link TRB
        unsafe {
            for i in 0..(RING_SIZE - 1) {
                core::ptr::write_volatile(out_ring_vaddr.add(i), Trb { param: 0, status: 0, control: 0 });
            }
            // Link TRB at last slot - TC=1 (toggle cycle), cycle=1
            let link_trb = Trb {
                param: out_ring_paddr,
                status: 0,
                control: (trb_type::LINK << 10) | (1 << 1) | 1, // TC=1, cycle=1
            };
            core::ptr::write_volatile(out_ring_vaddr.add(RING_SIZE - 1), link_trb);
        }

        // Step 3: Clear IN ring and set up link TRB
        unsafe {
            for i in 0..(RING_SIZE - 1) {
                core::ptr::write_volatile(in_ring_vaddr.add(i), Trb { param: 0, status: 0, control: 0 });
            }
            let link_trb = Trb {
                param: in_ring_paddr,
                status: 0,
                control: (trb_type::LINK << 10) | (1 << 1) | 1, // TC=1, cycle=1
            };
            core::ptr::write_volatile(in_ring_vaddr.add(RING_SIZE - 1), link_trb);
        }

        dsb();

        // Step 4: Set TR Dequeue Pointer to reset hardware state
        // DCS=1 means cycle bit should be 1 for new TRBs
        self.set_tr_dequeue_pointer(slot_id, bulk_out_dci, out_ring_paddr, true);
        self.set_tr_dequeue_pointer(slot_id, bulk_in_dci, in_ring_paddr, true);

        // Step 5: Reset software ring state
        self.bulk_out_enqueue = 0;
        self.bulk_out_cycle = true;
        self.bulk_in_enqueue = 0;
        self.bulk_in_cycle = true;

        log("[qemu-usbd] Bulk rings reset (hardware + software)");
    }

    /// Read blocks using the block stack
    /// Reads from RAW disk (disk0: exposes full disk, LBA 0 = MBR)
    fn read_blocks_for_service(&mut self, lba: u64, count: u32, buf: &mut [u8]) -> Result<(), &'static str> {
        use crate::block::{BlockDevice, ScsiBlockDevice, BotScsi};
        use crate::xhci_bulk::XhciBulk;

        // Copy disk info before borrowing self mutably
        let slot_id;
        let bulk_in_dci;
        let bulk_out_dci;
        let block_size;
        {
            let info = &self.partition_info;
            if !info.is_valid() {
                return Err("No disk configured");
            }
            slot_id = info.slot_id;
            bulk_in_dci = info.bulk_in_dci;
            bulk_out_dci = info.bulk_out_dci;
            block_size = info.block_size;
        }

        // Reset rings before each call to ensure clean state
        self.reset_bulk_rings(slot_id, bulk_in_dci, bulk_out_dci);

        let dev_idx = match (slot_id as usize).checked_sub(1) {
            Some(idx) if idx < layout::MAX_DEVICES => idx,
            _ => return Err("Invalid slot_id"),
        };

        let dma = self.dma.as_ref().ok_or("No DMA")?;
        let xhci = self.xhci.as_mut().ok_or("No xHCI")?;
        let evt_ring = self.evt_ring.as_mut().ok_or("No evt_ring")?;

        let vbase = dma.vaddr();
        let pbase = dma.paddr();

        // Device context and ring locations (same as test_block_stack)
        let dev_offset = layout::DEVICE_CTX_BASE + dev_idx * layout::DEVICE_CTX_SIZE;
        let bulk_out_ring_offset = layout::DEV_EP0_RING + 0x400;
        let bulk_in_ring_offset = bulk_out_ring_offset + 0x400;
        let data_buf_offset = bulk_in_ring_offset + 0x400;

        let out_ring_vaddr = vbase + dev_offset as u64 + bulk_out_ring_offset as u64;
        let out_ring_paddr = pbase + dev_offset as u64 + bulk_out_ring_offset as u64;
        let in_ring_vaddr = vbase + dev_offset as u64 + bulk_in_ring_offset as u64;
        let in_ring_paddr = pbase + dev_offset as u64 + bulk_in_ring_offset as u64;
        let data_buf_vaddr = vbase + dev_offset as u64 + data_buf_offset as u64;
        let data_buf_paddr = pbase + dev_offset as u64 + data_buf_offset as u64;
        let data_buf_size = 4096;

        // Build the stack
        let bulk = XhciBulk::new(
            xhci,
            evt_ring,
            slot_id,
            bulk_in_dci,
            bulk_out_dci,
            out_ring_vaddr as *mut Trb,
            out_ring_paddr,
            in_ring_vaddr as *mut Trb,
            in_ring_paddr,
            data_buf_vaddr as *mut u8,
            data_buf_paddr,
            data_buf_size,
        );

        let scsi = BotScsi::new(bulk);
        let mut raw_dev = ScsiBlockDevice::new(scsi).ok_or("Failed to create ScsiBlockDevice")?;

        // Read the requested blocks from RAW disk (disk0: exposes full disk, not partition)
        // Partition driver will read MBR at LBA 0 and create partition-specific ports
        let read_len = count as usize * block_size as usize;
        if buf.len() < read_len {
            return Err("Buffer too small");
        }

        raw_dev.read_blocks(lba, &mut buf[..read_len])
            .map_err(|_| "Read failed")
    }

    /// Read blocks to a shared memory pool for DataPort (TRUE ZERO-COPY)
    ///
    /// Data DMAs directly to consumer's pool. CBW/CSW use driver's scratch.
    fn read_blocks_to_pool(
        &mut self,
        lba: u64,
        count: u32,
        _pool_vaddr: *mut u8,
        pool_paddr: u64,
        pool_offset: u32,
        pool_len: u32,
    ) -> Result<u32, &'static str> {
        use crate::xhci_bulk::XhciBulk;

        // Copy disk info before borrowing self mutably
        let slot_id;
        let bulk_in_dci;
        let bulk_out_dci;
        let block_size;
        {
            let info = &self.partition_info;
            if !info.is_valid() {
                return Err("No disk configured");
            }
            slot_id = info.slot_id;
            bulk_in_dci = info.bulk_in_dci;
            bulk_out_dci = info.bulk_out_dci;
            block_size = info.block_size;
        }

        // Calculate read size
        let read_len = count as usize * block_size as usize;
        if pool_len < read_len as u32 {
            return Err("Pool buffer too small");
        }

        // Reset rings before each call to ensure clean state
        self.reset_bulk_rings(slot_id, bulk_in_dci, bulk_out_dci);

        let dev_idx = match (slot_id as usize).checked_sub(1) {
            Some(idx) if idx < layout::MAX_DEVICES => idx,
            _ => return Err("Invalid slot_id"),
        };

        let dma = self.dma.as_ref().ok_or("No DMA")?;
        let xhci = self.xhci.as_mut().ok_or("No xHCI")?;
        let evt_ring = self.evt_ring.as_mut().ok_or("No evt_ring")?;

        let vbase = dma.vaddr();
        let pbase = dma.paddr();

        // Device context and ring locations
        let dev_offset = layout::DEVICE_CTX_BASE + dev_idx * layout::DEVICE_CTX_SIZE;
        let bulk_out_ring_offset = layout::DEV_EP0_RING + 0x400;
        let bulk_in_ring_offset = bulk_out_ring_offset + 0x400;

        let out_ring_vaddr = vbase + dev_offset as u64 + bulk_out_ring_offset as u64;
        let out_ring_paddr = pbase + dev_offset as u64 + bulk_out_ring_offset as u64;
        let in_ring_vaddr = vbase + dev_offset as u64 + bulk_in_ring_offset as u64;
        let in_ring_paddr = pbase + dev_offset as u64 + bulk_in_ring_offset as u64;

        // Use SCRATCH area for CBW/CSW only (128 bytes is plenty)
        let scratch_vaddr = vbase + layout::SCRATCH_OFFSET as u64;
        let scratch_paddr = pbase + layout::SCRATCH_OFFSET as u64;
        let scratch_size = 128; // Only need CBW (64) + CSW (64)

        // Build XhciBulk with scratch buffer for CBW/CSW
        let mut bulk = XhciBulk::new(
            xhci,
            evt_ring,
            slot_id,
            bulk_in_dci,
            bulk_out_dci,
            out_ring_vaddr as *mut Trb,
            out_ring_paddr,
            in_ring_vaddr as *mut Trb,
            in_ring_paddr,
            scratch_vaddr as *mut u8,
            scratch_paddr,
            scratch_size,
        );

        // Build CBW for READ(10)
        let mut cbw = [0u8; 31];
        const CBW_SIG: u32 = 0x43425355;
        cbw[0..4].copy_from_slice(&CBW_SIG.to_le_bytes());
        cbw[4..8].copy_from_slice(&1u32.to_le_bytes()); // tag
        cbw[8..12].copy_from_slice(&(read_len as u32).to_le_bytes());
        cbw[12] = 0x80; // direction: device-to-host
        cbw[14] = 10;   // CDB length

        // READ(10) CDB
        let lba32 = lba as u32;
        let blocks = count as u16;
        cbw[15] = 0x28; // opcode
        cbw[17] = (lba32 >> 24) as u8;
        cbw[18] = (lba32 >> 16) as u8;
        cbw[19] = (lba32 >> 8) as u8;
        cbw[20] = lba32 as u8;
        cbw[22] = (blocks >> 8) as u8;
        cbw[23] = blocks as u8;

        // CSW buffer
        let mut csw = [0u8; 13];

        // Data physical address in consumer's pool - DMA goes directly here!
        let data_phys = pool_paddr + pool_offset as u64;

        // Execute with zero-copy: data DMAs to consumer's pool, CBW/CSW use scratch
        let transferred = bulk.bot_read_zero_copy(&cbw, data_phys, read_len as u32, &mut csw)
            .map_err(|_| "BOT command failed")?;

        // Check CSW status
        const CSW_SIG: u32 = 0x53425355;
        let sig = u32::from_le_bytes([csw[0], csw[1], csw[2], csw[3]]);
        if sig != CSW_SIG {
            return Err("Invalid CSW signature");
        }
        if csw[12] != 0 {
            return Err("CSW status indicates error");
        }

        Ok(transferred)
    }
}

// =============================================================================
// Logging Helpers
// =============================================================================

/// Enable verbose logging (disable for production)
const VERBOSE: bool = false;

#[inline]
fn log(_msg: &str) {
    // Verbose logging disabled - use log_always() for critical messages
}

#[inline]
fn log_hex(_prefix: &str, _val: u64) {
    // Verbose logging disabled
}

#[inline]
fn log_val(_prefix: &str, _val: u64) {
    // Verbose logging disabled
}

/// Log a message that should always appear (errors, ready status)
fn log_always(msg: &str) {
    syscall::klog(syscall::LogLevel::Info, msg.as_bytes());
}

// =============================================================================
// Entry Point
// =============================================================================

#[unsafe(no_mangle)]
fn main() {
    log("[xhcid] USB driver starting");

    // Connect to devd first to get spawn context
    let mut devd_client = match DevdClient::connect() {
        Ok(c) => c,
        Err(_) => {
            syscall::klog(syscall::LogLevel::Error, b"[xhcid] Failed to connect to devd");
            syscall::exit(1);
        }
    };

    // Get xHCI controller info from pcied via spawn context
    let pci_device = match get_device_from_spawn_context(&mut devd_client) {
        Some(dev) => dev,
        None => {
            syscall::klog(syscall::LogLevel::Error, b"[xhcid] No xHCI controller found");
            syscall::exit(1);
        }
    };

    let xhci_base = pci_device.bar0;
    let xhci_size = pci_device.bar_size;

    log_hex("[qemu-usbd] xHCI base: ", xhci_base);
    log_val("[qemu-usbd] xHCI size: ", xhci_size as u64);

    // Create and initialize driver
    let mut driver = QemuUsbDriver::new(xhci_base, xhci_size);

    match driver.init() {
        Ok(()) => {
            log("[qemu-usbd] Initialization successful");
        }
        Err(e) => {
            syscall::klog(syscall::LogLevel::Error, b"[qemu-usbd] Init failed: ");
            syscall::klog(syscall::LogLevel::Error, e.as_bytes());
            syscall::exit(1);
        }
    }

    // Enumerate all connected devices
    driver.enumerate_all_devices();

    // Detect MSC devices and configure them
    driver.detect_all_msc();

    // Test any MSC devices found using the composable block stack
    // Note: Do NOT call test_msc() here - it uses old bulk_in/bulk_out functions
    // that don't track ring enqueue pointer properly, which corrupts the ring state
    // for subsequent test_block_stack() calls.
    for slot_idx in 0..8 {
        let slot_id = match &driver.devices[slot_idx] {
            Some(d) if d.is_msc => d.slot_id,
            _ => continue,
        };
        // Test the composable block stack
        driver.test_block_stack(slot_id);
    }

    // Connect to devd ONCE - we'll use this for registration AND service loop
    let mut devd_client = match DevdClient::connect() {
        Ok(c) => {
            log("[qemu-usbd] Connected to devd");
            c
        }
        Err(_) => {
            log("[qemu-usbd] Failed to connect to devd-query");
            syscall::exit(1);
        }
    };

    // Summary of enumerated devices and register block ports
    log("[qemu-usbd] Enumeration complete");
    let mut disk_count: u8 = 0;
    for (i, dev) in driver.devices.iter().enumerate() {
        if let Some(d) = dev {
            log_val("[qemu-usbd] Device slot ", (i + 1) as u64);
            log_val("[qemu-usbd]   Port: ", d.port as u64);
            log_val("[qemu-usbd]   Speed: ", d.speed as u64);
            if d.is_msc {
                log("[qemu-usbd]   Type: Mass Storage");
                // Register this MSC device as a block port with devd
                if register_block_port_with_client(&mut devd_client, disk_count) {
                    log_val("[qemu-usbd]   Registered disk", disk_count as u64);
                    disk_count += 1;
                }
            }
        }
    }

    if disk_count > 0 && driver.partition_info.is_valid() {
        log_val("[qemu-usbd] Registered block devices: ", disk_count as u64);

        // Create DataPort for zero-copy block I/O (ring protocol)
        // NOTE: Must create DataPort and update shmem_id BEFORE reporting ready,
        // because report_state() triggers devd to send SPAWN_CHILD commands,
        // and if we call update_port_shmem_id() after that, it would consume
        // the SPAWN_CHILD message while waiting for its response.
        let config = DataPortConfig {
            ring_size: 64,
            side_size: 8,
            pool_size: 256 * 1024, // 256KB pool for data transfers
        };

        let data_port = match DataPort::create(config) {
            Ok(p) => {
                log("[qemu-usbd] Created DataPort for disk0:");
                log_val("[qemu-usbd]   shmem_id: ", p.shmem_id() as u64);
                log_val("[qemu-usbd]   pool_size: ", p.pool_size() as u64);
                log_hex("[qemu-usbd]   pool_phys: ", p.pool_phys());
                p
            }
            Err(_) => {
                log("[qemu-usbd] Failed to create DataPort");
                syscall::exit(1);
            }
        };

        // Make DataPort public for any consumer
        data_port.set_public();

        // Update disk0: port's shmem_id with devd
        // The port was already registered above; now we update its DataPort shmem_id
        // so consumers can discover it via devd query
        if let Err(_) = devd_client.update_port_shmem_id(
            b"disk0:",
            data_port.shmem_id(),
        ) {
            log("[qemu-usbd] Warning: failed to update disk0: shmem_id");
        }

        // Report ready state AFTER setting up DataPort
        // This triggers devd to send SPAWN_CHILD commands
        let _ = devd_client.report_state(DriverState::Ready);
        log_always("[xhcid] ready (1 disk)");

        // Spawn handler tracks children to prevent duplicates
        let mut spawn_handler = QemuUsbdSpawnHandler::new();

        // Block info for DataPort geometry responses
        let block_size = driver.partition_info.block_size;
        let block_count = driver.partition_info.block_count;

        // DataPort-only main loop
        loop {
            // === Process DataPort ring requests (zero-copy) ===
            while let Some(sqe) = data_port.recv() {
                match sqe.opcode {
                    io_op::READ => {
                        // Calculate number of blocks
                        let count = (sqe.data_len / block_size) as u32;
                        if count == 0 {
                            data_port.complete_error(sqe.tag, io_status::INVALID);
                            data_port.notify();
                            continue;
                        }

                        // Read directly to pool memory (zero-copy!)
                        match driver.read_blocks_to_pool(
                            sqe.lba,
                            count,
                            data_port.pool_slice_mut(0, data_port.pool_size()).unwrap().as_mut_ptr(),
                            data_port.pool_phys(),
                            sqe.data_offset,
                            sqe.data_len,
                        ) {
                            Ok(transferred) => {
                                data_port.complete_ok(sqe.tag, transferred);
                            }
                            Err(_) => {
                                data_port.complete_error(sqe.tag, io_status::IO_ERROR);
                            }
                        }
                        data_port.notify();
                    }
                    io_op::WRITE => {
                        data_port.complete_error(sqe.tag, io_status::INVALID);
                        data_port.notify();
                    }
                    _ => {
                        data_port.complete_error(sqe.tag, io_status::INVALID);
                        data_port.notify();
                    }
                }
            }

            // === Process DataPort sidechannel queries ===
            while let Some(entry) = data_port.poll_side_request() {
                use userlib::ring::side_msg;
                match entry.msg_type {
                    side_msg::QUERY_GEOMETRY => {
                        let info = userlib::data_port::GeometryInfo {
                            block_size,
                            block_count,
                            max_transfer: 64 * 1024,
                        };
                        data_port.respond_geometry(&entry, &info);
                        data_port.notify();
                    }
                    _ => {
                        let mut eol = entry;
                        eol.status = userlib::ring::side_status::EOL;
                        data_port.side_send(&eol);
                        data_port.notify();
                    }
                }
            }

            // === Check for devd commands ===
            if let Ok(Some(cmd)) = devd_client.poll_command() {
                match cmd {
                    userlib::devd::DevdCommand::SpawnChild { seq_id, binary, binary_len, filter, caps } => {
                        let binary_str = match core::str::from_utf8(&binary[..binary_len]) {
                            Ok(s) => s,
                            Err(_) => continue,
                        };
                        let (count, pids) = spawn_handler.handle_spawn(binary_str, &filter, caps);
                        let result = if count > 0 { 0i32 } else { -1i32 };
                        let child_pid = pids[0];
                        let _ = devd_client.ack_spawn(seq_id, result, child_pid);
                    }
                    userlib::devd::DevdCommand::StopChild { child_pid, .. } => {
                        spawn_handler.handle_stop(child_pid);
                    }
                    userlib::devd::DevdCommand::QueryInfo { seq_id } => {
                        let info = driver.format_info();
                        let info_len = info.iter().rposition(|&b| b != 0).map(|p| p + 1).unwrap_or(0);
                        let _ = devd_client.respond_info(seq_id, &info[..info_len]);
                    }
                    // Orchestration commands not handled by xhcid
                    _ => {}
                }
            }

            // Brief sleep to avoid busy-looping (1ms poll interval)
            syscall::sleep_us(1000);
        }
    } else {
        log_always("[xhcid] ready (no block devices)");
        syscall::exit(0);
    }
}

/// Custom spawn handler for qemu-usbd
///
/// Logs spawn commands and can be extended to track children.
struct QemuUsbdSpawnHandler {
    /// Track spawned children to prevent duplicate spawns
    spawned_partition: Option<u32>,
}

impl QemuUsbdSpawnHandler {
    fn new() -> Self {
        Self { spawned_partition: None }
    }
}

impl SpawnHandler for QemuUsbdSpawnHandler {
    fn handle_spawn(&mut self, binary: &str, filter: &SpawnFilter, caps: u64) -> (u8, [u32; 16]) {
        log("[qemu-usbd] Received SPAWN_CHILD command");
        log("[qemu-usbd]   Binary: ");
        log(binary);

        // Guard: if we've already spawned partition, return the existing PID
        if binary == "partition" {
            if let Some(existing_pid) = self.spawned_partition {
                log("[qemu-usbd]   Already spawned partition, returning existing PID");
                let mut pids = [0u32; 16];
                pids[0] = existing_pid;
                return (1, pids);
            }
        }

        // Log filter info
        match filter.mode {
            0 => { // EXACT
                log("[qemu-usbd]   Filter: EXACT");
                if let Ok(s) = core::str::from_utf8(filter.pattern_bytes()) {
                    log("[qemu-usbd]     Pattern: ");
                    log(s);
                }
            }
            1 => { // BY_TYPE
                log("[qemu-usbd]   Filter: BY_TYPE");
                log_val("[qemu-usbd]     Port type: ", filter.port_type as u64);
            }
            2 => { // CHILDREN_OF
                log("[qemu-usbd]   Filter: CHILDREN_OF");
                if let Ok(s) = core::str::from_utf8(filter.pattern_bytes()) {
                    log("[qemu-usbd]     Parent: ");
                    log(s);
                }
            }
            3 => { // ALL
                log("[qemu-usbd]   Filter: ALL (broadcast)");
            }
            _ => {
                log("[qemu-usbd]   Filter: Unknown");
            }
        }

        // Spawn the child process (with caps if specified)
        let mut pids = [0u32; 16];
        let pid = if caps != 0 {
            syscall::exec_with_caps(binary, caps)
        } else {
            syscall::exec(binary)
        };
        if pid > 0 {
            log_val("[qemu-usbd]   Spawned child PID: ", pid as u64);
            pids[0] = pid as u32;

            // Track partition spawn
            if binary == "partition" {
                self.spawned_partition = Some(pid as u32);
            }

            (1, pids)
        } else {
            log("[qemu-usbd]   Failed to spawn child");
            log_val("[qemu-usbd]   Error: ", (-pid) as u64);
            (0, pids)
        }
    }

    fn handle_stop(&mut self, pid: u32) {
        log_val("[qemu-usbd] Received STOP_CHILD for PID ", pid as u64);
        let result = syscall::kill(pid);
        if result == 0 {
            log("[qemu-usbd]   Child killed successfully");
            // Clear tracking if we killed the partition
            if self.spawned_partition == Some(pid) {
                self.spawned_partition = None;
            }
        } else {
            log_val("[qemu-usbd]   Kill failed, error: ", (-result) as u64);
        }
    }
}

/// Register a block device port and device with devd using an existing client
fn register_block_port_with_client(client: &mut DevdClient, disk_num: u8) -> bool {
    // Build port name: disk0:, disk1:, etc.
    let mut name = [0u8; 8];
    name[0..4].copy_from_slice(b"disk");
    name[4] = b'0' + disk_num;
    name[5] = b':';
    let name_len = 6;

    // Register port and device in one call
    match client.register_port_and_device(
        &name[..name_len],
        PortType::Block,
        DeviceInfo {
            port_name: &name[..name_len],
            class: DeviceClass::MassStorage,
            subclass: 0x06, // SCSI
            name: b"USB Mass Storage",
            ..Default::default()
        },
    ) {
        Ok(Some(_device_id)) => {
            log("[qemu-usbd] Port and device registered");
            true
        }
        Ok(None) => {
            log("[qemu-usbd] Port registered (device failed)");
            true
        }
        Err(_) => {
            log("[qemu-usbd] Port registration failed");
            false
        }
    }
}

// =============================================================================
// Helper Functions for Formatting
// =============================================================================

fn format_u32(buf: &mut [u8], val: u32) -> usize {
    if val == 0 {
        buf[0] = b'0';
        return 1;
    }
    let mut n = val;
    let mut len = 0;
    while n > 0 {
        len += 1;
        n /= 10;
    }
    n = val;
    for i in (0..len).rev() {
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    len
}

fn format_hex_16(buf: &mut [u8], val: u16) -> usize {
    const HEX_CHARS: &[u8] = b"0123456789ABCDEF";
    buf[0] = HEX_CHARS[((val >> 12) & 0xF) as usize];
    buf[1] = HEX_CHARS[((val >> 8) & 0xF) as usize];
    buf[2] = HEX_CHARS[((val >> 4) & 0xF) as usize];
    buf[3] = HEX_CHARS[(val & 0xF) as usize];
    4
}
