//! USB userspace driver for BPI-R4 kernel
//!
//! Full USB host controller driver running entirely in userspace.
//! IMPORTANT: This driver does NOT rely on U-Boot initialization.
//! All clocks, resets, PHY, and xHCI must be configured from scratch.
//!
//! ## Architecture
//!
//! This driver uses the layered USB architecture:
//! - Board: BPI-R4 specific configuration (GPIO, power)
//! - SoC: MT7988A IPPC wrapper (clock, reset, port control)
//! - PHY: T-PHY v2 driver (host mode configuration)
//! - xHCI: Standard xHCI controller (portable)
//!
//! ## MMIO Regions
//! - MMIO scheme for device register access
//! - IRQ scheme for interrupt handling

#![no_std]
#![no_main]

use userlib::{println, print, syscall};
use userlib::ring::{BlockRing, BlockRequest, BlockResponse};

// Import from the usb driver library
use usb::{
    // GPIO commands
    GPIO_CMD_USB_VBUS,
    // xHCI registers
    xhci_op, xhci_rt, xhci_ir,
    // TRB
    Trb, trb_type, trb_cc,
    // USB types
    usb_req, usb_hub as hub, ep_type,
    InputContext, DeviceContext,
    SsHubDescriptor, PortStatus,
    DeviceDescriptor, ConfigurationDescriptor, InterfaceDescriptor,
    // MSC/SCSI
    msc_const as msc, scsi, Cbw, Csw, BulkContext, TransferResult,
    CBW_OFFSET, CSW_OFFSET, DATA_OFFSET,
    // Ring structures
    Ring, EventRing, ErstEntry,
    // MMIO helpers
    MmioRegion, delay_ms, delay,
    // Port status parsing
    ParsedPortsc, portsc,
    event_completion_code, event_slot_id, event_port_id,
    completion_code, doorbell,
    // Enumeration helpers
    build_enable_slot_trb, build_noop_trb,
    // Hub helpers
    get_hub_descriptor_setup, get_port_status_setup,
    set_port_feature_setup, clear_port_feature_setup,
    // Cache ops (ARM64 cache maintenance for DMA)
    flush_cache_line, invalidate_cache_line, flush_buffer, invalidate_buffer, dsb,
};

// Modular architecture - Board abstraction hides SoC details
use usb::board::{Board, BpiR4, UsbControllerConfig};
use usb::soc::{SocUsb, Mt7988aSoc};
use usb::phy::{PhyDriver, Mt7988aTphy};
use usb::xhci::Controller as XhciController;

// Module definitions now imported from usb crate

// =============================================================================
// USB Driver
// =============================================================================

/// Memory layout for xHCI structures (allocated via mmap)
/// Page 0: DCBAA (2KB) + ERST (64B) + padding
/// Page 1: Command Ring (64 TRBs * 16 = 1KB)
/// Page 2: Event Ring (64 TRBs * 16 = 1KB)
const XHCI_MEM_SIZE: usize = 4096 * 3;  // 3 pages

struct UsbDriver {
    // New architecture: separate SoC and PHY drivers
    soc: Mt7988aSoc,
    phy: Mt7988aTphy,

    // Pure xHCI controller (new modular architecture)
    xhci: XhciController,

    // xHCI data structures
    xhci_mem: u64,          // Virtual address of allocated xHCI memory
    xhci_phys: u64,         // Physical address of allocated xHCI memory
    cmd_ring: Option<Ring>,
    event_ring: Option<EventRing>,

    // IRQ support
    irq_fd: i32,            // IRQ file descriptor (-1 if not registered)

    // MSC device info (for block device access)
    msc_device: Option<MscDeviceInfo>,
}

/// Mass Storage Class device info for block operations
/// Stores all state needed to perform bulk transfers without BulkContext
struct MscDeviceInfo {
    slot_id: u32,
    bulk_in_dci: u32,
    bulk_out_dci: u32,
    block_size: u32,
    block_count: u64,
    // Bulk IN ring
    bulk_in_ring: *mut Trb,
    bulk_in_ring_phys: u64,
    // Bulk OUT ring
    bulk_out_ring: *mut Trb,
    bulk_out_ring_phys: u64,
    // Data buffer
    data_buf: *mut u8,
    data_phys: u64,
    // Device context
    device_ctx: *mut DeviceContext,
    // Ring state (persisted between calls)
    out_enqueue: usize,
    in_enqueue: usize,
    out_cycle: bool,  // Cycle bit for bulk out ring
    in_cycle: bool,   // Cycle bit for bulk in ring
    tag: u32,
    // USB endpoint addresses (for CLEAR_FEATURE)
    bulk_in_addr: u8,
    bulk_out_addr: u8,
    // Interface number (for BOT Reset)
    interface_num: u8,
    // EP0 ring for control transfers (for USB recovery)
    ep0_ring: *mut Trb,
    ep0_ring_phys: u64,
    ep0_enqueue: usize,
}

impl UsbDriver {
    fn new(controller_id: u32) -> Option<Self> {
        // === Board Configuration ===
        // Use the BPI-R4 board configuration to get addresses
        let board = if controller_id == 0 {
            BpiR4::ssusb0()
        } else {
            BpiR4::ssusb1()
        };

        let config = board.controller_config(controller_id as u8)?;
        println!("Initializing {}...", config.name);

        // === Create SoC Wrapper ===
        let mut soc = board.create_soc(controller_id as u8)?;

        // Map MAC (xHCI + IPPC) region
        let mac = MmioRegion::open(config.mac_base, config.mac_size as u64)?;
        soc.set_mac(mac.clone());

        // SoC pre-init (IPPC clocks, power, port control)
        if soc.pre_init().is_err() {
            println!("  ERROR: SoC pre_init failed");
            return None;
        }

        // === Create PHY Driver ===
        let mut phy = board.create_phy(controller_id as u8)?;
        let phy_mmio = MmioRegion::open(config.phy_base, config.phy_size as u64)?;
        phy.set_mmio(phy_mmio.clone());

        // === Create xHCI Controller ===
        // The new Controller is pure xHCI - no vendor code
        let xhci = XhciController::new(mac);

        Some(Self {
            soc,
            phy,
            xhci,
            xhci_mem: 0,
            xhci_phys: 0,
            cmd_ring: None,
            event_ring: None,
            irq_fd: -1,
            msc_device: None,
        })
    }

    // Delegate register access to XhciController methods
    #[inline(always)]
    fn op_read32(&self, offset: usize) -> u32 {
        self.xhci.op_read32(offset)
    }

    #[inline(always)]
    fn rt_read64(&self, offset: usize) -> u64 {
        self.xhci.rt_read64(offset)
    }

    #[inline(always)]
    fn rt_write64(&self, offset: usize, value: u64) {
        self.xhci.rt_write64(offset, value)
    }

    #[inline(always)]
    fn ring_doorbell(&self, slot: u32, target: u32) {
        self.xhci.ring_doorbell(slot as u8, target as u8)
    }

    /// Get max ports
    fn num_ports(&self) -> u8 {
        self.xhci.max_ports()
    }

    /// Set the IRQ file descriptor for interrupt-based waiting
    fn set_irq_fd(&mut self, fd: i32) {
        self.irq_fd = fd;
    }

    /// Wait for xHCI interrupt (blocking)
    /// Now uses kernel blocking - read() will block until IRQ fires
    /// The timeout_ms parameter is currently ignored as the kernel handles blocking
    fn wait_for_irq(&self, _timeout_ms: u32) -> bool {
        if self.irq_fd < 0 {
            return false;
        }

        let mut buf = [0u8; 4];
        // Blocking read - kernel will wake us when IRQ fires
        let result = syscall::read(self.irq_fd as u32, &mut buf);
        result > 0
    }

    fn init(&mut self) -> bool {
        // Read xHCI capabilities
        let caps = match self.xhci.read_capabilities() {
            Some(c) => c,
            None => {
                println!("  ERROR: Failed to read xHCI capabilities");
                return false;
            }
        };
        print!("  xHCI v{:x}.{:x}, {} ports, {} slots",
               caps.version >> 8, caps.version & 0xff,
               caps.max_ports, caps.max_slots);

        // Halt and reset xHCI
        self.xhci.halt_and_reset();

        // Configure PHY for host mode (after xHCI reset)
        if self.phy.set_host_mode().is_err() {
            println!("  ERROR: PHY set_host_mode failed");
            return false;
        }
        let _ = self.soc.force_usb2_phy_power();

        // Allocate DMA memory for xHCI data structures
        let mut phys_addr: u64 = 0;
        let mem_addr = syscall::mmap_dma(XHCI_MEM_SIZE, &mut phys_addr);
        if mem_addr < 0 {
            println!("  ERROR: Failed to allocate xHCI memory");
            return false;
        }
        self.xhci_mem = mem_addr as u64;
        self.xhci_phys = phys_addr;

        // Memory layout
        let dcbaa_virt = self.xhci_mem as *mut u64;
        let dcbaa_phys = self.xhci_phys;
        let erst_virt = (self.xhci_mem + 0x800) as *mut ErstEntry;
        let erst_phys = self.xhci_phys + 0x800;
        let cmd_ring_virt = (self.xhci_mem + 0x1000) as *mut Trb;
        let cmd_ring_phys = self.xhci_phys + 0x1000;
        let evt_ring_virt = (self.xhci_mem + 0x2000) as *mut Trb;
        let evt_ring_phys = self.xhci_phys + 0x2000;

        // Zero and flush DCBAA
        unsafe {
            for i in 0..256 {
                *dcbaa_virt.add(i) = 0;
            }
            flush_buffer(dcbaa_virt as u64, 256 * 8);
        }

        // Initialize rings
        self.cmd_ring = Some(Ring::init(cmd_ring_virt, cmd_ring_phys));
        let usbsts_addr = self.xhci.mmio_base() + self.xhci.op_offset() as u64 + xhci_op::USBSTS as u64;
        self.event_ring = Some(EventRing::init(evt_ring_virt, evt_ring_phys, erst_virt, erst_phys, usbsts_addr));

        // Program xHCI registers
        self.xhci.set_max_slots(self.xhci.max_slots());
        self.xhci.set_dcbaap(dcbaa_phys);
        self.xhci.set_crcr(cmd_ring_phys, true);
        self.xhci.program_interrupter(0, erst_phys, 1, evt_ring_phys);

        // Start controller
        if !self.xhci.start() {
            println!("  ERROR: Controller failed to start");
            return false;
        }

        // Power on ports and wait for connection
        self.xhci.power_on_all_ports();
        if self.xhci.max_ports() > 0 {
            if let Some(port) = self.xhci.wait_for_connection(100000) {
                println!(", device on port {}", port + 1);
            } else {
                println!(", no device");
            }
        } else {
            println!();
        }

        true
    }

    /// Send a No-Op command to test the command ring
    fn send_noop(&mut self) -> bool {
        let cmd_ring = match &mut self.cmd_ring {
            Some(r) => r,
            None => return false,
        };
        let trb = build_noop_trb();
        cmd_ring.enqueue(&trb);
        self.ring_doorbell(0, doorbell::HOST_CONTROLLER);
        true
    }

    /// Poll for and process events (silent unless error)
    fn poll_events(&mut self) -> bool {
        let mut events: [Option<Trb>; 16] = [None; 16];
        let mut event_count = 0;
        let mut final_erdp = 0u64;

        if let Some(ref mut event_ring) = self.event_ring {
            while event_count < 16 {
                if let Some(trb) = event_ring.dequeue() {
                    events[event_count] = Some(trb);
                    event_count += 1;
                    final_erdp = event_ring.erdp();
                } else {
                    break;
                }
            }
        }

        if event_count == 0 {
            return false;
        }

        for i in 0..event_count {
            if let Some(ref trb) = events[i] {
                let evt_type = trb.get_type();
                match evt_type {
                    trb_type::PORT_STATUS_CHANGE => {
                        self.handle_port_change(event_port_id(trb));
                    }
                    trb_type::HOST_CONTROLLER => {
                        println!("ERROR: Host Controller Error!");
                    }
                    _ => {}
                }
            }
        }

        let ir0 = xhci_rt::IR0;
        self.rt_write64(ir0 + xhci_ir::ERDP, final_erdp | (1 << 3));
        true
    }

    /// Handle port status change (silent)
    fn handle_port_change(&mut self, port_id: u32) {
        if port_id == 0 || port_id > self.xhci.max_ports() as u32 {
            return;
        }
        let port = (port_id - 1) as u8;
        let raw = self.xhci.read_portsc(port);
        let status = ParsedPortsc::from_raw(raw);
        let clear_bits = raw & portsc::RW1C_BITS;
        if clear_bits != 0 {
            self.xhci.write_portsc(port, status.clear_changes_value());
        }
    }

    fn print_port_status(&self) {
        for p in 0..self.xhci.max_ports() {
            let raw = self.xhci.read_portsc(p);
            let status = ParsedPortsc::from_raw(raw);
            if !status.powered {
                continue;
            }

            // Only print if device connected
            if status.connected {
                print!("  Port {}: {} {}", p + 1, status.speed.as_str(),
                       if status.enabled { "[Enabled]" } else { "" });
                println!();
            }
        }
    }

    /// Check port power status (silent - just returns status)
    fn check_power_status(&self) {
        // Silent - power status check no longer prints
    }

    /// Reset a port to trigger device enumeration (silent unless error)
    fn reset_port(&self, port: u32) -> bool {
        if port == 0 || port > self.xhci.max_ports() as u32 {
            return false;
        }
        let port_idx = (port - 1) as u8;
        let raw = self.xhci.read_portsc(port_idx);
        let status = ParsedPortsc::from_raw(raw);

        if !status.powered || !status.connected {
            return false;
        }
        if status.enabled {
            return true;  // Already working
        }

        // Set PR (Port Reset) bit
        let write_val = (raw & portsc::PRESERVE_BITS) | portsc::PR;
        self.xhci.write_portsc(port_idx, write_val);

        for _ in 0..100 {
            delay(10000);
            let raw = self.xhci.read_portsc(port_idx);
            let status = ParsedPortsc::from_raw(raw);
            if !status.reset && status.reset_change {
                self.xhci.write_portsc(port_idx, raw | portsc::PRC);
                return true;
            }
        }
        false
    }

    /// Enable a slot for a new device (silent)
    fn enable_slot(&mut self) -> Option<u32> {
        let cmd_ring = self.cmd_ring.as_mut()?;
        let trb = build_enable_slot_trb();
        cmd_ring.enqueue(&trb);
        self.ring_doorbell(0, doorbell::HOST_CONTROLLER);
        delay(10000);

        let mut result: Option<(u32, u32)> = None;
        let mut erdp_to_update: Option<u64> = None;

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..20 {
                if let Some(evt) = event_ring.dequeue() {
                    erdp_to_update = Some(event_ring.erdp());
                    if evt.get_type() == trb_type::COMMAND_COMPLETION {
                        result = Some((event_completion_code(&evt), event_slot_id(&evt)));
                        break;
                    }
                }
                delay(1000);
            }
        }

        if let Some(erdp) = erdp_to_update {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
        }

        if let Some((cc, slot_id)) = result {
            if cc == trb_cc::SUCCESS {
                return Some(slot_id);
            }
        }
        None
    }

    /// Enumerate a device on the given port
    fn enumerate_device(&mut self, port: u32) -> Option<u32> {
        let slot_id = self.enable_slot()?;

        // Allocate memory for device context and input context
        let ctx_size = 4096;
        let mut ctx_phys: u64 = 0;
        let ctx_virt = syscall::mmap_dma(ctx_size, &mut ctx_phys);
        if ctx_virt < 0 {
            return None;
        }

        let ctx_base = ctx_virt as u64;
        let ctx_phys_base = ctx_phys;

        // Layout within the page:
        // 0x000: Input Context (1088 bytes, 64-aligned)
        // 0x500: Device Context (1024 bytes, 64-aligned)
        // 0x900: EP0 Transfer Ring (1024 bytes, 64-aligned)
        let input_ctx_virt = ctx_base as *mut InputContext;
        let input_ctx_phys = ctx_phys_base;
        let device_ctx_virt = (ctx_base + 0x500) as *mut DeviceContext;
        let device_ctx_phys = ctx_phys_base + 0x500;
        let ep0_ring_virt = (ctx_base + 0x900) as *mut Trb;
        let ep0_ring_phys = ctx_phys_base + 0x900;

        // Initialize EP0 transfer ring (64 TRBs)
        unsafe {
            for i in 0..64 {
                *ep0_ring_virt.add(i) = Trb::new();
            }
            // Link TRB at end
            let link = &mut *ep0_ring_virt.add(63);
            link.param = ep0_ring_phys;
            link.set_type(trb_type::LINK);
            link.control |= 1 << 5;  // Toggle cycle

            // Flush EP0 ring to memory
            flush_buffer(ep0_ring_virt as u64, 64 * core::mem::size_of::<Trb>());
        }

        // Get port speed from PORTSC using library helpers
        let raw = self.xhci.read_portsc((port - 1) as u8);
        let status = ParsedPortsc::from_raw(raw);
        let speed = status.speed;
        let speed_raw = speed.to_slot_speed();

        print!("  {} device on port {}", speed.full_name(), port);

        // Max packet size for EP0 based on speed
        let max_packet_size = speed.default_ep0_max_packet();

        // Step 3: Initialize Input Context
        unsafe {
            let input = &mut *input_ctx_virt;
            *input = InputContext::new();

            // Add Slot Context and EP0 Context (bits 0 and 1)
            input.control.add_flags = 0x3;  // A0=1, A1=1

            // Slot Context
            input.slot.set_route_string(0);
            input.slot.set_speed(speed_raw);
            input.slot.set_context_entries(1);  // Only EP0
            input.slot.set_root_hub_port(port);

            // EP0 Context (index 0 = control endpoint)
            input.endpoints[0].set_ep_type(ep_type::CONTROL);
            input.endpoints[0].set_max_packet_size(max_packet_size);
            input.endpoints[0].set_cerr(3);  // 3 retries
            input.endpoints[0].set_tr_dequeue_ptr(ep0_ring_phys, true);  // DCS=1
            input.endpoints[0].set_average_trb_length(8);

            // Flush input context to memory
            flush_buffer(input_ctx_virt as u64, core::mem::size_of::<InputContext>());

            // Initialize Device Context
            let device = &mut *device_ctx_virt;
            *device = DeviceContext::new();

            // Flush device context to memory
            flush_buffer(device_ctx_virt as u64, core::mem::size_of::<DeviceContext>());
        }

        // Step 4: Set DCBAA entry for this slot
        let dcbaa = self.xhci_mem as *mut u64;
        unsafe {
            *dcbaa.add(slot_id as usize) = device_ctx_phys;
            // Flush DCBAA entry
            flush_cache_line(dcbaa.add(slot_id as usize) as u64);
            dsb();
        }
        // Address Device command
        {
            let cmd_ring = self.cmd_ring.as_mut()?;
            let mut trb = Trb::new();
            trb.param = input_ctx_phys;
            trb.set_type(trb_type::ADDRESS_DEVICE);
            trb.control |= (slot_id & 0xFF) << 24;
            cmd_ring.enqueue(&trb);
            self.ring_doorbell(0, 0);
        }

        delay(50000);

        // Wait for Address Device completion
        let mut addr_result: Option<u32> = None;
        let mut erdp_to_write: Option<u64> = None;
        let mut fail_cc: Option<u32> = None;

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..50 {
                if let Some(evt) = event_ring.dequeue() {
                    erdp_to_write = Some(event_ring.erdp());
                    if evt.get_type() == trb_type::COMMAND_COMPLETION {
                        let cc = (evt.status >> 24) & 0xFF;
                        if cc == trb_cc::SUCCESS {
                            addr_result = Some(slot_id);
                        } else {
                            fail_cc = Some(cc);
                        }
                        break;
                    }
                }
                delay(1000);
            }
        }

        if let Some(erdp) = erdp_to_write {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
        }

        if addr_result.is_none() {
            if let Some(cc) = fail_cc {
                println!(" - Address failed (cc={})", cc);
            } else {
                println!(" - Address failed (no event)");
            }
            return None;
        }

        // Invalidate device context before reading (xHCI wrote to it)
        invalidate_buffer(device_ctx_virt as u64, core::mem::size_of::<DeviceContext>());

        // Get Device Descriptor

        let mut desc_phys: u64 = 0;
        let desc_virt = syscall::mmap_dma(4096, &mut desc_phys);
        if desc_virt < 0 {
            return Some(slot_id);
        }
        let desc_buf = desc_virt as *mut u8;

        // Build Setup Stage TRB
        // GET_DESCRIPTOR: bmRequestType=0x80, bRequest=6, wValue=0x0100 (device), wIndex=0, wLength=18
        let setup_data: u64 =
            (0x80 as u64) |               // bmRequestType: Device-to-host, standard, device
            ((0x06 as u64) << 8) |        // bRequest: GET_DESCRIPTOR
            ((0x0100 as u64) << 16) |     // wValue: Descriptor type (device=1) << 8 | index
            ((0 as u64) << 32) |          // wIndex: 0
            ((18 as u64) << 48);          // wLength: 18 bytes

        // EP0 transfer ring - track enqueue position for subsequent transfers
        let mut ep0_enqueue = 0usize;  // Start at beginning

        unsafe {
            // Setup TRB
            // Control bits: TRB_Type[15:10]=2, TRT[17:16]=3(IN), IDT[6]=1, Cycle[0]=1
            let setup_trb = &mut *ep0_ring_virt.add(ep0_enqueue);
            setup_trb.param = setup_data;
            setup_trb.status = 8;  // TRB transfer length = 8 (setup packet)
            setup_trb.control = (trb_type::SETUP << 10) | (3 << 16) | (1 << 6) | 1;  // TRT=3 (IN), IDT=1, Cycle=1

            // Data Stage TRB
            // Control bits: TRB_Type[15:10]=3, DIR[16]=1(IN), Cycle[0]=1
            let data_trb = &mut *ep0_ring_virt.add(ep0_enqueue + 1);
            data_trb.param = desc_phys;
            data_trb.status = 18;  // TRB transfer length = 18 bytes
            data_trb.control = (trb_type::DATA << 10) | (1 << 16) | 1;  // DIR=1 (IN), Cycle=1

            // Status Stage TRB
            // Control bits: TRB_Type[15:10]=4, DIR[16]=0(OUT for IN transfer), IOC[5]=1, Cycle[0]=1
            let status_trb = &mut *ep0_ring_virt.add(ep0_enqueue + 2);
            status_trb.param = 0;
            status_trb.status = 0;
            status_trb.control = (trb_type::STATUS << 10) | (1 << 5) | 1;  // IOC=1, DIR=0 (OUT), Cycle=1

            // Flush TRBs to memory before doorbell
            flush_buffer(ep0_ring_virt.add(ep0_enqueue) as u64, 3 * core::mem::size_of::<Trb>());
        }

        // Ring doorbell for EP0 (target = 1 for EP0)
        self.ring_doorbell(slot_id, 1);

        // Wait for transfer completion
        delay(50000);

        // Poll for transfer event - properly handle all events
        // Note: SuperSpeed direct connect may need more iterations than through a hub
        let mut transfer_result: Option<(u32, u32)> = None;  // (cc, remaining_len)
        let mut erdp_to_update: Option<u64> = None;

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..3000 {
                if let Some(evt) = event_ring.dequeue() {
                    // Always track ERDP for any consumed event
                    erdp_to_update = Some(event_ring.erdp());

                    let evt_type = evt.get_type();
                    if evt_type == trb_type::TRANSFER_EVENT {
                        let cc = (evt.status >> 24) & 0xFF;
                        let remaining = evt.status & 0xFFFFFF;
                        transfer_result = Some((cc, remaining));
                        break;
                    }
                    // Ignore other events but acknowledge them
                }
                // 1ms delay between polls using hardware timer
                delay_ms(1);
            }
        }

        // Update ERDP for all consumed events
        if let Some(erdp) = erdp_to_update {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
        }

        // Process transfer result
        let mut transfer_ok = false;
        if let Some((cc, _remaining)) = transfer_result {
            if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {
                transfer_ok = true;
                ep0_enqueue = 3;
            }
        }

        if transfer_ok {
            // Invalidate descriptor buffer after xHCI DMA write
            invalidate_buffer(desc_buf as u64, 18);
            unsafe {
                let desc = &*(desc_buf as *const DeviceDescriptor);

                // Print device class type
                if desc.device_class == 8 {
                    println!(", Mass Storage");
                } else if desc.device_class == 9 {
                    println!(", Hub");
                    self.enumerate_hub(slot_id, port, ep0_ring_virt, ep0_ring_phys, &mut ep0_enqueue);
                } else {
                    println!();
                }
            }
        } else {
            println!(" - Descriptor failed");
        }

        Some(slot_id)
    }

    /// Perform a control transfer on the given slot's EP0
    /// Returns the completion code and bytes transferred
    /// ep0_enqueue tracks the current position in the ring (updated after transfer)
    fn control_transfer(
        &mut self,
        slot_id: u32,
        ep0_ring_virt: *mut Trb,
        ep0_ring_phys: u64,
        ep0_enqueue: &mut usize,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        data_buf_phys: u64,
        length: u16,
    ) -> Option<(u32, u32)> {
        // Build setup data (8 bytes)
        let setup_data: u64 =
            (request_type as u64) |
            ((request as u64) << 8) |
            ((value as u64) << 16) |
            ((index as u64) << 32) |
            ((length as u64) << 48);

        let is_in = (request_type & 0x80) != 0;
        let has_data = length > 0;

        // Check if we have enough space before link TRB (index 63)
        // Need up to 3 TRBs (Setup, Data, Status)
        let trbs_needed = if has_data { 3 } else { 2 };
        if *ep0_enqueue + trbs_needed >= 63 {
            // Wrap around - reset to beginning
            // Clear old TRBs and reset enqueue pointer
            unsafe {
                for i in 0..63 {
                    let trb = &mut *ep0_ring_virt.add(i);
                    trb.param = 0;
                    trb.status = 0;
                    trb.control = 0;
                }
                // Re-setup link TRB
                let link = &mut *ep0_ring_virt.add(63);
                link.param = ep0_ring_phys;
                link.status = 0;
                link.control = (trb_type::LINK << 10) | (1 << 5) | 1;  // Toggle cycle, cycle=1

                // Flush EP0 ring to memory after reset
                flush_buffer(ep0_ring_virt as u64, 64 * core::mem::size_of::<Trb>());
            }
            *ep0_enqueue = 0;
        }

        let start_idx = *ep0_enqueue;

        unsafe {
            // Setup TRB
            let setup_trb = &mut *ep0_ring_virt.add(start_idx);
            setup_trb.param = setup_data;
            setup_trb.status = 8;
            let trt = if !has_data { 0 } else if is_in { 3 } else { 2 };
            setup_trb.control = (trb_type::SETUP << 10) | (trt << 16) | (1 << 6) | 1;

            let mut next_idx = start_idx + 1;

            // Data Stage TRB (if needed)
            if has_data {
                let data_trb = &mut *ep0_ring_virt.add(next_idx);
                data_trb.param = data_buf_phys;
                data_trb.status = length as u32;
                let dir = if is_in { 1 << 16 } else { 0 };
                data_trb.control = (trb_type::DATA << 10) | dir | 1;
                next_idx += 1;
            }

            // Status Stage TRB
            let status_trb = &mut *ep0_ring_virt.add(next_idx);
            status_trb.param = 0;
            status_trb.status = 0;
            // DIR is opposite of data direction (or IN if no data)
            let status_dir = if has_data && is_in { 0 } else { 1 << 16 };
            status_trb.control = (trb_type::STATUS << 10) | status_dir | (1 << 5) | 1;

            *ep0_enqueue = next_idx + 1;

            // Flush TRBs to memory before doorbell
            let trbs_written = next_idx + 1 - start_idx;
            flush_buffer(ep0_ring_virt.add(start_idx) as u64, trbs_written * core::mem::size_of::<Trb>());
        }

        // Ring doorbell for EP0
        self.ring_doorbell(slot_id, 1);

        // Wait for completion - longer wait for hub devices
        let initial_wait = if slot_id == 2 { 100000 } else { 50000 };
        delay(initial_wait);

        // Poll for transfer event
        let mut result: Option<(u32, u32)> = None;
        let mut erdp_to_update: Option<u64> = None;

        // Use more iterations for downstream hub devices
        let max_iterations = if slot_id == 2 { 300 } else { 100 };

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..max_iterations {
                if let Some(evt) = event_ring.dequeue() {
                    let evt_type = evt.get_type();
                    erdp_to_update = Some(event_ring.erdp());

                    if evt_type == trb_type::TRANSFER_EVENT {
                        let cc = (evt.status >> 24) & 0xFF;
                        let remaining = evt.status & 0xFFFFFF;
                        result = Some((cc, remaining));
                        break;
                    }
                }
                delay(1000);
            }
        }

        // If no events found but EINT is set, retry with extra delay
        if result.is_none() {
            let usbsts = self.op_read32(xhci_op::USBSTS);
            if (usbsts & (1 << 3)) != 0 {
                delay(50000);
                if let Some(ref mut event_ring) = self.event_ring {
                    for _ in 0..100 {
                        if let Some(evt) = event_ring.dequeue() {
                            let evt_type = evt.get_type();
                            erdp_to_update = Some(event_ring.erdp());

                            if evt_type == trb_type::TRANSFER_EVENT {
                                let cc = (evt.status >> 24) & 0xFF;
                                let remaining = evt.status & 0xFFFFFF;
                                result = Some((cc, remaining));
                                break;
                            }
                        }
                        delay(1000);
                    }
                }
            }
        }

        // Update ERDP outside the borrow
        if let Some(erdp) = erdp_to_update {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
        }

        if let Some((cc, remaining)) = result {
            let transferred = (length as u32).saturating_sub(remaining);
            Some((cc, transferred))
        } else {
            None
        }
    }

    /// Send SET_CONFIGURATION request (silent unless error)
    fn set_configuration(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, config: u8) -> bool {
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            0x00,  // Host-to-device, standard, device
            usb_req::SET_CONFIGURATION,
            config as u16,
            0, 0, 0
        ) {
            Some((cc, _)) if cc == trb_cc::SUCCESS => true,
            _ => false
        }
    }

    /// Get Hub Descriptor (SuperSpeed) - silent
    fn get_hub_descriptor(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, buf_phys: u64) -> Option<u8> {
        let setup = get_hub_descriptor_setup(true);
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            setup.request_type, setup.request, setup.value, setup.index,
            buf_phys, setup.length
        ) {
            Some((cc, len)) if completion_code::is_success(cc) => Some(len as u8),
            _ => None
        }
    }

    /// Get port status from hub
    fn hub_get_port_status(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, port: u16, buf_virt: u64, buf_phys: u64) -> Option<PortStatus> {
        let setup = get_port_status_setup(port);
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            setup.request_type,
            setup.request,
            setup.value,
            setup.index,
            buf_phys,
            setup.length
        ) {
            Some((cc, _)) if completion_code::is_success(cc) => {
                // Invalidate port status buffer after xHCI DMA write
                invalidate_buffer(buf_virt, core::mem::size_of::<PortStatus>());
                let ps = unsafe { *(buf_virt as *const PortStatus) };
                Some(ps)
            }
            _ => None
        }
    }

    /// Set a port feature
    fn hub_set_port_feature(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, port: u16, feature: u16) -> bool {
        let setup = set_port_feature_setup(port, feature);
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            setup.request_type,
            setup.request,
            setup.value,
            setup.index,
            0, 0
        ) {
            Some((cc, _)) if cc == trb_cc::SUCCESS => true,
            _ => false
        }
    }

    /// Clear a port feature
    fn hub_clear_port_feature(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, port: u16, feature: u16) -> bool {
        let setup = clear_port_feature_setup(port, feature);
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            setup.request_type,
            setup.request,
            setup.value,
            setup.index,
            0, 0
        ) {
            Some((cc, _)) if cc == trb_cc::SUCCESS => true,
            _ => false
        }
    }

    /// Enumerate devices connected through a hub
    /// Enumerate hub - mostly silent, only prints port count and connected devices
    fn enumerate_hub(&mut self, hub_slot: u32, hub_port: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize) {
        // Allocate buffer for hub descriptor and port status
        let mut buf_phys: u64 = 0;
        let buf_virt = syscall::mmap_dma(4096, &mut buf_phys);
        if buf_virt < 0 {
            return;
        }

        // Set configuration to activate hub
        if !self.set_configuration(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, 1) {
            return;
        }

        // Set hub depth (required for USB3 hubs)
        let _ = self.control_transfer(
            hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            0x20, hub::SET_HUB_DEPTH, 0, 0, 0, 0
        );

        // Get hub descriptor to find port count
        let hub_desc_len = match self.get_hub_descriptor(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, buf_phys) {
            Some(len) => len,
            None => return,
        };

        // Invalidate hub descriptor buffer after xHCI DMA write
        invalidate_buffer(buf_virt as u64, hub_desc_len as usize);
        let hub_desc = unsafe { &*(buf_virt as *const SsHubDescriptor) };
        let num_ports = hub_desc.num_ports;
        let pwr_time = hub_desc.pwr_on_2_pwr_good as u32 * 2;
        println!("    {} ports", num_ports);

        // Update hub's Slot Context via EVALUATE_CONTEXT
        {
            let mut eval_ctx_phys: u64 = 0;
            let eval_ctx_virt = syscall::mmap_dma(4096, &mut eval_ctx_phys);
            if eval_ctx_virt >= 0 {
                unsafe {
                    let input = eval_ctx_virt as *mut InputContext;
                    *input = InputContext::new();
                    (*input).control.add_flags = 0x1;
                    (*input).slot.set_hub(true);
                    (*input).slot.set_speed(4);
                    (*input).slot.set_context_entries(1);
                    (*input).slot.set_root_hub_port(hub_port);
                    (*input).slot.set_num_ports(num_ports as u32);
                }

                if let Some(ref mut cmd_ring) = self.cmd_ring {
                    let mut trb = Trb::new();
                    trb.param = eval_ctx_phys;
                    trb.set_type(trb_type::EVALUATE_CONTEXT);
                    trb.control |= (hub_slot & 0xFF) << 24;
                    cmd_ring.enqueue(&trb);
                    self.ring_doorbell(0, 0);
                    delay(10000);

                    let mut erdp_to_update: Option<u64> = None;
                    if let Some(ref mut event_ring) = self.event_ring {
                        for _ in 0..20 {
                            if let Some(evt) = event_ring.dequeue() {
                                erdp_to_update = Some(event_ring.erdp());
                                if evt.get_type() == trb_type::COMMAND_COMPLETION {
                                    break;
                                }
                            }
                            delay(1000);
                        }
                    }
                    if let Some(erdp) = erdp_to_update {
                        self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
                    }
                }
            }
        }

        // Power on all ports (silent)
        for port in 1..=num_ports as u16 {
            self.hub_set_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, hub::PORT_POWER);
        }

        // Wait for power stabilization (use hub-specified time, minimum 100ms)
        let wait_time = (pwr_time as u32 * 2).max(100);
        delay_ms(wait_time);

        // Poll for device connection (USB 3.0 link training can take time)
        // Check each port multiple times over ~2 seconds
        let buf_virt_u64 = buf_virt as u64;
        let mut ports_to_check: u16 = (1 << num_ports) - 1;  // Bitmask of ports to check

        for poll in 0..10 {
            if ports_to_check == 0 {
                break;  // All ports checked
            }

            for port in 1..=num_ports as u16 {
                let port_mask = 1 << (port - 1);
                if (ports_to_check & port_mask) == 0 {
                    continue;  // Already found device on this port
                }

                if let Some(ps) = self.hub_get_port_status(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, buf_virt_u64, buf_phys) {
                    if poll == 9 || (ps.status & hub::PS_CONNECTION) != 0 {
                        // Final poll or device found - print status
                        if poll == 9 {
                            println!("    Hub port {}: status=0x{:04x} change=0x{:04x}", port, ps.status, ps.change);
                        }
                        ports_to_check &= !port_mask;  // Mark port as checked
                    }
                }
            }

            if ports_to_check != 0 {
                delay_ms(200);  // Wait before next poll
            }
        }

        // Now enumerate devices on connected ports
        for port in 1..=num_ports as u16 {
            if let Some(ps) = self.hub_get_port_status(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, buf_virt_u64, buf_phys) {
                println!("    Hub port {}: status=0x{:04x} change=0x{:04x}", port, ps.status, ps.change);
                if (ps.status & hub::PS_CONNECTION) != 0 {
                    println!("      -> Device connected, resetting...");
                    // Clear connection change
                    self.hub_clear_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, hub::C_PORT_CONNECTION);

                    // Reset the port
                    if !self.hub_set_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, hub::PORT_RESET) {
                        println!("      -> Reset failed");
                        continue;
                    }

                    // Wait for reset to complete
                    let mut reset_ok = false;
                    for _ in 0..50 {
                        delay(10000);
                        if let Some(ps2) = self.hub_get_port_status(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, buf_virt_u64, buf_phys) {
                            if (ps2.change & hub::PS_C_RESET) != 0 {
                                self.hub_clear_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, hub::C_PORT_RESET);
                                if (ps2.status & hub::PS_ENABLE) != 0 {
                                    println!("      -> Port enabled, enumerating device");
                                    for _ in 0..100 {
                                        delay(1000);
                                    }
                                    self.enumerate_hub_device(hub_slot, hub_port, port as u32);
                                    reset_ok = true;
                                } else {
                                    println!("      -> Reset complete but port not enabled");
                                }
                                break;
                            }
                        }
                    }
                    if !reset_ok {
                        println!("      -> Reset timeout");
                    }
                }
            } else {
                println!("    Hub port {}: failed to get status", port);
            }
        }
    }

    /// Enumerate a device connected to a hub port (mostly silent)
    fn enumerate_hub_device(&mut self, hub_slot: u32, root_port: u32, hub_port: u32) {
        // Enable a new slot for this device
        let slot_id = match self.enable_slot() {
            Some(id) => id,
            None => return,
        };

        // Allocate context memory
        let mut ctx_phys: u64 = 0;
        let ctx_virt = syscall::mmap_dma(4096, &mut ctx_phys);
        if ctx_virt < 0 {
            return;
        }

        let input_ctx_virt = ctx_virt as *mut InputContext;
        let input_ctx_phys = ctx_phys;
        let device_ctx_virt = (ctx_virt as u64 + 0x500) as *mut DeviceContext;
        let device_ctx_phys = ctx_phys + 0x500;
        let ep0_ring_virt = (ctx_virt as u64 + 0x900) as *mut Trb;
        let ep0_ring_phys = ctx_phys + 0x900;

        // Initialize EP0 transfer ring
        unsafe {
            for i in 0..64 {
                *ep0_ring_virt.add(i) = Trb::new();
            }
            let link = &mut *ep0_ring_virt.add(63);
            link.param = ep0_ring_phys;
            link.set_type(trb_type::LINK);
            link.control |= 1 << 5;
            // Flush EP0 ring to memory
            flush_buffer(ep0_ring_virt as u64, 64 * core::mem::size_of::<Trb>());
        }

        // Assume SuperSpeed since we came from a USB3 hub
        let speed = 4u32;
        let max_packet_size = 512u32;

        // Initialize Input Context for Address Device command
        // Add flags 0x3 = add Slot Context (bit 0) and EP0 Context (bit 1)
        unsafe {
            let input = &mut *input_ctx_virt;
            *input = InputContext::new();
            input.control.add_flags = 0x3;

            // Slot Context: route string identifies path through hub topology
            // For tier-1 hub device, route string = hub port number (1-15)
            input.slot.set_route_string(hub_port);
            input.slot.set_speed(speed);
            input.slot.set_context_entries(1);
            input.slot.set_root_hub_port(root_port);

            // dw2: Hub Slot ID and Parent Port (only for LS/FS behind HS hub)
            // SuperSpeed devices use route string for routing, not dw2
            input.slot.dw2 = if speed <= 2 { (hub_slot << 0) | (hub_port << 8) } else { 0 };

            // EP0 Context: control endpoint with max packet size for speed
            input.endpoints[0].set_ep_type(ep_type::CONTROL);
            input.endpoints[0].set_max_packet_size(max_packet_size);
            input.endpoints[0].set_cerr(3);  // 3 retries on error
            input.endpoints[0].set_tr_dequeue_ptr(ep0_ring_phys, true);  // DCS=1
            input.endpoints[0].set_average_trb_length(8);

            // Flush input context to memory
            flush_buffer(input_ctx_virt as u64, core::mem::size_of::<InputContext>());

            // Initialize output Device Context (xHCI will write to this)
            let device = &mut *device_ctx_virt;
            *device = DeviceContext::new();
            // Flush device context to memory
            flush_buffer(device_ctx_virt as u64, core::mem::size_of::<DeviceContext>());
        }

        // Set DCBAA entry - xHCI uses this to find the device's output context
        let dcbaa = self.xhci_mem as *mut u64;
        unsafe {
            *dcbaa.add(slot_id as usize) = device_ctx_phys;
            // Flush DCBAA entry
            flush_cache_line(dcbaa.add(slot_id as usize) as u64);
            dsb();
        }

        // Wait 200ms before addressing - device needs time after reset
        for _ in 0..200 {
            delay(1000);
        }

        // Address Device command
        {
            let cmd_ring = match self.cmd_ring.as_mut() {
                Some(r) => r,
                None => return,
            };
            let mut trb = Trb::new();
            trb.param = input_ctx_phys;
            trb.set_type(trb_type::ADDRESS_DEVICE);
            trb.control |= (slot_id & 0xFF) << 24;
            cmd_ring.enqueue(&trb);
            self.ring_doorbell(0, 0);
        }

        for _ in 0..100 {
            delay(1000);
        }

        // Poll for Address Device completion
        let mut addr_result: Option<u32> = None;
        let mut erdp_to_update: Option<u64> = None;

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..50 {
                if let Some(evt) = event_ring.dequeue() {
                    erdp_to_update = Some(event_ring.erdp());
                    if evt.get_type() == trb_type::COMMAND_COMPLETION {
                        let cc = (evt.status >> 24) & 0xFF;
                        addr_result = Some(cc);
                        break;
                    }
                }
                delay(1000);
            }
        }

        if let Some(erdp) = erdp_to_update {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
        }

        let addr_ok = match addr_result {
            Some(cc) if cc == trb_cc::SUCCESS => true,
            _ => false,
        };

        if !addr_ok {
            return;
        }

        // Wait for device ready
        for _ in 0..100 {
            delay(1000);
        }

        // Get Device Descriptor
        let mut desc_phys: u64 = 0;
        let desc_virt = syscall::mmap_dma(4096, &mut desc_phys);
        if desc_virt < 0 {
            return;
        }

        let mut dev_ep0_enqueue = 0usize;

        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, &mut dev_ep0_enqueue,
            0x80, usb_req::GET_DESCRIPTOR, (usb_req::DESC_DEVICE << 8) as u16,
            0, desc_phys, 18
        ) {
            Some((cc, _)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
                let desc = unsafe { &*(desc_virt as *const DeviceDescriptor) };
                print!("  Hub port {}: ", hub_port);
                if desc.device_class == 8 || desc.device_class == 0 {
                    println!("Mass Storage");
                    self.configure_mass_storage(
                        slot_id, ep0_ring_virt, ep0_ring_phys, &mut dev_ep0_enqueue,
                        desc_virt as *mut u8, desc_phys, device_ctx_virt
                    );
                } else {
                    println!("Device class {}", desc.device_class);
                }
            }
            _ => {}
        }
    }

    /// Configure a mass storage device (silent unless error)
    fn configure_mass_storage(
        &mut self,
        slot_id: u32,
        ep0_ring_virt: *mut Trb,
        ep0_ring_phys: u64,
        ep0_enqueue: &mut usize,
        buf_virt: *mut u8,
        buf_phys: u64,
        device_ctx_virt: *mut DeviceContext,
    ) {
        // Get Configuration Descriptor (first 9 bytes to get total length)
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            0x80, usb_req::GET_DESCRIPTOR, (usb_req::DESC_CONFIGURATION << 8) as u16,
            0, buf_phys, 9
        ) {
            Some((cc, _)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
                // Invalidate first 9 bytes of config descriptor
                invalidate_buffer(buf_virt as u64, 9);
                let config = unsafe { &*(buf_virt as *const ConfigurationDescriptor) };
                let total_len = config.total_length;

                if total_len > 9 && total_len <= 256 {
                    delay(10000);
                    match self.control_transfer(
                        slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
                        0x80, usb_req::GET_DESCRIPTOR, (usb_req::DESC_CONFIGURATION << 8) as u16,
                        0, buf_phys, total_len
                    ) {
                        Some((cc, _)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
                            // Invalidate full config descriptor
                            invalidate_buffer(buf_virt as u64, total_len as usize);
                            self.parse_config_descriptor(
                                slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
                                buf_virt, total_len as usize, device_ctx_virt
                            );
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }

    /// Parse configuration descriptor and set up mass storage if found (silent)
    fn parse_config_descriptor(
        &mut self,
        slot_id: u32,
        ep0_ring_virt: *mut Trb,
        ep0_ring_phys: u64,
        ep0_enqueue: &mut usize,
        buf: *mut u8,
        total_len: usize,
        device_ctx_virt: *mut DeviceContext,
    ) {
        let mut offset = 0usize;
        let mut config_value = 0u8;
        let mut bulk_in_ep = 0u8;
        let mut bulk_out_ep = 0u8;
        let mut bulk_in_max_packet = 0u16;
        let mut bulk_out_max_packet = 0u16;
        let mut is_mass_storage = false;
        let mut msc_interface_num = 0u8;

        while offset + 2 <= total_len {
            let len = unsafe { *buf.add(offset) } as usize;
            let desc_type = unsafe { *buf.add(offset + 1) };

            if len < 2 || offset + len > total_len {
                break;
            }

            match desc_type {
                2 => {
                    if len >= 9 {
                        let config = unsafe { &*(buf.add(offset) as *const ConfigurationDescriptor) };
                        config_value = config.configuration_value;
                    }
                }
                4 => {
                    if len >= 9 {
                        let iface = unsafe { &*(buf.add(offset) as *const InterfaceDescriptor) };
                        if iface.interface_class == msc::CLASS &&
                           iface.interface_subclass == msc::SUBCLASS_SCSI &&
                           iface.interface_protocol == msc::PROTOCOL_BOT {
                            is_mass_storage = true;
                            msc_interface_num = iface.interface_number;
                        }
                    }
                }
                5 => {
                    if len >= 7 {
                        let ep_addr = unsafe { core::ptr::read_unaligned(buf.add(offset + 2)) };
                        let ep_attr = unsafe { core::ptr::read_unaligned(buf.add(offset + 3)) };
                        let ep_max_pkt = unsafe { core::ptr::read_unaligned(buf.add(offset + 4) as *const u16) };

                        if is_mass_storage && (ep_attr & 0x03) == 2 {
                            if (ep_addr & 0x80) != 0 {
                                bulk_in_ep = ep_addr;
                                bulk_in_max_packet = ep_max_pkt;
                            } else {
                                bulk_out_ep = ep_addr;
                                bulk_out_max_packet = ep_max_pkt;
                            }
                        }
                    }
                }
                _ => {}
            }
            offset += len;
        }

        if is_mass_storage && bulk_in_ep != 0 && bulk_out_ep != 0 {
            if self.set_configuration(slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, config_value) {
                delay(50000);
            }
            self.setup_bulk_endpoints(
                slot_id, device_ctx_virt, bulk_in_ep, bulk_in_max_packet, bulk_out_ep, bulk_out_max_packet,
                msc_interface_num, ep0_ring_virt, ep0_ring_phys, *ep0_enqueue
            );
        }
    }

    /// Set up bulk endpoints for mass storage (silent)
    fn setup_bulk_endpoints(
        &mut self,
        slot_id: u32,
        device_ctx_virt: *mut DeviceContext,
        bulk_in_ep: u8,
        bulk_in_max_packet: u16,
        bulk_out_ep: u8,
        bulk_out_max_packet: u16,
        interface_num: u8,
        ep0_ring_virt: *mut Trb,
        ep0_ring_phys: u64,
        ep0_enqueue: usize,
    ) {
        // Allocate transfer rings for bulk endpoints
        let mut bulk_in_ring_phys: u64 = 0;
        let bulk_in_ring_virt = syscall::mmap_dma(4096, &mut bulk_in_ring_phys);
        if bulk_in_ring_virt < 0 { return; }

        let mut bulk_out_ring_phys: u64 = 0;
        let bulk_out_ring_virt = syscall::mmap_dma(4096, &mut bulk_out_ring_phys);
        if bulk_out_ring_virt < 0 { return; }

        // Initialize transfer rings with Link TRBs at index 255 (for 256-entry rings)
        // Note: Ring::init() uses RING_SIZE=64, but bulk rings need 256 entries
        unsafe {
            use usb::transfer::{flush_cache_line, dsb};
            use usb::trb::trb_type;

            // Initialize bulk IN ring (256 TRBs, Link at index 255)
            let in_ring = bulk_in_ring_virt as *mut Trb;
            for i in 0..256 {
                *in_ring.add(i) = Trb::new();
            }
            // Set up Link TRB at index 255
            let in_link = &mut *in_ring.add(255);
            in_link.param = bulk_in_ring_phys;  // Points back to start
            in_link.set_type(trb_type::LINK);
            in_link.control |= 1 << 1;  // Toggle Cycle bit (TC)
            // Flush entire ring
            for i in 0..256 {
                flush_cache_line(in_ring.add(i) as u64);
            }
            dsb();

            // Initialize bulk OUT ring (256 TRBs, Link at index 255)
            let out_ring = bulk_out_ring_virt as *mut Trb;
            for i in 0..256 {
                *out_ring.add(i) = Trb::new();
            }
            // Set up Link TRB at index 255
            let out_link = &mut *out_ring.add(255);
            out_link.param = bulk_out_ring_phys;  // Points back to start
            out_link.set_type(trb_type::LINK);
            out_link.control |= 1 << 1;  // Toggle Cycle bit (TC)
            // Flush entire ring
            for i in 0..256 {
                flush_cache_line(out_ring.add(i) as u64);
            }
            dsb();
        }

        // Calculate Device Context Index (DCI) for each endpoint
        // DCI = (endpoint_number * 2) + direction (0=OUT, 1=IN)
        let bulk_in_num = (bulk_in_ep & 0x0F) as u32;
        let bulk_out_num = (bulk_out_ep & 0x0F) as u32;
        let bulk_in_dci = bulk_in_num * 2 + 1;   // IN endpoint
        let bulk_out_dci = bulk_out_num * 2;     // OUT endpoint

        // Allocate Input Context for Configure Endpoint command
        let mut input_ctx_phys: u64 = 0;
        let input_ctx_virt = syscall::mmap_dma(4096, &mut input_ctx_phys);
        if input_ctx_virt < 0 { return; }

        // Set up Input Context for Configure Endpoint
        unsafe {
            let input = &mut *(input_ctx_virt as *mut InputContext);
            core::ptr::write_bytes(input, 0, 1);

            // Add flags: bit 0 = slot, bit N = endpoint DCI N
            input.control.add_flags = 1 | (1 << bulk_in_dci) | (1 << bulk_out_dci);

            // Copy current slot context and update context entries count
            let dev_ctx = &*device_ctx_virt;
            input.slot = dev_ctx.slot;
            let max_dci = bulk_in_dci.max(bulk_out_dci);
            input.slot.set_context_entries(max_dci);

            // Configure Bulk IN endpoint
            let bulk_in_idx = (bulk_in_dci - 1) as usize;
            input.endpoints[bulk_in_idx].set_ep_type(ep_type::BULK_IN);
            input.endpoints[bulk_in_idx].set_max_packet_size(bulk_in_max_packet as u32);
            input.endpoints[bulk_in_idx].set_max_burst_size(0);
            input.endpoints[bulk_in_idx].set_cerr(3);
            input.endpoints[bulk_in_idx].set_tr_dequeue_ptr(bulk_in_ring_phys, true);
            input.endpoints[bulk_in_idx].set_average_trb_length(1024);

            // Configure Bulk OUT endpoint
            let bulk_out_idx = (bulk_out_dci - 1) as usize;
            input.endpoints[bulk_out_idx].set_ep_type(ep_type::BULK_OUT);
            input.endpoints[bulk_out_idx].set_max_packet_size(bulk_out_max_packet as u32);
            input.endpoints[bulk_out_idx].set_max_burst_size(0);
            input.endpoints[bulk_out_idx].set_cerr(3);
            input.endpoints[bulk_out_idx].set_tr_dequeue_ptr(bulk_out_ring_phys, true);
            input.endpoints[bulk_out_idx].set_average_trb_length(1024);

            // Flush input context to memory
            flush_buffer(input_ctx_virt as u64, core::mem::size_of::<InputContext>());
        }

        // Issue Configure Endpoint command
        let mut cmd = Trb::new();
        cmd.param = input_ctx_phys;
        cmd.status = 0;
        cmd.control = slot_id << 24;
        cmd.set_type(trb_type::CONFIGURE_ENDPOINT);

        if let Some(ref mut cmd_ring) = self.cmd_ring {
            cmd_ring.enqueue(&cmd);
        }
        self.ring_doorbell(0, 0);
        delay(50000);

        // Poll for completion
        let mut config_ok = false;
        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..100 {
                if let Some(evt) = event_ring.dequeue() {
                    if evt.get_type() == trb_type::COMMAND_COMPLETION {
                        let cc = (evt.status >> 24) & 0xFF;
                        config_ok = cc == trb_cc::SUCCESS;
                        let erdp = event_ring.erdp() | (1 << 3);
                        self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp);
                        break;
                    }
                }
                delay(1000);
            }
        }

        if !config_ok { return; }

        // Invalidate device context after xHCI wrote to it
        invalidate_buffer(device_ctx_virt as u64, core::mem::size_of::<DeviceContext>());

        // Allocate data buffer for SCSI commands
        let mut data_phys: u64 = 0;
        let data_virt = syscall::mmap_dma(4096, &mut data_phys);
        if data_virt < 0 { return; }

        let mut ctx = BulkContext::new(
            slot_id, bulk_in_dci, bulk_out_dci,
            bulk_in_ring_virt as *mut Trb, bulk_in_ring_phys,
            bulk_out_ring_virt as *mut Trb, bulk_out_ring_phys,
            data_virt as *mut u8, data_phys, device_ctx_virt,
            bulk_in_ep, bulk_out_ep, interface_num,
            ep0_ring_virt, ep0_ring_phys,
        );
        ctx.ep0_enqueue = ep0_enqueue;
        self.run_scsi_tests(&mut ctx);
    }

    // =========================================================================
    // SCSI command helpers (BulkContext-based)
    // =========================================================================

    /// Send CBW (Command Block Wrapper) on bulk OUT and wait for completion
    /// Returns true if CBW was sent successfully
    fn send_cbw(&mut self, ctx: &mut BulkContext, cbw: &Cbw) -> bool {
        // Write CBW to buffer
        unsafe {
            let cbw_ptr = ctx.data_buf.add(CBW_OFFSET) as *mut Cbw;
            core::ptr::write_volatile(cbw_ptr, *cbw);
            ctx.flush_buffer(cbw_ptr as u64, 31);
        }

        // Create TRB for CBW (31 bytes) with proper cycle bit
        let (idx, cycle) = ctx.advance_out();
        unsafe {
            let trb = &mut *ctx.bulk_out_ring.add(idx);
            trb.param = ctx.data_phys + CBW_OFFSET as u64;
            trb.status = 31;  // CBW is 31 bytes
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;  // IOC, cycle
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }

        // Ring doorbell
        self.ring_doorbell(ctx.slot_id, ctx.bulk_out_dci);

        // Get ERDP register address for direct writes (avoids borrow conflict)
        let erdp_addr = self.xhci.mmio_base() + self.xhci.rt_offset() as u64
            + (xhci_rt::IR0 + xhci_ir::ERDP) as u64;

        // Poll for completion (CBW is fast, no need for IRQ)
        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..100 {
                if let Some(evt) = event_ring.dequeue() {
                    let evt_type = evt.get_type();
                    let cc = (evt.status >> 24) & 0xFF;
                    // Update ERDP for all events
                    let erdp = event_ring.erdp() | (1 << 3);
                    unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }
                    if evt_type == trb_type::TRANSFER_EVENT {
                        return cc == trb_cc::SUCCESS;
                    }
                }
                syscall::yield_now();
            }
        }
        false
    }

    /// Wait for a transfer event via IRQ
    /// Returns (success, completion_code)
    fn wait_transfer_irq(&mut self, timeout_ms: u32) -> (TransferResult, u64) {
        let mac_base = self.xhci.mmio_base();
        let rt_base = self.xhci.rt_offset();
        let op_base = self.xhci.op_offset();

        // Clear IMAN.IP before waiting
        unsafe {
            let iman_addr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *mut u32;
            let iman = core::ptr::read_volatile(iman_addr);
            if (iman & 1) != 0 {
                core::ptr::write_volatile(iman_addr, iman | 1);
            }

            // Clear EINT in USBSTS
            let usbsts_addr = (mac_base + op_base as u64 + xhci_op::USBSTS as u64) as *mut u32;
            let usbsts = core::ptr::read_volatile(usbsts_addr);
            if (usbsts & 0x8) != 0 {
                core::ptr::write_volatile(usbsts_addr, 0x8);
            }
        }

        // Wait for IRQ with retry loop
        let max_irq_waits = 10;
        for _irq_attempt in 0..max_irq_waits {
            if !self.wait_for_irq(timeout_ms) {
                return (TransferResult::Timeout, 0);
            }

            // Check event ring
            if let Some(ref mut event_ring) = self.event_ring {
                if let Some(evt) = event_ring.dequeue() {
                    let evt_type = evt.get_type();
                    let cc = (evt.status >> 24) & 0xFF;

                    // Update ERDP
                    let erdp = event_ring.erdp() | (1 << 3);
                    unsafe {
                        let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
                        core::ptr::write_volatile(ptr, erdp);
                    }

                    if evt_type == trb_type::TRANSFER_EVENT {
                        // Get residue from TRB pointer field bits [23:0] of status
                        let residue = evt.status & 0xFFFFFF;
                        return match cc {
                            x if x == trb_cc::SUCCESS => (TransferResult::Success, residue as u64),
                            x if x == trb_cc::SHORT_PACKET => (TransferResult::ShortPacket, residue as u64),
                            _ => (TransferResult::Error(cc), residue as u64),
                        };
                    }
                    // Got non-transfer event, continue waiting
                }
                // No event yet, continue waiting
            }
        }
        (TransferResult::Timeout, 0)
    }

    /// Receive data on bulk IN using IRQ-based waiting
    /// Returns (result, bytes_transferred)
    fn receive_bulk_in_irq(&mut self, ctx: &mut BulkContext, offset: usize, length: usize) -> (TransferResult, usize) {
        let phys_addr = ctx.data_phys + offset as u64;
        let virt_addr = ctx.data_buf as u64 + offset as u64;

        // Invalidate destination buffer before DMA (uses VIRTUAL address for cache ops)
        ctx.invalidate_buffer(virt_addr, length);

        // Create TRB for bulk IN with proper cycle bit
        let (idx, cycle) = ctx.advance_in();
        unsafe {
            let trb = &mut *ctx.bulk_in_ring.add(idx);
            trb.param = phys_addr;
            trb.status = length as u32;
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;  // IOC, cycle
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }

        // Ring doorbell
        self.ring_doorbell(ctx.slot_id, ctx.bulk_in_dci);

        // Wait for transfer via IRQ
        let (result, residue) = self.wait_transfer_irq(5000);

        // Calculate bytes transferred
        let bytes = if residue as usize <= length {
            length - residue as usize
        } else {
            length
        };

        (result, bytes)
    }

    /// Receive bulk IN data directly to an external physical address (zero-copy DMA)
    /// For use by other processes that pass their own DMA buffer
    fn receive_bulk_in_dma(&mut self, ctx: &mut BulkContext, target_phys: u64, length: usize) -> (TransferResult, usize) {
        // NOTE: We don't invalidate cache here - the target is in another process's address space
        // The client is responsible for cache management on their buffer

        // Create TRB for bulk IN with proper cycle bit
        let (idx, cycle) = ctx.advance_in();
        unsafe {
            let trb = &mut *ctx.bulk_in_ring.add(idx);
            trb.param = target_phys;  // Direct to client's DMA buffer
            trb.status = length as u32;
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;  // IOC, cycle
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }

        // Ring doorbell
        self.ring_doorbell(ctx.slot_id, ctx.bulk_in_dci);

        // Wait for transfer via IRQ
        let (result, residue) = self.wait_transfer_irq(5000);

        // Calculate bytes transferred
        let bytes = if residue as usize <= length {
            length - residue as usize
        } else {
            length
        };

        (result, bytes)
    }

    /// Execute SCSI command with data IN going directly to external DMA buffer (zero-copy)
    /// CBW and CSW use internal buffer, only data phase uses client's buffer
    fn scsi_command_in_dma(&mut self, ctx: &mut BulkContext, cmd: &[u8], target_phys: u64, data_length: u32) -> (TransferResult, Option<Csw>) {
        let tag = ctx.next_tag();

        // Build CBW
        let cbw = Cbw::new(tag, data_length, true, 0, cmd);

        // Send CBW (uses internal buffer)
        if !self.send_cbw(ctx, &cbw) {
            return (TransferResult::Error(0), None);
        }

        // Data phase - goes directly to client's DMA buffer
        if data_length > 0 {
            let (result, _bytes) = self.receive_bulk_in_dma(ctx, target_phys, data_length as usize);
            if result != TransferResult::Success && result != TransferResult::ShortPacket {
                return (result, None);
            }
        }

        // CSW phase (uses internal buffer)
        self.receive_csw_irq(ctx)
    }

    /// Receive CSW (Command Status Wrapper) on bulk IN
    /// Returns (result, CSW)
    fn receive_csw_irq(&mut self, ctx: &mut BulkContext) -> (TransferResult, Option<Csw>) {
        let (result, _bytes) = self.receive_bulk_in_irq(ctx, CSW_OFFSET, 13);

        if result == TransferResult::Success || result == TransferResult::ShortPacket {
            // Read CSW from buffer
            unsafe {
                let csw_ptr = ctx.data_buf.add(CSW_OFFSET) as *const Csw;
                ctx.invalidate_buffer(csw_ptr as u64, 13);
                let csw = core::ptr::read_volatile(csw_ptr);
                return (result, Some(csw));
            }
        }

        (result, None)
    }

    /// Execute a complete SCSI command: CBW -> optional Data IN -> CSW
    /// For commands with data phase, data is placed at DATA_OFFSET in the buffer
    fn scsi_command_in(&mut self, ctx: &mut BulkContext, cmd: &[u8], data_length: u32) -> (TransferResult, Option<Csw>) {
        let tag = ctx.next_tag();

        // Build CBW
        let cbw = Cbw::new(tag, data_length, true, 0, cmd);

        // Send CBW
        if !self.send_cbw(ctx, &cbw) {
            return (TransferResult::Error(0), None);
        }

        // Data phase (if any)
        if data_length > 0 {
            let (result, _bytes) = self.receive_bulk_in_irq(ctx, DATA_OFFSET, data_length as usize);
            if result != TransferResult::Success && result != TransferResult::ShortPacket {
                return (result, None);
            }
        }

        // CSW phase
        self.receive_csw_irq(ctx)
    }

    /// Execute a complete SCSI command with data OUT: CBW -> Data OUT -> CSW
    fn scsi_command_out(&mut self, ctx: &mut BulkContext, cmd: &[u8], data: &[u8]) -> (TransferResult, Option<Csw>) {
        let tag = ctx.next_tag();
        let data_length = data.len() as u32;

        // Build CBW (direction = false for OUT)
        let cbw = Cbw::new(tag, data_length, false, 0, cmd);

        // Send CBW
        if !self.send_cbw(ctx, &cbw) {
            println!("    CBW send failed");
            return (TransferResult::Error(0), None);
        }

        // Data OUT phase (if any)
        if data_length > 0 {
            // Copy data to buffer at DATA_OFFSET
            unsafe {
                let data_ptr = ctx.data_buf.add(DATA_OFFSET);
                core::ptr::copy_nonoverlapping(data.as_ptr(), data_ptr, data.len());
                ctx.flush_buffer(data_ptr as u64, data.len());
            }

            // Create TRB for data OUT with proper cycle bit
            let (idx, cycle) = ctx.advance_out();
            unsafe {
                let trb = &mut *ctx.bulk_out_ring.add(idx);
                trb.param = ctx.data_phys + DATA_OFFSET as u64;
                trb.status = data_length;
                trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;  // IOC, cycle
                ctx.flush_buffer(trb as *const _ as u64, 16);
            }

            // Ring doorbell and wait for completion
            self.ring_doorbell(ctx.slot_id, ctx.bulk_out_dci);

            // Get ERDP register address for direct writes (avoids borrow conflict)
            let erdp_addr = self.xhci.mmio_base() + self.xhci.rt_offset() as u64
                + (xhci_rt::IR0 + xhci_ir::ERDP) as u64;

            // Poll for completion
            if let Some(ref mut event_ring) = self.event_ring {
                let mut completed = false;
                for _ in 0..100 {
                    if let Some(evt) = event_ring.dequeue() {
                        let evt_type = evt.get_type();
                        let cc = (evt.status >> 24) & 0xFF;
                        // Update ERDP for all events
                        let erdp = event_ring.erdp() | (1 << 3);
                        unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }
                        if evt_type == trb_type::TRANSFER_EVENT {
                            if cc != trb_cc::SUCCESS && cc != trb_cc::SHORT_PACKET {
                                println!("[DATA] error cc={} at TRB {:x}", cc, evt.param);
                                return (TransferResult::Error(cc), None);
                            }
                            completed = true;
                            break;
                        }
                    }
                    syscall::yield_now();
                }
                if !completed {
                    println!("[DATA] timeout at idx={}", idx);
                    return (TransferResult::Error(0), None);
                }
            }
        }

        // CSW phase
        self.receive_csw_irq(ctx)
    }

    /// SCSI WRITE(10) command - write sectors to device
    fn scsi_write_10(&mut self, ctx: &mut BulkContext, lba: u32, data: &[u8]) -> bool {
        let count = (data.len() / 512) as u16;
        let (cmd, _data_length) = scsi::build_write_10(lba, count, 512);

        let (_result, csw) = self.scsi_command_out(ctx, &cmd, data);

        if let Some(csw) = csw {
            let status = csw.status;
            if csw.signature == msc::CSW_SIGNATURE && status == msc::CSW_STATUS_PASSED {
                return true;
            }
        }
        false
    }

    /// SCSI READ(10) command - read sectors from device
    /// lba: starting logical block address
    /// count: number of blocks to read (each block is typically 512 bytes)
    /// Returns (success, data slice)
    fn scsi_read_10<'a>(&mut self, ctx: &'a mut BulkContext, lba: u32, count: u16) -> Option<&'a [u8]> {
        // Use library CDB builder (assumes 512 bytes per block)
        let (cmd, data_length) = scsi::build_read_10(lba, count, 512);

        let (result, csw) = self.scsi_command_in(ctx, &cmd, data_length);

        if let Some(csw) = csw {
            let sig = csw.signature;
            let residue = csw.data_residue;
            let status = csw.status;

            if sig == msc::CSW_SIGNATURE && status == msc::CSW_STATUS_PASSED &&
               (result == TransferResult::Success || result == TransferResult::ShortPacket) {
                let actual_bytes = (data_length - residue) as usize;
                unsafe {
                    let data_ptr = ctx.data_buf.add(DATA_OFFSET);
                    ctx.invalidate_buffer(data_ptr as u64, actual_bytes);
                    return Some(core::slice::from_raw_parts(data_ptr, actual_bytes));
                }
            }
        }

        None
    }

    /// SCSI READ_CAPACITY(10) - get device capacity
    /// Returns (last_lba, block_size) or None on error
    fn scsi_read_capacity_10(&mut self, ctx: &mut BulkContext) -> Option<(u32, u32)> {
        // Use library CDB builder
        let (cmd, data_length) = scsi::build_read_capacity_10();

        println!("  SCSI READ_CAPACITY(10)...");

        let (result, csw) = self.scsi_command_in(ctx, &cmd, data_length);

        if let Some(csw) = csw {
            if csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED {
                // Use library parser
                unsafe {
                    let resp_ptr = ctx.data_buf.add(DATA_OFFSET);
                    ctx.invalidate_buffer(resp_ptr as u64, 8);

                    let data = core::slice::from_raw_parts(resp_ptr, 8);
                    if let Some((last_lba, block_size)) = scsi::parse_read_capacity_10(data) {
                        println!("    Last LBA: {}", last_lba);
                        println!("    Block size: {} bytes", block_size);
                        println!("    Capacity: {} MB", ((last_lba as u64 + 1) * block_size as u64) / (1024 * 1024));
                        return Some((last_lba, block_size));
                    }
                }
            }
        }

        println!("    READ_CAPACITY failed: {:?}", result);
        None
    }

    /// SCSI TEST_UNIT_READY - check if device is ready
    /// Returns true if device is ready, false otherwise
    fn scsi_test_unit_ready_ctx(&mut self, ctx: &mut BulkContext) -> bool {
        // Use library CDB builder
        let (cmd, data_length) = scsi::build_test_unit_ready();

        println!("  SCSI TEST_UNIT_READY...");

        let (result, csw) = self.scsi_command_in(ctx, &cmd, data_length);

        if let Some(csw) = csw {
            if csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED {
                println!("    Device is ready");
                return true;
            } else {
                println!("    Device not ready (status={})", csw.status);
            }
        } else {
            println!("    TEST_UNIT_READY failed: {:?}", result);
        }
        false
    }

    /// Query VPD page 0xB0 (Block Limits) to get device transfer limits
    /// Returns BlockLimits struct if supported, None otherwise
    fn scsi_inquiry_vpd_block_limits(&mut self, ctx: &mut BulkContext) -> Option<usb::BlockLimits> {
        // First, query supported VPD pages (page 0x00)
        let (cmd, data_length) = scsi::build_inquiry_vpd(scsi::VPD_SUPPORTED_PAGES, 64);
        let (_result, csw) = self.scsi_command_in(ctx, &cmd, data_length);

        let supports_b0 = if let Some(csw) = csw {
            if csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED {
                unsafe {
                    let resp_ptr = ctx.data_buf.add(DATA_OFFSET);
                    ctx.invalidate_buffer(resp_ptr as u64, 64);
                    let data = core::slice::from_raw_parts(resp_ptr, 64);
                    if let Some(pages) = scsi::parse_vpd_supported_pages(data) {
                        pages.contains(&scsi::VPD_BLOCK_LIMITS)
                    } else {
                        false
                    }
                }
            } else {
                false
            }
        } else {
            false
        };

        if !supports_b0 {
            return None;
        }

        // Query Block Limits VPD page (0xB0)
        let (cmd, data_length) = scsi::build_inquiry_vpd(scsi::VPD_BLOCK_LIMITS, 64);
        let (_result, csw) = self.scsi_command_in(ctx, &cmd, data_length);

        if let Some(csw) = csw {
            if csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED {
                unsafe {
                    let resp_ptr = ctx.data_buf.add(DATA_OFFSET);
                    ctx.invalidate_buffer(resp_ptr as u64, 64);
                    let data = core::slice::from_raw_parts(resp_ptr, 64);
                    return scsi::parse_vpd_block_limits(data);
                }
            }
        }

        None
    }

    /// Run SCSI tests - only prints capacity result
    /// Also stores device info for block access via ring buffer
    fn run_scsi_tests(&mut self, ctx: &mut BulkContext) {
        // Wait for device to be ready
        let mut ready = false;
        for _ in 0..10 {
            if self.scsi_test_unit_ready_ctx(ctx) {
                ready = true;
                break;
            }
            delay_ms(500);
        }

        if !ready { return; }

        // Query device transfer limits via VPD page 0xB0
        println!("  SCSI VPD Block Limits...");
        if let Some(limits) = self.scsi_inquiry_vpd_block_limits(ctx) {
            println!("    Device reports transfer limits:");
            if limits.max_transfer_length > 0 {
                println!("      Max transfer: {} blocks ({} KB)",
                    limits.max_transfer_length,
                    (limits.max_transfer_length as u64 * 512) / 1024);
            } else {
                println!("      Max transfer: not reported (unlimited)");
            }
            if limits.optimal_transfer_length > 0 {
                println!("      Optimal transfer: {} blocks ({} KB)",
                    limits.optimal_transfer_length,
                    (limits.optimal_transfer_length as u64 * 512) / 1024);
            }
            if limits.optimal_transfer_granularity > 0 {
                println!("      Granularity: {} blocks", limits.optimal_transfer_granularity);
            }
        } else {
            println!("    VPD Block Limits not supported (common for USB drives)");
        }

        // Read capacity and report
        if let Some((last_lba, block_size)) = self.scsi_read_capacity_10(ctx) {
            // Calculate and print size
            let block_count = last_lba as u64 + 1;
            let size_bytes = block_count * block_size as u64;
            let size_mb = size_bytes / (1024 * 1024);
            let size_gb = size_mb / 1024;
            if size_gb > 0 {
                println!("    Capacity: {} GB", size_gb);
            } else {
                println!("    Capacity: {} MB", size_mb);
            }

            // Verify we can read
            if self.scsi_read_10(ctx, 0, 1).is_some() {
                println!("    Read test: OK");

                // Store device info for ring buffer block access
                // Copy all BulkContext state so we can perform reads later
                self.msc_device = Some(MscDeviceInfo {
                    slot_id: ctx.slot_id,
                    bulk_in_dci: ctx.bulk_in_dci,
                    bulk_out_dci: ctx.bulk_out_dci,
                    block_size,
                    block_count,
                    bulk_in_ring: ctx.bulk_in_ring,
                    bulk_in_ring_phys: ctx.bulk_in_ring_phys,
                    bulk_out_ring: ctx.bulk_out_ring,
                    bulk_out_ring_phys: ctx._bulk_out_ring_phys,
                    data_buf: ctx.data_buf,
                    data_phys: ctx.data_phys,
                    device_ctx: ctx.device_ctx,
                    out_enqueue: ctx.out_enqueue,
                    in_enqueue: ctx.in_enqueue,
                    out_cycle: ctx.out_cycle,
                    in_cycle: ctx.in_cycle,
                    tag: ctx.tag,
                    bulk_in_addr: ctx.bulk_in_addr,
                    bulk_out_addr: ctx.bulk_out_addr,
                    interface_num: ctx.interface_num,
                    ep0_ring: ctx.ep0_ring,
                    ep0_ring_phys: ctx.ep0_ring_phys,
                    ep0_enqueue: ctx.ep0_enqueue,
                });
                println!("    Block device ready: {} blocks x {} bytes", block_count, block_size);

                // Performance test disabled - uncomment to enable
                // WARNING: Make sure test_lba is safe (currently LBA 1000)
                // self.run_performance_test(ctx, block_count);
                // if let Some(ref mut msc) = self.msc_device {
                //     msc.out_enqueue = ctx.out_enqueue;
                //     msc.in_enqueue = ctx.in_enqueue;
                //     msc.out_cycle = ctx.out_cycle;
                //     msc.in_cycle = ctx.in_cycle;
                //     msc.tag = ctx.tag;
                //     msc.ep0_enqueue = ctx.ep0_enqueue;
                // }
            }
        }
    }

    /// Run direct USB performance test (uses internal buffer)
    fn run_performance_test(&mut self, ctx: &mut BulkContext, block_count: u64) {
        println!("\n=== USB Performance Test ===");
        println!("  Ring state: OUT idx={} cycle={}, IN idx={} cycle={}",
            ctx.out_enqueue, ctx.out_cycle as u8, ctx.in_enqueue, ctx.in_cycle as u8);

        // Allocate a larger DMA buffer for performance testing (64KB)
        let mut perf_buf_phys: u64 = 0;
        let perf_buf_virt = syscall::mmap_dma(65536, &mut perf_buf_phys);
        if perf_buf_virt < 0 {
            println!("  Failed to allocate performance buffer");
            return;
        }

        // Test parameters - use LBA 1000 (safe zone, avoids sector 0 and partition tables)
        // NOTE: Don't use "near end of device" - many USB drives report fake capacity!
        // A "60GB" drive might only be 32GB, and writing beyond real capacity fails.
        let test_lba = 1000u32;

        // Get timer frequency
        let freq: u64;
        unsafe { core::arch::asm!("mrs {}, cntfrq_el0", out(reg) freq); }

        // Test different transfer sizes (small to large)
        // This way we get good data before potentially breaking the device
        let transfer_sizes = [1u16, 4, 8, 16, 32, 64]; // Ascending order
        let mut max_reliable_size = 1u16;

        for &sectors_per_transfer in &transfer_sizes {
            let bytes_per_transfer = sectors_per_transfer as usize * 512;
            if bytes_per_transfer > 65536 {
                continue;
            }

            // Fill buffer with test pattern
            unsafe {
                let buf = perf_buf_virt as *mut u8;
                for i in 0..bytes_per_transfer {
                    *buf.add(i) = (i & 0xFF) as u8;
                }
                // Flush buffer to memory for DMA
                flush_buffer(perf_buf_virt as u64, bytes_per_transfer);
            }

            // Write test: 100 transfers
            let write_count = 100u32;

            let start: u64;
            unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) start); }

            let mut success = 0u32;
            for i in 0..write_count {
                let lba = test_lba + i * sectors_per_transfer as u32;
                if self.perf_write(ctx, lba, perf_buf_phys, bytes_per_transfer) {
                    success += 1;
                } else {
                    if success == 0 {
                        println!("  First write failed at OUT idx={}", ctx.out_enqueue);
                    }
                    // Recovery: reset endpoints and clear device error
                    self.recover_endpoints(ctx);
                    break;
                }
            }

            let end: u64;
            unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) end); }

            let elapsed_us = ((end - start) * 1_000_000) / freq;
            let bytes_written = success as u64 * bytes_per_transfer as u64;
            let throughput_kbs = if elapsed_us > 0 { (bytes_written * 1_000_000) / (elapsed_us * 1024) } else { 0 };

            println!("  Write {}x{} sectors: {}/{} OK, {} KB/s",
                sectors_per_transfer, write_count, success, write_count, throughput_kbs);

            // Track max reliable size (update as we find larger working sizes)
            if success == write_count {
                max_reliable_size = sectors_per_transfer;
            }

            // Read test: 100 transfers
            let start: u64;
            unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) start); }

            let mut success = 0u32;
            for i in 0..write_count {
                let lba = test_lba + i * sectors_per_transfer as u32;
                if self.perf_read(ctx, lba, sectors_per_transfer, perf_buf_phys, bytes_per_transfer) {
                    success += 1;
                } else {
                    // Recovery: reset endpoints and clear device error
                    self.recover_endpoints(ctx);
                    break;
                }
            }

            let end: u64;
            unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) end); }

            let elapsed_us = ((end - start) * 1_000_000) / freq;
            let bytes_read = success as u64 * bytes_per_transfer as u64;
            let throughput_kbs = if elapsed_us > 0 { (bytes_read * 1_000_000) / (elapsed_us * 1024) } else { 0 };

            println!("  Read  {}x{} sectors: {}/{} OK, {} KB/s",
                sectors_per_transfer, write_count, success, write_count, throughput_kbs);
        }

        // Recovery verification: try 1 sector write/read to confirm device works
        println!("  --- Recovery Verification ---");
        let bytes_per_transfer = 512;
        unsafe {
            let buf = perf_buf_virt as *mut u8;
            for i in 0..512 {
                *buf.add(i) = 0xAA;  // Different pattern
            }
            // Flush buffer to memory for DMA
            flush_buffer(perf_buf_virt as u64, 512);
        }

        let write_ok = self.perf_write(ctx, test_lba, perf_buf_phys, bytes_per_transfer);
        let read_ok = if write_ok {
            self.perf_read(ctx, test_lba, 1, perf_buf_phys, bytes_per_transfer)
        } else {
            false
        };

        if write_ok && read_ok {
            println!("  Post-test 1-sector: PASS (recovery works!)");
        } else {
            println!("  Post-test 1-sector: FAIL (write={}, read={})", write_ok, read_ok);
            println!("  Recovery did NOT restore device to working state");
        }

        println!("=== Performance Test Complete ===");
        println!("  Max reliable transfer: {} sectors ({} KB)",
            max_reliable_size, max_reliable_size as u32 * 512 / 1024);
        println!();
    }

    /// Performance write helper - direct SCSI write with command queuing
    /// Queues CBW + Data TRBs, rings doorbell once, then waits
    fn perf_write(&mut self, ctx: &mut BulkContext, lba: u32, data_phys: u64, len: usize) -> bool {
        let count = (len / 512) as u16;
        let (cmd, data_length) = scsi::build_write_10(lba, count, 512);

        // Build CBW
        let tag = ctx.next_tag();
        let cbw = Cbw::new(tag, data_length, false, 0, &cmd);

        // Write CBW to ctx data buffer
        unsafe {
            let cbw_ptr = ctx.data_buf.add(CBW_OFFSET) as *mut Cbw;
            core::ptr::write_volatile(cbw_ptr, cbw);
            ctx.flush_buffer(cbw_ptr as u64, 31);
        }

        // Queue CBW TRB (no IOC - we'll wait on the data TRB)
        let (idx, cycle) = ctx.advance_out();
        unsafe {
            let trb = &mut *ctx.bulk_out_ring.add(idx);
            trb.param = ctx.data_phys + CBW_OFFSET as u64;
            trb.status = 31;
            trb.control = (trb_type::NORMAL << 10) | cycle;  // No IOC
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }

        // Queue Data OUT TRB (with IOC)
        let (idx, cycle) = ctx.advance_out();
        unsafe {
            let trb = &mut *ctx.bulk_out_ring.add(idx);
            trb.param = data_phys;
            trb.status = len as u32;
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;  // IOC
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }

        // Ring doorbell once for both TRBs
        self.ring_doorbell(ctx.slot_id, ctx.bulk_out_dci);

        // Wait for data completion (single wait for both TRBs)
        let timeout = 100 + (len / 512) as u32 * 10;
        if !self.wait_transfer_complete(timeout) { return false; }

        // Queue CSW TRB
        let (idx, cycle) = ctx.advance_in();
        unsafe {
            let trb = &mut *ctx.bulk_in_ring.add(idx);
            trb.param = ctx.data_phys + CSW_OFFSET as u64;
            trb.status = 13 | (1 << 17);
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }
        self.ring_doorbell(ctx.slot_id, ctx.bulk_in_dci);

        // Wait for CSW
        if !self.wait_transfer_complete(100) { return false; }

        // Check CSW
        unsafe {
            let csw_ptr = ctx.data_buf.add(CSW_OFFSET);
            ctx.invalidate_buffer(csw_ptr as u64, 13);
            let csw = core::ptr::read_volatile(csw_ptr as *const Csw);
            csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED
        }
    }

    /// Performance read helper - with command queuing
    /// Queues Data IN + CSW TRBs, rings doorbell once
    fn perf_read(&mut self, ctx: &mut BulkContext, lba: u32, count: u16, data_phys: u64, len: usize) -> bool {
        let (cmd, data_length) = scsi::build_read_10(lba, count, 512);

        // Build CBW
        let tag = ctx.next_tag();
        let cbw = Cbw::new(tag, data_length, true, 0, &cmd);

        // Write CBW
        unsafe {
            let cbw_ptr = ctx.data_buf.add(CBW_OFFSET) as *mut Cbw;
            core::ptr::write_volatile(cbw_ptr, cbw);
            ctx.flush_buffer(cbw_ptr as u64, 31);
        }

        // Send CBW (separate ring since it's OUT, data is IN)
        let (idx, cycle) = ctx.advance_out();
        unsafe {
            let trb = &mut *ctx.bulk_out_ring.add(idx);
            trb.param = ctx.data_phys + CBW_OFFSET as u64;
            trb.status = 31;
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }
        self.ring_doorbell(ctx.slot_id, ctx.bulk_out_dci);
        if !self.wait_transfer_complete(100) { return false; }

        // Queue Data IN TRB (no IOC)
        let (idx, cycle) = ctx.advance_in();
        unsafe {
            let trb = &mut *ctx.bulk_in_ring.add(idx);
            trb.param = data_phys;
            trb.status = (len as u32) | (1 << 17);
            trb.control = (trb_type::NORMAL << 10) | cycle;  // No IOC
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }

        // Queue CSW TRB (with IOC)
        let (idx, cycle) = ctx.advance_in();
        unsafe {
            let trb = &mut *ctx.bulk_in_ring.add(idx);
            trb.param = ctx.data_phys + CSW_OFFSET as u64;
            trb.status = 13 | (1 << 17);
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;  // IOC
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }

        // Ring doorbell once for both IN TRBs
        self.ring_doorbell(ctx.slot_id, ctx.bulk_in_dci);

        // Wait for CSW completion (covers both data and CSW)
        // Use high spin count for reads since device needs to fetch from flash
        let timeout = 100 + (len / 512) as u32 * 10;
        if !self.wait_transfer_complete_read(timeout) { return false; }

        // Check CSW
        unsafe {
            let csw_ptr = ctx.data_buf.add(CSW_OFFSET);
            ctx.invalidate_buffer(csw_ptr as u64, 13);
            let csw = core::ptr::read_volatile(csw_ptr as *const Csw);
            csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED
        }
    }

    /// Recover endpoints after a failed transfer
    /// Performs full USB BOT recovery sequence:
    /// 1. BOT Reset (class-specific reset)
    /// 2. CLEAR_FEATURE(ENDPOINT_HALT) for both bulk endpoints
    /// 3. xHCI Reset Endpoint commands
    /// 4. Set TR Dequeue Pointer commands
    fn recover_endpoints(&mut self, ctx: &mut BulkContext) {
        println!("  [Recovery] Starting USB BOT recovery...");

        // Drain any pending events first
        let erdp_addr = self.xhci.mmio_base() + self.xhci.rt_offset() as u64
            + (xhci_rt::IR0 + xhci_ir::ERDP) as u64;
        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..50 {
                if let Some(_evt) = event_ring.dequeue() {
                    let erdp = event_ring.erdp() | (1 << 3);
                    unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }
                } else {
                    break;
                }
            }
        }

        // Step 1: USB BOT Reset (class-specific reset)
        // bmRequestType=0x21 (class, interface), bRequest=0xFF, wIndex=interface
        println!("  [Recovery] Sending BOT Reset to interface {}...", ctx.interface_num);
        let result = self.control_transfer(
            ctx.slot_id, ctx.ep0_ring, ctx.ep0_ring_phys, &mut ctx.ep0_enqueue,
            0x21, 0xFF, 0, ctx.interface_num as u16, 0, 0
        );
        match result {
            Some((cc, _)) if cc == trb_cc::SUCCESS => println!("  [Recovery] BOT Reset OK"),
            _ => println!("  [Recovery] BOT Reset failed (continuing anyway)"),
        }
        delay_ms(10);

        // Step 2: CLEAR_FEATURE(ENDPOINT_HALT) for bulk OUT
        // bmRequestType=0x02 (endpoint), bRequest=1 (CLEAR_FEATURE), wValue=0 (HALT), wIndex=endpoint
        println!("  [Recovery] Clearing HALT on bulk OUT endpoint 0x{:02x}...", ctx.bulk_out_addr);
        let result = self.control_transfer(
            ctx.slot_id, ctx.ep0_ring, ctx.ep0_ring_phys, &mut ctx.ep0_enqueue,
            0x02, 1, 0, ctx.bulk_out_addr as u16, 0, 0
        );
        match result {
            Some((cc, _)) if cc == trb_cc::SUCCESS => println!("  [Recovery] Clear HALT OUT OK"),
            _ => println!("  [Recovery] Clear HALT OUT failed"),
        }

        // Step 3: CLEAR_FEATURE(ENDPOINT_HALT) for bulk IN
        println!("  [Recovery] Clearing HALT on bulk IN endpoint 0x{:02x}...", ctx.bulk_in_addr);
        let result = self.control_transfer(
            ctx.slot_id, ctx.ep0_ring, ctx.ep0_ring_phys, &mut ctx.ep0_enqueue,
            0x02, 1, 0, ctx.bulk_in_addr as u16, 0, 0
        );
        match result {
            Some((cc, _)) if cc == trb_cc::SUCCESS => println!("  [Recovery] Clear HALT IN OK"),
            _ => println!("  [Recovery] Clear HALT IN failed"),
        }

        // Step 4: xHCI Reset bulk OUT endpoint
        let reset_out = usb::enumeration::build_reset_endpoint_trb(ctx.slot_id, ctx.bulk_out_dci, false);
        if let Some(ref mut cmd_ring) = self.cmd_ring {
            cmd_ring.enqueue(&reset_out);
        }
        self.ring_doorbell(0, 0);
        self.wait_command_complete(100);

        // Step 5: xHCI Reset bulk IN endpoint
        let reset_in = usb::enumeration::build_reset_endpoint_trb(ctx.slot_id, ctx.bulk_in_dci, false);
        if let Some(ref mut cmd_ring) = self.cmd_ring {
            cmd_ring.enqueue(&reset_in);
        }
        self.ring_doorbell(0, 0);
        self.wait_command_complete(100);

        // Step 6: Reinitialize the transfer rings (clear old TRBs, reset Link TRB)
        println!("  [Recovery] Reinitializing transfer rings...");
        unsafe {
            use usb::transfer::{flush_cache_line, dsb};
            use usb::trb::trb_type;

            // Reinitialize bulk OUT ring
            let out_ring = ctx.bulk_out_ring;
            for i in 0..256 {
                let trb = &mut *out_ring.add(i);
                trb.param = 0;
                trb.status = 0;
                trb.control = 0;
            }
            // Set up Link TRB at index 255
            let out_link = &mut *out_ring.add(255);
            out_link.param = ctx._bulk_out_ring_phys;
            out_link.set_type(trb_type::LINK);
            out_link.control |= 1 << 1;  // Toggle Cycle bit (TC)
            // Link TRB cycle = 0 (opposite of producer cycle=1)
            for i in 0..256 {
                flush_cache_line(out_ring.add(i) as u64);
            }
            dsb();

            // Reinitialize bulk IN ring
            let in_ring = ctx.bulk_in_ring;
            for i in 0..256 {
                let trb = &mut *in_ring.add(i);
                trb.param = 0;
                trb.status = 0;
                trb.control = 0;
            }
            // Set up Link TRB at index 255
            let in_link = &mut *in_ring.add(255);
            in_link.param = ctx.bulk_in_ring_phys;
            in_link.set_type(trb_type::LINK);
            in_link.control |= 1 << 1;  // Toggle Cycle bit (TC)
            // Link TRB cycle = 0 (opposite of producer cycle=1)
            for i in 0..256 {
                flush_cache_line(in_ring.add(i) as u64);
            }
            dsb();
        }

        // Reset ring state - start fresh
        ctx.out_enqueue = 0;
        ctx.in_enqueue = 0;
        ctx.out_cycle = true;
        ctx.in_cycle = true;

        // Step 7: Set TR Dequeue Pointer for OUT endpoint (DCS=1 to match our cycle)
        let set_deq_out = usb::enumeration::build_set_tr_dequeue_trb(
            ctx.slot_id, ctx.bulk_out_dci, ctx._bulk_out_ring_phys, true
        );
        if let Some(ref mut cmd_ring) = self.cmd_ring {
            cmd_ring.enqueue(&set_deq_out);
        }
        self.ring_doorbell(0, 0);
        self.wait_command_complete(100);

        // Step 8: Set TR Dequeue Pointer for IN endpoint (DCS=1 to match our cycle)
        let set_deq_in = usb::enumeration::build_set_tr_dequeue_trb(
            ctx.slot_id, ctx.bulk_in_dci, ctx.bulk_in_ring_phys, true
        );
        if let Some(ref mut cmd_ring) = self.cmd_ring {
            cmd_ring.enqueue(&set_deq_in);
        }
        self.ring_doorbell(0, 0);
        self.wait_command_complete(100);

        // Step 9: Wait for device to be ready (TEST_UNIT_READY polling)
        println!("  [Recovery] Waiting for device ready...");
        delay_ms(100);
        let mut ready = false;
        for attempt in 0..10 {
            if self.test_unit_ready_quick(ctx) {
                println!("  [Recovery] Device ready after {} attempts", attempt + 1);
                ready = true;
                break;
            }
            delay_ms(100);
        }
        if !ready {
            println!("  [Recovery] Device NOT ready after 10 attempts!");
        }

        println!("  [Recovery] Complete");
    }

    /// Quick TEST_UNIT_READY for recovery (no debug output)
    fn test_unit_ready_quick(&mut self, ctx: &mut BulkContext) -> bool {
        let (cmd, data_length) = usb::msc::scsi::build_test_unit_ready();
        let tag = ctx.next_tag();
        let cbw = Cbw::new(tag, data_length, true, 0, &cmd);

        // Write CBW
        unsafe {
            let cbw_ptr = ctx.data_buf.add(CBW_OFFSET) as *mut Cbw;
            core::ptr::write_volatile(cbw_ptr, cbw);
            ctx.flush_buffer(cbw_ptr as u64, 31);
        }

        // Send CBW
        let (idx, cycle) = ctx.advance_out();
        unsafe {
            let trb = &mut *ctx.bulk_out_ring.add(idx);
            trb.param = ctx.data_phys + CBW_OFFSET as u64;
            trb.status = 31;
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }
        self.ring_doorbell(ctx.slot_id, ctx.bulk_out_dci);
        if !self.wait_transfer_complete(50) { return false; }

        // Receive CSW
        let (idx, cycle) = ctx.advance_in();
        unsafe {
            let trb = &mut *ctx.bulk_in_ring.add(idx);
            trb.param = ctx.data_phys + CSW_OFFSET as u64;
            trb.status = 13 | (1 << 17);
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }
        self.ring_doorbell(ctx.slot_id, ctx.bulk_in_dci);
        if !self.wait_transfer_complete(50) { return false; }

        // Check CSW
        unsafe {
            let csw_ptr = ctx.data_buf.add(CSW_OFFSET);
            ctx.invalidate_buffer(csw_ptr as u64, 13);
            let csw = core::ptr::read_volatile(csw_ptr as *const Csw);
            csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED
        }
    }

    /// Wait for command completion (for recovery)
    fn wait_command_complete(&mut self, max_polls: u32) -> bool {
        let erdp_addr = self.xhci.mmio_base() + self.xhci.rt_offset() as u64
            + (xhci_rt::IR0 + xhci_ir::ERDP) as u64;

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..max_polls {
                if let Some(evt) = event_ring.dequeue() {
                    let erdp = event_ring.erdp() | (1 << 3);
                    unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }
                    if evt.get_type() == trb_type::COMMAND_COMPLETION {
                        return true;
                    }
                }
                syscall::yield_now();
            }
        }
        false
    }

    /// Wait for transfer event completion (optimized polling)
    /// spin_count: number of spin iterations before falling back to yield
    /// max_polls: maximum yield iterations after spin
    fn wait_transfer_complete_ex(&mut self, spin_count: u32, max_polls: u32) -> bool {
        let erdp_addr = self.xhci.mmio_base() + self.xhci.rt_offset() as u64
            + (xhci_rt::IR0 + xhci_ir::ERDP) as u64;

        if let Some(ref mut event_ring) = self.event_ring {
            // First, spin locally without yielding (USB should be fast)
            for _ in 0..spin_count {
                if let Some(evt) = event_ring.dequeue() {
                    let erdp = event_ring.erdp() | (1 << 3);
                    unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }

                    if evt.get_type() == trb_type::TRANSFER_EVENT {
                        let cc = (evt.status >> 24) & 0xFF;
                        return cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET;
                    }
                }
                core::hint::spin_loop();
            }

            // If still waiting, use slower polling with yields
            for _ in 0..max_polls {
                if let Some(evt) = event_ring.dequeue() {
                    let erdp = event_ring.erdp() | (1 << 3);
                    unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }

                    if evt.get_type() == trb_type::TRANSFER_EVENT {
                        let cc = (evt.status >> 24) & 0xFF;
                        return cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET;
                    }
                }
                syscall::yield_now();
            }
        }
        false
    }

    /// Wait for transfer completion with default spin count (for writes/CBW)
    fn wait_transfer_complete(&mut self, max_polls: u32) -> bool {
        self.wait_transfer_complete_ex(1000, max_polls)
    }

    /// Wait for transfer completion with high spin count (for reads from flash)
    /// Reads need more spin time because device must fetch data from flash storage
    fn wait_transfer_complete_read(&mut self, max_polls: u32) -> bool {
        self.wait_transfer_complete_ex(10000, max_polls)
    }

    /// Read blocks from the MSC device (for ring buffer handler)
    /// Returns bytes read on success, writes data to internal buffer
    fn block_read(&mut self, lba: u64, count: u32) -> Option<usize> {
        // Check limits
        if count == 0 || count > 64 {
            return None; // Max 64 blocks (32KB at 512 bytes/block)
        }

        // Check if we have an MSC device - take ownership temporarily
        let mut msc = self.msc_device.take()?;

        // Reconstruct BulkContext from stored MscDeviceInfo
        let mut ctx = BulkContext {
            slot_id: msc.slot_id,
            bulk_in_dci: msc.bulk_in_dci,
            bulk_out_dci: msc.bulk_out_dci,
            bulk_in_ring: msc.bulk_in_ring,
            bulk_in_ring_phys: msc.bulk_in_ring_phys,
            bulk_out_ring: msc.bulk_out_ring,
            _bulk_out_ring_phys: msc.bulk_out_ring_phys,
            data_buf: msc.data_buf,
            data_phys: msc.data_phys,
            device_ctx: msc.device_ctx,
            out_enqueue: msc.out_enqueue,
            in_enqueue: msc.in_enqueue,
            out_cycle: msc.out_cycle,
            in_cycle: msc.in_cycle,
            tag: msc.tag,
            bulk_in_addr: msc.bulk_in_addr,
            bulk_out_addr: msc.bulk_out_addr,
            interface_num: msc.interface_num,
            ep0_ring: msc.ep0_ring,
            ep0_ring_phys: msc.ep0_ring_phys,
            ep0_enqueue: msc.ep0_enqueue,
        };

        // Perform the read and extract length immediately
        let bytes_read = self.scsi_read_10(&mut ctx, lba as u32, count as u16)
            .map(|data| data.len());

        // Update ring state in MscDeviceInfo (result no longer borrows ctx)
        msc.out_enqueue = ctx.out_enqueue;
        msc.in_enqueue = ctx.in_enqueue;
        msc.out_cycle = ctx.out_cycle;
        msc.in_cycle = ctx.in_cycle;
        msc.tag = ctx.tag;
        msc.ep0_enqueue = ctx.ep0_enqueue;

        // Restore the device info
        self.msc_device = Some(msc);

        // Return bytes read
        bytes_read
    }

    /// Read blocks directly to client's DMA buffer (zero-copy)
    /// Returns bytes read on success
    fn block_read_dma(&mut self, lba: u64, count: u32, target_phys: u64) -> Option<usize> {
        // Check limits
        if count == 0 || count > 64 {
            return None; // Max 64 blocks (32KB at 512 bytes/block)
        }

        // Check if we have an MSC device - take ownership temporarily
        let mut msc = self.msc_device.take()?;

        // Reconstruct BulkContext from stored MscDeviceInfo
        let mut ctx = BulkContext {
            slot_id: msc.slot_id,
            bulk_in_dci: msc.bulk_in_dci,
            bulk_out_dci: msc.bulk_out_dci,
            bulk_in_ring: msc.bulk_in_ring,
            bulk_in_ring_phys: msc.bulk_in_ring_phys,
            bulk_out_ring: msc.bulk_out_ring,
            _bulk_out_ring_phys: msc.bulk_out_ring_phys,
            data_buf: msc.data_buf,
            data_phys: msc.data_phys,
            device_ctx: msc.device_ctx,
            out_enqueue: msc.out_enqueue,
            in_enqueue: msc.in_enqueue,
            out_cycle: msc.out_cycle,
            in_cycle: msc.in_cycle,
            tag: msc.tag,
            bulk_in_addr: msc.bulk_in_addr,
            bulk_out_addr: msc.bulk_out_addr,
            interface_num: msc.interface_num,
            ep0_ring: msc.ep0_ring,
            ep0_ring_phys: msc.ep0_ring_phys,
            ep0_enqueue: msc.ep0_enqueue,
        };

        // Build READ(10) command
        let (cmd, data_length) = scsi::build_read_10(lba as u32, count as u16, 512);

        // Execute with DMA directly to client's buffer
        let (result, csw) = self.scsi_command_in_dma(&mut ctx, &cmd, target_phys, data_length);

        // Update ring state in MscDeviceInfo
        msc.out_enqueue = ctx.out_enqueue;
        msc.in_enqueue = ctx.in_enqueue;
        msc.out_cycle = ctx.out_cycle;
        msc.in_cycle = ctx.in_cycle;
        msc.tag = ctx.tag;
        msc.ep0_enqueue = ctx.ep0_enqueue;

        // Restore the device info
        self.msc_device = Some(msc);

        // Check result
        if let Some(csw) = csw {
            if csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED &&
               (result == TransferResult::Success || result == TransferResult::ShortPacket) {
                let actual_bytes = (data_length - csw.data_residue) as usize;
                return Some(actual_bytes);
            }
        }

        None
    }

    /// Write blocks directly from client's DMA buffer (zero-copy)
    /// Returns bytes written on success
    fn block_write_dma(&mut self, _lba: u64, _count: u32, _source_phys: u64) -> Option<usize> {
        // TODO: Implement SCSI WRITE(10) with DMA from source buffer
        // For now, return error - we'll implement this when needed
        None
    }

    /// Get block device info
    fn get_block_info(&self) -> Option<(u32, u64)> {
        self.msc_device.as_ref().map(|msc| (msc.block_size, msc.block_count))
    }

    /// Get pointer to data buffer (after block_read)
    fn get_data_buffer(&self) -> Option<*const u8> {
        self.msc_device.as_ref().map(|msc| {
            unsafe { msc.data_buf.add(DATA_OFFSET) as *const u8 }
        })
    }

    /// Get mutable pointer to data buffer (for block_write)
    fn get_data_buffer_mut(&mut self) -> Option<*mut u8> {
        self.msc_device.as_ref().map(|msc| {
            unsafe { msc.data_buf.add(DATA_OFFSET) }
        })
    }

    /// Write blocks to the MSC device (for ring buffer handler)
    /// Returns bytes written on success
    fn block_write(&mut self, lba: u64, data: &[u8]) -> Option<usize> {
        // Check limits (max 64 blocks = 32KB at 512 bytes/block)
        if data.is_empty() || data.len() > 32768 {
            return None;
        }

        // Check if we have an MSC device - take ownership temporarily
        let mut msc = self.msc_device.take()?;

        // Reconstruct BulkContext from stored MscDeviceInfo
        let mut ctx = BulkContext {
            slot_id: msc.slot_id,
            bulk_in_dci: msc.bulk_in_dci,
            bulk_out_dci: msc.bulk_out_dci,
            bulk_in_ring: msc.bulk_in_ring,
            bulk_in_ring_phys: msc.bulk_in_ring_phys,
            bulk_out_ring: msc.bulk_out_ring,
            _bulk_out_ring_phys: msc.bulk_out_ring_phys,
            data_buf: msc.data_buf,
            data_phys: msc.data_phys,
            device_ctx: msc.device_ctx,
            out_enqueue: msc.out_enqueue,
            in_enqueue: msc.in_enqueue,
            out_cycle: msc.out_cycle,
            in_cycle: msc.in_cycle,
            tag: msc.tag,
            bulk_in_addr: msc.bulk_in_addr,
            bulk_out_addr: msc.bulk_out_addr,
            interface_num: msc.interface_num,
            ep0_ring: msc.ep0_ring,
            ep0_ring_phys: msc.ep0_ring_phys,
            ep0_enqueue: msc.ep0_enqueue,
        };

        // Copy data to DMA buffer
        unsafe {
            let data_ptr = ctx.data_buf.add(DATA_OFFSET);
            core::ptr::copy_nonoverlapping(data.as_ptr(), data_ptr, data.len());
            // Flush data to memory for DMA
            flush_buffer(data_ptr as u64, data.len());
        }

        // Perform the write
        let success = self.scsi_write_10(&mut ctx, lba as u32, data);

        // Update ring state in MscDeviceInfo
        msc.out_enqueue = ctx.out_enqueue;
        msc.in_enqueue = ctx.in_enqueue;
        msc.out_cycle = ctx.out_cycle;
        msc.in_cycle = ctx.in_cycle;
        msc.tag = ctx.tag;
        msc.ep0_enqueue = ctx.ep0_enqueue;

        // Restore the device info
        self.msc_device = Some(msc);

        if success {
            Some(data.len())
        } else {
            None
        }
    }
}

// NOTE: Old scsi_inquiry and scsi_test_unit_ready functions (~1000 lines combined) were removed.
// Functionality consolidated into BulkContext-based scsi_test_unit_ready_ctx() and run_scsi_tests().

// =============================================================================
// Helper Functions
// =============================================================================

/// Format an IRQ scheme URL from IRQ number
/// Returns a static string for common IRQs
fn format_irq_url(irq: u32) -> &'static str {
    // MT7988A USB controller IRQs (GIC SPI + 32)
    // SSUSB0: SPI 173 + 32 = 205
    // SSUSB1: SPI 172 + 32 = 204
    match irq {
        204 => "irq:204",
        205 => "irq:205",
        _ => "irq:204",  // Default to SSUSB1
    }
}

// =============================================================================
// VBUS Power Control via GPIO Driver
// =============================================================================

/// Request USB VBUS enable from the GPIO expander driver (silent)
fn request_vbus_enable() -> bool {
    let channel = syscall::port_connect(b"gpio");
    if channel < 0 { return false; }
    let ch = channel as u32;

    let cmd = [GPIO_CMD_USB_VBUS, 1];
    if syscall::send(ch, &cmd) < 0 {
        syscall::channel_close(ch);
        return false;
    }

    let mut response = [0u8; 1];
    let recv_result = syscall::receive(ch, &mut response);
    syscall::channel_close(ch);

    recv_result > 0 && response[0] as i8 == 0
}

// =============================================================================
// Block Device Server (Ring Buffer)
// =============================================================================

/// Block device server - handles requests from BlockClient via shared ring buffer
///
/// Uses zero-copy DMA: client requests specify offset in shared data buffer,
/// USB DMA writes directly to that buffer, client reads without copy.
struct BlockServer {
    /// Channel used only for initial handshake (PID + shmem_id exchange)
    handshake_channel: u32,
    /// Shared ring buffer for all block operations after handshake
    ring: Option<BlockRing>,
    /// Client process ID (for shmem_allow)
    client_pid: u32,
    /// Temporary buffer for handshake messages only
    handshake_buf: [u8; 64],
}

impl BlockServer {
    /// Create a new block server from an accepted connection channel
    fn new(channel: u32) -> Self {
        Self {
            handshake_channel: channel,
            ring: None,
            client_pid: 0,
            handshake_buf: [0u8; 64],
        }
    }

    /// Complete handshake with client - exchange PID and shmem_id
    ///
    /// After handshake, all communication goes through the ring buffer.
    /// The handshake channel is no longer used.
    fn try_handshake(&mut self) -> bool {
        // Receive client's PID
        let recv_len = syscall::receive(self.handshake_channel, &mut self.handshake_buf);
        if recv_len < 4 {
            return false;
        }

        self.client_pid = u32::from_le_bytes([
            self.handshake_buf[0], self.handshake_buf[1],
            self.handshake_buf[2], self.handshake_buf[3]
        ]);

        // Create shared ring buffer (64 entries, 1MB DMA data buffer)
        let ring = match BlockRing::create(64, 1024 * 1024) {
            Some(r) => r,
            None => {
                println!("[usbd] Failed to create ring buffer");
                return false;
            }
        };

        // Grant client permission to map the shared memory
        if !ring.allow(self.client_pid) {
            println!("[usbd] Failed to allow client access to ring");
            return false;
        }

        // Send shmem_id to client (final handshake message)
        let shmem_id_bytes = ring.shmem_id().to_le_bytes();
        self.handshake_buf[..4].copy_from_slice(&shmem_id_bytes);
        syscall::send(self.handshake_channel, &self.handshake_buf[..4]);

        // Handshake complete - all further communication via ring
        self.ring = Some(ring);
        true
    }

    /// Check if ring is ready
    fn is_ready(&self) -> bool {
        self.ring.is_some()
    }

    /// Process pending ring requests - returns number of requests processed
    fn process_requests(&self, driver: &mut UsbDriver) -> u32 {
        let ring = match &self.ring {
            Some(r) => r,
            None => return 0,
        };

        let mut processed = 0u32;

        // Process all pending requests
        while let Some(req) = ring.next_request() {
            let resp = self.handle_request(driver, ring, &req);
            ring.complete(&resp);
            processed += 1;
        }

        // Notify client if we processed any requests
        if processed > 0 {
            ring.notify();
        }

        processed
    }

    /// Handle a single block request
    fn handle_request(&self, driver: &mut UsbDriver, ring: &BlockRing, req: &BlockRequest) -> BlockResponse {
        match req.cmd {
            BlockRequest::CMD_INFO => {
                // Return device info from MSC device
                if let Some(ref msc) = driver.msc_device {
                    BlockResponse::info(req.tag, msc.block_size, msc.block_count)
                } else {
                    BlockResponse::error(req.tag, -19) // ENODEV
                }
            }
            BlockRequest::CMD_READ => {
                self.handle_read(driver, ring, req)
            }
            BlockRequest::CMD_WRITE => {
                self.handle_write(driver, ring, req)
            }
            _ => {
                BlockResponse::error(req.tag, -38) // ENOSYS
            }
        }
    }

    /// Handle a read request
    fn handle_read(&self, driver: &mut UsbDriver, ring: &BlockRing, req: &BlockRequest) -> BlockResponse {
        // Validate request
        if req.count == 0 {
            return BlockResponse::error(req.tag, -22); // EINVAL
        }

        let block_size = driver.msc_device.as_ref().map(|m| m.block_size).unwrap_or(512);
        let bytes = (req.count as u64) * (block_size as u64);
        let buf_offset = req.buf_offset as usize;

        // Check buffer bounds
        if buf_offset + bytes as usize > ring.data_size() {
            return BlockResponse::error(req.tag, -22);
        }

        // Get physical address of target buffer in ring
        let target_phys = ring.data_phys() + buf_offset as u64;

        // Perform DMA read directly to ring buffer
        match driver.block_read_dma(req.lba, req.count, target_phys) {
            Some(bytes_read) => BlockResponse::ok(req.tag, bytes_read as u32),
            None => BlockResponse::error(req.tag, -5), // EIO
        }
    }

    /// Handle a write request
    fn handle_write(&self, driver: &mut UsbDriver, ring: &BlockRing, req: &BlockRequest) -> BlockResponse {
        // Validate request
        if req.count == 0 {
            return BlockResponse::error(req.tag, -22); // EINVAL
        }

        let block_size = driver.msc_device.as_ref().map(|m| m.block_size).unwrap_or(512);
        let bytes = (req.count as u64) * (block_size as u64);
        let buf_offset = req.buf_offset as usize;

        // Check buffer bounds
        if buf_offset + bytes as usize > ring.data_size() {
            return BlockResponse::error(req.tag, -22);
        }

        // Get physical address of source buffer in ring
        let source_phys = ring.data_phys() + buf_offset as u64;

        // Perform DMA write from ring buffer
        match driver.block_write_dma(req.lba, req.count, source_phys) {
            Some(bytes_written) => BlockResponse::ok(req.tag, bytes_written as u32),
            None => BlockResponse::error(req.tag, -5), // EIO
        }
    }
}

// =============================================================================
// Main
// =============================================================================

#[unsafe(no_mangle)]
fn main() {
    println!("USB driver starting...");

    // Register the "usb" port to ensure only one instance runs
    let port_result = syscall::port_register(b"usb");
    if port_result < 0 {
        if port_result == -17 {
            println!("ERROR: USB daemon already running");
        } else {
            println!("ERROR: Failed to register USB port: {}", port_result);
        }
        syscall::exit(1);
    }
    let usb_port = port_result as u32;

    // === Board Initialization ===
    // The board abstraction handles all platform-specific setup:
    // - SoC clock/reset initialization
    // - VBUS power control
    // - Controller configuration
    let mut board = BpiR4::new();

    // Request VBUS enable and wait for power stabilization
    let _vbus_ok = request_vbus_enable();
    delay_ms(100);

    // Board pre-init handles SoC-level clocks and resets
    if board.pre_init().is_err() {
        println!("ERROR: Board pre-init failed");
        syscall::exit(1);
    }

    // === Controller Initialization ===
    // Try controllers in board-defined priority order (SSUSB1 first, then SSUSB0)
    let mut driver: Option<UsbDriver> = None;
    let mut active_config: Option<&UsbControllerConfig> = None;

    // Get controllers from board (portable - no hardcoded indices)
    let controllers = board.usb_controllers();
    // Try SSUSB1 first (index 1), then SSUSB0 (index 0)
    for &idx in &[1u8, 0u8] {
        if idx as usize >= controllers.len() {
            continue;
        }

        if let Some(mut d) = UsbDriver::new(idx as u32) {
            if d.init() {
                if d.xhci.max_ports() == 0 {
                    continue;
                }

                // Poll for device connection (up to 2 seconds)
                let mut has_device = false;
                for _ in 0..20 {
                    for p in 0..d.xhci.max_ports() {
                        if (d.xhci.read_portsc(p) & 1) != 0 {
                            has_device = true;
                            break;
                        }
                    }
                    if has_device { break; }
                    delay_ms(100);
                }

                driver = Some(d);
                active_config = Some(&controllers[idx as usize]);
                break;
            }
        }
    }

    let (mut driver, config) = match (driver, active_config) {
        (Some(d), Some(c)) => (d, c),
        _ => {
            println!("ERROR: No USB controller initialized");
            syscall::exit(1);
        }
    };

    // Test command ring and process events
    driver.send_noop();
    delay_ms(10);
    driver.poll_events();

    // Try port reset to detect devices
    for p in 1..=driver.xhci.max_ports() as u32 {
        driver.reset_port(p);
        delay_ms(50);
    }

    // Process post-reset events
    for _ in 0..5 {
        if !driver.poll_events() { break; }
        delay_ms(10);
    }

    // Register for IRQ (get IRQ number from controller config)
    let irq_url = format_irq_url(config.irq);
    let irq_fd = syscall::scheme_open(&irq_url, 0);
    if irq_fd >= 0 {
        driver.set_irq_fd(irq_fd);
    }

    // Poll for device connection on the active controller
    for _round in 0..30 {
        let mut connected_port: Option<u32> = None;

        for p in 0..driver.xhci.max_ports() {
            if (driver.xhci.read_portsc(p) & 1) != 0 {
                connected_port = Some((p + 1) as u32);
                break;
            }
        }

        if let Some(port) = connected_port {
            driver.enumerate_device(port);
            break;
        }
        delay_ms(1000);
    }

    // Daemonize
    let result = syscall::daemonize();
    if result == 0 {
        println!("USB daemon running (ring buffer mode)");

        const MAX_CLIENTS: usize = 4;
        let mut clients: [Option<BlockServer>; MAX_CLIENTS] = [None, None, None, None];
        let mut num_clients = 0usize;
        let mut poll_counter = 0u32;

        loop {
            poll_counter += 1;
            if poll_counter >= 100 {
                poll_counter = 0;
                driver.poll_events();
            }

            // Accept new client connections
            let accept_result = syscall::port_accept(usb_port);
            if accept_result > 0 && num_clients < MAX_CLIENTS {
                let client_channel = accept_result as u32;
                println!("[usbd] Client connected on channel {}", client_channel);

                // Create BlockServer and complete handshake
                let mut client = BlockServer::new(client_channel);
                if client.try_handshake() {
                    println!("[usbd] Ring handshake complete for PID {}", client.client_pid);
                    // Add to clients array
                    for i in 0..MAX_CLIENTS {
                        if clients[i].is_none() {
                            clients[i] = Some(client);
                            num_clients += 1;
                            break;
                        }
                    }
                } else {
                    println!("[usbd] Ring handshake failed");
                }
            }

            // Process ring requests from all ready clients
            for i in 0..MAX_CLIENTS {
                if let Some(ref client) = clients[i] {
                    if client.is_ready() {
                        client.process_requests(&mut driver);
                    }
                }
            }

            syscall::yield_now();
        }
    } else {
        println!("  Daemonize failed: {}", result);
    }

    // Only reached if daemonize failed
    if irq_fd >= 0 {
        syscall::close(irq_fd as u32);
    }

    syscall::exit(0);
}
