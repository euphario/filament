//! USB userspace driver for BPI-R4 kernel
//!
//! Full USB host controller driver running entirely in userspace.
//! IMPORTANT: This driver does NOT rely on U-Boot initialization.
//! All clocks, resets, PHY, and xHCI must be configured from scratch.
//!
//! - MMIO scheme for device register access
//! - IRQ scheme for interrupt handling
//! - Complete clock/reset, IPPC, PHY, and xHCI initialization

#![no_std]
#![no_main]

use userlib::{println, print, syscall};

// Import from the usb driver library
use usb::{
    // GPIO IPC
    GPIO_CMD_USB_VBUS,
    // Clock/reset control (for diagnostics)
    INFRACFG_AO_BASE, INFRACFG_AO_SIZE,
    INFRA_RST0_STA, INFRA_RST1_CLR, INFRA_RST1_STA,
    INFRA_CG0_STA, INFRA_CG1_CLR, INFRA_CG1_STA, INFRA_CG2_STA,
    RST1_SSUSB_TOP0, RST1_SSUSB_TOP1, CG1_SSUSB0, CG1_SSUSB1,
    // xHCI registers
    xhci_op, xhci_port, xhci_rt, xhci_ir,
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
    Ring, EventRing, ErstEntry, RING_SIZE,
    // MMIO helpers
    MmioRegion, format_mmio_url, delay_ms, delay, print_hex64, print_hex32, print_hex8,
    // Controller helpers
    ParsedPortsc, portsc, XhciController,
    event_completion_code, event_slot_id, event_port_id,
    completion_code, doorbell,
    // Enumeration helpers
    build_enable_slot_trb, build_noop_trb,
    // Hub helpers
    get_hub_descriptor_setup, get_port_status_setup,
    set_port_feature_setup, clear_port_feature_setup,
    // HAL - MediaTek MT7988A specific
    SocHal, Mt7988aHal, mt7988a,
};

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
    // xHCI controller abstraction (from library)
    xhci: XhciController,
    // xHCI data structures
    xhci_mem: u64,          // Virtual address of allocated xHCI memory
    xhci_phys: u64,         // Physical address of allocated xHCI memory
    cmd_ring: Option<Ring>,
    event_ring: Option<EventRing>,
    // IRQ support
    irq_fd: i32,            // IRQ file descriptor (-1 if not registered)
}

impl UsbDriver {
    fn new(controller_id: u32) -> Option<Self> {
        // Create HAL for the target SoC
        let mut hal = Mt7988aHal::new(controller_id);

        // Get addresses from HAL trait
        let mac_phys = hal.mac_base() as u64;
        let phy_phys = hal.phy_base() as u64;
        let phy_size = hal.phy_size();

        println!("  Mapping SSUSB{} MMIO regions...", controller_id);

        // Map MAC (xHCI + IPPC) region
        let mac = MmioRegion::open(mac_phys, mt7988a::MAC_SIZE as u64)?;
        print!("    MAC+IPPC @ 0x");
        print_hex32(mac_phys as u32);
        println!(" -> OK (IPPC at +0x{:x})", mt7988a::IPPC_OFFSET);

        // Map PHY region
        let phy = MmioRegion::open(phy_phys, phy_size as u64)?;
        print!("    PHY @ 0x");
        print_hex32(phy_phys as u32);
        print!(" size=0x");
        print_hex32(phy_size as u32);
        println!(" -> OK");

        // Provide MMIO regions to the HAL
        hal.set_mac(mac);
        hal.set_phy(phy);

        // Run SoC-specific initialization (IPPC + T-PHY)
        if !hal.phy_init() {
            println!("  ERROR: HAL phy_init() failed");
            return None;
        }

        // Extract xHCI controller from HAL (consumes the HAL)
        let xhci = hal.into_controller()?;

        Some(Self {
            xhci,
            xhci_mem: 0,
            xhci_phys: 0,
            cmd_ring: None,
            event_ring: None,
            irq_fd: -1,
        })
    }

    // Delegate register access to XhciController methods
    #[inline(always)]
    fn ippc_read32(&self, offset: usize) -> u32 {
        self.xhci.ippc_read32(offset)
    }

    #[inline(always)]
    fn ippc_write32(&self, offset: usize, value: u32) {
        self.xhci.ippc_write32(offset, value)
    }

    #[inline(always)]
    fn op_read32(&self, offset: usize) -> u32 {
        self.xhci.op_read32(offset)
    }

    #[inline(always)]
    fn op_write32(&self, offset: usize, value: u32) {
        self.xhci.op_write32(offset, value)
    }

    #[inline(always)]
    fn op_write64(&self, offset: usize, value: u64) {
        self.xhci.op_write64(offset, value)
    }

    #[inline(always)]
    fn rt_read32(&self, offset: usize) -> u32 {
        self.xhci.rt_read32(offset)
    }

    #[inline(always)]
    fn rt_read64(&self, offset: usize) -> u64 {
        self.xhci.rt_read64(offset)
    }

    #[inline(always)]
    fn rt_write32(&self, offset: usize, value: u32) {
        self.xhci.rt_write32(offset, value)
    }

    #[inline(always)]
    fn rt_write64(&self, offset: usize, value: u64) {
        self.xhci.rt_write64(offset, value)
    }

    #[inline(always)]
    fn ring_doorbell(&self, slot: u32, target: u32) {
        self.xhci.ring_doorbell(slot, target)
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
        // NOTE: SoC-specific initialization (IPPC, PHY) was done in new() via the HAL.
        // This function only handles generic xHCI initialization.

        // === xHCI Capability Registers (generic xHCI) ===
        if self.xhci.read_capabilities().is_none() {
            println!("  ERROR: Failed to read xHCI capabilities");
            return false;
        }

        // Print port status before reset (for debugging)
        self.xhci.print_port_status_before_reset();

        // === xHCI Halt and Reset (generic xHCI) ===
        if !self.xhci.halt_and_reset() {
            return false;
        }

        // === xHCI Data Structures (driver-specific allocation) ===
        println!();
        println!("=== xHCI Data Structures ===");

        let mut phys_addr: u64 = 0;
        let mem_addr = syscall::mmap_dma(XHCI_MEM_SIZE, &mut phys_addr);
        if mem_addr < 0 {
            println!("  ERROR: Failed to allocate xHCI memory");
            return false;
        }
        self.xhci_mem = mem_addr as u64;
        self.xhci_phys = phys_addr;
        print!("  Allocated xHCI memory: VA=0x");
        print_hex32(self.xhci_mem as u32);
        print!(", PA=0x");
        print_hex32(self.xhci_phys as u32);
        println!(" ({} bytes)", XHCI_MEM_SIZE);

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
            for i in 0..256 {
                let addr = dcbaa_virt.add(i) as u64;
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
        println!("  DCBAA initialized at 0x{:x}", dcbaa_phys);

        // Initialize rings
        self.cmd_ring = Some(Ring::init(cmd_ring_virt, cmd_ring_phys));
        println!("  Command Ring at 0x{:x}", cmd_ring_phys);

        let usbsts_addr = self.xhci.mac.base + self.xhci.op_base as u64 + xhci_op::USBSTS as u64;
        self.event_ring = Some(EventRing::init(evt_ring_virt, evt_ring_phys,
                                                erst_virt, erst_phys, usbsts_addr));
        println!("  Event Ring at 0x{:x}, ERST at 0x{:x}", evt_ring_phys, erst_phys);

        // === Programming xHCI Registers (generic xHCI) ===
        // Use library methods
        self.xhci.program_operational_registers(dcbaa_phys, cmd_ring_phys);
        self.xhci.program_interrupter(erst_phys, evt_ring_phys);

        // === Start Controller (generic xHCI) ===
        if !self.xhci.start() {
            return false;
        }

        // === Power on ports (generic xHCI) ===
        self.xhci.power_on_ports();
        self.xhci.wait_for_link(100000);

        true
    }

    /// Send a No-Op command to test the command ring
    fn send_noop(&mut self) -> bool {
        let cmd_ring = match &mut self.cmd_ring {
            Some(r) => r,
            None => return false,
        };

        println!("  Sending No-Op command...");

        // Use library helper to build TRB
        let trb = build_noop_trb();
        cmd_ring.enqueue(&trb);

        // Ring doorbell for command ring
        self.ring_doorbell(0, doorbell::COMMAND_RING);

        true
    }

    /// Poll for and process events
    fn poll_events(&mut self) -> bool {
        // Collect events first to avoid borrow issues
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

        // Process collected events
        for i in 0..event_count {
            if let Some(ref trb) = events[i] {
                let evt_type = trb.get_type();
                let cc = event_completion_code(trb);

                match evt_type {
                    trb_type::COMMAND_COMPLETION => {
                        print!("  Event: Command Completion, CC=");
                        print_hex32(cc);
                        if completion_code::is_success(cc) {
                            println!(" ({})", completion_code::name(cc));
                        } else {
                            println!(" ({})", completion_code::name(cc));
                        }
                    }
                    trb_type::PORT_STATUS_CHANGE => {
                        let port_id = event_port_id(trb);
                        println!("  Event: Port {} Status Change", port_id);
                        self.handle_port_change(port_id);
                    }
                    trb_type::TRANSFER_EVENT => {
                        println!("  Event: Transfer Complete");
                    }
                    trb_type::HOST_CONTROLLER => {
                        println!("  Event: Host Controller Error!");
                    }
                    _ => {
                        print!("  Event: Unknown type ");
                        print_hex32(evt_type);
                        println!();
                    }
                }
            }
        }

        // Update ERDP to acknowledge events
        let ir0 = xhci_rt::IR0;
        self.rt_write64(ir0 + xhci_ir::ERDP, final_erdp | (1 << 3)); // EHB bit

        true
    }

    /// Handle port status change
    fn handle_port_change(&mut self, port_id: u32) {
        if port_id == 0 || port_id > self.xhci.num_ports {
            return;
        }

        let portsc_off = self.xhci.port_base + ((port_id - 1) as usize * 0x10) + xhci_port::PORTSC;
        let raw = self.xhci.mac.read32(portsc_off);
        let status = ParsedPortsc::from_raw(raw);

        // Clear status change bits by writing 1 to them (W1C)
        let clear_bits = raw & portsc::CHANGE_BITS;
        if clear_bits != 0 {
            self.xhci.mac.write32(portsc_off, status.clear_changes_value());
        }

        print!("    Port {}: ", port_id);
        if !status.connected {
            println!("Disconnected");
        } else {
            print!("Connected ({})", status.speed.full_name());
            if status.enabled {
                println!(" [Enabled]");
            } else {
                println!(" [Not enabled yet]");
            }
        }
    }

    fn print_port_status(&self) {
        println!();
        println!("=== Port Status ===");

        for p in 0..self.xhci.num_ports {
            let portsc_off = self.xhci.port_base + (p as usize * 0x10) + xhci_port::PORTSC;
            let raw = self.xhci.mac.read32(portsc_off);
            let status = ParsedPortsc::from_raw(raw);

            print!("  Port {}: PORTSC=0x", p + 1);
            print_hex32(raw);

            if !status.powered {
                println!(" [Not powered]");
                continue;
            }

            print!(" PLS={}", status.link_state.as_str());

            if !status.connected {
                println!(" [No device]");
                continue;
            }

            print!(" {}Speed", status.speed.as_str());
            if status.enabled {
                print!(" [Enabled]");
            }
            println!();
        }
    }

    /// Check and display USB port power status
    /// This helps diagnose whether VBUS is properly enabled
    fn check_power_status(&self) {
        println!();
        println!("=== USB PORT POWER STATUS ===");
        println!("  (PP=1 means port has power, VBUS is OK)");

        let mut all_powered = true;
        let mut any_connected = false;

        for p in 0..self.xhci.num_ports {
            let portsc_off = self.xhci.port_base + (p as usize * 0x10) + xhci_port::PORTSC;
            let raw = self.xhci.mac.read32(portsc_off);
            let status = ParsedPortsc::from_raw(raw);

            print!("  Port {}: PP={}", p + 1, if status.powered { 1 } else { 0 });

            if !status.powered {
                print!(" [NO POWER!]");
                all_powered = false;
            } else {
                print!(" [POWERED]");
            }

            if status.connected {
                print!(" Device connected");
                any_connected = true;
            } else {
                print!(" No device");
            }

            println!(" PLS={}", status.link_state.as_str());
        }

        println!();
        if all_powered {
            println!("  VBUS Status: ALL PORTS POWERED - OK");
        } else {
            println!("  VBUS Status: SOME PORTS HAVE NO POWER!");
            println!("  Possible causes:");
            println!("    - U-Boot did not enable USB power");
            println!("    - Hardware VBUS enable is not working");
        }

        if any_connected {
            println!("  Device Status: Device(s) detected");
        } else {
            println!("  Device Status: No devices connected");
            println!("    - Is a USB device plugged in?");
            println!("    - BPI-R4 has internal VL822 hub on Type-A port");
        }
        println!();
    }

    /// Reset a port to trigger device enumeration
    fn reset_port(&self, port: u32) -> bool {
        if port == 0 || port > self.xhci.num_ports {
            return false;
        }

        let portsc_off = self.xhci.port_base + ((port - 1) as usize * 0x10) + xhci_port::PORTSC;

        // Read current PORTSC
        let raw = self.xhci.mac.read32(portsc_off);
        let status = ParsedPortsc::from_raw(raw);

        print!("  Port {}: ", port);

        // Skip if not powered
        if !status.powered {
            println!("not powered, skipping");
            return false;
        }

        // Skip if no device connected
        if !status.connected {
            println!("no device connected, skipping reset");
            return false;
        }

        println!("device present, resetting...");

        // Set PR (Port Reset) bit, preserve PP
        let write_val = (raw & portsc::PRESERVE_MASK) | portsc::PR;
        self.xhci.mac.write32(portsc_off, write_val);

        // Wait for reset to complete
        for _ in 0..100 {
            delay(10000);
            let raw = self.xhci.mac.read32(portsc_off);
            let status = ParsedPortsc::from_raw(raw);

            if !status.reset && status.reset_change {
                // Reset complete, clear PRC
                self.xhci.mac.write32(portsc_off, raw | portsc::PRC);
                println!("    Reset complete");
                return true;
            }
        }

        println!("    Reset timeout");
        false
    }

    /// Drain any pending events from the event ring
    /// This ensures the event ring is in a clean state before critical operations
    fn drain_pending_events(&mut self) {
        let mut count = 0u32;
        let mut erdp_to_update: Option<u64> = None;

        if let Some(ref mut event_ring) = self.event_ring {
            loop {
                if let Some(evt) = event_ring.dequeue() {
                    erdp_to_update = Some(event_ring.erdp());
                    count += 1;
                    // Just consume the event
                    let evt_type = evt.get_type();
                    if evt_type != trb_type::COMMAND_COMPLETION && evt_type != trb_type::TRANSFER_EVENT {
                        print!("    [drained event type={}]", evt_type);
                    }
                } else {
                    break;
                }
            }
        }

        if let Some(erdp) = erdp_to_update {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
        }

        if count > 0 {
            println!("    Drained {} pending events", count);
        }
    }

    /// Enable a slot for a new device
    fn enable_slot(&mut self) -> Option<u32> {
        let cmd_ring = self.cmd_ring.as_mut()?;

        println!("  Enabling slot...");

        // Use library helper to build TRB
        let trb = build_enable_slot_trb();
        cmd_ring.enqueue(&trb);
        self.ring_doorbell(0, doorbell::COMMAND_RING);

        // Wait for completion
        delay(10000);

        // Poll for event - collect result first to avoid borrow issues
        let mut result: Option<(u32, u32)> = None;  // (cc, slot_id)
        let mut erdp_to_update: Option<u64> = None;
        let mut other_events = 0u32;

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..20 {
                if let Some(evt) = event_ring.dequeue() {
                    // Always track ERDP for any consumed event
                    erdp_to_update = Some(event_ring.erdp());

                    let evt_type = evt.get_type();
                    if evt_type == trb_type::COMMAND_COMPLETION {
                        let cc = event_completion_code(&evt);
                        let slot_id = event_slot_id(&evt);
                        println!("    Enable Slot event: CC={}, slot={}", cc, slot_id);
                        result = Some((cc, slot_id));
                        break;
                    } else {
                        // Count other events
                        other_events += 1;
                    }
                }
                delay(1000);
            }
        }

        if other_events > 0 {
            println!("    (skipped {} other events)", other_events);
        }

        // Update ERDP for all consumed events
        if let Some(erdp) = erdp_to_update {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
            // Verify ERDP was written
            let erdp_readback = self.rt_read64(xhci_rt::IR0 + xhci_ir::ERDP);
            if (erdp_readback & !0xF) != (erdp & !0xF) {
                print!("    WARNING: ERDP mismatch! wrote=");
                print_hex64(erdp);
                print!(", read=");
                print_hex64(erdp_readback);
                println!();
            }
        }

        // Process result
        if let Some((cc, slot_id)) = result {
            if cc == trb_cc::SUCCESS {
                println!("    Got slot {}", slot_id);
                return Some(slot_id);
            } else {
                print!("    Enable Slot failed: CC=");
                print_hex32(cc);
                println!();
                return None;
            }
        }

        println!("    No response");
        None
    }

    /// Enumerate a device on the given port
    /// Returns the slot ID if successful
    fn enumerate_device(&mut self, port: u32) -> Option<u32> {
        println!();
        println!("=== Enumerating Device on Port {} ===", port);

        // Step 1: Enable slot
        let slot_id = self.enable_slot()?;

        // Step 2: Allocate memory for device context and input context
        // We'll use a simple bump allocator from after the event ring
        // Input Context: ~1088 bytes, Device Context: ~1024 bytes
        // EP0 Transfer Ring: 64 * 16 = 1024 bytes
        let ctx_size = 4096;  // One page for all contexts
        let mut ctx_phys: u64 = 0;
        let ctx_virt = syscall::mmap_dma(ctx_size, &mut ctx_phys);
        if ctx_virt < 0 {
            println!("  Failed to allocate context memory");
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

            // U-Boot pattern: flush EP0 ring to memory after initialization
            for i in 0..64 {
                let addr = ep0_ring_virt.add(i) as u64;
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Get port speed from PORTSC using library helpers
        let portsc_off = self.xhci.port_base + ((port - 1) as usize * 0x10);
        let raw = self.xhci.mac.read32(portsc_off);
        let status = ParsedPortsc::from_raw(raw);
        let speed = status.speed;
        let speed_raw = speed.to_slot_speed();

        println!("  Port {} speed: {} ({})", port, speed_raw, speed.full_name());

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

            // U-Boot pattern: flush input context to memory after CPU writes
            // InputContext is ~1088 bytes, flush entire structure
            let input_addr = input_ctx_virt as u64;
            let input_size = core::mem::size_of::<InputContext>();
            for offset in (0..input_size).step_by(64) {
                let addr = input_addr + offset as u64;
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            // Initialize Device Context
            let device = &mut *device_ctx_virt;
            *device = DeviceContext::new();

            // U-Boot pattern: flush device context to memory
            let device_addr = device_ctx_virt as u64;
            let device_size = core::mem::size_of::<DeviceContext>();
            for offset in (0..device_size).step_by(64) {
                let addr = device_addr + offset as u64;
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Step 4: Set DCBAA entry for this slot
        let dcbaa = self.xhci_mem as *mut u64;
        unsafe {
            *dcbaa.add(slot_id as usize) = device_ctx_phys;

            // U-Boot pattern: flush DCBAA entry to memory
            let dcbaa_entry_addr = dcbaa.add(slot_id as usize) as u64;
            core::arch::asm!(
                "dc cvac, {addr}",
                addr = in(reg) dcbaa_entry_addr,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
        print!("  DCBAA[{}] = 0x", slot_id);
        print_hex32(device_ctx_phys as u32);
        println!();

        // Step 5: Address Device command
        println!("  Sending Address Device command...");
        let cmd_trb_phys: u64;
        {
            let cmd_ring = self.cmd_ring.as_mut()?;
            println!("    Cmd ring: enq={}, cycle={}", cmd_ring.enqueue, cmd_ring.cycle);
            let mut trb = Trb::new();
            trb.param = input_ctx_phys;
            trb.set_type(trb_type::ADDRESS_DEVICE);
            trb.control |= (slot_id & 0xFF) << 24;  // Slot ID in control[31:24]
            // BSR=0 (assign address immediately)
            cmd_trb_phys = cmd_ring.enqueue(&trb);
            print!("    TRB at ");
            print_hex64(cmd_trb_phys);
            println!();
            self.ring_doorbell(0, 0);
        }

        // Wait for completion
        delay(50000);

        // Debug: dump event ring state and first few raw entries
        if let Some(ref event_ring) = self.event_ring {
            println!("    Event ring: deq={}, cycle={}", event_ring.dequeue, event_ring.cycle);
            // Dump first 4 event TRBs to see raw state
            for i in 0..4usize {
                unsafe {
                    let trb = &*event_ring.trbs.add(i);
                    print!("    EvtTRB[{}]: ctrl={:08x} ", i, trb.control);
                    let cycle = trb.control & 1;
                    println!("(cycle={})", cycle);
                }
            }
        }

        // Collect events first to avoid borrow issues
        let mut addr_result: Option<u32> = None;
        let mut last_cc: Option<u32> = None;
        let mut erdp_to_write: Option<u64> = None;
        let mut event_count = 0u32;

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..50 {
                if let Some(evt) = event_ring.dequeue() {
                    let evt_type = evt.get_type();
                    // Store for printing later
                    println!("  Event: type={}, status={:08x}, ctrl={:08x}",
                             evt_type, evt.status, evt.control);
                    event_count += 1;

                    erdp_to_write = Some(event_ring.erdp());

                    if evt_type == trb_type::COMMAND_COMPLETION {
                        let cc = (evt.status >> 24) & 0xFF;
                        last_cc = Some(cc);
                        if cc == trb_cc::SUCCESS {
                            addr_result = Some(slot_id);
                        }
                        break;
                    }
                }
                delay(1000);
            }
        }

        // Update ERDP outside the borrow
        if let Some(erdp) = erdp_to_write {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
        }

        // Report result
        if addr_result.is_some() {
            println!("  Address Device succeeded");
        } else if let Some(cc) = last_cc {
            print!("  Address Device failed: CC=");
            print_hex32(cc);
            println!();
            return None;
        } else {
            println!("  Address Device: no response (events={})", event_count);
            return None;
        }

        // Read back device address from Device Context
        // U-Boot pattern: invalidate cache before reading data that hardware wrote
        let dev_addr = unsafe {
            let device_addr = device_ctx_virt as u64;
            core::arch::asm!(
                "dc civac, {addr}",
                addr = in(reg) device_addr,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            let device = &*device_ctx_virt;
            device.slot.dw3 & 0xFF
        };
        println!("  Device address: {}", dev_addr);

        // Step 6: Get Device Descriptor
        println!("  Getting Device Descriptor...");

        // Allocate buffer for descriptor (64 bytes to be safe)
        let mut desc_phys: u64 = 0;
        let desc_virt = syscall::mmap_dma(4096, &mut desc_phys);
        if desc_virt < 0 {
            println!("  Failed to allocate descriptor buffer");
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

            // U-Boot pattern: flush TRBs to memory after CPU writes (before hardware reads)
            for i in 0..3 {
                let addr = ep0_ring_virt.add(ep0_enqueue + i) as u64;
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
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
        if let Some((cc, remaining)) = transfer_result {

            if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {
                transfer_ok = true;
                // Update enqueue position: used Setup(0), Data(1), Status(2) -> next is 3
                ep0_enqueue = 3;
                println!("  Transfer complete, {} bytes remaining", remaining);
            } else {
                print!("  Transfer failed: CC=");
                print_hex32(cc);
                println!();
            }
        } else {
            println!("  No transfer event received (timeout)");
        }

        if transfer_ok {
            // Parse and print device descriptor
            // U-Boot pattern: invalidate cache before reading data that hardware wrote
            unsafe {
                let buf_addr = desc_buf as u64;
                core::arch::asm!(
                    "dc civac, {addr}",
                    addr = in(reg) buf_addr,
                    options(nostack, preserves_flags)
                );
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));

                let desc = &*(desc_buf as *const DeviceDescriptor);
                println!();
                println!("  === Device Descriptor ===");
                println!("  USB Version: {}.{}", desc.bcd_usb >> 8, (desc.bcd_usb >> 4) & 0xF);
                println!("  Class: {}, Subclass: {}, Protocol: {}",
                         desc.device_class, desc.device_subclass, desc.device_protocol);
                print!("  Vendor ID: 0x");
                print_hex32(desc.vendor_id as u32);
                print!(", Product ID: 0x");
                print_hex32(desc.product_id as u32);
                println!();
                println!("  Max Packet Size EP0: {}", desc.max_packet_size0);
                println!("  Configurations: {}", desc.num_configurations);

                // Check if it's a mass storage device (class 8) or hub (class 9)
                if desc.device_class == 0 {
                    println!("  Class defined at interface level");
                } else if desc.device_class == 8 {
                    println!("  Mass Storage Device!");
                } else if desc.device_class == 9 {
                    println!("  Hub Device - will enumerate downstream devices");
                    // Enumerate devices connected through this hub
                    self.enumerate_hub(slot_id, port, ep0_ring_virt, ep0_ring_phys, &mut ep0_enqueue);
                }
            }
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

                // Flush entire ring after reset
                for i in 0..64 {
                    let addr = ep0_ring_virt.add(i) as u64;
                    core::arch::asm!(
                        "dc cvac, {addr}",
                        addr = in(reg) addr,
                        options(nostack, preserves_flags)
                    );
                }
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));
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

            // U-Boot pattern: flush TRBs to memory after CPU writes (before hardware reads)
            // Flush all TRBs we just wrote (Setup, optionally Data, Status)
            for i in start_idx..=next_idx {
                let addr = ep0_ring_virt.add(i) as u64;
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            // Debug: print TRBs for slot 2 (hub device)
            if slot_id == 2 {
                print!("    TRB[{}] Setup: ctrl=", start_idx);
                print_hex32((*ep0_ring_virt.add(start_idx)).control);
                println!();
                if has_data {
                    print!("    TRB[{}] Data:  ctrl=", start_idx + 1);
                    print_hex32((*ep0_ring_virt.add(start_idx + 1)).control);
                    println!();
                    print!("    TRB[{}] Status: ctrl=", start_idx + 2);
                    print_hex32((*ep0_ring_virt.add(start_idx + 2)).control);
                    println!();
                } else {
                    print!("    TRB[{}] Status: ctrl=", start_idx + 1);
                    print_hex32((*ep0_ring_virt.add(start_idx + 1)).control);
                    println!();
                }
            }
        }

        // Ring doorbell for EP0
        if slot_id == 2 {
            println!("    Ringing doorbell: slot={}, target=1", slot_id);
        }
        self.ring_doorbell(slot_id, 1);

        // Wait for completion - longer wait for hub devices
        let initial_wait = if slot_id == 2 { 100000 } else { 50000 };
        delay(initial_wait);

        // Poll for transfer event
        let mut result: Option<(u32, u32)> = None;
        let mut erdp_to_update: Option<u64> = None;
        let mut event_count = 0u32;
        let mut other_event_types = 0u32;

        // Use more iterations for downstream hub devices
        let max_iterations = if slot_id == 2 { 300 } else { 100 };

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..max_iterations {
                if let Some(evt) = event_ring.dequeue() {
                    let evt_type = evt.get_type();
                    event_count += 1;
                    // Always update ERDP for any consumed event
                    erdp_to_update = Some(event_ring.erdp());

                    if evt_type == trb_type::TRANSFER_EVENT {
                        let cc = (evt.status >> 24) & 0xFF;
                        let remaining = evt.status & 0xFFFFFF;
                        let evt_slot = (evt.control >> 24) & 0xFF;
                        println!("  Transfer event: slot={}, CC={}, remain={}", evt_slot, cc, remaining);
                        result = Some((cc, remaining));
                        break;
                    } else {
                        // Track other event types
                        other_event_types |= 1 << evt_type;
                    }
                }
                delay(1000);
            }

        }

        // If no events found but EINT is set, retry with extra delay
        if result.is_none() && event_count == 0 {
            let usbsts = self.op_read32(xhci_op::USBSTS);
            if (usbsts & (1 << 3)) != 0 {  // EINT set
                if slot_id == 2 {
                    println!("    EINT set, retrying poll...");
                }
                delay(50000);  // Extra 50ms wait
                if let Some(ref mut event_ring) = self.event_ring {
                    for _ in 0..100 {
                        if let Some(evt) = event_ring.dequeue() {
                            let evt_type = evt.get_type();
                            event_count += 1;
                            erdp_to_update = Some(event_ring.erdp());

                            if evt_type == trb_type::TRANSFER_EVENT {
                                let cc = (evt.status >> 24) & 0xFF;
                                let remaining = evt.status & 0xFFFFFF;
                                let evt_slot = (evt.control >> 24) & 0xFF;
                                println!("  Transfer event (retry): slot={}, CC={}, remain={}", evt_slot, cc, remaining);
                                result = Some((cc, remaining));
                                break;
                            } else {
                                other_event_types |= 1 << evt_type;
                            }
                        }
                        delay(1000);
                    }
                }
            }
        }

        if event_count == 0 {
            println!("  (no events received)");
            // Debug for slot 2: check USBSTS and EP context
            if slot_id == 2 {
                let usbsts = self.op_read32(xhci_op::USBSTS);
                print!("    USBSTS=");
                print_hex32(usbsts);
                if usbsts & (1 << 2) != 0 { print!(" HSE"); }  // Host System Error
                if usbsts & (1 << 3) != 0 { print!(" EINT"); } // Event Interrupt
                if usbsts & (1 << 4) != 0 { print!(" PCD"); }  // Port Change Detect
                if usbsts & (1 << 12) != 0 { print!(" HCE"); } // Host Controller Error
                println!();

                // Dump raw event ring entries to debug
                if let Some(ref event_ring) = self.event_ring {
                    println!("    Event ring state: deq={}, cycle={}", event_ring.dequeue, event_ring.cycle);
                    unsafe {
                        for i in 0..4usize {
                            let idx = event_ring.dequeue + i;
                            if idx < 64 {
                                let trb = &*event_ring.trbs.add(idx);
                                print!("    Raw TRB[{}]: ctrl=", idx);
                                print_hex32(trb.control);
                                let trb_cycle = trb.control & 1;
                                print!(" cycle={}", trb_cycle);
                                let trb_type = (trb.control >> 10) & 0x3F;
                                println!(" type={}", trb_type);
                            }
                        }
                    }
                }
            }
        } else if result.is_none() {
            println!("  ({} events, no transfer, types=0x{:x})", event_count, other_event_types);
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

    /// Send SET_CONFIGURATION request
    fn set_configuration(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, config: u8) -> bool {
        println!("  SET_CONFIGURATION({})...", config);

        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            0x00,  // Host-to-device, standard, device
            usb_req::SET_CONFIGURATION,
            config as u16,
            0, 0, 0
        ) {
            Some((cc, _)) if cc == trb_cc::SUCCESS => {
                println!("    Configuration set");
                true
            }
            Some((cc, _)) => {
                print!("    Failed: CC=");
                print_hex32(cc);
                println!();
                false
            }
            None => {
                println!("    No response");
                false
            }
        }
    }

    /// Get Hub Descriptor (SuperSpeed)
    fn get_hub_descriptor(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, buf_phys: u64) -> Option<u8> {
        println!("  Getting Hub Descriptor...");

        // For USB3 hubs, use SS Hub Descriptor type (0x2A)
        // Use hub helper to get setup packet values
        let setup = get_hub_descriptor_setup(true);  // true = SuperSpeed
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            setup.request_type,
            setup.request,
            setup.value,
            setup.index,
            buf_phys,
            setup.length
        ) {
            Some((cc, len)) if completion_code::is_success(cc) => {
                println!("    Got {} bytes", len);
                Some(len as u8)
            }
            Some((cc, _)) => {
                print!("    Failed: CC=");
                print_hex32(cc);
                println!();
                None
            }
            None => {
                println!("    No response");
                None
            }
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
                // U-Boot pattern: invalidate cache before reading data that hardware wrote
                unsafe {
                    core::arch::asm!(
                        "dc civac, {addr}",
                        addr = in(reg) buf_virt,
                        options(nostack, preserves_flags)
                    );
                    core::arch::asm!("dsb sy", options(nostack, preserves_flags));
                }
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
    fn enumerate_hub(&mut self, hub_slot: u32, hub_port: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize) {
        println!();
        println!("=== Enumerating Hub (Slot {}) ===", hub_slot);

        // Allocate buffer for hub descriptor and port status
        let mut buf_phys: u64 = 0;
        let buf_virt = syscall::mmap_dma(4096, &mut buf_phys);
        if buf_virt < 0 {
            println!("  Failed to allocate buffer");
            return;
        }

        // Step 1: Set configuration to activate hub
        if !self.set_configuration(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, 1) {
            return;
        }

        // Step 1.5: Set hub depth (required for USB3 hubs before get_hub_descriptor)
        // Hub depth = 0 for hubs directly connected to root port
        println!("  SET_HUB_DEPTH(0)...");
        match self.control_transfer(
            hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            0x20,  // Host-to-device, Class, Device
            hub::SET_HUB_DEPTH,
            0,     // wValue: depth = 0 (directly connected to root)
            0, 0, 0
        ) {
            Some((cc, _)) if cc == trb_cc::SUCCESS => {
                println!("    Hub depth set");
            }
            Some((cc, _)) => {
                print!("    SET_HUB_DEPTH failed: CC=");
                print_hex32(cc);
                println!();
                // Continue anyway - some hubs might not require this
            }
            None => {
                println!("    SET_HUB_DEPTH: no response (continuing anyway)");
            }
        }

        // Step 2: Get hub descriptor to find port count
        let _hub_desc_len = match self.get_hub_descriptor(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, buf_phys) {
            Some(len) => len,
            None => {
                println!("  Failed to get hub descriptor");
                return;
            }
        };

        // U-Boot pattern: invalidate cache before reading data that hardware wrote
        unsafe {
            core::arch::asm!(
                "dc civac, {addr}",
                addr = in(reg) buf_virt,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
        let hub_desc = unsafe { &*(buf_virt as *const SsHubDescriptor) };
        let num_ports = hub_desc.num_ports;
        let pwr_time = hub_desc.pwr_on_2_pwr_good as u32 * 2;  // Convert to ms
        println!("  Hub has {} ports, power-on time: {}ms", num_ports, pwr_time);

        // Step 2.5: Update hub's Slot Context via EVALUATE_CONTEXT
        // This tells xHCI that this device is a hub so it can route downstream devices
        println!("  Configuring hub slot context...");
        {
            // Allocate input context for evaluate
            let mut eval_ctx_phys: u64 = 0;
            let eval_ctx_virt = syscall::mmap_dma(4096, &mut eval_ctx_phys);
            if eval_ctx_virt >= 0 {
                unsafe {
                    let input = eval_ctx_virt as *mut InputContext;
                    *input = InputContext::new();

                    // Only evaluating Slot Context (A0 = 1)
                    (*input).control.add_flags = 0x1;

                    // Set Slot Context fields properly:
                    // dw0: Hub bit [26], Speed [23:20], Context Entries [31:27]
                    // dw1: Root Hub Port [23:16], Number of Ports [31:24]
                    (*input).slot.set_hub(true);
                    (*input).slot.set_speed(4);  // SuperSpeed
                    (*input).slot.set_context_entries(1);
                    (*input).slot.set_root_hub_port(hub_port);
                    (*input).slot.set_num_ports(num_ports as u32);

                    print!("    Slot ctx: dw0=");
                    print_hex32((*input).slot.dw0);
                    print!(", dw1=");
                    print_hex32((*input).slot.dw1);
                    println!();
                }

                // Send EVALUATE_CONTEXT command
                if let Some(ref mut cmd_ring) = self.cmd_ring {
                    let mut trb = Trb::new();
                    trb.param = eval_ctx_phys;
                    trb.set_type(trb_type::EVALUATE_CONTEXT);
                    trb.control |= (hub_slot & 0xFF) << 24;
                    cmd_ring.enqueue(&trb);
                    self.ring_doorbell(0, 0);

                    delay(10000);

                    // Check result - properly handle all events
                    let mut eval_result: Option<u32> = None;
                    let mut erdp_to_update: Option<u64> = None;

                    if let Some(ref mut event_ring) = self.event_ring {
                        for _ in 0..20 {
                            if let Some(evt) = event_ring.dequeue() {
                                // Always track ERDP for any consumed event
                                erdp_to_update = Some(event_ring.erdp());

                                if evt.get_type() == trb_type::COMMAND_COMPLETION {
                                    let cc = (evt.status >> 24) & 0xFF;
                                    eval_result = Some(cc);
                                    break;
                                }
                                // Ignore other events but acknowledge them
                            }
                            delay(1000);
                        }
                    }

                    // Update ERDP for all consumed events
                    if let Some(erdp) = erdp_to_update {
                        self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
                    }

                    if let Some(cc) = eval_result {
                        if cc == trb_cc::SUCCESS {
                            println!("    Hub context updated");
                        } else {
                            print!("    Evaluate Context failed: CC=");
                            print_hex32(cc);
                            println!();
                        }
                    }
                }
            }
        }

        // Step 3: Power on all ports
        println!("  Powering hub ports...");
        for port in 1..=num_ports as u16 {
            if self.hub_set_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, hub::PORT_POWER) {
                print!("    Port {} powered", port);
            } else {
                print!("    Port {} power failed", port);
            }
            println!();
        }

        // Wait for power stabilization (use hub's time + extra margin)
        let wait_time = pwr_time.max(500);  // At least 500ms
        print!("  Waiting {}ms for power stabilization", wait_time);
        for i in 0..wait_time {
            delay(1000);
            if i % 100 == 99 {
                print!(".");
            }
        }
        println!(" done");

        // Step 4: Check port status and enumerate connected devices
        println!("  Checking port status...");
        let buf_virt_u64 = buf_virt as u64;
        for port in 1..=num_ports as u16 {
            if let Some(ps) = self.hub_get_port_status(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, buf_virt_u64, buf_phys) {
                print!("    Port {}: status=0x{:04x} change=0x{:04x}", port, ps.status, ps.change);

                if (ps.status & hub::PS_CONNECTION) != 0 {
                    println!(" [CONNECTED]");

                    // Clear connection change
                    self.hub_clear_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, hub::C_PORT_CONNECTION);

                    // Reset the port
                    println!("    Resetting port {}...", port);
                    if !self.hub_set_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, hub::PORT_RESET) {
                        println!("      Reset failed");
                        continue;
                    }

                    // Wait for reset to complete
                    for _ in 0..50 {
                        delay(10000);
                        if let Some(ps2) = self.hub_get_port_status(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, buf_virt_u64, buf_phys) {
                            if (ps2.change & hub::PS_C_RESET) != 0 {
                                // Clear reset change
                                self.hub_clear_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, port, hub::C_PORT_RESET);

                                if (ps2.status & hub::PS_ENABLE) != 0 {
                                    println!("      Port enabled after reset");
                                    // Wait for device to be ready after reset (USB 3.0: 10ms min)
                                    println!("      Waiting 100ms for device...");
                                    for _ in 0..100 {
                                        delay(1000);
                                    }
                                    // Now enumerate the device!
                                    self.enumerate_hub_device(hub_slot, hub_port, port as u32);
                                } else {
                                    println!("      Port not enabled after reset");
                                }
                                break;
                            }
                        }
                    }
                } else {
                    println!(" [empty]");
                }
            } else {
                println!("    Port {}: status read failed", port);
            }
        }
    }

    /// Enumerate a device connected to a hub port
    fn enumerate_hub_device(&mut self, hub_slot: u32, root_port: u32, hub_port: u32) {
        println!();
        println!("=== Enumerating Device on Hub Port {} ===", hub_port);

        // Step 1: Enable a new slot for this device
        let slot_id = match self.enable_slot() {
            Some(id) => id,
            None => {
                println!("  Failed to enable slot");
                return;
            }
        };

        // Step 2: Allocate context memory
        let mut ctx_phys: u64 = 0;
        let ctx_virt = syscall::mmap_dma(4096, &mut ctx_phys);
        if ctx_virt < 0 {
            println!("  Failed to allocate context memory");
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

            // U-Boot pattern: flush EP0 ring to memory after initialization
            for i in 0..64 {
                let addr = ep0_ring_virt.add(i) as u64;
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Get hub's port status to determine device speed
        // For now, assume SuperSpeed since we came from a USB3 hub
        let speed = 4u32;  // SuperSpeed
        let max_packet_size = 512u32;

        println!("  Slot {} for hub port {} device", slot_id, hub_port);

        // Step 3: Initialize Input Context
        unsafe {
            let input = &mut *input_ctx_virt;
            *input = InputContext::new();
            input.control.add_flags = 0x3;

            // Slot Context - include hub info
            // Route String: for tier-1 hub device, route string = port number (1-15)
            input.slot.set_route_string(hub_port);
            input.slot.set_speed(speed);
            input.slot.set_context_entries(1);
            input.slot.set_root_hub_port(root_port);

            // dw2 fields for hub-attached devices:
            // - dw2[7:0]  = Hub Slot ID
            // - dw2[15:8] = Parent Port Number
            // - dw2[22:16] = TT Hub Slot ID (LS/FS behind HS hub only)
            // - dw2[31:24] = TT Port Number (LS/FS behind HS hub only)
            //
            // For SuperSpeed devices behind SuperSpeed hubs, the xHCI spec says
            // Hub Slot ID should be set, BUT setting it causes CC=17 (Slot Not Enabled)
            // errors. The routing is handled by Route String in dw0 for SS.
            // Keep dw2=0 for SuperSpeed devices - control and bulk OUT work fine.
            if speed <= 2 {
                // Low/Full-Speed needs TT info
                input.slot.dw2 = (hub_slot << 0) | (hub_port << 8);
                println!("  Setting Hub Slot ID={}, Parent Port={} (LS/FS)", hub_slot, hub_port);
            } else {
                // SuperSpeed/High-Speed - route string handles it
                input.slot.dw2 = 0;
                println!("  SuperSpeed device - routing via route string only");
            }

            print!("  Child slot ctx: dw0=");
            print_hex32(input.slot.dw0);
            print!(", dw1=");
            print_hex32(input.slot.dw1);
            print!(", dw2=");
            print_hex32(input.slot.dw2);
            println!();
            println!("    Route={}, Speed={}, RootPort={}", hub_port, speed, root_port);

            // EP0 Context
            input.endpoints[0].set_ep_type(ep_type::CONTROL);
            input.endpoints[0].set_max_packet_size(max_packet_size);
            input.endpoints[0].set_cerr(3);
            input.endpoints[0].set_tr_dequeue_ptr(ep0_ring_phys, true);
            input.endpoints[0].set_average_trb_length(8);

            // U-Boot pattern: flush input context to memory
            let input_addr = input_ctx_virt as u64;
            let input_size = core::mem::size_of::<InputContext>();
            for offset in (0..input_size).step_by(64) {
                let addr = input_addr + offset as u64;
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            let device = &mut *device_ctx_virt;
            *device = DeviceContext::new();

            // U-Boot pattern: flush device context to memory
            let device_addr = device_ctx_virt as u64;
            let device_size = core::mem::size_of::<DeviceContext>();
            for offset in (0..device_size).step_by(64) {
                let addr = device_addr + offset as u64;
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Step 4: Set DCBAA entry
        let dcbaa = self.xhci_mem as *mut u64;
        unsafe {
            // Debug: show hub's DCBAA entry to verify it's still valid
            let hub_dcbaa_entry = *dcbaa.add(hub_slot as usize);
            print!("  DCBAA[{}] (hub) = ", hub_slot);
            print_hex64(hub_dcbaa_entry);
            println!();

            *dcbaa.add(slot_id as usize) = device_ctx_phys;

            // U-Boot pattern: flush DCBAA entry to memory
            let dcbaa_entry_addr = dcbaa.add(slot_id as usize) as u64;
            core::arch::asm!(
                "dc cvac, {addr}",
                addr = in(reg) dcbaa_entry_addr,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            print!("  DCBAA[{}] (child) = ", slot_id);
            print_hex64(device_ctx_phys);
            println!();

            // Verify input context values
            let input = &*input_ctx_virt;
            print!("  Input control: add=");
            print_hex32(input.control.add_flags);
            print!(", drop=");
            print_hex32(input.control.drop_flags);
            println!();

            print!("  Input EP0: dw0=");
            print_hex32(input.endpoints[0].dw0);
            print!(", dw1=");
            print_hex32(input.endpoints[0].dw1);
            print!(", tr_dequeue=");
            // TR Dequeue is in dw2 (lo) and dw3 (hi)
            let tr_dequeue = (input.endpoints[0].dw2 as u64) | ((input.endpoints[0].dw3 as u64) << 32);
            print_hex64(tr_dequeue);
            println!();
        }

        // Add a delay to ensure Enable Slot has fully completed and device is ready
        println!("  Waiting 200ms before addressing...");
        for _ in 0..200 {
            delay(1000);
        }

        // Step 5: Address Device
        println!("  Addressing device (slot {})...", slot_id);

        // Debug: show command ring state
        if let Some(ref cmd_ring) = self.cmd_ring {
            println!("    Cmd ring: enq={}, cycle={}", cmd_ring.enqueue, cmd_ring.cycle);
        }

        {
            let cmd_ring = match self.cmd_ring.as_mut() {
                Some(r) => r,
                None => return,
            };
            let mut trb = Trb::new();
            trb.param = input_ctx_phys;
            trb.set_type(trb_type::ADDRESS_DEVICE);
            trb.control |= (slot_id & 0xFF) << 24;
            let trb_addr = cmd_ring.enqueue(&trb);
            print!("    TRB at ");
            print_hex64(trb_addr);
            print!(", input_ctx=");
            print_hex64(input_ctx_phys);
            println!();
            self.ring_doorbell(0, 0);
        }

        // Wait for Address Device (can take longer for devices behind hubs)
        for _ in 0..100 {
            delay(1000);
        }

        // Debug: show event ring state
        if let Some(ref event_ring) = self.event_ring {
            println!("    Event ring: deq={}, cycle={}", event_ring.dequeue, event_ring.cycle);
        }

        // Poll for Address Device completion - properly handle all events
        let mut addr_result: Option<u32> = None;
        let mut erdp_to_update: Option<u64> = None;

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..50 {
                if let Some(evt) = event_ring.dequeue() {
                    // Always track ERDP for any consumed event
                    erdp_to_update = Some(event_ring.erdp());

                    println!("    Event: type={}, status={:08x}, ctrl={:08x}",
                             evt.get_type(), evt.status, evt.control);
                    if evt.get_type() == trb_type::COMMAND_COMPLETION {
                        let cc = (evt.status >> 24) & 0xFF;
                        addr_result = Some(cc);
                        break;
                    }
                    // Ignore other events but acknowledge them
                }
                delay(1000);
            }
        }

        // Update ERDP for all consumed events
        if let Some(erdp) = erdp_to_update {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | (1 << 3));
        }

        let addr_ok = match addr_result {
            Some(cc) if cc == trb_cc::SUCCESS => true,
            Some(cc) => {
                print!("  Address failed: CC=");
                print_hex32(cc);
                println!();
                false
            }
            None => false,
        };

        if !addr_ok {
            println!("  Failed to address device");
            return;
        }

        let dev_addr = unsafe { (*device_ctx_virt).slot.dw3 & 0xFF };
        println!("  Device address: {}", dev_addr);

        // Debug: Dump device context after Address Device
        unsafe {
            let dev_ctx = &*device_ctx_virt;
            print!("  Output slot ctx: dw0=");
            print_hex32(dev_ctx.slot.dw0);
            print!(", dw3=");
            print_hex32(dev_ctx.slot.dw3);
            println!();
            print!("  Output EP0 ctx: dw0=");
            print_hex32(dev_ctx.endpoints[0].dw0);
            print!(", dw1=");
            print_hex32(dev_ctx.endpoints[0].dw1);
            println!();
            // Print TR Dequeue Pointer from output context
            let tr_deq_lo = dev_ctx.endpoints[0].dw2;
            let tr_deq_hi = dev_ctx.endpoints[0].dw3;
            let tr_dequeue_out = (tr_deq_lo as u64) | ((tr_deq_hi as u64) << 32);
            print!("    TR Dequeue (output): ");
            print_hex64(tr_dequeue_out);
            print!(" (expected: ");
            print_hex64(ep0_ring_phys | 1);  // with DCS=1
            println!(")");
            let ep0_state = dev_ctx.endpoints[0].dw0 & 0x7;
            println!("    EP0 state: {} (0=Disabled, 1=Running, 2=Halted, 3=Stopped)",
                     ep0_state);
        }

        // Add delay for device to be ready
        println!("  Waiting 100ms for device ready...");
        for _ in 0..100 {
            delay(1000);
        }

        // Step 6: Get Device Descriptor
        println!("  Getting Device Descriptor...");
        let mut desc_phys: u64 = 0;
        let desc_virt = syscall::mmap_dma(4096, &mut desc_phys);
        if desc_virt < 0 {
            return;
        }

        // Track EP0 ring position for this device
        let mut dev_ep0_enqueue = 0usize;

        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, &mut dev_ep0_enqueue,
            0x80, usb_req::GET_DESCRIPTOR,
            (usb_req::DESC_DEVICE << 8) as u16,
            0, desc_phys, 18
        ) {
            Some((cc, len)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
                println!("  Got {} bytes", len);
                let desc = unsafe { &*(desc_virt as *const DeviceDescriptor) };
                println!();
                println!("  === Device Descriptor ===");
                println!("  USB Version: {}.{}", desc.bcd_usb >> 8, (desc.bcd_usb >> 4) & 0xF);
                println!("  Class: {}, Subclass: {}, Protocol: {}",
                         desc.device_class, desc.device_subclass, desc.device_protocol);
                print!("  Vendor ID: 0x");
                print_hex32(desc.vendor_id as u32);
                print!(", Product ID: 0x");
                print_hex32(desc.product_id as u32);
                println!();
                println!("  Configurations: {}", desc.num_configurations);

                // Check if we should probe for mass storage
                let should_probe_msc = desc.device_class == 0 || desc.device_class == msc::CLASS;

                if should_probe_msc {
                    // Get Configuration Descriptor to find interfaces and endpoints
                    self.configure_mass_storage(
                        slot_id, ep0_ring_virt, ep0_ring_phys, &mut dev_ep0_enqueue,
                        desc_virt as *mut u8, desc_phys, device_ctx_virt
                    );
                }
            }
            Some((cc, _)) => {
                print!("  Get descriptor failed: CC=");
                print_hex32(cc);
                println!();
            }
            None => {
                println!("  No response");
            }
        }
    }

    /// Configure a mass storage device
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
        println!();
        println!("=== Probing for Mass Storage ===");

        // Step 1: Get Configuration Descriptor (first 9 bytes to get total length)
        println!("  Reading Configuration Descriptor...");
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            0x80, usb_req::GET_DESCRIPTOR,
            (usb_req::DESC_CONFIGURATION << 8) as u16,
            0, buf_phys, 9
        ) {
            Some((cc, _)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
                // U-Boot pattern: invalidate cache before reading data that hardware wrote
                unsafe {
                    core::arch::asm!(
                        "dc civac, {addr}",
                        addr = in(reg) buf_virt,
                        options(nostack, preserves_flags)
                    );
                    core::arch::asm!("dsb sy", options(nostack, preserves_flags));
                }
                let config = unsafe { &*(buf_virt as *const ConfigurationDescriptor) };
                let total_len = config.total_length;
                println!("  Config: {} interfaces, total {} bytes",
                         config.num_interfaces, total_len);

                // Now get the full configuration descriptor
                if total_len > 9 && total_len <= 256 {
                    delay(10000);
                    match self.control_transfer(
                        slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
                        0x80, usb_req::GET_DESCRIPTOR,
                        (usb_req::DESC_CONFIGURATION << 8) as u16,
                        0, buf_phys, total_len
                    ) {
                        Some((cc, _)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
                            // Invalidate cache for full config descriptor
                            unsafe {
                                // Invalidate multiple cache lines for the full buffer
                                let lines = (total_len as usize + 63) / 64;
                                for i in 0..lines {
                                    let addr = (buf_virt as u64) + (i * 64) as u64;
                                    core::arch::asm!(
                                        "dc civac, {addr}",
                                        addr = in(reg) addr,
                                        options(nostack, preserves_flags)
                                    );
                                }
                                core::arch::asm!("dsb sy", options(nostack, preserves_flags));
                            }
                            // Parse the configuration descriptor hierarchy
                            self.parse_config_descriptor(
                                slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
                                buf_virt, total_len as usize, device_ctx_virt
                            );
                        }
                        Some((cc, _)) => {
                            print!("  Full config read failed: CC=");
                            print_hex32(cc);
                            println!();
                        }
                        None => println!("  No response to full config read"),
                    }
                }
            }
            Some((cc, _)) => {
                print!("  Config descriptor failed: CC=");
                print_hex32(cc);
                println!();
            }
            None => println!("  No response"),
        }
    }

    /// Parse configuration descriptor and set up mass storage if found
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
        let mut _interface_num = 0u8;

        println!("  Parsing {} bytes of config data...", total_len);

        while offset + 2 <= total_len {
            let len = unsafe { *buf.add(offset) } as usize;
            let desc_type = unsafe { *buf.add(offset + 1) };

            if len < 2 || offset + len > total_len {
                break;
            }

            match desc_type {
                2 => {
                    // Configuration Descriptor
                    if len >= 9 {
                        let config = unsafe { &*(buf.add(offset) as *const ConfigurationDescriptor) };
                        config_value = config.configuration_value;
                        println!("    Configuration {}", config_value);
                    }
                }
                4 => {
                    // Interface Descriptor
                    if len >= 9 {
                        let iface = unsafe { &*(buf.add(offset) as *const InterfaceDescriptor) };
                        _interface_num = iface.interface_number;
                        println!("    Interface {}: Class={}, Subclass={}, Protocol={}",
                                 iface.interface_number, iface.interface_class,
                                 iface.interface_subclass, iface.interface_protocol);

                        // Check for Mass Storage Class
                        if iface.interface_class == msc::CLASS &&
                           iface.interface_subclass == msc::SUBCLASS_SCSI &&
                           iface.interface_protocol == msc::PROTOCOL_BOT {
                            println!("    *** MASS STORAGE (SCSI/BOT) ***");
                            is_mass_storage = true;
                        }
                    }
                }
                5 => {
                    // Endpoint Descriptor
                    if len >= 7 {
                        // Read packed struct fields safely to avoid alignment issues
                        let ep_addr = unsafe { core::ptr::read_unaligned(buf.add(offset + 2)) };
                        let ep_attr = unsafe { core::ptr::read_unaligned(buf.add(offset + 3)) };
                        let ep_max_pkt = unsafe {
                            core::ptr::read_unaligned(buf.add(offset + 4) as *const u16)
                        };

                        let ep_num = ep_addr & 0x0F;
                        let ep_dir = if (ep_addr & 0x80) != 0 { "IN" } else { "OUT" };
                        let ep_type_str = match ep_attr & 0x03 {
                            0 => "Control",
                            1 => "Isochronous",
                            2 => "Bulk",
                            3 => "Interrupt",
                            _ => "Unknown",
                        };
                        println!("      EP{} {}: {} (max {} bytes)",
                                 ep_num, ep_dir, ep_type_str, ep_max_pkt);

                        // Track bulk endpoints for mass storage
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
                _ => {
                    // Skip other descriptors
                }
            }

            offset += len;
        }

        // If we found a mass storage interface with bulk endpoints, configure it
        if is_mass_storage && bulk_in_ep != 0 && bulk_out_ep != 0 {
            println!();
            println!("  Found Mass Storage device:");
            print!("    Bulk IN:  EP");
            print_hex8(bulk_in_ep);
            println!(" (max {} bytes)", bulk_in_max_packet);
            print!("    Bulk OUT: EP");
            print_hex8(bulk_out_ep);
            println!(" (max {} bytes)", bulk_out_max_packet);

            // Set Configuration (use existing function, add delay)
            if self.set_configuration(slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, config_value) {
                delay(50000);  // Wait for device to configure
            }

            // Configure bulk endpoints
            self.setup_bulk_endpoints(
                slot_id, device_ctx_virt,
                bulk_in_ep, bulk_in_max_packet,
                bulk_out_ep, bulk_out_max_packet
            );
        }
    }

    /// Set up bulk endpoints for mass storage
    fn setup_bulk_endpoints(
        &mut self,
        slot_id: u32,
        device_ctx_virt: *mut DeviceContext,
        bulk_in_ep: u8,
        bulk_in_max_packet: u16,
        bulk_out_ep: u8,
        bulk_out_max_packet: u16,
    ) {
        println!();
        println!("  Setting up bulk endpoints...");

        // Allocate transfer rings for bulk endpoints
        let mut bulk_in_ring_phys: u64 = 0;
        let bulk_in_ring_virt = syscall::mmap_dma(4096, &mut bulk_in_ring_phys);
        if bulk_in_ring_virt < 0 {
            println!("    Failed to allocate bulk IN ring");
            return;
        }

        let mut bulk_out_ring_phys: u64 = 0;
        let bulk_out_ring_virt = syscall::mmap_dma(4096, &mut bulk_out_ring_phys);
        if bulk_out_ring_virt < 0 {
            println!("    Failed to allocate bulk OUT ring");
            return;
        }

        // Initialize the rings (sets up link TRBs)
        let _bulk_in_ring = Ring::init(bulk_in_ring_virt as *mut Trb, bulk_in_ring_phys);
        let _bulk_out_ring = Ring::init(bulk_out_ring_virt as *mut Trb, bulk_out_ring_phys);

        // Calculate DCI (Device Context Index) for each endpoint
        // DCI = (endpoint_number * 2) + direction (0=OUT, 1=IN)
        let bulk_in_num = (bulk_in_ep & 0x0F) as u32;
        let bulk_out_num = (bulk_out_ep & 0x0F) as u32;
        let bulk_in_dci = bulk_in_num * 2 + 1;   // IN endpoint
        let bulk_out_dci = bulk_out_num * 2;     // OUT endpoint

        println!("    Bulk IN DCI: {}, Bulk OUT DCI: {}", bulk_in_dci, bulk_out_dci);

        // Allocate Input Context for Configure Endpoint command
        let mut input_ctx_phys: u64 = 0;
        let input_ctx_virt = syscall::mmap_dma(4096, &mut input_ctx_phys);
        if input_ctx_virt < 0 {
            println!("    Failed to allocate input context");
            return;
        }

        // Set up Input Context for Configure Endpoint
        unsafe {
            let input = &mut *(input_ctx_virt as *mut InputContext);

            // Clear the input context
            core::ptr::write_bytes(input, 0, 1);

            // Input Control Context - add slot and both bulk endpoints
            // Add flags: bit 0 = slot, bit N = endpoint DCI N
            input.control.add_flags = 1 | (1 << bulk_in_dci) | (1 << bulk_out_dci);

            // Copy current slot context and update context entries
            let dev_ctx = &*device_ctx_virt;
            input.slot = dev_ctx.slot;
            // Update context entries to include our new endpoints
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

            // U-Boot pattern: flush input context to memory after CPU writes
            let input_addr = input_ctx_virt as u64;
            let input_size = core::mem::size_of::<InputContext>();
            for offset in (0..input_size).step_by(64) {
                let addr = input_addr + offset as u64;
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Issue Configure Endpoint command
        println!("    Sending Configure Endpoint command...");
        let mut cmd = Trb::new();
        cmd.param = input_ctx_phys;
        cmd.status = 0;
        cmd.control = (slot_id << 24);
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
                        if cc == trb_cc::SUCCESS {
                            println!("    Configure Endpoint: SUCCESS");
                            config_ok = true;
                        } else {
                            print!("    Configure Endpoint failed: CC=");
                            print_hex32(cc);
                            println!();
                        }
                        // Update ERDP
                        let erdp = event_ring.erdp() | (1 << 3);
                        self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp);
                        break;
                    }
                }
                delay(1000);
            }
        }

        if !config_ok {
            println!("    Configure Endpoint did not complete");
            return;
        }

        // Debug: Check endpoint states after Configure Endpoint
        // CRITICAL: Invalidate cache first - xHCI wrote to output context via DMA
        unsafe {
            let dev_ctx_addr = device_ctx_virt as u64;
            let dev_ctx_size = core::mem::size_of::<DeviceContext>();
            for offset in (0..dev_ctx_size).step_by(64) {
                let addr = dev_ctx_addr + offset as u64;
                core::arch::asm!(
                    "dc civac, {addr}",
                    addr = in(reg) addr,
                    options(nostack, preserves_flags)
                );
            }
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            let dev_ctx = &*device_ctx_virt;
            println!("    Endpoint context details after configure:");

            // Print slot context entries to verify it was updated
            let slot_ctx_entries = (dev_ctx.slot.dw0 >> 27) & 0x1F;
            println!("      Slot Context Entries: {} (should be {})", slot_ctx_entries, bulk_in_dci);

            // Bulk OUT (DCI=2, index=1 in endpoints array)
            let bulk_out_ctx = &dev_ctx.endpoints[(bulk_out_dci - 1) as usize];
            let out_state = bulk_out_ctx.dw0 & 0x7;
            let out_ep_type = (bulk_out_ctx.dw1 >> 3) & 0x7;
            let out_max_pkt = (bulk_out_ctx.dw1 >> 16) & 0xFFFF;
            let out_cerr = (bulk_out_ctx.dw1 >> 1) & 0x3;
            print!("      EP OUT (DCI={}): ", bulk_out_dci);
            print!("state={}, type={} (2=BulkOUT), ", out_state, out_ep_type);
            print!("maxpkt={}, cerr={}", out_max_pkt, out_cerr);
            println!();

            // Bulk IN (DCI=5, index=4 in endpoints array)
            let bulk_in_ctx = &dev_ctx.endpoints[(bulk_in_dci - 1) as usize];
            let in_state = bulk_in_ctx.dw0 & 0x7;
            let in_ep_type = (bulk_in_ctx.dw1 >> 3) & 0x7;
            let in_max_pkt = (bulk_in_ctx.dw1 >> 16) & 0xFFFF;
            let in_cerr = (bulk_in_ctx.dw1 >> 1) & 0x3;
            print!("      EP IN (DCI={}): ", bulk_in_dci);
            print!("state={}, type={} (6=BulkIN), ", in_state, in_ep_type);
            print!("maxpkt={}, cerr={}", in_max_pkt, in_cerr);
            println!();

            // Print full dw0/dw1 for analysis
            print!("      EP OUT raw: dw0=");
            print_hex32(bulk_out_ctx.dw0);
            print!(", dw1=");
            print_hex32(bulk_out_ctx.dw1);
            println!();
            print!("      EP IN raw: dw0=");
            print_hex32(bulk_in_ctx.dw0);
            print!(", dw1=");
            print_hex32(bulk_in_ctx.dw1);
            println!();

            // Print TR dequeue pointers with DCS bits
            let out_deq_raw = (bulk_out_ctx.dw2 as u64) | ((bulk_out_ctx.dw3 as u64) << 32);
            let out_dcs = bulk_out_ctx.dw2 & 1;
            let in_deq_raw = (bulk_in_ctx.dw2 as u64) | ((bulk_in_ctx.dw3 as u64) << 32);
            let in_dcs = bulk_in_ctx.dw2 & 1;
            print!("      EP OUT TR Dequeue: ");
            print_hex64(out_deq_raw & !0xF);
            println!(", DCS={}", out_dcs);
            print!("      EP IN TR Dequeue: ");
            print_hex64(in_deq_raw & !0xF);
            println!(", DCS={}", in_dcs);

            // Verify against ring physical addresses
            print!("      Expected OUT ring: ");
            print_hex64(bulk_out_ring_phys);
            println!();
            print!("      Expected IN ring: ");
            print_hex64(bulk_in_ring_phys);
            println!();
            if (out_deq_raw & !0xF) != bulk_out_ring_phys {
                println!("      *** WARNING: OUT dequeue doesn't match ring base! ***");
            }
            if (in_deq_raw & !0xF) != bulk_in_ring_phys {
                println!("      *** WARNING: IN dequeue doesn't match ring base! ***");
            }
            if out_dcs != 1 || in_dcs != 1 {
                println!("      *** WARNING: DCS not 1 - cycle bits may mismatch! ***");
            }
        }

        // Allocate data buffer for SCSI commands
        let mut data_phys: u64 = 0;
        let data_virt = syscall::mmap_dma(4096, &mut data_phys);
        if data_virt < 0 {
            println!("  Failed to allocate data buffer");
            return;
        }

        // Create BulkContext and run SCSI tests
        let mut ctx = BulkContext::new(
            slot_id,
            bulk_in_dci,
            bulk_out_dci,
            bulk_in_ring_virt as *mut Trb,
            bulk_in_ring_phys,
            bulk_out_ring_virt as *mut Trb,
            bulk_out_ring_phys,
            data_virt as *mut u8,
            data_phys,
            device_ctx_virt,
        );
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

        // Create TRB for CBW (31 bytes)
        unsafe {
            let trb = &mut *ctx.bulk_out_ring.add(ctx.out_enqueue);
            trb.param = ctx.data_phys + CBW_OFFSET as u64;
            trb.status = 31;  // CBW is 31 bytes
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | 1;  // IOC, cycle=1
            ctx.flush_buffer(trb as *const _ as u64, 16);
            ctx.out_enqueue += 1;
        }

        // Ring doorbell
        self.ring_doorbell(ctx.slot_id, ctx.bulk_out_dci);

        // Poll for completion (CBW is fast, no need for IRQ)
        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..100 {
                if let Some(evt) = event_ring.dequeue() {
                    if evt.get_type() == trb_type::TRANSFER_EVENT {
                        let cc = (evt.status >> 24) & 0xFF;
                        // Update ERDP
                        let erdp = event_ring.erdp() | (1 << 3);
                        self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp);
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
        let mac_base = self.xhci.mac.base;
        let rt_base = self.xhci.rt_base;
        let op_base = self.xhci.op_base;

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

        // Invalidate destination buffer before DMA
        ctx.invalidate_buffer(phys_addr, length);

        // Get current DCS from endpoint context
        let dcs = ctx.get_bulk_in_dcs();

        // Create TRB for bulk IN
        unsafe {
            let trb = &mut *ctx.bulk_in_ring.add(ctx.in_enqueue);
            trb.param = phys_addr;
            trb.status = length as u32;
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | dcs;  // IOC, cycle=DCS
            ctx.flush_buffer(trb as *const _ as u64, 16);
            ctx.in_enqueue += 1;
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
            println!("    CBW send failed");
            return (TransferResult::Error(0), None);
        }

        // Data phase (if any)
        if data_length > 0 {
            let (result, bytes) = self.receive_bulk_in_irq(ctx, DATA_OFFSET, data_length as usize);
            if result != TransferResult::Success && result != TransferResult::ShortPacket {
                println!("    Data phase failed: {:?}", result);
                return (result, None);
            }
            println!("    Data received: {} bytes", bytes);
        }

        // CSW phase
        self.receive_csw_irq(ctx)
    }

    /// SCSI READ(10) command - read sectors from device
    /// lba: starting logical block address
    /// count: number of blocks to read (each block is typically 512 bytes)
    /// Returns (success, data slice)
    fn scsi_read_10<'a>(&mut self, ctx: &'a mut BulkContext, lba: u32, count: u16) -> Option<&'a [u8]> {
        // Use library CDB builder (assumes 512 bytes per block)
        let (cmd, data_length) = scsi::build_read_10(lba, count, 512);
        println!("  SCSI READ(10): LBA={}, count={}, bytes={}", lba, count, data_length);

        let (result, csw) = self.scsi_command_in(ctx, &cmd, data_length);

        if let Some(csw) = csw {
            // Copy fields from packed struct to avoid unaligned references
            let sig = csw.signature;
            let tag = csw.tag;
            let residue = csw.data_residue;
            let status = csw.status;

            if sig == msc::CSW_SIGNATURE {
                print!("    CSW: tag={}, residue={}, status=", tag, residue);
                match status {
                    0 => println!("PASSED"),
                    1 => println!("FAILED"),
                    2 => println!("PHASE_ERROR"),
                    _ => println!("{}", status),
                }

                if status == msc::CSW_STATUS_PASSED && (result == TransferResult::Success || result == TransferResult::ShortPacket) {
                    // Return slice to data
                    let actual_bytes = (data_length - residue) as usize;
                    unsafe {
                        let data_ptr = ctx.data_buf.add(DATA_OFFSET);
                        return Some(core::slice::from_raw_parts(data_ptr, actual_bytes));
                    }
                }
            } else {
                print!("    Invalid CSW signature: ");
                print_hex32(sig);
                println!();
            }
        } else {
            println!("    No CSW received: {:?}", result);
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

    /// Run SCSI tests using the new refactored code
    fn run_scsi_tests(&mut self, ctx: &mut BulkContext) {
        println!();
        println!("=== Running SCSI Tests ===");

        // First, wait for device to be ready with TEST_UNIT_READY
        println!("  Waiting for device to initialize...");
        let mut ready = false;
        for attempt in 0..10 {
            if self.scsi_test_unit_ready_ctx(ctx) {
                ready = true;
                break;
            }
            // Wait a bit before retrying
            println!("    Retry {} of 10...", attempt + 1);
            delay_ms(500);
        }

        if !ready {
            println!("  Device not ready after 10 attempts");
            return;
        }

        // Read capacity to know device size
        if let Some((last_lba, block_size)) = self.scsi_read_capacity_10(ctx) {
            println!();

            // Read first sector (LBA 0) - usually contains MBR or GPT
            if let Some(data) = self.scsi_read_10(ctx, 0, 1) {
                println!("    Read {} bytes from LBA 0", data.len());

                // Dump first 64 bytes
                println!("    First 64 bytes:");
                for row in 0..4 {
                    print!("      {:04x}: ", row * 16);
                    for col in 0..16 {
                        if row * 16 + col < data.len() {
                            print_hex8(data[row * 16 + col]);
                            print!(" ");
                        }
                    }
                    println!();
                }

                // Check for MBR signature (0x55, 0xAA at offset 510)
                if data.len() >= 512 {
                    if data[510] == 0x55 && data[511] == 0xAA {
                        println!("    MBR signature found! (valid partition table)");
                    } else {
                        print!("    No MBR signature (bytes 510-511: ");
                        print_hex8(data[510]);
                        print!(" ");
                        print_hex8(data[511]);
                        println!(")");
                    }
                }
            }

            // Also try reading last sector if device is large enough
            if last_lba > 0 && block_size == 512 {
                println!();
                println!("  Reading last sector (LBA {})...", last_lba);
                if let Some(data) = self.scsi_read_10(ctx, last_lba, 1) {
                    println!("    Read {} bytes from last sector", data.len());
                }
            }
        } else {
            println!("  Cannot run read tests without knowing device capacity");
        }
    }
}

// NOTE: Old scsi_inquiry and scsi_test_unit_ready functions (~1000 lines combined) were removed.
// Functionality consolidated into BulkContext-based scsi_test_unit_ready_ctx() and run_scsi_tests().

// =============================================================================
// USB Clock and Reset Control
// =============================================================================

/// Initialize USB subsystem clocks and deassert resets
/// This is required when NOT relying on U-Boot initialization
fn init_usb_clocks_and_resets() -> bool {
    println!("=== USB Clock/Reset Initialization ===");
    println!("  (Not relying on U-Boot)");

    // Open infracfg_ao MMIO region
    let mut url_buf = [0u8; 32];
    let url_len = format_mmio_url(&mut url_buf, INFRACFG_AO_BASE, INFRACFG_AO_SIZE);
    let url = unsafe { core::str::from_utf8_unchecked(&url_buf[..url_len]) };

    let fd = syscall::scheme_open(url, 2);  // O_RDWR
    if fd < 0 {
        println!("  Failed to open infracfg_ao: {}", fd);
        return false;
    }

    // Read virtual address
    let mut virt_buf = [0u8; 8];
    if syscall::read(fd as u32, &mut virt_buf) < 8 {
        println!("  Failed to read infracfg_ao VA");
        syscall::close(fd as u32);
        return false;
    }
    let base = u64::from_le_bytes(virt_buf);

    // Helper to read register
    let read_reg = |offset: usize| -> u32 {
        unsafe {
            let ptr = (base + offset as u64) as *const u32;
            core::ptr::read_volatile(ptr)
        }
    };

    // Helper to write register
    let write_reg = |offset: usize, value: u32| {
        unsafe {
            let ptr = (base + offset as u64) as *mut u32;
            core::ptr::write_volatile(ptr, value);
        }
    };

    // Dump current state for diagnostics
    println!();
    println!("  Current register state:");
    print!("    RST0_STA=0x"); print_hex32(read_reg(INFRA_RST0_STA)); println!();
    print!("    RST1_STA=0x"); print_hex32(read_reg(INFRA_RST1_STA)); println!();
    print!("    CG0_STA=0x"); print_hex32(read_reg(INFRA_CG0_STA)); println!();
    print!("    CG1_STA=0x"); print_hex32(read_reg(INFRA_CG1_STA)); println!();
    print!("    CG2_STA=0x"); print_hex32(read_reg(INFRA_CG2_STA)); println!();

    // Step 1: Deassert USB resets (write to CLR register)
    println!();
    println!("  Deasserting USB resets...");
    let usb_rst_bits = RST1_SSUSB_TOP0 | RST1_SSUSB_TOP1;
    write_reg(INFRA_RST1_CLR, usb_rst_bits);
    delay(1000);

    // Verify resets deasserted
    let rst1_after = read_reg(INFRA_RST1_STA);
    print!("    RST1_STA after: 0x"); print_hex32(rst1_after);
    if (rst1_after & usb_rst_bits) == 0 {
        println!(" (resets deasserted)");
    } else {
        println!(" (WARNING: resets still asserted)");
    }

    // Step 2: Ungate USB clocks (write to CLR register to clear gate bits)
    println!();
    println!("  Ungating USB clocks...");
    let usb_cg_bits = CG1_SSUSB0 | CG1_SSUSB1;
    write_reg(INFRA_CG1_CLR, usb_cg_bits);
    delay(1000);

    // Verify clocks ungated
    let cg1_after = read_reg(INFRA_CG1_STA);
    print!("    CG1_STA after: 0x"); print_hex32(cg1_after);
    if (cg1_after & usb_cg_bits) == 0 {
        println!(" (clocks ungated)");
    } else {
        println!(" (WARNING: clocks still gated)");
    }

    // Additional delay for clocks to stabilize
    delay(10000);

    syscall::close(fd as u32);
    true
}

// =============================================================================
// VBUS Power Control via GPIO Driver IPC
// =============================================================================

/// Request USB VBUS enable from the GPIO expander driver
/// Returns true if successful, false if GPIO driver not available
///
/// BPI-R4 has a PCA9555 GPIO expander on I2C2 that controls USB VBUS.
/// The GPIO driver must be started FIRST (`gpio` command) to enable VBUS.
/// If VBUS isn't enabled, the integrated USB hub won't be powered and
/// devices will appear directly on the root port instead of through the hub.
fn request_vbus_enable() -> bool {
    println!("=== Requesting USB VBUS Power ===");
    println!("  Note: Run 'gpio' command first to enable USB VBUS");

    // Connect to the GPIO driver
    let channel = syscall::port_connect(b"gpio");
    if channel < 0 {
        println!("  GPIO driver not available (error {})", channel);
        println!("  WARNING: USB VBUS may not be enabled!");
        println!("  Run 'gpio' command first, then 'usb'");
        return false;
    }
    let ch = channel as u32;
    println!("  Connected to GPIO driver (channel {})", ch);

    // Send USB VBUS enable command: [GPIO_CMD_USB_VBUS, enable=1]
    let cmd = [GPIO_CMD_USB_VBUS, 1];
    let send_result = syscall::send(ch, &cmd);
    if send_result < 0 {
        println!("  Failed to send VBUS enable command: {}", send_result);
        syscall::channel_close(ch);
        return false;
    }

    // Wait for response
    let mut response = [0u8; 1];
    let recv_result = syscall::receive(ch, &mut response);
    syscall::channel_close(ch);

    if recv_result <= 0 {
        println!("  No response from GPIO driver");
        return false;
    }

    if response[0] as i8 == 0 {
        println!("  USB VBUS power enabled!");
        return true;
    } else {
        println!("  GPIO driver returned error: {}", response[0] as i8);
        return false;
    }
}

// =============================================================================
// IPC Handler for USB scheme
// =============================================================================

use usb::{
    UsbRequest, UsbStatus, UsbMessageHeader, UsbResponseHeader,
    BulkTransferRequest, BulkTransferResponse,
    USB_MSG_MAX_SIZE,
};

/// Connected IPC client
struct IpcClient {
    channel: u32,
    msg_buf: [u8; USB_MSG_MAX_SIZE],
}

impl IpcClient {
    fn new(channel: u32) -> Self {
        Self {
            channel,
            msg_buf: [0u8; USB_MSG_MAX_SIZE],
        }
    }

    /// Try to receive and handle one IPC message (non-blocking would be ideal)
    fn handle_message(&mut self, driver: &mut UsbDriver) -> bool {
        // Try to receive (this may block - future: use poll/select)
        let recv_result = syscall::receive(self.channel, &mut self.msg_buf);
        if recv_result <= 0 {
            return false;
        }

        let recv_len = recv_result as usize;
        if recv_len < UsbMessageHeader::SIZE {
            self.send_error(UsbStatus::InvalidRequest);
            return true;
        }

        // Parse request header
        let header = match UsbMessageHeader::from_bytes(&self.msg_buf) {
            Some(h) => h,
            None => {
                self.send_error(UsbStatus::InvalidRequest);
                return true;
            }
        };

        // Handle request based on type
        match header.request {
            UsbRequest::ListDevices => {
                self.handle_list_devices(driver);
            }
            UsbRequest::GetDeviceInfo => {
                // TODO: implement
                self.send_error(UsbStatus::InvalidRequest);
            }
            UsbRequest::ControlTransfer => {
                // TODO: implement
                self.send_error(UsbStatus::InvalidRequest);
            }
            UsbRequest::BulkOut => {
                // TODO: implement bulk OUT
                self.send_error(UsbStatus::InvalidRequest);
            }
            UsbRequest::BulkIn => {
                // TODO: implement bulk IN
                self.send_error(UsbStatus::InvalidRequest);
            }
            UsbRequest::ConfigureDevice => {
                // TODO: implement
                self.send_error(UsbStatus::InvalidRequest);
            }
            UsbRequest::SetupEndpoints => {
                // TODO: implement
                self.send_error(UsbStatus::InvalidRequest);
            }
        }

        true
    }

    fn handle_list_devices(&mut self, _driver: &UsbDriver) {
        // For now, just return empty list
        // TODO: maintain device list in UsbDriver
        let resp = UsbResponseHeader::new(UsbStatus::Ok, 0);
        let resp_bytes = resp.to_bytes();
        self.msg_buf[..UsbResponseHeader::SIZE].copy_from_slice(&resp_bytes);
        let _ = syscall::send(self.channel, &self.msg_buf[..UsbResponseHeader::SIZE]);
    }

    fn send_error(&mut self, status: UsbStatus) {
        let resp = UsbResponseHeader::new(status, 0);
        let resp_bytes = resp.to_bytes();
        self.msg_buf[..UsbResponseHeader::SIZE].copy_from_slice(&resp_bytes);
        let _ = syscall::send(self.channel, &self.msg_buf[..UsbResponseHeader::SIZE]);
    }
}

// =============================================================================
// Main
// =============================================================================

#[unsafe(no_mangle)]
fn main() {
    println!("========================================");
    println!("  USB Userspace Driver");
    println!("  MT7988A SSUSB (Full Init)");
    println!("========================================");
    println!();

    // Register the "usb" port to ensure only one instance runs
    // If another USB daemon is already running, this will fail
    let port_result = syscall::port_register(b"usb");
    if port_result < 0 {
        if port_result == -17 {  // EEXIST
            println!("ERROR: USB daemon already running");
        } else {
            println!("ERROR: Failed to register USB port: {}", port_result);
        }
        syscall::exit(1);
    }
    let usb_port = port_result as u32;
    println!("  Registered 'usb' port (channel {})", usb_port);

    // Request VBUS enable from GPIO driver before probing
    // This powers the USB ports via the PCA9555 GPIO expander
    println!();
    let _vbus_ok = request_vbus_enable();

    // Wait for power to stabilize after VBUS enable
    println!();
    println!("  Waiting for VBUS power stabilization...");
    for _ in 0..100 {
        delay(10000);  // ~100ms total
    }

    // Initialize USB clocks and deassert resets
    // This is needed when NOT relying on U-Boot
    println!();
    if !init_usb_clocks_and_resets() {
        println!("WARNING: Clock/reset init failed - continuing anyway");
    }

    // Try both USB controllers to find one with a device
    println!();
    println!("=== Probing USB Controllers ===");

    let mut driver: Option<UsbDriver> = None;
    let mut active_controller = 0u32;

    // Try SSUSB0 first, then SSUSB1
    for ctrl in 0..=1 {
        println!();
        println!("--- Trying SSUSB{} ---", ctrl);

        if let Some(mut d) = UsbDriver::new(ctrl) {
            if d.init() {
                // Check and display port power status (VBUS diagnostic)
                d.check_power_status();

                // Wait for hub/device to power up and connect (USB hubs need ~500ms)
                println!("  Waiting for device connection...");
                let mut has_device = false;

                // Poll for up to 2 seconds for device connection
                for attempt in 0..20 {
                    for p in 0..d.xhci.num_ports {
                        let portsc_off = d.xhci.port_base + (p as usize * 0x10);
                        let portsc = d.xhci.mac.read32(portsc_off);

                        if attempt == 0 || attempt == 19 {
                            // Print PORTSC on first and last attempt
                            print!("    Port {} PORTSC=0x", p + 1);
                            print_hex32(portsc);
                            let pls = (portsc >> 5) & 0xF;
                            let ccs = portsc & 1;
                            println!(" CCS={} PLS={}", ccs, pls);
                        }

                        if (portsc & 1) != 0 {  // CCS bit
                            has_device = true;
                            println!("  Found device on port {}!", p + 1);
                        }
                    }

                    if has_device {
                        break;
                    }

                    // Wait 100ms between polls
                    for _ in 0..100 {
                        delay(1000);
                    }
                }

                if has_device {
                    driver = Some(d);
                    active_controller = ctrl;
                    break;
                } else {
                    println!("  No devices found on SSUSB{} after 2s", ctrl);
                }
            }
        } else {
            println!("  Failed to map SSUSB{}", ctrl);
        }
    }

    // If no device found initially, use whichever controller initialized successfully
    let mut driver = match driver {
        Some(d) => d,
        None => {
            println!();
            println!("No USB devices found during initial probe");
            println!("Will monitor SSUSB1 for device connection...");
            active_controller = 1;
            // Try to use SSUSB1 since its PHY works
            match UsbDriver::new(1) {
                Some(mut d) => {
                    if d.init() {
                        // Check power status in fallback case too
                        d.check_power_status();
                        d
                    } else {
                        println!("ERROR: Failed to initialize SSUSB1");
                        syscall::exit(1);
                    }
                }
                None => {
                    println!("ERROR: Failed to map SSUSB1");
                    syscall::exit(1);
                }
            }
        }
    };

    println!();
    println!("=== Monitoring SSUSB{} ===", active_controller);

    driver.print_port_status();

    // Test command ring with No-Op
    println!();
    println!("=== Testing Command Ring ===");
    driver.send_noop();

    // Poll for command completion
    delay(10000);
    driver.poll_events();

    // Check for any pending port status change events
    println!();
    println!("=== Checking for Events ===");
    if !driver.poll_events() {
        println!("  No pending events");
    }

    // Try port reset to detect devices
    println!();
    println!("=== Trying Port Reset ===");
    for p in 1..=driver.xhci.num_ports {
        driver.reset_port(p);
        delay(50000);  // Wait after reset
    }

    // Check port status again
    driver.print_port_status();

    // Poll for port status change events
    println!();
    println!("=== Post-Reset Events ===");
    for _ in 0..5 {
        if driver.poll_events() {
            delay(10000);
        } else {
            break;
        }
    }

    // Final port status
    driver.print_port_status();

    // Register for IRQ
    // GIC SPI 172 = GIC interrupt ID 204 (SPI numbers start at GIC ID 32)
    println!();
    println!("=== IRQ Registration ===");
    let irq_url = "irq:204";
    let irq_fd = syscall::scheme_open(irq_url, 0);
    if irq_fd >= 0 {
        println!("  IRQ {} registered (fd={})", mt7988a::SSUSB1_IRQ, irq_fd);
        driver.set_irq_fd(irq_fd);
    } else {
        println!("  IRQ registration failed: {}", irq_fd);
    }

    println!();
    println!("========================================");
    println!("  USB initialization complete!");
    println!("  Monitoring ports - plug in a device");
    println!("========================================");

    // Also open SSUSB0 for monitoring (Type-A might be there)
    println!();
    println!("=== Opening SSUSB0 for monitoring ===");
    let ssusb0_mac = MmioRegion::open(mt7988a::SSUSB0_MAC_PHYS, mt7988a::MAC_SIZE as u64);

    // Poll ports on BOTH controllers
    println!();
    println!("=== Port Monitor (30 seconds) ===");
    println!("  Monitoring SSUSB0 and SSUSB1 - try plugging device into different ports");

    for round in 0..30 {
        let mut found = false;

        // Check SSUSB0 ports
        if let Some(ref mac0) = ssusb0_mac {
            let cap0 = mac0.read32(mt7988a::IPPC_OFFSET + 0x24);  // IP_XHCI_CAP
            let num_ports0 = ((cap0 >> 8) & 0xF) + (cap0 & 0xF);
            let caplength0 = mac0.read32(0) & 0xFF;
            let port_base0 = caplength0 as usize + 0x400;

            for p in 0..num_ports0 {
                let portsc = mac0.read32(port_base0 + (p as usize * 0x10));
                let ccs = portsc & 1;
                if ccs != 0 {
                    print!("  [{}s] SSUSB0 Port {} CONNECTED! PORTSC=0x", round, p + 1);
                    print_hex32(portsc);
                    println!();
                    found = true;
                }
            }
        }

        // Check SSUSB1 ports (using driver)
        let mut connected_port: Option<u32> = None;
        for p in 0..driver.xhci.num_ports {
            let portsc_off = driver.xhci.port_base + (p as usize * 0x10);
            let portsc = driver.xhci.mac.read32(portsc_off);
            let ccs = portsc & 1;
            if ccs != 0 {
                print!("  [{}s] SSUSB1 Port {} CONNECTED! PORTSC=0x", round, p + 1);
                print_hex32(portsc);
                println!();
                found = true;
                connected_port = Some(p + 1);  // Ports are 1-indexed
            }
        }

        if found {
            // Enumerate the connected device
            if let Some(port) = connected_port {
                driver.enumerate_device(port);
            }
            break;
        }
        if round % 5 == 0 {
            println!("  [{}s] Waiting for device on either controller...", round);
        }
        // Wait ~1 second
        for _ in 0..1000 {
            delay(1000);
        }
    }

    // Final status
    driver.print_port_status();

    // Daemonize - detach from shell and run in background
    let result = syscall::daemonize();
    if result == 0 {
        println!("  Daemonized - running as background USB driver");
        println!("  Use 'ps' to see status, 'kill <pid>' to stop");
        println!("  IPC: Listening for class driver connections on 'usb' port");

        // IPC client list (simple array for now)
        const MAX_CLIENTS: usize = 4;
        let mut clients: [Option<IpcClient>; MAX_CLIENTS] = [None, None, None, None];
        let mut num_clients = 0usize;

        // Run forever as daemon, polling for USB events and handling IPC
        let mut poll_counter = 0u32;
        loop {
            // Poll for USB events periodically
            poll_counter += 1;
            if poll_counter >= 100 {
                poll_counter = 0;
                driver.poll_events();
            }

            // Try to accept new client connections (non-blocking)
            let accept_result = syscall::port_accept(usb_port);
            if accept_result > 0 && num_clients < MAX_CLIENTS {
                let client_channel = accept_result as u32;
                println!("  IPC: New client connected (channel {})", client_channel);
                // Find empty slot
                for i in 0..MAX_CLIENTS {
                    if clients[i].is_none() {
                        clients[i] = Some(IpcClient::new(client_channel));
                        num_clients += 1;
                        break;
                    }
                }
            }

            // Handle messages from connected clients
            // Note: This is a simple round-robin approach
            // A better approach would use poll/select for non-blocking I/O
            for i in 0..MAX_CLIENTS {
                if let Some(ref mut client) = clients[i] {
                    // TODO: Use non-blocking receive when available
                    // For now, we skip clients that don't have pending messages
                    // This won't work well without proper async I/O support
                    let _ = client.handle_message(&mut driver);
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
