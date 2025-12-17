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
    // Hardware addresses
    SSUSB0_MAC_PHYS, SSUSB0_PHY_PHYS, SSUSB0_PHY_SIZE, SSUSB0_IRQ,
    SSUSB1_MAC_PHYS, SSUSB1_PHY_PHYS, SSUSB1_PHY_SIZE, SSUSB1_IRQ,
    SSUSB_MAC_SIZE, SSUSB_IPPC_OFFSET,
    // Clock/reset control
    INFRACFG_AO_BASE, INFRACFG_AO_SIZE,
    INFRA_RST0_STA, INFRA_RST1_SET, INFRA_RST1_CLR, INFRA_RST1_STA,
    INFRA_CG0_STA, INFRA_CG1_SET, INFRA_CG1_CLR, INFRA_CG1_STA, INFRA_CG2_STA,
    RST1_SSUSB_TOP0, RST1_SSUSB_TOP1, CG1_SSUSB0, CG1_SSUSB1,
    // GPIO IPC
    GPIO_CMD_USB_VBUS,
    // PHY modules
    ippc, tphy,
    // xHCI registers
    xhci_cap, xhci_op, usbcmd, usbsts, xhci_port, xhci_rt, xhci_ir,
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
    controller: u32,        // 0 = SSUSB0 (XS-PHY), 1 = SSUSB1 (T-PHY)
    mac: MmioRegion,        // Also contains IPPC at offset 0x3e00
    phy: MmioRegion,
    // xHCI register offsets (from mac.base)
    op_base: usize,         // Operational registers
    rt_base: usize,         // Runtime registers
    db_base: usize,         // Doorbell registers
    port_base: usize,       // Port register array
    // Capability info
    caplength: u32,
    num_ports: u32,
    max_slots: u32,
    // xHCI data structures
    xhci_mem: u64,          // Virtual address of allocated xHCI memory
    xhci_phys: u64,         // Physical address of allocated xHCI memory
    cmd_ring: Option<Ring>,
    event_ring: Option<EventRing>,
    // IRQ support
    irq_fd: i32,            // IRQ file descriptor (-1 if not registered)
}

impl UsbDriver {
    fn new(controller: u32) -> Option<Self> {
        let (mac_phys, phy_phys, phy_size, _irq) = match controller {
            0 => (SSUSB0_MAC_PHYS, SSUSB0_PHY_PHYS, SSUSB0_PHY_SIZE, SSUSB0_IRQ),
            1 => (SSUSB1_MAC_PHYS, SSUSB1_PHY_PHYS, SSUSB1_PHY_SIZE, SSUSB1_IRQ),
            _ => return None,
        };

        println!("  Mapping SSUSB{} MMIO regions...", controller);

        let mac = MmioRegion::open(mac_phys, SSUSB_MAC_SIZE)?;
        print!("    MAC+IPPC @ 0x");
        print_hex32(mac_phys as u32);
        println!(" -> OK (IPPC at +0x3e00)");

        let phy = MmioRegion::open(phy_phys, phy_size)?;
        print!("    PHY @ 0x");
        print_hex32(phy_phys as u32);
        print!(" size=0x");
        print_hex32(phy_size as u32);
        println!(" -> OK");

        Some(Self {
            controller,
            mac,
            phy,
            op_base: 0,
            rt_base: 0,
            db_base: 0,
            port_base: 0,
            caplength: 0,
            num_ports: 0,
            max_slots: 0,
            xhci_mem: 0,
            xhci_phys: 0,
            cmd_ring: None,
            event_ring: None,
            irq_fd: -1,
        })
    }

    /// Read from IPPC registers (offset from MAC base)
    #[inline(always)]
    fn ippc_read32(&self, offset: usize) -> u32 {
        self.mac.read32(SSUSB_IPPC_OFFSET + offset)
    }

    /// Write to IPPC registers (offset from MAC base)
    #[inline(always)]
    fn ippc_write32(&self, offset: usize, value: u32) {
        self.mac.write32(SSUSB_IPPC_OFFSET + offset, value)
    }

    /// Read operational register
    #[inline(always)]
    fn op_read32(&self, offset: usize) -> u32 {
        self.mac.read32(self.op_base + offset)
    }

    /// Write operational register
    #[inline(always)]
    fn op_write32(&self, offset: usize, value: u32) {
        self.mac.write32(self.op_base + offset, value)
    }

    /// Write 64-bit operational register
    #[inline(always)]
    fn op_write64(&self, offset: usize, value: u64) {
        self.mac.write64(self.op_base + offset, value)
    }

    /// Read runtime register
    #[inline(always)]
    fn rt_read32(&self, offset: usize) -> u32 {
        self.mac.read32(self.rt_base + offset)
    }

    /// Read 64-bit runtime register
    #[inline(always)]
    fn rt_read64(&self, offset: usize) -> u64 {
        self.mac.read64(self.rt_base + offset)
    }

    /// Write runtime register
    #[inline(always)]
    fn rt_write32(&self, offset: usize, value: u32) {
        self.mac.write32(self.rt_base + offset, value)
    }

    /// Write 64-bit runtime register
    #[inline(always)]
    fn rt_write64(&self, offset: usize, value: u64) {
        self.mac.write64(self.rt_base + offset, value)
    }

    /// Ring doorbell
    /// Includes memory barrier and PCIe serialization to ensure DMA writes are visible
    #[inline(always)]
    fn ring_doorbell(&self, slot: u32, target: u32) {
        // Data synchronization barrier to ensure all memory writes
        // (including DMA ring TRBs) are complete from CPU perspective
        unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)) };

        // CRITICAL: On non-coherent ARM systems, we need to force PCIe write serialization.
        // Reading from the device forces any pending PCIe writes to complete before
        // the doorbell write goes out. This prevents a race where xHCI sees the doorbell
        // before seeing the TRB data. (Based on Linux xHCI driver fix for VL805)
        let _ = self.mac.read32(self.db_base);  // Read doorbell[0] to force serialization

        // Now write the actual doorbell
        self.mac.write32(self.db_base + (slot as usize * 4), target);
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
        println!();
        println!("=== IPPC Initialization ===");

        // Software reset
        println!("  Software reset...");
        self.ippc_write32(ippc::IP_PW_CTRL0, ippc::IP_SW_RST);
        delay(1000);
        self.ippc_write32(ippc::IP_PW_CTRL0, 0);
        delay(1000);

        // Power on host, power down device
        println!("  Host power on...");
        self.ippc_write32(ippc::IP_PW_CTRL2, ippc::IP_DEV_PDN);
        self.ippc_write32(ippc::IP_PW_CTRL1, 0); // Clear host PDN

        // Read port counts
        let cap = self.ippc_read32(ippc::IP_XHCI_CAP);
        let u3_ports = (cap >> 8) & 0xF;
        let u2_ports = cap & 0xF;
        println!("  Ports: {} USB3, {} USB2", u3_ports, u2_ports);

        // Configure U3 ports (if any)
        for i in 0..u3_ports {
            let offset = ippc::U3_CTRL_P0 + (i as usize * 8);
            let mut ctrl = self.ippc_read32(offset);
            ctrl &= !(ippc::PORT_PDN | ippc::PORT_DIS);
            ctrl |= ippc::PORT_HOST_SEL;
            self.ippc_write32(offset, ctrl);
        }

        // Configure U2 ports
        for i in 0..u2_ports {
            let offset = ippc::U2_CTRL_P0 + (i as usize * 8);
            let mut ctrl = self.ippc_read32(offset);
            ctrl &= !(ippc::PORT_PDN | ippc::PORT_DIS);
            ctrl |= ippc::PORT_HOST_SEL;
            self.ippc_write32(offset, ctrl);
        }

        // Wait for clocks
        println!("  Waiting for clocks...");
        let required = ippc::SYSPLL_STABLE | ippc::REF_RST | ippc::SYS125_RST | ippc::XHCI_RST;
        for i in 0..1000 {
            let sts = self.ippc_read32(ippc::IP_PW_STS1);
            if (sts & required) == required {
                println!("  Clocks stable (after {} iterations)", i);
                break;
            }
            if i == 999 {
                print!("  Clock timeout! IP_PW_STS1=0x");
                print_hex32(sts);
                println!();
                return false;
            }
            delay(1000);
        }

        println!();
        println!("=== xHCI Capability Registers ===");

        // Read capability registers
        self.caplength = self.mac.read32(xhci_cap::CAPLENGTH) & 0xFF;
        let version = (self.mac.read32(xhci_cap::CAPLENGTH) >> 16) & 0xFFFF;
        println!("  xHCI version: {}.{}", version >> 8, version & 0xFF);
        println!("  Capability length: 0x{:x}", self.caplength);

        let hcsparams1 = self.mac.read32(xhci_cap::HCSPARAMS1);
        self.num_ports = (hcsparams1 >> 24) & 0xFF;
        self.max_slots = hcsparams1 & 0xFF;
        println!("  Max ports: {}, Max slots: {}", self.num_ports, self.max_slots);

        let hcsparams2 = self.mac.read32(xhci_cap::HCSPARAMS2);
        let max_scratchpad = ((hcsparams2 >> 27) & 0x1F) | ((hcsparams2 >> 16) & 0x3E0);
        println!("  Max scratchpad buffers: {}", max_scratchpad);

        // Calculate register offsets
        self.op_base = self.caplength as usize;
        self.rt_base = self.mac.read32(xhci_cap::RTSOFF) as usize & !0x1F;
        self.db_base = self.mac.read32(xhci_cap::DBOFF) as usize & !0x3;
        self.port_base = self.op_base + 0x400;
        println!("  Op base: 0x{:x}, Runtime: 0x{:x}, Doorbell: 0x{:x}",
                 self.op_base, self.rt_base, self.db_base);

        // Check port status BEFORE touching anything
        println!();
        println!("=== Current Port Status (before reset) ===");
        for p in 0..self.num_ports {
            let portsc_off = self.port_base + (p as usize * 0x10);
            let portsc = self.mac.read32(portsc_off);
            print!("  Port {}: PORTSC=0x", p + 1);
            print_hex32(portsc);
            let ccs = portsc & 1;
            let ped = (portsc >> 1) & 1;
            let pls = (portsc >> 5) & 0xF;
            let speed = (portsc >> 10) & 0xF;
            println!(" CCS={} PED={} PLS={} Speed={}", ccs, ped, pls, speed);
        }

        println!();
        println!("=== xHCI Reset ===");

        // Halt controller
        println!("  Halting controller...");
        let cmd = self.op_read32(xhci_op::USBCMD);
        self.op_write32(xhci_op::USBCMD, cmd & !usbcmd::RUN);

        // Wait for halt
        for _ in 0..100 {
            let sts = self.op_read32(xhci_op::USBSTS);
            if (sts & usbsts::HCH) != 0 {
                break;
            }
            delay(1000);
        }

        // Reset
        println!("  Resetting...");
        self.op_write32(xhci_op::USBCMD, usbcmd::HCRST);

        // Wait for reset complete
        for _ in 0..100 {
            let cmd = self.op_read32(xhci_op::USBCMD);
            let sts = self.op_read32(xhci_op::USBSTS);
            if (cmd & usbcmd::HCRST) == 0 && (sts & usbsts::CNR) == 0 {
                println!("  Reset complete");
                break;
            }
            delay(1000);
        }

        // Initialize PHY for host mode
        // Both SSUSB0 (XFI T-PHY) and SSUSB1 (T-PHY v2) use similar register layout
        println!();
        println!("=== PHY Initialization ===");

        // T-PHY v2 has USB2 PHY at COM bank offset 0x300
        // Try multiple offsets to find the right one
        let offsets_to_try: &[(usize, &str)] = &[
            (0x300, "COM+0x300"),      // Standard T-PHY v2 COM bank
            (0x000, "base+0x000"),     // Direct offset
            (0x100, "base+0x100"),     // Alternative
            (0x400, "base+0x400"),     // Another alternative
        ];

        let phy_name = if self.controller == 0 { "XFI T-PHY" } else { "T-PHY v2" };
        println!("  Configuring {} for host mode...", phy_name);

        // Scan for valid PHY registers (non-zero reads indicate valid registers)
        let mut found_offset: Option<usize> = None;
        for &(base_off, name) in offsets_to_try {
            let dtm1_off = base_off + 0x6C;
            let dtm1 = self.phy.read32(dtm1_off);
            if dtm1 != 0 || base_off == 0x300 {
                // Either we read non-zero, or try the standard offset
                print!("    {} DTM1@0x{:x}=0x", name, dtm1_off);
                print_hex32(dtm1);
                println!();
                if found_offset.is_none() {
                    found_offset = Some(base_off);
                }
            }
        }

        // Use the found offset or default to standard T-PHY layout
        let com_offset = found_offset.unwrap_or(0x300);
        let dtm1_offset = com_offset + 0x6C;

        let dtm1 = self.phy.read32(dtm1_offset);
        print!("    Using offset 0x{:x}, DTM1 before: 0x", dtm1_offset);
        print_hex32(dtm1);
        println!();

        // Set host mode bits: force vbusvalid/avalid, set vbusvalid/avalid
        let new_dtm1 = dtm1 | tphy::HOST_MODE;
        self.phy.write32(dtm1_offset, new_dtm1);

        delay(1000);

        let dtm1_after = self.phy.read32(dtm1_offset);
        print!("    After:  DTM1=0x");
        print_hex32(dtm1_after);
        if dtm1_after != dtm1 {
            println!(" (host mode configured)");
        } else {
            println!(" (WARNING: register unchanged!)");
        }

        // Force USB2 PHY power on via IPPC U2_PHY_PLL register
        // This is critical for device detection!
        println!();
        println!("=== Forcing USB2 PHY Power ===");
        let pll = self.ippc_read32(ippc::U2_PHY_PLL);
        print!("  U2_PHY_PLL before: 0x");
        print_hex32(pll);
        println!();
        self.ippc_write32(ippc::U2_PHY_PLL, pll | 1);  // Set bit 0 to force power on
        delay(1000);
        let pll_after = self.ippc_read32(ippc::U2_PHY_PLL);
        print!("  U2_PHY_PLL after:  0x");
        print_hex32(pll_after);
        println!(" (PHY power forced on)");

        println!();
        println!("=== xHCI Data Structures ===");

        // Allocate DMA-capable memory for xHCI structures
        // This returns both virtual and physical addresses
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

        // Memory layout:
        // Page 0 (0x000): DCBAA (2KB, 64-byte aligned)
        // Page 0 (0x800): ERST (64 bytes, 64-byte aligned)
        // Page 1 (0x1000): Command Ring (64 TRBs)
        // Page 2 (0x2000): Event Ring (64 TRBs)
        let dcbaa_virt = self.xhci_mem as *mut u64;
        let dcbaa_phys = self.xhci_phys;
        let erst_virt = (self.xhci_mem + 0x800) as *mut ErstEntry;
        let erst_phys = self.xhci_phys + 0x800;
        let cmd_ring_virt = (self.xhci_mem + 0x1000) as *mut Trb;
        let cmd_ring_phys = self.xhci_phys + 0x1000;
        let evt_ring_virt = (self.xhci_mem + 0x2000) as *mut Trb;
        let evt_ring_phys = self.xhci_phys + 0x2000;

        // Zero DCBAA
        unsafe {
            for i in 0..256 {
                *dcbaa_virt.add(i) = 0;
            }
            // U-Boot pattern: flush DCBAA to memory after CPU writes
            // DCBAA is 2KB (256 * 8 bytes), flush all cache lines
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

        // Initialize Command Ring
        self.cmd_ring = Some(Ring::init(cmd_ring_virt, cmd_ring_phys));
        println!("  Command Ring at 0x{:x}", cmd_ring_phys);

        // Initialize Event Ring
        // Pass USBSTS address for PCIe serialization on non-coherent ARM systems
        let usbsts_addr = self.mac.base + self.op_base as u64 + xhci_op::USBSTS as u64;
        self.event_ring = Some(EventRing::init(evt_ring_virt, evt_ring_phys,
                                                erst_virt, erst_phys, usbsts_addr));
        println!("  Event Ring at 0x{:x}, ERST at 0x{:x}", evt_ring_phys, erst_phys);

        println!();
        println!("=== Programming xHCI Registers ===");

        // Set max device slots
        let config = self.max_slots.min(16);  // Limit to 16 slots
        self.op_write32(xhci_op::CONFIG, config);
        println!("  MaxSlotsEn = {}", config);

        // Set DCBAAP (Device Context Base Address Array Pointer)
        self.op_write64(xhci_op::DCBAAP, dcbaa_phys);
        println!("  DCBAAP = 0x{:x}", dcbaa_phys);

        // Set CRCR (Command Ring Control Register)
        // Bit 0 = RCS (Ring Cycle State) = 1
        self.op_write64(xhci_op::CRCR, cmd_ring_phys | 1);
        println!("  CRCR = 0x{:x}", cmd_ring_phys | 1);

        // Program Event Ring: Interrupter 0
        let ir0 = xhci_rt::IR0;

        // Set ERSTSZ (Event Ring Segment Table Size) = 1 segment
        self.rt_write32(ir0 + xhci_ir::ERSTSZ, 1);

        // Set ERDP (Event Ring Dequeue Pointer)
        self.rt_write64(ir0 + xhci_ir::ERDP, evt_ring_phys);

        // Set ERSTBA (Event Ring Segment Table Base Address)
        // This must be done after ERSTSZ
        self.rt_write64(ir0 + xhci_ir::ERSTBA, erst_phys);
        println!("  Interrupter 0 configured");

        // Enable interrupter
        self.rt_write32(ir0 + xhci_ir::IMAN, 0x2);  // IE=1, IP=0

        println!();
        println!("=== Starting Controller ===");

        // Start controller with interrupt enable
        let cmd = self.op_read32(xhci_op::USBCMD);
        self.op_write32(xhci_op::USBCMD, cmd | usbcmd::RUN | usbcmd::INTE);

        // Wait for running
        delay(10000);
        let sts = self.op_read32(xhci_op::USBSTS);
        if (sts & usbsts::HCH) == 0 {
            println!("  Controller running");
        } else {
            println!("  ERROR: Controller not running!");
            return false;
        }

        // Power on ports
        println!("  Powering ports...");
        for p in 0..self.num_ports {
            let portsc_off = self.port_base + (p as usize * 0x10) + xhci_port::PORTSC;
            let portsc = self.mac.read32(portsc_off);
            // Set PP (Port Power) bit 9
            self.mac.write32(portsc_off, portsc | (1 << 9));
        }

        // Wait for link stabilization
        println!("  Waiting for link...");
        delay(100000);

        true
    }

    /// Send a No-Op command to test the command ring
    fn send_noop(&mut self) -> bool {
        let cmd_ring = match &mut self.cmd_ring {
            Some(r) => r,
            None => return false,
        };

        println!("  Sending No-Op command...");

        let mut trb = Trb::new();
        trb.set_type(trb_type::NOOP_CMD);

        cmd_ring.enqueue(&trb);

        // Ring doorbell 0 (host controller) with target 0 (command ring)
        self.ring_doorbell(0, 0);

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
            if let Some(trb) = events[i] {
                let trb_type = trb.get_type();
                let completion_code = (trb.status >> 24) & 0xFF;

                match trb_type {
                    trb_type::COMMAND_COMPLETION => {
                        print!("  Event: Command Completion, CC=");
                        print_hex32(completion_code);
                        if completion_code == trb_cc::SUCCESS {
                            println!(" (Success)");
                        } else {
                            println!(" (Error)");
                        }
                    }
                    trb_type::PORT_STATUS_CHANGE => {
                        let port_id = (trb.param >> 24) as u32 & 0xFF;
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
                        print_hex32(trb_type);
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
        if port_id == 0 || port_id > self.num_ports {
            return;
        }

        let portsc_off = self.port_base + ((port_id - 1) as usize * 0x10) + xhci_port::PORTSC;
        let portsc = self.mac.read32(portsc_off);

        // Clear status change bits by writing 1 to them (W1C)
        // Preserve PP (bit 9), clear PRC (bit 21), CSC (bit 17), etc.
        let clear_bits = portsc & 0x00FE0000;  // Status change bits
        if clear_bits != 0 {
            self.mac.write32(portsc_off, (portsc & 0xFE01FFFF) | clear_bits);
        }

        let ccs = (portsc >> 0) & 1;
        let ped = (portsc >> 1) & 1;
        let speed = (portsc >> 10) & 0xF;

        print!("    Port {}: ", port_id);
        if ccs == 0 {
            println!("Disconnected");
        } else {
            let speed_str = match speed {
                1 => "Full Speed",
                2 => "Low Speed",
                3 => "High Speed",
                4 => "Super Speed",
                _ => "Unknown",
            };
            print!("Connected ({})", speed_str);
            if ped != 0 {
                println!(" [Enabled]");
            } else {
                println!(" [Not enabled yet]");
            }
        }
    }

    fn print_port_status(&self) {
        println!();
        println!("=== Port Status ===");

        for p in 0..self.num_ports {
            let portsc_off = self.port_base + (p as usize * 0x10) + xhci_port::PORTSC;
            let portsc = self.mac.read32(portsc_off);

            let ccs = (portsc >> 0) & 1;
            let ped = (portsc >> 1) & 1;
            let pp = (portsc >> 9) & 1;
            let speed = (portsc >> 10) & 0xF;
            let pls = (portsc >> 5) & 0xF;

            print!("  Port {}: PORTSC=0x", p + 1);
            print_hex32(portsc);

            if pp == 0 {
                println!(" [Not powered]");
                continue;
            }

            let pls_str = match pls {
                0 => "U0",
                1 => "U1",
                2 => "U2",
                3 => "U3",
                4 => "Disabled",
                5 => "RxDetect",
                6 => "Inactive",
                7 => "Polling",
                8 => "Recovery",
                9 => "Hot Reset",
                10 => "Compliance",
                11 => "Test",
                15 => "Resume",
                _ => "?",
            };

            print!(" PLS={}", pls_str);

            if ccs == 0 {
                println!(" [No device]");
                continue;
            }

            let speed_str = match speed {
                1 => "FS",
                2 => "LS",
                3 => "HS",
                4 => "SS",
                5 => "SS+",
                _ => "?",
            };

            print!(" {}Speed", speed_str);
            if ped == 1 {
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

        for p in 0..self.num_ports {
            let portsc_off = self.port_base + (p as usize * 0x10) + xhci_port::PORTSC;
            let portsc = self.mac.read32(portsc_off);

            let ccs = (portsc >> 0) & 1;  // Current Connect Status
            let pp = (portsc >> 9) & 1;   // Port Power
            let pls = (portsc >> 5) & 0xF; // Port Link State

            print!("  Port {}: PP={}", p + 1, pp);

            if pp == 0 {
                print!(" [NO POWER!]");
                all_powered = false;
            } else {
                print!(" [POWERED]");
            }

            if ccs != 0 {
                print!(" Device connected");
                any_connected = true;
            } else {
                print!(" No device");
            }

            // Show link state for debugging
            let pls_str = match pls {
                0 => "U0",
                1 => "U1",
                2 => "U2",
                3 => "U3",
                5 => "RxDetect",
                6 => "Inactive",
                7 => "Polling",
                _ => "?",
            };
            println!(" PLS={}", pls_str);
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
        if port == 0 || port > self.num_ports {
            return false;
        }

        let portsc_off = self.port_base + ((port - 1) as usize * 0x10) + xhci_port::PORTSC;

        // Read current PORTSC
        let portsc = self.mac.read32(portsc_off);
        let ccs = portsc & 1;
        let pp = (portsc >> 9) & 1;

        print!("  Port {}: ", port);

        // Skip if not powered
        if pp == 0 {
            println!("not powered, skipping");
            return false;
        }

        // Skip if no device connected
        if ccs == 0 {
            println!("no device connected, skipping reset");
            return false;
        }

        println!("device present, resetting...");

        // Set PR (Port Reset) bit 4, preserve PP bit 9
        // Write 1 to clear status change bits (17-23) first
        let write_val = (portsc & 0x0E00C3E0) | (1 << 4);  // Keep PP, PLS write strobe, set PR
        self.mac.write32(portsc_off, write_val);

        // Wait for reset to complete
        for _ in 0..100 {
            delay(10000);
            let portsc = self.mac.read32(portsc_off);
            let pr = (portsc >> 4) & 1;
            let prc = (portsc >> 21) & 1;

            if pr == 0 && prc == 1 {
                // Reset complete, clear PRC
                self.mac.write32(portsc_off, portsc | (1 << 21));
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

        // Drain any pending events first
        // (moved outside borrow)

        let mut trb = Trb::new();
        trb.set_type(trb_type::ENABLE_SLOT);
        cmd_ring.enqueue(&trb);
        self.ring_doorbell(0, 0);

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
                        let cc = (evt.status >> 24) & 0xFF;
                        let slot_id = (evt.control >> 24) & 0xFF;
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

        // Get port speed from PORTSC
        let portsc_off = self.port_base + ((port - 1) as usize * 0x10);
        let portsc = self.mac.read32(portsc_off);
        let speed = (portsc >> 10) & 0xF;

        let speed_str = match speed {
            1 => "Full",
            2 => "Low",
            3 => "High",
            4 => "Super",
            5 => "Super+",
            _ => "Unknown",
        };
        println!("  Port {} speed: {} ({})", port, speed, speed_str);

        // Max packet size for EP0 based on speed
        let max_packet_size = match speed {
            1 => 8,    // Full speed
            2 => 8,    // Low speed
            3 => 64,   // High speed
            4 => 512,  // Super speed
            5 => 512,  // Super speed+
            _ => 8,
        };

        // Step 3: Initialize Input Context
        unsafe {
            let input = &mut *input_ctx_virt;
            *input = InputContext::new();

            // Add Slot Context and EP0 Context (bits 0 and 1)
            input.control.add_flags = 0x3;  // A0=1, A1=1

            // Slot Context
            input.slot.set_route_string(0);
            input.slot.set_speed(speed);
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
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            hub::RT_HUB_GET,
            hub::GET_DESCRIPTOR,
            (usb_req::DESC_SS_HUB << 8) as u16,  // wValue: descriptor type << 8
            0,
            buf_phys,
            12  // SS Hub descriptor is 12 bytes
        ) {
            Some((cc, len)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
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
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            hub::RT_PORT_GET,
            hub::GET_STATUS,
            0,
            port,
            buf_phys,
            4
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
                let ps = unsafe { *(buf_virt as *const PortStatus) };
                Some(ps)
            }
            _ => None
        }
    }

    /// Set a port feature
    fn hub_set_port_feature(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, port: u16, feature: u16) -> bool {
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            hub::RT_PORT_SET,
            hub::SET_FEATURE,
            feature,
            port,
            0, 0
        ) {
            Some((cc, _)) if cc == trb_cc::SUCCESS => true,
            _ => false
        }
    }

    /// Clear a port feature
    fn hub_clear_port_feature(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, port: u16, feature: u16) -> bool {
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue,
            hub::RT_PORT_SET,
            hub::CLEAR_FEATURE,
            feature,
            port,
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

        // Now run SCSI commands
        println!();
        println!("=== Running SCSI Commands ===");

        // Allocate data buffer for SCSI commands
        let mut data_phys: u64 = 0;
        let data_virt = syscall::mmap_dma(4096, &mut data_phys);
        if data_virt < 0 {
            println!("  Failed to allocate data buffer");
            return;
        }

        // SCSI INQUIRY (old code - uses TEST_UNIT_READY)
        self.scsi_inquiry(
            slot_id, bulk_in_dci, bulk_out_dci,
            bulk_in_ring_virt as *mut Trb, bulk_in_ring_phys,
            bulk_out_ring_virt as *mut Trb, bulk_out_ring_phys,
            data_virt as *mut u8, data_phys,
            device_ctx_virt
        );

        // Run new refactored SCSI tests (READ_CAPACITY, READ_10)
        // Need a fresh data buffer since the old code used offsets differently
        let mut test_data_phys: u64 = 0;
        let test_data_virt = syscall::mmap_dma(4096, &mut test_data_phys);
        if test_data_virt >= 0 {
            let mut ctx = BulkContext::new(
                slot_id,
                bulk_in_dci,
                bulk_out_dci,
                bulk_in_ring_virt as *mut Trb,
                bulk_in_ring_phys,
                bulk_out_ring_virt as *mut Trb,
                bulk_out_ring_phys,
                test_data_virt as *mut u8,
                test_data_phys,
                device_ctx_virt,
            );
            self.run_scsi_tests(&mut ctx);
        }
    }

    /// Perform SCSI TEST_UNIT_READY command (helps wake up device)
    fn scsi_test_unit_ready(
        &mut self,
        slot_id: u32,
        bulk_in_dci: u32,
        bulk_out_dci: u32,
        bulk_in_ring_virt: *mut Trb,
        bulk_out_ring_virt: *mut Trb,
        data_virt: *mut u8,
        data_phys: u64,
        out_enqueue: &mut usize,
        in_enqueue: &mut usize,
    ) -> bool {
        println!("  SCSI TEST_UNIT_READY...");

        // TEST_UNIT_READY: no data transfer, just CBW + CSW
        let tur_cmd = [scsi::TEST_UNIT_READY, 0, 0, 0, 0, 0];
        let cbw = Cbw::new(0x100, 0, false, 0, &tur_cmd);

        // Write CBW
        unsafe {
            let cbw_ptr = data_virt as *mut Cbw;
            core::ptr::write_volatile(cbw_ptr, cbw);

            // Flush CBW data to main memory for non-coherent DMA
            let cbw_addr = cbw_ptr as u64;
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!(
                "dc cvac, {addr}",
                addr = in(reg) cbw_addr,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Send CBW
        unsafe {
            let trb = &mut *bulk_out_ring_virt.add(*out_enqueue);
            trb.param = data_phys;
            trb.status = 31;
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | 1;

            // Debug: print TRB details
            print!("    TRB[{}] param=", *out_enqueue);
            print_hex64(trb.param);
            print!(", status=");
            print_hex32(trb.status);
            print!(", ctrl=");
            print_hex32(trb.control);
            println!();

            *out_enqueue += 1;

            // Flush TRB to main memory for non-coherent DMA
            let trb_addr = trb as *const _ as u64;
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!(
                "dc cvac, {addr}",
                addr = in(reg) trb_addr,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Debug: print doorbell info and check USBSTS before ringing
        let mac_base = self.mac.base;
        let rt_base = self.rt_base;
        let op_base = self.op_base;

        unsafe {
            let usbsts = core::ptr::read_volatile(
                (mac_base + op_base as u64 + xhci_op::USBSTS as u64) as *const u32
            );
            print!("    USBSTS before doorbell: ");
            print_hex32(usbsts);
            println!();
        }

        let db_addr = self.mac.base + self.db_base as u64 + (slot_id as u64 * 4);
        print!("    Doorbell: slot={}, target={}, addr=", slot_id, bulk_out_dci);
        print_hex64(db_addr);
        println!();

        self.ring_doorbell(slot_id, bulk_out_dci);

        // Small delay then check USBSTS again
        delay(10000);
        unsafe {
            let usbsts = core::ptr::read_volatile(
                (mac_base + op_base as u64 + xhci_op::USBSTS as u64) as *const u32
            );
            print!("    USBSTS after doorbell: ");
            print_hex32(usbsts);
            println!();
        }

        delay(40000);

        // Poll for CBW completion - print ALL events we see
        let mut cbw_ok = false;
        if let Some(ref mut event_ring) = self.event_ring {
            for i in 0..100 {
                if let Some(evt) = event_ring.dequeue() {
                    let evt_type = evt.get_type();
                    let cc = (evt.status >> 24) & 0xFF;
                    let evt_slot = (evt.control >> 24) & 0xFF;
                    let evt_ep = (evt.control >> 16) & 0x1F;

                    print!("    Event[{}]: type={}, CC={}, slot={}, ep={}", i, evt_type, cc, evt_slot, evt_ep);
                    println!();

                    if evt_type == trb_type::TRANSFER_EVENT {
                        if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {
                            cbw_ok = true;
                        }
                        let erdp = event_ring.erdp() | (1 << 3);
                        unsafe {
                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
                            core::ptr::write_volatile(ptr, erdp);
                        }
                        break;
                    } else if evt_type == trb_type::PORT_STATUS_CHANGE {
                        // Acknowledge port status change
                        let erdp = event_ring.erdp() | (1 << 3);
                        unsafe {
                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
                            core::ptr::write_volatile(ptr, erdp);
                        }
                        // Continue looking for transfer event
                    } else {
                        // Unknown event, acknowledge it
                        let erdp = event_ring.erdp() | (1 << 3);
                        unsafe {
                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
                            core::ptr::write_volatile(ptr, erdp);
                        }
                    }
                }
                delay(1000);
            }
        }

        if !cbw_ok {
            println!("    TUR CBW failed - dumping debug info...");

            // Check endpoint context via DCBAA
            unsafe {
                let dcbaa = self.xhci_mem as *mut u64;
                let dev_ctx_phys = *dcbaa.add(slot_id as usize);
                print!("      DCBAA[{}] = ", slot_id);
                print_hex64(dev_ctx_phys);
                println!();

                // Note: We can't directly access physical memory from userspace
                // But we can check IMAN and ERDP to see event ring state
                let iman = core::ptr::read_volatile(
                    (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *const u32
                );
                print!("      IMAN = ");
                print_hex32(iman);
                println!();

                let erdp_val = core::ptr::read_volatile(
                    (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *const u64
                );
                print!("      ERDP = ");
                print_hex64(erdp_val);
                println!();

                // Check CRCR (Command Ring Control)
                let crcr = core::ptr::read_volatile(
                    (mac_base + op_base as u64 + xhci_op::CRCR as u64) as *const u64
                );
                print!("      CRCR = ");
                print_hex64(crcr);
                println!();

                // Check USBSTS
                let usbsts = core::ptr::read_volatile(
                    (mac_base + op_base as u64 + xhci_op::USBSTS as u64) as *const u32
                );
                print!("      USBSTS = ");
                print_hex32(usbsts);
                if (usbsts & 1) != 0 { print!(" HCH"); }
                if (usbsts & 4) != 0 { print!(" HSE"); }
                if (usbsts & 8) != 0 { print!(" EINT"); }
                if (usbsts & 0x10) != 0 { print!(" PCD"); }
                if (usbsts & 0x1000) != 0 { print!(" CNR"); }
                println!();
            }

            // Dump any remaining events
            if let Some(ref mut event_ring) = self.event_ring {
                for j in 0..10 {
                    if let Some(evt) = event_ring.dequeue() {
                        let evt_type = evt.get_type();
                        let cc = (evt.status >> 24) & 0xFF;
                        let evt_slot = (evt.control >> 24) & 0xFF;
                        print!("      Extra event[{}]: type={}, slot={}, CC={}", j, evt_type, evt_slot, cc);
                        println!();
                        let erdp = event_ring.erdp() | (1 << 3);
                        unsafe {
                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
                            core::ptr::write_volatile(ptr, erdp);
                        }
                    }
                }
            }

            return false;
        }

        // Receive CSW (no data stage for TEST_UNIT_READY)
        println!("    Setting up CSW receive on bulk IN (DCI={})...", bulk_in_dci);
        let csw_phys = data_phys + 64;
        unsafe {
            let trb = &mut *bulk_in_ring_virt.add(*in_enqueue);
            trb.param = csw_phys;
            trb.status = 13;  // CSW is 13 bytes
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | 1;

            print!("    CSW TRB[{}] param=", *in_enqueue);
            print_hex64(trb.param);
            print!(", status=");
            print_hex32(trb.status);
            print!(", ctrl=");
            print_hex32(trb.control);
            println!();

            *in_enqueue += 1;
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        self.ring_doorbell(slot_id, bulk_in_dci);

        // Wait for device to respond - use yields instead of tight delay
        for _ in 0..10 {
            syscall::yield_now();
        }

        let mut csw_ok = false;
        if let Some(ref mut event_ring) = self.event_ring {
            // Increase iteration count and check all event types
            for i in 0..200 {
                if let Some(evt) = event_ring.dequeue() {
                    let evt_type = evt.get_type();
                    let cc = (evt.status >> 24) & 0xFF;
                    let evt_slot = (evt.control >> 24) & 0xFF;
                    let evt_ep = (evt.control >> 16) & 0x1F;

                    print!("    CSW Event[{}]: type={}, CC={}, slot={}, ep={}", i, evt_type, cc, evt_slot, evt_ep);
                    println!();

                    if evt_type == trb_type::TRANSFER_EVENT {
                        if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {
                            csw_ok = true;
                        } else {
                            print!("    TUR CSW failed: CC=");
                            print_hex32(cc);
                            println!();
                        }
                        let erdp = event_ring.erdp() | (1 << 3);
                        unsafe {
                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
                            core::ptr::write_volatile(ptr, erdp);
                        }
                        break;
                    } else {
                        // Not a transfer event - acknowledge and continue
                        let erdp = event_ring.erdp() | (1 << 3);
                        unsafe {
                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
                            core::ptr::write_volatile(ptr, erdp);
                        }
                    }
                }
                // Yield to scheduler instead of tight polling
                syscall::yield_now();
            }
        }

        if !csw_ok {
            println!("    CSW not received - dumping event ring state...");

            // Dump event ring state
            if let Some(ref event_ring) = self.event_ring {
                println!("      Event ring dequeue index: {}", event_ring.dequeue);
                println!("      Event ring cycle: {}", event_ring.cycle);
                print!("      Event ring phys: ");
                print_hex64(event_ring.trbs_phys);
                println!();

                // Directly read the next few TRBs to see what's there
                unsafe {
                    for i in 0..4 {
                        let idx = (event_ring.dequeue + i) % RING_SIZE;
                        let trb_ptr = event_ring.trbs.add(idx);
                        let ctrl = core::ptr::read_volatile(&(*trb_ptr).control);
                        let param = core::ptr::read_volatile(&(*trb_ptr).param);
                        let status = core::ptr::read_volatile(&(*trb_ptr).status);
                        let trb_cycle = ctrl & 1;
                        let trb_type = (ctrl >> 10) & 0x3F;

                        print!("      TRB[{}]: type={}, cycle={}, ctrl=", idx, trb_type, trb_cycle);
                        print_hex32(ctrl);
                        if trb_type != 0 {
                            print!(", param=");
                            print_hex64(param);
                            print!(", CC={}", (status >> 24) & 0xFF);
                        }
                        println!();
                    }
                }
            }

            unsafe {
                let usbsts = core::ptr::read_volatile(
                    (mac_base + op_base as u64 + xhci_op::USBSTS as u64) as *const u32
                );
                print!("      USBSTS = ");
                print_hex32(usbsts);
                println!();
            }
        }

        if csw_ok {
            // Check CSW
            let csw = unsafe { &*(data_virt.add(64) as *const Csw) };
            print!("    CSW signature: ");
            print_hex32(csw.signature);
            println!();
            if csw.signature == msc::CSW_SIGNATURE {
                println!("    TEST_UNIT_READY: status={}", csw.status);
                return csw.status == msc::CSW_STATUS_PASSED;
            } else {
                println!("    Invalid CSW signature!");
            }
        }

        println!("    TUR timeout/failed");
        false
    }

    // =========================================================================
    // Refactored SCSI command helpers
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
        let mac_base = self.mac.base;
        let rt_base = self.rt_base;
        let op_base = self.op_base;

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
        // READ(10) CDB format:
        // [0] = 0x28 (READ_10 opcode)
        // [1] = flags (0)
        // [2-5] = LBA (big-endian)
        // [6] = group number (0)
        // [7-8] = transfer length in blocks (big-endian)
        // [9] = control (0)
        let cmd = [
            scsi::READ_10,
            0,
            ((lba >> 24) & 0xFF) as u8,
            ((lba >> 16) & 0xFF) as u8,
            ((lba >> 8) & 0xFF) as u8,
            (lba & 0xFF) as u8,
            0,
            ((count >> 8) & 0xFF) as u8,
            (count & 0xFF) as u8,
            0,
        ];

        // Assume 512 bytes per block
        let data_length = (count as u32) * 512;
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
        // READ_CAPACITY(10) CDB: just the opcode, rest is zeros
        let cmd = [scsi::READ_CAPACITY_10, 0, 0, 0, 0, 0, 0, 0, 0, 0];

        println!("  SCSI READ_CAPACITY(10)...");

        let (result, csw) = self.scsi_command_in(ctx, &cmd, 8);

        if let Some(csw) = csw {
            if csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED {
                // Parse response (8 bytes, big-endian)
                unsafe {
                    let resp_ptr = ctx.data_buf.add(DATA_OFFSET);
                    ctx.invalidate_buffer(resp_ptr as u64, 8);

                    let last_lba = u32::from_be_bytes([
                        *resp_ptr.add(0), *resp_ptr.add(1),
                        *resp_ptr.add(2), *resp_ptr.add(3)
                    ]);
                    let block_size = u32::from_be_bytes([
                        *resp_ptr.add(4), *resp_ptr.add(5),
                        *resp_ptr.add(6), *resp_ptr.add(7)
                    ]);

                    println!("    Last LBA: {}", last_lba);
                    println!("    Block size: {} bytes", block_size);
                    println!("    Capacity: {} MB", ((last_lba as u64 + 1) * block_size as u64) / (1024 * 1024));

                    return Some((last_lba, block_size));
                }
            }
        }

        println!("    READ_CAPACITY failed: {:?}", result);
        None
    }

    /// Run SCSI tests using the new refactored code
    fn run_scsi_tests(&mut self, ctx: &mut BulkContext) {
        println!();
        println!("=== Running SCSI Tests (refactored) ===");

        // Read capacity first to know device size
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

    /// Perform SCSI INQUIRY command
    fn scsi_inquiry(
        &mut self,
        slot_id: u32,
        bulk_in_dci: u32,
        bulk_out_dci: u32,
        bulk_in_ring_virt: *mut Trb,
        bulk_in_ring_phys: u64,
        bulk_out_ring_virt: *mut Trb,
        _bulk_out_ring_phys: u64,
        data_virt: *mut u8,
        data_phys: u64,
        device_ctx_virt: *mut DeviceContext,
    ) {
        // Track ring positions
        let mut out_enqueue = 0usize;
        let mut in_enqueue = 0usize;

        // Cache register bases
        let mac_base = self.mac.base;
        let rt_base = self.rt_base;
        let op_base = self.op_base;

        // Clear any pending interrupts before starting
        unsafe {
            // Clear IMAN.IP by writing 1 to it
            let iman_addr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *mut u32;
            let iman = core::ptr::read_volatile(iman_addr);
            if (iman & 1) != 0 {
                println!("  Clearing pending interrupt (IMAN.IP)...");
                core::ptr::write_volatile(iman_addr, iman | 1);  // Write 1 to clear IP
            }

            // Clear PCD by writing 1 to it in USBSTS
            let usbsts_addr = (mac_base + op_base as u64 + xhci_op::USBSTS as u64) as *mut u32;
            let usbsts = core::ptr::read_volatile(usbsts_addr);
            if (usbsts & 0x10) != 0 {
                println!("  Clearing Port Change Detect...");
                core::ptr::write_volatile(usbsts_addr, 0x10);  // Write 1 to clear PCD
            }
        }

        // Give device time to be fully ready after SET_CONFIGURATION
        // USB mass storage devices can take several seconds to be ready
        println!("  Waiting for device to initialize...");
        for i in 0..100 {
            // Yield to scheduler - each yield may be ~10ms due to timer slice
            syscall::yield_now();
            // Print progress every 10 yields
            if i > 0 && i % 10 == 0 {
                print!(".");
            }
        }
        println!(" done");

        // Try TEST_UNIT_READY first - no data phase, just CBW + CSW
        println!("  SCSI TEST_UNIT_READY (no data phase)...");

        // Build TUR command (6 bytes, no data transfer)
        let tur_cmd = [scsi::TEST_UNIT_READY, 0, 0, 0, 0, 0];
        let cbw = Cbw::new(1, 0, false, 0, &tur_cmd);  // direction_in=false, data_length=0

        // Write CBW to data buffer
        let cbw_offset = 256usize;
        let csw_offset = cbw_offset + 64;  // CSW buffer after CBW
        let csw_phys = data_phys + csw_offset as u64;

        unsafe {
            let cbw_ptr = data_virt.add(cbw_offset) as *mut Cbw;
            core::ptr::write_volatile(cbw_ptr, cbw);

            // Flush CBW data to main memory for non-coherent DMA
            let cbw_addr = cbw_ptr as u64;
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!(
                "dc cvac, {addr}",
                addr = in(reg) cbw_addr,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // For TUR: No data phase, just CBW -> CSW
        // APPROACH: Send CBW first, wait for completion, then set up and ring bulk IN

        // Step 1: Send CBW on bulk OUT
        println!("    Step 1: Sending CBW on bulk OUT...");
        unsafe {
            let trb = &mut *bulk_out_ring_virt.add(out_enqueue);
            trb.param = data_phys + cbw_offset as u64;
            trb.status = 31;
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | 1;  // IOC, cycle
            out_enqueue += 1;

            // Flush TRB to main memory for non-coherent DMA
            let trb_addr = trb as *const _ as u64;
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!(
                "dc cvac, {addr}",
                addr = in(reg) trb_addr,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));

            print!("      CBW TRB: param=");
            print_hex64(trb.param);
            print!(", len=31, ctrl=");
            print_hex32(trb.control);
            println!();
        }

        self.ring_doorbell(slot_id, bulk_out_dci);

        // Poll for CBW completion with yields
        let mut cbw_sent = false;
        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..100 {
                if let Some(evt) = event_ring.dequeue() {
                    if evt.get_type() == trb_type::TRANSFER_EVENT {
                        let cc = (evt.status >> 24) & 0xFF;
                        let slot = (evt.control >> 24) & 0xFF;
                        print!("    Event: slot={}, CC={}", slot, cc);
                        println!();
                        if cc == trb_cc::SUCCESS {
                            println!("    CBW sent successfully");
                            cbw_sent = true;
                        }
                        let erdp = event_ring.erdp() | (1 << 3);
                        unsafe {
                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
                            core::ptr::write_volatile(ptr, erdp);
                        }
                        break;
                    }
                }
                syscall::yield_now();
            }
        }

        if !cbw_sent {
            println!("    CBW send timeout");
            return;
        }

        // Step 2: Now set up CSW receive on bulk IN
        println!("    Step 2: Setting up CSW receive on bulk IN (13 bytes)...");
        print!("      CSW buffer: ");
        print_hex64(csw_phys);
        println!();

        // U-Boot pattern: Invalidate CSW destination buffer BEFORE starting IN transfer
        // This ensures no dirty cache lines that could corrupt DMA data
        unsafe {
            let csw_virt = data_virt.add(csw_offset);
            // CSW is 13 bytes but invalidate full cache line (64 bytes)
            core::arch::asm!(
                "dc civac, {addr}",
                addr = in(reg) csw_virt as u64,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }

        // Get current EP DCS from endpoint context (it may have been updated)
        // CRITICAL: Invalidate cache first since xHCI writes to output context
        let (current_dcs, raw_dw2) = unsafe {
            let dev_ctx = &*device_ctx_virt;
            let bulk_in_idx = (bulk_in_dci - 1) as usize;
            let bulk_in_ctx = &dev_ctx.endpoints[bulk_in_idx];

            // Invalidate the endpoint context cache line before reading
            let ctx_addr = bulk_in_ctx as *const _ as u64;
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!(
                "dc civac, {addr}",
                addr = in(reg) ctx_addr,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!("isb", options(nostack, preserves_flags));

            // Now read with volatile
            let dw2 = core::ptr::read_volatile(&bulk_in_ctx.dw2);
            ((dw2 & 1) as u32, dw2)
        };
        print!("      Current bulk IN EP DCS: {}, raw dw2: ", current_dcs);
        print_hex32(raw_dw2);
        println!();

        unsafe {
            let trb = &mut *bulk_in_ring_virt.add(in_enqueue);
            let trb_addr = trb as *const _ as u64;

            // Write TRB multiple times with explicit memory barriers
            // This is a brute-force approach to force data to main memory
            for write_attempt in 0..3 {
                // Use volatile write to ensure compiler doesn't optimize away
                core::ptr::write_volatile(&mut trb.param, csw_phys);
                core::ptr::write_volatile(&mut trb.status, 13);
                core::ptr::write_volatile(&mut trb.control, (trb_type::NORMAL << 10) | (1 << 5) | current_dcs);

                // Full barrier
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));

                // Clean to Point of Persistence (DC CVAP) if available, else DC CVAC
                // DC CVAP ensures data reaches persistent storage point
                core::arch::asm!(
                    "dc cvac, {addr}",
                    addr = in(reg) trb_addr,
                    options(nostack, preserves_flags)
                );

                core::arch::asm!("dsb sy", options(nostack, preserves_flags));

                // Small delay between attempts
                if write_attempt < 2 {
                    for _ in 0..5000 { core::arch::asm!("nop", options(nostack)); }
                }
            }

            in_enqueue += 1;

            // Final barrier
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!("isb", options(nostack, preserves_flags));

            print!("      CSW TRB: param=");
            print_hex64(trb.param);
            print!(", len=13, ctrl=");
            print_hex32(trb.control);
            println!();

            // Verify TRB was written by reading it back (after invalidating cache)
            core::arch::asm!(
                "dc civac, {addr}",
                addr = in(reg) trb_addr,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            let verify_param = core::ptr::read_volatile(&trb.param);
            let verify_ctrl = core::ptr::read_volatile(&trb.control);
            print!("      TRB verify after flush: param=");
            print_hex64(verify_param);
            print!(", ctrl=");
            print_hex32(verify_ctrl);
            println!();
        }

        // Ring bulk IN doorbell
        println!("    Ringing bulk IN doorbell (DCI={})...", bulk_in_dci);
        self.ring_doorbell(slot_id, bulk_in_dci);

        // Yield a few times then check if xHCI started processing
        for _ in 0..3 {
            syscall::yield_now();
        }

        // Re-check endpoint context to see if xHCI even looked at our TRB
        unsafe {
            let dev_ctx = &*device_ctx_virt;
            let bulk_in_idx = (bulk_in_dci - 1) as usize;
            let bulk_in_ctx = &dev_ctx.endpoints[bulk_in_idx];
            let ctx_addr = bulk_in_ctx as *const _ as u64;
            core::arch::asm!("dc civac, {}", in(reg) ctx_addr, options(nostack));
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            let ep_dw0 = core::ptr::read_volatile(&bulk_in_ctx.dw0);
            let ep_dw2 = core::ptr::read_volatile(&bulk_in_ctx.dw2);
            let ep_state = ep_dw0 & 0x7;
            let ep_deq = ep_dw2 & !0xF;
            let ep_dcs = ep_dw2 & 1;
            print!("      After doorbell: EP state={}, DCS={}, deq_idx={}", ep_state, ep_dcs, (ep_deq as usize).saturating_sub(bulk_in_ring_phys as usize) / 16);
            println!();
        }

        // Also verify our TRB is still in memory
        unsafe {
            let trb_check = &*bulk_in_ring_virt.add(0);  // Index 0 is where we put our TRB
            let trb_addr = trb_check as *const _ as u64;
            core::arch::asm!("dc civac, {}", in(reg) trb_addr, options(nostack));
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            let param = core::ptr::read_volatile(&trb_check.param);
            let ctrl = core::ptr::read_volatile(&trb_check.control);
            print!("      Bulk IN TRB[0] in memory: param=");
            print_hex64(param);
            print!(", ctrl=");
            print_hex32(ctrl);
            println!();
        }

        // Wait for CSW using IRQ-based waiting
        println!("    Step 3: Waiting for CSW (IRQ-based)...");

        // Get timer frequency for timing measurements
        let timer_freq: u64;
        unsafe { core::arch::asm!("mrs {}, cntfrq_el0", out(reg) timer_freq); }

        // Record start time (right after doorbell was rung)
        let start_time: u64;
        unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) start_time); }

        // Clear IMAN.IP before waiting - so we get a fresh interrupt
        unsafe {
            let iman_addr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *mut u32;
            let iman = core::ptr::read_volatile(iman_addr);
            print!("      Initial IMAN: ");
            print_hex32(iman);
            if (iman & 1) != 0 {
                print!(" (clearing IP)");
                core::ptr::write_volatile(iman_addr, iman | 1);  // Write 1 to clear IP
            }
            println!();

            // Clear EINT in USBSTS
            let usbsts_addr = (mac_base + op_base as u64 + xhci_op::USBSTS as u64) as *mut u32;
            let usbsts = core::ptr::read_volatile(usbsts_addr);
            if (usbsts & 0x8) != 0 {
                core::ptr::write_volatile(usbsts_addr, 0x8);
            }
        }

        // Use IRQ-based waiting - blocks until xHCI interrupt fires
        // Loop until we get the CSW event (may need multiple IRQs if earlier events pending)
        println!("    Waiting for IRQ...");

        let mut csw_received = false;
        let max_irq_waits = 10;  // Prevent infinite loop

        for irq_attempt in 0..max_irq_waits {
            let irq_received = self.wait_for_irq(5000);

            if !irq_received {
                println!("    IRQ wait failed (fd={})!", self.irq_fd);
                break;
            }

            if irq_attempt > 0 {
                println!("    IRQ #{} received, checking event ring...", irq_attempt + 1);
            } else {
                println!("    IRQ received, checking event ring...");
            }

            // Check event ring for CSW completion
            if let Some(ref mut event_ring) = self.event_ring {
                if let Some(evt) = event_ring.dequeue() {
                    let evt_type = evt.get_type();
                    let cc = (evt.status >> 24) & 0xFF;

                    if evt_type == trb_type::TRANSFER_EVENT {
                        if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {
                            csw_received = true;
                            // Calculate elapsed time
                            let end_time: u64;
                            unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) end_time); }
                            let elapsed_ticks = end_time - start_time;
                            let elapsed_us = (elapsed_ticks * 1_000_000) / timer_freq;
                            println!("      CSW received via IRQ in {} us ({} ticks)", elapsed_us, elapsed_ticks);
                        } else {
                            print!("      Transfer failed CC=");
                            print_hex32(cc);
                            println!();
                        }
                        // Update ERDP
                        let erdp = event_ring.erdp() | (1 << 3);
                        unsafe {
                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
                            core::ptr::write_volatile(ptr, erdp);
                        }
                        break;
                    } else {
                        // Got some other event type, update ERDP and wait for more
                        println!("      Got event type {} (not transfer), waiting for more...", evt_type);
                        let erdp = event_ring.erdp() | (1 << 3);
                        unsafe {
                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
                            core::ptr::write_volatile(ptr, erdp);
                        }
                    }
                } else {
                    // No event in ring yet - the IRQ might have been for something else
                    // or the event hasn't been written yet. Wait for another IRQ.
                    println!("      No event in ring, waiting for another IRQ...");
                }
            }
        }

        if !csw_received {
            println!("      CSW not received after {} IRQ attempts", max_irq_waits);

            // Basic diagnostics
            unsafe {
                let iman = core::ptr::read_volatile(
                    (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *const u32
                );
                print!("      IMAN=");
                print_hex32(iman);
                println!(" (IP={}, IE={})", iman & 1, (iman >> 1) & 1);

                // Check EP state
                let dev_ctx = &*device_ctx_virt;
                let bulk_in_idx = (bulk_in_dci - 1) as usize;
                let bulk_in_ctx = &dev_ctx.endpoints[bulk_in_idx];
                let ctx_addr = bulk_in_ctx as *const _ as u64;
                core::arch::asm!("dc civac, {}", in(reg) ctx_addr, options(nostack));
                core::arch::asm!("dsb sy", options(nostack, preserves_flags));
                let ep_dw0 = core::ptr::read_volatile(&bulk_in_ctx.dw0);
                let ep_state = ep_dw0 & 0x7;
                println!("      Bulk IN EP state={} (1=Running, 2=Halted)", ep_state);
            }
            return;
        }

        // =========================================================================
        // OLD POLLING-BASED APPROACH (kept for reference)
        // =========================================================================
        // // Poll for CSW completion
        // // Note: Interrupt-based waiting doesn't work - xHCI doesn't complete transfer when CPU is idle.
        // // Active polling keeps the bus/memory subsystem active which seems necessary.
        // println!("    Waiting for CSW...");
        //
        // // Initial wait before polling - give device time to process command
        // // Check IMAN before/after to see when interrupt fires
        // unsafe {
        //     let iman = core::ptr::read_volatile(
        //         (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *const u32
        //     );
        //     println!("    IMAN before delay: IP={}, IE={}", iman & 1, (iman >> 1) & 1);
        // }
        // println!("    Waiting 2 seconds before first check...");
        // delay_ms(2000);
        // unsafe {
        //     let iman = core::ptr::read_volatile(
        //         (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *const u32
        //     );
        //     println!("    IMAN after delay: IP={}, IE={}", iman & 1, (iman >> 1) & 1);
        // }
        // println!("    Wait complete, now checking for event...");
        //
        // let mut csw_received = false;
        // let mut poll_iterations = 0u32;
        //
        // // Get start timestamp
        // let start_ts: u64;
        // unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) start_ts); }
        // println!("      Start timestamp: {}", start_ts);
        //
        // if let Some(ref mut event_ring) = self.event_ring {
        //     for i in 0..5000 {
        //         poll_iterations = i;
        //
        //         // Progress every 500 iterations with timestamp
        //         if i > 0 && i % 500 == 0 {
        //             let ts: u64;
        //             unsafe { core::arch::asm!("mrs {}, cntpct_el0", out(reg) ts); }
        //             println!("      Still waiting... iteration {} (ts={})", i, ts);
        //         }
        //
        //         // Check for event with cache invalidation
        //         if let Some(evt) = event_ring.dequeue() {
        //             let evt_type = evt.get_type();
        //             let cc = (evt.status >> 24) & 0xFF;
        //
        //             if evt_type == trb_type::TRANSFER_EVENT {
        //                 if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {
        //                     csw_received = true;
        //                     println!("      CSW received at iteration {}", i);
        //                 } else {
        //                     print!("      Transfer failed CC=");
        //                     print_hex32(cc);
        //                     println!();
        //                 }
        //                 // Update ERDP
        //                 let erdp = event_ring.erdp() | (1 << 3);
        //                 unsafe {
        //                     let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
        //                     core::ptr::write_volatile(ptr, erdp);
        //                 }
        //                 break;
        //             }
        //         }
        //
        //         // 1ms delay between polls using hardware timer
        //         delay_ms(1);
        //     }
        // }
        //
        // if !csw_received {
        //     println!("      CSW timeout after {} attempts", poll_iterations);
        //
        //     // Basic diagnostics
        //     unsafe {
        //         let iman = core::ptr::read_volatile(
        //             (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *const u32
        //         );
        //         print!("      IMAN=");
        //         print_hex32(iman);
        //         println!(" (IP={}, IE={})", iman & 1, (iman >> 1) & 1);
        //
        //         // Check EP state
        //         let dev_ctx = &*device_ctx_virt;
        //         let bulk_in_idx = (bulk_in_dci - 1) as usize;
        //         let bulk_in_ctx = &dev_ctx.endpoints[bulk_in_idx];
        //         let ctx_addr = bulk_in_ctx as *const _ as u64;
        //         core::arch::asm!("dc civac, {}", in(reg) ctx_addr, options(nostack));
        //         core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        //         let ep_dw0 = core::ptr::read_volatile(&bulk_in_ctx.dw0);
        //         let ep_state = ep_dw0 & 0x7;
        //         println!("      Bulk IN EP state={} (1=Running, 2=Halted)", ep_state);
        //     }
        //
        //     // Try one more dequeue with fresh cache invalidation
        //     if let Some(ref mut event_ring) = self.event_ring {
        //         if let Some(evt) = event_ring.dequeue() {
        //             let evt_type = evt.get_type();
        //             let cc = (evt.status >> 24) & 0xFF;
        //             println!("      Late event found! type={}, CC={}", evt_type, cc);
        //             if evt_type == trb_type::TRANSFER_EVENT && (cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET) {
        //                 csw_received = true;
        //                 let erdp = event_ring.erdp() | (1 << 3);
        //                 unsafe {
        //                     let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;
        //                     core::ptr::write_volatile(ptr, erdp);
        //                 }
        //             }
        //         }
        //
        //     }
        //     return;
        // }
        // =========================================================================

        // CSW received successfully - read the CSW data
        unsafe {
            let csw_ptr = data_virt.add(csw_offset) as *const Csw;
            // U-Boot pattern: invalidate cache before reading DMA data
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            core::arch::asm!(
                "dc civac, {addr}",
                addr = in(reg) csw_ptr,
                options(nostack, preserves_flags)
            );
            core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));

            let csw = core::ptr::read_volatile(csw_ptr);
            let sig = csw.signature;
            let tag = csw.tag;
            let status = csw.status;
            print!("      CSW: sig=");
            print_hex32(sig);
            print!(", tag={}, status={}", tag, status);
            println!();
        }

        // TUR succeeded!
        println!("    TEST_UNIT_READY complete - bulk IN works!");
        println!("    Device is ready for SCSI commands");
    }
}

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
                    for p in 0..d.num_ports {
                        let portsc_off = d.port_base + (p as usize * 0x10);
                        let portsc = d.mac.read32(portsc_off);

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
    for p in 1..=driver.num_ports {
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
        println!("  IRQ {} registered (fd={})", SSUSB1_IRQ, irq_fd);
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
    let ssusb0_mac = MmioRegion::open(SSUSB0_MAC_PHYS, SSUSB_MAC_SIZE);

    // Poll ports on BOTH controllers
    println!();
    println!("=== Port Monitor (30 seconds) ===");
    println!("  Monitoring SSUSB0 and SSUSB1 - try plugging device into different ports");

    for round in 0..30 {
        let mut found = false;

        // Check SSUSB0 ports
        if let Some(ref mac0) = ssusb0_mac {
            let cap0 = mac0.read32(SSUSB_IPPC_OFFSET + 0x24);  // IP_XHCI_CAP
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
        for p in 0..driver.num_ports {
            let portsc_off = driver.port_base + (p as usize * 0x10);
            let portsc = driver.mac.read32(portsc_off);
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
