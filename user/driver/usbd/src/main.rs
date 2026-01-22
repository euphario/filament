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
#![allow(dead_code)]  // Many functions reserved for future use or debugging

mod device;
mod bot_state;

use device::{is_msc_bbb, is_hub_class};
pub use bot_state::{BotStateMachine, DeviceState, TransferPhase, RecoveryState, RecoveryAction, CswStatus, TransportResult};
use userlib::{println, syscall};
use userlib::{uinfo, uwarn, uerror};
use userlib::syscall::O_NONBLOCK;
use userlib::syscall::{
    Handle, WaitFilter, WaitRequest, WaitResult,
    handle_timer_create, handle_timer_set, handle_wait,
    handle_wrap_channel,
};
use userlib::ring::{BlockRing, BlockRequest, BlockResponse};

// Import from the usb driver library
use usb::{
    // GPIO commands
    GPIO_CMD_USB_VBUS,
    // xHCI registers
    xhci_op, xhci_rt, xhci_ir,
    // TRB
    Trb, trb_type, trb_cc, trb_ctrl,
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
    EP0_RING_SIZE, EP0_RING_USABLE, BULK_RING_SIZE,
    // MMIO helpers
    MmioRegion, delay_ms, delay,
    // Port status parsing
    ParsedPortsc, portsc,
    event_completion_code, event_slot_id, event_port_id,
    completion_code, doorbell,
    // Enumeration helpers
    build_enable_slot_trb, build_disable_slot_trb, build_noop_trb,
    // Memory layout constants
    CTX_OFFSET_DEVICE, CTX_OFFSET_EP0_RING,
    XHCI_OFFSET_ERST, XHCI_OFFSET_CMD_RING, XHCI_OFFSET_EVT_RING, XHCI_MEM_SIZE,
    // Hub helpers
    get_hub_descriptor_setup, get_port_status_setup,
    set_port_feature_setup, clear_port_feature_setup,
    // Cache ops (ARM64 cache maintenance for DMA) - kept for potential future use
    flush_cache_line, flush_buffer, invalidate_buffer, invalidate_cache_line, dsb,
    // Register bit constants
    ERDP_EHB, USBSTS_EINT, USBSTS_PCD,
};

// Modular architecture - Board abstraction hides SoC details
use usb::board::{Board, BpiR4, UsbControllerConfig};
use usb::soc::{SocUsb, Mt7988aSoc};
use usb::phy::{PhyDriver, Mt7988aTphy};
use usb::xhci::Controller as XhciController;

// Module definitions now imported from usb crate

// =============================================================================
// USB Timing Constants (from Linux kernel drivers/usb/core/hub.c)
// =============================================================================

mod usb_timing {
    // Hub debounce - connection must be stable before enumeration
    pub const HUB_DEBOUNCE_TIMEOUT: u32 = 2000;  // Max time to wait for stable connection
    pub const HUB_DEBOUNCE_STEP: u32 = 25;       // Poll interval during debounce
    pub const HUB_DEBOUNCE_STABLE: u32 = 100;    // Must be stable for this long

    // Reset timing
    pub const HUB_SHORT_RESET_TIME: u32 = 10;    // Minimum reset signal time
    pub const HUB_BH_RESET_TIME: u32 = 50;       // Warm reset (USB3)
    pub const HUB_LONG_RESET_TIME: u32 = 200;    // Extended reset for problematic devices
    pub const HUB_RESET_TIMEOUT: u32 = 800;      // Max wait for reset completion

    // Enumeration retries
    pub const PORT_RESET_TRIES: u32 = 5;
    pub const SET_ADDRESS_TRIES: u32 = 2;
    pub const GET_DESCRIPTOR_TRIES: u32 = 2;
    pub const GET_CONFIG_TRIES: u32 = 2;

    // Status/control timeouts
    pub const USB_STS_TIMEOUT: u32 = 1000;       // Port status fetch timeout
    pub const USB_CTRL_TIMEOUT: u32 = 5000;      // Control transfer timeout

    // Interrupt endpoint
    pub const HUB_IRQ_RETRY_DELAY: u32 = 1000;   // Retry delay on submission failure
    pub const HUB_IRQ_MAX_ERRORS: u32 = 10;      // Reset hub after this many errors

    // Post-operation delays
    pub const POST_RESET_DELAY: u32 = 10;        // After reset completion
    pub const POST_ADDRESS_DELAY: u32 = 2;       // After SET_ADDRESS (USB spec: 2ms)
    pub const POST_CONFIG_DELAY: u32 = 10;       // After SET_CONFIGURATION
}

// =============================================================================
// USB Driver
// =============================================================================

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

    // Hub device info (for interrupt-based port monitoring)
    hub_device: Option<HubDeviceInfo>,

    // Track which ports have enumerated devices (for event-driven model)
    // Bit N = 1 means port N+1 has an enumerated device
    enumerated_ports: u32,
}

/// Hub device info for interrupt-based port status monitoring
pub struct HubDeviceInfo {
    pub slot_id: u32,
    pub num_ports: u8,
    pub hub_port: u32,          // xHCI port this hub is on
    // Interrupt endpoint
    pub int_in_dci: u32,        // Device Context Index for interrupt IN
    pub int_in_ring: *mut Trb,
    pub int_in_ring_phys: u64,
    pub int_in_enqueue: usize,
    pub int_in_cycle: bool,
    // Status buffer (DMA target for interrupt transfers)
    pub status_buf: *mut u8,
    pub status_phys: u64,
    // EP0 for control transfers
    pub ep0_ring: *mut Trb,
    pub ep0_ring_phys: u64,
    pub ep0_enqueue: usize,
    pub ep0_cycle: bool,  // Cycle bit for EP0 ring wrapping
    // Device context
    pub device_ctx: *mut DeviceContext,
    // Pending transfer flag
    pub transfer_pending: bool,
    // Error tracking for robustness (Linux-style)
    pub irq_errors: u32,        // Consecutive interrupt errors
    pub irq_retry_pending: bool, // Need to retry interrupt submission
}

/// Mass Storage Class device info for block operations
/// Stores all state needed to perform bulk transfers without BulkContext
pub struct MscDeviceInfo {
    pub slot_id: u32,
    pub bulk_in_dci: u32,
    pub bulk_out_dci: u32,
    pub block_size: u32,
    pub block_count: u64,
    // Bulk IN ring
    pub bulk_in_ring: *mut Trb,
    pub bulk_in_ring_phys: u64,
    // Bulk OUT ring
    pub bulk_out_ring: *mut Trb,
    pub bulk_out_ring_phys: u64,
    // Data buffer
    pub data_buf: *mut u8,
    pub data_phys: u64,
    // Device context
    pub device_ctx: *mut DeviceContext,
    // Ring state (persisted between calls)
    pub out_enqueue: usize,
    pub in_enqueue: usize,
    pub out_cycle: bool,  // Cycle bit for bulk out ring
    pub in_cycle: bool,   // Cycle bit for bulk in ring
    pub tag: u32,
    // USB endpoint addresses (for CLEAR_FEATURE)
    pub bulk_in_addr: u8,
    pub bulk_out_addr: u8,
    // Interface number (for BOT Reset)
    pub interface_num: u8,
    // EP0 ring for control transfers (for USB recovery)
    pub ep0_ring: *mut Trb,
    pub ep0_ring_phys: u64,
    pub ep0_enqueue: usize,
    pub ep0_cycle: bool,  // Cycle bit for EP0 ring wrapping
    // Port info for recovery
    pub root_port: u8,         // Root port number (0-indexed)
    pub hub_slot: Option<u32>, // Hub slot if behind a hub, None if direct
    pub hub_port: u8,          // Hub port number if behind a hub
    // BOT state machine for transfer and recovery management
    pub state_machine: BotStateMachine,
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
        uinfo!("usbd", "init_start"; controller = config.name);

        // === Create SoC Wrapper ===
        let mut soc = board.create_soc(controller_id as u8)?;

        // Map MAC (xHCI + IPPC) region - soc and xhci need separate handles
        // Opening the same physical region twice gives us independent fds
        let soc_mac = MmioRegion::open(config.mac_base, config.mac_size as u64)?;
        soc.set_mac(soc_mac);

        // SoC pre-init (IPPC clocks, power, port control)
        if soc.pre_init().is_err() {
            uerror!("usbd", "soc_preinit_failed");
            return None;
        }

        // === Create PHY Driver ===
        let mut phy = board.create_phy(controller_id as u8)?;
        let phy_mmio = MmioRegion::open(config.phy_base, config.phy_size as u64)?;
        phy.set_mmio(phy_mmio);

        // === Create xHCI Controller ===
        // Open separate MAC region for xHCI (needs its own fd)
        let xhci_mac = MmioRegion::open(config.mac_base, config.mac_size as u64)?;
        let xhci = XhciController::new(xhci_mac);

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
            hub_device: None,
            enumerated_ports: 0,
        })
    }

    // Delegate register access to XhciController methods
    #[inline(always)]
    fn op_read32(&self, offset: usize) -> u32 {
        self.xhci.op_read32(offset)
    }

    #[inline(always)]
    fn op_write32(&self, offset: usize, value: u32) {
        self.xhci.op_write32(offset, value)
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
    fn wait_for_irq(&self, timeout_ms: u32) -> bool {
        if self.irq_fd < 0 {
            return false;
        }

        let mut buf = [0u8; 4];

        // Poll with timeout using non-blocking read and gettime
        let start = syscall::gettime();
        let timeout_ns = (timeout_ms as u64) * 1_000_000;

        loop {
            // Try non-blocking read
            let result = syscall::read(self.irq_fd as u32, &mut buf);
            if result > 0 {
                return true;
            }

            // Check timeout
            let elapsed = syscall::gettime() - start;
            if elapsed >= timeout_ns {
                return false;
            }

            // Yield to scheduler
            syscall::yield_now();
        }
    }

    fn init(&mut self) -> bool {
        // Read xHCI capabilities
        let caps = match self.xhci.read_capabilities() {
            Some(c) => c,
            None => {
                uerror!("usbd", "xhci_caps_failed");
                return false;
            }
        };
        uinfo!("usbd", "xhci_caps"; version = caps.version, ports = caps.max_ports as u64, slots = caps.max_slots as u64);

        // Halt and reset xHCI (critical for warm resets)
        let (halt_ok, reset_ok) = self.xhci.halt_and_reset();
        if !halt_ok || !reset_ok {
            uwarn!("usbd", "xhci_reset_issue"; halt = halt_ok, reset = reset_ok);
        }
        // Extra delay after reset for controller to stabilize
        delay_ms(50);

        // Configure PHY for host mode (after xHCI reset)
        if self.phy.set_host_mode().is_err() {
            uerror!("usbd", "phy_host_mode_failed");
            return false;
        }
        let _ = self.soc.force_usb2_phy_power();

        // Allocate DMA memory for xHCI data structures
        let mut phys_addr: u64 = 0;
        let mem_addr = syscall::mmap_dma(XHCI_MEM_SIZE, &mut phys_addr);
        if mem_addr < 0 {
            uerror!("usbd", "dma_alloc_failed"; size = XHCI_MEM_SIZE as u64);
            return false;
        }
        self.xhci_mem = mem_addr as u64;
        self.xhci_phys = phys_addr;

        // Memory layout (using constants from enumeration module)
        let dcbaa_virt = self.xhci_mem as *mut u64;
        let dcbaa_phys = self.xhci_phys;
        let erst_virt = (self.xhci_mem + XHCI_OFFSET_ERST) as *mut ErstEntry;
        let erst_phys = self.xhci_phys + XHCI_OFFSET_ERST;
        let cmd_ring_virt = (self.xhci_mem + XHCI_OFFSET_CMD_RING) as *mut Trb;
        let cmd_ring_phys = self.xhci_phys + XHCI_OFFSET_CMD_RING;
        let evt_ring_virt = (self.xhci_mem + XHCI_OFFSET_EVT_RING) as *mut Trb;
        let evt_ring_phys = self.xhci_phys + XHCI_OFFSET_EVT_RING;

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
            uerror!("usbd", "xhci_start_failed");
            return false;
        }

        // Power on ports and wait for connection
        self.xhci.power_on_all_ports();
        let mp = self.xhci.max_ports();
        if mp > 0 {
            if let Some(port) = self.xhci.wait_for_connection(100000) {
                uinfo!("usbd", "init_complete"; port = port as u64 + 1);
            } else {
                uinfo!("usbd", "init_complete"; port = 0u64);
            }
        } else {
            uwarn!("usbd", "no_ports");
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

    /// Poll for and process events
    /// Returns a bitmask of ports that need enumeration (bit N = port N+1 needs enumeration)
    fn poll_events(&mut self) -> u32 {
        let mut events: [Option<Trb>; 16] = [None; 16];
        let mut event_count = 0;
        let mut final_erdp = 0u64;
        let mut ports_to_enumerate = 0u32;

        // Check USBSTS for pending events
        let usbsts = self.op_read32(xhci_op::USBSTS);
        let eint = (usbsts & USBSTS_EINT) != 0;
        let pcd = (usbsts & USBSTS_PCD) != 0;

        // Auto-resync: check for cycle bit mismatch before reading
        let mut erdp_update: Option<u64> = None;
        if let Some(ref mut event_ring) = self.event_ring {
            let (idx, expected_cycle, ctrl) = event_ring.debug_state();
            let actual_cycle = (ctrl & 1) != 0;

            // If there's a mismatch and TRB is not empty, fix it
            if expected_cycle != actual_cycle && ctrl != 0 {
                // Scan for sync point
                let ring_size = 64usize;
                for offset in 0..ring_size {
                    let check_idx = (idx + offset) % ring_size;
                    unsafe {
                        let ptr = event_ring.trbs.add(check_idx);
                        invalidate_cache_line(ptr as u64);
                        dsb();
                        let trb = core::ptr::read_volatile(ptr);
                        let c = (trb.control & 1) != 0;
                        if c == expected_cycle {
                            event_ring.dequeue = check_idx;
                            erdp_update = Some(event_ring.erdp());
                            break;
                        }
                    }
                }
                // If no sync point found, flip cycle
                if erdp_update.is_none() {
                    event_ring.cycle = !event_ring.cycle;
                    event_ring.dequeue = 0;
                    erdp_update = Some(event_ring.erdp());
                }
            }

            // Now read events
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

        // Update ERDP if we did a resync
        if let Some(erdp) = erdp_update {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | ERDP_EHB);
        }

        if event_count == 0 {
            // Clear sticky EINT and PCD bits if no actual events found
            if eint || pcd {
                // Debug: EINT/PCD set but no events - cycle bit mismatch?
                if let Some(ref event_ring) = self.event_ring {
                    let (idx, expected_cycle, ctrl) = event_ring.debug_state();
                    let actual_cycle = (ctrl & 1) != 0;
                    let erdp_hw = self.rt_read64(xhci_rt::IR0 + xhci_ir::ERDP);
                    let erdp_sw = event_ring.erdp();
                    // Debug: EINT/PCD set but no events - cycle bit mismatch
                    // Verbose debug output removed - use uwarn for significant issues only
                    let _ = (eint, pcd, idx, expected_cycle, actual_cycle, ctrl, erdp_hw, erdp_sw);
                }
                self.op_write32(xhci_op::USBSTS, USBSTS_EINT | USBSTS_PCD);
            }
            return 0;
        }

        for i in 0..event_count {
            if let Some(ref trb) = events[i] {
                let evt_type = trb.get_type();
                match evt_type {
                    trb_type::PORT_STATUS_CHANGE => {
                        if let Some(port_id) = self.handle_port_change(event_port_id(trb)) {
                            // Mark this port for enumeration (port_id is 1-based)
                            if port_id > 0 && port_id <= 32 {
                                ports_to_enumerate |= 1 << (port_id - 1);
                            }
                        }
                    }
                    trb_type::TRANSFER_EVENT => {
                        // Check if this is a hub interrupt transfer completion
                        let slot_id = event_slot_id(trb);
                        let endpoint_id = ((trb.control >> 16) & 0x1F) as u32;
                        let cc = (trb.status >> 24) & 0xFF;
                        let transfer_len = trb.status & 0xFFFFFF;

                        // Check if this is our hub's interrupt endpoint
                        let is_hub_int = self.hub_device.as_ref()
                            .map(|h| slot_id == h.slot_id && endpoint_id == h.int_in_dci)
                            .unwrap_or(false);

                        if is_hub_int {
                            if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {
                                // Success - reset error counter
                                if let Some(ref mut hub) = self.hub_device {
                                    hub.irq_errors = 0;
                                }
                                // Handle the hub status change
                                let status_bitmap = self.handle_hub_status_transfer(transfer_len);
                                if status_bitmap != 0 {
                                    self.process_hub_port_changes(status_bitmap);
                                } else {
                                    // No changes - re-queue the transfer
                                    self.requeue_hub_interrupt();
                                }
                            } else {
                                // Error - increment counter and check for hub reset
                                let should_reset = if let Some(ref mut hub) = self.hub_device {
                                    hub.irq_errors += 1;
                                    uwarn!("usbd", "hub_interrupt_error"; cc = cc as u64, errors = hub.irq_errors as u64);
                                    hub.irq_errors >= usb_timing::HUB_IRQ_MAX_ERRORS
                                } else {
                                    false
                                };

                                if should_reset {
                                    uwarn!("usbd", "hub_interrupt_reset"; reason = "too_many_errors");
                                    // Reset error counter and perform full endpoint reset
                                    if let Some(ref mut hub) = self.hub_device {
                                        hub.irq_errors = 0;
                                    }
                                    self.reset_hub_interrupt_endpoint();
                                } else {
                                    self.requeue_hub_interrupt();
                                }
                            }
                        }
                    }
                    trb_type::HOST_CONTROLLER => {
                        uerror!("usbd", "host_controller_error");
                    }
                    _ => {}
                }
            }
        }

        let ir0 = xhci_rt::IR0;
        self.rt_write64(ir0 + xhci_ir::ERDP, final_erdp | ERDP_EHB);
        ports_to_enumerate
    }

    /// Resync event ring state before daemon loop
    /// This drains any pending events and ensures ERDP is properly synchronized with hardware
    fn resync_event_ring(&mut self) {
        // First try to drain any remaining events
        let mut drained = 0;
        let mut erdp_to_write: Option<u64> = None;

        if let Some(ref mut event_ring) = self.event_ring {
            // Check if we have a cycle bit mismatch
            let (idx, expected_cycle, ctrl) = event_ring.debug_state();
            let actual_cycle = (ctrl & 1) != 0;

            if expected_cycle != actual_cycle && ctrl != 0 {
                // Cycle bit mismatch detected! Hard reset the event ring.
                uwarn!("usbd", "event_ring_desync"; idx = idx as u64);

                // Scan to find where hardware's cycle actually is
                let ring_size = 64usize;
                let mut found_sync_point = false;

                for offset in 0..ring_size {
                    let check_idx = (idx + offset) % ring_size;
                    unsafe {
                        let ptr = event_ring.trbs.add(check_idx);
                        invalidate_cache_line(ptr as u64);
                        dsb();
                        let trb = core::ptr::read_volatile(ptr);
                        let c = (trb.control & 1) != 0;

                        // Found a TRB matching our expected cycle?
                        if c == expected_cycle {
                            event_ring.dequeue = check_idx;
                            found_sync_point = true;
                            erdp_to_write = Some(event_ring.erdp());
                            break;
                        }
                    }
                }

                if !found_sync_point {
                    // All TRBs have wrong cycle - flip our cycle to match hardware
                    event_ring.cycle = !event_ring.cycle;
                    event_ring.dequeue = 0;
                    erdp_to_write = Some(event_ring.erdp());
                }
            }

            // Now try to drain any events we can read
            for _ in 0..256 {
                if let Some(_evt) = event_ring.dequeue() {
                    drained += 1;
                    erdp_to_write = Some(event_ring.erdp());
                } else {
                    break;
                }
            }
        }

        // Update ERDP after releasing the borrow
        if let Some(erdp) = erdp_to_write {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | ERDP_EHB);
        }

        // Verbose debug output removed - stale event drain is normal operation

        // Clear sticky USBSTS bits (W1C)
        self.op_write32(xhci_op::USBSTS, USBSTS_EINT | USBSTS_PCD);
    }

    /// Handle port status change event
    /// Returns Some(port_id) if a new device was connected and needs enumeration
    fn handle_port_change(&mut self, port_id: u32) -> Option<u32> {
        if port_id == 0 || port_id > self.xhci.max_ports() as u32 {
            return None;
        }
        let port = (port_id - 1) as u8;
        let raw = self.xhci.read_portsc(port);
        let status = ParsedPortsc::from_raw(raw);

        // Clear all change bits first
        let clear_bits = raw & portsc::RW1C_BITS;
        if clear_bits != 0 {
            self.xhci.write_portsc(port, status.clear_changes_value());
        }

        // Check for connect status change
        if status.connect_change {
            let port_mask = 1u32 << port;

            if status.connected {
                // Device connected - check if not already enumerated
                if (self.enumerated_ports & port_mask) == 0 {
                    uinfo!("usbd", "device_connected"; port = port_id as u64);
                    return Some(port_id);
                }
            } else {
                // Device disconnected - mark as not enumerated
                if (self.enumerated_ports & port_mask) != 0 {
                    uinfo!("usbd", "device_disconnected"; port = port_id as u64);
                    self.enumerated_ports &= !port_mask;
                    // Clean up device resources (slot, DMA buffers)
                    self.cleanup_port_device(port_id);
                }
            }
        }

        None
    }

    #[allow(dead_code)]
    fn print_port_status(&self) {
        // Debug port status printing disabled - use structured logging if needed
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
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | ERDP_EHB);
            // Clear EINT bit (W1C) to acknowledge we processed the event
            self.op_write32(xhci_op::USBSTS, USBSTS_EINT);
        }

        if let Some((cc, slot_id)) = result {
            if cc == trb_cc::SUCCESS {
                return Some(slot_id);
            }
        }
        None
    }

    /// Disable a slot when a device is disconnected
    fn disable_slot(&mut self, slot_id: u32) -> bool {
        let cmd_ring = match self.cmd_ring.as_mut() {
            Some(r) => r,
            None => return false,
        };

        // Send DISABLE_SLOT command
        let trb = build_disable_slot_trb(slot_id);
        cmd_ring.enqueue(&trb);
        self.ring_doorbell(0, doorbell::HOST_CONTROLLER);
        delay(10000);

        let mut result: Option<u32> = None;
        let mut erdp_to_update: Option<u64> = None;

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..20 {
                if let Some(evt) = event_ring.dequeue() {
                    erdp_to_update = Some(event_ring.erdp());
                    if evt.get_type() == trb_type::COMMAND_COMPLETION {
                        result = Some(event_completion_code(&evt));
                        break;
                    }
                }
                delay(1000);
            }
        }

        if let Some(erdp) = erdp_to_update {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | ERDP_EHB);
            self.op_write32(xhci_op::USBSTS, USBSTS_EINT);
        }

        // Clear DCBAA entry for this slot
        let dcbaa = self.xhci_mem as *mut u64;
        unsafe {
            *dcbaa.add(slot_id as usize) = 0;
            flush_cache_line(dcbaa.add(slot_id as usize) as u64);
        }

        if let Some(cc) = result {
            cc == trb_cc::SUCCESS
        } else {
            false
        }
    }

    /// Clean up MSC device resources when disconnected
    fn cleanup_msc_device(&mut self) {
        if let Some(msc) = self.msc_device.take() {
            uinfo!("usbd", "msc_cleanup"; slot = msc.slot_id as u64);

            // Disable the slot in xHCI
            self.disable_slot(msc.slot_id);

            // Free DMA buffers - each is a 4KB page except data_buf which is larger
            // These were allocated with mmap_dma, so munmap them
            syscall::munmap(msc.bulk_in_ring as u64, 4096);
            syscall::munmap(msc.bulk_out_ring as u64, 4096);
            syscall::munmap(msc.data_buf as u64, 64 * 1024);  // DATA_BUFFER_SIZE
            // device_ctx and ep0_ring are in the same allocation (ctx_virt)
            // The context allocation includes input_ctx + device_ctx + ep0_ring
            // Calculate base of the allocation (device_ctx is at CTX_OFFSET_DEVICE)
            let ctx_base = (msc.device_ctx as u64) - CTX_OFFSET_DEVICE;
            syscall::munmap(ctx_base, 4096);
        }
    }

    /// Clean up Hub device resources when disconnected
    fn cleanup_hub_device(&mut self) {
        if let Some(hub) = self.hub_device.take() {
            uinfo!("usbd", "hub_cleanup"; slot = hub.slot_id as u64);

            // Disable the slot in xHCI
            self.disable_slot(hub.slot_id);

            // Free DMA buffers
            // int_in_ring is a separate 4KB allocation
            syscall::munmap(hub.int_in_ring as u64, 4096);
            // status_buf is a separate 4KB allocation
            syscall::munmap(hub.status_buf as u64, 4096);
            // device_ctx and ep0_ring are in the same allocation
            let ctx_base = (hub.device_ctx as u64) - CTX_OFFSET_DEVICE;
            syscall::munmap(ctx_base, 4096);
        }
    }

    /// Clean up device on a specific port
    fn cleanup_port_device(&mut self, port: u32) {
        // Check if MSC device is on this port
        if let Some(ref msc) = self.msc_device {
            // MSC devices come through the hub, check hub_port
            // For now, just clean up if we detect disconnect
            let _ = msc;  // suppress warning
        }

        // Check if Hub device is on this port
        if let Some(ref hub) = self.hub_device {
            if hub.hub_port == port {
                // Hub is on this port, clean it up
                // First clean up any MSC device connected through the hub
                self.cleanup_msc_device();
                self.cleanup_hub_device();
                return;
            }
        }

        // If we get here, might be a direct MSC device on this port
        // For simplicity, clean up MSC if it exists
        self.cleanup_msc_device();
    }

    /// Enumerate a port (reset + enumerate + mark as enumerated)
    /// Called when a new device is detected via port status change event
    /// Includes retry logic for flaky USB devices
    fn enumerate_port(&mut self, port: u32) -> Option<u32> {
        // Retry the entire enumeration process
        for attempt in 0..3 {
            if attempt > 0 {
                delay_ms(200);
            }

            // Check port status first
            let port_idx = (port - 1) as u8;
            let raw = self.xhci.read_portsc(port_idx);
            let status = ParsedPortsc::from_raw(raw);

            if !status.connected {
                return None;
            }

            // Reset the port first (unless already enabled)
            if !status.enabled {
                if !self.reset_port(port) {
                    continue; // Retry
                }
                // Small delay after reset
                delay_ms(50);
            }

            // Enumerate the device
            if let Some(slot_id) = self.enumerate_device(port) {
                // Mark port as enumerated
                if port > 0 && port <= 32 {
                    self.enumerated_ports |= 1 << (port - 1);
                }
                uinfo!("usbd", "port_enumerated"; port = port as u64, slot = slot_id as u64);
                return Some(slot_id);
            }
            // enumerate_device failed, will retry
        }

        uerror!("usbd", "enumeration_failed"; port = port as u64);
        None
    }

    /// Enumerate all ports in a bitmask
    fn enumerate_pending_ports(&mut self, ports_mask: u32) {
        for bit in 0..32 {
            if (ports_mask & (1 << bit)) != 0 {
                let port = bit + 1;  // Ports are 1-based
                self.enumerate_port(port);
            }
        }
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
        let device_ctx_virt = (ctx_base + CTX_OFFSET_DEVICE) as *mut DeviceContext;
        let device_ctx_phys = ctx_phys_base + CTX_OFFSET_DEVICE;
        let ep0_ring_virt = (ctx_base + CTX_OFFSET_EP0_RING) as *mut Trb;
        let ep0_ring_phys = ctx_phys_base + CTX_OFFSET_EP0_RING;

        // Initialize EP0 transfer ring (EP0_RING_SIZE TRBs - fits at CTX_OFFSET_EP0_RING)
        // This gives us EP0_RING_USABLE slots, enough for hub enumeration without wrapping
        unsafe {
            for i in 0..EP0_RING_SIZE {
                *ep0_ring_virt.add(i) = Trb::new();
            }
            // Link TRB at end
            let link = &mut *ep0_ring_virt.add(EP0_RING_USABLE);
            link.param = ep0_ring_phys;
            link.set_type(trb_type::LINK);
            link.control |= trb_ctrl::TC;  // Toggle Cycle

            // Flush EP0 ring to memory
            flush_buffer(ep0_ring_virt as u64, EP0_RING_SIZE * core::mem::size_of::<Trb>());
        }

        // Get port speed from PORTSC using library helpers
        let raw = self.xhci.read_portsc((port - 1) as u8);
        let status = ParsedPortsc::from_raw(raw);
        let speed = status.speed;
        let speed_raw = speed.to_slot_speed();

        uinfo!("usbd", "port_enumerate"; port = port as u64, speed = speed_raw as u64);

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

        // Wait for completion (longer timeout for reliability)
        delay_ms(100);

        let mut addr_result: Option<u32> = None;
        let mut erdp_to_write: Option<u64> = None;
        let mut fail_cc: Option<u32> = None;

        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..100 {
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
                delay(2000);
            }
        }

        if let Some(erdp) = erdp_to_write {
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | ERDP_EHB);
            // Clear EINT bit (W1C) to acknowledge we processed the event
            self.op_write32(xhci_op::USBSTS, USBSTS_EINT);
        }

        if addr_result.is_none() {
            uerror!("usbd", "address_failed"; slot = slot_id as u64, cc = fail_cc.unwrap_or(0) as u64);
            // Clean up allocated DMA buffer on failure
            syscall::munmap(ctx_virt as u64, 4096);
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

        // EP0 transfer ring - track enqueue position and cycle bit for ring wrapping
        let mut ep0_enqueue = 0usize;  // Start at beginning
        let mut ep0_cycle = true;  // Cycle starts at 1

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
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | ERDP_EHB);
            // Clear EINT bit (W1C) to acknowledge we processed the event
            self.op_write32(xhci_op::USBSTS, USBSTS_EINT);
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

                // Log device class type
                if is_hub_class(desc.device_class) {
                    // Hub detection logged in enumerate_hub with port count
                    self.enumerate_hub(slot_id, port, ep0_ring_virt, ep0_ring_phys, &mut ep0_enqueue, &mut ep0_cycle, device_ctx_virt);
                }
            }
        } else {
            uerror!("usbd", "descriptor_failed"; slot = slot_id as u64);
        }

        // Free temporary descriptor buffer
        syscall::munmap(desc_virt as u64, 4096);

        Some(slot_id)
    }

    /// Perform a control transfer on the given slot's EP0
    /// Returns the completion code and bytes transferred
    /// ep0_enqueue tracks the current position in the ring (updated after transfer)
    /// ep0_cycle tracks the cycle bit (toggled when ring wraps)
    fn control_transfer(
        &mut self,
        slot_id: u32,
        ep0_ring_virt: *mut Trb,
        _ep0_ring_phys: u64,
        ep0_enqueue: &mut usize,
        ep0_cycle: &mut bool,
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

        // EP0 ring uses EP0_RING_SIZE TRBs (EP0_RING_USABLE usable + link at end)
        let trbs_needed = if has_data { 3 } else { 2 };

        // If not enough space before link TRB, wrap to beginning
        if *ep0_enqueue + trbs_needed >= EP0_RING_USABLE {
            unsafe {
                let cycle_bit = *ep0_cycle as u32;

                // Fill gap from current position to Link TRB with No-Op TRBs
                // so hardware can advance past them to reach the Link TRB
                for i in *ep0_enqueue..EP0_RING_USABLE {
                    let noop = &mut *ep0_ring_virt.add(i);
                    noop.param = 0;
                    noop.status = 0;
                    // No-Op Transfer TRB with current cycle bit
                    noop.control = (trb_type::NOOP_TRANSFER << 10) | cycle_bit;
                }

                // Flush the No-Op TRBs
                if *ep0_enqueue < EP0_RING_USABLE {
                    let gap_size = EP0_RING_USABLE - *ep0_enqueue;
                    flush_buffer(
                        ep0_ring_virt.add(*ep0_enqueue) as u64,
                        gap_size * core::mem::size_of::<Trb>()
                    );
                }

                // Update Link TRB with current cycle bit before wrapping
                let link = &mut *ep0_ring_virt.add(EP0_RING_USABLE);
                // Set Link TRB cycle bit to current cycle (hardware will process it)
                // Keep TC bit set (bit 1) so hardware toggles when following link
                link.control = (trb_type::LINK << 10) | trb_ctrl::TC | cycle_bit;
                flush_cache_line(link as *const _ as u64);
            }
            // Toggle software cycle and wrap enqueue
            *ep0_cycle = !*ep0_cycle;
            *ep0_enqueue = 0;
        }

        let start_idx = *ep0_enqueue;
        let cycle_bit = *ep0_cycle as u32;

        unsafe {
            // Setup TRB
            let setup_trb = &mut *ep0_ring_virt.add(start_idx);
            setup_trb.param = setup_data;
            setup_trb.status = 8;
            let trt = if !has_data { 0 } else if is_in { 3 } else { 2 };
            setup_trb.control = (trb_type::SETUP << 10) | (trt << 16) | (1 << 6) | cycle_bit;

            let mut next_idx = start_idx + 1;

            // Data Stage TRB (if needed)
            if has_data {
                let data_trb = &mut *ep0_ring_virt.add(next_idx);
                data_trb.param = data_buf_phys;
                data_trb.status = length as u32;
                let dir = if is_in { 1 << 16 } else { 0 };
                data_trb.control = (trb_type::DATA << 10) | dir | cycle_bit;
                next_idx += 1;
            }

            // Status Stage TRB
            let status_trb = &mut *ep0_ring_virt.add(next_idx);
            status_trb.param = 0;
            status_trb.status = 0;
            // DIR is opposite of data direction (or IN if no data)
            let status_dir = if has_data && is_in { 0 } else { 1 << 16 };
            status_trb.control = (trb_type::STATUS << 10) | status_dir | (1 << 5) | cycle_bit;

            *ep0_enqueue = next_idx + 1;

            // Flush TRBs to memory before doorbell
            let trbs_written = next_idx + 1 - start_idx;
            flush_buffer(ep0_ring_virt.add(start_idx) as u64, trbs_written * core::mem::size_of::<Trb>());
        }

        // Ring doorbell for EP0
        self.ring_doorbell(slot_id, 1);

        // Debug: check USBSTS before waiting
        let usbsts_before = self.op_read32(xhci_op::USBSTS);

        // Wait for completion - longer wait for hub and downstream devices
        // slot_id 1 = hub, slot_id >= 2 = devices behind hub
        let initial_wait = if slot_id >= 2 { 100000 } else { 50000 };
        delay(initial_wait);

        let _ = usbsts_before;  // Suppress warning

        // Poll for transfer event
        let mut result: Option<(u32, u32)> = None;
        let mut erdp_to_update: Option<u64> = None;

        // Use more iterations for downstream hub devices
        let max_iterations = if slot_id >= 2 { 300 } else { 100 };

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
            if (usbsts & USBSTS_EINT) != 0 {
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
            self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | ERDP_EHB);
            // Clear EINT bit (W1C) to acknowledge we processed the event
            self.op_write32(xhci_op::USBSTS, USBSTS_EINT);
        }

        if let Some((cc, remaining)) = result {
            let transferred = (length as u32).saturating_sub(remaining);
            Some((cc, transferred))
        } else {
            None
        }
    }

    /// Send SET_CONFIGURATION request (silent unless error)
    fn set_configuration(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, ep0_cycle: &mut bool, config: u8) -> bool {
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
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
    fn get_hub_descriptor(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, ep0_cycle: &mut bool, buf_phys: u64) -> Option<u8> {
        let setup = get_hub_descriptor_setup(true);
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
            setup.request_type, setup.request, setup.value, setup.index,
            buf_phys, setup.length
        ) {
            Some((cc, len)) if completion_code::is_success(cc) => Some(len as u8),
            _ => None
        }
    }

    /// Get Hub Descriptor (USB 2.0) - silent
    fn get_usb2_hub_descriptor(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, ep0_cycle: &mut bool, buf_phys: u64) -> Option<u8> {
        let setup = get_hub_descriptor_setup(false);  // false = USB 2.0 hub descriptor (0x29)
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
            setup.request_type, setup.request, setup.value, setup.index,
            buf_phys, setup.length
        ) {
            Some((cc, len)) if completion_code::is_success(cc) => Some(len as u8),
            _ => None
        }
    }

    /// Get port status from hub
    fn hub_get_port_status(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, ep0_cycle: &mut bool, port: u16, buf_virt: u64, buf_phys: u64) -> Option<PortStatus> {
        let setup = get_port_status_setup(port);
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
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

    /// Parse hub configuration descriptor to find interrupt IN endpoint
    /// Returns (ep_addr, max_packet, interval) if found
    fn parse_hub_config_descriptor(&self, buf: *const u8, total_len: usize) -> Option<(u8, u16, u8)> {
        let mut offset = 0usize;
        let mut in_hub_interface = false;

        while offset + 2 <= total_len {
            let len = unsafe { *buf.add(offset) } as usize;
            let desc_type = unsafe { *buf.add(offset + 1) };

            if len < 2 || offset + len > total_len {
                break;
            }

            match desc_type {
                4 => {  // Interface descriptor
                    if len >= 9 {
                        let iface_class = unsafe { *buf.add(offset + 5) };
                        // Hub class = 0x09
                        in_hub_interface = iface_class == 0x09;
                    }
                }
                5 => {  // Endpoint descriptor
                    if len >= 7 && in_hub_interface {
                        let ep_addr = unsafe { *buf.add(offset + 2) };
                        let ep_attr = unsafe { *buf.add(offset + 3) };
                        let ep_max_pkt = unsafe { core::ptr::read_unaligned(buf.add(offset + 4) as *const u16) };
                        let ep_interval = unsafe { *buf.add(offset + 6) };

                        // Interrupt IN endpoint: attr & 0x03 == 3, addr & 0x80 != 0
                        if (ep_attr & 0x03) == 3 && (ep_addr & 0x80) != 0 {
                            return Some((ep_addr, ep_max_pkt, ep_interval));
                        }
                    }
                }
                _ => {}
            }
            offset += len;
        }
        None
    }

    /// Set up hub interrupt endpoint for status change notifications
    fn setup_hub_interrupt_endpoint(
        &mut self,
        hub_slot: u32,
        hub_port: u32,
        num_ports: u8,
        int_ep_addr: u8,
        int_max_packet: u16,
        int_interval: u8,
        device_ctx_virt: *mut DeviceContext,
        ep0_ring_virt: *mut Trb,
        ep0_ring_phys: u64,
        ep0_enqueue: usize,
    ) -> bool {
        // Allocate interrupt transfer ring (small - 16 entries is enough)
        let mut int_ring_phys: u64 = 0;
        let int_ring_virt = syscall::mmap_dma(4096, &mut int_ring_phys);
        if int_ring_virt < 0 { return false; }

        // Allocate status buffer (1 byte for hub status bitmap)
        let mut status_phys: u64 = 0;
        let status_virt = syscall::mmap_dma(4096, &mut status_phys);
        if status_virt < 0 {
            syscall::munmap(int_ring_virt as u64, 4096);
            return false;
        }

        // Initialize interrupt ring with Link TRB at index 15
        unsafe {
            let ring = int_ring_virt as *mut Trb;
            for i in 0..16 {
                *ring.add(i) = Trb::new();
            }
            // Link TRB at index 15
            let link = &mut *ring.add(15);
            link.param = int_ring_phys;
            link.set_type(trb_type::LINK);
            link.control |= trb_ctrl::TC;  // Toggle Cycle
            // Flush entire ring
            flush_buffer(ring as u64, 16 * core::mem::size_of::<Trb>());
        }

        // Calculate DCI for interrupt IN endpoint
        let ep_num = (int_ep_addr & 0x0F) as u32;
        let int_in_dci = ep_num * 2 + 1;  // IN endpoint

        // Allocate Input Context for Configure Endpoint
        let mut input_ctx_phys: u64 = 0;
        let input_ctx_virt = syscall::mmap_dma(4096, &mut input_ctx_phys);
        if input_ctx_virt < 0 {
            syscall::munmap(int_ring_virt as u64, 4096);
            syscall::munmap(status_virt as u64, 4096);
            return false;
        }

        // Set up Input Context
        unsafe {
            let input = &mut *(input_ctx_virt as *mut InputContext);
            core::ptr::write_bytes(input, 0, 1);

            // Add flags: slot + interrupt endpoint
            input.control.add_flags = 1 | (1 << int_in_dci);

            // Copy current slot context
            let dev_ctx = &*device_ctx_virt;
            input.slot = dev_ctx.slot;
            input.slot.set_context_entries(int_in_dci);

            // Configure interrupt IN endpoint
            let ep_idx = (int_in_dci - 1) as usize;
            input.endpoints[ep_idx].set_ep_type(ep_type::INTERRUPT_IN);
            input.endpoints[ep_idx].set_max_packet_size(int_max_packet as u32);
            input.endpoints[ep_idx].set_max_burst_size(0);
            input.endpoints[ep_idx].set_cerr(3);
            input.endpoints[ep_idx].set_tr_dequeue_ptr(int_ring_phys, true);
            input.endpoints[ep_idx].set_average_trb_length(2);  // Match max packet size
            // For SS periodic endpoints, Max ESIT Payload = Max Packet Size * (Max Burst Size + 1)
            // Max Burst Size = 0, so Max ESIT Payload = Max Packet Size = 2
            input.endpoints[ep_idx].set_max_esit_payload_lo(int_max_packet as u32);
            // Interval: for SS, value is 2^(interval-1) * 125us
            // For hub status, we can use a reasonable polling interval
            input.endpoints[ep_idx].set_interval(int_interval as u32);

            // Flush input context
            flush_buffer(input_ctx_virt as u64, core::mem::size_of::<InputContext>());
        }

        // Issue CONFIGURE_ENDPOINT command
        let mut cmd_success = false;
        if let Some(ref mut cmd_ring) = self.cmd_ring {
            let mut trb = Trb::new();
            trb.param = input_ctx_phys;
            trb.set_type(trb_type::CONFIGURE_ENDPOINT);
            trb.control |= (hub_slot & 0xFF) << 24;
            cmd_ring.enqueue(&trb);
            self.ring_doorbell(0, doorbell::HOST_CONTROLLER);
            delay(10000);

            // Wait for command completion
            let mut erdp_to_update: Option<u64> = None;
            if let Some(ref mut event_ring) = self.event_ring {
                for _ in 0..50 {
                    if let Some(evt) = event_ring.dequeue() {
                        erdp_to_update = Some(event_ring.erdp());
                        if evt.get_type() == trb_type::COMMAND_COMPLETION {
                            let cc = (evt.status >> 24) & 0xFF;
                            cmd_success = cc == trb_cc::SUCCESS;
                            break;
                        }
                    }
                    delay(1000);
                }
            }
            if let Some(erdp) = erdp_to_update {
                self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | ERDP_EHB);
            }
        }

        // Free temporary input context - no longer needed after command
        syscall::munmap(input_ctx_virt as u64, 4096);

        if !cmd_success {
            uerror!("usbd", "hub_configure_failed"; slot = hub_slot as u64, dci = int_in_dci as u64);
            syscall::munmap(int_ring_virt as u64, 4096);
            syscall::munmap(status_virt as u64, 4096);
            return false;
        }

        // Endpoint context debug removed - verbose register dump

        // Store hub device info
        // Note: ep0_cycle is true because we haven't wrapped the ring yet during enumeration
        self.hub_device = Some(HubDeviceInfo {
            slot_id: hub_slot,
            num_ports,
            hub_port,
            int_in_dci,
            int_in_ring: int_ring_virt as *mut Trb,
            int_in_ring_phys: int_ring_phys,
            int_in_enqueue: 0,
            int_in_cycle: true,
            status_buf: status_virt as *mut u8,
            status_phys,
            ep0_ring: ep0_ring_virt,
            ep0_ring_phys,
            ep0_enqueue,
            ep0_cycle: true,  // Cycle starts at 1, toggled when ring wraps
            device_ctx: device_ctx_virt,
            transfer_pending: false,
            irq_errors: 0,
            irq_retry_pending: false,
        });

        true
    }

    /// Queue an interrupt transfer on the hub's status endpoint
    fn queue_hub_status_transfer(&mut self) -> bool {
        // Extract values we need before borrowing self mutably for doorbell
        let (slot_id, int_in_dci) = {
            let hub = match self.hub_device.as_mut() {
                Some(h) => h,
                None => {
                    return false;
                }
            };

            if hub.transfer_pending {
                return true;  // Already have a transfer queued
            }

            // Build Normal TRB for interrupt transfer
            // Use max packet size (2) to match endpoint descriptor
            // This might help xHCI schedule the transfer correctly
            let transfer_len = 2usize;  // max_packet from endpoint descriptor

            unsafe {
                let ring = hub.int_in_ring;
                let trb = &mut *ring.add(hub.int_in_enqueue);

                trb.param = hub.status_phys;
                trb.status = transfer_len as u32;
                // Normal TRB: ISP (bit 2), IOC (bit 5), cycle bit (bit 0)
                // ISP = Interrupt on Short Packet (for IN endpoints)
                // IOC = Interrupt On Completion
                // Interrupter Target = 0 (bits 22-31)
                trb.control = (trb_type::NORMAL << 10) | (1 << 5) | (1 << 2) | (hub.int_in_cycle as u32);

                flush_cache_line(trb as *const Trb as u64);
                dsb();

                // Advance enqueue pointer
                hub.int_in_enqueue += 1;
                if hub.int_in_enqueue >= 15 {  // Link TRB at 15
                    hub.int_in_enqueue = 0;
                    hub.int_in_cycle = !hub.int_in_cycle;
                }
            }

            hub.transfer_pending = true;
            (hub.slot_id, hub.int_in_dci)
        };
        let _ = int_in_dci;  // Suppress warning

        // Ring the doorbell for the interrupt endpoint
        self.ring_doorbell(slot_id, int_in_dci as u32);

        true
    }

    /// Reset hub interrupt endpoint after too many errors (Linux-style)
    /// This performs xHCI endpoint reset, reinitializes the transfer ring, and requeues
    fn reset_hub_interrupt_endpoint(&mut self) {
        // Extract hub info first to avoid borrow issues
        let (slot_id, int_in_dci, ring, ring_phys) = match self.hub_device.as_ref() {
            Some(h) => (h.slot_id, h.int_in_dci, h.int_in_ring, h.int_in_ring_phys),
            None => return,
        };

        // Step 1: Reset endpoint via xHCI command
        let reset_trb = usb::enumeration::build_reset_endpoint_trb(slot_id, int_in_dci, false);
        if let Some(ref mut cmd_ring) = self.cmd_ring {
            cmd_ring.enqueue(&reset_trb);
        }
        self.ring_doorbell(0, 0);
        if !self.wait_command_complete(100) {
            uerror!("usbd", "hub_reset_ep_failed"; slot = slot_id as u64, dci = int_in_dci as u64);
        }

        // Step 2: Reinitialize the interrupt ring
        unsafe {
            // Clear all TRBs
            for i in 0..16 {
                let trb = &mut *ring.add(i);
                trb.param = 0;
                trb.status = 0;
                trb.control = 0;
            }
            // Setup Link TRB at index 15
            let link = &mut *ring.add(15);
            link.param = ring_phys;
            link.set_type(trb_type::LINK);
            link.control |= trb_ctrl::TC;  // Toggle Cycle
            flush_buffer(ring as u64, 16 * core::mem::size_of::<Trb>());
        }

        // Reset ring state
        if let Some(ref mut hub) = self.hub_device {
            hub.int_in_enqueue = 0;
            hub.int_in_cycle = true;
            hub.transfer_pending = false;
        }

        // Step 3: Set TR Dequeue Pointer (DCS=1 to match cycle)
        let set_deq = usb::enumeration::build_set_tr_dequeue_trb(slot_id, int_in_dci, ring_phys, true);
        if let Some(ref mut cmd_ring) = self.cmd_ring {
            cmd_ring.enqueue(&set_deq);
        }
        self.ring_doorbell(0, 0);
        let _ = self.wait_command_complete(100);

        // Step 4: Re-queue the interrupt transfer
        self.requeue_hub_interrupt();
    }

    /// Requeue hub interrupt with retry logic (Linux-style)
    /// If queue fails, sets retry flag for main loop to handle
    fn requeue_hub_interrupt(&mut self) {
        if !self.queue_hub_status_transfer() {
            // Failed to queue - set retry flag (will be retried in main loop)
            if let Some(ref mut hub) = self.hub_device {
                if !hub.irq_retry_pending {
                    hub.irq_retry_pending = true;
                }
            }
        } else {
            // Successfully queued - clear retry flag
            if let Some(ref mut hub) = self.hub_device {
                hub.irq_retry_pending = false;
            }
        }
    }

    /// Handle hub status change interrupt transfer completion
    /// Returns bitmask of ports with status changes
    fn handle_hub_status_transfer(&mut self, transfer_len: u32) -> u8 {
        let hub = match self.hub_device.as_mut() {
            Some(h) => h,
            None => return 0,
        };

        hub.transfer_pending = false;

        if transfer_len == 0 {
            return 0;
        }

        // Invalidate status buffer
        invalidate_buffer(hub.status_buf as u64, transfer_len as usize);

        // Read status bitmap - bit N indicates port N has a change
        // Bit 0 = hub status change, bits 1-N = port 1-N changes
        unsafe { *hub.status_buf }
    }

    /// Process hub port changes from interrupt transfer
    /// Called when we receive a Transfer Event for the hub's interrupt endpoint
    fn process_hub_port_changes(&mut self, status_bitmap: u8) {
        // Get hub info - we need to copy what we need before borrowing self mutably
        let (hub_slot, num_ports, root_port, ep0_ring, ep0_ring_phys, mut ep0_enqueue, mut ep0_cycle) = {
            let hub = match self.hub_device.as_ref() {
                Some(h) => h,
                None => return,
            };
            (hub.slot_id, hub.num_ports, hub.hub_port, hub.ep0_ring, hub.ep0_ring_phys, hub.ep0_enqueue, hub.ep0_cycle)
        };

        // Allocate buffer for port status queries
        let mut buf_phys: u64 = 0;
        let buf_virt = syscall::mmap_dma(4096, &mut buf_phys);
        if buf_virt < 0 {
            return;
        }

        // Bit 0 = hub status change (ignored for now)
        // Bit N (1-7) = port N has status change
        for port in 1..=num_ports {
            if (status_bitmap & (1 << port)) != 0 {
                // Get port status
                if let Some(ps) = self.hub_get_port_status(
                    hub_slot, ep0_ring, ep0_ring_phys, &mut ep0_enqueue, &mut ep0_cycle,
                    port as u16, buf_virt as u64, buf_phys
                ) {
                    // Clear any change bits first
                    if (ps.change & hub::PS_C_CONNECTION) != 0 {
                        self.hub_clear_port_feature(hub_slot, ep0_ring, ep0_ring_phys, &mut ep0_enqueue, &mut ep0_cycle, port as u16, hub::C_PORT_CONNECTION);
                    }

                    // Check if device is connected
                    if (ps.status & hub::PS_CONNECTION) != 0 {
                        uinfo!("usbd", "hub_port_connect"; port = port as u64);

                        // Reset the port
                        if self.hub_set_port_feature(hub_slot, ep0_ring, ep0_ring_phys, &mut ep0_enqueue, &mut ep0_cycle, port as u16, hub::PORT_RESET) {
                            // Wait for reset to complete
                            delay_ms(100);

                            // Check port status again
                            if let Some(ps2) = self.hub_get_port_status(
                                hub_slot, ep0_ring, ep0_ring_phys, &mut ep0_enqueue, &mut ep0_cycle,
                                port as u16, buf_virt as u64, buf_phys
                            ) {
                                // Clear reset change
                                if (ps2.change & hub::PS_C_RESET) != 0 {
                                    self.hub_clear_port_feature(hub_slot, ep0_ring, ep0_ring_phys, &mut ep0_enqueue, &mut ep0_cycle, port as u16, hub::C_PORT_RESET);
                                }

                                if (ps2.status & hub::PS_ENABLE) != 0 {
                                    delay_ms(100);  // Give device time to stabilize
                                    self.enumerate_hub_device(hub_slot, root_port, port as u32);
                                } else {
                                    uerror!("usbd", "hub_port_not_enabled"; port = port as u64);
                                }
                            }
                        } else {
                            uerror!("usbd", "hub_port_reset_failed"; port = port as u64);
                        }
                    } else {
                        uinfo!("usbd", "hub_port_disconnect"; port = port as u64);
                        // Clean up MSC device (hub child devices go through here)
                        self.cleanup_msc_device();
                    }
                }
            }
        }

        // Update ep0_enqueue and ep0_cycle in hub_device
        if let Some(ref mut hub) = self.hub_device {
            hub.ep0_enqueue = ep0_enqueue;
            hub.ep0_cycle = ep0_cycle;
        }

        // Re-queue the interrupt transfer for next status change
        self.queue_hub_status_transfer();
    }

    /// Set a port feature
    fn hub_set_port_feature(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, ep0_cycle: &mut bool, port: u16, feature: u16) -> bool {
        let setup = set_port_feature_setup(port, feature);
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
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
    fn hub_clear_port_feature(&mut self, slot_id: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, ep0_cycle: &mut bool, port: u16, feature: u16) -> bool {
        let setup = clear_port_feature_setup(port, feature);
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
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

    /// Debounce a hub port connection (Linux-style)
    /// Returns true if connection is stable, false if unstable/disconnected
    fn debounce_hub_port(
        &mut self,
        hub_slot: u32,
        ep0_ring_virt: *mut Trb,
        ep0_ring_phys: u64,
        ep0_enqueue: &mut usize,
        ep0_cycle: &mut bool,
        port: u16,
        buf_virt: u64,
        buf_phys: u64,
    ) -> bool {
        use usb_timing::*;

        let mut stable_time: u32 = 0;
        let mut total_time: u32 = 0;
        let mut last_connected = false;
        let mut first = true;

        while total_time < HUB_DEBOUNCE_TIMEOUT {
            let connected = if let Some(ps) = self.hub_get_port_status(
                hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
                port, buf_virt, buf_phys
            ) {
                (ps.status & hub::PS_CONNECTION) != 0
            } else {
                false
            };

            if first {
                last_connected = connected;
                first = false;
            }

            if connected == last_connected {
                stable_time += HUB_DEBOUNCE_STEP;
                if stable_time >= HUB_DEBOUNCE_STABLE {
                    return connected;
                }
            } else {
                // State changed - reset stability counter
                stable_time = 0;
                last_connected = connected;
            }

            delay_ms(HUB_DEBOUNCE_STEP);
            total_time += HUB_DEBOUNCE_STEP;
        }

        uwarn!("usbd", "hub_debounce_timeout"; port = port as u64);
        false
    }

    /// Enumerate devices connected through a hub
    /// Enumerate hub - mostly silent, only prints port count and connected devices
    fn enumerate_hub(&mut self, hub_slot: u32, hub_port: u32, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, ep0_cycle: &mut bool, device_ctx_virt: *mut DeviceContext) {
        // Allocate buffer for hub descriptor and port status
        let mut buf_phys: u64 = 0;
        let buf_virt = syscall::mmap_dma(4096, &mut buf_phys);
        if buf_virt < 0 {
            return;
        }

        // =====================================================================
        // Step 1: Get configuration descriptor to find interrupt endpoint
        // =====================================================================
        let mut int_ep_info: Option<(u8, u16, u8)> = None;

        // Get config descriptor (first 9 bytes to get total length)
        match self.control_transfer(
            hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
            0x80, usb_req::GET_DESCRIPTOR, (usb_req::DESC_CONFIGURATION << 8) as u16,
            0, buf_phys, 9
        ) {
            Some((cc, _)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
                invalidate_buffer(buf_virt as u64, 9);
                let config = unsafe { &*(buf_virt as *const ConfigurationDescriptor) };
                let total_len = config.total_length;

                if total_len > 9 && total_len <= 256 {
                    delay(10000);
                    // Get full config descriptor
                    match self.control_transfer(
                        hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
                        0x80, usb_req::GET_DESCRIPTOR, (usb_req::DESC_CONFIGURATION << 8) as u16,
                        0, buf_phys, total_len
                    ) {
                        Some((cc, _)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
                            invalidate_buffer(buf_virt as u64, total_len as usize);
                            // Parse to find interrupt IN endpoint
                            int_ep_info = self.parse_hub_config_descriptor(buf_virt as *const u8, total_len as usize);
                        }
                        _ => {}
                    }
                }
            }
            _ => {
                uerror!("usbd", "hub_config_desc_failed"; slot = hub_slot as u64);
            }
        }

        // =====================================================================
        // Step 2: Set configuration to activate hub
        // =====================================================================
        if !self.set_configuration(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, 1) {
            return;
        }

        // Set hub depth (required for USB3 hubs)
        // Depth 0 = directly connected to root hub
        let _ = self.control_transfer(
            hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
            0x20, hub::SET_HUB_DEPTH, 0, 0, 0, 0
        );

        // Get hub descriptor to find port count
        let hub_desc_len = match self.get_hub_descriptor(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, buf_phys) {
            Some(len) => len,
            None => return,
        };

        // Validate descriptor length before casting
        const SS_HUB_DESC_SIZE: usize = core::mem::size_of::<SsHubDescriptor>();
        if (hub_desc_len as usize) < SS_HUB_DESC_SIZE {
            uerror!("usbd", "hub_desc_too_short"; len = hub_desc_len as u64, expected = SS_HUB_DESC_SIZE as u64);
            return;
        }

        // Invalidate hub descriptor buffer after xHCI DMA write
        invalidate_buffer(buf_virt as u64, hub_desc_len as usize);
        let hub_desc = unsafe { &*(buf_virt as *const SsHubDescriptor) };
        let num_ports = hub_desc.num_ports;
        let pwr_time = hub_desc.pwr_on_2_pwr_good as u32 * 2;

        // Log hub detection with port count
        uinfo!("usbd", "hub_detected"; slot = hub_slot as u64, ports = num_ports as u64);

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
                        self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | ERDP_EHB);
                    }
                }
            }
        }

        // =====================================================================
        // Step 3: Set up hub interrupt endpoint for status change notifications
        // =====================================================================
        if let Some((int_ep_addr, int_max_packet, int_interval)) = int_ep_info {
            if !self.setup_hub_interrupt_endpoint(
                hub_slot, hub_port, num_ports,
                int_ep_addr, int_max_packet, int_interval,
                device_ctx_virt, ep0_ring_virt, ep0_ring_phys, *ep0_enqueue
            ) {
                uerror!("usbd", "hub_int_ep_setup_failed"; slot = hub_slot as u64);
            }
        }

        // =====================================================================
        // Step 4: Power on all ports and warm reset for USB 3.0 link training
        // =====================================================================
        for port in 1..=num_ports as u16 {
            self.hub_set_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, hub::PORT_POWER);
        }

        // Wait for power stabilization (use hub-specified time, minimum 100ms)
        let wait_time = (pwr_time as u32 * 2).max(100);
        delay_ms(wait_time);

        // Warm reset (BH_PORT_RESET) all ports - needed for USB 3.0 link establishment
        for port in 1..=num_ports as u16 {
            self.hub_set_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, hub::BH_PORT_RESET);
        }

        // Wait for warm reset completion by polling port status (max 500ms with backoff)
        // BH Reset change bit is bit 5 in port status change field
        const C_BH_RESET: u16 = 1 << 5;
        let buf_virt_u64 = buf_virt as u64;
        for port in 1..=num_ports as u16 {
            for attempt in 0..50 {  // 50 attempts * 10ms = 500ms max
                if let Some(ps) = self.hub_get_port_status(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, buf_virt_u64, buf_phys) {
                    if (ps.change & C_BH_RESET) != 0 {
                        // Warm reset complete - clear the change bit
                        self.hub_clear_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, hub::C_BH_PORT_RESET);
                        break;
                    }
                }
                delay_ms(10);
                if attempt == 49 {
                    // Timeout - clear change bit anyway in case we missed it
                    self.hub_clear_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, hub::C_BH_PORT_RESET);
                }
            }
        }

        // =====================================================================
        // Step 5: Wait for link training and do initial port poll
        // =====================================================================
        // USB 3.0 link training can take time - wait before checking ports
        delay_ms(2000);

        // Do initial port poll to find devices that connected during warm reset
        // The interrupt endpoint only reports NEW status changes
        // (buf_virt_u64 already declared above)
        for port in 1..=num_ports as u16 {
            if let Some(ps) = self.hub_get_port_status(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, buf_virt_u64, buf_phys) {
                // Clear any pending change bits
                if (ps.change & hub::PS_C_CONNECTION) != 0 {
                    self.hub_clear_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, hub::C_PORT_CONNECTION);
                }

                // If device connected, debounce then reset and enumerate
                if (ps.status & hub::PS_CONNECTION) != 0 {
                    // Debounce: ensure connection is stable before proceeding
                    if !self.debounce_hub_port(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, buf_virt_u64, buf_phys) {
                        continue;
                    }

                    // Reset with retry logic (Linux: PORT_RESET_TRIES = 5)
                    let mut enumerated = false;
                    for _reset_try in 0..usb_timing::PORT_RESET_TRIES {
                        if !self.hub_set_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, hub::PORT_RESET) {
                            delay_ms(100);
                            continue;
                        }

                        // Poll for reset completion (800ms timeout per Linux HUB_RESET_TIMEOUT)
                        let max_attempts = usb_timing::HUB_RESET_TIMEOUT / 10;
                        for _attempt in 0..max_attempts {
                            if let Some(ps2) = self.hub_get_port_status(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, buf_virt_u64, buf_phys) {
                                if (ps2.change & hub::PS_C_RESET) != 0 {
                                    self.hub_clear_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, hub::C_PORT_RESET);
                                    if (ps2.status & hub::PS_ENABLE) != 0 {
                                        delay_ms(usb_timing::POST_RESET_DELAY);
                                        self.enumerate_hub_device(hub_slot, hub_port, port as u32);
                                        enumerated = true;
                                    }
                                    break;
                                }
                            }
                            delay_ms(10);
                        }

                        if enumerated {
                            break;
                        }
                    }

                    if !enumerated {
                        uerror!("usbd", "hub_port_reset_exhausted"; port = port as u64);
                    }
                }
            }
        }

        // =====================================================================
        // Step 6: Update hub_device.ep0_enqueue and ep0_cycle with final values
        // =====================================================================
        // IMPORTANT: hub_device was stored earlier with stale ep0_enqueue/cycle values.
        // After all the PORT_POWER, BH_PORT_RESET, hub_get_port_status operations,
        // ep0_enqueue has advanced (and cycle may have toggled if ring wrapped).
        // We MUST update hub_device with the final values or daemon loop will fail.
        if let Some(ref mut hub) = self.hub_device {
            hub.ep0_enqueue = *ep0_enqueue;
            hub.ep0_cycle = *ep0_cycle;
        }

        // =====================================================================
        // Step 7: Queue interrupt transfer for future status changes
        // =====================================================================
        if self.hub_device.is_some() {
            self.queue_hub_status_transfer();
        }
    }

    /// Poll hub ports once (fallback when interrupt endpoint is not available)
    fn poll_hub_ports_once(&mut self, hub_slot: u32, root_port: u32, num_ports: u8, ep0_ring_virt: *mut Trb, ep0_ring_phys: u64, ep0_enqueue: &mut usize, ep0_cycle: &mut bool, buf_virt: u64, buf_phys: u64) {
        // Wait for devices to connect (USB 3.0 link training can take time)
        delay_ms(2000);

        for port in 1..=num_ports as u16 {
            if let Some(ps) = self.hub_get_port_status(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, buf_virt, buf_phys) {
                if (ps.status & hub::PS_CONNECTION) != 0 {
                    self.hub_clear_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, hub::C_PORT_CONNECTION);
                    if self.hub_set_port_feature(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, hub::PORT_RESET) {
                        delay_ms(100);
                        if let Some(ps2) = self.hub_get_port_status(hub_slot, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, port, buf_virt, buf_phys) {
                            if (ps2.status & hub::PS_ENABLE) != 0 {
                                self.enumerate_hub_device(hub_slot, root_port, port as u32);
                            }
                        }
                    }
                }
            }
        }
    }

    /// Quick hub port poll for daemon loop (no delays)
    /// Uses stored hub_device info to poll all hub ports for status changes
    fn poll_hub_ports_quick(&mut self) {
        // Extract hub info first to avoid borrow issues
        let (hub_slot, num_ports, root_port, ep0_ring, ep0_phys, mut ep0_enqueue, mut ep0_cycle, device_ctx) = {
            match self.hub_device.as_ref() {
                Some(h) => (h.slot_id, h.num_ports, h.hub_port, h.ep0_ring, h.ep0_ring_phys, h.ep0_enqueue, h.ep0_cycle, h.device_ctx),
                None => {
                    return;
                }
            }
        };

        // Debug: Check EP0 endpoint context state and xHCI operational state
        let usbcmd = self.op_read32(xhci_op::USBCMD);
        let usbsts = self.op_read32(xhci_op::USBSTS);
        let ep0_state: u32;
        let ep0_trdp: u64;
        unsafe {
            use usb::transfer::invalidate_buffer;
            invalidate_buffer(device_ctx as u64, 128);  // Invalidate slot + EP0 contexts
            // DeviceContext: SlotContext (32 bytes) + endpoints[31] (31 * 32 bytes)
            // EP0 context is at offset 32 (after SlotContext)
            let ep0_ctx = (device_ctx as *const u8).add(32) as *const u32;
            let dw0 = core::ptr::read_volatile(ep0_ctx);
            let dw2 = core::ptr::read_volatile(ep0_ctx.add(2));
            let dw3 = core::ptr::read_volatile(ep0_ctx.add(3));
            ep0_state = dw0 & 0x7;  // EP state is bits 0-2
            ep0_trdp = ((dw3 as u64) << 32) | (dw2 as u64);
        }
        let ep_state_name = match ep0_state {
            0 => "Disabled",
            1 => "Running",
            2 => "Halted",
            3 => "Stopped",
            4 => "Error",
            _ => "Unknown",
        };
        let _ = (usbcmd, usbsts, ep0_state, ep_state_name, ep0_trdp);  // Suppress warnings

        // Allocate a temp buffer for port status (4 bytes)
        let mut buf_phys: u64 = 0;
        let buf_virt = syscall::mmap_dma(4096, &mut buf_phys);
        if buf_virt < 0 {
            return;
        }
        let buf_virt_u64 = buf_virt as u64;

        for port in 1..=num_ports as u16 {
            let ps_result = self.hub_get_port_status(hub_slot, ep0_ring, ep0_phys, &mut ep0_enqueue, &mut ep0_cycle, port, buf_virt_u64, buf_phys);
            if ps_result.is_none() {
                continue;
            }
            let ps = ps_result.unwrap();

            // Only clear connection change bit (bit 0) for now - other bits may cause issues
            if (ps.change & 0x0001) != 0 {
                self.hub_clear_port_feature(hub_slot, ep0_ring, ep0_phys, &mut ep0_enqueue, &mut ep0_cycle, port, 16);
            }

            // Check for connection change
            if (ps.change & hub::PS_C_CONNECTION) != 0 {
                if (ps.status & hub::PS_CONNECTION) != 0 {
                    // Device connected - reset and enumerate
                    uinfo!("usbd", "hub_port_connect"; port = port as u64);
                    if self.hub_set_port_feature(hub_slot, ep0_ring, ep0_phys, &mut ep0_enqueue, &mut ep0_cycle, port, hub::PORT_RESET) {
                        delay_ms(50);
                        if let Some(ps2) = self.hub_get_port_status(hub_slot, ep0_ring, ep0_phys, &mut ep0_enqueue, &mut ep0_cycle, port, buf_virt_u64, buf_phys) {
                            if (ps2.status & hub::PS_ENABLE) != 0 {
                                self.enumerate_hub_device(hub_slot, root_port, port as u32);
                            }
                        }
                    }
                } else {
                    // Device disconnected
                    uinfo!("usbd", "hub_port_disconnect"; port = port as u64);
                    // Clean up MSC device connected through this hub port
                    self.cleanup_msc_device();
                }
            }
        }

        // Update stored ep0_enqueue and ep0_cycle
        if let Some(ref mut h) = self.hub_device {
            h.ep0_enqueue = ep0_enqueue;
            h.ep0_cycle = ep0_cycle;
        }

        // Free temp buffer
        syscall::munmap(buf_virt as u64, 4096);
    }

    /// Enumerate a device connected to a hub port
    fn enumerate_hub_device(&mut self, hub_slot: u32, root_port: u32, hub_port: u32) {
        // Enable a new slot for this device
        let slot_id = match self.enable_slot() {
            Some(id) => id,
            None => {
                uerror!("usbd", "hub_enable_slot_failed"; port = hub_port as u64);
                return;
            }
        };
        let _ = hub_slot;  // Suppress warning

        // Allocate context memory
        let mut ctx_phys: u64 = 0;
        let ctx_virt = syscall::mmap_dma(4096, &mut ctx_phys);
        if ctx_virt < 0 {
            return;
        }

        let input_ctx_virt = ctx_virt as *mut InputContext;
        let input_ctx_phys = ctx_phys;
        let device_ctx_virt = (ctx_virt as u64 + CTX_OFFSET_DEVICE) as *mut DeviceContext;
        let device_ctx_phys = ctx_phys + CTX_OFFSET_DEVICE;
        let ep0_ring_virt = (ctx_virt as u64 + CTX_OFFSET_EP0_RING) as *mut Trb;
        let ep0_ring_phys = ctx_phys + CTX_OFFSET_EP0_RING;

        // Initialize EP0 transfer ring (same as enumerate_device)
        unsafe {
            for i in 0..EP0_RING_SIZE {
                *ep0_ring_virt.add(i) = Trb::new();
            }
            let link = &mut *ep0_ring_virt.add(EP0_RING_USABLE);
            link.param = ep0_ring_phys;
            link.set_type(trb_type::LINK);
            link.control |= trb_ctrl::TC;  // Toggle Cycle
            // Flush EP0 ring to memory
            flush_buffer(ep0_ring_virt as u64, EP0_RING_SIZE * core::mem::size_of::<Trb>());
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

        // Address Device with retry logic (Linux: SET_ADDRESS_TRIES = 2)
        let mut addr_ok = false;
        for addr_try in 0..usb_timing::SET_ADDRESS_TRIES {
            if addr_try > 0 {
                delay_ms(100);  // Wait before retry
            }

            // Wait before addressing - device needs time after reset
            delay_ms(usb_timing::HUB_LONG_RESET_TIME);  // 200ms

            // Address Device command
            {
                let cmd_ring = match self.cmd_ring.as_mut() {
                    Some(r) => r,
                    None => {
                        syscall::munmap(ctx_virt as u64, 4096);
                        return;
                    }
                };
                let mut trb = Trb::new();
                trb.param = input_ctx_phys;
                trb.set_type(trb_type::ADDRESS_DEVICE);
                trb.control |= (slot_id & 0xFF) << 24;
                cmd_ring.enqueue(&trb);
                self.ring_doorbell(0, 0);
            }

            delay_ms(100);  // Wait for command processing

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
                self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp | ERDP_EHB);
            }

            match addr_result {
                Some(cc) if cc == trb_cc::SUCCESS => {
                    addr_ok = true;
                    break;
                }
                _ => {}
            }
        }

        if !addr_ok {
            uerror!("usbd", "hub_address_failed"; port = hub_port as u64);
            syscall::munmap(ctx_virt as u64, 4096);
            return;
        }

        // Wait for device ready (USB spec: 2ms after SET_ADDRESS)
        delay_ms(usb_timing::POST_ADDRESS_DELAY.max(10));

        // Get Device Descriptor with retry (Linux: GET_DESCRIPTOR_TRIES = 2)
        let mut desc_phys: u64 = 0;
        let desc_virt = syscall::mmap_dma(4096, &mut desc_phys);
        if desc_virt < 0 {
            // Note: ctx_virt is needed for the addressed device, don't free it here
            // The device is successfully addressed, just can't read descriptor
            return;
        }

        let mut dev_ep0_enqueue = 0usize;
        let mut dev_ep0_cycle = true;  // Cycle starts at 1 for new ring

        let mut desc_ok = false;
        for desc_try in 0..usb_timing::GET_DESCRIPTOR_TRIES {
            if desc_try > 0 {
                delay_ms(50);  // Brief delay before retry
            }

            match self.control_transfer(
                slot_id, ep0_ring_virt, ep0_ring_phys, &mut dev_ep0_enqueue, &mut dev_ep0_cycle,
                0x80, usb_req::GET_DESCRIPTOR, (usb_req::DESC_DEVICE << 8) as u16,
                0, desc_phys, 18
            ) {
                Some((cc, _len)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
                    let desc_ptr = desc_virt as *const u8;
                    let device_class = unsafe { *desc_ptr.add(4) };  // offset 4 = bDeviceClass
                    let vendor_id = unsafe {
                        let p = desc_ptr.add(8) as *const u16;
                        core::ptr::read_unaligned(p)
                    };
                    let product_id = unsafe {
                        let p = desc_ptr.add(10) as *const u16;
                        core::ptr::read_unaligned(p)
                    };
                    if device_class == msc::CLASS || device_class == 0 {
                        // Class 0 means class is defined at interface level (common for MSC)
                        uinfo!("usbd", "msc_detected"; slot = slot_id as u64, vendor = vendor_id as u64, product = product_id as u64);
                        self.configure_mass_storage(
                            slot_id, ep0_ring_virt, ep0_ring_phys, &mut dev_ep0_enqueue, &mut dev_ep0_cycle,
                            desc_virt as *mut u8, desc_phys, device_ctx_virt
                        );
                    } else {
                        uinfo!("usbd", "device_detected"; slot = slot_id as u64, class = device_class as u64, vendor = vendor_id as u64, product = product_id as u64);
                    }
                    desc_ok = true;
                    break;
                }
                _ => {}
            }
        }

        if !desc_ok {
            uerror!("usbd", "hub_descriptor_failed"; port = hub_port as u64);
        }

        // Free temporary descriptor buffer
        syscall::munmap(desc_virt as u64, 4096);
    }

    /// Configure a mass storage device
    fn configure_mass_storage(
        &mut self,
        slot_id: u32,
        ep0_ring_virt: *mut Trb,
        ep0_ring_phys: u64,
        ep0_enqueue: &mut usize,
        ep0_cycle: &mut bool,
        buf_virt: *mut u8,
        buf_phys: u64,
        device_ctx_virt: *mut DeviceContext,
    ) {
        // Get Configuration Descriptor (first 9 bytes to get total length)
        match self.control_transfer(
            slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
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
                        slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
                        0x80, usb_req::GET_DESCRIPTOR, (usb_req::DESC_CONFIGURATION << 8) as u16,
                        0, buf_phys, total_len
                    ) {
                        Some((cc, _)) if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET => {
                            // Invalidate full config descriptor
                            invalidate_buffer(buf_virt as u64, total_len as usize);
                            self.parse_config_descriptor(
                                slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle,
                                buf_virt, total_len as usize, device_ctx_virt
                            );
                        }
                        Some((cc, _)) => {
                            uerror!("usbd", "msc_config_failed"; slot = slot_id as u64, cc = cc as u64);
                        }
                        None => {
                            uerror!("usbd", "msc_config_timeout"; slot = slot_id as u64);
                        }
                    }
                } else {
                    uerror!("usbd", "msc_invalid_config_len"; slot = slot_id as u64, len = total_len as u64);
                }
            }
            Some((cc, _)) => {
                uerror!("usbd", "msc_config_failed"; slot = slot_id as u64, cc = cc as u64);
            }
            None => {
                uerror!("usbd", "msc_config_timeout"; slot = slot_id as u64);
            }
        }
    }

    /// Parse configuration descriptor and set up mass storage if found (silent)
    fn parse_config_descriptor(
        &mut self,
        slot_id: u32,
        ep0_ring_virt: *mut Trb,
        ep0_ring_phys: u64,
        ep0_enqueue: &mut usize,
        ep0_cycle: &mut bool,
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
                        if is_msc_bbb(iface.interface_class, iface.interface_subclass, iface.interface_protocol) {
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
            if self.set_configuration(slot_id, ep0_ring_virt, ep0_ring_phys, ep0_enqueue, ep0_cycle, config_value) {
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
        if bulk_out_ring_virt < 0 {
            syscall::munmap(bulk_in_ring_virt as u64, 4096);
            return;
        }

        // Initialize transfer rings with Link TRBs (BULK_RING_SIZE entries)
        unsafe {
            // Initialize bulk IN ring
            let in_ring = bulk_in_ring_virt as *mut Trb;
            for i in 0..BULK_RING_SIZE {
                *in_ring.add(i) = Trb::new();
            }
            // Set up Link TRB at last position
            let in_link = &mut *in_ring.add(BULK_RING_SIZE - 1);
            in_link.param = bulk_in_ring_phys;  // Points back to start
            in_link.set_type(trb_type::LINK);
            in_link.control |= trb_ctrl::TC;  // Toggle Cycle
            // Flush entire ring using bulk flush
            flush_buffer(in_ring as u64, BULK_RING_SIZE * core::mem::size_of::<Trb>());

            // Initialize bulk OUT ring
            let out_ring = bulk_out_ring_virt as *mut Trb;
            for i in 0..BULK_RING_SIZE {
                *out_ring.add(i) = Trb::new();
            }
            // Set up Link TRB at last position
            let out_link = &mut *out_ring.add(BULK_RING_SIZE - 1);
            out_link.param = bulk_out_ring_phys;  // Points back to start
            out_link.set_type(trb_type::LINK);
            out_link.control |= trb_ctrl::TC;  // Toggle Cycle
            // Flush entire ring using bulk flush
            flush_buffer(out_ring as u64, BULK_RING_SIZE * core::mem::size_of::<Trb>());
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
        if input_ctx_virt < 0 {
            syscall::munmap(bulk_in_ring_virt as u64, 4096);
            syscall::munmap(bulk_out_ring_virt as u64, 4096);
            return;
        }

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
                        let erdp = event_ring.erdp() | ERDP_EHB;
                        self.rt_write64(xhci_rt::IR0 + xhci_ir::ERDP, erdp);
                        break;
                    }
                }
                delay(1000);
            }
        }

        // Free temporary input context
        syscall::munmap(input_ctx_virt as u64, 4096);

        if !config_ok {
            syscall::munmap(bulk_in_ring_virt as u64, 4096);
            syscall::munmap(bulk_out_ring_virt as u64, 4096);
            return;
        }

        // Invalidate device context after xHCI wrote to it
        invalidate_buffer(device_ctx_virt as u64, core::mem::size_of::<DeviceContext>());

        // Allocate data buffer for SCSI commands
        let mut data_phys: u64 = 0;
        let data_virt = syscall::mmap_dma(4096, &mut data_phys);
        if data_virt < 0 {
            // Endpoints configured but no data buffer - cleanup rings
            syscall::munmap(bulk_in_ring_virt as u64, 4096);
            syscall::munmap(bulk_out_ring_virt as u64, 4096);
            return;
        }

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
        let slot_id = ctx.slot_id;
        let out_dci = ctx.bulk_out_dci;

        // Poll for completion with proper event matching (CBW is fast, no need for IRQ)
        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..100 {
                // Drain events looking for matching transfer completion
                for _ in 0..32 {
                    if let Some(evt) = event_ring.dequeue() {
                        // Update ERDP for all events
                        let erdp = event_ring.erdp() | ERDP_EHB;
                        unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }

                        let evt_type = evt.get_type();
                        if evt_type == trb_type::TRANSFER_EVENT {
                            // Only accept events matching our slot and endpoint
                            if evt.matches_transfer(slot_id, out_dci) {
                                let cc = evt.event_completion_code();
                                return cc == trb_cc::SUCCESS;
                            }
                            // Non-matching transfer event: discard and continue
                        }
                    } else {
                        break; // No more events
                    }
                }
                syscall::yield_now();
            }
        }
        false
    }

    /// Wait for a transfer event via IRQ, matching specific slot, endpoint, and optionally TRB
    /// Drains the event ring looking for a matching transfer event.
    /// Non-matching events are consumed but ignored (prevents BOT desync).
    /// If expected_trb_phys != 0, also validates that the event's TRB pointer matches.
    /// Returns (result, residue)
    fn wait_transfer_irq(&mut self, slot_id: u32, dci: u32, expected_trb_phys: u64, timeout_ms: u32) -> (TransferResult, u64) {
        let mac_base = self.xhci.mmio_base();
        let rt_base = self.xhci.rt_offset();
        let op_base = self.xhci.op_offset();
        let erdp_addr = mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64;

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
            let irq_received = self.wait_for_irq(timeout_ms);

            // IMPORTANT: Always check event ring, even on timeout!
            // Drain ALL events looking for a matching transfer event.
            if let Some(ref mut event_ring) = self.event_ring {
                // Drain up to 32 events per IRQ (handle stale event buildup)
                for _ in 0..32 {
                    if let Some(evt) = event_ring.dequeue() {
                        // Always update ERDP for consumed event
                        let erdp = event_ring.erdp() | ERDP_EHB;
                        unsafe {
                            core::ptr::write_volatile(erdp_addr as *mut u64, erdp);
                        }

                        let evt_type = evt.get_type();

                        if evt_type == trb_type::TRANSFER_EVENT {
                            // Check if this event matches our expected slot, endpoint, and TRB
                            let matches = if expected_trb_phys != 0 {
                                evt.matches_transfer_exact(slot_id, dci, expected_trb_phys)
                            } else {
                                evt.matches_transfer(slot_id, dci)
                            };
                            if matches {
                                let cc = evt.event_completion_code();
                                let residue = evt.event_residue();
                                return match cc {
                                    x if x == trb_cc::SUCCESS => (TransferResult::Success, residue as u64),
                                    x if x == trb_cc::SHORT_PACKET => (TransferResult::ShortPacket, residue as u64),
                                    _ => (TransferResult::Error(cc), residue as u64),
                                };
                            }
                            // Non-matching transfer event: discard and continue draining
                        }
                        // Non-transfer event (PORT_STATUS_CHANGE, etc): discard
                    } else {
                        break; // No more events in ring
                    }
                }
            }

            // If wait_for_irq timed out and no matching event found, return timeout
            if !irq_received {
                return (TransferResult::Timeout, 0);
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
        // Calculate TRB physical address for exact matching
        let trb_phys = ctx.bulk_in_ring_phys + (idx * 16) as u64;
        unsafe {
            let trb = &mut *ctx.bulk_in_ring.add(idx);
            trb.param = phys_addr;
            trb.status = length as u32;
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;  // IOC, cycle
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }

        // Ring doorbell
        self.ring_doorbell(ctx.slot_id, ctx.bulk_in_dci);

        // Wait for transfer via IRQ - matching IN endpoint and exact TRB
        let (result, residue) = self.wait_transfer_irq(ctx.slot_id, ctx.bulk_in_dci, trb_phys, 5000);

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
        // Create TRB for bulk IN with proper cycle bit
        let (idx, cycle) = ctx.advance_in();
        // Calculate TRB physical address for exact matching
        let trb_phys = ctx.bulk_in_ring_phys + (idx * 16) as u64;
        unsafe {
            let trb = &mut *ctx.bulk_in_ring.add(idx);
            trb.param = target_phys;  // Direct to client's DMA buffer
            trb.status = length as u32;
            trb.control = (trb_type::NORMAL << 10) | (1 << 5) | cycle;  // IOC, cycle
            ctx.flush_buffer(trb as *const _ as u64, 16);
        }

        // Ring doorbell
        self.ring_doorbell(ctx.slot_id, ctx.bulk_in_dci);

        // Wait for transfer via IRQ - matching IN endpoint and exact TRB
        let (result, residue) = self.wait_transfer_irq(ctx.slot_id, ctx.bulk_in_dci, trb_phys, 5000);

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
            // CBW failed - recover and return error
            self.recover_endpoints(ctx);
            return (TransferResult::Error(0), None);
        }

        // Data phase - goes directly to client's DMA buffer
        if data_length > 0 {
            let (result, _bytes) = self.receive_bulk_in_dma(ctx, target_phys, data_length as usize);
            if result != TransferResult::Success && result != TransferResult::ShortPacket {
                // Data phase failed - recover and return error
                self.recover_endpoints(ctx);
                return (result, None);
            }
        }

        // CSW phase (uses internal buffer) - validates signature, tag, status
        let (result, csw) = self.receive_csw_irq(ctx, tag);
        if csw.is_none() {
            // CSW failed or invalid - recover
            self.recover_endpoints(ctx);
        }
        (result, csw)
    }

    /// Receive CSW (Command Status Wrapper) on bulk IN with strict validation
    /// Returns (result, CSW) - CSW is only returned if valid (signature + tag match + valid status)
    fn receive_csw_irq(&mut self, ctx: &mut BulkContext, expected_tag: u32) -> (TransferResult, Option<Csw>) {
        let (result, _bytes) = self.receive_bulk_in_irq(ctx, CSW_OFFSET, 13);

        if result == TransferResult::Success || result == TransferResult::ShortPacket {
            // Read CSW from buffer
            unsafe {
                let csw_ptr = ctx.data_buf.add(CSW_OFFSET) as *const Csw;
                ctx.invalidate_buffer(csw_ptr as u64, 13);
                let csw = core::ptr::read_volatile(csw_ptr);

                // Strict CSW validation per BOT spec
                // Copy packed fields to avoid unaligned references
                let sig = csw.signature;
                let tag = csw.tag;
                let status = csw.status;

                // 1. Signature must be 'USBS'
                if sig != msc::CSW_SIGNATURE {
                    uerror!("usbd", "csw_bad_signature"; sig = sig as u64);
                    return (TransferResult::Error(0), None);
                }

                // 2. Tag must match CBW tag
                if tag != expected_tag {
                    uerror!("usbd", "csw_tag_mismatch"; expected = expected_tag as u64, got = tag as u64);
                    return (TransferResult::Error(0), None);
                }

                // 3. Status must be valid (0=passed, 1=failed, 2=phase error)
                if status > msc::CSW_STATUS_PHASE_ERROR {
                    uerror!("usbd", "csw_invalid_status"; status = status as u64);
                    return (TransferResult::Error(0), None);
                }

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
            // CBW failed - recover and return error
            self.recover_endpoints(ctx);
            return (TransferResult::Error(0), None);
        }

        // Data phase (if any)
        if data_length > 0 {
            let (result, _bytes) = self.receive_bulk_in_irq(ctx, DATA_OFFSET, data_length as usize);
            if result != TransferResult::Success && result != TransferResult::ShortPacket {
                // Data phase failed - recover and return error
                self.recover_endpoints(ctx);
                return (result, None);
            }
        }

        // CSW phase - validates signature, tag, status
        let (result, csw) = self.receive_csw_irq(ctx, tag);
        if csw.is_none() {
            // CSW failed or invalid - recover
            self.recover_endpoints(ctx);
        }
        (result, csw)
    }

    /// Execute a complete SCSI command with data OUT: CBW -> Data OUT -> CSW
    fn scsi_command_out(&mut self, ctx: &mut BulkContext, cmd: &[u8], data: &[u8]) -> (TransferResult, Option<Csw>) {
        let tag = ctx.next_tag();
        let data_length = data.len() as u32;

        // Build CBW (direction = false for OUT)
        let cbw = Cbw::new(tag, data_length, false, 0, cmd);

        // Send CBW
        if !self.send_cbw(ctx, &cbw) {
            // CBW failed - recover and return error
            self.recover_endpoints(ctx);
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
            let slot_id = ctx.slot_id;
            let out_dci = ctx.bulk_out_dci;

            // Poll for completion with proper event matching
            if let Some(ref mut event_ring) = self.event_ring {
                let mut completed = false;
                for _ in 0..100 {
                    // Drain events looking for matching transfer completion
                    for _ in 0..32 {
                        if let Some(evt) = event_ring.dequeue() {
                            // Update ERDP for all events
                            let erdp = event_ring.erdp() | ERDP_EHB;
                            unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }

                            let evt_type = evt.get_type();
                            if evt_type == trb_type::TRANSFER_EVENT {
                                // Only accept events matching our slot and endpoint
                                if evt.matches_transfer(slot_id, out_dci) {
                                    let cc = evt.event_completion_code();
                                    if cc != trb_cc::SUCCESS && cc != trb_cc::SHORT_PACKET {
                                        uerror!("usbd", "transfer_error"; cc = cc as u64);
                                        // Data phase failed - recover and return error
                                        self.recover_endpoints(ctx);
                                        return (TransferResult::Error(cc), None);
                                    }
                                    completed = true;
                                    break;
                                }
                                // Non-matching transfer event: discard and continue
                            }
                        } else {
                            break; // No more events
                        }
                    }
                    if completed {
                        break;
                    }
                    syscall::yield_now();
                }
                if !completed {
                    uerror!("usbd", "data_transfer_timeout"; idx = idx as u64);
                    // Data phase timed out - recover and return error
                    self.recover_endpoints(ctx);
                    return (TransferResult::Error(0), None);
                }
            }
        }

        // CSW phase - validates signature, tag, status
        let (result, csw) = self.receive_csw_irq(ctx, tag);
        if csw.is_none() {
            // CSW failed or invalid - recover
            self.recover_endpoints(ctx);
        }
        (result, csw)
    }

    /// Execute a complete BOT command using the state machine for tracking and recovery.
    /// This is the preferred method for production use as it provides proper error
    /// handling based on USB BOT spec section 5.3.
    ///
    /// Returns TransportResult indicating the overall command status.
    fn bot_command(
        &mut self,
        ctx: &mut BulkContext,
        sm: &mut BotStateMachine,
        cmd: &[u8],
        is_data_in: bool,
        data_length: u32,
        target_phys: Option<u64>,  // For DMA reads, None for internal buffer
    ) -> TransportResult {
        // Check if device is ready
        if !sm.is_ready() {
            uerror!("usbd", "bot_device_not_ready");
            return TransportResult::Error;
        }

        // Get tag and start command
        let tag = sm.next_tag();
        ctx.tag = tag;
        sm.start_command();

        // Build CBW
        let cbw = Cbw::new(tag, data_length, is_data_in, 0, cmd);

        // === CBW Phase ===
        if !self.send_cbw(ctx, &cbw) {
            let action = sm.cbw_stalled();
            return self.handle_recovery_action(ctx, sm, action);
        }

        // Transition to data or CSW phase
        let has_data_in = is_data_in && data_length > 0;
        let has_data_out = !is_data_in && data_length > 0;
        sm.cbw_sent(has_data_in, has_data_out);

        // === Data Phase ===
        if has_data_in {
            let (result, _bytes) = if let Some(phys) = target_phys {
                self.receive_bulk_in_dma(ctx, phys, data_length as usize)
            } else {
                self.receive_bulk_in_irq(ctx, DATA_OFFSET, data_length as usize)
            };

            match result {
                TransferResult::Success | TransferResult::ShortPacket => {
                    sm.data_complete();
                }
                TransferResult::Error(cc) if cc == trb_cc::STALL => {
                    let action = sm.data_stalled(true);
                    if let RecoveryAction::ClearHalt { is_bulk_in } = action {
                        if self.clear_endpoint_halt(ctx, is_bulk_in) {
                            sm.halt_cleared();
                        } else {
                            return self.handle_recovery_action(ctx, sm, RecoveryAction::BotReset);
                        }
                    }
                }
                _ => {
                    let action = sm.transfer_error();
                    return self.handle_recovery_action(ctx, sm, action);
                }
            }
        } else if has_data_out {
            // Data OUT phase not implemented with state machine yet
            // For now, fall through to CSW
            sm.data_complete();
        }

        // === CSW Phase ===
        self.receive_csw_with_sm(ctx, sm, tag)
    }

    /// Receive CSW with state machine tracking
    fn receive_csw_with_sm(
        &mut self,
        ctx: &mut BulkContext,
        sm: &mut BotStateMachine,
        expected_tag: u32,
    ) -> TransportResult {
        let max_retries = 3;

        for _retry in 0..max_retries {
            let (result, _bytes) = self.receive_bulk_in_irq(ctx, CSW_OFFSET, 13);

            match result {
                TransferResult::Success | TransferResult::ShortPacket => {
                    // Read and validate CSW
                    unsafe {
                        let csw_ptr = ctx.data_buf.add(CSW_OFFSET) as *const Csw;
                        ctx.invalidate_buffer(csw_ptr as u64, 13);
                        let csw = core::ptr::read_volatile(csw_ptr);

                        let sig = csw.signature;
                        let tag = csw.tag;
                        let status = csw.status;

                        // Validate signature
                        if sig != msc::CSW_SIGNATURE {
                            uerror!("usbd", "bot_csw_bad_signature"; sig = sig as u64);
                            let action = sm.csw_invalid();
                            match action {
                                RecoveryAction::RetryCSW => continue,
                                _ => return self.handle_recovery_action(ctx, sm, action),
                            }
                        }

                        // Validate tag
                        if tag != expected_tag {
                            uerror!("usbd", "bot_csw_tag_mismatch"; expected = expected_tag as u64, got = tag as u64);
                            let action = sm.csw_invalid();
                            match action {
                                RecoveryAction::RetryCSW => continue,
                                _ => return self.handle_recovery_action(ctx, sm, action),
                            }
                        }

                        // Process CSW status
                        return sm.csw_received(CswStatus::from(status));
                    }
                }
                TransferResult::Error(cc) if cc == trb_cc::STALL => {
                    // CSW stalled - clear halt and retry
                    let action = sm.csw_stalled();
                    if let RecoveryAction::ClearHalt { is_bulk_in } = action {
                        if self.clear_endpoint_halt(ctx, is_bulk_in) {
                            sm.halt_cleared();
                            continue;
                        }
                    }
                    return self.handle_recovery_action(ctx, sm, RecoveryAction::BotReset);
                }
                _ => {
                    let action = sm.transfer_error();
                    return self.handle_recovery_action(ctx, sm, action);
                }
            }
        }

        // Too many retries
        self.handle_recovery_action(ctx, sm, RecoveryAction::BotReset)
    }

    /// Handle recovery action from state machine
    fn handle_recovery_action(
        &mut self,
        ctx: &mut BulkContext,
        sm: &mut BotStateMachine,
        action: RecoveryAction,
    ) -> TransportResult {
        match action {
            RecoveryAction::None => TransportResult::Good,
            RecoveryAction::Retry | RecoveryAction::RetryCSW => {
                // Caller should retry
                TransportResult::Error
            }
            RecoveryAction::ClearHalt { is_bulk_in } => {
                if self.clear_endpoint_halt(ctx, is_bulk_in) {
                    sm.halt_cleared();
                    TransportResult::Error // Caller should retry
                } else {
                    self.handle_recovery_action(ctx, sm, RecoveryAction::BotReset)
                }
            }
            RecoveryAction::BotReset => {
                uwarn!("usbd", "bot_class_reset");
                let success = self.bot_class_reset(ctx);
                let next_action = sm.bot_reset_complete(success);
                if next_action == RecoveryAction::None {
                    TransportResult::Error // Reset succeeded, but command failed
                } else {
                    self.handle_recovery_action(ctx, sm, next_action)
                }
            }
            RecoveryAction::PortReset => {
                uerror!("usbd", "bot_port_reset_required");
                sm.on_disconnect();
                TransportResult::Error
            }
            RecoveryAction::ReEnumerate => {
                uwarn!("usbd", "bot_reenumerate_required");
                TransportResult::Error
            }
            RecoveryAction::GiveUp => {
                uerror!("usbd", "bot_device_unrecoverable");
                sm.on_disconnect();
                TransportResult::Error
            }
        }
    }

    /// Clear endpoint halt (CLEAR_FEATURE ENDPOINT_HALT)
    fn clear_endpoint_halt(&mut self, ctx: &mut BulkContext, is_bulk_in: bool) -> bool {
        let ep_addr = if is_bulk_in { ctx.bulk_in_addr } else { ctx.bulk_out_addr };

        let result = self.control_transfer(
            ctx.slot_id, ctx.ep0_ring, ctx.ep0_ring_phys, &mut ctx.ep0_enqueue, &mut ctx.ep0_cycle,
            0x02, 1, 0, ep_addr as u16, 0, 0
        );

        match result {
            Some((cc, _)) if cc == trb_cc::SUCCESS => true,
            _ => {
                uerror!("usbd", "clear_halt_failed"; ep = ep_addr as u64);
                false
            }
        }
    }

    /// BOT class reset (class-specific mass storage reset)
    fn bot_class_reset(&mut self, ctx: &mut BulkContext) -> bool {
        let result = self.control_transfer(
            ctx.slot_id, ctx.ep0_ring, ctx.ep0_ring_phys, &mut ctx.ep0_enqueue, &mut ctx.ep0_cycle,
            0x21, 0xFF, 0, ctx.interface_num as u16, 0, 0
        );

        let reset_ok = match result {
            Some((cc, _)) if cc == trb_cc::SUCCESS => true,
            _ => {
                uerror!("usbd", "bot_class_reset_failed"; interface = ctx.interface_num as u64);
                false
            }
        };

        if reset_ok {
            delay_ms(10);
            // Clear both endpoints
            let out_ok = self.clear_endpoint_halt(ctx, false);
            let in_ok = self.clear_endpoint_halt(ctx, true);
            out_ok && in_ok
        } else {
            false
        }
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

        let (_result, csw) = self.scsi_command_in(ctx, &cmd, data_length);

        if let Some(csw) = csw {
            if csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED {
                // Use library parser
                unsafe {
                    let resp_ptr = ctx.data_buf.add(DATA_OFFSET);
                    ctx.invalidate_buffer(resp_ptr as u64, 8);

                    let data = core::slice::from_raw_parts(resp_ptr, 8);
                    if let Some((last_lba, block_size)) = scsi::parse_read_capacity_10(data) {
                        return Some((last_lba, block_size));
                    }
                }
            }
        }

        // Get sense data to see what went wrong (silent)
        self.scsi_request_sense(ctx);
        None
    }

    /// SCSI REQUEST_SENSE - get error information after a failed command
    fn scsi_request_sense(&mut self, ctx: &mut BulkContext) {
        let (cmd, data_length) = scsi::build_request_sense(18);  // Standard 18-byte sense data

        let (_result, csw) = self.scsi_command_in(ctx, &cmd, data_length);

        if let Some(csw) = csw {
            if csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED {
                unsafe {
                    let resp_ptr = ctx.data_buf.add(DATA_OFFSET);
                    ctx.invalidate_buffer(resp_ptr as u64, 18);
                    let data = core::slice::from_raw_parts(resp_ptr, 18);

                    // Parse sense data for error logging
                    let sense_key = data[2] & 0x0F;
                    let asc = data[12];  // Additional Sense Code
                    let ascq = data[13]; // Additional Sense Code Qualifier

                    // Log non-trivial sense errors
                    if sense_key != 0 && sense_key != 6 {  // Not NO_SENSE or UNIT_ATTENTION
                        uwarn!("usbd", "scsi_sense"; key = sense_key as u64, asc = asc as u64, ascq = ascq as u64);
                    }
                }
            }
        }
    }

    /// SCSI TEST_UNIT_READY - check if device is ready
    /// Returns true if device is ready, false otherwise
    fn scsi_test_unit_ready_ctx(&mut self, ctx: &mut BulkContext) -> bool {
        // Use library CDB builder
        let (cmd, data_length) = scsi::build_test_unit_ready();

        let (_result, csw) = self.scsi_command_in(ctx, &cmd, data_length);

        if let Some(csw) = csw {
            csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED
        } else {
            false
        }
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

    /// Run SCSI tests - initializes device and stores capacity info
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

        // Query device transfer limits via VPD page 0xB0 (silent)
        let _ = self.scsi_inquiry_vpd_block_limits(ctx);

        // Read capacity with retry (USB devices can be slow after wake)
        let mut capacity = None;
        for _attempt in 0..3 {
            if let Some(cap) = self.scsi_read_capacity_10(ctx) {
                capacity = Some(cap);
                break;
            }
            delay_ms(500);
        }

        if let Some((last_lba, block_size)) = capacity {
            // Calculate size
            let block_count = last_lba as u64 + 1;
            let size_bytes = block_count * block_size as u64;
            let size_mb = size_bytes / (1024 * 1024);

            // Verify we can read
            if self.scsi_read_10(ctx, 0, 1).is_some() {
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
                    ep0_cycle: ctx.ep0_cycle,
                    // Port info for recovery - filled later if needed
                    root_port: 0,
                    hub_slot: None,
                    hub_port: 0,
                    // Initialize state machine as ready
                    state_machine: {
                        let mut sm = BotStateMachine::new();
                        sm.on_enumerated();
                        sm
                    },
                });
                uinfo!("usbd", "msc_ready"; slot = ctx.slot_id as u64, capacity_mb = size_mb, block_size = block_size as u64);

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
    /// NOTE: This is disabled by default - uncomment call in run_scsi_tests to enable
    #[allow(dead_code)]
    fn run_performance_test(&mut self, ctx: &mut BulkContext, _block_count: u64) {
        // Allocate a larger DMA buffer for performance testing (64KB)
        let mut perf_buf_phys: u64 = 0;
        let perf_buf_virt = syscall::mmap_dma(65536, &mut perf_buf_phys);
        if perf_buf_virt < 0 {
            return;
        }

        // Test parameters - use LBA 1000 (safe zone, avoids sector 0 and partition tables)
        let test_lba = 1000u32;
        let transfer_sizes = [1u16, 4, 8, 16, 32, 64];
        let mut _max_reliable_size = 1u16;

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
                flush_buffer(perf_buf_virt as u64, bytes_per_transfer);
            }

            // Write test: 100 transfers
            let write_count = 100u32;
            let mut success = 0u32;
            for i in 0..write_count {
                let lba = test_lba + i * sectors_per_transfer as u32;
                if self.perf_write(ctx, lba, perf_buf_phys, bytes_per_transfer) {
                    success += 1;
                } else {
                    self.recover_endpoints(ctx);
                    break;
                }
            }

            if success == write_count {
                _max_reliable_size = sectors_per_transfer;
            }

            // Read test: 100 transfers
            let mut _success = 0u32;
            for i in 0..write_count {
                let lba = test_lba + i * sectors_per_transfer as u32;
                if self.perf_read(ctx, lba, sectors_per_transfer, perf_buf_phys, bytes_per_transfer) {
                    _success += 1;
                } else {
                    self.recover_endpoints(ctx);
                    break;
                }
            }
        }
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

        // Wait for data completion (single wait for both TRBs) - matching OUT endpoint
        let timeout = 100 + (len / 512) as u32 * 10;
        if !self.wait_transfer_complete(ctx.slot_id, ctx.bulk_out_dci, timeout) { return false; }

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

        // Wait for CSW - matching IN endpoint
        if !self.wait_transfer_complete(ctx.slot_id, ctx.bulk_in_dci, 100) { return false; }

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
        // Wait for CBW - matching OUT endpoint
        if !self.wait_transfer_complete(ctx.slot_id, ctx.bulk_out_dci, 100) { return false; }

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

        // Wait for CSW completion (covers both data and CSW) - matching IN endpoint
        // Use high spin count for reads since device needs to fetch from flash
        let timeout = 100 + (len / 512) as u32 * 10;
        if !self.wait_transfer_complete_read(ctx.slot_id, ctx.bulk_in_dci, timeout) { return false; }

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
    /// Returns true if recovery succeeded, false if device needs port reset
    fn recover_endpoints(&mut self, ctx: &mut BulkContext) -> bool {
        uwarn!("usbd", "endpoint_recovery_start"; slot = ctx.slot_id as u64);

        // Drain any pending events first
        let erdp_addr = self.xhci.mmio_base() + self.xhci.rt_offset() as u64
            + (xhci_rt::IR0 + xhci_ir::ERDP) as u64;
        if let Some(ref mut event_ring) = self.event_ring {
            for _ in 0..50 {
                if let Some(_evt) = event_ring.dequeue() {
                    let erdp = event_ring.erdp() | ERDP_EHB;
                    unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }
                } else {
                    break;
                }
            }
        }

        // Step 1: USB BOT Reset (class-specific reset)
        // bmRequestType=0x21 (class, interface), bRequest=0xFF, wIndex=interface
        let _ = self.control_transfer(
            ctx.slot_id, ctx.ep0_ring, ctx.ep0_ring_phys, &mut ctx.ep0_enqueue, &mut ctx.ep0_cycle,
            0x21, 0xFF, 0, ctx.interface_num as u16, 0, 0
        );
        delay_ms(10);

        // Step 2: CLEAR_FEATURE(ENDPOINT_HALT) for bulk OUT
        // bmRequestType=0x02 (endpoint), bRequest=1 (CLEAR_FEATURE), wValue=0 (HALT), wIndex=endpoint
        let result = self.control_transfer(
            ctx.slot_id, ctx.ep0_ring, ctx.ep0_ring_phys, &mut ctx.ep0_enqueue, &mut ctx.ep0_cycle,
            0x02, 1, 0, ctx.bulk_out_addr as u16, 0, 0
        );
        let clear_out_ok = matches!(result, Some((cc, _)) if cc == trb_cc::SUCCESS);

        // Step 3: CLEAR_FEATURE(ENDPOINT_HALT) for bulk IN
        let result = self.control_transfer(
            ctx.slot_id, ctx.ep0_ring, ctx.ep0_ring_phys, &mut ctx.ep0_enqueue, &mut ctx.ep0_cycle,
            0x02, 1, 0, ctx.bulk_in_addr as u16, 0, 0
        );
        let clear_in_ok = matches!(result, Some((cc, _)) if cc == trb_cc::SUCCESS);

        // If both Clear HALT failed, device needs port reset - don't waste time with xHCI recovery
        if !clear_out_ok && !clear_in_ok {
            uerror!("usbd", "endpoint_recovery_failed"; slot = ctx.slot_id as u64);
            return false;
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
        unsafe {
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
            out_link.control |= trb_ctrl::TC;  // Toggle Cycle
            // Link TRB cycle = 0 (opposite of producer cycle=1)
            flush_buffer(out_ring as u64, BULK_RING_SIZE * core::mem::size_of::<Trb>());

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
            in_link.control |= trb_ctrl::TC;  // Toggle Cycle
            // Link TRB cycle = 0 (opposite of producer cycle=1)
            flush_buffer(in_ring as u64, BULK_RING_SIZE * core::mem::size_of::<Trb>());
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
        delay_ms(100);
        let mut ready = false;
        for _attempt in 0..10 {
            if self.test_unit_ready_quick(ctx) {
                ready = true;
                break;
            }
            delay_ms(100);
        }

        if ready {
            uinfo!("usbd", "endpoint_recovery_ok"; slot = ctx.slot_id as u64);
            true
        } else {
            uerror!("usbd", "endpoint_recovery_failed"; slot = ctx.slot_id as u64);
            false
        }
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
        // Wait for CBW - matching OUT endpoint
        if !self.wait_transfer_complete(ctx.slot_id, ctx.bulk_out_dci, 50) { return false; }

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
        // Wait for CSW - matching IN endpoint
        if !self.wait_transfer_complete(ctx.slot_id, ctx.bulk_in_dci, 50) { return false; }

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
                    let erdp = event_ring.erdp() | ERDP_EHB;
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
    /// slot_id: expected slot ID for matching events
    /// dci: expected endpoint DCI for matching events
    fn wait_transfer_complete_ex(&mut self, slot_id: u32, dci: u32, spin_count: u32, max_polls: u32) -> bool {
        let erdp_addr = self.xhci.mmio_base() + self.xhci.rt_offset() as u64
            + (xhci_rt::IR0 + xhci_ir::ERDP) as u64;

        if let Some(ref mut event_ring) = self.event_ring {
            // First, spin locally without yielding (USB should be fast)
            for _ in 0..spin_count {
                // Drain all available events looking for our match
                for _ in 0..32 {
                    if let Some(evt) = event_ring.dequeue() {
                        let erdp = event_ring.erdp() | ERDP_EHB;
                        unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }

                        if evt.get_type() == trb_type::TRANSFER_EVENT {
                            // Only accept events matching our slot and endpoint
                            if evt.matches_transfer(slot_id, dci) {
                                let cc = evt.event_completion_code();
                                return cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET;
                            }
                            // Non-matching transfer event: discard and continue draining
                        }
                    } else {
                        break; // No more events in ring
                    }
                }
                core::hint::spin_loop();
            }

            // If still waiting, use slower polling with yields
            for _ in 0..max_polls {
                // Drain all available events looking for our match
                for _ in 0..32 {
                    if let Some(evt) = event_ring.dequeue() {
                        let erdp = event_ring.erdp() | ERDP_EHB;
                        unsafe { core::ptr::write_volatile(erdp_addr as *mut u64, erdp); }

                        if evt.get_type() == trb_type::TRANSFER_EVENT {
                            // Only accept events matching our slot and endpoint
                            if evt.matches_transfer(slot_id, dci) {
                                let cc = evt.event_completion_code();
                                return cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET;
                            }
                            // Non-matching transfer event: discard and continue draining
                        }
                    } else {
                        break; // No more events in ring
                    }
                }
                syscall::yield_now();
            }
        }
        false
    }

    /// Wait for transfer completion with default spin count (for writes/CBW)
    fn wait_transfer_complete(&mut self, slot_id: u32, dci: u32, max_polls: u32) -> bool {
        self.wait_transfer_complete_ex(slot_id, dci, 1000, max_polls)
    }

    /// Wait for transfer completion with high spin count (for reads from flash)
    /// Reads need more spin time because device must fetch data from flash storage
    fn wait_transfer_complete_read(&mut self, slot_id: u32, dci: u32, max_polls: u32) -> bool {
        self.wait_transfer_complete_ex(slot_id, dci, 10000, max_polls)
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
            ep0_cycle: msc.ep0_cycle,
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
            ep0_cycle: msc.ep0_cycle,
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

    /// Read blocks to internal buffer (for copy-based transfer)
    /// Data ends up in the MSC device's internal buffer at DATA_OFFSET
    fn block_read_internal(&mut self, lba: u64, count: u32) -> Option<usize> {
        if count == 0 || count > 64 {
            return None;
        }

        let mut msc = self.msc_device.take()?;

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
            ep0_cycle: msc.ep0_cycle,
        };

        // Build and execute READ(10) to internal buffer
        let (cmd, data_length) = scsi::build_read_10(lba as u32, count as u16, 512);
        let (result, csw) = self.scsi_command_in(&mut ctx, &cmd, data_length);

        // Update ring state
        msc.out_enqueue = ctx.out_enqueue;
        msc.in_enqueue = ctx.in_enqueue;
        msc.out_cycle = ctx.out_cycle;
        msc.in_cycle = ctx.in_cycle;
        msc.tag = ctx.tag;
        msc.ep0_enqueue = ctx.ep0_enqueue;

        self.msc_device = Some(msc);

        if let Some(csw) = csw {
            if csw.signature == msc::CSW_SIGNATURE && csw.status == msc::CSW_STATUS_PASSED &&
               (result == TransferResult::Success || result == TransferResult::ShortPacket) {
                let actual_bytes = (data_length - csw.data_residue) as usize;
                return Some(actual_bytes);
            }
        }

        None
    }

    /// Get pointer to internal data buffer (for copying after block_read_internal)
    fn get_data_ptr(&self) -> Option<*const u8> {
        self.msc_device.as_ref().map(|msc| {
            // Invalidate cache first so we see what DMA wrote
            unsafe {
                let ptr = msc.data_buf.add(DATA_OFFSET);
                invalidate_buffer(ptr as u64, 32768); // Invalidate up to 64 blocks
                ptr as *const u8
            }
        })
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
            ep0_cycle: msc.ep0_cycle,
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
                uerror!("usbd", "ring_create_failed");
                return false;
            }
        };

        // Grant client permission to map the shared memory
        if !ring.allow(self.client_pid) {
            uerror!("usbd", "ring_allow_failed"; pid = self.client_pid as u64);
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
    fn process_requests(&mut self, driver: &mut UsbDriver) -> u32 {
        let ring = match &mut self.ring {
            Some(r) => r,
            None => return 0,
        };

        let mut processed = 0u32;

        // Process all pending requests
        while let Some(req) = ring.next_request() {
            let resp = Self::handle_request_static(driver, ring, &req);
            ring.complete(&resp);
            processed += 1;
        }

        // Notify client if we processed any requests
        if processed > 0 {
            ring.notify();
        }

        processed
    }

    /// Handle a single block request (static to avoid borrow issues)
    fn handle_request_static(driver: &mut UsbDriver, ring: &mut BlockRing, req: &BlockRequest) -> BlockResponse {
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
                Self::handle_read_static(driver, ring, req)
            }
            BlockRequest::CMD_WRITE => {
                Self::handle_write_static(driver, ring, req)
            }
            _ => {
                BlockResponse::error(req.tag, -38) // ENOSYS
            }
        }
    }

    /// Handle a read request (static)
    fn handle_read_static(driver: &mut UsbDriver, ring: &mut BlockRing, req: &BlockRequest) -> BlockResponse {
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

        // Zero-copy DMA: read directly into shared memory ring buffer
        // Shared memory uses non-cacheable attributes, so no cache ops needed
        let target_phys = ring.data_phys() + buf_offset as u64;

        match driver.block_read_dma(req.lba, req.count, target_phys) {
            Some(bytes_read) => {
                BlockResponse::ok(req.tag, bytes_read as u32)
            },
            None => {
                uerror!("usbd", "block_read_failed"; lba = req.lba, count = req.count as u64);
                BlockResponse::error(req.tag, -5) // EIO
            }
        }
    }

    /// Handle a write request
    fn handle_write_static(driver: &mut UsbDriver, ring: &BlockRing, req: &BlockRequest) -> BlockResponse {
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
    uinfo!("usbd", "init_start");

    // NOTE: We delay port registration until MSC device is ready.
    // This way clients (fatfs) block on port_connect() until we can serve them.

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
        uerror!("usbd", "board_pre_init_failed");
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

                // Controller initialized successfully
                driver = Some(d);
                active_config = Some(&controllers[idx as usize]);
                break;
            }
        }
    }

    let (mut driver, config) = match (driver, active_config) {
        (Some(d), Some(c)) => (d, c),
        _ => {
            uerror!("usbd", "no_controller");
            syscall::exit(1);
        }
    };

    // Test command ring
    driver.send_noop();
    delay_ms(10);

    // Register for IRQ BEFORE polling events (so we don't miss any)
    // Open with O_NONBLOCK so wait_for_irq can implement proper timeout
    let irq_url = format_irq_url(config.irq);
    let irq_fd = syscall::scheme_open(&irq_url, O_NONBLOCK);
    if irq_fd >= 0 {
        driver.set_irq_fd(irq_fd);
    }

    // === Event-Driven Device Detection ===
    // Wait for PORT_STATUS_CHANGE events from xHCI
    // Devices already connected at boot will generate CSC events

    // Process initial events (CSC events for already-connected devices)
    let initial_ports = driver.poll_events();
    if initial_ports != 0 {
        driver.enumerate_pending_ports(initial_ports);
    }

    // Check if port 2 has anything (USB 2.0 portion of hub?)
    if driver.xhci.max_ports() >= 2 {
        let raw = driver.xhci.read_portsc(1);  // Port 2 (0-indexed)
        let status = ParsedPortsc::from_raw(raw);
        if status.connected && driver.enumerated_ports & 2 == 0 {
            driver.enumerate_port(2);
        }
    }

    // Resync event ring after enumeration - fix any cycle bit desync
    driver.resync_event_ring();

    // Re-queue hub interrupt transfer (may have been lost in stale events)
    if let Some(ref mut hub) = driver.hub_device {
        // Reset transfer_pending since the old transfer was lost in stale events
        hub.transfer_pending = false;
    }
    if driver.hub_device.is_some() {
        driver.queue_hub_status_transfer();
    }

    // Wait for USB device enumeration (no timeout - keep monitoring)
    let mut wait_count = 0;
    while driver.msc_device.is_none() {
        wait_count += 1;

        // Every 1 second, poll hub port status via control transfers
        if wait_count % 10 == 0 {
            // Check if we need to retry hub interrupt submission
            if let Some(ref hub) = driver.hub_device {
                if hub.irq_retry_pending {
                    driver.requeue_hub_interrupt();
                }
            }
            driver.poll_hub_ports_quick();
        }

        delay_ms(100);

        // Poll events and enumerate any new devices
        let ports_to_enum = driver.poll_events();
        if ports_to_enum != 0 {
            driver.enumerate_pending_ports(ports_to_enum);
        }
    }

    // Now that MSC is ready, register the "usb" port
    // Clients (fatfs) will have been blocking on port_connect() until now
    let port_result = syscall::port_register(b"usb");
    if port_result < 0 {
        if port_result == -17 {
            uerror!("usbd", "already_running");
        } else {
            uerror!("usbd", "port_register_failed"; err = port_result as u64);
        }
        if irq_fd >= 0 {
            syscall::close(irq_fd as u32);
        }
        syscall::exit(1);
    }
    let usb_port = port_result as u32;
    uinfo!("usbd", "port_registered");

    // Create timer handle for polling (Handle API)
    // 10ms polling interval for xHCI events
    const POLL_INTERVAL_NS: u64 = 10_000_000; // 10ms
    let timer_handle = match handle_timer_create() {
        Ok(h) => h,
        Err(_) => {
            uerror!("usbd", "timer_create_failed");
            syscall::exit(1);
        }
    };

    // Set timer as recurring
    if handle_timer_set(timer_handle, POLL_INTERVAL_NS, POLL_INTERVAL_NS).is_err() {
        uerror!("usbd", "timer_set_failed");
        syscall::exit(1);
    }

    // Wrap the listen channel as a handle (Handle API)
    let port_handle = match handle_wrap_channel(usb_port) {
        Ok(h) => h,
        Err(_) => {
            uerror!("usbd", "port_wrap_failed");
            syscall::exit(1);
        }
    };

    // Initialize client tracking
    const MAX_CLIENTS: usize = 4;
    let mut clients: [Option<BlockServer>; MAX_CLIENTS] = [None, None, None, None];
    let mut num_clients = 0usize;

    // Accept any immediately pending connections
    loop {
        let accept_result = syscall::port_accept(usb_port);
        if accept_result <= 0 {
            break;
        }
        let client_channel = accept_result as u32;
        let mut client = BlockServer::new(client_channel);
        if client.try_handshake() && num_clients < MAX_CLIENTS {
            for i in 0..MAX_CLIENTS {
                if clients[i].is_none() {
                    clients[i] = Some(client);
                    num_clients += 1;
                    uinfo!("usbd", "client_connected"; count = num_clients as u64);
                    break;
                }
            }
        }
    }

    // Enter daemon loop directly (devd supervises us, no need to daemonize)
    uinfo!("usbd", "daemon_running");

    // Resync event ring before daemon loop - drain any pending events and update ERDP
    driver.resync_event_ring();

    let mut heartbeat_counter = 0u32;

    // Event-driven main loop (Handle API)
    let requests = [
        WaitRequest::new(timer_handle, WaitFilter::Timer),
        WaitRequest::new(port_handle, WaitFilter::Readable),
    ];
    let mut results = [WaitResult::empty(); 2];

    loop {
        // Wait for timer or client connection (blocking)
        let count = match handle_wait(&requests, &mut results, u64::MAX) {
            Ok(n) => n,
            Err(_) => {
                syscall::yield_now();
                continue;
            }
        };

        // Process each ready handle
        for result in &results[..count] {
            if result.handle.raw() == timer_handle.raw() {
                // Timer fired - poll xHCI events
                heartbeat_counter += 1;

                // Reset heartbeat counter every ~100s
                if heartbeat_counter >= 10000 {
                    heartbeat_counter = 0;
                }

                // Poll xHCI events every timer tick for responsive hotplug
                let ports_to_enum = driver.poll_events();
                if ports_to_enum != 0 {
                    driver.enumerate_pending_ports(ports_to_enum);
                }

                // Poll hub ports every ~1 second (100 iterations) for hotplug detection
                if heartbeat_counter % 100 == 0 && heartbeat_counter > 0 {
                    // Check if we need to retry hub interrupt submission
                    if let Some(ref hub) = driver.hub_device {
                        if hub.irq_retry_pending {
                            driver.requeue_hub_interrupt();
                        }
                    }
                    driver.poll_hub_ports_quick();
                }

                // Process ring requests from all ready clients
                for i in 0..MAX_CLIENTS {
                    if let Some(ref mut client) = clients[i] {
                        if client.is_ready() {
                            client.process_requests(&mut driver);
                        }
                    }
                }
            } else if result.handle.raw() == port_handle.raw() {
                // New client connection ready
                let accept_result = syscall::port_accept(usb_port);
                if accept_result > 0 && num_clients < MAX_CLIENTS {
                    let client_channel = accept_result as u32;

                    // Create BlockServer and complete handshake
                    let mut client = BlockServer::new(client_channel);
                    if client.try_handshake() {
                        // Add to clients array
                        for i in 0..MAX_CLIENTS {
                            if clients[i].is_none() {
                                clients[i] = Some(client);
                                num_clients += 1;
                                uinfo!("usbd", "client_connected"; count = num_clients as u64);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    // Note: loop is infinite, this is unreachable
    // If we ever need graceful shutdown, add signal handling
}
