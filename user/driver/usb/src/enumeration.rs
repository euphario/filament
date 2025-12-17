//! USB Device Enumeration Helpers
//!
//! This module provides types and helpers for USB device enumeration.
//! The actual enumeration logic remains in usbd due to tight coupling
//! with hardware state (command ring, event ring, MMIO).

use crate::trb::{Trb, trb_type};
use crate::usb::{SlotContext, EndpointContext, InputControlContext};

/// Slot state values (from xHCI spec)
pub mod slot_state {
    pub const DISABLED: u32 = 0;
    pub const DEFAULT: u32 = 1;
    pub const ADDRESSED: u32 = 2;
    pub const CONFIGURED: u32 = 3;
}

/// Endpoint state values (from xHCI spec)
pub mod endpoint_state {
    pub const DISABLED: u32 = 0;
    pub const RUNNING: u32 = 1;
    pub const HALTED: u32 = 2;
    pub const STOPPED: u32 = 3;
    pub const ERROR: u32 = 4;
}

/// Endpoint type values for endpoint context
pub mod endpoint_type {
    pub const NOT_VALID: u32 = 0;
    pub const ISOCH_OUT: u32 = 1;
    pub const BULK_OUT: u32 = 2;
    pub const INTERRUPT_OUT: u32 = 3;
    pub const CONTROL: u32 = 4;
    pub const ISOCH_IN: u32 = 5;
    pub const BULK_IN: u32 = 6;
    pub const INTERRUPT_IN: u32 = 7;
}

/// Device speed values for slot context
pub mod device_speed {
    pub const FULL: u32 = 1;
    pub const LOW: u32 = 2;
    pub const HIGH: u32 = 3;
    pub const SUPER: u32 = 4;
    pub const SUPER_PLUS: u32 = 5;
}

/// Build an ENABLE_SLOT command TRB
pub fn build_enable_slot_trb() -> Trb {
    let mut trb = Trb::new();
    trb.set_type(trb_type::ENABLE_SLOT);
    trb
}

/// Build a DISABLE_SLOT command TRB
pub fn build_disable_slot_trb(slot_id: u32) -> Trb {
    Trb {
        param: 0,
        status: 0,
        control: (trb_type::DISABLE_SLOT << 10) | (slot_id << 24) | 1,
    }
}

/// Build an ADDRESS_DEVICE command TRB
pub fn build_address_device_trb(input_ctx_phys: u64, slot_id: u32, bsr: bool) -> Trb {
    let bsr_bit = if bsr { 1 << 9 } else { 0 };
    Trb {
        param: input_ctx_phys,
        status: 0,
        control: (trb_type::ADDRESS_DEVICE << 10) | bsr_bit | (slot_id << 24) | 1,
    }
}

/// Build a CONFIGURE_ENDPOINT command TRB
pub fn build_configure_endpoint_trb(input_ctx_phys: u64, slot_id: u32, deconfigure: bool) -> Trb {
    let dc_bit = if deconfigure { 1 << 9 } else { 0 };
    Trb {
        param: input_ctx_phys,
        status: 0,
        control: (trb_type::CONFIGURE_ENDPOINT << 10) | dc_bit | (slot_id << 24) | 1,
    }
}

/// Build an EVALUATE_CONTEXT command TRB
pub fn build_evaluate_context_trb(input_ctx_phys: u64, slot_id: u32) -> Trb {
    Trb {
        param: input_ctx_phys,
        status: 0,
        control: (trb_type::EVALUATE_CONTEXT << 10) | (slot_id << 24) | 1,
    }
}

/// Build a RESET_ENDPOINT command TRB
pub fn build_reset_endpoint_trb(slot_id: u32, endpoint_id: u32, preserve_state: bool) -> Trb {
    let tsp_bit = if preserve_state { 1 << 9 } else { 0 };
    Trb {
        param: 0,
        status: 0,
        control: (trb_type::RESET_ENDPOINT << 10) | tsp_bit | (endpoint_id << 16) | (slot_id << 24) | 1,
    }
}

/// Build a STOP_ENDPOINT command TRB
pub fn build_stop_endpoint_trb(slot_id: u32, endpoint_id: u32, suspend: bool) -> Trb {
    let sp_bit = if suspend { 1 << 23 } else { 0 };
    Trb {
        param: 0,
        status: 0,
        control: (trb_type::STOP_ENDPOINT << 10) | sp_bit | (endpoint_id << 16) | (slot_id << 24) | 1,
    }
}

/// Build a SET_TR_DEQUEUE_POINTER command TRB
pub fn build_set_tr_dequeue_trb(slot_id: u32, endpoint_id: u32, dequeue_ptr: u64, cycle: bool) -> Trb {
    let cycle_bit = if cycle { 1 } else { 0 };
    Trb {
        param: (dequeue_ptr & !0xF) | cycle_bit,
        status: 0,
        control: (trb_type::SET_TR_DEQUEUE << 10) | (endpoint_id << 16) | (slot_id << 24) | 1,
    }
}

/// Build a NO_OP command TRB
pub fn build_noop_trb() -> Trb {
    let mut trb = Trb::new();
    trb.set_type(trb_type::NOOP_CMD);
    trb
}

/// Initialize a slot context for device enumeration
///
/// # Arguments
/// * `slot` - Pointer to slot context to initialize
/// * `speed` - Device speed (use device_speed constants)
/// * `route_string` - Route string for the device
/// * `root_hub_port` - Root hub port number (1-indexed)
/// * `context_entries` - Number of context entries (1 = slot only, 2 = slot + EP0, etc.)
/// * `hub_slot` - Parent hub slot ID (0 if directly connected to root)
/// * `hub_port` - Port number on parent hub (0 if directly connected to root)
pub fn init_slot_context(
    slot: &mut SlotContext,
    speed: u32,
    route_string: u32,
    root_hub_port: u32,
    context_entries: u32,
    hub_slot: u32,
    hub_port: u32,
) {
    // Clear the context
    *slot = SlotContext::default();

    // DW0: Route String (bits 0-19) | Speed (bits 20-23) | Context Entries (bits 27-31)
    slot.dw0 = (route_string & 0xFFFFF) | ((speed & 0xF) << 20) | ((context_entries & 0x1F) << 27);

    // DW1: Root Hub Port Number (bits 16-23)
    slot.dw1 = (root_hub_port & 0xFF) << 16;

    // DW2: Parent hub info for devices behind a hub
    if hub_slot != 0 {
        slot.dw2 = ((hub_slot & 0xFF) << 0) | ((hub_port & 0xFF) << 8);
    }
}

/// Initialize EP0 (control endpoint) context
///
/// # Arguments
/// * `ep` - Pointer to endpoint context to initialize
/// * `max_packet_size` - Maximum packet size (8, 16, 32, 64, or 512 for SS)
/// * `tr_dequeue_ptr` - Physical address of transfer ring (with cycle bit in bit 0)
pub fn init_ep0_context(ep: &mut EndpointContext, max_packet_size: u32, tr_dequeue_ptr: u64) {
    // Clear the context
    *ep = EndpointContext::default();

    // DW1: EP Type (Control = 4) | Max Packet Size | Error Count = 3
    ep.dw1 = (3 << 1) |  // CErr = 3 (max retries)
             (endpoint_type::CONTROL << 3) |
             ((max_packet_size & 0xFFFF) << 16);

    // DW2-3: TR Dequeue Pointer (64-bit, must include DCS bit)
    ep.dw2 = (tr_dequeue_ptr & 0xFFFFFFFF) as u32;
    ep.dw3 = (tr_dequeue_ptr >> 32) as u32;

    // DW4: Average TRB Length (8 for control, can be tuned)
    ep.dw4 = 8;
}

/// Initialize a bulk endpoint context
///
/// # Arguments
/// * `ep` - Pointer to endpoint context to initialize
/// * `ep_type` - Endpoint type (BULK_IN or BULK_OUT)
/// * `max_packet_size` - Maximum packet size
/// * `max_burst` - Maximum burst size (0 for USB 2.0)
/// * `tr_dequeue_ptr` - Physical address of transfer ring (with cycle bit)
pub fn init_bulk_endpoint_context(
    ep: &mut EndpointContext,
    ep_type: u32,
    max_packet_size: u32,
    max_burst: u32,
    tr_dequeue_ptr: u64,
) {
    *ep = EndpointContext::default();

    // DW1: EP Type | Max Packet Size | Max Burst | CErr = 3
    ep.dw1 = (3 << 1) |  // CErr = 3
             ((ep_type & 0x7) << 3) |
             ((max_burst & 0xFF) << 8) |
             ((max_packet_size & 0xFFFF) << 16);

    // DW2-3: TR Dequeue Pointer
    ep.dw2 = (tr_dequeue_ptr & 0xFFFFFFFF) as u32;
    ep.dw3 = (tr_dequeue_ptr >> 32) as u32;

    // DW4: Average TRB Length (use max packet size as estimate)
    ep.dw4 = max_packet_size.min(3072);
}

/// Initialize input control context for ADDRESS_DEVICE
///
/// Sets up the Add Context flags for slot context and EP0
pub fn init_input_control_for_address(icc: &mut InputControlContext) {
    *icc = InputControlContext::default();
    // Add slot context (bit 0) and EP0 (bit 1)
    icc.add_flags = 0x03;
    icc.drop_flags = 0;
}

/// Initialize input control context for CONFIGURE_ENDPOINT
///
/// # Arguments
/// * `add_eps` - Bitmask of endpoints to add (bit 0 = slot, bit 1 = EP0, bit 2 = EP1 OUT, etc.)
/// * `drop_eps` - Bitmask of endpoints to drop
pub fn init_input_control_for_configure(icc: &mut InputControlContext, add_eps: u32, drop_eps: u32) {
    *icc = InputControlContext::default();
    icc.add_flags = add_eps;
    icc.drop_flags = drop_eps;
}

/// Calculate endpoint context index (DCI) from endpoint address
///
/// USB endpoint address format: bit 7 = direction (1=IN), bits 3-0 = endpoint number
/// DCI = 2 * ep_num + direction (where IN=1, OUT=0)
/// DCI 0 is reserved, DCI 1 is EP0 (bidirectional)
pub fn endpoint_address_to_dci(ep_addr: u8) -> u32 {
    let ep_num = (ep_addr & 0x0F) as u32;
    let is_in = (ep_addr & 0x80) != 0;

    if ep_num == 0 {
        1  // EP0 is always DCI 1
    } else {
        2 * ep_num + if is_in { 1 } else { 0 }
    }
}

/// Calculate max packet size based on device speed
pub fn default_max_packet_size(speed: u32) -> u32 {
    match speed {
        device_speed::LOW => 8,
        device_speed::FULL => 64,
        device_speed::HIGH => 64,
        device_speed::SUPER | device_speed::SUPER_PLUS => 512,
        _ => 8,
    }
}
