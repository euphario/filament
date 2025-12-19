//! USB driver library for MT7988A SSUSB
//!
//! This crate provides a layered USB driver architecture:
//!
//! ## Layer Architecture (bottom to top)
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                 Class Drivers (100% portable)               │
//! │            MSC, HID, CDC - pure USB protocol                │
//! ├─────────────────────────────────────────────────────────────┤
//! │                   USB Core (100% portable)                  │
//! │         Enumeration, hubs, transfers - USB standard         │
//! ├─────────────────────────────────────────────────────────────┤
//! │                 xHCI Driver (100% portable)                 │
//! │       Rings, doorbells, events - xHCI is a standard         │
//! ├─────────────────────────────────────────────────────────────┤
//! │            SoC USB Wrapper (SoC-specific)                   │
//! │      MT7988A: IPPC registers, port control                  │
//! ├─────────────────────────────────────────────────────────────┤
//! │               PHY Driver (Board-specific)                   │
//! │      T-PHY configuration, host mode setup                   │
//! ├─────────────────────────────────────────────────────────────┤
//! │              Board Config (Board-specific)                  │
//! │      GPIO, power rails, controller selection                │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Modules
//!
//! - `xhci` - Pure xHCI controller (NO vendor code)
//! - `phy` - PHY driver trait and implementations
//! - `soc` - SoC USB wrapper trait and implementations
//! - `board` - Board configuration trait and implementations

#![no_std]

// =============================================================================
// Modular Architecture
// =============================================================================

/// Pure xHCI implementation (100% portable, NO vendor code)
pub mod xhci;

/// PHY driver abstraction (board-specific)
pub mod phy;

/// SoC USB wrapper abstraction (SoC-specific)
pub mod soc;

/// Board configuration (board-specific)
pub mod board;

// =============================================================================
// Core USB Types and Helpers
// =============================================================================

/// TRB (Transfer Request Block) structures
pub mod trb;

/// Ring management (command, transfer, event rings)
pub mod ring;

/// USB transfer helpers (setup packets, cache maintenance)
pub mod transfer;

/// USB descriptors and types
pub mod usb;

/// USB hub support
pub mod hub;

/// Device enumeration helpers
pub mod enumeration;

/// Mass Storage Class (MSC) and SCSI
pub mod msc;

/// IPC protocol definitions (ring buffer based)
pub mod protocol;

/// Block device client (for userspace drivers like fatfs)
pub mod block_client;

/// MMIO region helpers
pub mod mmio;

/// MT7988A hardware addresses and constants
pub mod consts;

// =============================================================================
// Re-exports for convenience
// =============================================================================

// Core types
pub use trb::{Trb, trb_type, trb_cc, trb_ctrl};
pub use ring::{Ring, EventRing, ErstEntry, Dcbaa, RING_SIZE, EP0_RING_SIZE, EP0_RING_USABLE, BULK_RING_SIZE, BULK_RING_USABLE};
pub use mmio::{MmioRegion, format_mmio_url, format_hex, delay_ms, delay, print_hex64, print_hex32, print_hex8};

// xHCI types and helpers
pub use xhci::{
    // Register definitions
    cap as xhci_cap, op as xhci_op, rt as xhci_rt, ir as xhci_ir, port as xhci_port,
    usbcmd, usbsts, portsc, iman,
    // Controller
    Controller as XhciController, XhciCaps,
    // Port status parsing
    ParsedPortsc, PortLinkState, PortSpeed,
    // Event helpers
    event_completion_code, event_slot_id, event_endpoint_id, event_port_id,
    event_trb_pointer, event_transfer_length, event_is_short_packet,
    completion_code, doorbell,
};

// USB types
pub use usb::{
    usb_req, hub as usb_hub, ep_type,
    SlotContext, EndpointContext, InputControlContext, InputContext, DeviceContext,
    HubDescriptor, SsHubDescriptor, PortStatus,
    DeviceDescriptor, ConfigurationDescriptor, InterfaceDescriptor, EndpointDescriptor,
};

// MSC/SCSI
pub use msc::{
    msc as msc_const, scsi, sense_key,
    Cbw, Csw, BulkContext, TransferResult,
    InquiryResponse, ReadCapacity10Response, SenseData, BlockLimits,
    CBW_OFFSET, CSW_OFFSET, DATA_OFFSET,
    RING_USABLE,
};

// Transfer helpers
pub use transfer::{
    SetupPacket, Direction, ControlStage, TransferType,
    build_setup_trb, build_data_trb, build_status_trb,
    build_link_trb, build_normal_trb,
    flush_trb, flush_trb_range, dsb, isb,
    flush_cache_line, invalidate_cache_line,
    flush_buffer, invalidate_buffer,
    // Timing constants
    DELAY_POST_RESET_US, DELAY_POST_ADDRESS_US, DELAY_POLL_INTERVAL_US,
    DELAY_DOWNSTREAM_DEVICE_US, DELAY_DIRECT_DEVICE_US, DELAY_POST_CONFIG_US,
    DELAY_INTER_CMD_US, POLL_MAX_DIRECT, POLL_MAX_DOWNSTREAM, POLL_MAX_DESCRIPTOR,
    // Register bit constants
    ERDP_EHB, USBSTS_EINT, USBSTS_PCD,
};

// Hub helpers
pub use hub::{
    port_feature, port_status, port_change,
    get_hub_descriptor_setup, get_port_status_setup,
    set_port_feature_setup, clear_port_feature_setup, set_hub_depth_setup,
    parse_ss_hub_descriptor, HubInfo, OverCurrentMode,
    ParsedPortStatus, DeviceSpeed,
};

// Enumeration helpers
pub use enumeration::{
    slot_state, endpoint_state, endpoint_type, device_speed,
    build_enable_slot_trb, build_disable_slot_trb, build_address_device_trb,
    build_configure_endpoint_trb, build_evaluate_context_trb,
    build_reset_endpoint_trb, build_stop_endpoint_trb, build_set_tr_dequeue_trb,
    build_noop_trb,
    init_slot_context, init_ep0_context, init_bulk_endpoint_context,
    init_input_control_for_address, init_input_control_for_configure,
    endpoint_address_to_dci, default_max_packet_size,
    // Memory layout constants
    CTX_OFFSET_INPUT, CTX_OFFSET_DEVICE, CTX_OFFSET_EP0_RING,
    XHCI_OFFSET_ERST, XHCI_OFFSET_CMD_RING, XHCI_OFFSET_EVT_RING, XHCI_MEM_SIZE,
};

// Block device client (ring buffer based)
pub use block_client::BlockClient;

// Hardware constants
pub use consts::*;

// SoC and PHY types
pub use soc::{SocUsb, SocError, PortCount};
pub use soc::mt7988a::{Mt7988aSoc, ControllerId, addrs as mt7988a};
pub use phy::{PhyDriver, PhyError};
pub use phy::mt7988a::Mt7988aTphy;
pub use board::{Board, BoardError};
pub use board::bpi_r4::BpiR4;
