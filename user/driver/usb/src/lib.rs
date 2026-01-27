//! USB driver library
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
//! │      GenericSoc (no-op), MT7988A: IPPC registers            │
//! ├─────────────────────────────────────────────────────────────┤
//! │               PHY Driver (Board-specific)                   │
//! │      T-PHY configuration, host mode setup                   │
//! ├─────────────────────────────────────────────────────────────┤
//! │              Board Config (Board-specific)                  │
//! │      GPIO, power rails, controller selection                │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Features
//!
//! - `mt7988a` - MediaTek MT7988A SoC wrapper, PHY, and BPI-R4 board
//! - `block-client` - Block device client (requires ring buffer protocol)
//! - `usbd` - Everything needed for usbd on MT7988A (combines mt7988a + block-client)
//!
//! ## Modules
//!
//! - `xhci` - Pure xHCI controller (NO vendor code)
//! - `soc` - SoC USB wrapper trait and GenericSoc
//! - `phy` - PHY driver trait (implementations behind features)
//! - `board` - Board configuration trait (implementations behind features)

#![no_std]
#![allow(dead_code)]  // USB constants and helpers for future use

// =============================================================================
// Modular Architecture (always available)
// =============================================================================

/// Pure xHCI implementation (100% portable, NO vendor code)
pub mod xhci;

/// SoC USB wrapper abstraction - GenericSoc always available
pub mod soc;

/// PHY driver abstraction (implementations behind features)
pub mod phy;

/// Board configuration (implementations behind features)
pub mod board;

// =============================================================================
// Core USB Types and Helpers (always available)
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

/// MMIO region helpers
pub mod mmio;

/// MT7988A hardware addresses and constants
#[cfg(feature = "mt7988a")]
pub mod consts;

// =============================================================================
// Feature-gated modules
// =============================================================================

/// Block device client (for userspace drivers like fatfs)
#[cfg(feature = "block-client")]
pub mod block_client;

// =============================================================================
// Re-exports for convenience
// =============================================================================

// Core types (always available)
pub use trb::{Trb, trb_type, trb_cc, trb_ctrl};
pub use ring::{Ring, EventRing, ErstEntry, Dcbaa, RING_SIZE, EP0_RING_SIZE, EP0_RING_USABLE, BULK_RING_SIZE, BULK_RING_USABLE};
pub use mmio::{MmioRegion, DmaPool, delay_ms, delay_us, format_mmio_url, format_hex, delay, print_hex64, print_hex32, print_hex8};

// xHCI types and helpers (always available - xHCI is a standard)
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

// USB types (always available - USB is a standard)
pub use usb::{
    usb_req, hub as usb_hub, ep_type,
    SlotContext, EndpointContext, InputControlContext, InputContext, DeviceContext,
    HubDescriptor, SsHubDescriptor, PortStatus,
    DeviceDescriptor, ConfigurationDescriptor, InterfaceDescriptor, EndpointDescriptor,
};

// MSC/SCSI (always available - MSC is a USB class standard)
pub use msc::{
    msc as msc_const, scsi, sense_key,
    Cbw, Csw, BulkContext, TransferResult,
    InquiryResponse, ReadCapacity10Response, SenseData, BlockLimits,
    CBW_OFFSET, CSW_OFFSET, DATA_OFFSET,
    RING_USABLE,
};

// Transfer helpers (always available)
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

// Hub helpers (always available)
pub use hub::{
    port_feature, port_status, port_change,
    get_hub_descriptor_setup, get_port_status_setup,
    set_port_feature_setup, clear_port_feature_setup, set_hub_depth_setup,
    parse_ss_hub_descriptor, HubInfo, OverCurrentMode,
    ParsedPortStatus, DeviceSpeed,
};

// Enumeration helpers (always available)
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

// SoC traits (always available) + GenericSoc
pub use soc::{SocUsb, SocError, PortCount, GenericSoc};

// PHY traits (always available)
pub use phy::{PhyDriver, PhyError};

// Board traits (always available)
pub use board::{Board, BoardError};

// =============================================================================
// Feature-gated re-exports
// =============================================================================

// Block device client (ring buffer based)
#[cfg(feature = "block-client")]
pub use block_client::BlockClient;

// Hardware constants (MT7988A specific)
#[cfg(feature = "mt7988a")]
pub use consts::*;

// MT7988A SoC and PHY types
#[cfg(feature = "mt7988a")]
pub use soc::mt7988a::{Mt7988aSoc, ControllerId, addrs as mt7988a};
#[cfg(feature = "mt7988a")]
pub use phy::mt7988a::Mt7988aTphy;
#[cfg(feature = "mt7988a")]
pub use board::bpi_r4::BpiR4;
