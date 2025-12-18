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
//! ## New Modular Architecture
//!
//! - `xhci` - Pure xHCI controller (NO vendor code)
//! - `phy` - PHY driver trait and implementations
//! - `soc` - SoC USB wrapper trait and implementations
//! - `board` - Board configuration trait and implementations
//!
//! ## Legacy Modules (for backward compatibility)
//!
//! - `xhci_regs` - Old xHCI register definitions
//! - `phy_regs` - Old MediaTek PHY register definitions
//! - `hal` - Old HAL (being phased out)
//! - `controller` - Old controller helpers

#![no_std]

// =============================================================================
// New Modular Architecture
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

/// IPC protocol definitions
pub mod protocol;

/// Block device client (for userspace drivers like fatfs)
pub mod block_client;

/// MMIO region helpers
pub mod mmio;

// =============================================================================
// Legacy Modules (backward compatibility)
// =============================================================================

/// Old xHCI register definitions (use xhci::regs instead)
#[path = "xhci_regs.rs"]
pub mod xhci_regs;

/// Old MediaTek PHY register definitions
#[path = "phy_regs.rs"]
pub mod phy_regs;

/// Old hardware addresses (MediaTek specific)
pub mod consts;

/// Old HAL (being phased out - use soc/phy/board instead)
pub mod hal;

/// Old controller helpers (being merged into xhci module)
pub mod controller;

// =============================================================================
// Re-exports for convenience
// =============================================================================

// Core types
pub use trb::{Trb, trb_type, trb_cc};
pub use ring::{Ring, EventRing, ErstEntry, Dcbaa, RING_SIZE};
pub use mmio::{MmioRegion, format_mmio_url, format_hex, delay_ms, delay, print_hex64, print_hex32, print_hex8};

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
};

// Protocol types
pub use protocol::{
    UsbRequest, UsbStatus, UsbDeviceEntry, UsbDeviceInfo,
    ControlTransferRequest, ControlTransferResponse,
    BulkTransferRequest, BulkTransferResponse,
    ConfigureDeviceRequest, ConfigureDeviceResponse,
    SetupEndpointsRequest, SetupEndpointsResponse, EndpointInfo,
    UsbMessageHeader, UsbResponseHeader,
    USB_MSG_MAX_SIZE, USB_DATA_MAX_SIZE,
    // Block device protocol
    BlockReadRequest, BlockReadResponse,
    BlockInfoRequest, BlockInfoResponse,
    BlockWriteRequest, BlockWriteResponse,
    // Zero-copy DMA block protocol
    BlockReadDmaRequest, BlockReadDmaResponse,
    BlockWriteDmaRequest, BlockWriteDmaResponse,
};

// Block device client (ring buffer based)
pub use block_client::BlockClient;

// =============================================================================
// Legacy Re-exports (for backward compatibility with existing code)
// =============================================================================

// Old register definitions (aliased to new location)
pub use xhci_regs::{xhci_cap, xhci_op, usbcmd, usbsts, xhci_port, xhci_rt, xhci_ir};
pub use phy_regs::{ippc, tphy, xsphy};
pub use consts::*;

// Old controller helpers
pub use controller::{
    PortLinkState, PortSpeed, portsc, ParsedPortsc, XhciCapabilities,
    event_completion_code, event_slot_id, event_endpoint_id, event_port_id,
    event_trb_pointer, event_transfer_length, event_is_short_packet,
    event_type, completion_code, doorbell,
};

// Old HAL (still used by usbd until migration)
pub use hal::{SocHal, ControllerId, XhciController, mt7988a};
pub use hal::mt7988a::Mt7988aHal;
