//! USB driver library for MT7988A SSUSB
//!
//! This crate provides USB-related structures and constants for the
//! MediaTek MT7988A SSUSB host controller.

#![no_std]

pub mod consts;
pub mod phy;
pub mod xhci;
pub mod trb;
pub mod usb;
pub mod msc;
pub mod ring;
pub mod mmio;
pub mod protocol;
pub mod transfer;
pub mod hub;
pub mod enumeration;
pub mod controller;
pub mod hal;

// Re-export commonly used items at crate root
pub use consts::*;
pub use trb::{Trb, trb_type, trb_cc};
pub use usb::{
    usb_req, hub as usb_hub, ep_type,
    SlotContext, EndpointContext, InputControlContext, InputContext, DeviceContext,
    HubDescriptor, SsHubDescriptor, PortStatus,
    DeviceDescriptor, ConfigurationDescriptor, InterfaceDescriptor, EndpointDescriptor,
};
pub use msc::{msc as msc_const, scsi, Cbw, Csw, BulkContext, TransferResult, CBW_OFFSET, CSW_OFFSET, DATA_OFFSET};
pub use ring::{Ring, EventRing, ErstEntry, Dcbaa, RING_SIZE};
pub use mmio::{MmioRegion, format_mmio_url, format_hex, delay_ms, delay, print_hex64, print_hex32, print_hex8};
pub use phy::{ippc, tphy, xsphy};
pub use xhci::{xhci_cap, xhci_op, usbcmd, usbsts, xhci_port, xhci_rt, xhci_ir};
pub use protocol::{
    UsbRequest, UsbStatus, UsbDeviceEntry, UsbDeviceInfo,
    ControlTransferRequest, ControlTransferResponse,
    BulkTransferRequest, BulkTransferResponse,
    ConfigureDeviceRequest, ConfigureDeviceResponse,
    SetupEndpointsRequest, SetupEndpointsResponse, EndpointInfo,
    UsbMessageHeader, UsbResponseHeader,
    USB_MSG_MAX_SIZE, USB_DATA_MAX_SIZE,
};
pub use transfer::{
    SetupPacket, Direction, ControlStage, TransferType,
    build_setup_trb, build_data_trb, build_status_trb,
    build_link_trb, build_normal_trb,
    flush_trb, flush_trb_range, dsb,
};
pub use hub::{
    port_feature, port_status, port_change,
    get_hub_descriptor_setup, get_port_status_setup,
    set_port_feature_setup, clear_port_feature_setup, set_hub_depth_setup,
    parse_ss_hub_descriptor, HubInfo, OverCurrentMode,
    ParsedPortStatus, DeviceSpeed,
};
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
pub use controller::{
    PortLinkState, PortSpeed, portsc, ParsedPortsc, XhciCapabilities,
    event_completion_code, event_slot_id, event_endpoint_id, event_port_id,
    event_trb_pointer, event_transfer_length, event_is_short_packet,
    event_type, completion_code, doorbell,
};
pub use hal::{SocHal, ControllerId, XhciController, mt7988a};
pub use hal::mt7988a::Mt7988aHal;
