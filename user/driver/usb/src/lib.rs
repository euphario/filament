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

// Re-export commonly used items at crate root
pub use consts::*;
pub use trb::{Trb, trb_type, trb_cc};
pub use usb::{
    usb_req, hub, ep_type,
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
