//! USB standard requests, hub definitions, and xHCI context structures

// =============================================================================
// USB Standard Requests
// =============================================================================

pub mod usb_req {
    // Request types (bmRequestType)
    pub const DIR_OUT: u8 = 0x00;
    pub const DIR_IN: u8 = 0x80;
    pub const TYPE_STANDARD: u8 = 0x00;
    pub const TYPE_CLASS: u8 = 0x20;
    pub const RECIP_DEVICE: u8 = 0x00;
    pub const RECIP_INTERFACE: u8 = 0x01;
    pub const RECIP_ENDPOINT: u8 = 0x02;

    // Standard requests (bRequest)
    pub const GET_STATUS: u8 = 0;
    pub const CLEAR_FEATURE: u8 = 1;
    pub const SET_FEATURE: u8 = 3;
    pub const SET_ADDRESS: u8 = 5;
    pub const GET_DESCRIPTOR: u8 = 6;
    pub const SET_DESCRIPTOR: u8 = 7;
    pub const GET_CONFIGURATION: u8 = 8;
    pub const SET_CONFIGURATION: u8 = 9;

    // Descriptor types
    pub const DESC_DEVICE: u16 = 1;
    pub const DESC_CONFIGURATION: u16 = 2;
    pub const DESC_STRING: u16 = 3;
    pub const DESC_INTERFACE: u16 = 4;
    pub const DESC_ENDPOINT: u16 = 5;
    pub const DESC_HUB: u16 = 0x29;
    pub const DESC_SS_HUB: u16 = 0x2A;  // SuperSpeed Hub Descriptor
}

// =============================================================================
// USB Hub Definitions
// =============================================================================

pub mod hub {
    // Hub class request types
    pub const RT_HUB_GET: u8 = 0xA0;      // Device-to-host, class, device
    pub const RT_HUB_SET: u8 = 0x20;      // Host-to-device, class, device
    pub const RT_PORT_GET: u8 = 0xA3;     // Device-to-host, class, other (port)
    pub const RT_PORT_SET: u8 = 0x23;     // Host-to-device, class, other (port)

    // Hub class requests
    pub const GET_STATUS: u8 = 0;
    pub const CLEAR_FEATURE: u8 = 1;
    pub const SET_FEATURE: u8 = 3;
    pub const GET_DESCRIPTOR: u8 = 6;

    // Port features
    pub const PORT_CONNECTION: u16 = 0;
    pub const PORT_ENABLE: u16 = 1;
    pub const PORT_RESET: u16 = 4;
    pub const PORT_POWER: u16 = 8;
    pub const C_PORT_CONNECTION: u16 = 16;
    pub const C_PORT_RESET: u16 = 20;
    pub const PORT_LINK_STATE: u16 = 5;
    pub const BH_PORT_RESET: u16 = 28;    // USB3 warm reset

    // Hub class requests
    pub const SET_HUB_DEPTH: u8 = 12;     // USB3 hub depth setting

    // Port status bits (wPortStatus)
    pub const PS_CONNECTION: u16 = 1 << 0;
    pub const PS_ENABLE: u16 = 1 << 1;
    pub const PS_RESET: u16 = 1 << 4;
    pub const PS_POWER: u16 = 1 << 9;

    // Port status change bits (wPortChange) - in high word
    pub const PS_C_CONNECTION: u16 = 1 << 0;
    pub const PS_C_ENABLE: u16 = 1 << 1;
    pub const PS_C_RESET: u16 = 1 << 4;
}

/// USB Hub Descriptor (USB 2.0)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct HubDescriptor {
    pub length: u8,
    pub desc_type: u8,
    pub num_ports: u8,
    pub characteristics: u16,
    pub pwr_on_2_pwr_good: u8,  // Time in 2ms intervals
    pub hub_contr_current: u8,
    // Variable length fields follow (device removable bitmap)
}

/// USB 3.0 SuperSpeed Hub Descriptor
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct SsHubDescriptor {
    pub length: u8,
    pub desc_type: u8,
    pub num_ports: u8,
    pub characteristics: u16,
    pub pwr_on_2_pwr_good: u8,
    pub hub_contr_current: u8,
    pub hub_hdr_dec_lat: u8,
    pub hub_delay: u16,
    pub device_removable: u16,
}

/// Port Status response (4 bytes)
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct PortStatus {
    pub status: u16,   // wPortStatus
    pub change: u16,   // wPortChange
}

// =============================================================================
// xHCI Context Structures
// =============================================================================

/// Slot Context - 32 bytes (8 DWORDs)
#[repr(C, align(32))]
#[derive(Clone, Copy, Default)]
pub struct SlotContext {
    pub dw0: u32,  // Route String, Speed, MTT, Hub, Context Entries
    pub dw1: u32,  // Max Exit Latency, Root Hub Port Number, Num Ports
    pub dw2: u32,  // Parent Hub Slot ID, Parent Port Number, TTT, Interrupter Target
    pub dw3: u32,  // USB Device Address, Slot State
    pub reserved: [u32; 4],
}

impl SlotContext {
    pub fn set_route_string(&mut self, route: u32) {
        self.dw0 = (self.dw0 & !0xFFFFF) | (route & 0xFFFFF);
    }

    pub fn set_speed(&mut self, speed: u32) {
        self.dw0 = (self.dw0 & !(0xF << 20)) | ((speed & 0xF) << 20);
    }

    pub fn set_context_entries(&mut self, entries: u32) {
        self.dw0 = (self.dw0 & !(0x1F << 27)) | ((entries & 0x1F) << 27);
    }

    pub fn set_root_hub_port(&mut self, port: u32) {
        self.dw1 = (self.dw1 & !(0xFF << 16)) | ((port & 0xFF) << 16);
    }

    pub fn set_num_ports(&mut self, num_ports: u32) {
        self.dw1 = (self.dw1 & !(0xFF << 24)) | ((num_ports & 0xFF) << 24);
    }

    pub fn set_hub(&mut self, is_hub: bool) {
        if is_hub {
            self.dw0 |= 1 << 26;
        } else {
            self.dw0 &= !(1 << 26);
        }
    }

    pub fn set_interrupter_target(&mut self, target: u32) {
        self.dw2 = (self.dw2 & !0x3FF) | (target & 0x3FF);
    }
}

/// Endpoint Context - 32 bytes (8 DWORDs)
#[repr(C, align(32))]
#[derive(Clone, Copy, Default)]
pub struct EndpointContext {
    pub dw0: u32,  // EP State, Mult, MaxPStreams, LSA, Interval, MaxESITPayloadHi
    pub dw1: u32,  // MaxPacketSize, MaxBurstSize, HID, EP Type, CErr
    pub dw2: u32,  // TR Dequeue Pointer Lo (+ DCS bit 0)
    pub dw3: u32,  // TR Dequeue Pointer Hi
    pub dw4: u32,  // Average TRB Length, Max ESIT Payload Lo
    pub reserved: [u32; 3],
}

impl EndpointContext {
    pub fn set_ep_type(&mut self, ep_type: u32) {
        self.dw1 = (self.dw1 & !(0x7 << 3)) | ((ep_type & 0x7) << 3);
    }

    pub fn set_max_packet_size(&mut self, size: u32) {
        self.dw1 = (self.dw1 & !0xFFFF0000) | ((size & 0xFFFF) << 16);
    }

    pub fn set_max_burst_size(&mut self, burst: u32) {
        self.dw1 = (self.dw1 & !(0xFF << 8)) | ((burst & 0xFF) << 8);
    }

    pub fn set_cerr(&mut self, cerr: u32) {
        self.dw1 = (self.dw1 & !(0x3 << 1)) | ((cerr & 0x3) << 1);
    }

    pub fn set_tr_dequeue_ptr(&mut self, ptr: u64, dcs: bool) {
        self.dw2 = (ptr as u32 & !0xF) | (dcs as u32);
        self.dw3 = (ptr >> 32) as u32;
    }

    pub fn set_average_trb_length(&mut self, len: u32) {
        self.dw4 = (self.dw4 & !0xFFFF) | (len & 0xFFFF);
    }

    pub fn set_interval(&mut self, interval: u32) {
        self.dw0 = (self.dw0 & !(0xFF << 16)) | ((interval & 0xFF) << 16);
    }
}

// Endpoint types
pub mod ep_type {
    pub const CONTROL: u32 = 4;
    pub const ISOCH_OUT: u32 = 1;
    pub const BULK_OUT: u32 = 2;
    pub const INTERRUPT_OUT: u32 = 3;
    pub const ISOCH_IN: u32 = 5;
    pub const BULK_IN: u32 = 6;
    pub const INTERRUPT_IN: u32 = 7;
}

/// Input Control Context - 32 bytes
#[repr(C, align(32))]
#[derive(Clone, Copy, Default)]
pub struct InputControlContext {
    pub drop_flags: u32,
    pub add_flags: u32,
    pub reserved: [u32; 6],
}

/// Input Context - contains Input Control + Slot + 31 Endpoint Contexts
/// Total: 32 + 32 + 31*32 = 1056 bytes, aligned to 64 bytes
#[repr(C, align(64))]
pub struct InputContext {
    pub control: InputControlContext,
    pub slot: SlotContext,
    pub endpoints: [EndpointContext; 31],
}

impl InputContext {
    pub fn new() -> Self {
        Self {
            control: InputControlContext::default(),
            slot: SlotContext::default(),
            endpoints: [EndpointContext::default(); 31],
        }
    }
}

/// Device Context - Slot + 31 Endpoint Contexts
#[repr(C, align(64))]
pub struct DeviceContext {
    pub slot: SlotContext,
    pub endpoints: [EndpointContext; 31],
}

impl DeviceContext {
    pub fn new() -> Self {
        Self {
            slot: SlotContext::default(),
            endpoints: [EndpointContext::default(); 31],
        }
    }
}

/// USB Device Descriptor (18 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct DeviceDescriptor {
    pub length: u8,
    pub descriptor_type: u8,
    pub bcd_usb: u16,
    pub device_class: u8,
    pub device_subclass: u8,
    pub device_protocol: u8,
    pub max_packet_size0: u8,
    pub vendor_id: u16,
    pub product_id: u16,
    pub bcd_device: u16,
    pub manufacturer: u8,
    pub product: u8,
    pub serial_number: u8,
    pub num_configurations: u8,
}

/// USB Configuration Descriptor (9 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct ConfigurationDescriptor {
    pub length: u8,
    pub descriptor_type: u8,
    pub total_length: u16,
    pub num_interfaces: u8,
    pub configuration_value: u8,
    pub configuration: u8,
    pub attributes: u8,
    pub max_power: u8,
}

/// USB Interface Descriptor (9 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct InterfaceDescriptor {
    pub length: u8,
    pub descriptor_type: u8,
    pub interface_number: u8,
    pub alternate_setting: u8,
    pub num_endpoints: u8,
    pub interface_class: u8,
    pub interface_subclass: u8,
    pub interface_protocol: u8,
    pub interface: u8,
}

/// USB Endpoint Descriptor (7 bytes)
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct EndpointDescriptor {
    pub length: u8,
    pub descriptor_type: u8,
    pub endpoint_address: u8,
    pub attributes: u8,
    pub max_packet_size: u16,
    pub interval: u8,
}
