//! USB IPC Protocol
//!
//! Defines message formats for communication between usbd and class drivers.
//! Class drivers (MSC, HID, etc.) connect to usbd via the "usb" port and
//! send requests to perform USB operations.

/// USB IPC Request Types
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UsbRequest {
    /// List all enumerated USB devices
    /// Response: UsbDeviceList
    ListDevices = 0,

    /// Get device information (descriptors, endpoints)
    /// Payload: slot_id (u32)
    /// Response: UsbDeviceInfo
    GetDeviceInfo = 1,

    /// Perform a control transfer
    /// Payload: ControlTransferRequest
    /// Response: ControlTransferResponse
    ControlTransfer = 2,

    /// Perform a bulk OUT transfer
    /// Payload: BulkTransferRequest + data
    /// Response: BulkTransferResponse
    BulkOut = 3,

    /// Perform a bulk IN transfer
    /// Payload: BulkTransferRequest
    /// Response: BulkTransferResponse + data
    BulkIn = 4,

    /// Configure device (set configuration, setup endpoints)
    /// Payload: ConfigureDeviceRequest
    /// Response: ConfigureDeviceResponse
    ConfigureDevice = 5,

    /// Setup bulk endpoints for a device
    /// Payload: SetupEndpointsRequest
    /// Response: SetupEndpointsResponse
    SetupEndpoints = 6,
}

/// USB IPC Response Status
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UsbStatus {
    Ok = 0,
    Error = 1,
    Stall = 2,
    Timeout = 3,
    NotFound = 4,
    Busy = 5,
    InvalidRequest = 6,
}

/// USB Device Entry in device list
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct UsbDeviceEntry {
    pub slot_id: u32,
    pub vendor_id: u16,
    pub product_id: u16,
    pub device_class: u8,
    pub device_subclass: u8,
    pub device_protocol: u8,
    pub speed: u8,  // 1=LS, 2=FS, 3=HS, 4=SS, 5=SS+
}

/// Control Transfer Request
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct ControlTransferRequest {
    pub slot_id: u32,
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
    pub direction: u8,  // 0 = OUT (host to device), 1 = IN (device to host)
    pub _pad: u8,
}

/// Control Transfer Response
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct ControlTransferResponse {
    pub status: UsbStatus,
    pub bytes_transferred: u16,
    pub _pad: u8,
    // Followed by data if direction was IN
}

/// Bulk Transfer Request
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BulkTransferRequest {
    pub slot_id: u32,
    pub endpoint: u8,     // Endpoint address (with direction bit)
    pub _pad: [u8; 3],
    pub length: u32,      // For OUT: data follows this header; For IN: max bytes to receive
}

/// Bulk Transfer Response
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BulkTransferResponse {
    pub status: UsbStatus,
    pub _pad: [u8; 3],
    pub bytes_transferred: u32,
    // For IN transfers: data follows this header
}

/// Configure Device Request
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct ConfigureDeviceRequest {
    pub slot_id: u32,
    pub configuration: u8,
    pub _pad: [u8; 3],
}

/// Configure Device Response
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct ConfigureDeviceResponse {
    pub status: UsbStatus,
    pub _pad: [u8; 3],
}

/// Setup Endpoints Request
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct SetupEndpointsRequest {
    pub slot_id: u32,
    pub interface_number: u8,
    pub alternate_setting: u8,
    pub endpoint_count: u8,
    pub _pad: u8,
    // Followed by EndpointDescriptor entries
}

/// Endpoint info for setup
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct EndpointInfo {
    pub address: u8,
    pub attributes: u8,
    pub max_packet_size: u16,
    pub interval: u8,
    pub _pad: [u8; 3],
}

/// Setup Endpoints Response
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct SetupEndpointsResponse {
    pub status: UsbStatus,
    pub _pad: [u8; 3],
}

/// Device Info Response (variable length)
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct UsbDeviceInfo {
    pub slot_id: u32,
    pub vendor_id: u16,
    pub product_id: u16,
    pub device_class: u8,
    pub device_subclass: u8,
    pub device_protocol: u8,
    pub speed: u8,
    pub num_configurations: u8,
    pub current_configuration: u8,
    pub num_interfaces: u8,
    pub _pad: u8,
    // Followed by interface/endpoint descriptors
}

/// Message header for all USB IPC messages
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct UsbMessageHeader {
    pub request: UsbRequest,
    pub _pad: [u8; 3],
    pub payload_length: u32,
}

impl UsbMessageHeader {
    pub const SIZE: usize = core::mem::size_of::<UsbMessageHeader>();

    pub fn new(request: UsbRequest, payload_length: u32) -> Self {
        Self {
            request,
            _pad: [0; 3],
            payload_length,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        unsafe { core::mem::transmute(*self) }
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < Self::SIZE {
            return None;
        }
        let mut arr = [0u8; Self::SIZE];
        arr.copy_from_slice(&bytes[..Self::SIZE]);
        Some(unsafe { core::mem::transmute(arr) })
    }
}

/// Response header for all USB IPC responses
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct UsbResponseHeader {
    pub status: UsbStatus,
    pub _pad: [u8; 3],
    pub payload_length: u32,
}

impl UsbResponseHeader {
    pub const SIZE: usize = core::mem::size_of::<UsbResponseHeader>();

    pub fn new(status: UsbStatus, payload_length: u32) -> Self {
        Self {
            status,
            _pad: [0; 3],
            payload_length,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        unsafe { core::mem::transmute(*self) }
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < Self::SIZE {
            return None;
        }
        let mut arr = [0u8; Self::SIZE];
        arr.copy_from_slice(&bytes[..Self::SIZE]);
        Some(unsafe { core::mem::transmute(arr) })
    }
}

// Buffer size for IPC messages
pub const USB_MSG_MAX_SIZE: usize = 4096;
pub const USB_DATA_MAX_SIZE: usize = USB_MSG_MAX_SIZE - UsbMessageHeader::SIZE - 64;
