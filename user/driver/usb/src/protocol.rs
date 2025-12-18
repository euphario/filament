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

    /// Read blocks from a mass storage device
    /// Payload: BlockReadRequest
    /// Response: BlockReadResponse + data
    BlockRead = 0x10,

    /// Get block device info (capacity, block size)
    /// Payload: BlockInfoRequest
    /// Response: BlockInfoResponse
    BlockInfo = 0x11,

    /// Write blocks to a mass storage device
    /// Payload: BlockWriteRequest + data
    /// Response: BlockWriteResponse
    BlockWrite = 0x12,
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

// ============================================================================
// Block Device Protocol (for MSC/FAT)
// ============================================================================

/// Block Read Request
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BlockReadRequest {
    /// USB slot ID of the mass storage device
    pub slot_id: u32,
    /// Logical Block Address to start reading from
    pub lba: u64,
    /// Number of blocks to read
    pub block_count: u32,
    pub _pad: u32,
}

impl BlockReadRequest {
    pub const SIZE: usize = core::mem::size_of::<BlockReadRequest>();

    pub fn new(slot_id: u32, lba: u64, block_count: u32) -> Self {
        Self {
            slot_id,
            lba,
            block_count,
            _pad: 0,
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

/// Block Read Response
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BlockReadResponse {
    /// Response type - always UsbRequest::BlockRead (0x10)
    pub response_type: u8,
    pub status: UsbStatus,
    pub _pad: [u8; 2],
    /// Number of bytes read (block_count * block_size)
    pub bytes_read: u32,
    // Followed by data bytes
}

impl BlockReadResponse {
    pub const SIZE: usize = core::mem::size_of::<BlockReadResponse>();

    pub fn new(status: UsbStatus, bytes_read: u32) -> Self {
        Self {
            response_type: UsbRequest::BlockRead as u8,
            status,
            _pad: [0; 2],
            bytes_read,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        unsafe { core::mem::transmute(*self) }
    }
}

/// Block Info Request
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BlockInfoRequest {
    /// USB slot ID of the mass storage device
    pub slot_id: u32,
}

impl BlockInfoRequest {
    pub const SIZE: usize = core::mem::size_of::<BlockInfoRequest>();

    pub fn new(slot_id: u32) -> Self {
        Self { slot_id }
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

/// Block Info Response
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BlockInfoResponse {
    /// Response type - always UsbRequest::BlockInfo (0x11)
    pub response_type: u8,
    pub status: UsbStatus,
    pub _pad: [u8; 2],
    /// Size of each block in bytes (typically 512)
    pub block_size: u32,
    /// Total number of blocks on the device
    pub block_count: u64,
}

impl BlockInfoResponse {
    pub const SIZE: usize = core::mem::size_of::<BlockInfoResponse>();

    pub fn new(status: UsbStatus, block_size: u32, block_count: u64) -> Self {
        Self {
            response_type: UsbRequest::BlockInfo as u8,
            status,
            _pad: [0; 2],
            block_size,
            block_count,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        unsafe { core::mem::transmute(*self) }
    }
}

/// Block Write Request
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BlockWriteRequest {
    /// USB slot ID of the mass storage device
    pub slot_id: u32,
    /// Logical Block Address to start writing to
    pub lba: u64,
    /// Number of blocks to write
    pub block_count: u32,
    pub _pad: u32,
    // Followed by data bytes
}

impl BlockWriteRequest {
    pub const SIZE: usize = core::mem::size_of::<BlockWriteRequest>();

    pub fn new(slot_id: u32, lba: u64, block_count: u32) -> Self {
        Self {
            slot_id,
            lba,
            block_count,
            _pad: 0,
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

/// Block Write Response
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct BlockWriteResponse {
    /// Response type - always UsbRequest::BlockWrite (0x12)
    pub response_type: u8,
    pub status: UsbStatus,
    pub _pad: [u8; 2],
    /// Number of bytes written
    pub bytes_written: u32,
}

impl BlockWriteResponse {
    pub const SIZE: usize = core::mem::size_of::<BlockWriteResponse>();

    pub fn new(status: UsbStatus, bytes_written: u32) -> Self {
        Self {
            response_type: UsbRequest::BlockWrite as u8,
            status,
            _pad: [0; 2],
            bytes_written,
        }
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        unsafe { core::mem::transmute(*self) }
    }
}
