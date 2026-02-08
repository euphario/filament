//! Bus Control Protocol
//!
//! Messages and structures for communication between kernel and devd
//! over bus control ports.

// =============================================================================
// Bus Control Messages (Protocol)
// =============================================================================

/// Messages between kernel and devd over bus control port
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BusControlMsgType {
    // ─── Kernel → devd (first byte of payload) ───

    /// Sent immediately on connect: current bus state snapshot
    StateSnapshot = 0,

    /// State changed notification
    StateChanged = 1,

    /// Error notification
    Error = 2,

    /// Device list chunk (sent after StateSnapshot on connect)
    DeviceList = 3,

    // ─── owner → kernel (requests on owner channel) ───

    /// Enable bus mastering for a device
    EnableBusMastering = 16,

    /// Disable bus mastering for a device
    DisableBusMastering = 17,

    /// Request IOMMU mapping
    MapDma = 18,

    /// Release IOMMU mapping
    UnmapDma = 19,

    /// Request nuclear reset
    RequestReset = 20,

    /// Atomic handoff to new owner (for live update)
    Handoff = 21,

    /// Register a device on this bus (probed → kernel via write(bus_handle))
    RegisterDevice = 23,

    // SetDriver removed — owner is identified by who connects

    // ─── Responses ───

    /// Success response
    Ok = 128,

    /// Error response
    Err = 129,
}

/// State snapshot sent on connect
#[derive(Clone, Copy)]
#[repr(C)]
pub struct StateSnapshot {
    pub msg_type: u8,           // BusControlMsgType::StateSnapshot
    pub bus_type: u8,           // BusType
    pub bus_index: u8,          // Which bus of this type (0, 1, ...)
    pub state: u8,              // BusState
    pub device_count: u8,       // Number of devices currently on bus
    pub capabilities: u8,       // Bus capabilities flags
    pub _reserved: [u8; 2],
    pub since_boot_ms: u64,     // Time since boot when snapshot was taken
}

impl StateSnapshot {
    pub fn to_bytes(&self) -> [u8; 16] {
        let mut bytes = [0u8; 16];
        bytes[0] = self.msg_type;
        bytes[1] = self.bus_type;
        bytes[2] = self.bus_index;
        bytes[3] = self.state;
        bytes[4] = self.device_count;
        bytes[5] = self.capabilities;
        bytes[8..16].copy_from_slice(&self.since_boot_ms.to_le_bytes());
        bytes
    }
}

/// State change notification
#[derive(Clone, Copy)]
#[repr(C)]
pub struct StateChangedMsg {
    pub msg_type: u8,           // BusControlMsgType::StateChanged
    pub from_state: u8,         // Previous state
    pub to_state: u8,           // New state
    pub reason: u8,             // StateChangeReason
}

/// Reasons for state changes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StateChangeReason {
    /// Initial connection
    Connected = 0,
    /// Owner disconnected cleanly
    Disconnected = 1,
    /// Owner process crashed
    OwnerCrashed = 2,
    /// Reset requested by owner
    ResetRequested = 3,
    /// Reset completed
    ResetComplete = 4,
    /// Handoff to new owner
    Handoff = 5,
    /// Driver claimed bus via SetDriver
    DriverClaimed = 6,
    /// Driver process exited
    DriverExited = 7,
}

impl StateChangeReason {
    pub fn as_str(&self) -> &'static str {
        match self {
            StateChangeReason::Connected => "connected",
            StateChangeReason::Disconnected => "disconnected",
            StateChangeReason::OwnerCrashed => "owner_crashed",
            StateChangeReason::ResetRequested => "reset_requested",
            StateChangeReason::ResetComplete => "reset_complete",
            StateChangeReason::Handoff => "handoff",
            StateChangeReason::DriverClaimed => "driver_claimed",
            StateChangeReason::DriverExited => "driver_exited",
        }
    }
}
