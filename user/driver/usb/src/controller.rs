//! xHCI Controller Helpers
//!
//! This module provides xHCI controller-related utilities including
//! event TRB parsing and port status interpretation.

use crate::trb::Trb;

/// Port Link State values from PORTSC register
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PortLinkState {
    U0 = 0,           // Active
    U1 = 1,           // Standby
    U2 = 2,           // Sleep
    U3 = 3,           // Suspend
    Disabled = 4,
    RxDetect = 5,
    Inactive = 6,
    Polling = 7,
    Recovery = 8,
    HotReset = 9,
    ComplianceMode = 10,
    TestMode = 11,
    Resume = 15,
    Unknown = 255,
}

impl PortLinkState {
    pub fn from_bits(bits: u32) -> Self {
        match bits & 0xF {
            0 => Self::U0,
            1 => Self::U1,
            2 => Self::U2,
            3 => Self::U3,
            4 => Self::Disabled,
            5 => Self::RxDetect,
            6 => Self::Inactive,
            7 => Self::Polling,
            8 => Self::Recovery,
            9 => Self::HotReset,
            10 => Self::ComplianceMode,
            11 => Self::TestMode,
            15 => Self::Resume,
            _ => Self::Unknown,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            Self::U0 => "U0",
            Self::U1 => "U1",
            Self::U2 => "U2",
            Self::U3 => "U3",
            Self::Disabled => "Disabled",
            Self::RxDetect => "RxDetect",
            Self::Inactive => "Inactive",
            Self::Polling => "Polling",
            Self::Recovery => "Recovery",
            Self::HotReset => "HotReset",
            Self::ComplianceMode => "Compliance",
            Self::TestMode => "Test",
            Self::Resume => "Resume",
            Self::Unknown => "?",
        }
    }
}

/// Port Speed values from PORTSC register
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PortSpeed {
    Invalid = 0,
    FullSpeed = 1,    // 12 Mbps
    LowSpeed = 2,     // 1.5 Mbps
    HighSpeed = 3,    // 480 Mbps
    SuperSpeed = 4,   // 5 Gbps
    SuperSpeedPlus = 5, // 10 Gbps
    Unknown = 255,
}

impl PortSpeed {
    pub fn from_bits(bits: u32) -> Self {
        match bits & 0xF {
            0 => Self::Invalid,
            1 => Self::FullSpeed,
            2 => Self::LowSpeed,
            3 => Self::HighSpeed,
            4 => Self::SuperSpeed,
            5 => Self::SuperSpeedPlus,
            _ => Self::Unknown,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Invalid => "Invalid",
            Self::FullSpeed => "FS",
            Self::LowSpeed => "LS",
            Self::HighSpeed => "HS",
            Self::SuperSpeed => "SS",
            Self::SuperSpeedPlus => "SS+",
            Self::Unknown => "?",
        }
    }

    pub fn full_name(&self) -> &'static str {
        match self {
            Self::Invalid => "Invalid",
            Self::FullSpeed => "Full Speed",
            Self::LowSpeed => "Low Speed",
            Self::HighSpeed => "High Speed",
            Self::SuperSpeed => "Super Speed",
            Self::SuperSpeedPlus => "Super Speed Plus",
            Self::Unknown => "Unknown",
        }
    }

    /// Get default max packet size for EP0 at this speed
    pub fn default_ep0_max_packet(&self) -> u32 {
        match self {
            Self::LowSpeed => 8,
            Self::FullSpeed => 64,
            Self::HighSpeed => 64,
            Self::SuperSpeed | Self::SuperSpeedPlus => 512,
            _ => 8,
        }
    }

    /// Convert to xHCI slot context speed value
    pub fn to_slot_speed(&self) -> u32 {
        match self {
            Self::FullSpeed => 1,
            Self::LowSpeed => 2,
            Self::HighSpeed => 3,
            Self::SuperSpeed => 4,
            Self::SuperSpeedPlus => 5,
            _ => 0,
        }
    }
}

/// PORTSC register bit masks
pub mod portsc {
    pub const CCS: u32 = 1 << 0;      // Current Connect Status
    pub const PED: u32 = 1 << 1;      // Port Enabled/Disabled
    pub const OCA: u32 = 1 << 3;      // Over-current Active
    pub const PR: u32 = 1 << 4;       // Port Reset
    pub const PLS_MASK: u32 = 0xF << 5;  // Port Link State
    pub const PLS_SHIFT: u32 = 5;
    pub const PP: u32 = 1 << 9;       // Port Power
    pub const SPEED_MASK: u32 = 0xF << 10;  // Port Speed
    pub const SPEED_SHIFT: u32 = 10;
    pub const PIC_MASK: u32 = 0x3 << 14;  // Port Indicator Control
    pub const LWS: u32 = 1 << 16;     // Link State Write Strobe
    pub const CSC: u32 = 1 << 17;     // Connect Status Change
    pub const PEC: u32 = 1 << 18;     // Port Enabled/Disabled Change
    pub const WRC: u32 = 1 << 19;     // Warm Port Reset Change
    pub const OCC: u32 = 1 << 20;     // Over-current Change
    pub const PRC: u32 = 1 << 21;     // Port Reset Change
    pub const PLC: u32 = 1 << 22;     // Port Link State Change
    pub const CEC: u32 = 1 << 23;     // Port Config Error Change
    pub const CAS: u32 = 1 << 24;     // Cold Attach Status
    pub const WCE: u32 = 1 << 25;     // Wake on Connect Enable
    pub const WDE: u32 = 1 << 26;     // Wake on Disconnect Enable
    pub const WOE: u32 = 1 << 27;     // Wake on Over-current Enable
    pub const DR: u32 = 1 << 30;      // Device Removable
    pub const WPR: u32 = 1 << 31;     // Warm Port Reset

    /// Status change bits that are Write-1-to-Clear
    pub const CHANGE_BITS: u32 = CSC | PEC | WRC | OCC | PRC | PLC | CEC;

    /// Bits to preserve when writing (don't accidentally clear status)
    pub const PRESERVE_MASK: u32 = PP | PIC_MASK;
}

/// Parsed PORTSC register value
#[derive(Copy, Clone, Debug)]
pub struct ParsedPortsc {
    pub raw: u32,
    pub connected: bool,
    pub enabled: bool,
    pub over_current: bool,
    pub reset: bool,
    pub link_state: PortLinkState,
    pub powered: bool,
    pub speed: PortSpeed,
    // Status change flags
    pub connect_change: bool,
    pub enable_change: bool,
    pub warm_reset_change: bool,
    pub over_current_change: bool,
    pub reset_change: bool,
    pub link_state_change: bool,
}

impl ParsedPortsc {
    /// Parse a raw PORTSC register value
    pub fn from_raw(raw: u32) -> Self {
        Self {
            raw,
            connected: (raw & portsc::CCS) != 0,
            enabled: (raw & portsc::PED) != 0,
            over_current: (raw & portsc::OCA) != 0,
            reset: (raw & portsc::PR) != 0,
            link_state: PortLinkState::from_bits((raw >> portsc::PLS_SHIFT) & 0xF),
            powered: (raw & portsc::PP) != 0,
            speed: PortSpeed::from_bits((raw >> portsc::SPEED_SHIFT) & 0xF),
            connect_change: (raw & portsc::CSC) != 0,
            enable_change: (raw & portsc::PEC) != 0,
            warm_reset_change: (raw & portsc::WRC) != 0,
            over_current_change: (raw & portsc::OCC) != 0,
            reset_change: (raw & portsc::PRC) != 0,
            link_state_change: (raw & portsc::PLC) != 0,
        }
    }

    /// Get value to write to clear all status change bits
    pub fn clear_changes_value(&self) -> u32 {
        // Preserve PP and other writable bits, set change bits to clear them
        (self.raw & portsc::PRESERVE_MASK) | (self.raw & portsc::CHANGE_BITS)
    }

    /// Check if device is ready for enumeration
    pub fn is_ready_for_enum(&self) -> bool {
        self.powered && self.connected && self.enabled && !self.reset
    }
}

// =============================================================================
// Event TRB Parsing Helpers
// =============================================================================

/// Extract completion code from event TRB status field
#[inline]
pub fn event_completion_code(trb: &Trb) -> u32 {
    (trb.status >> 24) & 0xFF
}

/// Extract slot ID from command completion or transfer event TRB
#[inline]
pub fn event_slot_id(trb: &Trb) -> u32 {
    (trb.control >> 24) & 0xFF
}

/// Extract endpoint ID from transfer event TRB
#[inline]
pub fn event_endpoint_id(trb: &Trb) -> u32 {
    (trb.control >> 16) & 0x1F
}

/// Extract port ID from port status change event TRB
#[inline]
pub fn event_port_id(trb: &Trb) -> u32 {
    ((trb.param >> 24) & 0xFF) as u32
}

/// Extract TRB pointer from event TRB (for transfer/command completion)
#[inline]
pub fn event_trb_pointer(trb: &Trb) -> u64 {
    trb.param
}

/// Extract transfer length from transfer event TRB
#[inline]
pub fn event_transfer_length(trb: &Trb) -> u32 {
    trb.status & 0xFFFFFF
}

/// Check if event indicates a short packet (for transfer events)
#[inline]
pub fn event_is_short_packet(trb: &Trb) -> bool {
    event_completion_code(trb) == crate::trb_cc::SHORT_PACKET
}

/// Event type constants for convenience
pub mod event_type {
    pub use crate::trb::trb_type::{
        TRANSFER_EVENT,
        COMMAND_COMPLETION,
        PORT_STATUS_CHANGE,
        HOST_CONTROLLER,
    };
}

/// Completion code helpers
pub mod completion_code {
    pub use crate::trb::trb_cc::*;

    /// Check if completion code indicates success
    pub fn is_success(cc: u32) -> bool {
        cc == SUCCESS || cc == SHORT_PACKET
    }

    /// Get human-readable completion code name
    pub fn name(cc: u32) -> &'static str {
        match cc {
            SUCCESS => "Success",
            SHORT_PACKET => "Short Packet",
            STALL => "Stall",
            USB_TRANSACTION_ERROR => "Transaction Error",
            TRB_ERROR => "TRB Error",
            BABBLE => "Babble",
            _ => "Unknown",
        }
    }
}

// =============================================================================
// Doorbell Target Values
// =============================================================================

/// Doorbell register target values
pub mod doorbell {
    /// Host controller command (doorbell 0)
    pub const COMMAND_RING: u32 = 0;

    /// EP0 (control endpoint) for any slot
    pub const EP0: u32 = 1;

    /// Calculate doorbell target for endpoint
    /// For EP0 (bidirectional): target = 1
    /// For other endpoints: target = DCI = 2*ep_num + direction
    pub fn endpoint_target(ep_addr: u8) -> u32 {
        let ep_num = ep_addr & 0x0F;
        if ep_num == 0 {
            1
        } else {
            let is_in = (ep_addr & 0x80) != 0;
            (2 * ep_num as u32) + if is_in { 1 } else { 0 }
        }
    }
}
