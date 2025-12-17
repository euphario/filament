//! USB Hub Support
//!
//! This module provides USB hub-related utilities and types.
//! The actual hub enumeration logic remains in usbd due to tight
//! coupling with the control transfer implementation.

use crate::usb::{hub, usb_req, PortStatus};
use crate::transfer::SetupPacket;

/// Hub port feature selectors (for SET_FEATURE/CLEAR_FEATURE)
pub mod port_feature {
    pub const CONNECTION: u16 = 0;
    pub const ENABLE: u16 = 1;
    pub const SUSPEND: u16 = 2;
    pub const OVER_CURRENT: u16 = 3;
    pub const RESET: u16 = 4;
    pub const POWER: u16 = 8;
    pub const LOW_SPEED: u16 = 9;
    pub const C_CONNECTION: u16 = 16;
    pub const C_ENABLE: u16 = 17;
    pub const C_SUSPEND: u16 = 18;
    pub const C_OVER_CURRENT: u16 = 19;
    pub const C_RESET: u16 = 20;
    // USB 3.0 specific
    pub const BH_PORT_RESET: u16 = 28;
    pub const C_BH_PORT_RESET: u16 = 29;
}

/// Hub port status bits
pub mod port_status {
    pub const CONNECTION: u16 = 1 << 0;
    pub const ENABLE: u16 = 1 << 1;
    pub const SUSPEND: u16 = 1 << 2;
    pub const OVER_CURRENT: u16 = 1 << 3;
    pub const RESET: u16 = 1 << 4;
    pub const POWER: u16 = 1 << 8;
    pub const LOW_SPEED: u16 = 1 << 9;
    pub const HIGH_SPEED: u16 = 1 << 10;
}

/// Hub port change bits (upper 16 bits of port status)
pub mod port_change {
    pub const C_CONNECTION: u16 = 1 << 0;
    pub const C_ENABLE: u16 = 1 << 1;
    pub const C_SUSPEND: u16 = 1 << 2;
    pub const C_OVER_CURRENT: u16 = 1 << 3;
    pub const C_RESET: u16 = 1 << 4;
}

/// Build a GET_HUB_DESCRIPTOR setup packet
pub fn get_hub_descriptor_setup(ss: bool) -> SetupPacket {
    let desc_type = if ss { usb_req::DESC_SS_HUB } else { usb_req::DESC_HUB };
    SetupPacket::new(
        hub::RT_HUB_GET,
        hub::GET_DESCRIPTOR,
        (desc_type as u16) << 8,
        0,
        if ss { 12 } else { 8 },  // SS hub desc is 12 bytes, USB2 is 8
    )
}

/// Build a GET_PORT_STATUS setup packet
pub fn get_port_status_setup(port: u16) -> SetupPacket {
    SetupPacket::new(
        hub::RT_PORT_GET,
        hub::GET_STATUS,
        0,
        port,
        4,
    )
}

/// Build a SET_PORT_FEATURE setup packet
pub fn set_port_feature_setup(port: u16, feature: u16) -> SetupPacket {
    SetupPacket::new(
        hub::RT_PORT_SET,
        hub::SET_FEATURE,
        feature,
        port,
        0,
    )
}

/// Build a CLEAR_PORT_FEATURE setup packet
pub fn clear_port_feature_setup(port: u16, feature: u16) -> SetupPacket {
    SetupPacket::new(
        hub::RT_PORT_SET,
        hub::CLEAR_FEATURE,
        feature,
        port,
        0,
    )
}

/// Build a SET_HUB_DEPTH setup packet (USB 3.0)
pub fn set_hub_depth_setup(depth: u16) -> SetupPacket {
    SetupPacket::new(
        0x20,  // Host-to-device, Class, Device
        hub::SET_HUB_DEPTH,
        depth,
        0,
        0,
    )
}

/// Parse hub descriptor from raw bytes
pub fn parse_ss_hub_descriptor(data: &[u8]) -> Option<HubInfo> {
    if data.len() < 12 {
        return None;
    }

    let desc_len = data[0];
    let desc_type = data[1];

    if desc_type != usb_req::DESC_SS_HUB as u8 {
        return None;
    }

    Some(HubInfo {
        num_ports: data[2],
        characteristics: u16::from_le_bytes([data[3], data[4]]),
        power_on_delay: data[5] * 2,  // In 2ms units
        hub_current: data[6],
        header_decode_latency: data[7],
        hub_delay: u16::from_le_bytes([data[8], data[9]]),
        removable_bitmap: u16::from_le_bytes([data[10], data[11]]),
        is_ss: true,
        raw_length: desc_len,
    })
}

/// Parsed hub information
#[derive(Copy, Clone, Debug)]
pub struct HubInfo {
    pub num_ports: u8,
    pub characteristics: u16,
    pub power_on_delay: u8,  // In milliseconds
    pub hub_current: u8,     // In mA
    pub header_decode_latency: u8,
    pub hub_delay: u16,
    pub removable_bitmap: u16,
    pub is_ss: bool,
    pub raw_length: u8,
}

impl HubInfo {
    /// Check if hub is compound device
    pub fn is_compound(&self) -> bool {
        (self.characteristics & 0x04) != 0
    }

    /// Get over-current protection mode
    pub fn overcurrent_mode(&self) -> OverCurrentMode {
        match (self.characteristics >> 3) & 0x03 {
            0 => OverCurrentMode::Global,
            1 => OverCurrentMode::Individual,
            _ => OverCurrentMode::None,
        }
    }
}

/// Over-current protection mode
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum OverCurrentMode {
    Global,
    Individual,
    None,
}

/// Parsed port status
#[derive(Copy, Clone, Debug)]
pub struct ParsedPortStatus {
    pub connected: bool,
    pub enabled: bool,
    pub suspended: bool,
    pub over_current: bool,
    pub reset: bool,
    pub powered: bool,
    pub low_speed: bool,
    pub high_speed: bool,
    pub change_connection: bool,
    pub change_enable: bool,
    pub change_suspend: bool,
    pub change_over_current: bool,
    pub change_reset: bool,
}

impl ParsedPortStatus {
    /// Parse from raw PortStatus
    pub fn from_raw(status: PortStatus) -> Self {
        let s = status.status;
        let c = status.change;
        Self {
            connected: (s & port_status::CONNECTION) != 0,
            enabled: (s & port_status::ENABLE) != 0,
            suspended: (s & port_status::SUSPEND) != 0,
            over_current: (s & port_status::OVER_CURRENT) != 0,
            reset: (s & port_status::RESET) != 0,
            powered: (s & port_status::POWER) != 0,
            low_speed: (s & port_status::LOW_SPEED) != 0,
            high_speed: (s & port_status::HIGH_SPEED) != 0,
            change_connection: (c & port_change::C_CONNECTION) != 0,
            change_enable: (c & port_change::C_ENABLE) != 0,
            change_suspend: (c & port_change::C_SUSPEND) != 0,
            change_over_current: (c & port_change::C_OVER_CURRENT) != 0,
            change_reset: (c & port_change::C_RESET) != 0,
        }
    }
}

/// Determine device speed from port status
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DeviceSpeed {
    Low,       // 1.5 Mbps
    Full,      // 12 Mbps
    High,      // 480 Mbps
    Super,     // 5 Gbps
    SuperPlus, // 10 Gbps
}

impl DeviceSpeed {
    /// Determine speed from port status (for USB 2.0 hubs)
    pub fn from_port_status(status: &ParsedPortStatus) -> Self {
        if status.low_speed {
            DeviceSpeed::Low
        } else if status.high_speed {
            DeviceSpeed::High
        } else {
            DeviceSpeed::Full
        }
    }

    /// Get xHCI speed ID
    pub fn to_xhci_speed(&self) -> u8 {
        match self {
            DeviceSpeed::Low => 2,
            DeviceSpeed::Full => 1,
            DeviceSpeed::High => 3,
            DeviceSpeed::Super => 4,
            DeviceSpeed::SuperPlus => 5,
        }
    }
}
