//! xHCI Transfer Request Block (TRB) structures

// =============================================================================
// xHCI Data Structures
// =============================================================================

/// Transfer Request Block (TRB) - 16 bytes
/// Used in Command Ring, Event Ring, and Transfer Rings
#[repr(C, align(16))]
#[derive(Clone, Copy, Default)]
pub struct Trb {
    pub param: u64,         // Parameter (varies by TRB type)
    pub status: u32,        // Status/transfer length
    pub control: u32,       // Control: cycle bit, TRB type, flags
}

impl Trb {
    pub const fn new() -> Self {
        Self { param: 0, status: 0, control: 0 }
    }

    pub fn set_type(&mut self, trb_type: u32) {
        self.control = (self.control & !0xFC00) | ((trb_type & 0x3F) << 10);
    }

    pub fn get_type(&self) -> u32 {
        (self.control >> 10) & 0x3F
    }

    pub fn set_cycle(&mut self, cycle: bool) {
        if cycle {
            self.control |= 1;
        } else {
            self.control &= !1;
        }
    }

    pub fn get_cycle(&self) -> bool {
        (self.control & 1) != 0
    }
}

// TRB Types
pub mod trb_type {
    // Transfer TRB types
    pub const NORMAL: u32 = 1;
    pub const SETUP: u32 = 2;
    pub const DATA: u32 = 3;
    pub const STATUS: u32 = 4;
    pub const LINK: u32 = 6;

    // Command TRB types
    pub const ENABLE_SLOT: u32 = 9;
    pub const DISABLE_SLOT: u32 = 10;
    pub const ADDRESS_DEVICE: u32 = 11;
    pub const CONFIGURE_ENDPOINT: u32 = 12;
    pub const EVALUATE_CONTEXT: u32 = 13;
    pub const RESET_ENDPOINT: u32 = 14;
    pub const STOP_ENDPOINT: u32 = 15;
    pub const SET_TR_DEQUEUE: u32 = 16;
    pub const NOOP_CMD: u32 = 23;

    // Event TRB types
    pub const TRANSFER_EVENT: u32 = 32;
    pub const COMMAND_COMPLETION: u32 = 33;
    pub const PORT_STATUS_CHANGE: u32 = 34;
    pub const HOST_CONTROLLER: u32 = 37;
}

// TRB Completion Codes
pub mod trb_cc {
    pub const SUCCESS: u32 = 1;
    pub const SHORT_PACKET: u32 = 13;
    pub const STALL: u32 = 6;
    pub const USB_TRANSACTION_ERROR: u32 = 4;
    pub const TRB_ERROR: u32 = 5;
    pub const BABBLE: u32 = 3;
}
