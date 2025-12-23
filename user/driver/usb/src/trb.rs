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

    // === Transfer Event TRB helpers ===
    // For parsing TRANSFER_EVENT TRBs from the event ring

    /// For Transfer Event TRBs: extract slot_id from bits [31:24] of control
    #[inline]
    pub fn event_slot_id(&self) -> u32 {
        (self.control >> 24) & 0xFF
    }

    /// For Transfer Event TRBs: extract endpoint DCI from bits [20:16] of control
    #[inline]
    pub fn event_endpoint_id(&self) -> u32 {
        (self.control >> 16) & 0x1F
    }

    /// For Transfer Event TRBs: get completion code from bits [31:24] of status
    #[inline]
    pub fn event_completion_code(&self) -> u32 {
        (self.status >> 24) & 0xFF
    }

    /// For Transfer Event TRBs: get transfer length/residue from bits [23:0] of status
    #[inline]
    pub fn event_residue(&self) -> u32 {
        self.status & 0xFFFFFF
    }

    /// For Transfer Event TRBs: get the TRB pointer (address of completed TRB)
    #[inline]
    pub fn event_trb_pointer(&self) -> u64 {
        self.param
    }

    /// For Transfer Event TRBs: check if this event matches expected slot and endpoint
    #[inline]
    pub fn matches_transfer(&self, slot_id: u32, endpoint_dci: u32) -> bool {
        self.event_slot_id() == slot_id && self.event_endpoint_id() == endpoint_dci
    }

    /// For Transfer Event TRBs: check if this event matches slot, endpoint, AND TRB pointer
    /// This is the strictest matching - ensures we got completion for the exact TRB we posted
    #[inline]
    pub fn matches_transfer_exact(&self, slot_id: u32, endpoint_dci: u32, trb_phys: u64) -> bool {
        self.event_slot_id() == slot_id
            && self.event_endpoint_id() == endpoint_dci
            && self.event_trb_pointer() == trb_phys
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
    pub const NOOP_TRANSFER: u32 = 8;  // No-Op for transfer rings

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

// TRB Control Field Bit Positions
pub mod trb_ctrl {
    /// Cycle bit (bit 0) - indicates TRB ownership
    pub const CYCLE: u32 = 1 << 0;
    /// Toggle Cycle bit (bit 1) - for Link TRBs
    pub const TC: u32 = 1 << 1;
    /// Interrupt on Short Packet (bit 2)
    pub const ISP: u32 = 1 << 2;
    /// No Snoop (bit 3)
    pub const NS: u32 = 1 << 3;
    /// Chain bit (bit 4)
    pub const CH: u32 = 1 << 4;
    /// Interrupt On Completion (bit 5)
    pub const IOC: u32 = 1 << 5;
    /// Immediate Data (bit 6) - for Setup TRBs
    pub const IDT: u32 = 1 << 6;
    /// Block Set Address Request (bit 9) - for Address Device
    pub const BSR: u32 = 1 << 9;
    /// Direction bit (bit 16) - for Data/Status TRBs (1=IN, 0=OUT)
    pub const DIR_IN: u32 = 1 << 16;
    /// Transfer Type field (bits 16-17) for Setup TRB
    pub const TRT_NO_DATA: u32 = 0 << 16;
    pub const TRT_OUT: u32 = 2 << 16;
    pub const TRT_IN: u32 = 3 << 16;
}
