//! xHCI Register Definitions
//!
//! Standard xHCI register offsets and bit definitions as per the xHCI specification.
//! This file is 100% portable across all xHCI implementations.

// =============================================================================
// Capability Registers (read-only)
// =============================================================================

pub mod cap {
    /// Capability register length (also contains HCIVERSION in upper 16 bits)
    pub const CAPLENGTH: usize = 0x00;
    /// Structural parameters 1 (max slots, max intrs, max ports)
    pub const HCSPARAMS1: usize = 0x04;
    /// Structural parameters 2 (IST, ERST max, SPR, SPB max)
    pub const HCSPARAMS2: usize = 0x08;
    /// Structural parameters 3 (U1/U2 latency)
    pub const HCSPARAMS3: usize = 0x0C;
    /// Capability parameters 1 (64-bit addressing, etc.)
    pub const HCCPARAMS1: usize = 0x10;
    /// Doorbell array offset from base
    pub const DBOFF: usize = 0x14;
    /// Runtime register space offset from base
    pub const RTSOFF: usize = 0x18;
    /// Capability parameters 2 (xHCI 1.1+)
    pub const HCCPARAMS2: usize = 0x1C;

    // HCSPARAMS1 bit fields
    pub const HCSPARAMS1_MAX_SLOTS_MASK: u32 = 0xFF;
    pub const HCSPARAMS1_MAX_INTRS_SHIFT: u32 = 8;
    pub const HCSPARAMS1_MAX_INTRS_MASK: u32 = 0x7FF << 8;
    pub const HCSPARAMS1_MAX_PORTS_SHIFT: u32 = 24;
    pub const HCSPARAMS1_MAX_PORTS_MASK: u32 = 0xFF << 24;
}

// =============================================================================
// Operational Registers (base + CAPLENGTH)
// =============================================================================

pub mod op {
    /// USB Command register
    pub const USBCMD: usize = 0x00;
    /// USB Status register
    pub const USBSTS: usize = 0x04;
    /// Page size register
    pub const PAGESIZE: usize = 0x08;
    /// Device notification control
    pub const DNCTRL: usize = 0x14;
    /// Command ring control register (64-bit)
    pub const CRCR: usize = 0x18;
    /// Device context base address array pointer (64-bit)
    pub const DCBAAP: usize = 0x30;
    /// Configure register (max slots enabled)
    pub const CONFIG: usize = 0x38;
}

/// USBCMD bits
pub mod usbcmd {
    /// Run/Stop - set to 1 to run
    pub const RUN: u32 = 1 << 0;
    /// Host Controller Reset
    pub const HCRST: u32 = 1 << 1;
    /// Interrupter Enable
    pub const INTE: u32 = 1 << 2;
    /// Host System Error Enable
    pub const HSEE: u32 = 1 << 3;
    /// Light Host Controller Reset (xHCI 1.0+)
    pub const LHCRST: u32 = 1 << 7;
    /// Controller Save State
    pub const CSS: u32 = 1 << 8;
    /// Controller Restore State
    pub const CRS: u32 = 1 << 9;
    /// Enable Wrap Event
    pub const EWE: u32 = 1 << 10;
    /// Enable U3 MFINDEX Stop
    pub const EU3S: u32 = 1 << 11;
}

/// USBSTS bits
pub mod usbsts {
    /// HC Halted - controller has stopped
    pub const HCH: u32 = 1 << 0;
    /// Host System Error
    pub const HSE: u32 = 1 << 2;
    /// Event Interrupt - an event is pending
    pub const EINT: u32 = 1 << 3;
    /// Port Change Detect
    pub const PCD: u32 = 1 << 4;
    /// Save State Status (xHCI 1.0+)
    pub const SSS: u32 = 1 << 8;
    /// Restore State Status (xHCI 1.0+)
    pub const RSS: u32 = 1 << 9;
    /// Save/Restore Error (xHCI 1.0+)
    pub const SRE: u32 = 1 << 10;
    /// Controller Not Ready
    pub const CNR: u32 = 1 << 11;
    /// Host Controller Error
    pub const HCE: u32 = 1 << 12;
}

// =============================================================================
// Port Register Set (base + CAPLENGTH + 0x400 + 0x10*port)
// =============================================================================

pub mod port {
    /// Port status and control
    pub const PORTSC: usize = 0x00;
    /// Port power management status and control
    pub const PORTPMSC: usize = 0x04;
    /// Port link info
    pub const PORTLI: usize = 0x08;
    /// Port hardware LPM control (xHCI 1.1+)
    pub const PORTHLPMC: usize = 0x0C;
}

/// PORTSC bits
pub mod portsc {
    /// Current connect status
    pub const CCS: u32 = 1 << 0;
    /// Port enabled/disabled
    pub const PED: u32 = 1 << 1;
    /// Over-current active
    pub const OCA: u32 = 1 << 3;
    /// Port reset
    pub const PR: u32 = 1 << 4;
    /// Port link state (bits 8:5)
    pub const PLS_MASK: u32 = 0xF << 5;
    pub const PLS_SHIFT: u32 = 5;
    /// Port power
    pub const PP: u32 = 1 << 9;
    /// Port speed (bits 13:10)
    pub const SPEED_MASK: u32 = 0xF << 10;
    pub const SPEED_SHIFT: u32 = 10;
    /// Port indicator control (bits 15:14)
    pub const PIC_MASK: u32 = 0x3 << 14;
    /// Port link state write strobe
    pub const LWS: u32 = 1 << 16;
    /// Connect status change
    pub const CSC: u32 = 1 << 17;
    /// Port enabled/disabled change
    pub const PEC: u32 = 1 << 18;
    /// Warm port reset change (USB3 only)
    pub const WRC: u32 = 1 << 19;
    /// Over-current change
    pub const OCC: u32 = 1 << 20;
    /// Port reset change
    pub const PRC: u32 = 1 << 21;
    /// Port link state change
    pub const PLC: u32 = 1 << 22;
    /// Port config error change (xHCI 1.1+)
    pub const CEC: u32 = 1 << 23;
    /// Cold attach status (USB3)
    pub const CAS: u32 = 1 << 24;
    /// Wake on connect enable
    pub const WCE: u32 = 1 << 25;
    /// Wake on disconnect enable
    pub const WDE: u32 = 1 << 26;
    /// Wake on over-current enable
    pub const WOE: u32 = 1 << 27;
    /// Device removable (read-only)
    pub const DR: u32 = 1 << 30;
    /// Warm port reset (USB3 only)
    pub const WPR: u32 = 1 << 31;

    /// Port link state values
    pub mod pls {
        pub const U0: u32 = 0;          // USB3 link on
        pub const U1: u32 = 1;          // USB3 standby
        pub const U2: u32 = 2;          // USB3 sleep
        pub const U3: u32 = 3;          // USB3 suspend
        pub const DISABLED: u32 = 4;
        pub const RX_DETECT: u32 = 5;
        pub const INACTIVE: u32 = 6;
        pub const POLLING: u32 = 7;
        pub const RECOVERY: u32 = 8;
        pub const HOT_RESET: u32 = 9;
        pub const COMPLIANCE: u32 = 10;
        pub const TEST_MODE: u32 = 11;
        pub const RESUME: u32 = 15;
    }

    /// Port speed values
    pub mod speed {
        pub const FULL: u32 = 1;        // 12 Mb/s
        pub const LOW: u32 = 2;         // 1.5 Mb/s
        pub const HIGH: u32 = 3;        // 480 Mb/s
        pub const SUPER: u32 = 4;       // 5 Gb/s
        pub const SUPER_PLUS: u32 = 5;  // 10 Gb/s
    }

    /// Bits that are write-1-to-clear (RW1C) - don't write 1 accidentally
    pub const RW1C_BITS: u32 = CSC | PEC | WRC | OCC | PRC | PLC | CEC;

    /// Bits that are preserved on write (RsvdP)
    pub const PRESERVE_BITS: u32 = CCS | OCA | PLS_MASK | PP | SPEED_MASK | PIC_MASK | DR;
}

// =============================================================================
// Runtime Registers (base + RTSOFF)
// =============================================================================

pub mod rt {
    /// Microframe index register
    pub const MFINDEX: usize = 0x00;
    /// Interrupter register set 0 base (first interrupter)
    pub const IR0: usize = 0x20;
    /// Interrupter register set spacing
    pub const IR_SPACING: usize = 0x20;
}

/// Interrupter register set (relative to IR base)
pub mod ir {
    /// Interrupter management
    pub const IMAN: usize = 0x00;
    /// Interrupter moderation
    pub const IMOD: usize = 0x04;
    /// Event ring segment table size
    pub const ERSTSZ: usize = 0x08;
    /// Event ring segment table base address (64-bit)
    pub const ERSTBA: usize = 0x10;
    /// Event ring dequeue pointer (64-bit)
    pub const ERDP: usize = 0x18;
}

/// IMAN bits
pub mod iman {
    /// Interrupt pending
    pub const IP: u32 = 1 << 0;
    /// Interrupt enable
    pub const IE: u32 = 1 << 1;
}

// =============================================================================
// Doorbell Register (base + DBOFF)
// =============================================================================

pub mod doorbell {
    /// Doorbell target for host controller (command ring)
    pub const HOST_CONTROLLER: u32 = 0;

    /// Create doorbell value from slot and target
    #[inline]
    pub const fn value(target: u8, stream_id: u16) -> u32 {
        (target as u32) | ((stream_id as u32) << 16)
    }

    /// Doorbell target for control endpoint (EP0)
    pub const EP0: u8 = 1;
}
