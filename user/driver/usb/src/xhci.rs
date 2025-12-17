//! xHCI register definitions

// =============================================================================
// xHCI registers
// =============================================================================

pub mod xhci_cap {
    pub const CAPLENGTH: usize = 0x00;      // Also contains HCIVERSION in upper 16 bits
    pub const HCSPARAMS1: usize = 0x04;     // Structural parameters 1
    pub const HCSPARAMS2: usize = 0x08;     // Structural parameters 2
    pub const HCSPARAMS3: usize = 0x0C;     // Structural parameters 3
    pub const HCCPARAMS1: usize = 0x10;     // Capability parameters 1
    pub const DBOFF: usize = 0x14;          // Doorbell offset
    pub const RTSOFF: usize = 0x18;         // Runtime register space offset
}

pub mod xhci_op {
    pub const USBCMD: usize = 0x00;
    pub const USBSTS: usize = 0x04;
    pub const PAGESIZE: usize = 0x08;
    pub const DNCTRL: usize = 0x14;         // Device notification control
    pub const CRCR: usize = 0x18;           // Command ring control (64-bit)
    pub const DCBAAP: usize = 0x30;         // Device context base address array pointer (64-bit)
    pub const CONFIG: usize = 0x38;         // Configure register
}

pub mod usbcmd {
    pub const RUN: u32 = 1 << 0;
    pub const HCRST: u32 = 1 << 1;
    pub const INTE: u32 = 1 << 2;           // Interrupter enable
    pub const HSEE: u32 = 1 << 3;           // Host system error enable
}

pub mod usbsts {
    pub const HCH: u32 = 1 << 0;            // HC halted
    pub const HSE: u32 = 1 << 2;            // Host system error
    pub const EINT: u32 = 1 << 3;           // Event interrupt
    pub const PCD: u32 = 1 << 4;            // Port change detect
    pub const CNR: u32 = 1 << 11;           // Controller not ready
}

pub mod xhci_port {
    pub const PORTSC: usize = 0x00;
    pub const PORTPMSC: usize = 0x04;
    pub const PORTLI: usize = 0x08;
}

// Runtime registers (offset from RTSOFF)
pub mod xhci_rt {
    pub const MFINDEX: usize = 0x00;        // Microframe index
    // Interrupter register set starts at offset 0x20
    pub const IR0: usize = 0x20;            // Interrupter 0 base
}

// Interrupter register set (relative to IR base)
pub mod xhci_ir {
    pub const IMAN: usize = 0x00;           // Interrupter management
    pub const IMOD: usize = 0x04;           // Interrupter moderation
    pub const ERSTSZ: usize = 0x08;         // Event ring segment table size
    pub const ERSTBA: usize = 0x10;         // Event ring segment table base address (64-bit)
    pub const ERDP: usize = 0x18;           // Event ring dequeue pointer (64-bit)
}
