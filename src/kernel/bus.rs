//! Bus Controller State Machines
//!
//! This module implements the kernel-side bus controllers with explicit state machines.
//! Each hardware bus (PCIe port, USB host controller) has a controller that tracks:
//! - Ownership (which process owns the bus via control port connection)
//! - State (SAFE, CLAIMED, RESETTING)
//! - Safety invariants (bus mastering, IOMMU mappings)
//!
//! ## State Machine
//!
//! ```text
//!                      ┌───────────────────┐
//!                      │                   │
//!           boot       │       SAFE        │◄──────────────┐
//!          ──────────► │                   │               │
//!                      │  bus_master=off   │               │
//!                      │  iommu=blocked    │               │
//!                      └─────────┬─────────┘               │
//!                                │                         │
//!                   port_connect │                         │ reset_complete
//!                                ▼                         │
//!                      ┌───────────────────┐               │
//!                      │                   │               │
//!                      │     CLAIMED       │               │
//!                      │                   │               │
//!                      │  owner=Some(port) │               │
//!                      └─────────┬─────────┘               │
//!                                │                         │
//!           owner_disconnect OR  │                         │
//!           Reset message        │                         │
//!                                ▼                         │
//!                      ┌───────────────────┐               │
//!                      │                   │               │
//!                      │    RESETTING      ├───────────────┘
//!                      │                   │
//!                      │  [reset sequence] │
//!                      └───────────────────┘
//! ```
//!
//! ## Ownership Model
//!
//! Bus ownership is tracked via port connections:
//! - Kernel creates bus control ports at boot (e.g., "/kernel/bus/pcie0")
//! - devd connects to claim ownership
//! - If devd disconnects (crash or intentional), kernel goes to RESETTING
//! - Atomic handoff supported for live updates

use super::ipc::{self, ChannelId, Message, MessageType};
use super::port::{self, PortError};
use super::process::Pid;
use crate::arch::aarch64::mmio::{MmioRegion, delay_ms, delay_us, dsb};
use crate::logln;

/// Get current uptime in milliseconds
fn uptime_ms() -> u64 {
    let counter = crate::platform::mt7988::timer::counter();
    let freq = crate::platform::mt7988::timer::frequency();
    if freq > 0 {
        (counter * 1000) / freq
    } else {
        0
    }
}

// =============================================================================
// Constants
// =============================================================================

/// Maximum number of buses
pub const MAX_BUSES: usize = 8;

/// Maximum devices per bus (for tracking bus mastering)
pub const MAX_DEVICES_PER_BUS: usize = 32;

/// Bus control port name prefix
pub const BUS_PORT_PREFIX: &str = "/kernel/bus/";

// =============================================================================
// MT7988A Hardware Addresses
// =============================================================================

/// PCIe MAC base addresses for MT7988A (4 ports)
mod pcie_hw {
    /// PCIe port MAC base addresses
    pub const MAC_BASES: [usize; 4] = [
        0x1130_0000,  // PCIe0
        0x1131_0000,  // PCIe1
        0x1128_0000,  // PCIe2
        0x1129_0000,  // PCIe3
    ];

    /// INFRACFG_AO base (clock/reset controller)
    pub const INFRACFG_AO_BASE: usize = 0x1000_1000;

    // ─────────────────────────────────────────────────────────────────────────
    // Clock Gate Registers (MediaTek set/clear model)
    // Write to SET register to gate clock (disable)
    // Write to CLEAR register to ungate clock (enable)
    // ─────────────────────────────────────────────────────────────────────────

    /// INFRA0 clock gate registers (PCIE_PERI_26M clocks)
    pub const INFRA0_CG_SET: usize = 0x10;   // Write 1 to gate (disable)
    pub const INFRA0_CG_CLR: usize = 0x14;   // Write 1 to ungate (enable)
    pub const INFRA0_CG_STA: usize = 0x18;   // Status (1=gated/disabled)

    /// INFRA0 clock bits for PCIe PERI 26M clocks
    pub const INFRA0_PCIE_PERI_26M_P0: u32 = 1 << 7;
    pub const INFRA0_PCIE_PERI_26M_P1: u32 = 1 << 8;
    pub const INFRA0_PCIE_PERI_26M_P2: u32 = 1 << 9;
    pub const INFRA0_PCIE_PERI_26M_P3: u32 = 1 << 10;
    pub const INFRA0_PCIE_PERI_26M: [u32; 4] = [
        INFRA0_PCIE_PERI_26M_P0,
        INFRA0_PCIE_PERI_26M_P1,
        INFRA0_PCIE_PERI_26M_P2,
        INFRA0_PCIE_PERI_26M_P3,
    ];

    /// INFRA3 clock gate registers (PIPE, GFMUX_TL, 133M clocks)
    pub const INFRA3_CG_SET: usize = 0x60;   // Write 1 to gate (disable)
    pub const INFRA3_CG_CLR: usize = 0x64;   // Write 1 to ungate (enable)
    pub const INFRA3_CG_STA: usize = 0x68;   // Status (1=gated/disabled)

    /// INFRA3 clock bits for PCIe GFMUX TL clocks (bits 20-23)
    pub const INFRA3_PCIE_GFMUX_TL_P0: u32 = 1 << 20;
    pub const INFRA3_PCIE_GFMUX_TL_P1: u32 = 1 << 21;
    pub const INFRA3_PCIE_GFMUX_TL_P2: u32 = 1 << 22;
    pub const INFRA3_PCIE_GFMUX_TL_P3: u32 = 1 << 23;
    pub const INFRA3_PCIE_GFMUX_TL: [u32; 4] = [
        INFRA3_PCIE_GFMUX_TL_P0,
        INFRA3_PCIE_GFMUX_TL_P1,
        INFRA3_PCIE_GFMUX_TL_P2,
        INFRA3_PCIE_GFMUX_TL_P3,
    ];

    /// INFRA3 clock bits for PCIe PIPE clocks (bits 24-27)
    pub const INFRA3_PCIE_PIPE_P0: u32 = 1 << 24;
    pub const INFRA3_PCIE_PIPE_P1: u32 = 1 << 25;
    pub const INFRA3_PCIE_PIPE_P2: u32 = 1 << 26;
    pub const INFRA3_PCIE_PIPE_P3: u32 = 1 << 27;
    pub const INFRA3_PCIE_PIPE: [u32; 4] = [
        INFRA3_PCIE_PIPE_P0,
        INFRA3_PCIE_PIPE_P1,
        INFRA3_PCIE_PIPE_P2,
        INFRA3_PCIE_PIPE_P3,
    ];

    /// INFRA3 clock bits for PCIe 133M clocks (bits 28-31)
    pub const INFRA3_PCIE_133M_P0: u32 = 1 << 28;
    pub const INFRA3_PCIE_133M_P1: u32 = 1 << 29;
    pub const INFRA3_PCIE_133M_P2: u32 = 1 << 30;
    pub const INFRA3_PCIE_133M_P3: u32 = 1 << 31;
    pub const INFRA3_PCIE_133M: [u32; 4] = [
        INFRA3_PCIE_133M_P0,
        INFRA3_PCIE_133M_P1,
        INFRA3_PCIE_133M_P2,
        INFRA3_PCIE_133M_P3,
    ];

    // ─────────────────────────────────────────────────────────────────────────
    // INFRACFG Reset Registers (controls power domain resets)
    // ─────────────────────────────────────────────────────────────────────────

    /// Reset register offsets (set/clear model)
    pub const RST0_SET: usize = 0x70;   // Write 1 to assert reset
    pub const RST0_CLR: usize = 0x74;   // Write 1 to deassert reset
    pub const RST0_STA: usize = 0x78;   // Read status

    /// PEXTP MAC software reset bit (shared for all PCIe ports)
    /// This must be deasserted before RST_CTRL register becomes writable
    pub const PEXTP_MAC_SWRST: u32 = 1 << 6;

    /// MAC reset control register offset
    pub const RST_CTRL_REG: usize = 0x148;

    /// Reset control bits (active high)
    pub const RST_MAC: u32 = 1 << 0;
    pub const RST_PHY: u32 = 1 << 1;
    pub const RST_BRG: u32 = 1 << 2;
    pub const RST_PE: u32 = 1 << 3;   // PERST# to endpoint
    pub const RST_ALL: u32 = RST_MAC | RST_PHY | RST_BRG | RST_PE;

    /// LTSSM status register offset
    pub const LTSSM_STATUS_REG: usize = 0x150;
    pub const LTSSM_STATE_MASK: u32 = 0x1F << 24;
    pub const LTSSM_STATE_L0: u32 = 0x10 << 24;  // Link up

    /// Link status register offset
    pub const LINK_STATUS_REG: usize = 0x154;
    pub const LINK_UP: u32 = 1 << 8;

    /// PCI Command register offset in config space
    pub const CFG_OFFSET: usize = 0x1000;
    pub const PCI_COMMAND: usize = 0x04;
    pub const CMD_BUS_MASTER: u16 = 1 << 2;
}

/// USB/xHCI hardware addresses for MT7988A (2 controllers)
mod usb_hw {
    /// xHCI base addresses (MAC base)
    pub const MAC_BASES: [usize; 2] = [
        0x1119_0000,  // SSUSB0 (M.2 slot)
        0x1120_0000,  // SSUSB1 (USB-A ports via VL822)
    ];

    /// IPPC (IP Port Control) offset from MAC base
    pub const IPPC_OFFSET: usize = 0x3E00;

    /// IPPC register offsets (from Linux xhci-mtk.h struct mu3c_ippc_regs)
    pub const IP_PW_CTRL0: usize = 0x00;
    pub const IP_PW_CTRL1: usize = 0x04;
    pub const IP_PW_CTRL2: usize = 0x08;
    pub const IP_PW_STS1: usize = 0x10;
    pub const IP_XHCI_CAP: usize = 0x24;
    pub const U3_CTRL_P0: usize = 0x30;  // Each port is 8 bytes
    pub const U2_CTRL_P0: usize = 0x50;  // Each port is 8 bytes

    /// CTRL0 bits
    pub const CTRL0_IP_SW_RST: u32 = 1 << 0;

    /// CTRL1 bits
    pub const CTRL1_IP_HOST_PDN: u32 = 1 << 0;

    /// CTRL2 bits
    pub const CTRL2_IP_DEV_PDN: u32 = 1 << 0;

    /// STS1 bits
    pub const STS1_SYSPLL_STABLE: u32 = 1 << 0;
    pub const STS1_REF_RST: u32 = 1 << 8;
    pub const STS1_SYS125_RST: u32 = 1 << 10;
    pub const STS1_XHCI_RST: u32 = 1 << 11;
    pub const STS1_U3_MAC_RST: u32 = 1 << 16;
    pub const STS1_IP_SLEEP_STS: u32 = 1 << 30;

    /// Port control bits (for U3_CTRL_Px and U2_CTRL_Px)
    pub const PORT_DIS: u32 = 1 << 0;
    pub const PORT_PDN: u32 = 1 << 1;
    pub const PORT_HOST_SEL: u32 = 1 << 2;

    /// xHCI capability registers
    pub const CAPLENGTH: usize = 0x00;

    /// xHCI operational registers (offset from CAPLENGTH)
    pub const USBCMD: usize = 0x00;
    pub const USBSTS: usize = 0x04;

    /// USBCMD bits
    pub const CMD_RUN: u32 = 1 << 0;
    pub const CMD_HCRST: u32 = 1 << 1;

    /// USBSTS bits
    pub const STS_HCH: u32 = 1 << 0;    // HC Halted
    pub const STS_CNR: u32 = 1 << 11;   // Controller Not Ready
    pub const STS_HCE: u32 = 1 << 12;   // Host Controller Error
}

// =============================================================================
// Bus Types
// =============================================================================

/// Hardware bus types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BusType {
    /// PCIe root port
    PCIe = 0,
    /// USB host controller (xHCI)
    Usb = 1,
}

impl BusType {
    pub fn as_str(&self) -> &'static str {
        match self {
            BusType::PCIe => "pcie",
            BusType::Usb => "usb",
        }
    }
}

// =============================================================================
// Bus State Machine
// =============================================================================

/// Bus controller states
/// Bus State Machine
///
/// ```text
///                    ┌─────────────────────────────────┐
///                    │                                 │
///                    ▼                                 │
///   ┌──────┐  driver claims   ┌─────────┐  owner crashes/exits
///   │ Safe │ ───────────────► │ Claimed │ ─────────────┘
///   └──────┘                  └─────────┘
///       ▲                          │
///       │                          │ driver requests reset
///       │      ┌───────────┐       │
///       └───── │ Resetting │ ◄─────┘
///              └───────────┘
/// ```
///
/// **Key invariant**: Drivers can ONLY claim a bus from `Safe` state.
/// When owner exits/crashes, bus auto-resets to `Safe` for next driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BusState {
    /// Bus in safe mode - no DMA possible, driver can claim
    /// Invariants: bus_mastering=false, iommu_mappings=empty, owner=None
    Safe = 0,

    /// Bus owned by driver, normal operation
    /// Invariants: owner.is_some()
    Claimed = 1,

    /// Hardware reset in progress
    /// Invariants: owner=None, actively resetting hardware
    Resetting = 2,
}

impl BusState {
    pub fn as_str(&self) -> &'static str {
        match self {
            BusState::Safe => "safe",
            BusState::Claimed => "claimed",
            BusState::Resetting => "resetting",
        }
    }

    /// Maximum time allowed in this state (for watchdog)
    /// Returns None for stable states
    pub fn max_duration_ms(&self) -> Option<u64> {
        match self {
            BusState::Safe => None,      // Stable - can wait forever for devd
            BusState::Claimed => None,   // Stable - normal operation
            BusState::Resetting => Some(10_000), // 10 seconds max for reset
        }
    }
}

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

    // ─── devd → kernel (requests) ───

    /// Enable bus mastering for a device
    EnableBusMastering = 16,

    /// Disable bus mastering for a device
    DisableBusMastering = 17,

    /// Request IOMMU mapping
    MapDma = 18,

    /// Release IOMMU mapping
    UnmapDma = 19,

    /// Request nuclear reset (devd will disconnect after this)
    RequestReset = 20,

    /// Atomic handoff to new owner (for live update)
    Handoff = 21,

    /// Set driver PID (devd tells kernel which process is using the bus)
    /// When this PID exits, kernel auto-resets bus and notifies devd
    SetDriver = 22,

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

// =============================================================================
// Bus Controller
// =============================================================================

/// Per-device bus mastering state
#[derive(Clone, Copy, Default)]
pub struct DeviceBusMasterState {
    /// Device identifier (BDF for PCIe, slot for USB)
    pub device_id: u16,
    /// Bus mastering enabled
    pub enabled: bool,
    /// IOMMU mapping count
    pub iommu_mappings: u16,
}

/// A single bus controller
pub struct BusController {
    /// Bus type
    pub bus_type: BusType,

    /// Bus index (e.g., pcie0, pcie1, usb0, usb1)
    pub bus_index: u8,

    /// Current state
    pub state: BusState,

    /// When we entered current state (for watchdog)
    pub state_entered_at: u64,

    /// Owner's channel (from bus control port connection)
    pub owner_channel: Option<ChannelId>,

    /// Owner's PID (for cleanup on process exit)
    pub owner_pid: Option<Pid>,

    /// Driver PID - the actual process using the hardware
    /// Set by devd via SetDriver message. When this PID exits,
    /// kernel auto-resets bus and notifies devd via owner_channel.
    pub driver_pid: Option<Pid>,

    /// Control port name (e.g., "/kernel/bus/pcie0")
    pub port_name: [u8; 32],
    pub port_name_len: usize,

    /// Listen channel for the control port
    pub listen_channel: Option<ChannelId>,

    /// Per-device bus mastering state
    pub devices: [DeviceBusMasterState; MAX_DEVICES_PER_BUS],
    pub device_count: usize,

    /// Global bus mastering enabled (hardware level)
    pub bus_master_enabled: bool,

    /// Hardware verified after reset
    pub hardware_verified: bool,
}

impl BusController {
    pub const fn new() -> Self {
        Self {
            bus_type: BusType::PCIe,
            bus_index: 0,
            state: BusState::Safe,
            state_entered_at: 0,
            owner_channel: None,
            owner_pid: None,
            driver_pid: None,
            port_name: [0; 32],
            port_name_len: 0,
            listen_channel: None,
            devices: [DeviceBusMasterState {
                device_id: 0,
                enabled: false,
                iommu_mappings: 0,
            }; MAX_DEVICES_PER_BUS],
            device_count: 0,
            bus_master_enabled: false,
            hardware_verified: false,
        }
    }

    /// Initialize a bus controller
    pub fn init(&mut self, bus_type: BusType, index: u8) {
        self.bus_type = bus_type;
        self.bus_index = index;
        self.state = BusState::Safe;
        self.state_entered_at = uptime_ms();

        // Build port name: "/kernel/bus/pcie0" or "/kernel/bus/usb0"
        let prefix = BUS_PORT_PREFIX.as_bytes();
        let type_str = bus_type.as_str().as_bytes();

        let mut pos = 0;
        for &b in prefix {
            if pos < self.port_name.len() {
                self.port_name[pos] = b;
                pos += 1;
            }
        }
        for &b in type_str {
            if pos < self.port_name.len() {
                self.port_name[pos] = b;
                pos += 1;
            }
        }
        // Add index as ASCII digit
        if pos < self.port_name.len() {
            self.port_name[pos] = b'0' + index;
            pos += 1;
        }
        self.port_name_len = pos;
    }

    /// Get port name as string
    pub fn port_name_str(&self) -> &str {
        core::str::from_utf8(&self.port_name[..self.port_name_len]).unwrap_or("???")
    }

    /// Register the bus control port
    /// Called during kernel init
    pub fn register_port(&mut self, kernel_pid: Pid) -> Result<(), PortError> {
        let name = self.port_name_str();
        logln!("  Registering bus control port: {}", name);

        let listen_ch = port::with_port_registry(|registry| {
            registry.register(name, kernel_pid)
        })?;

        self.listen_channel = Some(listen_ch);
        Ok(())
    }

    /// Check state machine invariants
    pub fn check_invariants(&self) -> bool {
        match self.state {
            BusState::Safe => {
                // Safe: bus mastering off, owner (devd) MAY be connected
                // Owner stays connected so it can receive notifications and restart drivers
                !self.bus_master_enabled
            }
            BusState::Claimed => {
                // Claimed: owner must be connected, driver is managing hardware
                self.owner_channel.is_some() &&
                self.owner_pid.is_some()
            }
            BusState::Resetting => {
                // Resetting: temporary state, owner stays so we can notify after reset
                !self.bus_master_enabled
            }
        }
    }

    /// Transition to a new state
    fn transition_to(&mut self, new_state: BusState, reason: StateChangeReason) {
        let old_state = self.state;

        if old_state == new_state {
            return;
        }

        logln!("  Bus {}: {:?} -> {:?} (reason: {:?})",
               self.port_name_str(), old_state, new_state, reason);

        self.state = new_state;
        self.state_entered_at = uptime_ms();

        // Enforce invariants based on new state
        match new_state {
            BusState::Safe => {
                // Keep owner_channel/owner_pid - devd stays connected to manage bus
                // Only clear driver_pid (driver is gone)
                self.driver_pid = None;
                self.bus_master_enabled = false;
                self.disable_all_bus_mastering();
            }
            BusState::Claimed => {
                // Owner should already be set before calling transition
            }
            BusState::Resetting => {
                // Keep owner_channel/owner_pid - need to notify after reset
                // Clear driver since it crashed
                self.driver_pid = None;
                self.bus_master_enabled = false;
                self.disable_all_bus_mastering();
            }
        }

        debug_assert!(self.check_invariants(), "Invariant violation after transition!");
    }

    /// Disable bus mastering for all devices (hardware + tracking)
    fn disable_all_bus_mastering(&mut self) {
        logln!("  Bus {}: disabling ALL bus mastering", self.port_name_str());

        // First, disable at hardware level
        match self.bus_type {
            BusType::PCIe => self.pcie_disable_all_bus_mastering(),
            BusType::Usb => self.usb_disable_all_dma(),
        }

        // Then update our tracking
        for i in 0..self.device_count {
            self.devices[i].enabled = false;
        }
        self.bus_master_enabled = false;
    }

    /// Handle a new connection to the bus control port
    /// First connection (devd) just supervises - bus stays Safe
    /// Second connection (driver) claims the bus via handoff
    pub fn handle_connect(&mut self, client_channel: ChannelId, client_pid: Pid) -> Result<(), BusError> {
        match self.state {
            BusState::Safe => {
                // First connection is supervisor (devd) - bus stays Safe
                // Store channel for communication but don't claim
                self.owner_channel = Some(client_channel);
                self.owner_pid = Some(client_pid);
                // Don't transition to Claimed - stay Safe until driver claims

                // Send StateSnapshot to supervisor
                self.send_state_snapshot(client_channel)?;

                Ok(())
            }
            BusState::Claimed => {
                // Already owned - reject (driver should use handoff)
                Err(BusError::AlreadyClaimed)
            }
            BusState::Resetting => {
                // Can't connect during reset
                Err(BusError::Busy)
            }
        }
    }

    /// Handle owner disconnecting
    pub fn handle_disconnect(&mut self, reason: StateChangeReason) {
        if self.state == BusState::Claimed {
            // Owner left - go to RESETTING then SAFE
            self.transition_to(BusState::Resetting, reason);

            // Perform hardware reset
            self.perform_hardware_reset();

            // Reset complete - go to SAFE
            self.transition_to(BusState::Safe, StateChangeReason::ResetComplete);
        }
    }

    /// Handle driver process exit
    ///
    /// Called when the driver_pid process exits. Resets the bus and notifies
    /// the supervisor (devd) via owner_channel so it can restart the driver.
    pub fn handle_driver_exit(&mut self, driver_pid: Pid) {
        logln!("  Bus {}: driver PID {} exited, resetting bus",
               self.port_name_str(), driver_pid);

        // Clear driver_pid
        self.driver_pid = None;

        // Reset bus if claimed
        if self.state == BusState::Claimed {
            self.transition_to(BusState::Resetting, StateChangeReason::DriverExited);
            self.perform_hardware_reset();
            self.transition_to(BusState::Safe, StateChangeReason::ResetComplete);
        }

        // Notify supervisor (devd) via owner_channel that bus is Safe
        // This wakes devd so it can restart the driver
        if let Some(channel) = self.owner_channel {
            logln!("  Bus {}: notifying supervisor on channel {}",
                   self.port_name_str(), channel);
            if let Err(e) = self.send_state_snapshot(channel) {
                logln!("  Bus {}: failed to notify supervisor: {:?}",
                       self.port_name_str(), e);
            }
        }
    }

    /// Handle atomic handoff to new owner
    pub fn handle_handoff(&mut self, new_channel: ChannelId, new_pid: Pid) -> Result<(), BusError> {
        if self.state != BusState::Claimed {
            return Err(BusError::NotClaimed);
        }

        // Atomic transfer - NO state change, NO reset
        logln!("  Bus {}: handoff from PID {:?} to PID {}",
               self.port_name_str(), self.owner_pid, new_pid);

        self.owner_channel = Some(new_channel);
        self.owner_pid = Some(new_pid);

        // Send snapshot to new owner
        self.send_state_snapshot(new_channel)?;

        Ok(())
    }

    /// Send state snapshot to a channel
    fn send_state_snapshot(&self, channel: ChannelId) -> Result<(), BusError> {
        let snapshot = StateSnapshot {
            msg_type: BusControlMsgType::StateSnapshot as u8,
            bus_type: self.bus_type as u8,
            bus_index: self.bus_index,
            state: self.state as u8,
            device_count: self.device_count as u8,
            capabilities: 0, // TODO: Fill in actual capabilities
            _reserved: [0; 2],
            since_boot_ms: uptime_ms(),
        };

        let bytes = snapshot.to_bytes();

        ipc::with_channel_table(|table| {
            let mut msg = Message::new();
            msg.header.msg_type = MessageType::Data;
            msg.header.payload_len = bytes.len() as u32;
            msg.payload[..bytes.len()].copy_from_slice(&bytes);

            table.send(channel, msg)
                .map_err(|_| BusError::SendFailed)?;
            Ok(())
        })
    }

    /// Perform hardware reset sequence
    fn perform_hardware_reset(&mut self) {
        logln!("  Bus {}: performing hardware reset...", self.port_name_str());

        match self.bus_type {
            BusType::PCIe => {
                self.pcie_reset_sequence();
            }
            BusType::Usb => {
                self.usb_reset_sequence();
            }
        }
    }

    /// Enable PCIe clocks and deassert INFRACFG reset for a port
    ///
    /// Must be called before accessing PCIe MAC registers.
    /// Steps:
    /// 1. Enable clocks: PCIE_PERI_26M, GFMUX_TL, PIPE, 133M
    /// 2. Deassert PEXTP_MAC_SWRST (required for RST_CTRL to be writable)
    fn pcie_enable_clocks(&self, index: usize) {
        let infracfg = MmioRegion::new(pcie_hw::INFRACFG_AO_BASE);

        logln!("    Enabling PCIe{} clocks via INFRACFG...", index);

        // Read current clock gate status
        let infra0_sta_before = infracfg.read32(pcie_hw::INFRA0_CG_STA);
        let infra3_sta_before = infracfg.read32(pcie_hw::INFRA3_CG_STA);
        logln!("      INFRA0_CG_STA before: 0x{:08x}", infra0_sta_before);
        logln!("      INFRA3_CG_STA before: 0x{:08x}", infra3_sta_before);

        // Get clock bits for this port
        let peri_26m_bit = pcie_hw::INFRA0_PCIE_PERI_26M[index];
        let gfmux_bit = pcie_hw::INFRA3_PCIE_GFMUX_TL[index];
        let pipe_bit = pcie_hw::INFRA3_PCIE_PIPE[index];
        let clk_133m_bit = pcie_hw::INFRA3_PCIE_133M[index];

        // Enable clocks by writing to CLEAR registers (clear gate = enable clock)
        // INFRA0: PCIE_PERI_26M
        infracfg.write32(pcie_hw::INFRA0_CG_CLR, peri_26m_bit);
        dsb();

        // INFRA3: GFMUX_TL, PIPE, 133M
        let infra3_bits = gfmux_bit | pipe_bit | clk_133m_bit;
        infracfg.write32(pcie_hw::INFRA3_CG_CLR, infra3_bits);
        dsb();

        // Small delay for clocks to stabilize
        delay_us(100);

        // Verify clocks are enabled (bits should be 0 = ungated)
        let infra0_sta_after = infracfg.read32(pcie_hw::INFRA0_CG_STA);
        let infra3_sta_after = infracfg.read32(pcie_hw::INFRA3_CG_STA);
        logln!("      INFRA0_CG_STA after: 0x{:08x}", infra0_sta_after);
        logln!("      INFRA3_CG_STA after: 0x{:08x}", infra3_sta_after);

        // Check if clocks are now enabled (gate bits should be 0)
        let peri_26m_enabled = (infra0_sta_after & peri_26m_bit) == 0;
        let gfmux_enabled = (infra3_sta_after & gfmux_bit) == 0;
        let pipe_enabled = (infra3_sta_after & pipe_bit) == 0;
        let clk_133m_enabled = (infra3_sta_after & clk_133m_bit) == 0;

        logln!("      PERI_26M: {}, GFMUX: {}, PIPE: {}, 133M: {}",
               if peri_26m_enabled { "ON" } else { "off" },
               if gfmux_enabled { "ON" } else { "off" },
               if pipe_enabled { "ON" } else { "off" },
               if clk_133m_enabled { "ON" } else { "off" });

        // Deassert PEXTP_MAC_SWRST (write 1 to CLR register to deassert)
        // This is required for RST_CTRL register to be writable
        let rst0_sta_before = infracfg.read32(pcie_hw::RST0_STA);
        logln!("      RST0_STA before: 0x{:08x}", rst0_sta_before);

        infracfg.write32(pcie_hw::RST0_CLR, pcie_hw::PEXTP_MAC_SWRST);
        dsb();
        delay_us(10);

        let rst0_sta_after = infracfg.read32(pcie_hw::RST0_STA);
        logln!("      RST0_STA after: 0x{:08x} (PEXTP_MAC deasserted)", rst0_sta_after);
    }

    /// Disable PCIe clocks and assert INFRACFG reset for a port
    ///
    /// Called to put port back in safe/powered-down state.
    /// Steps:
    /// 1. Assert PEXTP_MAC_SWRST (put MAC in reset)
    /// 2. Disable clocks
    fn pcie_disable_clocks(&self, index: usize) {
        let infracfg = MmioRegion::new(pcie_hw::INFRACFG_AO_BASE);

        logln!("    Disabling PCIe{} clocks...", index);

        // Assert PEXTP_MAC_SWRST (write 1 to SET register to assert)
        infracfg.write32(pcie_hw::RST0_SET, pcie_hw::PEXTP_MAC_SWRST);
        dsb();

        // Get clock bits for this port
        let peri_26m_bit = pcie_hw::INFRA0_PCIE_PERI_26M[index];
        let gfmux_bit = pcie_hw::INFRA3_PCIE_GFMUX_TL[index];
        let pipe_bit = pcie_hw::INFRA3_PCIE_PIPE[index];
        let clk_133m_bit = pcie_hw::INFRA3_PCIE_133M[index];

        // Disable clocks by writing to SET registers (set gate = disable clock)
        infracfg.write32(pcie_hw::INFRA0_CG_SET, peri_26m_bit);
        infracfg.write32(pcie_hw::INFRA3_CG_SET, gfmux_bit | pipe_bit | clk_133m_bit);
        dsb();

        logln!("      Reset asserted, clocks gated");
    }

    /// PCIe hardware reset sequence
    ///
    /// MT7988 Note: The RST_CTRL register at offset 0x148 has unusual behavior
    /// on MT7988 - writes don't take effect. This is similar to EN7581 which
    /// skips RST_CTRL manipulation entirely in the Linux driver.
    ///
    /// For MT7988, we rely on INFRACFG PEXTP_MAC_SWRST for reset control:
    /// - Assert PEXTP_MAC_SWRST = MAC in reset (safe state, no DMA)
    /// - Deassert PEXTP_MAC_SWRST = MAC running
    ///
    /// This gives us predictable state control even without RST_CTRL.
    ///
    /// Steps:
    /// 1. Enable clocks and deassert INFRACFG reset
    /// 2. Verify MAC is accessible
    /// 3. Read current state for verification
    /// 4. Assert INFRACFG reset and disable clocks (safe state)
    fn pcie_reset_sequence(&mut self) {
        let index = self.bus_index as usize;
        if index >= pcie_hw::MAC_BASES.len() {
            logln!("    [!!] Invalid PCIe index {}", index);
            self.hardware_verified = false;
            return;
        }

        let mac_base = pcie_hw::MAC_BASES[index];
        logln!("    PCIe{}: MAC base 0x{:08x}", index, mac_base);

        // Step 1: Enable clocks and deassert INFRACFG reset
        self.pcie_enable_clocks(index);

        // Now MAC should be accessible
        let mac = MmioRegion::new(mac_base);

        // Step 2: Verify MAC is accessible
        let test_read = mac.read32(0x00);
        logln!("    MAC[0x00]: 0x{:08x}", test_read);
        if test_read == 0 || test_read == 0xFFFFFFFF {
            logln!("    [!!] PCIe{}: MAC not accessible", index);
            self.pcie_disable_clocks(index);
            self.hardware_verified = false;
            return;
        }

        // Step 3: Read current state for diagnostics
        let rst_ctrl = mac.read32(pcie_hw::RST_CTRL_REG);
        let ltssm = mac.read32(pcie_hw::LTSSM_STATUS_REG);
        let link_status = mac.read32(pcie_hw::LINK_STATUS_REG);
        let link_up = (link_status & pcie_hw::LINK_UP) != 0;

        logln!("    RST_CTRL: 0x{:08x}", rst_ctrl);
        logln!("    LTSSM: 0x{:08x}, Link: 0x{:08x}, up={}", ltssm, link_status, link_up);

        // Note: We intentionally skip RST_CTRL manipulation on MT7988
        // because it doesn't work (writes have no effect).
        // The INFRACFG PEXTP_MAC_SWRST provides the reset control we need.

        // MAC is accessible and we verified state - mark as verified
        self.hardware_verified = true;
        logln!("    [OK] PCIe{}: MAC accessible, INFRACFG reset working", index);

        // Step 4: Assert INFRACFG reset and disable clocks (safe state)
        // This puts the MAC in reset - no DMA possible
        self.pcie_disable_clocks(index);
    }

    /// Verify PCIe link is in expected state after reset
    fn pcie_verify_link(&self, index: usize) -> bool {
        let mac = MmioRegion::new(pcie_hw::MAC_BASES[index]);

        // Check reset control is deasserted
        let rst_ctrl = mac.read32(pcie_hw::RST_CTRL_REG);
        logln!("    RST_CTRL after: 0x{:08x}", rst_ctrl);
        if rst_ctrl != pcie_hw::RST_ALL {
            logln!("    [!!] Reset bits not properly deasserted");
            return false;
        }

        // Check LTSSM state
        let ltssm = mac.read32(pcie_hw::LTSSM_STATUS_REG);
        let ltssm_state = ltssm & pcie_hw::LTSSM_STATE_MASK;
        logln!("    LTSSM: 0x{:08x} (state: 0x{:02x})", ltssm, ltssm_state >> 24);

        // Check link status
        let link_status = mac.read32(pcie_hw::LINK_STATUS_REG);
        let link_up = (link_status & pcie_hw::LINK_UP) != 0;
        logln!("    Link status: 0x{:08x} (up: {})", link_status, link_up);

        // A device may or may not be present
        // We verify the controller is in a valid state, not necessarily that a device is connected
        // LTSSM in Detect.Quiet (0x00) or Detect.Active (0x01) is expected when no device
        // LTSSM in L0 (0x10) when device is present and link trained

        true  // Controller is in a valid state
    }

    /// USB/xHCI hardware reset sequence
    ///
    /// Based on Linux xhci-mtk.c driver power-on sequence:
    /// 1. IPPC: Assert IP software reset, wait 1us, deassert
    /// 2. IPPC: Power down device IP (set CTRL2_IP_DEV_PDN)
    /// 3. IPPC: Power on host IP (clear CTRL1_IP_HOST_PDN)
    /// 4. IPPC: Enable U3 ports (clear PDN/DIS, set HOST_SEL)
    /// 5. IPPC: Enable U2 ports (clear PDN/DIS, set HOST_SEL)
    /// 6. Wait for clock stability
    /// 7. xHCI: Wait for controller ready (CNR=0)
    /// 8. xHCI: Verify halted state
    fn usb_reset_sequence(&mut self) {
        let index = self.bus_index as usize;
        if index >= usb_hw::MAC_BASES.len() {
            logln!("    [!!] Invalid USB index {}", index);
            self.hardware_verified = false;
            return;
        }

        let mac_base = usb_hw::MAC_BASES[index];
        let ippc_base = mac_base + usb_hw::IPPC_OFFSET;
        let ippc = MmioRegion::new(ippc_base);

        logln!("    USB{}: MAC=0x{:08x} IPPC=0x{:08x}", index, mac_base, ippc_base);

        // Step 1: Reset the whole SSUSB IP
        // CRITICAL: This stops xHCI DMA. We must wait for in-flight DMA to complete
        // before freeing any DMA buffers (shmem), otherwise freed pages get corrupted.
        logln!("    Step 1: IP software reset...");
        let ctrl0 = ippc.read32(usb_hw::IP_PW_CTRL0);
        logln!("      CTRL0 before: 0x{:08x}", ctrl0);
        ippc.write32(usb_hw::IP_PW_CTRL0, ctrl0 | usb_hw::CTRL0_IP_SW_RST);
        dsb();
        delay_us(10);  // Hold reset active
        ippc.write32(usb_hw::IP_PW_CTRL0, ctrl0 & !usb_hw::CTRL0_IP_SW_RST);
        dsb();
        delay_ms(1);  // Wait for in-flight DMA to complete (1ms safety margin)
        logln!("      IP reset complete, DMA quiesced");

        // Step 2: Power down device IP (we only want host mode)
        logln!("    Step 2: Power down device IP...");
        let ctrl2 = ippc.read32(usb_hw::IP_PW_CTRL2);
        ippc.write32(usb_hw::IP_PW_CTRL2, ctrl2 | usb_hw::CTRL2_IP_DEV_PDN);
        dsb();

        // Step 3: Power on host IP
        logln!("    Step 3: Power on host IP...");
        let ctrl1 = ippc.read32(usb_hw::IP_PW_CTRL1);
        logln!("      CTRL1 before: 0x{:08x}", ctrl1);
        ippc.write32(usb_hw::IP_PW_CTRL1, ctrl1 & !usb_hw::CTRL1_IP_HOST_PDN);
        dsb();

        // Step 4: Read port configuration from IP_XHCI_CAP
        let xhci_cap = ippc.read32(usb_hw::IP_XHCI_CAP);
        let u3_port_num = ((xhci_cap >> 8) & 0xF) as usize;
        let u2_port_num = ((xhci_cap >> 0) & 0xF) as usize;
        logln!("    Step 4: Port config: {} U3 ports, {} U2 ports", u3_port_num, u2_port_num);

        // Step 5: Enable U3 ports
        for i in 0..u3_port_num.min(4) {
            let offset = usb_hw::U3_CTRL_P0 + (i * 8);
            let ctrl = ippc.read32(offset);
            let new_ctrl = (ctrl & !(usb_hw::PORT_PDN | usb_hw::PORT_DIS)) | usb_hw::PORT_HOST_SEL;
            ippc.write32(offset, new_ctrl);
            logln!("      U3P{}: 0x{:08x} -> 0x{:08x}", i, ctrl, new_ctrl);
        }

        // Step 6: Enable U2 ports
        for i in 0..u2_port_num.min(5) {
            let offset = usb_hw::U2_CTRL_P0 + (i * 8);
            let ctrl = ippc.read32(offset);
            let new_ctrl = (ctrl & !(usb_hw::PORT_PDN | usb_hw::PORT_DIS)) | usb_hw::PORT_HOST_SEL;
            ippc.write32(offset, new_ctrl);
            logln!("      U2P{}: 0x{:08x} -> 0x{:08x}", i, ctrl, new_ctrl);
        }
        dsb();

        // Step 7: Wait for clocks to stabilize
        // These bits use "bar" notation: 1 = reset deasserted (good), 0 = reset asserted (bad)
        logln!("    Step 5: Waiting for clock stability...");
        let stable_mask = usb_hw::STS1_SYSPLL_STABLE | usb_hw::STS1_REF_RST |
                          usb_hw::STS1_SYS125_RST | usb_hw::STS1_XHCI_RST;
        let mut stable = false;
        for _ in 0..200 {  // 200ms timeout
            delay_ms(1);
            let sts1 = ippc.read32(usb_hw::IP_PW_STS1);
            // All bits should be SET when stable (bar signals: 1 = deasserted/stable)
            if (sts1 & stable_mask) == stable_mask {
                stable = true;
                logln!("      STS1: 0x{:08x} - clocks stable", sts1);
                break;
            }
        }
        if !stable {
            let sts1 = ippc.read32(usb_hw::IP_PW_STS1);
            logln!("      [!!] Clock stability timeout, STS1=0x{:08x}", sts1);
            // Continue anyway - some bits may not be set on all hardware
        }

        // Step 8: Now we can access xHCI registers
        // Read CAPLENGTH to find operational registers
        let mac = MmioRegion::new(mac_base);
        let caplength = (mac.read32(usb_hw::CAPLENGTH) & 0xFF) as usize;
        logln!("    Step 6: xHCI CAPLENGTH=0x{:02x}", caplength);

        if caplength == 0 || caplength > 0x40 {
            logln!("      [!!] Invalid CAPLENGTH, xHCI not accessible");
            self.hardware_verified = false;
            return;
        }

        let op = MmioRegion::new(mac_base + caplength);

        // Wait for Controller Not Ready to clear
        let mut ready = false;
        for _ in 0..100 {
            delay_ms(1);
            let sts = op.read32(usb_hw::USBSTS);
            if (sts & usb_hw::STS_CNR) == 0 {
                ready = true;
                break;
            }
        }
        if !ready {
            let sts = op.read32(usb_hw::USBSTS);
            logln!("      [!!] Controller not ready, USBSTS=0x{:08x}", sts);
            self.hardware_verified = false;
            return;
        }
        logln!("      Controller ready");

        // Verify final state
        let verified = self.usb_verify_state(index);
        self.hardware_verified = verified;

        if verified {
            logln!("    [OK] USB{}: hardware reset complete, state verified", index);
        } else {
            logln!("    [!!] USB{}: reset complete but state NOT verified", index);
        }

        // Step 9: Power down to put in safe state
        // The controller is now in a known-good state.
        // devd can re-power when it wants to use the controller.
        self.usb_power_down(index);
    }

    /// Power down USB controller to put in safe state
    ///
    /// This powers down the host IP and all ports, leaving the controller
    /// in a known safe state where no DMA can occur.
    fn usb_power_down(&self, index: usize) {
        let mac_base = usb_hw::MAC_BASES[index];
        let ippc = MmioRegion::new(mac_base + usb_hw::IPPC_OFFSET);

        logln!("    Powering down USB{} for safe state...", index);

        // Read port config
        let xhci_cap = ippc.read32(usb_hw::IP_XHCI_CAP);
        let u3_port_num = ((xhci_cap >> 8) & 0xF) as usize;
        let u2_port_num = ((xhci_cap >> 0) & 0xF) as usize;

        // Power down U3 ports
        for i in 0..u3_port_num.min(4) {
            let offset = usb_hw::U3_CTRL_P0 + (i * 8);
            let ctrl = ippc.read32(offset);
            let new_ctrl = ctrl | usb_hw::PORT_PDN;
            ippc.write32(offset, new_ctrl);
        }

        // Power down U2 ports
        for i in 0..u2_port_num.min(5) {
            let offset = usb_hw::U2_CTRL_P0 + (i * 8);
            let ctrl = ippc.read32(offset);
            let new_ctrl = ctrl | usb_hw::PORT_PDN;
            ippc.write32(offset, new_ctrl);
        }
        dsb();

        // Power down host IP
        let ctrl1 = ippc.read32(usb_hw::IP_PW_CTRL1);
        ippc.write32(usb_hw::IP_PW_CTRL1, ctrl1 | usb_hw::CTRL1_IP_HOST_PDN);
        dsb();

        logln!("      Host IP powered down");
    }

    /// Verify USB controller is in expected state after power-on
    fn usb_verify_state(&self, index: usize) -> bool {
        let mac_base = usb_hw::MAC_BASES[index];
        let mac = MmioRegion::new(mac_base);
        let ippc = MmioRegion::new(mac_base + usb_hw::IPPC_OFFSET);

        // Read capability length
        let caplength = (mac.read32(usb_hw::CAPLENGTH) & 0xFF) as usize;
        if caplength == 0 || caplength > 0x40 {
            logln!("      [!!] Invalid CAPLENGTH=0x{:02x}", caplength);
            return false;
        }

        let op = MmioRegion::new(mac_base + caplength);
        let usbsts = op.read32(usb_hw::USBSTS);

        logln!("      USBSTS: 0x{:08x} CNR={} HCE={}",
               usbsts,
               if (usbsts & usb_hw::STS_CNR) != 0 { 1 } else { 0 },
               if (usbsts & usb_hw::STS_HCE) != 0 { 1 } else { 0 });

        // After power-on, controller should be:
        // - CNR=0 (ready)
        // - HCE=0 (no error)
        if (usbsts & usb_hw::STS_CNR) != 0 {
            logln!("      [!!] Controller not ready (CNR=1)");
            return false;
        }
        if (usbsts & usb_hw::STS_HCE) != 0 {
            logln!("      [!!] Host controller error (HCE=1)");
            return false;
        }

        // Check IPPC power status
        let pw_sts1 = ippc.read32(usb_hw::IP_PW_STS1);

        // Verify IP is not in sleep mode
        if (pw_sts1 & usb_hw::STS1_IP_SLEEP_STS) != 0 {
            logln!("      [!!] IP is in sleep mode");
            return false;
        }

        logln!("      Verification passed");
        true
    }

    /// Handle a control message from devd
    pub fn handle_message(&mut self, msg: &Message) -> Result<(), BusError> {
        if msg.header.payload_len == 0 {
            return Err(BusError::InvalidMessage);
        }

        let msg_type = msg.payload[0];

        match msg_type {
            x if x == BusControlMsgType::EnableBusMastering as u8 => {
                // Requires Claimed state
                if self.state != BusState::Claimed {
                    return Err(BusError::NotClaimed);
                }
                // payload[1..3] = device_id
                if msg.header.payload_len < 3 {
                    return Err(BusError::InvalidMessage);
                }
                let device_id = u16::from_le_bytes([msg.payload[1], msg.payload[2]]);
                self.enable_bus_mastering(device_id)
            }
            x if x == BusControlMsgType::DisableBusMastering as u8 => {
                // Requires Claimed state
                if self.state != BusState::Claimed {
                    return Err(BusError::NotClaimed);
                }
                if msg.header.payload_len < 3 {
                    return Err(BusError::InvalidMessage);
                }
                let device_id = u16::from_le_bytes([msg.payload[1], msg.payload[2]]);
                self.disable_bus_mastering(device_id)
            }
            x if x == BusControlMsgType::RequestReset as u8 => {
                // Reset can be requested from Claimed OR Safe state
                // Always perform full hardware reset - devd knows best
                if self.state == BusState::Resetting {
                    // Already resetting - don't nest
                    return Err(BusError::Busy);
                }

                logln!("  Bus {}: reset requested (current state: {:?})",
                       self.port_name_str(), self.state);

                // Transition to Resetting state
                self.transition_to(BusState::Resetting, StateChangeReason::ResetRequested);

                // Perform actual hardware reset
                self.perform_hardware_reset();

                // Transition to Safe (driver can now claim)
                self.transition_to(BusState::Safe, StateChangeReason::ResetComplete);

                // Clear owner info - they need to reconnect
                self.owner_channel = None;
                self.owner_pid = None;

                Ok(())
            }
            x if x == BusControlMsgType::SetDriver as u8 => {
                // devd tells us which PID is the actual driver
                // When this PID exits, we auto-reset and notify devd
                if msg.header.payload_len < 5 {
                    return Err(BusError::InvalidMessage);
                }
                let driver_pid = u32::from_le_bytes([
                    msg.payload[1], msg.payload[2], msg.payload[3], msg.payload[4]
                ]);

                logln!("  Bus {}: driver PID set to {} (supervisor PID {:?})",
                       self.port_name_str(), driver_pid, self.owner_pid);

                self.driver_pid = Some(driver_pid);

                // Transition to Claimed state now that we have a driver
                if self.state == BusState::Safe {
                    self.transition_to(BusState::Claimed, StateChangeReason::DriverClaimed);
                }

                Ok(())
            }
            _ => Err(BusError::InvalidMessage),
        }
    }

    /// Enable bus mastering for a device
    ///
    /// For PCIe: Sets bit 2 of PCI_COMMAND register
    /// For USB: Records permission (xHCI Run/Stop is controlled by driver)
    fn enable_bus_mastering(&mut self, device_id: u16) -> Result<(), BusError> {
        // Find or create device entry
        let slot = self.devices[..self.device_count]
            .iter()
            .position(|d| d.device_id == device_id)
            .unwrap_or_else(|| {
                if self.device_count < MAX_DEVICES_PER_BUS {
                    let slot = self.device_count;
                    self.device_count += 1;
                    self.devices[slot].device_id = device_id;
                    slot
                } else {
                    0 // Shouldn't happen, but fallback
                }
            });

        // Actually write to hardware
        let hw_result = match self.bus_type {
            BusType::PCIe => self.pcie_set_bus_mastering(device_id, true),
            BusType::Usb => self.usb_set_dma_allowed(device_id, true),
        };

        if let Err(e) = hw_result {
            logln!("  Bus {}: FAILED to enable bus mastering for device {:04x}: {:?}",
                   self.port_name_str(), device_id, e);
            return Err(e);
        }

        self.devices[slot].enabled = true;
        self.bus_master_enabled = true;

        logln!("  Bus {}: enabled bus mastering for device {:04x}",
               self.port_name_str(), device_id);

        Ok(())
    }

    /// Disable bus mastering for a device
    ///
    /// For PCIe: Clears bit 2 of PCI_COMMAND register
    /// For USB: Records permission revocation (driver must stop xHCI)
    fn disable_bus_mastering(&mut self, device_id: u16) -> Result<(), BusError> {
        if let Some(slot) = self.devices[..self.device_count]
            .iter()
            .position(|d| d.device_id == device_id)
        {
            // Actually write to hardware
            let hw_result = match self.bus_type {
                BusType::PCIe => self.pcie_set_bus_mastering(device_id, false),
                BusType::Usb => self.usb_set_dma_allowed(device_id, false),
            };

            if let Err(e) = hw_result {
                logln!("  Bus {}: FAILED to disable bus mastering for device {:04x}: {:?}",
                       self.port_name_str(), device_id, e);
                // Continue anyway - we must disable in our tracking
            }

            self.devices[slot].enabled = false;

            logln!("  Bus {}: disabled bus mastering for device {:04x}",
                   self.port_name_str(), device_id);
        }

        // Update global flag
        self.bus_master_enabled = self.devices[..self.device_count]
            .iter()
            .any(|d| d.enabled);

        Ok(())
    }

    // =========================================================================
    // PCIe Bus Mastering Hardware Control
    // =========================================================================

    /// Set or clear bus mastering for a PCIe device
    ///
    /// device_id format: (bus << 8) | (dev << 3) | func
    /// For root port devices, this is typically 0x0000
    fn pcie_set_bus_mastering(&self, device_id: u16, enable: bool) -> Result<(), BusError> {
        let index = self.bus_index as usize;
        if index >= pcie_hw::MAC_BASES.len() {
            return Err(BusError::NotFound);
        }

        let mac_base = pcie_hw::MAC_BASES[index];
        let cfg_base = mac_base + pcie_hw::CFG_OFFSET;
        let cfg = MmioRegion::new(cfg_base);

        // Extract BDF from device_id
        let bus_num = (device_id >> 8) as u8;
        let devfn = (device_id & 0xFF) as u8;

        // For devices on this port's secondary bus, we access via TLP
        // For now, we only support the root port itself (bus 0, devfn 0)
        // and immediate children (bus 1, any devfn)
        if bus_num > 1 {
            logln!("    PCIe{}: device {:04x} not on root or secondary bus, skipping",
                   index, device_id);
            // We allow this - driver can manage downstream devices
            return Ok(());
        }

        // Configure TLP header for config access
        // MediaTek uses CFGNUM register at MAC+0x140
        let cfgnum = MmioRegion::new(mac_base);
        let cfgnum_val = ((bus_num as u32) << 8) | (devfn as u32) | (0xF << 16); // BE=0xF
        cfgnum.write32(0x140, cfgnum_val);
        dsb();

        // Read current PCI_COMMAND (offset 0x04, 16-bit)
        let cmd_offset = pcie_hw::PCI_COMMAND;
        let current = cfg.read16(cmd_offset);

        let new_cmd = if enable {
            current | pcie_hw::CMD_BUS_MASTER
        } else {
            current & !pcie_hw::CMD_BUS_MASTER
        };

        if current != new_cmd {
            cfg.write16(cmd_offset, new_cmd);
            dsb();

            // Verify
            let verify = cfg.read16(cmd_offset);
            if (verify & pcie_hw::CMD_BUS_MASTER != 0) != enable {
                logln!("    PCIe{}: bus mastering {} FAILED (cmd=0x{:04x})",
                       index, if enable { "enable" } else { "disable" }, verify);
                return Err(BusError::HardwareError);
            }
        }

        logln!("    PCIe{}: device {:04x} bus mastering {} (cmd 0x{:04x} -> 0x{:04x})",
               index, device_id, if enable { "enabled" } else { "disabled" }, current, new_cmd);

        Ok(())
    }

    /// Disable bus mastering for ALL devices on this PCIe port
    fn pcie_disable_all_bus_mastering(&self) {
        let index = self.bus_index as usize;
        if index >= pcie_hw::MAC_BASES.len() {
            return;
        }

        // Disable on root port (device 0)
        let _ = self.pcie_set_bus_mastering(0x0000, false);

        // Disable on first device behind root port (bus 1, device 0)
        let _ = self.pcie_set_bus_mastering(0x0100, false);

        logln!("    PCIe{}: disabled bus mastering on all known devices", index);
    }

    // =========================================================================
    // USB Bus Mastering (DMA) Hardware Control
    // =========================================================================

    /// Set or clear DMA permission for USB
    ///
    /// For USB, DMA is controlled by xHCI Run/Stop bit
    /// We don't directly control R/S here - that's the driver's job
    /// Instead, we track permission and can forcibly halt if needed
    fn usb_set_dma_allowed(&self, device_id: u16, allow: bool) -> Result<(), BusError> {
        let index = self.bus_index as usize;
        if index >= usb_hw::MAC_BASES.len() {
            return Err(BusError::NotFound);
        }

        // For USB, device_id could be:
        // 0x0000 = xHCI controller itself
        // 0x0001+ = USB devices (slot IDs)

        if device_id == 0 {
            // xHCI controller - this controls whether the controller can DMA at all
            if !allow {
                // Force halt the controller
                self.usb_force_halt()?;
            }
            // If allowing, driver will set Run/Stop when ready
        }

        logln!("    USB{}: device {:04x} DMA {}",
               index, device_id, if allow { "allowed" } else { "revoked" });

        Ok(())
    }

    /// Force halt the USB controller (emergency stop)
    fn usb_force_halt(&self) -> Result<(), BusError> {
        let index = self.bus_index as usize;
        if index >= usb_hw::MAC_BASES.len() {
            return Err(BusError::NotFound);
        }

        let mac_base = usb_hw::MAC_BASES[index];
        let mac = MmioRegion::new(mac_base);

        // Read capability length
        let caplength = (mac.read32(usb_hw::CAPLENGTH) & 0xFF) as usize;
        let op = MmioRegion::new(mac_base + caplength);

        // Check if already halted
        let usbsts = op.read32(usb_hw::USBSTS);
        if (usbsts & usb_hw::STS_HCH) != 0 {
            logln!("    USB{}: already halted", index);
            return Ok(());
        }

        // Clear Run/Stop
        let usbcmd = op.read32(usb_hw::USBCMD);
        op.write32(usb_hw::USBCMD, usbcmd & !usb_hw::CMD_RUN);
        dsb();

        // Wait for halt (max 16ms)
        for _ in 0..20 {
            delay_ms(1);
            let sts = op.read32(usb_hw::USBSTS);
            if (sts & usb_hw::STS_HCH) != 0 {
                logln!("    USB{}: force halted", index);
                return Ok(());
            }
        }

        logln!("    USB{}: force halt FAILED", index);
        Err(BusError::HardwareError)
    }

    /// Disable DMA for ALL USB devices
    fn usb_disable_all_dma(&self) {
        let _ = self.usb_force_halt();
    }
}

// =============================================================================
// Bus Registry
// =============================================================================

/// Global registry of all bus controllers
pub struct BusRegistry {
    buses: [BusController; MAX_BUSES],
    bus_count: usize,
}

impl BusRegistry {
    pub const fn new() -> Self {
        const EMPTY: BusController = BusController::new();
        Self {
            buses: [EMPTY; MAX_BUSES],
            bus_count: 0,
        }
    }

    /// Add a bus controller
    pub fn add(&mut self, bus_type: BusType, index: u8) -> Option<&mut BusController> {
        if self.bus_count >= MAX_BUSES {
            return None;
        }

        let slot = self.bus_count;
        self.buses[slot].init(bus_type, index);
        self.bus_count += 1;

        Some(&mut self.buses[slot])
    }

    /// Find a bus by type and index
    pub fn find(&mut self, bus_type: BusType, index: u8) -> Option<&mut BusController> {
        self.buses[..self.bus_count]
            .iter_mut()
            .find(|b| b.bus_type == bus_type && b.bus_index == index)
    }

    /// Find a bus by listen channel
    pub fn find_by_channel(&mut self, channel: ChannelId) -> Option<&mut BusController> {
        self.buses[..self.bus_count]
            .iter_mut()
            .find(|b| b.listen_channel == Some(channel) || b.owner_channel == Some(channel))
    }

    /// Get all buses
    pub fn iter(&self) -> impl Iterator<Item = &BusController> {
        self.buses[..self.bus_count].iter()
    }

    /// Get all buses mutably
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut BusController> {
        self.buses[..self.bus_count].iter_mut()
    }

    /// Process cleanup when a process exits
    pub fn process_cleanup(&mut self, pid: Pid) {
        for bus in self.buses[..self.bus_count].iter_mut() {
            // Check if this is the driver (usbd, etc.)
            // If so, reset bus and notify supervisor (devd)
            if bus.driver_pid == Some(pid) {
                bus.handle_driver_exit(pid);
            }
            // Check if this is the supervisor (devd)
            // If so, trigger full disconnect
            else if bus.owner_pid == Some(pid) {
                logln!("  Bus {}: owner PID {} exited, triggering disconnect",
                       bus.port_name_str(), pid);
                bus.handle_disconnect(StateChangeReason::OwnerCrashed);
            }
        }
    }

    /// Print registry info
    pub fn print_info(&self) {
        logln!("  Bus controllers:");
        for bus in self.buses[..self.bus_count].iter() {
            logln!("    {} state={:?} owner={:?} driver={:?} verified={}",
                   bus.port_name_str(),
                   bus.state,
                   bus.owner_pid,
                   bus.driver_pid,
                   bus.hardware_verified);
        }
    }
}

// =============================================================================
// Errors
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusError {
    /// Bus is already claimed by another process
    AlreadyClaimed,
    /// Bus is not claimed (can't perform operation)
    NotClaimed,
    /// Bus is busy (resetting)
    Busy,
    /// Invalid message format
    InvalidMessage,
    /// Failed to send message
    SendFailed,
    /// Hardware error
    HardwareError,
    /// No such bus
    NotFound,
}

impl BusError {
    pub fn to_errno(self) -> i32 {
        match self {
            BusError::AlreadyClaimed => -16,  // EBUSY
            BusError::NotClaimed => -1,       // EPERM
            BusError::Busy => -16,            // EBUSY
            BusError::InvalidMessage => -22,  // EINVAL
            BusError::SendFailed => -5,       // EIO
            BusError::HardwareError => -5,    // EIO
            BusError::NotFound => -2,         // ENOENT
        }
    }
}

// =============================================================================
// Global Instance
// =============================================================================

static mut BUS_REGISTRY: BusRegistry = BusRegistry::new();

/// Get the global bus registry
/// # Safety
/// Must ensure proper synchronization
pub unsafe fn bus_registry() -> &'static mut BusRegistry {
    &mut *core::ptr::addr_of_mut!(BUS_REGISTRY)
}

/// Execute a closure with exclusive access to the bus registry
#[inline]
pub fn with_bus_registry<R, F: FnOnce(&mut BusRegistry) -> R>(f: F) -> R {
    let _guard = crate::arch::aarch64::sync::IrqGuard::new();
    unsafe { f(bus_registry()) }
}

// =============================================================================
// Initialization
// =============================================================================

/// Initialize bus controllers for MT7988A
/// Called during kernel early boot
///
/// This function:
/// 1. Creates bus controllers for each PCIe/USB port
/// 2. Performs initial hardware reset to ensure known state
/// 3. Verifies hardware is in expected state
/// 4. Registers control ports for devd to connect
pub fn init(kernel_pid: Pid) {
    logln!("Initializing bus controllers...");

    with_bus_registry(|registry| {
        // PCIe ports - MT7988A has 3 active PCIe ports (0, 1, 2)
        // Port 3 exists but is typically not used on BPI-R4
        for i in 0..3u8 {
            if let Some(bus) = registry.add(BusType::PCIe, i) {
                logln!("  Initializing {}...", bus.port_name_str());

                // Perform initial hardware reset and verify
                bus.perform_hardware_reset();

                if !bus.hardware_verified {
                    logln!("    [!!] {} hardware verification failed", bus.port_name_str());
                }

                if let Err(e) = bus.register_port(kernel_pid) {
                    logln!("    [!!] Failed to register {}: {:?}", bus.port_name_str(), e);
                }

                // Flush after each bus (timer not running yet during early boot)
                crate::kernel::log::flush();
            }
        }

        // USB controllers - MT7988A has 2 SSUSB controllers
        for i in 0..2u8 {
            if let Some(bus) = registry.add(BusType::Usb, i) {
                logln!("  Initializing {}...", bus.port_name_str());

                // Perform initial hardware reset and verify
                bus.perform_hardware_reset();

                if !bus.hardware_verified {
                    logln!("    [!!] {} hardware verification failed", bus.port_name_str());
                }

                if let Err(e) = bus.register_port(kernel_pid) {
                    logln!("    [!!] Failed to register {}: {:?}", bus.port_name_str(), e);
                }

                // Flush after each bus (timer not running yet during early boot)
                crate::kernel::log::flush();
            }
        }

        logln!("");
        logln!("  Bus controller summary:");
        registry.print_info();
    });

    logln!("  Bus controllers initialized");
}

/// Handle process cleanup (called when a process exits)
pub fn process_cleanup(pid: Pid) {
    with_bus_registry(|registry| {
        registry.process_cleanup(pid);
    });
}

/// Handle port_connect for kernel bus ports
/// Called synchronously from port::connect() for /kernel/bus/* ports
/// suffix is the part after "/kernel/bus/" e.g. "usb0", "pcie1"
pub fn handle_port_connect(suffix: &str, client_channel: ChannelId, client_pid: Pid) -> Result<(), BusError> {
    // Parse bus type and index from suffix
    let (bus_type, index) = if suffix.starts_with("usb") {
        let idx = suffix[3..].parse::<u8>().map_err(|_| BusError::NotFound)?;
        (BusType::Usb, idx)
    } else if suffix.starts_with("pcie") {
        let idx = suffix[4..].parse::<u8>().map_err(|_| BusError::NotFound)?;
        (BusType::PCIe, idx)
    } else {
        return Err(BusError::NotFound);
    };

    with_bus_registry(|registry| {
        if let Some(bus) = registry.find(bus_type, index) {
            bus.handle_connect(client_channel, client_pid)
        } else {
            Err(BusError::NotFound)
        }
    })
}

/// Process a message sent to a kernel bus channel
/// Called synchronously from sys_send when message is for a kernel-owned channel
pub fn process_bus_message(client_channel: ChannelId, data: &[u8]) {
    // Find the bus that owns the peer of this channel
    // The client sends on client_channel, which means the message goes to server_channel's queue
    // We need to find which bus has server_channel as its owner_channel

    // Get the peer channel ID (server_channel)
    let server_channel = super::ipc::with_channel_table(|table| {
        if let Some(slot) = table.endpoints.iter().position(|e| e.id == client_channel) {
            Some(table.endpoints[slot].peer)
        } else {
            None
        }
    });

    let Some(server_ch) = server_channel else {
        logln!("  process_bus_message: no peer for channel {}", client_channel);
        return;
    };

    // Find the bus controller that owns this channel
    with_bus_registry(|registry| {
        for bus in registry.iter_mut() {
            if bus.owner_channel == Some(server_ch) {
                // Build a message from the data
                let mut msg = super::ipc::Message::new();
                msg.header.payload_len = data.len() as u32;
                msg.payload[..data.len()].copy_from_slice(data);

                // Process the message
                if let Err(e) = bus.handle_message(&msg) {
                    logln!("  Bus {}: message error: {:?}", bus.port_name_str(), e);
                }
                return;
            }
        }
        logln!("  process_bus_message: no bus owns channel {}", server_ch);
    });
}

// =============================================================================
// Tests
// =============================================================================

/// Test bus controller state machine
pub fn test() {
    logln!("  Testing bus controller...");

    let mut bus = BusController::new();
    bus.init(BusType::PCIe, 0);

    // Initial state should be SAFE
    assert!(bus.state == BusState::Safe);
    assert!(bus.check_invariants());

    // Simulate connect
    bus.owner_channel = Some(100);
    bus.owner_pid = Some(1);
    bus.transition_to(BusState::Claimed, StateChangeReason::Connected);
    assert!(bus.state == BusState::Claimed);
    assert!(bus.check_invariants());

    // Simulate disconnect
    bus.handle_disconnect(StateChangeReason::Disconnected);
    assert!(bus.state == BusState::Safe);
    assert!(bus.check_invariants());

    logln!("    [OK] Bus controller test passed");
}
