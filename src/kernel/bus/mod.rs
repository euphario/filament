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

mod types;
mod protocol;
mod controller;
mod hw_pcie;
mod hw_usb;

// Re-export for use by hw modules
use super::hw_poll;

pub use types::{BusType, BusInfo, DeviceInfo, bus_caps};
pub use protocol::StateChangeReason;
pub use controller::BusController;

use super::ipc::{ChannelId, Message};
use super::process::Pid;
use crate::{kinfo, kerror, print_direct};
use core::sync::atomic::{AtomicBool, Ordering};

/// Maximum number of buses
pub const MAX_BUSES: usize = 8;

/// Once set, no more buses can be created via open(Bus)
static BUS_CREATION_LOCKED: AtomicBool = AtomicBool::new(false);

/// Lock bus creation permanently (called after probed exits)
pub fn lock_bus_creation() {
    BUS_CREATION_LOCKED.store(true, Ordering::SeqCst);
    kinfo!("bus", "creation_locked");
}

/// Check if bus creation is locked
pub fn is_bus_creation_locked() -> bool {
    BUS_CREATION_LOCKED.load(Ordering::SeqCst)
}

// =============================================================================
// Bus State Machine
// =============================================================================

/// Bus controller states
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

    /// Valid state transitions
    pub fn can_transition_to(&self, new: BusState) -> bool {
        match (*self, new) {
            // Safe -> Claimed (devd sets driver)
            (BusState::Safe, BusState::Claimed) => true,
            // Safe -> Resetting (explicit reset request)
            (BusState::Safe, BusState::Resetting) => true,
            // Claimed -> Resetting (driver crashed or reset requested)
            (BusState::Claimed, BusState::Resetting) => true,
            // Resetting -> Safe (reset complete)
            (BusState::Resetting, BusState::Safe) => true,
            _ => false,
        }
    }

    /// Get human-readable state name
    pub fn name(&self) -> &'static str {
        self.as_str()
    }
}

impl super::ipc::traits::StateMachine for BusState {
    fn can_transition_to(&self, new: &Self) -> bool {
        BusState::can_transition_to(self, *new)
    }

    fn name(&self) -> &'static str {
        BusState::name(self)
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
                kinfo!("bus", "owner_exit"; name = bus.port_name_str(), pid = pid as u64);
                bus.handle_disconnect(StateChangeReason::OwnerCrashed);
            }
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
///
/// NOTE: Prefer `with_bus_registry()` for safe access with automatic IRQ guard.
pub(crate) unsafe fn bus_registry() -> &'static mut BusRegistry {
    &mut *core::ptr::addr_of_mut!(BUS_REGISTRY)
}

/// Execute a closure with exclusive access to the bus registry
#[inline]
pub fn with_bus_registry<R, F: FnOnce(&mut BusRegistry) -> R>(f: F) -> R {
    let _guard = crate::kernel::arch::sync::IrqGuard::new();
    unsafe { f(bus_registry()) }
}

// =============================================================================
// Bus Creation (from userspace via probed)
// =============================================================================

/// Create a bus from userspace (called by probed via open(Bus))
/// Returns Ok(()) on success
pub fn create_bus(info: &abi::BusCreateInfo, kernel_pid: Pid) -> Result<(), BusError> {
    if is_bus_creation_locked() {
        return Err(BusError::NotClaimed); // Repurpose: creation locked
    }

    let bus_type = match info.bus_type {
        abi::bus_type::PCIE => BusType::PCIe,
        abi::bus_type::USB => BusType::Usb,
        abi::bus_type::PLATFORM => BusType::Platform,
        abi::bus_type::ETHERNET => BusType::Ethernet,
        abi::bus_type::UART => BusType::Uart,
        abi::bus_type::KLOG => BusType::Klog,
        _ => return Err(BusError::InvalidMessage),
    };

    let ecam_based = (info.flags & abi::bus_create_flags::ECAM) != 0;

    // Determine initial state: PCIe/USB need hardware reset, others start Safe
    let initial_state = match bus_type {
        BusType::PCIe | BusType::Usb => BusState::Resetting,
        BusType::Platform | BusType::Ethernet | BusType::Uart | BusType::Klog => BusState::Safe,
    };

    with_bus_registry(|registry| {
        let bus = registry.add(bus_type, info.bus_index)
            .ok_or(BusError::AlreadyClaimed)?;

        bus.init_from_create_info(info.base_addr, info.size, info.irq, ecam_based);
        bus.set_initial_state(initial_state);
        let _ = bus.register_port(kernel_pid);

        kinfo!("bus", "created"; name = bus.port_name_str(),
            state = initial_state.as_str(),
            base = crate::klog::hex64(info.base_addr));

        // For ECAM PCIe: register the host controller
        if bus_type == BusType::PCIe && ecam_based && info.base_addr != 0 {
            crate::kernel::pci::register_ecam_host(info.base_addr as usize);
        }

        Ok(())
    })
}

// =============================================================================
// Initialization
// =============================================================================

/// Get current uptime in milliseconds
fn uptime_ms() -> u64 {
    crate::platform::current::timer::now_ns() / 1_000_000
}

/// Initialize bus registry
/// Called during kernel early boot to set up the empty registry.
/// Buses are created later by probed (userspace) via open(Bus) syscall.
pub fn init(_kernel_pid: Pid) {
    // Registry is statically initialized. Nothing to do here.
    // Buses will be created by probed via create_bus().
}

/// Continue bus initialization work (called from timer interrupt)
/// Processes one bus per call to avoid blocking too long.
/// Returns true if there's more work to do.
pub fn continue_init() -> bool {
    with_bus_registry(|registry| {
        // Find first bus still in Resetting state
        for bus in registry.iter_mut() {
            if bus.state() == BusState::Resetting {
                // Do hardware reset for this bus
                bus.perform_hardware_reset();
                if !bus.hardware_verified {
                    kerror!("bus", "hw_verify_failed"; name = bus.port_name_str());
                }
                // Transition to Safe (notifies connected devd if any)
                bus.transition_to(BusState::Safe, StateChangeReason::ResetComplete);

                // If this is a PCIe bus, enumerate devices now that hardware is ready
                if bus.bus_type == BusType::PCIe {
                    let count = super::pci::enumerate();
                    kinfo!("bus", "pci_enumerated"; devices = count as u64);
                }

                // Return true = more work might remain
                return true;
            }
        }
        // All buses are Safe
        false
    })
}

/// Check if all buses have completed initialization
pub fn init_complete() -> bool {
    with_bus_registry(|registry| {
        registry.iter().all(|bus| bus.state() != BusState::Resetting)
    })
}

/// Handle process cleanup (called when a process exits)
pub fn process_cleanup(pid: Pid) {
    with_bus_registry(|registry| {
        registry.process_cleanup(pid);
    });
}

/// Reset all buses to Safe state (for devd restart)
/// This is called when devd crashes and needs to be restarted.
/// All hardware is reset and put in safe state so devd can claim buses again.
pub fn reset_all_buses() {
    kinfo!("bus", "reset_all_start");
    with_bus_registry(|registry| {
        for bus in registry.iter_mut() {
            // Clear ownership
            bus.owner_channel = None;
            bus.owner_pid = None;
            bus.driver_pid = None;

            // Perform hardware reset sequence
            bus.perform_hardware_reset();

            // Transition to Safe state using proper transition method
            bus.transition_to(BusState::Safe, StateChangeReason::ResetComplete);

            kinfo!("bus", "reset_ok"; name = bus.port_name_str());
        }
    });
    kinfo!("bus", "reset_all_complete");
}

/// Handle port_connect for kernel bus ports
/// Called synchronously from port::connect() for /kernel/bus/* ports
/// suffix is the part after "/kernel/bus/" e.g. "usb0", "pcie1", "platform0"
pub fn handle_port_connect(suffix: &str, client_channel: ChannelId, client_pid: Pid) -> Result<(), BusError> {
    // Parse bus type and index from suffix
    let (bus_type, index) = if suffix.starts_with("usb") {
        let idx = suffix[3..].parse::<u8>().map_err(|_| BusError::NotFound)?;
        (BusType::Usb, idx)
    } else if suffix.starts_with("pcie") {
        let idx = suffix[4..].parse::<u8>().map_err(|_| BusError::NotFound)?;
        (BusType::PCIe, idx)
    } else if suffix.starts_with("platform") {
        let idx = suffix[8..].parse::<u8>().map_err(|_| BusError::NotFound)?;
        (BusType::Platform, idx)
    } else if suffix.starts_with("eth") {
        let idx = suffix[3..].parse::<u8>().map_err(|_| BusError::NotFound)?;
        (BusType::Ethernet, idx)
    } else if suffix.starts_with("uart") {
        let idx = suffix[4..].parse::<u8>().map_err(|_| BusError::NotFound)?;
        (BusType::Uart, idx)
    } else if suffix.starts_with("klog") {
        let idx = suffix[4..].parse::<u8>().map_err(|_| BusError::NotFound)?;
        (BusType::Klog, idx)
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
    let server_channel = super::ipc::get_peer_id(client_channel);

    let Some(server_ch) = server_channel else {
        return;
    };

    // Find the bus controller that owns this channel
    with_bus_registry(|registry| {
        for bus in registry.iter_mut() {
            if bus.owner_channel == Some(server_ch) {
                // Build a message from the data
                let mut msg = Message::new();
                msg.header.payload_len = data.len() as u32;
                msg.payload[..data.len()].copy_from_slice(data);

                // Process the message
                if let Err(e) = bus.handle_message(&msg) {
                    kerror!("bus", "message_error"; name = bus.port_name_str());
                    let _ = e;  // error logged
                }
                return;
            }
        }
    });
}

/// Get list of all buses for bus_list syscall
/// Returns number of buses written to buffer
pub fn get_bus_list(buf: &mut [BusInfo]) -> usize {
    with_bus_registry(|registry| {
        let count = registry.bus_count.min(buf.len());
        for i in 0..count {
            buf[i] = registry.buses[i].to_info();
        }
        count
    })
}

/// Get total number of buses (for sizing buffer)
pub fn get_bus_count() -> usize {
    with_bus_registry(|registry| registry.bus_count)
}

// =============================================================================
// Unified Device List
// =============================================================================

/// Platform device definitions
/// Now empty — all buses and devices are registered by probed.
/// Bus controllers report their own DeviceInfo via to_device_info().
static PLATFORM_DEVICES: &[DeviceInfo] = &[];

/// Get unified list of all devices (platform + bus controllers)
/// Returns number of devices written to buffer
pub fn get_device_list(buf: &mut [DeviceInfo]) -> usize {
    let mut count = 0;

    // First: platform devices
    for dev in PLATFORM_DEVICES {
        if count >= buf.len() {
            break;
        }
        buf[count] = *dev;
        count += 1;
    }

    // Second: bus controllers (which can enumerate their own children)
    with_bus_registry(|registry| {
        for bus in registry.iter() {
            if count >= buf.len() {
                break;
            }
            // Skip platform pseudo-bus (its devices are already listed above)
            if bus.bus_type == BusType::Platform {
                continue;
            }
            buf[count] = bus.to_device_info();
            count += 1;
        }
    });

    count
}

/// Get total count of devices for buffer sizing
pub fn get_device_count() -> usize {
    let platform_count = PLATFORM_DEVICES.len();
    let bus_count = with_bus_registry(|registry| {
        registry.iter().filter(|b| b.bus_type != BusType::Platform).count()
    });
    platform_count + bus_count
}

// =============================================================================
// Tests
// =============================================================================

/// Test bus controller state machine
pub fn test() {
    print_direct!("  Testing bus controller...\n");

    let mut bus = BusController::new();
    bus.init(BusType::PCIe, 0);

    // Initial state should be SAFE
    assert!(bus.state() == BusState::Safe);
    assert!(bus.check_invariants());

    // Simulate connect
    bus.owner_channel = Some(100);
    bus.owner_pid = Some(1);
    bus.transition_to(BusState::Claimed, StateChangeReason::Connected);
    assert!(bus.state() == BusState::Claimed);
    assert!(bus.check_invariants());

    // Simulate disconnect
    bus.handle_disconnect(StateChangeReason::Disconnected);
    assert!(bus.state() == BusState::Safe);
    assert!(bus.check_invariants());

    print_direct!("    [OK] Bus controller test passed\n");
}
