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
use crate::kernel::lock::{SpinLock, lock_class};

pub use types::{BusType, BusInfo, DeviceInfo, bus_caps};
pub use protocol::StateChangeReason;
pub use controller::BusController;

use super::ipc::{ChannelId, Message};
use super::process::Pid;
use crate::{kinfo, kdebug, kerror, print_direct};
use core::sync::atomic::{AtomicBool, Ordering};

/// Maximum number of buses (MT7988A needs 10: 4xPCIe + 2xUSB + eth + platform + uart + klog)
pub const MAX_BUSES: usize = 12;

/// Once set, no more buses can be created via open(Bus)
static BUS_CREATION_LOCKED: AtomicBool = AtomicBool::new(false);

/// Lock bus creation permanently (called after probed exits)
pub fn lock_bus_creation() {
    BUS_CREATION_LOCKED.store(true, Ordering::SeqCst);
    kdebug!("bus", "creation_locked");
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

    /// Find a bus by listen channel, supervisor channel, or owner channel
    pub fn find_by_channel(&mut self, channel: ChannelId) -> Option<&mut BusController> {
        self.buses[..self.bus_count]
            .iter_mut()
            .find(|b| b.listen_channel == Some(channel) || b.supervisor_ch == Some(channel) || b.owner_ch == Some(channel))
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
            // Check if this is the owner (driver process)
            // If so, reset bus hardware and notify supervisor
            if bus.is_owner(pid) {
                bus.handle_owner_exit(pid);
            }
            // Check if this is the supervisor (devd)
            // Independent check — not else-if
            if bus.is_supervisor(pid) {
                kdebug!("bus", "supervisor_exit"; name = bus.port_name_str(), pid = pid as u64);
                bus.handle_supervisor_disconnect(StateChangeReason::OwnerCrashed);
            }
        }
    }
}

// =============================================================================
// Global Instance
// =============================================================================

static BUS_REGISTRY: SpinLock<BusRegistry> = SpinLock::new(lock_class::BUS, BusRegistry::new());

/// Execute a closure with exclusive access to the bus registry
#[inline]
pub fn with_bus_registry<R, F: FnOnce(&mut BusRegistry) -> R>(f: F) -> R {
    let mut guard = BUS_REGISTRY.lock();
    f(&mut guard)
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

        kdebug!("bus", "created"; name = bus.port_name_str(),
            state = initial_state.as_str(),
            base = crate::klog::hex64(info.base_addr));

        // Register the PCIe host controller
        if bus_type == BusType::PCIe && info.base_addr != 0 {
            if ecam_based {
                crate::kernel::pci::register_ecam_host(info.base_addr as usize);
            } else {
                crate::kernel::pci::register_mt7988a_port(info.bus_index, info.base_addr as usize);
            }
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
        // Find first bus still in Resetting state that hasn't been processed
        for bus in registry.iter_mut() {
            if bus.state() == BusState::Resetting && !bus.init_complete {
                // Do hardware reset for this bus
                bus.perform_hardware_reset();
                bus.init_complete = true;

                if !bus.hardware_verified {
                    // Hardware failed or no link — stay in Resetting state.
                    // devd can't claim a bus in Resetting, so no driver spawned.
                    kdebug!("bus", "hw_init_skipped"; name = bus.port_name_str());
                    return true;
                }

                // Transition to Safe (notifies connected devd if any)
                bus.transition_to(BusState::Safe, StateChangeReason::ResetComplete);

                // If this is a PCIe bus, enumerate devices now that hardware is ready
                if bus.bus_type == BusType::PCIe {
                    let bus_index = bus.bus_index;
                    let ecam = bus.ecam_based;
                    let count = super::pci::enumerate();
                    // Copy only THIS bus's devices into BusController for generic delivery.
                    // For ECAM (single host): all devices belong to bus 0.
                    // For MT7988A MAC-TLP: port is encoded in upper nibble of BDF bus number.
                    super::pci::with_devices(|dev_registry| {
                        for i in 0..super::pci::device_count() {
                            if let Some(dev) = dev_registry.get(i) {
                                let mine = if ecam {
                                    bus_index == 0 // ECAM: all on bus 0
                                } else {
                                    (dev.bdf.bus >> 4) == bus_index
                                };
                                if mine {
                                    bus.add_enum_device(pci_to_bus_device(dev));
                                }
                            }
                        }
                    });
                    kinfo!("bus", "pci_enumerated"; devices = count as u64, bus_idx = bus_index as u64);
                }

                // Return true = more work might remain
                return true;
            }
        }
        // All buses processed
        false
    })
}

/// Check if all buses have completed initialization
pub fn init_complete() -> bool {
    with_bus_registry(|registry| {
        registry.iter().all(|bus| bus.state() != BusState::Resetting || bus.init_complete)
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
    kdebug!("bus", "reset_all_start");
    with_bus_registry(|registry| {
        for bus in registry.iter_mut() {
            // Clear all connections
            bus.supervisor_ch = None;
            bus.supervisor_pid = None;
            bus.owner_ch = None;
            bus.owner_pid = None;

            // Perform hardware reset sequence
            bus.perform_hardware_reset();

            // Transition to Safe state using proper transition method
            bus.transition_to(BusState::Safe, StateChangeReason::ResetComplete);

            kdebug!("bus", "reset_ok"; name = bus.port_name_str());
        }
    });
    kdebug!("bus", "reset_all_complete");
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
    // client_channel is the user's end. owner_ch stores server_channel
    // (set in handle_connect via connect_to_bus_port's server_channel arg).
    // Look up peer to find server_channel.
    let server_channel = super::ipc::get_peer_id(client_channel);

    let Some(server_ch) = server_channel else {
        return;
    };

    with_bus_registry(|registry| {
        for bus in registry.iter_mut() {
            if bus.owner_ch == Some(server_ch) {
                let mut msg = Message::new();
                msg.header.payload_len = data.len() as u32;
                msg.payload[..data.len()].copy_from_slice(data);

                if let Err(_e) = bus.handle_message(&msg) {
                    kerror!("bus", "message_error"; name = bus.port_name_str());
                }
                return;
            }
        }
    });
}

/// Get list of all buses for bus_list syscall
/// Returns number of buses written to buffer.
/// Excludes buses that failed hardware verification (stuck in Resetting after init).
pub fn get_bus_list(buf: &mut [abi::PortInfo]) -> usize {
    with_bus_registry(|registry| {
        let mut written = 0;
        for bus in registry.iter() {
            if written >= buf.len() {
                break;
            }
            // Skip buses that completed init but failed hardware verification.
            // They are stuck in Resetting and can never be claimed.
            if bus.state() == BusState::Resetting && bus.init_complete {
                continue;
            }
            buf[written] = bus.to_port_info();
            written += 1;
        }
        written
    })
}

/// Get total number of buses (for sizing buffer)
/// Matches get_bus_list() filtering — excludes failed buses.
pub fn get_bus_count() -> usize {
    with_bus_registry(|registry| {
        registry.iter()
            .filter(|b| !(b.state() == BusState::Resetting && b.init_complete))
            .count()
    })
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
// Device Registration (from probed via write(Bus))
// =============================================================================

/// Register a device on a bus (called from write(Bus) syscall)
/// probed uses this to seed root hardware devices on platform buses.
pub fn register_device(bus_type_id: u8, bus_index: u8, dev: abi::BusDevice) -> Result<(), BusError> {
    let bus_type = match bus_type_id {
        abi::bus_type::PCIE => BusType::PCIe,
        abi::bus_type::USB => BusType::Usb,
        abi::bus_type::PLATFORM => BusType::Platform,
        abi::bus_type::ETHERNET => BusType::Ethernet,
        abi::bus_type::UART => BusType::Uart,
        abi::bus_type::KLOG => BusType::Klog,
        _ => return Err(BusError::InvalidMessage),
    };

    with_bus_registry(|registry| {
        let bus = registry.find(bus_type, bus_index)
            .ok_or(BusError::NotFound)?;
        if bus.add_enum_device(dev) {
            Ok(())
        } else {
            Err(BusError::AlreadyClaimed) // Repurpose: device list full
        }
    })
}

// =============================================================================
// PCI → BusDevice conversion
// =============================================================================

/// Convert a kernel PciDevice to a generic BusDevice for bus protocol delivery
fn pci_to_bus_device(dev: &super::pci::PciDevice) -> abi::BusDevice {
    use abi::bus_device_flags;

    let mut flags: u16 = bus_device_flags::DMA | bus_device_flags::MMIO;
    if dev.msi_cap != 0 {
        flags |= bus_device_flags::MSI;
    }
    if dev.msix_cap != 0 {
        flags |= bus_device_flags::MSIX;
    }
    if dev.is_bridge {
        flags |= bus_device_flags::BRIDGE;
    }

    // Pack class_code as class(8)|subclass(8)|prog_if(8)|revision(8)
    // PciDevice.class_code is 24-bit: class(8)|subclass(8)|prog_if(8) — shift left 8 to add revision
    let class_code = (dev.class_code << 8) | (dev.revision as u32);

    abi::BusDevice {
        id: dev.bdf.to_u32(),
        vendor_id: dev.vendor_id,
        device_id: dev.device_id,
        class_code,
        resource0: dev.bar0_addr,
        resource1: dev.bar0_size as u32,
        flags,
        _reserved: 0,
    }
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

    // Simulate owner connect
    bus.supervisor_ch = Some(99);
    bus.supervisor_pid = Some(1);
    bus.owner_ch = Some(100);
    bus.owner_pid = Some(2);
    bus.transition_to(BusState::Claimed, StateChangeReason::Connected);
    assert!(bus.state() == BusState::Claimed);
    assert!(bus.check_invariants());

    // Simulate owner exit
    bus.handle_owner_exit(2);
    assert!(bus.state() == BusState::Safe);
    assert!(bus.check_invariants());

    print_direct!("    [OK] Bus controller test passed\n");
}
