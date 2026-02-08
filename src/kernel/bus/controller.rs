//! Bus Controller
//!
//! Per-bus state machine that manages ownership, hardware state,
//! and bus mastering for PCIe, USB, and platform buses.

use super::{
    BusType, BusState, BusError, BusInfo, DeviceInfo,
    hw_pcie, hw_usb,
    types::{bus_caps, device_flags},
    protocol::{BusControlMsgType, StateSnapshot, StateChangeReason},
};
use crate::kernel::ipc::{self, ChannelId, Message, MessageType, waker, WakeReason, IpcError};
use crate::kernel::process::Pid;
use crate::{kinfo, kerror};

/// Maximum devices per bus (for tracking bus mastering)
pub const MAX_DEVICES_PER_BUS: usize = 32;

/// Bus control port name prefix
pub const BUS_PORT_PREFIX: &str = "/kernel/bus/";

/// Get current uptime in milliseconds
fn uptime_ms() -> u64 {
    crate::platform::current::timer::now_ns() / 1_000_000
}

/// Bus mastering state for a device
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum BusMasteringState {
    /// Device doesn't have bus mastering enabled
    #[default]
    Disabled,
    /// Device has bus mastering enabled
    Enabled,
    /// Device is suspended (driver crashed, waiting for reset)
    Suspended,
}

impl BusMasteringState {
    /// Check if bus mastering is active
    pub fn is_enabled(&self) -> bool {
        matches!(self, BusMasteringState::Enabled)
    }

    /// Valid state transitions
    pub fn can_transition_to(&self, new: BusMasteringState) -> bool {
        match (*self, new) {
            // Can always disable
            (_, BusMasteringState::Disabled) => true,
            // Can enable from Disabled
            (BusMasteringState::Disabled, BusMasteringState::Enabled) => true,
            // Can suspend from any state (for crash handling)
            (_, BusMasteringState::Suspended) => true,
            // Can resume from Suspended to Enabled
            (BusMasteringState::Suspended, BusMasteringState::Enabled) => true,
            _ => false,
        }
    }

    /// Get human-readable state name
    pub fn name(&self) -> &'static str {
        match self {
            BusMasteringState::Disabled => "disabled",
            BusMasteringState::Enabled => "enabled",
            BusMasteringState::Suspended => "suspended",
        }
    }
}

impl crate::kernel::ipc::traits::StateMachine for BusMasteringState {
    fn can_transition_to(&self, new: &Self) -> bool {
        BusMasteringState::can_transition_to(self, *new)
    }

    fn name(&self) -> &'static str {
        BusMasteringState::name(self)
    }
}

/// Per-device bus mastering state
#[derive(Clone, Copy, Default)]
pub struct DeviceBusMasterState {
    /// Device identifier (BDF for PCIe, slot for USB)
    pub device_id: u16,
    /// Bus mastering state - private, use state() and transition methods
    state: BusMasteringState,
    /// IOMMU mapping count
    pub iommu_mappings: u16,
}

impl DeviceBusMasterState {
    /// Get current state
    pub fn state(&self) -> BusMasteringState { self.state }
}

impl DeviceBusMasterState {
    /// Reset device state completely (for cleanup)
    pub fn reset(&mut self) {
        // Direct assignment acceptable here - reset() is the canonical way to return to initial state
        self.state = BusMasteringState::Disabled;
        self.iommu_mappings = 0;
    }

    /// Check if bus mastering is enabled
    pub fn is_enabled(&self) -> bool {
        self.state.is_enabled()
    }

    /// Enable bus mastering (Disabled -> Enabled)
    /// Returns true if transition succeeded
    pub fn enable(&mut self) -> bool {
        if self.state.can_transition_to(BusMasteringState::Enabled) {
            self.state = BusMasteringState::Enabled;
            true
        } else {
            false
        }
    }

    /// Disable bus mastering (any -> Disabled)
    pub fn disable(&mut self) {
        self.state = BusMasteringState::Disabled;
    }

    /// Suspend bus mastering (any -> Suspended)
    pub fn suspend(&mut self) {
        self.state = BusMasteringState::Suspended;
    }
}

/// A single bus controller
pub struct BusController {
    /// Bus type
    pub bus_type: BusType,

    /// Bus index (e.g., pcie0, pcie1, usb0, usb1)
    pub bus_index: u8,

    /// Current state - private, use state() and transition methods
    state: BusState,

    /// When we entered current state (for watchdog)
    pub state_entered_at: u64,

    /// Supervisor (devd) — persists across owners
    pub supervisor_ch: Option<ChannelId>,
    pub supervisor_pid: Option<Pid>,

    /// Owner — whoever claimed this port (consoled, pcied, etc.)
    /// Set when owner process connects to the bus port.
    /// When this PID exits, kernel auto-resets bus and notifies supervisor.
    pub owner_ch: Option<ChannelId>,
    pub owner_pid: Option<Pid>,

    /// Control port name (e.g., "/kernel/bus/pcie0")
    pub port_name: [u8; 32],
    pub port_name_len: usize,

    /// Listen channel for the control port
    pub listen_channel: Option<ChannelId>,

    /// Per-device bus mastering state
    pub bm_devices: [DeviceBusMasterState; MAX_DEVICES_PER_BUS],
    pub bm_count: usize,

    /// Enumerated devices on this bus (generic, delivered to drivers)
    pub enum_devices: [abi::BusDevice; MAX_DEVICES_PER_BUS],
    pub enum_count: usize,

    /// Global bus mastering enabled (hardware level)
    pub bus_master_enabled: bool,

    /// Hardware verified after reset
    pub hardware_verified: bool,

    /// MMIO base address (from probed)
    pub base_addr: u64,
    /// MMIO region size
    pub size: u64,
    /// Primary IRQ number
    pub irq: u32,
    /// ECAM-based PCIe (vs MAC-based)
    pub ecam_based: bool,
}

impl BusController {
    /// Get current state
    pub fn state(&self) -> BusState { self.state }

    pub const fn new() -> Self {
        Self {
            bus_type: BusType::PCIe,
            bus_index: 0,
            state: BusState::Safe,
            state_entered_at: 0,
            supervisor_ch: None,
            supervisor_pid: None,
            owner_ch: None,
            owner_pid: None,
            port_name: [0; 32],
            port_name_len: 0,
            listen_channel: None,
            bm_devices: [DeviceBusMasterState {
                device_id: 0,
                state: BusMasteringState::Disabled,
                iommu_mappings: 0,
            }; MAX_DEVICES_PER_BUS],
            bm_count: 0,
            enum_devices: [abi::BusDevice::empty(); MAX_DEVICES_PER_BUS],
            enum_count: 0,
            bus_master_enabled: false,
            hardware_verified: false,
            base_addr: 0,
            size: 0,
            irq: 0,
            ecam_based: false,
        }
    }

    /// Initialize a bus controller
    pub fn init(&mut self, bus_type: BusType, index: u8) {
        self.bus_type = bus_type;
        self.bus_index = index;
        // Use set_initial_state for constructor-like initialization
        self.set_initial_state(BusState::Safe);

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

    /// Initialize hardware fields from BusCreateInfo
    pub fn init_from_create_info(&mut self, base_addr: u64, size: u64, irq: u32, ecam_based: bool) {
        self.base_addr = base_addr;
        self.size = size;
        self.irq = irq;
        self.ecam_based = ecam_based;
    }

    /// Get port name as string
    pub fn port_name_str(&self) -> &str {
        core::str::from_utf8(&self.port_name[..self.port_name_len]).unwrap_or("???")
    }

    /// Set initial state during construction (before bus is active)
    /// This is the only place where direct state assignment is acceptable
    /// because the bus is brand new and has no previous state.
    pub fn set_initial_state(&mut self, state: BusState) {
        self.state = state;
        self.state_entered_at = uptime_ms();
    }

    /// Convert to BusInfo for syscall
    pub fn to_info(&self) -> BusInfo {
        let mut info = BusInfo {
            bus_type: self.bus_type as u8,
            bus_index: self.bus_index,
            state: self.state as u8,
            _pad: 0,
            base_addr: self.base_addr as u32,
            owner_pid: self.owner_pid.unwrap_or(0),
            path: [0; 32],
            path_len: self.port_name_len as u8,
            _reserved: [0; 3],
        };

        // Copy port name
        let len = self.port_name_len.min(32);
        info.path[..len].copy_from_slice(&self.port_name[..len]);

        info
    }

    /// Check if a PID is the supervisor
    pub fn is_supervisor(&self, pid: Pid) -> bool {
        self.supervisor_pid == Some(pid)
    }

    /// Check if a PID is the owner
    pub fn is_owner(&self, pid: Pid) -> bool {
        self.owner_pid == Some(pid)
    }

    /// Convert to DeviceInfo for unified device list
    /// Uses chipset:instance naming (e.g., "mt7988-pcie:0", "xhci:0")
    pub fn to_device_info(&self) -> DeviceInfo {
        // Build device name: "mt7988-pcie:0", "xhci:0", etc.
        let mut name = [0u8; 16];
        let mut pos = 0;

        let chipset_type: &[u8] = match self.bus_type {
            BusType::PCIe => b"mt7988-pcie:",
            BusType::Usb => b"xhci:",
            BusType::Platform => b"platform:",
            BusType::Ethernet => b"gmac:",
            BusType::Uart => b"uart:",
            BusType::Klog => b"klog:",
        };

        for &b in chipset_type {
            if pos < 15 {
                name[pos] = b;
                pos += 1;
            }
        }
        name[pos] = b'0' + self.bus_index;
        pos += 1;

        DeviceInfo {
            name,
            name_len: pos as u8,
            flags: device_flags::ENUMERABLE | device_flags::MMIO,
            irq: self.irq as u16,
            base_addr: self.base_addr,
            size: self.size as u32,
            _reserved: [0; 4],
        }
    }

    /// Query hardware capabilities for this bus
    /// Returns a bitmask of bus_caps flags appropriate for the bus type
    pub fn query_capabilities(&self) -> u8 {
        match self.bus_type {
            BusType::PCIe => hw_pcie::query_capabilities(self.bus_index, self.ecam_based, self.base_addr),
            BusType::Usb => hw_usb::query_capabilities(self.bus_index, self.base_addr),
            BusType::Platform | BusType::Ethernet => bus_caps::PLATFORM_MMIO | bus_caps::PLATFORM_IRQ,
            BusType::Uart | BusType::Klog => 0, // No hardware capabilities to report
        }
    }

    /// Register the bus control port with structured PortInfo
    /// Called during kernel init
    pub fn register_port(&mut self, kernel_pid: Pid) -> Result<(), IpcError> {
        let name = self.port_name_str();
        kinfo!("bus", "register_port"; name = name);

        // Build PortInfo for this bus
        let mut info = ipc::PortInfo::new(name.as_bytes(), self.port_class());
        info.port_subclass = self.port_subclass();
        info.caps = self.port_capabilities();

        let (_port_id, listen_ch) = ipc::port_register_with_info(name.as_bytes(), kernel_pid, info)?;

        self.listen_channel = Some(listen_ch);
        Ok(())
    }

    /// Get the PortClass for this bus type
    fn port_class(&self) -> ipc::PortClass {
        match self.bus_type {
            BusType::PCIe => ipc::PortClass::Pcie,
            BusType::Usb => ipc::PortClass::Usb,
            BusType::Platform => ipc::PortClass::Service,
            BusType::Ethernet => ipc::PortClass::Ethernet,
            BusType::Uart => ipc::PortClass::Uart,
            BusType::Klog => ipc::PortClass::Klog,
        }
    }

    /// Get the port subclass for this bus type
    fn port_subclass(&self) -> u16 {
        match self.bus_type {
            BusType::PCIe => 0, // Root port
            BusType::Usb => ipc::port_subclass::USB_XHCI,
            BusType::Platform => 0,
            BusType::Ethernet => ipc::port_subclass::NET_ETHERNET,
            BusType::Uart => ipc::port_subclass::CONSOLE_SERIAL,
            BusType::Klog => 0,
        }
    }

    /// Get capability flags for this bus type
    fn port_capabilities(&self) -> u16 {
        use ipc::port_caps;
        match self.bus_type {
            BusType::PCIe => port_caps::DMA | port_caps::IRQ | port_caps::MMIO,
            BusType::Usb => port_caps::DMA | port_caps::IRQ | port_caps::MMIO,
            BusType::Platform => port_caps::MMIO | port_caps::IRQ,
            BusType::Ethernet => port_caps::DMA | port_caps::IRQ | port_caps::MMIO,
            BusType::Uart | BusType::Klog => 0, // No hardware capabilities
        }
    }

    /// Check state machine invariants
    pub fn check_invariants(&self) -> bool {
        match self.state {
            BusState::Safe => {
                // Safe: bus mastering off, supervisor MAY be connected
                !self.bus_master_enabled
            }
            BusState::Claimed => {
                // Claimed: owner must exist
                self.owner_pid.is_some()
            }
            BusState::Resetting => {
                // Resetting: temporary state, bus mastering off
                !self.bus_master_enabled
            }
        }
    }

    /// Transition to a new state
    pub fn transition_to(&mut self, new_state: BusState, reason: StateChangeReason) {
        let old_state = self.state;

        if old_state == new_state {
            return;
        }

        kinfo!("bus", "state_change";
            name = self.port_name_str(),
            from = old_state.as_str(),
            to = new_state.as_str(),
            reason = reason.as_str());

        self.state = new_state;
        self.state_entered_at = uptime_ms();

        // Enforce invariants based on new state
        match new_state {
            BusState::Safe => {
                // Clear owner (owner is gone or reset complete)
                self.owner_ch = None;
                self.owner_pid = None;
                self.bus_master_enabled = false;
                self.disable_all_bus_mastering();

                // Notify supervisor (devd) — port is Safe, ready for new owner
                let _ = self.notify_supervisor(old_state, new_state, reason);
            }
            BusState::Claimed => {
                // Owner should already be set before calling transition
                let _ = self.notify_supervisor(old_state, new_state, reason);
            }
            BusState::Resetting => {
                // Owner is gone, disable hardware
                self.owner_ch = None;
                self.owner_pid = None;
                self.bus_master_enabled = false;
                self.disable_all_bus_mastering();

                // Notify supervisor (devd) that bus is resetting
                let _ = self.notify_supervisor(old_state, new_state, reason);
            }
        }

        debug_assert!(self.check_invariants(), "Invariant violation after transition!");
    }

    /// Disable bus mastering for all devices (hardware + tracking)
    fn disable_all_bus_mastering(&mut self) {
        // First, disable at hardware level
        match self.bus_type {
            BusType::PCIe => hw_pcie::disable_all_bus_mastering(self.bus_index, self.ecam_based, self.base_addr),
            BusType::Usb => { let _ = hw_usb::force_halt(self.bus_index, self.base_addr); }
            BusType::Platform | BusType::Ethernet | BusType::Uart | BusType::Klog => {
                // No bus mastering to disable
            }
        }

        // Then update our tracking - reset all device states including IOMMU mappings
        for i in 0..self.bm_count {
            self.bm_devices[i].reset();
        }
        self.bus_master_enabled = false;
    }

    /// Handle a new connection to the bus control port
    ///
    /// Accepts one owner connection at a time. Bus must be in Safe state.
    /// Supervisor (devd) channels are set separately when devd connects
    /// via the supervision protocol.
    pub fn handle_connect(&mut self, client_channel: ChannelId, client_pid: Pid) -> Result<(), BusError> {
        if self.state != BusState::Safe {
            return Err(BusError::Busy);
        }
        if self.owner_pid.is_some() {
            return Err(BusError::AlreadyClaimed);
        }

        self.owner_ch = Some(client_channel);
        self.owner_pid = Some(client_pid);
        kinfo!("bus", "owner_connected";
            name = self.port_name_str(),
            pid = client_pid as u64);

        // Transition to Claimed — owner is now managing hardware
        self.transition_to(BusState::Claimed, StateChangeReason::Connected);

        // Send state snapshot to owner so it knows hardware details
        self.send_state_snapshot(client_channel)?;

        // Send enumerated device list (if any devices)
        self.send_device_list(client_channel)?;
        Ok(())
    }

    /// Handle supervisor (devd) disconnecting
    pub fn handle_supervisor_disconnect(&mut self, _reason: StateChangeReason) {
        // Close the kernel-owned supervision channel
        if let Some(ch) = self.supervisor_ch {
            if let Ok(Some(peer)) = ipc::close_unchecked(ch) {
                let wake_list = crate::kernel::object_service::object_service()
                    .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::CLOSED);
                waker::wake(&wake_list, WakeReason::Closed);
            }
        }

        // Clear supervisor state so new devd can connect
        self.supervisor_ch = None;
        self.supervisor_pid = None;

        // Bus stays in current state — owner keeps running if present
        // devd crash doesn't kill a working driver
    }

    /// Handle owner (driver) disconnecting/exiting
    pub fn handle_owner_exit(&mut self, owner_pid: Pid) {
        kinfo!("bus", "owner_exit"; name = self.port_name_str(), pid = owner_pid as u64);

        // Close the kernel-owned owner channel
        if let Some(ch) = self.owner_ch {
            if let Ok(Some(peer)) = ipc::close_unchecked(ch) {
                let wake_list = crate::kernel::object_service::object_service()
                    .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::CLOSED);
                waker::wake(&wake_list, WakeReason::Closed);
            }
        }

        // Clear owner state
        self.owner_ch = None;
        self.owner_pid = None;

        if self.state == BusState::Claimed {
            // Owner left while bus was claimed — reset hardware
            self.transition_to(BusState::Resetting, StateChangeReason::DriverExited);
            self.perform_hardware_reset();
            self.transition_to(BusState::Safe, StateChangeReason::ResetComplete);
        }
    }

    /// Handle atomic handoff to new owner
    pub fn handle_handoff(&mut self, new_channel: ChannelId, new_pid: Pid) -> Result<(), BusError> {
        if self.state != BusState::Claimed {
            return Err(BusError::NotClaimed);
        }

        // Atomic transfer - NO state change, NO reset
        kinfo!("bus", "handoff"; name = self.port_name_str(), new_pid = new_pid as u64);

        self.owner_ch = Some(new_channel);
        self.owner_pid = Some(new_pid);

        // Send snapshot and device list to new owner
        self.send_state_snapshot(new_channel)?;
        self.send_device_list(new_channel)?;

        Ok(())
    }

    /// Send state snapshot to a channel
    fn send_state_snapshot(&self, channel: ChannelId) -> Result<(), BusError> {
        let snapshot = StateSnapshot {
            msg_type: BusControlMsgType::StateSnapshot as u8,
            bus_type: self.bus_type as u8,
            bus_index: self.bus_index,
            state: self.state as u8,
            device_count: self.enum_count as u8,
            capabilities: self.query_capabilities(),
            _reserved: [0; 2],
            since_boot_ms: uptime_ms(),
        };

        let bytes = snapshot.to_bytes();

        {
            let mut msg = Message::new();
            msg.header.msg_type = MessageType::Data;
            msg.header.payload_len = bytes.len() as u32;
            msg.payload[..bytes.len()].copy_from_slice(&bytes);

            match ipc::send_unchecked(channel, msg) {
                Ok(peer) => {
                    let wake_list = crate::kernel::object_service::object_service()
                        .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::READABLE);
                    waker::wake(&wake_list, WakeReason::Readable);
                    Ok(())
                }
                Err(_) => Err(BusError::SendFailed),
            }
        }
    }

    /// Notify supervisor (devd) of state change via supervision protocol
    ///
    /// Sends a StateChanged message on the supervision channel.
    fn notify_supervisor(&self, old_state: BusState, new_state: BusState, reason: StateChangeReason) -> Result<(), BusError> {
        let Some(channel) = self.supervisor_ch else {
            return Ok(());
        };

        // Build supervision::STATE_CHANGED message
        let reason_code = match reason {
            StateChangeReason::Connected => abi::supervision::REASON_OWNER_CONNECTED,
            StateChangeReason::DriverExited | StateChangeReason::OwnerCrashed | StateChangeReason::Disconnected => {
                abi::supervision::REASON_OWNER_EXITED
            }
            StateChangeReason::ResetComplete | StateChangeReason::ResetRequested => {
                abi::supervision::REASON_RESET_COMPLETE
            }
            StateChangeReason::Handoff | StateChangeReason::DriverClaimed => {
                abi::supervision::REASON_OWNER_CONNECTED
            }
        };

        let payload = [
            abi::supervision::STATE_CHANGED,
            old_state as u8,
            new_state as u8,
            reason_code,
        ];

        let mut msg = Message::new();
        msg.header.msg_type = MessageType::Data;
        msg.header.payload_len = payload.len() as u32;
        msg.payload[..payload.len()].copy_from_slice(&payload);

        match ipc::send_unchecked(channel, msg) {
            Ok(peer) => {
                let wake_list = crate::kernel::object_service::object_service()
                    .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::READABLE);
                waker::wake(&wake_list, WakeReason::Readable);
                Ok(())
            }
            Err(_) => Err(BusError::SendFailed),
        }
    }

    /// Perform hardware reset sequence
    pub fn perform_hardware_reset(&mut self) {
        match self.bus_type {
            BusType::PCIe => self.pcie_reset_sequence(),
            BusType::Usb => self.usb_reset_sequence(),
            BusType::Platform | BusType::Ethernet | BusType::Uart | BusType::Klog => {
                self.hardware_verified = true; // No hardware to reset
            }
        }
    }

    /// PCIe hardware reset sequence
    ///
    /// Behavior depends on platform:
    /// - ECAM-based (QEMU): Skip hardware reset, just mark as verified
    /// - MAC-based (MT7988): Full reset via INFRACFG
    ///
    /// MT7988 Note: The RST_CTRL register at offset 0x148 has unusual behavior
    /// on MT7988 - writes don't take effect. This is similar to EN7581 which
    /// skips RST_CTRL manipulation entirely in the Linux driver.
    ///
    /// For MT7988, we rely on INFRACFG PEXTP_MAC_SWRST for reset control:
    /// - Assert PEXTP_MAC_SWRST = MAC in reset (safe state, no DMA)
    /// - Deassert PEXTP_MAC_SWRST = MAC running
    fn pcie_reset_sequence(&mut self) {
        use crate::kernel::arch::mmio::MmioRegion;
        use crate::klog;

        let index = self.bus_index as usize;

        // ECAM-based platforms (QEMU virt): no hardware reset needed
        if self.ecam_based {
            kinfo!("bus", "pcie_ecam_mode"; port = index as u64);
            self.hardware_verified = true;
            return;
        }

        // MAC-based platforms (MT7988): full hardware reset sequence
        let mac_base = self.base_addr as usize;
        if mac_base == 0 {
            kerror!("bus", "pcie_invalid_base"; index = index as u64);
            self.hardware_verified = false;
            return;
        };

        // Step 1: Enable clocks and deassert INFRACFG reset
        hw_pcie::enable_clocks(index);

        // Now MAC should be accessible
        let mac = MmioRegion::new(mac_base);

        // Step 2: Verify MAC is accessible
        let test_read = mac.read32(0x00);
        if test_read == 0 || test_read == 0xFFFFFFFF {
            kerror!("bus", "pcie_mac_not_accessible"; index = index as u64, mac = klog::hex32(mac_base as u32));
            hw_pcie::disable_clocks(index);
            self.hardware_verified = false;
            return;
        }

        // MAC is accessible - mark as verified
        self.hardware_verified = true;
        kinfo!("bus", "pcie_verified"; port = index as u64, mac = klog::hex32(mac_base as u32));

        // Step 3: Assert INFRACFG reset and disable clocks (safe state)
        hw_pcie::disable_clocks(index);
    }

    /// USB/xHCI hardware reset sequence
    fn usb_reset_sequence(&mut self) {
        let (verified, u3_count, u2_count) = hw_usb::reset_sequence(self.bus_index, self.base_addr);
        self.hardware_verified = verified;

        if verified {
            kinfo!("bus", "usb_verified"; port = self.bus_index as u64, u3 = u3_count as u64, u2 = u2_count as u64);
        } else {
            kerror!("bus", "usb_verify_failed"; index = self.bus_index as u64);
        }

        // Power down to safe state
        hw_usb::power_down(self.bus_index, self.base_addr);
    }

    /// Add an enumerated device to this bus
    /// Returns true if added, false if full
    pub fn add_enum_device(&mut self, dev: abi::BusDevice) -> bool {
        if self.enum_count >= MAX_DEVICES_PER_BUS {
            return false;
        }
        self.enum_devices[self.enum_count] = dev;
        self.enum_count += 1;
        true
    }

    /// Send device list to a channel (after StateSnapshot)
    /// Chunks devices into IPC-sized messages (max 17 per chunk at 32 bytes each)
    fn send_device_list(&self, channel: ChannelId) -> Result<(), BusError> {
        if self.enum_count == 0 {
            return Ok(());
        }

        const MAX_PER_CHUNK: usize = 17; // (576 - 8 header) / 32 = 17
        let total = self.enum_count;
        let mut offset = 0;

        while offset < total {
            let count = (total - offset).min(MAX_PER_CHUNK);

            // Build DeviceList message
            let mut msg = Message::new();
            msg.header.msg_type = MessageType::Data;

            // Header: [msg_type, total, offset, count, reserved(4)]
            msg.payload[0] = super::protocol::BusControlMsgType::DeviceList as u8;
            msg.payload[1] = total as u8;
            msg.payload[2] = offset as u8;
            msg.payload[3] = count as u8;
            // payload[4..8] = reserved (already zeroed)

            // Copy BusDevice entries
            let dev_start = 8;
            for i in 0..count {
                let dev = &self.enum_devices[offset + i];
                let dev_bytes = unsafe {
                    core::slice::from_raw_parts(
                        dev as *const abi::BusDevice as *const u8,
                        core::mem::size_of::<abi::BusDevice>(),
                    )
                };
                let dst = dev_start + i * 32;
                msg.payload[dst..dst + 32].copy_from_slice(dev_bytes);
            }

            msg.header.payload_len = (dev_start + count * 32) as u32;

            match ipc::send_unchecked(channel, msg) {
                Ok(peer) => {
                    let wake_list = crate::kernel::object_service::object_service()
                        .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::READABLE);
                    waker::wake(&wake_list, WakeReason::Readable);
                }
                Err(_) => return Err(BusError::SendFailed),
            }

            offset += count;
        }

        Ok(())
    }

    /// Handle a control message on the owner channel
    ///
    /// Owner sends hardware-access messages (bus mastering, reset).
    /// SetDriver is removed — owner is identified by who connects.
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
                if self.state == BusState::Resetting {
                    return Err(BusError::Busy);
                }

                kinfo!("bus", "reset_requested"; name = self.port_name_str());

                // Transition to Resetting state
                self.transition_to(BusState::Resetting, StateChangeReason::ResetRequested);

                // Perform actual hardware reset
                self.perform_hardware_reset();

                // Transition to Safe
                self.transition_to(BusState::Safe, StateChangeReason::ResetComplete);

                Ok(())
            }
            _ => Err(BusError::InvalidMessage),
        }
    }

    /// Enable bus mastering for a device
    fn enable_bus_mastering(&mut self, device_id: u16) -> Result<(), BusError> {
        // Find or create device entry
        let slot = self.bm_devices[..self.bm_count]
            .iter()
            .position(|d| d.device_id == device_id)
            .unwrap_or_else(|| {
                if self.bm_count < MAX_DEVICES_PER_BUS {
                    let slot = self.bm_count;
                    self.bm_count += 1;
                    self.bm_devices[slot].device_id = device_id;
                    slot
                } else {
                    0 // Shouldn't happen, but fallback
                }
            });

        // Actually write to hardware
        let hw_result = match self.bus_type {
            BusType::PCIe => hw_pcie::set_bus_mastering(self.bus_index, device_id, true, self.ecam_based, self.base_addr),
            BusType::Usb => hw_usb::set_dma_allowed(self.bus_index, device_id, true, self.base_addr),
            BusType::Platform | BusType::Ethernet | BusType::Uart | BusType::Klog => Ok(()), // No bus mastering control
        };

        if let Err(e) = hw_result {
            kerror!("bus", "bm_enable_failed"; name = self.port_name_str(), device = device_id as u64);
            return Err(e);
        }

        // Use transition method
        let _ = self.bm_devices[slot].enable();
        self.bus_master_enabled = true;

        Ok(())
    }

    /// Disable bus mastering for a device
    fn disable_bus_mastering(&mut self, device_id: u16) -> Result<(), BusError> {
        if let Some(slot) = self.bm_devices[..self.bm_count]
            .iter()
            .position(|d| d.device_id == device_id)
        {
            // Actually write to hardware
            let hw_result = match self.bus_type {
                BusType::PCIe => hw_pcie::set_bus_mastering(self.bus_index, device_id, false, self.ecam_based, self.base_addr),
                BusType::Usb => hw_usb::set_dma_allowed(self.bus_index, device_id, false, self.base_addr),
                BusType::Platform | BusType::Ethernet | BusType::Uart | BusType::Klog => Ok(()), // No bus mastering control
            };

            if let Err(e) = hw_result {
                kerror!("bus", "bm_disable_failed"; name = self.port_name_str(), device = device_id as u64);
                let _ = e;  // error logged
                // Continue anyway - we must disable in our tracking
            }

            // Use transition method
            self.bm_devices[slot].disable();
        }

        // Update global flag
        self.bus_master_enabled = self.bm_devices[..self.bm_count]
            .iter()
            .any(|d| d.is_enabled());

        Ok(())
    }
}
