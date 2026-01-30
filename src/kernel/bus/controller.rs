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
    /// Get current state
    pub fn state(&self) -> BusState { self.state }

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
                state: BusMasteringState::Disabled,
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
        let config = super::config::bus_config();
        let base_addr = match self.bus_type {
            BusType::PCIe => {
                config.pcie_base(self.bus_index as usize).unwrap_or(0) as u32
            }
            BusType::Usb => {
                config.usb_base(self.bus_index as usize).unwrap_or(0) as u32
            }
            BusType::Platform => 0, // Platform devices have individual addresses
        };

        let mut info = BusInfo {
            bus_type: self.bus_type as u8,
            bus_index: self.bus_index,
            state: self.state as u8,
            _pad: 0,
            base_addr,
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

    /// Convert to DeviceInfo for unified device list
    /// Uses chipset:instance naming (e.g., "mt7988-pcie:0", "xhci:0")
    pub fn to_device_info(&self) -> DeviceInfo {
        let config = super::config::bus_config();
        let (base_addr, size) = match self.bus_type {
            BusType::PCIe => {
                if let Some(base) = config.pcie_base(self.bus_index as usize) {
                    (base as u64, 0x10000)
                } else {
                    (0, 0)
                }
            }
            BusType::Usb => {
                if let Some(base) = config.usb_base(self.bus_index as usize) {
                    (base as u64, 0x10000)
                } else {
                    (0, 0)
                }
            }
            BusType::Platform => (0, 0),
        };

        // Build device name: "mt7988-pcie:0", "xhci:0", etc.
        let mut name = [0u8; 16];
        let mut pos = 0;

        let chipset_type: &[u8] = match self.bus_type {
            BusType::PCIe => b"mt7988-pcie:",
            BusType::Usb => b"xhci:",
            BusType::Platform => b"platform:",
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
            irq: 0,  // Bus controllers don't have a single IRQ
            base_addr,
            size,
            _reserved: [0; 4],
        }
    }

    /// Query hardware capabilities for this bus
    /// Returns a bitmask of bus_caps flags appropriate for the bus type
    pub fn query_capabilities(&self) -> u8 {
        match self.bus_type {
            BusType::PCIe => hw_pcie::query_capabilities(self.bus_index),
            BusType::Usb => hw_usb::query_capabilities(self.bus_index),
            BusType::Platform => bus_caps::PLATFORM_MMIO | bus_caps::PLATFORM_IRQ,
        }
    }

    /// Register the bus control port
    /// Called during kernel init
    pub fn register_port(&mut self, kernel_pid: Pid) -> Result<(), IpcError> {
        let name = self.port_name_str();
        kinfo!("bus", "register_port"; name = name);

        let (_port_id, listen_ch) = ipc::port_register(name.as_bytes(), kernel_pid)?;

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
                // Keep owner_channel/owner_pid - devd stays connected to manage bus
                // Only clear driver_pid (driver is gone)
                self.driver_pid = None;
                self.bus_master_enabled = false;
                self.disable_all_bus_mastering();

                // Notify owner (devd) that bus is now Safe
                // This allows devd to spawn drivers
                let _ = self.notify_owner();
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

                // Notify owner (devd) that bus is resetting
                let _ = self.notify_owner();
            }
        }

        debug_assert!(self.check_invariants(), "Invariant violation after transition!");
    }

    /// Disable bus mastering for all devices (hardware + tracking)
    fn disable_all_bus_mastering(&mut self) {
        // First, disable at hardware level
        match self.bus_type {
            BusType::PCIe => hw_pcie::disable_all_bus_mastering(self.bus_index),
            BusType::Usb => { let _ = hw_usb::force_halt(self.bus_index); }
            BusType::Platform => {
                // Platform devices don't have bus mastering
            }
        }

        // Then update our tracking - reset all device states including IOMMU mappings
        for i in 0..self.device_count {
            self.devices[i].reset();
        }
        self.bus_master_enabled = false;
    }

    /// Handle a new connection to the bus control port
    /// Only supervisor (devd) should connect - drivers are authorized via SetDriver
    pub fn handle_connect(&mut self, client_channel: ChannelId, client_pid: Pid) -> Result<(), BusError> {
        // Reject if already has an owner
        if self.owner_pid.is_some() {
            return Err(BusError::AlreadyClaimed);
        }

        match self.state {
            BusState::Safe | BusState::Resetting => {
                // Accept connection in any state
                // devd sees current state and waits for Safe if needed
                self.owner_channel = Some(client_channel);
                self.owner_pid = Some(client_pid);
                kinfo!("bus", "supervisor_connected";
                    name = self.port_name_str(),
                    pid = client_pid as u64,
                    state = self.state.as_str());
                self.send_state_snapshot(client_channel)?;
                Ok(())
            }
            BusState::Claimed => {
                // Bus is in use by a driver - reject
                Err(BusError::AlreadyClaimed)
            }
        }
    }

    /// Handle owner (devd) disconnecting
    pub fn handle_disconnect(&mut self, reason: StateChangeReason) {
        // Close the kernel-owned channel to free resources
        if let Some(ch) = self.owner_channel {
            if let Ok(wake_list) = ipc::close_unchecked(ch) {
                waker::wake(&wake_list, WakeReason::Closed);
            }
        }

        // Clear ownership so new devd can connect
        self.owner_channel = None;
        self.owner_pid = None;
        self.driver_pid = None;

        if self.state == BusState::Claimed {
            // Owner left while bus was claimed - go to RESETTING then SAFE
            self.transition_to(BusState::Resetting, reason);

            // Perform hardware reset
            self.perform_hardware_reset();

            // Reset complete - go to SAFE
            self.transition_to(BusState::Safe, StateChangeReason::ResetComplete);
        }
        // If already Safe, just clear ownership (done above)
    }

    /// Handle driver process exit
    ///
    /// Called when the driver_pid process exits. Resets the bus and notifies
    /// the supervisor (devd) via owner_channel so it can restart the driver.
    pub fn handle_driver_exit(&mut self, driver_pid: Pid) {
        kinfo!("bus", "driver_exit"; name = self.port_name_str(), pid = driver_pid as u64);

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
        self.notify_owner_safe();
    }

    /// Handle atomic handoff to new owner
    pub fn handle_handoff(&mut self, new_channel: ChannelId, new_pid: Pid) -> Result<(), BusError> {
        if self.state != BusState::Claimed {
            return Err(BusError::NotClaimed);
        }

        // Atomic transfer - NO state change, NO reset
        kinfo!("bus", "handoff"; name = self.port_name_str(), new_pid = new_pid as u64);

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
                Ok(wake_list) => {
                    waker::wake(&wake_list, WakeReason::Readable);
                    Ok(())
                }
                Err(_) => Err(BusError::SendFailed),
            }
        }
    }

    /// Notify owner (devd) of state change
    ///
    /// Sends a state snapshot to the owner channel if connected.
    /// Returns Ok(()) if no owner or send succeeded, Err on send failure.
    fn notify_owner(&self) -> Result<(), BusError> {
        if let Some(channel) = self.owner_channel {
            self.send_state_snapshot(channel)
        } else {
            Ok(())
        }
    }

    /// Notify owner, logging error if it fails
    ///
    /// Use this when notification failure shouldn't affect control flow.
    fn notify_owner_safe(&self) {
        if let Err(_) = self.notify_owner() {
            kerror!("bus", "notify_failed"; name = self.port_name_str());
        }
    }

    /// Perform hardware reset sequence
    pub fn perform_hardware_reset(&mut self) {
        match self.bus_type {
            BusType::PCIe => self.pcie_reset_sequence(),
            BusType::Usb => self.usb_reset_sequence(),
            BusType::Platform => self.hardware_verified = true, // No hardware to reset
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
        use crate::arch::aarch64::mmio::MmioRegion;
        use crate::klog;

        let index = self.bus_index as usize;
        let config = super::config::bus_config();

        // ECAM-based platforms (QEMU virt): no hardware reset needed
        // The hypervisor manages PCIe state, we just need to register the bus
        if config.is_pcie_ecam_based() {
            kinfo!("bus", "pcie_ecam_mode"; port = index as u64);
            self.hardware_verified = true;
            return;
        }

        // MAC-based platforms (MT7988): full hardware reset sequence
        let Some(mac_base) = config.pcie_base(index) else {
            kerror!("bus", "pcie_invalid_index"; index = index as u64);
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
        let (verified, u3_count, u2_count) = hw_usb::reset_sequence(self.bus_index);
        self.hardware_verified = verified;

        if verified {
            kinfo!("bus", "usb_verified"; port = self.bus_index as u64, u3 = u3_count as u64, u2 = u2_count as u64);
        } else {
            kerror!("bus", "usb_verify_failed"; index = self.bus_index as u64);
        }

        // Power down to safe state
        hw_usb::power_down(self.bus_index);
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

                kinfo!("bus", "reset_requested"; name = self.port_name_str());

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

                kinfo!("bus", "driver_set"; name = self.port_name_str(), driver_pid = driver_pid as u64);

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
            BusType::PCIe => hw_pcie::set_bus_mastering(self.bus_index, device_id, true),
            BusType::Usb => hw_usb::set_dma_allowed(self.bus_index, device_id, true),
            BusType::Platform => Ok(()), // Platform devices don't have bus mastering
        };

        if let Err(e) = hw_result {
            kerror!("bus", "bm_enable_failed"; name = self.port_name_str(), device = device_id as u64);
            return Err(e);
        }

        // Use transition method
        let _ = self.devices[slot].enable();
        self.bus_master_enabled = true;

        Ok(())
    }

    /// Disable bus mastering for a device
    fn disable_bus_mastering(&mut self, device_id: u16) -> Result<(), BusError> {
        if let Some(slot) = self.devices[..self.device_count]
            .iter()
            .position(|d| d.device_id == device_id)
        {
            // Actually write to hardware
            let hw_result = match self.bus_type {
                BusType::PCIe => hw_pcie::set_bus_mastering(self.bus_index, device_id, false),
                BusType::Usb => hw_usb::set_dma_allowed(self.bus_index, device_id, false),
                BusType::Platform => Ok(()), // Platform devices don't have bus mastering
            };

            if let Err(e) = hw_result {
                kerror!("bus", "bm_disable_failed"; name = self.port_name_str(), device = device_id as u64);
                let _ = e;  // error logged
                // Continue anyway - we must disable in our tracking
            }

            // Use transition method
            self.devices[slot].disable();
        }

        // Update global flag
        self.bus_master_enabled = self.devices[..self.device_count]
            .iter()
            .any(|d| d.is_enabled());

        Ok(())
    }
}
