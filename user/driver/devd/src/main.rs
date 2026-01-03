//! Device Supervisor Daemon (devd)
//!
//! The unified device manager and service supervisor for the microkernel.
//! Acts as PID 1 (init) - connects to bus control ports, enumerates devices,
//! spawns drivers, and supervises all with proper state machines.
//!
//! ## Architecture
//!
//! ```text
//! devd (PID 1)
//!  ├─ connects to /kernel/bus/pcie0, /kernel/bus/usb0
//!  ├─ receives StateSnapshot from kernel
//!  ├─ enumerates devices on each bus
//!  ├─ matches devices to drivers, spawns driver daemons
//!  ├─ tracks device state machines (DISCOVERED→BINDING→BOUND→...)
//!  └─ handles faults with escalation (device reset → link reset → bus reset)
//! ```
//!
//! ## State Machines
//!
//! Every device has an explicit state machine. See ARCHITECTURE-MASTER.md.

#![no_std]
#![no_main]
#![allow(dead_code)]  // Device manager state machine reserved for future use

use userlib::println;
use userlib::syscall::{self, PciDeviceInfo, Event, event_type, event_flags};
use userlib::ipc::protocols::devd::{
    DevdRequest, DevdResponse, Node, NodeList, PropertyList, EventType,
    MAX_PATH, MAX_NODES,
};
use userlib::ipc::Message;

// =============================================================================
// Constants
// =============================================================================

const MAX_BUSES: usize = 8;
const MAX_DRIVERS: usize = 16;
const MAX_DEVICES: usize = 32;
const MAX_RESTARTS: u32 = 3;
const RESTART_DELAY_MS: u64 = 1000;
const BIND_TIMEOUT_NS: u64 = 30_000_000_000;  // 30s in nanoseconds - USB enumeration can be slow
const QUIET_PERIOD_MS: u64 = 60_000;

// PCI class codes
const PCI_CLASS_USB: u32 = 0x0C0300;
const PCI_CLASS_NVME: u32 = 0x010802;

// Bus control message types (must match kernel bus.rs)
const MSG_STATE_SNAPSHOT: u8 = 0;
const MSG_STATE_CHANGED: u8 = 1;
const MSG_ENABLE_BUS_MASTERING: u8 = 16;
const MSG_DISABLE_BUS_MASTERING: u8 = 17;
const MSG_REQUEST_RESET: u8 = 20;
const MSG_SET_DRIVER: u8 = 22;

// hw: scheme message types
const HW_MSG_OPEN: u8 = 1;
const HW_MSG_READ: u8 = 2;
const HW_MSG_CLOSE: u8 = 3;
const HW_MSG_RESPONSE: u8 = 128;

// Device tree constants
const MAX_TREE_NODES: usize = 64;
const MAX_SUBSCRIPTIONS: usize = 16;

// =============================================================================
// Bus Types and State
// =============================================================================

/// Hardware bus types (must match kernel)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BusType {
    PCIe = 0,
    Usb = 1,
    Platform = 2,
}

/// Bus state as reported by kernel
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum KernelBusState {
    Safe = 0,
    Claimed = 1,
    Resetting = 2,
}

/// Our connection to a kernel bus controller
pub struct BusConnection {
    /// Bus type
    pub bus_type: BusType,
    /// Bus index (0, 1, ...)
    pub bus_index: u8,
    /// Channel to kernel bus controller
    pub channel: u32,
    /// Last known kernel state
    pub kernel_state: KernelBusState,
    /// Are we connected?
    pub connected: bool,
    /// Controller base address (MMIO)
    pub base_addr: u32,
}

impl BusConnection {
    pub const fn empty() -> Self {
        Self {
            bus_type: BusType::PCIe,
            bus_index: 0,
            channel: 0,
            kernel_state: KernelBusState::Safe,
            connected: false,
            base_addr: 0,
        }
    }
}

// MT7988A bus controller addresses
const PCIE_MAC_BASES: [u32; 4] = [
    0x1130_0000,  // PCIe0
    0x1131_0000,  // PCIe1
    0x1128_0000,  // PCIe2
    0x1129_0000,  // PCIe3
];
const USB_MAC_BASES: [u32; 2] = [
    0x1119_0000,  // SSUSB0 (M.2 slot)
    0x1120_0000,  // SSUSB1 (USB-A ports)
];

// =============================================================================
// Device State Machine
// =============================================================================

/// Device state machine
/// See ARCHITECTURE-MASTER.md for full state diagram
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceState {
    /// Device discovered, no driver assigned yet
    Discovered,
    /// Driver is being bound (timeout in progress)
    Binding { deadline: u64 },
    /// Driver successfully bound, operating normally
    Bound,
    /// Device has faulted, tracking escalation
    Faulted,
    /// Device is being reset at current escalation level
    Resetting,
    /// Bus is resetting, device suspended
    Suspended,
    /// Device has failed permanently
    Dead,
}

impl DeviceState {
    pub fn as_str(&self) -> &'static str {
        match self {
            DeviceState::Discovered => "discovered",
            DeviceState::Binding { .. } => "binding",
            DeviceState::Bound => "bound",
            DeviceState::Faulted => "faulted",
            DeviceState::Resetting => "resetting",
            DeviceState::Suspended => "suspended",
            DeviceState::Dead => "dead",
        }
    }
}

// =============================================================================
// Fault Tracker (Escalation)
// =============================================================================

/// Escalation levels for fault handling
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum EscalationLevel {
    /// Function-level reset or D3→D0 power cycle
    DeviceReset = 0,
    /// PCIe link retrain (LTSSM reset)
    LinkRetrain = 1,
    /// Secondary bus reset (affects slot/segment)
    SegmentReset = 2,
    /// Full bus reset via kernel (nuclear option)
    BusReset = 3,
    /// Give up - device is dead
    Dead = 4,
}

impl EscalationLevel {
    pub fn next(self) -> Self {
        match self {
            EscalationLevel::DeviceReset => EscalationLevel::LinkRetrain,
            EscalationLevel::LinkRetrain => EscalationLevel::SegmentReset,
            EscalationLevel::SegmentReset => EscalationLevel::BusReset,
            EscalationLevel::BusReset => EscalationLevel::Dead,
            EscalationLevel::Dead => EscalationLevel::Dead,
        }
    }

    pub fn max_attempts(self) -> u32 {
        match self {
            EscalationLevel::DeviceReset => 3,
            EscalationLevel::LinkRetrain => 2,
            EscalationLevel::SegmentReset => 2,
            EscalationLevel::BusReset => 1,
            EscalationLevel::Dead => 0,
        }
    }
}

/// Tracks fault escalation for a device
#[derive(Clone, Copy)]
pub struct FaultTracker {
    /// Current escalation level
    pub level: EscalationLevel,
    /// Attempts at current level
    pub attempts: u32,
    /// Backoff time in ms
    pub backoff_ms: u32,
    /// Time of last fault
    pub last_fault: u64,
    /// Consecutive successful operations (for de-escalation)
    pub consecutive_success: u32,
}

impl FaultTracker {
    pub const fn new() -> Self {
        Self {
            level: EscalationLevel::DeviceReset,
            attempts: 0,
            backoff_ms: 5000,  // 5 seconds for debugging
            last_fault: 0,
            consecutive_success: 0,
        }
    }

    /// Record a fault and get the action to take
    pub fn escalate(&mut self, now: u64) -> (EscalationLevel, u64) {
        // If quiet for a while, reset escalation
        if now > self.last_fault + QUIET_PERIOD_MS {
            self.level = EscalationLevel::DeviceReset;
            self.attempts = 0;
            self.backoff_ms = 5000;  // 5 seconds for debugging
        }

        self.attempts += 1;
        self.last_fault = now;
        self.consecutive_success = 0;

        // Escalate if we've exhausted attempts at this level
        if self.attempts > self.level.max_attempts() {
            self.level = self.level.next();
            self.attempts = 1;
            self.backoff_ms = 5000;  // 5 seconds for debugging
        }

        let wait = self.backoff_ms as u64;
        self.backoff_ms = (self.backoff_ms * 2).min(5000);

        (self.level, wait)
    }

    /// Record a successful operation
    pub fn record_success(&mut self) {
        self.consecutive_success += 1;
        // De-escalate after many successful operations
        if self.consecutive_success > 100 && self.level > EscalationLevel::DeviceReset {
            self.level = EscalationLevel::DeviceReset;
            self.consecutive_success = 0;
            self.attempts = 0;
        }
    }
}

// =============================================================================
// Device and Driver Structures
// =============================================================================

/// Device class for matching
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceClass {
    Serial,
    UsbController,
    Storage,
    Network,
    Hid,
    Other,
}

/// A discovered device
#[derive(Clone)]
pub struct Device {
    /// Unique device ID
    pub id: u16,
    /// Device path
    pub path: [u8; 64],
    pub path_len: usize,
    /// Bus this device is on
    pub bus_index: u8,
    pub bus_type: BusType,
    /// Device class
    pub class: DeviceClass,
    /// Vendor/Device IDs
    pub vendor_id: u16,
    pub device_id: u16,
    /// PCI BDF (if PCIe)
    pub pci_bdf: u32,
    /// Current state
    pub state: DeviceState,
    /// Assigned driver
    pub driver_id: Option<usize>,
    /// Fault tracker
    pub fault_tracker: FaultTracker,
}

impl Device {
    pub const fn empty() -> Self {
        Self {
            id: 0,
            path: [0; 64],
            path_len: 0,
            bus_index: 0,
            bus_type: BusType::PCIe,
            class: DeviceClass::Other,
            vendor_id: 0,
            device_id: 0,
            pci_bdf: 0,
            state: DeviceState::Discovered,
            driver_id: None,
            fault_tracker: FaultTracker::new(),
        }
    }

    pub fn set_path(&mut self, s: &str) {
        let bytes = s.as_bytes();
        let len = bytes.len().min(self.path.len());
        self.path[..len].copy_from_slice(&bytes[..len]);
        self.path_len = len;
    }

    pub fn path_str(&self) -> &str {
        core::str::from_utf8(&self.path[..self.path_len]).unwrap_or("")
    }

    /// Transition to a new state (with logging)
    pub fn transition(&mut self, new_state: DeviceState) {
        if self.state != new_state {
            println!("[devd] Device {} {:?} -> {:?}",
                     self.path_str(), self.state, new_state);
            self.state = new_state;
        }
    }
}

/// Driver state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverState {
    Stopped,
    Starting,
    Running,
    Crashed,
    Failed,
}

/// A supervised driver
pub struct DriverEntry {
    pub binary: &'static str,
    pub pid: u32,
    pub state: DriverState,
    pub restart_count: u32,
    pub device_id: Option<u16>,
    pub restart_at: u64,
}

impl DriverEntry {
    pub const fn empty() -> Self {
        Self {
            binary: "",
            pid: 0,
            state: DriverState::Stopped,
            restart_count: 0,
            device_id: None,
            restart_at: 0,
        }
    }

    pub fn is_active(&self) -> bool {
        self.pid != 0
    }
}

/// Match rule
pub struct MatchRule {
    pub bus: Option<BusType>,
    pub class: Option<DeviceClass>,
    pub pci_class: Option<u32>,
    pub driver: &'static str,
}

// =============================================================================
// Match Rules
// =============================================================================

static MATCH_RULES: &[MatchRule] = &[
    // Platform devices
    MatchRule {
        bus: Some(BusType::Platform),
        class: Some(DeviceClass::Serial),
        pci_class: None,
        driver: "shell",
    },
    // DISABLED for wifid2 testing
    // MatchRule {
    //     bus: Some(BusType::Usb),
    //     class: Some(DeviceClass::UsbController),
    //     pci_class: None,
    //     driver: "usbd",
    // },
    // MatchRule {
    //     bus: Some(BusType::PCIe),
    //     class: None,
    //     pci_class: Some(PCI_CLASS_USB),
    //     driver: "usbd",
    // },
    MatchRule {
        bus: Some(BusType::PCIe),
        class: None,
        pci_class: Some(PCI_CLASS_NVME),
        driver: "nvmed",
    },
];

// =============================================================================
// Device Tree (Path-based Resource Registry)
// =============================================================================

/// A subscription to device tree changes
#[derive(Clone, Copy)]
struct Subscription {
    /// Pattern to match (glob-style)
    pattern: [u8; MAX_PATH],
    pattern_len: u8,
    /// Channel to notify when matching node changes
    channel: u32,
    /// Active flag
    active: bool,
}

impl Subscription {
    const fn empty() -> Self {
        Self {
            pattern: [0; MAX_PATH],
            pattern_len: 0,
            channel: 0,
            active: false,
        }
    }

    fn pattern_str(&self) -> &str {
        core::str::from_utf8(&self.pattern[..self.pattern_len as usize]).unwrap_or("")
    }
}

/// The device tree - stores nodes with paths and properties
struct DeviceTree {
    nodes: [Node; MAX_TREE_NODES],
    node_count: usize,
    subscriptions: [Subscription; MAX_SUBSCRIPTIONS],
    sub_count: usize,
}

impl DeviceTree {
    const fn new() -> Self {
        const EMPTY_NODE: Node = Node::empty();
        const EMPTY_SUB: Subscription = Subscription::empty();
        Self {
            nodes: [EMPTY_NODE; MAX_TREE_NODES],
            node_count: 0,
            subscriptions: [EMPTY_SUB; MAX_SUBSCRIPTIONS],
            sub_count: 0,
        }
    }

    /// Register a new node (or update if exists)
    fn register(&mut self, path: &str, props: PropertyList) -> Result<(), i32> {
        // Check if node already exists
        for i in 0..self.node_count {
            if self.nodes[i].path_str() == path {
                // Update existing node
                self.nodes[i].properties = props;
                self.notify_subscribers(path, EventType::Updated);
                return Ok(());
            }
        }

        // Add new node
        if self.node_count >= MAX_TREE_NODES {
            return Err(-12); // ENOMEM
        }

        let mut node = Node::new(path);
        node.properties = props;
        self.nodes[self.node_count] = node;
        self.node_count += 1;

        self.notify_subscribers(path, EventType::Created);
        Ok(())
    }

    /// Update an existing node's properties
    fn update(&mut self, path: &str, props: PropertyList) -> Result<(), i32> {
        for i in 0..self.node_count {
            if self.nodes[i].path_str() == path {
                self.nodes[i].properties = props;
                self.notify_subscribers(path, EventType::Updated);
                return Ok(());
            }
        }
        Err(-2) // ENOENT
    }

    /// Remove a node
    fn remove(&mut self, path: &str) -> Result<(), i32> {
        for i in 0..self.node_count {
            if self.nodes[i].path_str() == path {
                // Notify before removal
                self.notify_subscribers(path, EventType::Removed);

                // Shift remaining nodes down
                for j in i..self.node_count - 1 {
                    self.nodes[j] = self.nodes[j + 1].clone();
                }
                self.node_count -= 1;
                return Ok(());
            }
        }
        Err(-2) // ENOENT
    }

    /// Query nodes matching a glob pattern
    fn query(&self, pattern: &str) -> NodeList {
        let mut result = NodeList::new();
        for i in 0..self.node_count {
            let path = self.nodes[i].path_str();
            if glob_match(pattern, path) && (result.count as usize) < MAX_NODES {
                result.push(self.nodes[i].clone());
            }
        }
        result
    }

    /// Add a subscription
    fn subscribe(&mut self, pattern: &str, channel: u32) -> Result<(), i32> {
        // Check for existing subscription from same channel
        for i in 0..self.sub_count {
            if self.subscriptions[i].active
                && self.subscriptions[i].channel == channel
                && self.subscriptions[i].pattern_str() == pattern
            {
                return Ok(()); // Already subscribed
            }
        }

        // Find empty slot or add new
        let slot = self.subscriptions[..self.sub_count]
            .iter()
            .position(|s| !s.active)
            .unwrap_or(self.sub_count);

        if slot >= MAX_SUBSCRIPTIONS {
            return Err(-12); // ENOMEM
        }

        let len = pattern.len().min(MAX_PATH);
        self.subscriptions[slot].pattern[..len].copy_from_slice(&pattern.as_bytes()[..len]);
        self.subscriptions[slot].pattern_len = len as u8;
        self.subscriptions[slot].channel = channel;
        self.subscriptions[slot].active = true;

        if slot >= self.sub_count {
            self.sub_count = slot + 1;
        }

        Ok(())
    }

    /// Remove a subscription
    fn unsubscribe(&mut self, pattern: &str, channel: u32) {
        for i in 0..self.sub_count {
            if self.subscriptions[i].active
                && self.subscriptions[i].channel == channel
                && self.subscriptions[i].pattern_str() == pattern
            {
                self.subscriptions[i].active = false;
                return;
            }
        }
    }

    /// Notify subscribers about a change
    fn notify_subscribers(&self, path: &str, event_type: EventType) {
        for i in 0..self.sub_count {
            if !self.subscriptions[i].active {
                continue;
            }

            if glob_match(self.subscriptions[i].pattern_str(), path) {
                // Find the node to send
                let node = self.nodes[..self.node_count]
                    .iter()
                    .find(|n| n.path_str() == path)
                    .cloned()
                    .unwrap_or_else(|| {
                        // For Removed events, node may be gone - create minimal node
                        Node::new(path)
                    });

                let response = DevdResponse::Event { event_type, node };
                let mut buf = [0u8; 576];
                if let Ok(len) = response.serialize(&mut buf) {
                    let _ = syscall::send(self.subscriptions[i].channel, &buf[..len]);
                }
            }
        }
    }

    /// Get node by path
    fn get(&self, path: &str) -> Option<&Node> {
        self.nodes[..self.node_count]
            .iter()
            .find(|n| n.path_str() == path)
    }

    /// Format device tree for hw:tree command
    fn format_tree(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;
        pos += write_str(&mut buf[pos..], "# Device Tree\r\n");
        pos += write_str(&mut buf[pos..], "# path {properties...}\r\n");

        for i in 0..self.node_count {
            let node = &self.nodes[i];
            pos += write_str(&mut buf[pos..], node.path_str());
            pos += write_str(&mut buf[pos..], " {");

            let mut first = true;
            for prop in node.properties.iter() {
                if !first {
                    pos += write_str(&mut buf[pos..], ", ");
                }
                first = false;
                pos += write_str(&mut buf[pos..], prop.key_str());
                pos += write_str(&mut buf[pos..], ": \"");
                pos += write_str(&mut buf[pos..], prop.value_str());
                pos += write_str(&mut buf[pos..], "\"");
            }

            pos += write_str(&mut buf[pos..], "}\r\n");
        }

        pos
    }

    /// Append device tree nodes in hw list format (for combining with DeviceManager output)
    /// Format: path,bus,class,vendor:device,state
    fn append_to_list(&self, buf: &mut [u8], start_pos: usize) -> usize {
        let mut pos = start_pos;

        for i in 0..self.node_count {
            let node = &self.nodes[i];

            // Path
            pos += write_str(&mut buf[pos..], node.path_str());
            pos += write_str(&mut buf[pos..], ",");

            // Bus (from properties or "unknown")
            let bus = node.properties.get("bus").unwrap_or("unknown");
            pos += write_str(&mut buf[pos..], bus);
            pos += write_str(&mut buf[pos..], ",");

            // Class (from properties or "other")
            let class = node.properties.get("class").unwrap_or("other");
            pos += write_str(&mut buf[pos..], class);
            pos += write_str(&mut buf[pos..], ",");

            // Vendor:Device
            let vendor = node.properties.get("vendor").unwrap_or("0000");
            let device = node.properties.get("device").unwrap_or("0000");
            pos += write_str(&mut buf[pos..], vendor);
            pos += write_str(&mut buf[pos..], ":");
            pos += write_str(&mut buf[pos..], device);
            pos += write_str(&mut buf[pos..], ",");

            // State
            let state = node.properties.get("state").unwrap_or("unknown");
            pos += write_str(&mut buf[pos..], state);
            pos += write_str(&mut buf[pos..], "\r\n");
        }

        pos
    }
}

/// Simple glob pattern matching
/// Supports: * (any chars), ? (single char), ** (any path segments)
fn glob_match(pattern: &str, path: &str) -> bool {
    glob_match_impl(pattern.as_bytes(), path.as_bytes())
}

fn glob_match_impl(pattern: &[u8], text: &[u8]) -> bool {
    let mut pi = 0;
    let mut ti = 0;
    let mut star_pi = usize::MAX;
    let mut star_ti = 0;

    while ti < text.len() {
        if pi < pattern.len() {
            // Handle ** (match any path segments)
            if pi + 1 < pattern.len() && pattern[pi] == b'*' && pattern[pi + 1] == b'*' {
                // ** can match anything including '/'
                star_pi = pi;
                star_ti = ti;
                pi += 2;
                // Skip optional trailing /
                if pi < pattern.len() && pattern[pi] == b'/' {
                    pi += 1;
                }
                continue;
            }

            // Handle * (match any chars except /)
            if pattern[pi] == b'*' {
                star_pi = pi;
                star_ti = ti;
                pi += 1;
                continue;
            }

            // Handle ? (match single char except /)
            if pattern[pi] == b'?' && text[ti] != b'/' {
                pi += 1;
                ti += 1;
                continue;
            }

            // Exact match
            if pattern[pi] == text[ti] {
                pi += 1;
                ti += 1;
                continue;
            }
        }

        // No match - try backtracking to last * or **
        if star_pi != usize::MAX {
            // For *, don't match /
            if pattern[star_pi] == b'*' && star_pi + 1 < pattern.len() && pattern[star_pi + 1] == b'*' {
                // ** can match anything
                star_ti += 1;
                ti = star_ti;
                pi = star_pi + 2;
                if pi < pattern.len() && pattern[pi] == b'/' {
                    pi += 1;
                }
            } else {
                // * cannot match /
                if text[star_ti] == b'/' {
                    return false;
                }
                star_ti += 1;
                ti = star_ti;
                pi = star_pi + 1;
            }
            continue;
        }

        return false;
    }

    // Skip trailing *s
    while pi < pattern.len() && pattern[pi] == b'*' {
        pi += 1;
    }

    pi == pattern.len()
}

// =============================================================================
// Device Manager
// =============================================================================

struct DeviceManager {
    buses: [BusConnection; MAX_BUSES],
    bus_count: usize,
    devices: [Device; MAX_DEVICES],
    device_count: usize,
    next_device_id: u16,
    drivers: [DriverEntry; MAX_DRIVERS],
    driver_count: usize,
}

impl DeviceManager {
    const fn new() -> Self {
        const EMPTY_BUS: BusConnection = BusConnection::empty();
        const EMPTY_DEV: Device = Device::empty();
        const EMPTY_DRV: DriverEntry = DriverEntry::empty();
        Self {
            buses: [EMPTY_BUS; MAX_BUSES],
            bus_count: 0,
            devices: [EMPTY_DEV; MAX_DEVICES],
            device_count: 0,
            next_device_id: 1,
            drivers: [EMPTY_DRV; MAX_DRIVERS],
            driver_count: 0,
        }
    }

    /// Connect to a kernel bus control port and register the bus as a device
    fn connect_bus(&mut self, bus_type: BusType, index: u8) -> bool {
        if self.bus_count >= MAX_BUSES {
            println!("[devd] Bus table full!");
            return false;
        }

        // Build port name
        let name: &[u8] = match bus_type {
            BusType::Platform => b"/kernel/bus/platform",
            BusType::PCIe => match index {
                0 => b"/kernel/bus/pcie0",
                1 => b"/kernel/bus/pcie1",
                2 => b"/kernel/bus/pcie2",
                3 => b"/kernel/bus/pcie3",
                _ => return false,
            },
            BusType::Usb => match index {
                0 => b"/kernel/bus/usb0",
                1 => b"/kernel/bus/usb1",
                _ => return false,
            },
        };

        let channel = syscall::port_connect(name);
        if channel < 0 {
            println!("[devd] Failed to connect to {:?}{}: {}",
                     bus_type, index, channel);
            return false;
        }

        let slot = self.bus_count;
        self.buses[slot].bus_type = bus_type;
        self.buses[slot].bus_index = index;
        self.buses[slot].channel = channel as u32;
        self.buses[slot].connected = true;
        // Set controller base address
        self.buses[slot].base_addr = match bus_type {
            BusType::Platform => 0,  // No MMIO base for pseudo-bus
            BusType::PCIe => PCIE_MAC_BASES.get(index as usize).copied().unwrap_or(0),
            BusType::Usb => USB_MAC_BASES.get(index as usize).copied().unwrap_or(0),
        };
        self.bus_count += 1;

        println!("[devd] Connected to {:?}{} @ 0x{:08x} (channel {})",
                 bus_type, index, self.buses[slot].base_addr, channel);

        // Wait for StateSnapshot from kernel
        let mut buf = [0u8; 64];
        let len = syscall::receive(channel as u32, &mut buf);
        let kernel_state = if len > 0 && buf[0] == MSG_STATE_SNAPSHOT {
            match buf[3] {
                0 => KernelBusState::Safe,
                1 => KernelBusState::Claimed,
                2 => KernelBusState::Resetting,
                _ => KernelBusState::Safe,
            }
        } else {
            KernelBusState::Safe
        };
        self.buses[slot].kernel_state = kernel_state;
        println!("[devd]   Kernel state: {:?}", kernel_state);

        // Note: Buses are NOT added as devices - they're shown separately in hw list
        // from the buses array. Only enumerated devices go in the devices array.

        true
    }

    /// Add a device
    /// If device.state is already set (not Discovered), preserve it
    fn add_device(&mut self, mut device: Device) -> Option<u16> {
        if self.device_count >= MAX_DEVICES {
            println!("[devd] Device table full!");
            return None;
        }

        device.id = self.next_device_id;
        self.next_device_id += 1;
        // Only set to Discovered if not already set to something else
        // (allows platform devices like uart0 to be pre-bound)

        let slot = self.device_count;
        self.devices[slot] = device;
        self.device_count += 1;

        Some(self.devices[slot].id)
    }

    /// Find a device by ID
    fn find_device(&mut self, id: u16) -> Option<&mut Device> {
        self.devices[..self.device_count].iter_mut().find(|d| d.id == id)
    }

    /// Request bus reset from kernel for a device's bus
    fn request_bus_reset(&mut self, device_id: u16) {
        // Find the device to get its bus type
        let bus_info = self.devices[..self.device_count]
            .iter()
            .find(|d| d.id == device_id)
            .map(|d| (d.bus_type, d.bus_index));

        if let Some((bus_type, bus_index)) = bus_info {
            // Find the bus connection
            if let Some(bus) = self.buses[..self.bus_count]
                .iter()
                .find(|b| b.bus_type == bus_type && b.bus_index == bus_index)
            {
                println!("[devd] Requesting reset for {:?}{}", bus_type, bus_index);

                // Send reset request to kernel bus controller
                let msg = [MSG_REQUEST_RESET];
                let result = syscall::send(bus.channel, &msg);
                if result < 0 {
                    println!("[devd] Failed to send reset request: {}", result);
                }
            }
        }
    }

    /// Register driver PID with kernel for a device's bus
    ///
    /// Tells the kernel which process is the actual driver for this bus.
    /// When this PID exits, kernel will auto-reset the bus and notify us.
    fn set_driver_pid(&mut self, device_id: u16, driver_pid: u32) {
        // Find the device to get its bus type
        let bus_info = self.devices[..self.device_count]
            .iter()
            .find(|d| d.id == device_id)
            .map(|d| (d.bus_type, d.bus_index));

        if let Some((bus_type, bus_index)) = bus_info {
            // Find the bus connection
            if let Some(bus) = self.buses[..self.bus_count]
                .iter()
                .find(|b| b.bus_type == bus_type && b.bus_index == bus_index)
            {
                println!("[devd] Registering driver PID {} for {:?}{}",
                         driver_pid, bus_type, bus_index);

                // Send SetDriver message to kernel bus controller
                let pid_bytes = driver_pid.to_le_bytes();
                let msg = [MSG_SET_DRIVER, pid_bytes[0], pid_bytes[1], pid_bytes[2], pid_bytes[3]];
                let result = syscall::send(bus.channel, &msg);
                if result < 0 {
                    println!("[devd] Failed to register driver PID: {}", result);
                }
            }
        }
    }

    /// Spawn a driver for a device
    fn spawn_driver(&mut self, binary: &'static str, device_id: u16) -> Option<u32> {
        let slot = self.drivers[..self.driver_count]
            .iter()
            .position(|d| !d.is_active())
            .unwrap_or(self.driver_count);

        if slot >= MAX_DRIVERS {
            println!("[devd] Driver table full!");
            return None;
        }

        let pid = syscall::exec(binary);
        if pid < 0 {
            println!("[devd] Failed to spawn {}: {}", binary, pid);
            return None;
        }

        let pid = pid as u32;

        // Update driver entry
        let entry = &mut self.drivers[slot];
        entry.binary = binary;
        entry.pid = pid;
        entry.state = DriverState::Running;
        entry.restart_count = 0;
        entry.device_id = Some(device_id);
        entry.restart_at = 0;

        if slot >= self.driver_count {
            self.driver_count = slot + 1;
        }

        // Update device state
        if let Some(device) = self.find_device(device_id) {
            device.driver_id = Some(slot);

            // Platform devices don't need binding handshake - mark as Bound immediately
            // Other buses use Binding state with timeout for driver to confirm
            if device.bus_type == BusType::Platform {
                device.transition(DeviceState::Bound);
            } else {
                let deadline = syscall::gettime() as u64 + BIND_TIMEOUT_NS;
                device.transition(DeviceState::Binding { deadline });
            }
            println!("[devd] Spawned {} (PID {}) for device {}",
                     binary, pid, device.path_str());
        }

        // Register driver PID with kernel so it knows to auto-reset
        // the bus when this driver exits
        self.set_driver_pid(device_id, pid);

        Some(pid)
    }

    /// Handle driver exit
    fn handle_driver_exit(&mut self, pid: u32, exit_code: i32) {
        // Find the driver index first
        let driver_idx = match (0..self.driver_count).find(|&i| self.drivers[i].pid == pid) {
            Some(idx) => idx,
            None => {
                println!("[devd] Unknown child PID {} exited", pid);
                return;
            }
        };

        let binary = self.drivers[driver_idx].binary;
        let device_id = self.drivers[driver_idx].device_id;

        if exit_code == 0 {
            println!("[devd] {} (PID {}) exited normally", binary, pid);
            self.drivers[driver_idx].state = DriverState::Stopped;
            self.drivers[driver_idx].pid = 0;

            // Clean unbind - device goes back to DISCOVERED
            if let Some(dev_id) = device_id {
                if let Some(device) = self.find_device(dev_id) {
                    device.transition(DeviceState::Discovered);
                    device.driver_id = None;
                }
            }
        } else {
            println!("[devd] {} (PID {}) crashed (code {})", binary, pid, exit_code);
            self.drivers[driver_idx].state = DriverState::Crashed;
            self.drivers[driver_idx].restart_count += 1;
            self.drivers[driver_idx].pid = 0;

            // Device goes to FAULTED and escalate
            if let Some(dev_id) = device_id {
                let now_ns = syscall::gettime() as u64;
                let now_ms = now_ns / 1_000_000;  // Convert to ms for fault tracker

                // Note: Kernel auto-resets bus when driver_pid exits
                // We will receive a StateSnapshot on our bus channel when it's Safe

                // Find device and escalate
                let (level, wait_ms, is_dead) = if let Some(dev_slot) = self.devices[..self.device_count]
                    .iter()
                    .position(|d| d.id == dev_id)
                {
                    self.devices[dev_slot].transition(DeviceState::Faulted);
                    let (level, wait_ms) = self.devices[dev_slot].fault_tracker.escalate(now_ms);
                    let is_dead = level == EscalationLevel::Dead;

                    if is_dead {
                        self.devices[dev_slot].transition(DeviceState::Dead);
                        println!("[devd] {} marked as DEAD", self.devices[dev_slot].path_str());
                    }

                    (level, wait_ms, is_dead)
                } else {
                    return;
                };

                if is_dead {
                    self.drivers[driver_idx].state = DriverState::Failed;
                } else {
                    // Schedule restart via kernel timer
                    let wait_ns = wait_ms * 1_000_000;
                    self.drivers[driver_idx].restart_at = now_ns + wait_ns;
                    syscall::timer_set(wait_ns);
                    println!("[devd] {} will retry in {}ms (level {:?})", binary, wait_ms, level);
                }
            }
        }
    }

    /// Process pending restarts and binding timeouts
    fn process_timers(&mut self) {
        let now_ns = syscall::gettime() as u64;
        let now_ms = now_ns / 1_000_000;

        // Check binding timeouts
        for i in 0..self.device_count {
            if let DeviceState::Binding { deadline } = self.devices[i].state {
                if now_ns >= deadline {
                    println!("[devd] Device {} bind timeout", self.devices[i].path_str());
                    self.devices[i].transition(DeviceState::Faulted);

                    // Escalate (uses milliseconds)
                    let (level, _wait_ms) = self.devices[i].fault_tracker.escalate(now_ms);
                    if level == EscalationLevel::Dead {
                        self.devices[i].transition(DeviceState::Dead);
                    }
                }
            }
        }

        // Check driver restarts
        for i in 0..self.driver_count {
            let entry = &mut self.drivers[i];
            if entry.restart_at > 0 && now_ns >= entry.restart_at {
                let binary = entry.binary;
                let device_id = entry.device_id;
                entry.restart_at = 0;

                if entry.restart_count > MAX_RESTARTS {
                    entry.state = DriverState::Failed;
                    println!("[devd] {} exceeded max restarts, marking failed", binary);

                    if let Some(dev_id) = device_id {
                        if let Some(device) = self.find_device(dev_id) {
                            device.transition(DeviceState::Dead);
                        }
                    }
                } else {
                    // Respawn
                    let new_pid = syscall::exec(binary);
                    if new_pid > 0 {
                        entry.pid = new_pid as u32;
                        entry.state = DriverState::Running;
                        println!("[devd] {} restarted as PID {}", binary, new_pid);

                        if let Some(dev_id) = device_id {
                            if let Some(device) = self.find_device(dev_id) {
                                let deadline = now_ns + BIND_TIMEOUT_NS;
                                device.transition(DeviceState::Binding { deadline });
                            }
                            // Register new driver PID with kernel
                            self.set_driver_pid(dev_id, new_pid as u32);
                        }
                    } else {
                        println!("[devd] Failed to restart {}", binary);
                        entry.state = DriverState::Failed;
                    }
                }
            }
        }
    }

    /// Mark driver bind as complete
    pub fn driver_bind_complete(&mut self, pid: u32) {
        for i in 0..self.driver_count {
            if self.drivers[i].pid == pid {
                if let Some(dev_id) = self.drivers[i].device_id {
                    if let Some(device) = self.find_device(dev_id) {
                        if matches!(device.state, DeviceState::Binding { .. }) {
                            device.transition(DeviceState::Bound);
                            device.fault_tracker.record_success();
                            println!("[devd] Device {} bound successfully", device.path_str());
                        }
                    }
                }
                return;
            }
        }
    }

    // =========================================================================
    // hw: Scheme Handler
    // =========================================================================

    /// Format device list for hw:list
    pub fn format_device_list(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;

        // Header (use \r\n for serial terminal)
        pos += write_str(&mut buf[pos..], "# Hardware Devices\r\n");
        pos += write_str(&mut buf[pos..], "# path,bus,class,vendor:device,state\r\n");

        // List all buses first (from buses array, not as matchable devices)
        for i in 0..self.bus_count {
            let bus = &self.buses[i];
            if !bus.connected {
                continue;
            }
            // Format: /bus/pcie0,pcie,bus,-,state or /bus/platform,platform,bus,-,state
            pos += write_str(&mut buf[pos..], "/bus/");
            let bus_name = match bus.bus_type {
                BusType::PCIe => "pcie",
                BusType::Usb => "usb",
                BusType::Platform => "platform",
            };
            pos += write_str(&mut buf[pos..], bus_name);
            // Only add index for numbered buses (not platform)
            if bus.bus_type != BusType::Platform {
                pos += write_num(&mut buf[pos..], bus.bus_index as usize);
            }
            pos += write_str(&mut buf[pos..], ",");
            pos += write_str(&mut buf[pos..], bus_name);
            pos += write_str(&mut buf[pos..], ",bus,");
            // Show controller address (or - for platform pseudo-bus)
            if bus.base_addr != 0 {
                pos += write_hex16(&mut buf[pos..], (bus.base_addr >> 16) as u16);
                pos += write_str(&mut buf[pos..], ":");
                pos += write_hex16(&mut buf[pos..], (bus.base_addr & 0xFFFF) as u16);
            } else {
                pos += write_str(&mut buf[pos..], "-");
            }
            pos += write_str(&mut buf[pos..], ",");
            // Show kernel state for bus
            pos += write_str(&mut buf[pos..], match bus.kernel_state {
                KernelBusState::Safe => "safe",
                KernelBusState::Claimed => "claimed",
                KernelBusState::Resetting => "resetting",
            });
            pos += write_str(&mut buf[pos..], "\r\n");
        }

        // List all devices
        for i in 0..self.device_count {
            let dev = &self.devices[i];
            // Format: path,bus_type,class,vendor:device,state
            pos += write_str(&mut buf[pos..], dev.path_str());
            pos += write_str(&mut buf[pos..], ",");
            pos += write_str(&mut buf[pos..], match dev.bus_type {
                BusType::PCIe => "pcie",
                BusType::Usb => "usb",
                BusType::Platform => "platform",
            });
            pos += write_str(&mut buf[pos..], ",");
            pos += write_str(&mut buf[pos..], match dev.class {
                DeviceClass::UsbController => "usb",
                DeviceClass::Serial => "serial",
                DeviceClass::Storage => "storage",
                DeviceClass::Network => "network",
                DeviceClass::Hid => "hid",
                DeviceClass::Other => "other",
            });
            pos += write_str(&mut buf[pos..], ",");
            pos += write_hex16(&mut buf[pos..], dev.vendor_id);
            pos += write_str(&mut buf[pos..], ":");
            pos += write_hex16(&mut buf[pos..], dev.device_id);
            pos += write_str(&mut buf[pos..], ",");
            pos += write_str(&mut buf[pos..], dev.state.as_str());
            pos += write_str(&mut buf[pos..], "\r\n");
        }

        pos
    }

    /// Format bus list for hw:bus/list
    pub fn format_bus_list(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;

        pos += write_str(&mut buf[pos..], "# Bus Controllers\r\n");
        pos += write_str(&mut buf[pos..], "# name,type,state,connected\r\n");

        for i in 0..self.bus_count {
            let bus = &self.buses[i];
            // Format: name,type,state,connected
            let bus_name = match bus.bus_type {
                BusType::PCIe => "pcie",
                BusType::Usb => "usb",
                BusType::Platform => "platform",
            };
            pos += write_str(&mut buf[pos..], bus_name);
            if bus.bus_type != BusType::Platform {
                pos += write_num(&mut buf[pos..], bus.bus_index as usize);
            }
            pos += write_str(&mut buf[pos..], ",");
            pos += write_str(&mut buf[pos..], bus_name);
            pos += write_str(&mut buf[pos..], ",");
            pos += write_str(&mut buf[pos..], match bus.kernel_state {
                KernelBusState::Safe => "safe",
                KernelBusState::Claimed => "claimed",
                KernelBusState::Resetting => "resetting",
            });
            pos += write_str(&mut buf[pos..], ",");
            pos += write_str(&mut buf[pos..], if bus.connected { "true" } else { "false" });
            pos += write_str(&mut buf[pos..], "\r\n");
        }

        pos
    }

    /// Format device info for hw:device/<path>
    pub fn format_device_info(&self, path: &str, buf: &mut [u8]) -> usize {
        // Find device by path
        for i in 0..self.device_count {
            let dev = &self.devices[i];
            if dev.path_str() == path {
                let mut pos = 0;
                pos += write_str(&mut buf[pos..], "path=");
                pos += write_str(&mut buf[pos..], dev.path_str());
                pos += write_str(&mut buf[pos..], "\r\nbus=");
                let bus_name = match dev.bus_type {
                    BusType::PCIe => "pcie",
                    BusType::Usb => "usb",
                    BusType::Platform => "platform",
                };
                pos += write_str(&mut buf[pos..], bus_name);
                if dev.bus_type != BusType::Platform {
                    pos += write_num(&mut buf[pos..], dev.bus_index as usize);
                }
                pos += write_str(&mut buf[pos..], "\r\nclass=");
                pos += write_str(&mut buf[pos..], match dev.class {
                    DeviceClass::UsbController => "usb-controller",
                    DeviceClass::Serial => "serial",
                    DeviceClass::Storage => "storage",
                    DeviceClass::Network => "network",
                    DeviceClass::Hid => "hid",
                    DeviceClass::Other => "other",
                });
                pos += write_str(&mut buf[pos..], "\r\nvendor=");
                pos += write_hex16(&mut buf[pos..], dev.vendor_id);
                pos += write_str(&mut buf[pos..], "\r\ndevice=");
                pos += write_hex16(&mut buf[pos..], dev.device_id);
                pos += write_str(&mut buf[pos..], "\r\nstate=");
                pos += write_str(&mut buf[pos..], dev.state.as_str());
                if let Some(drv_idx) = dev.driver_id {
                    if drv_idx < self.driver_count {
                        pos += write_str(&mut buf[pos..], "\r\ndriver=");
                        pos += write_str(&mut buf[pos..], self.drivers[drv_idx].binary);
                        pos += write_str(&mut buf[pos..], "\r\ndriver_pid=");
                        pos += write_num(&mut buf[pos..], self.drivers[drv_idx].pid as usize);
                    }
                }
                pos += write_str(&mut buf[pos..], "\r\n");
                return pos;
            }
        }

        // Not found
        write_str(buf, "error=not_found\r\n")
    }

    /// Handle hw: scheme request
    pub fn handle_hw_request(&self, path: &str, buf: &mut [u8]) -> usize {
        match path {
            "list" | "" => self.format_device_list(buf),
            "bus" | "bus/list" => self.format_bus_list(buf),
            _ => {
                // Check for device/<path> format
                if let Some(dev_path) = path.strip_prefix("device/") {
                    self.format_device_info(dev_path, buf)
                } else if let Some(dev_path) = path.strip_prefix("/") {
                    // Allow hw:/platform/uart0 format
                    self.format_device_info(dev_path, buf)
                } else {
                    write_str(buf, "error=invalid_path\n")
                }
            }
        }
    }
}

// Helper functions for formatting
fn write_str(buf: &mut [u8], s: &str) -> usize {
    let bytes = s.as_bytes();
    let len = bytes.len().min(buf.len());
    buf[..len].copy_from_slice(&bytes[..len]);
    len
}

fn write_num(buf: &mut [u8], n: usize) -> usize {
    if n == 0 {
        if !buf.is_empty() {
            buf[0] = b'0';
            return 1;
        }
        return 0;
    }

    let mut digits = [0u8; 10];
    let mut i = 0;
    let mut val = n;
    while val > 0 && i < 10 {
        digits[i] = b'0' + (val % 10) as u8;
        val /= 10;
        i += 1;
    }

    let len = i.min(buf.len());
    for j in 0..len {
        buf[j] = digits[i - 1 - j];
    }
    len
}

fn write_hex16(buf: &mut [u8], val: u16) -> usize {
    const HEX: &[u8] = b"0123456789abcdef";
    if buf.len() < 4 {
        return 0;
    }
    buf[0] = HEX[(val >> 12) as usize & 0xF];
    buf[1] = HEX[(val >> 8) as usize & 0xF];
    buf[2] = HEX[(val >> 4) as usize & 0xF];
    buf[3] = HEX[val as usize & 0xF];
    4
}

// =============================================================================
// Bus Scanning
// =============================================================================

// MT7988A platform device addresses
const UART0_BASE: u32 = 0x1100_2000;  // UART0 MMIO base
const I2C0_BASE: u32 = 0x1100_7000;   // I2C0 (for GPIO expander)

fn scan_platform(mgr: &mut DeviceManager) {
    println!("[devd] Scanning platform bus...");

    // UART0 - always present on platform bus
    let mut uart = Device::empty();
    uart.set_path("/platform/uart0");
    uart.bus_type = BusType::Platform;
    uart.bus_index = 0;
    uart.class = DeviceClass::Serial;
    // Use MMIO base as "vendor:device" identifier
    uart.vendor_id = (UART0_BASE >> 16) as u16;
    uart.device_id = (UART0_BASE & 0xFFFF) as u16;
    // Leave as Discovered - match_and_spawn will match and spawn shell

    if let Some(id) = mgr.add_device(uart) {
        println!("[devd]   Found: /platform/uart0 @ 0x{:08x} (id={})", UART0_BASE, id);
    }

    // GPIO - I2C-attached PCA9555 via I2C0
    let mut gpio = Device::empty();
    gpio.set_path("/platform/gpio");
    gpio.bus_type = BusType::Platform;
    gpio.bus_index = 0;
    gpio.class = DeviceClass::Other;
    // I2C address 0x27 on I2C0
    gpio.vendor_id = (I2C0_BASE >> 16) as u16;
    gpio.device_id = 0x0027;  // I2C address

    if let Some(id) = mgr.add_device(gpio) {
        println!("[devd]   Found: /platform/gpio @ I2C0:0x27 (id={})", id);
    }

    // Note: USB controllers (ssusb0, ssusb1) are buses, not devices
    // They're listed under /bus/usb0, /bus/usb1
}

fn scan_pcie(mgr: &mut DeviceManager) {
    println!("[devd] Scanning PCIe bus...");

    let mut devices = [PciDeviceInfo::default(); 16];
    let count = syscall::pci_enumerate(&mut devices);

    if count < 0 {
        println!("[devd]   PCIe enumeration failed: {}", count);
        return;
    }

    let count = count as usize;
    println!("[devd]   Found {} PCIe device(s)", count);

    for i in 0..count {
        let pci = &devices[i];
        let class = pci.class_code >> 8;

        let dev_class = match class {
            0x0C03 => DeviceClass::UsbController,
            0x0200 => DeviceClass::Network,
            0x0100 | 0x0106 => DeviceClass::Storage,
            _ => DeviceClass::Other,
        };

        let bdf = pci.bdf;
        let bus = (bdf >> 8) & 0xFF;
        let dev = (bdf >> 3) & 0x1F;
        let func = bdf & 0x7;

        let mut device = Device::empty();
        let path = match dev_class {
            DeviceClass::UsbController => "/pcie/usb",
            DeviceClass::Network => "/pcie/net",
            DeviceClass::Storage => "/pcie/storage",
            _ => "/pcie/unknown",
        };
        device.set_path(path);
        device.bus_type = BusType::PCIe;
        device.bus_index = 0;
        device.class = dev_class;
        device.vendor_id = pci.vendor_id;
        device.device_id = pci.device_id;
        device.pci_bdf = bdf;

        println!("[devd]   {:04x}:{:02x}:{:02x}.{} - {:04x}:{:04x} ({:?})",
                 (bdf >> 16) & 0xFF, bus, dev, func,
                 pci.vendor_id, pci.device_id, dev_class);

        mgr.add_device(device);
    }
}

// =============================================================================
// Driver Matching
// =============================================================================

fn match_and_spawn(mgr: &mut DeviceManager) {
    println!("[devd] Matching devices to drivers...");

    for i in 0..mgr.device_count {
        let device = &mgr.devices[i];

        // Skip if already has driver or not in DISCOVERED state
        if device.driver_id.is_some() || device.state != DeviceState::Discovered {
            continue;
        }

        // Find matching rule
        let mut driver_to_spawn: Option<&'static str> = None;

        for rule in MATCH_RULES {
            let mut matched = true;

            if let Some(rule_bus) = rule.bus {
                if rule_bus != device.bus_type {
                    matched = false;
                }
            }

            if let Some(rule_class) = rule.class {
                if rule_class != device.class {
                    matched = false;
                }
            }

            if let Some(pci_class) = rule.pci_class {
                if device.bus_type != BusType::PCIe {
                    matched = false;
                } else if pci_class == PCI_CLASS_USB && device.class != DeviceClass::UsbController {
                    matched = false;
                }
            }

            if matched {
                driver_to_spawn = Some(rule.driver);
                break;
            }
        }

        if let Some(driver) = driver_to_spawn {
            let device_id = mgr.devices[i].id;
            mgr.spawn_driver(driver, device_id);
        }
    }
}

// =============================================================================
// DevdProtocol Request Handler
// =============================================================================

fn handle_devd_request(tree: &mut DeviceTree, req: &DevdRequest, client_ch: u32) -> DevdResponse {
    match req {
        DevdRequest::Register { path, path_len, properties } => {
            let path_str = core::str::from_utf8(&path[..*path_len as usize]).unwrap_or("");
            println!("[devd] Register: {}", path_str);
            match tree.register(path_str, properties.clone()) {
                Ok(()) => DevdResponse::Ok,
                Err(e) => DevdResponse::Error(e),
            }
        }

        DevdRequest::Update { path, path_len, properties } => {
            let path_str = core::str::from_utf8(&path[..*path_len as usize]).unwrap_or("");
            println!("[devd] Update: {}", path_str);
            match tree.update(path_str, properties.clone()) {
                Ok(()) => DevdResponse::Ok,
                Err(e) => DevdResponse::Error(e),
            }
        }

        DevdRequest::Query { pattern, pattern_len } => {
            let pattern_str = core::str::from_utf8(&pattern[..*pattern_len as usize]).unwrap_or("");
            println!("[devd] Query: {}", pattern_str);
            let nodes = tree.query(pattern_str);
            DevdResponse::Nodes(nodes)
        }

        DevdRequest::Subscribe { pattern, pattern_len } => {
            let pattern_str = core::str::from_utf8(&pattern[..*pattern_len as usize]).unwrap_or("");
            println!("[devd] Subscribe: {} (ch={})", pattern_str, client_ch);
            match tree.subscribe(pattern_str, client_ch) {
                Ok(()) => DevdResponse::Ok,
                Err(e) => DevdResponse::Error(e),
            }
        }

        DevdRequest::Unsubscribe { pattern, pattern_len } => {
            let pattern_str = core::str::from_utf8(&pattern[..*pattern_len as usize]).unwrap_or("");
            println!("[devd] Unsubscribe: {} (ch={})", pattern_str, client_ch);
            tree.unsubscribe(pattern_str, client_ch);
            DevdResponse::Ok
        }

        DevdRequest::Remove { path, path_len } => {
            let path_str = core::str::from_utf8(&path[..*path_len as usize]).unwrap_or("");
            println!("[devd] Remove: {}", path_str);
            match tree.remove(path_str) {
                Ok(()) => DevdResponse::Ok,
                Err(e) => DevdResponse::Error(e),
            }
        }
    }
}

// =============================================================================
// Main Loop
// =============================================================================

#[unsafe(no_mangle)]
fn main() -> ! {
    println!();
    println!("========================================");
    println!("  devd - Device Supervisor");
    println!("  State Machine Architecture");
    println!("========================================");
    println!();

    let mut mgr = DeviceManager::new();
    let mut tree = DeviceTree::new();

    // Phase 1: Connect to kernel bus control ports
    println!("[devd] Connecting to bus control ports...");

    // Connect to platform bus (uart, gpio, i2c, spi, etc.)
    mgr.connect_bus(BusType::Platform, 0);

    // Connect to USB buses
    mgr.connect_bus(BusType::Usb, 0);
    mgr.connect_bus(BusType::Usb, 1);

    // Connect to PCIe buses (4 ports on BPI-R4)
    mgr.connect_bus(BusType::PCIe, 0);
    mgr.connect_bus(BusType::PCIe, 1);
    mgr.connect_bus(BusType::PCIe, 2);
    mgr.connect_bus(BusType::PCIe, 3);

    // Phase 2: Scan buses for devices
    println!();
    scan_platform(&mut mgr);  // Enumerates uart0, gpio, etc. on platform bus
    scan_pcie(&mut mgr);

    // Phase 3: Match and spawn drivers
    println!();
    match_and_spawn(&mut mgr);

    // Phase 4: Register hw: scheme for device discovery
    println!();
    println!("[devd] Registering hw: scheme...");

    // Create a channel for hw: scheme requests
    // channel_create returns (ch_b << 32) | ch_a
    // When kernel sends to ch_a, message goes to ch_b's queue (peer)
    // So: register ch_a with scheme, receive on ch_b
    let hw_channel_pair = syscall::channel_create();
    let mut hw_channel_valid = false;
    let hw_daemon_channel = (hw_channel_pair & 0xFFFFFFFF) as u32;  // Register this
    let hw_channel = ((hw_channel_pair >> 32) & 0xFFFFFFFF) as u32; // Receive on this
    if hw_channel_pair < 0 {
        println!("[devd] Failed to create hw: channel: {}", hw_channel_pair);
    } else {
        let result = syscall::scheme_register("hw", hw_daemon_channel);
        if result < 0 {
            println!("[devd] Failed to register hw: scheme: {}", result);
        } else {
            println!("[devd] Registered hw: scheme (daemon_ch={}, recv_ch={})",
                     hw_daemon_channel, hw_channel);
            hw_channel_valid = true;
        }
    }

    // Phase 4b: Register "devd" port for protocol requests
    println!("[devd] Registering devd port...");
    let devd_port = syscall::port_register(b"devd");
    let mut devd_port_valid = false;
    if devd_port < 0 {
        println!("[devd] Failed to register devd port: {}", devd_port);
    } else {
        println!("[devd] Registered devd port ({})", devd_port);
        devd_port_valid = true;
    }

    println!();
    println!("[devd] Supervision started ({} buses, {} devices, {} drivers)",
             mgr.bus_count, mgr.device_count, mgr.driver_count);
    println!("[devd] Hardware discovery via hw:list, hw:bus, hw:device/<path>");
    println!("[devd] Protocol queries via devd port");
    println!();

    // Phase 5: Subscribe to events (CRITICAL - failure means system unusable)
    println!("[devd] Subscribing to events...");

    // Subscribe to ChildExit events - CRITICAL for driver supervision
    // If this fails, devd cannot detect crashed drivers and system will hang
    let result = syscall::event_subscribe(event_type::CHILD_EXIT, 0);
    if result != 0 {
        println!("[devd] FATAL: Failed to subscribe to ChildExit events (error {})", result);
        println!("[devd] Cannot supervise drivers - system will hang. Watchdog should reset.");
        panic!("[devd] FATAL: ChildExit subscription failed");
    }
    println!("[devd]   ChildExit: OK");

    // Subscribe to Timer events - for driver restart scheduling
    let result = syscall::event_subscribe(event_type::TIMER, 0);
    if result != 0 {
        println!("[devd] FATAL: Failed to subscribe to Timer events (error {})", result);
        panic!("[devd] FATAL: Timer subscription failed");
    }
    println!("[devd]   Timer: OK");

    // Subscribe to IpcReady for hw: channel - CRITICAL for hw: scheme
    if hw_channel_valid {
        let result = syscall::event_subscribe(event_type::IPC_READY, hw_channel as u64);
        if result != 0 {
            println!("[devd] FATAL: Failed to subscribe to IpcReady for hw: channel (error {})", result);
            panic!("[devd] FATAL: hw: channel subscription failed");
        }
        println!("[devd]   IpcReady(hw:{}): OK", hw_channel);
    }

    // Subscribe to IpcReady for bus control channels - CRITICAL
    for i in 0..mgr.bus_count {
        if mgr.buses[i].connected {
            let ch = mgr.buses[i].channel;
            let result = syscall::event_subscribe(event_type::IPC_READY, ch as u64);
            if result != 0 {
                println!("[devd] FATAL: Failed to subscribe to IpcReady for bus channel {} (error {})", ch, result);
                panic!("[devd] FATAL: Bus channel subscription failed");
            }
            println!("[devd]   IpcReady(bus:{}): OK", ch);
        }
    }

    // Subscribe to IpcReady for devd protocol port
    if devd_port_valid {
        let result = syscall::event_subscribe(event_type::IPC_READY, devd_port as u64);
        if result != 0 {
            println!("[devd] FATAL: Failed to subscribe to IpcReady for devd port (error {})", result);
            panic!("[devd] FATAL: devd port subscription failed");
        }
        println!("[devd]   IpcReady(devd:{}): OK", devd_port);
    }

    println!();

    // Phase 6: Main supervision loop (event-driven)
    println!("[devd] === ENTERING EVENT LOOP ===");
    let mut loop_count = 0u32;
    let mut event = Event::empty();

    loop {
        // Wait for an event (blocking - will wake on Timer, ChildExit, IpcReady)
        let result = syscall::event_wait(&mut event, event_flags::BLOCKING);

        if result == -11 {
            // EAGAIN - kernel marked us blocked, yield and retry
            syscall::yield_now();
            continue;
        }

        if result < 0 {
            println!("[devd] event_wait error: {}", result);
            syscall::yield_now();
            continue;
        }

        if result == 0 {
            // No event (shouldn't happen with blocking wait)
            syscall::yield_now();
            continue;
        }

        // Got an event!
        loop_count = loop_count.wrapping_add(1);

        match event.event_type {
            et if et == event_type::CHILD_EXIT => {
                // Child process exited
                let child_pid = event.data as u32;
                let exit_code = event.flags as i32;
                println!("[devd] Event: ChildExit pid={} code={}", child_pid, exit_code);
                mgr.handle_driver_exit(child_pid, exit_code);
            }

            et if et == event_type::IPC_READY => {
                // IPC message available on a channel
                let channel = event.data as u32;
                println!("[devd] IpcReady: channel={} (hw={}, devd={})",
                         channel, hw_channel, devd_port);

                if hw_channel_valid && channel == hw_channel {
                    // hw: scheme request
                    let mut buf = [0u8; 256];
                    let len = syscall::receive(hw_channel, &mut buf);
                    if len >= 8 {
                        let server_ch = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                        let _flags = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
                        let path = core::str::from_utf8(&buf[8..len as usize]).unwrap_or("");

                        println!("[devd] hw: request path='{}' server_ch={}", path, server_ch);

                        // Handle hw: scheme requests
                        let mut response = [0u8; 2048];
                        let data_len = if path == "tree" {
                            // Show device tree only
                            tree.format_tree(&mut response)
                        } else if path == "list" || path.is_empty() {
                            // Combine DeviceManager devices and DeviceTree nodes
                            let mgr_len = mgr.format_device_list(&mut response);
                            tree.append_to_list(&mut response, mgr_len)
                        } else {
                            mgr.handle_hw_request(path, &mut response)
                        };

                        let result = syscall::send(server_ch, &response[..data_len]);
                        if result < 0 {
                            println!("[devd] hw: send failed: {}", result);
                        }
                    }
                } else if devd_port_valid && channel == devd_port as u32 {
                    // DevdProtocol request - accept connection
                    let client_ch = syscall::port_accept(devd_port as u32);
                    if client_ch > 0 {
                        // Receive the request
                        let mut buf = [0u8; 576];
                        let len = syscall::receive(client_ch as u32, &mut buf);
                        if len > 0 {
                            // Deserialize and handle the request
                            let response = match DevdRequest::deserialize(&buf[..len as usize]) {
                                Ok((req, _)) => handle_devd_request(&mut tree, &req, client_ch as u32),
                                Err(_) => DevdResponse::Error(-22), // EINVAL
                            };

                            // Serialize and send response
                            let mut resp_buf = [0u8; 576];
                            if let Ok(resp_len) = response.serialize(&mut resp_buf) {
                                let _ = syscall::send(client_ch as u32, &resp_buf[..resp_len]);
                            }
                        }
                        // Close the connection channel
                        syscall::channel_close(client_ch as u32);
                    }
                } else {
                    // Check if it's a bus control channel
                    for i in 0..mgr.bus_count {
                        if mgr.buses[i].connected && mgr.buses[i].channel == channel {
                            let mut buf = [0u8; 64];
                            let len = syscall::receive(channel, &mut buf);
                            if len > 0 && buf[0] == MSG_STATE_SNAPSHOT {
                                let new_state = match buf[3] {
                                    0 => KernelBusState::Safe,
                                    1 => KernelBusState::Claimed,
                                    2 => KernelBusState::Resetting,
                                    _ => mgr.buses[i].kernel_state,
                                };
                                if new_state != mgr.buses[i].kernel_state {
                                    println!("[devd] {:?}{} state: {:?} -> {:?}",
                                             mgr.buses[i].bus_type, mgr.buses[i].bus_index,
                                             mgr.buses[i].kernel_state, new_state);
                                    mgr.buses[i].kernel_state = new_state;
                                }
                            }
                            break;
                        }
                    }
                }
            }

            et if et == event_type::TIMER => {
                // Timer expired - process driver restarts and binding timeouts
                mgr.process_timers();
            }

            _ => {
                println!("[devd] Unknown event type: {}", event.event_type);
            }
        }
    }
}
