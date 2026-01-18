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

use userlib::{uinfo, uwarn, uerror, EventFilter, kevent_subscribe, kevent_unsubscribe, kevent_wait};  // Structured logging + kevent
use userlib::syscall::{self, Event, event_type, CpuStatsEntry, caps};
use userlib::ipc::protocols::devd::{
    DevdRequest, DevdResponse, Node, NodeList, PropertyList, EventType,
    MAX_PATH, MAX_NODES,
};
use userlib::ipc::protocols::pcie::{PcieRequest, PcieResponse};
use userlib::ipc::Message;
use userlib::ansi::{Style, Box as BoxChar, Symbol};

// =============================================================================
// Constants
// =============================================================================

const MAX_BUSES: usize = 8;
const MAX_DRIVERS: usize = 16;
const MAX_DEVICES: usize = 32;
const MAX_CLIENTS: usize = 8;
const MAX_RESTARTS: u32 = 3;
const RESTART_DELAY_MS: u64 = 1000;
const BIND_TIMEOUT_NS: u64 = 30_000_000_000;  // 30s in nanoseconds - USB enumeration can be slow
const QUIET_PERIOD_MS: u64 = 60_000;

// Error recovery constants
const RESET_TIMEOUT_MS: u64 = 5000;      // Timeout waiting for reset completion
const IPC_TIMEOUT_MS: u64 = 1000;        // Timeout for IPC requests to pcied
const PCIED_CONNECT_RETRIES: u32 = 3;    // Retries for connecting to pcied
const MAX_RECOVERY_HISTORY: usize = 8;   // Track last N recovery attempts per device

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
///
/// State machine:
///   Safe → Active (when bus driver starts managing)
///   Active → Resetting (when bus is being reset)
///   Resetting → Active (after reset completes)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum KernelBusState {
    /// No driver assigned to this bus
    Safe = 0,
    /// Bus driver (pcied, usbd) is actively managing
    Active = 1,
    /// Bus is being reset
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

// Bus base addresses now come from kernel (DTB) via bus_list syscall

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

    /// Reset tracking state (after device truly recovers)
    pub fn reset(&mut self) {
        self.level = EscalationLevel::DeviceReset;
        self.attempts = 0;
        self.backoff_ms = 5000;
        self.consecutive_success = 0;
    }
}

// =============================================================================
// Error Recovery Tracking
// =============================================================================

/// Source of a device error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorSource {
    /// Driver crashed with non-zero exit code
    DriverCrash(i32),
    /// Driver failed to bind within timeout
    BindTimeout,
    /// IPC communication failure
    IpcFailure,
    /// Reset operation failed
    ResetFailed,
    /// Hardware error detected
    HardwareError,
}

/// Record of a single recovery attempt
#[derive(Clone, Copy)]
pub struct RecoveryRecord {
    /// When the error occurred
    pub timestamp_ms: u64,
    /// What caused the error
    pub source: ErrorSource,
    /// Escalation level at time of error
    pub level: EscalationLevel,
    /// Did recovery succeed?
    pub recovered: bool,
    /// Time to recovery in ms (if recovered)
    pub recovery_time_ms: u64,
}

impl RecoveryRecord {
    pub const fn empty() -> Self {
        Self {
            timestamp_ms: 0,
            source: ErrorSource::DriverCrash(0),
            level: EscalationLevel::DeviceReset,
            recovered: false,
            recovery_time_ms: 0,
        }
    }
}

/// Recovery history for a device
#[derive(Clone, Copy)]
pub struct RecoveryHistory {
    /// Circular buffer of recent recovery attempts
    pub records: [RecoveryRecord; MAX_RECOVERY_HISTORY],
    /// Next write index
    pub write_idx: usize,
    /// Total errors ever recorded
    pub total_errors: u32,
    /// Total successful recoveries
    pub total_recoveries: u32,
    /// Current pending recovery (if any)
    pub pending_recovery_start: u64,
}

impl RecoveryHistory {
    pub const fn new() -> Self {
        Self {
            records: [RecoveryRecord::empty(); MAX_RECOVERY_HISTORY],
            write_idx: 0,
            total_errors: 0,
            total_recoveries: 0,
            pending_recovery_start: 0,
        }
    }

    /// Record an error occurrence
    pub fn record_error(&mut self, timestamp_ms: u64, source: ErrorSource, level: EscalationLevel) {
        self.records[self.write_idx] = RecoveryRecord {
            timestamp_ms,
            source,
            level,
            recovered: false,
            recovery_time_ms: 0,
        };
        self.write_idx = (self.write_idx + 1) % MAX_RECOVERY_HISTORY;
        self.total_errors += 1;
        self.pending_recovery_start = timestamp_ms;
    }

    /// Record successful recovery
    pub fn record_recovery(&mut self, timestamp_ms: u64) {
        self.total_recoveries += 1;
        if self.pending_recovery_start > 0 {
            let recovery_time = timestamp_ms.saturating_sub(self.pending_recovery_start);
            // Find the most recent record and mark it recovered
            let prev_idx = if self.write_idx == 0 {
                MAX_RECOVERY_HISTORY - 1
            } else {
                self.write_idx - 1
            };
            self.records[prev_idx].recovered = true;
            self.records[prev_idx].recovery_time_ms = recovery_time;
            self.pending_recovery_start = 0;
        }
    }

    /// Calculate recovery success rate (percentage)
    pub fn recovery_rate(&self) -> u32 {
        if self.total_errors == 0 {
            100
        } else {
            (self.total_recoveries * 100) / self.total_errors
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
    /// Driver name (set when claimed)
    pub driver_name: [u8; 16],
    pub driver_name_len: u8,
    /// Fault tracker
    pub fault_tracker: FaultTracker,
    /// Recovery history for diagnostics
    pub recovery_history: RecoveryHistory,
    /// PCIe port number (for reset operations)
    pub pci_port: u8,
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
            driver_name: [0; 16],
            driver_name_len: 0,
            fault_tracker: FaultTracker::new(),
            recovery_history: RecoveryHistory::new(),
            pci_port: 0,
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

    pub fn set_driver_name(&mut self, name: &str) {
        let bytes = name.as_bytes();
        let len = bytes.len().min(self.driver_name.len());
        self.driver_name[..len].copy_from_slice(&bytes[..len]);
        self.driver_name_len = len as u8;
    }

    pub fn driver_name_str(&self) -> &str {
        core::str::from_utf8(&self.driver_name[..self.driver_name_len as usize]).unwrap_or("-")
    }

    /// Transition to a new state
    pub fn transition(&mut self, new_state: DeviceState) {
        self.state = new_state;
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
    /// Waiting for bus to be Safe before restart
    pub waiting_for_bus_safe: bool,
    /// True if driver was auto-started by devd (eligible for auto-restart)
    /// False if manually started (no auto-restart on crash)
    pub auto_started: bool,
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
            waiting_for_bus_safe: false,
            auto_started: false,
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
// Capability Grants
// =============================================================================

/// Determine appropriate capabilities for a driver based on its binary name
fn caps_for_driver(binary: &str) -> u64 {
    match binary {
        // Bus drivers need full hardware access + ability to spawn device drivers
        "pcied" => caps::BUS_DRIVER,

        // Device drivers need hardware access for their specific bus
        "usbd" | "nvmed" | "wifid" => caps::DEVICE_DRIVER,

        // GPIO driver needs MMIO but not IRQ/DMA
        "gpio" | "pwm" => caps::GPIO_DRIVER,

        // Filesystem drivers
        "vfsd" | "fatfs" => caps::FS_DRIVER,

        // Service daemons (no direct hardware access)
        "consoled" | "logd" => caps::SERVICE_DRIVER,

        // Shell and user programs - minimal capabilities
        "shell" | _ => caps::USER_DEFAULT,
    }
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
    /// Channel to pcied for reset operations (0 = not connected)
    pcied_channel: u32,
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
            pcied_channel: 0,
        }
    }

    /// Connect to pcied for reset operations
    fn connect_pcied(&mut self) -> bool {
        if self.pcied_channel != 0 {
            return true; // Already connected
        }

        let ch = syscall::port_connect(b"pcie");
        if ch < 0 {
            uwarn!("devd", "pcied_connect_failed"; err = ch);
            return false;
        }
        self.pcied_channel = ch as u32;
        uinfo!("devd", "pcied_connected"; channel = ch);
        true
    }

    /// Execute escalation action for a device
    fn execute_escalation(&mut self, device_idx: usize, level: EscalationLevel) -> bool {
        if device_idx >= self.device_count {
            return false;
        }

        let device = &self.devices[device_idx];
        let port = device.pci_port;
        let bus = (device.pci_bdf >> 16) as u8;
        let dev = ((device.pci_bdf >> 8) & 0xFF) as u8;
        let func = (device.pci_bdf & 0xFF) as u8;

        uinfo!("devd", "execute_escalation";
            device_id = device.id, level = level as u8, port = port);

        // For non-PCIe devices, just restart the driver
        if device.bus_type != BusType::PCIe {
            return true;
        }

        // Ensure pcied connection
        if !self.connect_pcied() {
            uerror!("devd", "escalation_no_pcied");
            return false;
        }

        let success = match level {
            EscalationLevel::DeviceReset => {
                // For device reset, we clear device status and restart driver
                // The actual device reset is handled by pcied's auto-reset on disconnect
                self.pcied_clear_device_status(port, bus, dev, func)
            }
            EscalationLevel::LinkRetrain => {
                // Reset the PCIe port (PERST# assert/deassert)
                self.pcied_reset_port(port)
            }
            EscalationLevel::SegmentReset => {
                // Same as link retrain for now - secondary bus reset
                self.pcied_reset_port(port)
            }
            EscalationLevel::BusReset => {
                // Full port reset
                self.pcied_reset_port(port)
            }
            EscalationLevel::Dead => {
                // Nothing to do - device is dead
                false
            }
        };

        if success {
            uinfo!("devd", "escalation_success"; level = level as u8);
        } else {
            uerror!("devd", "escalation_failed"; level = level as u8);
        }

        success
    }

    /// Send reset port request to pcied
    fn pcied_reset_port(&self, port: u8) -> bool {
        if self.pcied_channel == 0 {
            return false;
        }

        let request = PcieRequest::ResetPort { port };
        let mut buf = [0u8; 64];
        let Ok(len) = request.serialize(&mut buf) else {
            return false;
        };

        let result = syscall::send(self.pcied_channel, &buf[..len]);
        if result < 0 {
            uerror!("devd", "pcied_reset_send_failed"; err = result);
            return false;
        }

        // Wait for response
        let n = syscall::receive_timeout(self.pcied_channel, &mut buf, IPC_TIMEOUT_MS as u32);
        if n <= 0 {
            uerror!("devd", "pcied_reset_recv_failed"; err = n as i32);
            return false;
        }

        let Ok((response, _)) = PcieResponse::deserialize(&buf[..n as usize]) else {
            return false;
        };

        match response {
            PcieResponse::ResetResult(success) => success,
            _ => false,
        }
    }

    /// Clear device status in pcied
    fn pcied_clear_device_status(&self, port: u8, bus: u8, device: u8, function: u8) -> bool {
        if self.pcied_channel == 0 {
            return false;
        }

        let request = PcieRequest::ClearDeviceStatus { port, bus, device, function };
        let mut buf = [0u8; 64];
        let Ok(len) = request.serialize(&mut buf) else {
            return false;
        };

        let result = syscall::send(self.pcied_channel, &buf[..len]);
        if result < 0 {
            return false;
        }

        // Wait for response
        let n = syscall::receive_timeout(self.pcied_channel, &mut buf, IPC_TIMEOUT_MS as u32);
        if n <= 0 {
            return false;
        }

        let Ok((response, _)) = PcieResponse::deserialize(&buf[..n as usize]) else {
            return false;
        };

        match response {
            PcieResponse::ResetResult(success) => success,
            _ => false,
        }
    }

    /// Connect to a bus using kernel-provided path (single source of truth)
    fn connect_bus_by_path(&mut self, bus_type: BusType, index: u8, path: &str, base_addr: u32) -> bool {
        if self.bus_count >= MAX_BUSES {
            return false;
        }

        let path_bytes = path.as_bytes();
        let channel = syscall::port_connect(path_bytes);
        if channel < 0 {
            return false;
        }

        let slot = self.bus_count;
        self.buses[slot].bus_type = bus_type;
        self.buses[slot].bus_index = index;
        self.buses[slot].channel = channel as u32;
        self.buses[slot].connected = true;
        self.buses[slot].base_addr = base_addr;
        self.bus_count += 1;

        // Wait for StateSnapshot from kernel
        let mut buf = [0u8; 64];
        let len = syscall::receive(channel as u32, &mut buf);
        let kernel_state = if len > 0 && buf[0] == MSG_STATE_SNAPSHOT {
            match buf[3] {
                0 => KernelBusState::Safe,
                1 => KernelBusState::Active,
                2 => KernelBusState::Resetting,
                _ => KernelBusState::Safe,
            }
        } else {
            KernelBusState::Safe
        };
        self.buses[slot].kernel_state = kernel_state;

        true
    }

    /// Add a device
    fn add_device(&mut self, mut device: Device) -> Option<u16> {
        if self.device_count >= MAX_DEVICES {
            return None;
        }

        device.id = self.next_device_id;
        self.next_device_id += 1;

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
        let bus_info = self.devices[..self.device_count]
            .iter()
            .find(|d| d.id == device_id)
            .map(|d| (d.bus_type, d.bus_index));

        if let Some((bus_type, bus_index)) = bus_info {
            if let Some(bus) = self.buses[..self.bus_count]
                .iter()
                .find(|b| b.bus_type == bus_type && b.bus_index == bus_index)
            {
                let msg = [MSG_REQUEST_RESET];
                let _ = syscall::send(bus.channel, &msg);
            }
        }
    }

    /// Register driver PID with kernel for a device's bus
    fn set_driver_pid(&mut self, device_id: u16, driver_pid: u32) {
        let bus_info = self.devices[..self.device_count]
            .iter()
            .find(|d| d.id == device_id)
            .map(|d| (d.bus_type, d.bus_index));

        if let Some((bus_type, bus_index)) = bus_info {
            if let Some(bus) = self.buses[..self.bus_count]
                .iter()
                .find(|b| b.bus_type == bus_type && b.bus_index == bus_index)
            {
                let pid_bytes = driver_pid.to_le_bytes();
                let msg = [MSG_SET_DRIVER, pid_bytes[0], pid_bytes[1], pid_bytes[2], pid_bytes[3]];
                let _ = syscall::send(bus.channel, &msg);
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
            return None;
        }

        // Determine appropriate capabilities based on driver type
        let driver_caps = caps_for_driver(binary);
        let pid = syscall::exec_with_caps(binary, driver_caps);
        if pid < 0 {
            uerror!("devd", "spawn_failed"; binary = binary, err = pid);
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
        entry.waiting_for_bus_safe = false;
        entry.auto_started = true;  // Auto-started by devd, eligible for auto-restart

        if slot >= self.driver_count {
            self.driver_count = slot + 1;
        }

        // Update device state
        if let Some(device) = self.find_device(device_id) {
            device.driver_id = Some(slot);

            if device.bus_type == BusType::Platform {
                device.transition(DeviceState::Bound);
            } else {
                let deadline = syscall::gettime() as u64 + BIND_TIMEOUT_NS;
                device.transition(DeviceState::Binding { deadline });
            }
            uinfo!("devd", [pid = pid], "driver_spawned"; binary = binary);
        }

        // Register driver PID with kernel so it knows to auto-reset
        // the bus when this driver exits
        self.set_driver_pid(device_id, pid);

        Some(pid)
    }

    /// Handle driver exit with proper error recovery
    fn handle_driver_exit(&mut self, pid: u32, exit_code: i32) {
        // Find the driver index first
        let driver_idx = match (0..self.driver_count).find(|&i| self.drivers[i].pid == pid) {
            Some(idx) => idx,
            None => {
                uwarn!("devd", [pid = pid], "unknown_child_exit"; exit_code = exit_code);
                return;
            }
        };

        let binary = self.drivers[driver_idx].binary;
        let device_id = self.drivers[driver_idx].device_id;
        let now_ns = syscall::gettime() as u64;
        let now_ms = now_ns / 1_000_000;

        if exit_code == 0 {
            // Clean exit - successful operation
            uinfo!("devd", [pid = pid], "driver_exit_ok"; binary = binary);
            self.drivers[driver_idx].state = DriverState::Stopped;
            self.drivers[driver_idx].pid = 0;

            // Clean unbind - device goes back to DISCOVERED
            if let Some(dev_id) = device_id {
                if let Some(device) = self.find_device(dev_id) {
                    device.transition(DeviceState::Discovered);
                    device.driver_id = None;
                    // Record successful recovery if we were in recovery
                    if device.recovery_history.pending_recovery_start > 0 {
                        device.recovery_history.record_recovery(now_ms);
                        device.fault_tracker.reset();
                    }
                }
            }
        } else {
            // Driver crashed - begin error recovery
            let error_source = ErrorSource::DriverCrash(exit_code);
            let auto_started = self.drivers[driver_idx].auto_started;
            uerror!("devd", [pid = pid], "driver_crashed";
                binary = binary, exit_code = exit_code, auto_restart = auto_started);

            self.drivers[driver_idx].state = DriverState::Crashed;
            self.drivers[driver_idx].restart_count += 1;
            self.drivers[driver_idx].pid = 0;

            // If manually started, don't auto-restart
            if !auto_started {
                uinfo!("devd", "no_auto_restart"; binary = binary);
                return;
            }

            // Find device and perform escalation
            if let Some(dev_id) = device_id {
                let dev_slot = match self.devices[..self.device_count]
                    .iter()
                    .position(|d| d.id == dev_id)
                {
                    Some(idx) => idx,
                    None => return,
                };

                // Transition to FAULTED and escalate
                self.devices[dev_slot].transition(DeviceState::Faulted);
                let (level, wait_ms) = self.devices[dev_slot].fault_tracker.escalate(now_ms);

                // Record error in recovery history
                self.devices[dev_slot].recovery_history.record_error(now_ms, error_source, level);

                // Extract values for logging before any mutable borrows
                let total_errors = self.devices[dev_slot].recovery_history.total_errors;
                let recovery_rate = self.devices[dev_slot].recovery_history.recovery_rate();
                let attempts = self.devices[dev_slot].fault_tracker.attempts;

                // Log detailed recovery state
                uinfo!("devd", "escalation_state";
                    level = level as u8,
                    attempts = attempts,
                    total_errors = total_errors,
                    recovery_rate = recovery_rate);

                if level == EscalationLevel::Dead {
                    // Device is permanently dead
                    self.devices[dev_slot].transition(DeviceState::Dead);
                    self.drivers[driver_idx].state = DriverState::Failed;
                    uerror!("devd", "device_dead";
                        binary = binary,
                        total_errors = total_errors,
                        recovery_rate = recovery_rate);
                } else {
                    // Transition to RESETTING and execute escalation action
                    self.devices[dev_slot].transition(DeviceState::Resetting);

                    // Execute the escalation action (reset, retrain, etc.)
                    let escalation_success = self.execute_escalation(dev_slot, level);

                    if escalation_success {
                        // Mark for restart after wait period - will be triggered when bus is Safe
                        let wait_ns = wait_ms * 1_000_000;
                        self.drivers[driver_idx].restart_at = now_ns + wait_ns;
                        self.drivers[driver_idx].waiting_for_bus_safe = true;
                        uinfo!("devd", "driver_restart_pending";
                            binary = binary, delay_ms = wait_ms, level = level as u8);
                    } else {
                        // Escalation action failed - still mark for restart
                        uwarn!("devd", "escalation_action_failed"; level = level as u8);
                        let wait_ns = wait_ms * 1_000_000;
                        self.drivers[driver_idx].restart_at = now_ns + wait_ns;
                        self.drivers[driver_idx].waiting_for_bus_safe = true;
                    }
                }
            }
        }
    }

    /// Process pending restarts and binding timeouts
    fn process_timers(&mut self) {
        let now_ns = syscall::gettime() as u64;
        let now_ms = now_ns / 1_000_000;

        // Check binding timeouts - handle with proper error recovery
        for i in 0..self.device_count {
            if let DeviceState::Binding { deadline } = self.devices[i].state {
                if now_ns >= deadline {
                    let device_id = self.devices[i].id;
                    uwarn!("devd", "bind_timeout"; device_id = device_id);

                    // Record error in history
                    let (level, wait_ms) = self.devices[i].fault_tracker.escalate(now_ms);
                    self.devices[i].recovery_history.record_error(
                        now_ms,
                        ErrorSource::BindTimeout,
                        level
                    );

                    if level == EscalationLevel::Dead {
                        self.devices[i].transition(DeviceState::Dead);
                        let history = &self.devices[i].recovery_history;
                        uerror!("devd", "device_dead_bind_timeout";
                            device_id = device_id,
                            total_errors = history.total_errors);
                    } else {
                        // Transition to FAULTED and try escalation
                        self.devices[i].transition(DeviceState::Faulted);

                        // Execute escalation and schedule retry
                        let escalation_ok = self.execute_escalation(i, level);
                        if !escalation_ok {
                            uwarn!("devd", "bind_timeout_escalation_failed"; level = level as u8);
                        }

                        // Find driver and schedule restart
                        if let Some(driver_idx) = self.devices[i].driver_id {
                            if driver_idx < self.driver_count {
                                let wait_ns = wait_ms * 1_000_000;
                                self.drivers[driver_idx].restart_at = now_ns + wait_ns;
                                syscall::timer_set(wait_ns);
                            }
                        }
                    }
                }
            }
        }

        // Check driver restarts (timer-triggered, bus should already be Safe)
        for i in 0..self.driver_count {
            if self.drivers[i].restart_at > 0 && now_ns >= self.drivers[i].restart_at {
                // Verify bus is Safe before restarting
                let bus_safe = self.drivers[i].device_id
                    .and_then(|dev_id| self.devices[..self.device_count]
                        .iter()
                        .find(|d| d.id == dev_id))
                    .and_then(|d| self.buses[..self.bus_count]
                        .iter()
                        .find(|b| b.bus_type == d.bus_type && b.bus_index == d.bus_index))
                    .map(|b| b.kernel_state == KernelBusState::Safe)
                    .unwrap_or(true);  // Platform devices have no bus state

                if !bus_safe {
                    // Bus not Safe yet - wait for StateSnapshot(Safe)
                    self.drivers[i].waiting_for_bus_safe = true;
                    continue;
                }

                self.drivers[i].restart_at = 0;
                self.do_driver_restart(i);
            }
        }
    }

    /// Mark driver bind as complete
    pub fn driver_bind_complete(&mut self, pid: u32) {
        let now_ms = syscall::gettime() as u64 / 1_000_000;
        for i in 0..self.driver_count {
            if self.drivers[i].pid == pid {
                if let Some(dev_id) = self.drivers[i].device_id {
                    if let Some(device) = self.find_device(dev_id) {
                        if matches!(device.state, DeviceState::Binding { .. }) {
                            device.transition(DeviceState::Bound);
                            device.fault_tracker.record_success();

                            // If we were in recovery, record successful recovery
                            if device.recovery_history.pending_recovery_start > 0 {
                                device.recovery_history.record_recovery(now_ms);
                                device.fault_tracker.reset();
                                uinfo!("devd", [pid = pid], "recovery_complete";
                                    recovery_rate = device.recovery_history.recovery_rate());
                            }
                        }
                    }
                }
                return;
            }
        }
    }

    /// Handle bus becoming Safe - restart any pending drivers on that bus
    /// Called when StateSnapshot with state=Safe arrives from kernel
    pub fn handle_bus_safe(&mut self, bus_type: BusType, bus_index: u8) {
        let now_ns = syscall::gettime() as u64;

        for i in 0..self.driver_count {
            if !self.drivers[i].waiting_for_bus_safe {
                continue;
            }

            // Check if this driver is for a device on the bus that became Safe
            let device_on_this_bus = self.drivers[i].device_id
                .and_then(|dev_id| self.devices[..self.device_count]
                    .iter()
                    .find(|d| d.id == dev_id))
                .map(|d| d.bus_type == bus_type && d.bus_index == bus_index)
                .unwrap_or(false);

            if !device_on_this_bus {
                continue;
            }

            let binary = self.drivers[i].binary;
            let restart_at = self.drivers[i].restart_at;

            if restart_at <= now_ns {
                // Delay has passed - restart immediately
                self.drivers[i].waiting_for_bus_safe = false;
                uinfo!("devd", "bus_safe_restart_now"; binary = binary);
                self.do_driver_restart(i);
            } else {
                // Delay not yet passed - set timer for remaining time
                let remaining_ns = restart_at - now_ns;
                self.drivers[i].waiting_for_bus_safe = false;
                syscall::timer_set(remaining_ns);
                uinfo!("devd", "bus_safe_restart_delayed";
                    binary = binary, delay_ms = remaining_ns / 1_000_000);
            }
        }
    }

    /// Perform driver restart (extracted from process_timers for reuse)
    fn do_driver_restart(&mut self, driver_idx: usize) {
        let binary = self.drivers[driver_idx].binary;
        let device_id = self.drivers[driver_idx].device_id;
        let now_ns = syscall::gettime() as u64;

        if self.drivers[driver_idx].restart_count > MAX_RESTARTS {
            uerror!("devd", "max_restarts_exceeded";
                binary = binary,
                restart_count = self.drivers[driver_idx].restart_count);

            if let Some(dev_id) = device_id {
                if let Some(device) = self.find_device(dev_id) {
                    device.transition(DeviceState::Dead);
                }
            }
            return;
        }

        let driver_caps = caps_for_driver(binary);
        let new_pid = syscall::exec_with_caps(binary, driver_caps);
        if new_pid > 0 {
            self.drivers[driver_idx].pid = new_pid as u32;
            self.drivers[driver_idx].state = DriverState::Running;
            uinfo!("devd", [pid = new_pid], "driver_restarted";
                binary = binary,
                attempt = self.drivers[driver_idx].restart_count);

            if let Some(dev_id) = device_id {
                if let Some(device) = self.find_device(dev_id) {
                    let deadline = now_ns + BIND_TIMEOUT_NS;
                    device.transition(DeviceState::Binding { deadline });
                }
                self.set_driver_pid(dev_id, new_pid as u32);
            }
        } else {
            uerror!("devd", "restart_exec_failed"; binary = binary, err = new_pid);
            self.drivers[driver_idx].state = DriverState::Failed;

            if let Some(dev_id) = device_id {
                if let Some(device) = self.find_device(dev_id) {
                    device.transition(DeviceState::Dead);
                }
            }
        }
    }

    // =========================================================================
    // hw: Scheme Handler
    // =========================================================================

    /// Format device list for hw:list (compact format to fit IPC limit)
    pub fn format_device_list(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;

        // Compact header
        pos += write_str(&mut buf[pos..], "BUSES:\r\n");

        // List buses compactly
        for i in 0..self.bus_count {
            if !self.buses[i].connected {
                continue;
            }
            let bus = &self.buses[i];
            pos += write_str(&mut buf[pos..], "  ");
            let bus_name = match bus.bus_type {
                BusType::PCIe => "pcie",
                BusType::Usb => "usb",
                BusType::Platform => "plat",
            };
            pos += write_str(&mut buf[pos..], bus_name);
            if bus.bus_type != BusType::Platform {
                pos += write_num(&mut buf[pos..], bus.bus_index as usize);
            }
            pos += write_str(&mut buf[pos..], " [");
            pos += write_str(&mut buf[pos..], match bus.kernel_state {
                KernelBusState::Safe => "safe",
                KernelBusState::Active => "actv",
                KernelBusState::Resetting => "rst",
            });
            pos += write_str(&mut buf[pos..], "]\r\n");
        }

        // Devices section - compact format: "  path vid:did class [state]"
        if self.device_count > 0 {
            pos += write_str(&mut buf[pos..], "DEVICES:\r\n");
            for i in 0..self.device_count {
                // Check buffer space (leave 60 bytes margin)
                if pos + 60 > buf.len() {
                    pos += write_str(&mut buf[pos..], "  ...(truncated)\r\n");
                    break;
                }

                let dev = &self.devices[i];
                pos += write_str(&mut buf[pos..], "  ");

                // Short path (e.g., "pcie0/01:00.0")
                let path = dev.path_str();
                // Strip "/bus/" prefix if present
                let display_path = if let Some(stripped) = path.strip_prefix("/bus/") {
                    stripped
                } else {
                    path
                };
                pos += write_str(&mut buf[pos..], display_path);
                pos += write_str(&mut buf[pos..], " ");

                // vendor:device
                pos += write_hex16(&mut buf[pos..], dev.vendor_id);
                pos += write_str(&mut buf[pos..], ":");
                pos += write_hex16(&mut buf[pos..], dev.device_id);
                pos += write_str(&mut buf[pos..], " ");

                // class
                pos += write_str(&mut buf[pos..], match dev.class {
                    DeviceClass::UsbController => "usb",
                    DeviceClass::Serial => "ser",
                    DeviceClass::Storage => "sto",
                    DeviceClass::Network => "net",
                    DeviceClass::Hid => "hid",
                    DeviceClass::Other => "oth",
                });

                // state
                pos += write_str(&mut buf[pos..], " [");
                pos += write_str(&mut buf[pos..], match dev.state {
                    DeviceState::Discovered => "disc",
                    DeviceState::Binding { .. } => "bind",
                    DeviceState::Bound => "BOUND",
                    DeviceState::Faulted => "FAIL",
                    DeviceState::Resetting => "rst",
                    DeviceState::Suspended => "susp",
                    DeviceState::Dead => "DEAD",
                });
                // Add driver name if bound
                if let DeviceState::Bound = dev.state {
                    let drv = dev.driver_name_str();
                    if drv != "-" && !drv.is_empty() {
                        pos += write_str(&mut buf[pos..], ":");
                        pos += write_str(&mut buf[pos..], drv);
                    }
                }
                pos += write_str(&mut buf[pos..], "]\r\n");
            }
        }

        // CPU stats section
        pos += format_cpu_stats(&mut buf[pos..]);

        pos
    }

    /// Format device tree with colors for hw:tree (prettier output)
    pub fn format_tree_colored(&self, buf: &mut [u8]) -> usize {
        let mut pos = 0;

        // Title
        pos += write_str(&mut buf[pos..], Style::EMPHASIS);
        pos += write_str(&mut buf[pos..], "Hardware Tree");
        pos += write_str(&mut buf[pos..], Style::RESET);
        pos += write_str(&mut buf[pos..], "\r\n");

        // Count connected buses
        let connected_buses: usize = (0..self.bus_count)
            .filter(|&i| self.buses[i].connected)
            .count();
        let mut bus_idx = 0;

        // List buses with their devices
        for i in 0..self.bus_count {
            if !self.buses[i].connected {
                continue;
            }
            let bus = &self.buses[i];
            bus_idx += 1;
            let is_last_bus = bus_idx == connected_buses;

            // Bus line: ├─ pcie0 ● Active
            pos += write_str(&mut buf[pos..], if is_last_bus { BoxChar::LAST } else { BoxChar::BRANCH });
            pos += write_str(&mut buf[pos..], " ");

            // Bus name in cyan
            pos += write_str(&mut buf[pos..], Style::BUS);
            let bus_name = match bus.bus_type {
                BusType::PCIe => "pcie",
                BusType::Usb => "usb",
                BusType::Platform => "platform",
            };
            pos += write_str(&mut buf[pos..], bus_name);
            if bus.bus_type != BusType::Platform {
                pos += write_num(&mut buf[pos..], bus.bus_index as usize);
            }
            pos += write_str(&mut buf[pos..], Style::RESET);
            pos += write_str(&mut buf[pos..], " ");

            // Status indicator
            match bus.kernel_state {
                KernelBusState::Active => {
                    pos += write_str(&mut buf[pos..], Style::STATUS_BOUND);
                    pos += write_str(&mut buf[pos..], Symbol::ACTIVE);
                    pos += write_str(&mut buf[pos..], Style::RESET);
                    pos += write_str(&mut buf[pos..], " Active");
                }
                KernelBusState::Safe => {
                    pos += write_str(&mut buf[pos..], Style::STATUS_IDLE);
                    pos += write_str(&mut buf[pos..], Symbol::IDLE);
                    pos += write_str(&mut buf[pos..], Style::RESET);
                    pos += write_str(&mut buf[pos..], " Idle");
                }
                KernelBusState::Resetting => {
                    pos += write_str(&mut buf[pos..], Style::WARN);
                    pos += write_str(&mut buf[pos..], Symbol::ACTIVE);
                    pos += write_str(&mut buf[pos..], Style::RESET);
                    pos += write_str(&mut buf[pos..], " Resetting");
                }
            }
            pos += write_str(&mut buf[pos..], "\r\n");

            // Find devices on this bus
            let bus_prefix = if is_last_bus { "   " } else { BoxChar::CONT };
            let mut devices_on_bus: [usize; MAX_DEVICES] = [0; MAX_DEVICES];
            let mut dev_count = 0;

            for j in 0..self.device_count {
                let dev = &self.devices[j];
                if dev.bus_index == bus.bus_index && dev.bus_type == bus.bus_type {
                    if dev_count < MAX_DEVICES {
                        devices_on_bus[dev_count] = j;
                        dev_count += 1;
                    }
                }
            }

            // Print devices
            for d in 0..dev_count {
                let dev_idx = devices_on_bus[d];
                let dev = &self.devices[dev_idx];
                let is_last_dev = d == dev_count - 1;

                // Device line: │  ├─ 01:00.0 ● MT7996 [wifid3]
                pos += write_str(&mut buf[pos..], bus_prefix);
                pos += write_str(&mut buf[pos..], if is_last_dev { BoxChar::LAST } else { BoxChar::BRANCH });
                pos += write_str(&mut buf[pos..], " ");

                // BDF or device path (short form)
                pos += write_str(&mut buf[pos..], Style::DEVICE);
                let path = dev.path_str();
                // Extract just the device part (after last /)
                let short_path = path.rsplit('/').next().unwrap_or(path);
                pos += write_str(&mut buf[pos..], short_path);
                pos += write_str(&mut buf[pos..], Style::RESET);
                pos += write_str(&mut buf[pos..], " ");

                // State indicator
                match dev.state {
                    DeviceState::Bound => {
                        pos += write_str(&mut buf[pos..], Style::STATUS_BOUND);
                        pos += write_str(&mut buf[pos..], Symbol::ACTIVE);
                        pos += write_str(&mut buf[pos..], Style::RESET);
                    }
                    DeviceState::Faulted | DeviceState::Dead => {
                        pos += write_str(&mut buf[pos..], Style::STATUS_ERROR);
                        pos += write_str(&mut buf[pos..], Symbol::CROSS);
                        pos += write_str(&mut buf[pos..], Style::RESET);
                    }
                    _ => {
                        pos += write_str(&mut buf[pos..], Style::STATUS_IDLE);
                        pos += write_str(&mut buf[pos..], Symbol::IDLE);
                        pos += write_str(&mut buf[pos..], Style::RESET);
                    }
                }
                pos += write_str(&mut buf[pos..], " ");

                // Vendor:device in yellow
                pos += write_str(&mut buf[pos..], Style::ID);
                pos += write_hex16(&mut buf[pos..], dev.vendor_id);
                pos += write_str(&mut buf[pos..], ":");
                pos += write_hex16(&mut buf[pos..], dev.device_id);
                pos += write_str(&mut buf[pos..], Style::RESET);

                // Driver name in magenta if bound
                if let DeviceState::Bound = dev.state {
                    let drv = dev.driver_name_str();
                    if drv != "-" && !drv.is_empty() {
                        pos += write_str(&mut buf[pos..], " ");
                        pos += write_str(&mut buf[pos..], Style::DRIVER);
                        pos += write_str(&mut buf[pos..], drv);
                        pos += write_str(&mut buf[pos..], Style::RESET);
                    }
                }

                pos += write_str(&mut buf[pos..], "\r\n");
            }
        }

        // Legend
        pos += write_str(&mut buf[pos..], "\r\n");
        pos += write_str(&mut buf[pos..], Style::MUTED);
        pos += write_str(&mut buf[pos..], Symbol::ACTIVE);
        pos += write_str(&mut buf[pos..], "=active ");
        pos += write_str(&mut buf[pos..], Symbol::IDLE);
        pos += write_str(&mut buf[pos..], "=idle ");
        pos += write_str(&mut buf[pos..], Symbol::CROSS);
        pos += write_str(&mut buf[pos..], "=error");
        pos += write_str(&mut buf[pos..], Style::RESET);
        pos += write_str(&mut buf[pos..], "\r\n");

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
                KernelBusState::Active => "active",
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
            "tree" => self.format_tree_colored(buf),
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
// CPU Stats
// =============================================================================

/// Format CPU statistics for hw:list output
fn format_cpu_stats(buf: &mut [u8]) -> usize {
    let mut pos = 0;

    // Get CPU stats from kernel
    let mut cpu_stats = [CpuStatsEntry::empty(); 4];  // MAX_CPUS
    let num_cpus = syscall::cpu_stats(&mut cpu_stats);

    if num_cpus <= 0 {
        return 0;
    }

    pos += write_str(&mut buf[pos..], "CPUS:\r\n");

    for i in 0..(num_cpus as usize) {
        let cpu = &cpu_stats[i];

        // "  cpu0 busy% [ticks/idle]"
        pos += write_str(&mut buf[pos..], "  cpu");
        pos += write_num(&mut buf[pos..], cpu.cpu_id as usize);
        pos += write_str(&mut buf[pos..], " ");

        // Calculate and display busy percentage
        if let Some(busy_pct) = cpu.busy_percent() {
            pos += write_num(&mut buf[pos..], busy_pct as usize);
            pos += write_str(&mut buf[pos..], "%");
        } else {
            pos += write_str(&mut buf[pos..], "--%");
        }

        // Show tick counts in brackets
        pos += write_str(&mut buf[pos..], " [");
        pos += write_num(&mut buf[pos..], cpu.tick_count as usize);
        pos += write_str(&mut buf[pos..], "/");
        pos += write_num(&mut buf[pos..], cpu.idle_ticks as usize);
        pos += write_str(&mut buf[pos..], "]\r\n");
    }

    pos
}

// =============================================================================
// Bus Scanning
// =============================================================================

// MT7988A platform device addresses
const UART0_BASE: u32 = 0x1100_2000;  // UART0 MMIO base
const I2C0_BASE: u32 = 0x1100_7000;   // I2C0 (for GPIO expander)

fn scan_platform(mgr: &mut DeviceManager) {
    // UART0 - always present on platform bus
    let mut uart = Device::empty();
    uart.set_path("/platform/uart0");
    uart.bus_type = BusType::Platform;
    uart.bus_index = 0;
    uart.class = DeviceClass::Serial;
    uart.vendor_id = (UART0_BASE >> 16) as u16;
    uart.device_id = (UART0_BASE & 0xFFFF) as u16;
    mgr.add_device(uart);

    // GPIO - I2C-attached PCA9555 via I2C0
    let mut gpio = Device::empty();
    gpio.set_path("/platform/gpio");
    gpio.bus_type = BusType::Platform;
    gpio.bus_index = 0;
    gpio.class = DeviceClass::Other;
    gpio.vendor_id = (I2C0_BASE >> 16) as u16;
    gpio.device_id = 0x0027;
    mgr.add_device(gpio);
}

// =============================================================================
// Driver Matching
// =============================================================================

fn match_and_spawn(mgr: &mut DeviceManager) {
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

fn handle_devd_request(
    tree: &mut DeviceTree,
    mgr: &mut DeviceManager,
    req: &DevdRequest,
    client_ch: u32,
) -> DevdResponse {
    match req {
        DevdRequest::Register { path, path_len, properties } => {
            let path_str = core::str::from_utf8(&path[..*path_len as usize]).unwrap_or("");

            // Parse path to determine bus and add to DeviceManager
            if path_str.starts_with("/bus/") {
                let rest = &path_str[5..];
                if let Some(slash_pos) = rest.find('/') {
                    let bus_part = &rest[..slash_pos];
                    let (bus_type, bus_index) = if bus_part.starts_with("pcie") {
                        (BusType::PCIe, bus_part[4..].parse::<u8>().unwrap_or(0))
                    } else if bus_part.starts_with("usb") {
                        (BusType::Usb, bus_part[3..].parse::<u8>().unwrap_or(0))
                    } else {
                        (BusType::Platform, 0)
                    };

                    let vendor_id = properties.get("vendor")
                        .and_then(|v| u16::from_str_radix(v.trim_start_matches("0x"), 16).ok())
                        .unwrap_or(0);
                    let device_id = properties.get("device")
                        .and_then(|v| u16::from_str_radix(v.trim_start_matches("0x"), 16).ok())
                        .unwrap_or(0);
                    let class_str = properties.get("class").unwrap_or("other");
                    let class = match class_str {
                        "network" => DeviceClass::Network,
                        "storage" => DeviceClass::Storage,
                        "usb" | "usb-ctrl" => DeviceClass::UsbController,
                        "serial" => DeviceClass::Serial,
                        "hid" => DeviceClass::Hid,
                        _ => DeviceClass::Other,
                    };

                    // Add to DeviceManager if not already present
                    let exists = mgr.devices[..mgr.device_count].iter().any(|d| d.path_str() == path_str);
                    if !exists && mgr.device_count < MAX_DEVICES {
                        let dev = &mut mgr.devices[mgr.device_count];
                        dev.id = mgr.device_count as u16;
                        dev.set_path(path_str);
                        dev.bus_type = bus_type;
                        dev.bus_index = bus_index;
                        dev.vendor_id = vendor_id;
                        dev.device_id = device_id;
                        dev.class = class;
                        dev.state = DeviceState::Discovered;
                        mgr.device_count += 1;
                        uinfo!("devd", "device_discovered"; vendor = vendor_id, device = device_id);
                    }
                }
            }

            match tree.register(path_str, properties.clone()) {
                Ok(()) => DevdResponse::Ok,
                Err(e) => DevdResponse::Error(e),
            }
        }

        DevdRequest::Update { path, path_len, properties } => {
            let path_str = core::str::from_utf8(&path[..*path_len as usize]).unwrap_or("");
            match tree.update(path_str, properties.clone()) {
                Ok(()) => DevdResponse::Ok,
                Err(e) => DevdResponse::Error(e),
            }
        }

        DevdRequest::Query { pattern, pattern_len } => {
            let pattern_str = core::str::from_utf8(&pattern[..*pattern_len as usize]).unwrap_or("");
            DevdResponse::Nodes(tree.query(pattern_str))
        }

        DevdRequest::Subscribe { pattern, pattern_len } => {
            let pattern_str = core::str::from_utf8(&pattern[..*pattern_len as usize]).unwrap_or("");
            match tree.subscribe(pattern_str, client_ch) {
                Ok(()) => DevdResponse::Ok,
                Err(e) => DevdResponse::Error(e),
            }
        }

        DevdRequest::Unsubscribe { pattern, pattern_len } => {
            let pattern_str = core::str::from_utf8(&pattern[..*pattern_len as usize]).unwrap_or("");
            tree.unsubscribe(pattern_str, client_ch);
            DevdResponse::Ok
        }

        DevdRequest::Remove { path, path_len } => {
            let path_str = core::str::from_utf8(&path[..*path_len as usize]).unwrap_or("");
            match tree.remove(path_str) {
                Ok(()) => DevdResponse::Ok,
                Err(e) => DevdResponse::Error(e),
            }
        }

        DevdRequest::ClaimBus { path, path_len } => {
            // Security: ClaimBus requires SCHEME_CREATE capability
            let client_pid = syscall::channel_get_peer(client_ch);
            if client_pid > 0 {
                let caps = syscall::get_capabilities(client_pid as u32);
                if caps >= 0 && !syscall::caps::has(caps as u64, syscall::caps::SCHEME_CREATE) {
                    uerror!("devd", "claim_bus_denied"; client_pid = client_pid as u32);
                    return DevdResponse::Error(-1); // Permission denied
                }
            }

            let path_str = core::str::from_utf8(&path[..*path_len as usize]).unwrap_or("");

            if let Some(bus_name) = path_str.strip_prefix("/bus/") {
                let (bus_type, bus_index) = if let Some(idx_str) = bus_name.strip_prefix("pcie") {
                    (BusType::PCIe, idx_str.parse::<u8>().unwrap_or(255))
                } else if let Some(idx_str) = bus_name.strip_prefix("usb") {
                    (BusType::Usb, idx_str.parse::<u8>().unwrap_or(255))
                } else {
                    return DevdResponse::Error(-2);
                };

                for i in 0..mgr.bus_count {
                    if mgr.buses[i].bus_type == bus_type && mgr.buses[i].bus_index == bus_index {
                        mgr.buses[i].kernel_state = KernelBusState::Active;
                        uinfo!("devd", "bus_claimed"; bus = bus_name);
                        return DevdResponse::Ok;
                    }
                }
                DevdResponse::Error(-2)
            } else {
                DevdResponse::Error(-22)
            }
        }

        DevdRequest::ClaimDevice { path, path_len, driver, driver_len } => {
            // Security: ClaimDevice requires IPC capability
            let client_pid = syscall::channel_get_peer(client_ch);
            if client_pid > 0 {
                let caps = syscall::get_capabilities(client_pid as u32);
                if caps >= 0 && !syscall::caps::has(caps as u64, syscall::caps::IPC) {
                    uerror!("devd", "claim_device_denied"; client_pid = client_pid as u32);
                    return DevdResponse::Error(-1); // Permission denied
                }
            }

            let path_str = core::str::from_utf8(&path[..*path_len as usize]).unwrap_or("");
            let driver_str = core::str::from_utf8(&driver[..*driver_len as usize]).unwrap_or("");

            for i in 0..mgr.device_count {
                if mgr.devices[i].path_str() == path_str {
                    mgr.devices[i].state = DeviceState::Bound;
                    mgr.devices[i].set_driver_name(driver_str);
                    break;
                }
            }

            let props = PropertyList::new()
                .add("state", "bound")
                .add("driver", driver_str);

            match tree.update(path_str, props) {
                Ok(()) => {
                    uinfo!("devd", "device_bound"; driver = driver_str);
                    DevdResponse::Ok
                }
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
    // Disable all stdout to not corrupt consoled's display
    userlib::io::disable_stdout();
    userlib::ulog::disable_stdout();

    userlib::ulog::init();
    uinfo!("devd", "init_start"; version = "1.0");

    let mut mgr = DeviceManager::new();
    let mut tree = DeviceTree::new();

    // Phase 1: Query kernel for available buses and connect
    let mut buses = [syscall::BusInfo::empty(); 16];
    let bus_count = syscall::bus_list(&mut buses);
    if bus_count < 0 {
        uerror!("devd", "bus_list_failed"; err = bus_count);
        loop { syscall::exit(1); }
    }
    let bus_count = bus_count as usize;

    // Connect to each bus and track if we found PCIe buses
    let mut has_pcie = false;
    for i in 0..bus_count {
        let info = &buses[i];
        let bus_type = match info.bus_type {
            syscall::bus_type::PCIE => { has_pcie = true; BusType::PCIe },
            syscall::bus_type::USB => BusType::Usb,
            syscall::bus_type::PLATFORM => BusType::Platform,
            _ => continue,
        };
        mgr.connect_bus_by_path(bus_type, info.bus_index, info.path_str(), info.base_addr);
    }
    uinfo!("devd", "buses_connected"; count = mgr.bus_count);

    // Phase 2: Scan platform bus for static devices
    scan_platform(&mut mgr);

    // Phase 3: Register schemes and ports BEFORE spawning drivers

    // Create a channel for hw: scheme requests
    // channel_create returns (ch_b << 32) | ch_a
    // When kernel sends to ch_a, message goes to ch_b's queue (peer)
    // So: register ch_a with scheme, receive on ch_b
    let hw_channel_pair = syscall::channel_create();
    let mut hw_channel_valid = false;
    let hw_daemon_channel = (hw_channel_pair & 0xFFFFFFFF) as u32;  // Register this
    let hw_channel = ((hw_channel_pair >> 32) & 0xFFFFFFFF) as u32; // Receive on this
    if hw_channel_pair < 0 {
        uerror!("devd", "channel_create_failed"; scheme = "hw", err = hw_channel_pair);
    } else {
        let result = syscall::scheme_register("hw", hw_daemon_channel);
        if result < 0 {
            uerror!("devd", "scheme_register_failed"; scheme = "hw", err = result);
        } else {
            uinfo!("devd", "scheme_registered"; scheme = "hw", daemon_ch = hw_daemon_channel, recv_ch = hw_channel);
            hw_channel_valid = true;
        }
    }

    // Register "devd" port for protocol requests
    // IMPORTANT: Must be registered BEFORE spawning drivers so they can connect
    let devd_port = syscall::port_register(b"devd");
    let mut devd_port_valid = false;
    if devd_port < 0 {
        uerror!("devd", "port_register_failed"; port = "devd", err = devd_port);
    } else {
        uinfo!("devd", "port_registered"; port = "devd", port_id = devd_port);
        devd_port_valid = true;

        // Subscribe to IpcReady for devd port immediately (kevent API)
        if kevent_subscribe(EventFilter::Ipc(devd_port as u32)).is_err() {
            uwarn!("devd", "ipc_subscribe_failed"; port = "devd");
            devd_port_valid = false;
        }
    }

    // Phase 4: Auto-start system services
    // vfsd provides VFS protocol for initrd filesystem access
    let vfsd_pid = syscall::exec_with_caps("vfsd", caps::FS_DRIVER);
    if vfsd_pid > 0 {
        uinfo!("devd", "service_spawned"; service = "vfsd", pid = vfsd_pid);
        // Register vfsd in device tree as a service
        let props = PropertyList::new()
            .add("type", "service")
            .add("state", "running")
            .add("pid", "vfsd");  // Note: will show "vfsd" not actual pid
        let _ = tree.register("/service/vfsd", props);
    } else {
        uerror!("devd", "service_spawn_failed"; service = "vfsd", err = vfsd_pid);
    }

    // consoled manages terminal display with split regions for logs and shell
    let consoled_pid = syscall::exec_with_caps("consoled", caps::SERVICE_DRIVER);
    if consoled_pid > 0 {
        uinfo!("devd", "service_spawned"; service = "consoled", pid = consoled_pid);
        let props = PropertyList::new()
            .add("type", "service")
            .add("state", "running")
            .add("pid", "consoled");
        let _ = tree.register("/service/consoled", props);
    } else {
        uerror!("devd", "service_spawn_failed"; service = "consoled", err = consoled_pid);
    }

    // Phase 5: Auto-start bus drivers for discovered buses
    // pcied manages PCIe enumeration and device discovery
    if has_pcie {
        let pid = syscall::exec_with_caps("pcied", caps::BUS_DRIVER);
        if pid > 0 {
            uinfo!("devd", "bus_driver_spawned"; driver = "pcied", pid = pid);
        } else {
            uerror!("devd", "bus_driver_spawn_failed"; driver = "pcied", err = pid);
        }
    }

    // Phase 6: Match and spawn device drivers
    match_and_spawn(&mut mgr);

    uinfo!("devd", "supervision_started"; buses = mgr.bus_count, devices = mgr.device_count, drivers = mgr.driver_count);

    // Phase 7: Subscribe to events (CRITICAL) - kevent API
    if kevent_subscribe(EventFilter::ChildExit(0)).is_err() {
        uerror!("devd", "event_subscribe_failed"; event = "ChildExit");
        panic!("devd: ChildExit subscription failed");
    }

    if kevent_subscribe(EventFilter::Timer(0)).is_err() {
        uerror!("devd", "event_subscribe_failed"; event = "Timer");
        panic!("devd: Timer subscription failed");
    }

    if hw_channel_valid {
        if kevent_subscribe(EventFilter::Ipc(hw_channel)).is_err() {
            hw_channel_valid = false;
        }
    }

    for i in 0..mgr.bus_count {
        if mgr.buses[i].connected {
            let ch = mgr.buses[i].channel;
            if kevent_subscribe(EventFilter::Ipc(ch)).is_err() {
                uerror!("devd", "event_subscribe_failed"; event = "IpcReady", channel = ch);
                panic!("devd: Bus channel subscription failed");
            }
        }
    }

    // Phase 8: Main supervision loop
    let mut loop_count = 0u32;
    let mut events = [Event::empty(); 8];  // Batch receive up to 8 events

    // Track active client connections (persistent channels from port_accept)
    let mut client_channels: [u32; MAX_CLIENTS] = [0; MAX_CLIENTS];
    let mut client_count: usize = 0;

    loop {
        // Wait for events (blocking, batch receive) - kevent API
        let count = match kevent_wait(&mut events, u64::MAX) {
            Ok(n) => n,
            Err(_) => {
                syscall::yield_now();
                continue;
            }
        };

        if count == 0 {
            syscall::yield_now();
            continue;
        }

        // Process all received events
        for event in &events[..count] {
        loop_count = loop_count.wrapping_add(1);

        match event.event_type {
            et if et == event_type::CHILD_EXIT => {
                // Child process exited
                let child_pid = event.data as u32;
                let exit_code = event.flags as i32;
                mgr.handle_driver_exit(child_pid, exit_code);
            }

            et if et == event_type::IPC_READY => {
                let channel = event.data as u32;

                if hw_channel_valid && channel == hw_channel {
                    // hw: scheme request
                    let mut buf = [0u8; 256];
                    let len = syscall::receive(hw_channel, &mut buf);
                    if len >= 8 {
                        let server_ch = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                        let _flags = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
                        let path = core::str::from_utf8(&buf[8..len as usize]).unwrap_or("");

                        let mut response = [0u8; 560];
                        let data_len = if path == "tree" {
                            tree.format_tree(&mut response)
                        } else if path == "list" || path.is_empty() {
                            let mgr_len = mgr.format_device_list(&mut response);
                            tree.append_to_list(&mut response, mgr_len)
                        } else {
                            mgr.handle_hw_request(path, &mut response)
                        };

                        let send_len = data_len.min(560);
                        let _ = syscall::send(server_ch, &response[..send_len]);
                    }
                } else if devd_port_valid && channel == devd_port as u32 {
                    // New connection on devd port
                    let client_ch = syscall::port_accept(devd_port as u32);
                    if client_ch > 0 {
                        if client_count < MAX_CLIENTS {
                            client_channels[client_count] = client_ch as u32;
                            client_count += 1;
                            let _ = kevent_subscribe(EventFilter::Ipc(client_ch as u32));

                            // Handle any pending request
                            let mut buf = [0u8; 576];
                            let len = syscall::receive(client_ch as u32, &mut buf);
                            if len > 0 {
                                let response = match DevdRequest::deserialize(&buf[..len as usize]) {
                                    Ok((req, _)) => handle_devd_request(&mut tree, &mut mgr, &req, client_ch as u32),
                                    Err(_) => DevdResponse::Error(-22),
                                };
                                let mut resp_buf = [0u8; 576];
                                if let Ok(resp_len) = response.serialize(&mut resp_buf) {
                                    let _ = syscall::send(client_ch as u32, &resp_buf[..resp_len]);
                                }
                            }
                        } else {
                            syscall::channel_close(client_ch as u32);
                        }
                    }
                } else {
                    // Check if it's a client channel
                    let mut found_client = false;
                    for i in 0..client_count {
                        if client_channels[i] == channel {
                            found_client = true;
                            let mut buf = [0u8; 576];
                            let len = syscall::receive(channel, &mut buf);
                            if len > 0 {
                                let response = match DevdRequest::deserialize(&buf[..len as usize]) {
                                    Ok((req, _)) => handle_devd_request(&mut tree, &mut mgr, &req, channel),
                                    Err(_) => DevdResponse::Error(-22),
                                };
                                let mut resp_buf = [0u8; 576];
                                if let Ok(resp_len) = response.serialize(&mut resp_buf) {
                                    let send_result = syscall::send(channel, &resp_buf[..resp_len]);
                                    if send_result < 0 {
                                        let _ = kevent_unsubscribe(EventFilter::Ipc(channel));
                                        client_channels[i] = client_channels[client_count - 1];
                                        client_channels[client_count - 1] = 0;
                                        client_count -= 1;
                                    }
                                }
                            } else if len == -3 {
                                let _ = kevent_unsubscribe(EventFilter::Ipc(channel));
                                syscall::channel_close(channel);
                                client_channels[i] = client_channels[client_count - 1];
                                client_channels[client_count - 1] = 0;
                                client_count -= 1;
                            }
                            break;
                        }
                    }

                    // If not a client channel, check if it's a bus control channel
                    if !found_client {
                        for i in 0..mgr.bus_count {
                            if mgr.buses[i].connected && mgr.buses[i].channel == channel {
                                let mut buf = [0u8; 64];
                                let len = syscall::receive(channel, &mut buf);
                                if len > 0 && buf[0] == MSG_STATE_SNAPSHOT {
                                    let bus_type = mgr.buses[i].bus_type;
                                    let bus_index = mgr.buses[i].bus_index;
                                    let new_state = match buf[3] {
                                        0 => KernelBusState::Safe,
                                        1 => KernelBusState::Active,
                                        2 => KernelBusState::Resetting,
                                        _ => mgr.buses[i].kernel_state,
                                    };
                                    let state_changed = new_state != mgr.buses[i].kernel_state;
                                    if state_changed {
                                        uinfo!("devd", "bus_state_changed"; bus_index = bus_index, new_state = new_state as u8);
                                        mgr.buses[i].kernel_state = new_state;
                                    }
                                    // If bus became Safe, trigger pending driver restarts
                                    if new_state == KernelBusState::Safe && state_changed {
                                        mgr.handle_bus_safe(bus_type, bus_index);
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
            }

            et if et == event_type::TIMER => {
                // Timer expired - process driver restarts and binding timeouts
                mgr.process_timers();
            }

            _ => {}
        }
        }  // end for event in events
    }  // end loop
}
