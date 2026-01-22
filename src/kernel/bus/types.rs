//! Bus Types and Structures
//!
//! Contains bus type definitions, capability flags, and info structures
//! for the bus subsystem.

/// Maximum columns in device name
pub const MAX_NAME_LEN: usize = 16;

// =============================================================================
// Bus Capability Flags (reported in StateSnapshot)
// =============================================================================

/// Bus capability flags - reported to devd so it knows what features are available
pub mod bus_caps {
    // PCIe capabilities
    /// PCIe: Bus mastering (DMA) supported
    pub const PCIE_BUS_MASTER: u8 = 1 << 0;
    /// PCIe: MSI interrupts supported
    pub const PCIE_MSI: u8 = 1 << 1;
    /// PCIe: MSI-X interrupts supported
    pub const PCIE_MSIX: u8 = 1 << 2;
    /// PCIe: Link is up and trained
    pub const PCIE_LINK_UP: u8 = 1 << 3;

    // USB capabilities
    /// USB: USB 2.0 (EHCI/high-speed) supported
    pub const USB_2_0: u8 = 1 << 0;
    /// USB: USB 3.0 (xHCI/super-speed) supported
    pub const USB_3_0: u8 = 1 << 1;
    /// USB: Controller is running (not halted)
    pub const USB_RUNNING: u8 = 1 << 2;

    // Platform capabilities
    /// Platform: MMIO regions available
    pub const PLATFORM_MMIO: u8 = 1 << 0;
    /// Platform: IRQs available
    pub const PLATFORM_IRQ: u8 = 1 << 1;
}

/// Device flags for DeviceInfo
pub mod device_flags {
    /// Device has children to enumerate (pcie, usb controllers)
    pub const ENUMERABLE: u8 = 1 << 0;
    /// Device has MMIO region
    pub const MMIO: u8 = 1 << 1;
    /// Device has IRQ
    pub const IRQ: u8 = 1 << 2;
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
    /// Platform pseudo-bus (uart, gpio, i2c, spi, etc.)
    Platform = 2,
}

impl BusType {
    pub fn as_str(&self) -> &'static str {
        match self {
            BusType::PCIe => "pcie",
            BusType::Usb => "usb",
            BusType::Platform => "platform",
        }
    }
}

// =============================================================================
// Bus Info Structure
// =============================================================================

/// Bus information returned by bus_list syscall
/// Layout must match userlib::syscall::BusInfo
#[derive(Clone, Copy)]
#[repr(C)]
pub struct BusInfo {
    /// Bus type (0=PCIe, 1=USB, 2=Platform)
    pub bus_type: u8,
    /// Bus index within type (e.g., 0 for pcie0)
    pub bus_index: u8,
    /// Current state (0=Safe, 1=Claimed, 2=Resetting)
    pub state: u8,
    /// Padding for alignment
    pub _pad: u8,
    /// MMIO base address (from DTB/hardcoded)
    pub base_addr: u32,
    /// Owner PID (0 if no owner)
    pub owner_pid: u32,
    /// Port path (e.g., "/kernel/bus/pcie0")
    pub path: [u8; 32],
    /// Length of path string
    pub path_len: u8,
    /// Reserved for future use
    pub _reserved: [u8; 3],
}

impl BusInfo {
    pub const fn empty() -> Self {
        Self {
            bus_type: 0,
            bus_index: 0,
            state: 0,
            _pad: 0,
            base_addr: 0,
            owner_pid: 0,
            path: [0; 32],
            path_len: 0,
            _reserved: [0; 3],
        }
    }
}

// =============================================================================
// Device Info Structure
// =============================================================================

/// Unified device information
/// All devices are equal - platform devices and bus controllers
/// Layout must match userlib::syscall::DeviceInfo
#[derive(Clone, Copy)]
#[repr(C)]
pub struct DeviceInfo {
    /// Device name (becomes port name with ':' suffix)
    /// e.g., "uart0", "gpio0", "pcie0", "usb0"
    pub name: [u8; MAX_NAME_LEN],
    /// Length of name
    pub name_len: u8,
    /// Device flags (see device_flags module)
    pub flags: u8,
    /// IRQ number (0 = none/unknown)
    pub irq: u16,
    /// MMIO base address
    pub base_addr: u64,
    /// MMIO region size
    pub size: u32,
    /// Reserved
    pub _reserved: [u8; 4],
}

impl DeviceInfo {
    pub const fn empty() -> Self {
        Self {
            name: [0; MAX_NAME_LEN],
            name_len: 0,
            flags: 0,
            irq: 0,
            base_addr: 0,
            size: 0,
            _reserved: [0; 4],
        }
    }

    /// Create a device info with name and flags
    pub fn new(name: &str, flags: u8, base_addr: u64, size: u32, irq: u16) -> Self {
        let mut info = Self::empty();
        let len = name.len().min(MAX_NAME_LEN);
        info.name[..len].copy_from_slice(&name.as_bytes()[..len]);
        info.name_len = len as u8;
        info.flags = flags;
        info.base_addr = base_addr;
        info.size = size;
        info.irq = irq;
        info
    }

    /// Get name as string slice
    pub fn name_str(&self) -> &str {
        core::str::from_utf8(&self.name[..self.name_len as usize]).unwrap_or("")
    }
}
