//! PCIe Subsystem for Microkernel
//!
//! Provides kernel-mediated PCIe access with capability-based security.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────┐
//! │                  User Space                      │
//! │  ┌─────────┐  ┌─────────┐  ┌─────────┐          │
//! │  │ wifid   │  │ nvmed   │  │ netd    │          │
//! │  └────┬────┘  └────┬────┘  └────┬────┘          │
//! │       │            │            │                │
//! │       └────────────┼────────────┘                │
//! │                    │ syscalls                    │
//! ├────────────────────┼────────────────────────────┤
//! │                    ▼                             │
//! │  ┌─────────────────────────────────────┐        │
//! │  │         PCI Subsystem               │        │
//! │  │  ┌──────────┐  ┌──────────────┐     │        │
//! │  │  │ Registry │  │ MSI Allocator│     │        │
//! │  │  └──────────┘  └──────────────┘     │        │
//! │  │  ┌──────────────────────────────┐   │        │
//! │  │  │     PciHost Trait            │   │        │
//! │  │  └──────────────────────────────┘   │        │
//! │  └─────────────────────────────────────┘        │
//! │                    │                             │
//! │                    ▼                             │
//! │  ┌─────────────────────────────────────┐        │
//! │  │    Platform: MT7988A PCIe MAC       │        │
//! │  └─────────────────────────────────────┘        │
//! │                  Kernel                          │
//! └─────────────────────────────────────────────────┘
//! ```
//!
//! ## Syscalls
//!
//! - `pci_config_read(bdf, offset, size)` - Read config space
//! - `pci_config_write(bdf, offset, size, value)` - Write config space
//! - `pci_msi_alloc(bdf, count)` - Allocate MSI vectors
//!
//! Note: Device enumeration and BAR mapping are handled by pcied via IPC.

#![allow(dead_code)]

mod host;
mod device;
mod config;
mod msi;

pub use host::{PciHost, PciHostOps};
pub use device::{PciDevice, PciDeviceRegistry, PciBdf};
pub use msi::MsiAllocator;

use crate::{kinfo, print_direct};

/// Maximum number of PCI devices we track
pub const MAX_PCI_DEVICES: usize = 64;

/// Maximum number of MSI vectors
pub const MAX_MSI_VECTORS: usize = 256;

/// PCI error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PciError {
    /// Device not found
    NotFound,
    /// Permission denied (capability check failed)
    PermissionDenied,
    /// Invalid BDF address
    InvalidBdf,
    /// Invalid BAR number
    InvalidBar,
    /// BAR not memory-mapped
    BarNotMemory,
    /// Out of MSI vectors
    NoMsiVectors,
    /// Device doesn't support MSI
    MsiNotSupported,
    /// Out of memory
    OutOfMemory,
    /// Already mapped
    AlreadyMapped,
    /// Host controller not initialized
    NotInitialized,
}

impl PciError {
    /// Convert to errno-style error code
    pub fn to_errno(self) -> i32 {
        match self {
            PciError::NotFound => -2,          // ENOENT
            PciError::PermissionDenied => -1,  // EPERM
            PciError::InvalidBdf => -22,       // EINVAL
            PciError::InvalidBar => -22,       // EINVAL
            PciError::BarNotMemory => -22,     // EINVAL
            PciError::NoMsiVectors => -12,     // ENOMEM
            PciError::MsiNotSupported => -95,  // EOPNOTSUPP
            PciError::OutOfMemory => -12,      // ENOMEM
            PciError::AlreadyMapped => -16,    // EBUSY
            PciError::NotInitialized => -6,    // ENXIO
        }
    }
}

/// Result type for PCI operations
pub type PciResult<T> = Result<T, PciError>;

/// Global PCI subsystem state
static mut PCI_SUBSYSTEM: Option<PciSubsystem> = None;

/// The PCI subsystem
pub struct PciSubsystem {
    /// Device registry
    registry: PciDeviceRegistry,
    /// MSI allocator
    msi: MsiAllocator,
    /// Host controller (trait object would need alloc, use enum instead)
    host: Option<PciHostImpl>,
}

/// Concrete host implementations (avoid dyn trait)
pub(crate) enum PciHostImpl {
    Mt7988a(Mt7988aPciHost),
}

impl PciSubsystem {
    /// Create new subsystem
    const fn new() -> Self {
        Self {
            registry: PciDeviceRegistry::new(),
            msi: MsiAllocator::new(),
            host: None,
        }
    }

    /// Register a host controller
    pub fn register_host(&mut self, host: PciHostImpl) {
        self.host = Some(host);
    }

    /// Get host operations
    fn host_ops(&self) -> Option<&dyn PciHostOps> {
        match &self.host {
            Some(PciHostImpl::Mt7988a(h)) => Some(h),
            None => None,
        }
    }

    /// Get mutable host operations
    fn host_ops_mut(&mut self) -> Option<&mut dyn PciHostOps> {
        match &mut self.host {
            Some(PciHostImpl::Mt7988a(h)) => Some(h),
            None => None,
        }
    }
}

/// MT7988A-specific PCI host controller
pub struct Mt7988aPciHost {
    /// Base addresses for each port's MAC registers
    port_bases: [usize; 4],
    /// Number of active ports
    port_count: usize,
    /// Port link status
    link_up: [bool; 4],
}

impl Mt7988aPciHost {
    /// Create new MT7988A PCI host
    pub const fn new() -> Self {
        Self {
            port_bases: [0; 4],
            port_count: 0,
            link_up: [false; 4],
        }
    }

    /// Initialize port
    pub fn init_port(&mut self, port: usize, base: usize) {
        if port < 4 {
            self.port_bases[port] = base;
            if port >= self.port_count {
                self.port_count = port + 1;
            }
        }
    }

    /// Set link status
    pub fn set_link_up(&mut self, port: usize, up: bool) {
        if port < 4 {
            self.link_up[port] = up;
        }
    }

    /// Read MAC register
    fn read_mac(&self, port: usize, offset: usize) -> u32 {
        if port >= self.port_count || self.port_bases[port] == 0 {
            return 0xFFFF_FFFF;
        }
        let addr = self.port_bases[port] + offset;
        unsafe { core::ptr::read_volatile(addr as *const u32) }
    }

    /// Write MAC register
    fn write_mac(&self, port: usize, offset: usize, value: u32) {
        if port >= self.port_count || self.port_bases[port] == 0 {
            return;
        }
        let addr = self.port_bases[port] + offset;
        unsafe { core::ptr::write_volatile(addr as *mut u32, value) }
    }
}

impl PciHostOps for Mt7988aPciHost {
    fn port_count(&self) -> usize {
        self.port_count
    }

    fn link_up(&self, port: usize) -> bool {
        if port < 4 {
            self.link_up[port]
        } else {
            false
        }
    }

    fn config_read32(&self, bdf: PciBdf, offset: u16) -> u32 {
        // MT7988A uses TLP-based config access
        // Port is encoded in upper bits of bus number or passed separately
        let port = (bdf.bus >> 4) as usize; // Simple encoding: port in upper nibble

        if port >= self.port_count {
            return 0xFFFF_FFFF;
        }

        // Setup TLP header
        let devfn = ((bdf.device as u32) << 3) | (bdf.function as u32);
        let bus = bdf.bus & 0x0F; // Lower nibble is actual bus

        // CFGNUM register format for MT7988A
        const CFGNUM_FORCE_BYTE_EN: u32 = 1 << 18;
        const CFGNUM_BYTE_EN_SHIFT: u32 = 14;
        const CFGNUM_BUS_SHIFT: u32 = 6;

        let cfgnum = CFGNUM_FORCE_BYTE_EN
            | (0xF << CFGNUM_BYTE_EN_SHIFT)  // All bytes enabled
            | ((bus as u32) << CFGNUM_BUS_SHIFT)
            | (devfn & 0x3F);

        // Write CFGNUM register (offset 0x8)
        self.write_mac(port, 0x8, cfgnum);

        // Read from config space window (offset 0x1000 + reg)
        let cfg_offset = 0x1000 + ((offset as usize) & !0x3);
        self.read_mac(port, cfg_offset)
    }

    fn config_write32(&self, bdf: PciBdf, offset: u16, value: u32) {
        let port = (bdf.bus >> 4) as usize;

        if port >= self.port_count {
            return;
        }

        let devfn = ((bdf.device as u32) << 3) | (bdf.function as u32);
        let bus = bdf.bus & 0x0F;

        const CFGNUM_FORCE_BYTE_EN: u32 = 1 << 18;
        const CFGNUM_BYTE_EN_SHIFT: u32 = 14;
        const CFGNUM_BUS_SHIFT: u32 = 6;

        let cfgnum = CFGNUM_FORCE_BYTE_EN
            | (0xF << CFGNUM_BYTE_EN_SHIFT)
            | ((bus as u32) << CFGNUM_BUS_SHIFT)
            | (devfn & 0x3F);

        self.write_mac(port, 0x8, cfgnum);

        let cfg_offset = 0x1000 + ((offset as usize) & !0x3);
        self.write_mac(port, cfg_offset, value);
    }

    fn bar_info(&self, bdf: PciBdf, bar: u8) -> Option<(u64, u64)> {
        if bar > 5 {
            return None;
        }

        let bar_offset = 0x10 + (bar as u16) * 4;
        let bar_val = self.config_read32(bdf, bar_offset);

        // Check if I/O BAR
        if (bar_val & 1) != 0 {
            return None; // I/O BARs not supported
        }

        // Check BAR type (bits 2:1)
        let is_64bit = ((bar_val >> 1) & 3) == 2;

        // Get address
        let addr_lo = (bar_val & !0xF) as u64;
        let addr = if is_64bit && bar < 5 {
            let bar_hi = self.config_read32(bdf, bar_offset + 4);
            addr_lo | ((bar_hi as u64) << 32)
        } else {
            addr_lo
        };

        // Probe size
        self.config_write32(bdf, bar_offset, 0xFFFF_FFFF);
        let size_mask = self.config_read32(bdf, bar_offset);
        self.config_write32(bdf, bar_offset, bar_val); // Restore

        let size_bits = size_mask & !0xF;
        if size_bits == 0 {
            return Some((addr, 0));
        }

        let size = (!size_bits).wrapping_add(1) & size_bits;
        Some((addr, size as u64))
    }
}

// ============================================================================
// Global API
// ============================================================================

/// Initialize the PCI subsystem
pub fn init() {
    unsafe {
        PCI_SUBSYSTEM = Some(PciSubsystem::new());
    }
    kinfo!("pci", "init_ok");
}

/// Get subsystem reference
fn subsystem() -> Option<&'static PciSubsystem> {
    unsafe { (*core::ptr::addr_of!(PCI_SUBSYSTEM)).as_ref() }
}

/// Get mutable subsystem reference
fn subsystem_mut() -> Option<&'static mut PciSubsystem> {
    unsafe { (*core::ptr::addr_of_mut!(PCI_SUBSYSTEM)).as_mut() }
}

/// Register the MT7988A host controller
pub fn register_mt7988a_host(host: Mt7988aPciHost) {
    let port_count = host.port_count;
    if let Some(pci) = subsystem_mut() {
        pci.register_host(PciHostImpl::Mt7988a(host));
        kinfo!("pci", "host_registered"; ports = port_count as u64);
    }
}

/// Register a discovered device
pub fn register_device(device: PciDevice) -> PciResult<()> {
    if let Some(pci) = subsystem_mut() {
        pci.registry.register(device)
    } else {
        Err(PciError::NotInitialized)
    }
}

/// Find device by vendor/device ID
pub fn find_device(vendor_id: u16, device_id: u16) -> Option<&'static PciDevice> {
    subsystem()?.registry.find_by_id(vendor_id, device_id)
}

/// Find device by BDF
pub fn find_by_bdf(bdf: PciBdf) -> Option<&'static PciDevice> {
    subsystem()?.registry.find_by_bdf(bdf)
}

/// Read config dword
pub fn config_read32(bdf: PciBdf, offset: u16) -> PciResult<u32> {
    let pci = subsystem().ok_or(PciError::NotInitialized)?;
    let host = pci.host_ops().ok_or(PciError::NotInitialized)?;
    Ok(host.config_read32(bdf, offset))
}

/// Write config dword
pub fn config_write32(bdf: PciBdf, offset: u16, value: u32) -> PciResult<()> {
    let pci = subsystem().ok_or(PciError::NotInitialized)?;
    let host = pci.host_ops().ok_or(PciError::NotInitialized)?;
    host.config_write32(bdf, offset, value);
    Ok(())
}

/// Get BAR info
pub fn bar_info(bdf: PciBdf, bar: u8) -> PciResult<(u64, u64)> {
    let pci = subsystem().ok_or(PciError::NotInitialized)?;
    let host = pci.host_ops().ok_or(PciError::NotInitialized)?;
    host.bar_info(bdf, bar).ok_or(PciError::InvalidBar)
}

/// Allocate MSI vector for a device
pub fn msi_alloc(bdf: PciBdf, count: u8) -> PciResult<u32> {
    let pci = subsystem_mut().ok_or(PciError::NotInitialized)?;
    pci.msi.allocate(bdf, count)
}

/// Free MSI vectors for a device
pub fn msi_free(bdf: PciBdf) {
    if let Some(pci) = subsystem_mut() {
        pci.msi.free(bdf);
    }
}

/// Claim ownership of a device for a process
pub fn claim_device(bdf: PciBdf, pid: u32) -> PciResult<()> {
    let pci = subsystem_mut().ok_or(PciError::NotInitialized)?;
    pci.registry.claim(bdf, pid)
}

/// Release ownership of a device
pub fn release_device(bdf: PciBdf, pid: u32) -> PciResult<()> {
    let pci = subsystem_mut().ok_or(PciError::NotInitialized)?;
    pci.registry.release(bdf, pid)
}

/// Release all devices owned by a process (called on process exit)
/// Also frees any MSI vectors allocated to those devices
pub fn release_all_devices(pid: u32) {
    if let Some(pci) = subsystem_mut() {
        let released = pci.registry.release_all(pid);
        // Free MSI vectors for each released device
        for bdf_opt in released.iter() {
            if let Some(bdf) = bdf_opt {
                pci.msi.free(*bdf);
            }
        }
    }
}

/// Get all registered devices
pub fn devices() -> impl Iterator<Item = &'static PciDevice> {
    struct DeviceIter {
        index: usize,
    }

    impl Iterator for DeviceIter {
        type Item = &'static PciDevice;

        fn next(&mut self) -> Option<Self::Item> {
            let pci = subsystem()?;
            while self.index < MAX_PCI_DEVICES {
                let i = self.index;
                self.index += 1;
                if let Some(dev) = pci.registry.get(i) {
                    return Some(dev);
                }
            }
            None
        }
    }

    DeviceIter { index: 0 }
}

/// Print all registered devices
pub fn list_devices() {
    print_direct!("[pci] Registered devices:\n");
    for dev in devices() {
        print_direct!("  {:02x}:{:02x}.{} [{:04x}:{:04x}] {}\n",
            dev.bdf.bus, dev.bdf.device, dev.bdf.function,
            dev.vendor_id, dev.device_id,
            dev.class_name());
    }
}
