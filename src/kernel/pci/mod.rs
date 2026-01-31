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
//! The kernel enumerates PCI devices at boot (BAR probing, address allocation)
//! and populates the registry. pcied reads the device list via PciBus objects.

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

/// Simple bump allocator for PCI BAR addresses
struct BarAllocator {
    next: u64,
    end: u64,
}

impl BarAllocator {
    const fn empty() -> Self {
        Self { next: 0, end: 0 }
    }

    fn init(&mut self, base: u64, end: u64) {
        self.next = base;
        self.end = end;
    }

    /// Allocate aligned region of given size. Returns base address or None.
    fn alloc(&mut self, size: u64) -> Option<u64> {
        if size == 0 || self.next == 0 {
            return None;
        }
        let align_mask = size - 1;
        let aligned = (self.next + align_mask) & !align_mask;
        if aligned + size > self.end + 1 {
            return None;
        }
        self.next = aligned + size;
        Some(aligned)
    }
}

/// The PCI subsystem
pub struct PciSubsystem {
    /// Device registry
    registry: PciDeviceRegistry,
    /// MSI allocator
    msi: MsiAllocator,
    /// Host controller (trait object would need alloc, use enum instead)
    host: Option<PciHostImpl>,
    /// BAR address allocator (for ECAM platforms where kernel assigns BARs)
    bar_alloc: BarAllocator,
}

/// Concrete host implementations (avoid dyn trait)
pub(crate) enum PciHostImpl {
    Mt7988a(Mt7988aPciHost),
    Ecam(EcamPciHost),
}

impl PciSubsystem {
    /// Create new subsystem
    const fn new() -> Self {
        Self {
            registry: PciDeviceRegistry::new(),
            msi: MsiAllocator::new(),
            host: None,
            bar_alloc: BarAllocator::empty(),
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
            Some(PciHostImpl::Ecam(h)) => Some(h),
            None => None,
        }
    }

    /// Get mutable host operations
    fn host_ops_mut(&mut self) -> Option<&mut dyn PciHostOps> {
        match &mut self.host {
            Some(PciHostImpl::Mt7988a(h)) => Some(h),
            Some(PciHostImpl::Ecam(h)) => Some(h),
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

/// ECAM-based PCI host controller (QEMU virt, generic PCIe)
///
/// Uses memory-mapped ECAM (Enhanced Configuration Access Mechanism)
/// for PCI configuration space access. The ECAM region is identity-mapped
/// in the kernel's address space.
pub struct EcamPciHost {
    /// Virtual/physical base address of ECAM region (identity-mapped)
    base: usize,
}

impl EcamPciHost {
    /// Create new ECAM host with given base address
    pub const fn new(base: usize) -> Self {
        Self { base }
    }

    /// Calculate config space virtual address for a BDF + register
    #[inline]
    fn config_addr(&self, bdf: PciBdf, offset: u16) -> usize {
        use crate::arch::aarch64::mmu;
        let phys = self.base
            + ((bdf.bus as usize) << 20)
            + ((bdf.device as usize) << 15)
            + ((bdf.function as usize) << 12)
            + ((offset & 0xFFF) as usize);
        if mmu::is_enabled() {
            phys | (mmu::KERNEL_VIRT_BASE as usize)
        } else {
            phys
        }
    }
}

impl PciHostOps for EcamPciHost {
    fn port_count(&self) -> usize {
        1
    }

    fn link_up(&self, _port: usize) -> bool {
        true // ECAM is always "up"
    }

    fn config_read32(&self, bdf: PciBdf, offset: u16) -> u32 {
        let addr = self.config_addr(bdf, offset & !0x3);
        unsafe { core::ptr::read_volatile(addr as *const u32) }
    }

    fn config_write32(&self, bdf: PciBdf, offset: u16, value: u32) {
        let addr = self.config_addr(bdf, offset & !0x3);
        unsafe { core::ptr::write_volatile(addr as *mut u32, value) }
    }

    fn bar_info(&self, bdf: PciBdf, bar: u8) -> Option<(u64, u64)> {
        if bar > 5 {
            return None;
        }

        let bar_offset = 0x10 + (bar as u16) * 4;
        let bar_val = self.config_read32(bdf, bar_offset);

        // Skip I/O BARs
        if (bar_val & 1) != 0 {
            return None;
        }

        let is_64bit = ((bar_val >> 1) & 3) == 2;

        // Get current address
        let addr_lo = (bar_val & !0xF) as u64;
        let addr = if is_64bit && bar < 5 {
            let bar_hi = self.config_read32(bdf, bar_offset + 4);
            addr_lo | ((bar_hi as u64) << 32)
        } else {
            addr_lo
        };

        // Probe size: write all-ones, read back, restore
        self.config_write32(bdf, bar_offset, 0xFFFF_FFFF);
        let size_mask = self.config_read32(bdf, bar_offset);
        self.config_write32(bdf, bar_offset, bar_val); // Restore

        // For 64-bit BARs, also probe the high word for size
        let size64 = if is_64bit && bar < 5 {
            let bar_hi_orig = self.config_read32(bdf, bar_offset + 4);
            self.config_write32(bdf, bar_offset + 4, 0xFFFF_FFFF);
            let size_hi = self.config_read32(bdf, bar_offset + 4);
            self.config_write32(bdf, bar_offset + 4, bar_hi_orig); // Restore
            Some(size_hi)
        } else {
            None
        };

        let size_bits = size_mask & !0xF;
        if size_bits == 0 && size64.map_or(true, |h| h == 0) {
            return Some((addr, 0));
        }

        // Compute size from mask bits (full 64-bit for 64-bit BARs)
        let size = if let Some(hi) = size64 {
            let full_mask = ((hi as u64) << 32) | (size_bits as u64);
            (!full_mask).wrapping_add(1) & full_mask
        } else {
            let s = (!size_bits).wrapping_add(1) & size_bits;
            s as u64
        };
        Some((addr, size))
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
    // Log in main.rs after init returns
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

/// Register an ECAM-based host controller (QEMU virt)
pub fn register_ecam_host(base: usize) {
    // Map the ECAM region into kernel virtual address space.
    // ECAM config space is at most 256MB (256 buses * 32 devices * 8 functions * 4KB).
    // Map the containing 1GB block as device memory.
    let gb_aligned = (base as u64) & !0x3FFF_FFFF;
    crate::arch::aarch64::mmu::map_kernel_device_1gb(gb_aligned);
    kinfo!("pci", "ecam_mapped"; phys = gb_aligned);

    if let Some(pci) = subsystem_mut() {
        pci.register_host(PciHostImpl::Ecam(EcamPciHost::new(base)));
        kinfo!("pci", "ecam_registered"; base = base as u64);
    }
}

/// Enumerate PCI devices and populate the registry.
///
/// Walks all buses/devices/functions, probes BARs, allocates addresses
/// from the MMIO window if needed, walks capability list for MSI/MSI-X,
/// and registers each device in the PciDeviceRegistry.
///
/// Returns the number of devices found.
pub fn enumerate() -> usize {
    use crate::kernel::bus::bus_config;

    let pci = match subsystem_mut() {
        Some(p) => p,
        None => return 0,
    };

    let host = match &pci.host {
        Some(h) => h,
        None => return 0,
    };

    // Initialize BAR allocator from platform config
    if let Some((base, end)) = bus_config().pcie_mmio_window() {
        pci.bar_alloc.init(base, end);
    }

    let mut count = 0;

    // Walk bus 0..4, device 0..32, function 0..8
    for bus in 0..4u8 {
        for device in 0..32u8 {
            // Check function 0 first
            let vendor_device = host_config_read32(host, PciBdf::new(bus, device, 0), 0x00);
            let vendor_id = (vendor_device & 0xFFFF) as u16;
            if vendor_id == 0xFFFF || vendor_id == 0 {
                continue;
            }

            // Check if multi-function
            let header_reg = host_config_read32(host, PciBdf::new(bus, device, 0), 0x0C);
            let header_type = ((header_reg >> 16) & 0xFF) as u8;
            let multi_function = (header_type & 0x80) != 0;
            let max_func = if multi_function { 8u8 } else { 1 };

            for function in 0..max_func {
                let bdf = PciBdf::new(bus, device, function);

                if function > 0 {
                    let vd = host_config_read32(host, bdf, 0x00);
                    if (vd & 0xFFFF) as u16 == 0xFFFF {
                        continue;
                    }
                }

                if let Some(dev) = enumerate_device(host, bdf, &mut pci.bar_alloc) {
                    if pci.registry.register(dev).is_ok() {
                        count += 1;
                        kinfo!("pci", "device_found";
                            bdf_bus = bdf.bus as u64,
                            bdf_dev = bdf.device as u64,
                            bdf_func = bdf.function as u64,
                            vendor = dev.vendor_id as u64,
                            device_id = dev.device_id as u64,
                            bar0 = dev.bar0_addr);
                    }
                }
            }
        }
    }

    count
}

/// Helper: read config via host enum variant (avoids borrow issues with subsystem)
fn host_config_read32(host: &PciHostImpl, bdf: PciBdf, offset: u16) -> u32 {
    match host {
        PciHostImpl::Mt7988a(h) => h.config_read32(bdf, offset),
        PciHostImpl::Ecam(h) => h.config_read32(bdf, offset),
    }
}

fn host_config_write32(host: &PciHostImpl, bdf: PciBdf, offset: u16, value: u32) {
    match host {
        PciHostImpl::Mt7988a(h) => h.config_write32(bdf, offset, value),
        PciHostImpl::Ecam(h) => h.config_write32(bdf, offset, value),
    }
}

fn host_bar_info(host: &PciHostImpl, bdf: PciBdf, bar: u8) -> Option<(u64, u64)> {
    match host {
        PciHostImpl::Mt7988a(h) => h.bar_info(bdf, bar),
        PciHostImpl::Ecam(h) => h.bar_info(bdf, bar),
    }
}

/// Enumerate a single device at the given BDF
fn enumerate_device(host: &PciHostImpl, bdf: PciBdf, bar_alloc: &mut BarAllocator) -> Option<PciDevice> {
    let vendor_device = host_config_read32(host, bdf, 0x00);
    let vendor_id = (vendor_device & 0xFFFF) as u16;
    let device_id = ((vendor_device >> 16) & 0xFFFF) as u16;

    if vendor_id == 0xFFFF || vendor_id == 0 {
        return None;
    }

    let class_rev = host_config_read32(host, bdf, 0x08);
    let revision = (class_rev & 0xFF) as u8;
    let prog_if = ((class_rev >> 8) & 0xFF) as u8;
    let subclass = ((class_rev >> 16) & 0xFF) as u8;
    let base_class = ((class_rev >> 24) & 0xFF) as u8;
    let class_code = ((base_class as u32) << 16) | ((subclass as u32) << 8) | (prog_if as u32);

    let header_reg = host_config_read32(host, bdf, 0x0C);
    let header_type = ((header_reg >> 16) & 0x7F) as u8;

    // Skip bridges (type 1 headers)
    if header_type != 0 {
        return None;
    }

    // Probe and allocate ALL MMIO BARs, record the first one as bar0
    let mut bar0_addr = 0u64;
    let mut bar0_size = 0u64;
    let mut bar_idx = 0u8;
    while bar_idx <= 5 {
        let bar_offset = 0x10 + (bar_idx as u16) * 4;
        let raw = host_config_read32(host, bdf, bar_offset);
        let is_io = (raw & 1) != 0;
        let is_64bit = !is_io && ((raw >> 1) & 0x3) == 2;

        if !is_io {
            if let Some((addr, size)) = host_bar_info(host, bdf, bar_idx) {
                if size > 0 {
                    let final_addr = if addr == 0 {
                        // BAR unassigned — allocate from MMIO window
                        if let Some(allocated) = bar_alloc.alloc(size) {
                            let bar_value = (allocated as u32) | (raw & 0xF);
                            host_config_write32(host, bdf, bar_offset, bar_value);
                            if is_64bit {
                                host_config_write32(host, bdf, bar_offset + 4, 0);
                            }
                            allocated
                        } else {
                            0
                        }
                    } else {
                        addr
                    };

                    // Ensure MEMORY_SPACE is enabled in command register
                    if final_addr != 0 {
                        let cmd = host_config_read32(host, bdf, 0x04);
                        if (cmd & 0x0002) == 0 {
                            host_config_write32(host, bdf, 0x04, cmd | 0x0002);
                        }
                    }

                    // Record first MMIO BAR as bar0
                    if final_addr != 0 && bar0_addr == 0 {
                        bar0_addr = final_addr;
                        bar0_size = size;
                    }
                }
            }
        }

        bar_idx += if is_64bit { 2 } else { 1 };
    }

    // Walk capability list for MSI (0x05) and MSI-X (0x11)
    let mut msi_cap = 0u8;
    let mut msix_cap = 0u8;
    let status = (host_config_read32(host, bdf, 0x04) >> 16) as u16;
    if (status & 0x10) != 0 {
        // Capabilities list present
        let mut cap_ptr = (host_config_read32(host, bdf, 0x34) & 0xFC) as u8;
        for _ in 0..48 {
            if cap_ptr == 0 {
                break;
            }
            let cap_val = host_config_read32(host, bdf, cap_ptr as u16);
            let cap_id = (cap_val & 0xFF) as u8;
            match cap_id {
                0x05 => msi_cap = cap_ptr,
                0x11 => msix_cap = cap_ptr,
                _ => {}
            }
            cap_ptr = ((cap_val >> 8) & 0xFC) as u8;
        }
    }

    Some(PciDevice {
        bdf,
        vendor_id,
        device_id,
        class_code,
        revision,
        header_type,
        is_bridge: false,
        bar0_addr,
        bar0_size,
        msi_cap,
        msix_cap,
        owner_pid: 0,
    })
}

/// Get the number of devices in the registry
pub fn device_count() -> usize {
    match subsystem() {
        Some(pci) => pci.registry.len(),
        None => 0,
    }
}

/// Execute a closure with access to all devices in the registry
pub fn with_devices<F, R>(f: F) -> R
where
    F: FnOnce(&PciDeviceRegistry) -> R,
{
    let pci = subsystem().expect("PCI subsystem not initialized");
    f(&pci.registry)
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
