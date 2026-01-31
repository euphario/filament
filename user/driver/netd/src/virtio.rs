//! Virtio 1.0+ PCI Transport
//!
//! Capability discovery, virtqueue management, and device initialization
//! for virtio PCI devices. Reusable across future virtio device types.

use userlib::mmio::{MmioRegion, DmaPool};
use userlib::ipc::PciDevice;
use userlib::{uinfo, uerror};

// =============================================================================
// Virtio PCI Capability Types (virtio 1.0 spec §4.1.4)
// =============================================================================

const VIRTIO_PCI_CAP_COMMON_CFG: u8 = 1;
const VIRTIO_PCI_CAP_NOTIFY_CFG: u8 = 2;
const VIRTIO_PCI_CAP_ISR_CFG: u8 = 3;
const VIRTIO_PCI_CAP_DEVICE_CFG: u8 = 4;

// PCI capability header offsets
const PCI_CAP_VNDR: u8 = 0x09; // Vendor-specific capability

// Virtio PCI capability structure offsets (within the capability)
const VIRTIO_PCI_CAP_CFG_TYPE: u16 = 3;
const VIRTIO_PCI_CAP_BAR: u16 = 4;
const VIRTIO_PCI_CAP_OFFSET: u16 = 8;
const VIRTIO_PCI_CAP_LENGTH: u16 = 12;

// Notify capability has an extra field
const VIRTIO_PCI_NOTIFY_CAP_MULT: u16 = 16;

// =============================================================================
// Virtio Common Configuration (virtio 1.0 spec §4.1.4.3)
// =============================================================================

// Offsets within the common configuration structure
const VIRTIO_COMMON_DFSELECT: usize = 0x00;   // u32: device feature select
const VIRTIO_COMMON_DF: usize = 0x04;         // u32: device features
const VIRTIO_COMMON_GFSELECT: usize = 0x08;   // u32: driver feature select
const VIRTIO_COMMON_GF: usize = 0x0C;         // u32: driver features
const VIRTIO_COMMON_MSIX: usize = 0x10;       // u16: config MSIX vector
const VIRTIO_COMMON_NUMQ: usize = 0x12;       // u16: num queues
const VIRTIO_COMMON_STATUS: usize = 0x14;     // u8: device status
const VIRTIO_COMMON_CFGGEN: usize = 0x15;     // u8: config generation
const VIRTIO_COMMON_Q_SELECT: usize = 0x16;   // u16: queue select
const VIRTIO_COMMON_Q_SIZE: usize = 0x18;     // u16: queue size
const VIRTIO_COMMON_Q_MSIX: usize = 0x1A;     // u16: queue MSIX vector
const VIRTIO_COMMON_Q_ENABLE: usize = 0x1C;   // u16: queue enable
const VIRTIO_COMMON_Q_NOTIFY_OFF: usize = 0x1E; // u16: queue notify offset
const VIRTIO_COMMON_Q_DESC_LO: usize = 0x20;  // u32: descriptor table low
const VIRTIO_COMMON_Q_DESC_HI: usize = 0x24;  // u32: descriptor table high
const VIRTIO_COMMON_Q_AVAIL_LO: usize = 0x28; // u32: available ring low
const VIRTIO_COMMON_Q_AVAIL_HI: usize = 0x2C; // u32: available ring high
const VIRTIO_COMMON_Q_USED_LO: usize = 0x30;  // u32: used ring low
const VIRTIO_COMMON_Q_USED_HI: usize = 0x34;  // u32: used ring high

// Device status bits
pub const VIRTIO_STATUS_ACKNOWLEDGE: u8 = 1;
pub const VIRTIO_STATUS_DRIVER: u8 = 2;
pub const VIRTIO_STATUS_FEATURES_OK: u8 = 8;
pub const VIRTIO_STATUS_DRIVER_OK: u8 = 4;
pub const VIRTIO_STATUS_FAILED: u8 = 128;

// =============================================================================
// Virtqueue Descriptor Flags
// =============================================================================

pub const VIRTQ_DESC_F_NEXT: u16 = 1;
pub const VIRTQ_DESC_F_WRITE: u16 = 2;

// =============================================================================
// Virtio PCI Capabilities
// =============================================================================

/// Discovered virtio PCI capability offsets (all relative to the capability BAR)
pub struct VirtioPciCaps {
    pub bar: u8,         // Which BAR the capabilities live in
    pub common_off: u32,
    pub common_len: u32,
    pub notify_off: u32,
    pub notify_mul: u32,
    pub device_off: u32,
    pub device_len: u32,
    pub isr_off: u32,
}

impl VirtioPciCaps {
    /// Compute the minimum BAR size needed from capability offsets and lengths.
    ///
    /// Finds the maximum (offset + length) across all discovered caps,
    /// then rounds up to page size (4096).
    pub fn min_bar_size(&self) -> u64 {
        let mut max_end: u64 = 0;

        if self.common_len > 0 {
            let end = self.common_off as u64 + self.common_len as u64;
            if end > max_end { max_end = end; }
        }
        if self.notify_off > 0 || self.notify_mul > 0 {
            // Notify region: offset + at least one queue's worth
            // Conservative: offset + notify_mul * QUEUE_COUNT, but we don't know
            // queue count here. Use offset + 4096 as minimum.
            let end = self.notify_off as u64 + 4096;
            if end > max_end { max_end = end; }
        }
        if self.device_len > 0 {
            let end = self.device_off as u64 + self.device_len as u64;
            if end > max_end { max_end = end; }
        }
        if self.isr_off > 0 {
            let end = self.isr_off as u64 + 4; // ISR is typically 4 bytes
            if end > max_end { max_end = end; }
        }

        // Round up to page size
        if max_end == 0 {
            4096
        } else {
            (max_end + 4095) & !4095
        }
    }
}

/// Walk PCI config space to find virtio capability structures.
///
/// Each virtio PCI capability has type 0x09 (vendor-specific) and contains:
/// - cfg_type: which config structure (common, notify, ISR, device)
/// - bar: which BAR the structure lives in
/// - offset: byte offset within the BAR
/// - length: structure size
pub fn discover_caps(pci: &PciDevice) -> Option<VirtioPciCaps> {
    let mut caps = VirtioPciCaps {
        bar: 0,
        common_off: 0,
        common_len: 0,
        notify_off: 0,
        notify_mul: 0,
        device_off: 0,
        device_len: 0,
        isr_off: 0,
    };

    // Read PCI identity
    if let Ok(v) = pci.config_read(0x00, 4) {
        uinfo!("virtio", "pci_id"; vendor = userlib::ulog::hex32(v & 0xFFFF), device = userlib::ulog::hex32((v >> 16) & 0xFFFF));
    }

    // Read status register to check capabilities list present
    let status = match pci.config_read(0x06, 2) {
        Ok(v) => v as u16,
        Err(_) => return None,
    };

    if status & (1 << 4) == 0 {
        uerror!("virtio", "no_caps_list";);
        return None;
    }

    // Get capabilities pointer
    let mut cap_ptr = match pci.config_read(0x34, 1) {
        Ok(v) => (v & 0xFF) as u16,
        Err(_) => return None,
    };

    let mut found_common = false;
    let mut found_notify = false;
    let mut cap_bar: Option<u8> = None; // Lock to first BAR that has common cfg
    let mut iterations = 0;

    while cap_ptr != 0 && iterations < 48 {
        iterations += 1;

        // Read capability ID and next pointer
        let cap_id = match pci.config_read(cap_ptr, 1) {
            Ok(v) => (v & 0xFF) as u8,
            Err(_) => break,
        };
        let cap_next = match pci.config_read(cap_ptr + 1, 1) {
            Ok(v) => (v & 0xFF) as u16,
            Err(_) => break,
        };

        if cap_id == PCI_CAP_VNDR {
            let cfg_type = match pci.config_read(cap_ptr + VIRTIO_PCI_CAP_CFG_TYPE, 1) {
                Ok(v) => (v & 0xFF) as u8,
                Err(_) => { cap_ptr = cap_next; continue; }
            };
            let bar = match pci.config_read(cap_ptr + VIRTIO_PCI_CAP_BAR, 1) {
                Ok(v) => (v & 0xFF) as u8,
                Err(_) => { cap_ptr = cap_next; continue; }
            };
            let offset = match pci.config_read(cap_ptr + VIRTIO_PCI_CAP_OFFSET, 4) {
                Ok(v) => v,
                Err(_) => { cap_ptr = cap_next; continue; }
            };
            let length = match pci.config_read(cap_ptr + VIRTIO_PCI_CAP_LENGTH, 4) {
                Ok(v) => v,
                Err(_) => { cap_ptr = cap_next; continue; }
            };

            uinfo!("virtio", "cap"; cfg_type = cfg_type as u32, bar = bar as u32,
                off = userlib::ulog::hex32(offset), len = userlib::ulog::hex32(length));

            // Lock to the BAR of the first meaningful virtio cap (types 1-4)
            if cap_bar.is_none() && cfg_type >= 1 && cfg_type <= 4 {
                cap_bar = Some(bar);
            }

            if cap_bar == Some(bar) {
                match cfg_type {
                    VIRTIO_PCI_CAP_COMMON_CFG => {
                        caps.common_off = offset;
                        caps.common_len = length;
                        caps.bar = bar;
                        found_common = true;
                    }
                    VIRTIO_PCI_CAP_NOTIFY_CFG => {
                        caps.notify_off = offset;
                        caps.notify_mul = match pci.config_read(cap_ptr + VIRTIO_PCI_NOTIFY_CAP_MULT, 4) {
                            Ok(v) => v,
                            Err(_) => 0,
                        };
                        found_notify = true;
                    }
                    VIRTIO_PCI_CAP_ISR_CFG => {
                        caps.isr_off = offset;
                    }
                    VIRTIO_PCI_CAP_DEVICE_CFG => {
                        caps.device_off = offset;
                        caps.device_len = length;
                    }
                    _ => {}
                }
            }
        }

        cap_ptr = cap_next;
    }

    uinfo!("virtio", "cap_walk_done";
        common = found_common, notify = found_notify, bar = cap_bar.unwrap_or(0xFF) as u32);

    if found_common && found_notify {
        Some(caps)
    } else {
        None
    }
}

// =============================================================================
// Virtqueue Structures (virtio 1.0 spec §2.6)
// =============================================================================

/// Virtqueue descriptor (16 bytes)
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct VirtqDesc {
    pub addr: u64,
    pub len: u32,
    pub flags: u16,
    pub next: u16,
}

/// Used ring element (8 bytes)
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct VirtqUsedElem {
    pub id: u32,
    pub len: u32,
}

// =============================================================================
// Virtqueue
// =============================================================================

/// Virtqueue size (descriptor + avail + used ring) calculation.
///
/// Layout in DMA memory:
/// - Descriptor table: size * 16 bytes
/// - Available ring: 4 + size * 2 + 2 bytes (flags, idx, ring[size], used_event)
/// - Padding to 4096 alignment
/// - Used ring: 4 + size * 8 + 2 bytes (flags, idx, ring[size], avail_event)
fn virtqueue_size(queue_size: u16) -> usize {
    let n = queue_size as usize;
    let desc_size = n * 16;
    let avail_size = 4 + n * 2 + 2;
    let used_size = 4 + n * 8 + 2;
    // Align used ring to 4096
    let avail_end = desc_size + avail_size;
    let used_start = (avail_end + 4095) & !4095;
    used_start + used_size
}

/// A single virtqueue with descriptor table, available ring, and used ring.
pub struct Virtqueue {
    size: u16,
    // Virtual pointers into DMA memory
    desc_base: u64,
    avail_base: u64,
    used_base: u64,
    // Physical addresses (for device programming)
    desc_phys: u64,
    avail_phys: u64,
    used_phys: u64,
    // Free list management
    free_head: u16,
    num_free: u16,
    // Used ring tracking
    last_used_idx: u16,
    // Notification
    notify_offset: u32,
}

impl Virtqueue {
    /// Allocate a virtqueue in the provided DMA pool at the given offset.
    ///
    /// Returns (virtqueue, bytes_consumed).
    pub fn new(
        dma: &DmaPool,
        dma_offset: usize,
        size: u16,
        notify_off: u32,
    ) -> (Self, usize) {
        let n = size as usize;
        let total = virtqueue_size(size);

        let base_virt = dma.vaddr() + dma_offset as u64;
        let base_phys = dma.paddr() + dma_offset as u64;

        // Zero the region
        unsafe {
            core::ptr::write_bytes(base_virt as *mut u8, 0, total);
        }

        let desc_size = n * 16;
        let avail_size = 4 + n * 2 + 2;
        let avail_end = desc_size + avail_size;
        let used_start = (avail_end + 4095) & !4095;

        let desc_base = base_virt;
        let avail_base = base_virt + desc_size as u64;
        let used_base = base_virt + used_start as u64;

        let desc_phys = base_phys;
        let avail_phys = base_phys + desc_size as u64;
        let used_phys = base_phys + used_start as u64;

        // Initialize free list: chain all descriptors
        for i in 0..n {
            let desc = (desc_base + (i * 16) as u64) as *mut VirtqDesc;
            unsafe {
                (*desc).next = if i + 1 < n { (i + 1) as u16 } else { 0 };
            }
        }

        let vq = Virtqueue {
            size,
            desc_base,
            avail_base,
            used_base,
            desc_phys,
            avail_phys,
            used_phys,
            free_head: 0,
            num_free: size,
            last_used_idx: 0,
            notify_offset: notify_off,
        };

        (vq, total)
    }

    /// Physical addresses for device programming
    pub fn desc_phys(&self) -> u64 { self.desc_phys }
    pub fn avail_phys(&self) -> u64 { self.avail_phys }
    pub fn used_phys(&self) -> u64 { self.used_phys }

    /// Add a single buffer to the virtqueue.
    ///
    /// Returns the descriptor index, or None if the queue is full.
    pub fn add_buf(&mut self, addr: u64, len: u32, flags: u16) -> Option<u16> {
        if self.num_free == 0 {
            return None;
        }

        let idx = self.free_head;
        let desc = self.desc_ptr(idx);

        // Advance free list
        self.free_head = unsafe { (*desc).next };
        self.num_free -= 1;

        // Fill descriptor
        unsafe {
            (*desc).addr = addr;
            (*desc).len = len;
            (*desc).flags = flags;
            (*desc).next = 0;
        }

        // Add to available ring
        let avail_idx = self.avail_idx();
        let ring_idx = (avail_idx % self.size) as usize;
        unsafe {
            core::ptr::write_volatile(
                (self.avail_base + 4 + (ring_idx * 2) as u64) as *mut u16,
                idx,
            );
            // Memory barrier before updating index
            core::arch::asm!("dmb ish", options(nostack, preserves_flags));
            // Increment avail idx
            core::ptr::write_volatile(
                (self.avail_base + 2) as *mut u16,
                avail_idx.wrapping_add(1),
            );
        }

        Some(idx)
    }

    /// Notify the device that new buffers are available.
    pub fn kick(&self, bar: &MmioRegion, notify_base: u32) {
        let offset = notify_base as usize + self.notify_offset as usize;
        bar.write16(offset, 0); // queue index doesn't matter for simple notification
    }

    /// Poll the used ring for completed descriptors.
    ///
    /// Returns (descriptor_index, bytes_written) or None if nothing new.
    pub fn poll_used(&mut self) -> Option<(u16, u32)> {
        // Memory barrier before reading used ring
        unsafe { core::arch::asm!("dmb ish", options(nostack, preserves_flags)); }

        let used_idx = self.used_idx();
        if self.last_used_idx == used_idx {
            return None;
        }

        let ring_idx = (self.last_used_idx % self.size) as usize;
        let elem_ptr = (self.used_base + 4 + (ring_idx * 8) as u64) as *const VirtqUsedElem;
        let elem = unsafe { core::ptr::read_volatile(elem_ptr) };

        self.last_used_idx = self.last_used_idx.wrapping_add(1);

        Some((elem.id as u16, elem.len))
    }

    /// Return a descriptor to the free list.
    pub fn reclaim(&mut self, desc_idx: u16) {
        let desc = self.desc_ptr(desc_idx);
        unsafe {
            (*desc).next = self.free_head;
            (*desc).addr = 0;
            (*desc).len = 0;
            (*desc).flags = 0;
        }
        self.free_head = desc_idx;
        self.num_free += 1;
    }

    fn desc_ptr(&self, idx: u16) -> *mut VirtqDesc {
        (self.desc_base + (idx as u64 * 16)) as *mut VirtqDesc
    }

    fn avail_idx(&self) -> u16 {
        unsafe { core::ptr::read_volatile((self.avail_base + 2) as *const u16) }
    }

    fn used_idx(&self) -> u16 {
        unsafe { core::ptr::read_volatile((self.used_base + 2) as *const u16) }
    }
}

// =============================================================================
// Virtio Device Initialization (virtio 1.0 spec §3.1)
// =============================================================================

/// Perform the virtio initialization sequence up to FEATURES_OK.
///
/// After this call, the caller should set up virtqueues, then call
/// `set_driver_ok()` to complete initialization.
///
/// Returns negotiated feature bits on success.
pub fn virtio_init(
    bar: &MmioRegion,
    caps: &VirtioPciCaps,
    driver_features: u64,
) -> Result<u64, ()> {
    let common = caps.common_off as usize;

    // 1. Reset device
    bar.write8(common + VIRTIO_COMMON_STATUS, 0);
    // Brief delay for reset
    for _ in 0..1000 { unsafe { core::arch::asm!("nop"); } }

    // 2. Set ACKNOWLEDGE
    bar.write8(common + VIRTIO_COMMON_STATUS, VIRTIO_STATUS_ACKNOWLEDGE);

    // 3. Set DRIVER
    bar.write8(common + VIRTIO_COMMON_STATUS,
        VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER);

    // 4. Read device features
    bar.write32(common + VIRTIO_COMMON_DFSELECT, 0);
    let dev_features_lo = bar.read32(common + VIRTIO_COMMON_DF) as u64;
    bar.write32(common + VIRTIO_COMMON_DFSELECT, 1);
    let dev_features_hi = bar.read32(common + VIRTIO_COMMON_DF) as u64;
    let dev_features = dev_features_lo | (dev_features_hi << 32);

    // 5. Negotiate features
    let negotiated = dev_features & driver_features;

    // 6. Write driver features
    bar.write32(common + VIRTIO_COMMON_GFSELECT, 0);
    bar.write32(common + VIRTIO_COMMON_GF, negotiated as u32);
    bar.write32(common + VIRTIO_COMMON_GFSELECT, 1);
    bar.write32(common + VIRTIO_COMMON_GF, (negotiated >> 32) as u32);

    // 7. Set FEATURES_OK
    bar.write8(common + VIRTIO_COMMON_STATUS,
        VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER | VIRTIO_STATUS_FEATURES_OK);

    // 8. Verify FEATURES_OK
    let status = bar.read8(common + VIRTIO_COMMON_STATUS);
    if status & VIRTIO_STATUS_FEATURES_OK == 0 {
        bar.write8(common + VIRTIO_COMMON_STATUS, VIRTIO_STATUS_FAILED);
        return Err(());
    }

    Ok(negotiated)
}

/// Set up a virtqueue in the common config.
///
/// Must be called after `virtio_init()` and before `set_driver_ok()`.
/// Returns the queue's notify offset (for kick calculation).
pub fn setup_queue(
    bar: &MmioRegion,
    caps: &VirtioPciCaps,
    queue_idx: u16,
    vq: &Virtqueue,
) {
    let common = caps.common_off as usize;

    // Select queue
    bar.write16(common + VIRTIO_COMMON_Q_SELECT, queue_idx);

    // Write descriptor table address
    bar.write32(common + VIRTIO_COMMON_Q_DESC_LO, vq.desc_phys() as u32);
    bar.write32(common + VIRTIO_COMMON_Q_DESC_HI, (vq.desc_phys() >> 32) as u32);

    // Write available ring address
    bar.write32(common + VIRTIO_COMMON_Q_AVAIL_LO, vq.avail_phys() as u32);
    bar.write32(common + VIRTIO_COMMON_Q_AVAIL_HI, (vq.avail_phys() >> 32) as u32);

    // Write used ring address
    bar.write32(common + VIRTIO_COMMON_Q_USED_LO, vq.used_phys() as u32);
    bar.write32(common + VIRTIO_COMMON_Q_USED_HI, (vq.used_phys() >> 32) as u32);

    // Disable MSIX for this queue (0xFFFF = no vector)
    bar.write16(common + VIRTIO_COMMON_Q_MSIX, 0xFFFF);

    // Enable queue
    bar.write16(common + VIRTIO_COMMON_Q_ENABLE, 1);
}

/// Read the maximum queue size for a given queue index.
pub fn read_queue_size(bar: &MmioRegion, caps: &VirtioPciCaps, queue_idx: u16) -> u16 {
    let common = caps.common_off as usize;
    bar.write16(common + VIRTIO_COMMON_Q_SELECT, queue_idx);
    bar.read16(common + VIRTIO_COMMON_Q_SIZE)
}

/// Read the queue notify offset for a given queue index.
pub fn read_queue_notify_off(bar: &MmioRegion, caps: &VirtioPciCaps, queue_idx: u16) -> u16 {
    let common = caps.common_off as usize;
    bar.write16(common + VIRTIO_COMMON_Q_SELECT, queue_idx);
    bar.read16(common + VIRTIO_COMMON_Q_NOTIFY_OFF)
}

/// Write queue size.
pub fn write_queue_size(bar: &MmioRegion, caps: &VirtioPciCaps, queue_idx: u16, size: u16) {
    let common = caps.common_off as usize;
    bar.write16(common + VIRTIO_COMMON_Q_SELECT, queue_idx);
    bar.write16(common + VIRTIO_COMMON_Q_SIZE, size);
}

/// Complete initialization: set DRIVER_OK.
pub fn set_driver_ok(bar: &MmioRegion, caps: &VirtioPciCaps) {
    let common = caps.common_off as usize;
    bar.write8(common + VIRTIO_COMMON_STATUS,
        VIRTIO_STATUS_ACKNOWLEDGE | VIRTIO_STATUS_DRIVER
        | VIRTIO_STATUS_FEATURES_OK | VIRTIO_STATUS_DRIVER_OK);
}

/// Read number of queues from common config.
pub fn read_num_queues(bar: &MmioRegion, caps: &VirtioPciCaps) -> u16 {
    let common = caps.common_off as usize;
    bar.read16(common + VIRTIO_COMMON_NUMQ)
}
