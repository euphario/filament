//! MT7996 WFDMA (WiFi DMA) Driver
//!
//! The MT7996 uses WFDMA for host-to-device communication including:
//! - Firmware download (FWDL queue)
//! - MCU commands (WM/WA queues)
//! - TX/RX data
//!
//! ## DMA Ring Structure
//!
//! Each ring consists of:
//! - Base address register (ring descriptor array physical address)
//! - Count register (number of descriptors)
//! - CPU index (host write pointer)
//! - DMA index (device read pointer)
//!
//! ## Register Layout (per ring at base + queue_offset)
//!
//! - 0x00: Base address (low 32 bits)
//! - 0x04: Max count
//! - 0x08: CPU index
//! - 0x0C: DMA index
//! - 0x10: Base address (high 32 bits, if 64-bit)

use crate::device::Mt7996Device;
use userlib::syscall;

/// WFDMA base addresses (relative to BAR0)
pub mod wfdma {
    /// WFDMA0 base
    pub const WFDMA0_BASE: u32 = 0xd4000;
    /// WFDMA1 base
    pub const WFDMA1_BASE: u32 = 0xd5000;
    /// WFDMA0 on PCIe1 (for dual-band)
    pub const WFDMA0_PCIE1_BASE: u32 = 0xd8000;

    /// Global config register
    pub const GLO_CFG: u32 = 0x208;
    /// Reset register
    pub const RST: u32 = 0x100;
    /// Reset DTX pointer
    pub const RST_DTX_PTR: u32 = 0x20c;
    /// Interrupt source
    pub const INT_SRC: u32 = 0x200;
    /// Interrupt mask
    pub const INT_MASK: u32 = 0x204;
    /// MCU command register
    pub const MCU_CMD: u32 = 0x1f0;

    /// MCU queue ring base offset (from WFDMA base)
    pub const MCU_RING_BASE: u32 = 0x300;
    /// RX queue ring base offset
    pub const RX_RING_BASE: u32 = 0x500;

    /// Ring register offsets
    pub const RING_BASE_LO: u32 = 0x00;
    pub const RING_MAX_CNT: u32 = 0x04;
    pub const RING_CPU_IDX: u32 = 0x08;
    pub const RING_DMA_IDX: u32 = 0x0c;
    pub const RING_BASE_HI: u32 = 0x10;

    /// Ring size for firmware download
    pub const FWDL_RING_SIZE: u32 = 128;
}

/// MCU queue IDs
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum McuQueue {
    /// Firmware download queue
    Fwdl = 0,
    /// WM (WiFi Manager) queue
    Wm = 1,
    /// WA (WiFi Agent) queue
    Wa = 2,
}

/// MCU command flags (for MCU_CMD register)
pub mod mcu_cmd {
    /// Stop DMA
    pub const STOP_DMA: u32 = 1 << 0;
    /// Reset done
    pub const RESET_DONE: u32 = 1 << 1;
    /// Normal state (firmware running)
    pub const NORMAL_STATE: u32 = 1 << 5;
    /// Wake event
    pub const WAKE_EVENT: u32 = 1 << 2;
}

/// Global config register bits
pub mod glo_cfg {
    /// Enable TX DMA
    pub const TX_DMA_EN: u32 = 1 << 0;
    /// Enable RX DMA
    pub const RX_DMA_EN: u32 = 1 << 2;
    /// TX DMA busy
    pub const TX_DMA_BUSY: u32 = 1 << 1;
    /// RX DMA busy
    pub const RX_DMA_BUSY: u32 = 1 << 3;
    /// Byte swap
    pub const BYTE_SWAP: u32 = 1 << 4;
    /// DMA descriptor size (0 = 8 DWORDs, 1 = 16 DWORDs)
    pub const DESC_SIZE_16DW: u32 = 1 << 8;
    /// Omit TX info
    pub const OMIT_TX_INFO: u32 = 1 << 28;
    /// Omit RX info
    pub const OMIT_RX_INFO: u32 = 1 << 27;
}

/// DMA descriptor (TXD - Transmit Descriptor)
/// 8 DWORDs = 32 bytes
#[repr(C, align(16))]
#[derive(Clone, Copy, Default)]
pub struct TxDesc {
    /// DMA buffer address (low 32 bits)
    pub buf_addr_lo: u32,
    /// DMA buffer address (high 32 bits) + flags
    pub buf_addr_hi: u32,
    /// Token ID and flags
    pub token: u32,
    /// Reserved
    pub reserved: u32,
    /// TX descriptor info 0
    pub info0: u32,
    /// TX descriptor info 1
    pub info1: u32,
    /// TX descriptor info 2
    pub info2: u32,
    /// TX descriptor info 3
    pub info3: u32,
}

impl TxDesc {
    /// Create a new descriptor for firmware download
    pub fn for_firmware(buf_phys: u64, len: usize, last: bool) -> Self {
        let mut desc = Self::default();
        desc.buf_addr_lo = buf_phys as u32;
        desc.buf_addr_hi = (buf_phys >> 32) as u32;

        // Set length in info0 (bits 15:0)
        desc.info0 = (len as u32) & 0xFFFF;

        // Set last segment flag if needed
        if last {
            desc.info0 |= 1 << 30; // LS (Last Segment) bit
        }

        desc
    }
}

/// DMA ring for firmware download
pub struct FwdlRing {
    /// Virtual address of descriptor ring
    pub desc_vaddr: u64,
    /// Physical address of descriptor ring
    pub desc_paddr: u64,
    /// Number of descriptors
    pub size: u32,
    /// Current CPU index (write pointer)
    pub cpu_idx: u32,
}

impl FwdlRing {
    /// Create a new firmware download ring
    pub fn new() -> Option<Self> {
        let size = wfdma::FWDL_RING_SIZE;
        let desc_size = (size as usize) * core::mem::size_of::<TxDesc>();

        // Allocate DMA memory for descriptors
        let mut paddr: u64 = 0;
        let vaddr = syscall::mmap_dma(desc_size, &mut paddr);
        if vaddr < 0 {
            return None;
        }

        // Zero the descriptor ring
        let desc_ptr = vaddr as *mut u8;
        unsafe {
            core::ptr::write_bytes(desc_ptr, 0, desc_size);
        }

        Some(Self {
            desc_vaddr: vaddr as u64,
            desc_paddr: paddr,
            size,
            cpu_idx: 0,
        })
    }

    /// Get a descriptor by index
    pub fn get_desc(&self, idx: u32) -> &mut TxDesc {
        let descs = self.desc_vaddr as *mut TxDesc;
        unsafe { &mut *descs.add(idx as usize) }
    }

    /// Advance CPU index
    pub fn advance(&mut self) {
        self.cpu_idx = (self.cpu_idx + 1) % self.size;
    }
}

impl Drop for FwdlRing {
    fn drop(&mut self) {
        let desc_size = (self.size as usize) * core::mem::size_of::<TxDesc>();
        syscall::munmap(self.desc_vaddr, desc_size);
    }
}

/// WFDMA controller
pub struct Wfdma<'a> {
    dev: &'a Mt7996Device,
    fwdl_ring: Option<FwdlRing>,
}

impl<'a> Wfdma<'a> {
    /// Create new WFDMA controller
    pub fn new(dev: &'a Mt7996Device) -> Self {
        Self {
            dev,
            fwdl_ring: None,
        }
    }

    /// Read WFDMA0 register
    fn read(&self, offset: u32) -> u32 {
        self.dev.read32_raw(wfdma::WFDMA0_BASE + offset)
    }

    /// Write WFDMA0 register
    fn write(&self, offset: u32, value: u32) {
        self.dev.write32_raw(wfdma::WFDMA0_BASE + offset, value);
    }

    /// Initialize WFDMA for firmware download
    pub fn init(&mut self) -> bool {
        userlib::println!("  Initializing WFDMA...");

        // Disable DMA first
        self.write(wfdma::GLO_CFG, 0);

        // Wait for DMA to stop
        for _ in 0..100 {
            let cfg = self.read(wfdma::GLO_CFG);
            if (cfg & (glo_cfg::TX_DMA_BUSY | glo_cfg::RX_DMA_BUSY)) == 0 {
                break;
            }
            syscall::yield_now();
        }

        // Allocate firmware download ring
        self.fwdl_ring = FwdlRing::new();
        if self.fwdl_ring.is_none() {
            userlib::println!("  Failed to allocate FWDL ring");
            return false;
        }

        let ring = self.fwdl_ring.as_ref().unwrap();
        userlib::println!("  FWDL ring: vaddr=0x{:x}, paddr=0x{:x}, size={}",
            ring.desc_vaddr, ring.desc_paddr, ring.size);

        // Configure FWDL ring (MCU queue 0)
        let ring_base = wfdma::MCU_RING_BASE + (McuQueue::Fwdl as u32) * 0x10;

        self.write(ring_base + wfdma::RING_BASE_LO, ring.desc_paddr as u32);
        self.write(ring_base + wfdma::RING_BASE_HI, (ring.desc_paddr >> 32) as u32);
        self.write(ring_base + wfdma::RING_MAX_CNT, ring.size);
        self.write(ring_base + wfdma::RING_CPU_IDX, 0);

        // Enable TX DMA with 8-DWORD descriptors
        let cfg = glo_cfg::TX_DMA_EN;
        self.write(wfdma::GLO_CFG, cfg);

        userlib::println!("  WFDMA initialized");
        true
    }

    /// Send firmware data via FWDL queue
    pub fn send_firmware_chunk(&mut self, data_paddr: u64, len: usize, last: bool) -> bool {
        // Get cpu_idx before mutable borrow
        let cpu_idx = match self.fwdl_ring.as_ref() {
            Some(r) => r.cpu_idx,
            None => return false,
        };

        // Fill descriptor and advance index
        let new_cpu_idx = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };

            let desc = ring.get_desc(cpu_idx);
            *desc = TxDesc::for_firmware(data_paddr, len, last);

            // Memory barrier
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

            // Advance CPU index
            ring.advance();
            ring.cpu_idx
        };

        // Update CPU index register (rings doorbell)
        let ring_base = wfdma::MCU_RING_BASE + (McuQueue::Fwdl as u32) * 0x10;
        self.write(ring_base + wfdma::RING_CPU_IDX, new_cpu_idx);

        true
    }

    /// Wait for DMA to complete (DMA index catches up to CPU index)
    pub fn wait_dma_done(&self, timeout_ms: u32) -> bool {
        let ring = match self.fwdl_ring.as_ref() {
            Some(r) => r,
            None => return false,
        };

        let ring_base = wfdma::MCU_RING_BASE + (McuQueue::Fwdl as u32) * 0x10;

        for _ in 0..(timeout_ms * 10) {
            let dma_idx = self.read(ring_base + wfdma::RING_DMA_IDX);
            if dma_idx == ring.cpu_idx {
                return true;
            }
            syscall::yield_now();
        }

        userlib::println!("  DMA timeout: cpu_idx={}, dma_idx={}",
            ring.cpu_idx, self.read(ring_base + wfdma::RING_DMA_IDX));
        false
    }

    /// Check MCU state
    pub fn mcu_state(&self) -> u32 {
        self.read(wfdma::MCU_CMD)
    }

    /// Set MCU command
    pub fn set_mcu_cmd(&self, cmd: u32) {
        self.write(wfdma::MCU_CMD, cmd);
    }
}
