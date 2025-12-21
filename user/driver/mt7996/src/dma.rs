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
//! Linux struct mt76_queue_regs is exactly 16 bytes:
//! - 0x00: desc_base  (descriptor ring base address, 32-bit only!)
//! - 0x04: ring_size  (number of descriptors)
//! - 0x08: cpu_idx    (host write pointer)
//! - 0x0C: dma_idx    (device read pointer)
//!
//! IMPORTANT: There is NO 0x10 offset for base_hi! Offset 0x10 would be the
//! next ring's desc_base. Our physical addresses fit in 32 bits anyway.

use crate::device::Mt7996Device;
use userlib::{syscall, trace, trace_err};

/// Cache line size (64 bytes for Cortex-A73)
const CACHE_LINE: usize = 64;

/// Flush (clean) a cache line to memory.
/// Use before DMA reads (hardware reading data that CPU wrote).
#[inline]
fn flush_cache_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc cvac, {}", in(reg) addr, options(nostack, preserves_flags));
    }
}

/// Data Synchronization Barrier
#[inline]
fn dsb() {
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// Flush a buffer to memory (for DMA reads by hardware)
#[inline]
fn flush_buffer(addr: u64, size: usize) {
    let start = addr & !(CACHE_LINE as u64 - 1);
    let end = (addr + size as u64 + CACHE_LINE as u64 - 1) & !(CACHE_LINE as u64 - 1);
    let mut a = start;
    while a < end {
        flush_cache_line(a);
        a += CACHE_LINE as u64;
    }
    dsb();
}

/// Address remapping registers (for accessing addresses > 0x100000)
pub mod remap {
    /// CONN_BUS_CR_VON_BASE
    pub const CONN_BUS_CR_VON_BASE: u32 = 0x155000;

    /// HIF_REMAP_L1 register offset (MT7990)
    pub const HIF_REMAP_L1_OFFSET: u32 = 0x8;
    /// Full HIF_REMAP_L1 address
    pub const HIF_REMAP_L1: u32 = CONN_BUS_CR_VON_BASE + HIF_REMAP_L1_OFFSET;

    /// Where remapped region appears in BAR space (MT7990)
    pub const HIF_REMAP_BASE_L1: u32 = 0x40000;

    /// L1 remap granularity (256KB)
    pub const L1_REMAP_MASK: u32 = 0x3ffff;
    pub const L1_BASE_MASK: u32 = !L1_REMAP_MASK;

    /// CBTOP remap for MT7990 (for addresses in 0x70000000 range)
    pub const HIF_REMAP_CBTOP: u32 = 0x1f6554;
    /// Where CBTOP remapped region appears in BAR space
    pub const HIF_REMAP_BASE_CBTOP: u32 = 0x1c0000;
    /// CBTOP remap granularity (64KB)
    pub const CBTOP_REMAP_MASK: u32 = 0xffff;

    /// CBTOP1 physical address range start
    pub const CBTOP1_PHY_START: u32 = 0x70000000;

    /// Wireless subsystem reset register (in CBTOP1 range, needs CBTOP remap)
    pub const WF_SUBSYS_RST: u32 = 0x70028600;
}

/// WFDMA base addresses (relative to BAR0)
pub mod wfdma {
    /// WFDMA0 base
    pub const WFDMA0_BASE: u32 = 0xd4000;
    /// WFDMA1 base
    pub const WFDMA1_BASE: u32 = 0xd5000;
    /// WFDMA0 on PCIe1 (for dual-band)
    pub const WFDMA0_PCIE1_BASE: u32 = 0xd8000;

    /// Reset register
    pub const RST: u32 = 0x100;
    /// Reset bits
    pub const RST_LOGIC_RST: u32 = 1 << 4;
    pub const RST_DMASHDL_ALL_RST: u32 = 1 << 5;

    /// Busy enable register
    pub const BUSY_ENA: u32 = 0x13c;
    pub const BUSY_ENA_TX_FIFO0: u32 = 1 << 0;
    pub const BUSY_ENA_TX_FIFO1: u32 = 1 << 1;
    pub const BUSY_ENA_RX_FIFO: u32 = 1 << 2;

    /// MCU command register
    pub const MCU_CMD: u32 = 0x1f0;

    /// Interrupt source
    pub const INT_SRC: u32 = 0x200;
    /// Interrupt mask
    pub const INT_MASK: u32 = 0x204;

    /// Global config register
    pub const GLO_CFG: u32 = 0x208;

    /// Reset DTX pointer
    pub const RST_DTX_PTR: u32 = 0x20c;

    /// Interrupt delay configuration registers (CORRECT addresses!)
    /// NOT 0x20c/0x230 - those are wrong and clobber other regs!
    pub const PRI_DLY_INT_CFG0: u32 = 0x2f0;
    pub const PRI_DLY_INT_CFG1: u32 = 0x2f4;
    pub const PRI_DLY_INT_CFG2: u32 = 0x2f8;

    /// Extended global config registers
    pub const GLO_CFG_EXT0: u32 = 0x2b0;
    pub const GLO_CFG_EXT1: u32 = 0x2b4;

    /// GLO_CFG_EXT0 bits
    pub const GLO_CFG_EXT0_RX_WB_RXD: u32 = 1 << 18;

    /// RX pause threshold registers
    pub const PAUSE_RX_Q_45_TH: u32 = 0x268;
    pub const PAUSE_RX_Q_67_TH: u32 = 0x26c;
    pub const PAUSE_RX_Q_89_TH: u32 = 0x270;

    /// WFDMA Extended CSR base
    pub const EXT_CSR_BASE: u32 = 0xd7000;
    /// HIF misc status register (relative to BAR0)
    pub const EXT_CSR_HIF_MISC: u32 = 0xd7044;
    /// BUSY bit in HIF_MISC
    pub const EXT_CSR_HIF_MISC_BUSY: u32 = 1 << 0;

    /// GLO_CFG_EXT1 bits
    pub const GLO_CFG_EXT1_TX_FCTRL_MODE: u32 = 1 << 28;

    /// MCU queue ring base offset (from WFDMA base)
    pub const MCU_RING_BASE: u32 = 0x300;
    /// RX queue ring base offset
    pub const RX_RING_BASE: u32 = 0x500;

    /// RX MCU queue ID
    pub const RXQ_MCU: u32 = 0;
    /// RX MCU ring size
    pub const RX_MCU_RING_SIZE: u32 = 64;

    /// Ring register offsets (16 bytes per ring, NO base_hi!)
    pub const RING_BASE: u32 = 0x00;     // desc_base (32-bit address)
    pub const RING_MAX_CNT: u32 = 0x04;  // ring_size
    pub const RING_CPU_IDX: u32 = 0x08;  // cpu_idx
    pub const RING_DMA_IDX: u32 = 0x0c;  // dma_idx
    // Note: offset 0x10 is the NEXT ring's desc_base, not base_hi!

    /// Ring size for firmware download
    pub const FWDL_RING_SIZE: u32 = 128;

    /// Extended control register offsets (relative to WFDMA base)
    /// MCU TX queue prefetch control
    pub const MCUQ_EXT_CTRL_BASE: u32 = 0x600;
    /// Data TX queue prefetch control
    pub const TXQ_EXT_CTRL_BASE: u32 = 0x640;
    /// RX queue prefetch control
    pub const RXQ_EXT_CTRL_BASE: u32 = 0x680;

    /// For backwards compatibility
    pub const EXT_CTRL_BASE: u32 = MCUQ_EXT_CTRL_BASE;

    /// Prefetch depth for MT7990 MCU queues
    pub const PREFETCH_DEPTH_MCU_MT7990: u32 = 4;
}

/// MT7996 MCU TX queue IDs (from mt76/mt7996 driver)
/// These are the actual hardware queue indices, NOT the mt76_mcuq_id enum!
/// Ring registers are at MCU_RING_BASE + queue_id * 0x10
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McuQueue {
    /// Firmware download queue - TXQ 16
    Fwdl = 16,
    /// WM (WiFi Manager) queue - TXQ 17
    Wm = 17,
    /// WA (WiFi Agent) queue - TXQ 20
    Wa = 20,
}

impl McuQueue {
    /// Get ring register base offset from WFDMA base
    #[inline]
    pub const fn ring_offset(self) -> u32 {
        wfdma::MCU_RING_BASE + (self as u32) * 0x10
    }

    /// Get prefetch control register offset from WFDMA base
    #[inline]
    pub const fn ext_ctrl_offset(self) -> u32 {
        wfdma::MCUQ_EXT_CTRL_BASE + (self as u32) * 4
    }
}

/// Typed TX queue index (prevents mixing with RX queues or raw offsets)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxQ(pub u8);

impl TxQ {
    pub const FWDL: TxQ = TxQ(16);
    pub const WM: TxQ = TxQ(17);
    pub const WA: TxQ = TxQ(20);

    /// Get ring register base offset from WFDMA base
    #[inline]
    pub const fn ring_offset(self) -> u32 {
        wfdma::MCU_RING_BASE + (self.0 as u32) * 0x10
    }
}

/// Typed RX queue index
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RxQ(pub u8);

impl RxQ {
    pub const MCU: RxQ = RxQ(0);

    /// Get ring register base offset from WFDMA base
    #[inline]
    pub const fn ring_offset(self) -> u32 {
        wfdma::RX_RING_BASE + (self.0 as u32) * 0x10
    }

    /// Get prefetch control register offset from WFDMA base
    #[inline]
    pub const fn ext_ctrl_offset(self) -> u32 {
        wfdma::RXQ_EXT_CTRL_BASE + (self.0 as u32) * 4
    }
}

/// WFDMA controller state machine
///
/// Explicit states prevent "works only if you call X before Y" bugs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WfdmaState {
    /// Not initialized - just created
    Uninit,
    /// Reset complete, ready for ring allocation
    Reset,
    /// Rings allocated and configured
    RingsConfigured,
    /// DMA engines enabled, ready for transfers
    Ready,
    /// Error state - needs full reset
    Error,
}

/// MCU command flags (for MCU_CMD register at WFDMA0+0x1f0)
pub mod mcu_cmd {
    /// Stop DMA
    pub const STOP_DMA: u32 = 1 << 2;
    /// Reset done
    pub const RESET_DONE: u32 = 1 << 3;
    /// Recovery done
    pub const RECOVERY_DONE: u32 = 1 << 4;
    /// Normal state (firmware running)
    pub const NORMAL_STATE: u32 = 1 << 5;
    /// Error mask (bits 5-1)
    pub const ERROR_MASK: u32 = 0x3e;
    /// WM watchdog timer
    pub const WM_WDT: u32 = 1 << 30;
    /// WA watchdog timer
    pub const WA_WDT: u32 = 1 << 31;
    /// Combined WDT mask
    pub const WDT_MASK: u32 = WM_WDT | WA_WDT;
}

/// MCU interrupt event register (at 0x2108)
pub mod mcu_int {
    /// MCU interrupt event register address
    pub const EVENT: u32 = 0x2108;
    /// DMA stopped
    pub const DMA_STOPPED: u32 = 1 << 0;
    /// DMA init
    pub const DMA_INIT: u32 = 1 << 1;
    /// Reset done
    pub const RESET_DONE: u32 = 1 << 3;
}

/// MCU command IDs
pub mod mcu {
    pub const CMD_FW_SCATTER: u8 = 0xee;
    pub const CMD_TARGET_ADDRESS_LEN_REQ: u8 = 0x01;
    pub const CMD_FW_START_REQ: u8 = 0x02;
    pub const CMD_PATCH_START_REQ: u8 = 0x05;
    pub const CMD_PATCH_SEM_CTRL: u8 = 0x11;
    pub const CMD_PATCH_FINISH_REQ: u8 = 0x07;

    /// MCU packet type
    pub const PKT_ID: u8 = 0xa0;

    /// Source to destination index (host to MCU)
    pub const S2D_H2N: u8 = 0x00;

    /// Patch semaphore operations
    pub const PATCH_SEM_GET: u32 = 0x01;
    pub const PATCH_SEM_RELEASE: u32 = 0x00;

    /// Patch load mode
    pub const PATCH_MODE_ENCRYPTED: u32 = 0x01;
    pub const PATCH_MODE_NORMAL: u32 = 0x00;
}

/// Patch header structure (mt76_connac2_patch_hdr)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PatchHeader {
    pub build_date: [u8; 16],
    pub platform: [u8; 4],
    pub hw_sw_ver: u32,  // big-endian
    pub patch_ver: u32,  // big-endian
    pub checksum: u16,   // big-endian
    pub rsv: u16,
    // descriptor
    pub desc_patch_ver: u32,  // big-endian
    pub desc_subsys: u32,     // big-endian
    pub desc_feature: u32,    // big-endian
    pub desc_n_region: u32,   // big-endian
    pub desc_crc: u32,
    pub desc_rsv: [u32; 11],
}

impl PatchHeader {
    /// Get number of regions
    pub fn n_region(&self) -> u32 {
        u32::from_be(self.desc_n_region)
    }
}

/// Patch section structure (mt76_connac2_patch_sec)
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PatchSection {
    pub sec_type: u32,  // big-endian
    pub offs: u32,      // big-endian
    pub size: u32,      // big-endian
    // info union
    pub addr: u32,          // big-endian - target address
    pub len: u32,           // big-endian - data length
    pub sec_key_idx: u32,   // big-endian - security key index
    pub align_len: u32,     // big-endian
    pub rsv: [u32; 9],
}

impl PatchSection {
    /// Get target address
    pub fn target_addr(&self) -> u32 {
        u32::from_be(self.addr)
    }

    /// Get data length
    pub fn data_len(&self) -> u32 {
        u32::from_be(self.len)
    }

    /// Get section offset in file
    pub fn offset(&self) -> u32 {
        u32::from_be(self.offs)
    }

    /// Get section size in file
    pub fn sec_size(&self) -> u32 {
        u32::from_be(self.size)
    }
}

/// TXD field definitions
pub mod txd {
    /// TXD0: TX_BYTES field (bits 15:0)
    pub const TXD0_TX_BYTES_MASK: u32 = 0xFFFF;
    /// TXD0: PKT_FMT field (bits 24:23) - 2 = CMD
    pub const TXD0_PKT_FMT_CMD: u32 = 2 << 23;
    /// TXD0: Q_IDX field (bits 31:25)
    pub const TXD0_Q_IDX_SHIFT: u32 = 25;

    /// TXD1: HDR_FORMAT field (bits 17:16) - 1 = CMD
    pub const TXD1_HDR_FORMAT_CMD: u32 = 1 << 16;
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
    /// Byte swap - MUST be cleared for little-endian descriptors!
    pub const BYTE_SWAP: u32 = 1 << 4;
    /// DMA descriptor size (0 = 8 DWORDs, 1 = 16 DWORDs)
    pub const DESC_SIZE_16DW: u32 = 1 << 8;
    /// Omit RX info in prefetch
    pub const OMIT_RX_INFO_PFET2: u32 = 1 << 21;
    /// External enable (required for firmware loading!)
    pub const EXT_EN: u32 = 1 << 26;
    /// Omit RX info
    pub const OMIT_RX_INFO: u32 = 1 << 27;
    /// Omit TX info
    pub const OMIT_TX_INFO: u32 = 1 << 28;
}

/// Ring descriptor control field bits
pub mod dma_ctl {
    /// Segment data length 0 (bits 29:16)
    pub const SD_LEN0_SHIFT: u32 = 16;
    pub const SD_LEN0_MASK: u32 = 0x3FFF << 16;
    /// Last segment indicator for buffer 0
    pub const LAST_SEC0: u32 = 1 << 30;
    /// DMA done flag
    pub const DMA_DONE: u32 = 1 << 31;
}

/// DMA ring descriptor (mt76_desc format - 16 bytes)
/// This is what goes in the DMA ring to point to the actual packet buffer
#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct TxDesc {
    /// Buffer 0 address (low 32 bits)
    pub buf0: u32,
    /// Control: SD_LEN0(29:16), LAST_SEC0(30), DMA_DONE(31)
    pub ctrl: u32,
    /// Buffer 1 address / high bits
    pub buf1: u32,
    /// Info field
    pub info: u32,
}

/// MCU command TXD header (prepended to MCU command packets)
/// 32 bytes TXD + 28 bytes MCU fields = 60 bytes total
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct McuTxd {
    /// TXD header (8 DWORDs)
    pub txd: [u32; 8],
    /// Payload length (excluding this header)
    pub len: u16,
    /// Port/Queue ID
    pub pq_id: u16,
    /// Command ID
    pub cid: u8,
    /// Packet type (0xa0 for MCU)
    pub pkt_type: u8,
    /// Set/Query flag
    pub set_query: u8,
    /// Sequence number
    pub seq: u8,
    /// Reserved
    pub uc_d2b0_rev: u8,
    /// Extended command ID
    pub ext_cid: u8,
    /// Source to destination index
    pub s2d_index: u8,
    /// Extended CID ack
    pub ext_cid_ack: u8,
    /// Reserved
    pub rsv: [u32; 5],
}

impl Default for McuTxd {
    fn default() -> Self {
        Self {
            txd: [0; 8],
            len: 0,
            pq_id: 0,
            cid: 0,
            pkt_type: 0,
            set_query: 0,
            seq: 0,
            uc_d2b0_rev: 0,
            ext_cid: 0,
            s2d_index: 0,
            ext_cid_ack: 0,
            rsv: [0; 5],
        }
    }
}

impl McuTxd {
    /// Build MCU TXD for a command
    /// queue_id: Hardware TX queue index (16 for FWDL, 17 for WM, 20 for WA)
    pub fn new(cmd_id: u8, payload_len: usize, seq: u8, queue_id: u32) -> Self {
        let total_len = core::mem::size_of::<McuTxd>() + payload_len;
        let mut txd = Self::default();

        // TXD0: length | PKT_FMT=CMD | Q_IDX
        // Q_IDX must match the hardware queue we're submitting to!
        txd.txd[0] = ((total_len as u32) & txd::TXD0_TX_BYTES_MASK)
            | txd::TXD0_PKT_FMT_CMD
            | (queue_id << txd::TXD0_Q_IDX_SHIFT);

        // TXD1: HDR_FORMAT=CMD
        txd.txd[1] = txd::TXD1_HDR_FORMAT_CMD;

        // MCU fields
        txd.len = (payload_len as u16).to_le();
        txd.pq_id = 0; // Will be set by hardware
        txd.cid = cmd_id;
        txd.pkt_type = mcu::PKT_ID;
        txd.seq = seq;
        txd.s2d_index = mcu::S2D_H2N;

        txd
    }
}

impl TxDesc {
    /// Create a new descriptor for MCU command
    pub fn for_firmware(buf_phys: u64, len: usize, last: bool) -> Self {
        let mut desc = Self::default();

        // Buffer 0 address
        desc.buf0 = buf_phys as u32;

        // Control: length in bits 29:16, last segment flag in bit 30
        desc.ctrl = ((len as u32) << dma_ctl::SD_LEN0_SHIFT) & dma_ctl::SD_LEN0_MASK;
        if last {
            desc.ctrl |= dma_ctl::LAST_SEC0;
        }

        // Buffer 1 (high bits of 64-bit address)
        desc.buf1 = (buf_phys >> 32) as u32;

        // Info field (unused for simple TX)
        desc.info = 0;

        desc
    }
}

/// Size of MCU command buffer (header + max payload)
const MCU_CMD_BUF_SIZE: usize = core::mem::size_of::<McuTxd>() + 4096;

/// DMA ring for firmware download
pub struct FwdlRing {
    /// Virtual address of descriptor ring
    pub desc_vaddr: u64,
    /// Physical address of descriptor ring
    pub desc_paddr: u64,
    /// Virtual address of command buffer
    pub cmd_vaddr: u64,
    /// Physical address of command buffer
    pub cmd_paddr: u64,
    /// Number of descriptors
    pub size: u32,
    /// Current CPU index (write pointer)
    pub cpu_idx: u32,
    /// Command sequence number
    pub seq: u8,
}

impl FwdlRing {
    /// Create a new firmware download ring
    pub fn new() -> Option<Self> {
        let size = wfdma::FWDL_RING_SIZE;
        let desc_size = (size as usize) * core::mem::size_of::<TxDesc>();

        // Allocate DMA memory for descriptors
        let mut desc_paddr: u64 = 0;
        let desc_vaddr = syscall::mmap_dma(desc_size, &mut desc_paddr);
        if desc_vaddr < 0 {
            return None;
        }

        // Zero the descriptor ring
        unsafe {
            core::ptr::write_bytes(desc_vaddr as *mut u8, 0, desc_size);
        }

        // Allocate DMA memory for command buffer
        let mut cmd_paddr: u64 = 0;
        let cmd_vaddr = syscall::mmap_dma(MCU_CMD_BUF_SIZE, &mut cmd_paddr);
        if cmd_vaddr < 0 {
            syscall::munmap(desc_vaddr as u64, desc_size);
            return None;
        }

        Some(Self {
            desc_vaddr: desc_vaddr as u64,
            desc_paddr,
            cmd_vaddr: cmd_vaddr as u64,
            cmd_paddr,
            size,
            cpu_idx: 0,
            seq: 0,
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

    /// Get next sequence number
    pub fn next_seq(&mut self) -> u8 {
        let s = self.seq;
        self.seq = self.seq.wrapping_add(1);
        s
    }

    /// Build FW_SCATTER command in buffer
    /// Returns physical address and total length of command
    pub fn build_fw_scatter(&mut self, fw_data: &[u8]) -> (u64, usize) {
        let payload_len = fw_data.len();
        let header = McuTxd::new(mcu::CMD_FW_SCATTER, payload_len, self.next_seq(), McuQueue::Fwdl as u32);

        // Write header to command buffer
        let buf_ptr = self.cmd_vaddr as *mut McuTxd;
        unsafe {
            core::ptr::write(buf_ptr, header);
        }

        // Copy firmware data after header
        let data_ptr = (self.cmd_vaddr as usize + core::mem::size_of::<McuTxd>()) as *mut u8;
        unsafe {
            core::ptr::copy_nonoverlapping(fw_data.as_ptr(), data_ptr, payload_len);
        }

        let total_len = core::mem::size_of::<McuTxd>() + payload_len;

        // Flush command buffer to RAM so DMA can see it
        flush_buffer(self.cmd_vaddr, total_len);

        (self.cmd_paddr, total_len)
    }

    /// Build TARGET_ADDRESS_LEN_REQ command
    /// This tells the MCU where to load the firmware data
    /// Returns physical address and total length of command
    pub fn build_init_download(&mut self, addr: u32, len: u32, mode: u32) -> (u64, usize) {
        // Payload is 3 u32s: addr, len, mode
        let payload_len = 12;
        let header = McuTxd::new(mcu::CMD_TARGET_ADDRESS_LEN_REQ, payload_len, self.next_seq(), McuQueue::Fwdl as u32);

        // Write header to command buffer
        let buf_ptr = self.cmd_vaddr as *mut McuTxd;
        unsafe {
            core::ptr::write(buf_ptr, header);
        }

        // Write payload (addr, len, mode) as little-endian
        let data_ptr = (self.cmd_vaddr as usize + core::mem::size_of::<McuTxd>()) as *mut u32;
        unsafe {
            core::ptr::write(data_ptr, addr.to_le());
            core::ptr::write(data_ptr.add(1), len.to_le());
            core::ptr::write(data_ptr.add(2), mode.to_le());
        }

        let total_len = core::mem::size_of::<McuTxd>() + payload_len;

        // Flush command buffer to RAM so DMA can see it
        flush_buffer(self.cmd_vaddr, total_len);

        (self.cmd_paddr, total_len)
    }

    /// Build PATCH_SEM_CTRL command (acquire/release semaphore)
    pub fn build_patch_sem_ctrl(&mut self, get: bool) -> (u64, usize) {
        // Payload is 1 u32: operation (GET=1, RELEASE=0)
        let payload_len = 4;
        let header = McuTxd::new(mcu::CMD_PATCH_SEM_CTRL, payload_len, self.next_seq(), McuQueue::Fwdl as u32);

        let buf_ptr = self.cmd_vaddr as *mut McuTxd;
        unsafe {
            core::ptr::write(buf_ptr, header);
        }

        let data_ptr = (self.cmd_vaddr as usize + core::mem::size_of::<McuTxd>()) as *mut u32;
        let op = if get { mcu::PATCH_SEM_GET } else { mcu::PATCH_SEM_RELEASE };
        unsafe {
            core::ptr::write(data_ptr, op.to_le());
        }

        let total_len = core::mem::size_of::<McuTxd>() + payload_len;

        // Flush command buffer to RAM so DMA can see it
        flush_buffer(self.cmd_vaddr, total_len);

        (self.cmd_paddr, total_len)
    }

    /// Build PATCH_FINISH_REQ command
    pub fn build_patch_finish(&mut self) -> (u64, usize) {
        // No payload
        let header = McuTxd::new(mcu::CMD_PATCH_FINISH_REQ, 0, self.next_seq(), McuQueue::Fwdl as u32);

        let buf_ptr = self.cmd_vaddr as *mut McuTxd;
        unsafe {
            core::ptr::write(buf_ptr, header);
        }

        let total_len = core::mem::size_of::<McuTxd>();

        // Flush command buffer to RAM so DMA can see it
        flush_buffer(self.cmd_vaddr, total_len);

        (self.cmd_paddr, total_len)
    }
}

impl Drop for FwdlRing {
    fn drop(&mut self) {
        let desc_size = (self.size as usize) * core::mem::size_of::<TxDesc>();
        syscall::munmap(self.desc_vaddr, desc_size);
        syscall::munmap(self.cmd_vaddr, MCU_CMD_BUF_SIZE);
    }
}

/// RX buffer size for MCU events
const RX_BUF_SIZE: usize = 2048;

/// Simple RX ring for MCU responses
pub struct RxRing {
    /// Virtual address of descriptor ring
    pub desc_vaddr: u64,
    /// Physical address of descriptor ring
    pub desc_paddr: u64,
    /// Virtual address of RX buffers
    pub buf_vaddr: u64,
    /// Physical address of RX buffers
    pub buf_paddr: u64,
    /// Number of descriptors
    pub size: u32,
}

impl RxRing {
    /// Create a new RX ring
    pub fn new(size: u32) -> Option<Self> {
        let desc_size = (size as usize) * core::mem::size_of::<TxDesc>();
        let buf_total = (size as usize) * RX_BUF_SIZE;

        // Allocate DMA memory for descriptors
        let mut desc_paddr: u64 = 0;
        let desc_vaddr = syscall::mmap_dma(desc_size, &mut desc_paddr);
        if desc_vaddr < 0 {
            return None;
        }

        // Allocate DMA memory for RX buffers
        let mut buf_paddr: u64 = 0;
        let buf_vaddr = syscall::mmap_dma(buf_total, &mut buf_paddr);
        if buf_vaddr < 0 {
            syscall::munmap(desc_vaddr as u64, desc_size);
            return None;
        }

        // Zero everything
        unsafe {
            core::ptr::write_bytes(desc_vaddr as *mut u8, 0, desc_size);
            core::ptr::write_bytes(buf_vaddr as *mut u8, 0, buf_total);
        }

        // Initialize descriptors to point to RX buffers
        let descs = desc_vaddr as *mut TxDesc;
        for i in 0..size {
            let buf_addr = buf_paddr + (i as u64 * RX_BUF_SIZE as u64);
            unsafe {
                let desc = &mut *descs.add(i as usize);
                desc.buf0 = buf_addr as u32;
                desc.buf1 = (buf_addr >> 32) as u32;
                // Set buffer length in ctrl field for RX
                desc.ctrl = (RX_BUF_SIZE as u32) << dma_ctl::SD_LEN0_SHIFT;
            }
        }

        // Flush all RX descriptors to RAM so DMA can see them
        flush_buffer(desc_vaddr as u64, desc_size);

        Some(Self {
            desc_vaddr: desc_vaddr as u64,
            desc_paddr,
            buf_vaddr: buf_vaddr as u64,
            buf_paddr,
            size,
        })
    }
}

impl Drop for RxRing {
    fn drop(&mut self) {
        let desc_size = (self.size as usize) * core::mem::size_of::<TxDesc>();
        let buf_total = (self.size as usize) * RX_BUF_SIZE;
        syscall::munmap(self.desc_vaddr, desc_size);
        syscall::munmap(self.buf_vaddr, buf_total);
    }
}

/// WFDMA controller
pub struct Wfdma<'a> {
    dev: &'a Mt7996Device,
    fwdl_ring: Option<FwdlRing>,
    rx_mcu_ring: Option<RxRing>,
    state: WfdmaState,
}

impl<'a> Wfdma<'a> {
    /// Create new WFDMA controller
    pub fn new(dev: &'a Mt7996Device) -> Self {
        Self {
            dev,
            fwdl_ring: None,
            rx_mcu_ring: None,
            state: WfdmaState::Uninit,
        }
    }

    /// Get current state
    pub fn state(&self) -> WfdmaState {
        self.state
    }

    /// Check if ready for transfers
    pub fn is_ready(&self) -> bool {
        self.state == WfdmaState::Ready
    }

    /// Transition to error state
    fn set_error(&mut self) {
        self.state = WfdmaState::Error;
    }

    /// Read WFDMA0 register (relative to WFDMA0_BASE)
    fn read(&self, offset: u32) -> u32 {
        self.dev.read32_raw(wfdma::WFDMA0_BASE + offset)
    }

    /// Write WFDMA0 register (relative to WFDMA0_BASE)
    fn write(&self, offset: u32, value: u32) {
        self.dev.write32_raw(wfdma::WFDMA0_BASE + offset, value);
    }

    /// Read register at absolute BAR offset
    fn read_bar(&self, offset: u32) -> u32 {
        self.dev.read32_raw(offset)
    }

    /// Write register at absolute BAR offset
    fn write_bar(&self, offset: u32, value: u32) {
        self.dev.write32_raw(offset, value);
    }

    /// Read register via L1 remapping (for addresses > 0x100000 but < 0x70000000)
    fn read_remap_l1(&self, addr: u32) -> u32 {
        // Calculate offset within remap window and base address
        let offset = addr & remap::L1_REMAP_MASK;
        let base = addr & remap::L1_BASE_MASK;

        // Write base to HIF_REMAP_L1 register
        self.write_bar(remap::HIF_REMAP_L1, base >> 18);

        // Read back to ensure write completes
        let _ = self.read_bar(remap::HIF_REMAP_L1);

        // Read via remap window
        self.read_bar(remap::HIF_REMAP_BASE_L1 + offset)
    }

    /// Write register via L1 remapping (for addresses > 0x100000 but < 0x70000000)
    fn write_remap_l1(&self, addr: u32, value: u32) {
        let offset = addr & remap::L1_REMAP_MASK;
        let base = addr & remap::L1_BASE_MASK;

        // Write base to HIF_REMAP_L1 register
        self.write_bar(remap::HIF_REMAP_L1, base >> 18);

        // Read back to ensure write completes
        let _ = self.read_bar(remap::HIF_REMAP_L1);

        // Write via remap window
        self.write_bar(remap::HIF_REMAP_BASE_L1 + offset, value);
    }

    /// Read register via CBTOP remapping (for addresses in 0x70000000 range - MT7990)
    fn read_remap_cbtop(&self, addr: u32) -> u32 {
        // CBTOP uses 64KB windows with 16-bit offset/base
        let offset = addr & remap::CBTOP_REMAP_MASK;
        let base = (addr >> 16) & 0xffff;

        // Write base to HIF_REMAP_CBTOP register
        self.write_bar(remap::HIF_REMAP_CBTOP, base);

        // Read back to ensure write completes
        let readback = self.read_bar(remap::HIF_REMAP_CBTOP);
        trace!(DMA, "  CBTOP remap: wrote 0x{:04x}, readback 0x{:08x}", base, readback);

        // Read via remap window
        self.read_bar(remap::HIF_REMAP_BASE_CBTOP + offset)
    }

    /// Write register via CBTOP remapping (for addresses in 0x70000000 range - MT7990)
    fn write_remap_cbtop(&self, addr: u32, value: u32) {
        // CBTOP uses 64KB windows with 16-bit offset/base
        let offset = addr & remap::CBTOP_REMAP_MASK;
        let base = (addr >> 16) & 0xffff;

        // Write base to HIF_REMAP_CBTOP register
        self.write_bar(remap::HIF_REMAP_CBTOP, base);

        // Read back to ensure write completes
        let _ = self.read_bar(remap::HIF_REMAP_CBTOP);

        // Write via remap window
        self.write_bar(remap::HIF_REMAP_BASE_CBTOP + offset, value);
    }

    /// Perform wireless subsystem reset
    pub fn wfsys_reset(&self) {
        trace!(DMA, "Performing wfsys_reset...");

        // WF_SUBSYS_RST is in CBTOP1 range (0x70000000+), use CBTOP remap
        let val = self.read_remap_cbtop(remap::WF_SUBSYS_RST);
        trace!(DMA, "  WF_SUBSYS_RST initial: 0x{:08x}", val);

        // Set reset bit
        self.write_remap_cbtop(remap::WF_SUBSYS_RST, val | 0x1);

        // Wait 20ms
        for _ in 0..200 {
            syscall::yield_now();
        }

        // Clear reset bit
        self.write_remap_cbtop(remap::WF_SUBSYS_RST, val & !0x1);

        // Wait 20ms
        for _ in 0..200 {
            syscall::yield_now();
        }

        let val_after = self.read_remap_cbtop(remap::WF_SUBSYS_RST);
        trace!(DMA, "  WF_SUBSYS_RST after: 0x{:08x}", val_after);
    }

    /// Initialize WFDMA for firmware download
    pub fn init(&mut self) -> bool {
        trace!(DMA, "Initializing WFDMA...");

        // Step 0: Check and clear WDT (watchdog) bits
        // If WDT is set, the device experienced a firmware crash
        let mcu_cmd = self.read(wfdma::MCU_CMD);
        trace!(DMA, "MCU_CMD initial: 0x{:08x}", mcu_cmd);

        if (mcu_cmd & mcu_cmd::WDT_MASK) != 0 {
            trace!(DMA, "WDT detected, clearing...");
            // Clear WDT bits by writing them
            self.write(wfdma::MCU_CMD, mcu_cmd & !mcu_cmd::WDT_MASK);
            let mcu_cmd_after = self.read(wfdma::MCU_CMD);
            trace!(DMA, "MCU_CMD after WDT clear: 0x{:08x}", mcu_cmd_after);
        }

        // Step 1: WFDMA reset sequence (reference code: set → delay → clear)
        // We must END with RST=0 so WFDMA is OUT of reset before configuring rings
        trace!(DMA, "Resetting WFDMA...");
        let rst_bits = wfdma::RST_LOGIC_RST | wfdma::RST_DMASHDL_ALL_RST;

        // First disable DMA engines
        let glo = self.read(wfdma::GLO_CFG);
        self.write(wfdma::GLO_CFG, glo & !(glo_cfg::TX_DMA_EN | glo_cfg::RX_DMA_EN | glo_cfg::EXT_EN));

        // Assert reset (set bits)
        self.write(wfdma::RST, rst_bits);
        trace!(DMA, "RST after set: 0x{:08x}", self.read(wfdma::RST));

        // Delay
        for _ in 0..50_000 { core::hint::spin_loop(); }

        // Deassert reset (clear bits) - WFDMA should now be operational
        self.write(wfdma::RST, 0);
        trace!(DMA, "RST after clear: 0x{:08x}", self.read(wfdma::RST));

        // State: Reset complete
        self.state = WfdmaState::Reset;

        // Check GLO_CFG initial state (don't enable yet - Linux enables AFTER ring config)
        let glo_val = self.read(wfdma::GLO_CFG);
        trace!(DMA, "GLO_CFG initial: 0x{:08x}", glo_val);

        // Allocate firmware download ring
        self.fwdl_ring = FwdlRing::new();
        if self.fwdl_ring.is_none() {
            trace!(DMA, "Failed to allocate FWDL ring");
            self.set_error();
            return false;
        }

        let ring = self.fwdl_ring.as_ref().unwrap();
        trace!(DMA, "FWDL ring: vaddr=0x{:x}, paddr=0x{:x}, size={}",
            ring.desc_vaddr, ring.desc_paddr, ring.size);

        // CRITICAL: Verify DMA addresses are below 4GB (MT7996 has 32-bit DMA on some paths)
        if (ring.desc_paddr >> 32) != 0 {
            trace!(DMA, "ERROR: FWDL ring paddr 0x{:x} is above 4GB!", ring.desc_paddr);
            self.set_error();
            return false;
        }

        // Configure FWDL ring
        let ring_base = wfdma::MCU_RING_BASE + (McuQueue::Fwdl as u32) * 0x10;

        // Debug: dump ring register space before config
        trace!(DMA, "Ring@WFDMA+0x{:x} before write:", ring_base);
        trace!(DMA, "  regs: {:08x} {:08x} {:08x} {:08x}",
            self.read(ring_base + 0x00), self.read(ring_base + 0x04),
            self.read(ring_base + 0x08), self.read(ring_base + 0x0c));

        // Write ring configuration (Linux order: base, size, then indices)
        // DEBUG: Print exact addresses being written
        let addr_base = wfdma::WFDMA0_BASE + ring_base + wfdma::RING_BASE;
        let addr_cnt = wfdma::WFDMA0_BASE + ring_base + wfdma::RING_MAX_CNT;
        trace!(DMA, "Writing ring config:");
        trace!(DMA, "  base addr 0x{:x} <- 0x{:08x}", addr_base, ring.desc_paddr as u32);
        trace!(DMA, "  cnt addr 0x{:x} <- {}", addr_cnt, ring.size);

        self.write(ring_base + wfdma::RING_BASE, ring.desc_paddr as u32);
        self.write(ring_base + wfdma::RING_MAX_CNT, ring.size);
        self.write(ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(ring_base + wfdma::RING_DMA_IDX, 0);

        // Immediate readback to verify
        let immediate_base = self.read(ring_base + wfdma::RING_BASE);
        let immediate_cnt = self.read(ring_base + wfdma::RING_MAX_CNT);
        trace!(DMA, "  Immediate readback: base=0x{:08x}, cnt={}", immediate_base, immediate_cnt);

        // Verify ring configuration
        trace!(DMA, "Ring@WFDMA+0x{:x} after write:", ring_base);
        let rb_lo = self.read(ring_base + wfdma::RING_BASE);
        let rb_cnt = self.read(ring_base + wfdma::RING_MAX_CNT);
        let rb_cpu = self.read(ring_base + wfdma::RING_CPU_IDX);
        let rb_dma = self.read(ring_base + wfdma::RING_DMA_IDX);
        trace!(DMA, "  base=0x{:08x}, cnt={}, cpu={}, dma={}",
            rb_lo, rb_cnt, rb_cpu, rb_dma);

        // Step 1: Reset DTX pointer (as Linux driver does)
        trace!(DMA, "Resetting DTX pointer...");
        self.write(wfdma::RST_DTX_PTR, 0xFFFFFFFF);

        // Step 2: Set BUSY_ENA for TX FIFO (critical for DMA operation!)
        trace!(DMA, "Setting BUSY_ENA...");
        let busy_val = wfdma::BUSY_ENA_TX_FIFO0 | wfdma::BUSY_ENA_TX_FIFO1 | wfdma::BUSY_ENA_RX_FIFO;
        self.write(wfdma::BUSY_ENA, busy_val);
        trace!(DMA, "BUSY_ENA: 0x{:08x}", self.read(wfdma::BUSY_ENA));

        // Step 3: Configure interrupt delay (disable it)
        // IMPORTANT: Use correct registers 0x2f0/0x2f4/0x2f8, NOT 0x20c (that's RST_DTX_PTR!)
        trace!(DMA, "Configuring interrupt delay (off)...");
        self.write(wfdma::PRI_DLY_INT_CFG0, 0);
        self.write(wfdma::PRI_DLY_INT_CFG1, 0);
        self.write(wfdma::PRI_DLY_INT_CFG2, 0);

        // Step 4: Configure extended GLO_CFG registers
        trace!(DMA, "Setting GLO_CFG_EXT0/EXT1...");
        self.write(wfdma::GLO_CFG_EXT0, wfdma::GLO_CFG_EXT0_RX_WB_RXD);
        self.write(wfdma::GLO_CFG_EXT1, wfdma::GLO_CFG_EXT1_TX_FCTRL_MODE);
        trace!(DMA, "GLO_CFG_EXT0: 0x{:08x}", self.read(wfdma::GLO_CFG_EXT0));
        trace!(DMA, "GLO_CFG_EXT1: 0x{:08x}", self.read(wfdma::GLO_CFG_EXT1));

        // Configure prefetch for MCU TX queues (required for DMA to work!)
        // Prefetch format: (ring_offset << 16) | depth
        // MT7996 uses: FWDL=16, WM=17, WA=20
        trace!(DMA, "Configuring MCU TX queue prefetch...");
        let mcu_queues = [
            (McuQueue::Fwdl as u32, "FWDL"),
            (McuQueue::Wm as u32, "WM"),
            (McuQueue::Wa as u32, "WA"),
        ];
        for (q, name) in mcu_queues.iter() {
            let ring_offset = wfdma::MCU_RING_BASE + q * 0x10;  // Ring register offset
            let prefetch_val = (ring_offset << 16) | wfdma::PREFETCH_DEPTH_MCU_MT7990;
            let ext_ctrl = wfdma::MCUQ_EXT_CTRL_BASE + q * 4;
            self.write(ext_ctrl, prefetch_val);
            trace!(DMA, "  MCU_TX{} ({}) @WFDMA+0x{:x}: prefetch=0x{:08x}", q, name, ext_ctrl, prefetch_val);
        }

        // Configure prefetch for RX MCU queue
        {
            let ring_offset = wfdma::RX_RING_BASE + wfdma::RXQ_MCU * 0x10;
            let prefetch_val = (ring_offset << 16) | wfdma::PREFETCH_DEPTH_MCU_MT7990;
            let ext_ctrl = wfdma::RXQ_EXT_CTRL_BASE + wfdma::RXQ_MCU * 4;
            self.write(ext_ctrl, prefetch_val);
            trace!(DMA, "  RX_MCU @WFDMA+0x{:x}: prefetch=0x{:08x}", ext_ctrl, prefetch_val);
        }

        // Step 4: Set up RX MCU queue for receiving MCU responses
        // This is required for the MCU to send responses to our commands
        trace!(DMA, "Setting up RX MCU queue...");
        self.rx_mcu_ring = RxRing::new(wfdma::RX_MCU_RING_SIZE);
        if self.rx_mcu_ring.is_none() {
            trace!(DMA, "Failed to allocate RX MCU ring");
            self.set_error();
            return false;
        }

        let rx_ring = self.rx_mcu_ring.as_ref().unwrap();
        trace!(DMA, "RX MCU ring: vaddr=0x{:x}, paddr=0x{:x}, size={}",
            rx_ring.desc_vaddr, rx_ring.desc_paddr, rx_ring.size);

        // CRITICAL: Verify DMA addresses are below 4GB
        if (rx_ring.desc_paddr >> 32) != 0 {
            trace!(DMA, "ERROR: RX ring paddr 0x{:x} is above 4GB!", rx_ring.desc_paddr);
            self.set_error();
            return false;
        }

        // State: Rings configured
        self.state = WfdmaState::RingsConfigured;

        // Configure RX MCU ring registers
        let rx_ring_base = wfdma::RX_RING_BASE + (wfdma::RXQ_MCU) * 0x10;
        trace!(DMA, "RX ring@WFDMA+0x{:x} before write:", rx_ring_base);
        trace!(DMA, "  regs: {:08x} {:08x} {:08x} {:08x}",
            self.read(rx_ring_base + 0x00), self.read(rx_ring_base + 0x04),
            self.read(rx_ring_base + 0x08), self.read(rx_ring_base + 0x0c));

        // Write RX ring configuration (Linux order: base, size, then indices)
        // Note: No BASE_HI write - it would corrupt next ring's desc_base!
        self.write(rx_ring_base + wfdma::RING_BASE, rx_ring.desc_paddr as u32);
        self.write(rx_ring_base + wfdma::RING_MAX_CNT, rx_ring.size);
        self.write(rx_ring_base + wfdma::RING_CPU_IDX, 0);
        self.write(rx_ring_base + wfdma::RING_DMA_IDX, 0);

        // Verify RX ring configuration
        trace!(DMA, "RX ring@WFDMA+0x{:x} after write:", rx_ring_base);
        let rx_rb_lo = self.read(rx_ring_base + wfdma::RING_BASE);
        let rx_rb_cnt = self.read(rx_ring_base + wfdma::RING_MAX_CNT);
        let rx_rb_cpu = self.read(rx_ring_base + wfdma::RING_CPU_IDX);
        let rx_rb_dma = self.read(rx_ring_base + wfdma::RING_DMA_IDX);
        trace!(DMA, "  base=0x{:08x}, cnt={}, cpu={}, dma={}",
            rx_rb_lo, rx_rb_cnt, rx_rb_cpu, rx_rb_dma);

        // For RX, we need to set CPU_IDX to size-1 to indicate all buffers are available
        // (Linux does this with mt76_queue_update_read_idx)
        self.write(rx_ring_base + wfdma::RING_CPU_IDX, rx_ring.size - 1);
        trace!(DMA, "RX ring cpu_idx set to {} (all buffers available)", rx_ring.size - 1);

        // Check MCU_INT_EVENT register
        let int_event = self.read_bar(mcu_int::EVENT);
        trace!(DMA, "MCU_INT_EVENT: 0x{:08x}", int_event);

        // Check and clear INT_SRC (pending interrupts might block DMA)
        let int_src = self.read(wfdma::INT_SRC);
        trace!(DMA, "INT_SRC: 0x{:08x}", int_src);
        if int_src != 0 {
            self.write(wfdma::INT_SRC, int_src); // Write to clear
            trace!(DMA, "INT_SRC after clear: 0x{:08x}", self.read(wfdma::INT_SRC));
        }

        // Check INT_MASK
        let int_mask = self.read(wfdma::INT_MASK);
        trace!(DMA, "INT_MASK: 0x{:08x}", int_mask);

        // Also check the MCU_CMD register state
        let mcu_cmd = self.read(wfdma::MCU_CMD);
        trace!(DMA, "MCU_CMD: 0x{:08x}", mcu_cmd);

        // Try clearing STOP_DMA and setting RESET_DONE to signal MCU we're ready
        let mcu_cmd_new = (mcu_cmd & !mcu_cmd::STOP_DMA) | mcu_cmd::RESET_DONE;
        self.write(wfdma::MCU_CMD, mcu_cmd_new);
        trace!(DMA, "MCU_CMD after write: 0x{:08x}", self.read(wfdma::MCU_CMD));

        // Wait for WFDMA to be idle before enabling (Linux does this!)
        trace!(DMA, "Waiting for WFDMA idle...");
        let mut hif_misc = self.read_bar(wfdma::EXT_CSR_HIF_MISC);
        for i in 0..100 {
            if (hif_misc & wfdma::EXT_CSR_HIF_MISC_BUSY) == 0 {
                trace!(DMA, "WFDMA idle after {}ms", i);
                break;
            }
            syscall::yield_now();
            hif_misc = self.read_bar(wfdma::EXT_CSR_HIF_MISC);
        }
        if (hif_misc & wfdma::EXT_CSR_HIF_MISC_BUSY) != 0 {
            trace!(DMA, "Warning: WFDMA still busy (HIF_MISC=0x{:08x})", hif_misc);
        }

        // Set RX pause thresholds (Linux values)
        trace!(DMA, "Setting RX pause thresholds...");
        self.write(wfdma::PAUSE_RX_Q_45_TH, 0xc000c);
        self.write(wfdma::PAUSE_RX_Q_67_TH, 0x10008);
        self.write(wfdma::PAUSE_RX_Q_89_TH, 0x10008);

        // Verify RST is cleared (should be 0 from earlier reset sequence)
        let rst_val = self.read(wfdma::RST);
        trace!(DMA, "RST before DMA enable: 0x{:08x}", rst_val);
        if rst_val != 0 {
            trace!(DMA, "Warning: RST should be 0, WFDMA might still be in reset!");
        }

        // NOW enable DMA - this is mt7996_dma_start() in Linux
        // GLO_CFG: TX_DMA_EN | RX_DMA_EN | OMIT_TX_INFO | OMIT_RX_INFO_PFET2 | EXT_EN
        // IMPORTANT: Clear BYTE_SWAP - we use native little-endian descriptors!
        let glo_val = self.read(wfdma::GLO_CFG);
        trace!(DMA, "GLO_CFG before enable: 0x{:08x} (BYTE_SWAP={})",
            glo_val, if (glo_val & glo_cfg::BYTE_SWAP) != 0 { "SET" } else { "clear" });
        let enable_bits = glo_cfg::TX_DMA_EN | glo_cfg::RX_DMA_EN | glo_cfg::OMIT_TX_INFO |
                          glo_cfg::OMIT_RX_INFO_PFET2 | glo_cfg::EXT_EN;
        let clear_bits = glo_cfg::BYTE_SWAP;
        let new_glo_val = (glo_val | enable_bits) & !clear_bits;
        self.write(wfdma::GLO_CFG, new_glo_val);
        trace!(DMA, "GLO_CFG after enable: 0x{:08x}", self.read(wfdma::GLO_CFG));

        // Final state check
        let rst_final = self.read(wfdma::RST);
        let glo_final = self.read(wfdma::GLO_CFG);
        trace!(DMA, "Final: RST=0x{:08x}, GLO_CFG=0x{:08x}", rst_final, glo_final);

        // Verify ring config is still intact
        let rb_lo3 = self.read(ring_base + wfdma::RING_BASE);
        let rb_cnt3 = self.read(ring_base + wfdma::RING_MAX_CNT);
        let rb_cpu3 = self.read(ring_base + wfdma::RING_CPU_IDX);
        trace!(DMA, "Ring final: base=0x{:08x}, cnt={}, cpu={}", rb_lo3, rb_cnt3, rb_cpu3);

        // State: Ready for transfers
        self.state = WfdmaState::Ready;
        trace!(DMA, "WFDMA initialized, state={:?}", self.state);
        true
    }

    /// Send firmware data via FWDL queue using MCU FW_SCATTER command
    pub fn send_fw_scatter(&mut self, fw_data: &[u8]) -> bool {
        // State check: must be Ready
        if self.state != WfdmaState::Ready {
            trace!(DMA, "send_fw_scatter: wrong state {:?}", self.state);
            return false;
        }

        // Build MCU command in ring's command buffer
        let (cmd_paddr, cmd_len) = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };
            ring.build_fw_scatter(fw_data)
        };

        // Get cpu_idx
        let cpu_idx = match self.fwdl_ring.as_ref() {
            Some(r) => r.cpu_idx,
            None => return false,
        };

        // Fill descriptor to point at command buffer
        let new_cpu_idx = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };

            let desc = ring.get_desc(cpu_idx);
            *desc = TxDesc::for_firmware(cmd_paddr, cmd_len, true);

            // Flush descriptor to RAM so DMA can see it
            flush_buffer(desc as *const _ as u64, core::mem::size_of::<TxDesc>());

            // Debug: dump descriptor
            if cpu_idx == 0 {
                trace!(DMA, "First descriptor:");
                trace!(DMA, "  buf0=0x{:08x} ctrl=0x{:08x}", desc.buf0, desc.ctrl);
                trace!(DMA, "  buf1=0x{:08x} info=0x{:08x}", desc.buf1, desc.info);
                trace!(DMA, "  cmd_paddr=0x{:x} cmd_len={}", cmd_paddr, cmd_len);

                // Dump first 32 bytes of command buffer (MCU TXD header)
                let cmd_ptr = ring.cmd_vaddr as *const u32;
                unsafe {
                    trace!(DMA, "  TXD[0-3]: {:08x} {:08x} {:08x} {:08x}",
                        *cmd_ptr, *cmd_ptr.add(1), *cmd_ptr.add(2), *cmd_ptr.add(3));
                    trace!(DMA, "  TXD[4-7]: {:08x} {:08x} {:08x} {:08x}",
                        *cmd_ptr.add(4), *cmd_ptr.add(5), *cmd_ptr.add(6), *cmd_ptr.add(7));
                    trace!(DMA, "  MCU[0-3]: {:08x} {:08x} {:08x} {:08x}",
                        *cmd_ptr.add(8), *cmd_ptr.add(9), *cmd_ptr.add(10), *cmd_ptr.add(11));
                }
            }

            // Advance CPU index
            ring.advance();
            ring.cpu_idx
        };

        // Update CPU index register (rings doorbell) - WFDMA relative
        let ring_base = wfdma::MCU_RING_BASE + (McuQueue::Fwdl as u32) * 0x10;

        // Debug: check state before doorbell
        if cpu_idx == 0 {
            let pre_cpu = self.read(ring_base + wfdma::RING_CPU_IDX);
            let glo_cfg = self.read(wfdma::GLO_CFG);
            trace!(DMA, "Pre-doorbell: cpu_idx={}, GLO_CFG=0x{:08x}", pre_cpu, glo_cfg);
        }

        // Ring the doorbell
        self.write(ring_base + wfdma::RING_CPU_IDX, new_cpu_idx);

        // Debug: verify ring state after doorbell
        if cpu_idx == 0 {
            let after1 = self.read(ring_base + wfdma::RING_CPU_IDX);

            // Try writing again with barrier
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            self.write(ring_base + wfdma::RING_CPU_IDX, new_cpu_idx);
            let after2 = self.read(ring_base + wfdma::RING_CPU_IDX);

            let rb_dma = self.read(ring_base + wfdma::RING_DMA_IDX);
            trace!(DMA, "Post-doorbell: cpu_idx={}/{}, dma_idx={}", after1, after2, rb_dma);

            // Dump all FWDL ring registers to verify configuration
            trace!(DMA, "FWDL ring regs: {:08x} {:08x} {:08x} {:08x} {:08x}",
                self.read(ring_base + 0x00),
                self.read(ring_base + 0x04),
                self.read(ring_base + 0x08),
                self.read(ring_base + 0x0c),
                self.read(ring_base + 0x10));
        }

        true
    }

    /// Send a generic command through DMA
    fn send_command(&mut self, cmd_paddr: u64, cmd_len: usize, debug_name: &str) -> bool {
        // State check: must be Ready
        if self.state != WfdmaState::Ready {
            trace!(DMA, "send_command({}): wrong state {:?}", debug_name, self.state);
            return false;
        }

        let cpu_idx = match self.fwdl_ring.as_ref() {
            Some(r) => r.cpu_idx,
            None => return false,
        };

        let new_cpu_idx = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };

            let desc = ring.get_desc(cpu_idx);
            *desc = TxDesc::for_firmware(cmd_paddr, cmd_len, true);

            // Flush descriptor to RAM so DMA can see it
            flush_buffer(desc as *const _ as u64, core::mem::size_of::<TxDesc>());

            // Debug for first command
            trace!(DMA, "Sending {}: buf=0x{:x} len={}", debug_name, cmd_paddr, cmd_len);

            ring.advance();
            ring.cpu_idx
        };

        let ring_base = wfdma::MCU_RING_BASE + (McuQueue::Fwdl as u32) * 0x10;
        self.write(ring_base + wfdma::RING_CPU_IDX, new_cpu_idx);

        true
    }

    /// Send TARGET_ADDRESS_LEN_REQ (init_download) command
    pub fn send_init_download(&mut self, addr: u32, len: u32, mode: u32) -> bool {
        let (cmd_paddr, cmd_len) = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };
            ring.build_init_download(addr, len, mode)
        };

        trace!(DMA, "init_download: addr=0x{:08x} len={} mode=0x{:x}", addr, len, mode);
        self.send_command(cmd_paddr, cmd_len, "TARGET_ADDRESS_LEN_REQ")
    }

    /// Send patch semaphore control command
    pub fn send_patch_sem_ctrl(&mut self, get: bool) -> bool {
        let (cmd_paddr, cmd_len) = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };
            ring.build_patch_sem_ctrl(get)
        };

        self.send_command(cmd_paddr, cmd_len, if get { "SEM_GET" } else { "SEM_RELEASE" })
    }

    /// Send patch finish command
    pub fn send_patch_finish(&mut self) -> bool {
        let (cmd_paddr, cmd_len) = {
            let ring = match self.fwdl_ring.as_mut() {
                Some(r) => r,
                None => return false,
            };
            ring.build_patch_finish()
        };

        self.send_command(cmd_paddr, cmd_len, "PATCH_FINISH")
    }

    /// Wait for command completion - checks for MCU response or state change
    pub fn wait_dma_done(&mut self, timeout_ms: u32) -> bool {
        let ring = match self.fwdl_ring.as_ref() {
            Some(r) => r,
            None => return false,
        };

        let tx_ring_base = wfdma::MCU_RING_BASE + (McuQueue::Fwdl as u32) * 0x10;
        let rx_ring_base = wfdma::RX_RING_BASE + wfdma::RXQ_MCU * 0x10;

        let initial_dma_idx = self.read(tx_ring_base + wfdma::RING_DMA_IDX);
        let initial_rx_dma_idx = self.read(rx_ring_base + wfdma::RING_DMA_IDX);

        for i in 0..(timeout_ms * 10) {
            // Check TX DMA progress
            let tx_dma_idx = self.read(tx_ring_base + wfdma::RING_DMA_IDX);

            // Check RX DMA progress (MCU responses)
            let rx_dma_idx = self.read(rx_ring_base + wfdma::RING_DMA_IDX);

            // Check interrupt status
            let int_src = self.read(wfdma::INT_SRC);
            let mcu_int = self.read_bar(mcu_int::EVENT);

            // If any progress or interrupt, report and continue
            if tx_dma_idx != initial_dma_idx || rx_dma_idx != initial_rx_dma_idx ||
               int_src != 0 || mcu_int != 0 {
                trace!(DMA, "Progress @{}ms: TX dma_idx={} RX dma_idx={} INT_SRC=0x{:x} MCU_INT=0x{:x}",
                    i / 10, tx_dma_idx, rx_dma_idx, int_src, mcu_int);

                // Clear interrupts
                if int_src != 0 {
                    self.write(wfdma::INT_SRC, int_src);
                }

                // Check if TX completed
                if tx_dma_idx == ring.cpu_idx {
                    return true;
                }
            }

            // Poll MCU_CMD for state changes
            let mcu_cmd = self.read(wfdma::MCU_CMD);
            if (mcu_cmd & mcu_cmd::NORMAL_STATE) != 0 {
                trace!(DMA, "MCU reached normal state: 0x{:08x}", mcu_cmd);
            }

            syscall::yield_now();
        }

        // Final status dump
        let tx_dma_idx = self.read(tx_ring_base + wfdma::RING_DMA_IDX);
        let rx_dma_idx = self.read(rx_ring_base + wfdma::RING_DMA_IDX);
        let int_src = self.read(wfdma::INT_SRC);
        let mcu_cmd = self.read(wfdma::MCU_CMD);
        let glo_cfg = self.read(wfdma::GLO_CFG);

        trace!(DMA, "Timeout after {}ms:", timeout_ms);
        trace!(DMA, "  TX: cpu_idx={}, dma_idx={}", ring.cpu_idx, tx_dma_idx);
        trace!(DMA, "  RX: dma_idx={}", rx_dma_idx);
        trace!(DMA, "  INT_SRC=0x{:08x} MCU_CMD=0x{:08x} GLO_CFG=0x{:08x}",
            int_src, mcu_cmd, glo_cfg);

        // Return false on timeout - DMA didn't complete
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
