//! WFDMA Ring Management — TxRing, RxRing, DMA init/enable/disable
//!
//! DMA descriptor format, ring allocation, and buffer management for the
//! MT7996 WFDMA engine. Uses Linux head/tail/queued model to avoid
//! wrap-boundary ambiguity.

use userlib::uwarn;
use crate::regs::*;
use crate::device::Mt76Device;

// ============================================================================
// Cache/barrier utilities
// ============================================================================

/// Write memory barrier for DMA — ensures stores are visible to PCIe device.
#[inline(always)]
pub fn dma_wmb() {
    unsafe { core::arch::asm!("dsb st", options(nostack, preserves_flags)); }
}

/// Full data synchronization barrier.
#[inline(always)]
pub fn dsb_sy() {
    unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)); }
}

/// Flush a single cache line to memory.
#[inline(always)]
pub fn flush_cache_line(addr: u64) {
    unsafe { core::arch::asm!("dc cvac, {}", in(reg) addr, options(nostack, preserves_flags)); }
}

/// Flush a buffer range to memory.
pub fn flush_buffer(virt_addr: u64, size: usize) {
    const CACHE_LINE: usize = 64;
    let start = virt_addr & !(CACHE_LINE as u64 - 1);
    let end = (virt_addr + size as u64 + CACHE_LINE as u64 - 1) & !(CACHE_LINE as u64 - 1);
    let mut a = start;
    while a < end {
        flush_cache_line(a);
        a += CACHE_LINE as u64;
    }
    dsb_sy();
}

// ============================================================================
// DMA Descriptor — 16 bytes, matches hardware layout
// ============================================================================

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct Descriptor {
    pub buf0: u32,
    pub ctrl: u32,
    pub buf1: u32,
    pub info: u32,
}

// ============================================================================
// TX Ring
// ============================================================================

/// Transmit ring — firmware download, MCU commands, and data TX.
pub struct TxRing {
    pub regs_base: u32,
    pub ndesc: u32,
    pub cpu_idx: u32,
    pub sweep_idx: u32,
    desc_virt: u64,
    desc_phys: u64,
    buf_virt: u64,
    buf_phys: u64,
    pub buf_stride: usize,
    /// Response RX queue registers (for send_and_wait).
    pub rx_regs: u32,
    pub rx_buf_virt: u64,
    pub rx_buf_size: u32,
}

impl TxRing {
    pub fn new(
        regs_base: u32, ndesc: u32,
        desc_virt: u64, desc_phys: u64,
        buf_virt: u64, buf_phys: u64,
    ) -> Self {
        Self {
            regs_base, ndesc,
            cpu_idx: 0, sweep_idx: 0,
            desc_virt, desc_phys,
            buf_virt, buf_phys,
            buf_stride: MCU_FW_DL_BUF_SIZE,
            rx_regs: 0,
            rx_buf_virt: 0,
            rx_buf_size: 0,
        }
    }

    /// Descriptor pointer at index.
    pub fn desc(&self, idx: u32) -> *mut Descriptor {
        unsafe { (self.desc_virt as *mut Descriptor).add(idx as usize) }
    }

    /// Buffer pointer at index.
    pub fn buf(&self, idx: u32) -> *mut u8 {
        unsafe { (self.buf_virt as *mut u8).add(idx as usize * self.buf_stride) }
    }

    /// Buffer physical address at index.
    pub fn buf_phys_at(&self, idx: u32) -> u64 {
        self.buf_phys + (idx as u64 * self.buf_stride as u64)
    }

    /// Index of the most recently submitted descriptor.
    pub fn last_submitted(&self) -> u32 {
        if self.cpu_idx == 0 { self.ndesc - 1 } else { self.cpu_idx - 1 }
    }

    /// Write header + payload to a buffer, program the descriptor, and kick DMA.
    ///
    /// This is the single submission path for all MCU commands and firmware chunks.
    /// The header is written first, then the payload appended. The total length
    /// is `header.len() + payload.len()`.
    pub fn submit_cmd(
        &mut self, dev: &Mt76Device,
        header: &[u8], payload: &[u8],
    ) {
        let idx = self.cpu_idx;
        let total = header.len() + payload.len();
        let buf = self.buf(idx);
        unsafe {
            core::ptr::write_bytes(buf, 0, total);
            core::ptr::copy_nonoverlapping(header.as_ptr(), buf, header.len());
            if !payload.is_empty() {
                core::ptr::copy_nonoverlapping(payload.as_ptr(), buf.add(header.len()), payload.len());
            }
        }
        self.write_desc_and_kick(dev, idx, total);
    }

    /// Write raw data to a buffer, program the descriptor, and kick DMA.
    ///
    /// Used for firmware scatter chunks that have no header/payload split.
    pub fn submit_raw(&mut self, dev: &Mt76Device, data: &[u8]) {
        let idx = self.cpu_idx;
        let buf = self.buf(idx);
        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), buf, data.len());
        }
        self.write_desc_and_kick(dev, idx, data.len());
    }

    /// Submit data to the ring (legacy fw download path with full-check).
    pub fn submit(&mut self, dev: &Mt76Device, data: &[u8]) -> Result<(), RingFull> {
        let next = (self.cpu_idx + 1) % self.ndesc;
        if next == self.sweep_idx {
            return Err(RingFull);
        }
        self.submit_raw(dev, data);
        Ok(())
    }

    /// Program a descriptor and kick DMA. Shared by all submission paths.
    fn write_desc_and_kick(&mut self, dev: &Mt76Device, idx: u32, len: usize) {
        let phys = self.buf_phys_at(idx);
        let desc = self.desc(idx);
        let ctrl = ((len as u32) << 16) | MT_DMA_CTL_LAST_SEC0;
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(phys));
            core::ptr::write_volatile(&mut (*desc).buf1, 0);
            core::ptr::write_volatile(&mut (*desc).info, dma_addr_hi(phys));
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl);
        }
        flush_buffer(self.buf(idx) as u64, len);
        flush_buffer(desc as u64, core::mem::size_of::<Descriptor>());

        self.cpu_idx = (self.cpu_idx + 1) % self.ndesc;
        dma_wmb();
        dev.wr(self.regs_base + MT_QUEUE_CPU_IDX, self.cpu_idx);
    }

    /// Reclaim completed TX descriptors (sweep from sweep_idx to DMA_IDX).
    pub fn reclaim(&mut self, dev: &Mt76Device) -> u32 {
        let mut count = 0u32;
        loop {
            if self.sweep_idx == self.cpu_idx {
                break;
            }
            let desc = self.desc(self.sweep_idx);
            let ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
            if ctrl & MT_DMA_CTL_DMA_DONE == 0 {
                break;
            }
            self.sweep_idx = (self.sweep_idx + 1) % self.ndesc;
            count += 1;
        }
        let _ = dev; // used for potential register reads
        count
    }
}

/// TX ring is full — no free descriptors.
#[derive(Debug)]
pub struct RingFull;

// ============================================================================
// RX Ring
// ============================================================================

/// Receive ring — MCU events, data RX.
///
/// Uses Linux head/tail/queued model:
/// - `head`: next slot to refill (written to CPU_IDX after refill)
/// - `tail`: next slot to dequeue (check DMA_DONE here)
/// - `queued`: slots owned by hardware
pub struct RxRing {
    pub hw_idx: u32,
    pub regs_base: u32,
    pub ndesc: u32,
    pub desc_virt: u64,
    pub buf_size: u32,
    pub buf_phys: u64,
    pub buf_virt: u64,
    pub head: u32,
    pub tail: u32,
    pub queued: u32,
}

impl RxRing {
    pub const ZERO: Self = Self {
        hw_idx: 0, regs_base: 0, ndesc: 0, desc_virt: 0,
        buf_size: 0, buf_phys: 0, buf_virt: 0,
        head: 0, tail: 0, queued: 0,
    };

    /// Reset software counters for full reinit.
    pub fn reset_counters(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.queued = 0;
    }

    /// Descriptor pointer at index.
    pub fn desc(&self, idx: u32) -> *mut Descriptor {
        unsafe { (self.desc_virt as *mut Descriptor).add(idx as usize) }
    }

    /// Buffer slice at index (read-only view).
    pub fn buf_slice(&self, idx: u32) -> &[u8] {
        unsafe {
            let ptr = (self.buf_virt as *const u8).add(idx as usize * self.buf_size as usize);
            core::slice::from_raw_parts(ptr, self.buf_size as usize)
        }
    }

    /// Fill ring with buffers, up to ndesc-1 (one-empty-slot convention).
    /// Source: dma.c:684-694
    pub fn fill(&mut self, dev: &Mt76Device) {
        let mut filled = 0u32;
        while self.queued < self.ndesc - 1 {
            let i = self.head as usize;
            let desc = self.desc(self.head);
            let phys = self.buf_phys + (i as u64 * self.buf_size as u64);

            // RX: ctrl = SD_LEN0(buf_size), no LAST_SEC0 (hardware-generated).
            // DMA_DONE cleared so hardware can set it after writing.
            let ctrl = (self.buf_size) << 16;
            unsafe {
                core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(phys));
                core::ptr::write_volatile(&mut (*desc).buf1, dma_addr_hi(phys));
                core::ptr::write_volatile(&mut (*desc).info, 0);
                core::ptr::write_volatile(&mut (*desc).ctrl, ctrl);
            }

            self.head = (self.head + 1) % self.ndesc;
            self.queued += 1;
            filled += 1;
        }

        if filled > 0 {
            let desc_bytes = self.ndesc as usize * core::mem::size_of::<Descriptor>();
            flush_buffer(self.desc_virt, desc_bytes);
            dma_wmb();
            dev.wr(self.regs_base + MT_QUEUE_CPU_IDX, self.head);
        }
    }

    /// Dequeue all DMA_DONE entries, clearing them for reuse.
    /// Returns number of entries drained.
    pub fn drain(&mut self, dev: &Mt76Device) -> u32 {
        let mut count = 0u32;
        while self.queued > 0 {
            let desc = self.desc(self.tail);
            let ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
            if ctrl & MT_DMA_CTL_DMA_DONE == 0 { break; }

            unsafe {
                core::ptr::write_volatile(&mut (*desc).ctrl, (self.buf_size) << 16);
            }
            self.tail = (self.tail + 1) % self.ndesc;
            self.queued -= 1;
            count += 1;
        }
        if count > 0 {
            self.fill(dev);
        }
        count
    }

    /// Process entries with a callback. For each DMA_DONE entry, calls `f`
    /// with the buffer contents. Returns number of entries processed.
    pub fn process<F>(&mut self, dev: &Mt76Device, mut f: F) -> u32
    where
        F: FnMut(&[u8]),
    {
        let mut count = 0u32;
        while self.queued > 0 {
            let desc = self.desc(self.tail);
            let ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
            if ctrl & MT_DMA_CTL_DMA_DONE == 0 { break; }

            f(self.buf_slice(self.tail));

            unsafe {
                core::ptr::write_volatile(&mut (*desc).ctrl, (self.buf_size) << 16);
            }
            self.tail = (self.tail + 1) % self.ndesc;
            self.queued -= 1;
            count += 1;
        }
        if count > 0 {
            self.fill(dev);
        }
        count
    }
}

// ============================================================================
// DMA Queue Configuration
// ============================================================================

pub struct QueueConfig {
    pub int_mask: u32,
    pub hw_idx: u32,
}

/// DMA configuration — queue mapping for MT7996.
/// Source: dma.c:50-151
pub struct DmaConfig {
    pub mcuq: [(u32, u32); 3],   // (int_mask, hw_idx) for FWDL, WM, WA
    pub rxq: [(u32, u32); 6],    // (int_mask, hw_idx) for 6 RX queues
}

impl DmaConfig {
    pub fn new(has_hif2: bool) -> Self {
        let _ = has_hif2; // same config for MT7996
        Self {
            mcuq: [
                (MT_INT_TX_DONE_FWDL,   MT7996_TXQ_FWDL),
                (MT_INT_TX_DONE_MCU_WM,  MT7996_TXQ_MCU_WM),
                (MT_INT_TX_DONE_MCU_WA,  MT7996_TXQ_MCU_WA),
            ],
            rxq: [
                (MT_INT_RX_DONE_WM,      MT7996_RXQ_MCU_WM),
                (MT_INT_RX_DONE_WA,      MT7996_RXQ_MCU_WA),
                (MT_INT_RX_DONE_WA_MAIN, MT7996_RXQ_MCU_WA_MAIN),
                (MT_INT_RX_DONE_WA_TRI,  MT7996_RXQ_MCU_WA_TRI),
                (MT_INT_RX_DONE_BAND0,   MT7996_RXQ_BAND0),
                (MT_INT_RX_DONE_BAND2,   MT7996_RXQ_BAND2),
            ],
        }
    }
}

// ============================================================================
// DMA Engine Control
// ============================================================================

/// Disable WFDMA DMA engines, optionally with reset.
/// Source: dma.c:545-585
pub fn dma_disable(dev: &Mt76Device, reset: bool) {
    let hif1 = if dev.has_hif2 { HIF1_OFS } else { 0 };

    if reset {
        dev.clear(MT_WFDMA0_RST,
            MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
        dev.set(MT_WFDMA0_RST,
            MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);

        if dev.has_hif2 {
            dev.clear(MT_WFDMA0_RST + hif1,
                MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
            dev.set(MT_WFDMA0_RST + hif1,
                MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
        }
    }

    let glo_bits = MT_WFDMA0_GLO_CFG_TX_DMA_EN
        | MT_WFDMA0_GLO_CFG_RX_DMA_EN
        | MT_WFDMA0_GLO_CFG_OMIT_TX_INFO
        | MT_WFDMA0_GLO_CFG_OMIT_RX_INFO
        | MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2;
    dev.clear(MT_WFDMA0_GLO_CFG, glo_bits);
    if dev.has_hif2 {
        dev.clear(MT_WFDMA0_GLO_CFG + hif1, glo_bits);
    }
}

/// Start WFDMA DMA engines + configure interrupt mask.
/// Source: dma.c:587-625
pub fn dma_start(dev: &Mt76Device, reset: bool) {
    let hif1 = if dev.has_hif2 { HIF1_OFS } else { 0 };

    if !reset {
        let glo_bits = MT_WFDMA0_GLO_CFG_TX_DMA_EN
            | MT_WFDMA0_GLO_CFG_RX_DMA_EN
            | MT_WFDMA0_GLO_CFG_OMIT_TX_INFO
            | MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2
            | MT_WFDMA0_GLO_CFG_EXT_EN;
        dev.set(MT_WFDMA0_GLO_CFG, glo_bits);
        if dev.has_hif2 {
            dev.set(MT_WFDMA0_GLO_CFG + hif1, glo_bits);
        }
    }

    // Interrupt mask — controls which rings deliver frames.
    let irq_mask = MT_INT_MCU_CMD
        | MT_INT_RX_DONE_MCU
        | MT_INT_TX_DONE_MCU
        | MT_INT_RX_DONE_BAND0
        | MT_INT_RX_DONE_WA_MAIN
        | MT_INT_TX_DONE_BAND0;

    dev.wr(MT_INT_MASK_CSR, irq_mask);
    if dev.has_hif2 { dev.wr(MT_INT1_MASK_CSR, irq_mask); }

    // Clear pending interrupts
    dev.wr(MT_INT_MASK_CSR, 0);
    if dev.has_hif2 { dev.wr(MT_INT1_MASK_CSR, 0); }

    let src = dev.rr(MT_INT_SOURCE_CSR);
    dev.wr(MT_INT_SOURCE_CSR, src);
    if dev.has_hif2 {
        let src1 = dev.rr(MT_INT1_SOURCE_CSR);
        dev.wr(MT_INT1_SOURCE_CSR, src1);
    }

    dev.wr(MT_INT_MASK_CSR, irq_mask);
    if dev.has_hif2 { dev.wr(MT_INT1_MASK_CSR, irq_mask); }
}

/// Configure DMA prefetch registers.
/// Source: dma.c:167-240, 520-523
fn dma_prefetch_hif(dev: &Mt76Device, ofs: u32) {
    // TX prefetch
    dev.wr(MT_WFDMA0_BASE + 0x640 + ofs, 0x00000002);
    dev.wr(MT_WFDMA0_BASE + 0x644 + ofs, 0x00200002);
    dev.wr(MT_WFDMA0_BASE + 0x648 + ofs, 0x00400008);
    dev.wr(MT_WFDMA0_BASE + 0x64c + ofs, 0x00c00008);
    dev.wr(MT_WFDMA0_BASE + 0x650 + ofs, 0x01400002);
    dev.wr(MT_WFDMA0_BASE + 0x654 + ofs, 0x01600008);

    // RX prefetch
    dev.wr(MT_WFDMA0_BASE + 0x680 + ofs, 0x01e00002);
    dev.wr(MT_WFDMA0_BASE + 0x684 + ofs, 0x02000002);
    dev.wr(MT_WFDMA0_BASE + 0x688 + ofs, 0x02200002);
    dev.wr(MT_WFDMA0_BASE + 0x68c + ofs, 0x02400002);
    dev.wr(MT_WFDMA0_BASE + 0x690 + ofs, 0x02600010);
    dev.wr(MT_WFDMA0_BASE + 0x694 + ofs, 0x03600010);

    dev.set(WF_WFDMA0_GLO_CFG_EXT1 + ofs, WF_WFDMA0_GLO_CFG_EXT1_CALC_MODE);
}

/// Full DMA enable sequence.
/// Source: dma.c:627-754
pub fn dma_enable(dev: &Mt76Device, reset: bool) {
    let hif1 = if dev.has_hif2 { HIF1_OFS } else { 0 };

    dev.wr(MT_WFDMA0_RST_DTX_PTR, !0u32);
    if dev.has_hif2 { dev.wr(MT_WFDMA0_RST_DTX_PTR + hif1, !0u32); }

    // Disable delay interrupts
    dev.wr(MT_WFDMA0_PRI_DLY_INT_CFG0, 0);
    dev.wr(MT_WFDMA0_PRI_DLY_INT_CFG1, 0);
    dev.wr(MT_WFDMA0_PRI_DLY_INT_CFG2, 0);
    if dev.has_hif2 {
        dev.wr(MT_WFDMA0_PRI_DLY_INT_CFG0 + hif1, 0);
        dev.wr(MT_WFDMA0_PRI_DLY_INT_CFG1 + hif1, 0);
        dev.wr(MT_WFDMA0_PRI_DLY_INT_CFG2 + hif1, 0);
    }

    // Prefetch
    dma_prefetch_hif(dev, 0);
    if dev.has_hif2 { dma_prefetch_hif(dev, HIF1_OFS); }

    // Busy enable
    dev.set(MT_WFDMA0_BUSY_ENA,
        MT_WFDMA0_BUSY_ENA_TX_FIFO0
        | MT_WFDMA0_BUSY_ENA_TX_FIFO1
        | MT_WFDMA0_BUSY_ENA_RX_FIFO);
    if dev.has_hif2 {
        dev.set(MT_WFDMA0_BUSY_ENA + hif1,
            MT_WFDMA0_PCIE1_BUSY_ENA_TX_FIFO0
            | MT_WFDMA0_PCIE1_BUSY_ENA_TX_FIFO1
            | MT_WFDMA0_PCIE1_BUSY_ENA_RX_FIFO);
    }

    // Wait for HIF not busy
    if dev.poll(MT_WFDMA_EXT_CSR_HIF_MISC, MT_WFDMA_EXT_CSR_HIF_MISC_BUSY, 0, 1).is_err() {
        uwarn!("dma", "hif_misc_busy_timeout");
    }

    // GLO_CFG_EXT0/1
    dev.set(WF_WFDMA0_GLO_CFG_EXT0,
        WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD | WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);
    dev.set(WF_WFDMA0_GLO_CFG_EXT1,
        WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);

    // Pause RX thresholds
    dev.wr(MT_WFDMA0_PAUSE_RX_Q_45_TH, 0xc000c);
    dev.wr(MT_WFDMA0_PAUSE_RX_Q_67_TH, 0x10008);
    dev.wr(MT_WFDMA0_PAUSE_RX_Q_89_TH, 0x10008);
    dev.wr(MT_WFDMA0_PAUSE_RX_Q_RRO_TH, 0x20);

    if dev.has_hif2 {
        dev.set(WF_WFDMA0_GLO_CFG_EXT0 + hif1,
            WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD | WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);
        dev.set(WF_WFDMA0_GLO_CFG_EXT1 + hif1,
            WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);

        dev.set(MT_WFDMA_HOST_CONFIG, MT_WFDMA_HOST_CONFIG_PDMA_BAND);
        dev.clear(MT_WFDMA_HOST_CONFIG,
            MT_WFDMA_HOST_CONFIG_BAND0_PCIE1
            | MT_WFDMA_HOST_CONFIG_BAND1_PCIE1
            | MT_WFDMA_HOST_CONFIG_BAND2_PCIE1);
        dev.set(MT_WFDMA_HOST_CONFIG, MT_WFDMA_HOST_CONFIG_BAND2_PCIE1);

        dev.rmw(MT_WFDMA_AXI_R2A_CTRL, MT_WFDMA_AXI_R2A_CTRL_OUTSTAND_MASK, 0x14);

        dev.wr(MT_WFDMA0_PAUSE_RX_Q_45_TH + hif1, 0xc000c);
        dev.wr(MT_WFDMA0_PAUSE_RX_Q_67_TH + hif1, 0x10008);
        dev.wr(MT_WFDMA0_PAUSE_RX_Q_89_TH + hif1, 0x10008);
        dev.wr(MT_WFDMA0_PAUSE_RX_Q_RRO_TH + hif1, 0x20);
    }

    if dev.has_hif2 {
        dev.set(MT_WFDMA0_RX_INT_PCIE_SEL, MT_WFDMA0_RX_INT_SEL_RING3);
    }

    dma_start(dev, reset);
}

// ============================================================================
// TX — Management and Data Frame Enqueue
// ============================================================================

/// Hardware layout of mt76_connac_fw_txp (44 bytes at TXD + 32).
/// Source: mt76_connac.h struct mt76_connac_fw_txp
#[repr(C, packed)]
struct FwTxp {
    flags: u16,       // CT info flags
    token: u16,       // TX token for completion matching
    _rsvd: u8,
    rept_wds_wcid: u16, // STA WCID (unaligned — LE)
    nbuf: u8,
    buf_addr: [u32; 4], // buffer physical addresses (lower 32 bits)
    _pad: [u8; 16],
    buf_len: [u16; 4],  // buffer lengths | upper 4 addr bits
}

/// Write a fw_txp into a DMA buffer and the frame after it.
/// Shared by both management and data TX paths.
unsafe fn write_txp_and_frame(
    buf: *mut u8, buf_phys: u64,
    flags: u16, token: u16, wcid: u16,
    frame: &[u8],
) {
    let txp = buf.add(MT_TXD_SIZE) as *mut FwTxp;
    core::ptr::write_bytes(txp, 0, 1);

    let frame_phys = buf_phys + MT_TXWI_SIZE as u64;
    let addr_hi = ((frame_phys >> 32) & 0xF) as u16;

    (*txp).flags = flags.to_le();
    (*txp).token = token.to_le();
    (*txp).rept_wds_wcid = wcid.to_le();
    (*txp).nbuf = 1;
    (*txp).buf_addr[0] = (frame_phys as u32).to_le();
    (*txp).buf_len[0] = ((frame.len() as u16 & 0x0FFF) | (addr_hi << 12)).to_le();

    core::ptr::copy_nonoverlapping(frame.as_ptr(), buf.add(MT_TXWI_SIZE), frame.len());
}

/// Program a 2-buffer TX descriptor and kick DMA.
/// buf0 = TXD+TXP region, buf1 = frame region (for firmware CT parsing).
fn commit_tx_desc(ring: &mut TxRing, dev: &Mt76Device, idx: u32, frame_len: usize) {
    let buf_phys = ring.buf_phys_at(idx);
    let frame_phys = buf_phys + MT_TXWI_SIZE as u64;

    let ctrl = (MT_TXWI_SIZE as u32) << 16
        | (MT_CT_PARSE_LEN as u32 & MT_DMA_CTL_SD_LEN1_MASK)
        | MT_DMA_CTL_LAST_SEC1;
    let info = dma_addr_hi(buf_phys) | (dma_addr_hi(frame_phys) << 16);

    let desc = ring.desc(idx);
    unsafe {
        core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(buf_phys));
        core::ptr::write_volatile(&mut (*desc).buf1, dma_addr_lo(frame_phys));
        core::ptr::write_volatile(&mut (*desc).info, info);
        core::ptr::write_volatile(&mut (*desc).ctrl, ctrl);
    }

    flush_buffer(ring.buf(idx) as u64, MT_TXWI_SIZE + frame_len);
    flush_buffer(desc as u64, core::mem::size_of::<Descriptor>());

    ring.cpu_idx = (ring.cpu_idx + 1) % ring.ndesc;
    dma_wmb();
    dev.wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);
}

/// Build a management frame TXD (32 bytes) + copy 802.11 frame into `buf`.
/// Returns total length (TXD + frame), or 0 on error.
///
/// Source: wifid/main.rs:1466-1524 wrap_mgmt_txd()
pub fn wrap_mgmt_txd(buf: &mut [u8], frame: &[u8]) -> usize {
    let total = MT_TXD_SIZE + frame.len();
    if buf.len() < total || frame.len() < 2 {
        return 0;
    }

    for b in buf[..MT_TXD_SIZE].iter_mut() { *b = 0; }

    let fc0 = frame[0];
    let subtype = (fc0 >> 4) & 0xF;
    let is_beacon = subtype == 0x8;
    let is_bcast = frame.len() >= 10 && frame[4..10] == [0xFF; 6];

    // TXD0: TX_BYTES | PKT_FMT(CT) | Q_IDX(ALTX0)
    let txd0 = (total as u32)
        | ((MT_TX_TYPE_CT as u32) << MT_TXD0_PKT_FMT_SHIFT)
        | ((MT_LMAC_ALTX0 as u32) << MT_TXD0_Q_IDX_SHIFT);
    buf[0..4].copy_from_slice(&txd0.to_le_bytes());

    // TXD1: FIXED_RATE | HDR_FORMAT(802.11) | HDR_INFO(24/2=12) | OWN_MAC | WLAN_IDX(BMC)
    let txd1 = MT_TXD1_FIXED_RATE
        | ((MT_HDR_FORMAT_802_11 as u32) << MT_TXD1_HDR_FORMAT_SHIFT)
        | (12u32 << MT_TXD1_HDR_INFO_SHIFT)
        | (MT_TX_NORMAL << MT_TXD1_TID_SHIFT)
        | ((HW_BSSID_0 as u32) << MT_TXD1_OWN_MAC_SHIFT)
        | (MT7996_WTBL_RESERVED as u32);
    buf[4..8].copy_from_slice(&txd1.to_le_bytes());

    // TXD2: SUB_TYPE from FC
    buf[8..12].copy_from_slice(&(subtype as u32).to_le_bytes());

    // TXD3: retry/ACK control
    let txd3 = if is_beacon {
        MT_TXD3_NO_ACK | MT_TXD3_BCM | (31u32 << MT_TXD3_REM_TX_COUNT_SHIFT) | MT_TXD3_BA_DISABLE
    } else {
        MT_TXD3_SW_POWER_MGMT | (15u32 << MT_TXD3_REM_TX_COUNT_SHIFT) | MT_TXD3_BA_DISABLE
            | if is_bcast { MT_TXD3_BCM } else { 0 }
    };
    buf[12..16].copy_from_slice(&txd3.to_le_bytes());

    // TXD6: DAS | VTA | DIS_MAT | MSDU_CNT(1) | TX_RATE(basic) | FIXED_BW
    let txd6 = MT_TXD6_DAS | MT_TXD6_VTA | MT_TXD6_DIS_MAT
        | (1u32 << MT_TXD6_MSDU_CNT_SHIFT)
        | ((MT7996_BASIC_RATES_TBL as u32) << MT_TXD6_TX_RATE_SHIFT)
        | MT_TXD6_FIXED_BW;
    buf[24..28].copy_from_slice(&txd6.to_le_bytes());

    buf[MT_TXD_SIZE..total].copy_from_slice(frame);
    total
}

/// Enqueue a pre-built [TXD(32B) | 802.11 frame] on a CT-mode TX ring.
///
/// Source: wifid/dma.rs:1266-1370 tx_enqueue()
pub fn tx_enqueue_mgmt(ring: &mut TxRing, dev: &Mt76Device, data: &[u8], token: u16) -> Result<(), RingFull> {
    if data.len() < MT_TXD_SIZE { return Err(RingFull); }
    let next = (ring.cpu_idx + 1) % ring.ndesc;
    if next == ring.sweep_idx { return Err(RingFull); }

    let idx = ring.cpu_idx;
    let buf = ring.buf(idx);
    let frame = &data[MT_TXD_SIZE..];

    // Write TXD at offset 0
    unsafe { core::ptr::copy_nonoverlapping(data.as_ptr(), buf, MT_TXD_SIZE); }

    let flags = MT_CT_INFO_APPLY_TXD | MT_CT_INFO_NONE_CIPHER_FRAME
        | MT_CT_INFO_MGMT_FRAME | MT_CT_INFO_FROM_HOST;
    unsafe { write_txp_and_frame(buf, ring.buf_phys_at(idx), flags, token, 0xfff, frame); }

    commit_tx_desc(ring, dev, idx, frame.len());
    Ok(())
}

/// Enqueue an Ethernet (802.3) data frame for TX.
///
/// Source: wifid/dma.rs:1380-1464 tx_enqueue_data()
pub fn tx_enqueue_data(
    ring: &mut TxRing, dev: &Mt76Device,
    frame: &[u8], wcid: u16, token: u16, no_cipher: bool,
) -> Result<(), RingFull> {
    if frame.len() < 14 || frame.len() > 1514 { return Err(RingFull); }
    let next = (ring.cpu_idx + 1) % ring.ndesc;
    if next == ring.sweep_idx { return Err(RingFull); }

    let idx = ring.cpu_idx;
    let buf = ring.buf(idx);

    // TXD: zeroed for 802.3 mode — firmware fills from CT info
    unsafe { core::ptr::write_bytes(buf, 0, MT_TXD_SIZE); }

    let flags = MT_CT_INFO_FROM_HOST
        | if no_cipher { MT_CT_INFO_NONE_CIPHER_FRAME } else { 0 };
    unsafe { write_txp_and_frame(buf, ring.buf_phys_at(idx), flags, token, wcid, frame); }

    commit_tx_desc(ring, dev, idx, frame.len());
    Ok(())
}

/// Sweep completed TX descriptors. Returns number of tokens collected.
///
/// Walks from sweep_idx to hardware DMA_IDX (like Linux mt76_dma_tx_cleanup).
/// Reads token from fw_txp at offset 2 in each completed buffer.
///
/// Source: wifid/dma.rs:1560-1577 tx_sweep()
pub fn tx_sweep(ring: &mut TxRing, dev: &Mt76Device, tokens_out: &mut [u16]) -> usize {
    let dma_idx = dev.rr(ring.regs_base + MT_QUEUE_DMA_IDX);
    let mut count = 0;

    while ring.sweep_idx != dma_idx && count < tokens_out.len() {
        let buf = ring.buf(ring.sweep_idx);
        let txp = unsafe { buf.add(MT_TXD_SIZE) };
        let mut tok_bytes = [0u8; 2];
        unsafe { core::ptr::copy_nonoverlapping(txp.add(2), tok_bytes.as_mut_ptr(), 2); }
        tokens_out[count] = u16::from_le_bytes(tok_bytes);

        count += 1;
        ring.sweep_idx = (ring.sweep_idx + 1) % ring.ndesc;
    }

    count
}

// ============================================================================
// Queue setup
// ============================================================================

/// Program a hardware queue: set descriptor base, ring size, zero indices.
pub fn program_queue(dev: &Mt76Device, regs_base: u32, desc_phys: u64, ndesc: u32) {
    dev.wr(regs_base + MT_QUEUE_CPU_IDX, 0);
    dev.wr(regs_base + MT_QUEUE_DMA_IDX, 0);
    dev.wr(regs_base + MT_QUEUE_DESC_BASE, desc_phys as u32);
    dev.wr(regs_base + MT_QUEUE_RING_SIZE, ndesc);
    let _ = dev.rr(regs_base + MT_QUEUE_DMA_IDX); // readback
}

/// Initialize descriptors for a queue (zero all, set DMA_DONE, program HW).
pub fn init_queue(dev: &Mt76Device, ring_base: u32, hw_idx: u32, ndesc: u32, desc_phys: u64, desc_virt: u64) {
    let regs_base = ring_base + hw_idx * MT_RING_SIZE;

    for i in 0..ndesc as usize {
        let desc = unsafe { (desc_virt as *mut Descriptor).add(i) };
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, 0);
            core::ptr::write_volatile(&mut (*desc).buf1, 0);
            core::ptr::write_volatile(&mut (*desc).info, 0);
            core::ptr::write_volatile(&mut (*desc).ctrl, MT_DMA_CTL_DMA_DONE);
        }
    }

    program_queue(dev, regs_base, desc_phys, ndesc);
}
