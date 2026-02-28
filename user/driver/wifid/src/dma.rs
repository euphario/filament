//! MT7996 DMA — Descriptor structs, ring management, DMA init/enable/disable
//!
//! EXACT translation of Linux mt76/dma.c and mt7996/dma.c.
//! Contains Mt76Desc, DmaConfig, TxRing, RxRing, and all DMA initialization.

use userlib::{uinfo, uwarn, udebug};
use crate::regs::*;
use crate::device::Mt7996Dev;
use crate::event::{self, RxMibCounters, RxFrameClass};

// ============================================================================
// Cache/barrier utilities
// ============================================================================

/// Write memory barrier for DMA — ensures stores are visible to PCIe devices
/// Linux wmb() on ARM64 is "dsb st"
#[inline(always)]
pub fn dma_wmb() {
    unsafe {
        core::arch::asm!("dsb st", options(nostack, preserves_flags));
    }
}

/// Full data synchronization barrier
#[inline(always)]
pub fn dsb_sy() {
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// Flush a single cache line to memory (DC CVAC)
#[inline(always)]
pub fn flush_cache_line(addr: u64) {
    unsafe {
        core::arch::asm!("dc cvac, {}", in(reg) addr, options(nostack, preserves_flags));
    }
}

/// Flush a buffer to memory (for CPU writes that DMA device will read)
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
// DMA Descriptor Structure — EXACT Linux layout
// Source: dma.h struct mt76_desc
// ============================================================================

#[repr(C, align(4))]
#[derive(Clone, Copy, Default)]
pub struct Mt76Desc {
    pub buf0: u32,
    pub ctrl: u32,
    pub buf1: u32,
    pub info: u32,
}

// ============================================================================
// RX Queue Info — for tracking RX queues that need buffer fill
// ============================================================================

/// RX queue state — matches Linux mt76_queue fields.
///
/// Linux uses head/tail/queued counters instead of raw hardware index
/// comparison. This avoids the wrap-boundary ambiguity where
/// `(cpu_idx+1) % ndesc == dma_idx` falsely reads "empty" when DMA_IDX
/// wraps to 0 and CPU_IDX = ndesc-1.
///
/// - `head`: next slot to refill (after refill, written to CPU_IDX)
/// - `tail`: next slot to dequeue (check DMA_DONE here)
/// - `queued`: number of slots given to hardware (head - tail, mod ndesc)
#[derive(Clone, Copy, Default)]
pub struct RxQueueInfo {
    pub hw_idx: u32,
    pub regs_base: u32,
    pub ndesc: u32,
    pub desc_virt: u64,
    pub buf_size: u32,
    pub buf_phys: u64,
    pub buf_virt: u64,
    /// Next slot to fill — written to CPU_IDX after refill.
    /// Linux: q->head
    pub head: u32,
    /// Next slot to dequeue — check DMA_DONE bit here.
    /// Linux: q->tail
    pub tail: u32,
    /// Number of slots owned by hardware.
    /// Linux: q->queued
    pub queued: u32,
}

impl RxQueueInfo {
    pub const ZERO: Self = Self {
        hw_idx: 0, regs_base: 0, ndesc: 0, desc_virt: 0,
        buf_size: 0, buf_phys: 0, buf_virt: 0,
        head: 0, tail: 0, queued: 0,
    };

    /// Reset software counters so rx_fill() will refill the entire ring.
    /// Use before rx_fill() when reinitializing a ring from scratch.
    pub fn reset_counters(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.queued = 0;
    }
}

// ============================================================================
// Queue Configuration — from mt7996_dma_config()
// Source: dma.c:50-151
// ============================================================================

pub struct QueueConfig {
    pub int_mask: u32,
    pub hw_idx: u32,
    pub is_wfdma0: bool,
}

pub struct DmaConfig {
    pub q_wfdma_mask: u32,
    pub mcuq: [QueueConfig; 3],
    pub txq: [QueueConfig; 3],
    pub rxq: [QueueConfig; 6],
}

impl DmaConfig {
    /// mt7996_dma_config() — EXACT Linux translation
    /// Source: dma.c:50-151
    pub fn new(has_hif2: bool) -> Self {
        let mut cfg = Self {
            q_wfdma_mask: 0,
            mcuq: [
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
            ],
            txq: [
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
            ],
            rxq: [
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
                QueueConfig { int_mask: 0, hw_idx: 0, is_wfdma0: false },
            ],
        };

        // RX queue config
        cfg.rxq_config(MT_RXQ_MCU as usize, WFDMA0, MT_INT_RX_DONE_WM, MT7996_RXQ_MCU_WM);
        cfg.rxq_config(MT_RXQ_MCU_WA as usize, WFDMA0, MT_INT_RX_DONE_WA, MT7996_RXQ_MCU_WA);
        cfg.rxq_config(MT_RXQ_MAIN as usize, WFDMA0, MT_INT_RX_DONE_BAND0, MT7996_RXQ_BAND0);
        cfg.rxq_config(MT_RXQ_MAIN_WA as usize, WFDMA0, MT_INT_RX_DONE_WA_MAIN, MT7996_RXQ_MCU_WA_MAIN);
        cfg.rxq_config(MT_RXQ_BAND2_WA as usize, WFDMA0, MT_INT_RX_DONE_WA_TRI, MT7996_RXQ_MCU_WA_TRI);
        cfg.rxq_config(MT_RXQ_BAND2 as usize, WFDMA0, MT_INT_RX_DONE_BAND2, MT7996_RXQ_BAND2);

        // TX data queues
        cfg.txq_config(0, WFDMA0, MT_INT_TX_DONE_BAND0, MT7996_TXQ_BAND0);
        if has_hif2 {
            cfg.txq_config(1, WFDMA0, MT_INT_TX_DONE_BAND1, MT7996_TXQ_BAND1);
            cfg.txq_config(2, WFDMA0, MT_INT_TX_DONE_BAND1, MT7996_TXQ_BAND2);
        } else {
            cfg.txq_config(2, WFDMA0, MT_INT_TX_DONE_BAND1, MT7996_TXQ_BAND1);
        }

        // MCU TX queues
        cfg.mcuq_config(MT_MCUQ_FWDL as usize, WFDMA0, MT_INT_TX_DONE_FWDL, MT7996_TXQ_FWDL);
        cfg.mcuq_config(MT_MCUQ_WM as usize, WFDMA0, MT_INT_TX_DONE_MCU_WM, MT7996_TXQ_MCU_WM);
        cfg.mcuq_config(MT_MCUQ_WA as usize, WFDMA0, MT_INT_TX_DONE_MCU_WA, MT7996_TXQ_MCU_WA);

        cfg
    }

    fn mcuq_config(&mut self, q: usize, wfdma: bool, int: u32, id: u32) {
        if q < 3 {
            if wfdma { self.q_wfdma_mask |= 1 << q; }
            self.mcuq[q] = QueueConfig { int_mask: int, hw_idx: id, is_wfdma0: wfdma };
        }
    }

    fn txq_config(&mut self, q: usize, wfdma: bool, int: u32, id: u32) {
        if q < 3 {
            if wfdma { self.q_wfdma_mask |= 1 << (q + 3); }
            self.txq[q] = QueueConfig { int_mask: int, hw_idx: id, is_wfdma0: wfdma };
        }
    }

    fn rxq_config(&mut self, q: usize, wfdma: bool, int: u32, id: u32) {
        if q < 6 {
            if wfdma { self.q_wfdma_mask |= 1 << (q + 6); }
            self.rxq[q] = QueueConfig { int_mask: int, hw_idx: id, is_wfdma0: wfdma };
        }
    }
}

// ============================================================================
// TX/RX Ring Management
// ============================================================================

/// TX ring state
pub struct TxRing {
    pub regs_base: u32,
    pub ndesc: u32,
    pub cpu_idx: u32,
    /// Next descriptor to check for DMA completion (for tx_sweep).
    pub sweep_idx: u32,
    pub desc_virt: u64,
    pub desc_phys: u64,
    pub buf_virt: u64,
    pub buf_phys: u64,
    pub buf_stride: usize,
    /// RX queue registers to poll for MCU responses.
    /// WM ring → MCU_WM_RX_REGS (q0), WA ring → MCU_WA_RX_REGS (q1).
    /// Linux: mt7996_mcu_send_message() lines 295-298.
    pub rx_regs: u32,
    /// RX buffer virtual address and size for response parsing.
    pub rx_buf_virt: u64,
    pub rx_buf_size: u32,
}

impl TxRing {
    pub fn new(regs_base: u32, ndesc: u32, desc_virt: u64, desc_phys: u64, buf_virt: u64, buf_phys: u64) -> Self {
        Self {
            regs_base,
            ndesc,
            cpu_idx: 0,
            sweep_idx: 0,
            desc_virt,
            desc_phys,
            buf_virt,
            buf_phys,
            buf_stride: MCU_FW_DL_BUF_SIZE,
            rx_regs: 0,
            rx_buf_virt: 0,
            rx_buf_size: 0,
        }
    }

    /// Get descriptor at index
    pub fn desc(&self, idx: u32) -> *mut Mt76Desc {
        unsafe {
            (self.desc_virt as *mut Mt76Desc).add(idx as usize)
        }
    }

    /// Get buffer for descriptor at index
    pub fn buf(&self, idx: u32) -> *mut u8 {
        unsafe {
            (self.buf_virt as *mut u8).add(idx as usize * self.buf_stride)
        }
    }

    /// Get buffer physical address for descriptor
    pub fn buf_phys(&self, idx: u32) -> u64 {
        self.buf_phys + (idx as u64 * self.buf_stride as u64)
    }
}

/// RX ring state (for MCU responses)
pub struct RxRing {
    pub regs_base: u32,
    pub ndesc: u32,
    pub cpu_idx: u32,
    pub desc_virt: u64,
    pub buf_virt: u64,
    pub buf_phys: u64,
}

impl RxRing {
    pub fn new(regs_base: u32, ndesc: u32, desc_virt: u64, buf_virt: u64, buf_phys: u64) -> Self {
        Self {
            regs_base,
            ndesc,
            cpu_idx: 0,
            desc_virt,
            buf_virt,
            buf_phys,
        }
    }

    pub fn desc(&self, idx: u32) -> *mut Mt76Desc {
        unsafe { (self.desc_virt as *mut Mt76Desc).add(idx as usize) }
    }

    pub fn buf(&self, idx: u32) -> *mut u8 {
        unsafe { (self.buf_virt as *mut u8).add(idx as usize * 2048) }
    }
}

// ============================================================================
// DMA Functions on Mt7996Dev
// ============================================================================

/// Diagnostic result from TX free processing
pub struct TxFreeResult {
    pub tokens_freed: u32,
    pub entries: u32,
    pub txrx_notify_count: u32,
    /// First TXRX_NOTIFY: RXD[0..7] + tx_info[0..3] for debugging
    pub first_rxd: [u32; 8],
    pub first_payload: [u32; 4],
}

/// Header-translated RX data frame copied from the DMA buffer.
/// Frame data lives in the caller's copy buffer at `copy_offset..copy_offset+len`.
pub struct RxDataFrame {
    /// Offset into the caller-provided copy buffer where frame data starts
    pub copy_offset: u32,
    /// Frame length in bytes (Ethernet header + payload)
    pub len: u16,
}

/// RX Null Function / PM status notification extracted from non-HDR_TRANS data frames.
/// Used for power save state tracking.
pub struct RxPmEvent {
    /// Source MAC of the STA
    pub mac: [u8; 6],
    /// PM bit from FC byte 1 bit 4: true = entering PS, false = exiting PS
    pub pm: bool,
}

impl Mt7996Dev {
    // ========================================================================
    // mt7996_dma_disable() — EXACT Linux translation
    // Source: dma.c:545-585
    // ========================================================================

    pub fn mt7996_dma_disable(&self, reset: bool) {
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        if reset {
            self.mt76_clear(MT_WFDMA0_RST,
                MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
            self.mt76_set(MT_WFDMA0_RST,
                MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);

            if self.has_hif2 {
                self.mt76_clear(MT_WFDMA0_RST + hif1_ofs,
                    MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
                self.mt76_set(MT_WFDMA0_RST + hif1_ofs,
                    MT_WFDMA0_RST_DMASHDL_ALL_RST | MT_WFDMA0_RST_LOGIC_RST);
            }
        }

        let glo_clear_bits = MT_WFDMA0_GLO_CFG_TX_DMA_EN |
            MT_WFDMA0_GLO_CFG_RX_DMA_EN |
            MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
            MT_WFDMA0_GLO_CFG_OMIT_RX_INFO |
            MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2;
        self.mt76_clear(MT_WFDMA0_GLO_CFG, glo_clear_bits);

        if self.has_hif2 {
            self.mt76_clear(MT_WFDMA0_GLO_CFG + hif1_ofs, glo_clear_bits);
        }
    }

    // ========================================================================
    // mt7996_dma_start() — EXACT Linux translation
    // Source: dma.c:587-625
    // ========================================================================

    pub fn mt7996_dma_start(&self, reset: bool, _wed_reset: bool) {
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        if !reset {
            let bits_to_set = MT_WFDMA0_GLO_CFG_TX_DMA_EN |
                MT_WFDMA0_GLO_CFG_RX_DMA_EN |
                MT_WFDMA0_GLO_CFG_OMIT_TX_INFO |
                MT_WFDMA0_GLO_CFG_OMIT_RX_INFO_PFET2 |
                MT_WFDMA0_GLO_CFG_EXT_EN;

            self.mt76_set(MT_WFDMA0_GLO_CFG, bits_to_set);
            if self.has_hif2 {
                self.mt76_set(MT_WFDMA0_GLO_CFG + hif1_ofs, bits_to_set);
            }
        }

        // Interrupt mask: Linux dma.c:327-355 mt7996_dma_start()
        // CRITICAL: The WFDMA engine checks the interrupt mask to decide which
        // rings to deliver frames to. Without MT_INT_RX_DONE_BAND0 enabled,
        // over-the-air frames are NEVER delivered to the BAND0 data RX ring.
        // Without MT_INT_RX_DONE_WA_MAIN, TX free notifications don't flow,
        // causing firmware token exhaustion and silent TX drops.
        let irq_mask = MT_INT_MCU_CMD
            | MT_INT_RX_DONE_MCU           // MCU WM + WA RX
            | MT_INT_TX_DONE_MCU           // MCU TX done
            | MT_INT_RX_DONE_BAND0         // BAND0 data RX — over-the-air frames!
            | MT_INT_RX_DONE_WA_MAIN       // WA_MAIN — TX free notifications
            | MT_INT_TX_DONE_BAND0;        // TX BAND0 done

        self.mt76_wr(MT_INT_MASK_CSR, irq_mask);
        if self.has_hif2 { self.mt76_wr(MT_INT1_MASK_CSR, irq_mask); }

        self.mt76_wr(MT_INT_MASK_CSR, 0);
        if self.has_hif2 { self.mt76_wr(MT_INT1_MASK_CSR, 0); }

        let int_src = self.mt76_rr(MT_INT_SOURCE_CSR);
        self.mt76_wr(MT_INT_SOURCE_CSR, int_src);
        if self.has_hif2 {
            let int_src1 = self.mt76_rr(MT_INT1_SOURCE_CSR);
            self.mt76_wr(MT_INT1_SOURCE_CSR, int_src1);
        }

        self.mt76_wr(MT_INT_MASK_CSR, irq_mask);
        if self.has_hif2 { self.mt76_wr(MT_INT1_MASK_CSR, irq_mask); }
    }

    // ========================================================================
    // __mt7996_dma_prefetch() — EXACT Linux translation
    // Source: dma.c:167-240
    // ========================================================================

    fn __mt7996_dma_prefetch(&self, ofs: u32) {
        // TX prefetch registers
        self.mt76_wr(MT_WFDMA0_BASE + 0x640 + ofs, 0x00000002);
        self.mt76_wr(MT_WFDMA0_BASE + 0x644 + ofs, 0x00200002);
        self.mt76_wr(MT_WFDMA0_BASE + 0x648 + ofs, 0x00400008);
        self.mt76_wr(MT_WFDMA0_BASE + 0x64c + ofs, 0x00c00008);
        self.mt76_wr(MT_WFDMA0_BASE + 0x650 + ofs, 0x01400002);
        self.mt76_wr(MT_WFDMA0_BASE + 0x654 + ofs, 0x01600008);

        // RX prefetch registers
        self.mt76_wr(MT_WFDMA0_BASE + 0x680 + ofs, 0x01e00002);
        self.mt76_wr(MT_WFDMA0_BASE + 0x684 + ofs, 0x02000002);
        self.mt76_wr(MT_WFDMA0_BASE + 0x688 + ofs, 0x02200002);
        self.mt76_wr(MT_WFDMA0_BASE + 0x68c + ofs, 0x02400002);
        self.mt76_wr(MT_WFDMA0_BASE + 0x690 + ofs, 0x02600010);
        self.mt76_wr(MT_WFDMA0_BASE + 0x694 + ofs, 0x03600010);

        self.mt76_set(WF_WFDMA0_GLO_CFG_EXT1 + ofs, WF_WFDMA0_GLO_CFG_EXT1_CALC_MODE);
    }

    // ========================================================================
    // mt7996_dma_prefetch() — EXACT Linux translation
    // Source: dma.c:520-523
    // ========================================================================

    pub fn mt7996_dma_prefetch(&self) {
        udebug!("dma", "prefetch"; hif = 1, ofs = 0);
        self.__mt7996_dma_prefetch(0);
        if self.has_hif2 {
            udebug!("dma", "prefetch"; hif = 2, ofs = HIF1_OFS);
            self.__mt7996_dma_prefetch(HIF1_OFS);
        }
    }

    // ========================================================================
    // mt7996_dma_enable() — EXACT Linux translation
    // Source: dma.c:627-754
    // ========================================================================

    pub fn mt7996_dma_enable(&self, reset: bool) {
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        self.mt76_wr(MT_WFDMA0_RST_DTX_PTR, !0u32);
        if self.has_hif2 {
            self.mt76_wr(MT_WFDMA0_RST_DTX_PTR + hif1_ofs, !0u32);
        }

        self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG0, 0);
        self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG1, 0);
        self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG2, 0);
        if self.has_hif2 {
            self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG0 + hif1_ofs, 0);
            self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG1 + hif1_ofs, 0);
            self.mt76_wr(MT_WFDMA0_PRI_DLY_INT_CFG2 + hif1_ofs, 0);
        }

        self.mt7996_dma_prefetch();

        self.mt76_set(MT_WFDMA0_BUSY_ENA,
            MT_WFDMA0_BUSY_ENA_TX_FIFO0 |
            MT_WFDMA0_BUSY_ENA_TX_FIFO1 |
            MT_WFDMA0_BUSY_ENA_RX_FIFO);

        if self.has_hif2 {
            self.mt76_set(MT_WFDMA0_BUSY_ENA + hif1_ofs,
                MT_WFDMA0_PCIE1_BUSY_ENA_TX_FIFO0 |
                MT_WFDMA0_PCIE1_BUSY_ENA_TX_FIFO1 |
                MT_WFDMA0_PCIE1_BUSY_ENA_RX_FIFO);
        }

        if !self.mt76_poll(MT_WFDMA_EXT_CSR_HIF_MISC,
            MT_WFDMA_EXT_CSR_HIF_MISC_BUSY, 0, 1000) {
            uwarn!("dma", "hif_misc_busy_timeout");
        }

        self.mt76_set(WF_WFDMA0_GLO_CFG_EXT0,
            WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD |
            WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);

        self.mt76_set(WF_WFDMA0_GLO_CFG_EXT1,
            WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);

        self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_45_TH, 0xc000c);
        self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_67_TH, 0x10008);
        self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_89_TH, 0x10008);
        self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_RRO_TH, 0x20);

        if self.has_hif2 {
            self.mt76_set(WF_WFDMA0_GLO_CFG_EXT0 + hif1_ofs,
                WF_WFDMA0_GLO_CFG_EXT0_RX_WB_RXD |
                WF_WFDMA0_GLO_CFG_EXT0_WED_MERGE_MODE);

            self.mt76_set(WF_WFDMA0_GLO_CFG_EXT1 + hif1_ofs,
                WF_WFDMA0_GLO_CFG_EXT1_TX_FCTRL_MODE);

            self.mt76_set(MT_WFDMA_HOST_CONFIG,
                MT_WFDMA_HOST_CONFIG_PDMA_BAND);

            self.mt76_clear(MT_WFDMA_HOST_CONFIG,
                MT_WFDMA_HOST_CONFIG_BAND0_PCIE1 |
                MT_WFDMA_HOST_CONFIG_BAND1_PCIE1 |
                MT_WFDMA_HOST_CONFIG_BAND2_PCIE1);

            self.mt76_set(MT_WFDMA_HOST_CONFIG,
                MT_WFDMA_HOST_CONFIG_BAND2_PCIE1);

            self.mt76_rmw(MT_WFDMA_AXI_R2A_CTRL,
                MT_WFDMA_AXI_R2A_CTRL_OUTSTAND_MASK, 0x14);

            self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_45_TH + hif1_ofs, 0xc000c);
            self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_67_TH + hif1_ofs, 0x10008);
            self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_89_TH + hif1_ofs, 0x10008);
            self.mt76_wr(MT_WFDMA0_PAUSE_RX_Q_RRO_TH + hif1_ofs, 0x20);
        }

        if self.has_hif2 {
            self.mt76_set(MT_WFDMA0_RX_INT_PCIE_SEL, MT_WFDMA0_RX_INT_SEL_RING3);
        }

        self.mt7996_dma_start(reset, true);
    }

    // ========================================================================
    // Queue allocation — EXACT Linux translation
    // ========================================================================

    pub fn program_queue(&self, regs_base: u32, hw_idx: u32, desc_phys: u64, ndesc: u32) {
        let desc_lo = desc_phys as u32;

        self.mt76_wr(regs_base + MT_QUEUE_CPU_IDX, 0);
        self.mt76_wr(regs_base + MT_QUEUE_DMA_IDX, 0);

        self.mt76_wr(regs_base + MT_QUEUE_DESC_BASE, desc_lo);
        self.mt76_wr(regs_base + MT_QUEUE_RING_SIZE, ndesc);

        let _ = self.mt76_rr(regs_base + MT_QUEUE_DMA_IDX);
    }

    pub fn init_mcu_queue(&self, hw_idx: u32, ring_base: u32, ndesc: u32, desc_phys: u64, desc_virt: u64) {
        let regs_base = ring_base + hw_idx * MT_RING_SIZE;

        for i in 0..ndesc as usize {
            let desc_ptr = (desc_virt as *mut Mt76Desc).wrapping_add(i);
            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, MT_DMA_CTL_DMA_DONE);
            }
        }

        self.program_queue(regs_base, hw_idx, desc_phys, ndesc);
    }

    pub fn init_rx_queue(&self, hw_idx: u32, ring_base: u32, ndesc: u32, desc_phys: u64, desc_virt: u64) {
        let regs_base = ring_base + hw_idx * MT_RING_SIZE;

        for i in 0..ndesc as usize {
            let desc_ptr = (desc_virt as *mut Mt76Desc).wrapping_add(i);
            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, MT_DMA_CTL_DMA_DONE);
            }
        }

        self.program_queue(regs_base, hw_idx, desc_phys, ndesc);
    }

    // ========================================================================
    // mt76_dma_rx_fill() — Fill RX queue with buffers
    // Source: dma.c:684-694
    // ========================================================================

    /// Fill RX queue with buffers, matching Linux mt76_dma_rx_fill().
    ///
    /// Fills up to ndesc-1 slots (one-empty-slot convention: hardware
    /// treats DMA_IDX == CPU_IDX as "full", so we leave one slot empty
    /// to distinguish full from empty). Tracks head/queued counters.
    ///
    /// Called at init (fills ndesc-1 from 0) and after processing
    /// (refills consumed slots).
    ///
    /// Source: dma.c:684-694 mt76_dma_rx_fill() + mt76_dma_kick_queue()
    pub fn rx_fill(&self, q: &mut RxQueueInfo) {
        let ndesc = q.ndesc;
        let mut filled = 0u32;

        while q.queued < ndesc - 1 {
            let i = q.head as usize;
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let buf_phys = q.buf_phys + (i as u64 * q.buf_size as u64);

            // Linux mt76_dma_add_rx_buf(): ctrl = SD_LEN0(buf_size), NO LAST_SEC0.
            // LAST_SEC0 is hardware-generated on RX to indicate packet boundaries.
            // DMA_DONE is cleared (0) so hardware can set it after writing.
            let ctrl = (q.buf_size as u32) << 16;

            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, dma_addr_lo(buf_phys));
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, dma_addr_hi(buf_phys));
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, ctrl);
            }

            q.head = (q.head + 1) % ndesc;
            q.queued += 1;
            filled += 1;
        }

        if filled > 0 {
            let desc_bytes = ndesc as usize * core::mem::size_of::<Mt76Desc>();
            flush_buffer(q.desc_virt, desc_bytes);

            dma_wmb();

            // Linux mt76_dma_kick_queue(): write CPU_IDX = q->head
            // This tells hardware "buffers are valid up to head-1;
            // you can DMA from tail to head-1".
            self.mt76_wr(q.regs_base + MT_QUEUE_CPU_IDX, q.head);
        }
    }

    /// Drain an RX ring: dequeue all DMA_DONE entries and refill.
    ///
    /// Returns number of entries drained. Uses DMA_DONE bit like all
    /// other dequeue paths — no index comparison.
    pub fn rx_drain(&self, q: &mut RxQueueInfo) -> u32 {
        let mut count = 0u32;

        // Dequeue all ready entries
        while q.queued > 0 {
            let i = q.tail as usize;
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let ctrl = unsafe { core::ptr::read_volatile(&(*desc_ptr).ctrl) };

            if ctrl & MT_DMA_CTL_DMA_DONE == 0 { break; }

            // Clear DMA_DONE so hardware can reuse this descriptor.
            // dma.c:224 mt76_dma_rx_fill_buf() — ctrl = buf_size, DMA_DONE=0
            unsafe {
                core::ptr::write_volatile(
                    &mut (*desc_ptr).ctrl,
                    (q.buf_size as u32) << 16,
                );
            }

            q.tail = (q.tail + 1) % q.ndesc;
            q.queued -= 1;
            count += 1;
        }

        // Refill consumed slots
        if count > 0 {
            self.rx_fill(q);
        }
        count
    }

    /// Soft drain: dequeue DMA_DONE entries (advance tail) and refill,
    /// without resetting descriptor buffer pointers first.
    /// Use for event rings where the firmware may need buffer contents intact.
    pub fn rx_advance(&self, q: &mut RxQueueInfo) -> u32 {
        let mut count = 0u32;

        while q.queued > 0 {
            let i = q.tail as usize;
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let ctrl = unsafe { core::ptr::read_volatile(&(*desc_ptr).ctrl) };

            if ctrl & MT_DMA_CTL_DMA_DONE == 0 { break; }

            // Clear DMA_DONE so hardware can reuse this descriptor.
            unsafe {
                core::ptr::write_volatile(
                    &mut (*desc_ptr).ctrl,
                    (q.buf_size as u32) << 16,
                );
            }

            q.tail = (q.tail + 1) % q.ndesc;
            q.queued -= 1;
            count += 1;
        }

        if count > 0 {
            self.rx_fill(q);
        }
        count
    }

    // ========================================================================
    // rx_process_mcu() — Process MCU event queue (WM/WA RX rings)
    // Reads buffer content, dispatches to event::process_mcu_event()
    // ========================================================================

    /// Process MCU events from WM or WA RX queue.
    /// Reads each consumed descriptor's buffer and passes to event parser.
    /// Returns number of entries processed.
    ///
    /// Uses Linux mt76_dma_dequeue() model: check DMA_DONE at tail,
    /// advance tail, decrement queued. After processing, rx_fill()
    /// refills consumed slots and kicks CPU_IDX.
    ///
    /// Source: dma.c:593-620 mt76_dma_rx_process()
    pub fn rx_process_mcu(&self, q: &mut RxQueueInfo, counters: &mut RxMibCounters) -> u32 {
        let mut count = 0u32;

        // Dequeue loop: check DMA_DONE at tail
        while q.queued > 0 {
            let i = q.tail as usize;
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let ctrl = unsafe { core::ptr::read_volatile(&(*desc_ptr).ctrl) };

            // Linux dma.c:593 — entry not yet DMA'd by firmware
            if ctrl & MT_DMA_CTL_DMA_DONE == 0 { break; }

            // Read buffer content — DMA pool is NORMAL_NC, no cache invalidate needed
            let buf_base = q.buf_virt + (i as u64 * q.buf_size as u64);
            let buf_len = q.buf_size as usize;
            let buf = unsafe {
                core::slice::from_raw_parts(buf_base as *const u8, buf_len)
            };

            // Process MCU event
            event::process_mcu_event(buf, counters);

            // Clear DMA_DONE so hardware can reuse this descriptor.
            unsafe {
                core::ptr::write_volatile(
                    &mut (*desc_ptr).ctrl,
                    (q.buf_size as u32) << 16,
                );
            }

            // Advance tail, decrement queued
            q.tail = (q.tail + 1) % q.ndesc;
            q.queued -= 1;
            count += 1;
        }

        // Refill consumed slots and kick CPU_IDX
        if count > 0 {
            self.rx_fill(q);
        }
        count
    }

    // ========================================================================
    // rx_classify() — Process data RX queue with frame classification
    // Reads RXD0-RXD3, classifies frame type, updates MIB counters
    // ========================================================================

    /// Process data RX queue with frame classification.
    /// For each consumed descriptor, reads RXD words, classifies the frame,
    /// and updates software MIB counters.
    ///
    /// Uses DMA_DONE dequeue model (Linux mt76_dma_dequeue). After the
    /// dequeue loop, rx_fill() refills consumed slots and kicks CPU_IDX.
    ///
    /// Management frames (ProbeReq, Auth, AssocReq, Deauth, Disassoc) are
    /// collected as `RxMgmtFrame` for the AP state machine.
    ///
    /// Header-translated data frames (firmware sets HDR_TRANS bit) are
    /// copied into `data_buf` before descriptor reset.
    ///
    /// Returns (entries_processed, mgmt_frame_count, data_frame_count, pm_event_count).
    pub fn rx_classify(&self, q: &mut RxQueueInfo, counters: &mut RxMibCounters,
                       mgmt_frames: &mut [wifi80211::types::RxMgmtFrame],
                       max_mgmt: usize,
                       data_frames: &mut [RxDataFrame],
                       max_data: usize,
                       data_buf: &mut [u8],
                       pm_events: &mut [RxPmEvent],
                       max_pm: usize) -> (u32, usize, usize, usize) {
        let mut data_buf_pos: u32 = 0;

        let mut count = 0u32;
        let mut mgmt_count = 0usize;
        let mut data_count = 0usize;
        let mut pm_count = 0usize;

        while q.queued > 0 {
            let i = q.tail as usize;

            // Check DMA_DONE before processing
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let ctrl = unsafe { core::ptr::read_volatile(&(*desc_ptr).ctrl) };
            if ctrl & MT_DMA_CTL_DMA_DONE == 0 { break; }
            let buf_base = q.buf_virt + (i as u64 * q.buf_size as u64);
            let buf_ptr = buf_base as *const u32;

            // Read RXD0-RXD3 (first 16 bytes of buffer)
            let rxd0 = unsafe { core::ptr::read_volatile(buf_ptr) };
            let rxd1 = unsafe { core::ptr::read_volatile(buf_ptr.add(1)) };
            let rxd2 = unsafe { core::ptr::read_volatile(buf_ptr.add(2)) };
            let rxd3 = unsafe { core::ptr::read_volatile(buf_ptr.add(3)) };

            // Classify frame
            let result = event::classify_rx_frame(rxd0, rxd1, rxd2, rxd3);
            let mut class = result.class;

            // For ALL normal frames (Data or Mgmt), read FC byte to classify.
            // The NDATA bit in RXD2 is unreliable — unicast management frames
            // (auth, assoc) may arrive with NDATA=0, getting misclassified as Data.
            // Always check the 802.11 FC field for PKT_TYPE_NORMAL frames.
            //
            // IMPORTANT: Skip FC byte check when HDR_TRANS is set — firmware has
            // replaced the 802.11 header with an Ethernet header, so frame_ofs
            // points at the Ethernet DA, not the 802.11 FC byte. Reading it as FC
            // would misclassify data frames as management (e.g. fc0=0x00 → AssocReq).
            let mut frame_ofs: usize = 0;
            let hdr_trans = rxd2 & MT_RXD2_NORMAL_HDR_TRANS != 0;
            if matches!(class, RxFrameClass::Mgmt | RxFrameClass::Data) {
                // Navigate past RXD groups to find 802.11 header (or Ethernet header if HDR_TRANS)
                // Group order: G4(16), G1(16), G2(16), G3(16), G5(96)
                frame_ofs = 32; // Base RXD size
                if rxd1 & MT_RXD1_NORMAL_GROUP_4 != 0 { frame_ofs += 16; }
                if rxd1 & MT_RXD1_NORMAL_GROUP_1 != 0 { frame_ofs += 16; }
                if rxd1 & MT_RXD1_NORMAL_GROUP_2 != 0 { frame_ofs += 16; }
                if rxd1 & MT_RXD1_NORMAL_GROUP_3 != 0 { frame_ofs += 16; }
                if rxd1 & MT_RXD1_NORMAL_GROUP_5 != 0 { frame_ofs += 96; }
                let remove_pad = ((rxd2 >> 13) & 0x7) as usize;
                frame_ofs += 2 * remove_pad;

                // Only read FC bytes for non-header-translated frames
                if !hdr_trans && frame_ofs + 2 < q.buf_size as usize {
                    let fc0 = unsafe {
                        core::ptr::read_volatile((buf_base as *const u8).add(frame_ofs))
                    };
                    let fc1 = unsafe {
                        core::ptr::read_volatile((buf_base as *const u8).add(frame_ofs + 1))
                    };
                    let fc_type = (fc0 >> 2) & 0x3;
                    if fc_type == 0 {
                        // Management frame — refine subtype
                        class = event::refine_mgmt_subtype(fc0);
                    } else if fc_type == 2 {
                        // Data frame (non-HDR_TRANS). Extract PM bit for PS tracking.
                        // Subtype 4 = Null Function, subtype 12 = QoS Null
                        let subtype = (fc0 >> 4) & 0xF;
                        if (subtype == 4 || subtype == 12) && pm_count < max_pm {
                            let pm_bit = (fc1 & 0x10) != 0; // FC byte 1 bit 4 = PM
                            // Extract source MAC (addr2 at offset 10)
                            let addr2_ofs = frame_ofs + 10;
                            if addr2_ofs + 6 <= q.buf_size as usize {
                                let mut mac = [0u8; 6];
                                let src = unsafe {
                                    core::slice::from_raw_parts(
                                        (buf_base as *const u8).add(addr2_ofs), 6,
                                    )
                                };
                                mac.copy_from_slice(src);
                                pm_events[pm_count] = RxPmEvent { mac, pm: pm_bit };
                                pm_count += 1;
                            }
                        }
                    }
                }
            }

            // Extract RX PHY info from Group 3 (P-RXV).
            // Group order: G4(16), G1(16), G2(16), G3(16 bytes = 4 DWORDs).
            // P-RXV DW0: rate[6:0], nss[9:7], bw[11:10] (partial), mode[14:12],
            //            gi[16:15], stbc[22]
            // P-RXV DW1: bw full [14:11] (connac3)
            // P-RXV DW3: RCPI chain0[7:0], chain1[15:8], chain2[23:16], chain3[31:24]
            // Source: Linux mt76/mac.c mt76_connac3_mac_fill_rx(), mt76/mt7996/mac.c:14
            let phy: wifi80211::types::RxPhyInfo = if rxd1 & MT_RXD1_NORMAL_GROUP_3 != 0 {
                let mut g3_ofs: usize = 32;
                if rxd1 & MT_RXD1_NORMAL_GROUP_4 != 0 { g3_ofs += 16; }
                if rxd1 & MT_RXD1_NORMAL_GROUP_1 != 0 { g3_ofs += 16; }
                if rxd1 & MT_RXD1_NORMAL_GROUP_2 != 0 { g3_ofs += 16; }
                if g3_ofs + 16 <= q.buf_size as usize {
                    let g3_dw0 = unsafe { core::ptr::read_volatile(buf_ptr.add(g3_ofs / 4)) };
                    let g3_dw1 = unsafe { core::ptr::read_volatile(buf_ptr.add((g3_ofs + 4) / 4)) };
                    let g3_dw3 = unsafe { core::ptr::read_volatile(buf_ptr.add((g3_ofs + 12) / 4)) };
                    // Per-chain RCPI (0-255, 0=unused)
                    let rcpi = [
                        (g3_dw3 & 0xFF) as u8,
                        ((g3_dw3 >> 8) & 0xFF) as u8,
                        ((g3_dw3 >> 16) & 0xFF) as u8,
                        ((g3_dw3 >> 24) & 0xFF) as u8,
                    ];
                    // RSSI from chain 0 RCPI. Source: Linux mac.c:14
                    let rssi = (((rcpi[0] as i32) - 220) / 2).clamp(-128, 0) as i8;
                    // Rate info from P-RXV DW0
                    let rate = (g3_dw0 & 0x7F) as u8;
                    let nss = ((g3_dw0 >> 7) & 0x7) as u8;
                    let mode = ((g3_dw0 >> 12) & 0x7) as u8;
                    let gi = ((g3_dw0 >> 15) & 0x3) as u8;
                    let stbc = (g3_dw0 >> 22) & 1 != 0;
                    // BW from DW1[14:11] (connac3 uses wider field)
                    // Source: Linux mt76/connac3_mac.h MT_PRXV_FRAME_MODE
                    let bw = ((g3_dw1 >> 11) & 0xF) as u8;
                    wifi80211::types::RxPhyInfo { rssi, rcpi, rate, nss, bw, mode, gi, stbc }
                } else {
                    wifi80211::types::RxPhyInfo::UNKNOWN
                }
            } else {
                wifi80211::types::RxPhyInfo::UNKNOWN
            };
            let rssi = phy.rssi;

            // Collect management frames for AP state machine
            // ProbeReq, Auth, AssocReq, Deauth, Disassoc, Action all need
            // addr2, addr3, and frame body for protocol processing.
            let is_ap_mgmt = matches!(class,
                RxFrameClass::ProbeReq | RxFrameClass::Auth |
                RxFrameClass::AssocReq | RxFrameClass::ReassocReq |
                RxFrameClass::Deauth | RxFrameClass::Disassoc |
                RxFrameClass::Action);
            if is_ap_mgmt && mgmt_count < max_mgmt {
                let addr2_ofs = frame_ofs + 10; // FC(2) + Duration(2) + DA(6)
                let addr3_ofs = frame_ofs + 16; // FC(2) + Duration(2) + DA(6) + SA(6)
                let body_ofs = frame_ofs + 24;  // After 24-byte MAC header
                if addr3_ofs + 6 <= q.buf_size as usize {
                    let mut addr2 = [0u8; 6];
                    let mut addr3 = [0u8; 6];
                    let src2 = unsafe {
                        core::slice::from_raw_parts((buf_base as *const u8).add(addr2_ofs), 6)
                    };
                    addr2.copy_from_slice(src2);
                    let src3 = unsafe {
                        core::slice::from_raw_parts((buf_base as *const u8).add(addr3_ofs), 6)
                    };
                    addr3.copy_from_slice(src3);
                    // Skip frames with all-zero addr2 — false positive from
                    // stale buffer (fc0=0x00 = AssocReq subtype). No valid client
                    // has MAC 00:00:00:00:00:00.
                    if addr2 != [0u8; 6] {
                        let subtype = match class {
                            RxFrameClass::ProbeReq => wifi80211::types::MgmtSubtype::ProbeReq,
                            RxFrameClass::Auth => wifi80211::types::MgmtSubtype::Auth,
                            RxFrameClass::AssocReq => wifi80211::types::MgmtSubtype::AssocReq,
                            RxFrameClass::ReassocReq => wifi80211::types::MgmtSubtype::ReassocReq,
                            RxFrameClass::Deauth => wifi80211::types::MgmtSubtype::Deauth,
                            RxFrameClass::Disassoc => wifi80211::types::MgmtSubtype::Disassoc,
                            RxFrameClass::Action => wifi80211::types::MgmtSubtype::Action,
                            _ => unreachable!(),
                        };
                        // Copy frame body after MAC header for protocol parsing
                        // (ADDBA params, assoc IEs, etc.)
                        let mut body = [0u8; wifi80211::types::MAX_MGMT_BODY_LEN];
                        let mut body_len = 0u16;
                        let byte_cnt = (rxd0 & 0xFFFF) as usize;
                        if body_ofs < byte_cnt && body_ofs < q.buf_size as usize {
                            let avail = (byte_cnt - body_ofs).min((q.buf_size as usize) - body_ofs);
                            let copy_len = avail.min(wifi80211::types::MAX_MGMT_BODY_LEN);
                            if copy_len > 0 {
                                let body_src = unsafe {
                                    core::slice::from_raw_parts(
                                        (buf_base as *const u8).add(body_ofs), copy_len,
                                    )
                                };
                                body[..copy_len].copy_from_slice(body_src);
                                body_len = copy_len as u16;
                            }
                        }
                        mgmt_frames[mgmt_count] = wifi80211::types::RxMgmtFrame {
                            subtype, addr2, addr3, rssi, phy, body, body_len,
                        };
                        mgmt_count += 1;
                    }
                }
            }

            // Collect header-translated data frames for the IP stack.
            // When HDR_TRANS is set, firmware has converted 802.11 → Ethernet
            // format in the DMA buffer starting at frame_ofs.
            // IMPORTANT: Copy frame data NOW, before descriptor reset below
            // returns this buffer to hardware.
            if rxd2 & MT_RXD2_NORMAL_HDR_TRANS != 0 && data_count < max_data && frame_ofs > 0 {
                // RXD0 bits[15:0] = RX_BYTE_CNT (total bytes including RXD groups)
                let byte_cnt = (rxd0 & 0xFFFF) as usize;
                if byte_cnt > frame_ofs && byte_cnt <= q.buf_size as usize {
                    let eth_len = byte_cnt - frame_ofs;
                    if eth_len >= 14 && eth_len <= 1514
                        && (data_buf_pos as usize + eth_len) <= data_buf.len()
                    {
                        // Copy from DMA buffer into caller's copy buffer
                        let src = unsafe {
                            core::slice::from_raw_parts(
                                (buf_base as *const u8).add(frame_ofs),
                                eth_len,
                            )
                        };
                        let dst_start = data_buf_pos as usize;
                        data_buf[dst_start..dst_start + eth_len].copy_from_slice(src);
                        data_frames[data_count] = RxDataFrame {
                            copy_offset: data_buf_pos,
                            len: eth_len as u16,
                        };
                        data_buf_pos += eth_len as u32;
                        data_count += 1;
                    }
                }
            }

            counters.record(class, result.addr_type, result.fcs_err);

            // Clear DMA_DONE so hardware can reuse this descriptor.
            unsafe {
                core::ptr::write_volatile(
                    &mut (*desc_ptr).ctrl,
                    (q.buf_size as u32) << 16,
                );
            }

            // Advance tail, decrement queued (descriptor refilled by rx_fill below)
            q.tail = (q.tail + 1) % q.ndesc;
            q.queued -= 1;
            count += 1;
        }

        // Refill consumed slots and kick CPU_IDX
        if count > 0 {
            self.rx_fill(q);
        }
        (count, mgmt_count, data_count, pm_count)
    }

    // ========================================================================
    // rx_process_tx_free() — Process TX free events from WA_MAIN/WA_TRI
    // Source: mt7996/mac.c:1312-1437 mt7996_mac_tx_free()
    // ========================================================================

    /// Process TX free notifications from the WA firmware.
    ///
    /// WA_MAIN (q3) and WA_TRI (q5) carry TX free events. Buffer format:
    ///   tx_free[0] = PKT_TYPE(31:27) | MSDU_CNT(25:16) | RX_BYTE(15:0)
    ///   tx_free[1] = VER(19:16) | ...
    ///   tx_free[2..] = PAIR/HEADER/MSDU entries
    ///
    /// NO 32-byte RXD header — the WA firmware writes tx_free data directly
    /// at buffer offset 0. Linux mt7996_mac_tx_free() receives skb->data
    /// pointing at the buffer start (no __skb_pull for WA events).
    ///
    /// Source: mt7996/mac.c:1312-1437 mt7996_mac_tx_free()
    pub fn rx_process_tx_free(&self, q: &mut RxQueueInfo) -> TxFreeResult {
        let mut result = TxFreeResult {
            tokens_freed: 0,
            entries: 0,
            txrx_notify_count: 0,
            first_rxd: [0; 8],
            first_payload: [0; 4],
        };

        while q.queued > 0 {
            let i = q.tail as usize;

            // Check DMA_DONE before processing
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let ctrl = unsafe { core::ptr::read_volatile(&(*desc_ptr).ctrl) };
            if ctrl & MT_DMA_CTL_DMA_DONE == 0 { break; }
            let buf_base = q.buf_virt + (i as u64 * q.buf_size as u64);

            // TX free data starts at offset 0 — NO RXD header on WA events.
            // tx_free[0] bits[31:27] = PKT_TYPE
            let tx_free = buf_base as *const u32;
            let dw0 = unsafe { core::ptr::read_volatile(tx_free) };
            let pkt_type = (dw0 >> MT_RXD0_PKT_TYPE_SHIFT) & 0x1F;
            result.entries += 1;

            if pkt_type == PKT_TYPE_TXRX_NOTIFY {
                result.txrx_notify_count += 1;

                // Capture first TXRX_NOTIFY buffer for diagnostics
                if result.txrx_notify_count == 1 {
                    for j in 0..8 {
                        result.first_rxd[j] = unsafe {
                            core::ptr::read_volatile(tx_free.add(j))
                        };
                    }
                }

                // RX_BYTE_CNT in bits[15:0] tells us total data length
                let byte_cnt = (dw0 & 0xFFFF) as usize;
                let max_dwords = (byte_cnt / 4).min(256);

                if max_dwords >= 2 {
                    let dw1 = unsafe { core::ptr::read_volatile(tx_free.add(1)) };

                    let msdu_cnt = (dw0 & MT_TXFREE0_MSDU_CNT_MASK) >> MT_TXFREE0_MSDU_CNT_SHIFT;
                    let ver = (dw1 & MT_TXFREE1_VER_MASK) >> MT_TXFREE1_VER_SHIFT;

                    // Capture ver in first_payload[0] for diagnostics
                    if result.txrx_notify_count == 1 {
                        result.first_payload[0] = ver;
                        result.first_payload[1] = msdu_cnt;
                        result.first_payload[2] = byte_cnt as u32;
                    }

                    // Source: mac.c:1340 — WARN_ON_ONCE(ver < 5)
                    // Parse entries regardless of ver for now (diagnostic)
                    let mut count = 0u32;
                    let mut di = 2usize; // Start at tx_free[2]

                    while count < msdu_cnt && di < max_dwords {
                        let info = unsafe { core::ptr::read_volatile(tx_free.add(di)) };
                        di += 1;

                        // PAIR marker — new WCID context, skip
                        if info & MT_TXFREE_INFO_PAIR != 0 { continue; }
                        // HEADER — retry/status stats, skip
                        if info & MT_TXFREE_INFO_HEADER != 0 { continue; }

                        // Two packed 15-bit MSDU IDs per DWORD
                        for shift in [0u32, 15] {
                            let msdu = (info >> shift) & MT_TXFREE_INFO_MSDU_ID;
                            if msdu == MT_TXFREE_INFO_MSDU_ID { continue; } // 0x7FFF = padding
                            count += 1;
                            result.tokens_freed += 1;
                        }
                    }
                }
            }

            // Clear DMA_DONE so hardware can reuse this descriptor.
            unsafe {
                core::ptr::write_volatile(
                    &mut (*desc_ptr).ctrl,
                    (q.buf_size as u32) << 16,
                );
            }

            // Advance tail, decrement queued (descriptor refilled by rx_fill below)
            q.tail = (q.tail + 1) % q.ndesc;
            q.queued -= 1;
        }

        // Refill consumed slots and kick CPU_IDX
        if result.entries > 0 {
            self.rx_fill(q);
        }
        result
    }

    // ========================================================================
    // tx_enqueue() — Enqueue a pre-built frame on a TX ring
    // Used for data/management frame transmission via TX BAND0
    // ========================================================================

    /// Enqueue a management frame on a CT-mode TX ring.
    ///
    /// The caller provides [TXD(32 bytes) | 802.11 frame(N bytes)].
    /// This function inserts a fw_txp (44 bytes) between TXD and frame,
    /// producing [TXD | fw_txp | frame] in the DMA buffer.
    ///
    /// CT mode requires the fw_txp so the MAC can find the frame data via
    /// buffer pointers. Without it, the MAC misinterprets raw frame bytes
    /// as TXP and silently drops the frame (tx_ok=0).
    ///
    /// Descriptor uses 2-buffer mode (Linux mt76_dma_tx_queue_skb):
    ///   buf0 = TXD + TXP (76 bytes), SD_LEN0 = 76
    ///   buf1 = frame header (first 72 bytes for FW parsing), SD_LEN1 = min(72, frame_len)
    ///   LAST_SEC1 set (not LAST_SEC0)
    ///
    /// Source: Linux mt76/dma.c mt76_dma_tx_queue_skb(), mt7996/mac.c mt7996_tx_prepare_skb()
    pub fn tx_enqueue(&self, ring: &mut TxRing, data: &[u8], token: u16) -> Result<(), i32> {
        if data.len() < MT_TXD_SIZE {
            return Err(-1);
        }

        let txd = &data[..MT_TXD_SIZE];
        let frame = &data[MT_TXD_SIZE..];
        let frame_len = frame.len();

        // Check if ring is full
        let next_idx = (ring.cpu_idx + 1) % ring.ndesc;
        let dma_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
        if next_idx == dma_idx {
            return Err(-1);
        }

        let idx = ring.cpu_idx;
        let buf = ring.buf(idx);
        let buf_phys = ring.buf_phys(idx);

        // Write TXD at offset 0 (32 bytes)
        unsafe { core::ptr::copy_nonoverlapping(txd.as_ptr(), buf, MT_TXD_SIZE); }

        // Build fw_txp at offset 32 (44 bytes)
        // Source: mt76_connac.h struct mt76_connac_fw_txp (packed)
        //   [0..1]  flags (le16)
        //   [2..3]  token (le16)
        //   [4]     bss_idx (u8)
        //   [5..6]  rept_wds_wcid (le16, packed/unaligned)
        //   [7]     nbuf (u8)
        //   [8..31] buf[0..5] (6 × le32)
        //   [32..43] len[0..5] (6 × le16)
        let txp = unsafe { buf.add(MT_TXD_SIZE) };
        unsafe { core::ptr::write_bytes(txp, 0, MT_FW_TXP_SIZE); }

        let frame_phys = buf_phys + MT_TXWI_SIZE as u64;

        // flags: APPLY_TXD | NONE_CIPHER | MGMT_FRAME | FROM_HOST
        // Source: mt7996/mac.c:1181-1190 mt7996_tx_prepare_skb()
        let flags: u16 = MT_CT_INFO_APPLY_TXD | MT_CT_INFO_NONE_CIPHER_FRAME
                       | MT_CT_INFO_MGMT_FRAME | MT_CT_INFO_FROM_HOST;
        unsafe { core::ptr::copy_nonoverlapping(flags.to_le_bytes().as_ptr(), txp, 2); }

        // token at offset 2 — firmware uses this to track TX completion
        // Source: mt7996/mac.c:1201 txp->fw.token = cpu_to_le16(id)
        unsafe { core::ptr::copy_nonoverlapping(token.to_le_bytes().as_ptr(), txp.add(2), 2); }

        // rept_wds_wcid = 0xfff (no WDS/no STA) at offset 5
        // Source: mt7996/mac.c:1202 — cpu_to_le16(0xfff) for broadcast
        let wcid: u16 = 0xfff;
        unsafe { core::ptr::copy_nonoverlapping(wcid.to_le_bytes().as_ptr(), txp.add(5), 2); }

        // nbuf = 1 at offset 7
        unsafe { *txp.add(7) = 1; }

        // buf[0] at offset 8: frame data physical address (lower 32 bits)
        unsafe { core::ptr::copy_nonoverlapping(
            (frame_phys as u32).to_le_bytes().as_ptr(), txp.add(8), 4); }

        // len[0] at offset 32: frame length + upper 4 address bits
        // Source: mt76_connac3_mac.h:294-295 MT_TXP_BUF_LEN | MT_TXP_DMA_ADDR_H
        let addr_h = ((frame_phys >> 32) & 0xF) as u16;
        let len0: u16 = (frame_len as u16 & 0x0FFF) | (addr_h << 12);
        unsafe { core::ptr::copy_nonoverlapping(len0.to_le_bytes().as_ptr(), txp.add(32), 2); }

        // Write frame data at offset 76 (after TXD + TXP)
        unsafe { core::ptr::copy_nonoverlapping(
            frame.as_ptr(), buf.add(MT_TXWI_SIZE), frame_len); }

        // 2-buffer descriptor: buf0 = TXD+TXP, buf1 = frame header
        // Source: Linux dma.c mt76_dma_add_buf() with nbufs=2
        // SD_LEN1 is ALWAYS MT_CT_PARSE_LEN (72), not the actual frame length.
        // Linux mac.c:1208 unconditionally overrides: tx_info->buf[1].len = MT_CT_PARSE_LEN
        // The firmware expects exactly 72 bytes for the CT parse header buffer.
        let sd_len0 = MT_TXWI_SIZE as u32;
        let sd_len1 = MT_CT_PARSE_LEN as u32;
        let ctrl_val = (sd_len0 << 16)
            | (sd_len1 & MT_DMA_CTL_SD_LEN1_MASK)
            | MT_DMA_CTL_LAST_SEC1;

        // info field carries upper address bits for BOTH buffers:
        //   SDP0_H (bits[3:0])   = buf_phys[35:32]  (TXWI address)
        //   SDP1_H (bits[19:16]) = frame_phys[35:32] (frame header address)
        // Source: Linux dma.c:332-339 mt76_dma_add_buf()
        let info_val = dma_addr_hi(buf_phys)
            | (dma_addr_hi(frame_phys) << 16);

        let desc = ring.desc(idx);
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(buf_phys));
            core::ptr::write_volatile(&mut (*desc).buf1, dma_addr_lo(frame_phys));
            core::ptr::write_volatile(&mut (*desc).info, info_val);
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        let total_write = MT_TXWI_SIZE + frame_len;
        flush_buffer(buf as u64, total_write);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        ring.cpu_idx = next_idx;
        dma_wmb();
        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        Ok(())
    }

    /// Enqueue an Ethernet (802.3) data frame for TX.
    ///
    /// For 802.3 TX mode (is_8023=true), the firmware handles 802.11 encapsulation.
    /// TXD is zeroed (firmware fills from CT info), fw_txp carries FROM_HOST and
    /// NONE_CIPHER (no APPLY_TXD, no MGMT_FRAME).
    ///
    /// Source: Linux mt7996/mac.c:1133-1208 mt7996_tx_prepare_skb()
    ///         (is_8023 path with pid < PACKET_ID_FIRST)
    pub fn tx_enqueue_data(&self, ring: &mut TxRing, frame: &[u8],
                           wcid: u16, token: u16) -> Result<(), i32> {
        let frame_len = frame.len();
        if frame_len < 14 || frame_len > 1514 {
            return Err(-1);
        }

        // Check if ring is full
        let next_idx = (ring.cpu_idx + 1) % ring.ndesc;
        let dma_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
        if next_idx == dma_idx {
            return Err(-2);
        }

        let idx = ring.cpu_idx;
        let buf = ring.buf(idx);
        let buf_phys = ring.buf_phys(idx);

        // TXD: zeroed for 802.3 mode — firmware fills from CT info
        // Source: mac.c:1133 — is_8023 with pid < PACKET_ID_FIRST skips mac_write_txwi
        unsafe { core::ptr::write_bytes(buf, 0, MT_TXD_SIZE); }

        // Build fw_txp at offset 32 (44 bytes)
        let txp = unsafe { buf.add(MT_TXD_SIZE) };
        unsafe { core::ptr::write_bytes(txp, 0, MT_FW_TXP_SIZE); }

        let frame_phys = buf_phys + MT_TXWI_SIZE as u64;

        // flags: FROM_HOST | NONE_CIPHER (no APPLY_TXD, no MGMT_FRAME)
        // Source: mac.c:1181 CT_INFO_FROM_HOST always set
        //         mac.c:1183-1184 APPLY_TXD only if !is_8023 or pid >= PACKET_ID_FIRST
        //         mac.c:1186-1187 NONE_CIPHER if no key
        let flags: u16 = MT_CT_INFO_FROM_HOST | MT_CT_INFO_NONE_CIPHER_FRAME;
        unsafe { core::ptr::copy_nonoverlapping(flags.to_le_bytes().as_ptr(), txp, 2); }

        // token at offset 2
        unsafe { core::ptr::copy_nonoverlapping(token.to_le_bytes().as_ptr(), txp.add(2), 2); }

        // rept_wds_wcid at offset 5 — actual STA WCID for unicast, 0xfff for broadcast
        unsafe { core::ptr::copy_nonoverlapping(wcid.to_le_bytes().as_ptr(), txp.add(5), 2); }

        // nbuf = 1 at offset 7
        unsafe { *txp.add(7) = 1; }

        // buf[0] at offset 8: frame data physical address (lower 32 bits)
        unsafe { core::ptr::copy_nonoverlapping(
            (frame_phys as u32).to_le_bytes().as_ptr(), txp.add(8), 4); }

        // len[0] at offset 32: frame length + upper 4 address bits
        let addr_h = ((frame_phys >> 32) & 0xF) as u16;
        let len0: u16 = (frame_len as u16 & 0x0FFF) | (addr_h << 12);
        unsafe { core::ptr::copy_nonoverlapping(len0.to_le_bytes().as_ptr(), txp.add(32), 2); }

        // Write Ethernet frame data at offset 76 (after TXD + TXP)
        unsafe { core::ptr::copy_nonoverlapping(
            frame.as_ptr(), buf.add(MT_TXWI_SIZE), frame_len); }

        // 2-buffer descriptor: buf0 = TXD+TXP, buf1 = frame header
        let sd_len0 = MT_TXWI_SIZE as u32;
        let sd_len1 = MT_CT_PARSE_LEN as u32;
        let ctrl_val = (sd_len0 << 16)
            | (sd_len1 & MT_DMA_CTL_SD_LEN1_MASK)
            | MT_DMA_CTL_LAST_SEC1;

        let info_val = dma_addr_hi(buf_phys)
            | (dma_addr_hi(frame_phys) << 16);

        let desc = ring.desc(idx);
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(buf_phys));
            core::ptr::write_volatile(&mut (*desc).buf1, dma_addr_lo(frame_phys));
            core::ptr::write_volatile(&mut (*desc).info, info_val);
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        let total_write = MT_TXWI_SIZE + frame_len;
        flush_buffer(buf as u64, total_write);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        ring.cpu_idx = next_idx;
        dma_wmb();
        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        Ok(())
    }

    /// Zero-copy TX: frame data stays in DataPort pool, DMA reads it directly.
    ///
    /// Same as tx_enqueue_data but frame_pool_phys points to the Ethernet frame
    /// in the DataPort shared memory pool (already DMA-accessible, NORMAL_NC).
    /// No frame data is copied — only the 76-byte TXD+fw_txp is written to the
    /// DMA buffer. The descriptor's buf1 points directly at pool memory.
    pub fn tx_enqueue_data_zerocopy(&self, ring: &mut TxRing,
                                     frame_pool_phys: u64, frame_len: usize,
                                     wcid: u16, token: u16) -> Result<(), i32> {
        if frame_len < 14 || frame_len > 1514 {
            return Err(-1);
        }

        // Check if ring is full
        let next_idx = (ring.cpu_idx + 1) % ring.ndesc;
        let dma_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
        if next_idx == dma_idx {
            return Err(-2);
        }

        let idx = ring.cpu_idx;
        let buf = ring.buf(idx);
        let buf_phys = ring.buf_phys(idx);

        // TXD: zeroed for 802.3 mode
        unsafe { core::ptr::write_bytes(buf, 0, MT_TXD_SIZE); }

        // Build fw_txp at offset 32 (44 bytes)
        let txp = unsafe { buf.add(MT_TXD_SIZE) };
        unsafe { core::ptr::write_bytes(txp, 0, MT_FW_TXP_SIZE); }

        // flags: FROM_HOST | NONE_CIPHER
        let flags: u16 = MT_CT_INFO_FROM_HOST | MT_CT_INFO_NONE_CIPHER_FRAME;
        unsafe { core::ptr::copy_nonoverlapping(flags.to_le_bytes().as_ptr(), txp, 2); }

        // token at offset 2
        unsafe { core::ptr::copy_nonoverlapping(token.to_le_bytes().as_ptr(), txp.add(2), 2); }

        // rept_wds_wcid at offset 5
        unsafe { core::ptr::copy_nonoverlapping(wcid.to_le_bytes().as_ptr(), txp.add(5), 2); }

        // nbuf = 1 at offset 7
        unsafe { *txp.add(7) = 1; }

        // buf[0] at offset 8: frame_pool_phys lower 32 bits
        unsafe { core::ptr::copy_nonoverlapping(
            (frame_pool_phys as u32).to_le_bytes().as_ptr(), txp.add(8), 4); }

        // len[0] at offset 32: frame length + upper 4 address bits of pool phys
        let addr_h = ((frame_pool_phys >> 32) & 0xF) as u16;
        let len0: u16 = (frame_len as u16 & 0x0FFF) | (addr_h << 12);
        unsafe { core::ptr::copy_nonoverlapping(len0.to_le_bytes().as_ptr(), txp.add(32), 2); }

        // NO frame data copy — pool memory is DMA-accessible

        // 2-buffer descriptor: buf0 = TXD+TXP (76B), buf1 = frame in pool
        let sd_len0 = MT_TXWI_SIZE as u32;
        let sd_len1 = MT_CT_PARSE_LEN as u32;
        let ctrl_val = (sd_len0 << 16)
            | (sd_len1 & MT_DMA_CTL_SD_LEN1_MASK)
            | MT_DMA_CTL_LAST_SEC1;

        let info_val = dma_addr_hi(buf_phys)
            | (dma_addr_hi(frame_pool_phys) << 16);

        let desc = ring.desc(idx);
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(buf_phys));
            core::ptr::write_volatile(&mut (*desc).buf1, dma_addr_lo(frame_pool_phys));
            core::ptr::write_volatile(&mut (*desc).info, info_val);
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        // Only flush TXD+fw_txp (76 bytes), not frame data (already non-cacheable)
        flush_buffer(buf as u64, MT_TXWI_SIZE);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        ring.cpu_idx = next_idx;
        dma_wmb();
        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        Ok(())
    }

    /// Sweep completed TX descriptors. Returns number of completed entries.
    ///
    /// Walks from ring.sweep_idx to the hardware DMA index, collecting tokens
    /// from completed descriptors (DMA_DONE bit set in ctrl). Each completed
    /// descriptor's token is written to `tokens_out`. Returns the count.
    /// Sweep completed TX descriptors. Returns number of completed entries.
    ///
    /// Walks from ring.sweep_idx to hardware DMA_IDX (like Linux mt76_dma_tx_cleanup).
    /// Linux does NOT check DMA_DONE for TX — DMA_IDX already indicates completion.
    /// Each completed descriptor's token is read from the fw_txp header.
    pub fn tx_sweep(&self, ring: &mut TxRing, tokens_out: &mut [u16]) -> usize {
        let dma_idx = self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX);
        let mut count = 0;

        while ring.sweep_idx != dma_idx && count < tokens_out.len() {
            // Read token from fw_txp at offset 2 in the buffer
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

    // ========================================================================
    // mt7996_dma_init() — EXACT Linux translation
    // Source: dma.c:599-854
    // ========================================================================

    /// Returns (mcu_wa_rx_buf_virt, mcu_wa_rx_buf_size, rx_queues, rx_queue_count) for MCU
    /// response parsing and RX ring management.
    pub fn mt7996_dma_init(&self, desc_phys: u64, desc_virt: u64, _desc_size: usize,
                       rx_buf_phys: u64, rx_buf_virt: u64, _rx_buf_size: usize) -> (u64, u32, [RxQueueInfo; NUM_RX_QUEUES], usize) {
        let hif1_ofs = if self.has_hif2 { HIF1_OFS } else { 0 };

        const DESC_SIZE: usize = 16;
        #[inline]
        const fn ring_bytes(ndesc: u32) -> usize {
            let raw = (ndesc as usize) * DESC_SIZE;
            (raw + 4095) & !4095
        }

        self.mt7996_dma_disable(true);

        let mut offset: usize = 0;
        let mut rx_queues: [RxQueueInfo; NUM_RX_QUEUES] = [RxQueueInfo::default(); NUM_RX_QUEUES];
        let mut rx_queue_idx: usize = 0;
        let mut rx_buf_offset: usize = 0;

        let tx_ring_base = MT_WFDMA0_TX_RING_BASE;
        let rx_ring_base = MT_WFDMA0_RX_RING_BASE;

        // TX BAND0
        self.init_mcu_queue(MT7996_TXQ_BAND0, tx_ring_base, MT7996_TX_RING_SIZE,
            desc_phys + offset as u64, desc_virt + offset as u64);
        offset += ring_bytes(MT7996_TX_RING_SIZE);
        udebug!("dma", "queue_init"; queue = "TX_BAND0", reg = tx_ring_base + MT7996_TXQ_BAND0 * MT_RING_SIZE);

        // TX MCU_WM
        self.init_mcu_queue(MT7996_TXQ_MCU_WM, tx_ring_base, MT7996_TX_MCU_RING_SIZE,
            desc_phys + offset as u64, desc_virt + offset as u64);
        offset += ring_bytes(MT7996_TX_MCU_RING_SIZE);
        udebug!("dma", "queue_init"; queue = "TX_MCU_WM", reg = tx_ring_base + MT7996_TXQ_MCU_WM * MT_RING_SIZE);

        // TX MCU_WA
        self.init_mcu_queue(MT7996_TXQ_MCU_WA, tx_ring_base, MT7996_TX_MCU_RING_SIZE,
            desc_phys + offset as u64, desc_virt + offset as u64);
        offset += ring_bytes(MT7996_TX_MCU_RING_SIZE);
        udebug!("dma", "queue_init"; queue = "TX_MCU_WA", reg = tx_ring_base + MT7996_TXQ_MCU_WA * MT_RING_SIZE);

        // TX FWDL
        self.init_mcu_queue(MT7996_TXQ_FWDL, tx_ring_base, MT7996_TX_FWDL_RING_SIZE,
            desc_phys + offset as u64, desc_virt + offset as u64);
        offset += ring_bytes(MT7996_TX_FWDL_RING_SIZE);
        udebug!("dma", "queue_init"; queue = "TX_FWDL", reg = tx_ring_base + MT7996_TXQ_FWDL * MT_RING_SIZE);

        // RX MCU_WM
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(MT7996_RXQ_MCU_WM, rx_ring_base, MT7996_RX_MCU_RING_SIZE,
            desc_phys + offset as u64, rx_desc_virt);
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_MCU_WM,
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WM * MT_RING_SIZE,
            ndesc: MT7996_RX_MCU_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_MCU_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
            head: 0, tail: 0, queued: 0,
        };
        rx_buf_offset += MT7996_RX_MCU_RING_SIZE as usize * MT7996_RX_MCU_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_MCU_RING_SIZE);

        // RX MCU_WA
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(MT7996_RXQ_MCU_WA, rx_ring_base, MT7996_RX_MCU_RING_SIZE_WA,
            desc_phys + offset as u64, rx_desc_virt);
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_MCU_WA,
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WA * MT_RING_SIZE,
            ndesc: MT7996_RX_MCU_RING_SIZE_WA,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_MCU_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
            head: 0, tail: 0, queued: 0,
        };
        rx_buf_offset += MT7996_RX_MCU_RING_SIZE_WA as usize * MT7996_RX_MCU_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_MCU_RING_SIZE_WA);

        // RX BAND0
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(MT7996_RXQ_BAND0, rx_ring_base, MT7996_RX_RING_SIZE,
            desc_phys + offset as u64, rx_desc_virt);
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_BAND0,
            regs_base: rx_ring_base + MT7996_RXQ_BAND0 * MT_RING_SIZE,
            ndesc: MT7996_RX_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
            head: 0, tail: 0, queued: 0,
        };
        rx_buf_offset += MT7996_RX_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_RING_SIZE);

        // RX WA_MAIN
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(MT7996_RXQ_MCU_WA_MAIN, rx_ring_base, MT7996_RX_MCU_RING_SIZE,
            desc_phys + offset as u64, rx_desc_virt);
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_MCU_WA_MAIN,
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WA_MAIN * MT_RING_SIZE,
            ndesc: MT7996_RX_MCU_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
            head: 0, tail: 0, queued: 0,
        };
        rx_buf_offset += MT7996_RX_MCU_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_MCU_RING_SIZE);

        // RX BAND2
        let rx_base_band2 = rx_ring_base + hif1_ofs;
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(MT7996_RXQ_BAND2, rx_base_band2, MT7996_RX_RING_SIZE,
            desc_phys + offset as u64, rx_desc_virt);
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_BAND2,
            regs_base: rx_base_band2 + MT7996_RXQ_BAND2 * MT_RING_SIZE,
            ndesc: MT7996_RX_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
            head: 0, tail: 0, queued: 0,
        };
        rx_buf_offset += MT7996_RX_RING_SIZE as usize * MT7996_RX_BUF_SIZE as usize;
        rx_queue_idx += 1;
        offset += ring_bytes(MT7996_RX_RING_SIZE);

        // RX WA_TRI
        let rx_desc_virt = desc_virt + offset as u64;
        self.init_rx_queue(MT7996_RXQ_MCU_WA_TRI, rx_ring_base, MT7996_RX_MCU_RING_SIZE,
            desc_phys + offset as u64, rx_desc_virt);
        rx_queues[rx_queue_idx] = RxQueueInfo {
            hw_idx: MT7996_RXQ_MCU_WA_TRI,
            regs_base: rx_ring_base + MT7996_RXQ_MCU_WA_TRI * MT_RING_SIZE,
            ndesc: MT7996_RX_MCU_RING_SIZE,
            desc_virt: rx_desc_virt,
            buf_size: MT7996_RX_BUF_SIZE,
            buf_phys: rx_buf_phys + rx_buf_offset as u64,
            buf_virt: rx_buf_virt + rx_buf_offset as u64,
            head: 0, tail: 0, queued: 0,
        };
        let _ = rx_buf_offset;
        rx_queue_idx += 1;
        let _ = offset;

        // Fill RX buffers BEFORE enabling DMA
        for i in 0..rx_queue_idx {
            self.rx_fill(&mut rx_queues[i]);
        }

        // Enable DMA AFTER RX queues are filled
        self.mt7996_dma_enable(false);

        // Return MCU_WA RX buffer info (queue index 1) for response parsing,
        // plus all RX queue infos for drain/monitoring.
        (rx_queues[1].buf_virt, rx_queues[1].buf_size, rx_queues, rx_queue_idx)
    }
}
