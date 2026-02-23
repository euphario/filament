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

#[derive(Clone, Copy, Default)]
pub struct RxQueueInfo {
    pub hw_idx: u32,
    pub regs_base: u32,
    pub ndesc: u32,
    pub desc_virt: u64,
    pub buf_size: u32,
    pub buf_phys: u64,
    pub buf_virt: u64,
}

impl RxQueueInfo {
    pub const ZERO: Self = Self {
        hw_idx: 0, regs_base: 0, ndesc: 0, desc_virt: 0,
        buf_size: 0, buf_phys: 0, buf_virt: 0,
    };
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

    pub fn rx_fill(&self, q: &RxQueueInfo) {
        let fill_count = (q.ndesc - 1) as usize;

        udebug!("dma", "rx_fill"; count = fill_count, buf_size = q.buf_size, buf_phys = q.buf_phys);

        for i in 0..fill_count {
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let buf_phys = q.buf_phys + (i as u64 * q.buf_size as u64);

            // Linux mt76_dma_add_rx_buf(): ctrl = SD_LEN0(buf_size), NO LAST_SEC0.
            // LAST_SEC0 is hardware-generated on RX to indicate packet boundaries.
            let ctrl = (q.buf_size as u32) << 16;

            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, dma_addr_lo(buf_phys));
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, dma_addr_hi(buf_phys));
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, ctrl);
            }
        }

        let desc_bytes = fill_count * core::mem::size_of::<Mt76Desc>();
        flush_buffer(q.desc_virt, desc_bytes);

        dma_wmb();

        self.mt76_wr(q.regs_base + MT_QUEUE_CPU_IDX, fill_count as u32);
    }

    /// Drain an RX ring: reset consumed descriptors and advance CPU_IDX.
    ///
    /// Returns number of entries drained. The firmware can then reuse those
    /// descriptor slots for new events. We don't process the data — just recycle.
    pub fn rx_drain(&self, q: &RxQueueInfo) -> u32 {
        let dma_idx = self.mt76_rr(q.regs_base + MT_QUEUE_DMA_IDX);
        let cpu_idx = self.mt76_rr(q.regs_base + MT_QUEUE_CPU_IDX);

        if dma_idx == cpu_idx {
            return 0; // Nothing to drain
        }

        // Walk from (cpu_idx+1) to dma_idx (wrapping), reset each descriptor
        let ndesc = q.ndesc;
        let mut count = 0u32;
        let mut idx = (cpu_idx + 1) % ndesc;

        loop {
            let i = idx as usize;
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let buf_phys = q.buf_phys + (i as u64 * q.buf_size as u64);

            // Reset descriptor: refill buffer pointer, clear DMA_DONE.
            // Linux mt76_dma_add_rx_buf(): ctrl = SD_LEN0(buf_size), NO LAST_SEC0.
            let ctrl = (q.buf_size as u32) << 16;
            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, dma_addr_lo(buf_phys));
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, dma_addr_hi(buf_phys));
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, ctrl);
            }

            count += 1;
            if idx == dma_idx { break; }
            idx = (idx + 1) % ndesc;
        }

        // Flush descriptor memory and advance CPU_IDX
        dma_wmb();
        self.mt76_wr(q.regs_base + MT_QUEUE_CPU_IDX, dma_idx);

        count
    }

    /// Soft drain: advance CPU_IDX without resetting descriptors.
    /// Use for event rings where the firmware may need buffer contents intact.
    pub fn rx_advance(&self, q: &RxQueueInfo) -> u32 {
        let dma_idx = self.mt76_rr(q.regs_base + MT_QUEUE_DMA_IDX);
        let cpu_idx = self.mt76_rr(q.regs_base + MT_QUEUE_CPU_IDX);

        if dma_idx == cpu_idx {
            return 0;
        }

        let count = if dma_idx > cpu_idx {
            dma_idx - cpu_idx
        } else {
            q.ndesc - cpu_idx + dma_idx
        };

        self.mt76_wr(q.regs_base + MT_QUEUE_CPU_IDX, dma_idx);
        count
    }

    // ========================================================================
    // rx_process_mcu() — Process MCU event queue (WM/WA RX rings)
    // Reads buffer content, dispatches to event::process_mcu_event()
    // ========================================================================

    /// Process MCU events from WM or WA RX queue.
    /// Reads each consumed descriptor's buffer and passes to event parser.
    /// Returns number of entries processed.
    pub fn rx_process_mcu(&self, q: &RxQueueInfo, counters: &mut RxMibCounters) -> u32 {
        let dma_idx = self.mt76_rr(q.regs_base + MT_QUEUE_DMA_IDX);
        let cpu_idx = self.mt76_rr(q.regs_base + MT_QUEUE_CPU_IDX);

        if dma_idx == cpu_idx {
            return 0;
        }

        let ndesc = q.ndesc;
        let mut count = 0u32;
        let mut idx = (cpu_idx + 1) % ndesc;

        loop {
            let i = idx as usize;

            // Read buffer content — DMA pool is NORMAL_NC, no cache invalidate needed
            let buf_base = q.buf_virt + (i as u64 * q.buf_size as u64);
            let buf_len = q.buf_size as usize;
            let buf = unsafe {
                core::slice::from_raw_parts(buf_base as *const u8, buf_len)
            };

            // Process MCU event
            event::process_mcu_event(buf, counters);

            // Reset descriptor for reuse
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let buf_phys = q.buf_phys + (i as u64 * q.buf_size as u64);
            let ctrl = (q.buf_size as u32) << 16;
            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, dma_addr_lo(buf_phys));
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, dma_addr_hi(buf_phys));
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, ctrl);
            }

            count += 1;
            if idx == dma_idx { break; }
            idx = (idx + 1) % ndesc;
        }

        dma_wmb();
        self.mt76_wr(q.regs_base + MT_QUEUE_CPU_IDX, dma_idx);
        count
    }

    // ========================================================================
    // rx_classify() — Process data RX queue with frame classification
    // Reads RXD0-RXD3, classifies frame type, updates MIB counters
    // ========================================================================

    /// Process data RX queue with frame classification.
    /// For each consumed descriptor, reads RXD words, classifies the frame,
    /// and updates software MIB counters. Resets descriptors for reuse.
    ///
    /// If `probe_req_macs` is provided, probe request source MACs (addr2) are
    /// collected for probe response generation. Returns (entries_processed, probe_req_count).
    pub fn rx_classify(&self, q: &RxQueueInfo, counters: &mut RxMibCounters,
                       probe_req_macs: &mut [[u8; 6]], max_probes: usize) -> (u32, usize) {
        let dma_idx = self.mt76_rr(q.regs_base + MT_QUEUE_DMA_IDX);
        let cpu_idx = self.mt76_rr(q.regs_base + MT_QUEUE_CPU_IDX);

        if dma_idx == cpu_idx {
            return (0, 0);
        }

        let ndesc = q.ndesc;
        let mut count = 0u32;
        let mut probe_count = 0usize;
        let mut idx = (cpu_idx + 1) % ndesc;

        loop {
            let i = idx as usize;
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

            // For management frames, try to read FC to distinguish beacon/probe subtypes
            let mut frame_ofs: usize = 0;
            if matches!(class, RxFrameClass::Mgmt) {
                // Navigate past RXD groups to find 802.11 header
                // Group order: G4(16), G1(16), G2(16), G3(16), G5(96)
                frame_ofs = 32; // Base RXD size
                if rxd1 & MT_RXD1_NORMAL_GROUP_4 != 0 { frame_ofs += 16; }
                if rxd1 & MT_RXD1_NORMAL_GROUP_1 != 0 { frame_ofs += 16; }
                if rxd1 & MT_RXD1_NORMAL_GROUP_2 != 0 { frame_ofs += 16; }
                if rxd1 & MT_RXD1_NORMAL_GROUP_3 != 0 { frame_ofs += 16; }
                if rxd1 & MT_RXD1_NORMAL_GROUP_5 != 0 { frame_ofs += 96; }
                let remove_pad = ((rxd2 >> 13) & 0x7) as usize;
                frame_ofs += 2 * remove_pad;

                if frame_ofs + 1 < q.buf_size as usize {
                    let fc0 = unsafe {
                        core::ptr::read_volatile((buf_base as *const u8).add(frame_ofs))
                    };
                    class = event::refine_mgmt_subtype(fc0);
                }
            }

            // Extract probe request source MAC (addr2) before resetting descriptor
            // 802.11 header: FC(2) + Duration(2) + addr1(6) + addr2(6) = addr2 at offset +10
            if matches!(class, RxFrameClass::ProbeReq) && probe_count < max_probes {
                let addr2_ofs = frame_ofs + 10; // FC(2) + Duration(2) + DA(6)
                if addr2_ofs + 6 <= q.buf_size as usize {
                    let src = unsafe {
                        core::slice::from_raw_parts((buf_base as *const u8).add(addr2_ofs), 6)
                    };
                    probe_req_macs[probe_count].copy_from_slice(src);
                    probe_count += 1;
                }
            }

            counters.record(class, result.addr_type, result.fcs_err);

            // Reset descriptor for reuse
            let desc_ptr = unsafe { (q.desc_virt as *mut Mt76Desc).add(i) };
            let buf_phys = q.buf_phys + (i as u64 * q.buf_size as u64);
            let ctrl = (q.buf_size as u32) << 16;
            unsafe {
                core::ptr::write_volatile(&mut (*desc_ptr).buf0, dma_addr_lo(buf_phys));
                core::ptr::write_volatile(&mut (*desc_ptr).buf1, dma_addr_hi(buf_phys));
                core::ptr::write_volatile(&mut (*desc_ptr).info, 0);
                core::ptr::write_volatile(&mut (*desc_ptr).ctrl, ctrl);
            }

            count += 1;
            if idx == dma_idx { break; }
            idx = (idx + 1) % ndesc;
        }

        dma_wmb();
        self.mt76_wr(q.regs_base + MT_QUEUE_CPU_IDX, dma_idx);
        (count, probe_count)
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

        // rept_wds_wcid = 0x3fff (no WDS) at offset 5
        let wcid: u16 = 0x3fff;
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
        };
        let _ = rx_buf_offset;
        rx_queue_idx += 1;
        let _ = offset;

        // Fill RX buffers BEFORE enabling DMA
        for i in 0..rx_queue_idx {
            self.rx_fill(&rx_queues[i]);
        }

        // Enable DMA AFTER RX queues are filled
        self.mt7996_dma_enable(false);

        // Return MCU_WA RX buffer info (queue index 1) for response parsing,
        // plus all RX queue infos for drain/monitoring.
        (rx_queues[1].buf_virt, rx_queues[1].buf_size, rx_queues, rx_queue_idx)
    }
}
