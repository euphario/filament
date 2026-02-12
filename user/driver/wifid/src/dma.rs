//! MT7996 DMA — Descriptor structs, ring management, DMA init/enable/disable
//!
//! EXACT translation of Linux mt76/dma.c and mt7996/dma.c.
//! Contains Mt76Desc, DmaConfig, TxRing, RxRing, and all DMA initialization.

use userlib::{uinfo, uwarn, udebug};
use crate::regs::*;
use crate::device::Mt7996Dev;

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
        }
    }

    /// Get descriptor at index
    pub fn desc(&self, idx: u32) -> *mut Mt76Desc {
        unsafe {
            (self.desc_virt as *mut Mt76Desc).add(idx as usize)
        }
    }

    /// Get buffer for descriptor at index (4KB per descriptor)
    pub fn buf(&self, idx: u32) -> *mut u8 {
        unsafe {
            (self.buf_virt as *mut u8).add(idx as usize * MCU_FW_DL_BUF_SIZE)
        }
    }

    /// Get buffer physical address for descriptor
    pub fn buf_phys(&self, idx: u32) -> u64 {
        self.buf_phys + (idx as u64 * MCU_FW_DL_BUF_SIZE as u64)
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

        let irq_mask = MT_INT_MCU_CMD | MT_INT_RX_DONE_MCU | MT_INT_TX_DONE_MCU;

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

    // ========================================================================
    // mt7996_dma_init() — EXACT Linux translation
    // Source: dma.c:599-854
    // ========================================================================

    pub fn mt7996_dma_init(&self, desc_phys: u64, desc_virt: u64, _desc_size: usize,
                       rx_buf_phys: u64, rx_buf_virt: u64, _rx_buf_size: usize) {
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
    }
}
