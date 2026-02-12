//! MCU Command Protocol — McuTxd, UniTxd, command encoding, send/wait
//!
//! EXACT translation of Linux mt76_connac_mcu.h and mt7996/mcu.c.
//! Contains both the legacy MCU TXD (for firmware download and WA commands)
//! and the UNI TXD (for post-firmware init commands).

use userlib::{uinfo, uerror, udebug};
use crate::regs::*;
use crate::device::Mt7996Dev;
use crate::dma::{TxRing, Mt76Desc, dma_wmb, flush_buffer};

// ============================================================================
// MCU packet/command constants — from mt76_connac_mcu.h
// ============================================================================

/// MCU packet type
pub const MCU_PKT_ID: u8 = 0xa0;

/// MCU command IDs — from mt76_connac_mcu.h enum
pub mod mcu_cmd {
    pub const TARGET_ADDRESS_LEN_REQ: u8 = 0x01;
    pub const FW_START_REQ: u8 = 0x02;
    pub const PATCH_START_REQ: u8 = 0x05;
    pub const PATCH_FINISH_REQ: u8 = 0x07;
    pub const PATCH_SEM_CTRL: u8 = 0x10;
    pub const WA_PARAM: u8 = 0xc4;
    pub const EXT_CID: u8 = 0xed;
    pub const FW_SCATTER: u8 = 0xee;
}

/// MCU download modes — from mt76_connac_mcu.h
pub mod dl_mode {
    pub const ENCRYPT: u32 = 1 << 0;
    pub const KEY_IDX_MASK: u32 = 0x3 << 1;
    pub const RESET_SEC_IV: u32 = 1 << 3;
    pub const WORKING_PDA_CR4: u32 = 1 << 4;
    pub const VALID_RAM_ENTRY: u32 = 1 << 5;
    pub const NEED_RSP: u32 = 1 << 31;
}

/// Feature set flags — from mt76_connac_mcu.h
pub mod fw_feature {
    pub const SET_ENCRYPT: u8 = 1 << 0;
    pub const SET_KEY_IDX_MASK: u8 = 0x6;
    pub const OVERRIDE_ADDR: u8 = 1 << 5;
}

/// FW_START option flags — from mt76_connac_mcu.h
pub mod fw_start {
    pub const OVERRIDE: u32 = 1 << 0;
    pub const WORKING_PDA_CR4: u32 = 1 << 2;
    pub const WORKING_PDA_DSP: u32 = 1 << 3;
}

/// MCU query type — from mt76_connac_mcu.h
#[allow(dead_code)]
pub mod mcu_q {
    pub const QUERY: u8 = 0;
    pub const SET: u8 = 1;
    pub const RESERVED: u8 = 2;
    pub const NA: u8 = 3;
}

/// Source-to-destination index — from mt76_connac_mcu.h
pub const S2D_H2N: u8 = 0;    // Host to WM (N9)
pub const S2D_C2N: u8 = 1;    // WA to WM
pub const S2D_H2C: u8 = 2;    // Host to WA
pub const S2D_H2CN: u8 = 3;   // Host to both WM and WA

/// MCU option bits — from mt76_connac_mcu.h
pub const MCU_CMD_ACK: u8 = 1 << 0;
pub const MCU_CMD_UNI: u8 = 1 << 1;
pub const MCU_CMD_SET: u8 = 1 << 2;
pub const MCU_CMD_UNI_EXT_ACK: u8 = MCU_CMD_ACK | MCU_CMD_UNI | MCU_CMD_SET;
pub const MCU_CMD_UNI_QUERY_ACK: u8 = MCU_CMD_ACK | MCU_CMD_UNI;

/// __MCU_CMD_FIELD bits for encoding cmd parameter
const CMD_FIELD_QUERY: u32 = 1 << 16;
const CMD_FIELD_UNI: u32 = 1 << 17;
const CMD_FIELD_WA: u32 = 1 << 19;
const CMD_FIELD_WM: u32 = 1 << 20;

/// UNI command IDs — from mt76_connac_mcu.h enum
pub const MCU_UNI_CMD_WSYS_CONFIG: u16 = 0x0b;
pub const MCU_UNI_CMD_EFUSE_CTRL: u16 = 0x2d;
pub const MCU_UNI_CMD_VOW: u16 = 0x37;
pub const MCU_UNI_CMD_FIXED_RATE_TABLE: u16 = 0x40;
pub const MCU_UNI_CMD_RRO: u16 = 0x57;
pub const MCU_UNI_CMD_SDO: u16 = 0x88;

/// UNI TLV tags — from mt7996/mcu.h
const UNI_WSYS_CONFIG_FW_LOG_CTRL: u16 = 0;
const UNI_VOW_RX_AT_AIRTIME_EN: u16 = 0x0b;
const UNI_VOW_RX_AT_AIRTIME_CLR_EN: u16 = 0x0e;

/// EFUSE TLV tags — from mt7996/mcu.h
const UNI_EFUSE_BUFFER_MODE: u16 = 2;

/// EEPROM mode/format — from mt76_connac_mcu.h
const EE_MODE_EFUSE: u8 = 0;
const EE_FORMAT_WHOLE: u8 = 1;

/// RRO TLV tags — from mt7996/mcu.h enum (starts at DEL_ENTRY=1)
pub const UNI_RRO_SET_PLATFORM_TYPE: u16 = 0x2;
pub const UNI_RRO_SET_BYPASS_MODE: u16 = 0x4;
pub const UNI_RRO_SET_TXFREE_PATH: u16 = 0x5;
pub const UNI_RRO_SET_FLUSH_TIMEOUT: u16 = 0x7;

/// MCU_EXT_CMD — from mt76_connac_mcu.h
const MCU_EXT_CMD_MWDS_SUPPORT: u8 = 0x80;

/// MCU_WA_PARAM — from mt7996/mcu.h
pub const MCU_WA_PARAM_RED: u32 = 0x0e;
pub const MCU_WA_PARAM_HW_PATH_HIF_VER: u32 = 0x2f;

/// Compute download mode from region's feature_set.
/// Exact translation of Linux mt76_connac_mcu_gen_dl_mode().
pub fn gen_dl_mode(feature_set: u8, is_wa: bool) -> u32 {
    let mut mode: u32 = 0;
    if feature_set & fw_feature::SET_ENCRYPT != 0 {
        mode |= dl_mode::ENCRYPT | dl_mode::RESET_SEC_IV;
    }
    mode |= (feature_set & fw_feature::SET_KEY_IDX_MASK) as u32;
    if is_wa {
        mode |= dl_mode::WORKING_PDA_CR4;
    }
    mode
}

// ============================================================================
// Legacy MCU TXD — for firmware download and WA commands
// Source: mt76_connac_mcu.h struct mt76_connac2_mcu_txd
// ============================================================================

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct McuTxd {
    pub txd: [u32; 8],
    pub len: u16,
    pub pq_id: u16,
    pub cid: u8,
    pub pkt_type: u8,
    pub set_query: u8,
    pub seq: u8,
    pub uc_d2b0_rev: u8,
    pub ext_cid: u8,
    pub s2d_index: u8,
    pub ext_cid_ack: u8,
    pub rsv: [u32; 5],
}

impl McuTxd {
    pub const SIZE: usize = 64;

    pub fn new(cmd: u8, len: u16, seq: u8) -> Self {
        let mut txd = Self::default();
        let total_len = (Self::SIZE + len as usize) as u32;
        const MT_TX_TYPE_CMD: u32 = 2;
        const MT_TX_MCU_PORT_RX_Q0: u32 = 0x20;
        txd.txd[0] = (total_len & 0xffff) | (MT_TX_TYPE_CMD << 23) | (MT_TX_MCU_PORT_RX_Q0 << 25);
        const MT_HDR_FORMAT_CMD: u32 = 1;
        txd.txd[1] = MT_HDR_FORMAT_CMD << 14;
        txd.len = ((Self::SIZE + len as usize - 32) as u16).to_le();
        txd.pq_id = 0x8000u16.to_le();
        txd.cid = cmd;
        txd.pkt_type = MCU_PKT_ID;
        txd.set_query = mcu_q::NA;
        txd.seq = seq;
        txd.s2d_index = S2D_H2N;
        txd
    }

    /// Build legacy MCU TXD with ext_cid (for WA EXT commands)
    pub fn new_ext(cid: u8, ext_cid: u8, len: u16, seq: u8, s2d: u8) -> Self {
        let mut txd = Self::new(cid, len, seq);
        txd.ext_cid = ext_cid;
        txd.s2d_index = s2d;
        txd.set_query = mcu_q::SET;
        txd.ext_cid_ack = 1; // ext_cid requires ack
        txd
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, Self::SIZE)
        }
    }
}

// ============================================================================
// UNI MCU TXD — for post-firmware UNI commands
// Source: mt76_connac_mcu.h struct mt76_connac2_mcu_uni_txd (48 bytes)
// ============================================================================

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
pub struct UniTxd {
    pub txd: [u32; 8],      // 0-31: Hardware TXD
    pub len: u16,            // 32-33: Payload length (after TXD)
    pub cid: u16,            // 34-35: Command ID (le16)
    pub rsv: u8,             // 36
    pub pkt_type: u8,        // 37: 0xa0
    pub frag_n: u8,          // 38
    pub seq: u8,             // 39
    pub checksum: u16,       // 40-41: 0
    pub s2d_index: u8,       // 42: routing
    pub option: u8,          // 43: ACK|UNI|SET bits
    pub rsv1: [u8; 4],       // 44-47
}

impl UniTxd {
    pub const SIZE: usize = 48;

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, Self::SIZE)
        }
    }
}

// ============================================================================
// Init download / patch semaphore request structs
// ============================================================================

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct InitDlRequest {
    pub addr: u32,
    pub len: u32,
    pub mode: u32,
}

impl InitDlRequest {
    pub fn new(addr: u32, len: u32, mode: u32) -> Self {
        Self {
            addr: addr.to_le(),
            len: len.to_le(),
            mode: mode.to_le(),
        }
    }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, core::mem::size_of::<Self>())
        }
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct PatchSemCtrl {
    pub op: u32,
}

impl PatchSemCtrl {
    pub fn get() -> Self { Self { op: 1u32.to_le() } }
    pub fn release() -> Self { Self { op: 0u32.to_le() } }

    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(self as *const _ as *const u8, 4)
        }
    }
}

// ============================================================================
// MCU Command Functions on Mt7996Dev
// ============================================================================

impl Mt7996Dev {
    /// Send MCU command via a TX ring (legacy McuTxd format)
    pub fn mcu_send_cmd(&self, ring: &mut TxRing, cmd: u8, data: &[u8], seq: u8) -> Result<(), i32> {
        let idx = ring.cpu_idx;
        let txd = McuTxd::new(cmd, data.len() as u16, seq);

        let buf = ring.buf(idx);
        let total_len = McuTxd::SIZE + data.len();
        unsafe {
            core::ptr::write_bytes(buf, 0, total_len);
            core::ptr::copy_nonoverlapping(txd.as_bytes().as_ptr(), buf, McuTxd::SIZE);
            if !data.is_empty() {
                core::ptr::copy_nonoverlapping(data.as_ptr(), buf.add(McuTxd::SIZE), data.len());
            }
        }

        let buf_phys = ring.buf_phys(idx);
        let desc = ring.desc(idx);
        let ctrl_val = ((total_len as u32) << 16) | MT_DMA_CTL_LAST_SEC0;
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(buf_phys));
            core::ptr::write_volatile(&mut (*desc).buf1, 0);
            core::ptr::write_volatile(&mut (*desc).info, dma_addr_hi(buf_phys));
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        flush_buffer(buf as u64, total_len);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        ring.cpu_idx = (ring.cpu_idx + 1) % ring.ndesc;
        dma_wmb();
        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        // Clear and restore interrupts on both HIFs
        let int_mask1_old = self.mt76_rr(MT_INT_MASK_CSR);
        let int_mask2_old = self.mt76_rr(MT_INT1_MASK_CSR);
        self.mt76_wr(MT_INT_MASK_CSR, 0);
        self.mt76_wr(MT_INT1_MASK_CSR, 0);
        self.mt76_wr(MT_INT_MASK_CSR, 0);
        self.mt76_wr(MT_INT1_MASK_CSR, 0);

        let int_src1 = self.mt76_rr(MT_INT_SOURCE_CSR);
        self.mt76_wr(MT_INT_SOURCE_CSR, int_src1);
        let int_src2 = self.mt76_rr(MT_INT1_SOURCE_CSR);
        self.mt76_wr(MT_INT1_SOURCE_CSR, int_src2);

        self.mt76_wr(MT_INT_MASK_CSR, int_mask1_old);
        self.mt76_wr(MT_INT1_MASK_CSR, int_mask2_old);

        Ok(())
    }

    /// Send MCU command with legacy McuTxd + ext_cid (for WA EXT commands like MWDS)
    fn mcu_send_cmd_ext(&self, ring: &mut TxRing, cid: u8, ext_cid: u8, s2d: u8, data: &[u8], seq: u8) -> Result<(), i32> {
        let idx = ring.cpu_idx;
        let txd = McuTxd::new_ext(cid, ext_cid, data.len() as u16, seq, s2d);

        let buf = ring.buf(idx);
        let total_len = McuTxd::SIZE + data.len();
        unsafe {
            core::ptr::write_bytes(buf, 0, total_len);
            core::ptr::copy_nonoverlapping(txd.as_bytes().as_ptr(), buf, McuTxd::SIZE);
            if !data.is_empty() {
                core::ptr::copy_nonoverlapping(data.as_ptr(), buf.add(McuTxd::SIZE), data.len());
            }
        }

        let buf_phys = ring.buf_phys(idx);
        let desc = ring.desc(idx);
        let ctrl_val = ((total_len as u32) << 16) | MT_DMA_CTL_LAST_SEC0;
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(buf_phys));
            core::ptr::write_volatile(&mut (*desc).buf1, 0);
            core::ptr::write_volatile(&mut (*desc).info, dma_addr_hi(buf_phys));
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        flush_buffer(buf as u64, total_len);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        ring.cpu_idx = (ring.cpu_idx + 1) % ring.ndesc;
        dma_wmb();
        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        let int_mask1_old = self.mt76_rr(MT_INT_MASK_CSR);
        let int_mask2_old = self.mt76_rr(MT_INT1_MASK_CSR);
        self.mt76_wr(MT_INT_MASK_CSR, 0);
        self.mt76_wr(MT_INT1_MASK_CSR, 0);
        self.mt76_wr(MT_INT_MASK_CSR, 0);
        self.mt76_wr(MT_INT1_MASK_CSR, 0);
        let int_src1 = self.mt76_rr(MT_INT_SOURCE_CSR);
        self.mt76_wr(MT_INT_SOURCE_CSR, int_src1);
        let int_src2 = self.mt76_rr(MT_INT1_SOURCE_CSR);
        self.mt76_wr(MT_INT1_SOURCE_CSR, int_src2);
        self.mt76_wr(MT_INT_MASK_CSR, int_mask1_old);
        self.mt76_wr(MT_INT1_MASK_CSR, int_mask2_old);

        Ok(())
    }

    /// Send UNI MCU command via MCU_WM ring
    ///
    /// Builds UniTxd header, copies TLV payload after it, sends on the ring.
    /// cmd encodes destination bits per Linux __MCU_CMD_FIELD macros:
    ///   - Bits [7:0] = command ID (for cid)
    ///   - BIT(16) = QUERY (affects option field)
    ///   - BIT(19) = WA destination
    ///   - BIT(20) = WM destination
    ///
    /// Reference: mt7996/mcu.c:275-331 mt7996_mcu_send_message()
    pub fn mcu_send_uni_cmd(&self, ring: &mut TxRing, cmd: u32, data: &[u8], wait: bool, seq: u8) -> Result<(), i32> {
        let idx = ring.cpu_idx;
        let mcu_cmd = (cmd & 0xFF) as u16;

        let mut uni_txd = UniTxd::default();

        let total_len = UniTxd::SIZE + data.len();

        // TXD[0]: tx_bytes | pkt_fmt=CMD(2) | q_idx=0x20
        const MT_TX_TYPE_CMD: u32 = 2;
        const MT_TX_MCU_PORT_RX_Q0: u32 = 0x20;
        uni_txd.txd[0] = (total_len as u32 & 0xffff) | (MT_TX_TYPE_CMD << 23) | (MT_TX_MCU_PORT_RX_Q0 << 25);

        // TXD[1]: hdr_format=CMD(1) — NO LONG_FORMAT (Linux mcu.c:305-306)
        const MT_HDR_FORMAT_CMD: u32 = 1;
        uni_txd.txd[1] = MT_HDR_FORMAT_CMD << 14;

        // UNI header fields
        uni_txd.len = ((total_len - 32) as u16).to_le();  // len = total - sizeof(txd[8])
        uni_txd.cid = mcu_cmd.to_le();
        uni_txd.pkt_type = MCU_PKT_ID;
        uni_txd.seq = seq;

        // s2d_index routing — mt7996/mcu.c:324-329
        // Default is H2CN for MT7996 (which has WA)
        uni_txd.s2d_index = S2D_H2CN;
        if (cmd & CMD_FIELD_WA) != 0 && (cmd & CMD_FIELD_WM) != 0 {
            uni_txd.s2d_index = S2D_H2CN;
        } else if (cmd & CMD_FIELD_WA) != 0 {
            uni_txd.s2d_index = S2D_H2C;
        } else if (cmd & CMD_FIELD_WM) != 0 {
            uni_txd.s2d_index = S2D_H2N;
        }

        // option — mt7996/mcu.c:316-319
        if (cmd & CMD_FIELD_QUERY) != 0 {
            uni_txd.option = MCU_CMD_UNI_QUERY_ACK;
        } else {
            uni_txd.option = MCU_CMD_UNI_EXT_ACK;
        }

        // SDO special case — mt7996/mcu.c:321-322
        if mcu_cmd == MCU_UNI_CMD_SDO {
            uni_txd.option &= !MCU_CMD_ACK;
        }

        // Snapshot RX DMA_IDX BEFORE sending — must be before CPU_IDX write
        // After firmware loaded, responses arrive on WA RX queue (q1), not WM (q0).
        // Linux: mt7996_mcu_send_message() lines 295-298
        let rx_snap = if wait { self.snapshot_rx_idx(MCU_WA_RX_REGS) } else { 0 };

        // Copy UniTxd + data to buffer
        let buf = ring.buf(idx);
        unsafe {
            core::ptr::write_bytes(buf, 0, total_len);
            core::ptr::copy_nonoverlapping(uni_txd.as_bytes().as_ptr(), buf, UniTxd::SIZE);
            if !data.is_empty() {
                core::ptr::copy_nonoverlapping(data.as_ptr(), buf.add(UniTxd::SIZE), data.len());
            }
        }

        let buf_phys = ring.buf_phys(idx);
        let desc = ring.desc(idx);
        let ctrl_val = ((total_len as u32) << 16) | MT_DMA_CTL_LAST_SEC0;
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(buf_phys));
            core::ptr::write_volatile(&mut (*desc).buf1, 0);
            core::ptr::write_volatile(&mut (*desc).info, dma_addr_hi(buf_phys));
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        flush_buffer(buf as u64, total_len);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        ring.cpu_idx = (ring.cpu_idx + 1) % ring.ndesc;
        dma_wmb();
        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        // Interrupt management
        let int_mask1_old = self.mt76_rr(MT_INT_MASK_CSR);
        let int_mask2_old = self.mt76_rr(MT_INT1_MASK_CSR);
        self.mt76_wr(MT_INT_MASK_CSR, 0);
        self.mt76_wr(MT_INT1_MASK_CSR, 0);
        self.mt76_wr(MT_INT_MASK_CSR, 0);
        self.mt76_wr(MT_INT1_MASK_CSR, 0);
        let int_src1 = self.mt76_rr(MT_INT_SOURCE_CSR);
        self.mt76_wr(MT_INT_SOURCE_CSR, int_src1);
        let int_src2 = self.mt76_rr(MT_INT1_SOURCE_CSR);
        self.mt76_wr(MT_INT1_SOURCE_CSR, int_src2);
        self.mt76_wr(MT_INT_MASK_CSR, int_mask1_old);
        self.mt76_wr(MT_INT1_MASK_CSR, int_mask2_old);

        // Wait for TX completion
        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 2000) {
            uerror!("mcu", "uni_cmd_tx_timeout"; cid = mcu_cmd);
            return Err(-1);
        }

        // Wait for MCU response if requested (on WA RX queue)
        if wait {
            self.wait_rx_response(MCU_WA_RX_REGS, rx_snap, 5000)?;
        }

        Ok(())
    }

    /// Wait for TX descriptor to complete
    pub fn mcu_wait_tx_done(&self, ring: &TxRing, idx: u32, timeout_ms: u32) -> bool {
        let desc = ring.desc(idx);
        for _ in 0..timeout_ms {
            let ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
            if (ctrl & MT_DMA_CTL_DMA_DONE) != 0 {
                return true;
            }
            userlib::delay_ms(1);
        }
        false
    }

    /// Snapshot MCU RX DMA_IDX — call BEFORE sending a command.
    /// rx_regs: MCU_WM_RX_REGS (firmware loading) or MCU_WA_RX_REGS (post-firmware)
    pub fn snapshot_rx_idx(&self, rx_regs: u32) -> u32 {
        self.mt76_rr(rx_regs + MT_QUEUE_DMA_IDX)
    }

    /// Legacy: snapshot on MCU_WM RX queue (used by firmware loading)
    pub fn snapshot_mcu_rx_idx(&self) -> u32 {
        self.snapshot_rx_idx(MCU_WM_RX_REGS)
    }

    /// Wait for MCU response — polls rx_regs DMA_IDX until it advances past pre_send_dma_idx.
    pub fn wait_rx_response(&self, rx_regs: u32, pre_send_dma_idx: u32, timeout_ms: u32) -> Result<(), i32> {
        for _ in 0..timeout_ms {
            let dma_idx = self.mt76_rr(rx_regs + MT_QUEUE_DMA_IDX);
            if dma_idx != pre_send_dma_idx {
                udebug!("mcu", "rx_ok"; snap = pre_send_dma_idx, now = dma_idx);
                return Ok(());
            }
            userlib::delay_ms(1);
        }

        let rx_base = MT_WFDMA0_BASE + 0x500;
        let q0 = self.mt76_rr(rx_base + 0 * MT_RING_SIZE + MT_QUEUE_DMA_IDX);
        let q1 = self.mt76_rr(rx_base + 1 * MT_RING_SIZE + MT_QUEUE_DMA_IDX);
        let q2 = self.mt76_rr(rx_base + 2 * MT_RING_SIZE + MT_QUEUE_DMA_IDX);
        let q3 = self.mt76_rr(rx_base + 3 * MT_RING_SIZE + MT_QUEUE_DMA_IDX);
        uerror!("mcu", "rx_response_timeout"; snap = pre_send_dma_idx, q0 = q0, q1 = q1, q2 = q2, q3 = q3);
        Err(-1)
    }

    /// Legacy: wait on MCU_WM RX queue (used by firmware loading)
    pub fn wait_mcu_rx_response(&self, pre_send_dma_idx: u32, timeout_ms: u32) -> Result<(), i32> {
        self.wait_rx_response(MCU_WM_RX_REGS, pre_send_dma_idx, timeout_ms)
    }

    /// Send init download command
    pub fn mcu_init_download(&self, ring: &mut TxRing, addr: u32, len: u32, mode: u32, seq: u8) -> Result<(), i32> {
        let cmd = if addr == 0x900000 {
            udebug!("mcu", "init_download_cmd"; addr = addr, cmd = "PATCH_START_REQ");
            mcu_cmd::PATCH_START_REQ
        } else {
            udebug!("mcu", "init_download_cmd"; addr = addr, cmd = "TARGET_ADDRESS_LEN_REQ");
            mcu_cmd::TARGET_ADDRESS_LEN_REQ
        };

        let req = InitDlRequest::new(addr, len, mode | dl_mode::NEED_RSP);
        let rx_snap = self.snapshot_mcu_rx_idx();
        self.mcu_send_cmd(ring, cmd, req.as_bytes(), seq)?;

        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 1000) {
            return Err(-1);
        }

        if let Err(_) = self.wait_mcu_rx_response(rx_snap, 2000) {
            return Err(-1);
        }

        Ok(())
    }

    /// Send firmware chunk (scatter command — raw data, NO TXD header)
    pub fn mcu_send_firmware_chunk(&self, ring: &mut TxRing, data: &[u8], _seq: u8) -> Result<(), i32> {
        let idx = ring.cpu_idx;
        let buf = ring.buf(idx);

        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), buf, data.len());
        }

        let total_len = data.len();
        let buf_phys = ring.buf_phys(idx);
        let desc = ring.desc(idx);
        let ctrl_val = ((total_len as u32) << 16) | MT_DMA_CTL_LAST_SEC0;
        unsafe {
            core::ptr::write_volatile(&mut (*desc).buf0, dma_addr_lo(buf_phys));
            core::ptr::write_volatile(&mut (*desc).buf1, 0);
            core::ptr::write_volatile(&mut (*desc).info, dma_addr_hi(buf_phys));
            core::ptr::write_volatile(&mut (*desc).ctrl, ctrl_val);
        }

        flush_buffer(buf as u64, total_len);
        flush_buffer(desc as u64, core::mem::size_of::<Mt76Desc>());

        ring.cpu_idx = (ring.cpu_idx + 1) % ring.ndesc;
        dma_wmb();
        self.mt76_wr(ring.regs_base + MT_QUEUE_CPU_IDX, ring.cpu_idx);

        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 100) {
            uerror!("wifid", "fw_chunk_timeout";);
            return Err(-1);
        }

        Ok(())
    }

    /// Send patch/firmware start command
    pub fn mcu_start_firmware(&self, ring: &mut TxRing, is_patch: bool, option: u32, addr: u32, seq: u8) -> Result<(), i32> {
        let cmd = if is_patch {
            mcu_cmd::PATCH_FINISH_REQ
        } else {
            mcu_cmd::FW_START_REQ
        };

        let rx_snap = self.snapshot_mcu_rx_idx();
        if is_patch {
            let req: [u8; 4] = [0, 0, 0, 0];
            self.mcu_send_cmd(ring, cmd, &req, seq)?;
        } else {
            let mut req = [0u8; 8];
            req[0..4].copy_from_slice(&option.to_le_bytes());
            req[4..8].copy_from_slice(&addr.to_le_bytes());
            self.mcu_send_cmd(ring, cmd, &req, seq)?;
        }

        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 1000) {
            uerror!("wifid", "mcu_start_fw_timeout";);
            return Err(-1);
        }

        if let Err(_) = self.wait_mcu_rx_response(rx_snap, 5000) {
            return Err(-1);
        }
        Ok(())
    }

    /// Get/release patch semaphore
    pub fn mcu_patch_sem_ctrl(&self, ring: &mut TxRing, get: bool, seq: u8) -> Result<(), i32> {
        let req = if get { PatchSemCtrl::get() } else { PatchSemCtrl::release() };
        let rx_snap = self.snapshot_mcu_rx_idx();
        self.mcu_send_cmd(ring, mcu_cmd::PATCH_SEM_CTRL, req.as_bytes(), seq)?;

        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 1000) {
            uerror!("wifid", "patch_sem_timeout";);
            return Err(-1);
        }

        self.wait_mcu_rx_response(rx_snap, 2000)?;
        Ok(())
    }

    // ========================================================================
    // Post-firmware MCU init commands
    // ========================================================================

    /// Enable firmware logging
    /// Linux: mt7996/mcu.c:3213 mt7996_mcu_fw_log_2_host()
    ///
    /// type: MCU_FW_LOG_WM(0) or MCU_FW_LOG_WA(1)
    /// ctrl: 0 to enable default logging
    pub fn mcu_fw_log_2_host(&self, ring: &mut TxRing, fw_type: u8, ctrl: u8, seq: u8) -> Result<(), i32> {
        // struct: { u8 _rsv[4]; le16 tag; le16 len; u8 ctrl; u8 interval; u8 _rsv2[2]; }
        // total 12 bytes — 4 bytes uni_header + 8 bytes TLV
        let mut data = [0u8; 12];
        // _rsv[4] = 0 (uni_header)
        // tag = UNI_WSYS_CONFIG_FW_LOG_CTRL (0)
        data[4..6].copy_from_slice(&(UNI_WSYS_CONFIG_FW_LOG_CTRL as u16).to_le_bytes());
        // len = sizeof(data) - 4 = 8
        data[6..8].copy_from_slice(&8u16.to_le_bytes());
        // ctrl
        data[8] = ctrl;
        // interval = 0, _rsv2 = 0

        // Route to WM or WA
        // Linux: MCU_WA_UNI_CMD(WSYS_CONFIG) = __MCU_CMD_FIELD_UNI | 0x0b | __MCU_CMD_FIELD_WA
        //        MCU_WM_UNI_CMD(WSYS_CONFIG) = __MCU_CMD_FIELD_UNI | 0x0b | __MCU_CMD_FIELD_WM
        let cmd = if fw_type == 1 {
            // MCU_FW_LOG_WA → MCU_WA_UNI_CMD(WSYS_CONFIG)
            CMD_FIELD_UNI | (MCU_UNI_CMD_WSYS_CONFIG as u32) | CMD_FIELD_WA
        } else {
            // MCU_FW_LOG_WM → MCU_WM_UNI_CMD(WSYS_CONFIG)
            CMD_FIELD_UNI | (MCU_UNI_CMD_WSYS_CONFIG as u32) | CMD_FIELD_WM
        };

        udebug!("mcu", "fw_log_2_host"; fw_type = fw_type, ctrl = ctrl);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq)
    }

    /// Enable MWDS (multi-wire download steering)
    /// Linux: mt7996/mcu.c:3258 mt7996_mcu_set_mwds()
    ///
    /// Uses legacy MCU command MCU_WA_EXT_CMD(MWDS_SUPPORT) — NOT UNI.
    /// Sent to WA (s2d=H2C), no wait.
    pub fn mcu_set_mwds(&self, ring: &mut TxRing, enabled: bool, seq: u8) -> Result<(), i32> {
        let mut req = [0u8; 4];
        req[0] = if enabled { 1 } else { 0 };

        // MCU_WA_EXT_CMD(MWDS_SUPPORT):
        //   cid = MCU_CMD_EXT_CID (0xed)
        //   ext_cid = MCU_EXT_CMD_MWDS_SUPPORT (0x80)
        //   s2d = H2C (sent to WA)
        udebug!("mcu", "set_mwds"; enabled = enabled as u8);

        let rx_snap = self.snapshot_mcu_rx_idx();
        self.mcu_send_cmd_ext(ring, mcu_cmd::EXT_CID, MCU_EXT_CMD_MWDS_SUPPORT, S2D_H2C, &req, seq)?;

        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 2000) {
            uerror!("mcu", "mwds_tx_timeout";);
            return Err(-1);
        }

        // Linux passes wait=false for MWDS, but we briefly wait to avoid overlap
        userlib::delay_ms(10);
        Ok(())
    }

    /// Initialize RX airtime for all bands
    /// Linux: mt7996/mcu.c:3288 mt7996_mcu_init_rx_airtime()
    ///
    /// Sends a UNI VOW command with 2 TLVs per band (CLR_EN + EN).
    /// MT7996 has 3 bands (0, 1, 2).
    pub fn mcu_init_rx_airtime(&self, ring: &mut TxRing, seq: u8) -> Result<(), i32> {
        // 3 bands × 2 TLVs = 6 TLVs
        // Each TLV: struct vow_rx_airtime { le16 tag; le16 len; u8 enable; u8 band; u8 _rsv[2]; } = 8 bytes
        // uni_header(4) + 6 × 8 = 52 bytes
        let mut data = [0u8; 52];

        // uni_header = 0 (4 bytes)
        let mut offset = 4usize;

        // For each band 0, 1, 2
        for band in 0u8..3 {
            // TLV 1: UNI_VOW_RX_AT_AIRTIME_CLR_EN
            data[offset..offset + 2].copy_from_slice(&UNI_VOW_RX_AT_AIRTIME_CLR_EN.to_le_bytes());
            data[offset + 2..offset + 4].copy_from_slice(&8u16.to_le_bytes()); // len = 8
            data[offset + 4] = 1; // enable = true
            data[offset + 5] = band;
            offset += 8;

            // TLV 2: UNI_VOW_RX_AT_AIRTIME_EN
            data[offset..offset + 2].copy_from_slice(&UNI_VOW_RX_AT_AIRTIME_EN.to_le_bytes());
            data[offset + 2..offset + 4].copy_from_slice(&8u16.to_le_bytes()); // len = 8
            data[offset + 4] = 1; // enable = true
            data[offset + 5] = band;
            offset += 8;
        }

        // MCU_WM_UNI_CMD(VOW) = __MCU_CMD_FIELD_UNI | 0x37 | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_VOW as u32) | CMD_FIELD_WM;

        udebug!("mcu", "init_rx_airtime");
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq)
    }

    /// Send WA parameter command
    /// Linux: mt7996/mcu.c:365 mt7996_mcu_wa_cmd()
    ///
    /// When mt7996_has_wa (always true for MT7996), sends req.args (12 bytes)
    /// as a legacy MCU command via MCU_WA_CMD(WA_PARAM) = MCU_CMD_WA_PARAM | __MCU_CMD_FIELD_WA.
    ///
    /// The cmd encoding: cid = MCU_CMD_WA_PARAM (0xc4), ext_cid from __MCU_CMD_FIELD_EXT_ID,
    /// sent to WA (s2d = H2C).
    pub fn mcu_wa_cmd(&self, ring: &mut TxRing, a1: u32, a2: u32, a3: u32, seq: u8) -> Result<(), i32> {
        // MT7996 has WA → send args directly as legacy command
        // MCU_WA_PARAM_CMD(SET) = MCU_CMD(WA_PARAM) | __MCU_CMD_FIELD_WA |
        //                         FIELD_PREP(__MCU_CMD_FIELD_EXT_ID, MCU_WA_PARAM_CMD_SET)
        // MCU_WA_PARAM_CMD_SET = 1, so ext_id = 1
        let mut args = [0u8; 12];
        args[0..4].copy_from_slice(&a1.to_le_bytes());
        args[4..8].copy_from_slice(&a2.to_le_bytes());
        args[8..12].copy_from_slice(&a3.to_le_bytes());

        // cid = WA_PARAM (0xc4), ext_cid = MCU_WA_PARAM_CMD_SET (1), s2d = H2C (WA)
        udebug!("mcu", "wa_cmd"; a1 = a1, a2 = a2, a3 = a3);

        let rx_snap = self.snapshot_mcu_rx_idx();
        self.mcu_send_cmd_ext(ring, mcu_cmd::WA_PARAM, 1, S2D_H2C, &args, seq)?;

        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 2000) {
            uerror!("mcu", "wa_cmd_tx_timeout";);
            return Err(-1);
        }

        // Linux passes wait=false for this command
        userlib::delay_ms(10);
        Ok(())
    }

    // ========================================================================
    // MAC Init MCU Commands
    // ========================================================================

    /// Set RRO (Reorder) module parameters
    /// Linux: mt7996/mcu.c:4688-4739 mt7996_mcu_set_rro()
    ///
    /// UNI command MCU_WM_UNI_CMD(RRO) (0x57).
    /// Tags: PLATFORM_TYPE=2, BYPASS_MODE=4, TXFREE_PATH=5, FLUSH_TIMEOUT=7
    pub fn mcu_set_rro(&self, ring: &mut TxRing, tag: u16, val: u16, seq: u8) -> Result<(), i32> {
        // Struct layout from Linux mcu.c:4688-4720:
        //   u8 __rsv1[4];        // uni_header
        //   le16 tag;
        //   le16 len;
        //   union { ... } (max 8 bytes)
        // Total: 4 + 4 + 8 = 16 bytes
        let mut data = [0u8; 16];

        // uni_header = 0 (4 bytes)
        // tag
        data[4..6].copy_from_slice(&tag.to_le_bytes());
        // len = sizeof(req) - 4 = 12
        data[6..8].copy_from_slice(&12u16.to_le_bytes());

        // Union contents depend on tag — mcu.c:4722-4737
        match tag {
            UNI_RRO_SET_PLATFORM_TYPE => {
                // platform_type.type = val
                data[8] = val as u8;
            }
            UNI_RRO_SET_BYPASS_MODE => {
                // bypass_mode.type = val
                data[8] = val as u8;
            }
            UNI_RRO_SET_TXFREE_PATH => {
                // txfree_path.path = val
                data[8] = val as u8;
            }
            UNI_RRO_SET_FLUSH_TIMEOUT => {
                // timeout.flush_one = val, timeout.flush_all = 2*val
                data[8..10].copy_from_slice(&val.to_le_bytes());
                data[10..12].copy_from_slice(&(2 * val).to_le_bytes());
            }
            _ => return Err(-1),
        }

        // MCU_WM_UNI_CMD(RRO) = __MCU_CMD_FIELD_UNI | 0x57 | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_RRO as u32) | CMD_FIELD_WM;

        udebug!("mcu", "set_rro"; tag = tag, val = val);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq)
    }

    /// Tell MCU about EEPROM mode (efuse, not flash)
    /// Linux: mt7996/mcu.c:3810-3824 mt7996_mcu_set_eeprom()
    ///
    /// UNI command MCU_WM_UNI_CMD(EFUSE_CTRL) (0x2d).
    /// Sends buffer_mode=0 (efuse), format=1 (whole).
    pub fn mcu_set_eeprom(&self, ring: &mut TxRing, seq: u8) -> Result<(), i32> {
        // struct mt7996_mcu_eeprom from mcu.h:150-158:
        //   u8 _rsv[4];          // uni_header
        //   le16 tag;            // UNI_EFUSE_BUFFER_MODE (2)
        //   le16 len;
        //   u8 buffer_mode;      // EE_MODE_EFUSE (0)
        //   u8 format;           // EE_FORMAT_WHOLE (1)
        //   le16 buf_len;        // 0
        // Total: 12 bytes
        let mut data = [0u8; 12];

        // tag = UNI_EFUSE_BUFFER_MODE (2)
        data[4..6].copy_from_slice(&UNI_EFUSE_BUFFER_MODE.to_le_bytes());
        // len = sizeof(req) - 4 = 8
        data[6..8].copy_from_slice(&8u16.to_le_bytes());
        // buffer_mode = EE_MODE_EFUSE (0)
        data[8] = EE_MODE_EFUSE;
        // format = EE_FORMAT_WHOLE (1)
        data[9] = EE_FORMAT_WHOLE;
        // buf_len = 0 (le16, already zero)

        // MCU_WM_UNI_CMD(EFUSE_CTRL) = __MCU_CMD_FIELD_UNI | 0x2d | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_EFUSE_CTRL as u32) | CMD_FIELD_WM;

        udebug!("mcu", "set_eeprom"; mode = EE_MODE_EFUSE, fmt = EE_FORMAT_WHOLE);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq)
    }

    /// Enable/disable radio for a band
    /// Linux: mt7996/mcu.c:4539-4558 mt7996_mcu_set_radio_en()
    ///
    /// UNI cmd MCU_WM_UNI_CMD(BAND_CONFIG) (0x08), tag=UNI_BAND_CONFIG_RADIO_ENABLE(0).
    /// Layout (12 bytes): {band_idx:u8, rsv[3], tag:le16, len:le16, enable:u8, rsv2[3]}
    /// Note: first 4 bytes are band_idx header (NOT empty uni_header — BAND_CONFIG specific).
    /// Wait: yes
    pub fn mcu_set_radio_en(&self, ring: &mut TxRing, band: u8, enable: bool, seq: u8) -> Result<(), i32> {
        // struct from mcu.c:4539-4558
        //   u8 band_idx;      // band index (NOT in uni_header position)
        //   u8 _rsv[3];
        //   le16 tag;         // UNI_BAND_CONFIG_RADIO_ENABLE (0)
        //   le16 len;         // sizeof(req) - 4 = 8
        //   u8 enable;
        //   u8 _rsv2[3];
        // Total: 12 bytes
        let mut data = [0u8; 12];

        // band_idx
        data[0] = band;
        // tag = UNI_BAND_CONFIG_RADIO_ENABLE (0)
        data[4..6].copy_from_slice(&UNI_BAND_CONFIG_RADIO_ENABLE.to_le_bytes());
        // len = 8
        data[6..8].copy_from_slice(&8u16.to_le_bytes());
        // enable
        data[8] = if enable { 1 } else { 0 };

        // MCU_WM_UNI_CMD(BAND_CONFIG) = __MCU_CMD_FIELD_UNI | 0x08 | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BAND_CONFIG as u32) | CMD_FIELD_WM;

        udebug!("mcu", "set_radio_en"; band = band, enable = enable as u8);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq)
    }

    /// Enable/disable thermal protection for a band
    /// Linux: mt7996/mcu.c:4105-4143 mt7996_mcu_set_thermal_protect()
    ///
    /// Always sends DISABLE first (tag=0x7, 12 bytes).
    /// If enable, follows with ENABLE (tag=0x6, 24 bytes) with temperature thresholds.
    /// Temps in raw °C: trigger=120, restore=110, sustain=10.
    /// Wait: no
    pub fn mcu_set_thermal_protect(&self, ring: &mut TxRing, band: u8, enable: bool, seq: u8) -> Result<(), i32> {
        // MCU_WM_UNI_CMD(THERMAL) = __MCU_CMD_FIELD_UNI | 0x35 | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_THERMAL as u32) | CMD_FIELD_WM;

        // Step 1: Always send DISABLE first — mcu.c:4130-4132
        // struct { u8 _rsv[4]; le16 tag; le16 len; thermal_ctrl(4 bytes) } = 12 bytes
        // thermal_ctrl: { ctrl_id:0, band_idx, protect_type:1, trigger_type:1 }
        {
            let mut data = [0u8; 12];
            // uni_header = 0 (4 bytes)
            // tag = UNI_CMD_THERMAL_PROTECT_DISABLE (7)
            data[4..6].copy_from_slice(&UNI_CMD_THERMAL_PROTECT_DISABLE.to_le_bytes());
            // len = sizeof(req) - 4 = 8  (excludes uni_header, includes tag+len+ctrl)
            data[6..8].copy_from_slice(&8u16.to_le_bytes());
            // thermal_ctrl: ctrl_id=0, band_idx, protect_type=1, trigger_type=1
            data[8] = 0;       // ctrl_id
            data[9] = band;    // band_idx
            data[10] = 1;      // protect_type (duty admit)
            data[11] = 1;      // trigger_type (high)

            udebug!("mcu", "thermal_protect_disable"; band = band);
            self.mcu_send_uni_cmd(ring, cmd, &data, false, seq)?;
        }

        if !enable {
            return Ok(());
        }

        // Step 2: Send ENABLE with temperature thresholds — mcu.c:4135-4142
        // struct { u8 _rsv[4]; le16 tag; le16 len; thermal_ctrl(4); thermal_enable(12) } = 24 bytes
        // thermal_enable: { trigger_temp:le32, restore_temp:le32, sustain_time:le16, rsv[2] }
        {
            let mut data = [0u8; 24];
            // tag = UNI_CMD_THERMAL_PROTECT_ENABLE (6)
            data[4..6].copy_from_slice(&UNI_CMD_THERMAL_PROTECT_ENABLE.to_le_bytes());
            // len = sizeof(req) - 4 = 20
            data[6..8].copy_from_slice(&20u16.to_le_bytes());
            // thermal_ctrl: ctrl_id=0, band_idx, protect_type=1, trigger_type=1
            data[8] = 0;
            data[9] = band;
            data[10] = 1;      // protect_type
            data[11] = 1;      // trigger_type
            // thermal_enable: trigger_temp (120°C)
            data[12..16].copy_from_slice(&MT7996_MAX_TEMP.to_le_bytes());
            // restore_temp (110°C)
            data[16..20].copy_from_slice(&MT7996_CRIT_TEMP.to_le_bytes());
            // sustain_time = 10 (SUSTAIN_PERIOD from mcu.c:4106)
            data[20..22].copy_from_slice(&10u16.to_le_bytes());

            udebug!("mcu", "thermal_protect_enable"; band = band, trigger = MT7996_MAX_TEMP, restore = MT7996_CRIT_TEMP);
            self.mcu_send_uni_cmd(ring, cmd, &data, false, seq)?;
        }

        Ok(())
    }

    /// Set thermal throttling duty cycle for a band
    /// Linux: mt7996/mcu.c:4072-4103 mt7996_mcu_set_thermal_throttling()
    ///
    /// Sends 4 commands for duty levels 0-3, each halving the duty cycle:
    ///   Level 0: duty_cycle=state, Level 1: state/2, Level 2: state/4, Level 3: state/8
    /// Wait: no
    pub fn mcu_set_thermal_throttling(&self, ring: &mut TxRing, band: u8, state: u8, seq: u8) -> Result<(), i32> {
        // MCU_WM_UNI_CMD(THERMAL) = __MCU_CMD_FIELD_UNI | 0x35 | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_THERMAL as u32) | CMD_FIELD_WM;

        // struct { u8 _rsv[4]; le16 tag; le16 len; thermal_ctrl(4) } = 12 bytes
        // thermal_ctrl.duty: { ctrl_id:0, band_idx, duty_level, duty_cycle }
        let mut duty_cycle = state;

        for level in 0u8..4 {
            let mut data = [0u8; 12];
            // tag = UNI_CMD_THERMAL_PROTECT_DUTY_CONFIG (8)
            data[4..6].copy_from_slice(&UNI_CMD_THERMAL_PROTECT_DUTY_CONFIG.to_le_bytes());
            // len = sizeof(req) - 4 = 8
            data[6..8].copy_from_slice(&8u16.to_le_bytes());
            // thermal_ctrl.duty: ctrl_id=0, band_idx, duty_level, duty_cycle
            data[8] = 0;          // ctrl_id
            data[9] = band;       // band_idx
            data[10] = level;     // duty_level
            data[11] = duty_cycle; // duty_cycle

            self.mcu_send_uni_cmd(ring, cmd, &data, false, seq)?;
            duty_cycle /= 2;
        }

        udebug!("mcu", "thermal_throttle_set"; band = band, state = state);
        Ok(())
    }

    // ========================================================================
    // Band 0 Radio Startup + Interface Creation MCU Commands
    // ========================================================================

    /// Set RTS threshold for a band
    /// Linux: mt7996/mcu.c:4517-4537 mt7996_mcu_set_rts_thresh()
    ///
    /// UNI cmd MCU_WM_UNI_CMD(BAND_CONFIG) (0x08), tag=UNI_BAND_CONFIG_RTS_THRESHOLD(0x08).
    /// Layout (16 bytes): {band_idx:u8, rsv[3], tag:le16, len:le16=12, len_thresh:le32, pkt_thresh:le32=0x2}
    /// Wait: yes
    pub fn mcu_set_rts_thresh(&self, ring: &mut TxRing, band: u8, val: u32, seq: u8) -> Result<(), i32> {
        let mut data = [0u8; 16];

        // band_idx
        data[0] = band;
        // tag = UNI_BAND_CONFIG_RTS_THRESHOLD (0x08)
        data[4..6].copy_from_slice(&UNI_BAND_CONFIG_RTS_THRESHOLD.to_le_bytes());
        // len = 12
        data[6..8].copy_from_slice(&12u16.to_le_bytes());
        // len_thresh = val
        data[8..12].copy_from_slice(&val.to_le_bytes());
        // pkt_thresh = 0x2
        data[12..16].copy_from_slice(&2u32.to_le_bytes());

        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BAND_CONFIG as u32) | CMD_FIELD_WM;

        udebug!("mcu", "set_rts_thresh"; band = band, val = val);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq)
    }

    /// Set channel info (switch or RX path)
    /// Linux: mt7996/mcu.c:3696-3762 mt7996_mcu_set_chan_info()
    ///
    /// UNI cmd MCU_WMWA_UNI_CMD(CHANNEL_SWITCH) (0x34) — both WM+WA.
    /// tag: UNI_CHANNEL_SWITCH(0) or UNI_CHANNEL_RX_PATH(1)
    /// Wait: yes
    pub fn mcu_set_chan_info(&self, ring: &mut TxRing, band: u8, tag: u16,
                            channel: u8, bw: u8, channel_band: u8, seq: u8) -> Result<(), i32> {
        // 76 bytes: {rsv[4], tag:le16, len:le16=72, fields...}
        let mut data = [0u8; 76];

        // tag
        data[4..6].copy_from_slice(&tag.to_le_bytes());
        // len = 72
        data[6..8].copy_from_slice(&72u16.to_le_bytes());

        // control_ch = channel
        data[8] = channel;
        // center_ch = channel (same for 20MHz)
        data[9] = channel;
        // bw
        data[10] = bw;

        // tx_path_num: count of TX antennas (MT7996 band 0 = 2T2R → 2)
        // Linux: hweight16(phy->chainmask) — band 0 chainmask = 0x3 → 2
        data[11] = 2;

        // rx_path: depends on tag
        // For UNI_CHANNEL_RX_PATH: bitmask (chainmask = 0x3)
        // For UNI_CHANNEL_SWITCH: hweight8(rx_path) = count of bits
        // Linux mcu.c:3731-3735
        if tag == UNI_CHANNEL_RX_PATH {
            data[12] = 0x3;  // chainmask bitmask
        } else {
            data[12] = 2;    // hweight8(0x3) = 2
        }

        // switch_reason
        data[13] = CH_SWITCH_NORMAL;
        // band_idx
        data[14] = band;
        // center_ch2 = 0 (not 80+80)
        // cac_case = 0
        // channel_band
        data[18] = channel_band;
        // rest is zeros (outband_freq, txpower_drop, ap_bw, ap_center_ch, rsv)

        // MCU_WMWA_UNI_CMD(CHANNEL_SWITCH) = __MCU_CMD_FIELD_UNI | 0x34 | WM | WA
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_CHANNEL_SWITCH as u32) | CMD_FIELD_WM | CMD_FIELD_WA;

        udebug!("mcu", "set_chan_info"; band = band, tag = tag, ch = channel, bw = bw);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq)
    }

    /// Enable/disable RX header translation
    /// Linux: mt7996/mcu.c:3387-3420 mt7996_mcu_set_hdr_trans()
    ///
    /// UNI cmd MCU_WM_UNI_CMD(RX_HDR_TRANS) (0x12) — WM only.
    /// Multi-TLV: HDR_TRANS_EN + HDR_TRANS_VLAN + HDR_TRANS_BLACKLIST (if enable)
    /// Wait: yes
    pub fn mcu_set_hdr_trans(&self, ring: &mut TxRing, enable: bool, seq: u8) -> Result<(), i32> {
        // 4-byte uni_header + 3 TLVs (8 bytes each) = 28 bytes when enabled
        // 4-byte uni_header + 2 TLVs (8 bytes each) = 20 bytes when disabled
        let total = if enable { 28 } else { 20 };
        let mut data = [0u8; 28];

        // uni_header = 0 (4 bytes)
        let mut off = 4usize;

        // TLV1: HDR_TRANS_EN — mcu.c:3395-3402
        data[off..off + 2].copy_from_slice(&UNI_HDR_TRANS_EN.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&8u16.to_le_bytes());
        data[off + 4] = if enable { 1 } else { 0 };  // enable
        data[off + 5] = 0;  // check_bssid
        data[off + 6] = 0;  // mode
        off += 8;

        // TLV2: HDR_TRANS_VLAN — mcu.c:3404-3409
        data[off..off + 2].copy_from_slice(&UNI_HDR_TRANS_VLAN.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&8u16.to_le_bytes());
        // insert_vlan=0, remove_vlan=0, tid=0
        off += 8;

        // TLV3: HDR_TRANS_BLACKLIST (only if enable) — mcu.c:3411-3418
        if enable {
            data[off..off + 2].copy_from_slice(&UNI_HDR_TRANS_BLACKLIST.to_le_bytes());
            data[off + 2..off + 4].copy_from_slice(&8u16.to_le_bytes());
            data[off + 4] = 0;  // idx
            data[off + 5] = 1;  // enable
            data[off + 6..off + 8].copy_from_slice(&ETH_P_PAE.to_le_bytes());  // type
        }

        // MCU_WM_UNI_CMD(RX_HDR_TRANS) = __MCU_CMD_FIELD_UNI | 0x12 | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_RX_HDR_TRANS as u32) | CMD_FIELD_WM;

        udebug!("mcu", "set_hdr_trans"; enable = enable as u8);
        self.mcu_send_uni_cmd(ring, cmd, &data[..total], true, seq)
    }

    /// Add/remove device info (OMAC)
    /// Linux: mt7996/mcu.c:2623-2651 mt7996_mcu_add_dev_info()
    ///
    /// UNI cmd MCU_WMWA_UNI_CMD(DEV_INFO_UPDATE) (0x01) — both WM+WA.
    /// Wait: yes
    pub fn mcu_add_dev_info(&self, ring: &mut TxRing, band: u8, omac_idx: u8,
                            mac_addr: &[u8; 6], enable: bool, seq: u8) -> Result<(), i32> {
        // hdr(4) + DEV_INFO_ACTIVE tlv: tag(2) + len(2) + active(1) + rsv(1) + omac_addr(6) = 16
        let mut data = [0u8; 16];

        // Header: omac_idx, band_idx, rsv[2]
        data[0] = omac_idx;
        data[1] = band;

        // TLV: DEV_INFO_ACTIVE — mcu.c:2632-2643
        data[4..6].copy_from_slice(&DEV_INFO_ACTIVE.to_le_bytes());
        // len = 12 (includes tag+len itself)
        data[6..8].copy_from_slice(&12u16.to_le_bytes());
        data[8] = if enable { 1 } else { 0 };  // active
        // rsv = 0
        data[10..16].copy_from_slice(mac_addr);  // omac_addr[6]

        // MCU_WMWA_UNI_CMD(DEV_INFO_UPDATE) = __MCU_CMD_FIELD_UNI | 0x01 | WM | WA
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_DEV_INFO_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;

        udebug!("mcu", "add_dev_info"; band = band, omac = omac_idx, enable = enable as u8);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq)
    }

    /// Add/remove BSS info
    /// Linux: mt7996/mcu.c:1123-1185 mt7996_mcu_bss_basic_tlv()
    ///        mt7996/mcu.c:1216-1222 mt7996_mcu_bss_sec_tlv()
    ///
    /// UNI cmd MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE) (0x02) — both WM+WA.
    /// Minimal TLVs: BSS_INFO_BASIC + BSS_INFO_SEC
    /// Wait: yes
    pub fn mcu_add_bss_info(&self, ring: &mut TxRing, band: u8, omac_idx: u8,
                            hw_bss_idx: u8, mac_addr: &[u8; 6], enable: bool, seq: u8) -> Result<(), i32> {
        // Layout: uni_header(4) + BSS_INFO_BASIC(32) + BSS_INFO_SEC(8) = 44 bytes
        let mut data = [0u8; 44];

        // uni_header = 0 (4 bytes)

        // === BSS_INFO_BASIC TLV (32 bytes) — mcu.c:1123-1185 ===
        let off = 4;
        // tag = UNI_BSS_INFO_BASIC (0)
        data[off..off + 2].copy_from_slice(&UNI_BSS_INFO_BASIC.to_le_bytes());
        // len = 32
        data[off + 2..off + 4].copy_from_slice(&32u16.to_le_bytes());
        // active
        data[off + 4] = if enable { 1 } else { 0 };
        // omac_idx
        data[off + 5] = omac_idx;
        // hw_bss_idx
        data[off + 6] = hw_bss_idx;
        // band_idx
        data[off + 7] = band;
        // conn_type:le32 = CONNECTION_INFRA_STA
        data[off + 8..off + 12].copy_from_slice(&CONNECTION_INFRA_STA.to_le_bytes());
        // conn_state = DISCONNECT
        data[off + 12] = CONN_STATE_DISCONNECT;
        // wmm_idx = 0
        // bssid[6] = zeros (unassociated)
        // bmc_tx_wlan_idx:le16 = 0
        // bcn_interval:le16 = 100
        data[off + 22..off + 24].copy_from_slice(&100u16.to_le_bytes());
        // dtim_period = 1
        data[off + 24] = 1;
        // phymode = 0
        // sta_idx:le16 = 0
        // nonht_basic_phy:le16 = 0
        // phymode_ext = 0
        // link_idx = 0

        // === BSS_INFO_SEC TLV (8 bytes) — mcu.c:1216-1222 ===
        let off2 = 36;
        // tag = UNI_BSS_INFO_SEC (16)
        data[off2..off2 + 2].copy_from_slice(&UNI_BSS_INFO_SEC.to_le_bytes());
        // len = 8
        data[off2 + 2..off2 + 4].copy_from_slice(&8u16.to_le_bytes());
        // rsv[2] = 0
        // cipher = 0 (NONE)
        // rsv = 0

        // MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE) = __MCU_CMD_FIELD_UNI | 0x02 | WM | WA
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BSS_INFO_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;

        udebug!("mcu", "add_bss_info"; band = band, omac = omac_idx, bss = hw_bss_idx, enable = enable as u8);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq)
    }

    /// Add/update STA record
    /// Linux: mt7996/mcu.c:2438-2477 mt7996_mcu_add_sta()
    ///        mt76_connac_mcu.c:388-415 mt76_connac_mcu_sta_basic_tlv()
    ///
    /// UNI cmd MCU_WMWA_UNI_CMD(STA_REC_UPDATE) (0x03) — both WM+WA.
    /// Layout: sta_req_hdr(8) + STA_REC_BASIC(20) = 28 bytes
    /// Wait: yes
    pub fn mcu_add_sta(&self, ring: &mut TxRing, bss_idx: u8, wlan_idx: u16,
                       omac_idx: u8, conn_state: u8, mac_addr: &[u8; 6],
                       newly: bool, seq: u8) -> Result<(), i32> {
        let mut data = [0u8; 28];

        // === sta_req_hdr (8 bytes) — mt76_connac_mcu.h ===
        // bss_idx
        data[0] = bss_idx;
        // wlan_idx_lo (low 8 bits)
        data[1] = wlan_idx as u8;
        // tlv_num = 1
        data[2..4].copy_from_slice(&1u16.to_le_bytes());
        // is_tlv_append = 1
        data[4] = 1;
        // muar_idx = omac_idx
        data[5] = omac_idx;
        // wlan_idx_hi (high 8 bits)
        data[6] = (wlan_idx >> 8) as u8;
        // rsv = 0

        // === STA_REC_BASIC TLV (20 bytes) — mt76_connac_mcu.c:388-415 ===
        let off = 8;
        // tag = STA_REC_BASIC (0)
        data[off..off + 2].copy_from_slice(&STA_REC_BASIC.to_le_bytes());
        // len = 20
        data[off + 2..off + 4].copy_from_slice(&20u16.to_le_bytes());
        // conn_type:le32 = CONNECTION_INFRA_STA
        data[off + 4..off + 8].copy_from_slice(&CONNECTION_INFRA_STA.to_le_bytes());
        // conn_state
        data[off + 8] = conn_state;
        // qos = 1
        data[off + 9] = 1;
        // aid:le16 = 0
        // peer_addr[6]
        data[off + 12..off + 18].copy_from_slice(mac_addr);
        // extra_info:le16
        let mut extra = EXTRA_INFO_VER;
        if newly {
            extra |= EXTRA_INFO_NEW;
        }
        data[off + 18..off + 20].copy_from_slice(&extra.to_le_bytes());

        // MCU_WMWA_UNI_CMD(STA_REC_UPDATE) = __MCU_CMD_FIELD_UNI | 0x03 | WM | WA
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;

        udebug!("mcu", "add_sta"; bss = bss_idx, wlan = wlan_idx, state = conn_state);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq)
    }

    /// Program a fixed rate table entry
    /// Linux: mt7996/mcu.c:4604-4631 mt7996_mcu_set_fixed_rate_table()
    ///
    /// UNI command MCU_WM_UNI_CMD(FIXED_RATE_TABLE) (0x40), tag=0.
    /// Used for basic rate programming (12 CCK+OFDM rates).
    pub fn mcu_set_fixed_rate_table(&self, ring: &mut TxRing, table_idx: u8, rate_idx: u16, seq: u8) -> Result<(), i32> {
        // struct fixed_rate_table_ctrl from mcu.h:954-972:
        //   u8 _rsv[4];          // uni_header
        //   le16 tag;            // UNI_FIXED_RATE_TABLE_SET (0)
        //   le16 len;
        //   u8 table_idx;
        //   u8 antenna_idx;
        //   le16 rate_idx;
        //   u8 spe_idx_sel;      // SPE_IXD_SELECT_BMC_WTBL (1) for non-beacon
        //   u8 spe_idx;
        //   u8 gi;               // 1
        //   u8 he_ltf;           // 1
        //   bool ldpc;
        //   bool txbf;
        //   bool dynamic_bw;
        //   u8 _rsv2;
        // Total: 4 + 16 = 20 bytes
        let mut data = [0u8; 20];

        // tag = UNI_FIXED_RATE_TABLE_SET (0)
        data[4..6].copy_from_slice(&0u16.to_le_bytes());
        // len = sizeof(req) - 4 = 16
        data[6..8].copy_from_slice(&16u16.to_le_bytes());
        // table_idx
        data[8] = table_idx;
        // antenna_idx = 0
        // rate_idx
        data[10..12].copy_from_slice(&rate_idx.to_le_bytes());
        // spe_idx_sel = SPE_IXD_SELECT_BMC_WTBL (1) for non-beacon
        data[12] = 1;
        // spe_idx = 0
        // gi = 1
        data[14] = 1;
        // he_ltf = 1
        data[15] = 1;
        // ldpc, txbf, dynamic_bw, _rsv2 = 0

        // MCU_WM_UNI_CMD(FIXED_RATE_TABLE) = __MCU_CMD_FIELD_UNI | 0x40 | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_FIXED_RATE_TABLE as u32) | CMD_FIELD_WM;

        self.mcu_send_uni_cmd(ring, cmd, &data, false, seq)
    }
}
