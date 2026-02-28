//! MCU Command Protocol — McuTxd, UniTxd, command encoding, send/wait
//!
//! EXACT translation of Linux mt76_connac_mcu.h and mt7996/mcu.c.
//! Contains both the legacy MCU TXD (for firmware download and WA commands)
//! and the UNI TXD (for post-firmware init commands).

use userlib::{uerror, uwarn, udebug};
use userlib::ipc::{Irq, Mux, MuxFilter};
use crate::regs::*;
use crate::device::Mt7996Dev;
use crate::dma::{TxRing, Mt76Desc, dma_wmb, flush_buffer};

/// IRQ waiter with a pre-created Mux.
///
/// Created once before firmware upload and reused for all DMA waits.
/// The Mux timeout is set once at creation; `wait()` uses a single read
/// syscall per wake. This keeps syscall rate low even if the device fires
/// interrupts faster than expected.
pub struct FwIrq {
    pub irq: Irq,
    mux: Mux,
}

impl FwIrq {
    /// Create a new FwIrq from an existing Irq.
    /// Allocates a Mux, registers the Irq handle, and sets a 20ms timeout.
    pub fn new(irq: Irq) -> Result<Self, i32> {
        let mux = Mux::new().map_err(|_| -1i32)?;
        mux.add(irq.handle(), MuxFilter::Readable).map_err(|_| -1i32)?;
        // Set timeout once — stays until changed. Each wait() is 1 read syscall.
        let _ = mux.set_timeout(20);
        Ok(Self { irq, mux })
    }

    /// Block until IRQ fires or timeout expires.
    /// Single read syscall — timeout was set at creation.
    pub fn wait(&self) -> bool {
        self.mux.wait().is_ok()
    }

    /// Acknowledge the interrupt (clears pending, re-enables at kernel).
    pub fn ack(&mut self) {
        let _ = self.irq.ack();
    }

    /// Consume the Irq out of this wrapper (for post-firmware Mux transfer).
    pub fn into_irq(self) -> Irq {
        // Mux is dropped here, releasing its handle
        self.irq
    }
}

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
pub const MCU_UNI_CMD_EDCA_UPDATE: u16 = 0x04;
pub const MCU_UNI_CMD_WSYS_CONFIG: u16 = 0x0b;
pub const MCU_UNI_CMD_EFUSE_CTRL: u16 = 0x2d;
pub const MCU_UNI_CMD_VOW: u16 = 0x37;
pub const MCU_UNI_CMD_BF: u16 = 0x33;
pub const MCU_UNI_CMD_FIXED_RATE_TABLE: u16 = 0x40;
pub const MCU_UNI_CMD_RRO: u16 = 0x57;
pub const MCU_UNI_CMD_OFFCH_SCAN_CTRL: u16 = 0x58; // mt76_connac_mcu.h:1307
pub const MCU_UNI_CMD_ALL_STA_INFO: u16 = 0x6e;
pub const MCU_UNI_CMD_SDO: u16 = 0x88;

/// UNI TLV tags — from mt7996/mcu.h
const UNI_WSYS_CONFIG_FW_LOG_CTRL: u16 = 0;
const UNI_VOW_RX_AT_AIRTIME_EN: u16 = 0x0b;
const UNI_VOW_RX_AT_AIRTIME_CLR_EN: u16 = 0x0e;

/// EFUSE TLV tags — from mt7996/mcu.h enum (starts at ACCESS=1)
const UNI_EFUSE_ACCESS: u16 = 1;
const UNI_EFUSE_BUFFER_MODE: u16 = 2;
const UNI_EFUSE_FREE_BLOCK: u16 = 3;

/// EEPROM mode/format — from mt76_connac_mcu.h
const _EE_MODE_EFUSE: u8 = 0;
const EE_MODE_BUFFER: u8 = 1;
const EE_FORMAT_WHOLE: u8 = 1;

/// Flash-mode page size — from mt7996/mcu.c:3768
const PER_PAGE_SIZE: usize = 0x400; // 1024 bytes

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

/// Find the byte offset of an IE (by tag) within a raw 802.11 beacon frame.
/// Searches the IE area after the fixed beacon fields.
/// Returns offset from frame start, or None if not found.
fn find_ie_offset(frame: &[u8], ie_tag: u8) -> Option<usize> {
    // Beacon: MAC hdr(24) + timestamp(8) + interval(2) + cap(2) = 36
    let mut pos = 36;
    while pos + 2 <= frame.len() {
        if frame[pos] == ie_tag {
            return Some(pos);
        }
        let ie_len = frame[pos + 1] as usize;
        pos += 2 + ie_len;
    }
    None
}

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
    pub fn mcu_send_uni_cmd(&self, ring: &mut TxRing, cmd: u32, data: &[u8], wait: bool, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        // Linux: mt7996_mcu_send_message() lines 295-298
        // WM ring → responses on MCU_WM RX (q0), WA ring → responses on MCU_WA RX (q1)
        let rx_snap = if wait { self.snapshot_rx_idx(ring.rx_regs) } else { 0 };

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

        // Clear stale interrupt state before waiting for this command's response.
        // This is a one-shot clear (not in the wait loop) — gives a clean slate
        // so irq.wait() in wait_rx_response blocks until the NEW response arrives.
        if irq.is_some() {
            let saved_mask = self.mt76_rr(MT_INT_MASK_CSR);
            self.mt76_wr(MT_INT_MASK_CSR, 0);
            let src = self.mt76_rr(MT_INT_SOURCE_CSR);
            if src != 0 {
                self.mt76_wr(MT_INT_SOURCE_CSR, src);
            }
            self.mt76_wr(MT_INT_MASK_CSR, saved_mask);
        }

        // Wait for TX completion
        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done(ring, prev_idx, 2000) {
            uerror!("mcu", "uni_cmd_tx_timeout"; cid = mcu_cmd);
            return Err(-1);
        }

        // Wait for MCU response on the corresponding RX queue
        if wait {
            let _ = self.wait_rx_response(ring, rx_snap, seq, 5000, irq)?;
        }

        Ok(())
    }

    /// Wait for TX descriptor to complete
    pub fn mcu_wait_tx_done(&self, ring: &TxRing, idx: u32, timeout_ms: u32) -> bool {
        self.mcu_wait_tx_done_irq(ring, idx, timeout_ms, None)
    }

    /// Wait for TX DMA completion on a descriptor.
    /// With IRQ: blocks on interrupt notification instead of busy-waiting.
    /// Without IRQ: falls back to delay_ms(1) polling.
    pub fn mcu_wait_tx_done_irq(&self, ring: &TxRing, idx: u32, timeout_ms: u32, irq: Option<&mut FwIrq>) -> bool {
        let desc = ring.desc(idx);

        // Quick check before waiting
        let ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
        if (ctrl & MT_DMA_CTL_DMA_DONE) != 0 {
            return true;
        }

        if let Some(irq) = irq {
            // IRQ-based: block until WFDMA interrupt, then check descriptor.
            let iterations = (timeout_ms + 19) / 20;
            for _ in 0..iterations {
                let _ = irq.wait();
                let src = self.mt76_rr(MT_INT_SOURCE_CSR);
                if src != 0 {
                    self.mt76_wr(MT_INT_SOURCE_CSR, src);
                }
                irq.ack();

                let ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
                if (ctrl & MT_DMA_CTL_DMA_DONE) != 0 {
                    return true;
                }
            }
            false
        } else {
            // Polling fallback
            for _ in 0..timeout_ms {
                let ctrl = unsafe { core::ptr::read_volatile(&(*desc).ctrl) };
                if (ctrl & MT_DMA_CTL_DMA_DONE) != 0 {
                    return true;
                }
                userlib::delay_ms(1);
            }
            false
        }
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

    /// Wait for MCU response — polls rx_regs DMA_IDX until a response with matching seq arrives.
    ///
    /// Fire-and-forget WA commands (mcu_wa_cmd, mcu_set_mwds) don't consume their
    /// RX responses. Those responses sit in the shared WA RX queue and may arrive
    /// at the position we're monitoring. We skip responses with non-matching seq
    /// numbers and keep scanning until we find ours or timeout.
    ///
    /// Linux matches responses by seq number (mt76_mcu_wait_response + parse_response),
    /// not by eid. Different response types (UNI, WA, QUERY) may use different eids.
    ///
    /// Response format (UNI commands):
    ///   mt76_connac2_mcu_rxd (44 bytes):
    ///     rxd[8] (32B) + len(2) + pkt_type(2) + eid(1) + seq(1) + option(1) + rsv(1) + ext_eid(1) + rsv[3]
    ///   mt7996_mcu_uni_event (8 bytes):
    ///     cid(1) + rsv[3] + status(le32)
    ///
    /// Linux checks status for DEV_INFO_UPDATE, BSS_INFO_UPDATE, STA_REC_UPDATE, RRO.
    /// Non-zero status = firmware rejected the command.
    pub fn wait_rx_response(&self, ring: &TxRing, pre_send_dma_idx: u32, expected_seq: u8, timeout_ms: u32, mut irq: Option<&mut FwIrq>) -> Result<u32, i32> {
        // Snapshot the alternate RX queue to detect misrouted responses
        let alt_regs = if ring.rx_regs == MCU_WA_RX_REGS { MCU_WM_RX_REGS } else { MCU_WA_RX_REGS };
        let alt_snap = self.mt76_rr(alt_regs + MT_QUEUE_DMA_IDX);

        let rx_ndesc: u32 = if ring.rx_regs == MCU_WA_RX_REGS {
            MT7996_RX_MCU_RING_SIZE_WA
        } else {
            MT7996_RX_MCU_RING_SIZE
        };

        // Current position to check in the RX ring
        let mut check_pos = pre_send_dma_idx;

        // IRQ path uses 20ms blocks, polling uses 1ms blocks
        let iterations = if irq.is_some() { (timeout_ms + 19) / 20 } else { timeout_ms };

        for _ in 0..iterations {
            // Check for new RX data — scan through non-matching entries
            'scan: loop {
                let dma_idx = self.mt76_rr(ring.rx_regs + MT_QUEUE_DMA_IDX);
                if dma_idx == check_pos {
                    break 'scan; // No new data
                }
                // New response(s) available — check from check_pos
                if ring.rx_buf_virt != 0 {
                    let buf_ptr = (ring.rx_buf_virt + check_pos as u64 * ring.rx_buf_size as u64) as *const u8;

                    // MCU RXD header fields
                    let rxd_eid = unsafe { core::ptr::read_volatile(buf_ptr.add(36)) };
                    let rxd_seq = unsafe { core::ptr::read_volatile(buf_ptr.add(37)) };
                    let rxd_ext_eid = unsafe { core::ptr::read_volatile(buf_ptr.add(40)) };

                    if rxd_seq != expected_seq {
                        // Response for a different command — skip and advance CPU_IDX
                        // to return consumed descriptors to hardware (like Linux NAPI).
                        udebug!("mcu", "rx_skip"; pos = check_pos, eid = rxd_eid,
                            seq = rxd_seq, expected = expected_seq, ext = rxd_ext_eid);
                        self.rx_advance_cpu_idx(ring, check_pos, rx_ndesc);
                        check_pos = (check_pos + 1) % rx_ndesc;
                        continue 'scan;
                    }

                    // Status check for UNI event responses (eid=1)
                    if rxd_eid == 1 {
                        let resp_cid = unsafe { core::ptr::read_volatile(buf_ptr.add(44)) };
                        let status = unsafe { core::ptr::read_volatile(buf_ptr.add(48) as *const u32) };
                        let status = u32::from_le(status);

                        if status != 0 {
                            uerror!("mcu", "cmd_REJECTED"; cid = resp_cid, status = status,
                                seq = rxd_seq, eid = rxd_eid, ext = rxd_ext_eid);
                            self.rx_advance_cpu_idx(ring, check_pos, rx_ndesc);
                            return Err(status as i32);
                        }
                    }

                    udebug!("mcu", "rx_ok"; snap = pre_send_dma_idx, pos = check_pos,
                        seq = rxd_seq, eid = rxd_eid, ext = rxd_ext_eid);
                } else {
                    udebug!("mcu", "rx_ok"; snap = pre_send_dma_idx, now = dma_idx);
                }
                // Advance CPU_IDX to return consumed descriptors to hardware
                self.rx_advance_cpu_idx(ring, check_pos, rx_ndesc);
                return Ok(check_pos);
            }

            // Wait for next check — IRQ-based or polling
            // Post-firmware: do NOT clear INT_SOURCE_CSR here. Empirically,
            // SOURCE clearing in the RX wait loop prevents MCU responses from
            // arriving (set_eeprom timeout). Use IRQ wait + delay for spin
            // protection only. Firmware loading uses separate wait functions
            // (wait_mcu_rx_response_irq) that DO clear SOURCE.
            if let Some(ref mut irq) = irq {
                let _ = irq.wait();
                irq.ack();
                userlib::delay_ms(1);
            } else {
                userlib::delay_ms(1);
            }
        }

        // Timeout — check all queues to diagnose where the response went
        let rx_base = MT_WFDMA0_BASE + 0x500;
        let q0 = self.mt76_rr(rx_base + 0 * MT_RING_SIZE + MT_QUEUE_DMA_IDX);
        let q1 = self.mt76_rr(rx_base + 1 * MT_RING_SIZE + MT_QUEUE_DMA_IDX);
        let q2 = self.mt76_rr(rx_base + 2 * MT_RING_SIZE + MT_QUEUE_DMA_IDX);
        let q3 = self.mt76_rr(rx_base + 3 * MT_RING_SIZE + MT_QUEUE_DMA_IDX);
        let alt_now = self.mt76_rr(alt_regs + MT_QUEUE_DMA_IDX);
        uerror!("mcu", "rx_response_timeout"; snap = pre_send_dma_idx, check = check_pos, seq = expected_seq, q0 = q0, q1 = q1, q2 = q2, q3 = q3, alt_snap = alt_snap, alt_now = alt_now);
        Err(-1)
    }

    /// No-op: CPU_IDX is now managed exclusively by rx_fill() via
    /// head/tail/queued tracking in RxQueueInfo. The MCU response handler
    /// reads the response buffer but does NOT advance CPU_IDX — that
    /// happens when rx_process_mcu() runs on the next timer tick.
    ///
    /// Writing CPU_IDX here would desync from the software head/tail/queued
    /// state, causing ring stalls at the ndesc wrap boundary.
    fn rx_advance_cpu_idx(&self, _ring: &TxRing, _pos: u32, _rx_ndesc: u32) {
        // Intentionally empty — rx_fill() owns CPU_IDX now
    }

    /// Legacy: wait on MCU_WM RX queue (used by firmware loading, no response parsing)
    pub fn wait_mcu_rx_response(&self, pre_send_dma_idx: u32, timeout_ms: u32) -> Result<(), i32> {
        self.wait_mcu_rx_response_irq(pre_send_dma_idx, timeout_ms, None)
    }

    /// Wait on MCU_WM RX queue with optional IRQ-based waiting.
    pub fn wait_mcu_rx_response_irq(&self, pre_send_dma_idx: u32, timeout_ms: u32, irq: Option<&mut FwIrq>) -> Result<(), i32> {
        // Quick check
        let dma_idx = self.mt76_rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX);
        if dma_idx != pre_send_dma_idx {
            return Ok(());
        }

        if let Some(irq) = irq {
            let iterations = (timeout_ms + 19) / 20;
            for _ in 0..iterations {
                let _ = irq.wait();
                let src = self.mt76_rr(MT_INT_SOURCE_CSR);
                if src != 0 {
                    self.mt76_wr(MT_INT_SOURCE_CSR, src);
                }
                irq.ack();

                let dma_idx = self.mt76_rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX);
                if dma_idx != pre_send_dma_idx {
                    return Ok(());
                }
            }
        } else {
            for _ in 0..timeout_ms {
                let dma_idx = self.mt76_rr(MCU_WM_RX_REGS + MT_QUEUE_DMA_IDX);
                if dma_idx != pre_send_dma_idx {
                    return Ok(());
                }
                userlib::delay_ms(1);
            }
        }
        uerror!("mcu", "rx_response_timeout_wm"; snap = pre_send_dma_idx);
        Err(-1)
    }

    /// Send init download command
    pub fn mcu_init_download(&self, ring: &mut TxRing, addr: u32, len: u32, mode: u32, seq: u8) -> Result<(), i32> {
        self.mcu_init_download_irq(ring, addr, len, mode, seq, None)
    }

    /// Send init download command with optional IRQ-based waiting.
    pub fn mcu_init_download_irq(&self, ring: &mut TxRing, addr: u32, len: u32, mode: u32, seq: u8, mut irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        if !self.mcu_wait_tx_done_irq(ring, prev_idx, 1000, irq.as_mut().map(|r| &mut **r)) {
            return Err(-1);
        }

        if let Err(_) = self.wait_mcu_rx_response_irq(rx_snap, 2000, irq) {
            return Err(-1);
        }

        Ok(())
    }

    /// Send firmware chunk (scatter command — raw data, NO TXD header)
    pub fn mcu_send_firmware_chunk(&self, ring: &mut TxRing, data: &[u8], _seq: u8) -> Result<(), i32> {
        self.mcu_send_firmware_chunk_irq(ring, data, _seq, None)
    }

    /// Send firmware chunk — fire-and-forget like Linux __mt76_mcu_send_firmware().
    ///
    /// Does NOT wait for each chunk's TX completion or RX response.
    /// Only waits if the TX ring is full (next slot not yet consumed by DMA).
    /// Linux: mt76_dma_tx_queue_skb_raw() + mt76_dma_tx_cleanup() between chunks.
    pub fn mcu_send_firmware_chunk_irq(&self, ring: &mut TxRing, data: &[u8], _seq: u8, _irq: Option<&mut FwIrq>) -> Result<(), i32> {
        // Check if next slot is available (ring full check).
        // Like Linux tx_cleanup — if DMA hasn't consumed the slot we need, wait briefly.
        let idx = ring.cpu_idx;
        let next_idx = (idx + 1) % ring.ndesc;
        if next_idx == self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX) {
            // Ring full — wait for DMA to consume at least one entry
            for _ in 0..100u32 {
                if next_idx != self.mt76_rr(ring.regs_base + MT_QUEUE_DMA_IDX) {
                    break;
                }
                userlib::delay_ms(1);
            }
        }

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

        Ok(())
    }

    /// Send patch/firmware start command
    pub fn mcu_start_firmware(&self, ring: &mut TxRing, is_patch: bool, option: u32, addr: u32, seq: u8) -> Result<(), i32> {
        self.mcu_start_firmware_irq(ring, is_patch, option, addr, seq, None)
    }

    /// Send patch/firmware start command with optional IRQ-based waiting.
    pub fn mcu_start_firmware_irq(&self, ring: &mut TxRing, is_patch: bool, option: u32, addr: u32, seq: u8, mut irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        if !self.mcu_wait_tx_done_irq(ring, prev_idx, 1000, irq.as_mut().map(|r| &mut **r)) {
            uerror!("wifid", "mcu_start_fw_timeout";);
            return Err(-1);
        }

        if let Err(_) = self.wait_mcu_rx_response_irq(rx_snap, 5000, irq) {
            return Err(-1);
        }
        Ok(())
    }

    /// Get/release patch semaphore
    pub fn mcu_patch_sem_ctrl(&self, ring: &mut TxRing, get: bool, seq: u8) -> Result<(), i32> {
        self.mcu_patch_sem_ctrl_irq(ring, get, seq, None)
    }

    /// Get/release patch semaphore with optional IRQ-based waiting.
    pub fn mcu_patch_sem_ctrl_irq(&self, ring: &mut TxRing, get: bool, seq: u8, mut irq: Option<&mut FwIrq>) -> Result<(), i32> {
        let req = if get { PatchSemCtrl::get() } else { PatchSemCtrl::release() };
        let rx_snap = self.snapshot_mcu_rx_idx();
        self.mcu_send_cmd(ring, mcu_cmd::PATCH_SEM_CTRL, req.as_bytes(), seq)?;

        let prev_idx = if ring.cpu_idx == 0 { ring.ndesc - 1 } else { ring.cpu_idx - 1 };
        if !self.mcu_wait_tx_done_irq(ring, prev_idx, 1000, irq.as_mut().map(|r| &mut **r)) {
            uerror!("wifid", "patch_sem_timeout";);
            return Err(-1);
        }

        self.wait_mcu_rx_response_irq(rx_snap, 2000, irq)?;
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
    pub fn mcu_fw_log_2_host(&self, ring: &mut TxRing, fw_type: u8, ctrl: u8, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
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
    pub fn mcu_init_rx_airtime(&self, ring: &mut TxRing, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
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
    pub fn mcu_set_rro(&self, ring: &mut TxRing, tag: u16, val: u16, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
    }

    /// Read an RF register via MCU.
    /// Linux: mt7996/mcu.c:4633-4670 mt7996_mcu_rf_regval()
    ///
    /// regidx = (chip_idx << 24) | register_offset
    /// Returns the 32-bit register value.
    pub fn mcu_rf_regval(&self, ring: &mut TxRing, regidx: u32, seq: u8, irq: Option<&mut FwIrq>) -> Result<u32, i32> {
        // Layout: rsv(4) + tag(2) + len(2) + idx(2) + rsv(2) + ofs(4) + data(4) = 20 bytes
        let mut data = [0u8; 20];

        // tag = UNI_CMD_ACCESS_RF_REG_BASIC (1)
        data[4..6].copy_from_slice(&UNI_CMD_ACCESS_RF_REG_BASIC.to_le_bytes());
        // len = 16
        data[6..8].copy_from_slice(&16u16.to_le_bytes());
        // idx = regidx[31:24] (chip index)
        let idx = ((regidx >> 24) & 0xFF) as u16;
        data[8..10].copy_from_slice(&idx.to_le_bytes());
        // ofs = regidx[23:0]
        let ofs = regidx & 0x00FFFFFF;
        data[12..16].copy_from_slice(&ofs.to_le_bytes());
        // data = 0 (read)

        // Snapshot RX index before sending
        let rx_snap = self.snapshot_rx_idx(ring.rx_regs);

        // MCU_WM_UNI_CMD_QUERY(REG_ACCESS)
        let cmd = CMD_FIELD_UNI | CMD_FIELD_QUERY | (MCU_UNI_CMD_REG_ACCESS as u32) | CMD_FIELD_WM;

        // Send without waiting — we call wait_rx_response ourselves to get actual position
        self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)?;
        let resp_pos = self.wait_rx_response(ring, rx_snap, seq, 5000, irq)?;

        // Read result from RX buffer
        // Response layout: RXD(44) + {rsv(4) + tag(2) + len(2) + idx(2) + rsv(2) + ofs(4) + data(4)}
        // data field at raw offset 44 + 16 = 60
        if ring.rx_buf_virt == 0 {
            return Err(-1);
        }
        let buf_ptr = (ring.rx_buf_virt + resp_pos as u64 * ring.rx_buf_size as u64) as *const u8;
        let val = unsafe {
            u32::from_le(core::ptr::read_volatile(buf_ptr.add(60) as *const u32))
        };
        Ok(val)
    }

    /// Tell MCU about EEPROM mode (efuse, not flash)
    /// Linux: mt7996/mcu.c:3810-3824 mt7996_mcu_set_eeprom()
    ///
    /// UNI command MCU_WM_UNI_CMD(EFUSE_CTRL) (0x2d).
    /// Sends buffer_mode=0 (efuse), format=1 (whole).
    pub fn mcu_set_eeprom(&self, ring: &mut TxRing, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        data[8] = _EE_MODE_EFUSE;
        // format = EE_FORMAT_WHOLE (1)
        data[9] = EE_FORMAT_WHOLE;
        // buf_len = 0 (le16, already zero)

        // MCU_WM_UNI_CMD(EFUSE_CTRL) = __MCU_CMD_FIELD_UNI | 0x2d | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_EFUSE_CTRL as u32) | CMD_FIELD_WM;

        udebug!("mcu", "set_eeprom"; mode = 0u8, fmt = EE_FORMAT_WHOLE);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
    }

    /// Query number of free eFuse blocks from firmware.
    /// Linux: mt7996/mcu.c:3872-3900 mt7996_mcu_get_eeprom_free_block()
    ///
    /// Uses MCU_WM_UNI_CMD_QUERY(EFUSE_CTRL) — the QUERY variant sets CMD_FIELD_QUERY
    /// which changes option from MCU_CMD_UNI_EXT_ACK(7) to MCU_CMD_UNI_QUERY_ACK(3).
    ///
    /// If free_block_num >= 59, eFuse is empty → need default EEPROM binary (flash upload).
    /// Response: free_block_num at RXD(44) + offset 8 in response payload.
    pub fn mcu_get_eeprom_free_block(&self, ring: &mut TxRing, seq: u8, irq: Option<&mut FwIrq>) -> Result<u8, i32> {
        // struct {
        //     u8 _rsv[4];          // uni_header
        //     __le16 tag;          // UNI_EFUSE_FREE_BLOCK (3)
        //     __le16 len;          // sizeof(req) - 4 = 8
        //     u8 num;              // 0 (filled by firmware in response)
        //     u8 version;          // 2
        //     u8 die_idx;          // 0
        //     u8 _rsv2;            // 0
        // } = 12 bytes total
        let mut data = [0u8; 12];

        // tag = UNI_EFUSE_FREE_BLOCK (3)
        data[4..6].copy_from_slice(&UNI_EFUSE_FREE_BLOCK.to_le_bytes());
        // len = sizeof(req) - 4 = 8
        data[6..8].copy_from_slice(&8u16.to_le_bytes());
        // num = 0 (already zero)
        // version = 2
        data[9] = 2;
        // die_idx = 0 (already zero)

        // Snapshot RX index before sending — need this to find response buffer
        let rx_snap = self.snapshot_rx_idx(ring.rx_regs);

        // MCU_WM_UNI_CMD_QUERY(EFUSE_CTRL) = CMD_FIELD_UNI | CMD_FIELD_QUERY | CMD_FIELD_WM | 0x2d
        let cmd = CMD_FIELD_UNI | CMD_FIELD_QUERY | (MCU_UNI_CMD_EFUSE_CTRL as u32) | CMD_FIELD_WM;

        // Send without waiting — we call wait_rx_response ourselves to get actual position
        self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)?;
        let resp_pos = self.wait_rx_response(ring, rx_snap, seq, 5000, irq)?;

        // Read response: free_block_num at RXD(44) + offset 8 = byte 52
        if ring.rx_buf_virt != 0 {
            let buf_ptr = (ring.rx_buf_virt + resp_pos as u64 * ring.rx_buf_size as u64) as *const u8;
            let free_blocks = unsafe { core::ptr::read_volatile(buf_ptr.add(52)) };
            udebug!("mcu", "efuse_free_blocks"; count = free_blocks);
            Ok(free_blocks)
        } else {
            // No RX buffer info — can't read response, assume eFuse empty
            uwarn!("mcu", "efuse_free_block_no_rx_buf");
            Ok(59)
        }
    }

    /// Read a 16-byte EEPROM block from eFuse via firmware.
    /// Linux: mt7996/mcu.c:3826-3870 mt7996_mcu_get_eeprom()
    ///
    /// Uses MCU_WM_UNI_CMD_QUERY(EFUSE_CTRL) with tag UNI_EFUSE_ACCESS(1).
    /// Response: valid flag at RXD(44)+16, data at RXD(44)+48 (16 bytes).
    /// Returns 16 bytes of EEPROM data at the given offset, or Err if invalid.
    pub fn mcu_get_eeprom(&self, ring: &mut TxRing, offset: u32, seq: u8, irq: Option<&mut FwIrq>) -> Result<[u8; 16], i32> {
        // struct {
        //     u8 _rsv[4];          // uni_header
        //     __le16 tag;          // UNI_EFUSE_ACCESS (1)
        //     __le16 len;          // sizeof(req) - 4 = 28
        //     __le32 addr;         // offset (rounded down to 16-byte block)
        //     __le32 valid;        // 0
        //     u8 data[16];         // 0
        // } = 32 bytes total
        let mut data = [0u8; 32];

        // tag = UNI_EFUSE_ACCESS (1)
        data[4..6].copy_from_slice(&UNI_EFUSE_ACCESS.to_le_bytes());
        // len = 28
        data[6..8].copy_from_slice(&28u16.to_le_bytes());
        // addr = offset (rounded to 16-byte boundary)
        let aligned_offset = offset & !0xF;
        data[8..12].copy_from_slice(&aligned_offset.to_le_bytes());

        // Snapshot RX index before sending
        let rx_snap = self.snapshot_rx_idx(ring.rx_regs);

        // MCU_WM_UNI_CMD_QUERY(EFUSE_CTRL) = CMD_FIELD_UNI | CMD_FIELD_QUERY | CMD_FIELD_WM | 0x2d
        let cmd = CMD_FIELD_UNI | CMD_FIELD_QUERY | (MCU_UNI_CMD_EFUSE_CTRL as u32) | CMD_FIELD_WM;

        // Send without waiting — we call wait_rx_response ourselves to get actual position
        self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)?;
        let resp_pos = self.wait_rx_response(ring, rx_snap, seq, 5000, irq)?;

        // Read response from RX buffer
        if ring.rx_buf_virt == 0 {
            return Err(-1);
        }
        let buf_ptr = (ring.rx_buf_virt + resp_pos as u64 * ring.rx_buf_size as u64) as *const u8;

        // Dump first 16 u32s of response payload (after 44-byte RXD) for debugging
        let w0 = unsafe { u32::from_le(core::ptr::read_volatile(buf_ptr.add(44) as *const u32)) };
        let w1 = unsafe { u32::from_le(core::ptr::read_volatile(buf_ptr.add(48) as *const u32)) };
        let w2 = unsafe { u32::from_le(core::ptr::read_volatile(buf_ptr.add(52) as *const u32)) };
        let w3 = unsafe { u32::from_le(core::ptr::read_volatile(buf_ptr.add(56) as *const u32)) };
        let w4 = unsafe { u32::from_le(core::ptr::read_volatile(buf_ptr.add(60) as *const u32)) };
        let w5 = unsafe { u32::from_le(core::ptr::read_volatile(buf_ptr.add(64) as *const u32)) };
        udebug!("mcu", "efuse_resp_dump"; snap = rx_snap, w0 = w0, w1 = w1, w2 = w2, w3 = w3, w4 = w4, w5 = w5);

        // Response layout after RXD (44 bytes):
        //   mt7996_mcu_uni_event: {cid(1), rsv(3), status(4)} = 8 bytes
        //   TLV: {tag(2), len(2), addr(4), valid(4), ...padding..., data(16)}
        //   Linux: skb->data+16 = valid, skb_pull(48) then data
        //   Raw offsets: valid at 44+16=60, data at 44+48=92
        let valid = unsafe {
            let v = core::ptr::read_volatile(buf_ptr.add(44 + 16) as *const u32);
            u32::from_le(v)
        };
        if valid == 0 {
            uwarn!("mcu", "efuse_read_invalid"; offs = aligned_offset, valid = valid);
            return Err(-2); // Invalid block
        }

        let mut result = [0u8; 16];
        for i in 0..16 {
            result[i] = unsafe { core::ptr::read_volatile(buf_ptr.add(44 + 48 + i)) };
        }
        Ok(result)
    }

    /// Upload EEPROM data to firmware in flash/buffer mode.
    /// Linux: mt7996/mcu.c:3765-3808 mt7996_mcu_set_eeprom_flash()
    ///
    /// Sends the full EEPROM binary (7680 bytes) in 1024-byte pages.
    /// Each page is a separate MCU_WM_UNI_CMD(EFUSE_CTRL) message with
    /// buffer_mode=EE_MODE_BUFFER(1) and format encoding page index/total.
    ///
    /// struct mt7996_mcu_eeprom (8 bytes after uni_header):
    ///   le16 tag;        // UNI_EFUSE_BUFFER_MODE (2)
    ///   le16 len;        // payload length excluding uni_header
    ///   u8 buffer_mode;  // EE_MODE_BUFFER (1)
    ///   u8 format;       // MAX_PAGE_IDX[7:5] | PAGE_IDX[4:2] | EE_FORMAT_WHOLE
    ///   le16 buf_len;    // EEPROM data length in this page
    pub fn mcu_set_eeprom_flash(&self, ring: &mut TxRing, eeprom: &[u8], seq: &mut u8, mut irq: Option<&mut FwIrq>) -> Result<(), i32> {
        let eeprom_size = eeprom.len();
        let total_pages = (eeprom_size + PER_PAGE_SIZE - 1) / PER_PAGE_SIZE; // DIV_ROUND_UP

        udebug!("mcu", "eeprom_flash"; size = eeprom_size as u32, pages = total_pages as u32);

        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_EFUSE_CTRL as u32) | CMD_FIELD_WM;

        for i in 0..total_pages {
            let offset = i * PER_PAGE_SIZE;
            let eep_len = if i == total_pages - 1 && (eeprom_size % PER_PAGE_SIZE) != 0 {
                eeprom_size % PER_PAGE_SIZE // last page: remainder
            } else {
                PER_PAGE_SIZE // full page
            };

            // Message layout: uni_header(4) + eeprom_hdr(8) + eeprom_data(eep_len)
            let msg_len = 4 + 8 + eep_len;

            // format byte: MAX_PAGE_IDX[7:5] | PAGE_IDX[4:2] | EE_FORMAT_WHOLE
            // Linux: FIELD_PREP(GENMASK(7,5), total-1) | FIELD_PREP(GENMASK(4,2), i) | EE_FORMAT_WHOLE
            let format = (((total_pages - 1) as u8) << 5) | ((i as u8) << 2) | EE_FORMAT_WHOLE;

            // Build in a stack buffer: max 4 + 8 + 1024 = 1036 bytes
            let mut data = [0u8; 4 + 8 + PER_PAGE_SIZE];
            let data = &mut data[..msg_len];

            // uni_header: 4 zero bytes (already zero)

            // tag = UNI_EFUSE_BUFFER_MODE (2) at offset 4
            data[4..6].copy_from_slice(&UNI_EFUSE_BUFFER_MODE.to_le_bytes());
            // len = msg_len - 4 (exclude uni_header)
            data[6..8].copy_from_slice(&((msg_len - 4) as u16).to_le_bytes());
            // buffer_mode = EE_MODE_BUFFER (1) at offset 8
            data[8] = EE_MODE_BUFFER;
            // format at offset 9
            data[9] = format;
            // buf_len at offset 10 (le16)
            data[10..12].copy_from_slice(&(eep_len as u16).to_le_bytes());

            // Copy EEPROM data at offset 12
            data[12..12 + eep_len].copy_from_slice(&eeprom[offset..offset + eep_len]);

            self.mcu_send_uni_cmd(ring, cmd, data, true, *seq, irq.as_deref_mut())?;
            *seq = seq.wrapping_add(1);
        }

        udebug!("mcu", "eeprom_flash_done");
        Ok(())
    }

    /// Enable/disable radio for a band
    /// Linux: mt7996/mcu.c:4539-4558 mt7996_mcu_set_radio_en()
    ///
    /// UNI cmd MCU_WM_UNI_CMD(BAND_CONFIG) (0x08), tag=UNI_BAND_CONFIG_RADIO_ENABLE(0).
    /// Layout (12 bytes): {band_idx:u8, rsv[3], tag:le16, len:le16, enable:u8, rsv2[3]}
    /// Note: first 4 bytes are band_idx header (NOT empty uni_header — BAND_CONFIG specific).
    /// Wait: yes
    pub fn mcu_set_radio_en(&self, ring: &mut TxRing, band: u8, enable: bool, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
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
            self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)?;
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
            self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)?;
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

            self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)?;
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
    pub fn mcu_set_rts_thresh(&self, ring: &mut TxRing, band: u8, val: u32, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
    }

    /// Set channel info (switch or RX path)
    /// Linux: mt7996/mcu.c:3696-3762 mt7996_mcu_set_chan_info()
    ///
    /// UNI cmd MCU_WMWA_UNI_CMD(CHANNEL_SWITCH) (0x34) — both WM+WA.
    /// tag: UNI_CHANNEL_SWITCH(0) or UNI_CHANNEL_RX_PATH(1)
    /// Wait: yes
    /// `sec_ch_offset`: secondary channel offset for 40MHz. -1=below, 0=none, +1=above.
    pub fn mcu_set_chan_info(&self, ring: &mut TxRing, band: u8, tag: u16,
                            channel: u8, bw: u8, channel_band: u8,
                            sec_ch_offset: i8,
                            seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
        // 80 bytes: {rsv[4], tag:le16, len:le16=76, fields..., rsv1[53]}
        // Linux struct is 80 bytes total, len = sizeof(req) - 4 = 76
        let mut data = [0u8; 80];

        // tag
        data[4..6].copy_from_slice(&tag.to_le_bytes());
        // len = 76
        data[6..8].copy_from_slice(&76u16.to_le_bytes());

        // control_ch = channel
        data[8] = channel;
        // center_ch: for 40MHz, primary +/- 2
        let center_ch = match (bw, sec_ch_offset) {
            (1, 1) => channel.wrapping_add(2),  // secondary above
            (1, -1) => channel.wrapping_sub(2), // secondary below
            _ => channel,                        // 20MHz: same as control
        };
        data[9] = center_ch;
        // bw
        data[10] = bw;

        // tx_path_num: count of TX antennas
        // Linux: hweight16(phy->chainmask)
        // BPI-R4 MT7996: assumed 2T2R pending eFuse readback confirmation
        data[11] = 2;

        // rx_path: depends on tag
        // For UNI_CHANNEL_RX_PATH: bitmask (rx_chainmask)
        // For UNI_CHANNEL_SWITCH: hweight8(rx_path) = count of bits
        // Linux mcu.c:3731-3753
        // BPI-R4 MT7996: assumed 2T2R pending eFuse readback confirmation
        if tag == UNI_CHANNEL_RX_PATH {
            data[12] = 0x3;  // chainmask bitmask (2 chains)
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
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
    }

    /// Set TX power SKU rate limits
    /// Linux: mt7996/mcu.c:4798-4857 mt7996_mcu_set_txpower_sku()
    ///
    /// Sends per-rate power limits to firmware. Without this, the firmware
    /// may not transmit (no power limits configured → default to zero).
    ///
    /// In Linux, limits come from mt76_get_rate_power_limits() which fills
    /// all rate entries with target_power from EEPROM. We use 127 (max s8)
    /// to impose no host-side limit, letting firmware use EEPROM defaults.
    ///
    /// UNI command MCU_WM_UNI_CMD(TXPOWER) (0x2b) — WM only.
    /// Wait: yes
    pub fn mcu_set_txpower_sku(&self, ring: &mut TxRing, band: u8, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
        // struct tx_power_limit_table_ctrl from mcu.c:4803-4816:
        //   u8 __rsv1[4];           // uni_header (4)
        //   le16 tag;               // UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL (4)
        //   le16 len;               // sizeof(ctrl) + SKU_PATH_NUM - 4 = 501
        //   u8 power_ctrl_id;       // UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL (4)
        //   u8 power_limit_type;    // TX_POWER_LIMIT_TABLE_RATE (0)
        //   u8 band_idx;
        // Total header: 11 bytes, then SKU_PATH_NUM (494) bytes of rate power limits
        const HEADER_SIZE: usize = 11;
        const TOTAL_SIZE: usize = HEADER_SIZE + MT7996_SKU_PATH_NUM; // 505

        let mut data = [0u8; TOTAL_SIZE];

        // tag = UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL (4) — mcu.c:4812
        data[4..6].copy_from_slice(&UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL.to_le_bytes());
        // len = sizeof(ctrl) + SKU_PATH_NUM - 4 = 11 + 494 - 4 = 501 — mcu.c:4813
        data[6..8].copy_from_slice(&((HEADER_SIZE + MT7996_SKU_PATH_NUM - 4) as u16).to_le_bytes());
        // power_ctrl_id = UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL (4) — mcu.c:4814
        data[8] = UNI_TXPOWER_POWER_LIMIT_TABLE_CTRL as u8;
        // power_limit_type = TX_POWER_LIMIT_TABLE_RATE (0) — mcu.c:4815
        data[9] = 0;
        // band_idx — mcu.c:4816
        data[10] = band;

        // Fill rate power limits with 127 (max s8 = +63.5 dBm half-dBm)
        // This imposes no host-side limit — firmware uses EEPROM-based power.
        // Linux: memset(dest, target_power, ...) in mt76_get_rate_power_limits()
        // fills with EEPROM target power (typically ~34 half-dBm for 2.4GHz).
        // Using 127 is safe: firmware caps to its own EEPROM limits.
        for b in &mut data[HEADER_SIZE..] {
            *b = 127;
        }

        // MCU_WM_UNI_CMD(TXPOWER) = __MCU_CMD_FIELD_UNI | 0x2b | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_TXPOWER as u32) | CMD_FIELD_WM;

        udebug!("mcu", "set_txpower_sku"; band = band);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
    }

    /// Enable/disable RX header translation
    /// Linux: mt7996/mcu.c:3387-3420 mt7996_mcu_set_hdr_trans()
    ///
    /// UNI cmd MCU_WM_UNI_CMD(RX_HDR_TRANS) (0x12) — WM only.
    /// Multi-TLV: HDR_TRANS_EN + HDR_TRANS_VLAN + HDR_TRANS_BLACKLIST (if enable)
    /// Wait: yes
    pub fn mcu_set_hdr_trans(&self, ring: &mut TxRing, enable: bool, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        self.mcu_send_uni_cmd(ring, cmd, &data[..total], true, seq, irq)
    }

    /// Add/remove device info (OMAC)
    /// Linux: mt7996/mcu.c:2623-2651 mt7996_mcu_add_dev_info()
    ///
    /// UNI cmd MCU_WMWA_UNI_CMD(DEV_INFO_UPDATE) (0x01) — both WM+WA.
    /// Wait: yes
    pub fn mcu_add_dev_info(&self, ring: &mut TxRing, band: u8, omac_idx: u8,
                            mac_addr: &[u8; 6], enable: bool, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
    }

    /// Add/remove BSS info with all required AP TLVs
    /// Linux: mt7996/mcu.c:1028-1090 mt7996_mcu_bss_basic_tlv()
    ///        mt7996/mcu.c:1216 mt7996_mcu_bss_sec_tlv()
    ///        mt7996/mcu.c:805 mt7996_mcu_bss_rfch_tlv()
    ///        mt7996/mcu.c:834 mt7996_mcu_bss_ra_tlv()
    ///        mt7996/mcu.c:896 mt7996_mcu_bss_bmc_tlv()
    ///        mt7996/mcu.c:916 mt7996_mcu_bss_txcmd_tlv()
    ///        mt7996/mcu.c:1002 mt7996_mcu_bss_ifs_timing_tlv()
    ///
    /// UNI cmd MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE) (0x02) — both WM+WA.
    /// TLVs: BASIC(32) + SEC(8) + RLM(16) + RA(16) + RATE(24) + TXCMD(8) + IFS_TIME(20)
    /// Wait: yes
    pub fn mcu_add_bss_info(&self, ring: &mut TxRing, band: u8, omac_idx: u8,
                            hw_bss_idx: u8, mac_addr: &[u8; 6], enable: bool,
                            channel: u8, bw: u8, sec_ch_offset: i8,
                            seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
        // Layout: uni_header(4) + BASIC(32) + SEC(8) + RLM(16) + RA(16) + RATE(24) + TXCMD(8) + IFS_TIME(20) + MLD(16) = 144
        let mut data = [0u8; 144];

        // uni_header: bss_idx = 0 (first vif) — __mt7996_mcu_alloc_bss_req()
        // In Linux, bss_req_hdr.bss_idx = mvif->idx (VIF software index)

        // === BSS_INFO_BASIC TLV (32 bytes) — mcu.c:1028-1090 ===
        let off = 4;
        data[off..off + 2].copy_from_slice(&UNI_BSS_INFO_BASIC.to_le_bytes());  // tag
        data[off + 2..off + 4].copy_from_slice(&32u16.to_le_bytes());           // len
        data[off + 4] = if enable { 1 } else { 0 };  // active — mcu.c:1082
        data[off + 5] = omac_idx;                      // omac_idx
        // hw_bss_idx MUST match bss_req_hdr.bss_idx (uni_header byte 0).
        // Linux: mt7996_get_hw_bss_idx() = mvif->idx & 0x1f = same as bss_idx.
        data[off + 6] = hw_bss_idx;                    // hw_bss_idx — mcu.c:1087
        data[off + 7] = band;                          // band_idx
        // conn_type = CONNECTION_INFRA_AP — mcu.c:1038
        data[off + 8..off + 12].copy_from_slice(&CONNECTION_INFRA_AP.to_le_bytes());
        // conn_state = !enable — mcu.c:1083 (0 for AP enable)
        data[off + 12] = if enable { 0 } else { 1 };
        // wmm_idx = 0
        // bssid[6] at off+14..off+20: set to our MAC for AP mode
        data[off + 14..off + 20].copy_from_slice(mac_addr);
        // bmc_tx_wlan_idx:le16 at off+20 — Linux: mcu.c:1078 = mvif->sta.wcid.idx
        // MT7996_WTBL_RESERVED - bss_idx (1087 for first BSS)
        let bmc_wlan_idx = MT7996_WTBL_RESERVED - hw_bss_idx as u16;
        data[off + 20..off + 22].copy_from_slice(&bmc_wlan_idx.to_le_bytes());
        // bcn_interval:le16 = 100
        data[off + 22..off + 24].copy_from_slice(&100u16.to_le_bytes());
        // dtim_period = 1
        data[off + 24] = 1;
        // phymode = B|G|GN for 2.4GHz — mt76_connac_mcu.c:1319
        data[off + 25] = PHY_MODE_B | PHY_MODE_G | PHY_MODE_GN;  // 0x0E
        // sta_idx:le16 at off+26 — Linux: mcu.c:1079 = wlan_idx (BMC STA WCID)
        data[off + 26..off + 28].copy_from_slice(&bmc_wlan_idx.to_le_bytes());
        // nonht_basic_phy:le16 at off+28 — mt7996's bss_basic_tlv() does NOT set this
        // (only mt76_connac_mcu.c for older chips sets it to 0x0015)
        // Leave at 0 to match mt7996 behavior.

        // === BSS_INFO_SEC TLV (8 bytes) — mcu.c:1216 ===
        let off = 36;
        data[off..off + 2].copy_from_slice(&UNI_BSS_INFO_SEC.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&8u16.to_le_bytes());
        // cipher = 0 (NONE/open)

        // === BSS_INFO_RLM TLV (16 bytes) — mcu.c:805-832 ===
        let off = 44;
        data[off..off + 2].copy_from_slice(&UNI_BSS_INFO_RLM.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&16u16.to_le_bytes());
        data[off + 4] = channel;   // control_channel
        // center_chan: same for 20MHz, +/-2 for 40MHz — mcu.c:812
        data[off + 5] = if bw >= 1 {
            (channel as i8 + sec_ch_offset * 2) as u8
        } else {
            channel
        };
        // center_chan2 = 0
        data[off + 7] = bw;        // bw — CMD_CBW_20MHZ(0) or CMD_CBW_40MHZ(1)
        // tx_streams = hweight8(antenna_mask) — mcu.c:823
        // BPI-R4 MT7996: assumed 2T2R pending eFuse readback confirmation
        let nstreams: u8 = 2;
        data[off + 8] = nstreams;  // tx_streams
        data[off + 9] = nstreams;  // rx_streams
        // ht_op_info = 0, sco = 0
        data[off + 12] = 1;        // band = 1 (2GHz) — mcu.c:807-810 rlm_ch_band

        // === BSS_INFO_RA TLV (16 bytes) — mcu.c:834-844 ===
        let off = 60;
        data[off..off + 2].copy_from_slice(&UNI_BSS_INFO_RA.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&16u16.to_le_bytes());
        data[off + 4] = 1;         // short_preamble = true — mcu.c:843

        // === BSS_INFO_RATE TLV (24 bytes) — mcu.c:896-914 ===
        let off = 76;
        data[off..off + 2].copy_from_slice(&UNI_BSS_INFO_RATE.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&24u16.to_le_bytes());
        // __rsv1[4] at off+4..off+8
        // bc_trans:le16 at off+8, mc_trans:le16 at off+10
        data[off + 12] = 1;        // short_preamble = true (2GHz) — mcu.c:911
        data[off + 13] = MT7996_BASIC_RATES_TBL;  // bc_fixed_rate — mcu.c:912
        data[off + 14] = MT7996_BASIC_RATES_TBL;  // mc_fixed_rate — mcu.c:913

        // === BSS_INFO_TXCMD TLV (8 bytes) — mcu.c:916-926 ===
        let off = 100;
        data[off..off + 2].copy_from_slice(&UNI_BSS_INFO_TXCMD.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&8u16.to_le_bytes());
        data[off + 4] = 1;         // txcmd_mode = true — mcu.c:925

        // === BSS_INFO_IFS_TIME TLV (20 bytes) — mcu.c:1002-1026 ===
        let off = 108;
        data[off..off + 2].copy_from_slice(&UNI_BSS_INFO_IFS_TIME.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&20u16.to_le_bytes());
        data[off + 4] = 1;         // slot_valid
        data[off + 5] = 1;         // sifs_valid
        data[off + 6] = 1;         // rifs_valid
        data[off + 7] = 1;         // eifs_valid
        data[off + 8..off + 10].copy_from_slice(&9u16.to_le_bytes());    // slot_time (short slot)
        data[off + 10..off + 12].copy_from_slice(&10u16.to_le_bytes());  // sifs_time — mcu.c:1018
        data[off + 12..off + 14].copy_from_slice(&2u16.to_le_bytes());   // rifs_time — mcu.c:1019
        data[off + 14..off + 16].copy_from_slice(&78u16.to_le_bytes());  // eifs_time (2GHz) — mcu.c:1020
        data[off + 16] = 1;        // eifs_cck_valid (2GHz) — mcu.c:1022
        // rsv = 0
        data[off + 18..off + 20].copy_from_slice(&314u16.to_le_bytes()); // eifs_cck_time — mcu.c:1024

        // === BSS_INFO_MLD TLV (16 bytes) — mcu.c:929-951 ===
        // MANDATORY: "this tag is necessary no matter if the vif is MLD" (mcu.c:1161)
        let off = 128;
        data[off..off + 2].copy_from_slice(&UNI_BSS_INFO_MLD.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&16u16.to_le_bytes());
        data[off + 4] = 0xff;         // group_mld_id = 0xff (not MLD) — mcu.c:948
        data[off + 5] = hw_bss_idx;   // own_mld_id = BSS index — mcu.c:940
        // mac_addr[6] at off+6..off+12: zeros for non-MLD
        data[off + 12] = 0xff;        // remap_idx = 0xff (not MLD) — mcu.c:949
        data[off + 13] = 0;           // link_id = 0 (single link) — mcu.c:941
        // __rsv[2] = 0

        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BSS_INFO_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;

        udebug!("mcu", "add_bss_info"; band = band, omac = omac_idx, bss = hw_bss_idx, enable = enable as u8);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
    }

    /// Add/update STA record
    /// Linux: mt7996/mcu.c:2438-2477 mt7996_mcu_add_sta()
    ///        mt76_connac_mcu.c:388-415 mt76_connac_mcu_sta_basic_tlv()
    ///
    /// UNI cmd MCU_WMWA_UNI_CMD(STA_REC_UPDATE) (0x03) — both WM+WA.
    /// Layout: sta_req_hdr(8) + STA_REC_BASIC(20) + STA_REC_HDR_TRANS(8) + STA_REC_TX_PROC(8) = 44 bytes
    /// Wait: yes
    ///
    /// Linux sends 3 TLVs for the BMC STA (link_sta=NULL path):
    ///   1. STA_REC_BASIC — connection type, state, extra_info
    ///   2. STA_REC_HDR_TRANS — header translation config (mcu.c:1908-1937)
    ///   3. STA_REC_TX_PROC — TX processing flags (mcu.c:1884-1893)
    pub fn mcu_add_sta(&self, ring: &mut TxRing, bss_idx: u8, wlan_idx: u16,
                       omac_idx: u8, conn_state: u8, mac_addr: &[u8; 6],
                       newly: bool, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
        let mut data = [0u8; 44];

        // === sta_req_hdr (8 bytes) — mt76_connac_mcu.h ===
        // bss_idx
        data[0] = bss_idx;
        // wlan_idx_lo (low 8 bits)
        data[1] = wlan_idx as u8;
        // tlv_num = 3 (BASIC + HDR_TRANS + TX_PROC)
        data[2..4].copy_from_slice(&3u16.to_le_bytes());
        // is_tlv_append = 1
        data[4] = 1;
        // muar_idx: Linux mt76_connac_mcu.c:280-286
        // For BMC STA (!wcid->sta && !wcid->sta_disabled), muar_idx = 0x0e.
        // This tells firmware to use the wildcard MUAR entry, not a specific OMAC.
        data[5] = 0x0e;
        // wlan_idx_hi (high 8 bits)
        data[6] = (wlan_idx >> 8) as u8;
        // rsv = 0

        // === STA_REC_BASIC TLV (20 bytes) — mt76_connac_mcu.c:388-415 ===
        // This is the broadcast/multicast STA for AP mode (link_sta=NULL path).
        let off = 8;
        // tag = STA_REC_BASIC (0)
        data[off..off + 2].copy_from_slice(&STA_REC_BASIC.to_le_bytes());
        // len = 20
        data[off + 2..off + 4].copy_from_slice(&20u16.to_le_bytes());
        // conn_type:le32 = CONNECTION_INFRA_BC — mt76_connac_mcu.c:396
        // Broadcast/multicast STA for AP mode (link_sta==NULL → INFRA_BC)
        data[off + 4..off + 8].copy_from_slice(&CONNECTION_INFRA_BC.to_le_bytes());
        // conn_state
        data[off + 8] = conn_state;
        // qos = 0 (broadcast STA has no WME) — mt76_connac_mcu.c:411
        // aid:le16 = 0
        // peer_addr[6] = ff:ff:ff:ff:ff:ff (broadcast) — mt76_connac_mcu.c:397
        // AP mode + link_sta==NULL → eth_broadcast_addr(basic->peer_addr)
        data[off + 12..off + 18].copy_from_slice(&[0xff; 6]);
        // extra_info:le16
        let mut extra = EXTRA_INFO_VER;
        if newly {
            extra |= EXTRA_INFO_NEW;
        }
        data[off + 18..off + 20].copy_from_slice(&extra.to_le_bytes());

        // === STA_REC_HDR_TRANS TLV (8 bytes) — mcu.c:1908-1937 ===
        // For AP mode: from_ds=1, to_ds=0
        // dis_rx_hdr_tran=1 (new WCID, MT_WCID_FLAG_HDR_TRANS not yet set)
        let off = 28;
        data[off..off + 2].copy_from_slice(&STA_REC_HDR_TRANS.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&8u16.to_le_bytes());
        data[off + 4] = 1;  // from_ds = true (AP mode) — mcu.c:1921
        data[off + 5] = 0;  // to_ds = false (AP mode)
        data[off + 6] = 1;  // dis_rx_hdr_tran = true (no HDR_TRANS flag) — mcu.c:1916
        // mesh = 0

        // === STA_REC_TX_PROC TLV (8 bytes) — mcu.c:1884-1893 ===
        // flag = 0 (no special TX processing flags)
        let off = 36;
        data[off..off + 2].copy_from_slice(&STA_REC_TX_PROC.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&8u16.to_le_bytes());
        // flag:le32 = 0 — mcu.c:1892

        // MCU_WMWA_UNI_CMD(STA_REC_UPDATE) = __MCU_CMD_FIELD_UNI | 0x03 | WM | WA
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;

        udebug!("mcu", "add_sta"; bss = bss_idx, wlan = wlan_idx, state = conn_state);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
    }

    /// Add/update a client STA record in firmware.
    /// Like mcu_add_sta() but for real client STAs (not BMC):
    ///   - conn_type = CONNECTION_INFRA_STA (not INFRA_BC)
    ///   - Actual client MAC address (not broadcast)
    ///   - AID set from association
    ///   - QoS enabled (WME)
    ///   - muar_idx = omac_idx (maps to specific OMAC, not wildcard)
    ///
    /// Source: Linux mt76_connac_mcu.c:388-415 (link_sta != NULL path)
    ///         mt7996/mcu.c:1391-1407 mt7996_mcu_sta_ht_tlv()
    pub fn mcu_add_client_sta(&self, ring: &mut TxRing, bss_idx: u8, wlan_idx: u16,
                              omac_idx: u8, conn_state: u8, mac_addr: &[u8; 6],
                              aid: u16, newly: bool, seq: u8,
                              irq: Option<&mut FwIrq>, wait: bool,
                              ht_cap: u16, ht_param: u8, sta_flags: u16,
                              ) -> Result<(), i32> {
        let has_ht = sta_flags & wifi80211::types::STA_FLAG_HT != 0;
        // Buffer: sta_req_hdr(8) + BASIC(20) + HDR_TRANS(8) + TX_PROC(8) + HT(12) = 56
        let tlv_count: u16 = if has_ht { 4 } else { 3 };
        let total_size: usize = if has_ht { 56 } else { 44 };
        let mut data = [0u8; 56];

        // === sta_req_hdr (8 bytes) ===
        data[0] = bss_idx;
        data[1] = wlan_idx as u8;
        data[2..4].copy_from_slice(&tlv_count.to_le_bytes());
        data[4] = 1; // is_tlv_append
        // muar_idx: for real STAs with link_sta, use omac_idx
        // Linux: mt76_connac_mcu.c:280 — if (wcid->sta) muar_idx = mvif->omac_idx
        data[5] = omac_idx;
        data[6] = (wlan_idx >> 8) as u8;

        // === STA_REC_BASIC TLV (20 bytes) ===
        let off = 8;
        data[off..off + 2].copy_from_slice(&STA_REC_BASIC.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&20u16.to_le_bytes());
        // conn_type = CONNECTION_INFRA_STA (client STA, not BMC)
        data[off + 4..off + 8].copy_from_slice(&CONNECTION_INFRA_STA.to_le_bytes());
        data[off + 8] = conn_state;
        data[off + 9] = 1; // qos = 1 (WME enabled for client STAs)
        data[off + 10..off + 12].copy_from_slice(&aid.to_le_bytes());
        data[off + 12..off + 18].copy_from_slice(mac_addr);
        let mut extra = EXTRA_INFO_VER;
        if newly { extra |= EXTRA_INFO_NEW; }
        data[off + 18..off + 20].copy_from_slice(&extra.to_le_bytes());

        // === STA_REC_HDR_TRANS TLV (8 bytes) ===
        let off = 28;
        data[off..off + 2].copy_from_slice(&STA_REC_HDR_TRANS.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&8u16.to_le_bytes());
        data[off + 4] = 1; // from_ds = true (AP mode)
        data[off + 5] = 0; // to_ds = false
        data[off + 6] = 0; // dis_rx_hdr_tran = false → enable header translation

        // === STA_REC_TX_PROC TLV (8 bytes) ===
        let off = 36;
        data[off..off + 2].copy_from_slice(&STA_REC_TX_PROC.to_le_bytes());
        data[off + 2..off + 4].copy_from_slice(&8u16.to_le_bytes());

        // === STA_REC_HT TLV (12 bytes) — mt7996/mcu.c:1391-1407 ===
        // struct sta_rec_ht_uni: tag(2) + len(2) + ht_cap(2) + ht_cap_ext(2) + ampdu_param(1) + rsv(3)
        // Without this, firmware uses legacy (non-HT) mode — no aggregation, PLE stall.
        if has_ht {
            let off = 44;
            data[off..off + 2].copy_from_slice(&STA_REC_HT.to_le_bytes());  // tag = 9
            data[off + 2..off + 4].copy_from_slice(&12u16.to_le_bytes());   // len = 12
            data[off + 4..off + 6].copy_from_slice(&ht_cap.to_le_bytes());  // ht_cap
            // ht_cap_ext: leave 0 (we don't parse extended HT caps from assoc req)
            data[off + 8] = ht_param;                                       // ampdu_param
            // rsv[3] = 0
        }

        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;

        udebug!("mcu", "add_client_sta"; bss = bss_idx, wlan = wlan_idx, state = conn_state,
                aid = aid, ht = has_ht as u8, ht_cap = ht_cap);
        self.mcu_send_uni_cmd(ring, cmd, &data[..total_size], wait, seq, irq)
    }

    /// Send STA_REC_RA TLV for rate adaptation.
    /// Source: Linux mt7996/mcu.c:2169-2274 mt7996_mcu_sta_rate_ctrl_tlv()
    ///
    /// Must be sent AFTER mcu_add_client_sta so firmware has the STA record.
    /// Sets auto_rate=true so firmware does dynamic rate adaptation.
    ///
    /// Layout: sta_req_hdr(8) + sta_rec_ra_uni(58) = 66 bytes
    pub fn mcu_sta_rate_ctrl(&self, ring: &mut TxRing, bss_idx: u8, wlan_idx: u16,
                             omac_idx: u8, channel: u8, bw: u8, ht_cap: u16, ht_param: u8,
                             sta_flags: u16, seq: u8) -> Result<(), i32> {
        let has_ht = sta_flags & wifi80211::types::STA_FLAG_HT != 0;
        let is_5g = channel >= 36;
        let mut data = [0u8; 66]; // sta_req_hdr(8) + sta_rec_ra_uni(58)

        // === sta_req_hdr (8 bytes) ===
        data[0] = bss_idx;
        data[1] = wlan_idx as u8;
        data[2..4].copy_from_slice(&1u16.to_le_bytes()); // tlv_num = 1
        data[4] = 1; // is_tlv_append
        data[5] = omac_idx;
        data[6] = (wlan_idx >> 8) as u8;

        // === STA_REC_RA TLV (58 bytes) — mt7996/mcu.c:2169-2274 ===
        // struct sta_rec_ra_uni: see mt7996/mcu.h
        let off = 8;
        data[off..off + 2].copy_from_slice(&STA_REC_RA.to_le_bytes());     // tag = 1
        data[off + 2..off + 4].copy_from_slice(&58u16.to_le_bytes());      // len = 58

        data[off + 4] = 1; // valid = true
        data[off + 5] = 1; // auto_rate = true

        // phy_mode — mt76_connac_get_phy_mode() in mt76_connac_mcu.c:922-965
        let phy_mode: u8 = if is_5g {
            if has_ht { PHY_MODE_A | PHY_MODE_AN } else { PHY_MODE_A }
        } else {
            if has_ht { PHY_MODE_B | PHY_MODE_G | PHY_MODE_GN } else { PHY_MODE_B | PHY_MODE_G }
        };
        data[off + 6] = phy_mode;   // phy_mode
        data[off + 7] = channel;    // channel

        data[off + 8] = bw;          // bw — CMD_CBW_20MHZ(0) or CMD_CBW_40MHZ(1)
        data[off + 9] = if is_5g { 1 } else { 0 }; // disable_cck (5GHz = no CCK)

        // ht_mcs32, ht_gf: leave 0 (not supported)

        // ht_mcs[4] at offset 12 — MCS bitmask (1 byte per spatial stream)
        // MCS 0-7 = 1 spatial stream. Linux: mt7996_mcu_set_sta_ht_mcs()
        // copies from ht_cap.mcs.rx_mask[0..3]
        if has_ht {
            data[off + 12] = 0xFF;  // MCS 0-7 supported (1 stream)
            // ht_mcs[1..3] = 0 for 1-stream (conservative — we don't parse MCS set from assoc)
        }

        // mmps_mode at offset 16: 3 = disabled (static SMPS)
        // Linux: mt7996_mcu_get_mmps_mode() — returns 3 if SMPS_OFF
        data[off + 16] = 3; // SMPS disabled

        // af (AMPDU factor) at offset 18 — from HT capabilities AMPDU params
        // Linux: max of HT and VHT factors. We only have HT.
        // AMPDU factor = bits[1:0] of ht_param
        data[off + 18] = ht_param & 0x03; // af = ampdu_factor

        // rate_len at offset 20 — number of legacy rates
        data[off + 20] = if is_5g { 8 } else { 12 }; // OFDM=8, CCK+OFDM=4+8=12

        // supp_mode at offset 21 — same as phy_mode
        data[off + 21] = phy_mode;

        // supp_cck_rate at offset 22 — 4 CCK rates (1, 2, 5.5, 11 Mbps)
        data[off + 22] = if is_5g { 0 } else { 0x0F };

        // supp_ofdm_rate at offset 23 — 8 OFDM rates (6-54 Mbps)
        data[off + 23] = 0xFF;

        // supp_ht_mcs at offset 24 (le32) — same as ht_mcs
        if has_ht {
            data[off + 24..off + 28].copy_from_slice(&0x000000FFu32.to_le_bytes());
        }

        // supp_vht_mcs[4] at offset 28 — leave 0 (no VHT for now)

        // op_mode at offset 36: leave 0
        // op_vht_chan_width at offset 37: leave 0

        // sta_cap at offset 40 (le32) — capability flags for firmware rate control
        // Linux: mt7996_mcu_sta_rate_ctrl_tlv() lines 2237-2274
        let mut sta_cap_val: u32 = STA_CAP_WMM; // always WMM
        if has_ht {
            sta_cap_val |= STA_CAP_HT;
            // Short GI 20MHz: ht_cap bit 5
            if ht_cap & 0x0020 != 0 {
                sta_cap_val |= STA_CAP_SGI_20;
            }
            // Short GI 40MHz: ht_cap bit 6
            if ht_cap & 0x0040 != 0 {
                sta_cap_val |= STA_CAP_SGI_40;
            }
            // TX STBC: ht_cap bit 7
            if ht_cap & 0x0080 != 0 {
                sta_cap_val |= STA_CAP_TX_STBC;
            }
            // RX STBC: ht_cap bits [9:8]
            if ht_cap & 0x0300 != 0 {
                sta_cap_val |= STA_CAP_RX_STBC;
            }
            // LDPC: ht_cap bit 0
            if ht_cap & 0x0001 != 0 {
                sta_cap_val |= STA_CAP_LDPC;
            }
        }
        data[off + 40..off + 44].copy_from_slice(&sta_cap_val.to_le_bytes());

        // phy sub-struct at offset 44 (10 bytes): type, flag, stbc, sgi, bw, ldpc, mcs, nss, he_ltf, rsv[3]
        // Leave all 0 for auto-rate mode — firmware fills in.

        // rx_rcpi[4] at offset 54 — initialize to INIT_RCPI (180)
        // Linux: mt76_connac_mcu.c uses INIT_RCPI = 180
        data[off + 54] = INIT_RCPI;
        data[off + 55] = INIT_RCPI;
        data[off + 56] = INIT_RCPI;
        data[off + 57] = INIT_RCPI;

        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;

        udebug!("mcu", "sta_rate_ctrl"; wlan = wlan_idx, phy = phy_mode, sta_cap = sta_cap_val);
        self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)
    }

    /// Send STA_REC_BA TLV to notify firmware about a BA session start/stop.
    /// Source: Linux mt7996/mcu.c:1191-1218 mt7996_mcu_sta_ba()
    ///
    /// Layout: sta_req_hdr(8) + sta_rec_ba_uni(16) = 24 bytes
    /// sta_rec_ba_uni: tag(2) + len(2) + tid(1) + ba_type(1) + amsdu(1) +
    ///                 ba_en(1) + ssn(2) + winsize(2) + ba_rdd_rro(1) + rsv[3]
    ///
    /// Command: MCU_WMWA_UNI_CMD(STA_REC_UPDATE) — same as add_sta
    pub fn mcu_sta_ba(&self, ring: &mut TxRing, bss_idx: u8, wlan_idx: u16,
                      omac_idx: u8, tid: u8, ssn: u16, win_size: u16,
                      enable: bool, seq: u8) -> Result<(), i32> {
        let mut data = [0u8; 24]; // sta_req_hdr(8) + sta_rec_ba_uni(16)

        // === sta_req_hdr (8 bytes) ===
        data[0] = bss_idx;
        data[1] = wlan_idx as u8;
        data[2..4].copy_from_slice(&1u16.to_le_bytes()); // tlv_num = 1
        data[4] = 1; // is_tlv_append
        data[5] = omac_idx;
        data[6] = (wlan_idx >> 8) as u8;

        // === sta_rec_ba_uni TLV (16 bytes) — mt7996/mcu.h:486-497 ===
        let off = 8;
        data[off..off + 2].copy_from_slice(&STA_REC_BA.to_le_bytes()); // tag = 6
        data[off + 2..off + 4].copy_from_slice(&16u16.to_le_bytes()); // len = 16
        data[off + 4] = tid;                                          // tid
        // ba_type: 2 = MT_BA_TYPE_RECIPIENT (we are AP receiving ADDBA from STA)
        // Source: mt76_connac_mcu.h:996-1000
        data[off + 5] = 2;                                            // ba_type = RECIPIENT
        data[off + 6] = 0;                                            // amsdu = 0
        data[off + 7] = if enable { 1u8 << tid } else { 0 };          // ba_en = bitmap
        data[off + 8..off + 10].copy_from_slice(&ssn.to_le_bytes());  // ssn
        data[off + 10..off + 12].copy_from_slice(&win_size.to_le_bytes()); // winsize
        data[off + 12] = 0;                                           // ba_rdd_rro = 0 (no HW RRO)
        // rsv[3] = 0

        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_STA_REC_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;

        udebug!("mcu", "sta_ba"; wlan = wlan_idx, tid = tid, ssn = ssn, win = win_size, en = enable as u8);
        self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)
    }

    /// Assign WCID to BSS scheduling group (VOW DRR control)
    /// Linux: mt7996/mcu.c:2343-2367 mt7996_mcu_add_group()
    ///
    /// Called after add_sta() to assign the WCID to the correct BSS group
    /// for airtime fairness scheduling.
    ///
    /// Command: MCU_WM_UNI_CMD(VOW) = CMD_FIELD_UNI | 0x37 | CMD_FIELD_WM
    /// Wait: yes
    pub fn mcu_add_group(&self, ring: &mut TxRing, bss_idx: u8, wlan_idx: u16,
                         seq: u8, irq: Option<&mut FwIrq>, wait: bool) -> Result<(), i32> {
        // Layout: uni_header(4) + TLV(20) = 24 bytes
        // Linux struct: __rsv1[4] + tag(2) + len(2) + wlan_idx(2) + __rsv2[2] + action(4) + val(4) + __rsv3[8]
        let mut data = [0u8; 24];

        // uni_header: bss_idx (not actually used by VOW, but kept for consistency)

        // TLV: UNI_VOW_DRR_CTRL
        let off = 4;
        data[off..off + 2].copy_from_slice(&UNI_VOW_DRR_CTRL.to_le_bytes());   // tag = 0
        data[off + 2..off + 4].copy_from_slice(&20u16.to_le_bytes());           // len = 20 (sizeof - 4)
        data[off + 4..off + 6].copy_from_slice(&wlan_idx.to_le_bytes());        // wlan_idx
        // __rsv2[2] = 0
        // action = MT_STA_BSS_GROUP (1) — mcu.c:2346,2360
        data[off + 8..off + 12].copy_from_slice(&1u32.to_le_bytes());
        // val = bss_idx % 16 — mcu.c:2361
        data[off + 12..off + 16].copy_from_slice(&((bss_idx as u32) % 16).to_le_bytes());
        // __rsv3[8] = 0

        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_VOW as u32) | CMD_FIELD_WM;

        udebug!("mcu", "add_group"; wlan = wlan_idx, bss = bss_idx);
        self.mcu_send_uni_cmd(ring, cmd, &data, wait, seq, irq)
    }

    /// Initialize transmit beamforming subsystem
    /// Linux: mt7996/init.c:640-658 mt7996_txbf_init()
    ///        mt7996/mcu.c:4185-4235 mt7996_mcu_set_txbf()
    ///
    /// Sends 3 MCU commands via MCU_WM_UNI_CMD(BF) (0x33):
    ///   1. BF_MOD_EN_CTRL (tag=20) — enable BF for all bands
    ///   2. BF_SOUNDING_ON (tag=1) — set sounding mode to BF_PROCESSING(4)
    ///   3. BF_HW_EN_UPDATE (tag=17) — enable explicit beamforming
    ///
    /// Called from init_work() after mac_init(). Required before TX works.
    pub fn mcu_txbf_init(&self, ring: &mut TxRing, seq: &mut u8, mut irq: Option<&mut FwIrq>) -> Result<(), i32> {
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BF as u32) | CMD_FIELD_WM;

        // 1. BF_MOD_EN_CTRL (tag=20, len=16) — init.c:645-654
        // MT7996 tri-band: bf_num=3, bf_bitmap=0x07
        {
            let mut data = [0u8; 20]; // uni_header(4) + TLV(16)
            data[4..6].copy_from_slice(&20u16.to_le_bytes()); // tag = BF_MOD_EN_CTRL
            data[6..8].copy_from_slice(&16u16.to_le_bytes()); // len
            data[8] = 3;    // bf_num = 3 (tri-band)
            data[9] = 0x07; // bf_bitmap = GENMASK(2,0)
            // bf_sel[8] + rsv[2] = 0
            self.mcu_send_uni_cmd(ring, cmd, &data, true, *seq, irq.as_deref_mut())?;
            *seq = seq.wrapping_add(1);
        }

        // 2. BF_SOUNDING_ON (tag=1, len=20) — init.c:656
        {
            let mut data = [0u8; 24]; // uni_header(4) + TLV(20)
            data[4..6].copy_from_slice(&1u16.to_le_bytes()); // tag = BF_SOUNDING_ON
            data[6..8].copy_from_slice(&20u16.to_le_bytes()); // len
            data[8] = 4; // snd_mode = BF_PROCESSING
            // sta_num, rsv, wlan_id[4], snd_period = 0
            self.mcu_send_uni_cmd(ring, cmd, &data, true, *seq, irq.as_deref_mut())?;
            *seq = seq.wrapping_add(1);
        }

        // 3. BF_HW_EN_UPDATE (tag=17, len=8) — init.c:657
        {
            let mut data = [0u8; 12]; // uni_header(4) + TLV(8)
            data[4..6].copy_from_slice(&17u16.to_le_bytes()); // tag = BF_HW_EN_UPDATE
            data[6..8].copy_from_slice(&8u16.to_le_bytes());  // len
            data[8] = 1; // ebf = true (explicit BF always enabled)
            data[9] = 0; // ibf = false (implicit BF off by default)
            self.mcu_send_uni_cmd(ring, cmd, &data, true, *seq, irq)?;
            *seq = seq.wrapping_add(1);
        }

        udebug!("mcu", "txbf_init_ok");
        Ok(())
    }

    /// Send EDCA/WMM queue parameters for AP mode
    /// Linux: mt7996/mcu.c:3423-3481 mt7996_mcu_set_tx()
    ///
    /// Called on BSS_CHANGED_BEACON_ENABLED to configure per-AC queue parameters.
    /// Sends 4 EDCA TLVs (one per AC) with default AP EDCA values.
    /// Comment from Linux main.c:883: "ensure that enable txcmd_mode after bss_info"
    ///
    /// Command: MCU_WM_UNI_CMD(EDCA_UPDATE) = CMD_FIELD_UNI | 0x04 | CMD_FIELD_WM
    /// Wait: yes
    pub fn mcu_set_edca(&self, ring: &mut TxRing, bss_idx: u8, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
        // Layout: hdr(4) + 4 × edca_tlv(12) = 52 bytes
        // struct edca from mcu.h:284-295:
        //   tag:le16, len:le16, queue:u8, set:u8, cw_min:u8, cw_max:u8,
        //   txop:le16, aifs:u8, __rsv:u8
        let mut data = [0u8; 52];

        // Header: bss_idx + 3 bytes padding — mcu.c:3444-3447
        data[0] = bss_idx;

        // Default AP EDCA values (802.11-2020, OFDM, bss_notify=true)
        // cw_min/cw_max values are fls() of the CW values — mcu.c:3466-3474
        //   AC_VO: aifs=2, cw_min=fls(3)=2,  cw_max=fls(7)=3,    txop=47
        //   AC_VI: aifs=2, cw_min=fls(7)=3,  cw_max=fls(15)=4,   txop=94
        //   AC_BE: aifs=3, cw_min=fls(15)=4, cw_max=fls(1023)=10, txop=0
        //   AC_BK: aifs=7, cw_min=fls(15)=4, cw_max=fls(1023)=10, txop=0
        // (queue, cw_min, cw_max, txop, aifs)
        const EDCA_PARAMS: [(u8, u8, u8, u16, u8); 4] = [
            (0, 2, 3, 47, 2),   // AC_VO
            (1, 3, 4, 94, 2),   // AC_VI
            (2, 4, 10, 0, 3),   // AC_BE
            (3, 4, 10, 0, 7),   // AC_BK
        ];

        const WMM_PARAM_SET: u8 = 0x0F; // WMM_AIFS_SET|CW_MIN_SET|CW_MAX_SET|TXOP_SET

        for (i, &(queue, cw_min, cw_max, txop, aifs)) in EDCA_PARAMS.iter().enumerate() {
            let off = 4 + i * 12;
            // tag = MCU_EDCA_AC_PARAM (0) — mcu.c:3457
            data[off..off + 2].copy_from_slice(&0u16.to_le_bytes());
            // len = 12
            data[off + 2..off + 4].copy_from_slice(&12u16.to_le_bytes());
            data[off + 4] = queue;        // queue — mcu.c:3460
            data[off + 5] = WMM_PARAM_SET; // set — mcu.c:3459
            data[off + 6] = cw_min;        // cw_min — mcu.c:3466
            data[off + 7] = cw_max;        // cw_max — mcu.c:3471
            data[off + 8..off + 10].copy_from_slice(&txop.to_le_bytes()); // txop — mcu.c:3462
            data[off + 10] = aifs;         // aifs — mcu.c:3461
            // rsv = 0
        }

        // MCU_WM_UNI_CMD(EDCA_UPDATE) = CMD_FIELD_UNI | 0x04 | CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_EDCA_UPDATE as u32) | CMD_FIELD_WM;

        udebug!("mcu", "set_edca"; bss = bss_idx);
        self.mcu_send_uni_cmd(ring, cmd, &data, true, seq, irq)
    }

    /// Upload beacon template to MCU
    /// Linux: mt7996/mcu.c:2766 mt7996_mcu_add_beacon() + mcu.c:2732 mt7996_mcu_beacon_cont()
    ///
    /// Builds BSS_INFO_UPDATE with UNI_BSS_INFO_BCN_CONTENT TLV containing:
    ///   - bcn_content_tlv header (14 bytes)
    ///   - Hardware TXD (32 bytes)
    ///   - 802.11 beacon frame (65 bytes)
    ///
    /// Uses MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE) (0x02) — same as mcu_add_bss_info.
    /// Wait: yes
    /// Upload beacon template to firmware.
    ///
    /// `beacon_frame`: pre-built 802.11 beacon frame from ApManager::beacon().
    /// The frame includes dynamic TIM bitmap, ERP, and HT protection IEs.
    /// Maximum beacon frame size: 256 bytes.
    ///
    /// Linux: mt7996/mcu.c:2766 mt7996_mcu_add_beacon()
    pub fn mcu_set_beacon(&self, ring: &mut TxRing, band: u8, omac_idx: u8,
                           beacon_frame: &[u8], enable: bool,
                           seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
        const MAX_BEACON: usize = 256;
        let bcn_len = beacon_frame.len().min(MAX_BEACON);

        // TLV data after uni_header:
        //   bcn_content header (14) + TXD (32) + beacon (bcn_len)
        let tlv_body_len = 14 + MT_TXD_SIZE + bcn_len;
        let tlv_aligned = (tlv_body_len + 3) & !3;
        let data_len = 4 + tlv_aligned;

        // Stack buffer — max 256 + 46 + 4 = 306, round up
        let mut data = [0u8; 4 + 14 + MT_TXD_SIZE + MAX_BEACON + 4]; // 310
        if data_len > data.len() {
            return Err(-1);
        }

        // uni_header = 0 (4 bytes)

        // === bcn_content_tlv header (14 bytes) — mcu.c:2732 mt7996_mcu_beacon_cont() ===
        let off = 4usize;
        // tag = UNI_BSS_INFO_BCN_CONTENT (7)
        data[off..off + 2].copy_from_slice(&UNI_BSS_INFO_BCN_CONTENT.to_le_bytes());
        // len = ALIGN(14 + 32 + beacon_len, 4)
        data[off + 2..off + 4].copy_from_slice(&(tlv_aligned as u16).to_le_bytes());
        // tim_ie_pos — offset of TIM IE tag from start of beacon frame.
        // Find TIM IE (tag=5) in the beacon frame body.
        let tim_pos = find_ie_offset(beacon_frame, 5).unwrap_or(59) as u16;
        data[off + 4..off + 6].copy_from_slice(&tim_pos.to_le_bytes());
        // csa_ie_pos = 0
        // bcc_ie_pos = 0
        // enable
        data[off + 10] = if enable { 1 } else { 0 };
        // type = 0
        // pkt_len = TXD + beacon
        data[off + 12..off + 14].copy_from_slice(&((MT_TXD_SIZE + bcn_len) as u16).to_le_bytes());

        // === TXD (32 bytes = 8 × u32 LE) — mac.c:892 mt7996_mac_write_txwi() ===
        let txd_off = off + 14;

        // txd[0]: TX_BYTES | PKT_FMT | Q_IDX
        let txd0 = ((MT_TXD_SIZE + bcn_len) as u32)
            | ((MT_TX_TYPE_FW as u32) << 23)
            | ((MT_LMAC_BCN0 as u32) << 25);
        data[txd_off..txd_off + 4].copy_from_slice(&txd0.to_le_bytes());

        // txd[1]: WLAN_IDX(0) | HDR_FORMAT(802.11) | HDR_INFO(12) | OWN_MAC | FIXED_RATE
        let txd1 = ((MT_HDR_FORMAT_802_11 as u32) << 14)
            | (12u32 << 16)
            | ((omac_idx as u32) << 25)
            | (1u32 << 31);
        data[txd_off + 4..txd_off + 8].copy_from_slice(&txd1.to_le_bytes());

        // txd[2]: FRAME_TYPE(mgmt) | SUB_TYPE(beacon=8)
        let txd2 = 8u32;
        data[txd_off + 8..txd_off + 12].copy_from_slice(&txd2.to_le_bytes());

        // txd[3]: NO_ACK | BCM | REM_TX_COUNT(31) | BA_DISABLE
        let txd3 = 1u32 | (1u32 << 4) | (0x1Fu32 << 11) | (1u32 << 28);
        data[txd_off + 12..txd_off + 16].copy_from_slice(&txd3.to_le_bytes());

        // txd[4]: 0 (no PN / encryption)
        // txd[5]: 0 (no PID tracking)

        // txd[6]: DAS | DIS_MAT | MSDU_CNT(1) | TX_RATE(beacon_rates_tbl) | FIXED_BW | VTA
        let txd6 = (1u32 << 2)
            | (1u32 << 3)
            | (1u32 << 4)
            | ((MT7996_BEACON_RATES_TBL as u32) << 16)
            | (1u32 << 25)
            | (1u32 << 28);
        data[txd_off + 24..txd_off + 28].copy_from_slice(&txd6.to_le_bytes());

        // txd[7]: 0

        // === Copy pre-built beacon frame ===
        let bcn_off = txd_off + MT_TXD_SIZE;
        data[bcn_off..bcn_off + bcn_len].copy_from_slice(&beacon_frame[..bcn_len]);
        // seq_ctrl = 0 (MCU manages sequence)
        data[bcn_off + 22] = 0;
        data[bcn_off + 23] = 0;

        // MCU_WMWA_UNI_CMD(BSS_INFO_UPDATE)
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_BSS_INFO_UPDATE as u32) | CMD_FIELD_WM | CMD_FIELD_WA;

        udebug!("mcu", "set_beacon"; band = band, omac = omac_idx, bcn_len = bcn_len as u32, enable = enable as u8);
        self.mcu_send_uni_cmd(ring, cmd, &data[..data_len], true, seq, irq)
    }

    /// Program a fixed rate table entry
    /// Linux: mt7996/mcu.c:4604-4631 mt7996_mcu_set_fixed_rate_table()
    ///
    /// UNI command MCU_WM_UNI_CMD(FIXED_RATE_TABLE) (0x40), tag=0.
    /// Used for basic rate programming (12 CCK+OFDM rates).
    pub fn mcu_set_fixed_rate_table(&self, ring: &mut TxRing, table_idx: u8, rate_idx: u16,
                                    is_beacon: bool, band: u8, seq: u8, irq: Option<&mut FwIrq>) -> Result<(), i32> {
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
        // spe_idx_sel: beacon → SPE_IXD_SELECT_TXD (0), non-beacon → SPE_IXD_SELECT_BMC_WTBL (1)
        // spe_idx: beacon → 24 + band_idx, non-beacon → 0
        // Linux: mcu.c:4617-4621
        if is_beacon {
            data[12] = 0; // SPE_IXD_SELECT_TXD
            data[13] = 24 + band;
        } else {
            data[12] = 1; // SPE_IXD_SELECT_BMC_WTBL
        }
        // gi = 1
        data[14] = 1;
        // he_ltf = 1
        data[15] = 1;
        // ldpc, txbf, dynamic_bw, _rsv2 = 0

        // MCU_WM_UNI_CMD(FIXED_RATE_TABLE) = __MCU_CMD_FIELD_UNI | 0x40 | __MCU_CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_FIXED_RATE_TABLE as u32) | CMD_FIELD_WM;

        self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, irq)
    }

    /// Background chain control — start/stop offchannel scan
    /// Linux: mt7996/mcu.c:3604-3658 mt7996_mcu_background_chain_ctrl()
    /// Struct: mt7996/mcu.h:122-148 mt7996_mcu_background_chain_ctrl (24 bytes)
    ///
    /// scan_mode: 0=stop, 1=start
    /// ch_band: 0=2.4GHz, 1=5GHz
    pub fn mcu_background_chain_ctrl(
        &self, ring: &mut TxRing,
        band: u8, channel: u8, bw: u8, ch_band: u8,
        scan_mode: u8,
        seq: u8,
    ) -> Result<(), i32> {
        // struct mt7996_mcu_background_chain_ctrl — 24 bytes packed
        // [0..4]   _rsv (uni_header)
        // [4..6]   tag = 0 (le16)
        // [6..8]   len = sizeof(req) - 4 = 20 (le16)
        // [8]      chan (primary channel)
        // [9]      central_chan
        // [10]     bw
        // [11]     tx_stream
        // [12]     rx_stream (antenna mask)
        // [13]     monitor_chan
        // [14]     monitor_central_chan
        // [15]     monitor_bw
        // [16]     monitor_tx_stream
        // [17]     monitor_rx_stream
        // [18]     scan_mode
        // [19]     band_idx (DBDC)
        // [20]     monitor_scan_type
        // [21]     band (0=2.4GHz, 1=5GHz)
        // [22..24] _rsv2
        let mut data = [0u8; 24];

        // tag = 0 — mcu.c:3613
        data[4..6].copy_from_slice(&0u16.to_le_bytes());
        // len = 20 — mcu.c:3614
        data[6..8].copy_from_slice(&20u16.to_le_bytes());
        // monitor_scan_type = 2 (simple rx) — mcu.c:3615
        data[20] = 2;

        if scan_mode == 1 {
            // CH_SWITCH_BACKGROUND_SCAN_START — mcu.c:3625-3635
            data[8] = channel;                // chan — mcu.c:3626
            data[9] = channel;                // central_chan — mcu.c:3627
            data[10] = bw;                    // bw — mcu.c:3628
            data[13] = channel;               // monitor_chan — mcu.c:3629
            data[14] = channel;               // monitor_central_chan — mcu.c:3630-3631
            data[15] = bw;                    // monitor_bw — mcu.c:3632
            data[18] = 1;                     // scan_mode = 1 — mcu.c:3634
            data[19] = band;                  // band_idx — mcu.c:3633
        } else {
            // CH_SWITCH_BACKGROUND_SCAN_STOP — mcu.c:3644-3650
            data[8] = channel;                // chan — mcu.c:3645
            data[9] = channel;                // central_chan — mcu.c:3646
            data[10] = bw;                    // bw — mcu.c:3647
            data[11] = 2;                     // tx_stream = hweight8(antenna_mask=3) = 2 — mcu.c:3648
            data[12] = 3;                     // rx_stream = antenna_mask = 0x3 (2T2R) — mcu.c:3649
            data[18] = 0;                     // scan_mode = 0
        }

        // band — mcu.c:3654
        data[21] = ch_band;

        // MCU_WM_UNI_CMD(OFFCH_SCAN_CTRL) — mcu.c:3656
        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_OFFCH_SCAN_CTRL as u32) | CMD_FIELD_WM;

        udebug!("mcu", "background_chain_ctrl"; scan_mode = scan_mode, channel = channel);
        self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)  // wait=false — mcu.c:3657
    }

    /// Query all STA info — periodic heartbeat to firmware.
    /// Linux: mt7996/mcu.c:4741-4756 mt7996_mcu_get_all_sta_info()
    /// Called every 500ms from mt7996_mac_work() (HZ/10 interval × 5 ticks).
    /// MCU_WM_UNI_CMD(ALL_STA_INFO), wait=false (fire and forget).
    pub fn mcu_get_all_sta_info(&self, ring: &mut TxRing, tag: u16, seq: u8) -> Result<(), i32> {
        let mut data = [0u8; 8];
        // _rsv[4] = 0 (uni_header)
        data[4..6].copy_from_slice(&tag.to_le_bytes());
        data[6..8].copy_from_slice(&4u16.to_le_bytes()); // len = sizeof(req) - 4 = 4

        let cmd = CMD_FIELD_UNI | (MCU_UNI_CMD_ALL_STA_INFO as u32) | CMD_FIELD_WM;
        self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)
    }

    /// Read and clear channel MIB counters (TX time, RX time, OBSS airtime, non-WiFi time).
    /// Linux: mt7996/mcu.c:3948-4025 mt7996_mcu_get_chan_mib_info()
    /// Called every 100ms from mt7996_mac_work() via mt76_update_survey().
    ///
    /// Without periodic clearing, firmware CCA counters accumulate indefinitely,
    /// causing the MAC to assess the channel as permanently busy and stop TX
    /// after ~42 seconds.
    ///
    /// MCU_WM_UNI_CMD_QUERY(GET_MIB_INFO) — Linux uses wait=true to read
    /// response, but we use wait=false since we only need the side effect
    /// of clearing the counters. The response arrives on the MCU WM RX ring
    /// and is silently recycled.
    pub fn mcu_get_chan_mib_info(&self, ring: &mut TxRing, band: u8, seq: u8) -> Result<(), i32> {
        // Request layout (36 bytes):
        //   hdr:  { band: u8, __rsv: [u8; 3] }                      = 4 bytes
        //   data: [{ tag: le16, len: le16, offs: le32 }; 4]          = 32 bytes
        // Source: mt7996/mcu.c:3957-3969
        let mut data = [0u8; 36];

        // hdr.band — mcu.c:3968
        data[0] = band;
        // data[1..4] = 0 (__rsv)

        // Four MIB entries — mcu.c:3983-3987
        // Each: tag=UNI_CMD_MIB_DATA(0), len=8, offs=UNI_MIB_*
        let offsets: [u32; 4] = [
            UNI_MIB_TX_TIME,       // mcu.h:280
            UNI_MIB_RX_TIME,       // mcu.h:281
            UNI_MIB_OBSS_AIRTIME,  // mcu.h:278
            UNI_MIB_NON_WIFI_TIME, // mcu.h:279
        ];

        for i in 0..4 {
            let base = 4 + i * 8; // offset into data[]
            data[base..base + 2].copy_from_slice(&UNI_CMD_MIB_DATA.to_le_bytes()); // tag
            data[base + 2..base + 4].copy_from_slice(&8u16.to_le_bytes());          // len = sizeof(data[i]) = 8
            data[base + 4..base + 8].copy_from_slice(&offsets[i].to_le_bytes());    // offs
        }

        // MCU_WM_UNI_CMD_QUERY(GET_MIB_INFO) — mcu.c:3989
        // QUERY variant: CMD_FIELD_UNI | CMD_FIELD_QUERY | cmd_id | CMD_FIELD_WM
        let cmd = CMD_FIELD_UNI | CMD_FIELD_QUERY | (MCU_UNI_CMD_GET_MIB_INFO as u32) | CMD_FIELD_WM;

        // wait=false: we don't need the response data, just the clearing side-effect.
        // The response will arrive on MCU WM RX and be recycled by rx_process_mcu().
        self.mcu_send_uni_cmd(ring, cmd, &data, false, seq, None)
    }
}
